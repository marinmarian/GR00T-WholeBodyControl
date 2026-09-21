"""PICO hand tracking -> Inspire hand inputs (g1-vr-teleop #35).

The tracked hand produces the same (menu, left_trigger, right_trigger, left_grip, right_grip)
tuple the controllers produce (trigger = mean finger curl, grip = thumb closure), which keeps the
hand IK for the ZMQ message and the recorded hand-state space untouched. Since phase 2 the
Inspire bridge additionally gets PER-FINGER closures through ``finger_targets()``
([little, ring, middle, index, thumb_bend, thumb_rot]), so each finger moves on its own;
trigger/grip stay as the fallback mapping when a side is on its controller.

Curl is computed from bone angles (flexion summed along the finger), which is scale-free and
frame-free: no Unity->robot transform, no per-user calibration. Joint layout is OpenXR
XR_HAND_JOINT (26 joints, PICO's ``HandJointLocations``): palm, wrist, thumb x4, then index,
middle, ring, little x5 each (metacarpal, proximal, intermediate, distal, tip).

Menu button, A/B/X/Y and the sticks keep coming from the controllers (mode toggle, e-stop and
recording gestures live there), so at least one controller should stay within reach.
"""
from __future__ import annotations

import math
import os
import time

import numpy as np

# OpenXR hand joint indices
PALM, WRIST = 0, 1
THUMB = (2, 3, 4, 5)                  # metacarpal, proximal, distal, tip
INDEX = (6, 7, 8, 9, 10)              # metacarpal, proximal, intermediate, distal, tip
MIDDLE = (11, 12, 13, 14, 15)
RING = (16, 17, 18, 19, 20)
LITTLE = (21, 22, 23, 24, 25)
FINGERS = {"index": INDEX, "middle": MIDDLE, "ring": RING, "little": LITTLE}
NUM_JOINTS = 26

# Flexion sums (degrees) that map to fully open (0) / fully closed (1). Measured on the PICO 4 Ultra
# 2026-09-21 (tools/hand_tracking_probe.py): relaxed hand 26-53 deg per finger, fist 142-190.
FINGER_OPEN_DEG = 55.0
FINGER_CLOSED_DEG = 150.0
# The thumb barely flexes in a fist on this tracker (32-35 deg vs 15-31 relaxed): it wraps by
# opposition. So the thumb uses the distance thumb tip -> little-finger proximal joint, divided by
# the palm length (wrist -> middle proximal) to stay scale-free. Probe 2026-09-21: open hand 1.4-1.6,
# natural fist 0.8-1.0 (thumb over the fingers), thumb folded across the palm ~0.55.
THUMB_OPEN_RATIO = 1.35
THUMB_CLOSED_RATIO = 0.85
THUMB_OPEN_DEG = 25.0        # flexion-sum variant, kept for the probe / tuning only
THUMB_CLOSED_DEG = 100.0
# Thumb base rotation (Inspire DOF 5): angle between the thumb proximal->distal bone and the palm
# normal. Thumb flat in the finger plane ~80-90 deg -> closure 0 (Inspire 1000); thumb opposed,
# pointing out of the palm, ~20-40 deg -> closure 1 (Inspire 0 = pinch preset). Tune with the probe
# (tra=). HAND_TRACKING_THUMB_ROT=0 leaves the rotation at rest.
THUMB_ROT_FLAT_DEG = 75.0
THUMB_ROT_OPPOSED_DEG = 35.0
MIN_BONE_M = 0.003                    # shorter bones = untracked / zero data -> invalid
SNAP_HIGH = 0.90                      # curl above this -> 1.0 (bridge FULL_PUSH is 0.95: no flapping)
SNAP_LOW = 0.05                       # below this -> 0.0


def _angle_deg(u: np.ndarray, v: np.ndarray) -> float:
    nu, nv = np.linalg.norm(u), np.linalg.norm(v)
    if nu < MIN_BONE_M or nv < MIN_BONE_M:
        raise ValueError("degenerate bone")
    c = float(np.dot(u, v) / (nu * nv))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def flexion_sum_deg(positions: np.ndarray, chain) -> float:
    """Sum of the angles between consecutive bones of one finger chain (0 = straight)."""
    p = np.asarray(positions, dtype=np.float64)
    bones = [p[chain[i + 1]] - p[chain[i]] for i in range(len(chain) - 1)]
    return sum(_angle_deg(bones[i], bones[i + 1]) for i in range(len(bones) - 1))


def thumb_opposition_ratio(positions: np.ndarray) -> float:
    """Thumb tip to little proximal distance over palm length; small = thumb across the palm."""
    p = np.asarray(positions, dtype=np.float64)
    palm = np.linalg.norm(p[MIDDLE[1]] - p[WRIST])
    if palm < MIN_BONE_M:
        raise ValueError("degenerate palm")
    return float(np.linalg.norm(p[THUMB[3]] - p[LITTLE[1]]) / palm)


def palm_normal(positions: np.ndarray) -> np.ndarray:
    """Unit normal of the palm plane from wrist, index metacarpal and little metacarpal."""
    p = np.asarray(positions, dtype=np.float64)
    u, v = p[INDEX[0]] - p[WRIST], p[LITTLE[0]] - p[WRIST]
    n = np.cross(u, v)
    ln = np.linalg.norm(n)
    if ln < MIN_BONE_M * MIN_BONE_M:
        raise ValueError("degenerate palm")
    return n / ln


def thumb_rotation_angle_deg(positions: np.ndarray) -> float:
    """Angle (0..90 deg) between the thumb proximal->distal bone and the palm normal."""
    p = np.asarray(positions, dtype=np.float64)
    bone = p[THUMB[2]] - p[THUMB[1]]
    lb = np.linalg.norm(bone)
    if lb < MIN_BONE_M:
        raise ValueError("degenerate thumb")
    c = abs(float(np.dot(bone / lb, palm_normal(p))))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def _normalize(x: float, open_deg: float, closed_deg: float) -> float:
    c = (x - open_deg) / (closed_deg - open_deg)
    c = min(1.0, max(0.0, c))
    if c >= SNAP_HIGH:
        return 1.0
    if c <= SNAP_LOW:
        return 0.0
    return c


def hand_curls(joints: np.ndarray):
    """Per-finger curls in [0, 1] from a (26, >=3) joint array; None if the hand is not tracked.

    Returns dict with keys index, middle, ring, little, thumb, plus 'fingers' (mean of the four).
    """
    j = np.asarray(joints, dtype=np.float64)
    if j.ndim != 2 or j.shape[0] != NUM_JOINTS or j.shape[1] < 3:
        return None
    pos = j[:, :3]
    if not np.all(np.isfinite(pos)) or np.allclose(pos, 0.0):
        return None
    try:
        curls = {name: _normalize(flexion_sum_deg(pos, chain), FINGER_OPEN_DEG, FINGER_CLOSED_DEG)
                 for name, chain in FINGERS.items()}
        curls["thumb"] = _normalize(thumb_opposition_ratio(pos), THUMB_OPEN_RATIO, THUMB_CLOSED_RATIO)
        curls["thumb_rot"] = _normalize(thumb_rotation_angle_deg(pos), THUMB_ROT_FLAT_DEG, THUMB_ROT_OPPOSED_DEG)
    except ValueError:
        return None
    curls["fingers"] = float(np.mean([curls[n] for n in FINGERS]))
    return curls


def raw_flexion_sums(joints: np.ndarray):
    """Unnormalised flexion sums (deg) per finger, for the probe / tuning. None if degenerate."""
    j = np.asarray(joints, dtype=np.float64)
    if j.ndim != 2 or j.shape[0] != NUM_JOINTS:
        return None
    try:
        out = {name: flexion_sum_deg(j[:, :3], chain) for name, chain in FINGERS.items()}
        out["thumb"] = flexion_sum_deg(j[:, :3], THUMB)
        out["thumb_ratio"] = thumb_opposition_ratio(j[:, :3])
        out["thumb_rot_deg"] = thumb_rotation_angle_deg(j[:, :3])
        return out
    except ValueError:
        return None


class HandTrackingInputs:
    """Callable returning (menu, left_trigger, right_trigger, left_grip, right_grip).

    Per side: tracked hand (active flag set and geometry valid) -> smoothed virtual trigger/grip;
    tracking just lost -> hold the last value for ``hold_s`` (a controller lying on the table
    reads 0 and would open the hand at once, dropping the object); after that -> the controller
    value for that side, so a controller held in one hand keeps working normally.

    ``read_hand(side) -> (joints26x7, active)`` and ``controller_inputs() -> 5-tuple`` are
    injected so the class is testable without the headset; ``from_xrt`` wires the SDK.
    """

    def __init__(self, controller_inputs, read_hand, alpha=0.4, hold_s=0.5, debug=False,
                 clock=time.monotonic):
        self._controller_inputs = controller_inputs
        self._read_hand = read_hand
        self._alpha = float(alpha)
        self._hold_s = float(hold_s)
        self._debug = debug
        self._clock = clock
        self._ema = {"left": None, "right": None}       # (trigger, grip)
        # per-DOF closures in Inspire order [little, ring, middle, index, thumb_bend, thumb_rot], smoothed
        self._ema_dof = {"left": None, "right": None}
        self._thumb_rot = os.environ.get("HAND_TRACKING_THUMB_ROT", "1") not in ("", "0")
        self._last_valid = {"left": None, "right": None}
        self._source = {"left": "controller", "right": "controller"}
        self._diag = {"left": "", "right": ""}
        self._last_print = 0.0
        # HAND_TRACKING_IGNORE_ACTIVE=1: use the joints whenever the geometry is valid, even if the
        # headset reports isActive=0 (some app versions report 0 = "low quality" while tracking).
        self._ignore_active = os.environ.get("HAND_TRACKING_IGNORE_ACTIVE", "") not in ("", "0")

    @classmethod
    def from_xrt(cls, controller_inputs, **kw):
        import xrobotoolkit_sdk as xrt  # lazy: tests and the probe's --synthetic path run without it

        def read_hand(side):
            if side == "left":
                return xrt.get_left_hand_tracking_state(), xrt.get_left_hand_is_active()
            return xrt.get_right_hand_tracking_state(), xrt.get_right_hand_is_active()

        return cls(controller_inputs, read_hand, **kw)

    def _side(self, side, ctrl_trigger, ctrl_grip, now):
        try:
            joints, active = self._read_hand(side)
            j = np.asarray(joints, dtype=np.float64)
            has_data = j.shape[0] == NUM_JOINTS and not np.allclose(j[:, :3], 0.0)
            use = has_data and (int(active) == 1 or self._ignore_active)
            curls = hand_curls(j) if use else None
            if self._debug:
                sums = raw_flexion_sums(j) if has_data else None
                self._diag[side] = (f"active={int(active)} data={'y' if has_data else 'n'}"
                                    + (" sums " + "/".join(f"{sums[k]:.0f}" for k in ("index", "middle", "ring", "little")) + f" thr {sums['thumb_ratio']:.2f} tra {sums['thumb_rot_deg']:.0f}" if sums else ""))
        except Exception as e:  # noqa: BLE001 - never let hand tracking take the teleop loop down
            curls = None
            if self._debug:
                self._diag[side] = f"read error {e}"
        if curls is not None:
            dof = [curls["little"], curls["ring"], curls["middle"], curls["index"], curls["thumb"], curls["thumb_rot"]]
            prev_dof = self._ema_dof[side]
            if prev_dof is None:
                ema_dof = dof
            else:
                ema_dof = [prev_dof[i] + self._alpha * (dof[i] - prev_dof[i]) for i in range(6)]
            self._ema_dof[side] = [1.0 if v >= SNAP_HIGH else (0.0 if v <= SNAP_LOW else v) for v in ema_dof]
            target = (curls["fingers"], curls["thumb"])
            prev = self._ema[side]
            if prev is None:
                ema = target
            else:
                ema = tuple(prev[i] + self._alpha * (target[i] - prev[i]) for i in range(2))
            # keep the snap semantics after smoothing so FULL_PUSH is stable
            ema = tuple(1.0 if v >= SNAP_HIGH else (0.0 if v <= SNAP_LOW else v) for v in ema)
            self._ema[side] = ema
            self._last_valid[side] = now
            self._source[side] = "hand"
            return ema
        if self._last_valid[side] is not None and now - self._last_valid[side] < self._hold_s:
            self._source[side] = "hold"
            return self._ema[side]
        self._ema[side] = None
        self._ema_dof[side] = None
        self._source[side] = "controller"
        return (float(ctrl_trigger), float(ctrl_grip))

    def finger_targets(self):
        """Per-DOF closures for the Inspire bridge: {"left": [little, ring, middle, index,
        thumb_bend, thumb_rot] | None, "right": ...}. None while that side is on its controller
        (the bridge then uses the trigger/grip mapping). thumb_rot is None (= rest) when
        HAND_TRACKING_THUMB_ROT=0. Uses the values of the last __call__."""
        out = {}
        for side in ("left", "right"):
            d = self._ema_dof[side]
            if d is None or self._source[side] == "controller":
                out[side] = None
            else:
                out[side] = list(d[:5]) + [d[5] if self._thumb_rot else None]
        return out

    def __call__(self):
        menu, lt, rt, lg, rg = self._controller_inputs()
        now = self._clock()
        lt2, lg2 = self._side("left", lt, lg, now)
        rt2, rg2 = self._side("right", rt, rg, now)
        if self._debug and now - self._last_print > 1.0:
            self._last_print = now
            print(f"[HandTracking] L {self._source['left']:10s} trig {lt2:.2f} grip {lg2:.2f} ({self._diag['left']}) | "
                  f"R {self._source['right']:10s} trig {rt2:.2f} grip {rg2:.2f} ({self._diag['right']})", flush=True)
        return menu, lt2, rt2, lg2, rg2

    @property
    def sources(self):
        return dict(self._source)


def debug_enabled() -> bool:
    return os.environ.get("HAND_TRACKING_DEBUG", "") not in ("", "0")

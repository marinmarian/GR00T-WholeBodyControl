"""PICO hand tracking -> Inspire hand inputs (g1-vr-teleop #35).

Phase 1: the tracked hand produces the same (menu, left_trigger, right_trigger, left_grip,
right_grip) tuple the controllers produce, so the Inspire bridge, its force/stall logic, the
dump publisher and the recorded hand-state space stay untouched:

    trigger = curl of the four fingers (mean)      -> four-finger close in the bridge
    grip    = curl of the thumb                     -> thumb bend in the bridge

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

# Flexion sums (degrees) that map to fully open (0) / fully closed (1). A relaxed hand sums to
# roughly 30-60 deg per finger, a fist to 200+; tune with tools/hand_tracking_probe.py.
FINGER_OPEN_DEG = 40.0
FINGER_CLOSED_DEG = 190.0
THUMB_OPEN_DEG = 25.0
THUMB_CLOSED_DEG = 100.0
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
        curls["thumb"] = _normalize(flexion_sum_deg(pos, THUMB), THUMB_OPEN_DEG, THUMB_CLOSED_DEG)
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
        self._last_valid = {"left": None, "right": None}
        self._source = {"left": "controller", "right": "controller"}
        self._last_print = 0.0

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
            curls = hand_curls(np.asarray(joints)) if int(active) == 1 else None
        except Exception:  # noqa: BLE001 - never let hand tracking take the teleop loop down
            curls = None
        if curls is not None:
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
        self._source[side] = "controller"
        return (float(ctrl_trigger), float(ctrl_grip))

    def __call__(self):
        menu, lt, rt, lg, rg = self._controller_inputs()
        now = self._clock()
        lt2, lg2 = self._side("left", lt, lg, now)
        rt2, rg2 = self._side("right", rt, rg, now)
        if self._debug and now - self._last_print > 1.0:
            self._last_print = now
            print(f"[HandTracking] L {self._source['left']:10s} trig {lt2:.2f} grip {lg2:.2f} | "
                  f"R {self._source['right']:10s} trig {rt2:.2f} grip {rg2:.2f}", flush=True)
        return menu, lt2, rt2, lg2, rg2

    @property
    def sources(self):
        return dict(self._source)


def debug_enabled() -> bool:
    return os.environ.get("HAND_TRACKING_DEBUG", "") not in ("", "0")

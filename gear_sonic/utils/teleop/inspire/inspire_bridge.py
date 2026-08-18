"""
InspireBridge — drive Inspire RH56DFTP hands from teleop controller inputs.

Runs a background daemon thread at ~90 Hz that polls a caller-supplied input
function (latest PICO trigger/grip each tick; no interpolation) and writes
finger angles to both hands over Modbus TCP (via the vendored
InspireHandModbusTCP driver). Completely decoupled from the body teleop
pipeline: the C++ deploy's (inert) Dex3 output is untouched, and this works
in every stream mode (PLANNER, VR_3PT, POSE) as well as with either input
source (PICO/xrt or Quest/isaac-teleop) since both expose the same
trigger/squeeze controller values.

Mapping (mode="trigger"):
  trigger (0..1)  -> four-finger curl  [little, ring, middle, index]
  squeeze (0..1)  -> thumb bend
  thumb_rotation  -> parked at THUMB_ROT_REST (0 = opposed / orthogonal to the
                     finger plane; 1000 = in-plane, same surface as the fingers)
  angle = 1000 - value * 1000   (Inspire: 1000 = open, 0 = closed)

Safety: on connect, FORCE_SET is rest FORCE_ACT plus FORCE_LIMIT_G extra
grams (cruise cap; firmware FORCE_ACT is biased at open). On contact/stall
the close command is left as-is so firmware holds torque into the object —
no backoff toward open. Free fingers keep tracking the trigger's desired
angle (not wrap-to-0); contacted siblings must not freeze them. Full
trigger/grip (kickdown) raises FORCE_SET to the firmware maximum for those
DOFs; the process does not stop.
"""

import atexit
import threading
import time

from .inspire_dump import (
    build_inspire_snapshots,
    empty_inspire_snapshots,
    setup_last_run_log,
    write_last_run_sample,
)
from .inspire_hand_modbus import (
    InspireHandModbusTCP,
    DEFAULT_LEFT_IP,
    DEFAULT_RIGHT_IP,
    Inspire_Num_Motors,
    DOF_TACTILE_REGION,
)

OPEN = 1000
# Base (mount) thumb rotation at rest. 1000 lays the thumb in the finger plane;
# 0 matches the pinch preset — thumb opposed, orthogonal to the palm.
THUMB_ROT_REST = 0
# Hands control loop. Latest get_inputs() each tick; no interpolation.
HANDS_RATE_HZ = 90.0
# EMA smoothing factor per control tick (higher = snappier, lower = smoother).
EMA_ALPHA = 0.5
# Skip Modbus writes when no DOF changed by more than this many angle units.
WRITE_DEADBAND = 3
RECONNECT_PERIOD_S = 2.0
# Tip-pad reads are 5 Modbus transactions / hand. Last-run t_mono p95 ≈ 22 ms
# overruns a 90 Hz slot (~11.1 ms); command + FORCE_ACT stay every tick.
TACTILE_PERIOD_TICKS = 2
# Extra grams above the per-DOF rest FORCE_ACT baseline. DFTP FORCE_ACT is
# often hundreds of grams while the fingers are fully open; an absolute 400 g
# cap then blocks closing (firmware) and false-triggers stall yield.
FORCE_LIMIT_G = 400
# RH56 FORCE_SET register range is 0-3000 g (inspire_hand_modbus.write_force_limits).
# FORCE_ACT can read higher (~4000); kickdown uses this firmware FORCE_SET max.
FORCE_SET_MAX_G = 3000
# Trigger/grip at or above this: no software force cap for that DOF group.
FULL_PUSH = 0.95
# Stall detect (logging only): closing DOF, >STALL_ERR behind, and FORCE_ACT
# rose this many grams above rest baseline (OR tactile contact). Does not
# retract toward open — user close cmd stays; FORCE_SET holds the push.
FORCE_YIELD_G = 200
STALL_ERR = 80
# Enveloping: a stalled finger still this many units more open than its
# stalled siblings is wrapping, not contacted (coupled FORCE_ACT).
WRAP_OPEN_MARGIN = 80
# Ticks of no closing at FORCE_SET max before a wrapping finger is contacted.
WRAP_HIT_TICKS = 6
FINGER_DOFS = 4
# DFTP fingertip taxels (0-4095). Rise above resting baseline counts as contact.
# Firmware FORCE_SET cannot see these; stall detect ORs them with FORCE_ACT.
TACTILE_MARGIN = 40
TACTILE_BASELINE_SAMPLES = 3


class InspireBridge:
    """Background thread mapping controller inputs to Inspire hand angles.

    Args:
        get_inputs: callable returning the 5-tuple from
            ``get_controller_inputs(reader)``:
            (menu_button, left_trigger, right_trigger, left_squeeze, right_squeeze).
            Must be thread-safe (both the xrt globals and IsaacTeleopReader
            snapshot accessors are).
        mode: "trigger" (supported) or "handtracking" (not implemented yet).
        left_ip / right_ip: hand Modbus TCP addresses.
        rate_hz: write loop frequency (default HANDS_RATE_HZ).
        speed: Inspire per-DOF speed setting (0-1000) written once on connect.
        dump_publisher: optional object with ``publish(mechanical, tactile)``;
            called after every tick so the exporter can record without opening
            a second Modbus TCP session to the hand.
    """

    def __init__(self, get_inputs, mode="trigger",
                 left_ip=DEFAULT_LEFT_IP, right_ip=DEFAULT_RIGHT_IP,
                 rate_hz=HANDS_RATE_HZ, speed=1000, sides=None, dump_publisher=None):
        if mode == "handtracking":
            raise NotImplementedError(
                "inspire-hands mode 'handtracking' requires per-finger data from "
                "the headset reader, which is not wired up yet — use 'trigger'."
            )
        if mode != "trigger":
            raise ValueError(f"unknown inspire-hands mode '{mode}'")
        if sides is None:
            sides = ("left", "right")
        sides = tuple(sides)
        if not sides:
            raise ValueError("sides must be non-empty")
        unknown = [s for s in sides if s not in ("left", "right")]
        if unknown:
            raise ValueError(f"unknown hand sides {unknown}")
        self._get_inputs = get_inputs
        self._rate_hz = float(rate_hz)
        self._dt = 1.0 / self._rate_hz
        self._speed = int(speed)
        self._loop_tick = 0
        ips = {"left": left_ip, "right": right_ip}
        labels = {"left": "InspireL", "right": "InspireR"}
        self._hands = {
            side: InspireHandModbusTCP(ips[side], label=labels[side])
            for side in sides
        }
        rest = [float(v) for v in self._open_pose()]
        self._smoothed = {side: list(rest) for side in self._hands}
        self._last_written = {side: None for side in self._hands}
        self._last_reconnect = {side: 0.0 for side in self._hands}
        self._last_stall_log = {side: 0.0 for side in self._hands}
        self._tactile_base = {side: {} for side in self._hands}
        self._last_peaks = {side: None for side in self._hands}
        self._last_tactile = {
            side: [False] * Inspire_Num_Motors for side in self._hands
        }
        self._force_base = {side: [0] * Inspire_Num_Motors for side in self._hands}
        self._last_force_set = {side: None for side in self._hands}
        self._wrap_boost = {side: [False] * Inspire_Num_Motors for side in self._hands}
        self._wrap_stuck_ticks = {side: [0] * Inspire_Num_Motors for side in self._hands}
        self._prev_actual = {side: None for side in self._hands}
        self._stop_evt = threading.Event()
        self._thread = None
        self._dump_publisher = dump_publisher

    # ------------------------------------------------------------------ API

    def start(self):
        try:
            path = setup_last_run_log()
            print(f"[InspireBridge] last-run log: {path}", flush=True)
        except Exception as e:
            print(f"[InspireBridge] last-run log setup failed (continuing): {e}",
                  flush=True)
        for side, hand in self._hands.items():
            if hand.connect():
                self._prepare_hand(side, hand)
            else:
                print(f"[InspireBridge] {side} hand not reachable at "
                      f"{hand.ip}:{hand.port} — will keep retrying")
        self._thread = threading.Thread(target=self._run, name="InspireBridge",
                                        daemon=True)
        self._thread.start()
        atexit.register(self.stop)
        print("[InspireBridge] started (trigger mode: trigger=fingers, "
              f"squeeze=thumb, thumb_rot rest={THUMB_ROT_REST}, "
              f"rate={self._rate_hz:g}Hz, "
              f"force_limit={FORCE_LIMIT_G}g)")

    def stop(self):
        if self._stop_evt.is_set():
            return
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        for hand in self._hands.values():
            try:
                if hand.connected:
                    hand.write_angles(self._open_pose())
                hand.close()
            except Exception:
                pass
        print("[InspireBridge] stopped (hands opened)")

    # ------------------------------------------------------------ internals

    def _prepare_hand(self, side, hand):
        hand.write_speed([self._speed] * Inspire_Num_Motors)
        force_base = [0] * Inspire_Num_Motors
        for _ in range(TACTILE_BASELINE_SAMPLES):
            forces = hand.read_forces()
            if forces is None:
                continue
            for i in range(min(Inspire_Num_Motors, len(forces))):
                force_base[i] = max(force_base[i], int(forces[i]))
        self._force_base[side] = force_base
        limits = self._force_set_from_baseline(force_base)
        if not hand.write_force_limits(limits):
            print(f"[InspireBridge] {hand.label}: FORCE_SET {limits} failed",
                  flush=True)
        else:
            self._last_force_set[side] = limits
        base = {}
        for _ in range(TACTILE_BASELINE_SAMPLES):
            peaks = hand.read_tactile_tip_peaks()
            if peaks is None:
                continue
            for region, val in peaks.items():
                base[region] = max(base.get(region, 0), val)
        self._tactile_base[side] = base
        pose = self._open_pose()
        if hand.write_angles(pose):
            print(f"[InspireBridge] {hand.label}: rest pose {pose} "
                  f"force_set={limits} force_base={force_base} tactile_base={base}",
                  flush=True)
        else:
            print(f"[InspireBridge] {hand.label}: rest-pose write failed",
                  flush=True)

    @staticmethod
    def _open_pose():
        """Fingers + thumb bend open; thumb rotation at the orthogonal rest."""
        return [OPEN, OPEN, OPEN, OPEN, OPEN, THUMB_ROT_REST]

    @staticmethod
    def _force_set_from_baseline(baseline, extra_g=FORCE_LIMIT_G):
        """Cruise FORCE_SET: rest FORCE_ACT plus extra_g, clipped to firmware max."""
        n = Inspire_Num_Motors
        base = list(baseline) if baseline is not None else []
        base = base[:n] + [0] * max(0, n - len(base))
        return [int(min(FORCE_SET_MAX_G, max(0, int(b) + extra_g))) for b in base]

    @staticmethod
    def _force_set_for_inputs(baseline, trigger, squeeze, contacts=None,
                              actual=None, desired=None):
        """Cruise cap, or firmware max on full trigger (fingers) / grip (thumb_bend).

        Partial trigger: if some fingers have contacted, free fingers that are
        still short of the trigger desired angle get FORCE_SET_MAX so coupled
        FORCE_ACT cannot freeze them. Contacted fingers stay at cruise.
        Closing in air (no contact) stays cruise. Cmd never wraps past desired.
        """
        limits = InspireBridge._force_set_from_baseline(baseline)
        if float(trigger) >= FULL_PUSH:
            for i in range(FINGER_DOFS):
                limits[i] = FORCE_SET_MAX_G
        elif contacts is not None and any(
                i < len(contacts) and contacts[i] for i in range(FINGER_DOFS)):
            for i in range(FINGER_DOFS):
                if i < len(contacts) and contacts[i]:
                    continue
                short = (
                    actual is not None and desired is not None
                    and i < len(actual) and i < len(desired)
                    and actual[i] - desired[i] > STALL_ERR
                )
                if short:
                    limits[i] = FORCE_SET_MAX_G
        if float(squeeze) >= FULL_PUSH:
            limits[4] = FORCE_SET_MAX_G
        return limits

    @staticmethod
    def _yield_on_stall(cmd, actual, forces, tactile=None, force_baseline=None):
        """Detect closing contact/stall without retracting toward open.

        Contact is a FORCE_ACT *rise* above *this DOF's* rest baseline OR DFTP
        tactile tip contact — never Fmax / sibling force. A finger that is
        clearly more open than other stalled siblings stays free (coupled
        FORCE_ACT). Opening commands are never blocked. On stall the user
        close command is left as-is (the trigger desired angle — no wrap-to-0
        and no backoff). Missing readings are a no-op so teleop continues.
        Returns (cmd unchanged, stalled_dof_indices).
        """
        if actual is None:
            return list(cmd), []
        if forces is None and tactile is None:
            return list(cmd), []
        n = min(len(cmd), len(actual), Inspire_Num_Motors)
        base = list(force_baseline) if force_baseline is not None else []
        base = base[:n] + [0] * max(0, n - len(base))
        out = list(cmd)
        stalled = []
        for i in range(n):
            if out[i] >= actual[i]:
                continue
            if actual[i] - out[i] < STALL_ERR:
                continue
            force_hit = (
                forces is not None and i < len(forces)
                and (forces[i] - base[i]) >= FORCE_YIELD_G
            )
            tac_hit = tactile is not None and i < len(tactile) and tactile[i]
            if not force_hit and not tac_hit:
                continue
            stalled.append(i)
        stalled = InspireBridge._uncouple_open_fingers(stalled, actual)
        return out, stalled

    @staticmethod
    def _uncouple_open_fingers(stalled, actual, margin=WRAP_OPEN_MARGIN):
        """Drop fingers that are still wrapping vs braced siblings.

        Coupled FORCE_ACT can mark all four stalled; the still-open finger
        must keep tracking the trigger desired angle.
        """
        if actual is None:
            return list(stalled)
        finger_hit = [i for i in stalled if i < FINGER_DOFS]
        if len(finger_hit) < 2:
            return list(stalled)
        drop = set()
        for i in finger_hit:
            others = [actual[j] for j in finger_hit if j != i and j < len(actual)]
            if others and i < len(actual) and actual[i] > max(others) + margin:
                drop.add(i)
        return [i for i in stalled if i not in drop]

    @staticmethod
    def _latch_wrap_contacts(
        contacts, actual, desired, tactile=None,
        prev_boost=None, prev_actual=None, last_force_set=None,
        stuck_ticks=None,
    ):
        """Keep a free finger wrapping to desired until it arrives or hits.

        Returns (contacts, boost, stuck_ticks) as length-6 lists.
        """
        n = Inspire_Num_Motors
        out = list(contacts) if contacts is not None else [False] * n
        out = out[:n] + [False] * max(0, n - len(out))
        boost = list(prev_boost) if prev_boost is not None else [False] * n
        boost = boost[:n] + [False] * max(0, n - len(boost))
        stuck = list(stuck_ticks) if stuck_ticks is not None else [0] * n
        stuck = stuck[:n] + [0] * max(0, n - len(stuck))
        new_boost = [False] * n
        new_stuck = [0] * n
        for i in range(FINGER_DOFS):
            short = (
                actual is not None and desired is not None
                and i < len(actual) and i < len(desired)
                and actual[i] - desired[i] > STALL_ERR
            )
            tac = tactile is not None and i < len(tactile) and tactile[i]
            if tac:
                out[i] = True
                continue
            if boost[i] and short:
                progressed = (
                    prev_actual is None
                    or i >= len(prev_actual)
                    or actual[i] < prev_actual[i] - WRITE_DEADBAND
                )
                was_max = (
                    last_force_set is not None
                    and i < len(last_force_set)
                    and last_force_set[i] >= FORCE_SET_MAX_G
                )
                if progressed:
                    out[i] = False
                    new_boost[i] = True
                elif was_max:
                    new_stuck[i] = stuck[i] + 1
                    if new_stuck[i] >= WRAP_HIT_TICKS:
                        out[i] = True
                    else:
                        out[i] = False
                        new_boost[i] = True
                else:
                    out[i] = False
                    new_boost[i] = True
            else:
                new_boost[i] = (not out[i]) and short
        return out, new_boost, new_stuck

    @staticmethod
    def _tactile_this_tick(tick, period=TACTILE_PERIOD_TICKS):
        """True when this control tick should Modbus-read the five tip pads."""
        return (int(tick) % int(period)) == 0

    @staticmethod
    def _tactile_hits_from_peaks(peaks, baseline, margin=TACTILE_MARGIN):
        """Per-DOF bools: fingertip taxel peak rose `margin` above rest."""
        hits = [False] * Inspire_Num_Motors
        if not peaks:
            return hits
        for dof, region in DOF_TACTILE_REGION.items():
            rise = peaks.get(region, 0) - baseline.get(region, 0)
            if rise >= margin:
                hits[dof] = True
        return hits

    @staticmethod
    def _targets_from_triggers(trigger, squeeze):
        """Map (trigger, squeeze) in 0..1 to 6 Inspire angles."""
        t = min(max(float(trigger), 0.0), 1.0)
        s = min(max(float(squeeze), 0.0), 1.0)
        finger = OPEN - t * OPEN
        thumb_bend = OPEN - s * OPEN
        # DOF order: [little, ring, middle, index, thumb_bend, thumb_rotation]
        return [finger, finger, finger, finger, thumb_bend, THUMB_ROT_REST]

    @staticmethod
    def _format_change(dt_s, lt, rt, lg, rg, writes):
        parts = [
            f"[InspireBridge] change dt={dt_s * 1000:6.0f}ms "
            f"lt={lt:.2f} rt={rt:.2f} lg={lg:.2f} rg={rg:.2f}"
        ]
        for side in ("left", "right"):
            w = writes.get(side)
            if not w:
                continue
            parts.append(
                f"{side}: cmd={w['cmd_ms']:.1f}ms fb={w['fb_ms']:.1f}ms err={w['err']}"
            )
            if w.get("stall"):
                parts.append(
                    f"stall={w['stall']} Fmax={w.get('force_max', '?')}"
                )
        return "  ".join(parts)

    def _publish_side(self, side, cmd, actual, forces, peaks, tactile, stalled, valid,
                      lt=0.0, rt=0.0, lg=0.0, rg=0.0, dt_s=None, force_set_g=None):
        t_unix = time.time()
        t_mono = time.monotonic()
        if valid:
            mechanical, tactile_snap = build_inspire_snapshots(
                side=side,
                cmd=cmd,
                actual=actual,
                force_act_g=forces,
                tip_peaks=peaks,
                tactile_hit=tactile,
                stall_dofs=stalled,
                yielded=bool(stalled),
                valid=1,
                t_unix=t_unix,
                t_mono=t_mono,
                force_set_g=force_set_g,
                tactile_valid=1 if peaks is not None else 0,
            )
        else:
            mechanical, tactile_snap = empty_inspire_snapshots(side, t_unix, t_mono)
        try:
            write_last_run_sample(
                mechanical, lt=lt, rt=rt, lg=lg, rg=rg, dt_s=dt_s,
            )
        except Exception as e:
            print(f"[InspireBridge] last-run log failed (continuing): {e}",
                  flush=True)
        if self._dump_publisher is None:
            return
        try:
            self._dump_publisher.publish(mechanical, tactile_snap)
        except Exception as e:
            print(f"[InspireBridge] dump publish failed (continuing): {e}", flush=True)

    def _write_side(self, side, targets, readback=False,
                    lt=0.0, rt=0.0, lg=0.0, rg=0.0, dt_s=None,
                    read_tactile=True):
        hand = self._hands[side]
        if not hand.connected:
            now = time.time()
            if now - self._last_reconnect[side] >= RECONNECT_PERIOD_S:
                self._last_reconnect[side] = now
                if hand.reconnect():
                    self._prepare_hand(side, hand)
                    self._last_written[side] = None
            if not hand.connected:
                self._publish_side(
                    side,
                    cmd=None,
                    actual=None,
                    forces=None,
                    peaks=None,
                    tactile=None,
                    stalled=[],
                    valid=0,
                    lt=lt, rt=rt, lg=lg, rg=rg, dt_s=dt_s,
                )
                return None
        sm = self._smoothed[side]
        for i in range(Inspire_Num_Motors):
            sm[i] += EMA_ALPHA * (targets[i] - sm[i])
        cmd = [int(min(max(v, 0), OPEN)) for v in sm]
        act = hand.read_angles()
        forces = hand.read_forces()
        if read_tactile:
            peaks = hand.read_tactile_tip_peaks()
            tactile = self._tactile_hits_from_peaks(
                peaks or {}, self._tactile_base[side])
            self._last_peaks[side] = peaks
            self._last_tactile[side] = list(tactile)
        else:
            peaks = self._last_peaks[side]
            tactile = list(self._last_tactile[side])
        cmd, stalled = self._yield_on_stall(
            cmd, act, forces, tactile, self._force_base.get(side))
        contacts = [False] * Inspire_Num_Motors
        for i in stalled:
            if 0 <= i < Inspire_Num_Motors:
                contacts[i] = True
        contacts, self._wrap_boost[side], self._wrap_stuck_ticks[side] = (
            self._latch_wrap_contacts(
                contacts, act, cmd, tactile,
                prev_boost=self._wrap_boost[side],
                prev_actual=self._prev_actual[side],
                last_force_set=self._last_force_set[side],
                stuck_ticks=self._wrap_stuck_ticks[side],
            )
        )
        stalled = [i for i, hit in enumerate(contacts) if hit]
        trigger, squeeze = (lt, lg) if side == "left" else (rt, rg)
        force_set = self._force_set_for_inputs(
            self._force_base.get(side), trigger, squeeze,
            contacts=contacts, actual=act, desired=cmd)
        if act is not None:
            self._prev_actual[side] = list(act)
        if self._last_force_set[side] != force_set:
            if hand.write_force_limits(force_set):
                self._last_force_set[side] = force_set
        published_force_set = (
            self._last_force_set[side] if self._last_force_set[side] is not None
            else force_set
        )
        last = self._last_written[side]
        skip_write = (
            last is not None and all(
                abs(cmd[i] - last[i]) <= WRITE_DEADBAND
                for i in range(Inspire_Num_Motors))
        )
        t0 = time.monotonic()
        t_cmd = t0
        ok = True
        if not skip_write:
            ok = hand.write_angles(cmd)
            t_cmd = time.monotonic()
            if ok:
                self._last_written[side] = cmd
        if ok and readback and not skip_write:
            act = hand.read_angles()
        t_fb = time.monotonic()
        self._publish_side(
            side,
            cmd=cmd,
            actual=act,
            forces=forces,
            peaks=peaks,
            tactile=tactile,
            stalled=stalled,
            valid=1,
            lt=lt, rt=rt, lg=lg, rg=rg, dt_s=dt_s,
            force_set_g=published_force_set,
        )
        err = (
            int(max(abs(c - a) for c, a in zip(cmd, act)))
            if act is not None else "?"
        )
        force_max = max(forces) if forces else "?"
        if stalled:
            now = time.monotonic()
            if now - self._last_stall_log[side] >= 0.25:
                self._last_stall_log[side] = now
                print(f"[InspireBridge] {side} stall dofs={stalled} "
                      f"Fmax={force_max} tactile={tactile} "
                      f"holding (teleop continues)",
                      flush=True)
        if skip_write:
            return None
        return {
            "cmd_ms": (t_cmd - t0) * 1000.0,
            "fb_ms": (t_fb - t0) * 1000.0,
            "err": err,
            "stall": stalled,
            "force_max": force_max,
        }

    def _run(self):
        while not self._stop_evt.is_set():
            t0 = time.time()
            try:
                _, lt, rt, lg, rg = self._get_inputs()
                _cur = (round(lt, 2), round(rt, 2), round(lg, 2), round(rg, 2))
                _prev = getattr(self, "_dbg_prev", None)
                changed = _prev is not None and _cur != _prev
                writes = {}
                trig = {"left": (lt, lg), "right": (rt, rg)}
                _now = time.monotonic()
                _dt = _now - getattr(self, "_last_run_t", _now)
                self._last_run_t = _now
                read_tactile = self._tactile_this_tick(self._loop_tick)
                self._loop_tick += 1
                for side in self._hands:
                    t, s = trig[side]
                    w = self._write_side(
                        side, self._targets_from_triggers(t, s),
                        readback=changed,
                        lt=lt, rt=rt, lg=lg, rg=rg, dt_s=_dt,
                        read_tactile=read_tactile)
                    if w:
                        writes[side] = w
                if changed:
                    _now = time.monotonic()
                    _dt = _now - getattr(self, "_dbg_prev_t", _now)
                    print(self._format_change(_dt, lt, rt, lg, rg, writes),
                          flush=True)
                    self._dbg_prev_t = _now
                self._dbg_prev = _cur
            except Exception as e:
                # Never let hand I/O disturb the teleop loop; log and continue.
                print(f"[InspireBridge] loop error (continuing): {e}")
                time.sleep(0.5)
            sleep_t = self._dt - (time.time() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)

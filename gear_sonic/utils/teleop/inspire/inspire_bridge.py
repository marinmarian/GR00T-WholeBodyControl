"""
InspireBridge — drive Inspire RH56DFTP hands from teleop controller inputs.

Runs a background daemon thread at ~60 Hz that polls a caller-supplied input
function and writes finger angles to both hands over Modbus TCP (via the
vendored InspireHandModbusTCP driver). Completely decoupled from the body
teleop pipeline: the C++ deploy's (inert) Dex3 output is untouched, and this
works in every stream mode (PLANNER, VR_3PT, POSE) as well as with either
input source (PICO/xrt or Quest/isaac-teleop) since both expose the same
trigger/squeeze controller values.

Mapping (mode="trigger"):
  trigger (0..1)  -> four-finger curl  [little, ring, middle, index]
  squeeze (0..1)  -> thumb bend, plus partial thumb rotation (opposition)
  angle = 1000 - value * 1000   (Inspire: 1000 = open, 0 = closed)

Safety: on stop() / process exit the hands are commanded fully open and the
connections closed. Modbus errors trigger periodic reconnects, never raise
into the teleop loop.
"""

import atexit
import threading
import time

from .inspire_hand_modbus import (
    InspireHandModbusTCP,
    DEFAULT_LEFT_IP,
    DEFAULT_RIGHT_IP,
    Inspire_Num_Motors,
)

OPEN = 1000
# Fraction of full thumb rotation applied at full squeeze (partial opposition
# gives a more natural pinch than full rotation).
THUMB_ROT_SCALE = 0.6
# EMA smoothing factor per 60 Hz tick (higher = snappier, lower = smoother).
EMA_ALPHA = 0.5
# Skip Modbus writes when no DOF changed by more than this many angle units.
WRITE_DEADBAND = 3
RECONNECT_PERIOD_S = 2.0


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
        rate_hz: write loop frequency.
        speed: Inspire per-DOF speed setting (0-1000) written once on connect.
    """

    def __init__(self, get_inputs, mode="trigger",
                 left_ip=DEFAULT_LEFT_IP, right_ip=DEFAULT_RIGHT_IP,
                 rate_hz=60.0, speed=1000):
        if mode == "handtracking":
            raise NotImplementedError(
                "inspire-hands mode 'handtracking' requires per-finger data from "
                "the headset reader, which is not wired up yet — use 'trigger'."
            )
        if mode != "trigger":
            raise ValueError(f"unknown inspire-hands mode '{mode}'")
        self._get_inputs = get_inputs
        self._dt = 1.0 / rate_hz
        self._speed = int(speed)
        self._hands = {
            "left": InspireHandModbusTCP(left_ip, label="InspireL"),
            "right": InspireHandModbusTCP(right_ip, label="InspireR"),
        }
        self._smoothed = {"left": [float(OPEN)] * Inspire_Num_Motors,
                          "right": [float(OPEN)] * Inspire_Num_Motors}
        self._last_written = {"left": None, "right": None}
        self._last_reconnect = {"left": 0.0, "right": 0.0}
        self._stop_evt = threading.Event()
        self._thread = None

    # ------------------------------------------------------------------ API

    def start(self):
        for side, hand in self._hands.items():
            if hand.connect():
                hand.write_speed([self._speed] * Inspire_Num_Motors)
            else:
                print(f"[InspireBridge] {side} hand not reachable at "
                      f"{hand.ip}:{hand.port} — will keep retrying")
        self._thread = threading.Thread(target=self._run, name="InspireBridge",
                                        daemon=True)
        self._thread.start()
        atexit.register(self.stop)
        print("[InspireBridge] started (trigger mode: trigger=fingers, "
              "squeeze=thumb)")

    def stop(self):
        if self._stop_evt.is_set():
            return
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        for hand in self._hands.values():
            try:
                if hand.connected:
                    hand.write_angles([OPEN] * Inspire_Num_Motors)
                hand.close()
            except Exception:
                pass
        print("[InspireBridge] stopped (hands opened)")

    # ------------------------------------------------------------ internals

    @staticmethod
    def _targets_from_triggers(trigger, squeeze):
        """Map (trigger, squeeze) in 0..1 to 6 Inspire angles."""
        t = min(max(float(trigger), 0.0), 1.0)
        s = min(max(float(squeeze), 0.0), 1.0)
        finger = OPEN - t * OPEN
        thumb_bend = OPEN - s * OPEN
        thumb_rot = OPEN - s * OPEN * THUMB_ROT_SCALE
        # DOF order: [little, ring, middle, index, thumb_bend, thumb_rotation]
        return [finger, finger, finger, finger, thumb_bend, thumb_rot]

    def _write_side(self, side, targets):
        hand = self._hands[side]
        if not hand.connected:
            now = time.time()
            if now - self._last_reconnect[side] >= RECONNECT_PERIOD_S:
                self._last_reconnect[side] = now
                if hand.reconnect():
                    hand.write_speed([self._speed] * Inspire_Num_Motors)
                    self._last_written[side] = None
            if not hand.connected:
                return
        sm = self._smoothed[side]
        for i in range(Inspire_Num_Motors):
            sm[i] += EMA_ALPHA * (targets[i] - sm[i])
        cmd = [int(min(max(v, 0), OPEN)) for v in sm]
        last = self._last_written[side]
        if last is not None and all(
                abs(cmd[i] - last[i]) <= WRITE_DEADBAND
                for i in range(Inspire_Num_Motors)):
            return
        if hand.write_angles(cmd):
            self._last_written[side] = cmd

    def _run(self):
        _last_dbg = 0.0
        while not self._stop_evt.is_set():
            t0 = time.time()
            try:
                _, lt, rt, lg, rg = self._get_inputs()
                _cur = (round(lt,2), round(rt,2), round(lg,2), round(rg,2))
                _prev = getattr(self, "_dbg_prev", None)
                if _prev is not None and _cur != _prev:
                    _now = time.monotonic()
                    _dt = _now - getattr(self, "_dbg_prev_t", _now)
                    print(f"[InspireBridge] change dt={_dt*1000:6.0f}ms lt={lt:.2f} rt={rt:.2f} lg={lg:.2f} rg={rg:.2f}", flush=True)
                    self._dbg_prev_t = _now
                self._dbg_prev = _cur
                self._write_side("left", self._targets_from_triggers(lt, lg))
                self._write_side("right", self._targets_from_triggers(rt, rg))
            except Exception as e:
                # Never let hand I/O disturb the teleop loop; log and continue.
                print(f"[InspireBridge] loop error (continuing): {e}")
                time.sleep(0.5)
            sleep_t = self._dt - (time.time() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)

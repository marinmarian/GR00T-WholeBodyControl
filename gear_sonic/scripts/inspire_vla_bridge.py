"""Drive the Inspire RH56 hands from GR00T VLA hand actions and publish their state.

Why this exists: the C++ SONIC deploy only knows Dex3 hands, so with Inspire hands the
policy's ``left/right_hand_joints`` would go nowhere and the observation's hand state
would be all zeros. This process closes both gaps, mirroring what the PICO streamer's
``InspireBridge`` did during data collection:

  * SUB  tcp://127.0.0.1:5556  topic ``pose``  (run_vla_inference.py latent action v4)
      -> 7-DoF G1 hand joints per side -> closure ratios -> Inspire angles (0..1000)
  * PUB  tcp://127.0.0.1:5558  topic ``inspire_hand`` (InspireDumpPublisher) so
      run_vla_inference.py records the REAL hand state in the observation.

Closure mapping is the exact inverse of ``inspire_actual_to_g1_hand_q`` used by the
exporter: fingers = mean over the index/middle slots of (q-open)/(closed-open),
thumb = mean over the thumb slots (slots where closed == open are skipped).
Ctrl-C, SIGTERM (pkill) or a Python exception -> InspireBridge.stop() opens the hands.
SIGKILL / power loss cannot be caught: the hands stay where they were.
"""
from __future__ import annotations

import json
import signal
import sys
import threading
import time

import numpy as np
import tyro
import zmq

from gear_sonic.utils.teleop.inspire.inspire_bridge import (
    DEFAULT_LEFT_IP,
    DEFAULT_RIGHT_IP,
    HANDS_RATE_HZ,
    InspireBridge,
)
from gear_sonic.utils.teleop.inspire.inspire_dump import (
    G1_HAND_FINGER_SLOTS,
    G1_HAND_THUMB_SLOTS,
    INSPIRE_DUMP_PORT,
    InspireDumpPublisher,
)
from gear_sonic.utils.teleop.solver.hand.g1_gripper_ik_solver import (
    G1GripperInverseKinematicsSolver,
)

HEADER_SIZE = 1280  # zmq_planner_sender._build_header pads the JSON header to 1280 bytes
_DTYPES = {"f32": np.float32, "f64": np.float64, "i32": np.int32, "i64": np.int64, "bool": bool}


def unpack_pose_message(packed: bytes, topic: str = "pose") -> dict:
    """[topic][1280-byte JSON header][binary fields] -> dict of arrays (copy of the exporter's)."""
    tb = topic.encode("utf-8")
    if not packed.startswith(tb):
        raise ValueError("not a pose message")
    off = len(tb)
    hdr = packed[off : off + HEADER_SIZE]
    nul = hdr.find(b"\x00")
    header = json.loads((hdr[:nul] if nul > 0 else hdr).decode("utf-8"))
    out, cur = {}, off + HEADER_SIZE
    for f in header.get("fields", []):
        dt = _DTYPES.get(f["dtype"], np.float32)
        shape = tuple(f["shape"])
        n = int(np.prod(shape)) * np.dtype(dt).itemsize
        out[f["name"]] = np.frombuffer(packed[cur : cur + n], dtype=dt).reshape(shape).copy()
        cur += n
    return out


def hand_open_closed_q(side: str) -> tuple[np.ndarray, np.ndarray]:
    """Same solver + fingertip trick as run_data_exporter / run_vla_inference."""
    solver = G1GripperInverseKinematicsSolver(side=side)
    tips = np.zeros([25, 4, 4])
    tips[4, 0, 3] = 1.0  # thumb tip
    open_q = np.asarray(solver({"position": tips}), dtype=np.float64).reshape(-1)
    tips[14, 0, 3] = 1.0  # middle tip -> closed grip
    closed_q = np.asarray(solver({"position": tips}), dtype=np.float64).reshape(-1)
    return open_q, closed_q


def closure_ratios(q7, open_q, closed_q, thumb_slots=G1_HAND_THUMB_SLOTS,
                   finger_slots=G1_HAND_FINGER_SLOTS) -> tuple[float, float]:
    """(finger_ratio, thumb_ratio) in [0,1] from a 7-DoF G1 hand joint vector."""
    q = np.asarray(q7, dtype=np.float64).reshape(-1)
    span = closed_q - open_q
    def ratio(slots):
        vals = [(q[k] - open_q[k]) / span[k] for k in slots if abs(span[k]) > 1e-6]
        return float(np.clip(np.mean(vals), 0.0, 1.0)) if vals else 0.0
    return ratio(finger_slots), ratio(thumb_slots)


class VLAHandTargets:
    """Latest VLA hand action -> (trigger, squeeze) per side, thread-safe."""

    def __init__(self, action_host: str, action_port: int, threshold: float | None, max_age_s: float):
        self._lock = threading.Lock()
        self._ratios = {"left": (0.0, 0.0), "right": (0.0, 0.0)}
        self._t_last = 0.0
        self._n = 0
        self._threshold = threshold
        self._max_age = max_age_s
        self._open_q, self._closed_q = {}, {}
        for side in ("left", "right"):
            self._open_q[side], self._closed_q[side] = hand_open_closed_q(side)
            print(f"[VLAHands] {side} open_q={np.round(self._open_q[side],2)} closed_q={np.round(self._closed_q[side],2)}", flush=True)
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.setsockopt(zmq.SUBSCRIBE, b"pose")
        self._sock.setsockopt(zmq.RCVHWM, 8)
        self._sock.connect(f"tcp://{action_host}:{action_port}")
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print(f"[VLAHands] SUB tcp://{action_host}:{action_port} topic=pose", flush=True)

    def _loop(self):
        poller = zmq.Poller(); poller.register(self._sock, zmq.POLLIN)
        while not self._stop.is_set():
            if not dict(poller.poll(100)):
                continue
            try:
                msg = unpack_pose_message(self._sock.recv(zmq.NOBLOCK))
            except Exception:
                continue
            new = {}
            for side, key in (("left", "left_hand_joints"), ("right", "right_hand_joints")):
                if key not in msg:
                    continue
                f, t = closure_ratios(msg[key].reshape(-1)[:7], self._open_q[side], self._closed_q[side])
                if self._threshold is not None:  # snap to open/closed (the data is near-binary)
                    f, t = float(f >= self._threshold), float(t >= self._threshold)
                new[side] = (f, t)
            if new:
                with self._lock:
                    self._ratios.update(new); self._t_last = time.monotonic(); self._n += 1

    def get_inputs(self):
        """InspireBridge contract: (menu_button, left_trigger, right_trigger, left_squeeze, right_squeeze)."""
        with self._lock:
            l, r, age = self._ratios["left"], self._ratios["right"], time.monotonic() - self._t_last
        if self._t_last == 0.0 or age > self._max_age:
            return (0, 0.0, 0.0, 0.0, 0.0)  # no fresh action -> open hands
        return (0, l[0], r[0], l[1], r[1])

    @property
    def count(self):
        return self._n

    def close(self):
        self._stop.set(); self._thread.join(timeout=1.0)
        self._sock.close(0); self._ctx.term()


def main(action_host: str = "127.0.0.1", action_port: int = 5556, dump_port: int = INSPIRE_DUMP_PORT,
         left_ip: str = DEFAULT_LEFT_IP, right_ip: str = DEFAULT_RIGHT_IP, rate_hz: float = HANDS_RATE_HZ,
         sides: str = "left,right", snap_threshold: float | None = 0.5, action_max_age: float = 2.0,
         dry_run: bool = False):
    """Args: snap_threshold: closure ratio >= this -> fully closed, else open (None = proportional).
    action_max_age: seconds without a fresh action before the hands are commanded open.
    dry_run: no Modbus, just print the decoded targets (for testing without hands)."""
    def _sigterm(signum, frame):  # pkill / tmux kill -> same path as Ctrl-C (opens the hands)
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _sigterm)
    signal.signal(signal.SIGHUP, _sigterm)

    targets = VLAHandTargets(action_host, action_port, snap_threshold, action_max_age)
    if dry_run:
        try:
            while True:
                time.sleep(0.5)
                print(f"[VLAHands dry-run] msgs={targets.count} inputs={tuple(round(x,2) for x in targets.get_inputs())}", flush=True)
        except KeyboardInterrupt:
            print("[VLAHands dry-run] shutdown signal received, exiting cleanly", flush=True)
            targets.close(); return
    dump = InspireDumpPublisher(port=dump_port)
    bridge = InspireBridge(targets.get_inputs, mode="trigger", left_ip=left_ip, right_ip=right_ip,
                           rate_hz=rate_hz, sides=tuple(s for s in sides.split(",") if s), dump_publisher=dump)
    bridge.start()
    print("[VLAHands] running: VLA hand actions -> Inspire hands; state -> dump port "
          f"{dump_port}. Ctrl-C opens the hands and exits.", flush=True)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        print("[VLAHands] stopping: opening hands", flush=True)
        bridge.stop(); targets.close(); dump.close()


if __name__ == "__main__":
    tyro.cli(main)

"""Policy relay + intervention arbitration for DAgger-style data collection (g1-vr-teleop #25).

The C++ SONIC deploy accepts ONE ZMQ input source on 5556 and locks the ``pose`` protocol
version for the whole streaming session (v3 = SMPL poses from the PICO streamer, v4 = motion
tokens from ``run_vla_inference.py``); a version change mid-session makes it leave streaming
mode "for safety". So the policy and the teleop cannot both talk to the deploy, and a human
cannot take over by simply streaming poses while the policy runs.

What works within those rules, and what this module supports:

  * The PICO streamer (``pico_manager_thread_server.py --manager``) stays the single owner of
    5556 and gains ``StreamMode.POLICY`` (6): it SUBscribes to the VLA client's action PUB
    (default 5576) and relays its v4 ``pose`` messages to the deploy unchanged
    (:class:`PolicyRelay`). The Inspire hands follow the relayed hand joints in that mode.
  * An intervention is the existing ``PLANNER_VR_3PT`` mode (5): the streamer switches the deploy
    to PLANNER mode (one ``command`` message, a safety reset on the C++ side) and drives the
    arms from the VR wrists, recalibrated onto the robot's MEASURED joints at the moment of the
    take-over so nothing jumps (that is the abc.bot "delta since the intervention" idea, and it
    was already there for PLANNER -> VR_3PT). Releasing switches back to STREAMED mode, which
    resets the protocol lock, and the relay resumes with fresh tokens.
  * The VLA client follows the streamer's ``manager_state.stream_mode`` (:class:`RelayArbiter`):
    it yields while the operator has the robot and resumes on its own when POLICY mode returns
    (only if it was running before), blending from the deploy's last encoder token.
  * The exporter records every frame's ``teleop.stream_mode`` already, so a DAgger episode needs
    no schema change: 6 = policy frames, 5 = intervention frames. ``action.motion_token`` is the
    token the deploy actually executed in both cases (external in 6, encoder output in 5).

Everything here is socket-free (callables are injected) so tools/tests/test_policy_relay.py can
run it without a robot.
"""
from __future__ import annotations

import json
import time
from typing import Callable, Iterable, Sequence

import numpy as np

POLICY_STREAM_MODE = 6
"""``StreamMode.POLICY`` in pico_manager_thread_server.py: the streamer relays policy tokens."""

INTERVENTION_STREAM_MODE = 5
"""``StreamMode.PLANNER_VR_3PT``: the operator drives the upper body; this is an intervention
whenever it happens inside a policy episode."""

SMPL_STREAM_MODES = (1, 4)
"""``POSE`` and ``POSE_PAUSE``: the only modes in which ``teleop.smpl_pose`` is meaningful."""

STREAM_MODE_NAMES = {
    0: "OFF",
    1: "POSE",
    2: "PLANNER",
    3: "PLANNER_FROZEN_UPPER_BODY",
    4: "POSE_PAUSE",
    5: "PLANNER_VR_3PT",
    6: "POLICY",
}

DEFAULT_POLICY_ACTION_PORT = 5576
"""Where run_vla_inference.py --relay binds its action PUB (5556 is the streamer's)."""

HEADER_SIZE = 1280  # zmq_planner_sender._build_header pads the JSON header to this
_DTYPES = {"f32": np.float32, "f64": np.float64, "i32": np.int32, "i64": np.int64, "bool": bool}


def unpack_pose_message(packed: bytes, topic: str = "pose") -> dict:
    """[topic][1280-byte JSON header][binary fields] -> {"version": v, field: ndarray, ...}."""
    tb = topic.encode("utf-8")
    if not packed.startswith(tb):
        raise ValueError(f"message does not start with topic '{topic}'")
    off = len(tb)
    if len(packed) < off + HEADER_SIZE:
        raise ValueError(f"packed data too small: {len(packed)} < {off + HEADER_SIZE}")
    hdr = packed[off : off + HEADER_SIZE]
    nul = hdr.find(b"\x00")
    header = json.loads((hdr[:nul] if nul > 0 else hdr).decode("utf-8"))
    out: dict = {"version": int(header.get("v", 0))}
    cur = off + HEADER_SIZE
    for f in header.get("fields", []):
        dt = _DTYPES.get(f["dtype"], np.float32)
        shape = tuple(f["shape"])
        n = int(np.prod(shape)) * np.dtype(dt).itemsize
        out[f["name"]] = np.frombuffer(packed[cur : cur + n], dtype=dt).reshape(shape).copy()
        cur += n
    return out


def is_token_message(msg: dict) -> bool:
    """True for a latent-action (protocol v4) ``pose`` message from run_vla_inference.py."""
    return "token_state" in msg


# ---------------------------------------------------------------------------
# Streamer side
# ---------------------------------------------------------------------------


class PolicyRelay:
    """Forward the VLA client's ``pose`` messages to the deploy while the streamer is in POLICY mode.

    ``recv()`` returns one raw message or None (non-blocking); ``send(raw)`` publishes on the
    streamer's PUB socket (the deploy's input). Only ``pose`` messages that carry ``token_state``
    are relayed: the VLA client's own ``command`` messages (k/x) are dropped here on purpose - the
    streamer owns start/stop and the PLANNER/STREAMED mode of the deploy.

    ``closure_fn(side, q7) -> (finger_ratio, thumb_ratio)`` turns the relayed 7-DoF G1 hand
    joints into Inspire closures for :meth:`hand_inputs` (same mapping as inspire_vla_bridge.py).
    """

    def __init__(
        self,
        recv: Callable[[], bytes | None],
        send: Callable[[bytes], None],
        closure_fn: Callable[[str, np.ndarray], tuple[float, float]] | None = None,
        snap_threshold: float | None = 0.5,
        hand_max_age_s: float = 2.0,
        clock: Callable[[], float] = time.monotonic,
    ):
        self._recv = recv
        self._send = send
        self._closure_fn = closure_fn
        self._snap = snap_threshold
        self._hand_max_age = float(hand_max_age_s)
        self._clock = clock
        self.frames_relayed = 0
        self.frames_dropped = 0
        self.last_relay_time: float | None = None
        self.last_hand_joints: dict[str, np.ndarray] = {}
        self._last_hand_time: float | None = None

    def pump(self, active: bool, max_msgs: int = 64) -> int:
        """Drain the SUB queue; relay token messages when ``active``. Returns messages relayed.

        Draining while inactive matters: otherwise the queue fills up during an intervention and
        the deploy would receive seconds-old tokens the instant POLICY mode returns.
        """
        relayed = 0
        for _ in range(max_msgs):
            raw = self._recv()
            if raw is None:
                break
            if not raw.startswith(b"pose"):
                self.frames_dropped += 1
                continue
            try:
                msg = unpack_pose_message(raw)
            except Exception:  # noqa: BLE001 - never let a bad frame kill the manager loop
                self.frames_dropped += 1
                continue
            if not is_token_message(msg):
                self.frames_dropped += 1
                continue
            if not active:
                self.frames_dropped += 1
                continue
            self._send(raw)
            relayed += 1
            self.frames_relayed += 1
            self.last_relay_time = self._clock()
            for side, key in (("left", "left_hand_joints"), ("right", "right_hand_joints")):
                if key in msg:
                    self.last_hand_joints[side] = np.asarray(msg[key], dtype=np.float64).reshape(-1)[:7]
                    self._last_hand_time = self.last_relay_time
        return relayed

    def reset(self) -> None:
        """Entering POLICY mode: throw away whatever was queued while we were not relaying."""
        self.pump(active=False, max_msgs=100000)
        self.last_hand_joints = {}
        self._last_hand_time = None

    def hand_inputs(self) -> tuple[int, float, float, float, float]:
        """InspireBridge ``get_inputs`` contract: (menu, left_trigger, right_trigger, left_squeeze, right_squeeze).

        Trigger = finger closure, squeeze = thumb closure, decoded from the last relayed hand
        joints; all zeros (open hands) when nothing fresh was relayed, mirroring the
        ``--action-max-age`` behaviour of inspire_vla_bridge.py.
        """
        if self._closure_fn is None or self._last_hand_time is None:
            return (0, 0.0, 0.0, 0.0, 0.0)
        if self._clock() - self._last_hand_time > self._hand_max_age:
            return (0, 0.0, 0.0, 0.0, 0.0)
        out = {}
        for side in ("left", "right"):
            q = self.last_hand_joints.get(side)
            if q is None:
                out[side] = (0.0, 0.0)
                continue
            f, t = self._closure_fn(side, q)
            if self._snap is not None:
                f, t = float(f >= self._snap), float(t >= self._snap)
            out[side] = (float(f), float(t))
        return (0, out["left"][0], out["right"][0], out["left"][1], out["right"][1])


# ---------------------------------------------------------------------------
# VLA client side
# ---------------------------------------------------------------------------


class RelayArbiter:
    """Follow the streamer's ``stream_mode`` and tell the VLA client when to yield and resume.

    ``update(stream_mode, now, paused)`` is called every control tick with the latest
    ``manager_state.stream_mode`` seen (None = none yet) and the client's own pause flag.
    Events (each fires once per transition):

    ``"yield"``     the streamer left POLICY mode (intervention, PLANNER, OFF ...) -> stop acting,
                    drop the cached chunk. ``was_running`` records whether the policy was live.
    ``"resume"``    POLICY mode is back and the policy was running before -> resume by itself.
    ``"handover"``  POLICY mode is back (or reached for the first time) but the client is paused
                    -> stay paused, the operator presses ``p``.
    ``"lost"``      no ``manager_state`` for ``manager_timeout_s`` -> treat as yield without
                    auto-resume (the streamer died or 5556 is not ours to talk to).
    """

    def __init__(self, policy_mode: int = POLICY_STREAM_MODE, manager_timeout_s: float = 2.0):
        self.policy_mode = int(policy_mode)
        self.manager_timeout_s = float(manager_timeout_s)
        self.mode: int | None = None
        self.last_seen: float | None = None
        self.yielded = True          # until the streamer says POLICY we are not the source
        self.was_running = False     # policy live before the last yield -> auto-resume
        self.lost = False

    @property
    def in_policy_mode(self) -> bool:
        return self.mode == self.policy_mode and not self.lost

    def update(self, stream_mode: int | None, now: float, paused: bool) -> str | None:
        if stream_mode is not None:
            self.mode = int(stream_mode)
            self.last_seen = now
            self.lost = False
        if self.last_seen is None:
            return None
        if now - self.last_seen > self.manager_timeout_s:
            if not self.lost:
                self.lost = True
                self.yielded = True
                self.was_running = False
                return "lost"
            return None
        if self.mode == self.policy_mode:
            if self.yielded:
                self.yielded = False
                if self.was_running:
                    self.was_running = False
                    return "resume"
                return "handover"
            return None
        if not self.yielded:
            self.yielded = True
            self.was_running = not paused
            return "yield"
        return None

    def set_resume_intent(self, running: bool) -> None:
        """``p`` pressed while yielded: decide whether POLICY mode returning resumes the policy."""
        self.was_running = bool(running)


def resume_blend_token(start: np.ndarray, target: np.ndarray, step: int, num_steps: int) -> np.ndarray:
    """Linear blend from the deploy's last token to the policy's, step 1..num_steps -> target."""
    if num_steps <= 0:
        return np.asarray(target, dtype=np.float32)
    alpha = min(1.0, max(0.0, step / float(num_steps)))
    return ((1.0 - alpha) * np.asarray(start, dtype=np.float32)
            + alpha * np.asarray(target, dtype=np.float32)).astype(np.float32)


# ---------------------------------------------------------------------------
# Recorded episodes
# ---------------------------------------------------------------------------


def contiguous_segments(mask: Sequence[bool] | np.ndarray) -> list[tuple[int, int]]:
    """[start, end) index ranges of the True runs in ``mask``."""
    m = np.asarray(mask, dtype=bool)
    if m.size == 0:
        return []
    edges = np.flatnonzero(np.diff(np.concatenate(([0], m.astype(np.int8), [0]))))
    return [(int(edges[i]), int(edges[i + 1])) for i in range(0, len(edges), 2)]


def stream_mode_summary(modes: Iterable[int]) -> dict:
    """Per-episode DAgger bookkeeping from the frames' ``teleop.stream_mode`` values."""
    arr = np.asarray(list(modes), dtype=np.int64).reshape(-1)
    by_mode = {int(k): int(v) for k, v in zip(*np.unique(arr, return_counts=True))} if arr.size else {}
    segs = contiguous_segments(arr == INTERVENTION_STREAM_MODE)
    return {
        "frames": int(arr.size),
        "by_mode": by_mode,
        "policy_frames": by_mode.get(POLICY_STREAM_MODE, 0),
        "intervention_frames": by_mode.get(INTERVENTION_STREAM_MODE, 0),
        "intervention_segments": len(segs),
        "segments": segs,
    }


def format_stream_mode_summary(summary: dict) -> str:
    n = summary["frames"]
    if n == 0:
        return "0 frames"
    parts = [
        f"{STREAM_MODE_NAMES.get(m, m)} {c} ({100.0 * c / n:.0f}%)"
        for m, c in sorted(summary["by_mode"].items())
    ]
    text = f"{n} frames: " + ", ".join(parts)
    if summary["intervention_segments"]:
        text += f"; {summary['intervention_segments']} intervention segment(s)"
    return text

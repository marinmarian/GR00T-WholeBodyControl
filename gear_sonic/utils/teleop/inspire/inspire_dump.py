"""Inspire hand dump: snapshot schema, ZMQ PUB/SUB, LeRobot feature keys.

Mechanical angles/FORCE_ACT and DFTP tip peaks share t_unix/t_mono but travel
on separate ZMQ topics so they are not mixed in one table conceptually.
LeRobot 2.1 only has a single dataset-wide fps, so the exporter stores both
as separate feature columns on the same parquet frame grid.

Also owns the rewritable last-run debug log (``inspire-last-run.log``) with
per-tick cmd/actual/force 6-vectors. That file is independent of the dated
event log, which skips per-tick ``change dt=`` lines.
"""

import os
import threading

import msgpack
import numpy as np
import zmq

# Local copy so the exporter can import this module without pymodbus
# (inspire_hand_modbus is host-side Modbus; the container venv does not have it).
Inspire_Num_Motors = 6

# Keep in sync with inspire_bridge.FORCE_LIMIT_G (avoid circular import).
FORCE_SET_G_DEFAULT = 400

INSPIRE_DUMP_HOST = "127.0.0.1"
INSPIRE_DUMP_PORT = 5558
INSPIRE_HAND_TOPIC = "inspire_hand"
INSPIRE_TACTILE_TOPIC = "inspire_tactile"
# Shared parquet fps for body + hands. LeRobot 2.1 info.json has one fps.
# Body control_frequency is 50 Hz, so some proprio rows are latest-wins repeats.
INSPIRE_RECORD_FPS = 60
LAST_RUN_LOG_NAME = "inspire-last-run.log"

_LAST_RUN_LOCK = threading.Lock()
_LAST_RUN_FP = None
_LAST_RUN_PATH = None

INSPIRE_DOF_NAMES = [
    "little",
    "ring",
    "middle",
    "index",
    "thumb_bend",
    "thumb_rot",
]
INSPIRE_TIP_REGION_NAMES = [
    "little_finger",
    "ring_finger",
    "middle_finger",
    "index_finger",
    "thumb",
]


def format_dump_addr_in_use(port, host=INSPIRE_DUMP_HOST, pid=None):
    """Operator hint when the dump PUB cannot bind (leftover pico_vive_bridge)."""
    if pid is None:
        pid = os.getpid()
    return (
        f"[InspireDump] Address already in use tcp://{host}:{port} "
        f"pid={pid}. Leftover pico_vive_bridge holding 5555/5556/5558? "
        f"pgrep -af pico_vive_bridge then kill PID. Do not kill the XR service."
    )


def last_run_log_path(log_dir=None):
    """Path of the rewritable last-run debug log (honors G1_VR_LOG_DIR)."""
    if not log_dir:
        log_dir = os.environ.get("G1_VR_LOG_DIR") or os.path.join(os.getcwd(), "logs")
    return os.path.join(log_dir, LAST_RUN_LOG_NAME)


def setup_last_run_log(log_dir=None, path=None):
    """Open the last-run log in ``'w'`` so it holds only the current run."""
    global _LAST_RUN_FP, _LAST_RUN_PATH
    path = path or last_run_log_path(log_dir)
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with _LAST_RUN_LOCK:
        if _LAST_RUN_FP is not None:
            try:
                _LAST_RUN_FP.close()
            except Exception:
                pass
        _LAST_RUN_FP = open(path, "w", encoding="utf-8")
        _LAST_RUN_PATH = path
    return path


def close_last_run_log():
    """Close the last-run handle (tests / shutdown)."""
    global _LAST_RUN_FP, _LAST_RUN_PATH
    with _LAST_RUN_LOCK:
        if _LAST_RUN_FP is not None:
            try:
                _LAST_RUN_FP.close()
            except Exception:
                pass
            _LAST_RUN_FP = None
        _LAST_RUN_PATH = None


def _fmt_ints(values):
    return "[" + ",".join(str(int(v)) for v in values) + "]"


def _fmt_bools(values):
    return "[" + ",".join("1" if v else "0" for v in values) + "]"


def format_last_run_line(mechanical, lt=0.0, rt=0.0, lg=0.0, rg=0.0, dt_s=None):
    """One grep-friendly sample: cmd/actual/force 6-vectors plus stall flags."""
    cmd = mechanical.get("cmd") or []
    actual = mechanical.get("actual") or []
    forces = mechanical.get("force_act_g") or []
    n = min(len(cmd), len(actual))
    if n:
        err = int(max(abs(int(cmd[i]) - int(actual[i])) for i in range(n)))
    else:
        err = "?"
    force_max = max((int(v) for v in forces), default="?")
    stall = mechanical.get("stall_dofs") or []
    tactile = mechanical.get("tactile_hit") or []
    force_set = mechanical.get("force_set_g") or []
    if isinstance(force_set, (int, float)):
        force_set = [force_set]
    dt_part = "" if dt_s is None else f" dt_ms={dt_s * 1000:.0f}"
    return (
        f"t_unix={mechanical.get('t_unix', 0.0)} "
        f"t_mono={mechanical.get('t_mono', 0.0)}"
        f"{dt_part} "
        f"side={mechanical.get('side', '?')} "
        f"lt={float(lt):.2f} rt={float(rt):.2f} lg={float(lg):.2f} rg={float(rg):.2f} "
        f"cmd={_fmt_ints(cmd)} "
        f"actual={_fmt_ints(actual)} "
        f"force_act_g={_fmt_ints(forces)} "
        f"force_set_g={_fmt_ints(force_set)} "
        f"err={err} "
        f"stall={_fmt_ints(stall)} "
        f"Fmax={force_max} "
        f"tactile={_fmt_bools(tactile)} "
        f"yielded={1 if mechanical.get('yielded') else 0} "
        f"valid={int(mechanical.get('valid', 0))}"
    )


def write_last_run_sample(mechanical, lt=0.0, rt=0.0, lg=0.0, rg=0.0, dt_s=None):
    """Append one last-run line if setup_last_run_log() has opened the file."""
    if _LAST_RUN_FP is None:
        return
    line = format_last_run_line(
        mechanical, lt=lt, rt=rt, lg=lg, rg=rg, dt_s=dt_s,
    )
    with _LAST_RUN_LOCK:
        if _LAST_RUN_FP is None:
            return
        _LAST_RUN_FP.write(line + "\n")
        _LAST_RUN_FP.flush()


def _zeros(n):
    return [0] * n


def _as_int(value):
    try:
        return int(value)
    except (TypeError, ValueError):
        return 0


def _tip_list(tip_peaks):
    if not tip_peaks:
        return _zeros(len(INSPIRE_TIP_REGION_NAMES))
    return [_as_int(tip_peaks.get(name, 0)) for name in INSPIRE_TIP_REGION_NAMES]


def _force_set_list(force_set_g, n):
    """Per-DOF FORCE_SET grams written that tick (never a lone cruise scalar)."""
    if force_set_g is None:
        return [FORCE_SET_G_DEFAULT] * n
    if isinstance(force_set_g, (int, float)):
        return [_as_int(force_set_g)] * n
    values = [_as_int(v) for v in force_set_g]
    return values[:n] + _zeros(max(0, n - len(values)))


def build_inspire_snapshots(
    side,
    cmd,
    actual,
    force_act_g,
    tip_peaks,
    tactile_hit,
    stall_dofs,
    yielded,
    valid,
    t_unix,
    t_mono,
    force_set_g=FORCE_SET_G_DEFAULT,
    tactile_valid=None,
):
    """Two dicts, same clocks: mechanical vs fingertip peaks."""
    n = Inspire_Num_Motors
    cmd = list(cmd) if cmd is not None else _zeros(n)
    actual = list(actual) if actual is not None else _zeros(n)
    force_act_g = list(force_act_g) if force_act_g is not None else _zeros(n)
    tactile_hit = list(tactile_hit) if tactile_hit is not None else [False] * n
    stall_dofs = [int(i) for i in (stall_dofs or [])]
    if tactile_valid is None:
        tactile_valid = int(bool(valid) and tip_peaks is not None)
    mechanical = {
        "t_unix": float(t_unix),
        "t_mono": float(t_mono),
        "side": side,
        "cmd": [int(v) for v in cmd[:n]] + _zeros(max(0, n - len(cmd))),
        "actual": [int(v) for v in actual[:n]] + _zeros(max(0, n - len(actual))),
        "force_act_g": [int(v) for v in force_act_g[:n]] + _zeros(max(0, n - len(force_act_g))),
        "force_set_g": _force_set_list(force_set_g, n),
        "tactile_hit": [bool(v) for v in tactile_hit[:n]] + [False] * max(0, n - len(tactile_hit)),
        "stall_dofs": stall_dofs,
        "yielded": bool(yielded),
        "valid": int(valid),
    }
    tactile = {
        "t_unix": float(t_unix),
        "t_mono": float(t_mono),
        "side": side,
        "tip_peaks": _tip_list(tip_peaks),
        "valid": int(tactile_valid),
    }
    return mechanical, tactile


def empty_inspire_snapshots(side, t_unix, t_mono):
    return build_inspire_snapshots(
        side=side,
        cmd=_zeros(Inspire_Num_Motors),
        actual=_zeros(Inspire_Num_Motors),
        force_act_g=_zeros(Inspire_Num_Motors),
        tip_peaks=None,
        tactile_hit=[False] * Inspire_Num_Motors,
        stall_dofs=[],
        yielded=False,
        valid=0,
        t_unix=t_unix,
        t_mono=t_mono,
        tactile_valid=0,
    )


def get_inspire_dataset_features():
    """LeRobot 2.1 feature dict. Tactile keys are not under observation.state."""
    dof_names = list(INSPIRE_DOF_NAMES)
    tip_names = list(INSPIRE_TIP_REGION_NAMES)
    return {
        "observation.inspire_cmd": {
            "dtype": "float64", "shape": (6,), "names": dof_names,
        },
        "observation.inspire_q": {
            "dtype": "float64", "shape": (6,), "names": dof_names,
        },
        "observation.inspire_force_g": {
            "dtype": "float64", "shape": (6,), "names": dof_names,
        },
        "observation.inspire_valid": {
            "dtype": "float64", "shape": (1,), "names": ["valid"],
        },
        "observation.inspire_t_unix": {
            "dtype": "float64", "shape": (1,), "names": ["t_unix"],
        },
        "observation.inspire_t_mono": {
            "dtype": "float64", "shape": (1,), "names": ["t_mono"],
        },
        "observation.inspire_tactile_hit": {
            "dtype": "float64", "shape": (6,), "names": dof_names,
        },
        "observation.inspire_stall_mask": {
            "dtype": "float64", "shape": (6,), "names": dof_names,
        },
        "observation.inspire_yielded": {
            "dtype": "float64", "shape": (1,), "names": ["yielded"],
        },
        "observation.inspire_tactile_tip": {
            "dtype": "float64", "shape": (5,), "names": tip_names,
        },
        "observation.inspire_tactile_t_unix": {
            "dtype": "float64", "shape": (1,), "names": ["t_unix"],
        },
        "observation.inspire_tactile_t_mono": {
            "dtype": "float64", "shape": (1,), "names": ["t_mono"],
        },
        "observation.inspire_tactile_valid": {
            "dtype": "float64", "shape": (1,), "names": ["valid"],
        },
        "observation.proprio_t_unix": {
            "dtype": "float64", "shape": (1,), "names": ["t_unix"],
        },
    }


def get_inspire_modality_config():
    return {
        "inspire": {
            "cmd": {"start": 0, "end": 6, "original_key": "observation.inspire_cmd"},
            "q": {"start": 0, "end": 6, "original_key": "observation.inspire_q"},
            "force_g": {"start": 0, "end": 6, "original_key": "observation.inspire_force_g"},
        },
        "inspire_tactile": {
            "tip": {"start": 0, "end": 5, "original_key": "observation.inspire_tactile_tip"},
        },
    }


def inspire_frame_from_snapshots(mechanical, tactile, proprio_t_unix):
    """Numpy columns for one LeRobot parquet frame."""
    n = Inspire_Num_Motors
    stall_mask = [0.0] * n
    for i in mechanical.get("stall_dofs") or []:
        if 0 <= int(i) < n:
            stall_mask[int(i)] = 1.0
    hit = [1.0 if v else 0.0 for v in mechanical.get("tactile_hit") or []]
    hit = hit[:n] + [0.0] * max(0, n - len(hit))
    return {
        "observation.inspire_cmd": np.asarray(mechanical["cmd"], dtype=np.float64),
        "observation.inspire_q": np.asarray(mechanical["actual"], dtype=np.float64),
        "observation.inspire_force_g": np.asarray(mechanical["force_act_g"], dtype=np.float64),
        "observation.inspire_valid": np.asarray([mechanical.get("valid", 0)], dtype=np.float64),
        "observation.inspire_t_unix": np.asarray([mechanical.get("t_unix", 0.0)], dtype=np.float64),
        "observation.inspire_t_mono": np.asarray([mechanical.get("t_mono", 0.0)], dtype=np.float64),
        "observation.inspire_tactile_hit": np.asarray(hit, dtype=np.float64),
        "observation.inspire_stall_mask": np.asarray(stall_mask, dtype=np.float64),
        "observation.inspire_yielded": np.asarray(
            [1.0 if mechanical.get("yielded") else 0.0], dtype=np.float64
        ),
        "observation.inspire_tactile_tip": np.asarray(tactile["tip_peaks"], dtype=np.float64),
        "observation.inspire_tactile_t_unix": np.asarray(
            [tactile.get("t_unix", 0.0)], dtype=np.float64
        ),
        "observation.inspire_tactile_t_mono": np.asarray(
            [tactile.get("t_mono", 0.0)], dtype=np.float64
        ),
        "observation.inspire_tactile_valid": np.asarray(
            [tactile.get("valid", 0)], dtype=np.float64
        ),
        "observation.proprio_t_unix": np.asarray([proprio_t_unix], dtype=np.float64),
    }


class InspireDumpPublisher:
    """PUB mechanical + tactile on one port, multipart topic prefix.

    CONFLATE is not set on the PUB: two topics share the socket, and CONFLATE
    would keep only the last message. Subscribers set CONFLATE=1 per topic.
    """

    def __init__(self, ctx=None, port=INSPIRE_DUMP_PORT, pack=None, bind_host=INSPIRE_DUMP_HOST):
        owns_ctx = ctx is None
        self._owns_ctx = owns_ctx
        self._ctx = ctx if ctx is not None else zmq.Context()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.setsockopt(zmq.SNDHWM, 2)
        self._sock.setsockopt(zmq.LINGER, 0)
        try:
            self._sock.bind(f"tcp://{bind_host}:{port}")
        except zmq.ZMQError:
            print(format_dump_addr_in_use(port, host=bind_host), flush=True)
            try:
                self._sock.close(0)
            except Exception:
                pass
            if owns_ctx:
                try:
                    self._ctx.term()
                except Exception:
                    pass
            raise
        self._pack = pack if pack is not None else (lambda obj: msgpack.packb(obj, use_bin_type=True))
        print(f"[InspireDump] PUB tcp://{bind_host}:{port} "
              f"topics={INSPIRE_HAND_TOPIC},{INSPIRE_TACTILE_TOPIC}", flush=True)

    def publish(self, mechanical, tactile):
        self._sock.send_multipart(
            [INSPIRE_HAND_TOPIC.encode("utf-8"), self._pack(mechanical)]
        )
        self._sock.send_multipart(
            [INSPIRE_TACTILE_TOPIC.encode("utf-8"), self._pack(tactile)]
        )

    def close(self):
        try:
            self._sock.close(0)
        except Exception:
            pass
        if self._owns_ctx:
            try:
                self._ctx.term()
            except Exception:
                pass


class _ConflatedTopicSub:
    """One CONFLATE SUB socket for a single multipart topic on the dump port."""

    def __init__(self, ctx, host, port, topic):
        self._topic = topic.encode("utf-8") if isinstance(topic, str) else topic
        self._sock = ctx.socket(zmq.SUB)
        self._sock.setsockopt(zmq.SUBSCRIBE, self._topic)
        self._sock.setsockopt(zmq.CONFLATE, 1)
        self._sock.setsockopt(zmq.RCVTIMEO, 0)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.connect(f"tcp://{host}:{port}")
        self._msg = None

    def poll(self):
        try:
            parts = self._sock.recv_multipart(zmq.NOBLOCK)
        except zmq.Again:
            return
        if len(parts) < 2:
            return
        self._msg = msgpack.unpackb(parts[1], raw=False)

    def get_msg(self, clear=False):
        self.poll()
        msg = self._msg
        if clear:
            self._msg = None
        return msg

    def close(self):
        self._sock.close(0)


class InspireDumpSubscriber:
    """Latest-wins mechanical + tactile SUBs (two sockets, same port)."""

    def __init__(self, host=INSPIRE_DUMP_HOST, port=INSPIRE_DUMP_PORT, ctx=None):
        self._owns_ctx = ctx is None
        self._ctx = ctx if ctx is not None else zmq.Context()
        self._hand = _ConflatedTopicSub(self._ctx, host, port, INSPIRE_HAND_TOPIC)
        self._tactile = _ConflatedTopicSub(self._ctx, host, port, INSPIRE_TACTILE_TOPIC)
        print(f"[InspireDump] SUB tcp://{host}:{port} "
              f"topics={INSPIRE_HAND_TOPIC},{INSPIRE_TACTILE_TOPIC}", flush=True)

    def get_latest(self):
        return self._hand.get_msg(clear=False), self._tactile.get_msg(clear=False)

    def close(self):
        self._hand.close()
        self._tactile.close()
        if self._owns_ctx:
            try:
                self._ctx.term()
            except Exception:
                pass

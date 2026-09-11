"""Minimal Isaac-GR00T ``PolicyClient`` for the Python 3.10 inference venv.

Isaac-GR00T itself requires Python >=3.12 (and pulls torch/transformers), so the
``gear_sonic[inference]`` extra cannot be installed on this Jetson. The ZMQ/msgpack
wire protocol needs none of that: this module is a trimmed copy of
``gr00t/policy/server_client.py`` (Isaac-GR00T @ 51d4c89, Apache-2.0) with the
``ModalityConfig`` dataclass inlined. Only the client side is kept.
"""
from __future__ import annotations

from dataclasses import dataclass, field, asdict
import functools
import io
import json
from typing import Any

import msgpack
import msgpack_numpy as mnp
import numpy as np
import zmq


@dataclass
class ModalityConfig:
    """Mirror of ``gr00t.data.types.ModalityConfig`` (only the fields the client needs)."""

    delta_indices: list[int] = field(default_factory=list)
    modality_keys: list[str] = field(default_factory=list)
    sin_cos_embedding_keys: list[str] | None = None
    mean_std_embedding_keys: list[str] | None = None
    action_configs: Any = None
    extra: dict = field(default_factory=dict, repr=False)

    @classmethod
    def from_payload(cls, payload: dict) -> "ModalityConfig":
        known = {k: payload[k] for k in ("delta_indices", "modality_keys", "sin_cos_embedding_keys",
                                        "mean_std_embedding_keys", "action_configs") if k in payload}
        extra = {k: v for k, v in payload.items() if k not in known}
        return cls(**known, extra=extra)


def _to_json_serializable(obj: Any) -> Any:
    if isinstance(obj, ModalityConfig):
        d = asdict(obj); d.pop("extra", None); return _to_json_serializable(d)
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, (np.floating,)):
        return float(obj)
    if isinstance(obj, np.bool_):
        return bool(obj)
    if isinstance(obj, dict):
        return {k: _to_json_serializable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_to_json_serializable(v) for v in obj]
    return obj


class MsgSerializer:
    """msgpack_numpy serializer with the same ``allow_pickle=False`` boundary as upstream."""

    @staticmethod
    def to_bytes(data: Any) -> bytes:
        default = functools.partial(MsgSerializer._safe_encode, chain=MsgSerializer._encode_custom)
        return msgpack.packb(data, default=default)

    @staticmethod
    def from_bytes(data: bytes) -> Any:
        object_hook = functools.partial(MsgSerializer._safe_decode, chain=MsgSerializer._decode_custom)
        return msgpack.unpackb(data, object_hook=object_hook, raw=False)

    @staticmethod
    def _safe_encode(obj, chain=None):
        if isinstance(obj, np.ndarray) and obj.dtype.kind == "O":
            raise TypeError(f"Refusing to encode object-dtype ndarray (shape={obj.shape})")
        return mnp.encode(obj, chain=chain)

    @staticmethod
    def _safe_decode(obj, chain=None):
        if isinstance(obj, dict):
            marker = obj.get("__ndarray_class__", obj.get(b"__ndarray_class__"))
            if marker:
                payload = obj.get("as_npy", obj.get(b"as_npy"))
                if payload is None:
                    raise ValueError("Malformed ndarray payload: marker present but 'as_npy' missing")
                return np.load(io.BytesIO(payload), allow_pickle=False)
            nd_val = obj.get(b"nd", obj.get("nd"))
            kind_val = obj.get(b"kind", obj.get("kind"))
            if nd_val and kind_val in (b"O", "O"):
                raise ValueError("Refusing to decode object-dtype (pickle-bearing) ndarray payload")
        return mnp.decode(obj, chain=chain)

    @staticmethod
    def _encode_custom(obj):
        if isinstance(obj, ModalityConfig):
            return {"__ModalityConfig__": True, "as_json": _to_json_serializable(obj)}
        return obj

    @staticmethod
    def _decode_custom(obj):
        if not isinstance(obj, dict):
            return obj
        if any(k in obj for k in ("__ModalityConfig__", b"__ModalityConfig__",
                                  "__ModalityConfig_class__", b"__ModalityConfig_class__")):
            key = next((k for k in ("as_json", b"as_json") if k in obj), None)
            if key is None:
                raise ValueError("Malformed ModalityConfig payload: 'as_json' missing")
            payload = obj[key]
            if isinstance(payload, bytes):
                payload = payload.decode()
            if isinstance(payload, str):
                payload = json.loads(payload)
            return ModalityConfig.from_payload(payload)
        return obj


class PolicyClient:
    """ZMQ REQ client for ``gr00t.eval.run_gr00t_server``. API-compatible subset."""

    def __init__(self, host: str = "localhost", port: int = 5555, timeout_ms: int = 15000,
                 api_token: str | None = None, strict: bool = False):
        self.strict = strict
        self._closed = False
        self.context = zmq.Context()
        self.host, self.port, self.timeout_ms, self.api_token = host, port, timeout_ms, api_token
        self._init_socket()

    def _init_socket(self):
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        self.socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.connect(f"tcp://{self.host}:{self.port}")

    def ping(self) -> bool:
        try:
            self.call_endpoint("ping", requires_input=False)
            return True
        except zmq.error.ZMQError:
            self._init_socket()
            return False

    def call_endpoint(self, endpoint: str, data: dict | None = None, requires_input: bool = True) -> Any:
        request: dict = {"endpoint": endpoint}
        if requires_input:
            request["data"] = data
        if self.api_token:
            request["api_token"] = self.api_token
        try:
            self.socket.send(MsgSerializer.to_bytes(request))
            message = self.socket.recv()
        except zmq.error.Again:
            self.socket.close(linger=0)
            self._init_socket()
            raise
        if message == b"ERROR":
            raise RuntimeError("Server error. Make sure we are running the correct policy server.")
        response = MsgSerializer.from_bytes(message)
        if isinstance(response, dict) and "error" in response:
            raise RuntimeError(f"Server error: {response['error']}")
        return response

    def get_action(self, observation: dict[str, Any], options: dict[str, Any] | None = None):
        response = self.call_endpoint("get_action", {"observation": observation, "options": options})
        return tuple(response)  # (action, info)

    def reset(self, options: dict[str, Any] | None = None) -> dict[str, Any]:
        return self.call_endpoint("reset", {"options": options})

    def get_modality_config(self) -> dict[str, ModalityConfig]:
        return self.call_endpoint("get_modality_config", requires_input=False)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self.socket.close(linger=0)
        except Exception:
            pass
        try:
            self.context.term()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

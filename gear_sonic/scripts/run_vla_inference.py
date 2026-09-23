"""
VLA inference runner — NO ROS 2 DEPENDENCY.

Runs an Isaac-GR00T VLA policy against the Sonic whole-body control stack.
All communication uses ZMQ:
  1. Robot state  -> ZMQ SUB on ``g1_debug`` topic (from C++ zmq_output_handler)
  2. Actions out  -> ZMQ PUB (latent protocol v4: motion token + hand joints)
  3. Camera       -> ZMQ/TCP via ComposedCameraClientSensor
  4. Keyboard     -> ZMQ SUB via ZMQKeyboardSubscriber

Uses the Isaac-GR00T PolicyClient (ZMQ REQ/REP) to communicate with a
running PolicyServer.

Keyboard commands (received via ZMQ from the standalone keyboard publisher):
  p  -> pause / resume the policy loop
  k  -> start / stop the C++ control loop
  i  -> blend smoothly to initial pose (or snap if no prior token) and switch to POSE mode
  t  -> change prompt at runtime (publisher sends ``prompt:<text>``)
  [  -> toggle left hand open/closed for initial pose
  ]  -> toggle right hand open/closed for initial pose
  c  -> start recording (handled by data exporter if running)
  s  -> stop recording success (handled by data exporter)
  f  -> stop recording failure (handled by data exporter)

Relay mode (``--relay``, DAgger interventions, g1-vr-teleop #25): the PICO streamer owns the
deploy's input socket (5556) and relays this client's ``pose`` messages while it is in
StreamMode.POLICY. This client then binds its action PUB on ``--action-zmq-port`` (5576), sends
no C++ start/stop commands (k/x only print), and follows ``manager_state.stream_mode`` on 5556:
it yields while the operator intervenes (VR_3PT) and resumes on its own when POLICY mode
returns, blending from the deploy's last encoder token. See gear_sonic/utils/teleop/policy_relay.py.
"""

from dataclasses import dataclass
import queue
import threading
import time

import numpy as np
import tyro
import zmq

from gear_sonic.camera.composed_camera import ComposedCameraClientSensor
from gear_sonic.data.robot_model.instantiation.g1 import instantiate_g1_robot_model
from gear_sonic.utils.data_collection.keyboard_subscriber import (
    DEFAULT_ZMQ_KEYBOARD_PORT,
    ZMQKeyboardSubscriber,
)
from gear_sonic.utils.data_collection.telemetry import Telemetry
from gear_sonic.utils.data_collection.transforms import compute_projected_gravity
from gear_sonic.utils.data_collection.zmq_state_subscriber import ZMQStateSubscriber
from gear_sonic.utils.inference.initial_poses import LATENT_INITIAL_MOTION_TOKEN
from gear_sonic.utils.teleop.policy_relay import (
    DEFAULT_POLICY_ACTION_PORT,
    STREAM_MODE_NAMES,
    RelayArbiter,
    resume_blend_token,
    unpack_pose_message,
)
from gear_sonic.utils.teleop.inspire.inspire_dump import (
    INSPIRE_DUMP_HOST,
    INSPIRE_DUMP_PORT,
    InspireHandStateSubscriber,
    inspire_actual_to_g1_hand_q,
)
from gear_sonic.utils.inference.vla_utils import (
    calculate_latency_compensated_index,
    concat_action,
    prepare_observation_for_eval,
    should_trigger_new_inference,
)
from gear_sonic.utils.teleop.solver.hand.g1_gripper_ik_solver import (
    G1GripperInverseKinematicsSolver,
)
from gear_sonic.utils.teleop.zmq.zmq_planner_sender import (
    build_command_message,
    pack_pose_message,
)


@dataclass
class InferenceConfig:
    """CLI config for the VLA inference runner."""

    # Policy server (Isaac-GR00T PolicyServer)
    host: str = "localhost"
    """The host address of the Isaac-GR00T PolicyServer."""

    port: int = 5550
    """The port of the Isaac-GR00T PolicyServer."""

    # Control
    action_publish_rate: int = 50
    """Rate at which individual actions are published to the C++ control loop (Hz)."""

    action_horizon: int = 40
    """Action horizon of the VLA policy (number of future actions per inference)."""

    rate: float = 1 / 0.4
    """Rate at which we run the forward pass of the VLA policy (Hz)."""

    # Camera
    camera_host: str = "localhost"
    """Camera server host."""

    camera_port: int = 5555
    """Camera server port."""

    # ZMQ: Robot state (from C++ zmq_output_handler, g1_debug topic)
    state_zmq_host: str = "localhost"
    """ZMQ host for robot state (g1_debug topic from C++ deploy)."""

    state_zmq_port: int = 5557
    """ZMQ port for robot state (same socket as robot_config topic)."""

    # ZMQ: Action output (latent actions to C++ control loop)
    action_zmq_host: str = "localhost"
    """ZMQ host for action output (PUB socket)."""

    action_zmq_port: int = 5556
    """ZMQ port for action output."""

    # ZMQ: Keyboard input
    keyboard_zmq_host: str = "localhost"
    """ZMQ host for keyboard input."""

    keyboard_zmq_port: int = DEFAULT_ZMQ_KEYBOARD_PORT
    """ZMQ port for keyboard input."""

    # Embodiment
    embodiment_tag: str = "unitree_g1_sonic"
    """Embodiment tag for policy inference."""

    # Prompt / eval
    prompt: str = "put bottles with red cap in red bottle holder"
    """The language prompt for the VLA policy (default = the restocking training prompt)."""

    # Inspire hands (this rig): real hand state comes from the Inspire bridge dump port,
    # not from the C++ deploy (Dex3-only, streams zeros). 0 disables and uses C++ hand_q.
    inspire_dump_host: str = INSPIRE_DUMP_HOST
    inspire_dump_port: int = INSPIRE_DUMP_PORT
    inspire_state_max_age: float = 0.25
    """Seconds after which an Inspire snapshot is stale -> fall back to C++ hand_q (warns)."""

    initial_motion_token_path: str = ""
    """Optional .npy with a 64-dim motion token for the initial pose (extracted from our
    own demonstrations). Empty = NVIDIA default LATENT_INITIAL_MOTION_TOKEN."""

    # Initial pose
    initial_pose_blend_duration: float = 1.0
    """Duration (seconds) for smooth interpolation to initial pose. The robot
    blends from its current motion token to the initial pose token over this
    period. Set to 0 to snap instantly (no blend)."""

    # Sensor-loss guard (g1-vr-teleop #26)
    sensor_loss_pause_s: float = 1.0
    """Pause the policy when no VALID observation (both camera views present and fresh, robot state
    present) has been seen for this long. The C++ loop keeps holding; no actions are sent, so the
    Inspire bridge opens the hands after its --action-max-age (2 s). 0 disables the guard."""

    auto_resume_after_sensor_loss: bool = False
    """Resume on its own once observations are valid again. Default False: stay paused until the
    operator presses 'p' (2026-09-11 the robot resumed unprompted when the head camera came back)."""

    # Recording (g1-vr-teleop #25)
    exporter_keys: bool = False
    """An episode exporter shares the keys channel (RECORD=1 in tools/vla-inference.sh): c = start /
    stop+save and x = DISCARD are its keys, so x does not stop the C++ loop here. Stop it with O
    in the deploy pane instead."""

    # Relay through the PICO streamer (DAgger interventions, g1-vr-teleop #25)
    relay: bool = False
    """Publish actions for the PICO streamer's POLICY mode instead of driving the deploy directly.
    The streamer (pico_manager_thread_server.py --manager --policy-port <action_zmq_port>) owns
    5556 and relays our ``pose`` messages while in POLICY mode. Here: bind the action PUB on
    ``action_zmq_port`` (use 5576, not the streamer's 5556), send no C++ start/stop commands
    (k/x are the streamer's job), and follow ``manager_state.stream_mode`` on ``manager_zmq_port``:
    yield while the operator intervenes (VR_3PT), resume when POLICY mode returns."""

    manager_zmq_host: str = "localhost"
    manager_zmq_port: int = 5556
    """The streamer's PUB (manager_state topic) in relay mode."""

    manager_timeout_s: float = 2.0
    """Relay mode: no manager_state for this long -> treat as yielded, no auto-resume."""

    resume_blend_s: float = 0.5
    """Relay mode: after an intervention, blend from the deploy's last token (g1_debug
    token_state = what the encoder produced under the operator) to the policy's tokens over this
    many seconds so the hand-back does not jump. 0 = snap."""

    # Debug
    verbose_timing: bool = False
    """Whether to always print timing info (not just when loop is slow)."""


# Max age difference between camera views in one message before the observation is dropped.
MAX_VIEW_SKEW_S = 0.5


def print_green(x):
    print(f"\033[92m{x}\033[0m")


def print_red(x):
    print(f"\033[91m{x}\033[0m", flush=True)


def print_yellow(x):
    print(f"\033[93m{x}\033[0m", flush=True)


class SensorLossGuard:
    """Pause the policy after a sensor outage and keep it paused until the operator resumes (#26).

    Pure logic so it can be unit-tested (tools/tests/test_sensor_loss_guard.py). The inference worker
    calls ``observation_valid(now)`` whenever ``prepare_observation_from_sensors`` returns an
    observation; the control loop calls ``update(now, paused)`` every tick and acts on the event:

    ``"pause"``      no valid observation for more than ``threshold_s`` while running -> stop acting
    ``"recovered"``  observations are valid again but we stay paused (printed once) -> wait for 'p'
    ``"resume"``     same, with ``auto_resume`` -> resume
    ``None``         nothing to do

    Unarmed until the first valid observation (before 'k' there is no robot state at all).
    """

    def __init__(self, threshold_s: float, auto_resume: bool = False):
        self.threshold_s = float(threshold_s)
        self.auto_resume = auto_resume
        self.last_valid: float | None = None
        self.tripped = False
        self.outage_s = 0.0
        self._recovered_reported = False

    def observation_valid(self, now: float) -> None:
        self.last_valid = now

    def operator_resumed(self) -> None:
        """'p' pressed to resume: a still-dead sensor re-trips on the next tick."""
        self.tripped = False

    def update(self, now: float, paused: bool):
        if self.threshold_s <= 0 or self.last_valid is None:
            return None
        age = now - self.last_valid
        if not self.tripped:
            if not paused and age > self.threshold_s:
                self.tripped = True
                self.outage_s = age
                self._recovered_reported = False
                return "pause"
            return None
        if age <= self.threshold_s:
            if self.auto_resume:
                self.tripped = False
                return "resume"
            if not self._recovered_reported:
                self._recovered_reported = True
                return "recovered"
        return None


# ---------------------------------------------------------------------------
# Action packing (latent protocol v4)
# ---------------------------------------------------------------------------


def pack_latent_action_message(
    motion_token: np.ndarray,
    frame_index: np.ndarray,
    left_hand_joints: np.ndarray = None,
    right_hand_joints: np.ndarray = None,
) -> bytes:
    """Pack a single motion-token action into a ZMQ message (Protocol v4).

    Args:
        motion_token: Shape ``[64]`` (flat) or ``[1, 64]``.
        frame_index:  Shape ``[1]``.
        left_hand_joints:  Shape ``[7]`` or ``[1, 7]``, optional.
        right_hand_joints: Shape ``[7]`` or ``[1, 7]``, optional.

    Returns:
        Packed ZMQ message bytes.
    """
    motion_token = np.asarray(motion_token, dtype=np.float32)
    frame_index = np.asarray(frame_index, dtype=np.int64)

    if frame_index.ndim == 0:
        frame_index = np.array([frame_index], dtype=np.int64)
    elif frame_index.shape[0] != 1:
        frame_index = frame_index[:1]

    if motion_token.ndim == 1:
        motion_token = motion_token.reshape(1, -1)

    pose_data = {
        "token_state": motion_token,
        "frame_index": frame_index,
    }

    if left_hand_joints is not None:
        left_hand_joints = np.asarray(left_hand_joints, dtype=np.float32)
        if left_hand_joints.ndim == 1:
            if left_hand_joints.shape[0] != 7:
                raise ValueError(
                    f"left_hand_joints must have shape [7], got {left_hand_joints.shape}"
                )
            left_hand_joints = left_hand_joints.reshape(1, 7)
        pose_data["left_hand_joints"] = left_hand_joints

    if right_hand_joints is not None:
        right_hand_joints = np.asarray(right_hand_joints, dtype=np.float32)
        if right_hand_joints.ndim == 1:
            if right_hand_joints.shape[0] != 7:
                raise ValueError(
                    f"right_hand_joints must have shape [7], got {right_hand_joints.shape}"
                )
            right_hand_joints = right_hand_joints.reshape(1, 7)
        pose_data["right_hand_joints"] = right_hand_joints

    return pack_pose_message(pose_data, topic="pose", version=4)


def get_action_field(action_dict: dict, key: str):
    """Get action field from dict, checking both with and without 'action.' prefix."""
    value = action_dict.get(key)
    if value is not None:
        return value
    value = action_dict.get(f"action.{key}")
    if value is not None:
        return value
    raise AssertionError(
        f"Required action field '{key}' (or 'action.{key}') not found in processed_action. "
        f"Available keys: {list(action_dict.keys())}"
    )


# ---------------------------------------------------------------------------
# Observation / inference helpers
# ---------------------------------------------------------------------------


def prepare_observation_from_sensors(
    camera_subscriber,
    state_subscriber,
    robot_model,
    language_prompt: str,
    log_errors: bool = False,
    video_keys=("ego_view",),
    hand_state_fn=None,
):
    """Read sensors and prepare observation for the VLA policy.

    ``video_keys`` are the camera keys the policy server advertises (queried at
    startup); every one of them must be present in the camera message or the
    observation is skipped. ``hand_state_fn(state_msg) -> (left_q7, right_q7)``
    supplies the 7-DoF hand state (Inspire hands); ``None`` = C++ Dex3 hand_q.

    Returns:
        observation dict, or None if sensor data not yet available.
    """
    camera_msg = camera_subscriber.read()
    if camera_msg is None:
        if log_errors:
            print("[DEBUG] prepare_observation: waiting for camera msg..", flush=True)
        return None

    state_msg = state_subscriber.get_msg()
    if state_msg is None:
        if log_errors:
            print("[DEBUG] prepare_observation: waiting for state msg..", flush=True)
        return None

    images = camera_msg["images"]
    missing = [k for k in video_keys if k not in images]
    if missing:
        if log_errors:
            print(
                f"[DEBUG] prepare_observation: camera message lacks {missing} "
                f"(has {sorted(images.keys())}); skipping this observation",
                flush=True,
            )
        return None
    # All views must be fresh relative to each other: the camera sender keeps re-sending the
    # LAST head_view JPEG if the g1 RTP push dies, so a present key is not proof of a live view.
    stamps = camera_msg.get("timestamps", {})
    if len(video_keys) > 1 and all(k in stamps for k in video_keys):
        newest = max(stamps[k] for k in video_keys)
        stale = {k: newest - stamps[k] for k in video_keys if newest - stamps[k] > MAX_VIEW_SKEW_S}
        if stale:
            if log_errors:
                print(
                    "[DEBUG] prepare_observation: stale camera view(s) "
                    + ", ".join(f"{k} lags {v * 1000:.0f} ms" for k, v in stale.items())
                    + " — skipping this observation (head camera push dead?)",
                    flush=True,
                )
            return None
    video = {k: images[k][np.newaxis, np.newaxis] for k in video_keys}
    primary_key = video_keys[0]

    if hand_state_fn is not None:
        left_hand_q, right_hand_q = hand_state_fn(state_msg)
    else:
        # Dex3 hands: copy index finger data to middle finger (hardware coupling)
        state_msg["left_hand_q"][5] = state_msg["left_hand_q"][3]
        state_msg["left_hand_q"][6] = state_msg["left_hand_q"][4]
        left_hand_q, right_hand_q = state_msg["left_hand_q"], state_msg["right_hand_q"]

    qpos = robot_model.get_configuration_from_actuated_joints(
        body_actuated_joint_values=state_msg["body_q"],
        left_hand_actuated_joint_values=left_hand_q,
        right_hand_actuated_joint_values=right_hand_q,
    )

    observation = {
        "video": video,
        "state": {},
        "language": {
            "annotation.human.task_description": [[language_prompt]],
        },
        "q": np.asarray(qpos, dtype=np.float32)[np.newaxis, np.newaxis],
        "timestamps": camera_msg["timestamps"][primary_key],
    }

    observation = prepare_observation_for_eval(robot_model, observation)

    # Projected gravity for Sonic latent embodiment
    assert "base_quat" in state_msg, "base_quat not found in state_msg"
    base_quat = np.asarray(state_msg["base_quat"], dtype=np.float64)
    assert base_quat.shape == (4,), "base_quat must have shape (4,)"
    projected_gravity = compute_projected_gravity(base_quat)
    observation["state"]["projected_gravity"] = np.asarray(
        projected_gravity, dtype=np.float32
    )[np.newaxis, np.newaxis]

    return observation


def run_policy_inference_and_process(policy, observation, robot_model):
    """Run policy inference via Isaac-GR00T PolicyClient and process results.

    Returns:
        processed_action dict or None on error.
    """
    try:
        action, _info = policy.get_action(observation)

        action.pop("task_progress", None)
        action.pop("action.task_progress", None)

        motion_key = "motion_token" if "motion_token" in action else "action.motion_token"
        if np.abs(action[motion_key]).max() > 1.25:
            print(
                f"[Warning] action['{motion_key}'] max "
                f"({np.abs(action[motion_key]).max():.4f}) > 1.25. "
                "Exceeds action bound, skipping."
            )
            return None

        processed_action = concat_action(robot_model, action)
        return processed_action
    except Exception as e:
        print(f"Error in inference: {e}")
        import traceback

        traceback.print_exc()
        return None


def _inference_worker_loop(
    inference_queue: queue.Queue,
    result_queue: queue.Queue,
    stop_event: threading.Event,
    busy_event: threading.Event,
    prepare_obs_fn,
    inference_fn,
):
    """Persistent worker thread for async inference."""
    while not stop_event.is_set():
        try:
            try:
                inference_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            busy_event.set()
            try:
                observation = prepare_obs_fn()
                if observation is None:
                    print("[DEBUG] Worker thread: Observation is None, skipping", flush=True)
                    continue

                inference_start_time = time.monotonic()
                processed_action = inference_fn(observation)

                if processed_action is not None:
                    try:
                        result_queue.put_nowait((processed_action, inference_start_time))
                    except queue.Full:
                        try:
                            result_queue.get_nowait()
                            result_queue.put_nowait((processed_action, inference_start_time))
                        except queue.Empty:
                            result_queue.put_nowait((processed_action, inference_start_time))
            finally:
                busy_event.clear()
        except Exception as e:
            print(f"Error in inference worker thread: {e}")
            import traceback

            traceback.print_exc()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def _compute_closed_hand_joints(side: str) -> np.ndarray:
    """Compute closed hand joint positions using G1GripperInverseKinematicsSolver."""
    side_str = "left" if side.upper() == "L" else "right"
    solver = G1GripperInverseKinematicsSolver(side=side_str)
    return solver._get_middle_close_q_desired().astype(np.float32)


def main(config: InferenceConfig):
    pause_loop = True

    robot_model = instantiate_g1_robot_model(waist_location="lower_and_upper_body")

    # Isaac-GR00T PolicyClient (the real package needs Python >= 3.12; on this Jetson the
    # .venv_inference is 3.10, so fall back to the vendored wire-compatible client).
    try:
        from gr00t.policy.server_client import PolicyClient
    except ImportError:
        from gear_sonic.utils.inference.gr00t_client import PolicyClient

    n1_policy = PolicyClient(host=config.host, port=config.port)

    print(f"Connecting to PolicyServer at {config.host}:{config.port}...")
    video_keys = ("ego_view",)
    if n1_policy.ping():
        print_green("PolicyServer is reachable.")
        try:
            server_mc = n1_policy.get_modality_config()
            video_keys = tuple(server_mc["video"].modality_keys)
            print_green(f"Policy video keys (from server): {list(video_keys)}")
        except Exception as e:  # noqa: BLE001
            print(f"WARNING: could not query modality config ({e}); assuming {list(video_keys)}")
    else:
        print("WARNING: PolicyServer not reachable. Inference will fail until server is up.")

    # Initial pose token: our own demonstrations' standing pose, if provided.
    global LATENT_INITIAL_MOTION_TOKEN
    if config.initial_motion_token_path:
        tok = np.load(config.initial_motion_token_path).astype(np.float32).reshape(-1)
        assert tok.shape == (64,), f"initial motion token must be 64-dim, got {tok.shape}"
        LATENT_INITIAL_MOTION_TOKEN = tok
        print_green(f"Initial motion token loaded from {config.initial_motion_token_path}")

    # Inspire hand state -> 7-DoF G1 hand joints, exactly as the data exporter recorded it.
    hand_state_fn = None
    if config.inspire_dump_port:
        try:
            inspire_sub = InspireHandStateSubscriber(
                host=config.inspire_dump_host, port=config.inspire_dump_port
            )
        except Exception as e:  # noqa: BLE001
            inspire_sub = None
            print(f"WARNING: Inspire hand-state SUB failed ({e}); using C++ hand_q (zeros!)")
        if inspire_sub is not None:
            hand_open_q, hand_closed_q, hand_thumb_slots, hand_finger_slots = {}, {}, {}, {}
            for side in ("left", "right"):
                solver = G1GripperInverseKinematicsSolver(side=side)
                fingertips = np.zeros([25, 4, 4])
                fingertips[4, 0, 3] = 1.0  # thumb tip (generate_finger_data)
                hand_open_q[side] = np.asarray(solver({"position": fingertips}), dtype=np.float64).reshape(-1)
                fingertips[14, 0, 3] = 1.0  # middle tip -> closed grip
                hand_closed_q[side] = np.asarray(solver({"position": fingertips}), dtype=np.float64).reshape(-1)
                hand_idx = list(robot_model.get_hand_actuated_joint_indices(side))
                slot_names = [robot_model.joint_names[i] for i in hand_idx]
                hand_thumb_slots[side] = tuple(k for k, nm in enumerate(slot_names) if "thumb" in nm)
                hand_finger_slots[side] = tuple(k for k, nm in enumerate(slot_names) if "thumb" not in nm)
            inspire_warn = {"t": 0.0}

            def hand_state_fn(state_msg, _sub=inspire_sub):
                # The bridge publishes ~180 msg/s and get_actual() drains only 64 per call; at
                # our 2.5 Hz inference rate that would lag by seconds. Drain fully first.
                _sub.poll(max_msgs=100000)
                out = []
                for side, key in (("left", "left_hand_q"), ("right", "right_hand_q")):
                    actual, age = _sub.get_actual(side, max_age_s=config.inspire_state_max_age)
                    if actual is None:
                        now = time.time()
                        if now - inspire_warn["t"] >= 5.0:
                            inspire_warn["t"] = now
                            why = "no snapshot yet" if age is None else f"stale ({age * 1000:.0f} ms)"
                            print(f"[Inspire] {side} hand state unavailable ({why}); using C++ hand_q", flush=True)
                        out.append(np.asarray(state_msg[key], dtype=np.float32))
                    else:
                        out.append(
                            inspire_actual_to_g1_hand_q(
                                actual, hand_open_q[side], hand_closed_q[side],
                                thumb_slots=hand_thumb_slots[side], finger_slots=hand_finger_slots[side],
                            ).astype(np.float32)
                        )
                return out[0], out[1]

            print_green("Hand state source: Inspire bridge (dump port "
                        f"{config.inspire_dump_host}:{config.inspire_dump_port})")

    state_subscriber = ZMQStateSubscriber(
        host=config.state_zmq_host,
        port=config.state_zmq_port,
    )

    camera_subscriber = ComposedCameraClientSensor(
        server_ip=config.camera_host, port=config.camera_port
    )

    if config.relay and config.action_zmq_port == 5556:
        raise SystemExit(
            "--relay: 5556 is the PICO streamer's socket; bind the action PUB elsewhere "
            f"(--action-zmq-port {DEFAULT_POLICY_ACTION_PORT}, the streamer's --policy-port)"
        )

    zmq_context = zmq.Context()
    zmq_socket = zmq_context.socket(zmq.PUB)
    zmq_socket.bind(f"tcp://{config.action_zmq_host}:{config.action_zmq_port}")
    time.sleep(0.1)
    print_green(
        f"ZMQ action socket bound to tcp://{config.action_zmq_host}:{config.action_zmq_port}"
        + (" (RELAY: the PICO streamer forwards these in POLICY mode)" if config.relay else "")
    )
    print_green(f"Using embodiment tag: {config.embodiment_tag}")

    keyboard_listener = ZMQKeyboardSubscriber(
        port=config.keyboard_zmq_port, host=config.keyboard_zmq_host
    )

    # Relay mode: follow the streamer's stream_mode, and read the deploy's last token for the
    # resume blend on a socket of our own (the worker thread owns state_subscriber's).
    manager_sub = None
    arbiter = None
    token_state_sub = None
    if config.relay:
        manager_sub = zmq_context.socket(zmq.SUB)
        manager_sub.setsockopt(zmq.SUBSCRIBE, b"manager_state")
        manager_sub.setsockopt(zmq.RCVHWM, 16)
        manager_sub.connect(f"tcp://{config.manager_zmq_host}:{config.manager_zmq_port}")
        arbiter = RelayArbiter(manager_timeout_s=config.manager_timeout_s)
        token_state_sub = ZMQStateSubscriber(host=config.state_zmq_host, port=config.state_zmq_port)
        print_green(
            f"Relay mode: following manager_state on tcp://{config.manager_zmq_host}:{config.manager_zmq_port}; "
            "k/x do nothing here (the streamer starts/stops the deploy); the policy yields during "
            f"interventions and resumes by itself (blend {config.resume_blend_s:.1f} s)"
        )

    def read_stream_mode():
        """Latest manager_state.stream_mode from the streamer, or None if nothing new."""
        mode = None
        for _ in range(32):
            try:
                raw = manager_sub.recv(zmq.NOBLOCK)
            except zmq.Again:
                break
            try:
                data = unpack_pose_message(raw, topic="manager_state")
            except Exception:  # noqa: BLE001
                continue
            if "stream_mode" in data:
                mode = int(np.asarray(data["stream_mode"]).flat[0])
        return mode

    telemetry = Telemetry(window_size=100)

    loop_rate = config.action_publish_rate
    loop_period = 1.0 / loop_rate

    # Track C++ control loop state
    cpp_loop_running = False
    cpp_mode = "OFF"  # "OFF", "PLANNER", or "POSE"

    sensor_guard = SensorLossGuard(config.sensor_loss_pause_s, config.auto_resume_after_sensor_loss)

    # Track initial pose hand states
    initial_pose_left_hand_closed = False
    initial_pose_right_hand_closed = False

    def publish_initial_pose():
        """Publish initial pose command to move robot to starting position."""
        print("Moving to initial pose")
        left_hand = (
            _compute_closed_hand_joints("L")
            if initial_pose_left_hand_closed
            else np.zeros(7, dtype=np.float32)
        )
        right_hand = (
            _compute_closed_hand_joints("R")
            if initial_pose_right_hand_closed
            else np.zeros(7, dtype=np.float32)
        )
        zmq_message = pack_latent_action_message(
            motion_token=LATENT_INITIAL_MOTION_TOKEN,
            frame_index=np.array([0], dtype=np.int64),
            left_hand_joints=left_hand,
            right_hand_joints=right_hand,
        )
        zmq_socket.send(zmq_message)
        print_green("Sent latent initial pose via ZMQ")
        time.sleep(1.0)
        print("Initial pose done.")

    def blend_to_initial_pose(duration_s: float) -> bool:
        """Smoothly interpolate from the last sent motion token to the initial pose.

        Linearly blends over ``duration_s`` seconds at the action publish rate,
        sending intermediate tokens each loop iteration. Returns True if blend
        was performed, False if skipped (no previous token available).
        """
        nonlocal last_sent_motion_token
        if last_sent_motion_token is None:
            print("No previous motion token — snapping to initial pose instead.")
            publish_initial_pose()
            return False

        start_token = last_sent_motion_token.copy()
        target_token = LATENT_INITIAL_MOTION_TOKEN.copy()
        num_steps = max(1, round(config.action_publish_rate * duration_s))
        step_period = 1.0 / config.action_publish_rate

        left_hand = (
            _compute_closed_hand_joints("L")
            if initial_pose_left_hand_closed
            else np.zeros(7, dtype=np.float32)
        )
        right_hand = (
            _compute_closed_hand_joints("R")
            if initial_pose_right_hand_closed
            else np.zeros(7, dtype=np.float32)
        )

        print(
            f"Blending to initial pose over {duration_s:.2f}s "
            f"({num_steps} steps at {config.action_publish_rate} Hz)"
        )

        for step in range(num_steps):
            t_step_start = time.monotonic()
            alpha = (step + 1) / num_steps
            blended_token = ((1.0 - alpha) * start_token + alpha * target_token).astype(
                np.float32
            )
            zmq_message = pack_latent_action_message(
                motion_token=blended_token,
                frame_index=np.array([0], dtype=np.int64),
                left_hand_joints=left_hand,
                right_hand_joints=right_hand,
            )
            zmq_socket.send(zmq_message)
            last_sent_motion_token = blended_token.copy()

            elapsed = time.monotonic() - t_step_start
            remaining = step_period - elapsed
            if remaining > 0:
                time.sleep(remaining)

        print_green("Initial pose blend complete.")
        return True

    def send_cpp_control_command(start: bool, planner: bool = False):
        """Send C++ control loop start/stop commands via ZMQ."""
        nonlocal cpp_loop_running, cpp_mode
        if config.relay:
            print_yellow(
                "Relay mode: the PICO streamer owns the deploy - start/stop it there "
                "(A+B+X+Y), enter POLICY mode with g; nothing sent."
            )
            return False
        try:
            cmd_msg = build_command_message(start=start, stop=not start, planner=planner)
            zmq_socket.send(cmd_msg)
            time.sleep(0.01)
            action_str = "start" if start else "stop"
            mode_str = "planner" if planner else "pose"
            cpp_loop_running = start
            if start:
                cpp_mode = "PLANNER" if planner else "POSE"
            else:
                cpp_mode = "OFF"
            print_green(f"Sent ZMQ command: {action_str} control loop ({mode_str} mode)")
            return True
        except Exception as e:
            action_str = "start" if start else "stop"
            print(f"Warning: Failed to send {action_str} command message: {e}")
            return False

    # Async inference state
    cached_action_chunk = None
    action_chunk_index = 0
    last_inference_time = 0.0
    inference_interval = 1.0 / config.rate

    zmq_frame_counter = 0
    last_sent_motion_token: np.ndarray | None = None

    # Resume blend after an intervention (relay mode)
    blend_start_token: np.ndarray | None = None
    blend_step = 0
    blend_steps = max(0, round(config.action_publish_rate * config.resume_blend_s))

    PROMPT_MSG_PREFIX = "prompt:"

    def check_keyboard_input():
        nonlocal pause_loop, cpp_loop_running, cpp_mode
        nonlocal initial_pose_left_hand_closed, initial_pose_right_hand_closed
        nonlocal cached_action_chunk, action_chunk_index, last_inference_time
        nonlocal zmq_frame_counter, last_sent_motion_token

        key = keyboard_listener.read_msg()
        if key is None:
            return

        if key.startswith(PROMPT_MSG_PREFIX):
            new_prompt = key[len(PROMPT_MSG_PREFIX):]
            if new_prompt:
                old_prompt = language_prompt_ref[0]
                language_prompt_ref[0] = new_prompt
                print_green(f'Inference prompt changed: "{old_prompt}" -> "{new_prompt}"')
            else:
                print("Received empty prompt change -- ignoring.")
            return

        if key == "c":
            print("Keyboard: 'c' (start recording -- handled by data exporter)")
        elif key == "s":
            print("Keyboard: 's' (stop recording success -- handled by data exporter)")
        elif key == "f":
            print("Keyboard: 'f' (stop recording failure -- handled by data exporter)")
        elif key == "i":
            if config.relay and arbiter.yielded:
                print_yellow(
                    f"Relay mode: streamer is in {STREAM_MODE_NAMES.get(arbiter.mode, arbiter.mode)}, "
                    "not POLICY - the initial pose tokens are dropped there (g in the keys pane first)"
                )
            if cpp_loop_running and cpp_mode == "PLANNER":
                if send_cpp_control_command(start=True, planner=False):
                    print("Switched to POSE mode (from PLANNER mode)")
                else:
                    print("Warning: Failed to switch to POSE mode")
            elif not cpp_loop_running:
                print("Note: C++ loop not running - press 'k' to start")

            pause_loop = True
            if config.initial_pose_blend_duration > 0 and last_sent_motion_token is not None:
                blend_to_initial_pose(config.initial_pose_blend_duration)
            else:
                publish_initial_pose()

            zmq_frame_counter = 0
            cached_action_chunk = None
            action_chunk_index = 0
            print("Cleared cached action chunk, reset frame counter")
        elif key == "p":
            if config.relay and arbiter.yielded:
                # Not our robot right now: p only decides what happens when POLICY mode returns.
                arbiter.set_resume_intent(not arbiter.was_running)
                print_yellow(
                    "Relay mode: yielded to the streamer "
                    f"({STREAM_MODE_NAMES.get(arbiter.mode, arbiter.mode)}); the policy will "
                    f"{'RESUME' if arbiter.was_running else 'stay PAUSED'} when POLICY mode returns"
                )
                return
            pause_loop = not pause_loop
            print(f"{'Paused' if pause_loop else 'Resumed'} policy loop")
            if pause_loop:
                print("Policy loop paused (C++ loop still running - press 'k' to stop)")
            else:
                sensor_guard.operator_resumed()
                print("Policy loop resumed")
        elif key == "k":
            # Stateless on purpose: the C++ deploy may have been restarted behind our back (it
            # exits on any error), so a toggle keyed on our own bookkeeping sends STOP when the
            # operator means START. 'k' always starts (PLANNER mode); 'x' always stops.
            if cpp_loop_running:
                print(f"Note: we believed the C++ loop was already running ({cpp_mode}); re-sending START anyway")
            print("Starting C++ control loop in PLANNER mode...")
            if send_cpp_control_command(start=True, planner=True):
                print("Started C++ control loop in PLANNER mode")
                print("Press 'i' to send initial pose and switch to POSE mode")
                if pause_loop:
                    print("Note: Policy loop is paused - press 'p' to resume")
        elif key == "x" and config.exporter_keys:
            print("Keyboard: 'x' (discard episode -- handled by data exporter; O in the deploy pane stops the C++ loop)")
        elif key == "x":
            current_planner = cpp_mode == "PLANNER"
            print(f"Stopping C++ control loop (from {cpp_mode} mode)...")
            pause_loop = True
            if send_cpp_control_command(start=False, planner=current_planner):
                print("Stopped C++ control loop (policy paused)")
        elif key == "[":
            initial_pose_left_hand_closed = not initial_pose_left_hand_closed
            print(
                f"Initial pose left hand: {'closed' if initial_pose_left_hand_closed else 'open'}"
            )
        elif key == "]":
            initial_pose_right_hand_closed = not initial_pose_right_hand_closed
            print(
                f"Initial pose right hand: "
                f"{'closed' if initial_pose_right_hand_closed else 'open'}"
            )

    # Mutable prompt container (single-writer from keyboard, single-reader from inference)
    language_prompt_ref: list[str] = [config.prompt]
    print(f"Starting the policy loop with language prompt: {language_prompt_ref[0]}")

    inference_queue = queue.Queue(maxsize=1)
    result_queue = queue.Queue(maxsize=1)
    inference_stop_event = threading.Event()
    inference_busy_event = threading.Event()

    if config.sensor_loss_pause_s > 0:
        print_green(
            f"Sensor-loss guard: pause after {config.sensor_loss_pause_s:.1f} s without a valid observation"
            + (", auto-resume ON" if config.auto_resume_after_sensor_loss else ", resume with 'p'")
        )

    def prepare_obs_with_guard():
        obs = prepare_observation_from_sensors(
            camera_subscriber=camera_subscriber,
            state_subscriber=state_subscriber,
            robot_model=robot_model,
            language_prompt=language_prompt_ref[0],
            log_errors=True,
            video_keys=video_keys,
            hand_state_fn=hand_state_fn,
        )
        if obs is not None:
            sensor_guard.observation_valid(time.monotonic())
        return obs

    inference_worker_thread = threading.Thread(
        target=_inference_worker_loop,
        args=(
            inference_queue,
            result_queue,
            inference_stop_event,
            inference_busy_event,
            prepare_obs_with_guard,
            lambda obs: run_policy_inference_and_process(
                policy=n1_policy,
                observation=obs,
                robot_model=robot_model,
            ),
        ),
        daemon=True,
    )
    inference_worker_thread.start()

    try:
        while True:
            t_start = time.monotonic()
            check_keyboard_input()

            if arbiter is not None:
                relay_event = arbiter.update(read_stream_mode(), time.monotonic(), pause_loop)
                if relay_event == "yield":
                    was_running = arbiter.was_running
                    pause_loop = True
                    cached_action_chunk = None  # never replay a pre-intervention chunk
                    blend_start_token = None
                    mode_name = STREAM_MODE_NAMES.get(arbiter.mode, arbiter.mode)
                    if was_running:
                        print_yellow(
                            f"INTERVENTION: streamer switched to {mode_name} - policy yielded, no actions "
                            "sent; resumes by itself when POLICY mode returns (p while yielded flips that)."
                        )
                    else:
                        print_yellow(f"Streamer in {mode_name}: waiting for POLICY mode (g in the keys pane).")
                elif relay_event == "resume":
                    pause_loop = False
                    cached_action_chunk = None
                    sensor_guard.operator_resumed()
                    blend_start_token = None
                    if blend_steps > 0:
                        st = token_state_sub.get_msg(clear=True)
                        tok = None if st is None else st.get("token_state")
                        if tok is not None and np.asarray(tok).size == 64:
                            blend_start_token = np.asarray(tok, dtype=np.float32).reshape(64)
                            blend_step = 0
                    print_green(
                        "POLICY mode is back - resuming with a fresh chunk"
                        + (f", blending from the deploy's last token over {config.resume_blend_s:.1f} s"
                           if blend_start_token is not None else " (no token_state to blend from: snap)")
                    )
                elif relay_event == "handover":
                    print_yellow("Streamer in POLICY mode: our tokens reach the deploy now. i = initial pose, p = run.")
                elif relay_event == "lost":
                    pause_loop = True
                    cached_action_chunk = None
                    print_red(
                        f"No manager_state from the streamer for {config.manager_timeout_s:.0f} s - policy PAUSED. "
                        "Is pico_manager_thread_server.py --policy-port running? p resumes once it is back."
                    )

            # Consume result first so last_inference_time is fresh before trigger check
            try:
                processed_action, inference_start_time = result_queue.get_nowait()
                inference_delay = time.monotonic() - inference_start_time
                action_chunk_index = calculate_latency_compensated_index(
                    inference_delay, config.action_publish_rate, config.action_horizon
                )
                cached_action_chunk = processed_action
                last_inference_time = time.monotonic()
                print_green(
                    f'New action chunk (prompt: "{language_prompt_ref[0]}", '
                    f"latency: {inference_delay:.3f}s)"
                )
            except queue.Empty:
                pass

            worker_is_busy = inference_busy_event.is_set()
            should_start = should_trigger_new_inference(
                cached_chunk_exists=(cached_action_chunk is not None),
                inference_thread_running=worker_is_busy,
                time_since_last_inference=(time.monotonic() - last_inference_time),
                inference_interval=inference_interval,
            )

            if should_start:
                try:
                    inference_queue.put_nowait(None)
                except queue.Full:
                    pass

            guard_event = sensor_guard.update(time.monotonic(), pause_loop)
            if guard_event == "pause":
                pause_loop = True
                cached_action_chunk = None  # never replay a pre-outage chunk after resume
                print_red(
                    f"SENSOR LOSS: no valid observation for {sensor_guard.outage_s:.1f} s (camera view missing "
                    "or stale, or no robot state) -> policy PAUSED. No actions are sent; the Inspire bridge opens "
                    "the hands after 2 s. Fix the sensor (head camera: unplug 10 s, replug), then press 'p'."
                )
            elif guard_event == "recovered":
                print_yellow("Observations valid again - policy still PAUSED by the sensor-loss guard. Press 'p' to resume.")
            elif guard_event == "resume":
                pause_loop = False
                print_yellow("Observations valid again - resuming (auto_resume_after_sensor_loss).")

            if pause_loop:
                print("Pausing...", end="", flush=True)
                time.sleep(0.2)
                print(".", end="", flush=True)
                continue

            with telemetry.timer("total_loop"):
                if cached_action_chunk is None:
                    print("[DEBUG] No cached chunk yet, waiting...", flush=True)
                    _sleep_remaining(t_start, loop_period)
                    continue

                processed_action = cached_action_chunk

                if processed_action is None or not processed_action:
                    print("[DEBUG] processed_action is None or empty, skipping", flush=True)
                else:
                    motion_token = np.asarray(
                        get_action_field(processed_action, "motion_token"),
                        dtype=np.float32,
                    )
                    left_hand_joints = np.asarray(
                        get_action_field(processed_action, "left_hand_joints"),
                        dtype=np.float32,
                    )
                    right_hand_joints = np.asarray(
                        get_action_field(processed_action, "right_hand_joints"),
                        dtype=np.float32,
                    )

                    # Action arrays arrive as (B, T, D) from the model.
                    # Squeeze batch dim to get (T, D), then index by time step.
                    if motion_token.ndim == 3:
                        motion_token = motion_token[0]
                    if left_hand_joints.ndim == 3:
                        left_hand_joints = left_hand_joints[0]
                    if right_hand_joints.ndim == 3:
                        right_hand_joints = right_hand_joints[0]

                    horizon = motion_token.shape[0] if motion_token.ndim == 2 else 1
                    current_idx = min(action_chunk_index, horizon - 1)

                    if motion_token.ndim == 2:
                        motion_token = motion_token[current_idx]
                    if left_hand_joints.ndim == 2:
                        left_hand_joints = left_hand_joints[current_idx]
                    if right_hand_joints.ndim == 2:
                        right_hand_joints = right_hand_joints[current_idx]

                    if blend_start_token is not None:
                        blend_step += 1
                        motion_token = resume_blend_token(
                            blend_start_token, motion_token, blend_step, blend_steps
                        )
                        if blend_step >= blend_steps:
                            blend_start_token = None

                    frame_index = np.array([zmq_frame_counter], dtype=np.int64)
                    zmq_frame_counter += 1

                    zmq_message = pack_latent_action_message(
                        motion_token,
                        frame_index,
                        left_hand_joints=left_hand_joints,
                        right_hand_joints=right_hand_joints,
                    )
                    zmq_socket.send(zmq_message)
                    last_sent_motion_token = motion_token.copy()
                    if zmq_frame_counter % 50 == 0:
                        print_green(
                            f"ZMQ: Sent latent action - "
                            f"frame: {frame_index[0]}, "
                            f"token shape: {motion_token.shape}"
                        )

                action_chunk_index = min(action_chunk_index + 1, config.action_horizon - 1)

            end_time = time.monotonic()

            if config.verbose_timing:
                telemetry.log_timing_info(context="VLA Inference Loop", threshold=0.0)
            elif (end_time - t_start) > (1 / config.rate):
                telemetry.log_timing_info(
                    context="VLA Inference Loop Missed", threshold=0.001
                )

            _sleep_remaining(t_start, loop_period)

    except KeyboardInterrupt:
        print("VLA inference loop terminated by user")

    finally:
        inference_stop_event.set()
        inference_worker_thread.join(timeout=1.0)
        zmq_socket.close()
        if manager_sub is not None:
            manager_sub.close()
        if token_state_sub is not None:
            token_state_sub.close()
        zmq_context.term()
        state_subscriber.close()
        keyboard_listener.close()
        print("Shutdown complete.")


def _sleep_remaining(t_start: float, loop_period: float):
    """Sleep for the remainder of the loop period."""
    elapsed = time.monotonic() - t_start
    remaining = loop_period - elapsed
    if remaining > 0:
        time.sleep(remaining)


if __name__ == "__main__":
    config = tyro.cli(InferenceConfig)
    main(config)

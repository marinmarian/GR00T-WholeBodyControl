"""Let the decoupled-WBC data exporter record from the SONIC C++ deploy.

``run_g1_data_exporter.py`` was written against the Python control loop
(``run_g1_control_loop.py``). The SONIC stack replaces that loop with the C++
``g1_deploy_onnx_ref`` binary, which differs in two ways handled here:

* **Robot config.** The Python loop serves it as a ``std_srvs/Trigger`` service
  on ``WBCPolicy/robot_config``. The C++ ROS2 output handler publishes it once
  on a *topic* of the same name (``ByteMultiArray`` msgpack, transient_local
  QoS) and never creates the service, so the old ``ROSServiceClient`` waited
  forever. ``wait_for_robot_config`` accepts whichever source shows up first.

* **State schema.** The Python loop publishes ``q``, ``wrist_pose``, ``action``,
  ``action.eef``, ``navigate_command``, ``base_height_command`` and
  ``timestamps.proprio`` on ``G1Env/env_state_act``. The C++ handler publishes
  ``body_q`` / ``last_action`` (29 values, MuJoCo order, offsets already
  applied), 7-DoF ``left/right_hand_q`` and ``last_left/right_hand_action``
  arrays, and ``ros_timestamp``. ``adapt_cpp_state_msg`` rebuilds the former
  from the latter with the robot model - the same body-order assumption that
  ``gear_sonic/scripts/run_data_exporter.py`` makes - and takes both wrist
  poses from forward kinematics of ``left/right_wrist_yaw_link``.

  The C++ handler does not publish the locomotion command, so
  ``navigate_command`` and ``base_height_command`` are recorded at their
  defaults from ``decoupled_wbc.control.main.constants``.
"""

import base64
import time
from typing import Optional

import msgpack
import msgpack_numpy as mnp
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import ByteMultiArray
from std_srvs.srv import Trigger

from decoupled_wbc.control.main.constants import DEFAULT_BASE_HEIGHT, DEFAULT_NAV_CMD

# Any wall-clock timestamp is far above this; ROS time of 0 or a monotonic
# clock would fall below it.
_MIN_WALL_CLOCK_SEC = 1e9


def _decode_byte_multi_array(data) -> dict:
    """Decode the msgpack payload of a ``ByteMultiArray`` (list of 1-byte objects)."""
    return msgpack.unpackb(bytes([b for chunk in data for b in chunk]), object_hook=mnp.decode)


def wait_for_robot_config(
    node,
    name: str,
    timeout_sec: Optional[float] = None,
    poll_sec: float = 0.2,
    log_every_sec: float = 5.0,
) -> dict:
    """Return the robot config from the C++ topic or the Python service, whichever
    appears first.

    ``name`` is both the topic and the service name (``WBCPolicy/robot_config``).
    Waits indefinitely unless ``timeout_sec`` is given, in which case an empty
    dict is returned after the timeout. Must be called before ``node`` is
    handed to a spinning executor.
    """
    received = {}

    def _on_topic(msg):
        received["config"] = _decode_byte_multi_array(msg.data)

    qos = QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    sub = node.create_subscription(ByteMultiArray, name, _on_topic, qos)
    client = node.create_client(Trigger, name)
    t_start = time.monotonic()
    t_last_log = t_start
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=poll_sec)

            if "config" in received:
                config = received["config"]
                print(f"[robot_config] received from topic {name} ({len(config)} fields, C++ deploy)")
                return config

            if client.service_is_ready():
                future = client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
                result = future.result()
                if result is not None and result.success:
                    config = msgpack.unpackb(
                        base64.b64decode(result.message.encode("ascii")), object_hook=mnp.decode
                    )
                    print(f"[robot_config] received from service {name} ({len(config)} fields, Python control loop)")
                    return config
                print("[robot_config] service call returned no config, retrying")

            now = time.monotonic()
            if timeout_sec is not None and now - t_start > timeout_sec:
                print(f"[robot_config] WARNING: nothing on {name} after {timeout_sec:.0f}s; recording with an empty config")
                return {}
            if now - t_last_log >= log_every_sec:
                print(
                    f"[robot_config] waiting for {name} (topic from g1_deploy_onnx_ref "
                    "or service from run_g1_control_loop) - is the deploy running?"
                )
                t_last_log = now
    finally:
        node.destroy_subscription(sub)
        node.destroy_client(client)
    return {}


def is_cpp_state_msg(msg) -> bool:
    """True for a ``G1Env/env_state_act`` message from the C++ ROS2 output handler."""
    return isinstance(msg, dict) and "body_q" in msg and "q" not in msg


def _full_configuration(robot_model, body_values, left_hand_values, right_hand_values) -> np.ndarray:
    body = np.asarray(body_values, dtype=np.float64).reshape(-1)
    n_body = len(robot_model.get_body_actuated_joint_indices())
    if body.shape[0] != n_body:
        raise ValueError(f"C++ state has {body.shape[0]} body joints, robot model expects {n_body}")

    kwargs = {}
    for side, values in (("left", left_hand_values), ("right", right_hand_values)):
        if values is None:
            continue
        hand = np.asarray(values, dtype=np.float64).reshape(-1)
        if hand.shape[0] == len(robot_model.get_hand_actuated_joint_indices(side)):
            kwargs[f"{side}_hand_actuated_joint_values"] = hand
    return robot_model.get_configuration_from_actuated_joints(body_actuated_joint_values=body, **kwargs)


def _wrist_poses(robot_model, q: np.ndarray) -> np.ndarray:
    """[left xyz, left wxyz, right xyz, right wxyz] via FK, matching G1Env.get_eef_obs."""
    robot_model.cache_forward_kinematics(q)
    parts = []
    for side in ("left", "right"):
        placement = robot_model.frame_placement(robot_model.supplemental_info.hand_frame_names[side])
        quat = R.from_matrix(placement.rotation).as_quat(scalar_first=True)
        parts.append(np.concatenate([placement.translation[:3], quat]))
    return np.concatenate(parts)


def adapt_cpp_state_msg(msg, robot_model, receive_time: Optional[float] = None):
    """Return ``msg`` with the Python control-loop keys added when it came from the
    C++ deploy; messages that already carry ``q`` pass through untouched."""
    if not is_cpp_state_msg(msg):
        return msg
    if robot_model is None:
        raise ValueError("C++ state message received but no robot model given to translate it")

    q = _full_configuration(robot_model, msg["body_q"], msg.get("left_hand_q"), msg.get("right_hand_q"))
    action = _full_configuration(
        robot_model,
        msg["last_action"],
        msg.get("last_left_hand_action"),
        msg.get("last_right_hand_action"),
    )
    wrist_pose = _wrist_poses(robot_model, q)
    action_eef = _wrist_poses(robot_model, action)

    stamp = float(msg.get("ros_timestamp") or 0.0)
    if stamp < _MIN_WALL_CLOCK_SEC:
        stamp = receive_time if receive_time is not None else time.time()

    adapted = dict(msg)
    adapted.update(
        {
            "q": q,
            "wrist_pose": wrist_pose,
            "action": action,
            "action.eef": action_eef,
            "navigate_command": list(DEFAULT_NAV_CMD),
            "base_height_command": DEFAULT_BASE_HEIGHT,
            "timestamps": {"main_loop": stamp, "proprio": stamp},
        }
    )
    return adapted

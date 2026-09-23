"""run_data_exporter.py in the streamer's POLICY mode (DAgger, g1-vr-teleop #25).

Relayed VLA actions (protocol v4 `pose` messages with token_state) supply teleop.*_hand_joints
while stream_mode is 6; stale/absent actions record open hands; SMPL modes are untouched; the
per-episode stream-mode list feeds the save-time summary. Needs the exporter's imports
(lerobot etc.), i.e. the wbc-marin container venv:

    ~/wbc-marin-exec.sh python tools/tests/test_exporter_policy_mode.py
"""
import time
from types import SimpleNamespace

import numpy as np

from gear_sonic.scripts.run_data_exporter import GrootDataCollector
from gear_sonic.utils.data_collection.episode_state import EpisodeState
from gear_sonic.utils.data_collection.runtime_prompt import RuntimePrompt
from gear_sonic.utils.teleop.policy_relay import POLICY_STREAM_MODE, stream_mode_summary
from gear_sonic.utils.teleop.zmq.zmq_planner_sender import pack_pose_message

c = GrootDataCollector.__new__(GrootDataCollector)
c.text_to_speech = None
c.frequency = 50
c.upload_bucket_path = None
c._upload_threads = []
c._initial_yaw = None
c._manager_toggle_dc = False
c._manager_toggle_da = False
c._episode_state = EpisodeState()
c._keyboard_listener = SimpleNamespace(read_msg=lambda: None)
c.data_exporter = SimpleNamespace(task="demo", episode_buffer={"episode_index": 0, "size": 0})
c._runtime_prompt = RuntimePrompt("demo")
c.sonic_timing_monitor = SimpleNamespace(reset=lambda: None, log_time_delta=lambda *_: None, failure_count=0)
c.latest_sonic_msg = None
c.latest_planner_msg = None
c.latest_policy_msg = None
c._episode_stream_modes = []
c.current_stream_mode = 0
c._manager_state_seen = False
c._policy_source_announced = False

left = np.array([0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3], dtype=np.float32)
right = np.zeros(7, dtype=np.float32)
raw_v4 = pack_pose_message({
    "token_state": np.full((1, 64), 0.5, dtype=np.float32),
    "frame_index": np.array([42], dtype=np.int64),
    "left_hand_joints": left.reshape(1, 7),
    "right_hand_joints": right.reshape(1, 7),
}, topic="pose", version=4)

# a v4 message is recognised and never mistaken for an SMPL pose
c._handle_pose_message(raw_v4)
assert c.latest_sonic_msg is None
assert c.latest_policy_msg is not None and int(c.latest_policy_msg["frame_index"][0]) == 42
assert np.allclose(c.latest_policy_msg["left_hand_joints"], left)
# plain closed-loop stack (RECORD=1, no streamer): the token source labels the frames POLICY
assert c.current_stream_mode == POLICY_STREAM_MODE
# ... but a streamer's manager_state, once seen, is the authority (its POLICY/VR_3PT switches)
c._handle_manager_state(pack_pose_message({"stream_mode": np.array([5], dtype=np.int32)}, topic="manager_state"))
assert c.current_stream_mode == 5 and c._manager_state_seen
c._handle_pose_message(raw_v4)
assert c.current_stream_mode == 5

# POLICY mode: hand joints come from the relayed action
c.current_stream_mode = POLICY_STREAM_MODE
fd = {"teleop.delta_heading": np.zeros(1)}
c._add_sonic_pose_features(fd)
assert int(fd["teleop.stream_mode"][0]) == POLICY_STREAM_MODE
assert np.allclose(fd["teleop.left_hand_joints"], left) and np.allclose(fd["teleop.right_hand_joints"], right)
assert not np.any(fd["teleop.smpl_pose"]) and not np.any(fd["teleop.smpl_joints"])
assert int(fd["teleop.planner_mode"][0]) == 0
# a stale action (policy paused) records open hands, not the last grasp
c.latest_policy_msg["receive_timestamp"] = time.time() - 1.0
fd = {"teleop.delta_heading": np.zeros(1)}
c._add_sonic_pose_features(fd)
assert not np.any(fd["teleop.left_hand_joints"])
# a stale PLANNER message from before POLICY mode must not leak into policy frames either
c.latest_planner_msg = {"left_hand_joints": np.ones(7, np.float32), "right_hand_joints": np.ones(7, np.float32),
                        "planner_mode": 2, "planner_movement": np.zeros(3, np.float32),
                        "planner_facing": np.array([1, 0, 0], np.float32), "planner_speed": -1.0,
                        "planner_height": -1.0, "vr_3pt_position": None, "vr_3pt_orientation": None,
                        "receive_timestamp": time.time()}
fd = {"teleop.delta_heading": np.zeros(1)}
c._add_sonic_pose_features(fd)
assert not np.any(fd["teleop.left_hand_joints"])
# intervention (VR_3PT, mode 5): planner path as before -> planner hand joints
c.current_stream_mode = 5
fd = {"teleop.delta_heading": np.zeros(1)}
c._add_sonic_pose_features(fd)
assert np.all(fd["teleop.left_hand_joints"] == 1.0) and int(fd["teleop.planner_mode"][0]) == 2
# the per-episode mode list drives the summary
assert c._episode_stream_modes == [6, 6, 6, 5]
s = stream_mode_summary(c._episode_stream_modes)
assert s["policy_frames"] == 3 and s["intervention_frames"] == 1 and s["intervention_segments"] == 1
# starting an episode resets it; saving prints the summary and resets it
c._episode_state.change_state()          # IDLE -> RECORDING via 'c' path
c.current_stream_mode = POLICY_STREAM_MODE
c._keyboard_listener = SimpleNamespace(read_msg=iter(["c"]).__next__)
c._episode_stream_modes = [1, 2, 3]
c._episode_state.reset_state()
c._check_recording_commands()            # 'c' -> RECORDING, list cleared
assert c._episode_state.get_state() == c._episode_state.RECORDING and c._episode_stream_modes == []
print("EXPORTER_POLICY_MODE_TEST_OK")

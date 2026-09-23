"""FeedbackReader reads the robot pose from g1_debug whether the deploy sends the viz field
`body_q_measured` or only `body_q` (g1-vr-teleop #25: the VR_3PT recalibration that every
intervention relies on used to fall back to an all-zero pose without it). Imports the streamer
module, so it needs the teleop venv:

    cd ~/GR00T-WholeBodyControl && .venv_teleop/bin/python tools/tests/test_streamer_feedback.py
"""
import numpy as np

from gear_sonic.scripts.pico_manager_thread_server import FeedbackReader, StreamMode
from gear_sonic.utils.teleop.policy_relay import POLICY_STREAM_MODE

idx = FeedbackReader._get_upper_body_joint_indices(None)
assert len(idx) == 17

q = list(np.arange(29, dtype=float) * 0.1)
# viz fields present: used as before
ub, lh, rh, full = FeedbackReader.targets_from_feedback(
    {"body_q_measured": q, "left_hand_q_measured": [1] * 7, "right_hand_q_measured": [2] * 7,
     "body_q": [9.0] * 29, "left_hand_q": [0] * 7}, idx)
assert full is not None and np.allclose(full, q) and ub == [q[i] for i in idx]
assert lh == [1] * 7 and rh == [2] * 7
# only the always-streamed fields: same result from body_q / *_hand_q
ub2, lh2, rh2, full2 = FeedbackReader.targets_from_feedback(
    {"body_q": q, "left_hand_q": [3] * 7, "right_hand_q": [4] * 7}, idx)
assert np.allclose(full2, q) and ub2 == ub and lh2 == [3] * 7 and rh2 == [4] * 7
# neither -> None everywhere (caller falls back to zeros with a warning)
assert FeedbackReader.targets_from_feedback({"foo": 1}, idx) == (None, None, None, None)
# a body_q of the wrong length is not trusted
assert FeedbackReader.targets_from_feedback({"body_q": [1.0] * 12}, idx)[3] is None

assert StreamMode.POLICY.value == POLICY_STREAM_MODE == 6
assert StreamMode.PLANNER_VR_3PT.value == 5
print("STREAMER_FEEDBACK_TEST_OK")

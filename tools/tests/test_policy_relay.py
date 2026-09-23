"""Unit tests for gear_sonic/utils/teleop/policy_relay.py (DAgger interventions, g1-vr-teleop #25).
No robot, no sockets: injected recv/send callables and a fake clock. numpy only.

    cd ~/GR00T-WholeBodyControl && .venv_inference/bin/python tools/tests/test_policy_relay.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
from gear_sonic.utils.teleop.policy_relay import (  # noqa: E402
    INTERVENTION_STREAM_MODE,
    POLICY_STREAM_MODE,
    PolicyRelay,
    RelayArbiter,
    contiguous_segments,
    format_stream_mode_summary,
    is_token_message,
    resume_blend_token,
    stream_mode_summary,
    unpack_pose_message,
)
from gear_sonic.utils.teleop.zmq.zmq_planner_sender import (  # noqa: E402
    build_command_message,
    pack_pose_message,
)


def token_msg(tok_value=0.5, left=None, right=None, frame=0):
    data = {
        "token_state": np.full((1, 64), tok_value, dtype=np.float32),
        "frame_index": np.array([frame], dtype=np.int64),
    }
    if left is not None:
        data["left_hand_joints"] = np.asarray(left, dtype=np.float32).reshape(1, 7)
    if right is not None:
        data["right_hand_joints"] = np.asarray(right, dtype=np.float32).reshape(1, 7)
    return pack_pose_message(data, topic="pose", version=4)


def smpl_msg():
    return pack_pose_message({"smpl_joints": np.zeros((5, 24, 3), dtype=np.float32),
                              "frame_index": np.arange(5, dtype=np.int64)}, topic="pose", version=3)


# --- unpack ---------------------------------------------------------------------------
m = unpack_pose_message(token_msg(0.25, left=np.arange(7), frame=7))
assert m["version"] == 4 and m["token_state"].shape == (1, 64) and float(m["token_state"][0, 3]) == 0.25
assert m["frame_index"][0] == 7 and list(m["left_hand_joints"][0]) == list(range(7))
assert is_token_message(m) and not is_token_message(unpack_pose_message(smpl_msg()))
ms = unpack_pose_message(pack_pose_message({"stream_mode": np.array([6], dtype=np.int32)}, topic="manager_state"),
                         topic="manager_state")
assert int(ms["stream_mode"][0]) == 6

# --- PolicyRelay ----------------------------------------------------------------------
clock = {"t": 100.0}
queue, sent = [], []


def recv():
    return queue.pop(0) if queue else None


def closure_fn(side, q7):
    # pretend: slot 0 is the thumb (0..1), slot 3 the fingers (0..1)
    return float(q7[3]), float(q7[0])


relay = PolicyRelay(recv, sent.append, closure_fn=closure_fn, snap_threshold=0.5, hand_max_age_s=2.0,
                    clock=lambda: clock["t"])
# inactive: everything drained, nothing forwarded
queue += [token_msg(), token_msg(), build_command_message(True, False, False)]
assert relay.pump(active=False) == 0 and not queue and not sent
assert relay.frames_dropped == 3 and relay.hand_inputs() == (0, 0.0, 0.0, 0.0, 0.0)
# active: token messages forwarded byte-for-byte, commands and v3 poses dropped
raw = token_msg(0.1, left=[0.9, 0, 0, 0.9, 0, 0, 0], right=[0.1, 0, 0, 0.1, 0, 0, 0], frame=1)
queue += [build_command_message(True, False, True), smpl_msg(), raw]
assert relay.pump(active=True) == 1
assert sent == [raw] and relay.frames_relayed == 1 and relay.frames_dropped == 5
# hands: left closed (fingers 0.9 -> trigger 1, thumb 0.9 -> squeeze 1), right open
assert relay.hand_inputs() == (0, 1.0, 0.0, 1.0, 0.0)
# stale after hand_max_age -> open
clock["t"] += 2.5
assert relay.hand_inputs() == (0, 0.0, 0.0, 0.0, 0.0)
clock["t"] -= 2.5
assert relay.hand_inputs()[1] == 1.0
# reset drops queued tokens and forgets the hands
queue += [token_msg(), token_msg()]
relay.reset()
assert not queue and len(sent) == 1 and relay.hand_inputs() == (0, 0.0, 0.0, 0.0, 0.0)
# proportional mode
relay_p = PolicyRelay(recv, sent.append, closure_fn=closure_fn, snap_threshold=None, clock=lambda: clock["t"])
queue.append(token_msg(0.1, left=[0.25, 0, 0, 0.75, 0, 0, 0]))
relay_p.pump(True)
assert relay_p.hand_inputs() == (0, 0.75, 0.0, 0.25, 0.0)
# a corrupt frame never raises
queue.append(b"pose" + b"\x00" * 10)
assert relay.pump(True) == 0

# --- RelayArbiter ---------------------------------------------------------------------
a = RelayArbiter(manager_timeout_s=2.0)
assert a.update(None, 0.0, paused=True) is None                 # nothing seen yet: quiet
assert a.yielded and not a.in_policy_mode
# streamer in PLANNER at start-up: we are paused anyway -> no event (already yielded)
assert a.update(2, 0.1, paused=True) is None
# g -> POLICY while paused: handover, operator presses p
assert a.update(POLICY_STREAM_MODE, 0.2, paused=True) == "handover"
assert a.in_policy_mode and not a.yielded
assert a.update(None, 0.3, paused=False) is None                 # no new message, still fine
# intervention while running -> yield, remembers we were running
assert a.update(INTERVENTION_STREAM_MODE, 1.0, paused=False) == "yield"
assert a.yielded and a.was_running
assert a.update(INTERVENTION_STREAM_MODE, 1.5, paused=True) is None
# back to POLICY -> resume once
assert a.update(POLICY_STREAM_MODE, 2.0, paused=True) == "resume"
assert a.update(POLICY_STREAM_MODE, 2.1, paused=False) is None and not a.was_running
# intervention while PAUSED -> yield, and coming back is only a handover
assert a.update(INTERVENTION_STREAM_MODE, 3.0, paused=True) == "yield" and not a.was_running
assert a.update(POLICY_STREAM_MODE, 3.5, paused=True) == "handover"
# p while yielded flips the resume intent
assert a.update(INTERVENTION_STREAM_MODE, 4.0, paused=True) == "yield"
a.set_resume_intent(True)
assert a.update(POLICY_STREAM_MODE, 4.5, paused=True) == "resume"
# streamer stops talking -> lost once, then quiet; recovery is a handover (never auto-resume)
assert a.update(None, 4.6, paused=False) is None
assert a.update(None, 7.0, paused=False) == "lost" and a.yielded and not a.in_policy_mode
assert a.update(None, 9.0, paused=True) is None
assert a.update(POLICY_STREAM_MODE, 9.5, paused=True) == "handover" and a.in_policy_mode
# OFF (A+B+X+Y) while running -> yield like any other mode
assert a.update(0, 10.0, paused=False) == "yield" and a.was_running

# --- blend ----------------------------------------------------------------------------
start, target = np.zeros(64, np.float32), np.ones(64, np.float32)
assert np.allclose(resume_blend_token(start, target, 1, 4), 0.25)
assert np.allclose(resume_blend_token(start, target, 4, 4), 1.0)
assert np.allclose(resume_blend_token(start, target, 9, 4), 1.0)
assert np.allclose(resume_blend_token(start, target, 1, 0), 1.0)      # 0 steps = snap
assert resume_blend_token(start, target, 1, 4).dtype == np.float32

# --- episode summary ------------------------------------------------------------------
assert contiguous_segments([]) == []
assert contiguous_segments([0, 0, 0]) == []
assert contiguous_segments([1, 1, 0, 1, 0, 0, 1]) == [(0, 2), (3, 4), (6, 7)]
modes = [6] * 10 + [5] * 5 + [6] * 3 + [5] * 2
s = stream_mode_summary(modes)
assert s["frames"] == 20 and s["policy_frames"] == 13 and s["intervention_frames"] == 7
assert s["intervention_segments"] == 2 and s["segments"] == [(10, 15), (18, 20)]
txt = format_stream_mode_summary(s)
assert "POLICY 13 (65%)" in txt and "PLANNER_VR_3PT 7 (35%)" in txt and "2 intervention segment(s)" in txt
assert format_stream_mode_summary(stream_mode_summary([])) == "0 frames"
assert "intervention" not in format_stream_mode_summary(stream_mode_summary([1, 1, 1]))
print("POLICY_RELAY_TEST_OK")

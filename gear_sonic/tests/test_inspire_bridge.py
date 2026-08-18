"""Unit tests for Inspire trigger mapping and change-log latency fields."""

from pathlib import Path
from unittest.mock import MagicMock

import pytest

from gear_sonic.utils.teleop.inspire.inspire_bridge import (
    FORCE_LIMIT_G,
    FORCE_SET_MAX_G,
    HANDS_RATE_HZ,
    OPEN,
    TACTILE_PERIOD_TICKS,
    WRAP_OPEN_MARGIN,
    InspireBridge,
)
from gear_sonic.utils.teleop.inspire.inspire_dump import (
    close_last_run_log,
    setup_last_run_log,
)

# Pinch-preset thumb_rotation: opposed / orthogonal to the finger plane (1000 = in-plane).
THUMB_ORTHOGONAL = 0


def test_rest_pose_holds_thumb_rotation_orthogonal_to_palm():
    angles = InspireBridge._targets_from_triggers(0.0, 0.0)
    assert angles[:5] == [OPEN, OPEN, OPEN, OPEN, OPEN]
    assert angles[5] == THUMB_ORTHOGONAL
    assert angles[5] != OPEN


def test_squeeze_bends_thumb_without_folding_rotation_back_into_palm_plane():
    angles = InspireBridge._targets_from_triggers(0.0, 1.0)
    assert angles[4] == 0
    assert angles[5] == THUMB_ORTHOGONAL


def test_trigger_curls_fingers_only():
    angles = InspireBridge._targets_from_triggers(1.0, 0.0)
    assert angles[:4] == [0, 0, 0, 0]
    assert angles[4] == OPEN
    assert angles[5] == THUMB_ORTHOGONAL


def test_sides_right_omits_left_hand():
    bridge = InspireBridge(
        get_inputs=lambda: (False, 0.0, 0.0, 0.0, 0.0),
        sides=("right",),
    )
    assert list(bridge._hands) == ["right"]


def test_hands_loop_default_is_90_hz():
    assert HANDS_RATE_HZ == 90.0
    bridge = InspireBridge(
        get_inputs=lambda: (False, 0.0, 0.0, 0.0, 0.0),
        sides=("right",),
    )
    assert bridge._rate_hz == 90.0
    assert bridge._dt == pytest.approx(1.0 / 90.0)


def test_tactile_reads_every_other_control_tick():
    assert TACTILE_PERIOD_TICKS == 2
    assert InspireBridge._tactile_this_tick(0) is True
    assert InspireBridge._tactile_this_tick(1) is False
    assert InspireBridge._tactile_this_tick(2) is True


def test_write_side_skips_tip_pads_when_tactile_rate_limited():
    hand = _connected_hand()
    bridge = InspireBridge(
        get_inputs=lambda: (False, 0.0, 0.0, 0.0, 0.0),
        sides=("right",),
    )
    bridge._hands["right"] = hand
    bridge._write_side("right", InspireBridge._open_pose(), read_tactile=False)
    assert hand.read_tactile_tip_peaks.call_count == 0
    hand.read_angles.assert_called()
    hand.read_forces.assert_called()


def test_write_side_holds_last_tactile_on_skip_tick():
    hand = _connected_hand()
    hand.read_tactile_tip_peaks.return_value = {
        "little_finger": 80, "ring_finger": 0, "middle_finger": 0,
        "index_finger": 0, "thumb": 0,
    }
    bridge = InspireBridge(
        get_inputs=lambda: (False, 0.0, 0.0, 0.0, 0.0),
        sides=("right",),
    )
    bridge._hands["right"] = hand
    bridge._tactile_base["right"] = {
        "little_finger": 0, "ring_finger": 0, "middle_finger": 0,
        "index_finger": 0, "thumb": 0,
    }
    bridge._write_side("right", InspireBridge._open_pose(), read_tactile=True)
    assert hand.read_tactile_tip_peaks.call_count == 1
    assert bridge._last_tactile["right"][0] is True
    hand.read_tactile_tip_peaks.reset_mock()
    bridge._write_side("right", InspireBridge._open_pose(), read_tactile=False)
    assert hand.read_tactile_tip_peaks.call_count == 0
    assert bridge._last_tactile["right"][0] is True


def test_yield_holds_close_cmd_on_high_force():
    cmd, stalled = InspireBridge._yield_on_stall(
        cmd=[0, 0, 0, 0, OPEN, 0],
        actual=[400, 400, 400, 400, OPEN, 0],
        forces=[500, 500, 500, 500, 0, 0],
    )
    assert stalled == [0, 1, 2, 3]
    assert cmd[:4] == [0, 0, 0, 0]


def test_stall_does_not_retract_toward_open():
    actual = [400, 400, 400, 400, OPEN, 0]
    cmd, stalled = InspireBridge._yield_on_stall(
        cmd=[0, 0, 0, 0, OPEN, 0],
        actual=actual,
        forces=[500, 500, 500, 500, 0, 0],
    )
    assert stalled
    for i in range(4):
        assert cmd[i] == 0
        assert cmd[i] != actual[i] + 40


def test_yield_ignores_rest_force_bias_when_closing():
    """DFTP FORCE_ACT is ~550g at open rest; that is bias, not contact."""
    rest_force = [555, 425, 176, 225, 227, 188]
    cmd, stalled = InspireBridge._yield_on_stall(
        cmd=[0, 0, 0, 0, OPEN, 0],
        actual=[OPEN, OPEN, OPEN, OPEN, OPEN, 0],
        forces=rest_force,
        force_baseline=rest_force,
    )
    assert stalled == []
    assert cmd[:4] == [0, 0, 0, 0]


def test_yield_on_force_rise_above_baseline():
    cmd, stalled = InspireBridge._yield_on_stall(
        cmd=[0, 0, 0, 0, OPEN, 0],
        actual=[400, 400, 400, 400, OPEN, 0],
        forces=[555, 425, 176, 425, 0, 0],
        force_baseline=[355, 425, 176, 225, 0, 0],
    )
    assert stalled == [0, 3]
    assert cmd[0] == 0
    assert cmd[1] == 0
    assert cmd[3] == 0


def test_force_set_adds_limit_above_rest_baseline():
    limits = InspireBridge._force_set_from_baseline(
        [555, 425, 176, 225, 227, 188]
    )
    assert limits[0] == 555 + FORCE_LIMIT_G
    assert limits[1] == 425 + FORCE_LIMIT_G
    assert limits[2] == 176 + FORCE_LIMIT_G
    assert all(lim > 400 for lim in limits[:2])


def test_full_trigger_raises_finger_force_set_to_firmware_max():
    rest = [555, 425, 176, 225, 227, 188]
    limits = InspireBridge._force_set_for_inputs(rest, trigger=0.95, squeeze=0.0)
    assert limits[:4] == [FORCE_SET_MAX_G] * 4
    assert limits[4] == 227 + FORCE_LIMIT_G
    assert limits[5] == 188 + FORCE_LIMIT_G


def test_full_grip_raises_thumb_bend_force_set_to_firmware_max():
    rest = [555, 425, 176, 225, 227, 188]
    limits = InspireBridge._force_set_for_inputs(rest, trigger=0.50, squeeze=1.00)
    assert limits[0] == 555 + FORCE_LIMIT_G
    assert limits[4] == FORCE_SET_MAX_G
    assert limits[5] == 188 + FORCE_LIMIT_G


def test_partial_trigger_keeps_cruise_force_cap():
    rest = [555, 425, 176, 225, 227, 188]
    limits = InspireBridge._force_set_for_inputs(rest, trigger=0.80, squeeze=0.80)
    assert limits == InspireBridge._force_set_from_baseline(rest)
    assert limits[0] == 555 + FORCE_LIMIT_G
    assert limits[4] == 227 + FORCE_LIMIT_G


def test_prepare_hand_writes_force_set_above_rest_bias():
    rest_force = [555, 425, 176, 225, 227, 188]
    hand = MagicMock()
    hand.connected = True
    hand.label = "InspireR"
    hand.read_forces.return_value = rest_force
    hand.read_tactile_tip_peaks.return_value = {
        "little_finger": 0, "ring_finger": 0, "middle_finger": 0,
        "index_finger": 0, "thumb": 0,
    }
    hand.write_speed.return_value = True
    hand.write_force_limits.return_value = True
    hand.write_angles.return_value = True

    bridge = InspireBridge(
        get_inputs=lambda: (False, 0.0, 0.0, 0.0, 0.0),
        sides=("right",),
    )
    bridge._prepare_hand("right", hand)
    limits = hand.write_force_limits.call_args[0][0]
    assert limits[0] == 555 + FORCE_LIMIT_G
    assert limits[1] == 425 + FORCE_LIMIT_G
    assert bridge._force_base["right"] == rest_force


def test_write_side_does_not_snap_close_cmd_to_rest_on_force_bias():
    rest_force = [555, 425, 176, 225, 227, 188]
    hand = MagicMock()
    hand.connected = True
    hand.ip = "127.0.0.1"
    hand.port = 6000
    hand.label = "fake"
    hand.read_angles.return_value = [OPEN] * 6
    hand.read_forces.return_value = rest_force
    hand.read_tactile_tip_peaks.return_value = {
        "little_finger": 0, "ring_finger": 0, "middle_finger": 0,
        "index_finger": 0, "thumb": 0,
    }
    hand.write_angles.return_value = True

    bridge = InspireBridge(
        get_inputs=lambda: (False, 0.0, 1.0, 0.0, 0.0),
        sides=("right",),
    )
    bridge._hands["right"] = hand
    bridge._force_base["right"] = list(rest_force)
    targets = InspireBridge._targets_from_triggers(1.0, 0.0)
    bridge._write_side("right", targets)
    written = hand.write_angles.call_args[0][0]
    assert written[0] < 900
    assert written[0] < OPEN


def _connected_hand(rest_force=None):
    hand = MagicMock()
    hand.connected = True
    hand.ip = "127.0.0.1"
    hand.port = 6000
    hand.label = "fake"
    hand.read_angles.return_value = [OPEN] * 6
    hand.read_forces.return_value = rest_force or [0] * 6
    hand.read_tactile_tip_peaks.return_value = {
        "little_finger": 0, "ring_finger": 0, "middle_finger": 0,
        "index_finger": 0, "thumb": 0,
    }
    hand.write_angles.return_value = True
    hand.write_force_limits.return_value = True
    return hand


def test_write_side_full_trigger_writes_firmware_max_force_set():
    rest_force = [555, 425, 176, 225, 227, 188]
    hand = _connected_hand(rest_force)
    bridge = InspireBridge(
        get_inputs=lambda: (False, 0.0, 1.0, 0.0, 0.0),
        sides=("right",),
    )
    bridge._hands["right"] = hand
    bridge._force_base["right"] = list(rest_force)
    targets = InspireBridge._targets_from_triggers(1.0, 0.0)
    bridge._write_side("right", targets, rt=1.0, rg=0.0)
    limits = hand.write_force_limits.call_args[0][0]
    assert limits[:4] == [FORCE_SET_MAX_G] * 4
    assert limits[4] == 227 + FORCE_LIMIT_G


def test_write_side_partial_trigger_writes_cruise_force_set():
    rest_force = [555, 425, 176, 225, 227, 188]
    hand = _connected_hand(rest_force)
    bridge = InspireBridge(
        get_inputs=lambda: (False, 0.0, 0.5, 0.0, 0.5),
        sides=("right",),
    )
    bridge._hands["right"] = hand
    bridge._force_base["right"] = list(rest_force)
    targets = InspireBridge._targets_from_triggers(0.5, 0.5)
    bridge._write_side("right", targets, rt=0.50, rg=0.50)
    limits = hand.write_force_limits.call_args[0][0]
    assert limits == InspireBridge._force_set_from_baseline(rest_force)


def test_yield_allows_closing_when_force_is_low():
    cmd, stalled = InspireBridge._yield_on_stall(
        cmd=[200, 200, 200, 200, OPEN, 0],
        actual=[400, 400, 400, 400, OPEN, 0],
        forces=[10, 10, 10, 10, 0, 0],
    )
    assert stalled == []
    assert cmd[:4] == [200, 200, 200, 200]


def test_yield_on_tactile_contact_even_if_force_is_zero():
    cmd, stalled = InspireBridge._yield_on_stall(
        cmd=[0, 0, 0, 0, OPEN, 0],
        actual=[400, 400, 400, 400, OPEN, 0],
        forces=[0, 0, 0, 0, 0, 0],
        tactile=[True, False, False, False, False, False],
    )
    assert stalled == [0]
    assert cmd[0] == 0
    assert cmd[1] == 0


def test_tactile_hits_use_peak_above_baseline():
    hits = InspireBridge._tactile_hits_from_peaks(
        peaks={
            "little_finger": 80, "ring_finger": 10, "middle_finger": 0,
            "index_finger": 0, "thumb": 5,
        },
        baseline={
            "little_finger": 20, "ring_finger": 10, "middle_finger": 0,
            "index_finger": 0, "thumb": 5,
        },
        margin=40,
    )
    assert hits[0] is True
    assert hits[1] is False
    assert hits[4] is False
    assert hits[5] is False


def test_yield_never_blocks_opening():
    cmd, stalled = InspireBridge._yield_on_stall(
        cmd=[OPEN, OPEN, OPEN, OPEN, OPEN, 0],
        actual=[0, 0, 0, 0, 0, 0],
        forces=[800, 800, 800, 800, 800, 0],
    )
    assert cmd[0] == OPEN
    assert 0 not in stalled


def test_change_log_includes_modbus_write_and_readback_rtt():
    line = InspireBridge._format_change(
        dt_s=0.017,
        lt=0.0,
        rt=0.85,
        lg=0.0,
        rg=1.0,
        writes={
            "right": {"cmd_ms": 3.14, "fb_ms": 7.8, "err": 12},
        },
    )
    assert "dt=    17ms" in line
    assert "rt=0.85" in line
    assert "rg=1.00" in line
    assert "right: cmd=3.1ms fb=7.8ms err=12" in line
    assert "left:" not in line


def test_write_side_writes_last_run_cmd_actual_vectors(tmp_path):
    stale = tmp_path / "inspire-last-run.log"
    stale.write_text("OLD RUN\n", encoding="utf-8")
    path = Path(setup_last_run_log(log_dir=str(tmp_path)))
    try:
        hand = MagicMock()
        hand.connected = True
        hand.ip = "127.0.0.1"
        hand.port = 6000
        hand.label = "fake"
        hand.read_angles.return_value = [11, 21, 31, 41, 51, 61]
        hand.read_forces.return_value = [1, 2, 3, 4, 5, 6]
        hand.read_tactile_tip_peaks.return_value = {
            "little_finger": 0, "ring_finger": 0, "middle_finger": 0,
            "index_finger": 0, "thumb": 0,
        }
        hand.write_angles.return_value = True

        bridge = InspireBridge(
            get_inputs=lambda: (False, 0.0, 0.5, 0.0, 1.0),
            sides=("right",),
        )
        bridge._hands["right"] = hand
        cmd = [10, 20, 30, 40, 50, 60]
        bridge._smoothed["right"] = [float(v) for v in cmd]
        bridge._write_side(
            "right", cmd, lt=0.0, rt=0.50, lg=0.0, rg=1.00,
        )
        text = path.read_text(encoding="utf-8")
        assert "OLD RUN" not in text
        assert "side=right" in text
        assert "cmd=[10,20,30,40,50,60]" in text
        assert "actual=[11,21,31,41,51,61]" in text
        assert "force_act_g=[1,2,3,4,5,6]" in text
        assert "rt=0.50" in text
        assert "rg=1.00" in text
        assert "err=1" in text
    finally:
        close_last_run_log()


def test_write_side_last_run_still_logs_when_write_is_deadbanded(tmp_path):
    path = Path(setup_last_run_log(log_dir=str(tmp_path)))
    try:
        hand = MagicMock()
        hand.connected = True
        hand.ip = "127.0.0.1"
        hand.port = 6000
        hand.label = "fake"
        rest = InspireBridge._open_pose()
        hand.read_angles.return_value = list(rest)
        hand.read_forces.return_value = [0] * 6
        hand.read_tactile_tip_peaks.return_value = {
            "little_finger": 0, "ring_finger": 0, "middle_finger": 0,
            "index_finger": 0, "thumb": 0,
        }
        hand.write_angles.return_value = True

        bridge = InspireBridge(
            get_inputs=lambda: (False, 0.0, 0.0, 0.0, 0.0),
            sides=("right",),
        )
        bridge._hands["right"] = hand
        bridge._smoothed["right"] = [float(v) for v in rest]
        bridge._last_written["right"] = list(rest)
        out = bridge._write_side("right", rest, rt=0.10)
        assert out is None
        assert hand.write_angles.call_count == 0
        text = path.read_text(encoding="utf-8")
        assert "cmd=" in text
        assert "actual=" in text
        assert "side=right" in text
        assert "rt=0.10" in text
    finally:
        close_last_run_log()


# Last-run bottle grasp (rt≈0.77): shared cmd, little/ring/middle braced,
# index still open. FORCE_ACT rose on all four (coupled); tactile all false.
_BOTTLE_RT = 0.77
_BOTTLE_DESIRED = int(OPEN - _BOTTLE_RT * OPEN)  # 230
_BOTTLE_ACTUAL = [316, 482, 576, 693, OPEN, 0]
_BOTTLE_FORCE = [1380, 1130, 700, 704, 250, 177]
_BOTTLE_BASE = [579, 410, 169, 220, 249, 177]
_BOTTLE_TACTILE = [False, False, False, False, False, False]


def test_partial_trigger_maps_shared_desired_not_full_close():
    """One trigger → same angle for DOFs 0-3; 0.77 is not wrap-to-0."""
    angles = InspireBridge._targets_from_triggers(_BOTTLE_RT, 0.0)
    assert angles[:4] == [angles[0]] * 4
    assert angles[0] == OPEN - _BOTTLE_RT * OPEN
    assert int(angles[0]) == _BOTTLE_DESIRED
    assert angles[3] > 0
    assert angles[4] == OPEN


def test_enveloping_free_index_tracks_desired_not_wrap_to_zero():
    """Three fingers braced, index free: cmd stays on the shared trigger angle."""
    desired = [_BOTTLE_DESIRED] * 4 + [OPEN, 0]
    cmd, stalled = InspireBridge._yield_on_stall(
        cmd=desired,
        actual=_BOTTLE_ACTUAL,
        forces=_BOTTLE_FORCE,
        tactile=_BOTTLE_TACTILE,
        force_baseline=_BOTTLE_BASE,
    )
    assert 3 not in stalled
    assert stalled == [0, 1, 2]
    assert cmd[3] == _BOTTLE_DESIRED
    assert cmd[3] != 0
    assert cmd[:3] == [_BOTTLE_DESIRED] * 3


def test_coupled_fmax_does_not_stall_open_finger_without_own_rise():
    """High Fmax on siblings must not freeze an index still at rest force."""
    forces = [1380, 1130, 700, _BOTTLE_BASE[3], 250, 177]
    desired = [_BOTTLE_DESIRED] * 4 + [OPEN, 0]
    cmd, stalled = InspireBridge._yield_on_stall(
        cmd=desired,
        actual=_BOTTLE_ACTUAL,
        forces=forces,
        tactile=_BOTTLE_TACTILE,
        force_baseline=_BOTTLE_BASE,
    )
    assert 3 not in stalled
    assert cmd[3] == _BOTTLE_DESIRED
    assert cmd[:3] == [_BOTTLE_DESIRED] * 3


def test_partial_trigger_boosts_free_finger_force_set_only():
    """At rt=0.77 cruise stays on contacted DOFs; free index can reach desired."""
    rest = list(_BOTTLE_BASE)
    contacts = [True, True, True, False, False, False]
    desired = [_BOTTLE_DESIRED] * 4 + [OPEN, 0]
    limits = InspireBridge._force_set_for_inputs(
        rest,
        trigger=_BOTTLE_RT,
        squeeze=0.0,
        contacts=contacts,
        actual=_BOTTLE_ACTUAL,
        desired=desired,
    )
    assert limits[3] == FORCE_SET_MAX_G
    assert limits[0] == rest[0] + FORCE_LIMIT_G
    assert limits[1] == rest[1] + FORCE_LIMIT_G
    assert limits[2] == rest[2] + FORCE_LIMIT_G
    assert limits[4] == rest[4] + FORCE_LIMIT_G


def test_no_contact_keeps_cruise_force_while_closing_in_air():
    rest = list(_BOTTLE_BASE)
    desired = [_BOTTLE_DESIRED] * 4 + [OPEN, 0]
    actual = [OPEN, OPEN, OPEN, OPEN, OPEN, 0]
    limits = InspireBridge._force_set_for_inputs(
        rest,
        trigger=_BOTTLE_RT,
        squeeze=0.0,
        contacts=[False] * 6,
        actual=actual,
        desired=desired,
    )
    assert limits == InspireBridge._force_set_from_baseline(rest)


def test_write_side_bottle_grasp_lets_free_index_reach_desired():
    rest_force = list(_BOTTLE_BASE)
    hand = _connected_hand(rest_force)
    hand.read_angles.return_value = list(_BOTTLE_ACTUAL)
    hand.read_forces.return_value = list(_BOTTLE_FORCE)
    bridge = InspireBridge(
        get_inputs=lambda: (False, 0.0, _BOTTLE_RT, 0.0, 0.0),
        sides=("right",),
    )
    bridge._hands["right"] = hand
    bridge._force_base["right"] = list(rest_force)
    desired = InspireBridge._targets_from_triggers(_BOTTLE_RT, 0.0)
    bridge._smoothed["right"] = [float(v) for v in desired]
    bridge._write_side("right", desired, rt=_BOTTLE_RT, rg=0.0)
    written = hand.write_angles.call_args[0][0]
    assert written[:4] == [_BOTTLE_DESIRED] * 4
    assert written[3] != 0
    limits = hand.write_force_limits.call_args[0][0]
    assert limits[3] == FORCE_SET_MAX_G
    assert limits[0] == rest_force[0] + FORCE_LIMIT_G


def test_uncouple_drops_only_when_clearly_more_open_than_stalled_max():
    stalled = [0, 1, 2, 3]
    actual = [400, 400, 400, 400 + WRAP_OPEN_MARGIN + 1, OPEN, 0]
    out = InspireBridge._uncouple_open_fingers(stalled, actual)
    assert 3 not in out
    assert out == [0, 1, 2]


def test_uncouple_keeps_slightly_more_open_finger_stalled():
    stalled = [0, 1, 2, 3]
    actual = [400, 400, 400, 450, OPEN, 0]
    assert actual[3] - max(actual[:3]) < WRAP_OPEN_MARGIN
    out = InspireBridge._uncouple_open_fingers(stalled, actual)
    assert 3 in out
    assert out == [0, 1, 2, 3]


def test_uncouple_two_similarly_open_fingers_protect_each_other():
    """Neither wrapping finger is above the max of the other stalled set."""
    stalled = [0, 1, 2, 3]
    actual = [400, 400, 700, 710, OPEN, 0]
    out = InspireBridge._uncouple_open_fingers(stalled, actual)
    assert 2 in out
    assert 3 in out
    assert out == [0, 1, 2, 3]


"""Inspire dump snapshots, ZMQ topics, and LeRobot feature split.

Does not open TCP/Modbus to the hands. Localhost ZMQ bind tests use an
ephemeral port so they do not collide with a live bridge on 5558.
"""

import socket
from pathlib import Path
from unittest.mock import MagicMock

import zmq

from gear_sonic.utils.teleop.inspire.inspire_bridge import (
    FORCE_LIMIT_G,
    FORCE_SET_MAX_G,
    HANDS_RATE_HZ,
    InspireBridge,
)
from gear_sonic.utils.teleop.inspire.inspire_dump import (
    INSPIRE_DUMP_PORT,
    INSPIRE_HAND_TOPIC,
    INSPIRE_RECORD_FPS,
    INSPIRE_TACTILE_TOPIC,
    INSPIRE_TIP_REGION_NAMES,
    InspireDumpPublisher,
    InspireDumpSubscriber,
    build_inspire_snapshots,
    close_last_run_log,
    empty_inspire_snapshots,
    format_dump_addr_in_use,
    format_last_run_line,
    get_inspire_dataset_features,
    last_run_log_path,
    setup_last_run_log,
    write_last_run_sample,
)

WBC_ROOT = Path(__file__).resolve().parents[2]
PICO_VIVE_BRIDGE_PY = WBC_ROOT / "pico_vive_bridge.py"
INSPIRE_BRIDGE_PY = (
    WBC_ROOT / "gear_sonic" / "utils" / "teleop" / "inspire" / "inspire_bridge.py"
)


def _peaks(**overrides):
    peaks = {name: 0 for name in INSPIRE_TIP_REGION_NAMES}
    peaks.update(overrides)
    return peaks


def _unused_tcp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def test_mechanical_snapshot_shape_and_dof_order():
    mech, tac = build_inspire_snapshots(
        side="right",
        cmd=[10, 20, 30, 40, 50, 60],
        actual=[11, 21, 31, 41, 51, 61],
        force_act_g=[1, 2, 3, 4, 5, 6],
        tip_peaks=_peaks(index_finger=90, thumb=7),
        tactile_hit=[False, False, False, True, False, False],
        stall_dofs=[3],
        yielded=True,
        valid=1,
        t_unix=100.5,
        t_mono=12.0,
    )
    assert mech["t_unix"] == 100.5
    assert mech["t_mono"] == 12.0
    assert mech["side"] == "right"
    assert mech["cmd"] == [10, 20, 30, 40, 50, 60]
    assert mech["actual"] == [11, 21, 31, 41, 51, 61]
    assert mech["force_act_g"] == [1, 2, 3, 4, 5, 6]
    assert mech["force_set_g"] == [FORCE_LIMIT_G] * 6
    assert mech["tactile_hit"] == [False, False, False, True, False, False]
    assert mech["stall_dofs"] == [3]
    assert mech["yielded"] is True
    assert mech["valid"] == 1
    assert "tip_peaks" not in mech
    assert "tactile_tip" not in mech


def test_tactile_snapshot_is_separate_and_shares_clock():
    mech, tac = build_inspire_snapshots(
        side="right",
        cmd=[0] * 6,
        actual=[0] * 6,
        force_act_g=[0] * 6,
        tip_peaks=_peaks(little_finger=1, ring_finger=2, middle_finger=3, index_finger=4, thumb=5),
        tactile_hit=[False] * 6,
        stall_dofs=[],
        yielded=False,
        valid=1,
        t_unix=7.0,
        t_mono=8.0,
    )
    assert tac["t_unix"] == mech["t_unix"] == 7.0
    assert tac["t_mono"] == mech["t_mono"] == 8.0
    assert tac["side"] == "right"
    assert tac["tip_peaks"] == [1, 2, 3, 4, 5]
    assert list(tac["tip_peaks"]) == [
        tac["tip_peaks"][0],
        tac["tip_peaks"][1],
        tac["tip_peaks"][2],
        tac["tip_peaks"][3],
        tac["tip_peaks"][4],
    ]
    assert "cmd" not in tac
    assert "actual" not in tac
    assert "force_act_g" not in tac


def test_zmq_topic_names_and_port():
    assert INSPIRE_DUMP_PORT == 5558
    assert INSPIRE_HAND_TOPIC == "inspire_hand"
    assert INSPIRE_TACTILE_TOPIC == "inspire_tactile"
    assert INSPIRE_HAND_TOPIC != INSPIRE_TACTILE_TOPIC
    assert INSPIRE_DUMP_PORT != 5556


def test_empty_snapshot_marks_missing_side_invalid():
    mech, tac = empty_inspire_snapshots(side="left", t_unix=1.0, t_mono=2.0)
    assert mech["valid"] == 0
    assert tac["valid"] == 0
    assert mech["cmd"] == [0] * 6
    assert mech["actual"] == [0] * 6
    assert tac["tip_peaks"] == [0] * 5
    assert mech["t_unix"] == tac["t_unix"] == 1.0


def test_write_side_publishes_mechanical_and_tactile_every_tick():
    recorded = []

    class _Dump:
        def publish(self, mechanical, tactile):
            recorded.append((mechanical, tactile))

    hand = MagicMock()
    hand.connected = True
    hand.ip = "127.0.0.1"
    hand.port = 6000
    hand.label = "fake"
    hand.read_angles.return_value = [1000] * 6
    hand.read_forces.return_value = [0] * 6
    hand.read_tactile_tip_peaks.return_value = _peaks()
    hand.write_angles.return_value = True

    bridge = InspireBridge(
        get_inputs=lambda: (False, 0.0, 0.0, 0.0, 0.0),
        sides=("right",),
        dump_publisher=_Dump(),
    )
    bridge._hands["right"] = hand
    targets = InspireBridge._open_pose()
    bridge._write_side("right", targets)
    assert len(recorded) == 1
    mech, tac = recorded[0]
    assert mech["side"] == "right"
    assert mech["valid"] == 1
    assert tac["tip_peaks"] == [0] * 5
    assert tac["t_unix"] == mech["t_unix"]
    hand.read_tactile_tip_peaks.assert_called()


def test_write_side_still_publishes_dump_when_tactile_skipped():
    recorded = []

    class _Dump:
        def publish(self, mechanical, tactile):
            recorded.append((mechanical, tactile))

    hand = MagicMock()
    hand.connected = True
    hand.ip = "127.0.0.1"
    hand.port = 6000
    hand.label = "fake"
    hand.read_angles.return_value = [1000] * 6
    hand.read_forces.return_value = [0] * 6
    hand.read_tactile_tip_peaks.return_value = _peaks(thumb=3)
    hand.write_angles.return_value = True

    bridge = InspireBridge(
        get_inputs=lambda: (False, 0.0, 0.0, 0.0, 0.0),
        sides=("right",),
        dump_publisher=_Dump(),
    )
    bridge._hands["right"] = hand
    targets = InspireBridge._open_pose()
    bridge._write_side("right", targets, read_tactile=True)
    bridge._write_side("right", targets, read_tactile=False)
    assert hand.read_tactile_tip_peaks.call_count == 1
    assert len(recorded) == 2
    assert recorded[1][1]["tip_peaks"][-1] == 3


def test_write_side_still_dumps_when_write_is_deadbanded():
    recorded = []

    class _Dump:
        def publish(self, mechanical, tactile):
            recorded.append((mechanical, tactile))

    hand = MagicMock()
    hand.connected = True
    hand.ip = "127.0.0.1"
    hand.port = 6000
    hand.label = "fake"
    rest = InspireBridge._open_pose()
    hand.read_angles.return_value = list(rest)
    hand.read_forces.return_value = [0] * 6
    hand.read_tactile_tip_peaks.return_value = _peaks(thumb=3)
    hand.write_angles.return_value = True

    bridge = InspireBridge(
        get_inputs=lambda: (False, 0.0, 0.0, 0.0, 0.0),
        sides=("right",),
        dump_publisher=_Dump(),
    )
    bridge._hands["right"] = hand
    bridge._last_written["right"] = list(rest)
    out = bridge._write_side("right", rest)
    assert out is None
    assert hand.write_angles.call_count == 0
    assert len(recorded) == 1
    assert recorded[0][1]["tip_peaks"][-1] == 3


def test_write_side_reads_tip_peaks_when_not_stalled():
    hand = MagicMock()
    hand.connected = True
    hand.ip = "127.0.0.1"
    hand.port = 6000
    hand.label = "fake"
    hand.read_angles.return_value = [1000] * 6
    hand.read_forces.return_value = [0] * 6
    hand.read_tactile_tip_peaks.return_value = _peaks()
    hand.write_angles.return_value = True

    bridge = InspireBridge(
        get_inputs=lambda: (False, 0.0, 0.0, 0.0, 0.0),
        sides=("right",),
    )
    bridge._hands["right"] = hand
    bridge._write_side("right", InspireBridge._open_pose())
    assert hand.read_tactile_tip_peaks.call_count == 1


def test_disconnected_side_publishes_valid_zero():
    recorded = []

    class _Dump:
        def publish(self, mechanical, tactile):
            recorded.append((mechanical, tactile))

    hand = MagicMock()
    hand.connected = False
    hand.ip = "192.168.123.210"
    hand.port = 6000
    hand.label = "InspireL"

    bridge = InspireBridge(
        get_inputs=lambda: (False, 0.0, 0.0, 0.0, 0.0),
        sides=("left",),
        dump_publisher=_Dump(),
    )
    bridge._hands["left"] = hand
    bridge._last_reconnect["left"] = 10**12
    bridge._write_side("left", InspireBridge._open_pose())
    assert recorded
    assert recorded[0][0]["valid"] == 0
    assert recorded[0][1]["valid"] == 0
    hand.read_angles.assert_not_called()


def test_dump_publisher_sends_two_multipart_topics():
    from gear_sonic.utils.teleop.inspire.inspire_dump import InspireDumpPublisher

    sent = []

    class _Sock:
        def send_multipart(self, parts):
            sent.append(parts)

    mech, tac = build_inspire_snapshots(
        side="right",
        cmd=[1, 2, 3, 4, 5, 6],
        actual=[1, 2, 3, 4, 5, 6],
        force_act_g=[0] * 6,
        tip_peaks=_peaks(),
        tactile_hit=[False] * 6,
        stall_dofs=[],
        yielded=False,
        valid=1,
        t_unix=1.0,
        t_mono=2.0,
    )
    pub = InspireDumpPublisher.__new__(InspireDumpPublisher)
    pub._sock = _Sock()
    pub._pack = lambda obj: b"packed:" + str(obj.get("side", "")).encode()
    pub.publish(mech, tac)
    topics = [parts[0].decode() if isinstance(parts[0], bytes) else parts[0] for parts in sent]
    assert topics == [INSPIRE_HAND_TOPIC, INSPIRE_TACTILE_TOPIC]


def test_write_side_dump_publishes_per_dof_force_set():
    recorded = []

    class _Dump:
        def publish(self, mechanical, tactile):
            recorded.append((mechanical, tactile))

    rest_force = [555, 425, 176, 225, 227, 188]
    hand = MagicMock()
    hand.connected = True
    hand.ip = "127.0.0.1"
    hand.port = 6000
    hand.label = "fake"
    hand.read_angles.return_value = [1000] * 6
    hand.read_forces.return_value = list(rest_force)
    hand.read_tactile_tip_peaks.return_value = _peaks()
    hand.write_angles.return_value = True
    hand.write_force_limits.return_value = True

    bridge = InspireBridge(
        get_inputs=lambda: (False, 0.0, 1.0, 0.0, 0.0),
        sides=("right",),
        dump_publisher=_Dump(),
    )
    bridge._hands["right"] = hand
    bridge._force_base["right"] = list(rest_force)
    targets = InspireBridge._targets_from_triggers(1.0, 0.0)
    bridge._write_side("right", targets, rt=1.0, rg=0.0)
    assert recorded
    force_set = recorded[0][0]["force_set_g"]
    assert force_set[:4] == [FORCE_SET_MAX_G] * 4
    assert force_set[4] == 227 + FORCE_LIMIT_G
    assert force_set[5] == 188 + FORCE_LIMIT_G
    assert force_set != [FORCE_LIMIT_G] * 6


def test_inspire_lerobot_features_keep_tactile_out_of_state():
    features = get_inspire_dataset_features()
    mechanical = {
        "observation.inspire_cmd",
        "observation.inspire_q",
        "observation.inspire_force_g",
        "observation.inspire_valid",
    }
    tactile = {"observation.inspire_tactile_tip"}
    assert mechanical <= set(features)
    assert tactile <= set(features)
    assert features["observation.inspire_cmd"]["shape"] == (6,)
    assert features["observation.inspire_q"]["shape"] == (6,)
    assert features["observation.inspire_force_g"]["shape"] == (6,)
    assert features["observation.inspire_tactile_tip"]["shape"] == (5,)
    assert "observation.state" not in features
    for key in tactile:
        assert "state" not in key.split(".")[1]


def test_hands_loop_is_90_dump_record_fps_constant_is_60():
    assert HANDS_RATE_HZ == 90.0
    assert INSPIRE_RECORD_FPS == 60
    bridge_text = INSPIRE_BRIDGE_PY.read_text()
    assert "HANDS_RATE_HZ = 90.0" in bridge_text
    assert "rate_hz=HANDS_RATE_HZ" in bridge_text
    pico_text = PICO_VIVE_BRIDGE_PY.read_text()
    assert "rate_hz=HANDS_RATE_HZ" in pico_text


def test_last_run_log_truncated_on_setup(tmp_path):
    stale = tmp_path / "inspire-last-run.log"
    stale.write_text("stale line from previous run\ncmd=[1,2,3,4,5,6]\n", encoding="utf-8")
    path = Path(setup_last_run_log(log_dir=str(tmp_path)))
    try:
        assert path == stale
        assert path.read_text(encoding="utf-8") == ""
    finally:
        close_last_run_log()


def test_last_run_path_honors_g1_vr_log_dir(tmp_path, monkeypatch):
    monkeypatch.setenv("G1_VR_LOG_DIR", str(tmp_path))
    assert last_run_log_path() == str(tmp_path / "inspire-last-run.log")


def test_format_last_run_line_includes_cmd_actual_force():
    mech, _ = build_inspire_snapshots(
        side="right",
        cmd=[10, 20, 30, 40, 50, 60],
        actual=[11, 21, 31, 41, 51, 61],
        force_act_g=[1, 2, 3, 4, 5, 6],
        tip_peaks=_peaks(index_finger=90),
        tactile_hit=[False, False, False, True, False, False],
        stall_dofs=[3],
        yielded=True,
        valid=1,
        t_unix=100.5,
        t_mono=12.0,
    )
    line = format_last_run_line(
        mech, lt=0.0, rt=0.85, lg=0.0, rg=1.0, dt_s=0.016,
    )
    assert "side=right" in line
    assert "cmd=[10,20,30,40,50,60]" in line
    assert "actual=[11,21,31,41,51,61]" in line
    assert "force_act_g=[1,2,3,4,5,6]" in line
    assert "force_set_g=" in line
    assert "err=1" in line
    assert "stall=[3]" in line
    assert "Fmax=6" in line
    assert "tactile=[0,0,0,1,0,0]" in line
    assert "yielded=1" in line
    assert "rt=0.85" in line
    assert "rg=1.00" in line
    assert "dt_ms=" in line
    assert "t_unix=100.5" in line
    assert "\n" not in line


def test_write_last_run_sample_is_one_line_per_call(tmp_path):
    path = Path(setup_last_run_log(log_dir=str(tmp_path)))
    try:
        mech, _ = build_inspire_snapshots(
            side="right",
            cmd=[1000, 1000, 1000, 1000, 1000, 0],
            actual=[998, 997, 999, 1000, 1000, 0],
            force_act_g=[10, 12, 8, 9, 5, 3],
            tip_peaks=_peaks(),
            tactile_hit=[False] * 6,
            stall_dofs=[],
            yielded=False,
            valid=1,
            t_unix=1.0,
            t_mono=2.0,
        )
        write_last_run_sample(mech, lt=0.0, rt=0.2, lg=0.0, rg=0.0, dt_s=0.016)
        write_last_run_sample(mech, lt=0.0, rt=0.3, lg=0.0, rg=0.0, dt_s=0.017)
        lines = path.read_text(encoding="utf-8").strip().splitlines()
        assert len(lines) == 2
        assert "cmd=[1000,1000,1000,1000,1000,0]" in lines[0]
        assert "actual=[998,997,999,1000,1000,0]" in lines[0]
        assert "rt=0.20" in lines[0]
        assert "rt=0.30" in lines[1]
    finally:
        close_last_run_log()


def test_dump_publisher_sets_linger_zero():
    ctx = zmq.Context()
    port = _unused_tcp_port()
    pub = InspireDumpPublisher(ctx=ctx, port=port)
    try:
        assert pub._sock.getsockopt(zmq.LINGER) == 0
    finally:
        pub.close()
        ctx.term()


def test_dump_publisher_close_unbinds_so_second_bind_succeeds():
    ctx = zmq.Context()
    port = _unused_tcp_port()
    pub1 = InspireDumpPublisher(ctx=ctx, port=port)
    pub1.close()
    pub2 = InspireDumpPublisher(ctx=ctx, port=port)
    try:
        assert pub2._sock.getsockopt(zmq.LINGER) == 0
    finally:
        pub2.close()
        ctx.term()


def test_dump_publisher_second_bind_prints_addr_in_use(capsys):
    ctx = zmq.Context()
    port = _unused_tcp_port()
    pub1 = InspireDumpPublisher(ctx=ctx, port=port)
    pub2 = None
    try:
        try:
            pub2 = InspireDumpPublisher(ctx=ctx, port=port)
        except zmq.ZMQError:
            pass
        else:
            raise AssertionError("second bind should raise ZMQError")
        logged = capsys.readouterr()
        combined = f"{logged.out}{logged.err}"
        assert str(port) in combined
        assert "pico_vive_bridge" in combined
    finally:
        if pub2 is not None:
            pub2.close()
        pub1.close()
        ctx.term()


def test_dump_subscriber_sets_linger_zero():
    ctx = zmq.Context()
    port = _unused_tcp_port()
    pub = InspireDumpPublisher(ctx=ctx, port=port)
    sub = InspireDumpSubscriber(host="127.0.0.1", port=port, ctx=ctx)
    try:
        assert sub._hand._sock.getsockopt(zmq.LINGER) == 0
        assert sub._tactile._sock.getsockopt(zmq.LINGER) == 0
    finally:
        sub.close()
        pub.close()
        ctx.term()


def test_format_dump_addr_in_use_includes_pid_and_hint():
    msg = format_dump_addr_in_use(5558, host="127.0.0.1", pid=206156)
    assert "5558" in msg
    assert "206156" in msg
    assert "pico_vive_bridge" in msg


def test_pico_vive_bridge_binds_dump_before_xrt_init():
    text = (WBC_ROOT / "pico_vive_bridge.py").read_text()
    assert text.index("InspireDumpPublisher(") < text.index("    xrt.init()\n")
    assert "\n    finally:" in text
    close_at = text.index("dump_pub.close()")
    finally_at = text.index("\n    finally:")
    assert finally_at < close_at

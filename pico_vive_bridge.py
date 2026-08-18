#!/usr/bin/env python3
"""PICO -> decoupled_wbc IK bridge (host side).

Reads PICO headset + controller poses via xrobotoolkit_sdk and re-serves them
over the ZMQ "vive" protocol that decoupled_wbc's ViveStreamer speaks, so the
container-side IK teleop loop (which has pink/pinocchio but NOT the PICO SDK or
roboticsservice) can be driven from a PICO.

Why a bridge: wbc-dev has no xrobotoolkit_sdk / roboticsservice, and the host
.venv_teleop has the PICO SDK but no pink. The container is --network host, so
it reaches this bridge at 127.0.0.1. This mirrors quest_vive_bridge.py but
sources poses from the PICO SDK (xrobotoolkit_sdk) instead of CloudXR.

Run ORDER (all on the HOST unless noted):
  1) PICO PC service:  ~/start_xrsvc.sh        (PICO app connects to 10.42.0.1)
  2) this bridge (in .venv_teleop):
         python pico_vive_bridge.py            [--inspire-hands trigger] [--tracked_hands right]
  3) container teleop loop (--body_control_device quest reads this over ZMQ):
         run_teleop_policy_loop.py --body_control_device quest \
             --body_streamer_ip 127.0.0.1 --body_streamer_keyword wrist \
             --hand_control_device None --tracked_hands right

Both wrists are ALWAYS streamed (an invalid/idle side falls back to identity) so
the container never KeyErrors on a missing key; single-arm is enforced in the IK
via --tracked_hands (the untracked wrist task is deweighted to zero), NOT by
dropping it from the stream.

Pose processing (_process_xr_pose) is identical to decoupled_wbc PicoStreamer /
quest_vive_bridge: XR y-up -> robot z-up, expressed relative to the headset and
de-yawed by the headset heading.
"""
import argparse
import json
import time

import numpy as np
from scipy.spatial.transform import Rotation as R
import zmq
import xrobotoolkit_sdk as xrt

from gear_sonic.utils.teleop.inspire.inspire_bridge import (
    HANDS_RATE_HZ,
    InspireBridge,
)
from gear_sonic.utils.teleop.inspire.inspire_dump import (
    INSPIRE_DUMP_PORT,
    InspireDumpPublisher,
)

# XR (Y-up) -> robot world (Z-up); identical to decoupled_wbc PicoStreamer.
R_HEADSET_TO_WORLD = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])


def _process_xr_pose(controller_pose, headset_pose):
    """Headset-relative, yaw-compensated 4x4 transform in robot frame.
    Poses are [x, y, z, qx, qy, qz, qw]."""
    xyz = np.asarray(controller_pose)[:3]
    quat = np.asarray(controller_pose)[3:7]
    if np.allclose(quat, 0):
        quat = np.array([0, 0, 0, 1])
    xyz = R_HEADSET_TO_WORLD @ xyz
    rot = R_HEADSET_TO_WORLD @ R.from_quat(quat).as_matrix() @ R_HEADSET_TO_WORLD.T

    h_xyz = np.asarray(headset_pose)[:3]
    h_quat = np.asarray(headset_pose)[3:7]
    if np.allclose(h_quat, 0):
        h_quat = np.array([0, 0, 0, 1])
    h_xyz = R_HEADSET_TO_WORLD @ h_xyz
    h_rot = R_HEADSET_TO_WORLD @ R.from_quat(h_quat).as_matrix() @ R_HEADSET_TO_WORLD.T

    delta = xyz - h_xyz
    yaw = R.from_matrix(h_rot).as_euler("xyz")[2]
    inv_yaw = R.from_euler("z", -yaw).as_matrix()

    T = np.eye(4)
    T[:3, :3] = inv_yaw @ rot
    T[:3, 3] = inv_yaw @ delta
    return T


def _to_vive_json_side(T):
    quat = R.from_matrix(T[:3, :3]).as_quat()  # xyzw
    pos = T[:3, 3]
    return {
        "position": {"x": float(pos[0]), "y": float(pos[1]), "z": float(pos[2])},
        "orientation": {
            "x": float(quat[0]), "y": float(quat[1]),
            "z": float(quat[2]), "w": float(quat[3]),
        },
    }


def _side_json(controller_pose, headset_pose):
    """Vive-json for one side; identity fallback if the pose is invalid/idle so
    the key is always present (the IK deweights the untracked side anyway)."""
    if (headset_pose is not None and controller_pose is not None
            and np.any(np.asarray(controller_pose)[:3])):
        return _to_vive_json_side(_process_xr_pose(controller_pose, headset_pose))
    return _to_vive_json_side(np.eye(4))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5555, help="ZMQ REP port (vive protocol)")
    ap.add_argument("--keyword", type=str, default="wrist",
                    help="Vive keyword; ViveStreamer reads left_<kw>/right_<kw>")
    ap.add_argument("--inspire-hands", type=str, default="off",
                    choices=["off", "trigger"], help="Drive Inspire hands in-process")
    ap.add_argument("--tracked_hands", "--tracked-hands", dest="tracked_hands",
                    type=str, default="both", choices=["both", "left", "right"],
                    help="Which side's Inspire hand to drive (poses always stream both)")
    ap.add_argument("--tap-port", type=int, default=5556,
                    help="ZMQ PUB copy of each vive frame for xr_rerun_logger; 0 disables")
    ap.add_argument("--inspire-dump-port", type=int, default=INSPIRE_DUMP_PORT,
                    help="ZMQ PUB dump of Inspire cmd/actual/force + tactile; 0 disables")
    args = ap.parse_args()

    tracked = {"both": ["left", "right"], "left": ["left"], "right": ["right"]}[args.tracked_hands]

    ctx = zmq.Context()
    inspire = None
    dump_pub = None
    xrt_ready = False
    try:
        # Bind dump before xrt.init so a leftover 5558 fails fast without
        # grabbing the XR SDK session.
        if args.inspire_hands != "off" and args.inspire_dump_port:
            dump_pub = InspireDumpPublisher(ctx=ctx, port=args.inspire_dump_port)

        print("[pico-bridge] xrt.init() -- the PICO PC service must already be running "
              "(~/start_xrsvc.sh) and the PICO app connected ...", flush=True)
        xrt.init()
        xrt_ready = True

        if args.inspire_hands != "off":
            def _hand_inputs():
                # (menu, left_trigger, right_trigger, left_grip, right_grip); untracked side forced open
                lt = xrt.get_left_trigger() if "left" in tracked else 0.0
                lg = xrt.get_left_grip() if "left" in tracked else 0.0
                rt = xrt.get_right_trigger() if "right" in tracked else 0.0
                rg = xrt.get_right_grip() if "right" in tracked else 0.0
                return (False, lt, rt, lg, rg)

            inspire = InspireBridge(
                get_inputs=_hand_inputs, mode="trigger", sides=tracked,
                rate_hz=HANDS_RATE_HZ,
                dump_publisher=dump_pub)
            inspire.start()

        sock = ctx.socket(zmq.REP)
        sock.bind(f"tcp://*:{args.port}")
        print(f"[pico-bridge] serving vive protocol on tcp://*:{args.port} "
              f"(keyword '{args.keyword}', hand-side {tracked}). Waiting for PICO poses ...",
              flush=True)
        tap = None
        if args.tap_port:
            tap = ctx.socket(zmq.PUB)
            tap.bind(f"tcp://127.0.0.1:{args.tap_port}")
            print(f"[pico-bridge] observability tap PUB tcp://127.0.0.1:{args.tap_port}",
                  flush=True)

        kw = args.keyword
        last_log = 0.0
        poller = zmq.Poller()
        poller.register(sock, zmq.POLLIN)
        while True:
            try:
                events = dict(poller.poll(20))
            except KeyboardInterrupt:
                break
            out = {}
            try:
                head = xrt.get_headset_pose()
                lc = xrt.get_left_controller_pose()
                rc = xrt.get_right_controller_pose()
                out[f"left_{kw}"] = _side_json(lc, head)
                out[f"right_{kw}"] = _side_json(rc, head)
                # Thumbstick axes for locomotion (ViveStreamer maps them to navigate_cmd;
                # zeros when idle/disconnected -> zero velocity, safe failure mode)
                out["left_joystick"] = [float(v) for v in xrt.get_left_axis()]
                out["right_joystick"] = [float(v) for v in xrt.get_right_axis()]
                now = time.monotonic()
                if now - last_log > 1.0:
                    last_log = now

                    def _fmt(side):
                        d = out.get(f"{side}_{kw}")
                        if not d:
                            return "n/a"
                        p = d["position"]; o = d["orientation"]
                        e = R.from_quat([o["x"], o["y"], o["z"], o["w"]]).as_euler("xyz", degrees=True)
                        return (f"pos({p['x']:+.2f},{p['y']:+.2f},{p['z']:+.2f}) "
                                f"rpy({e[0]:+.0f},{e[1]:+.0f},{e[2]:+.0f})")

                    print(f"[pico-bridge] L {_fmt('left')} | R {_fmt('right')}", flush=True)
            except Exception as e:
                print(f"[pico-bridge] read error: {e}", flush=True)
            payload = json.dumps(out)
            if sock in events:
                try:
                    sock.recv_string()  # ViveStreamer request; content ignored
                    sock.send_string(payload)
                except KeyboardInterrupt:
                    break
            if tap is not None:
                tap.send_string(payload)
    finally:
        if inspire is not None:
            inspire.stop()
        if dump_pub is not None:
            dump_pub.close()
        if xrt_ready:
            xrt.close()


if __name__ == "__main__":
    main()

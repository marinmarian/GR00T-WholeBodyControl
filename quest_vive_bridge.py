#!/usr/bin/env python3
"""Quest -> decoupled_wbc IK bridge.

Hosts the CloudXR / IsaacTeleop runtime (via gear_sonic's IsaacTeleopClient) and
re-serves the Quest controller poses over the ZMQ "vive" protocol that
decoupled_wbc's ViveStreamer speaks, so the pink/pinocchio IK teleop pipeline can
be driven from a Meta Quest 3.

Runs on the HOST in .venv_teleop (which owns isaacteleop + the CloudXR runtime).
It REPLACES pico_manager_thread_server as the CloudXR host — only one CloudXR
runtime may exist, and the two teleop stacks are mutually exclusive anyway.

Controller poses are processed exactly like decoupled_wbc's PicoStreamer
(_process_xr_pose): XR y-up -> robot z-up, expressed relative to the headset and
de-yawed by the headset heading, so the operator can face any direction. The
resulting 4x4 is decomposed back to position + xyzw quaternion, which ViveStreamer's
get_transformation() reconstructs verbatim.

Optionally also drives the Inspire hands (trigger->fingers, grip->thumb) in-process
via gear_sonic's InspireBridge — independent of the IK body pipeline.

Usage (host, .venv_teleop active), then in the wbc container run:
  run_teleop_policy_loop.py --body_control_device vive \
      --body_streamer_ip <this-host-ip> --body_streamer_keyword wrist \
      --hand_control_device None
"""
import argparse
import json
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation as R
import zmq

from gear_sonic.utils.teleop.isaac_teleop_client import IsaacTeleopClient

# XR (Y-up) -> robot world (Z-up); identical to decoupled_wbc PicoStreamer.
R_HEADSET_TO_WORLD = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])


def _process_xr_pose(controller_pose, headset_pose):
    """Port of decoupled_wbc PicoStreamer._process_xr_pose: headset-relative,
    yaw-compensated 4x4 transform in robot frame."""
    xyz = np.asarray(controller_pose)[:3]
    quat = np.asarray(controller_pose)[3:]
    if np.allclose(quat, 0):
        quat = np.array([0, 0, 0, 1])
    xyz = R_HEADSET_TO_WORLD @ xyz
    rot = R_HEADSET_TO_WORLD @ R.from_quat(quat).as_matrix() @ R_HEADSET_TO_WORLD.T

    h_xyz = np.asarray(headset_pose)[:3]
    h_quat = np.asarray(headset_pose)[3:]
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5555, help="ZMQ REP port (vive protocol)")
    ap.add_argument("--keyword", type=str, default="wrist",
                    help="Vive keyword; ViveStreamer reads left_<kw>/right_<kw>")
    ap.add_argument("--inspire-hands", type=str, default="off",
                    choices=["off", "trigger"], help="Drive Inspire hands in-process")
    ap.add_argument("--use-adb", action="store_true", help="USB-local CloudXR transport")
    args = ap.parse_args()

    print("[bridge] starting CloudXR / IsaacTeleop runtime ...", flush=True)
    client = IsaacTeleopClient(use_adb=args.use_adb)
    client.start_streaming()

    inspire = None
    if args.inspire_hands != "off":
        from gear_sonic.utils.teleop.inspire.inspire_bridge import InspireBridge

        def _hand_inputs():
            # (menu, left_trigger, right_trigger, left_grip, right_grip)
            return (
                False,
                client.get_key_value_by_name("left_trigger"),
                client.get_key_value_by_name("right_trigger"),
                client.get_key_value_by_name("left_grip"),
                client.get_key_value_by_name("right_grip"),
            )

        inspire = InspireBridge(get_inputs=_hand_inputs, mode="trigger")
        inspire.start()

    ctx = zmq.Context()
    sock = ctx.socket(zmq.REP)
    sock.bind(f"tcp://*:{args.port}")
    print(f"[bridge] serving vive protocol on tcp://*:{args.port} "
          f"(keyword '{args.keyword}'). Waiting for Quest body data ...", flush=True)

    kw = args.keyword
    warned = False
    last_log = 0.0
    while True:
        try:
            _req = sock.recv_string()  # "get_vive_data"
        except KeyboardInterrupt:
            break
        out = {}
        try:
            head = client.get_pose_by_name("headset")
            lc = client.get_pose_by_name("left_controller")
            rc = client.get_pose_by_name("right_controller")
            if head is not None and lc is not None and np.any(np.asarray(lc)[:3]):
                out[f"left_{kw}"] = _to_vive_json_side(_process_xr_pose(lc, head))
            if head is not None and rc is not None and np.any(np.asarray(rc)[:3]):
                out[f"right_{kw}"] = _to_vive_json_side(_process_xr_pose(rc, head))
            if out:
                now = time.monotonic()
                if now - last_log > 1.0:
                    last_log = now
                    def _fmt(side):
                        d = out.get(f"{side}_{kw}")
                        if not d:
                            return "n/a"
                        p = d["position"]; o = d["orientation"]
                        eul = R.from_quat([o["x"], o["y"], o["z"], o["w"]]).as_euler("xyz", degrees=True)
                        return (f"pos({p['x']:+.2f},{p['y']:+.2f},{p['z']:+.2f}) "
                                f"rpy({eul[0]:+.0f},{eul[1]:+.0f},{eul[2]:+.0f})")
                    print(f"[bridge] L {_fmt('left')} | R {_fmt('right')}", flush=True)
            elif not warned:
                warned = True
                print("[bridge] connected, but no controller poses yet "
                      "(put headset on, wake controllers)", flush=True)
        except Exception as e:
            print(f"[bridge] read error: {e}", flush=True)
        sock.send_string(json.dumps(out))

    if inspire is not None:
        inspire.stop()
    client.close()


if __name__ == "__main__":
    main()

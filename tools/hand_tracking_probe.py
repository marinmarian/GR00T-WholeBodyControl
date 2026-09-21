#!/usr/bin/env python3
"""Probe PICO hand tracking through the XRoboToolkit SDK (g1-vr-teleop #35).

Run with the teleop DOWN (tools/sonic-teleop.sh down) and the xr-service up (~/start_xrsvc.sh),
headset on, PICO app connected to the PC service with hand tracking enabled:

    cd ~/GR00T-WholeBodyControl && source .venv_teleop/bin/activate && python tools/hand_tracking_probe.py

Prints, 5x per second and per hand: active flag, whether the 26 joints carry data, raw flexion
sums per finger (deg), the virtual trigger/grip the retargeter would send, the controller
trigger/grip for comparison, plus body-tracking availability and the body tracker's wrist
positions (the streamer's arm tracking comes from the body tracker, not from the controllers).

What to do while it runs:
  1. Hold the controllers, wiggle fingers: hand rows should say active=0 or no data.
  2. Put the controllers down, hands in view: active=1, data=yes, sums change when you curl.
  3. Curl ONE finger at a time (index, middle, ring, little, thumb): the matching column must move.
     If a different column moves, the joint order differs from OpenXR -> report it.
  4. Make a fist: 'fingers' trigger must read 1.00. Open flat: 0.00. Note the raw sums for tuning.
  5. Watch 'body' and the wrist positions with controllers down: if they stop updating, arm
     tracking will not work in hand-tracking mode.
"""
import argparse, time
import numpy as np
from gear_sonic.utils.teleop import hand_tracking as ht

ap = argparse.ArgumentParser()
ap.add_argument("--hz", type=float, default=5.0)
ap.add_argument("--synthetic", action="store_true", help="no SDK: feed a synthetic fist (self-test)")
a = ap.parse_args()

if a.synthetic:
    import sys; sys.path.insert(0, "tools/tests")
    from test_hand_tracking import synth_hand
    class _Fake:
        def get_left_hand_tracking_state(self): return synth_hand(70, 60)
        def get_right_hand_tracking_state(self): return synth_hand(0, 0)
        def get_left_hand_is_active(self): return 1
        def get_right_hand_is_active(self): return 1
        def get_left_trigger(self): return 0.0
        def get_right_trigger(self): return 0.0
        def get_left_grip(self): return 0.0
        def get_right_grip(self): return 0.0
        def is_body_data_available(self): return False
        def get_body_joints_pose(self): return np.zeros((24, 7))
        def init(self): pass
        def close(self): pass
    xrt = _Fake()
else:
    import xrobotoolkit_sdk as xrt

xrt.init()
print("SDK initialised. Ctrl+C to stop.", flush=True)
try:
    while True:
        rows = []
        for side, get_state, get_active in (("L", xrt.get_left_hand_tracking_state, xrt.get_left_hand_is_active),
                                            ("R", xrt.get_right_hand_tracking_state, xrt.get_right_hand_is_active)):
            j = np.asarray(get_state(), dtype=np.float64)
            active = int(get_active())
            has_data = j.shape == (26, 7) and not np.allclose(j[:, :3], 0.0)
            sums = ht.raw_flexion_sums(j) if has_data else None
            curls = ht.hand_curls(j) if has_data else None
            s = f"{side} active={active} data={'yes' if has_data else 'no '}"
            if sums:
                s += " sums(deg) " + " ".join(f"{k[:3]}={v:5.0f}" for k, v in sums.items())
            if curls:
                s += f" -> trigger {curls['fingers']:.2f} grip {curls['thumb']:.2f}"
            if has_data:
                w = j[ht.WRIST, :3]; s += f" wrist({w[0]:+.2f},{w[1]:+.2f},{w[2]:+.2f})"
            rows.append(s)
        ctrl = (f"controllers: Ltrig {xrt.get_left_trigger():.2f} Lgrip {xrt.get_left_grip():.2f} "
                f"Rtrig {xrt.get_right_trigger():.2f} Rgrip {xrt.get_right_grip():.2f}")
        body = xrt.is_body_data_available()
        b = f"body={'yes' if body else 'no '}"
        if body:
            bp = np.asarray(xrt.get_body_joints_pose(), dtype=np.float64)
            if bp.shape[0] >= 21:  # SMPL-24: 20 = left wrist, 21 = right wrist
                b += f" Lwrist({bp[20,0]:+.2f},{bp[20,1]:+.2f},{bp[20,2]:+.2f}) Rwrist({bp[21,0]:+.2f},{bp[21,1]:+.2f},{bp[21,2]:+.2f})"
        print(time.strftime("%H:%M:%S"), "|", rows[0], "|", rows[1], "|", ctrl, "|", b, flush=True)
        time.sleep(1.0 / a.hz)
except KeyboardInterrupt:
    pass
finally:
    xrt.close()

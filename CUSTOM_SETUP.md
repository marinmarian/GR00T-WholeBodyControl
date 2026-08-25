# Custom Teleop Setup (marinmarian fork)

**Day-to-day operations live in [RUNBOOK.md](RUNBOOK.md)** — quick start,
per-component commands, and recovery for every failure we have hit.

Notes and additions on top of upstream `NVlabs/GR00T-WholeBodyControl` for our G1 rig.
Covers hardware/network layout, the Inspire-hand integration, the Meta Quest 3 (CloudXR)
path, the PICO 4 (XRoboToolkit) path, and the decoupled_wbc IK pipeline. Everything here
is specific to our machine **mjolnir** (Jetson AGX Thor backpack) driving a Unitree G1.

---

## 1. Hardware & network

- **mjolnir** — Jetson AGX Thor, the compute driving the G1. Wired to the robot via a
  **TP‑Link TL‑SG1005P switch** on one flat `192.168.123.0/24` LAN (mjolnir `enP2p1s0` =
  `192.168.123.222`). G1 low‑level `.161`, G1 PC `.164`, Inspire hands `.210`/`.211`.
- **IGMP querier (required):** the unmanaged switch does IGMP snooping and there is no
  querier on the LAN, so it prunes the G1's DDS multicast (`rt/lowstate`, `rt/secondary_imu`)
  ~4 min after the deploy subscribes → the deploy dies with *"LowState or IMUState is not
  available"*. Fix: run `~/igmp_querier.py` on mjolnir (raw‑socket IGMPv2 general query to
  `224.0.0.1` every 60 s) for the whole session:
  ```bash
  sudo python3 ~/igmp_querier.py &
  ```
- **Headset Wi‑Fi:** mjolnir's built‑in Wi‑Fi runs as a dedicated 5 GHz hotspot
  **`mjolnir-xr`** (NM connection `quest-hotspot`, AP `10.42.0.1`) so the headset gets a
  low‑latency single hop (~1.6 ms vs 65–175 ms through the room router). Bring up with
  `nmcli con up quest-hotspot`. Docker sets the FORWARD policy to DROP, which breaks NM's
  shared‑mode NAT, so for headset **internet** re‑add:
  ```bash
  sudo iptables -t nat -A POSTROUTING -s 10.42.0.0/24 ! -d 10.42.0.0/24 -j MASQUERADE
  sudo iptables -I FORWARD 1 -i wlP1p1s0 -j ACCEPT
  sudo iptables -I FORWARD 1 -o wlP1p1s0 -j ACCEPT
  ```
  **PICO / XRoboToolkit on the hotspot (recommended for latency):** the room router adds
  65–175 ms to the headset vs ~1.6 ms on the hotspot — noticeable in both tracking and the
  Remote Vision camera stream. Setup (redo after any mjolnir reboot, AFTER docker is up
  since Docker resets the FORWARD policy to DROP):
  ```bash
  sudo nmcli con up quest-hotspot
  sudo iptables -I FORWARD 1 -i wlP1p1s0 -j ACCEPT
  sudo iptables -I FORWARD 1 -o wlP1p1s0 -j ACCEPT
  # on the robot PC, so camera video can reach the headset across subnets:
  ssh g1 'sudo ip route add 10.42.0.0/24 via 192.168.123.222'
  ```
  PICO joins `mjolnir-xr` (it will warn "no internet" — keep the connection; add the
  MASQUERADE rule above only if headset internet is wanted). In the app: **PC service =
  `10.42.0.1`** (not .222), Remote Vision stays `192.168.123.164`.

  **Camera-video gotchas learned the hard way:** (1) the g1 route above is what makes the
  camera stream reach the headset — persist it (`sudo nmcli con mod unitree1 +ipv4.routes
  "10.42.0.0/24 192.168.123.222"` on the robot PC), because a plain `ip route add` was
  silently lost and every "No route to host" of 2026-07-29 traced back to that; (2) do NOT
  relay/proxy the video through another host — Remote Vision kills streams whose source IP
  differs from the configured camera address after ~1-2 s (looks like a freeze); (3) the
  sender caps the headset's requested 20 Mbps at 8 Mbps by default (`--max-bitrate`). Verified working with
  the full right-arm-IK + dual-camera stack 2026-07-27.

---

## 2. Inspire RH56 hand integration (`gear_sonic/utils/teleop/inspire/`)

The upstream deploy only drives Unitree **Dex3** hands; our G1 has **Inspire RH56** hands.
This package drives them **independently** of the body pipeline, over Modbus TCP:

- `inspire_hand_modbus.py` — vendored Modbus TCP driver (hands at `192.168.123.210` /
  `.211`, port 6000; `1000` = open, `0` = closed).
- `inspire_bridge.py` — `InspireBridge`, a ~90 Hz (one-hand average) daemon mapping
  controller inputs to finger angles: **trigger → 4‑finger curl, grip/squeeze → thumb**.
  Cruise `FORCE_SET` = rest `FORCE_ACT` + 400 g; stall **holds** (no +40 retract);
  wrap tracks the trigger desired angle. `EMA_ALPHA=0.5` smooths cmd (not raw trigger).
- `inspire_dump.py` — ZMQ PUB `127.0.0.1:5558` topics `inspire_hand` / `inspire_tactile`
  (LINGER=0); last-run log `inspire-last-run.log`. No ROS topic. One Modbus client.
- `test_hands.py` — standalone connectivity check (`--cycle` opens/closes both hands).

Enable in the streamer with `--inspire-hands trigger`. This rig: `TRACKED_HANDS=right`.
Leftover process: `pgrep -af pico_vive_bridge` then `kill PID` (ports 5555/5556/5558);
do not kill the XR service. Dual-hand sequential Modbus cannot hold 90 Hz.
It runs alongside the (inert, Dex3) deploy output; for the decoupled_wbc pipeline
run that with `--no-with-hands` and let this bridge own the hands.

---

## 3. Meta Quest 3 path (CloudXR / IsaacTeleop, `--input-source isaac-teleop`)

The Quest connects via NVIDIA CloudXR through the **hosted web client**. Setup and gotchas:

- `~/cloudxr.env`: `NV_DEVICE_PROFILE=Quest3`, plus `NV_CXR_EMPTY_FRAME_WIDTH=1920` /
  `HEIGHT=1824` (headless streamer submits no frames; without a size the encoder crashes at
  0×0 → client error `0xF22300`).
- Client URL **must** force H.264: `.../IsaacTeleop/client/?codec=h264` (AV1 crashes NVENC on
  Thor). Accept the cert at `https://<host>:48322` first.
- Patched `.venv_teleop/.../isaacteleop/cloudxr/wss.py` to inject `{"hb":1}` heartbeats every
  10 s — the Quest browser throttles page JS in immersive mode and the runtime otherwise
  killed sessions at ~2 min.
- **Input limits (verified):** the web client provides trigger/grip inputs and a
  *server‑synthesized* 24‑joint body estimate (head + 2 controllers), but **no thumbstick
  clicks, no menu button, no reliable 6‑DoF controller poses**. Hence the `B+X` remap below,
  and why raw controller‑pose teleop (the IK bridge) is starved of clean input on the Quest.

### `pico_manager_thread_server.py` additions
- `--inspire-hands {off,trigger,handtracking}` — drive Inspire hands (see §2).
- `--vr3pt-scale` (0.8), `--vr3pt-clamp` (0.45 m), `--vr3pt-smooth` (0.3) — scale human reach
  into the G1 workspace, clamp out‑of‑reach targets, EMA‑smooth the streamed VR_3PT targets.
  Planner poll rate raised 20→50 Hz. Neck reference made **yaw‑only** (the synthesized neck's
  roll/pitch corrupts translation while cancelling for rotation).
- **`B+X`** accepted as the VR_3PT mode toggle (Quest browser sends no thumbstick click);
  the real Left‑Stick‑Click still works on PICO.
- **`--use-adb`** — USB‑local CloudXR transport (adb reverse + local coturn); needs `adb`
  and `coturn` installed and PICO/Quest developer‑mode USB debugging. Plumbed through to
  `IsaacTeleopReader(use_adb=…)`.

---

## 4. PICO 4 path (XRoboToolkit, `--input-source xrt`) — the native, crisp path

This is the device the stack is built for (real body + controller tracking).

- **PC service:** install `gear_sonic_deploy/thirdparty/roboticsservice_1.0.0.0_arm64.deb`
  → `/opt/apps/roboticsservice/`. It's built for Ubuntu 22.04 (ICU 70); mjolnir is 24.04
  (ICU 74), so fetch `libicu70_70.1-2_arm64.deb` and drop `libicu*.so.70*` into
  `/opt/apps/roboticsservice/lib/`. Start via `~/start_xrsvc.sh` (sets LD_LIBRARY_PATH,
  `exec ./RoboticsServiceProcess`) — needs a display; "release mode" = up.
- **PICO app:** XRoboToolkit APK + two ankle motion trackers paired & calibrated. Join
  `mjolnir-xr`, set **PC Service = `10.42.0.1`** (Status → **WORKING**), tick **Head,
  Controller, Full body** and **Send**. The streamer sits at *"waiting for body data"* until
  the trackers are active + Full body + Send are on — **do not engage before then.**

---

## 5. decoupled_wbc IK pipeline (manipulation‑grade upper‑body IK)

Pink/pinocchio QP IK (per‑wrist FrameTask + posture regularization; <1 cm/<1° in its unit
tests) running upper‑body IK on top of the learned balance/walk policy. Container `wbc-dev`
(built from `g1-deploy-dev`), venv `/opt/wbc-venv`, CycloneDDS built in‑container at
`/opt/cyclonedds`. Additions here:

- **`quest` body device** — `teleop_streamer.py` routes `quest`→`ViveStreamer` (vive ZMQ
  transport), and `wrists.py` treats `quest` like `pico` (identity calibration, no
  vive‑tracker rotation correction — that correction is asymmetric L/R and wrong for our
  headset‑relative poses).
- `quest_vive_bridge.py` — hosts CloudXR and re‑serves Quest controller poses over the vive
  protocol (port 5555) to the IK loop; optional in‑process `--inspire-hands trigger`.
- `bridge_poll.py` — diagnostic poller for that bridge.
- `ARM_VELOCITY_LIMIT` 6→8 rad/s in `joint_safety.py` — the balance policy snaps the arms
  from a hanging pose to nominal on activation and briefly exceeded 6; the strict limit
  latched safe mode (needs `docker restart wbc-dev`).

Known limitation: on the **Quest browser** client, `get_pose_by_name` controller poses are
unreliable (invalid), so the IK bridge is starved of clean input. Real controller poses come
from the **PICO** natively — that's the device to use for precise manipulation.

---

## 6. Run sheets

### SONIC whole‑body teleop with PICO (the "worked well" path)
Prereqs up: IGMP querier, hotspot, XRoboToolkit service, robot on+damp.
1. **Deploy** (in `g1-deploy-dev` via `gear_sonic_deploy/docker/run-ros2-dev.sh`):
   `./target/release/g1_deploy_onnx_ref enP2p1s0 policy/release/model_decoder.onnx reference/example/ --obs-config policy/release/observation_config.yaml --encoder-file policy/release/model_encoder.onnx --planner-file planner/target_vel/V2/planner_sonic.onnx --input-type zmq_manager --output-type all --zmq-host localhost` → `Init Done`.
2. **Streamer:** `.venv_teleop` → `python gear_sonic/scripts/pico_manager_thread_server.py --manager --input-source xrt --inspire-hands trigger`.
3. **PICO:** connect (§4), confirm live body data.
4. **Robot:** calibration pose → `A+B+X+Y` (stand) → `A+X` = whole‑body POSE, or
   `Left Stick Click` = upper‑body VR_3PT (legs on planner, sticks to walk). `O` / `A+B+X+Y`
   = e‑stop. **Never engage while the streamer says "waiting for body data."**

### decoupled_wbc IK pipeline
Stop the SONIC deploy first (both publish `rt/lowcmd`). Bridge (`quest_vive_bridge.py`) →
`run_g1_control_loop.py --interface real --no-with-hands --keyboard_dispatcher_type ros`
(or `-it` terminal for raw keyboard) → `run_teleop_policy_loop.py --body_control_device quest
--body_streamer_ip 127.0.0.1 --body_streamer_keyword wrist --hand_control_device None`.
Keys: `]` balance, `l` arms, `o` off. Restart the teleop loop whenever you restart the
control loop.

### Safety
Robot hoisted, area clear, e‑stop within reach. Killing the deploy with SIGKILL skips the
graceful damping command — prefer `O` / Ctrl‑C, and `L2+B` on the remote to damp.

---

## 7. Head camera in the PICO (XRoboToolkit Remote Vision)

The G1 head camera (Intel RealSense **D430i**, IR-only -- no RGB) can be shown as a 2D
screen inside the PICO **during teleop** via XRoboToolkit's **Remote Vision** video path,
which is a separate channel from the `--input-source xrt` tracking. We stream the left-IR
node (`/dev/video2`, GRAY8 640x480) HW-encoded to H.264 from the robot's onboard PC (a
Jetson Orin Nano). The installed `roboticsservice` PC-service is tracking-only and is
**not** the camera path.

Tooling lives in **`tools/headcam_pico/`**: `main_web_ir.cpp` (a drop-in for the
third-party `XR-Robotics/XRoboToolkit-Orin-Video-Sender`, ported from its
`main_zed_tcp.cpp` to capture the RealSense IR node instead of a ZED and to accept any
camera type), `run_headcam_sender.sh` (launch helper), and a README with full
build / run / protocol details.

Run on the robot PC: `./run_headcam_sender.sh` (listens on `0.0.0.0:13579`); in the PICO
set Remote Vision **camera source IP = <robot PC IP>**. The headset connects, sends
`OPEN_CAMERA` with its callback ip:port, and the sender streams H.264 back. Runs alongside
teleop. Known rough edge: the mono IR is upscaled to the headset's stereo canvas so it
looks stretched -- letterbox or a proper side-by-side split is a TODO.

---

## 8. Single-arm teleop (`--tracked_hands`) + one-command runner

The left arm on our G1 is hardware-disabled (failed temperature sensor on the left
shoulder-yaw motor; Unitree firmware disables the whole limb). For right-arm-only teleop
the decoupled_wbc pipeline gained a shared config flag **`--tracked_hands {both,left,right}`**
(default `both` = unchanged behavior):

- **teleop loop** — the untracked wrist's IK FrameTask gets zero position/orientation cost
  (`body_ik_solver.py`), so that arm settles to the posture-nominal instead of chasing a
  controller.
- **control loop** — `JointSafetyMonitor` stops monitoring the untracked arm
  (`joint_safety.py` `disabled_arms`), so the limp limb's passive motion can't trigger the
  critical velocity shutdown.

Pass the flag to BOTH `run_g1_control_loop.py` and `run_teleop_policy_loop.py`.

**PICO body input** for this pipeline runs through **`pico_vive_bridge.py`** (repo root, host
`.venv_teleop`): the wbc container has no `xrobotoolkit_sdk`/roboticsservice, so the bridge
reads the PICO SDK on the host and serves poses over the vive-ZMQ protocol (port 5555) —
the container teleop loop consumes it with `--body_control_device quest
--body_streamer_ip 127.0.0.1 --body_streamer_keyword wrist`. `--inspire-hands trigger`
also drives the Inspire hand(s) of the tracked side (right trigger -> fingers, grip -> thumb).

**One-command runner: `tools/g1-teleop.sh`** (deploy to `~/g1-teleop.sh` on the host).
`up` starts everything in a tmux session `g1` with ordering waits built in (xr-service ->
bridge; control loop -> teleop loop), `down` stops it, `status` health-checks. Windows:
`svc` = xr-service / head-cam / bridge, `run` = control + teleop loops (activation keys
`]` `l` `o` go in the left pane). The IGMP querier stays manual (`sudo python3
~/igmp_querier.py`, foreground) and the PICO app steps stay manual (PC service = mjolnir IP,
Remote Vision = robot-PC IP).

Known trip: a fast right-arm motion (or the engage snap) can exceed the 8 rad/s arm
velocity limit -> safe mode latches -> `docker restart wbc-dev`, rerun. Move gently for the
first seconds after `l`.

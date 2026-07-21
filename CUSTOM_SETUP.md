# Custom Teleop Setup (marinmarian fork)

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
  (Not needed for the PICO/XRoboToolkit path — that's a local link, no internet required.)

---

## 2. Inspire RH56 hand integration (`gear_sonic/utils/teleop/inspire/`)

The upstream deploy only drives Unitree **Dex3** hands; our G1 has **Inspire RH56** hands.
This package drives them **independently** of the body pipeline, over Modbus TCP:

- `inspire_hand_modbus.py` — vendored Modbus TCP driver (hands at `192.168.123.210` /
  `.211`, port 6000; `1000` = open, `0` = closed).
- `inspire_bridge.py` — `InspireBridge`, a ~60 Hz daemon mapping controller inputs to finger
  angles: **trigger → 4‑finger curl, grip/squeeze → thumb**. Works in any stream mode and
  with either input source. Opens hands and disconnects on exit.
- `test_hands.py` — standalone connectivity check (`--cycle` opens/closes both hands).

Enable in the streamer with `--inspire-hands trigger`. It runs alongside the (inert, Dex3)
deploy output; for the decoupled_wbc pipeline run that with `--no-with-hands` and let this
bridge own the hands.

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

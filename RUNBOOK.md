# G1 Teleop Runbook (SONIC whole-body + recording)

Operational guide for our rig as of 2026-08-25. Covers the one-command script,
every component run separately, and recovery for everything that has ever
broken. Machine names: **mjolnir** = Jetson Thor backpack (`192.168.123.222`,
hotspot AP `10.42.0.1`), **g1** = robot's onboard Orin Nano
(`192.168.123.164`, ssh alias `g1`), robot low-level = `192.168.123.161`.

## System map

| piece | runs on | notes |
|---|---|---|
| hotspot `mjolnir-xr` | mjolnir wifi | AP `10.42.0.1`, pw `g1teleop2026` |
| IGMP querier | mjolnir (sudo) | keeps robot DDS multicast alive |
| xr-service (`RoboticsServiceProcess`) | mjolnir | PICO tracking backend; **may be another user's — never kill** |
| streamer (`pico_manager_thread_server`) | mjolnir `.venv_teleop` | PICO → SONIC; drives Inspire hands |
| deploy (`g1_deploy_onnx_ref`) | mjolnir, `g1-deploy-dev` container | SONIC policy → robot; `O` = e-stop |
| camera sender (`OrinVideoSenderIR`) | mjolnir | D455f color + composite → headset + recording tee |
| head-IR push (gst-launch) | g1 | D430i IR → mjolnir via RTP (5600 video / 5601 recorder) |
| episode exporter + keys | mjolnir, `wbc-dev` container | LeRobot datasets + S3 upload |

Cameras: **D455f (color) on mjolnir USB**, **D430i (head IR) on g1**.
PICO enters exactly two addresses: **PC service `10.42.0.1`**, **Remote Vision `10.42.0.1`**.

---

## Quick start (script)

```bash
ssh mjolnir
sudo -v                    # cache sudo so the script can start hotspot+querier itself
~/sonic-teleop.sh up
tmux attach -t sonic
```

- `run` window: LEFT = streamer, RIGHT = **deploy** (`O` = e-stop). Wait for `Init Done`.
  Below them: exporter pane and **KEYS pane** (recording controls are typed there).
- `svc` window (`Ctrl-b n`): xr-service / g1 IR push / thor camera sender.
- `~/sonic-teleop.sh status` — health table. `~/sonic-teleop.sh down` — stop everything
  (robot first: `A+B+X+Y` or `O`, then `L2+B` damp).
- Dataset naming: `DATASET=stack_cups TASK="stack the cups" ~/sonic-teleop.sh up`

### PICO setup
1. WiFi → `mjolnir-xr` (keep connection despite "no internet")
2. App → PC service `10.42.0.1` → **WORKING**; tick **Head + Controller + Full body** and **Send**;
   ankle trackers powered, paired, **calibrated**
3. Remote Vision → `10.42.0.1` (color left, head-IR right)
4. **Never activate while the streamer prints `waiting for body data`** — that message
   must stop first (it means trackers/Full body/Send aren't right yet).

### Drive
- Calibration pose → **one brief `A+B+X+Y`** → ~10 s → robot stands (PLANNER mode)
- **`A+X`** = whole-body POSE (full mimicry, both arms; triggers = fingers, grip = thumb)
- **`A+X`** again = back to PLANNER (upper body freezes; locomotion by sticks)
- Gaits in PLANNER: **`A+B`** = next, **`X+Y`** = previous: IDLE(0) → SLOW_WALK(1) →
  WALK(2) → RUN(3); left stick walks/strafes, right stick turns. **Stay in 1–3**
  (4+ are squat/kneel/lying/boxing/jump stunts that fight the hoist).
- Stop: `O` in deploy pane / `A+B+X+Y` / `L2+B` damp.

### Recording (KEYS pane or PICO controller)
- `c` start episode → `c` again stop & save → `g`/`v`/`b` rate good/neutral/bad
- `x` = discard while recording
- Controller (streamer maps gestures → UDP 5559 → the KEYS-pane bridge, so that
  pane must be running): **right-stick click** tap = `c`, hold ≥1.5 s = `x`;
  fallback **A+Y** held ¼ s = `c`. Ratings stay on the keyboard.
- Episodes: `~/GR00T-WholeBodyControl/outputs/<dataset>/` (LeRobot format:
  parquet + `ego_view` + `head_view` videos). Ratings sort copies into
  `recorded/{good,neutral,bad}/` and upload to `s3://darwin-robot-data/`
  (region eu-central-1; creds in `~/.aws` + inside wbc-dev). Upload failures
  warn and keep everything local — nothing is lost.
- Keep Remote Vision open while recording: camera frames only flow during a
  live headset video session.

---

## Running every piece separately

### Hotspot (mjolnir, sudo)
```bash
sudo nmcli con up quest-hotspot
```
Headset internet (optional, stops Android nagging):
```bash
sudo iptables -t nat -A POSTROUTING -s 10.42.0.0/24 ! -d 10.42.0.0/24 -j MASQUERADE
```

### IGMP querier (mjolnir, sudo) — REQUIRED before robot work
```bash
sudo nohup python3 ~/igmp_querier.py >/tmp/igmp.log 2>&1 &
```
Without it the switch prunes the robot's DDS multicast ~4 min after the deploy
subscribes → deploy dies with `LowState or IMUState is not available`.

### xr-service (mjolnir)
```bash
pgrep -f RoboticsServiceProcess || DISPLAY=:0 ~/start_xrsvc.sh
```
Wait for `release mode`. Needs a real terminal (dies when launched over
non-tty ssh). If it's already running it may be a colleague's instance — reuse it.

### Streamer (mjolnir)
```bash
cd ~/GR00T-WholeBodyControl && source .venv_teleop/bin/activate
python gear_sonic/scripts/pico_manager_thread_server.py --manager --input-source xrt --inspire-hands trigger
```

### Deploy (mjolnir)
```bash
cd ~/GR00T-WholeBodyControl/gear_sonic_deploy && ./docker/run-ros2-dev.sh
# inside the container:
./target/release/g1_deploy_onnx_ref enP2p1s0 policy/sonic_v1_1/model_decoder.onnx reference/example/ \
  --obs-config policy/sonic_v1_1/observation_config.yaml \
  --encoder-file policy/sonic_v1_1/model_encoder.onnx \
  --planner-file planner/target_vel/V2/planner_sonic.onnx \
  --input-type zmq_manager --output-type all --zmq-host localhost
```
Wait for `Init Done`. Alternative policies: swap the three `policy/sonic_v1_1/`
paths for `policy/release/` or `policy/low_latency/`. First run of a new policy
spends minutes building TensorRT engines.

### Camera sender — D455f color + composite (mjolnir)
```bash
cd ~/XRoboToolkit-Orin-Video-Sender && ./OrinVideoSenderIR --listen 0.0.0.0:13579 \
  --device /dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_455f_Intel_R__RealSense_TM__Depth_Camera_455f_254643069357-video-index0 \
  --pixfmt YUY2 --width 1280 --height 720 --fps 15 \
  --second-device udp:5600 --second-width 640 --second-height 480 --zmq-pub 5555
```
Drop everything from `--second-device` onward for color-only. `--zmq-pub 5555`
is the recording tee (ports 5556/5557 are taken by the streamer). Source lives
in `tools/headcam_pico/main_web_ir.cpp`; rebuild:
```bash
g++ -std=c++11 -O2 -I./asio-1.30.2/include $(pkg-config --cflags gstreamer-1.0 gstreamer-app-1.0 glib-2.0) \
  main_web_ir.cpp -o OrinVideoSenderIR $(pkg-config --cflags --libs libzmq) \
  $(pkg-config --libs gstreamer-1.0 gstreamer-app-1.0 glib-2.0) -lpthread
```

### Head-IR push (g1)
```bash
ssh g1
gst-launch-1.0 v4l2src device=/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_430i_Intel_R__RealSense_TM__Depth_Camera_430i_349623061587-video-index2 \
  ! video/x-raw,format=GRAY8,width=640,height=480,framerate=15/1 ! videoconvert ! video/x-raw,format=I420 \
  ! jpegenc quality=80 ! rtpjpegpay ! multiudpsink clients=192.168.123.222:5600,192.168.123.222:5601
```
5600 feeds the headset composite; 5601 feeds the dataset's `head_view` column.

### Episode exporter + recording keys (mjolnir, wbc-dev container)
```bash
docker start wbc-dev
~/wbc-exec.sh python decoupled_wbc/control/main/teleop/run_g1_data_exporter.py \
  --camera-host 127.0.0.1 --camera-port 5555 --dataset-name my_dataset \
  --task-prompt "describe the task" --data-collection --no-add-stereo-camera \
  --add-head-camera --no-text-to-speech
# separate terminal — recording keys need their own publisher in the SONIC stack:
~/wbc-exec.sh python /workspace/wbc/tools/record_keys.py
# (also bridges the PICO recording gestures: listens on udp://127.0.0.1:5559)
```

### IK stack (right-arm / decoupled) — different workflow entirely
`~/g1-teleop.sh {up,down,status}` — pink/pinocchio IK, `--tracked_hands right`,
keys `]`/`l`/`o` in the control-loop pane. See `CUSTOM_SETUP.md`.

---

## Troubleshooting (everything that has actually happened)

**Hotspot gone / PICO can't connect** — the rtl8852ce driver's power-save wedges
AP mode and NM takes its NAT rules down with the connection.
```bash
sudo nmcli con up quest-hotspot
# if that errors ("supplicant took too long", "not authorized"):
sudo systemctl restart wpa_supplicant NetworkManager && sleep 5 && sudo nmcli con up quest-hotspot
```
Permanent fix (do once):
```bash
echo "options rtl8852ce rtw_ips_mode=0 rtw_lps_mode=0" | sudo tee /etc/modprobe.d/rtl8852ce-hotspot.conf
sudo nmcli con mod quest-hotspot connection.autoconnect yes connection.autoconnect-priority 10
```
After any hotspot/NM restart the PICO must rejoin the WiFi and reconnect the app.

**No camera feed in Remote Vision**
1. Close the panel fully and reopen → `10.42.0.1` (the app single-accepts one
   video connection per request and gives up silently; a fresh open re-arms it).
2. Still nothing → restart the XRoboToolkit app completely, reconnect, retry.
3. Check the sender pane: `Connection refused` retrying = the headset's
   listener isn't up (app state); the sender knocks for 60 s per request.
4. **Never probe/connect to the headset's port 12345 while testing** — you will
   consume the app's only accept slot.
5. The video MUST originate from the hotspot subnet AND from the exact IP typed
   as camera source (the headset drops everything else). This is why the sender
   lives on mjolnir and Remote Vision points at `10.42.0.1`. Do not relay or
   route the video from another host.
6. Feeds connect but never render (2026 app versions): the client waits for an
   `OPEN_CAMERA_ACK` reply on the control connection before showing video, and
   double-fires `OPEN_CAMERA` (a second video connection gets mirrored). The
   sender handles both since commit `a170c63` — look for `Sent OPEN_CAMERA_ACK`
   in its pane; if missing, rebuild `OrinVideoSenderIR` from
   `tools/headcam_pico/`. This was the root cause of the Aug 24–25 outage.

**Camera present but pipeline dead / won't preroll** — RealSense firmware wedge
after a USB drop: unplug the camera, count to 10, replug. (The D455f's USB-C
cable drops often — replace it someday.) Check enumeration: `lsusb | grep -i 455f`.

**Right half of composite black** — the g1 IR push died (it stalls silently
sometimes). Restart the push; the color half keeps playing by design.

**Recording keys do nothing** — the KEYS-pane bridge (`record_keys.py`) must be
running (before 2026-08-28 it crashed on startup with an `IndexError` from
`keyboard_dispatcher` — pull if you see that). A working `c` prints
`Started recording N` in the exporter pane. If typing works but the controller
doesn't: check the streamer pane for `[EpisodeKeys]` lines (gestures are
detected there) — present there but not in KEYS means the UDP hop
(127.0.0.1:5559) is broken, i.e. the processes aren't on the same host.

**Deploy dies: `LowState or IMUState is not available`** — querier is down
(reboot kills it). Start it, rerun the deploy.

**Deploy: `CheckMode ... 3102 None` / `NoneType not subscriptable`** — cold DDS
discovery flake. Just rerun the same command; if it persists, power-cycle the robot.

**Deploy: `Waiting for planner ... timeout`** — the streamer had no body data
(PICO not sending Full body). Fix the headset side, rerun the deploy.

**Streamer stuck `waiting for body data`** — on the PICO: Full body + Send
ticked, both ankle trackers active and calibrated, PC service WORKING. Refuses
`A+B+X+Y` until this clears (by design).

**Robot unreachable (`.161`/`.164` no ping)** — robot is powered off or its
ethernet is loose. Everything mjolnir-side survives; restart the g1 push and
rerun the deploy after boot.

**sudo eats the password / commands echo as `command not found`** — never
background sudo directly: run `sudo -v` first (foreground), then the `&` command.

**Killing things by name** — `pkill -f pico_manager` and similar match their own
command line (and once killed the tmux server). Use the bracket trick:
`pkill -f "pico_manage[r]_thread"`, `pkill -f "OrinVideoSenderI[R]"`.

**Shared machine** — `wbc-thor` container and (sometimes) the running
xr-service belong to a colleague. Never kill processes you can't account for,
and commit working-tree changes promptly: uncommitted edits have vanished here.

**AWS/S3** — bucket `darwin-robot-data` (eu-central-1), IAM user
`robotics-developer`, creds in `~/.aws/credentials` (profiles `default` +
`darwin`) and copied into `wbc-dev:/root/.aws/`. Test:
`aws s3 ls s3://darwin-robot-data/raw/ --profile darwin` (or the boto3
one-liner in CUSTOM_SETUP.md). `InvalidAccessKeyId` = key
deactivated/rotated — ask the account owner.

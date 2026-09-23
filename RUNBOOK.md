# G1 Teleop Runbook (SONIC whole-body + recording)

Operational guide for our rig as of 2026-08-25. Covers the one-command script,
every component run separately, and recovery for everything that has ever
broken. Machine names: **mjolnir** = Jetson Thor backpack (`192.168.123.222`,
hotspot AP `10.42.0.1`), **g1** = robot's onboard Orin Nano
(`192.168.123.164`, ssh alias `g1`), robot low-level = `192.168.123.161`.

This fork is the working checkout on mjolnir. The team's canonical copy of this runbook and the project docs live in
**[prosus-robotics/g1-vr-teleop](https://github.com/prosus-robotics/g1-vr-teleop)** (an overlay of the same files):
experiment log `docs/EXPERIMENTS.md`, dataset/checkpoint registry `docs/DATA_AND_MODELS.md`, and the work tracked in
two milestones — [Bartending demo, AI House (Aug 2026)](https://github.com/prosus-robotics/g1-vr-teleop/milestone/2)
(closed) and [Restocking VLA: from first closed loop to reliable placement](https://github.com/prosus-robotics/g1-vr-teleop/milestone/1) (open).

## System map

| piece | runs on | notes |
|---|---|---|
| hotspot `mjolnir-xr` | mjolnir wifi | AP `10.42.0.1`, pw `g1teleop2026` |
| IGMP querier | mjolnir (sudo) | keeps robot DDS multicast alive |
| xr-service (`RoboticsServiceProcess`) | mjolnir | PICO tracking backend; **may be another user's — never kill** |
| streamer (`pico_manager_thread_server`) | mjolnir `.venv_teleop` | PICO → SONIC; drives Inspire hands |
| deploy (`g1_deploy_onnx_ref`) | mjolnir, `g1-deploy-dev` container | SONIC policy → robot; `O` = e-stop |
| camera sender (`OrinVideoSenderIR`) | mjolnir | OBSBOT Tiny 2 Lite color + composite → headset + recording tee |
| head-IR push (gst-launch) | g1 | D430i IR → mjolnir via RTP (5600 video / 5601 recorder) |
| episode exporter + keys | mjolnir, `wbc-marin` container | LeRobot datasets + S3 upload |

Cameras: **OBSBOT Tiny 2 Lite (color) on mjolnir USB** (replaced the D455f 2026-09-07), **D430i (head IR) on g1**.
PICO enters exactly two addresses: **PC service `10.42.0.1`**, **Remote Vision `10.42.0.1`**.

---

## Quick start (script)

```bash
ssh mjolnir
sudo -v                    # cache sudo so the script can start hotspot+querier itself
~/sonic-teleop.sh up       # symlink -> ~/GR00T-WholeBodyControl/tools/sonic-teleop.sh (since 2026-09-21; it was a stale copy before)
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

### Hand tracking instead of triggers (experimental, 2026-09-21, issue #35)

`INSPIRE_HANDS=handtracking ~/sonic-teleop.sh up` makes the Inspire hands follow your fingers instead
of the triggers: **each finger on its own** (little, ring, middle, index from the finger curl, thumb bend
from the thumb-across-palm distance; thumb rotation stays at rest), same bridge, same force logic per
finger, same recorded hand state. Everything else stays on the controllers: `A+X` mode toggle,
`A+B+X+Y` e-stop, gaits, sticks, recording gestures — the streamer has no keyboard control, so
**keep the controllers within reach**; `O` in the deploy pane still works from the keyboard.

- PICO app: enable hand tracking in the XRoboToolkit app and put the controllers down; the headset
  switches to hands on its own. A hand that is not tracked (out of view, controller picked up) holds
  its last pose for 0.5 s, then follows that side's controller trigger/grip again.
- First time / after headset updates, run the probe with the teleop **down** and the xr-service up:
  `cd ~/GR00T-WholeBodyControl && source .venv_teleop/bin/activate && python tools/hand_tracking_probe.py`
  (curl one finger at a time, check the matching column moves; fist → trigger 1.00; watch `body=`
  and the wrist positions with controllers down — arm tracking comes from the body tracker).
- `HAND_TRACKING_DEBUG=1` prints once a second whether each side is driven by `hand`, `hold` or
  `controller`. Tuning constants (`FINGER_OPEN_DEG` …) live in `gear_sonic/utils/teleop/hand_tracking.py`.
- Measured 2026-09-21 with the probe: hands become `active=1` about **5 s** after the controllers are
  put down (before that the headset sends a placeholder pose, ignored); picking a controller up drops
  the hands within 1 s and its buttons register about **2 s** later. Body tracking (arms) keeps working
  with the controllers down. Joint order is OpenXR as assumed. Relaxed fingers 26-53°, fist 142-190°.
- Stopping: `A+B+X+Y` needs all four buttons **together** on both controllers, at least 2 s after
  pickup, and the streamer ignores a second combo within 3 s. `O` in the deploy pane is the sure stop.

### Recording (PICO controller or KEYS pane)
The script runs the **ZMQ exporter** (`gear_sonic/scripts/run_data_exporter.py`)
by default. `EXPORTER=ros2` selects the older ROS 2 exporter, which does not work
on this stack — see the ROS 2 entry in Troubleshooting.

- **left grip + A** = start episode → again = stop & save
- **left grip + B** = discard while recording
- Or type `c` / `x` in the KEYS pane (`tools/record_keys_zmq.py`, publishes on
  ZMQ 5580). No ratings on this path: `g`/`v`/`b` do nothing.
- **Prompt per episode** (tic-tac-toe, g1-vr-teleop #40): digits `1`-`9` in the KEYS
  pane set the prompt for the **next** episode to the matching cell (reading order:
  1 = top left … 5 = center … 9 = bottom right, template `PROMPT_TEMPLATE`, default
  `put a white piece in the {cell} cell`); `0` restores `$TASK`. While idle the change
  applies at once, while an episode is open it is queued until that episode is saved
  or discarded — an episode never carries two prompts. Each prompt becomes a row in
  `meta/tasks.jsonl`; the exporter stores only the row index per frame.
- `Started recording 0: "<prompt>"` in the **exporter** pane is the only proof it
  took, and of which prompt the episode carries. The KEYS pane echoing a key means
  nothing on its own.
- Episodes: `~/GR00T-WholeBodyControl/outputs/<dataset>/` (LeRobot v2.1: parquet
  + `ego_view` and `head_view` videos, both 640x480 @ 50 fps).
- Each saved episode uploads to `s3://$DATASET_BUCKET/raw/<dataset>/` (default
  bucket `darwin-robot-data`, profile `darwin`, region eu-central-1) on a
  background thread. Local files are the source of truth; a failed upload only
  warns. On exit the exporter waits up to 5 min for uploads still in flight —
  without that wait they die with the process (they are daemon threads) and a
  large episode silently never reaches S3. If it gives up, or the exporter was
  killed outright, catch the bucket up with:
  ```bash
  ~/wbc-marin-exec.sh python tools/s3_resync_dataset.py \
    --dataset-path outputs/<dataset>          # add --dry-run to just look
  ```
  It compares every local file against the bucket and uploads what is missing or
  a different size. Safe to re-run; it never deletes.
- Still no episode ratings (`g`/`v`/`b`) and no `raw/`+`recorded/` quality split
  on this path — those are ROS 2 exporter features.
- **Camera frames flow from launch** (sender started with `--autostart`, 2026-09-23): the
  ZMQ 5555 recording tee no longer waits for a headset Remote Vision session. Opening
  Remote Vision restarts the capture pipeline for the headset and the tee follows within
  a second. Do **not** close Remote Vision mid-episode: `CLOSE_CAMERA` tears the pipeline
  and the tee down with it (see gotchas).
- **Pre-flight, before the first gesture:** the exporter pane must have stopped printing
  `Waiting for message. Avail msg: proprio X | image Y` (once per second). `image False` =
  nothing on the tee: look at the sender pane (`svc` window); the OBSBOT parks its gimbal
  far off centre when it sleeps (`tilt_absolute` in the 100000s) — re-centre it with
  `v4l2-ctl -d /dev/video0 -c pan_absolute=0,tilt_absolute=0,zoom_absolute=0` and restart
  the sender pane if the tee stays silent. `proprio False` = the deploy's control loop is
  not running yet: the robot has to be started (`A+B+X+Y`) before state reaches the exporter.
- The exporter pane is recorded to `logs/exporter-last-run.log` (both launchers); read it
  after `down` when a session produced fewer episodes than expected. Half-initialised
  datasets (meta only) must be removed before relaunching with the same `DATASET`:
  `docker exec wbc-marin rm -rf /workspace/wbc/outputs/<dataset>`.

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

### Camera sender — OBSBOT color + composite (mjolnir)
```bash
cd ~/XRoboToolkit-Orin-Video-Sender && ./OrinVideoSenderIR --listen 0.0.0.0:13579 \
  --device /dev/v4l/by-id/usb-Remo_Tech_Co.__Ltd._OBSBOT_Tiny_2_Lite-video-index0 \
  --pixfmt MJPG --width 1280 --height 720 --fps 30 \
  --second-device udp:5600 --second-width 640 --second-height 480 --zmq-pub 5555 --autostart
```
`--autostart` runs the capture and the ZMQ tee from start-up instead of waiting for the
headset's OPEN_CAMERA (default in both launchers since 2026-09-23). The sender has no
GStreamer bus watch: a pipeline that dies keeps the pane quiet, so an empty tee with the
OBSBOT enumerated means "restart this pane", not "check the camera".
The OBSBOT (UVC webcam, `video-index0`; `index1` is its metadata node) only offers
raw YUYV at 640x480, so we take its **MJPEG** 720p mode — `--pixfmt MJPG` makes the
sender insert a software `jpegdec` (`nvjpegdec` rejects this camera's stream) before
the HW H.264 encoder; measured 30 fps sustained at 720p and 1080p on mjolnir. Keep the
camera's **AI tracking off** (the gimbal must not hunt during teleop) and re-centre it
if it was moved: `v4l2-ctl -d <device> -c pan_absolute=0,tilt_absolute=0,zoom_absolute=0`.
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

### Episode exporter + recording keys (mjolnir, wbc-marin container)
```bash
docker start wbc-marin
~/wbc-marin-exec.sh python gear_sonic/scripts/run_data_exporter.py \
  --camera-host 127.0.0.1 --camera-port 5555 --dataset-name my_dataset \
  --task-prompt "describe the task" --no-text-to-speech
# separate terminal — optional for c/x (left grip + A / left grip + B work without
# it), required to change the prompt per episode: digits 1-9 = cell prompt, 0 = the
# base prompt; --line-mode adds `t <text>` for free text (Enter after each command).
~/wbc-marin-exec.sh python /workspace/wbc/tools/record_keys_zmq.py \
  --base-prompt "describe the task" [--prompt-template "put a white piece in the {cell} cell"] [--line-mode]
```
This exporter has **no ROS 2 dependency**: robot state and `robot_config` come
from the C++ deploy's ZMQ output (`g1_debug` topic, port 5557 — verified carrying
`body_q`, `body_dq`, `last_action`, `base_quat`, plus a `robot_config` frame every
~2 s), the camera from the sender on 5555, and the recording toggles from the
streamer's `manager_state` topic on 5556.

**Hand state comes from the Inspire bridge, not the deploy.** The C++ deploy only
knows Dex3 hands, so its `left/right_hand_q` is all zeros with Inspire hands and
the first `restocking` episode recorded 14 zero columns in `observation.state`.
Now the streamer's `InspireBridge` publishes each tick's measured finger angles on
`tcp://127.0.0.1:5558` (topic `inspire_hand`, `--inspire-hands trigger` is
enough) and the exporter maps them onto the 7-DoF G1 hand joints — same solver
and open/closed poses as `teleop.*_hand_joints`, fingers from the mean of the four
finger DOFs, thumb from thumb-bend. Proof it works: streamer pane prints
`[InspireDump] PUB tcp://127.0.0.1:5558`, exporter pane prints
`[InspireDump] hand-state SUB` and NO `[Inspire] ... hand state unavailable`
lines while recording. If those warnings appear the frame falls back to the C++
zeros (hands unreachable, or snapshot older than `--inspire-state-max-age`, 0.25 s).
`--inspire-dump-port 0` disables it. `script_config.hand_state_source` in
`meta/info.json` says `inspire` or `cpp`.

The old ROS 2 exporter is still there behind `EXPORTER=ros2` and buys ratings
(`g`/`v`/`b`), S3 upload and a `head_view` column, but it cannot start on this
stack — see below.
Startup prints `[robot_config] received from topic WBCPolicy/robot_config` once the
deploy is up (the C++ deploy publishes the config as a topic; the IK stack's Python
loop serves it as a service — `sonic_compat.py` accepts either). The C++ state
message (`body_q`/`last_action`, MuJoCo order) is translated into the dataset's
`observation.state`/`action` columns and the wrist poses come from FK. The C++
deploy does not publish the locomotion command, so `teleop.navigate_command` and
`teleop.base_height_command` are recorded at their defaults on this stack.
`~/wbc-marin-exec.sh` must set `FASTRTPS_DEFAULT_PROFILES_FILE` to
`tools/fastdds_udp_only.xml` — see the DDS troubleshooting entry below.

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
   (The sender copy built on g1 is still pre-ACK — rebuild it before ever
   running the sender on g1 again.)

**Camera present but pipeline dead / won't preroll** — for the D430i on g1 this is
the RealSense firmware wedge after a USB drop: unplug the camera, count to 10, replug.
For the OBSBOT on mjolnir check enumeration (`lsusb | grep -i obsbot`, the by-id
symlink in the sender command must exist) and that nothing else holds the node
(`fuser /dev/video0`); `--pixfmt MJPG` must match a mode from
`v4l2-ctl -d /dev/video0 --list-formats-ext` (720p/1080p/4K are MJPEG-only).

**Right half of composite black** — the g1 IR push died (it stalls silently
sometimes). Restart the push; the color half keeps playing by design.

**Exporter stuck on `service not available, waiting again...`** — it never reached
the recording loop, so every `c` is dropped even though the KEYS pane shows them.
Before 2026-09-07 the exporter waited for a ROS *service* that only the IK stack's
Python loop provides; the C++ deploy publishes the config as a topic. Pull. If it
still waits with the deploy running, check DDS (next entry).

**Exporter (`EXPORTER=ros2`) stays on `[robot_config] waiting ...` with the deploy
up** — the deploy publishes NOTHING over ROS 2, so this exporter can never start
and every keypress is silently dropped. Measured 2026-09-07 from inside
`g1-deploy-dev`: no `g1_output_handler` node exists, `/G1Env/env_state_act` does
not exist, and `/WBCPolicy/robot_config` shows `Publisher count: 0` with
`Subscription count: 1` (the exporter). The binary does support it
(`./target/release/g1_deploy_onnx_ref` with no args prints
`--output-type <zmq|all|ros2>`, so `HAS_ROS2=1`) and it is launched with
`--output-type all`; why the handler is not live at runtime is still unexplained —
its startup lines had scrolled out of the tmux pane.

**This is NOT a DDS problem.** The earlier theory (Fast DDS shared memory cannot
cross the deploy container's `--ipc host` boundary, hence
`tools/fastdds_udp_only.xml` and `ROS_LOCALHOST_ONLY`) is wrong: discovery across
the two containers works fine — from `g1-deploy-dev`, a daemon-free
`ros2 topic list` sees wbc-marin's `/data_exporter` and `/record_keys_bridge`.
Don't spend time on DDS profiles here.

To diagnose properly: restart the deploy and read its first ~40 lines. Either
`Initialized ROS2 output interface` + `[ROS2 Output] Published robot config to
WBCPolicy/robot_config (immediate)` appear (then the problem is downstream), or a
`[ROS2 Output ERROR] ...` line names the cause. Until then use the default ZMQ
exporter, which needs none of this.

**Recording keys do nothing** — a working toggle prints `Started recording N` in
the **exporter** pane; anything the KEYS pane echoes is only proof it read your
keyboard. On the default ZMQ path, `left grip + A` also prints
`[Manager] recording toggle sent: ...` in the streamer pane — present there but
nothing in the exporter means the exporter is not reading 5556, absent there means
the headset is not reporting the grip or A. Keyboard `c`/`x` need the KEYS pane
(`tools/record_keys_zmq.py`) running, and `g`/`v`/`b` genuinely do nothing on this
path (no ratings).

Before 2026-09-07 the streamer published `manager_state` on every iteration of an
unthrottled loop (~50 kHz), and the exporter drains only 20 messages per 50 Hz
tick — so a one-shot toggle edge had ~2% odds of being read at all, and `pose` was
buried under the flood. The manager loop now polls at 500 Hz, publishes
`manager_state` at 50 Hz, and latches toggle edges so none are lost between sends.
If toggles feel unreliable, check that fix is still in
`pico_manager_thread_server.py` (`MANAGER_STATE_PERIOD_S`).

On `EXPORTER=ros2` only: keys go through `record_keys.py` and the controller
gestures hop over UDP 127.0.0.1:5559 — check the streamer pane for `[EpisodeKeys]`
lines.

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
`darwin`) and bind-mounted into `wbc-marin:/root/.aws`. Test:
`aws s3 ls s3://darwin-robot-data/raw/ --profile darwin` (or the boto3
one-liner in CUSTOM_SETUP.md). `InvalidAccessKeyId` = key
deactivated/rotated — ask the account owner.

## VLA inference (GR00T N1.7 restocking policy)

Runs the fine-tuned GR00T N1.7 policy on the real G1. Since 2026-09-21 **everything runs on mjolnir**,
policy server included (`POLICY_MODE=local`, the default); the EC2 GPU box (`darwin-gpu`) behind an SSH tunnel
is the fallback (`POLICY_MODE=ec2`). Same SONIC deploy, cameras and Inspire hands as teleop;
`run_vla_inference.py` takes the PICO streamer's place on ZMQ 5556/5557.

### What runs where

| piece | where | notes |
|---|---|---|
| PolicyServer (Isaac-GR00T), **local** | mjolnir, `vla` session, `svc` window, pane `serve` | `~/g1-vr-teleop/rig/thor/serve_policy.sh best34\|v2` on `127.0.0.1:5550`, started by the launcher. Thor venv from `rig/thor/install_thor.sh` (Isaac-GR00T @ `51d4c89`, JetPack 7.1 / CUDA 13.0 stack), checkpoints `~/checkpoints/restocking_{best34,v2}/checkpoint-10000` (from S3, `rig/thor/s3_pull_checkpoints.py`). Loads ~12 GB, takes ~1 min (56 s measured; the very first start also downloads the 4.6 GB Cosmos backbone from HF, ~10 min) to come up. Bench 2026-09-21: 145 ms median / 165 ms p95 per 40-step chunk in-process, 0.15–0.17 s round trip over loopback (first call after start 0.77 s warm-up), 6 GiB GPU. |
| PolicyServer, **EC2 fallback** | darwin-gpu, tmux `policy` + SSH tunnel to 5550 from the `svc` window | `POLICY_MODE=ec2 tools/vla-inference.sh up` and `ssh darwin-gpu '~/serve_policy.sh v2\|best34'` yourself. EC2 does not open 5550; mjolnir's ed25519 key is authorised there. 0.25 s per inference on the bench, 0.40–0.54 s in the runs (two raw frames through the office uplink). |
| head-IR push | g1 → mjolnir udp 5600/5601 | started automatically once g1 answers ping |
| camera sender | mjolnir, `svc` window | teleop command **+ `--autostart`**: capture and the ZMQ 5555 tee run without a PICO Remote Vision session. `ego_view` = OBSBOT side view, `head_view` = D430i IR. |
| C++ deploy | mjolnir, `run` window, `g1-deploy-dev` container | `./run_sonic.sh` = the teleop command (`--input-type zmq_manager --output-type all`). |
| `run_vla_inference.py` | mjolnir `.venv_inference` | camera keys from the server, Inspire hand state from dump port 5558, initial pose token from our own episodes, training prompt as default |
| `inspire_vla_bridge.py` | mjolnir | policy hand actions → Inspire hands (Modbus) and measured hand state → 5558. Without it the hands never move (the deploy only knows Dex3) and the hand state is zeros. |
| `tools/vla_keys.py` | mjolnir | operator keys → ZMQ 5580 |

### Before you start

1. Robot powered, **on the hoist**, in damping. Inspire hands powered (192.168.123.210/.211).
2. Cameras: OBSBOT on mjolnir USB, D430i on g1 — same physical placement as during recording.
   Check with the training-vs-live picture: `~/Desktop/restocking_eval/live/cam_compare.sh` on the Mac
   (needs the sender running). The side camera must show the table the way the recordings do.
3. **Teleop must be down** (`tools/sonic-teleop.sh down`): the PICO streamer and the VLA client both bind 5556.
4. `sudo -v` on mjolnir so the launcher can start the IGMP querier. Without the querier the switch drops the
   robot's DDS multicast ~4 min after the deploy subscribes and the deploy dies at `k` with
   `LowState or IMUState is not available` (happened on the first run).
5. Policy server: nothing to do in local mode — the launcher starts it (`POLICY_MODEL=best34` default, `v2` the other
   one) and the inference pane waits until it listens. First time on a freshly set-up mjolnir: `rig/thor/README.md`
   (install, checkpoint pull, HF token). EC2 fallback only:
   ```bash
   ssh darwin-gpu '~/serve_policy.sh best34'      # or v2; keeps running in tmux "policy"
   POLICY_MODE=ec2 ~/GR00T-WholeBodyControl/tools/vla-inference.sh up
   ```

### Run

```bash
sudo -v
~/GR00T-WholeBodyControl/tools/vla-inference.sh up      # POLICY_MODEL=v2 POLICY_MODE=ec2 PROMPT="..." HANDS=0 overrides
tmux attach -t vla
```

Window `run` (the one you land in), panes:

```
 ┌───────────────────────────┬───────────────────────────┐
 │ deploy (container)        │ inference                 │
 │ Init Done / O = e-stop    │ run_vla_inference.py      │
 ├───────────────────────────┼───────────────────────────┤
 │ keys  ← type here         │ hands (Inspire bridge)    │
 └───────────────────────────┴───────────────────────────┘
```
Move between panes with `Ctrl-b` + arrow. Window `svc` (`Ctrl-b n`): serve (local policy server; tunnel in ec2 mode),
cam-g1, cam-sender.

**Type only in the keys pane.** The deploy pane reads keystrokes too: `Enter` toggles ZMQ streaming,
`o` stops, `i` re-initialises — and arrow keys in the keys pane insert `^[[A` garbage (`Ctrl-U` clears
the line).

Then, in order:

| step | do | expect |
|---|---|---|
| 1 | wait | serve pane (svc window): model loading, then the ZMQ bind. deploy pane: `Init Done`. inference pane: `waiting for policy server on :5550 ...` until the server is up, then `PolicyServer is reachable`, `Policy video keys … ['head_view', 'ego_view']`, `Hand state source: Inspire bridge`, then image-latency lines for **both** views and `waiting for state msg` (normal until step 2). hands pane: `InspireL/InspireR: rest pose …`, `[VLAHands] running`. |
| 2 | keys: `k` ⏎ | deploy: `Planner enabled` then no timeout; the robot comes under power and stands under the planner (hoist!). inference: `New action chunk (… latency …)` lines start (policy still paused) — expect ~0.15 s with the local server, 0.4–0.5 s via EC2. |
| 3 | keys: `i` ⏎ | robot blends (1 s) to the initial pose taken from our demonstrations. |
| 4 | keys: `p` ⏎ | policy drives the robot. `p` again pauses (see gotchas), `x` stops the C++ loop, `t <text>` changes the prompt — **keep the training prompt** `put bottles with red cap in red bottle holder`. |

E-stop: `O` in the deploy pane (or A+B+X+Y on the controllers if the streamer were running — it is not).

### Stop

```bash
~/GR00T-WholeBodyControl/tools/vla-inference.sh down    # kills session, inference, bridge (opens hands), sender, deploy, g1 push
```
`down` also kills the local policy server. In ec2 mode the server on darwin-gpu keeps running; stop it with
`ssh darwin-gpu 'tmux kill-session -t policy'`.

### Gotchas

- **`p` (pause) opens the hands after 2 s.** No actions for >2 s = comms loss for the bridge → both hands open
  (`--action-max-age`). A held bottle is dropped. Deliberate safety default; `--action-max-age 1e9` holds the last grasp.
- **Frozen head view is rejected.** If the g1 push dies the sender re-sends the last `head_view`; observations whose
  views differ by >0.5 s are dropped (`MAX_VIEW_SKEW_S`, log `stale camera view(s)`).
- **Sensor loss pauses the policy by itself** (since 2026-09-21, `--sensor-loss-pause-s`, default 1 s): after 1 s
  without a valid observation (a view missing or stale, or no robot state) the inference pane prints a red
  `SENSOR LOSS … policy PAUSED` line, stops sending actions and stays paused until you press `p` — it no longer
  resumes when the sensor comes back (it did on 2026-09-11 and the robot moved unprompted). Same consequence as a
  manual `p`: the Inspire bridge opens the hands 2 s later, so a carried bottle is dropped. When the sensor is back
  the pane prints `Observations valid again - policy still PAUSED`; then `p`. `--auto-resume-after-sensor-loss`
  restores the old behaviour, don't.
- **Hands open on Ctrl-C / SIGTERM / crash of the bridge**, not on SIGKILL or power loss.
- **Do not open-then-close PICO Remote Vision during a run**: `CLOSE_CAMERA` tears down the capture pipeline and
  the ZMQ tee with it. Either keep the headset out of it (`--autostart`) or leave the session open.
- **`k` always STARTs, `x` STOPs.** The upstream `k` toggle relied on the client's own memory; after a deploy
  restart it sent STOP for START and the deploy quit (second run, 2026-09-11).
- **Deploy died at `k` with `LowState or IMUState is not available` → `Planner initialization timeout`**: IGMP
  querier down (see "Before you start"). Lowstate itself: `~/eval_vla/lowstate_check.py enP2p1s0` → ~1 kHz.
  Restart the deploy from the container prompt with `./run_sonic.sh`.
- **`Address already in use 5550`**: a previous server is still on the port; both `serve_policy.sh` (Thor and darwin-gpu)
  stop it first, `vla-inference.sh up` kills a stale local server, or `tmux kill-session -t policy` on darwin-gpu.
- **Inference pane started before the server → `Policy video keys` missing / single `ego_view`.** `run_vla_inference.py`
  asks the server for its camera keys once at start-up and silently falls back to `ego_view` only. The launcher now
  waits for the port; if you start the client by hand, start it after the server listens.
- **Local server: never `uv run`/`uv sync` in `~/Isaac-GR00T`.** Its root pyproject targets x86_64 cu128 and would
  replace the Thor venv. `source ~/g1-vr-teleop/rig/thor/env.sh` and use plain `python` (the scripts do).
- **Local server and the C++ deploy share the Thor GPU.** Verified on the robot 2026-09-21 (closed loop ran with the
  local server, controller healthy — Marin). If it ever needs re-checking, the check is the normal flow, nothing extra: `up`, wait for the serve pane to listen and `Init Done`, `k`.
  The client now requests a chunk every 0.4 s while the policy is still paused (step 2), so the GPU load is real —
  watch the deploy pane's timing lines (LowState age, policy, motor command) for a minute before `i`/`p`. Do not run
  `bench_policy.py` next to a live loop: it loads a second copy of the model.
- `install_scripts/install_inference.sh` is broken under uv (upstream too: dependency named `Isaac-GR00T`, package is
  `gr00t`); `.venv_inference` was built by hand (gear_sonic + pyzmq msgpack msgpack-numpy pin tyro opencv scipy
  pymodbus==3.13.1) and `run_vla_inference.py` falls back to the vendored `gear_sonic/utils/inference/gr00t_client.py`.

- **Head camera gone: inference pane says `camera message lacks ['head_view']`, then `SENSOR LOSS … PAUSED`.** The
  D430i has dropped off g1's USB bus (`ssh g1 lsusb` shows no `8086:0b4b`, `/dev/v4l/by-id` empty); the `svc` cam-g1 pane
  says `head camera device missing`. Unplug it at the robot, count to 10, replug. The cam-g1 pane restarts the push on
  its own (watchdog loop on g1, since 2026-09-21), the inference pane reports `Observations valid again`, and only
  your `p` resumes the policy. Check the picture is live (image-latency lines for both views) before pressing it.
- **Wrist motors overheat during long hovers.** The policy tends to hold the bottle raised at full reach; the wrist
  motors heat up and fault before placement. `f` in the deploy pane prints motor temperatures — check before each run,
  let the wrists cool between attempts, don't restart while hot.
- **Policy round trip 0.4–0.5 s in the real runs** (bench: 0.25 s): two raw 640x480 frames per request through the
  office uplink. Sending JPEGs is the planned fix; until then expect hesitant motion.

### Recording closed-loop attempts as episodes (`RECORD=1`, 2026-09-23, issue #25)

```bash
RECORD=1 DATASET=policy_2026-09-23 ~/GR00T-WholeBodyControl/tools/vla-inference.sh up   # + POLICY_MODEL, PROMPT as usual
```
Same stack as above plus the ZMQ exporter (wbc-marin container, `--add-head-camera`, S3 upload to
`s3://$DATASET_BUCKET/raw/$DATASET/`) and `record_keys_zmq.py` in the keys pane instead of `vla_keys.py`.
No headset, no streamer. **Every key needs Enter** (line mode): `k` start C++ loop, `i` initial pose, `p`
run/pause, `c` start episode / stop+save, `x` DISCARD the open episode (it no longer stops the C++ loop —
use `O` in the deploy pane for that), `t <text>` prompt of exporter *and* client, `1`-`9` cell prompts.

What lands in the dataset: identical schema to the teleop recordings. `action.motion_token` = the token the
deploy executed (the policy's), `teleop.*_hand_joints` = the policy's hand action, `observation.state` hands
from the Inspire bridge's dump port as in teleop, both camera views. Every frame has `teleop.stream_mode` 6
(POLICY): the exporter sees VLA tokens on the `pose` topic and no streamer `manager_state`, and says so once
(`recording frames as stream_mode 6 (POLICY)`). SMPL columns are zero by construction; `process_dataset.py`
keeps such frames (`--keep-zero-smpl-modes 5 6`), so the usual cleaning + merge works on these datasets.
Proof an episode took: `Started recording N: "<prompt>" [POLICY episode]` in the exporter pane, and the
`Episode N stream modes: 1234 frames: POLICY 1234 (100%)` line after `c` again.

**Verified on the robot 2026-09-23**: 3 episodes (`outputs/policy_2026-09-23`, 7,899 frames), all POLICY-labelled,
tokens and hand actions populated, videos matching the parquets, all files in S3 — see `docs/EXPERIMENTS.md`.
Unit tests: `tools/tests/test_exporter_policy_mode.py`.

### DAgger: record policy attempts and intervene from the PICO (experimental, 2026-09-22, issue #25)

**Not yet verified on the robot.** Everything below is implemented and unit-tested against the C++
deploy's ZMQ protocol as read from its source; the first live run has to confirm the two mode
switches (see the checklist at the end). Run it on the hoist.

**What it does.** `DAGGER=1 tools/vla-inference.sh up` is `RECORD=1` (above) *plus* the PICO
streamer. Every closed-loop attempt is recorded as a LeRobot episode, and the
operator wearing the headset can take the arms and hands over at any moment (an *intervention*), then hand
back to the policy. Per frame the dataset says who was in control: `teleop.stream_mode` 6 = policy,
5 = human intervention (`PLANNER_VR_3PT`). `action.motion_token` is the token the deploy executed in
both cases (the policy's, or the SONIC encoder's output under the operator), so intervention frames are
training data in the same action space. `process_dataset.py --intervention-segments` cuts the corrections
out afterwards (DAgger = aggregate them into the training set and fine-tune again).

**Why it is wired this way.** The deploy takes one ZMQ input source (5556) and locks the `pose` protocol
version per streaming session: v3 SMPL poses (teleop) and v4 tokens (policy) cannot alternate, a change
makes it leave streaming mode "for safety". So:

| piece | role in DAGGER mode |
|---|---|
| PICO streamer (`--policy-port 5576 --keys-port 5580`) | owns 5556. New `StreamMode.POLICY` (6): relays the VLA client's v4 `pose` messages to the deploy unchanged; its Inspire bridge drives the hands from the relayed hand joints. Intervention = the existing `PLANNER_VR_3PT` mode (deploy in PLANNER mode, upper body from the VR wrists, recalibrated onto the robot's **measured** joints at the moment of the take-over so nothing jumps). Back to POLICY = deploy back to STREAMED mode, which also resets the protocol lock. |
| `run_vla_inference.py --relay --action-zmq-port 5576` | publishes tokens for the streamer instead of the deploy; `k`/`x` do nothing (the streamer starts/stops the deploy); follows `manager_state.stream_mode`: yields during an intervention, resumes by itself when POLICY returns *if it was running*, blending 0.5 s from the deploy's last token (`--resume-blend-s`). |
| exporter (container) | unchanged schema; in mode 6 `teleop.*_hand_joints` come from the relayed action; prints the stream-mode summary of every saved episode (`POLICY 1234 (81%), PLANNER_VR_3PT 290 (19%); 2 intervention segment(s)`). |
| keys pane (`record_keys_zmq.py --line-mode --passthrough-keys "pigh[]"`) | one pane for all three: `c`/`x` episode, `p`/`i`/`[`/`]` VLA client, `g`/`h` streamer, `t <text>` prompt of exporter **and** client, `1`-`9` cell prompts. **Every key needs Enter.** |
| `inspire_vla_bridge.py` | not started; the streamer's bridge does the hands in every mode. |

**Run**

```bash
sudo -v
DAGGER=1 DATASET=policy_2026-09-22 ~/GR00T-WholeBodyControl/tools/vla-inference.sh up   # + POLICY_MODEL, PROMPT, INSPIRE_HANDS as usual
tmux attach -t vla
```
Run window: deploy (top-left) | inference (top-right) | streamer (mid-left) | exporter (bottom-right) |
keys (bottom-left). `svc` window has xr-service in addition to serve / cam-g1 / cam-sender.

Headset: PC service 10.42.0.1, **Full body + Send** (the intervention needs body tracking for the 3-point
pose), ankle trackers not needed, Remote Vision open **before** `g` and kept open (closing it kills the
camera tee). Operator in the calibration pose.

| step | who | do | expect |
|---|---|---|---|
| 1 | | wait for serve (model loaded), deploy `Init Done`, inference `PolicyServer is reachable` + both camera views, streamer past `waiting for body data`, exporter `Recording to outputs/<dataset>` | inference pane: `Streamer in OFF: waiting for POLICY mode` |
| 2 | PICO | **A+B+X+Y** (brief) | robot stands under the planner (`PLANNER`, 3-point calibration captured). |
| 3 | keys | `g` ⏎ | streamer `StreamMode switch: PLANNER -> POLICY`, deploy `Switched to: STREAMED MOTION mode`, inference `Streamer in POLICY mode: our tokens reach the deploy now`. Robot holds still (encoder on the frozen pose until tokens arrive). |
| 4 | keys | `i` ⏎ then `p` ⏎ | initial pose blend, then the policy drives; deploy prints `Protocol version 4 established`. |
| 5 | keys | `c` ⏎ | exporter `Started recording N: "<prompt>" [POLICY episode]`. |
| 6 | PICO | **left-stick click** (or B+X, or `h` ⏎) when the policy is failing or about to | streamer `INTERVENTION: operator has the upper body`, inference `INTERVENTION: ... policy yielded`. Your wrists now steer the arms from where the robot's arms are (no jump); triggers close the hands. Sticks = planner locomotion, avoid them. |
| 7 | PICO | same gesture again | streamer `intervention over: back to POLICY mode`, inference `POLICY mode is back - resuming with a fresh chunk, blending ...`. |
| 8 | keys | `c` ⏎ (save) or `x` ⏎ (discard) | exporter summary line with the mode percentages and the number of intervention segments. Repeat from 4/5 (use `i` between attempts). |

`p` while yielded does not move anything: it flips whether the policy resumes when POLICY mode returns
(the pane says which). `A+X` = back to PLANNER (the client yields); `A+B+X+Y` = stop everything as in teleop;
`O` in the deploy pane = e-stop. `down` sends Ctrl-C to every pane first (streamer opens the hands, exporter
saves and finishes uploads).

**Post-processing.** Clean as usual (policy / VR_3PT frames have zero `teleop.smpl_pose` by design and are
no longer removed as stale, `--keep-zero-smpl-modes 5 6`), then cut the corrections:
```bash
~/wbc-marin-exec.sh python gear_sonic/scripts/process_dataset.py \
  --dataset-path outputs/policy_2026-09-22 --output-path outputs/policy_2026-09-22_corrections \
  --intervention-segments --intervention-preroll-s 1.0      # one episode per correction + 1 s of policy lead-in
# --keep-episodes-without-interventions keeps clean autonomous successes whole
```
Then merge with the demonstrations (`--dataset-path <demos> <corrections> --output-path <merged>`) and
fine-tune. The processor prints per-episode mode counts and a `DAgger:` summary line.

**Gotchas / to verify live**
- Two mode switches per intervention (STREAMED→PLANNER→STREAMED). Each is a C++ safety reset: the robot
  holds its pose for the switch. On the way back the deploy must accept v4 again after the reset
  (`Protocol version 4 established` a second time). If instead it prints `Protocol version changed ...
  Exiting ZMQ streaming mode`, the lock did not reset — stop (`A+B+X+Y`) and report; nothing else in this
  section is safe until that is fixed.
- The first VR_3PT frames follow the *recalibrated* wrists. The recalibration reads `body_q` from
  `g1_debug` (it used to look only for `body_q_measured` and silently fell back to a zero pose, i.e. a jump —
  fixed in this branch). Watch the streamer pane for `VR 3PT recalibration scheduled with measured robot pose`.
- Interventions record the SONIC encoder's tokens in VR-3-point mode; the demonstrations were POSE mode
  (SMPL encoder mode). Same decoder and latent space, possibly a different token distribution — check the
  first fine-tune on corrections open-loop before trusting it.
- Hands: in POLICY mode they follow the relayed action (open after 2 s without one, like the VLA bridge); in
  VR_3PT the triggers. During the switch the hands keep their last command.
- The VLA client's sensor-loss guard still applies; `Streamer ... lost` after 2 s without `manager_state`
  pauses it (streamer died?). The exporter needs `robot_config` from the deploy before it records, as always.
- `g` before `A+B+X+Y` is ignored (`robot not started`); `h` only from POLICY / VR_3PT.

### Status (2026-09-23)

**Closed-loop attempts are recorded as LeRobot episodes** (`RECORD=1`, verified 2026-09-23, 3 episodes, #25).
Interventions from the PICO (`DAGGER=1`) are implemented and unit-tested, not yet run on the robot.

### Status (2026-09-16)

**Closed loop verified on the real robot (2026-09-11, 8 attempts, both checkpoints).** Approach and grasp of the
red-capped bottle work almost every time, including with a Fanta distractor; placement does not yet — the robot hovers
with the bottle raised and the wrist motors overheat and fault first. Details, videos and next steps:
[`docs/EXPERIMENTS.md`](https://github.com/prosus-robotics/g1-vr-teleop/blob/main/docs/EXPERIMENTS.md) in g1-vr-teleop. Datasets,
checkpoints and videos are archived in `s3://darwin-robot-data`
([`docs/DATA_AND_MODELS.md`](https://github.com/prosus-robotics/g1-vr-teleop/blob/main/docs/DATA_AND_MODELS.md)). Open follow-ups:
[milestone 1](https://github.com/prosus-robotics/g1-vr-teleop/milestone/1) (#18–#21, #23, #25–#30 there).

Verified on hardware: both camera views at 30 Hz without a headset (skew ≤ 50 ms), hands close/open through the
bridge per side, hand state published, policy round trip 0.25 s on the bench / 0.4–0.5 s in the runs, deploy →
`Init Done` → `k` → `Planner enabled` → `i` → `p` → autonomous motion. Known hardware quirks (also in the training
data): right index finger reads closed at its open end-stop (0.25 closure at rest); index fingers stall at ~1/3 travel
when closing.

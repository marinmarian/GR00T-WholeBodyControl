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

### Recording (PICO controller or KEYS pane)
The script runs the **ZMQ exporter** (`gear_sonic/scripts/run_data_exporter.py`)
by default. `EXPORTER=ros2` selects the older ROS 2 exporter, which does not work
on this stack — see the ROS 2 entry in Troubleshooting.

- **left grip + A** = start episode → again = stop & save
- **left grip + B** = discard while recording
- Or type `c` / `x` in the KEYS pane (`tools/record_keys_zmq.py`, publishes on
  ZMQ 5580). No ratings on this path: `g`/`v`/`b` do nothing.
- `Started recording 0` in the **exporter** pane is the only proof it took. The
  KEYS pane echoing a key means nothing on its own.
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

### Camera sender — OBSBOT color + composite (mjolnir)
```bash
cd ~/XRoboToolkit-Orin-Video-Sender && ./OrinVideoSenderIR --listen 0.0.0.0:13579 \
  --device /dev/v4l/by-id/usb-Remo_Tech_Co.__Ltd._OBSBOT_Tiny_2_Lite-video-index0 \
  --pixfmt MJPG --width 1280 --height 720 --fps 30 \
  --second-device udp:5600 --second-width 640 --second-height 480 --zmq-pub 5555
```
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
# separate terminal — optional, only so c/x can be typed instead of using the
# controller (left grip + A / left grip + B work without it):
~/wbc-marin-exec.sh python /workspace/wbc/tools/record_keys_zmq.py
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

## VLA inference (GR00T N1.7 restocking policy) — added 2026-09-11

One command, mirrors the teleop runner:
```bash
~/GR00T-WholeBodyControl/tools/vla-inference.sh up      # PROMPT=... HANDS=0 POLICY_MODEL=... overrides
tmux attach -t vla                                        # down: tools/vla-inference.sh down
```
| piece | where | notes |
|---|---|---|
| PolicyServer (Isaac-GR00T) | **darwin-gpu** (EC2, RTX PRO 6000) | `ssh darwin-gpu '~/serve_policy.sh v2\|best34'` — tmux `policy`. Checkpoints: `~/checkpoints/restocking_{v2,best34}/…/checkpoint-10000`. Always passes `~/configs/g1_sonic_two_cam.py`. |
| SSH tunnel | mjolnir `svc` pane | `ssh -N -L 5550:127.0.0.1:5550 ubuntu@13.40.142.104` (auto-retry). EC2 SG does not open 5550; mjolnir's ed25519 key is authorised on darwin-gpu. RTT ~10 ms, one inference ≈ 0.25 s (budget 0.4 s at 2.5 Hz). |
| camera sender | mjolnir `svc` pane | same OBSBOT + head-IR composite as teleop **plus `--autostart`**: capture + ZMQ 5555 tee run without a PICO Remote Vision session. `ego_view` = OBSBOT side view, `head_view` = D430i IR (needs the g1 gst push, auto-started when g1 answers ping). |
| C++ deploy | mjolnir `run` pane, `g1-deploy-dev` container | identical command to teleop (`--input-type zmq_manager --output-type all`); `run_vla_inference.py` replaces the PICO streamer on 5556/5557. **The PICO streamer must be down** (same port). |
| `run_vla_inference.py` | mjolnir `.venv_inference` | patched: video keys queried from the server (`[head_view, ego_view]`), Inspire hand STATE from the dump port 5558 (exporter's mapping), vendored `PolicyClient` (Isaac-GR00T needs py3.12), `--initial-motion-token-path` = standing token averaged from our episodes, default prompt = training prompt. |
| `inspire_vla_bridge.py` | mjolnir `run` pane | NEW. SUB 5556 `pose` → 7-DoF hand actions → closure ratio (≥0.5 = closed) → Inspire Modbus; PUB measured state on 5558. Reuses `InspireBridge`; Ctrl-C/crash → hands open. Without it hands never move (deploy drives Dex3 only) and hand state is zeros. |
| keys | mjolnir `run` pane | `tools/vla_keys.py` → ZMQ 5580. `k` START C++ loop (always), `x` STOP it, `i` initial pose, `p` run/pause policy, `t <text>` prompt. |

Operator sequence: deploy `Init Done` (hoist!) → inference pane shows `PolicyServer is reachable` + `Policy video keys` + image latency lines for BOTH views → keys: `k`, `i`, then `p` (`x` stops the C++ loop). E-stop `O` in the deploy pane. Prompt must stay `put bottles with red cap in red bottle holder`.

Verified 2026-09-11 without the robot: synthetic observation → server → 40×(64+7+7) actions in 0.25 s; sender `--autostart` at 30 Hz; hand mapping unit test; bridge dry-run; inference script startup. NOT yet verified: g1 head push + `head_view` in the live message, Inspire Modbus writes from the bridge, deploy ↔ inference handshake, closed loop.

**VLA gotchas (read before the first run):**
- **`p` (pause) opens the hands after 2 s.** Pausing stops the action stream; `inspire_vla_bridge.py` treats >2 s without a fresh action as comms loss and commands both hands open (`--action-max-age`). A held bottle is dropped. Safety default, deliberate; to hold the last grasp instead run the bridge with `--action-max-age 1e9`.
- **Frozen head view is rejected, not used.** If the g1 RTP push dies the sender keeps re-sending the last `head_view` JPEG; `run_vla_inference.py` drops observations whose views differ by >0.5 s (`MAX_VIEW_SKEW_S`) and logs `stale camera view(s)`. The policy then keeps replaying its last chunk until it runs out — pause (`p`) and fix the camera.
- **Hands open on Ctrl-C / SIGTERM / Python crash of the bridge**, not on SIGKILL or power loss.
- **Do not open-then-close PICO Remote Vision during a run**: `CLOSE_CAMERA` tears down the capture pipeline and the ZMQ tee with it. Keep the headset out of it (autostart) or leave the session open.
- `install_scripts/install_inference.sh` is broken under uv (upstream too: dependency named `Isaac-GR00T`, package is `gr00t`). `.venv_inference` was built by hand; see memory/this section.
- **`LowState or IMUState is not available` right after `k`, then `Planner initialization timeout`** (seen 2026-09-11 on the first VLA run): the IGMP querier was down. The launcher can only start it if sudo is cached — run `sudo -v` BEFORE `tools/vla-inference.sh up`, or start it by hand: `sudo nohup python3 ~/igmp_querier.py >/tmp/igmp.log 2>&1 &`. Lowstate itself was fine (`~/eval_vla/lowstate_check.py enP2p1s0` → ~1 kHz); the switch prunes the multicast ~4 min after the deploy subscribes, so `Init Done` → `k` must also not wait longer than that without the querier. Restart the deploy from the container prompt with `./run_sonic.sh`.

#!/bin/bash
# sonic-teleop.sh — one-command SONIC whole-body teleop stack (PICO, both arms).
#
#   ~/sonic-teleop.sh up       start everything (then: tmux attach -t sonic)
#   ~/sonic-teleop.sh down     stop everything (DAMP THE ROBOT FIRST: O / L2+B)
#   ~/sonic-teleop.sh status   show what's running
#   ~/sonic-teleop.sh net hp2|hotspot   switch the PICO/laptop network (sudo nmcli; see RUNBOOK "Network")
#
# tmux session "sonic", two windows:
#   svc : [xr-service] [cam-g1: head-IR RTP push] [cam-thor: OBSBOT + composite sender]
#   run : [streamer] [DEPLOY  <- 'O' = e-stop lives HERE]
#
# Policy: sonic_v1_1. Cameras: OBSBOT Tiny 2 Lite color (thor USB, left half;
# MJPEG 1280x720@30 -> software jpegdec -> HW H.264) + D430i IR (g1, right half
# via RTP/UDP:5600). Remote Vision IP for the PICO = mjolnir's address on the PICO's wifi (printed by `up`):
# 192.168.123.250 on the HP2 router (mode A, default) or 10.42.0.1 on the mjolnir-xr hotspot (mode B, standalone).
# OBSBOT: keep its AI tracking OFF (gimbal must not hunt during teleop); reset
# pan/tilt/zoom with:  v4l2-ctl -d $CAM_COLOR -c pan_absolute=0,tilt_absolute=0,zoom_absolute=0
#
# Manual steps that stay manual:
#   * PICO app: on wifi HP2 or mjolnir-xr, PC service = the address `up` prints (WORKING); Head+Controller+Full body+Send;
#     ankle trackers calibrated. Streamer must STOP saying "waiting for body data".
#   * Robot: on + damped (L2+B), hoisted, floor clear (POSE mode moves legs).
#   * Activate: calibration pose -> one brief A+B+X+Y -> ~10 s -> stand -> A+X = POSE.
#   * xr-service may be another user's (root) — this script never kills it.

S=sonic
CAM_COLOR=/dev/v4l/by-id/usb-Remo_Tech_Co.__Ltd._OBSBOT_Tiny_2_Lite-video-index0
CAM_G1=/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_430i_Intel_R__RealSense_TM__Depth_Camera_430i_349623061587-video-index2
# INSPIRE_HANDS=trigger (default: controller trigger/grip) | handtracking (PICO hand tracking, #35, experimental:
# fingers -> trigger, thumb -> grip; buttons/sticks/e-stop stay on the controllers, keep them within reach) | off
INSPIRE_HANDS="${INSPIRE_HANDS:-trigger}"

# Network mode of the PICO/laptop side (RUNBOOK "Network"). The robot cable on mgbe0_0 (.222) is the same in both.
#   hp2     : PICO + laptop + mjolnir on the GL.iNet router "HP2" (192.168.123.1); mjolnir over its own wifi (profile
#             HP2, 192.168.123.250) or, cabled on purpose, mgbe1_0 ("Wired connection 2", DHCP). Shares the
#             192.168.123.0/24 prefix with the robot cable, so the `robot` profile only carries on-link routes for
#             .160/29 (robot) + .208/29 (Inspire hands) at metric 1000 and the rest of the /24 goes out the HP2 side.
#   hotspot : everyone on mjolnir's own AP "mjolnir-xr" (profile quest-hotspot, 10.42.0.1, no router, no internet).
# Sets NET_MODE (hp2|hotspot|none), NET_IP (what the PICO enters), NET_SSID.
net_detect() {
  NET_MODE=none; NET_IP=""; NET_SSID=""
  if nmcli -t -f NAME,DEVICE con show --active 2>/dev/null | grep -q '^quest-hotspot:' \
     && ip -4 addr show wlP1p1s0 2>/dev/null | grep -q 'inet 10\.42\.0\.1/'; then
    NET_MODE=hotspot; NET_IP=10.42.0.1; NET_SSID=mjolnir-xr
  else
    HP2_IP=$(ip -4 route get 192.168.123.1 2>/dev/null | sed -n 's/.* src \([0-9.]*\).*/\1/p')
    if [ -n "$HP2_IP" ] && [ "$HP2_IP" != 192.168.123.222 ]; then NET_MODE=hp2; NET_IP=$HP2_IP; NET_SSID=HP2; fi
  fi
}
NET_HELP='sudo nmcli con up HP2  (router, mode A)   |   sudo nmcli con up quest-hotspot  (standalone mjolnir-xr, mode B)'
# Streamer output is also written to logs/streamer-last-run.log (unbuffered) so a run can be diagnosed after `down`.
STREAMER_CMD="cd ~/GR00T-WholeBodyControl && source .venv_teleop/bin/activate && mkdir -p logs && ${HAND_TRACKING_DEBUG:+HAND_TRACKING_DEBUG=1 }python -u gear_sonic/scripts/pico_manager_thread_server.py --manager --input-source xrt --inspire-hands $INSPIRE_HANDS 2>&1 | tee logs/streamer-last-run.log"
DEPLOY_ENTER='cd ~/GR00T-WholeBodyControl/gear_sonic_deploy && ./docker/run-ros2-dev.sh'
DEPLOY_RUN='./target/release/g1_deploy_onnx_ref mgbe0_0 policy/sonic_v1_1/model_decoder.onnx reference/example/ --obs-config policy/sonic_v1_1/observation_config.yaml --encoder-file policy/sonic_v1_1/model_encoder.onnx --planner-file planner/target_vel/V2/planner_sonic.onnx --input-type zmq_manager --output-type all --zmq-host localhost'
# Head IR is pushed from g1 to mjolnir over RTP (5600 = composite's right half,
# 5601 = the dataset head_view recorder); the OBSBOT + composite sender run here.
CAMG1_CMD='ssh g1 "gst-launch-1.0 v4l2src device='$CAM_G1' ! video/x-raw,format=GRAY8,width=640,height=480,framerate=15/1 ! videoconvert ! video/x-raw,format=I420 ! jpegenc quality=80 ! rtpjpegpay ! multiudpsink clients=192.168.123.222:5600,192.168.123.222:5601"'
# --autostart: capture + the ZMQ 5555 recording tee run from launch, not only during a
# headset Remote Vision session (2026-09-23: the tee sat silent until the headset connected,
# and a silently dead pipeline left the exporter without images). The headset reconnecting
# restarts the pipeline and the tee follows; verified live the same day.
CAMSEND_CMD='cd ~/XRoboToolkit-Orin-Video-Sender && ./OrinVideoSenderIR --listen 0.0.0.0:13579 --device '$CAM_COLOR' --pixfmt MJPG --width 1280 --height 720 --fps 30 --second-device udp:5600 --second-width 640 --second-height 480 --zmq-pub 5555 --autostart'
# Episode recording. EXPORTER=zmq (default) or EXPORTER=ros2.
#
# zmq  - gear_sonic/scripts/run_data_exporter.py. State + robot_config come from
#        the C++ deploy's ZMQ output (g1_debug on 5557), camera from 5555. This is
#        the path that WORKS on this stack. Episodes land in outputs/<dataset>/.
#        Keys: left grip + A = start/stop+save, left grip + B = discard (via the
#        streamer's manager_state topic), or c/x typed in the KEYS pane.
#        Records ego_view + head_view; each saved episode is uploaded to
#        s3://$DATASET_BUCKET/raw/<dataset>/ on a background thread (local files
#        are the source of truth; a failed upload only warns). Still no episode
#        ratings (g/v/b) on this path.
# ros2 - decoupled_wbc/.../run_g1_data_exporter.py. Has ratings (g/v/b), S3 upload
#        and head_view, and the right-stick-click / A+Y gestures over UDP 5559.
#        BROKEN as of 2026-09-07: the deploy publishes nothing over ROS 2 (no
#        g1_output_handler node, WBCPolicy/robot_config has Publisher count 0), so
#        the exporter never leaves its "[robot_config] waiting ..." loop and every
#        keypress is dropped. Not a DDS problem - discovery across the containers
#        works. Restart the deploy and read its first ~40 lines for the ROS2 output
#        handler's verdict before using this.
if [ "${EXPORTER:-zmq}" = "ros2" ]; then
  EXPORTER_CMD='~/wbc-marin-exec.sh python decoupled_wbc/control/main/teleop/run_g1_data_exporter.py --camera-host 127.0.0.1 --camera-port 5555 --dataset-name '"${DATASET:-g1_sonic}"' --task-prompt "'"${TASK:-whole_body_teleop}"'" --data-collection --no-add-stereo-camera --add-head-camera --no-text-to-speech'
  KEYS_CMD='~/wbc-marin-exec.sh python /workspace/wbc/tools/record_keys.py'
  RECORD_HELP='c=start/stop+save x=discard g/v/b=rate | PICO: right-stick click tap=c, hold 1.5s=x, A+Y=c'
else
  EXPORTER_CMD='~/wbc-marin-exec.sh python gear_sonic/scripts/run_data_exporter.py --camera-host 127.0.0.1 --camera-port 5555 --dataset-name '"${DATASET:-g1_sonic}"' --task-prompt "'"${TASK:-whole_body_teleop}"'" --add-head-camera --upload-bucket-path '"${DATASET_BUCKET:-darwin-robot-data}"' --no-text-to-speech'
  # PROMPT_TEMPLATE: prompt behind digits 1-9 in the KEYS pane (one per tic-tac-toe cell,
  # applies to the NEXT episode; g1-vr-teleop #40). 0 restores $TASK.
  KEYS_CMD='~/wbc-marin-exec.sh python /workspace/wbc/tools/record_keys_zmq.py --base-prompt "'"${TASK:-whole_body_teleop}"'"'"${PROMPT_TEMPLATE:+ --prompt-template \"$PROMPT_TEMPLATE\"}"
  RECORD_HELP='PICO: left grip + A = start/stop+save, left grip + B = discard | or c/x in the KEYS pane; 1-9 = cell prompt for the NEXT episode, 0 = $TASK'
fi
XRSVC_CMD='pgrep -f RoboticsServiceProcess >/dev/null && echo "xr-service already running" || DISPLAY=:0 ~/start_xrsvc.sh'

case "${1:-up}" in
# ─────────────────────────────────────────────────────────────────────────────
up)
  # 0. PICO/laptop network: mode A (HP2 router) or mode B (mjolnir-xr hotspot) — see net_detect above.
  net_detect
  case $NET_MODE in
    hotspot)
      N_STA=$(iw dev wlP1p1s0 station dump 2>/dev/null | grep -c '^Station')
      echo "[ok] hotspot mjolnir-xr up: mjolnir is $NET_IP, $N_STA device(s) joined  (PICO + laptop join mjolnir-xr; PC service + Remote Vision -> $NET_IP)" ;;
    hp2)
      if ping -c1 -W1 192.168.123.1 >/dev/null 2>&1; then
        echo "[ok] HP2 LAN up: mjolnir is $NET_IP  (PICO + laptop on wifi HP2; PC service + Remote Vision -> $NET_IP)"
      else
        echo "[!!] HP2 router 192.168.123.1 not answering from $NET_IP — check HP2 wifi (sudo nmcli con up HP2) or go standalone (sudo nmcli con up quest-hotspot)"
      fi ;;
    *)
      echo "[!!] no PICO network (HP2 not routed, hotspot down; replies to the PICO would leave on the robot cable):  $NET_HELP" ;;
  esac
  if [ "$INSPIRE_HANDS" != off ] && ! ip -4 route get 192.168.123.210 2>/dev/null | grep -q 'dev mgbe0_0'; then
    echo "[!!] Inspire hands .210/.211 are NOT routed to the robot cable:  sudo nmcli con mod robot +ipv4.routes 192.168.123.208/29 && sudo nmcli con up robot"
  fi

  # 1. igmp querier — systemd unit from boot (rig/igmp-querier.service, #18); without it DDS dies ~4 min after the deploy subscribes
  if systemctl is-active --quiet igmp-querier; then
    echo "[ok] igmp querier running (systemd)"
  else
    echo "[!!] IGMP QUERIER DOWN:  sudo systemctl restart igmp-querier   (not installed? sudo rig/install_igmp_querier.sh)"
  fi

  # 2. robot reachable?
  ping -c1 -W1 192.168.123.161 >/dev/null 2>&1 && echo "[ok] robot lowlevel up" || echo "[!!] robot lowlevel NOT reachable — power it on"

  # 3. clean slate (never touch xr-service: may be another user's)
  tmux kill-session -t $S 2>/dev/null
  pkill -f "pico_manage[r]_thread" 2>/dev/null
  pkill -f "OrinVideoSenderI[R]" 2>/dev/null
  ssh -o BatchMode=yes -o ConnectTimeout=4 g1 'pkill -f "OrinVideoSenderI[R]"' 2>/dev/null
  pkill -f "g1_deploy_onnx_re[f]" 2>/dev/null
  ssh -o BatchMode=yes -o ConnectTimeout=4 g1 'pkill -f "gst-launc[h]"' 2>/dev/null
  docker exec wbc-marin bash -c 'pkill -f "run_.*data_exporte[r]"; pkill -f "record_key[s]"' 2>/dev/null
  sleep 1

  # 4. svc window: xr-service | cam-g1 | cam-thor
  tmux new-session -d -s $S -n svc
  P_XR=$(tmux display -t $S:svc -p '#{pane_id}')
  P_CG1=$(tmux split-window -t "$P_XR" -h -P -F '#{pane_id}')
  P_CTH=$(tmux split-window -t "$P_XR" -v -P -F '#{pane_id}')
  tmux send-keys -t "$P_XR"  "$XRSVC_CMD" C-m
  tmux send-keys -t "$P_CG1" "until ping -c1 -W1 192.168.123.164 >/dev/null 2>&1; do echo waiting for g1...; sleep 3; done; $CAMG1_CMD" C-m
  tmux send-keys -t "$P_CTH" "$CAMSEND_CMD" C-m
  tmux select-layout -t $S:svc tiled

  # 5. run window: streamer | deploy
  tmux new-window -t $S -n run
  P_STR=$(tmux display -t $S:run -p '#{pane_id}')
  P_DEP=$(tmux split-window -t "$P_STR" -h -P -F '#{pane_id}')
  tmux send-keys -t "$P_STR" "until pgrep -f RoboticsServiceProcess >/dev/null; do echo waiting for xr-service...; sleep 1; done; sleep 2; $STREAMER_CMD" C-m
  tmux send-keys -t "$P_DEP" "$DEPLOY_ENTER" C-m
  ( sleep 12; tmux send-keys -t "$P_DEP" "$DEPLOY_RUN" C-m ) &
  P_EXP=$(tmux split-window -t "$P_STR" -v -P -F '#{pane_id}')
  # The exporter pane is the only proof of what got recorded ("Started recording N", "Waiting for
  # message", tracebacks), so keep a copy: `script` records the pane to logs/exporter-last-run.log
  # while leaving the exporter its tty (docker exec -it, Ctrl-C).
  EXP_Q=${EXPORTER_CMD//\'/\'\\\'\'}   # single-quote-safe copy for `script -c '...'`
  tmux send-keys -t "$P_EXP" "docker start wbc-marin >/dev/null 2>&1; sleep 3; mkdir -p ~/GR00T-WholeBodyControl/logs; script -qfc '$EXP_Q' ~/GR00T-WholeBodyControl/logs/exporter-last-run.log" C-m
  P_KEY=$(tmux split-window -t "$P_DEP" -v -P -F '#{pane_id}')
  tmux send-keys -t "$P_KEY" "docker start wbc-marin >/dev/null 2>&1; sleep 5; $KEYS_CMD" C-m
  tmux select-window -t $S:run
  tmux select-pane -t "$P_DEP"

  echo
  echo "Session '$S' up.  Attach:   tmux attach -t $S"
  echo "  run window: LEFT = streamer, RIGHT = deploy ('O' = e-stop). Wait for 'Init Done'."
  echo "  svc window (Ctrl-b n): xr-service / cam-g1 / cam-thor"
  echo "  PICO (wifi ${NET_SSID:-HP2 or mjolnir-xr}): PC service ${NET_IP:-<mjolnir ip, network is down>} -> WORKING; Full body + Send + ankle trackers;"
  echo "        Remote Vision -> ${NET_IP:-<same>} (color left, head-IR right)"
  echo "  DO NOT press A+B+X+Y while the streamer says 'waiting for body data'."
  echo "  Drive: calibration pose -> A+B+X+Y (stand) -> A+X (whole-body POSE)."
  echo "  Record (EXPORTER=${EXPORTER:-zmq}): $RECORD_HELP"
  echo "          Watch the exporter pane for 'Started recording 0' - that is the only proof it took."
  echo "          Episodes: ~/GR00T-WholeBodyControl/outputs/${DATASET:-g1_sonic}/"
  ;;
# ─────────────────────────────────────────────────────────────────────────────
down)
  echo "!! Make sure the robot is stopped (A+B+X+Y or O) and damped (L2+B) first."
  # graceful deploy stop (sends damping), then kill the rest
  for p in $(tmux list-panes -s -t $S -F '#{pane_id}' 2>/dev/null); do tmux send-keys -t $p C-c 2>/dev/null; done
  sleep 3
  tmux kill-session -t $S 2>/dev/null
  pkill -f "pico_manage[r]_thread" 2>/dev/null
  pkill -f "OrinVideoSenderI[R]" 2>/dev/null
  ssh -o BatchMode=yes -o ConnectTimeout=4 g1 'pkill -f "OrinVideoSenderI[R]"' 2>/dev/null
  pkill -f "g1_deploy_onnx_re[f]" 2>/dev/null
  docker exec wbc-marin bash -c 'pkill -f "run_.*data_exporte[r]"; pkill -f "record_key[s]"' 2>/dev/null
  ssh -o BatchMode=yes -o ConnectTimeout=4 g1 'pkill -f "gst-launc[h]"' 2>/dev/null
  echo "[ok] stopped session, streamer, deploy, cameras, exporter."
  echo "     left running: xr-service (may be shared), igmp querier (systemd), the network (HP2 / hotspot)."
  ;;
# ─────────────────────────────────────────────────────────────────────────────
status)
  echo -n "tmux session:   "; tmux has-session -t $S 2>/dev/null && echo UP || echo down
  echo -n "network:        "; net_detect; case $NET_MODE in
    hotspot) echo "hotspot mjolnir-xr (mjolnir = $NET_IP, $(iw dev wlP1p1s0 station dump 2>/dev/null | grep -c '^Station') device(s) joined)" ;;
    hp2) ping -c1 -W1 192.168.123.1 >/dev/null 2>&1 && echo "HP2 router (mjolnir = $NET_IP)" || echo "HP2 routed via $NET_IP but router 192.168.123.1 not answering" ;;
    *) echo "DOWN ($NET_HELP)" ;; esac
  echo -n "igmp querier:   "; systemctl is-active --quiet igmp-querier && echo UP || echo "DOWN (sudo systemctl restart igmp-querier)"
  echo -n "hotspot fallbk: "; case $(systemctl is-enabled hotspot-fallback 2>/dev/null) in enabled) echo "enabled at boot ($(systemctl show -p Result --value hotspot-fallback 2>/dev/null) this boot: $(journalctl -u hotspot-fallback -b -o cat 2>/dev/null | grep '^hotspot-fallback:' | tail -1 | cut -c19-110))" ;; *) echo "not installed (sudo rig/install_hotspot_fallback.sh) — no hotspot without HP2 unless 'net hotspot'" ;; esac
  echo -n "xr-service:     "; pgrep -f RoboticsServiceProcess >/dev/null && echo UP || echo down
  echo -n "streamer:       "; pgrep -f "pico_manage[r]_thread" >/dev/null && echo UP || echo down
  echo -n "deploy:         "; pgrep -f "g1_deploy_onnx_re[f]" >/dev/null && echo UP || echo down
  echo -n "cam thor OBSBOT:"; [ -e "$CAM_COLOR" ] && pgrep -f "OrinVideoSenderI[R]" >/dev/null && echo " UP" || { [ -e "$CAM_COLOR" ] && echo " down (camera present, sender not running)" || echo " DOWN (OBSBOT not on USB)"; }
  echo -n "cam g1 push:    "; ssh -o BatchMode=yes -o ConnectTimeout=4 g1 'pgrep -f "gst-launc[h]" >/dev/null' 2>/dev/null && echo UP || echo down
  echo -n "data exporter:  "; docker exec wbc-marin pgrep -f "run_.*data_exporter" >/dev/null 2>&1 && echo UP || echo down
  echo -n "record keys:    "; docker exec wbc-marin pgrep -f "record_key" >/dev/null 2>&1 && echo UP || echo down
  echo -n "robot lowlevel: "; ping -c1 -W1 192.168.123.161 >/dev/null 2>&1 && echo UP || echo DOWN
  echo -n "LAN neighbours: "; ip neigh show 2>/dev/null | grep -v 'dev mgbe0_0' | awk '/REACHABLE|STALE|DELAY/ && $1!="192.168.123.1"{print $1}' | sort -u | tr '\n' ' ' | grep . || echo "none seen (PICO/laptop not on ${NET_SSID:-the network} yet?)"
  ;;
# ─────────────────────────────────────────────────────────────────────────────
net)
  # Switch the PICO/laptop network (one wifi radio: activating one profile drops the other). Any ssh session riding on
  # mjolnir's wifi (HP2 or Tailscale) drops with it — run this at the keyboard or expect to reconnect.
  case "${2:-}" in
    hp2)     PROF=HP2;           WHAT="mode A: PICO + laptop on the HP2 router" ;;
    hotspot) PROF=quest-hotspot; WHAT="mode B: PICO + laptop join mjolnir-xr (standalone, no internet)" ;;
    *) echo "usage: $0 net {hp2|hotspot}"; net_detect; echo "current: $NET_MODE ${NET_IP:+(mjolnir = $NET_IP)}"; exit 1 ;;
  esac
  net_detect
  if [ "$NET_MODE" = "${2}" ]; then echo "[ok] already $WHAT (mjolnir = $NET_IP)"; exit 0; fi
  echo "switching -> $WHAT   (sudo nmcli con up $PROF; ssh over wifi/Tailscale will drop)"
  if sudo nmcli con up "$PROF"; then
    sleep 2; net_detect
    [ "$NET_MODE" = "${2}" ] && echo "[ok] $WHAT: mjolnir = $NET_IP  (PICO: PC service + Remote Vision -> $NET_IP)" \
                              || echo "[!!] profile $PROF is up but the mode was not detected — check: ip -4 addr show wlP1p1s0; nmcli con show --active"
  else
    echo "[!!] nmcli con up $PROF failed. Hotspot wedged? sudo systemctl restart wpa_supplicant NetworkManager && sleep 5 && sudo nmcli con up $PROF  (RUNBOOK 'Hotspot gone')"
    exit 1
  fi
  ;;
*)
  echo "usage: $0 {up|down|status|net hp2|net hotspot}"; exit 1 ;;
esac

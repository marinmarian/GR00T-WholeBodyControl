#!/bin/bash
# sonic-teleop.sh — one-command SONIC whole-body teleop stack (PICO, both arms).
#
#   ~/sonic-teleop.sh up       start everything (then: tmux attach -t sonic)
#   ~/sonic-teleop.sh down     stop everything (DAMP THE ROBOT FIRST: O / L2+B)
#   ~/sonic-teleop.sh status   show what's running
#
# tmux session "sonic", two windows:
#   svc : [xr-service] [cam-g1: head-IR RTP push] [cam-thor: composite sender]
#   run : [streamer] [DEPLOY  <- 'O' = e-stop lives HERE]
#
# Policy: sonic_v1_1. Cameras: D455f color (thor, left half) + D430i IR (g1,
# right half via RTP/UDP:5600). Remote Vision IP for the PICO = 10.42.0.1.
#
# Manual steps that stay manual:
#   * PICO app: PC service 10.42.0.1 (WORKING); Head+Controller+Full body+Send;
#     ankle trackers calibrated. Streamer must STOP saying "waiting for body data".
#   * Robot: on + damped (L2+B), hoisted, floor clear (POSE mode moves legs).
#   * Activate: calibration pose -> one brief A+B+X+Y -> ~10 s -> stand -> A+X = POSE.
#   * xr-service may be another user's (root) — this script never kills it.

S=sonic
CAM_THOR=/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_455f_Intel_R__RealSense_TM__Depth_Camera_455f_254643069357-video-index0
CAM_G1=/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_430i_Intel_R__RealSense_TM__Depth_Camera_430i_349623061587-video-index2
STREAMER_CMD='cd ~/GR00T-WholeBodyControl && source .venv_teleop/bin/activate && python gear_sonic/scripts/pico_manager_thread_server.py --manager --input-source xrt --inspire-hands trigger'
DEPLOY_ENTER='cd ~/GR00T-WholeBodyControl/gear_sonic_deploy && ./docker/run-ros2-dev.sh'
DEPLOY_RUN='./target/release/g1_deploy_onnx_ref enP2p1s0 policy/sonic_v1_1/model_decoder.onnx reference/example/ --obs-config policy/sonic_v1_1/observation_config.yaml --encoder-file policy/sonic_v1_1/model_encoder.onnx --planner-file planner/target_vel/V2/planner_sonic.onnx --input-type zmq_manager --output-type all --zmq-host localhost'
CAMG1_CMD='ssh g1 "gst-launch-1.0 v4l2src device='$CAM_G1' ! video/x-raw,format=GRAY8,width=640,height=480,framerate=15/1 ! videoconvert ! video/x-raw,format=I420 ! jpegenc quality=80 ! rtpjpegpay ! udpsink host=192.168.123.222 port=5600"'
CAMTHOR_CMD='cd ~/XRoboToolkit-Orin-Video-Sender && ./OrinVideoSenderIR --listen 0.0.0.0:13579 --device '$CAM_THOR' --pixfmt YUY2 --width 1280 --height 720 --fps 15 --second-device udp:5600 --second-width 640 --second-height 480 --zmq-pub 5555'
# Episode recording (LeRobot + S3 when creds present). Type c/x/g/v/b in the KEYS pane.
EXPORTER_CMD='~/wbc-exec.sh python decoupled_wbc/control/main/teleop/run_g1_data_exporter.py --camera-host 127.0.0.1 --camera-port 5555 --dataset-name '"${DATASET:-g1_sonic}"' --task-prompt "'"${TASK:-whole body teleop}"'" --data-collection --no-add-stereo-camera --no-text-to-speech'
KEYS_CMD='~/wbc-exec.sh python /workspace/wbc/tools/record_keys.py'
XRSVC_CMD='pgrep -f RoboticsServiceProcess >/dev/null && echo "xr-service already running" || DISPLAY=:0 ~/start_xrsvc.sh'

case "${1:-up}" in
# ─────────────────────────────────────────────────────────────────────────────
up)
  # 0. hotspot (sudo)
  if ip -brief addr show wlP1p1s0 2>/dev/null | grep -q UP; then
    echo "[ok] hotspot up"
  elif sudo -n true 2>/dev/null; then
    sudo nmcli con up quest-hotspot >/dev/null 2>&1 && echo "[ok] hotspot started" || echo "[!!] hotspot failed to start"
  else
    echo "[!!] HOTSPOT DOWN (needs sudo):  sudo nmcli con up quest-hotspot"
  fi

  # 1. igmp querier (sudo) — without it DDS dies ~4 min after the deploy subscribes
  if pgrep -f igmp_querier.py >/dev/null; then
    echo "[ok] igmp querier running"
  elif sudo -n true 2>/dev/null; then
    sudo nohup python3 ~/igmp_querier.py >/tmp/igmp.log 2>&1 &
    echo "[ok] igmp querier started"
  else
    echo "[!!] IGMP QUERIER DOWN (needs sudo):  sudo nohup python3 ~/igmp_querier.py >/tmp/igmp.log 2>&1 &"
  fi

  # 2. robot reachable?
  ping -c1 -W1 192.168.123.161 >/dev/null 2>&1 && echo "[ok] robot lowlevel up" || echo "[!!] robot lowlevel NOT reachable — power it on"

  # 3. clean slate (never touch xr-service: may be another user's)
  tmux kill-session -t $S 2>/dev/null
  pkill -f "pico_manage[r]_thread" 2>/dev/null
  pkill -f "OrinVideoSenderI[R]" 2>/dev/null
  pkill -f "g1_deploy_onnx_re[f]" 2>/dev/null
  ssh -o BatchMode=yes -o ConnectTimeout=4 g1 'pkill -f "gst-launc[h]"' 2>/dev/null
  sleep 1

  # 4. svc window: xr-service | cam-g1 | cam-thor
  tmux new-session -d -s $S -n svc
  P_XR=$(tmux display -t $S:svc -p '#{pane_id}')
  P_CG1=$(tmux split-window -t "$P_XR" -h -P -F '#{pane_id}')
  P_CTH=$(tmux split-window -t "$P_XR" -v -P -F '#{pane_id}')
  tmux send-keys -t "$P_XR"  "$XRSVC_CMD" C-m
  tmux send-keys -t "$P_CG1" "until ping -c1 -W1 192.168.123.164 >/dev/null 2>&1; do echo waiting for g1...; sleep 3; done; $CAMG1_CMD" C-m
  tmux send-keys -t "$P_CTH" "$CAMTHOR_CMD" C-m
  tmux select-layout -t $S:svc tiled

  # 5. run window: streamer | deploy
  tmux new-window -t $S -n run
  P_STR=$(tmux display -t $S:run -p '#{pane_id}')
  P_DEP=$(tmux split-window -t "$P_STR" -h -P -F '#{pane_id}')
  tmux send-keys -t "$P_STR" "until pgrep -f RoboticsServiceProcess >/dev/null; do echo waiting for xr-service...; sleep 1; done; sleep 2; $STREAMER_CMD" C-m
  tmux send-keys -t "$P_DEP" "$DEPLOY_ENTER" C-m
  ( sleep 12; tmux send-keys -t "$P_DEP" "$DEPLOY_RUN" C-m ) &
  P_EXP=$(tmux split-window -t "$P_STR" -v -P -F '#{pane_id}')
  tmux send-keys -t "$P_EXP" "docker start wbc-dev >/dev/null 2>&1; sleep 3; $EXPORTER_CMD" C-m
  P_KEY=$(tmux split-window -t "$P_DEP" -v -P -F '#{pane_id}')
  tmux send-keys -t "$P_KEY" "docker start wbc-dev >/dev/null 2>&1; sleep 5; $KEYS_CMD" C-m
  tmux select-window -t $S:run
  tmux select-pane -t "$P_DEP"

  echo
  echo "Session '$S' up.  Attach:   tmux attach -t $S"
  echo "  run window: LEFT = streamer, RIGHT = deploy ('O' = e-stop). Wait for 'Init Done'."
  echo "  svc window (Ctrl-b n): xr-service / cam-g1 / cam-thor"
  echo "  PICO: PC service 10.42.0.1 -> WORKING; Full body + Send + ankle trackers;"
  echo "        Remote Vision -> 10.42.0.1 (color left, head-IR right)"
  echo "  DO NOT press A+B+X+Y while the streamer says 'waiting for body data'."
  echo "  Drive: calibration pose -> A+B+X+Y (stand) -> A+X (whole-body POSE)."
  echo "  Record: type in the KEYS pane (bottom-right): c=start/stop+save x=discard g/v/b=rate."
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
  pkill -f "g1_deploy_onnx_re[f]" 2>/dev/null
  docker exec wbc-dev bash -c 'pkill -f "run_g1_data_exporte[r]"; pkill -f "record_key[s]"' 2>/dev/null
  ssh -o BatchMode=yes -o ConnectTimeout=4 g1 'pkill -f "gst-launc[h]"' 2>/dev/null
  echo "[ok] stopped session, streamer, deploy, cameras, exporter."
  echo "     left running: xr-service (may be shared), igmp querier, hotspot."
  ;;
# ─────────────────────────────────────────────────────────────────────────────
status)
  echo -n "tmux session:   "; tmux has-session -t $S 2>/dev/null && echo UP || echo down
  echo -n "hotspot:        "; ip -brief addr show wlP1p1s0 2>/dev/null | grep -q UP && echo UP || echo DOWN
  echo -n "igmp querier:   "; pgrep -f igmp_querier.py >/dev/null && echo UP || echo DOWN
  echo -n "xr-service:     "; pgrep -f RoboticsServiceProcess >/dev/null && echo UP || echo down
  echo -n "streamer:       "; pgrep -f "pico_manage[r]_thread" >/dev/null && echo UP || echo down
  echo -n "deploy:         "; pgrep -f "g1_deploy_onnx_re[f]" >/dev/null && echo UP || echo down
  echo -n "cam thor D455f: "; lsusb 2>/dev/null | grep -qi 455f && echo -n "enumerated, " || echo -n "NOT ON USB, "; pgrep -f "OrinVideoSenderI[R]" >/dev/null && echo "sender UP" || echo "sender down"
  echo -n "cam g1 push:    "; ssh -o BatchMode=yes -o ConnectTimeout=4 g1 'pgrep -f "gst-launc[h]" >/dev/null' 2>/dev/null && echo UP || echo down
  echo -n "data exporter:  "; docker exec wbc-dev pgrep -f run_g1_data_exporter >/dev/null 2>&1 && echo UP || echo down
  echo -n "robot lowlevel: "; ping -c1 -W1 192.168.123.161 >/dev/null 2>&1 && echo UP || echo DOWN
  echo -n "PICO on hotspot:"; ip neigh show dev wlP1p1s0 2>/dev/null | grep -q REACHABLE && echo " yes" || echo " not seen"
  ;;
*)
  echo "usage: $0 {up|down|status}"; exit 1 ;;
esac

#!/usr/bin/env bash
# One-command GR00T VLA inference on the G1 (restocking task) — mirrors tools/sonic-teleop.sh.
#   ~/GR00T-WholeBodyControl/tools/vla-inference.sh up|down|status
# Env overrides: PROMPT, POLICY_HOST/POLICY_PORT (default: SSH tunnel -> darwin-gpu:5550),
#                POLICY_MODEL (v2|best34, only used to print a reminder), HANDS=0 (skip Inspire bridge)
#
# Layout:  window "svc": tunnel | cam-g1 (head IR push) | cam-sender (OBSBOT + head, --autostart, ZMQ 5555)
#          window "run": deploy (C++ SONIC, container) | inference (run_vla_inference) | hands (Inspire) | keys
# Operator sequence (all in window "run"):
#   1. deploy pane: wait for "Init Done" (robot on the hoist; O = e-stop there).
#   2. inference pane: wait for "PolicyServer is reachable" + camera latency lines.
#   3. keys pane: k  (start C++ loop, PLANNER)  ->  i  (blend to initial pose)  ->  p  (run policy).
#      p again = pause,  k again = stop C++ loop.  t <text> = change prompt (stay on the training prompt!).
set -u
S=vla
REPO=~/GR00T-WholeBodyControl
PROMPT="${PROMPT:-put bottles with red cap in red bottle holder}"
POLICY_HOST="${POLICY_HOST:-127.0.0.1}"
POLICY_PORT="${POLICY_PORT:-5550}"
GPU_HOST="${GPU_HOST:-ubuntu@13.40.142.104}"          # darwin-gpu (EC2). Port 5550 is NOT open in its
TUNNEL_CMD="while true; do ssh -N -o ServerAliveInterval=15 -o ServerAliveCountMax=3 -o ExitOnForwardFailure=yes -o BatchMode=yes -L 127.0.0.1:${POLICY_PORT}:127.0.0.1:5550 ${GPU_HOST}; echo 'tunnel dropped, retrying in 3s'; sleep 3; done"
CAM_COLOR=/dev/v4l/by-id/usb-Remo_Tech_Co.__Ltd._OBSBOT_Tiny_2_Lite-video-index0
CAM_G1=/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_430i_Intel_R__RealSense_TM__Depth_Camera_430i_349623061587-video-index2
CAMG1_CMD='ssh g1 "gst-launch-1.0 v4l2src device='$CAM_G1' ! video/x-raw,format=GRAY8,width=640,height=480,framerate=15/1 ! videoconvert ! video/x-raw,format=I420 ! jpegenc quality=80 ! rtpjpegpay ! multiudpsink clients=192.168.123.222:5600,192.168.123.222:5601"'
# --autostart: capture + ZMQ tee run without a PICO Remote Vision session (headset optional).
CAMSEND_CMD='cd ~/XRoboToolkit-Orin-Video-Sender && ./OrinVideoSenderIR --listen 0.0.0.0:13579 --device '$CAM_COLOR' --pixfmt MJPG --width 1280 --height 720 --fps 30 --second-device udp:5600 --second-width 640 --second-height 480 --zmq-pub 5555 --autostart'
DEPLOY_ENTER='cd ~/GR00T-WholeBodyControl/gear_sonic_deploy && ./docker/run-ros2-dev.sh'
# Identical to teleop: run_vla_inference.py replaces the PICO streamer as the zmq_manager source (5556 in, 5557 out).
DEPLOY_RUN='./target/release/g1_deploy_onnx_ref enP2p1s0 policy/sonic_v1_1/model_decoder.onnx reference/example/ --obs-config policy/sonic_v1_1/observation_config.yaml --encoder-file policy/sonic_v1_1/model_encoder.onnx --planner-file planner/target_vel/V2/planner_sonic.onnx --input-type zmq_manager --output-type all --zmq-host localhost'
INFER_CMD="cd $REPO && source .venv_inference/bin/activate && python gear_sonic/scripts/run_vla_inference.py --host $POLICY_HOST --port $POLICY_PORT --embodiment-tag unitree_g1_sonic --prompt '$PROMPT' --camera-host 127.0.0.1 --camera-port 5555 --initial-motion-token-path gear_sonic/utils/inference/initial_motion_token_restocking.npy"
HANDS_CMD="cd $REPO && source .venv_inference/bin/activate && python gear_sonic/scripts/inspire_vla_bridge.py"
KEYS_CMD="cd $REPO && source .venv_inference/bin/activate && python tools/vla_keys.py"

case "${1:-up}" in
up)
  if pgrep -f igmp_querier.py >/dev/null; then echo "[ok] igmp querier running"
  elif sudo -n true 2>/dev/null; then sudo nohup python3 ~/igmp_querier.py >/tmp/igmp.log 2>&1 & echo "[ok] igmp querier started"
  else echo "[!!] IGMP QUERIER DOWN (needs sudo):  sudo nohup python3 ~/igmp_querier.py >/tmp/igmp.log 2>&1 &"; fi
  ping -c1 -W1 192.168.123.161 >/dev/null 2>&1 && echo "[ok] robot lowlevel up" || echo "[!!] robot lowlevel NOT reachable — power it on"
  ping -c1 -W1 192.168.123.164 >/dev/null 2>&1 && echo "[ok] robot PC (g1) up" || echo "[!!] robot PC (g1) not reachable — head camera push will wait"
  for ip in 192.168.123.210 192.168.123.211; do ping -c1 -W1 $ip >/dev/null 2>&1 && echo "[ok] inspire hand $ip up" || echo "[!!] inspire hand $ip not reachable"; done
  tmux kill-session -t $S 2>/dev/null
  pkill -f "pico_manage[r]_thread" 2>/dev/null          # the teleop streamer also binds 5556 — must be down
  pkill -f "OrinVideoSenderI[R]" 2>/dev/null
  pkill -f "run_vla_inferenc[e]" 2>/dev/null
  pkill -f "inspire_vla_bridg[e]" 2>/dev/null
  pkill -f "g1_deploy_onnx_re[f]" 2>/dev/null
  ssh -o BatchMode=yes -o ConnectTimeout=4 g1 'pkill -f "gst-launc[h]"' 2>/dev/null
  tmux kill-session -t vla-tunnel 2>/dev/null
  sleep 1
  tmux new-session -d -s $S -n svc
  P_TUN=$(tmux display -t $S:svc -p '#{pane_id}')
  P_CG1=$(tmux split-window -t "$P_TUN" -h -P -F '#{pane_id}')
  P_CAM=$(tmux split-window -t "$P_TUN" -v -P -F '#{pane_id}')
  tmux send-keys -t "$P_TUN" "$TUNNEL_CMD" C-m
  tmux send-keys -t "$P_CG1" "until ping -c1 -W1 192.168.123.164 >/dev/null 2>&1; do echo waiting for g1...; sleep 3; done; $CAMG1_CMD" C-m
  tmux send-keys -t "$P_CAM" "$CAMSEND_CMD" C-m
  tmux select-layout -t $S:svc tiled
  tmux new-window -t $S -n run
  P_DEP=$(tmux display -t $S:run -p '#{pane_id}')
  P_INF=$(tmux split-window -t "$P_DEP" -h -P -F '#{pane_id}')
  P_HND=$(tmux split-window -t "$P_INF" -v -P -F '#{pane_id}')
  P_KEY=$(tmux split-window -t "$P_DEP" -v -P -F '#{pane_id}')
  tmux send-keys -t "$P_DEP" "$DEPLOY_ENTER" C-m
  ( sleep 12; tmux send-keys -t "$P_DEP" "$DEPLOY_RUN" C-m ) &
  tmux send-keys -t "$P_INF" "sleep 4; $INFER_CMD" C-m
  if [ "${HANDS:-1}" = "1" ]; then tmux send-keys -t "$P_HND" "sleep 6; $HANDS_CMD" C-m; else tmux send-keys -t "$P_HND" "echo 'HANDS=0: Inspire bridge disabled (hands will not move, hand state = zeros)'" C-m; fi
  tmux send-keys -t "$P_KEY" "sleep 8; $KEYS_CMD" C-m
  tmux select-window -t $S:run
  tmux select-pane -t "$P_KEY"
  echo
  echo "Session '$S' up.  Attach:   tmux attach -t $S"
  echo "  run window: TOP-LEFT deploy ('O' = e-stop, wait for 'Init Done') | TOP-RIGHT inference | BOTTOM-RIGHT hands | BOTTOM-LEFT keys"
  echo "  keys pane:  k = START C++ loop   x = STOP C++ loop   i = initial pose   p = run/pause policy   t <text> = prompt"
  echo "  svc window (Ctrl-b n): tunnel -> darwin-gpu | cam-g1 head push | cam-sender (ZMQ 5555, --autostart)"
  echo "  Policy server: make sure darwin-gpu serves the model you want:  ssh darwin-gpu '~/serve_policy.sh v2|best34'"
  echo "  Prompt: \"$PROMPT\"  (the training prompt — anything else is out of distribution)"
  ;;
down)
  tmux kill-session -t $S 2>/dev/null
  pkill -f "run_vla_inferenc[e]" 2>/dev/null
  pkill -f "inspire_vla_bridg[e]" 2>/dev/null           # its stop() opens the hands
  pkill -f "OrinVideoSenderI[R]" 2>/dev/null
  pkill -f "g1_deploy_onnx_re[f]" 2>/dev/null
  ssh -o BatchMode=yes -o ConnectTimeout=4 g1 'pkill -f "gst-launc[h]"' 2>/dev/null
  echo "vla session down"
  ;;
status)
  tmux ls 2>/dev/null | grep -E "^$S:" || echo "no '$S' session"
  ss -ltn 2>/dev/null | grep -E ":5550 |:5555 |:5556 |:5557 |:5558 |:5580 " | awk '{print "  listening", $4}'
  pgrep -af "run_vla_inferenc[e]|inspire_vla_bridg[e]|OrinVideoSenderI[R]|g1_deploy_onnx_re[f]" | cut -c1-120
  ;;
*) echo "usage: $0 up|down|status"; exit 1 ;;
esac

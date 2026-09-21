#!/usr/bin/env bash
# One-command GR00T VLA inference on the G1 (restocking task) — mirrors tools/sonic-teleop.sh.
#   ~/GR00T-WholeBodyControl/tools/vla-inference.sh up|down|status
# Env overrides: POLICY_MODE=local|ec2 (default local: GR00T server on this Thor, loopback 5550;
#                ec2: SSH tunnel -> darwin-gpu:5550, start the server there yourself),
#                POLICY_MODEL=best34|v2 (checkpoint the local server loads), PROMPT, POLICY_HOST/POLICY_PORT,
#                HANDS=0 (skip Inspire bridge)
#
# Layout:  window "svc": serve (local policy server) or tunnel | cam-g1 (head IR push) | cam-sender (OBSBOT + head, --autostart, ZMQ 5555)
#          window "run": deploy (C++ SONIC, container) | inference (run_vla_inference) | hands (Inspire) | keys
# Operator sequence (all in window "run"):
#   1. deploy pane: wait for "Init Done" (robot on the hoist; O = e-stop there).
#   2. inference pane: it waits for the policy server (local model load ~1-2 min), then prints
#      "PolicyServer is reachable" + "Policy video keys (from server): ['head_view', 'ego_view']" + camera latency lines.
#   3. keys pane: k  (start C++ loop, PLANNER)  ->  i  (blend to initial pose)  ->  p  (run policy).
#      p again = pause,  k again = stop C++ loop.  t <text> = change prompt (stay on the training prompt!).
set -u
S=vla
REPO=~/GR00T-WholeBodyControl
PROMPT="${PROMPT:-put bottles with red cap in red bottle holder}"
POLICY_MODE="${POLICY_MODE:-local}"                    # local (Thor venv, rig/thor) | ec2 (tunnel to darwin-gpu)
POLICY_MODEL="${POLICY_MODEL:-best34}"
POLICY_HOST="${POLICY_HOST:-127.0.0.1}"
POLICY_PORT="${POLICY_PORT:-5550}"
THOR_RIG="${THOR_RIG:-$HOME/g1-vr-teleop/rig/thor}"    # prosus-robotics/g1-vr-teleop clone: serve_policy.sh + Isaac-GR00T venv install
SERVE_CMD="$THOR_RIG/serve_policy.sh $POLICY_MODEL $POLICY_PORT"
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
# run_vla_inference.py queries the server's video keys ONCE at start-up and falls back to ego_view only if the
# server is down -> never start it before the server listens (a local server needs 1-2 min to load 12 GB).
WAIT_SERVER="until ss -ltn | grep -q ':$POLICY_PORT '; do echo 'waiting for policy server on :$POLICY_PORT ...'; sleep 3; done"
INFER_CMD="cd $REPO && source .venv_inference/bin/activate && python gear_sonic/scripts/run_vla_inference.py --host $POLICY_HOST --port $POLICY_PORT --embodiment-tag unitree_g1_sonic --prompt '$PROMPT' --camera-host 127.0.0.1 --camera-port 5555 --initial-motion-token-path gear_sonic/utils/inference/initial_motion_token_restocking.npy"
HANDS_CMD="cd $REPO && source .venv_inference/bin/activate && python gear_sonic/scripts/inspire_vla_bridge.py"
KEYS_CMD="cd $REPO && source .venv_inference/bin/activate && python tools/vla_keys.py"

case "${1:-up}" in
up)
  case "$POLICY_MODE" in local|ec2) ;; *) echo "POLICY_MODE must be local or ec2"; exit 1;; esac
  if [ "$POLICY_MODE" = local ]; then
    [ -x "$THOR_RIG/serve_policy.sh" ] || { echo "[!!] $THOR_RIG/serve_policy.sh missing — clone prosus-robotics/g1-vr-teleop or use POLICY_MODE=ec2"; exit 1; }
    [ -d "$HOME/checkpoints/restocking_$POLICY_MODEL/checkpoint-10000" ] || { echo "[!!] no local checkpoint ~/checkpoints/restocking_$POLICY_MODEL — see rig/thor/README.md"; exit 1; }
  fi
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
  pkill -f "run_gr00t_serve[r].py" 2>/dev/null           # stale local policy server would hold 5550
  sleep 1
  tmux new-session -d -s $S -n svc
  P_TUN=$(tmux display -t $S:svc -p '#{pane_id}')
  P_CG1=$(tmux split-window -t "$P_TUN" -h -P -F '#{pane_id}')
  P_CAM=$(tmux split-window -t "$P_TUN" -v -P -F '#{pane_id}')
  if [ "$POLICY_MODE" = local ]; then tmux send-keys -t "$P_TUN" "$SERVE_CMD" C-m; else tmux send-keys -t "$P_TUN" "$TUNNEL_CMD" C-m; fi
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
  tmux send-keys -t "$P_INF" "sleep 4; $WAIT_SERVER; $INFER_CMD" C-m
  if [ "${HANDS:-1}" = "1" ]; then tmux send-keys -t "$P_HND" "sleep 6; $HANDS_CMD" C-m; else tmux send-keys -t "$P_HND" "echo 'HANDS=0: Inspire bridge disabled (hands will not move, hand state = zeros)'" C-m; fi
  tmux send-keys -t "$P_KEY" "sleep 8; $KEYS_CMD" C-m
  tmux select-window -t $S:run
  tmux select-pane -t "$P_KEY"
  echo
  echo "Session '$S' up.  Attach:   tmux attach -t $S"
  echo "  run window: TOP-LEFT deploy ('O' = e-stop, wait for 'Init Done') | TOP-RIGHT inference | BOTTOM-RIGHT hands | BOTTOM-LEFT keys"
  echo "  keys pane:  k = START C++ loop   x = STOP C++ loop   i = initial pose   p = run/pause policy   t <text> = prompt"
  if [ "$POLICY_MODE" = local ]; then
    echo "  svc window (Ctrl-b n): serve (local GR00T server, model $POLICY_MODEL, loads ~1-2 min) | cam-g1 head push | cam-sender (ZMQ 5555, --autostart)"
    echo "  Other model: POLICY_MODEL=v2 $0 up.   EC2 fallback: POLICY_MODE=ec2 $0 up  (+ ssh darwin-gpu '~/serve_policy.sh v2|best34')"
  else
    echo "  svc window (Ctrl-b n): tunnel -> darwin-gpu | cam-g1 head push | cam-sender (ZMQ 5555, --autostart)"
    echo "  Policy server: make sure darwin-gpu serves the model you want:  ssh darwin-gpu '~/serve_policy.sh v2|best34'"
  fi
  echo "  Prompt: \"$PROMPT\"  (the training prompt — anything else is out of distribution)"
  ;;
down)
  tmux kill-session -t $S 2>/dev/null
  pkill -f "run_vla_inferenc[e]" 2>/dev/null
  pkill -f "run_gr00t_serve[r].py" 2>/dev/null           # local policy server (ec2 mode: nothing to kill here)
  pkill -f "inspire_vla_bridg[e]" 2>/dev/null           # its stop() opens the hands
  pkill -f "OrinVideoSenderI[R]" 2>/dev/null
  pkill -f "g1_deploy_onnx_re[f]" 2>/dev/null
  ssh -o BatchMode=yes -o ConnectTimeout=4 g1 'pkill -f "gst-launc[h]"' 2>/dev/null
  echo "vla session down"
  ;;
status)
  tmux ls 2>/dev/null | grep -E "^$S:" || echo "no '$S' session"
  ss -ltn 2>/dev/null | grep -E ":5550 |:5555 |:5556 |:5557 |:5558 |:5580 " | awk '{print "  listening", $4}'
  pgrep -af "run_vla_inferenc[e]|inspire_vla_bridg[e]|OrinVideoSenderI[R]|g1_deploy_onnx_re[f]|run_gr00t_serve[r]" | cut -c1-120
  ;;
*) echo "usage: $0 up|down|status"; exit 1 ;;
esac

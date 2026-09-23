#!/usr/bin/env bash
# One-command GR00T VLA inference on the G1 (restocking task) — mirrors tools/sonic-teleop.sh.
#   ~/GR00T-WholeBodyControl/tools/vla-inference.sh up|down|status
# Env overrides: POLICY_MODE=local|ec2 (default local: GR00T server on this Thor, loopback 5550;
#                ec2: SSH tunnel -> darwin-gpu:5550, start the server there yourself),
#                POLICY_MODEL=best34|v2 (checkpoint the local server loads), PROMPT, POLICY_HOST/POLICY_PORT,
#                HANDS=0 (skip Inspire bridge), HANDS_SIDES=right|left (one hand detached; that side's
#                hand state falls back to the C++ zeros = open, which is what the training data shows)
#                RECORD=1: record every closed-loop attempt as a LeRobot episode (g1-vr-teleop #25): the ZMQ
#                  exporter runs in the wbc-marin container, the keys pane becomes record_keys_zmq.py (c = start /
#                  stop+save, x = discard, p/i/k/[/] still go to the VLA client, t <text> = prompt of both).
#                  Frames are labelled stream_mode 6 (POLICY); process_dataset.py keeps them. Env: DATASET,
#                  DATASET_BUCKET. No headset, no streamer.
#                DAGGER=1: RECORD=1 plus interventions from the PICO operator (g1-vr-teleop #25).
#                  The PICO streamer owns the deploy socket (POLICY mode relays the policy's tokens), the
#                  exporter records every attempt as a LeRobot episode, the VLA client runs in --relay mode
#                  behind the streamer, and the streamer's Inspire bridge drives the hands (no inspire_vla_bridge).
#                  Extra env: DATASET (outputs/<name>, default policy_<date>), DATASET_BUCKET (S3, default
#                  darwin-robot-data), INSPIRE_HANDS=trigger|handtracking|off. Needs the hotspot + PICO like teleop.
#
# Layout:  window "svc": serve (local policy server) or tunnel | cam-g1 (head IR push) | cam-sender (OBSBOT + head, --autostart, ZMQ 5555)
#          window "run": deploy (C++ SONIC, container) | inference (run_vla_inference) | hands (Inspire) | keys
#          DAGGER=1: svc also has xr-service; run = deploy | inference | streamer | exporter | keys (record_keys_zmq --line-mode)
# Safety: the inference client pauses itself after 1 s without a valid observation (camera view missing/stale,
#         no robot state) and stays paused until you press p again (--sensor-loss-pause-s, #26). The cam-g1 pane
#         restarts the head push on its own when the camera comes back.
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
# The D430i sometimes re-enumerates WITHOUT its serial in the by-id name (seen 2026-09-22 after a replug), so the
# watchdog matches by glob and resolves the node each time it (re)starts the push. index2 = left IR stream.
CAM_G1_GLOB='/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_430i_*video-index2'
GST_G1_TAIL="! video/x-raw,format=GRAY8,width=640,height=480,framerate=15/1 ! videoconvert ! video/x-raw,format=I420 ! jpegenc quality=80 ! rtpjpegpay ! multiudpsink clients=192.168.123.222:5600,192.168.123.222:5601"
# Head-camera watchdog (issue #26): runs ON g1 inside one ssh session. Waits for the D430i device node, starts
# the push, and restarts it when it exits or the camera drops off the USB bus. Safe now because the VLA client
# pauses itself after 1 s without a valid observation and stays paused until 'p' (it used to resume unprompted).
# -tt gives the remote loop a pty so killing the pane hangs it up; up/down also pkill the HEADCAM_WATCHDOG marker.
headcam_push_cmd() {   # $1 device glob, $2 gst pipeline after "v4l2src device=<node>"  -> remote bash command (no single quotes inside)
  echo "HEADCAM_WATCHDOG=1; while true; do DEV=\$(ls $1 2>/dev/null | head -1); until [ -n \"\$DEV\" ]; do echo \"[\$(date +%T)] head camera device missing - D430i off the USB bus? unplug 10 s, replug\"; sleep 2; DEV=\$(ls $1 2>/dev/null | head -1); done; echo \"[\$(date +%T)] head camera present at \$DEV, starting push\"; gst-launch-1.0 v4l2src device=\$DEV $2; echo \"[\$(date +%T)] head push exited, retrying in 2 s (VLA client pauses itself on sensor loss; press p to resume)\"; sleep 2; done"
}
CAMG1_CMD="ssh -tt g1 '$(headcam_push_cmd "$CAM_G1_GLOB" "$GST_G1_TAIL")'"
# --autostart: capture + ZMQ tee run without a PICO Remote Vision session (headset optional).
CAMSEND_CMD='cd ~/XRoboToolkit-Orin-Video-Sender && ./OrinVideoSenderIR --listen 0.0.0.0:13579 --device '$CAM_COLOR' --pixfmt MJPG --width 1280 --height 720 --fps 30 --second-device udp:5600 --second-width 640 --second-height 480 --zmq-pub 5555 --autostart'
DEPLOY_ENTER='cd ~/GR00T-WholeBodyControl/gear_sonic_deploy && ./docker/run-ros2-dev.sh'
# Identical to teleop: run_vla_inference.py replaces the PICO streamer as the zmq_manager source (5556 in, 5557 out).
DEPLOY_RUN='./target/release/g1_deploy_onnx_ref enP2p1s0 policy/sonic_v1_1/model_decoder.onnx reference/example/ --obs-config policy/sonic_v1_1/observation_config.yaml --encoder-file policy/sonic_v1_1/model_encoder.onnx --planner-file planner/target_vel/V2/planner_sonic.onnx --input-type zmq_manager --output-type all --zmq-host localhost'
# run_vla_inference.py queries the server's video keys ONCE at start-up and falls back to ego_view only if the
# server is down -> never start it before the server listens (a local server needs 1-2 min to load 12 GB).
WAIT_SERVER="until ss -ltn | grep -q ':$POLICY_PORT '; do echo 'waiting for policy server on :$POLICY_PORT ...'; sleep 3; done"
INFER_CMD="cd $REPO && source .venv_inference/bin/activate && python gear_sonic/scripts/run_vla_inference.py --host $POLICY_HOST --port $POLICY_PORT --embodiment-tag unitree_g1_sonic --prompt '$PROMPT' --camera-host 127.0.0.1 --camera-port 5555 --initial-motion-token-path gear_sonic/utils/inference/initial_motion_token_restocking.npy"
HANDS_SIDES="${HANDS_SIDES:-left,right}"               # e.g. HANDS_SIDES=right when the left Inspire hand is detached
HANDS_CMD="cd $REPO && source .venv_inference/bin/activate && python gear_sonic/scripts/inspire_vla_bridge.py --sides $HANDS_SIDES"
KEYS_CMD="cd $REPO && source .venv_inference/bin/activate && python tools/vla_keys.py"
# DAGGER=1 (g1-vr-teleop #25): streamer owns 5556 and relays the VLA client's PUB on 5576 in POLICY mode.
DAGGER="${DAGGER:-0}"
RECORD="${RECORD:-0}"; [ "$DAGGER" = "1" ] && RECORD=1
POLICY_ACTION_PORT=5576
DATASET="${DATASET:-policy_$(date +%F)}"
DATASET_BUCKET="${DATASET_BUCKET:-darwin-robot-data}"
INSPIRE_HANDS="${INSPIRE_HANDS:-trigger}"
STREAMER_CMD="cd $REPO && source .venv_teleop/bin/activate && python gear_sonic/scripts/pico_manager_thread_server.py --manager --input-source xrt --inspire-hands $INSPIRE_HANDS --policy-port $POLICY_ACTION_PORT --keys-port 5580"
XRSVC_CMD='pgrep -f RoboticsServiceProcess >/dev/null && echo "xr-service already running" || DISPLAY=:0 ~/start_xrsvc.sh'
EXPORTER_CMD='~/wbc-marin-exec.sh python gear_sonic/scripts/run_data_exporter.py --camera-host 127.0.0.1 --camera-port 5555 --dataset-name '"$DATASET"' --task-prompt "'"$PROMPT"'" --add-head-camera --upload-bucket-path '"$DATASET_BUCKET"' --no-text-to-speech'
# One keys pane for all three subscribers: c/x = episode (exporter), p/i/[/] = VLA client, g/h = streamer, t <text> = prompt of both.
DAGGER_KEYS_CMD='~/wbc-marin-exec.sh python /workspace/wbc/tools/record_keys_zmq.py --line-mode --passthrough-keys "pigh[]" --base-prompt "'"$PROMPT"'"'
# As in sonic-teleop.sh: the exporter pane is the proof of what got recorded, so `script` keeps a copy of it
# in logs/exporter-last-run.log (single-quote-safe copy of the command for script -c '...').
EXP_Q=${EXPORTER_CMD//\'/\'\\\'\'}
# RECORD=1 without the streamer: k still starts the C++ loop from the keys pane; x is the exporter's discard.
RECORD_KEYS_CMD='~/wbc-marin-exec.sh python /workspace/wbc/tools/record_keys_zmq.py --line-mode --passthrough-keys "pik[]" --base-prompt "'"$PROMPT"'"'
if [ "$DAGGER" = "1" ]; then
  INFER_CMD="$INFER_CMD --relay --action-zmq-port $POLICY_ACTION_PORT"
elif [ "$RECORD" = "1" ]; then
  INFER_CMD="$INFER_CMD --exporter-keys"
fi

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
  if [ "$DAGGER" = "1" ]; then
    if ip -brief addr show wlP1p1s0 2>/dev/null | grep -q UP; then echo "[ok] hotspot up (PICO)"
    elif sudo -n true 2>/dev/null; then sudo nmcli con up quest-hotspot >/dev/null 2>&1 && echo "[ok] hotspot started" || echo "[!!] hotspot failed to start"
    else echo "[!!] HOTSPOT DOWN (needs sudo):  sudo nmcli con up quest-hotspot"; fi
  fi
  tmux kill-session -t $S 2>/dev/null
  pkill -f "pico_manage[r]_thread" 2>/dev/null          # the teleop streamer also binds 5556 — must be down (DAGGER starts its own)
  pkill -f "OrinVideoSenderI[R]" 2>/dev/null
  pkill -f "run_vla_inferenc[e]" 2>/dev/null
  pkill -f "inspire_vla_bridg[e]" 2>/dev/null
  pkill -f "g1_deploy_onnx_re[f]" 2>/dev/null
  docker exec wbc-marin bash -c 'pkill -f "run_.*data_exporte[r]"; pkill -f "record_key[s]"' 2>/dev/null
  ssh -o BatchMode=yes -o ConnectTimeout=4 g1 'pkill -f "HEADCAM_WATCHDO[G]"; pkill -f "gst-launc[h]"' 2>/dev/null
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
  if [ "$DAGGER" = "1" ]; then
    P_XR=$(tmux split-window -t "$P_CG1" -v -P -F '#{pane_id}')
    tmux send-keys -t "$P_XR" "$XRSVC_CMD" C-m
  fi
  tmux select-layout -t $S:svc tiled
  tmux new-window -t $S -n run
  P_DEP=$(tmux display -t $S:run -p '#{pane_id}')
  P_INF=$(tmux split-window -t "$P_DEP" -h -P -F '#{pane_id}')
  P_HND=$(tmux split-window -t "$P_INF" -v -P -F '#{pane_id}')
  P_KEY=$(tmux split-window -t "$P_DEP" -v -P -F '#{pane_id}')
  tmux send-keys -t "$P_DEP" "$DEPLOY_ENTER" C-m
  ( sleep 12; tmux send-keys -t "$P_DEP" "$DEPLOY_RUN" C-m ) &
  tmux send-keys -t "$P_INF" "sleep 4; $WAIT_SERVER; $INFER_CMD" C-m
  if [ "$DAGGER" = "1" ]; then
    # hands pane -> exporter; the pane under the deploy -> streamer; a new pane below it -> keys
    P_STR="$P_KEY"
    P_KEY=$(tmux split-window -t "$P_STR" -v -P -F '#{pane_id}')
    tmux send-keys -t "$P_STR" "until pgrep -f RoboticsServiceProcess >/dev/null; do echo waiting for xr-service...; sleep 1; done; sleep 2; $STREAMER_CMD" C-m
    tmux send-keys -t "$P_HND" "docker start wbc-marin >/dev/null 2>&1; sleep 6; mkdir -p $REPO/logs; script -qfc '$EXP_Q' $REPO/logs/exporter-last-run.log" C-m
    tmux send-keys -t "$P_KEY" "docker start wbc-marin >/dev/null 2>&1; sleep 8; $DAGGER_KEYS_CMD" C-m
  elif [ "$RECORD" = "1" ]; then
    # hands as usual; exporter under the keys pane (left column), keys = record_keys_zmq (line mode)
    if [ "${HANDS:-1}" = "1" ]; then tmux send-keys -t "$P_HND" "sleep 6; $HANDS_CMD" C-m; else tmux send-keys -t "$P_HND" "echo 'HANDS=0: Inspire bridge disabled (hands will not move, hand state = zeros)'" C-m; fi
    P_EXP=$(tmux split-window -t "$P_KEY" -v -P -F '#{pane_id}')
    tmux send-keys -t "$P_EXP" "docker start wbc-marin >/dev/null 2>&1; sleep 6; mkdir -p $REPO/logs; script -qfc '$EXP_Q' $REPO/logs/exporter-last-run.log" C-m
    tmux send-keys -t "$P_KEY" "docker start wbc-marin >/dev/null 2>&1; sleep 8; $RECORD_KEYS_CMD" C-m
  else
    if [ "${HANDS:-1}" = "1" ]; then tmux send-keys -t "$P_HND" "sleep 6; $HANDS_CMD" C-m; else tmux send-keys -t "$P_HND" "echo 'HANDS=0: Inspire bridge disabled (hands will not move, hand state = zeros)'" C-m; fi
    tmux send-keys -t "$P_KEY" "sleep 8; $KEYS_CMD" C-m
  fi
  tmux select-window -t $S:run
  tmux select-pane -t "$P_KEY"
  echo
  echo "Session '$S' up.  Attach:   tmux attach -t $S"
  if [ "$DAGGER" = "1" ]; then
    echo "  DAGGER run window: TOP-LEFT deploy ('O' = e-stop, 'Init Done') | TOP-RIGHT inference (--relay) | MID-LEFT streamer | BOTTOM-RIGHT exporter | BOTTOM-LEFT keys"
    echo "  PICO: calibration pose -> A+B+X+Y (stand, PLANNER)  ->  keys: g (POLICY mode)  ->  i  ->  p  ->  c (record)"
    echo "        intervene: left-stick click / B+X (or h) = you drive the arms + hands; same again = policy resumes"
    echo "        c again = stop+save, x = discard; A+X = PLANNER; A+B+X+Y = stop.  Episodes: $REPO/outputs/$DATASET/  (prompt: \"$PROMPT\")"
    echo "        Every key needs Enter in the keys pane (line mode). Remote Vision: open it BEFORE g and keep it open."
  elif [ "$RECORD" = "1" ]; then
    echo "  RECORD run window: TOP-LEFT deploy ('O' = e-stop / STOP, 'Init Done') | TOP-RIGHT inference | MID-LEFT keys | BOTTOM-LEFT exporter | BOTTOM-RIGHT hands"
    echo "  keys pane (Enter after each key):  k = START C++ loop   i = initial pose   p = run/pause   c = start / stop+save episode   x = DISCARD episode   t <text> = prompt"
    echo "  Watch the exporter pane for 'Started recording N: ... [POLICY episode]'. Episodes: $REPO/outputs/$DATASET/  -> s3://$DATASET_BUCKET/raw/$DATASET/"
  else
  echo "  run window: TOP-LEFT deploy ('O' = e-stop, wait for 'Init Done') | TOP-RIGHT inference | BOTTOM-RIGHT hands | BOTTOM-LEFT keys"
  echo "  keys pane:  k = START C++ loop   x = STOP C++ loop   i = initial pose   p = run/pause policy   t <text> = prompt"
  fi
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
  # graceful stop first (the streamer's Ctrl-C opens the hands, the exporter saves/uploads), then kill the rest
  for p in $(tmux list-panes -s -t $S -F '#{pane_id}' 2>/dev/null); do tmux send-keys -t $p C-c 2>/dev/null; done
  sleep 3
  tmux kill-session -t $S 2>/dev/null
  pkill -f "run_vla_inferenc[e]" 2>/dev/null
  pkill -f "run_gr00t_serve[r].py" 2>/dev/null           # local policy server (ec2 mode: nothing to kill here)
  pkill -f "inspire_vla_bridg[e]" 2>/dev/null           # its stop() opens the hands
  pkill -f "pico_manage[r]_thread" 2>/dev/null          # DAGGER streamer (its Inspire bridge opens the hands on SIGTERM)
  docker exec wbc-marin bash -c 'pkill -f "run_.*data_exporte[r]"; pkill -f "record_key[s]"' 2>/dev/null
  pkill -f "OrinVideoSenderI[R]" 2>/dev/null
  pkill -f "g1_deploy_onnx_re[f]" 2>/dev/null
  ssh -o BatchMode=yes -o ConnectTimeout=4 g1 'pkill -f "HEADCAM_WATCHDO[G]"; pkill -f "gst-launc[h]"' 2>/dev/null
  echo "vla session down"
  ;;
status)
  tmux ls 2>/dev/null | grep -E "^$S:" || echo "no '$S' session"
  ss -ltn 2>/dev/null | grep -E ":5550 |:5555 |:5556 |:5557 |:5558 |:5576 |:5580 " | awk '{print "  listening", $4}'
  pgrep -af "run_vla_inferenc[e]|inspire_vla_bridg[e]|OrinVideoSenderI[R]|g1_deploy_onnx_re[f]|run_gr00t_serve[r]|pico_manage[r]_thread" | cut -c1-120
  echo -n "  data exporter (container): "; docker exec wbc-marin pgrep -f "run_.*data_exporter" >/dev/null 2>&1 && echo UP || echo down
  ;;
camcmd) echo "$CAMG1_CMD" ;;                              # paste into the svc cam-g1 pane after Ctrl-C to restart the head push by hand
*) echo "usage: $0 up|down|status|camcmd"; exit 1 ;;
esac

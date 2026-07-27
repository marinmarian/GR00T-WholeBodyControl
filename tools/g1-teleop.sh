#!/bin/bash
# g1-teleop.sh — one-command G1 right-arm teleop stack, all in one tmux session.
#
#   ~/g1-teleop.sh up       start everything (then: tmux attach -t g1)
#   ~/g1-teleop.sh down     stop everything (DAMP THE ROBOT FIRST: 'o' then L2+B)
#   ~/g1-teleop.sh status   show what's running
#
# tmux session "g1", two windows:
#   svc : [xr-service] [head-cam(ssh g1)] [pico-bridge]
#   run : [CONTROL LOOP  <- keys ] l o go HERE] [teleop loop]
#
# Startup ordering/waits are built in:
#   xr-service -> bridge (waits for RoboticsServiceProcess)
#   control loop -> teleop loop (waits for bridge :5555 + control loop in container)
#
# Manual steps that stay manual:
#   * PICO app: PC service 192.168.123.222 (WORKING) + Remote Vision -> 192.168.123.164
#   * keys in run window: ]  = balance,  l = teleop on,  o = off   (L2+B = damp)
#   * IGMP querier needs sudo: run it yourself if this script can't (it will tell you).

S=g1
CONTROL_CMD='~/wbc-exec.sh python decoupled_wbc/control/main/teleop/run_g1_control_loop.py --interface real --no-with-hands --tracked_hands right'
TELEOP_CMD='~/wbc-exec.sh python decoupled_wbc/control/main/teleop/run_teleop_policy_loop.py --body_control_device quest --body_streamer_ip 127.0.0.1 --body_streamer_keyword wrist --hand_control_device None --tracked_hands right'
BRIDGE_CMD='cd ~/GR00T-WholeBodyControl && source .venv_teleop/bin/activate && python -u pico_vive_bridge.py --tracked_hands right --inspire-hands trigger'
# NOTE: launch-only — no pkill in this string (the launch text itself would make a
# remote pkill -f self-match and kill its own wrapper). Cleanup happens in up/down bodies.
CAMERA_CMD='ssh g1 "cd ~/XRoboToolkit-Orin-Video-Sender && exec ./OrinVideoSenderIR --listen 0.0.0.0:13579 --device /dev/v4l/by-id/usb-Intel_Intel_F450_00.00.01-video-index0 --pixfmt YUY2 --width 704 --height 1280 --fps 15 --second-device /dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_430i_Intel_R__RealSense_TM__Depth_Camera_430i_349623061587-video-index2 --second-pixfmt GRAY8 --second-width 640 --second-height 480 --second-fps 15"'
XRSVC_CMD='pgrep -f RoboticsServiceProcess >/dev/null && echo "xr-service already running" || DISPLAY=:0 ~/start_xrsvc.sh'

case "${1:-up}" in
# ─────────────────────────────────────────────────────────────────────────────
up)
  # 0. IGMP querier (sudo). Try cached credentials; otherwise tell the user.
  if pgrep -f igmp_querier.py >/dev/null; then
    echo "[ok] igmp querier running"
  elif sudo -n true 2>/dev/null; then
    sudo nohup python3 ~/igmp_querier.py >/tmp/igmp.log 2>&1 &
    echo "[ok] igmp querier started"
  else
    echo "############################################################"
    echo "# IGMP QUERIER NOT RUNNING (needs sudo).  In any terminal: #"
    echo "#     sudo python3 ~/igmp_querier.py &                     #"
    echo "# Without it the DDS link dies ~4 min after connecting.    #"
    echo "############################################################"
  fi

  # 1. container
  docker start wbc-dev >/dev/null 2>&1 && echo "[ok] wbc-dev container up"

  # 2. clean slate for the session (leave igmp/container alone)
  tmux kill-session -t $S 2>/dev/null
  docker exec wbc-dev bash -c 'pkill -9 -f "run_teleop_policy_loo[p]"; pkill -9 -f "run_g1_control_loo[p]"' 2>/dev/null
  pkill -f pico_vive_bridge.py 2>/dev/null
  ssh -o BatchMode=yes g1 'pkill -9 -f "OrinVideoSenderI[R]"' 2>/dev/null
  sleep 1

  # 3. svc window: xr-service | head-cam | bridge  (address panes by ID — index-proof)
  tmux new-session -d -s $S -n svc
  P_XR=$(tmux display -t $S:svc -p '#{pane_id}')
  P_CAM=$(tmux split-window -t "$P_XR" -h -P -F '#{pane_id}')
  P_BR=$(tmux split-window -t "$P_XR" -v -P -F '#{pane_id}')
  tmux send-keys -t "$P_XR"  "$XRSVC_CMD" C-m
  tmux send-keys -t "$P_CAM" "$CAMERA_CMD" C-m
  tmux send-keys -t "$P_BR"  "until pgrep -f RoboticsServiceProcess >/dev/null; do echo waiting for xr-service...; sleep 1; done; sleep 3; $BRIDGE_CMD" C-m
  tmux select-layout -t $S:svc tiled

  # 4. run window: control loop | teleop loop (keys go to the LEFT pane)
  tmux new-window -t $S -n run
  P_CTL=$(tmux display -t $S:run -p '#{pane_id}')
  P_TEL=$(tmux split-window -t "$P_CTL" -h -P -F '#{pane_id}')
  tmux send-keys -t "$P_CTL" "$CONTROL_CMD" C-m
  tmux send-keys -t "$P_TEL" "until (echo > /dev/tcp/127.0.0.1/5555) 2>/dev/null; do echo waiting for bridge...; sleep 1; done; until docker exec wbc-dev pgrep -f run_g1_control_loop >/dev/null; do echo waiting for control loop...; sleep 1; done; echo control loop up, giving it 12s...; sleep 12; $TELEOP_CMD" C-m
  tmux select-window -t $S:run
  tmux select-pane   -t "$P_CTL"

  echo
  echo "Session '$S' is up.  Attach with:   tmux attach -t $S"
  echo "  window 'run' (you land here): LEFT pane = control loop -> press ] then l"
  echo "  window 'svc' (Ctrl-b n):      xr-service / head-cam / bridge logs"
  echo "  PICO app: PC service 192.168.123.222 -> WORKING; Remote Vision -> 192.168.123.164"
  echo "  DO NOT press l until the bridge pane shows moving 'R pos(...)'."
  ;;
# ─────────────────────────────────────────────────────────────────────────────
down)
  echo "!! Make sure the robot is deactivated ('o') and damped (L2+B) first."
  docker exec wbc-dev bash -c 'pkill -f "run_teleop_policy_loo[p]"; sleep 1; pkill -f "run_g1_control_loo[p]"; sleep 1; pkill -9 -f "run_teleop_policy_loo[p]"; pkill -9 -f "run_g1_control_loo[p]"' 2>/dev/null
  sleep 1
  tmux kill-session -t $S 2>/dev/null
  pkill -f pico_vive_bridge.py 2>/dev/null
  pkill -f RoboticsServiceProcess 2>/dev/null
  ssh -o BatchMode=yes g1 'pkill -9 -f OrinVideoSenderI[R]' 2>/dev/null
  echo "[ok] stopped session, loops, bridge, xr-service, head-cam."
  echo "     still running: wbc-dev container, igmp querier (sudo pkill -f igmp_querier.py)"
  ;;
# ─────────────────────────────────────────────────────────────────────────────
status)
  echo -n "tmux session:   "; tmux has-session -t $S 2>/dev/null && echo UP || echo down
  echo -n "wbc-dev:        "; docker ps --format '{{.Status}}' -f name=wbc-dev | grep -q . && echo UP || echo down
  echo -n "igmp querier:   "; pgrep -f igmp_querier.py >/dev/null && echo UP || echo DOWN
  echo -n "xr-service:     "; pgrep -f RoboticsServiceProcess >/dev/null && echo UP || echo down
  echo -n "pico bridge:    "; pgrep -f pico_vive_bridge.py >/dev/null && echo UP || echo down
  echo -n "control loop:   "; docker exec wbc-dev pgrep -f run_g1_control_loop >/dev/null 2>&1 && echo UP || echo down
  echo -n "teleop loop:    "; docker exec wbc-dev pgrep -f run_teleop_policy_loop >/dev/null 2>&1 && echo UP || echo down
  echo -n "head-cam (g1):  "; ssh -o BatchMode=yes -o ConnectTimeout=4 g1 'pgrep -f "OrinVideoSenderI[R]" >/dev/null' 2>/dev/null && echo UP || echo down
  echo -n "right hand:     "; timeout 1 bash -c 'echo > /dev/tcp/192.168.123.211/6000' 2>/dev/null && echo UP || echo down
  ;;
*)
  echo "usage: $0 {up|down|status}"; exit 1 ;;
esac

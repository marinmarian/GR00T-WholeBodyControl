#!/bin/bash
# Stream the G1 head-camera IR feed to the PICO via XRoboToolkit "Remote Vision".
# Camera: Intel RealSense D430i, left-IR node /dev/video2, GRAY8 640x480 -> HW H.264.
# In the PICO app: Remote Vision -> camera source IP = this robot PC's IP (port below).
# The PICO connects, sends OPEN_CAMERA with its callback ip:port, and we stream H.264 back.
# Usage:  ./run_headcam_sender.sh [listen_port]     (default 13579)
#
# Needs OrinVideoSenderIR, built from main_web_ir.cpp in a clone of
# XR-Robotics/XRoboToolkit-Orin-Video-Sender (see README.md). Override the clone
# location with SENDER_DIR=... if it is not ~/XRoboToolkit-Orin-Video-Sender.
#
# If the PICO is on mjolnir's 10.42.0.x hotspot (not the wired 192.168.123.0/24 LAN),
# first add a return route on this PC:
#   sudo ip route add 10.42.0.0/24 via 192.168.123.222
set -e
PORT="${1:-13579}"
SENDER_DIR="${SENDER_DIR:-$HOME/XRoboToolkit-Orin-Video-Sender}"
cd "$SENDER_DIR"
echo "[sender] listening on 0.0.0.0:$PORT  (PICO Remote Vision -> camera source IP = this robot PC)"
exec ./OrinVideoSenderIR --listen "0.0.0.0:$PORT"

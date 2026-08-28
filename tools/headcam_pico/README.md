# Head camera -> PICO (XRoboToolkit "Remote Vision")

Show the G1 head camera as a 2D screen inside the PICO headset during teleop, using
XRoboToolkit's **Remote Vision** video path. This is a **separate channel** from the
`--input-source xrt` body-tracking, so it runs alongside a normal teleop session.

The installed `roboticsservice` PC-service is tracking/control only and is **not** the
camera path.

## Camera

The G1 head camera is an **Intel RealSense D430i** (USB PID `0x0b4b`) on the robot's
onboard PC (a Jetson Orin Nano). It has **no RGB sensor**, so we stream the **left-IR**
node `/dev/video2` (GRAY8 640x480 @ 30) HW-encoded to H.264.

## What `main_web_ir.cpp` is

A drop-in source file for the third-party
[XR-Robotics/XRoboToolkit-Orin-Video-Sender](https://github.com/XR-Robotics/XRoboToolkit-Orin-Video-Sender)
(MIT). Ported from that repo's `main_zed_tcp.cpp`: it keeps the `OPEN_CAMERA` /
`CLOSE_CAMERA` control protocol and the H.264 TCP framing verbatim, but replaces the ZED
SDK capture with a direct v4l2 GStreamer pipeline on the RealSense IR node, and drops the
"only ZED cameras" rejection so any Remote-Vision source works.

## Build (on the robot PC)

    git clone https://github.com/XR-Robotics/XRoboToolkit-Orin-Video-Sender
    cp main_web_ir.cpp XRoboToolkit-Orin-Video-Sender/
    cd XRoboToolkit-Orin-Video-Sender
    g++ -std=c++11 -O2 -I./asio-1.30.2/include \
        $(pkg-config --cflags gstreamer-1.0 gstreamer-app-1.0 glib-2.0) \
        main_web_ir.cpp -o OrinVideoSenderIR \
        $(pkg-config --libs gstreamer-1.0 gstreamer-app-1.0 glib-2.0) -lpthread

Deps (already present on the Orin Nano's JetPack 5.1.1): GStreamer 1.16+, gstreamer-app,
glib. No ZED SDK / CUDA / OpenCV / FFmpeg needed. (The repo's stock Makefile builds the
ZED variant and links `-lsl_zed`; this direct g++ line bypasses that.)

## Run

    ./run_headcam_sender.sh                 # listens on 0.0.0.0:13579
    # or directly:
    ./OrinVideoSenderIR --listen 0.0.0.0:13579

Then in the PICO app: **Remote Vision -> camera source IP = <robot PC IP>** (port 13579
if asked). The picture appears as a 2D screen; controller **B** toggles 2D <-> stereo.

## Protocol (as implemented)

- Sender **listens** (TCP) on `:13579`.
- PICO connects and sends: `[4B BE bodyLen]` then a NetworkDataProtocol body
  `[4B LE cmdLen]["OPEN_CAMERA"][4B LE dataLen][CameraRequestData]`, where
  `CameraRequestData` = magic `0xCA 0xFE`, version `1`, seven LE int32
  (width, height, fps, bitrate, enableMvHevc, renderMode, **callback port**), then two
  1-byte-length-prefixed strings (camera type, **callback ip**).
- Sender opens a TCP connection back to that ip:port and streams frames as
  `[4B BE len][H.264 NAL]`.
- On every `OPEN_CAMERA` the sender also replies on the control connection with
  `[4B BE bodyLen][4B LE cmdLen]["OPEN_CAMERA_ACK"][4B LE dataLen][JSON]` (schema
  `g1_wuji_audio_ports_v2`, audio ports 0, `video_projection: "flat"`,
  `video_stereo_layout: "mono"`). **2026 app versions do not render without this
  ACK.** The client also re-fires `OPEN_CAMERA` (ignored while the stream is live)
  and may open a second video connection (mirrored, not restarted).
- The 640x480 IR is `nvvidconv`-scaled to the width/height the headset requested.

## Notes / TODO

- Observed live: PICO requested 2160x810 @ 60, ~20 Mbps, type "VR". Letterboxing to the
  requested canvas is now built in (`--fit`), and the sender has since grown
  `--second-device` compositing, `--zmq-pub` recording frames, `--flip`, `--max-bitrate`,
  and the ACK/duplicate handling above — the CLI flags at the top of `main_web_ir.cpp`
  and `RUNBOOK.md` are the current reference.
- Networking: if the PICO is on the wired 192.168.123.0/24 LAN (same subnet as the robot
  PC) it connects directly. If it is on mjolnir's 10.42.0.x hotspot, add on the robot PC
  `sudo ip route add 10.42.0.0/24 via 192.168.123.222` and `FORWARD` ACCEPT on mjolnir.

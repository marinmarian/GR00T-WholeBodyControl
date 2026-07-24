#!/usr/bin/env python3
"""Mac camera -> PICO "Remote Vision" sender (second camera feed).

Python port of the XRoboToolkit video-sender protocol (see
tools/headcam_pico/main_web_ir.cpp): listens for the PICO's Remote Vision
control connection, parses OPEN_CAMERA (which carries the headset's callback
ip:port and the requested canvas/bitrate), then captures a Mac camera via
ffmpeg/avfoundation, HW-encodes H.264 with VideoToolbox, and streams
[4-byte BE length][H.264 access unit] packets back to the headset.

Camera-agnostic: works with any camera macOS sees (UVC). Currently used with
the Intel F450/F455 RealSense (RGB shows up as a plain UVC webcam).

Usage:
    python3 mac_cam_sender.py                      # auto-pick external camera
    python3 mac_cam_sender.py --device "Intel F450" --capture 1280x704@15
    python3 mac_cam_sender.py --list               # show cameras

On the PICO: Remote Vision -> camera source IP = this Mac's IP.
(One Remote Vision stream at a time: enter the robot PC's IP for the head
cam, this Mac's IP for this one.)

To move this feed to the robot later, run the C++ OrinVideoSenderIR there
instead with the camera's V4L2 node (same protocol, HW nvenc).
"""
import argparse
import re
import socket
import struct
import subprocess
import sys
import threading
import time

FFMPEG = "ffmpeg"
BUILTIN_RE = re.compile(r"MacBook|Desk View|Capture screen|iPhone", re.I)


# ── device discovery ─────────────────────────────────────────────────────────
def list_video_devices():
    out = subprocess.run(
        [FFMPEG, "-hide_banner", "-f", "avfoundation", "-list_devices", "true", "-i", ""],
        capture_output=True, text=True,
    ).stderr
    devs, in_video = [], False
    for line in out.splitlines():
        if "video devices" in line:
            in_video = True
            continue
        if "audio devices" in line:
            in_video = False
            continue
        m = re.search(r"\[(\d+)\]\s+(.*)$", line) if in_video else None
        if m:
            devs.append((int(m.group(1)), m.group(2).strip()))
    return devs


def pick_device(name):
    devs = list_video_devices()
    if not devs:
        sys.exit("no avfoundation video devices found")
    if name != "auto":
        for _, n in devs:
            if name.lower() in n.lower():
                return n
        sys.exit(f"camera '{name}' not found; available: {[n for _, n in devs]}")
    for _, n in devs:
        if not BUILTIN_RE.search(n):
            return n
    print(f"[warn] no external camera found, using '{devs[0][1]}'")
    return devs[0][1]


# ── protocol parsing (mirrors CameraRequestDeserializer in main_web_ir.cpp) ──
def _i32(b, o):
    return struct.unpack_from("<i", b, o)[0]


def _compact_str(b, o):
    ln = b[o]
    return b[o + 1:o + 1 + ln].decode("utf-8", "replace"), o + 1 + ln


def parse_open_camera(data):
    if len(data) < 10 or data[0] != 0xCA or data[1] != 0xFE or data[2] != 1:
        raise ValueError("bad magic/version")
    w, h, fps, br, hevc, render, port = struct.unpack_from("<7i", data, 3)
    cam, o = _compact_str(data, 31)
    ip, _ = _compact_str(data, o)
    return {"w": w, "h": h, "fps": fps, "bitrate": br, "ip": ip, "port": port, "camera": cam}


def parse_control(buf):
    """[4B BE bodyLen][ [4B LE cmdLen][cmd] [4B LE dataLen][data] ] -> (cmd, data) or None."""
    if len(buf) < 4:
        return None, buf
    body_len = struct.unpack_from(">I", buf, 0)[0]
    if len(buf) < 4 + body_len:
        return None, buf
    body, rest = buf[4:4 + body_len], buf[4 + body_len:]
    cmd_len = _i32(body, 0)
    cmd = body[4:4 + cmd_len].split(b"\0")[0].decode("ascii", "replace")
    data_len = _i32(body, 4 + cmd_len)
    data = body[8 + cmd_len:8 + cmd_len + data_len]
    return (cmd, data), rest


# ── streaming session ────────────────────────────────────────────────────────
class Session:
    """One OPEN_CAMERA session: ffmpeg capture -> H.264 AUs -> headset."""

    def __init__(self, device, capture, cfg):
        self.device, self.capture, self.cfg = device, capture, cfg
        self.proc = None
        self.sock = None
        self.stop_flag = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _ffmpeg_cmd(self):
        cw, ch, cfps = self.capture
        out_w = self.cfg["w"] or cw
        out_h = self.cfg["h"] or ch
        br = self.cfg["bitrate"] or 4_000_000
        return [
            FFMPEG, "-hide_banner", "-loglevel", "warning",
            "-f", "avfoundation", "-framerate", str(cfps),
            "-video_size", f"{cw}x{ch}", "-pixel_format", "uyvy422",
            "-i", self.device,
            "-vf", f"scale={out_w}:{out_h}",
            "-c:v", "h264_videotoolbox", "-realtime", "true",
            "-b:v", str(br), "-g", "15",
            "-bsf:v", "h264_metadata=aud=insert",
            "-f", "h264", "-",
        ]

    def _run(self):
        try:
            self.sock = socket.create_connection((self.cfg["ip"], self.cfg["port"]), timeout=5)
            print(f"[session] video -> {self.cfg['ip']}:{self.cfg['port']}", flush=True)
        except OSError as e:
            print(f"[session] cannot reach headset video port: {e}", flush=True)
            return
        cmd = self._ffmpeg_cmd()
        print("[session]", " ".join(cmd), flush=True)
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

        aud3, aud4 = b"\x00\x00\x01\x09", b"\x00\x00\x00\x01\x09"
        buf = b""
        sent = 0
        t0 = time.monotonic()
        try:
            while not self.stop_flag.is_set():
                chunk = self.proc.stdout.read(65536)
                if not chunk:
                    break
                buf += chunk
                # split on Access Unit Delimiters (one packet per frame)
                while True:
                    i4 = buf.find(aud4, 1)
                    # 3-byte start-code AUD — but skip matches that are really the
                    # tail of a 4-byte start code (preceding byte is 0x00)
                    i3 = buf.find(aud3, 1)
                    while i3 != -1 and buf[i3 - 1] == 0:
                        i3 = buf.find(aud3, i3 + 1)
                    cut = min(x for x in (i4, i3) if x != -1) if (i4 != -1 or i3 != -1) else -1
                    if cut <= 0:
                        break
                    au, buf = buf[:cut], buf[cut:]
                    self.sock.sendall(struct.pack(">I", len(au)) + au)
                    sent += len(au) + 4
                    if time.monotonic() - t0 > 2:
                        print(f"[session] streaming {sent * 8 / (time.monotonic() - t0) / 1e6:.1f} Mbps",
                              flush=True)
                        sent, t0 = 0, time.monotonic()
        except OSError as e:
            print(f"[session] video connection lost: {e}", flush=True)
        finally:
            self.close()

    def close(self):
        self.stop_flag.set()
        if self.proc and self.proc.poll() is None:
            self.proc.kill()
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        print("[session] closed", flush=True)


# ── control server ───────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--listen", default="0.0.0.0:13579")
    ap.add_argument("--device", default="auto", help="camera name substring, or 'auto'")
    ap.add_argument("--capture", default="1280x704@15",
                    help="capture mode WxH@fps (F450: 1280x704@15 or 640x352@30)")
    ap.add_argument("--list", action="store_true", help="list cameras and exit")
    args = ap.parse_args()

    if args.list:
        for i, n in list_video_devices():
            print(f"[{i}] {n}")
        return

    device = pick_device(args.device)
    m = re.match(r"(\d+)x(\d+)@(\d+)", args.capture)
    capture = (int(m.group(1)), int(m.group(2)), int(m.group(3)))
    print(f"[sender] camera '{device}', capture {capture[0]}x{capture[1]}@{capture[2]}", flush=True)

    host, port = args.listen.rsplit(":", 1)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, int(port)))
    srv.listen(1)
    print(f"[sender] Remote Vision protocol on {args.listen} "
          f"(PICO: camera source IP = this Mac)", flush=True)

    session = None
    while True:
        conn, addr = srv.accept()
        print(f"[sender] control connection from {addr[0]}", flush=True)
        buf = b""
        try:
            while True:
                d = conn.recv(4096)
                if not d:
                    break
                buf += d
                while True:
                    msg, buf = parse_control(buf)
                    if msg is None:
                        break
                    cmd, data = msg
                    print(f"[sender] command: {cmd}", flush=True)
                    if cmd == "OPEN_CAMERA":
                        cfg = parse_open_camera(data)
                        print(f"[sender] {cfg}", flush=True)
                        if session:
                            session.close()
                        session = Session(device, capture, cfg)
                    elif cmd == "CLOSE_CAMERA" and session:
                        session.close()
                        session = None
        except OSError:
            pass
        finally:
            conn.close()
            if session:
                session.close()
                session = None
            print("[sender] control disconnected, waiting for new connection", flush=True)


if __name__ == "__main__":
    main()

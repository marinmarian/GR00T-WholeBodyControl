#!/usr/bin/env python3
"""Keyboard + PICO controller -> /Gr00tKeyboardListener publisher for recording.

The data exporter takes its episode keys (c = start/stop+save, x = discard,
g/v/b = rate) from the ROS keyboard topic. In the decoupled-WBC stack the
control loop republishes its own keys there; the SONIC stack has no such
process, so this bridge provides one:

  * type the keys in this pane (needs a TTY), and/or
  * PICO gestures - the SONIC streamer maps right-stick click (tap = c,
    hold 1.5 s = x) and A+Y (held 0.25 s = c) to UDP datagrams on
    127.0.0.1:5559, which this process republishes to the topic.
"""
import socket
import threading

import rclpy

from decoupled_wbc.control.utils.keyboard_dispatcher import (
    KeyboardDispatcher,
    KeyboardListenerPublisher,
)

UDP_HOST = "127.0.0.1"
UDP_PORT = 5559
VALID_KEYS = set("cxgvb")


def udp_listener(pub):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((UDP_HOST, UDP_PORT))
    print(f"[record-keys] controller bridge on udp://{UDP_HOST}:{UDP_PORT}")
    while True:
        data, _ = sock.recvfrom(16)
        key = data.decode(errors="replace").strip()
        if key in VALID_KEYS:
            print(f"[record-keys] controller -> '{key}'")
            pub.handle_keyboard_button(key)
        else:
            print(f"[record-keys] ignored UDP payload {data!r}")


def main():
    rclpy.init()
    # KeyboardListenerPublisher expects a node on the global executor;
    # this process has none, so create one first.
    rclpy.get_global_executor().add_node(rclpy.create_node("record_keys_bridge"))
    pub = KeyboardListenerPublisher()

    threading.Thread(target=udp_listener, args=(pub,), daemon=True).start()

    print("[record-keys] type here:  c = start / stop+save   x = discard   g/v/b = rate")
    disp = KeyboardDispatcher()
    disp.register(pub)
    disp.start_listening()
    # start_listening returns if stdin listening fails (no TTY) or stops -
    # stay alive so the controller UDP path keeps working.
    print("[record-keys] keyboard listener exited; controller bridge still active")
    threading.Event().wait()


if __name__ == "__main__":
    main()

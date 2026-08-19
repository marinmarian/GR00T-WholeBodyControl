#!/usr/bin/env python3
"""Keyboard -> /Gr00tKeyboardListener publisher for recording control.

The data exporter takes its episode keys (c = start/stop+save, x = discard,
g/v/b = rate) from the ROS keyboard topic. In the decoupled-WBC stack the
control loop republishes its own keys there; the SONIC stack has no such
process, so this tiny bridge provides one. Run it in a real terminal/tmux
pane (needs a TTY) inside the wbc container and type the keys here.
"""
import rclpy

from decoupled_wbc.control.utils.keyboard_dispatcher import (
    KeyboardDispatcher,
    KeyboardListenerPublisher,
)

def main():
    rclpy.init()
    pub = KeyboardListenerPublisher()
    disp = KeyboardDispatcher()
    disp.register(pub)
    print("[record-keys] type here:  c = start / stop+save   x = discard   g/v/b = rate")
    disp.start_listening()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Keyboard -> ZMQ PUB bridge for gear_sonic/scripts/run_data_exporter.py.

The ZMQ exporter has no ROS 2 dependency: it reads episode keys from a ZMQ PUB
socket on port 5580 (ZMQKeyboardSubscriber connects to it) and from the
`toggle_data_collection` / `toggle_data_abort` flags in the streamer's
`manager_state` topic. The controller path works on its own -- left grip + A
starts/stops, left grip + B discards -- so this bridge exists only so the keys
can also be typed in a tmux pane.

  c = start / stop+save     x = discard

Note: unlike the ROS 2 exporter, run_data_exporter.py has no episode ratings, so
g/v/b do nothing here and are rejected rather than silently dropped.
"""
import zmq
from sshkeyboard import listen_keyboard

PORT = 5580
VALID_KEYS = {"c", "x"}
IGNORED_KEYS = {"g", "v", "b"}


def main():
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    # The subscriber connects, so this side binds.
    sock.bind(f"tcp://*:{PORT}")
    print(f"[record-keys-zmq] publishing on tcp://*:{PORT}")
    print("[record-keys-zmq] type here:  c = start / stop+save   x = discard")
    print("[record-keys-zmq] controller: left grip + A = c, left grip + B = x")

    def on_press(key):
        if key in VALID_KEYS:
            sock.send_string(key)
            print(f"[record-keys-zmq] -> '{key}'")
        elif key in IGNORED_KEYS:
            print(f"[record-keys-zmq] '{key}' ignored: this exporter has no ratings")

    try:
        listen_keyboard(on_press=on_press)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        ctx.term()


if __name__ == "__main__":
    main()

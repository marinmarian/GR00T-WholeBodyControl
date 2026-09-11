"""Keyboard publisher for run_vla_inference.py (same protocol as launch_inference.py's inline one).
k = START C++ control loop (always), x = STOP it, i = initial pose, p = run/pause policy, [ / ] = toggle initial-pose hands,
t <text> = change prompt.  Publishes on tcp://localhost:5580 (DEFAULT_ZMQ_KEYBOARD_PORT)."""
import time, zmq
from gear_sonic.utils.data_collection.keyboard_subscriber import DEFAULT_ZMQ_KEYBOARD_PORT
ctx = zmq.Context(); pub = ctx.socket(zmq.PUB); pub.bind(f"tcp://localhost:{DEFAULT_ZMQ_KEYBOARD_PORT}"); time.sleep(0.5)
print(f"Keyboard publisher ready on :{DEFAULT_ZMQ_KEYBOARD_PORT}.  k=START C++ loop  x=STOP C++ loop  i=init pose  p=run/pause policy  [ ]=init-pose hands  t <text>=prompt", flush=True)
while True:
    try:
        key = input()
    except EOFError:
        break
    if key.startswith("t "):
        pub.send_string("prompt:" + key[2:]); print("Sent prompt:", key[2:], flush=True)
    elif key:
        pub.send_string(key); print("Sent:", key, flush=True)

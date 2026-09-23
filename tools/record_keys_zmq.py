#!/usr/bin/env python3
"""Keyboard -> ZMQ PUB bridge for gear_sonic/scripts/run_data_exporter.py.

The ZMQ exporter has no ROS 2 dependency: it reads episode keys from a ZMQ PUB
socket on port 5580 (ZMQKeyboardSubscriber connects to it) and from the
`toggle_data_collection` / `toggle_data_abort` flags in the streamer's
`manager_state` topic. The controller path works on its own -- left grip + A
starts/stops, left grip + B discards -- so this bridge exists so the keys can
also be typed in a tmux pane, and so the prompt can be set per episode
(g1-vr-teleop #40):

  c = start / stop+save     x = discard
  1-9 = prompt for the NEXT episode = the tic-tac-toe cell (reading order,
        template --prompt-template, default from gear_sonic/utils/tictactoe/cells.py)
  0   = back to --base-prompt (the exporter's --task-prompt), if given

The exporter applies a prompt at once while idle and queues it while an episode is
open; its pane prints `Started recording N: "<prompt>"`, which is the only proof of
which prompt an episode carries. This pane echoing a key means nothing on its own.

--line-mode reads whole lines instead of single keys (for free text): `c`, `x`,
`1`..`9`, `0`, or `t <text>` -> prompt:<text>.

--passthrough-keys KEYS forwards each listed single key unchanged, for the other
subscribers of the same channel in the DAgger stack (g1-vr-teleop #25):
run_vla_inference.py (p = run/pause, i = initial pose, [ ] = init-pose hands) and the
PICO streamer (g = POLICY mode on/off, h = intervention on/off). `t <text>` then changes
the prompt of the exporter AND the VLA client at once.

Note: unlike the ROS 2 exporter, run_data_exporter.py has no episode ratings, so
g/v/b do nothing here and are rejected rather than silently dropped.
"""
import argparse
import os
import sys

import zmq

# Invoked by absolute path from the container (python /workspace/wbc/tools/record_keys_zmq.py),
# so the repo root is not on sys.path.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from gear_sonic.utils.tictactoe.cells import (  # noqa: E402
    DEFAULT_PROMPT_TEMPLATE, cell_prompt, cell_table, prompt_message,
)

PORT = 5580
VALID_KEYS = {"c", "x"}
IGNORED_KEYS = {"g", "v", "b"}
TAG = "[record-keys-zmq]"


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--prompt-template", default=DEFAULT_PROMPT_TEMPLATE,
                    help="prompt for digits 1-9, must contain {cell}")
    ap.add_argument("--base-prompt", default=None,
                    help="prompt restored by 0 (pass the exporter's --task-prompt)")
    ap.add_argument("--line-mode", action="store_true",
                    help="read lines with input() instead of single keys; adds `t <text>`")
    ap.add_argument("--passthrough-keys", default="",
                    help="single keys forwarded unchanged for other subscribers, e.g. 'pigh[]' "
                         "(VLA client p/i/[/], streamer g/h) in the DAgger stack")
    return ap.parse_args()


def message_for(key: str, args) -> str | None:
    """Wire message for one key (single-key mode), None if the key does nothing."""
    if key in VALID_KEYS:
        return key
    if key and key in getattr(args, "passthrough_keys", ""):
        return key
    if key.isdigit() and key != "0":
        return prompt_message(cell_prompt(int(key) - 1, args.prompt_template))
    if key == "0" and args.base_prompt:
        return prompt_message(args.base_prompt)
    return None


def message_for_line(line: str, args) -> str | None:
    line = line.strip()
    if len(line) == 1:
        return message_for(line, args)
    if line.startswith("t "):
        text = line[2:].strip()
        return prompt_message(text) if text else None
    return None


def main():
    args = parse_args()
    cell_prompt(0, args.prompt_template)  # validate the template early

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    # The subscriber connects, so this side binds.
    sock.bind(f"tcp://*:{args.port}")
    print(f"{TAG} publishing on tcp://*:{args.port}")
    print(f"{TAG} type here:  c = start / stop+save   x = discard")
    print(f"{TAG} controller: left grip + A = c, left grip + B = x")
    print(f"{TAG} prompt for the NEXT episode:")
    print(cell_table(args.prompt_template))
    if args.base_prompt:
        print(f'  0 = {args.base_prompt}')
    if args.passthrough_keys:
        print(f"{TAG} forwarded unchanged: {' '.join(args.passthrough_keys)}  "
              "(VLA client: p=run/pause i=init pose [ ]=init hands | streamer: g=POLICY mode h=intervention)")
    if args.line_mode:
        print(f"{TAG} line mode: also `t <text>` (Enter after each command)")

    def send(msg: str):
        sock.send_string(msg)
        print(f"{TAG} -> '{msg}'")

    def on_press(key):
        if key in IGNORED_KEYS and key not in args.passthrough_keys:
            print(f"{TAG} '{key}' ignored: this exporter has no ratings")
            return
        msg = message_for(key, args)
        if msg is not None:
            send(msg)

    try:
        if args.line_mode:
            while True:
                try:
                    line = input()
                except EOFError:
                    break
                msg = message_for_line(line, args)
                if msg is not None:
                    send(msg)
                elif line.strip():
                    print(f"{TAG} '{line.strip()}' not understood: c, x, 1-9, 0, t <text>")
        else:
            from sshkeyboard import listen_keyboard
            listen_keyboard(on_press=on_press)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        ctx.term()


if __name__ == "__main__":
    main()

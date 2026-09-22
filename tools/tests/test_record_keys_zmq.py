"""Key -> wire-message mapping of tools/record_keys_zmq.py (g1-vr-teleop #40). No sockets.

    cd ~/GR00T-WholeBodyControl && .venv_inference/bin/python tools/tests/test_record_keys_zmq.py
"""
import importlib.util
import os
import types

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("record_keys_zmq", os.path.join(HERE, "..", "record_keys_zmq.py"))
rk = importlib.util.module_from_spec(spec); spec.loader.exec_module(rk)

args = types.SimpleNamespace(prompt_template="put a white piece in the {cell} cell", base_prompt="demo")
assert rk.message_for("c", args) == "c"
assert rk.message_for("x", args) == "x"
assert rk.message_for("1", args) == "prompt:put a white piece in the top left cell"
assert rk.message_for("5", args) == "prompt:put a white piece in the center cell"
assert rk.message_for("9", args) == "prompt:put a white piece in the bottom right cell"
assert rk.message_for("0", args) == "prompt:demo"
assert rk.message_for("0", types.SimpleNamespace(prompt_template=args.prompt_template, base_prompt=None)) is None
for k in ("g", "v", "b", "q", " ", "t"):
    assert rk.message_for(k, args) is None, k
# line mode
assert rk.message_for_line(" 7 \n", args) == "prompt:put a white piece in the bottom left cell"
assert rk.message_for_line("c", args) == "c"
assert rk.message_for_line("t only cola", args) == "prompt:only cola"
assert rk.message_for_line("t   ", args) is None
assert rk.message_for_line("hello", args) is None
print("RECORD_KEYS_ZMQ_TEST_OK")

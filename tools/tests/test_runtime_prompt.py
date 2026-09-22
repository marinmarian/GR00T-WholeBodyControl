"""Unit test for the exporter's per-episode prompt (g1-vr-teleop #40). No sockets.

    cd ~/GR00T-WholeBodyControl && .venv_inference/bin/python tools/tests/test_runtime_prompt.py
"""
from gear_sonic.utils.data_collection.runtime_prompt import RuntimePrompt
from gear_sonic.utils.tictactoe.cells import (
    CELL_NAMES, DEFAULT_PROMPT_TEMPLATE, cell_prompt, prompt_message,
)

p = RuntimePrompt("demo")
assert p.active == "demo" and p.pending is None
# not ours: c / x / anything else -> None, state untouched
assert p.on_message("c", recording=False) is None
assert p.on_message(None, recording=False) is None
assert p.active == "demo"
# IDLE: applies immediately
note = p.on_message("prompt:put a white piece in the center cell", recording=False)
assert note and "active" in note
assert p.active == "put a white piece in the center cell" and p.pending is None
# empty ignored
assert "ignored" in p.on_message("prompt:   ", recording=False)
assert p.active == "put a white piece in the center cell"
# RECORDING: deferred, active unchanged until the episode closes
note = p.on_message("prompt:put a white piece in the top left cell", recording=True)
assert "NEXT" in note
assert p.active == "put a white piece in the center cell"
assert p.pending == "put a white piece in the top left cell"
# a second change while still recording replaces the queued one
p.on_message("prompt:put a white piece in the top right cell", recording=True)
assert p.pending == "put a white piece in the top right cell"
# episode closed -> promoted, once
assert "active" in p.on_episode_closed()
assert p.active == "put a white piece in the top right cell" and p.pending is None
assert p.on_episode_closed() is None
# whitespace trimmed
p.on_message("prompt:  spaced out  ", recording=False)
assert p.active == "spaced out"

# cells
assert len(CELL_NAMES) == 9
assert cell_prompt(0) == "put a white piece in the top left cell"
assert cell_prompt(4) == "put a white piece in the center cell"
assert cell_prompt(8) == "put a white piece in the bottom right cell"
assert cell_prompt(1, "place {cell}") == "place top center"
assert prompt_message(cell_prompt(4)) == "prompt:put a white piece in the center cell"
assert RuntimePrompt.parse(prompt_message(cell_prompt(4))) == cell_prompt(4)
for bad in (-1, 9):
    try:
        cell_prompt(bad); raise AssertionError("expected ValueError")
    except ValueError:
        pass
try:
    cell_prompt(0, "no placeholder"); raise AssertionError("expected ValueError")
except ValueError:
    pass
assert "{cell}" in DEFAULT_PROMPT_TEMPLATE
print("RUNTIME_PROMPT_TEST_OK")

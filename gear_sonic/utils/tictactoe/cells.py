"""Tic-tac-toe cell names and the per-cell language prompt (g1-vr-teleop #40, #43).

The VLA learns one skill, "put a piece of my colour in cell X", conditioned on the
prompt. The same template must be used for recording (run_data_exporter.py, via
record_keys_zmq.py digits 1-9) and inference (tictactoe_orchestrator.py), byte for
byte: GR00T lowercases the prompt and strips punctuation, nothing else.

Cell numbering is reading order, 1-9 on the keyboard = board index 0-8:

    1 top left      2 top center     3 top right
    4 middle left   5 center         6 middle right
    7 bottom left   8 bottom center  9 bottom right
"""

CELL_NAMES: tuple[str, ...] = (
    "top left", "top center", "top right",
    "middle left", "center", "middle right",
    "bottom left", "bottom center", "bottom right",
)

DEFAULT_PROMPT_TEMPLATE = "put a white piece in the {cell} cell"

# Wire format on the ZMQ keys channel (port 5580), shared with run_vla_inference.py.
PROMPT_MSG_PREFIX = "prompt:"
PAUSE_MSG = "pause"
RESUME_MSG = "resume"


def cell_prompt(cell_index: int, template: str = DEFAULT_PROMPT_TEMPLATE) -> str:
    """Prompt for board index 0-8 (keyboard digit minus one)."""
    if not 0 <= cell_index < len(CELL_NAMES):
        raise ValueError(f"cell index must be 0..8, got {cell_index}")
    if "{cell}" not in template:
        raise ValueError(f"prompt template must contain '{{cell}}': {template!r}")
    return template.format(cell=CELL_NAMES[cell_index])


def prompt_message(text: str) -> str:
    """The keys-channel message that sets `text` as the prompt."""
    return f"{PROMPT_MSG_PREFIX}{text}"


def cell_table(template: str = DEFAULT_PROMPT_TEMPLATE) -> str:
    """Human-readable digit -> prompt table for the KEYS pane."""
    return "\n".join(f"  {i + 1} = {cell_prompt(i, template)}" for i in range(len(CELL_NAMES)))

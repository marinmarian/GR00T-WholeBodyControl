"""Non-blocking exporter CLI prompts.

Tmux panes must not sit on input(). Prompt only when stdin is a TTY and
``--task-prompt`` / ``--dataset-name`` were left default/empty.
"""

from datetime import datetime
from typing import Any, Callable, Sequence

_TASK_FLAGS = ("--task-prompt", "--task_prompt")
_DATASET_FLAGS = ("--dataset-name", "--dataset_name")


def _argv_has_flag(argv: Sequence[str], flags: tuple[str, ...]) -> bool:
    for arg in argv:
        for flag in flags:
            if arg == flag or arg.startswith(f"{flag}="):
                return True
    return False


def _fill_noninteractive_defaults(config: Any) -> None:
    robot_id = getattr(config, "robot_id", None) or "sim"
    config.robot_id = robot_id
    if not getattr(config, "dataset_name", None):
        stamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        config.dataset_name = f"{stamp}-G1-{robot_id}"
    if not getattr(config, "teleoperator_username", None):
        config.teleoperator_username = "NEW_USER"
    if not getattr(config, "support_operator_username", None):
        config.support_operator_username = "NEW_USER"


def apply_exporter_prompts(
    config: Any,
    *,
    stdin_isatty: bool,
    argv: Sequence[str],
    input_fn: Callable[[str], str] = input,
) -> None:
    """Skip input() unless this is an interactive TTY with empty CLI values."""
    flags_set = _argv_has_flag(argv, _TASK_FLAGS) or _argv_has_flag(
        argv, _DATASET_FLAGS
    )
    has_dataset = bool(getattr(config, "dataset_name", None))
    if (not stdin_isatty) or flags_set or has_dataset:
        _fill_noninteractive_defaults(config)
        return

    config.task_prompt = input_fn("Enter the task prompt: ").strip().lower()
    add_to_existing = input_fn("Add to existing dataset? (y/n): ").strip().lower()
    if add_to_existing == "y":
        config.dataset_name = input_fn("Enter the dataset name: ").strip().lower()
        return
    config.robot_id = getattr(config, "robot_id", None) or "sim"
    stamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    config.dataset_name = f"{stamp}-G1-{config.robot_id}"
    config.teleoperator_username = "NEW_USER"
    config.support_operator_username = "NEW_USER"

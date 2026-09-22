"""Per-episode language prompt for the ZMQ data exporter (g1-vr-teleop #40).

`run_data_exporter.py` takes one `--task-prompt` per process. For a task with several
prompts (tic-tac-toe: one per cell) the operator sets the prompt for the NEXT episode
from the KEYS pane instead of restarting the stack. Socket-free, like SensorLossGuard,
so it can be unit-tested: the exporter feeds it every keys-channel message and reads
`.active` back into `Gr00tDataExporter.task`, which stamps every frame
(gear_sonic/data/exporter.py add_frame).

Rules:
  * `prompt:<text>` while IDLE          -> active immediately.
  * `prompt:<text>` while an episode is open (RECORDING / NEED_TO_SAVE) -> queued and
    applied when that episode is closed (saved or discarded), so no episode ever carries
    two task strings.
  * empty text is ignored; any other message is not ours (returns None).
"""

PROMPT_MSG_PREFIX = "prompt:"


class RuntimePrompt:
    def __init__(self, initial: str):
        self.active: str = initial
        self.pending: str | None = None

    @staticmethod
    def parse(msg) -> str | None:
        """The prompt text if `msg` is a prompt message, else None."""
        if not isinstance(msg, str) or not msg.startswith(PROMPT_MSG_PREFIX):
            return None
        return msg[len(PROMPT_MSG_PREFIX):].strip()

    def on_message(self, msg, recording: bool) -> str | None:
        """Handle one keys-channel message. Returns a log line, or None if not a prompt."""
        text = self.parse(msg)
        if text is None:
            return None
        if not text:
            return "[Prompt] empty prompt ignored"
        if recording:
            self.pending = text
            return (
                f'[Prompt] queued for the NEXT episode: "{text}" '
                f'(the open episode keeps "{self.active}")'
            )
        self.active = text
        self.pending = None
        return f'[Prompt] active: "{text}"'

    def on_episode_closed(self) -> str | None:
        """Promote a queued prompt once the episode is saved or discarded."""
        if self.pending is None:
            return None
        self.active = self.pending
        self.pending = None
        return f'[Prompt] active: "{self.active}"'

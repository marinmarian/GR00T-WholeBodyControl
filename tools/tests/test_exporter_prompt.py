"""run_data_exporter.py applies `prompt:<text>` per episode (g1-vr-teleop #40).

Drives GrootDataCollector._check_recording_commands / _finalize_frame with a fake keys
channel and a fake Gr00tDataExporter; no ZMQ, no camera, no robot. Needs the exporter's
imports (lerobot etc.), i.e. the wbc-marin container venv:

    ~/wbc-marin-exec.sh python tools/tests/test_exporter_prompt.py
"""
import time
from types import SimpleNamespace

from gear_sonic.scripts.run_data_exporter import GrootDataCollector
from gear_sonic.utils.data_collection.episode_state import EpisodeState
from gear_sonic.utils.data_collection.runtime_prompt import RuntimePrompt


class FakeKeys:
    def __init__(self):
        self.queue = []

    def read_msg(self):
        return self.queue.pop(0) if self.queue else None


class FakeExporter:
    """Just enough of Gr00tDataExporter: .task is stamped on frames, saves bump the index."""

    def __init__(self, task):
        self.task = task
        self.episode_buffer = {"episode_index": 0, "size": 0}
        self.saved = []      # (episode_index, task at save time)
        self.discarded = []

    def save_episode(self):
        self.saved.append((self.episode_buffer["episode_index"], self.task))
        self.episode_buffer = {"episode_index": self.episode_buffer["episode_index"] + 1, "size": 0}

    def save_episode_as_discarded(self):
        self.discarded.append((self.episode_buffer["episode_index"], self.task))
        self.episode_buffer = {"episode_index": self.episode_buffer["episode_index"] + 1, "size": 0}


c = GrootDataCollector.__new__(GrootDataCollector)   # skip __init__ (sockets, robot model)
c.text_to_speech = None
c.frequency = 50
c.upload_bucket_path = None
c._upload_threads = []
c._initial_yaw = None
c._manager_toggle_dc = False
c._manager_toggle_da = False
c._episode_state = EpisodeState()
c._keyboard_listener = FakeKeys()
c.data_exporter = FakeExporter("demo")
c._runtime_prompt = RuntimePrompt(c.data_exporter.task)
c.sonic_timing_monitor = SimpleNamespace(reset=lambda: None)

A, B, C_ = ("put a white piece in the top left cell", "put a white piece in the center cell",
            "put a white piece in the bottom right cell")
keys, ex, st = c._keyboard_listener, c.data_exporter, c._episode_state


def tick(*msgs):
    for m in msgs:
        keys.queue.append(m)
        c._check_recording_commands()


# idle: prompt applies immediately
tick(f"prompt:{A}")
assert ex.task == A and st.get_state() == st.IDLE
# start episode 0 with A; prompt B arrives mid-episode -> queued, frames keep A
tick("c")
assert st.get_state() == st.RECORDING
tick(f"prompt:{B}")
assert ex.task == A and c._runtime_prompt.pending == B
# stop + save -> episode 0 saved with A, B becomes active
tick("c")
assert st.get_state() == st.NEED_TO_SAVE and ex.task == A
ex.episode_buffer["size"] = 100
assert c._finalize_frame(time.monotonic()) is True
assert st.get_state() == st.IDLE
assert ex.saved == [(0, A)]
assert ex.task == B and c._runtime_prompt.pending is None
# discard path: episode 1 with B and some frames, prompt C queued, x -> discarded with B, C active
tick("c", f"prompt:{C_}")
assert ex.task == B
ex.episode_buffer["size"] = 5
tick("x")
assert st.get_state() == st.IDLE and ex.discarded == [(1, B)] and ex.task == C_
# PICO start gesture in the same tick as a prompt message: the prompt is read first
# (still idle), so the episode that starts carries it -- digit-then-grip+A is what the
# operator means even when the two land in one 20 ms tick.
keys.queue.append(f"prompt:{A}")
c._manager_toggle_dc = True
c._check_recording_commands()
assert st.get_state() == st.RECORDING and c._manager_toggle_dc is False
assert ex.task == A and c._runtime_prompt.pending is None
tick("c"); ex.episode_buffer["size"] = 3; c._finalize_frame(time.monotonic())
assert ex.saved[-1] == (2, A) and ex.task == A
# empty save (no frames) still promotes the queued prompt
tick("c", f"prompt:{B}", "c")
assert ex.task == A and c._runtime_prompt.pending == B
assert c._finalize_frame(time.monotonic()) is True
assert ex.saved[-1] == (2, A) and ex.task == B and st.get_state() == st.IDLE
# c / x / junk never touch the prompt
tick("zzz")
assert ex.task == B


# --- 2026-09-23 crash: discard with an empty buffer, and loop-error recovery -----------------
class StrictExporter(FakeExporter):
    """save_episode_as_discarded raises on an empty buffer, like lerobot's validate_episode_buffer."""

    def __init__(self, task):
        super().__init__(task)
        self.resets = 0

    def save_episode_as_discarded(self):
        if self.episode_buffer["size"] == 0:
            raise ValueError("You must add one or several frames with `add_frame` before calling `add_episode`.")
        super().save_episode_as_discarded()

    def skip_and_start_new_episode(self):
        self.resets += 1
        self.episode_buffer = {"episode_index": self.episode_buffer["episode_index"], "size": 0}


ex = c.data_exporter = StrictExporter(B)
c._runtime_prompt = RuntimePrompt(B)
idx0 = ex.episode_buffer["episode_index"]
# x while recording with no frames: back to IDLE, nothing saved, no exception, queued prompt promoted
tick("c", f"prompt:{A}")
assert st.get_state() == st.RECORDING and ex.episode_buffer["size"] == 0
tick("x")
assert st.get_state() == st.IDLE and ex.discarded == [] and ex.saved == []
assert ex.episode_buffer["episode_index"] == idx0 and ex.task == A
# x with frames still discards
tick("c"); ex.episode_buffer["size"] = 7; tick("x")
assert ex.discarded == [(idx0, A)] and st.get_state() == st.IDLE
# loop error while recording with frames: episode kept as discarded, IDLE, prompt promoted
tick("c", f"prompt:{B}"); ex.episode_buffer["size"] = 4
c._recover_from_loop_error(RuntimeError("boom"))
assert st.get_state() == st.IDLE and ex.discarded[-1] == (idx0 + 1, A) and ex.task == B
# loop error while recording with no frames: buffer reset, no save
tick("c"); assert ex.episode_buffer["size"] == 0
c._recover_from_loop_error(ValueError("empty"))
assert st.get_state() == st.IDLE and ex.resets == 1 and len(ex.discarded) == 2
# loop error while idle: nothing to do but stay alive
c._recover_from_loop_error(RuntimeError("idle boom"))
assert st.get_state() == st.IDLE and ex.resets == 1
# a failing discard inside the recovery is swallowed too
ex.episode_buffer["size"] = 2
ex.save_episode_as_discarded = lambda: (_ for _ in ()).throw(OSError("disk full"))
tick("c"); c._recover_from_loop_error(RuntimeError("boom2"))
assert st.get_state() == st.IDLE
print("EXPORTER_PROMPT_TEST_OK")

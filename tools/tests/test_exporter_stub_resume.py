"""Gr00tDataExporter.create() on an existing outputs/<dataset> directory (g1-vr-teleop #59).

A session torn down before its first save_episode() leaves meta/info.json + meta/modality.json
and empty video dirs behind. Resuming that stub used to send the LeRobot loader to the Hub for
"tmp/tmp_dataset" and die as "corrupted", so every default `sonic-teleop.sh up`/`down` cycle
without a recording broke the next `up`. Now: an episode-less stub is removed and recreated, a
dataset with episodes resumes, and a dataset with episodes but missing meta files raises the
"corrupted" ValueError locally (no Hub round-trip, same result offline). Needs the exporter's
imports (lerobot etc.), i.e. the wbc-marin container venv:

    ~/wbc-marin-exec.sh python tools/tests/test_exporter_stub_resume.py
"""
import json
from pathlib import Path
import shutil
import tempfile

from gear_sonic.data.exporter import Gr00tDataExporter

FPS = 10
FEATURES = {
    "observation.images.ego_view": {
        "dtype": "video",
        "shape": (16, 16, 3),
        "names": ["height", "width", "channel"],
    },
    "observation.state": {"dtype": "float64", "shape": (2,), "names": ["a", "b"]},
    "action": {"dtype": "float64", "shape": (2,), "names": ["a", "b"]},
}
MODALITY = {
    "state": {"body": {"start": 0, "end": 2}},
    "action": {"body": {"start": 0, "end": 2}},
    "video": {"ego_view": {"original_key": "observation.images.ego_view"}},
    "annotation": {},
}


def create(root):
    return Gr00tDataExporter.create(
        save_root=root, fps=FPS, features=FEATURES, modality_config=MODALITY, task="demo"
    )


def write_jsonl(path, rows):
    with open(path, "w") as f:
        for row in rows:
            f.write(json.dumps(row) + "\n")


def expect_corrupted(root, *fragments):
    try:
        create(root)
    except ValueError as e:
        msg = str(e)
        assert "corrupted" in msg, msg
        for fragment in fragments:
            assert fragment in msg, (fragment, msg)
    else:
        raise AssertionError(f"create() on {root} should have raised")


tmp = Path(tempfile.mkdtemp(prefix="stub_resume_"))
try:
    root = tmp / "g1_sonic"

    # 1. fresh dataset: exactly the stub layout a torn-down session leaves behind
    exp = create(root)
    assert exp.meta.total_episodes == 0
    assert (root / "meta" / "info.json").is_file() and (root / "meta" / "modality.json").is_file()
    assert not (root / "meta" / "episodes.jsonl").exists()
    assert Gr00tDataExporter.is_episode_less_stub(root)

    # 2. `up` again without ever having recorded: no "corrupted", stub replaced by a fresh one
    marker = root / "videos" / "chunk-000" / "leftover.txt"
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.write_text("junk from the previous session")
    exp = create(root)
    assert exp.meta.total_episodes == 0
    assert not marker.exists(), "stub must be removed, not resumed"
    assert (root / "meta" / "info.json").is_file()

    # 3. a dataset with a recorded episode resumes with its count intact
    info = json.loads((root / "meta" / "info.json").read_text())
    info.update(total_episodes=1, total_frames=5, total_tasks=1, total_videos=1, total_chunks=1)
    info["splits"] = {"train": "0:1"}
    (root / "meta" / "info.json").write_text(json.dumps(info, indent=4))
    write_jsonl(root / "meta" / "tasks.jsonl", [{"task_index": 0, "task": "demo"}])
    write_jsonl(root / "meta" / "episodes.jsonl", [{"episode_index": 0, "tasks": ["demo"], "length": 5}])
    stat = {"min": [0.0, 0.0], "max": [1.0, 1.0], "mean": [0.5, 0.5], "std": [0.1, 0.1], "count": [5]}
    write_jsonl(
        root / "meta" / "episodes_stats.jsonl",
        [{"episode_index": 0, "stats": {"observation.state": stat, "action": stat}}],
    )
    assert not Gr00tDataExporter.is_episode_less_stub(root)
    exp = create(root)
    assert exp.meta.total_episodes == 1, exp.meta.total_episodes
    assert exp.episode_buffer["episode_index"] == 1
    assert (root / "meta" / "episodes.jsonl").is_file(), "resume must not touch the dataset"

    # 4. episodes recorded but a meta file gone: real corruption, reported locally, dir kept
    (root / "meta" / "tasks.jsonl").unlink()
    assert not Gr00tDataExporter.is_episode_less_stub(root)
    expect_corrupted(root, "meta/tasks.jsonl")
    assert (root / "meta" / "episodes.jsonl").is_file()

    # 5. parquet on disk but no episode index: not a stub either
    shutil.rmtree(root)
    root.mkdir()
    (root / "meta").mkdir()
    (root / "data" / "chunk-000").mkdir(parents=True)
    (root / "data" / "chunk-000" / "episode_000000.parquet").write_bytes(b"\x00")
    assert not Gr00tDataExporter.is_episode_less_stub(root)
    expect_corrupted(root, "meta/episodes.jsonl")
    assert (root / "data" / "chunk-000" / "episode_000000.parquet").is_file()

    # 6. an empty directory (mkdir'd by hand, nothing inside) is a stub too
    shutil.rmtree(root)
    root.mkdir()
    assert Gr00tDataExporter.is_episode_less_stub(root)
    exp = create(root)
    assert exp.meta.total_episodes == 0

    # 7. a non-empty directory without meta/info.json is NOT a stub and is never deleted
    #    (think save_root pointing at outputs/ itself, or at an unrelated folder)
    shutil.rmtree(root)
    (root / "restocking" / "meta").mkdir(parents=True)
    (root / "restocking" / "meta" / "info.json").write_text("{}")
    assert not Gr00tDataExporter.is_episode_less_stub(root)
    expect_corrupted(root, "meta/info.json")
    assert (root / "restocking" / "meta" / "info.json").is_file(), "unrelated content must survive"

    # 8. info.json present but unreadable or without total_episodes: not a stub either
    shutil.rmtree(root)
    (root / "meta").mkdir(parents=True)
    (root / "meta" / "info.json").write_text("not json")
    assert not Gr00tDataExporter.is_episode_less_stub(root)
    (root / "meta" / "info.json").write_text("{}")
    assert not Gr00tDataExporter.is_episode_less_stub(root)
    expect_corrupted(root, "meta/tasks.jsonl")
    assert (root / "meta" / "info.json").is_file()

    print("test_exporter_stub_resume: OK")
finally:
    shutil.rmtree(tmp, ignore_errors=True)

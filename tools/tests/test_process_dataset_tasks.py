"""process_dataset.py keeps per-episode prompts intact (g1-vr-teleop #41).

Two mini datasets record the same prompts in a different tasks.jsonl order; after a
merge every row must still resolve to its original string. Also: a single dataset with
--exclude-episodes keeps its strings. No videos, so `av` is stubbed; needs pandas +
pyarrow (the wbc-marin container venv, or ~/Isaac-GR00T/.venv):

    cd ~/GR00T-WholeBodyControl && ~/Isaac-GR00T/.venv/bin/python tools/tests/test_process_dataset_tasks.py
"""
import json
import os
import sys
import tempfile
import types
from pathlib import Path

import numpy as np
import pandas as pd

sys.modules.setdefault("av", types.ModuleType("av"))  # only needed for video filtering
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
from gear_sonic.scripts.process_dataset import ProcessDatasetConfig, main, merge_tasks_meta  # noqa: E402

FPS = 50


def make_dataset(root: Path, tasks: list[str], episodes: list[tuple[str, int]]):
    """episodes = [(task string, length)], tasks = tasks.jsonl order."""
    (root / "meta").mkdir(parents=True)
    (root / "data" / "chunk-000").mkdir(parents=True)
    task_index = {t: i for i, t in enumerate(tasks)}
    with open(root / "meta" / "tasks.jsonl", "w") as f:
        for t in tasks:
            f.write(json.dumps({"task_index": task_index[t], "task": t}) + "\n")
    total = 0
    with open(root / "meta" / "episodes.jsonl", "w") as f:
        for ep, (t, n) in enumerate(episodes):
            df = pd.DataFrame({
                "observation.state": [np.full(3, ep, dtype=np.float64) for _ in range(n)],
                "teleop.smpl_pose": [np.ones(63, dtype=np.float32) for _ in range(n)],
                "timestamp": np.arange(n) / FPS,
                "frame_index": np.arange(n),
                "episode_index": np.full(n, ep),
                "index": np.arange(total, total + n),
                "task_index": np.full(n, task_index[t], dtype=np.int64),
            })
            df.to_parquet(root / "data" / "chunk-000" / f"episode_{ep:06d}.parquet")
            f.write(json.dumps({"episode_index": ep, "tasks": [t], "length": n}) + "\n")
            total += n
    info = {
        "codebase_version": "v2.1", "fps": FPS, "total_episodes": len(episodes),
        "total_frames": total, "total_tasks": len(tasks), "chunks_size": 1000,
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
        "features": {}, "splits": {"train": f"0:{len(episodes)}"},
        "script_config": {"control_frequency": 50.0},
    }
    json.dump(info, open(root / "meta" / "info.json", "w"))


def read_back(root: Path) -> list[tuple[int, str, int]]:
    """(episode_index, task string, length) per output episode, resolved through tasks.jsonl."""
    tasks = {}
    for line in open(root / "meta" / "tasks.jsonl"):
        row = json.loads(line); tasks[row["task_index"]] = row["task"]
    out = []
    for line in open(root / "meta" / "episodes.jsonl"):
        ep = json.loads(line)
        df = pd.read_parquet(root / "data" / "chunk-000" / f"episode_{ep['episode_index']:06d}.parquet")
        idx = set(df["task_index"].tolist())
        assert len(idx) == 1, idx
        out.append((ep["episode_index"], tasks[idx.pop()], len(df)))
    return out


TL, C, BR = "put a white piece in the top left cell", "put a white piece in the center cell", "put a white piece in the bottom right cell"

with tempfile.TemporaryDirectory() as tmp:
    tmp = Path(tmp)
    # session 1 recorded TL then C; session 2 recorded C first, then BR
    make_dataset(tmp / "s1", [TL, C], [(TL, 10), (C, 12), (TL, 11)])
    make_dataset(tmp / "s2", [C, BR], [(C, 9), (BR, 13)])

    tasks_meta, remaps = merge_tasks_meta([tmp / "s1", tmp / "s2"])
    assert [t["task"] for t in tasks_meta] == [TL, C, BR]
    assert [t["task_index"] for t in tasks_meta] == [0, 1, 2]
    assert remaps[tmp / "s1"] == {0: 0, 1: 1}
    assert remaps[tmp / "s2"] == {0: 1, 1: 2}          # s2's index 0 is "center", not "top left"

    main(ProcessDatasetConfig(dataset_path=[str(tmp / "s1"), str(tmp / "s2")],
                              output_path=str(tmp / "merged"), remove_stale_smpl=False))
    got = read_back(tmp / "merged")
    assert got == [(0, TL, 10), (1, C, 12), (2, TL, 11), (3, C, 9), (4, BR, 13)], got
    info = json.load(open(tmp / "merged" / "meta" / "info.json"))
    assert info["total_tasks"] == 3 and info["total_episodes"] == 5 and info["total_frames"] == 55

    # single dataset, --exclude-episodes: survivors renumbered, strings intact, tasks.jsonl unchanged
    main(ProcessDatasetConfig(dataset_path=[str(tmp / "s1")], output_path=str(tmp / "s1_cut"),
                              remove_stale_smpl=False, exclude_episodes=[1]))
    assert read_back(tmp / "s1_cut") == [(0, TL, 10), (1, TL, 11)]
    assert [json.loads(l)["task"] for l in open(tmp / "s1_cut" / "meta" / "tasks.jsonl")] == [TL, C]

    # stale-SMPL removal keeps the task column too
    make_dataset(tmp / "s3", [BR], [(BR, 20)])
    p = tmp / "s3" / "data" / "chunk-000" / "episode_000000.parquet"
    df = pd.read_parquet(p)
    df["teleop.smpl_pose"] = [np.zeros(63, dtype=np.float32) if i < 5 else np.ones(63, dtype=np.float32)
                              for i in range(len(df))]
    df.to_parquet(p)
    main(ProcessDatasetConfig(dataset_path=[str(tmp / "s3")], output_path=str(tmp / "s3_clean")))
    assert read_back(tmp / "s3_clean") == [(0, BR, 15)]

    # a row pointing at a task_index missing from tasks.jsonl is refused on merge
    make_dataset(tmp / "bad", [TL], [(TL, 4)])
    p = tmp / "bad" / "data" / "chunk-000" / "episode_000000.parquet"
    df = pd.read_parquet(p); df["task_index"] = 7; df.to_parquet(p)
    try:
        main(ProcessDatasetConfig(dataset_path=[str(tmp / "s1"), str(tmp / "bad")],
                                  output_path=str(tmp / "never"), remove_stale_smpl=False))
        raise AssertionError("expected SystemExit")
    except SystemExit as e:
        assert e.code == 1

print("PROCESS_DATASET_TASKS_TEST_OK")

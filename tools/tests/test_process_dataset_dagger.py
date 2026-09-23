"""process_dataset.py on policy episodes (DAgger, g1-vr-teleop #25).

Mini datasets with a teleop.stream_mode column: policy frames (6) and interventions (5) have an
all-zero teleop.smpl_pose by construction and must survive the stale-SMPL cleaning;
--intervention-segments cuts one output episode per correction with its pre-roll. No videos
(`av` stubbed); needs pandas + pyarrow:

    cd ~/GR00T-WholeBodyControl && ~/Isaac-GR00T/.venv/bin/python tools/tests/test_process_dataset_dagger.py
"""
import json
import os
import sys
import tempfile
import types
from pathlib import Path

import numpy as np
import pandas as pd

sys.modules.setdefault("av", types.ModuleType("av"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
from gear_sonic.scripts.process_dataset import (  # noqa: E402
    ProcessDatasetConfig,
    build_stale_mask,
    intervention_windows,
    main,
)

FPS = 50


def make_dataset(root: Path, episodes: list[list[int]]):
    """episodes = per-episode list of stream modes (one per frame); smpl_pose zero unless mode 1."""
    (root / "meta").mkdir(parents=True)
    (root / "data" / "chunk-000").mkdir(parents=True)
    with open(root / "meta" / "tasks.jsonl", "w") as f:
        f.write(json.dumps({"task_index": 0, "task": "demo"}) + "\n")
    total = 0
    with open(root / "meta" / "episodes.jsonl", "w") as f:
        for ep, modes in enumerate(episodes):
            n = len(modes)
            df = pd.DataFrame({
                "observation.state": [np.array([ep, i], dtype=np.float64) for i in range(n)],
                "teleop.smpl_pose": [
                    (np.full(63, 1.0 + i, dtype=np.float32) if m == 1 else np.zeros(63, dtype=np.float32))
                    for i, m in enumerate(modes)
                ],
                "teleop.stream_mode": [np.array([m], dtype=np.int32) for m in modes],
                "timestamp": np.arange(n) / FPS,
                "frame_index": np.arange(n),
                "episode_index": np.full(n, ep),
                "index": np.arange(total, total + n),
                "task_index": np.zeros(n, dtype=np.int64),
            })
            df.to_parquet(root / "data" / "chunk-000" / f"episode_{ep:06d}.parquet")
            f.write(json.dumps({"episode_index": ep, "tasks": ["demo"], "length": n}) + "\n")
            total += n
    info = {
        "fps": FPS, "total_episodes": len(episodes), "total_frames": total, "total_tasks": 1,
        "chunks_size": 1000, "video_keys": [], "features": {},
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "script_config": {"robot": "g1"},
    }
    with open(root / "meta" / "info.json", "w") as f:
        json.dump(info, f)


def read_output(out: Path):
    eps = [json.loads(l) for l in open(out / "meta" / "episodes.jsonl") if l.strip()]
    dfs = [pd.read_parquet(out / "data" / "chunk-000" / f"episode_{e['episode_index']:06d}.parquet") for e in eps]
    return eps, dfs


def modes_of(df):
    return [int(np.asarray(x)[0]) for x in df["teleop.stream_mode"]]


# windows: pre-roll, clipping at 0, merging of overlapping windows
assert intervention_windows(np.array([6] * 10 + [5] * 5 + [6] * 10), 3) == [(7, 15)]
assert intervention_windows(np.array([5, 5, 6, 6]), 3) == [(0, 2)]
assert intervention_windows(np.array([6, 5, 6, 6, 5, 5, 6]), 2) == [(0, 6)]        # merged
assert intervention_windows(np.array([6, 5, 6, 6, 6, 6, 5, 5, 6]), 2) == [(0, 2), (4, 8)]
assert intervention_windows(np.array([6] * 5), 2) == []
# the stale mask itself is unchanged: zero rows are stale
assert build_stale_mask(np.zeros((3, 63), np.float32)).all()

with tempfile.TemporaryDirectory() as tmp:
    tmp = Path(tmp)
    # ep0: teleop POSE episode with a 2-frame SMPL dropout at the end (must still be cleaned)
    # ep1: policy episode, two interventions
    # ep2: policy episode without interventions
    # ep3: teleop POSE episode with an OFF tail (mode 0, zero SMPL -> stale, removed as before)
    ep0 = [1] * 20
    ep1 = [6] * 30 + [5] * 10 + [6] * 20 + [5] * 5 + [6] * 5
    ep2 = [6] * 25
    ep3 = [1] * 15 + [0] * 5
    make_dataset(tmp / "src", [ep0, ep1, ep2, ep3])
    # make ep0's last two SMPL rows zero (a real dropout in POSE mode)
    p0 = tmp / "src" / "data" / "chunk-000" / "episode_000000.parquet"
    d0 = pd.read_parquet(p0)
    col = list(d0["teleop.smpl_pose"])
    col[18] = np.zeros(63, np.float32)
    col[19] = np.zeros(63, np.float32)
    d0["teleop.smpl_pose"] = col
    d0.to_parquet(p0)

    # 1. default cleaning keeps every policy / intervention frame, cleans real dropouts
    main(ProcessDatasetConfig(dataset_path=[str(tmp / "src")], output_path=str(tmp / "clean")))
    eps, dfs = read_output(tmp / "clean")
    assert [e["length"] for e in eps] == [18, 70, 25, 15], [e["length"] for e in eps]
    assert modes_of(dfs[1]) == ep1
    assert modes_of(dfs[3]) == [1] * 15

    # 2. --intervention-segments: one episode per correction with 0.2 s (10 frames) pre-roll
    main(ProcessDatasetConfig(dataset_path=[str(tmp / "src")], output_path=str(tmp / "segs"),
                              intervention_segments=True, intervention_preroll_s=0.2))
    eps, dfs = read_output(tmp / "segs")
    assert [e["length"] for e in eps] == [20, 15], [e["length"] for e in eps]
    assert modes_of(dfs[0]) == [6] * 10 + [5] * 10
    assert modes_of(dfs[1]) == [6] * 10 + [5] * 5
    # frames come from the right place in the source episode (observation.state = [ep, frame])
    assert [int(np.asarray(x)[1]) for x in dfs[0]["observation.state"]] == list(range(20, 40))
    assert [int(np.asarray(x)[1]) for x in dfs[1]["observation.state"]] == list(range(50, 65))
    assert all(int(np.asarray(x)[0]) == 1 for x in dfs[0]["observation.state"])
    assert np.allclose(dfs[1]["timestamp"], np.arange(15) / FPS)
    assert list(dfs[1]["frame_index"]) == list(range(15)) and list(dfs[1]["episode_index"]) == [1] * 15
    info = json.load(open(tmp / "segs" / "meta" / "info.json"))
    assert info["total_episodes"] == 2 and info["total_frames"] == 35

    # 3. keep episodes without interventions whole (the teleop ones and the clean policy run)
    main(ProcessDatasetConfig(dataset_path=[str(tmp / "src")], output_path=str(tmp / "segs_keep"),
                              intervention_segments=True, intervention_preroll_s=0.2,
                              keep_episodes_without_interventions=True))
    eps, dfs = read_output(tmp / "segs_keep")
    assert [e["length"] for e in eps] == [18, 20, 15, 25, 15], [e["length"] for e in eps]

    # 4. an old-style dataset without the column is untouched by the new options
    make_dataset(tmp / "old", [[1] * 10])
    pold = tmp / "old" / "data" / "chunk-000" / "episode_000000.parquet"
    dold = pd.read_parquet(pold).drop(columns=["teleop.stream_mode"])
    dold.to_parquet(pold)
    main(ProcessDatasetConfig(dataset_path=[str(tmp / "old")], output_path=str(tmp / "old_out")))
    eps, dfs = read_output(tmp / "old_out")
    assert [e["length"] for e in eps] == [10]

    # 5. --intervention-segments in place is refused
    try:
        main(ProcessDatasetConfig(dataset_path=[str(tmp / "src")], intervention_segments=True))
        raise AssertionError("in-place split should be refused")
    except SystemExit as e:
        assert e.code == 1
print("PROCESS_DATASET_DAGGER_TEST_OK")

"""
Post-process a LeRobot dataset recorded by the data exporter.

Removes discarded episodes (flagged during collection) and stale SMPL frames
(all-zero teleop.smpl_pose and frozen lead-in frames that precede them) which
occur during teleop pauses or ZMQ frame drops.  Can also merge multiple
recording sessions into a single dataset.

The script operates directly on the LeRobot v2.1 on-disk format
(parquet + mp4) without any external training framework dependencies.

Usage:

    # Clean a single dataset in-place
    python gear_sonic/scripts/process_dataset.py \\
        --dataset-path outputs/my_dataset

    # Clean and write to a new directory (non-destructive)
    python gear_sonic/scripts/process_dataset.py \\
        --dataset-path outputs/my_dataset \\
        --output-path outputs/my_dataset_cleaned

    # Merge multiple datasets into one (validates matching script_config)
    python gear_sonic/scripts/process_dataset.py \\
        --dataset-path outputs/session1 outputs/session2 outputs/session3 \\
        --output-path outputs/merged_dataset

    # Merge from a text file listing dataset paths (one per line)
    python gear_sonic/scripts/process_dataset.py \\
        --dataset-list datasets.txt \\
        --output-path outputs/merged_dataset

    # Skip SMPL cleaning (merge only)
    python gear_sonic/scripts/process_dataset.py \\
        --dataset-path outputs/session1 outputs/session2 \\
        --output-path outputs/merged \\
        --no-remove-stale-smpl

    # Drop specific saved episodes (misfires, bad takes); survivors renumbered
    python gear_sonic/scripts/process_dataset.py \\
        --dataset-path outputs/my_dataset \\
        --output-path outputs/my_dataset_cleaned \\
        --exclude-episodes 0 1 2 --no-remove-stale-smpl

    # Remove discarded episodes (flagged during collection via 'x' key)
    python gear_sonic/scripts/process_dataset.py \\
        --dataset-path outputs/my_dataset \\
        --output-path outputs/my_dataset_cleaned \\
        --remove-discarded

    # DAgger (g1-vr-teleop #25): from policy episodes keep only the operator's corrections
    # (teleop.stream_mode == 5, PLANNER_VR_3PT) with 1 s of policy lead-in, one output episode
    # per correction; episodes without a correction are dropped
    python gear_sonic/scripts/process_dataset.py \\
        --dataset-path outputs/policy_session \\
        --output-path outputs/policy_session_corrections \\
        --intervention-segments --intervention-preroll-s 1.0

Frames whose stream_mode is 5 (VR_3PT) or 6 (POLICY) have an all-zero teleop.smpl_pose by
design and are never treated as stale SMPL frames (--keep-zero-smpl-modes).
"""

from dataclasses import dataclass, field
import json
from pathlib import Path
import shutil
from typing import Optional

import av
import numpy as np
import pandas as pd
import tyro


SMPL_POSE_COLUMN = "teleop.smpl_pose"
STREAM_MODE_COLUMN = "teleop.stream_mode"
POLICY_STREAM_MODE = 6         # StreamMode.POLICY: the streamer relays the VLA tokens
INTERVENTION_STREAM_MODE = 5   # StreamMode.PLANNER_VR_3PT: the operator drives the upper body


# ---------------------------------------------------------------------------
# Stale SMPL frame detection
# ---------------------------------------------------------------------------

def build_stale_mask(smpl_arr: np.ndarray) -> np.ndarray:
    """Return a boolean mask where True = frame should be removed.

    Marks all-zero rows AND any consecutive frozen (identical-to-next) rows
    that immediately precede a zero row.  Frozen runs that do NOT lead into
    a zero row are left untouched — those occur naturally when the SMPL
    stream publishes at a slightly lower rate than the collection loop.
    """
    n = len(smpl_arr)
    is_zero = np.all(smpl_arr == 0, axis=1)
    remove = is_zero.copy()

    diffs = np.zeros(n)
    diffs[1:] = np.sum(np.abs(smpl_arr[1:] - smpl_arr[:-1]), axis=1)

    for i in range(n):
        if is_zero[i]:
            j = i - 1
            while j >= 0 and diffs[j] == 0.0 and not is_zero[j]:
                remove[j] = True
                j -= 1

    return remove


def stream_modes_of(df: pd.DataFrame) -> np.ndarray | None:
    """Per-frame teleop.stream_mode as int array, or None for datasets without the column."""
    if STREAM_MODE_COLUMN not in df.columns:
        return None
    return np.asarray([int(np.asarray(x).reshape(-1)[0]) for x in df[STREAM_MODE_COLUMN]], dtype=np.int64)


def contiguous_segments(mask: np.ndarray) -> list[tuple[int, int]]:
    """[start, end) index ranges of the True runs in ``mask``."""
    m = np.asarray(mask, dtype=bool)
    if m.size == 0:
        return []
    edges = np.flatnonzero(np.diff(np.concatenate(([0], m.astype(np.int8), [0]))))
    return [(int(edges[i]), int(edges[i + 1])) for i in range(0, len(edges), 2)]


def intervention_windows(
    stream_modes: np.ndarray, preroll_frames: int, mode: int = INTERVENTION_STREAM_MODE
) -> list[tuple[int, int]]:
    """One [start, end) window per intervention: the correction plus ``preroll_frames`` of
    whatever preceded it (policy frames), clipped to the episode. Windows that would overlap
    after adding the pre-roll are merged so no frame is emitted twice."""
    segs = contiguous_segments(np.asarray(stream_modes) == mode)
    windows: list[tuple[int, int]] = []
    for start, end in segs:
        start = max(0, start - int(preroll_frames))
        if windows and start <= windows[-1][1]:
            windows[-1] = (windows[-1][0], end)
        else:
            windows.append((start, end))
    return windows


def format_mode_counts(stream_modes: np.ndarray) -> str:
    names = {0: "OFF", 1: "POSE", 2: "PLANNER", 3: "FROZEN", 4: "POSE_PAUSE", 5: "VR_3PT", 6: "POLICY"}
    modes, counts = np.unique(stream_modes, return_counts=True)
    return ", ".join(f"{names.get(int(m), int(m))}={int(c)}" for m, c in zip(modes, counts))


# ---------------------------------------------------------------------------
# LeRobot on-disk helpers
# ---------------------------------------------------------------------------

def load_info(dataset_path: Path) -> dict:
    info_path = dataset_path / "meta" / "info.json"
    with open(info_path, encoding="utf-8") as f:
        return json.load(f)


def load_episodes_meta(dataset_path: Path) -> list[dict]:
    episodes_path = dataset_path / "meta" / "episodes.jsonl"
    episodes = []
    with open(episodes_path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                episodes.append(json.loads(line))
    return episodes


def load_tasks_meta(dataset_path: Path) -> list[dict]:
    tasks_path = dataset_path / "meta" / "tasks.jsonl"
    tasks = []
    if tasks_path.exists():
        with open(tasks_path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line:
                    tasks.append(json.loads(line))
    return tasks


def get_parquet_path(dataset_path: Path, info: dict, episode_index: int) -> Path:
    data_path_pattern = info.get("data_path", "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet")
    chunks_size = info.get("chunks_size", 1000)
    episode_chunk = episode_index // chunks_size
    return dataset_path / data_path_pattern.format(
        episode_chunk=episode_chunk, episode_index=episode_index,
    )


def get_video_keys(info: dict) -> list[str]:
    """Extract video keys from info.json, falling back to features if needed."""
    keys = info.get("video_keys", [])
    if not keys:
        keys = [
            k for k, v in info.get("features", {}).items()
            if v.get("dtype") in ("video", "image")
        ]
    return keys


def get_video_paths(dataset_path: Path, info: dict, episode_index: int) -> dict[str, Path]:
    video_path_pattern = info.get(
        "video_path",
        "videos/{video_key}/episode_{episode_index:06d}.mp4",
    )
    video_keys = get_video_keys(info)
    chunks_size = info.get("chunks_size", 1000)
    episode_chunk = episode_index // chunks_size
    paths = {}
    for key in video_keys:
        paths[key] = dataset_path / video_path_pattern.format(
            video_key=key, episode_index=episode_index,
            episode_chunk=episode_chunk,
        )
    return paths


def filter_video_frames(video_path: Path, valid_indices: np.ndarray, fps: int):
    """Re-encode a video keeping only frames at valid_indices."""
    input_container = av.open(str(video_path))
    input_stream = input_container.streams.video[0]

    all_frames = []
    for frame in input_container.decode(input_stream):
        all_frames.append(frame.to_ndarray(format="rgb24"))
    input_container.close()

    if len(all_frames) == 0:
        return

    filtered = [all_frames[i] for i in valid_indices if i < len(all_frames)]
    if len(filtered) == 0:
        return

    tmp_path = video_path.with_suffix(".tmp.mp4")
    output_container = av.open(str(tmp_path), mode="w")
    output_stream = output_container.add_stream("h264", rate=fps)
    h, w = filtered[0].shape[:2]
    output_stream.width = w
    output_stream.height = h
    output_stream.pix_fmt = "yuv420p"

    for img in filtered:
        frame = av.VideoFrame.from_ndarray(img, format="rgb24")
        for packet in output_stream.encode(frame):
            output_container.mux(packet)
    for packet in output_stream.encode():
        output_container.mux(packet)
    output_container.close()

    tmp_path.replace(video_path)


# ---------------------------------------------------------------------------
# Script config validation
# ---------------------------------------------------------------------------

def validate_script_configs(dataset_paths: list[Path]) -> dict | None:
    """Check that all datasets share the same script_config.

    Returns the common config if they match, or raises an error with
    details about which datasets differ.
    """
    configs = {}
    for ds_path in dataset_paths:
        info = load_info(ds_path)
        sc = info.get("script_config")
        if sc is not None:
            configs[ds_path.name] = sc

    if not configs:
        return None

    canonical = json.dumps(next(iter(configs.values())), sort_keys=True)
    mismatched = []
    for name, cfg in configs.items():
        if json.dumps(cfg, sort_keys=True) != canonical:
            mismatched.append(name)

    if mismatched:
        print("\nERROR: script_config mismatch across datasets.")
        print("The following datasets have different robot configurations:\n")
        ref_name = next(iter(configs.keys()))
        print(f"  Reference: {ref_name}")
        for name in mismatched:
            print(f"  Differs:   {name}")
        print(
            "\nDatasets recorded with different robot configurations cannot be "
            "merged. Verify that all sessions used the same robot setup."
        )
        raise SystemExit(1)

    return next(iter(configs.values()))


# ---------------------------------------------------------------------------
# Core processing
# ---------------------------------------------------------------------------

def process_single_dataset(
    dataset_path: Path,
    remove_stale_smpl: bool,
    remove_discarded: bool = False,
    episode_index_offset: int = 0,
    exclude_episodes: set[int] | None = None,
    task_index_remap: dict[int, int] | None = None,
    keep_zero_smpl_modes: tuple[int, ...] = (INTERVENTION_STREAM_MODE, POLICY_STREAM_MODE),
    intervention_segments: bool = False,
    intervention_preroll_frames: int = 0,
    keep_episodes_without_interventions: bool = False,
) -> dict:
    """Process one dataset: optionally clean stale SMPL frames.

    Returns stats dict and the list of (parquet_df, video_paths, episode_meta)
    tuples for merging.

    ``task_index_remap`` maps this dataset's ``task_index`` values onto the output
    dataset's ``tasks.jsonl`` (see ``merge_tasks_meta``); every row must resolve.

    ``keep_zero_smpl_modes``: frames in these stream modes are never "stale SMPL" (their
    smpl_pose is zero by construction). ``intervention_segments``: emit one output episode per
    intervention window (stream_mode 5 plus ``intervention_preroll_frames`` before it) and drop
    the rest of the episode; episodes without interventions are dropped unless
    ``keep_episodes_without_interventions``.
    """
    info = load_info(dataset_path)
    episodes_meta = load_episodes_meta(dataset_path)
    fps = info.get("fps", 50)

    discarded_indices = set(info.get("discarded_episode_indices", [])) if remove_discarded else set()
    excluded_indices = set(exclude_episodes or ())

    stats = {
        "total_episodes": len(episodes_meta),
        "episodes_with_stale": 0,
        "total_frames": 0,
        "frames_removed": 0,
        "zero_frames": 0,
        "frozen_leadin_frames": 0,
        "episodes_dropped": 0,
        "episodes_discarded": 0,
        "episodes_excluded": 0,
        "intervention_segments": 0,
        "intervention_frames": 0,
        "policy_frames": 0,
        "episodes_without_interventions": 0,
    }
    processed_episodes = []

    for ep_meta in episodes_meta:
        ep_idx = ep_meta["episode_index"]

        if ep_idx in discarded_indices:
            stats["episodes_discarded"] += 1
            print(f"  Episode {ep_idx}: discarded during collection — removing")
            continue

        if ep_idx in excluded_indices:
            stats["episodes_excluded"] += 1
            print(f"  Episode {ep_idx}: excluded by --exclude-episodes — removing")
            continue

        parquet_path = get_parquet_path(dataset_path, info, ep_idx)
        video_paths = get_video_paths(dataset_path, info, ep_idx)

        if not parquet_path.exists():
            print(f"  WARNING: Missing parquet for episode {ep_idx}, skipping")
            continue

        df = pd.read_parquet(parquet_path)
        ep_len = len(df)
        stats["total_frames"] += ep_len

        if task_index_remap is not None and "task_index" in df.columns:
            mapped = df["task_index"].map(task_index_remap)
            if mapped.isna().any():
                unknown = sorted(set(df["task_index"][mapped.isna()].tolist()))
                print(
                    f"ERROR: episode {ep_idx} of {dataset_path} uses task_index {unknown} "
                    "which is not in its meta/tasks.jsonl — corrupt dataset."
                )
                raise SystemExit(1)
            df["task_index"] = mapped.astype(df["task_index"].dtype)

        valid_indices = None
        stream_modes = stream_modes_of(df)
        if stream_modes is not None:
            n_int = int((stream_modes == INTERVENTION_STREAM_MODE).sum())
            n_pol = int((stream_modes == POLICY_STREAM_MODE).sum())
            stats["intervention_frames"] += n_int
            stats["policy_frames"] += n_pol
            n_segs = len(contiguous_segments(stream_modes == INTERVENTION_STREAM_MODE))
            stats["intervention_segments"] += n_segs
            if n_pol or n_int:
                print(f"  Episode {ep_idx}: stream modes {format_mode_counts(stream_modes)}"
                      + (f" — {n_segs} intervention segment(s)" if n_segs else ""))

        if remove_stale_smpl and SMPL_POSE_COLUMN in df.columns:
            smpl_arr = np.vstack(
                [np.asarray(x, dtype=np.float32) for x in df[SMPL_POSE_COLUMN]]
            )
            mask = build_stale_mask(smpl_arr)
            if stream_modes is not None and keep_zero_smpl_modes:
                # zero SMPL is the normal state of policy / VR_3PT frames, not a dropout
                mask &= ~np.isin(stream_modes, list(keep_zero_smpl_modes))
            n_remove = int(mask.sum())
            n_zero = int(np.all(smpl_arr == 0, axis=1).sum())
            n_frozen = n_remove - n_zero

            if n_remove > 0:
                stats["episodes_with_stale"] += 1
                stats["frames_removed"] += n_remove
                stats["zero_frames"] += n_zero
                stats["frozen_leadin_frames"] += n_frozen
                pct = 100.0 * n_remove / ep_len
                print(
                    f"  Episode {ep_idx}: removing {n_remove}/{ep_len} frames "
                    f"({pct:.1f}%) — {n_zero} zero + {n_frozen} frozen lead-in"
                )

                if n_remove == ep_len:
                    print(f"  Episode {ep_idx}: ALL frames stale — dropping episode")
                    stats["episodes_dropped"] += 1
                    continue

                valid_indices = np.where(~mask)[0]
                df = df.iloc[valid_indices].copy().reset_index(drop=True)
                if "timestamp" in df.columns:
                    df["timestamp"] -= df["timestamp"].iloc[0]

        if intervention_segments:
            # Indices below are into the (possibly stale-cleaned) df; map back to source video frames.
            source_rows = valid_indices if valid_indices is not None else np.arange(len(df))
            modes_now = stream_modes_of(df)
            windows = (
                intervention_windows(modes_now, intervention_preroll_frames)
                if modes_now is not None else []
            )
            if not windows:
                stats["episodes_without_interventions"] += 1
                if not keep_episodes_without_interventions:
                    print(f"  Episode {ep_idx}: no intervention — dropping (--keep-episodes-without-interventions keeps it)")
                    stats["episodes_dropped"] += 1
                    continue
            else:
                for w_i, (w_start, w_end) in enumerate(windows):
                    rows = np.arange(w_start, w_end)
                    sub = df.iloc[rows].copy().reset_index(drop=True)
                    if "timestamp" in sub.columns:
                        sub["timestamp"] -= sub["timestamp"].iloc[0]
                    print(f"  Episode {ep_idx}: intervention window {w_i}: frames {w_start}-{w_end - 1} "
                          f"({w_end - w_start} frames, {int((modes_now[rows] == INTERVENTION_STREAM_MODE).sum())} corrected)")
                    processed_episodes.append({
                        "df": sub,
                        "source_video_paths": video_paths,
                        "valid_indices": np.asarray(source_rows)[rows],
                        "episode_meta": ep_meta,
                        "new_episode_index": len(processed_episodes) + episode_index_offset,
                        "fps": fps,
                    })
                continue

        new_ep_idx = ep_idx + episode_index_offset
        processed_episodes.append({
            "df": df,
            "source_video_paths": video_paths,
            "valid_indices": valid_indices,
            "episode_meta": ep_meta,
            "new_episode_index": new_ep_idx,
            "fps": fps,
        })

    return stats, processed_episodes, info


def merge_tasks_meta(dataset_paths: list[Path]) -> tuple[list[dict], dict[Path, dict[int, int]]]:
    """Merge the inputs' ``tasks.jsonl`` by task STRING.

    Returns the output ``tasks.jsonl`` rows (first-seen order, indices 0..n-1) and,
    per input dataset, the ``task_index`` remap to apply to its parquet rows. Two
    sessions that recorded the same prompts in a different order (per-episode
    prompts, g1-vr-teleop #40) must not end up relabelled: deduplicating by
    ``task_index`` alone did exactly that (g1-vr-teleop #41).
    """
    merged: dict[str, int] = {}
    remaps: dict[Path, dict[int, int]] = {}
    for ds in dataset_paths:
        remap: dict[int, int] = {}
        for task in load_tasks_meta(ds):
            if "task_index" not in task or "task" not in task:
                print(f"ERROR: malformed row in {ds / 'meta' / 'tasks.jsonl'}: {task}")
                raise SystemExit(1)
            text = task["task"]
            if text not in merged:
                merged[text] = len(merged)
            remap[int(task["task_index"])] = merged[text]
        remaps[ds] = remap
    tasks_meta = [{"task_index": i, "task": text} for text, i in merged.items()]
    return tasks_meta, remaps


def write_output_dataset(
    dest_path: Path,
    all_episodes: list[dict],
    reference_info: dict,
    tasks_meta: list[dict],
    script_config: dict | None,
):
    """Write processed episodes to a new LeRobot dataset directory."""
    dest_path.mkdir(parents=True, exist_ok=True)
    meta_dir = dest_path / "meta"
    meta_dir.mkdir(exist_ok=True)

    info = reference_info.copy()
    fps = info.get("fps", 50)
    chunks_size = info.get("chunks_size", 1000)

    if script_config is not None:
        info["script_config"] = script_config

    total_frames = 0
    episodes_jsonl = []

    for i, ep in enumerate(all_episodes):
        df = ep["df"]
        ep_len = len(df)

        df["episode_index"] = i
        df["index"] = range(total_frames, total_frames + ep_len)
        df["frame_index"] = range(ep_len)
        if "timestamp" in df.columns:
            df["timestamp"] = [j / fps for j in range(ep_len)]

        episode_chunk = i // chunks_size
        data_path_pattern = info.get(
            "data_path",
            "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        )
        parquet_rel = data_path_pattern.format(episode_chunk=episode_chunk, episode_index=i)
        parquet_path = dest_path / parquet_rel
        parquet_path.parent.mkdir(parents=True, exist_ok=True)
        df.to_parquet(parquet_path)

        video_keys = get_video_keys(info)
        video_path_pattern = info.get(
            "video_path",
            "videos/{video_key}/episode_{episode_index:06d}.mp4",
        )
        for vkey in video_keys:
            src_video = ep["source_video_paths"].get(vkey)
            dst_rel = video_path_pattern.format(
                video_key=vkey, episode_index=i, episode_chunk=episode_chunk,
            )
            dst_video = dest_path / dst_rel
            dst_video.parent.mkdir(parents=True, exist_ok=True)

            if src_video and src_video.exists():
                if ep["valid_indices"] is not None:
                    shutil.copy2(src_video, dst_video)
                    filter_video_frames(dst_video, ep["valid_indices"], fps)
                else:
                    shutil.copy2(src_video, dst_video)

        ep_meta = {
            "episode_index": i,
            "tasks": ep["episode_meta"].get("tasks", []),
            "length": ep_len,
        }
        episodes_jsonl.append(ep_meta)

        total_frames += ep_len

    info["total_episodes"] = len(all_episodes)
    info["total_frames"] = total_frames
    # total_videos/total_tasks/splits describe the OUTPUT, so recompute them —
    # carrying the source values over leaves e.g. total_videos counting episodes
    # that were just dropped.
    info["total_videos"] = len(all_episodes) * len(get_video_keys(info))
    if tasks_meta:
        info["total_tasks"] = len(tasks_meta)
    if "splits" in info:
        info["splits"] = {"train": f"0:{len(all_episodes)}"}
    info.pop("discarded_episode_indices", None)

    with open(meta_dir / "info.json", "w", encoding="utf-8") as f:
        json.dump(info, f, indent=4)

    with open(meta_dir / "episodes.jsonl", "w", encoding="utf-8") as f:
        for ep in episodes_jsonl:
            f.write(json.dumps(ep) + "\n")

    if tasks_meta:
        with open(meta_dir / "tasks.jsonl", "w", encoding="utf-8") as f:
            for task in tasks_meta:
                f.write(json.dumps(task) + "\n")

    return total_frames


def copy_modality_json(source_paths: list[Path], output_path: Path):
    """Copy modality.json from the first source that has one."""
    for src in source_paths:
        modality_path = src / "meta" / "modality.json"
        if modality_path.exists():
            dst = output_path / "meta" / "modality.json"
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(modality_path, dst)
            return


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

@dataclass
class ProcessDatasetConfig:
    """Post-process LeRobot datasets: clean stale SMPL frames and/or merge."""

    dataset_path: list[str] = field(default_factory=list)
    """One or more dataset directories to process."""

    dataset_list: Optional[str] = None
    """Path to a text file listing dataset directories (one per line).
    Can be used instead of or in addition to --dataset-path."""

    output_path: Optional[str] = None
    """Output directory for the processed dataset. If not specified and a
    single dataset is given, the dataset is modified in-place."""

    remove_stale_smpl: bool = True
    """Remove frames where teleop.smpl_pose is all zeros (stale/dropped
    SMPL data) and frozen lead-in frames that precede them."""

    remove_discarded: bool = True
    """Remove episodes that were flagged as discarded during data collection
    (stored in meta/info.json under discarded_episode_indices)."""

    exclude_episodes: list[int] = field(default_factory=list)
    """Episode indices to drop, on top of the discarded ones — for takes that
    were saved but are not wanted (misfires, bad runs). Indices refer to the
    SOURCE dataset; surviving episodes are renumbered contiguously from 0. When
    merging several datasets these apply to every input, so exclude in separate
    runs if that is not what you want."""

    keep_zero_smpl_modes: list[int] = field(default_factory=lambda: [INTERVENTION_STREAM_MODE, POLICY_STREAM_MODE])
    """Stream modes whose frames are never removed as stale SMPL (their teleop.smpl_pose is
    zero by construction): 5 = PLANNER_VR_3PT (interventions), 6 = POLICY. Datasets without a
    teleop.stream_mode column are unaffected."""

    intervention_segments: bool = False
    """DAgger (g1-vr-teleop #25): write one output episode per intervention (run of
    stream_mode 5 frames) plus the pre-roll before it, and drop the rest of each episode.
    Requires --output-path."""

    intervention_preroll_s: float = 1.0
    """Seconds of context kept before each intervention (policy frames, so the model sees
    the state it was in when the operator took over). Overlapping windows are merged."""

    keep_episodes_without_interventions: bool = False
    """With --intervention-segments: keep episodes that have no intervention whole (e.g.
    clean autonomous successes) instead of dropping them."""


def main(cfg: ProcessDatasetConfig):
    dataset_paths = [Path(p) for p in cfg.dataset_path]

    if cfg.dataset_list:
        list_file = Path(cfg.dataset_list)
        with open(list_file, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#"):
                    dataset_paths.append(Path(line))

    if not dataset_paths:
        print("ERROR: No dataset paths provided. Use --dataset-path or --dataset-list.")
        raise SystemExit(1)

    for ds in dataset_paths:
        if not ds.exists():
            print(f"ERROR: Dataset path does not exist: {ds}")
            raise SystemExit(1)
        if not (ds / "meta" / "info.json").exists():
            print(f"ERROR: Not a valid LeRobot dataset (missing meta/info.json): {ds}")
            raise SystemExit(1)

    merging = len(dataset_paths) > 1
    in_place = cfg.output_path is None

    if merging and in_place:
        print("ERROR: --output-path is required when merging multiple datasets.")
        raise SystemExit(1)

    if cfg.intervention_segments and in_place:
        print("ERROR: --output-path is required with --intervention-segments (episodes are split).")
        raise SystemExit(1)

    output_path = Path(cfg.output_path) if cfg.output_path else dataset_paths[0]

    print("=" * 70)
    print("  LeRobot Dataset Processor")
    print("=" * 70)
    print(f"  Input datasets:       {len(dataset_paths)}")
    for ds in dataset_paths:
        print(f"    - {ds}")
    print(f"  Output:               {output_path}{'  (in-place)' if in_place else ''}")
    print(f"  Remove stale SMPL:    {cfg.remove_stale_smpl}")
    print(f"  Remove discarded:     {cfg.remove_discarded}")
    if cfg.exclude_episodes:
        print(f"  Exclude episodes:     {sorted(cfg.exclude_episodes)}")
    if cfg.intervention_segments:
        print(f"  Intervention segments: yes (pre-roll {cfg.intervention_preroll_s:.2f} s, "
              f"{'keep' if cfg.keep_episodes_without_interventions else 'drop'} episodes without one)")
    print("=" * 70)

    # Validate script configs match across all datasets
    if merging:
        print("\nValidating script_config across datasets...")
        script_config = validate_script_configs(dataset_paths)
        print("  All datasets have matching robot configurations.\n")
    else:
        info = load_info(dataset_paths[0])
        script_config = info.get("script_config")

    # Merge tasks by string; rows are remapped onto the merged tasks.jsonl. In-place
    # runs leave tasks.jsonl untouched, so their rows are not remapped either.
    all_tasks_meta, task_remaps = merge_tasks_meta(dataset_paths)

    # Process each dataset
    all_episodes = []
    total_stats = {
        "total_episodes": 0,
        "episodes_with_stale": 0,
        "total_frames": 0,
        "frames_removed": 0,
        "zero_frames": 0,
        "frozen_leadin_frames": 0,
        "episodes_dropped": 0,
        "episodes_discarded": 0,
        "episodes_excluded": 0,
        "intervention_segments": 0,
        "intervention_frames": 0,
        "policy_frames": 0,
        "episodes_without_interventions": 0,
    }
    reference_info = None

    for ds_path in dataset_paths:
        print(f"\nProcessing: {ds_path}")
        fps_ds = load_info(ds_path).get("fps", 50)
        stats, episodes, info = process_single_dataset(
            ds_path,
            remove_stale_smpl=cfg.remove_stale_smpl,
            remove_discarded=cfg.remove_discarded,
            episode_index_offset=len(all_episodes),
            exclude_episodes=set(cfg.exclude_episodes),
            task_index_remap=None if in_place else task_remaps[ds_path],
            keep_zero_smpl_modes=tuple(cfg.keep_zero_smpl_modes),
            intervention_segments=cfg.intervention_segments,
            intervention_preroll_frames=int(round(cfg.intervention_preroll_s * fps_ds)),
            keep_episodes_without_interventions=cfg.keep_episodes_without_interventions,
        )

        if reference_info is None:
            reference_info = info

        all_episodes.extend(episodes)
        for key in total_stats:
            total_stats[key] += stats[key]

    if not all_episodes:
        print("\nERROR: No valid episodes after processing.")
        raise SystemExit(1)

    # Write output
    if in_place:
        # In-place: rewrite parquet files and re-encode videos
        print(f"\nRewriting dataset in-place at {output_path}...")
        ds_info = load_info(output_path)
        fps = ds_info.get("fps", 50)

        # Delete files for discarded episodes
        if cfg.remove_discarded:
            discarded_indices = set(ds_info.get("discarded_episode_indices", []))
            for ep_idx in discarded_indices:
                parquet_path = get_parquet_path(output_path, ds_info, ep_idx)
                if parquet_path.exists():
                    parquet_path.unlink()
                video_paths = get_video_paths(output_path, ds_info, ep_idx)
                for _vkey, vpath in video_paths.items():
                    if vpath.exists():
                        vpath.unlink()

        for ep in all_episodes:
            ep_idx = ep["episode_meta"]["episode_index"]
            parquet_path = get_parquet_path(output_path, ds_info, ep_idx)
            ep["df"].to_parquet(parquet_path)

            if ep["valid_indices"] is not None:
                video_paths = get_video_paths(output_path, ds_info, ep_idx)
                for _vkey, vpath in video_paths.items():
                    if vpath.exists():
                        filter_video_frames(vpath, ep["valid_indices"], fps)

        # Update episode metadata
        episodes_meta = []
        for ep in all_episodes:
            meta = ep["episode_meta"].copy()
            meta["length"] = len(ep["df"])
            episodes_meta.append(meta)

        with open(output_path / "meta" / "episodes.jsonl", "w", encoding="utf-8") as f:
            for em in episodes_meta:
                f.write(json.dumps(em) + "\n")

        ds_info["total_frames"] = sum(len(ep["df"]) for ep in all_episodes)
        ds_info["total_episodes"] = len(all_episodes)
        if cfg.remove_discarded:
            ds_info.pop("discarded_episode_indices", None)
        with open(output_path / "meta" / "info.json", "w", encoding="utf-8") as f:
            json.dump(ds_info, f, indent=4)
    else:
        print(f"\nWriting output dataset to {output_path}...")
        write_output_dataset(
            output_path, all_episodes, reference_info, all_tasks_meta, script_config,
        )
        copy_modality_json(dataset_paths, output_path)

    # Print summary
    kept = total_stats["total_frames"] - total_stats["frames_removed"]
    kept_episodes = (
        total_stats["total_episodes"]
        - total_stats["episodes_dropped"]
        - total_stats["episodes_discarded"]
        - total_stats["episodes_excluded"]
    )

    print("\n" + "=" * 70)
    print("  Processing complete!")
    print("=" * 70)
    print(f"  Episodes:  {kept_episodes} kept / {total_stats['total_episodes']} total"
          f"  ({total_stats['episodes_dropped']} dropped, "
          f"{total_stats['episodes_discarded']} discarded, "
          f"{total_stats['episodes_excluded']} excluded)")
    print(f"  Frames:    {kept} kept / {total_stats['total_frames']} total"
          f"  ({total_stats['frames_removed']} removed)")
    if total_stats["frames_removed"] > 0:
        print(f"    Zero SMPL:       {total_stats['zero_frames']}")
        print(f"    Frozen lead-in:  {total_stats['frozen_leadin_frames']}")
        print(f"    Episodes affected: {total_stats['episodes_with_stale']}")
    if total_stats["policy_frames"] or total_stats["intervention_frames"]:
        print(f"  DAgger:    {total_stats['policy_frames']} policy frames, "
              f"{total_stats['intervention_frames']} intervention frames in "
              f"{total_stats['intervention_segments']} segment(s)"
              + (f"; {len(all_episodes)} output episode(s) from the segments"
                 if cfg.intervention_segments else ""))
    print(f"  Output:    {output_path}")
    print("=" * 70)


if __name__ == "__main__":
    main(tyro.cli(ProcessDatasetConfig))

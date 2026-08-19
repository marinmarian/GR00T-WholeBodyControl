"""Episode storage layout for G1 teleop recording.

Captures always land under `raw/` (observability, not an opt-in storage mode).
Copying rated episodes into a sibling `recorded/{quality}/` tree requires
`data_collection=True` (`--data-collection`):

    {root_output_dir}/
      raw/{dataset_name}/                 # always written (key `c`)
      recorded/                           # only with data_collection
        good/{dataset_name}/              # rated good (key `g`)
        neutral/{dataset_name}/           # rated neutral (key `v`)
        bad/{dataset_name}/               # rated bad (key `b`)

The same suffixes are applied to `BUCKET_BASE_PATH` for uploads. Discarded
episodes (key `x`) stay in `raw/` and are never copied into `recorded/`.

Each leaf folder keeps a LeRobot dataset layout (meta/, data/, videos/).
"""

from dataclasses import dataclass
import json
from pathlib import Path
import shutil
from typing import Literal, Optional

from decoupled_wbc.data.constants import BUCKET_BASE_PATH

SaveLayout = Literal["raw-recorded", "flat"]

# Keyboard keys already used by the exporter. `x` (discard) is intentionally
# absent: discarded episodes must not be promoted into recorded/.
RATING_KEY_TO_QUALITY = {
    "g": "good",
    "v": "neutral",
    "b": "bad",
}

_SHARED_META_FILES = ("info.json", "modality.json", "tasks.jsonl")
_EPISODE_JSONL_FILES = ("episodes.jsonl", "episodes_stats.jsonl")


@dataclass(frozen=True)
class EpisodeSavePaths:
    """Resolved local and bucket paths for one teleop dataset."""

    root_output_dir: str
    dataset_name: str
    save_layout: SaveLayout = "raw-recorded"
    data_collection: bool = False
    bucket_base: str = BUCKET_BASE_PATH

    def capture_root(self) -> Path:
        """Directory where live teleop captures are written."""
        if self.save_layout == "raw-recorded":
            return Path(self.root_output_dir) / "raw" / self.dataset_name
        if self.save_layout == "flat":
            return Path(self.root_output_dir) / self.dataset_name
        raise ValueError(f"Unknown save_layout: {self.save_layout}")

    def recorded_root(self, quality: str) -> Path:
        """Sibling LeRobot dataset for episodes rated `quality`."""
        return Path(self.root_output_dir) / "recorded" / quality / self.dataset_name

    def upload_bucket_path(self, quality: Optional[str] = None) -> str:
        """Upload prefix under the existing bucket. `quality=None` is the raw dump."""
        if self.save_layout == "flat":
            return self.bucket_base
        if quality is None:
            return f"{self.bucket_base}/raw"
        return f"{self.bucket_base}/recorded/{quality}"


def quality_for_rating_key(rating_key: str) -> str:
    """Map a rating key to a recorded/ folder name. Rejects discard (`x`)."""
    try:
        return RATING_KEY_TO_QUALITY[rating_key]
    except KeyError as exc:
        raise ValueError(
            f"Unknown rating key {rating_key!r}. "
            f"Expected one of {sorted(RATING_KEY_TO_QUALITY)} "
            "(discarded episodes with key 'x' are not promoted into recorded/)."
        ) from exc


def apply_episode_rating(
    *,
    paths: EpisodeSavePaths,
    raw_root: Path,
    episode_index: int,
    rating_key: str,
) -> str:
    """Record a quality rating and optionally copy into `recorded/`.

    Always appends `{episode_index, rating}` to `raw_root/meta/ratings.jsonl`.
    When `data_collection` is True, also copies that episode's LeRobot files
    into `recorded/{quality}/{dataset_name}/`.
    """
    quality = quality_for_rating_key(rating_key)
    _append_rating(Path(raw_root) / "meta" / "ratings.jsonl", episode_index, quality)

    if not paths.data_collection:
        return quality

    dest_root = paths.recorded_root(quality)
    _promote_episode_files(Path(raw_root), dest_root, episode_index)
    _append_rating(dest_root / "meta" / "ratings.jsonl", episode_index, quality)
    return quality


def _append_rating(ratings_path: Path, episode_index: int, quality: str) -> None:
    ratings_path.parent.mkdir(parents=True, exist_ok=True)
    with ratings_path.open("a") as f:
        f.write(json.dumps({"episode_index": episode_index, "rating": quality}) + "\n")


def _promote_episode_files(raw_root: Path, dest_root: Path, episode_index: int) -> None:
    dest_root.mkdir(parents=True, exist_ok=True)
    padded = f"{episode_index:06d}"
    for src in (
        *raw_root.rglob(f"episode_{padded}.parquet"),
        *raw_root.rglob(f"episode_{padded}.mp4"),
    ):
        rel = src.relative_to(raw_root)
        if rel.parts[0] not in ("data", "videos"):
            continue
        dest = dest_root / rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dest)

    dest_meta = dest_root / "meta"
    dest_meta.mkdir(parents=True, exist_ok=True)
    src_meta = raw_root / "meta"
    for name in _SHARED_META_FILES:
        src = src_meta / name
        dest = dest_meta / name
        if src.exists() and not dest.exists():
            shutil.copy2(src, dest)

    for name in _EPISODE_JSONL_FILES:
        _append_matching_jsonl(src_meta / name, dest_meta / name, episode_index)


def _append_matching_jsonl(src: Path, dest: Path, episode_index: int) -> None:
    if not src.exists():
        return
    dest.parent.mkdir(parents=True, exist_ok=True)
    with src.open() as f_in, dest.open("a") as f_out:
        for line in f_in:
            stripped = line.strip()
            if not stripped:
                continue
            record = json.loads(stripped)
            if record.get("episode_index") == episode_index:
                f_out.write(json.dumps(record) + "\n")

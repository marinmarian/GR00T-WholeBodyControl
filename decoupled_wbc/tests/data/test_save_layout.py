"""Path layout for G1 teleop episode storage: raw dump vs recorded/{quality}.

Quality vocabulary (keyboard keys already used by the exporter):
    g -> good
    v -> neutral
    b -> bad
    x -> discarded (never promoted into recorded/)

Raw capture is unconditional. Copying into recorded/ requires data_collection.
"""

import json
from pathlib import Path

from decoupled_wbc.data.constants import BUCKET_BASE_PATH
from decoupled_wbc.data.save_layout import (
    RATING_KEY_TO_QUALITY,
    EpisodeSavePaths,
    apply_episode_rating,
)


def _fake_lerobot_episode(root: Path, episode_index: int) -> None:
    padded = f"{episode_index:06d}"
    parquet = root / "data" / "chunk-000" / f"episode_{padded}.parquet"
    video = (
        root
        / "videos"
        / "chunk-000"
        / "observation.images.ego_view"
        / f"episode_{padded}.mp4"
    )
    parquet.parent.mkdir(parents=True, exist_ok=True)
    video.parent.mkdir(parents=True, exist_ok=True)
    parquet.write_bytes(b"parquet")
    video.write_bytes(b"mp4")
    meta = root / "meta"
    meta.mkdir(parents=True, exist_ok=True)
    (meta / "info.json").write_text("{}")
    (meta / "modality.json").write_text("{}")
    with (meta / "episodes.jsonl").open("a") as f:
        f.write(json.dumps({"episode_index": episode_index, "length": 10}) + "\n")


def test_default_capture_root_is_raw_dataset(tmp_path):
    paths = EpisodeSavePaths(root_output_dir=str(tmp_path), dataset_name="g1_teleop")
    assert paths.data_collection is False
    assert paths.capture_root() == tmp_path / "raw" / "g1_teleop"
    assert paths.upload_bucket_path() == "darwin-robot-data/raw"


def test_default_bucket_base_is_darwin_robot_data():
    assert BUCKET_BASE_PATH == "darwin-robot-data"
    paths = EpisodeSavePaths(root_output_dir="outputs", dataset_name="g1_teleop")
    assert paths.bucket_base == "darwin-robot-data"


def test_default_rating_does_not_create_recorded(tmp_path):
    paths = EpisodeSavePaths(root_output_dir=str(tmp_path), dataset_name="g1_teleop")
    raw = paths.capture_root()
    _fake_lerobot_episode(raw, 0)

    apply_episode_rating(
        paths=paths,
        raw_root=raw,
        episode_index=0,
        rating_key="g",
    )

    assert json.loads((raw / "meta" / "ratings.jsonl").read_text().splitlines()[-1]) == {
        "episode_index": 0,
        "rating": "good",
    }
    assert not (tmp_path / "recorded").exists()


def test_flat_layout_keeps_legacy_save_root(tmp_path):
    paths = EpisodeSavePaths(
        root_output_dir=str(tmp_path),
        dataset_name="g1_teleop",
        save_layout="flat",
    )
    assert paths.capture_root() == tmp_path / "g1_teleop"


def test_recorded_root_splits_by_quality(tmp_path):
    paths = EpisodeSavePaths(
        root_output_dir=str(tmp_path),
        dataset_name="g1_teleop",
    )
    assert paths.recorded_root("good") == tmp_path / "recorded" / "good" / "g1_teleop"
    assert paths.recorded_root("neutral") == tmp_path / "recorded" / "neutral" / "g1_teleop"
    assert paths.recorded_root("bad") == tmp_path / "recorded" / "bad" / "g1_teleop"


def test_upload_bucket_reuses_bucket_base_path():
    paths = EpisodeSavePaths(
        root_output_dir="outputs",
        dataset_name="g1_teleop",
    )
    assert paths.upload_bucket_path() == "darwin-robot-data/raw"
    assert paths.upload_bucket_path("good") == "darwin-robot-data/recorded/good"
    assert paths.upload_bucket_path("neutral") == "darwin-robot-data/recorded/neutral"
    assert paths.upload_bucket_path("bad") == "darwin-robot-data/recorded/bad"


def test_custom_bucket_base_overrides_default():
    paths = EpisodeSavePaths(
        root_output_dir="outputs",
        dataset_name="g1_teleop",
        bucket_base="custom-bucket",
    )
    assert paths.upload_bucket_path() == "custom-bucket/raw"
    assert paths.upload_bucket_path("good") == "custom-bucket/recorded/good"
    assert paths.upload_bucket_path("neutral") == "custom-bucket/recorded/neutral"
    assert paths.upload_bucket_path("bad") == "custom-bucket/recorded/bad"


def test_rating_keys_map_to_good_neutral_bad():
    assert RATING_KEY_TO_QUALITY == {"g": "good", "v": "neutral", "b": "bad"}
    assert "x" not in RATING_KEY_TO_QUALITY


def test_data_collection_copies_episode_into_quality_dataset(tmp_path):
    paths = EpisodeSavePaths(
        root_output_dir=str(tmp_path),
        dataset_name="g1_teleop",
        data_collection=True,
    )
    raw = paths.capture_root()
    _fake_lerobot_episode(raw, 0)

    apply_episode_rating(
        paths=paths,
        raw_root=raw,
        episode_index=0,
        rating_key="g",
    )

    dest = paths.recorded_root("good")
    assert (dest / "data" / "chunk-000" / "episode_000000.parquet").exists()
    assert (
        dest / "videos" / "chunk-000" / "observation.images.ego_view" / "episode_000000.mp4"
    ).exists()
    assert (dest / "meta" / "info.json").exists()
    assert (dest / "meta" / "modality.json").exists()

    raw_rating = json.loads((raw / "meta" / "ratings.jsonl").read_text().splitlines()[-1])
    assert raw_rating == {"episode_index": 0, "rating": "good"}
    dest_rating = json.loads((dest / "meta" / "ratings.jsonl").read_text().splitlines()[-1])
    assert dest_rating == {"episode_index": 0, "rating": "good"}

    episodes = [
        json.loads(line)
        for line in (dest / "meta" / "episodes.jsonl").read_text().splitlines()
        if line.strip()
    ]
    assert episodes == [{"episode_index": 0, "length": 10}]


def test_neutral_and_bad_ratings_use_matching_recorded_folders(tmp_path):
    paths = EpisodeSavePaths(
        root_output_dir=str(tmp_path),
        dataset_name="g1_teleop",
        data_collection=True,
    )
    raw = paths.capture_root()
    _fake_lerobot_episode(raw, 1)
    _fake_lerobot_episode(raw, 2)

    apply_episode_rating(paths=paths, raw_root=raw, episode_index=1, rating_key="v")
    apply_episode_rating(paths=paths, raw_root=raw, episode_index=2, rating_key="b")

    assert (paths.recorded_root("neutral") / "data" / "chunk-000" / "episode_000001.parquet").exists()
    assert (paths.recorded_root("bad") / "data" / "chunk-000" / "episode_000002.parquet").exists()
    assert not (paths.recorded_root("good") / "data").exists()


def test_discard_key_is_not_promoted_into_recorded(tmp_path):
    paths = EpisodeSavePaths(
        root_output_dir=str(tmp_path),
        dataset_name="g1_teleop",
        data_collection=True,
    )
    raw = paths.capture_root()
    _fake_lerobot_episode(raw, 0)

    try:
        apply_episode_rating(
            paths=paths,
            raw_root=raw,
            episode_index=0,
            rating_key="x",
        )
    except ValueError:
        pass
    else:
        raise AssertionError("discarded episodes must not be promoted")

    assert not (tmp_path / "recorded").exists()
    assert not (raw / "meta" / "ratings.jsonl").exists()


def test_flat_layout_rates_in_place_without_recorded_copy(tmp_path):
    paths = EpisodeSavePaths(
        root_output_dir=str(tmp_path),
        dataset_name="g1_teleop",
        save_layout="flat",
    )
    raw = paths.capture_root()
    _fake_lerobot_episode(raw, 0)

    apply_episode_rating(
        paths=paths,
        raw_root=raw,
        episode_index=0,
        rating_key="g",
    )

    assert json.loads((raw / "meta" / "ratings.jsonl").read_text().splitlines()[-1]) == {
        "episode_index": 0,
        "rating": "good",
    }
    assert not (tmp_path / "recorded").exists()

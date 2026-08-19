"""Warn + local fallback when an S3 put fails. Creds are never a skip path.

Mocks boto3; does not require live AWS. Runtime always attempts the put;
HeadObject on raw/test.log is the access check (s3_log_accessible). The live
append probe against the real bucket lives in g1-vr-teleop
(tests/test_s3_test_log.py) and is deliberately not vendored here.
"""

from __future__ import annotations

from pathlib import Path

from botocore.exceptions import ClientError, NoCredentialsError
import pytest

from decoupled_wbc.data.s3_util import (
    put_bytes_with_fallback,
    resolve_observability_dir,
    s3_log_accessible,
    try_upload_episode_files,
)


def _client_error(code: str, op: str = "PutObject") -> ClientError:
    status = 403 if code == "AccessDenied" else 400
    if code in ("NoSuchKey", "NoSuchBucket", "404"):
        status = 404
    return ClientError(
        {
            "Error": {"Code": code, "Message": code},
            "ResponseMetadata": {"HTTPStatusCode": status},
        },
        op,
    )


def test_observability_dir_uses_g1_vr_log_dir(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setenv("G1_VR_LOG_DIR", str(tmp_path / "logs"))
    assert resolve_observability_dir() == tmp_path / "logs" / "observability"


def test_profile_set_is_not_enough_without_head_object(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("AWS_PROFILE", "darwin")

    class _Denied:
        def head_object(self, Bucket: str, Key: str) -> dict:
            raise _client_error("AccessDenied", "HeadObject")

    monkeypatch.setattr(
        "decoupled_wbc.data.s3_util._s3_client", lambda *args, **kwargs: _Denied()
    )
    monkeypatch.setattr(
        "decoupled_wbc.data.s3_util.resolve_dataset_bucket",
        lambda: "darwin-robot-data",
    )
    assert s3_log_accessible() is False


def test_put_still_attempts_when_log_not_accessible(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    class _NoCreds:
        def put_object(self, Bucket: str, Key: str, Body) -> dict:
            raise NoCredentialsError()

    monkeypatch.setattr(
        "decoupled_wbc.data.s3_util.s3_log_accessible", lambda: False
    )
    monkeypatch.setattr(
        "decoupled_wbc.data.s3_util.resolve_dataset_bucket",
        lambda: "darwin-robot-data",
    )
    monkeypatch.setattr(
        "decoupled_wbc.data.s3_util._s3_client", lambda *args, **kwargs: _NoCreds()
    )
    local_path = tmp_path / "observability" / "probe.bin"
    result = put_bytes_with_fallback(
        key="raw/xr/probe.bin",
        body=b"kept-locally",
        local_path=local_path,
    )
    captured = capsys.readouterr()
    text = captured.out + captured.err
    assert "WARNING:" in text
    assert str(local_path) in text
    assert local_path.is_file()
    assert local_path.read_bytes() == b"kept-locally"
    assert result.uploaded is False
    assert result.local_path == local_path


def test_s3_access_denied_warns_and_writes_local(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    class _DenyS3:
        def put_object(self, Bucket: str, Key: str, Body) -> dict:
            raise _client_error("AccessDenied")

    monkeypatch.setattr(
        "decoupled_wbc.data.s3_util.resolve_dataset_bucket",
        lambda: "darwin-robot-data",
    )
    monkeypatch.setattr(
        "decoupled_wbc.data.s3_util._s3_client", lambda *args, **kwargs: _DenyS3()
    )
    local_path = tmp_path / "observability" / "denied.rrd"
    result = put_bytes_with_fallback(
        key="raw/xr/denied.rrd",
        body=b"rrd-bytes",
        local_path=local_path,
    )
    captured = capsys.readouterr()
    text = captured.out + captured.err
    assert "WARNING:" in text
    assert "AccessDenied" in text
    assert str(local_path) in text
    assert local_path.is_file()
    assert local_path.read_bytes() == b"rrd-bytes"
    assert result.uploaded is False


def test_s3_no_such_bucket_warns_and_writes_local(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    class _MissingBucket:
        def put_object(self, Bucket: str, Key: str, Body) -> dict:
            raise _client_error("NoSuchBucket")

    monkeypatch.setattr(
        "decoupled_wbc.data.s3_util.resolve_dataset_bucket",
        lambda: "darwin-robot-data",
    )
    monkeypatch.setattr(
        "decoupled_wbc.data.s3_util._s3_client",
        lambda *args, **kwargs: _MissingBucket(),
    )
    local_path = tmp_path / "observability" / "nobucket.rrd"
    result = put_bytes_with_fallback(
        key="raw/xr/nobucket.rrd",
        body=b"still-here",
        local_path=local_path,
    )
    captured = capsys.readouterr()
    text = captured.out + captured.err
    assert "NoSuchBucket" in text
    assert local_path.read_bytes() == b"still-here"
    assert result.uploaded is False


def test_s3_timeout_warns_and_writes_local(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    class _TimeoutS3:
        def put_object(self, Bucket: str, Key: str, Body) -> dict:
            raise TimeoutError("timed out")

    monkeypatch.setattr(
        "decoupled_wbc.data.s3_util.resolve_dataset_bucket",
        lambda: "darwin-robot-data",
    )
    monkeypatch.setattr(
        "decoupled_wbc.data.s3_util._s3_client", lambda *args, **kwargs: _TimeoutS3()
    )
    local_path = tmp_path / "observability" / "timeout.rrd"
    result = put_bytes_with_fallback(
        key="raw/xr/timeout.rrd",
        body=b"local-copy",
        local_path=local_path,
    )
    captured = capsys.readouterr()
    text = captured.out + captured.err
    assert "WARNING:" in text
    assert "TimeoutError" in text
    assert local_path.read_bytes() == b"local-copy"
    assert result.uploaded is False


def test_s3_success_uploads_and_keeps_local(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    stored: dict[tuple[str, str], bytes] = {}

    class _OkS3:
        def put_object(self, Bucket: str, Key: str, Body) -> dict:
            if isinstance(Body, str):
                Body = Body.encode("utf-8")
            stored[(Bucket, Key)] = bytes(Body)
            return {}

    monkeypatch.setattr(
        "decoupled_wbc.data.s3_util.resolve_dataset_bucket",
        lambda: "darwin-robot-data",
    )
    monkeypatch.setattr(
        "decoupled_wbc.data.s3_util._s3_client", lambda *args, **kwargs: _OkS3()
    )
    local_path = tmp_path / "observability" / "ok.rrd"
    result = put_bytes_with_fallback(
        key="raw/xr/ok.rrd",
        body=b"uploaded",
        local_path=local_path,
    )
    assert result.uploaded is True
    assert local_path.read_bytes() == b"uploaded"
    assert stored[("darwin-robot-data", "raw/xr/ok.rrd")] == b"uploaded"
    assert result.s3_uri == "s3://darwin-robot-data/raw/xr/ok.rrd"


def test_try_upload_episode_files_puts_parquet_not_videos(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    stored: dict[tuple[str, str], bytes] = {}

    class _OkS3:
        def put_object(self, Bucket: str, Key: str, Body) -> dict:
            if isinstance(Body, str):
                Body = Body.encode("utf-8")
            stored[(Bucket, Key)] = bytes(Body)
            return {}

    monkeypatch.setattr(
        "decoupled_wbc.data.s3_util.resolve_dataset_bucket",
        lambda: "darwin-robot-data",
    )
    monkeypatch.setattr(
        "decoupled_wbc.data.s3_util._s3_client", lambda *args, **kwargs: _OkS3()
    )
    episode = tmp_path / "raw" / "g1_teleop"
    parquet = episode / "data" / "chunk-000" / "episode_000000.parquet"
    video = episode / "videos" / "chunk-000" / "cam" / "episode_000000.mp4"
    parquet.parent.mkdir(parents=True)
    video.parent.mkdir(parents=True)
    parquet.write_bytes(b"parquet")
    video.write_bytes(b"mp4-bytes")
    results = try_upload_episode_files(
        local_root=episode,
        s3_prefix="raw/g1_teleop",
    )
    keys = {key for (_bucket, key) in stored}
    assert "raw/g1_teleop/data/chunk-000/episode_000000.parquet" in keys
    assert not any(key.endswith(".mp4") for key in keys)
    assert any(r.uploaded for r in results)

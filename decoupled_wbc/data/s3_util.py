"""Minimal S3 helpers for the episode dataset bucket probe and XR recordings.

Default credentials: ~/.aws profile ``darwin``. ``AWS_PROFILE`` still wins when
set. This module does not print access keys.

Access is proven by HeadObject on ``s3://$DATASET_BUCKET/raw/test.log``.
``AWS_PROFILE`` being set is not enough. Recording paths always write locally
first; a failed put warns and keeps the local file. This module never
CreateBucket.
"""

from configparser import ConfigParser
from dataclasses import dataclass
from datetime import datetime, timezone
import os
from pathlib import Path
import sys
from typing import Optional

import boto3
from botocore.exceptions import ClientError, NoCredentialsError

from decoupled_wbc.data.constants import BUCKET_BASE_PATH

DEFAULT_OBJECT_KEY = "raw/test.log"
DARWIN_PROFILE = "darwin"
PROBE_LINE_MARKER = "g1-vr-teleop s3 probe"
_FALLBACK_REGION = "eu-central-1"
_SKIP_UPLOAD_SUFFIXES = frozenset({".mp4", ".avi", ".mkv", ".mov", ".webm"})


@dataclass(frozen=True)
class AppendResult:
    bucket: str
    key: str
    region: str
    bucket_created: bool
    account_id: str
    previous_line_count: int
    new_line_count: int
    last_line: str
    object_created: bool


@dataclass(frozen=True)
class StoreResult:
    uploaded: bool
    local_path: Path
    s3_uri: Optional[str] = None
    warning: Optional[str] = None
    error_code: Optional[str] = None


def resolve_dataset_bucket() -> str:
    return os.environ.get("DATASET_BUCKET") or BUCKET_BASE_PATH


def _shared_credentials_path() -> Path:
    override = os.environ.get("AWS_SHARED_CREDENTIALS_FILE")
    if override:
        return Path(override)
    return Path.home() / ".aws" / "credentials"


def _credential_file_profiles() -> frozenset[str]:
    path = _shared_credentials_path()
    if not path.is_file():
        return frozenset()
    parser = ConfigParser()
    parser.read(path)
    return frozenset(parser.sections())


def resolve_aws_profile() -> Optional[str]:
    """Return the profile to use. AWS_PROFILE wins; otherwise darwin if present."""
    env_profile = os.environ.get("AWS_PROFILE") or os.environ.get("AWS_DEFAULT_PROFILE")
    if env_profile:
        return env_profile
    if DARWIN_PROFILE in _credential_file_profiles():
        return DARWIN_PROFILE
    return None


def resolve_observability_dir() -> Path:
    """Local fallback root: $G1_VR_LOG_DIR/observability or g1-vr-teleop/logs/observability."""
    root = os.environ.get("G1_VR_LOG_DIR")
    if root:
        return Path(root) / "observability"
    project_root = Path(__file__).resolve().parents[3]
    return project_root / "logs" / "observability"


def _emit_warning(message: str) -> None:
    print(message, flush=True)
    print(message, file=sys.stderr, flush=True)


def _write_local(local_path: Path, body: bytes) -> None:
    local_path.parent.mkdir(parents=True, exist_ok=True)
    local_path.write_bytes(body)


def put_bytes_with_fallback(
    *,
    key: str,
    body: bytes,
    local_path: Path,
    bucket: Optional[str] = None,
) -> StoreResult:
    """Write ``body`` locally, then try S3. Always attempt the put (no creds skip)."""
    local_path = Path(local_path)
    _write_local(local_path, body)
    target_bucket = bucket or resolve_dataset_bucket()
    s3_uri = f"s3://{target_bucket}/{key}"
    try:
        _s3_client(resolve_aws_region()).put_object(
            Bucket=target_bucket, Key=key, Body=body
        )
    except ClientError as exc:
        code = str(exc.response.get("Error", {}).get("Code") or "ClientError")
        warning = f"WARNING: S3 put failed ({code}); storing locally at {local_path}"
        _emit_warning(warning)
        return StoreResult(
            uploaded=False,
            local_path=local_path,
            s3_uri=s3_uri,
            warning=warning,
            error_code=code,
        )
    except Exception as exc:
        code = type(exc).__name__
        warning = f"WARNING: S3 put failed ({code}); storing locally at {local_path}"
        _emit_warning(warning)
        return StoreResult(
            uploaded=False,
            local_path=local_path,
            s3_uri=s3_uri,
            warning=warning,
            error_code=code,
        )
    return StoreResult(uploaded=True, local_path=local_path, s3_uri=s3_uri)


def upload_file_with_fallback(
    *,
    key: str,
    local_path: Path,
    bucket: Optional[str] = None,
) -> StoreResult:
    """Upload an existing local file, streaming from disk.

    Unlike :func:`put_bytes_with_fallback` this never loads the file into memory
    and uses boto3's managed (multipart) transfer, so it is the right call for
    episode videos. The local file is the source of truth and is left untouched;
    a failed upload warns and keeps it.
    """
    local_path = Path(local_path)
    target_bucket = bucket or resolve_dataset_bucket()
    s3_uri = f"s3://{target_bucket}/{key}"
    if not local_path.is_file():
        warning = f"WARNING: nothing to upload, missing {local_path}"
        _emit_warning(warning)
        return StoreResult(
            uploaded=False, local_path=local_path, s3_uri=s3_uri, warning=warning
        )
    try:
        _s3_client(resolve_aws_region()).upload_file(
            str(local_path), target_bucket, key
        )
    except ClientError as exc:
        code = str(exc.response.get("Error", {}).get("Code") or "ClientError")
    except Exception as exc:  # noqa: BLE001 - keep the episode local, never raise
        code = type(exc).__name__
    else:
        return StoreResult(uploaded=True, local_path=local_path, s3_uri=s3_uri)

    warning = f"WARNING: S3 upload failed ({code}); keeping local file {local_path}"
    _emit_warning(warning)
    return StoreResult(
        uploaded=False,
        local_path=local_path,
        s3_uri=s3_uri,
        warning=warning,
        error_code=code,
    )


def try_upload_episode_files(
    *,
    local_root: Path,
    s3_prefix: str,
    bucket: Optional[str] = None,
    include_videos: bool = False,
    only_episode: Optional[int] = None,
) -> list[StoreResult]:
    """Put parquet/meta under ``s3_prefix``.

    Videos are skipped by default (they used to be considered too large); pass
    ``include_videos=True`` to stream them up with the managed transfer instead.
    ``only_episode`` restricts data/video files to that episode index so a
    long session does not re-upload every earlier episode after each save —
    ``meta/`` is always included, since it is rewritten on every save.
    """
    local_root = Path(local_root)
    results: list[StoreResult] = []
    if not local_root.is_dir():
        return results
    prefix = s3_prefix.strip("/")
    episode_stem = None if only_episode is None else f"episode_{only_episode:06d}"
    for path in sorted(local_root.rglob("*")):
        if not path.is_file():
            continue
        is_video = path.suffix.lower() in _SKIP_UPLOAD_SUFFIXES
        if is_video and not include_videos:
            continue
        rel = path.relative_to(local_root).as_posix()
        if episode_stem is not None and not rel.startswith("meta/"):
            if path.stem != episode_stem:
                continue
        key = f"{prefix}/{rel}" if prefix else rel
        if is_video:
            results.append(
                upload_file_with_fallback(key=key, local_path=path, bucket=bucket)
            )
        else:
            results.append(
                put_bytes_with_fallback(
                    key=key, body=path.read_bytes(), local_path=path, bucket=bucket
                )
            )
    return results


def resolve_aws_session() -> boto3.Session:
    profile = resolve_aws_profile()
    if profile:
        return boto3.Session(profile_name=profile)
    return boto3.Session()


def resolve_aws_region() -> str:
    return (
        os.environ.get("AWS_DEFAULT_REGION")
        or os.environ.get("AWS_REGION")
        or resolve_aws_session().region_name
        or _FALLBACK_REGION
    )


def s3_log_accessible(
    *, bucket: Optional[str] = None, key: str = DEFAULT_OBJECT_KEY
) -> bool:
    """True when HeadObject on ``raw/test.log`` succeeds or the object is missing.

    ``AWS_PROFILE`` set is not enough. NoSuchBucket / AccessDenied / network
    errors return False. NoSuchKey still means creds+policy+bucket work.
    """
    target_bucket = bucket or resolve_dataset_bucket()
    try:
        _s3_client(resolve_aws_region()).head_object(Bucket=target_bucket, Key=key)
    except ClientError as exc:
        code = str(exc.response.get("Error", {}).get("Code") or "")
        if code == "NoSuchBucket":
            return False
        if code in ("NoSuchKey", "404", "NotFound") or _is_missing_object(exc):
            return code != "NoSuchBucket"
        return False
    except (NoCredentialsError, OSError, TimeoutError):
        return False
    except Exception:
        return False
    return True


def _s3_client(region: str, session: Optional[boto3.Session] = None):
    sess = session or resolve_aws_session()
    return sess.client("s3", region_name=region)


def _sts_client(region: str, session: Optional[boto3.Session] = None):
    sess = session or resolve_aws_session()
    return sess.client("sts", region_name=region)


def _bucket_exists(client, bucket: str) -> bool:
    try:
        client.head_bucket(Bucket=bucket)
    except ClientError as exc:
        http = exc.response.get("ResponseMetadata", {}).get("HTTPStatusCode")
        code = str(exc.response.get("Error", {}).get("Code", ""))
        if http == 404 or code in ("404", "NoSuchBucket", "NotFound"):
            return False
        raise
    return True


def ensure_bucket(client, bucket: str, region: str) -> bool:
    """Create `bucket` if missing. Return True when this call created it.

    CreateBucket on a bucket that already exists is not a no-op: AWS returns
    BucketAlreadyOwnedByYou or BucketAlreadyExists. Those are treated as exists.
    """
    if _bucket_exists(client, bucket):
        return False
    create_kwargs = {"Bucket": bucket}
    if region != "us-east-1":
        create_kwargs["CreateBucketConfiguration"] = {"LocationConstraint": region}
    try:
        client.create_bucket(**create_kwargs)
    except ClientError as exc:
        code = str(exc.response.get("Error", {}).get("Code", ""))
        if code in ("BucketAlreadyOwnedByYou", "BucketAlreadyExists"):
            return False
        raise
    return True


def _is_missing_object(exc: ClientError) -> bool:
    http = exc.response.get("ResponseMetadata", {}).get("HTTPStatusCode")
    code = str(exc.response.get("Error", {}).get("Code", ""))
    if code == "NoSuchBucket":
        return False
    return http == 404 or code in ("404", "NoSuchKey", "NotFound")


def format_probe_line(when: Optional[datetime] = None) -> str:
    stamp = when or datetime.now(timezone.utc)
    if stamp.tzinfo is None:
        stamp = stamp.replace(tzinfo=timezone.utc)
    utc = stamp.astimezone(timezone.utc)
    return f"{utc.strftime('%Y-%m-%dT%H:%M:%SZ')} {PROBE_LINE_MARKER}"


def count_log_lines(text: str) -> int:
    return len([line for line in text.splitlines() if line != ""])


def get_object_bytes(bucket: str, key: str, region: Optional[str] = None) -> bytes:
    client = _s3_client(region or resolve_aws_region())
    response = client.get_object(Bucket=bucket, Key=key)
    return response["Body"].read()


def get_object_text(
    bucket: str, key: str, region: Optional[str] = None
) -> Optional[str]:
    try:
        return get_object_bytes(bucket, key, region=region).decode("utf-8")
    except ClientError as exc:
        if _is_missing_object(exc):
            return None
        raise


def _join_appended_text(existing: str, line: str) -> str:
    body = existing
    if body and not body.endswith("\n"):
        body += "\n"
    return f"{body}{line}\n"


def append_probe_log(
    *,
    bucket: Optional[str] = None,
    key: str = DEFAULT_OBJECT_KEY,
    line: Optional[str] = None,
) -> AppendResult:
    target_bucket = bucket or resolve_dataset_bucket()
    session = resolve_aws_session()
    region = resolve_aws_region()
    profile = resolve_aws_profile() or "default-chain"
    print(f"s3 probe profile={profile} region={region} bucket={target_bucket}")
    s3 = _s3_client(region, session=session)
    account_id = _sts_client(region, session=session).get_caller_identity()["Account"]
    last_line = line or format_probe_line()
    existing: Optional[str]
    try:
        existing = s3.get_object(Bucket=target_bucket, Key=key)["Body"].read().decode(
            "utf-8"
        )
    except ClientError as exc:
        if not _is_missing_object(exc):
            raise
        existing = None
    previous_text = existing or ""
    previous_count = count_log_lines(previous_text)
    object_created = existing is None
    new_text = _join_appended_text(previous_text, last_line)
    new_count = count_log_lines(new_text)
    print(f"s3 probe put key={key}")
    s3.put_object(Bucket=target_bucket, Key=key, Body=new_text.encode("utf-8"))
    print(
        f"s3 probe previous_line_count={previous_count} "
        f"new_line_count={new_count} last_line={last_line}"
    )
    return AppendResult(
        bucket=target_bucket,
        key=key,
        region=region,
        bucket_created=False,
        account_id=account_id,
        previous_line_count=previous_count,
        new_line_count=new_count,
        last_line=last_line,
        object_created=object_created,
    )

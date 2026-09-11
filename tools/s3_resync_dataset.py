#!/usr/bin/env python3
"""Upload whatever S3 is missing for a local LeRobot dataset.

run_data_exporter.py uploads each episode as it is saved, but an upload can be
cut short (the stack torn down mid-transfer, no network, credentials expired).
The local dataset is always the source of truth, so this catches the bucket up:
it compares every local file against the objects under the dataset's prefix and
uploads the ones that are missing or different.

Episode parquet/mp4 files are compared by size. ``meta/`` files are compared by
content (local MD5 vs the S3 ETag): they are rewritten on every save and an edit
such as dropping the last episode can leave ``info.json`` the same length with
different numbers, which a size check misses. ``--hash-all`` extends the content
check to every file (downloads any multipart-uploaded object to hash it).

Safe to re-run; it never deletes anything and never touches the local files.

    python tools/s3_resync_dataset.py --dataset-path outputs/restocking
    python tools/s3_resync_dataset.py --dataset-path outputs/restocking --dry-run
"""
import argparse
import hashlib
from pathlib import Path
import sys

from decoupled_wbc.data.s3_util import (
    resolve_aws_region,
    resolve_aws_session,
    resolve_dataset_bucket,
    upload_file_with_fallback,
)


def _s3_client():
    return resolve_aws_session().client("s3", region_name=resolve_aws_region())


def list_remote(bucket: str, prefix: str, client=None) -> dict[str, tuple[int, str]]:
    """Return {key: (size, etag)} for everything under ``prefix``.

    The ETag is the object's MD5 for single-part uploads; multipart uploads
    (large videos) carry a ``<md5-of-parts>-<n>`` ETag instead.
    """
    client = client or _s3_client()
    paginator = client.get_paginator("list_objects_v2")
    remote: dict[str, tuple[int, str]] = {}
    for page in paginator.paginate(Bucket=bucket, Prefix=prefix):
        for obj in page.get("Contents", []) or []:
            remote[obj["Key"]] = (obj["Size"], obj.get("ETag", "").strip('"'))
    return remote


def _md5_of(fp) -> str:
    h = hashlib.md5()
    for chunk in iter(lambda: fp.read(1 << 20), b""):
        h.update(chunk)
    return h.hexdigest()


def local_md5(path: Path) -> str:
    with open(path, "rb") as f:
        return _md5_of(f)


def remote_md5(client, bucket: str, key: str, etag: str) -> str:
    """MD5 of an S3 object: the ETag when single-part, else hash the download."""
    if etag and "-" not in etag:
        return etag
    return _md5_of(client.get_object(Bucket=bucket, Key=key)["Body"])


def is_meta_file(rel: str) -> bool:
    return rel.startswith("meta/")


def content_differs(client, bucket: str, key: str, path: Path, etag: str) -> str | None:
    """Reason string when local content != remote content, else None."""
    local = local_md5(path)
    remote = remote_md5(client, bucket, key, etag)
    if local != remote:
        return f"content md5 {remote[:8]} != {local[:8]}"
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--dataset-path", required=True, help="local dataset root, e.g. outputs/restocking"
    )
    ap.add_argument(
        "--bucket", default=None, help="S3 bucket (default: $DATASET_BUCKET / configured)"
    )
    ap.add_argument(
        "--prefix",
        default=None,
        help="S3 prefix (default: raw/<dataset dir name>, matching the exporter)",
    )
    ap.add_argument(
        "--dry-run", action="store_true", help="report what would upload, change nothing"
    )
    ap.add_argument(
        "--hash-all",
        action="store_true",
        help="compare every file by content, not just meta/ (downloads multipart objects)",
    )
    args = ap.parse_args()

    root = Path(args.dataset_path).resolve()
    if not root.is_dir():
        print(f"error: no such dataset directory: {root}", file=sys.stderr)
        return 1

    bucket = args.bucket or resolve_dataset_bucket()
    prefix = (args.prefix or f"raw/{root.name}").strip("/")

    local = {
        path.relative_to(root).as_posix(): path
        for path in sorted(root.rglob("*"))
        if path.is_file()
    }
    if not local:
        print(f"nothing to upload: {root} has no files")
        return 0

    client = _s3_client()
    remote = list_remote(bucket, prefix, client=client)
    print(f"local:  {len(local)} files under {root}")
    print(f"remote: {len(remote)} objects under s3://{bucket}/{prefix}/")

    todo: list[tuple[str, Path, str]] = []
    hashed = 0
    for rel, path in local.items():
        key = f"{prefix}/{rel}"
        size = path.stat().st_size
        if key not in remote:
            todo.append((key, path, "missing"))
            continue
        remote_size, etag = remote[key]
        if remote_size != size:
            todo.append((key, path, f"size {remote_size} != {size}"))
            continue
        if args.hash_all or is_meta_file(rel):
            hashed += 1
            why = content_differs(client, bucket, key, path, etag)
            if why:
                todo.append((key, path, why))
    print(f"compared: {len(local) - hashed} by size, {hashed} by content")

    if not todo:
        print("in sync - nothing to upload")
        return 0

    total_mb = sum(p.stat().st_size for _, p, _ in todo) / 1e6
    print(f"to upload: {len(todo)} files, {total_mb:.1f} MB")
    for key, path, why in todo:
        print(f"  {path.stat().st_size:>10}  {key}  ({why})")
    if args.dry_run:
        print("dry run - nothing uploaded")
        return 0

    failed = 0
    for key, path, _ in todo:
        result = upload_file_with_fallback(key=key, local_path=path, bucket=bucket)
        if result.uploaded:
            print(f"  uploaded {key}")
        else:
            failed += 1
    print(f"done: {len(todo) - failed}/{len(todo)} uploaded")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())

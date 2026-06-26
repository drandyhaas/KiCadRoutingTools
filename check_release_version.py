#!/usr/bin/env python3
"""Fail fast if VERSION, metadata.json (and optionally the release tag) disagree.

Run this BEFORE tagging a release — and it also runs as the first CI gate in
`.github/workflows/release.yml`, ahead of the build matrix:

    python3 check_release_version.py                  # VERSION <-> metadata.json
    python3 check_release_version.py --tag v0.17.0     # also assert tag == vVERSION

It catches the exact half-bumped state that bit v0.17.0: `VERSION` and
`rust_router/Cargo.toml` were bumped but `metadata.json` was left behind, so the
release built all four binaries and then failed in `package_pcm.write_top_level`
("metadata.json has no version entry for 0.17.0"). That check is correct but
runs ~10 minutes too late; this one runs in a second.

`rust_router/Cargo.toml` is intentionally NOT checked here — the Rust crate
version is bumped only when the crate changes and legitimately differs from
`VERSION` in its patch component (see docs/release-pipeline.md).
"""
import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def fail(msg: str) -> None:
    print(f"ERROR: {msg}", file=sys.stderr)
    sys.exit(1)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tag", help="release tag to check against VERSION, e.g. v0.17.0")
    args = ap.parse_args()

    version = (HERE / "VERSION").read_text().strip()

    meta = json.loads((HERE / "metadata.json").read_text())
    versions = [v.get("version") for v in meta.get("versions", [])]
    if version not in versions:
        fail(f"metadata.json has no version entry for {version}; available: {versions}. "
             f"Bump metadata.json's versions[].version (and its download_url) to {version}.")

    # The PCM zip the package job builds is named for VERSION; the version
    # entry's download_url must point at the matching release asset.
    entry = next(v for v in meta["versions"] if v.get("version") == version)
    expected = f"/releases/download/v{version}/KiCadRoutingTools-{version}.zip"
    url = entry.get("download_url", "")
    if not url.endswith(expected):
        fail(f"metadata.json download_url for {version} should end with {expected!r}, "
             f"got {url!r}. Update the download_url path to v{version}.")

    if args.tag is not None and args.tag != f"v{version}":
        fail(f"tag {args.tag} does not match VERSION {version} (expected v{version}).")

    print(f"OK: VERSION={version} matches metadata.json"
          + (f" and tag {args.tag}" if args.tag else "") + ".")


if __name__ == "__main__":
    main()

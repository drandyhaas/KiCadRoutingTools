#!/usr/bin/env python3
"""
Patch metadata.json with download_url / sha256 / size values from
package_pcm.py's sidecar files.

Each sidecar (`dist/KiCadRoutingTools-<ver>-<plat>.zip.meta.json`) contains
{platform, version, filename, download_sha256, download_size, install_size}.
This script finds the matching version entry in metadata.json (by version
string + platforms containing the platform) and fills it in. The
download_url is set to the GitHub Releases URL for the same tag.

Usage:
    python update_metadata.py --sidecars dist/*.meta.json \
        --repo drandyhaas/KiCadRoutingTools
"""

import argparse
import glob
import json
import sys
from pathlib import Path


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--metadata", default="metadata.json")
    p.add_argument("--sidecars", nargs="+", required=True,
                   help="Sidecar JSON files (globs allowed)")
    p.add_argument("--repo", required=True,
                   help="GitHub repo owner/name for download_url")
    args = p.parse_args()

    sidecars = []
    for pattern in args.sidecars:
        sidecars.extend(glob.glob(pattern))
    if not sidecars:
        print(f"ERROR: no sidecars matched {args.sidecars}", file=sys.stderr)
        return 1

    meta_path = Path(args.metadata)
    meta = json.loads(meta_path.read_text())

    updated = 0
    for s in sidecars:
        info = json.loads(Path(s).read_text())
        platform = info["platform"]
        version = info["version"]
        url = (
            f"https://github.com/{args.repo}/releases/download/"
            f"v{version}/{info['filename']}"
        )
        match = None
        for entry in meta["versions"]:
            if entry.get("version") != version:
                continue
            if platform in entry.get("platforms", []):
                match = entry
                break
        if match is None:
            print(f"ERROR: no metadata.json version entry for "
                  f"version={version} platform={platform}", file=sys.stderr)
            return 1
        match["download_url"] = url
        match["download_sha256"] = info["download_sha256"]
        match["download_size"] = info["download_size"]
        match["install_size"] = info["install_size"]
        updated += 1
        print(f"  patched {platform} v{version}: sha256={info['download_sha256'][:12]}…, "
              f"size={info['download_size']}")

    meta_path.write_text(json.dumps(meta, indent=2) + "\n")
    print(f"\nUpdated {updated} entries in {meta_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
Patch the matching version entry in metadata.json with download_url,
download_sha256, download_size, and install_size from package_pcm.py's
sidecar JSON.

Usage:
    python update_metadata.py --sidecar dist/KiCadRoutingTools-0.15.5.zip.meta.json \
        --repo drandyhaas/KiCadRoutingTools
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': [], 'kind': 'utility'}

import argparse
import json
import sys
from pathlib import Path


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--metadata", default="metadata.json")
    p.add_argument("--sidecar", required=True)
    p.add_argument("--repo", required=True,
                   help="GitHub repo owner/name for download_url")
    args = p.parse_args()

    info = json.loads(Path(args.sidecar).read_text())
    meta_path = Path(args.metadata)
    meta = json.loads(meta_path.read_text())

    version = info["version"]
    url = (f"https://github.com/{args.repo}/releases/download/"
           f"v{version}/{info['filename']}")

    match = next((v for v in meta["versions"] if v.get("version") == version),
                 None)
    if match is None:
        print(f"ERROR: no metadata.json version entry for {version}",
              file=sys.stderr)
        return 1

    match["download_url"] = url
    match["download_sha256"] = info["download_sha256"]
    match["download_size"] = info["download_size"]
    match["install_size"] = info["install_size"]

    meta_path.write_text(json.dumps(meta, indent=2) + "\n")
    print(f"Patched v{version}: sha256={info['download_sha256'][:12]}…, "
          f"size={info['download_size']}, install={info['install_size']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

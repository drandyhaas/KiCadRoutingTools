#!/usr/bin/env python3
"""Build the independent KiCad Track Gloss PCM archive."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import zipfile


ROOT = Path(__file__).resolve().parent
REPO = ROOT.parent
VERSION = "0.3.3"
RUNTIME_FILES = (
    "__init__.py", "action_plugin.py", "board_adapter.py", "connectivity.py",
    "geometry.py", "gloss_engine.py",
    "model.py", "icon_24.png", "icon_24_dark.png", "LICENSE", "NOTICE", "README.md",
)


def build(output_dir):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    out = output_dir / f"KiCadTrackGloss-{VERSION}.zip"
    with tempfile.TemporaryDirectory(prefix="track-gloss-pcm-") as tmp_name:
        tmp = Path(tmp_name)
        plugins = tmp / "plugins"
        plugins.mkdir(parents=True)
        for name in RUNTIME_FILES:
            source = ROOT / name
            if not source.exists():
                raise FileNotFoundError(source)
            # PCM installs the contents of this directory as one plugin package.
            # KiCad explicitly forbids a second package-directory level here.
            shutil.copy2(source, plugins / name)
        shutil.copy2(ROOT / "metadata.json", tmp / "metadata.json")
        resources = tmp / "resources"
        resources.mkdir()
        shutil.copy2(ROOT / "icon_64.png", resources / "icon.png")
        if out.exists():
            out.unlink()
        with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
            for path in sorted(tmp.rglob("*")):
                if path.is_file():
                    archive.write(path, path.relative_to(tmp).as_posix())
    digest = hashlib.sha256(out.read_bytes()).hexdigest()
    sidecar = out.with_suffix(".meta.json")
    sidecar.write_text(json.dumps({"version": VERSION, "sha256": digest,
                                   "download_size": out.stat().st_size}, indent=2) + "\n",
                       encoding="utf-8")
    print(out)
    print("sha256=" + digest)
    return out


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default=str(REPO / "dist"))
    args = parser.parse_args()
    build(args.output_dir)

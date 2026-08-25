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

try:
    from .version import __version__
except ImportError:
    from version import __version__


ROOT = Path(__file__).resolve().parent
REPO = ROOT.parent
VERSION = __version__
RUNTIME_FILES = (
    "__init__.py", "action_plugin.py", "icon_24.png", "icon_24_dark.png",
    "version.py", "LICENSE", "NOTICE", "README.md",
)
RUNTIME_DIRECTORIES = ("engine", "kicad")


def build(output_dir, release_tag=None):
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
            # The ActionPlugin entry point stays directly under plugins/.
            # Internal engine/ and kicad/ support packages are copied below.
            shutil.copy2(source, plugins / name)
        for name in RUNTIME_DIRECTORIES:
            source = ROOT / name
            if not source.is_dir():
                raise FileNotFoundError(source)
            shutil.copytree(source, plugins / name,
                            ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
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
    with zipfile.ZipFile(out) as archive:
        install_size = sum(item.file_size for item in archive.infolist())
    sidecar = out.with_suffix(".meta.json")
    sidecar.write_text(json.dumps({"version": VERSION, "sha256": digest,
                                   "download_size": out.stat().st_size,
                                   "install_size": install_size}, indent=2) + "\n",
                       encoding="utf-8")
    release_tag = release_tag or "v{}-alpha".format(VERSION)
    submission = json.loads((ROOT / "metadata.json").read_text(encoding="utf-8"))
    release = submission["versions"][0]
    release.update({
        "download_sha256": digest,
        "download_size": out.stat().st_size,
        "install_size": install_size,
        "download_url": (
            "https://github.com/fca1/KiCadRoutingTools/releases/download/"
            "{}/{}".format(release_tag, out.name)),
    })
    official = (output_dir / "kicad-official" / "packages" /
                submission["identifier"])
    official.mkdir(parents=True, exist_ok=True)
    (official / "metadata.json").write_text(
        json.dumps(submission, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8")
    shutil.copy2(ROOT / "icon_64.png", official / "icon.png")
    print(out)
    print("sha256=" + digest)
    print("official_metadata=" + str(official / "metadata.json"))
    return out


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default=str(REPO / "dist"))
    parser.add_argument(
        "--release-tag", default=None,
        help="public GitHub release tag used by official PCM metadata")
    args = parser.parse_args()
    build(args.output_dir, args.release_tag)

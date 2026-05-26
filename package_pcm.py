#!/usr/bin/env python3
"""
Build a KiCad PCM (Plugin and Content Manager) zip for this project.

The KiCad PCM expects a zip with this layout (see go.kicad.org/pcm/schemas/v1):

    metadata.json
    resources/icon.png
    plugins/__init__.py
    plugins/...everything else...

This script stages the existing plugin tree into a `plugins/` folder, drops
in the platform-specific Rust binary (downloaded from the GitHub Release),
and zips it up alongside metadata.json and resources/icon.png.

Usage:
    python package_pcm.py --platform linux
    python package_pcm.py --platform macos
    python package_pcm.py --platform windows

    # Use already-downloaded binaries from a directory instead of fetching:
    python package_pcm.py --platform macos --binary-dir ./artifacts

    # Set the package version (default: read from VERSION file)
    python package_pcm.py --platform linux --version 0.15.4

The output zip is `dist/KiCadRoutingTools-<version>-<platform>.zip`. The
script also prints sha256, download_size, and install_size so they can be
patched into metadata.json before submission.
"""

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import urllib.request
import zipfile
from pathlib import Path


SCRIPT_DIR = Path(__file__).parent.resolve()
GITHUB_REPO = "drandyhaas/KiCadRoutingTools"

# Files at the repo root that ship inside plugins/. Anything not listed is
# excluded. Mirrors install_plugin.copy_plugin's ignore logic but explicit.
PLUGIN_FILES_KEEP = {
    "__init__.py",
    "VERSION",
    "LICENSE",
    "requirements.txt",
}
# Directories at the repo root to include under plugins/.
PLUGIN_DIRS_KEEP = {
    "kicad_routing_plugin",
    "rust_router",  # binary is placed here separately
}
# Plus every *.py at the repo root that isn't this script or build/install
# scripts.
ROOT_SCRIPTS_EXCLUDE = {
    "build_router.py",
    "install_plugin.py",
    "package_pcm.py",
}


# Per-platform: list of release-asset filenames to bundle into rust_router/.
# Linux/Windows: a single binary, renamed to the canonical name so
# `import grid_router` works without any resolver. macOS: ship both arm64 and
# x86_64 binaries under their platform-suffix names; the root __init__.py
# resolves which to use at startup.
PLATFORM_BINARIES = {
    "linux": [("grid_router-linux-x86_64.so", "grid_router.so")],
    "windows": [("grid_router-windows-x86_64.pyd", "grid_router.pyd")],
    "macos": [
        ("grid_router-macos-arm64.so", "grid_router-macos-arm64.so"),
        ("grid_router-macos-x86_64.so", "grid_router-macos-x86_64.so"),
    ],
}


def read_version():
    return (SCRIPT_DIR / "VERSION").read_text().strip()


def _is_py_module(name):
    return name.endswith(".py") and name not in ROOT_SCRIPTS_EXCLUDE


def stage_plugins(stage_root: Path):
    """Copy plugin source into <stage_root>/plugins/."""
    plugins_dir = stage_root / "plugins"
    plugins_dir.mkdir(parents=True, exist_ok=True)

    # Root-level files
    for name in PLUGIN_FILES_KEEP:
        src = SCRIPT_DIR / name
        if src.exists():
            shutil.copy2(src, plugins_dir / name)

    # All root-level python modules except the build/install scripts
    for entry in SCRIPT_DIR.iterdir():
        if entry.is_file() and _is_py_module(entry.name):
            shutil.copy2(entry, plugins_dir / entry.name)

    # Subdirectories we keep
    for dirname in PLUGIN_DIRS_KEEP:
        src = SCRIPT_DIR / dirname
        if not src.is_dir():
            continue
        dst = plugins_dir / dirname
        shutil.copytree(
            src,
            dst,
            ignore=shutil.ignore_patterns(
                "__pycache__", "*.pyc", ".DS_Store",
                "target",  # rust_router/target/ is the cargo build dir
                "grid_router.so", "grid_router.pyd", "grid_router.abi3.so",
                "Cargo.lock",
            ),
        )

    return plugins_dir


def _download_asset(version: str, asset_name: str, dest: Path):
    url = (
        f"https://github.com/{GITHUB_REPO}/releases/download/"
        f"v{version}/{asset_name}"
    )
    print(f"  Downloading {url}")
    req = urllib.request.Request(url, headers={"User-Agent": "package_pcm.py"})
    with urllib.request.urlopen(req, timeout=120) as resp, open(dest, "wb") as f:
        shutil.copyfileobj(resp, f)


def install_binary(plugins_dir: Path, platform_key: str, version: str,
                   binary_dir: Path | None):
    """Place the platform-specific Rust binary into plugins/rust_router/."""
    rust_dir = plugins_dir / "rust_router"
    rust_dir.mkdir(parents=True, exist_ok=True)
    for asset_name, install_name in PLATFORM_BINARIES[platform_key]:
        src: Path
        if binary_dir is not None:
            src = binary_dir / asset_name
            if not src.exists():
                raise FileNotFoundError(
                    f"Expected {src} (asset name from release). Use --binary-dir "
                    f"with the GitHub Release artifacts, or omit it to download."
                )
            shutil.copy2(src, rust_dir / install_name)
        else:
            _download_asset(version, asset_name, rust_dir / install_name)


def write_top_level(stage_root: Path):
    """Copy metadata.json and resources/icon.png into the staging root."""
    shutil.copy2(SCRIPT_DIR / "metadata.json", stage_root / "metadata.json")

    resources = stage_root / "resources"
    resources.mkdir(exist_ok=True)
    # PCM requires 64x64; we already ship icon_64.png in kicad_routing_plugin/.
    icon_src = SCRIPT_DIR / "kicad_routing_plugin" / "icon_64.png"
    if not icon_src.exists():
        raise FileNotFoundError(f"Missing icon: {icon_src}")
    shutil.copy2(icon_src, resources / "icon.png")


def make_zip(stage_root: Path, out_zip: Path):
    """Zip the staging tree to out_zip with stable mtimes."""
    if out_zip.exists():
        out_zip.unlink()
    install_size = 0
    with zipfile.ZipFile(out_zip, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as zf:
        for path in sorted(stage_root.rglob("*")):
            rel = path.relative_to(stage_root)
            if path.is_dir():
                continue
            install_size += path.stat().st_size
            zf.write(path, rel.as_posix())
    return install_size


def sha256_of(path: Path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--platform", required=True,
                        choices=sorted(PLATFORM_BINARIES.keys()))
    parser.add_argument("--version", default=None,
                        help="Override version (default: read VERSION file)")
    parser.add_argument("--binary-dir", default=None,
                        help="Directory containing pre-fetched release assets")
    parser.add_argument("--out-dir", default=str(SCRIPT_DIR / "dist"),
                        help="Output directory (default: ./dist)")
    args = parser.parse_args()

    version = args.version or read_version()
    binary_dir = Path(args.binary_dir).resolve() if args.binary_dir else None
    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    zip_name = f"KiCadRoutingTools-{version}-{args.platform}.zip"
    out_zip = out_dir / zip_name

    print(f"Building PCM package: {zip_name}")
    with tempfile.TemporaryDirectory(prefix="kicadrt-pcm-") as tmp:
        stage_root = Path(tmp)
        plugins_dir = stage_plugins(stage_root)
        install_binary(plugins_dir, args.platform, version, binary_dir)
        write_top_level(stage_root)
        install_size = make_zip(stage_root, out_zip)

    download_size = out_zip.stat().st_size
    digest = sha256_of(out_zip)

    print()
    print(f"  Output:        {out_zip}")
    print(f"  download_sha256: {digest}")
    print(f"  download_size:   {download_size}")
    print(f"  install_size:    {install_size}")

    # Emit a JSON sidecar so CI can patch metadata.json deterministically.
    sidecar = out_dir / f"{zip_name}.meta.json"
    sidecar.write_text(json.dumps({
        "platform": args.platform,
        "version": version,
        "filename": zip_name,
        "download_sha256": digest,
        "download_size": download_size,
        "install_size": install_size,
    }, indent=2) + "\n")
    print(f"  sidecar:       {sidecar}")


if __name__ == "__main__":
    main()

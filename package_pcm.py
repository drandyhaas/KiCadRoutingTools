#!/usr/bin/env python3
"""
Build a KiCad PCM (Plugin and Content Manager) zip for this project.

The KiCad PCM expects a zip with this layout (see go.kicad.org/pcm/schemas/v1):

    metadata.json
    resources/icon.png
    plugins/__init__.py
    plugins/...everything else...

KiCad PCM's metadata schema rejects multiple `versions[]` entries with the
same `version` string, so we ship ONE zip per version that bundles binaries
for all supported platforms (linux x86_64, macOS arm64, macOS x86_64, windows
x86_64) under platform-suffix filenames. The plugin's root __init__.py
picks the right binary at startup.

The metadata.json inside the zip is also constrained: it must have exactly
one version entry and no download_* fields. This script generates that
"inner" metadata from the repository-style metadata.json automatically.

Usage:
    # Use already-downloaded binaries from a directory:
    python package_pcm.py --binary-dir ./artifacts

    # Or download all 4 binaries from the matching GitHub Release:
    python package_pcm.py

    # Override the version (default: read from VERSION file)
    python package_pcm.py --version 0.15.5

Output: dist/KiCadRoutingTools-<version>.zip plus a .meta.json sidecar
containing sha256/download_size/install_size for metadata.json patching.
"""

import argparse
import copy
import hashlib
import json
import os
import shutil
import sys
import tempfile
import urllib.request
import zipfile
from pathlib import Path


SCRIPT_DIR = Path(__file__).parent.resolve()
GITHUB_REPO = "drandyhaas/KiCadRoutingTools"

# The plugin ships the whole repo working tree minus the entries below, so a
# PCM install contains the same code as git: the GUI runs, and the command-line
# tools work straight from the installed plugin directory. Using a denylist
# (rather than an allowlist of files/dirs) means new modules and packages ship
# automatically -- forgetting to allowlist a package was issue #49.
ROOT_EXCLUDE = {
    # Build / release / install tooling: not needed at runtime.
    "build_router.py",
    "install_plugin.py",
    "package_pcm.py",
    "update_metadata.py",
    # The repo metadata.json is rewritten to the zip root separately by
    # write_top_level(); it must not also appear under plugins/.
    "metadata.json",
    # Dev-only directories.
    "tests",
    "docs",
    "kicad_files",  # sample boards: large and unnecessary at runtime
    # Project / internal notes that aren't shipped to users.
    "CLAUDE.md",
}

# Names skipped anywhere in the tree (VCS, caches, build artifacts). The Rust
# crate's build dir and the per-platform binaries are stripped here; the right
# binary is added back under platform-suffix names by install_binaries().
IGNORE_PATTERNS = shutil.ignore_patterns(
    ".git", ".github", ".claude", ".gitignore", ".DS_Store",
    "__pycache__", "*.pyc", "dist",
    "target",  # rust_router/target/ is the cargo build dir
    "grid_router.so", "grid_router.pyd", "grid_router.abi3.so",
    "Cargo.lock",
)

# All binaries bundled in every PCM zip. The startup resolver in the root
# __init__.py picks the right one based on sys.platform + machine.
ALL_BINARIES = [
    "grid_router-linux-x86_64.so",
    "grid_router-macos-arm64.so",
    "grid_router-macos-x86_64.so",
    "grid_router-windows-x86_64.pyd",
]


def read_version():
    return (SCRIPT_DIR / "VERSION").read_text().strip()


def stage_plugins(stage_root: Path):
    """Copy the repo working tree (minus ROOT_EXCLUDE / IGNORE_PATTERNS) into
    <stage_root>/plugins/."""
    plugins_dir = stage_root / "plugins"
    plugins_dir.mkdir(parents=True, exist_ok=True)

    ignored_names = set(ROOT_EXCLUDE)
    for entry in sorted(SCRIPT_DIR.iterdir()):
        if entry.name in ignored_names:
            continue
        # IGNORE_PATTERNS handles dotfiles/caches uniformly at the top level too.
        if IGNORE_PATTERNS(str(SCRIPT_DIR), [entry.name]):
            continue
        dst = plugins_dir / entry.name
        if entry.is_dir():
            shutil.copytree(entry, dst, ignore=IGNORE_PATTERNS)
        else:
            shutil.copy2(entry, dst)

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


def install_binaries(plugins_dir: Path, version: str, binary_dir: Path | None):
    """Place ALL platform Rust binaries into plugins/rust_router/ under their
    platform-suffix names. The plugin's startup resolver picks the right one.
    """
    rust_dir = plugins_dir / "rust_router"
    rust_dir.mkdir(parents=True, exist_ok=True)
    for name in ALL_BINARIES:
        dst = rust_dir / name
        if binary_dir is not None:
            src = binary_dir / name
            if not src.exists():
                raise FileNotFoundError(f"Expected {src}")
            shutil.copy2(src, dst)
        else:
            _download_asset(version, name, dst)


def write_top_level(stage_root: Path, version: str):
    """Place metadata.json (stripped to one version, no download_* fields)
    and resources/icon.png at the zip root.
    """
    repo_meta = json.loads((SCRIPT_DIR / "metadata.json").read_text())

    # Find the version entry that matches the package version, then strip it
    # of download_* fields. The PCM repo's validator requires the in-package
    # metadata.json to have exactly one version entry and no download_*.
    matches = [v for v in repo_meta["versions"] if v.get("version") == version]
    if not matches:
        raise ValueError(
            f"metadata.json has no version entry for {version}; "
            f"available: {[v.get('version') for v in repo_meta['versions']]}"
        )
    inner_version = copy.deepcopy(matches[0])
    for key in ("download_url", "download_sha256", "download_size", "install_size"):
        inner_version.pop(key, None)

    inner_meta = copy.deepcopy(repo_meta)
    inner_meta["versions"] = [inner_version]

    (stage_root / "metadata.json").write_text(
        json.dumps(inner_meta, indent=2) + "\n"
    )

    resources = stage_root / "resources"
    resources.mkdir(exist_ok=True)
    icon_src = SCRIPT_DIR / "kicad_routing_plugin" / "icon_64.png"
    if not icon_src.exists():
        raise FileNotFoundError(f"Missing icon: {icon_src}")
    shutil.copy2(icon_src, resources / "icon.png")


def make_zip(stage_root: Path, out_zip: Path):
    """Zip the staging tree."""
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
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
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

    zip_name = f"KiCadRoutingTools-{version}.zip"
    out_zip = out_dir / zip_name

    print(f"Building PCM package: {zip_name}")
    with tempfile.TemporaryDirectory(prefix="kicadrt-pcm-") as tmp:
        stage_root = Path(tmp)
        plugins_dir = stage_plugins(stage_root)
        install_binaries(plugins_dir, version, binary_dir)
        write_top_level(stage_root, version)
        install_size = make_zip(stage_root, out_zip)

    download_size = out_zip.stat().st_size
    digest = sha256_of(out_zip)

    print()
    print(f"  Output:        {out_zip}")
    print(f"  download_sha256: {digest}")
    print(f"  download_size:   {download_size}")
    print(f"  install_size:    {install_size}")

    sidecar = out_dir / f"{zip_name}.meta.json"
    sidecar.write_text(json.dumps({
        "version": version,
        "filename": zip_name,
        "download_sha256": digest,
        "download_size": download_size,
        "install_size": install_size,
    }, indent=2) + "\n")
    print(f"  sidecar:       {sidecar}")


if __name__ == "__main__":
    main()

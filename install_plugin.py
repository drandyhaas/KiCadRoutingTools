#!/usr/bin/env python3
"""
KiCad Routing Tools Plugin Installer (KiCad 10 IPC)

Drops the plugin source tree (or a symlink) into KiCad 10's user plugins
directory so it shows up in the toolbar / action menu. KiCad 10 provisions
a per-plugin Python venv from `requirements.txt` automatically on first
action invocation, so this installer no longer pip-installs anything.

Usage:
    python install_plugin.py [--uninstall] [--symlink] [--keep-pcm]

Options:
    --uninstall    Remove the plugin instead of installing
    --symlink      Symlink the source tree (for development; edits take
                   effect on the next action invocation without re-copy)
    --keep-pcm     Don't disable conflicting PCM copies of this plugin

On install, any copy of this plugin previously installed through KiCad's Plugin &
Content Manager is detected (it would sit next to the local install in the
plugins directory and shadow it on sys.path) and moved aside, unless --keep-pcm
is given.
"""
from __future__ import annotations

import os
import sys
import json
import shutil
import platform
import argparse
from pathlib import Path
from typing import Optional

from startup_checks import get_cargo_version


PLUGIN_NAME = "KiCadRoutingTools"
PLUGIN_DISPLAY_NAME = "KiCad Routing Tools"
PLUGIN_IDENTIFIER = "com.github.drandyhaas.kicadroutingtools"
KICAD_VERSIONS = ["10.0"]  # IPC plugins are KiCad 10+ only
# PCM (Plugin & Content Manager) package identifier from metadata.json. A PCM
# install lands in the plugins directory under <identifier-with-dots-as-
# underscores>/ next to our dev install, putting the same top-level modules on
# sys.path and shadowing the local version. We detect and disable such copies.
PCM_IDENTIFIER = PLUGIN_IDENTIFIER


def get_kicad_base_dir() -> Path:
    """Get the KiCad documents directory for the current platform."""
    system = platform.system()

    if system == "Linux":
        return Path.home() / ".local" / "share" / "kicad"
    elif system == "Darwin":  # macOS
        return Path.home() / "Documents" / "KiCad"
    elif system == "Windows":
        # On modern Windows, Documents is usually redirected into OneDrive.
        # Try the env vars OneDrive sets, then ~/OneDrive/Documents, then
        # plain ~/Documents. Return the first that exists; if none do, fall
        # back to the OneDrive path so a fresh install lands somewhere KiCad
        # will pick up (KiCad itself follows the same redirection).
        onedrive = (
            os.environ.get("OneDrive")
            or os.environ.get("OneDriveConsumer")
            or os.environ.get("OneDriveCommercial")
        )
        candidates = []
        if onedrive:
            candidates.append(Path(onedrive) / "Documents" / "KiCad")
        candidates.extend([
            Path.home() / "OneDrive" / "Documents" / "KiCad",
            Path.home() / "Documents" / "KiCad",
        ])
        for candidate in candidates:
            if candidate.exists():
                return candidate
        return candidates[0]
    else:
        raise RuntimeError(f"Unsupported operating system: {system}")


def get_kicad_plugin_dirs() -> list:
    """Get KiCad 10+ plugin directories that exist on this machine."""
    base_dir = get_kicad_base_dir()
    plugin_dirs = []

    for version in KICAD_VERSIONS:
        version_dir = base_dir / version
        if platform.system() == "Windows":
            install_dir = Path(rf"C:\Program Files\KiCad\{version}")
        else:
            install_dir = version_dir
        if install_dir.exists() or version_dir.exists():
            plugin_dir = version_dir / "plugins"
            plugin_dirs.append((version, plugin_dir))

    return plugin_dirs


def get_source_dir() -> Path:
    """Get the source directory (KiCadRoutingTools root)."""
    return Path(__file__).parent.resolve()


def check_rust_router() -> bool:
    """Verify the Rust router is built and its version matches Cargo.toml."""
    source_dir = get_source_dir()
    rust_dir = source_dir / "rust_router"

    cargo_version = get_cargo_version()
    if cargo_version is None:
        print("  Warning: Could not read version from Cargo.toml")
        return False

    if str(rust_dir) not in sys.path:
        sys.path.insert(0, str(rust_dir))

    try:
        import grid_router
        installed_version = getattr(grid_router, '__version__', 'unknown')
    except ImportError:
        print("  Error: Rust router module not found")
        print("  Please build it first with:")
        print("    python build_router.py")
        return False

    if installed_version != cargo_version:
        print(f"  Error: Rust router version mismatch")
        print(f"    Installed: {installed_version}")
        print(f"    Expected:  {cargo_version}")
        print("  Please rebuild with:")
        print("    python build_router.py")
        return False

    print(f"  Rust router v{installed_version} OK")
    return True


def copy_plugin(source_dir: Path, dest_dir: Path):
    """Copy the entire KiCadRoutingTools directory to destination."""
    # Remove existing installation if present. Handle symlinks separately:
    # a prior `--symlink` dev install leaves a symlink here, and shutil.rmtree
    # refuses to operate on a symlink (raises an opaque "[Errno None] None").
    if dest_dir.is_symlink():
        print(f"  Removing existing symlink at {dest_dir}")
        dest_dir.unlink()
    elif dest_dir.exists():
        print(f"  Removing existing installation at {dest_dir}")
        shutil.rmtree(dest_dir)

    # Copy everything, excluding unnecessary files
    def ignore_patterns(_directory, files):
        ignored = []
        for f in files:
            # Keep .claude/ (routing skills) for use from the installed plugin.
            if f == '.claude':
                continue
            # Skip hidden files, cache, and non-essential directories
            if f.startswith('.') or f == '__pycache__' or f.endswith('.pyc'):
                ignored.append(f)
            elif f.startswith('test_') or f == 'docs':
                ignored.append(f)
            elif f == 'kicad_files':
                ignored.append(f)
        return ignored

    print(f"  Copying to {dest_dir}")
    shutil.copytree(source_dir, dest_dir, ignore=ignore_patterns)
    print("  Done")


def create_symlink(source_dir: Path, dest_dir: Path):
    """Create a symlink to the plugin source (for development)."""
    if dest_dir.exists() or dest_dir.is_symlink():
        if dest_dir.is_symlink():
            dest_dir.unlink()
        else:
            shutil.rmtree(dest_dir)

    print(f"  Creating symlink: {dest_dir} -> {source_dir}")
    dest_dir.symlink_to(source_dir)


def uninstall_plugin(dest_dir: Path):
    """Remove the plugin installation."""
    if dest_dir.exists() or dest_dir.is_symlink():
        if dest_dir.is_symlink():
            dest_dir.unlink()
            print(f"  Removed symlink: {dest_dir}")
        else:
            shutil.rmtree(dest_dir)
            print(f"  Removed directory: {dest_dir}")
    else:
        print(f"  Plugin not installed at {dest_dir}")


def _dir_is_our_pcm_plugin(d: Path) -> bool:
    """True if directory `d` is a PCM-installed copy of THIS plugin.

    Identified authoritatively by a metadata.json declaring our PCM identifier,
    falling back to the PCM directory-naming convention (identifier with dots
    replaced by underscores) if the metadata file is missing.
    """
    meta = d / "metadata.json"
    if meta.is_file():
        try:
            data = json.loads(meta.read_text(encoding="utf-8"))
            if data.get("identifier") == PCM_IDENTIFIER:
                return True
        except (OSError, ValueError):
            pass
    return d.name == PCM_IDENTIFIER.replace(".", "_")


def find_conflicting_pcm_installs(plugins_dir: Path) -> list:
    """Find PCM-installed copies of this plugin under `plugins_dir`.

    Any PCM copy shares our top-level module names (route.py, obstacle_map.py,
    ...) on sys.path and will shadow the locally-installed version, so every PCM
    copy is treated as a conflict. Our own install (named PLUGIN_NAME) is skipped.
    """
    if not plugins_dir.is_dir():
        return []
    conflicts = []
    # Skip our own dev install. On IPC it's named by the identifier
    # (PLUGIN_IDENTIFIER); the legacy SWIG name is kept for safety.
    own_names = {PLUGIN_NAME, PLUGIN_IDENTIFIER}
    for child in sorted(plugins_dir.iterdir()):
        if not child.is_dir() or child.name in own_names:
            continue
        if _dir_is_our_pcm_plugin(child):
            conflicts.append(child)
    return conflicts


def disable_pcm_install(pcm_dir: Path, version: str) -> Optional[Path]:
    """Move a conflicting PCM plugin copy out of the plugin search path.

    Moved to <kicad_base>/disabled_pcm_plugins/<version>/ so it leaves sys.path
    (resolving the conflict) but stays recoverable. Returns the backup path, or
    None if the move failed.
    """
    backup_root = get_kicad_base_dir() / "disabled_pcm_plugins" / version
    try:
        backup_root.mkdir(parents=True, exist_ok=True)
        dest = backup_root / pcm_dir.name
        counter = 1
        while dest.exists():
            dest = backup_root / f"{pcm_dir.name}.{counter}"
            counter += 1
        shutil.move(str(pcm_dir), str(dest))
        return dest
    except OSError as e:
        print(f"    Error: could not move PCM copy aside: {e}")
        print(f"    Please remove it manually via KiCad's Plugin & Content Manager,")
        print(f"    or delete: {pcm_dir}")
        return None


def handle_pcm_conflicts(plugins_dir: Path, version: str) -> int:
    """Detect and disable PCM copies of this plugin that would shadow our install.

    Returns the number of conflicting copies disabled.
    """
    conflicts = find_conflicting_pcm_installs(plugins_dir)
    if not conflicts:
        return 0
    print(f"  Found {len(conflicts)} PCM-installed copy(ies) of this plugin that "
          f"would shadow the local install:")
    disabled = 0
    for pcm_dir in conflicts:
        print(f"    - {pcm_dir}")
        backup = disable_pcm_install(pcm_dir, version)
        if backup is not None:
            print(f"      Disabled (moved to {backup})")
            disabled += 1
    if disabled:
        print(f"  Note: the Plugin & Content Manager may still list this package as")
        print(f"        installed; you can formally remove it there (Plugins -> Manage")
        print(f"        -> Uninstall), or just delete the backup folder above.")
    return disabled


def main():
    parser = argparse.ArgumentParser(
        description=f"Install {PLUGIN_DISPLAY_NAME} plugin for KiCad 10+ (IPC)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python install_plugin.py              # Install the plugin
    python install_plugin.py --symlink    # Symlink (for development)
    python install_plugin.py --uninstall  # Remove the plugin
"""
    )
    parser.add_argument("--uninstall", "-u", action="store_true",
                        help="Remove the plugin instead of installing")
    parser.add_argument("--symlink", "-s", action="store_true",
                        help="Create symlink instead of copying (for development)")
    parser.add_argument("--keep-pcm", action="store_true",
                        help="Don't disable conflicting PCM (Plugin & Content Manager) copies of this plugin")

    args = parser.parse_args()

    print(f"\n{PLUGIN_DISPLAY_NAME} Installer (IPC / KiCad 10)")
    print("=" * 50)

    try:
        source_dir = get_source_dir()
        plugin_dirs = get_kicad_plugin_dirs()
    except Exception as e:
        print(f"Error: {e}")
        return 1

    print(f"Platform: {platform.system()}")
    print(f"Source: {source_dir}")
    print()

    if not plugin_dirs:
        print("Error: no supported KiCad version (10.0+) found on this system.")
        return 1

    versions_found = [v for v, _ in plugin_dirs]
    print(f"KiCad versions found: {', '.join(versions_found)}")
    print()

    if not args.uninstall:
        print("Checking Rust router...")
        if not check_rust_router():
            return 1
        print()

    installed_count = 0
    for version, dest_base in plugin_dirs:
        dest_dir = dest_base / PLUGIN_IDENTIFIER
        print(f"KiCad {version}:")
        print(f"  Destination: {dest_dir}")

        if args.uninstall:
            uninstall_plugin(dest_dir)
            continue

        # Disable any PCM-installed copy that would shadow this install
        if not args.keep_pcm:
            handle_pcm_conflicts(dest_base, version)

        # Create destination directory if needed
        if not dest_base.exists():
            print(f"  Creating plugins directory: {dest_base}")
            dest_base.mkdir(parents=True, exist_ok=True)

        try:
            if args.symlink:
                create_symlink(source_dir, dest_dir)
            else:
                copy_plugin(source_dir, dest_dir)
            installed_count += 1
        except Exception as e:
            print(f"  Error installing plugin: {e}")

        print()

    print("=" * 50)
    if args.uninstall:
        print("Uninstall complete!")
    elif installed_count == 0:
        print("Installation FAILED - no versions were installed.")
        if platform.system() == "Windows" and args.symlink:
            print()
            print("On Windows, creating symlinks requires either:")
            print("  1. Run this script as Administrator, OR")
            print("  2. Enable Developer Mode in Windows Settings:")
            print("     Settings -> Update & Security -> For developers -> Developer Mode")
            print()
            print("Alternatively, install without --symlink to copy files instead.")
        return 1
    else:
        print(f"Installation complete! ({installed_count} version(s))")
        print()
        print("To use the plugin:")
        print("  1. Open KiCad 10 (the IPC API must be enabled in")
        print("     Preferences -> Plugins).")
        print("  2. Open a PCB. KiCad will provision the plugin's Python venv")
        print("     from requirements.txt on first action invocation; this")
        print("     can take a minute the first time.")
        print("  3. Click the KiCadRoutingTools toolbar button in the PCB Editor.")
        print()

        if args.symlink:
            print("Note: Installed as symlink for development.")
            print("      Changes to source files will be reflected on the next")
            print("      action invocation without re-installing.")

    return 0


if __name__ == "__main__":
    sys.exit(main())

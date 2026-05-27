#!/usr/bin/env python3
"""
KiCad Routing Tools Plugin Installer (KiCad 10 IPC)

Drops the plugin source tree (or a symlink) into KiCad 10's user plugins
directory so it shows up in the toolbar / action menu. KiCad 10 provisions
a per-plugin Python venv from `requirements.txt` automatically on first
action invocation, so this installer no longer pip-installs anything.

Usage:
    python install_plugin.py [--uninstall] [--symlink]

Options:
    --uninstall    Remove the plugin instead of installing
    --symlink      Symlink the source tree (for development; edits take
                   effect on the next action invocation without re-copy)
"""

import os
import sys
import shutil
import platform
import argparse
from pathlib import Path

from startup_checks import get_cargo_version


PLUGIN_NAME = "KiCadRoutingTools"
PLUGIN_DISPLAY_NAME = "KiCad Routing Tools"
PLUGIN_IDENTIFIER = "com.github.drandyhaas.kicadroutingtools"
KICAD_VERSIONS = ["10.0"]  # IPC plugins are KiCad 10+ only


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
    if dest_dir.exists():
        print(f"  Removing existing installation at {dest_dir}")
        shutil.rmtree(dest_dir)

    def ignore_patterns(directory, files):
        ignored = []
        for f in files:
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

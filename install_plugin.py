#!/usr/bin/env python3
"""
KiCad Routing Tools Plugin Installer

Installs the plugin to the KiCad 9 plugins directory.

Usage:
    python install_plugin.py [--no-deps] [--uninstall] [--symlink]

Options:
    --no-deps               Skip installing Python dependencies
    --uninstall             Remove the plugin instead of installing
    --symlink               Create symlink instead of copying (for development)
"""

import os
import sys
import shutil
import platform
import argparse
import subprocess
from pathlib import Path


PLUGIN_NAME = "KiCadRoutingTools"
PLUGIN_DISPLAY_NAME = "KiCad Routing Tools"
KICAD_VERSIONS = ["9.0", "9.99"]  # 9.0 = stable, 9.99 = nightly


def get_kicad_base_dir() -> Path:
    """
    Get the KiCad base directory for the current platform.

    Returns:
        Path to the KiCad base directory (without version)
    """
    system = platform.system()

    if system == "Linux":
        return Path.home() / ".local" / "share" / "kicad"
    elif system == "Darwin":  # macOS
        return Path.home() / "Documents" / "KiCad"
    elif system == "Windows":
        return Path.home() / "Documents" / "KiCad"
    else:
        raise RuntimeError(f"Unsupported operating system: {system}")


def get_kicad_plugin_dirs() -> list:
    """
    Get KiCad plugin directories for all installed versions.

    Returns:
        List of (version, plugin_dir) tuples for versions that exist
    """
    base_dir = get_kicad_base_dir()
    plugin_dirs = []

    for version in KICAD_VERSIONS:
        version_dir = base_dir / version
        if version == "9.0" or version_dir.exists():
            # Always include 9.0, include others only if they exist
            # Use 3rdparty/plugins for plugins
            plugin_dir = version_dir / "3rdparty" / "plugins"
            plugin_dirs.append((version, plugin_dir))

    return plugin_dirs


def get_source_dir() -> Path:
    """Get the source directory (KiCadRoutingTools root)."""
    return Path(__file__).parent.resolve()


def install_dependencies():
    """Install Python dependencies from requirements.txt."""
    source_dir = get_source_dir()
    requirements_file = source_dir / "requirements.txt"

    if not requirements_file.exists():
        print("  No requirements.txt found, skipping dependency installation")
        return True

    print("Installing Python dependencies...")
    try:
        # Try to install dependencies
        result = subprocess.run(
            [sys.executable, "-m", "pip", "install", "-r", str(requirements_file)],
            capture_output=True,
            text=True
        )

        if result.returncode != 0:
            print(f"  Warning: Failed to install some dependencies:")
            print(f"  {result.stderr}")
            print("  You may need to install them manually:")
            print(f"    pip install -r {requirements_file}")
            return False
        else:
            print("  Dependencies installed successfully")
            return True

    except Exception as e:
        print(f"  Warning: Could not install dependencies: {e}")
        return False


def copy_plugin(source_dir: Path, dest_dir: Path):
    """Copy the entire KiCadRoutingTools directory to destination."""
    # Remove existing installation if present
    if dest_dir.exists():
        print(f"  Removing existing installation at {dest_dir}")
        shutil.rmtree(dest_dir)

    # Copy everything, excluding unnecessary files
    def ignore_patterns(directory, files):
        ignored = []
        for f in files:
            # Skip hidden files, cache, and non-essential directories
            if f.startswith('.') or f == '__pycache__' or f.endswith('.pyc'):
                ignored.append(f)
            # Skip test files and docs
            elif f.startswith('test_') or f == 'docs':
                ignored.append(f)
            # Skip kicad_files (sample PCBs)
            elif f == 'kicad_files':
                ignored.append(f)
        return ignored

    print(f"  Copying to {dest_dir}")
    shutil.copytree(source_dir, dest_dir, ignore=ignore_patterns)
    print("  Done")


def create_symlink(source_dir: Path, dest_dir: Path):
    """Create a symlink to the plugin source (for development)."""
    # Remove existing installation if present
    if dest_dir.exists() or dest_dir.is_symlink():
        if dest_dir.is_symlink():
            dest_dir.unlink()
        else:
            shutil.rmtree(dest_dir)

    # Create symlink
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
        description=f"Install {PLUGIN_DISPLAY_NAME} plugin for KiCad 9+",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python install_plugin.py              # Install the plugin
    python install_plugin.py --symlink    # Create symlink (for development)
    python install_plugin.py --uninstall  # Remove the plugin
"""
    )
    parser.add_argument(
        "--no-deps",
        action="store_true",
        help="Skip installing Python dependencies"
    )
    parser.add_argument(
        "--uninstall", "-u",
        action="store_true",
        help="Remove the plugin instead of installing"
    )
    parser.add_argument(
        "--symlink", "-s",
        action="store_true",
        help="Create symlink instead of copying (for development)"
    )

    args = parser.parse_args()

    print(f"\n{PLUGIN_DISPLAY_NAME} Installer")
    print("=" * 50)

    # Get directories
    try:
        source_dir = get_source_dir()
        plugin_dirs = get_kicad_plugin_dirs()
    except Exception as e:
        print(f"Error: {e}")
        return 1

    print(f"Platform: {platform.system()}")
    print(f"Source: {source_dir}")
    print()

    # Show which versions will be installed to
    versions_found = [v for v, _ in plugin_dirs]
    print(f"KiCad versions found: {', '.join(versions_found)}")
    print()

    # Install dependencies (once, not per version)
    if not args.uninstall and not args.no_deps:
        install_dependencies()
        print()

    # Process each KiCad version
    installed_count = 0
    for version, dest_base in plugin_dirs:
        dest_dir = dest_base / PLUGIN_NAME
        print(f"KiCad {version}:")
        print(f"  Destination: {dest_dir}")

        # Uninstall if requested
        if args.uninstall:
            uninstall_plugin(dest_dir)
            continue

        # Create destination directory if needed
        if not dest_base.exists():
            print(f"  Creating plugins directory: {dest_base}")
            dest_base.mkdir(parents=True, exist_ok=True)

        # Install the plugin
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
    else:
        print(f"Installation complete! ({installed_count} version(s))")
        print()
        print("To use the plugin:")
        print("  1. Open KiCad 9.0 or later")
        print("  2. Open a PCB in Pcbnew")
        print("  3. Go to Tools -> External Plugins -> Route Nets")
        print()

        if args.symlink:
            print("Note: Installed as symlink for development.")
            print("      Changes to source files will be reflected immediately.")

    return 0


if __name__ == "__main__":
    sys.exit(main())

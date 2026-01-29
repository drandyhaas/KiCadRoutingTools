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

from startup_checks import get_cargo_version


PLUGIN_NAME = "KiCadRoutingTools"
PLUGIN_DISPLAY_NAME = "KiCad Routing Tools"
KICAD_VERSIONS = ["9.0", "9.99"]  # 9.0 = stable, 9.99 = nightly


def get_kicad_python() -> Path | None:
    """
    Find KiCad's bundled Python executable.

    Returns:
        Path to KiCad's Python, or None if not found
    """
    system = platform.system()

    if system == "Windows":
        # Check common KiCad installation paths
        candidates = [
            Path(r"C:\Program Files\KiCad\9.0\bin\python.exe"),
            Path(r"C:\Program Files\KiCad\9.99\bin\python.exe"),
            Path(r"C:\Program Files (x86)\KiCad\9.0\bin\python.exe"),
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate

    elif system == "Darwin":  # macOS
        candidates = [
            Path("/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3"),
            Path("/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/3.9/bin/python3"),
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate

    elif system == "Linux":
        # Linux KiCad typically uses system Python, but check for flatpak/snap
        # For now, return None to use system Python
        pass

    return None


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
    """Install Python dependencies into KiCad's Python environment."""
    source_dir = get_source_dir()
    requirements_file = source_dir / "requirements.txt"

    if not requirements_file.exists():
        print("  No requirements.txt found, skipping dependency installation")
        return True

    # Parse and display requirements
    print("Required packages:")
    packages = []
    with open(requirements_file, 'r') as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#'):
                packages.append(line)
                print(f"  - {line}")
    print()

    # Find KiCad's Python
    kicad_python = get_kicad_python()

    if kicad_python:
        print(f"Installing to KiCad's Python: {kicad_python}")
        python_exe = str(kicad_python)
    else:
        print("Warning: KiCad Python not found, using system Python.")
        print("  KiCad uses its own Python - you may need to install manually.")
        python_exe = sys.executable

    print()

    try:
        # Run pip with output visible to user (no capture)
        result = subprocess.run(
            [python_exe, "-m", "pip", "install", "--progress-bar", "on", "-r", str(requirements_file)],
        )

        if result.returncode != 0:
            print()
            print("  Warning: Failed to install some dependencies.")
            if kicad_python:
                print("  You may need to run as Administrator:")
                print(f"    \"{kicad_python}\" -m pip install -r \"{requirements_file}\"")
            else:
                print("  You may need to install them manually to KiCad's Python.")
            return False
        else:
            print()
            print("  Dependencies installed successfully")
            return True

    except PermissionError:
        print()
        print("  Error: Permission denied. Try running as Administrator.")
        if kicad_python:
            print(f"  Or run manually: \"{kicad_python}\" -m pip install -r \"{requirements_file}\"")
        return False
    except Exception as e:
        print(f"  Warning: Could not install dependencies: {e}")
        return False


def check_rust_router():
    """
    Check that the Rust router is built and version matches Cargo.toml.

    Returns:
        True if check passes, False otherwise
    """
    source_dir = get_source_dir()
    rust_dir = source_dir / "rust_router"

    # Get expected version from Cargo.toml
    cargo_version = get_cargo_version()
    if cargo_version is None:
        print("  Warning: Could not read version from Cargo.toml")
        return False

    # Add rust_router to path for import
    if str(rust_dir) not in sys.path:
        sys.path.insert(0, str(rust_dir))

    # Try to import the Rust library
    try:
        import grid_router
        installed_version = getattr(grid_router, '__version__', 'unknown')
    except ImportError:
        print("  Error: Rust router module not found")
        print("  Please build it first with:")
        print("    python build_router.py")
        return False

    # Check version match
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

    # Check Rust router is built and up to date
    if not args.uninstall:
        print("Checking Rust router...")
        if not check_rust_router():
            return 1
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
        print("  1. Open KiCad 9.0 or later")
        print("  2. Open a PCB in Pcbnew")
        print("  3. Go to Tools -> External Plugins -> KiCadRoutingTools")
        print()

        if args.symlink:
            print("Note: Installed as symlink for development.")
            print("      Changes to source files will be reflected immediately.")

    return 0


if __name__ == "__main__":
    sys.exit(main())

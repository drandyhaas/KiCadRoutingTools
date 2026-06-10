#!/usr/bin/env python3
"""
KiCad Routing Tools Plugin Installer

Installs the plugin to the KiCad 9 plugins directory.

Usage:
    python install_plugin.py [--no-deps] [--uninstall] [--symlink] [--keep-pcm]

Options:
    --no-deps               Skip installing Python dependencies
    --uninstall             Remove the plugin instead of installing
    --symlink               Create symlink instead of copying (for development)
    --keep-pcm              Don't disable conflicting PCM copies of this plugin

On install, any copy of this plugin previously installed through KiCad's Plugin &
Content Manager is detected (it sits next to the local install in
3rdparty/plugins and would shadow it on sys.path) and moved aside, unless
--keep-pcm is given.
"""

import os
import sys
import json
import shutil
import platform
import argparse
import subprocess
from pathlib import Path

from startup_checks import get_cargo_version


PLUGIN_NAME = "KiCadRoutingTools"
PLUGIN_DISPLAY_NAME = "KiCad Routing Tools"
KICAD_VERSIONS = ["10.0", "9.99", "9.0"]  # Checked in order; 9.99 = nightly
# PCM (Plugin & Content Manager) package identifier from metadata.json. A PCM
# install lands in 3rdparty/plugins/<identifier-with-dots-as-underscores>/ next
# to our dev install, putting the same top-level modules on sys.path and
# shadowing the local version. We detect and disable such copies on install.
PCM_IDENTIFIER = "com.github.drandyhaas.kicadroutingtools"


def get_kicad_python() -> Path | None:
    """
    Find KiCad's bundled Python executable.

    Returns:
        Path to KiCad's Python, or None if not found
    """
    system = platform.system()

    if system == "Windows":
        # Check common KiCad installation paths (newest version first)
        candidates = [
            Path(r"C:\Program Files\KiCad\10.0\bin\python.exe"),
            Path(r"C:\Program Files (x86)\KiCad\10.0\bin\python.exe"),
            Path(r"C:\Program Files\KiCad\9.99\bin\python.exe"),
            Path(r"C:\Program Files\KiCad\9.0\bin\python.exe"),
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
    """
    Get KiCad plugin directories for all installed versions.

    Returns:
        List of (version, plugin_dir) tuples for versions that exist
    """
    base_dir = get_kicad_base_dir()
    plugin_dirs = []

    for version in KICAD_VERSIONS:
        version_dir = base_dir / version
        # Check if this KiCad version is installed on the system
        install_dir = Path(rf"C:\Program Files\KiCad\{version}") if platform.system() == "Windows" else version_dir
        if install_dir.exists() or version_dir.exists():
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
    def ignore_patterns(directory, files):
        ignored = []
        for f in files:
            # Keep .claude/ (routing skills) for use from the installed plugin.
            if f == '.claude':
                continue
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
    for child in sorted(plugins_dir.iterdir()):
        if not child.is_dir() or child.name == PLUGIN_NAME:
            continue
        if _dir_is_our_pcm_plugin(child):
            conflicts.append(child)
    return conflicts


def disable_pcm_install(pcm_dir: Path, version: str) -> Path | None:
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
    parser.add_argument(
        "--keep-pcm",
        action="store_true",
        help="Don't disable conflicting PCM (Plugin & Content Manager) copies of this plugin"
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

        # Disable any PCM-installed copy that would shadow this install
        if not args.keep_pcm:
            handle_pcm_conflicts(dest_base, version)

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
        print("  1. Open KiCad (9.0 or later)")
        print("  2. Open a PCB in Pcbnew")
        print("  3. Go to Tools -> External Plugins -> KiCadRoutingTools")
        print()

        if args.symlink:
            print("Note: Installed as symlink for development.")
            print("      Changes to source files will be reflected immediately.")

    return 0


if __name__ == "__main__":
    sys.exit(main())

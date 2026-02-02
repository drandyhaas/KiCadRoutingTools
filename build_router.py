#!/usr/bin/env python3
"""Build script for the Rust grid router module."""

import argparse
import subprocess
import shutil
import sys
import os


def clean(script_dir, rust_dir):
    """Remove all Rust build outputs and compiled library files."""
    print("Cleaning Rust build outputs...")

    # Remove target directory (all Rust build outputs)
    target_dir = os.path.join(rust_dir, 'target')
    if os.path.exists(target_dir):
        print(f"Removing {target_dir}")
        shutil.rmtree(target_dir)

    # Remove compiled library files in rust_router directory
    lib_files = [
        os.path.join(rust_dir, 'grid_router.pyd'),
        os.path.join(rust_dir, 'grid_router.so'),
        os.path.join(rust_dir, 'grid_router.abi3.so'),
    ]
    for lib_file in lib_files:
        if os.path.exists(lib_file):
            print(f"Removing {lib_file}")
            os.remove(lib_file)

    # Remove stale copies in parent directory
    stale_files = [
        os.path.join(script_dir, 'grid_router.pyd'),
        os.path.join(script_dir, 'grid_router.so'),
        os.path.join(script_dir, 'grid_router.abi3.so'),
    ]
    for stale in stale_files:
        if os.path.exists(stale):
            print(f"Removing stale module: {stale}")
            os.remove(stale)

    print("Clean complete.")


def build(script_dir, rust_dir):
    """Build the Rust router module."""
    print("Building Rust router...")
    try:
        result = subprocess.run(
            ['cargo', 'build', '--release'],
            cwd=rust_dir,
            capture_output=False
        )
    except FileNotFoundError:
        print("ERROR: Rust is not installed or 'cargo' is not in PATH.")
        print()
        print("To install Rust, follow these instructions:")
        print()
        if sys.platform == 'win32':
            print("  Windows:")
            print("    1. Download rustup-init.exe from https://rustup.rs/")
            print("    2. Run the installer and follow the prompts")
            print("    3. Restart your terminal/command prompt")
            print()
            print("  Or using winget:")
            print("    winget install Rustlang.Rustup")
        elif sys.platform == 'darwin':
            print("  macOS:")
            print("    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh")
            print()
            print("  Or using Homebrew:")
            print("    brew install rust")
        else:
            print("  Linux:")
            print("    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh")
            print()
            print("  Or using your package manager:")
            print("    Ubuntu/Debian: sudo apt install rustc cargo")
            print("    Fedora: sudo dnf install rust cargo")
            print("    Arch: sudo pacman -S rust")
        print()
        print("After installation, restart your terminal and run this script again.")
        sys.exit(1)

    if result.returncode != 0:
        print("ERROR: Rust build failed!")
        sys.exit(1)

    # Determine source and destination paths
    if sys.platform == 'win32':
        src = os.path.join(rust_dir, 'target', 'release', 'grid_router.dll')
        dst = os.path.join(rust_dir, 'grid_router.pyd')
    elif sys.platform == 'darwin':
        src = os.path.join(rust_dir, 'target', 'release', 'libgrid_router.dylib')
        dst = os.path.join(rust_dir, 'grid_router.so')
    else:
        src = os.path.join(rust_dir, 'target', 'release', 'libgrid_router.so')
        dst = os.path.join(rust_dir, 'grid_router.so')

    # Copy to destination
    print(f"Copying {src} -> {dst}")
    shutil.copy2(src, dst)

    # Verify version
    sys.path.insert(0, rust_dir)

    # Force reimport if already loaded
    if 'grid_router' in sys.modules:
        del sys.modules['grid_router']

    import grid_router
    print(f"Successfully built grid_router v{grid_router.__version__}")

    # Remove any stale copies in the parent directory
    stale_files = [
        os.path.join(script_dir, 'grid_router.pyd'),
        os.path.join(script_dir, 'grid_router.so'),
        os.path.join(script_dir, 'grid_router.abi3.so'),
    ]
    for stale in stale_files:
        if os.path.exists(stale):
            print(f"Removing stale module: {stale}")
            os.remove(stale)


def main():
    parser = argparse.ArgumentParser(description="Build the Rust grid router module")
    parser.add_argument('--clean', action='store_true',
                        help="Remove all Rust build outputs and compiled library files")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    rust_dir = os.path.join(script_dir, 'rust_router')

    if args.clean:
        clean(script_dir, rust_dir)
    else:
        build(script_dir, rust_dir)


if __name__ == '__main__':
    main()

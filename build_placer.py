#!/usr/bin/env python3
"""Build script for the Rust placement scorer module."""

import argparse
import subprocess
import shutil
import sys
import os


def clean(script_dir, rust_dir):
    """Remove all Rust build outputs and compiled library files."""
    print("Cleaning Rust build outputs...")

    target_dir = os.path.join(rust_dir, 'target')
    if os.path.exists(target_dir):
        print(f"Removing {target_dir}")
        shutil.rmtree(target_dir)

    lib_files = [
        os.path.join(rust_dir, 'placement_scorer.pyd'),
        os.path.join(rust_dir, 'placement_scorer.so'),
        os.path.join(rust_dir, 'placement_scorer.abi3.so'),
    ]
    for lib_file in lib_files:
        if os.path.exists(lib_file):
            print(f"Removing {lib_file}")
            os.remove(lib_file)

    stale_files = [
        os.path.join(script_dir, 'placement_scorer.pyd'),
        os.path.join(script_dir, 'placement_scorer.so'),
        os.path.join(script_dir, 'placement_scorer.abi3.so'),
    ]
    for stale in stale_files:
        if os.path.exists(stale):
            print(f"Removing stale module: {stale}")
            os.remove(stale)

    print("Clean complete.")


def build(script_dir, rust_dir):
    """Build the Rust placement scorer module."""
    print("Building Rust placement scorer...")
    try:
        result = subprocess.run(
            ['cargo', 'build', '--release'],
            cwd=rust_dir,
            capture_output=False
        )
    except FileNotFoundError:
        print("ERROR: Rust is not installed or 'cargo' is not in PATH.")
        print()
        print("To install Rust:")
        if sys.platform == 'darwin':
            print("  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh")
            print("  Or: brew install rust")
        elif sys.platform == 'win32':
            print("  Download rustup-init.exe from https://rustup.rs/")
        else:
            print("  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh")
        print()
        print("After installation, restart your terminal and run this script again.")
        sys.exit(1)

    if result.returncode != 0:
        print("ERROR: Rust build failed!")
        sys.exit(1)

    # Determine source and destination paths
    if sys.platform == 'win32':
        src = os.path.join(rust_dir, 'target', 'release', 'placement_scorer.dll')
        dst = os.path.join(rust_dir, 'placement_scorer.pyd')
    elif sys.platform == 'darwin':
        src = os.path.join(rust_dir, 'target', 'release', 'libplacement_scorer.dylib')
        dst = os.path.join(rust_dir, 'placement_scorer.so')
    else:
        src = os.path.join(rust_dir, 'target', 'release', 'libplacement_scorer.so')
        dst = os.path.join(rust_dir, 'placement_scorer.so')

    print(f"Copying {src} -> {dst}")
    shutil.copy2(src, dst)

    # Verify version
    sys.path.insert(0, rust_dir)

    if 'placement_scorer' in sys.modules:
        del sys.modules['placement_scorer']

    import placement_scorer
    print(f"Successfully built placement_scorer v{placement_scorer.__version__}")

    # Remove any stale copies in the parent directory
    stale_files = [
        os.path.join(script_dir, 'placement_scorer.pyd'),
        os.path.join(script_dir, 'placement_scorer.so'),
        os.path.join(script_dir, 'placement_scorer.abi3.so'),
    ]
    for stale in stale_files:
        if os.path.exists(stale):
            print(f"Removing stale module: {stale}")
            os.remove(stale)


def main():
    parser = argparse.ArgumentParser(description="Build the Rust placement scorer module")
    parser.add_argument('--clean', action='store_true',
                        help="Remove all Rust build outputs and compiled library files")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    rust_dir = os.path.join(script_dir, 'rust_placer')

    if args.clean:
        clean(script_dir, rust_dir)
    else:
        build(script_dir, rust_dir)


if __name__ == '__main__':
    main()

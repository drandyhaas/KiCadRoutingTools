"""
Startup checks for the PCB router.

Verifies that:
1. Required Python libraries are available (numpy)
2. The Rust library is available
3. The Rust library version matches Cargo.toml

If version mismatch is detected, automatically rebuilds using build_router.py.
"""

import sys
import os


def check_python_dependencies():
    """Check that required Python libraries are available."""
    missing = []

    # Check numpy (required by the Rust router module)
    try:
        import numpy
    except ImportError:
        missing.append('numpy')

    # Check scipy (required for optimal target assignment)
    try:
        from scipy.optimize import linear_sum_assignment
    except ImportError:
        missing.append('scipy')

    if missing:
        print("ERROR: Missing required Python libraries:")
        for lib in missing:
            print(f"  - {lib}")
        print("\nInstall with:")
        print(f"  pip install {' '.join(missing)}")
        print(f"  (or pip3 install {' '.join(missing)})")
        sys.exit(1)


def get_cargo_version():
    """Read the version from Cargo.toml."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    cargo_path = os.path.join(script_dir, 'rust_router', 'Cargo.toml')

    if not os.path.exists(cargo_path):
        return None

    with open(cargo_path, 'r') as f:
        for line in f:
            if line.startswith('version'):
                # Parse: version = "0.8.3"
                parts = line.split('=', 1)
                if len(parts) == 2:
                    version = parts[1].strip().strip('"').strip("'")
                    return version
    return None


def check_rust_library():
    """
    Check that the Rust library is available and version matches Cargo.toml.
    Rebuilds automatically if version mismatch detected.

    Returns the installed version string.
    """
    script_dir = os.path.dirname(os.path.abspath(__file__))
    rust_dir = os.path.join(script_dir, 'rust_router')

    # Add rust_router to path for import
    if rust_dir not in sys.path:
        sys.path.insert(0, rust_dir)

    cargo_version = get_cargo_version()
    if cargo_version is None:
        print("WARNING: Could not read version from Cargo.toml")
        cargo_version = "unknown"

    # Try to import the Rust library
    try:
        import grid_router
        installed_version = getattr(grid_router, '__version__', 'unknown')
    except ImportError:
        installed_version = None

    # Check if rebuild is needed
    needs_rebuild = False
    if installed_version is None:
        print("Rust router module not found - building...")
        needs_rebuild = True
    elif installed_version != cargo_version:
        print(f"Rust router version mismatch: installed={installed_version}, Cargo.toml={cargo_version}")
        print("Rebuilding...")
        needs_rebuild = True

    if needs_rebuild:
        # On Windows, the DLL is locked once imported, so we can't rebuild in-process
        if sys.platform == 'win32' and installed_version is not None:
            print("\nERROR: Cannot rebuild while the library is loaded (Windows limitation).")
            print("Please run the build manually in a new terminal:")
            print("  python build_router.py")
            print("\nThen re-run your command.")
            sys.exit(1)

        # Run build_router.py to rebuild
        build_script = os.path.join(script_dir, 'build_router.py')
        if not os.path.exists(build_script):
            print("ERROR: build_router.py not found!")
            print("Build the Rust router manually with:")
            print("  cd rust_router && cargo build --release")
            sys.exit(1)

        import subprocess
        result = subprocess.run([sys.executable, build_script], capture_output=False)
        if result.returncode != 0:
            print("ERROR: Rust router build failed!")
            sys.exit(1)

        # Force reimport after rebuild
        if 'grid_router' in sys.modules:
            del sys.modules['grid_router']

        try:
            import grid_router
            installed_version = getattr(grid_router, '__version__', 'unknown')
        except ImportError as e:
            print(f"ERROR: Rust router module still not found after rebuild: {e}")
            sys.exit(1)

        # Verify version after rebuild
        if installed_version != cargo_version:
            print(f"ERROR: Version still mismatched after rebuild!")
            print(f"  Installed: {installed_version}")
            print(f"  Expected:  {cargo_version}")
            print("This may indicate a problem with the Rust library's __version__ attribute.")
            sys.exit(1)

        print(f"Successfully rebuilt Rust router v{installed_version}")

    return installed_version


def run_all_checks():
    """Run all startup checks. Returns the Rust library version."""
    check_python_dependencies()
    return check_rust_library()


if __name__ == '__main__':
    # Allow running standalone to check/rebuild
    version = run_all_checks()
    print(f"All checks passed. Rust router v{version}")

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

    # Check scipy (required for optimal target assignment and Voronoi)
    try:
        from scipy.optimize import linear_sum_assignment
    except ImportError:
        missing.append('scipy')

    # Check shapely (required for polygon union in multi-net plane layers)
    try:
        from shapely.geometry import Polygon
    except ImportError:
        missing.append('shapely')

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
        print("Rust router module not found")
        needs_rebuild = True
    elif installed_version != cargo_version:
        print(f"Rust router version mismatch: installed={installed_version}, Cargo.toml={cargo_version}")
        needs_rebuild = True

    if needs_rebuild:
        print("Please run:")
        print("  python build_router.py")
        print("\nThen re-run your command.")
        sys.exit(1)
    return installed_version


def run_all_checks():
    """Run all startup checks. Returns the Rust library version."""
    check_python_dependencies()
    return check_rust_library()


if __name__ == '__main__':
    # Allow running standalone to check/rebuild
    version = run_all_checks()
    print(f"All checks passed. Rust router v{version}")

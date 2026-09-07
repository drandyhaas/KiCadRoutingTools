"""
Startup checks for the PCB router.

Verifies that:
1. Required Python libraries are available (numpy)
2. The Rust library is available
3. The Rust library version matches Cargo.toml

If version mismatch is detected, automatically rebuilds using build_router.py.

These checks RAISE `StartupCheckError` rather than calling sys.exit (#457 item 3).
The CLI entry points still exit 1 with the same message -- they catch it at their
module-scope call site and only exit when they are the program being run -- but an
IMPORT of route.py (a test, the GUI plugin, another tool) now gets a catchable
exception. sys.exit at import time killed pytest COLLECTION outright:
`SystemExit` escapes collection as an INTERNALERROR rather than a per-file error,
so on a checkout without a built router the 8 test files that import a routing
module at module level took the entire ~200-test suite down with them.
"""

import sys
import os


class StartupCheckError(RuntimeError):
    """A startup precondition (Python deps, Rust router) is not satisfied.

    Carries the full user-facing message, including install/build instructions,
    so a caller can print it verbatim.
    """


def check_python_dependencies():
    """Check that required Python libraries are available.

    Raises StartupCheckError naming the missing libraries.
    """
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

    # Check Pillow (#887). route_render.py and render_placement.py import it at
    # MODULE SCOPE with no fallback, so every board still, review sheet and
    # movie needs it -- yet it was in neither this list nor requirements.txt. A
    # fresh clone following the README could render nothing, and learned that
    # from a runtime ImportError string rather than from this check, which is
    # the thing that exists to say so up front.
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        missing.append('Pillow')

    if missing:
        lines = ["ERROR: Missing required Python libraries:"]
        lines += [f"  - {lib}" for lib in missing]
        lines += ["", "Install with:",
                  f"  pip install {' '.join(missing)}",
                  f"  (or pip3 install {' '.join(missing)})"]
        raise StartupCheckError("\n".join(lines))


def get_cargo_version():
    """Read the version from Cargo.toml."""
    script_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
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


# rust_router/Cargo.toml pins pyo3 with `abi3-py39` unconditionally, so the
# extension targets CPython 3.9+ whether it is downloaded or built from source.
MIN_PYTHON = (3, 9)


def check_rust_library():
    """
    Check that the Rust library is available and version matches Cargo.toml.

    Raises StartupCheckError with the build instructions when it is missing or
    stale. Returns the installed version string.
    """
    script_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
    rust_dir = os.path.join(script_dir, 'rust_router')

    # Add rust_router to path for import
    if rust_dir not in sys.path:
        sys.path.insert(0, rust_dir)

    cargo_version = get_cargo_version()
    if cargo_version is None:
        print("WARNING: Could not read version from Cargo.toml")
        cargo_version = "unknown"

    # Try to import the Rust library
    import_error = None
    try:
        import grid_router
        installed_version = getattr(grid_router, '__version__', 'unknown')
    except ImportError as exc:
        installed_version = None
        # Keep the message. A bare `except ImportError` used to discard it, and
        # it is the only place the real cause is stated: on a too-old
        # interpreter the module builds and copies fine and then fails to load
        # with "symbol not found in flat namespace '_PyCMethod_New'" -- a symbol
        # added in CPython 3.9. Swallowing that turned a one-line fix into a
        # hunt through PYO3_PYTHON and cargo clean.
        import_error = str(exc)

    # Check if rebuild is needed
    reason = None
    if installed_version is None:
        reason = "Rust router module not found"
        if import_error:
            reason += f"\n  ({import_error})"
    elif installed_version != cargo_version:
        reason = (f"Rust router version mismatch: installed={installed_version}, "
                  f"Cargo.toml={cargo_version}")

    if reason:
        lines = [reason, "Please run:", "  python build_router.py", "",
                 "Then re-run your command."]
        if sys.version_info < MIN_PYTHON:
            lines += ["",
                      f"NOTE: this Python is {sys.version_info.major}."
                      f"{sys.version_info.minor}, but the router is built "
                      f"abi3-py{MIN_PYTHON[0]}{MIN_PYTHON[1]} and requires "
                      f"Python {MIN_PYTHON[0]}.{MIN_PYTHON[1]}+ -- building "
                      "from source will NOT lower that floor."]
        raise StartupCheckError("\n".join(lines))
    return installed_version


def run_all_checks():
    """Run all startup checks. Returns the Rust library version.

    Raises StartupCheckError; see the module docstring for why this does not
    exit. CLI entry points wrap this with `exit_on_error_if_main`.
    """
    check_python_dependencies()
    return check_rust_library()


def exit_on_error_if_main(module_name):
    """Run the checks at a CLI module's import, preserving the old behaviour.

    Used as `exit_on_error_if_main(__name__)` at module scope. When this module
    IS the program, a failure prints the message and exits 1 exactly as before.
    When it is merely being imported, the StartupCheckError propagates so the
    importer can handle it -- and so pytest reports one collect error instead of
    aborting the session (#457 item 3).
    """
    try:
        return run_all_checks()
    except StartupCheckError as exc:
        if module_name == '__main__':
            print(exc)
            sys.exit(1)
        raise


if __name__ == '__main__':
    # Allow running standalone to check/rebuild
    try:
        version = run_all_checks()
    except StartupCheckError as exc:
        print(exc)
        sys.exit(1)
    print(f"All checks passed. Rust router v{version}")

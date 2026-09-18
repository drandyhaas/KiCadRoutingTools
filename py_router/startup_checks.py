"""
Startup checks for the PCB router.

Verifies that:
1. Required Python libraries are available (numpy)
2. The Rust library is available
3. The Rust library version matches Cargo.toml

Rendering is a SEPARATE gate (`check_render_dependencies`, #887): Pillow is not
needed to route, and requiring it here stops a machine that can route from
routing. See that function for what it cost when the two were merged.

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

import importlib
import os
import re
import sys


class StartupCheckError(RuntimeError):
    """A startup precondition (Python deps, Rust router) is not satisfied.

    Carries the full user-facing message, including install/build instructions,
    so a caller can print it verbatim.
    """


class RenderDependencyError(StartupCheckError, ImportError):
    """The RASTER stack (Pillow) is missing -- rendering only, never routing.

    Also an `ImportError`, deliberately (#943). Every consumer that DISABLES
    rendering rather than failing spells that `except ImportError`, because the
    thing it guards is a `from PIL import ...`. A bare StartupCheckError is a
    RuntimeError, so it walked straight past those handlers: the GUI's routing
    movie reported "failed (<install instructions>)" instead of "not rendered -
    install Pillow", and the placement tab's preview fell into its silent
    branch without setting `_preview_ok = False`, so it re-attempted the render
    on every board. Being both types means a caller can catch whichever it
    means -- the missing import, or the startup precondition.
    """


# ---------------------------------------------------------------------------
# What each dependency is, and how old is too old
# ---------------------------------------------------------------------------
#
# THE TABLE IS SHARED. `kicad_routing_plugin/deps_check.py` imports IMPORT_TESTS
# and the probes below rather than restating them -- the GUI used to carry its
# own hand-written copy of this list (in two places), and a hand-written copy of
# a list is a list that drifts.

# pip distribution name -> import statement used to verify it. The specific
# submodule imports catch broken/partial installs (e.g. shapely without its
# native lib) better than a bare `import pkg`. Packages absent from this dict
# fall back to `import <pip name>`, which is only right while the pip name IS
# the import name -- Pillow imports as `PIL`, and the missing entry once made
# the GUI unopenable on every machine (#943).
IMPORT_TESTS = {
    "numpy": "import numpy",
    "scipy": "from scipy.optimize import linear_sum_assignment",
    "shapely": "from shapely.geometry import Polygon",
    "Pillow": "from PIL import Image",
}

# pip distribution name -> the module whose `__version__` is read to decide
# whether it is too old. Read from the IMPORTED module, deliberately, and never
# from `importlib.metadata`: metadata answers "what did pip install", and the
# question here is "what will this interpreter actually import". Those differ
# exactly when it matters -- a stale copy earlier on sys.path shadowing a newer
# installed one is the failure this gate exists to name.
VERSION_MODULES = {
    "numpy": "numpy",
    "scipy": "scipy",
    "shapely": "shapely",
    "Pillow": "PIL",
}

# The packages ROUTING needs. Pillow is not among them and must not become one:
# it is the raster path, gated separately by `check_render_dependencies` (#887).
ROUTING_PACKAGES = ("numpy", "scipy", "shapely")

# Last-resort floors for an install that shipped without requirements.txt.
# requirements.txt is the source of truth (`requirement_floors` reads it and
# these only fill gaps); `tests/test_dependency_version_floor.py` asserts this
# constant against the file, so bumping one without the other fails.
#
# numpy 1.22 is the floor that MATTERS, and it is worth knowing why, because
# neither symptom names us and neither names the real cause:
#   * `numpy._DTypeMeta.__class_getitem__` arrived in 1.22. Below it, any
#     package annotating `np.ndarray[Any, np.dtype[...]]` at import time dies
#     with `TypeError: 'numpy._DTypeMeta' object is not subscriptable`.
#   * scipy >= 1.11 warns "A NumPy version >=1.22.4 and <2.3.0 is required for
#     this version of SciPy (detected version X)" -- a numpy-version complaint
#     that looks like it comes from us, and that a user checks by running
#     `pip show numpy` against a DIFFERENT interpreter, where it is fine.
# Both were reported together from one KiCad 10 install (2026-09-18).
FALLBACK_FLOORS = {
    "numpy": "1.22",
}


def requirements_path():
    """Path to requirements.txt, in the repo layout or a flat PCM install."""
    here = os.path.dirname(os.path.abspath(__file__))
    for candidate in (os.path.join(os.path.dirname(here), 'requirements.txt'),
                      os.path.join(here, 'requirements.txt')):
        if os.path.isfile(candidate):
            return candidate
    return None


# Package name at the start of a requirements line, then whatever specifier
# follows it. Stops the name at the first specifier/marker character.
_REQ_LINE_RE = re.compile(r"^\s*([A-Za-z0-9][A-Za-z0-9._-]*)\s*(.*)$")

# The FLOOR inside a specifier. `>=` and `==` only: this gate answers "is the
# installed one too old", which is the failure users hit. Upper bounds are
# pip's business -- enforcing `<2.3` here would refuse to start on a numpy that
# merely makes one dependency grumble.
_FLOOR_RE = re.compile(r"(?:>=|==)\s*([0-9][0-9A-Za-z.\-+]*)")


def parse_requirements(path):
    """Return [(pip_name, specifier)] from a requirements file.

    Skips comments, blanks and pip directives (`-r ...`, URLs). The specifier is
    kept, not stripped: it is what makes the difference between asking pip for
    `numpy` -- which an already-installed too-old numpy satisfies -- and asking
    for `numpy>=1.22`, which it does not.
    """
    out = []
    if not path or not os.path.isfile(path):
        return out
    with open(path, 'r') as fh:
        for raw in fh:
            line = raw.split('#', 1)[0].strip()
            if not line or line.startswith('-') or '://' in line:
                continue
            m = _REQ_LINE_RE.match(line)
            if m:
                out.append((m.group(1), m.group(2).strip()))
    return out


def requirement_floors():
    """{pip_name: minimum version string or None} from requirements.txt.

    `FALLBACK_FLOORS` fills in for a package the file does not floor, and for an
    install that shipped no requirements.txt at all -- where the old code
    checked nothing whatsoever, because an unreadable file parsed to an empty
    list and an empty list is a gate that passes everything.
    """
    floors = {}
    for name, spec in parse_requirements(requirements_path()):
        m = _FLOOR_RE.search(spec or '')
        floors[name] = m.group(1) if m else None
    for name, floor in FALLBACK_FLOORS.items():
        if not floors.get(name):
            floors[name] = floor
    return floors


def parse_version(text):
    """Version string -> comparable tuple of ints. '1.22.4rc1' -> (1, 22, 4).

    Stops at the first component with no leading digits, so a dev/rc/post
    suffix orders with its release rather than raising.
    """
    parts = []
    for chunk in str(text).split('.'):
        digits = ''
        for ch in chunk:
            if not ch.isdigit():
                break
            digits += ch
        if not digits:
            break
        parts.append(int(digits))
    return tuple(parts)


def version_satisfies(installed, floor):
    """True when `installed` is at least `floor`.

    An absent floor, or a version neither side can parse, satisfies: this gate
    refuses to start a program, so an unrecognised version string must not be
    the reason it refuses.
    """
    if not floor or not installed:
        return True
    have, need = parse_version(installed), parse_version(floor)
    if not have or not need:
        return True
    width = max(len(have), len(need))
    have += (0,) * (width - len(have))
    need += (0,) * (width - len(need))
    return have >= need


def imported_version(pip_name):
    """(version, file) of the module this interpreter imports for `pip_name`.

    (None, None) when it cannot be imported. The FILE is returned because it is
    the answer to the question a version complaint always raises -- "but mine is
    up to date" -- and it is usually a path the user did not expect.
    """
    module_name = VERSION_MODULES.get(pip_name, pip_name)
    try:
        module = importlib.import_module(module_name)
    except Exception:                                          # noqa: BLE001
        return None, None
    return (getattr(module, '__version__', None),
            getattr(module, '__file__', None))


class Problem(object):
    """One dependency that is absent, or present and too old."""

    def __init__(self, name, state, installed=None, floor=None, path=None):
        self.name = name
        self.state = state            # 'absent' | 'outdated'
        self.installed = installed
        self.floor = floor
        self.path = path

    @property
    def requirement(self):
        """What to hand pip: `numpy>=1.22`, or bare `numpy` with no floor."""
        return f"{self.name}>={self.floor}" if self.floor else self.name

    def describe(self):
        if self.state == 'absent':
            return f"  {self.name}: not installed"
        lines = [f"  {self.name}: {self.installed} is too old "
                 f"(need >= {self.floor})"]
        if self.path:
            lines.append(f"      imported from {self.path}")
        return "\n".join(lines)

    def __repr__(self):                                        # pragma: no cover
        return f"<Problem {self.name} {self.state} {self.installed}>"


def dependency_problems(names=ROUTING_PACKAGES, floors=None):
    """Return [Problem] for `names`: what cannot be imported, and what is stale.

    The two states are separate on purpose. "Missing scipy" and "numpy 1.21.6,
    which every one of these packages is newer than" need different sentences
    and different pip commands, and collapsing them into one list of names is
    how a too-old package became invisible: the old probe was `except
    ImportError` and nothing else, so a numpy from before 1.22 passed every
    gate, raised no dialog, and surfaced later as somebody else's error text.
    """
    floors = requirement_floors() if floors is None else floors
    problems = []
    for name in names:
        try:
            exec(IMPORT_TESTS.get(name, f"import {name}"), {})
        except ImportError:
            problems.append(Problem(name, 'absent', floor=floors.get(name)))
            continue
        floor = floors.get(name)
        installed, path = imported_version(name)
        if floor and installed and not version_satisfies(installed, floor):
            problems.append(Problem(name, 'outdated', installed=installed,
                                    floor=floor, path=path))
    return problems


def format_problems(problems, header):
    """The user-facing block for a list of Problems: what, where, and the fix.

    Names the INTERPRETER, because the whole difficulty of a version complaint
    from inside KiCad is that the python being complained about is not the one
    the user checks.
    """
    lines = [header]
    lines += [p.describe() for p in problems]
    lines += ["", f"Python: {sys.executable or '(embedded)'}",
              "", "Install with:",
              "  \"" + (sys.executable or 'python3') + "\" -m pip install "
              "--upgrade " + " ".join(f'"{p.requirement}"' for p in problems)]
    return "\n".join(lines)


def check_python_dependencies():
    """Check the libraries routing needs are importable AND new enough.

    Raises StartupCheckError naming what is missing or stale.
    """
    problems = dependency_problems(ROUTING_PACKAGES)
    if problems:
        raise StartupCheckError(format_problems(
            problems,
            "ERROR: Python libraries missing or too old for KiCad Routing "
            "Tools:"))


def _raise_if_missing(missing, exc=StartupCheckError):
    """Raise the actionable install message for a list of distribution names.

    `exc` is the class to raise: the routing gate wants a plain
    StartupCheckError, the render gate wants RenderDependencyError so that
    `except ImportError` sites catch it (#943).
    """
    if missing:
        lines = ["ERROR: Missing required Python libraries:"]
        lines += [f"  - {lib}" for lib in missing]
        lines += ["", "Install with:",
                  f"  pip install {' '.join(missing)}",
                  f"  (or pip3 install {' '.join(missing)})"]
        raise exc("\n".join(lines))


def check_render_dependencies():
    """Check the libraries the RASTER path needs. Raises RenderDependencyError.

    Pillow only (#887). Every board still, review sheet and movie needs it --
    yet it was declared in neither requirements.txt nor these checks, and a
    fresh clone learned that from a runtime ImportError string rather than from
    the check that exists to say so up front.

    SEPARATE from `check_python_dependencies`, which is the ROUTING gate, and
    that separation is the whole point. Pillow was briefly added to that gate
    instead, which made `route.py`, `route_diff.py`, `route_planes.py` and
    `repair_planes.py` refuse to start without it -- on a machine that can route
    perfectly well, because route.py imports `route_render` lazily and only
    under `--preview-png`. The cost was not hypothetical: the Modal corpus image
    (`tests/stress/modal_sweep/modal_app.py`, `_PY_PINS`) installs numpy, scipy
    and shapely and NOT Pillow, so every cloud A/B replay would have exited 1 on
    every board -- taking out the instrument that grades routing changes.

    Call it from the render entry points, which is where the requirement is
    real: at module scope in `route_render.py`, which is nothing but the raster
    stack, and at the DRAW sites in `render_placement.py`, which is also where
    `PlacementModel` and `legality_findings` live and is imported for those by
    `board_context.py` and the stress predictors -- tools that grade a
    placement and draw nothing (#943).

    It raises `RenderDependencyError`, which is an `ImportError` as well as a
    StartupCheckError, so the consumers that disable rendering rather than
    failing keep working. See that class.
    """
    try:
        from PIL import Image, ImageDraw, ImageFont     # noqa: F401
    except ImportError:
        _raise_if_missing(['Pillow'], RenderDependencyError)
        return

    # Too old counts as well, and reaches the same `except ImportError`
    # consumers -- but only ever for the RASTER feature, never for routing.
    floor = requirement_floors().get('Pillow')
    installed, path = imported_version('Pillow')
    if floor and installed and not version_satisfies(installed, floor):
        raise RenderDependencyError(format_problems(
            [Problem('Pillow', 'outdated', installed, floor, path)],
            "ERROR: Pillow is too old for the raster path:"))


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

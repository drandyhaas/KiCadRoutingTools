#!/usr/bin/env python3
"""#943 / #944: the GUI's dependency gate blocked on a package it does not need,
with a probe that could never pass, on an interpreter that could never install.

Three defects, one dialog:

  A. `_required_packages` fell back to `import <pip name>` for any package not
     in IMPORT_TESTS, and Pillow imports as `PIL`. So `import Pillow` raised
     ModuleNotFoundError on EVERY machine, installed or not. `action_plugin`
     spells the gate `if not ensure_dependencies(...): return`, and the
     post-install re-probe fails identically -- so from the day Pillow entered
     requirements.txt the plugin could not be opened at all, and "install it"
     and "restart KiCad" both changed nothing.

  B. Pillow is the RASTER path only: the movie checkbox and the placement
     preview, both of which already disable themselves. `startup_checks`
     records (#887) what it cost to make routing depend on it; reading
     requirements.txt wholesale re-made that mistake one layer up. It is now
     OPTIONAL_PACKAGES -- reported, installed alongside a blocking package,
     never a gate.

  C. `check_render_dependencies` raised StartupCheckError, a RuntimeError, so
     every `except ImportError` consumer that MEANS to disable rendering
     walked past it. It raises RenderDependencyError now, which is both.

  D. (#944) On a PEP 668 interpreter -- which is what KiCad's Linux packages
     run -- pip refuses every install into the prefix, so the one-click offer
     could not succeed. The gate names the distro packages instead.

Run with:  python3 tests/test_943_optional_render_dependency.py
"""
import importlib
import os
import sys
import types

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))   # #522
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))    # #522
sys.path.insert(0, os.path.join(ROOT_DIR, 'kicad_routing_plugin'))

# deps_check imports wx at module scope and the cloud image has none; the probe
# under test touches no wx symbol, so a stub keeps this in the unit lane rather
# than in the KiCad-python-only one.
sys.modules.setdefault('wx', types.ModuleType('wx'))

import deps_check  # noqa: E402
from startup_checks import (RenderDependencyError, StartupCheckError,  # noqa: E402
                            check_render_dependencies)

FAILS = []


def check(cond, msg):
    if not cond:
        FAILS.append(msg)
    return cond


# ---------------------------------------------------------------------------
# A. the probe agrees with what is actually installed
# ---------------------------------------------------------------------------
def test_probe_matches_installed_distributions():
    """Every requirements.txt name that IS installed must probe as present.

    Resolved against `importlib.metadata`, not against IMPORT_TESTS -- asking
    the same table the code asks would pass whatever the table said. The
    denominator is printed because a run that resolved no distribution at all
    has tested nothing.
    """
    from importlib import metadata

    blocking, optional = deps_check._missing_packages()
    reported = set(blocking) | set(optional)
    resolved = 0
    for pip_name, _stmt in deps_check._required_packages():
        try:
            metadata.distribution(pip_name)
        except metadata.PackageNotFoundError:
            continue          # genuinely not installed; the probe may say so
        resolved += 1
        check(pip_name not in reported,
              f"{pip_name} is an installed distribution but the probe reports "
              f"it missing -- its IMPORT_TESTS entry is wrong or absent (#943)")
    print(f"  probe vs importlib.metadata: {resolved} installed "
          f"distribution(s) checked")
    check(resolved > 0,
          "BROKEN TEST: no requirements.txt distribution could be resolved at "
          "all, so nothing was compared")


# ---------------------------------------------------------------------------
# B. Pillow does not block
# ---------------------------------------------------------------------------
def test_missing_pillow_does_not_block():
    """With the raster probe forced to fail, Pillow lands in the OPTIONAL
    bucket and `_missing_packages()[0]` -- the gate -- stays empty."""
    check("Pillow" in deps_check.OPTIONAL_PACKAGES,
          "Pillow is not in OPTIONAL_PACKAGES, so a machine without it is "
          "refused the plugin (#943 B)")

    saved = dict(deps_check.IMPORT_TESTS)
    try:
        deps_check.IMPORT_TESTS["Pillow"] = "import _kr_no_such_module_943"
        blocking, optional = deps_check._missing_packages()
        check("Pillow" in optional,
              f"forced-absent Pillow did not reach the optional bucket: "
              f"{optional}")
        check(blocking == [],
              f"a missing Pillow BLOCKS the plugin: blocking={blocking}")
    finally:
        deps_check.IMPORT_TESTS.clear()
        deps_check.IMPORT_TESTS.update(saved)


# ---------------------------------------------------------------------------
# C. the render gate is catchable as an ImportError
# ---------------------------------------------------------------------------
def test_render_gate_is_an_import_error():
    check(issubclass(RenderDependencyError, ImportError),
          "RenderDependencyError is not an ImportError, so the `except "
          "ImportError` consumers that disable rendering miss it (#943 C)")
    check(issubclass(RenderDependencyError, StartupCheckError),
          "RenderDependencyError is no longer a StartupCheckError, so callers "
          "that catch the startup contract miss it")

    saved = sys.modules.get('PIL', '<absent>')
    sys.modules['PIL'] = None          # makes `from PIL import ...` raise
    try:
        raised = None
        try:
            check_render_dependencies()
        except Exception as exc:                                # noqa: BLE001
            raised = exc
        check(isinstance(raised, RenderDependencyError),
              f"check_render_dependencies raised {type(raised).__name__} with "
              f"Pillow unavailable, not RenderDependencyError")
        check(raised is not None and 'Pillow' in str(raised),
              "the render gate's message does not name Pillow, so it is not "
              "the actionable message #887 asked for")
    finally:
        if saved == '<absent>':
            sys.modules.pop('PIL', None)
        else:
            sys.modules['PIL'] = saved


# ---------------------------------------------------------------------------
# B2. placement GRADING does not need the raster stack
# ---------------------------------------------------------------------------
def test_render_placement_imports_without_pillow():
    """`board_context` and the stress predictors import render_placement for
    PlacementModel / legality_findings and draw nothing. A module-scope raster
    gate made Pillow a requirement of grading a placement."""
    saved_pil = sys.modules.get('PIL', '<absent>')
    dropped = [m for m in list(sys.modules)
               if m == 'render_placement' or m.startswith('render_placement.')]
    saved_mods = {m: sys.modules.pop(m) for m in dropped}
    sys.modules['PIL'] = None
    try:
        mod = importlib.import_module('render_placement')
        for name in ('PlacementModel', 'legality_findings'):
            check(hasattr(mod, name),
                  f"render_placement imported without Pillow but has no "
                  f"{name} -- the lazy split dropped a non-raster export")
    except Exception as exc:                                    # noqa: BLE001
        check(False,
              f"render_placement cannot be imported without Pillow: "
              f"{type(exc).__name__}: {exc} (#943)")
    finally:
        sys.modules.pop('render_placement', None)
        sys.modules.update(saved_mods)
        if saved_pil == '<absent>':
            sys.modules.pop('PIL', None)
        else:
            sys.modules['PIL'] = saved_pil


# ---------------------------------------------------------------------------
# D. PEP 668 (#944)
# ---------------------------------------------------------------------------
def test_pep668_probe_and_message():
    """Both arms of the probe, against a PLANTED marker.

    Reading only the real interpreter would make this vacuous on every machine
    that is not a PEP 668 distro -- which is every machine this repo is
    developed on, and the one arm that matters would never run.

    The probe and the distro tables live in `startup_checks` since #1026,
    shared with install_plugin.py; the dialog below is still deps_check's.
    """
    import sysconfig as _sysconfig
    import tempfile
    import startup_checks

    real = startup_checks.externally_managed_marker()
    check(real is None or os.path.isfile(real),
          f"externally_managed_marker returned {real!r}, which is not a file")

    with tempfile.TemporaryDirectory() as tmp:
        planted = os.path.join(tmp, "EXTERNALLY-MANAGED")
        with open(planted, "w") as fh:
            fh.write("[externally-managed]\n")
        saved_get_path = _sysconfig.get_path
        saved_prefix, saved_base = sys.prefix, sys.base_prefix
        try:
            _sysconfig.get_path = (
                lambda key, *a, **k: tmp if key in ("stdlib", "platstdlib")
                else saved_get_path(key, *a, **k))

            sys.prefix = sys.base_prefix = "/usr"      # a system interpreter
            check(startup_checks.externally_managed_marker() == planted,
                  "externally_managed_marker did not find a planted PEP 668 "
                  "marker, so the #944 branch can never fire")

            sys.prefix = "/usr/venv-943"              # prefix != base_prefix
            check(startup_checks.externally_managed_marker() is None,
                  "externally_managed_marker claims a venv is externally "
                  "managed; PEP 668 exempts venvs and pip installs into them "
                  "fine, so this would withhold a working one-click install "
                  "(#944)")
        finally:
            _sysconfig.get_path = saved_get_path
            sys.prefix, sys.base_prefix = saved_prefix, saved_base

    names = ['scipy', 'shapely', 'Pillow']
    apt = startup_checks.distro_command(names, startup_checks.DISTRO_PACKAGES)
    dnf = startup_checks.distro_command(names, startup_checks.FEDORA_PACKAGES)
    arch = startup_checks.distro_command(names, startup_checks.ARCH_PACKAGES)
    check('python3-pil ' not in dnf + ' ' and dnf.endswith('python3-pillow'),
          f"the Fedora spelling of Pillow is python3-pillow, got: {dnf}")
    check(apt.endswith('python3-pil'),
          f"the Debian spelling of Pillow is python3-pil, got: {apt}")
    check(arch == 'python-scipy python-shapely python-pillow',
          f"the Arch spelling is python-<name>, got: {arch}")
    for n in names:
        for label, table in (('Debian', startup_checks.DISTRO_PACKAGES),
                             ('Arch', startup_checks.ARCH_PACKAGES)):
            check(n in table,
                  f"{n} has no {label} package name, so the #944 message "
                  f"would offer `{n.lower()}`")

    # The dialog itself, through the wx stub: every distro line, and an
    # override command that survives being pasted into a shell. It used to
    # join the requirements UNQUOTED, and `numpy>=1.22.0` is a redirect there.
    shown = []
    wx_stub = deps_check.wx
    saved = {k: getattr(wx_stub, k, None)
             for k in ('MessageBox', 'OK', 'ICON_INFORMATION')}
    try:
        wx_stub.MessageBox = lambda text, *a, **k: shown.append(text)
        wx_stub.OK = wx_stub.ICON_INFORMATION = 0
        deps_check._report_externally_managed(
            None,
            [startup_checks.Problem('numpy', 'outdated', '1.21.5', '1.22.0')],
            [startup_checks.Problem('Pillow', 'absent', floor='9.2.0')],
            '/usr/lib/python3.12/EXTERNALLY-MANAGED')
    finally:
        for k, v in saved.items():
            if v is None:
                if hasattr(wx_stub, k):
                    delattr(wx_stub, k)
            else:
                setattr(wx_stub, k, v)
    text = shown[0] if shown else ''
    for want in ('sudo apt install python3-numpy python3-pil',
                 'sudo dnf install python3-numpy python3-pillow',
                 'sudo pacman -S --needed python-numpy python-pillow',
                 '--break-system-packages "numpy>=1.22.0" "Pillow>=9.2.0"',
                 '/usr/lib/python3.12/EXTERNALLY-MANAGED'):
        check(want in text,
              f"the PEP 668 dialog does not say {want!r}; it said:\n{text}")


def run():
    test_probe_matches_installed_distributions()
    test_missing_pillow_does_not_block()
    test_render_gate_is_an_import_error()
    test_render_placement_imports_without_pillow()
    test_pep668_probe_and_message()

    if FAILS:
        for f in FAILS:
            print(f"  FAIL  {f}")
        print(f"\n{len(FAILS)} check(s) FAILED")
        return False
    print("PASS  #943/#944 optional raster dependency, catchable render gate, "
          "PEP 668 install path")
    return True


if __name__ == '__main__':
    sys.exit(0 if run() else 1)

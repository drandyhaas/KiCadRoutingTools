#!/usr/bin/env python3
"""A dependency that is present but TOO OLD must be caught, named, and fixable.

Reported 2026-09-18 from a KiCad 10 install: the plugin "immediately complains
about the numpy version in spite of the fact that the numpy version on my system
is up to date", plus `TypeError: 'numpy._DTypeMeta' object is not subscriptable`.
Both are one cause -- the interpreter KiCad runs imports a numpy older than 1.22
-- and neither message comes from this repo, which is why they were unreadable:

  * `numpy._DTypeMeta.__class_getitem__` arrived in 1.22, so below it any
    package annotating `np.ndarray[Any, np.dtype[...]]` at import raises that
    TypeError (verified against 1.21.6 and 2.0.2).
  * scipy >= 1.11 warns "A NumPy version >=1.22.4 and <2.3.0 is required for
    this version of SciPy (detected version X)" -- the "numpy version"
    complaint, emitted by scipy.

Every gate probed with `except ImportError` and nothing else, and
`_parse_requirements` deliberately STRIPPED the specifier (`numpy>=1.21.0` ->
`numpy`), so a too-old package passed silently AND could not be repaired by the
one-click install: `pip install --upgrade` ran only for MISSING packages, and
pip considers a bare `numpy` satisfied by whatever is already there.

What this pins:
  1. the floor is READ from requirements.txt, is at least 1.22, and the
     stripped-install fallback is not looser than the file;
  2. an outdated package is reported -- with its version and the FILE the
     running interpreter imports, which is the answer to "but mine is up to
     date" (it is a different interpreter);
  3. the pip requirement carries the floor, so the install is not a no-op;
  4. both fronts run the SAME probe and the SAME table (identity, not equality);
  5. the version comparator orders rc/dev/short forms correctly;
  6. a satisfied dependency is NOT reported -- the negative control, without
     which every assertion above passes on a gate that flags everything.

Run with:  python3 tests/test_dependency_version_floor.py
"""
import os
import sys
import types

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))
sys.path.insert(0, os.path.join(ROOT_DIR, 'kicad_routing_plugin'))

# deps_check imports wx at module scope; nothing under test touches a wx symbol.
sys.modules.setdefault('wx', types.ModuleType('wx'))

import startup_checks as sc        # noqa: E402
import deps_check                  # noqa: E402

FAILS = []


def check(cond, msg):
    if not cond:
        FAILS.append(msg)
    return cond


def test_floor_is_read_from_requirements():
    path = sc.requirements_path()
    check(path is not None and os.path.isfile(path),
          "BROKEN TEST: requirements.txt was not found, so no floor was read")
    names = dict(sc.parse_requirements(path))
    check('numpy' in names,
          "requirements.txt no longer lists numpy at all")
    check('>=' in names.get('numpy', ''),
          f"numpy's requirements.txt entry carries no floor: {names.get('numpy')!r}")

    floors = sc.requirement_floors()
    check(sc.version_satisfies(floors.get('numpy'), '1.22'),
          f"the numpy floor is {floors.get('numpy')!r}, below the 1.22 at which "
          f"`np.dtype[...]` subscripting works -- the configuration that "
          f"produced both reported errors would still pass")

    # The stripped-install fallback must not be LOOSER than the file, or an
    # install that shipped no requirements.txt is gated more weakly than one
    # that did -- silently, since an unreadable file used to parse to an empty
    # list and an empty list is a gate that passes everything.
    for name, fallback in sc.FALLBACK_FLOORS.items():
        declared = floors.get(name)
        check(sc.version_satisfies(fallback, declared),
              f"FALLBACK_FLOORS[{name!r}] = {fallback} is below "
              f"requirements.txt's {declared} -- bump it with the file")


def test_outdated_is_reported_with_version_and_path():
    """A planted stale module must be reported as outdated, not as fine.

    Planted rather than read off this machine: the developing machine has a
    current numpy, so the one arm that matters would never run.
    """
    fake = types.ModuleType('_kr_stale_pkg')
    fake.__version__ = '1.21.6'
    fake.__file__ = '/somewhere/unexpected/_kr_stale_pkg/__init__.py'
    sys.modules['_kr_stale_pkg'] = fake
    saved_modules = dict(sc.VERSION_MODULES)
    saved_tests = dict(sc.IMPORT_TESTS)
    try:
        sc.VERSION_MODULES['_kr_stale_pkg'] = '_kr_stale_pkg'
        sc.IMPORT_TESTS['_kr_stale_pkg'] = 'import _kr_stale_pkg'

        problems = sc.dependency_problems(['_kr_stale_pkg'],
                                          floors={'_kr_stale_pkg': '1.22'})
        if not check(len(problems) == 1,
                     f"a 1.21.6 package against a 1.22 floor produced "
                     f"{len(problems)} problem(s), not 1"):
            return
        p = problems[0]
        check(p.state == 'outdated',
              f"state is {p.state!r}, not 'outdated' -- an installed-but-stale "
              f"package must not be reported as absent")
        check(p.installed == '1.21.6',
              f"the reported version is {p.installed!r}")
        check(p.path == fake.__file__,
              f"the reported path is {p.path!r}; without the real file path "
              f"the user cannot see WHICH copy is being imported")
        check('1.21.6' in p.describe() and '1.22' in p.describe(),
              f"describe() names neither the installed version nor the floor: "
              f"{p.describe()!r}")
        check(fake.__file__ in p.describe(),
              f"describe() drops the path: {p.describe()!r}")

        # 3. the pip requirement carries the floor
        check(p.requirement == '_kr_stale_pkg>=1.22',
              f"the pip requirement is {p.requirement!r}; a bare name is "
              f"already satisfied by the stale copy, so the one-click install "
              f"would report success and change nothing")

        # the interpreter is named, because that is the thing the user has not
        # checked -- KiCad's python is not the python in their terminal
        block = sc.format_problems(problems, "header:")
        check((sys.executable or '(embedded)') in block,
              "format_problems does not name the interpreter")
        check('pip install --upgrade' in block and '_kr_stale_pkg>=1.22' in block,
              f"the printed fix does not upgrade to the floor:\n{block}")

        # 4. the GUI front reaches the same verdict through its own entry point
        saved_opt = dict(deps_check.OPTIONAL_PACKAGES)
        saved_parse = deps_check._parse_requirements
        saved_floors = deps_check.requirement_floors
        try:
            deps_check._parse_requirements = lambda _p: ['_kr_stale_pkg']
            deps_check.requirement_floors = lambda: {'_kr_stale_pkg': '1.22'}
            blocking, optional = deps_check._missing_packages()
            check(blocking == ['_kr_stale_pkg'],
                  f"deps_check._missing_packages did not block on the stale "
                  f"package: blocking={blocking} optional={optional}")
        finally:
            deps_check._parse_requirements = saved_parse
            deps_check.requirement_floors = saved_floors
            deps_check.OPTIONAL_PACKAGES.clear()
            deps_check.OPTIONAL_PACKAGES.update(saved_opt)
    finally:
        sys.modules.pop('_kr_stale_pkg', None)
        sc.VERSION_MODULES.clear(); sc.VERSION_MODULES.update(saved_modules)
        sc.IMPORT_TESTS.clear(); sc.IMPORT_TESTS.update(saved_tests)


def test_satisfied_package_is_not_reported():
    """The negative control. Same planted module, version ABOVE the floor."""
    fake = types.ModuleType('_kr_fresh_pkg')
    fake.__version__ = '2.0.2'
    fake.__file__ = '/somewhere/_kr_fresh_pkg/__init__.py'
    sys.modules['_kr_fresh_pkg'] = fake
    saved_modules = dict(sc.VERSION_MODULES)
    saved_tests = dict(sc.IMPORT_TESTS)
    try:
        sc.VERSION_MODULES['_kr_fresh_pkg'] = '_kr_fresh_pkg'
        sc.IMPORT_TESTS['_kr_fresh_pkg'] = 'import _kr_fresh_pkg'
        problems = sc.dependency_problems(['_kr_fresh_pkg'],
                                          floors={'_kr_fresh_pkg': '1.22'})
        check(problems == [],
              f"a 2.0.2 package against a 1.22 floor was reported as a "
              f"problem: {problems} -- the gate flags everything, so the "
              f"outdated assertions above prove nothing")
    finally:
        sys.modules.pop('_kr_fresh_pkg', None)
        sc.VERSION_MODULES.clear(); sc.VERSION_MODULES.update(saved_modules)
        sc.IMPORT_TESTS.clear(); sc.IMPORT_TESTS.update(saved_tests)


def test_absent_stays_absent():
    """An unimportable package is 'absent', not 'outdated' -- they need
    different sentences and different commands."""
    problems = sc.dependency_problems(['_kr_no_such_pkg_0918'],
                                      floors={'_kr_no_such_pkg_0918': '1.0'})
    if check(len(problems) == 1, f"absent package produced {problems}"):
        check(problems[0].state == 'absent',
              f"state is {problems[0].state!r}, not 'absent'")
        check('not installed' in problems[0].describe(),
              f"describe() is not the absent sentence: {problems[0].describe()!r}")


def test_version_comparator():
    cases = [
        ('1.21.6', '1.22', False),
        ('1.22.0', '1.22', True),
        ('1.22', '1.22.0', True),
        ('2.0.2', '1.22', True),
        ('1.22.4rc1', '1.22.4', True),
        ('2.3.0.dev0', '1.22', True),
        ('1.9.0', '1.22', False),      # 9 > 22 as a STRING; must not be
        ('10.1.0', '9.2.0', True),     # likewise
    ]
    for installed, floor, want in cases:
        got = sc.version_satisfies(installed, floor)
        check(got is want,
              f"version_satisfies({installed!r}, {floor!r}) = {got}, want {want}")
    # Unparseable must not be the reason a program refuses to start.
    check(sc.version_satisfies('weird-build', '1.22') is True,
          "an unparseable version blocks startup")
    check(sc.version_satisfies('1.0', None) is True,
          "an absent floor blocks startup")


def test_both_fronts_share_one_table():
    """Identity, not equality: two dicts that happen to agree today are the
    hand-mirrored copies this change removed."""
    check(deps_check.IMPORT_TESTS is sc.IMPORT_TESTS,
          "deps_check.IMPORT_TESTS is a COPY of startup_checks.IMPORT_TESTS, "
          "so the two fronts can drift apart again")
    check(deps_check.dependency_problems is sc.dependency_problems,
          "deps_check does not call the shared probe")
    for name in sc.IMPORT_TESTS:
        check(name in sc.VERSION_MODULES,
              f"{name} has an import test but no VERSION_MODULES entry, so its "
              f"version is never checked")
    check('Pillow' not in sc.ROUTING_PACKAGES,
          "Pillow is in ROUTING_PACKAGES -- routing does not need the raster "
          "stack (#887), and gating on it refuses a board this can route")

    # The GUI's in-dialog probe must go through the shared function too.
    gui = os.path.join(ROOT_DIR, 'kicad_routing_plugin', 'swig_gui.py')
    with open(gui) as fh:
        text = fh.read()
    check('dependency_problems(ROUTING_PACKAGES)' in text,
          "swig_gui.py no longer calls the shared dependency probe -- it used "
          "to hand-mirror the package list, which is how it drifted")


def run():
    test_floor_is_read_from_requirements()
    test_outdated_is_reported_with_version_and_path()
    test_satisfied_package_is_not_reported()
    test_absent_stays_absent()
    test_version_comparator()
    test_both_fronts_share_one_table()

    if FAILS:
        for f in FAILS:
            print(f"  FAIL  {f}")
        print(f"\n{len(FAILS)} check(s) FAILED")
        return False
    print("PASS  dependency version floor: outdated is caught, named with its "
          "path, and fixable; both fronts share one probe")
    return True


if __name__ == '__main__':
    sys.exit(0 if run() else 1)

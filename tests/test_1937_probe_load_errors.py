#!/usr/bin/env python3
"""#1937: a package whose import raises something other than ImportError is
reported as BROKEN by the dependency probe, never raised out of it.

On Windows, importing a package whose native library clashes with a DLL already
loaded in the process fails with `OSError: [WinError 127]`, not ImportError.
The probe caught ImportError alone, so the error escaped
`deps_check.ensure_dependencies()` and stopped the plugin before its dialog
opened. #1938 stopped probing the one package that did it (ortools); this pins
the probe itself, for every package and both load modes.

What this pins:
  1. load=True (the GUI and bus gates): a planted OSError is a `broken`
     Problem carrying the error text, not an exception;
  2. load=False (the CLI gate): a top-level package that raises OSError on
     import is `broken`, and names the file it was found at;
  3. the message says what to do, which `pip install --upgrade` is not;
  4. a package that is simply not there is still `absent` -- the control.

ipc-migration: main's fourth check -- deps_check's GUI split reads a broken
required package as BLOCKING -- has no counterpart here. deps_check is the
SWIG plugin's first-launch pip dialog, deleted by the port: KiCad provisions
the IPC plugin's venv from requirements.txt itself. The probe it calls,
startup_checks.dependency_problems, is shared and pinned in full.

Run with:  python3 tests/test_1937_probe_load_errors.py
"""
import os
import shutil
import sys
import tempfile
from unittest.mock import patch

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import startup_checks                                          # noqa: E402
from startup_checks import (BROKEN_ADVICE, IMPORT_TESTS,       # noqa: E402
                            dependency_problems, format_problems)

FAILS = []
WIN127 = "raise OSError(127, 'The specified procedure could not be found')"


def check(cond, msg):
    if not cond:
        FAILS.append(msg)


def _only(problems, what):
    check(len(problems) == 1, f"{what}: expected one Problem, got {problems}")
    return problems[0] if problems else None


def test_load_true_reports_broken():
    with patch.dict(IMPORT_TESTS, {'_krt_dll_clash': WIN127}):
        try:
            problems = dependency_problems(['_krt_dll_clash'], floors={},
                                           load=True)
        except Exception as exc:                               # noqa: BLE001
            FAILS.append(f"load=True raised {type(exc).__name__}: {exc}")
            return
    p = _only(problems, "load=True")
    if p is None:
        return
    check(p.state == 'broken', f"load=True state {p.state!r}, want 'broken'")
    check('OSError' in (p.error or '') and 'procedure' in (p.error or ''),
          f"load=True error text lost: {p.error!r}")
    check('fails to load' in p.describe(),
          f"describe() does not say it fails to load: {p.describe()!r}")


def test_load_false_reports_broken_with_path():
    tmp = tempfile.mkdtemp(prefix='krt1937_')
    try:
        pkg = os.path.join(tmp, '_krt_broken_pkg')
        os.makedirs(pkg)
        init = os.path.join(pkg, '__init__.py')
        with open(init, 'w') as fh:
            fh.write(WIN127 + "\n")
        sys.path.insert(0, tmp)
        try:
            for load in (False, True):
                sys.modules.pop('_krt_broken_pkg', None)
                try:
                    problems = dependency_problems(['_krt_broken_pkg'],
                                                   floors={}, load=load)
                except Exception as exc:                       # noqa: BLE001
                    FAILS.append(f"load={load} raised "
                                 f"{type(exc).__name__}: {exc}")
                    continue
                p = _only(problems, f"load={load} package")
                if p is None:
                    continue
                check(p.state == 'broken',
                      f"load={load} package state {p.state!r}, want 'broken'")
                check(p.path and os.path.samefile(p.path, init),
                      f"load={load} names {p.path!r}, want {init}")
        finally:
            sys.path.remove(tmp)
            sys.modules.pop('_krt_broken_pkg', None)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_message_says_what_to_do():
    with patch.dict(IMPORT_TESTS, {'_krt_dll_clash': WIN127}):
        problems = dependency_problems(['_krt_dll_clash'], floors={})
    text = format_problems(problems, "header")
    check(BROKEN_ADVICE in text, "format_problems omits the broken advice")
    check('--force-reinstall' in BROKEN_ADVICE,
          "the advice does not name the reinstall pip actually needs")


def test_absent_stays_absent():
    for load in (True, False):
        problems = dependency_problems(['_krt_not_installed_1937'],
                                       floors={}, load=load)
        p = _only(problems, f"absent load={load}")
        if p is not None:
            check(p.state == 'absent',
                  f"a missing package reads {p.state!r} at load={load}")


def run():
    test_load_true_reports_broken()
    test_load_false_reports_broken_with_path()
    test_message_says_what_to_do()
    test_absent_stays_absent()
    check(startup_checks.Problem('x', 'broken', error='E').describe()
          .startswith('  x: installed but fails to load (E)'),
          "Problem.describe() format for 'broken' changed")

    if FAILS:
        for f in FAILS:
            print(f"  FAIL  {f}")
        print(f"\n{len(FAILS)} check(s) FAILED")
        return False
    print("PASS  #1937: a package that fails to load is reported broken, "
          "never raised out of the probe")
    return True


if __name__ == '__main__':
    sys.exit(0 if run() else 1)

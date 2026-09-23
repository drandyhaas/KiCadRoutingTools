#!/usr/bin/env python3
"""#1026: install_plugin.py on a PEP 668 interpreter.

KiCad's Linux packages run the system python, which Arch, Debian 12+, Ubuntu
23.04+ and Fedora 38+ mark EXTERNALLY-MANAGED. pip refuses there before it
resolves anything, so the installer printed pip's refusal and "run as
Administrator" on a machine whose packages were all installed already. The GUI
had been fixed for the same interpreter in #944; the installer had not.

  A. The out-of-process dependency probe (contributed in PR #1026) answers
     what the runtime gate answers -- including for a requirement with no
     floor, which that probe's first version dropped and so reported as
     satisfied whether it was installed or not.
  B. The marker probe asks the TARGET interpreter, not this one: a marker
     planted in the target (a sitecustomize on PYTHONPATH) is found, and this
     process's own answer is untouched.
  C. install_dependencies end to end, with the real probes and only pip
     intercepted:
       satisfied           -> True, pip never runs
       missing + PEP 668   -> False, pip never runs -- the distro commands and
                              a QUOTED override are printed, and nothing says
                              "Administrator"
       missing, no marker  -> pip runs once, its output NOT captured (the user
                              watches the download) and never with
                              --break-system-packages
       probe failed + PEP 668 -> every requirement named, none claimed absent

Run with:  python3 tests/test_1026_installer_pep668.py
"""
import contextlib
import io
import os
import subprocess
import sys
import tempfile
from pathlib import Path

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))   # #522

import install_plugin  # noqa: E402
import startup_checks  # noqa: E402

# Seconds, not minutes: the probes are two short python subprocesses per arm.
RUN_ALL_FAST_OK = True

FAILS = []

# An importable module name that is never installed. (A hyphenated one would
# be a SyntaxError inside the probe, not an ImportError, and prove nothing.)
ABSENT = 'krt_absent_probe_1026'


def check(cond, msg):
    if not cond:
        FAILS.append(msg)
    return cond


@contextlib.contextmanager
def patched(obj, **attrs):
    saved = {k: getattr(obj, k) for k in attrs}
    for k, v in attrs.items():
        setattr(obj, k, v)
    try:
        yield
    finally:
        for k, v in saved.items():
            setattr(obj, k, v)


@contextlib.contextmanager
def target_stdlib(with_marker):
    """Make every python this test STARTS a non-venv whose stdlib is a temp dir,
    holding a PEP 668 marker or not. Returns the marker path (or None).

    Out-of-process on purpose: the installer asks KiCad's interpreter, so a
    plant in THIS process would test the wrong interpreter. Both arms are
    planted so neither depends on what the host python happens to be.
    """
    with tempfile.TemporaryDirectory() as tmp:
        marker = os.path.join(tmp, 'EXTERNALLY-MANAGED')
        if with_marker:
            with open(marker, 'w') as fh:
                fh.write('[externally-managed]\n')
        with open(os.path.join(tmp, 'sitecustomize.py'), 'w') as fh:
            fh.write(
                "import sys, sysconfig\n"
                f"_TMP = {tmp!r}\n"
                "_get = sysconfig.get_path\n"
                "sysconfig.get_path = (lambda key, *a, **k: _TMP\n"
                "    if key in ('stdlib', 'platstdlib') else _get(key, *a, **k))\n"
                "sys.base_prefix = sys.prefix\n")
        saved = os.environ.get('PYTHONPATH')
        os.environ['PYTHONPATH'] = tmp + (os.pathsep + saved if saved else '')
        try:
            yield marker if with_marker else None
        finally:
            if saved is None:
                os.environ.pop('PYTHONPATH', None)
            else:
                os.environ['PYTHONPATH'] = saved


@contextlib.contextmanager
def intercepted_pip():
    """Let the probes run for real; record every `-m pip` call instead."""
    calls = []
    real = subprocess.run

    def fake(argv, *a, **k):
        if list(argv[1:3]) == ['-m', 'pip']:
            calls.append((list(argv), dict(k)))
            return subprocess.CompletedProcess(argv, 0, '', '')
        return real(argv, *a, **k)

    with patched(install_plugin.subprocess, run=fake):
        yield calls


def install(floors, **overrides):
    """Run install_dependencies against THIS interpreter with `floors`.

    Returns (result, stdout, pip calls).
    """
    out = io.StringIO()
    with patched(install_plugin,
                 get_kicad_python=lambda: Path(sys.executable),
                 requirement_floors=lambda: dict(floors),
                 **overrides), \
            intercepted_pip() as calls, contextlib.redirect_stdout(out):
        result = install_plugin.install_dependencies()
    return result, out.getvalue(), calls


# ---------------------------------------------------------------------------
# A. the dependency probe answers what the runtime gate answers
# ---------------------------------------------------------------------------
def test_probe_agrees_with_runtime_gate():
    floors = dict(startup_checks.requirement_floors())
    floors[ABSENT] = None                      # no floor, and not installed
    want = {p.name: p.state for p in
            startup_checks.dependency_problems(list(floors), floors)}
    present = [n for n in floors if n not in want]
    if present:                                # a floor nobody can meet
        floors[present[0]] = '999'
        want[present[0]] = 'outdated'

    problems = install_plugin._target_dependency_problems(sys.executable,
                                                          floors)
    if not check(problems is not None,
                 "BROKEN TEST: the out-of-process probe did not run at all, "
                 "so nothing was compared"):
        return
    got = {p.name: p.state for p in problems}
    check(got.get(ABSENT) == 'absent',
          f"a requirement with NO floor that is not installed must be "
          f"reported absent; the probe said {got.get(ABSENT)!r} -- the "
          f"installer would print 'already satisfied' and skip pip")
    check(got == want,
          f"the installer's probe disagrees with the runtime gate "
          f"(startup_checks.dependency_problems): probe {got}, gate {want}")
    if not present:
        print("  note: no requirement is installed here, so the too-old arm "
              "was not exercised")


# ---------------------------------------------------------------------------
# B. the marker probe asks the target
# ---------------------------------------------------------------------------
def test_marker_probe_asks_the_target():
    with target_stdlib(with_marker=True) as marker:
        got = install_plugin._target_externally_managed(sys.executable)
        check(got == marker,
              f"a PEP 668 marker planted in the TARGET interpreter was not "
              f"found (got {got!r}), so the installer would run pip into an "
              f"interpreter that refuses it")
        check(startup_checks.externally_managed_marker() != marker,
              "the plant leaked into this process, so the arm above did not "
              "prove the question was asked out-of-process")
    with target_stdlib(with_marker=False):
        got = install_plugin._target_externally_managed(sys.executable)
        check(got is None,
              f"a target with no marker was reported managed ({got!r}), so "
              f"the installer would withhold a pip install that works")


# ---------------------------------------------------------------------------
# C. install_dependencies, end to end with pip intercepted
# ---------------------------------------------------------------------------
def test_satisfied_skips_pip():
    with target_stdlib(with_marker=True):
        result, out, calls = install({'json': None})
    check(result is True and not calls,
          f"everything importable on a PEP 668 python must succeed WITHOUT "
          f"running pip (the #1026 report); got {result!r}, pip calls "
          f"{calls}\n{out}")
    check('already satisfied' in out,
          f"the skip should say why pip did not run:\n{out}")


def test_pep668_names_distro_packages_and_runs_no_pip():
    floors = {ABSENT: '1.0', 'json': None}
    with target_stdlib(with_marker=True) as marker:
        result, out, calls = install(floors)
    check(result is False,
          f"nothing was installed, so this is not a success: {result!r}")
    check(not calls,
          f"pip ran on a PEP 668 interpreter: {calls}. Either it fails, or -- "
          f"with --break-system-packages -- it writes into the prefix the "
          f"distro's package manager owns, which #944 left to the user")
    for want in (marker,
                 f'{ABSENT}: not installed',
                 'sudo apt install ',
                 'sudo dnf install ',
                 'sudo pacman -S --needed ',
                 f'--break-system-packages "{ABSENT}>=1.0"'):
        check(want in out, f"the PEP 668 report does not say {want!r}:\n{out}")
    check('json' not in out.split('Install the distribution')[-1],
          f"an installed requirement was offered for install:\n{out}")
    check('Administrator' not in out,
          f"'run as Administrator' is the wrong advice on a PEP 668 python -- "
          f"sudo pip is refused the same way:\n{out}")


def test_unmanaged_target_runs_pip_live():
    with target_stdlib(with_marker=False):
        result, out, calls = install({ABSENT: None})
    if not check(len(calls) == 1,
                 f"a missing package on an ordinary python must run pip "
                 f"once; pip calls: {calls}\n{out}"):
        return
    argv, kwargs = calls[0]
    check('--break-system-packages' not in argv,
          f"pip was given --break-system-packages on its own: {argv}")
    captured = {'capture_output', 'stdout', 'stderr'} & set(kwargs)
    check(not captured,
          f"pip's output is captured ({sorted(captured)}), so a first install "
          f"-- scipy alone is ~20 MB -- runs silent until it finishes")
    check(result is True, f"pip exited 0, so this is a success: {result!r}")


def test_failed_probe_names_everything():
    floors = {'numpy': '1.22.0', ABSENT: None}
    with target_stdlib(with_marker=True):
        result, out, calls = install(
            floors, _target_dependency_problems=lambda exe, fl: None)
    check(result is False and not calls,
          f"got {result!r}, pip calls {calls}\n{out}")
    check('Could not probe' in out and 'not installed' not in out,
          f"with no probe answer nothing may be CLAIMED absent:\n{out}")
    check('"numpy>=1.22.0"' in out and f'"{ABSENT}"' in out,
          f"with no probe answer every requirement must be named:\n{out}")


def run():
    test_probe_agrees_with_runtime_gate()
    test_marker_probe_asks_the_target()
    test_satisfied_skips_pip()
    test_pep668_names_distro_packages_and_runs_no_pip()
    test_unmanaged_target_runs_pip_live()
    test_failed_probe_names_everything()

    if FAILS:
        for f in FAILS:
            print(f"  FAIL  {f}")
        print(f"\n{len(FAILS)} check(s) FAILED")
        return False
    print("PASS  #1026 installer on a PEP 668 python: probe first, distro "
          "advice instead of pip, live pip everywhere else")
    return True


if __name__ == '__main__':
    sys.exit(0 if run() else 1)

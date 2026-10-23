#!/usr/bin/env python3
"""The kwarg-parity gate checks its Class 2 when launched with plain python3
(found reviewing #1272).

tests/gui_parity/test_engine_kwarg_parity.py has two halves. Class 1 (a kwarg
the CLI passes and the GUI call does not) is static. Class 2 (a config key a
GUI call reads that no builder supplies -- a dead control) builds the real
dialog, so it needs wx + pcbnew. The gate never looked for KiCad's python:
`python3 tests/gui_parity/test_engine_kwarg_parity.py` printed
"SKIP (needs KiCad python ...)" under "ENGINE KWARG PARITY: OK" and exited 0
-- on Windows every time, with KiCad installed -- while every sibling gate
re-execs. Rows:

  absent     no python with wx + pcbnew exists (both blocked, and the shared
             launcher made to find none): exit 77 with a "SKIP: Class 2 NOT
             RUN" line, never 0 -- and Class 1 still runs and is reported.
             Fails on the old gate (exit 0).
  absent+c1  control: the same launch on a copy whose Planes tab drops a kwarg
             the CLI passes exits 1 for THAT divergence; a Class 1 failure is
             never hidden behind the skip.
  direct     control: launched with KiCad's python itself, the gate runs both
             classes and exits 0, as it always did.
  precond    the unmutated copy passes the python3 launch (else the next row
             could fail for a reason of the copy's own).
  dead       the negative control: the copy with a dead Planes control (the
             Basic tab's ripup_blocker_select no longer supplied under its
             name, the #424 shape) FAILS the python3 launch, for that reason.
             Fails on the old gate (Class 2 skipped, exit 0).
  reexec     the real tree, launched with this python: Class 2's engine rows
             are printed, and the whole report is the one `direct` printed.

Rows that need KiCad's python self-skip (exit 77) when none exists.

    python3 tests/test_engine_kwarg_parity_launch.py
"""
import glob
import io
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
from run_utils import check  # noqa: E402

GATE_REL = os.path.join('tests', 'gui_parity', 'test_engine_kwarg_parity.py')
SKIP_EXIT = 77

# What the dialog needs to construct, copied (not the corpus boards).
COPY_DIRS = ('py_router', 'py_tools', 'py_placer', 'kicad_routing_plugin')
COPY_FILES = ('krt_capabilities.py', 'krt_registry.py', 'VERSION',
              'metadata.json', 'kicad_files/splitflap_driver.kicad_pcb',
              GATE_REL)

# The Planes tab's get_shared_params supplies ripup_blocker_select (#424). The
# anchor is the 16-space dict line; the Route tab's builder spells it at 12.
DEAD_FILE = 'kicad_routing_plugin/swig_gui.py'
DEAD_OLD = ("                'ripup_blocker_select': "
            "self.ripup_blocker_select.GetString(")
DEAD_NEW = DEAD_OLD.replace("'ripup_blocker_select'", "'ripup_blocker_select_dead'")
# A kwarg route_planes.py passes create_plane, dropped from the Planes tab.
C1_FILE = 'kicad_routing_plugin/planes_gui.py'
C1_OLD = "skip_existing_zones=config.get('skip_existing_zones', False),"

# Run a gate as if no python with wx + pcbnew existed: both imports blocked,
# and the shared launcher (headless_plan.reexec_into_kicad) finding nothing.
ABSENT_WRAPPER = '''import os, runpy, sys
gate = sys.argv[1]
sys.modules['wx'] = None
sys.modules['pcbnew'] = None
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(gate)))), 'py_router'))
import headless_plan
headless_plan.reexec_into_kicad = lambda *a, **k: False
sys.argv = [gate]
runpy.run_path(gate, run_name='__main__')
'''

FAILS = []


def row(name, fn):
    try:
        fn()
        print(f"  ok    {name}")
    except AssertionError as e:
        FAILS.append(name)
        print(f"  FAIL  {name}\n{e}")


def _src(path):
    with io.open(path, encoding='utf-8', newline='') as f:
        return f.read()


def _put(path, text):
    with io.open(path, 'w', encoding='utf-8', newline='') as f:
        f.write(text)


def _mutated(tree, rel, old, new):
    """Context: `rel` in `tree` with `old` (exactly one occurrence) -> `new`."""
    class _M:
        def __enter__(self):
            self.path = os.path.join(tree, rel)
            self.before = _src(self.path)
            n = self.before.count(old)
            assert n == 1, (f"mutation anchor occurs {n}x in {rel}, not once -- "
                            f"the control would test nothing")
            _put(self.path, self.before.replace(old, new))

        def __exit__(self, *exc):
            _put(self.path, self.before)
            return False
    return _M()


def kicad_python():
    """A python with wx + pcbnew: this one, else the first the shared
    launcher's candidates (headless_plan.KICAD_PYTHONS) verify. None if none."""
    probe = 'import pcbnew, wx'
    if subprocess.run([sys.executable, '-c', probe],
                      capture_output=True).returncode == 0:
        return sys.executable
    import headless_plan
    for cand in headless_plan.KICAD_PYTHONS:
        if cand and cand != sys.executable and os.path.isfile(cand) and \
                subprocess.run([cand, '-c', probe],
                               capture_output=True).returncode == 0:
            return cand
    return None


def report(out):
    """The gate's report: CLASS 1 header through the verdict, without the
    wx chatter a dialog prints on stderr (which varies run to run)."""
    lines = out.splitlines()
    start = next(i for i, ln in enumerate(lines) if ln.startswith('CLASS 1'))
    return [ln for ln in lines[start:] if ln.startswith(('  ', 'CLASS', 'ENGINE',
                                                          'COPLANAR', '#511', '='))]


def copy_tree(dst):
    ign = shutil.ignore_patterns('__pycache__', '*.pyc')
    for d in COPY_DIRS:
        shutil.copytree(os.path.join(ROOT, d), os.path.join(dst, d), ignore=ign)
    # The router binary, under whichever name this platform imports.
    binaries = [os.path.relpath(p, ROOT) for p in
                glob.glob(os.path.join(ROOT, 'rust_router', 'grid_router*'))
                if os.path.isfile(p)]
    for f in COPY_FILES + tuple(binaries):
        src = os.path.join(ROOT, f)
        if os.path.isfile(src):
            os.makedirs(os.path.dirname(os.path.join(dst, f)) or dst, exist_ok=True)
            shutil.copy(src, os.path.join(dst, f))


def main():
    env = None      # the children inherit this environment, KICAD_PYTHON too
    kp = kicad_python()
    print(f"KiCad python: {kp or 'none found'}")
    gate = os.path.join(ROOT, GATE_REL)
    tmp = tempfile.mkdtemp(prefix='kwarg_launch_')
    try:
        tree = os.path.join(tmp, 'tree')
        copy_tree(tree)
        copy_gate = os.path.join(tree, GATE_REL)
        wrapper = os.path.join(tmp, 'absent.py')
        _put(wrapper, ABSENT_WRAPPER)

        def absent():
            r = check([sys.executable, '-X', 'utf8', wrapper, gate],
                      refuse='SKIP: Class 2 NOT RUN', code=SKIP_EXIT,
                      allow=('ImportError', 'ModuleNotFoundError'), env=env)
            assert 'CLASS 1' in r.stdout and \
                '  batch_route              OK' in r.stdout, \
                f"Class 1 did not run under the skip:\n{r.stdout[-1500:]}"
            assert 'ENGINE KWARG PARITY: OK' not in r.stdout, \
                "the skip still prints the OK verdict"
        row('absent: no KiCad python -> exit 77 with a SKIP line, Class 1 run',
            absent)

        def absent_c1():
            with _mutated(tree, C1_FILE, C1_OLD, ''):
                check([sys.executable, '-X', 'utf8', wrapper, copy_gate],
                      refuse="CLI passes 'skip_existing_zones'", code=1,
                      allow=('ImportError', 'ModuleNotFoundError'), env=env)
        row('absent+c1: a Class 1 divergence exits 1 under the skip', absent_c1)

        if kp is None:
            print("SKIP: no python with wx + pcbnew found; the rows that launch "
                  "the dialog (direct, precond, dead, reexec) need one.")
            return 1 if FAILS else SKIP_EXIT

        seen = {}

        def direct():
            r = check([kp, '-X', 'utf8', gate], accept=True, env=env)
            assert '  create_plane             OK  (' in r.stdout, \
                f"Class 2 did not run under KiCad's python:\n{r.stdout[-1500:]}"
            seen['direct'] = report(r.stdout)
        row("direct: control -- KiCad's python itself, exit 0, Class 2 run",
            direct)

        def precond():
            r = check([sys.executable, '-X', 'utf8', copy_gate], accept=True,
                      env=env)
            assert '  create_plane             OK  (' in r.stdout, \
                f"the copy's Class 2 did not run:\n{r.stdout[-1500:]}"
        row('precond: the unmutated copy passes the python3 launch', precond)

        def dead():
            with _mutated(tree, DEAD_FILE, DEAD_OLD, DEAD_NEW):
                check([sys.executable, '-X', 'utf8', copy_gate], code=1, env=env,
                      refuse="reads config['ripup_blocker_select'] for kwarg "
                             "'ripup_blocker_select' but no builder supplies it")
        row('dead: a dead Planes control fails the python3 launch', dead)

        def reexec():
            r = check([sys.executable, '-X', 'utf8', gate], accept=True, env=env)
            for fn in ('batch_route_diff_pairs', 'create_plane',
                       'generate_bga_fanout', 'generate_qfn_fanout'):
                assert f'  {fn:24} OK  (' in r.stdout, \
                    f"Class 2 row for {fn} missing:\n{r.stdout[-2000:]}"
            assert 'SKIP' not in r.stdout, r.stdout[-2000:]
            got, want = report(r.stdout), seen.get('direct')
            assert want is None or got == want, (
                "the python3 launch reports differently from KiCad's python:\n"
                + "\n".join(got) + "\n---\n" + "\n".join(want))
        row("reexec: the python3 launch runs Class 2 and reports as KiCad's "
            "python does", reexec)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('=' * 60)
    if FAILS:
        print(f"FAILED ({len(FAILS)}): " + "; ".join(FAILS))
        return 1
    print("PASS: the kwarg-parity gate runs Class 2 from a python3 launch, and "
          "skips (77) rather than passes without KiCad's python")
    return 0


if __name__ == '__main__':
    sys.exit(main())

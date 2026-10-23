#!/usr/bin/env python3
"""test_505_min_connection_clamp's GUI (pcbnew) half runs under run_all's
python3 (found reviewing #1272).

The half pins the GUI writeback's m_MinConn clamp. Under a python without
pcbnew it printed "[skip]" and the file passed -- on every box, since run_all
launches python3 and the cloud suite has no KiCad -- so it ran nowhere. Rows:

  launch    plain python3: the GUI half runs (under KiCad's python) and its
            lines are reported. bb5943a0: "[skip]", exit 0.
  broken    negative control: a tree copy whose GUI writeback no longer maps
            min_connection -> m_MinConn fails the python3 launch for THAT
            check -- the child's failure reaches the verdict.
  absent    no python with pcbnew (pcbnew blocked, the finder finding none):
            exit 77 with a "SKIP:" line, never 0 -- and the file-writeback
            half still ran. bb5943a0: exit 0.
  direct    control: KiCad's python itself, exit 0, as before.

Rows that need KiCad's python self-skip (exit 77) when none exists.

    python3 tests/test_505_gui_half_runs.py
"""
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

TARGET_REL = os.path.join('tests', 'test_505_min_connection_clamp.py')
SKIP_EXIT = 77
GUI_LINE = 'pcbnew exposes m_MinConn'
PURE_LINE = 'stale web rule 0.127 clamped to the routed floor'

# The GUI writeback's map (8-space indent; the project writer's is at 4).
BROKEN_FILE = os.path.join('py_router', 'fix_kicad_drc_settings.py')
BROKEN_OLD = '        "min_connection": "m_MinConn",'
BROKEN_NEW = '        "min_connection": "m_MinConn_gone",'

ABSENT_WRAPPER = '''import os, runpy, sys
target = sys.argv[1]
sys.modules['pcbnew'] = None
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(target))), 'py_router'))
import kicad_exact_fill
kicad_exact_fill.find_kicad_python = lambda: None
sys.argv = [target]
runpy.run_path(target, run_name='__main__')
'''

FAILS = []


def row(name, fn):
    try:
        fn()
        print(f"  ok    {name}")
    except AssertionError as e:
        FAILS.append(name)
        print(f"  FAIL  {name}\n{e}")


def _put(path, text):
    with io.open(path, 'w', encoding='utf-8', newline='') as f:
        f.write(text)


def main():
    import kicad_exact_fill
    probe = subprocess.run([sys.executable, '-c', 'import pcbnew'],
                           capture_output=True).returncode == 0
    kp = sys.executable if probe else kicad_exact_fill.find_kicad_python()
    print(f"KiCad python: {kp or 'none found'}")
    target = os.path.join(ROOT, TARGET_REL)
    tmp = tempfile.mkdtemp(prefix='t505_gui_')
    try:
        wrapper = os.path.join(tmp, 'absent.py')
        _put(wrapper, ABSENT_WRAPPER)

        def absent():
            r = check([sys.executable, '-X', 'utf8', wrapper, target],
                      refuse='SKIP: the GUI (pcbnew) half did not run',
                      code=SKIP_EXIT)
            assert f'ok  {PURE_LINE}' in r.stdout, \
                f"the file-writeback half did not run:\n{r.stdout[-1500:]}"
        row('absent: no python with pcbnew -> exit 77 with a SKIP line',
            absent)

        if kp is None:
            print("SKIP: no python with pcbnew found; the launch, broken and "
                  "direct rows need one.")
            return 1 if FAILS else SKIP_EXIT

        def direct():
            r = check([kp, '-X', 'utf8', target], accept=True)
            assert f'ok  {GUI_LINE}' in r.stdout and 'ALL PASS' in r.stdout, \
                r.stdout[-1500:]
        row("direct: control -- KiCad's python itself, exit 0", direct)

        def launch():
            r = check([sys.executable, '-X', 'utf8', target], accept=True)
            assert f'ok  {GUI_LINE}' in r.stdout, \
                f"the GUI half did not run:\n{r.stdout[-1500:]}"
            assert '[skip]' not in r.stdout and 'ALL PASS' in r.stdout, \
                r.stdout[-1500:]
        row('launch: plain python3 runs the GUI half', launch)

        def broken():
            tree = os.path.join(tmp, 'tree')
            ign = shutil.ignore_patterns('__pycache__', '*.pyc')
            for d in ('py_router', 'py_tools'):
                shutil.copytree(os.path.join(ROOT, d), os.path.join(tree, d),
                                ignore=ign)
            os.makedirs(os.path.join(tree, 'tests'))
            shutil.copy(target, os.path.join(tree, TARGET_REL))
            path = os.path.join(tree, BROKEN_FILE)
            with io.open(path, encoding='utf-8', newline='') as f:
                src = f.read()
            assert src.count(BROKEN_OLD) == 1, \
                "mutation anchor is not unique -- the control would test nothing"
            _put(path, src.replace(BROKEN_OLD, BROKEN_NEW))
            check([sys.executable, '-X', 'utf8', os.path.join(tree, TARGET_REL)],
                  code=1, refuse="FAIL GUI path run under KiCad's python")
        row('broken: a GUI writeback that drops m_MinConn fails the python3 '
            'launch', broken)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('=' * 60)
    if FAILS:
        print(f"FAILED ({len(FAILS)}): " + "; ".join(FAILS))
        return 1
    print("PASS: test_505's GUI half runs under a python3 launch, and skips "
          "(77) rather than passes without pcbnew")
    return 0


if __name__ == '__main__':
    sys.exit(main())

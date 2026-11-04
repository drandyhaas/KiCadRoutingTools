#!/usr/bin/env python3
"""#1617: grade_final's `fully_connected` is check_connected's verdict.

grade_final spawned `<repo>/check_connected.py`, which the #522 reorg moved
to py_router/. The child exited 2 with `can't open file` on stderr, the
stdout grade_final searched was empty, and every board -- connected or not --
graded `"fully_connected": false`. Its check_drc fallback built the path the
same way. What each case pins:

* a board check_connected passes grades `fully_connected: true`, and one it
  fails grades `false` (the negative control: the flag is not just `true`);
* a grader that did not run is not a verdict: with check_connected's path
  pointed at a missing file, `fully_connected` is None and `conn_err` names
  the grader and its exit, rather than `false`.

    python3 tests/test_1617_grade_final_connected.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

import grade_final  # noqa: E402
from run_utils import evidence  # noqa: E402

CONNECTED = os.path.join(ROOT, 'kicad_files', 'qfn_underpad_coupling.kicad_pcb')
OPEN = os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb')

FAILS = []


def check(name, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {name}" + (f": {detail}" if detail and not ok else ''))
    if not ok:
        FAILS.append(name)


def stage(src, tmp, name):
    d = os.path.join(tmp, name)
    os.makedirs(d)
    subprocess.run([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'copy_board.py'),
                    src, os.path.join(d, 'final.kicad_pcb')], check=True, capture_output=True)
    evidence(os.path.join(d, 'final.kicad_pcb'), 'staged board')
    with open(os.path.join(d, 'result.json'), 'w') as fh:
        fh.write('{}')
    return d


def grade(d):
    """grade_final.main() in-process, so a case can swap its tool resolver."""
    argv = sys.argv
    sys.argv = ['grade_final.py', d, os.path.join(d, 'result.json')]
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            grade_final.main()
    finally:
        sys.argv = argv
    with open(evidence(os.path.join(d, 'authoritative_grade.json'), 'grade')) as fh:
        return json.load(fh)


def main():
    tmp = tempfile.mkdtemp(prefix='t1617_')
    try:
        g = grade(stage(CONNECTED, tmp, 'connected'))
        check('a board check_connected passes grades fully_connected true',
              g.get('fully_connected') is True, json.dumps(g)[:300])
        g = grade(stage(OPEN, tmp, 'open'))
        check('a board check_connected fails grades fully_connected false',
              g.get('fully_connected') is False, json.dumps(g)[:300])

        real = grade_final._tool_path
        grade_final._tool_path = lambda root, name: (
            os.path.join(tmp, 'missing', name) if name == 'check_connected.py' else real(root, name))
        try:
            g = grade(stage(CONNECTED, tmp, 'no_grader'))
        finally:
            grade_final._tool_path = real
        check('a grader that did not run grades None, not false',
              g.get('fully_connected') is None, json.dumps(g)[:300])
        err = g.get('conn_err') or ''
        check('conn_err names the grader and its exit',
              'check_connected.py exited 2' in err, err)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if FAILS:
        print(f"FAIL: {len(FAILS)}: " + '; '.join(FAILS))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())

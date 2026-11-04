#!/usr/bin/env python3
"""#1479: an intent's `arrays[]` rows are graded, never seated, and every
reader says so.

Seating a declared row rigidly was tried three ways (routed, paired over
seeds) and routed worse on each, so no placement step forms rows:
`check_floorplan` grades whatever the placement left (`array_formation`).
Before this the rows changed nothing and nothing said so.

  1. `arrays.graded_only_warning`: one line for N declared rows, None for none;
  2. `check_floorplan --emit-intent` prints it when a brief compiles a row
     into the intent (here one the detector suggests on the in-repo
     splitflap_driver), and not without one;
  3. `place_simple` prints the same line (it reads the intent's rows).

    python3 tests/test_1479_arrays_graded_only.py
"""
import ast
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_placer'))
sys.path.insert(0, TESTS)
from run_utils import evidence  # noqa: E402
from placement import arrays  # noqa: E402

RUN_ALL_TIMEOUT = 300
BOARD = os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb')
PY = [sys.executable, '-B', '-X', 'utf8']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def run(argv):
    return subprocess.run(PY + argv, capture_output=True, text=True, cwd=REPO)


def main():
    print('1. the one line')
    graded_only_warning = getattr(arrays, 'graded_only_warning', None)
    if graded_only_warning is None:
        check('placement.arrays.graded_only_warning exists', False)
        graded_only_warning = lambda rows: None  # noqa: E731
    w = graded_only_warning([{'members': ['R1', 'R2']}, {'members': ['R3', 'R4']}])
    check('two rows: named as graded only', w is not None and w.startswith(
        'arrays[]: 2 declared row(s) are graded only'), str(w))
    check('no rows: nothing to say', graded_only_warning([]) is None
          and graded_only_warning(None) is None)

    print('2. check_floorplan --emit-intent')
    td = tempfile.mkdtemp(prefix='t1479_')
    try:
        board = os.path.join(td, 'b.kicad_pcb')
        r = run([os.path.join('py_router', 'copy_board.py'), BOARD, board])
        evidence(board)
        sugg = run([os.path.join('py_tools', 'check_floorplan.py'), board,
                    '--suggest-arrays'])
        rows = json.loads(sugg.stdout)['suggestions']
        check('the detector suggests a row on the board', bool(rows),
              sugg.stderr[-300:])
        out = os.path.join(td, 'none.json')
        e0 = run([os.path.join('py_tools', 'check_floorplan.py'), board,
                  '--emit-intent', out])
        check('no row declared: no warning', 'arrays[]' not in e0.stdout,
              e0.stdout[-300:])
        with open(os.path.join(td, 'b.design-brief.json'), 'w') as fh:
            json.dump({'schema': 1, 'kind': 'design-brief', 'units': 'mm',
                       'arrays': [rows[0]['row']]}, fh)
        out = os.path.join(td, 'rows.json')
        e1 = run([os.path.join('py_tools', 'check_floorplan.py'), board,
                  '--emit-intent', out])
        evidence(out)
        with open(out) as fh:
            n = len(json.load(fh).get('arrays') or [])
        check('the brief row reaches the intent', n == 1, f'{n} row(s)')
        check('...and the emit says it is graded only',
              'WARNING arrays[]: 1 declared row(s) are graded only' in e1.stdout,
              e1.stdout[-400:])
    finally:
        shutil.rmtree(td, ignore_errors=True)

    print('3. place_simple')
    src = open(os.path.join(REPO, 'py_placer', 'place_simple.py')).read()
    calls = [c for c in ast.walk(ast.parse(src)) if isinstance(c, ast.Call)
             and getattr(c.func, 'id', None) == 'graded_only_warning']
    check('place_simple prints the line from the intent\'s rows', len(calls) == 1,
          f'{len(calls)} call(s)')

    print('FAILED: ' + ', '.join(failures) if failures else 'ALL PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

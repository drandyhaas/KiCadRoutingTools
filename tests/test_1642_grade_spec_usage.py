#!/usr/bin/env python3
"""#1642: the placement skill's grade.py on a --spec its graders cannot take.

A misspelt `--spec min_track_width=0.09` reached board_score and both
check_complete runs, which died on `unrecognized arguments` (exit 2). Their
message went to a discarded log, the missing score read as an empty one, and
grade.py printed `{"done": false, "blocking": null}` and exited 4 -- the code
of a board that graded and is not done. In `--mode place`, which runs neither
grader, the same spec was ignored and the board read DONE. What each case pins:

* a --spec board_score or check_complete would refuse -- a misspelt name, a
  flag only one of them takes, a flag grade.py sets itself, a value of the
  wrong type -- is refused with the reader's own reason, exit 2, before any
  grader runs (no grade file is written);
* a --spec with --mode place is refused (exit 2), not ignored;
* a correctly spelled spec passes the check (the negative control);
* a grader that exits 2 anyway makes grade.py exit 2, naming it and its
  error line, with `done: null` in the file and no verdict line printed.

    python3 tests/test_1642_grade_spec_usage.py
"""
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement', 'scripts')
GRADE = os.path.join(SCRIPTS, 'grade.py')
sys.path.insert(0, SCRIPTS)
sys.path.insert(0, os.path.join(ROOT, 'tests'))

import grade  # noqa: E402
import run_utils  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb')
PY = [sys.executable, '-X', 'utf8']

FAILS = []


def check(name, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {name}" + (f": {detail}" if detail and not ok else ''))
    if not ok:
        FAILS.append(name)


def refused_up_front(td, args, reason, name):
    td = tempfile.mkdtemp(dir=td)
    try:
        run_utils.check(PY + [GRADE, BOARD, '--baseline', BOARD, '--out-dir', td,
                              '--label', 'x', *args],
                        refuse=reason, code=2,
                        allow=('usage:', 'unrecognized arguments', 'error: argument'))
        wrote = [f for f in os.listdir(td) if f.startswith('grade_')]
        check(name, not wrote, f'a refused spec still graded: {wrote}')
    except AssertionError as e:
        check(name, False, str(e)[:600])


def main():
    run_utils.evidence(BOARD)
    with tempfile.TemporaryDirectory() as td:
        refused_up_front(td, ['--spec', 'min_track_width=0.09'],
                         'board_score would refuse it: '
                         'unrecognized arguments: --min_track_width',
                         'a misspelt spec name is refused before any grader runs')
        refused_up_front(td, ['--spec', 'size-margin=0.01'],
                         'check_complete would refuse it',
                         'a flag only board_score takes is refused (check_complete would die)')
        refused_up_front(td, ['--spec', 'json=x.json'],
                         '--json is set by grade.py itself',
                         "a flag grade.py passes itself is refused")
        refused_up_front(td, ['--spec', 'min-track-width=thin'],
                         "invalid float value: 'thin'",
                         'a value of the wrong type is refused with the reader\'s reason')
        refused_up_front(td, ['--mode', 'place', '--spec', 'min-track-width=0.09'],
                         '--mode place would ignore it',
                         '--spec with --mode place is refused, not ignored')

    ok = grade.spec_problem(['--min-track-width', '0.09', '--net-min-widths', 'w.json',
                             '--impedance-nets', 'A*', '--impedance-nets', 'B*'])
    check('a correctly spelled spec passes the check', ok is None, str(ok))

    # The grader-exit-2 path, past the up-front check: a spec that reaches
    # board_score and check_complete and kills both on usage.
    with tempfile.TemporaryDirectory() as td:
        real = grade.spec_problem
        grade.spec_problem = lambda _args: None
        err, out = io.StringIO(), io.StringIO()
        try:
            with contextlib.redirect_stderr(err), contextlib.redirect_stdout(out):
                rc = grade.main([BOARD, '--baseline', BOARD, '--out-dir', td,
                                 '--label', 'u', '--spec', 'min_track_width=0.09'])
        finally:
            grade.spec_problem = real
        e = err.getvalue()
        check('a grader that exits 2 makes grade.py exit 2', rc == 2, f'rc {rc}')
        check('stderr names each grader and its error line',
              'board_score.py exited 2' in e and 'check_complete.py exited 2' in e
              and 'unrecognized arguments: --min_track_width' in e, e[-600:])
        check('no verdict line is printed', '"done"' not in out.getvalue(), out.getvalue())
        with open(run_utils.evidence(os.path.join(td, 'grade_u.json'), 'grade file')) as fh:
            doc = json.load(fh)
        check('the grade file says done: null, not false', doc.get('done') is None,
              str(doc.get('done')))

    if FAILS:
        print(f"FAIL: {len(FAILS)}: " + '; '.join(FAILS))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())

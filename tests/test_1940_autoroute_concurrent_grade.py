"""#1940: autoroute grades a board with both checkers at once.

`Chain.grade` ran the connectivity probe, waited, then ran check_drc. Both
only read the board, and the probe runs with the KiCad cross-check off, so
they now run side by side and the grade is read from them in the order it
always was.

  1. the grade of an in-repo board equals the one the two checkers give when
     run one after the other;
  2. the two children overlap: check_drc is started before the probe ends;
  3. a probe that fails gives None with its one message, as before, leaves no
     report file behind and no child running;
  4. `Chain.check_drc` alone still returns the report.
"""
import contextlib
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'py_router'))

import run_utils  # noqa: E402
import autoroute  # noqa: E402

FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


class Recorder:
    """subprocess.Popen, recording when each child starts and is reaped."""
    def __init__(self):
        self.rows = []
        self.real = subprocess.Popen

    def __call__(self, argv, *a, **k):
        proc = self.real(argv, *a, **k)
        row = {'kind': 'drc' if any('check_drc.py' in str(x) for x in argv) else 'conn',
               'start': time.monotonic(), 'end': None, 'proc': proc}
        self.rows.append(row)
        for name in ('wait', 'communicate'):
            orig = getattr(proc, name)

            def wrapped(*aa, _orig=orig, _row=row, **kk):
                out = _orig(*aa, **kk)
                if _row['end'] is None:
                    _row['end'] = time.monotonic()
                return out
            setattr(proc, name, wrapped)
        return proc


def sequential(ch, board):
    """The grade as the two checkers give it one after the other."""
    env = autoroute._child_env()
    env['KICAD_NO_GRADE_RECONCILE'] = '1'
    r = subprocess.run([sys.executable, '-X', 'utf8', '-c', autoroute._CONN_PROBE,
                        autoroute.HERE, board], capture_output=True, text=True, env=env)
    m = re.search(r'^AUTOROUTE_OPEN: (\[.*\])$', r.stdout, re.M)
    assert r.returncode == 0 and m, f"BROKEN TEST: the probe failed\n{r.stderr[-800:]}"
    d = ch.check_drc(board)
    assert d is not None, "BROKEN TEST: check_drc failed"
    return json.loads(m.group(1)), int(d['violations'])


def main():
    td = tempfile.mkdtemp(prefix='krt1940_')
    try:
        board = os.path.join(td, 'b.kicad_pcb')
        subprocess.run([sys.executable, os.path.join(REPO, 'py_router', 'copy_board.py'),
                        os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb'),
                        board], capture_output=True, cwd=REPO)
        run_utils.evidence(board)
        ch = autoroute.Chain(board, td, clearance=0.2)

        print('1-2. the grade, and the overlap')
        want_open, want_drc = sequential(ch, board)
        rec = Recorder()
        autoroute.subprocess.Popen = rec
        try:
            got = ch.grade(board)
        finally:
            autoroute.subprocess.Popen = rec.real
        ok(got is not None and got['open_nets'] == want_open and got['drc'] == want_drc
           and got['open'] == len(want_open),
           f"the grade is the sequential one: open {got and got['open']} / {len(want_open)}, "
           f"drc {got and got['drc']} / {want_drc}")
        ok(len(want_open) > 0, f"the board has open nets to report (non-vacuity): {len(want_open)}")
        by = {r['kind']: r for r in rec.rows}
        ok(set(by) == {'drc', 'conn'} and len(rec.rows) == 2,
           f"one probe and one check_drc were started: {[r['kind'] for r in rec.rows]}")
        ok(all(r['end'] is not None for r in rec.rows), 'both children were reaped')
        ok('drc' in by and 'conn' in by and by['drc']['start'] < by['conn']['end'],
           'check_drc was started before the probe ended')

        print('3. a probe that fails')
        before = set(os.listdir(td))
        probe = autoroute._CONN_PROBE
        autoroute._CONN_PROBE = 'import sys; sys.exit(3)'
        rec = Recorder()
        autoroute.subprocess.Popen = rec
        buf = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf):
                got = ch.grade(board)
        finally:
            autoroute.subprocess.Popen = rec.real
            autoroute._CONN_PROBE = probe
        ok(got is None, 'the grade is None')
        ok(buf.getvalue().count('grade:') == 1
           and 'check_connected failed' in buf.getvalue(),
           f"with the probe's one message: {buf.getvalue().strip()!r}")
        ok(all(r['proc'].poll() is not None for r in rec.rows), 'no child is left running')
        ok(set(os.listdir(td)) == before, 'no report file is left behind')

        print('4. check_drc alone')
        d = ch.check_drc(board)
        ok(d is not None and int(d['violations']) == want_drc,
           f"returns the report: {d and d.get('violations')}")
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if FAILS:
        print(f'FAIL: {len(FAILS)}: ' + '; '.join(FAILS))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())

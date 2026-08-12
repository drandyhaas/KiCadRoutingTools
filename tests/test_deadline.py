"""The deadline contract: a stage that stops itself must SAY so, and a partial
must be exactly as fatal as no result at all.

Run 9 lost two stages to an external `timeout`: no output board, no
JSON_SUMMARY, and an exit code belonging to the shell rather than the tool. The
fix lets a stage stop on its own budget -- which introduces a NEW hazard the
tests below exist to pin: a partial summary PARSES, so a consumer that only
checked "is there a summary" would now silently accept an unfinished run's
tallies as a whole board's. That would be quieter, and worse, than the failure
it replaced.

    python3 -X utf8 tests/test_deadline.py
"""
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout

BAD = []


def want(cond, label):
    print(f"  {'PASS' if cond else 'FAIL'}  {label}")
    if not cond:
        BAD.append(label)


def test_deadline_primitives():
    import krt_deadline as K
    K.reset_for_test()
    want(K.arm(None, tool='t') is None,
         'no budget -> arm() returns None, so callers pay nothing')
    K.reset_for_test()
    d = K.arm(0, tool='t')
    want(d is not None and d.expired(),
         '--deadline 0 is a real budget (stop now), not an absent one')
    K.reset_for_test()
    d = K.Deadline(1.0, tool='t')
    want(d.expired(reserve=1.0) and not d.expired(),
         'the reserve band trips early while the raw deadline has clock left')
    K.reset_for_test()
    d = K.Deadline(0.0, tool='t')
    cc = d.cancel_check('repair')
    want(callable(cc) and cc() and d.stopped_in == 'repair',
         'cancel_check() is a zero-arg closure and records the phase')
    K.reset_for_test()
    want(K.emit({'a': 1}) and not K.emit({'a': 2}),
         'emit() fires at most once (a second JSON_SUMMARY would be merged '
         'as a later pass and overwrite the real tally)')
    K.reset_for_test()
    want(K.emit({'x': 1}) is True, 'emit() is callable after reset')
    K.reset_for_test()
    r = {}
    K.mark(r, K.Deadline(1.0, tool='t'), staged_board='s.kicad_pcb')
    want(r['complete'] is False and r['status'] == 'deadline'
         and r['partial']['staged_board'] == 's.kicad_pcb',
         'mark() stamps complete:false, status:"deadline" and the partial')
    want(K.DEADLINE_EXIT not in (0, 4, 124),
         'the deadline exit code collides with neither the shell nor "measured '
         'defect"')


def test_partial_is_as_fatal_as_missing():
    """The regression the deadline work could have introduced."""
    from route_summary import merge_summaries
    part = {'complete': False, 'status': 'deadline', 'stopped_in': 'repair',
            'failed_single': ['A'], 'elapsed_s': 61.0, 'deadline_s': 60}
    full = {'complete': True, 'status': 'ok', 'failed_single': []}
    m = merge_summaries([part, full])
    want(m.get('complete') is False,
         'a complete LATER pass cannot whitewash a partial earlier one')
    want(m.get('stopped_in') == 'repair',
         'the merged summary keeps WHY it was partial')
    m2 = merge_summaries([{'failed_single': []}, {'failed_single': []}])
    want(m2.get('complete', True) is True,
         'pre-deadline logs (no `complete` key) still read as complete')

    src = open(os.path.join(ROOT, 'py_placer', 'place_route_loop.py'), encoding='utf-8').read()
    want("summary.get('complete', True)" in src,
         'place_route_loop REJECTS a partial summary -- a partial parses, so '
         '"is there a summary" is no longer a sufficient check')


def _reconstruct(board, out, *extra):
    return subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(ROOT, 'py_placer', 'place_reconstruct.py'), board, out,
         '--stages', 'legalize', *extra],
        capture_output=True, text=True, encoding='utf-8',
        errors='replace', cwd=ROOT, timeout=900)


def _summaries(stdout):
    return [json.loads(l.split('JSON_SUMMARY: ', 1)[1])
            for l in stdout.splitlines() if l.startswith('JSON_SUMMARY: ')]


def test_a_deadline_is_only_observable_when_there_is_work():
    """A budget of 0 on a board with NOTHING to legalize still completes.

    This is not a loophole, it is the contract: the check sits at the head of
    the violator loop, so a board with no violators never reaches it and has
    nothing to abandon. Worth pinning, because the obvious test ("--deadline 0
    must always produce a partial") asserts the wrong thing and fails against
    correct behaviour -- it did, when this file was first written.
    """
    board = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
    if not os.path.isfile(board):
        print('  SKIP  no corpus board'); return
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, 'o.kicad_pcb')
        p = _reconstruct(board, out, '--deadline', '0')
        want(p.returncode != 124,
             'the tool chose its own exit code (124 would be the shell)')
        want(len(_summaries(p.stdout)) == 1,
             'exactly one JSON_SUMMARY, always')
        doc = (_summaries(p.stdout) or [{}])[0]
        want(doc.get('complete') is True,
             'a healthy board with no violators COMPLETES at --deadline 0 -- '
             'there was no work to abandon')
        want(os.path.exists(out),
             'and its output board is written normally')
        want('DEADLINE:' in p.stdout,
             'the DEADLINE: breadcrumb prints (its absence is the standing '
             'diagnosis for a run with no budget)')


def test_a_board_with_work_reports_the_partial():
    """The case run 9 actually hit: violators exist and the clock runs out."""
    board = os.path.join(ROOT, 'wk', 'run9', 'ottercast', 'board_recon.kicad_pcb')
    if not os.path.isfile(board):
        print('  SKIP  no damaged board on disk (wk/ is not committed)'); return
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, 'o.kicad_pcb')
        p = _reconstruct(board, out, '--clearance', '0.2', '--deadline', '0')
        doc = (_summaries(p.stdout) or [{}])[0]
        want(doc.get('complete') is False,
             'a board WITH violators reports complete:false')
        want(doc.get('status') == 'deadline' and doc.get('stopped_in'),
             'and says it was the deadline, and in which stage')
        want(doc.get('partial', {}).get('deadline_skipped'),
             'the untried violators are named -- NOT filed as unrepairable, '
             'which would report an unfinished search as a measured failure')
        want(not os.path.exists(out),
             'the OUTPUT path is not written -- run-7 A11: it only ever holds '
             'a complete board')
        want(os.path.exists(out + '.staging.kicad_pcb'),
             'the partial board is left at the staging path and named in the '
             'summary, so it is findable without being mistaken for a result')


def test_stages_are_validated():
    # Via run_utils.check, which separates "the guard held" from "my control
    # broke". A bare `returncode == 2` here would also pass on an argparse
    # typo, an ImportError or a missing board -- and `--stages` validation is
    # itself argparse-adjacent, so that confusion is one keystroke away.
    # `allow` because p.error() IS an argparse usage message: it is the
    # expected mechanism here, not an accident.
    sys.path.insert(0, os.path.join(ROOT, 'tests'))
    from run_utils import check
    try:
        check([sys.executable, '-X', 'utf8',
               os.path.join(ROOT, 'py_placer', 'place_reconstruct.py'), 'a.kicad_pcb',
               'b.kicad_pcb', '--stages', 'legalise'],
              refuse='unknown stage', code=2, allow=('error: argument',))
        ok = True
    except AssertionError as e:
        ok = False
        print('   ' + str(e)[:300])
    want(ok, 'a misspelt --stages is an ERROR naming the valid stages, not a '
             'silent no-op that gates nothing and exits 0')


if __name__ == '__main__':
    for fn in (test_deadline_primitives, test_partial_is_as_fatal_as_missing,
               test_a_deadline_is_only_observable_when_there_is_work,
               test_a_board_with_work_reports_the_partial,
               test_stages_are_validated):
        print(fn.__name__)
        fn()
    print('FAIL: %d' % len(BAD) if BAD else 'OK')
    sys.exit(1 if BAD else 0)

#!/usr/bin/env python3
"""The attempts band's Y axis makes the search's progress readable (#946, #1036).

A search spends most of its laps in a narrow WORKING RANGE and a few attempts
far outside it. Run 32 opens at blocking 12703 and spends ~200 laps between
19 and 43: on the symlog axis this band used first, all of those laps sat in
the top ~7% of the plot and the record's drops 41 -> 38 -> 33 -> 32 -> 30
were invisible. The axis is now BROKEN -- the working range gets the plot,
the outliers a thin strip under a break mark.

Pinned here, on the run-32 ledger when a machine has it (self-skipped and
SAID otherwise) and always on a synthetic track with one huge outlier:

  * the working-range laps (graded attempts up to the 90th percentile) span
    at least half the plot height;
  * the record step line visibly descends across the working range (at least
    three distinct record rows, each 2+ px apart);
  * CONTROL: with the axis switched back to 'symlog' or 'linear' the same
    check FAILS -- a check both old axes pass would prove nothing.

And `join_tracks` no longer repeats a half's "ungraded" clause in its note.
"""
import json
import os
import sys
import tempfile

RUN_ALL_FAST_OK = True

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from PIL import Image, ImageDraw
except ImportError as exc:
    print('SKIP: needs Pillow (%s)' % exc)
    sys.exit(77)

import frame_layout as FL          # noqa: E402
import movie_attempts as MA        # noqa: E402

RUN32 = os.path.join(ROOT, 'wk', 'run32')

_FAIL = []
_NOTES = []


def fail(msg):
    _FAIL.append(msg)
    print('  FAIL: %s' % msg)


def _synthetic():
    """One pile-sized outlier, a few mid-size drops, then a long working range
    with a descending record -- the shape run 32 has."""
    scores = [9000, 450, 300, 260] + [48 - (i // 12) for i in range(150)]
    rows = tuple(MA.Attempt(i, 'l%d' % i, 'completion', i - 1 if i else None,
                            (i % 3) != 1, False, float(s), s == 0, None)
                 for i, s in enumerate(scores))
    return MA.Track(rows, 'blocking (lower better)', 'converge', 'synthetic')


def _measure(track, w=1400, h=126):
    dbg = {}
    im = Image.new('RGB', (w, h))
    if not MA.draw_track(ImageDraw.Draw(im), FL.Box(0, 0, w, h), track,
                         debug=dbg):
        return None
    px0, py0, px1, py1 = dbg['plot']
    graded = sorted(v for v, _y in dbg['ys'])
    # the working range, derived HERE rather than read back from the drawer
    hi = graded[min(len(graded) - 1, int(0.9 * len(graded)))]
    ys = [y for v, y in dbg['ys'] if v <= hi]
    span = (max(ys) - min(ys)) / float(py1 - py0) if ys else 0.0
    # the record's rows inside the working range
    recs = [r for _i, r in MA.best_so_far(track.attempts) if r <= hi]
    yof = dict(dbg['ys'])
    rys = sorted({round(yof.get(r, -1), 1) for r in recs if r in yof})
    steps = sum(1 for a, b in zip(rys, rys[1:]) if b - a >= 2.0)
    return span, steps + 1 if rys else 0, dbg.get('mode')


def _check(track, name, expect_pass=True):
    got = _measure(track)
    if got is None:
        fail('BROKEN: %s: the band declined' % name)
        return None
    span, rows, mode = got
    ok = span >= 0.5 and rows >= 3
    if expect_pass and not ok:
        fail('%s (%s axis): working laps span %.0f%% of the plot, the record '
             'shows %d row(s) there' % (name, mode, 100 * span, rows))
    return ok, span, rows, mode


def test_the_working_range_gets_the_plot():
    _mark = len(_FAIL)
    tracks = [('synthetic', _synthetic())]
    led = os.path.join(RUN32, 'ledger.jsonl')
    if os.path.isfile(led):
        tracks.append(('run-32 ledger', MA.attempts_from_converge_ledger(led)))
    else:
        _NOTES.append('run-32 ledger absent: checked on the synthetic track '
                      'only')
        print('    (run-32 ledger absent: synthetic track only)')
    for name, t in tracks:
        got = _check(t, name)
        if got:
            print('    %-14s %s axis: working laps span %.0f%%, record %d rows'
                  % (name, got[3], 100 * got[1], got[2]))
        # CONTROL: both rejected axes must FAIL the same check
        saved = MA.AXIS_MODE
        try:
            for mode in ('symlog', 'linear'):
                MA.AXIS_MODE = mode
                c = _check(t, name, expect_pass=False)
                if c and c[0]:
                    fail('BROKEN: %s passes on the %s axis too (span %.0f%%) '
                         '-- the check cannot see the defect'
                         % (name, mode, 100 * c[1]))
                elif c:
                    print('    %-14s CONTROL %s: span %.0f%%, record %d rows '
                          '-> fails, as it must' % (name, mode, 100 * c[1],
                                                   c[2]))
        finally:
            MA.AXIS_MODE = saved
    if len(_FAIL) == _mark:
        print('  PASS: the working range gets >= 50% of the plot and the '
              'record descends across it; symlog and linear both fail')


def test_a_joined_note_states_ungraded_once():
    _mark = len(_FAIL)
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, 'ledger.jsonl')
        rows = [(0, None, 'a', True, 9), (1, 'a', 'b', False, None),
                (2, None, 'c', True, 4)]
        with open(p, 'w', encoding='utf-8') as f:
            for it, par, res, acc, b in rows:
                f.write(json.dumps({'iteration': it, 'kind': 'completion',
                                    'parent_sha': par, 'result_sha': res,
                                    'accepted': acc,
                                    'score': {'blocking': b}}) + '\n')
        led = MA.attempts_from_converge_ledger(p)
    loop = MA.Track((MA.Attempt(0, 'r0', 'round', None, True, False, 5.0,
                                False, None),
                     MA.Attempt(1, 'r1', 'round', 0, False, False, None,
                                False, None)),
                    'failures (lower better)', 'loop', 'x')
    j = MA.join_tracks(led, loop)
    if j.note.count('ungraded') != 1:
        fail('the joined note states "ungraded" %d times: %r'
             % (j.note.count('ungraded'), j.note))
    if 'last-accepted' not in j.note or '#1034' not in j.note:
        fail('the joined note lost the lineage disclosure: %r' % j.note)
    if len(_FAIL) == _mark:
        print('  PASS: %s' % j.note)


TESTS = (
    test_the_working_range_gets_the_plot,
    test_a_joined_note_states_ungraded_once,
)


def main():
    for fn in TESTS:
        print('%s:' % fn.__name__)
        fn()
    if _FAIL:
        print('')
        print('%d FAILURE(S)' % len(_FAIL))
        for m in _FAIL:
            print('  - %s' % m)
        return 1
    print('')
    print('all %d checks passed%s' % (len(TESTS), ('; NOTE: ' + '; '.join(
        _NOTES)) if _NOTES else ''))
    return 0


if __name__ == '__main__':
    sys.exit(main())

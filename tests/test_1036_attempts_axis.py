#!/usr/bin/env python3
"""The attempts band's Y axis makes the search's progress readable (#946, #1036).

A search spends most of its laps in a narrow WORKING RANGE and a few attempts
far outside it. Run 32 opens at blocking 12703 and spends ~200 laps between
19 and 43, and the record's drops there (41 -> 38 -> 33 -> 32 -> 30) are the
part worth seeing. The axis is BROKEN -- the working range gets the plot, the
outliers a thin strip under a break mark.

Pinned here, on the run-32 ledger when a machine has it (self-skipped and
SAID otherwise) and always on a synthetic track with one huge outlier:

  * the working-range laps (graded attempts up to the 90th percentile) span
    at least half the plot height;
  * the record step line visibly descends across the working range (at least
    three distinct record rows, each 2+ px apart);
  * CONTROL: the same check FAILS on a plain linear axis (the band's own
    fallback, drawn by `_draw_track(_mode='linear')`) and on a log axis
    computed here from the same values -- a check either of those passed
    would prove nothing about the working range.

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


def _draw_dbg(track, mode=None, w=1400, h=126):
    """The drawer's debug record: `draw_track` (the broken axis), or the
    linear fallback itself when `mode='linear'`."""
    dbg = {}
    im = Image.new('RGB', (w, h))
    d, box = ImageDraw.Draw(im), FL.Box(0, 0, w, h)
    ok = (MA._draw_track(d, box, track, debug=dbg, _mode=mode) if mode
          else MA.draw_track(d, box, track, debug=dbg))
    return dbg if ok else None


def _log_axis(dbg):
    """CONTROL: the same values on a log10(1 + v) axis over the same plot --
    computed HERE, so the shipped drawer carries no axis it does not use."""
    import math
    px0, py0, px1, py1 = dbg['plot']
    vals = [v for v, _y in dbg['ys']]
    f = [math.log10(1.0 + max(0.0, v)) for v in vals]
    fmin, fmax = min(f), max(f)
    if fmax - fmin < 1e-12:
        return None
    ys = [(v, py0 + (py1 - py0) * ((fv - fmin) / (fmax - fmin)))
          for v, fv in zip(vals, f)]
    return dict(dbg, ys=ys, mode='log (control)')


def _measure(track, mode=None, dbg=None):
    dbg = dbg if dbg is not None else _draw_dbg(track, mode)
    if dbg is None:
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


def _check(track, name, expect_pass=True, mode=None, dbg=None):
    got = _measure(track, mode=mode, dbg=dbg)
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
        # CONTROL: a linear axis and a log axis must both FAIL the check
        lin = _draw_dbg(t, mode='linear')
        for ctl, dbg in (('linear', lin),
                         ('log', _log_axis(lin) if lin else None)):
            if dbg is None:
                fail('BROKEN: %s: the %s control could not be drawn'
                     % (name, ctl))
                continue
            c = _check(t, name, expect_pass=False, dbg=dbg)
            if c and c[0]:
                fail('BROKEN: %s passes on the %s axis too (span %.0f%%) '
                     '-- the check cannot see the defect'
                     % (name, ctl, 100 * c[1]))
            elif c:
                print('    %-14s CONTROL %s: span %.0f%%, record %d rows '
                      '-> fails, as it must' % (name, ctl, 100 * c[1], c[2]))
    if len(_FAIL) == _mark:
        print('  PASS: the working range gets >= 50% of the plot and the '
              'record descends across it; linear and log both fail')


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


class _Rec(object):
    def __init__(self, d):
        self._d, self.texts = d, []

    def text(self, xy, txt, *a, **kw):
        self.texts.append(txt)
        return self._d.text(xy, txt, *a, **kw)

    def __getattr__(self, k):
        return getattr(self._d, k)


def _track(scores):
    rows = tuple(MA.Attempt(i, 'l%d' % i, 'completion', i - 1 if i else None,
                            True, False, float(s), False, None)
                 for i, s in enumerate(scores))
    return MA.Track(rows, 'blocking (lower better)', 'converge', 't')


def _draw(t, w=900, h=140):
    im = Image.new('RGB', (w, h))
    rec = _Rec(ImageDraw.Draw(im))
    dbg = {}
    ok = MA.draw_track(rec, FL.Box(0, 0, w, h), t, debug=dbg)
    return ok, dbg, rec.texts, im


def test_negative_scores_draw_and_a_band_is_never_blank():
    """#1036 review: log10(1 + v) raised on a negative working range, the
    except swallowed it and the reserved band stayed EMPTY; and the break
    guard misfired once hi_w < 0."""
    _mark = len(_FAIL)
    cases = {
        '-100..-10 then 5': [float(-100 + i) for i in range(0, 91, 3)] + [5],
        'all negative': [float(-5000)] + [float(-100 + i)
                                          for i in range(0, 91, 3)],
        'negative + huge outlier': [float(-100 + i % 10)
                                    for i in range(40)] + [5000.0],
    }
    for name, sc in cases.items():
        ok, dbg, _t, _im = _draw(_track(sc))
        if not ok:
            fail('%s: the band did not draw' % name)
            continue
        print('    %-24s -> %s axis' % (name, dbg.get('mode')))
    ok, dbg, _t, _im = _draw(_track(cases['negative + huge outlier']))
    if dbg.get('mode') != 'broken':
        fail('a negative range with a huge outlier did not break: %r'
             % dbg.get('mode'))
    # the fallback: make the broken path fail, and the band must still draw
    saved = MA.STRIP_FRAC
    MA.STRIP_FRAC = 'not a number'
    try:
        ok, dbg, _t, _im = _draw(_synthetic())
    finally:
        MA.STRIP_FRAC = saved
    if not ok or dbg.get('mode') != 'linear':
        fail('a failing axis left the band %s (mode %r)'
             % ('drawn' if ok else 'BLANK', dbg.get('mode')))
    if len(_FAIL) == _mark:
        print('  PASS: negative ranges draw; a failing broken axis falls back '
              'to linear rather than leaving the band blank')


def test_tick_precision_follows_the_span():
    """#1036 review: a 0.12..0.9 axis was labelled 0 / 1 / 1."""
    _mark = len(_FAIL)
    sc = [0.9 - 0.004 * i for i in range(200)] + [50.0]
    ok, dbg, texts, _im = _draw(_track(sc))
    # the ticks are drawn first, then the caption, then the record labels
    ci = next((i for i, t in enumerate(texts) if 'axis broken' in t),
              len(texts))
    ticks = texts[:ci]
    if not ok:
        fail('BROKEN: the band did not draw')
        return
    if len(set(ticks)) != len(ticks) or any(t in ('0', '1') for t in ticks):
        fail('the ticks are not distinct at this span: %r' % ticks)
    cap = [t for t in texts if 'axis broken above' in t]
    if not cap or '.' not in cap[0].split('above')[1]:
        fail('the caption does not share the tick precision: %r' % cap)
    if len(_FAIL) == _mark:
        print('  PASS: ticks %r; caption %r' % (ticks, cap[0][:60]))


TESTS = (
    test_the_working_range_gets_the_plot,
    test_a_joined_note_states_ungraded_once,
    test_negative_scores_draw_and_a_band_is_never_blank,
    test_tick_precision_follows_the_span,
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

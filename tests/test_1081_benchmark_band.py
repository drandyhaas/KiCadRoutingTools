#!/usr/bin/env python3
"""The stage3d benchmark band (#1081): one curve, a DONE line, and records
ranked the way the run ranks them.

What this file pins:

  * **the DONE predicate** (`ledger_score.row_done`): blocking 0, nothing
    unknown, no lens FAILed, and a score about THIS board. An ABSENT lens is
    not a failure (ordinary laps carry none); a FAIL lens keeps the whole band
    above the line; a placement lap is never "working" -- it is not a routed
    board;
  * **the ranking below the line is `converge._score_key`'s quality half**, on
    a shuffled ledger -- the film's records ARE the run's records;
  * **a lap that ties on vias and wins on copper is a record**, labelled with
    the term that decided it (`copper -9.5 mm`);
  * **never a weighted sum**: fewer vias beats less copper, and lower blocking
    beats any quality;
  * **gold needs a WORKING benchmark and a STRICT beat**: a tie is labelled
    "matches", a benchmark that is itself broken earns no gold and the caption
    says why, and no benchmark means no line, no gold, and a caption saying
    "no benchmark board";
  * **the band draws what it claims** -- the DONE line, the crossing chip and
    the gold marker are found in the PIXELS, not only in the debug record;
  * **degradation is said**: no ledger, no box, a box too small -- the frames
    come back untouched with a reason.
"""
import json
import math
import os
import random
import sys
import tempfile

RUN_ALL_FAST_OK = True

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router'),
           os.path.join(ROOT, 'py_placer')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from PIL import Image, ImageDraw
except ImportError as exc:
    print('SKIP: needs Pillow (%s)' % exc)
    sys.exit(77)

import frame_layout as FL                                       # noqa: E402
import ledger_score as LS                                       # noqa: E402
import movie_benchmark as MB                                    # noqa: E402

_FAIL = []
T0 = 1.7e9


def _check(ok, msg):
    print('  %s %s' % ('ok  ' if ok else 'FAIL', msg))
    if not ok:
        _FAIL.append(msg)


def _ledger(rows):
    p = os.path.join(tempfile.mkdtemp(prefix='t1081b_'), 'ledger.jsonl')
    with open(p, 'w', encoding='utf-8') as f:
        for r in rows:
            f.write((r if isinstance(r, str) else json.dumps(r)) + '\n')
    return p


def _row(i, b, vias=None, copper=None, segs=None, kind='completion',
         accepted=True, **kw):
    q = {}
    if vias is not None:
        q['vias'] = vias
    if copper is not None:
        q['copper_mm'] = copper
    if segs is not None:
        q['segments'] = segs
    r = {'iteration': i, 'kind': kind, 'accepted': accepted,
         't': T0 + 100 * i, 'score': {'blocking': b, 'quality': q}}
    r.update(kw)
    return r


def _run():
    """A placement-then-routing run that crosses the line at lap 4 and then
    improves: vias 10 -> 10 (copper wins) -> 8."""
    return [_row(0, 40, kind='placement'), _row(1, 12, kind='placement'),
            _row(2, 5, 30, 900.0, 400), _row(3, 2, 20, 800.0, 400),
            _row(4, 0, 10, 700.0, 300), _row(5, 0, 10, 690.5, 300),
            _row(6, 0, 12, 100.0, 100, accepted=False),
            _row(7, 0, 8, 750.0, 320)]


def test_the_done_predicate():
    cases = [
        (_row(0, 0, 1), True, 'blocking 0, no lenses: working (measured)'),
        (_row(0, 1, 1), False, 'blocking 1: not working'),
        (_row(0, 0, 1, kind='placement'), None,
         'a placement lap is never a working PCB'),
        (_row(0, True, 1), None, 'a bool blocking is ungraded'),
        (_row(0, float('nan'), 1), None, 'NaN blocking is ungraded'),
        (_row(0, 0, 1, lenses=['VERDICT=FAIL:lens=drc']), False,
         'a FAIL lens is not working'),
        (_row(0, 0, 1, lenses=['VERDICT=PASS:lens=drc',
                               'VERDICT=FAIL:lens=DRC']), False,
         'FAIL wins over PASS for one lens, case-folded'),
        (dict(_row(0, 0, 1), score={'blocking': 0, 'unknown': ['impedance']}),
         False, 'something unknown is not working'),
        (_row(0, 0, 1, score_stale={'binding': 'other'}), False,
         'a score about another board is not working'),
        ('not a row', None, 'a non-object row says nothing'),
    ]
    for row, want, why in cases:
        got = LS.row_done(row)
        _check(got is want, '%s (%r)' % (why, got))
    final = _row(0, 0, 1, final=True, lenses=[
        'VERDICT=PASS:lens=connectivity', 'VERDICT=PASS:lens=drc',
        'VERDICT=PASS:lens=spec'])
    _check(LS.done_evidence(final) == 'verified',
           'a --final row with every lens PASS is verified')
    _check(LS.done_evidence(_row(0, 0, 1)) == 'measured',
           'an ordinary blocking-0 row is measured, not verified')


def test_quality_key_is_the_runs_own_ranking():
    import converge
    rng = random.Random(1081)
    vals = [0, 1, 7, 7.5, -0.0, True, None, 'x', float('nan'),
            float('inf'), 10 ** 400, 3.25]
    bad = 0
    for _ in range(400):
        q = {k: rng.choice(vals) for k in LS.TERMS if rng.random() < 0.85}
        sc = {'blocking': 0, 'quality': q}
        ck = converge._score_key(sc)
        if ck is None:
            continue
        if tuple(ck[1]) != LS.quality_key(sc):
            bad += 1
            if bad < 4:
                print('    diverge on %r: converge %r, film %r'
                      % (q, ck[1], LS.quality_key(sc)))
    _check(bad == 0, 'quality_key == converge._score_key quality on 400 '
           'shuffled documents (%d diverge)' % bad)
    _check(LS.quality_key({'quality': 'x'}) == (math.inf,) * 3,
           'a non-dict quality ranks last on every term')


def test_records_are_lexicographic_and_labelled():
    tr = MB.from_converge_ledger(_ledger(_run()))
    pl = MB.plan(tr)
    idx = {i: pl.order[i].index for i in range(len(pl.order))}
    recs = [(idx[i], lab) for i, lab in pl.records]
    _check(pl.done_at is not None and idx[pl.done_at] == 4,
           'the line is crossed at lap 4 (%s)'
           % (pl.done_at is not None and idx[pl.done_at]))
    below = [r for r in recs if r[0] >= 4]
    _check(below == [(4, ''), (5, 'copper -9.5 mm'), (7, 'vias -2')],
           'records below the line: %s' % below)
    _check(6 not in [r[0] for r in recs],
           'a REJECTED lap is never a record, however good its key')
    above = [r[0] for r in recs if r[0] < 4]
    _check(above == [0, 1, 2, 3], 'placement and routing laps are one '
           'blocking staircase above the line: %s' % above)


def test_never_a_weighted_sum():
    # fewer vias beats far less copper; lower blocking beats any quality
    rows = [_row(0, 1, 2, 10.0, 10), _row(1, 0, 50, 5000.0, 900),
            _row(2, 0, 40, 9000.0, 999), _row(3, 0, 40, 8999.0, 999)]
    pl = MB.plan(MB.from_converge_ledger(_ledger(rows)))
    recs = [pl.order[i].index for i, _l in pl.records]
    _check(pl.order[pl.done_at].index == 1,
           'blocking 1 with 2 vias is not working; blocking 0 with 50 is')
    _check(recs[-2:] == [2, 3],
           'vias 50 -> 40 is a record despite +4000 mm copper, and the '
           'copper tie-break then decides (%s)' % recs)


def test_a_fail_lens_keeps_the_band_above_the_line():
    rows = [_row(0, 3, 9), _row(1, 0, 8, lenses=['VERDICT=FAIL:lens=spec']),
            _row(2, 0, 7, lenses=['VERDICT=FAIL:lens=drc'])]
    pl = MB.plan(MB.from_converge_ledger(_ledger(rows)))
    _check(pl.done_at is None, 'no crossing while every blocking-0 lap has a '
           'FAILed lens')


def test_gold_needs_a_working_benchmark_and_a_strict_beat():
    tr = MB.from_converge_ledger(_ledger(_run()))
    ok = MB.Benchmark('human', (9.0, 500.0, 200.0), 0.0, 'test')
    pl = MB.plan(MB.with_benchmark(tr, ok))
    _check(pl.gold_at is not None and pl.order[pl.gold_at].index == 7,
           'gold at lap 7, the first record strictly under the human key')
    tie = MB.Benchmark('human', (8.0, 750.0, 320.0), 0.0, 'test')
    pl = MB.plan(MB.with_benchmark(tr, tie))
    _check(pl.gold_at is None and pl.ties_at is not None,
           'an EQUAL key is a match, not gold')
    broken = MB.Benchmark('human', (20.0, 900.0, 400.0), 3.0, 'test')
    pl = MB.plan(MB.with_benchmark(tr, broken))
    _check(pl.gold_at is None and pl.ties_at is None,
           'a benchmark that is not itself working earns nothing')
    ungraded = MB.Benchmark('human', (20.0, 900.0, 400.0), None, 'no score')
    pl = MB.plan(MB.with_benchmark(tr, ungraded))
    _check(pl.gold_at is None, 'nor does an ungraded one')
    pl = MB.plan(tr)
    _check(pl.gold_at is None and pl.ref_vias == 10,
           'no benchmark: no gold, and 100 %% is the first working board '
           '(%s)' % pl.ref_vias)


def _draw(track, upto=None, theme='dark'):
    img = Image.new('RGB', (1600, 220))
    dbg = {}
    drew = MB.draw_band(ImageDraw.Draw(img), FL.Box(0, 0, 1600, 220), track,
                        upto=upto, theme=theme, debug=dbg)
    return img, dbg, drew


def test_the_band_draws_what_it_claims():
    import render_theme
    tr = MB.from_converge_ledger(_ledger(_run()))
    for theme in ('dark', 'light'):
        th = render_theme.theme(theme)
        ok, gold = th.rgb('status_kept'), th.rgb('status_best')
        img, dbg, drew = _draw(MB.with_benchmark(
            tr, MB.Benchmark('human', (9.0, 500.0, 200.0), 0.0, 'test')),
            theme=theme)
        _check(drew, '%s: drew' % theme)
        x0, _y0, x1, _y1 = dbg['plot']
        ly = dbg['line_y']
        line = sum(1 for x in range(int(x0), int(x1), 7)
                   if img.getpixel((x, ly)) == tuple(ok))
        _check(line > 0.8 * len(range(int(x0), int(x1), 7)),
               '%s: the DONE line is in the pixels (%d samples)'
               % (theme, line))
        _check(dbg['chip'] and dbg['chip'].startswith('WORKING @'),
               '%s: the crossing chip (%r)' % (theme, dbg['chip']))
        g = [i for i in dbg['shown'] if MB.plan(MB.with_benchmark(
            tr, MB.Benchmark('human', (9.0, 500.0, 200.0), 0.0, 't')))
            .gold_at == i]
        _check(bool(g), '%s: the gold lap is revealed' % theme)
        gx, gy = dbg['gold_xy']
        _check(img.getpixel((int(gx), int(gy))) == tuple(gold),
               '%s: the gold marker is in the pixels' % theme)
        _check(dbg['marker'] and 'beats benchmark' in dbg['marker'],
               '%s: and labelled (%r)' % (theme, dbg['marker']))
    _img, dbg, _d = _draw(tr)
    _check(dbg['marker'] is None and 'no benchmark board' in dbg['caption'],
           'no benchmark: no marker, and the caption says so (%r)'
           % dbg['caption'])
    _img, dbg, _d = _draw(MB.with_benchmark(
        tr, MB.Benchmark('human', (9.0, 500.0, 200.0), 3.0, 'test')))
    _check('not a working board' in dbg['caption'],
           'a broken benchmark is named as such (%r)' % dbg['caption'])
    _img, dbg, _d = _draw(tr, upto=3)
    _check(not dbg['crossed'] and dbg['chip'] is None,
           'before lap 4 is revealed, nothing says WORKING')


def test_degradation_is_said():
    frames = [Image.new('RGB', (400, 300)) for _ in range(3)]
    out, rep = MB.attach(frames, None, box=FL.Box(0, 200, 400, 100))
    _check(out is frames and not rep['drawn'] and rep['why'],
           'no track: untouched, said (%r)' % rep['why'])
    tr = MB.from_converge_ledger(_ledger(_run()))
    out, rep = MB.attach(frames, tr, box=None)
    _check(out is frames and 'no band' in rep['why'],
           'no box: untouched, said (%r)' % rep['why'])
    out, rep = MB.attach(frames, tr, box=FL.Box(0, 280, 60, 20))
    _check(out is frames and 'too small' in rep['why'],
           'a tiny box: untouched, said (%r)' % rep['why'])
    out, rep = MB.attach(frames, tr, box=FL.Box(0, 150, 400, 150))
    _check(rep['drawn'] and len(list(out)) == 3,
           'a real box: drawn on every frame (%r)' % rep['why'])
    _check(MB.status_line(rep).startswith('benchmark band: converge'),
           'the status line names its source (%r)' % MB.status_line(rep))


def test_hostile_rows_are_ungraded_not_raised():
    rows = [_row(0, 5, 3), 'not json', '[1, 2]', _row(1, True, 3),
            _row(2, float('nan'), 3), _row(3, '4', 3),
            {'iteration': 4, 'kind': 'completion', 'accepted': True,
             'score': 'x'},
            _row(5, 0, 2), _row(6, 0, 10 ** 400)]
    tr = MB.from_converge_ledger(_ledger([
        r if isinstance(r, (dict, str)) else r for r in rows]))
    graded = [p.index for p in tr.points if p.blocking is not None]
    _check(graded == [0, 5, 6], 'only countable blockings are graded (%s)'
           % graded)
    _check(tr.domain is None, 'a lap without a time makes x the lap order')
    _img, _dbg, drew = _draw(tr)
    _check(drew, 'and the band still draws -- a 400-digit via count '
           'included (it ranks, it just cannot be plotted)')


def test_only_an_accepted_lap_makes_the_board_working():
    """The phase-4 verifier: three real ledgers put WORKING on a lap the run
    itself REJECTED (run 25's verdict of record was STUCK)."""
    rows = [_row(0, 4, 9), _row(1, 0, 8, accepted=False), _row(2, 3, 9)]
    pl = MB.plan(MB.from_converge_ledger(_ledger(rows)))
    _check(pl.done_at is None and not pl.spans,
           'a rejected blocking-0 lap is not a crossing (%s)' % (pl.spans,))
    _check([pl.order[i].index for i, _l in pl.records] == [0, 2],
           'and the accepted 4 -> 3 is still drawn as progress')


def test_a_regression_ends_the_working_span_and_says_so():
    rows = [_row(0, 3, 9), _row(1, 0, 8), _row(2, 2, 7), _row(3, 0, 7)]
    tr = MB.from_converge_ledger(_ledger(rows))
    pl = MB.plan(tr)
    _check([(pl.order[a].index, b if b is None else pl.order[b].index)
            for a, b in pl.spans] == [(1, 2), (3, None)],
           'working 1..2, then again from 3 (%s)' % (pl.spans,))
    img = Image.new('RGB', (1600, 220))
    dbg = {}
    MB.draw_band(ImageDraw.Draw(img), FL.Box(0, 0, 1600, 220), tr,
                 theme='dark', debug=dbg)
    x0, y0, x1, y1 = dbg['plot']
    import render_theme
    ground = render_theme.theme('dark').rgb('ground')
    xa, xb = dbg['xs'][1], dbg['xs'][2]
    mid = int((xa + xb) / 2)
    after = int((xb + dbg['xs'][3]) / 2)
    _check(img.getpixel((mid, y1 - 2)) != tuple(ground)
           and img.getpixel((after, y1 - 2)) == tuple(ground),
           'the green ground stops at the regression')
    recs = [pl.order[i].index for i, _l in pl.records]
    _check(recs == [0, 1, 3],
           'the regression (lap 2) is never a record; the re-cross with '
           'fewer vias is (%s)' % recs)


def test_final_and_exhausted_rows_are_not_laps():
    rows = [_row(0, 3, 9), _row(1, 0, 8),
            dict(_row(2, 1, 8), final=True, stop_condition='STUCK',
                 lenses=['VERDICT=FAIL:lens=spec']),
            dict(_row(3, 0, 8), exhausted={'half': 'routing', 'reason': 'x'})]
    tr = MB.from_converge_ledger(_ledger(rows))
    _check([p.iteration for p in tr.points] == [0, 1],
           'only the two laps are points (%s)'
           % [p.iteration for p in tr.points])
    _check(tr.final == 'final: STUCK (spec FAIL)',
           'the final row\'s verdict is carried for the caption (%r)'
           % tr.final)
    _img, dbg, _d = _draw(tr)
    _check('final: STUCK' in dbg['caption'], 'and shown (%r)'
           % dbg['caption'])
    ok = rows[:2] + [dict(_row(2, 0, 8), final=True, stop_condition='DONE',
                          lenses=['VERDICT=PASS:lens=connectivity',
                                  'VERDICT=PASS:lens=drc',
                                  'VERDICT=PASS:lens=spec'])]
    tr = MB.from_converge_ledger(_ledger(ok))
    _check(tr.final == 'final: DONE, verified',
           'a --final DONE row with every lens PASS reads verified (%r)'
           % tr.final)


def test_ungraded_is_unexamined_not_passed():
    """`converge verdict` calls a board with an `ungraded` LIST done and names
    the list UNEXAMINED (run 24 and run 19 shipped DONE-EXHAUSTED so); the
    band agrees and SAYS it. A non-list names no component -- re-score."""
    r = dict(_row(0, 0, 1), score={'blocking': 0,
                                   'ungraded': ['impedance', 'length']})
    _check(LS.row_done(r) is True and LS.unexamined(r) == ['impedance',
                                                            'length'],
           'an ungraded LIST is working, with its components named')
    for v in ('impedance', 3):
        r = dict(_row(0, 0, 1), score={'blocking': 0, 'ungraded': v})
        _check(LS.row_done(r) is False,
               'ungraded=%r names no component: not working' % (v,))
    rows = [_row(0, 2, 9), dict(_row(1, 0, 8), score={
        'blocking': 0, 'ungraded': ['impedance'],
        'quality': {'vias': 8}})]
    _img, dbg, _d = _draw(MB.from_converge_ledger(_ledger(rows)))
    _check(dbg['chip'] and '1 unexamined' in dbg['chip'],
           'the chip counts what nobody examined (%r)' % dbg['chip'])


def test_a_benchmark_is_checked_before_it_is_believed():
    tr = MB.from_converge_ledger(_ledger(_run()))
    unm = MB.Benchmark('human', (math.inf, 500.0, 200.0), 0.0, 'test')
    pl = MB.plan(MB.with_benchmark(tr, unm))
    _check(not pl.bench_line and pl.ref_vias == 10 and pl.gold_at is None,
           'an unmeasured benchmark draws no 100 % line, earns no gold, and '
           '100 % is the first working board')
    _img, dbg, _d = _draw(MB.with_benchmark(tr, unm))
    _check('unmeasured' in dbg['caption'], 'the caption says why (%r)'
           % dbg['caption'])
    board = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
    sj = os.path.join(tempfile.mkdtemp(prefix='t1081s_'), 's.json')
    with open(sj, 'w') as f:
        json.dump({'board_sha': '0' * 64, 'blocking': 0}, f)
    b = MB.grade_benchmark(board, score_json=sj)
    _check(b.blocking is None and 'ANOTHER board' in b.why,
           'a score about another board is refused (%r)' % b.why)
    with open(sj, 'w') as f:
        json.dump({'board_sha': MB._sha256(board), 'blocking': 0}, f)
    b = MB.grade_benchmark(board, score_json=sj)
    _check(b.blocking == 0, 'the same board\'s score is read (%r)' % (b,))


def test_labels_never_overprint_and_outliers_do_not_flatten_the_axis():
    rows = [_row(0, 12703, kind='placement')]
    rows += [_row(i, 41 - i, kind='placement') for i in range(1, 12)]
    rows += [_row(12 + i, 0, 40 - (i // 2), 900.0 - i, 300)
             for i in range(24)]
    tr = MB.from_converge_ledger(_ledger(rows))
    pl = MB.plan(tr)
    _check(pl.bmax < 200, 'one 12703 pile does not set the axis (top %g)'
           % pl.bmax)
    for theme in ('dark', 'light'):
        img = Image.new('RGB', (900, 200))
        dbg = {}
        MB.draw_band(ImageDraw.Draw(img), FL.Box(0, 0, 900, 200), tr,
                     theme=theme, debug=dbg)
        rects = dbg['text_rects']
        bad = [(a, b) for k, a in enumerate(rects) for b in rects[k + 1:]
               if MB._overlaps(a, b)]
        _check(not bad, '%s: %d text boxes, none overlapping (%s)'
               % (theme, len(rects), bad[:2]))
        x0, y0, x1, y1 = dbg['plot']
        _check(all(r[2] <= x1 + 3 for r in rects),
               '%s: no text past the plot\'s right edge' % theme)
    # esp_prog, run 35: placement laps whose labels carry their terms, at
    # the band's left end. The caption and the axis words were drawn AFTER
    # the labels and were not obstacles, so a label printed under the
    # caption, and one ran off the band's left edge.
    # The lap times are the run's own: the two placement laps 2.6 s apart
    # both sit on the axis, and the second's label tried a left offset.
    rows = [_row(0, 288, kind='placement', t=T0,
                 score={'blocking': 288, 'quality': {},
                        'blocking_by': {'assembly': 76, 'drc': 195,
                                        'unrouted': 17}}),
            _row(1, 25, kind='placement', t=T0 + 2.6,
                 score={'blocking': 25, 'quality': {},
                        'blocking_by': {'drc': 2, 'unrouted': 17,
                                        'floorplan': 6}}),
            _row(2, 17, kind='placement', t=T0 + 67.7),
            _row(3, 1, 37, 399.0, 314, t=T0 + 205.2),
            _row(4, 0, 37, 399.9, 314, t=T0 + 366.0),
            _row(5, 0, 36, 339.4, 250, t=T0 + 733.0)]
    tr = MB.from_converge_ledger(_ledger(rows))
    for size in ((900, 126), (1400, 126), (560, 90)):
        img = Image.new('RGB', size)
        dbg = {}
        MB.draw_band(ImageDraw.Draw(img), FL.Box(0, 0, size[0], size[1]), tr,
                     theme='light', debug=dbg)
        rects = dbg['text_rects']
        cap = [r for r in rects if r[1] <= 3]
        bad = [(a, b) for k, a in enumerate(rects) for b in rects[k + 1:]
               if MB._overlaps(a, b)]
        _check(cap and not bad, '%dx%d: the caption is an obstacle and no '
               'text overlaps (%s)' % (size[0], size[1], bad[:2]))
        _check(all(r[0] >= -2 for r in rects),
               '%dx%d: no text starts left of the band (%s)'
               % (size[0], size[1], min(r[0] for r in rects)))


TESTS = (
    test_only_an_accepted_lap_makes_the_board_working,
    test_a_regression_ends_the_working_span_and_says_so,
    test_final_and_exhausted_rows_are_not_laps,
    test_ungraded_is_unexamined_not_passed,
    test_a_benchmark_is_checked_before_it_is_believed,
    test_labels_never_overprint_and_outliers_do_not_flatten_the_axis,
    test_the_done_predicate,
    test_quality_key_is_the_runs_own_ranking,
    test_records_are_lexicographic_and_labelled,
    test_never_a_weighted_sum,
    test_a_fail_lens_keeps_the_band_above_the_line,
    test_gold_needs_a_working_benchmark_and_a_strict_beat,
    test_the_band_draws_what_it_claims,
    test_degradation_is_said,
    test_hostile_rows_are_ungraded_not_raised,
)


def main():
    for fn in TESTS:
        print('%s:' % fn.__name__)
        fn()
    if _FAIL:
        print('')
        print('%d FAILURE(S)' % len(_FAIL))
        for msg in _FAIL:
            print('  - %s' % msg)
        return 1
    print('')
    print('all %d checks passed' % len(TESTS))
    return 0


if __name__ == '__main__':
    sys.exit(main())

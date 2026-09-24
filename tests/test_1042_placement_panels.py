#!/usr/bin/env python3
"""The placement panels, in placement currency, beside the verdict band (#1042).

  * **Run 32's table, reproduced to the digit** from the film's own boards
    (`render_placement --json-out`) and the ledger, for the boards this
    machine has (glasgow_unplaced = the pile, placed_v2, placed_v3) and the
    human as-built benchmark (kicad_files/glasgow_revC). Self-skipped, and
    SAID, when wk/run32 is absent.
  * every panel NAMES its instrument; the arrangement panel says it is a
    SCREEN, not the verdict;
  * ONE point per placement BOARD, never per frame -- the points drawn grow
    with the beat on screen, board by board;
  * units are never mixed: crossings and hpwl each get their own axis;
  * a copper-free placement row is OFF the verdict axis, and the attempts
    note says so;
  * the panels say "placement settled" once the film is routing;
  * degradation: a chain with no copper-free boards, or one where nothing
    moved, gets NO panel and no measurement is paid for;
  * exact frame sizes across layout x ratio x theme with the panels on.

Needs Pillow; renders small in-repo boards (render_placement ~2-5 s each,
cached per board sha).
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

RUN_ALL_TIMEOUT = 1500

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router'),
           os.path.join(ROOT, 'py_placer'), os.path.join(ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from PIL import Image, ImageDraw
except ImportError as exc:
    print('SKIP: needs Pillow (%s)' % exc)
    sys.exit(77)

import frame_layout as FL          # noqa: E402
import movie_placement as MP       # noqa: E402

KF = os.path.join(ROOT, 'kicad_files')
SEED = os.path.join(KF, 'interf_u_unrouted.kicad_pcb')
PLACED = os.path.join(KF, 'interf_u_unrouted_placed.kicad_pcb')
ROUTED = os.path.join(KF, 'routed_output.kicad_pcb')
BENCH = os.path.join(KF, 'glasgow_revC.kicad_pcb')
RUN32 = os.path.join(ROOT, 'wk', 'run32')

_FAIL = []
_NOTES = []


def fail(msg):
    _FAIL.append(msg)
    print('  FAIL: %s' % msg)


#: #1042's table, for the boards present here: off-outline parts, conflict
#: pairs, overlap mm2 (2 dp), crossings, hpwl mm (rounded), floorplan errors.
TABLE = {'glasgow_unplaced': (243, 3214, 9503.03, 10974, 4834, None),
         'placed_v2': (0, 6, 23.69, 3740, 5760, 12),
         'placed_v3': (0, 6, 23.69, 3750, 5743, 11)}
BENCH_ROW = (0, 10, 70.05, 1352, 3641)


def test_run32_reproduces_the_table():
    _mark = len(_FAIL)
    boards = [os.path.join(RUN32, b + '.kicad_pcb') for b in TABLE]
    if not all(os.path.isfile(b) for b in boards):
        _NOTES.append('run-32 table not reproduced: wk/run32 boards absent')
        print('    (wk/run32 absent: the table check did not run)')
        return
    steps = [(os.path.splitext(os.path.basename(b))[0], b, None)
             for b in boards]
    t, why = MP.build_track(steps, [],
                            ledger=os.path.join(RUN32, 'ledger.jsonl'),
                            benchmark=BENCH,
                            intent=os.path.join(RUN32, 'glasgow.intent.json'))
    if t is None:
        fail('BROKEN: no track (%s)' % why)
        return
    for bt in t.beats:
        want = TABLE[bt.label]
        got = (bt.off_outline, bt.conflict_pairs, round(bt.overlap_mm2, 2),
               bt.crossings, int(round(bt.hpwl)))
        if got != want[:5]:
            fail('%s: %r, the table says %r' % (bt.label, got, want[:5]))
        if want[5] is not None and bt.floorplan != want[5]:
            fail('%s: floorplan %r (%s), the table says %r'
                 % (bt.label, bt.floorplan, bt.floorplan_source, want[5]))
        print('    %-17s %s  floorplan %s (%s)'
              % (bt.label, got, bt.floorplan, bt.floorplan_source))
    b = t.benchmark or {}
    got = (b.get('off_outline'), b.get('conflict_pairs'),
           round(b.get('overlap_mm2') or 0, 2), b.get('crossings'),
           int(round(b.get('hpwl') or 0)))
    if got != BENCH_ROW:
        fail('benchmark %r, the table says %r' % (got, BENCH_ROW))
    else:
        print('    %-17s %s' % ('glasgow_revC', got))
    flagged = [(f.beat, f.row) for f in t.flags]
    if not any(r == 8 for _b, r in flagged):
        fail('the L3 placement classification (ledger row 8) is not flagged')
    if len(_FAIL) == _mark:
        print('  PASS: run 32 reproduced for the 3 boards here + the '
              'benchmark; %d defect flag(s)' % len(flagged))


def _track(n=3, bench=True):
    beats = tuple(MP.Beat('b%d' % i, 'b%d' % i, 10 * i,
                          [240, 0, 0][i % 3], [3000, 6, 6][i % 3],
                          [9500.0, 23.69, 23.69][i % 3],
                          [10974, 3740, 3750][i % 3],
                          [4834.0, 5760.0, 5743.0][i % 3], 6,
                          [131, 12, 11][i % 3], 'check_floorplan', None)
                  for i in range(n))
    bm = ({'off_outline': 0, 'conflict_pairs': 10, 'overlap_mm2': 70.05,
           'crossings': 1352, 'hpwl': 3641.0, 'locked_pairs': 6}
          if bench else None)
    return MP.PlacementTrack(beats, bm, 'glasgow_revC' if bench else '',
                             (MP.Flag(1, 'U30 vias in back-side cap pads',
                                      8),), ())


class _Rec(object):
    def __init__(self, d):
        self._d, self.texts, self.dots = d, [], 0

    def text(self, xy, txt, *a, **kw):
        self.texts.append(txt)
        return self._d.text(xy, txt, *a, **kw)

    def ellipse(self, *a, **kw):
        self.dots += 1
        return self._d.ellipse(*a, **kw)

    def __getattr__(self, k):
        return getattr(self._d, k)


def _draw(track, cur, w=900, h=180, theme='dark', routing=False):
    im = Image.new('RGB', (w, h))
    rec = _Rec(ImageDraw.Draw(im))
    dbg = {}
    ok = MP.draw_panels(rec, FL.Box(0, 0, w, h), track, cur=cur, theme=theme,
                        frame_h=788, routing=routing, debug=dbg)
    return ok, dbg, rec


def test_panels_name_their_instruments_and_the_screen():
    _mark = len(_FAIL)
    ok, dbg, rec = _draw(_track(), 2)
    if not ok:
        fail('BROKEN: the panels did not draw')
        return
    foot = dbg.get('footers') or []
    if sum('render_placement' in f for f in foot) != 2:
        fail('legality and arrangement do not both name render_placement: '
             '%r' % foot)
    if not any('check_floorplan' in f for f in foot):
        fail('the intent panel does not name check_floorplan: %r' % foot)
    titles = ' | '.join(dbg.get('titles') or [])
    if 'SCREEN' not in titles or 'not the verdict' not in titles:
        fail('the arrangement panel does not say it is a SCREEN, not the '
             'verdict: %r' % titles)
    if dbg.get('floor') != 6:
        fail('the locked-parts floor is not labelled: %r' % dbg.get('floor'))
    elif 'floor 6 = locked parts' not in rec.texts:
        fail('the floor label is not drawn')
    legs = dbg.get('legends') or []
    if legs[0] != ['off-outline parts', 'conflict pairs', 'overlap mm2']:
        fail('the legality legend does not name each series and unit: %r'
             % legs[0])
    drawn = ' '.join(rec.texts)
    for word in ('render_placement', 'SCREEN', 'floorplan errors',
                 'hpwl mm', 'airwire crossings'):
        if word not in drawn:
            fail('%r is not DRAWN on the panels' % word)
    if len(_FAIL) == _mark:
        print('  PASS: instruments named in every footer; SCREEN, not the '
              'verdict; legends name units')


def test_one_point_per_board_and_units_never_mixed():
    _mark = len(_FAIL)
    t = _track(3)
    counts = []
    for cur in (0, 1, 2):
        ok, dbg, rec = _draw(t, cur)
        counts.append(rec.dots)
        for name, s in (dbg.get('series') or {}).items():
            if len(s) != len(t.beats):
                fail('series %r has %d points for %d boards'
                     % (name, len(s), len(t.beats)))
    # 3 legality + 2 arrangement + 1 intent series = 6 points per beat shown
    if counts != [6, 12, 18]:
        fail('points drawn per beat on screen are %r, not [6, 12, 18] -- '
             'one point per board, revealed board by board' % counts)
    ok, dbg, _rec = _draw(t, 2)
    ax = dbg.get('axes') or {}
    if set(ax) != {'crossings', 'hpwl mm'}:
        fail('the arrangement axes are %r' % sorted(ax))
    elif ax['crossings'][2] == ax['hpwl mm'][2] or \
            ax['crossings'][1] == ax['hpwl mm'][1]:
        fail('crossings and hpwl share an axis: %r' % ax)
    if len(_FAIL) == _mark:
        print('  PASS: %r points as the film advances; crossings and hpwl on '
              'their own axes' % counts)


def test_placement_rows_are_off_the_verdict_axis():
    _mark = len(_FAIL)
    import movie_attempts as MA
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, 'ledger.jsonl')
        rows = [(0, 'placement', 267), (1, 'placement', 251),
                (2, 'completion', 41), (3, 'completion', 33)]
        with open(p, 'w', encoding='utf-8') as f:
            for it, kind, b in rows:
                f.write(json.dumps({'iteration': it, 'kind': kind,
                                    'accepted': True,
                                    'score': {'blocking': b}}) + '\n')
        t = MA.attempts_from_converge_ledger(p)
    scores = [a.score for a in t.attempts]
    if 267.0 in scores or 251.0 in scores:
        fail('a copper-free placement row is ON the verdict axis: %r'
             % scores)
    if '2 placement lap(s) off this axis' not in t.note:
        fail('the note does not say the placement laps were taken off: %r'
             % t.note)
    if len(_FAIL) == _mark:
        print('  PASS: %r on the axis; %s' % (scores, t.note))


def test_the_panels_say_when_placement_is_settled():
    _mark = len(_FAIL)
    ok, _d, rec = _draw(_track(), 2, routing=True)
    if 'placement settled' not in rec.texts:
        fail('a routing frame does not say the placement is settled')
    ok, _d, rec2 = _draw(_track(), 1, routing=False)
    if 'placement settled' in rec2.texts:
        fail('a placement frame says the placement is settled')
    marks = [('a', 'b0', 0, 10), ('b', 'b1', 10, 20), ('c', 'b2', 20, 30),
             ('route', 'r', 30, 50)]
    t = _track()
    if MP.routing_from(t, marks) != 30:
        fail('routing starts at %r, not frame 30' % MP.routing_from(t, marks))
    # and COMPOSE hands `routing=True` to exactly the frames from there on
    seen = []
    orig = MP.draw_panels

    def _spy(d, box, track, **kw):
        seen.append(kw.get('routing'))
        return True
    MP.draw_panels = _spy
    try:
        frames = [Image.new('RGB', (40, 20)) for _ in range(50)]
        MP.compose(frames, FL.Box(0, 0, 40, 20), t, marks, 'dark', 720)
    finally:
        MP.draw_panels = orig
    if seen != [False] * 30 + [True] * 20:
        fail('compose flags routing on the wrong frames: %r' % seen)
    if [MP.beat_at(t, i) for i in (0, 9, 10, 25, 45)] != [0, 0, 1, 2, 2]:
        fail('the beat on screen per frame is wrong: %r'
             % [MP.beat_at(t, i) for i in (0, 9, 10, 25, 45)])
    if len(_FAIL) == _mark:
        print('  PASS: routing frames say "placement settled"; the beat '
              'follows the frame')


def test_degradation_draws_and_measures_nothing():
    _mark = len(_FAIL)
    called = []
    orig = MP.measure_board
    MP.measure_board = lambda *a, **k: called.append(a) or None
    try:
        t, why = MP.build_track([('r', ROUTED, None)], [])
    finally:
        MP.measure_board = orig
    if t is not None or called:
        fail('a routed-only chain got a track (%r) or paid for %d '
             'measurement(s)' % (why, len(called)))
    d = tempfile.mkdtemp(prefix='t1042d_')
    try:
        a = os.path.join(d, 'a.kicad_pcb')
        b = os.path.join(d, 'b.kicad_pcb')
        shutil.copy(SEED, a)
        shutil.copy(SEED, b)
        t2, _w = MP.build_track([('a', a, None), ('b', b, None)], [])
        if MP.placed_anything(t2, [('a', a, None), ('b', b, None)]):
            fail('two identical boards read as a placement')
    finally:
        shutil.rmtree(d, ignore_errors=True)
    if MP.draw_panels(ImageDraw.Draw(Image.new('RGB', (10, 10))),
                      FL.Box(0, 0, 400, 100), None):
        fail('no track still drew a panel')
    if len(_FAIL) == _mark:
        print('  PASS: routed-only chain -> no track, nothing measured; '
              'nothing moved -> no placement; no track -> no panel')


def test_exact_frame_sizes_with_the_panels():
    _mark = len(_FAIL)
    import make_movie
    tmp = tempfile.mkdtemp(prefix='t1042f_')
    n = 0
    try:
        for lk in ('split', 'stacked', 'sidebar', 'legacy'):
            for rk in ('16:9', '9:16'):
                for th in ('dark', 'light'):
                    err = io.StringIO()
                    out = os.path.join(tmp, '%s_%s_%s.gif'
                                       % (lk, rk.replace(':', 'x'), th))
                    with contextlib.redirect_stderr(err):
                        got = make_movie.make_movie(
                            [SEED, PLACED], out=out, size=400, quiet=True,
                            layout=lk, aspect=rk, theme=th, attempts=False,
                            camera='off')
                    n += 1
                    if 'placement panels: 2 placement board(s)' not in \
                            err.getvalue():
                        fail('%s/%s/%s: no placement panels: %s'
                             % (lk, rk, th, err.getvalue()[-200:]))
                    r = FL.parse_ratio(rk)
                    want = ((FL.even(400), FL.even(round(400 / r))) if r >= 1
                            else (FL.even(round(400 * r)), FL.even(400)))
                    with Image.open(got) as im:
                        if im.size != want:
                            fail('%s/%s/%s: %r, declared %r'
                                 % (lk, rk, th, im.size, want))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if len(_FAIL) == _mark:
        print('  PASS: %d films (4 layouts x 2 ratios x 2 themes) with the '
              'panels, each the declared size' % n)


TESTS = (
    test_run32_reproduces_the_table,
    test_panels_name_their_instruments_and_the_screen,
    test_one_point_per_board_and_units_never_mixed,
    test_placement_rows_are_off_the_verdict_axis,
    test_the_panels_say_when_placement_is_settled,
    test_degradation_draws_and_measures_nothing,
    test_exact_frame_sizes_with_the_panels,
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

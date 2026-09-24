#!/usr/bin/env python3
"""The placement panels, in placement currency, beside the verdict band (#1042).

  * **Run 32's table, reproduced to the digit** from the film's own boards
    (render_placement's numbers, measured in process) for the boards this
    machine has (glasgow_unplaced = the pile, placed_v2, placed_v3) and the
    human as-built benchmark (kicad_files/glasgow_revC). ONE instrument per
    intent line: `check_floorplan --intent` on every beat with an intent,
    the ledger's value only without one. Self-skipped, and SAID, when
    wk/run32 is absent.
  * **No subprocess of sys.executable, ever** -- inside KiCad that is the
    pcbnew binary and the child hangs. Every subprocess entry point is made
    to fail, and a real measurement (render_placement + check_floorplan)
    still runs.
  * **Cheap gates first**: a copper-free-plus-routed chain, and a chain in
    which nothing moved, never reach the measurement.
  * every panel NAMES its instrument; the arrangement panel says it is a
    screen, and "not the verdict -- see band" is rendered WHOLE wherever the
    panel is drawn;
  * ONE point per placement BOARD, never per frame; before the first beat
    lands NOTHING of the future is drawn (no point, no flag);
  * units never mixed; axis tops are ROUND ticks; no intent reads
    'unmeasured', never a number;
  * x is RUN TIME shared with the verdict band when the ledger carries `t`,
    else the board order, and the header says which;
  * READABLE OR NOT DRAWN, across layout x ratio x size {500,1000,1400}:
    every plot >= 48 px, every text inside its panel, no two overlapping,
    fewer panels (named in the header) when narrow, a decline disclosed when
    the frame is too small;
  * a glide shows the SOURCE board's numbers until it lands; flags on one
    beat stack; a failed draw repaints; an unmeasured board is not zero;
  * `--attempts-ledger` works from COPIES rendered away from the run dir;
  * exact frame sizes across layout x ratio x theme with the panels on.

Needs Pillow; renders small in-repo boards.
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
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
#: pairs, overlap mm2 (2 dp), crossings, hpwl mm (rounded); then floorplan
#: errors by check_floorplan --intent, and by the ledger (None = no row).
TABLE = {'glasgow_unplaced': (243, 3214, 9503.03, 10974, 4834, 131, None),
         'placed_v2': (0, 6, 23.69, 3740, 5760, 12, 12),
         'placed_v3': (0, 6, 23.69, 3750, 5743, 11, 11)}
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
    led = os.path.join(RUN32, 'ledger.jsonl')
    t, why = MP.build_track(steps, [], ledger=led, benchmark=BENCH,
                            intent=os.path.join(RUN32, 'glasgow.intent.json'))
    t2, why2 = MP.build_track(steps, [], ledger=led)
    if t is None or t2 is None:
        fail('BROKEN: no track (%s / %s)' % (why, why2))
        return
    for bt, bt2 in zip(t.beats, t2.beats):
        want = TABLE[bt.label]
        got = (bt.off_outline, bt.conflict_pairs, round(bt.overlap_mm2, 2),
               bt.crossings, int(round(bt.hpwl)))
        if got != want[:5]:
            fail('%s: %r, the table says %r' % (bt.label, got, want[:5]))
        # ONE INSTRUMENT PER LINE: with an intent, check_floorplan on every
        # beat -- the pile included -- and never the ledger's number
        if bt.floorplan_source != MP.FP_INSTRUMENT or bt.floorplan != want[5]:
            fail('%s: floorplan %r from %r, want %r from %s'
                 % (bt.label, bt.floorplan, bt.floorplan_source, want[5],
                    MP.FP_INSTRUMENT))
        # without one, the ledger's own value, labelled, and the pile has
        # no row: unmeasured, not borrowed from another instrument
        if bt2.floorplan != want[6] or (
                want[6] is not None
                and not bt2.floorplan_source.startswith('ledger row')):
            fail('%s: no-intent floorplan %r from %r, want %r from the ledger'
                 % (bt.label, bt2.floorplan, bt2.floorplan_source, want[6]))
        print('    %-17s %s  floorplan %s (%s) | ledger %s (%s)'
              % (bt.label, got, bt.floorplan, bt.floorplan_source,
                 bt2.floorplan, bt2.floorplan_source or 'none'))
    if t.floorplan_source != MP.FP_INSTRUMENT or \
            t2.floorplan_source != MP.FP_LEDGER:
        fail('track sources %r / %r' % (t.floorplan_source,
                                         t2.floorplan_source))
    b = t.benchmark or {}
    got = (b.get('off_outline'), b.get('conflict_pairs'),
           round(b.get('overlap_mm2') or 0, 2), b.get('crossings'),
           int(round(b.get('hpwl') or 0)))
    if got != BENCH_ROW:
        fail('benchmark %r, the table says %r' % (got, BENCH_ROW))
    else:
        print('    %-17s %s' % ('glasgow_revC', got))
    # RUN TIME: the ledger's own clock, and the pile (no row) at its start
    if not t.x_domain:
        fail('run 32 has a clock and the track has no time domain')
    else:
        ts = [bt.t for bt in t.beats]
        if ts[0] != t.x_domain[0] or ts != sorted(ts):
            fail('beat times %r over domain %r' % (ts, t.x_domain))
    flagged = [(f.beat, f.row) for f in t.flags]
    if not any(r == 8 for _b, r in flagged):
        fail('the L3 placement classification (ledger row 8) is not flagged')
    if len(_FAIL) == _mark:
        print('  PASS: run 32 reproduced for the 3 boards here + the '
              'benchmark; one instrument per intent line; %d defect flag(s)'
              % len(flagged))


def _track(n=3, bench=True, x_domain=(0.0, 60000.0), flags=None,
           src='check_floorplan --intent'):
    ts = [0.0, 1400.0, 20000.0]
    beats = tuple(MP.Beat('b%d' % i, 'b%d' % i, 10 * i,
                          [240, 0, 0][i % 3], [3000, 6, 6][i % 3],
                          [9500.0, 23.69, 23.69][i % 3],
                          [10974, 3740, 3750][i % 3],
                          [4834.0, 5760.0, 5743.0][i % 3], 6,
                          [131, 12, 11][i % 3] if src else None, src, None,
                          ts[i % 3] if x_domain else None)
                  for i in range(n))
    bm = ({'off_outline': 0, 'conflict_pairs': 10, 'overlap_mm2': 70.05,
           'crossings': 1352, 'hpwl': 3641.0, 'locked_pairs': 6}
          if bench else None)
    if flags is None:
        flags = (MP.Flag(1, 'ONE pocket', 8),)
    return MP.PlacementTrack(beats, bm, 'glasgow_revC' if bench else '',
                             tuple(flags), (), x_domain, src)


class _Rec(object):
    def __init__(self, d):
        self._d, self.texts, self.dots, self.lines = d, [], 0, []

    def text(self, xy, txt, *a, **kw):
        self.texts.append(txt)
        return self._d.text(xy, txt, *a, **kw)

    def ellipse(self, *a, **kw):
        self.dots += 1
        return self._d.ellipse(*a, **kw)

    def line(self, xy, *a, **kw):
        self.lines.append((tuple(xy), kw.get('fill')))
        return self._d.line(xy, *a, **kw)

    def __getattr__(self, k):
        return getattr(self._d, k)


def _draw(track, cur, w=1000, h=240, theme='dark', routing=False,
          frame_h=1000):
    im = Image.new('RGB', (w, h))
    rec = _Rec(ImageDraw.Draw(im))
    dbg = {}
    ok = MP.draw_panels(rec, FL.Box(0, 0, w, h), track, cur=cur, theme=theme,
                        frame_h=frame_h, routing=routing, debug=dbg)
    return ok, dbg, rec, im


def test_panels_name_their_instruments_and_the_screen():
    _mark = len(_FAIL)
    ok, dbg, rec, _im = _draw(_track(), 2)
    if not ok:
        fail('BROKEN: the panels did not draw (%r)' % dbg)
        return
    foot = dbg.get('footers') or []
    if sum('render_placement' in f for f in foot) != 2:
        fail('legality and arrangement do not both name render_placement: '
             '%r' % foot)
    if not any('check_floorplan' in f for f in foot):
        fail('the intent panel does not name check_floorplan: %r' % foot)
    if 'ARRANGEMENT (screen)' not in (dbg.get('titles') or []):
        fail('the arrangement panel is not titled a screen: %r'
             % dbg.get('titles'))
    if MP.SCREEN_NOTE not in rec.texts:
        fail('%r is not rendered whole' % MP.SCREEN_NOTE)
    if dbg.get('floor') != 6:
        fail('the locked-parts floor is not labelled: %r' % dbg.get('floor'))
    elif 'floor 6 = locked parts' not in rec.texts:
        fail('the floor label is not drawn')
    legs = dbg.get('legends') or []
    if legs[0][:3] != ['off-outline parts', 'conflict pairs', 'overlap mm2']:
        fail('the legality legend does not name each series and unit: %r'
             % legs[0])
    drawn = ' '.join(rec.texts)
    for word in ('render_placement', 'floorplan errors', 'hpwl mm',
                 'airwire crossings'):
        if word not in drawn:
            fail('%r is not DRAWN on the panels' % word)
    if len(_FAIL) == _mark:
        print('  PASS: instruments named in every footer; the SCREEN note '
              'whole; legends name units')


def test_one_point_per_board_and_nothing_before_the_first():
    _mark = len(_FAIL)
    t = _track(3)
    counts = []
    for cur in (None, 0, 1, 2):
        ok, dbg, rec, _im = _draw(t, cur)
        counts.append(rec.dots)
        for name, s in (dbg.get('series') or {}).items():
            if len(s) != len(t.beats):
                fail('series %r has %d points for %d boards'
                     % (name, len(s), len(t.beats)))
        if cur is None and dbg.get('flags'):
            fail('a flag is drawn before any beat landed: %r'
                 % dbg.get('flags'))
    # 3 legality + 2 arrangement + 1 intent series = 6 points per beat shown,
    # and NONE before the first beat has landed
    if counts != [0, 6, 12, 18]:
        fail('points drawn per beat on screen are %r, not [0, 6, 12, 18] -- '
             'one point per board, revealed board by board' % counts)
    ok, dbg, _rec, _im = _draw(t, 2)
    ax = dbg.get('axes') or {}
    if set(ax) != {'crossings', 'hpwl mm'}:
        fail('the arrangement axes are %r' % sorted(ax))
    elif ax['crossings'][2] == ax['hpwl mm'][2]:
        fail('crossings and hpwl share an axis: %r' % ax)
    # ROUND ticks, never max * 1.08
    for name, (_lo, hi, _s) in ax.items():
        if MP._nice_ceil(hi) != hi:
            fail('%s axis tops at %r, not a round tick' % (name, hi))
    if len(_FAIL) == _mark:
        print('  PASS: %r points as the film advances (none before the '
              'first); own axes, round tops' % counts)


def test_unmeasured_is_said_never_zero():
    _mark = len(_FAIL)
    # no intent and no ledger: 'unmeasured', not an axis of made-up 1.10
    ok, dbg, rec, _im = _draw(_track(src=''), 2)
    if 'unmeasured' not in rec.texts:
        fail('an intent with no instrument does not say unmeasured')
    ticks = [tx for _r, tx, p in dbg.get('texts') or []
             if p == 'intent' and tx in ('0', '1', '1.10')
             and _r[1] >= dbg['plots']['intent'][1]]
    if ticks:
        fail('an unmeasured intent axis carries a number: %r' % ticks)
    # a board the instrument cannot answer for is UNMEASURED
    d = tempfile.mkdtemp(prefix='t1042u_')
    try:
        empty = os.path.join(d, 'empty.kicad_pcb')
        with open(empty, 'w', encoding='utf-8') as f:
            f.write('(kicad_pcb (version 20240108) (generator "t")\n'
                    '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal))\n)\n')
        m = MP.measure_board(empty, cache={})
        if not m.get('unmeasured') or m.get('off_outline') is not None:
            fail('a board with no parts measured as %r' % m)
    finally:
        shutil.rmtree(d, ignore_errors=True)
    # and drawn as a mark, not as zeros: beat 1 has no legality numbers
    t = _track(3)
    b1 = t.beats[1]._replace(off_outline=None, conflict_pairs=None,
                             overlap_mm2=None, crossings=None, hpwl=None,
                             unmeasured='no parts on the board')
    t = t._replace(beats=(t.beats[0], b1, t.beats[2]))
    ok, dbg, rec, _im = _draw(t, 2)
    if rec.dots != 13:
        fail('an unmeasured beat drew %d points, want 13 (18 - its 5)'
             % rec.dots)
    if len(_FAIL) == _mark:
        print('  PASS: no intent reads "unmeasured"; a part-less board is '
              'unmeasured, drawn as a mark and not as zeros')


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
    ok, _d, rec, _im = _draw(_track(), 2, routing=True)
    if 'placement settled' not in rec.texts:
        fail('a routing frame does not say the placement is settled')
    ok, _d, rec2, _im = _draw(_track(), 1, routing=False)
    if 'placement settled' in rec2.texts:
        fail('a placement frame says the placement is settled')
    marks = [('a', 'b0', 0, 10), ('b', 'b1', 10, 20), ('c', 'b2', 20, 30),
             ('route', 'r', 30, 50)]
    t = _track()
    if MP.routing_from(t, marks) != 30:
        fail('routing starts at %r, not frame 30' % MP.routing_from(t, marks))
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
    # A GLIDE shows the source board until it LANDS: with_firsts reads the
    # landing frame build_boards records, not the step's first frame
    t3 = MP.with_firsts(t, marks, {os.path.normcase(os.path.abspath('b1')):
                                   16})
    if [b.first for b in t3.beats] != [0, 16, 20]:
        fail('the beat does not change at the landing frame: %r'
             % [b.first for b in t3.beats])
    if MP.beat_at(t3, 12) != 0:
        fail('mid-glide frame 12 shows beat %r, not the source board'
             % MP.beat_at(t3, 12))
    if len(_FAIL) == _mark:
        print('  PASS: routing frames say "placement settled"; the beat '
              'changes where the glide lands')


def _boom(*a, **k):
    raise AssertionError('a subprocess was spawned: %r' % (a[:1],))


def test_no_subprocess_and_cheap_gates_first():
    _mark = len(_FAIL)
    import check_floorplan as CF
    d = tempfile.mkdtemp(prefix='t1042s_')
    try:
        intent = os.path.join(d, 'i.json')
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            try:
                CF.main([PLACED, '--emit-intent', intent, '-q'])
            except SystemExit:
                pass
        if not os.path.isfile(intent):
            fail('BROKEN: could not emit an intent for the fixture')
            return
        # a ledger that ALSO scores the placed board, with a number no
        # instrument would give: with an intent, the line must not read it
        led = os.path.join(d, 'ledger.jsonl')
        with open(led, 'w', encoding='utf-8') as f:
            f.write(json.dumps({'iteration': 0, 'kind': 'placement',
                                't': 1.0, 'result_sha': _sha(PLACED),
                                'score': {'blocking_by':
                                          {'floorplan': 777}}}) + '\n')
        saved = {n: getattr(subprocess, n) for n in
                 ('run', 'Popen', 'call', 'check_call', 'check_output')}
        saved_os = (os.system, os.popen)
        for n in saved:
            setattr(subprocess, n, _boom)
        os.system = os.popen = _boom
        try:
            t, why = MP.build_track([('seed', SEED, None),
                                     ('placed', PLACED, None)], [],
                                    intent=intent, ledger=led, cache={})
        except AssertionError as exc:
            t, why = None, str(exc)
        finally:
            for n, fn in saved.items():
                setattr(subprocess, n, fn)
            os.system, os.popen = saved_os
        if t is None:
            fail('the measurement needed a subprocess (or failed): %s' % why)
        else:
            bt = t.beats[-1]
            if bt.crossings is None or bt.floorplan is None or \
                    bt.floorplan_source != MP.FP_INSTRUMENT:
                fail('in-process measurement incomplete: %r' % (bt,))
            elif bt.floorplan == 777:
                fail('with an intent the line read the LEDGER\'s floorplan '
                     '(two instruments on one line)')
            else:
                print('    in process: placed crossings %s, floorplan %s'
                      % (bt.crossings, bt.floorplan))
    finally:
        shutil.rmtree(d, ignore_errors=True)

    called = []
    orig_m, orig_f = MP.measure_board, MP.check_floorplan_errors
    MP.measure_board = lambda *a, **k: called.append(a) or {}
    MP.check_floorplan_errors = lambda *a, **k: called.append(a) or (0, '')
    try:
        t1, why1 = MP.build_track([('r', ROUTED, None)], [])
        # ONE copper-free board plus a routed one: #1042's routing-only
        # chain, which paid 5.2 s for a measurement it then threw away
        t2, why2 = MP.build_track([('s', SEED, None), ('r', ROUTED, None)],
                                  [], intent='x.json')
        d = tempfile.mkdtemp(prefix='t1042d_')
        try:
            a = os.path.join(d, 'a.kicad_pcb')
            b = os.path.join(d, 'b.kicad_pcb')
            shutil.copy(SEED, a)
            shutil.copy(SEED, b)
            t3, why3 = MP.build_track([('a', a, None), ('b', b, None)], [])
        finally:
            shutil.rmtree(d, ignore_errors=True)
    finally:
        MP.measure_board, MP.check_floorplan_errors = orig_m, orig_f
    for t, why, name in ((t1, why1, 'routed-only'),
                         (t2, why2, 'copper-free + routed'),
                         (t3, why3, 'nothing moved')):
        if t is not None:
            fail('%s chain got a track (%s)' % (name, why))
    if called:
        fail('the cheap gates paid for %d measurement(s)' % len(called))
    if MP.draw_panels(ImageDraw.Draw(Image.new('RGB', (10, 10))),
                      FL.Box(0, 0, 400, 100), None):
        fail('no track still drew a panel')
    if len(_FAIL) == _mark:
        print('  PASS: every subprocess entry blocked and the measurement '
              'still ran; routed-only, copper-free+routed and nothing-moved '
              'chains measured nothing (%s)' % why2)


def test_run_time_is_the_shared_x_axis():
    _mark = len(_FAIL)
    import movie_attempts as MA
    t = _track()
    xs = MP._beat_xs(t, 100.0, 700.0)
    want = [100.0 + 600.0 * bt.t / 60000.0 for bt in t.beats]
    if any(abs(a - b) > 1e-6 for a, b in zip(xs, want)):
        fail('beats are not placed by run time: %r vs %r' % (xs, want))
    # boards seconds apart on an hours-long axis keep a point EACH
    tc = t._replace(beats=tuple(bt._replace(t=float(i)) for i, bt in
                                enumerate(t.beats)), x_domain=(0.0, 1e6))
    xc = MP._beat_xs(tc, 100.0, 700.0)
    if any(b - a < MP.MIN_BEAT_PX - 1e-6 for a, b in zip(xc, xc[1:])):
        fail('beats one second apart share a pixel: %r' % xc)
    _ok, dbg, rec, _im = _draw(t, 2)
    if dbg.get('x_mode') != 'time' or not any(
            'x = run time' in tx for tx in rec.texts):
        fail('a timed track does not say x is run time: %r' % dbg.get(
            'header'))
    _ok, dbg2, rec2, _im = _draw(_track(x_domain=None), 2)
    if dbg2.get('x_mode') != 'index' or not any(
            'board order' in tx for tx in rec2.texts):
        fail('an untimed track does not say x is the board order: %r'
             % dbg2.get('header'))
    # the verdict band and the panels read ONE domain off one ledger
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, 'ledger.jsonl')
        with open(p, 'w', encoding='utf-8') as f:
            for it, kind, b, tt in ((0, 'placement', 250, 1000.0),
                                    (1, 'completion', 41, 2000.0),
                                    (2, 'completion', 33, 9000.0)):
                f.write(json.dumps({'iteration': it, 'kind': kind, 't': tt,
                                    'accepted': True,
                                    'score': {'blocking': b}}) + '\n')
        vt = MA.attempts_from_converge_ledger(p)
        dom = MA.ledger_time_domain(MP.read_ledger(p))
    if vt.x_domain != (1000.0, 9000.0) or dom != vt.x_domain:
        fail('the band domain %r is not the ledger\'s %r (placement lap '
             'included)' % (vt.x_domain, dom))
    if not MA.x_is_time(vt):
        fail('a timed converge track does not draw x as run time')
    im = Image.new('RGB', (600, 160))
    dbg3 = {}
    MA.draw_track(ImageDraw.Draw(im), FL.Box(0, 0, 600, 160), vt, debug=dbg3)
    if dbg3.get('x_mode') != 'time':
        fail('the verdict band did not draw run time: %r'
             % dbg3.get('x_mode'))
    if len(_FAIL) == _mark:
        print('  PASS: x is run time on both, over one ledger domain; the '
              'board order says so when there is no clock')


def test_flags_stack_and_a_failure_repaints():
    _mark = len(_FAIL)
    t = _track(flags=(MP.Flag(1, 'ONE pocket', 8),
                      MP.Flag(2, 'Focus panels', 49),
                      MP.Flag(2, 'GLOBAL capacity, not per-net', 94)))
    _ok, dbg, rec, _im = _draw(t, 2)
    lv = [f[2] for f in dbg.get('flags') or []]
    if lv != [0, 0, 1]:
        fail('flags on one beat do not stack: levels %r' % lv)
    plot = (dbg.get('plots') or {}).get('intent')
    if plot is not None:
        for (r, tx, pnl) in dbg.get('texts') or []:
            if pnl == 'intent' and tx in ('1', '2', '3') and r[3] > plot[1]:
                fail('flag number %r is drawn inside the plot, on the '
                     'series' % tx)
    # a FAILED draw repaints the box and says so, never half a panel
    orig = MP._PANEL['arrangement']

    def _raise(*a, **k):
        raise RuntimeError('boom')
    MP._PANEL['arrangement'] = _raise
    try:
        ok, dbg2, rec2, im = _draw(_track(), 2)
    finally:
        MP._PANEL['arrangement'] = orig
    if ok or 'error' not in dbg2:
        fail('a failing panel still reported drawn')
    import render_theme
    th = render_theme.theme('dark')
    series = {th.rgb(MP.ROLE[k]) for k in ('off-outline parts',
                                            'conflict pairs', 'overlap mm2')}
    if any(px in series for px in im.getdata()):
        fail('a half-drawn legality panel survived the failure')
    if not any('not drawn' in tx for tx in rec2.texts):
        fail('the repaint does not say the panels were not drawn')
    if len(_FAIL) == _mark:
        print('  PASS: flags on one beat stack above the plot; a failed draw '
              'repaints and says so')


def test_series_colours_are_distinct_in_both_themes():
    _mark = len(_FAIL)
    import palette_audit as PA
    import render_theme
    pairs = (('off-outline parts', 'conflict pairs'),
             ('conflict pairs', 'airwire crossings'),
             ('off-outline parts', 'overlap mm2'),
             ('conflict pairs', 'overlap mm2'),
             ('airwire crossings', 'hpwl mm'),
             ('marker', 'floorplan errors'),
             ('flag', 'floorplan errors'))
    for name in ('dark', 'light'):
        th = render_theme.theme(name)
        for a, b in pairs:
            ca, cb = th.rgb(MP.ROLE[a]), th.rgb(MP.ROLE[b])
            dn = PA.rgb_distance(ca, cb)
            dd = PA.rgb_distance(PA.deuteranope(ca), PA.deuteranope(cb))
            # palette_audit's crossing threshold, and a deuteranope floor
            if dn < 34.0 or dd < 25.0:
                fail('%s: %s vs %s are %.1f apart (%.1f deuteranope)'
                     % (name, a, b, dn, dd))
    if len(_FAIL) == _mark:
        print('  PASS: %d series pairs distinct in dark and light, for a '
              'deuteranope too' % len(pairs))


def _sweep_one(tr, lk, rk, size, verdict):
    bounds = (0, 0, 100, 60)
    kw = dict(layout=lk, ratio=FL.parse_ratio(rk), size=size,
              panel=(lk != 'legacy'), legacy_size=(size, int(size * 0.6)))
    g = FL.plan_frame(bounds, **kw)
    fn = MP.band_px(tr, verdict)
    bh = fn(g.frame.w, g.frame.h)
    bh -= bh % 2
    plan = fn.plans[-1]
    if plan.mode == 'declined':
        return plan, None, None, None
    g2 = FL.plan_frame(bounds, track_px=bh, **kw)
    pbox, vbox = MP.split_band(g2.track, both=verdict, track=tr,
                               frame_h=g2.frame.h)
    im = Image.new('RGB', (g2.frame.w, g2.frame.h))
    dbg = {}
    ok = MP.draw_panels(ImageDraw.Draw(im), pbox, tr, cur=2, theme='dark',
                        frame_h=g2.frame.h, routing=True, debug=dbg)
    return plan, ok, dbg, (g2, pbox, vbox)


def test_readable_or_not_drawn_across_layouts_ratios_sizes():
    _mark = len(_FAIL)
    tr = _track(flags=(MP.Flag(1, 'ONE pocket', 8),
                       MP.Flag(2, 'Focus panels', 49),
                       MP.Flag(2, 'GLOBAL capacity, not per-net (after '
                               'this film)', 94)))
    drawn = declined = dropped = 0
    declines = []
    for lk in ('legacy', 'stacked', 'sidebar', 'inset', 'split'):
        for rk in (None, '16:9', '9:16', '1:1', '4:3'):
            for size in (500, 1000, 1400):
                for verdict in (True, False):
                    tag = '%s/%s/%d/%s' % (lk, rk, size,
                                           'both' if verdict else 'place')
                    plan, ok, dbg, geo = _sweep_one(tr, lk, rk, size,
                                                    verdict)
                    if plan.mode == 'declined':
                        declined += 1
                        declines.append(tag)
                        if not plan.why:
                            fail('%s: declined without a reason' % tag)
                        if size >= 1000:
                            fail('%s: declined at size %d (%s)'
                                 % (tag, size, plan.why))
                        continue
                    if not ok:
                        fail('%s: planned but not drawn: %r' % (tag, dbg))
                        continue
                    drawn += 1
                    g2, pbox, vbox = geo
                    if verdict and (vbox is None or vbox.h < 64):
                        fail('%s: the verdict graph lost its band (%r)'
                             % (tag, vbox))
                    if g2.board.h < 0.30 * g2.frame.h - 1:
                        fail('%s: the board box is %d px of %d'
                             % (tag, g2.board.h, g2.frame.h))
                    for name, p in dbg['plots'].items():
                        # 48 is the SPEC (#1042 verification), not the
                        # module's constant: a lowered constant must fail
                        if p[3] - p[1] < 48 or p[2] <= p[0]:
                            fail('%s: %s plot %r' % (tag, name, p))
                    subs = dict(zip(dbg['names'], dbg['panels']))
                    txt = dbg['texts']
                    for r, tx, pnl in txt:
                        s = subs.get(pnl, pbox)
                        if not (r[0] >= s.x - 0.5 and r[1] >= s.y - 0.5
                                and r[2] <= s.x + s.w + 0.5
                                and r[3] <= s.y + s.h + 0.5):
                            fail('%s: %r leaves its panel %s' % (tag, tx,
                                                                 pnl))
                    for i, (a, ta, _p) in enumerate(txt):
                        for b, tb, _q in txt[i + 1:]:
                            if not (a[2] <= b[0] or b[2] <= a[0]
                                    or a[3] <= b[1] or b[3] <= a[1]):
                                fail('%s: %r overlaps %r' % (tag, ta, tb))
                    texts = [tx for _r, tx, _p in txt]
                    if 'arrangement' in dbg['names'] and \
                            MP.SCREEN_NOTE not in texts:
                        fail('%s: the SCREEN note is not whole: %r'
                             % (tag, [x for x in texts if 'verdict' in x]))
                    if len(dbg['names']) < 3:
                        dropped += 1
                        if not any('dropped' in h for h in dbg['header']):
                            fail('%s: %d panel(s), and the header does not '
                                 'say which were dropped' % (
                                     tag, len(dbg['names'])))
    print('    %d drawn (%d with fewer panels, named), %d declined: %s'
          % (drawn, dropped, declined, ', '.join(declines)))
    if len(_FAIL) == _mark:
        print('  PASS: 5 layouts x 5 ratios x 3 sizes x 2 bands -- every '
              'plot >= %d px, every text whole, inside, unoverlapped'
              % MP.PLOT_MIN_PX)


def _sha(p):
    import hashlib
    with open(p, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


def test_attempts_ledger_from_copies():
    """Run 32 had to render from copies: the boards away from the run dir,
    the ledger named with --attempts-ledger. Both the verdict band and the
    placement panels must read it."""
    _mark = len(_FAIL)
    import make_movie
    d = tempfile.mkdtemp(prefix='t1042c_')
    try:
        bd = os.path.join(d, 'copies')
        ld = os.path.join(d, 'run')
        os.makedirs(bd)
        os.makedirs(ld)
        a = os.path.join(bd, 'a.kicad_pcb')
        b = os.path.join(bd, 'b.kicad_pcb')
        shutil.copy(SEED, a)
        shutil.copy(PLACED, b)
        led = os.path.join(ld, 'ledger.jsonl')
        rows = [{'iteration': 0, 'kind': 'placement', 't': 1000.0,
                 'accepted': True, 'result_sha': _sha(b),
                 'score': {'blocking': 250, 'blocking_by': {'floorplan': 7}}},
                {'iteration': 1, 'kind': 'completion', 't': 2000.0,
                 'accepted': True, 'parent_sha': _sha(b),
                 'result_sha': 'x1', 'score': {'blocking': 40}},
                {'iteration': 2, 'kind': 'completion', 't': 4000.0,
                 'accepted': True, 'parent_sha': 'x1', 'result_sha': 'x2',
                 'score': {'blocking': 30}},
                {'iteration': 3, 'kind': 'classification', 't': 4500.0,
                 'shape': 'placement', 'lever': 'ONE pocket: detail'}]
        with open(led, 'w', encoding='utf-8') as f:
            for r in rows:
                f.write(json.dumps(r) + '\n')
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            got = make_movie.make_movie(
                [a, b], out=os.path.join(d, 'f.gif'), size=1000, quiet=True,
                layout='split', aspect='16:9', camera='off',
                attempts_ledger=led)
        e = err.getvalue()
        for want in ('from converge', 'placement panels: 2 placement '
                     'board(s)', 'x run time',
                     'intent from ledger board_score'):
            if want not in e:
                fail('from copies: %r not in the status lines:\n%s'
                     % (want, e[-600:]))
        if not got or not os.path.isfile(got):
            fail('from copies: no film written')
        t, _w = MP.build_track([('a', a, None), ('b', b, None)], [],
                               ledger=led)
        if t is None or t.beats[-1].floorplan != 7 or \
                t.beats[0].floorplan is not None:
            fail('from copies: floorplans %r'
                 % ([bt.floorplan for bt in t.beats] if t else None,))
        if t is not None and (len(t.flags) != 1
                              or t.flags[0].text
                              != 'ONE pocket (after this film)'):
            fail('from copies: flags %r' % (t.flags,))
    finally:
        shutil.rmtree(d, ignore_errors=True)
    if len(_FAIL) == _mark:
        print('  PASS: --attempts-ledger read by both the band and the '
              'panels, with the boards copied away from the run dir')


def test_exact_frame_sizes_with_the_panels():
    _mark = len(_FAIL)
    import make_movie
    tmp = tempfile.mkdtemp(prefix='t1042f_')
    n = drawn = 0
    try:
        for lk in ('split', 'stacked', 'sidebar', 'inset', 'legacy'):
            for rk in ('16:9', '9:16', '1:1', '4:3'):
                for th in (('dark', 'light') if lk == 'split'
                           else ('light',)):
                    err = io.StringIO()
                    out = os.path.join(tmp, '%s_%s_%s.gif'
                                       % (lk, rk.replace(':', 'x'), th))
                    with contextlib.redirect_stderr(err):
                        got = make_movie.make_movie(
                            [SEED, PLACED], out=out, size=1000, quiet=True,
                            layout=lk, aspect=rk, theme=th, attempts=False,
                            camera='off')
                    n += 1
                    e = err.getvalue()
                    if 'placement panels: 2 placement board(s)' in e:
                        drawn += 1
                    elif 'placement panels: not drawn -- declined' not in e:
                        fail('%s/%s/%s: the panels neither drew nor said '
                             'why: %s' % (lk, rk, th, e[-300:]))
                    r = FL.parse_ratio(rk)
                    want = ((FL.even(1000), FL.even(round(1000 / r)))
                            if r >= 1
                            else (FL.even(round(1000 * r)), FL.even(1000)))
                    with Image.open(got) as im:
                        if im.size != want:
                            fail('%s/%s/%s: %r, declared %r'
                                 % (lk, rk, th, im.size, want))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if drawn != n:
        fail('%d of %d films at size 1000 declined the panels' % (n - drawn,
                                                                   n))
    if len(_FAIL) == _mark:
        print('  PASS: %d films (5 layouts x 4 ratios, both themes on split) '
              'with the panels drawn, each the declared size' % n)


TESTS = (
    test_run32_reproduces_the_table,
    test_panels_name_their_instruments_and_the_screen,
    test_one_point_per_board_and_nothing_before_the_first,
    test_unmeasured_is_said_never_zero,
    test_placement_rows_are_off_the_verdict_axis,
    test_the_panels_say_when_placement_is_settled,
    test_no_subprocess_and_cheap_gates_first,
    test_run_time_is_the_shared_x_axis,
    test_flags_stack_and_a_failure_repaints,
    test_series_colours_are_distinct_in_both_themes,
    test_readable_or_not_drawn_across_layouts_ratios_sizes,
    test_attempts_ledger_from_copies,
    test_exact_frame_sizes_with_the_panels,
)


def main():
    only = sys.argv[1] if len(sys.argv) > 1 else None
    for fn in TESTS:
        if only and only not in fn.__name__:
            continue
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

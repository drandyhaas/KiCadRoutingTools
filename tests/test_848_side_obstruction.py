"""#848: a lane-ledger neighbour is charged the sides it SHARES, not all of it.

Both ledgers already decide WHETHER to charge a neighbour with the symmetric
`own & other` over `sides_occupied` (#835). What each then charged was
`CopperGeometry.rect`, the box over ALL that part's pads on BOTH faces -- so an
F.Cu part escaping past a B.Cu connector with a few through-hole pins was
charged the connector's whole back-side pad field, copper these tracks never
have to share. `legality.rect_on_sides` is the fix, and it lives in `legality`
because BOTH ledgers charge this rectangle: a per-side box in one of them
re-opens the disagreement #841 closed.

What is asserted, and why each arm is here rather than implied by another:

1. THE CENSUS. 20 (ref, side) boxes differ from `rect` over the 22 tracked
   boards, board by board. #848 tabulated 18 over the nine boards it looked at
   and this reproduces those nine exactly; sonde_u is the board it did not
   cover. A census is the only arm that fails if the per-side box is built but
   never differs from the whole-part one.

2. THE CORPUS POSITIVE. #848 predicted the change would be inert. The DEFICIT
   half of that is true -- no `deficit_*` moves on any of the 22 boards, and
   arm 6 pins it -- but SUPPLY and the blocker attribution do move, on
   orangecrab_ext_pll, where U8 is a through-hole part whose back-side field
   was charged against two front-side resistor networks. This arm is the one
   that fails if the resolver is wired up wrong, and it needs no fixture.

3. BOTH LEDGERS, ONE RECT. A spy on the shared kernel `escape.span_eaten`,
   asserting the two ledgers hand it the SAME obstacle list for the same ref.
   Comparing their VALUES cannot do this: they resolve different pitches,
   different bands and different quantization, so they legitimately disagree
   downstream of the rect. The obstacle list is the shared input, and its
   divergence is exactly what re-opens #841.

4-5. THE SEMANTICS, on hand-built fixtures, because the corpus does not
   discriminate them. A holes-in-both-boxes arm (the obvious wrong
   implementation drops a drilled obstruction, which is the #835 rp2350-U6
   lesson relocated into the rectangle); a shared-sides-not-the-neighbour's
   arm; and a union-not-intersection arm. Rows 4 and 5 all pass under an
   implementation that charges the neighbour's own sides, or the first of
   them, which is why they are separate.

6. THE FALLBACKS. `sides=None`, an empty union, and a union naming a side the
   PAD model does not reach all return `geom.rect`. `g.sides` is
   courtyard-derived and `rect_sides` is pad-derived, so the two can
   legitimately disagree, and a caller must never silently LOSE an obstruction
   -- the rule `_blocked_span` already states for its three `.get` maps.

7. SINGLE-SIDED PARTS ARE BIT-IDENTICAL, over the whole corpus. That is a
   property of `extent_local_side` (holes go in both boxes, and a one-sided
   part's pads all pass `_sides_interact`), and it is what makes the census in
   arm 1 small rather than the whole board.

Run: python3 -X utf8 tests/test_848_side_obstruction.py
"""
import os
import sys

RUN_ALL_TIMEOUT = 300

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_HERE, _ROOT, os.path.join(_ROOT, 'py_placer'),
           os.path.join(_ROOT, 'py_router'), os.path.join(_ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import run_utils                                             # noqa: E402
import synth                                                 # noqa: E402
from kicad_parser import parse_kicad_pcb                     # noqa: E402
from placement import escape as E                            # noqa: E402
from placement import legality as L                          # noqa: E402
from placement import routability as R                       # noqa: E402

SKIP_EXIT = 77
FAILURES = []

#: The basis every corpus number below is measured at. Stated, not defaulted:
#: `part_copper_geometry`'s NPTH hole extents grow below 0.20mm clearance, so a
#: census taken at another clearance is a different census.
CLR, TRK, GRID = 0.2, 0.2, 0.05

#: {board basename: {'REF/side', ...}} -- every per-side box that DIFFERS from
#: the whole-part `rect`, at CLR. Recorded by this file (`--census` prints it).
#: #848's own table, for the nine boards it covered: glasgow 5, watchy 5,
#: rp2350 2, tigard 2, ulx3s 2, orangecrab 1, esp_prog 1, kit-dev 0,
#: splitflap 0 -- 18, and every one of those nine reproduces here.
CENSUS = {
    'esp_prog': {'USB1/B'},
    'glasgow_revC': {'J1/B', 'J5/B', 'U1/B', 'U36/B', 'U8/B'},
    'orangecrab_ext_pll': {'U8/B'},
    'rp2350_fpga_eensy_prePlane': {'J1/B', 'U6/B'},
    'sonde_u': {'J1/B', 'J2/F'},
    'tigard': {'J1/B', 'U3/B'},
    'ulx3s': {'AUDIO1/B', 'GPDI1/B'},
    'watchy': {'J2/B', 'SW1/B', 'SW2/B', 'SW3/B', 'SW4/B'},
}

#: Section 6 deliberately holds NO recorded deficit table. It measures both
#: arms in one process instead -- see `the_deficit_did_not_move`.


def check(name, cond, detail=''):
    if cond:
        print('  PASS  %s' % name)
    else:
        FAILURES.append(name)
        print('  FAIL  %s%s' % (name, ('\n        ' + detail) if detail else ''))


def _boards():
    """{basename: path} over the git-TRACKED corpus, or {} when git cannot say.

    Never a plain glob of `kicad_files/`: a used working copy carries 11
    untracked generated boards, and the resulting "33" was cited in production
    code once and had to be retracted.
    """
    return {os.path.splitext(os.path.basename(p))[0]: os.path.join(_ROOT, p)
            for p in run_utils.corpus_boards()}


def _geom(path):
    pcb = parse_kicad_pcb(path)
    return pcb, L.part_copper_geometry(pcb.footprints or {}, CLR)


# --- 1. the census ----------------------------------------------------------

def the_census(boards):
    seen = {}
    single_sided_identical = 0
    for name, path in sorted(boards.items()):
        _pcb, geom = _geom(path)
        diff = set()
        for ref, g in geom.items():
            for side, box in g.rect_sides.items():
                if box != g.rect:
                    diff.add('%s/%s' % (ref, side))
                elif len(g.rect_sides) == 1:
                    single_sided_identical += 1
        if diff:
            seen[name] = diff
    check('census: the differing (ref, side) boxes are exactly the recorded set',
          seen == CENSUS,
          'only here: %r\nonly recorded: %r'
          % ({k: sorted(v - CENSUS.get(k, set())) for k, v in seen.items()
              if v - CENSUS.get(k, set())},
             {k: sorted(v - seen.get(k, set())) for k, v in CENSUS.items()
              if v - seen.get(k, set())}))
    total = sum(len(v) for v in seen.values())
    check('census: 20 boxes differ at all, so the resolver is not inert',
          total == 20, 'got %d' % total)
    # ...and the negative half: a per-side box that differed EVERYWHERE would
    # satisfy the equality above only if CENSUS were re-recorded, so pin the
    # order of magnitude against the parts that were graded.
    check('census: single-sided parts are bit-identical in bulk',
          single_sided_identical > 500, 'got %d' % single_sided_identical)


# --- 2. the corpus positive -------------------------------------------------

def the_corpus_positive(boards):
    path = boards.get('orangecrab_ext_pll')
    if path is None:
        check('corpus positive: orangecrab_ext_pll is present', False)
        return
    pcb = parse_kicad_pcb(run_utils.evidence(path, 'the #848 witness board'))
    ctx = R.board_lane_context(pcb, CLR, pcb_file=path)
    rows = {}
    for ref in ('RN3', 'RN5'):
        rows[ref] = {r['face']: r for r in R.face_lane_ledger(
            pcb, ref, clearance=CLR, track_width=TRK, grid_step=GRID,
            pcb_file=path, context=ctx)}
    # U8 is drilled, so it occupies both faces and is charged against these
    # front-side networks. What it may charge is its FRONT copper.
    e3 = dict(rows['RN3']['E']['eaten_by'])
    e5 = dict(rows['RN5']['E']['eaten_by'])
    check('positive: RN3 east charges U8 0.75 lanes, not the 2.63 of its whole box',
          abs(e3.get('U8', -1) - 0.75) < 1e-9, 'eaten_by=%r' % (e3,))
    check('positive: RN3 east supply is 2, not 1',
          rows['RN3']['E']['supply_finest_grid'] == 2
          and rows['RN3']['E']['supply_routed_grid'] == 2,
          '%r' % (rows['RN3']['E'],))
    check('positive: RN5 east no longer charges U8 at all',
          'U8' not in e5, 'eaten_by=%r' % (e5,))
    # ...and the same two refs on the OTHER ledger, which is the point of #841
    # staying closed: one rectangle, so both instruments move together.
    esc = {p.ref: p for p in E.escape_ledger(pcb, refs=['RN3', 'RN5'],
                                             pcb_file=path,
                                             track_width=TRK, clearance=CLR)}
    f3 = {f.face: f for f in esc['RN3'].faces}['east']
    f5 = {f.face: f for f in esc['RN5'].faces}['east']
    check('positive: escape RN3 east blocked 0.80mm (was 1.15) and supply 2',
          abs(f3.blocked_mm - 0.80) < 1e-4 and f3.supply == 2,
          'blocked=%r supply=%r' % (f3.blocked_mm, f3.supply))
    check('positive: escape RN5 east drops U8 from its blockers',
          'U8' not in f5.blockers, 'blockers=%r' % (f5.blockers,))


# --- 3. both ledgers, one rect ----------------------------------------------

def both_ledgers_get_the_same_rect(boards):
    path = boards.get('orangecrab_ext_pll')
    if path is None:
        check('one rect: orangecrab_ext_pll is present', False)
        return
    pcb = parse_kicad_pcb(path)
    real = E.span_eaten
    seen = {}

    def spy(lo, hi, band, horizontal, obstacles):
        seen.setdefault((round(lo, 6), round(hi, 6), round(band[0], 6),
                         round(band[1], 6)), []).append(
            {r: tuple(round(v, 9) for v in rect) for r, rect in obstacles})
        return real(lo, hi, band, horizontal, obstacles)

    E.span_eaten = spy
    try:
        R.face_lane_ledger(pcb, 'RN3', clearance=CLR, track_width=TRK,
                           grid_step=GRID, pcb_file=path)
        lane = dict(seen)
        seen.clear()
        E.escape_ledger(pcb, refs=['RN3'], pcb_file=path,
                        track_width=TRK, clearance=CLR)
        esc = dict(seen)
    finally:
        E.span_eaten = real
    check('one rect: the spy landed on both ledgers',
          len(lane) == 4 and len(esc) == 4,
          'lane keys %d, escape keys %d' % (len(lane), len(esc)))
    # The two ledgers resolve DIFFERENT bands (raw vs grid-quantized lane), so
    # the keys do not line up; what must agree is the rect each ref
    # contributes, over the refs both charged.
    lane_rects, esc_rects = {}, {}
    for calls in lane.values():
        for d in calls:
            lane_rects.update(d)
    for calls in esc.values():
        for d in calls:
            esc_rects.update(d)
    shared = set(lane_rects) & set(esc_rects)
    bad = {r: (lane_rects[r], esc_rects[r]) for r in shared
           if lane_rects[r] != esc_rects[r]}
    check('one rect: every neighbour both ledgers charge gets an identical box',
          not bad and len(shared) > 20,
          'shared=%d disagreeing=%r' % (len(shared), sorted(bad)[:4]))
    check('one rect: U8 is among them and is its FRONT box in both',
          'U8' in shared, 'shared refs: %d' % len(shared))


# --- 4-5. the semantics, on fixtures ----------------------------------------

def _pad(num, x, y, *, w=0.6, h=0.6, layers='"F.Cu"', drill=None, net=0):
    kind = 'thru_hole circle' if drill else 'smd rect'
    hole = '\n\t\t\t(drill %s)' % drill if drill else ''
    return ('\t\t(pad "%s" %s\n\t\t\t(at %s %s)\n\t\t\t(size %s %s)%s\n'
            '\t\t\t(layers %s)\n\t\t\t(net %d "N%d")\n\t\t\t(uuid "p%s")\n\t\t)\n'
            % (num, kind, x, y, w, h, hole, layers, net, net, num))


#: The fixture's geometry, so every expected number below is arithmetic on
#: these rather than a recording. U1 is a COLUMN of six pads, so its east face
#: is long enough for a neighbour to cover part of it and not the rest -- a
#: short face is covered either wholly or not at all and discriminates nothing.
U1_AT = (20.0, 20.0)
J1_AT = (22.5, 20.0)
PAD = 0.6                 # U1's pad, square
PIN = 0.8                 # J1's through pin, square-ish
U1_EAST_X = U1_AT[0] + 1.5 + PAD / 2.0                 # 21.8
U1_FACE_LO = U1_AT[1] - 2.5 - PAD / 2.0                # 17.2
U1_FACE_HI = U1_AT[1] + 2.5 + PAD / 2.0                # 22.8
PINS_LO = J1_AT[1] - 0.5 - PIN / 2.0                   # 19.1
PINS_HI = J1_AT[1] + 0.5 + PIN / 2.0                   # 20.9


def _fixture(neighbour_pads, *, neighbour_layer='B.Cu', tmpdir):
    """A 2-part board: a front-side column U1 and a neighbour J1 east of it."""
    u1 = ''.join(_pad(str(i + 1), 1.5, -2.5 + 1.0 * i, net=i + 1)
                 for i in range(6))
    fps = (synth.footprint_text('U1', U1_AT[0], U1_AT[1], pads=0, extra=u1)
           + synth.footprint_text('J1', J1_AT[0], J1_AT[1], pads=0,
                                  layer=neighbour_layer,
                                  extra=''.join(neighbour_pads)))
    path = os.path.join(tmpdir, 'f848.kicad_pcb')
    synth.write_board(synth.board_text(fps, nets=range(0, 8)), path)
    return path


def the_semantics(tmpdir):
    # J1 is a BACK-side part carrying two through pins near U1's east face and
    # a wide back-side field further out. U1 is front-only, so the shared side
    # is {'F'} and only the drilled pins reach it.
    tht = [_pad('1', 0.0, -0.5, w=PIN, h=PIN, drill=0.4,
                layers='"*.Cu" "*.Mask"'),
           _pad('2', 0.0, 0.5, w=PIN, h=PIN, drill=0.4,
                layers='"*.Cu" "*.Mask"')]
    smd_b = [_pad('3', 0.0, -3.5, w=1.0, h=1.0, layers='"B.Cu"'),
             _pad('4', 0.0, 3.5, w=1.0, h=1.0, layers='"B.Cu"')]
    path = _fixture(tht + smd_b, tmpdir=tmpdir)
    pcb = parse_kicad_pcb(run_utils.evidence(path, 'the #848 fixture'))
    geom = L.part_copper_geometry(pcb.footprints or {}, CLR)
    check('fixture: both parts are modelled', set(geom) == {'U1', 'J1'},
          'geom=%r' % sorted(geom))
    g = geom.get('J1')
    if g is None:
        return
    F, B = g.rect_sides.get('F'), g.rect_sides.get('B')
    check('fixture: J1 reaches both sides', F is not None and B is not None,
          'rect_sides=%r' % (g.rect_sides,))
    if F is None or B is None:
        return
    check('holes are in BOTH side boxes: the front box is the drilled pins',
          abs(F[1] - PINS_LO) < 1e-6 and abs(F[3] - PINS_HI) < 1e-6,
          'F=%r expected y in [%r, %r]' % (F, PINS_LO, PINS_HI))
    check('the back box is the whole part, and equals `rect`', B == g.rect,
          'B=%r rect=%r' % (B, g.rect))
    check('and the front box is strictly smaller, so the fixture bites',
          (F[3] - F[1]) < (B[3] - B[1]) / 2.0, 'F=%r B=%r' % (F, B))

    # SHARED, not the neighbour's own sides: U1 is front-only, J1 is on both.
    check('shared sides: {F} gives the pins, not J1\'s own {F, B}',
          L.rect_on_sides(g, frozenset({'F'})) == F)
    check('shared sides: {F, B} unions to the whole part',
          L.rect_on_sides(g, frozenset({'F', 'B'})) == g.rect)
    # UNION, not intersection and not the first side: the two boxes are
    # deliberately different, so an intersection would be strictly smaller
    # than either and "the first side" would be one of them.
    u = L.rect_on_sides(g, frozenset({'F', 'B'}))
    check('union, not intersection', (u[3] - u[1]) >= (B[3] - B[1]) - 1e-9,
          'u=%r F=%r B=%r' % (u, F, B))
    # THE FALLBACKS: never lose an obstruction.
    check('fallback: sides=None is the whole box',
          L.rect_on_sides(g, None) == g.rect)
    check('fallback: an empty union is the whole box',
          L.rect_on_sides(g, frozenset()) == g.rect)
    check('fallback: a side the pad model does not reach is the whole box',
          L.rect_on_sides(g, frozenset({'X'})) == g.rect)
    check('fallback: an unmodelled part has no side boxes to read',
          L.rect_on_sides(L.CopperGeometry(ref='z', rect=(0, 0, 1, 1),
                                           copper=(0, 0, 1, 1), pads={},
                                           modelled=False, rect_sides={}),
                          frozenset({'F'})) == (0, 0, 1, 1))

    # ...and the LEDGER reads it: U1's east face is charged the pins only.
    ctx = R.board_lane_context(pcb, CLR, pcb_file=path)
    rows = {r['face']: r for r in R.face_lane_ledger(
        pcb, 'U1', clearance=CLR, track_width=TRK, grid_step=GRID,
        pcb_file=path, context=ctx)}
    east = rows.get('E', {})
    eaten = dict(east.get('eaten_by', []))
    face = U1_FACE_HI - U1_FACE_LO                       # 5.6mm
    pins = PINS_HI - PINS_LO                             # 1.8mm
    pitch = TRK + CLR                                    # 0.4mm, grid-aligned
    check('the fixture face is the length the arithmetic assumes',
          abs(east.get('length_mm', 0) - face) < 1e-6,
          'length_mm=%r expected %r' % (east.get('length_mm'), face))
    check('the ledger charges J1 its FRONT copper (the pins), not its whole box',
          abs(eaten.get('J1', -1) - round(pins / pitch, 2)) < 1e-9,
          'eaten_by=%r expected J1 %.2f' % (eaten, pins / pitch))
    check('...so the face keeps the supply the back-side field used to eat',
          east.get('supply_finest_grid') == int((face - pins) // pitch),
          'row=%r expected %d' % (east, int((face - pins) // pitch)))
    # The negative control, on the SAME fixture: charged whole-part, J1's box
    # spans the entire face and the supply is 0. That is what this arm fails
    # back to if `rect_on_sides` is reverted to `geom.rect`.
    whole = L.rect_on_sides(g, None)
    check('negative control: the whole box covers the face end to end',
          whole[1] <= U1_FACE_LO and whole[3] >= U1_FACE_HI,
          'whole=%r face=[%r, %r]' % (whole, U1_FACE_LO, U1_FACE_HI))


# --- 6. the deficit half of the issue's prediction --------------------------

def _deficits(boards):
    out = {}
    for name, path in sorted(boards.items()):
        pcb = parse_kicad_pcb(path)
        ctx = R.board_lane_context(pcb, CLR, pcb_file=path)
        tot = 0
        for ref in E.fine_pitch_parts(pcb):
            for r in R.face_lane_ledger(pcb, ref, clearance=CLR,
                                        track_width=TRK, grid_step=GRID,
                                        pcb_file=path, context=ctx):
                tot += r['deficit_finest_grid']
        if tot:
            out[name] = tot
    return out


def the_deficit_did_not_move(boards):
    """#848's own prediction, measured by TOGGLING #848 -- not by a constant.

    An earlier draft of this arm pinned the per-board deficit totals as
    literals recorded at the #848 tip. That is a recording of the whole
    ENGINE, not of #848: the very next commit (#850, the demand half) moved
    every one of them and this arm failed for a reason it does not name. A
    change detector that fires on unrelated changes is not a change detector.

    So both arms are measured in ONE process: `rect_on_sides` charged
    per-shared-side, and then monkeypatched back to the whole-part box, which
    is the state before #848. Whatever else has moved since moves both.
    """
    now = _deficits(boards)
    real = L.rect_on_sides
    try:
        L.rect_on_sides = lambda geom, sides=None: geom.rect
        before = _deficits(boards)
    finally:
        L.rect_on_sides = real
    check('the deficit is unmoved by #848 on all 22 boards, as it predicted',
          now == before,
          'per-shared-side %r\nwhole-part      %r' % (now, before))
    # ...and the arm is not vacuous: SOMETHING must be in deficit, or an
    # engine that reported zero everywhere would satisfy the equality.
    check('and the corpus carries a deficit for that to be a statement about',
          sum(now.values()) > 100, 'total %d' % sum(now.values()))


def main():
    boards = _boards()
    if not boards:
        print('SKIP: git could not name the tracked corpus')
        return SKIP_EXIT
    import tempfile
    print('1. the census')
    the_census(boards)
    print('2. the corpus positive')
    the_corpus_positive(boards)
    print('3. both ledgers, one rect')
    both_ledgers_get_the_same_rect(boards)
    print('4-5. the semantics, on a fixture')
    with tempfile.TemporaryDirectory() as td:
        the_semantics(td)
    print('6. the deficit half of the prediction')
    the_deficit_did_not_move(boards)
    if FAILURES:
        print('\nFAIL: %d check(s): %s' % (len(FAILURES), ', '.join(FAILURES)))
        return 1
    print('\nOK')
    return 0


if __name__ == '__main__':
    sys.exit(main())

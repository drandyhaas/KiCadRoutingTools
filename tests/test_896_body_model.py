"""#896: one body model -- the ladders, the refusals, and the run-25 numbers.

The acceptance rows come from `wk/run25`'s lap 3 and lap 5, staged into
`tests/fixtures/run25/` (`wk/` is gitignored, so a test reading it there would
be green-while-covering-nothing on every other machine). Each row is here
because it is a case where one rung of the ladder CHANGES the answer, not
because it was convenient to assert.

The three signed numbers were measured by hand, by an adversarial reviewer
writing its own geometry during run 25, before any of this code existed
(JOURNAL_placement.md entries at lap 3 and lap 5). They are the reason the
issue exists, and reproducing them is what says the model reads the same
geometry a human read.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path[:0] = [os.path.join(ROOT, 'py_router'),
                os.path.join(ROOT, 'py_placer'),
                os.path.join(ROOT, 'py_tools')]
sys.path.insert(0, HERE)

RUN_ALL_TIMEOUT = 900
RUN_ALL_FAST_OK = True

FAILURES = []


def check(name, cond, detail=''):
    if cond:
        print(f'  PASS: {name}')
    else:
        FAILURES.append(f'{name}{(" -- " + detail) if detail else ""}')
        print(f'  FAIL: {name} -- {detail}')


def fixture(stem):
    return os.path.join(ROOT, 'tests', 'fixtures', 'run25',
                        f'esp_prog_{stem}.kicad_pcb')


def _pcb(path):
    from kicad_parser import parse_kicad_pcb
    return parse_kicad_pcb(path)


def _bodies(path):
    from placement.body import board_bodies
    return board_bodies(_pcb(path), path)


def _board_rect(fp, local):
    from placement.legality import rotate_local_bounds
    x0, y0, x1, y1 = rotate_local_bounds(*local, fp.rotation or 0.0)
    return (fp.x + x0, fp.y + y0, fp.x + x1, fp.y + y1)


# The rows. Each: (lap, ref_a, ref_b, expected_gap_mm, source_a, source_b, why)
#
# `wk/run25/esp_prog/JOURNAL_placement.md` records the first two at lap 3
# ("CON1 housing top 93.400 vs CON2 body bottom 93.520 = -0.120mm";
# "CON2 body west 126.810 vs U2 body east 126.900 = -0.090mm") and the third at
# lap 5 ("U1 body end 102.410 overlaps Y1 body top 102.277 by 0.133mm").
ACCEPTANCE = [
    ('lap3', 'CON1', 'CON2', -0.1200, 'silk', 'silk',
     'two hand-drawn OLIMEX connector housings; neither draws .Fab, so silk '
     'is the only body either has'),
    ('lap3', 'CON2', 'U2', -0.0900, 'silk', 'silk',
     'header plastic into the SOT89 marking'),
    ('lap5', 'U1', 'Y1', -0.1330, 'silk', 'fab',
     'THE mixed-source row: U1 draws no .Fab and falls to silk, Y1 draws one '
     'and does not. A single-rung model gets this pair wrong whichever rung '
     'it picks.'),
]


def test_acceptance_numbers():
    """The three collisions run 25 paid two laps to find."""
    from placement.legality import rect_gap
    for lap, ra, rb, want, sa, sb, why in ACCEPTANCE:
        path = fixture(lap)
        pcb = _pcb(path)
        bodies = _bodies(path)
        ga, gb = bodies.get(ra), bodies.get(rb)
        # Named refusals rather than a KeyError/TypeError. A mutation battery
        # over this module killed three rows by TRACEBACK, and a traceback is
        # the same non-zero exit a satisfied assertion gives -- it says the
        # gate reacted, not that it saw the thing it names.
        if ga is None or gb is None:
            check(f'{lap} {ra}<->{rb}: both parts are in the body model',
                  False, f'{ra}={ga is not None} {rb}={gb is not None}')
            continue
        if ga.drawn_local is None or gb.drawn_local is None:
            check(f'{lap} {ra}<->{rb}: both parts have a DRAWN body', False,
                  f'{ra} drawn={ga.drawn_source} {rb} drawn={gb.drawn_source}')
            continue
        rect_a = _board_rect(pcb.footprints[ra], ga.drawn_local)
        rect_b = _board_rect(pcb.footprints[rb], gb.drawn_local)
        got = round(rect_gap(rect_a, rect_b), 4)
        check(f'{lap} {ra}<->{rb} = {want:+.4f}mm', got == want,
              f'got {got:+.4f} ({why})')
        check(f'{lap} {ra}<->{rb} sources {sa}/{sb}',
              (ga.drawn_source, gb.drawn_source) == (sa, sb),
              f'got {ga.drawn_source}/{gb.drawn_source}')


def test_the_channel_reports_them():
    """Not just the model: `grade_body_overlap`'s own drawn-body channel.

    The model could be right and the instrument still blind -- which is
    precisely what #896 is about, since the fab channel HAD the right idea and
    a one-rung ladder.
    """
    from placement.legality import grade_body_overlap
    for lap in ('lap3', 'lap5'):
        path = fixture(lap)
        g = grade_body_overlap(_pcb(path), 0.15, pcb_file=path)
        got = {(q.a, q.b): q.depth_mm for q in g['pairs'] if q.kind == 'fab'}
        for l2, ra, rb, want, _sa, _sb, _why in ACCEPTANCE:
            if l2 != lap:
                continue
            key = (min(ra, rb), max(ra, rb))
            check(f'{lap} channel reports {ra}<->{rb}', key in got,
                  f'pairs present: {sorted(got)}')
            if key in got:
                check(f'{lap} {ra}<->{rb} depth {abs(want)}',
                      abs(got[key] - abs(want)) < 5e-4,
                      f'got depth {got[key]}')


def test_no_offset_is_applied_to_silk():
    """The issue proposes expanding silk by "~0.2 mm on OLIMEX". Refuted by
    its own acceptance numbers, and the refutation is asserted rather than
    left in a comment: with a 0.2mm expansion on both parts the three numbers
    become roughly -0.52 / -0.49 / -0.53.

    This is a CHANGE DETECTOR. If someone adds an expansion constant, the
    numbers above already move -- but this states the magnitude, so the diff
    has to argue with it rather than re-record three literals.
    """
    from placement.legality import rect_gap
    lap, ra, rb, want = 'lap3', 'CON1', 'CON2', -0.1200
    path = fixture(lap)
    pcb = _pcb(path)
    bodies = _bodies(path)

    def grown(ref, by):
        g = bodies[ref]
        if g.drawn_local is None:
            return None
        x0, y0, x1, y1 = g.drawn_local
        return _board_rect(pcb.footprints[ref],
                           (x0 - by, y0 - by, x1 + by, y1 + by))
    _a, _b = grown(ra, 0.2), grown(rb, 0.2)
    if _a is None or _b is None:
        check('both parts have a drawn body to expand', False,
              f'{ra}/{rb} drawn bodies missing')
        return
    got = round(rect_gap(_a, _b), 4)
    check('a 0.2mm silk expansion would break the issue\'s own numbers',
          abs(got - (want - 0.4)) < 1e-6,
          f'expanded gap {got:+.4f}, unexpanded {want:+.4f}')


def test_silk_is_refused_where_it_is_not_a_body():
    """Two refusals, both measured on this board.

    Q1/Q2 are SOT23s whose silk is a pair of ticks INSIDE the pad field: taken
    bare it would shrink them from a 3.610 x 2.902mm pad box to 0.838 x 2.845,
    which is the unsafe direction. The three reference-less blocks are OLIMEX
    logos with no pads at all -- decoration, not parts to collide with.
    """
    path = fixture('lap3')
    bodies = _bodies(path)
    pcb = _pcb(path)
    for ref in ('Q1', 'Q2'):
        g = bodies.get(ref)
        if g is None:
            check(f'{ref} is in the body model', False, 'absent')
            continue
        check(f'{ref} silk refused as a tick mark',
              g.silk_rejected and g.drawn_source == 'none'
              and g.source == 'pad_bbox',
              f'silk_rejected={g.silk_rejected} drawn={g.drawn_source} '
              f'source={g.source}')
    padless = [r for r, fp in pcb.footprints.items() if not (fp.pads or ())]
    check('the board carries pad-less blocks to refuse', len(padless) >= 3,
          f'{len(padless)} found')
    for ref in padless:
        g = bodies.get(ref)
        if g is None:
            check(f'{ref} is in the body model', False, 'absent')
            continue
        check(f'{ref} (pad-less) claims no silk body',
              g.drawn_source != 'silk', f'drawn={g.drawn_source}')


def test_occupancy_never_shrinks_below_the_pads():
    """The rect the grading chain consumes is monotone.

    A .Fab body is routinely NARROWER than the pads it sits between, so a
    model handing the bare body to an occupancy consumer silently removes
    findings -- measured over the corpus, three on ulx3s and two on esp_prog
    before this was fixed.
    """
    from placement.utility import compute_footprint_bbox_local
    import run_utils
    boards = run_utils.corpus_boards()
    check('corpus is not empty', len(boards) >= 22, f'{len(boards)} boards')
    shrunk = []
    checked = 0
    for b in boards:
        pcb = _pcb(b)
        for ref, g in _bodies(b).items():
            fp = pcb.footprints.get(ref)
            if fp is None or not (fp.pads or ()) or g.occupancy_local is None:
                continue
            try:
                pads = compute_footprint_bbox_local(fp)
            except Exception:                                # noqa: BLE001
                continue
            checked += 1
            o = g.occupancy_local
            if (o[0] > pads[0] + 1e-9 or o[1] > pads[1] + 1e-9
                    or o[2] < pads[2] - 1e-9 or o[3] < pads[3] - 1e-9):
                shrunk.append((os.path.basename(b), ref, g.source))
    check('something was checked', checked > 500, f'{checked} parts')
    check('no occupancy rect is smaller than its own pad bbox',
          shrunk == [], f'{len(shrunk)}: {shrunk[:5]}')


def test_a_silk_body_is_never_smaller_than_its_pads():
    """Rule 1, asserted where it can FAIL.

    Added because a mutation battery removed the union from the silk rung and
    every other arm here still passed: the acceptance numbers cannot see it
    (CON1/CON2/U2's silk already contains their pad field, so the union is a
    no-op there) and the occupancy arm cannot see it either (occupancy unions
    with the pads again at the end regardless). The union's whole job is to
    stop `drawn_local` -- the rect the BODY channel grades -- from being
    narrower than the copper it sits on, and only this arm watches that.
    """
    from placement.utility import compute_footprint_bbox_local
    import run_utils
    boards = run_utils.corpus_boards()
    checked, shrunk = 0, []
    for b in boards:
        pcb = _pcb(b)
        for ref, g in _bodies(b).items():
            if g.drawn_source != 'silk' or g.drawn_local is None:
                continue
            fp = pcb.footprints.get(ref)
            if fp is None or not (fp.pads or ()):
                continue
            pads = compute_footprint_bbox_local(fp)
            checked += 1
            d = g.drawn_local
            if (d[0] > pads[0] + 1e-9 or d[1] > pads[1] + 1e-9
                    or d[2] < pads[2] - 1e-9 or d[3] < pads[3] - 1e-9):
                shrunk.append((os.path.basename(b), ref))
    # Non-vacuity: this arm is worthless if no part on the corpus takes the
    # silk rung at all, which is exactly what a future ladder change could
    # cause without anyone noticing.
    check('some corpus part takes the silk rung', checked > 0,
          f'{checked} silk-sourced parts')
    check('no silk body is narrower than its own pads', shrunk == [],
          f'{len(shrunk)}: {shrunk[:5]}')


def test_the_courtyard_is_not_on_the_drawn_ladder():
    """A courtyard is a body PLUS an assembly margin plus any shell overhang.

    Run-6 calibrated the courtyard channel and the fab channel apart for that
    reason -- the courtyard ships frac-1.0 containment on four healthy boards
    -- so putting the courtyard on the drawn ladder would quietly re-merge
    them. esp_prog cannot see this (no part on it draws a courtyard at all),
    which is why the arm sweeps the corpus for parts that draw BOTH.
    """
    from placement.parser import extract_courtyard_sides, extract_fab_sides
    import run_utils
    both, wrong = 0, []
    for b in run_utils.corpus_boards():
        crt, fab = extract_courtyard_sides(b), extract_fab_sides(b)
        bodies = _bodies(b)
        for ref in set(crt) & set(fab):
            g = bodies.get(ref)
            if g is None:
                continue
            both += 1
            if g.drawn_source != 'fab':
                wrong.append((os.path.basename(b), ref, g.drawn_source))
            if g.source != 'courtyard':
                wrong.append((os.path.basename(b), ref,
                              f'occupancy={g.source}'))
    check('the corpus has parts drawing both a courtyard and a fab body',
          both > 100, f'{both} parts')
    check('such a part reads fab as its DRAWN body and courtyard as its '
          'occupancy', wrong == [], f'{len(wrong)}: {wrong[:5]}')


def test_a_silk_body_never_gates():
    """The rule with the most consequences in #896, asserted both ways.

    Added because a mutation row that DELETED the exclusion survived every
    other arm here: the model was right, the disclosure was right, and nothing
    watched the one decision that can turn a board NOT BUILDABLE.

    Both directions matter, and the positive control is the half that is easy
    to leave out. esp_prog's R1 clears U2's real body by 2.1mm and reads 89%
    CONTAINED inside U2's silk square -- four corner brackets 5.2mm apart
    around a 4.5mm part -- so the pair MUST be in the disclosed census and
    MUST NOT be in the gating one. An arm checking only the gating side would
    pass just as well on a build that had stopped producing the pair at all.
    """
    from placement.legality import grade_body_overlap
    import run_utils

    path = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
    g = grade_body_overlap(_pcb(path), 0.15, pcb_file=path)
    disclosed = {(q.a, q.b) for q in g['pairs'] if q.contained}
    check('esp_prog DISCLOSES R1<->U2 as contained (the positive control -- '
          'without it the arm below passes vacuously)',
          ('R1', 'U2') in disclosed, f'contained pairs: {sorted(disclosed)}')
    gating = {(q.a, q.b) for q in g['containment_blocking_pairs']}
    check('esp_prog does not GATE on R1<->U2', ('R1', 'U2') not in gating,
          f'gating pairs: {sorted(gating)}')

    offenders = []
    for b in run_utils.corpus_boards():
        gg = grade_body_overlap(_pcb(b), 0.15, pcb_file=b)
        silk_drawn = {r for r, s in (gg.get('body_sources') or {}).items()
                      if s == 'silk'}
        silk_occ = {r for r, x in _bodies(b).items() if x.source == 'silk'}
        for q in gg['containment_blocking_pairs']:
            if q.a in silk_drawn or q.b in silk_drawn:
                offenders.append((os.path.basename(b), 'containment',
                                  q.a, q.b))
        for q in gg['courtyard_blocking_pairs']:
            if q.a in silk_occ or q.b in silk_occ:
                offenders.append((os.path.basename(b), 'courtyard', q.a, q.b))
    check('no corpus board gates on a silk-sourced body', offenders == [],
          f'{len(offenders)}: {offenders[:5]}')


def test_seam_is_signed_and_named():
    """A seam and a collision are ONE number, and it says what it rests on."""
    from placement.legality import grade_body_overlap
    g3 = grade_body_overlap(_pcb(fixture('lap3')), 0.15,
                            pcb_file=fixture('lap3'))
    seam = g3.get('body_seam')
    check('lap3 reports a seam', seam is not None)
    if seam:
        check('lap3 seam is the U1<->Y1 overlap, signed negative',
              seam['mm'] == -0.133 and {seam['ref_a'], seam['ref_b']}
              == {'U1', 'Y1'}, str(seam))
        check('lap3 seam names both sources',
              {seam['source_a'], seam['source_b']} == {'silk', 'fab'},
              str(seam))


TESTS = [test_acceptance_numbers, test_the_channel_reports_them,
         test_no_offset_is_applied_to_silk,
         test_silk_is_refused_where_it_is_not_a_body,
         test_occupancy_never_shrinks_below_the_pads,
         test_a_silk_body_is_never_smaller_than_its_pads,
         test_the_courtyard_is_not_on_the_drawn_ladder,
         test_a_silk_body_never_gates,
         test_seam_is_signed_and_named]


def main():
    for t in TESTS:
        print(f'--- {t.__name__}')
        t()
    print(f"\n{'FAIL' if FAILURES else 'PASS'}: #896 body model, "
          f"{len(FAILURES)} failure(s)")
    for f in FAILURES:
        print(f'  - {f}')
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())

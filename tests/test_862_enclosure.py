#!/usr/bin/env python3
"""#862: the enclosure test -- the kernel, on arithmetic derived by hand.

`escape.face_of` calls a pad INTERIOR when its copper box is further than
`max(pad_pitch/2, INTERIOR_EPS)` from all four edges of the part's copper box.
That box is the union of EVERY pad's copper, and a box cannot tell a few small
marks outside the pad field from a ring that encloses it: on ulx3s U1, eight
UNNETTED 0.127 x 0.508mm alignment marks sit 0.954mm beyond the ball field on
all four sides, so all 379 netted balls read interior and the part reports
demand 0 on every face.

This file is the gate for the OCCUPANCY half that answers it. Sections 1-2
cover the kernel alone -- `free_run` and `pad_band` -- because the corpus
numbers that follow in later sections are only meaningful if the arithmetic
under them was specified before it was measured. EVERY expected value here is
derived in the comment above it, from the fixture's own numbers; none was read
off an implementation.

Deliberately unit-only and fixture-only: it runs no board and takes well under
a second, so it stays a FAST test that `run_all.py --fast` collects.

    python3 -X utf8 tests/test_862_enclosure.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'py_placer'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'py_router'))

from placement import escape as E                              # noqa: E402

RUN_ALL_TIMEOUT = 300

FAILURES = []
PASSED = []
SECTIONS = 7


def check(name, cond, detail=''):
    if cond:
        PASSED.append(name)
    else:
        FAILURES.append('{}{}'.format(name, ': ' + detail if detail else ''))


def near(a, b, eps=1e-9):
    return abs(a - b) < eps


# ---------------------------------------------------------------- section 1
def the_free_run_kernel():
    """`free_run` returns the largest CONTIGUOUS gap, not the total.

    The distinction is the whole reason this is not `span_eaten`: measured on
    the tracked corpus at clearance 0.09, thresholding on the total instead
    moves nine refs and takes `qfn_interior_pads` U1 from 5 interior pads to
    2 -- while being a value no-op at clearance 0.20 and 0.25, so a test
    written at the census basis alone cannot tell the two apart.
    """
    # Nothing in the way: the whole strip is free.
    check('empty obstacle list gives the whole strip',
          near(E.free_run(0.0, 1.0, (0.0, 1.0), True, []), 1.0))

    # One obstacle spanning x 0.0..1.0 inside the band covers everything.
    full = [(0.0, 0.0, 1.0, 1.0)]
    check('an obstacle covering the strip leaves nothing',
          near(E.free_run(0.0, 1.0, (0.0, 1.0), True, full), 0.0))

    # THE DISCRIMINATING FIXTURE. Strip [0, 1]. Three obstacles at
    # x 0.2..0.3, 0.5..0.6 and 0.8..0.9 leave gaps 0.2 / 0.2 / 0.2 / 0.1,
    # total free 0.7 but the largest single run 0.2. A 0.3mm track fits in
    # NONE of them, and a rule that thresholds on the total would pass it.
    three = [(0.2, 0.0, 0.3, 1.0), (0.5, 0.0, 0.6, 1.0), (0.8, 0.0, 0.9, 1.0)]
    got = E.free_run(0.0, 1.0, (0.0, 1.0), True, three)
    check('three small gaps do not add up to one lane', near(got, 0.2),
          'largest run {} (total free is 0.7)'.format(got))

    # The leading and trailing gaps count too: one obstacle at x 0.4..0.6
    # leaves 0.4 before it and 0.4 after it.
    mid = [(0.4, 0.0, 0.6, 1.0)]
    check('the leading and trailing gaps are runs too',
          near(E.free_run(0.0, 1.0, (0.0, 1.0), True, mid), 0.4))

    # OVERLAPPING obstacles are unioned, not summed: 0.2..0.5 and 0.4..0.7
    # cover 0.2..0.7, leaving 0.3 before and 0.3 after.
    lap = [(0.2, 0.0, 0.5, 1.0), (0.4, 0.0, 0.7, 1.0)]
    check('overlapping obstacles are unioned',
          near(E.free_run(0.0, 1.0, (0.0, 1.0), True, lap), 0.3))

    # `grow` inflates ALONG the strip. The three obstacles at 0.2..0.3,
    # 0.5..0.6 and 0.8..0.9, grown by 0.1 on each side, become 0.1..0.4,
    # 0.4..0.7 and 0.7..1.0 -- which between them cover 0.1..1.0, so the only
    # free run left is the leading 0.0..0.1. Every 0.2 gap a bare measurement
    # saw is eaten by the clearance a track may not touch.
    grown = [(0.2, 0.0, 0.3, 1.0), (0.5, 0.0, 0.6, 1.0), (0.8, 0.0, 0.9, 1.0)]
    check('grow closes the gaps a clearance eats',
          near(E.free_run(0.0, 1.0, (0.0, 1.0), True, grown, grow=0.1), 0.1))

    # Grow is NOT applied across the band, so an obstacle that does not reach
    # the band is not pulled into it by the clearance. This one spans
    # y 2.0..3.0 against a band of (0.0, 1.0).
    off = [(0.0, 2.0, 1.0, 3.0)]
    check('grow does not drag a far obstacle into the band',
          near(E.free_run(0.0, 1.0, (0.0, 1.0), True, off, grow=0.5), 1.0))

    # STRICTNESS. An obstacle whose band-axis span merely TOUCHES the band is
    # not in the way. This is what excludes the escaping pad itself (its rect
    # starts exactly where the band ends) and its own row-mates, with no
    # self-exclusion bookkeeping -- the property the whole design rests on.
    touch = [(0.0, 1.0, 1.0, 2.0)]      # y 1.0..2.0 against band (0.0, 1.0)
    check('an obstacle touching the band edge is not in the way',
          near(E.free_run(0.0, 1.0, (0.0, 1.0), True, touch), 1.0))
    touch_lo = [(0.0, -1.0, 1.0, 0.0)]  # y -1.0..0.0, touching the far edge
    check('an obstacle touching the far band edge is not in the way',
          near(E.free_run(0.0, 1.0, (0.0, 1.0), True, touch_lo), 1.0))
    # ... but one that genuinely enters the band is.
    inside = [(0.0, 0.5, 1.0, 2.0)]
    check('an obstacle that enters the band is in the way',
          near(E.free_run(0.0, 1.0, (0.0, 1.0), True, inside), 0.0))

    # The vertical orientation is the same arithmetic with the axes swapped:
    # the run is measured along y and the band is on x.
    vert = [(0.0, 0.4, 1.0, 0.6)]
    check('the vertical orientation measures along y',
          near(E.free_run(0.0, 1.0, (0.0, 1.0), False, vert), 0.4))

    # THE EPSILON, on real coordinates rather than invented ones. ulx3s U1's
    # ball A2 parses to a copper box whose x runs 131.48000000000002 ..
    # 131.88, so its span is 0.39999999999997726 and a bare `>= 0.4` is
    # FALSE for a nominally 0.4mm pad. Measured, 385 of that part's 389 pads
    # are sub-nominal this way, so it is the rule and not a corner case: a
    # bare comparison puts the BGA's whole outer ring back in the interior
    # bucket at track 0.4 while answering correctly at track 0.2.
    #
    # Invented coordinates do NOT reproduce it -- 139.08 - 138.68 is
    # 0.4000000000000057, on the other side of the nominal width -- which is
    # exactly why this fixture carries the parser's numbers.
    lo, hi = 131.48000000000002, 131.88
    span = E.free_run(lo, hi, (0.0, 1.0), True, [])
    check('the kernel reports the true subtracted span', near(span, hi - lo))
    check('a real ball span is BELOW its nominal width', span < 0.4,
          'A2 span {!r} -- if this ever stops being true the epsilon is '
          'untested, not unnecessary'.format(span))
    check('and clears the nominal width once the epsilon is applied',
          span >= 0.4 - E.FREE_RUN_EPS)


# ---------------------------------------------------------------- section 2
def the_pad_band_geometry():
    """`pad_band` is `face_band` turned inward: pad edge to box edge."""
    # A 10x10 copper box with a 1x1 pad at (4,4)..(5,5).
    rect = (0.0, 0.0, 10.0, 10.0)
    box = (4.0, 4.0, 5.0, 5.0)

    # North is toward miny (KiCad y grows downward, as `_face_geometry` has
    # it), so the band runs from the box's miny up to the pad's own py0, and
    # the strip is the pad's x span.
    lo, hi, band, horiz = E.pad_band(rect, 'north', box)
    check('north: strip is the pad x span, band is miny..py0',
          (lo, hi, band, horiz) == (4.0, 5.0, (0.0, 4.0), True))

    lo, hi, band, horiz = E.pad_band(rect, 'south', box)
    check('south: band is py1..maxy',
          (lo, hi, band, horiz) == (4.0, 5.0, (5.0, 10.0), True))

    lo, hi, band, horiz = E.pad_band(rect, 'west', box)
    check('west: strip is the pad y span, band is minx..px0',
          (lo, hi, band, horiz) == (4.0, 5.0, (0.0, 4.0), False))

    lo, hi, band, horiz = E.pad_band(rect, 'east', box)
    check('east: band is px1..maxx',
          (lo, hi, band, horiz) == (4.0, 5.0, (5.0, 10.0), False))

    # A pad ON an edge of the box gets a zero-depth band there -- which is
    # what "already on the box edge" means, and the case the caller reads as
    # an unconditional escape.
    edge = (4.0, 0.0, 5.0, 1.0)
    _lo, _hi, band, _h = E.pad_band(rect, 'north', edge)
    check('a pad on the box edge has a zero-depth band',
          near(band[1] - band[0], 0.0))

    # The strip is the PAD's span and nothing else -- not a pitch-wide window.
    # Measured, a window collapses ulx3s U1 to 0 interior pads at clearance
    # 0.09, and `pad_pitch` reads 0.100 on qfn_interior_pads, so a
    # pitch-derived strip is fragile by construction.
    narrow = (4.4, 4.0, 4.6, 5.0)
    lo, hi, _b, _h = E.pad_band(rect, 'north', narrow)
    check('the strip is the pad span, not a pitch-wide window',
          near(hi - lo, 0.2))



#: The modelled fixture. `qfn_interior_pads` exists in the corpus for exactly
#: this question -- an HVQFN-32 whose pads 34-37 (INT1, INT2 and two GND
#: guards) sit 1.075mm inside the part's south copper edge, boxed in by the
#: QFN's UNNETTED south pin row. It is the board a previous attempt at #862
#: broke, so it is the right fixture to build the gate on. Real geometry
#: rather than stubs, because `part_copper_geometry` refuses to model a pad
#: that carries no size and the whole point here is the MODELLED path.
_FIXTURE = 'qfn_interior_pads'
_FIXTURE_REF = 'U1'
_CACHE = {}


def _fixture(clearance=0.2):
    key = round(clearance, 6)
    if key not in _CACHE:
        from kicad_parser import parse_kicad_pcb
        from placement import legality as L
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        pcb = parse_kicad_pcb(os.path.join(root, 'kicad_files',
                                           _FIXTURE + '.kicad_pcb'))
        geom = L.part_copper_geometry(pcb.footprints, clearance)[_FIXTURE_REF]
        _CACHE[key] = (pcb.footprints[_FIXTURE_REF], geom)
    return _CACHE[key]


# ---------------------------------------------------------------- section 3
def the_basis_is_a_pair_or_it_refuses():
    """A half-named basis is a plausible wrong number, so it raises.

    This lives in `assign_faces` rather than in each ledger because that is
    the one function both lane ledgers share: a check placed in either caller
    could be present in one instrument and missing from the other, which is
    the shape of every defect #835, #841, #847, #849 and #850 closed.
    """
    fp, geom = _fixture()

    def raises(**kw):
        try:
            E.assign_faces(fp, geom, lane_mm=0.4, **kw)
        except ValueError as exc:
            return str(exc)
        return None

    msg = raises(clearance=0.2)
    check('a clearance without a track width refuses', msg is not None)
    check('and the refusal says it is a pair', bool(msg) and 'PAIR' in msg,
          str(msg))

    check('a track width without a clearance refuses',
          raises(track_width=0.2) is not None)

    msg = raises(clearance=0.2, track_width=0.0)
    check('a zero-width track refuses', msg is not None)
    check('and the refusal cites what a zero threshold measures',
          bool(msg) and 'qfn_interior_pads' in msg, str(msg))

    check('a negative clearance refuses',
          raises(clearance=-0.1, track_width=0.2) is not None)

    # Naming NEITHER is the documented way to ask for the pre-#862 answer.
    a = E.assign_faces(fp, geom, lane_mm=0.4)
    check('naming neither is not an error',
          a.corridor_source == 'not_measured', a.corridor_source)
    check('and then the union answer IS the box answer',
          list(a.box_faces) == [f for _p, f in a.faces])


# ---------------------------------------------------------------- section 4
def the_degradation_paths_are_declared():
    """Every path that cannot run the corridor SAYS which one it was.

    Four distinct strings rather than one `unknown`, because "nobody asked"
    and "this part has no pad model" are different facts and a reader has to
    be able to tell them apart.
    """
    fp, geom = _fixture()

    a = E.assign_faces(fp, None, lane_mm=0.4, fallback_rect=(0.0, 0.0, 4.0,
                                                             4.0),
                       clearance=0.2, track_width=0.2)
    check('an unmodelled part says so', a.corridor_source == 'unmodelled',
          a.corridor_source)

    a = E.assign_faces(fp, geom._replace(pads={}), lane_mm=0.4,
                       clearance=0.2, track_width=0.2)
    check('a part with no pad boxes says so',
          a.corridor_source == 'no_pad_boxes', a.corridor_source)

    a = E.assign_faces(fp, geom, lane_mm=0.4, clearance=0.2, track_width=0.2)
    check('a modelled part reports the caller as the source',
          a.corridor_source == 'caller', a.corridor_source)
    check('and records the basis it ran at',
          (a.corridor_clearance_mm, a.corridor_track_mm) == (0.2, 0.2))

    # THE COVERAGE DECLARATION, asserted rather than assumed.
    # `tests/test_escape_ledger.py`'s synthetic parts carry no pad sizes, so
    # `part_copper_geometry` returns `modelled=False` with an EMPTY `pads`
    # table and `legality.pad_box` answers None for every pad. Its
    # `interior_pads == 9` oracle is preserved by the degradation path above
    # -- and it witnesses NOTHING about the corridor. Pinned here so that the
    # day those fixtures gain a pad model, a test says so instead of that
    # oracle quietly changing meaning.
    import importlib.util
    from placement import legality as L
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    spec = importlib.util.spec_from_file_location(
        '_tel', os.path.join(root, 'tests', 'test_escape_ledger.py'))
    tel = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tel)
    bare = tel._grid_part(n=5, pitch=0.5, interior=True)
    g = L.part_copper_geometry({'U1': bare}, 0.25).get('U1')
    check('the escape_ledger fixture is unmodelled',
          g is not None and not g.modelled)
    check('... with an empty pad table', g is not None and not g.pads)
    check('... so every pad_box is None',
          g is not None and all(L.pad_box(g, p) is None for p in bare.pads))
    a = E.assign_faces(bare, g, lane_mm=0.45, clearance=0.25, track_width=0.2)
    check('... the corridor never runs there',
          a.corridor_source == 'no_pad_boxes', a.corridor_source)
    check('... and its interior verdict is the box rule, unchanged',
          list(a.box_faces) == [f for _p, f in a.faces])
    check('... which is still 9 interior pads',
          sum(1 for _p, f in a.faces if f is None) == 9,
          str(sum(1 for _p, f in a.faces if f is None)))


# ---------------------------------------------------------------- section 5
def the_corridor_is_only_asked_about_box_interior_pads():
    """Not an optimisation -- a proof, and it is asserted as one.

    When `min(d) <= tol` the argmin direction is itself in the escaping set,
    so filtering the tie block by the corridor cannot move the answer. The
    union therefore only ADDS escapes, and asking the corridor about a pad the
    box half already placed would be dead work that could only change an
    answer by being wrong.
    """
    from placement import legality as L
    fp, geom = _fixture()
    seen = []
    real = E.PadCorridors.clear

    def spy(self, face, pad_box):
        seen.append(pad_box)
        return real(self, face, pad_box)

    E.PadCorridors.clear = spy
    try:
        a = E.assign_faces(fp, geom, lane_mm=0.4,
                           clearance=0.2, track_width=0.2)
    finally:
        E.PadCorridors.clear = real

    asked = set(seen)
    accepted = {L.pad_box(geom, p)
                for p, f in zip(fp.pads, a.box_faces)
                if f is not None and L.pad_box(geom, p) is not None}
    check('the corridor ran at all', len(seen) > 0, str(len(seen)))
    check('and was never asked about a pad the box half accepted',
          not (asked & accepted),
          '{} of {} accepted pads were asked'.format(
              len(asked & accepted), len(accepted)))

    # The union can only ADD escapes, never remove one. Provable, and this is
    # the arm that fails the moment someone makes the corridor a REPLACEMENT
    # for the box test rather than a second sufficient condition.
    grew = [i for i, (_p, f) in enumerate(a.faces)
            if f is None and a.box_faces[i] is not None]
    check('no pad the box half placed became interior', not grew,
          '{} pad(s) went the wrong way'.format(len(grew)))



#: `{(board, ref): {basis: (box interior, union interior)}}` -- the two-sided
#: gate, on the two boards that fail in OPPOSITE directions plus the four
#: refs whose numbers must NOT move.
#:
#: Read the rows, they are the finding:
#:   ulx3s U1   379 -> 308. An LFE5U BGA whose eight UNNETTED alignment marks
#:              set the copper box 0.954mm outside the ball field, so the box
#:              rule calls all 379 netted balls interior and the part reports
#:              demand 0 on every face. 71 recover -- more than the 67-ball
#:              outer ring of the 20x20 lattice, because some second-row balls
#:              escape through sites where the outer row has no ball, which is
#:              the thing a ring-vs-interior rule could not do.
#:   qfn_interior_pads U1   5 -> 5, AND IT IS THE SAME FIVE PADS. The negative
#:              control: pads 34-37 plus one sit behind the QFN's UNNETTED
#:              south pin row and are genuinely enclosed. A previous attempt at
#:              #862 (`_assignment_rect`, reverted in e1f233b4) measured
#:              against the NETTED pads only and took this to 1. Measured, a
#:              netted-only obstacle set still takes it to 0 today.
#:   tigard U3 / glasgow_revC U1 / rp2350 U2 / watchy U1   unchanged, at every
#:              basis in the envelope. Four refs that the rule must not touch.
#:
#: THE BASIS IS PART OF EVERY ROW, because since #862 `interior_pads` is a
#: function of clearance and track width where before it was a function of
#: geometry alone. 0.09/0.10 is where `check_channels` runs the corpus,
#: 0.20/0.20 is `test_850`'s census basis, 0.25/0.30 is the CLI's own default.
#: 0.40/0.40 is not a basis any caller uses, and it is here for a reason a
#: mutation run supplied: WITHOUT it, dropping FREE_RUN_EPS survives this
#: whole gate. The epsilon only bites where a free run lands exactly on the
#: threshold, which on this corpus is clearance 0.05 and 0.40 and nowhere
#: else -- so a gate pinned only at the bases in use cannot see the trap that
#: motivated the constant. At 0.40/0.40 a bare comparison puts 33 of ulx3s
#: U1'''s recovered balls back in the interior bucket (308 -> 341).
BASES = ((0.09, 0.10), (0.20, 0.20), (0.25, 0.30), (0.40, 0.40))
GOLDEN_862 = {
    ('ulx3s', 'U1'):              {b: (379, 308) for b in BASES},
    ('qfn_interior_pads', 'U1'):  {b: (5, 5) for b in BASES},
    ('tigard', 'U3'):             {b: (18, 18) for b in BASES},
    ('glasgow_revC', 'U1'):       {b: (27, 27) for b in BASES},
    ('rp2350_fpga_eensy_prePlane', 'U2'): {b: (25, 25) for b in BASES},
    ('watchy', 'U1'):             {b: (0, 0) for b in BASES},
}


def _interior(board, ref, clr, trk):
    """(box interior, union interior, the union's interior pad numbers)."""
    from kicad_parser import parse_kicad_pcb
    from placement import legality as L
    key = (board, round(clr, 6))
    if key not in _CACHE:
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        pcb = parse_kicad_pcb(os.path.join(root, 'kicad_files',
                                           board + '.kicad_pcb'))
        _CACHE[key] = (pcb, L.part_copper_geometry(pcb.footprints, clr))
    pcb, geoms = _CACHE[key]
    fp = pcb.footprints[ref]
    a = E.assign_faces(fp, geoms[ref], lane_mm=trk + clr,
                       clearance=clr, track_width=trk)
    box = un = 0
    names = []
    for i, (pad, face) in enumerate(a.faces):
        if not getattr(pad, 'net_id', 0):
            continue
        if a.box_faces[i] is None:
            box += 1
        if face is None:
            un += 1
            names.append(pad.pad_number)
    return box, un, tuple(sorted(names))


# ---------------------------------------------------------------- section 6
def the_two_sided_gate():
    """ulx3s must fall a long way; qfn_interior_pads must not move at all.

    They fail in OPPOSITE directions, which is why the issue insists on both:
    a rule generous enough to free the BGA's outer ring is generous enough to
    free four pads that are genuinely walled in behind an unnetted pin row.
    """
    for (board, ref), rows in sorted(GOLDEN_862.items()):
        for (clr, trk), (want_box, want_un) in sorted(rows.items()):
            box, un, _n = _interior(board, ref, clr, trk)
            check('{}:{} at {}/{} interior {} -> {}'
                  .format(board, ref, clr, trk, want_box, want_un),
                  (box, un) == (want_box, want_un),
                  'measured {} -> {}'.format(box, un))

    # A COUNT IS NOT A SET. `qfn_interior_pads` reads 5 before and 5 after,
    # and that is only a negative control if they are the SAME five pads --
    # the reverted experiment's census printed "5 -> 1" as a win without ever
    # asking which four had moved.
    from placement import legality as L
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for clr, trk in BASES:
        pcb, geoms = _CACHE[('qfn_interior_pads', round(clr, 6))]
        fp = pcb.footprints['U1']
        a = E.assign_faces(fp, geoms['U1'], lane_mm=trk + clr,
                           clearance=clr, track_width=trk)
        before = tuple(sorted(p.pad_number for i, (p, _f)
                              in enumerate(a.faces)
                              if getattr(p, 'net_id', 0)
                              and a.box_faces[i] is None))
        after = tuple(sorted(p.pad_number for p, f in a.faces
                             if getattr(p, 'net_id', 0) and f is None))
        check('qfn_interior_pads:U1 keeps the SAME pads at {}/{}'
              .format(clr, trk), before == after,
              '{} -> {}'.format(before, after))

    # And the recovery on ulx3s is a real rescue, not a re-labelling: the 71
    # pads the corridor frees must all have been box-interior.
    box, un, _n = _interior('ulx3s', 'U1', 0.20, 0.20)
    check('ulx3s:U1 recovers 71 balls', box - un == 71,
          'recovered {}'.format(box - un))


# ---------------------------------------------------------------- section 7
def the_band_test_is_open_not_closed():
    """The parameter the prose never named, and it decides everything.

    "An obstacle counts when its band-axis span INTERSECTS the band" can be
    read open (positive overlap) or closed (touching counts). Open is right,
    and the witness is real geometry: `tigard` U3's exposed pad is drawn as
    sub-rects sitting inside a full-size EP rect, and a sub-pad shares the
    edge y=58.225 with that rect. The EP lies entirely SOUTH of the sub-pad,
    so it cannot block a NORTHWARD escape -- but a closed test counts it as
    blocking a band it overlaps by exactly zero.

    The stake is not a few pads. Under the closed reading, an obstacle list
    that includes the pad's own rect -- which this one deliberately does,
    because strictness makes self-exclusion free -- blocks every pad against
    itself, and the union collapses to the box rule EXACTLY: 2054 interior at
    every basis, every witness ref still reading 308 and 5, and the rule
    doing nothing at all while every corpus golden still passes.
    """
    # An obstacle whose span touches the band's near edge, and one that
    # touches its far edge, are both entirely outside it.
    check('touching the near band edge does not block',
          near(E.free_run(0.0, 1.0, (0.0, 1.0), True,
                          [(0.0, 1.0, 1.0, 2.0)]), 1.0))
    check('touching the far band edge does not block',
          near(E.free_run(0.0, 1.0, (0.0, 1.0), True,
                          [(0.0, -1.0, 1.0, 0.0)]), 1.0))
    # ... while any positive overlap does.
    check('a hair of overlap does block',
          near(E.free_run(0.0, 1.0, (0.0, 1.0), True,
                          [(0.0, 0.999999, 1.0, 2.0)]), 0.0))

    # THE SELF-CHARGE. A pad's own rect is in the obstacle list. Its band
    # starts at its own edge, so under the open test it contributes nothing
    # -- and if it ever did, it would cover its own strip end to end and
    # NOTHING would escape anywhere.
    pad = (4.0, 4.0, 5.0, 5.0)
    lo, hi, band, horiz = E.pad_band((0.0, 0.0, 10.0, 10.0), 'north', pad)
    check('a pad does not block its own corridor',
          near(E.free_run(lo, hi, band, horiz, [pad]), 1.0))
    check('... in every direction',
          all(near(E.free_run(*E.pad_band((0.0, 0.0, 10.0, 10.0), f, pad),
                              obstacles=[pad]), 1.0)
              for f in E.FACES))


def main():
    for fn in (the_free_run_kernel, the_pad_band_geometry,
               the_basis_is_a_pair_or_it_refuses,
               the_degradation_paths_are_declared,
               the_corridor_is_only_asked_about_box_interior_pads,
               the_two_sided_gate,
               the_band_test_is_open_not_closed):
        fn()
    for f in FAILURES:
        print('FAIL: {}'.format(f))
    if FAILURES:
        print('\n{} FAILED of {} checks in {} sections'
              .format(len(FAILURES), len(FAILURES) + len(PASSED), SECTIONS))
        return 1
    print('OK -- {} checks in {} sections'.format(len(PASSED), SECTIONS))
    return 0


if __name__ == '__main__':
    sys.exit(main())

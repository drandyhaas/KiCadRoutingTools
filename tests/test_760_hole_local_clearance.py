#!/usr/bin/env python3
"""#760: copper-to-hole in pcb_modification must carry the hole pad's own
`local_clearance`.

`check_drc` grades a track against an NPTH drill at `max(npth_clr, lc)` where
`lc` is the HOLE PAD's `(clearance ...)` override (check_drc.py, the `holes`
list; #326/#505). Two sites in `pcb_modification` resolved the same geometry
without that term:

    _seg_worst_offender        the shortfall ranking that drives the shift
    nudge_grazing_microshift   the graze detector + the shift acceptance gate

Both already read the board's declared `min_hole_clearance` (#617), so `lc`
was the one remaining term -- and it is the one that is PER HOLE, not board
wide. That is why it is folded into the DISTANCE (each hole's excess over the
flat floor is subtracted, the trick #436 uses for foreign net-class clearance)
rather than into the scalar floor: raising `npth_clr` for one overriding hole
would raise it for every hole on the board.

WHAT THIS CHANGES AND WHAT IT DOES NOT. #617 split these five sites into two
groups and this change keeps that split exactly:

  CHANGED   _seg_worst_offender          detector, low risk by its own comment
            nudge_grazing_microshift     detector + acceptance -- a TRADE

  UNCHANGED close_soft_joints            flat on measured grounds;
            _connector_clear             `_seg_foreign_hole_dist` defaults
            nudge_grazing_octolinear     `base_clearance=None`, so all four
            smooth_octolinear_chains     are bit-identical here.

#617's own list has THREE unchanged sites, not four: `smooth_octolinear_chains`
(the #536 route-smoothing pass) carries a copper-to-hole floor of its own at
`pcb_modification.py` ~:3903 and is enumerated nowhere. It is left flat here
for the same reason as the octolinear re-bend -- but a later pass at
"finishing the job" needs the complete list, so it is written down.

The octolinear arm below is the CHANGE DETECTOR for that: its `clears()` block
is BYTE-IDENTICAL to the micro-shift's (pcb_modification.py ~:3687 vs ~:4735),
so a future text-matched edit would silently take it along.

ONE OF #617's REASONS DOES NOT SURVIVE THE LARGER REQUIREMENT, and the
octolinear arm measures it rather than repeating it. #617 kept that site flat
partly because "the net-to-net OVERLAP it repairs is a bigger defect than the
hole shortfall it leaves" -- 0.1000mm overlap vs a 0.0300mm shortfall at its
declared 0.25. At a 0.40 pad override the same fixture gives 0.1000 vs
0.1800, so the comparison INVERTS above a 0.32mm requirement. The site is
still left flat, on the reason that does survive: its re-bend is
all-or-nothing, so gating it buys no smaller violation, it abandons the
repair. The numbers are pinned so a future pass re-measures instead of
quoting the inverted half.

THE TRADE, inherited from #617 in full: the same term sits in the candidate-
acceptance `clears()`, so on a board carrying an override a copper-graze repair
whose only escape direction points at that hole is REFUSED when every candidate
would land inside the override band. `test_the_trade_is_a_refusal_not_a_repair`
pins that, because it is a trade and not a free win.

CORPUS SCOPE, measured over the 22 boards `run_utils.corpus_boards()` returns
(37 NPTH copper-less drilled pads):

    ulx3s.kicad_pcb    2 pads (AUDIO1, drill 1.700, local_clearance 0.400)
                       -> BINDING: 0.400 > the 0.20 NPTH_TO_TRACK_CLEARANCE
    watchy.kicad_pcb   8 pads, all local_clearance 0.100
                       -> NUMERICALLY INERT: 0.100 is BELOW the fab floor, so
                          max(0.20, 0.10) is still 0.20. It looks like a
                          fixture for this and is not -- `test_an_override
                          _below_the_fab_floor_changes_nothing` is that case.
    the other 7 boards carrying NPTH pads declare no override at all.

So exactly one corpus board can move, and `test_the_corpus_anchor` reads its
two pads off the real file rather than restating the numbers.

AND ON THAT BOARD IT IS STILL A NULL, measured rather than assumed. ulx3s was
routed twice -- flag off (main) and flag on -- over the same 28 nets, chosen
from the FIXTURE (every net with a pad within 12mm of AUDIO1's two holes) so
the scope is identical on both arms and is not "the nets that moved". Fresh
output path per arm, no `.kicad_pro` carryover; ulx3s has no sibling project
at all, so `resolve_hole_clearance` contributes nothing and the pad override
is the ONLY term that can differ:

    metric        before    after   delta
    DRC              13       13      0     (VIA-SEGMENT 8, PAD-SEGMENT 5 both)
    conn issues     261      261      0     (301 unrequested nets, both arms)
    broken            0        0      0
    segments       1952     1952      0
    vias            109      109      0
    closest copper-to-hole gap: 0.4565mm on BOTH arms

The reason is the last row: the nearest copper misses the 0.40 band by
0.0565mm, so the term never fires. That is a NARROW miss, not a comfortable
one -- a different net order or a tighter route would put copper in the band,
which is why the synthetic arms above carry the burden of proof and this is
recorded as "inert on today's corpus", not as "inert".

    python3 tests/test_760_hole_local_clearance.py
"""
import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import obstacle_map
from kicad_parser import BoardInfo
from single_ended_routing import (_seg_foreign_hole_dist, _seg_foreign_pad_dist,
                                  _seg_foreign_seg_dist, _seg_foreign_via_dist)
from pcb_modification import (_seg_worst_offender, nudge_grazing_microshift,
                              nudge_grazing_octolinear)
from routing_defaults import NPTH_TO_TRACK_CLEARANCE
from synth import make_pad, make_pcb, make_seg

HOLE_X, HOLE_Y = 10.0, 10.0
DRILL = 1.0                 # -> hole wall at radius 0.5
W = 0.2                     # track width -> half 0.1
BAND = 0.22                 # copper-to-hole-WALL gap under test
Y = HOLE_Y + DRILL / 2.0 + BAND + W / 2.0       # 10.82: copper centreline
CLEARANCE = 0.15            # routing clearance, below the NPTH fab floor
OVERRIDE = 0.40             # the hole pad's own clearance, ABOVE the floor
UNDER_FLOOR = 0.10          # watchy's value: below the floor, so inert
SHORTFALL = OVERRIDE - BAND                     # 0.18mm

# A foreign-net pad above the copper: far enough that the 0.18mm shift has a
# legal landing, near enough that a shift that ignored it would be caught.
FOREIGN_Y = 11.70
FOREIGN_SIZE = 0.30


def _board(tmp, name):
    """A board file with a sibling project declaring NOTHING, so the flat
    `max(clearance, NPTH_TO_TRACK_CLEARANCE)` = 0.20 floor is in force and the
    only thing that can move a decision is the pad override under test."""
    d = os.path.join(tmp, name)
    os.makedirs(d, exist_ok=True)
    pcb = os.path.join(d, 'b.kicad_pcb')
    with open(pcb, 'w', encoding='utf-8') as f:
        f.write('(kicad_pcb (version 20240108))\n')
    with open(os.path.join(d, 'b.kicad_pro'), 'w', encoding='utf-8') as f:
        json.dump({'board': {'design_settings': {'rules': {}}}}, f)
    return pcb


def _npth(lc, x=HOLE_X, y=HOLE_Y, drill=DRILL, layers=('F.Mask', 'B.Mask')):
    """An NPTH mounting hole carrying `lc` as its own clearance override.

    MASK-ONLY, and that is load-bearing, not tidiness. `_foreign_pad_arrays`
    admits any pad whose layers contain the queried layer OR `*.Cu`, and it
    carries each pad's `local_clearance` in its own `plc` term -- so an NPTH
    written `('*.Cu', '*.Mask')` enters the COPPER channel as a 1.0mm pad with
    a 0.4mm override, and every arm below then passes for a reason that has
    nothing to do with the hole channel under test. Measured: with `*.Cu` the
    mutation battery killed 1 row of 6; mask-only it kills 6 of 6."""
    return make_pad(net_id=0, x=x, y=y, ref='AUDIO1', num='H1',
                    size_x=drill, size_y=drill, shape='circle',
                    layers=list(layers), drill=drill,
                    pad_type='np_thru_hole', local_clearance=lc)


def _pcb(path, lc, segs, extra_pads=None, bounds=(0.0, 0.0, 20.0, 20.0),
         hole=None):
    bi = BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=bounds)
    pads = {0: [hole if hole is not None else _npth(lc)]}
    if extra_pads:
        pads.update(extra_pads)
    return make_pcb(board_info=bi, segments=list(segs), pads_by_net=pads,
                    source_path=path, zones=[])


def _copper_gap(pcb, net_id=1):
    """Closest approach to FOREIGN COPPER -- the defect these passes repair,
    and the one they must never make worse."""
    out = []
    for s in pcb.segments:
        if s.net_id != net_id:
            continue
        a = (s.start_x, s.start_y, s.end_x, s.end_y)
        out.append(min(_seg_foreign_pad_dist(pcb, s.net_id, *a, s.layer),
                       _seg_foreign_seg_dist(pcb, s.net_id, *a, s.layer),
                       _seg_foreign_via_dist(pcb, s.net_id, *a, s.layer))
                   - s.width / 2.0)
    return min(out)


def _hole_gap(pcb, net_id=1):
    """Raw copper-to-hole-WALL gap, with NO override folded in -- the geometry,
    not the requirement. Deliberately calls the default (base_clearance=None)
    arm, so this measurement cannot move with the fix."""
    return min(_seg_foreign_hole_dist(pcb, s.net_id, s.start_x, s.start_y,
                                      s.end_x, s.end_y) - s.width / 2.0
               for s in pcb.segments if s.net_id == net_id)


def run():
    fails = []

    def check(name, cond, note=''):
        print(('PASS' if cond else 'FAIL') + f': {name}' +
              (f'  [{note}]' if note else ''))
        if not cond:
            fails.append(name)

    with tempfile.TemporaryDirectory() as tmp:
        path = _board(tmp, 'silent')
        obstacle_map._HOLE_CLR_CACHE.clear()

        print(f'NPTH fab floor {NPTH_TO_TRACK_CLEARANCE}mm; routing clearance '
              f'{CLEARANCE}mm; board declares nothing.')
        print(f'  hole ({HOLE_X},{HOLE_Y}) drill {DRILL}, pad override '
              f'{OVERRIDE}mm; copper {BAND}mm off the wall '
              f'-> {SHORTFALL:.2f}mm short of what check_drc requires.')
        print()

        _detector(check, path)
        _detector_controls(check, path)
        _microshift(check, path)
        _trade(check, path)
        _acceptance_gate(check, path)
        _octolinear_stays_flat(check, path)
        _corpus_anchor(check)

    print()
    if fails:
        print(f'{len(fails)} FAILURE(S): {fails}')
        return 1
    print('all checks passed')
    return 0


# === CHANGED site 1: _seg_worst_offender ===================================
def _detector(check, path):
    print('CHANGED site 1: _seg_worst_offender -- the shortfall ranking')
    s = make_seg(8.0, Y, 12.0, Y, width=W, net_id=1)
    got = _seg_worst_offender(_pcb(path, OVERRIDE, [s]), 1, s, CLEARANCE)
    print(f'        override {OVERRIDE}: '
          + (f'shortfall {got[0]:.4f}mm, away=({got[2]:.3f},{got[3]:.3f})'
             if got else 'no offender'))
    check("the hole's own override makes the 0.22 approach a ranked offender",
          got is not None and abs(got[0] - SHORTFALL) < 1e-3,
          f'want {SHORTFALL:.4f}')
    # The shift the ranking hands the micro-shift must point AWAY from the
    # hole: a shortfall with the wrong sign would move copper deeper in.
    check('and the away-direction points away from the hole (+y)',
          got is not None and got[3] > 0.99)
    print()


def _detector_controls(check, path):
    print('  controls -- the term must be raise-only and per-hole')
    s = make_seg(8.0, Y, 12.0, Y, width=W, net_id=1)
    none = _seg_worst_offender(_pcb(path, 0.0, [s]), 1, s, CLEARANCE)
    check('no override -> still no offender (raise-only)', none is None,
          'this is the pre-fix behaviour on 35 of the 37 corpus NPTH pads')

    s2 = make_seg(8.0, Y, 12.0, Y, width=W, net_id=1)
    low = _seg_worst_offender(_pcb(path, UNDER_FLOOR, [s2]), 1, s2, CLEARANCE)
    check(f'an override BELOW the fab floor ({UNDER_FLOOR}) changes nothing',
          low is None, "watchy's 8 pads -- max(0.20, 0.10) is still 0.20")

    # Per-hole, not board-wide. The near hole carries NOTHING and the copper
    # is 0.22 off it -- legal at the flat floor. A second hole 5mm away DOES
    # carry the override. If the override were folded into the scalar floor
    # instead of into each hole's distance, the near hole would inherit it and
    # this copper would be reported 0.18mm short. Written this way round on
    # purpose: the obvious spelling (override near, plain hole far) passes
    # either way, because the far hole is never the closest approach.
    s3 = make_seg(8.0, Y, 12.0, Y, width=W, net_id=1)
    pcb = _pcb(path, 0.0, [s3])
    pcb.pads_by_net[0].append(_npth(OVERRIDE, x=HOLE_X, y=HOLE_Y + 5.0))
    leak = _seg_worst_offender(pcb, 1, s3, CLEARANCE)
    print(f'        near hole no override, far hole {OVERRIDE}: '
          + (f'shortfall {leak[0]:.4f}mm' if leak else 'no offender'))
    check("one hole's override does not raise the floor for another hole",
          leak is None,
          'a board-wide max() here would report 0.1800mm short')

    # A WIDE override has to be seeable, not just priceable. The offender scan
    # windows its samples at R = max(required, hole_required) + 0.2, sized from
    # the flat floor -- so an override wider than that window would have
    # excluded the very hole it applies to, and the term would have read as
    # inert for the worst cases rather than the mildest. R now carries the
    # largest excess on the board.
    wide, gap = 1.2, 0.90
    y_wide = HOLE_Y + DRILL / 2.0 + gap + W / 2.0
    s4 = make_seg(8.0, y_wide, 12.0, y_wide, width=W, net_id=1)
    far_band = _seg_worst_offender(_pcb(path, wide, [s4]), 1, s4, CLEARANCE)
    print(f'        override {wide} with the copper {gap} away: '
          + (f'shortfall {far_band[0]:.4f}mm' if far_band else 'no offender'))
    check('an override wider than the old sampling window is still seen',
          far_band is not None and abs(far_band[0] - (wide - gap)) < 1e-3,
          f'want {wide - gap:.4f}; the old R was '
          f'{max(CLEARANCE, NPTH_TO_TRACK_CLEARANCE) + W / 2.0 + 0.2:.2f}mm')
    print()


# === CHANGED site 2: nudge_grazing_microshift ==============================
def _microshift(check, path):
    """The pass that MOVES copper: it must see the override band, actually
    clear it, and not trade away the foreign-copper clearance it already had."""
    print('CHANGED site 2: nudge_grazing_microshift -- detector + acceptance')
    foreign = {2: [make_pad(net_id=2, x=10.0, y=FOREIGN_Y, ref='R1', num='1',
                            size_x=FOREIGN_SIZE, size_y=FOREIGN_SIZE,
                            shape='rect', layers=['F.Cu'], pad_type='smd')]}
    for lc, label, want in ((OVERRIDE, f'override {OVERRIDE}', True),
                            (0.0, 'no override', False),
                            (UNDER_FLOOR, f'override {UNDER_FLOOR}', False)):
        s = make_seg(8.0, Y, 12.0, Y, width=W, net_id=1)
        pcb = _pcb(path, lc, [s], foreign)
        g0 = _hole_gap(pcb)
        res = [{'new_segments': [s], 'new_vias': []}]
        changed, nets, _rm, _add = nudge_grazing_microshift(
            res, pcb, {1}, clearance=CLEARANCE, max_shift=0.25)
        g1 = _hole_gap(pcb)
        print(f'        {label}: segs_changed={changed} nets={nets}; '
              f'copper-to-hole {g0:.4f} -> {g1:.4f}')
        check(f'micro-shift {"fires" if want else "stays inert"} '
              f'when {label}', (nets == 1) is want)
        if want:
            check('the shifted copper clears the OVERRIDE, not just the floor',
                  g1 >= OVERRIDE - 1e-4, f'{g1:.4f} >= {OVERRIDE}')
        else:
            check('untouched copper keeps its exact geometry',
                  abs(g1 - BAND) < 1e-9)
    print()


def _trade(check, path):
    """The raised requirement also sits in the candidate-acceptance clears(),
    so a repair whose only escape direction points at the overriding hole is
    REFUSED rather than made. #617's fixture, switched on the pad override: a
    net-1 track grazing a foreign pad from below, with a mask-only NPTH just
    beneath it, so the only clearing shift direction is toward the hole.

    BOTH arms are pinned. Without the override the SAME repair proceeds -- that
    is what makes this a trade and not a free win, and it is why the arm is not
    written as "nothing happened either way"."""
    print('CHANGED site 2b: the acceptance TRADE, both arms pinned')
    pads = {2: [make_pad(net_id=2, x=2.0, y=0.34, ref='R1', num='1',
                         size_x=0.3, size_y=0.3, shape='rect',
                         layers=['F.Cu'], pad_type='smd')]}
    for lc, label, repaired in ((OVERRIDE, f'override {OVERRIDE}', False),
                                (0.0, 'no override', True)):
        s = make_seg(0.0, 0.0, 4.0, 0.0, width=0.2, net_id=1)
        pcb = _pcb(path, lc, [s], pads, bounds=(-5.0, -5.0, 15.0, 15.0),
                   hole=_npth(lc, x=2.0, y=-0.86, drill=1.0,
                              layers=('F.Mask', 'B.Mask')))
        cop0, hole0 = _copper_gap(pcb), _hole_gap(pcb)
        _c, nets, _rm, _add = nudge_grazing_microshift(
            [], pcb, clearance=0.1)
        cop1, hole1 = _copper_gap(pcb), _hole_gap(pcb)
        print(f'        {label}: nets={nets}; foreign-copper '
              f'{cop0:+.4f} -> {cop1:+.4f}; copper-to-hole '
              f'{hole0:.4f} -> {hole1:.4f}')
        if repaired:
            check('no override: the copper-graze repair PROCEEDS (the flat '
                  'floor permits the shift toward the hole)',
                  nets == 1 and cop1 > cop0)
            check('and stays legal at the flat NPTH floor',
                  hole1 >= NPTH_TO_TRACK_CLEARANCE - 1e-4)
        else:
            check('with the override: the SAME repair is REFUSED rather than '
                  'moved into the override band (the deliberate trade)',
                  nets == 0 and abs(cop1 - cop0) < 1e-12)
            check('and the incumbent copper-to-hole gap is untouched',
                  abs(hole1 - hole0) < 1e-12)
    print()



def _acceptance_gate(check, path):
    """The acceptance gate on its OWN, separated from the ranking.

    In `_trade` above the refusal is decided by the RANKING (the hole is the
    worst offender there, so the shift already points away from it) -- measured:
    reverting `clears()` alone leaves that arm green. This fixture puts the gate
    in sole charge. Numbers, with clearance 0.15 and the hole override 0.40:

        foreign pad 0.05 off the track   -> pad shortfall 0.10  <- ranked worst
        overriding hole 0.35 off it      -> hole shortfall 0.05

    so the ranking picks the PAD and shifts 0.10mm toward the hole, landing
    0.25mm off it: legal at the flat 0.20 floor, inside the 0.40 override. The
    only thing that can refuse that landing is the term in `clears()`."""
    print('CHANGED site 2c: the acceptance gate, in sole charge of the refusal')
    clr = 0.15
    pads = {2: [make_pad(net_id=2, x=2.0, y=0.30, ref='R1', num='1',
                         size_x=0.3, size_y=0.3, shape='rect',
                         layers=['F.Cu'], pad_type='smd')]}
    for lc, label, repaired in ((OVERRIDE, f'override {OVERRIDE}', False),
                                (0.0, 'no override', True)):
        s = make_seg(0.0, 0.0, 4.0, 0.0, width=0.2, net_id=1)
        pcb = _pcb(path, lc, [s], pads, bounds=(-5.0, -5.0, 15.0, 15.0),
                   hole=_npth(lc, x=2.0, y=-0.95, drill=1.0))
        cop0, hole0 = _copper_gap(pcb), _hole_gap(pcb)
        # max_shift must clear the 0.10mm the ranking asks for -- at the
        # 0.025 default NEITHER arm moves, and the refusal would be the shift
        # budget rather than the gate under test.
        _c, nets, _rm, _add = nudge_grazing_microshift(
            [], pcb, clearance=clr, max_shift=0.15)
        cop1, hole1 = _copper_gap(pcb), _hole_gap(pcb)
        print(f'        {label}: nets={nets}; foreign-copper '
              f'{cop0:+.4f} -> {cop1:+.4f}; copper-to-hole '
              f'{hole0:.4f} -> {hole1:.4f}')
        if repaired:
            check('no override: the pad graze is repaired by shifting toward '
                  'the hole, landing legally at the flat floor',
                  nets == 1 and cop1 > cop0
                  and hole1 >= NPTH_TO_TRACK_CLEARANCE - 1e-4
                  and hole1 < OVERRIDE)
        else:
            check('with the override the acceptance gate REFUSES that same '
                  'landing (the gate alone decides -- the ranking still '
                  'points at the pad)',
                  nets == 0 and abs(cop1 - cop0) < 1e-12
                  and abs(hole1 - hole0) < 1e-12)
    print()


# === UNCHANGED site: nudge_grazing_octolinear ==============================
def _octolinear_stays_flat(check, path):
    """CHANGE DETECTOR, and the sharpest of them. This site's clears() block is
    BYTE-IDENTICAL to the micro-shift's (~:3687 vs ~:4735), so a text-matched
    edit takes it along by accident -- match by function, not by text.

    #617's fixture, switched on the override: a net-1 jog A(0,0) -> apex(1,0.5)
    -> B(2,0) whose apex OVERLAPS a foreign pad by 0.1mm. The only clearing
    octolinear bend is the direct A-B line, which runs 0.22mm off a mask-only
    NPTH at (1,-0.62). Gating the all-or-nothing re-bend on the override would
    not route around the hole -- it would leave the net-to-net copper overlap
    exactly where it was."""
    print('UNCHANGED site: nudge_grazing_octolinear -- still flat')
    got = {}
    for lc, label in ((OVERRIDE, f'override {OVERRIDE}'), (0.0, 'no override')):
        segs = [make_seg(0.0, 0.0, 1.0, 0.5, width=W, net_id=1),
                make_seg(1.0, 0.5, 2.0, 0.0, width=W, net_id=1)]
        pads = {2: [make_pad(net_id=2, x=1.0, y=0.6, ref='R1', num='1',
                             size_x=0.3, size_y=0.3, shape='rect',
                             layers=['F.Cu'], pad_type='smd')]}
        pcb = _pcb(path, lc, segs, pads, bounds=(-5.0, -5.0, 15.0, 15.0),
                   hole=_npth(lc, x=1.0, y=-0.62, drill=0.6,
                              layers=('F.Mask', 'B.Mask')))
        pad0 = _copper_gap(pcb)
        _c, nets, _rm, _add = nudge_grazing_octolinear(
            [], pcb, clearance=0.1)
        got[label] = (nets, pad0, _copper_gap(pcb), _hole_gap(pcb))
        print(f'        {label}: nets re-bent={nets}; foreign-pad '
              f'{pad0:+.4f} -> {got[label][2]:+.4f} '
              f'(negative = net-to-net OVERLAP); copper-to-hole '
              f'{got[label][3]:.4f}')
    ov, base = got[f'override {OVERRIDE}'], got['no override']
    check('the re-bend FIRES (an inert fixture would detect no change at all)',
          base[0] == 1 and base[1] < 0.0 and base[2] > 0.0)
    check('and decides identically with the override present -- this site is '
          'deliberately flat (#617)',
          ov[0] == base[0] and abs(ov[2] - base[2]) < 1e-12
          and abs(ov[3] - base[3]) < 1e-12)
    # MEASURED, and it does NOT go the way #617's rationale did -- recorded
    # here rather than asserted away. #617 justified leaving this site flat by
    # "the overlap it repairs is the bigger defect": true at its declared 0.25
    # (0.1000 overlap vs a 0.0300 shortfall), FALSE at a 0.40 override
    # (0.1000 vs 0.1800). The balance inverts once the requirement exceeds
    # gap + overlap = 0.22 + 0.10 = 0.32. The site still stays flat, because
    # the alternative here is not a smaller violation but NO repair at all --
    # but the justification is fixture-dependent and this pins the numbers so
    # a future pass at the octolinear site re-measures instead of quoting it.
    inversion = ov[3] + -ov[1]
    print(f'        MEASURED: re-bend repairs a {-ov[1]:.4f}mm overlap and '
          f'leaves a {OVERRIDE - ov[3]:.4f}mm hole shortfall; the '
          f'"bigger defect" argument inverts above a {inversion:.4f}mm '
          f'requirement')
    check('the measured trade-off has not drifted (overlap 0.1000, '
          'post-re-bend hole gap 0.2200)',
          abs(-ov[1] - 0.1000) < 1e-4 and abs(ov[3] - 0.2200) < 1e-4)
    print()


# === the one corpus board that can move ====================================
def _corpus_anchor(check):
    """Read ulx3s's override off the real file rather than restating it, and
    show the term actually reaches the shared distance function."""
    print('CORPUS anchor: ulx3s AUDIO1 (the only binding override on 22 boards)')
    try:
        from run_utils import corpus_boards
        from kicad_parser import parse_kicad_pcb
        from check_drc import _pad_has_no_copper
    except Exception as exc:                                  # pragma: no cover
        print(f'        SKIP: {exc}')
        return
    board = next((b for b in corpus_boards()
                  if os.path.basename(b) == 'ulx3s.kicad_pcb'), None)
    if board is None:
        print('        SKIP: ulx3s not in the corpus checkout')
        return
    pcb = parse_kicad_pcb(str(board))
    binding = [(p.component_ref, p.drill, p.local_clearance)
               for pads in pcb.pads_by_net.values() for p in pads
               if (getattr(p, 'drill', 0) or 0) > 0 and _pad_has_no_copper(p)
               and (getattr(p, 'local_clearance', 0.0) or 0.0)
               > NPTH_TO_TRACK_CLEARANCE]
    print(f'        binding NPTH overrides: {binding}')
    check('ulx3s still carries the 2 AUDIO1 overrides this fix is for',
          len(binding) == 2 and all(r[0] == 'AUDIO1' and abs(r[2] - 0.4) < 1e-9
                                    for r in binding),
          'if this fails the corpus moved, not the code')

    # The term reaches _seg_foreign_hole_dist: measuring a segment beside one
    # of those holes with base_clearance set must cost exactly the excess.
    ref, drill, lc = binding[0]
    hole = next(p for pads in pcb.pads_by_net.values() for p in pads
                if p.component_ref == ref and (getattr(p, 'drill', 0) or 0) > 0
                and _pad_has_no_copper(p))
    x0, y0 = hole.global_x, hole.global_y
    probe = (x0 + 3.0, y0 - 5.0, x0 + 3.0, y0 + 5.0)   # clear of the hole
    flat = _seg_foreign_hole_dist(pcb, -1, *probe)
    with_lc = _seg_foreign_hole_dist(pcb, -1, *probe,
                                     base_clearance=NPTH_TO_TRACK_CLEARANCE)
    print(f'        probe beside {ref}: flat {flat:.4f} -> priced '
          f'{with_lc:.4f} (excess {flat - with_lc:.4f})')
    check('the per-hole excess reaches the shared distance function',
          abs((flat - with_lc) - (lc - NPTH_TO_TRACK_CLEARANCE)) < 1e-6,
          f'want {lc - NPTH_TO_TRACK_CLEARANCE:.4f}')
    print()


if __name__ == '__main__':
    sys.exit(run())

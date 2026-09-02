#!/usr/bin/env python3
"""#834: the PAIR_TEST_CAP branch stops pricing cross-side pads as shared.

`LegalityContext.pair_shortfall` grades a pair one of two ways, chosen by
`pa.n_pads * pb.n_pads > PAIR_TEST_CAP`. The per-pad sweep filters pad pairs
on `_sides_interact`; the over-cap branch fell back to whole-part `extent()`
boxes, which span BOTH faces. So a part on B.Cu was charged a full extent
shortfall against a BGA on F.Cu -- and only when the pad-pair product crossed
4096, while `grade_pad_legality` (no cap, always side-filtered) called the same
pose clean.

The gap this file closes: **nothing in the suite put a cross-side pair over the
cap.** `tests/test_761_legality_npth_keepout.py` straddles the cap with F.Cu
pads only, and `tests/test_placement_pad_legality.py`'s one cross-side arm uses
1x1 pads, so it never leaves the per-pad branch. Both halves were covered and
their intersection was not.

Pinned here:

  1. `pad_sides` is EXACT with respect to `_sides_interact`, not merely
     conservative -- disjoint iff the per-pad sweep would find zero
     interacting pairs. That equivalence is what licenses skipping the sweep.
  2. A cross-side pair gets the SAME verdict either side of the cap. The
     invariant `tests/test_761...:648` states -- "the answer depended on the
     PAD-PAIR PRODUCT, which is a performance switch and must never change a
     verdict" -- with the side dimension it did not cover.
  3. `stack` and `pad_overlap` are False for a cross-side pair whose EXTENTS
     overlap, above the cap as below it.
  4. The HOLE channel still fires cross-side. A drill pierces both faces, so
     the disjoint-sides early return may not become a blanket zero.
  5. A THT pad (`pside is None`) interacts with both faces, so a drilled part
     is never skipped.
  6. The per-side extent is REACHED, by a part with SMD pads on both faces
     above the cap. No tracked board has one, so without this arm that code
     would be unreachable and deletable without a red.
  7. On the real ulx3s: the issue's own verification, plus a same-side
     over-cap pair that must not move.

No board is routed and no file is written; the real-board arms parse one
in-repo board.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (ROOT, os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_placer'),
           os.path.join(ROOT, 'py_tools'), os.path.join(ROOT, 'tests')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from placement import legality as L                           # noqa: E402
from placement.legality import (LegalityContext, PairShortfall,  # noqa: E402
                                PartPads, ZERO_SHORTFALL,
                                _sides_interact, build_part_pads)
from test_placement_pad_legality import FakeFP, FakePad        # noqa: E402

RUN_ALL_FAST_OK = True

ULX3S = os.path.join(ROOT, 'kicad_files', 'ulx3s.kicad_pcb')


def _grid(ref, n, x0, y0, layers=('F.Cu',), pitch=1.0, size=0.4, net_base=1,
          drill=0.0, pad_type='smd'):
    """An n x n pad lattice, so the pad-pair product is (n*n)^2."""
    pads = []
    k = 0
    for i in range(n):
        for j in range(n):
            pads.append(FakePad(x0 + i * pitch, y0 + j * pitch, size, size,
                                net=net_base + k, layers=layers, drill=drill,
                                pad_type=pad_type))
            k += 1
    return FakeFP(ref, x0, y0, pads=pads)


def _ctx(fps, clearance=0.1):
    parts = build_part_pads({f.reference: f for f in fps}, clearance)
    pose = {f.reference: (f.x, f.y, f.rotation) for f in fps}
    return LegalityContext(parts, None, clearance,
                           pose_of=lambda r: pose[r],
                           seed_of=lambda r: pose[r]), parts


def test_pad_sides_is_exact_not_merely_conservative():
    """Disjoint `pad_sides` iff the per-pad sweep finds no interacting pair.

    Both directions, over every combination of the three pad sides. If this
    ever weakens to "conservative", the early return has to grow an EPS and
    stop claiming bit-identity with the sweep.
    """
    cases = {
        'F.Cu': ('F.Cu',),
        'B.Cu': ('B.Cu',),
        'both': ('F.Cu', 'B.Cu'),
        'star': ('*.Cu',),
    }
    for na, la in cases.items():
        for nb, lb in cases.items():
            drill_a = 0.0
            drill_b = 0.0
            a = FakeFP('A', 0, 0, pads=[FakePad(0, 0, 1, 1, net=1, layers=la,
                                                drill=drill_a)])
            b = FakeFP('B', 5, 0, pads=[FakePad(5, 0, 1, 1, net=2, layers=lb,
                                                drill=drill_b)])
            _, parts = _ctx([a, b])
            pa, pb = parts['A'], parts['B']
            sweep = any(_sides_interact(ra[5], rb[5])
                        for ra in pa.pad_rects(0, 0, 0)
                        for rb in pb.pad_rects(5, 0, 0))
            sets = bool(pa.pad_sides & pb.pad_sides)
            assert sweep == sets, (
                'pad_sides disagrees with the sweep for {} x {}: '
                'sweep={} sets={} ({} vs {})'.format(
                    na, nb, sweep, sets, sorted(pa.pad_sides),
                    sorted(pb.pad_sides)))
    print('  PASS: pad_sides is exact over all 16 side combinations')


def test_an_smd_pad_on_both_faces_is_through_not_back():
    """The `pside` hardening. Without it the early return is a false ACCEPT.

    A non-drilled pad listed on `*.Cu`, or on F.Cu and B.Cu both, used to fall
    to the `'B'` arm and declare its F copper away -- harmless while the branch
    ignored sides, a gate bypass once a disjoint pair is skipped.
    """
    for layers in (('*.Cu',), ('F.Cu', 'B.Cu'), ('B.Cu', 'F.Cu')):
        fp = FakeFP('A', 0, 0, pads=[FakePad(0, 0, 1, 1, layers=layers)])
        pp = PartPads(fp, 0.1)
        assert pp.pad_rects(0, 0, 0)[0][5] is None, (
            'a non-drilled pad on {} must read as BOTH faces, not back-only'
            .format(layers))
        assert pp.pad_sides == L.BOTH_SIDES, sorted(pp.pad_sides)
    # ...and a single-face pad is unchanged.
    for layers, want in ((('F.Cu',), 'F'), (('B.Cu',), 'B')):
        fp = FakeFP('A', 0, 0, pads=[FakePad(0, 0, 1, 1, layers=layers)])
        assert PartPads(fp, 0.1).pad_rects(0, 0, 0)[0][5] == want, layers
    print('  PASS: an ambiguous SMD pad reads as both faces; single-face '
          'pads unchanged')


def test_the_verdict_does_not_depend_on_the_pad_pair_product():
    """A cross-side pair, straddling the cap. Same answer on both sides.

    n=64 gives 4096 pad pairs (the per-pad sweep, since the test is strictly
    greater) and n=65 gives 4225 (the extent branch). The parts are stacked at
    the same origin, so their EXTENTS overlap completely -- the case the old
    branch charged as a 100% conflict.
    """
    seen = {}
    for n in (64, 65):
        a = _grid('A', 8, 0.0, 0.0, layers=('F.Cu',), net_base=1)
        b = _grid('B', 8, 0.0, 0.0, layers=('B.Cu',), net_base=500)
        # 8x8 = 64 pads each -> 4096 and, with one more pad, over the cap.
        if n == 65:
            b.pads.append(FakePad(0.0, -1.0, 0.4, 0.4, net=999,
                                  layers=('B.Cu',)))
        ctx, parts = _ctx([a, b])
        product = parts['A'].n_pads * parts['B'].n_pads
        over = product > L.PAIR_TEST_CAP
        assert over == (n == 65), (
            'the fixture no longer straddles the cap: n={} product={} '
            'cap={}'.format(n, product, L.PAIR_TEST_CAP))
        sf = ctx.pair_shortfall('A', 'B')
        seen[n] = (sf, product, over)
    (sf_under, p_under, _), (sf_over, p_over, _) = seen[64], seen[65]
    assert sf_under.pad == sf_over.pad == 0.0, seen
    assert not sf_under.pad_overlap and not sf_over.pad_overlap, seen
    assert not sf_under.stack and not sf_over.stack, seen
    print('  PASS: {} pad pairs (sweep) and {} (extent branch) agree: '
          'pad 0.0, no overlap, no stack'.format(p_under, p_over))


def test_a_cross_side_stack_is_not_an_assembly_conflict():
    """`stack` is the ANY-net channel, and it too was side-blind above the cap.

    The board's own sheet already says so: `render_placement` prints
    front-to-back overlaps as "opposite faces, NOT a conflict" and
    `tests/test_run23_courtyard_channel.py` pins that. Above the cap the gate
    refused the same arrangement, so the sheet and the gate disagreed and the
    tiebreak was a pad count.
    """
    a = _grid('A', 8, 0.0, 0.0, layers=('F.Cu',), net_base=1)
    b = _grid('B', 9, 0.0, 0.0, layers=('B.Cu',), net_base=500)
    ctx, parts = _ctx([a, b])
    assert parts['A'].n_pads * parts['B'].n_pads > L.PAIR_TEST_CAP
    sf = ctx.pair_shortfall('A', 'B')
    assert sf is ZERO_SHORTFALL, sf
    assert ctx.pads_ok('B', 0.0, 0.0, 0.0, ['A']), 'a legal pose was refused'
    print('  PASS: fully overlapping cross-side extents -> the singleton, '
          'and the pose is admitted')


def test_the_hole_channel_still_fires_across_the_faces():
    """A drill pierces both faces, so the early return may not zero it.

    Not corpus-visible at seed poses, which is exactly why it needs a test: an
    early return that answered a blanket ZERO_SHORTFALL would pass every board
    in the tree and delete #761's channel for cross-side pairs.
    """
    # An NPTH mounting hole (no copper) on a front part...
    hole = FakeFP('H', 0, 0, layer='F.Cu',
                  pads=[FakePad(0, 0, 3.2, 3.2, net=0, layers=('*.Cu',),
                                drill=2.2, pad_type='np_thru_hole')])
    # ...and a BACK-side pad parked right on it.
    back = FakeFP('B', 0, 0, layer='B.Cu',
                  pads=[FakePad(0.0, 0.0, 1.0, 1.0, net=7,
                                layers=('B.Cu',))])
    ctx, parts = _ctx([hole, back], clearance=0.2)
    assert parts['H'].pad_sides == frozenset(), sorted(parts['H'].pad_sides)
    assert parts['B'].pad_sides == frozenset(('B',))
    sf = ctx.pair_shortfall('H', 'B')
    assert sf.pad == 0.0 and not sf.pad_overlap and not sf.stack, sf
    assert sf.hole > 0.0, (
        'the NPTH keep-out stopped being measured for a cross-side pair; the '
        'disjoint-sides return must not be a blanket zero: {}'.format(sf))
    assert sf is not ZERO_SHORTFALL, 'the singleton hides a real hole charge'
    print('  PASS: cross-side pad 0.0 with hole {:.3f}mm still charged'
          .format(sf.hole))


def test_a_through_pad_is_never_skipped():
    """`pside is None` occupies both faces, so a drilled part interacts with
    everything -- the case a naive `footprint_side` comparison gets wrong."""
    tht = FakeFP('T', 0, 0, layer='F.Cu',
                 pads=[FakePad(0, 0, 1.6, 1.6, net=3, layers=('*.Cu',),
                               drill=0.8, pad_type='thru_hole')])
    back = FakeFP('B', 0, 0, layer='B.Cu',
                  pads=[FakePad(0.4, 0, 1.0, 1.0, net=4, layers=('B.Cu',))])
    ctx, parts = _ctx([tht, back])
    assert parts['T'].pad_sides == L.BOTH_SIDES
    assert parts['T'].pad_sides & parts['B'].pad_sides
    sf = ctx.pair_shortfall('T', 'B')
    assert sf.pad > 0.0, (
        'a through pad must still conflict with back-side copper: {}'
        .format(sf))
    print('  PASS: a drilled part still conflicts across the faces '
          '(pad {:.3f}mm)'.format(sf.pad))


def test_the_per_side_extent_is_reached_by_a_two_faced_part():
    """The over-cap branch's per-side box, exercised where it CHANGES a verdict.

    No tracked board has a part with SMD pads on both faces above the cap, so
    without this arm `extent_side` is unreachable and could be deleted with the
    suite still green.

    A carries an F lattice at the origin and a B lattice 50mm away; B is
    back-side only and sits ON A's F lattice. The two share only the BACK face,
    where they are 50mm apart -- but A's WHOLE extent spans both lattices, so
    the extent branch used to see a full overlap and charge it. The first
    version of this arm put B on A's back lattice instead, where both models
    agree, and a mutation reverting the branch to `rect_gap(ea, eb)` survived
    it. The battery caught that.
    """
    a = _grid('A', 8, 0.0, 0.0, layers=('F.Cu',), net_base=1)
    for i in range(8):
        for j in range(8):
            a.pads.append(FakePad(50.0 + i, 50.0 + j, 0.4, 0.4,
                                  net=200 + i * 8 + j, layers=('B.Cu',)))
    b = _grid('B', 8, 0.0, 0.0, layers=('B.Cu',), net_base=900)
    ctx, parts = _ctx([a, b])
    assert parts['A'].pad_sides == L.BOTH_SIDES, sorted(parts['A'].pad_sides)
    assert parts['A'].pad_sides & parts['B'].pad_sides, 'the pair must not be '
    assert parts['A'].n_pads * parts['B'].n_pads > L.PAIR_TEST_CAP, (
        parts['A'].n_pads, parts['B'].n_pads)
    # The whole extent spans both lattices and OVERLAPS B; the shared-side box
    # does not. That difference is the arm.
    ea = parts['A'].extent(0.0, 0.0, 0.0)
    eb = parts['B'].extent(0.0, 0.0, 0.0)
    assert L.rect_gap(ea, eb) < 0.0, (
        'the whole extents no longer overlap, so this arm cannot tell the two '
        'models apart: {} vs {}'.format(ea, eb))
    back_a = parts['A'].extent_side(0.0, 0.0, 0.0, 'B')
    back_b = parts['B'].extent_side(0.0, 0.0, 0.0, 'B')
    assert L.rect_gap(back_a, back_b) > 40.0, (back_a, back_b)
    assert parts['A'].extent_side(0.0, 0.0, 0.0, 'F') != back_a
    sf = ctx.pair_shortfall('A', 'B')
    assert sf.pad == 0.0 and not sf.stack and not sf.pad_overlap, (
        'the pair is 50mm apart on the only face they share; the whole-part '
        'extent is what says otherwise: {}'.format(sf))
    # ...and moving B onto A's BACK lattice IS a conflict, so the arm is not
    # merely asserting that everything is clear.
    near = ctx.pair_shortfall('A', 'B', pose_b=(50.0, 50.0, 0.0))
    assert near.pad > 0.0 and near.stack, near
    print('  PASS: shared-face gap {:.1f}mm grades clean while the whole '
          'extents overlap; on the shared lattice it still conflicts'
          .format(L.rect_gap(back_a, back_b)))


def _ulx3s_ctx(clearance=0.2):
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(ULX3S)
    fps = pcb.footprints
    model = L.PadClearanceModel.for_board(pcb, clearance, ULX3S)
    notes = list(model.notes)
    if not model.active:
        model = None
    parts = build_part_pads(
        fps, clearance, model,
        npth_floor=L.resolve_npth_floor(pcb, ULX3S, notes))
    pose = {r: (fps[r].x, fps[r].y, fps[r].rotation or 0.0) for r in parts}
    return LegalityContext(parts, None, clearance, lambda r: pose[r],
                           lambda r: pose[r], model), parts, pose


def test_the_issue_verification_on_ulx3s():
    """#834's own acceptance test, on the board it was filed against."""
    if not os.path.isfile(ULX3S):
        print('  SKIP: ulx3s not present')
        return
    ctx, parts, pose = _ulx3s_ctx()
    assert parts['U1'].n_pads * parts['U9'].n_pads > L.PAIR_TEST_CAP, (
        'U1 x U9 no longer exceeds the cap; this arm proves nothing')
    assert not (parts['U1'].pad_sides & parts['U9'].pad_sides), (
        sorted(parts['U1'].pad_sides), sorted(parts['U9'].pad_sides))
    target = (138.480, 87.800, pose['U9'][2])
    sf = ctx.pair_shortfall('U1', 'U9', pose_b=target)
    assert sf.pad == 0.0 and not sf.pad_overlap, sf
    assert ctx.pads_ok('U9', target[0], target[1], target[2], ['U1']), (
        'the search still refuses a pose with no interacting pad pair')
    # ...and the same-side over-cap pair is untouched.
    same = ctx.pair_shortfall('U1', 'U2')
    assert parts['U1'].n_pads * parts['U2'].n_pads > L.PAIR_TEST_CAP
    assert parts['U1'].pad_sides & parts['U2'].pad_sides
    assert same.pad == 0.0 and not same.stack, same
    print('  PASS: ulx3s U1xU9 at U1 centre -> pad 0.0, pads_ok True; '
          'U1xU2 unchanged')


def test_the_gate_and_the_grader_now_agree_on_ulx3s():
    """The disagreement #834 is about: `pads_ok` refused what
    `grade_pad_legality` -- which has no cap and always filters by side --
    reported clean."""
    if not os.path.isfile(ULX3S):
        print('  SKIP: ulx3s not present')
        return
    ctx, parts, pose = _ulx3s_ctx()
    for a, b in (('U1', 'U9'), ('U1', 'U6')):
        assert not (parts[a].pad_sides & parts[b].pad_sides), (a, b)
        ea = parts[a].extent(*pose[a])
        centre = ((ea[0] + ea[2]) / 2.0, (ea[1] + ea[3]) / 2.0, pose[b][2])
        sf = ctx.pair_shortfall(a, b, pose_b=centre)
        assert sf.pad == 0.0 and not sf.stack and not sf.pad_overlap, (
            a, b, sf)
    print('  PASS: both ulx3s cross-side over-cap pairs grade clean at full '
          'overlap, as the grader always did')


def main():
    bad = []
    for name, fn in sorted(globals().items()):
        if not name.startswith('test_'):
            continue
        print('--- {}'.format(name))
        try:
            fn()
        except Exception as exc:                             # noqa: BLE001
            bad.append('{}: {}: {}'.format(name, type(exc).__name__, exc))
            print('  FAIL: {}'.format(bad[-1]))
    if bad:
        print('\n{} FAILED'.format(len(bad)))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())

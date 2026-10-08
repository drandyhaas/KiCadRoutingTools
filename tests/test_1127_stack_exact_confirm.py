#!/usr/bin/env python3
"""#1127: the placement gate's stack test, confirmed on the pads' outlines.

`legality.pads_ok` refuses a NEW `PairShortfall.stack` -- two parts' pad
BOXES overlapping on a shared side. Conservative by design, and measured
wrong where it matters: esp_prog's C4 turned to -45 beside Y1's corner, 35 of
121 poses read as a box stack that check_assembly's exact channel calls clean
(tests/measure_1064_box_stack_sweep.py). `legality.STACK_EXACT_CONFIRM`
confirms a box hit with `_exact_pad_stack` -- the check
`pad_intersection_pairs` makes -- before the gate counts it. What each case
pins:

* the 121-pose grid: with the toggle the gate's `.stack` equals
  check_assembly's pad_intersection verdict at EVERY pose, both directions;
  without it they disagree at 35 (a fixture arm: the grid itself is pinned);
* a real stack is still a stack: a same-net SMD pad on another part's, and a
  through-hole `*.Cu` pad on an SMD one -- the layer case a gate with no
  copper-layer list would wave through;
* the over-cap extent shortcut stays a box answer whatever the toggle;
* the mode is fixed when the context is built, and a part built without a
  snapshot falls back to the box answer (falsely reject, never accept);
* check_drc's id-keyed perimeter and polygon caches HOLD their pad: an entry
  whose fingerprint matches but whose pad is another object is recomputed,
  not served -- the posed copies the gate makes are short-lived;
* and the perimeter fingerprint is every input of the outline: the SAME pad
  turned about its own centre, reshaped, re-cornered or given new polygons
  is recomputed.

    python3 -X utf8 tests/test_1127_stack_exact_confirm.py [case ...]
"""
import copy
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_utils  # noqa: E402

ROOT = run_utils.ROOT_DIR
for _sub in ('py_router', 'py_tools', 'py_placer'):
    _p = os.path.join(ROOT, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from kicad_parser import parse_kicad_pcb            # noqa: E402
from placement import legality                      # noqa: E402

RUN_ALL_TIMEOUT = 900
ESP = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
_TMP = tempfile.TemporaryDirectory(prefix='t1127_')


class _toggle:
    def __init__(self, on):
        self.on, self.saved = on, None

    def __enter__(self):
        self.saved = legality.STACK_EXACT_CONFIRM
        legality.STACK_EXACT_CONFIRM = self.on

    def __exit__(self, *exc):
        legality.STACK_EXACT_CONFIRM = self.saved
        return False


def _state(board, on):
    from placement.quench import QuenchState
    with _toggle(on):
        return QuenchState(parse_kicad_pcb(board), board, clearance=0.2,
                           board_edge_clearance=0.55, crossing_penalty=10.0,
                           halo_base=0.5, halo_coef=0.25, halo_weight=2.0,
                           edge_halo=2.0, edge_weight=2.0, grid_step=0.1,
                           length_weight=1.0)


def _exact(pcb, ref, other, pose):
    """check_assembly's pad_intersection verdict for the pair, with `ref`
    posed in memory."""
    p2 = copy.copy(pcb)
    p2.footprints = dict(pcb.footprints)
    p2.footprints[ref] = legality.footprint_at_pose(pcb.footprints[ref], pose)
    return any({q.a, q.b} == {ref, other}
               for q in legality.pad_intersection_pairs(p2, 0.2))


def _grid():
    return [(round(135.86 + i * 0.04, 4), round(103.42 + j * 0.04, 4), 315.0)
            for j in range(11) for i in range(11)]


def test_the_grid_agrees_with_check_assembly():
    pcb = parse_kicad_pcb(ESP)
    on, off = _state(ESP, True), _state(ESP, False)
    assert on.legality_ctx.stack_exact and not off.legality_ctx.stack_exact
    diff_on = diff_off = 0
    for pose in _grid():
        ex = _exact(pcb, 'C4', 'Y1', pose)
        for st in (on, off):
            st.apply_move('C4', *pose)
        diff_on += on.legality_ctx.pair_shortfall('C4', 'Y1').stack != ex
        diff_off += off.legality_ctx.pair_shortfall('C4', 'Y1').stack != ex
    assert diff_off == 35, (diff_off, 'the grid moved: #1064 measured 35')
    assert diff_on == 0, diff_on
    # the same points turned 45 degrees: the gate's posed-pad cache must
    # key on the angle, or a turn at one x, y reads the last angle's pads
    turned = 0
    for x, y, _r in _grid():
        pose = (x, y, 0.0)
        ex = _exact(pcb, 'C4', 'Y1', pose)
        on.apply_move('C4', *pose)
        turned += on.legality_ctx.pair_shortfall('C4', 'Y1').stack != ex
    assert turned == 0, turned
    print(f"  PASS: 121 poses -- box vs exact disagree at {diff_off} with the "
          f"toggle off, at {diff_on} with it on")


SMD = ('(kicad_pcb (version 20240108) (generator "t1127")\n'
       '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
       '  (net 0 "") (net 1 "N1")\n'
       '  (gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts"))\n'
       '  (footprint "a" (layer "F.Cu") (at 10 10)\n'
       '    (property "Reference" "A")\n'
       '    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "N1")))\n'
       '  (footprint "b" (layer "F.Cu") (at 10.5 10)\n'
       '    (property "Reference" "B")\n'
       '    (pad "1" {kind} rect (at 0 0) (size 1 1){drill} (layers {layers})'
       ' (net 1 "N1"))))\n')


def _board(kind='smd', layers='"F.Cu"', drill=''):
    path = os.path.join(_TMP.name, f'b_{kind}.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(SMD.format(kind=kind, layers=layers, drill=drill))
    return path


def _back_board():
    """A through-hole pad (A, `*.Cu`) on a back-side SMD pad (B on B.Cu):
    the stack is on the BACK face, which a confirmation on F.Cu alone
    would miss."""
    text = (SMD.format(kind='smd', layers='"B.Cu"', drill='')
            .replace('(pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu")',
                     '(pad "1" thru_hole rect (at 0 0) (size 1 1) (drill 0.4)'
                     ' (layers "*.Cu" "*.Mask")', 1)
            .replace('(footprint "b" (layer "F.Cu")',
                     '(footprint "b" (layer "B.Cu")', 1))
    path = os.path.join(_TMP.name, 'b_back.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return path


def test_a_real_stack_is_still_a_stack():
    for kind, layers, drill in (('smd', '"F.Cu"', ''),
                                ('thru_hole', '"*.Cu" "*.Mask"',
                                 ' (drill 0.4)'),
                                ('back', '"B.Cu"', '')):
        path = (_back_board() if kind == 'back'
                else _board(kind, layers, drill))
        st = _state(path, True)
        sf = st.legality_ctx.pair_shortfall('A', 'B')
        assert sf.stack, (kind, sf)
        # the fixture's layer table parses to no copper layers, and
        # pad_intersection_pairs expands `*.Cu` against the board's
        pcb = parse_kicad_pcb(path)
        pcb.board_info.copper_layers = ['F.Cu', 'B.Cu']
        assert _exact(pcb, 'A', 'B', (10.0, 10.0, 0.0)), kind
        # and the gate refuses a move that keeps the two stacked when the
        # seed was clean
        st2 = _state(path, True)
        st2.parts['A'].seed_x = 2.0     # seed: far apart
        st2.legality_ctx._baselines.clear()
        assert not st2.legality_ctx.pads_ok('A', 10.0, 10.0, 0.0, ['B'])
    print("  PASS: a same-net SMD stack, a THT-on-SMD stack and a THT pad "
          "on a BACK pad are stacks with the toggle on, and pads_ok refuses "
          "them")


TWO = ('(kicad_pcb (version 20240108) (generator "t1127")\n'
       '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
       '  (net 0 "") (net 1 "N1") (net 2 "N2")\n'
       '  (gr_rect (start 0 0) (end 30 30) (layer "Edge.Cuts"))\n'
       '  (footprint "a" (layer "F.Cu") (at 10 10)\n'
       '    (property "Reference" "A")\n'
       '    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "N1"))\n'
       '    (pad "2" smd circle (at 3 0) (size 1 1) (layers "F.Cu") (net 2 "N2")))\n'
       '  (footprint "b" (layer "F.Cu") (at 10.5 10)\n'
       '    (property "Reference" "B")\n'
       '    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "N1"))\n'
       '    (pad "2" smd circle (at {bx} {by}) (size 1 1) (layers "F.Cu")'
       ' (net 2 "N2"))))\n')


def _two(bx, by, name):
    path = os.path.join(_TMP.name, name)
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(TWO.format(bx=bx, by=by))
    return path


def test_a_box_only_hit_after_a_real_stack_keeps_the_stack():
    """A1 really stacks on B1; A2 and B2 are circles whose boxes overlap
    diagonally while the circles stay 0.06 mm apart. The box-only pair comes
    LATER in the sweep, and it must not turn the pair's confirmed stack back
    off."""
    path = _two(3.25, 0.75, 'two.kicad_pcb')
    st = _state(path, True)
    sf = st.legality_ctx.pair_shortfall('A', 'B')
    assert sf.stack, sf
    # and the box-only pair alone is no stack (the control)
    path2 = _two(3.25 + 5.0, 0.75, 'two_far.kicad_pcb')
    p2 = parse_kicad_pcb(path2)
    a2 = [p for p in p2.footprints['A'].pads if p.pad_number == '2'][0]
    b2 = [p for p in parse_kicad_pcb(path).footprints['B'].pads
          if p.pad_number == '2'][0]
    assert not legality._exact_pad_stack(a2, b2, ['F.Cu', 'B.Cu'])
    print("  PASS: a box-only hit later in the sweep leaves a confirmed stack "
          "standing")


def test_the_exact_check_needs_contact_not_nearness():
    """Two 1 mm circles 1.0005 mm apart are near (closer than the check's
    epsilon) and not touching; 0.9995 mm apart they overlap. Only the second
    is a stack -- in the gate and in check_assembly, which share the check."""
    import math
    out = {}
    for d in (1.0005, 0.9995):
        off = d / math.sqrt(2.0)
        pcb = parse_kicad_pcb(_two(round(off - 0.5, 6) + 3.0, round(off, 6),
                                   f'near_{d}.kicad_pcb'))
        a2 = [p for p in pcb.footprints['A'].pads if p.pad_number == '2'][0]
        b2 = [p for p in pcb.footprints['B'].pads if p.pad_number == '2'][0]
        got = math.hypot(a2.global_x - b2.global_x, a2.global_y - b2.global_y)
        assert abs(got - d) < 2e-6, (d, got)
        out[d] = legality._exact_pad_stack(a2, b2, ['F.Cu', 'B.Cu'])
    assert out == {1.0005: False, 0.9995: True}, out
    print("  PASS: 1.0005 mm apart is not a stack, 0.9995 mm apart is")


def test_the_extent_shortcut_is_a_box_answer():
    real = legality.PAIR_TEST_CAP
    legality.PAIR_TEST_CAP = 0
    try:
        on, off = _state(ESP, True), _state(ESP, False)
        n = 0
        for pose in _grid()[::10]:
            for st in (on, off):
                st.apply_move('C4', *pose)
            a = on.legality_ctx.pair_shortfall('C4', 'Y1')
            b = off.legality_ctx.pair_shortfall('C4', 'Y1')
            assert a == b, (pose, a, b)
            n += a.stack
    finally:
        legality.PAIR_TEST_CAP = real
    assert n, 'no pose reached the shortcut with a stack: the case tests nothing'
    print(f"  PASS: over the cap the verdict is the box's whatever the toggle "
          f"({n} stacked poses)")


def test_the_mode_is_fixed_at_build_and_a_snapshotless_part_is_a_box():
    pcb = parse_kicad_pcb(ESP)
    on = _state(ESP, True)
    over = [p for p in _grid()
            if not _exact(pcb, 'C4', 'Y1', p)]
    with _toggle(False):                 # flipping the global later ...
        on.apply_move('C4', *over[0])
        assert not on.legality_ctx.pair_shortfall('C4', 'Y1').stack
    # ... and a context whose parts were built WITHOUT the snapshot answers
    # with the box even when the context itself confirms
    off = _state(ESP, False)
    off.legality_ctx.stack_exact = True
    hits = 0
    for pose in over:
        off.apply_move('C4', *pose)
        hits += off.legality_ctx.pair_shortfall('C4', 'Y1').stack
    assert hits == 35, hits
    print("  PASS: the context keeps the mode it was built with; parts with no "
          "snapshot keep the box answer (35 of the exact-clean poses)")


def test_check_drc_caches_hold_their_pad():
    import check_drc
    pcb = parse_kicad_pcb(ESP)
    pad = pcb.footprints['C4'].pads[0]
    fp = check_drc._pad_perimeter_fingerprint(pad)
    check_drc._PAD_PERIMETER_CACHE[id(pad)] = (fp, 'STALE', object())
    got = check_drc._pad_perimeter_array(pad)
    assert got != 'STALE', 'an entry held for another pad was served'
    polys = [[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]]
    check_drc._POLYS_EDGE_CACHE[id(polys)] = ((1, (3,)), 'STALE', object())
    assert check_drc._polys_edge_arrays(polys) != 'STALE'
    # the held entry is served
    assert check_drc._pad_perimeter_array(pad) is got
    print("  PASS: a cache entry whose fingerprint matches but whose object "
          "is another is recomputed; the held one is served")


def test_check_drc_cache_sees_a_pad_changed_in_place():
    import check_drc
    pcb = parse_kicad_pcb(ESP)
    pad = copy.copy(pcb.footprints['C4'].pads[0])
    # ONE input changes per step, so each fingerprint field is tested alone.
    pad.polygons = None
    pad.shape, pad.rect_rotation, pad.roundrect_rratio = 'rect', 30.0, 0.25

    def served_is_fresh(what):
        got = check_drc._pad_perimeter_array(pad)[2]
        assert got == check_drc._pad_perimeter_points(pad), (
            f'{what}: the cache served the outline it had before')
        return got

    first = served_is_fresh('turned 30')
    # Same pad object, same centre and size box: only the turn moves.
    pad.rect_rotation = -30.0
    turned = served_is_fresh('turned 30 -> 330 in place')
    assert turned != first
    pad.shape = 'roundrect'
    rounded = served_is_fresh('rect -> roundrect in place')
    assert rounded != turned
    pad.roundrect_rratio = 0.5
    assert served_is_fresh('corner ratio changed in place') != rounded
    x, y = pad.global_x, pad.global_y
    pad.polygons = [[(x - 1, y - 1), (x + 1, y - 1), (x + 1, y + 1)]]
    tri = served_is_fresh('polygons assigned')
    pad.polygons = [[(x - 2, y - 2), (x + 2, y - 2), (x + 2, y + 2)]]
    assert served_is_fresh('polygons reassigned') != tri
    print("  PASS: a pad turned, reshaped, re-cornered or given new polygons "
          "IN PLACE is recomputed, not served its old outline")


TESTS = [
    test_the_grid_agrees_with_check_assembly,
    test_a_real_stack_is_still_a_stack,
    test_a_box_only_hit_after_a_real_stack_keeps_the_stack,
    test_the_exact_check_needs_contact_not_nearness,
    test_the_extent_shortcut_is_a_box_answer,
    test_the_mode_is_fixed_at_build_and_a_snapshotless_part_is_a_box,
    test_check_drc_caches_hold_their_pad,
    test_check_drc_cache_sees_a_pad_changed_in_place,
]


if __name__ == '__main__':
    want = sys.argv[1:]
    for t in TESTS:
        if want and not any(w in t.__name__ for w in want):
            continue
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")

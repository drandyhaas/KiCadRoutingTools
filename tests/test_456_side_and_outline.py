"""
Tests for the quench clearance model's board-side and board-outline awareness
(issue #456 items 1 and 2), and for the must-not-worsen acceptance rule.

Item 1 -- side. `_Part` never read `fp.layer`, so `candidate_valid` and
`_halo_pair_penalty` treated every part as coplanar. A back-side decoupling cap
that legitimately sits under a front-side BGA failed the clearance check for
every candidate near its seed, so the part was effectively FROZEN (only
candidates are validated, never the incumbent), and halo pressure pushed
cross-side parts apart for no reason. But a THROUGH-HOLE part does occupy both
sides: a part occupies its own side with its full courtyard and the opposite side
only with the bounding box of its drilled pads.

Item 2 -- outline. `usable` is an inset of the axis-aligned `board_bounds`, blind
to L-shaped outlines and interior cutouts, so parts got nudged into the notch or
the hole. Candidates are now additionally gated against the true Edge.Cuts rings
(`legality.BoardOutlineGate`, ported from the fanout-clearance repair pass).

Frozen seeds. `candidate_valid` accepts a candidate whose violation is no worse
than the CURRENT pose's when the current pose is already illegal, so a part that
starts in violation can still move instead of being pinned forever.
"""

import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # #522
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # #522

from kicad_parser import parse_kicad_pcb
from placement import legality
from placement.legality import BoardOutlineGate, GradedPart
from placement.quench import QuenchState, quench

KICAD_FILES = os.path.join(os.path.dirname(__file__), '..', 'kicad_files')

# The canonical QuenchState construction recipe at quench() defaults.
KNOBS = dict(clearance=0.25, board_edge_clearance=0.55, crossing_penalty=10.0,
             halo_base=0.3, halo_coef=0.15, halo_weight=1.0,
             edge_halo=1.0, edge_weight=1.0, grid_step=0.05)


# --- synthetic two-sided board ------------------------------------------------

def _part(ref, x, y, layer, net_id, net_name, cy=2.0, drill=0.0, locked=False):
    """A 2-pad part with an explicit square courtyard of half-width `cy`.
    drill > 0 makes pad 1 through-hole, so the part reaches both sides."""
    lock = '\t\t(locked yes)\n' if locked else ''
    if drill > 0:
        pad1 = (f'\t\t(pad "1" thru_hole circle\n\t\t\t(at -0.5 0)\n'
                f'\t\t\t(size {drill * 2} {drill * 2})\n'
                f'\t\t\t(drill {drill})\n\t\t\t(layers "*.Cu")\n'
                f'\t\t\t(net {net_id} "{net_name}")\n\t\t\t(uuid "p1-{ref}")\n\t\t)\n')
    else:
        pad1 = (f'\t\t(pad "1" smd rect\n\t\t\t(at -0.5 0)\n\t\t\t(size 0.6 0.8)\n'
                f'\t\t\t(layers "{layer}")\n'
                f'\t\t\t(net {net_id} "{net_name}")\n\t\t\t(uuid "p1-{ref}")\n\t\t)\n')
    cyl = layer[0] + '.CrtYd'
    return f'''\t(footprint "test:P{ref}"
\t\t(layer "{layer}")
{lock}\t\t(uuid "fp-{ref}")
\t\t(at {x} {y})
\t\t(property "Reference" "{ref}"
\t\t\t(at 0 0)
\t\t)
\t\t(fp_rect
\t\t\t(start {-cy} {-cy})
\t\t\t(end {cy} {cy})
\t\t\t(layer "{cyl}")
\t\t\t(uuid "cy-{ref}")
\t\t)
{pad1}\t\t(pad "2" smd rect
\t\t\t(at 0.5 0)
\t\t\t(size 0.6 0.8)
\t\t\t(layers "{layer}")
\t\t\t(uuid "p2-{ref}")
\t\t)
\t)
'''


def _write(footprints, outline=None):
    outline = outline or ('\t(gr_rect\n\t\t(start 100 80)\n\t\t(end 200 120)\n'
                          '\t\t(layer "Edge.Cuts")\n\t\t(uuid "e1")\n\t)\n')
    body = ('(kicad_pcb\n\t(version 20241229)\n'
            '\t(net 0 "")\n\t(net 1 "NA")\n\t(net 2 "NB")\n'
            + outline + footprints + ')\n')
    fd, path = tempfile.mkstemp(suffix='.kicad_pcb')
    with os.fdopen(fd, 'w') as f:
        f.write(body)
    return parse_kicad_pcb(path), path


def _state(footprints, outline=None, **over):
    pcb, path = _write(footprints, outline)
    knobs = dict(KNOBS)
    knobs.update(over)
    return QuenchState(pcb, path, **knobs), path


# --- item 1: cross-side parts do not collide ---------------------------------

def test_back_side_part_under_front_part_is_not_a_collision():
    """The motivating case: a B.Cu decap directly under an F.Cu part. Their
    courtyards overlap completely in XY and they must still both be legal."""
    body = (_part('U1', 150.0, 100.0, 'F.Cu', 1, 'NA', cy=4.0)
            + _part('C1', 150.0, 100.0, 'B.Cu', 2, 'NB', cy=1.0))
    st, path = _state(body)
    assert st.parts['U1'].side == 'F' and st.parts['C1'].side == 'B'
    assert st.parts['C1'].gap_to(st.parts['U1']) is None, \
        "cross-side SMD parts must share no side"
    assert st.candidate_valid('C1', 150.0, 100.0, 0.0), \
        "back-side part under a front-side part must be a legal pose"
    assert st._halo_pair_penalty(st.parts['C1'], st.parts['C1'].rect(),
                                 st.parts['U1'], st.parts['U1'].rect()) == 0.0, \
        "halo must not push cross-side parts apart"
    assert st.violation('C1') == 0.0, f"violation: {st.violation('C1')}"
    os.unlink(path)


def test_same_side_overlap_is_still_a_collision():
    """The side rule must not weaken the same-side check. C1 is seeded CLEAR of
    U1 so the probe is a genuine candidate, not the incumbent (a pose is always
    allowed to be no worse than itself)."""
    body = (_part('U1', 150.0, 100.0, 'F.Cu', 1, 'NA', cy=4.0)
            + _part('C1', 165.0, 100.0, 'F.Cu', 2, 'NB', cy=1.0))
    st, path = _state(body)
    assert st.violation('C1') == 0.0, "fixture error: C1's seed should be legal"
    assert not st.candidate_valid('C1', 150.0, 100.0, 0.0), \
        "same-side courtyard overlap must still be rejected"
    assert st.violation('C1', 150.0, 100.0, 0.0) > 0.0
    c1, u1 = st.parts['C1'], st.parts['U1']
    assert st._halo_pair_penalty(c1, c1.rect(150.0, 100.0, 0.0), u1, u1.rect(),
                                 rects_a=c1.rects(150.0, 100.0, 0.0)) > 0.0, \
        "same-side parts must still pay the halo penalty"
    os.unlink(path)


def test_cross_side_part_is_no_longer_frozen():
    """End to end: the back-side cap's airwire pulls it left, and before the fix
    every candidate was rejected by the front-side part it sits under."""
    body = (_part('U1', 150.0, 100.0, 'F.Cu', 1, 'NA', cy=6.0)
            + _part('C1', 150.0, 100.0, 'B.Cu', 2, 'NB', cy=1.0)
            + _part('J2', 130.0, 100.0, 'F.Cu', 2, 'NB', cy=1.0, locked=True))
    pcb, path = _write(body)
    result = quench(pcb, path, max_displacement=5.0, step=1.0,
                    allow_rotations=False, allow_swaps=False, max_passes=2)
    moved = {r['reference'] for r in result}
    assert 'C1' in moved, \
        f"the back-side cap under U1 is still frozen (moved: {moved})"
    c1 = next(r for r in result if r['reference'] == 'C1')
    assert c1['new_x'] < 150.0, f"C1 should move toward J2, went to {c1['new_x']}"
    os.unlink(path)


# --- item 1: through-hole parts DO reach the far side ------------------------

def test_through_hole_lead_field_blocks_the_far_side():
    """An F.Cu part with a drilled pad blocks the B side where its leads are --
    but only there, not across its whole courtyard."""
    # C1 is seeded well clear of J1 so both probes are genuine candidates.
    body = (_part('J1', 150.0, 100.0, 'F.Cu', 1, 'NA', cy=4.0, drill=1.0)
            + _part('C1', 165.0, 100.0, 'B.Cu', 2, 'NB', cy=0.5))
    st, path = _state(body)
    j1, c1 = st.parts['J1'], st.parts['C1']
    assert j1.has_tht and j1.sides == frozenset(('F', 'B'))
    assert not c1.has_tht and c1.sides == frozenset(('B',))
    assert j1.tht_rect() is not None, "THT part must present a far-side rect"
    assert st.violation('C1') == 0.0, "fixture error: C1's seed should be legal"
    # Directly on the pin field: rejected.
    assert not st.candidate_valid('C1', 149.5, 100.0, 0.0), \
        "a back-side part inside a front connector's pin field must be rejected"
    # Same courtyard, but clear of the lead field: accepted. The lead box is
    # (-1.5,-1)-(0.5,1) local, so 3mm out in +x is clear of it and still well
    # inside J1's 8x8mm courtyard -- exactly what side-blindness forbade.
    assert st.candidate_valid('C1', 153.0, 100.0, 0.0), \
        "a back-side part clear of the leads but under the courtyard is legal"
    os.unlink(path)


def test_two_smd_parts_on_the_same_side_of_a_tht_neighbour():
    """A THT part still enforces its full courtyard on its OWN side."""
    body = (_part('J1', 150.0, 100.0, 'F.Cu', 1, 'NA', cy=4.0, drill=1.0)
            + _part('C1', 165.0, 100.0, 'F.Cu', 2, 'NB', cy=0.5))
    st, path = _state(body)
    assert st.violation('C1') == 0.0, "fixture error: C1's seed should be legal"
    assert not st.candidate_valid('C1', 153.0, 100.0, 0.0), \
        "same side: the full courtyard applies, not just the lead field"
    os.unlink(path)


# --- item 2: the real board outline ------------------------------------------

L_OUTLINE = (
    # An L: the full 100x40 rectangle minus the top-right 40x20 quadrant.
    '\t(gr_poly\n\t\t(pts\n'
    '\t\t\t(xy 100 80)\n\t\t\t(xy 160 80)\n\t\t\t(xy 160 100)\n'
    '\t\t\t(xy 200 100)\n\t\t\t(xy 200 120)\n\t\t\t(xy 100 120)\n'
    '\t\t)\n\t\t(layer "Edge.Cuts")\n\t\t(uuid "e1")\n\t)\n')


def test_notch_in_the_outline_is_rejected():
    """(180, 90) is inside the bounding box inset and outside the real board."""
    body = _part('C1', 150.0, 110.0, 'F.Cu', 1, 'NA', cy=1.0)
    st, path = _state(body, outline=L_OUTLINE)
    assert st.edge_gate.active, "the L outline must activate the gate"
    notch = (179.0, 89.0, 181.0, 91.0)
    u = st.usable
    assert (notch[0] >= u[0] and notch[1] >= u[1]
            and notch[2] <= u[2] and notch[3] <= u[3]), \
        "fixture error: the notch point should be inside the bbox inset"
    assert st.edge_gate.rect_blocked(notch), "the notch must be rejected"
    assert st.edge_gate.rect_outside_amount(notch) > 0.0
    assert not st.candidate_valid('C1', 180.0, 90.0, 0.0), \
        "candidate_valid must refuse a pose in the notch"
    # ... and a pose in the solid part of the L is still fine.
    assert st.candidate_valid('C1', 150.0, 110.0, 0.0)
    assert st.edge_gate.rect_outside_amount(st.parts['C1'].rect()) == 0.0
    os.unlink(path)


def test_rectangular_board_leaves_the_gate_inert():
    """No cutouts, one rectangular ring: the bbox inset is already exact, so the
    exact ring test must be skipped entirely (behaviour unchanged)."""
    body = _part('C1', 150.0, 100.0, 'F.Cu', 1, 'NA', cy=1.0)
    st, path = _state(body)
    assert not st.edge_gate.active, \
        "a plain rectangular outline must not activate the exact gate"
    os.unlink(path)


def test_real_boards_exercise_the_outline_gate():
    """watchy has 2 interior cutouts, interf_u a non-rectangular outline. Both
    accept rects that the bbox inset alone accepts and the outline rejects."""
    watchy = os.path.join(KICAD_FILES, 'watchy.kicad_pcb')
    if os.path.exists(watchy):
        pcb = parse_kicad_pcb(watchy)
        gate = BoardOutlineGate(pcb.board_info, 0.55)
        assert gate.active and len(gate.cutouts) == 2, \
            f"watchy: active={gate.active} cutouts={len(gate.cutouts)}"
        caught = 0
        for cut in gate.cutouts:
            xs = [p[0] for p in cut]
            ys = [p[1] for p in cut]
            cx, cy = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0
            r = (cx - 0.5, cy - 0.5, cx + 0.5, cy + 0.5)
            u = gate.usable
            in_bbox = (r[0] >= u[0] and r[1] >= u[1]
                       and r[2] <= u[2] and r[3] <= u[3])
            if in_bbox and gate.rect_blocked(r):
                caught += 1
        assert caught == 2, f"watchy cutouts caught by the gate: {caught}/2"
        print(f"  PASS: watchy: both cutouts rejected by the outline gate")

    interf = os.path.join(KICAD_FILES, 'interf_u_unrouted.kicad_pcb')
    if os.path.exists(interf):
        pcb = parse_kicad_pcb(interf)
        gate = BoardOutlineGate(pcb.board_info, 0.55)
        assert gate.active, "interf_u's outline is not a rectangle"
        u = gate.usable
        caught = 0
        for i in range(40):
            for j in range(40):
                x = u[0] + (u[2] - u[0]) * i / 39.0
                y = u[1] + (u[3] - u[1]) * j / 39.0
                r = (x - 0.5, y - 0.5, x + 0.5, y + 0.5)
                if (r[0] >= u[0] and r[1] >= u[1] and r[2] <= u[2]
                        and r[3] <= u[3] and gate.rect_blocked(r)):
                    caught += 1
        assert caught > 0, \
            "interf_u: the outline gate rejected nothing the bbox accepted"
        print(f"  PASS: interf_u: {caught}/1600 bbox-legal probes off-outline")


def test_gate_falls_back_to_bbox_without_rings():
    """Most corpus boards yield no usable Edge.Cuts ring; the gate must go
    inert and leave the bbox inset in charge rather than reject everything."""
    tigard = os.path.join(KICAD_FILES, 'tigard.kicad_pcb')
    if not os.path.exists(tigard):
        return
    pcb = parse_kicad_pcb(tigard)
    gate = BoardOutlineGate(pcb.board_info, 0.55)
    assert not gate.rings and not gate.active, \
        f"tigard unexpectedly has rings: {len(gate.rings)}"
    u = gate.usable
    inside = ((u[0] + u[2]) / 2 - 1, (u[1] + u[3]) / 2 - 1,
              (u[0] + u[2]) / 2 + 1, (u[1] + u[3]) / 2 + 1)
    assert gate.rect_outside_amount(inside) == 0.0
    outside = (u[0] - 5, u[1] - 5, u[0] - 3, u[1] - 3)
    assert gate.rect_outside_amount(outside) > 0.0
    print("  PASS: no rings -> gate inert, bbox inset still enforced")


# --- must-not-worsen acceptance ----------------------------------------------

def test_illegally_seeded_part_can_still_move():
    """A part with NO legal pose in reach used to be pinned forever: only
    candidates were validated, never the incumbent, so the seed was legal by
    default and every alternative was rejected.

    C1 is seeded off the board at x=95 with only 3mm of displacement budget, so
    no reachable pose is inside `usable` (which starts at x=100.55). Its airwire
    to J2 pulls it toward the board; it must now be able to walk back, and its
    violation must strictly decrease.
    """
    body = (_part('C1', 95.0, 100.0, 'F.Cu', 2, 'NB', cy=1.0)
            + _part('J2', 150.0, 100.0, 'F.Cu', 2, 'NB', cy=1.0, locked=True))
    st, path = _state(body)
    seed_violation = st.violation('C1')
    assert seed_violation > 0.0, "fixture error: C1's seed should be illegal"
    for dx in (0.5, 1.0, 2.0, 3.0):
        assert st.violation('C1', 95.0 + dx, 100.0, 0.0) > 0.0, \
            f"fixture error: +{dx}mm should still be off-board"
    assert st.candidate_valid('C1', 98.0, 100.0, 0.0), \
        "moving back TOWARD the board must be allowed from an illegal seed"
    assert not st.candidate_valid('C1', 92.0, 100.0, 0.0), \
        "moving FURTHER off the board must still be refused"

    pcb2, path2 = _write(body)
    result = quench(pcb2, path2, max_displacement=3.0, step=0.5,
                   allow_rotations=False, allow_swaps=False, max_passes=3)
    moved = {r['reference'] for r in result}
    assert 'C1' in moved, f"part with no legal pose in reach is frozen ({moved})"
    c1 = next(r for r in result if r['reference'] == 'C1')
    assert c1['new_x'] > 95.0, f"C1 moved away from the board: {c1['new_x']}"

    st2 = QuenchState(parse_kicad_pcb(path2), path2, **KNOBS)
    st2.parts['C1'].x, st2.parts['C1'].y = c1['new_x'], c1['new_y']
    assert st2.violation('C1') < seed_violation, \
        "the accepted move did not reduce the violation"
    os.unlink(path)
    os.unlink(path2)


def test_overlapping_part_gets_no_licence_to_slide():
    """The unfreezing rule is BOARD-only on purpose.

    Extending it to overlaps -- "any pose no worse than the one you are in" --
    hands almost every part on a dense board a licence to move while still
    overlapping, and total courtyard overlap goes UP (measured on watchy:
    9.1 -> 37.9mm2 with a no-worse rule, 16.8 with a strict-decrease one, against
    9.1 -> 0.04 for the original strict gate). The violation measure is a
    distance while the thing at stake is an area, and the two do not move
    together. So an OVERLAPPING part may only move to a fully legal pose.
    """
    body = (_part('U1', 150.0, 100.0, 'F.Cu', 1, 'NA', cy=4.0, locked=True)
            + _part('C1', 152.0, 100.0, 'F.Cu', 2, 'NB', cy=1.0)
            + _part('J2', 120.0, 100.0, 'F.Cu', 2, 'NB', cy=1.0, locked=True))
    st, path = _state(body)
    board, overlap = st.violation_parts('C1')
    assert overlap > 0.0 and board == 0.0, \
        f"fixture error: C1 should overlap U1 and be on-board ({board}, {overlap})"
    # A shallower overlap is still an overlap: refused, however much better.
    shallower = st.violation_parts('C1', 153.5, 100.0, 0.0)
    assert shallower[1] > 0.0 and shallower[1] < overlap, "fixture error"
    assert not st.candidate_valid('C1', 153.5, 100.0, 0.0), \
        "an overlapping part must not be licensed to slide to a lesser overlap"
    # A fully legal pose is of course still accepted.
    assert st.candidate_valid('C1', 160.0, 100.0, 0.0)
    os.unlink(path)


def test_offboard_part_may_not_slide_into_an_overlap():
    """The board-only escape hatch must not let an off-board part come back
    ON TOP of something."""
    body = (_part('C1', 95.0, 100.0, 'F.Cu', 2, 'NB', cy=1.0)
            + _part('U1', 102.0, 100.0, 'F.Cu', 1, 'NA', cy=1.0, locked=True))
    st, path = _state(body)
    board, overlap = st.violation_parts('C1')
    assert board > 0.0 and overlap == 0.0, "fixture error: C1 off-board, clear"
    # Straight onto U1: closer to the board, but overlapping -> refused.
    assert st.violation_parts('C1', 102.0, 100.0, 0.0)[1] > 0.0, "fixture error"
    assert not st.candidate_valid('C1', 102.0, 100.0, 0.0), \
        "coming back on-board must not be allowed to create an overlap"
    os.unlink(path)


def test_legal_part_is_not_allowed_to_become_illegal():
    """The escape hatch must not leak: from a LEGAL pose, an illegal candidate
    is still refused."""
    body = (_part('U1', 150.0, 100.0, 'F.Cu', 1, 'NA', cy=4.0, locked=True)
            + _part('C1', 160.0, 100.0, 'F.Cu', 2, 'NB', cy=1.0))
    st, path = _state(body)
    assert st.violation('C1') == 0.0, "fixture error: C1's seed should be legal"
    assert not st.candidate_valid('C1', 150.0, 100.0, 0.0), \
        "a legal part must not be allowed to move into an overlap"
    os.unlink(path)


# --- the graders (#456 comment: shared with the #411/#110 scorecard) ---------

def test_overlap_area_grader_is_side_aware():
    a = GradedPart(ref='U1', side='F', rect=(0.0, 0.0, 4.0, 4.0))
    b_front = GradedPart(ref='C1', side='F', rect=(2.0, 2.0, 6.0, 6.0))
    b_back = GradedPart(ref='C1', side='B', rect=(2.0, 2.0, 6.0, 6.0))
    assert legality.placement_overlap_area([a, b_front]) == 4.0, \
        "same-side overlap must be reported as an area"
    assert legality.placement_overlap_area([a, b_back]) == 0.0, \
        "cross-side XY overlap is not a component overlap"
    # A THT front part's lead field reaching the back DOES overlap a back part.
    a_tht = a._replace(has_tht=True, tht_rect=(2.5, 2.5, 3.5, 3.5))
    assert legality.placement_overlap_area([a_tht, b_back]) == 1.0, \
        "a through-hole lead field must overlap a part on the far side"


def test_out_of_board_grader():
    interf = os.path.join(KICAD_FILES, 'interf_u_unrouted.kicad_pcb')
    if not os.path.exists(interf):
        return
    pcb = parse_kicad_pcb(interf)
    b = pcb.board_info.board_bounds
    ok = GradedPart(ref='OK', side='F',
                    rect=((b[0] + b[2]) / 2 - 1, (b[1] + b[3]) / 2 - 1,
                          (b[0] + b[2]) / 2 + 1, (b[1] + b[3]) / 2 + 1))
    off = GradedPart(ref='OFF', side='F',
                     rect=(b[0] - 4.0, b[1] - 4.0, b[0] - 2.0, b[1] - 2.0))
    res = legality.placement_out_of_board([ok], pcb.board_info, 0.55)
    assert res.count == 0 and res.area == 0.0, f"clean board graded {res}"
    res = legality.placement_out_of_board([ok, off], pcb.board_info, 0.55)
    assert res.count == 1, f"off-board part not counted: {res}"
    assert res.area > 3.9, f"off-board area: {res.area} (want ~4mm^2)"
    print(f"  PASS: OoB grader: {res}")


def test_state_legality_metrics_are_zero_on_a_legal_placement():
    body = (_part('U1', 150.0, 100.0, 'F.Cu', 1, 'NA', cy=4.0)
            + _part('C1', 150.0, 100.0, 'B.Cu', 2, 'NB', cy=1.0))
    st, path = _state(body)
    m = st.legality_metrics()
    # Legality keys only -- `hpwl` rides along in the same dict but is a quality
    # number, not a legality one, so it is not expected to be zero.
    legality_keys = {k: m[k] for k in
                     ('overlap_area', 'oob_count', 'oob_amount', 'oob_area')}
    assert legality_keys == {'overlap_area': 0.0, 'oob_count': 0,
                             'oob_amount': 0.0, 'oob_area': 0.0}, \
        f"cross-side pair graded as illegal: {m}"
    os.unlink(path)


def test_state_legality_metrics_report_a_real_overlap():
    body = (_part('U1', 150.0, 100.0, 'F.Cu', 1, 'NA', cy=4.0)
            + _part('C1', 150.0, 100.0, 'F.Cu', 2, 'NB', cy=1.0))
    st, path = _state(body)
    m = st.legality_metrics()
    # C1's 2x2mm courtyard sits entirely inside U1's 8x8mm one.
    assert abs(m['overlap_area'] - 4.0) < 1e-6, f"overlap_area: {m}"
    os.unlink(path)


# --- #504 follow-up: defects in the merged #456 far-side model ----------------

def test_far_side_box_is_not_transposed_on_a_rotated_footprint():
    """D1: size_x/size_y are BOARD-axis resolved (kicad_parser swaps them for a
    pad at ~90 degrees), so using them as LOCAL half-extents transposed the box
    on every 90/270-degree footprint -- kit-dev SW_ONOFF201 was modelled
    2.54 x 13.97 where the truth is 3.81 x 12.70, under-blocking 1.27mm on the
    far side. The box must be projected through the pad's local tilt, exactly as
    placement/utility.compute_footprint_bbox_local does."""
    from placement.legality import through_pad_bounds_local
    board = os.path.join(KICAD_FILES, 'kit-dev-coldfire-xilinx_5213.kicad_pcb')
    if not os.path.exists(board):
        return
    pcb = parse_kicad_pcb(board)
    checked = 0
    for ref, fp in pcb.footprints.items():
        drilled = [p for p in fp.pads if (p.drill or 0) > 0]
        if not drilled:
            continue
        b = through_pad_bounds_local(fp)
        xs, ys = [], []
        for p in drilled:
            sx, sy = p.size_x, p.size_y
            if abs((fp.rotation % 180) - 90) < 1.0:   # board->local axis swap
                sx, sy = sy, sx
            rx, ry = max((p.drill or 0) / 2, sx / 2), max((p.drill or 0) / 2, sy / 2)
            xs += [p.local_x - rx, p.local_x + rx]
            ys += [p.local_y - ry, p.local_y + ry]
        assert abs((b[2] - b[0]) - (max(xs) - min(xs))) < 1e-6,             f"{ref} (rot {fp.rotation}): width {b[2]-b[0]:.3f} vs {max(xs)-min(xs):.3f}"
        assert abs((b[3] - b[1]) - (max(ys) - min(ys))) < 1e-6,             f"{ref} (rot {fp.rotation}): height {b[3]-b[1]:.3f} vs {max(ys)-min(ys):.3f}"
        checked += 1
    assert checked > 5, f"only checked {checked} THT footprints"
    print(f"  PASS: {checked} THT footprints, no transposed far-side box")


def test_far_side_box_sits_on_the_hole_not_offset_from_it():
    """D2: local_x/local_y IS the hole for a drilled pad -- kicad_parser records
    hole_x/hole_y BEFORE shifting global_x/global_y to the copper centre. The
    old code 'corrected' by (hole - global), which is MINUS the offset, landing
    the box a full offset on the wrong side."""
    from placement.legality import through_pad_bounds_local

    class _P:
        shape = 'circle'
        rect_rotation = 0.0
        def __init__(s):
            s.local_x, s.local_y = 0.0, 0.0      # anchor == hole, local frame
            s.size_x = s.size_y = 1.0
            s.drill = 0.8
            s.global_x, s.global_y = 10.3, 20.0  # copper, shifted +0.3 in x
            s.hole_x, s.hole_y = 10.0, 20.0      # anchor in board coords

    class _F:
        rotation = 0.0
        pads = [_P()]

    b = through_pad_bounds_local(_F())
    cx, cy = (b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0
    assert abs(cx) < 1e-9 and abs(cy) < 1e-9,         f"far-side box centred at ({cx}, {cy}); the hole is at the local origin"


def test_diagonal_near_miss_agrees_between_the_gate_and_the_grader():
    """D4: candidate_valid's SMD fast path used the per-axis test as the VERDICT
    while violation_parts falls through to the EUCLIDEAN gap. Two courtyards
    offset diagonally by (0.2, 0.2) at clearance 0.25 have a true gap of 0.283 --
    legal -- but failed every axis, so violation()==0 while candidate_valid()
    said False."""
    body = (_part('U1', 150.0, 100.0, 'F.Cu', 1, 'NA', cy=1.0)
            + _part('C1', 165.0, 100.0, 'F.Cu', 2, 'NB', cy=1.0))
    st, path = _state(body)
    x = y = None
    x, y = 150.0 + 2.0 + 0.2, 100.0 + 2.0 + 0.2      # dx = dy = 0.2 past U1
    gap = legality.rect_gap(st.parts['C1'].rect(x, y, 0.0), st.parts['U1'].rect())
    assert gap > st.clearance, f"fixture error: gap {gap} <= clearance"
    assert st.violation('C1', x, y, 0.0) == 0.0, "grader should call this legal"
    assert st.candidate_valid('C1', x, y, 0.0),         "the hard gate must agree with the grader on a diagonal near-miss"
    # ... and a genuine same-side overlap is still rejected.
    assert not st.candidate_valid('C1', 150.0, 100.0, 0.0)
    os.unlink(path)


TESTS = [
    test_back_side_part_under_front_part_is_not_a_collision,
    test_same_side_overlap_is_still_a_collision,
    test_cross_side_part_is_no_longer_frozen,
    test_through_hole_lead_field_blocks_the_far_side,
    test_two_smd_parts_on_the_same_side_of_a_tht_neighbour,
    test_notch_in_the_outline_is_rejected,
    test_rectangular_board_leaves_the_gate_inert,
    test_real_boards_exercise_the_outline_gate,
    test_gate_falls_back_to_bbox_without_rings,
    test_illegally_seeded_part_can_still_move,
    test_overlapping_part_gets_no_licence_to_slide,
    test_offboard_part_may_not_slide_into_an_overlap,
    test_legal_part_is_not_allowed_to_become_illegal,
    test_overlap_area_grader_is_side_aware,
    test_out_of_board_grader,
    test_state_legality_metrics_are_zero_on_a_legal_placement,
    test_state_legality_metrics_report_a_real_overlap,
    test_far_side_box_is_not_transposed_on_a_rotated_footprint,
    test_far_side_box_sits_on_the_hole_not_offset_from_it,
    test_diagonal_near_miss_agrees_between_the_gate_and_the_grader,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")

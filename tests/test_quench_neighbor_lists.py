"""
Tests for the quench pruned neighbour lists (issue #430).

candidate_valid / part_geometry_cost used to scan every part for every
candidate pose (O(parts) per probe, O(parts^2) per pass). The fix adds
QuenchState.build_neighbor_lists(travel_budget): per-movable-ref lists built
from union-of-bounds_by_rot seed rectangles, keeping only pairs whose
axis-wise seed gap is within both parts' travel budgets plus the largest
interaction reach (hard clearance / summed halos). Because a movable part can
never leave its seed-centered union box inflated by the budget, excluding a
far pair is EXACT -- pruned pairs can neither collide nor contribute halo
penalty -- so consumers must return bit-identical results with and without
the lists. quench() builds the lists with budget max_displacement+grid_step.

CROSS-PROCESS quench output is nondeterministic (pre-existing: _net_points
iterates a set, so ordering follows PYTHONHASHSEED); every equality
comparison here is same-process by construction.
"""

import math
import os
import random
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest

from kicad_parser import parse_kicad_pcb
from placement.quench import ROTATIONS, QuenchState, quench

ORANGECRAB = os.path.join(os.path.dirname(__file__), '..', 'kicad_files',
                          'orangecrab_ext_pll.kicad_pcb')
INTERF_U = os.path.join(os.path.dirname(__file__), '..', 'kicad_files',
                        'interf_u_unrouted.kicad_pcb')

# The nine QuenchState knob arguments, set to the quench() defaults.
KNOBS = dict(clearance=0.25, board_edge_clearance=0.55, crossing_penalty=10.0,
             halo_base=0.5, halo_coef=0.25, halo_weight=2.0,
             edge_halo=2.0, edge_weight=2.0, grid_step=0.1)


def test_neighbor_list_parity():
    """Pruned and brute-force consumers are bit-identical (the pruning is
    exact, not approximate): for sampled refs x offsets x rotations, a state
    with built neighbour lists and an unbuilt twin agree exactly on
    candidate_valid and part_geometry_cost. Skipped pairs contribute exactly
    +0.0 in the same summation order, so `==` on the floats is the right
    assertion, not approx."""
    pcb = parse_kicad_pcb(ORANGECRAB)
    built = QuenchState(pcb, ORANGECRAB, **KNOBS)
    plain = QuenchState(pcb, ORANGECRAB, **KNOBS)
    assert built._neighbors is None, "lists must be opt-in (None until built)"
    # quench(max_displacement=10.0) would build with 10.0 + grid_step
    built.build_neighbor_lists(10.0 + 0.1)
    assert plain._neighbors is None

    movable = [r for r, p in built.parts.items() if not p.locked]
    random.seed(430)
    sample = random.sample(movable, min(12, len(movable)))

    # Poses stay within the travel budget (max offset hypot = 10 <= 10.1),
    # which is the regime the exactness guarantee covers.
    offsets = [(0.0, 0.0), (10.0, 0.0), (-10.0, 0.0),
               (0.0, 10.0), (0.0, -10.0), (7.0, 7.0)]
    nonzero_costs = 0
    for ref in sample:
        part = built.parts[ref]
        for dx, dy in offsets:
            x, y = part.seed_x + dx, part.seed_y + dy
            for rot in ROTATIONS:
                assert (built.candidate_valid(ref, x, y, rot)
                        == plain.candidate_valid(ref, x, y, rot)), \
                    f"candidate_valid diverged: {ref} at +({dx},{dy}) rot={rot}"
                a = built.part_geometry_cost(ref, x, y, rot)
                b = plain.part_geometry_cost(ref, x, y, rot)
                assert a == b, (f"part_geometry_cost diverged: {ref} at "
                                f"+({dx},{dy}) rot={rot}: {a!r} != {b!r}")
                if a > 0.0:
                    nonzero_costs += 1
        # exclude= must thread through both code paths identically
        if built._neighbors[ref]:
            excl = {built._neighbors[ref][0]}
            assert (built.part_geometry_cost(ref, exclude=excl)
                    == plain.part_geometry_cost(ref, exclude=excl)), \
                f"part_geometry_cost(exclude={excl}) diverged for {ref}"
            assert (built.candidate_valid(ref, part.seed_x, part.seed_y,
                                          part.rot, exclude=excl)
                    == plain.candidate_valid(ref, part.seed_x, part.seed_y,
                                             part.rot, exclude=excl)), \
                f"candidate_valid(exclude={excl}) diverged for {ref}"
    assert nonzero_costs > 0, \
        "vacuous fixture: no pose produced a nonzero geometry cost"

    # The lists must actually prune something, or the perf fix is a no-op.
    assert min(len(v) for v in built._neighbors.values()) \
        < len(built.parts) - 1, "neighbour lists pruned nothing"


def test_quench_output_unchanged_by_pruning(monkeypatch):
    """Same-process A/B: a full quench run with the neighbour lists vs one
    with build_neighbor_lists no-opped (leaving _neighbors None, i.e. the
    brute-force fallback) must return the exact same placements. Each run
    parses the board fresh for isolation."""
    pcb1 = parse_kicad_pcb(INTERF_U)
    pruned = quench(pcb1, INTERF_U, max_displacement=2, step=1.0, max_passes=2)

    monkeypatch.setattr(QuenchState, 'build_neighbor_lists',
                        lambda self, budget: None)
    pcb2 = parse_kicad_pcb(INTERF_U)
    brute = quench(pcb2, INTERF_U, max_displacement=2, step=1.0, max_passes=2)

    assert pruned, "fixture should produce at least one improving move"
    assert pruned == brute, (
        f"pruning changed the quench result:\n  with lists: {pruned}\n"
        f"  brute force: {brute}")


# ---- rotation-fallback coverage board (test 3) -----------------------------
#
# R1 is an elongated 2-pad part (pads 8mm apart, 1x1mm -> 0-degree local bbox
# 9.0 x 1.0mm, half-extents 4.5 x 0.5) SEEDED AT 45 DEGREES, so _Part adds a
# bounds_by_rot[45] entry (half-extents (4.5+0.5)/sqrt(2) = 3.5355 square).
# R2 is a small 2-pad part (union half-extents 0.8) placed R2_DY straight
# below R1. Discriminating geometry, with both parts movable at
# BUDGET = 1.0 each and margin
#   m = 1.0 + 1.0 + max(clearance=0.25, halo_R1 + halo_R2) + 1e-9
#     = 2.0 + 2 * (0.5 + 0.25 * sqrt(2)) + 1e-9          = 3.7071,
# the center-to-center y reach against R2's union box (0.8) is:
#   R1 0-degree box  (y half 0.5)   : 0.5    + 0.8 + m = 5.007
#   R1 45-degree box (y half 3.5355): 3.5355 + 0.8 + m = 8.043
#   R1 union box     (y half 4.5)   : 4.5    + 0.8 + m = 9.007
# R2_DY = 6.5 sits BETWEEN 5.007 and 8.043: a (buggy) 0-degree-only seed box
# would prune the pair, while the 45-degree entry alone -- and hence the
# union the implementation takes over bounds_by_rot -- keeps it. (For this
# elongated part the 90-degree entries also reach 6.5; the union covers
# every entry, and the hand check below contrasts box0 vs 45 vs union.)
R1_XY = (30.0, 30.0)
R1_ROT = 45.0
R1_PAD_HALF_SPAN = 4.0   # pads at local (+-4, 0), size 1x1
R2_DY = 6.5
BUDGET = 1.0
# Parity probe within BUDGET of the seed (0.9 <= 1.0) that moves R1 close
# enough for its halo to reach R2 (gap 1.66 < summed halos 1.707): the cost
# is nonzero ONLY if R2 survived pruning, so parity here proves membership
# behaviorally, not just structurally.
R1_PROBE_NEAR_R2 = (30.0, 30.9)


def _rot45_board():
    """60x60mm board, R1 at 45 degrees + R2 below it (see block comment).
    Writes the board to a temp file, returns (pcb_data, path)."""
    body = '(kicad_pcb\n\t(version 20241229)\n'
    body += '\t(net 0 "")\n\t(net 1 "NA")\n\t(net 2 "NB")\n'
    body += '''\t(gr_rect
\t\t(start 0 0)
\t\t(end 60 60)
\t\t(stroke
\t\t\t(width 0.1)
\t\t\t(type solid)
\t\t)
\t\t(layer "Edge.Cuts")
\t\t(uuid "edge1")
\t)
'''
    # Pad angles repeat the footprint rotation (KiCad writes absolute angles).
    body += f'''\t(footprint "test:LONG2P"
\t\t(layer "F.Cu")
\t\t(uuid "fp-R1")
\t\t(at {R1_XY[0]} {R1_XY[1]} {R1_ROT:.0f})
\t\t(property "Reference" "R1"
\t\t\t(at 0 0)
\t\t)
\t\t(pad "1" smd rect
\t\t\t(at -{R1_PAD_HALF_SPAN} 0 {R1_ROT:.0f})
\t\t\t(size 1 1)
\t\t\t(layers "F.Cu")
\t\t\t(net 1 "NA")
\t\t\t(uuid "p1-R1")
\t\t)
\t\t(pad "2" smd rect
\t\t\t(at {R1_PAD_HALF_SPAN} 0 {R1_ROT:.0f})
\t\t\t(size 1 1)
\t\t\t(layers "F.Cu")
\t\t\t(net 1 "NA")
\t\t\t(uuid "p2-R1")
\t\t)
\t)
'''
    body += f'''\t(footprint "test:SMALL2P"
\t\t(layer "F.Cu")
\t\t(uuid "fp-R2")
\t\t(at {R1_XY[0]} {R1_XY[1] + R2_DY})
\t\t(property "Reference" "R2"
\t\t\t(at 0 0)
\t\t)
\t\t(pad "1" smd rect
\t\t\t(at -0.5 0)
\t\t\t(size 0.6 0.8)
\t\t\t(layers "F.Cu")
\t\t\t(net 2 "NB")
\t\t\t(uuid "p1-R2")
\t\t)
\t\t(pad "2" smd rect
\t\t\t(at 0.5 0)
\t\t\t(size 0.6 0.8)
\t\t\t(layers "F.Cu")
\t\t\t(net 2 "NB")
\t\t\t(uuid "p2-R2")
\t\t)
\t)
'''
    body += ')\n'
    fd, path = tempfile.mkstemp(suffix='.kicad_pcb')
    with os.fdopen(fd, 'w') as f:
        f.write(body)
    return parse_kicad_pcb(path), path


def _seed_box(part, bounds):
    return (part.seed_x + bounds[0], part.seed_y + bounds[1],
            part.seed_x + bounds[2], part.seed_y + bounds[3])


def _margin_overlaps(ra, rb, m):
    """The exact axis-wise inclusion test build_neighbor_lists applies."""
    return (ra[2] + m >= rb[0] and rb[2] + m >= ra[0]
            and ra[3] + m >= rb[1] and rb[3] + m >= ra[1])


def test_neighbor_lists_cover_rotation_fallback():
    """White-box guard: the seed rectangles must be the union over ALL
    bounds_by_rot entries (including the non-90-degree seed rotation _Part
    records), never just the 0-degree box -- otherwise a 45-degree-seeded
    part's true footprint sticks out of its pruning box and real neighbours
    get dropped."""
    pcb, path = _rot45_board()
    state = QuenchState(pcb, path, **KNOBS)
    r1, r2 = state.parts['R1'], state.parts['R2']
    assert r1.rot == 45.0
    assert 45.0 in r1.bounds_by_rot, "_Part must record the non-90 seed entry"

    # Hand-computed margin test with the implementation's own numbers:
    # the 0-degree-only box would prune the pair, the 45-degree entry (and
    # so the union) keeps it. This proves R2_DY genuinely discriminates.
    m = BUDGET + BUDGET + max(state.clearance, r1.halo + r2.halo) + 1e-9
    r2_union = _seed_box(r2, (
        min(b[0] for b in r2.bounds_by_rot.values()),
        min(b[1] for b in r2.bounds_by_rot.values()),
        max(b[2] for b in r2.bounds_by_rot.values()),
        max(b[3] for b in r2.bounds_by_rot.values())))
    r1_box0 = _seed_box(r1, r1.bounds_by_rot[0.0])
    r1_box45 = _seed_box(r1, r1.bounds_by_rot[45.0])
    r1_union = _seed_box(r1, (
        min(b[0] for b in r1.bounds_by_rot.values()),
        min(b[1] for b in r1.bounds_by_rot.values()),
        max(b[2] for b in r1.bounds_by_rot.values()),
        max(b[3] for b in r1.bounds_by_rot.values())))
    assert not _margin_overlaps(r1_box0, r2_union, m), \
        "geometry no longer discriminates: 0-degree box already reaches R2"
    assert _margin_overlaps(r1_box45, r2_union, m), \
        "geometry no longer discriminates: 45-degree box misses R2"
    assert _margin_overlaps(r1_union, r2_union, m)

    state.build_neighbor_lists(BUDGET)
    assert 'R2' in state._neighbors['R1'], \
        "45-degree-seeded R1 lost its real neighbour R2 (0-degree-only box?)"
    assert 'R1' in state._neighbors['R2'], \
        "R2 lost its real neighbour R1 (0-degree-only box?)"

    # Behaviour parity at the 45-degree pose against an unbuilt twin,
    # including a probe where R1's halo actually reaches R2: a nonzero,
    # bit-identical cost is only possible if the pair survived pruning.
    twin = QuenchState(pcb, path, **KNOBS)
    assert twin._neighbors is None
    poses = [(R1_XY[0], R1_XY[1], R1_ROT),
             (R1_PROBE_NEAR_R2[0], R1_PROBE_NEAR_R2[1], R1_ROT)]
    for x, y, rot in poses:
        assert (state.part_geometry_cost('R1', x, y, rot)
                == twin.part_geometry_cost('R1', x, y, rot)), (x, y, rot)
        assert (state.candidate_valid('R1', x, y, rot)
                == twin.candidate_valid('R1', x, y, rot)), (x, y, rot)
    assert state.part_geometry_cost('R1', *R1_PROBE_NEAR_R2, R1_ROT) > 0.0, \
        "probe pose should incur a halo penalty against R2"

    os.unlink(path)


if __name__ == '__main__':
    test_neighbor_list_parity()
    mp = pytest.MonkeyPatch()
    try:
        test_quench_output_unchanged_by_pruning(mp)
    finally:
        mp.undo()
    test_neighbor_lists_cover_rotation_fallback()
    print("ALL PASS")

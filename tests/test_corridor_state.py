#!/usr/bin/env python3
"""The corridor cut as a QuenchState term, on a real board.

`test_corridor_cut.py` proves the clipper. These are the three claims that make
it safe to ship inside an optimizer:

  1. OFF is bit-identical. The flag defaults to 0.0, and at 0.0 no corridor is
     ever built, so the objective is the one that shipped.
  2. The corridors are FROZEN. `_cluster_ends` reads live pad positions, so a
     corridor rebuilt after a move would make the cost of a pose depend on when
     it was evaluated -- an accepted gain would not match the recomputed total
     and the descent could cycle. This asserts the rectangles are the same
     objects after moves that would certainly have moved them.
  3. The term is not vacuous: on a board with a declared bus it is nonzero and
     it responds to a move, or it would be measuring nothing at real cost.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # placement split

from kicad_parser import parse_kicad_pcb                       # noqa: E402
from placement.quench import QuenchState                       # noqa: E402
import routing_defaults as defaults                            # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ULX = os.path.join(ROOT, 'kicad_files', 'ulx3s.kicad_pcb')
SPEC = [{'name': 'sdram', 'nets': ['SDRAM_*'], 'width_mm': 8.0}]

_PCB = {}


def _pcb(path=ULX):
    if path not in _PCB:
        _PCB[path] = parse_kicad_pcb(path)
    return _PCB[path]


def _state(path=ULX, **kw):
    return QuenchState(_pcb(path), path, clearance=defaults.CLEARANCE,
                       board_edge_clearance=0.55, crossing_penalty=10.0,
                       halo_base=0.5, halo_coef=0.25, halo_weight=2.0,
                       edge_halo=2.0, edge_weight=2.0,
                       grid_step=defaults.GRID_STEP, length_weight=1.0, **kw)


def test_off_by_default_builds_no_corridor_at_all():
    """Not 'builds them and multiplies by zero' -- never builds them. That is
    what makes OFF free as well as identical."""
    st = _state()
    assert st.corridor_weight == 0.0
    assert st._corridor_boxes == []
    # ... and specs without a weight stay unbuilt too, so passing an intent
    # around cannot silently switch the objective on.
    st2 = _state(corridor_specs=SPEC)
    assert st2._corridor_boxes == [], "specs alone must not arm the term"
    print("  PASS: no weight => no corridor built, even with specs present")


def test_off_is_bit_identical_to_the_shipped_objective():
    off = _state().total_cost()
    armed = _state(corridor_weight=0.0, corridor_specs=SPEC).total_cost()
    for k in ('total', 'length', 'crossings', 'halo', 'edge', 'hpwl'):
        assert off[k] == armed[k], (k, off[k], armed[k])
    assert off['corridor_cut'] == 0.0
    print("  PASS: every cost term bit-identical with the flag at 0.0")


def test_the_term_is_nonzero_on_a_board_with_a_declared_bus():
    st = _state(corridor_weight=1.0, corridor_specs=SPEC)
    assert st._corridor_boxes, "the sdram corridor must resolve on ulx3s"
    cost = st.total_cost()
    assert cost['corridor_cut'] > 0.0, "foreign nets do cut the sdram lane"
    print(f"  PASS: sdram corridor cut = {cost['corridor_cut']:.1f}mm "
          f"over {len(st._corridor_boxes)} corridor(s)")


def test_the_weight_scales_the_total_and_nothing_else():
    base = _state(corridor_weight=1.0, corridor_specs=SPEC).total_cost()
    tenx = _state(corridor_weight=10.0, corridor_specs=SPEC).total_cost()
    assert abs(base['corridor_cut'] - tenx['corridor_cut']) < 1e-9, \
        "the measured cut is a geometric fact; the weight is the price"
    want = base['total'] + 9.0 * base['corridor_cut']
    assert abs(tenx['total'] - want) < 1e-6, (tenx['total'], want)
    print("  PASS: the weight prices the cut; it does not change it")


def test_corridors_are_frozen_across_moves():
    """The load-bearing invariant. Move the parts that DEFINE the corridor's
    endpoints and the rectangle must not follow them."""
    st = _state(corridor_weight=1.0, corridor_specs=SPEC)
    before = [(b.ax, b.ay, b.ux, b.uy, b.length, b.half_w)
              for b in st._corridor_boxes]
    # Move every movable part that owns a corridor net -- exactly the parts
    # _cluster_ends derives the endpoints from.
    moved = 0
    own = set()
    for b in st._corridor_boxes:
        own.update(int(n) for n in range(len(b.skip)) if b.skip[n])
    for ref in sorted(st.parts):
        part = st.parts[ref]
        if part.locked or not (set(part.nets) & own):
            continue
        st.apply_move(ref, part.x + 3.0, part.y + 3.0, part.rot)
        moved += 1
        if moved >= 12:
            break
    assert moved, "the fixture must actually move corridor-owning parts"
    after = [(b.ax, b.ay, b.ux, b.uy, b.length, b.half_w)
             for b in st._corridor_boxes]
    assert before == after, "a corridor followed its parts -- objective is now "\
                            "non-stationary"
    print(f"  PASS: {len(before)} corridor(s) unmoved after {moved} moves")


def test_the_term_responds_to_a_move():
    """Frozen corridors, not a frozen cost. If nothing a part does changes the
    cut, the optimizer cannot act on it and the term is dead weight."""
    st = _state(corridor_weight=1.0, corridor_specs=SPEC)
    start = st.total_cost()['corridor_cut']
    seen = {round(start, 6)}
    for ref in sorted(st.parts):
        part = st.parts[ref]
        if part.locked:
            continue
        st.apply_move(ref, part.x + 8.0, part.y + 8.0, part.rot)
        seen.add(round(st.total_cost()['corridor_cut'], 6))
        if len(seen) > 2:
            break
    assert len(seen) > 1, "the cut never moved -- the term measures nothing"
    print(f"  PASS: the cut takes {len(seen)} distinct values under moves")


def test_a_board_with_no_matching_bus_arms_nothing():
    """A spec whose globs match nothing must leave the objective alone rather
    than build a degenerate rectangle at the origin."""
    st = _state(corridor_weight=5.0,
                corridor_specs=[{'name': 'nope', 'nets': ['zzz_no_such_*'],
                                 'width_mm': 8.0}])
    assert st._corridor_boxes == []
    assert st.total_cost()['corridor_cut'] == 0.0
    print("  PASS: an unmatched corridor spec is inert, not degenerate")


if __name__ == '__main__':
    for fn in (test_off_by_default_builds_no_corridor_at_all,
               test_off_is_bit_identical_to_the_shipped_objective,
               test_the_term_is_nonzero_on_a_board_with_a_declared_bus,
               test_the_weight_scales_the_total_and_nothing_else,
               test_corridors_are_frozen_across_moves,
               test_the_term_responds_to_a_move,
               test_a_board_with_no_matching_bus_arms_nothing):
        print(fn.__name__)
        fn()
    print("\nALL PASS")

#!/usr/bin/env python3
"""`cut_mm` and `cover`: the two corridor numbers worth READING.

`cover` is the honesty check. `corridors_from_intent` builds a confident
rectangle for any set of nets, including a "bus" that is really N identical
LOCAL sub-blocks scattered over the board -- their endpoint centroids average
to the middle of nothing and every signal computed against that rectangle
measures a fiction. Two real cases on the in-repo corpus, both asserted here:

  * splitflap's OUT_* is 24 nets across six identical motor channels. Cover 0.0.
  * ulx3s's SDRAM_* is a REAL bus, but merging its address and data halves under
    one glob drops cover from 0.81 to 0.46, because the two halves leave the
    part on different faces and the average lands between them. This one is the
    dangerous case: the corridor looks plausible.

The last test pins the measured NEGATIVE result about `--corridor-weight`, so
that "the chord has a gradient, let us optimise it" cannot be quietly
reintroduced without the evidence being re-run.
"""
import io
import contextlib
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # placement split

from kicad_parser import parse_kicad_pcb                       # noqa: E402
from placement.quench import QuenchState                       # noqa: E402
from placement.routability import (CORRIDOR_MIN_COVER,          # noqa: E402
                                   corridor_cover, corridor_cut_mm,
                                   corridors_from_intent, foreign_crossings)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ULX = os.path.join(ROOT, 'kicad_files', 'ulx3s.kicad_pcb')
SPLIT = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')

_CACHE = {}


def _state(path):
    if path not in _CACHE:
        pcb = parse_kicad_pcb(path)
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            st = QuenchState(pcb, path, clearance=0.2,
                             board_edge_clearance=0.55)
        _CACHE[path] = (pcb, st)
    return _CACHE[path]


def _corridor(path, spec):
    pcb, st = _state(path)
    cors = corridors_from_intent(st, pcb, [spec])
    return st, (cors[0] if cors else None)


def test_a_real_bus_scores_high_cover():
    st, c = _corridor(ULX, {'name': 'a', 'nets': ['SDRAM_A*'],
                            'width_mm': 8.0})
    assert c is not None, "SDRAM_A* must resolve on ulx3s"
    cover = corridor_cover(st, c)
    assert cover >= CORRIDOR_MIN_COVER, cover
    print(f"  PASS: SDRAM_A* ({len(c.net_ids)} nets) cover {cover:.2f}")


def test_scattered_local_sub_blocks_score_zero_cover():
    """The phantom. Six motor channels, one name, no bus."""
    st, c = _corridor(SPLIT, {'name': 'out', 'nets': ['OUT_*'],
                              'width_mm': 6.0})
    assert c is not None, "the fixture needs the corridor to be BUILT -- the "\
                          "point is that it builds one and it is a fiction"
    cover = corridor_cover(st, c)
    assert cover < CORRIDOR_MIN_COVER, cover
    print(f"  PASS: splitflap OUT_* ({len(c.net_ids)} nets) cover {cover:.2f} "
          "-- built, and correctly disbelieved")


def test_merging_sub_buses_is_caught():
    """The dangerous case: a real bus, merged, still looks plausible."""
    st, split = _corridor(ULX, {'name': 'a', 'nets': ['SDRAM_A*'],
                                'width_mm': 8.0})
    _st2, merged = _corridor(ULX, {'name': 'all', 'nets': ['SDRAM_*'],
                                   'width_mm': 8.0})
    a = corridor_cover(st, split)
    b = corridor_cover(st, merged)
    assert b < a, (a, b)
    assert b < CORRIDOR_MIN_COVER <= a, (a, b)
    print(f"  PASS: SDRAM_A* {a:.2f} vs merged SDRAM_* {b:.2f} -- merging "
          "sub-buses is flagged")


def test_cut_mm_and_the_count_are_not_redundant():
    """If the cut were just a rescaled count there would be no reason to add
    it. It is not: obliqueness and lane width move one and not the other."""
    st, c = _corridor(ULX, {'name': 'a', 'nets': ['SDRAM_A*'],
                            'width_mm': 8.0})
    n, _guilty = foreign_crossings(st, c)
    cut = corridor_cut_mm(st, c)
    assert n > 0 and cut > 0.0
    # A perpendicular crossing of a width-8 lane cuts exactly 8mm, so a pure
    # count would predict cut == 8*n. Real crossings are oblique and clipped.
    assert abs(cut - 8.0 * n) > 1.0, (
        f"cut {cut:.1f} vs 8*count {8.0 * n:.1f}: if these agreed the cut "
        "would be carrying no information the count does not")
    print(f"  PASS: {n} crossings but {cut:.0f}mm cut "
          f"({cut / n:.1f}mm each, vs {c.width_mm}mm if all were square)")


def test_the_cut_is_position_invariant_for_a_traversing_wire():
    """The measured reason --corridor-weight is inert at nudge scale.

    A wire that fully traverses a lane pays w/sin(theta) WHEREVER along the
    lane it crosses. Sliding it along the corridor changes nothing, so a small
    move gives a greedy descent no direction. This is asserted rather than left
    in prose because it is the whole argument against optimising the term, and
    the idea of optimising it is an easy one to have twice.
    """
    import math
    import numpy as np
    from placement.quench import _CorridorBox, _corridor_cut_np
    skip = np.zeros(4, dtype=bool)
    box = _CorridorBox(ax=0.0, ay=0.0, ux=1.0, uy=0.0, length=40.0,
                       half_w=4.0, skip=skip)
    seen = set()
    for x in (5.0, 12.0, 20.0, 28.0, 35.0):       # slide along the lane
        aw = np.asarray([[x, -20.0, x, 20.0, 1]], dtype=float)
        seen.add(round(_corridor_cut_np(aw, [box]), 9))
    assert seen == {8.0}, f"a traversing chord must not depend on x: {seen}"
    # ...but it DOES depend on angle, which is the one axis it can be moved on.
    angles = set()
    for deg in (90.0, 60.0, 40.0):
        th = math.radians(deg)
        h = 60.0
        aw = np.asarray([[20.0 - h * math.cos(th), -h * math.sin(th),
                          20.0 + h * math.cos(th), h * math.sin(th), 1]],
                        dtype=float)
        angles.add(round(_corridor_cut_np(aw, [box]), 6))
    assert len(angles) == 3, angles
    print("  PASS: chord constant across 5 positions, distinct across 3 "
          "angles -- position-invariant, angle-sensitive")


if __name__ == '__main__':
    for fn in (test_a_real_bus_scores_high_cover,
               test_scattered_local_sub_blocks_score_zero_cover,
               test_merging_sub_buses_is_caught,
               test_cut_mm_and_the_count_are_not_redundant,
               test_the_cut_is_position_invariant_for_a_traversing_wire):
        print(fn.__name__)
        fn()
    print("\nALL PASS")

#!/usr/bin/env python3
"""#662 grammar pour: background sheet + hull islands + connectivity invariants.

Pins the engine-side algorithm (`route_planes._grammar_zone_polygons`) that
replaced the pad-Voronoi multi-net layer partition, and the invariant-3b
enforcement added to finish the issue. Nothing else pins any of it: the
caller falls back to Voronoi SILENTLY when the grammar returns None, so a
regression here would be invisible to the rest of the suite.

Covered (all wx-free, board-file-free):

  1. composition: the dominant net (pad spread x consumer count, scored on
     PADS -- seed lists carry MST route samples) owns the layer as a
     background sheet; every other net becomes compact inflated cluster
     hulls, one per single-linkage cluster, each containing its points
     (invariant 3a is by construction: convex hulls are connected and
     cover their cluster).
  2. degenerate inputs return None so the caller reverts to Voronoi.
  3. invariant 3b, severing side: an island that would cut the background
     sheet into two pieces that BOTH hold dominant-net consumers (a
     mid-board full-height wall) is demoted to tracks after the shrink
     retries fail -- the sheet ships connected.
  4. invariant 3b, tolerance side: a full-height island NEAR the board
     edge detaches only a source-less sliver; that is what fill island
     removal culls, NOT a severing, so the island is kept. 3+4 together
     pin the discrimination (dominant-consumer content decides, not mere
     component count).
  5. the raster primitive `_grammar_sheet_ok` directly: wall severs,
     compact box does not.
"""
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.join(_ROOT, 'py_router'))

from route_planes import (_grammar_zone_polygons, _grammar_sheet_raster,
                          _grammar_sheet_ok, _grammar_point_in_poly)

BOUNDS = (0.0, 0.0, 100.0, 100.0)
SHEET = [(2.0, 2.0), (98.0, 2.0), (98.0, 98.0), (2.0, 98.0)]

# Dominant rail: consumers spread over the whole board (6x5 grid, x >= 15
# so the edge-sliver case below stays source-less).
DOM_PADS = [(15.0 + 16.0 * i, 10.0 + 20.0 * j) for i in range(6) for j in range(5)]
DOM_SEEDS = DOM_PADS[:6]  # fewer SEEDS than the minor rail: dominance must
                          # come from the pad set, not the seed list


def cluster(cx, cy, n=5, r=1.5):
    """n points in a tight blob (single-linkage keeps them one cluster)."""
    import math
    return [(cx + r * math.cos(2 * math.pi * k / n),
             cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def build(seeds_by_net, pads_by_net):
    names = {nid: f"net{nid}" for nid in seeds_by_net}
    return _grammar_zone_polygons(seeds_by_net, SHEET, BOUNDS, names,
                                  pads_by_net=pads_by_net, carve_mm=0.8)


def test_composition():
    minor_a = cluster(20.0, 20.0, n=25)        # MORE seeds than the dominant
    minor_b = cluster(70.0, 20.0) + cluster(30.0, 70.0)
    out = build({1: DOM_SEEDS, 2: minor_a, 3: minor_b},
                {1: DOM_PADS, 2: minor_a, 3: minor_b})
    assert out is not None
    assert out[1] == [SHEET], "board-wide pad set must own the background sheet"
    assert len(out[2]) == 1, f"one tight cluster -> one island, got {len(out[2])}"
    assert len(out[3]) == 2, f"two clusters -> two islands, got {len(out[3])}"
    for nid, pts in ((2, minor_a), (3, minor_b)):
        for x, y in pts:
            assert any(_grammar_point_in_poly(x, y, poly) for poly in out[nid]), \
                f"net {nid} point ({x:.1f},{y:.1f}) outside every island (3a)"
    print("  composition: sheet + hull islands, all points covered")


def test_degenerate():
    assert build({1: DOM_SEEDS}, {1: DOM_PADS}) is None, \
        "single seeded net must return None (Voronoi fallback)"
    assert build({1: DOM_SEEDS, 2: []}, {1: DOM_PADS, 2: []}) is None, \
        "one seeded + one empty net must return None"
    print("  degenerate: None -> Voronoi fallback")


def test_severing_wall_demotes():
    # Rail pads in a mid-board full-height column: its hull is a wall that
    # cuts the sheet into left/right halves, both holding dominant pads.
    wall = [(50.0, y) for y in range(4, 97, 4)]  # 4mm pitch < 5mm link: ONE cluster
    keeper = cluster(80.0, 80.0)
    out = build({1: DOM_SEEDS, 4: wall, 5: keeper},
                {1: DOM_PADS, 4: wall, 5: keeper})
    assert out is not None
    assert out[1] == [SHEET]
    assert 4 not in out, \
        f"a sheet-severing wall must demote to tracks (NO zone entry), " \
        f"got {len(out.get(4, []))} island(s)"
    assert len(out[5]) == 1, "an innocent island must survive the wall's demotion"
    print("  invariant 3b: severing wall demoted, innocent island kept")


def test_edge_sliver_tolerated():
    # The same full-height column NEAR the left edge detaches only a thin
    # source-less strip (no dominant pad west of x=15): fill island removal
    # culls that; it is not a severing, so the island must be KEPT.
    edge_col = [(8.0, y) for y in range(4, 97, 4)]
    out = build({1: DOM_SEEDS, 6: edge_col}, {1: DOM_PADS, 6: edge_col})
    assert out is not None
    assert len(out[6]) == 1, \
        "a source-less edge sliver is not a severing; the island must be kept"
    print("  invariant 3b tolerance: source-less edge sliver does not demote")


def test_sheet_ok_primitive():
    raster = _grammar_sheet_raster(SHEET)
    dom_pts = DOM_PADS
    box = [(45.0, 45.0), (55.0, 45.0), (55.0, 55.0), (45.0, 55.0)]
    assert _grammar_sheet_ok(raster, [box], 0.8, dom_pts), \
        "a compact central box must not read as severing"
    wall = [(49.0, 2.0), (51.0, 2.0), (51.0, 98.0), (49.0, 98.0)]
    assert not _grammar_sheet_ok(raster, [wall], 0.8, dom_pts), \
        "a full-height wall must read as severing"
    print("  raster primitive: box ok, wall severs")


def main():
    test_composition()
    test_degenerate()
    test_severing_wall_demotes()
    test_edge_sliver_tolerated()
    test_sheet_ok_primitive()
    print("PASS: grammar pour composition + invariants (#662)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

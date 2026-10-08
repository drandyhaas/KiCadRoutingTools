#!/usr/bin/env python3
"""plane_split_raster: the spine split's raster finishing (route_planes, a layer several nets share).

Pins, wx-free and board-file-free:

  1. the raster primitives: a polygon with a hole rasterises to its area within a cell's rounding; polygonise
     gives it back with the hole; cells_of covers a box, a disc and a CAPSULE (a milled slot) as their areas.
  2. the human shapes: octagon() is the chamfered rectangle round its points (every edge at a multiple of 45
     degrees, every point inside, the margin on every side); mitred_band() keeps a spine's 0/45/90 bends; a
     rasterised diamond straightens back to its four edges.
  3. the fill model's PRIORITY: a region pulls back from the zones that outrank it, never from the background
     sheet beneath it. A region whose two blobs are joined by a neck narrower than two clearance bands keeps the
     far blob -- modelled as yielding to the background as well, the neck would close and the far blob, holding
     none of the net's anchors, would be handed away.
  4. a piece nothing feeds goes to the neighbour whose fill carries it on to one of its own anchors: a region cut
     in two by a milled slot hands the anchor-less half to the region beside it.
  5. what no neighbour can feed goes back to the background, never kept in a region's outline: the same slot with
     no region beside it; and a board CUTOUT does the slot's work (the board's real shape, not the zone outline).
  6. an offer the neighbour cannot feed either goes back to the background in the same pass.
  7. finish() is deterministic: the same inputs give the same polygons.
"""
import math
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, 'py_router'))

from shapely.geometry import Point, Polygon, box  # noqa: E402

import plane_split_raster as psr  # noqa: E402

DOM, R1, R2 = 1, 2, 3
OUTLINE = box(0.0, 0.0, 40.0, 20.0)
ZC, MT, EDGE = 0.2, 0.2, 0.3


def octilinear_edges(poly):
    """Every edge of the polygon's exterior at a multiple of 45 degrees"""
    pts = list(poly.exterior.coords)
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        ang = math.degrees(math.atan2(by - ay, bx - ax)) % 45.0
        if min(ang, 45.0 - ang) > 0.5:
            return False
    return True


def covers(polys, x, y):
    return any(p.contains(Point(x, y)) for p in polys)


def run(labels, anchors, holes=(), board=None, copper=(), rounds=4):
    grid = psr.RasterGrid(OUTLINE.bounds, 0.05)
    return psr.finish(labels, DOM, OUTLINE, grid, anchors, list(copper), list(holes), ZC, MT, EDGE, board=board,
                      rounds=rounds)


def test_primitives():
    g = psr.RasterGrid((0, 0, 50, 40), 0.1)
    ring = box(5, 5, 45, 35).difference(Point(25, 20).buffer(8, 64))
    m = g.rasterize(ring)
    assert abs(m.sum() * 0.01 - ring.area) < 0.01 * ring.area, (m.sum() * 0.01, ring.area)
    back = g.polygonize(m)
    assert len(back) == 1 and len(back[0].interiors) == 1, "the hole must survive polygonise"
    assert abs(back[0].area - m.sum() * 0.01) < 1e-6
    for kind, shp, area in (('box', (1, 1, 3, 2), 2.0), ('disc', (10, 10, 1.0), math.pi),
                            ('capsule', (20, 10, 26, 10, 0.5), 6 * 1.0 + math.pi * 0.25)):
        sl, local = g.cells_of(kind, shp)
        got = local.sum() * 0.01
        assert abs(got - area) < 0.12 * area, (kind, got, area)
    print("  primitives: rasterise / polygonise / box, disc, capsule cells")


def test_human_shapes():
    pts = [(0, 0), (10, 0), (10, 4), (0, 4), (5, 8)]
    o = psr.octagon(pts, 2.0)
    assert octilinear_edges(o), "an octagon's edges are at multiples of 45 degrees"
    assert all(o.contains(Point(x, y)) for x, y in pts)
    assert o.exterior.distance(Point(5, 2)) >= 2.0 - 1e-9, "the margin keeps every side 2 mm out"
    band = psr.mitred_band([(0, 0), (10, 0), (15, 5)], 1.5)
    assert octilinear_edges(band), "a corridor along a 0/45 spine keeps 0/45 edges"
    g = psr.RasterGrid((0, 0, 50, 40), 0.1)
    dia = g.polygonize(g.rasterize(Polygon([(25, 2), (45, 20), (25, 38), (5, 20)])))[0]
    s = psr.octilinear(dia, 0.1)
    assert len(s.exterior.coords) == 5, f"a rasterised diamond straightens to 4 edges, got {len(s.exterior.coords) - 1}"
    print("  human shapes: octagon, mitred corridor, octilinear outline")


def test_priority_neck():
    # R1: blob A (its anchor) and blob B (none) joined by a 0.4 mm neck -- narrower than the two 0.2 mm clearance
    # bands a both-sides model would cut, wider than the 0.2 mm minimum width
    neck = box(5, 5, 10, 15).union(box(10, 9.8, 14, 10.2)).union(box(14, 5, 19, 15))
    polys, stats = run({R1: neck}, {DOM: [(35.0, 10.0)], R1: [(7.5, 10.0)]})
    assert covers(polys[R1], 16.5, 10.0), "the far blob is fed through the neck: it stays the region's"
    assert stats['dead_pieces'] == 0, stats
    print("  priority: a region does not pull back from the background sheet")


def test_dead_piece_to_neighbour():
    # a milled slot (a capsule hole) cuts R1 in two; the right half holds none of R1's anchors and borders R2
    slot = [('capsule', (14.0, 4.0, 14.0, 16.0, 0.3 + EDGE))]
    labels = {R1: box(5, 5, 16, 15), R2: box(16, 5, 26, 15)}
    polys, stats = run(labels, {DOM: [(35.0, 10.0)], R1: [(8.0, 10.0)], R2: [(21.0, 10.0)]}, holes=slot)
    assert covers(polys[R2], 15.2, 10.0), "R1's fed-by-nothing half goes to R2, whose fill carries it on"
    assert not covers(polys[R1], 15.2, 10.0)
    assert stats['reassigned_cells'] > 0, stats
    print("  dead piece: given to the neighbour that can feed it")


def test_dead_piece_to_background():
    slot = [('capsule', (14.0, 4.0, 14.0, 16.0, 0.3 + EDGE))]
    polys, stats = run({R1: box(5, 5, 16, 15)}, {DOM: [(35.0, 10.0)], R1: [(8.0, 10.0)]}, holes=slot)
    assert not covers(polys[R1], 15.2, 10.0), "no neighbour feeds it: back to the background, not the region"
    assert stats['background_cells'] > 0, stats
    # a board CUTOUT does the slot's work: the fill model reads the board's real shape
    board = OUTLINE.difference(box(13.8, 4.0, 14.2, 16.0))
    polys, stats = run({R1: box(5, 5, 16, 15)}, {DOM: [(35.0, 10.0)], R1: [(8.0, 10.0)]}, board=board)
    assert not covers(polys[R1], 15.2, 10.0), "a board cutout cuts the fill as a slot does"
    print("  dead piece: no neighbour feeds it -> the background; a board cutout cuts the fill")


def test_unfed_offer_returns_to_background():
    # both regions cut by a slot: R1's anchor-less right half is offered to R2 (the longest border), but R2's own
    # band beside it is cut from R2's anchor too -- R2 cannot feed it, so it goes back to the background, in ONE pass
    # (no later pass gets to repair a piece left in an outline that cannot feed it)
    slots = [('capsule', (14.0, 4.0, 14.0, 16.0, 0.3 + EDGE)), ('capsule', (18.0, 4.0, 18.0, 16.0, 0.3 + EDGE))]
    labels = {R1: box(5, 5, 16, 15), R2: box(16, 5, 26, 15)}
    polys, stats = run(labels, {DOM: [(35.0, 10.0)], R1: [(8.0, 10.0)], R2: [(24.0, 10.0)]}, holes=slots, rounds=1)
    for nid in (R1, R2):
        assert not covers(polys.get(nid, []), 15.2, 10.0), \
            f"net {nid} keeps a piece nothing feeds: an unfed offer must go back to the background"
    print("  unfed offer: back to the background in the same pass")


def test_deterministic():
    labels = {R1: box(5, 5, 16, 15), R2: box(16, 5, 26, 15)}
    anchors = {DOM: [(35.0, 10.0)], R1: [(8.0, 10.0)], R2: [(21.0, 10.0)]}
    a, _ = run(labels, anchors)
    b, _ = run(labels, anchors)
    assert {n: [p.wkt for p in ps] for n, ps in a.items()} == {n: [p.wkt for p in ps] for n, ps in b.items()}
    print("  deterministic")


def main():
    test_primitives()
    test_human_shapes()
    test_priority_neck()
    test_dead_piece_to_neighbour()
    test_dead_piece_to_background()
    test_unfed_offer_returns_to_background()
    test_deterministic()
    print("PASS: plane_split_raster finishing")
    return 0


if __name__ == "__main__":
    sys.exit(main())

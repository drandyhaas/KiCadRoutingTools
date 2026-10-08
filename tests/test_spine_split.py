#!/usr/bin/env python3
"""The spine split (route_planes): a plane layer several nets share, split round spines routed on it.

Pins the route_planes side (the raster finishing is tests/test_plane_split_raster.py), wx-free:

  1. ONE layer model (_layer_geometry) for the spine router, the background check and the finishing: a round pad a
     disc, a tilted rect pad its bounding box, an NPTH round hole kept the zone clearance, an NPTH SLOT the
     board-edge clearance (as KiCad grades a slot, #448), the board its outline less its Edge.Cuts cutouts.
  2. _blockers_of: every other net's copper and every hole, never the net's own pads.
  3. _plane_pad_cells: each cell carries its net (a hole's: 0), so a net leaves out only its own copper.
  4. _spine_samples: a seed every quarter of the distance to another net's nearest seed (a straight boundary
     beside a pad, no sawtooth), the plain interval far from everything.
  5. _background_net: the board-wide rail with the most consumers.
  6. _reach_and_sheet: a net's two groups (octagons) joined by its spine (a corridor) into ONE region covering every
     anchor; the background the whole sheet. A corridor that would cut the background into two pieces that both
     hold its anchors is REFUSED (#662 invariant 3b). Two parallel corridors a little apart run together into one
     band. A background anchor inside another net's region keeps a pocket of its own.
  7. voronoi_cells: every seed's cell by its net, clipped to the bounds; fewer than two seeds is an error (the
     caller then pours the full outline for its first net).
  8. end to end: route_planes.py on an in-repo board pours a shared layer through the split (KiCad-free).
"""
import math
import os
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, 'py_router'))

from shapely.geometry import Point, box  # noqa: E402
from scipy.spatial import cKDTree  # noqa: E402

import route_planes as rp  # noqa: E402
from plane_zone_geometry import voronoi_cells  # noqa: E402
from routing_config import GridCoord  # noqa: E402

ZC, EDGE, MT = 0.2, 0.5, 0.2


def pad(net, x, y, sx, sy, layers=('F.Cu',), ptype='smd', drill=0.0, shape='rect', rect_rotation=0.0,
        drill_w=0.0, drill_h=0.0, rotation=0.0):
    return NS(net_id=net, global_x=x, global_y=y, size_x=sx, size_y=sy, layers=list(layers), pad_type=ptype,
              drill=drill, shape=shape, rect_rotation=rect_rotation, drill_w=drill_w, drill_h=drill_h,
              rotation=rotation, hole_x=None, hole_y=None)


def test_layer_geometry():
    pads = [pad(5, 10, 10, 1.0, 1.0, shape='circle'),
            pad(6, 20, 10, 2.0, 1.0, rect_rotation=45.0),
            pad(0, 30, 10, 3.0, 3.0, layers=('*.Cu',), ptype='np_thru_hole', drill=3.0),
            pad(0, 40, 10, 4.0, 2.0, layers=('*.Cu',), ptype='np_thru_hole', drill=4.0, drill_w=4.0, drill_h=2.0),
            pad(7, 50, 10, 1.0, 1.0, layers=('B.Cu',))]            # not on F.Cu: not in the model
    pcb = NS(footprints={'U1': NS(pads=pads)}, vias=[NS(net_id=5, x=60, y=10, size=0.5)],
             board_info=NS(board_outlines=[[(0, 0), (100, 0), (100, 50), (0, 50)]],
                           board_cutouts=[[(70, 20), (80, 20), (80, 30), (70, 30)]], board_bounds=(0, 0, 100, 50)))
    geom = rp._layer_geometry(pcb, 'F.Cu', ZC, EDGE)
    kinds = {(n, k) for n, k, _ in geom['copper']}
    assert (5, 'disc') in kinds and (6, 'box') in kinds and not any(n == 7 for n, _, _ in geom['copper'])
    tilted = next(s for n, k, s in geom['copper'] if n == 6)
    half = (2.0 / 2) * math.cos(math.radians(45)) + (1.0 / 2) * math.sin(math.radians(45))
    assert abs((tilted[2] - tilted[0]) / 2 - half) < 1e-9, "a tilted pad is modelled by its bounding box"
    clr = sorted(c for _, _, c in geom['holes'])
    assert clr == [ZC, EDGE], f"a round NPTH keeps the zone clearance, a slot the edge clearance: {clr}"
    assert not geom['board'].contains(Point(75, 25)) and geom['board'].contains(Point(50, 25)), \
        "the board is its outline less its cutouts"
    plain = NS(footprints={}, vias=[], board_info=NS(board_outlines=[], board_cutouts=[], board_bounds=(0, 0, 9, 9)))
    assert abs(rp._layer_geometry(plain, 'F.Cu', ZC, EDGE)['board'].area - 81.0) < 1e-9
    # 2. what a fill of net 5 cannot cross: net 6's pad and both holes -- not its own pad or via
    bl = rp._blockers_of(geom, 5)
    assert bl.contains(Point(20, 10)) and bl.contains(Point(30, 10)) and bl.contains(Point(40, 10))
    assert not bl.contains(Point(10, 10)) and not bl.contains(Point(60, 10))
    # 3. the spine router's cells carry their net
    cells, nets = rp._plane_pad_cells(geom, MT / 2.0, GridCoord(0.1))
    gx, gy = GridCoord(0.1).to_grid(10, 10)
    at = nets[(cells[:, 0] == gx) & (cells[:, 1] == gy)]
    assert set(at.tolist()) == {5}, "net 5's own pad: its cells are net 5's, so net 5 leaves them out"
    hx, hy = GridCoord(0.1).to_grid(30, 10)
    assert set(nets[(cells[:, 0] == hx) & (cells[:, 1] == hy)].tolist()) == {0}, "a hole's cells are no net's"
    print("  layer model: discs, bounding boxes, hole and slot clearances, cutouts; blockers; spine cells")


def test_spine_samples():
    path = [(0.0, 0.0), (20.0, 0.0)]
    other = cKDTree([(5.0, 1.0)])                 # another net's seed 1 mm beside x = 5
    s = rp._spine_samples(path, other, 2.0, 0.05)
    near = sorted(x for x, _ in s if 4.0 <= x <= 6.0)
    # each gap at most a quarter of its end's distance to the pad (rounded up to the 0.05 mm candidate step)
    bad = [(round(a, 2), round(b, 2)) for a, b in zip(near, near[1:])
           if b - a > math.hypot(b - 5.0, 1.0) / 4.0 + 0.05 + 1e-9]
    assert len(near) >= 6 and not bad, f"seeds beside a pad are a quarter of its distance apart: {bad}"
    far = sorted(x for x, _ in s if x >= 12.0)
    assert all(b - a <= 2.0 + 0.05 + 1e-9 for a, b in zip(far, far[1:])) and len(far) <= 6, \
        "far from every other seed: the plain interval"
    print("  spine samples: a quarter of the distance beside a pad, the interval elsewhere")


def test_background_net():
    spread = [(x, y) for x in range(5, 100, 10) for y in range(5, 60, 10)]
    tight = [(50 + i * 0.5, 30) for i in range(80)]           # more pads, but in one corner of the board
    assert rp._background_net({1: [], 2: []}, {1: spread, 2: tight}, (0, 0, 100, 60)) == 1
    print("  background: the board-wide rail")


ZONE = [(1.0, 1.0), (99.0, 1.0), (99.0, 59.0), (1.0, 59.0)]
DOM = 1
DOM_PADS = [(float(x), float(y)) for x in range(5, 100, 10) for y in (5, 15, 25, 35, 45, 55)]
GEOM = {'copper': [], 'holes': [], 'board': box(0, 0, 100, 60), 'zc': ZC, 'edge': EDGE}


def split(minor_anchors, routes, extra_dom=(), extra_dom_seeds=(), geom=None):
    """_reach_and_sheet on a synthetic layer: the background's pads on a 10 mm grid, net 2's anchors and spines;
    the shares from the real Voronoi over anchors and spine samples (extra_dom_seeds: background seeds that are not
    its anchors, as its own spine samples are)"""
    dom_pts = DOM_PADS + list(extra_dom)
    seeds = {DOM: list(dom_pts) + list(extra_dom_seeds), 2: list(minor_anchors)}
    for path in routes:
        seeds[2] += rp.sample_route_for_voronoi(path, sample_interval=0.25)
    raw = voronoi_cells(seeds, (0, 0, 100, 60))
    out = rp._reach_and_sheet(DOM, raw, {DOM: dom_pts, 2: list(minor_anchors)}, [(2, p) for p in routes],
                              ZONE, MT, {DOM: 'BG', 2: 'N2'}, geom or GEOM)
    return out, [rp_poly(p) for p in out.get(2, [])]


def rp_poly(pts):
    from shapely.geometry import Polygon
    return Polygon(pts)


def test_groups_joined_by_corridor():
    a = [(20 + dx, 30 + dy) for dx, dy in ((0, 0), (1.5, 0), (0, 1.5), (-1.5, 0), (0, -1.5))]
    b = [(70 + dx, 30 + dy) for dx, dy in ((0, 0), (1.5, 0), (0, 1.5), (-1.5, 0), (0, -1.5))]
    out, polys = split(a + b, [[(21.5, 30.0), (68.5, 30.0)]])
    assert out[DOM][0] == ZONE, "the background is the whole sheet"
    assert len(polys) == 1, f"two groups and the spine between them: one region, got {len(polys)}"
    assert all(polys[0].contains(Point(x, y)) for x, y in a + b), "every anchor inside its net's region"
    print("  two groups + spine: one region; the background the whole sheet")


def test_severing_corridor_refused():
    ends = [(50.0, 3.0), (50.0, 57.0)]
    out, polys = split(ends, [[(50.0, 3.0), (50.0, 57.0)]])
    assert not any(p.contains(Point(50.0, 30.0)) for p in polys), \
        "a full-height corridor would cut the background in two halves that both hold its pads: refused"
    assert len(polys) == 2
    print("  severing corridor: refused (invariant 3b)")


def test_parallel_corridors_merge():
    # corridors 5.5 mm apart, 3 mm wide: a 2.5 mm gap of background between them, OPEN to the sheet at both ends
    # (their end groups are separate octagons) -- so the gap's fill is live background the finishing keeps, and
    # only the merge makes the two one band
    anchors = [(20.0, 27.25), (20.0, 32.75), (80.0, 27.25), (80.0, 32.75)]
    out, polys = split(anchors, [[(20.0, 27.25), (80.0, 27.25)], [(20.0, 32.75), (80.0, 32.75)]])
    assert any(p.contains(Point(50.0, 30.0)) for p in polys), \
        "two corridors a 2.5 mm gap apart run together into one band"
    print("  parallel corridors: one band")


def test_background_pocket():
    ring = [(50 + 3 * math.cos(k * math.pi / 6), 30 + 3 * math.sin(k * math.pi / 6)) for k in range(12)]
    out, polys = split(ring, [], extra_dom=[(50.0, 30.0)])
    pockets = [rp_poly(p) for p in out[DOM][1:]]
    assert any(p.contains(Point(50.0, 30.0)) for p in pockets), \
        "a background pad inside another net's region keeps a pocket of its own"
    # the same cells without a background PAD (a seed of its own spine, say), walled off by a ring of milled slots
    # so the region round them cannot feed them either (a panel's rails, behind their tabs): copper nothing feeds,
    # so no pocket zone -- it would ship as a dead island
    slots = [('capsule', (50 + 1.6 * math.cos(a), 30 + 1.6 * math.sin(a), 50 + 1.6 * math.cos(b),
                          30 + 1.6 * math.sin(b), 0.25), EDGE)
             for a, b in ((k * math.pi / 8, (k + 1) * math.pi / 8) for k in range(16))]
    out, polys = split(ring, [], extra_dom_seeds=[(50.0, 30.0)], geom=dict(GEOM, holes=slots))
    assert len(out[DOM]) == 1, f"a background piece holding none of its anchors is no pocket: {len(out[DOM]) - 1}"
    print("  background pocket: kept round its own pad, none without one")


def test_voronoi_cells():
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    seeds = {1: [(10.0, 10.0), (30.0, 10.0)], 2: [(20.0, 30.0)]}
    cells = voronoi_cells(seeds, (0, 0, 40, 40), board_edge_clearance=1.0)
    assert [len(cells[1]), len(cells[2])] == [2, 1], "one cell per seed, by its net"
    for nid, pts in seeds.items():
        for (x, y), c in zip(pts, cells[nid]):
            assert Polygon(c).contains(Point(x, y)), f"net {nid}'s cell holds its seed ({x}, {y})"
    whole = unary_union([Polygon(c) for cs in cells.values() for c in cs])
    assert abs(whole.area - 38.0 * 38.0) < 1e-6, f"the cells tile the inset bounds: {whole.area:.3f}"
    for few in ({1: [(5.0, 5.0)]}, {1: []}):
        try:
            voronoi_cells(few, (0, 0, 40, 40))
        except ValueError:
            continue
        raise AssertionError(f"fewer than two seeds must raise: {few}")
    print("  voronoi_cells: a cell per seed, tiling the inset bounds; one seed refused")


def test_end_to_end():
    board = os.path.join(_ROOT, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, 'split.kicad_pcb')
        p = subprocess.run([sys.executable, '-X', 'utf8', os.path.join(_ROOT, 'py_router', 'route_planes.py'),
                            board, out, '--nets', 'GND', '+3V3', '+1V1', '--plane-layers', 'In1.Cu', 'In2.Cu',
                            'In2.Cu'], capture_output=True, text=True, cwd=d)
        assert p.returncode == 0, p.stdout[-2000:] + p.stderr[-2000:]
        assert 'Spine split:' in p.stdout, "the shared layer went through the spine split"
        txt = open(out, encoding='utf-8').read()
        for net in ('+3V3', '+1V1'):
            assert f'(net "{net}")' in txt or f'(net_name "{net}")' in txt
        assert txt.count('(layer "In2.Cu")') >= 2, "both nets' zones on the shared layer"
    print("  end to end: route_planes.py pours the shared layer through the spine split")


def main():
    test_layer_geometry()
    test_spine_samples()
    test_background_net()
    test_groups_joined_by_corridor()
    test_severing_corridor_refused()
    test_parallel_corridors_merge()
    test_background_pocket()
    test_voronoi_cells()
    test_end_to_end()
    print("PASS: spine split")
    return 0


if __name__ == "__main__":
    sys.exit(main())

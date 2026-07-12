#!/usr/bin/env python3
"""Custom comb/finger pads carry their real copper polygon, so the finger
channels stay routable instead of being filled by the bounding box (issue #188).

Covers:
  1. parser local->global transform for a custom gr_poly pad;
  2. check_drc polygon clearance (track/via in a finger channel clears the pad,
     over the solid body it does not);
  3. obstacle map + per-net cache: the channel cell stays OPEN for the comb
     polygon but is BLOCKED by the bounding-box control;
  4. reading real repo boards: a board with custom-poly pads populates polygons;
  5. gr_circle / gr_rect / gr_line primitives are modelled as their real shape
     (a round mounting/logo pad becomes its disc at the true offset, not the
     anchor-centred bounding box) -- issue #232.

    python3 tests/test_custom_pad_polygon.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import _custom_pad_global_polygons, parse_kicad_pcb, Pad
from check_drc import _point_to_polys_distance, _segment_to_polys_distance
from routing_config import GridRouteConfig, GridCoord
from obstacle_map import GridObstacleMap, _add_pad_obstacle
from obstacle_cache import _collect_pad_obstacles
import numpy as np


# A comb in local quadrant x[-5,0] y[0,4.56] with notches (slots between x=-5 and
# x=-4.32) -- the bitaxe Q1/Q2 MOSFET source-pad shape. Anchor at (0,0).
PAD_TEXT = """(pad "5" smd custom (at 0 0 0) (size 0.01 0.01)
  (primitives (gr_poly (pts
    (xy 0 0) (xy 0 4.56) (xy -5 4.56) (xy -5 3.898) (xy -4.32 3.898)
    (xy -4.32 3.2647) (xy -5 3.2647) (xy -5 2.5997) (xy -4.32 2.5997)
    (xy -4.32 1.9664) (xy -5 1.9664) (xy -5 1.3014) (xy -4.32 1.3014)
    (xy -4.32 0.6681) (xy -5 0.6681) (xy -5 0)
  ))))"""

ANCHOR = (10.0, 20.0)
# At orientation 0 the transform is identity: global = anchor + raw.
BODY = (ANCHOR[0] - 2.0, ANCHOR[1] + 2.0)          # raw (-2, 2): solid copper
SLOT = (ANCHOR[0] - 4.66, ANCHOR[1] + 3.5814)      # raw (-4.66, 3.5814): in a notch


def _pip(x, y, poly):
    n = len(poly); inside = False; j = n - 1
    for i in range(n):
        xi, yi = poly[i]; xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def _comb_polys():
    return _custom_pad_global_polygons(PAD_TEXT, ANCHOR[0], ANCHOR[1], 0.0)


def _make_pad(polygons):
    return Pad(component_ref='Q1', pad_number='5', global_x=ANCHOR[0], global_y=ANCHOR[1],
               local_x=0.0, local_y=0.0, size_x=10.0, size_y=9.13, shape='custom',
               layers=['F.Cu', 'F.Mask'], net_id=1, net_name='/SRC', polygons=polygons)


def run():
    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)

    # 1. Parser transform -------------------------------------------------------
    polys = _comb_polys()
    check("parser returns one polygon", polys is not None and len(polys) == 1)
    poly = polys[0]
    xs = [x for x, y in poly]; ys = [y for x, y in poly]
    check("polygon bbox correct",
          abs(min(xs) - 5.0) < 1e-6 and abs(max(xs) - 10.0) < 1e-6 and
          abs(min(ys) - 20.0) < 1e-6 and abs(max(ys) - 24.56) < 1e-6)
    check("slot point outside copper", not _pip(*SLOT, poly))
    check("body point inside copper", _pip(*BODY, poly))

    # 2. check_drc polygon clearance -------------------------------------------
    clr = 0.1
    d_slot, _ = _segment_to_polys_distance(SLOT[0] - 0.25, SLOT[1], SLOT[0] + 0.2, SLOT[1], polys)
    d_body, _ = _segment_to_polys_distance(BODY[0] - 1.0, BODY[1], BODY[0] + 1.0, BODY[1], polys)
    check("DRC: track in slot clears copper", d_slot > clr)
    check("DRC: track over body does not clear", d_body <= clr)
    check("DRC: via in slot clears", _point_to_polys_distance(*SLOT, polys) > clr)
    check("DRC: via over body does not clear", _point_to_polys_distance(*BODY, polys) <= clr)

    # 3. Obstacle map + cache: channel open for polygon, blocked for bbox -------
    cfg = GridRouteConfig(grid_step=0.05, clearance=0.1, track_width=0.127,
                          via_size=0.5, via_drill=0.3, layers=['F.Cu', 'B.Cu'])
    coord = GridCoord(cfg.grid_step)
    lm = {'F.Cu': 0, 'B.Cu': 1}
    sgx, sgy = coord.to_grid(*SLOT)
    bgx, bgy = coord.to_grid(*BODY)

    obs_poly = GridObstacleMap(2)
    _add_pad_obstacle(obs_poly, _make_pad(polys), coord, lm, cfg)
    check("obstacle(polygon): solid body blocked", obs_poly.is_blocked(bgx, bgy, 0))
    check("obstacle(polygon): notch channel OPEN", not obs_poly.is_blocked(sgx, sgy, 0))

    obs_bbox = GridObstacleMap(2)
    _add_pad_obstacle(obs_bbox, _make_pad(None), coord, lm, cfg)  # bbox control
    check("obstacle(bbox control): notch channel BLOCKED", obs_bbox.is_blocked(sgx, sgy, 0))

    # per-net cache path (what the routing loop actually uses)
    bc, bv = [], []
    _collect_pad_obstacles(_make_pad(polys), coord, lm, cfg, 0.0, bc, bv)
    blocked = set()
    for arr in bc:
        for row in arr:
            blocked.add((int(row[0]), int(row[1]), int(row[2])))
    check("cache(polygon): solid body blocked", (bgx, bgy, 0) in blocked)
    check("cache(polygon): notch channel OPEN", (sgx, sgy, 0) not in blocked)

    # 4. Real repo boards -------------------------------------------------------
    oc = parse_kicad_pcb('kicad_files/orangecrab_ext_pll.kicad_pcb')
    n_poly = sum(1 for fp in oc.footprints.values() for pd in fp.pads if pd.polygons)
    check("orangecrab: some custom pads carry polygons", n_poly > 0)
    # every polygon is a list of >=3 global (x,y) vertices
    ok_shape = all(len(p) >= 3 and all(len(v) == 2 for v in p)
                   for fp in oc.footprints.values() for pd in fp.pads
                   for p in (pd.polygons or []))
    check("orangecrab: polygons well-formed", ok_shape)

    tg = parse_kicad_pcb('kicad_files/tigard.kicad_pcb')
    tg_custom = [pd for fp in tg.footprints.values() for pd in fp.pads if pd.shape == 'custom']
    check("tigard: has custom pads", len(tg_custom) > 0)
    # #232: tigard's custom pads mix gr_poly + gr_circle -- now modelled (not bbox).
    check("tigard: gr_circle custom pads carry polygons (#232)",
          all(pd.polygons for pd in tg_custom))
    check("tigard: custom pad polygons well-formed",
          all(len(p) >= 3 and all(len(v) == 2 for v in p)
              for pd in tg_custom for p in pd.polygons))

    # 5. gr_circle ring -> disc at the true offset, not the anchor bbox (#232) -----
    # A round mounting pad: ring centerline r=1.35 at local (1.35,0), stroke 0.4,
    # so real copper is a disc out to r+w/2=1.55 centred at local (1.35,0). The
    # (size) bbox would be ~5.8x3.1 centred on the anchor with a big empty -x half.
    RING = """(pad "1" smd custom (at 0 0 0) (size 5.8 3.1)
      (primitives (gr_circle (center 1.35 0) (end 2.7 0) (width 0.4) (fill no))))"""
    rpolys = _custom_pad_global_polygons(RING, 100.0, 50.0, 0.0)
    check("gr_circle: one disc polygon", rpolys is not None and len(rpolys) == 1)
    if rpolys:
        rp = rpolys[0]
        rxs = [x for x, y in rp]; rys = [y for x, y in rp]
        # disc spans local x[1.35-1.55, 1.35+1.55]=[-0.2,2.9], y[-1.55,1.55] -> global +100/+50
        check("gr_circle: disc at offset centre (+x side only)",
              abs(min(rxs) - 99.8) < 0.05 and abs(max(rxs) - 102.9) < 0.05 and
              abs(min(rys) - 48.45) < 0.05 and abs(max(rys) - 51.55) < 0.05)
        # the empty -x half of the old bbox (local x ~ -2) is now OUTSIDE the copper
        check("gr_circle: anchor-side empty region no longer copper",
              not _pip(98.0, 50.0, rp))
        check("gr_circle: disc centre is copper", _pip(101.35, 50.0, rp))

    print("=" * 60)
    if fails:
        for f in fails:
            print(f"  FAIL: {f}")
        print(f"\n{len(fails)} check(s) failed")
        return 1
    print("  PASS: custom comb pad polygon parsed; channels stay clear in DRC + "
          "obstacle map + cache while solid copper blocks; repo boards read OK")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(run())

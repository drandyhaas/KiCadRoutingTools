#!/usr/bin/env python3
"""The plane engines' obstacle map holds VIA copper off NPTH holes the way the
signal map does.

`plane_region_connector.build_base_obstacles` is the map behind region joins,
repair_planes and every KiCad-oracle link leg (kicad_oracle builds it per link
and per via rung). It stamped a via keep-out around NPTH holes only at the
hole-to-hole DRILL minimum, and had neither of the signal map's via-copper
arms (`obstacle_map.add_drill_hole_obstacles`):

  * #505: an NPTH pad's clearance OVERRIDE (`pad.local_clearance`) holds via
    copper that far off the hole wall;
  * #1038: the board's declared copper-to-hole floor (min_hole_clearance /
    fab_floor_origin / config.hole_clearance) holds via copper off every NPTH
    wall.

Found on neo6502 (route.py GND step, plane finalize): two 0.7 mm oracle-link
vias landed 0.15 and 0.10 mm inside MH3's 1.85 mm override (KiCad: hole
clearance). Both maps now call one helper,
`obstacle_map.block_via_cells_near_npth_floors`.

Geometry (as test_505_npth_override_via): NPTH hole at (15,15), drill 2.1
(wall at r=1.05), via 0.55/0.254, h2h 0.25, 0.1 grid. The plane map's own
drill stamp is a cell disc of radius ceil(1.427 / 0.1) = 15 cells, so every
probe below sits OUTSIDE it (a probe inside is blocked either way and
measures nothing):
  * override 0.9 -> via copper needs 1.05 + 0.9 + 0.275 = 2.225: a via at
    1.9 must be blocked, one at 3.0 free;
  * declared 0.25 -> 1.05 + 0.25 + 0.275 = 1.575: the cell (+1.3, +0.8),
    1.526 off the centre (copper 0.20 off the wall), blocked; 1.6 free;
  * nothing declared, no override -> that cell stays free (raise-only);
  * the hole's own nominal net being the plane net changes nothing (a drill
    does not care whose net it is);
  * over a window around the hole, the plane map's via-blocked cells equal
    the signal map's, in both armed cases.

    python3 tests/test_plane_map_npth_via_arms.py
"""
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import parse_kicad_pcb  # noqa: E402
from obstacle_map import GridObstacleMap, add_drill_hole_obstacles  # noqa: E402
from plane_region_connector import build_base_obstacles  # noqa: E402
from routing_config import GridRouteConfig, GridCoord  # noqa: E402

fails = []


def check(name, cond, detail=''):
    print(("  ok  " if cond else " FAIL ") + name + (f"  {detail}" if detail else ''))
    if not cond:
        fails.append(name)


def _rect_edge(x1, y1, x2, y2):
    pts = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
    return "\n".join(
        f' (gr_line (start {pts[i][0]} {pts[i][1]}) '
        f'(end {pts[(i + 1) % 4][0]} {pts[(i + 1) % 4][1]}) '
        f'(layer "Edge.Cuts") (width 0.1))' for i in range(4))


def board(clearance_tok, hole_net=0):
    net_tok = f' (net {hole_net} "GND")' if hole_net else ''
    return f'''(kicad_pcb
 (version 20221018)
 (net 0 "")
 (net 1 "GND")
 (net 2 "SIG")
{_rect_edge(0, 0, 30, 30)}
 (footprint "t:mh" (layer "F.Cu") (at 15 15)
  (property "Reference" "MH1" (at 0 -2) (layer "F.SilkS"))
  (pad "" np_thru_hole circle (at 0 0) (size 2.1 2.1) (drill 2.1)
       (layers "F&B.Cu" "*.Mask"){clearance_tok}{net_tok})
 )
 (segment (start 2 2) (end 4 2) (width 0.2) (layer "F.Cu") (net 2) (uuid "s"))
)'''


def cfg(hole_clr=0.0):
    c = GridRouteConfig(layers=["F.Cu", "B.Cu"], grid_step=0.1, clearance=0.2,
                        track_width=0.2, via_size=0.55, via_drill=0.254,
                        hole_to_hole_clearance=0.25)
    c.hole_clearance = hole_clr
    return c


def plane_map(pcb, c, exclude=(1,)):
    obs, _ = build_base_obstacles(
        exclude_net_ids=set(exclude), routing_layers=list(c.layers),
        pcb_data=pcb, config=c, track_width=c.track_width,
        track_via_clearance=0.2,
        hole_to_hole_clearance=c.hole_to_hole_clearance)
    return obs


def signal_map(pcb, c):
    obs = GridObstacleMap(len(c.layers))
    add_drill_hole_obstacles(obs, pcb, c, set())
    return obs


def blocked(obs, c, xy):
    gx, gy = GridCoord(c.grid_step).to_grid(*xy)
    return obs.is_via_blocked(gx, gy)


def window_diff(a, b, c, half=2.6):
    coord = GridCoord(c.grid_step)
    g0x, g0y = coord.to_grid(15 - half, 15 - half)
    g1x, g1y = coord.to_grid(15 + half, 15 + half)
    return [(gx, gy) for gx in range(g0x, g1x + 1) for gy in range(g0y, g1y + 1)
            if a.is_via_blocked(gx, gy) != b.is_via_blocked(gx, gy)]


OVR = (16.9, 15.0)    # 1.9 mm: outside the drill disc, inside the override
FLOOR = (16.3, 15.8)  # 1.526 mm, cell (13, 8): 13^2 + 8^2 = 233 > 15^2


def parse(txt):
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, 'b.kicad_pcb')
        with open(p, 'w', encoding='utf-8') as f:
            f.write(txt)
        return parse_kicad_pcb(p)


# 1. #505: the override arm
pcb = parse(board(' (clearance 0.9)'))
c = cfg()
obs = plane_map(pcb, c)
check("override 0.9: a via at 1.9 mm (copper inside the override) is "
      "blocked", blocked(obs, c, OVR))
check("override 0.9: a via at 3.0 mm (clears 2.225) stays free",
      not blocked(obs, c, (18.0, 15.0)))
d = window_diff(obs, signal_map(pcb, c), c)
check("override 0.9: plane map == signal map around the hole", not d,
      f"{len(d)} cell(s) differ, e.g. {d[:3]}")

# 2. #1038: the declared copper-to-hole floor arm
pcb = parse(board(''))
c = cfg(0.25)
obs = plane_map(pcb, c)
check("declared 0.25: the via at 1.526 mm (copper 0.20 off the wall) is "
      "blocked", blocked(obs, c, FLOOR))
check("declared 0.25: a via at 1.6 mm stays free",
      not blocked(obs, c, (16.6, 15.0)))
d = window_diff(obs, signal_map(pcb, c), c)
check("declared 0.25: plane map == signal map around the hole", not d,
      f"{len(d)} cell(s) differ, e.g. {d[:3]}")

# 3. raise-only: nothing declared, no override
c = cfg()
check("nothing declared, no override: the via at 1.526 mm stays free",
      not blocked(plane_map(pcb, c), c, FLOOR))

# 4. net-independent: the hole nominally on the plane net being routed
pcb = parse(board(' (clearance 0.9)', hole_net=1))
c = cfg()
check("override on a GND-tied NPTH, GND routed: still blocked",
      blocked(plane_map(pcb, c, exclude=(1,)), c, OVR))
pcb = parse(board('', hole_net=1))
c = cfg(0.25)
check("declared floor, GND-tied NPTH, GND routed: still blocked",
      blocked(plane_map(pcb, c, exclude=(1,)), c, FLOOR))

print("-" * 60)
if fails:
    print(f"{len(fails)} FAILED: " + "; ".join(fails))
    sys.exit(1)
print("ALL PASS")

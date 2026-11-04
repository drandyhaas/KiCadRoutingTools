#!/usr/bin/env python3
"""Issue #1514: the plane-tap via map holds via COPPER off NPTH holes at the
board's declared copper-to-hole floor, as the signal map does.

`plane_obstacle_builder.build_via_obstacle_map` is the via map behind every
plane tap (`plane_pad_tap`, its SharedViaMaps and per-pad window maps,
route_planes) and the net-rescue bare-ball escape (`tap_pad_with_escalation`).
It stamped only the hole-to-hole DRILL minimum, so a via whose drill cleared
an NPTH at h2h was admitted while its annulus sat inside the declared floor:
a 0.45/0.2 via 0.7276 mm from a 0.65 mm NPTH puts copper 0.1776 mm off the
wall on a board declaring 0.2. The signal map (`add_drill_hole_obstacles`) and
the plane connector's map have held that floor since #1038, through
`obstacle_map.block_via_cells_near_npth_floors`; the tap map now calls it too.

Geometry: NPTH drill 0.65 at (10.038, 10.0); via 0.45/0.2; clearance 0.125;
h2h 0.25; grid 0.05. Drill stamp centre distance 0.325 + 0.1 + 0.25 = 0.675;
floor stamp 0.325 + 0.2 + 0.225 = 0.75. The probe via at (10.75, 10.15) is
0.7276 mm off the hole centre: outside the drill stamp, inside the floor.

  1. the floor declared as the chain declares it (rules 0.125 after a
     writeback, fab_floor_origin 0.2): the probe is blocked in the tap map, as
     in the signal map;
  2. a via at 0.8 mm (copper clears 0.2) stays free;
  3. the hole's nominal net being the tapped net changes nothing;
  4. control, nothing declared: the probe stays free (raise-only);
  5. a #505 NPTH pad override still holds via copper (the tap map's pad
     stamp held it before; the shared arm must not loosen it);
  6. over a window around the hole, every cell the signal map blocks the
     tap map blocks too (the tap map is allowed to be stricter: it also
     stamps the NPTH pad's outline as a pad).

    python3 tests/test_1514_tap_via_map_npth_floor.py
"""
import contextlib
import io
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import obstacle_map  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from obstacle_map import GridObstacleMap, add_drill_hole_obstacles  # noqa: E402
from plane_obstacle_builder import build_via_obstacle_map  # noqa: E402
from routing_config import GridRouteConfig, GridCoord  # noqa: E402

FAILS = []
HOLE = (10.038, 10.0)
PROBE = (10.75, 10.15)    # 0.7276 mm: copper 0.1776 off the wall
FREE = (10.85, 10.0)      # 0.812 mm: copper 0.262 off the wall


def check(name, cond, detail=''):
    print(("  ok   " if cond else "  FAIL ") + name + (f"  {detail}" if detail else ''))
    if not cond:
        FAILS.append(name)


def _edge(x1, y1, x2, y2):
    pts = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
    return "\n".join(
        f' (gr_line (start {pts[i][0]} {pts[i][1]}) '
        f'(end {pts[(i + 1) % 4][0]} {pts[(i + 1) % 4][1]}) '
        f'(layer "Edge.Cuts") (width 0.1))' for i in range(4))


def board_text(extra='', hole_net=0):
    net_tok = f' (net {hole_net} "PWR")' if hole_net else ''
    return f'''(kicad_pcb
 (version 20240108)
 (net 0 "")
 (net 1 "PWR")
 (net 2 "SIG")
{_edge(0, 0, 20, 20)}
 (footprint "t:conn" (layer "F.Cu") (at {HOLE[0]} {HOLE[1]})
  (property "Reference" "J1" (at 0 -2) (layer "F.SilkS"))
  (pad "" np_thru_hole circle (at 0 0) (size 0.65 0.65) (drill 0.65)
       (layers "F&B.Cu" "*.Mask"){extra}{net_tok})
 )
 (segment (start 2 2) (end 4 2) (width 0.2) (layer "F.Cu") (net 2) (uuid "s"))
)'''


def load(td, name, text, rule=None, origin=None):
    """Write the board (and a project when a floor is declared) and parse it."""
    for c in (obstacle_map._HOLE_CLR_CACHE, obstacle_map._HOLE_CLR_ANNOUNCED,
              obstacle_map._HOLE_CLR_ORIGIN):
        c.clear()
    pcb = os.path.join(td, name + '.kicad_pcb')
    with open(pcb, 'w', encoding='utf-8') as fh:
        fh.write(text)
    if rule is not None or origin is not None:
        proj = {"board": {"design_settings": {"rules": {}}}}
        if rule is not None:
            proj["board"]["design_settings"]["rules"]["min_hole_clearance"] = rule
        if origin is not None:
            proj["kicad_routing_tools"] = {
                "fab_floor_origin": {"min_hole_clearance": origin}}
        with open(os.path.splitext(pcb)[0] + '.kicad_pro', 'w',
                  encoding='utf-8') as fh:
            json.dump(proj, fh)
    return parse_kicad_pcb(pcb)


def cfg():
    return GridRouteConfig(layers=["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"],
                           grid_step=0.05, clearance=0.125, track_width=0.1,
                           via_size=0.45, via_drill=0.2,
                           hole_to_hole_clearance=0.25)


def maps(pcb, c):
    with contextlib.redirect_stdout(io.StringIO()):
        tap = build_via_obstacle_map(pcb, c, 1, verbose=False)
        sig = GridObstacleMap(len(c.layers))
        add_drill_hole_obstacles(sig, pcb, c, set())
    return tap, sig


def blocked(obs, c, xy):
    gx, gy = GridCoord(c.grid_step).to_grid(*xy)
    return obs.is_via_blocked(gx, gy)


def admitted(tap, sig, c, half=1.5):
    """Cells around the hole the signal map refuses a via but the tap map
    admits."""
    coord = GridCoord(c.grid_step)
    g0x, g0y = coord.to_grid(HOLE[0] - half, HOLE[1] - half)
    g1x, g1y = coord.to_grid(HOLE[0] + half, HOLE[1] + half)
    return [(gx, gy) for gx in range(g0x, g1x + 1) for gy in range(g0y, g1y + 1)
            if sig.is_via_blocked(gx, gy) and not tap.is_via_blocked(gx, gy)]


def main():
    c = cfg()
    with tempfile.TemporaryDirectory() as td:
        print('1-2. floor declared as a chain declares it (rules 0.125, origin 0.2)')
        pcb = load(td, 'declared', board_text(), rule=0.125, origin=0.2)
        tap, sig = maps(pcb, c)
        check('signal map blocks the 0.7276 mm via (reference)',
              blocked(sig, c, PROBE))
        check('tap map blocks the 0.7276 mm via (copper 0.1776 < 0.2)',
              blocked(tap, c, PROBE))
        check('tap map leaves the 0.812 mm via free', not blocked(tap, c, FREE))
        d = admitted(tap, sig, c)
        check('the tap map admits no via the signal map refuses', not d,
              f'{len(d)} cell(s), e.g. {d[:3]}')

        print('3. the hole on the tapped net itself')
        pcb = load(td, 'ownnet', board_text(hole_net=1), rule=0.125, origin=0.2)
        tap, sig = maps(pcb, c)
        check('tap map still blocks the probe', blocked(tap, c, PROBE))

        print('4. control: nothing declared')
        pcb = load(td, 'silent', board_text())
        tap, sig = maps(pcb, c)
        check('tap map leaves the probe free (raise-only)',
              not blocked(tap, c, PROBE))
        d = admitted(tap, sig, c)
        check('the tap map admits no via the signal map refuses', not d,
              f'{len(d)} cell(s), e.g. {d[:3]}')

        print('5. #505: an NPTH pad clearance override')
        pcb = load(td, 'override', board_text(' (clearance 0.3)'))
        tap, sig = maps(pcb, c)
        check('tap map blocks a via whose copper is inside the 0.3 override',
              blocked(tap, c, FREE))
        d = admitted(tap, sig, c)
        check('the tap map admits no via the signal map refuses', not d,
              f'{len(d)} cell(s), e.g. {d[:3]}')

    if FAILS:
        print(f'\nFAILED ({len(FAILS)})')
        sys.exit(1)
    print('\nPASS')


if __name__ == '__main__':
    main()

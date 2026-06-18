#!/usr/bin/env python3
"""Hole-to-hole via keepout must be enforced by REAL mm distance, not a floored
integer-cell disk, so a via cannot land a sub-cell inside the drill hole-to-hole
minimum (issue #70 / #125 -- the castor_pollux RV3 PAD-DRILL-VIA-DRILL).

    python3 tests/test_drill_hole_keepout.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Pad, PCBData, BoardInfo
from routing_config import GridRouteConfig, GridCoord
from obstacle_map import GridObstacleMap, add_drill_hole_obstacles


def _pcb_with_pad(x, y, drill):
    pad = Pad(pad_number='M', net_id=0, net_name='', global_x=x, global_y=y,
              size_x=2.5, size_y=4.5, shape='oval', layers=['*.Cu'], drill=drill,
              component_ref='RV3', local_x=0.0, local_y=0.0)
    pcb = PCBData(board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu']),
                  nets={}, footprints={}, vias=[], segments=[],
                  pads_by_net={0: [pad]})
    return pcb


def run():
    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)

    # Reproduce the castor_pollux geometry: a 3.0mm mounting-lug drill, default
    # 0.1mm grid, via drill 0.3, hole-to-hole 0.2 -> required c-c = 1.85mm.
    cfg = GridRouteConfig(grid_step=0.1, via_drill=0.3, hole_to_hole_clearance=0.2)
    coord = GridCoord(cfg.grid_step)
    pcb = _pcb_with_pad(79.80, 80.70, 3.0)

    obs = GridObstacleMap(4)
    add_drill_hole_obstacles(obs, pcb, cfg, nets_to_route_set=set())

    gx, gy = coord.to_grid(79.80, 80.70)

    # The offending via cell at (81.50,81.40): real distance 1.838mm < 1.85mm.
    # The old floored integer-disk test (expand=18, 17^2+7^2=338 > 324) MISSED
    # this cell; the mm-distance test must block it.
    vx, vy = coord.to_grid(81.50, 81.40)
    check("violating via cell (1.838mm < 1.85) is blocked", obs.is_via_blocked(vx, vy))

    # A cell just outside the minimum must remain free (no over-blocking).
    # (81.50,81.50): distance sqrt(1.7^2+0.8^2)=1.879mm > 1.85mm.
    ox, oy = coord.to_grid(81.50, 81.50)
    check("cell beyond the minimum (1.879mm) stays free", not obs.is_via_blocked(ox, oy))

    # A cell well inside is blocked; a far cell is free.
    ix, iy = coord.to_grid(80.50, 80.70)   # 0.70mm away
    check("cell well inside keepout is blocked", obs.is_via_blocked(ix, iy))
    fx, fy = coord.to_grid(85.00, 85.00)   # far away
    check("far cell is free", not obs.is_via_blocked(fx, fy))

    print("=" * 60)
    if fails:
        for f in fails:
            print(f"  FAIL  {f}")
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("  PASS  hole-to-hole via keepout enforced by mm distance: blocks the")
    print("        sub-cell-inside via, leaves just-outside/far cells free")
    print("\n4/4 checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(run())

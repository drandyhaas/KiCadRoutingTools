#!/usr/bin/env python3
"""Cross-class via-to-via clearance + sub-cell obstacle-via center: positive tests.

KiCad's pairwise clearance between nets of different classes is max(classA, classB).
Two obstacle-map defect classes shipped sub-clearance vias against pre-placed vias
(live KiCad DRC on a real board: 0.2220/0.2248 vs a 0.25 rule; 0.1963/0.1744 vs 0.2):

1. Cross-class: with no net_clearances map, a pre-placed POWER_HI via (0.25) seen
   while routing a Default group (--clearance 0.15) was priced at 0.15. Covered by
   per-obstacle max(routing-side clearance, the obstacle net's own class clearance);
   the map is INERT when not supplied.
2. Sub-cell snap: an off-grid obstacle via's keepout must be measured from the true
   mm center, not a grid-snapped cell (the float-radius disc model), or the keepout
   under-blocks by up to half a cell in real mm.

    python3 tests/test_cross_class_clearance.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import PCBData, BoardInfo, Via
from routing_config import GridRouteConfig, GridCoord
from obstacle_map import build_base_obstacle_map

GRID = 0.1
N_PWR, N_DEF_OBST, N_ROUTE = 1, 2, 99

# One POWER_HI via (grid-aligned), one Default via (over-block probe), and one
# OFF-GRID Default-class via (the sub-cell case, the live (40.1065, 71.09) instance).
V_PWR = Via(x=55.6, y=71.7, size=0.8, drill=0.4, layers=['F.Cu', 'B.Cu'], net_id=N_PWR)
V_DEF = Via(x=30.0, y=30.0, size=0.8, drill=0.4, layers=['F.Cu', 'B.Cu'], net_id=N_DEF_OBST)
V_OFF = Via(x=40.1065, y=71.09, size=0.8, drill=0.4, layers=['F.Cu', 'B.Cu'], net_id=N_DEF_OBST)


def make_pcb():
    return PCBData(board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu'],
                                        board_bounds=None),
                   nets={}, footprints={}, vias=[V_PWR, V_DEF, V_OFF],
                   segments=[], pads_by_net={})


def cfg(clearance):
    # hole_to_hole 0 isolates the copper via-via keepout from the hole-to-hole check.
    return GridRouteConfig(grid_step=GRID, clearance=clearance, via_size=0.6,
                           track_width=0.2, via_drill=0.3,
                           hole_to_hole_clearance=0.0, layers=['F.Cu', 'B.Cu'])


def build(net_clearances, clearance=0.15):
    return build_base_obstacle_map(make_pcb(), cfg(clearance),
                                   nets_to_route=[N_ROUTE],
                                   net_clearances=net_clearances)


def run():
    coord = GridCoord(GRID)
    passed = failed = 0

    def check(name, got, want):
        nonlocal passed, failed
        ok = got == want
        passed += ok
        failed += not ok
        print(f"  {'PASS' if ok else 'FAIL'}  {name}: is_via_blocked={got} (want {want})")

    # The live VREF spot: c-c 0.92195 from the 0.8 POWER_HI via; a routed 0.6 via
    # there is 0.22195 edge-to-edge - legal at 0.15, ILLEGAL at the pair's 0.25.
    q_pwr = coord.to_grid(54.9, 71.1)
    # Over-block probe: c-c 0.922 from the 0.8 DEFAULT via - legal at 0.15 (needs
    # 0.85), must stay free even with the map (its own class is 0.15).
    q_def = coord.to_grid(29.3, 29.4)
    # Sub-cell probe: the live violating placement (40.0, 70.2): true c-c 0.89635 to
    # the off-grid via = 0.19635 edge-to-edge < 0.2; the mm-exact model blocks it.
    q_off = coord.to_grid(40.0, 70.2)
    # Its just-outside neighbour (39.9, 70.2): true edge 0.2136 >= 0.2, must stay free.
    q_off_ok = coord.to_grid(39.9, 70.2)

    print("empty map (prior behaviour must hold - the map is inert when absent):")
    obs = build(None)
    check("POWER_HI spot free at group clearance 0.15", obs.is_via_blocked(*q_pwr), False)
    check("Default probe free", obs.is_via_blocked(*q_def), False)

    print("full map {PWR: 0.25, Default-obstacle: 0.15, routed: 0.15}:")
    obs = build({N_PWR: 0.25, N_DEF_OBST: 0.15, N_ROUTE: 0.15})
    check("POWER_HI spot BLOCKED at max(0.15, 0.25)", obs.is_via_blocked(*q_pwr), True)
    check("Default probe still free (no over-block)", obs.is_via_blocked(*q_def), False)

    # Sub-cell: at this instance the pair clearance is the routing group's own 0.2,
    # so the map is irrelevant - the mm-exact center is what blocks the 0.19635
    # placement and keeps the 0.2136 one free.
    print("sub-cell (off-grid obstacle via, routed group clearance 0.2):")
    obs = build(None, clearance=0.2)
    check("true-0.19635 placement BLOCKED (mm-exact center)", obs.is_via_blocked(*q_off), True)
    check("true-0.2136 neighbour stays free", obs.is_via_blocked(*q_off_ok), False)

    print(f"{passed}/{passed + failed} cross-class + sub-cell clearance tests passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(run())

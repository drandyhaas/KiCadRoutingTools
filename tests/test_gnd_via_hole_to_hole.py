#!/usr/bin/env python3
"""GND return-via placement (add_gnd_vias) must keep the drill hole-to-hole
minimum from through-hole pad drills and other vias, not just the (smaller)
copper clearance -- issue #125 (PAD-DRILL-VIA-DRILL / VIA-DRILL-HOLE).

Pre-fix the pad-drill gate used config.clearance, so a GND via could land inside
the larger hole-to-hole minimum of a pad drill. This forces a GND via toward a
nearby through-hole pad and asserts every placed via clears the fab minimum.

    python3 tests/test_gnd_via_hole_to_hole.py
"""
import os
import sys
import math

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Pad, Via, Net, PCBData, BoardInfo
from routing_config import GridRouteConfig, GridCoord
from obstacle_map import GridObstacleMap  # also puts rust_router on sys.path
from add_gnd_vias import add_gnd_vias_to_existing_board


def run():
    fails = []

    # via 0.4/0.2, clearance 0.1 (copper via-via 0.5), hole-to-hole 0.6
    # (drill via-via 0.8). A signal via at the origin; a GND through-hole pad at
    # (1.2,0). The natural GND-via spot toward the pad sits ~0.4mm from the pad
    # drill -- legal on copper clearance (req 0.35) but well inside the 0.85mm
    # hole-to-hole minimum (0.15 + 0.6 + 0.1).
    cfg = GridRouteConfig(grid_step=0.1, via_size=0.4, via_drill=0.2,
                          clearance=0.1, hole_to_hole_clearance=0.6,
                          layers=['F.Cu', 'B.Cu'])
    coord = GridCoord(cfg.grid_step)

    # A foreign (non-GND) through-hole pad sits 0.8mm from the signal via, so the
    # GND via the function drops near the signal via is tempted to land inside the
    # pad's hole-to-hole keep-out. GND itself has no nearby copper, so placement is
    # not skipped as "already grounded".
    sig_via = Via(x=0.0, y=0.0, size=0.4, drill=0.2, layers=['F.Cu', 'B.Cu'], net_id=1)
    foreign_pad = Pad(pad_number='1', net_id=3, net_name='VCC', global_x=0.8, global_y=0.0,
                      size_x=1.0, size_y=1.0, shape='circle', layers=['*.Cu'], drill=0.3,
                      component_ref='J1', local_x=0.0, local_y=0.0)
    pcb = PCBData(board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu'],
                                       board_bounds=(-5.0, -5.0, 5.0, 5.0)),
                  nets={1: Net(net_id=1, name='SIG', pads=[]),
                        2: Net(net_id=2, name='GND', pads=[]),
                        3: Net(net_id=3, name='VCC', pads=[foreign_pad])},
                  footprints={}, vias=[sig_via], segments=[],
                  pads_by_net={1: [], 2: [], 3: [foreign_pad]})

    obs = GridObstacleMap(len(cfg.layers))  # empty board: no track obstacles

    gnd_vias = add_gnd_vias_to_existing_board(pcb, 'GND', 2.0, cfg, obs, coord)

    # The function must still place a GND via for the signal via (just not near
    # the pad drill) -- otherwise the test isn't exercising placement.
    if not gnd_vias:
        fails.append("no GND via placed (test did not exercise placement)")

    # Invariant: every placed GND via clears the drill hole-to-hole minimum from
    # the pad drill and from the signal via.
    pad_req = foreign_pad.drill / 2 + cfg.hole_to_hole_clearance + cfg.via_drill / 2
    via_req = cfg.via_drill + cfg.hole_to_hole_clearance
    eps = 1e-6
    for v in gnd_vias:
        d_pad = math.hypot(v.x - foreign_pad.global_x, v.y - foreign_pad.global_y)
        if d_pad < pad_req - eps:
            fails.append(f"GND via at ({v.x:.2f},{v.y:.2f}) is {d_pad:.3f}mm from the "
                         f"pad drill < hole-to-hole minimum {pad_req:.3f}mm")
        d_sig = math.hypot(v.x - sig_via.x, v.y - sig_via.y)
        if d_sig < via_req - eps:
            fails.append(f"GND via at ({v.x:.2f},{v.y:.2f}) is {d_sig:.3f}mm from the "
                         f"signal via < hole-to-hole minimum {via_req:.3f}mm")

    print("=" * 60)
    if fails:
        for f in fails:
            print(f"  FAIL: {f}")
        print(f"\n{len(fails)} check(s) failed")
        return 1
    print(f"  PASS: {len(gnd_vias)} GND via(s) placed, all clear the "
          f"{pad_req:.3f}mm pad-drill / {via_req:.3f}mm via hole-to-hole minimum")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(run())

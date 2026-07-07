#!/usr/bin/env python3
"""NPTH (no-copper) mounting holes block routed tracks (issue #233).

An NPTH mounting hole parses as drill>0 with only a *.Mask layer, so the copper
pad blocker stamps no cell for it. Before #233 the router would route a track
straight across the hole -- a real fab short, since the drill removes the copper.
This checks that the drill-hole obstacle pass now blocks track cells on every
copper layer around an NPTH hole, while a copper PTH pad is left to its own copper
obstacle (no double-block from the drill path).

The check_drc 'track-hole' detection side is validated end-to-end against cynthion
(4 NPTH grazes, matching `kicad-cli pcb drc`).

    python3 tests/test_npth_hole_keepout.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Pad, Net, PCBData, BoardInfo
from routing_config import GridRouteConfig, GridCoord
from obstacle_map import GridObstacleMap, add_drill_hole_obstacles, _pad_has_copper


def _npth(x, y, drill=0.7):
    return Pad(component_ref='J5', pad_number='', global_x=x, global_y=y,
               local_x=0.0, local_y=0.0, size_x=drill, size_y=drill, shape='circle',
               layers=['*.Mask'], net_id=0, net_name='', drill=drill)


def _pth(x, y, net_id, drill=0.7, size=1.2):
    return Pad(component_ref='J1', pad_number='1', global_x=x, global_y=y,
               local_x=0.0, local_y=0.0, size_x=size, size_y=size, shape='circle',
               layers=['*.Cu', '*.Mask'], net_id=net_id, net_name='/PWR', drill=drill)


def run():
    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)

    check("_pad_has_copper: NPTH has none", not _pad_has_copper(_npth(0, 0)))
    check("_pad_has_copper: PTH has copper", _pad_has_copper(_pth(0, 0, 5)))

    cfg = GridRouteConfig(grid_step=0.05, clearance=0.1, track_width=0.127,
                          via_size=0.5, via_drill=0.3, layers=['F.Cu', 'In1.Cu', 'B.Cu'])
    coord = GridCoord(cfg.grid_step)

    # NPTH hole blocks tracks on EVERY copper layer ----------------------------
    npth = _npth(10.0, 10.0)
    pcb = PCBData(footprints={}, nets={0: Net(0, '')}, segments=[], vias=[],
                  board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'In1.Cu', 'B.Cu']), pads_by_net={0: [npth]})
    obs = GridObstacleMap(len(cfg.layers))
    add_drill_hole_obstacles(obs, pcb, cfg, set())
    gx, gy = coord.to_grid(npth.global_x, npth.global_y)
    for li, name in enumerate(('F.Cu', 'In1.Cu', 'B.Cu')):
        check(f"NPTH centre blocked {name}", obs.is_blocked(gx, gy, li))
    # a cell just inside the keep-out radius (drill/2+track/2+clr ~= 0.51mm) blocked,
    # one well outside is open.
    inx, iny = coord.to_grid(npth.global_x + 0.45, npth.global_y)
    check("NPTH near edge blocked", obs.is_blocked(inx, iny, 0))
    fx, fy = coord.to_grid(npth.global_x + 1.5, npth.global_y)
    check("cell far from NPTH open", not obs.is_blocked(fx, fy, 0))

    # PTH copper pad: the drill path does NOT add its own track block (its copper
    # obstacle, added separately, owns that), and with hole-to-hole default 0 there
    # is no via block either -> centre stays open here.
    pth = _pth(20.0, 20.0, 5)
    pcb2 = PCBData(footprints={}, nets={5: Net(5, '/PWR')}, segments=[], vias=[],
                   board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'In1.Cu', 'B.Cu']), pads_by_net={5: [pth]})
    cfg0 = GridRouteConfig(grid_step=0.05, clearance=0.1, track_width=0.127,
                           via_size=0.5, via_drill=0.3, hole_to_hole_clearance=0.0,
                           layers=['F.Cu', 'In1.Cu', 'B.Cu'])
    obs2 = GridObstacleMap(len(cfg0.layers))
    add_drill_hole_obstacles(obs2, pcb2, cfg0, set())
    pgx, pgy = coord.to_grid(pth.global_x, pth.global_y)
    check("PTH not track-blocked by drill path", not obs2.is_blocked(pgx, pgy, 0))

    # #335/#328: drill keep-outs are NET-INDEPENDENT. An NPTH hole has no
    # copper to "reach" -- a track crossing it is cut by the drill regardless
    # of net membership, so the own-net exemption no longer applies.
    obs3 = GridObstacleMap(len(cfg.layers))
    add_drill_hole_obstacles(obs3, pcb, cfg, {0})  # net 0 "being routed"
    check("own-net NPTH hole still blocked", obs3.is_blocked(gx, gy, 0))

    print("=" * 60)
    if fails:
        for f in fails:
            print(f"  FAIL: {f}")
        print(f"\n{len(fails)} check(s) failed")
        return 1
    print("  PASS: NPTH holes block tracks on all copper layers; PTH left to its "
          "copper obstacle; own-net holes stay reachable")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(run())

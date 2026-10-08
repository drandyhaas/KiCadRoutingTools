#!/usr/bin/env python3
"""#1179: a plane tap never drops a via that would join only one layer.

sonde_xilinx pours GND on B.Cu only. Its B.Cu pads J1.20 and J1.25 reach only
a pinched fill island, so the in-run finalize's pad repair tapped each with a
B.Cu trace to a through via inside the B.Cu pour. There is no GND on F.Cu, so
each via joined one layer: KiCad `via_dangling` x2, check_weird
`dangling-via`, a 1.65 mm disc of F.Cu routing space and a drill each -- while
the trace's end, inside the pour, was the whole connection.

Checks (try_tap_pad(plane_tap=True), as repair_planes calls it, on a synthetic
2-layer board; a foreign F.Cu track over the pad keeps the via off the pad, so
the tap is a trace to a site):
  1. A net poured on the pad's layer only: the tap is that trace with NO via,
     and its recorded end is inside the pour (what the oracle verifies).
  2. The same net also poured on F.Cu over the site: the via joins two layers
     and is kept.
  3. The pad's layer not poured at all (pour on F.Cu): the via is the only
     way to the plane and is kept.
  4. A one-layer pour where the only site is IN the pad: no tap (a via there
     reaches nothing the pad does not), instead of a dangling via.
  5. Only a PLANE tap drops the via: the single-ended last-resort via, the
     plan's escapes and the fanout rescue call the same ladder for a layer
     change, which is useful with no pour at all.

    python3 tests/test_1179_tap_via_one_layer.py
"""
import contextlib
import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import Pad, Segment, Zone, PCBData, BoardInfo   # noqa: E402
from routing_config import GridRouteConfig                        # noqa: E402
from plane_pad_tap import try_tap_pad                             # noqa: E402
from check_connected import point_in_polygon                      # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def board(zone_layers, blocker=True, pad_size=0.6):
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'}, copper_layers=['F.Cu', 'B.Cu'])
    bi.board_bounds = (0.0, 0.0, 20.0, 20.0)
    pad = Pad(component_ref='J1', pad_number='20', global_x=10.0, global_y=10.0,
              local_x=0.0, local_y=0.0, size_x=pad_size, size_y=pad_size,
              shape='rect', layers=['B.Cu'], net_id=1, net_name='GND')
    poly = [(4.0, 4.0), (16.0, 4.0), (16.0, 16.0), (4.0, 16.0)]
    zones = [Zone(net_id=1, net_name='GND', layer=ly, polygon=list(poly))
             for ly in zone_layers]
    segs = []
    if blocker:
        # Foreign F.Cu copper over the pad: a through via cannot sit there.
        segs.append(Segment(start_x=7.0, start_y=10.0, end_x=13.0, end_y=10.0,
                            width=0.3, layer='F.Cu', net_id=2))
    return pad, PCBData(board_info=bi, nets={}, footprints={}, vias=[],
                        segments=segs, pads_by_net={1: [pad]}, zones=zones)


cfg = GridRouteConfig(track_width=0.2, clearance=0.15, via_size=0.6,
                      via_drill=0.3, grid_step=0.05, board_edge_clearance=0.2,
                      hole_to_hole_clearance=0.2, layers=['F.Cu', 'B.Cu'])


def tap(pad, pcb, plane_tap=True):
    with contextlib.redirect_stdout(io.StringIO()):
        return try_tap_pad(pad, 'B.Cu', 1, pcb, cfg, max_search_radius=3.0,
                           via_size=0.6, via_drill=0.3, disable_reuse=True,
                           plane_tap=plane_tap)


print("1. one-layer pour on the pad's layer")
pad, pcb = board(['B.Cu'])
r = tap(pad, pcb)
check("tapped with a trace and no via",
      r.success and r.via is None and bool(r.segments),
      f"success={r.success} via={r.via} segs={len(r.segments or [])}")
pos = getattr(r, 'reused_via_pos', None)
check("the trace's end is recorded, inside the pour",
      pos is not None and point_in_polygon(pos[0], pos[1], pcb.zones[0].polygon), f"{pos}")
check("... and off the pad (a real trace, not a stub)",
      pos is not None and abs(pos[0] - 10.0) + abs(pos[1] - 10.0) > 0.3)

print("2. the same site also poured on F.Cu")
pad, pcb = board(['B.Cu', 'F.Cu'])
r = tap(pad, pcb)
check("the via joins two layers and is kept",
      r.success and r.via is not None, f"via={r.via}")

print("3. the pad's layer not poured")
pad, pcb = board(['F.Cu'])
r = tap(pad, pcb)
check("the via is the way to the plane and is kept",
      r.success and r.via is not None, f"via={r.via}")

print("4. one-layer pour, the only site is in the pad")
pad, pcb = board(['B.Cu'], blocker=False, pad_size=1.2)
r = tap(pad, pcb)
check("no dangling via-in-pad tap", not r.success and r.via is None,
      f"success={r.success} via={r.via}")

print("5. an ESCAPE caller keeps its via on a one-layer pour")
pad, pcb = board(['B.Cu'])
r = tap(pad, pcb, plane_tap=False)
check("the via is a layer change the router routes on next",
      r.success and r.via is not None, f"via={r.via}")

if failures:
    print(f"\nFAILED: {failures}")
    sys.exit(1)
print("\nALL PASSED")

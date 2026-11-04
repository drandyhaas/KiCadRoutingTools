#!/usr/bin/env python3
"""#1547: a #535 off-pad escape stub keeps the full board-edge clearance.

The #189 last-resort via relaxes the board-edge clearance to the pad's own
distance from Edge.Cuts (#448), so an in-pad via adds no edge exposure the
placed pad does not already have. The relaxed value was handed to the off-pad
escape-stub rung as well, so the escape via and the stub trace away from the
pad were placed and routed at the pad's 0.1 mm instead of the board's 0.2032
mm -- a stub run 0.107 mm under an Edge.Cuts slot.

Board: a GND pad 0.1 mm from a slot cut into the outline, boxed by a foreign
net on its other sides so the escape must pass the slot; a positive
--same-net-pad-clearance forbids the in-pad arm, so the off-pad rung runs.

Checks:
  * an escape via and stub are placed (the rung still rescues the pad);
  * the via keeps the full edge clearance;
  * every stub segment not touching the pad keeps it too.

    python3 tests/test_1547_escape_stub_edge_clearance.py
"""
import contextlib
import io
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import single_ended_routing as ser  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from routing_config import GridRouteConfig, GridCoord  # noqa: E402
from check_drc import (board_edge_geometry, _segment_to_rings_distance,  # noqa: E402
                       _point_to_rings_distance, point_to_pad_distance)

fails = []


def check(name, cond, detail=''):
    print(("  ok  " if cond else " FAIL ") + name + (f"  {detail}" if detail else ''))
    if not cond:
        fails.append(name)


EDGE = 0.2032
BOARD = '''(kicad_pcb (version 20240108) (generator test)
 (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (44 "Edge.Cuts" user))
 (net 0 "")
 (net 1 "GND")
 (net 2 "SIG")
 (gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts") (width 0.1))
 (gr_rect (start 10.35 9.0) (end 12.65 10.275) (layer "Edge.Cuts") (width 0.1))
 (footprint "t:d" (layer "F.Cu") (at 10 10)
  (property "Reference" "D1" (at 0 0) (layer "F.SilkS"))
  (pad "1" smd roundrect (at 0 0) (size 0.5 0.55) (roundrect_rratio 0.25)
   (layers "F.Cu" "F.Mask") (net 1 "GND")))
 (segment (start 9.2 8) (end 9.2 12) (width 0.2) (layer "F.Cu") (net 2))
 (segment (start 9.2 9.2) (end 10.5 9.2) (width 0.2) (layer "F.Cu") (net 2))
 (segment (start 9.2 11.0) (end 10.2 11.0) (width 0.2) (layer "F.Cu") (net 2))
)
'''

with tempfile.TemporaryDirectory() as td:
    path = os.path.join(td, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(BOARD)
    pcb = parse_kicad_pcb(path)
check("the slot parses as a board cutout", len(pcb.board_info.board_cutouts) == 1)
pad = pcb.footprints['D1'].pads[0]
rings, _, _ = board_edge_geometry(pcb.board_info, rect_ring=True)
pad_gap = min(_point_to_rings_distance(x, y, rings)
              for x, y in ((10.25, 10.0), (10.25, 9.8), (10.25, 10.2)))
check("the pad itself sits inside the edge band", pad_gap < EDGE - 0.05,
      f"{pad_gap:.3f} mm")

cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], grid_step=0.05, clearance=EDGE,
                      track_width=0.3, via_size=0.45, via_drill=0.2,
                      board_edge_clearance=EDGE, hole_to_hole_clearance=0.254)
cfg.same_net_pad_clearance = 0.1      # no in-pad via: the off-pad rung runs
with contextlib.redirect_stdout(io.StringIO()):
    r = ser._place_shrunk_via_in_pad_impl(pad, None, cfg, pcb, 1,
                                          GridCoord(cfg.grid_step), cfg.layers)
check("an escape via and stub are placed", r is not None and bool(r[3]))
if r is not None:
    via, stub = r[0], r[3]
    vg = _point_to_rings_distance(via.x, via.y, rings) - via.size / 2
    check("the escape via keeps the board-edge clearance", vg >= EDGE - 1e-6,
          f"via ({via.x:.3f},{via.y:.3f}) {vg:.4f} mm")
    away = [s for s in stub
            if min(point_to_pad_distance(s.start_x, s.start_y, pad),
                   point_to_pad_distance(s.end_x, s.end_y, pad)) > 1e-6]
    worst = min((_segment_to_rings_distance(s.start_x, s.start_y, s.end_x,
                                            s.end_y, rings) - s.width / 2
                 for s in away), default=float('inf'))
    check("every stub segment away from the pad keeps the board-edge clearance",
          worst >= EDGE - 1e-6, f"{len(away)} segment(s), nearest {worst:.4f} mm")

print("FAIL" if fails else "PASS", f"({len(fails)} failure(s))" if fails else '')
sys.exit(1 if fails else 0)

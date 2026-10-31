#!/usr/bin/env python3
"""#1545: the plane / last-resort via maps hold a filled copper graphic's
INTERIOR, as the routing map does (#1181).

`plane_obstacle_builder.build_via_obstacle_map` (plane taps, the #189
via-in-pad unblock, its #535 escape stub) stamped a filled graphic's
perimeter segments and nothing inside, so a through-via landed wholly inside
a filled, net-tagged board graphic of another net on a layer it spans -- and
the window `make_local_window` cuts around one pad could miss the shape
altogether when its outline lies outside the window. The trace map
(`build_routing_obstacle_map`) had the same gap on its own layer.

Board: two layers; a filled B.Cu rect of GND, 10 x 10 mm; a SIG pad and a GND
pad on F.Cu over its middle, 4 mm from its outline.

Checks:
  * the window around the SIG pad carries the shape, though no edge of it
    is near the pad;
  * the via map blocks the shape's interior for SIG and leaves it to GND;
    the B.Cu trace map blocks it for SIG;
  * the #189 last-resort via for the SIG pad is not placed inside the shape;
    the GND pad, whose own copper the shape is, still gets one.

    python3 tests/test_1545_via_map_filled_graphic.py
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
from plane_pad_tap import make_local_window  # noqa: E402
from plane_obstacle_builder import (build_via_obstacle_map,  # noqa: E402
                                    build_routing_obstacle_map)
from check_drc import filled_graphic_shapes  # noqa: E402

fails = []


def check(name, cond, detail=''):
    print(("  ok  " if cond else " FAIL ") + name + (f"  {detail}" if detail else ''))
    if not cond:
        fails.append(name)


BOARD = '''(kicad_pcb (version 20260206) (generator test)
 (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
 (net 0 "")
 (net 1 "GND")
 (net 2 "SIG")
 (gr_rect (start 0 0) (end 30 30) (stroke (width 0.1) (type default))
  (fill no) (layer "Edge.Cuts"))
 (gr_rect (start 6 6) (end 16 16) (stroke (width 0.2) (type default))
  (fill yes) (layers "B.Cu") (net "GND"))
 (footprint "t:r" (layer "F.Cu") (at 10 11)
  (property "Reference" "R1" (at 0 0) (layer "F.SilkS"))
  (pad "1" smd rect (at 0 0) (size 1.2 1.2) (layers "F.Cu" "F.Mask") (net 2 "SIG"))
  (pad "2" smd rect (at 2 0) (size 1.2 1.2) (layers "F.Cu" "F.Mask") (net 1 "GND")))
)
'''

with tempfile.TemporaryDirectory() as td:
    path = os.path.join(td, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(BOARD)
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
shapes = filled_graphic_shapes(pcb)
check("the rect parses as one filled GND shape on B.Cu",
      len(shapes) == 1 and shapes[0].layer == 'B.Cu' and shapes[0].net_id == 1)
sig, gnd = sorted(pcb.footprints['R1'].pads, key=lambda p: p.pad_number)
cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], grid_step=0.05, clearance=0.1,
                      track_width=0.15, via_size=0.45, via_drill=0.3)
coord = GridCoord(cfg.grid_step)

local = make_local_window(pcb, sig.global_x, sig.global_y, 1.5)
check("the window around the SIG pad carries the shape",
      len(filled_graphic_shapes(local)) == 1,
      f"{len(local.segments)} segment(s) in the window")

gx, gy = coord.to_grid(sig.global_x, sig.global_y)
q = coord.to_grid(9.0, 12.0)    # inside the shape and the window, off every pad
with contextlib.redirect_stdout(io.StringIO()):
    vm_sig = build_via_obstacle_map(local, cfg, 2, verbose=False)
    vm_gnd = build_via_obstacle_map(local, cfg, 1, verbose=False)
    tm_sig = build_routing_obstacle_map(local, cfg, 2, 'B.Cu', verbose=False)
check("via map: the interior blocks a SIG via", vm_sig.is_via_blocked(*q))
check("via map: the interior is GND's own copper", not vm_gnd.is_via_blocked(*q))
check("B.Cu trace map: the interior blocks a SIG track", tm_sig.is_blocked(q[0], q[1], 0))

with contextlib.redirect_stdout(io.StringIO()):
    r_sig = ser._place_shrunk_via_in_pad_impl(sig, None, cfg, pcb, 2, coord, cfg.layers)
inside = r_sig is not None and shapes[0].contains(r_sig[0].x, r_sig[0].y)
check("no SIG last-resort via inside the GND shape", not inside,
      f"via at ({r_sig[0].x:.3f},{r_sig[0].y:.3f})" if r_sig else "none placed")
with contextlib.redirect_stdout(io.StringIO()):
    r_gnd = ser._place_shrunk_via_in_pad_impl(gnd, None, cfg, pcb, 1, coord, cfg.layers)
check("the GND pad still gets its in-pad via", r_gnd is not None)

print("FAIL" if fails else "PASS", f"({len(fails)} failure(s))" if fails else '')
sys.exit(1 if fails else 0)

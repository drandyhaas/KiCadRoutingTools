#!/usr/bin/env python3
"""#1546: a route continuing a stub keeps the clearance the stub kept to its
neighbour.

route_diff leaves a pair member at a stub end beside its partner, and
route.py then continues it from there. Two ways the continuation landed
closer to the partner than the stub itself:

  A. The launch cell. A row's cell is the NEAREST grid point of the stub's
     tip, which for an off-grid tip can lie toward the partner; the cell is
     then blocked, the A* enters it only through the endpoint override, and
     the tip -> grid-point connector and the first leg sit 0.071 mm from the
     partner at a 0.1 rule. The launch now takes the nearest corner of the
     tip's grid square the map leaves free (`_clear_launch_rows`).
  B. The terminal neck's floor. A wider route welded to a 0.1 mm stub end
     0.1 mm from the partner necks down -- but only to the 2-layer fab floor
     0.127, which still grazes (0.087 mm). The net's own 0.1 mm track ends
     there already, so the neck may take the terminal to that width
     (`_joined_track_width`).

Each case is a tiny locked P/N stub pair routed through the real engine.

    python3 tests/test_1546_stub_continuation_clearance.py
"""
import contextlib
import io
import math
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import parse_kicad_pcb  # noqa: E402
from route import batch_route  # noqa: E402

fails = []


def check(name, cond, detail=''):
    print(("  ok  " if cond else " FAIL ") + name + (f"  {detail}" if detail else ''))
    if not cond:
        fails.append(name)


HEAD = '''(kicad_pcb (version 20240108) (generator test)
 (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (44 "Edge.Cuts" user))
 (net 0 "")
 (net 1 "P")
 (net 2 "N")
 (gr_rect (start 0 0) (end 14 14) (layer "Edge.Cuts") (width 0.1))
'''

# A: coupled 45-degree stubs, 0.2 mm apart centre to centre, tips off-grid;
# P's other pad lies along the tip's grid row, past the partner's side.
OFF_GRID = HEAD + ''' (footprint "t:a" (layer "F.Cu") (at 7 11)
  (property "Reference" "J1" (at 0 0) (layer "F.SilkS"))
  (pad "1" smd rect (at 0 0) (size 0.3 0.3) (layers "F.Cu" "F.Mask") (net 1 "P"))
  (pad "2" smd rect (at -0.25 0) (size 0.2 0.3) (layers "F.Cu" "F.Mask") (net 2 "N")))
 (footprint "t:b" (layer "F.Cu") (at 10.5 9.3)
  (property "Reference" "J2" (at 0 0) (layer "F.SilkS"))
  (pad "1" smd rect (at 0 0) (size 0.3 0.3) (layers "F.Cu" "F.Mask") (net 1 "P")))
 (segment (start 7.0 11.0) (end 7.0 9.4) (width 0.1) (layer "F.Cu") (net 1) (locked yes))
 (segment (start 7.0 9.4) (end 7.020711 9.320711) (width 0.1) (layer "F.Cu") (net 1) (locked yes))
 (segment (start 6.75 11.0) (end 6.75 9.308579) (width 0.1) (layer "F.Cu") (net 2) (locked yes))
 (segment (start 6.75 9.308579) (end 6.879289 9.179289) (width 0.1) (layer "F.Cu") (net 2) (locked yes))
 (segment (start 6.6 9.2) (end 6.879289 9.179289) (width 0.1) (layer "F.Cu") (net 2) (locked yes))
)
'''

# B: parallel 0.1 mm stubs 0.1 mm apart, P's tip on grid; P continues at
# 0.25 mm and a 0.14 mm clearance, so its terminal must neck.
NECK = HEAD + ''' (footprint "t:a" (layer "F.Cu") (at 7 12)
  (property "Reference" "J1" (at 0 0) (layer "F.SilkS"))
  (pad "1" smd rect (at 0 0) (size 0.1 0.1) (layers "F.Cu" "F.Mask") (net 1 "P"))
  (pad "2" smd rect (at 0.2 0) (size 0.1 0.1) (layers "F.Cu" "F.Mask") (net 2 "N")))
 (footprint "t:b" (layer "F.Cu") (at 4 7)
  (property "Reference" "J2" (at 0 0) (layer "F.SilkS"))
  (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu" "F.Mask") (net 1 "P")))
 (segment (start 7.0 12.0) (end 7.0 9.3) (width 0.1) (layer "F.Cu") (net 1) (locked yes))
 (segment (start 7.2 12.0) (end 7.2 9.1) (width 0.1) (layer "F.Cu") (net 2) (locked yes))
)
'''


def seg_gap(a, b):
    """Edge-to-edge distance between two segments (sampled along `a`)."""
    best = float('inf')
    dx, dy = b.end_x - b.start_x, b.end_y - b.start_y
    L2 = dx * dx + dy * dy
    for i in range(401):
        t = i / 400
        x = a.start_x + (a.end_x - a.start_x) * t
        y = a.start_y + (a.end_y - a.start_y) * t
        u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - b.start_x) * dx
                                                   + (y - b.start_y) * dy) / L2))
        best = min(best, math.hypot(x - b.start_x - u * dx, y - b.start_y - u * dy))
    return best - (a.width + b.width) / 2


def route(board, **kw):
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, 'b.kicad_pcb')
        with open(p, 'w', encoding='utf-8') as f:
            f.write(board)
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(p)
            ok, fail, _t, data = batch_route(
                p, '', ['P'], pcb_data=pcb, return_results=True,
                layers=['F.Cu', 'B.Cu'], via_size=0.45, via_drill=0.2, **kw)
    new = [s for r in data['results'] for s in (r.get('new_segments') or [])]
    partner = [s for s in pcb.segments if s.net_id == 2]
    gap = min((seg_gap(s, n) for s in new for n in partner), default=float('inf'))
    return ok, fail, new, gap


print("A. an off-grid stub tip launches from a free grid point")
ok, fail, new, gap = route(OFF_GRID, track_width=0.1, clearance=0.1, grid_step=0.05)
check("P routes", ok == 1 and fail == 0 and bool(new))
check("the continuation keeps 0.1 mm to N", gap >= 0.1 - 1e-4, f"nearest {gap:.4f} mm")

print("B. the terminal on a 0.1 mm stub end necks to the stub's width")
ok, fail, new, gap = route(NECK, track_width=0.25, clearance=0.14, grid_step=0.1)
check("P routes", ok == 1 and fail == 0 and bool(new))
term = [s for s in new if (abs(s.start_x - 7.0) < 1e-6 and abs(s.start_y - 9.3) < 1e-6)
        or (abs(s.end_x - 7.0) < 1e-6 and abs(s.end_y - 9.3) < 1e-6)]
check("the terminal is no wider than the stub it welds to",
      bool(term) and max(s.width for s in term) <= 0.1 + 1e-9,
      str([s.width for s in term]))
check("... so it keeps the 0.1 mm the stub keeps to N", gap >= 0.1 - 1e-4,
      f"nearest {gap:.4f} mm")

print("FAIL" if fails else "PASS", f"({len(fails)} failure(s))" if fails else '')
sys.exit(1 if fails else 0)

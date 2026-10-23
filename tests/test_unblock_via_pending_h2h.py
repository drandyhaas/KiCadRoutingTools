#!/usr/bin/env python3
"""A #189/#535 unblock via holds hole-to-hole off the net's own vias that the
routing call has laid but not yet committed.

_place_shrunk_via_in_pad builds its via map from pcb_data (plus foreign
in-flight copper). Same-net copper is no obstacle, but a drill is a drill:
KiCad grades hole_to_hole whatever the nets. Two kinds of same-net via were
invisible to it because they are not in pcb_data yet:
  * the via the SAME _route_with_via_unblock call just dropped for the
    route's other end (glasgow_revC on main: two /GNDPLL0 #535 escape vias
    0.05 mm apart, target end then source end, in contact);
  * the vias of the tap call's earlier edges (glasgow_revC on main: a +3V3
    in-pad via 0.071 mm from an earlier edge's via).
The placement is now handed both as `pending_vias`.

Cases (a 1.2 mm F.Cu GND pad at (10,10); a pending GND via at its centre):
  * blind (no pending vias) -- the control: the in-pad via lands inside
    hole-to-hole of the pending via, the defect;
  * shown the pending via: whatever it places is >= hole-to-hole off it;
  * _route_with_via_unblock shows every placement the vias it already
    placed, and the tap caller passes its earlier edges' vias (AST).

    python3 tests/test_unblock_via_pending_h2h.py
"""
import ast
import contextlib
import inspect
import io
import math
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import single_ended_routing as ser  # noqa: E402
from kicad_parser import Via, parse_kicad_pcb  # noqa: E402
from routing_config import GridRouteConfig, GridCoord  # noqa: E402

fails = []


def check(name, cond, detail=''):
    print(("  ok  " if cond else " FAIL ") + name + (f"  {detail}" if detail else ''))
    if not cond:
        fails.append(name)


BOARD = '''(kicad_pcb (version 20240108) (generator test)
 (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (44 "Edge.Cuts" user))
 (net 0 "")
 (net 1 "GND")
 (gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts") (width 0.1))
 (footprint "t:c" (layer "F.Cu") (at 10 10)
  (property "Reference" "C1" (at 0 0) (layer "F.SilkS"))
  (pad "2" smd rect (at 0 0) (size 1.2 1.2) (layers "F.Cu" "F.Mask") (net 1 "GND")))
)
'''
H2H = 0.25
CFG = dict(layers=['F.Cu', 'B.Cu'], grid_step=0.05, clearance=0.1, track_width=0.15,
           via_size=0.4, via_drill=0.2, hole_to_hole_clearance=H2H)
HAS_PENDING = 'pending_vias' in inspect.signature(ser._place_shrunk_via_in_pad).parameters


def place(pending):
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, 'b.kicad_pcb')
        with open(p, 'w', encoding='utf-8') as f:
            f.write(BOARD)
        pcb = parse_kicad_pcb(p)
    cfg = GridRouteConfig(**CFG)
    pad = pcb.footprints['C1'].pads[0]
    kw = {'pending_vias': pending} if (pending and HAS_PENDING) else {}
    with contextlib.redirect_stdout(io.StringIO()):
        r = ser._place_shrunk_via_in_pad(pad, None, cfg, pcb, 1,
                                         GridCoord(cfg.grid_step), cfg.layers, **kw)
    return r[0] if r else None


PEND = Via(x=10.0, y=10.0, size=0.4, drill=0.2, layers=['F.Cu', 'B.Cu'], net_id=1)


def h2h_gap(v):
    return math.hypot(v.x - PEND.x, v.y - PEND.y) - (v.drill + PEND.drill) / 2


blind = place(None)
check("control: blind, the unblock via lands inside hole-to-hole of the "
      "pending via", blind is not None and h2h_gap(blind) < H2H - 1e-6,
      f"via {blind and (blind.x, blind.y)}, drill gap "
      f"{blind and round(h2h_gap(blind), 3)}")
shown = place([PEND])
check("shown the pending via: none placed, or >= hole-to-hole off it",
      shown is None or h2h_gap(shown) >= H2H - 1e-6,
      f"via {shown and (shown.x, shown.y)}, drill gap "
      f"{shown and round(h2h_gap(shown), 3)}")

# The callers hand the placement what they have laid.
tree = ast.parse(open(os.path.join(ROOT, 'py_router', 'single_ended_routing.py'),
                      encoding='utf-8').read())
fns = {n.name: n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)}


def calls(fn, name):
    return [c for c in ast.walk(fn) if isinstance(c, ast.Call)
            and isinstance(c.func, ast.Name) and c.func.id == name]


def kw(c, name):
    return next((k.value for k in c.keywords if k.arg == name), None)


unb = calls(fns['_route_with_via_unblock'], '_place_shrunk_via_in_pad')
check("_route_with_via_unblock: every placement is shown the vias it already "
      "placed", len(unb) >= 3 and all(
          kw(c, 'pending_vias') is not None
          and 'placed' in ast.unparse(kw(c, 'pending_vias')) for c in unb),
      f"{len(unb)} placement call(s)")
taps = fns['_route_multipoint_taps_impl']
tw = calls(taps, '_route_with_via_unblock')
tr = calls(taps, '_place_shrunk_via_in_pad')
check("the tap call passes its earlier edges' vias to both rescue paths",
      tw and tr and all(kw(c, 'pending_vias') is not None
                        and 'all_vias' in ast.unparse(kw(c, 'pending_vias'))
                        for c in tw + tr))

print("-" * 60)
if fails:
    print(f"{len(fails)} FAILED: " + "; ".join(fails))
    sys.exit(1)
print("ALL PASS")

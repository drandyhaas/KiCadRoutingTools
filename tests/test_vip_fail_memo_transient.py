#!/usr/bin/env python3
"""The #189 in-pad via failure memo also answers under pending / in-flight
copper, keyed on that copper's exact content.

_place_shrunk_via_in_pad used to bypass its failure memo whenever the caller
handed `pending_vias` or phase-3 in-flight copper was open, so a boxed pad
re-ran the whole fab-ladder tap search with the same pending via on every
rescue rung (tigard: 191 identical re-failures, 74 s of a 513 s run). The
memo now keys on the transient copper too: an identical state is answered,
any different state (another pending via, none, other in-flight copper)
still runs the placement.

Cases (a stub impl that always fails and counts its calls):
  * same pending via twice -> impl runs once;
  * a different pending via -> impl runs again;
  * no pending via after a pending failure -> impl runs (and vice versa);
  * in-flight copper keyed by content, not by presence;
  * a success is never memoised;
  * real board: the pending-via failure answer matches the impl's.

    python3 tests/test_vip_fail_memo_transient.py
"""
import contextlib
import io
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import single_ended_routing as ser  # noqa: E402
from kicad_parser import Segment, Via, parse_kicad_pcb  # noqa: E402
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
 (net 2 "SIG")
 (gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts") (width 0.1))
 (footprint "t:c" (layer "F.Cu") (at 10 10)
  (property "Reference" "C1" (at 0 0) (layer "F.SilkS"))
  (pad "2" smd rect (at 0 0) (size 1.2 1.2) (layers "F.Cu" "F.Mask") (net 1 "GND")))
)
'''
CFG = dict(layers=['F.Cu', 'B.Cu'], grid_step=0.05, clearance=0.1, track_width=0.15,
           via_size=0.4, via_drill=0.2, hole_to_hole_clearance=0.25)


def board():
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, 'b.kicad_pcb')
        with open(p, 'w', encoding='utf-8') as f:
            f.write(BOARD)
        return parse_kicad_pcb(p)


def via(x, y):
    return Via(x=x, y=y, size=0.4, drill=0.2, layers=['F.Cu', 'B.Cu'], net_id=1)


cfg = GridRouteConfig(**CFG)
coord = GridCoord(cfg.grid_step)

# --- stub impl: counts calls, returns a scripted result -------------------
calls = []
result = [None]
real_impl = ser._place_shrunk_via_in_pad_impl


def stub(pad, obstacles, config, pcb, net_id, coord, layer_names, pending_vias=None):
    calls.append(pending_vias)
    return result[0]


ser._place_shrunk_via_in_pad_impl = stub
try:
    pcb = board()
    pad = pcb.footprints['C1'].pads[0]

    def place(pending=None):
        n = len(calls)
        ser._place_shrunk_via_in_pad(pad, None, cfg, pcb, 1, coord, cfg.layers,
                                     pending_vias=pending)
        return len(calls) - n

    check("first failure with a pending via runs the impl", place([via(10, 10)]) == 1)
    check("identical pending via: answered from the memo", place([via(10, 10)]) == 0)
    check("a different pending via runs the impl", place([via(10.3, 10)]) == 1)
    check("no pending via: runs the impl (not answered by a pending failure)",
          place(None) == 1)
    check("no pending via again: answered from the memo", place(None) == 0)
    check("two pending vias: own key", place([via(10, 10), via(10.3, 10)]) == 1)

    pcb._copper_epoch = getattr(pcb, '_copper_epoch', 0) + 1
    check("epoch bump: identical pending via runs the impl again",
          place([via(10, 10)]) == 1)

    seg = Segment(start_x=9.0, start_y=9.0, end_x=11.0, end_y=9.0, width=0.15,
                  layer='B.Cu', net_id=2)
    pcb._inflight_copper = [([seg], [])]
    check("in-flight copper open: runs the impl", place(None) == 1)
    check("same in-flight copper: answered from the memo", place(None) == 0)
    seg2 = Segment(start_x=9.0, start_y=9.2, end_x=11.0, end_y=9.2, width=0.15,
                   layer='B.Cu', net_id=2)
    pcb._inflight_copper = [([seg2], [])]
    check("different in-flight copper: runs the impl", place(None) == 1)
    pcb._inflight_copper = []

    result[0] = ('via', (0, 0), 0, [])
    check("success: runs the impl", place([via(5, 5)]) == 1)
    check("success is never memoised", place([via(5, 5)]) == 1)
finally:
    ser._place_shrunk_via_in_pad_impl = real_impl

# --- real impl: the memo's answer equals a fresh placement ----------------
# A pending via at the pad centre with hole-to-hole 0.25 leaves no legal
# in-pad site for a fresh via unless it goes off-pad; whatever the impl says
# the first time, the memoised second call must say the same.
pcb = board()
pad = pcb.footprints['C1'].pads[0]
blocker = [via(10, 10)]
with contextlib.redirect_stdout(io.StringIO()):
    first = ser._place_shrunk_via_in_pad(pad, None, cfg, pcb, 1, coord, cfg.layers,
                                         pending_vias=blocker)
    fresh = ser._place_shrunk_via_in_pad_impl(pad, None, cfg, pcb, 1, coord,
                                              cfg.layers, pending_vias=blocker)
    second = ser._place_shrunk_via_in_pad(pad, None, cfg, pcb, 1, coord, cfg.layers,
                                          pending_vias=blocker)
check("real impl: memoised answer agrees with a fresh placement",
      (first is None) == (fresh is None) == (second is None),
      f"first {first is not None}, fresh {fresh is not None}, "
      f"second {second is not None}")

print("-" * 60)
if fails:
    print(f"{len(fails)} FAILED: " + "; ".join(fails))
    sys.exit(1)
print("ALL PASS")

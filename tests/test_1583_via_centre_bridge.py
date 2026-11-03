#!/usr/bin/env python3
"""#1583 part B / #1235: a track ending inside a via off its centre is cut
back to where it enters the via and bridged straight to the centre, and the
path measure charges that line before the bridge is laid.

KiCad runs a track into a via only through its centre: 10.0.4+ cuts a run
ending in a via pad at the pad edge and measures it straight to the centre,
10.0.3 joins a via only to tracks ending at its centre. The router's path
measure joined such an end for free, so the intra-pair matcher matched a pair
to a length KiCad never measures (0.0006 mm matched, 0.76 mm shipped).

Checks, on one route -- pad A, an F.Cu track ending 0.22 mm off a via's
centre, the via, a B.Cu track, pad B:
  1. close_soft_joints cuts the routed track at the via's edge and lays a
     bridge from there to the centre, a protected cleanup kind; the end is
     then no longer an off-centre end (the check_weird definition).
  2. The path measure reads the same length before the bridge and after it,
     and that is the copper's own length.
  3. KiCad's length model reads that length too, for 10.0.4+ and 10.0.3.
  4. Copper this run did not lay is never cut: it gets a bridge from where
     it ends.

    python3 tests/test_1583_via_centre_bridge.py
"""
import math
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from connectivity import off_centre_via_ends                  # noqa: E402
from kicad_parser import BoardInfo                             # noqa: E402
from net_queries import (kicad_net_length_nm, net_signal_length,  # noqa: E402
                         pin_pair_path_length)
from pcb_modification import CLOSE_SOFT_JOINT_KINDS, close_soft_joints  # noqa: E402
from routing_config import GridRouteConfig                     # noqa: E402
from synth import make_pad, make_pcb, make_seg, make_via       # noqa: E402

failures = []
R = 0.3                     # the via's radius
CX, CY = 5.0, 0.0           # its centre
P = (4.9, 0.2)              # the track's end, 0.2236 mm off the centre


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}" + (f": {detail}" if detail and not ok else ''))
    if not ok:
        failures.append(label)


def board():
    pa = make_pad(1, 0.0, 0.0, ref='U1', num='1', layers=('F.Cu',))
    pb = make_pad(1, 10.0, 0.0, ref='U2', num='1', layers=('B.Cu',))
    t1 = make_seg(0.0, 0.0, P[0], P[1], layer='F.Cu', width=0.15)
    t2 = make_seg(CX, CY, 10.0, 0.0, layer='B.Cu', width=0.15)
    via = make_via(CX, CY, size=2 * R, drill=0.3)
    info = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'}, copper_layers=['F.Cu', 'B.Cu'],
                     board_bounds=(-2, -2, 12, 2))
    pcb = make_pcb(segments=[t1, t2], vias=[via], pads_by_net={1: [pa, pb]},
                   board_info=info)
    return pcb, t1, t2, via, pa, pb


def cfg():
    return GridRouteConfig(clearance=0.15, track_width=0.15, via_size=0.6,
                           via_drill=0.3, layers=['F.Cu', 'B.Cu'], grid_step=0.05)


def entry_distance():
    """Along t1 from pad A, where it crosses into the via's disc."""
    L = math.hypot(*P)
    ux, uy = P[0] / L, P[1] / L
    b = -CX * ux - CY * uy
    return -b - math.sqrt(b * b - (CX * CX + CY * CY - R * R))


print("1. the routed track is cut at the via's edge and bridged to the centre")
pcb, t1, t2, via, pa, pb = board()
before = pin_pair_path_length(pcb, 1, pa, pb)
results = [{'new_segments': [t1, t2], 'new_vias': [via]}]
n = close_soft_joints(results, pcb, {1}, cfg())
bridge = [r for r in results if r.get('cleanup') == 'via_centre_bridge']
b = bridge[0]['new_segments'][0] if bridge else None
check("one bridge, a protected kind", n == 1 and b is not None
      and 'via_centre_bridge' in CLOSE_SOFT_JOINT_KINDS, f"{n} {results[1:]}")
te = entry_distance()
check("the track now ends where it enters the via",
      abs(math.hypot(t1.end_x - CX, t1.end_y - CY) - R) < 1e-9
      and abs(math.hypot(t1.end_x, t1.end_y) - te) < 1e-9,
      f"({t1.end_x:.4f}, {t1.end_y:.4f})")
check("the bridge runs from there to the centre", b is not None
      and (b.start_x, b.start_y) == (t1.end_x, t1.end_y)
      and (b.end_x, b.end_y) == (CX, CY) and b.width == t1.width and b.layer == 'F.Cu')
check("no off-centre end is left", off_centre_via_ends(pcb.segments, pcb.vias,
                                                       ['F.Cu', 'B.Cu']) == [])

print("2. the path measure reads the copper's length before and after")
after = pin_pair_path_length(pcb, 1, pa, pb)
barrel = pcb.get_via_barrel_length('F.Cu', 'B.Cu')
copper = te + R + barrel + 5.0
check("before the bridge: pad entry straight to the centre", abs(before - copper) < 1e-6,
      f"{before:.6f} vs {copper:.6f}")
check("after it: the same", abs(after - copper) < 1e-6, f"{after:.6f}")
check("net_signal_length agrees", abs(net_signal_length(pcb, 1) - copper) < 1e-6)

print("3. KiCad's length model reads it too")
for rel in ('10.0.6', '10.0.3'):
    k = kicad_net_length_nm(pcb, pcb.segments, pcb.vias, [pa, pb], release=rel) / 1e6
    check(f"KiCad {rel}", abs(k - after) < 1e-3, f"{k:.6f} vs {after:.6f}")

print("4. copper this run did not lay is bridged from where it ends")
pcb, t1, t2, via, pa, pb = board()
results = []
close_soft_joints(results, pcb, {1}, cfg())
b = [r for r in results if r.get('cleanup') == 'via_centre_bridge']
b = b[0]['new_segments'][0] if b else None
check("the track is not cut", (t1.end_x, t1.end_y) == P)
check("the bridge starts at its end", b is not None and (b.start_x, b.start_y) == P
      and (b.end_x, b.end_y) == (CX, CY))
check("no off-centre end is left", off_centre_via_ends(pcb.segments, pcb.vias,
                                                       ['F.Cu', 'B.Cu']) == [])

print()
if failures:
    print(f"FAIL: {len(failures)} check(s): {failures}")
    sys.exit(1)
print("PASS: test_1583_via_centre_bridge")

#!/usr/bin/env python3
"""#1318: a power route's neck zone is laid at the pad-entry width wherever it
fits, and necked only where it does not.

_neck_pass narrowed the whole neckdown_length (2.5 mm) zone at each pad end of
a necked or stepped-down power route, fit or not. A short edge lies entirely
inside those zones, so on steppenprobe every 1 mm U1 pin-to-pin GND run and
every ~1.2 mm pad-to-via drop shipped at the 0.15 signal width end to end
under a 0.3 mm --power-nets-widths request (13 GND short edges, 16.67 mm).

The zone is now fit-checked like the widen-back beyond it, at min(the net's
width, the short side of the pad that end lands on), and narrowed only where
that width does not fit.

Checks (_assign_wide_route_widths on synthetic geometry, net width 0.3):
  1. A short edge between two 0.2-wide pads in free space: the zones ship at
     the pad's 0.2, never wider than the pad; the middle at 0.3.
  2. An end on no pad (a via, a track): its zone ships at the net's 0.3.
  3. A pinch INSIDE a zone: only the pieces the pinch blocks are necked; the
     rest of the zone keeps the wide width.
  4. A long trunk (the necked_down path) gets the same zones.
  5. Nothing ships below the edge's width, and length is conserved.

    python3 tests/test_1318_neck_zone_fit.py
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, HERE)

from synth import make_seg                                         # noqa: E402
from kicad_parser import Pad                                       # noqa: E402
from routing_config import GridRouteConfig, GridCoord              # noqa: E402
from obstacle_map import GridObstacleMap                           # noqa: E402
from single_ended_routing import _assign_wide_route_widths         # noqa: E402

fails = []


def check(name, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {name}{('  ' + str(detail)) if detail else ''}")
    if not ok:
        fails.append(name)


class _Board:
    def __init__(self, pads):
        self.pads_by_net = {1: pads}


def pad(x, y, w, h=1.0, ref='U1', num='1'):
    return Pad(component_ref=ref, pad_number=num, global_x=x, global_y=y,
               local_x=0.0, local_y=0.0, size_x=w, size_y=h, shape='rect',
               layers=['F.Cu'], net_id=1, net_name='GND')


cfg = GridRouteConfig(layers=['F.Cu'], track_width=0.127, clearance=0.1, grid_step=0.1)
cfg.power_net_widths = {1: 0.3}
coord = GridCoord(cfg.grid_step)


def run(segs, board, obstacles=None, margin=0.0, necked=False, uniform=0.15):
    return _assign_wide_route_widths(
        segs, cfg, 1, obstacles or GridObstacleMap(1), coord, ['F.Cu'], margin,
        necked, None if necked else uniform, neck_start=True, pcb_data=board)


def L(s):
    return math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)


def width_at(out, x):
    """Width of the piece covering x on a run along y = 0."""
    for s in out:
        lo, hi = sorted((s.start_x, s.end_x))
        if lo - 1e-9 <= x <= hi + 1e-9:
            return round(s.width, 4)
    return None


def edge(x0=0.0, x1=9.0):
    return [make_seg(x0, 0, (x0 + x1) / 2, 0, width=0.3),
            make_seg((x0 + x1) / 2, 0, x1, 0, width=0.3)]


print("1. a short edge between two 0.2-wide pads, free space")
out = run(edge(), _Board([pad(0, 0, 0.2, num='1'), pad(9, 0, 0.2, num='2')]))
check("each zone ships at the pad's 0.2, never wider than the pad",
      width_at(out, 0.5) == 0.2 and width_at(out, 8.5) == 0.2,
      [(round(s.start_x, 2), round(s.width, 3)) for s in out])
check("the middle ships at the net's 0.3", width_at(out, 4.5) == 0.3)
check("nothing below the edge width, length conserved",
      min(s.width for s in out) >= 0.15 - 1e-9 and abs(sum(map(L, out)) - 9.0) < 1e-6)

print("2. an end on no pad")
out = run(edge(), _Board([pad(0, 0, 0.2)]))
check("the padded end ships at 0.2, the pad-less end at the net's 0.3",
      width_at(out, 0.5) == 0.2 and width_at(out, 8.5) == 0.3,
      [(round(s.start_x, 2), round(s.width, 3)) for s in out])

print("3. a pinch inside a zone")
mp = GridObstacleMap(1)
for gx in (5, 6, 7):                       # x 0.5..0.7 mm, one cell off the run
    mp.add_blocked_cell(gx, 1, 0)
out = run(edge(), _Board([pad(0, 0, 0.6, num='1'), pad(9, 0, 0.6, num='2')]),
          obstacles=mp, margin=1.5)
check("the pinched pieces are necked to the edge width",
      width_at(out, 0.6) == 0.15, [(round(s.start_x, 2), round(s.width, 3)) for s in out])
check("the rest of that zone keeps the wide width",
      width_at(out, 1.8) == 0.3 and width_at(out, 8.5) == 0.3)
# The fit test cuts on-grid runs into ~0.5 mm pieces, so the pinch necks the
# two pieces it touches (x 0..1.0); the stepped taper back to 0.3 follows.
check("the pinch is local: only the pieces it touches ship at the edge width",
      sum(L(s) for s in out if abs(s.width - 0.15) < 1e-9) <= 1.0 + 1e-6
      and width_at(out, 2.0) == 0.3,
      [(round(s.start_x, 2), round(s.end_x, 2), round(s.width, 3)) for s in out])

print("4. a long trunk (necked_down)")
out = run(edge(0.0, 20.0), _Board([pad(0, 0, 0.2, num='1'), pad(20, 0, 0.2, num='2')]),
          necked=True)
check("its zones ship at the pad's 0.2 and the trunk at 0.3",
      width_at(out, 0.5) == 0.2 and width_at(out, 19.5) == 0.2 and width_at(out, 10) == 0.3,
      [(round(s.start_x, 2), round(s.width, 3)) for s in out])

if fails:
    print(f"\nFAILED: {fails}")
    sys.exit(1)
print("\nALL PASSED")

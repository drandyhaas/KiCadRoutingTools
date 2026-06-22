#!/usr/bin/env python3
"""
Regression test for issue #165 (tier 2, root-cause 2): launching diff-pair
connectors from the pad EDGE facing the route, not the pad center.

`_pad_edge_launch` moves a bare-pad connector's launch point from the pad center
to the pad edge in the direction of the route, so the connector leaves the pad
toward the route instead of cutting back across a long (e.g. 1.45mm USB-C) pad's
field and grazing a neighbour. It must:
  - shift to (just inside) the pad edge along the route direction,
  - stay inside the pad copper (so the track still attaches),
  - never launch past the route's first point (short connectors),
  - leave non-pad (stub) launch points untouched.

Run:
    python3 tests/test_diff_pad_end_launch.py
"""

import math
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

from kicad_parser import Pad, PCBData
from routing_config import GridRouteConfig
from diff_pair_routing import _pad_edge_launch

NET = 10
HX, HY = 0.15, 0.725  # half-extents of a 0.3 x 1.45 mm USB-C-style pad


def _pad(x, y, sx=2 * HX, sy=2 * HY, rot=0.0, shape='roundrect'):
    return Pad('J1', 'A6', x, y, 0, 0, sx, sy, shape, ['F.Cu'],
               NET, '/USB_DP', 0, None, 0.0, None, 0.25, rot, None)


def _cfg():
    c = GridRouteConfig()
    c.track_width = 0.2
    return c


def _pcb(pads):
    return PCBData(footprints={}, nets={}, segments=[], vias=[],
                   board_info=None, pads_by_net={NET: pads})


def _inside(pad, x, y, eps=1e-6):
    """Point inside the (possibly rotated) pad rectangle?"""
    dx, dy = x - pad.global_x, y - pad.global_y
    if pad.rect_rotation:
        rad = math.radians(pad.rect_rotation)
        cr, sr = math.cos(rad), math.sin(rad)
        dx, dy = dx * cr + dy * sr, -dx * sr + dy * cr
    return abs(dx) <= pad.size_x / 2 + eps and abs(dy) <= pad.size_y / 2 + eps


def main():
    cfg = _cfg()
    results = []

    def check(name, ok, detail=""):
        results.append((name, ok))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}{('  ' + detail) if detail else ''}")

    # 1. Long axis: route far above the pad (+y) -> launch near the top edge.
    pcb = _pcb([_pad(0.0, 0.0)])
    x, y, _ = _pad_edge_launch(pcb, NET, 0.0, 0.0, 0.0, 5.0, cfg)
    check("long-axis launch moves toward route edge", y > HY * 0.6,
          f"y={y:.3f} (edge={HY:.3f})")
    check("long-axis launch stays inside pad", _inside(pcb.pads_by_net[NET][0], x, y),
          f"({x:.3f},{y:.3f})")

    # 2. Narrow axis: route to +x -> small shift bounded by the narrow half-extent.
    x, y, _ = _pad_edge_launch(pcb, NET, 0.0, 0.0, 5.0, 0.0, cfg)
    check("narrow-axis launch bounded by half-width", 0 < x < HX,
          f"x={x:.3f} (half-width={HX:.3f})")

    # 3. Short connector: route point only 0.2mm away -> must not overshoot it.
    x, y, _ = _pad_edge_launch(pcb, NET, 0.0, 0.0, 0.0, 0.2, cfg)
    check("short connector not launched past route start", 0 <= y <= 0.2 + 1e-9,
          f"y={y:.3f} (route at 0.2)")

    # 4. Stub endpoint: no pad at the launch point -> unchanged.
    pcb_far = _pcb([_pad(10.0, 10.0)])
    x, y, _ = _pad_edge_launch(pcb_far, NET, 0.0, 0.0, 0.0, 5.0, cfg)
    check("non-pad (stub) launch left untouched", x == 0.0 and y == 0.0,
          f"({x:.3f},{y:.3f})")

    # 5. Diagonal pad (rect_rotation): launch still lands inside the rotated rect.
    pcb_rot = _pcb([_pad(0.0, 0.0, rot=30.0)])
    x, y, _ = _pad_edge_launch(pcb_rot, NET, 0.0, 0.0, 3.0, 5.0, cfg)
    check("diagonal pad launch stays inside rotated rect",
          _inside(pcb_rot.pads_by_net[NET][0], x, y), f"({x:.3f},{y:.3f})")
    check("diagonal pad launch actually moved", (x * x + y * y) > 1e-6)

    # 6. Round (circle) pad, route at 45 deg: the launch must stay inside the
    # CIRCLE, not the rect bounding box (which would put it in a corner off-copper).
    R = 0.5
    pcb_c = _pcb([_pad(0.0, 0.0, sx=2 * R, sy=2 * R, shape='circle')])
    x, y, _ = _pad_edge_launch(pcb_c, NET, 0.0, 0.0, 5.0, 5.0, cfg)  # route up-right (45 deg)
    check("circle pad launch stays inside the circle (not the bbox corner)",
          math.hypot(x, y) <= R + 1e-6, f"r={math.hypot(x, y):.3f} (R={R})")
    check("circle pad launch actually moved", (x * x + y * y) > 1e-6)

    passed = sum(1 for _, ok in results if ok)
    total = len(results)
    print("\n" + "=" * 60)
    print(f"  {passed}/{total} checks passed")
    print("=" * 60)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())

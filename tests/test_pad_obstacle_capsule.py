#!/usr/bin/env python3
"""
Regression test for issue #289 (partner-pad capsule over-block seals hybrid
leg corridors).

`_pad_obstacle_segments` (diff_pair_routing) approximates a partner-net pad as
capsule segment(s) for a hybrid leg's track-obstacle list. It used to span the
FULL long axis with cap radius = short/2, so the caps bulged short/2 past both
pad ends -- for a square pad that DOUBLES its footprint. On ecp5_mini's 1.27mm
header the pair's own partner 1.7x1.7 pin pad then sealed the only escape
corridor between the 2.54mm-pitch pin rows, and every hybrid leg (and so 29/34
diff pairs) failed with "terminal legs could not attach to the coupled middle".

The fix insets the capsule spine by short/2 (caps flush with the pad ends):
  - circle/oval: that stadium IS the exact pad copper.
  - rect/roundrect: the flush caps would round the corners off (keks /CK_N
    grazed 0.046mm into R41.2's corner), so the capsule widens to short*sqrt(2)
    -- the cap radius reaches the square corners exactly.

Checks: extent along the long axis never exceeds the pad, rect corners stay
covered, layer expansion (SMD vs THT) unchanged.

Run:
    python3 tests/test_pad_obstacle_capsule.py
"""

import math
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

from kicad_parser import Pad
from diff_pair_routing import _pad_obstacle_segments

LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']


def _pad(shape, sx, sy, drill=0.0, layers=('F.Cu',), cx=10.0, cy=20.0):
    return Pad(component_ref='J4', pad_number='15', global_x=cx, global_y=cy,
               local_x=0.0, local_y=0.0, size_x=sx, size_y=sy, shape=shape,
               layers=list(layers), net_id=7, net_name='/PG26+', drill=drill)


def _capsule_extent_x(seg):
    """Max |x - cx| the capsule (segment + end caps) reaches."""
    return max(abs(seg.start_x - 10.0), abs(seg.end_x - 10.0)) + seg.width / 2.0


def _covers(seg, px, py):
    """Point inside the capsule (distance to spine <= width/2)?"""
    ax, ay, bx, by = seg.start_x, seg.start_y, seg.end_x, seg.end_y
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    qx, qy = ax + t * dx, ay + t * dy
    return math.hypot(px - qx, py - qy) <= seg.width / 2.0 + 1e-9


def main():
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}")
        if not cond:
            fails.append(name)

    # 1) square oval (the ecp5 1.7x1.7 header pin): exact extent, no doubling.
    segs = _pad_obstacle_segments(_pad('oval', 1.7, 1.7, drill=1.0), LAYERS)
    check("square oval: extent == pad half-size (was 2x, #289)",
          abs(_capsule_extent_x(segs[0]) - 0.85) < 1e-9)
    check("THT pad blocks every routing layer", len(segs) == len(LAYERS))

    # 2) elongated oval: stadium is exact -- extent == long/2, width == short.
    segs = _pad_obstacle_segments(_pad('oval', 1.0, 0.6), LAYERS)
    check("elongated oval: extent == long/2",
          abs(_capsule_extent_x(segs[0]) - 0.5) < 1e-9)
    check("elongated oval: width == short side", abs(segs[0].width - 0.6) < 1e-9)
    check("SMD pad blocks only its own layer", len(segs) == 1)

    # 3) rect: all four corners covered (the keks R41.2 corner graze), extent
    #    along the long axis still flush (no bulge past the pad ends).
    sx, sy = 0.6, 0.5
    segs = _pad_obstacle_segments(_pad('rect', sx, sy), LAYERS)
    seg = segs[0]
    corners = [(10.0 + ex * sx / 2, 20.0 + ey * sy / 2)
               for ex in (-1, 1) for ey in (-1, 1)]
    check("rect: all corners covered (keks R41.2)",
          all(_covers(seg, px, py) for px, py in corners))
    check("rect: no bulge past the pad ends",
          _capsule_extent_x(seg) <= sx / 2 + (math.sqrt(2) - 1) * sy / 2 + 1e-9)

    # 4) vertical orientation mirrors the horizontal one.
    segs = _pad_obstacle_segments(_pad('oval', 0.6, 1.0), LAYERS)
    ext_y = max(abs(segs[0].start_y - 20.0), abs(segs[0].end_y - 20.0)) + segs[0].width / 2
    check("vertical oval: extent == long/2", abs(ext_y - 0.5) < 1e-9)

    if fails:
        print(f"\nFAIL ({len(fails)})")
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())

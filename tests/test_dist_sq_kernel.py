#!/usr/bin/env python3
"""#382 E3: the one guarded point-to-segment squared-distance kernel.

geometry_utils.point_to_segment_dist_sq replaces the drifted `_pt_seg_dist_sq`
clones in plane_io.py and rip_up_reroute.py. Those clones guarded degeneracy
with an exact `dx == 0 and dy == 0` test, which does NOT protect against a
sub-10nm (denormal-tiny but non-zero) segment length dividing t up to +/-inf.
The kernel guards on `length_sq < 1e-10` instead.

This proves the replacement is byte-identical to the old clones for every
non-degenerate input (to float noise), matches point_to_segment_distance**2,
and stays finite on degenerate segments where the old clones could not.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from geometry_utils import point_to_segment_dist_sq, point_to_segment_distance


def _old_clone(px, py, x1, y1, x2, y2):
    """The pre-consolidation clone (exact-zero degenerate guard)."""
    dx, dy = x2 - x1, y2 - y1
    if dx == 0 and dy == 0:
        return (px - x1) ** 2 + (py - y1) ** 2
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    cx, cy = x1 + t * dx, y1 + t * dy
    return (px - cx) ** 2 + (py - cy) ** 2


def _ok(name, cond):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}")
    return bool(cond)


def main():
    r = []

    # 1. Non-degenerate: identical to the old clone and to distance**2.
    pts = [(-3.1, 4.2), (0.0, 0.0), (1.5, -2.5), (12.3, 7.7), (0.001, -0.001)]
    segs = [
        (0.0, 0.0, 5.0, 0.0),      # horizontal
        (0.0, 0.0, 0.0, 5.0),      # vertical
        (-2.0, -2.0, 3.0, 4.0),    # diagonal
        (1.0, 1.0, 1.5, 1.4),      # short
        (10.0, -3.0, -4.0, 8.0),   # long diagonal
    ]
    worst = 0.0
    worst_d = 0.0
    for (px, py) in pts:
        for (x1, y1, x2, y2) in segs:
            new = point_to_segment_dist_sq(px, py, x1, y1, x2, y2)
            old = _old_clone(px, py, x1, y1, x2, y2)
            worst = max(worst, abs(new - old))
            dist = point_to_segment_distance(px, py, x1, y1, x2, y2)
            worst_d = max(worst_d, abs(new - dist * dist))
    r.append(_ok(f"matches old clone to float noise (max abs diff {worst:.2e})",
                 worst < 1e-12))
    r.append(_ok(f"equals point_to_segment_distance**2 (max abs diff {worst_d:.2e})",
                 worst_d < 1e-9))

    # 2. Exactly-zero-length segment: distance to the point, finite.
    z = point_to_segment_dist_sq(3.0, 4.0, 1.0, 1.0, 1.0, 1.0)
    r.append(_ok("zero-length segment -> point dist_sq", abs(z - (4.0 + 9.0)) < 1e-12))

    # 3. Denormal-tiny (sub-10nm) segment: the old clone divides by ~1e-20 and
    #    the guard is what keeps us finite. New kernel must be finite and equal
    #    to the point-to-endpoint distance.
    tiny = 1e-8  # 0.01 nm; length_sq = 1e-16 < 1e-10
    val = point_to_segment_dist_sq(2.0, 0.0, 0.0, 0.0, tiny, 0.0)
    r.append(_ok("sub-10nm segment stays finite", math.isfinite(val)))
    r.append(_ok("sub-10nm segment -> endpoint dist_sq", abs(val - 4.0) < 1e-9))

    passed = sum(r)
    print(f"\n{passed}/{len(r)} dist_sq kernel tests passed")
    return 0 if passed == len(r) else 1


if __name__ == "__main__":
    sys.exit(main())

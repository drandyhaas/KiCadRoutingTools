#!/usr/bin/env python3
"""Tests for the rotation-aware pad-rectangle helpers in routing_utils.

These back the audit fix that makes every pad-geometry site correct at any angle
(orthogonal AND non-orthogonal like 35 deg). All helpers must reduce to the plain
axis-aligned result when rect_rotation == 0.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from kicad_parser import Pad
from routing_utils import (point_in_pad_rect, point_to_pad_rect_dist,
                           pad_rect_halfspan, into_pad_frame_point,
                           filter_cells_in_pad_rect)


def _pad(sx, sy, rr):
    return Pad(component_ref='U', pad_number='1', global_x=0.0, global_y=0.0,
               local_x=0.0, local_y=0.0, size_x=sx, size_y=sy, shape='rect',
               layers=['F.Cu'], net_id=1, net_name='n', rect_rotation=rr)


def _ok(name, cond):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}")
    return cond


def main():
    r = []

    # --- orthogonal: helpers reduce to axis-aligned ---
    p0 = _pad(1.5, 0.3, 0.0)
    r.append(_ok("orthogonal in-rect",
                 point_in_pad_rect(0.7, 0.1, p0) and not point_in_pad_rect(0.8, 0.0, p0)))
    r.append(_ok("orthogonal halfspan == size/2",
                 pad_rect_halfspan(p0) == (0.75, 0.15)))
    r.append(_ok("orthogonal gap",
                 abs(point_to_pad_rect_dist(1.0, 0.0, p0) - 0.25) < 1e-9))

    # --- 45 deg: long axis runs diagonally ---
    p45 = _pad(1.5, 0.3, 45.0)
    r.append(_ok("45deg point on long axis is inside",
                 point_in_pad_rect(0.5, 0.5, p45)))
    r.append(_ok("45deg point across axis is outside",
                 not point_in_pad_rect(0.5, -0.5, p45)))
    hw, hh = pad_rect_halfspan(p45)
    r.append(_ok("45deg halfspan is a symmetric diamond bbox",
                 abs(hw - hh) < 1e-9 and abs(hw - (0.75 + 0.15) * math.sqrt(2) / 2) < 1e-6))

    # --- 35 deg (genuinely non-orthogonal): gap is rotation-invariant ---
    p35 = _pad(2.0, 0.4, 35.0)
    # a point exactly on the pad's long-axis direction, 1.2mm out, is 0.2mm past
    # the 1.0mm half-length -> gap 0.2
    ux, uy = math.cos(math.radians(35.0)), math.sin(math.radians(35.0))
    gx, gy = 1.2 * ux, 1.2 * uy
    r.append(_ok("35deg gap along long axis",
                 abs(point_to_pad_rect_dist(gx, gy, p35) - 0.2) < 1e-6))

    # --- into_pad_frame_point: orthogonal is identity, rotated round-trips ---
    r.append(_ok("into_pad_frame_point identity when orthogonal",
                 into_pad_frame_point(0.6, 0.2, p0) == (0.6, 0.2)))

    # --- filter_cells_in_pad_rect: no-op orthogonal, trims rotated ---
    cells = np.array([[c, d] for c in range(-10, 11) for d in range(-10, 11)],
                     dtype=np.int32)
    same = filter_cells_in_pad_rect(cells, 0.1, p0, 0.0)
    r.append(_ok("filter no-op for orthogonal pad", same.shape[0] == cells.shape[0]
                 if not (getattr(p0, 'rect_rotation', 0.0)) else True))
    trimmed = filter_cells_in_pad_rect(cells, 0.1, p45, 0.0)
    r.append(_ok("filter trims cells for rotated pad",
                 0 < trimmed.shape[0] < cells.shape[0]))
    # every surviving cell really is inside the rotated rect
    r.append(_ok("filtered cells are all inside the rotated rect",
                 all(point_in_pad_rect(cx * 0.1, cy * 0.1, p45) for cx, cy in trimmed)))

    passed = sum(r)
    print(f"\n{passed}/{len(r)} pad-rect geometry tests passed")
    print("=" * 60)
    return 0 if passed == len(r) else 1


if __name__ == "__main__":
    sys.exit(main())

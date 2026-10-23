"""
Utility functions for component placement.
"""
from __future__ import annotations

import math
from typing import Tuple

from kicad_parser import Footprint, non_aperture_pads


def compute_footprint_bbox_local(footprint: Footprint) -> Tuple[float, float, float, float]:
    """
    Fallback: compute bounding box from pad LOCAL coordinates.
    Returns (min_x, min_y, max_x, max_y) in local coordinates.
    Used when no courtyard data is available.

    Aperture-only pads (paste/mask windows, `kicad_parser.pad_is_aperture_only`)
    are not part of the footprint's extent and are skipped (#1143); NPTH and
    drilled pads are kept. A footprint with no other pad gets the fallback box.
    """
    pads = non_aperture_pads(footprint)
    if not pads:
        return (-0.5, -0.5, 0.5, 0.5)

    min_x = float('inf')
    max_x = float('-inf')
    min_y = float('inf')
    max_y = float('-inf')

    # size_x/size_y are board-resolved; in the footprint's LOCAL frame the pad is
    # tilted by rect_rotation + the footprint rotation, so the local-axis bbox
    # half-extents follow from that combined angle (exact for any placement angle;
    # reduces to size/2 for an axis-aligned pad in an unrotated footprint).
    for pad in pads:
        cb = getattr(pad, 'copper_box', None)
        if cb is not None:
            # A custom pad's copper to one side of it (`Pad.copper_box`, board
            # frame about the pad): its corners into the footprint's frame.
            t = math.radians(footprint.rotation or 0.0)
            c, s = math.cos(t), math.sin(t)
            for u in (cb[0], cb[2]):
                for v in (cb[1], cb[3]):
                    lx = pad.local_x + u * c - v * s
                    ly = pad.local_y + u * s + v * c
                    min_x, max_x = min(min_x, lx), max(max_x, lx)
                    min_y, max_y = min(min_y, ly), max(max_y, ly)
            continue
        local_tilt = math.radians((getattr(pad, 'rect_rotation', 0.0) or 0.0)
                                  + (footprint.rotation or 0.0))
        c, s = abs(math.cos(local_tilt)), abs(math.sin(local_tilt))
        hx, hy = pad.size_x / 2, pad.size_y / 2
        half_sx = hx * c + hy * s
        half_sy = hx * s + hy * c
        min_x = min(min_x, pad.local_x - half_sx)
        max_x = max(max_x, pad.local_x + half_sx)
        min_y = min(min_y, pad.local_y - half_sy)
        max_y = max(max_y, pad.local_y + half_sy)

    # Ensure minimum size
    if max_x - min_x < 0.1:
        mid = (min_x + max_x) / 2
        min_x = mid - 0.05
        max_x = mid + 0.05
    if max_y - min_y < 0.1:
        mid = (min_y + max_y) / 2
        min_y = mid - 0.05
        max_y = mid + 0.05

    return (min_x, min_y, max_x, max_y)


def snap_to_grid(value: float, grid_step: float) -> float:
    """Snap a value to the nearest grid point."""
    return round(value / grid_step) * grid_step



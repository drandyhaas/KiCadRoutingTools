"""
Utility functions for component placement.
"""
from __future__ import annotations

import math
from typing import Tuple

from kicad_parser import Footprint


def compute_footprint_bbox_local(footprint: Footprint) -> Tuple[float, float, float, float]:
    """
    Fallback: compute bounding box from pad LOCAL coordinates.
    Returns (min_x, min_y, max_x, max_y) in local coordinates.
    Used when no courtyard data is available.
    """
    if not footprint.pads:
        return (-0.5, -0.5, 0.5, 0.5)

    min_x = float('inf')
    max_x = float('-inf')
    min_y = float('inf')
    max_y = float('-inf')

    # size_x/size_y are board-resolved; in the footprint's LOCAL frame the pad is
    # tilted by rect_rotation + the footprint rotation, so the local-axis bbox
    # half-extents follow from that combined angle (exact for any placement angle;
    # reduces to size/2 for an axis-aligned pad in an unrotated footprint).
    for pad in footprint.pads:
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


def refs_in_rect(pcb_data, rect, *, by='pad') -> list:
    """Every reference whose geometry lies in `rect`, HALF-OPEN on the far edge.

    THE one implementation, because two of them is a real defect rather than a
    tidiness point: `check_pockets` prints a rectangle and
    `place_seed --reseat-region` lifts the parts in it, and if the two resolve
    the same rectangle differently the census names parts the mover does not
    touch. Half-open (`x0 <= x < x1`) so a part on a shared boundary belongs to
    exactly one window of a tiling.

    by='pad'    -- any pad centre inside the rect. What the census has always
                   used, and the right question for "which parts cage this
                   window", since a large part reaches into a window its origin
                   is nowhere near.
    by='origin' -- the footprint origin only. This is `perturb._region_unit`'s
                   own membership rule, so it is what names the block that
                   `--block region:qN` would move.
    """
    x0, y0, x1, y1 = rect
    out = set()
    if by == 'origin':
        for ref, fp in pcb_data.footprints.items():
            if ref and x0 <= fp.x < x1 and y0 <= fp.y < y1:
                out.add(ref)
        return sorted(out)
    if by != 'pad':
        raise ValueError("refs_in_rect: by must be 'pad' or 'origin'")
    for fp in pcb_data.footprints.values():
        for pad in fp.pads:
            if (x0 <= pad.global_x < x1 and y0 <= pad.global_y < y1
                    and pad.component_ref):
                out.add(pad.component_ref)
                break
    return sorted(out)

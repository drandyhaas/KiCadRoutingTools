"""
Utility functions for component placement.
"""

from typing import Tuple

from kicad_parser import Footprint


def compute_footprint_bbox_local(footprint: Footprint) -> Tuple[float, float]:
    """
    Fallback: compute width/height from pad LOCAL coordinates.
    Used when no courtyard data is available.
    """
    if not footprint.pads:
        return (1.0, 1.0)

    min_x = float('inf')
    max_x = float('-inf')
    min_y = float('inf')
    max_y = float('-inf')

    for pad in footprint.pads:
        half_sx = pad.size_x / 2
        half_sy = pad.size_y / 2
        min_x = min(min_x, pad.local_x - half_sx)
        max_x = max(max_x, pad.local_x + half_sx)
        min_y = min(min_y, pad.local_y - half_sy)
        max_y = max(max_y, pad.local_y + half_sy)

    width = max_x - min_x
    height = max_y - min_y

    return (max(width, 0.1), max(height, 0.1))


def snap_to_grid(value: float, grid_step: float) -> float:
    """Snap a value to the nearest grid point."""
    return round(value / grid_step) * grid_step

"""
Geometry calculations for QFN/QFP fanout routing.

Functions for calculating fanout stub positions and angles.
"""

import math
from typing import Tuple

from qfn_fanout.types import QFNLayout, PadInfo


def calculate_fanout_stub(pad_info: PadInfo, layout: QFNLayout,
                          straight_length: float, max_diagonal_length: float,
                          grid_step: float = 0.0,
                          ) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """
    Calculate fanout stub with two segments: straight then 45 degrees.

    Returns (corner_pos, end_pos):
    - corner_pos: where the straight segment ends and the 45 deg fan begins
    - end_pos: final fanned-out endpoint

    Works entirely in GLOBAL board coordinates from the pad's escape vector
    (pad_info.escape_direction, the pad's outward long axis) and the along-edge
    tangent perpendicular to it, so it is correct for any board rotation or
    internal pad orientation. The straight segment escapes along the lead; the
    45 deg fan spreads tips toward the corners (length grows with distance from
    the edge midpoint).
    """
    pad = pad_info.pad
    pad_x, pad_y = pad.global_x, pad.global_y
    side = pad_info.side

    if side == 'center':
        return ((pad_x, pad_y), (pad_x, pad_y))

    # Escape vector (global, unit) and along-edge tangent perpendicular to it.
    esc_x, esc_y = pad_info.escape_direction
    tan_x, tan_y = -esc_y, esc_x

    # Corner point: end of straight segment (escape past the pad).
    corner_x = pad_x + esc_x * straight_length
    corner_y = pad_y + esc_y * straight_length

    # Signed offset along the edge from the package center (projection on tangent).
    off = (pad_x - layout.center_global_x) * tan_x + (pad_y - layout.center_global_y) * tan_y
    half_edge = (layout.width if side in ('top', 'bottom') else layout.height) / 2
    edge_position = min(abs(off) / half_edge, 1.0) if half_edge > 0 else 0.0

    # Diagonal length: 0 at center, max_diagonal_length at corners. A near-zero
    # fan collapses to the straight escape (end == corner).
    diagonal_length = edge_position * max_diagonal_length
    if diagonal_length < 0.01:
        end_x, end_y = corner_x, corner_y
    else:
        # True 45 deg: equal movement along escape and along the edge tangent.
        diag_component = diagonal_length / math.sqrt(2)
        fan_dir = 1.0 if off >= 0 else -1.0
        end_x = corner_x + esc_x * diag_component + fan_dir * tan_x * diag_component
        end_y = corner_y + esc_y * diag_component + fan_dir * tan_y * diag_component

    # Land the stub end on the routing grid (issue #149) so the router gets an
    # on-grid terminal and a foreign track on the nearest cell can't graze this
    # end by a sub-cell amount. Grid anchored at the origin. (Center pads return
    # early above with no stub, so there is nothing off-grid to snap there.)
    if grid_step > 0:
        end_x = round(end_x / grid_step) * grid_step
        end_y = round(end_y / grid_step) * grid_step

    return ((corner_x, corner_y), (end_x, end_y))

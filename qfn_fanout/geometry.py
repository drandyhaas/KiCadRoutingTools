"""
Geometry calculations for QFN/QFP fanout routing.

Functions for calculating fanout stub positions and angles.
"""

import math
from typing import Tuple

from qfn_fanout.types import QFNLayout, PadInfo


def calculate_fanout_stub(pad_info: PadInfo, layout: QFNLayout,
                          straight_length: float, max_diagonal_length: float,
                          fan_factor: float) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """
    Calculate fanout stub with two segments: straight then 45 degrees.

    Returns (corner_pos, end_pos):
    - corner_pos: where straight segment ends and 45 degrees begins
    - end_pos: final fanned-out endpoint

    The straight segment extends perpendicular to the chip edge (uniform for all pads).
    The 45 degree diagonal length varies by position:
    - Center pads: diagonal length = 0
    - Corner pads: diagonal length = max_diagonal_length
    - Linear interpolation in between

    Args:
        pad_info: Analyzed pad information
        layout: QFN layout parameters
        straight_length: Length of straight stub (pad_length / 2, uniform for all)
        max_diagonal_length: Max diagonal length for corner pads (chip_width / 3)
        fan_factor: Unused, kept for compatibility
    """
    pad = pad_info.pad
    pad_x, pad_y = pad.global_x, pad.global_y
    side = pad_info.side

    if side == 'center':
        return ((pad_x, pad_y), (pad_x, pad_y))

    # Escape direction perpendicular to edge
    esc_x, esc_y = pad_info.escape_direction

    # Corner point: end of straight segment
    corner_x = pad_x + esc_x * straight_length
    corner_y = pad_y + esc_y * straight_length

    # Calculate position along edge (0 = center, 1 = corner)
    if side in ('top', 'bottom'):
        half_width = layout.width / 2
        offset_from_center = abs(pad_x - layout.center_x)
        edge_position = offset_from_center / half_width if half_width > 0 else 0
    else:
        half_height = layout.height / 2
        offset_from_center = abs(pad_y - layout.center_y)
        edge_position = offset_from_center / half_height if half_height > 0 else 0

    # Diagonal length: 0 at center, max_diagonal_length at corners
    diagonal_length = edge_position * max_diagonal_length

    if diagonal_length < 0.01:
        # No 45 degree segment needed (center pads)
        return ((corner_x, corner_y), (corner_x, corner_y))

    # True 45 degrees means equal movement in escape direction and fan direction
    # Distance along 45 degree line = diagonal_length, so each component = diagonal_length / sqrt(2)
    diag_component = diagonal_length / math.sqrt(2)

    if side in ('top', 'bottom'):
        # Horizontal edge - fan left/right based on position
        offset_from_center = pad_x - layout.center_x
        fan_dir = 1 if offset_from_center >= 0 else -1

        # True 45 degrees: equal dx and dy
        end_x = corner_x + fan_dir * diag_component
        end_y = corner_y + esc_y * diag_component
    else:
        # Vertical edge - fan up/down based on position
        offset_from_center = pad_y - layout.center_y
        fan_dir = 1 if offset_from_center >= 0 else -1

        # True 45 degrees: equal dx and dy
        end_x = corner_x + esc_x * diag_component
        end_y = corner_y + fan_dir * diag_component

    return ((corner_x, corner_y), (end_x, end_y))

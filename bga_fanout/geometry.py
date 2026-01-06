"""
Geometry calculations for BGA fanout routing.

Functions for calculating 45-degree stubs, exit points, and jog endpoints.
"""

import math
from typing import List, Tuple, Optional

from bga_fanout.types import BGAGrid, Channel


def create_45_stub(pad_x: float, pad_y: float,
                   channel: Channel,
                   escape_dir: str,
                   channel_offset: float = 0.0) -> Tuple[float, float]:
    """
    Create 45-degree stub from pad to channel.

    For differential pairs, the offset is applied AFTER calculating the 45° stub
    based on the channel center. This ensures both P and N travel the same
    horizontal/vertical distance before applying the parallel offset.

    Args:
        pad_x, pad_y: Pad position
        channel: Target channel
        escape_dir: Direction of escape
        channel_offset: Offset from channel center (for diff pairs)

    Returns:
        End position of the 45° stub
    """
    if channel.orientation == 'horizontal':
        # Target Y includes the offset
        target_y = channel.position + channel_offset
        # For true 45°, dx = |dy|
        dy_to_target = target_y - pad_y
        if escape_dir == 'right':
            dx = abs(dy_to_target)
        else:
            dx = -abs(dy_to_target)

        return (pad_x + dx, target_y)
    else:
        # Target X includes the offset
        target_x = channel.position + channel_offset
        # For true 45°, dy = |dx|
        dx_to_target = target_x - pad_x
        if escape_dir == 'down':
            dy = abs(dx_to_target)
        else:
            dy = -abs(dx_to_target)

        return (target_x, pad_y + dy)


def calculate_exit_point(stub_end: Tuple[float, float],
                         channel: Channel,
                         escape_dir: str,
                         grid: BGAGrid,
                         margin: float = 0.5,
                         channel_offset: float = 0.0) -> Tuple[float, float]:
    """Calculate where the route exits the BGA boundary."""
    if channel.orientation == 'horizontal':
        exit_y = channel.position + channel_offset
        if escape_dir == 'right':
            return (grid.max_x + margin, exit_y)
        else:
            return (grid.min_x - margin, exit_y)
    else:
        exit_x = channel.position + channel_offset
        if escape_dir == 'down':
            return (exit_x, grid.max_y + margin)
        else:
            return (exit_x, grid.min_y - margin)


def calculate_jog_end(exit_pos: Tuple[float, float],
                      escape_dir: str,
                      layer: str,
                      layers: List[str],
                      jog_length: float,
                      is_diff_pair: bool = False,
                      is_outside_track: bool = False,
                      pair_spacing: float = 0.0) -> Tuple[Tuple[float, float], Optional[Tuple[float, float]]]:
    """
    Calculate the end position of the 45° jog at the exit.

    For differential pairs, the outside track needs to extend further before
    bending to maintain constant spacing through the 45° turn.

    Jog direction depends on layer:
    - Top layer (F.Cu): 45° to the left (from perspective walking towards BGA edge)
    - Bottom layer (B.Cu): 45° to the right
    - Middle layers: linear interpolation

    Args:
        exit_pos: Starting point of jog
        escape_dir: Direction of escape ('left', 'right', 'up', 'down')
        layer: Current layer
        layers: List of all available layers
        jog_length: Length of the jog (distance from BGA edge to first pad row/col)
        is_diff_pair: Whether this is part of a differential pair
        is_outside_track: Whether this is the outside track of the pair (needs extension)
        pair_spacing: Spacing between P and N tracks

    Returns:
        (jog_end, extension_point) - extension_point is the intermediate point for outside tracks
    """
    # Calculate layer position: 0 = top (left jog), 1 = bottom (right jog)
    try:
        layer_idx = layers.index(layer)
    except ValueError:
        layer_idx = 0

    num_layers = len(layers)
    if num_layers <= 1:
        layer_factor = 0.0  # Default to left jog
    else:
        layer_factor = layer_idx / (num_layers - 1)  # 0 to 1

    # Jog angle: -1 = left, +1 = right (from perspective of walking towards edge)
    # layer_factor 0 (top) -> -1 (left)
    # layer_factor 1 (bottom) -> +1 (right)
    jog_direction = 2 * layer_factor - 1  # Maps 0->-1, 1->+1

    # Calculate jog components based on escape direction
    # At 45°, both components equal jog_length / sqrt(2)
    # Reduce by factor of 4 for a shorter angled segment at the end of each stub
    diag = jog_length / math.sqrt(2) / 4

    ex, ey = exit_pos
    extension_point = None

    # For differential pairs, outside track extends further before bending
    # To maintain constant perpendicular spacing through a 45° turn:
    # extension = pair_spacing * (sqrt(2) - 1) ≈ 0.414 * pair_spacing
    if is_diff_pair and is_outside_track:
        extension = pair_spacing * (math.sqrt(2) - 1)
        if escape_dir == 'right':
            extension_point = (ex + extension, ey)
            ex = ex + extension
        elif escape_dir == 'left':
            extension_point = (ex - extension, ey)
            ex = ex - extension
        elif escape_dir == 'down':
            extension_point = (ex, ey + extension)
            ey = ey + extension
        else:  # up
            extension_point = (ex, ey - extension)
            ey = ey - extension

    if escape_dir == 'right':
        # Walking right, left is up (-Y), right is down (+Y)
        jog_end = (ex + diag, ey + jog_direction * diag)
    elif escape_dir == 'left':
        # Walking left, left is down (+Y), right is up (-Y)
        jog_end = (ex - diag, ey - jog_direction * diag)
    elif escape_dir == 'down':
        # Walking down, left is right (+X), right is left (-X)
        jog_end = (ex - jog_direction * diag, ey + diag)
    else:  # up
        # Walking up, left is left (-X), right is right (+X)
        jog_end = (ex + jog_direction * diag, ey - diag)

    return jog_end, extension_point

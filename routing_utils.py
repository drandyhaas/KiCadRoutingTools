"""
Shared base utilities for PCB routing.

This module contains the core position and geometry utilities used by all routing modules.
The bulk of routing functionality has been split into:
- connectivity.py: Endpoint finding, stub analysis, MST algorithms
- net_queries.py: Pad/net queries, MPS ordering, route length calculations
- pcb_modification.py: Add/remove routes, segment cleanup
"""

import math
from typing import Dict, List, Tuple

from kicad_parser import Segment, POSITION_DECIMALS


def build_layer_map(layers: List[str]) -> Dict[str, int]:
    """
    Build a mapping from layer names to indices.

    Args:
        layers: List of layer names (e.g., ['F.Cu', 'In1.Cu', 'B.Cu'])

    Returns:
        Dict mapping layer name to its index in the list
    """
    return {name: idx for idx, name in enumerate(layers)}


def pos_key(x: float, y: float) -> Tuple[float, float]:
    """
    Normalize coordinates for position-based lookups.

    Use this consistently when building position sets or checking position membership
    to avoid floating-point comparison issues.
    """
    return (round(x, POSITION_DECIMALS), round(y, POSITION_DECIMALS))


def segment_length(seg: Segment) -> float:
    """Calculate the length of a single segment."""
    return math.sqrt((seg.end_x - seg.start_x)**2 + (seg.end_y - seg.start_y)**2)


def dist_sq_to_rounded_rect(
    point_x: float, point_y: float,
    half_width: float, half_height: float,
    corner_radius: float = 0.0
) -> float:
    """
    Calculate squared distance from a point to a rounded rectangle centered at origin.

    The rectangle extends from (-half_width, -half_height) to (half_width, half_height).
    Corner radius creates rounded corners (0 for sharp corners).

    Args:
        point_x, point_y: Point coordinates (relative to rectangle center)
        half_width, half_height: Rectangle half-dimensions
        corner_radius: Radius of rounded corners (0 for sharp corners)

    Returns:
        Squared distance from point to rectangle edge (0 if inside)
    """
    abs_x, abs_y = abs(point_x), abs(point_y)

    if corner_radius > 0:
        # Inner rectangle bounds (where corners start)
        inner_half_w = half_width - corner_radius
        inner_half_h = half_height - corner_radius

        # Check if point is in a corner region
        if abs_x > inner_half_w and abs_y > inner_half_h:
            # Distance to corner arc center
            dx = abs_x - inner_half_w
            dy = abs_y - inner_half_h
            dist_to_corner_center = math.sqrt(dx * dx + dy * dy)
            # Distance to arc edge
            dist = dist_to_corner_center - corner_radius
            return dist * dist if dist > 0 else 0.0

    # Point is along a flat edge - rectangular distance
    closest_x = max(-half_width, min(point_x, half_width))
    closest_y = max(-half_height, min(point_y, half_height))
    dx = point_x - closest_x
    dy = point_y - closest_y
    return dx * dx + dy * dy


def iter_pad_blocked_cells(
    pad_gx: int, pad_gy: int,
    half_width: float, half_height: float,
    margin: float,
    grid_step: float,
    corner_radius: float = 0.0
):
    """
    Generate grid cells that should be blocked for a pad.

    Yields (gx, gy) tuples for all cells within margin distance of the pad edge.
    Uses rectangular-with-rounded-corners shape matching the actual pad geometry.

    Args:
        pad_gx, pad_gy: Pad center in grid coordinates
        half_width, half_height: Pad half-dimensions in mm
        margin: Blocking margin in mm (e.g., track_width/2 + clearance)
        grid_step: Grid step size in mm
        corner_radius: Corner radius for roundrect pads (0 for rectangular)

    Yields:
        (gx, gy) tuples for each blocked cell
    """
    # Add half grid step buffer to account for grid discretization
    # (a track through a cell could be grid_step/2 closer to the pad than the cell center)
    # This applies to ALL pad shapes since diagonal approaches can occur with rectangular pads too
    corner_buffer = grid_step / 2
    expand_x = int(math.ceil((half_width + margin + corner_buffer) / grid_step))
    expand_y = int(math.ceil((half_height + margin + corner_buffer) / grid_step))
    margin_sq = margin * margin
    buffered_margin_sq = (margin + corner_buffer) * (margin + corner_buffer)

    # Inner rectangle bounds (where corners start)
    inner_half_w = half_width - corner_radius if corner_radius > 0 else half_width
    inner_half_h = half_height - corner_radius if corner_radius > 0 else half_height

    for ex in range(-expand_x, expand_x + 1):
        for ey in range(-expand_y, expand_y + 1):
            cell_x = ex * grid_step
            cell_y = ey * grid_step
            abs_x, abs_y = abs(cell_x), abs(cell_y)

            # Use buffered margin in corner/diagonal regions where grid discretization
            # can cause tracks to be closer than expected (applies to all pad shapes)
            in_corner = abs_x > inner_half_w and abs_y > inner_half_h
            effective_margin_sq = buffered_margin_sq if in_corner else margin_sq

            dist_sq = dist_sq_to_rounded_rect(cell_x, cell_y, half_width, half_height, corner_radius)
            if dist_sq < effective_margin_sq:
                yield (pad_gx + ex, pad_gy + ey)

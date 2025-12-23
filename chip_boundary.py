"""Chip boundary ordering utilities for crossing detection.

This module provides functions to compute boundary positions along chip perimeters,
which enables more accurate crossing detection that respects the physical constraint
that routes cannot go through BGA chips.

The key concept is "unrolling" each chip's rectangular boundary into a linear ordering
starting from the "far side" (the side facing away from the other chip). Two nets cross
if their source ordering is inverted relative to their target ordering.
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

from kicad_parser import PCBData, Footprint


@dataclass
class ChipBoundary:
    """Information about a chip for boundary ordering."""
    reference: str  # Component reference (e.g., "U1")
    center: Tuple[float, float]  # Chip center
    bounds: Tuple[float, float, float, float]  # (min_x, min_y, max_x, max_y)


def build_chip_list(pcb_data: PCBData) -> List[ChipBoundary]:
    """
    Build list of chips with their boundaries from PCB data.

    Currently extracts all footprints with computed bounding boxes from their pads.
    """
    chips = []
    for ref, footprint in pcb_data.footprints.items():
        if not footprint.pads:
            continue

        # Compute bounding box from pad positions
        pad_xs = [p.global_x for p in footprint.pads]
        pad_ys = [p.global_y for p in footprint.pads]

        if not pad_xs or not pad_ys:
            continue

        min_x = min(pad_xs)
        max_x = max(pad_xs)
        min_y = min(pad_ys)
        max_y = max(pad_ys)

        # Add small margin around pads
        margin = 0.5  # mm
        min_x -= margin
        max_x += margin
        min_y -= margin
        max_y += margin

        center = ((min_x + max_x) / 2, (min_y + max_y) / 2)

        chips.append(ChipBoundary(
            reference=ref,
            center=center,
            bounds=(min_x, min_y, max_x, max_y)
        ))

    return chips


def identify_chip_for_point(
    point: Tuple[float, float],
    chips: List[ChipBoundary],
    tolerance: float = 2.0
) -> Optional[ChipBoundary]:
    """
    Find which chip a point belongs to.

    Strategy:
    1. Check if point is inside any chip's bounding box
    2. If not inside, find the nearest chip within tolerance

    Args:
        point: (x, y) position to identify
        chips: List of chip boundaries to search
        tolerance: Maximum distance to consider a point belonging to a chip

    Returns:
        The ChipBoundary the point belongs to, or None if not found
    """
    x, y = point

    # First check if inside any chip bounds
    for chip in chips:
        min_x, min_y, max_x, max_y = chip.bounds
        if min_x <= x <= max_x and min_y <= y <= max_y:
            return chip

    # Not inside any chip, find nearest within tolerance
    best_chip = None
    best_dist = float('inf')

    for chip in chips:
        min_x, min_y, max_x, max_y = chip.bounds

        # Distance to bounding box
        dx = max(min_x - x, 0, x - max_x)
        dy = max(min_y - y, 0, y - max_y)
        dist = (dx * dx + dy * dy) ** 0.5

        if dist < best_dist and dist <= tolerance:
            best_dist = dist
            best_chip = chip

    return best_chip


def compute_far_side(
    source_chip: ChipBoundary,
    target_chip: ChipBoundary
) -> Tuple[str, str]:
    """
    Determine the "far side" for each chip based on chip-to-chip direction.

    The far side is the side of the chip facing away from the other chip.
    This determines where we start counting positions along the boundary.

    Args:
        source_chip: The source chip boundary
        target_chip: The target chip boundary

    Returns:
        (source_far_side, target_far_side) where each is one of:
        'left', 'right', 'top', 'bottom'
    """
    dx = target_chip.center[0] - source_chip.center[0]
    dy = target_chip.center[1] - source_chip.center[1]

    # Determine primary direction
    if abs(dx) > abs(dy):
        # Horizontal arrangement
        if dx > 0:
            # Target is to the right of source
            source_far = 'left'   # Source's far side is left
            target_far = 'right'  # Target's far side is right
        else:
            # Target is to the left of source
            source_far = 'right'
            target_far = 'left'
    else:
        # Vertical arrangement
        if dy > 0:
            # Target is below source (Y increases downward in KiCad)
            source_far = 'top'
            target_far = 'bottom'
        else:
            # Target is above source
            source_far = 'bottom'
            target_far = 'top'

    return source_far, target_far


def _project_to_boundary(
    point: Tuple[float, float],
    bounds: Tuple[float, float, float, float]
) -> Tuple[Tuple[float, float], str]:
    """
    Project a point onto the nearest edge of a rectangular boundary.

    Args:
        point: (x, y) position to project
        bounds: (min_x, min_y, max_x, max_y) rectangle bounds

    Returns:
        (projected_point, edge) where edge is 'left', 'right', 'top', or 'bottom'
    """
    x, y = point
    min_x, min_y, max_x, max_y = bounds

    # Clamp point to be on or inside bounds first
    cx = max(min_x, min(max_x, x))
    cy = max(min_y, min(max_y, y))

    # Find distances to each edge
    dist_left = abs(cx - min_x)
    dist_right = abs(cx - max_x)
    dist_top = abs(cy - min_y)
    dist_bottom = abs(cy - max_y)

    min_dist = min(dist_left, dist_right, dist_top, dist_bottom)

    if min_dist == dist_left:
        return (min_x, cy), 'left'
    elif min_dist == dist_right:
        return (max_x, cy), 'right'
    elif min_dist == dist_top:
        return (cx, min_y), 'top'
    else:
        return (cx, max_y), 'bottom'


def compute_boundary_position(
    chip: ChipBoundary,
    point: Tuple[float, float],
    far_side: str,
    clockwise: bool = True
) -> float:
    """
    Compute normalized position [0, 1] along chip boundary.

    The boundary is "unrolled" starting from the far side corner,
    proceeding clockwise or counter-clockwise. Each point gets a position
    based on its distance along this unrolled perimeter.

    IMPORTANT: For crossing detection to work correctly, source and target
    chips must use opposite directions (one clockwise, one counter-clockwise)
    so that positions on facing edges increase in the same direction.

    Args:
        chip: The chip boundary information
        point: (x, y) position to compute boundary position for
        far_side: Which side to start from ('left', 'right', 'top', 'bottom')
        clockwise: Direction to traverse boundary. Use True for source chips,
                   False for target chips.

    Returns:
        Normalized position in [0, 1] along the boundary
    """
    min_x, min_y, max_x, max_y = chip.bounds
    width = max_x - min_x
    height = max_y - min_y
    perimeter = 2 * (width + height)

    if perimeter <= 0:
        return 0.0

    # Project point onto boundary
    projected, edge = _project_to_boundary(point, chip.bounds)
    px, py = projected

    # Define edge order and start corner based on far_side and direction
    # Clockwise: left->bottom->right->top (starting from appropriate corner)
    # Counter-clockwise: left->top->right->bottom (starting from appropriate corner)
    if far_side == 'left':
        if clockwise:
            # Start at top-left corner, go clockwise: left -> bottom -> right -> top
            edge_order = ['left', 'bottom', 'right', 'top']
        else:
            # Start at bottom-left corner, go counter-clockwise: left -> top -> right -> bottom
            edge_order = ['left', 'top', 'right', 'bottom']
    elif far_side == 'right':
        if clockwise:
            # Start at bottom-right corner, go clockwise: right -> top -> left -> bottom
            edge_order = ['right', 'top', 'left', 'bottom']
        else:
            # Start at top-right corner, go counter-clockwise: right -> bottom -> left -> top
            edge_order = ['right', 'bottom', 'left', 'top']
    elif far_side == 'top':
        if clockwise:
            # Start at top-right corner, go clockwise: top -> left -> bottom -> right
            edge_order = ['top', 'left', 'bottom', 'right']
        else:
            # Start at top-left corner, go counter-clockwise: top -> right -> bottom -> left
            edge_order = ['top', 'right', 'bottom', 'left']
    elif far_side == 'bottom':
        if clockwise:
            # Start at bottom-left corner, go clockwise: bottom -> right -> top -> left
            edge_order = ['bottom', 'right', 'top', 'left']
        else:
            # Start at bottom-right corner, go counter-clockwise: bottom -> left -> top -> right
            edge_order = ['bottom', 'left', 'top', 'right']
    else:
        # Default to left, clockwise
        edge_order = ['left', 'bottom', 'right', 'top']

    # Compute distance along perimeter from start corner to projected point
    # The starting point on each edge depends on the direction (clockwise vs counter-clockwise)
    distance = 0.0

    for current_edge in edge_order:
        if current_edge == 'left':
            edge_length = height
            if edge == 'left':
                if clockwise:
                    # Clockwise: left edge traversed top-to-bottom
                    distance += abs(py - min_y)
                else:
                    # Counter-clockwise: left edge traversed bottom-to-top
                    distance += abs(py - max_y)
                break
            else:
                distance += edge_length
        elif current_edge == 'bottom':
            edge_length = width
            if edge == 'bottom':
                if clockwise:
                    # Clockwise: bottom edge traversed left-to-right
                    distance += abs(px - min_x)
                else:
                    # Counter-clockwise: bottom edge traversed right-to-left
                    distance += abs(px - max_x)
                break
            else:
                distance += edge_length
        elif current_edge == 'right':
            edge_length = height
            if edge == 'right':
                if clockwise:
                    # Clockwise: right edge traversed bottom-to-top
                    distance += abs(py - max_y)
                else:
                    # Counter-clockwise: right edge traversed top-to-bottom
                    distance += abs(py - min_y)
                break
            else:
                distance += edge_length
        elif current_edge == 'top':
            edge_length = width
            if edge == 'top':
                if clockwise:
                    # Clockwise: top edge traversed right-to-left
                    distance += abs(px - max_x)
                else:
                    # Counter-clockwise: top edge traversed left-to-right
                    distance += abs(px - min_x)
                break
            else:
                distance += edge_length

    return distance / perimeter


def crossings_from_boundary_order(
    src_pos_a: float,
    tgt_pos_a: float,
    src_pos_b: float,
    tgt_pos_b: float
) -> bool:
    """
    Check if two nets cross based on their boundary positions.

    Two nets cross if their relative order is inverted between source and target:
    - If net A < net B on source side but net A > net B on target side -> crossing
    - If net A > net B on source side but net A < net B on target side -> crossing

    This is the classic bipartite crossing detection.

    Args:
        src_pos_a: Net A's position on source chip boundary [0, 1]
        tgt_pos_a: Net A's position on target chip boundary [0, 1]
        src_pos_b: Net B's position on source chip boundary [0, 1]
        tgt_pos_b: Net B's position on target chip boundary [0, 1]

    Returns:
        True if the nets cross, False otherwise
    """
    source_order = (src_pos_a < src_pos_b)
    target_order = (tgt_pos_a < tgt_pos_b)
    return source_order != target_order

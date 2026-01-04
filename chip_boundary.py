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


def build_chip_list(pcb_data: PCBData, min_pads: int = 4) -> List[ChipBoundary]:
    """
    Build list of chips with their boundaries from PCB data.

    Extracts footprints with computed bounding boxes from their pads.
    Only includes footprints with at least min_pads pads (filters out passives).

    Args:
        pcb_data: PCB data
        min_pads: Minimum number of pads to be considered a chip (default: 4)
    """
    chips = []
    for ref, footprint in pcb_data.footprints.items():
        if not footprint.pads or len(footprint.pads) < min_pads:
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
    clockwise: bool = True,
    exit_edge: str = None
) -> float:
    """
    Compute normalized position [0, 1] along chip boundary.

    The boundary is "unrolled" starting from the far side, with edge traversal
    directions following the clockwise/counter-clockwise direction. This ensures
    that routes connecting corresponding positions (e.g., bottom-left to bottom-left)
    are correctly detected as non-crossing.

    For horizontal chip arrangement (left chip → right chip):
    - Source chip uses counter-clockwise (left → top → right → bottom)
    - Target chip uses clockwise (right → top → left → bottom)
    - This makes bottom edges both increase going in opposite spatial directions,
      which correctly preserves the relative ordering for parallel routes.

    Args:
        chip: The chip boundary information
        point: (x, y) position to compute boundary position for
        far_side: Which side to start from ('left', 'right', 'top', 'bottom')
        clockwise: Direction to traverse boundary (affects within-edge direction)
        exit_edge: Optional edge override (e.g., from stub direction tracing)

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
    if exit_edge:
        # Use the specified edge and project point onto it
        edge = exit_edge
        x, y = point
        cx = max(min_x, min(max_x, x))
        cy = max(min_y, min(max_y, y))
        if edge == 'left':
            px, py = min_x, cy
        elif edge == 'right':
            px, py = max_x, cy
        elif edge == 'top':
            px, py = cx, min_y
        else:  # bottom
            px, py = cx, max_y
    else:
        projected, edge = _project_to_boundary(point, chip.bounds)
        px, py = projected

    # Define edge order and traversal directions based on far_side and clockwise
    # Each entry is (edge_name, forward_direction) where forward_direction indicates
    # whether we traverse in the positive spatial direction (True) or negative (False)
    #
    # Clockwise traversal (starting from far side):
    #   left:   left(↓) → bottom(→) → right(↑) → top(←)
    #   right:  right(↑) → top(←) → left(↓) → bottom(→)
    #   top:    top(←) → left(↓) → bottom(→) → right(↑)
    #   bottom: bottom(→) → right(↑) → top(←) → left(↓)
    #
    # Counter-clockwise is the reverse within each edge

    if far_side == 'left':
        if clockwise:
            # left(down/+Y) → bottom(right/+X) → right(up/-Y) → top(left/-X)
            edge_info = [('left', True), ('bottom', True), ('right', False), ('top', False)]
        else:
            # left(up/-Y) → top(right/+X) → right(down/+Y) → bottom(left/-X)
            edge_info = [('left', False), ('top', True), ('right', True), ('bottom', False)]
    elif far_side == 'right':
        if clockwise:
            # right(up/-Y) → top(left/-X) → left(down/+Y) → bottom(right/+X)
            edge_info = [('right', False), ('top', False), ('left', True), ('bottom', True)]
        else:
            # right(down/+Y) → bottom(left/-X) → left(up/-Y) → top(right/+X)
            edge_info = [('right', True), ('bottom', False), ('left', False), ('top', True)]
    elif far_side == 'top':
        if clockwise:
            # top(left/-X) → left(down/+Y) → bottom(right/+X) → right(up/-Y)
            edge_info = [('top', False), ('left', True), ('bottom', True), ('right', False)]
        else:
            # top(right/+X) → right(down/+Y) → bottom(left/-X) → left(up/-Y)
            edge_info = [('top', True), ('right', True), ('bottom', False), ('left', False)]
    elif far_side == 'bottom':
        if clockwise:
            # bottom(right/+X) → right(up/-Y) → top(left/-X) → left(down/+Y)
            edge_info = [('bottom', True), ('right', False), ('top', False), ('left', True)]
        else:
            # bottom(left/-X) → left(up/-Y) → top(right/+X) → right(down/+Y)
            edge_info = [('bottom', False), ('left', False), ('top', True), ('right', True)]
    else:
        edge_info = [('left', True), ('bottom', True), ('right', False), ('top', False)]

    # Compute distance along perimeter from start to projected point
    distance = 0.0

    for current_edge, forward in edge_info:
        if current_edge == 'left':
            edge_length = height
            if edge == 'left':
                if forward:
                    # Going down (increasing Y): distance from top
                    distance += (py - min_y)
                else:
                    # Going up (decreasing Y): distance from bottom
                    distance += (max_y - py)
                break
            else:
                distance += edge_length
        elif current_edge == 'bottom':
            edge_length = width
            if edge == 'bottom':
                if forward:
                    # Going right (increasing X): distance from left
                    distance += (px - min_x)
                else:
                    # Going left (decreasing X): distance from right
                    distance += (max_x - px)
                break
            else:
                distance += edge_length
        elif current_edge == 'right':
            edge_length = height
            if edge == 'right':
                if forward:
                    # Going down (increasing Y): distance from top
                    distance += (py - min_y)
                else:
                    # Going up (decreasing Y): distance from bottom
                    distance += (max_y - py)
                break
            else:
                distance += edge_length
        elif current_edge == 'top':
            edge_length = width
            if edge == 'top':
                if forward:
                    # Going right (increasing X): distance from left
                    distance += (px - min_x)
                else:
                    # Going left (decreasing X): distance from right
                    distance += (max_x - px)
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


def generate_boundary_debug_labels(
    centroids: List[Tuple[float, float]],
    chips: List[ChipBoundary],
    far_side: str,
    clockwise: bool,
    layer: str = "User.6"
) -> List[dict]:
    """
    Generate debug labels showing boundary position ordering.

    Args:
        centroids: List of (x, y) positions for each net
        chips: List of chip boundaries
        far_side: Far side for position computation
        clockwise: Direction for boundary traversal
        layer: Layer to place labels on

    Returns:
        List of dicts with 'text', 'x', 'y', 'layer' for each label
    """
    if not centroids:
        return []

    # Compute boundary positions for each centroid
    positions = []
    for centroid in centroids:
        chip = identify_chip_for_point(centroid, chips)
        if chip:
            pos = compute_boundary_position(chip, centroid, far_side, clockwise)
            # Project point onto chip boundary to get label position
            projected, edge = _project_to_boundary(centroid, chip.bounds)
            positions.append((pos, projected, centroid))
        else:
            positions.append((0.0, centroid, centroid))

    # Sort by position and assign ordering numbers
    sorted_indices = sorted(range(len(positions)), key=lambda i: positions[i][0])

    labels = []
    for order_num, idx in enumerate(sorted_indices, start=1):
        pos, projected, original = positions[idx]
        labels.append({
            'text': str(order_num),
            'x': projected[0],
            'y': projected[1],
            'layer': layer,
            'pos_value': pos  # Include for debugging
        })

    return labels

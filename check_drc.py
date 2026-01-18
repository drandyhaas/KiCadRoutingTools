"""
DRC Checker - Find overlapping tracks and vias between different nets.
"""

import sys
import argparse
import math
import fnmatch
from collections import defaultdict
from typing import List, Tuple, Set, Optional, Dict, Any
from kicad_parser import parse_kicad_pcb, Segment, Via, Pad
from geometry_utils import (
    point_to_segment_distance,
    closest_point_on_segment,
    segment_to_segment_closest_points,
)
from net_queries import expand_pad_layers


class SpatialIndex:
    """Grid-based spatial index for fast proximity queries."""

    def __init__(self, cell_size: float = 2.0):
        """Initialize with given cell size in mm."""
        self.cell_size = cell_size
        self.inv_cell_size = 1.0 / cell_size
        # Dict[layer][cell_key] -> list of (object, net_id)
        self.cells_by_layer: Dict[str, Dict[Tuple[int, int], List[Tuple[Any, int]]]] = defaultdict(lambda: defaultdict(list))
        # For objects that span all layers (vias)
        self.all_layer_cells: Dict[Tuple[int, int], List[Tuple[Any, int]]] = defaultdict(list)

    def _get_cell(self, x: float, y: float) -> Tuple[int, int]:
        """Get cell coordinates for a point."""
        return (int(x * self.inv_cell_size), int(y * self.inv_cell_size))

    def _get_segment_cells(self, seg: Segment) -> Set[Tuple[int, int]]:
        """Get all cells a segment passes through."""
        cells = set()
        x1, y1 = seg.start_x, seg.start_y
        x2, y2 = seg.end_x, seg.end_y

        # Add endpoint cells
        cells.add(self._get_cell(x1, y1))
        cells.add(self._get_cell(x2, y2))

        # Walk along segment and add intermediate cells
        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx*dx + dy*dy)
        if length > 0:
            # Sample every half cell size
            steps = max(1, int(length * self.inv_cell_size * 2))
            for i in range(1, steps):
                t = i / steps
                x = x1 + t * dx
                y = y1 + t * dy
                cells.add(self._get_cell(x, y))

        return cells

    def add_segment(self, seg: Segment, net_id: int):
        """Add a segment to the index."""
        cells = self._get_segment_cells(seg)
        layer_cells = self.cells_by_layer[seg.layer]
        for cell in cells:
            layer_cells[cell].append((seg, net_id))

    def add_via(self, via: Via, net_id: int):
        """Add a via to the index (spans all layers)."""
        cell = self._get_cell(via.x, via.y)
        self.all_layer_cells[cell].append((via, net_id))

    def add_pad(self, pad: Pad, net_id: int, expanded_layers: List[str]):
        """Add a pad to the index."""
        # Pad covers a rectangular area
        half_x = pad.size_x / 2
        half_y = pad.size_y / 2
        min_cell = self._get_cell(pad.global_x - half_x, pad.global_y - half_y)
        max_cell = self._get_cell(pad.global_x + half_x, pad.global_y + half_y)

        for layer in expanded_layers:
            if not layer.endswith('.Cu'):
                continue
            layer_cells = self.cells_by_layer[layer]
            for cx in range(min_cell[0], max_cell[0] + 1):
                for cy in range(min_cell[1], max_cell[1] + 1):
                    layer_cells[(cx, cy)].append((pad, net_id))

    def get_nearby_segments(self, seg: Segment) -> List[Tuple[Segment, int]]:
        """Get segments that might be near the given segment (same layer, nearby cells)."""
        cells = self._get_segment_cells(seg)
        layer_cells = self.cells_by_layer[seg.layer]

        seen = set()
        result = []
        for cell in cells:
            for obj, net_id in layer_cells.get(cell, []):
                if isinstance(obj, Segment) and id(obj) not in seen:
                    seen.add(id(obj))
                    result.append((obj, net_id))
        return result

    def get_nearby_for_via(self, via: Via, layer: str) -> List[Tuple[Any, int]]:
        """Get objects near a via on a specific layer."""
        cell = self._get_cell(via.x, via.y)
        # Check neighboring cells too (via has size)
        result = []
        seen = set()
        layer_cells = self.cells_by_layer[layer]
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                neighbor = (cell[0] + dx, cell[1] + dy)
                for obj, net_id in layer_cells.get(neighbor, []):
                    if id(obj) not in seen:
                        seen.add(id(obj))
                        result.append((obj, net_id))
        return result

    def get_nearby_vias(self, via: Via) -> List[Tuple[Via, int]]:
        """Get vias near the given via."""
        cell = self._get_cell(via.x, via.y)
        result = []
        seen = set()
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                neighbor = (cell[0] + dx, cell[1] + dy)
                for obj, net_id in self.all_layer_cells.get(neighbor, []):
                    if isinstance(obj, Via) and id(obj) not in seen:
                        seen.add(id(obj))
                        result.append((obj, net_id))
        return result

    def get_nearby_pads(self, x: float, y: float, layer: str) -> List[Tuple[Pad, int]]:
        """Get pads near a point on a specific layer."""
        cell = self._get_cell(x, y)
        result = []
        seen = set()
        layer_cells = self.cells_by_layer[layer]
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                neighbor = (cell[0] + dx, cell[1] + dy)
                for obj, net_id in layer_cells.get(neighbor, []):
                    if isinstance(obj, Pad) and id(obj) not in seen:
                        seen.add(id(obj))
                        result.append((obj, net_id))
        return result


def matches_any_pattern(name: str, patterns: List[str]) -> bool:
    """Check if a net name matches any of the given patterns (fnmatch style)."""
    for pattern in patterns:
        if fnmatch.fnmatch(name, pattern):
            return True
    return False


def segment_to_segment_distance(seg1: Segment, seg2: Segment) -> float:
    """Calculate minimum distance between two segments."""
    dist, _, _ = segment_to_segment_closest_points(seg1, seg2)
    return dist


def segments_cross(seg1: Segment, seg2: Segment, tolerance: float = 0.001) -> Tuple[bool, Optional[Tuple[float, float]]]:
    """Check if two segments on the same layer cross each other.

    Returns (True, intersection_point) if they cross, (False, None) otherwise.
    Segments that share an endpoint are not considered crossing.
    """
    if seg1.layer != seg2.layer:
        return False, None

    x1, y1 = seg1.start_x, seg1.start_y
    x2, y2 = seg1.end_x, seg1.end_y
    x3, y3 = seg2.start_x, seg2.start_y
    x4, y4 = seg2.end_x, seg2.end_y

    # Check if segments share an endpoint (not a crossing)
    def points_equal(ax, ay, bx, by):
        return abs(ax - bx) < tolerance and abs(ay - by) < tolerance

    if (points_equal(x1, y1, x3, y3) or points_equal(x1, y1, x4, y4) or
        points_equal(x2, y2, x3, y3) or points_equal(x2, y2, x4, y4)):
        return False, None

    # Direction vectors
    dx1, dy1 = x2 - x1, y2 - y1
    dx2, dy2 = x4 - x3, y4 - y3

    # Cross product of direction vectors
    cross = dx1 * dy2 - dy1 * dx2

    if abs(cross) < 1e-10:
        # Parallel segments - no crossing
        return False, None

    # Solve for parameters t and u where:
    # (x1, y1) + t * (dx1, dy1) = (x3, y3) + u * (dx2, dy2)
    dx3, dy3 = x3 - x1, y3 - y1
    t = (dx3 * dy2 - dy3 * dx2) / cross
    u = (dx3 * dy1 - dy3 * dx1) / cross

    # Check if intersection is within both segments (exclusive of endpoints)
    eps = 0.001  # Small margin to exclude near-endpoint intersections
    if eps < t < 1 - eps and eps < u < 1 - eps:
        # Calculate intersection point
        ix = x1 + t * dx1
        iy = y1 + t * dy1
        return True, (ix, iy)

    return False, None


def check_segment_overlap(seg1: Segment, seg2: Segment, clearance: float, clearance_margin: float = 0.10):
    """Check if two segments on the same layer violate clearance.

    Args:
        clearance_margin: Fraction of clearance to use as tolerance (default 0.10 = 10%).
                         Violations smaller than clearance * clearance_margin are ignored.

    Returns:
        (has_violation, overlap, closest_pt1, closest_pt2)
    """
    if seg1.layer != seg2.layer:
        return False, 0.0, None, None

    # Required distance is half-widths plus clearance
    required_dist = seg1.width / 2 + seg2.width / 2 + clearance
    actual_dist, pt1, pt2 = segment_to_segment_closest_points(seg1, seg2)
    overlap = required_dist - actual_dist

    # Use clearance-based tolerance (10% of clearance by default)
    tolerance = clearance * clearance_margin
    if overlap > tolerance:
        return True, overlap, pt1, pt2
    return False, 0.0, None, None


def check_via_segment_overlap(via: Via, seg: Segment, clearance: float, clearance_margin: float = 0.10) -> Tuple[bool, float]:
    """Check if a via overlaps with a segment on any common layer.

    Args:
        clearance_margin: Fraction of clearance to use as tolerance (default 0.10 = 10%).
    """
    # Standard through-hole vias go through ALL copper layers, not just the ones listed
    # Only skip non-copper layers
    if not seg.layer.endswith('.Cu'):
        return False, 0.0

    required_dist = via.size / 2 + seg.width / 2 + clearance
    actual_dist = point_to_segment_distance(via.x, via.y,
                                            seg.start_x, seg.start_y,
                                            seg.end_x, seg.end_y)
    overlap = required_dist - actual_dist

    tolerance = clearance * clearance_margin
    if overlap > tolerance:
        return True, overlap
    return False, 0.0


def check_via_via_overlap(via1: Via, via2: Via, clearance: float, clearance_margin: float = 0.10) -> Tuple[bool, float]:
    """Check if two vias overlap.

    Args:
        clearance_margin: Fraction of clearance to use as tolerance (default 0.10 = 10%).
    """
    # All vias are through-hole, so they always potentially conflict
    required_dist = via1.size / 2 + via2.size / 2 + clearance
    actual_dist = math.sqrt((via1.x - via2.x)**2 + (via1.y - via2.y)**2)
    overlap = required_dist - actual_dist

    tolerance = clearance * clearance_margin
    if overlap > tolerance:
        return True, overlap
    return False, 0.0


def point_to_rect_distance(px: float, py: float, cx: float, cy: float,
                           half_x: float, half_y: float,
                           corner_radius: float = 0.0) -> float:
    """Calculate distance from a point to an axis-aligned rectangle with optional rounded corners.

    Args:
        px, py: Point coordinates
        cx, cy: Rectangle center coordinates
        half_x, half_y: Rectangle half-widths
        corner_radius: Radius of rounded corners (0 for sharp corners)

    Returns:
        Distance from point to rectangle edge (0 if point is inside)
    """
    # Position relative to rectangle center
    rel_x = abs(px - cx)
    rel_y = abs(py - cy)

    if corner_radius > 0:
        # Inner rectangle bounds (where corners start)
        inner_half_x = half_x - corner_radius
        inner_half_y = half_y - corner_radius

        # Check if point is in a corner region
        if rel_x > inner_half_x and rel_y > inner_half_y:
            # Distance to corner arc center
            dx = rel_x - inner_half_x
            dy = rel_y - inner_half_y
            dist_to_corner_center = math.sqrt(dx * dx + dy * dy)
            # Distance to arc edge (negative if inside)
            return max(0, dist_to_corner_center - corner_radius)

    # Point is along a flat edge - rectangular distance
    dx = max(0, rel_x - half_x)
    dy = max(0, rel_y - half_y)
    return math.sqrt(dx * dx + dy * dy)


def segment_to_rect_distance(x1: float, y1: float, x2: float, y2: float,
                             cx: float, cy: float, half_x: float, half_y: float) -> Tuple[float, Tuple[float, float]]:
    """Calculate minimum distance from a segment to an axis-aligned rectangle.

    Args:
        x1, y1, x2, y2: Segment endpoints
        cx, cy: Rectangle center coordinates
        half_x, half_y: Rectangle half-widths

    Returns:
        (distance, closest_point_on_segment)
    """
    # Sample points along the segment and find minimum distance to rectangle
    # This is a simplified approach - for production code would use proper geometry
    min_dist = float('inf')
    closest_pt = (x1, y1)

    # Check endpoints and intermediate points
    num_samples = max(10, int(math.sqrt((x2-x1)**2 + (y2-y1)**2) / 0.05))  # Sample every ~0.05mm
    for i in range(num_samples + 1):
        t = i / num_samples
        px = x1 + t * (x2 - x1)
        py = y1 + t * (y2 - y1)
        dist = point_to_rect_distance(px, py, cx, cy, half_x, half_y)
        if dist < min_dist:
            min_dist = dist
            closest_pt = (px, py)

    return min_dist, closest_pt


def check_pad_segment_overlap(pad: Pad, seg: Segment, clearance: float,
                               routing_layers: List[str],
                               clearance_margin: float = 0.10) -> Tuple[bool, float, Optional[Tuple[float, float]]]:
    """Check if a segment is too close to a pad on the same layer.

    Args:
        pad: Pad object with global_x, global_y, size_x, size_y, layers
        seg: Segment to check against
        clearance: Minimum clearance in mm
        routing_layers: List of routing layer names (for expanding *.Cu wildcards)
        clearance_margin: Fraction of clearance to use as tolerance (default 0.10 = 10%).

    Returns:
        (has_violation, overlap_mm, closest_point_on_segment)
    """
    # Expand pad layers (handles *.Cu wildcards)
    expanded_layers = expand_pad_layers(pad.layers, routing_layers)

    # Check if segment is on a layer the pad is on
    if seg.layer not in expanded_layers:
        return False, 0.0, None

    # Calculate distance from segment to rectangular pad
    dist_to_pad, closest_pt = segment_to_rect_distance(
        seg.start_x, seg.start_y, seg.end_x, seg.end_y,
        pad.global_x, pad.global_y, pad.size_x / 2, pad.size_y / 2
    )

    # Required clearance: segment half-width + clearance
    # (dist_to_pad is already edge-to-edge from pad)
    required_dist = seg.width / 2 + clearance
    overlap = required_dist - dist_to_pad

    tolerance = clearance * clearance_margin
    if overlap > tolerance:
        return True, overlap, closest_pt

    return False, 0.0, None


def check_pad_via_overlap(pad: Pad, via: Via, clearance: float,
                          routing_layers: List[str],
                          clearance_margin: float = 0.10) -> Tuple[bool, float]:
    """Check if a via is too close to a pad.

    Args:
        pad: Pad object
        via: Via to check against
        clearance: Minimum clearance in mm
        routing_layers: List of routing layer names (for expanding *.Cu wildcards)
        clearance_margin: Fraction of clearance to use as tolerance (default 0.10 = 10%).

    Returns:
        (has_violation, overlap_mm)
    """
    # Expand pad layers (handles *.Cu wildcards)
    expanded_layers = expand_pad_layers(pad.layers, routing_layers)

    # Vias are through-hole, so they conflict with pads on any copper layer
    if not any(layer.endswith('.Cu') for layer in expanded_layers):
        return False, 0.0

    # Corner radius for roundrect pads
    corner_radius = pad.roundrect_rratio * min(pad.size_x, pad.size_y) if pad.shape == 'roundrect' else 0.0

    # Distance from via center to pad edge (accounts for rounded corners)
    dist_to_pad = point_to_rect_distance(
        via.x, via.y,
        pad.global_x, pad.global_y,
        pad.size_x / 2, pad.size_y / 2,
        corner_radius
    )

    # Required clearance: via half-size + clearance
    # (dist_to_pad is already edge-to-edge from pad)
    required_dist = via.size / 2 + clearance
    overlap = required_dist - dist_to_pad

    tolerance = clearance * clearance_margin
    if overlap > tolerance:
        return True, overlap

    return False, 0.0


def check_via_drill_overlap(via1: Via, via2: Via, hole_to_hole_clearance: float,
                            clearance_margin: float = 0.10) -> Tuple[bool, float]:
    """Check if two via drill holes violate hole-to-hole clearance.

    Args:
        via1, via2: Via objects with drill attribute
        hole_to_hole_clearance: Minimum clearance between drill hole edges in mm
        clearance_margin: Fraction of clearance to use as tolerance (default 0.10 = 10%).

    Returns:
        (has_violation, overlap_mm)
    """
    # Required distance between drill hole centers
    required_dist = via1.drill / 2 + via2.drill / 2 + hole_to_hole_clearance
    actual_dist = math.sqrt((via1.x - via2.x)**2 + (via1.y - via2.y)**2)
    overlap = required_dist - actual_dist

    tolerance = hole_to_hole_clearance * clearance_margin
    if overlap > tolerance:
        return True, overlap
    return False, 0.0


def check_pad_drill_via_overlap(pad: Pad, via: Via, hole_to_hole_clearance: float,
                                clearance_margin: float = 0.10) -> Tuple[bool, float]:
    """Check if a via drill hole is too close to a pad's drill hole.

    Args:
        pad: Pad object with drill attribute (through-hole pad)
        via: Via to check against
        hole_to_hole_clearance: Minimum clearance between drill hole edges in mm
        clearance_margin: Fraction of clearance to use as tolerance (default 0.10 = 10%).

    Returns:
        (has_violation, overlap_mm)
    """
    if pad.drill <= 0:
        return False, 0.0  # SMD pad, no drill

    # Required distance between drill hole centers
    required_dist = pad.drill / 2 + via.drill / 2 + hole_to_hole_clearance
    actual_dist = math.sqrt((pad.global_x - via.x)**2 + (pad.global_y - via.y)**2)
    overlap = required_dist - actual_dist

    tolerance = hole_to_hole_clearance * clearance_margin
    if overlap > tolerance:
        return True, overlap
    return False, 0.0


def check_segment_board_edge(seg: Segment, board_bounds: Tuple[float, float, float, float],
                             clearance: float, clearance_margin: float = 0.10) -> Tuple[bool, float, str]:
    """Check if a segment is too close to the board edge.

    Args:
        seg: Segment to check
        board_bounds: (min_x, min_y, max_x, max_y) of the board
        clearance: Minimum clearance from board edge in mm
        clearance_margin: Fraction of clearance to use as tolerance (default 0.10 = 10%).

    Returns:
        (has_violation, overlap_mm, edge_name)
    """
    min_x, min_y, max_x, max_y = board_bounds
    half_width = seg.width / 2
    required_clearance = clearance + half_width

    # Check all segment points against all edges
    for x, y in [(seg.start_x, seg.start_y), (seg.end_x, seg.end_y)]:
        # Left edge
        dist_left = x - min_x
        if dist_left < required_clearance:
            overlap = required_clearance - dist_left
            tolerance = clearance * clearance_margin
            if overlap > tolerance:
                return True, overlap, "left"

        # Right edge
        dist_right = max_x - x
        if dist_right < required_clearance:
            overlap = required_clearance - dist_right
            tolerance = clearance * clearance_margin
            if overlap > tolerance:
                return True, overlap, "right"

        # Bottom edge
        dist_bottom = y - min_y
        if dist_bottom < required_clearance:
            overlap = required_clearance - dist_bottom
            tolerance = clearance * clearance_margin
            if overlap > tolerance:
                return True, overlap, "bottom"

        # Top edge
        dist_top = max_y - y
        if dist_top < required_clearance:
            overlap = required_clearance - dist_top
            tolerance = clearance * clearance_margin
            if overlap > tolerance:
                return True, overlap, "top"

    return False, 0.0, ""


def check_via_board_edge(via: Via, board_bounds: Tuple[float, float, float, float],
                         clearance: float, clearance_margin: float = 0.10) -> Tuple[bool, float, str]:
    """Check if a via is too close to the board edge.

    Args:
        via: Via to check
        board_bounds: (min_x, min_y, max_x, max_y) of the board
        clearance: Minimum clearance from board edge in mm
        clearance_margin: Fraction of clearance to use as tolerance (default 0.10 = 10%).

    Returns:
        (has_violation, overlap_mm, edge_name)
    """
    min_x, min_y, max_x, max_y = board_bounds
    half_size = via.size / 2
    required_clearance = clearance + half_size

    x, y = via.x, via.y

    # Left edge
    dist_left = x - min_x
    if dist_left < required_clearance:
        overlap = required_clearance - dist_left
        tolerance = clearance * clearance_margin
        if overlap > tolerance:
            return True, overlap, "left"

    # Right edge
    dist_right = max_x - x
    if dist_right < required_clearance:
        overlap = required_clearance - dist_right
        tolerance = clearance * clearance_margin
        if overlap > tolerance:
            return True, overlap, "right"

    # Bottom edge
    dist_bottom = y - min_y
    if dist_bottom < required_clearance:
        overlap = required_clearance - dist_bottom
        tolerance = clearance * clearance_margin
        if overlap > tolerance:
            return True, overlap, "bottom"

    # Top edge
    dist_top = max_y - y
    if dist_top < required_clearance:
        overlap = required_clearance - dist_top
        tolerance = clearance * clearance_margin
        if overlap > tolerance:
            return True, overlap, "top"

    return False, 0.0, ""


def write_debug_lines(pcb_file: str, violations: List[dict], clearance: float, layer: str = "User.7"):
    """Write debug lines to PCB file showing violation locations.

    Adds gr_line elements connecting closest points of violating segments.
    """
    import uuid

    # Read the PCB file
    with open(pcb_file, 'r', encoding='utf-8') as f:
        content = f.read()

    # Generate gr_line elements for segment-segment violations
    debug_lines = []
    print(f"\nDebug lines (center-to-center distance, required clearance = {clearance}mm):")
    for v in violations:
        if v['type'] == 'segment-segment' and 'closest_pt1' in v and v['closest_pt1']:
            pt1 = v['closest_pt1']
            pt2 = v['closest_pt2']
            dist = math.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)
            # Track width is typically 0.1mm, so required center-to-center = 0.1 + clearance = 0.2mm
            required = 0.1 + clearance  # half-width + half-width + clearance = track_width + clearance
            violation_amt = required - dist
            print(f"  {v['net1']} <-> {v['net2']}: dist={dist:.4f}mm, required={required:.3f}mm, violation={violation_amt:.4f}mm")
            print(f"    from ({pt1[0]:.4f}, {pt1[1]:.4f}) to ({pt2[0]:.4f}, {pt2[1]:.4f})")

            line = f'''\t(gr_line
\t\t(start {pt1[0]:.6f} {pt1[1]:.6f})
\t\t(end {pt2[0]:.6f} {pt2[1]:.6f})
\t\t(stroke
\t\t\t(width 0.05)
\t\t\t(type solid)
\t\t)
\t\t(layer "{layer}")
\t\t(uuid "{uuid.uuid4()}")
\t)'''
            debug_lines.append(line)

    if not debug_lines:
        print(f"No debug lines to write")
        return

    # Insert before the final closing paren
    debug_text = '\n'.join(debug_lines)
    last_paren = content.rfind(')')
    new_content = content[:last_paren] + '\n' + debug_text + '\n' + content[last_paren:]

    with open(pcb_file, 'w', encoding='utf-8') as f:
        f.write(new_content)

    print(f"\nWrote {len(debug_lines)} debug line(s) to layer {layer}")


def run_drc(pcb_file: str, clearance: float = 0.1, net_patterns: Optional[List[str]] = None,
            debug_output: bool = False, quiet: bool = False,
            hole_to_hole_clearance: float = 0.2, board_edge_clearance: float = 0.0):
    """Run DRC checks on the PCB file.

    Args:
        pcb_file: Path to the KiCad PCB file
        clearance: Minimum clearance in mm
        net_patterns: Optional list of net name patterns (fnmatch style) to focus on.
                     If provided, only checks involving at least one matching net are reported.
        debug_output: If True, write debug lines to User.7 layer showing violation locations
        quiet: If True, only print a summary line unless there are violations
        hole_to_hole_clearance: Minimum clearance between drill hole edges in mm (default: 0.2)
        board_edge_clearance: Minimum clearance from board edge in mm (0 = use clearance)
    """
    # Use track clearance for board edge if not specified
    effective_board_edge_clearance = board_edge_clearance if board_edge_clearance > 0 else clearance
    if quiet and net_patterns:
        # Print a brief summary line in quiet mode
        print(f"Checking {', '.join(net_patterns)} for DRC...", end=" ", flush=True)
    elif not quiet:
        print(f"Loading {pcb_file}...")

    pcb_data = parse_kicad_pcb(pcb_file)

    if not quiet:
        print(f"Found {len(pcb_data.segments)} segments and {len(pcb_data.vias)} vias")

    # Helper to check if a net_id matches the filter patterns
    def net_matches_filter(net_id: int) -> bool:
        if net_patterns is None:
            return True  # No filter, include all
        net_info = pcb_data.nets.get(net_id, None)
        if net_info is None:
            return False
        return matches_any_pattern(net_info.name, net_patterns)

    # Helper to check if a violation involves at least one matching net
    def violation_matches_filter(net1_str: str, net2_str: str) -> bool:
        if net_patterns is None:
            return True
        return matches_any_pattern(net1_str, net_patterns) or matches_any_pattern(net2_str, net_patterns)

    if net_patterns and not quiet:
        print(f"Filtering to nets matching: {net_patterns}")

    # Get routing layers for pad layer expansion
    routing_layers = list(set(seg.layer for seg in pcb_data.segments if seg.layer.endswith('.Cu')))
    if not routing_layers:
        routing_layers = ['F.Cu', 'B.Cu']  # Fallback

    # Build spatial index for fast proximity queries
    if not quiet:
        print("Building spatial index...")
    spatial_idx = SpatialIndex(cell_size=2.0)  # 2mm cells

    # Add all segments to spatial index
    for seg in pcb_data.segments:
        spatial_idx.add_segment(seg, seg.net_id)

    # Add all vias to spatial index
    for via in pcb_data.vias:
        spatial_idx.add_via(via, via.net_id)

    # Add all pads to spatial index
    pads_by_net = pcb_data.pads_by_net
    for net_id, pads in pads_by_net.items():
        for pad in pads:
            expanded_layers = expand_pad_layers(pad.layers, routing_layers)
            spatial_idx.add_pad(pad, net_id, expanded_layers)

    # Group vias by net (still needed for some checks)
    vias_by_net = {}
    for via in pcb_data.vias:
        if via.net_id not in vias_by_net:
            vias_by_net[via.net_id] = []
        vias_by_net[via.net_id].append(via)

    violations = []

    # Pre-compute matching nets for filtering
    if net_patterns:
        matching_net_ids = set(net_id for net_id in pcb_data.nets.keys() if net_matches_filter(net_id))
        if not quiet:
            print(f"Filtering to {len(matching_net_ids)} matching nets")
    else:
        matching_net_ids = None

    # Check segment-to-segment violations using spatial index
    if not quiet:
        print("\nChecking segment-to-segment clearances...")

    checked_pairs = set()  # Track checked segment pairs to avoid duplicates
    for seg1 in pcb_data.segments:
        net1 = seg1.net_id
        net1_matches = matching_net_ids is None or net1 in matching_net_ids

        # Get nearby segments from spatial index (same layer only)
        for seg2, net2 in spatial_idx.get_nearby_segments(seg1):
            if net1 == net2:
                continue  # Same net
            if seg1 is seg2:
                continue  # Same segment

            # Skip if neither net matches filter
            net2_matches = matching_net_ids is None or net2 in matching_net_ids
            if not net1_matches and not net2_matches:
                continue

            # Avoid checking same pair twice
            pair_key = (min(id(seg1), id(seg2)), max(id(seg1), id(seg2)))
            if pair_key in checked_pairs:
                continue
            checked_pairs.add(pair_key)

            has_violation, overlap, pt1, pt2 = check_segment_overlap(seg1, seg2, clearance)
            if has_violation:
                net1_name = pcb_data.nets.get(net1, None)
                net2_name = pcb_data.nets.get(net2, None)
                net1_str = net1_name.name if net1_name else f"net_{net1}"
                net2_str = net2_name.name if net2_name else f"net_{net2}"
                violations.append({
                    'type': 'segment-segment',
                    'net1': net1_str,
                    'net2': net2_str,
                    'layer': seg1.layer,
                    'overlap_mm': overlap,
                    'loc1': (seg1.start_x, seg1.start_y, seg1.end_x, seg1.end_y),
                    'loc2': (seg2.start_x, seg2.start_y, seg2.end_x, seg2.end_y),
                    'closest_pt1': pt1,
                    'closest_pt2': pt2,
                })

            # Also check for segment crossings (different nets)
            crosses, cross_point = segments_cross(seg1, seg2)
            if crosses:
                net1_name = pcb_data.nets.get(net1, None)
                net2_name = pcb_data.nets.get(net2, None)
                net1_str = net1_name.name if net1_name else f"net_{net1}"
                net2_str = net2_name.name if net2_name else f"net_{net2}"
                violations.append({
                    'type': 'segment-crossing',
                    'net1': net1_str,
                    'net2': net2_str,
                    'layer': seg1.layer,
                    'cross_point': cross_point,
                    'loc1': (seg1.start_x, seg1.start_y, seg1.end_x, seg1.end_y),
                    'loc2': (seg2.start_x, seg2.start_y, seg2.end_x, seg2.end_y),
                })

    # Check for same-net segment crossings using spatial index
    if not quiet:
        print("Checking for same-net segment crossings...")
    same_net_checked = set()
    for seg1 in pcb_data.segments:
        net_id = seg1.net_id
        if matching_net_ids is not None and net_id not in matching_net_ids:
            continue
        for seg2, net2 in spatial_idx.get_nearby_segments(seg1):
            if net2 != net_id:
                continue  # Different net
            if seg1 is seg2:
                continue
            pair_key = (min(id(seg1), id(seg2)), max(id(seg1), id(seg2)))
            if pair_key in same_net_checked:
                continue
            same_net_checked.add(pair_key)
            crosses, cross_point = segments_cross(seg1, seg2)
            if crosses:
                net_name = pcb_data.nets.get(net_id, None)
                net_str = net_name.name if net_name else f"net_{net_id}"
                violations.append({
                    'type': 'segment-crossing-same-net',
                    'net1': net_str,
                    'net2': net_str,
                    'layer': seg1.layer,
                    'cross_point': cross_point,
                    'loc1': (seg1.start_x, seg1.start_y, seg1.end_x, seg1.end_y),
                    'loc2': (seg2.start_x, seg2.start_y, seg2.end_x, seg2.end_y),
                })

    # Check via-to-segment violations using spatial index
    if not quiet:
        print("Checking via-to-segment clearances...")
    for via in pcb_data.vias:
        via_net = via.net_id
        via_net_matches = matching_net_ids is None or via_net in matching_net_ids

        # Check against segments on each copper layer (vias go through all layers)
        for layer in routing_layers:
            for obj, seg_net in spatial_idx.get_nearby_for_via(via, layer):
                if not isinstance(obj, Segment):
                    continue
                seg = obj
                if via_net == seg_net:
                    continue  # Same net

                seg_net_matches = matching_net_ids is None or seg_net in matching_net_ids
                if not via_net_matches and not seg_net_matches:
                    continue

                has_violation, overlap = check_via_segment_overlap(via, seg, clearance)
                if has_violation:
                    via_net_name = pcb_data.nets.get(via_net, None)
                    seg_net_name = pcb_data.nets.get(seg_net, None)
                    via_net_str = via_net_name.name if via_net_name else f"net_{via_net}"
                    seg_net_str = seg_net_name.name if seg_net_name else f"net_{seg_net}"
                    violations.append({
                        'type': 'via-segment',
                        'net1': via_net_str,
                        'net2': seg_net_str,
                        'layer': seg.layer,
                        'overlap_mm': overlap,
                        'via_loc': (via.x, via.y),
                        'seg_loc': (seg.start_x, seg.start_y, seg.end_x, seg.end_y),
                    })

    # Check via-to-via violations using spatial index
    if not quiet:
        print("Checking via-to-via clearances...")
    via_via_checked = set()
    for via1 in pcb_data.vias:
        net1 = via1.net_id
        net1_matches = matching_net_ids is None or net1 in matching_net_ids

        for via2, net2 in spatial_idx.get_nearby_vias(via1):
            if via1 is via2:
                continue

            net2_matches = matching_net_ids is None or net2 in matching_net_ids
            if not net1_matches and not net2_matches:
                continue

            pair_key = (min(id(via1), id(via2)), max(id(via1), id(via2)))
            if pair_key in via_via_checked:
                continue
            via_via_checked.add(pair_key)

            has_violation, overlap = check_via_via_overlap(via1, via2, clearance)
            if has_violation:
                net1_name = pcb_data.nets.get(net1, None)
                net2_name = pcb_data.nets.get(net2, None)
                net1_str = net1_name.name if net1_name else f"net_{net1}"
                net2_str = net2_name.name if net2_name else f"net_{net2}"
                violations.append({
                    'type': 'via-via' if net1 != net2 else 'via-via-same-net',
                    'net1': net1_str,
                    'net2': net2_str,
                    'overlap_mm': overlap,
                    'loc1': (via1.x, via1.y),
                    'loc2': (via2.x, via2.y),
                })

    # Check pad-to-segment violations using spatial index
    if not quiet:
        print("Checking pad-to-segment clearances...")

    pad_net_ids = list(pads_by_net.keys())
    for seg in pcb_data.segments:
        seg_net = seg.net_id
        seg_net_matches = matching_net_ids is None or seg_net in matching_net_ids

        # Get pads near the segment endpoints
        for x, y in [(seg.start_x, seg.start_y), (seg.end_x, seg.end_y)]:
            for pad, pad_net in spatial_idx.get_nearby_pads(x, y, seg.layer):
                if pad_net == seg_net:
                    continue  # Same net

                pad_net_matches = matching_net_ids is None or pad_net in matching_net_ids
                if not seg_net_matches and not pad_net_matches:
                    continue

                has_violation, overlap, closest_pt = check_pad_segment_overlap(
                    pad, seg, clearance, routing_layers
                )
                if has_violation:
                    pad_net_name = pcb_data.nets.get(pad_net, None)
                    seg_net_name = pcb_data.nets.get(seg_net, None)
                    pad_net_str = pad_net_name.name if pad_net_name else f"net_{pad_net}"
                    seg_net_str = seg_net_name.name if seg_net_name else f"net_{seg_net}"
                    violations.append({
                        'type': 'pad-segment',
                        'net1': pad_net_str,
                        'net2': seg_net_str,
                        'layer': seg.layer,
                        'overlap_mm': overlap,
                        'pad_loc': (pad.global_x, pad.global_y),
                        'pad_ref': f"{pad.component_ref}.{pad.pad_number}",
                        'seg_loc': (seg.start_x, seg.start_y, seg.end_x, seg.end_y),
                        'closest_pt': closest_pt,
                    })

    # Check pad-to-via violations using spatial index
    if not quiet:
        print("Checking pad-to-via clearances...")

    for via in pcb_data.vias:
        via_net = via.net_id
        via_net_matches = matching_net_ids is None or via_net in matching_net_ids

        for layer in routing_layers:
            for pad, pad_net in spatial_idx.get_nearby_pads(via.x, via.y, layer):
                if pad_net == via_net:
                    continue  # Same net

                pad_net_matches = matching_net_ids is None or pad_net in matching_net_ids
                if not via_net_matches and not pad_net_matches:
                    continue

                has_violation, overlap = check_pad_via_overlap(
                    pad, via, clearance, routing_layers
                )
                if has_violation:
                    pad_net_name = pcb_data.nets.get(pad_net, None)
                    via_net_name = pcb_data.nets.get(via_net, None)
                    pad_net_str = pad_net_name.name if pad_net_name else f"net_{pad_net}"
                    via_net_str = via_net_name.name if via_net_name else f"net_{via_net}"
                    violations.append({
                        'type': 'pad-via',
                        'net1': pad_net_str,
                        'net2': via_net_str,
                        'overlap_mm': overlap,
                        'pad_loc': (pad.global_x, pad.global_y),
                        'pad_ref': f"{pad.component_ref}.{pad.pad_number}",
                        'via_loc': (via.x, via.y),
                    })

    # Dummy variables for compatibility with remaining code
    via_net_ids = list(vias_by_net.keys())
    matching_via_nets = matching_net_ids
    matching_seg_net_set = matching_net_ids
    matching_pad_nets = matching_net_ids

    # Check hole-to-hole clearance (via drill to via drill)
    if hole_to_hole_clearance > 0:
        if not quiet:
            print("Checking via drill hole-to-hole clearances...")
        all_vias = list(pcb_data.vias)
        for i in range(len(all_vias)):
            via1 = all_vias[i]
            via1_matches = matching_via_nets is None or via1.net_id in matching_via_nets
            for j in range(i + 1, len(all_vias)):
                via2 = all_vias[j]
                # Skip if neither net matches filter
                via2_matches = matching_via_nets is None or via2.net_id in matching_via_nets
                if not via1_matches and not via2_matches:
                    continue
                has_violation, overlap = check_via_drill_overlap(via1, via2, hole_to_hole_clearance)
                if has_violation:
                    net1_name = pcb_data.nets.get(via1.net_id, None)
                    net2_name = pcb_data.nets.get(via2.net_id, None)
                    net1_str = net1_name.name if net1_name else f"net_{via1.net_id}"
                    net2_str = net2_name.name if net2_name else f"net_{via2.net_id}"
                    violations.append({
                        'type': 'via-drill-hole',
                        'net1': net1_str,
                        'net2': net2_str,
                        'overlap_mm': overlap,
                        'loc1': (via1.x, via1.y),
                        'loc2': (via2.x, via2.y),
                    })

        # Check via drill to pad drill (through-hole pads)
        # NOTE: Hole-to-hole clearance applies regardless of net (manufacturing constraint)
        if not quiet:
            print("Checking via drill to pad drill clearances...")
        for via in all_vias:
            via_matches = matching_via_nets is None or via.net_id in matching_via_nets
            for pad_net, pads in pads_by_net.items():
                # Don't skip same-net - hole clearance is a manufacturing constraint
                pad_net_matches = matching_pad_nets is None or pad_net in matching_pad_nets
                if not via_matches and not pad_net_matches:
                    continue
                for pad in pads:
                    if pad.drill <= 0:
                        continue  # SMD pad
                    has_violation, overlap = check_pad_drill_via_overlap(pad, via, hole_to_hole_clearance)
                    if has_violation:
                        via_net_name = pcb_data.nets.get(via.net_id, None)
                        pad_net_name = pcb_data.nets.get(pad_net, None)
                        via_net_str = via_net_name.name if via_net_name else f"net_{via.net_id}"
                        pad_net_str = pad_net_name.name if pad_net_name else f"net_{pad_net}"
                        same_net = pad_net == via.net_id
                        violations.append({
                            'type': 'pad-drill-via-drill-same-net' if same_net else 'pad-drill-via-drill',
                            'net1': pad_net_str,
                            'net2': via_net_str,
                            'overlap_mm': overlap,
                            'pad_loc': (pad.global_x, pad.global_y),
                            'pad_ref': f"{pad.component_ref}.{pad.pad_number}",
                            'via_loc': (via.x, via.y),
                        })

    # Check board edge clearances
    board_bounds = pcb_data.board_info.board_bounds
    if board_bounds and effective_board_edge_clearance > 0:
        if not quiet:
            print("Checking board edge clearances...")

        # Check segments
        for seg in pcb_data.segments:
            seg_matches = matching_seg_net_set is None or seg.net_id in matching_seg_net_set
            if matching_seg_net_set is not None and not seg_matches:
                continue
            has_violation, overlap, edge = check_segment_board_edge(seg, board_bounds, effective_board_edge_clearance)
            if has_violation:
                net_name = pcb_data.nets.get(seg.net_id, None)
                net_str = net_name.name if net_name else f"net_{seg.net_id}"
                violations.append({
                    'type': 'segment-board-edge',
                    'net1': net_str,
                    'edge': edge,
                    'layer': seg.layer,
                    'overlap_mm': overlap,
                    'seg_loc': (seg.start_x, seg.start_y, seg.end_x, seg.end_y),
                })

        # Check vias
        for via in pcb_data.vias:
            via_matches = matching_via_nets is None or via.net_id in matching_via_nets
            if matching_via_nets is not None and not via_matches:
                continue
            has_violation, overlap, edge = check_via_board_edge(via, board_bounds, effective_board_edge_clearance)
            if has_violation:
                net_name = pcb_data.nets.get(via.net_id, None)
                net_str = net_name.name if net_name else f"net_{via.net_id}"
                violations.append({
                    'type': 'via-board-edge',
                    'net1': net_str,
                    'edge': edge,
                    'overlap_mm': overlap,
                    'via_loc': (via.x, via.y),
                })

    # Report violations
    if quiet:
        if violations:
            print(f"FAILED ({len(violations)} violations)")
        else:
            print("OK")
            return violations

    # Print detailed results (always for non-quiet, or when violations in quiet mode)
    if not quiet or violations:
        print("\n" + "=" * 60 if not quiet else "=" * 60)
        if violations:
            print(f"FOUND {len(violations)} DRC VIOLATIONS:\n")

            # Group by type
            by_type = {}
            for v in violations:
                t = v['type']
                if t not in by_type:
                    by_type[t] = []
                by_type[t].append(v)

            for vtype, vlist in by_type.items():
                print(f"\n{vtype.upper()} violations ({len(vlist)}):")
                print("-" * 40)
                for v in vlist[:20]:  # Show first 20 of each type
                    if vtype == 'segment-segment':
                        print(f"  {v['net1']} <-> {v['net2']}")
                        print(f"    Layer: {v['layer']}, Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Seg1: ({v['loc1'][0]:.2f},{v['loc1'][1]:.2f})-({v['loc1'][2]:.2f},{v['loc1'][3]:.2f})")
                        print(f"    Seg2: ({v['loc2'][0]:.2f},{v['loc2'][1]:.2f})-({v['loc2'][2]:.2f},{v['loc2'][3]:.2f})")
                    elif vtype == 'via-segment':
                        print(f"  Via:{v['net1']} <-> Seg:{v['net2']}")
                        print(f"    Layer: {v['layer']}, Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Via: ({v['via_loc'][0]:.2f},{v['via_loc'][1]:.2f})")
                        print(f"    Seg: ({v['seg_loc'][0]:.2f},{v['seg_loc'][1]:.2f})-({v['seg_loc'][2]:.2f},{v['seg_loc'][3]:.2f})")
                    elif vtype == 'via-via':
                        print(f"  {v['net1']} <-> {v['net2']}")
                        print(f"    Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Via1: ({v['loc1'][0]:.2f},{v['loc1'][1]:.2f})")
                        print(f"    Via2: ({v['loc2'][0]:.2f},{v['loc2'][1]:.2f})")
                    elif vtype in ('segment-crossing', 'segment-crossing-same-net'):
                        print(f"  {v['net1']} <-> {v['net2']}")
                        print(f"    Layer: {v['layer']}, Cross at: ({v['cross_point'][0]:.3f},{v['cross_point'][1]:.3f})")
                        print(f"    Seg1: ({v['loc1'][0]:.2f},{v['loc1'][1]:.2f})-({v['loc1'][2]:.2f},{v['loc1'][3]:.2f})")
                        print(f"    Seg2: ({v['loc2'][0]:.2f},{v['loc2'][1]:.2f})-({v['loc2'][2]:.2f},{v['loc2'][3]:.2f})")
                    elif vtype == 'pad-segment':
                        print(f"  Pad:{v['net1']} ({v['pad_ref']}) <-> Seg:{v['net2']}")
                        print(f"    Layer: {v['layer']}, Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Pad: ({v['pad_loc'][0]:.2f},{v['pad_loc'][1]:.2f})")
                        print(f"    Seg: ({v['seg_loc'][0]:.2f},{v['seg_loc'][1]:.2f})-({v['seg_loc'][2]:.2f},{v['seg_loc'][3]:.2f})")
                    elif vtype == 'pad-via':
                        print(f"  Pad:{v['net1']} ({v['pad_ref']}) <-> Via:{v['net2']}")
                        print(f"    Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Pad: ({v['pad_loc'][0]:.2f},{v['pad_loc'][1]:.2f})")
                        print(f"    Via: ({v['via_loc'][0]:.2f},{v['via_loc'][1]:.2f})")
                    elif vtype == 'via-drill-hole':
                        print(f"  Via:{v['net1']} <-> Via:{v['net2']} (drill hole clearance)")
                        print(f"    Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Via1: ({v['loc1'][0]:.2f},{v['loc1'][1]:.2f})")
                        print(f"    Via2: ({v['loc2'][0]:.2f},{v['loc2'][1]:.2f})")
                    elif vtype in ('pad-drill-via-drill', 'pad-drill-via-drill-same-net'):
                        same_net_msg = " [SAME NET]" if vtype.endswith('same-net') else ""
                        print(f"  Pad:{v['net1']} ({v['pad_ref']}) <-> Via:{v['net2']} (drill hole clearance){same_net_msg}")
                        print(f"    Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Pad: ({v['pad_loc'][0]:.2f},{v['pad_loc'][1]:.2f})")
                        print(f"    Via: ({v['via_loc'][0]:.2f},{v['via_loc'][1]:.2f})")
                    elif vtype == 'segment-board-edge':
                        print(f"  {v['net1']} too close to {v['edge']} board edge")
                        print(f"    Layer: {v['layer']}, Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Seg: ({v['seg_loc'][0]:.2f},{v['seg_loc'][1]:.2f})-({v['seg_loc'][2]:.2f},{v['seg_loc'][3]:.2f})")
                    elif vtype == 'via-board-edge':
                        print(f"  Via:{v['net1']} too close to {v['edge']} board edge")
                        print(f"    Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Via: ({v['via_loc'][0]:.2f},{v['via_loc'][1]:.2f})")

                if len(vlist) > 20:
                    print(f"  ... and {len(vlist) - 20} more")
        else:
            print("NO DRC VIOLATIONS FOUND!")

        print("=" * 60)

    # Write debug lines if requested
    if debug_output and violations:
        write_debug_lines(pcb_file, violations, clearance)

    return violations


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Check PCB for DRC violations (clearance errors)')
    parser.add_argument('pcb', help='Input PCB file')
    parser.add_argument('--clearance', '-c', type=float, default=0.1,
                        help='Minimum clearance in mm (default: 0.1)')
    parser.add_argument('--hole-to-hole-clearance', type=float, default=0.2,
                        help='Minimum drill hole edge-to-edge clearance in mm (default: 0.2)')
    parser.add_argument('--board-edge-clearance', type=float, default=0.0,
                        help='Minimum clearance from board edge in mm (0 = use --clearance value)')
    parser.add_argument('--nets', '-n', nargs='+', default=None,
                        help='Optional net name patterns to focus on (fnmatch wildcards supported, e.g., "*lvds*")')
    parser.add_argument('--debug-lines', '-d', action='store_true',
                        help='Write debug lines to User.7 layer showing violation locations')
    parser.add_argument('--quiet', '-q', action='store_true',
                        help='Only print a summary line unless there are violations')

    args = parser.parse_args()

    violations = run_drc(args.pcb, args.clearance, args.nets, args.debug_lines, args.quiet,
                         args.hole_to_hole_clearance, args.board_edge_clearance)
    sys.exit(1 if violations else 0)

"""
DRC Checker - Find overlapping tracks and vias between different nets.
"""

import sys
import argparse
import math
import fnmatch
import numpy as np
from collections import defaultdict
from typing import List, Tuple, Set, Optional, Dict, Any
from kicad_parser import parse_kicad_pcb, Segment, Via, Pad
from geometry_utils import (
    point_to_segment_distance,
    closest_point_on_segment,
    segment_to_segment_closest_points,
    segment_to_segment_distance as _seg_seg_dist_coords,
)
from net_queries import expand_pad_layers
import routing_defaults as defaults


# Per-run memo for expand_pad_layers. Within one run_drc call routing_layers is a
# single fixed list, and pads share a handful of distinct layer-sets (['F.Cu'],
# ['*.Cu'], ...), yet expand_pad_layers is called once per pad per check (tens of
# thousands of times). Caching by the pad's layer tuple collapses that to a few
# real computations. Identical results -- the expansion is a pure function of
# (pad_layers, routing_layers). The cache is keyed on the routing_layers object
# identity; a new run passes a new list, which clears the cache. We hold a
# reference to that list so its id() can't be recycled while the cache is live.
_EXPAND_CACHE: Dict[tuple, List[str]] = {}
_EXPAND_ROUTING = None


def _expand_cu(pad_layers: List[str], routing_layers: List[str]) -> List[str]:
    """Memoized expand_pad_layers for the duration of one run_drc call."""
    global _EXPAND_ROUTING
    if routing_layers is not _EXPAND_ROUTING:
        _EXPAND_CACHE.clear()
        _EXPAND_ROUTING = routing_layers
    key = tuple(pad_layers)
    cached = _EXPAND_CACHE.get(key)
    if cached is None:
        cached = expand_pad_layers(pad_layers, routing_layers)
        _EXPAND_CACHE[key] = cached
    return cached


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

    def get_nearby_pads_for_pad(self, pad: "Pad", layer: str) -> List[Tuple["Pad", int]]:
        """Get pads near another pad on a layer. Unlike get_nearby_pads (which
        keys off a single point), this scans every cell the query pad spans plus
        a one-cell margin, so a large pad does not miss a neighbour sitting near
        its edge rather than its center (pad-pad check, #234)."""
        half_x = pad.size_x / 2
        half_y = pad.size_y / 2
        min_cell = self._get_cell(pad.global_x - half_x, pad.global_y - half_y)
        max_cell = self._get_cell(pad.global_x + half_x, pad.global_y + half_y)
        layer_cells = self.cells_by_layer[layer]
        seen = set()
        result = []
        for cx in range(min_cell[0] - 1, max_cell[0] + 2):
            for cy in range(min_cell[1] - 1, max_cell[1] + 2):
                for obj, net_id in layer_cells.get((cx, cy), []):
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


def check_segment_overlap(seg1: Segment, seg2: Segment, clearance: float, clearance_margin: float = 0.05):
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


def check_via_segment_overlap(via: Via, seg: Segment, clearance: float, clearance_margin: float = 0.05) -> Tuple[bool, float]:
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


def check_via_via_overlap(via1: Via, via2: Via, clearance: float, clearance_margin: float = 0.05) -> Tuple[bool, float]:
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
                             cx: float, cy: float, half_x: float, half_y: float,
                             corner_radius: float = 0.0) -> Tuple[float, Tuple[float, float]]:
    """Calculate minimum distance from a segment to an axis-aligned rectangle.

    Args:
        x1, y1, x2, y2: Segment endpoints
        cx, cy: Rectangle center coordinates
        half_x, half_y: Rectangle half-widths
        corner_radius: Radius of rounded corners (0 for sharp corners)

    Returns:
        (distance, closest_point_on_segment)
    """
    # Sample points along the segment and find minimum distance to rectangle.
    # point_to_rect_distance is inlined here (this is the DRC hotspot -- one call
    # per sample, hundreds of thousands per board) with the corner-radius branch
    # invariants hoisted out of the loop. The per-sample arithmetic is identical
    # to point_to_rect_distance, so the result is unchanged.
    min_dist = float('inf')
    closest_pt = (x1, y1)
    dx_seg = x2 - x1
    dy_seg = y2 - y1
    has_corner = corner_radius > 0
    inner_half_x = half_x - corner_radius
    inner_half_y = half_y - corner_radius
    sqrt = math.sqrt

    # Check endpoints and intermediate points (keep the original expression form
    # verbatim so num_samples is bit-identical to before)
    num_samples = max(10, int(math.sqrt((x2-x1)**2 + (y2-y1)**2) / 0.05))  # Sample every ~0.05mm
    for i in range(num_samples + 1):
        t = i / num_samples
        px = x1 + t * dx_seg
        py = y1 + t * dy_seg
        rel_x = abs(px - cx)
        rel_y = abs(py - cy)
        if has_corner and rel_x > inner_half_x and rel_y > inner_half_y:
            ddx = rel_x - inner_half_x
            ddy = rel_y - inner_half_y
            dist = max(0, sqrt(ddx * ddx + ddy * ddy) - corner_radius)
        else:
            ex = max(0, rel_x - half_x)
            ey = max(0, rel_y - half_y)
            dist = sqrt(ex * ex + ey * ey)
        if dist < min_dist:
            min_dist = dist
            closest_pt = (px, py)

    return min_dist, closest_pt


def _into_pad_frame(x: float, y: float, pad: Pad,
                    cos_r: float, sin_r: float) -> Tuple[float, float]:
    """Rotate a global point into a diagonal pad's local frame about its center
    (R(-rect_rotation)), so the axis-aligned rect-distance routines apply."""
    dx = x - pad.global_x
    dy = y - pad.global_y
    return (pad.global_x + dx * cos_r + dy * sin_r,
            pad.global_y - dx * sin_r + dy * cos_r)


def _point_in_poly(x: float, y: float, poly) -> bool:
    n = len(poly)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def _point_to_polys_distance(x: float, y: float, polys) -> float:
    """Min distance from a point to any of the polygons (0 if inside one)."""
    best = float('inf')
    for poly in polys:
        if _point_in_poly(x, y, poly):
            return 0.0
        n = len(poly)
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            d = point_to_segment_distance(x, y, x1, y1, x2, y2)
            if d < best:
                best = d
    return best


def _segment_to_polys_distance(x1: float, y1: float, x2: float, y2: float, polys):
    """Min distance from a segment to custom-pad polygon copper (0 if it enters),
    sampled along the segment like segment_to_rect_distance. Returns (dist, pt)."""
    length = math.hypot(x2 - x1, y2 - y1)
    n = max(10, int(length / 0.05))
    best = float('inf')
    best_pt = (x1, y1)
    for i in range(n + 1):
        t = i / n
        px = x1 + t * (x2 - x1)
        py = y1 + t * (y2 - y1)
        d = _point_to_polys_distance(px, py, polys)
        if d < best:
            best = d
            best_pt = (px, py)
    return best, best_pt


def point_to_pad_distance(px: float, py: float, pad: Pad) -> float:
    """Edge-to-edge distance from a point to a pad's copper (0 if inside the
    copper). Handles custom-polygon pads, rounded/rect/circle/oval shapes, and
    diagonal (rect_rotation) pads -- the same geometry the pad-segment and
    pad-via checks use, factored out for reuse by the pad-pad check (#234)."""
    pad_polys = getattr(pad, 'polygons', None)
    if pad_polys:
        return _point_to_polys_distance(px, py, pad_polys)

    if pad.shape in ('circle', 'oval'):
        corner_radius = min(pad.size_x, pad.size_y) / 2
    elif pad.shape == 'roundrect':
        corner_radius = pad.roundrect_rratio * min(pad.size_x, pad.size_y)
    else:
        corner_radius = 0.0

    x, y = px, py
    if pad.rect_rotation:
        rad = math.radians(pad.rect_rotation)
        x, y = _into_pad_frame(x, y, pad, math.cos(rad), math.sin(rad))
    return point_to_rect_distance(x, y, pad.global_x, pad.global_y,
                                  pad.size_x / 2, pad.size_y / 2, corner_radius)


def _pad_perimeter_points(pad: Pad, n_per_side: int = 8) -> List[Tuple[float, float]]:
    """Sample points around a pad's copper perimeter in global coordinates.
    For custom-polygon pads, walk the real polygon edges; otherwise walk the
    (rotated) bounding rectangle. Used to measure pad-pad and pad-edge gaps by
    cross-sampling one pad's perimeter against the other shape's distance fn."""
    pad_polys = getattr(pad, 'polygons', None)
    if pad_polys:
        pts = []
        for poly in pad_polys:
            m = len(poly)
            for i in range(m):
                x1, y1 = poly[i]
                x2, y2 = poly[(i + 1) % m]
                for k in range(n_per_side):
                    t = k / n_per_side
                    pts.append((x1 + t * (x2 - x1), y1 + t * (y2 - y1)))
        return pts

    hx, hy = pad.size_x / 2, pad.size_y / 2
    cx, cy = pad.global_x, pad.global_y
    corners = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
    local = []
    for i in range(4):
        x1, y1 = corners[i]
        x2, y2 = corners[(i + 1) % 4]
        for k in range(n_per_side):
            t = k / n_per_side
            local.append((x1 + t * (x2 - x1), y1 + t * (y2 - y1)))
    if pad.rect_rotation:
        rad = math.radians(pad.rect_rotation)
        c, s = math.cos(rad), math.sin(rad)
        return [(cx + lx * c - ly * s, cy + lx * s + ly * c) for lx, ly in local]
    return [(cx + lx, cy + ly) for lx, ly in local]


def check_pad_pad_overlap(pad1: Pad, pad2: Pad, clearance: float,
                          routing_layers: List[str],
                          clearance_margin: float = 0.05
                          ) -> Tuple[bool, float, Optional[Tuple[float, float]]]:
    """Check if two pads of different nets are below clearance (or overlap/short)
    on a shared copper layer (issue #234). KiCad flags these as clearance /
    shorting_items; check_drc previously only had pad-segment and pad-via passes.

    Distance is edge-to-edge, computed by cross-sampling each pad's perimeter
    against the other pad's exact distance function (rect/roundrect/circle +
    rect_rotation, or the real polygon for custom pads -- avoiding the #232
    bounding-box phantom-hit caveat).

    Returns (has_violation, overlap_mm, closest_point).
    """
    l1 = _expand_cu(pad1.layers, routing_layers)
    l2 = _expand_cu(pad2.layers, routing_layers)
    shared = any(l in l2 and l.endswith('.Cu') for l in l1)
    if not shared:
        return False, 0.0, None

    best = float('inf')
    best_pt = None
    for px, py in _pad_perimeter_points(pad1):
        d = point_to_pad_distance(px, py, pad2)
        if d < best:
            best = d
            best_pt = (px, py)
    for px, py in _pad_perimeter_points(pad2):
        d = point_to_pad_distance(px, py, pad1)
        if d < best:
            best = d
            best_pt = (px, py)

    overlap = clearance - best
    if overlap > clearance * clearance_margin:
        return True, overlap, best_pt
    return False, 0.0, None


def check_pad_segment_overlap(pad: Pad, seg: Segment, clearance: float,
                               routing_layers: List[str],
                               clearance_margin: float = 0.05) -> Tuple[bool, float, Optional[Tuple[float, float]]]:
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
    expanded_layers = _expand_cu(pad.layers, routing_layers)

    # Check if segment is on a layer the pad is on
    if seg.layer not in expanded_layers:
        return False, 0.0, None

    # Custom comb/finger pads: measure against the real copper polygon(s), not the
    # bounding box, so a track legitimately threading a finger channel is not
    # flagged (issue #188).
    pad_polys = getattr(pad, 'polygons', None)
    if pad_polys:
        dist_to_pad, closest_pt = _segment_to_polys_distance(
            seg.start_x, seg.start_y, seg.end_x, seg.end_y, pad_polys)
        required_dist = seg.width / 2 + clearance
        overlap = required_dist - dist_to_pad
        if overlap > clearance * clearance_margin:
            return True, overlap, closest_pt
        return False, 0.0, None

    # Corner radius based on pad shape (circle/oval use min dimension, roundrect uses rratio)
    if pad.shape in ('circle', 'oval'):
        corner_radius = min(pad.size_x, pad.size_y) / 2
    elif pad.shape == 'roundrect':
        corner_radius = pad.roundrect_rratio * min(pad.size_x, pad.size_y)
    else:
        corner_radius = 0.0

    # Calculate distance from segment to rectangular pad (with optional rounded
    # corners). For diagonal pads, rotate the segment into the pad's local frame
    # so the axis-aligned rect distance is exact (distance is rotation-invariant).
    sx0, sy0, ex0, ey0 = seg.start_x, seg.start_y, seg.end_x, seg.end_y
    if pad.rect_rotation:
        rad = math.radians(pad.rect_rotation)
        cos_r, sin_r = math.cos(rad), math.sin(rad)
        sx0, sy0 = _into_pad_frame(sx0, sy0, pad, cos_r, sin_r)
        ex0, ey0 = _into_pad_frame(ex0, ey0, pad, cos_r, sin_r)
    dist_to_pad, closest_pt = segment_to_rect_distance(
        sx0, sy0, ex0, ey0,
        pad.global_x, pad.global_y, pad.size_x / 2, pad.size_y / 2,
        corner_radius
    )
    if pad.rect_rotation and closest_pt is not None:
        # Report the closest point back in the global frame (R(+rect_rotation)).
        cdx = closest_pt[0] - pad.global_x
        cdy = closest_pt[1] - pad.global_y
        closest_pt = (pad.global_x + cdx * cos_r - cdy * sin_r,
                      pad.global_y + cdx * sin_r + cdy * cos_r)

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
                          clearance_margin: float = 0.05) -> Tuple[bool, float]:
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
    expanded_layers = _expand_cu(pad.layers, routing_layers)

    # Vias are through-hole, so they conflict with pads on any copper layer
    if not any(layer.endswith('.Cu') for layer in expanded_layers):
        return False, 0.0

    # Distance from via center to pad edge. Handles custom comb/finger polygons
    # (issue #188), rounded corners, and diagonal (rect_rotation) pads.
    dist_to_pad = point_to_pad_distance(via.x, via.y, pad)

    # Required clearance: via half-size + clearance
    # (dist_to_pad is already edge-to-edge from pad)
    required_dist = via.size / 2 + clearance
    overlap = required_dist - dist_to_pad

    tolerance = clearance * clearance_margin
    if overlap > tolerance:
        return True, overlap

    return False, 0.0


def check_via_drill_overlap(via1: Via, via2: Via, hole_to_hole_clearance: float,
                            clearance_margin: float = 0.05) -> Tuple[bool, float]:
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
                                clearance_margin: float = 0.05) -> Tuple[bool, float]:
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
                             clearance: float, clearance_margin: float = 0.05) -> Tuple[bool, float, str]:
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
                         clearance: float, clearance_margin: float = 0.05) -> Tuple[bool, float, str]:
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


# --- Real-outline board-edge geometry (issue #236) ---------------------------
# The bbox checks above measure to the rectangular board extent. On a board with
# an internal cutout, slot, or notch, copper routed INTO the cutout sits inside
# the bbox and is never flagged. These helpers measure to the actual Edge.Cuts
# outline (outer ring + interior cutouts), matching KiCad's copper_edge_clearance.

def board_edge_geometry(board_info) -> Tuple[List[List[Tuple[float, float]]],
                                             Optional[List[Tuple[float, float]]],
                                             List[List[Tuple[float, float]]]]:
    """Return (edge_rings, outer_outline, cutouts) for the real Edge.Cuts.

    edge_rings is the flat list of closed vertex rings (outer outline + each
    cutout) to measure clearance against; outer_outline / cutouts are returned
    separately for the on-board (inside-outline, outside-cutouts) test. Any ring
    is returned only if it has >=3 vertices; an empty edge_rings means the parser
    found no usable outline and the caller should fall back to the bbox checks.
    """
    outline = getattr(board_info, 'board_outline', None) or []
    cutouts = [c for c in (getattr(board_info, 'board_cutouts', None) or []) if len(c) >= 3]
    outer = outline if len(outline) >= 3 else None
    rings = []
    if outer:
        rings.append(outer)
    rings.extend(cutouts)
    return rings, outer, cutouts


def _point_to_rings_distance(x: float, y: float,
                             rings: List[List[Tuple[float, float]]]) -> float:
    """Min distance from a point to any edge ring's boundary."""
    best = float('inf')
    for ring in rings:
        n = len(ring)
        for i in range(n):
            x1, y1 = ring[i]
            x2, y2 = ring[(i + 1) % n]
            d = point_to_segment_distance(x, y, x1, y1, x2, y2)
            if d < best:
                best = d
    return best


def _segment_to_rings_distance(x1: float, y1: float, x2: float, y2: float,
                               rings: List[List[Tuple[float, float]]]) -> float:
    """Min distance from a track segment to any edge ring's boundary (0 if it
    crosses an edge)."""
    best = float('inf')
    for ring in rings:
        n = len(ring)
        for i in range(n):
            ex1, ey1 = ring[i]
            ex2, ey2 = ring[(i + 1) % n]
            d = _seg_seg_dist_coords(x1, y1, x2, y2, ex1, ey1, ex2, ey2)
            if d < best:
                best = d
    return best


def _point_on_board(x: float, y: float, outer: Optional[List[Tuple[float, float]]],
                    cutouts: List[List[Tuple[float, float]]]) -> bool:
    """True if (x, y) is on copper-bearing board: inside the outer outline and
    not inside any cutout. A point off-board / inside a cutout is a hard edge
    violation regardless of its distance to the nearest edge segment."""
    if outer is not None and not _point_in_poly(x, y, outer):
        return False
    for cut in cutouts:
        if _point_in_poly(x, y, cut):
            return False
    return True


def check_segment_board_edge_poly(seg: Segment, rings, outer, cutouts,
                                   clearance: float,
                                   clearance_margin: float = 0.05
                                   ) -> Tuple[bool, float, str]:
    """Board-edge clearance for a track measured against the real Edge.Cuts."""
    required = clearance + seg.width / 2
    tolerance = clearance * clearance_margin
    # A track endpoint off-board / inside a cutout is a definite violation.
    for x, y in [(seg.start_x, seg.start_y), (seg.end_x, seg.end_y)]:
        if not _point_on_board(x, y, outer, cutouts):
            dist = _point_to_rings_distance(x, y, rings)
            return True, required + dist, "off-board"
    dist = _segment_to_rings_distance(seg.start_x, seg.start_y, seg.end_x, seg.end_y, rings)
    overlap = required - dist
    if overlap > tolerance:
        return True, overlap, "edge"
    return False, 0.0, ""


def check_via_board_edge_poly(via: Via, rings, outer, cutouts,
                              clearance: float,
                              clearance_margin: float = 0.05
                              ) -> Tuple[bool, float, str]:
    """Board-edge clearance for a via measured against the real Edge.Cuts."""
    required = clearance + via.size / 2
    tolerance = clearance * clearance_margin
    if not _point_on_board(via.x, via.y, outer, cutouts):
        dist = _point_to_rings_distance(via.x, via.y, rings)
        return True, required + dist, "off-board"
    dist = _point_to_rings_distance(via.x, via.y, rings)
    overlap = required - dist
    if overlap > tolerance:
        return True, overlap, "edge"
    return False, 0.0, ""


def check_pad_board_edge(pad: Pad, rings, outer, cutouts,
                         clearance: float, board_bounds,
                         clearance_margin: float = 0.05
                         ) -> Tuple[bool, float, str]:
    """Board-edge clearance for a pad (issue #236). Measures the pad copper edge
    (sampled perimeter, or the bbox-rectangle distance when no outline exists)
    to the real Edge.Cuts. Returns (has_violation, overlap_mm, edge)."""
    tolerance = clearance * clearance_margin
    if rings:
        best = float('inf')
        off_board = False
        for px, py in _pad_perimeter_points(pad):
            if not _point_on_board(px, py, outer, cutouts):
                off_board = True
            d = _point_to_rings_distance(px, py, rings)
            if d < best:
                best = d
        if off_board:
            return True, clearance + best, "off-board"
        overlap = clearance - best
        if overlap > tolerance:
            return True, overlap, "edge"
        return False, 0.0, ""

    # No usable outline -> bbox fallback, measured to the pad copper edge.
    if board_bounds is None:
        return False, 0.0, ""
    min_x, min_y, max_x, max_y = board_bounds
    best = float('inf')
    edge = ""
    for px, py in _pad_perimeter_points(pad):
        for d, name in ((px - min_x, "left"), (max_x - px, "right"),
                        (py - min_y, "bottom"), (max_y - py, "top")):
            if d < best:
                best = d
                edge = name
    overlap = clearance - best
    if overlap > tolerance:
        return True, overlap, edge
    return False, 0.0, ""


def check_track_width(seg: Segment, min_track_width: float,
                      size_margin: float = 0.0) -> Tuple[bool, float]:
    """Check if a segment is thinner than the minimum manufacturable track width.

    Args:
        seg: Segment to check
        min_track_width: Minimum allowed track width in mm
        size_margin: Absolute tolerance in mm; widths within this of the floor pass

    Returns:
        (is_too_thin, shortfall_mm)
    """
    shortfall = min_track_width - seg.width
    if shortfall > size_margin:
        return True, shortfall
    return False, 0.0


def check_via_size(via: Via, min_via_diameter: float, min_via_drill: float,
                   size_margin: float = 0.0) -> Tuple[bool, bool, float, float]:
    """Check if a via's outer diameter or drill is below the fab floor.

    Returns:
        (diameter_too_small, drill_too_small, dia_shortfall_mm, drill_shortfall_mm)
    """
    dia_short = min_via_diameter - via.size
    drill_short = min_via_drill - via.drill
    dia_bad = dia_short > size_margin
    drill_bad = drill_short > size_margin
    return dia_bad, drill_bad, dia_short, drill_short


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


def _edge_phrase(edge: str) -> str:
    """Human-readable board-edge violation phrase from the edge label."""
    if edge == 'off-board':
        return "off the board / in a cutout"
    if edge in ('', 'edge'):
        return "too close to board edge"
    return f"too close to {edge} board edge"  # bbox fallback: left/right/top/bottom


def run_drc(pcb_file: str, clearance: float = 0.1, net_patterns: Optional[List[str]] = None,
            debug_output: bool = False, quiet: bool = False,
            hole_to_hole_clearance: float = defaults.HOLE_TO_HOLE_CLEARANCE, board_edge_clearance: float = 0.0,
            clearance_margin: float = 0.05, max_print: int = 20,
            min_track_width: Optional[float] = None,
            min_via_diameter: Optional[float] = None,
            min_via_drill: Optional[float] = None,
            check_sizes: bool = True, size_margin: float = 0.0,
            check_pad_edge: bool = False):
    """Run DRC checks on the PCB file.

    Args:
        pcb_file: Path to the KiCad PCB file
        clearance: Minimum clearance in mm
        net_patterns: Optional list of net name patterns (fnmatch style) to focus on.
                     If provided, only checks involving at least one matching net are reported.
        debug_output: If True, write debug lines to User.7 layer showing violation locations
        quiet: If True, only print a summary line unless there are violations
        hole_to_hole_clearance: Minimum clearance between drill hole edges in mm
            (default: the fab floor, routing_defaults.HOLE_TO_HOLE_CLEARANCE)
        board_edge_clearance: Minimum clearance from board edge in mm (0 = use clearance)
        min_track_width: Minimum manufacturable track width in mm. None = derive the
            JLC fab floor from the board's copper-layer count (issue #176).
        min_via_diameter: Minimum via outer diameter in mm. None = fab floor.
        min_via_drill: Minimum via drill in mm. None = fab floor.
        check_sizes: If True, flag tracks/vias below the fab floor (track-width and
            via/hole size checks). These catch sub-fab copper that the clearance-only
            checks miss (a board's own min_track_width DRC rule can be lowered to match
            undersized copper, so it never trips -- the fab floor is the real limit).
        size_margin: Absolute tolerance in mm for the size checks; a width/diameter
            within this of the floor is not flagged (default 0 = exact floor).
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

    # Resolve the fab-floor minimums for the track-width / via-size checks. Default
    # to the JLC manufacturing floor for the board's copper-layer count (issue #176).
    if check_sizes:
        from list_nets import fab_floors
        copper_layers = getattr(pcb_data.board_info, 'copper_layers', None) or []
        copper_count = len(copper_layers) if copper_layers else 2
        fab = fab_floors(copper_count)
        # Use the FINE via floor (the smallest via the fab can make) as the hard
        # minimum -- 0.30mm dia / 0.15mm drill on 4+ layers. Flagging at the
        # larger standard via floor would reject legitimate fine-pitch vias.
        eff_min_track = min_track_width if min_track_width is not None else fab['track_width']
        eff_min_via_dia = min_via_diameter if min_via_diameter is not None else fab['fine_via_diameter']
        eff_min_via_drill = min_via_drill if min_via_drill is not None else fab['fine_via_drill']
        if not quiet:
            print(f"Size floors ({copper_count}-layer fab): track >= {eff_min_track}mm, "
                  f"via dia >= {eff_min_via_dia}mm, via drill >= {eff_min_via_drill}mm")

    # Helper to check if a net_id matches the filter patterns
    def net_matches_filter(net_id: int) -> bool:
        if net_patterns is None:
            return True  # No filter, include all
        net_info = pcb_data.nets.get(net_id, None)
        if net_info is None:
            return False
        return matches_any_pattern(net_info.name, net_patterns)

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
            expanded_layers = _expand_cu(pad.layers, routing_layers)
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

            has_violation, overlap, pt1, pt2 = check_segment_overlap(seg1, seg2, clearance, clearance_margin)
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

                has_violation, overlap = check_via_segment_overlap(via, seg, clearance, clearance_margin)
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

            has_violation, overlap = check_via_via_overlap(via1, via2, clearance, clearance_margin)
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
                    pad, seg, clearance, routing_layers, clearance_margin
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
                    pad, via, clearance, routing_layers, clearance_margin
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

    # Check pad-to-pad violations using spatial index (issue #234). Two pads of
    # DIFFERENT nets that overlap (a short) or sit below clearance on a shared
    # copper layer -- e.g. a placement step nudging a cap onto an IC pad. KiCad
    # flags these (shorting_items / clearance); check_drc previously had only the
    # pad-segment and pad-via passes.
    if not quiet:
        print("Checking pad-to-pad clearances...")
    pad_pad_checked = set()
    for pad_net, pads in pads_by_net.items():
        net1_matches = matching_net_ids is None or pad_net in matching_net_ids
        for pad1 in pads:
            for layer in _expand_cu(pad1.layers, routing_layers):
                if not layer.endswith('.Cu'):
                    continue
                for pad2, pad2_net in spatial_idx.get_nearby_pads_for_pad(pad1, layer):
                    if pad2 is pad1:
                        continue
                    if pad2_net == pad_net:
                        continue  # Same net -- allowed to touch
                    # Skip pads of the SAME footprint: a component's own adjacent
                    # pins are fixed library geometry (a fine-pitch part can have
                    # pad gaps below the routing clearance), never something a
                    # placement/routing step introduces or can fix. Flagging them
                    # would flood grading with pre-existing noise. The cases #234
                    # targets are between DIFFERENT footprints (e.g. a cap nudged
                    # onto an IC pad).
                    if pad2.component_ref == pad1.component_ref:
                        continue
                    net2_matches = matching_net_ids is None or pad2_net in matching_net_ids
                    if not net1_matches and not net2_matches:
                        continue
                    pair_key = (min(id(pad1), id(pad2)), max(id(pad1), id(pad2)))
                    if pair_key in pad_pad_checked:
                        continue
                    pad_pad_checked.add(pair_key)
                    has_violation, overlap, closest_pt = check_pad_pad_overlap(
                        pad1, pad2, clearance, routing_layers, clearance_margin)
                    if has_violation:
                        n1 = pcb_data.nets.get(pad_net, None)
                        n2 = pcb_data.nets.get(pad2_net, None)
                        n1s = n1.name if n1 else f"net_{pad_net}"
                        n2s = n2.name if n2 else f"net_{pad2_net}"
                        violations.append({
                            'type': 'pad-pad',
                            'net1': n1s,
                            'net2': n2s,
                            'layer': layer,
                            'overlap_mm': overlap,
                            'pad_ref': f"{pad1.component_ref}.{pad1.pad_number}",
                            'pad_ref2': f"{pad2.component_ref}.{pad2.pad_number}",
                            'pad_loc': (pad1.global_x, pad1.global_y),
                            'pad_loc2': (pad2.global_x, pad2.global_y),
                            'closest_pt': closest_pt,
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
        # Via drill hole-to-hole: vectorized over all via pairs. For each via i we
        # compute the center distance to every via j>i at once; the violation test
        # (overlap = drill_i/2 + drill_j/2 + clearance - dist > tolerance) and the
        # reported overlap mirror check_via_drill_overlap exactly. The default
        # clearance_margin there is 0.05, so tolerance = clearance * 0.05.
        all_vias = list(pcb_data.vias)
        nv = len(all_vias)
        if nv >= 2:
            vx = np.array([v.x for v in all_vias], dtype=np.float64)
            vy = np.array([v.y for v in all_vias], dtype=np.float64)
            vdrill = np.array([v.drill for v in all_vias], dtype=np.float64)
            if matching_via_nets is None:
                vmatch = None
            else:
                vmatch = np.array([v.net_id in matching_via_nets for v in all_vias], dtype=bool)
            tolerance = hole_to_hole_clearance * 0.05
            for i in range(nv - 1):
                required = vdrill[i] / 2 + vdrill[i + 1:] / 2 + hole_to_hole_clearance
                actual = np.sqrt((vx[i] - vx[i + 1:]) ** 2 + (vy[i] - vy[i + 1:]) ** 2)
                overlap = required - actual
                viol = overlap > tolerance
                if vmatch is not None:
                    # Skip pairs where neither net matches the filter.
                    viol &= vmatch[i] | vmatch[i + 1:]
                for k in np.nonzero(viol)[0].tolist():
                    via1 = all_vias[i]
                    via2 = all_vias[i + 1 + k]
                    net1_name = pcb_data.nets.get(via1.net_id, None)
                    net2_name = pcb_data.nets.get(via2.net_id, None)
                    net1_str = net1_name.name if net1_name else f"net_{via1.net_id}"
                    net2_str = net2_name.name if net2_name else f"net_{via2.net_id}"
                    violations.append({
                        'type': 'via-drill-hole',
                        'net1': net1_str,
                        'net2': net2_str,
                        'overlap_mm': float(overlap[k]),
                        'loc1': (via1.x, via1.y),
                        'loc2': (via2.x, via2.y),
                    })

        # Check via drill to pad drill (through-hole pads)
        # NOTE: Hole-to-hole clearance applies regardless of net (manufacturing constraint)
        if not quiet:
            print("Checking via drill to pad drill clearances...")
        # Flatten through-hole pads (SMD pads have no drill) keeping their net key,
        # then vectorize each via against all of them. Iteration order (via outer,
        # pad inner in pads_by_net order) and the overlap formula match the scalar
        # check_pad_drill_via_overlap so the violation list is identical.
        th_pads = []
        th_pad_nets = []
        for pad_net, pads in pads_by_net.items():
            for pad in pads:
                if pad.drill <= 0:
                    continue  # SMD pad
                th_pads.append(pad)
                th_pad_nets.append(pad_net)
        if th_pads and all_vias:
            px = np.array([p.global_x for p in th_pads], dtype=np.float64)
            py = np.array([p.global_y for p in th_pads], dtype=np.float64)
            pdrill = np.array([p.drill for p in th_pads], dtype=np.float64)
            if matching_pad_nets is None:
                pmatch = None
            else:
                pmatch = np.array([pn in matching_pad_nets for pn in th_pad_nets], dtype=bool)
            tolerance = hole_to_hole_clearance * 0.05
            for via in all_vias:
                via_matches = matching_via_nets is None or via.net_id in matching_via_nets
                required = pdrill / 2 + via.drill / 2 + hole_to_hole_clearance
                actual = np.sqrt((px - via.x) ** 2 + (py - via.y) ** 2)
                overlap = required - actual
                viol = overlap > tolerance
                # Skip (via, pad) where neither net matches the filter. When the via
                # matches, every pad is checked; otherwise only filter-matching pads.
                if not via_matches and pmatch is not None:
                    viol &= pmatch
                for k in np.nonzero(viol)[0].tolist():
                    pad = th_pads[k]
                    pad_net = th_pad_nets[k]
                    via_net_name = pcb_data.nets.get(via.net_id, None)
                    pad_net_name = pcb_data.nets.get(pad_net, None)
                    via_net_str = via_net_name.name if via_net_name else f"net_{via.net_id}"
                    pad_net_str = pad_net_name.name if pad_net_name else f"net_{pad_net}"
                    same_net = pad_net == via.net_id
                    violations.append({
                        'type': 'pad-drill-via-drill-same-net' if same_net else 'pad-drill-via-drill',
                        'net1': pad_net_str,
                        'net2': via_net_str,
                        'overlap_mm': float(overlap[k]),
                        'pad_loc': (pad.global_x, pad.global_y),
                        'pad_ref': f"{pad.component_ref}.{pad.pad_number}",
                        'via_loc': (via.x, via.y),
                    })

    # Check copper-to-hole clearance: a TRACK too close to an NPTH (no-copper) drill
    # hole of a DIFFERENT net (issue #233). The drill removes the copper that crosses
    # it -- a real fab short the via-drill-only hole check above misses (e.g. a track
    # routed straight across a connector mounting hole). PTH pads and vias carry
    # copper, so their track clearance is already covered by the pad-segment /
    # via-segment checks (with the real pad shape, not a round-drill approximation),
    # and KiCad likewise reports only the NPTH cases. Mirrors KiCad's hole_clearance.
    if not quiet:
        print("Checking copper-to-hole (track <-> NPTH drill) clearances...")
    holes = []  # (x, y, drill, net_id, ref) -- NPTH (no-copper) pad holes only
    for pad_net, pads in pads_by_net.items():
        for pad in pads:
            if pad.drill > 0 and not any(l == '*.Cu' or l.endswith('.Cu') for l in pad.layers):
                holes.append((pad.global_x, pad.global_y, pad.drill, pad_net,
                              f"{pad.component_ref}.{pad.pad_number}"))
    segs = list(pcb_data.segments)
    if holes and segs:
        sx1 = np.array([s.start_x for s in segs], dtype=np.float64)
        sy1 = np.array([s.start_y for s in segs], dtype=np.float64)
        sx2 = np.array([s.end_x for s in segs], dtype=np.float64)
        sy2 = np.array([s.end_y for s in segs], dtype=np.float64)
        sw = np.array([s.width for s in segs], dtype=np.float64)
        snet = np.array([s.net_id for s in segs])
        dx = sx2 - sx1
        dy = sy2 - sy1
        seglen2 = dx * dx + dy * dy
        safe_len2 = np.where(seglen2 > 0, seglen2, 1.0)
        # JLC "NPTH to Track" fab floor (never below the graded clearance).
        npth_clr = max(clearance, defaults.NPTH_TO_TRACK_CLEARANCE)
        tolerance = npth_clr * clearance_margin
        if matching_net_ids is not None:
            seg_match = np.array([n in matching_net_ids for n in snet], dtype=bool)
        for hx, hy, drill, hnet, ref in holes:
            # Point(hole)-to-segment distance, vectorized over all tracks. A drill
            # is a through-hole, so any track layer crossing it conflicts (no layer
            # filter). The hole's own-net track legitimately connects to it -> skip.
            t = np.clip(((hx - sx1) * dx + (hy - sy1) * dy) / safe_len2, 0.0, 1.0)
            cxp = sx1 + t * dx
            cyp = sy1 + t * dy
            dist = np.sqrt((hx - cxp) ** 2 + (hy - cyp) ** 2)
            overlap = (drill / 2.0 + sw / 2.0 + npth_clr) - dist
            viol = (overlap > tolerance) & (snet != hnet)
            if matching_net_ids is not None:
                viol &= seg_match | (hnet in matching_net_ids)
            for k in np.nonzero(viol)[0].tolist():
                seg = segs[k]
                hole_net_name = pcb_data.nets.get(hnet, None)
                seg_net_name = pcb_data.nets.get(seg.net_id, None)
                hole_net_str = hole_net_name.name if hole_net_name else f"net_{hnet}"
                seg_net_str = seg_net_name.name if seg_net_name else f"net_{seg.net_id}"
                violations.append({
                    'type': 'track-hole',
                    'net1': hole_net_str,        # the drill's net (0 = NPTH)
                    'net2': seg_net_str,         # the crossing track's net
                    'hole_ref': ref or 'via',
                    'layer': seg.layer,
                    'overlap_mm': float(overlap[k]),
                    'hole_loc': (hx, hy),
                    'seg_loc': (seg.start_x, seg.start_y, seg.end_x, seg.end_y),
                })

    # Check board edge clearances. Measure to the real Edge.Cuts outline (outer
    # ring + interior cutouts) when the parser found one, so copper routed into a
    # cutout/slot/notch -- which sits inside the bounding box -- is caught (issue
    # #236). Fall back to the rectangular bounding box otherwise.
    board_bounds = pcb_data.board_info.board_bounds
    if board_bounds and effective_board_edge_clearance > 0:
        edge_rings, edge_outer, edge_cutouts = board_edge_geometry(pcb_data.board_info)
        use_poly = bool(edge_rings)
        if not quiet:
            print("Checking board edge clearances "
                  f"({'real Edge.Cuts outline' if use_poly else 'bounding box'})...")

        # Check segments
        for seg in pcb_data.segments:
            seg_matches = matching_seg_net_set is None or seg.net_id in matching_seg_net_set
            if matching_seg_net_set is not None and not seg_matches:
                continue
            if use_poly:
                has_violation, overlap, edge = check_segment_board_edge_poly(
                    seg, edge_rings, edge_outer, edge_cutouts,
                    effective_board_edge_clearance, clearance_margin)
            else:
                has_violation, overlap, edge = check_segment_board_edge(
                    seg, board_bounds, effective_board_edge_clearance, clearance_margin)
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
            if use_poly:
                has_violation, overlap, edge = check_via_board_edge_poly(
                    via, edge_rings, edge_outer, edge_cutouts,
                    effective_board_edge_clearance, clearance_margin)
            else:
                has_violation, overlap, edge = check_via_board_edge(
                    via, board_bounds, effective_board_edge_clearance, clearance_margin)
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

        # Check pads (issue #236). Off by default: pad-to-edge violations are
        # almost always pre-existing edge-connector pads on the bare board (the
        # router never places pads), so flagging them by default just adds noise
        # to routed-board grading. Enable with --check-pad-edge to catch a
        # placement step that pushed a component off the board / into a cutout.
        if check_pad_edge:
            for pad_net, pads in pads_by_net.items():
                if matching_pad_nets is not None and pad_net not in matching_pad_nets:
                    continue
                for pad in pads:
                    if not any(l == '*.Cu' or l.endswith('.Cu') for l in pad.layers):
                        continue  # No copper on this pad (pure NPTH)
                    has_violation, overlap, edge = check_pad_board_edge(
                        pad, edge_rings, edge_outer, edge_cutouts,
                        effective_board_edge_clearance, board_bounds, clearance_margin)
                    if has_violation:
                        net_name = pcb_data.nets.get(pad_net, None)
                        net_str = net_name.name if net_name else f"net_{pad_net}"
                        violations.append({
                            'type': 'pad-board-edge',
                            'net1': net_str,
                            'edge': edge,
                            'overlap_mm': overlap,
                            'pad_ref': f"{pad.component_ref}.{pad.pad_number}",
                            'pad_loc': (pad.global_x, pad.global_y),
                        })

    # Check track widths and via/hole sizes against the fab floor (issue #176).
    # Unlike the clearance checks these are per-object (one net), so the net filter
    # applies to that single net.
    if check_sizes:
        if not quiet:
            print("Checking track widths and via/hole sizes...")
        for seg in pcb_data.segments:
            if matching_net_ids is not None and seg.net_id not in matching_net_ids:
                continue
            too_thin, shortfall = check_track_width(seg, eff_min_track, size_margin)
            if too_thin:
                net_name = pcb_data.nets.get(seg.net_id, None)
                net_str = net_name.name if net_name else f"net_{seg.net_id}"
                violations.append({
                    'type': 'track-width',
                    'net1': net_str,
                    'layer': seg.layer,
                    'width': seg.width,
                    'min_width': eff_min_track,
                    'shortfall_mm': shortfall,
                    'seg_loc': (seg.start_x, seg.start_y, seg.end_x, seg.end_y),
                })
        for via in pcb_data.vias:
            if matching_net_ids is not None and via.net_id not in matching_net_ids:
                continue
            dia_bad, drill_bad, dia_short, drill_short = check_via_size(
                via, eff_min_via_dia, eff_min_via_drill, size_margin)
            if dia_bad or drill_bad:
                net_name = pcb_data.nets.get(via.net_id, None)
                net_str = net_name.name if net_name else f"net_{via.net_id}"
                if dia_bad:
                    violations.append({
                        'type': 'via-size',
                        'net1': net_str,
                        'size': via.size,
                        'min_size': eff_min_via_dia,
                        'shortfall_mm': dia_short,
                        'via_loc': (via.x, via.y),
                    })
                if drill_bad:
                    violations.append({
                        'type': 'via-drill-size',
                        'net1': net_str,
                        'drill': via.drill,
                        'min_drill': eff_min_via_drill,
                        'shortfall_mm': drill_short,
                        'via_loc': (via.x, via.y),
                    })

    # Same-net COPPER overlaps are not DRC failures: same-net copper is allowed
    # to overlap (KiCad's own DRC permits it -- it only enforces clearance between
    # DIFFERENT nets). This covers both same-net segment crossings AND same-net
    # via-to-via copper clearance (two vias on the same net may touch -- e.g. a
    # plane stitching via next to a tap via). They are at most cosmetic clutter,
    # so report them as warnings and keep them out of the violation count / exit
    # status. (Same-net DRILL hole-to-hole checks -- 'via-drill-hole',
    # 'pad-drill-via-drill-same-net' -- stay violations: drill spacing is a real
    # fab constraint independent of net.)
    _samenet_copper = ('segment-crossing-same-net', 'via-via-same-net')
    seg_warns = [v for v in violations if v['type'] == 'segment-crossing-same-net']
    viavia_warns = [v for v in violations if v['type'] == 'via-via-same-net']
    warnings = [v for v in violations if v['type'] in _samenet_copper]
    violations = [v for v in violations if v['type'] not in _samenet_copper]

    def _warn_note():
        if warnings:
            print(f"\nWARNINGS ({len(warnings)}, not DRC failures):")
            if seg_warns:
                print(f"  same-net self-crossing: {len(seg_warns)} (same-net copper overlap; "
                      f"permitted by KiCad DRC)")
            if viavia_warns:
                print(f"  same-net via-via: {len(viavia_warns)} (same-net copper overlap; "
                      f"permitted by KiCad DRC -- drill hole-to-hole checked separately)")

    # Report violations
    if quiet:
        if violations:
            print(f"FAILED ({len(violations)} violations)")
        else:
            print("OK" + (f" ({len(warnings)} same-net copper warning(s))" if warnings else ""))
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

            # Per-type print cap. max_print <= 0 means print every violation
            # (issue #93: a fixed cap silently dropped most of a long list).
            limit = len(violations) if max_print is not None and max_print <= 0 else max_print
            for vtype, vlist in by_type.items():
                print(f"\n{vtype.upper()} violations ({len(vlist)}):")
                print("-" * 40)
                for v in vlist[:limit]:  # Show first `limit` of each type
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
                    elif vtype == 'pad-pad':
                        short = " [SHORT]" if v['overlap_mm'] >= clearance else ""
                        print(f"  Pad:{v['net1']} ({v['pad_ref']}) <-> Pad:{v['net2']} ({v['pad_ref2']}){short}")
                        print(f"    Layer: {v['layer']}, Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Pad1: ({v['pad_loc'][0]:.2f},{v['pad_loc'][1]:.2f})")
                        print(f"    Pad2: ({v['pad_loc2'][0]:.2f},{v['pad_loc2'][1]:.2f})")
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
                    elif vtype == 'track-hole':
                        print(f"  Hole:{v['net1']} ({v['hole_ref']}) <-> Track:{v['net2']} (copper-to-hole)")
                        print(f"    Layer: {v['layer']}, Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Hole: ({v['hole_loc'][0]:.2f},{v['hole_loc'][1]:.2f})")
                        print(f"    Seg: ({v['seg_loc'][0]:.2f},{v['seg_loc'][1]:.2f})-({v['seg_loc'][2]:.2f},{v['seg_loc'][3]:.2f})")
                    elif vtype == 'segment-board-edge':
                        where = _edge_phrase(v['edge'])
                        print(f"  {v['net1']} {where}")
                        print(f"    Layer: {v['layer']}, Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Seg: ({v['seg_loc'][0]:.2f},{v['seg_loc'][1]:.2f})-({v['seg_loc'][2]:.2f},{v['seg_loc'][3]:.2f})")
                    elif vtype == 'via-board-edge':
                        where = _edge_phrase(v['edge'])
                        print(f"  Via:{v['net1']} {where}")
                        print(f"    Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Via: ({v['via_loc'][0]:.2f},{v['via_loc'][1]:.2f})")
                    elif vtype == 'pad-board-edge':
                        where = _edge_phrase(v['edge'])
                        print(f"  Pad:{v['net1']} ({v['pad_ref']}) {where}")
                        print(f"    Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Pad: ({v['pad_loc'][0]:.2f},{v['pad_loc'][1]:.2f})")
                    elif vtype == 'track-width':
                        print(f"  {v['net1']} track too thin for fab")
                        print(f"    Layer: {v['layer']}, Width: {v['width']:.4f}mm "
                              f"< min {v['min_width']:.4f}mm (short {v['shortfall_mm']:.4f}mm)")
                        print(f"    Seg: ({v['seg_loc'][0]:.2f},{v['seg_loc'][1]:.2f})-({v['seg_loc'][2]:.2f},{v['seg_loc'][3]:.2f})")
                    elif vtype == 'via-size':
                        print(f"  Via:{v['net1']} diameter too small for fab")
                        print(f"    Size: {v['size']:.4f}mm < min {v['min_size']:.4f}mm "
                              f"(short {v['shortfall_mm']:.4f}mm)")
                        print(f"    Via: ({v['via_loc'][0]:.2f},{v['via_loc'][1]:.2f})")
                    elif vtype == 'via-drill-size':
                        print(f"  Via:{v['net1']} drill hole too small for fab")
                        print(f"    Drill: {v['drill']:.4f}mm < min {v['min_drill']:.4f}mm "
                              f"(short {v['shortfall_mm']:.4f}mm)")
                        print(f"    Via: ({v['via_loc'][0]:.2f},{v['via_loc'][1]:.2f})")

                if len(vlist) > limit:
                    print(f"  ... and {len(vlist) - limit} more "
                          f"(use --max-print 0 to show all)")
        else:
            print("NO DRC VIOLATIONS FOUND!")

        _warn_note()
        print("=" * 60)

    # Write debug lines if requested
    if debug_output and violations:
        write_debug_lines(pcb_file, violations, clearance)

    return violations


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Check PCB for DRC violations (clearance errors)')
    parser.add_argument('pcb', help='Input PCB file')
    parser.add_argument('--clearance', '-c', type=float, default=None,
                        help='Minimum clearance in mm to grade against. If omitted, '
                             'auto-detected from the sibling .kicad_pro Default net '
                             'class (the value the board was routed/graded to); falls '
                             'back to 0.2 if no project clearance is found.')
    parser.add_argument('--hole-to-hole-clearance', type=float, default=defaults.HOLE_TO_HOLE_CLEARANCE,
                        help=f'Minimum drill hole edge-to-edge clearance in mm '
                             f'(default: {defaults.HOLE_TO_HOLE_CLEARANCE}, the fab floor — same as routing)')
    parser.add_argument('--board-edge-clearance', type=float, default=0.0,
                        help='Minimum clearance from board edge in mm (0 = use --clearance value)')
    parser.add_argument('--clearance-margin', type=float, default=0.05,
                        help='Fraction of clearance to use as tolerance (default: 0.05 = 5%%). Violations smaller than clearance*margin are ignored.')
    parser.add_argument('--nets', '-n', nargs='+', default=None,
                        help='Optional net name patterns to focus on (fnmatch wildcards supported, e.g., "*lvds*")')
    parser.add_argument('--debug-lines', '-d', action='store_true',
                        help='Write debug lines to User.7 layer showing violation locations')
    parser.add_argument('--quiet', '-q', action='store_true',
                        help='Only print a summary line unless there are violations')
    parser.add_argument('--max-print', type=int, default=20,
                        help='Max violations to list per type before "... and N more" '
                             '(0 = print all). Default 20.')
    parser.add_argument('--no-size-checks', action='store_true',
                        help='Skip the track-width and via/hole-size fab-floor checks')
    parser.add_argument('--min-track-width', type=float, default=None,
                        help='Minimum manufacturable track width in mm '
                             '(default: JLC fab floor for the board layer count)')
    parser.add_argument('--min-via-diameter', type=float, default=None,
                        help='Minimum via outer diameter in mm (default: fab floor)')
    parser.add_argument('--min-via-drill', type=float, default=None,
                        help='Minimum via drill diameter in mm (default: fab floor)')
    parser.add_argument('--size-margin', type=float, default=0.0,
                        help='Absolute tolerance in mm for the size checks (default: 0)')
    parser.add_argument('--check-pad-edge', action='store_true',
                        help='Also check pad-to-board-edge clearance (issue #236). '
                             'Off by default: pad-edge violations are almost always '
                             'pre-existing edge-connector pads, not router-introduced.')

    args = parser.parse_args()

    # Grade at the clearance the board was actually routed to. When -c is not
    # given, read the sibling .kicad_pro Default net-class clearance -- which the
    # routers lower to the smallest clearance used in ANY step (incl. fine-pitch
    # tap escalation). Grading stricter than that invents phantom violations on
    # legitimately tight copper; grading looser hides real ones. (issue follow-up
    # to the route_disconnected_planes fine-tap grading confusion.)
    if args.clearance is None:
        args.clearance = 0.2
        try:
            import os, json
            from fix_kicad_drc_settings import find_project, project_copper_clearance
            pro = find_project(args.pcb)
            if os.path.isfile(pro):
                with open(pro) as f:
                    pc = project_copper_clearance(json.load(f))
                if pc:
                    args.clearance = pc
                    if not args.quiet:
                        print(f"Grading at clearance {pc:.4g} mm "
                              f"(from {os.path.basename(pro)} Default net class)")
        except Exception as e:
            print(f"  (could not read project clearance, using 0.2 mm: {e})")
        if args.clearance == 0.2 and not args.quiet:
            print("Grading at clearance 0.2 mm (no project clearance found; "
                  "pass -c to override)")

    violations = run_drc(args.pcb, args.clearance, args.nets, args.debug_lines, args.quiet,
                         args.hole_to_hole_clearance, args.board_edge_clearance,
                         args.clearance_margin, max_print=args.max_print,
                         min_track_width=args.min_track_width,
                         min_via_diameter=args.min_via_diameter,
                         min_via_drill=args.min_via_drill,
                         check_sizes=not args.no_size_checks,
                         size_margin=args.size_margin,
                         check_pad_edge=args.check_pad_edge)
    sys.exit(1 if violations else 0)

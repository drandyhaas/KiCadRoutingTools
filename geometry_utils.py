"""
Shared geometry utility functions for PCB routing.

This module consolidates geometry calculations used across multiple modules:
- Point-to-segment distance
- Segment-to-segment distance
- Segment intersection detection
- Closest point calculations
- Union-Find data structure for connectivity
"""

import math
from typing import Tuple, Optional, Dict, Any, TYPE_CHECKING


class UnionFind:
    """
    Union-Find (Disjoint Set Union) data structure with path compression and rank optimization.

    Used for efficiently tracking connected components in graphs.
    Supports arbitrary hashable keys.

    Example:
        uf = UnionFind()
        uf.union('a', 'b')
        uf.union('b', 'c')
        assert uf.find('a') == uf.find('c')  # a and c are connected
    """

    def __init__(self):
        self.parent: Dict[Any, Any] = {}
        self.rank: Dict[Any, int] = {}

    def find(self, x: Any) -> Any:
        """
        Find the root representative of the set containing x.

        Uses path compression for O(α(n)) amortized time complexity.
        Creates a new singleton set if x is not yet tracked.
        """
        if x not in self.parent:
            self.parent[x] = x
            self.rank[x] = 0
        if self.parent[x] != x:
            self.parent[x] = self.find(self.parent[x])  # Path compression
        return self.parent[x]

    def union(self, x: Any, y: Any) -> None:
        """
        Merge the sets containing x and y.

        Uses union by rank for O(α(n)) amortized time complexity.
        """
        px, py = self.find(x), self.find(y)
        if px == py:
            return  # Already in same set
        # Union by rank - attach smaller tree under larger tree
        if self.rank[px] < self.rank[py]:
            px, py = py, px
        self.parent[py] = px
        if self.rank[px] == self.rank[py]:
            self.rank[px] += 1

    def connected(self, x: Any, y: Any) -> bool:
        """Check if x and y are in the same set."""
        return self.find(x) == self.find(y)


def point_key(x: float, y: float, layer: str, tolerance: float = 0.02) -> Tuple[int, int, str]:
    """
    Create a hashable key for a point, quantized to tolerance.

    Args:
        x: X coordinate in mm
        y: Y coordinate in mm
        layer: Layer name
        tolerance: Quantization tolerance in mm (default 0.02mm = 20 microns)

    Returns:
        Tuple of (quantized_x, quantized_y, layer) suitable for use as dict key
    """
    return (round(x / tolerance), round(y / tolerance), layer)

if TYPE_CHECKING:
    from kicad_parser import Segment


def point_to_segment_distance(px: float, py: float,
                               x1: float, y1: float,
                               x2: float, y2: float) -> float:
    """
    Calculate minimum distance from point (px, py) to segment (x1,y1)-(x2,y2).

    Projects the point onto the line defined by the segment, clamps to segment
    bounds, and returns the Euclidean distance to that closest point.
    """
    dx = x2 - x1
    dy = y2 - y1
    length_sq = dx * dx + dy * dy

    if length_sq < 1e-10:
        # Segment is effectively a point
        return math.sqrt((px - x1)**2 + (py - y1)**2)

    # Project point onto line, clamped to segment [0, 1]
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / length_sq))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy

    return math.sqrt((px - proj_x)**2 + (py - proj_y)**2)


def point_to_segment_distance_seg(px: float, py: float, seg: "Segment") -> float:
    """Convenience wrapper taking a Segment object."""
    return point_to_segment_distance(px, py, seg.start_x, seg.start_y, seg.end_x, seg.end_y)


def closest_point_on_segment(px: float, py: float,
                              x1: float, y1: float,
                              x2: float, y2: float) -> Tuple[float, float]:
    """
    Find the closest point on segment (x1,y1)-(x2,y2) to point (px, py).

    Returns the (x, y) coordinates of the closest point on the segment.
    """
    dx = x2 - x1
    dy = y2 - y1
    length_sq = dx * dx + dy * dy

    if length_sq < 1e-10:
        return x1, y1

    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / length_sq))
    return x1 + t * dx, y1 + t * dy


def ccw(ax: float, ay: float, bx: float, by: float, cx: float, cy: float) -> bool:
    """
    Check if three points are in counter-clockwise order.

    Used by segment intersection algorithms.
    """
    return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax)


def segments_intersect(seg1_x1: float, seg1_y1: float, seg1_x2: float, seg1_y2: float,
                       seg2_x1: float, seg2_y1: float, seg2_x2: float, seg2_y2: float) -> bool:
    """
    Check if two line segments intersect (cross each other).

    Uses the CCW (counter-clockwise) orientation test. Returns True only for
    proper intersection, not endpoint touching.
    """
    return (ccw(seg1_x1, seg1_y1, seg2_x1, seg2_y1, seg2_x2, seg2_y2) !=
            ccw(seg1_x2, seg1_y2, seg2_x1, seg2_y1, seg2_x2, seg2_y2) and
            ccw(seg1_x1, seg1_y1, seg1_x2, seg1_y2, seg2_x1, seg2_y1) !=
            ccw(seg1_x1, seg1_y1, seg1_x2, seg1_y2, seg2_x2, seg2_y2))


def segments_intersect_tuple(p1: Tuple[float, float], p2: Tuple[float, float],
                              p3: Tuple[float, float], p4: Tuple[float, float]) -> bool:
    """
    Check if segment (p1->p2) intersects segment (p3->p4).

    Convenience wrapper taking tuple endpoints.
    """
    return segments_intersect(p1[0], p1[1], p2[0], p2[1], p3[0], p3[1], p4[0], p4[1])


def segments_intersect_2d(seg1_start: Tuple[float, float], seg1_end: Tuple[float, float],
                          seg2_start: Tuple[float, float], seg2_end: Tuple[float, float],
                          tolerance: float = 0.001) -> bool:
    """
    Check if two 2D line segments intersect, with tolerance for collinear cases.

    Uses cross product method. Handles collinear/touching cases with tolerance.

    Args:
        seg1_start, seg1_end: Endpoints of first segment
        seg2_start, seg2_end: Endpoints of second segment
        tolerance: Numerical tolerance for collinear/touching cases
    """
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    def on_segment(a, b, c):
        return (min(a[0], b[0]) - tolerance <= c[0] <= max(a[0], b[0]) + tolerance and
                min(a[1], b[1]) - tolerance <= c[1] <= max(a[1], b[1]) + tolerance)

    d1 = cross(seg2_start, seg2_end, seg1_start)
    d2 = cross(seg2_start, seg2_end, seg1_end)
    d3 = cross(seg1_start, seg1_end, seg2_start)
    d4 = cross(seg1_start, seg1_end, seg2_end)

    # Standard intersection: segments cross each other
    if ((d1 > tolerance and d2 < -tolerance) or (d1 < -tolerance and d2 > tolerance)) and \
       ((d3 > tolerance and d4 < -tolerance) or (d3 < -tolerance and d4 > tolerance)):
        return True

    # Collinear cases
    if abs(d1) <= tolerance and on_segment(seg2_start, seg2_end, seg1_start):
        return True
    if abs(d2) <= tolerance and on_segment(seg2_start, seg2_end, seg1_end):
        return True
    if abs(d3) <= tolerance and on_segment(seg1_start, seg1_end, seg2_start):
        return True
    if abs(d4) <= tolerance and on_segment(seg1_start, seg1_end, seg2_end):
        return True

    return False


def segment_to_segment_distance(seg1_x1: float, seg1_y1: float, seg1_x2: float, seg1_y2: float,
                                 seg2_x1: float, seg2_y1: float, seg2_x2: float, seg2_y2: float) -> float:
    """
    Calculate minimum distance between two line segments.

    First checks if segments intersect (distance 0), then checks all
    endpoint-to-segment distances and returns the minimum.
    """
    # If segments intersect, distance is 0
    if segments_intersect(seg1_x1, seg1_y1, seg1_x2, seg1_y2,
                          seg2_x1, seg2_y1, seg2_x2, seg2_y2):
        return 0.0

    # Check distance from each endpoint to the other segment
    d1 = point_to_segment_distance(seg1_x1, seg1_y1, seg2_x1, seg2_y1, seg2_x2, seg2_y2)
    d2 = point_to_segment_distance(seg1_x2, seg1_y2, seg2_x1, seg2_y1, seg2_x2, seg2_y2)
    d3 = point_to_segment_distance(seg2_x1, seg2_y1, seg1_x1, seg1_y1, seg1_x2, seg1_y2)
    d4 = point_to_segment_distance(seg2_x2, seg2_y2, seg1_x1, seg1_y1, seg1_x2, seg1_y2)

    return min(d1, d2, d3, d4)


def segment_to_segment_distance_seg(seg1: "Segment", seg2: "Segment") -> float:
    """Calculate minimum distance between two Segment objects."""
    return segment_to_segment_distance(
        seg1.start_x, seg1.start_y, seg1.end_x, seg1.end_y,
        seg2.start_x, seg2.start_y, seg2.end_x, seg2.end_y
    )


def segment_to_segment_closest_points(seg1: "Segment", seg2: "Segment") -> Tuple[float, Tuple[float, float], Tuple[float, float]]:
    """
    Find closest point pair between two segments.

    Returns (distance, point_on_seg1, point_on_seg2).
    """
    candidates = []

    # seg1 endpoints to seg2
    p1 = closest_point_on_segment(seg1.start_x, seg1.start_y,
                                   seg2.start_x, seg2.start_y, seg2.end_x, seg2.end_y)
    d1 = math.sqrt((seg1.start_x - p1[0])**2 + (seg1.start_y - p1[1])**2)
    candidates.append((d1, (seg1.start_x, seg1.start_y), p1))

    p2 = closest_point_on_segment(seg1.end_x, seg1.end_y,
                                   seg2.start_x, seg2.start_y, seg2.end_x, seg2.end_y)
    d2 = math.sqrt((seg1.end_x - p2[0])**2 + (seg1.end_y - p2[1])**2)
    candidates.append((d2, (seg1.end_x, seg1.end_y), p2))

    # seg2 endpoints to seg1
    p3 = closest_point_on_segment(seg2.start_x, seg2.start_y,
                                   seg1.start_x, seg1.start_y, seg1.end_x, seg1.end_y)
    d3 = math.sqrt((seg2.start_x - p3[0])**2 + (seg2.start_y - p3[1])**2)
    candidates.append((d3, p3, (seg2.start_x, seg2.start_y)))

    p4 = closest_point_on_segment(seg2.end_x, seg2.end_y,
                                   seg1.start_x, seg1.start_y, seg1.end_x, seg1.end_y)
    d4 = math.sqrt((seg2.end_x - p4[0])**2 + (seg2.end_y - p4[1])**2)
    candidates.append((d4, p4, (seg2.end_x, seg2.end_y)))

    return min(candidates, key=lambda x: x[0])

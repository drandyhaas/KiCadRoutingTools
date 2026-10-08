"""
Shared geometry utility functions for PCB routing.

This module consolidates geometry calculations used across multiple modules:
- Point-to-segment distance
- Segment-to-segment distance
- Segment intersection detection
- Closest point calculations
- Union-Find data structure for connectivity
- Path simplification (removing collinear points)
"""
from __future__ import annotations

import math
from typing import Tuple, Optional, Dict, List, Any, TYPE_CHECKING


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

    __slots__ = ('_index', '_keys', '_parent', '_rank')

    def __init__(self) -> None:
        # Keys are interned to dense int ids; parent/rank are flat int lists.
        # Root selection matches the classic dict implementation exactly
        # (same union-by-rank, same first-argument tie-break), so callers see
        # identical representatives.
        self._index: Dict[Any, int] = {}
        self._keys: List[Any] = []
        self._parent: List[int] = []
        self._rank: List[int] = []

    def find(self, x: Any) -> Any:
        """
        Find the root representative of the set containing x.

        Uses path compression for O(α(n)) amortized time complexity.
        Creates a new singleton set if x is not yet tracked.
        """
        i = self._index.get(x)
        if i is None:
            keys = self._keys
            self._index[x] = i = len(keys)
            keys.append(x)
            self._parent.append(i)
            self._rank.append(0)
            return x
        parent = self._parent
        root = i
        while parent[root] != root:
            root = parent[root]
        while parent[i] != root:  # Path compression
            parent[i], i = root, parent[i]
        return self._keys[root]

    def _root_id(self, x: Any) -> int:
        i = self._index.get(x)
        if i is None:
            keys = self._keys
            self._index[x] = i = len(keys)
            keys.append(x)
            self._parent.append(i)
            self._rank.append(0)
            return i
        parent = self._parent
        root = i
        while parent[root] != root:
            root = parent[root]
        while parent[i] != root:  # Path compression
            parent[i], i = root, parent[i]
        return root

    def union(self, x: Any, y: Any) -> None:
        """
        Merge the sets containing x and y.

        Uses union by rank for O(α(n)) amortized time complexity.
        """
        index = self._index
        keys = self._keys
        parent = self._parent
        i = index.get(x)
        if i is None:
            index[x] = i = len(keys)
            keys.append(x)
            parent.append(i)
            self._rank.append(0)
            px = i
        else:
            px = i
            while parent[px] != px:
                px = parent[px]
            while parent[i] != px:  # Path compression
                parent[i], i = px, parent[i]
        j = index.get(y)
        if j is None:
            index[y] = j = len(keys)
            keys.append(y)
            parent.append(j)
            self._rank.append(0)
            py = j
        else:
            py = j
            while parent[py] != py:
                py = parent[py]
            while parent[j] != py:  # Path compression
                parent[j], j = py, parent[j]
        if px == py:
            return  # Already in same set
        rank = self._rank
        # Union by rank - attach smaller tree under larger tree
        if rank[px] < rank[py]:
            px, py = py, px
        self._parent[py] = px
        if rank[px] == rank[py]:
            rank[px] += 1

    def connected(self, x: Any, y: Any) -> bool:
        """Check if x and y are in the same set."""
        return self._root_id(x) == self._root_id(y)


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


def point_to_segment_dist_sq(px: float, py: float,
                             x1: float, y1: float,
                             x2: float, y2: float) -> float:
    """Squared distance from point (px, py) to segment (x1,y1)-(x2,y2).

    The one guarded squared-distance kernel: like point_to_segment_distance but
    returns the square (skips the sqrt). Degenerate segments are guarded on
    length_sq < 1e-10 rather than an exact dx==dy==0 test, so a sub-10nm segment
    can never divide by a denormal-tiny length and blow t up to +/-inf.
    """
    dx = x2 - x1
    dy = y2 - y1
    length_sq = dx * dx + dy * dy

    if length_sq < 1e-10:
        # Segment is effectively a point
        return (px - x1) ** 2 + (py - y1) ** 2

    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / length_sq))
    cx = x1 + t * dx
    cy = y1 + t * dy
    return (px - cx) ** 2 + (py - cy) ** 2


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


def simplify_path(path):
    """
    Simplify a path by removing intermediate points on straight lines.
    Only keeps corner points and endpoints.

    Args:
        path: List of (gx, gy, layer) grid coordinates

    Returns:
        Simplified path with collinear points removed
    """
    if len(path) <= 2:
        return list(path)

    result = [path[0]]

    for i in range(1, len(path) - 1):
        prev = path[i - 1]
        curr = path[i]
        next_pt = path[i + 1]

        # Keep if layer changes
        if prev[2] != curr[2] or curr[2] != next_pt[2]:
            result.append(curr)
            continue

        # Direction vectors
        dx1 = curr[0] - prev[0]
        dy1 = curr[1] - prev[1]
        dx2 = next_pt[0] - curr[0]
        dy2 = next_pt[1] - curr[1]

        # Normalize (handle zero case)
        len1 = math.sqrt(dx1*dx1 + dy1*dy1) or 1
        len2 = math.sqrt(dx2*dx2 + dy2*dy2) or 1

        # Check if directions are the same (collinear)
        ndx1, ndy1 = dx1/len1, dy1/len1
        ndx2, ndy2 = dx2/len2, dy2/len2

        # Not collinear if directions differ (keep this corner point)
        if abs(ndx1 - ndx2) > 0.01 or abs(ndy1 - ndy2) > 0.01:
            result.append(curr)

    result.append(path[-1])
    return result


# ---------------------------------------------------------------------------
# Overlap thickness (#1094), shared since #1111
# ---------------------------------------------------------------------------
# Moved here VERBATIM from placement/legality.py so check_pads (#1111) measures
# a custom pad's copper overlap with the ruler the placement graders use --
# calling it, not mirroring it -- without the router importing placement.
# legality keeps `overlap_thickness` as a thin wrapper over this one.

#: The area (mm2) below which a polygon part is a sliver, not a region:
#: placement.legality's `EPS`, which the moved code compared against.
AREA_EPS_MM2 = 1e-6


#: The precision (mm) `overlap_thickness` measures a non-rectangular region
#: to: the last decimal `depth_mm` is rounded to. A rectangle is exact.
THICKNESS_TOL_MM = 1e-4


def _areal_parts(geom):
    """The Polygon parts of any geometry: an intersection can be a Polygon, a
    MultiPolygon, or a collection that also carries lines and points."""
    if geom is None or geom.is_empty:
        return []
    if geom.geom_type == 'Polygon':
        return [geom] if geom.area > AREA_EPS_MM2 else []
    out = []
    for g in getattr(geom, 'geoms', ()):
        out.extend(_areal_parts(g))
    return out


#: Public name for the placement graders (#1123, via
#: `check_pads.custom_pad_copper`); the body stays where #1111 anchored it.
areal_parts = _areal_parts


def _inscribed_radius(poly) -> float:
    """Radius of the largest circle inside `poly`, to `THICKNESS_TOL_MM`.
    shapely 2.1 has `maximum_inscribed_circle`; before it, `polylabel` finds
    the same centre (the pole of inaccessibility)."""
    import shapely
    mic = getattr(shapely, 'maximum_inscribed_circle', None)
    if mic is not None:
        return float(mic(poly, tolerance=THICKNESS_TOL_MM).length)
    from shapely.ops import polylabel
    return float(poly.boundary.distance(
        polylabel(poly, tolerance=THICKNESS_TOL_MM)))


def overlap_thickness(geom) -> float:
    """How thick an overlap region is, in mm: twice the radius of the largest
    circle it contains, over its thickest part.

    For a rectangle that is its shorter side -- the rect channel's
    `min(dx, dy)` -- and a rectangular region is measured exactly so, with
    no search. The shorter side of the region's minimum ROTATED rectangle,
    which this replaced, spans the whole region instead of its thickness: a
    part tucked into a stepped courtyard's corner notch, grazing both arms
    0.2 mm deep, read 2.2 mm and tripped the 0.3 mm blocking floor, and two
    disjoint slivers read as one region as wide as the gap between them.
    """
    best = 0.0
    for p in _areal_parts(geom):
        mrr = p.minimum_rotated_rectangle
        cs = list(mrr.exterior.coords) if hasattr(mrr, 'exterior') else []
        if len(cs) < 4:
            continue
        short = min(math.hypot(cs[1][0] - cs[0][0], cs[1][1] - cs[0][1]),
                    math.hypot(cs[2][0] - cs[1][0], cs[2][1] - cs[1][1]))
        if abs(mrr.area - p.area) <= 1e-9 * max(1.0, mrr.area):
            best = max(best, short)            # the region IS a rectangle
        else:
            best = max(best, min(short, 2.0 * _inscribed_radius(p)))
    return best

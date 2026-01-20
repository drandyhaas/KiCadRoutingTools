"""
Zone geometry utilities for copper plane generation.

Handles Voronoi zone boundary computation, polygon clipping, merging, and grouping.
"""

import math
from typing import List, Dict, Tuple, Optional

import numpy as np

from geometry_utils import UnionFind
from routing_constants import POLYGON_BUFFER_DISTANCE, POLYGON_EDGE_TOLERANCE
from scipy.spatial import Voronoi
from shapely.geometry import Polygon as ShapelyPolygon
from shapely.ops import unary_union
from shapely.validation import make_valid


def compute_zone_boundaries(
    vias_by_net: Dict[int, List[Tuple[float, float]]],
    board_bounds: Tuple[float, float, float, float],
    return_raw_polygons: bool = False,
    board_edge_clearance: float = 0.0,
    verbose: bool = False
) -> Dict[int, List[Tuple[float, float]]]:
    """
    Compute non-overlapping zone polygons for multiple nets using Voronoi.

    Algorithm:
    1. Create Voronoi cell around EACH via (not centroid)
    2. Label each cell with its via's net_id
    3. Clip cells to board bounds (inset by board_edge_clearance)
    4. Merge adjacent cells of the same net
    5. If same-net cells aren't adjacent → they become disconnected regions

    Args:
        vias_by_net: Dict mapping net_id → list of (x, y) via positions
        board_bounds: Tuple of (min_x, min_y, max_x, max_y)
        return_raw_polygons: If True, return tuple (merged_polygons, raw_polygons, via_to_polygon_idx)
                            where raw_polygons[net_id] is list of individual Voronoi cells,
                            and via_to_polygon_idx[net_id] maps via position to polygon index
        board_edge_clearance: Clearance from board edge for zone polygons (mm)

    Returns:
        If return_raw_polygons=False:
            Dict mapping net_id → polygon points (list of (x, y) tuples)
        If return_raw_polygons=True:
            Tuple of (merged_polygons, raw_polygons, via_to_polygon_idx)

    """
    min_x, min_y, max_x, max_y = board_bounds

    # Apply board edge clearance (inset the clipping bounds)
    clip_min_x = min_x + board_edge_clearance
    clip_min_y = min_y + board_edge_clearance
    clip_max_x = max_x - board_edge_clearance
    clip_max_y = max_y - board_edge_clearance

    # Collect all vias into a single list with net labels
    all_vias = []
    via_net_ids = []
    for net_id, positions in vias_by_net.items():
        for pos in positions:
            all_vias.append(pos)
            via_net_ids.append(net_id)

    if len(all_vias) < 2:
        # Single via or empty: return full board rectangle (with clearance) for the one net
        if len(all_vias) == 1:
            net_id = via_net_ids[0]
            return {net_id: [[(clip_min_x, clip_min_y), (clip_max_x, clip_min_y),
                             (clip_max_x, clip_max_y), (clip_min_x, clip_max_y)]]}
        return {}

    # Add mirror points outside board bounds to ensure all regions are finite
    # This is a standard technique for bounded Voronoi
    pad = max(max_x - min_x, max_y - min_y) * 2
    mirror_points = [
        (min_x - pad, min_y - pad),
        (max_x + pad, min_y - pad),
        (min_x - pad, max_y + pad),
        (max_x + pad, max_y + pad),
        ((min_x + max_x) / 2, min_y - pad),
        ((min_x + max_x) / 2, max_y + pad),
        (min_x - pad, (min_y + max_y) / 2),
        (max_x + pad, (min_y + max_y) / 2),
    ]

    points = np.array(all_vias + mirror_points)
    vor = Voronoi(points)

    # Build polygon for each real via (not mirror points)
    # Also track which via produced which polygon (for disconnection routing)
    via_polygons: Dict[int, List[List[Tuple[float, float]]]] = {net_id: [] for net_id in vias_by_net}
    via_to_polygon_idx: Dict[int, Dict[Tuple[float, float], int]] = {net_id: {} for net_id in vias_by_net}

    for via_idx in range(len(all_vias)):
        region_idx = vor.point_region[via_idx]
        region = vor.regions[region_idx]

        if -1 in region or len(region) == 0:
            # Infinite region (shouldn't happen with mirror points, but handle it)
            continue

        # Get polygon vertices
        polygon = [tuple(vor.vertices[i]) for i in region]

        # Clip polygon to board bounds (with edge clearance applied)
        clipped = clip_polygon_to_rect(polygon, clip_min_x, clip_min_y, clip_max_x, clip_max_y)

        if clipped and len(clipped) >= 3:
            net_id = via_net_ids[via_idx]
            polygon_idx = len(via_polygons[net_id])
            via_polygons[net_id].append(clipped)
            # Track which via produced this polygon
            via_pos = all_vias[via_idx]
            via_to_polygon_idx[net_id][via_pos] = polygon_idx

    # Merge polygons for each net
    result: Dict[int, List[Tuple[float, float]]] = {}

    for net_id, polygons in via_polygons.items():
        if not polygons:
            continue

        if len(polygons) == 1:
            result[net_id] = [polygons[0]]
        else:
            # Merge all polygons for this net using Shapely
            # This preserves Voronoi boundaries (unlike convex hull)
            if verbose:
                print(f"    Merging {len(polygons)} polygons for net {net_id}")
            merged = merge_polygons(polygons, verbose=verbose)
            if merged:
                result[net_id] = [merged]
            else:
                # Polygons are disconnected - use unary_union to combine what we can
                # This preserves boundaries rather than using convex hull which would overlap
                if verbose:
                    print(f"    merge_polygons returned None - using unary_union fallback")
                shapely_polys = []
                for poly in polygons:
                    if len(poly) >= 3:
                        sp = ShapelyPolygon(poly)
                        if not sp.is_valid:
                            sp = make_valid(sp)
                        if sp.is_valid and not sp.is_empty:
                            shapely_polys.append(sp)
                if shapely_polys:
                    # First try direct union
                    combined = unary_union(shapely_polys)

                    # If MultiPolygon, try buffering to merge nearly-touching polygons
                    # (Voronoi cells can have tiny gaps due to floating point precision)
                    if combined.geom_type == 'MultiPolygon':
                        buffer_dist = POLYGON_BUFFER_DISTANCE
                        buffered = [p.buffer(buffer_dist) for p in shapely_polys]
                        combined_buffered = unary_union(buffered)
                        if combined_buffered.geom_type == 'Polygon':
                            # Successfully merged - shrink back
                            combined = combined_buffered.buffer(-buffer_dist)
                            if verbose:
                                print(f"    Buffered merge succeeded")
                        elif combined_buffered.geom_type == 'MultiPolygon':
                            # Still disconnected - shrink each part back
                            shrunk_parts = []
                            for geom in combined_buffered.geoms:
                                shrunk = geom.buffer(-buffer_dist)
                                if not shrunk.is_empty:
                                    shrunk_parts.append(shrunk)
                            if shrunk_parts:
                                combined = unary_union(shrunk_parts) if len(shrunk_parts) > 1 else shrunk_parts[0]

                    if verbose:
                        print(f"    Fallback unary_union result: {combined.geom_type}")
                    if combined.geom_type == 'Polygon':
                        coords = list(combined.exterior.coords)[:-1]
                        result[net_id] = [[(float(x), float(y)) for x, y in coords]]
                    elif combined.geom_type == 'MultiPolygon':
                        # Keep ALL polygons - they'll become separate zones
                        if verbose:
                            print(f"    Net has {len(combined.geoms)} disconnected regions (separate zones will be created)")
                            for i, geom in enumerate(combined.geoms):
                                print(f"      Region {i}: area={geom.area:.2f}")
                        result[net_id] = []
                        for geom in combined.geoms:
                            coords = list(geom.exterior.coords)[:-1]
                            result[net_id].append([(float(x), float(y)) for x, y in coords])

    if return_raw_polygons:
        return result, via_polygons, via_to_polygon_idx
    return result


def clip_polygon_to_rect(
    polygon: List[Tuple[float, float]],
    min_x: float, min_y: float, max_x: float, max_y: float
) -> List[Tuple[float, float]]:
    """
    Clip a polygon to a rectangle using Sutherland-Hodgman algorithm.
    """
    def inside_edge(p, edge):
        """Check if point p is inside the clipping edge."""
        x, y = p
        if edge == 'left':
            return x >= min_x
        elif edge == 'right':
            return x <= max_x
        elif edge == 'bottom':
            return y >= min_y
        elif edge == 'top':
            return y <= max_y
        return True

    def intersect_edge(p1, p2, edge):
        """Find intersection of line p1-p2 with clipping edge."""
        x1, y1 = p1
        x2, y2 = p2
        dx = x2 - x1
        dy = y2 - y1

        if edge == 'left':
            if abs(dx) < 1e-10:
                return (min_x, y1)
            t = (min_x - x1) / dx
            return (min_x, y1 + t * dy)
        elif edge == 'right':
            if abs(dx) < 1e-10:
                return (max_x, y1)
            t = (max_x - x1) / dx
            return (max_x, y1 + t * dy)
        elif edge == 'bottom':
            if abs(dy) < 1e-10:
                return (x1, min_y)
            t = (min_y - y1) / dy
            return (x1 + t * dx, min_y)
        elif edge == 'top':
            if abs(dy) < 1e-10:
                return (x1, max_y)
            t = (max_y - y1) / dy
            return (x1 + t * dx, max_y)
        return p1

    output = polygon
    for edge in ['left', 'right', 'bottom', 'top']:
        if not output:
            return []
        input_poly = output
        output = []

        for i in range(len(input_poly)):
            p1 = input_poly[i]
            p2 = input_poly[(i + 1) % len(input_poly)]

            p1_inside = inside_edge(p1, edge)
            p2_inside = inside_edge(p2, edge)

            if p1_inside and p2_inside:
                output.append(p2)
            elif p1_inside and not p2_inside:
                output.append(intersect_edge(p1, p2, edge))
            elif not p1_inside and p2_inside:
                output.append(intersect_edge(p1, p2, edge))
                output.append(p2)
            # else: both outside, add nothing

    return output


def merge_polygons(polygons: List[List[Tuple[float, float]]], verbose: bool = False) -> Optional[List[Tuple[float, float]]]:
    """
    Merge a list of adjacent polygons into a single polygon.
    Returns None if polygons are not all adjacent (i.e., disconnected).

    Uses Shapely's unary_union for proper polygon union that preserves
    the original edges (unlike convex hull which loses concave details).
    """
    if not polygons:
        return None
    if len(polygons) == 1:
        return polygons[0]

    # Check if all polygons are connected via shared edges
    groups = find_polygon_groups(polygons, verbose=verbose)
    if len(groups) > 1:
        # Polygons are disconnected - return None to signal caller
        return None

    # Use Shapely for proper polygon union (preserves Voronoi boundaries)
    # Convert to Shapely polygons
    shapely_polys = []
    for poly in polygons:
        if len(poly) >= 3:
            sp = ShapelyPolygon(poly)
            if not sp.is_valid:
                sp = make_valid(sp)
            if sp.is_valid and not sp.is_empty:
                shapely_polys.append(sp)

    if not shapely_polys:
        return None

    # Union all polygons
    merged = unary_union(shapely_polys)

    if verbose:
        print(f"      merge_polygons: unary_union result type = {merged.geom_type}")

    # Extract exterior coordinates
    if merged.is_empty:
        return None
    if merged.geom_type == 'Polygon':
        coords = list(merged.exterior.coords)[:-1]  # Remove duplicate closing point
        return [(float(x), float(y)) for x, y in coords]
    elif merged.geom_type == 'MultiPolygon':
        # Multiple disconnected polygons - return None to let caller handle them all
        if verbose:
            print(f"      WARNING: unary_union returned MultiPolygon with {len(merged.geoms)} parts!")
            for i, geom in enumerate(merged.geoms):
                print(f"        Part {i}: area={geom.area:.2f}, centroid=({geom.centroid.x:.2f}, {geom.centroid.y:.2f})")
        return None  # Let compute_zone_boundaries handle all polygons via fallback
    else:
        return None


def polygons_share_edge(
    poly1: List[Tuple[float, float]],
    poly2: List[Tuple[float, float]],
    tolerance: float = POLYGON_EDGE_TOLERANCE
) -> bool:
    """
    Check if two polygons share a common edge (not just a point).

    Two polygons share an edge if they have two consecutive vertices that
    match (in either direction) within tolerance.
    """
    if len(poly1) < 2 or len(poly2) < 2:
        return False

    # Get all edges from both polygons
    edges1 = []
    for i in range(len(poly1)):
        p1 = poly1[i]
        p2 = poly1[(i + 1) % len(poly1)]
        edges1.append((p1, p2))

    edges2 = []
    for i in range(len(poly2)):
        p1 = poly2[i]
        p2 = poly2[(i + 1) % len(poly2)]
        edges2.append((p1, p2))

    def points_match(a: Tuple[float, float], b: Tuple[float, float]) -> bool:
        return abs(a[0] - b[0]) < tolerance and abs(a[1] - b[1]) < tolerance

    def edges_match(e1: Tuple, e2: Tuple) -> bool:
        # Check if edges match in either direction
        return ((points_match(e1[0], e2[0]) and points_match(e1[1], e2[1])) or
                (points_match(e1[0], e2[1]) and points_match(e1[1], e2[0])))

    # Check if any edge from poly1 matches any edge from poly2
    for e1 in edges1:
        for e2 in edges2:
            if edges_match(e1, e2):
                return True

    return False


def find_polygon_groups(
    polygons: List[List[Tuple[float, float]]],
    tolerance: float = POLYGON_EDGE_TOLERANCE,
    verbose: bool = False
) -> List[List[int]]:
    """
    Group polygons by adjacency using union-find.

    Two polygons are adjacent if they share at least one edge.

    Args:
        polygons: List of polygons, each polygon is list of (x, y) vertices
        tolerance: Distance tolerance for vertex matching
        verbose: Print debug information about polygon adjacency

    Returns:
        List of groups, each group is list of polygon indices
    """
    if not polygons:
        return []

    n = len(polygons)
    uf = UnionFind()

    if verbose:
        print(f"      find_polygon_groups: checking {n} polygons for adjacency")
        for i, poly in enumerate(polygons):
            # Compute centroid for easier identification
            cx = sum(p[0] for p in poly) / len(poly)
            cy = sum(p[1] for p in poly) / len(poly)
            print(f"        Polygon {i}: {len(poly)} vertices, centroid ({cx:.2f}, {cy:.2f})")

    # Check all pairs for shared edges
    adjacencies = []
    for i in range(n):
        for j in range(i + 1, n):
            if polygons_share_edge(polygons[i], polygons[j], tolerance):
                uf.union(i, j)
                adjacencies.append((i, j))

    if verbose:
        print(f"      Found {len(adjacencies)} adjacencies: {adjacencies}")

    # Group polygons by their root
    groups: Dict[int, List[int]] = {}
    for i in range(n):
        root = uf.find(i)
        if root not in groups:
            groups[root] = []
        groups[root].append(i)

    result = list(groups.values())
    if verbose:
        print(f"      Resulting groups: {result}")

    return result


def sample_route_for_voronoi(
    route_path: List[Tuple[float, float]],
    sample_interval: float = 2.0
) -> List[Tuple[float, float]]:
    """
    Sample points along a route path for Voronoi seeding.

    Args:
        route_path: List of (x, y) points along the route
        sample_interval: Distance between samples in mm

    Returns:
        List of (x, y) sample points along the route
    """
    if not route_path or len(route_path) < 2:
        return []

    samples = []
    accumulated_dist = 0.0

    for i in range(len(route_path) - 1):
        x1, y1 = route_path[i]
        x2, y2 = route_path[i + 1]

        seg_len = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        if seg_len < 0.001:
            continue

        # Direction vector
        dx = (x2 - x1) / seg_len
        dy = (y2 - y1) / seg_len

        # Sample along this segment
        dist_in_seg = 0.0
        while dist_in_seg < seg_len:
            # Check if we need to place a sample
            if accumulated_dist >= sample_interval:
                # Place sample
                x = x1 + dx * dist_in_seg
                y = y1 + dy * dist_in_seg
                samples.append((x, y))
                accumulated_dist = 0.0

            # Step forward
            step = min(sample_interval - accumulated_dist, seg_len - dist_in_seg)
            dist_in_seg += step
            accumulated_dist += step

    return samples

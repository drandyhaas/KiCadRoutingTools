"""
Plane resistance and current capacity calculations.

Provides functions to calculate resistance of copper plane polygons and
maximum current capacity based on IPC-2152 guidelines.
"""

import math
from typing import List, Dict, Tuple, Optional
from shapely.geometry import Polygon as ShapelyPolygon, LineString, Point
from shapely.validation import make_valid


# Physical constants
COPPER_RESISTIVITY = 1.68e-8  # Ohm·m at 20°C
OZ_TO_METERS = 35e-6  # 1 oz copper = 35 microns


def find_mst_diameter_path(
    mst_edges: List[Tuple[Tuple[float, float], Tuple[float, float]]],
    routed_paths: Dict[Tuple[Tuple[float, float], Tuple[float, float]], List[Tuple[float, float]]]
) -> Tuple[List[Tuple[float, float]], float]:
    """
    Find the longest path (diameter) through the MST and return the actual routed path.

    Args:
        mst_edges: List of (via_a, via_b) MST edges
        routed_paths: Dict mapping (via_a, via_b) -> list of route points

    Returns:
        (path_points, total_length) - the actual routed path and its length in mm
    """
    if not mst_edges:
        return [], 0.0

    # Build adjacency list from MST edges
    adjacency: Dict[Tuple[float, float], List[Tuple[Tuple[float, float], float, List[Tuple[float, float]]]]] = {}

    for via_a, via_b in mst_edges:
        # Get the routed path for this edge
        route = routed_paths.get((via_a, via_b)) or routed_paths.get((via_b, via_a))
        if not route:
            # Fall back to straight line
            route = [via_a, via_b]

        # Calculate route length
        length = 0.0
        for i in range(len(route) - 1):
            dx = route[i+1][0] - route[i][0]
            dy = route[i+1][1] - route[i][1]
            length += math.sqrt(dx*dx + dy*dy)

        # Add to adjacency (both directions)
        if via_a not in adjacency:
            adjacency[via_a] = []
        if via_b not in adjacency:
            adjacency[via_b] = []
        adjacency[via_a].append((via_b, length, route))
        adjacency[via_b].append((via_a, length, list(reversed(route))))

    if not adjacency:
        return [], 0.0

    # Find diameter using two BFS passes
    start_node = next(iter(adjacency.keys()))
    farthest_a, _ = _bfs_farthest(start_node, adjacency)
    farthest_b, path_info = _bfs_farthest(farthest_a, adjacency)

    # Reconstruct the path
    path_points, total_length = _reconstruct_path(farthest_a, farthest_b, path_info, adjacency)

    return path_points, total_length


def _bfs_farthest(
    start: Tuple[float, float],
    adjacency: Dict[Tuple[float, float], List[Tuple[Tuple[float, float], float, List[Tuple[float, float]]]]]
) -> Tuple[Tuple[float, float], Dict]:
    """BFS to find farthest node from start."""
    from collections import deque

    dist = {start: 0.0}
    parent = {start: None}
    queue = deque([start])
    farthest = start
    max_dist = 0.0

    while queue:
        node = queue.popleft()
        for neighbor, edge_len, _ in adjacency.get(node, []):
            if neighbor not in dist:
                dist[neighbor] = dist[node] + edge_len
                parent[neighbor] = node
                queue.append(neighbor)
                if dist[neighbor] > max_dist:
                    max_dist = dist[neighbor]
                    farthest = neighbor

    return farthest, {'dist': dist, 'parent': parent}


def _reconstruct_path(
    start: Tuple[float, float],
    end: Tuple[float, float],
    path_info: Dict,
    adjacency: Dict[Tuple[float, float], List[Tuple[Tuple[float, float], float, List[Tuple[float, float]]]]]
) -> Tuple[List[Tuple[float, float]], float]:
    """Reconstruct the actual routed path between start and end."""
    via_sequence = []
    current = end
    while current is not None:
        via_sequence.append(current)
        current = path_info['parent'].get(current)
    via_sequence.reverse()

    if len(via_sequence) < 2:
        return list(via_sequence), 0.0

    full_path = []
    total_length = 0.0

    for i in range(len(via_sequence) - 1):
        via_a = via_sequence[i]
        via_b = via_sequence[i + 1]

        route = None
        for neighbor, edge_len, edge_route in adjacency.get(via_a, []):
            if neighbor == via_b:
                route = edge_route
                total_length += edge_len
                break

        if route:
            if i == 0:
                full_path.extend(route)
            else:
                full_path.extend(route[1:])

    return full_path, total_length


def calculate_polygon_width_at_point(
    polygon: ShapelyPolygon,
    point: Tuple[float, float],
    direction: Tuple[float, float]
) -> float:
    """
    Calculate polygon width at a point, perpendicular to the given direction.

    Args:
        polygon: Shapely polygon
        point: (x, y) point inside polygon
        direction: (dx, dy) direction of travel (width is perpendicular)

    Returns:
        Width in mm, or 0 if point is outside polygon
    """
    mag = math.sqrt(direction[0]**2 + direction[1]**2)
    if mag < 1e-9:
        return 0.0
    dx, dy = direction[0] / mag, direction[1] / mag

    # Perpendicular direction
    perp_x, perp_y = -dy, dx

    p = Point(point)
    if not polygon.contains(p):
        return 0.0

    # Create line through point in perpendicular direction
    ray_length = 1000.0
    line = LineString([
        (point[0] - perp_x * ray_length, point[1] - perp_y * ray_length),
        (point[0] + perp_x * ray_length, point[1] + perp_y * ray_length)
    ])

    intersection = line.intersection(polygon)

    if intersection.is_empty:
        return 0.0

    if intersection.geom_type == 'LineString':
        return intersection.length
    elif intersection.geom_type == 'MultiLineString':
        for geom in intersection.geoms:
            if geom.distance(p) < 0.01:
                return geom.length
        return intersection.length

    return 0.0


def calculate_average_width_along_path(
    polygon: ShapelyPolygon,
    path: List[Tuple[float, float]],
    sample_interval: float = 1.0
) -> float:
    """
    Calculate average polygon width along a path.

    Args:
        polygon: Shapely polygon
        path: List of (x, y) points along the path
        sample_interval: Distance between samples in mm

    Returns:
        Average width in mm
    """
    if len(path) < 2:
        return 0.0

    widths = []
    accumulated_dist = 0.0

    for i in range(len(path) - 1):
        x1, y1 = path[i]
        x2, y2 = path[i + 1]

        seg_dx = x2 - x1
        seg_dy = y2 - y1
        seg_len = math.sqrt(seg_dx**2 + seg_dy**2)

        if seg_len < 0.001:
            continue

        direction = (seg_dx, seg_dy)
        dist_in_seg = 0.0

        while dist_in_seg < seg_len:
            if accumulated_dist >= sample_interval or len(widths) == 0:
                t = dist_in_seg / seg_len
                px = x1 + t * seg_dx
                py = y1 + t * seg_dy

                width = calculate_polygon_width_at_point(polygon, (px, py), direction)
                if width > 0:
                    widths.append(width)
                accumulated_dist = 0.0

            step = min(sample_interval - accumulated_dist, seg_len - dist_in_seg)
            dist_in_seg += step
            accumulated_dist += step

    if not widths:
        return 0.0

    return sum(widths) / len(widths)


def calculate_resistance(
    path_length_mm: float,
    avg_width_mm: float,
    copper_oz: float = 1.0
) -> float:
    """
    Calculate resistance of a plane section.

    R = ρ * L / (W * t)

    Args:
        path_length_mm: Length of current path in mm
        avg_width_mm: Average width perpendicular to current flow in mm
        copper_oz: Copper weight in oz (1 oz = 35 µm)

    Returns:
        Resistance in ohms
    """
    if path_length_mm < 0.001 or avg_width_mm < 0.001:
        return float('inf')

    length_m = path_length_mm * 1e-3
    width_m = avg_width_mm * 1e-3
    thickness_m = copper_oz * OZ_TO_METERS

    return COPPER_RESISTIVITY * length_m / (width_m * thickness_m)


def calculate_max_current_ipc(
    avg_width_mm: float,
    copper_oz: float = 1.0,
    temp_rise_c: float = 10.0,
    is_internal: bool = True
) -> float:
    """
    Calculate max current using IPC-2152 formula.

    I = k * ΔT^0.44 * A^0.725
    where A is cross-sectional area in mils²
    k = 0.024 for internal, 0.048 for external layers

    Args:
        avg_width_mm: Average width in mm
        copper_oz: Copper weight in oz
        temp_rise_c: Allowed temperature rise in °C
        is_internal: True for internal layers

    Returns:
        Maximum current in amps
    """
    if avg_width_mm < 0.001:
        return 0.0

    thickness_mm = copper_oz * 0.035
    cross_section_mm2 = avg_width_mm * thickness_mm
    cross_section_mils2 = cross_section_mm2 * (1000 / 25.4)**2

    k = 0.024 if is_internal else 0.048
    return k * (temp_rise_c ** 0.44) * (cross_section_mils2 ** 0.725)


def analyze_single_net_plane(
    polygon_points: List[Tuple[float, float]],
    layer: str,
    copper_oz: float = 1.0,
    temp_rise_c: float = 10.0
) -> Optional[Dict]:
    """
    Analyze resistance for a single-net plane using bounding box diagonal.

    Args:
        polygon_points: List of (x, y) vertices
        layer: Layer name (e.g., 'In4.Cu')
        copper_oz: Copper weight in oz
        temp_rise_c: Temperature rise limit in °C

    Returns:
        Dict with path_length, avg_width, resistance, max_current, or None on error
    """
    try:
        shapely_poly = ShapelyPolygon(polygon_points)
        if not shapely_poly.is_valid:
            shapely_poly = make_valid(shapely_poly)

        # Use bounding box diagonal
        minx, miny, maxx, maxy = shapely_poly.bounds
        point_a = (minx, miny)
        point_b = (maxx, maxy)
        path_length = math.sqrt((maxx - minx)**2 + (maxy - miny)**2)

        if path_length < 0.001:
            return None

        # Create straight-line path for width sampling
        num_samples = max(2, int(path_length / 1.0))
        straight_path = []
        for i in range(num_samples + 1):
            t = i / num_samples
            x = point_a[0] + t * (point_b[0] - point_a[0])
            y = point_a[1] + t * (point_b[1] - point_a[1])
            straight_path.append((x, y))

        avg_width = calculate_average_width_along_path(shapely_poly, straight_path, sample_interval=1.0)

        if avg_width < 0.001:
            avg_width = shapely_poly.area / path_length

        resistance = calculate_resistance(path_length, avg_width, copper_oz)
        is_internal = layer.startswith('In')
        max_current = calculate_max_current_ipc(avg_width, copper_oz, temp_rise_c, is_internal)

        return {
            'path_length': path_length,
            'avg_width': avg_width,
            'resistance': resistance,
            'max_current': max_current
        }
    except Exception:
        return None


def analyze_multi_net_plane(
    polygon_points: List[Tuple[float, float]],
    mst_edges: List[Tuple[Tuple[float, float], Tuple[float, float]]],
    routed_paths: Dict[Tuple[Tuple[float, float], Tuple[float, float]], List[Tuple[float, float]]],
    layer: str,
    copper_oz: float = 1.0,
    temp_rise_c: float = 10.0
) -> Optional[Dict]:
    """
    Analyze resistance for a multi-net plane using MST diameter path.

    Args:
        polygon_points: List of (x, y) vertices
        mst_edges: List of MST edges as (via_a, via_b) tuples
        routed_paths: Dict mapping edges to routed path points
        layer: Layer name
        copper_oz: Copper weight in oz
        temp_rise_c: Temperature rise limit in °C

    Returns:
        Dict with path_length, avg_width, resistance, max_current, or None on error
    """
    if not mst_edges or not routed_paths:
        return None

    try:
        # Find longest path through MST
        diameter_path, path_length = find_mst_diameter_path(mst_edges, routed_paths)

        if not diameter_path or path_length < 0.001:
            return None

        shapely_poly = ShapelyPolygon(polygon_points)
        if not shapely_poly.is_valid:
            shapely_poly = make_valid(shapely_poly)

        avg_width = calculate_average_width_along_path(shapely_poly, diameter_path, sample_interval=1.0)

        if avg_width < 0.001:
            avg_width = shapely_poly.area / path_length

        resistance = calculate_resistance(path_length, avg_width, copper_oz)
        is_internal = layer.startswith('In')
        max_current = calculate_max_current_ipc(avg_width, copper_oz, temp_rise_c, is_internal)

        return {
            'path_length': path_length,
            'avg_width': avg_width,
            'resistance': resistance,
            'max_current': max_current
        }
    except Exception:
        return None


def print_single_net_resistance(result: Dict, net_name: str):
    """Print resistance analysis for a single-net plane."""
    if not result:
        return

    print(f"\n  Plane Resistance Analysis (1 oz copper, 10°C rise):")
    print(f"    Path length: {result['path_length']:.1f} mm (diagonal)")
    print(f"    Avg width:   {result['avg_width']:.1f} mm")
    r_str = f"{result['resistance']*1000:.3f} mΩ" if result['resistance'] < float('inf') else "N/A"
    i_str = f"{result['max_current']:.2f} A" if result['max_current'] > 0 else "N/A"
    print(f"    Resistance:  {r_str}")
    print(f"    Max current: {i_str}")


def print_multi_net_resistance(results: Dict[str, Dict]):
    """
    Print resistance analysis table for multi-net planes.

    Args:
        results: Dict mapping net_name -> analysis result dict
    """
    if not results:
        return

    print(f"\n  Plane Resistance Analysis (1 oz copper, 10°C rise):")
    print(f"  {'-'*70}")
    print(f"  {'Net':<25} {'Path(mm)':<10} {'AvgW(mm)':<10} {'R(mΩ)':<10} {'Imax(A)':<10}")
    print(f"  {'-'*70}")

    for net_name, result in results.items():
        if result:
            r_str = f"{result['resistance']*1000:.3f}" if result['resistance'] < float('inf') else "N/A"
            i_str = f"{result['max_current']:.2f}" if result['max_current'] > 0 else "N/A"
            print(f"  {net_name:<25} {result['path_length']:<10.1f} {result['avg_width']:<10.1f} {r_str:<10} {i_str:<10}")
        else:
            print(f"  {net_name:<25} {'N/A':<10} {'N/A':<10} {'N/A':<10} {'N/A':<10}")

    print(f"  {'-'*70}")
    print(f"  Path = longest MST route, AvgW = avg polygon width along path")

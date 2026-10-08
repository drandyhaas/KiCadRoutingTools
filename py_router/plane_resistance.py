"""
Plane resistance and current capacity calculations.

Provides functions to calculate resistance of copper plane polygons and
maximum current capacity from the IPC current-carrying charts.

Which standard: the closed-form `I = k * dT^0.44 * A^0.725` here is the
**IPC-2221** chart fit -- it was labeled IPC-2152 throughout for a long time
(#489 §6). IPC-2152 came later and specifically overturned 2221's 2x derating of
internal layers. Both are offered explicitly; neither is a substitute for the
standard's own charts outside their data range, which is why results carry an
`in_chart_range` flag instead of being printed as bare ratings.
"""
from __future__ import annotations

import math
from typing import List, Dict, Tuple, Optional
from shapely.geometry import Polygon as ShapelyPolygon, LineString, Point
from shapely.validation import make_valid

from console_encoding import ascii_safe
from routing_constants import (
    IPC_2221_EXPONENT_DT,
    IPC_2221_EXPONENT_AREA,
    IPC_2221_K_INTERNAL,
    IPC_2221_K_EXTERNAL,
    IPC_2221_MAX_AREA_MILS2,
)


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


def cross_section_mils2(avg_width_mm: float, copper_oz: float) -> float:
    """Conductor cross-section in mils², the x-axis of the IPC current charts."""
    thickness_mm = copper_oz * OZ_TO_METERS * 1e3  # 1 oz = 35 µm
    return (avg_width_mm * thickness_mm) * (1000 / 25.4) ** 2


def ipc2221_area_in_range(avg_width_mm: float, copper_oz: float = 1.0) -> bool:
    """Is this conductor inside the IPC-2221 chart's data range?

    False means the number the fit produces is an EXTRAPOLATION, not a rating --
    the case that printed "Avg width 52.2 mm -> Max current 21.05 A" for a pour
    (#489 §6).
    """
    return cross_section_mils2(avg_width_mm, copper_oz) <= IPC_2221_MAX_AREA_MILS2


def calculate_max_current_ipc2221(
    avg_width_mm: float,
    copper_oz: float = 1.0,
    temp_rise_c: float = 10.0,
    is_internal: bool = True
) -> float:
    """
    Max current from the IPC-2221 chart fit.

        I = k * ΔT^0.44 * A^0.725     (A in mils²)
        k = 0.024 internal, 0.048 external

    This function was named `calculate_max_current_ipc` and documented as
    IPC-2152 in four places (#489 §6); the formula and both k values are
    IPC-2221's. The 2x internal derating is 2221's, and IPC-2152 overturned it
    -- see `calculate_max_current_ipc2152`.

    The result is only a rating inside the chart's data range; check
    `ipc2221_area_in_range` before presenting it (a wide pour extrapolates).

    Args:
        avg_width_mm: Conductor width in mm
        copper_oz: Copper weight in oz (1 oz = 35 µm)
        temp_rise_c: Allowed temperature rise in °C
        is_internal: True for internal layers (applies 2221's 2x derating)

    Returns:
        Maximum current in amps
    """
    if avg_width_mm < 0.001:
        return 0.0

    area = cross_section_mils2(avg_width_mm, copper_oz)
    k = IPC_2221_K_INTERNAL if is_internal else IPC_2221_K_EXTERNAL
    return k * (temp_rise_c ** IPC_2221_EXPONENT_DT) * (area ** IPC_2221_EXPONENT_AREA)


def calculate_max_current_ipc2152(
    avg_width_mm: float,
    copper_oz: float = 1.0,
    temp_rise_c: float = 10.0,
    is_internal: bool = True
) -> float:
    """
    Max current with IPC-2152's internal/external correction applied.

    IPC-2152 replaced IPC-2221's charts after measuring that an INTERNAL trace
    in FR4 runs cooler than an external trace in still air -- the opposite of the
    2x internal derating 2221 applied. So the correction that matters here is to
    stop derating inner layers: this uses the external k for both.

    HONEST SCOPE: this is the 2221 functional form with 2152's internal/external
    finding applied, NOT a reproduction of the IPC-2152 charts. 2152's real model
    also depends on the thermal environment (copper area around the conductor,
    board thickness, adjacent planes), none of which is modelled here. Treat it
    as an estimate, and the same chart-range caveat applies.

    Args:
        avg_width_mm: Conductor width in mm
        copper_oz: Copper weight in oz
        temp_rise_c: Allowed temperature rise in °C
        is_internal: Accepted for signature parity; 2152 does not derate on it

    Returns:
        Maximum current in amps
    """
    if avg_width_mm < 0.001:
        return 0.0

    area = cross_section_mils2(avg_width_mm, copper_oz)
    return (IPC_2221_K_EXTERNAL * (temp_rise_c ** IPC_2221_EXPONENT_DT)
            * (area ** IPC_2221_EXPONENT_AREA))


# Back-compat alias. Kept so an external caller does not break, but the name
# hides which standard is being applied -- prefer the explicit functions above.
calculate_max_current_ipc = calculate_max_current_ipc2221


def stackup_copper_oz(pcb_data, layer: str, default_oz: float = 1.0) -> float:
    """Copper weight in oz for `layer`, read from the board's own stackup.

    Both plane call sites omitted copper_oz, so every board was graded at 1 oz
    even when its stackup said 2 (#489 §6) -- while the thickness sat unread in
    the same PCBData. Falls back to `default_oz` when the board has no stackup
    or the layer is absent from it.
    """
    stackup = getattr(getattr(pcb_data, 'board_info', None), 'stackup', None) or []
    for entry in stackup:
        if entry.name == layer and entry.layer_type == 'copper':
            thickness_mm = entry.thickness or 0.0
            if thickness_mm > 0:
                return thickness_mm / (OZ_TO_METERS * 1e3)
    return default_oz


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

        return _plane_metrics(path_length, avg_width, layer, copper_oz, temp_rise_c)
    except Exception:
        return None


def _plane_metrics(path_length: float, avg_width: float, layer: str,
                   copper_oz: float, temp_rise_c: float) -> Dict:
    """Resistance + both ampacity estimates for one measured plane path.

    Shared by the single-net and multi-net analyzers so they cannot drift, and so
    the inputs actually used (copper weight, temperature rise) travel WITH the
    result -- these were print-and-discard, leaving nothing to gate on (#489 §6).
    """
    is_internal = layer.startswith('In')
    return {
        'path_length': path_length,
        'avg_width': avg_width,
        'resistance': calculate_resistance(path_length, avg_width, copper_oz),
        # 'max_current' keeps its meaning (IPC-2221, the value this has always
        # reported); the 2152-corrected figure sits alongside it.
        'max_current': calculate_max_current_ipc2221(
            avg_width, copper_oz, temp_rise_c, is_internal),
        'max_current_ipc2152': calculate_max_current_ipc2152(
            avg_width, copper_oz, temp_rise_c, is_internal),
        'current_standard': 'IPC-2221',
        'copper_oz': copper_oz,
        'temp_rise_c': temp_rise_c,
        'is_internal': is_internal,
        'cross_section_mils2': cross_section_mils2(avg_width, copper_oz),
        'in_chart_range': ipc2221_area_in_range(avg_width, copper_oz),
    }


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

        return _plane_metrics(path_length, avg_width, layer, copper_oz, temp_rise_c)
    except Exception:
        return None


def analyze_power_trace_net(pcb_data, net_id: int,
                            temp_rise_c: float = 10.0) -> Optional[Dict]:
    """Ampacity of a routed power net's TRACES (#487: the IPC model was
    plane-only). A trace net's current capacity is its WORST segment -- the
    narrowest copper on the thinnest layer -- so walk every segment, price
    each at its layer's stackup copper weight, and report the bottleneck.
    IPC-2152 chart fit, with the 2221 figure alongside (same convention as
    the plane metrics). Report-only: nothing is gated on it."""
    segs = [s for s in pcb_data.segments
            if s.net_id == net_id and s.width and s.width > 0.001]
    if not segs:
        return None
    oz_by_layer: Dict[str, float] = {}
    worst = None
    total_len = 0.0
    for s in segs:
        seg_len = math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
        total_len += seg_len
        oz = oz_by_layer.get(s.layer)
        if oz is None:
            oz = stackup_copper_oz(pcb_data, s.layer)
            oz_by_layer[s.layer] = oz
        i2152 = calculate_max_current_ipc2152(
            s.width, oz, temp_rise_c, s.layer.startswith('In'))
        if worst is None or i2152 < worst[0]:
            worst = (i2152, s)
    i2152, s = worst
    oz = oz_by_layer[s.layer]
    is_internal = s.layer.startswith('In')
    return {
        'bottleneck_width': s.width,
        'bottleneck_layer': s.layer,
        'copper_oz': oz,
        'max_current': calculate_max_current_ipc2221(
            s.width, oz, temp_rise_c, is_internal),
        'max_current_ipc2152': i2152,
        'temp_rise_c': temp_rise_c,
        'trace_length': total_len,
        'segments': len(segs),
    }


def print_power_trace_ampacity(results: Dict[str, Dict]):
    """Compact per-net table for the trace-side ampacity report (#487)."""
    if not results:
        return
    print(f"\nPower trace ampacity (bottleneck segment, IPC-2152):")
    for name, r in sorted(results.items()):
        print(f"  {name}: {r['max_current_ipc2152']:.2f}A max "
              f"({r['bottleneck_width']:.3f}mm on {r['bottleneck_layer']} "
              f"{_conditions_label(r)}, IPC-2221: {r['max_current']:.2f}A)")


# Process-local collector (#487): the plane analyzers run deep inside
# create_plane (whose tuple return the GUI shares -- not extendable casually),
# while route_planes' JSON_SUMMARY is assembled in main(). The call sites note
# their results here; main consumes them into the summary instead of the
# numbers living only in stdout. Same pattern as protected_nets' accumulator.
_noted_results: Dict[str, Dict] = {}


def note_resistance_result(net_name: str, result: Optional[Dict]) -> None:
    if result and net_name:
        _noted_results[net_name] = result


def consume_resistance_results() -> Dict[str, Dict]:
    global _noted_results
    out, _noted_results = _noted_results, {}
    return out


def _conditions_label(result: Dict) -> str:
    """"(2 oz copper, 10°C rise)" from the conditions the result was computed at
    -- the header used to assert "1 oz" regardless of what was used."""
    oz = result.get('copper_oz', 1.0)
    rise = result.get('temp_rise_c', 10.0)
    oz_str = f"{oz:.2f}".rstrip('0').rstrip('.')
    return f"({oz_str} oz copper, {rise:g}°C rise)"


def print_single_net_resistance(result: Dict, net_name: str):
    """Print resistance analysis for a single-net plane."""
    if not result:
        return

    print(f"\n  Plane Resistance Analysis {_conditions_label(result)}:")
    print(f"    Path length: {result['path_length']:.1f} mm (diagonal)")
    print(f"    Avg width:   {result['avg_width']:.1f} mm")
    r_str = f"{result['resistance']*1000:.3f} mΩ" if result['resistance'] < float('inf') else "N/A"
    # ascii_safe, not a bare print: create_plane is a SHARED ENGINE function and
    # is called in-process (tests, scripts, the GUI) where route_planes.py's
    # __main__ enable_utf8_console() never ran. On a cp1252 console that made
    # this one line abort the whole pour.
    print(ascii_safe(f"    Resistance:  {r_str}"))
    # An out-of-range area is NOT a rating: say so instead of printing a bare
    # number the reader will take as one (#489 §6).
    if result.get('max_current', 0) <= 0:
        print(f"    Max current: N/A")
    elif result.get('in_chart_range', True):
        print(f"    Max current: {result['max_current']:.2f} A (IPC-2221"
              + (f"; {result['max_current_ipc2152']:.2f} A IPC-2152-corrected"
                 if result.get('is_internal') else "") + ")")
    else:
        print(f"    Max current: not rated -- {result['cross_section_mils2']:.0f} mils² "
              f"cross-section is past the IPC-2221 chart range "
              f"({IPC_2221_MAX_AREA_MILS2:.0f} mils²); the fit would say "
              f"{result['max_current']:.2f} A by extrapolation")


def print_multi_net_resistance(results: Dict[str, Dict]):
    """
    Print resistance analysis table for multi-net planes.

    Args:
        results: Dict mapping net_name -> analysis result dict
    """
    if not results:
        return

    first = next((r for r in results.values() if r), None)
    label = _conditions_label(first) if first else "(1 oz copper, 10°C rise)"
    print(f"\n  Plane Resistance Analysis {label}:")
    print(f"  {'-'*74}")
    print(ascii_safe(f"  {'Net':<25} {'Path(mm)':<10} {'AvgW(mm)':<10} {'R(mΩ)':<10} {'Imax(A)':<12}"))
    print(f"  {'-'*74}")

    any_extrapolated = any_diag = False
    for net_name, result in results.items():
        if result:
            r_str = f"{result['resistance']*1000:.3f}" if result['resistance'] < float('inf') else "N/A"
            if result['max_current'] <= 0:
                i_str = "N/A"
            elif result.get('in_chart_range', True):
                i_str = f"{result['max_current']:.2f}"
            else:
                i_str = f"{result['max_current']:.2f}*"
                any_extrapolated = True
            diag = result.get('path_basis') == 'diagonal'
            any_diag = any_diag or diag
            print(f"  {net_name + (' +' if diag else ''):<25} {result['path_length']:<10.1f} {result['avg_width']:<10.1f} "
                  f"{r_str:<10} {i_str:<12}")
        else:
            print(f"  {net_name:<25} {'N/A':<10} {'N/A':<10} {'N/A':<10} {'N/A':<12}")

    print(f"  {'-'*74}")
    print(f"  Path = longest MST route, AvgW = avg polygon width along path")
    if any_diag:
        print(f"  + no route between the net's vias (none yet: the pour runs before any routing) -- "
              f"Path is its region's bounding diagonal")
    print(f"  Imax = IPC-2221 chart fit; inner layers carry 2221's 2x derating, "
          f"which IPC-2152 overturned")
    if any_extrapolated:
        print(f"  * extrapolated past the IPC-2221 chart range "
              f"({IPC_2221_MAX_AREA_MILS2:.0f} mils²) -- not a rating")

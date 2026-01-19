"""
Plane Region Connector - Detects and routes between disconnected plane regions.

After power planes are created, regions may be effectively split due to vias and
traces from other nets cutting through the plane. This module detects disconnected
regions and routes wide, short tracks between them to ensure electrical continuity.
"""

from typing import List, Optional, Tuple, Dict, Set
from collections import deque
import math

from kicad_parser import PCBData, Via, Segment, Pad
from routing_config import GridRouteConfig, GridCoord

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))
from grid_router import GridObstacleMap, GridRouter


def find_disconnected_zone_regions(
    net_id: int,
    plane_layer: str,
    zone_bounds: Tuple[float, float, float, float],  # min_x, min_y, max_x, max_y
    pcb_data: PCBData,
    config: GridRouteConfig,
    zone_clearance: float = 0.2,
    analysis_grid_step: float = 0.5  # Coarser grid for faster analysis
) -> Tuple[List[List[Tuple[float, float]]], List[Set[Tuple[int, int]]]]:
    """
    Find disconnected regions within a zone using flood fill on a grid.

    Args:
        net_id: Net ID of the plane
        plane_layer: Layer of the plane (e.g., 'B.Cu')
        zone_bounds: (min_x, min_y, max_x, max_y) of the zone area
        pcb_data: PCB data with vias, segments, pads
        config: Routing configuration
        zone_clearance: Clearance for zone fill around obstacles
        analysis_grid_step: Grid step for connectivity analysis (coarser = faster)

    Returns:
        Tuple of:
        - List of anchor point lists (vias/pads positions) per region
        - List of grid cell sets per region (for finding closest points)
    """
    # Use a coarser grid for connectivity analysis (much faster)
    coord = GridCoord(analysis_grid_step)
    min_x, min_y, max_x, max_y = zone_bounds

    # Convert bounds to grid
    min_gx, min_gy = coord.to_grid(min_x, min_y)
    max_gx, max_gy = coord.to_grid(max_x, max_y)

    # Create a local blocked set for this analysis
    blocked: Set[Tuple[int, int]] = set()

    # Block cells for other nets' vias (with zone_clearance)
    via_block_radius = max(1, coord.to_grid_dist(config.via_size / 2 + zone_clearance))
    for via in pcb_data.vias:
        if via.net_id == net_id:
            continue
        gx, gy = coord.to_grid(via.x, via.y)
        for dx in range(-via_block_radius, via_block_radius + 1):
            for dy in range(-via_block_radius, via_block_radius + 1):
                if dx * dx + dy * dy <= via_block_radius * via_block_radius:
                    blocked.add((gx + dx, gy + dy))

    # Block cells for other nets' segments on this layer (with zone_clearance)
    for seg in pcb_data.segments:
        if seg.net_id == net_id:
            continue
        if seg.layer != plane_layer:
            continue
        seg_block_radius = max(1, coord.to_grid_dist(seg.width / 2 + zone_clearance))
        _block_segment_cells(blocked, seg, coord, seg_block_radius)

    # Block cells for other nets' pads that touch this layer (with zone_clearance)
    for pad_net_id, pads in pcb_data.pads_by_net.items():
        if pad_net_id == net_id:
            continue
        for pad in pads:
            if plane_layer not in pad.layers and '*.Cu' not in pad.layers:
                continue
            _block_pad_cells(blocked, pad, coord, zone_clearance)

    # Find anchor points (vias and pads of our net on this layer)
    anchor_points: List[Tuple[float, float]] = []
    anchor_grid_points: List[Tuple[int, int]] = []

    # Add vias of our net
    for via in pcb_data.vias:
        if via.net_id != net_id:
            continue
        if plane_layer not in via.layers:
            continue
        if min_x <= via.x <= max_x and min_y <= via.y <= max_y:
            anchor_points.append((via.x, via.y))
            anchor_grid_points.append(coord.to_grid(via.x, via.y))

    # Add pads of our net on this layer
    for pad in pcb_data.pads_by_net.get(net_id, []):
        if plane_layer not in pad.layers and '*.Cu' not in pad.layers:
            continue
        if min_x <= pad.global_x <= max_x and min_y <= pad.global_y <= max_y:
            anchor_points.append((pad.global_x, pad.global_y))
            anchor_grid_points.append(coord.to_grid(pad.global_x, pad.global_y))

    if len(anchor_points) < 2:
        # Not enough anchors to have disconnected regions
        return [anchor_points], [set(anchor_grid_points)]

    # Build map from grid position to list of anchor indices at that position
    grid_to_anchors: Dict[Tuple[int, int], List[int]] = {}
    for i, gp in enumerate(anchor_grid_points):
        if gp not in grid_to_anchors:
            grid_to_anchors[gp] = []
        grid_to_anchors[gp].append(i)

    # Union-find to group anchors
    parent = list(range(len(anchor_points)))

    def find(x):
        if parent[x] != x:
            parent[x] = find(parent[x])
        return parent[x]

    def union(x, y):
        px, py = find(x), find(y)
        if px != py:
            parent[px] = py

    # Track which anchors have been visited (by their component)
    visited_anchors: Set[int] = set()
    global_visited: Set[Tuple[int, int]] = set()

    # Process each anchor - do ONE flood fill per connected component
    for start_anchor_idx in range(len(anchor_points)):
        if start_anchor_idx in visited_anchors:
            continue

        # Start a new flood fill from this anchor
        start_gx, start_gy = anchor_grid_points[start_anchor_idx]

        # Skip if this cell was already visited by a previous flood fill
        if (start_gx, start_gy) in global_visited:
            # Find which anchor we connected to and union
            for other_idx in range(start_anchor_idx):
                if anchor_grid_points[other_idx] in global_visited:
                    union(start_anchor_idx, other_idx)
                    break
            visited_anchors.add(start_anchor_idx)
            continue

        queue = deque([(start_gx, start_gy)])
        global_visited.add((start_gx, start_gy))
        component_anchors = [start_anchor_idx]
        visited_anchors.add(start_anchor_idx)

        while queue:
            gx, gy = queue.popleft()

            # Check if we reached anchor(s) at this position
            if (gx, gy) in grid_to_anchors:
                for anchor_idx in grid_to_anchors[(gx, gy)]:
                    if anchor_idx not in visited_anchors:
                        visited_anchors.add(anchor_idx)
                        component_anchors.append(anchor_idx)
                        union(start_anchor_idx, anchor_idx)

            # Expand to neighbors (4-connected for flood fill)
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nx, ny = gx + dx, gy + dy
                if (nx, ny) in global_visited:
                    continue
                if nx < min_gx or nx > max_gx or ny < min_gy or ny > max_gy:
                    continue
                if (nx, ny) in blocked:
                    continue
                global_visited.add((nx, ny))
                queue.append((nx, ny))

    # Group anchors by their root
    groups: Dict[int, List[int]] = {}
    for i in range(len(anchor_points)):
        root = find(i)
        if root not in groups:
            groups[root] = []
        groups[root].append(i)

    # Build result lists
    region_anchors: List[List[Tuple[float, float]]] = []
    region_cells: List[Set[Tuple[int, int]]] = []

    for root, indices in groups.items():
        anchors = [anchor_points[i] for i in indices]
        cells = set(anchor_grid_points[i] for i in indices)
        region_anchors.append(anchors)
        region_cells.append(cells)

    return region_anchors, region_cells


def _block_segment_cells(
    blocked: Set[Tuple[int, int]],
    seg: Segment,
    coord: GridCoord,
    block_radius: int
):
    """Block grid cells along a segment with given radius."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

    # Bresenham-like walk along segment
    dx = abs(gx2 - gx1)
    dy = abs(gy2 - gy1)
    sx = 1 if gx1 < gx2 else -1
    sy = 1 if gy1 < gy2 else -1
    gx, gy = gx1, gy1
    radius_sq = block_radius * block_radius

    if dx > dy:
        err = dx / 2
        while gx != gx2:
            for ex in range(-block_radius, block_radius + 1):
                for ey in range(-block_radius, block_radius + 1):
                    if ex * ex + ey * ey <= radius_sq:
                        blocked.add((gx + ex, gy + ey))
            err -= dy
            if err < 0:
                gy += sy
                err += dx
            gx += sx
    else:
        err = dy / 2
        while gy != gy2:
            for ex in range(-block_radius, block_radius + 1):
                for ey in range(-block_radius, block_radius + 1):
                    if ex * ex + ey * ey <= radius_sq:
                        blocked.add((gx + ex, gy + ey))
            err -= dx
            if err < 0:
                gx += sx
                err += dy
            gy += sy

    # Block endpoint
    for ex in range(-block_radius, block_radius + 1):
        for ey in range(-block_radius, block_radius + 1):
            if ex * ex + ey * ey <= radius_sq:
                blocked.add((gx2 + ex, gy2 + ey))


def _block_pad_cells(
    blocked: Set[Tuple[int, int]],
    pad: Pad,
    coord: GridCoord,
    clearance: float
):
    """Block grid cells for a pad with clearance."""
    half_w = pad.size_x / 2 + clearance
    half_h = pad.size_y / 2 + clearance

    min_gx, _ = coord.to_grid(pad.global_x - half_w, 0)
    max_gx, _ = coord.to_grid(pad.global_x + half_w, 0)
    _, min_gy = coord.to_grid(0, pad.global_y - half_h)
    _, max_gy = coord.to_grid(0, pad.global_y + half_h)

    for gx in range(min_gx, max_gx + 1):
        for gy in range(min_gy, max_gy + 1):
            blocked.add((gx, gy))


def find_region_connection_points(
    region_anchors: List[List[Tuple[float, float]]],
    region_cells: List[Set[Tuple[int, int]]],
    coord: GridCoord
) -> List[Tuple[int, int, Tuple[float, float], Tuple[float, float], float]]:
    """
    Find MST edges connecting disconnected regions using closest anchor points.

    Args:
        region_anchors: List of anchor point lists per region
        region_cells: List of grid cell sets per region (unused for now, anchors suffice)
        coord: Grid coordinate converter

    Returns:
        List of (region_i, region_j, point_i, point_j, distance) for MST edges
    """
    n_regions = len(region_anchors)
    if n_regions < 2:
        return []

    # Find closest points between each pair of regions
    edges: List[Tuple[float, int, int, Tuple[float, float], Tuple[float, float]]] = []

    for i in range(n_regions):
        for j in range(i + 1, n_regions):
            best_dist = float('inf')
            best_pi = None
            best_pj = None

            for pi in region_anchors[i]:
                for pj in region_anchors[j]:
                    dist = math.sqrt((pi[0] - pj[0])**2 + (pi[1] - pj[1])**2)
                    if dist < best_dist:
                        best_dist = dist
                        best_pi = pi
                        best_pj = pj

            if best_pi and best_pj:
                edges.append((best_dist, i, j, best_pi, best_pj))

    # Sort by distance and build MST using Kruskal's algorithm
    edges.sort(key=lambda e: e[0])

    parent = list(range(n_regions))

    def find(x):
        if parent[x] != x:
            parent[x] = find(parent[x])
        return parent[x]

    def union(x, y):
        px, py = find(x), find(y)
        if px != py:
            parent[px] = py
            return True
        return False

    mst_edges: List[Tuple[int, int, Tuple[float, float], Tuple[float, float], float]] = []

    for dist, i, j, pi, pj in edges:
        if union(i, j):
            mst_edges.append((i, j, pi, pj, dist))
            if len(mst_edges) == n_regions - 1:
                break

    return mst_edges


# ANSI color codes
GREEN = '\033[92m'
RED = '\033[91m'
YELLOW = '\033[93m'
RESET = '\033[0m'


def route_disconnected_regions(
    net_id: int,
    net_name: str,
    plane_layer: str,
    zone_bounds: Tuple[float, float, float, float],
    pcb_data: PCBData,
    config: GridRouteConfig,
    base_obstacles: GridObstacleMap,
    layer_map: Dict[str, int],
    zone_clearance: float = 0.2,
    max_track_width: float = 2.0,
    min_track_width: float = 0.2,
    track_via_clearance: float = 0.8,
    analysis_grid_step: float = 0.5,
    max_via_reuse_radius: float = 1.5,
    verbose: bool = False
) -> Tuple[List[Dict], List[Dict], int, List[List[Tuple[float, float]]]]:
    """
    Detect and route between disconnected zone regions.

    Args:
        net_id: Net ID of the plane
        net_name: Net name for logging
        plane_layer: Layer of the plane
        zone_bounds: (min_x, min_y, max_x, max_y) of the zone area
        pcb_data: PCB data
        config: Routing configuration
        base_obstacles: Pre-built obstacle map (will be modified with new routes)
        layer_map: Dict mapping layer names to indices
        zone_clearance: Clearance for zone fill
        max_track_width: Maximum track width for connections (mm)
        min_track_width: Minimum track width for connections (mm)
        track_via_clearance: Clearance from track to other nets' vias
        max_via_reuse_radius: Max distance to reuse existing via/pad (mm)
        verbose: Print debug info

    Returns:
        Tuple of (list of segment dicts, list of via dicts, number of routes added, list of route paths for debug)
    """
    coord = GridCoord(config.grid_step)

    # Find disconnected regions
    region_anchors, region_cells = find_disconnected_zone_regions(
        net_id, plane_layer, zone_bounds, pcb_data, config, zone_clearance,
        analysis_grid_step
    )

    n_regions = len(region_anchors)
    if n_regions < 2:
        n_anchors = len(region_anchors[0]) if region_anchors else 0
        print(f"  Zone is fully connected ({n_anchors} anchors in 1 region)")
        return [], [], 0, []

    # Count total anchors per region for display
    total_anchors = sum(len(anchors) for anchors in region_anchors)
    print(f"  Found {n_regions} disconnected regions ({total_anchors} total anchors)")
    if verbose:
        for i, anchors in enumerate(region_anchors):
            print(f"    Region {i}: {len(anchors)} anchor(s)")

    # Find MST edges to connect regions
    mst_edges = find_region_connection_points(region_anchors, region_cells, coord)
    print(f"  Routing {len(mst_edges)} connection(s) to join regions...")

    # Get plane layer index and routing layers from layer_map
    plane_layer_idx = layer_map.get(plane_layer)
    if plane_layer_idx is None:
        print(f"  Error: plane_layer '{plane_layer}' not in layer_map")
        return [], [], 0, []
    routing_layers = list(layer_map.keys())

    # Build list of existing vias and through-hole pads from this net (can be reused as layer transitions)
    net_vias: List[Tuple[float, float]] = [(v.x, v.y) for v in pcb_data.vias if v.net_id == net_id]
    # Add through-hole pads from this net (they connect all layers like vias)
    if net_id in pcb_data.pads_by_net:
        for pad in pcb_data.pads_by_net[net_id]:
            if '*.Cu' in pad.layers:  # Through-hole pad
                net_vias.append((pad.global_x, pad.global_y))

    segments: List[Dict] = []
    vias: List[Dict] = []
    routes_added = 0
    routes_failed = 0
    previous_routes: List[List[Tuple[float, float]]] = []

    for edge_idx, (region_i, region_j, point_i, point_j, dist) in enumerate(mst_edges):
        # Progress indicator
        print(f"    [{edge_idx+1}/{len(mst_edges)}] Region {region_i} -> {region_j} ({dist:.1f}mm)...", end=" ", flush=True)

        # Compute maximum safe track width for this connection
        track_width = compute_connection_track_width(
            point_i, point_j, plane_layer, net_id, pcb_data, config,
            zone_clearance, max_track_width, min_track_width
        )

        # Route the connection using pre-built obstacles
        result = route_plane_connection_wide(
            point_i, point_j,
            plane_layer_idx=plane_layer_idx,
            routing_layers=routing_layers,
            base_obstacles=base_obstacles,
            config=config,
            net_vias=net_vias,
            max_via_reuse_radius=max_via_reuse_radius,
            verbose=verbose
        )

        if result is None:
            print(f"{RED}FAILED{RESET}")
            routes_failed += 1
            if verbose:
                print(f"      From ({point_i[0]:.2f}, {point_i[1]:.2f}) to ({point_j[0]:.2f}, {point_j[1]:.2f})")
            continue

        route_points, via_positions = result

        # Calculate actual route length
        route_length = 0.0
        for k in range(len(route_points) - 1):
            p1, p2 = route_points[k], route_points[k + 1]
            route_length += math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

        # Count layers used
        layers_used = set(p[2] for p in route_points)
        layer_info = f", {len(layers_used)} layer(s)" if len(layers_used) > 1 else ""
        via_info = f", {len(via_positions)} via(s)" if via_positions else ""

        print(f"{GREEN}OK{RESET} width={track_width:.2f}mm, length={route_length:.1f}mm, {len(route_points)-1} seg(s){layer_info}{via_info}")

        # Incrementally add this route to base obstacles for subsequent routes
        add_route_to_obstacles(
            base_obstacles, route_points, via_positions, layer_map,
            track_width, config.clearance, track_via_clearance, config
        )

        # Keep track of route for debug output
        previous_routes.append([(p[0], p[1]) for p in route_points])

        # Generate segments from route points (with layer info)
        for k in range(len(route_points) - 1):
            p1, p2 = route_points[k], route_points[k + 1]
            # Only create segment if on same layer
            if p1[2] == p2[2]:
                segments.append({
                    'start': (p1[0], p1[1]),
                    'end': (p2[0], p2[1]),
                    'width': track_width,
                    'layer': p1[2],
                    'net_id': net_id
                })

        # Generate vias and add to net_vias for reuse by subsequent routes
        for vx, vy in via_positions:
            vias.append({
                'x': vx,
                'y': vy,
                'size': config.via_size,
                'drill': config.via_drill,
                'net_id': net_id
            })
            net_vias.append((vx, vy))  # Available for reuse

        routes_added += 1

    # Summary for this net
    if routes_failed > 0:
        print(f"  {YELLOW}Result: {routes_added}/{len(mst_edges)} routes succeeded, {routes_failed} failed{RESET}")
    elif routes_added > 0:
        print(f"  {GREEN}Result: All {routes_added} route(s) succeeded{RESET}")

    return segments, vias, routes_added, previous_routes


def compute_connection_track_width(
    point_a: Tuple[float, float],
    point_b: Tuple[float, float],
    plane_layer: str,
    net_id: int,
    pcb_data: PCBData,
    config: GridRouteConfig,
    zone_clearance: float,
    max_track_width: float,
    min_track_width: float
) -> float:
    """
    Compute the maximum track width that can be used for a connection
    while maintaining clearance from obstacles.

    Uses the corridor between points to find the narrowest gap.
    """
    # Start with max and reduce based on nearby obstacles
    track_width = max_track_width

    # Check distance to other nets' vias along the path
    # Sample points along the line
    dx = point_b[0] - point_a[0]
    dy = point_b[1] - point_a[1]
    length = math.sqrt(dx * dx + dy * dy)

    if length < 0.001:
        return min_track_width

    # Normalize direction
    ux, uy = dx / length, dy / length
    # Perpendicular direction
    px, py = -uy, ux

    n_samples = max(2, int(length / config.grid_step))

    for via in pcb_data.vias:
        if via.net_id == net_id:
            continue

        # Find closest point on line segment to via
        vx, vy = via.x, via.y

        # Vector from point_a to via
        ax, ay = vx - point_a[0], vy - point_a[1]

        # Project onto line direction
        t = (ax * ux + ay * uy) / length
        t = max(0.0, min(1.0, t))

        # Closest point on segment
        cx = point_a[0] + t * dx
        cy = point_a[1] + t * dy

        # Distance from via to closest point
        dist = math.sqrt((vx - cx)**2 + (vy - cy)**2)

        # Required clearance: via_size/2 + track_width/2 + zone_clearance
        # So max track_width = 2 * (dist - via_size/2 - zone_clearance)
        via_radius = config.via_size / 2
        max_tw_for_via = 2 * (dist - via_radius - zone_clearance)

        if max_tw_for_via < track_width:
            track_width = max_tw_for_via

    # Check distance to other nets' segments on this layer
    for seg in pcb_data.segments:
        if seg.net_id == net_id:
            continue
        if seg.layer != plane_layer:
            continue

        # Find minimum distance between two line segments
        dist = _segment_to_segment_distance(
            point_a, point_b,
            (seg.start_x, seg.start_y), (seg.end_x, seg.end_y)
        )

        # Required clearance: seg.width/2 + track_width/2 + zone_clearance
        max_tw_for_seg = 2 * (dist - seg.width / 2 - zone_clearance)

        if max_tw_for_seg < track_width:
            track_width = max_tw_for_seg

    # Check distance to other nets' pads on this layer
    for pad_net_id, pads in pcb_data.pads_by_net.items():
        if pad_net_id == net_id:
            continue
        for pad in pads:
            if plane_layer not in pad.layers and '*.Cu' not in pad.layers:
                continue

            # Find closest point on our line segment to pad center
            px, py = pad.global_x, pad.global_y
            ax, ay = px - point_a[0], py - point_a[1]
            t = (ax * ux + ay * uy) / length
            t = max(0.0, min(1.0, t))
            cx = point_a[0] + t * dx
            cy = point_a[1] + t * dy

            # Distance from pad center to closest point on line
            dist = math.sqrt((px - cx)**2 + (py - cy)**2)

            # Approximate pad as circle with radius = max(size_x, size_y) / 2
            pad_radius = max(pad.size_x, pad.size_y) / 2
            max_tw_for_pad = 2 * (dist - pad_radius - zone_clearance)

            if max_tw_for_pad < track_width:
                track_width = max_tw_for_pad

    # Clamp to bounds
    track_width = max(min_track_width, min(max_track_width, track_width))

    return track_width


def _segment_to_segment_distance(
    a1: Tuple[float, float], a2: Tuple[float, float],
    b1: Tuple[float, float], b2: Tuple[float, float]
) -> float:
    """Compute minimum distance between two line segments."""
    # Check all endpoint-to-segment and endpoint-to-endpoint distances
    distances = [
        _point_to_segment_distance(a1, b1, b2),
        _point_to_segment_distance(a2, b1, b2),
        _point_to_segment_distance(b1, a1, a2),
        _point_to_segment_distance(b2, a1, a2),
    ]
    return min(distances)


def _point_to_segment_distance(
    p: Tuple[float, float],
    s1: Tuple[float, float], s2: Tuple[float, float]
) -> float:
    """Compute distance from point p to line segment s1-s2."""
    dx = s2[0] - s1[0]
    dy = s2[1] - s1[1]
    length_sq = dx * dx + dy * dy

    if length_sq < 1e-10:
        # Degenerate segment
        return math.sqrt((p[0] - s1[0])**2 + (p[1] - s1[1])**2)

    # Project p onto line
    t = ((p[0] - s1[0]) * dx + (p[1] - s1[1]) * dy) / length_sq
    t = max(0.0, min(1.0, t))

    # Closest point on segment
    cx = s1[0] + t * dx
    cy = s1[1] + t * dy

    return math.sqrt((p[0] - cx)**2 + (p[1] - cy)**2)


def build_base_obstacles(
    exclude_net_ids: Set[int],
    routing_layers: List[str],
    pcb_data: PCBData,
    config: GridRouteConfig,
    track_width: float,
    track_via_clearance: float,
    hole_to_hole_clearance: float = 0.5,
    proximity_radius: float = 3.0,
    proximity_cost: float = 2.0
) -> Tuple[GridObstacleMap, Dict[str, int]]:
    """
    Build base obstacle map with all static obstacles, excluding specified nets.

    Args:
        exclude_net_ids: Set of net IDs to exclude from obstacles (the plane nets being routed)
        routing_layers: List of layer names available for routing
        pcb_data: PCB data
        config: Routing configuration
        track_width: Track width for clearance calculations
        track_via_clearance: Clearance from tracks to vias
        hole_to_hole_clearance: Minimum clearance between drill holes (mm)
        proximity_radius: Radius for proximity costs
        proximity_cost: Cost multiplier for proximity

    Returns:
        Tuple of (obstacle_map, layer_map)
    """
    coord = GridCoord(config.grid_step)

    num_layers = len(routing_layers)
    layer_map = {layer: idx for idx, layer in enumerate(routing_layers)}

    obstacles = GridObstacleMap(num_layers)

    # Block other nets' vias as hard obstacles on ALL layers (vias are through-hole)
    via_radius = coord.to_grid_dist(track_via_clearance)
    for via in pcb_data.vias:
        if via.net_id in exclude_net_ids:
            continue
        gx, gy = coord.to_grid(via.x, via.y)
        for layer_idx in range(num_layers):
            for ex in range(-via_radius, via_radius + 1):
                for ey in range(-via_radius, via_radius + 1):
                    if ex * ex + ey * ey <= via_radius * via_radius:
                        obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)

    # Add proximity costs around other nets' vias (on all layers)
    proximity_radius_grid = coord.to_grid_dist(proximity_radius)
    proximity_cost_grid = int(proximity_cost * 1000 / config.grid_step)

    all_other_vias = []
    for via in pcb_data.vias:
        if via.net_id not in exclude_net_ids:
            gx, gy = coord.to_grid(via.x, via.y)
            all_other_vias.append((gx, gy))

    if all_other_vias:
        obstacles.add_stub_proximity_costs_batch(
            all_other_vias,
            proximity_radius_grid,
            proximity_cost_grid,
            False
        )

    # Block via placement near OTHER nets' holes for hole-to-hole clearance
    # Our own net's holes can be used as layer transitions, so don't block them
    hole_clearance_grid = max(1, coord.to_grid_dist(hole_to_hole_clearance + config.via_drill / 2))

    # Block via placement near other nets' vias
    for via in pcb_data.vias:
        if via.net_id in exclude_net_ids:
            continue  # Don't block near our own vias
        gx, gy = coord.to_grid(via.x, via.y)
        for ex in range(-hole_clearance_grid, hole_clearance_grid + 1):
            for ey in range(-hole_clearance_grid, hole_clearance_grid + 1):
                if ex * ex + ey * ey <= hole_clearance_grid * hole_clearance_grid:
                    obstacles.add_blocked_via(gx + ex, gy + ey)

    # Block via placement near other nets' through-hole pad holes
    for pad_net_id, pads in pcb_data.pads_by_net.items():
        if pad_net_id in exclude_net_ids:
            continue  # Don't block near our own pads
        for pad in pads:
            if '*.Cu' in pad.layers and pad.drill > 0:  # Through-hole pad with drill
                gx, gy = coord.to_grid(pad.global_x, pad.global_y)
                # Use pad's drill size for clearance calculation
                pad_hole_clearance_grid = max(1, coord.to_grid_dist(hole_to_hole_clearance + pad.drill / 2))
                for ex in range(-pad_hole_clearance_grid, pad_hole_clearance_grid + 1):
                    for ey in range(-pad_hole_clearance_grid, pad_hole_clearance_grid + 1):
                        if ex * ex + ey * ey <= pad_hole_clearance_grid * pad_hole_clearance_grid:
                            obstacles.add_blocked_via(gx + ex, gy + ey)

    # Block existing segments from other nets on each layer
    # Also block vias along segments (vias span all layers, so can't place via where segment exists)
    via_expansion_mm = config.via_size / 2 + config.clearance
    via_expansion_grid = max(1, coord.to_grid_dist(via_expansion_mm))

    for seg in pcb_data.segments:
        if seg.net_id in exclude_net_ids:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        seg_expansion_mm = track_width / 2 + seg.width / 2 + config.clearance
        seg_expansion_grid = max(1, coord.to_grid_dist(seg_expansion_mm))
        _block_segment_obstacle(obstacles, seg, coord, layer_idx, seg_expansion_grid)
        # Also block vias along this segment
        _block_segment_via_obstacle(obstacles, seg, coord, via_expansion_grid)

    # Block pads from non-plane nets on their respective layers
    # (plane net pads are excluded here - they're anchors; other plane nets' pads
    # will be blocked per-net in route_disconnected_regions)
    for pad_net_id, pads in pcb_data.pads_by_net.items():
        if pad_net_id in exclude_net_ids:
            continue
        for pad in pads:
            # Determine which layers this pad is on
            pad_layers_on = []
            if '*.Cu' in pad.layers:
                pad_layers_on = list(range(num_layers))
            else:
                for pl in pad.layers:
                    if pl in layer_map:
                        pad_layers_on.append(layer_map[pl])

            if not pad_layers_on:
                continue

            # Block rectangular area around pad with clearance
            pad_expansion_mm = track_width / 2 + config.clearance
            half_w = pad.size_x / 2 + pad_expansion_mm
            half_h = pad.size_y / 2 + pad_expansion_mm
            min_gx, _ = coord.to_grid(pad.global_x - half_w, 0)
            max_gx, _ = coord.to_grid(pad.global_x + half_w, 0)
            _, min_gy = coord.to_grid(0, pad.global_y - half_h)
            _, max_gy = coord.to_grid(0, pad.global_y + half_h)
            for layer_idx in pad_layers_on:
                for gx in range(min_gx, max_gx + 1):
                    for gy in range(min_gy, max_gy + 1):
                        obstacles.add_blocked_cell(gx, gy, layer_idx)
            # Also block vias in pad area
            for gx in range(min_gx, max_gx + 1):
                for gy in range(min_gy, max_gy + 1):
                    obstacles.add_blocked_via(gx, gy)

    return obstacles, layer_map


def add_route_to_obstacles(
    obstacles: GridObstacleMap,
    route_points: List[Tuple[float, float, str]],
    via_positions: List[Tuple[float, float]],
    layer_map: Dict[str, int],
    track_width: float,
    clearance: float,
    via_clearance: float,
    config: GridRouteConfig
):
    """Add a completed route (segments and vias) to the obstacle map for subsequent routes to avoid."""
    coord = GridCoord(config.grid_step)
    num_layers = len(layer_map)

    # Block segments on their layers and also block vias along segments
    route_expansion_mm = track_width + clearance
    route_expansion_grid = max(1, coord.to_grid_dist(route_expansion_mm))
    via_seg_expansion_mm = config.via_size / 2 + clearance
    via_seg_expansion_grid = max(1, coord.to_grid_dist(via_seg_expansion_mm))

    for i in range(len(route_points) - 1):
        p1, p2 = route_points[i], route_points[i + 1]
        # Only block if on same layer (segments)
        if p1[2] == p2[2]:
            layer_idx = layer_map.get(p1[2], 0)
            gx1, gy1 = coord.to_grid(p1[0], p1[1])
            gx2, gy2 = coord.to_grid(p2[0], p2[1])
            _block_line_cells(obstacles, gx1, gy1, gx2, gy2, layer_idx, route_expansion_grid)
            # Also block vias along this segment
            _block_line_vias(obstacles, gx1, gy1, gx2, gy2, via_seg_expansion_grid)

    # Block vias on all layers (through-hole) and also block via placement
    via_radius = coord.to_grid_dist(via_clearance)
    for vx, vy in via_positions:
        gx, gy = coord.to_grid(vx, vy)
        for layer_idx in range(num_layers):
            for ex in range(-via_radius, via_radius + 1):
                for ey in range(-via_radius, via_radius + 1):
                    if ex * ex + ey * ey <= via_radius * via_radius:
                        obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
        # Also block via placement at this location
        for ex in range(-via_radius, via_radius + 1):
            for ey in range(-via_radius, via_radius + 1):
                if ex * ex + ey * ey <= via_radius * via_radius:
                    obstacles.add_blocked_via(gx + ex, gy + ey)


def route_plane_connection_wide(
    via_a: Tuple[float, float],
    via_b: Tuple[float, float],
    plane_layer_idx: int,
    routing_layers: List[str],
    base_obstacles: GridObstacleMap,
    config: GridRouteConfig,
    net_vias: List[Tuple[float, float]],
    max_via_reuse_radius: float = 0.5,
    max_iterations: int = 200000,
    verbose: bool = False
) -> Optional[Tuple[List[Tuple[float, float, str]], List[Tuple[float, float]]]]:
    """
    Route a wide trace between two points using pre-built obstacles.

    Args:
        via_a, via_b: Start and end points
        plane_layer_idx: Index of the plane layer in routing_layers
        routing_layers: List of layer names for routing
        base_obstacles: Pre-built obstacle map (will be cloned)
        config: Routing configuration
        net_vias: List of existing via positions from this net (can be reused)
        max_via_reuse_radius: Max distance to reuse an existing via (mm)
        max_iterations: Max routing iterations
        verbose: Print debug info

    Returns:
        Tuple of:
        - List of (x, y, layer) tuples for route segments
        - List of (x, y) tuples for NEW via positions (excluding reused ones)
        Or None if routing failed.
    """
    coord = GridCoord(config.grid_step)

    # Clone the base obstacles for this route (clone_fresh clears source/target cells)
    obstacles = base_obstacles.clone_fresh()

    # Set up source and target - on the plane layer
    via_a_gx, via_a_gy = coord.to_grid(via_a[0], via_a[1])
    via_b_gx, via_b_gy = coord.to_grid(via_b[0], via_b[1])

    # Make sure source and target are not blocked on plane layer
    obstacles.add_source_target_cell(via_a_gx, via_a_gy, plane_layer_idx)
    obstacles.add_source_target_cell(via_b_gx, via_b_gy, plane_layer_idx)

    sources = [(via_a_gx, via_a_gy, plane_layer_idx)]
    targets = [(via_b_gx, via_b_gy, plane_layer_idx)]

    # Create router and find path
    router = GridRouter(
        via_cost=config.via_cost * 1000,
        h_weight=config.heuristic_weight,
        turn_cost=config.turn_cost,
        via_proximity_cost=0
    )

    path, iterations, _ = router.route_with_frontier(
        obstacles, sources, targets, max_iterations,
        False,  # collinear_vias
        0,      # via_exclusion_radius
        None,   # start_direction
        None,   # end_direction
        0       # direction_steps
    )

    if path is None:
        if verbose:
            print(f"    Route failed after {iterations} iterations")
        return None

    if verbose:
        print(f"    Route found in {iterations} iterations, {len(path)} points")

    # Convert path to float coordinates with layer info
    route_points: List[Tuple[float, float, str]] = []
    for i, (gx, gy, layer_idx) in enumerate(path):
        x, y = coord.to_float(gx, gy)
        layer_name = routing_layers[layer_idx]
        route_points.append((x, y, layer_name))

    # Find layer transitions (vias) and check for reuse opportunities
    # For each via, we'll either keep it or replace the transition with a route to an existing via
    reuse_radius_sq = max_via_reuse_radius * max_via_reuse_radius
    new_via_positions: List[Tuple[float, float]] = []

    # Find all layer transition indices
    layer_transitions: List[int] = []  # Index where layer changes (the "after" point)
    for i in range(1, len(route_points)):
        if route_points[i][2] != route_points[i-1][2]:
            layer_transitions.append(i)

    # Process transitions from end to start (so indices stay valid when we modify)
    for trans_idx in reversed(layer_transitions):
        # The via is at the position of the point before the transition
        via_x, via_y = route_points[trans_idx - 1][0], route_points[trans_idx - 1][1]
        layer_before = route_points[trans_idx - 1][2]
        layer_after = route_points[trans_idx][2]

        # Check if there's an existing via from our net nearby to reuse
        reuse_pos = None
        for ex_vx, ex_vy in net_vias:
            dist_sq = (via_x - ex_vx) ** 2 + (via_y - ex_vy) ** 2
            if dist_sq <= reuse_radius_sq:
                reuse_pos = (ex_vx, ex_vy)
                break
        # Also check against new vias we've already decided to add
        if reuse_pos is None:
            for nv_x, nv_y in new_via_positions:
                dist_sq = (via_x - nv_x) ** 2 + (via_y - nv_y) ** 2
                if dist_sq <= reuse_radius_sq:
                    reuse_pos = (nv_x, nv_y)
                    break

        if reuse_pos and (reuse_pos[0] != via_x or reuse_pos[1] != via_y):
            # Reuse existing via - replace the transition region with direct route to existing via
            # Find the extent of the "transition region" - points close to the via on each layer
            # We want to find the last "far" point before the via and first "far" point after

            # Search backward to find where the route approaches the via
            start_of_approach = trans_idx - 1
            while start_of_approach > 0:
                prev_pt = route_points[start_of_approach - 1]
                if prev_pt[2] != layer_before:
                    break  # Different layer, stop
                dist = math.sqrt((prev_pt[0] - via_x)**2 + (prev_pt[1] - via_y)**2)
                if dist > max_via_reuse_radius:
                    break  # Far enough, this is where real route is
                start_of_approach -= 1

            # Search forward to find where the route leaves the via area
            end_of_departure = trans_idx
            while end_of_departure < len(route_points) - 1:
                next_pt = route_points[end_of_departure + 1]
                if next_pt[2] != layer_after:
                    break  # Different layer, stop
                dist = math.sqrt((next_pt[0] - via_x)**2 + (next_pt[1] - via_y)**2)
                if dist > max_via_reuse_radius:
                    break  # Far enough, this is where real route continues
                end_of_departure += 1

            # Replace the transition region with direct connection to reused via
            # Keep: route_points[0:start_of_approach+1] + [via on layer_before, via on layer_after] + route_points[end_of_departure:]
            new_route = (
                route_points[:start_of_approach + 1] +
                [(reuse_pos[0], reuse_pos[1], layer_before),
                 (reuse_pos[0], reuse_pos[1], layer_after)] +
                route_points[end_of_departure + 1:]
            )
            route_points = new_route
            # Don't add a new via - we're reusing an existing one
        else:
            # Add new via at original position (or it's already at reuse_pos)
            if reuse_pos is None:
                new_via_positions.append((via_x, via_y))
            # else: already at the reuse position, no new via needed

    # Remove duplicate consecutive points (same x,y) keeping layer transitions
    cleaned_points: List[Tuple[float, float, str]] = []
    for pt in route_points:
        if cleaned_points:
            last = cleaned_points[-1]
            if pt[0] == last[0] and pt[1] == last[1] and pt[2] == last[2]:
                continue  # Skip duplicate
        cleaned_points.append(pt)

    return cleaned_points, new_via_positions


def _block_segment_obstacle(
    obstacles: GridObstacleMap,
    seg: Segment,
    coord: GridCoord,
    layer_idx: int,
    expansion_grid: int
):
    """Block cells along a segment in the obstacle map."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
    _block_line_cells(obstacles, gx1, gy1, gx2, gy2, layer_idx, expansion_grid)


def _block_segment_via_obstacle(
    obstacles: GridObstacleMap,
    seg: Segment,
    coord: GridCoord,
    expansion_grid: int
):
    """Block via placement along a segment (vias span all layers)."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
    _block_line_vias(obstacles, gx1, gy1, gx2, gy2, expansion_grid)


def _block_line_vias(
    obstacles: GridObstacleMap,
    gx1: int, gy1: int,
    gx2: int, gy2: int,
    expansion_grid: int
):
    """Block via placement along a line with given expansion radius."""
    dx = abs(gx2 - gx1)
    dy = abs(gy2 - gy1)
    sx = 1 if gx1 < gx2 else -1
    sy = 1 if gy1 < gy2 else -1
    gx, gy = gx1, gy1
    radius_sq = expansion_grid * expansion_grid

    if dx > dy:
        err = dx / 2
        while gx != gx2:
            for ex in range(-expansion_grid, expansion_grid + 1):
                for ey in range(-expansion_grid, expansion_grid + 1):
                    if ex * ex + ey * ey <= radius_sq:
                        obstacles.add_blocked_via(gx + ex, gy + ey)
            err -= dy
            if err < 0:
                gy += sy
                err += dx
            gx += sx
    else:
        err = dy / 2
        while gy != gy2:
            for ex in range(-expansion_grid, expansion_grid + 1):
                for ey in range(-expansion_grid, expansion_grid + 1):
                    if ex * ex + ey * ey <= radius_sq:
                        obstacles.add_blocked_via(gx + ex, gy + ey)
            err -= dx
            if err < 0:
                gx += sx
                err += dy
            gy += sy

    # Block endpoint
    for ex in range(-expansion_grid, expansion_grid + 1):
        for ey in range(-expansion_grid, expansion_grid + 1):
            if ex * ex + ey * ey <= radius_sq:
                obstacles.add_blocked_via(gx2 + ex, gy2 + ey)


def _block_line_cells(
    obstacles: GridObstacleMap,
    gx1: int, gy1: int,
    gx2: int, gy2: int,
    layer_idx: int,
    expansion_grid: int
):
    """Block cells along a line with given expansion radius."""
    dx = abs(gx2 - gx1)
    dy = abs(gy2 - gy1)
    sx = 1 if gx1 < gx2 else -1
    sy = 1 if gy1 < gy2 else -1
    gx, gy = gx1, gy1
    radius_sq = expansion_grid * expansion_grid

    if dx > dy:
        err = dx / 2
        while gx != gx2:
            for ex in range(-expansion_grid, expansion_grid + 1):
                for ey in range(-expansion_grid, expansion_grid + 1):
                    if ex * ex + ey * ey <= radius_sq:
                        obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
            err -= dy
            if err < 0:
                gy += sy
                err += dx
            gx += sx
    else:
        err = dy / 2
        while gy != gy2:
            for ex in range(-expansion_grid, expansion_grid + 1):
                for ey in range(-expansion_grid, expansion_grid + 1):
                    if ex * ex + ey * ey <= radius_sq:
                        obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
            err -= dx
            if err < 0:
                gx += sx
                err += dy
            gy += sy

    # Block endpoint
    for ex in range(-expansion_grid, expansion_grid + 1):
        for ey in range(-expansion_grid, expansion_grid + 1):
            if ex * ex + ey * ey <= radius_sq:
                obstacles.add_blocked_cell(gx2 + ex, gy2 + ey, layer_idx)

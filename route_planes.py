"""
Copper Plane Generator - Creates filled zones with via stitching to net pads.

Creates a solid copper zone on a specified layer, places vias near target pads,
and routes short traces to connect vias to pads when direct placement is blocked.

Usage:
    python route_planes.py input.kicad_pcb output.kicad_pcb --net GND --layer B.Cu
"""

import sys
import os
import argparse
from typing import List, Optional, Tuple, Dict, Set
from dataclasses import dataclass

# Run startup checks first (validates numpy, scipy, shapely are installed)
from startup_checks import run_all_checks
run_all_checks()

# These imports are guaranteed to work after startup_checks passes
import numpy as np

from kicad_parser import parse_kicad_pcb, PCBData, Pad, Via, Segment
from kicad_writer import generate_zone_sexpr, generate_gr_line_sexpr
from routing_config import GridRouteConfig, GridCoord
from route import batch_route
from obstacle_cache import ViaPlacementObstacleData, precompute_via_placement_obstacles
from connectivity import compute_mst_segments

# Import from new refactored modules
from plane_io import (
    ZoneInfo,
    extract_zones,
    check_existing_zones,
    resolve_net_id,
    write_plane_output
)
from plane_obstacle_builder import (
    identify_target_pads,
    build_via_obstacle_map,
    build_routing_obstacle_map,
    block_via_position,
    _add_segment_routing_obstacle
)
from plane_blocker_detection import (
    ViaPlacementResult,
    find_via_position_blocker,
    find_route_blocker_from_frontier,
    try_place_via_with_ripup
)
from plane_zone_geometry import (
    compute_zone_boundaries,
    find_polygon_groups,
    sample_route_for_voronoi
)
from terminal_colors import GREEN, RED, RESET

# Import Rust router (startup_checks ensures it's available and up-to-date)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))
from grid_router import GridObstacleMap, GridRouter

# Plane resistance calculations
from plane_resistance import (
    analyze_single_net_plane,
    analyze_multi_net_plane,
    print_single_net_resistance,
    print_multi_net_resistance
)


def find_existing_via_nearby(
    pad: Pad,
    existing_vias: List[Tuple[float, float]],
    max_search_radius: float
) -> Optional[Tuple[float, float]]:
    """
    Find an existing via on the target net within search radius of the pad.

    Args:
        pad: The pad to connect
        existing_vias: List of (x, y) positions of existing vias on target net
        max_search_radius: Maximum distance to search

    Returns:
        (x, y) position of nearest existing via, or None if none found
    """
    best_via = None
    best_dist_sq = max_search_radius * max_search_radius

    for vx, vy in existing_vias:
        dx = vx - pad.global_x
        dy = vy - pad.global_y
        dist_sq = dx * dx + dy * dy
        if dist_sq <= best_dist_sq:
            best_dist_sq = dist_sq
            best_via = (vx, vy)

    return best_via


def find_via_position(
    pad: Pad,
    obstacles: GridObstacleMap,
    coord: GridCoord,
    max_search_radius: float,
    routing_obstacles: GridObstacleMap = None,
    config: GridRouteConfig = None,
    pad_layer: str = None,
    net_id: int = None,
    verbose: bool = False,
    failed_route_positions: Optional[Set[Tuple[int, int]]] = None,
    pending_pads: Optional[List[Dict]] = None
) -> Optional[Tuple[float, float]]:
    """
    Find the closest valid position for a via near a pad.

    Uses spiral search outward from pad center, checking clearances.
    If routing_obstacles is provided, also verifies that A* can route from the via to the pad.

    Args:
        pad: Target pad
        obstacles: Via obstacle map
        coord: Grid coordinate converter
        max_search_radius: Maximum distance to search
        routing_obstacles: Optional routing obstacle map for routability check
        config: Optional routing config (required if routing_obstacles provided)
        pad_layer: Layer for routing (required if routing_obstacles provided)
        net_id: Net ID for routing (required if routing_obstacles provided)
        verbose: Print debug output on failure
        failed_route_positions: Optional set of (gx, gy) positions where routing previously
            failed. Positions within 2x via-size will be skipped. Failed positions from
            this call will be added to the set.
        pending_pads: Optional list of pad_info dicts for pads that still need vias.
            Via positions too close to these pads' boundaries will be skipped to ensure
            routes can still reach them.

    Returns:
        (x, y) position for via, or None if no valid position found
    """
    pad_gx, pad_gy = coord.to_grid(pad.global_x, pad.global_y)

    # Try pad center first - if not blocked, use it (no routing needed)
    if not obstacles.is_via_blocked(pad_gx, pad_gy):
        return (pad.global_x, pad.global_y)

    # Spiral search outward
    max_radius_grid = coord.to_grid_dist(max_search_radius)

    # Skip radius: 2x via-size in grid units
    skip_radius_sq = 0
    if failed_route_positions is not None and config:
        skip_radius = coord.to_grid_dist(config.via_size * 2)
        skip_radius_sq = skip_radius * skip_radius

    # Precompute pending pad exclusion zones (rectangular, in grid coordinates)
    # Each zone ensures a route can still reach the pad from any direction
    # Zone extends: pad_half_size + 1.5*via_size + clearance from pad center
    # (1.5*via_size = via_size/2 for placed via + via_size/2 for future via + via_size/2 extra margin)
    pending_pad_zones = []  # List of (min_gx, min_gy, max_gx, max_gy)
    if pending_pads and config:
        margin = 1.5 * config.via_size + config.clearance
        for pad_info in pending_pads:
            p = pad_info['pad']
            half_w = p.size_x / 2 + margin
            half_h = p.size_y / 2 + margin
            min_gx = coord.to_grid(p.global_x - half_w, 0)[0]
            max_gx = coord.to_grid(p.global_x + half_w, 0)[0]
            min_gy = coord.to_grid(0, p.global_y - half_h)[1]
            max_gy = coord.to_grid(0, p.global_y + half_h)[1]
            pending_pad_zones.append((min_gx, min_gy, max_gx, max_gy))

    # Collect all valid via positions, sorted by distance
    valid_positions = []

    # Search in expanding rings
    for radius in range(1, max_radius_grid + 1):
        # Check all points at this Manhattan radius
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                # Skip if not on the ring edge
                if abs(dx) != radius and abs(dy) != radius:
                    continue

                gx, gy = pad_gx + dx, pad_gy + dy

                # Check if via can be placed here
                if obstacles.is_via_blocked(gx, gy):
                    continue

                # Skip if too close to a previously failed route position
                if failed_route_positions and skip_radius_sq > 0:
                    too_close = False
                    for failed_gx, failed_gy in failed_route_positions:
                        fdx = gx - failed_gx
                        fdy = gy - failed_gy
                        if fdx * fdx + fdy * fdy <= skip_radius_sq:
                            too_close = True
                            break
                    if too_close:
                        continue

                # Skip if inside a pending pad's exclusion zone
                if pending_pad_zones:
                    in_zone = False
                    for zone_idx, (min_gx, min_gy, max_gx, max_gy) in enumerate(pending_pad_zones):
                        if min_gx <= gx <= max_gx and min_gy <= gy <= max_gy:
                            in_zone = True
                            if verbose:
                                print(f"\n    DEBUG: Skipping ({gx},{gy}) - inside zone {zone_idx}: ({min_gx},{min_gy})-({max_gx},{max_gy})")
                            break
                    if in_zone:
                        continue

                # Valid via position - add to list with distance
                dist_sq = dx * dx + dy * dy
                via_pos = coord.to_float(gx, gy)
                valid_positions.append((dist_sq, via_pos, gx, gy))

    # Sort by distance (closest first)
    valid_positions.sort(key=lambda x: x[0])

    # If no routing check needed, return closest valid position
    if routing_obstacles is None or config is None:
        if valid_positions:
            return valid_positions[0][1]
        if verbose:
            print(f"\n    DEBUG: No valid via positions found (all blocked in obstacle map)")
            print(f"    DEBUG: Searched {max_radius_grid} grid steps ({max_search_radius}mm) from pad center")
        return None

    # Check routability for each position until we find one that works
    route_failures = 0
    skipped_count = 0
    for dist_sq, via_pos, gx, gy in valid_positions:
        # Skip if too close to a position where routing already failed (within this call)
        if failed_route_positions and skip_radius_sq > 0:
            too_close = False
            for failed_gx, failed_gy in failed_route_positions:
                fdx = gx - failed_gx
                fdy = gy - failed_gy
                if fdx * fdx + fdy * fdy <= skip_radius_sq:
                    too_close = True
                    break
            if too_close:
                skipped_count += 1
                continue

        # Try to route from this via position to the pad
        # Use verbose for first few failures to help debug
        route_verbose = verbose and route_failures < 3
        route_result = route_via_to_pad(
            via_pos, pad, pad_layer, net_id,
            routing_obstacles, config, verbose=route_verbose
        )
        if route_result is not None:
            # Routing succeeded - use this position
            if verbose and (route_failures > 0 or skipped_count > 0):
                print(f"[tried {route_failures+1}, skipped {skipped_count}]", end=" ")
            return via_pos

        # Routing failed - add to failed set so nearby positions are skipped
        if failed_route_positions is not None:
            failed_route_positions.add((gx, gy))
        route_failures += 1

    # Debug output on failure
    if verbose:
        print(f"[tried {route_failures}, skipped {skipped_count}]", end=" ")
        if not valid_positions:
            print(f"\n    DEBUG: No valid via positions found (all blocked in obstacle map)")
            print(f"    DEBUG: Searched {max_radius_grid} grid steps ({max_search_radius}mm) from pad center")
        else:
            print(f"\n    DEBUG: Found {len(valid_positions)} unblocked via positions, but routing failed for all")
            print(f"    DEBUG: Closest unblocked position: ({valid_positions[0][1][0]:.2f}, {valid_positions[0][1][1]:.2f})")
            print(f"    DEBUG: Tried to route on layer {pad_layer}")

    return None  # No valid position with routable path


@dataclass
class RouteResult:
    """Result of a routing attempt."""
    segments: Optional[List[Dict]]  # Segments if successful, None if failed
    blocked_cells: List[Tuple[int, int, int]]  # Blocked cells from frontier (for blocker analysis)
    success: bool


def route_via_to_pad(
    via_pos: Tuple[float, float],
    pad: Pad,
    pad_layer: str,
    net_id: int,
    routing_obstacles: GridObstacleMap,
    config: GridRouteConfig,
    max_iterations: int = 10000,
    verbose: bool = False,
    return_blocked_cells: bool = False
) -> Optional[List[Dict]]:
    """
    Route from via position to pad center using A* pathfinding.

    Args:
        via_pos: (x, y) position of the via
        pad: Target pad
        pad_layer: Layer to route on
        net_id: Net ID for the segments
        routing_obstacles: Obstacle map for the route layer
        config: Routing configuration
        max_iterations: Maximum A* iterations
        verbose: Print debug info on failure
        return_blocked_cells: If True, return RouteResult instead of segments

    Returns:
        List of segment dicts, or None if routing failed
        If return_blocked_cells=True, returns RouteResult instead
    """
    coord = GridCoord(config.grid_step)

    # Check if via is at pad center (within tolerance)
    if abs(via_pos[0] - pad.global_x) < 0.001 and abs(via_pos[1] - pad.global_y) < 0.001:
        if return_blocked_cells:
            return RouteResult(segments=[], blocked_cells=[], success=True)
        return []  # Via is at pad center, no trace needed

    # Set up source (via) and target (pad) - single layer routing
    layer_idx = 0
    via_gx, via_gy = coord.to_grid(via_pos[0], via_pos[1])
    pad_gx, pad_gy = coord.to_grid(pad.global_x, pad.global_y)

    # Add source and target as source_target_cells to override blocking
    # This allows routing to/from positions that might be blocked by nearby pad clearances
    routing_obstacles.add_source_target_cell(via_gx, via_gy, layer_idx)
    routing_obstacles.add_source_target_cell(pad_gx, pad_gy, layer_idx)

    # Debug: check if source/target are blocked (should be False now after adding to source_target_cells)
    source_blocked = routing_obstacles.is_blocked(via_gx, via_gy, layer_idx)
    target_blocked = routing_obstacles.is_blocked(pad_gx, pad_gy, layer_idx)

    if verbose and (source_blocked or target_blocked):
        print(f"\n    DEBUG: source_blocked={source_blocked}, target_blocked={target_blocked}")
        print(f"    DEBUG: via=({via_pos[0]:.2f}, {via_pos[1]:.2f}) grid=({via_gx}, {via_gy})")
        print(f"    DEBUG: pad=({pad.global_x:.2f}, {pad.global_y:.2f}) grid=({pad_gx}, {pad_gy})")

    # Check if all neighbors of the target pad are blocked (target is isolated)
    if verbose:
        blocked_neighbors = 0
        unblocked_dirs = []
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
            if routing_obstacles.is_blocked(pad_gx + dx, pad_gy + dy, layer_idx):
                blocked_neighbors += 1
            else:
                unblocked_dirs.append((dx, dy))
        if blocked_neighbors == 8:
            print(f"    DEBUG: Target pad is ISOLATED - all 8 neighbors blocked")
        elif blocked_neighbors >= 6:
            print(f"    DEBUG: Target pad nearly isolated - {blocked_neighbors}/8 neighbors blocked, open: {unblocked_dirs}")
            # Trace along open direction to find where blockage starts
            for dx, dy in unblocked_dirs[:1]:  # Just check first open direction
                blocked_at = None
                for dist in range(1, 30):  # Check up to 30 cells
                    nx, ny = pad_gx + dx * dist, pad_gy + dy * dist
                    if routing_obstacles.is_blocked(nx, ny, layer_idx):
                        blocked_at = dist
                        break
                if blocked_at:
                    print(f"    DEBUG: Open direction ({dx},{dy}) blocked after {blocked_at} cells at grid ({pad_gx + dx*blocked_at}, {pad_gy + dy*blocked_at})")

    sources = [(via_gx, via_gy, layer_idx)]
    targets = [(pad_gx, pad_gy, layer_idx)]

    # Create router and find path
    router = GridRouter(
        via_cost=config.via_cost * 1000,
        h_weight=config.heuristic_weight,
        turn_cost=config.turn_cost,
        via_proximity_cost=0
    )

    path, iterations, blocked_cells = router.route_with_frontier(
        routing_obstacles, sources, targets, max_iterations,
        False,  # collinear_vias
        0,      # via_exclusion_radius
        None,   # start_direction
        None,   # end_direction
        0       # direction_steps
    )

    if path is None:
        if verbose:
            print(f"    DEBUG: A* failed after {iterations} iterations")
        # Clear source_target_cells for next route
        routing_obstacles.clear_source_target_cells()
        if return_blocked_cells:
            return RouteResult(segments=None, blocked_cells=blocked_cells, success=False)
        return None  # Routing failed

    # Clear source_target_cells for next route
    routing_obstacles.clear_source_target_cells()

    # Convert path to segments
    segments = []

    # Add connecting segment from via to first path point
    if path:
        first_gx, first_gy, _ = path[0]
        first_x, first_y = coord.to_float(first_gx, first_gy)
        if abs(via_pos[0] - first_x) > 0.001 or abs(via_pos[1] - first_y) > 0.001:
            segments.append({
                'start': via_pos,
                'end': (first_x, first_y),
                'width': config.track_width,
                'layer': pad_layer,
                'net_id': net_id
            })

    # Convert path points to segments
    for i in range(len(path) - 1):
        gx1, gy1, _ = path[i]
        gx2, gy2, _ = path[i + 1]

        x1, y1 = coord.to_float(gx1, gy1)
        x2, y2 = coord.to_float(gx2, gy2)

        if (x1, y1) != (x2, y2):
            segments.append({
                'start': (x1, y1),
                'end': (x2, y2),
                'width': config.track_width,
                'layer': pad_layer,
                'net_id': net_id
            })

    # Add connecting segment from last path point to pad center
    if path:
        last_gx, last_gy, _ = path[-1]
        last_x, last_y = coord.to_float(last_gx, last_gy)
        if abs(pad.global_x - last_x) > 0.001 or abs(pad.global_y - last_y) > 0.001:
            segments.append({
                'start': (last_x, last_y),
                'end': (pad.global_x, pad.global_y),
                'width': config.track_width,
                'layer': pad_layer,
                'net_id': net_id
            })

    if return_blocked_cells:
        return RouteResult(segments=segments, blocked_cells=[], success=True)
    return segments


def route_plane_connection(
    via_a: Tuple[float, float],
    via_b: Tuple[float, float],
    plane_layer: str,
    net_id: int,
    other_nets_vias: Dict[int, List[Tuple[float, float]]],
    config: GridRouteConfig,
    pcb_data: PCBData,
    proximity_radius: float = 3.0,
    proximity_cost: float = 2.0,
    track_via_clearance: float = 0.8,
    max_iterations: int = 200000,
    verbose: bool = False,
    previous_routes: Optional[List[List[Tuple[float, float]]]] = None
) -> Optional[List[Tuple[float, float]]]:
    """
    Route a trace on the plane layer between two vias, avoiding other nets' vias.

    Args:
        via_a: (x, y) position of first via
        via_b: (x, y) position of second via
        plane_layer: Layer to route on (e.g., 'In5.Cu')
        net_id: Net ID for this connection
        other_nets_vias: Dict mapping other net_id -> list of (x, y) via positions
        config: Routing configuration
        pcb_data: PCB data for obstacle building
        proximity_radius: Radius around other vias to add proximity cost (mm)
        proximity_cost: Maximum proximity cost (mm equivalent)
        track_via_clearance: Clearance from track center to other nets' via centers (mm).
            This should be large enough to leave room for polygon fill.
        max_iterations: Maximum A* iterations
        verbose: Print debug info
        previous_routes: List of previously routed paths from other nets to avoid (each is a list of (x,y) points)

    Returns:
        List of (x, y) points along the route, or None if routing fails
    """
    coord = GridCoord(config.grid_step)

    # Build single-layer obstacle map
    num_layers = 1
    layer_idx = 0
    obstacles = GridObstacleMap(num_layers)

    # Block other nets' vias as hard obstacles
    # Use track_via_clearance as radius from via center to track center
    for other_net_id, via_positions in other_nets_vias.items():
        for vx, vy in via_positions:
            gx, gy = coord.to_grid(vx, vy)
            # Block a circular area around each via using track_via_clearance
            # This ensures routes stay far enough from vias for polygons to fit
            via_radius = coord.to_grid_dist(track_via_clearance)
            for ex in range(-via_radius, via_radius + 1):
                for ey in range(-via_radius, via_radius + 1):
                    if ex * ex + ey * ey <= via_radius * via_radius:
                        obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)

    # Add proximity costs around other nets' vias
    proximity_radius_grid = coord.to_grid_dist(proximity_radius)
    proximity_cost_grid = int(proximity_cost * 1000 / config.grid_step)

    all_other_vias = []
    for via_positions in other_nets_vias.values():
        for vx, vy in via_positions:
            gx, gy = coord.to_grid(vx, vy)
            all_other_vias.append((gx, gy))

    if all_other_vias:
        obstacles.add_stub_proximity_costs_batch(
            all_other_vias,
            proximity_radius_grid,
            proximity_cost_grid,
            False  # Don't block vias (we just want proximity cost)
        )

    # Also block existing segments on this layer from other nets
    for seg in pcb_data.segments:
        if seg.net_id == net_id:
            continue
        if seg.layer != plane_layer:
            continue
        seg_expansion_mm = config.track_width / 2 + seg.width / 2 + config.clearance
        seg_expansion_grid = max(1, coord.to_grid_dist(seg_expansion_mm))
        _add_segment_routing_obstacle(obstacles, seg, coord, layer_idx, seg_expansion_grid)

    # Block previous routes between disconnected regions (same net, avoid self-crossing)
    if previous_routes:
        route_expansion_mm = config.track_width + config.clearance
        route_expansion_grid = max(1, coord.to_grid_dist(route_expansion_mm))
        for route_path in previous_routes:
            for i in range(len(route_path) - 1):
                p1, p2 = route_path[i], route_path[i + 1]
                # Create a temporary segment-like structure for blocking
                gx1, gy1 = coord.to_grid(p1[0], p1[1])
                gx2, gy2 = coord.to_grid(p2[0], p2[1])
                # Block along the line with expansion
                dx = abs(gx2 - gx1)
                dy = abs(gy2 - gy1)
                sx = 1 if gx1 < gx2 else -1
                sy = 1 if gy1 < gy2 else -1
                gx, gy = gx1, gy1
                radius_sq = route_expansion_grid * route_expansion_grid
                if dx > dy:
                    err = dx / 2
                    while gx != gx2:
                        for ex in range(-route_expansion_grid, route_expansion_grid + 1):
                            for ey in range(-route_expansion_grid, route_expansion_grid + 1):
                                if ex*ex + ey*ey <= radius_sq:
                                    obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
                        err -= dy
                        if err < 0:
                            gy += sy
                            err += dx
                        gx += sx
                else:
                    err = dy / 2
                    while gy != gy2:
                        for ex in range(-route_expansion_grid, route_expansion_grid + 1):
                            for ey in range(-route_expansion_grid, route_expansion_grid + 1):
                                if ex*ex + ey*ey <= radius_sq:
                                    obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
                        err -= dx
                        if err < 0:
                            gx += sx
                            err += dy
                        gy += sy
                # Block endpoint
                for ex in range(-route_expansion_grid, route_expansion_grid + 1):
                    for ey in range(-route_expansion_grid, route_expansion_grid + 1):
                        if ex*ex + ey*ey <= radius_sq:
                            obstacles.add_blocked_cell(gx2 + ex, gy2 + ey, layer_idx)

    # Set up source and target
    via_a_gx, via_a_gy = coord.to_grid(via_a[0], via_a[1])
    via_b_gx, via_b_gy = coord.to_grid(via_b[0], via_b[1])

    # Make sure source and target are not blocked
    obstacles.add_source_target_cell(via_a_gx, via_a_gy, layer_idx)
    obstacles.add_source_target_cell(via_b_gx, via_b_gy, layer_idx)

    sources = [(via_a_gx, via_a_gy, layer_idx)]
    targets = [(via_b_gx, via_b_gy, layer_idx)]

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
            print(f"    Route between regions failed after {iterations} iterations")
        return None

    if verbose:
        print(f"    Route found in {iterations} iterations, {len(path)} points")

    # Convert path to float coordinates
    route_points = []
    for gx, gy, _ in path:
        x, y = coord.to_float(gx, gy)
        route_points.append((x, y))

    return route_points


def _generate_multinet_layer_zones(
    layer: str,
    nets_on_layer: List[str],
    pcb_data: PCBData,
    all_new_vias: List[Dict],
    zone_polygon: List[Tuple[float, float]],
    board_bounds: Tuple[float, float, float, float],
    config: GridRouteConfig,
    zone_clearance: float,
    min_thickness: float,
    plane_proximity_radius: float,
    plane_proximity_cost: float,
    plane_track_via_clearance: float,
    plane_max_iterations: int,
    voronoi_seed_interval: float,
    board_edge_clearance: float,
    debug_lines: bool,
    verbose: bool
) -> Tuple[List[str], List[str]]:
    """
    Generate Voronoi-based zone boundaries for a multi-net layer.

    Args:
        layer: Layer name (e.g., 'In1.Cu')
        nets_on_layer: List of net names sharing this layer
        pcb_data: PCB data
        all_new_vias: List of newly placed vias
        zone_polygon: Default zone polygon (full board)
        board_bounds: (min_x, min_y, max_x, max_y)
        config: Routing configuration
        zone_clearance: Zone clearance
        min_thickness: Minimum zone thickness
        plane_proximity_radius: Proximity radius for routing
        plane_proximity_cost: Proximity cost for routing
        plane_track_via_clearance: Track-to-via clearance
        plane_max_iterations: Max A* iterations
        voronoi_seed_interval: Sample interval for Voronoi seeds
        board_edge_clearance: Edge clearance for zones
        debug_lines: Whether to generate debug lines
        verbose: Verbose output

    Returns:
        Tuple of (zone_sexprs, debug_line_sexprs)
    """
    zone_sexprs = []
    debug_line_sexprs = []

    # Build vias_by_net for this layer
    vias_by_net: Dict[int, List[Tuple[float, float]]] = {}
    net_name_to_id = {}
    for net_name in nets_on_layer:
        net_id = next((nid for nid, n in pcb_data.nets.items() if n.name == net_name), None)
        if net_id is not None:
            net_name_to_id[net_name] = net_id
            vias_by_net[net_id] = []

    # Collect via positions for nets on this layer
    for via in all_new_vias:
        if via['net_id'] in vias_by_net:
            vias_by_net[via['net_id']].append((via['x'], via['y']))

    # Also include existing vias from the PCB
    for via in pcb_data.vias:
        if via.net_id in vias_by_net:
            via_pos = (via.x, via.y)
            if via_pos not in vias_by_net[via.net_id]:
                vias_by_net[via.net_id].append(via_pos)

    # Check for nets with no vias
    nets_with_vias = []
    for net_name in nets_on_layer:
        net_id = net_name_to_id.get(net_name)
        if net_id:
            via_count = len(vias_by_net.get(net_id, []))
            if via_count == 0:
                print(f"  Warning: Net '{net_name}' has no vias on layer {layer}, skipping zone")
            else:
                nets_with_vias.append(net_name)
                print(f"  Net '{net_name}': {via_count} vias")

    if len(nets_with_vias) < 2:
        # Only one net has vias, use full board rectangle
        if nets_with_vias:
            net_name = nets_with_vias[0]
            net_id = net_name_to_id[net_name]
            print(f"  Only '{net_name}' has vias, using full board rectangle")
            zone_sexpr = generate_zone_sexpr(
                net_id=net_id,
                net_name=net_name,
                layer=layer,
                polygon_points=zone_polygon,
                clearance=zone_clearance,
                min_thickness=min_thickness,
                direct_connect=True
            )
            zone_sexprs.append(zone_sexpr)
        return zone_sexprs, debug_line_sexprs

    # Compute MST edges for each net
    net_mst_edges: Dict[int, List[Tuple[Tuple[float, float], Tuple[float, float]]]] = {}
    net_debug_layers: Dict[int, str] = {}
    for net_idx, net_name in enumerate(nets_with_vias):
        net_id = net_name_to_id[net_name]
        net_vias = vias_by_net.get(net_id, [])
        if len(net_vias) >= 2:
            net_mst_edges[net_id] = compute_mst_segments(net_vias)
            net_debug_layers[net_id] = f"User.{net_idx + 1}"
            print(f"  Net '{net_name}': MST with {len(net_mst_edges[net_id])} edges between {len(net_vias)} vias")
        else:
            print(f"  Net '{net_name}': only {len(net_vias)} via(s), no MST needed")

    # Iteratively route all nets, reordering to put failed nets first
    max_mst_iterations = 5
    net_order = list(net_mst_edges.keys())
    failed_nets: Set[int] = set()
    best_result = None

    for mst_iteration in range(max_mst_iterations):
        if mst_iteration > 0:
            net_order = sorted(net_order, key=lambda nid: (0 if nid in failed_nets else 1))
            failed_net_names = [pcb_data.nets[nid].name for nid in failed_nets if nid in pcb_data.nets]
            print(f"  Retry {mst_iteration + 1}: reordering with failed nets first: {', '.join(failed_net_names)}")

        connection_routes = []
        routed_paths_by_edge: Dict[int, Dict[Tuple[Tuple[float, float], Tuple[float, float]], List[Tuple[float, float]]]] = {
            net_id: {} for net_id in net_mst_edges.keys()
        }
        augmented_vias_by_net = {net_id: list(vias) for net_id, vias in vias_by_net.items()}
        debug_lines_for_layer = []
        failed_nets = set()
        total_failed_edges = 0

        for net_id in net_order:
            net = pcb_data.nets.get(net_id)
            net_name = net.name if net else f"net_{net_id}"
            mst_edges = net_mst_edges[net_id]
            debug_layer = net_debug_layers[net_id]

            other_nets_vias: Dict[int, List[Tuple[float, float]]] = {}
            for other_net_id, other_vias in augmented_vias_by_net.items():
                if other_net_id != net_id:
                    other_nets_vias[other_net_id] = other_vias

            routed_count = 0
            failed_count = 0

            for via_a, via_b in mst_edges:
                other_nets_routes = [
                    route for route_net_id, _, route in connection_routes
                    if route_net_id != net_id
                ]

                route_path = route_plane_connection(
                    via_a=via_a,
                    via_b=via_b,
                    plane_layer=layer,
                    net_id=net_id,
                    other_nets_vias=other_nets_vias,
                    config=config,
                    pcb_data=pcb_data,
                    proximity_radius=plane_proximity_radius,
                    proximity_cost=plane_proximity_cost,
                    track_via_clearance=plane_track_via_clearance,
                    max_iterations=plane_max_iterations,
                    verbose=verbose,
                    previous_routes=other_nets_routes
                )

                if route_path:
                    routed_count += 1
                    connection_routes.append((net_id, layer, route_path))
                    routed_paths_by_edge[net_id][(via_a, via_b)] = route_path

                    if debug_lines and len(route_path) >= 2:
                        for i in range(len(route_path) - 1):
                            debug_lines_for_layer.append(generate_gr_line_sexpr(
                                route_path[i], route_path[i + 1],
                                width=0.1, layer=debug_layer
                            ))

                    samples = sample_route_for_voronoi(route_path, sample_interval=voronoi_seed_interval)
                    if samples:
                        augmented_vias_by_net[net_id].extend(samples)
                else:
                    failed_count += 1
                    if verbose:
                        print(f"    {net_name}: ({via_a[0]:.2f},{via_a[1]:.2f}) -> ({via_b[0]:.2f},{via_b[1]:.2f}) FAILED")

            if failed_count > 0:
                failed_nets.add(net_id)
                total_failed_edges += failed_count
                print(f"    {net_name}: {routed_count}/{len(mst_edges)} MST edges ({failed_count} failed)")
            else:
                print(f"    {net_name}: all {routed_count} MST edges routed")

        if best_result is None or total_failed_edges < best_result[0]:
            best_result = (total_failed_edges, connection_routes, augmented_vias_by_net, debug_lines_for_layer, routed_paths_by_edge)

        if total_failed_edges == 0:
            break

    # Use best result
    if best_result:
        _, connection_routes, augmented_vias_by_net, debug_lines_for_layer, routed_paths_by_edge = best_result
        debug_line_sexprs.extend(debug_lines_for_layer)
        if best_result[0] > 0:
            print(f"  Best result: {best_result[0]} failed edge(s)")

    # Compute final Voronoi zones
    total_seeds = sum(len(vias) for vias in augmented_vias_by_net.values())
    print(f"  Computing final Voronoi zones with {total_seeds} seed points")

    try:
        zone_polygons, _, _ = compute_zone_boundaries(
            augmented_vias_by_net, board_bounds,
            return_raw_polygons=True,
            board_edge_clearance=board_edge_clearance,
            verbose=verbose
        )
    except ValueError as e:
        print(f"  Error computing zone boundaries: {e}")
        print(f"  Falling back to full board rectangle for first net")
        net_name = nets_with_vias[0]
        net_id = net_name_to_id[net_name]
        zone_sexpr = generate_zone_sexpr(
            net_id=net_id,
            net_name=net_name,
            layer=layer,
            polygon_points=zone_polygon,
            clearance=zone_clearance,
            min_thickness=min_thickness,
            direct_connect=True
        )
        zone_sexprs.append(zone_sexpr)
        return zone_sexprs, debug_line_sexprs

    # Generate zones for each net
    for net_id, polygons in zone_polygons.items():
        net = pcb_data.nets.get(net_id)
        net_name = net.name if net else f"net_{net_id}"
        for poly_idx, polygon in enumerate(polygons):
            if len(polygons) > 1:
                print(f"  Creating zone {poly_idx+1}/{len(polygons)} for '{net_name}' with {len(polygon)} vertices")
            else:
                print(f"  Creating zone for '{net_name}' with {len(polygon)} vertices")
            zone_sexpr = generate_zone_sexpr(
                net_id=net_id,
                net_name=net_name,
                layer=layer,
                polygon_points=polygon,
                clearance=zone_clearance,
                min_thickness=min_thickness,
                direct_connect=True
            )
            zone_sexprs.append(zone_sexpr)

    # Calculate and print resistance
    resistance_results = {}
    for net_id, polygons in zone_polygons.items():
        net = pcb_data.nets.get(net_id)
        net_name = net.name if net else f"net_{net_id}"
        mst_edges = net_mst_edges.get(net_id, [])
        edge_routes = routed_paths_by_edge.get(net_id, {})
        largest_polygon = max(polygons, key=lambda p: len(p))
        result = analyze_multi_net_plane(largest_polygon, mst_edges, edge_routes, layer)
        resistance_results[net_name] = result

    print_multi_net_resistance(resistance_results)

    return zone_sexprs, debug_line_sexprs


def _write_output_and_reroute(
    input_file: str,
    output_file: str,
    all_zone_sexprs: List[str],
    all_debug_lines: List[str],
    all_new_vias: List[Dict],
    all_new_segments: List[Dict],
    all_ripped_net_ids: List[int],
    zones_to_replace: List[Tuple[int, str]],
    pcb_data: PCBData,
    reroute_ripped_nets: bool,
    all_layers: List[str],
    plane_layers: List[str],
    track_width: float,
    clearance: float,
    via_size: float,
    via_drill: float,
    grid_step: float,
    hole_to_hole_clearance: float,
    verbose: bool
) -> bool:
    """
    Write output file and optionally reroute ripped nets.

    Returns:
        True if output was written successfully
    """
    print(f"\nWriting output to {output_file}...")
    all_sexprs = all_zone_sexprs + all_debug_lines
    combined_zone_sexpr = '\n'.join(all_sexprs) if all_sexprs else None
    if all_debug_lines:
        print(f"  Adding {len(all_debug_lines)} debug lines on User.4")

    if not write_plane_output(input_file, output_file, combined_zone_sexpr, all_new_vias, all_new_segments,
                              exclude_net_ids=all_ripped_net_ids, zones_to_replace=zones_to_replace):
        print("Error writing output file")
        return False

    print(f"Output written to {output_file}")
    print("Note: Open in KiCad and press 'B' to refill zones")

    if all_ripped_net_ids:
        ripped_net_names = []
        for rid in all_ripped_net_ids:
            net = pcb_data.nets.get(rid)
            if net:
                ripped_net_names.append(net.name)

        if reroute_ripped_nets and ripped_net_names:
            print(f"\n{'='*60}")
            print(f"Re-routing {len(ripped_net_names)} ripped net(s)...")
            print(f"{'='*60}")
            old_recursion_limit = sys.getrecursionlimit()
            sys.setrecursionlimit(max(old_recursion_limit, 100000))
            try:
                routing_layers = [l for l in all_layers if l not in plane_layers]
                if not routing_layers:
                    routing_layers = ['F.Cu', 'B.Cu']
                all_copper_layers = list(set(all_layers + plane_layers))
                routed, failed, route_time = batch_route(
                    input_file=output_file,
                    output_file=output_file,
                    net_names=ripped_net_names,
                    layers=all_copper_layers,
                    track_width=track_width,
                    clearance=clearance,
                    via_size=via_size,
                    via_drill=via_drill,
                    grid_step=grid_step,
                    hole_to_hole_clearance=hole_to_hole_clearance,
                    verbose=verbose,
                    minimal_obstacle_cache=True
                )
                print(f"\nRe-routing complete: {routed} routed, {failed} failed in {route_time:.2f}s")
            finally:
                sys.setrecursionlimit(old_recursion_limit)
        else:
            print(f"WARNING: {len(all_ripped_net_ids)} net(s) were removed from output and need re-routing!")
            if ripped_net_names:
                print(f"  Ripped nets: {', '.join(ripped_net_names)}")

    return True


def create_plane(
    input_file: str,
    output_file: str,
    net_names: List[str],
    plane_layers: List[str],
    via_size: float = 0.3,
    via_drill: float = 0.2,
    track_width: float = 0.1,
    clearance: float = 0.1,
    zone_clearance: float = 0.2,
    min_thickness: float = 0.1,
    grid_step: float = 0.1,
    max_search_radius: float = 10.0,
    max_via_reuse_radius: float = 1.0,
    close_via_radius: float = None,
    hole_to_hole_clearance: float = 0.2,
    all_layers: List[str] = None,
    verbose: bool = False,
    dry_run: bool = False,
    rip_blocker_nets: bool = False,
    max_rip_nets: int = 3,
    reroute_ripped_nets: bool = False,
    layer_nets: Dict[str, List[str]] = None,
    plane_proximity_radius: float = 3.0,
    plane_proximity_cost: float = 2.0,
    plane_track_via_clearance: float = 0.8,
    board_edge_clearance: float = 0.5,
    voronoi_seed_interval: float = 2.0,
    plane_max_iterations: int = 200000,
    debug_lines: bool = False
) -> Tuple[int, int, int]:
    """
    Create copper plane zones and place vias to connect target pads for multiple nets.

    Args:
        net_names: List of net names to process (e.g., ['GND', 'VCC'])
        plane_layers: List of layers for each net (e.g., ['In1.Cu', 'In2.Cu'])
        rip_blocker_nets: If True, identify and temporarily remove nets blocking
                          via placement, then retry. Ripped nets excluded from output.
        max_rip_nets: Maximum number of blocker nets to rip up.
        reroute_ripped_nets: If True, automatically re-route ripped nets after placing vias.
        plane_proximity_radius: Radius around other nets' vias for proximity cost (mm).
        plane_proximity_cost: Maximum proximity cost around other nets' vias (mm equivalent).
        plane_track_via_clearance: Clearance from track center to other nets' via centers (mm).
            MST routes will avoid regions within this distance of other nets' vias to
            leave room for polygon fill.
        board_edge_clearance: Clearance from board edge for zone polygons (mm).
        voronoi_seed_interval: Sample interval for Voronoi seed points along routes (mm).
        plane_max_iterations: Max A* iterations for routing plane connections.

    Returns:
        (total_vias_placed, total_traces_added, total_pads_needing_vias)
    """
    if all_layers is None:
        all_layers = ['F.Cu', 'B.Cu']

    if len(net_names) != len(plane_layers):
        print(f"Error: Number of nets ({len(net_names)}) must match number of layers ({len(plane_layers)})")
        return (0, 0, 0)

    # Step 1: Load PCB
    print(f"Loading PCB from {input_file}...")
    pcb_data = parse_kicad_pcb(input_file)

    # Resolve all net IDs upfront
    net_ids = []
    for net_name in net_names:
        net_id = resolve_net_id(pcb_data, net_name)
        if net_id is None:
            print(f"Error: Net '{net_name}' not found in PCB")
            return (0, 0, 0)
        net_ids.append(net_id)
        print(f"Found net '{net_name}' with ID {net_id}")

    # Track failed pads per net for retry passes
    # Each entry is (net_id, net_name, plane_layer, pad_info)
    failed_pad_infos: List[Tuple[int, str, str, Dict]] = []

    # Step 2: Check for existing zones on each target layer
    existing_zones = extract_zones(input_file)
    should_create_zones = []  # Per-net flag for whether to create zone
    zones_to_replace = []  # List of (net_id, layer) tuples for zones to replace
    for i, (net_name, plane_layer, net_id) in enumerate(zip(net_names, plane_layers, net_ids)):
        should_create, should_continue, zone_to_replace = check_existing_zones(
            existing_zones, plane_layer, net_name, net_id, verbose
        )
        if not should_continue:
            print(f"Error: Zone conflict for net '{net_name}' on layer {plane_layer}")
            return (0, 0, 0)
        should_create_zones.append(should_create)
        if zone_to_replace:
            zones_to_replace.append((zone_to_replace.net_id, zone_to_replace.layer))

    # Step 3: Get board bounds for zone polygon
    board_bounds = pcb_data.board_info.board_bounds
    if not board_bounds:
        print("Error: Could not determine board bounds")
        return (0, 0, 0)

    min_x, min_y, max_x, max_y = board_bounds
    print(f"Board bounds: ({min_x:.2f}, {min_y:.2f}) to ({max_x:.2f}, {max_y:.2f})")

    # Create zone polygon from board bounds (with edge clearance applied)
    zone_polygon = [
        (min_x + board_edge_clearance, min_y + board_edge_clearance),
        (max_x - board_edge_clearance, min_y + board_edge_clearance),
        (max_x - board_edge_clearance, max_y - board_edge_clearance),
        (min_x + board_edge_clearance, max_y - board_edge_clearance)
    ]

    # Step 4: Build config and coordinate system
    config = GridRouteConfig(
        track_width=track_width,
        clearance=clearance,
        via_size=via_size,
        via_drill=via_drill,
        grid_step=grid_step,
        hole_to_hole_clearance=hole_to_hole_clearance,
        layers=all_layers
    )
    coord = GridCoord(grid_step)

    # Accumulated results across all nets
    all_new_vias = []
    all_new_segments = []
    all_zone_sexprs = []
    all_debug_lines = []  # Debug lines for inter-region routes (User.4)
    total_vias_placed = 0
    total_vias_reused = 0
    total_traces_added = 0
    total_failed_pads = 0
    total_pads_needing_vias = 0
    all_ripped_net_ids: List[int] = []

    # Collect ALL pads from ALL power nets that need vias (for cross-net protection)
    # This ensures when routing GND, we also protect +3.3V pad zones and vice versa
    all_power_pads_needing_vias: List[Dict] = []
    for net_id_tmp, plane_layer_tmp in zip(net_ids, plane_layers):
        target_pads_tmp = identify_target_pads(pcb_data, net_id_tmp, plane_layer_tmp)
        for p in target_pads_tmp:
            if p['needs_via']:
                p['_net_id'] = net_id_tmp  # Tag with net ID for filtering later
                all_power_pads_needing_vias.append(p)

    # Set to track pads that have been successfully processed (via placed)
    processed_pad_ids: Set[Tuple[float, float]] = set()  # (global_x, global_y) as key

    # Process each net/layer pair
    for net_idx, (net_name, plane_layer, net_id, should_create_zone) in enumerate(
            zip(net_names, plane_layers, net_ids, should_create_zones)):

        print(f"\n{'='*60}")
        print(f"Processing net '{net_name}' on layer {plane_layer}")
        print(f"{'='*60}")

        # Step 5: Identify target pads for this net
        target_pads = identify_target_pads(pcb_data, net_id, plane_layer)

        pads_through_hole = sum(1 for p in target_pads if p['type'] == 'through_hole')
        pads_direct = sum(1 for p in target_pads if p['type'] == 'direct')
        pads_need_via = sum(1 for p in target_pads if p['type'] == 'via_needed')
        total_pads_needing_vias += pads_need_via

        print(f"\nPad analysis for net '{net_name}':")
        print(f"  Through-hole pads (no via needed): {pads_through_hole}")
        print(f"  SMD pads on {plane_layer} (no via needed): {pads_direct}")
        print(f"  SMD pads on other layers (via needed): {pads_need_via}")

        # Step 6: Collect existing vias on target net (for reuse)
        existing_net_vias: List[Tuple[float, float]] = []
        for via in pcb_data.vias:
            if via.net_id == net_id:
                existing_net_vias.append((via.x, via.y))

        if verbose and existing_net_vias:
            print(f"  Existing vias on net '{net_name}': {len(existing_net_vias)}")

        # Step 7: Build obstacle map for via placement (exclude current net)
        if pads_need_via > 0:
            print("\nBuilding obstacle map for via placement...")
            obstacles = build_via_obstacle_map(pcb_data, config, net_id)
            # Also block positions of vias we've already placed in previous nets
            for placed_via in all_new_vias:
                block_via_position(obstacles, placed_via['x'], placed_via['y'], coord,
                                   hole_to_hole_clearance, via_drill)
        else:
            obstacles = None

        # Step 8: Build routing obstacle maps (cached per layer, but rebuild for each net)
        routing_obstacles_cache: Dict[str, GridObstacleMap] = {}
        if verbose:
            print(f"  pcb_data has {len(pcb_data.vias)} vias, {len(pcb_data.segments)} segments")

        def get_routing_obstacles(layer: str) -> GridObstacleMap:
            """Get or create routing obstacle map for a layer."""
            if layer not in routing_obstacles_cache:
                if verbose:
                    print(f"  Building routing obstacle map for {layer}...")
                routing_obstacles_cache[layer] = build_routing_obstacle_map(
                    pcb_data, config, net_id, layer, skip_pad_blocking=False
                )
            return routing_obstacles_cache[layer]

        # Step 9: Place vias near each target pad (or reuse existing)
        new_vias = []
        new_segments = []
        vias_placed = 0
        vias_reused = 0
        traces_added = 0
        failed_pads = 0
        ripped_net_ids: List[int] = []  # Nets ripped for this net

        # Track all available vias (existing + newly placed) for reuse
        available_vias = list(existing_net_vias)
        # Also include vias placed earlier for THIS net (not other nets!)
        for placed_via in all_new_vias:
            if placed_via['net_id'] == net_id:
                available_vias.append((placed_via['x'], placed_via['y']))

        # Cache for incremental obstacle updates during rip-up
        # Computed lazily when we first encounter each blocker net
        via_obstacle_cache: Dict[int, ViaPlacementObstacleData] = {}

        def ensure_via_obstacle_cache(blocker_net_id: int):
            """Ensure we have cached obstacles for a net (computed lazily)."""
            if blocker_net_id not in via_obstacle_cache:
                via_obstacle_cache[blocker_net_id] = precompute_via_placement_obstacles(
                    pcb_data, blocker_net_id, config, all_layers
                )

        # Build list of pads needing vias for this net
        pads_needing_vias = [p for p in target_pads if p['needs_via']]

        # Draw all pad exclusion zones on User.9 once at the start of FIRST net (for debugging)
        if debug_lines and net_idx == 0 and all_power_pads_needing_vias:
            margin = 1.5 * via_size + clearance  # via_size/2 for placed via + via_size/2 for future via + via_size/2 extra + clearance
            for pp_info in all_power_pads_needing_vias:
                pp = pp_info['pad']
                half_w = pp.size_x / 2 + margin
                half_h = pp.size_y / 2 + margin
                x1, y1 = pp.global_x - half_w, pp.global_y - half_h
                x2, y2 = pp.global_x + half_w, pp.global_y + half_h
                all_debug_lines.append(generate_gr_line_sexpr((x1, y1), (x2, y1), 0.05, "User.9"))
                all_debug_lines.append(generate_gr_line_sexpr((x2, y1), (x2, y2), 0.05, "User.9"))
                all_debug_lines.append(generate_gr_line_sexpr((x2, y2), (x1, y2), 0.05, "User.9"))
                all_debug_lines.append(generate_gr_line_sexpr((x1, y2), (x1, y1), 0.05, "User.9"))

        for pad_idx, pad_info in enumerate(pads_needing_vias):
            pad = pad_info['pad']
            pad_layer = pad_info.get('pad_layer')
            current_pad_key = (pad.global_x, pad.global_y)

            # Skip pads already processed in previous layer passes
            # (a via connects ALL layers, so once placed, pad is done)
            if current_pad_key in processed_pad_ids:
                continue

            # Pending pads are ALL pads from OTHER power nets (cross-net protection)
            # Plus pads from ripped nets that will need to be rerouted
            # We protect ALL pads from other nets, not just those needing vias
            pending_pads = []
            # Add ALL pads from other power nets (not just those needing vias)
            for other_net_id in net_ids:
                if other_net_id == net_id:
                    continue  # Skip current net's pads
                other_pads = pcb_data.pads_by_net.get(other_net_id, [])
                for op in other_pads:
                    op_key = (op.global_x, op.global_y)
                    if op_key != current_pad_key and op_key not in processed_pad_ids:
                        pending_pads.append({'pad': op, 'needs_via': False})
            # Also protect pads from ripped nets (they need to be rerouted later)
            for ripped_id in all_ripped_net_ids:
                ripped_pads = pcb_data.pads_by_net.get(ripped_id, [])
                for rp in ripped_pads:
                    rp_key = (rp.global_x, rp.global_y)
                    if rp_key != current_pad_key and rp_key not in processed_pad_ids:
                        # Create a pad_info dict compatible with pending_pads format
                        pending_pads.append({'pad': rp, 'needs_via': True})

            print(f"  Pad {pad.component_ref}.{pad.pad_number}...", end=" ")

            # First, check if there's already a via very close by (within ~2 via diameters)
            # This handles cases like decoupling caps where both pads are on same net
            # Check for nearby existing via before placing a new one
            effective_close_via_radius = close_via_radius if close_via_radius is not None else via_size * 2.5
            nearby_via = find_existing_via_nearby(pad, available_vias, effective_close_via_radius)
            if nearby_via:
                # Via already very close - try to reuse it
                via_pos = nearby_via
                dist = ((via_pos[0] - pad.global_x)**2 + (via_pos[1] - pad.global_y)**2)**0.5

                if pad_layer:
                    routing_obs = get_routing_obstacles(pad_layer)
                    route_result = route_via_to_pad(via_pos, pad, pad_layer, net_id,
                                                   routing_obs, config, verbose=verbose,
                                                   return_blocked_cells=True)
                    trace_segments = route_result.segments if route_result.success else None
                    if trace_segments is not None:
                        if trace_segments:
                            new_segments.extend(trace_segments)
                            traces_added += len(trace_segments)
                        vias_reused += 1
                        print(f"reusing nearby via at ({via_pos[0]:.2f}, {via_pos[1]:.2f}), {len(trace_segments) if trace_segments else 0} segs, {dist:.2f}mm")
                        processed_pad_ids.add(current_pad_key)
                        continue  # Move to next pad
                else:
                    # No pad layer (through-hole) - just count as reused
                    vias_reused += 1
                    print(f"reusing nearby via at ({via_pos[0]:.2f}, {via_pos[1]:.2f}), {dist:.2f}mm")
                    processed_pad_ids.add(current_pad_key)
                    continue

            # Next, try to place via within pad boundary (preferred - no trace needed)
            # This avoids creating long traces that block other signals
            # Search from pad center outward within pad bounds
            pad_gx, pad_gy = coord.to_grid(pad.global_x, pad.global_y)
            pad_half_w_grid = max(1, coord.to_grid_dist(pad.size_x / 2))
            pad_half_h_grid = max(1, coord.to_grid_dist(pad.size_y / 2))

            via_in_pad = None
            # Check pad center first
            if not obstacles.is_via_blocked(pad_gx, pad_gy):
                via_in_pad = (pad.global_x, pad.global_y)
            else:
                # Spiral search within pad boundary for closest unblocked position
                max_r = max(pad_half_w_grid, pad_half_h_grid)
                for r in range(1, max_r + 1):
                    if via_in_pad:
                        break
                    for dx in range(-r, r + 1):
                        if via_in_pad:
                            break
                        for dy in range(-r, r + 1):
                            if abs(dx) != r and abs(dy) != r:
                                continue  # Only check ring edge
                            # Check if within pad boundary
                            if abs(dx) > pad_half_w_grid or abs(dy) > pad_half_h_grid:
                                continue
                            gx, gy = pad_gx + dx, pad_gy + dy
                            if not obstacles.is_via_blocked(gx, gy):
                                # Convert grid back to world coordinates
                                via_in_pad = (gx * config.grid_step, gy * config.grid_step)
                                break

            if via_in_pad:
                # Found position within pad - place via there (no trace needed)
                # KiCad vias only specify start/end layers, not intermediate
                new_vias.append({
                    'x': via_in_pad[0], 'y': via_in_pad[1],
                    'size': via_size, 'drill': via_drill,
                    'layers': ['F.Cu', 'B.Cu'], 'net_id': net_id
                })
                available_vias.append(via_in_pad)
                vias_placed += 1
                # Block this via position for hole-to-hole clearance
                block_via_position(obstacles, via_in_pad[0], via_in_pad[1], coord,
                                   hole_to_hole_clearance, via_drill)
                if via_in_pad == (pad.global_x, pad.global_y):
                    print(f"via at pad center")
                else:
                    print(f"via at ({via_in_pad[0]:.2f}, {via_in_pad[1]:.2f}) within pad")
                processed_pad_ids.add(current_pad_key)
                continue  # Move to next pad

            # Pad center blocked - check if there's an existing via nearby to reuse
            existing_via = find_existing_via_nearby(pad, available_vias, max_via_reuse_radius)

            if existing_via:
                # Try to reuse existing via - route trace to connect
                via_pos = existing_via
                reuse_success = False

                if pad_layer:
                    routing_obs = get_routing_obstacles(pad_layer)
                    route_result = route_via_to_pad(via_pos, pad, pad_layer, net_id,
                                                       routing_obs, config, verbose=verbose,
                                                       return_blocked_cells=True)
                    trace_segments = route_result.segments if route_result.success else None
                    if trace_segments is None:
                        # Routing to existing via failed - fall back to placing new via
                        print(f"can't route to existing via, ", end="")
                        existing_via = None  # Trigger new via placement below
                    elif trace_segments:
                        new_segments.extend(trace_segments)
                        traces_added += len(trace_segments)
                        vias_reused += 1
                        reuse_success = True
                        dist = ((via_pos[0] - pad.global_x)**2 + (via_pos[1] - pad.global_y)**2)**0.5
                        print(f"reusing via at ({via_pos[0]:.2f}, {via_pos[1]:.2f}), {len(trace_segments)} segs, {dist:.2f}mm")
                    else:
                        vias_reused += 1
                        reuse_success = True
                        print(f"reusing via at pad center")
                else:
                    vias_reused += 1
                    reuse_success = True
                    print(f"reusing via at ({via_pos[0]:.2f}, {via_pos[1]:.2f})")

                if reuse_success:
                    processed_pad_ids.add(current_pad_key)
                    continue  # Move to next pad

            # Need to place a new via (pad center blocked, and either no existing via or reuse failed)
            routing_obs = get_routing_obstacles(pad_layer) if pad_layer else None
            failed_route_positions: Set[Tuple[int, int]] = set()  # Track failed positions for this pad
            via_pos = find_via_position(
                pad, obstacles, coord, max_search_radius,
                routing_obstacles=routing_obs,
                config=config,
                pad_layer=pad_layer,
                net_id=net_id,
                verbose=verbose,
                failed_route_positions=failed_route_positions,
                pending_pads=pending_pads
            )

            placement_success = False
            trace_segments = None
            via_blocked = via_pos is None
            blocked_cells = []

            if via_pos:
                via_at_pad_center = (abs(via_pos[0] - pad.global_x) < 0.001 and
                                     abs(via_pos[1] - pad.global_y) < 0.001)

                if via_at_pad_center:
                    placement_success = True
                elif pad_layer:
                    route_result = route_via_to_pad(via_pos, pad, pad_layer, net_id,
                                                       routing_obs, config, verbose=verbose,
                                                       return_blocked_cells=True)
                    if route_result.success:
                        trace_segments = route_result.segments
                        placement_success = True
                    else:
                        blocked_cells = route_result.blocked_cells
                else:
                    placement_success = True

            # If fast path failed and rip_blocker_nets enabled, try iterative rip-up
            if not placement_success and rip_blocker_nets:
                print(f"blocked, trying rip-up...", end=" ")
                result = try_place_via_with_ripup(
                    pad, pad_layer, net_id, pcb_data, config, coord,
                    max_search_radius, max_rip_nets,
                    obstacles, routing_obs,
                    via_obstacle_cache, routing_obstacles_cache, all_layers,
                    via_blocked=via_blocked,
                    blocked_cells=blocked_cells,
                    new_vias=new_vias,
                    hole_to_hole_clearance=hole_to_hole_clearance,
                    via_drill=via_drill,
                    protected_net_ids=set(net_ids),  # Protect all nets being routed (don't rip power nets we're routing)
                    verbose=verbose,
                    find_via_position_fn=find_via_position,
                    route_via_to_pad_fn=route_via_to_pad,
                    pending_pads=pending_pads
                )

                if result.success:
                    # Track ripped nets and remove their vias/segments from all_new_* lists
                    for rid in result.ripped_net_ids:
                        if rid not in ripped_net_ids:
                            ripped_net_ids.append(rid)
                        # Remove ripped net's vias and segments from accumulator lists
                        # (they were removed from pcb_data during rip-up)
                        all_new_vias[:] = [v for v in all_new_vias if v['net_id'] != rid]
                        all_new_segments[:] = [s for s in all_new_segments if s['net_id'] != rid]
                    # Note: obstacle maps were already updated incrementally in try_place_via_with_ripup

                    # Add via
                    new_vias.append({
                        'x': result.via_pos[0], 'y': result.via_pos[1],
                        'size': via_size, 'drill': via_drill,
                        'layers': ['F.Cu', 'B.Cu'], 'net_id': net_id
                    })
                    vias_placed += 1
                    processed_pad_ids.add(current_pad_key)
                    available_vias.append(result.via_pos)
                    new_segments.extend(result.segments)
                    traces_added += len(result.segments)
                    block_via_position(obstacles, result.via_pos[0], result.via_pos[1], coord,
                                       hole_to_hole_clearance, via_drill)
                    print(f"{GREEN}via at ({result.via_pos[0]:.2f}, {result.via_pos[1]:.2f}), ripped {len(result.ripped_net_ids)} nets{RESET}")
                else:
                    failed_pads += 1
                    failed_pad_infos.append((net_id, net_name, plane_layer, pad_info))
                    for rid in result.ripped_net_ids:
                        if rid not in ripped_net_ids:
                            ripped_net_ids.append(rid)
                    # Empty ripped_net_ids means nets were restored after failure
                    print(f"{RED}FAILED{RESET}")

            elif placement_success:
                # Fast path succeeded
                via_at_pad_center = (abs(via_pos[0] - pad.global_x) < 0.001 and
                                     abs(via_pos[1] - pad.global_y) < 0.001)
                new_vias.append({
                    'x': via_pos[0], 'y': via_pos[1],
                    'size': via_size, 'drill': via_drill,
                    'layers': ['F.Cu', 'B.Cu'], 'net_id': net_id
                })
                vias_placed += 1
                processed_pad_ids.add(current_pad_key)
                available_vias.append(via_pos)
                if trace_segments:
                    new_segments.extend(trace_segments)
                    traces_added += len(trace_segments)
                block_via_position(obstacles, via_pos[0], via_pos[1], coord,
                                   hole_to_hole_clearance, via_drill)

                if via_at_pad_center:
                    print(f"via at pad center")
                else:
                    print(f"via at ({via_pos[0]:.2f}, {via_pos[1]:.2f}), {len(trace_segments) if trace_segments else 0} segs")

            elif not rip_blocker_nets:
                # Fast path failed, no rip-up enabled - try fallback via reuse
                fallback_via = find_existing_via_nearby(pad, available_vias, max_search_radius)
                if fallback_via:
                    via_pos = fallback_via
                    if pad_layer:
                        routing_obs = get_routing_obstacles(pad_layer)
                        trace_segments = route_via_to_pad(via_pos, pad, pad_layer, net_id,
                                                           routing_obs, config, verbose=verbose)
                        if trace_segments is None:
                            print(f"{RED}ROUTING FAILED{RESET}")
                            failed_pads += 1
                            failed_pad_infos.append((net_id, net_name, plane_layer, pad_info))
                        elif trace_segments:
                            new_segments.extend(trace_segments)
                            traces_added += len(trace_segments)
                            vias_reused += 1
                            processed_pad_ids.add(current_pad_key)
                            print(f"fallback via at ({via_pos[0]:.2f}, {via_pos[1]:.2f}), {len(trace_segments)} segs")
                        else:
                            vias_reused += 1
                            processed_pad_ids.add(current_pad_key)
                            print(f"fallback via at ({via_pos[0]:.2f}, {via_pos[1]:.2f})")
                    else:
                        vias_reused += 1
                        processed_pad_ids.add(current_pad_key)
                        print(f"fallback via at ({via_pos[0]:.2f}, {via_pos[1]:.2f})")
                else:
                    print(f"{RED}FAILED - no valid position{RESET}")
                    failed_pads += 1
                    failed_pad_infos.append((net_id, net_name, plane_layer, pad_info))
            else:
                print(f"{RED}FAILED - no valid position{RESET}")
                failed_pads += 1
                failed_pad_infos.append((net_id, net_name, plane_layer, pad_info))

        # Step 10: Generate zone for this net (if needed)
        # For multi-net layers, defer zone generation until all nets are processed
        zone_sexpr = None
        is_multi_net_layer = layer_nets and len(layer_nets.get(plane_layer, [])) > 1
        if should_create_zone and not is_multi_net_layer:
            # Single-net layer: use full board rectangle
            zone_sexpr = generate_zone_sexpr(
                net_id=net_id,
                net_name=net_name,
                layer=plane_layer,
                polygon_points=zone_polygon,
                clearance=zone_clearance,
                min_thickness=min_thickness,
                direct_connect=True
            )
            all_zone_sexprs.append(zone_sexpr)

            # Calculate and print resistance for single-net layer
            result = analyze_single_net_plane(zone_polygon, plane_layer)
            print_single_net_resistance(result, net_name)

        # Print per-net results
        print(f"\nResults for '{net_name}':")
        was_replaced = (net_id, plane_layer) in zones_to_replace
        if should_create_zone and is_multi_net_layer:
            suffix = " (replaced existing)" if was_replaced else ""
            print(f"  Zone on {plane_layer} deferred (multi-net layer){suffix}")
        elif should_create_zone:
            suffix = " (replaced existing)" if was_replaced else ""
            print(f"  Zone created on {plane_layer}{suffix}")
        print(f"  New vias placed: {vias_placed}")
        print(f"  Existing vias reused: {vias_reused}")
        print(f"  Traces added: {traces_added}")
        if failed_pads > 0:
            print(f"  Failed pads: {failed_pads}")

        # Accumulate results
        all_new_vias.extend(new_vias)
        all_new_segments.extend(new_segments)
        total_vias_placed += vias_placed
        total_vias_reused += vias_reused
        total_traces_added += traces_added
        total_failed_pads += failed_pads
        for rid in ripped_net_ids:
            if rid not in all_ripped_net_ids:
                all_ripped_net_ids.append(rid)

        # Add new vias/segments to pcb_data so subsequent nets will avoid them
        for v in new_vias:
            pcb_data.vias.append(Via(
                x=v['x'], y=v['y'], size=v['size'], drill=v['drill'],
                layers=v['layers'], net_id=v['net_id']
            ))
        for s in new_segments:
            start = s['start']
            end = s['end']
            pcb_data.segments.append(Segment(
                start_x=start[0], start_y=start[1],
                end_x=end[0], end_y=end[1],
                width=s['width'], layer=s['layer'], net_id=s['net_id']
            ))

    # End of per-net loop

    # Generate zones for multi-net layers using Voronoi boundaries
    if layer_nets:
        for layer, nets_on_layer in layer_nets.items():
            if len(nets_on_layer) > 1:
                print(f"\n{'='*60}")
                print(f"Computing zone boundaries for multi-net layer {layer}")
                print(f"Nets: {', '.join(nets_on_layer)}")
                print(f"{'='*60}")

                zone_sexprs, debug_line_sexprs = _generate_multinet_layer_zones(
                    layer=layer,
                    nets_on_layer=nets_on_layer,
                    pcb_data=pcb_data,
                    all_new_vias=all_new_vias,
                    zone_polygon=zone_polygon,
                    board_bounds=board_bounds,
                    config=config,
                    zone_clearance=zone_clearance,
                    min_thickness=min_thickness,
                    plane_proximity_radius=plane_proximity_radius,
                    plane_proximity_cost=plane_proximity_cost,
                    plane_track_via_clearance=plane_track_via_clearance,
                    plane_max_iterations=plane_max_iterations,
                    voronoi_seed_interval=voronoi_seed_interval,
                    board_edge_clearance=board_edge_clearance,
                    debug_lines=debug_lines,
                    verbose=verbose
                )
                all_zone_sexprs.extend(zone_sexprs)
                all_debug_lines.extend(debug_line_sexprs)

    # Print overall totals
    print(f"\n{'='*60}")
    print(f"OVERALL TOTALS")
    print(f"{'='*60}")
    print(f"  Nets processed: {len(net_names)}")
    print(f"  Total new vias placed: {total_vias_placed}")
    print(f"  Total existing vias reused: {total_vias_reused}")
    print(f"  Total traces added: {total_traces_added}")
    if total_failed_pads > 0:
        print(f"  Total failed pads: {total_failed_pads}")

    if all_ripped_net_ids:
        ripped_names = []
        for rid in all_ripped_net_ids:
            net = pcb_data.nets.get(rid)
            ripped_names.append(net.name if net else f"net_{rid}")
        print(f"  Nets excluded from output: {', '.join(ripped_names)}")

    if dry_run:
        print("\nDry run - no output file written")
    else:
        _write_output_and_reroute(
            input_file=input_file,
            output_file=output_file,
            all_zone_sexprs=all_zone_sexprs,
            all_debug_lines=all_debug_lines,
            all_new_vias=all_new_vias,
            all_new_segments=all_new_segments,
            all_ripped_net_ids=all_ripped_net_ids,
            zones_to_replace=zones_to_replace,
            pcb_data=pcb_data,
            reroute_ripped_nets=reroute_ripped_nets,
            all_layers=all_layers,
            plane_layers=plane_layers,
            track_width=track_width,
            clearance=clearance,
            via_size=via_size,
            via_drill=via_drill,
            grid_step=grid_step,
            hole_to_hole_clearance=hole_to_hole_clearance,
            verbose=verbose
        )

    return (total_vias_placed, total_traces_added, total_pads_needing_vias)


def main():
    parser = argparse.ArgumentParser(
        description="Create copper zones with via stitching to net pads",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Single net:
    python route_planes.py input.kicad_pcb output.kicad_pcb --net GND --plane-layer B.Cu

    # Multiple nets (each net paired with corresponding plane layer):
    python route_planes.py input.kicad_pcb output.kicad_pcb --net GND +3.3V --plane-layer In1.Cu In2.Cu

    # With rip-up and automatic re-routing:
    python route_planes.py input.kicad_pcb output.kicad_pcb --net GND VCC --plane-layer In1.Cu In2.Cu --rip-blocker-nets --reroute-ripped-nets
"""
    )
    parser.add_argument("input_file", help="Input KiCad PCB file")
    parser.add_argument("output_file", help="Output KiCad PCB file")

    # Required options (can be multiple)
    parser.add_argument("--net", "-n", nargs="+", required=True,
                        help="Net name(s) for the plane(s) (e.g., GND VCC)")
    parser.add_argument("--plane-layer", "-p", nargs="+", required=True,
                        help="Plane layer(s) for the zone(s), one per net (e.g., In1.Cu In2.Cu)")

    # Via and track geometry
    parser.add_argument("--via-size", type=float, default=0.3, help="Via outer diameter in mm (default: 0.3)")
    parser.add_argument("--via-drill", type=float, default=0.2, help="Via drill size in mm (default: 0.2)")
    parser.add_argument("--track-width", type=float, default=0.1, help="Track width for via-to-pad connections in mm (default: 0.1)")
    parser.add_argument("--clearance", type=float, default=0.1, help="Clearance in mm (default: 0.1)")

    # Zone options
    parser.add_argument("--zone-clearance", type=float, default=0.2, help="Zone clearance from other copper in mm (default: 0.2)")
    parser.add_argument("--min-thickness", type=float, default=0.1, help="Minimum zone copper thickness in mm (default: 0.1)")

    # Algorithm options
    parser.add_argument("--grid-step", type=float, default=0.1, help="Grid resolution in mm (default: 0.1)")
    parser.add_argument("--max-search-radius", type=float, default=10.0, help="Max radius to search for valid via position in mm (default: 10.0)")
    parser.add_argument("--max-via-reuse-radius", type=float, default=1.0, help="Max radius to reuse existing via instead of placing new one in mm (default: 1.0)")
    parser.add_argument("--close-via-radius", type=float, default=None, help="Radius to check for nearby vias before placing new one (default: 2.5 * via-size)")
    parser.add_argument("--hole-to-hole-clearance", type=float, default=0.2, help="Minimum clearance between drill holes in mm (default: 0.2)")
    parser.add_argument("--layers", "-l", nargs="+", default=None,
                        help="All copper layers for routing and via span (default: F.Cu + plane-layers + B.Cu)")

    # Blocker rip-up options
    parser.add_argument("--rip-blocker-nets", action="store_true",
                        help="Identify and rip up nets blocking via placement, then retry. Ripped nets are excluded from output.")
    parser.add_argument("--max-rip-nets", type=int, default=3,
                        help="Maximum number of blocker nets to rip up (default: 3)")
    parser.add_argument("--reroute-ripped-nets", action="store_true",
                        help="Automatically re-route ripped nets after via placement")

    # Multi-net layer connection options
    parser.add_argument("--plane-proximity-radius", type=float, default=3.0,
                        help="Radius around other nets' vias to add proximity cost when routing plane connections (mm, default: 3.0)")
    parser.add_argument("--plane-proximity-cost", type=float, default=2.0,
                        help="Maximum proximity cost around other nets' vias when routing plane connections (mm equivalent, default: 2.0)")
    parser.add_argument("--plane-track-via-clearance", type=float, default=0.8,
                        help="Clearance from track center to other nets' via centers when routing MST connections (mm, default: 0.8)")
    parser.add_argument("--voronoi-seed-interval", type=float, default=2.0,
                        help="Sample interval for Voronoi seed points along plane connection routes (mm, default: 2.0)")
    parser.add_argument("--plane-max-iterations", type=int, default=200000,
                        help="Max A* iterations for routing plane connections (default: 200000)")

    # Board edge clearance
    parser.add_argument("--board-edge-clearance", type=float, default=0.5,
                        help="Clearance from board edge for zones in mm (default: 0.5)")

    # Debug options
    parser.add_argument("--dry-run", action="store_true", help="Analyze without writing output")
    parser.add_argument("--verbose", "-v", action="store_true", help="Print detailed DEBUG messages")
    parser.add_argument("--debug-lines", action="store_true", help="Output MST routes on User.1, User.2, etc. per net")

    args = parser.parse_args()

    # Default layers to F.Cu + plane-layers + B.Cu (need outer layers to reach pads)
    if args.layers is None:
        layers = ['F.Cu'] + args.plane_layer + ['B.Cu']
        # Remove duplicates while preserving order
        seen = set()
        args.layers = [l for l in layers if not (l in seen or seen.add(l))]

    # Validate net/plane-layer counts match
    if len(args.net) != len(args.plane_layer):
        print(f"Error: Number of net arguments ({len(args.net)}) must match number of plane layers ({len(args.plane_layer)})")
        print("Each net argument needs a corresponding plane layer")
        print("Use | to separate multiple nets on the same layer (e.g., --net GND 'VA19|VA11' --plane-layer In4.Cu In5.Cu)")
        return

    # Parse --net arguments: detect | separator for multi-net layers
    # Build data structures:
    #   net_names: List[str] - all individual net names (expanded)
    #   plane_layers: List[str] - layer for each net (expanded to match net_names)
    #   layer_nets: Dict[str, List[str]] - layer → list of nets on that layer
    net_names = []
    plane_layers = []
    layer_nets = {}

    for net_arg, layer in zip(args.net, args.plane_layer):
        nets_on_layer = [n.strip() for n in net_arg.split('|')]
        for net in nets_on_layer:
            net_names.append(net)
            plane_layers.append(layer)

        # Track nets per layer
        if layer not in layer_nets:
            layer_nets[layer] = []
        layer_nets[layer].extend(nets_on_layer)

    # Report multi-net layers
    for layer, nets in layer_nets.items():
        if len(nets) > 1:
            print(f"Layer {layer} has multiple nets: {', '.join(nets)}")

    create_plane(
        input_file=args.input_file,
        output_file=args.output_file,
        net_names=net_names,
        plane_layers=plane_layers,
        via_size=args.via_size,
        via_drill=args.via_drill,
        track_width=args.track_width,
        clearance=args.clearance,
        zone_clearance=args.zone_clearance,
        min_thickness=args.min_thickness,
        grid_step=args.grid_step,
        max_search_radius=args.max_search_radius,
        max_via_reuse_radius=args.max_via_reuse_radius,
        close_via_radius=args.close_via_radius,
        hole_to_hole_clearance=args.hole_to_hole_clearance,
        all_layers=args.layers,
        verbose=args.verbose,
        dry_run=args.dry_run,
        rip_blocker_nets=args.rip_blocker_nets,
        max_rip_nets=args.max_rip_nets,
        reroute_ripped_nets=args.reroute_ripped_nets,
        layer_nets=layer_nets,
        plane_proximity_radius=args.plane_proximity_radius,
        plane_proximity_cost=args.plane_proximity_cost,
        plane_track_via_clearance=args.plane_track_via_clearance,
        board_edge_clearance=args.board_edge_clearance,
        voronoi_seed_interval=args.voronoi_seed_interval,
        plane_max_iterations=args.plane_max_iterations,
        debug_lines=args.debug_lines
    )


if __name__ == "__main__":
    main()

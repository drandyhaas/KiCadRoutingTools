"""
Copper Plane Generator - Creates filled zones with via stitching to net pads.

Creates a solid copper zone on a specified layer, places vias near target pads,
and routes short traces to connect vias to pads when direct placement is blocked.

Usage:
    python route_plane.py input.kicad_pcb output.kicad_pcb --net GND --layer B.Cu
"""

import sys
import os
import re
import argparse
import math
from typing import List, Optional, Tuple, Dict, Set
from dataclasses import dataclass

# Run startup checks before other imports
from startup_checks import run_all_checks
run_all_checks()

from kicad_parser import parse_kicad_pcb, PCBData, Pad, Via, Segment
from kicad_writer import generate_via_sexpr, generate_segment_sexpr, generate_zone_sexpr
from routing_config import GridRouteConfig, GridCoord
from routing_utils import build_layer_map
from route_modification import remove_net_from_pcb_data

# Import Rust router (startup_checks ensures it's available and up-to-date)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))
from grid_router import GridObstacleMap, GridRouter


@dataclass
class ZoneInfo:
    """Information about a copper zone/pour."""
    net_id: int
    net_name: str
    layer: str


def extract_zones(pcb_file: str) -> List[ZoneInfo]:
    """Extract zone information from a KiCad PCB file.

    Args:
        pcb_file: Path to the .kicad_pcb file

    Returns:
        List of ZoneInfo objects for each zone found
    """
    with open(pcb_file, 'r', encoding='utf-8') as f:
        content = f.read()

    zones = []
    # Pattern to match zone blocks and extract net_id, net_name, and layer
    zone_pattern = r'\(zone\s+\(net\s+(\d+)\)\s+\(net_name\s+"([^"]+)"\)\s+\(layer\s+"([^"]+)"\)'

    for match in re.finditer(zone_pattern, content):
        zones.append(ZoneInfo(
            net_id=int(match.group(1)),
            net_name=match.group(2),
            layer=match.group(3)
        ))

    return zones


def check_existing_zones(zones: List[ZoneInfo], target_layer: str, target_net_name: str,
                          target_net_id: int, verbose: bool = False) -> Tuple[bool, bool]:
    """Check for existing zones on the target layer.

    Args:
        zones: List of existing zones
        target_layer: Layer we want to create zone on
        target_net_name: Net name we want
        target_net_id: Net ID we want
        verbose: Print verbose output

    Returns:
        (should_create_zone, should_continue) tuple
        - should_create_zone: False if zone already exists on same net
        - should_continue: False if zone exists on different net (error condition)
    """
    for zone in zones:
        if zone.layer == target_layer:
            if zone.net_id == target_net_id or zone.net_name == target_net_name:
                # Same net - warn and skip zone creation but continue with vias
                print(f"Warning: Zone already exists on {target_layer} for net '{zone.net_name}' (ID {zone.net_id})")
                print("  Skipping zone creation, but will place vias to connect to existing zone.")
                return (False, True)
            else:
                # Different net - error
                print(f"Error: Zone already exists on {target_layer} for DIFFERENT net '{zone.net_name}' (ID {zone.net_id})")
                print(f"  Cannot create {target_net_name} zone on same layer. Aborting.")
                return (False, False)

    # No existing zone on this layer
    return (True, True)


def resolve_net_id(pcb_data: PCBData, net_name: str) -> Optional[int]:
    """Resolve a net name to its ID.

    Args:
        pcb_data: Parsed PCB data
        net_name: Net name to look up

    Returns:
        Net ID if found, None otherwise
    """
    # Check in pcb.nets
    for net in pcb_data.nets.values():
        if net.name == net_name:
            return net.net_id

    # Check in pads_by_net (some nets may only appear in pad definitions)
    for net_id, pads in pcb_data.pads_by_net.items():
        for pad in pads:
            if pad.net_name == net_name:
                return net_id

    return None


def identify_target_pads(
    pcb_data: PCBData,
    net_id: int,
    plane_layer: str
) -> List[Dict]:
    """
    Identify pads that need via connections to the plane layer.

    Returns list of dicts with pad info and connection type:
    - "through_hole": Through-hole pad - can connect on any layer, no via needed
    - "direct": SMD pad on plane layer - zone connects directly, no via needed
    - "via_needed": SMD pad on opposite layer - needs via + trace
    """
    target_pads = []
    pads = pcb_data.pads_by_net.get(net_id, [])

    for pad in pads:
        # Check if pad has drill (through-hole)
        if pad.drill > 0:
            # Through-hole pad - directly connects to all layers including plane
            target_pads.append({
                'pad': pad,
                'type': 'through_hole',
                'needs_via': False,
                'needs_trace': False
            })
        elif plane_layer in pad.layers or "*.Cu" in pad.layers:
            # SMD pad on plane layer - direct zone connection
            target_pads.append({
                'pad': pad,
                'type': 'direct',
                'needs_via': False,
                'needs_trace': False
            })
        else:
            # SMD pad NOT on plane layer - needs via
            # Get the pad's actual layer for trace routing
            pad_layer = None
            for layer in pad.layers:
                if layer.endswith('.Cu') and not layer.startswith('*'):
                    pad_layer = layer
                    break

            target_pads.append({
                'pad': pad,
                'type': 'via_needed',
                'needs_via': True,
                'needs_trace': True,  # May need trace if via can't be at pad center
                'pad_layer': pad_layer
            })

    return target_pads


def build_via_obstacle_map(
    pcb_data: PCBData,
    config: GridRouteConfig,
    exclude_net_id: int
) -> GridObstacleMap:
    """
    Build obstacle map for via placement.

    Blocks:
    - Existing vias (all nets - via-via clearance)
    - Pads on all layers (except target net pads)
    - Tracks on all layers (except target net)
    - Board edge clearance
    - Through-hole pad drills (hole-to-hole clearance)
    """
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)
    layer_map = build_layer_map(config.layers)

    obstacles = GridObstacleMap(num_layers)

    # Precompute expansion for via-via clearance (squared, in grid units)
    via_via_expansion_mm = config.via_size + config.clearance
    via_via_radius_sq = (via_via_expansion_mm / config.grid_step) ** 2
    via_via_radius_int = int(math.ceil(math.sqrt(via_via_radius_sq)))

    # Add existing vias as obstacles (including same net - can't place via too close to another)
    for via in pcb_data.vias:
        gx, gy = coord.to_grid(via.x, via.y)
        # Block via placement within via-via clearance
        for ex in range(-via_via_radius_int, via_via_radius_int + 1):
            for ey in range(-via_via_radius_int, via_via_radius_int + 1):
                if ex*ex + ey*ey <= via_via_radius_sq:
                    obstacles.add_blocked_via(gx + ex, gy + ey)

    # Add existing segments as obstacles (via can't overlap with tracks on ANY layer)
    # Since vias span all layers, we must check segments on all copper layers, not just config.layers
    for seg in pcb_data.segments:
        if seg.net_id == exclude_net_id:
            continue
        # Include any copper layer (*.Cu)
        if not seg.layer.endswith('.Cu'):
            continue
        # Use actual segment width for clearance calculation (not config.track_width)
        seg_expansion_mm = config.via_size / 2 + seg.width / 2 + config.clearance
        _add_segment_via_obstacle(obstacles, seg, coord, seg_expansion_mm)

    # Add pads as obstacles (excluding target net pads)
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id == exclude_net_id:
            continue
        for pad in pads:
            _add_pad_via_obstacle(obstacles, pad, coord, config)

    # Add board edge via blocking
    _add_board_edge_via_obstacles(obstacles, pcb_data, config)

    # Add hole-to-hole clearance blocking for existing drills
    _add_drill_hole_via_obstacles(obstacles, pcb_data, config, exclude_net_id)

    return obstacles


def _add_segment_via_obstacle(obstacles: GridObstacleMap, seg: Segment,
                               coord: GridCoord, expansion_mm: float):
    """Add a segment as via blocking obstacle."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

    # Compute squared radius in grid units (avoids precision loss)
    radius_sq = (expansion_mm / coord.grid_step) ** 2

    # Use Bresenham-style line blocking
    dx = abs(gx2 - gx1)
    dy = abs(gy2 - gy1)
    sx = 1 if gx1 < gx2 else -1
    sy = 1 if gy1 < gy2 else -1

    x, y = gx1, gy1
    if dx > dy:
        err = dx / 2
        while x != gx2:
            _block_via_circle(obstacles, x, y, radius_sq)
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2
        while y != gy2:
            _block_via_circle(obstacles, x, y, radius_sq)
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    _block_via_circle(obstacles, gx2, gy2, radius_sq)


def _block_via_circle(obstacles: GridObstacleMap, cx: int, cy: int, radius_sq: float):
    """Block via placement in a circular region.

    Args:
        radius_sq: Squared radius in grid units (avoids precision loss from rounding before squaring)
    """
    # Use ceiling of sqrt to ensure we check all potentially blocked cells
    radius_int = int(math.ceil(math.sqrt(radius_sq)))
    for ex in range(-radius_int, radius_int + 1):
        for ey in range(-radius_int, radius_int + 1):
            if ex*ex + ey*ey <= radius_sq:
                obstacles.add_blocked_via(cx + ex, cy + ey)


def _add_pad_via_obstacle(obstacles: GridObstacleMap, pad: Pad,
                           coord: GridCoord, config: GridRouteConfig):
    """Add a pad as via blocking obstacle."""
    gx, gy = coord.to_grid(pad.global_x, pad.global_y)

    # Use pad size + clearance
    # Compute squared radius in grid units (avoids precision loss from rounding before squaring)
    pad_radius = max(pad.size_x, pad.size_y) / 2
    expansion_mm = pad_radius + config.via_size / 2 + config.clearance
    radius_sq = (expansion_mm / config.grid_step) ** 2

    _block_via_circle(obstacles, gx, gy, radius_sq)


def _add_board_edge_via_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                                    config: GridRouteConfig):
    """Block via placement near board edges."""
    board_bounds = pcb_data.board_info.board_bounds
    if not board_bounds:
        return

    coord = GridCoord(config.grid_step)
    min_x, min_y, max_x, max_y = board_bounds

    edge_clearance = config.board_edge_clearance if config.board_edge_clearance > 0 else config.clearance
    via_edge_clearance = edge_clearance + config.via_size / 2
    via_expand = coord.to_grid_dist(via_edge_clearance)

    gmin_x, gmin_y = coord.to_grid(min_x, min_y)
    gmax_x, gmax_y = coord.to_grid(max_x, max_y)
    grid_margin = via_expand + 5

    # Block edges
    for gx in range(gmin_x - grid_margin, gmax_x + grid_margin + 1):
        for gy in range(gmin_y - grid_margin, gmax_y + grid_margin + 1):
            # Outside board + margin
            if gx < gmin_x + via_expand or gx > gmax_x - via_expand or \
               gy < gmin_y + via_expand or gy > gmax_y - via_expand:
                obstacles.add_blocked_via(gx, gy)


def _add_drill_hole_via_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                                    config: GridRouteConfig, exclude_net_id: int):
    """Block via placement near existing drill holes."""
    if config.hole_to_hole_clearance <= 0:
        return

    coord = GridCoord(config.grid_step)

    # Collect drill holes
    drill_holes = []

    for via in pcb_data.vias:
        drill_holes.append((via.x, via.y, via.drill))

    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id == exclude_net_id:
            continue
        for pad in pads:
            if pad.drill > 0:
                drill_holes.append((pad.global_x, pad.global_y, pad.drill))

    for hx, hy, drill_dia in drill_holes:
        required_dist = drill_dia / 2 + config.via_drill / 2 + config.hole_to_hole_clearance
        radius_sq = (required_dist / config.grid_step) ** 2
        gx, gy = coord.to_grid(hx, hy)
        _block_via_circle(obstacles, gx, gy, radius_sq)


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
    failed_route_positions: Optional[Set[Tuple[int, int]]] = None
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


def block_via_position(obstacles: GridObstacleMap, via_x: float, via_y: float,
                        coord: GridCoord, hole_to_hole_clearance: float, via_drill: float):
    """Block the area around a newly placed via for hole-to-hole clearance.

    Args:
        obstacles: The obstacle map to update
        via_x, via_y: Position of the placed via
        coord: Grid coordinate converter
        hole_to_hole_clearance: Minimum clearance between drill holes
        via_drill: Drill diameter of the via
    """
    gx, gy = coord.to_grid(via_x, via_y)
    # Required distance: (this_drill/2) + (other_drill/2) + clearance
    # Since we're placing vias of the same size, it's: via_drill + clearance
    required_dist = via_drill + hole_to_hole_clearance
    radius_sq = (required_dist / coord.grid_step) ** 2
    _block_via_circle(obstacles, gx, gy, radius_sq)


def build_routing_obstacle_map(
    pcb_data: PCBData,
    config: GridRouteConfig,
    exclude_net_id: int,
    route_layer: str,
    skip_pad_blocking: bool = False
) -> GridObstacleMap:
    """
    Build obstacle map for A* routing on a specific layer.

    Blocks:
    - Pads on the route layer (except target net pads) - skipped if skip_pad_blocking
    - Segments on the route layer (except target net)
    - Vias (they occupy space on all layers)

    Args:
        skip_pad_blocking: If True, don't block based on pad clearances.
                          Use this for plane via connections where DRC is lenient.
    """
    coord = GridCoord(config.grid_step)
    # Single layer routing
    num_layers = 1
    layer_idx = 0

    obstacles = GridObstacleMap(num_layers)

    # Add pads on this layer as obstacles (excluding target net)
    # Skip this entirely for plane connections where we're more lenient
    if not skip_pad_blocking:
        for net_id, pads in pcb_data.pads_by_net.items():
            if net_id == exclude_net_id:
                continue
            for pad in pads:
                # Check if pad is on the route layer
                if route_layer in pad.layers or "*.Cu" in pad.layers:
                    gx, gy = coord.to_grid(pad.global_x, pad.global_y)
                    # Use rectangular expansion based on actual pad dimensions
                    # (not circular based on max dimension)
                    # Use floor (int) instead of round to avoid over-blocking at grid boundaries
                    half_width = pad.size_x / 2
                    half_height = pad.size_y / 2
                    margin = config.track_width / 2 + config.clearance
                    expand_x = int((half_width + margin) / config.grid_step)
                    expand_y = int((half_height + margin) / config.grid_step)
                    for ex in range(-expand_x, expand_x + 1):
                        for ey in range(-expand_y, expand_y + 1):
                            obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)

    # Add segments on this layer as obstacles (excluding target net)
    # Use actual segment width for proper clearance calculation
    for seg in pcb_data.segments:
        if seg.net_id == exclude_net_id:
            continue
        if seg.layer != route_layer:
            continue
        # Clearance needed: our track half-width + existing segment half-width + clearance
        seg_expansion_mm = config.track_width / 2 + seg.width / 2 + config.clearance
        seg_expansion_grid = max(1, coord.to_grid_dist(seg_expansion_mm))
        _add_segment_routing_obstacle(obstacles, seg, coord, layer_idx, seg_expansion_grid)

    # Add vias as obstacles (they block all layers)
    for via in pcb_data.vias:
        if via.net_id == exclude_net_id:
            continue
        gx, gy = coord.to_grid(via.x, via.y)
        via_expansion = coord.to_grid_dist(via.size / 2 + config.track_width / 2 + config.clearance)
        for ex in range(-via_expansion, via_expansion + 1):
            for ey in range(-via_expansion, via_expansion + 1):
                if ex*ex + ey*ey <= via_expansion * via_expansion:
                    obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)

    return obstacles


def _add_segment_routing_obstacle(obstacles: GridObstacleMap, seg: Segment,
                                    coord: GridCoord, layer_idx: int, expansion_grid: int):
    """Add a segment as a routing obstacle on a specific layer."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

    # Bresenham line with expansion
    dx = abs(gx2 - gx1)
    dy = abs(gy2 - gy1)
    sx = 1 if gx1 < gx2 else -1
    sy = 1 if gy1 < gy2 else -1

    x, y = gx1, gy1
    if dx > dy:
        err = dx / 2
        while x != gx2:
            _block_routing_circle(obstacles, x, y, layer_idx, expansion_grid)
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2
        while y != gy2:
            _block_routing_circle(obstacles, x, y, layer_idx, expansion_grid)
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    _block_routing_circle(obstacles, gx2, gy2, layer_idx, expansion_grid)


def _block_routing_circle(obstacles: GridObstacleMap, cx: int, cy: int,
                           layer_idx: int, radius: int):
    """Block cells in a circular region for routing."""
    for ex in range(-radius, radius + 1):
        for ey in range(-radius, radius + 1):
            if ex*ex + ey*ey <= radius * radius:
                obstacles.add_blocked_cell(cx + ex, cy + ey, layer_idx)


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


def create_via_to_pad_trace(
    via_pos: Tuple[float, float],
    pad: Pad,
    pad_layer: str,
    net_id: int,
    track_width: float
) -> Optional[Dict]:
    """
    Create a direct trace from via position to pad center.

    DEPRECATED: Use route_via_to_pad() for A* routed traces instead.

    Returns segment dict or None if via is at pad center.
    """
    # Check if via is at pad center (within tolerance)
    if abs(via_pos[0] - pad.global_x) < 0.001 and abs(via_pos[1] - pad.global_y) < 0.001:
        return None  # Via is at pad center, no trace needed

    return {
        'start': via_pos,
        'end': (pad.global_x, pad.global_y),
        'width': track_width,
        'layer': pad_layer,
        'net_id': net_id
    }


def _point_to_segment_dist_sq(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
    """Calculate squared distance from point (px, py) to line segment (x1,y1)-(x2,y2)."""
    dx = x2 - x1
    dy = y2 - y1
    length_sq = dx * dx + dy * dy

    if length_sq < 1e-10:
        # Degenerate segment (point)
        return (px - x1) ** 2 + (py - y1) ** 2

    # Project point onto line, clamped to segment
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / length_sq))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy

    return (px - proj_x) ** 2 + (py - proj_y) ** 2


def find_via_position_blocker(
    via_x: float,
    via_y: float,
    pcb_data: PCBData,
    config: GridRouteConfig,
    exclude_net_id: int
) -> Optional[int]:
    """
    Find the net that is blocking via placement at a specific position.

    Checks segments and vias from other nets to find what's blocking
    the given via position.

    Args:
        via_x, via_y: Position where via placement is blocked
        pcb_data: PCB data with all segments/vias
        config: Routing configuration
        exclude_net_id: Net ID to exclude (the target net)

    Returns:
        Net ID of the closest blocker, or None if no blocker found
    """
    best_blocker = None
    best_dist_sq = float('inf')

    # Check segments
    for seg in pcb_data.segments:
        if seg.net_id == exclude_net_id:
            continue
        dist_sq = _point_to_segment_dist_sq(via_x, via_y, seg.start_x, seg.start_y, seg.end_x, seg.end_y)
        clearance_needed = config.via_size / 2 + seg.width / 2 + config.clearance
        if dist_sq < clearance_needed ** 2 and dist_sq < best_dist_sq:
            best_dist_sq = dist_sq
            best_blocker = seg.net_id

    # Check vias
    for via in pcb_data.vias:
        if via.net_id == exclude_net_id:
            continue
        dx = via.x - via_x
        dy = via.y - via_y
        dist_sq = dx * dx + dy * dy
        clearance_needed = config.via_size / 2 + via.size / 2 + config.clearance
        if dist_sq < clearance_needed ** 2 and dist_sq < best_dist_sq:
            best_dist_sq = dist_sq
            best_blocker = via.net_id

    return best_blocker


def find_route_blocker_from_frontier(
    blocked_cells: List[Tuple[int, int, int]],
    pcb_data: PCBData,
    config: GridRouteConfig,
    exclude_net_id: int
) -> Optional[int]:
    """
    Find the net most responsible for blocking a route based on frontier data.

    Uses the blocked_cells from route_with_frontier to identify which net's
    segments/vias are blocking the most cells on the search frontier.

    Args:
        blocked_cells: List of (gx, gy, layer) cells from route_with_frontier
        pcb_data: PCB data with all segments/vias
        config: Routing configuration
        exclude_net_id: Net ID to exclude (the target net)

    Returns:
        Net ID of the top blocker, or None if no blocker found
    """
    if not blocked_cells:
        return None

    coord = GridCoord(config.grid_step)
    blocked_set = set(blocked_cells)

    # Count how many blocked cells each net is responsible for
    net_block_count: Dict[int, int] = {}

    # Check segments
    expansion_mm = config.track_width / 2 + config.clearance
    expansion_grid = max(1, coord.to_grid_dist(expansion_mm))

    for seg in pcb_data.segments:
        if seg.net_id == exclude_net_id:
            continue

        # Get layer index (assume single layer routing, layer 0)
        layer_idx = 0

        # Trace along segment and check for blocked cells
        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

        # Simple line walk
        dx = abs(gx2 - gx1)
        dy = abs(gy2 - gy1)
        sx = 1 if gx1 < gx2 else -1
        sy = 1 if gy1 < gy2 else -1
        err = dx - dy

        gx, gy = gx1, gy1
        count = 0
        while True:
            # Check expansion around this point
            for ex in range(-expansion_grid, expansion_grid + 1):
                for ey in range(-expansion_grid, expansion_grid + 1):
                    cell = (gx + ex, gy + ey, layer_idx)
                    if cell in blocked_set:
                        count += 1

            if gx == gx2 and gy == gy2:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                gx += sx
            if e2 < dx:
                err += dx
                gy += sy

        if count > 0:
            net_block_count[seg.net_id] = net_block_count.get(seg.net_id, 0) + count

    # Check vias
    via_expansion_grid = max(1, coord.to_grid_dist(config.via_size / 2 + config.track_width / 2 + config.clearance))

    for via in pcb_data.vias:
        if via.net_id == exclude_net_id:
            continue

        gx, gy = coord.to_grid(via.x, via.y)
        count = 0
        for ex in range(-via_expansion_grid, via_expansion_grid + 1):
            for ey in range(-via_expansion_grid, via_expansion_grid + 1):
                if ex * ex + ey * ey <= via_expansion_grid * via_expansion_grid:
                    # Vias block all layers, but for single-layer routing check layer 0
                    cell = (gx + ex, gy + ey, 0)
                    if cell in blocked_set:
                        count += 1

        if count > 0:
            net_block_count[via.net_id] = net_block_count.get(via.net_id, 0) + count

    if not net_block_count:
        return None

    # Return the net with the highest block count
    return max(net_block_count.keys(), key=lambda k: net_block_count[k])


def find_best_via_position_with_blocker(
    pad: Pad,
    obstacles: GridObstacleMap,
    coord: GridCoord,
    max_search_radius: float,
    pcb_data: PCBData,
    config: GridRouteConfig,
    exclude_net_id: int
) -> Tuple[Optional[Tuple[float, float]], Optional[int]]:
    """
    Find the best (closest) via position near a pad, and identify blocker if blocked.

    Returns:
        (via_pos, blocker_net_id) where:
        - via_pos is (x, y) if found, None if all positions blocked
        - blocker_net_id is the net blocking the closest position (if blocked), None otherwise
    """
    pad_gx, pad_gy = coord.to_grid(pad.global_x, pad.global_y)

    # Try pad center first
    if not obstacles.is_via_blocked(pad_gx, pad_gy):
        return ((pad.global_x, pad.global_y), None)

    # Find blocker at pad center
    center_blocker = find_via_position_blocker(pad.global_x, pad.global_y, pcb_data, config, exclude_net_id)

    # Search outward for valid position
    max_radius_grid = coord.to_grid_dist(max_search_radius)
    best_dist_sq = float('inf')
    best_pos = None

    for radius in range(1, max_radius_grid + 1):
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if abs(dx) != radius and abs(dy) != radius:
                    continue

                gx, gy = pad_gx + dx, pad_gy + dy
                if not obstacles.is_via_blocked(gx, gy):
                    dist_sq = dx * dx + dy * dy
                    if dist_sq < best_dist_sq:
                        best_dist_sq = dist_sq
                        best_pos = coord.to_float(gx, gy)

        # If we found a position at this radius, no need to search further
        if best_pos is not None:
            break

    if best_pos is not None:
        return (best_pos, None)

    # All positions blocked - return blocker for closest position (pad center)
    return (None, center_blocker)


@dataclass
class ViaPlacementResult:
    """Result of via placement attempt with potential rip-up."""
    success: bool
    via_pos: Optional[Tuple[float, float]]
    segments: List[Dict]
    ripped_net_ids: List[int]  # Nets that were ripped up to achieve success
    via_at_pad_center: bool


def try_place_via_with_ripup(
    pad: Pad,
    pad_layer: str,
    net_id: int,
    pcb_data: PCBData,
    config: GridRouteConfig,
    coord: GridCoord,
    max_search_radius: float,
    max_rip_nets: int,
    obstacles: GridObstacleMap,
    routing_obstacles: Optional[GridObstacleMap],
    build_via_obstacles_func,
    build_routing_obstacles_func,
    via_blocked: bool,  # True if via placement failed, False if routing failed
    blocked_cells: Optional[List[Tuple[int, int, int]]] = None,  # Frontier from failed route
    verbose: bool = False
) -> ViaPlacementResult:
    """
    Try to place a via and route to pad, ripping up blockers as needed.

    Called AFTER the fast path already failed. On first iteration, just finds and
    rips up the blocker without re-searching (since we already know search failed).
    """
    ripped_net_ids = []
    failed_route_positions: Set[Tuple[int, int]] = set()  # Track positions where routing failed

    for attempt in range(max_rip_nets):
        if attempt == 0:
            # First iteration: skip search, we already know it failed
            # Just find the blocker directly
            if via_blocked:
                blocker = find_via_position_blocker(
                    pad.global_x, pad.global_y, pcb_data, config, net_id
                )
            else:
                # Route was blocked - use frontier data
                blocker = find_route_blocker_from_frontier(
                    blocked_cells or [], pcb_data, config, net_id
                )
        else:
            # After rip-up, try again (skip positions near previously failed ones)
            via_pos = find_via_position(
                pad, obstacles, coord, max_search_radius,
                routing_obstacles=routing_obstacles,
                config=config,
                pad_layer=pad_layer,
                net_id=net_id,
                verbose=False,
                failed_route_positions=failed_route_positions
            )

            if via_pos:
                via_at_pad_center = (abs(via_pos[0] - pad.global_x) < 0.001 and
                                     abs(via_pos[1] - pad.global_y) < 0.001)
                if via_at_pad_center or not pad_layer:
                    return ViaPlacementResult(
                        success=True, via_pos=via_pos, segments=[],
                        ripped_net_ids=ripped_net_ids, via_at_pad_center=via_at_pad_center
                    )

                # Try routing
                route_result = route_via_to_pad(
                    via_pos, pad, pad_layer, net_id,
                    routing_obstacles, config,
                    verbose=False, return_blocked_cells=True
                )

                if route_result.success:
                    return ViaPlacementResult(
                        success=True, via_pos=via_pos, segments=route_result.segments,
                        ripped_net_ids=ripped_net_ids, via_at_pad_center=False
                    )

                # Routing failed - find blocker from frontier
                blocker = find_route_blocker_from_frontier(
                    route_result.blocked_cells, pcb_data, config, net_id
                )
            else:
                # Via placement blocked
                blocker = find_via_position_blocker(
                    pad.global_x, pad.global_y, pcb_data, config, net_id
                )

        if blocker is None:
            break

        # Rip up blocker
        blocker_net = pcb_data.nets.get(blocker)
        blocker_name = blocker_net.name if blocker_net else f"net_{blocker}"
        print(f"ripping {blocker_name}...", end=" ")
        removed_segs, removed_vias = remove_net_from_pcb_data(pcb_data, blocker)
        ripped_net_ids.append(blocker)

        # Full rebuild of obstacle maps
        obstacles = build_via_obstacles_func()
        if pad_layer:
            routing_obstacles = build_routing_obstacles_func(pad_layer)

    return ViaPlacementResult(
        success=False, via_pos=None, segments=[],
        ripped_net_ids=ripped_net_ids, via_at_pad_center=False
    )


def filter_nets_from_content(content: str, net_ids_to_exclude: List[int]) -> str:
    """
    Filter out segments and vias for specific net IDs from PCB file content.

    Args:
        content: Raw PCB file content
        net_ids_to_exclude: List of net IDs to remove

    Returns:
        Filtered content with those nets' segments and vias removed
    """
    if not net_ids_to_exclude:
        return content

    net_id_set = set(net_ids_to_exclude)
    lines = content.split('\n')
    result_lines = []

    i = 0
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()

        # Check if this starts a segment or via (may be multi-line)
        if stripped == '(segment' or stripped.startswith('(segment ') or \
           stripped == '(via' or stripped.startswith('(via '):
            # Collect all lines of this element
            element_lines = [line]
            open_parens = line.count('(') - line.count(')')
            while open_parens > 0 and i + 1 < len(lines):
                i += 1
                element_lines.append(lines[i])
                open_parens += lines[i].count('(') - lines[i].count(')')

            # Check if any line contains net ID to exclude
            element_text = '\n'.join(element_lines)
            net_match = re.search(r'\(net\s+(\d+)\)', element_text)
            if net_match:
                element_net_id = int(net_match.group(1))
                if element_net_id in net_id_set:
                    # Skip this element entirely
                    i += 1
                    continue

            # Keep this element
            result_lines.extend(element_lines)
        else:
            result_lines.append(line)

        i += 1

    return '\n'.join(result_lines)


def write_plane_output(
    input_file: str,
    output_file: str,
    zone_sexpr: Optional[str],
    new_vias: List[Dict],
    new_segments: List[Dict],
    exclude_net_ids: List[int] = None
) -> bool:
    """Write the complete output file with zone (optional), vias, and traces.

    Args:
        exclude_net_ids: Optional list of net IDs to exclude from output
                         (their segments/vias will be filtered out)
    """
    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()

    # Filter out excluded nets if specified
    if exclude_net_ids:
        content = filter_nets_from_content(content, exclude_net_ids)

    # Build routing text
    elements = []

    # Add zone if provided
    if zone_sexpr:
        elements.append(zone_sexpr)

    # Add vias
    for via in new_vias:
        elements.append(generate_via_sexpr(
            via['x'], via['y'], via['size'], via['drill'],
            via['layers'], via['net_id']
        ))

    # Add segments
    for seg in new_segments:
        elements.append(generate_segment_sexpr(
            seg['start'], seg['end'],
            seg['width'], seg['layer'], seg['net_id']
        ))

    if not elements:
        # Nothing to add, just copy the file
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(content)
        return True

    routing_text = '\n'.join(elements)

    # Insert before final paren
    last_paren = content.rfind(')')
    if last_paren == -1:
        print("Error: Could not find closing parenthesis in PCB file")
        return False

    new_content = content[:last_paren] + '\n' + routing_text + '\n' + content[last_paren:]

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(new_content)

    return True


def create_plane(
    input_file: str,
    output_file: str,
    net_name: str,
    plane_layer: str,
    via_size: float = 0.3,
    via_drill: float = 0.2,
    track_width: float = 0.2,
    clearance: float = 0.1,
    zone_clearance: float = 0.2,
    min_thickness: float = 0.1,
    grid_step: float = 0.1,
    max_search_radius: float = 10.0,
    max_via_reuse_radius: float = 1.0,
    hole_to_hole_clearance: float = 0.2,
    all_layers: List[str] = None,
    verbose: bool = False,
    dry_run: bool = False,
    rip_blocker_nets: bool = False,
    max_rip_nets: int = 3
) -> Tuple[int, int, int]:
    """
    Create a copper plane zone and place vias to connect target pads.

    Args:
        rip_blocker_nets: If True, identify and temporarily remove nets blocking
                          via placement, then retry. Ripped nets excluded from output.
        max_rip_nets: Maximum number of blocker nets to rip up.

    Returns:
        (vias_placed, traces_added, pads_needing_vias)
    """
    if all_layers is None:
        all_layers = ['F.Cu', 'B.Cu']

    # Step 1: Load PCB and find net
    print(f"Loading PCB from {input_file}...")
    pcb_data = parse_kicad_pcb(input_file)

    net_id = resolve_net_id(pcb_data, net_name)
    if net_id is None:
        print(f"Error: Net '{net_name}' not found in PCB")
        return (0, 0, 0)

    print(f"Found net '{net_name}' with ID {net_id}")

    # Step 2: Check for existing zones on target layer
    existing_zones = extract_zones(input_file)
    should_create_zone, should_continue = check_existing_zones(
        existing_zones, plane_layer, net_name, net_id, verbose
    )

    if not should_continue:
        # Zone exists on different net - abort
        return (0, 0, 0)

    # Step 3: Get board bounds for zone polygon
    board_bounds = pcb_data.board_info.board_bounds
    if not board_bounds:
        print("Error: Could not determine board bounds")
        return (0, 0, 0)

    min_x, min_y, max_x, max_y = board_bounds
    print(f"Board bounds: ({min_x:.2f}, {min_y:.2f}) to ({max_x:.2f}, {max_y:.2f})")

    # Create zone polygon from board bounds (only used if creating new zone)
    zone_polygon = [
        (min_x, min_y),
        (max_x, min_y),
        (max_x, max_y),
        (min_x, max_y)
    ]

    # Step 4: Identify target pads for via placement
    target_pads = identify_target_pads(pcb_data, net_id, plane_layer)

    pads_through_hole = sum(1 for p in target_pads if p['type'] == 'through_hole')
    pads_direct = sum(1 for p in target_pads if p['type'] == 'direct')
    pads_need_via = sum(1 for p in target_pads if p['type'] == 'via_needed')

    print(f"\nPad analysis for net '{net_name}':")
    print(f"  Through-hole pads (no via needed): {pads_through_hole}")
    print(f"  SMD pads on {plane_layer} (no via needed): {pads_direct}")
    print(f"  SMD pads on other layers (via needed): {pads_need_via}")

    # Step 5: Collect existing vias on target net (for reuse)
    existing_net_vias: List[Tuple[float, float]] = []
    for via in pcb_data.vias:
        if via.net_id == net_id:
            existing_net_vias.append((via.x, via.y))

    if verbose and existing_net_vias:
        print(f"  Existing vias on net '{net_name}': {len(existing_net_vias)}")

    # Step 6: Build obstacle map for via placement
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

    if pads_need_via > 0:
        print("\nBuilding obstacle map for via placement...")
        obstacles = build_via_obstacle_map(pcb_data, config, net_id)
    else:
        obstacles = None

    # Step 7: Build routing obstacle maps for each unique pad layer
    routing_obstacles_cache: Dict[str, GridObstacleMap] = {}

    def get_routing_obstacles(layer: str) -> GridObstacleMap:
        """Get or create routing obstacle map for a layer."""
        if layer not in routing_obstacles_cache:
            if verbose:
                print(f"  Building routing obstacle map for {layer}...")
            # Enable pad blocking so A* routes around other net pads
            routing_obstacles_cache[layer] = build_routing_obstacle_map(
                pcb_data, config, net_id, layer, skip_pad_blocking=False
            )
        return routing_obstacles_cache[layer]

    # Step 8: Place vias near each target pad (or reuse existing)
    new_vias = []
    new_segments = []
    vias_placed = 0
    vias_reused = 0
    traces_added = 0
    failed_pads = 0
    ripped_net_ids: List[int] = []  # Nets ripped, excluded from output

    # Track all available vias (existing + newly placed) for reuse
    available_vias = list(existing_net_vias)

    # Create obstacle map builder functions for iterative rip-up
    def build_via_obstacles():
        return build_via_obstacle_map(pcb_data, config, net_id)

    def build_routing_obstacles_for_layer(layer: str):
        return build_routing_obstacle_map(pcb_data, config, net_id, layer, skip_pad_blocking=False)

    for pad_info in target_pads:
        if not pad_info['needs_via']:
            continue

        pad = pad_info['pad']
        pad_layer = pad_info.get('pad_layer')

        print(f"  Pad {pad.component_ref}.{pad.pad_number}...", end=" ")

        # First check if there's an existing or already-placed via nearby (within reuse radius)
        existing_via = find_existing_via_nearby(pad, available_vias, max_via_reuse_radius)

        if existing_via:
            # Reuse existing via - route trace to connect
            via_pos = existing_via

            if pad_layer:
                routing_obs = get_routing_obstacles(pad_layer)
                route_result = route_via_to_pad(via_pos, pad, pad_layer, net_id,
                                                   routing_obs, config, verbose=verbose,
                                                   return_blocked_cells=True)
                trace_segments = route_result.segments if route_result.success else None
                if trace_segments is None:
                    # Routing failed - if rip_blocker_nets, try with iterative rip-up
                    if rip_blocker_nets:
                        print(f"routing blocked, trying rip-up...", end=" ")
                        result = try_place_via_with_ripup(
                            pad, pad_layer, net_id, pcb_data, config, coord,
                            max_search_radius, max_rip_nets,
                            obstacles, routing_obs,
                            build_via_obstacles, build_routing_obstacles_for_layer,
                            via_blocked=False,  # Route was blocked, not via placement
                            blocked_cells=route_result.blocked_cells,
                            verbose=verbose
                        )
                        if result.success:
                            # Track ripped nets
                            for rid in result.ripped_net_ids:
                                if rid not in ripped_net_ids:
                                    ripped_net_ids.append(rid)
                            # Invalidate routing cache since pcb_data changed
                            routing_obstacles_cache.clear()
                            # Add via and segments
                            new_vias.append({
                                'x': result.via_pos[0], 'y': result.via_pos[1],
                                'size': via_size, 'drill': via_drill,
                                'layers': ['F.Cu', 'B.Cu'], 'net_id': net_id
                            })
                            vias_placed += 1
                            new_segments.extend(result.segments)
                            traces_added += len(result.segments)
                            available_vias.append(result.via_pos)
                            rip_info = f", ripped {len(result.ripped_net_ids)} nets" if result.ripped_net_ids else ""
                            print(f"\033[92mSUCCESS via at ({result.via_pos[0]:.2f}, {result.via_pos[1]:.2f}){rip_info}\033[0m")
                        else:
                            print(f"\033[91mFAILED after {len(result.ripped_net_ids)} rip-ups\033[0m")
                            failed_pads += 1
                            for rid in result.ripped_net_ids:
                                if rid not in ripped_net_ids:
                                    ripped_net_ids.append(rid)
                    else:
                        print(f"\033[91mROUTING FAILED\033[0m")
                        failed_pads += 1
                elif trace_segments:
                    new_segments.extend(trace_segments)
                    traces_added += len(trace_segments)
                    vias_reused += 1
                    dist = ((via_pos[0] - pad.global_x)**2 + (via_pos[1] - pad.global_y)**2)**0.5
                    print(f"reusing via at ({via_pos[0]:.2f}, {via_pos[1]:.2f}), {len(trace_segments)} segs, {dist:.2f}mm")
                else:
                    vias_reused += 1
                    print(f"reusing via at pad center")
            else:
                vias_reused += 1
                print(f"reusing via at ({via_pos[0]:.2f}, {via_pos[1]:.2f})")
        else:
            # Need to place a new via - try fast path first
            routing_obs = get_routing_obstacles(pad_layer) if pad_layer else None
            failed_route_positions: Set[Tuple[int, int]] = set()  # Track failed positions for this pad
            via_pos = find_via_position(
                pad, obstacles, coord, max_search_radius,
                routing_obstacles=routing_obs,
                config=config,
                pad_layer=pad_layer,
                net_id=net_id,
                verbose=verbose,
                failed_route_positions=failed_route_positions
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
                    build_via_obstacles, build_routing_obstacles_for_layer,
                    via_blocked=via_blocked,
                    blocked_cells=blocked_cells,
                    verbose=verbose
                )

                if result.success:
                    # Track ripped nets
                    for rid in result.ripped_net_ids:
                        if rid not in ripped_net_ids:
                            ripped_net_ids.append(rid)

                    # Invalidate caches if we ripped nets
                    if result.ripped_net_ids:
                        routing_obstacles_cache.clear()
                        obstacles = build_via_obstacles()

                    # Add via
                    new_vias.append({
                        'x': result.via_pos[0], 'y': result.via_pos[1],
                        'size': via_size, 'drill': via_drill,
                        'layers': ['F.Cu', 'B.Cu'], 'net_id': net_id
                    })
                    vias_placed += 1
                    available_vias.append(result.via_pos)
                    new_segments.extend(result.segments)
                    traces_added += len(result.segments)
                    block_via_position(obstacles, result.via_pos[0], result.via_pos[1], coord,
                                       hole_to_hole_clearance, via_drill)
                    print(f"\033[92mvia at ({result.via_pos[0]:.2f}, {result.via_pos[1]:.2f}), ripped {len(result.ripped_net_ids)} nets\033[0m")
                else:
                    failed_pads += 1
                    for rid in result.ripped_net_ids:
                        if rid not in ripped_net_ids:
                            ripped_net_ids.append(rid)
                    print(f"\033[91mFAILED after {len(result.ripped_net_ids)} rip-ups\033[0m")

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
                            print(f"\033[91mROUTING FAILED\033[0m")
                            failed_pads += 1
                        elif trace_segments:
                            new_segments.extend(trace_segments)
                            traces_added += len(trace_segments)
                            vias_reused += 1
                            print(f"fallback via at ({via_pos[0]:.2f}, {via_pos[1]:.2f}), {len(trace_segments)} segs")
                        else:
                            vias_reused += 1
                            print(f"fallback via at ({via_pos[0]:.2f}, {via_pos[1]:.2f})")
                    else:
                        vias_reused += 1
                        print(f"fallback via at ({via_pos[0]:.2f}, {via_pos[1]:.2f})")
                else:
                    print(f"\033[91mFAILED - no valid position\033[0m")
                    failed_pads += 1

    # Step 9: Generate zone polygon (only if we should create one)
    zone_sexpr = None
    if should_create_zone:
        zone_sexpr = generate_zone_sexpr(
            net_id=net_id,
            net_name=net_name,
            layer=plane_layer,
            polygon_points=zone_polygon,
            clearance=zone_clearance,
            min_thickness=min_thickness,
            direct_connect=True
        )

    # Step 10: Write output
    print(f"\nResults:")
    if should_create_zone:
        print(f"  Zone created on {plane_layer}")
    else:
        print(f"  Using existing zone on {plane_layer}")
    print(f"  New vias placed: {vias_placed}")
    print(f"  Existing vias reused: {vias_reused}")
    print(f"  Traces added: {traces_added}")
    if failed_pads > 0:
        print(f"  Failed pads: {failed_pads}")

    if ripped_net_ids:
        ripped_names = []
        for rid in ripped_net_ids:
            net = pcb_data.nets.get(rid)
            ripped_names.append(net.name if net else f"net_{rid}")
        print(f"  Nets excluded from output: {', '.join(ripped_names)}")

    if dry_run:
        print("\nDry run - no output file written")
    else:
        print(f"\nWriting output to {output_file}...")
        if write_plane_output(input_file, output_file, zone_sexpr, new_vias, new_segments,
                               exclude_net_ids=ripped_net_ids):
            print(f"Output written to {output_file}")
            print("Note: Open in KiCad and press 'B' to refill zones")
            if ripped_net_ids:
                print(f"WARNING: {len(ripped_net_ids)} net(s) were removed from output and need re-routing!")
        else:
            print("Error writing output file")

    return (vias_placed, traces_added, pads_need_via)


def main():
    parser = argparse.ArgumentParser(
        description="Create copper zone with via stitching to net pads",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python route_plane.py input.kicad_pcb output.kicad_pcb --net GND --layer B.Cu
    python route_plane.py input.kicad_pcb output.kicad_pcb --net VCC --layer In2.Cu --via-size 0.5
    python route_plane.py input.kicad_pcb output.kicad_pcb --net GND --layer B.Cu --dry-run
"""
    )
    parser.add_argument("input_file", help="Input KiCad PCB file")
    parser.add_argument("output_file", help="Output KiCad PCB file")

    # Required options
    parser.add_argument("--net", "-n", required=True, help="Net name for the plane (e.g., GND, VCC)")
    parser.add_argument("--layer", "-l", required=True, help="Copper layer for the zone (e.g., B.Cu, In1.Cu)")

    # Via and track geometry
    parser.add_argument("--via-size", type=float, default=0.3, help="Via outer diameter in mm (default: 0.3)")
    parser.add_argument("--via-drill", type=float, default=0.2, help="Via drill size in mm (default: 0.2)")
    parser.add_argument("--track-width", type=float, default=0.2, help="Track width for via-to-pad connections in mm (default: 0.2)")
    parser.add_argument("--clearance", type=float, default=0.1, help="Clearance in mm (default: 0.1)")

    # Zone options
    parser.add_argument("--zone-clearance", type=float, default=0.2, help="Zone clearance from other copper in mm (default: 0.2)")
    parser.add_argument("--min-thickness", type=float, default=0.1, help="Minimum zone copper thickness in mm (default: 0.1)")

    # Algorithm options
    parser.add_argument("--grid-step", type=float, default=0.1, help="Grid resolution in mm (default: 0.1)")
    parser.add_argument("--max-search-radius", type=float, default=10.0, help="Max radius to search for valid via position in mm (default: 10.0)")
    parser.add_argument("--max-via-reuse-radius", type=float, default=1.0, help="Max radius to reuse existing via instead of placing new one in mm (default: 1.0)")
    parser.add_argument("--hole-to-hole-clearance", type=float, default=0.2, help="Minimum clearance between drill holes in mm (default: 0.2)")
    parser.add_argument("--all-layers", nargs="+", default=['F.Cu', 'B.Cu'],
                        help="All copper layers for via span (default: F.Cu B.Cu)")

    # Blocker rip-up options
    parser.add_argument("--rip-blocker-nets", action="store_true",
                        help="Identify and rip up nets blocking via placement, then retry. Ripped nets are excluded from output.")
    parser.add_argument("--max-rip-nets", type=int, default=3,
                        help="Maximum number of blocker nets to rip up (default: 3)")

    # Debug options
    parser.add_argument("--dry-run", action="store_true", help="Analyze without writing output")
    parser.add_argument("--verbose", "-v", action="store_true", help="Print detailed DEBUG messages")

    args = parser.parse_args()

    create_plane(
        input_file=args.input_file,
        output_file=args.output_file,
        net_name=args.net,
        plane_layer=args.layer,
        via_size=args.via_size,
        via_drill=args.via_drill,
        track_width=args.track_width,
        clearance=args.clearance,
        zone_clearance=args.zone_clearance,
        min_thickness=args.min_thickness,
        grid_step=args.grid_step,
        max_search_radius=args.max_search_radius,
        max_via_reuse_radius=args.max_via_reuse_radius,
        hole_to_hole_clearance=args.hole_to_hole_clearance,
        all_layers=args.all_layers,
        verbose=args.verbose,
        dry_run=args.dry_run,
        rip_blocker_nets=args.rip_blocker_nets,
        max_rip_nets=args.max_rip_nets
    )


if __name__ == "__main__":
    main()

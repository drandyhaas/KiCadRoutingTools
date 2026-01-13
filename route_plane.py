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

# scipy for Voronoi computation (multi-net layer support)
try:
    from scipy.spatial import Voronoi
    import numpy as np
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

# Run startup checks before other imports
from startup_checks import run_all_checks
run_all_checks()

from kicad_parser import parse_kicad_pcb, PCBData, Pad, Via, Segment
from kicad_writer import generate_via_sexpr, generate_segment_sexpr, generate_zone_sexpr
from routing_config import GridRouteConfig, GridCoord
from routing_utils import build_layer_map
from route_modification import remove_net_from_pcb_data
from route import batch_route
from obstacle_cache import (
    ViaPlacementObstacleData,
    precompute_via_placement_obstacles,
    remove_via_placement_obstacles
)
import numpy as np

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
    exclude_net_id: int,
    protected_net_ids: Optional[Set[int]] = None
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
        protected_net_ids: Set of net IDs that should never be identified as blockers

    Returns:
        Net ID of the closest non-protected blocker, or None if no blocker found
    """
    best_blocker = None
    best_dist_sq = float('inf')
    best_protected_blocker = None
    best_protected_dist_sq = float('inf')
    protected = protected_net_ids or set()

    # Check segments
    for seg in pcb_data.segments:
        if seg.net_id == exclude_net_id:
            continue
        dist_sq = _point_to_segment_dist_sq(via_x, via_y, seg.start_x, seg.start_y, seg.end_x, seg.end_y)
        clearance_needed = config.via_size / 2 + seg.width / 2 + config.clearance
        if dist_sq < clearance_needed ** 2:
            if seg.net_id in protected:
                if dist_sq < best_protected_dist_sq:
                    best_protected_dist_sq = dist_sq
                    best_protected_blocker = seg.net_id
            elif dist_sq < best_dist_sq:
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
        if dist_sq < clearance_needed ** 2:
            if via.net_id in protected:
                if dist_sq < best_protected_dist_sq:
                    best_protected_dist_sq = dist_sq
                    best_protected_blocker = via.net_id
            elif dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_blocker = via.net_id

    # Report protected blocker if it's closer than any non-protected blocker
    if best_protected_blocker is not None and best_protected_dist_sq < best_dist_sq:
        net = pcb_data.nets.get(best_protected_blocker)
        blocker_name = net.name if net else f"net_{best_protected_blocker}"
        print(f"blocked by {blocker_name} (protected, cannot rip)...", end=" ")

    return best_blocker


def find_route_blocker_from_frontier(
    blocked_cells: List[Tuple[int, int, int]],
    pcb_data: PCBData,
    config: GridRouteConfig,
    exclude_net_id: int,
    protected_net_ids: Optional[Set[int]] = None
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
        protected_net_ids: Set of net IDs that should never be identified as blockers

    Returns:
        Net ID of the top non-protected blocker, or None if no blocker found
    """
    if not blocked_cells:
        return None

    coord = GridCoord(config.grid_step)
    blocked_set = set(blocked_cells)
    protected = protected_net_ids or set()

    # Count how many blocked cells each net is responsible for (including protected)
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

    # Find top blocker overall (for diagnostics) and top non-protected blocker (for ripping)
    top_blocker = max(net_block_count.keys(), key=lambda k: net_block_count[k])

    # Check if top blocker is protected
    if top_blocker in protected:
        net = pcb_data.nets.get(top_blocker)
        blocker_name = net.name if net else f"net_{top_blocker}"
        print(f"blocked by {blocker_name} (protected, cannot rip)...", end=" ")

        # Find top non-protected blocker
        non_protected = {k: v for k, v in net_block_count.items() if k not in protected}
        if non_protected:
            return max(non_protected.keys(), key=lambda k: non_protected[k])
        return None

    return top_blocker


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
    via_obstacle_cache: Dict[int, ViaPlacementObstacleData],
    routing_obstacles_cache: Dict[str, GridObstacleMap],
    all_copper_layers: List[str],
    via_blocked: bool,  # True if via placement failed, False if routing failed
    blocked_cells: Optional[List[Tuple[int, int, int]]] = None,  # Frontier from failed route
    new_vias: List[Dict] = None,  # Previously placed vias to re-block after rebuild
    hole_to_hole_clearance: float = 0.2,
    via_drill: float = 0.4,
    protected_net_ids: Optional[Set[int]] = None,  # Nets that should never be ripped up
    verbose: bool = False
) -> ViaPlacementResult:
    """
    Try to place a via and route to pad, ripping up blockers as needed.

    Called AFTER the fast path already failed. On first iteration, just finds and
    rips up the blocker without re-searching (since we already know search failed).

    Uses incremental obstacle updates for performance - instead of rebuilding the
    entire obstacle map after ripping a net, we just remove that net's cached obstacles.
    """
    ripped_net_ids = []
    failed_route_positions: Set[Tuple[int, int]] = set()  # Track positions where routing failed

    for attempt in range(max_rip_nets):
        if attempt == 0:
            # First iteration: skip search, we already know it failed
            # Just find the blocker directly
            if via_blocked:
                blocker = find_via_position_blocker(
                    pad.global_x, pad.global_y, pcb_data, config, net_id, protected_net_ids
                )
            else:
                # Route was blocked - use frontier data
                blocker = find_route_blocker_from_frontier(
                    blocked_cells or [], pcb_data, config, net_id, protected_net_ids
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
                    route_result.blocked_cells, pcb_data, config, net_id, protected_net_ids
                )
            else:
                # Via placement blocked
                blocker = find_via_position_blocker(
                    pad.global_x, pad.global_y, pcb_data, config, net_id, protected_net_ids
                )

        if blocker is None:
            break

        # Compute cache BEFORE ripping (while segments/vias still exist in pcb_data)
        if blocker not in via_obstacle_cache:
            via_obstacle_cache[blocker] = precompute_via_placement_obstacles(
                pcb_data, blocker, config, all_copper_layers
            )

        # Rip up blocker
        blocker_net = pcb_data.nets.get(blocker)
        blocker_name = blocker_net.name if blocker_net else f"net_{blocker}"
        print(f"ripping {blocker_name}...", end=" ")
        removed_segs, removed_vias = remove_net_from_pcb_data(pcb_data, blocker)
        ripped_net_ids.append(blocker)

        # Incremental update: remove ripped net's obstacles from existing maps
        cache = via_obstacle_cache[blocker]
        # Remove from via obstacle map
        if len(cache.blocked_vias) > 0:
            obstacles.remove_blocked_vias_batch(cache.blocked_vias)
        # Remove from routing obstacle maps
        for layer, cells in cache.blocked_cells_by_layer.items():
            if layer in routing_obstacles_cache and len(cells) > 0:
                cells_3d = np.column_stack([cells, np.zeros(len(cells), dtype=np.int32)])
                routing_obstacles_cache[layer].remove_blocked_cells_batch(cells_3d)
        # Update routing_obstacles reference if it's for the current layer
        if pad_layer and pad_layer in routing_obstacles_cache:
            routing_obstacles = routing_obstacles_cache[pad_layer]

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


def compute_zone_boundaries(
    vias_by_net: Dict[int, List[Tuple[float, float]]],
    board_bounds: Tuple[float, float, float, float],
    return_raw_polygons: bool = False,
    board_edge_clearance: float = 0.0
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

    Raises:
        ValueError: If scipy is not available
    """
    if not SCIPY_AVAILABLE:
        raise ValueError("scipy is required for multi-net layer support. Install with: pip install scipy")

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
            return {net_id: [(clip_min_x, clip_min_y), (clip_max_x, clip_min_y),
                            (clip_max_x, clip_max_y), (clip_min_x, clip_max_y)]}
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
            result[net_id] = polygons[0]
        else:
            # Merge all polygons for this net
            merged = merge_polygons(polygons)
            if merged:
                result[net_id] = merged
            else:
                # Fallback: use convex hull of all polygon vertices
                all_vertices = []
                for poly in polygons:
                    all_vertices.extend(poly)
                result[net_id] = convex_hull(all_vertices)

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


def merge_polygons(polygons: List[List[Tuple[float, float]]]) -> Optional[List[Tuple[float, float]]]:
    """
    Merge a list of adjacent polygons into a single polygon.
    Returns None if polygons are not all adjacent (i.e., disconnected).

    Uses find_polygon_groups() to detect if polygons form multiple
    disconnected groups. Only merges if all polygons are in the same group.
    """
    if not polygons:
        return None
    if len(polygons) == 1:
        return polygons[0]

    # Check if all polygons are connected via shared edges
    groups = find_polygon_groups(polygons)
    if len(groups) > 1:
        # Polygons are disconnected - return None to signal caller
        return None

    # All polygons are adjacent - safe to use convex hull
    all_vertices = []
    for poly in polygons:
        all_vertices.extend(poly)

    return convex_hull(all_vertices)


def convex_hull(points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """
    Compute convex hull of points using Graham scan.
    """
    if len(points) < 3:
        return points

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    # Sort points by x, then by y
    points = sorted(set(points))

    if len(points) < 3:
        return points

    # Build lower hull
    lower = []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    # Build upper hull
    upper = []
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    # Remove last point of each half because it's repeated
    return lower[:-1] + upper[:-1]


def polygons_share_edge(
    poly1: List[Tuple[float, float]],
    poly2: List[Tuple[float, float]],
    tolerance: float = 0.001
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
    tolerance: float = 0.001
) -> List[List[int]]:
    """
    Group polygons by adjacency using union-find.

    Two polygons are adjacent if they share at least one edge.

    Args:
        polygons: List of polygons, each polygon is list of (x, y) vertices
        tolerance: Distance tolerance for vertex matching

    Returns:
        List of groups, each group is list of polygon indices
    """
    if not polygons:
        return []

    n = len(polygons)
    parent = list(range(n))

    def find(x):
        if parent[x] != x:
            parent[x] = find(parent[x])
        return parent[x]

    def union(x, y):
        px, py = find(x), find(y)
        if px != py:
            parent[px] = py

    # Check all pairs for shared edges
    for i in range(n):
        for j in range(i + 1, n):
            if polygons_share_edge(polygons[i], polygons[j], tolerance):
                union(i, j)

    # Group polygons by their root
    groups: Dict[int, List[int]] = {}
    for i in range(n):
        root = find(i)
        if root not in groups:
            groups[root] = []
        groups[root].append(i)

    return list(groups.values())


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
    verbose: bool = False
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
        verbose: Print debug info

    Returns:
        List of (x, y) points along the route, or None if routing fails
    """
    coord = GridCoord(config.grid_step)

    # Build single-layer obstacle map
    num_layers = 1
    layer_idx = 0
    obstacles = GridObstacleMap(num_layers)

    # Block other nets' vias as hard obstacles
    for other_net_id, via_positions in other_nets_vias.items():
        for vx, vy in via_positions:
            gx, gy = coord.to_grid(vx, vy)
            # Block a circular area around each via (via size + clearance)
            via_radius = coord.to_grid_dist(config.via_size / 2 + config.clearance)
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

    max_iterations = 50000  # Allow more iterations for longer routes
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
    board_edge_clearance: float = 0.5
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
        board_edge_clearance: Clearance from board edge for zone polygons (mm).

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

    # Set of all net IDs being processed - these should never be ripped up
    protected_net_ids = set(net_ids)

    # Step 2: Check for existing zones on each target layer
    existing_zones = extract_zones(input_file)
    should_create_zones = []  # Per-net flag for whether to create zone
    for i, (net_name, plane_layer, net_id) in enumerate(zip(net_names, plane_layers, net_ids)):
        should_create, should_continue = check_existing_zones(
            existing_zones, plane_layer, net_name, net_id, verbose
        )
        if not should_continue:
            print(f"Error: Zone conflict for net '{net_name}' on layer {plane_layer}")
            return (0, 0, 0)
        should_create_zones.append(should_create)

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
    total_vias_placed = 0
    total_vias_reused = 0
    total_traces_added = 0
    total_failed_pads = 0
    total_pads_needing_vias = 0
    all_ripped_net_ids: List[int] = []

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
                    # Debug: check for via at 144.50, 99.40
                    for v in pcb_data.vias:
                        if abs(v.x - 144.50) < 0.01 and abs(v.y - 99.40) < 0.01:
                            print(f"    DEBUG: Found via at (144.50, 99.40) net_id={v.net_id}, excluding={net_id}")
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

        for pad_info in target_pads:
            if not pad_info['needs_via']:
                continue

            pad = pad_info['pad']
            pad_layer = pad_info.get('pad_layer')

            print(f"  Pad {pad.component_ref}.{pad.pad_number}...", end=" ")

            # First check if there's an existing or already-placed via nearby (within reuse radius)
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
                    continue  # Move to next pad

            # Need to place a new via (either no existing via, or reuse failed)
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
                    via_obstacle_cache, routing_obstacles_cache, all_layers,
                    via_blocked=via_blocked,
                    blocked_cells=blocked_cells,
                    new_vias=new_vias,
                    hole_to_hole_clearance=hole_to_hole_clearance,
                    via_drill=via_drill,
                    protected_net_ids=protected_net_ids,
                    verbose=verbose
                )

                if result.success:
                    # Track ripped nets
                    for rid in result.ripped_net_ids:
                        if rid not in ripped_net_ids:
                            ripped_net_ids.append(rid)
                    # Note: obstacle maps were already updated incrementally in try_place_via_with_ripup

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
            else:
                print(f"\033[91mFAILED - no valid position\033[0m")
                failed_pads += 1

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

        # Print per-net results
        print(f"\nResults for '{net_name}':")
        if should_create_zone and is_multi_net_layer:
            print(f"  Zone on {plane_layer} deferred (multi-net layer)")
        elif should_create_zone:
            print(f"  Zone created on {plane_layer}")
        else:
            print(f"  Using existing zone on {plane_layer}")
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
                        all_zone_sexprs.append(zone_sexpr)
                    continue

                # Compute zone boundaries using Voronoi (with raw polygon info)
                try:
                    result = compute_zone_boundaries(vias_by_net, board_bounds, return_raw_polygons=True,
                                                      board_edge_clearance=board_edge_clearance)
                    zone_polygons, raw_polygons, via_to_polygon_idx = result
                except ValueError as e:
                    print(f"  Error computing zone boundaries: {e}")
                    print(f"  Falling back to full board rectangle for first net with vias")
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
                    all_zone_sexprs.append(zone_sexpr)
                    continue

                # Check for disconnected regions in each net and route to connect them
                augmented_vias_by_net = {net_id: list(vias) for net_id, vias in vias_by_net.items()}
                connection_routes = []  # Track routes for logging

                for net_id, polygons in raw_polygons.items():
                    if len(polygons) <= 1:
                        continue  # Single polygon, nothing to connect

                    # Find disconnected groups
                    groups = find_polygon_groups(polygons)
                    if len(groups) <= 1:
                        continue  # All polygons are connected

                    net = pcb_data.nets.get(net_id)
                    net_name = net.name if net else f"net_{net_id}"
                    print(f"  Net '{net_name}': Found {len(groups)} disconnected regions")

                    # Get vias for this net and map them to polygon groups
                    net_vias = vias_by_net.get(net_id, [])
                    if len(net_vias) < 2:
                        continue

                    # Build group -> vias mapping
                    group_vias: Dict[int, List[Tuple[float, float]]] = {i: [] for i in range(len(groups))}
                    for via_pos in net_vias:
                        poly_idx = via_to_polygon_idx.get(net_id, {}).get(via_pos)
                        if poly_idx is not None:
                            # Find which group this polygon belongs to
                            for group_idx, group in enumerate(groups):
                                if poly_idx in group:
                                    group_vias[group_idx].append(via_pos)
                                    break

                    # Build other_nets_vias for proximity cost (vias from other nets)
                    other_nets_vias: Dict[int, List[Tuple[float, float]]] = {}
                    for other_net_id, other_vias in vias_by_net.items():
                        if other_net_id != net_id:
                            other_nets_vias[other_net_id] = other_vias

                    # Route between groups using MST-style approach
                    # For simplicity, connect group 0 to all other groups
                    for group_idx in range(1, len(groups)):
                        if not group_vias[0] or not group_vias[group_idx]:
                            continue

                        # Pick representative vias: closest pair between groups
                        best_dist = float('inf')
                        best_via_a, best_via_b = None, None
                        for via_a in group_vias[0]:
                            for via_b in group_vias[group_idx]:
                                dist = math.sqrt((via_a[0] - via_b[0])**2 + (via_a[1] - via_b[1])**2)
                                if dist < best_dist:
                                    best_dist = dist
                                    best_via_a, best_via_b = via_a, via_b

                        if best_via_a is None or best_via_b is None:
                            continue

                        print(f"    Routing from region 0 to region {group_idx} (dist: {best_dist:.2f}mm)")

                        # Route connection
                        route_path = route_plane_connection(
                            via_a=best_via_a,
                            via_b=best_via_b,
                            plane_layer=layer,
                            net_id=net_id,
                            other_nets_vias=other_nets_vias,
                            config=config,
                            pcb_data=pcb_data,
                            proximity_radius=plane_proximity_radius,
                            proximity_cost=plane_proximity_cost,
                            verbose=verbose
                        )

                        if route_path:
                            print(f"      Route found with {len(route_path)} points")
                            connection_routes.append((net_id, layer, route_path))

                            # Sample route path for Voronoi seeding
                            samples = sample_route_for_voronoi(route_path, sample_interval=2.0)
                            if samples:
                                print(f"      Added {len(samples)} seed points along route")
                                augmented_vias_by_net[net_id].extend(samples)

                            # Add vias in group_idx to group 0 for future connections
                            group_vias[0].extend(group_vias[group_idx])
                        else:
                            print(f"      Failed to route - regions may remain disconnected")

                # Re-compute zone boundaries with augmented vias if we added any
                total_augmented = sum(len(augmented_vias_by_net[nid]) - len(vias_by_net.get(nid, []))
                                      for nid in augmented_vias_by_net)
                if total_augmented > 0:
                    print(f"  Re-computing zone boundaries with {total_augmented} additional seed points")
                    try:
                        zone_polygons = compute_zone_boundaries(augmented_vias_by_net, board_bounds,
                                                                 board_edge_clearance=board_edge_clearance)
                    except ValueError as e:
                        print(f"  Error re-computing zone boundaries: {e}")
                        # Fall back to original zone_polygons computed above

                # Generate zones for each net
                for net_id, polygon in zone_polygons.items():
                    net = pcb_data.nets.get(net_id)
                    net_name = net.name if net else f"net_{net_id}"
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
                    all_zone_sexprs.append(zone_sexpr)

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
        print(f"\nWriting output to {output_file}...")
        # Combine all zone sexprs
        combined_zone_sexpr = '\n'.join(all_zone_sexprs) if all_zone_sexprs else None
        if write_plane_output(input_file, output_file, combined_zone_sexpr, all_new_vias, all_new_segments,
                               exclude_net_ids=all_ripped_net_ids):
            print(f"Output written to {output_file}")
            print("Note: Open in KiCad and press 'B' to refill zones")
            if all_ripped_net_ids:
                ripped_net_names = []
                for rid in all_ripped_net_ids:
                    net = pcb_data.nets.get(rid)
                    if net:
                        ripped_net_names.append(net.name)
                if reroute_ripped_nets and ripped_net_names:
                    # Re-route the ripped nets using the batch router
                    print(f"\n{'='*60}")
                    print(f"Re-routing {len(ripped_net_names)} ripped net(s)...")
                    print(f"{'='*60}")
                    # Increase recursion limit for large PCBs with many zone segments
                    old_recursion_limit = sys.getrecursionlimit()
                    sys.setrecursionlimit(max(old_recursion_limit, 100000))
                    try:
                        # Determine routing layers: all copper layers except plane layers
                        routing_layers = [l for l in all_layers if l not in plane_layers]
                        if not routing_layers:
                            routing_layers = ['F.Cu', 'B.Cu']
                        # Use all copper layers for via obstacle checking
                        all_copper_layers = list(set(all_layers + plane_layers))
                        routed, failed, route_time = batch_route(
                            input_file=output_file,
                            output_file=output_file,
                            net_names=ripped_net_names,
                            layers=all_copper_layers,  # All layers for via clearance
                            track_width=track_width,
                            clearance=clearance,
                            via_size=via_size,
                            via_drill=via_drill,
                            grid_step=grid_step,
                            hole_to_hole_clearance=hole_to_hole_clearance,
                            verbose=verbose,
                            minimal_obstacle_cache=True  # Only build cache for nets being routed
                        )
                        print(f"\nRe-routing complete: {routed} routed, {failed} failed in {route_time:.2f}s")
                    finally:
                        sys.setrecursionlimit(old_recursion_limit)
                else:
                    print(f"WARNING: {len(all_ripped_net_ids)} net(s) were removed from output and need re-routing!")
                    if ripped_net_names:
                        print(f"  Ripped nets: {', '.join(ripped_net_names)}")
        else:
            print("Error writing output file")

    return (total_vias_placed, total_traces_added, total_pads_needing_vias)


def main():
    parser = argparse.ArgumentParser(
        description="Create copper zones with via stitching to net pads",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Single net:
    python route_plane.py input.kicad_pcb output.kicad_pcb --net GND --plane-layer B.Cu

    # Multiple nets (each net paired with corresponding plane layer):
    python route_plane.py input.kicad_pcb output.kicad_pcb --net GND +3.3V --plane-layer In1.Cu In2.Cu

    # With rip-up and automatic re-routing:
    python route_plane.py input.kicad_pcb output.kicad_pcb --net GND VCC --plane-layer In1.Cu In2.Cu --rip-blocker-nets --reroute-ripped-nets
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

    # Board edge clearance
    parser.add_argument("--board-edge-clearance", type=float, default=0.5,
                        help="Clearance from board edge for zones in mm (default: 0.5)")

    # Debug options
    parser.add_argument("--dry-run", action="store_true", help="Analyze without writing output")
    parser.add_argument("--verbose", "-v", action="store_true", help="Print detailed DEBUG messages")

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
        board_edge_clearance=args.board_edge_clearance
    )


if __name__ == "__main__":
    main()

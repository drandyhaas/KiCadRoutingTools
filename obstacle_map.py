"""
Obstacle map building functions for PCB routing.

Builds GridObstacleMap objects from PCB data, adding obstacles for segments,
vias, pads, BGA exclusion zones, and routed paths.
"""

from typing import List, Optional, Tuple, Dict, Set, Union
from dataclasses import dataclass, field
import numpy as np

from kicad_parser import PCBData, Segment, Via, Pad
from routing_config import GridRouteConfig, GridCoord
from routing_utils import build_layer_map, iter_pad_blocked_cells
from bresenham_utils import walk_line, is_diagonal_segment, get_diagonal_via_blocking_params
from net_queries import expand_pad_layers
from obstacle_costs import add_bga_proximity_costs

# Import Rust router
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))

try:
    from grid_router import GridObstacleMap
except ImportError:
    # Will fail at runtime if not available
    GridObstacleMap = None


def build_base_obstacle_map(pcb_data: PCBData, config: GridRouteConfig,
                            nets_to_route: List[int],
                            extra_clearance: float = 0.0) -> GridObstacleMap:
    """Build base obstacle map with static obstacles (BGA zones, pads, pre-existing tracks/vias).

    Excludes all nets that will be routed (nets_to_route) - their stubs will be added
    per-net in the routing loop (excluding the current net being routed).

    Args:
        extra_clearance: Additional clearance to add for routing (e.g., for diff pair centerline routing)
    """
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)
    layer_map = build_layer_map(config.layers)
    nets_to_route_set = set(nets_to_route)

    obstacles = GridObstacleMap(num_layers)

    # Set BGA exclusion zones - block vias AND tracks on ALL layers
    for zone in config.bga_exclusion_zones:
        min_x, min_y, max_x, max_y = zone[:4]
        gmin_x, gmin_y = coord.to_grid(min_x, min_y)
        gmax_x, gmax_y = coord.to_grid(max_x, max_y)
        obstacles.set_bga_zone(gmin_x, gmin_y, gmax_x, gmax_y)
        for layer_idx in range(num_layers):
            for gx in range(gmin_x, gmax_x + 1):
                for gy in range(gmin_y, gmax_y + 1):
                    obstacles.add_blocked_cell(gx, gy, layer_idx)

    # Add BGA proximity costs (penalize routing near BGA edges)
    add_bga_proximity_costs(obstacles, config)

    # Precompute per-layer grid expansions for impedance-controlled routing
    # Use to_grid_dist_safe for via-related clearances to avoid grid quantization DRC errors
    expansion_grid_by_layer = {}
    via_block_grid_by_layer = {}
    via_track_expansion_grid_list = []
    for layer_name in config.layers:
        layer_width = config.get_track_width(layer_name)
        expansion_mm = layer_width / 2 + config.clearance + extra_clearance
        expansion_grid_by_layer[layer_name] = max(1, coord.to_grid_dist(expansion_mm))
        via_block_mm = config.via_size / 2 + layer_width / 2 + config.clearance + extra_clearance
        via_block_grid_by_layer[layer_name] = max(1, coord.to_grid_dist_safe(via_block_mm))
        via_track_mm = config.via_size / 2 + layer_width / 2 + config.clearance + extra_clearance
        via_track_expansion_grid_list.append(max(1, coord.to_grid_dist_safe(via_track_mm)))
    via_via_expansion_grid = max(1, coord.to_grid_dist(config.via_size + config.clearance))

    # Add segments as obstacles (excluding nets we'll route - their stubs added per-net)
    for seg in pcb_data.segments:
        if seg.net_id in nets_to_route_set:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        # Use per-layer expansion values
        expansion_grid = expansion_grid_by_layer.get(seg.layer, 1)
        via_block_grid = via_block_grid_by_layer.get(seg.layer, 1)
        _add_segment_obstacle(obstacles, seg, coord, layer_idx, expansion_grid, via_block_grid)

    # Add vias as obstacles (excluding nets we'll route)
    for via in pcb_data.vias:
        if via.net_id in nets_to_route_set:
            continue
        _add_via_obstacle(obstacles, via, coord, num_layers, via_track_expansion_grid_list, via_via_expansion_grid)

    # Add pads as obstacles (excluding nets we'll route - their pads added per-net)
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id in nets_to_route_set:
            continue
        for pad in pads:
            _add_pad_obstacle(obstacles, pad, coord, layer_map, config, extra_clearance)

    # Add board edge clearance
    add_board_edge_obstacles(obstacles, pcb_data, config, extra_clearance)

    # Add hole-to-hole clearance blocking for existing drills
    add_drill_hole_obstacles(obstacles, pcb_data, config, nets_to_route_set)

    return obstacles


def add_board_edge_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                              config: GridRouteConfig, extra_clearance: float = 0.0):
    """Block tracks and vias near the board edge.

    Args:
        obstacles: The obstacle map to add to
        pcb_data: PCB data containing board bounds
        config: Routing configuration
        extra_clearance: Additional clearance to add
    """
    board_bounds = pcb_data.board_info.board_bounds
    if not board_bounds:
        return

    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)
    min_x, min_y, max_x, max_y = board_bounds

    # Use board_edge_clearance if set, otherwise use track clearance
    edge_clearance = config.board_edge_clearance if config.board_edge_clearance > 0 else config.clearance
    # Add track half-width to clearance (tracks need to stay away from edge)
    track_edge_clearance = edge_clearance + config.track_width / 2 + extra_clearance
    via_edge_clearance = edge_clearance + config.via_size / 2 + extra_clearance

    # Convert to grid coordinates
    track_expand = coord.to_grid_dist(track_edge_clearance)
    via_expand = coord.to_grid_dist(via_edge_clearance)

    # Get grid bounds
    gmin_x, gmin_y = coord.to_grid(min_x, min_y)
    gmax_x, gmax_y = coord.to_grid(max_x, max_y)

    # Block cells outside the board (with clearance margin)
    # We block a margin around the entire grid range
    grid_margin = max(track_expand, via_expand) + 5  # Extra margin for safety

    # Block left edge
    for gx in range(gmin_x - grid_margin, gmin_x + track_expand + 1):
        for gy in range(gmin_y - grid_margin, gmax_y + grid_margin + 1):
            for layer_idx in range(num_layers):
                obstacles.add_blocked_cell(gx, gy, layer_idx)
            if gx < gmin_x + via_expand:
                obstacles.add_blocked_via(gx, gy)

    # Block right edge
    for gx in range(gmax_x - track_expand, gmax_x + grid_margin + 1):
        for gy in range(gmin_y - grid_margin, gmax_y + grid_margin + 1):
            for layer_idx in range(num_layers):
                obstacles.add_blocked_cell(gx, gy, layer_idx)
            if gx > gmax_x - via_expand:
                obstacles.add_blocked_via(gx, gy)

    # Block top edge (excluding corners already done)
    for gy in range(gmin_y - grid_margin, gmin_y + track_expand + 1):
        for gx in range(gmin_x + track_expand + 1, gmax_x - track_expand):
            for layer_idx in range(num_layers):
                obstacles.add_blocked_cell(gx, gy, layer_idx)
            if gy < gmin_y + via_expand:
                obstacles.add_blocked_via(gx, gy)

    # Block bottom edge (excluding corners already done)
    for gy in range(gmax_y - track_expand, gmax_y + grid_margin + 1):
        for gx in range(gmin_x + track_expand + 1, gmax_x - track_expand):
            for layer_idx in range(num_layers):
                obstacles.add_blocked_cell(gx, gy, layer_idx)
            if gy > gmax_y - via_expand:
                obstacles.add_blocked_via(gx, gy)


def add_drill_hole_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                              config: GridRouteConfig, nets_to_route_set: set):
    """Block via placement near existing drill holes (hole-to-hole clearance).

    Args:
        obstacles: The obstacle map to add to
        pcb_data: PCB data containing vias and pads with drills
        config: Routing configuration
        nets_to_route_set: Set of net IDs being routed (excluded from blocking)
    """
    if config.hole_to_hole_clearance <= 0:
        return

    coord = GridCoord(config.grid_step)

    # Collect all existing drill holes: (x, y, drill_diameter)
    drill_holes = []

    # Add via drills (excluding nets being routed)
    for via in pcb_data.vias:
        if via.net_id not in nets_to_route_set:
            drill_holes.append((via.x, via.y, via.drill))

    # Add pad drills (through-hole pads have drills)
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id in nets_to_route_set:
            continue
        for pad in pads:
            if pad.drill > 0:
                drill_holes.append((pad.global_x, pad.global_y, pad.drill))

    # Block via placement near each drill hole
    # New via drill needs to be hole_to_hole_clearance away from existing drill edge
    for hx, hy, drill_dia in drill_holes:
        # Required center-to-center distance = (existing_drill/2) + (new_via_drill/2) + clearance
        required_dist = drill_dia / 2 + config.via_drill / 2 + config.hole_to_hole_clearance
        expand = coord.to_grid_dist(required_dist)
        gx, gy = coord.to_grid(hx, hy)

        for ex in range(-expand, expand + 1):
            for ey in range(-expand, expand + 1):
                if ex*ex + ey*ey <= expand*expand:
                    obstacles.add_blocked_via(gx + ex, gy + ey)


def add_net_stubs_as_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                                net_id: int, config: GridRouteConfig,
                                extra_clearance: float = 0.0):
    """Add a net's stub segments as obstacles to the map."""
    coord = GridCoord(config.grid_step)
    layer_map = build_layer_map(config.layers)

    # Precompute per-layer grid expansions for impedance-controlled routing
    # Use to_grid_dist_safe for via blocking to avoid grid quantization DRC errors
    expansion_grid_by_layer = {}
    via_block_grid_by_layer = {}
    for layer_name in config.layers:
        layer_width = config.get_track_width(layer_name)
        expansion_mm = layer_width / 2 + config.clearance + extra_clearance
        expansion_grid_by_layer[layer_name] = max(1, coord.to_grid_dist(expansion_mm))
        via_block_mm = config.via_size / 2 + layer_width / 2 + config.clearance + extra_clearance
        via_block_grid_by_layer[layer_name] = max(1, coord.to_grid_dist_safe(via_block_mm))

    for seg in pcb_data.segments:
        if seg.net_id != net_id:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        expansion_grid = expansion_grid_by_layer.get(seg.layer, 1)
        via_block_grid = via_block_grid_by_layer.get(seg.layer, 1)
        _add_segment_obstacle(obstacles, seg, coord, layer_idx, expansion_grid, via_block_grid)


def add_diff_pair_own_stubs_as_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                                          p_net_id: int, n_net_id: int,
                                          config: GridRouteConfig,
                                          exclude_endpoints: List[Tuple[float, float]] = None,
                                          extra_clearance: float = 0.0):
    """Add a diff pair's own stub segments as obstacles to prevent centerline from crossing them.

    This is different from add_net_stubs_as_obstacles which adds OTHER nets' stubs.
    Here we add the SAME pair's stubs so the centerline route avoids crossing them,
    but we exclude the stub endpoints where we need to connect.

    Args:
        obstacles: The obstacle map to modify
        pcb_data: PCB data containing segments
        p_net_id: Net ID of P net
        n_net_id: Net ID of N net
        config: Routing configuration
        exclude_endpoints: List of (x, y) positions to exclude from blocking (stub connection points)
        extra_clearance: Additional clearance to add
    """
    coord = GridCoord(config.grid_step)
    layer_map = build_layer_map(config.layers)

    # Precompute per-layer grid expansions for impedance-controlled routing
    # Use to_grid_dist_safe for via blocking to avoid grid quantization DRC errors
    expansion_grid_by_layer = {}
    via_block_grid_by_layer = {}
    for layer_name in config.layers:
        layer_width = config.get_track_width(layer_name)
        expansion_mm = layer_width / 2 + config.clearance + extra_clearance
        expansion_grid_by_layer[layer_name] = max(1, coord.to_grid_dist(expansion_mm))
        via_block_mm = config.via_size / 2 + layer_width / 2 + config.clearance + extra_clearance
        via_block_grid_by_layer[layer_name] = max(1, coord.to_grid_dist_safe(via_block_mm))

    # Convert exclude endpoints to grid coordinates with some radius
    # Use max track width for exclusion radius
    max_track_width = config.get_max_track_width()
    exclude_grid_cells = set()
    exclude_radius = max(2, coord.to_grid_dist(max_track_width * 2))  # 2x track width radius
    if exclude_endpoints:
        for ex, ey in exclude_endpoints:
            gex, gey = coord.to_grid(ex, ey)
            for dx in range(-exclude_radius, exclude_radius + 1):
                for dy in range(-exclude_radius, exclude_radius + 1):
                    if dx*dx + dy*dy <= exclude_radius * exclude_radius:
                        exclude_grid_cells.add((gex + dx, gey + dy))

    for seg in pcb_data.segments:
        if seg.net_id != p_net_id and seg.net_id != n_net_id:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue

        # Add segment as obstacle, but skip cells in excluded regions
        expansion_grid = expansion_grid_by_layer.get(seg.layer, 1)
        via_block_grid = via_block_grid_by_layer.get(seg.layer, 1)
        _add_segment_obstacle_with_exclusion(
            obstacles, seg, coord, layer_idx, expansion_grid, via_block_grid,
            exclude_grid_cells
        )


def _add_segment_obstacle_with_exclusion(obstacles: GridObstacleMap, seg, coord: GridCoord,
                                          layer_idx: int, expansion_grid: int, via_block_grid: int,
                                          exclude_cells: Set[Tuple[int, int]]):
    """Add a segment as obstacle, excluding certain grid cells."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

    # Bresenham line with expansion
    dx = abs(gx2 - gx1)
    dy = abs(gy2 - gy1)
    sx = 1 if gx1 < gx2 else -1
    sy = 1 if gy1 < gy2 else -1
    err = dx - dy

    # For diagonal segments, the actual line passes between grid points,
    # so we need a slightly larger blocking radius to ensure clearance is maintained.
    # Using +0.25 catches sqrt(10)~3.16 but not sqrt(11)~3.32.
    is_diagonal = dx > 0 and dy > 0
    effective_via_block_sq = (via_block_grid + 0.25) ** 2 if is_diagonal else via_block_grid * via_block_grid
    via_block_range = via_block_grid + 1 if is_diagonal else via_block_grid

    gx, gy = gx1, gy1
    while True:
        # Skip if this cell is in the exclusion zone
        if (gx, gy) not in exclude_cells:
            for ex in range(-expansion_grid, expansion_grid + 1):
                for ey in range(-expansion_grid, expansion_grid + 1):
                    if (gx + ex, gy + ey) not in exclude_cells:
                        obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
            for ex in range(-via_block_range, via_block_range + 1):
                for ey in range(-via_block_range, via_block_range + 1):
                    if ex*ex + ey*ey <= effective_via_block_sq:
                        if (gx + ex, gy + ey) not in exclude_cells:
                            obstacles.add_blocked_via(gx + ex, gy + ey)

        if gx == gx2 and gy == gy2:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            gx += sx
        if e2 < dx:
            err += dx
            gy += sy


def add_net_pads_as_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                               net_id: int, config: GridRouteConfig,
                               extra_clearance: float = 0.0):
    """Add a net's pads as obstacles to the map."""
    coord = GridCoord(config.grid_step)
    layer_map = build_layer_map(config.layers)

    pads = pcb_data.pads_by_net.get(net_id, [])
    for pad in pads:
        _add_pad_obstacle(obstacles, pad, coord, layer_map, config, extra_clearance)


def add_net_vias_as_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                               net_id: int, config: GridRouteConfig,
                               extra_clearance: float = 0.0,
                               diagonal_margin: float = 0.0):
    """Add a net's vias as obstacles to the map.

    Args:
        diagonal_margin: Extra margin (in grid units) for track blocking to catch diagonal
                        segments that pass between grid points. Use 0.25 for single-ended routing.
    """
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)

    # Compute per-layer expansion for impedance-controlled routing
    # Use to_grid_dist_safe for via-track clearance to avoid grid quantization DRC errors
    via_track_expansion_grid_list = []
    for layer_name in config.layers:
        layer_width = config.get_track_width(layer_name)
        via_track_mm = config.via_size / 2 + layer_width / 2 + config.clearance + extra_clearance
        via_track_expansion_grid_list.append(max(1, coord.to_grid_dist_safe(via_track_mm)))
    via_via_expansion_grid = max(1, coord.to_grid_dist(config.via_size + config.clearance))

    for via in pcb_data.vias:
        if via.net_id != net_id:
            continue
        _add_via_obstacle(obstacles, via, coord, num_layers, via_track_expansion_grid_list, via_via_expansion_grid, diagonal_margin)


def add_vias_list_as_obstacles(obstacles: GridObstacleMap, vias: list,
                                config: GridRouteConfig,
                                extra_clearance: float = 0.0,
                                diagonal_margin: float = 0.0):
    """Add a list of Via objects as obstacles to the map.

    This is useful for adding vias from a route result before it's committed to pcb_data.

    Args:
        obstacles: The obstacle map to add to
        vias: List of Via objects to add as obstacles
        config: Routing configuration
        extra_clearance: Additional clearance to add (for diff pairs)
        diagonal_margin: Extra margin (in grid units) for track blocking
    """
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)

    # Compute per-layer expansion for impedance-controlled routing
    # Use to_grid_dist_safe for via-track clearance to avoid grid quantization DRC errors
    via_track_expansion_grid_list = []
    for layer_name in config.layers:
        layer_width = config.get_track_width(layer_name)
        via_track_mm = config.via_size / 2 + layer_width / 2 + config.clearance + extra_clearance
        via_track_expansion_grid_list.append(max(1, coord.to_grid_dist_safe(via_track_mm)))
    via_via_expansion_grid = max(1, coord.to_grid_dist(config.via_size + config.clearance))

    for via in vias:
        _add_via_obstacle(obstacles, via, coord, num_layers, via_track_expansion_grid_list, via_via_expansion_grid, diagonal_margin)


def add_segments_list_as_obstacles(obstacles: GridObstacleMap, segments: list,
                                    config: GridRouteConfig,
                                    extra_clearance: float = 0.0):
    """Add a list of Segment objects as obstacles to the map.

    This is useful for adding segments from a route result before it's committed to pcb_data.

    Args:
        obstacles: The obstacle map to add to
        segments: List of Segment objects to add as obstacles
        config: Routing configuration
        extra_clearance: Additional clearance to add (for diff pairs)
    """
    coord = GridCoord(config.grid_step)
    layer_map = build_layer_map(config.layers)

    # Precompute per-layer grid expansions for impedance-controlled routing
    # Use to_grid_dist_safe for via blocking to avoid grid quantization DRC errors
    expansion_grid_by_layer = {}
    via_block_grid_by_layer = {}
    for layer_name in config.layers:
        layer_width = config.get_track_width(layer_name)
        expansion_mm = layer_width / 2 + config.clearance + extra_clearance
        expansion_grid_by_layer[layer_name] = max(1, coord.to_grid_dist(expansion_mm))
        via_block_mm = config.via_size / 2 + layer_width / 2 + config.clearance
        via_block_grid_by_layer[layer_name] = max(1, coord.to_grid_dist_safe(via_block_mm))

    for seg in segments:
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is not None:
            expansion_grid = expansion_grid_by_layer.get(seg.layer, 1)
            via_block_grid = via_block_grid_by_layer.get(seg.layer, 1)
            _add_segment_obstacle(obstacles, seg, coord, layer_idx, expansion_grid, via_block_grid)


def add_same_net_via_clearance(obstacles: GridObstacleMap, pcb_data: PCBData,
                                net_id: int, config: GridRouteConfig):
    """Add via-via clearance blocking for same-net vias.

    This blocks only via placement (not track routing) near existing vias on the same net,
    enforcing DRC via-via clearance even within a single net.
    """
    coord = GridCoord(config.grid_step)

    # Via-via clearance: center-to-center distance must be >= via_size + clearance
    # So we block via placement within this radius of existing vias
    via_via_expansion_grid = max(1, coord.to_grid_dist(config.via_size + config.clearance))

    for via in pcb_data.vias:
        if via.net_id != net_id:
            continue
        gx, gy = coord.to_grid(via.x, via.y)
        # Only block via placement, not track routing (tracks can pass through same-net vias)
        for ex in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
            for ey in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
                if ex*ex + ey*ey <= via_via_expansion_grid * via_via_expansion_grid:
                    obstacles.add_blocked_via(gx + ex, gy + ey)


def add_same_net_pad_drill_via_clearance(obstacles: GridObstacleMap, pcb_data: PCBData,
                                          net_id: int, config: GridRouteConfig):
    """Add via blocking near same-net pad drill holes (hole-to-hole clearance).

    This blocks via placement near through-hole pads on the same net,
    enforcing manufacturing hole-to-hole clearance even within a single net.
    New vias must maintain hole_to_hole_clearance from existing pad drill holes.

    IMPORTANT: The pad center itself is NOT blocked - the router can use existing
    through-hole pads for layer transitions without placing a new via. Only the
    area around the pad (within clearance distance) is blocked for new vias.
    """
    if config.hole_to_hole_clearance <= 0:
        return

    coord = GridCoord(config.grid_step)

    pads = pcb_data.pads_by_net.get(net_id, [])
    for pad in pads:
        if pad.drill <= 0:
            continue  # SMD pad, no drill hole

        # Required center-to-center distance = (pad_drill/2) + (new_via_drill/2) + clearance
        required_dist = pad.drill / 2 + config.via_drill / 2 + config.hole_to_hole_clearance
        expand = coord.to_grid_dist(required_dist)
        gx, gy = coord.to_grid(pad.global_x, pad.global_y)

        for ex in range(-expand, expand + 1):
            for ey in range(-expand, expand + 1):
                if ex*ex + ey*ey <= expand*expand:
                    # Skip the pad center - the router can use the existing
                    # through-hole for layer transitions without a new via
                    if ex == 0 and ey == 0:
                        continue
                    obstacles.add_blocked_via(gx + ex, gy + ey)


def get_same_net_through_hole_positions(pcb_data: PCBData, net_id: int,
                                        config: GridRouteConfig) -> Set[Tuple[int, int]]:
    """Get grid positions of through-hole pads on this net.

    These positions can be used for layer transitions without placing a new via,
    since the existing through-hole already connects all layers.

    Args:
        pcb_data: PCB data with pads_by_net
        net_id: Net ID to get through-hole positions for
        config: Grid routing config (for grid_step)

    Returns:
        Set of (gx, gy) grid coordinates where through-hole pads exist
    """
    coord = GridCoord(config.grid_step)
    positions = set()

    pads = pcb_data.pads_by_net.get(net_id, [])
    for pad in pads:
        if pad.drill and pad.drill > 0:
            gx, gy = coord.to_grid(pad.global_x, pad.global_y)
            positions.add((gx, gy))

    return positions


def _add_segment_obstacle(obstacles: GridObstacleMap, seg, coord: GridCoord,
                          layer_idx: int, expansion_grid: int, via_block_grid: int,
                          blocked_cells: List[Set[Tuple[int, int]]] = None,
                          blocked_vias: Set[Tuple[int, int]] = None):
    """Add a segment as obstacle to the map.

    Args:
        obstacles: The obstacle map to add to
        seg: Segment object with start_x, start_y, end_x, end_y
        coord: GridCoord for coordinate conversion
        layer_idx: Layer index for track blocking
        expansion_grid: Expansion radius for track blocking
        via_block_grid: Expansion radius for via blocking
        blocked_cells: Optional per-layer sets to collect blocked cells for visualization
        blocked_vias: Optional set to collect blocked via positions for visualization
    """
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

    is_diagonal = is_diagonal_segment(gx1, gy1, gx2, gy2)
    effective_via_block_sq, via_block_range = get_diagonal_via_blocking_params(via_block_grid, is_diagonal)

    for gx, gy in walk_line(gx1, gy1, gx2, gy2):
        for ex in range(-expansion_grid, expansion_grid + 1):
            for ey in range(-expansion_grid, expansion_grid + 1):
                obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
                if blocked_cells is not None:
                    blocked_cells[layer_idx].add((gx + ex, gy + ey))
        for ex in range(-via_block_range, via_block_range + 1):
            for ey in range(-via_block_range, via_block_range + 1):
                if ex*ex + ey*ey <= effective_via_block_sq:
                    obstacles.add_blocked_via(gx + ex, gy + ey)
                    if blocked_vias is not None:
                        blocked_vias.add((gx + ex, gy + ey))


def _add_via_obstacle(obstacles: GridObstacleMap, via, coord: GridCoord,
                      num_layers: int, via_track_expansion_grid, via_via_expansion_grid: int,
                      diagonal_margin: float = 0.0,
                      blocked_cells: List[Set[Tuple[int, int]]] = None,
                      blocked_vias: Set[Tuple[int, int]] = None):
    """Add a via as obstacle to the map.

    Args:
        via_track_expansion_grid: Either a single int (same for all layers) or a list of ints
                                 (per-layer expansion) for impedance-controlled routing.
        diagonal_margin: Extra margin (in grid units) for track blocking to catch diagonal
                        segments that pass between grid points. Use 0.25 for single-ended routing.
        blocked_cells: Optional per-layer sets to collect blocked cells for visualization
        blocked_vias: Optional set to collect blocked via positions for visualization
    """
    gx, gy = coord.to_grid(via.x, via.y)

    # Support per-layer expansion for impedance-controlled routing
    if isinstance(via_track_expansion_grid, list):
        # Per-layer blocking
        for layer_idx in range(num_layers):
            layer_expansion = via_track_expansion_grid[layer_idx]
            effective_track_block_sq = (layer_expansion + diagonal_margin) ** 2
            track_block_range = layer_expansion + 1
            for ex in range(-track_block_range, track_block_range + 1):
                for ey in range(-track_block_range, track_block_range + 1):
                    if ex*ex + ey*ey <= effective_track_block_sq:
                        obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
                        if blocked_cells is not None:
                            blocked_cells[layer_idx].add((gx + ex, gy + ey))
    else:
        # Single value for all layers (legacy behavior)
        effective_track_block_sq = (via_track_expansion_grid + diagonal_margin) ** 2
        track_block_range = via_track_expansion_grid + 1
        for ex in range(-track_block_range, track_block_range + 1):
            for ey in range(-track_block_range, track_block_range + 1):
                if ex*ex + ey*ey <= effective_track_block_sq:
                    for layer_idx in range(num_layers):
                        obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
                        if blocked_cells is not None:
                            blocked_cells[layer_idx].add((gx + ex, gy + ey))

    # Block cells for via placement
    for ex in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
        for ey in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
            if ex*ex + ey*ey <= via_via_expansion_grid * via_via_expansion_grid:
                obstacles.add_blocked_via(gx + ex, gy + ey)
                if blocked_vias is not None:
                    blocked_vias.add((gx + ex, gy + ey))


def _add_pad_obstacle(obstacles: GridObstacleMap, pad, coord: GridCoord,
                      layer_map: Dict[str, int], config: GridRouteConfig,
                      extra_clearance: float = 0.0,
                      blocked_cells: List[Set[Tuple[int, int]]] = None,
                      blocked_vias: Set[Tuple[int, int]] = None):
    """Add a pad as obstacle to the map.

    Uses rectangular-with-rounded-corners pattern matching other pad blocking functions.

    Args:
        obstacles: The obstacle map to add to
        pad: Pad object with global_x, global_y, size_x, size_y, layers
        coord: GridCoord for coordinate conversion
        layer_map: Mapping of layer names to layer indices
        config: Routing configuration
        extra_clearance: Additional clearance to add
        blocked_cells: Optional per-layer sets to collect blocked cells for visualization
        blocked_vias: Optional set to collect blocked via positions for visualization
    """
    gx, gy = coord.to_grid(pad.global_x, pad.global_y)
    half_width = pad.size_x / 2
    half_height = pad.size_y / 2
    margin = config.track_width / 2 + config.clearance + extra_clearance
    corner_radius = pad.roundrect_rratio * min(pad.size_x, pad.size_y) if pad.shape == 'roundrect' else 0

    # Expand wildcard layers like "*.Cu" to actual routing layers
    expanded_layers = expand_pad_layers(pad.layers, config.layers)

    # Use shared utility for consistent pad blocking
    for cell_gx, cell_gy in iter_pad_blocked_cells(gx, gy, half_width, half_height, margin, config.grid_step, corner_radius):
        for layer in expanded_layers:
            layer_idx = layer_map.get(layer)
            if layer_idx is not None:
                obstacles.add_blocked_cell(cell_gx, cell_gy, layer_idx)
                if blocked_cells is not None:
                    blocked_cells[layer_idx].add((cell_gx, cell_gy))

    # Via blocking near pads - block vias if pad is on any copper layer
    if any(layer.endswith('.Cu') for layer in expanded_layers):
        via_margin = config.via_size / 2 + config.clearance
        for cell_gx, cell_gy in iter_pad_blocked_cells(gx, gy, half_width, half_height, via_margin, config.grid_step, corner_radius):
            obstacles.add_blocked_via(cell_gx, cell_gy)
            if blocked_vias is not None:
                blocked_vias.add((cell_gx, cell_gy))


def add_routed_path_obstacles(obstacles: GridObstacleMap, path: List[Tuple[int, int, int]],
                               config: GridRouteConfig, diagonal_margin: float = 0.0):
    """Add a newly routed path as obstacles to the map.

    Args:
        diagonal_margin: Extra margin (in grid units) for track blocking around vias to catch
                        diagonal segments that pass between grid points. Use 0.25 for single-ended routing.
    """
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)

    # Precompute per-layer values for impedance-controlled routing
    # Each layer may have different track widths
    # Use to_grid_dist_safe for via-related clearances to avoid grid quantization DRC errors
    expansion_grid_by_layer = []
    via_block_grid_by_layer = []
    via_track_expansion_grid_by_layer = []
    for layer_name in config.layers:
        layer_width = config.get_track_width(layer_name)
        expansion_mm = layer_width / 2 + config.clearance
        expansion_grid_by_layer.append(max(1, coord.to_grid_dist(expansion_mm)))
        via_block_mm = config.via_size / 2 + layer_width / 2 + config.clearance
        via_block_grid_by_layer.append(max(1, coord.to_grid_dist_safe(via_block_mm)))
        via_track_mm = config.via_size / 2 + layer_width / 2 + config.clearance
        via_track_expansion_grid_by_layer.append(max(1, coord.to_grid_dist_safe(via_track_mm)))

    via_via_expansion_grid = max(1, coord.to_grid_dist(config.via_size + config.clearance))

    for i in range(len(path) - 1):
        gx1, gy1, layer1 = path[i]
        gx2, gy2, layer2 = path[i + 1]

        if layer1 != layer2:
            # Via - add via obstacle with per-layer track blocking
            for layer_idx in range(num_layers):
                via_track_expansion_grid = via_track_expansion_grid_by_layer[layer_idx]
                effective_track_block_sq = (via_track_expansion_grid + diagonal_margin) ** 2
                track_block_range = via_track_expansion_grid + 1
                for ex in range(-track_block_range, track_block_range + 1):
                    for ey in range(-track_block_range, track_block_range + 1):
                        if ex*ex + ey*ey <= effective_track_block_sq:
                            obstacles.add_blocked_cell(gx1 + ex, gy1 + ey, layer_idx)
            for ex in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
                for ey in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
                    if ex*ex + ey*ey <= via_via_expansion_grid * via_via_expansion_grid:
                        obstacles.add_blocked_via(gx1 + ex, gy1 + ey)
        else:
            # Segment on same layer - add track obstacle using Bresenham
            # Use this layer's track width for via blocking
            expansion_grid = expansion_grid_by_layer[layer1]
            via_block_grid = via_block_grid_by_layer[layer1]

            dx = abs(gx2 - gx1)
            dy = abs(gy2 - gy1)
            sx = 1 if gx1 < gx2 else -1
            sy = 1 if gy1 < gy2 else -1
            err = dx - dy

            # For diagonal segments, add +0.25 margin to via blocking
            is_diagonal = dx > 0 and dy > 0
            effective_via_block_sq = (via_block_grid + 0.25) ** 2 if is_diagonal else via_block_grid * via_block_grid
            via_block_range = via_block_grid + 1 if is_diagonal else via_block_grid

            gx, gy = gx1, gy1
            while True:
                for ex in range(-expansion_grid, expansion_grid + 1):
                    for ey in range(-expansion_grid, expansion_grid + 1):
                        obstacles.add_blocked_cell(gx + ex, gy + ey, layer1)
                for ex in range(-via_block_range, via_block_range + 1):
                    for ey in range(-via_block_range, via_block_range + 1):
                        if ex*ex + ey*ey <= effective_via_block_sq:
                            obstacles.add_blocked_via(gx + ex, gy + ey)

                if gx == gx2 and gy == gy2:
                    break
                e2 = 2 * err
                if e2 > -dy:
                    err -= dy
                    gx += sx
                if e2 < dx:
                    err += dx
                    gy += sy


def build_obstacle_map(pcb_data: PCBData, config: GridRouteConfig,
                       exclude_net_id: int, unrouted_stubs: Optional[List[Tuple[float, float]]] = None) -> GridObstacleMap:
    """Build Rust obstacle map from PCB data (legacy function for compatibility)."""
    # Build base map excluding just this net
    obstacles = build_base_obstacle_map(pcb_data, config, [exclude_net_id])

    # Add stub proximity costs
    if unrouted_stubs:
        add_stub_proximity_costs(obstacles, unrouted_stubs, config)

    # Add same-net via clearance blocking (for DRC - vias can't be too close even on same net)
    add_same_net_via_clearance(obstacles, pcb_data, exclude_net_id, config)

    # Add same-net pad drill hole-to-hole clearance blocking
    add_same_net_pad_drill_via_clearance(obstacles, pcb_data, exclude_net_id, config)

    return obstacles


# ============================================================================
# Visualization support - captures blocked cell data for rendering
# ============================================================================

@dataclass
class VisualizationData:
    """Data for visualization of obstacle map."""
    blocked_cells: List[Set[Tuple[int, int]]] = field(default_factory=list)  # Per-layer
    blocked_vias: Set[Tuple[int, int]] = field(default_factory=set)
    bga_zones_grid: List[Tuple[int, int, int, int]] = field(default_factory=list)
    bounds: Tuple[float, float, float, float] = (0, 0, 100, 100)  # min_x, min_y, max_x, max_y in mm


def build_base_obstacle_map_with_vis(pcb_data: PCBData, config: GridRouteConfig,
                                      nets_to_route: List[int],
                                      extra_clearance: float = 0.0) -> Tuple[GridObstacleMap, VisualizationData]:
    """Build base obstacle map and capture visualization data.

    Same as build_base_obstacle_map, but also returns VisualizationData
    for rendering blocked cells in the visualizer.
    """
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)
    layer_map = build_layer_map(config.layers)
    nets_to_route_set = set(nets_to_route)

    obstacles = GridObstacleMap(num_layers)

    # Visualization data
    blocked_cells: List[Set[Tuple[int, int]]] = [set() for _ in range(num_layers)]
    blocked_vias: Set[Tuple[int, int]] = set()
    bga_zones_grid: List[Tuple[int, int, int, int]] = []

    # Set BGA exclusion zones - block vias AND tracks on ALL layers
    # Set BGA proximity radius for vertical attraction exclusion
    bga_prox_radius_grid = coord.to_grid_dist(config.bga_proximity_radius)
    obstacles.set_bga_proximity_radius(bga_prox_radius_grid)

    for zone in config.bga_exclusion_zones:
        min_x, min_y, max_x, max_y = zone[:4]
        gmin_x, gmin_y = coord.to_grid(min_x, min_y)
        gmax_x, gmax_y = coord.to_grid(max_x, max_y)
        obstacles.set_bga_zone(gmin_x, gmin_y, gmax_x, gmax_y)
        bga_zones_grid.append((gmin_x, gmin_y, gmax_x, gmax_y))
        for layer_idx in range(num_layers):
            for gx in range(gmin_x, gmax_x + 1):
                for gy in range(gmin_y, gmax_y + 1):
                    obstacles.add_blocked_cell(gx, gy, layer_idx)
                    blocked_cells[layer_idx].add((gx, gy))

    # Add BGA proximity costs (penalize routing near BGA edges)
    add_bga_proximity_costs(obstacles, config)

    # Precompute per-layer grid expansions for impedance-controlled routing
    # Use to_grid_dist_safe for via-related clearances to avoid grid quantization DRC errors
    expansion_grid_by_layer = {}
    via_block_grid_by_layer = {}
    via_track_expansion_grid_list = []
    for layer_name in config.layers:
        layer_width = config.get_track_width(layer_name)
        expansion_mm = layer_width / 2 + config.clearance + extra_clearance
        expansion_grid_by_layer[layer_name] = max(1, coord.to_grid_dist(expansion_mm))
        via_block_mm = config.via_size / 2 + layer_width / 2 + config.clearance + extra_clearance
        via_block_grid_by_layer[layer_name] = max(1, coord.to_grid_dist_safe(via_block_mm))
        via_track_mm = config.via_size / 2 + layer_width / 2 + config.clearance + extra_clearance
        via_track_expansion_grid_list.append(max(1, coord.to_grid_dist_safe(via_track_mm)))
    via_via_expansion_grid = max(1, coord.to_grid_dist(config.via_size + config.clearance))

    # Add segments as obstacles (excluding nets we'll route)
    for seg in pcb_data.segments:
        if seg.net_id in nets_to_route_set:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        expansion_grid = expansion_grid_by_layer.get(seg.layer, 1)
        via_block_grid = via_block_grid_by_layer.get(seg.layer, 1)
        _add_segment_obstacle(obstacles, seg, coord, layer_idx, expansion_grid, via_block_grid,
                              blocked_cells, blocked_vias)

    # Add vias as obstacles (excluding nets we'll route)
    for via in pcb_data.vias:
        if via.net_id in nets_to_route_set:
            continue
        _add_via_obstacle(obstacles, via, coord, num_layers, via_track_expansion_grid_list, via_via_expansion_grid,
                          blocked_cells=blocked_cells, blocked_vias=blocked_vias)

    # Add pads as obstacles (excluding nets we'll route)
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id in nets_to_route_set:
            continue
        for pad in pads:
            _add_pad_obstacle(obstacles, pad, coord, layer_map, config, extra_clearance,
                              blocked_cells, blocked_vias)

    vis_data = VisualizationData(
        blocked_cells=blocked_cells,
        blocked_vias=blocked_vias,
        bga_zones_grid=bga_zones_grid
    )

    return obstacles, vis_data


def add_net_obstacles_with_vis(obstacles: GridObstacleMap, pcb_data: PCBData,
                                net_id: int, config: GridRouteConfig,
                                extra_clearance: float = 0.0,
                                blocked_cells: List[Set[Tuple[int, int]]] = None,
                                blocked_vias: Set[Tuple[int, int]] = None,
                                diagonal_margin: float = 0.0):
    """Add a net's segments, vias, and pads as obstacles, capturing vis data.

    This is a combined function for adding all of a net's obstacles at once,
    useful for incrementally building obstacles during batch routing.

    Args:
        diagonal_margin: Extra margin (in grid units) for track blocking to catch diagonal
                        segments that pass between grid points. Use 0.25 for single-ended routing.
    """
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)
    layer_map = build_layer_map(config.layers)

    # Precompute per-layer grid expansions for impedance-controlled routing
    # Use to_grid_dist_safe for via-related clearances to avoid grid quantization DRC errors
    expansion_grid_by_layer = {}
    via_block_grid_by_layer = {}
    via_track_expansion_grid_list = []
    for layer_name in config.layers:
        layer_width = config.get_track_width(layer_name)
        expansion_mm = layer_width / 2 + config.clearance + extra_clearance
        expansion_grid_by_layer[layer_name] = max(1, coord.to_grid_dist(expansion_mm))
        via_block_mm = config.via_size / 2 + layer_width / 2 + config.clearance + extra_clearance
        via_block_grid_by_layer[layer_name] = max(1, coord.to_grid_dist_safe(via_block_mm))
        via_track_mm = config.via_size / 2 + layer_width / 2 + config.clearance + extra_clearance
        via_track_expansion_grid_list.append(max(1, coord.to_grid_dist_safe(via_track_mm)))
    via_via_expansion_grid = max(1, coord.to_grid_dist(config.via_size + config.clearance))

    if blocked_cells is None:
        blocked_cells = [set() for _ in range(num_layers)]
    if blocked_vias is None:
        blocked_vias = set()

    # Add segments
    for seg in pcb_data.segments:
        if seg.net_id != net_id:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        expansion_grid = expansion_grid_by_layer.get(seg.layer, 1)
        via_block_grid = via_block_grid_by_layer.get(seg.layer, 1)
        _add_segment_obstacle(obstacles, seg, coord, layer_idx, expansion_grid, via_block_grid,
                              blocked_cells, blocked_vias)

    # Add vias
    for via in pcb_data.vias:
        if via.net_id != net_id:
            continue
        _add_via_obstacle(obstacles, via, coord, num_layers, via_track_expansion_grid_list, via_via_expansion_grid,
                          diagonal_margin, blocked_cells, blocked_vias)

    # Add pads
    pads = pcb_data.pads_by_net.get(net_id, [])
    for pad in pads:
        _add_pad_obstacle(obstacles, pad, coord, layer_map, config, extra_clearance,
                          blocked_cells, blocked_vias)


def check_line_clearance(obstacles: GridObstacleMap,
                         x1: float, y1: float,
                         x2: float, y2: float,
                         layer_idx: int,
                         config: GridRouteConfig) -> bool:
    """Check if a line segment from (x1,y1) to (x2,y2) is clear of track obstacles on the given layer.

    Uses fine sampling (half grid step) to ensure complete coverage.
    Returns True if the path is clear, False if any cell is blocked.
    """
    coord = GridCoord(config.grid_step)

    dx = x2 - x1
    dy = y2 - y1
    length = (dx * dx + dy * dy) ** 0.5

    if length < 0.001:
        # Point check only
        gx, gy = coord.to_grid(x1, y1)
        return not obstacles.is_blocked(gx, gy, layer_idx)

    # Normalize direction
    dir_x = dx / length
    dir_y = dy / length

    # Sample at half grid step for better coverage
    step = config.grid_step / 2
    checked = set()

    dist = 0.0
    while dist <= length:
        x = x1 + dir_x * dist
        y = y1 + dir_y * dist
        gx, gy = coord.to_grid(x, y)

        if (gx, gy) not in checked:
            checked.add((gx, gy))
            if obstacles.is_blocked(gx, gy, layer_idx):
                return False

        dist += step

    return True


def check_stub_layer_clearance(obstacles: GridObstacleMap,
                                stub_segments: List[Segment],
                                target_layer_idx: int,
                                config: GridRouteConfig) -> bool:
    """Check if all stub segments can be placed on target_layer without conflicts.

    Args:
        obstacles: The obstacle map to check against
        stub_segments: List of segments that form the stub
        target_layer_idx: Layer index to check clearance on
        config: Routing configuration

    Returns:
        True if all segments are clear on target_layer, False otherwise
    """
    for seg in stub_segments:
        if not check_line_clearance(obstacles, seg.start_x, seg.start_y,
                                     seg.end_x, seg.end_y, target_layer_idx, config):
            return False
    return True


def add_connector_region_via_blocking(obstacles: GridObstacleMap,
                                       center_x: float, center_y: float,
                                       dir_x: float, dir_y: float,
                                       setback_distance: float,
                                       spacing_mm: float,
                                       config: GridRouteConfig,
                                       debug: bool = False):
    """Block vias in the connector region between stub center and setback position.

    The connector region extends from the stub center in the stub direction
    to the setback distance plus margin. This region needs to remain clear
    for the angled connector segments that will be added after routing.
    Vias in this region cause DRC errors due to conflicts with the turn segments.

    Args:
        obstacles: The obstacle map to add via blocking to
        center_x, center_y: Center point between P and N stubs (in mm)
        dir_x, dir_y: Normalized direction from stubs toward route
        setback_distance: Distance from stub center to route start (in mm)
        spacing_mm: Half-spacing between P and N tracks (in mm)
        config: Routing configuration
        debug: If True, print debug info about blocked region
    """
    coord = GridCoord(config.grid_step)

    # Block from stub center to setback + margin for via geometry
    # The margin accounts for via size and clearance requirements
    margin = config.via_size + config.clearance
    total_distance = setback_distance + margin

    # Corridor width should accommodate both P and N tracks plus clearance
    # The turn segments extend perpendicular to the stub direction
    corridor_half_width = spacing_mm + config.track_width / 2 + config.via_size / 2 + config.clearance

    # Perpendicular direction for corridor width
    perp_x = -dir_y
    perp_y = dir_x

    if debug:
        print(f"    Blocking corridor: center=({center_x:.2f},{center_y:.2f}), "
              f"dir=({dir_x:.2f},{dir_y:.2f}), dist={total_distance:.2f}mm, "
              f"half_width={corridor_half_width:.3f}mm")

    # Sample points along the connector region and block vias
    # Use half grid step for better coverage of diagonal corridors
    # (diagonal directions can miss grid cells when using full grid step)
    step = config.grid_step / 2
    dist = 0.0
    blocked_set = set()  # Track unique grid positions to avoid duplicates
    while dist <= total_distance:
        # Center of corridor at this distance
        cx = center_x + dir_x * dist
        cy = center_y + dir_y * dist

        # Block vias across the corridor width (also using half step for width)
        width_step = config.grid_step / 2
        width_steps = int(corridor_half_width / width_step) + 1
        for w in range(-width_steps, width_steps + 1):
            px = cx + perp_x * w * width_step
            py = cy + perp_y * w * width_step
            gx, gy = coord.to_grid(px, py)
            if (gx, gy) not in blocked_set:
                blocked_set.add((gx, gy))
                obstacles.add_blocked_via(gx, gy)

        dist += step

    if debug:
        print(f"    Blocked {len(blocked_set)} via positions")


def get_net_bounds(pcb_data: PCBData, net_ids: List[int], padding: float = 5.0) -> Tuple[float, float, float, float]:
    """Get bounding box around all the nets' components.

    Returns (min_x, min_y, max_x, max_y) in mm.
    """
    xs = []
    ys = []

    for net_id in net_ids:
        pads = pcb_data.pads_by_net.get(net_id, [])
        segments = [s for s in pcb_data.segments if s.net_id == net_id]

        for pad in pads:
            xs.append(pad.global_x)
            ys.append(pad.global_y)

        for seg in segments:
            xs.extend([seg.start_x, seg.end_x])
            ys.extend([seg.start_y, seg.end_y])

    if not xs or not ys:
        return (0, 0, 100, 100)

    return (min(xs) - padding, min(ys) - padding, max(xs) + padding, max(ys) + padding)


def draw_exclusion_zones_debug(config: GridRouteConfig,
                                unrouted_stubs: List[Tuple[float, float]] = None) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Get exclusion zone outline lines for User.5 layer debugging.

    Returns line segments for:
    - Circles around stub proximity zones
    - Rectangles around BGA exclusion zones (inner and outer with proximity radius)

    Args:
        config: Routing configuration with exclusion zone settings
        unrouted_stubs: List of (x, y) or (x, y, layer) tuples for stub positions

    Returns:
        List of ((x1, y1), (x2, y2)) line segment tuples
    """
    import math

    lines = []

    # Draw BGA exclusion zone rectangles and proximity rectangles
    prox_radius = config.bga_proximity_radius
    for zone in config.bga_exclusion_zones:
        min_x, min_y, max_x, max_y = zone[:4]
        # Draw inner rectangle (BGA zone itself)
        corners = [
            (min_x, min_y), (max_x, min_y),
            (max_x, max_y), (min_x, max_y)
        ]
        for i in range(4):
            x1, y1 = corners[i]
            x2, y2 = corners[(i + 1) % 4]
            lines.append(((x1, y1), (x2, y2)))

        # Draw outer rectangle (BGA zone expanded by proximity radius)
        if prox_radius > 0:
            outer_corners = [
                (min_x - prox_radius, min_y - prox_radius),
                (max_x + prox_radius, min_y - prox_radius),
                (max_x + prox_radius, max_y + prox_radius),
                (min_x - prox_radius, max_y + prox_radius)
            ]
            for i in range(4):
                x1, y1 = outer_corners[i]
                x2, y2 = outer_corners[(i + 1) % 4]
                lines.append(((x1, y1), (x2, y2)))

    # Draw stub proximity circles
    if unrouted_stubs and config.stub_proximity_radius > 0:
        radius = config.stub_proximity_radius
        num_segments = 16  # Circle approximation segments

        for stub in unrouted_stubs:
            cx, cy = stub[0], stub[1]

            # Draw circle as connected line segments
            for i in range(num_segments):
                angle1 = 2 * math.pi * i / num_segments
                angle2 = 2 * math.pi * (i + 1) / num_segments
                x1 = cx + radius * math.cos(angle1)
                y1 = cy + radius * math.sin(angle1)
                x2 = cx + radius * math.cos(angle2)
                y2 = cy + radius * math.sin(angle2)
                lines.append(((x1, y1), (x2, y2)))

    return lines

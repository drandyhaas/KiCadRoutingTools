"""
Obstacle map building functions for PCB routing.

Builds GridObstacleMap objects from PCB data, adding obstacles for segments,
vias, pads, BGA exclusion zones, and routed paths.
"""

from typing import List, Optional, Tuple, Dict, Set
from dataclasses import dataclass, field

from kicad_parser import PCBData, Segment, Via, Pad
from routing_config import GridRouteConfig, GridCoord

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
    layer_map = {name: idx for idx, name in enumerate(config.layers)}
    nets_to_route_set = set(nets_to_route)

    obstacles = GridObstacleMap(num_layers)

    # Set BGA exclusion zones - block vias AND tracks on ALL layers
    for zone in config.bga_exclusion_zones:
        min_x, min_y, max_x, max_y = zone
        gmin_x, gmin_y = coord.to_grid(min_x, min_y)
        gmax_x, gmax_y = coord.to_grid(max_x, max_y)
        obstacles.set_bga_zone(gmin_x, gmin_y, gmax_x, gmax_y)
        for layer_idx in range(num_layers):
            for gx in range(gmin_x, gmax_x + 1):
                for gy in range(gmin_y, gmax_y + 1):
                    obstacles.add_blocked_cell(gx, gy, layer_idx)

    # Precompute grid expansions (with extra clearance)
    expansion_mm = config.track_width / 2 + config.clearance + extra_clearance
    expansion_grid = max(1, coord.to_grid_dist(expansion_mm))
    via_block_mm = config.via_size / 2 + config.track_width / 2 + config.clearance + extra_clearance
    via_block_grid = max(1, coord.to_grid_dist(via_block_mm))
    via_track_expansion_grid = max(1, coord.to_grid_dist(config.via_size / 2 + config.track_width / 2 + config.clearance + extra_clearance))
    via_via_expansion_grid = max(1, coord.to_grid_dist(config.via_size + config.clearance + extra_clearance))

    # Add segments as obstacles (excluding nets we'll route - their stubs added per-net)
    for seg in pcb_data.segments:
        if seg.net_id in nets_to_route_set:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue

        _add_segment_obstacle(obstacles, seg, coord, layer_idx, expansion_grid, via_block_grid)

    # Add vias as obstacles (excluding nets we'll route)
    for via in pcb_data.vias:
        if via.net_id in nets_to_route_set:
            continue
        _add_via_obstacle(obstacles, via, coord, num_layers, via_track_expansion_grid, via_via_expansion_grid)

    # Add pads as obstacles (excluding nets we'll route - their pads added per-net)
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id in nets_to_route_set:
            continue
        for pad in pads:
            _add_pad_obstacle(obstacles, pad, coord, layer_map, config, extra_clearance)

    return obstacles


def add_net_stubs_as_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                                net_id: int, config: GridRouteConfig,
                                extra_clearance: float = 0.0):
    """Add a net's stub segments as obstacles to the map."""
    coord = GridCoord(config.grid_step)
    layer_map = {name: idx for idx, name in enumerate(config.layers)}

    expansion_mm = config.track_width / 2 + config.clearance + extra_clearance
    expansion_grid = max(1, coord.to_grid_dist(expansion_mm))
    via_block_mm = config.via_size / 2 + config.track_width / 2 + config.clearance + extra_clearance
    via_block_grid = max(1, coord.to_grid_dist(via_block_mm))

    for seg in pcb_data.segments:
        if seg.net_id != net_id:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        _add_segment_obstacle(obstacles, seg, coord, layer_idx, expansion_grid, via_block_grid)


def add_net_pads_as_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                               net_id: int, config: GridRouteConfig,
                               extra_clearance: float = 0.0):
    """Add a net's pads as obstacles to the map."""
    coord = GridCoord(config.grid_step)
    layer_map = {name: idx for idx, name in enumerate(config.layers)}

    pads = pcb_data.pads_by_net.get(net_id, [])
    for pad in pads:
        _add_pad_obstacle(obstacles, pad, coord, layer_map, config, extra_clearance)


def add_net_vias_as_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                               net_id: int, config: GridRouteConfig,
                               extra_clearance: float = 0.0):
    """Add a net's vias as obstacles to the map."""
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)

    via_track_expansion_grid = max(1, coord.to_grid_dist(config.via_size / 2 + config.track_width / 2 + config.clearance + extra_clearance))
    via_via_expansion_grid = max(1, coord.to_grid_dist(config.via_size + config.clearance + extra_clearance))

    for via in pcb_data.vias:
        if via.net_id != net_id:
            continue
        _add_via_obstacle(obstacles, via, coord, num_layers, via_track_expansion_grid, via_via_expansion_grid)


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


def _add_segment_obstacle(obstacles: GridObstacleMap, seg, coord: GridCoord,
                          layer_idx: int, expansion_grid: int, via_block_grid: int):
    """Add a segment as obstacle to the map."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

    # Bresenham line with expansion
    dx = abs(gx2 - gx1)
    dy = abs(gy2 - gy1)
    sx = 1 if gx1 < gx2 else -1
    sy = 1 if gy1 < gy2 else -1
    err = dx - dy

    gx, gy = gx1, gy1
    while True:
        for ex in range(-expansion_grid, expansion_grid + 1):
            for ey in range(-expansion_grid, expansion_grid + 1):
                obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
        for ex in range(-via_block_grid, via_block_grid + 1):
            for ey in range(-via_block_grid, via_block_grid + 1):
                if ex*ex + ey*ey <= via_block_grid * via_block_grid:
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


def _add_via_obstacle(obstacles: GridObstacleMap, via, coord: GridCoord,
                      num_layers: int, via_track_expansion_grid: int, via_via_expansion_grid: int):
    """Add a via as obstacle to the map."""
    gx, gy = coord.to_grid(via.x, via.y)
    # Block cells for track routing
    for ex in range(-via_track_expansion_grid, via_track_expansion_grid + 1):
        for ey in range(-via_track_expansion_grid, via_track_expansion_grid + 1):
            if ex*ex + ey*ey <= via_track_expansion_grid * via_track_expansion_grid:
                for layer_idx in range(num_layers):
                    obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
    # Block cells for via placement
    for ex in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
        for ey in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
            if ex*ex + ey*ey <= via_via_expansion_grid * via_via_expansion_grid:
                obstacles.add_blocked_via(gx + ex, gy + ey)


def _add_pad_obstacle(obstacles: GridObstacleMap, pad, coord: GridCoord,
                      layer_map: Dict[str, int], config: GridRouteConfig,
                      extra_clearance: float = 0.0):
    """Add a pad as obstacle to the map."""
    gx, gy = coord.to_grid(pad.global_x, pad.global_y)

    # Rectangular expansion for track clearance
    half_x_mm = pad.size_x / 2 + config.clearance + extra_clearance
    half_y_mm = pad.size_y / 2 + config.clearance + extra_clearance
    expand_x = coord.to_grid_dist(half_x_mm)
    expand_y = coord.to_grid_dist(half_y_mm)

    for ex in range(-expand_x, expand_x + 1):
        for ey in range(-expand_y, expand_y + 1):
            for layer in pad.layers:
                layer_idx = layer_map.get(layer)
                if layer_idx is not None:
                    obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)

    # Via blocking near pads
    if 'F.Cu' in pad.layers or 'B.Cu' in pad.layers:
        via_clear_mm = config.via_size / 2 + config.clearance
        via_expand_x = int((pad.size_x / 2 + via_clear_mm) / config.grid_step)
        via_expand_y = int((pad.size_y / 2 + via_clear_mm) / config.grid_step)
        for ex in range(-via_expand_x, via_expand_x + 1):
            for ey in range(-via_expand_y, via_expand_y + 1):
                obstacles.add_blocked_via(gx + ex, gy + ey)


def add_routed_path_obstacles(obstacles: GridObstacleMap, path: List[Tuple[int, int, int]],
                               config: GridRouteConfig):
    """Add a newly routed path as obstacles to the map."""
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)

    expansion_mm = config.track_width / 2 + config.clearance
    expansion_grid = max(1, coord.to_grid_dist(expansion_mm))
    via_block_mm = config.via_size / 2 + config.track_width / 2 + config.clearance
    via_block_grid = max(1, coord.to_grid_dist(via_block_mm))
    via_track_expansion_grid = max(1, coord.to_grid_dist(config.via_size / 2 + config.track_width / 2 + config.clearance))
    via_via_expansion_grid = max(1, coord.to_grid_dist(config.via_size + config.clearance))

    for i in range(len(path) - 1):
        gx1, gy1, layer1 = path[i]
        gx2, gy2, layer2 = path[i + 1]

        if layer1 != layer2:
            # Via - add via obstacle
            for ex in range(-via_track_expansion_grid, via_track_expansion_grid + 1):
                for ey in range(-via_track_expansion_grid, via_track_expansion_grid + 1):
                    if ex*ex + ey*ey <= via_track_expansion_grid * via_track_expansion_grid:
                        for layer_idx in range(num_layers):
                            obstacles.add_blocked_cell(gx1 + ex, gy1 + ey, layer_idx)
            for ex in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
                for ey in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
                    if ex*ex + ey*ey <= via_via_expansion_grid * via_via_expansion_grid:
                        obstacles.add_blocked_via(gx1 + ex, gy1 + ey)
        else:
            # Segment on same layer - add track obstacle using Bresenham
            dx = abs(gx2 - gx1)
            dy = abs(gy2 - gy1)
            sx = 1 if gx1 < gx2 else -1
            sy = 1 if gy1 < gy2 else -1
            err = dx - dy

            gx, gy = gx1, gy1
            while True:
                for ex in range(-expansion_grid, expansion_grid + 1):
                    for ey in range(-expansion_grid, expansion_grid + 1):
                        obstacles.add_blocked_cell(gx + ex, gy + ey, layer1)
                for ex in range(-via_block_grid, via_block_grid + 1):
                    for ey in range(-via_block_grid, via_block_grid + 1):
                        if ex*ex + ey*ey <= via_block_grid * via_block_grid:
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


def add_stub_proximity_costs(obstacles: GridObstacleMap, unrouted_stubs: List[Tuple[float, float]],
                              config: GridRouteConfig):
    """Add stub proximity costs to the obstacle map."""
    coord = GridCoord(config.grid_step)
    stub_radius_grid = coord.to_grid_dist(config.stub_proximity_radius)
    stub_cost_grid = int(config.stub_proximity_cost * 1000 / config.grid_step)

    for stub_x, stub_y in unrouted_stubs:
        gcx, gcy = coord.to_grid(stub_x, stub_y)
        for dx in range(-stub_radius_grid, stub_radius_grid + 1):
            for dy in range(-stub_radius_grid, stub_radius_grid + 1):
                dist_sq = dx * dx + dy * dy
                if dist_sq <= stub_radius_grid * stub_radius_grid:
                    dist = (dist_sq ** 0.5)
                    proximity = 1.0 - (dist / stub_radius_grid) if stub_radius_grid > 0 else 1.0
                    cost = int(proximity * stub_cost_grid)
                    obstacles.set_stub_proximity(gcx + dx, gcy + dy, cost)


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
    layer_map = {name: idx for idx, name in enumerate(config.layers)}
    nets_to_route_set = set(nets_to_route)

    obstacles = GridObstacleMap(num_layers)

    # Visualization data
    blocked_cells: List[Set[Tuple[int, int]]] = [set() for _ in range(num_layers)]
    blocked_vias: Set[Tuple[int, int]] = set()
    bga_zones_grid: List[Tuple[int, int, int, int]] = []

    # Set BGA exclusion zones - block vias AND tracks on ALL layers
    for zone in config.bga_exclusion_zones:
        min_x, min_y, max_x, max_y = zone
        gmin_x, gmin_y = coord.to_grid(min_x, min_y)
        gmax_x, gmax_y = coord.to_grid(max_x, max_y)
        obstacles.set_bga_zone(gmin_x, gmin_y, gmax_x, gmax_y)
        bga_zones_grid.append((gmin_x, gmin_y, gmax_x, gmax_y))
        for layer_idx in range(num_layers):
            for gx in range(gmin_x, gmax_x + 1):
                for gy in range(gmin_y, gmax_y + 1):
                    obstacles.add_blocked_cell(gx, gy, layer_idx)
                    blocked_cells[layer_idx].add((gx, gy))

    # Precompute grid expansions (with extra clearance)
    expansion_mm = config.track_width / 2 + config.clearance + extra_clearance
    expansion_grid = max(1, coord.to_grid_dist(expansion_mm))
    via_block_mm = config.via_size / 2 + config.track_width / 2 + config.clearance + extra_clearance
    via_block_grid = max(1, coord.to_grid_dist(via_block_mm))
    via_track_expansion_grid = max(1, coord.to_grid_dist(config.via_size / 2 + config.track_width / 2 + config.clearance + extra_clearance))
    via_via_expansion_grid = max(1, coord.to_grid_dist(config.via_size + config.clearance + extra_clearance))

    # Add segments as obstacles (excluding nets we'll route)
    for seg in pcb_data.segments:
        if seg.net_id in nets_to_route_set:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        _add_segment_obstacle_vis(obstacles, seg, coord, layer_idx, expansion_grid, via_block_grid,
                                   blocked_cells, blocked_vias)

    # Add vias as obstacles (excluding nets we'll route)
    for via in pcb_data.vias:
        if via.net_id in nets_to_route_set:
            continue
        _add_via_obstacle_vis(obstacles, via, coord, num_layers, via_track_expansion_grid, via_via_expansion_grid,
                               blocked_cells, blocked_vias)

    # Add pads as obstacles (excluding nets we'll route)
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id in nets_to_route_set:
            continue
        for pad in pads:
            _add_pad_obstacle_vis(obstacles, pad, coord, layer_map, config, extra_clearance,
                                   blocked_cells, blocked_vias)

    vis_data = VisualizationData(
        blocked_cells=blocked_cells,
        blocked_vias=blocked_vias,
        bga_zones_grid=bga_zones_grid
    )

    return obstacles, vis_data


def _add_segment_obstacle_vis(obstacles: GridObstacleMap, seg, coord: GridCoord,
                               layer_idx: int, expansion_grid: int, via_block_grid: int,
                               blocked_cells: List[Set[Tuple[int, int]]],
                               blocked_vias: Set[Tuple[int, int]]):
    """Add a segment as obstacle and capture vis data."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

    # Bresenham line with expansion
    dx = abs(gx2 - gx1)
    dy = abs(gy2 - gy1)
    sx = 1 if gx1 < gx2 else -1
    sy = 1 if gy1 < gy2 else -1
    err = dx - dy

    gx, gy = gx1, gy1
    while True:
        for ex in range(-expansion_grid, expansion_grid + 1):
            for ey in range(-expansion_grid, expansion_grid + 1):
                obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
                blocked_cells[layer_idx].add((gx + ex, gy + ey))
        for ex in range(-via_block_grid, via_block_grid + 1):
            for ey in range(-via_block_grid, via_block_grid + 1):
                if ex*ex + ey*ey <= via_block_grid * via_block_grid:
                    obstacles.add_blocked_via(gx + ex, gy + ey)
                    blocked_vias.add((gx + ex, gy + ey))

        if gx == gx2 and gy == gy2:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            gx += sx
        if e2 < dx:
            err += dx
            gy += sy


def _add_via_obstacle_vis(obstacles: GridObstacleMap, via, coord: GridCoord,
                           num_layers: int, via_track_expansion_grid: int, via_via_expansion_grid: int,
                           blocked_cells: List[Set[Tuple[int, int]]],
                           blocked_vias: Set[Tuple[int, int]]):
    """Add a via as obstacle and capture vis data."""
    gx, gy = coord.to_grid(via.x, via.y)
    # Block cells for track routing
    for ex in range(-via_track_expansion_grid, via_track_expansion_grid + 1):
        for ey in range(-via_track_expansion_grid, via_track_expansion_grid + 1):
            if ex*ex + ey*ey <= via_track_expansion_grid * via_track_expansion_grid:
                for layer_idx in range(num_layers):
                    obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
                    blocked_cells[layer_idx].add((gx + ex, gy + ey))
    # Block cells for via placement
    for ex in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
        for ey in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
            if ex*ex + ey*ey <= via_via_expansion_grid * via_via_expansion_grid:
                obstacles.add_blocked_via(gx + ex, gy + ey)
                blocked_vias.add((gx + ex, gy + ey))


def _add_pad_obstacle_vis(obstacles: GridObstacleMap, pad, coord: GridCoord,
                           layer_map: Dict[str, int], config: GridRouteConfig,
                           extra_clearance: float,
                           blocked_cells: List[Set[Tuple[int, int]]],
                           blocked_vias: Set[Tuple[int, int]]):
    """Add a pad as obstacle and capture vis data."""
    gx, gy = coord.to_grid(pad.global_x, pad.global_y)

    # Rectangular expansion for track clearance
    half_x_mm = pad.size_x / 2 + config.clearance + extra_clearance
    half_y_mm = pad.size_y / 2 + config.clearance + extra_clearance
    expand_x = coord.to_grid_dist(half_x_mm)
    expand_y = coord.to_grid_dist(half_y_mm)

    for ex in range(-expand_x, expand_x + 1):
        for ey in range(-expand_y, expand_y + 1):
            for layer in pad.layers:
                layer_idx = layer_map.get(layer)
                if layer_idx is not None:
                    obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
                    blocked_cells[layer_idx].add((gx + ex, gy + ey))

    # Via blocking near pads
    if 'F.Cu' in pad.layers or 'B.Cu' in pad.layers:
        via_clear_mm = config.via_size / 2 + config.clearance
        via_expand_x = int((pad.size_x / 2 + via_clear_mm) / config.grid_step)
        via_expand_y = int((pad.size_y / 2 + via_clear_mm) / config.grid_step)
        for ex in range(-via_expand_x, via_expand_x + 1):
            for ey in range(-via_expand_y, via_expand_y + 1):
                obstacles.add_blocked_via(gx + ex, gy + ey)
                blocked_vias.add((gx + ex, gy + ey))


def add_net_obstacles_with_vis(obstacles: GridObstacleMap, pcb_data: PCBData,
                                net_id: int, config: GridRouteConfig,
                                extra_clearance: float = 0.0,
                                blocked_cells: List[Set[Tuple[int, int]]] = None,
                                blocked_vias: Set[Tuple[int, int]] = None):
    """Add a net's segments, vias, and pads as obstacles, capturing vis data.

    This is a combined function for adding all of a net's obstacles at once,
    useful for incrementally building obstacles during batch routing.
    """
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)
    layer_map = {name: idx for idx, name in enumerate(config.layers)}

    expansion_mm = config.track_width / 2 + config.clearance + extra_clearance
    expansion_grid = max(1, coord.to_grid_dist(expansion_mm))
    via_block_mm = config.via_size / 2 + config.track_width / 2 + config.clearance + extra_clearance
    via_block_grid = max(1, coord.to_grid_dist(via_block_mm))
    via_track_expansion_grid = max(1, coord.to_grid_dist(config.via_size / 2 + config.track_width / 2 + config.clearance + extra_clearance))
    via_via_expansion_grid = max(1, coord.to_grid_dist(config.via_size + config.clearance + extra_clearance))

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
        _add_segment_obstacle_vis(obstacles, seg, coord, layer_idx, expansion_grid, via_block_grid,
                                   blocked_cells, blocked_vias)

    # Add vias
    for via in pcb_data.vias:
        if via.net_id != net_id:
            continue
        _add_via_obstacle_vis(obstacles, via, coord, num_layers, via_track_expansion_grid, via_via_expansion_grid,
                               blocked_cells, blocked_vias)

    # Add pads
    pads = pcb_data.pads_by_net.get(net_id, [])
    for pad in pads:
        _add_pad_obstacle_vis(obstacles, pad, coord, layer_map, config, extra_clearance,
                               blocked_cells, blocked_vias)


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

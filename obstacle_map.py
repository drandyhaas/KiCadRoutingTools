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

    # Precompute grid expansions (with extra clearance)
    expansion_mm = config.track_width / 2 + config.clearance + extra_clearance
    expansion_grid = max(1, coord.to_grid_dist(expansion_mm))
    via_block_mm = config.via_size / 2 + config.track_width / 2 + config.clearance + extra_clearance
    via_block_grid = max(1, coord.to_grid_dist(via_block_mm))
    via_track_expansion_grid = max(1, coord.to_grid_dist(config.via_size / 2 + config.track_width / 2 + config.clearance + extra_clearance))
    via_via_expansion_grid = max(1, coord.to_grid_dist(config.via_size + config.clearance))

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
    layer_map = {name: idx for idx, name in enumerate(config.layers)}

    expansion_mm = config.track_width / 2 + config.clearance + extra_clearance
    expansion_grid = max(1, coord.to_grid_dist(expansion_mm))
    via_block_mm = config.via_size / 2 + config.track_width / 2 + config.clearance + extra_clearance
    via_block_grid = max(1, coord.to_grid_dist(via_block_mm))

    # Convert exclude endpoints to grid coordinates with some radius
    exclude_grid_cells = set()
    exclude_radius = max(2, coord.to_grid_dist(config.track_width * 2))  # 2x track width radius
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
    via_via_expansion_grid = max(1, coord.to_grid_dist(config.via_size + config.clearance))

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

    # For diagonal segments, the actual line passes between grid points,
    # so we need a slightly larger blocking radius to ensure clearance is maintained.
    # Using +0.25 catches sqrt(10)~3.16 but not sqrt(11)~3.32.
    is_diagonal = dx > 0 and dy > 0
    effective_via_block_sq = (via_block_grid + 0.25) ** 2 if is_diagonal else via_block_grid * via_block_grid
    # Need to iterate over slightly larger range for diagonal segments
    via_block_range = via_block_grid + 1 if is_diagonal else via_block_grid

    gx, gy = gx1, gy1
    while True:
        for ex in range(-expansion_grid, expansion_grid + 1):
            for ey in range(-expansion_grid, expansion_grid + 1):
                obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
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


def _add_via_obstacle(obstacles: GridObstacleMap, via, coord: GridCoord,
                      num_layers: int, via_track_expansion_grid: int, via_via_expansion_grid: int):
    """Add a via as obstacle to the map."""
    gx, gy = coord.to_grid(via.x, via.y)
    # Block cells for track routing.
    effective_track_block_sq = via_track_expansion_grid ** 2
    track_block_range = via_track_expansion_grid + 1
    for ex in range(-track_block_range, track_block_range + 1):
        for ey in range(-track_block_range, track_block_range + 1):
            if ex*ex + ey*ey <= effective_track_block_sq:
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
            effective_track_block_sq = via_track_expansion_grid ** 2
            track_block_range = via_track_expansion_grid + 1
            for ex in range(-track_block_range, track_block_range + 1):
                for ey in range(-track_block_range, track_block_range + 1):
                    if ex*ex + ey*ey <= effective_track_block_sq:
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


def add_stub_proximity_costs(obstacles: GridObstacleMap, unrouted_stubs: List[Tuple[float, float]],
                              config: GridRouteConfig):
    """Add stub proximity costs to the obstacle map."""
    coord = GridCoord(config.grid_step)
    stub_radius_grid = coord.to_grid_dist(config.stub_proximity_radius)
    stub_cost_grid = int(config.stub_proximity_cost * 1000 / config.grid_step)

    for stub in unrouted_stubs:
        # Handle both (x, y) and (x, y, layer) tuple formats
        stub_x, stub_y = stub[0], stub[1]
        gcx, gcy = coord.to_grid(stub_x, stub_y)
        for dx in range(-stub_radius_grid, stub_radius_grid + 1):
            for dy in range(-stub_radius_grid, stub_radius_grid + 1):
                dist_sq = dx * dx + dy * dy
                if dist_sq <= stub_radius_grid * stub_radius_grid:
                    dist = (dist_sq ** 0.5)
                    proximity = 1.0 - (dist / stub_radius_grid) if stub_radius_grid > 0 else 1.0
                    cost = int(proximity * stub_cost_grid)
                    obstacles.set_stub_proximity(gcx + dx, gcy + dy, cost)


def add_bga_proximity_costs(obstacles: GridObstacleMap, config: GridRouteConfig):
    """Add BGA proximity costs around zone edges (outside the zones).

    Penalizes routing near BGA edges with linear falloff from max cost at edge
    to zero at bga_proximity_radius distance.
    """
    if config.bga_proximity_radius <= 0:
        return  # Feature disabled

    coord = GridCoord(config.grid_step)
    radius_grid = coord.to_grid_dist(config.bga_proximity_radius)
    cost_grid = int(config.bga_proximity_cost * 1000 / config.grid_step)

    for zone in config.bga_exclusion_zones:
        min_x, min_y, max_x, max_y = zone[:4]
        gmin_x, gmin_y = coord.to_grid(min_x, min_y)
        gmax_x, gmax_y = coord.to_grid(max_x, max_y)

        # Iterate over cells within radius of BGA zone edges (outside the zone)
        for gx in range(gmin_x - radius_grid, gmax_x + radius_grid + 1):
            for gy in range(gmin_y - radius_grid, gmax_y + radius_grid + 1):
                # Skip if inside BGA zone (already blocked)
                if gmin_x <= gx <= gmax_x and gmin_y <= gy <= gmax_y:
                    continue

                # Calculate distance to nearest edge of rectangle
                # Clamp point to rect, then calculate distance from original to clamped
                cx = max(gmin_x, min(gx, gmax_x))
                cy = max(gmin_y, min(gy, gmax_y))
                dist = ((gx - cx) ** 2 + (gy - cy) ** 2) ** 0.5

                if dist <= radius_grid:
                    proximity = 1.0 - (dist / radius_grid)
                    cost = int(proximity * cost_grid)
                    obstacles.set_stub_proximity(gx, gy, cost)


def compute_track_proximity_for_net(pcb_data: PCBData, net_id: int, config: GridRouteConfig,
                                     layer_map: Dict[str, int]) -> Dict[int, Dict[Tuple[int, int], int]]:
    """Compute track proximity costs for a single net's segments.

    Returns a dict of layer_idx -> {(gx, gy) -> cost} that can be stored and merged later.
    This allows incremental updates: compute once when route succeeds, remove when ripped up.

    Args:
        pcb_data: PCB data containing routed segments
        net_id: Net ID to compute proximity for
        config: Routing configuration with track_proximity_distance and track_proximity_cost
        layer_map: Mapping of layer names to layer indices

    Returns:
        Dict mapping layer_idx -> {(gx, gy) -> cost}
    """
    result: Dict[int, Dict[Tuple[int, int], int]] = {}

    if config.track_proximity_distance <= 0 or config.track_proximity_cost <= 0:
        return result  # Feature disabled

    coord = GridCoord(config.grid_step)
    radius_grid = coord.to_grid_dist(config.track_proximity_distance)
    cost_grid = int(config.track_proximity_cost * 1000 / config.grid_step)

    # Sample every ~1mm along segments (not every grid step) for performance
    sample_interval = max(1, int(1.0 / config.grid_step))

    for seg in pcb_data.segments:
        if seg.net_id != net_id:
            continue

        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue

        if layer_idx not in result:
            result[layer_idx] = {}
        layer_costs = result[layer_idx]

        # Walk along segment using Bresenham, sampling every sample_interval points
        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

        dx = abs(gx2 - gx1)
        dy = abs(gy2 - gy1)
        sx = 1 if gx1 < gx2 else -1
        sy = 1 if gy1 < gy2 else -1
        err = dx - dy

        gx, gy = gx1, gy1
        step_count = 0

        while True:
            # Only process every sample_interval'th point
            if step_count % sample_interval == 0:
                # Add proximity costs around this track point
                for ex in range(-radius_grid, radius_grid + 1):
                    for ey in range(-radius_grid, radius_grid + 1):
                        dist_sq = ex * ex + ey * ey
                        if dist_sq <= radius_grid * radius_grid:
                            dist = dist_sq ** 0.5
                            proximity = 1.0 - (dist / radius_grid) if radius_grid > 0 else 1.0
                            cost = int(proximity * cost_grid)
                            cell = (gx + ex, gy + ey)
                            # Store max cost at each cell
                            if cell not in layer_costs or cost > layer_costs[cell]:
                                layer_costs[cell] = cost

            if gx == gx2 and gy == gy2:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                gx += sx
            if e2 < dx:
                err += dx
                gy += sy
            step_count += 1

    return result


def merge_track_proximity_costs(obstacles: GridObstacleMap,
                                 per_net_costs: Dict[int, Dict[int, Dict[Tuple[int, int], int]]]):
    """Merge pre-computed per-net track proximity costs into the obstacle map.

    Args:
        obstacles: The obstacle map to add costs to
        per_net_costs: Dict of net_id -> layer_idx -> {(gx, gy) -> cost}
    """
    for net_id, layer_costs in per_net_costs.items():
        for layer_idx, cells in layer_costs.items():
            for (gx, gy), cost in cells.items():
                obstacles.set_layer_proximity(gx, gy, layer_idx, cost)


def add_cross_layer_tracks(obstacles: GridObstacleMap, pcb_data: PCBData,
                            config: GridRouteConfig, layer_map: Dict[str, int],
                            exclude_net_ids: Set[int] = None):
    """Populate cross-layer track positions for vertical alignment attraction.

    Adds positions of all routed tracks (excluding specified nets) to the
    obstacle map's cross-layer lookup structure. This enables the router
    to give a cost bonus for routing on top of existing tracks on other layers.

    Args:
        obstacles: The obstacle map to populate
        pcb_data: PCB data containing segments
        config: Routing configuration with vertical_attraction_radius
        layer_map: Mapping of layer names to layer indices
        exclude_net_ids: Set of net IDs to exclude (typically the net being routed)
    """
    if config.vertical_attraction_radius <= 0:
        return  # Feature disabled

    coord = GridCoord(config.grid_step)
    exclude_set = exclude_net_ids or set()

    # Sample every ~0.5mm along segments for reasonable density
    sample_interval = max(1, int(0.5 / config.grid_step))

    for seg in pcb_data.segments:
        if seg.net_id in exclude_set:
            continue

        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue

        _add_segment_cross_layer_track(obstacles, seg, coord, layer_idx, sample_interval)


def _add_segment_cross_layer_track(obstacles: GridObstacleMap, seg, coord: GridCoord,
                                    layer_idx: int, sample_interval: int):
    """Add a single segment's positions to the cross-layer track data."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

    dx = abs(gx2 - gx1)
    dy = abs(gy2 - gy1)
    sx = 1 if gx1 < gx2 else -1
    sy = 1 if gy1 < gy2 else -1
    err = dx - dy
    gx, gy = gx1, gy1
    step_count = 0

    while True:
        if step_count % sample_interval == 0:
            obstacles.add_cross_layer_track(gx, gy, layer_idx)

        if gx == gx2 and gy == gy2:
            break

        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            gx += sx
        if e2 < dx:
            err += dx
            gy += sy
        step_count += 1


def add_routed_path_cross_layer_tracks(obstacles: GridObstacleMap,
                                        new_segments: List,
                                        config: GridRouteConfig,
                                        layer_map: Dict[str, int]):
    """Add newly routed path segments to the cross-layer track data.

    Call this after successfully routing a path to make it an attractor
    for future paths on different layers.

    Args:
        obstacles: The obstacle map to update
        new_segments: List of Segment objects from the routed path
        config: Routing configuration
        layer_map: Mapping of layer names to layer indices
    """
    if config.vertical_attraction_radius <= 0:
        return  # Feature disabled

    coord = GridCoord(config.grid_step)
    sample_interval = max(1, int(0.5 / config.grid_step))

    for seg in new_segments:
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        _add_segment_cross_layer_track(obstacles, seg, coord, layer_idx, sample_interval)


def add_track_proximity_costs(obstacles: GridObstacleMap, pcb_data: PCBData,
                               routed_net_ids: List[int], config: GridRouteConfig,
                               layer_map: Dict[str, int]):
    """Add track proximity costs around previously routed tracks (same layer only).

    DEPRECATED: Use compute_track_proximity_for_net() + merge_track_proximity_costs()
    for better performance with incremental updates.

    Penalizes routing near existing tracks with linear falloff from max cost at track
    to zero at track_proximity_distance.
    """
    if config.track_proximity_distance <= 0 or config.track_proximity_cost <= 0:
        return  # Feature disabled

    # Compute and merge costs for all routed nets
    per_net_costs = {}
    for net_id in routed_net_ids:
        per_net_costs[net_id] = compute_track_proximity_for_net(pcb_data, net_id, config, layer_map)
    merge_track_proximity_costs(obstacles, per_net_costs)


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

    # Precompute grid expansions (with extra clearance)
    expansion_mm = config.track_width / 2 + config.clearance + extra_clearance
    expansion_grid = max(1, coord.to_grid_dist(expansion_mm))
    via_block_mm = config.via_size / 2 + config.track_width / 2 + config.clearance + extra_clearance
    via_block_grid = max(1, coord.to_grid_dist(via_block_mm))
    via_track_expansion_grid = max(1, coord.to_grid_dist(config.via_size / 2 + config.track_width / 2 + config.clearance + extra_clearance))
    via_via_expansion_grid = max(1, coord.to_grid_dist(config.via_size + config.clearance))

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

    # For diagonal segments, the actual line passes between grid points,
    # so we need a slightly larger blocking radius to ensure clearance is maintained.
    # Using +0.25 catches sqrt(10)~3.16 but not sqrt(11)~3.32.
    is_diagonal = dx > 0 and dy > 0
    effective_via_block_sq = (via_block_grid + 0.25) ** 2 if is_diagonal else via_block_grid * via_block_grid
    via_block_range = via_block_grid + 1 if is_diagonal else via_block_grid

    gx, gy = gx1, gy1
    while True:
        for ex in range(-expansion_grid, expansion_grid + 1):
            for ey in range(-expansion_grid, expansion_grid + 1):
                obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
                blocked_cells[layer_idx].add((gx + ex, gy + ey))
        for ex in range(-via_block_range, via_block_range + 1):
            for ey in range(-via_block_range, via_block_range + 1):
                if ex*ex + ey*ey <= effective_via_block_sq:
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
    # Block cells for track routing.
    effective_track_block_sq = via_track_expansion_grid ** 2
    track_block_range = via_track_expansion_grid + 1
    for ex in range(-track_block_range, track_block_range + 1):
        for ey in range(-track_block_range, track_block_range + 1):
            if ex*ex + ey*ey <= effective_track_block_sq:
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

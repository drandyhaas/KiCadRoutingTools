"""
Proximity cost functions for PCB routing obstacle maps.

Handles track proximity costs, stub proximity costs, BGA proximity costs,
and cross-layer track alignment tracking.
"""

from typing import List, Tuple, Dict, Set
import numpy as np

from kicad_parser import PCBData
from routing_config import GridRouteConfig, GridCoord
from bresenham_utils import walk_line

# Module-level cache for pre-computed proximity offset tables
# Key: (radius_grid, cost_grid) -> List of (ex, ey, cost) tuples
_proximity_offset_cache: Dict[Tuple[int, int], List[Tuple[int, int, int]]] = {}


def _get_proximity_offsets(radius_grid: int, cost_grid: int) -> List[Tuple[int, int, int]]:
    """Get pre-computed proximity offsets and costs for a given radius.

    Returns a list of (ex, ey, cost) tuples for all grid cells within the radius.
    Uses caching to avoid recomputing the same table multiple times.
    """
    cache_key = (radius_grid, cost_grid)
    if cache_key in _proximity_offset_cache:
        return _proximity_offset_cache[cache_key]

    offsets = []
    radius_sq = radius_grid * radius_grid

    for ex in range(-radius_grid, radius_grid + 1):
        for ey in range(-radius_grid, radius_grid + 1):
            dist_sq = ex * ex + ey * ey
            if dist_sq <= radius_sq:
                dist = dist_sq ** 0.5
                # Use exact same formula as original to avoid floating-point differences
                proximity = 1.0 - (dist / radius_grid) if radius_grid > 0 else 1.0
                cost = int(proximity * cost_grid)
                offsets.append((ex, ey, cost))

    _proximity_offset_cache[cache_key] = offsets
    return offsets


# Import Rust router
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))

try:
    from grid_router import GridObstacleMap
except ImportError:
    GridObstacleMap = None


def add_stub_proximity_costs(obstacles: GridObstacleMap, unrouted_stubs: List[Tuple[float, float]],
                              config: GridRouteConfig):
    """Add stub proximity costs to the obstacle map.

    When via_proximity_cost == 0, vias are blocked within stub proximity radius.
    Uses batch Rust method for performance.
    Also registers zone centers for direction-aware cost calculation.
    """
    if not unrouted_stubs:
        return

    coord = GridCoord(config.grid_step)
    stub_radius_grid = coord.to_grid_dist(config.stub_proximity_radius)
    stub_cost_grid = int(config.stub_proximity_cost * 1000 / config.grid_step)
    block_vias = (config.via_proximity_cost == 0)

    # Convert stub positions to grid coordinates
    stub_grid_positions = []
    zone_centers = []
    for stub in unrouted_stubs:
        stub_x, stub_y = stub[0], stub[1]
        gcx, gcy = coord.to_grid(stub_x, stub_y)
        stub_grid_positions.append((gcx, gcy))
        zone_centers.append((gcx, gcy, stub_radius_grid))

    # Use batch Rust method for performance
    obstacles.add_stub_proximity_costs_batch(
        stub_grid_positions, stub_radius_grid, stub_cost_grid, block_vias
    )

    # Register zone centers for direction-aware cost calculation
    obstacles.add_proximity_zone_centers_batch(zone_centers)


def add_bga_proximity_costs(obstacles: GridObstacleMap, config: GridRouteConfig):
    """Add BGA proximity costs around zone edges (outside the zones).

    Penalizes routing near BGA edges with linear falloff from max cost at edge
    to zero at bga_proximity_radius distance.
    Also registers zone centers for direction-aware cost calculation.
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

        # Register the BGA zone center for direction-aware cost calculation
        # Using zone center as the proximity center point
        center_gx = (gmin_x + gmax_x) // 2
        center_gy = (gmin_y + gmax_y) // 2
        # Use a radius that covers from zone center to proximity edge
        zone_half_diag = int(((gmax_x - gmin_x) ** 2 + (gmax_y - gmin_y) ** 2) ** 0.5 / 2)
        effective_radius = zone_half_diag + radius_grid
        obstacles.add_proximity_zone_center(center_gx, center_gy, effective_radius)

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
                                     layer_map: Dict[str, int]) -> np.ndarray:
    """Compute track proximity costs for a single net's segments.

    Returns a numpy array with columns [layer, gx, gy, cost] for efficient storage and batch merge.
    This allows incremental updates: compute once when route succeeds, remove when ripped up.

    Args:
        pcb_data: PCB data containing routed segments
        net_id: Net ID to compute proximity for
        config: Routing configuration with track_proximity_distance and track_proximity_cost
        layer_map: Mapping of layer names to layer indices

    Returns:
        numpy array of shape (N, 4) with columns [layer, gx, gy, cost], dtype int32
    """
    # Use dict internally for efficient max tracking, convert to numpy at end
    result: Dict[Tuple[int, int, int], int] = {}  # (layer, gx, gy) -> cost

    if config.track_proximity_distance <= 0 or config.track_proximity_cost <= 0:
        return np.empty((0, 4), dtype=np.int32)  # Feature disabled

    coord = GridCoord(config.grid_step)
    radius_grid = coord.to_grid_dist(config.track_proximity_distance)
    cost_grid = int(config.track_proximity_cost * 1000 / config.grid_step)

    # Get pre-computed offset table (cached)
    offsets = _get_proximity_offsets(radius_grid, cost_grid)

    # Sample every ~1mm along segments (not every grid step) for performance
    sample_interval = max(1, int(1.0 / config.grid_step))

    for seg in pcb_data.segments:
        if seg.net_id != net_id:
            continue

        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue

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
                # Add proximity costs using pre-computed offset table
                for ex, ey, cost in offsets:
                    key = (layer_idx, gx + ex, gy + ey)
                    # Store max cost at each cell
                    if key not in result or cost > result[key]:
                        result[key] = cost

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

    # Convert to numpy array
    if not result:
        return np.empty((0, 4), dtype=np.int32)

    arr = np.array([[layer, gx, gy, cost] for (layer, gx, gy), cost in result.items()], dtype=np.int32)
    return arr


def merge_track_proximity_costs(obstacles: GridObstacleMap,
                                 per_net_costs: Dict[int, np.ndarray]):
    """Merge pre-computed per-net track proximity costs into the obstacle map.

    Args:
        obstacles: The obstacle map to add costs to
        per_net_costs: Dict of net_id -> numpy array with columns [layer, gx, gy, cost]
    """
    # Concatenate all arrays for efficient batch processing
    arrays_to_merge = [arr for arr in per_net_costs.values() if len(arr) > 0]
    if not arrays_to_merge:
        return

    all_costs = np.vstack(arrays_to_merge)
    obstacles.set_layer_proximity_batch(all_costs)


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

    for step_count, (gx, gy) in enumerate(walk_line(gx1, gy1, gx2, gy2)):
        if step_count % sample_interval == 0:
            obstacles.add_cross_layer_track(gx, gy, layer_idx)


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

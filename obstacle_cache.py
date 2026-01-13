"""
Net obstacle caching for PCB routing.

Provides pre-computation and incremental updates for per-net obstacles,
dramatically speeding up routing by avoiding redundant obstacle calculations.
"""

from typing import List, Tuple, Dict, Set
from dataclasses import dataclass, field
import numpy as np

from kicad_parser import PCBData
from routing_config import GridRouteConfig, GridCoord
from routing_utils import build_layer_map
from net_queries import expand_pad_layers

# Import Rust router
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))

try:
    from grid_router import GridObstacleMap
except ImportError:
    GridObstacleMap = None


@dataclass
class NetObstacleData:
    """Cached obstacle data for a single net.

    Pre-computed blocked cells and vias for fast batch adding.
    Uses numpy arrays for memory efficiency (12 bytes per cell vs 72+ for tuples).
    """
    # Blocked cells as numpy array of shape (N, 3) with columns [gx, gy, layer_idx]
    # dtype=int32 for efficient storage
    blocked_cells: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=np.int32))
    # Blocked via positions as numpy array of shape (M, 2) with columns [gx, gy]
    blocked_vias: np.ndarray = field(default_factory=lambda: np.empty((0, 2), dtype=np.int32))


def precompute_net_obstacles(pcb_data: PCBData, net_id: int, config: GridRouteConfig,
                              extra_clearance: float = 0.0,
                              diagonal_margin: float = 0.25) -> NetObstacleData:
    """Pre-compute obstacle cells for a single net.

    Returns cached data that can be quickly added to obstacle maps via batch operations.

    Args:
        pcb_data: PCB data structure
        net_id: Net ID to compute obstacles for
        config: Routing configuration
        extra_clearance: Additional clearance (for diff pair routing)
        diagonal_margin: Extra margin for via blocking near diagonal segments

    Returns:
        NetObstacleData with blocked_cells and blocked_vias lists
    """
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)
    layer_map = build_layer_map(config.layers)

    # Collect cells/vias into sets to deduplicate
    blocked_cells_set: Set[Tuple[int, int, int]] = set()
    blocked_vias_set: Set[Tuple[int, int]] = set()

    # Precompute expansion values
    expansion_mm = config.track_width / 2 + config.clearance + extra_clearance
    expansion_grid = max(1, coord.to_grid_dist(expansion_mm))
    via_block_mm = config.via_size / 2 + config.track_width / 2 + config.clearance + extra_clearance
    via_block_grid = max(1, coord.to_grid_dist(via_block_mm))
    via_track_expansion_mm = config.via_size / 2 + config.track_width / 2 + config.clearance + extra_clearance
    via_track_expansion_grid = max(1, coord.to_grid_dist(via_track_expansion_mm))
    via_via_expansion_grid = max(1, coord.to_grid_dist(config.via_size + config.clearance))

    # Process segments
    for seg in pcb_data.segments:
        if seg.net_id != net_id:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        _collect_segment_obstacles(seg, coord, layer_idx, expansion_grid, via_block_grid,
                                    blocked_cells_set, blocked_vias_set)

    # Process vias
    for via in pcb_data.vias:
        if via.net_id != net_id:
            continue
        _collect_via_obstacles(via, coord, num_layers, via_track_expansion_grid,
                                via_via_expansion_grid, diagonal_margin,
                                blocked_cells_set, blocked_vias_set)

    # Process pads
    pads = pcb_data.pads_by_net.get(net_id, [])
    for pad in pads:
        _collect_pad_obstacles(pad, coord, layer_map, config, extra_clearance,
                                blocked_cells_set, blocked_vias_set)

    # Convert sets to numpy arrays for memory efficiency
    if blocked_cells_set:
        blocked_cells_arr = np.array(list(blocked_cells_set), dtype=np.int32)
    else:
        blocked_cells_arr = np.empty((0, 3), dtype=np.int32)

    if blocked_vias_set:
        blocked_vias_arr = np.array(list(blocked_vias_set), dtype=np.int32)
    else:
        blocked_vias_arr = np.empty((0, 2), dtype=np.int32)

    return NetObstacleData(
        blocked_cells=blocked_cells_arr,
        blocked_vias=blocked_vias_arr
    )


def _collect_segment_obstacles(seg, coord: GridCoord, layer_idx: int,
                                expansion_grid: int, via_block_grid: int,
                                blocked_cells: Set[Tuple[int, int, int]],
                                blocked_vias: Set[Tuple[int, int]]):
    """Collect segment obstacle cells into sets (no obstacle map modification)."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

    dx = abs(gx2 - gx1)
    dy = abs(gy2 - gy1)
    sx = 1 if gx1 < gx2 else -1
    sy = 1 if gy1 < gy2 else -1
    err = dx - dy

    is_diagonal = dx > 0 and dy > 0
    effective_via_block_sq = (via_block_grid + 0.25) ** 2 if is_diagonal else via_block_grid * via_block_grid
    via_block_range = via_block_grid + 1 if is_diagonal else via_block_grid

    gx, gy = gx1, gy1
    while True:
        for ex in range(-expansion_grid, expansion_grid + 1):
            for ey in range(-expansion_grid, expansion_grid + 1):
                blocked_cells.add((gx + ex, gy + ey, layer_idx))
        for ex in range(-via_block_range, via_block_range + 1):
            for ey in range(-via_block_range, via_block_range + 1):
                if ex*ex + ey*ey <= effective_via_block_sq:
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


def _collect_via_obstacles(via, coord: GridCoord, num_layers: int,
                            via_track_expansion_grid: int, via_via_expansion_grid: int,
                            diagonal_margin: float,
                            blocked_cells: Set[Tuple[int, int, int]],
                            blocked_vias: Set[Tuple[int, int]]):
    """Collect via obstacle cells into sets (no obstacle map modification)."""
    gx, gy = coord.to_grid(via.x, via.y)

    effective_track_block_sq = (via_track_expansion_grid + diagonal_margin) ** 2
    track_block_range = via_track_expansion_grid + 1

    for ex in range(-track_block_range, track_block_range + 1):
        for ey in range(-track_block_range, track_block_range + 1):
            if ex*ex + ey*ey <= effective_track_block_sq:
                for layer_idx in range(num_layers):
                    blocked_cells.add((gx + ex, gy + ey, layer_idx))

    for ex in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
        for ey in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
            if ex*ex + ey*ey <= via_via_expansion_grid * via_via_expansion_grid:
                blocked_vias.add((gx + ex, gy + ey))


def _collect_pad_obstacles(pad, coord: GridCoord, layer_map: Dict[str, int],
                            config: GridRouteConfig, extra_clearance: float,
                            blocked_cells: Set[Tuple[int, int, int]],
                            blocked_vias: Set[Tuple[int, int]]):
    """Collect pad obstacle cells into sets (no obstacle map modification)."""
    gx, gy = coord.to_grid(pad.global_x, pad.global_y)

    half_x_mm = pad.size_x / 2 + config.track_width / 2 + config.clearance + extra_clearance
    half_y_mm = pad.size_y / 2 + config.track_width / 2 + config.clearance + extra_clearance
    expand_x = coord.to_grid_dist(half_x_mm)
    expand_y = coord.to_grid_dist(half_y_mm)

    expanded_layers = expand_pad_layers(pad.layers, config.layers)

    for ex in range(-expand_x, expand_x + 1):
        for ey in range(-expand_y, expand_y + 1):
            for layer in expanded_layers:
                layer_idx = layer_map.get(layer)
                if layer_idx is not None:
                    blocked_cells.add((gx + ex, gy + ey, layer_idx))

    if any(layer.endswith('.Cu') for layer in expanded_layers):
        via_clear_mm = config.via_size / 2 + config.clearance
        via_expand_x = int((pad.size_x / 2 + via_clear_mm) / config.grid_step)
        via_expand_y = int((pad.size_y / 2 + via_clear_mm) / config.grid_step)
        for ex in range(-via_expand_x, via_expand_x + 1):
            for ey in range(-via_expand_y, via_expand_y + 1):
                blocked_vias.add((gx + ex, gy + ey))


def precompute_all_net_obstacles(pcb_data: PCBData, net_ids: List[int], config: GridRouteConfig,
                                   extra_clearance: float = 0.0,
                                   diagonal_margin: float = 0.25) -> Dict[int, NetObstacleData]:
    """Pre-compute obstacles for all nets to route.

    This is called once at the start of routing to build a cache that
    dramatically speeds up per-route obstacle building.

    Args:
        pcb_data: PCB data structure
        net_ids: List of net IDs to precompute obstacles for
        config: Routing configuration
        extra_clearance: Additional clearance (for diff pair routing)
        diagonal_margin: Extra margin for via blocking near diagonal segments

    Returns:
        Dict mapping net_id to NetObstacleData
    """
    cache: Dict[int, NetObstacleData] = {}
    for net_id in net_ids:
        cache[net_id] = precompute_net_obstacles(pcb_data, net_id, config,
                                                   extra_clearance, diagonal_margin)
    return cache


def add_net_obstacles_from_cache(obstacles: GridObstacleMap, cache_data: NetObstacleData):
    """Add a net's obstacles from cache using batch operations.

    This is much faster than the traditional per-segment/via/pad approach
    because it uses batch FFI calls instead of individual cell additions.

    Args:
        obstacles: The obstacle map to add to
        cache_data: Pre-computed NetObstacleData for the net
    """
    if len(cache_data.blocked_cells) > 0:
        # Pass numpy array directly to Rust (no conversion needed)
        obstacles.add_blocked_cells_batch(cache_data.blocked_cells)
    if len(cache_data.blocked_vias) > 0:
        obstacles.add_blocked_vias_batch(cache_data.blocked_vias)


def remove_net_obstacles_from_cache(obstacles: GridObstacleMap, cache_data: NetObstacleData):
    """Remove a net's obstacles from the map using batch operations.

    Used to temporarily remove a net's obstacles before routing it,
    so the router can use the net's own stubs/pads as endpoints.

    Args:
        obstacles: The obstacle map to remove from
        cache_data: Pre-computed NetObstacleData for the net
    """
    if len(cache_data.blocked_cells) > 0:
        obstacles.remove_blocked_cells_batch(cache_data.blocked_cells)
    if len(cache_data.blocked_vias) > 0:
        obstacles.remove_blocked_vias_batch(cache_data.blocked_vias)


def build_working_obstacle_map(base_obstacles: GridObstacleMap,
                                 net_obstacles_cache: Dict[int, NetObstacleData]) -> GridObstacleMap:
    """Build a working obstacle map with base + all cached net obstacles.

    This creates a complete obstacle map at startup that can be incrementally
    updated during routing (remove current net, add new route segments).

    Args:
        base_obstacles: Base obstacle map with static obstacles
        net_obstacles_cache: Pre-computed obstacles for all nets to route

    Returns:
        Working obstacle map ready for incremental updates
    """
    working = base_obstacles.clone()
    for net_id, cache_data in net_obstacles_cache.items():
        add_net_obstacles_from_cache(working, cache_data)
    return working


def update_net_obstacles_after_routing(pcb_data, net_id: int, result: Dict,
                                         config, net_obstacles_cache: Dict[int, NetObstacleData]):
    """Update the net obstacles cache after a successful route.

    Recomputes the net's obstacles to include the newly routed segments/vias.
    This should be called after add_route_to_pcb_data().

    Args:
        pcb_data: PCB data (already updated with new route)
        net_id: The net that was just routed
        result: Routing result dict with new_segments and new_vias
        config: Routing configuration
        net_obstacles_cache: Cache to update
    """
    # Recompute this net's obstacles (now includes the new route)
    net_obstacles_cache[net_id] = precompute_net_obstacles(
        pcb_data, net_id, config,
        extra_clearance=0.0, diagonal_margin=0.25
    )


# =============================================================================
# Via Placement Obstacle Cache
# =============================================================================
# Separate cache for via placement in route_plane.py. Uses different clearance
# calculations than routing (actual segment widths vs uniform track width).

@dataclass
class ViaPlacementObstacleData:
    """Cached obstacle data for via placement (used by route_plane.py).

    Stores blocked via positions and routing cells per layer for incremental
    updates when ripping up nets during plane via placement.
    """
    # Blocked via positions as numpy array of shape (N, 2) with columns [gx, gy]
    # May contain duplicates to match reference counting in GridObstacleMap
    blocked_vias: np.ndarray
    # Blocked routing cells per layer as dict: layer_name -> numpy array of shape (M, 2) [gx, gy]
    blocked_cells_by_layer: Dict[str, np.ndarray]


def precompute_via_placement_obstacles(
    pcb_data: PCBData,
    net_id: int,
    config: GridRouteConfig,
    all_copper_layers: List[str]
) -> ViaPlacementObstacleData:
    """
    Pre-compute via placement obstacle positions for a single net.

    Used for incremental updates when ripping up nets - we can quickly remove
    this net's blocked positions instead of rebuilding the entire obstacle map.

    Unlike precompute_net_obstacles (for routing), this uses actual segment widths
    for clearance calculations rather than a uniform track width.

    Args:
        pcb_data: PCB data structure
        net_id: Net ID to compute obstacles for
        config: Routing configuration
        all_copper_layers: All copper layers for routing obstacle cells

    Returns:
        ViaPlacementObstacleData with blocked_vias and blocked_cells_by_layer
    """
    import math
    coord = GridCoord(config.grid_step)

    # Collect blocked via positions from this net's segments and vias
    # Use lists (not sets) to preserve duplicate counts - the obstacle map uses
    # reference counting, so we need to remove exactly as many times as we added
    blocked_vias_list: List[Tuple[int, int]] = []

    # Collect blocked routing cells per layer
    blocked_cells_by_layer: Dict[str, List[Tuple[int, int]]] = {
        layer: [] for layer in all_copper_layers
    }

    # Process this net's segments - they block via placement
    for seg in pcb_data.segments:
        if seg.net_id != net_id:
            continue
        # Via-segment clearance (via can't overlap segment on any layer)
        seg_expansion_mm = config.via_size / 2 + seg.width / 2 + config.clearance
        seg_expansion_sq = (seg_expansion_mm / config.grid_step) ** 2
        seg_expansion_int = int(math.ceil(math.sqrt(seg_expansion_sq)))

        # Routing clearance for this segment's layer
        route_expansion_mm = config.track_width / 2 + seg.width / 2 + config.clearance
        route_expansion_grid = max(1, coord.to_grid_dist(route_expansion_mm))

        # Walk along segment using Bresenham
        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
        dx = abs(gx2 - gx1)
        dy = abs(gy2 - gy1)
        sx = 1 if gx1 < gx2 else -1
        sy = 1 if gy1 < gy2 else -1

        x, y = gx1, gy1
        if dx > dy:
            err = dx // 2
            while x != gx2:
                # Block via positions
                for ex in range(-seg_expansion_int, seg_expansion_int + 1):
                    for ey in range(-seg_expansion_int, seg_expansion_int + 1):
                        if ex*ex + ey*ey <= seg_expansion_sq:
                            blocked_vias_list.append((x + ex, y + ey))
                # Block routing cells on this segment's layer
                if seg.layer in blocked_cells_by_layer:
                    for ex in range(-route_expansion_grid, route_expansion_grid + 1):
                        for ey in range(-route_expansion_grid, route_expansion_grid + 1):
                            blocked_cells_by_layer[seg.layer].append((x + ex, y + ey))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy // 2
            while y != gy2:
                for ex in range(-seg_expansion_int, seg_expansion_int + 1):
                    for ey in range(-seg_expansion_int, seg_expansion_int + 1):
                        if ex*ex + ey*ey <= seg_expansion_sq:
                            blocked_vias_list.append((x + ex, y + ey))
                if seg.layer in blocked_cells_by_layer:
                    for ex in range(-route_expansion_grid, route_expansion_grid + 1):
                        for ey in range(-route_expansion_grid, route_expansion_grid + 1):
                            blocked_cells_by_layer[seg.layer].append((x + ex, y + ey))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        # Final point
        for ex in range(-seg_expansion_int, seg_expansion_int + 1):
            for ey in range(-seg_expansion_int, seg_expansion_int + 1):
                if ex*ex + ey*ey <= seg_expansion_sq:
                    blocked_vias_list.append((x + ex, y + ey))
        if seg.layer in blocked_cells_by_layer:
            for ex in range(-route_expansion_grid, route_expansion_grid + 1):
                for ey in range(-route_expansion_grid, route_expansion_grid + 1):
                    blocked_cells_by_layer[seg.layer].append((x + ex, y + ey))

    # Process this net's vias - they block via placement and routing on all layers
    via_via_expansion_mm = config.via_size + config.clearance
    via_via_radius_sq = (via_via_expansion_mm / config.grid_step) ** 2
    via_via_radius_int = int(math.ceil(math.sqrt(via_via_radius_sq)))

    via_route_expansion = coord.to_grid_dist(config.via_size / 2 + config.track_width / 2 + config.clearance)

    for via in pcb_data.vias:
        if via.net_id != net_id:
            continue
        gx, gy = coord.to_grid(via.x, via.y)
        # Block via positions (via-via clearance)
        for ex in range(-via_via_radius_int, via_via_radius_int + 1):
            for ey in range(-via_via_radius_int, via_via_radius_int + 1):
                if ex*ex + ey*ey <= via_via_radius_sq:
                    blocked_vias_list.append((gx + ex, gy + ey))
        # Block routing cells on all layers (via spans all layers)
        for layer in all_copper_layers:
            for ex in range(-via_route_expansion, via_route_expansion + 1):
                for ey in range(-via_route_expansion, via_route_expansion + 1):
                    if ex*ex + ey*ey <= via_route_expansion * via_route_expansion:
                        blocked_cells_by_layer[layer].append((gx + ex, gy + ey))

    # Convert to numpy arrays (keep duplicates for reference counting)
    if blocked_vias_list:
        blocked_vias_arr = np.array(blocked_vias_list, dtype=np.int32)
    else:
        blocked_vias_arr = np.empty((0, 2), dtype=np.int32)

    blocked_cells_arrays = {}
    for layer, cells in blocked_cells_by_layer.items():
        if cells:
            blocked_cells_arrays[layer] = np.array(cells, dtype=np.int32)
        else:
            blocked_cells_arrays[layer] = np.empty((0, 2), dtype=np.int32)

    return ViaPlacementObstacleData(
        blocked_vias=blocked_vias_arr,
        blocked_cells_by_layer=blocked_cells_arrays
    )


def remove_via_placement_obstacles(
    via_obstacles: 'GridObstacleMap',
    routing_obstacles_by_layer: Dict[str, 'GridObstacleMap'],
    cache: ViaPlacementObstacleData
):
    """
    Remove a net's obstacles from the via and routing obstacle maps.

    Used for incremental updates when ripping up a net during plane via placement.

    Args:
        via_obstacles: Via obstacle map to update
        routing_obstacles_by_layer: Dict of routing obstacle maps per layer
        cache: Pre-computed ViaPlacementObstacleData for this net
    """
    # Remove from via obstacle map
    if len(cache.blocked_vias) > 0:
        via_obstacles.remove_blocked_vias_batch(cache.blocked_vias)

    # Remove from routing obstacle maps (single-layer maps use layer_idx=0)
    for layer, cells in cache.blocked_cells_by_layer.items():
        if layer in routing_obstacles_by_layer and len(cells) > 0:
            obs = routing_obstacles_by_layer[layer]
            cells_3d = np.column_stack([cells, np.zeros(len(cells), dtype=np.int32)])
            obs.remove_blocked_cells_batch(cells_3d)

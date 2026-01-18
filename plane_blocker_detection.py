"""
Blocker detection for copper plane via placement.

Identifies which nets are blocking via placement or routing,
and provides rip-up functionality to remove blockers.
"""

from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional, Set

import numpy as np

from kicad_parser import PCBData, Pad
from routing_config import GridRouteConfig, GridCoord
from routing_utils import iter_pad_blocked_cells
from route_modification import remove_net_from_pcb_data, restore_net_to_pcb_data
from obstacle_cache import ViaPlacementObstacleData, precompute_via_placement_obstacles

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))
from grid_router import GridObstacleMap


def _re_add_pad_obstacles_for_net(
    pcb_data: PCBData,
    net_id: int,
    config: GridRouteConfig,
    routing_obstacles_cache: Dict[str, GridObstacleMap]
):
    """
    Re-add pad obstacles for a net after its segments were removed.

    When segments are ripped, the segment obstacles are removed from the cache.
    But pad obstacles that overlap with segment obstacles are also removed.
    This function re-adds the pad obstacles since pads still exist after rip-up.

    Uses the same rectangular-with-rounded-corners pattern as build_routing_obstacle_map.
    """
    coord = GridCoord(config.grid_step)
    pads = pcb_data.pads_by_net.get(net_id, [])

    for pad in pads:
        for layer, obstacles in routing_obstacles_cache.items():
            # Check if pad is on this layer
            if layer not in pad.layers and "*.Cu" not in pad.layers:
                continue

            gx, gy = coord.to_grid(pad.global_x, pad.global_y)
            half_width = pad.size_x / 2
            half_height = pad.size_y / 2
            margin = config.track_width / 2 + config.clearance
            corner_radius = pad.roundrect_rratio * min(pad.size_x, pad.size_y) if pad.shape == 'roundrect' else 0

            for cell_gx, cell_gy in iter_pad_blocked_cells(gx, gy, half_width, half_height, margin, config.grid_step, corner_radius):
                obstacles.add_blocked_cell(cell_gx, cell_gy, 0)  # layer_idx=0 for single-layer maps


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
    verbose: bool = False,
    find_via_position_fn=None,  # Function to find via position
    route_via_to_pad_fn=None,  # Function to route via to pad
    pending_pads: Optional[List[Dict]] = None  # Pads that still need vias (for exclusion zones)
) -> ViaPlacementResult:
    """
    Try to place a via and route to pad, ripping up blockers as needed.

    Called AFTER the fast path already failed. On first iteration, just finds and
    rips up the blocker without re-searching (since we already know search failed).

    Uses incremental obstacle updates for performance - instead of rebuilding the
    entire obstacle map after ripping a net, we just remove that net's cached obstacles.

    Args:
        find_via_position_fn: Function to find via position (injected to avoid circular import)
        route_via_to_pad_fn: Function to route via to pad (injected to avoid circular import)
    """
    ripped_net_ids = []
    ripped_data = []  # Store (blocker_id, removed_segs, removed_vias) for restoration on failure
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
            if find_via_position_fn is None:
                break  # Can't continue without find_via_position

            via_pos = find_via_position_fn(
                pad, obstacles, coord, max_search_radius,
                routing_obstacles=routing_obstacles,
                config=config,
                pad_layer=pad_layer,
                net_id=net_id,
                verbose=False,
                failed_route_positions=failed_route_positions,
                pending_pads=pending_pads
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
                if route_via_to_pad_fn is None:
                    break  # Can't continue without route_via_to_pad

                route_result = route_via_to_pad_fn(
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
        ripped_data.append((blocker, removed_segs, removed_vias))

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
        # Re-add pad obstacles for the ripped net (pads still exist, only segments removed)
        _re_add_pad_obstacles_for_net(pcb_data, blocker, config, routing_obstacles_cache)
        # Update routing_obstacles reference if it's for the current layer
        if pad_layer and pad_layer in routing_obstacles_cache:
            routing_obstacles = routing_obstacles_cache[pad_layer]

        # Add ripped net's pads to pending_pads for exclusion zones in subsequent attempts
        if pending_pads is not None:
            ripped_pads = pcb_data.pads_by_net.get(blocker, [])
            for rp in ripped_pads:
                pending_pads.append({'pad': rp, 'needs_via': True})

    # Failed - restore all ripped nets to pcb_data and obstacles
    if ripped_data:
        for blocker_id, removed_segs, removed_vias in ripped_data:
            restore_net_to_pcb_data(pcb_data, removed_segs, removed_vias)
            # Restore obstacles from cache
            if blocker_id in via_obstacle_cache:
                cache = via_obstacle_cache[blocker_id]
                if len(cache.blocked_vias) > 0:
                    obstacles.add_blocked_vias_batch(cache.blocked_vias)
                for layer, cells in cache.blocked_cells_by_layer.items():
                    if layer in routing_obstacles_cache and len(cells) > 0:
                        cells_3d = np.column_stack([cells, np.zeros(len(cells), dtype=np.int32)])
                        routing_obstacles_cache[layer].add_blocked_cells_batch(cells_3d)
        print("(restored) ", end="")

    return ViaPlacementResult(
        success=False, via_pos=None, segments=[],
        ripped_net_ids=[],  # Empty since we restored them
        via_at_pad_center=False
    )

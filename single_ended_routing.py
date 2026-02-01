"""
Single-ended net routing functions.

Routes individual nets using A* pathfinding on a grid obstacle map.
"""

import math
import time
from typing import Dict, List, Optional, Set, Tuple
from terminal_colors import RED, YELLOW, RESET

from kicad_parser import PCBData, Segment, Via
from routing_config import GridRouteConfig, GridCoord
from routing_utils import build_layer_map
from connectivity import (
    get_net_endpoints,
    get_multipoint_net_pads,
    find_closest_point_on_segments,
    compute_mst_edges,
    get_zone_connected_pad_groups
)
from obstacle_map import build_obstacle_map, get_same_net_through_hole_positions
from bresenham_utils import walk_line
from geometry_utils import simplify_path

# Import Rust router
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))

try:
    from grid_router import GridObstacleMap, GridRouter, VisualRouter
except ImportError:
    GridObstacleMap = None
    GridRouter = None
    VisualRouter = None


def print_route_stats(stats: dict, print_prefix: str = "  "):
    """Print A* routing statistics in a readable format.

    Args:
        stats: Dictionary of statistics from route_multi_with_stats
        print_prefix: Prefix for each line (default: "  ")
    """
    print(f"{print_prefix}A* Search Statistics:")
    print(f"{print_prefix}  Cells expanded:  {int(stats.get('cells_expanded', 0)):,} (popped from open set)")
    print(f"{print_prefix}  Cells pushed:    {int(stats.get('cells_pushed', 0)):,} (added to open set)")
    print(f"{print_prefix}  Cells revisited: {int(stats.get('cells_revisited', 0)):,} (path improvements)")
    print(f"{print_prefix}  Duplicate skips: {int(stats.get('duplicate_skips', 0)):,} (already in closed)")
    print(f"{print_prefix}  Path length:     {int(stats.get('path_length', 0)):,} grid steps")
    print(f"{print_prefix}  Path cost:       {int(stats.get('path_cost', 0)):,}")
    print(f"{print_prefix}  Via count:       {int(stats.get('via_count', 0)):,}")
    print(f"{print_prefix}  Initial h:       {int(stats.get('initial_h', 0)):,}")
    print(f"{print_prefix}  Final g:         {int(stats.get('final_g', 0)):,}")
    print(f"{print_prefix}  Open set size:   {int(stats.get('open_set_size', 0)):,} (at termination)")
    print(f"{print_prefix}  Closed set size: {int(stats.get('closed_set_size', 0)):,} (unique visited)")

    # Computed ratios
    h_ratio = stats.get('heuristic_ratio', 0)
    if h_ratio > 0:
        # Note: h_ratio > 1.0 is expected when using weighted A* (h_weight > 1.0)
        # The heuristic is multiplied by h_weight to trade optimality for speed
        if abs(h_ratio - 1.0) < 0.01:
            quality = "perfect heuristic"
        elif h_ratio < 1.0:
            quality = "admissible (underestimate)"
        else:
            quality = f"weighted A* (h_weight ~{h_ratio:.1f})"
        print(f"{print_prefix}  Heuristic ratio: {h_ratio:.3f} (h/g, {quality})")

    exp_ratio = stats.get('expansion_ratio', 0)
    if exp_ratio > 0:
        quality = "excellent" if exp_ratio < 2 else "good" if exp_ratio < 5 else "poor" if exp_ratio < 20 else "very poor"
        print(f"{print_prefix}  Expansion ratio: {exp_ratio:.1f}x path length ({quality})")

    revisit_ratio = stats.get('revisit_ratio', 0)
    if revisit_ratio >= 0:
        print(f"{print_prefix}  Revisit ratio:   {revisit_ratio:.3f} (path improvements / expanded)")

    skip_ratio = stats.get('skip_ratio', 0)
    if skip_ratio >= 0:
        print(f"{print_prefix}  Skip ratio:      {skip_ratio:.3f} (duplicates / total pops)")


def _print_obstacle_map(obstacles: 'GridObstacleMap', center_gx: int, center_gy: int, layer: int, radius: int = 20, print_prefix: str = ""):
    """Print a visual map of blocking around a center point."""
    print(f"{print_prefix}  Obstacle map around ({center_gx}, {center_gy}) layer={layer} (radius={radius}):")
    for dy in range(-radius, radius + 1):
        row = []
        for dx in range(-radius, radius + 1):
            cx, cy = center_gx + dx, center_gy + dy
            if dx == 0 and dy == 0:
                row.append('T')
            elif obstacles.is_blocked(cx, cy, layer):
                row.append('#')
            else:
                row.append('.')
        print(f"{print_prefix}    {''.join(row)}")


def _identify_blocking_obstacles(
    blocked_positions: List[Tuple[int, int, int]],
    pcb_data: PCBData,
    config: GridRouteConfig,
    current_net_id: int = -1
) -> Dict[int, Tuple[str, int]]:
    """
    Identify which nets are blocking specific grid positions.

    Args:
        blocked_positions: List of (gx, gy, layer) blocked cells
        pcb_data: PCB data with segments, vias, pads
        config: Routing configuration
        current_net_id: Current net ID to exclude from results

    Returns:
        Dict of net_id -> (net_name, count) for blocking nets
    """
    coord = GridCoord(config.grid_step)
    layer_map = build_layer_map(config.layers)

    # Calculate expansion radius (same as obstacle map uses)
    expansion_mm = config.track_width / 2 + config.clearance + config.track_width / 2
    expansion_grid = max(1, coord.to_grid_dist(expansion_mm))
    via_expansion_mm = config.via_size / 2 + config.track_width / 2 + config.clearance
    via_expansion_grid = max(1, coord.to_grid_dist(via_expansion_mm))

    blockers: Dict[int, Tuple[str, int]] = {}

    # Convert blocked positions to set for faster lookup
    blocked_set = set(blocked_positions)

    # Check segments
    for seg in pcb_data.segments:
        if seg.net_id == current_net_id:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue

        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

        # Check if segment's expanded cells overlap with blocked positions
        for gx, gy in walk_line(gx1, gy1, gx2, gy2):
            for ex in range(-expansion_grid, expansion_grid + 1):
                for ey in range(-expansion_grid, expansion_grid + 1):
                    if (gx + ex, gy + ey, layer_idx) in blocked_set:
                        net_name = pcb_data.nets[seg.net_id].name if seg.net_id in pcb_data.nets else f"net_{seg.net_id}"
                        if seg.net_id in blockers:
                            blockers[seg.net_id] = (net_name, blockers[seg.net_id][1] + 1)
                        else:
                            blockers[seg.net_id] = (net_name, 1)
                        break  # Found overlap, move to next segment point
                else:
                    continue
                break
            else:
                continue
            break

    # Check vias
    for via in pcb_data.vias:
        if via.net_id == current_net_id:
            continue
        gx, gy = coord.to_grid(via.x, via.y)

        # Vias block all layers within via expansion radius
        for layer_idx in range(len(config.layers)):
            for ex in range(-via_expansion_grid, via_expansion_grid + 1):
                for ey in range(-via_expansion_grid, via_expansion_grid + 1):
                    if (gx + ex, gy + ey, layer_idx) in blocked_set:
                        net_name = pcb_data.nets[via.net_id].name if via.net_id in pcb_data.nets else f"net_{via.net_id}"
                        if via.net_id in blockers:
                            blockers[via.net_id] = (net_name, blockers[via.net_id][1] + 1)
                        else:
                            blockers[via.net_id] = (net_name, 1)
                        break
                else:
                    continue
                break
            else:
                continue
            break

    # Check pads (from other nets)
    for ref, footprint in pcb_data.footprints.items():
        for pad in footprint.pads:
            if pad.net_id == current_net_id or pad.net_id == 0:
                continue

            gx, gy = coord.to_grid(pad.global_x, pad.global_y)

            # Compute pad expansion
            pad_half_x = pad.size_x / 2 if hasattr(pad, 'size_x') else 0.5
            pad_half_y = pad.size_y / 2 if hasattr(pad, 'size_y') else 0.5
            pad_expansion_x = max(1, coord.to_grid_dist(pad_half_x + config.clearance + config.track_width / 2))
            pad_expansion_y = max(1, coord.to_grid_dist(pad_half_y + config.clearance + config.track_width / 2))

            # Check pad layers
            pad_layers = []
            if pad.drill and pad.drill > 0:
                # Through-hole pad - blocks all layers
                pad_layers = list(range(len(config.layers)))
            else:
                # SMD pad - only specific layer
                for layer_name in pad.layers:
                    if layer_name in layer_map:
                        pad_layers.append(layer_map[layer_name])

            for layer_idx in pad_layers:
                for ex in range(-pad_expansion_x, pad_expansion_x + 1):
                    for ey in range(-pad_expansion_y, pad_expansion_y + 1):
                        if (gx + ex, gy + ey, layer_idx) in blocked_set:
                            net_name = pcb_data.nets[pad.net_id].name if pad.net_id in pcb_data.nets else f"net_{pad.net_id}"
                            if pad.net_id in blockers:
                                blockers[pad.net_id] = (net_name, blockers[pad.net_id][1] + 1)
                            else:
                                blockers[pad.net_id] = (net_name, 1)
                            break
                    else:
                        continue
                    break
                else:
                    continue
                break

    return blockers


def _diagnose_blocked_start(obstacles: 'GridObstacleMap', cells: List, label: str, print_prefix: str = "", track_margin: int = 0,
                            pcb_data: PCBData = None, config: GridRouteConfig = None, current_net_id: int = -1):
    """
    Diagnose why routing couldn't start from the given cells.

    Checks blocking status of start cells and their immediate neighbors.
    If pcb_data and config are provided, also identifies which nets are blocking.
    """
    if not cells:
        print(f"{print_prefix}  {label}: no cells to check")
        return

    # Check a sample of cells (first few)
    sample_cells = cells[:3] if len(cells) > 3 else cells

    for gx, gy, layer in sample_cells:
        # Check if the cell itself is blocked
        cell_blocked = obstacles.is_blocked(gx, gy, layer)

        # Check neighbors (8-connected)
        blocked_neighbors = 0
        total_neighbors = 0
        blocked_details = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                total_neighbors += 1
                # Check with margin if specified
                if track_margin > 0:
                    neighbor_blocked = False
                    for mx in range(-track_margin, track_margin + 1):
                        for my in range(-track_margin, track_margin + 1):
                            if obstacles.is_blocked(gx + dx + mx, gy + dy + my, layer):
                                neighbor_blocked = True
                                break
                        if neighbor_blocked:
                            break
                else:
                    neighbor_blocked = obstacles.is_blocked(gx + dx, gy + dy, layer)
                if neighbor_blocked:
                    blocked_neighbors += 1
                    blocked_details.append(f"({gx+dx},{gy+dy})")

        status = "BLOCKED" if cell_blocked else "ok"
        margin_str = f" (margin={track_margin})" if track_margin > 0 else ""
        print(f"{print_prefix}  {label} cell ({gx}, {gy}, layer={layer}): {status}, {blocked_neighbors}/{total_neighbors} neighbors blocked{margin_str}")
        # Show which specific neighbors are blocked for debugging
        if blocked_neighbors == total_neighbors and blocked_neighbors > 0:
            print(f"{print_prefix}    ALL neighbors blocked: {', '.join(blocked_details)}")

        # Identify what's blocking if pcb_data and config are provided
        if blocked_neighbors > 0 and pcb_data is not None and config is not None:
            # Collect blocked neighbor positions
            blocked_positions = []
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    if track_margin > 0:
                        for mx in range(-track_margin, track_margin + 1):
                            for my in range(-track_margin, track_margin + 1):
                                if obstacles.is_blocked(gx + dx + mx, gy + dy + my, layer):
                                    blocked_positions.append((gx + dx + mx, gy + dy + my, layer))
                    else:
                        if obstacles.is_blocked(gx + dx, gy + dy, layer):
                            blocked_positions.append((gx + dx, gy + dy, layer))

            if blocked_positions:
                blockers = _identify_blocking_obstacles(blocked_positions, pcb_data, config, current_net_id)
                if blockers:
                    # Sort by count descending
                    sorted_blockers = sorted(blockers.items(), key=lambda x: x[1][1], reverse=True)
                    blocker_strs = [f"{name}({count})" for net_id, (name, count) in sorted_blockers[:5]]
                    print(f"{print_prefix}    Blocking obstacles: {', '.join(blocker_strs)}")


def _probe_route_with_frontier(
    router: 'GridRouter',
    obstacles: 'GridObstacleMap',
    forward_sources: List,
    forward_targets: List,
    config: 'GridRouteConfig',
    print_prefix: str = "",
    direction_labels: Tuple[str, str] = ("forward", "backward"),
    track_margin: int = 0,
    pcb_data: PCBData = None,
    current_net_id: int = -1
) -> Tuple[Optional[List], int, List, List, bool, int, int]:
    """
    Probe routing with fail-fast on stuck directions.

    Uses bidirectional probing to detect if either endpoint is blocked early,
    avoiding expensive full searches that will fail anyway.

    Args:
        router: GridRouter instance
        obstacles: Obstacle map
        forward_sources: Source cells for forward direction
        forward_targets: Target cells for forward direction
        config: Routing configuration
        print_prefix: Prefix for print messages (e.g., "  " or "      ")
        direction_labels: Names for forward/backward directions
        track_margin: Extra margin in grid cells for wide tracks (power nets)
        pcb_data: Optional PCB data for blocking obstacle identification
        current_net_id: Current net ID (for excluding from blocking analysis)

    Returns:
        (path, total_iterations, forward_blocked, backward_blocked, reversed_path, forward_iters, backward_iters)
        - path: The found path or None
        - total_iterations: Total iterations used
        - forward_blocked: Blocked cells from forward direction (for rip-up analysis)
        - backward_blocked: Blocked cells from backward direction (for rip-up analysis)
        - reversed_path: Whether path was found going backwards
        - forward_iters: Iterations used in forward direction
        - backward_iters: Iterations used in backward direction
    """
    first_label, second_label = direction_labels
    probe_iterations = config.max_probe_iterations

    # Track iterations per direction (first/second maps to forward/backward based on labels)
    first_total_iters = 0
    second_total_iters = 0

    # Probe forward direction
    path, iterations, blocked_cells = router.route_with_frontier(
        obstacles, forward_sources, forward_targets, probe_iterations, track_margin=track_margin)
    first_probe_iters = iterations
    first_total_iters = first_probe_iters
    first_blocked = blocked_cells
    total_iterations = first_probe_iters
    reversed_path = False

    # Track blocked cells for both directions
    forward_blocked = first_blocked
    backward_blocked = []

    # Helper to map first/second to forward/backward based on labels
    def get_fwd_bwd_iters():
        if first_label == "forward":
            return first_total_iters, second_total_iters
        else:
            return second_total_iters, first_total_iters

    if path is not None:
        # Found in first probe
        forward_blocked = []  # Success - clear blocked cells
        fwd_iters, bwd_iters = get_fwd_bwd_iters()
        return path, total_iterations, forward_blocked, backward_blocked, reversed_path, fwd_iters, bwd_iters

    # Probe backward direction
    path, iterations, blocked_cells = router.route_with_frontier(
        obstacles, forward_targets, forward_sources, probe_iterations, track_margin=track_margin)
    second_probe_iters = iterations
    second_total_iters = second_probe_iters
    second_blocked = blocked_cells
    total_iterations += second_probe_iters
    backward_blocked = second_blocked

    if path is not None:
        # Found in second probe
        backward_blocked = []  # Success - clear blocked cells
        fwd_iters, bwd_iters = get_fwd_bwd_iters()
        return path, total_iterations, forward_blocked, backward_blocked, True, fwd_iters, bwd_iters

    # Both probes failed to find a path - check if both reached max iterations
    # Only try full search if BOTH probes reached max-probe-iterations (meaning both directions are worth exploring)
    first_reached_max = first_probe_iters >= probe_iterations
    second_reached_max = second_probe_iters >= probe_iterations

    if not (first_reached_max and second_reached_max):
        # At least one probe didn't reach max - that direction is stuck, skip full search
        if not first_reached_max and not second_reached_max:
            print(f"{print_prefix}Both directions stuck ({first_label}={first_probe_iters}, {second_label}={second_probe_iters} < {probe_iterations})")
            _diagnose_blocked_start(obstacles, forward_sources, first_label, print_prefix, track_margin,
                                    pcb_data=pcb_data, config=config, current_net_id=current_net_id)
            _diagnose_blocked_start(obstacles, forward_targets, second_label, print_prefix, track_margin,
                                    pcb_data=pcb_data, config=config, current_net_id=current_net_id)
        elif not first_reached_max:
            print(f"{print_prefix}{first_label} stuck ({first_probe_iters} < {probe_iterations}), {second_label}={second_probe_iters}")
            _diagnose_blocked_start(obstacles, forward_sources, first_label, print_prefix, track_margin,
                                    pcb_data=pcb_data, config=config, current_net_id=current_net_id)
        else:
            print(f"{print_prefix}{second_label} stuck ({second_probe_iters} < {probe_iterations}), {first_label}={first_probe_iters}")
            _diagnose_blocked_start(obstacles, forward_targets, second_label, print_prefix, track_margin,
                                    pcb_data=pcb_data, config=config, current_net_id=current_net_id)
            # Print visual obstacle map around the stuck target
            if forward_targets and config.debug_lines:
                tgt = forward_targets[0]
                _print_obstacle_map(obstacles, tgt[0], tgt[1], tgt[2], radius=15, print_prefix=print_prefix)
        fwd_iters, bwd_iters = get_fwd_bwd_iters()
        return None, total_iterations, forward_blocked, backward_blocked, False, fwd_iters, bwd_iters

    # Both probes reached max iterations - do full search on forward direction
    print(f"{print_prefix}Probe: {first_label}={first_probe_iters}, {second_label}={second_probe_iters} iters, trying {first_label} with full iterations...")

    path, full_iters, full_blocked = router.route_with_frontier(
        obstacles, forward_sources, forward_targets, config.max_iterations, track_margin=track_margin)
    first_total_iters += full_iters
    total_iterations += full_iters

    if path is not None:
        forward_blocked = []
        fwd_iters, bwd_iters = get_fwd_bwd_iters()
        return path, total_iterations, forward_blocked, backward_blocked, False, fwd_iters, bwd_iters

    # Forward failed, try backward
    print(f"{print_prefix}No route found after {full_iters} iterations ({first_label}), trying {second_label}...")
    forward_blocked = full_blocked

    path, backward_full_iters, backward_full_blocked = router.route_with_frontier(
        obstacles, forward_targets, forward_sources, config.max_iterations, track_margin=track_margin)
    second_total_iters += backward_full_iters
    total_iterations += backward_full_iters

    if path is not None:
        backward_blocked = []
        fwd_iters, bwd_iters = get_fwd_bwd_iters()
        return path, total_iterations, forward_blocked, backward_blocked, True, fwd_iters, bwd_iters

    backward_blocked = backward_full_blocked
    fwd_iters, bwd_iters = get_fwd_bwd_iters()
    return None, total_iterations, forward_blocked, backward_blocked, False, fwd_iters, bwd_iters


def route_net(pcb_data: PCBData, net_id: int, config: GridRouteConfig,
              unrouted_stubs: Optional[List[Tuple[float, float]]] = None) -> Optional[dict]:
    """Route a single net using the Rust router."""
    # Find endpoints (segments or pads)
    sources, targets, error = get_net_endpoints(pcb_data, net_id, config)
    if error:
        print(f"  {error}")
        return None

    if not sources or not targets:
        print(f"  No valid source/target endpoints found")
        return None

    coord = GridCoord(config.grid_step)
    layer_names = config.layers

    # Extract grid-only coords for routing
    sources_grid = [(s[0], s[1], s[2]) for s in sources]
    targets_grid = [(t[0], t[1], t[2]) for t in targets]

    # Get stub free ends for proximity zone checking (where routing actually starts/ends)
    # This is more accurate than checking all segment endpoints
    free_end_sources, free_end_targets, _ = get_net_endpoints(pcb_data, net_id, config, use_stub_free_ends=True)
    if free_end_sources:
        prox_check_sources = [(s[0], s[1], s[2]) for s in free_end_sources]
    else:
        prox_check_sources = sources_grid  # Fallback to all endpoints
    if free_end_targets:
        prox_check_targets = [(t[0], t[1], t[2]) for t in free_end_targets]
    else:
        prox_check_targets = targets_grid  # Fallback to all endpoints

    # Build obstacles
    obstacles = build_obstacle_map(pcb_data, config, net_id, unrouted_stubs)

    # Add source and target positions as allowed cells to override BGA zone blocking
    # This only affects BGA zone blocking, not regular obstacle blocking (tracks, stubs, pads)
    allow_radius = 10
    for gx, gy, _ in sources_grid + targets_grid:
        for dx in range(-allow_radius, allow_radius + 1):
            for dy in range(-allow_radius, allow_radius + 1):
                obstacles.add_allowed_cell(gx + dx, gy + dy)

    # Mark exact source/target cells so routing can start/end there even if blocked by
    # adjacent track expansion (but NOT blocked by BGA zones - use allowed_cells for that)
    # NOTE: Must pass layer to only allow override on the specific layer of the endpoint
    for gx, gy, layer in sources_grid + targets_grid:
        obstacles.add_source_target_cell(gx, gy, layer)

    # Calculate vertical attraction parameters
    attraction_radius_grid = coord.to_grid_dist(config.vertical_attraction_radius) if config.vertical_attraction_radius > 0 else 0
    attraction_bonus = int(config.vertical_attraction_cost * 1000 / config.grid_step) if config.vertical_attraction_cost > 0 else 0

    # Check which proximity zones the stub free ends are in for precise heuristic estimate
    src_in_stub = any(obstacles.get_stub_proximity_cost(gx, gy) > 0 for gx, gy, _ in prox_check_sources)
    src_in_bga = any(obstacles.is_in_bga_proximity(gx, gy) for gx, gy, _ in prox_check_sources)
    tgt_in_stub = any(obstacles.get_stub_proximity_cost(gx, gy) > 0 for gx, gy, _ in prox_check_targets)
    tgt_in_bga = any(obstacles.is_in_bga_proximity(gx, gy) for gx, gy, _ in prox_check_targets)
    prox_h_cost = config.get_proximity_heuristic_for_zones(src_in_stub, src_in_bga, tgt_in_stub, tgt_in_bga)
    if config.verbose:
        zones = []
        if src_in_stub: zones.append("src:stub")
        if src_in_bga: zones.append("src:bga")
        if tgt_in_stub: zones.append("tgt:stub")
        if tgt_in_bga: zones.append("tgt:bga")
        print(f"  proximity_heuristic_cost={prox_h_cost} zones=[{', '.join(zones) if zones else 'none'}]")

    router = GridRouter(via_cost=config.via_cost * 1000, h_weight=config.heuristic_weight,
                        turn_cost=config.turn_cost, via_proximity_cost=int(config.via_proximity_cost),
                        vertical_attraction_radius=attraction_radius_grid,
                        vertical_attraction_bonus=attraction_bonus,
                        layer_costs=config.get_layer_costs(),
                        proximity_heuristic_cost=prox_h_cost,
                        layer_direction_preferences=config.get_layer_direction_preferences(),
                        direction_preference_cost=config.direction_preference_cost)

    # Calculate track margin for wide power tracks
    # Use ceiling + 1 to account for grid quantization and diagonal track approaches
    # Compare against layer-specific width (not base track_width) to handle impedance routing
    net_track_width = config.get_net_track_width(net_id, config.layers[0])
    layer_track_width = config.get_track_width(config.layers[0])
    extra_half_width = (net_track_width - layer_track_width) / 2
    track_margin = (int(math.ceil(extra_half_width / config.grid_step)) + 1) if extra_half_width > 0 else 0

    # Determine direction order (always deterministic)
    if config.direction_order in ("backwards", "backward"):
        start_backwards = True
    else:  # "forward" or default
        start_backwards = False

    # Set up first and second direction based on order
    if start_backwards:
        first_sources, first_targets = targets_grid, sources_grid
        second_sources, second_targets = sources_grid, targets_grid
        first_label, second_label = "backward", "forward"
    else:
        first_sources, first_targets = sources_grid, targets_grid
        second_sources, second_targets = targets_grid, sources_grid
        first_label, second_label = "forward", "backward"

    # Quick probe phase: test both directions with limited iterations to detect if stuck
    reversed_path = False
    total_iterations = 0
    probe_iterations = config.max_probe_iterations

    # Probe first direction
    path, iterations, _ = router.route_multi(obstacles, first_sources, first_targets, probe_iterations, track_margin=track_margin)
    first_probe_iters = iterations
    total_iterations = first_probe_iters

    if path is None:
        # Probe second direction
        path, iterations, _ = router.route_multi(obstacles, second_sources, second_targets, probe_iterations, track_margin=track_margin)
        second_probe_iters = iterations
        total_iterations += second_probe_iters

        if path is not None:
            reversed_path = not start_backwards
        else:
            # Both probes failed - only try full search if BOTH reached max iterations
            first_reached_max = first_probe_iters >= probe_iterations
            second_reached_max = second_probe_iters >= probe_iterations

            if not (first_reached_max and second_reached_max):
                # At least one probe didn't reach max - that direction is stuck, skip full search
                if not first_reached_max and not second_reached_max:
                    print(f"Both directions stuck ({first_label}={first_probe_iters}, {second_label}={second_probe_iters} < {probe_iterations})")
                    _diagnose_blocked_start(obstacles, first_sources, first_label, "", track_margin,
                                            pcb_data=pcb_data, config=config, current_net_id=net_id)
                    _diagnose_blocked_start(obstacles, second_sources, second_label, "", track_margin,
                                            pcb_data=pcb_data, config=config, current_net_id=net_id)
                elif not first_reached_max:
                    print(f"{first_label} stuck ({first_probe_iters} < {probe_iterations}), {second_label}={second_probe_iters}")
                    _diagnose_blocked_start(obstacles, first_sources, first_label, "", track_margin,
                                            pcb_data=pcb_data, config=config, current_net_id=net_id)
                else:
                    print(f"{second_label} stuck ({second_probe_iters} < {probe_iterations}), {first_label}={first_probe_iters}")
                    _diagnose_blocked_start(obstacles, second_sources, second_label, "", track_margin,
                                            pcb_data=pcb_data, config=config, current_net_id=net_id)
            else:
                # Both probes reached max - do full search on first direction
                print(f"Probe: {first_label}={first_probe_iters}, {second_label}={second_probe_iters} iters, trying {first_label} with full iterations...")

                path, full_iters, _ = router.route_multi(obstacles, first_sources, first_targets, config.max_iterations, track_margin=track_margin)
                total_iterations += full_iters

                if path is not None:
                    reversed_path = not start_backwards
                else:
                    # First direction failed, try second
                    print(f"No route found after {full_iters} iterations ({first_label}), trying {second_label}...")
                    path, fallback_full_iters, _ = router.route_multi(obstacles, second_sources, second_targets, config.max_iterations, track_margin=track_margin)
                    total_iterations += fallback_full_iters
                    if path is not None:
                        reversed_path = start_backwards

    if path is None:
        print(f"No route found after {total_iterations} iterations (both directions)")
        return None

    print(f"Route found in {total_iterations} iterations, path length: {len(path)}")

    # If path was found in reverse direction, swap sources/targets for connection logic
    if reversed_path:
        sources, targets = targets, sources

    # Find which source/target the path actually connects to
    path_start = path[0]
    path_end = path[-1]

    start_original = None
    for s in sources:
        if s[0] == path_start[0] and s[1] == path_start[1] and s[2] == path_start[2]:
            start_original = (s[3], s[4], layer_names[s[2]])
            break

    end_original = None
    for t in targets:
        if t[0] == path_end[0] and t[1] == path_end[1] and t[2] == path_end[2]:
            end_original = (t[3], t[4], layer_names[t[2]])
            break

    # Get through-hole pad positions for this net (layer transitions without via)
    through_hole_positions = get_same_net_through_hole_positions(pcb_data, net_id, config)

    # Simplify path by removing collinear intermediate points
    path = simplify_path(path)

    # Convert path to segments and vias
    new_segments = []
    new_vias = []

    # Add connecting segment from original start to first path point if needed
    if start_original:
        first_grid_x, first_grid_y = coord.to_float(path_start[0], path_start[1])
        orig_x, orig_y, orig_layer = start_original
        if abs(orig_x - first_grid_x) > 0.001 or abs(orig_y - first_grid_y) > 0.001:
            seg = Segment(
                start_x=orig_x, start_y=orig_y,
                end_x=first_grid_x, end_y=first_grid_y,
                width=config.get_net_track_width(net_id, orig_layer),
                layer=orig_layer,
                net_id=net_id
            )
            new_segments.append(seg)

    for i in range(len(path) - 1):
        gx1, gy1, layer1 = path[i]
        gx2, gy2, layer2 = path[i + 1]

        x1, y1 = coord.to_float(gx1, gy1)
        x2, y2 = coord.to_float(gx2, gy2)

        if layer1 != layer2:
            # Check if layer change is at an existing through-hole pad
            # If so, skip creating a via - the pad provides the layer transition
            if (gx1, gy1) not in through_hole_positions:
                via = Via(
                    x=x1, y=y1,
                    size=config.via_size,
                    drill=config.via_drill,
                    layers=["F.Cu", "B.Cu"],  # Always through-hole
                    net_id=net_id
                )
                new_vias.append(via)
        else:
            if (x1, y1) != (x2, y2):
                layer_name = layer_names[layer1]
                seg = Segment(
                    start_x=x1, start_y=y1,
                    end_x=x2, end_y=y2,
                    width=config.get_net_track_width(net_id, layer_name),
                    layer=layer_name,
                    net_id=net_id
                )
                new_segments.append(seg)

    # Add connecting segment from last path point to original end if needed
    if end_original:
        last_grid_x, last_grid_y = coord.to_float(path_end[0], path_end[1])
        orig_x, orig_y, orig_layer = end_original
        if abs(orig_x - last_grid_x) > 0.001 or abs(orig_y - last_grid_y) > 0.001:
            seg = Segment(
                start_x=last_grid_x, start_y=last_grid_y,
                end_x=orig_x, end_y=orig_y,
                width=config.get_net_track_width(net_id, orig_layer),
                layer=orig_layer,
                net_id=net_id
            )
            new_segments.append(seg)

    return {
        'new_segments': new_segments,
        'new_vias': new_vias,
        'iterations': total_iterations,
        'path_length': len(path),
        'path': path,  # Include raw path for incremental obstacle updates
    }


def route_net_with_obstacles(pcb_data: PCBData, net_id: int, config: GridRouteConfig,
                              obstacles: GridObstacleMap,
                              attraction_path: Optional[List[Tuple[int, int, int]]] = None) -> Optional[dict]:
    """Route a single net using pre-built obstacles (for incremental routing).

    Args:
        pcb_data: PCB data
        net_id: Net ID to route
        config: Routing configuration
        obstacles: Pre-built obstacle map
        attraction_path: Optional path to attract to (for bus routing).
                        List of (gx, gy, layer) tuples from a previously routed neighbor.
    """
    # Find endpoints (segments or pads)
    sources, targets, error = get_net_endpoints(pcb_data, net_id, config)
    if error:
        print(f"  {error}")
        return None

    if not sources or not targets:
        print(f"  No valid source/target endpoints found")
        return None

    coord = GridCoord(config.grid_step)
    layer_names = config.layers

    sources_grid = [(s[0], s[1], s[2]) for s in sources]
    targets_grid = [(t[0], t[1], t[2]) for t in targets]

    # Get stub free ends for proximity zone checking (where routing actually starts/ends)
    free_end_sources, free_end_targets, _ = get_net_endpoints(pcb_data, net_id, config, use_stub_free_ends=True)
    if free_end_sources:
        prox_check_sources = [(s[0], s[1], s[2]) for s in free_end_sources]
    else:
        prox_check_sources = sources_grid
    if free_end_targets:
        prox_check_targets = [(t[0], t[1], t[2]) for t in free_end_targets]
    else:
        prox_check_targets = targets_grid

    # Add source and target positions as allowed cells to override BGA zone blocking
    # This only affects BGA zone blocking, not regular obstacle blocking (tracks, stubs, pads)
    allow_radius = 10
    for gx, gy, _ in sources_grid + targets_grid:
        for dx in range(-allow_radius, allow_radius + 1):
            for dy in range(-allow_radius, allow_radius + 1):
                obstacles.add_allowed_cell(gx + dx, gy + dy)

    # Mark exact source/target cells so routing can start/end there even if blocked by
    # adjacent track expansion (but NOT blocked by BGA zones - use allowed_cells for that)
    # NOTE: Must pass layer to only allow override on the specific layer of the endpoint
    for gx, gy, layer in sources_grid + targets_grid:
        obstacles.add_source_target_cell(gx, gy, layer)

    # Calculate vertical attraction parameters
    attraction_radius_grid = coord.to_grid_dist(config.vertical_attraction_radius) if config.vertical_attraction_radius > 0 else 0
    attraction_bonus = int(config.vertical_attraction_cost * 1000 / config.grid_step) if config.vertical_attraction_cost > 0 else 0

    # Check which proximity zones the stub free ends are in for precise heuristic estimate
    src_in_stub = any(obstacles.get_stub_proximity_cost(gx, gy) > 0 for gx, gy, _ in prox_check_sources)
    src_in_bga = any(obstacles.is_in_bga_proximity(gx, gy) for gx, gy, _ in prox_check_sources)
    tgt_in_stub = any(obstacles.get_stub_proximity_cost(gx, gy) > 0 for gx, gy, _ in prox_check_targets)
    tgt_in_bga = any(obstacles.is_in_bga_proximity(gx, gy) for gx, gy, _ in prox_check_targets)
    prox_h_cost = config.get_proximity_heuristic_for_zones(src_in_stub, src_in_bga, tgt_in_stub, tgt_in_bga)
    if config.verbose:
        zones = []
        if src_in_stub: zones.append("src:stub")
        if src_in_bga: zones.append("src:bga")
        if tgt_in_stub: zones.append("tgt:stub")
        if tgt_in_bga: zones.append("tgt:bga")
        print(f"  proximity_heuristic_cost={prox_h_cost} zones=[{', '.join(zones) if zones else 'none'}]")

    # Calculate bus attraction parameters
    bus_attraction_radius_grid = coord.to_grid_dist(config.bus_attraction_radius) if config.bus_attraction_radius > 0 else 0
    bus_attraction_bonus = int(config.bus_attraction_bonus) if config.bus_attraction_bonus > 0 else 0

    router = GridRouter(via_cost=config.via_cost * 1000, h_weight=config.heuristic_weight,
                        turn_cost=config.turn_cost, via_proximity_cost=int(config.via_proximity_cost),
                        vertical_attraction_radius=attraction_radius_grid,
                        vertical_attraction_bonus=attraction_bonus,
                        layer_costs=config.get_layer_costs(),
                        proximity_heuristic_cost=prox_h_cost,
                        layer_direction_preferences=config.get_layer_direction_preferences(),
                        direction_preference_cost=config.direction_preference_cost,
                        attraction_radius=bus_attraction_radius_grid,
                        attraction_bonus=bus_attraction_bonus)

    # Set attraction path for bus routing (if provided)
    if attraction_path:
        router.set_attraction_path(attraction_path)

    # Calculate track margin for wide power tracks
    # Use ceiling + 1 to account for grid quantization and diagonal track approaches
    # Compare against layer-specific width (not base track_width) to handle impedance routing
    net_track_width = config.get_net_track_width(net_id, config.layers[0])
    layer_track_width = config.get_track_width(config.layers[0])
    extra_half_width = (net_track_width - layer_track_width) / 2
    track_margin = (int(math.ceil(extra_half_width / config.grid_step)) + 1) if extra_half_width > 0 else 0

    # Determine direction order (always deterministic)
    start_backwards = config.direction_order in ("backwards", "backward")

    # Set up forward/backward based on direction preference
    if start_backwards:
        forward_sources, forward_targets = targets_grid, sources_grid
        direction_labels = ("backward", "forward")
    else:
        forward_sources, forward_targets = sources_grid, targets_grid
        direction_labels = ("forward", "backward")

    # Use probe routing helper
    if config.verbose:
        print(f"    GridRouter sources: {forward_sources[:3]}{'...' if len(forward_sources) > 3 else ''}")
        print(f"    GridRouter targets: {forward_targets[:3]}{'...' if len(forward_targets) > 3 else ''}")
    path, total_iterations, forward_blocked, backward_blocked, reversed_path, fwd_iters, bwd_iters = _probe_route_with_frontier(
        router, obstacles, forward_sources, forward_targets, config,
        print_prefix="", direction_labels=direction_labels, track_margin=track_margin,
        pcb_data=pcb_data, current_net_id=net_id
    )

    # Adjust reversed_path based on start direction
    if start_backwards and path is not None:
        reversed_path = not reversed_path

    if path is None:
        print(f"No route found after {total_iterations} iterations (both directions)")
        return {
            'failed': True,
            'iterations': total_iterations,
            'blocked_cells_forward': forward_blocked,
            'blocked_cells_backward': backward_blocked,
            'iterations_forward': fwd_iters,
            'iterations_backward': bwd_iters,
        }

    print(f"Route found in {total_iterations} iterations, path length: {len(path)}")

    # Collect and print stats if enabled
    if config.collect_stats:
        # Re-run with stats collection on the same direction that succeeded
        # Use the actual source/target that worked
        if reversed_path:
            stats_sources, stats_targets = forward_targets, forward_sources
        else:
            stats_sources, stats_targets = forward_sources, forward_targets
        _, _, stats = router.route_multi(
            obstacles, stats_sources, stats_targets, config.max_iterations, track_margin=track_margin)
        print_route_stats(stats)

    if reversed_path:
        sources, targets = targets, sources

    path_start = path[0]
    path_end = path[-1]

    start_original = None
    for s in sources:
        if s[0] == path_start[0] and s[1] == path_start[1] and s[2] == path_start[2]:
            start_original = (s[3], s[4], layer_names[s[2]])
            break

    end_original = None
    for t in targets:
        if t[0] == path_end[0] and t[1] == path_end[1] and t[2] == path_end[2]:
            end_original = (t[3], t[4], layer_names[t[2]])
            break

    # Get through-hole pad positions for this net (layer transitions without via)
    through_hole_positions = get_same_net_through_hole_positions(pcb_data, net_id, config)

    # Simplify path by removing collinear intermediate points
    path = simplify_path(path)

    new_segments = []
    new_vias = []

    if start_original:
        first_grid_x, first_grid_y = coord.to_float(path_start[0], path_start[1])
        orig_x, orig_y, orig_layer = start_original
        if abs(orig_x - first_grid_x) > 0.001 or abs(orig_y - first_grid_y) > 0.001:
            seg = Segment(
                start_x=orig_x, start_y=orig_y,
                end_x=first_grid_x, end_y=first_grid_y,
                width=config.get_net_track_width(net_id, orig_layer),
                layer=orig_layer,
                net_id=net_id
            )
            new_segments.append(seg)

    for i in range(len(path) - 1):
        gx1, gy1, layer1 = path[i]
        gx2, gy2, layer2 = path[i + 1]

        x1, y1 = coord.to_float(gx1, gy1)
        x2, y2 = coord.to_float(gx2, gy2)

        if layer1 != layer2:
            # Check if layer change is at an existing through-hole pad
            # If so, skip creating a via - the pad provides the layer transition
            if (gx1, gy1) not in through_hole_positions:
                via = Via(
                    x=x1, y=y1,
                    size=config.via_size,
                    drill=config.via_drill,
                    layers=["F.Cu", "B.Cu"],  # Always through-hole
                    net_id=net_id
                )
                new_vias.append(via)
        else:
            if (x1, y1) != (x2, y2):
                layer_name = layer_names[layer1]
                seg = Segment(
                    start_x=x1, start_y=y1,
                    end_x=x2, end_y=y2,
                    width=config.get_net_track_width(net_id, layer_name),
                    layer=layer_name,
                    net_id=net_id
                )
                new_segments.append(seg)

    if end_original:
        last_grid_x, last_grid_y = coord.to_float(path_end[0], path_end[1])
        orig_x, orig_y, orig_layer = end_original
        if abs(orig_x - last_grid_x) > 0.001 or abs(orig_y - last_grid_y) > 0.001:
            seg = Segment(
                start_x=last_grid_x, start_y=last_grid_y,
                end_x=orig_x, end_y=orig_y,
                width=config.get_net_track_width(net_id, orig_layer),
                layer=orig_layer,
                net_id=net_id
            )
            new_segments.append(seg)

    return {
        'new_segments': new_segments,
        'new_vias': new_vias,
        'iterations': total_iterations,
        'path_length': len(path),
        'path': path,
    }


def route_net_with_visualization(pcb_data: PCBData, net_id: int, config: GridRouteConfig,
                                  obstacles: GridObstacleMap, vis_callback) -> Optional[dict]:
    """Route a single net with real-time visualization.

    Uses VisualRouter for incremental stepping with visualization callbacks.

    Args:
        pcb_data: PCB data
        net_id: Net ID to route
        config: Routing configuration
        obstacles: Pre-built obstacle map
        vis_callback: Visualization callback (implements VisualizationCallback protocol)

    Returns:
        Routing result dict, or None on failure
    """
    if VisualRouter is None:
        print("  VisualRouter not available, falling back to standard routing")
        return route_net_with_obstacles(pcb_data, net_id, config, obstacles)

    # Find endpoints (segments or pads)
    sources, targets, error = get_net_endpoints(pcb_data, net_id, config)
    if error:
        print(f"  {error}")
        return None

    if not sources or not targets:
        print(f"  No valid source/target endpoints found")
        return None

    coord = GridCoord(config.grid_step)
    layer_names = config.layers

    sources_grid = [(s[0], s[1], s[2]) for s in sources]
    targets_grid = [(t[0], t[1], t[2]) for t in targets]

    # Get stub free ends for proximity zone checking (where routing actually starts/ends)
    free_end_sources, free_end_targets, _ = get_net_endpoints(pcb_data, net_id, config, use_stub_free_ends=True)
    if free_end_sources:
        prox_check_sources = [(s[0], s[1], s[2]) for s in free_end_sources]
    else:
        prox_check_sources = sources_grid
    if free_end_targets:
        prox_check_targets = [(t[0], t[1], t[2]) for t in free_end_targets]
    else:
        prox_check_targets = targets_grid

    # Add source and target positions as allowed cells to override BGA zone blocking
    allow_radius = 10
    for gx, gy, _ in sources_grid + targets_grid:
        for dx in range(-allow_radius, allow_radius + 1):
            for dy in range(-allow_radius, allow_radius + 1):
                obstacles.add_allowed_cell(gx + dx, gy + dy)

    # Mark exact source/target cells so routing can start/end there
    for gx, gy, layer in sources_grid + targets_grid:
        obstacles.add_source_target_cell(gx, gy, layer)

    # Check which proximity zones the stub free ends are in for precise heuristic estimate
    src_in_stub = any(obstacles.get_stub_proximity_cost(gx, gy) > 0 for gx, gy, _ in prox_check_sources)
    src_in_bga = any(obstacles.is_in_bga_proximity(gx, gy) for gx, gy, _ in prox_check_sources)
    tgt_in_stub = any(obstacles.get_stub_proximity_cost(gx, gy) > 0 for gx, gy, _ in prox_check_targets)
    tgt_in_bga = any(obstacles.is_in_bga_proximity(gx, gy) for gx, gy, _ in prox_check_targets)
    prox_h_cost = config.get_proximity_heuristic_for_zones(src_in_stub, src_in_bga, tgt_in_stub, tgt_in_bga)
    if config.verbose:
        zones = []
        if src_in_stub: zones.append("src:stub")
        if src_in_bga: zones.append("src:bga")
        if tgt_in_stub: zones.append("tgt:stub")
        if tgt_in_bga: zones.append("tgt:bga")
        print(f"  proximity_heuristic_cost={prox_h_cost} zones=[{', '.join(zones) if zones else 'none'}]")

    # Calculate vertical attraction parameters
    attraction_radius_grid = coord.to_grid_dist(config.vertical_attraction_radius) if config.vertical_attraction_radius > 0 else 0
    attraction_bonus = int(config.vertical_attraction_cost * 1000 / config.grid_step) if config.vertical_attraction_cost > 0 else 0

    # Determine direction order (always deterministic)
    if config.direction_order in ("backwards", "backward"):
        start_backwards = True
    else:
        start_backwards = False

    if start_backwards:
        first_sources, first_targets = targets_grid, sources_grid
        second_sources, second_targets = sources_grid, targets_grid
        first_label, second_label = "backward", "forward"
    else:
        first_sources, first_targets = sources_grid, targets_grid
        second_sources, second_targets = targets_grid, sources_grid
        first_label, second_label = "forward", "backward"

    # Create visual router
    router = VisualRouter(via_cost=config.via_cost * 1000, h_weight=config.heuristic_weight,
                          turn_cost=config.turn_cost, via_proximity_cost=int(config.via_proximity_cost),
                          vertical_attraction_radius=attraction_radius_grid,
                          vertical_attraction_bonus=attraction_bonus,
                          layer_costs=config.get_layer_costs(),
                          proximity_heuristic_cost=prox_h_cost)

    # Try first direction with visualization
    if config.verbose:
        print(f"    VisualRouter sources: {first_sources[:3]}{'...' if len(first_sources) > 3 else ''}")
        print(f"    VisualRouter targets: {first_targets[:3]}{'...' if len(first_targets) > 3 else ''}")
    router.init(first_sources, first_targets, config.max_iterations)
    path = None
    total_iterations = 0
    direction_used = first_label

    while not router.is_done():
        # Check if we should pause
        while vis_callback.should_pause():
            # Keep handling events while paused
            snapshot = router.step(obstacles, 0)  # 0 iterations, just get current state
            if not vis_callback.on_route_step(snapshot):
                return None  # User quit

        # Get iterations to run
        iters = vis_callback.get_iterations_per_step()
        snapshot = router.step(obstacles, iters)

        # Update visualization
        if not vis_callback.on_route_step(snapshot):
            return None  # User quit

        if snapshot.found:
            path = snapshot.path
            total_iterations = snapshot.iteration
            break

    if path is None:
        total_iterations = router.step(obstacles, 0).iteration
        print(f"No route found after {total_iterations} iterations ({first_label}), trying {second_label}...")

        # Try second direction
        router = VisualRouter(via_cost=config.via_cost * 1000, h_weight=config.heuristic_weight,
                          turn_cost=config.turn_cost, via_proximity_cost=int(config.via_proximity_cost),
                          vertical_attraction_radius=attraction_radius_grid,
                          vertical_attraction_bonus=attraction_bonus,
                          layer_costs=config.get_layer_costs(),
                          proximity_heuristic_cost=prox_h_cost)
        router.init(second_sources, second_targets, config.max_iterations)
        direction_used = second_label

        while not router.is_done():
            while vis_callback.should_pause():
                snapshot = router.step(obstacles, 0)
                if not vis_callback.on_route_step(snapshot):
                    return None

            iters = vis_callback.get_iterations_per_step()
            snapshot = router.step(obstacles, iters)

            if not vis_callback.on_route_step(snapshot):
                return None

            if snapshot.found:
                path = snapshot.path
                total_iterations += snapshot.iteration
                break

        if path is None:
            total_iterations += router.step(obstacles, 0).iteration

    if path is None:
        print(f"No route found after {total_iterations} iterations (both directions)")
        return {'failed': True, 'iterations': total_iterations, 'direction': 'both'}

    print(f"Route found in {total_iterations} iterations ({direction_used}), path length: {len(path)}")

    # Print debug stats if verbose
    if config.verbose:
        stats = router.get_stats()
        print(f"    VisualRouter stats: expanded={stats.get('cells_expanded', 0)}, pushed={stats.get('cells_pushed', 0)}, duplicates={stats.get('duplicate_skips', 0)}, closed={stats.get('closed_size', 0)}")

    # Swap sources/targets if we used second direction
    reversed_path = (direction_used == second_label)
    if reversed_path:
        sources, targets = targets, sources

    path_start = path[0]
    path_end = path[-1]

    start_original = None
    for s in sources:
        if s[0] == path_start[0] and s[1] == path_start[1] and s[2] == path_start[2]:
            start_original = (s[3], s[4], layer_names[s[2]])
            break

    end_original = None
    for t in targets:
        if t[0] == path_end[0] and t[1] == path_end[1] and t[2] == path_end[2]:
            end_original = (t[3], t[4], layer_names[t[2]])
            break

    # Get through-hole pad positions for this net (layer transitions without via)
    through_hole_positions = get_same_net_through_hole_positions(pcb_data, net_id, config)

    # Simplify path by removing collinear intermediate points
    path = simplify_path(path)

    new_segments = []
    new_vias = []

    if start_original:
        first_grid_x, first_grid_y = coord.to_float(path_start[0], path_start[1])
        orig_x, orig_y, orig_layer = start_original
        if abs(orig_x - first_grid_x) > 0.001 or abs(orig_y - first_grid_y) > 0.001:
            seg = Segment(
                start_x=orig_x, start_y=orig_y,
                end_x=first_grid_x, end_y=first_grid_y,
                width=config.get_net_track_width(net_id, orig_layer),
                layer=orig_layer,
                net_id=net_id
            )
            new_segments.append(seg)

    for i in range(len(path) - 1):
        gx1, gy1, layer1 = path[i]
        gx2, gy2, layer2 = path[i + 1]

        x1, y1 = coord.to_float(gx1, gy1)
        x2, y2 = coord.to_float(gx2, gy2)

        if layer1 != layer2:
            # Check if layer change is at an existing through-hole pad
            # If so, skip creating a via - the pad provides the layer transition
            if (gx1, gy1) not in through_hole_positions:
                via = Via(
                    x=x1, y=y1,
                    size=config.via_size,
                    drill=config.via_drill,
                    layers=["F.Cu", "B.Cu"],  # Always through-hole
                    net_id=net_id
                )
                new_vias.append(via)
        else:
            if (x1, y1) != (x2, y2):
                layer_name = layer_names[layer1]
                seg = Segment(
                    start_x=x1, start_y=y1,
                    end_x=x2, end_y=y2,
                    width=config.get_net_track_width(net_id, layer_name),
                    layer=layer_name,
                    net_id=net_id
                )
                new_segments.append(seg)

    if end_original:
        last_grid_x, last_grid_y = coord.to_float(path_end[0], path_end[1])
        orig_x, orig_y, orig_layer = end_original
        if abs(orig_x - last_grid_x) > 0.001 or abs(orig_y - last_grid_y) > 0.001:
            seg = Segment(
                start_x=last_grid_x, start_y=last_grid_y,
                end_x=orig_x, end_y=orig_y,
                width=config.get_net_track_width(net_id, orig_layer),
                layer=orig_layer,
                net_id=net_id
            )
            new_segments.append(seg)

    return {
        'new_segments': new_segments,
        'new_vias': new_vias,
        'iterations': total_iterations,
        'path_length': len(path),
        'path': path,
        'direction': direction_used,
    }


def route_multipoint_main(
    pcb_data: PCBData,
    net_id: int,
    config: GridRouteConfig,
    obstacles: 'GridObstacleMap',
    pad_info: List[Tuple]
) -> Optional[dict]:
    """
    Route only the main (longest MST segment) connection of a multi-point net.

    This is Phase 1 of multi-point routing. It computes an MST between all pads
    and routes the longest segment first, creating a clean 2-point route
    suitable for length matching.

    After length matching is applied, call route_multipoint_taps() to
    complete the remaining connections using the remaining MST edges.

    Args:
        pcb_data: PCB data
        net_id: Net ID to route
        config: Grid routing configuration
        obstacles: Pre-built obstacle map
        pad_info: List of (gx, gy, layer_idx, orig_x, orig_y, pad) from get_multipoint_net_pads()

    Returns:
        Routing result dict with:
        - 'new_segments', 'new_vias', 'iterations', 'path_length', 'path'
        - 'is_multipoint': True (flag for Phase 3)
        - 'multipoint_pad_info': Full pad_info list for Phase 3
        - 'routed_pad_indices': Set of indices already routed (the longest MST edge)
        - 'mst_edges': List of (idx_a, idx_b, length) for all MST edges
        Or {'failed': True, 'iterations': N} on failure
    """
    if GridRouter is None:
        print("  GridRouter not available")
        return None

    if len(pad_info) < 3:
        print(f"  Multi-point routing requires 3+ pads, got {len(pad_info)}")
        return None

    coord = GridCoord(config.grid_step)
    layer_names = config.layers

    # Extract pad positions for MST computation
    pad_positions = [(info[3], info[4]) for info in pad_info]  # (orig_x, orig_y)

    # Compute MST with Manhattan distance (better for PCB routing)
    mst_edges = compute_mst_edges(pad_positions, use_manhattan=True)

    # Filter out MST edges between pads already connected through zones/planes
    # Also track pad_components for Phase 3 to know which pads are zone-connected
    pad_components = {i: i for i in range(len(pad_info))}  # Default: each pad is its own component
    net_zones = [z for z in pcb_data.zones if z.net_id == net_id]
    if net_zones:
        net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
        net_vias = [v for v in pcb_data.vias if v.net_id == net_id]
        pads_list = [info[5] for info in pad_info if len(info) > 5]
        if pads_list:
            pad_components = get_zone_connected_pad_groups(
                net_segments, net_vias, pads_list, net_zones, config.layers
            )
            # Group pads by component for quick lookup
            component_pads: dict = {}  # component_id -> list of pad indices
            for pad_idx, comp_id in pad_components.items():
                if comp_id not in component_pads:
                    component_pads[comp_id] = []
                component_pads[comp_id].append(pad_idx)

            # Filter edges and optimize: for edges crossing components, find closest pad in zone-connected group
            original_count = len(mst_edges)
            optimized_edges = []
            seen_component_pairs = set()  # Track which component pairs we've already connected

            for a, b, dist in mst_edges:
                comp_a = pad_components.get(a)
                comp_b = pad_components.get(b)
                if comp_a == comp_b:
                    continue  # Skip intra-component edges

                # Normalize component pair to avoid duplicates
                pair_key = (min(comp_a, comp_b), max(comp_a, comp_b))
                if pair_key in seen_component_pairs:
                    continue  # Already have an edge between these components
                seen_component_pairs.add(pair_key)

                # Optimize: find shortest edge between pads in these two components
                best_edge = None
                best_dist = float('inf')
                for pa in component_pads.get(comp_a, [a]):
                    for pb in component_pads.get(comp_b, [b]):
                        px_a, py_a = pad_positions[pa]
                        px_b, py_b = pad_positions[pb]
                        d = abs(px_a - px_b) + abs(py_a - py_b)  # Manhattan distance
                        if d < best_dist:
                            best_dist = d
                            best_edge = (pa, pb, d)
                if best_edge:
                    optimized_edges.append(best_edge)

            mst_edges = optimized_edges
            if original_count > len(mst_edges):
                skipped = original_count - len(mst_edges)
                print(f"  Skipping {skipped} MST edge(s) already connected through plane")

    if not mst_edges:
        print(f"  All pads already connected through plane - nothing to route")
        return None

    # Sort MST edges by length (longest first)
    mst_edges = sorted(mst_edges, key=lambda e: -e[2])

    # Route the longest MST edge first
    idx_a, idx_b, longest_len = mst_edges[0]

    print(f"  Multi-point net Phase 1: routing longest MST edge (pads {idx_a} and {idx_b}, length={longest_len:.2f}mm)")

    # Build source/target for farthest pair
    pad_a = pad_info[idx_a]
    pad_b = pad_info[idx_b]

    # For through-hole pads, create sources/targets on ALL layers (router can reach any layer)
    # This avoids unnecessary vias when connecting to through-hole pads
    pad_a_obj = pad_a[5] if len(pad_a) > 5 else None
    pad_b_obj = pad_b[5] if len(pad_b) > 5 else None

    if pad_a_obj and hasattr(pad_a_obj, 'layers') and '*.Cu' in pad_a_obj.layers:
        # Through-hole pad - can connect on any copper layer
        sources = [(pad_a[0], pad_a[1], layer_idx) for layer_idx in range(len(layer_names))]
    else:
        sources = [(pad_a[0], pad_a[1], pad_a[2])]  # (gx, gy, layer_idx)

    if pad_b_obj and hasattr(pad_b_obj, 'layers') and '*.Cu' in pad_b_obj.layers:
        # Through-hole pad - can connect on any copper layer
        targets = [(pad_b[0], pad_b[1], layer_idx) for layer_idx in range(len(layer_names))]
    else:
        targets = [(pad_b[0], pad_b[1], pad_b[2])]

    # Mark source/target cells
    for gx, gy, layer in sources + targets:
        obstacles.add_source_target_cell(gx, gy, layer)

    # Get stub free ends for proximity zone checking (consistent with route_net)
    free_end_sources, free_end_targets, _ = get_net_endpoints(pcb_data, net_id, config, use_stub_free_ends=True)
    if free_end_sources:
        prox_check_sources = [(s[0], s[1], s[2]) for s in free_end_sources]
    else:
        prox_check_sources = sources  # Fallback to pad positions
    if free_end_targets:
        prox_check_targets = [(t[0], t[1], t[2]) for t in free_end_targets]
    else:
        prox_check_targets = targets  # Fallback to pad positions

    # Calculate vertical attraction parameters
    attraction_radius_grid = coord.to_grid_dist(config.vertical_attraction_radius) if config.vertical_attraction_radius > 0 else 0
    attraction_bonus = int(config.vertical_attraction_cost * 1000 / config.grid_step) if config.vertical_attraction_cost > 0 else 0

    # Check which proximity zones the stub free ends are in for precise heuristic estimate
    src_in_stub = any(obstacles.get_stub_proximity_cost(gx, gy) > 0 for gx, gy, _ in prox_check_sources)
    src_in_bga = any(obstacles.is_in_bga_proximity(gx, gy) for gx, gy, _ in prox_check_sources)
    tgt_in_stub = any(obstacles.get_stub_proximity_cost(gx, gy) > 0 for gx, gy, _ in prox_check_targets)
    tgt_in_bga = any(obstacles.is_in_bga_proximity(gx, gy) for gx, gy, _ in prox_check_targets)
    prox_h_cost = config.get_proximity_heuristic_for_zones(src_in_stub, src_in_bga, tgt_in_stub, tgt_in_bga)
    if config.verbose:
        zones = []
        if src_in_stub: zones.append("src:stub")
        if src_in_bga: zones.append("src:bga")
        if tgt_in_stub: zones.append("tgt:stub")
        if tgt_in_bga: zones.append("tgt:bga")
        print(f"  proximity_heuristic_cost={prox_h_cost} zones=[{', '.join(zones) if zones else 'none'}]")

    # Route farthest pair with probe routing (same as single-ended)
    router = GridRouter(via_cost=config.via_cost * 1000, h_weight=config.heuristic_weight,
                        turn_cost=config.turn_cost, via_proximity_cost=int(config.via_proximity_cost),
                        vertical_attraction_radius=attraction_radius_grid,
                        vertical_attraction_bonus=attraction_bonus,
                        layer_costs=config.get_layer_costs(),
                        proximity_heuristic_cost=prox_h_cost,
                        layer_direction_preferences=config.get_layer_direction_preferences(),
                        direction_preference_cost=config.direction_preference_cost)

    # Calculate track margin for wide power tracks
    # Use ceiling + 1 to account for grid quantization and diagonal track approaches
    # Compare against layer-specific width (not base track_width) to handle impedance routing
    net_track_width = config.get_net_track_width(net_id, config.layers[0])
    layer_track_width = config.get_track_width(config.layers[0])
    extra_half_width = (net_track_width - layer_track_width) / 2
    track_margin = (int(math.ceil(extra_half_width / config.grid_step)) + 1) if extra_half_width > 0 else 0

    # Use probe routing helper
    path, total_iterations, forward_blocked, backward_blocked, reversed_path, fwd_iters, bwd_iters = _probe_route_with_frontier(
        router, obstacles, sources, targets, config,
        print_prefix="  ", direction_labels=("forward", "backward"), track_margin=track_margin,
        pcb_data=pcb_data, current_net_id=net_id
    )

    if path is None:
        print(f"  Failed to route farthest pair after {total_iterations} iterations")
        return {
            'failed': True,
            'iterations': total_iterations,
            'blocked_cells_forward': forward_blocked,
            'blocked_cells_backward': backward_blocked,
            'iterations_forward': fwd_iters,
            'iterations_backward': bwd_iters,
        }

    # If path was found in reverse direction, swap pad_a/pad_b for segment generation
    if reversed_path:
        pad_a, pad_b = pad_b, pad_a
        idx_a, idx_b = idx_b, idx_a

    # Get through-hole pad positions for this net (layer transitions without via)
    through_hole_positions = get_same_net_through_hole_positions(pcb_data, net_id, config)

    # Convert path to segments/vias
    segments, vias = _path_to_segments_vias(
        path, coord, layer_names, net_id, config,
        (pad_a[3], pad_a[4], layer_names[pad_a[2]]),  # start_original
        (pad_b[3], pad_b[4], layer_names[pad_b[2]]),  # end_original
        through_hole_positions
    )

    print(f"  Phase 1 routed in {total_iterations} iterations, {len(segments)} segments")

    return {
        'new_segments': segments,
        'new_vias': vias,
        'iterations': total_iterations,
        'path_length': len(path),
        'path': path,
        'is_multipoint': True,
        'multipoint_pad_info': pad_info,
        'routed_pad_indices': {idx_a, idx_b},
        'pad_components': pad_components,  # Zone-connected component for each pad
        # Store main pad positions for Phase 3 tap filtering
        'main_pad_a': (pad_a[3], pad_a[4]),  # (orig_x, orig_y) of first main pad
        'main_pad_b': (pad_b[3], pad_b[4]),  # (orig_x, orig_y) of second main pad
        # Store original segments for identifying meanders in Phase 3
        'original_segments': segments,
        # Store MST edges for Phase 3 (sorted longest first)
        'mst_edges': mst_edges,
        # Initial tap stats (Phase 1 connects 2 pads via 1 edge)
        'tap_edges_routed': 1,
        'tap_edges_failed': 0,
        'tap_pads_connected': 2,
        'tap_pads_total': len(pad_info),
    }


def get_all_segment_tap_points(
    segments: List[Segment],
    coord: GridCoord,
    layer_names: List[str],
    vias: List = None
) -> List[Tuple[int, int, int, float, float]]:
    """
    Get all grid points along existing segments and vias as potential tap sources.

    Returns list of (gx, gy, layer_idx, orig_x, orig_y) for each point.
    Points are sampled at grid resolution along each segment.
    Vias are added on ALL layers (they connect all copper layers).
    Sorted by grid coordinates for deterministic iteration.
    """
    # Use dict keyed by (gx, gy, layer_idx) to deduplicate while keeping original coords
    tap_points = {}  # (gx, gy, layer_idx) -> (orig_x, orig_y)
    layer_map = build_layer_map(layer_names)

    for seg in segments:
        layer_idx = layer_map.get(seg.layer, 0)

        # Sample points along the segment at grid resolution
        dx = seg.end_x - seg.start_x
        dy = seg.end_y - seg.start_y
        length = (dx*dx + dy*dy) ** 0.5

        if length < 0.001:
            # Point segment
            gx, gy = coord.to_grid(seg.start_x, seg.start_y)
            key = (gx, gy, layer_idx)
            if key not in tap_points:
                tap_points[key] = (seg.start_x, seg.start_y)
        else:
            # Sample along segment at grid step intervals
            num_steps = max(1, int(length / coord.grid_step))
            for i in range(num_steps + 1):
                t = i / num_steps
                x = seg.start_x + t * dx
                y = seg.start_y + t * dy
                gx, gy = coord.to_grid(x, y)
                key = (gx, gy, layer_idx)
                if key not in tap_points:
                    tap_points[key] = (x, y)

    # Add vias on ALL layers (vias connect all copper layers)
    if vias:
        for via in vias:
            gx, gy = coord.to_grid(via.x, via.y)
            for layer_idx in range(len(layer_names)):
                key = (gx, gy, layer_idx)
                if key not in tap_points:
                    tap_points[key] = (via.x, via.y)

    # Return sorted list for deterministic iteration
    return sorted([(gx, gy, layer_idx, ox, oy)
                   for (gx, gy, layer_idx), (ox, oy) in tap_points.items()])


def route_multipoint_taps(
    pcb_data: PCBData,
    net_id: int,
    config: GridRouteConfig,
    obstacles: 'GridObstacleMap',
    main_result: dict,
    global_offset: int = 0,
    global_total: int = 0,
    global_failed: int = 0
) -> Optional[dict]:
    """
    Route the remaining MST edges for a multi-point net.

    This is Phase 3 of multi-point routing - called AFTER length matching
    has been applied to the main route. It routes the remaining MST edges
    in order of length (longest first), connecting unrouted pads to the
    growing routed network.

    Args:
        pcb_data: PCB data
        net_id: Net ID to route
        config: Grid routing configuration
        obstacles: Pre-built obstacle map (should include length-matched segments)
        main_result: Result from route_multipoint_main() with meanders applied

    Returns:
        Updated result dict with tap segments/vias added, or None on failure
    """
    if GridRouter is None:
        print("  GridRouter not available")
        return None

    pad_info = main_result['multipoint_pad_info']
    routed_indices = set(main_result['routed_pad_indices'])
    mst_edges = main_result.get('mst_edges', [])
    pad_components = main_result.get('pad_components', {i: i for i in range(len(pad_info))})

    # Build set of "routed components" - components with at least one explicitly routed pad
    # Pads in zone-connected components are effectively routed if any pad in that component is routed
    routed_components = {pad_components.get(idx, idx) for idx in routed_indices}

    # Get the current segments (which may have meanders from length matching)
    all_segments = list(main_result['new_segments'])
    all_vias = list(main_result.get('new_vias', []))

    coord = GridCoord(config.grid_step)
    layer_names = config.layers

    # Get through-hole pad positions for this net (layer transitions without via)
    through_hole_positions = get_same_net_through_hole_positions(pcb_data, net_id, config)

    # Get remaining MST edges (skip the first one which was routed in Phase 1)
    # MST edges are already sorted longest-first
    remaining_edges = mst_edges[1:] if len(mst_edges) > 1 else []

    if not remaining_edges:
        print(f"  No remaining MST edges to route in Phase 3")
        return main_result

    print(f"  Multi-point net Phase 3: routing {len(remaining_edges)} remaining MST edges (longest first)")

    # Calculate vertical attraction parameters
    attraction_radius_grid = coord.to_grid_dist(config.vertical_attraction_radius) if config.vertical_attraction_radius > 0 else 0
    attraction_bonus = int(config.vertical_attraction_cost * 1000 / config.grid_step) if config.vertical_attraction_cost > 0 else 0

    router = GridRouter(via_cost=config.via_cost * 1000, h_weight=config.heuristic_weight,
                        turn_cost=config.turn_cost, via_proximity_cost=int(config.via_proximity_cost),
                        vertical_attraction_radius=attraction_radius_grid,
                        vertical_attraction_bonus=attraction_bonus,
                        layer_costs=config.get_layer_costs(),
                        proximity_heuristic_cost=0,  # Set per-route below
                        layer_direction_preferences=config.get_layer_direction_preferences(),
                        direction_preference_cost=config.direction_preference_cost)

    # Calculate track margin for wide power tracks
    # Use ceiling + 1 to account for grid quantization and diagonal track approaches
    # Compare against layer-specific width (not base track_width) to handle impedance routing
    net_track_width = config.get_net_track_width(net_id, config.layers[0])
    layer_track_width = config.get_track_width(config.layers[0])
    extra_half_width = (net_track_width - layer_track_width) / 2
    track_margin = (int(math.ceil(extra_half_width / config.grid_step)) + 1) if extra_half_width > 0 else 0

    total_iterations = 0

    # Route remaining MST edges in order (longest first)
    # Each edge connects a routed pad to an unrouted pad
    edges_routed = 0
    failed_edges = set()  # Track edges that failed to route
    failed_edge_blocking = {}  # Track blocking info for failed edges: edge_key -> (blocked_cells, tgt_xy)
    max_passes = len(remaining_edges) * 2  # Safety limit

    for pass_num in range(max_passes):
        if len(routed_indices) == len(pad_info):
            break  # All pads connected

        # Find an edge that connects routed to unrouted (skip failed edges)
        edge_to_route = None
        for edge in remaining_edges:
            idx_a, idx_b, length = edge
            edge_key = (min(idx_a, idx_b), max(idx_a, idx_b))
            if edge_key in failed_edges:
                continue

            # Check if pad is effectively routed (either explicitly or via zone-connected component)
            a_component = pad_components.get(idx_a, idx_a)
            b_component = pad_components.get(idx_b, idx_b)
            a_routed = idx_a in routed_indices or a_component in routed_components
            b_routed = idx_b in routed_indices or b_component in routed_components

            if a_routed and not b_routed:
                edge_to_route = (idx_a, idx_b, length)  # Route from a to b
                break
            elif b_routed and not a_routed:
                edge_to_route = (idx_b, idx_a, length)  # Route from b to a
                break

        if edge_to_route is None:
            # Count effectively unrouted pads (not in routed_indices AND not in a routed component)
            unrouted_pads = sum(1 for i in range(len(pad_info))
                               if i not in routed_indices and pad_components.get(i, i) not in routed_components)
            if unrouted_pads > 0:
                print(f"  {YELLOW}Warning: {unrouted_pads} pad(s) not connected ({len(failed_edges)} MST edge(s) failed){RESET}")
            break

        src_idx, tgt_idx, edge_len = edge_to_route

        src_pad = pad_info[src_idx]
        tgt_pad = pad_info[tgt_idx]

        # Show progress: [current/total] with failure count (global across all nets)
        current_global = global_offset + edges_routed + len(failed_edges) + 1
        total_failed = global_failed + len(failed_edges)
        fail_str = f" ({total_failed} failed)" if total_failed > 0 else ""
        print(f"    [{current_global}/{global_total}]{fail_str} Routing MST edge: pad {src_idx} -> pad {tgt_idx} (length={edge_len:.2f}mm) target=({tgt_pad[3]:.2f}, {tgt_pad[4]:.2f})")

        # Get target pad coordinates
        tgt_x, tgt_y = tgt_pad[3], tgt_pad[4]

        # Get ALL points along existing segments and vias as potential tap sources
        # The router will find the shortest path from ANY of these points
        # Vias are included on ALL layers since they connect all copper layers
        all_tap_points = get_all_segment_tap_points(all_segments, coord, layer_names, vias=all_vias)

        # Always include the designated source pad position as a potential source
        # This is critical for zone-connected pads that have no segments to them yet
        src_x, src_y = src_pad[3], src_pad[4]
        src_gx, src_gy = coord.to_grid(src_x, src_y)
        src_pad_obj = src_pad[5]

        # Build initial tap point map from segment/via tap points
        if all_tap_points:
            sources = [(gx, gy, layer_idx) for gx, gy, layer_idx, _, _ in all_tap_points]
            tap_point_map = {(gx, gy, layer_idx): (ox, oy, layer_names[layer_idx])
                            for gx, gy, layer_idx, ox, oy in all_tap_points}
        else:
            sources = []
            tap_point_map = {}

        # Add source pad position as a valid source (on all layers for through-hole)
        if hasattr(src_pad_obj, 'layers') and '*.Cu' in src_pad_obj.layers:
            # Through-hole pad - can connect on any copper layer
            for layer_idx in range(len(layer_names)):
                key = (src_gx, src_gy, layer_idx)
                if key not in tap_point_map:
                    sources.append(key)
                    tap_point_map[key] = (src_x, src_y, layer_names[layer_idx])
        else:
            # SMD pad - use specific layer from pad_info
            key = (src_gx, src_gy, src_pad[2])
            if key not in tap_point_map:
                sources.append(key)
                tap_point_map[key] = (src_x, src_y, layer_names[src_pad[2]])

        if not sources:
            print(f"      ERROR: No sources available for routing")
            continue

        # For through-hole pads, create targets on ALL layers (router can reach any layer)
        tgt_gx, tgt_gy = tgt_pad[0], tgt_pad[1]
        tgt_pad_obj = tgt_pad[5]
        if hasattr(tgt_pad_obj, 'layers') and '*.Cu' in tgt_pad_obj.layers:
            # Through-hole pad - can connect on any copper layer
            targets = [(tgt_gx, tgt_gy, layer_idx) for layer_idx in range(len(layer_names))]
        else:
            # SMD pad or specific layer - use the layer from pad_info
            targets = [(tgt_gx, tgt_gy, tgt_pad[2])]

        # Mark source/target cells
        for gx, gy, layer in sources + targets:
            obstacles.add_source_target_cell(gx, gy, layer)

        # Add allowed cells around target to escape blocked areas
        allow_radius = 5
        tgt_gx, tgt_gy = tgt_pad[0], tgt_pad[1]
        for dx in range(-allow_radius, allow_radius + 1):
            for dy in range(-allow_radius, allow_radius + 1):
                obstacles.add_allowed_cell(tgt_gx + dx, tgt_gy + dy)

        # Check which proximity zones the endpoints are in for precise heuristic estimate
        src_in_stub = any(obstacles.get_stub_proximity_cost(gx, gy) > 0 for gx, gy, _ in sources)
        src_in_bga = any(obstacles.is_in_bga_proximity(gx, gy) for gx, gy, _ in sources)
        tgt_in_stub = any(obstacles.get_stub_proximity_cost(gx, gy) > 0 for gx, gy, _ in targets)
        tgt_in_bga = any(obstacles.is_in_bga_proximity(gx, gy) for gx, gy, _ in targets)
        prox_h_cost = config.get_proximity_heuristic_for_zones(src_in_stub, src_in_bga, tgt_in_stub, tgt_in_bga)
        router.set_proximity_heuristic_cost(prox_h_cost)
        if config.verbose:
            zones = []
            if src_in_stub: zones.append("src:stub")
            if src_in_bga: zones.append("src:bga")
            if tgt_in_stub: zones.append("tgt:stub")
            if tgt_in_bga: zones.append("tgt:bga")
            print(f"      proximity_heuristic_cost={prox_h_cost} zones=[{', '.join(zones) if zones else 'none'}]")

        # Route from ANY tap point to target - router finds shortest path
        # Use probe routing helper to detect stuck directions early
        tap_start_time = time.time()

        path, tap_iterations, forward_blocked, backward_blocked, reversed_tap_path, _, _ = _probe_route_with_frontier(
            router, obstacles, sources, targets, config,
            print_prefix="      ", direction_labels=("forward", "backward"), track_margin=track_margin,
            pcb_data=pcb_data, current_net_id=net_id
        )

        # If path was found in reverse direction, reverse it so it goes sources -> targets
        if path is not None and reversed_tap_path:
            path = list(reversed(path))

        # Combine blocked cells from both directions for rip-up analysis
        blocked_cells = forward_blocked + backward_blocked

        tap_elapsed = time.time() - tap_start_time
        total_iterations += tap_iterations

        if path is None:
            print(f"      {YELLOW}Failed to route MST edge after {tap_iterations} iterations ({tap_elapsed:.2f}s){RESET}")
            edge_key = (min(src_idx, tgt_idx), max(src_idx, tgt_idx))
            failed_edges.add(edge_key)
            # Store blocking info for potential rip-up analysis
            if blocked_cells:
                failed_edge_blocking[edge_key] = (blocked_cells, (tgt_x, tgt_y))
            continue

        print(f"      Routed in {tap_iterations} iterations ({tap_elapsed:.2f}s)")

        # Get the actual tap point used (first point of path)
        path_start = path[0]  # (gx, gy, layer_idx)
        if path_start in tap_point_map:
            tap_x, tap_y, tap_layer = tap_point_map[path_start]
        else:
            # Fallback: convert grid coords back to original
            tap_x, tap_y = coord.from_grid(path_start[0], path_start[1])
            tap_layer = layer_names[path_start[2]]

        # Convert path to segments/vias
        # Use the actual end layer from the path (router may reach through-hole pad on any layer)
        path_end_layer = layer_names[path[-1][2]]
        segments, vias = _path_to_segments_vias(
            path, coord, layer_names, net_id, config,
            (tap_x, tap_y, tap_layer),  # start_original (actual tap point used)
            (tgt_x, tgt_y, path_end_layer),  # end_original (target pad on actual reached layer)
            through_hole_positions
        )
        all_segments.extend(segments)
        all_vias.extend(vias)

        # Note: We don't add segments as obstacles since they're the same net
        # and future tap routes can overlap with our own traces

        # Mark target pad as routed and its component as routed
        routed_indices.add(tgt_idx)
        tgt_component = pad_components.get(tgt_idx, tgt_idx)
        routed_components.add(tgt_component)
        remaining_edges = [e for e in remaining_edges if not (
            (e[0] == src_idx and e[1] == tgt_idx) or (e[0] == tgt_idx and e[1] == src_idx)
        )]
        edges_routed += 1

    # Count pads that are effectively connected (either explicitly routed or zone-connected to a routed pad)
    pads_connected = sum(1 for i in range(len(pad_info))
                         if i in routed_indices or pad_components.get(i, i) in routed_components)
    pads_total = len(pad_info)
    pads_failed = pads_total - pads_connected

    # Collect detailed info about failed (unconnected) pads
    failed_pads_info = []
    for i in range(len(pad_info)):
        if i not in routed_indices and pad_components.get(i, i) not in routed_components:
            pad = pad_info[i]
            pad_obj = pad[5] if len(pad) > 5 else None
            failed_pads_info.append({
                'pad_idx': i,
                'x': pad[3],  # orig_x
                'y': pad[4],  # orig_y
                'component_ref': getattr(pad_obj, 'component_ref', '?') if pad_obj else '?',
                'pad_number': getattr(pad_obj, 'pad_number', '?') if pad_obj else '?',
            })

    print(f"  Phase 3 routing complete: {edges_routed} edges, {len(all_segments)} total segments, {len(all_vias)} total vias")

    # Update result - preserve original fields, update segments/vias
    updated_result = dict(main_result)
    updated_result['new_segments'] = all_segments
    updated_result['new_vias'] = all_vias
    updated_result['iterations'] = main_result['iterations'] + total_iterations
    updated_result['routed_pad_indices'] = routed_indices
    # Tap routing stats (add Phase 3 to Phase 1 counts)
    updated_result['tap_edges_routed'] = main_result.get('tap_edges_routed', 0) + edges_routed
    updated_result['tap_edges_failed'] = main_result.get('tap_edges_failed', 0) + len(failed_edges)
    updated_result['tap_pads_connected'] = pads_connected
    updated_result['tap_pads_total'] = pads_total
    # Detailed info about unconnected pads (for summary)
    updated_result['failed_pads_info'] = failed_pads_info
    # Blocking info for failed edges (for rip-up analysis)
    updated_result['failed_edge_blocking'] = failed_edge_blocking

    return updated_result


def _path_to_segments_vias(
    path: List[Tuple[int, int, int]],
    coord: GridCoord,
    layer_names: List[str],
    net_id: int,
    config: GridRouteConfig,
    start_original: Tuple[float, float, str],
    end_original: Tuple[float, float, str],
    through_hole_positions: Set[Tuple[int, int]] = None
) -> Tuple[List[Segment], List[Via]]:
    """
    Convert a grid path to Segment and Via objects.

    Args:
        path: List of (gx, gy, layer_idx) grid points
        coord: Grid coordinate converter
        layer_names: List of layer names
        net_id: Net ID for segments/vias
        config: Routing config with track width, via size
        start_original: (x, y, layer) of path start in float coords
        end_original: (x, y, layer) of path end in float coords
        through_hole_positions: Optional set of (gx, gy) positions where through-hole
            pads exist on this net. Layer changes at these positions don't need a
            new via since the existing through-hole provides the layer transition.

    Returns:
        (segments, vias): Lists of Segment and Via objects
    """
    segments = []
    vias = []

    if not path:
        return segments, vias

    # Simplify path by removing collinear intermediate points
    path = simplify_path(path)

    path_start = path[0]
    path_end = path[-1]

    # Add connecting segment from original start to first path point if needed
    if start_original:
        first_grid_x, first_grid_y = coord.to_float(path_start[0], path_start[1])
        orig_x, orig_y, orig_layer = start_original
        # Use the actual path layer, not the original pad layer
        # (through-hole pads may have orig_layer=F.Cu but router chose In1.Cu)
        path_start_layer = layer_names[path_start[2]]
        if abs(orig_x - first_grid_x) > 0.001 or abs(orig_y - first_grid_y) > 0.001:
            seg = Segment(
                start_x=orig_x, start_y=orig_y,
                end_x=first_grid_x, end_y=first_grid_y,
                width=config.get_net_track_width(net_id, path_start_layer),
                layer=path_start_layer,
                net_id=net_id
            )
            segments.append(seg)

    # Convert path points to segments and vias
    for i in range(len(path) - 1):
        gx1, gy1, layer1 = path[i]
        gx2, gy2, layer2 = path[i + 1]

        x1, y1 = coord.to_float(gx1, gy1)
        x2, y2 = coord.to_float(gx2, gy2)

        if layer1 != layer2:
            # Check if layer change is at an existing through-hole pad
            # If so, skip creating a via - the pad provides the layer transition
            if through_hole_positions and (gx1, gy1) in through_hole_positions:
                # No via needed - existing through-hole pad connects all layers
                pass
            else:
                via = Via(
                    x=x1, y=y1,
                    size=config.via_size,
                    drill=config.via_drill,
                    layers=["F.Cu", "B.Cu"],  # Always through-hole
                    net_id=net_id
                )
                vias.append(via)
        else:
            if (x1, y1) != (x2, y2):
                layer_name = layer_names[layer1]
                seg = Segment(
                    start_x=x1, start_y=y1,
                    end_x=x2, end_y=y2,
                    width=config.get_net_track_width(net_id, layer_name),
                    layer=layer_name,
                    net_id=net_id
                )
                segments.append(seg)

    # Add connecting segment from last path point to original end if needed
    if end_original:
        last_grid_x, last_grid_y = coord.to_float(path_end[0], path_end[1])
        orig_x, orig_y, orig_layer = end_original
        # Use the actual path layer, not the original pad layer
        path_end_layer = layer_names[path_end[2]]
        if abs(orig_x - last_grid_x) > 0.001 or abs(orig_y - last_grid_y) > 0.001:
            seg = Segment(
                start_x=last_grid_x, start_y=last_grid_y,
                end_x=orig_x, end_y=orig_y,
                width=config.get_net_track_width(net_id, path_end_layer),
                layer=path_end_layer,
                net_id=net_id
            )
            segments.append(seg)

    return segments, vias

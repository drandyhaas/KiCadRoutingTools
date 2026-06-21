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
from routing_utils import build_layer_map, pad_rect_halfspan
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


def _pt_foreign_pad_dist(pcb_data, net_id, x, y, layer):
    """Min edge distance from point (x,y) on `layer` to any pad of a DIFFERENT
    net. The pad is treated as its full board-axis rect (rounded corners ignored
    -> a slight under-estimate, i.e. conservative)."""
    best = 1e9
    for nid, pads in pcb_data.pads_by_net.items():
        if nid == net_id:
            continue
        for pad in pads:
            if layer not in pad.layers and '*.Cu' not in pad.layers:
                continue
            dx = max(abs(x - pad.global_x) - pad.size_x / 2.0, 0.0)
            dy = max(abs(y - pad.global_y) - pad.size_y / 2.0, 0.0)
            d = math.hypot(dx, dy)
            if d < best:
                best = d
    return best


def _seg_foreign_pad_dist(pcb_data, net_id, x1, y1, x2, y2, layer):
    """Min foreign-pad edge distance sampled along a (short, terminal) segment."""
    n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.02) + 1)
    best = 1e9
    for i in range(n + 1):
        t = i / n
        d = _pt_foreign_pad_dist(pcb_data, net_id, x1 + (x2 - x1) * t, y1 + (y2 - y1) * t, layer)
        if d < best:
            best = d
    return best


def _merge_terminal_to_exact(path, term_idx, neighbor_idx, original, pts,
                             pcb_data, net_id, config, layer_names):
    """#4: route.py is on-grid, but a route's TERMINAL connects to an off-grid pad
    or fanout escape. When the terminal grid cell lands inside a foreign pad's
    clearance (a quantised stand-in for the real, off-grid endpoint) but the EXACT
    endpoint -- and the segment from the neighbour point to it -- clear that pad,
    replace pts[term_idx] with the exact endpoint so the terminal segment runs to
    the clean point and the grazing grid-cell vertex disappears. Returns True if
    merged (caller then skips the separate connection stub). Uses explicit path
    indices, not coordinate matching, so float noise can't pick the wrong segment."""
    if original is None or len(path) < 2:
        return False
    if path[term_idx][2] != path[neighbor_idx][2]:
        return False  # via at the very endpoint -> leave it on grid
    ox, oy, ol = original
    if layer_names[path[term_idx][2]] != ol:
        return False
    fx, fy = pts[term_idx]
    if abs(ox - fx) < 1e-9 and abs(oy - fy) < 1e-9:
        return False  # exact endpoint already is the grid cell
    margin = config.clearance + config.get_net_track_width(net_id, ol) / 2.0
    if _pt_foreign_pad_dist(pcb_data, net_id, fx, fy, ol) >= margin:
        return False  # grid cell already clear -> nothing to fix
    if _pt_foreign_pad_dist(pcb_data, net_id, ox, oy, ol) < margin:
        return False  # exact endpoint also too close (placement) -> can't fix here
    nx, ny = pts[neighbor_idx]
    if _seg_foreign_pad_dist(pcb_data, net_id, ox, oy, nx, ny, ol) < margin - 1e-6:
        return False  # merged terminal segment would graze -> keep grid + stub
    pts[term_idx] = (ox, oy)
    return True


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

            # Compute pad expansion (rotated-rect bbox so a tilted pad's blocked
            # cells are still attributed to it)
            if hasattr(pad, 'size_x'):
                pad_half_x, pad_half_y = pad_rect_halfspan(pad)
            else:
                pad_half_x = pad_half_y = 0.5
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
    current_net_id: int = -1,
    single_direction: bool = False
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
        single_direction: If True, only try forward direction (for bus routing)

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

    # For single_direction mode (bus routing), skip backward probe entirely
    if single_direction:
        first_reached_max = first_probe_iters >= probe_iterations
        if not first_reached_max:
            # Forward is stuck
            print(f"{print_prefix}{first_label} stuck ({first_probe_iters} < {probe_iterations}) [single-direction bus mode]")
            _diagnose_blocked_start(obstacles, forward_sources, first_label, print_prefix, track_margin,
                                    pcb_data=pcb_data, config=config, current_net_id=current_net_id)
            fwd_iters, bwd_iters = get_fwd_bwd_iters()
            return None, total_iterations, forward_blocked, backward_blocked, False, fwd_iters, bwd_iters

        # Forward probe reached max - do full search
        print(f"{print_prefix}Probe: {first_label}={first_probe_iters} iters [single-direction bus mode], trying full iterations...")
        path, full_iters, full_blocked = router.route_with_frontier(
            obstacles, forward_sources, forward_targets, config.max_iterations, track_margin=track_margin)
        first_total_iters += full_iters
        total_iterations += full_iters

        if path is not None:
            forward_blocked = []
        else:
            forward_blocked = full_blocked
        fwd_iters, bwd_iters = get_fwd_bwd_iters()
        return path, total_iterations, forward_blocked, backward_blocked, False, fwd_iters, bwd_iters

    # Probe backward direction (bidirectional mode)
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
    attraction_bonus = config.cell_cost(config.vertical_attraction_cost) if config.vertical_attraction_cost > 0 else 0

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

    router = GridRouter(via_cost=config.via_cost_units(), h_weight=config.heuristic_weight,
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

    # Per-point float positions. Connect the two TERMINAL segments to the EXACT
    # off-grid endpoint instead of its grid-cell stand-in when that cell grazes a
    # foreign pad but the exact endpoint clears it (#4 off-grid connection graze).
    pts = [coord.to_float(p[0], p[1]) for p in path]
    merge_start = _merge_terminal_to_exact(path, 0, 1, start_original, pts,
                                           pcb_data, net_id, config, layer_names)
    merge_end = _merge_terminal_to_exact(path, len(path) - 1, len(path) - 2, end_original, pts,
                                         pcb_data, net_id, config, layer_names)

    # Add connecting segment from original start to first path point if needed
    # (skipped when merged: the first path segment now ends at the exact point)
    if start_original and not merge_start:
        first_grid_x, first_grid_y = pts[0]
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

        x1, y1 = pts[i]
        x2, y2 = pts[i + 1]

        if layer1 != layer2:
            # Check if layer change is at an existing through-hole pad
            # If so, skip creating a via - the pad provides the layer transition
            if (gx1, gy1) not in through_hole_positions:
                vx, vy = coord.to_float(gx1, gy1)  # via stays on the grid cell
                via = Via(
                    x=vx, y=vy,
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
    # (skipped when merged: the last path segment now ends at the exact point)
    if end_original and not merge_end:
        last_grid_x, last_grid_y = pts[-1]
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
                              attraction_path: Optional[List[Tuple[int, int, int]]] = None,
                              reverse_direction: bool = False) -> Optional[dict]:
    """Route a single net using pre-built obstacles (for incremental routing).

    Args:
        pcb_data: PCB data
        net_id: Net ID to route
        config: Routing configuration
        obstacles: Pre-built obstacle map
        attraction_path: Optional path to attract to (for bus routing).
                        List of (gx, gy, layer) tuples from a previously routed neighbor.
        reverse_direction: If True, swap sources and targets (route from targets to sources).
                          Used for bus routing when the clique was formed by targets.
    """
    # Find endpoints (segments or pads)
    sources, targets, error = get_net_endpoints(pcb_data, net_id, config)
    if error:
        print(f"  {error}")
        return None

    if not sources or not targets:
        print(f"  No valid source/target endpoints found")
        return None

    # Swap source/target for bus routing from clustered targets
    if reverse_direction:
        sources, targets = targets, sources

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
    attraction_bonus = config.cell_cost(config.vertical_attraction_cost) if config.vertical_attraction_cost > 0 else 0

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
    bus_attraction_bonus = config.scaled_cell_units(config.bus_attraction_bonus) if config.bus_attraction_bonus > 0 else 0

    router = GridRouter(via_cost=config.via_cost_units(), h_weight=config.heuristic_weight,
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
        if config.verbose:
            layers_in_path = set(p[2] for p in attraction_path)
            print(f"    Bus attraction: {len(attraction_path)} path points, layers={layers_in_path}, radius={bus_attraction_radius_grid} grid, bonus={bus_attraction_bonus}")

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
    # For bus routing with reverse_direction, use single-direction mode to ensure
    # routes start from the clustered endpoints (where attraction can guide them)
    use_single_direction = reverse_direction
    if config.verbose:
        print(f"    GridRouter sources: {forward_sources[:3]}{'...' if len(forward_sources) > 3 else ''}")
        print(f"    GridRouter targets: {forward_targets[:3]}{'...' if len(forward_targets) > 3 else ''}")
        if use_single_direction:
            print(f"    Bus routing: single-direction mode (start from clustered endpoints)")
    path, total_iterations, forward_blocked, backward_blocked, reversed_path, fwd_iters, bwd_iters, necked_down = _route_main_connection(
        router, obstacles, config, forward_sources, forward_targets, track_margin,
        pcb_data, net_id, print_prefix="", direction_labels=direction_labels,
        single_direction=use_single_direction
    )

    # Adjust reversed_path based on start direction
    if start_backwards and path is not None:
        reversed_path = not reversed_path

    if path is None:
        dir_msg = "single direction" if use_single_direction else "both directions"
        print(f"No route found after {total_iterations} iterations ({dir_msg})")
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

    if necked_down:
        # Both endpoints are pads: neck the start side too
        new_segments = _apply_neckdown_widths(new_segments, config, net_id, obstacles,
                                              coord, layer_names, track_margin, neck_start=True)

    return {
        'new_segments': new_segments,
        'new_vias': new_vias,
        'iterations': total_iterations,
        'path_length': len(path),
        'path': path,
    }


# ---------------------------------------------------------------------------
# Guide corridor (waypoint) routing (issue #7)
# ---------------------------------------------------------------------------

def build_corridor_waypoints(pcb_data: PCBData, config: GridRouteConfig) -> List[Tuple[int, int]]:
    """Convert user-drawn guide polylines into ordered grid waypoint cells.

    By default the waypoints are just the endpoints of each drawn line segment
    (the polyline vertices); A* routes near-straight between them, hugging the
    drawn line. If guide_corridor_spacing > 0, long segments are subdivided so
    no two consecutive waypoints are farther apart than that spacing (useful to
    follow a curve more tightly). Returns [] when no guide paths are present.
    """
    if not getattr(config, 'guide_corridor_enabled', False) or not pcb_data.guide_paths:
        return []

    coord = GridCoord(config.grid_step)
    spacing_mm = getattr(config, 'guide_corridor_spacing', 0.0) or 0.0
    spacing = coord.to_grid_dist(spacing_mm) if spacing_mm > 0 else 0

    cells: List[Tuple[int, int]] = []
    for gp in pcb_data.guide_paths:
        pts = list(gp.points)
        if gp.is_closed and len(pts) >= 2:
            pts.append(pts[0])
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
            g1 = coord.to_grid(x1, y1)
            g2 = coord.to_grid(x2, y2)
            cells.append(g1)
            # Optionally subdivide a long segment into intermediate waypoints.
            if spacing > 0:
                seg_len = max(abs(g2[0] - g1[0]), abs(g2[1] - g1[1]))
                n = seg_len // spacing
                for k in range(1, int(n) + 1):
                    t = (k * spacing) / seg_len if seg_len else 0
                    if t >= 1.0:
                        break
                    cells.append((round(g1[0] + t * (g2[0] - g1[0])),
                                  round(g1[1] + t * (g2[1] - g1[1]))))
        cells.append(coord.to_grid(*pts[-1]))  # final vertex of this chain

    # Drop consecutive duplicates
    out: List[Tuple[int, int]] = []
    for c in cells:
        if not out or out[-1] != c:
            out.append(c)
    return out


def _cell_margin_clear(obstacles, x, y, layer, margin):
    """True if (x, y) and every cell within `margin` (Chebyshev) is unblocked on layer."""
    if margin <= 0:
        return not obstacles.is_blocked(x, y, layer)
    for ox in range(-margin, margin + 1):
        for oy in range(-margin, margin + 1):
            if obstacles.is_blocked(x + ox, y + oy, layer):
                return False
    return True


def _nearest_free_cell(obstacles, gx, gy, num_layers, max_radius=80, margin=0):
    """BFS for the nearest (gx, gy) unblocked on at least one layer.

    When margin > 0, the cell only qualifies if every cell within `margin`
    (Chebyshev) is also unblocked on that layer, so a track centered there
    clears nearby obstacles instead of clipping them (grid quantization).
    Falls back to margin=0 if no clearer cell is found, so it never fails to
    return when something is free. Returns (gx, gy, layer) or None.
    """
    from collections import deque

    def clear_on_layer(x, y, layer):
        return _cell_margin_clear(obstacles, x, y, layer, margin)

    q = deque([(gx, gy)])
    seen = {(gx, gy)}
    fallback = None  # nearest cell free at margin=0, used if no margin-clear cell exists
    while q:
        x, y = q.popleft()
        for layer in range(num_layers):
            if not obstacles.is_blocked(x, y, layer):
                if fallback is None:
                    fallback = (x, y, layer)
                if clear_on_layer(x, y, layer):
                    return (x, y, layer)
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)):
            nx, ny = x + dx, y + dy
            if (nx, ny) not in seen and abs(nx - gx) + abs(ny - gy) <= max_radius:
                seen.add((nx, ny))
                q.append((nx, ny))
    return fallback


def _route_leg(router, obstacles, config, sources, targets, track_margin, pcb_data, net_id):
    """Route one leg (sources -> targets). Returns (path, iterations).

    The path is normalized to run from a source to a target (the bidirectional
    probe may return it reversed), so legs chain correctly end-to-start.
    """
    path, iters, _fb, _bb, reversed_path, _fi, _bi = _probe_route_with_frontier(
        router, obstacles, sources, targets, config,
        print_prefix="      ", track_margin=track_margin,
        pcb_data=pcb_data, current_net_id=net_id)
    if path is not None and reversed_path:
        path = path[::-1]
    return path, iters


def _point_segment_dist2(px, py, ax, ay, bx, by):
    """Squared distance from point (px,py) to segment (ax,ay)-(bx,by)."""
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return (px - ax) ** 2 + (py - ay) ** 2
    t = ((px - ax) * dx + (py - ay) * dy) / float(dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * dx, ay + t * dy
    return (px - cx) ** 2 + (py - cy) ** 2


def assign_waypoints_to_mst_edges(waypoints, pad_grid, mst_edges):
    """Bucket corridor waypoints onto the MST edge each is nearest to (issue #7).

    Each MST segment then follows the contiguous run of waypoints "in its middle";
    a waypoint lands in exactly one bucket, so once a segment uses it the others
    need not care. Corridor order is preserved within each bucket.

    Args:
        waypoints: ordered list of (gx, gy) grid waypoints.
        pad_grid: list of (gx, gy) grid positions indexed by pad index.
        mst_edges: list of (idx_a, idx_b, dist) MST edges.

    Returns:
        dict mapping frozenset({idx_a, idx_b}) -> ordered list of (gx, gy).
    """
    buckets = {frozenset((ia, ib)): [] for ia, ib, _ in mst_edges}
    if not mst_edges:
        return buckets
    for (wx, wy) in waypoints:
        best_key, best_d = None, None
        for ia, ib, _ in mst_edges:
            ax, ay = pad_grid[ia]
            bx, by = pad_grid[ib]
            d = _point_segment_dist2(wx, wy, ax, ay, bx, by)
            if best_d is None or d < best_d:
                best_d, best_key = d, frozenset((ia, ib))
        buckets[best_key].append((wx, wy))
    return buckets


def _route_main_connection(router, obstacles, config, sources, targets, track_margin,
                           pcb_data, net_id, print_prefix="",
                           direction_labels=("forward", "backward"), single_direction=False,
                           waypoints=None):
    """Route sources->targets; wide routes that fail retry narrow (issue #72).

    Same return shape as _route_connection_at_margin plus a trailing
    necked_down flag: when a wide power route cannot fit, it is retried at
    the layer's default width and the caller applies neck-down widths to
    the resulting segments (_apply_neckdown_widths).
    """
    result = _route_connection_at_margin(
        router, obstacles, config, sources, targets, track_margin,
        pcb_data, net_id, print_prefix, direction_labels, single_direction, waypoints)
    if result[0] is not None or track_margin == 0 or not config.power_tap_neckdown:
        return result + (False,)
    print(f"{print_prefix}{YELLOW}Wide route blocked - retrying at default track width (neck-down){RESET}")
    retry = _route_connection_at_margin(
        router, obstacles, config, sources, targets, 0,
        pcb_data, net_id, print_prefix, direction_labels, single_direction, waypoints)
    if retry[0] is None:
        # Keep the WIDE attempt's frontier for rip-up analysis: blockers found
        # by the narrow frontier only help a narrow route, but ripped nets
        # re-route at their own wide width and can fail entirely
        return (result[0], result[1] + retry[1]) + result[2:] + (False,)
    return (retry[0], result[1] + retry[1]) + retry[2:] + (True,)


def _route_connection_at_margin(router, obstacles, config, sources, targets, track_margin,
                                pcb_data, net_id, print_prefix="",
                                direction_labels=("forward", "backward"), single_direction=False,
                                waypoints=None):
    """Route sources->targets, steering through the guide corridor (issue #7).

    A drop-in replacement for _probe_route_with_frontier with the SAME return
    shape. When a guide corridor is configured (config.corridor_waypoints) it
    routes sources -> waypoints -> targets as concatenated A* legs; otherwise it
    behaves exactly like _probe_route_with_frontier.

    The waypoints only steer the path BETWEEN the given sources and targets -
    endpoint/pad/MST selection is the caller's and is left untouched. It is
    strictly best-effort: a waypoint that can't be reached (or that would strand
    the target) is dropped, and if no waypoints can be followed it falls back to
    the direct sources->targets route. So a corridor can never make a connection
    fail that would otherwise route, and the worst it can do is be ignored.
    """
    def direct():
        return _probe_route_with_frontier(
            router, obstacles, sources, targets, config,
            print_prefix=print_prefix, direction_labels=direction_labels,
            track_margin=track_margin, pcb_data=pcb_data, current_net_id=net_id,
            single_direction=single_direction)

    # `waypoints` may be a per-segment bucket (multi-point MST edge); when not
    # given, fall back to the whole corridor (single-segment / 2-pad nets).
    if waypoints is None:
        waypoints = getattr(config, 'corridor_waypoints', None)
    # Bus routing (single_direction) has its own neighbor attraction; leave it be.
    if not waypoints or single_direction or not sources or not targets:
        return direct()

    # Legs use the SAME track_margin the direct route would - inflating it can make
    # a leg unroutable where a direct route succeeds, violating "a corridor never
    # makes a route worse than no corridor".
    num_layers = len(config.layers)
    net_track_width = config.get_net_track_width(net_id, config.layers[0])
    snap_margin = max(1, int(math.ceil((net_track_width / 2 + config.clearance) / config.grid_step)))

    # Orient waypoints to enter at the end nearest the sources.
    def d2(a, b):
        return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2
    s0 = sources[0]
    wp = list(waypoints)
    if d2(s0, wp[0]) > d2(s0, wp[-1]):
        wp.reverse()

    # Drop waypoints sitting essentially on this segment's own endpoints. The user
    # usually draws the guide starting/ending at the pads, but a waypoint right next
    # to a pad is redundant (the route reaches the pad anyway) and, because the pad's
    # clearance halo blocks the current layer there, can force a needless via/detour.
    endpoint_cells = list(sources) + list(targets)
    skip_d = 2 * snap_margin
    wp = [(wx, wy) for (wx, wy) in wp
          if not any(abs(wx - c[0]) + abs(wy - c[1]) <= skip_d for c in endpoint_cells)]
    if not wp:
        return direct()

    def _waypoint_cells(wgx, wgy, prefer_layer):
        """Candidate target cell(s) for a waypoint.

        Each leg is an independent A* that can't see the via cost at a leg
        boundary, so switching layers between waypoints would leave spurious
        vias. Once the route is committed to a layer we therefore only steer to a
        waypoint that's clear on THAT layer; if it isn't, we skip the waypoint
        (return []) rather than change layers - the leg to the next waypoint still
        follows the corridor, and a leg only vias when it genuinely must. Before a
        layer is committed (e.g. a through-hole start spanning layers) we pick any
        clear layer, snapping to the nearest clear cell if the vertex isn't clear.
        """
        if prefer_layer is not None:
            if _cell_margin_clear(obstacles, wgx, wgy, prefer_layer, snap_margin):
                return [(wgx, wgy, prefer_layer)]
            return []
        free = [(wgx, wgy, L) for L in range(num_layers)
                if _cell_margin_clear(obstacles, wgx, wgy, L, snap_margin)]
        if free:
            return free
        nf = _nearest_free_cell(obstacles, wgx, wgy, num_layers, margin=snap_margin)
        return [nf] if nf is not None else []

    spine: List[Tuple[int, int, int]] = []
    total = 0
    current = list(sources)
    # checkpoints[i] = (len(spine), current_sources) after accepting i waypoints.
    checkpoints = [(0, list(sources))]

    def _extend(path):
        if spine and spine[-1] == path[0]:
            spine.extend(path[1:])
        else:
            spine.extend(path)

    for (wgx, wgy) in wp:
        # The committed layer is the one all current sources share (a single cell,
        # or e.g. tap points all on the same track layer); None until committed
        # (a through-hole start spans both). _waypoint_cells keeps the route on the
        # committed layer (skipping waypoints not clear there) to avoid boundary vias.
        cur_layers = set(c[2] for c in current)
        prefer_layer = next(iter(cur_layers)) if len(cur_layers) == 1 else None
        tgts = _waypoint_cells(wgx, wgy, prefer_layer)
        if not tgts:
            continue
        path, iters = _route_leg(router, obstacles, config, current, tgts,
                                 track_margin, pcb_data, net_id)
        total += iters
        if path is None:
            continue  # waypoint unreachable from here -> drop it
        _extend(path)
        current = [path[-1]]
        checkpoints.append((len(spine), current))

    if len(checkpoints) == 1:
        # No waypoints could be placed/followed -> behave exactly like no corridor.
        return direct()

    # Reach the targets, backing off trailing waypoints if the approach is stranded.
    while True:
        path, iters = _route_leg(router, obstacles, config, current, targets,
                                 track_margin, pcb_data, net_id)
        total += iters
        if path is not None:
            _extend(path)
            break
        if len(checkpoints) > 1:
            checkpoints.pop()
            trunc, current = checkpoints[-1]
            del spine[trunc:]
            continue
        # No waypoints could be followed to the target; use the authoritative
        # direct route (also returns blocking info the caller needs for rip-up).
        return direct()

    kept = len(checkpoints) - 1
    dropped = len(wp) - kept
    if dropped > 0:
        print(f"{print_prefix}Guide corridor: followed {kept}/{len(wp)} waypoints "
              f"(dropped {dropped} that would have blocked this segment)")
    elif kept > 0:
        print(f"{print_prefix}Guide corridor: following {kept} waypoint(s)")

    return (spine, total, [], [], False, total, 0)


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
    attraction_bonus = config.cell_cost(config.vertical_attraction_cost) if config.vertical_attraction_cost > 0 else 0

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
    router = VisualRouter(via_cost=config.via_cost_units(), h_weight=config.heuristic_weight,
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
        router = VisualRouter(via_cost=config.via_cost_units(), h_weight=config.heuristic_weight,
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

    # Distribute guide-corridor waypoints across the MST edges: each waypoint
    # steers the edge it's nearest to, so the net follows the drawn line across
    # its whole topology, not just one edge (issue #7). Buckets are passed to the
    # main edge here and to the tap edges in route_multipoint_taps.
    pad_grid = [(info[0], info[1]) for info in pad_info]
    waypoint_buckets = assign_waypoints_to_mst_edges(
        getattr(config, 'corridor_waypoints', None) or [], pad_grid, mst_edges)

    # Get stub free ends for proximity zone checking (consistent with route_net)
    free_end_sources, free_end_targets, _ = get_net_endpoints(pcb_data, net_id, config, use_stub_free_ends=True)
    # Fallback to the net's pad grid positions when there are no stub free ends
    # (a multipoint net with no prior copper, e.g. a fresh all-pad power net).
    # The old code referenced undefined locals `sources`/`targets` here, which
    # crashed route_multipoint_main with UnboundLocalError on exactly those
    # nets (stress test: confirmed on 5+ boards). pad_info rows are
    # (gx, gy, layer_idx, orig_x, orig_y, endpoint_obj).
    pad_prox = [(info[0], info[1], info[2]) for info in pad_info]
    if free_end_sources:
        prox_check_sources = [(s[0], s[1], s[2]) for s in free_end_sources]
    else:
        prox_check_sources = pad_prox
    if free_end_targets:
        prox_check_targets = [(t[0], t[1], t[2]) for t in free_end_targets]
    else:
        prox_check_targets = pad_prox

    # Calculate vertical attraction parameters
    attraction_radius_grid = coord.to_grid_dist(config.vertical_attraction_radius) if config.vertical_attraction_radius > 0 else 0
    attraction_bonus = config.cell_cost(config.vertical_attraction_cost) if config.vertical_attraction_cost > 0 else 0

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
    router = GridRouter(via_cost=config.via_cost_units(), h_weight=config.heuristic_weight,
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

    # Route the main edge: try MST edges longest-first until one routes.
    # Issue #101: one boxed pad must not abandon the whole net - the old code
    # gave up entirely when the single longest edge failed, zeroing 55-pad
    # power nets whose other 54 pads had clear connections. A failed edge now
    # falls through to the next candidate; Phase 3 later handles every
    # remaining edge individually with honest failed-pad reporting.
    max_main_attempts = min(len(mst_edges), 8)
    path = None
    total_iterations = 0
    cumulative_iterations = 0
    last_failure = None
    routed_edge_pos = None
    for attempt in range(max_main_attempts):
        idx_a, idx_b, edge_len = mst_edges[attempt]
        label = ("routing longest MST edge" if attempt == 0
                 else f"retrying with next MST edge ({attempt + 1}/{max_main_attempts})")
        print(f"  Multi-point net Phase 1: {label} (pads {idx_a} and {idx_b}, length={edge_len:.2f}mm)")

        # Build source/target for this pad pair
        pad_a = pad_info[idx_a]
        pad_b = pad_info[idx_b]

        # For through-hole pads, create sources/targets on ALL layers (router
        # can reach any layer) - avoids unnecessary vias
        pad_a_obj = pad_a[5] if len(pad_a) > 5 else None
        pad_b_obj = pad_b[5] if len(pad_b) > 5 else None

        if pad_a_obj and hasattr(pad_a_obj, 'layers') and '*.Cu' in pad_a_obj.layers:
            sources = [(pad_a[0], pad_a[1], layer_idx) for layer_idx in range(len(layer_names))]
        else:
            sources = [(pad_a[0], pad_a[1], pad_a[2])]  # (gx, gy, layer_idx)

        if pad_b_obj and hasattr(pad_b_obj, 'layers') and '*.Cu' in pad_b_obj.layers:
            targets = [(pad_b[0], pad_b[1], layer_idx) for layer_idx in range(len(layer_names))]
        else:
            targets = [(pad_b[0], pad_b[1], pad_b[2])]

        # Mark source/target cells (same-net pad cells; safe to accumulate)
        for gx, gy, layer in sources + targets:
            obstacles.add_source_target_cell(gx, gy, layer)

        # Use probe routing helper, steered through this edge's bucket of
        # corridor waypoints (tap edges follow their own buckets later).
        path, total_iterations, forward_blocked, backward_blocked, reversed_path, fwd_iters, bwd_iters, necked_down = _route_main_connection(
            router, obstacles, config, sources, targets, track_margin,
            pcb_data, net_id, print_prefix="  ", direction_labels=("forward", "backward"),
            waypoints=waypoint_buckets.get(frozenset((idx_a, idx_b)), [])
        )
        cumulative_iterations += total_iterations

        if path is not None:
            routed_edge_pos = attempt
            break

        print(f"  Phase 1 edge (pads {idx_a},{idx_b}) failed after {total_iterations} iterations"
              + (" - trying next MST edge" if attempt + 1 < max_main_attempts else ""))
        last_failure = {
            'failed': True,
            'blocked_cells_forward': forward_blocked,
            'blocked_cells_backward': backward_blocked,
            'iterations_forward': fwd_iters,
            'iterations_backward': bwd_iters,
        }

    if path is None:
        print(f"  Failed to route a main edge after {cumulative_iterations} iterations "
              f"({max_main_attempts} edge(s) tried)")
        failure = dict(last_failure or {'failed': True})
        failure['iterations'] = cumulative_iterations
        return failure

    # Phase 3 assumes mst_edges[0] is the edge Phase 1 routed - move the
    # successful edge to the front when a fallback edge won.
    if routed_edge_pos:
        mst_edges = ([mst_edges[routed_edge_pos]] + mst_edges[:routed_edge_pos]
                     + mst_edges[routed_edge_pos + 1:])
    total_iterations = cumulative_iterations

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
        through_hole_positions,
        pcb_data
    )
    if necked_down:
        # Both endpoints are pads: neck the start side too
        segments = _apply_neckdown_widths(segments, config, net_id, obstacles,
                                          coord, layer_names, track_margin, neck_start=True)

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
        # Per-edge guide-corridor waypoint buckets (issue #7), for Phase 3 taps
        'waypoint_buckets': waypoint_buckets,
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
    waypoint_buckets = main_result.get('waypoint_buckets', {})  # per-edge corridor waypoints

    # Build set of "routed components" - components with at least one explicitly routed pad
    # Pads in zone-connected components are effectively routed if any pad in that component is routed
    routed_components = {pad_components.get(idx, idx) for idx in routed_indices}

    # Get the current segments (which may have meanders from length matching)
    all_segments = list(main_result['new_segments'])
    all_vias = list(main_result.get('new_vias', []))

    coord = GridCoord(config.grid_step)
    layer_names = config.layers

    # Cells needing NO new via on a layer change: same-net through-hole pads and
    # same-net vias (each already connects all layers). Seeding with the net's
    # current vias (the main edge + pre-existing) and updating it as each tap edge
    # places vias lets a later edge REUSE a via the main edge already dropped at
    # the same cell, instead of stacking a second coincident one (EPHY_TX_N).
    through_hole_positions = set(get_same_net_through_hole_positions(pcb_data, net_id, config))
    for _v in pcb_data.vias:
        if _v.net_id == net_id:
            through_hole_positions.add(coord.to_grid(_v.x, _v.y))

    # In-progress vias (this net's main + earlier tap edges) are NOT yet in
    # pcb_data, so the per-net obstacle clone doesn't know about them. Register
    # each one in the live map so a LATER edge (1) REUSES it as a zero-cost free
    # via when its path lands on the cell, and (2) cannot drop a SECOND via within
    # hole-to-hole of it. Without this, a later branch dropped a via a sub-mm away
    # -- the VTT multipoint junction double-via (hole_to_hole DRC). The ring skips
    # the via's own cell so reuse stays open.
    _vv_radius = (config.via_size + config.clearance) * coord.inv_step
    _vv_rng = int(math.ceil(_vv_radius))
    _vv_sq = _vv_radius * _vv_radius

    def _register_inprogress_via(v):
        vgx, vgy = coord.to_grid(v.x, v.y)
        obstacles.add_free_via(vgx, vgy)
        for ex in range(-_vv_rng, _vv_rng + 1):
            for ey in range(-_vv_rng, _vv_rng + 1):
                d = ex * ex + ey * ey
                if 0 < d <= _vv_sq:
                    obstacles.add_blocked_via(vgx + ex, vgy + ey)

    for _v in all_vias:
        _register_inprogress_via(_v)

    # Get remaining MST edges (skip the first one which was routed in Phase 1)
    # MST edges are already sorted longest-first
    remaining_edges = mst_edges[1:] if len(mst_edges) > 1 else []

    if not remaining_edges:
        print(f"  No remaining MST edges to route in Phase 3")
        return main_result

    print(f"  Multi-point net Phase 3: routing {len(remaining_edges)} remaining MST edges (longest first)")

    # Calculate vertical attraction parameters
    attraction_radius_grid = coord.to_grid_dist(config.vertical_attraction_radius) if config.vertical_attraction_radius > 0 else 0
    attraction_bonus = config.cell_cost(config.vertical_attraction_cost) if config.vertical_attraction_cost > 0 else 0

    router = GridRouter(via_cost=config.via_cost_units(), h_weight=config.heuristic_weight,
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
    fallback_attempted = set()  # Pads attempted directly after their MST edge chain failed
    max_passes = len(remaining_edges) * 2 + len(pad_info)  # Safety limit

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
            # Fallback: a failed MST edge orphans its entire downstream
            # subtree - those pads' edges are never eligible because their
            # source side never becomes routed. Since tap routing launches
            # from ALL existing copper anyway (the MST edge is only an
            # ordering), attempt each orphaned pad directly once. Even when
            # the attempt fails, its blocked frontier feeds the Phase 3
            # rip-up analysis with the pads' ACTUAL blockers (issues
            # #101/#103: previously a walled-off region produced no frontier
            # data at all, so nothing was ever ripped).
            best = None
            for i in range(len(pad_info)):
                if i in routed_indices or pad_components.get(i, i) in routed_components:
                    continue
                if i in fallback_attempted:
                    continue
                xi, yi = pad_info[i][3], pad_info[i][4]
                for j in routed_indices:
                    d = abs(xi - pad_info[j][3]) + abs(yi - pad_info[j][4])
                    if best is None or d < best[2]:
                        best = (j, i, d)
            if best is not None:
                fallback_attempted.add(best[1])
                edge_to_route = best
                print(f"    Fallback: attempting orphaned pad {best[1]} directly from connected copper")

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

        path, tap_iterations, forward_blocked, backward_blocked, reversed_tap_path, _, _, necked_down = _route_main_connection(
            router, obstacles, config, sources, targets, track_margin,
            pcb_data, net_id, print_prefix="      ", direction_labels=("forward", "backward"),
            waypoints=waypoint_buckets.get(frozenset((src_idx, tgt_idx)), [])
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
            through_hole_positions,
            pcb_data
        )
        if necked_down:
            segments = _apply_neckdown_widths(segments, config, net_id, obstacles,
                                              coord, layer_names, track_margin)
        all_segments.extend(segments)
        all_vias.extend(vias)
        # Make this edge's vias reusable by later edges of the same net, so a
        # later edge changing layers at one of these cells reuses the via, and
        # block a hole-to-hole ring so a later edge can't drop a via beside it.
        for _v in vias:
            through_hole_positions.add(coord.to_grid(_v.x, _v.y))
            _register_inprogress_via(_v)

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
    through_hole_positions: Set[Tuple[int, int]] = None,
    pcb_data: PCBData = None
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

    # Per-point float positions. Route the two TERMINAL segments to the EXACT
    # off-grid endpoint instead of its grid-cell stand-in when that cell grazes a
    # foreign pad but the exact endpoint clears it (#4 off-grid connection graze).
    pts = [coord.to_float(p[0], p[1]) for p in path]
    merge_start = merge_end = False
    if pcb_data is not None:
        merge_start = _merge_terminal_to_exact(path, 0, 1, start_original, pts,
                                               pcb_data, net_id, config, layer_names)
        merge_end = _merge_terminal_to_exact(path, len(path) - 1, len(path) - 2, end_original, pts,
                                             pcb_data, net_id, config, layer_names)

    # Add connecting segment from original start to first path point if needed
    # (skipped when merged: the first path segment now ends at the exact point)
    if start_original and not merge_start:
        first_grid_x, first_grid_y = pts[0]
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

        x1, y1 = pts[i]
        x2, y2 = pts[i + 1]

        if layer1 != layer2:
            # Check if layer change is at an existing through-hole pad
            # If so, skip creating a via - the pad provides the layer transition
            if through_hole_positions and (gx1, gy1) in through_hole_positions:
                # No via needed - existing through-hole pad connects all layers
                pass
            else:
                vx, vy = coord.to_float(gx1, gy1)  # via stays on the grid cell
                via = Via(
                    x=vx, y=vy,
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
    # (skipped when merged: the last path segment now ends at the exact point)
    if end_original and not merge_end:
        last_grid_x, last_grid_y = pts[-1]
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


def _seg_length(seg) -> float:
    return math.hypot(seg.end_x - seg.start_x, seg.end_y - seg.start_y)


def _split_segment_at(seg, dist_from_end: float):
    """Split a segment at dist_from_end mm before its end point.

    Returns (near_part, far_part) where far_part is the dist_from_end-long
    piece touching seg's end. Returns (None, seg) if the segment is shorter
    than dist_from_end.
    """
    length = _seg_length(seg)
    if length <= dist_from_end:
        return None, seg
    t = 1.0 - dist_from_end / length
    mx = seg.start_x + (seg.end_x - seg.start_x) * t
    my = seg.start_y + (seg.end_y - seg.start_y) * t
    near = Segment(start_x=seg.start_x, start_y=seg.start_y, end_x=mx, end_y=my,
                   width=seg.width, layer=seg.layer, net_id=seg.net_id)
    far = Segment(start_x=mx, start_y=my, end_x=seg.end_x, end_y=seg.end_y,
                  width=seg.width, layer=seg.layer, net_id=seg.net_id)
    return near, far


def _segment_fits_wide(seg, obstacles, coord: GridCoord, layer_idx: int, margin: int) -> bool:
    """True if every grid cell along the segment clears the wide-track margin."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
    for gx, gy in walk_line(gx1, gy1, gx2, gy2):
        if obstacles.is_blocked_with_margin(gx, gy, layer_idx, margin):
            return False
    return True


def _flip_segments(segments):
    """Reverse a connected segment run end-to-end (order and direction)."""
    return [Segment(start_x=s.end_x, start_y=s.end_y, end_x=s.start_x, end_y=s.start_y,
                    width=s.width, layer=s.layer, net_id=s.net_id)
            for s in reversed(segments)]


def _neck_pass(segments, config: GridRouteConfig, obstacles, coord: GridCoord,
               layer_map: Dict[str, int], track_margin: int):
    """Narrow the last neckdown_length mm of the run (the pad is at the list
    END); beyond that, keep the wide width only where the wide clearance
    fits. Never re-widens an already-narrow segment (so a second pass from
    the other end preserves the first pass's neck)."""
    def fits(s):
        return _segment_fits_wide(s, obstacles, coord, layer_map.get(s.layer, 0), track_margin)

    out = []  # built in reverse (pad-first)
    cum = 0.0
    for seg in reversed(segments):
        narrow_w = config.get_track_width(seg.layer)
        length = _seg_length(seg)
        if seg.width <= narrow_w:
            out.append(seg)
        elif cum >= config.neckdown_length:
            if not fits(seg):
                seg.width = narrow_w
            out.append(seg)
        elif cum + length > config.neckdown_length:
            # Straddles the neck boundary: split there (the far piece,
            # touching the pad side, is neckdown_length - cum long)
            near, far = _split_segment_at(seg, config.neckdown_length - cum)
            far.width = narrow_w
            out.append(far)
            if not fits(near):
                near.width = narrow_w
            out.append(near)
        else:
            seg.width = narrow_w
            out.append(seg)
        cum += length
    out.reverse()
    return out


def _apply_neckdown_widths(segments, config: GridRouteConfig, net_id: int,
                           obstacles, coord: GridCoord, layer_names: List[str],
                           track_margin: int, neck_start: bool = False):
    """Assign widths to a neck-down route (issue #72).

    The path was routed at the layer's default width because the power width
    did not fit. Segments within config.neckdown_length of the target pad
    (the END of the list; also the start when neck_start is set, for routes
    that end on pads at both ends) stay narrow; farther segments return to
    the power width wherever the wide clearance fits, with an optional
    stepped taper at each narrow->wide transition.

    Returns a new segment list (segments may be split for the taper).
    """
    layer_map = {name: i for i, name in enumerate(layer_names)}
    out = _neck_pass(segments, config, obstacles, coord, layer_map, track_margin)
    if neck_start:
        out = _flip_segments(_neck_pass(_flip_segments(out), config, obstacles,
                                        coord, layer_map, track_margin))
    wide_flags = [s.width > config.get_track_width(s.layer) for s in out]

    # Suppress short wide islands (a wide run between narrow pinches that is
    # barely longer than its tapers just adds notch noise)
    min_island = 2 * config.neckdown_taper_length
    i = 0
    while i < len(out):
        if not wide_flags[i]:
            i += 1
            continue
        j = i
        run_len = 0.0
        while j < len(out) and wide_flags[j]:
            run_len += _seg_length(out[j])
            j += 1
        is_island = i > 0 and j < len(out)  # narrow (or pad) on both sides
        if is_island and run_len <= min_island:
            for k in range(i, j):
                out[k].width = config.get_track_width(out[k].layer)
                wide_flags[k] = False
        i = j

    if config.neckdown_taper_length <= 0:
        return out

    # Stepped taper wherever a wide segment meets a narrow one on the same
    # layer: carve the wide segment's adjoining end into width steps
    TAPER_STEPS = 4

    def _taper_pieces(seg, narrow_end: str):
        """Split seg into [body + taper steps]; narrow_end is 'start' or 'end'."""
        narrow_w = config.get_track_width(seg.layer)
        wide_w = seg.width
        taper_len = min(config.neckdown_taper_length, _seg_length(seg) / 3)
        if taper_len <= 0:
            return [seg]
        flipped = narrow_end == 'start'
        if flipped:  # work as if the narrow side is at the end
            seg = Segment(start_x=seg.end_x, start_y=seg.end_y,
                          end_x=seg.start_x, end_y=seg.start_y,
                          width=seg.width, layer=seg.layer, net_id=seg.net_id)
        body, taper = _split_segment_at(seg, taper_len)
        if body is None:
            return [seg]
        pieces = [body]
        step_len = taper_len / TAPER_STEPS
        remaining = taper
        for s in range(TAPER_STEPS):
            if s < TAPER_STEPS - 1 and _seg_length(remaining) > step_len:
                piece, remaining = _split_segment_at(remaining, _seg_length(remaining) - step_len)
            else:
                piece, remaining = remaining, None
            piece.width = wide_w + (narrow_w - wide_w) * (s + 1) / (TAPER_STEPS + 1)
            pieces.append(piece)
            if remaining is None:
                break
        if flipped:  # restore original direction and order
            pieces = [Segment(start_x=p.end_x, start_y=p.end_y,
                              end_x=p.start_x, end_y=p.start_y,
                              width=p.width, layer=p.layer, net_id=p.net_id)
                      for p in reversed(pieces)]
        return pieces

    tapered = []
    for i, seg in enumerate(out):
        if not wide_flags[i]:
            tapered.append(seg)
            continue
        narrow_after = (i + 1 < len(out) and not wide_flags[i + 1]
                        and out[i + 1].layer == seg.layer)
        narrow_before = (i > 0 and not wide_flags[i - 1]
                         and out[i - 1].layer == seg.layer)
        pieces = [seg]
        if narrow_after:
            pieces = _taper_pieces(seg, 'end')
        if narrow_before:
            head = _taper_pieces(pieces[0], 'start')
            pieces = head + pieces[1:]
        tapered.extend(pieces)
    return tapered

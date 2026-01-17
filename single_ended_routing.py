"""
Single-ended net routing functions.

Routes individual nets using A* pathfinding on a grid obstacle map.
"""

import time
from typing import List, Optional, Set, Tuple

# ANSI color codes
RED = '\033[91m'
RESET = '\033[0m'

from kicad_parser import PCBData, Segment, Via
from routing_config import GridRouteConfig, GridCoord
from routing_utils import build_layer_map
from connectivity import (
    get_net_endpoints,
    get_multipoint_net_pads,
    find_closest_point_on_segments,
    compute_mst_edges
)
from obstacle_map import build_obstacle_map, get_same_net_through_hole_positions

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


def _probe_route_with_frontier(
    router: 'GridRouter',
    obstacles: 'GridObstacleMap',
    forward_sources: List,
    forward_targets: List,
    config: 'GridRouteConfig',
    print_prefix: str = "",
    direction_labels: Tuple[str, str] = ("forward", "backward"),
    track_margin: int = 0
) -> Tuple[Optional[List], int, List, List, bool]:
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

    Returns:
        (path, total_iterations, forward_blocked, backward_blocked, reversed_path)
        - path: The found path or None
        - total_iterations: Total iterations used
        - forward_blocked: Blocked cells from forward direction (for rip-up analysis)
        - backward_blocked: Blocked cells from backward direction (for rip-up analysis)
        - reversed_path: Whether path was found going backwards
    """
    first_label, second_label = direction_labels
    probe_iterations = config.max_probe_iterations

    # Probe forward direction
    path, iterations, blocked_cells = router.route_with_frontier(
        obstacles, forward_sources, forward_targets, probe_iterations, track_margin=track_margin)
    first_probe_iters = iterations
    first_blocked = blocked_cells
    total_iterations = first_probe_iters
    reversed_path = False

    # Track blocked cells for both directions
    forward_blocked = first_blocked
    backward_blocked = []

    if path is not None:
        # Found in first probe
        forward_blocked = []  # Success - clear blocked cells
        return path, total_iterations, forward_blocked, backward_blocked, reversed_path

    # Probe backward direction
    path, iterations, blocked_cells = router.route_with_frontier(
        obstacles, forward_targets, forward_sources, probe_iterations, track_margin=track_margin)
    second_probe_iters = iterations
    second_blocked = blocked_cells
    total_iterations += second_probe_iters
    backward_blocked = second_blocked

    if path is not None:
        # Found in second probe
        backward_blocked = []  # Success - clear blocked cells
        return path, total_iterations, forward_blocked, backward_blocked, True

    # Both probes failed to find a path - check if both reached max iterations
    # Only try full search if BOTH probes reached max-probe-iterations (meaning both directions are worth exploring)
    first_reached_max = first_probe_iters >= probe_iterations
    second_reached_max = second_probe_iters >= probe_iterations

    if not (first_reached_max and second_reached_max):
        # At least one probe didn't reach max - that direction is stuck, skip full search
        if not first_reached_max and not second_reached_max:
            print(f"{print_prefix}Both directions stuck ({first_label}={first_probe_iters}, {second_label}={second_probe_iters} < {probe_iterations})")
        elif not first_reached_max:
            print(f"{print_prefix}{first_label} stuck ({first_probe_iters} < {probe_iterations}), {second_label}={second_probe_iters}")
        else:
            print(f"{print_prefix}{second_label} stuck ({second_probe_iters} < {probe_iterations}), {first_label}={first_probe_iters}")
        return None, total_iterations, forward_blocked, backward_blocked, False

    # Both probes reached max iterations - do full search on forward direction
    print(f"{print_prefix}Probe: {first_label}={first_probe_iters}, {second_label}={second_probe_iters} iters, trying {first_label} with full iterations...")

    path, full_iters, full_blocked = router.route_with_frontier(
        obstacles, forward_sources, forward_targets, config.max_iterations, track_margin=track_margin)
    total_iterations += full_iters

    if path is not None:
        forward_blocked = []
        return path, total_iterations, forward_blocked, backward_blocked, False

    # Forward failed, try backward
    print(f"{print_prefix}No route found after {full_iters} iterations ({first_label}), trying {second_label}...")
    forward_blocked = full_blocked

    path, backward_full_iters, backward_full_blocked = router.route_with_frontier(
        obstacles, forward_targets, forward_sources, config.max_iterations, track_margin=track_margin)
    total_iterations += backward_full_iters

    if path is not None:
        backward_blocked = []
        return path, total_iterations, forward_blocked, backward_blocked, True

    backward_blocked = backward_full_blocked
    return None, total_iterations, forward_blocked, backward_blocked, False


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

    router = GridRouter(via_cost=config.via_cost * 1000, h_weight=config.heuristic_weight, turn_cost=config.turn_cost, via_proximity_cost=int(config.via_proximity_cost))

    # Calculate track margin for wide power tracks
    # Power nets need extra clearance from obstacles based on their wider track width
    net_track_width = config.get_net_track_width(net_id, config.layers[0])
    extra_half_width = (net_track_width - config.track_width) / 2
    track_margin = int(extra_half_width / config.grid_step) if extra_half_width > 0 else 0

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
    path, iterations = router.route_multi(obstacles, first_sources, first_targets, probe_iterations, track_margin=track_margin)
    first_probe_iters = iterations
    total_iterations = first_probe_iters

    if path is None:
        # Probe second direction
        path, iterations = router.route_multi(obstacles, second_sources, second_targets, probe_iterations, track_margin=track_margin)
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
                elif not first_reached_max:
                    print(f"{first_label} stuck ({first_probe_iters} < {probe_iterations}), {second_label}={second_probe_iters}")
                else:
                    print(f"{second_label} stuck ({second_probe_iters} < {probe_iterations}), {first_label}={first_probe_iters}")
            else:
                # Both probes reached max - do full search on first direction
                print(f"Probe: {first_label}={first_probe_iters}, {second_label}={second_probe_iters} iters, trying {first_label} with full iterations...")

                path, full_iters = router.route_multi(obstacles, first_sources, first_targets, config.max_iterations, track_margin=track_margin)
                total_iterations += full_iters

                if path is not None:
                    reversed_path = not start_backwards
                else:
                    # First direction failed, try second
                    print(f"No route found after {full_iters} iterations ({first_label}), trying {second_label}...")
                    path, fallback_full_iters = router.route_multi(obstacles, second_sources, second_targets, config.max_iterations, track_margin=track_margin)
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
                              obstacles: GridObstacleMap) -> Optional[dict]:
    """Route a single net using pre-built obstacles (for incremental routing)."""
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

    router = GridRouter(via_cost=config.via_cost * 1000, h_weight=config.heuristic_weight, turn_cost=config.turn_cost, via_proximity_cost=int(config.via_proximity_cost))

    # Calculate track margin for wide power tracks
    net_track_width = config.get_net_track_width(net_id, config.layers[0])
    extra_half_width = (net_track_width - config.track_width) / 2
    track_margin = int(extra_half_width / config.grid_step) if extra_half_width > 0 else 0

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
    path, total_iterations, forward_blocked, backward_blocked, reversed_path = _probe_route_with_frontier(
        router, obstacles, forward_sources, forward_targets, config,
        print_prefix="", direction_labels=direction_labels, track_margin=track_margin
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
            'iterations_forward': total_iterations // 2,  # Approximate
            'iterations_backward': total_iterations - total_iterations // 2,
        }

    print(f"Route found in {total_iterations} iterations, path length: {len(path)}")

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

    # Add source and target positions as allowed cells to override BGA zone blocking
    allow_radius = 10
    for gx, gy, _ in sources_grid + targets_grid:
        for dx in range(-allow_radius, allow_radius + 1):
            for dy in range(-allow_radius, allow_radius + 1):
                obstacles.add_allowed_cell(gx + dx, gy + dy)

    # Mark exact source/target cells so routing can start/end there
    for gx, gy, layer in sources_grid + targets_grid:
        obstacles.add_source_target_cell(gx, gy, layer)

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
    router = VisualRouter(via_cost=config.via_cost * 1000, h_weight=config.heuristic_weight)

    # Try first direction with visualization
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
        router = VisualRouter(via_cost=config.via_cost * 1000, h_weight=config.heuristic_weight)
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

    # Route farthest pair with probe routing (same as single-ended)
    router = GridRouter(via_cost=config.via_cost * 1000, h_weight=config.heuristic_weight, turn_cost=config.turn_cost, via_proximity_cost=int(config.via_proximity_cost))

    # Calculate track margin for wide power tracks
    net_track_width = config.get_net_track_width(net_id, config.layers[0])
    extra_half_width = (net_track_width - config.track_width) / 2
    track_margin = int(extra_half_width / config.grid_step) if extra_half_width > 0 else 0

    # Use probe routing helper
    path, total_iterations, forward_blocked, backward_blocked, reversed_path = _probe_route_with_frontier(
        router, obstacles, sources, targets, config,
        print_prefix="  ", direction_labels=("forward", "backward"), track_margin=track_margin
    )

    if path is None:
        print(f"  Failed to route farthest pair after {total_iterations} iterations")
        return {
            'failed': True,
            'iterations': total_iterations,
            'blocked_cells_forward': forward_blocked,
            'blocked_cells_backward': backward_blocked,
            'iterations_forward': total_iterations // 2,
            'iterations_backward': total_iterations - total_iterations // 2,
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

    router = GridRouter(via_cost=config.via_cost * 1000, h_weight=config.heuristic_weight, turn_cost=config.turn_cost, via_proximity_cost=int(config.via_proximity_cost))

    # Calculate track margin for wide power tracks
    net_track_width = config.get_net_track_width(net_id, config.layers[0])
    extra_half_width = (net_track_width - config.track_width) / 2
    track_margin = int(extra_half_width / config.grid_step) if extra_half_width > 0 else 0

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

            a_routed = idx_a in routed_indices
            b_routed = idx_b in routed_indices

            if a_routed and not b_routed:
                edge_to_route = (idx_a, idx_b, length)  # Route from a to b
                break
            elif b_routed and not a_routed:
                edge_to_route = (idx_b, idx_a, length)  # Route from b to a
                break

        if edge_to_route is None:
            unrouted_pads = len(pad_info) - len(routed_indices)
            if unrouted_pads > 0:
                print(f"  {RED}Warning: {unrouted_pads} pad(s) not connected ({len(failed_edges)} MST edge(s) failed){RESET}")
            break

        src_idx, tgt_idx, edge_len = edge_to_route

        src_pad = pad_info[src_idx]
        tgt_pad = pad_info[tgt_idx]

        # Show progress: [current/total] with failure count (global across all nets)
        current_global = global_offset + edges_routed + len(failed_edges) + 1
        total_failed = global_failed + len(failed_edges)
        fail_str = f" ({total_failed} failed)" if total_failed > 0 else ""
        print(f"    [{current_global}/{global_total}]{fail_str} Routing MST edge: pad {src_idx} -> pad {tgt_idx} (length={edge_len:.2f}mm)")

        # Get target pad coordinates
        tgt_x, tgt_y = tgt_pad[3], tgt_pad[4]

        # Get ALL points along existing segments and vias as potential tap sources
        # The router will find the shortest path from ANY of these points
        # Vias are included on ALL layers since they connect all copper layers
        all_tap_points = get_all_segment_tap_points(all_segments, coord, layer_names, vias=all_vias)

        if not all_tap_points:
            # Fall back to using source pad position directly
            src_x, src_y = src_pad[3], src_pad[4]
            src_gx, src_gy = coord.to_grid(src_x, src_y)
            sources = [(src_gx, src_gy, src_pad[2])]
            tap_point_map = {(src_gx, src_gy, src_pad[2]): (src_x, src_y, layer_names[src_pad[2]])}
            print(f"      No tap points found, using pad position directly")
        else:
            # Use all segment points as sources - router finds shortest path from any
            sources = [(gx, gy, layer_idx) for gx, gy, layer_idx, _, _ in all_tap_points]
            # Map grid coords back to original coords for segment generation
            tap_point_map = {(gx, gy, layer_idx): (ox, oy, layer_names[layer_idx])
                            for gx, gy, layer_idx, ox, oy in all_tap_points}

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

        # Route from ANY tap point to target - router finds shortest path
        # Use probe routing helper to detect stuck directions early
        tap_start_time = time.time()

        path, tap_iterations, forward_blocked, backward_blocked, reversed_tap_path = _probe_route_with_frontier(
            router, obstacles, sources, targets, config,
            print_prefix="      ", direction_labels=("forward", "backward"), track_margin=track_margin
        )

        # If path was found in reverse direction, reverse it so it goes sources -> targets
        if path is not None and reversed_tap_path:
            path = list(reversed(path))

        # Combine blocked cells from both directions for rip-up analysis
        blocked_cells = forward_blocked + backward_blocked

        tap_elapsed = time.time() - tap_start_time
        total_iterations += tap_iterations

        if path is None:
            print(f"      {RED}Failed to route MST edge after {tap_iterations} iterations ({tap_elapsed:.2f}s){RESET}")
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

        # Mark target pad as routed and remove edge from remaining
        routed_indices.add(tgt_idx)
        remaining_edges = [e for e in remaining_edges if not (
            (e[0] == src_idx and e[1] == tgt_idx) or (e[0] == tgt_idx and e[1] == src_idx)
        )]
        edges_routed += 1

    pads_connected = len(routed_indices)
    pads_total = len(pad_info)
    pads_failed = pads_total - pads_connected
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

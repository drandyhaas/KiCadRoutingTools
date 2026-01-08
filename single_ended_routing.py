"""
Single-ended net routing functions.

Routes individual nets using A* pathfinding on a grid obstacle map.
"""

import random
from typing import List, Optional, Tuple

from kicad_parser import PCBData, Segment, Via
from routing_config import GridRouteConfig, GridCoord
from routing_utils import (
    get_net_endpoints,
    get_multipoint_net_pads,
    find_farthest_pad_pair,
    find_closest_point_on_segments
)
from obstacle_map import build_obstacle_map

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

    router = GridRouter(via_cost=config.via_cost * 1000, h_weight=config.heuristic_weight, turn_cost=config.turn_cost)

    # Determine direction order
    if config.direction_order == "random":
        start_backwards = random.choice([True, False])
    elif config.direction_order in ("backwards", "backward"):
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

    # Try first direction, then second if first fails
    reversed_path = False
    total_iterations = 0

    path, iterations = router.route_multi(obstacles, first_sources, first_targets, config.max_iterations)
    total_iterations += iterations

    if path is None:
        print(f"No route found after {iterations} iterations ({first_label}), trying {second_label}...")
        path, iterations = router.route_multi(obstacles, second_sources, second_targets, config.max_iterations)
        total_iterations += iterations
        if path is not None:
            reversed_path = not start_backwards  # True if we ended up going backwards

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
                width=config.track_width,
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
            via = Via(
                x=x1, y=y1,
                size=config.via_size,
                drill=config.via_drill,
                layers=[layer_names[layer1], layer_names[layer2]],
                net_id=net_id
            )
            new_vias.append(via)
        else:
            if (x1, y1) != (x2, y2):
                seg = Segment(
                    start_x=x1, start_y=y1,
                    end_x=x2, end_y=y2,
                    width=config.track_width,
                    layer=layer_names[layer1],
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
                width=config.track_width,
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

    router = GridRouter(via_cost=config.via_cost * 1000, h_weight=config.heuristic_weight, turn_cost=config.turn_cost)

    # Determine direction order
    if config.direction_order == "random":
        start_backwards = random.choice([True, False])
    elif config.direction_order in ("backwards", "backward"):
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

    reversed_path = False
    total_iterations = 0

    # Track blocked cells from both directions for blocking analysis
    first_blocked_cells = []
    second_blocked_cells = []

    path, iterations, blocked_cells = router.route_with_frontier(
        obstacles, first_sources, first_targets, config.max_iterations)
    total_iterations += iterations
    first_blocked_cells = blocked_cells
    first_iterations = iterations

    if path is None:
        print(f"No route found after {iterations} iterations ({first_label}), trying {second_label}...")
        path, iterations, blocked_cells = router.route_with_frontier(
            obstacles, second_sources, second_targets, config.max_iterations)
        total_iterations += iterations
        second_blocked_cells = blocked_cells
        second_iterations = iterations
        if path is not None:
            reversed_path = not start_backwards
    else:
        second_iterations = 0

    if path is None:
        print(f"No route found after {total_iterations} iterations (both directions)")
        # Return blocked cells for each direction (forward/backward labels may be swapped)
        if first_label == "forward":
            forward_blocked = first_blocked_cells
            backward_blocked = second_blocked_cells
            forward_iters = first_iterations
            backward_iters = second_iterations
        else:
            forward_blocked = second_blocked_cells
            backward_blocked = first_blocked_cells
            forward_iters = second_iterations
            backward_iters = first_iterations
        return {
            'failed': True,
            'iterations': total_iterations,
            'blocked_cells_forward': forward_blocked,
            'blocked_cells_backward': backward_blocked,
            'iterations_forward': forward_iters,
            'iterations_backward': backward_iters,
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

    new_segments = []
    new_vias = []

    if start_original:
        first_grid_x, first_grid_y = coord.to_float(path_start[0], path_start[1])
        orig_x, orig_y, orig_layer = start_original
        if abs(orig_x - first_grid_x) > 0.001 or abs(orig_y - first_grid_y) > 0.001:
            seg = Segment(
                start_x=orig_x, start_y=orig_y,
                end_x=first_grid_x, end_y=first_grid_y,
                width=config.track_width,
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
            via = Via(
                x=x1, y=y1,
                size=config.via_size,
                drill=config.via_drill,
                layers=[layer_names[layer1], layer_names[layer2]],
                net_id=net_id
            )
            new_vias.append(via)
        else:
            if (x1, y1) != (x2, y2):
                seg = Segment(
                    start_x=x1, start_y=y1,
                    end_x=x2, end_y=y2,
                    width=config.track_width,
                    layer=layer_names[layer1],
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
                width=config.track_width,
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

    # Determine direction order
    if config.direction_order == "random":
        start_backwards = random.choice([True, False])
    elif config.direction_order in ("backwards", "backward"):
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

    new_segments = []
    new_vias = []

    if start_original:
        first_grid_x, first_grid_y = coord.to_float(path_start[0], path_start[1])
        orig_x, orig_y, orig_layer = start_original
        if abs(orig_x - first_grid_x) > 0.001 or abs(orig_y - first_grid_y) > 0.001:
            seg = Segment(
                start_x=orig_x, start_y=orig_y,
                end_x=first_grid_x, end_y=first_grid_y,
                width=config.track_width,
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
            via = Via(
                x=x1, y=y1,
                size=config.via_size,
                drill=config.via_drill,
                layers=[layer_names[layer1], layer_names[layer2]],
                net_id=net_id
            )
            new_vias.append(via)
        else:
            if (x1, y1) != (x2, y2):
                seg = Segment(
                    start_x=x1, start_y=y1,
                    end_x=x2, end_y=y2,
                    width=config.track_width,
                    layer=layer_names[layer1],
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
                width=config.track_width,
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
    Route only the main (farthest pair) connection of a multi-point net.

    This is Phase 1 of multi-point routing. It routes the two pads that are
    farthest apart by Manhattan distance, creating a clean 2-point route
    suitable for length matching.

    After length matching is applied, call route_multipoint_taps() to
    complete the remaining connections.

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
        - 'routed_pad_indices': Set of indices already routed (the farthest pair)
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

    # Extract pads from pad_info
    pads = [info[5] for info in pad_info]

    # Find the two FARTHEST pads (for length matching compatibility)
    idx_a, idx_b = find_farthest_pad_pair(pads)

    print(f"  Multi-point net Phase 1: routing farthest pair (pads {idx_a} and {idx_b})")

    # Build source/target for farthest pair
    pad_a = pad_info[idx_a]
    pad_b = pad_info[idx_b]

    sources = [(pad_a[0], pad_a[1], pad_a[2])]  # (gx, gy, layer_idx)
    targets = [(pad_b[0], pad_b[1], pad_b[2])]

    # Mark source/target cells
    for gx, gy, layer in sources + targets:
        obstacles.add_source_target_cell(gx, gy, layer)

    # Route farthest pair (use route_with_frontier to get blocked cells for analysis)
    router = GridRouter(via_cost=config.via_cost * 1000, h_weight=config.heuristic_weight, turn_cost=config.turn_cost)
    path, iterations, blocked_cells = router.route_with_frontier(obstacles, sources, targets, config.max_iterations)

    if path is None:
        print(f"  Failed to route farthest pair after {iterations} iterations")
        return {
            'failed': True,
            'iterations': iterations,
            'blocked_cells_forward': blocked_cells,
            'blocked_cells_backward': [],
            'iterations_forward': iterations,
            'iterations_backward': 0,
        }

    # Convert path to segments/vias
    segments, vias = _path_to_segments_vias(
        path, coord, layer_names, net_id, config,
        (pad_a[3], pad_a[4], layer_names[pad_a[2]]),  # start_original
        (pad_b[3], pad_b[4], layer_names[pad_b[2]])   # end_original
    )

    print(f"  Phase 1 routed in {iterations} iterations, {len(segments)} segments")

    return {
        'new_segments': segments,
        'new_vias': vias,
        'iterations': iterations,
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
    }


def route_multipoint_taps(
    pcb_data: PCBData,
    net_id: int,
    config: GridRouteConfig,
    obstacles: 'GridObstacleMap',
    main_result: dict
) -> Optional[dict]:
    """
    Route the remaining tap connections for a multi-point net.

    This is Phase 3 of multi-point routing - called AFTER length matching
    has been applied to the main route. It finds tap points on the (now
    meandered) segments and routes to remaining pads.

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
    routed_indices = main_result['routed_pad_indices']

    # Get the current segments (which may have meanders from length matching)
    all_segments = list(main_result['new_segments'])
    all_vias = list(main_result.get('new_vias', []))

    coord = GridCoord(config.grid_step)
    layer_names = config.layers

    # Extract pads from pad_info
    pads = [info[5] for info in pad_info]

    remaining_indices = [i for i in range(len(pad_info)) if i not in routed_indices]

    if not remaining_indices:
        print(f"  No remaining pads to route in Phase 3")
        return main_result

    print(f"  Multi-point net Phase 3: routing {len(remaining_indices)} remaining pads")

    router = GridRouter(via_cost=config.via_cost * 1000, h_weight=config.heuristic_weight, turn_cost=config.turn_cost)
    total_iterations = 0

    def get_endpoint_info(endpoint):
        """Get x, y, layers from Pad object or dict."""
        if hasattr(endpoint, 'global_x'):
            return endpoint.global_x, endpoint.global_y, endpoint.layers
        elif isinstance(endpoint, dict):
            return endpoint['x'], endpoint['y'], endpoint.get('layers', [endpoint.get('layer')])
        else:
            raise ValueError(f"Unknown endpoint type: {type(endpoint)}")

    # Get main pad positions for filtering tap segments
    # Taps should connect to segments on the tap pad's side (past the meander)
    main_pad_a = main_result.get('main_pad_a')
    main_pad_b = main_result.get('main_pad_b')

    for pad_idx in remaining_indices:
        pad = pad_info[pad_idx]
        target_endpoint = pads[pad_idx]

        # Get endpoint coordinates and layers
        target_x, target_y, target_layers = get_endpoint_info(target_endpoint)

        # Filter segments to only those reachable from closer main pad before hitting meanders
        # This ensures taps connect past the meander, not into it
        segments_for_tap = all_segments
        original_segments = main_result.get('original_segments', [])

        if main_pad_a and main_pad_b and original_segments:
            # Find which main pad is closer to this tap pad
            dist_a = abs(target_x - main_pad_a[0]) + abs(target_y - main_pad_a[1])
            dist_b = abs(target_x - main_pad_b[0]) + abs(target_y - main_pad_b[1])
            closer_main_pad = main_pad_a if dist_a < dist_b else main_pad_b

            # Build set of original segment signatures for comparison
            def seg_sig(seg):
                return (round(seg.start_x, 3), round(seg.start_y, 3),
                        round(seg.end_x, 3), round(seg.end_y, 3), seg.layer)
            original_sigs = set(seg_sig(s) for s in original_segments)

            # Trace connected segments from closer main pad, stopping at meander segments
            # Build adjacency: position -> list of segments touching that position
            pos_to_segs = {}
            for seg in all_segments:
                for pos in [(round(seg.start_x, 3), round(seg.start_y, 3)),
                            (round(seg.end_x, 3), round(seg.end_y, 3))]:
                    if pos not in pos_to_segs:
                        pos_to_segs[pos] = []
                    pos_to_segs[pos].append(seg)

            # Start from closer main pad position
            start_pos = (round(closer_main_pad[0], 3), round(closer_main_pad[1], 3))

            # BFS to find all original segments reachable from start without crossing meanders
            visited_segs = set()
            visited_pos = set()
            queue = [start_pos]
            valid_segments = []

            while queue:
                pos = queue.pop(0)
                if pos in visited_pos:
                    continue
                visited_pos.add(pos)

                for seg in pos_to_segs.get(pos, []):
                    if id(seg) in visited_segs:
                        continue

                    # Check if this segment is an original (non-meander) segment
                    if seg_sig(seg) not in original_sigs:
                        # This is a meander segment - don't traverse through it
                        continue

                    visited_segs.add(id(seg))
                    valid_segments.append(seg)

                    # Add the other endpoint to continue traversal
                    other_pos = (round(seg.end_x, 3), round(seg.end_y, 3))
                    if other_pos == pos:
                        other_pos = (round(seg.start_x, 3), round(seg.start_y, 3))
                    queue.append(other_pos)

            if valid_segments:
                segments_for_tap = valid_segments
                print(f"  Traced {len(valid_segments)}/{len(all_segments)} segments from main pad (before meander)")

        # Find closest tap point on filtered segments
        tap_result = find_closest_point_on_segments(
            segments_for_tap,
            target_x,
            target_y,
            target_layers
        )

        if tap_result is None:
            print(f"  Could not find tap point for pad {pad_idx}")
            continue

        tap_x, tap_y, tap_layer, tap_dist = tap_result
        tap_layer_idx = layer_names.index(tap_layer) if tap_layer in layer_names else 0

        # Convert tap point to grid
        tap_gx, tap_gy = coord.to_grid(tap_x, tap_y)

        # Build source (tap point) and target (pad)
        sources = [(tap_gx, tap_gy, tap_layer_idx)]
        targets = [(pad[0], pad[1], pad[2])]

        # Mark source/target cells
        for gx, gy, layer in sources + targets:
            obstacles.add_source_target_cell(gx, gy, layer)

        # Add allowed cells around tap point and target to escape blocked areas
        # Use a smaller radius to avoid overriding blocking from other nets' segments
        allow_radius = 5
        for gx, gy, _ in sources + targets:
            for dx in range(-allow_radius, allow_radius + 1):
                for dy in range(-allow_radius, allow_radius + 1):
                    obstacles.add_allowed_cell(gx + dx, gy + dy)

        # Route to this pad
        path, iterations = router.route_multi(obstacles, sources, targets, config.max_iterations)
        total_iterations += iterations

        if path is None:
            print(f"  Failed to route to pad {pad_idx} after {iterations} iterations")
            continue

        print(f"  Routed to pad {pad_idx} in {iterations} iterations")

        # Convert path to segments/vias
        segments, vias = _path_to_segments_vias(
            path, coord, layer_names, net_id, config,
            (tap_x, tap_y, tap_layer),  # start_original (tap point)
            (pad[3], pad[4], layer_names[pad[2]])  # end_original (pad)
        )
        all_segments.extend(segments)
        all_vias.extend(vias)

    print(f"  Phase 3 routing complete: {len(all_segments)} total segments, {len(all_vias)} total vias")

    # Update result - preserve original fields, update segments/vias
    updated_result = dict(main_result)
    updated_result['new_segments'] = all_segments
    updated_result['new_vias'] = all_vias
    updated_result['iterations'] = main_result['iterations'] + total_iterations

    return updated_result


def _path_to_segments_vias(
    path: List[Tuple[int, int, int]],
    coord: GridCoord,
    layer_names: List[str],
    net_id: int,
    config: GridRouteConfig,
    start_original: Tuple[float, float, str],
    end_original: Tuple[float, float, str]
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
        if abs(orig_x - first_grid_x) > 0.001 or abs(orig_y - first_grid_y) > 0.001:
            seg = Segment(
                start_x=orig_x, start_y=orig_y,
                end_x=first_grid_x, end_y=first_grid_y,
                width=config.track_width,
                layer=orig_layer,
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
            via = Via(
                x=x1, y=y1,
                size=config.via_size,
                drill=config.via_drill,
                layers=[layer_names[layer1], layer_names[layer2]],
                net_id=net_id
            )
            vias.append(via)
        else:
            if (x1, y1) != (x2, y2):
                seg = Segment(
                    start_x=x1, start_y=y1,
                    end_x=x2, end_y=y2,
                    width=config.track_width,
                    layer=layer_names[layer1],
                    net_id=net_id
                )
                segments.append(seg)

    # Add connecting segment from last path point to original end if needed
    if end_original:
        last_grid_x, last_grid_y = coord.to_float(path_end[0], path_end[1])
        orig_x, orig_y, orig_layer = end_original
        if abs(orig_x - last_grid_x) > 0.001 or abs(orig_y - last_grid_y) > 0.001:
            seg = Segment(
                start_x=last_grid_x, start_y=last_grid_y,
                end_x=orig_x, end_y=orig_y,
                width=config.track_width,
                layer=orig_layer,
                net_id=net_id
            )
            segments.append(seg)

    return segments, vias

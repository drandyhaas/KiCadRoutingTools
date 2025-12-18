"""
Batch PCB Router using Rust-accelerated A* - Routes multiple nets sequentially.

Usage:
    python route.py input.kicad_pcb output.kicad_pcb net1 net2 net3 ...

Requires the Rust router module. Build it with:
    cd rust_router && cargo build --release
    cp target/release/grid_router.dll grid_router.pyd  # Windows
    cp target/release/libgrid_router.so grid_router.so  # Linux
"""

import sys
import os
import time
import fnmatch
from typing import List, Optional, Tuple, Dict

from kicad_parser import (
    parse_kicad_pcb, PCBData, Pad,
    auto_detect_bga_exclusion_zones, find_components_by_type
)
from kicad_writer import (
    generate_segment_sexpr, generate_via_sexpr, generate_gr_line_sexpr,
    swap_segment_nets_at_positions, swap_via_nets_at_positions, swap_pad_nets_in_content
)

# Import from refactored modules
from routing_config import GridRouteConfig, GridCoord, DiffPair
from routing_utils import (
    find_differential_pairs, get_all_unrouted_net_ids, get_stub_endpoints,
    compute_mps_net_ordering, add_route_to_pcb_data,
    find_pad_nearest_to_position, find_connected_segment_positions
)
from obstacle_map import (
    build_base_obstacle_map, add_net_stubs_as_obstacles, add_net_pads_as_obstacles,
    add_net_vias_as_obstacles, add_same_net_via_clearance, add_stub_proximity_costs,
    build_base_obstacle_map_with_vis, add_net_obstacles_with_vis, get_net_bounds,
    VisualizationData, add_connector_region_via_blocking
)
from single_ended_routing import route_net, route_net_with_obstacles, route_net_with_visualization
from diff_pair_routing import route_diff_pair_with_obstacles, get_diff_pair_connector_regions
import re

# Add rust_router directory to path for importing the compiled module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))

# Import Rust router
try:
    import grid_router
    from grid_router import GridObstacleMap, GridRouter
    version = getattr(grid_router, '__version__', 'unknown')
    print(f"Using Rust router v{version}")
except ImportError as e:
    print("ERROR: Rust router module not found!")
    print("Build it with:")
    print("  cd rust_router && cargo build --release")
    print("  cp target/release/grid_router.dll grid_router.pyd  # Windows")
    print("  cp target/release/libgrid_router.so grid_router.so  # Linux")
    sys.exit(1)


def find_pad_at_position(pcb_data: PCBData, x: float, y: float, tolerance: float = 0.01) -> Optional[Pad]:
    """Find a pad at the given position within tolerance."""
    for pads in pcb_data.pads_by_net.values():
        for pad in pads:
            if abs(pad.global_x - x) < tolerance and abs(pad.global_y - y) < tolerance:
                return pad
    return None


def batch_route(input_file: str, output_file: str, net_names: List[str],
                layers: List[str] = None,
                bga_exclusion_zones: Optional[List[Tuple[float, float, float, float]]] = None,
                direction_order: str = None,
                ordering_strategy: str = "inside_out",
                disable_bga_zones: bool = False,
                track_width: float = 0.1,
                clearance: float = 0.1,
                via_size: float = 0.3,
                via_drill: float = 0.2,
                grid_step: float = 0.1,
                via_cost: int = 25,
                max_iterations: int = 200000,
                heuristic_weight: float = 1.5,
                stub_proximity_radius: float = 1.0,
                stub_proximity_cost: float = 3.0,
                diff_pair_patterns: Optional[List[str]] = None,
                diff_pair_gap: float = 0.1,
                min_diff_pair_centerline_setback: float = 0.6,
                max_diff_pair_centerline_setback: float = 5.0,
                diff_pair_turn_length: float = 0.3,
                min_turning_radius: float = 0.4,
                debug_lines: bool = False,
                fix_polarity: bool = False,
                vis_callback=None) -> Tuple[int, int, float]:
    """
    Route multiple nets using the Rust router.

    Args:
        input_file: Path to input KiCad PCB file
        output_file: Path to output KiCad PCB file
        net_names: List of net names to route
        layers: List of copper layers to route on (must be specified - cannot auto-detect
                which layers are ground planes vs signal layers)
        bga_exclusion_zones: Optional list of BGA exclusion zones (auto-detected if None)
        direction_order: Direction search order - "forward", "backwards", or "random"
                        (None = use GridRouteConfig default)
        ordering_strategy: Net ordering strategy:
            - "inside_out": Sort BGA nets by distance from BGA center (default)
            - "mps": Use Maximum Planar Subset algorithm to minimize crossing conflicts
            - "original": Keep nets in original order
        track_width: Track width in mm (default: 0.1)
        clearance: Clearance between tracks in mm (default: 0.1)
        via_size: Via outer diameter in mm (default: 0.3)
        via_drill: Via drill size in mm (default: 0.2)
        grid_step: Grid resolution in mm (default: 0.1)
        via_cost: Penalty for placing a via in grid steps (default: 25, doubled for diff pairs)
        max_iterations: Max A* iterations before giving up (default: 200000)
        heuristic_weight: A* heuristic weight, higher=faster but less optimal (default: 1.5)
        stub_proximity_radius: Radius around stubs to penalize in mm (default: 1.0)
        stub_proximity_cost: Cost penalty near stubs in mm equivalent (default: 3.0)
        diff_pair_patterns: Glob patterns for nets to route as differential pairs
        diff_pair_gap: Gap between P and N traces in differential pairs (default: 0.1mm)
        debug_lines: Output debug geometry on User.2/3/8/9 layers
        vis_callback: Optional visualization callback (implements VisualizationCallback protocol)

    Returns:
        (successful_count, failed_count, total_time)
    """
    visualize = vis_callback is not None
    print(f"Loading {input_file}...")
    pcb_data = parse_kicad_pcb(input_file)

    # Layers must be specified - we can't auto-detect which are ground planes
    if layers is None:
        layers = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']  # Default 4-layer signal stack
    print(f"Using {len(layers)} routing layers: {layers}")

    # Auto-detect BGA exclusion zones if not specified
    if disable_bga_zones:
        bga_exclusion_zones = []
        print("BGA exclusion zones disabled")
    elif bga_exclusion_zones is None:
        bga_exclusion_zones = auto_detect_bga_exclusion_zones(pcb_data, margin=0.5)
        if bga_exclusion_zones:
            bga_components = find_components_by_type(pcb_data, 'BGA')
            print(f"Auto-detected {len(bga_exclusion_zones)} BGA exclusion zone(s):")
            for i, (fp, zone) in enumerate(zip(bga_components, bga_exclusion_zones)):
                print(f"  {fp.reference}: ({zone[0]:.1f}, {zone[1]:.1f}) to ({zone[2]:.1f}, {zone[3]:.1f})")
        else:
            print("No BGA components detected - no exclusion zones needed")

    config_kwargs = dict(
        track_width=track_width,
        clearance=clearance,
        via_size=via_size,
        via_drill=via_drill,
        grid_step=grid_step,
        via_cost=via_cost,
        layers=layers,
        max_iterations=max_iterations,
        heuristic_weight=heuristic_weight,
        bga_exclusion_zones=bga_exclusion_zones,
        stub_proximity_radius=stub_proximity_radius,
        stub_proximity_cost=stub_proximity_cost,
        diff_pair_gap=diff_pair_gap,
        min_diff_pair_centerline_setback=min_diff_pair_centerline_setback,
        max_diff_pair_centerline_setback=max_diff_pair_centerline_setback,
        diff_pair_turn_length=diff_pair_turn_length,
        min_turning_radius=min_turning_radius,
        debug_lines=debug_lines,
        fix_polarity=fix_polarity,
    )
    if direction_order is not None:
        config_kwargs['direction_order'] = direction_order
    config = GridRouteConfig(**config_kwargs)

    # Find differential pairs if patterns provided
    diff_pairs: Dict[str, DiffPair] = {}
    diff_pair_net_ids = set()  # Net IDs that are part of differential pairs
    if diff_pair_patterns:
        diff_pairs = find_differential_pairs(pcb_data, diff_pair_patterns)
        if diff_pairs:
            print(f"\nFound {len(diff_pairs)} differential pairs:")
            for pair_name, pair in list(diff_pairs.items())[:5]:
                print(f"  {pair_name}: {pair.p_net_name} / {pair.n_net_name}")
            if len(diff_pairs) > 5:
                print(f"  ... and {len(diff_pairs) - 5} more")
            # Track which net IDs are part of pairs
            for pair in diff_pairs.values():
                diff_pair_net_ids.add(pair.p_net_id)
                diff_pair_net_ids.add(pair.n_net_id)
        else:
            print(f"\nNo differential pairs found matching patterns: {diff_pair_patterns}")

    # Find net IDs - check both pcb.nets and pads_by_net
    net_ids = []
    for net_name in net_names:
        net_id = None
        # First check pcb.nets
        for nid, net in pcb_data.nets.items():
            if net.name == net_name:
                net_id = nid
                break
        # If not found, check pads_by_net (for nets not in pcb.nets)
        if net_id is None:
            for nid, pads in pcb_data.pads_by_net.items():
                for pad in pads:
                    if pad.net_name == net_name:
                        net_id = nid
                        break
                if net_id is not None:
                    break
        if net_id is None:
            print(f"Warning: Net '{net_name}' not found, skipping")
        else:
            net_ids.append((net_name, net_id))

    if not net_ids:
        print("No valid nets to route!")
        return 0, 0, 0.0

    # Apply net ordering strategy
    if ordering_strategy == "mps":
        # Use Maximum Planar Subset algorithm to minimize crossing conflicts
        print(f"\nUsing MPS ordering strategy...")
        all_net_ids = [nid for _, nid in net_ids]
        ordered_ids = compute_mps_net_ordering(pcb_data, all_net_ids)
        # Rebuild net_ids in the new order
        id_to_name = {nid: name for name, nid in net_ids}
        net_ids = [(id_to_name[nid], nid) for nid in ordered_ids if nid in id_to_name]

    elif ordering_strategy == "inside_out" and bga_exclusion_zones:
        # Sort nets inside-out from BGA center(s) for better escape routing
        # Only applies to nets that have pads inside a BGA zone
        def pad_in_bga_zone(pad):
            """Check if a pad is inside any BGA zone."""
            for zone in bga_exclusion_zones:
                if zone[0] <= pad.global_x <= zone[2] and zone[1] <= pad.global_y <= zone[3]:
                    return True
            return False

        def get_min_distance_to_bga_center(net_id):
            """Get minimum distance from any BGA pad of this net to its BGA center."""
            pads = pcb_data.pads_by_net.get(net_id, [])
            if not pads:
                return float('inf')

            min_dist = float('inf')
            for zone in bga_exclusion_zones:
                center_x = (zone[0] + zone[2]) / 2
                center_y = (zone[1] + zone[3]) / 2
                for pad in pads:
                    # Only consider pads that are inside this BGA zone
                    if zone[0] <= pad.global_x <= zone[2] and zone[1] <= pad.global_y <= zone[3]:
                        dist = ((pad.global_x - center_x) ** 2 + (pad.global_y - center_y) ** 2) ** 0.5
                        min_dist = min(min_dist, dist)
            return min_dist

        # Separate BGA nets from non-BGA nets
        bga_nets = []
        non_bga_nets = []
        for net_name, net_id in net_ids:
            pads = pcb_data.pads_by_net.get(net_id, [])
            has_bga_pad = any(pad_in_bga_zone(pad) for pad in pads)
            if has_bga_pad:
                bga_nets.append((net_name, net_id))
            else:
                non_bga_nets.append((net_name, net_id))

        # Sort BGA nets inside-out, keep non-BGA nets in original order
        bga_nets.sort(key=lambda x: get_min_distance_to_bga_center(x[1]))
        net_ids = bga_nets + non_bga_nets

        if bga_nets:
            print(f"\nSorted {len(bga_nets)} BGA nets inside-out ({len(non_bga_nets)} non-BGA nets unchanged)")

    elif ordering_strategy == "original":
        print(f"\nUsing original net order (no sorting)")

    # Separate single-ended nets from differential pairs
    single_ended_nets = []
    diff_pair_ids_to_route = []  # (pair_name, pair) tuples
    processed_pair_net_ids = set()

    for net_name, net_id in net_ids:
        if net_id in diff_pair_net_ids and net_id not in processed_pair_net_ids:
            # Find the differential pair this net belongs to
            for pair_name, pair in diff_pairs.items():
                if pair.p_net_id == net_id or pair.n_net_id == net_id:
                    diff_pair_ids_to_route.append((pair_name, pair))
                    processed_pair_net_ids.add(pair.p_net_id)
                    processed_pair_net_ids.add(pair.n_net_id)
                    break
        elif net_id not in diff_pair_net_ids:
            single_ended_nets.append((net_name, net_id))

    total_routes = len(single_ended_nets) + len(diff_pair_ids_to_route)
    print(f"\nRouting {total_routes} items ({len(single_ended_nets)} single-ended nets, {len(diff_pair_ids_to_route)} differential pairs)...")
    print("=" * 60)

    results = []
    pad_swaps = []  # List of (pad1, pad2) tuples for nets that need swapping
    successful = 0
    failed = 0
    total_time = 0
    total_iterations = 0

    # Build base obstacle map once (excludes all nets we're routing)
    all_net_ids_to_route = [nid for _, nid in net_ids]
    print("Building base obstacle map...")
    base_start = time.time()

    # Use visualization-aware building if callback is provided
    base_vis_data = None
    if visualize:
        base_obstacles, base_vis_data = build_base_obstacle_map_with_vis(pcb_data, config, all_net_ids_to_route)
        # Set bounds for visualization
        base_vis_data.bounds = get_net_bounds(pcb_data, all_net_ids_to_route, padding=5.0)
    else:
        base_obstacles = build_base_obstacle_map(pcb_data, config, all_net_ids_to_route)

    base_elapsed = time.time() - base_start
    print(f"Base obstacle map built in {base_elapsed:.2f}s")

    # Build separate base obstacle map with extra clearance for diff pair centerline routing
    # Extra clearance = spacing from centerline to each track (at least one track width)
    diff_pair_extra_clearance = (config.track_width + config.diff_pair_gap) / 2 + config.track_width
    print(f"Building diff pair obstacle map (extra clearance: {diff_pair_extra_clearance:.3f}mm)...")
    dp_base_start = time.time()
    diff_pair_base_obstacles = build_base_obstacle_map(pcb_data, config, all_net_ids_to_route, diff_pair_extra_clearance)
    dp_base_elapsed = time.time() - dp_base_start
    print(f"Diff pair obstacle map built in {dp_base_elapsed:.2f}s")

    # Block connector regions for ALL diff pairs upfront
    # Vias span all layers, so we must block connector regions before any routing starts
    # to prevent one pair's vias from interfering with another pair's connector segments
    if diff_pair_ids_to_route:
        print(f"Blocking connector regions for {len(diff_pair_ids_to_route)} diff pair(s)...")
        for pair_name, pair in diff_pair_ids_to_route:
            connector_info = get_diff_pair_connector_regions(pcb_data, pair, config)
            if connector_info:
                # Block source connector region
                add_connector_region_via_blocking(
                    diff_pair_base_obstacles,
                    connector_info['src_center'][0], connector_info['src_center'][1],
                    connector_info['src_dir'][0], connector_info['src_dir'][1],
                    connector_info['src_setback'], connector_info['spacing_mm'], config
                )
                # Block target connector region
                add_connector_region_via_blocking(
                    diff_pair_base_obstacles,
                    connector_info['tgt_center'][0], connector_info['tgt_center'][1],
                    connector_info['tgt_dir'][0], connector_info['tgt_dir'][1],
                    connector_info['tgt_setback'], connector_info['spacing_mm'], config
                )

    # Notify visualization callback that routing is starting
    if visualize:
        vis_callback.on_routing_start(total_routes, layers, grid_step)

    # Track which nets have been routed (their segments/vias are now in pcb_data)
    routed_net_ids = []
    remaining_net_ids = list(all_net_ids_to_route)

    # Get ALL unrouted nets in the PCB for stub proximity costs
    # (not just the ones we're routing in this batch)
    all_unrouted_net_ids = set(get_all_unrouted_net_ids(pcb_data))
    print(f"Found {len(all_unrouted_net_ids)} unrouted nets in PCB for stub proximity")

    route_index = 0

    # Route differential pairs first (they're more constrained)
    for pair_name, pair in diff_pair_ids_to_route:
        route_index += 1
        print(f"\n[{route_index}/{total_routes}] Routing diff pair {pair_name}")
        print(f"  P: {pair.p_net_name} (id={pair.p_net_id})")
        print(f"  N: {pair.n_net_name} (id={pair.n_net_id})")
        print("-" * 40)

        start_time = time.time()

        # Clone diff pair base obstacles (with extra clearance for centerline routing)
        obstacles = diff_pair_base_obstacles.clone()

        # Add previously routed nets' segments/vias/pads as obstacles (with extra clearance)
        for routed_id in routed_net_ids:
            add_net_stubs_as_obstacles(obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
            add_net_vias_as_obstacles(obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
            add_net_pads_as_obstacles(obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)

        # Add other unrouted nets' stubs, vias, and pads as obstacles (excluding both P and N)
        other_unrouted = [nid for nid in remaining_net_ids
                         if nid != pair.p_net_id and nid != pair.n_net_id]
        for other_net_id in other_unrouted:
            add_net_stubs_as_obstacles(obstacles, pcb_data, other_net_id, config, diff_pair_extra_clearance)
            add_net_vias_as_obstacles(obstacles, pcb_data, other_net_id, config, diff_pair_extra_clearance)
            add_net_pads_as_obstacles(obstacles, pcb_data, other_net_id, config, diff_pair_extra_clearance)

        # Add stub proximity costs for ALL unrouted nets in PCB (not just current batch)
        stub_proximity_net_ids = [nid for nid in all_unrouted_net_ids
                                   if nid != pair.p_net_id and nid != pair.n_net_id
                                   and nid not in routed_net_ids]
        unrouted_stubs = get_stub_endpoints(pcb_data, stub_proximity_net_ids)
        if unrouted_stubs:
            add_stub_proximity_costs(obstacles, unrouted_stubs, config)

        # Add same-net via clearance for both P and N
        add_same_net_via_clearance(obstacles, pcb_data, pair.p_net_id, config)
        add_same_net_via_clearance(obstacles, pcb_data, pair.n_net_id, config)

        # Route the differential pair
        # Pass both diff pair obstacles (with extra clearance) and base obstacles (for extension routing)
        # Also pass unrouted stubs for finding clear extension endpoints
        result = route_diff_pair_with_obstacles(pcb_data, pair, config, obstacles, base_obstacles, unrouted_stubs)
        elapsed = time.time() - start_time
        total_time += elapsed

        if result and not result.get('failed'):
            print(f"  SUCCESS: {len(result['new_segments'])} segments, {len(result['new_vias'])} vias, {result['iterations']} iterations ({elapsed:.2f}s)")
            results.append(result)
            successful += 1
            total_iterations += result['iterations']

            # Check if polarity was fixed - need to swap target pad and stub nets
            # Do this BEFORE add_route_to_pcb_data so collapse_appendices sees correct net IDs
            if result.get('polarity_fixed') and result.get('swap_target_pads'):
                swap_info = result['swap_target_pads']
                p_pos = swap_info['p_pos']  # Original P target stub position
                n_pos = swap_info['n_pos']  # Original N target stub position
                p_net_id = swap_info['p_net_id']
                n_net_id = swap_info['n_net_id']

                # Swap stub net IDs in pcb_data.segments so collapse_appendices works correctly
                # Find segments connected to each stub position and swap their net IDs
                p_stub_positions = find_connected_segment_positions(pcb_data, p_pos[0], p_pos[1], p_net_id)
                n_stub_positions = find_connected_segment_positions(pcb_data, n_pos[0], n_pos[1], n_net_id)
                for seg in pcb_data.segments:
                    seg_start = (round(seg.start_x, 2), round(seg.start_y, 2))
                    seg_end = (round(seg.end_x, 2), round(seg.end_y, 2))
                    if seg.net_id == p_net_id and (seg_start in p_stub_positions or seg_end in p_stub_positions):
                        seg.net_id = n_net_id
                    elif seg.net_id == n_net_id and (seg_start in n_stub_positions or seg_end in n_stub_positions):
                        seg.net_id = p_net_id

                # Also swap via net IDs at stub positions
                for via in pcb_data.vias:
                    via_pos = (round(via.x, 2), round(via.y, 2))
                    if via.net_id == p_net_id and via_pos in p_stub_positions:
                        via.net_id = n_net_id
                    elif via.net_id == n_net_id and via_pos in n_stub_positions:
                        via.net_id = p_net_id

                # Find the target pads by net ID and proximity to stub positions
                pad_p = find_pad_nearest_to_position(pcb_data, p_net_id, p_pos[0], p_pos[1])
                pad_n = find_pad_nearest_to_position(pcb_data, n_net_id, n_pos[0], n_pos[1])

                if pad_p and pad_n:
                    pad_swaps.append({
                        'pad_p': pad_p,
                        'pad_n': pad_n,
                        'p_net_id': p_net_id,
                        'n_net_id': n_net_id,
                        # Store positions now before pcb_data net IDs are swapped
                        'p_stub_positions': p_stub_positions,
                        'n_stub_positions': n_stub_positions,
                    })
                    print(f"  Polarity fixed: will swap nets of {pad_p.component_ref}:{pad_p.pad_number} <-> {pad_n.component_ref}:{pad_n.pad_number}")
                else:
                    print(f"  WARNING: Could not find target pads to swap for polarity fix")
                    if not pad_p:
                        print(f"    Missing P pad (net {p_net_id}) near {p_pos}")
                    if not pad_n:
                        print(f"    Missing N pad (net {n_net_id}) near {n_pos}")

            add_route_to_pcb_data(pcb_data, result, debug_lines=config.debug_lines)

            if pair.p_net_id in remaining_net_ids:
                remaining_net_ids.remove(pair.p_net_id)
            if pair.n_net_id in remaining_net_ids:
                remaining_net_ids.remove(pair.n_net_id)
            routed_net_ids.append(pair.p_net_id)
            routed_net_ids.append(pair.n_net_id)
        else:
            iterations = result['iterations'] if result else 0
            print(f"  FAILED: Could not find route ({elapsed:.2f}s)")
            failed += 1
            total_iterations += iterations

    # Route single-ended nets
    user_quit = False
    for net_name, net_id in single_ended_nets:
        if user_quit:
            break

        route_index += 1
        print(f"\n[{route_index}/{total_routes}] Routing {net_name} (id={net_id})")
        print("-" * 40)

        start_time = time.time()

        # Clone base obstacles
        obstacles = base_obstacles.clone()

        # Build visualization data if needed
        vis_data = None
        if visualize:
            # Clone the base vis data
            vis_data = VisualizationData(
                blocked_cells=[set(s) for s in base_vis_data.blocked_cells],
                blocked_vias=set(base_vis_data.blocked_vias),
                bga_zones_grid=list(base_vis_data.bga_zones_grid),
                bounds=base_vis_data.bounds
            )

        # Add previously routed nets' segments/vias/pads as obstacles (from pcb_data)
        for routed_id in routed_net_ids:
            if visualize:
                add_net_obstacles_with_vis(obstacles, pcb_data, routed_id, config, 0.0,
                                            vis_data.blocked_cells, vis_data.blocked_vias)
            else:
                add_net_stubs_as_obstacles(obstacles, pcb_data, routed_id, config)
                add_net_vias_as_obstacles(obstacles, pcb_data, routed_id, config)
                add_net_pads_as_obstacles(obstacles, pcb_data, routed_id, config)

        # Add other unrouted nets' stubs, vias, and pads as obstacles (not the current net)
        other_unrouted = [nid for nid in remaining_net_ids if nid != net_id]
        for other_net_id in other_unrouted:
            if visualize:
                add_net_obstacles_with_vis(obstacles, pcb_data, other_net_id, config, 0.0,
                                            vis_data.blocked_cells, vis_data.blocked_vias)
            else:
                add_net_stubs_as_obstacles(obstacles, pcb_data, other_net_id, config)
                add_net_vias_as_obstacles(obstacles, pcb_data, other_net_id, config)
                add_net_pads_as_obstacles(obstacles, pcb_data, other_net_id, config)

        # Add stub proximity costs for ALL unrouted nets in PCB (not just current batch)
        stub_proximity_net_ids = [nid for nid in all_unrouted_net_ids
                                   if nid != net_id and nid not in routed_net_ids]
        unrouted_stubs = get_stub_endpoints(pcb_data, stub_proximity_net_ids)
        if unrouted_stubs:
            add_stub_proximity_costs(obstacles, unrouted_stubs, config)

        # Add same-net via clearance blocking (for DRC - vias can't be too close even on same net)
        add_same_net_via_clearance(obstacles, pcb_data, net_id, config)

        # Route the net using the prepared obstacles
        if visualize:
            # Get source/target grid coords for visualization
            from routing_utils import get_net_endpoints
            sources, targets, _ = get_net_endpoints(pcb_data, net_id, config)
            sources_grid = [(s[0], s[1], s[2]) for s in sources] if sources else []
            targets_grid = [(t[0], t[1], t[2]) for t in targets] if targets else []

            # Notify visualizer that net routing is starting
            vis_callback.on_net_start(net_name, route_index, net_id,
                                       sources_grid, targets_grid, obstacles, vis_data)

            # Route with visualization
            result = route_net_with_visualization(pcb_data, net_id, config, obstacles, vis_callback)

            # Notify visualizer that net routing is complete
            if result is None:
                # User quit during routing
                user_quit = True
                break

            path = result.get('path') if result and not result.get('failed') else None
            direction = result.get('direction', 'forward') if result else 'forward'
            iterations = result.get('iterations', 0) if result else 0
            success = result is not None and not result.get('failed')

            if not vis_callback.on_net_complete(net_name, success, path, iterations, direction):
                user_quit = True
                break
        else:
            result = route_net_with_obstacles(pcb_data, net_id, config, obstacles)

        elapsed = time.time() - start_time
        total_time += elapsed

        if result and not result.get('failed'):
            print(f"  SUCCESS: {len(result['new_segments'])} segments, {len(result['new_vias'])} vias, {result['iterations']} iterations ({elapsed:.2f}s)")
            results.append(result)
            successful += 1
            total_iterations += result['iterations']
            add_route_to_pcb_data(pcb_data, result, debug_lines=config.debug_lines)
            remaining_net_ids.remove(net_id)
            routed_net_ids.append(net_id)
        else:
            iterations = result['iterations'] if result else 0
            print(f"  FAILED: Could not find route ({elapsed:.2f}s)")
            failed += 1
            total_iterations += iterations

    # Notify visualization callback that all routing is complete
    if visualize:
        vis_callback.on_routing_complete(successful, failed, total_iterations)

    print("\n" + "=" * 60)
    print(f"Routing complete: {successful} successful, {failed} failed")
    print(f"Total time: {total_time:.2f}s")
    print(f"Total iterations: {total_iterations}")

    if results:
        print(f"\nWriting output to {output_file}...")
        with open(input_file, 'r', encoding='utf-8') as f:
            content = f.read()

        # Apply pad and stub net swaps for polarity fixes
        if pad_swaps:
            print(f"Applying {len(pad_swaps)} polarity fix(es) (swapping target pads and stubs)...")
            for swap in pad_swaps:
                pad_p = swap['pad_p']
                pad_n = swap['pad_n']
                p_net_id = swap['p_net_id']
                n_net_id = swap['n_net_id']

                # Swap pad nets
                content = swap_pad_nets_in_content(content, pad_p, pad_n)
                print(f"  Pads: {pad_p.component_ref}:{pad_p.pad_number} <-> {pad_n.component_ref}:{pad_n.pad_number}")

                # Use pre-computed stub positions (saved before pcb_data was modified)
                p_positions = swap.get('p_stub_positions')
                n_positions = swap.get('n_stub_positions')

                # Fallback to computing if not stored (for backward compatibility)
                if p_positions is None:
                    p_positions = find_connected_segment_positions(pcb_data, pad_p.global_x, pad_p.global_y, p_net_id)
                if n_positions is None:
                    n_positions = find_connected_segment_positions(pcb_data, pad_n.global_x, pad_n.global_y, n_net_id)

                # Swap entire stub chains - all segments connected to each pad
                content, p_seg_count = swap_segment_nets_at_positions(content, p_positions, p_net_id, n_net_id)
                content, n_seg_count = swap_segment_nets_at_positions(content, n_positions, n_net_id, p_net_id)

                # Also swap vias at the same positions
                content, p_via_count = swap_via_nets_at_positions(content, p_positions, p_net_id, n_net_id)
                content, n_via_count = swap_via_nets_at_positions(content, n_positions, n_net_id, p_net_id)
                print(f"  Stubs: swapped {p_seg_count}+{n_seg_count} segments, {p_via_count}+{n_via_count} vias")

        routing_text = ""
        for result in results:
            for seg in result['new_segments']:
                routing_text += generate_segment_sexpr(
                    (seg.start_x, seg.start_y), (seg.end_x, seg.end_y),
                    seg.width, seg.layer, seg.net_id
                ) + "\n"
            for via in result['new_vias']:
                routing_text += generate_via_sexpr(
                    via.x, via.y, via.size, via.drill,
                    via.layers, via.net_id
                ) + "\n"

        # Add debug paths if enabled (using gr_line for User layers)
        if debug_lines:
            print("Adding debug paths to User.2 (turns), User.3 (connectors), User.4 (stub dirs), User.8 (simplified), User.9 (raw A*)")
            for result in results:
                # Raw A* path on User.9
                raw_path = result.get('raw_astar_path', [])
                if len(raw_path) >= 2:
                    for i in range(len(raw_path) - 1):
                        x1, y1, _ = raw_path[i]
                        x2, y2, _ = raw_path[i + 1]
                        if abs(x1 - x2) > 0.001 or abs(y1 - y2) > 0.001:
                            routing_text += generate_gr_line_sexpr(
                                (x1, y1), (x2, y2),
                                0.05, "User.9"  # Thin line
                            ) + "\n"

                # Simplified path on User.8
                simplified_path = result.get('simplified_path', [])
                if len(simplified_path) >= 2:
                    for i in range(len(simplified_path) - 1):
                        x1, y1, _ = simplified_path[i]
                        x2, y2, _ = simplified_path[i + 1]
                        if abs(x1 - x2) > 0.001 or abs(y1 - y2) > 0.001:
                            routing_text += generate_gr_line_sexpr(
                                (x1, y1), (x2, y2),
                                0.15, "User.8"  # Thicker line
                            ) + "\n"

                # Turn segments on User.2
                turn_lines = result.get('debug_turn_lines', [])
                for start, end in turn_lines:
                    routing_text += generate_gr_line_sexpr(
                        start, end,
                        0.1, "User.2"
                    ) + "\n"

                # Connector segments on User.3
                connector_lines = result.get('debug_connector_lines', [])
                for start, end in connector_lines:
                    routing_text += generate_gr_line_sexpr(
                        start, end,
                        0.1, "User.3"
                    ) + "\n"

                # Stub direction arrows on User.4
                stub_arrows = result.get('debug_stub_arrows', [])
                for start, end in stub_arrows:
                    routing_text += generate_gr_line_sexpr(
                        start, end,
                        0.1, "User.4"
                    ) + "\n"

        last_paren = content.rfind(')')
        new_content = content[:last_paren] + '\n' + routing_text + '\n' + content[last_paren:]

        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(new_content)

        print(f"Successfully wrote {output_file}")

    return successful, failed, total_time


def expand_net_patterns(pcb_data: PCBData, patterns: List[str]) -> List[str]:
    """
    Expand wildcard patterns to matching net names.

    Patterns can include * and ? wildcards (fnmatch style).
    Example: "Net-(U2A-DATA_*)" matches Net-(U2A-DATA_0), Net-(U2A-DATA_1), etc.

    Returns list of unique net names in sorted order for patterns,
    preserving order of non-pattern names.
    """
    # Collect net names from both pcb.nets and pads_by_net
    all_net_names = set(net.name for net in pcb_data.nets.values())
    # Also include net names from pads (for nets not in pcb.nets)
    for pads in pcb_data.pads_by_net.values():
        for pad in pads:
            if pad.net_name:
                all_net_names.add(pad.net_name)
                break  # Only need one pad's net_name per net
    all_net_names = list(all_net_names)
    result = []
    seen = set()

    for pattern in patterns:
        if '*' in pattern or '?' in pattern:
            # It's a wildcard pattern - find all matching nets
            matches = sorted([name for name in all_net_names if fnmatch.fnmatch(name, pattern)])
            if not matches:
                print(f"Warning: Pattern '{pattern}' matched no nets")
            else:
                print(f"Pattern '{pattern}' matched {len(matches)} nets")
            for name in matches:
                if name not in seen:
                    result.append(name)
                    seen.add(name)
        else:
            # Literal net name
            if pattern not in seen:
                result.append(pattern)
                seen.add(pattern)

    return result


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Batch PCB Router - Routes multiple nets using Rust-accelerated A*",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Wildcard patterns supported:
  "Net-(U2A-DATA_*)"  - matches Net-(U2A-DATA_0), Net-(U2A-DATA_1), etc.
  "Net-(*CLK*)"       - matches any net containing CLK

Examples:
  python route.py fanout_starting_point.kicad_pcb routed.kicad_pcb "Net-(U2A-DATA_*)"
  python route.py input.kicad_pcb output.kicad_pcb "Net-(U2A-DATA_*)" --ordering mps

Differential pair routing:
  python route.py input.kicad_pcb output.kicad_pcb "*lvds*" --diff-pairs "*lvds*"

  The --diff-pairs option specifies patterns for nets that should be routed as differential pairs.
  Nets matching these patterns with _P/_N, P/N, or +/- suffixes will be routed together
  maintaining constant spacing (controlled by --diff-pair-gap).
"""
    )
    parser.add_argument("input_file", help="Input KiCad PCB file")
    parser.add_argument("output_file", help="Output KiCad PCB file")
    parser.add_argument("net_patterns", nargs="+", help="Net names or wildcard patterns to route")
    # Ordering and strategy options
    parser.add_argument("--ordering", "-o", choices=["inside_out", "mps", "original"],
                        default="mps",
                        help="Net ordering strategy: mps (default, crossing conflicts), inside_out, or original")
    parser.add_argument("--direction", "-d", choices=["forward", "backwards", "random"],
                        default=None,
                        help="Direction search order for each net route")
    parser.add_argument("--no-bga-zones", action="store_true",
                        help="Disable BGA exclusion zone detection (allows routing through BGA areas)")
    parser.add_argument("--layers", "-l", nargs="+",
                        default=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'],
                        help="Routing layers to use (default: F.Cu In1.Cu In2.Cu B.Cu)")

    # Track and via geometry
    parser.add_argument("--track-width", type=float, default=0.1,
                        help="Track width in mm (default: 0.1)")
    parser.add_argument("--clearance", type=float, default=0.1,
                        help="Clearance between tracks in mm (default: 0.1)")
    parser.add_argument("--via-size", type=float, default=0.3,
                        help="Via outer diameter in mm (default: 0.3)")
    parser.add_argument("--via-drill", type=float, default=0.2,
                        help="Via drill size in mm (default: 0.2)")

    # Router algorithm parameters
    parser.add_argument("--grid-step", type=float, default=0.1,
                        help="Grid resolution in mm (default: 0.1)")
    parser.add_argument("--via-cost", type=int, default=25,
                        help="Penalty for placing a via in grid steps (default: 25, doubled for diff pairs)")
    parser.add_argument("--max-iterations", type=int, default=200000,
                        help="Max A* iterations before giving up (default: 200000)")
    parser.add_argument("--heuristic-weight", type=float, default=1.5,
                        help="A* heuristic weight, higher=faster but less optimal (default: 1.5)")

    # Stub proximity penalty
    parser.add_argument("--stub-proximity-radius", type=float, default=1.5,
                        help="Radius around stubs to penalize routing in mm (default: 1.5)")
    parser.add_argument("--stub-proximity-cost", type=float, default=2.0,
                        help="Cost penalty near stubs in mm equivalent (default: 2.0)")

    # Differential pair routing
    parser.add_argument("--diff-pairs", "-D", nargs="+",
                        help="Glob patterns for nets to route as differential pairs (e.g., '*lvds*')")
    parser.add_argument("--diff-pair-gap", type=float, default=0.1,
                        help="Gap between P and N traces of differential pairs in mm (default: 0.1)")
    parser.add_argument("--min-diff-pair-centerline-setback", type=float, default=0.6,
                        help="Minimum distance in front of stubs to start centerline route in mm (default: 0.6)")
    parser.add_argument("--max-diff-pair-centerline-setback", type=float, default=5.0,
                        help="Maximum setback to try if minimum is blocked in mm (default: 5.0)")
    parser.add_argument("--diff-pair-turn-length", type=float, default=0.3,
                        help="Length of turn segments at start/end of diff pair routes in mm (default: 0.3)")
    parser.add_argument("--min-turning-radius", type=float, default=0.4,
                        help="Minimum turning radius for pose-based routing in mm (default: 0.4)")
    parser.add_argument("--fix-polarity", action="store_true",
                        help="Swap target pad net assignments if polarity swap is needed")

    # Debug options
    parser.add_argument("--debug-lines", action="store_true",
                        help="Output debug geometry on User.2 (turns), User.3 (connectors), User.4 (stub dirs), User.8 (simplified), User.9 (raw A*)")

    # Visualization options
    parser.add_argument("--visualize", "-V", action="store_true",
                        help="Show real-time visualization of the routing (requires pygame)")
    parser.add_argument("--auto", action="store_true",
                        help="Auto-advance to next net without waiting (with --visualize)")
    parser.add_argument("--display-time", type=float, default=0.0,
                        help="Seconds to display completed route before advancing (with --visualize --auto)")

    args = parser.parse_args()

    # Load PCB to expand wildcards
    print(f"Loading {args.input_file} to expand net patterns...")
    pcb_data = parse_kicad_pcb(args.input_file)
    net_names = expand_net_patterns(pcb_data, args.net_patterns)

    if not net_names:
        print("No nets matched the given patterns!")
        sys.exit(1)

    print(f"Routing {len(net_names)} nets: {net_names[:5]}{'...' if len(net_names) > 5 else ''}")

    # Create visualization callback if requested
    vis_callback = None
    if args.visualize:
        try:
            from pygame_visualizer.pygame_callback import create_pygame_callback
            vis_callback = create_pygame_callback(
                layers=args.layers,
                auto_advance=args.auto,
                display_time=args.display_time
            )
        except ImportError as e:
            print(f"Warning: Could not import pygame visualizer: {e}")
            print("Install pygame with: pip install pygame-ce")
            print("Continuing without visualization...")

    batch_route(args.input_file, args.output_file, net_names,
                direction_order=args.direction,
                ordering_strategy=args.ordering,
                disable_bga_zones=args.no_bga_zones,
                layers=args.layers,
                track_width=args.track_width,
                clearance=args.clearance,
                via_size=args.via_size,
                via_drill=args.via_drill,
                grid_step=args.grid_step,
                via_cost=args.via_cost,
                max_iterations=args.max_iterations,
                heuristic_weight=args.heuristic_weight,
                stub_proximity_radius=args.stub_proximity_radius,
                stub_proximity_cost=args.stub_proximity_cost,
                diff_pair_patterns=args.diff_pairs,
                diff_pair_gap=args.diff_pair_gap,
                min_diff_pair_centerline_setback=args.min_diff_pair_centerline_setback,
                max_diff_pair_centerline_setback=args.max_diff_pair_centerline_setback,
                diff_pair_turn_length=args.diff_pair_turn_length,
                min_turning_radius=args.min_turning_radius,
                debug_lines=args.debug_lines,
                fix_polarity=args.fix_polarity,
                vis_callback=vis_callback)

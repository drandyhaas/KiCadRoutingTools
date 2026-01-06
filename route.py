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
from typing import List, Optional, Tuple, Dict, Set

from kicad_parser import (
    parse_kicad_pcb, PCBData, Pad,
    auto_detect_bga_exclusion_zones, find_components_by_type
)
from kicad_writer import (
    generate_segment_sexpr, generate_via_sexpr, generate_gr_line_sexpr, generate_gr_text_sexpr,
    swap_segment_nets_at_positions, swap_via_nets_at_positions, swap_pad_nets_in_content,
    modify_segment_layers
)

# Import from refactored modules
from routing_config import GridRouteConfig, GridCoord, DiffPairNet
from routing_utils import (
    find_differential_pairs, get_all_unrouted_net_ids, get_stub_endpoints,
    compute_mps_net_ordering, add_route_to_pcb_data, remove_route_from_pcb_data,
    find_pad_nearest_to_position, find_connected_segment_positions,
    find_stub_free_ends, find_connected_groups, pos_key,
    is_edge_stub, find_pad_at_position, expand_net_patterns
)
from obstacle_map import (
    build_base_obstacle_map, add_net_stubs_as_obstacles, add_net_pads_as_obstacles,
    add_net_vias_as_obstacles, add_same_net_via_clearance, add_stub_proximity_costs,
    build_base_obstacle_map_with_vis, add_net_obstacles_with_vis, get_net_bounds,
    VisualizationData, add_connector_region_via_blocking, add_diff_pair_own_stubs_as_obstacles,
    compute_track_proximity_for_net, merge_track_proximity_costs, add_cross_layer_tracks,
    draw_exclusion_zones_debug, add_vias_list_as_obstacles, add_segments_list_as_obstacles
)
from single_ended_routing import route_net, route_net_with_obstacles, route_net_with_visualization
from diff_pair_routing import route_diff_pair_with_obstacles, get_diff_pair_connector_regions
from blocking_analysis import analyze_frontier_blocking, print_blocking_analysis, filter_rippable_blockers
from rip_up_reroute import rip_up_net, restore_net
from polarity_swap import apply_polarity_swap, get_canonical_net_id
from layer_swap_fallback import try_fallback_layer_swap, add_own_stubs_as_obstacles_for_diff_pair
from layer_swap_optimization import apply_diff_pair_layer_swaps, apply_single_ended_layer_swaps
from routing_context import (
    build_single_ended_obstacles, build_diff_pair_obstacles,
    record_single_ended_success, record_diff_pair_success,
    restore_ripped_net
)
from routing_state import RoutingState, create_routing_state
from diff_pair_loop import route_diff_pairs
from single_ended_loop import route_single_ended_nets
from reroute_loop import run_reroute_loop
from length_matching import (
    apply_length_matching_to_group, find_nets_matching_patterns, auto_group_ddr4_nets
)
import re

# ANSI color codes for terminal output
RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
RESET = '\033[0m'

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
                via_cost: int = 50,
                max_iterations: int = 200000,
                max_probe_iterations: int = 5000,
                heuristic_weight: float = 2.0,
                turn_cost: int = 1000,
                stub_proximity_radius: float = 2.0,
                stub_proximity_cost: float = 0.2,
                via_proximity_cost: float = 50.0,
                bga_proximity_radius: float = 10.0,
                bga_proximity_cost: float = 0.2,
                track_proximity_distance: float = 2.0,
                track_proximity_cost: float = 0.2,
                diff_pair_patterns: Optional[List[str]] = None,
                diff_pair_gap: float = 0.101,
                diff_pair_centerline_setback: float = None,
                min_turning_radius: float = 0.2,
                debug_lines: bool = False,
                verbose: bool = False,
                fix_polarity: bool = True,
                max_rip_up_count: int = 3,
                max_setback_angle: float = 22.5,
                enable_layer_switch: bool = True,
                crossing_layer_check: bool = True,
                can_swap_to_top_layer: bool = False,
                swappable_net_patterns: Optional[List[str]] = None,
                crossing_penalty: float = 1000.0,
                mps_unroll: bool = True,
                skip_routing: bool = False,
                routing_clearance_margin: float = 1.15,
                max_turn_angle: float = 180.0,
                gnd_via_enabled: bool = True,
                vertical_attraction_radius: float = 1.0,
                vertical_attraction_cost: float = 0.1,
                length_match_groups: Optional[List[List[str]]] = None,
                length_match_tolerance: float = 0.1,
                meander_amplitude: float = 1.0,
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
        via_cost: Penalty for placing a via in grid steps (default: 50, doubled for diff pairs)
        max_iterations: Max A* iterations before giving up (default: 200000)
        heuristic_weight: A* heuristic weight, higher=faster but less optimal (default: 2.0)
        stub_proximity_radius: Radius around stubs to penalize in mm (default: 2.0)
        stub_proximity_cost: Cost penalty near stubs in mm equivalent (default: 0.2)
        bga_proximity_radius: Radius around BGA edges to penalize in mm (default: 10.0)
        bga_proximity_cost: Cost penalty near BGA edges in mm equivalent (default: 0.2)
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
                edge_tol = zone[4] if len(zone) > 4 else 1.6
                print(f"  {fp.reference}: ({zone[0]:.1f}, {zone[1]:.1f}) to ({zone[2]:.1f}, {zone[3]:.1f}), edge_tol={edge_tol:.2f}mm")
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
        max_probe_iterations=max_probe_iterations,
        heuristic_weight=heuristic_weight,
        turn_cost=turn_cost,
        bga_exclusion_zones=bga_exclusion_zones,
        stub_proximity_radius=stub_proximity_radius,
        stub_proximity_cost=stub_proximity_cost,
        via_proximity_cost=via_proximity_cost,
        bga_proximity_radius=bga_proximity_radius,
        bga_proximity_cost=bga_proximity_cost,
        track_proximity_distance=track_proximity_distance,
        track_proximity_cost=track_proximity_cost,
        diff_pair_gap=diff_pair_gap,
        diff_pair_centerline_setback=diff_pair_centerline_setback,
        min_turning_radius=min_turning_radius,
        debug_lines=debug_lines,
        verbose=verbose,
        fix_polarity=fix_polarity,
        max_rip_up_count=max_rip_up_count,
        max_setback_angle=max_setback_angle,
        target_swap_crossing_penalty=crossing_penalty,
        crossing_layer_check=crossing_layer_check,
        routing_clearance_margin=routing_clearance_margin,
        max_turn_angle=max_turn_angle,
        gnd_via_enabled=gnd_via_enabled,
        vertical_attraction_radius=vertical_attraction_radius,
        vertical_attraction_cost=vertical_attraction_cost,
        length_match_groups=length_match_groups,
        length_match_tolerance=length_match_tolerance,
        meander_amplitude=meander_amplitude,
    )
    if direction_order is not None:
        config_kwargs['direction_order'] = direction_order
    config = GridRouteConfig(**config_kwargs)

    # Find differential pairs if patterns provided
    diff_pairs: Dict[str, DiffPairNet] = {}
    diff_pair_net_ids = set()  # Net IDs that are part of differential pairs
    if diff_pair_patterns:
        diff_pairs = find_differential_pairs(pcb_data, diff_pair_patterns)
        if diff_pairs:
            # Track which net IDs are part of pairs
            for pair in diff_pairs.values():
                diff_pair_net_ids.add(pair.p_net_id)
                diff_pair_net_ids.add(pair.n_net_id)

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

    # Track all segment layer modifications for file output
    all_segment_modifications = []
    # Track all vias added during stub layer swapping
    all_swap_vias = []
    # Track total number of layer swaps applied
    total_layer_swaps = 0

    # Identify which diff pairs we'll be routing BEFORE ordering
    # (Layer swaps must happen before MPS ordering since ordering depends on layers)
    diff_pair_ids_to_route_set = []  # (pair_name, pair) tuples - unordered
    processed_pair_net_ids_early = set()
    for net_name, net_id in net_ids:
        if net_id in diff_pair_net_ids and net_id not in processed_pair_net_ids_early:
            for pair_name, pair in diff_pairs.items():
                if pair.p_net_id == net_id or pair.n_net_id == net_id:
                    diff_pair_ids_to_route_set.append((pair_name, pair))
                    processed_pair_net_ids_early.add(pair.p_net_id)
                    processed_pair_net_ids_early.add(pair.n_net_id)
                    break

    # Apply target swaps for swappable-nets feature
    # This swaps net IDs at targets BEFORE routing, so routing sees the swapped configuration
    target_swaps: Dict[str, str] = {}  # pair_name -> swapped_target_pair_name
    target_swap_info: List[Dict] = []  # Info needed to apply swaps to output file
    boundary_debug_labels: List[Dict] = []  # Debug labels for boundary positions
    if swappable_net_patterns and diff_pair_ids_to_route_set:
        from target_swap import apply_target_swaps, generate_debug_boundary_labels
        from diff_pair_routing import get_diff_pair_endpoints

        swappable_diff_pairs = find_differential_pairs(pcb_data, swappable_net_patterns)
        # Only include pairs that are also being routed (not just in diff_pairs, but in the route set)
        pairs_being_routed = {name for name, pair in diff_pair_ids_to_route_set}
        swappable_pairs = [(name, pair) for name, pair in swappable_diff_pairs.items()
                          if name in pairs_being_routed]

        if len(swappable_pairs) >= 2:
            # Generate debug labels if requested (before swaps, so we see original positions)
            if debug_lines and mps_unroll:
                boundary_debug_labels = generate_debug_boundary_labels(
                    pcb_data, swappable_pairs,
                    lambda pair: get_diff_pair_endpoints(pcb_data, pair.p_net_id, pair.n_net_id, config)
                )

            target_swaps, target_swap_info = apply_target_swaps(
                pcb_data, swappable_pairs, config,
                lambda pair: get_diff_pair_endpoints(pcb_data, pair.p_net_id, pair.n_net_id, config),
                use_boundary_ordering=mps_unroll
            )

    # Apply target swaps for single-ended swappable-nets
    single_ended_target_swaps: Dict[str, str] = {}
    single_ended_target_swap_info: List[Dict] = []
    if swappable_net_patterns:
        from target_swap import apply_single_ended_target_swaps
        from routing_utils import find_single_ended_nets, get_net_endpoints

        # Get diff pair net IDs to exclude
        diff_pair_net_id_set: Set[int] = set()
        for pair_name, pair in diff_pairs.items():
            diff_pair_net_id_set.add(pair.p_net_id)
            diff_pair_net_id_set.add(pair.n_net_id)

        # Find matching single-ended nets
        swappable_se_nets = find_single_ended_nets(
            pcb_data,
            swappable_net_patterns,
            exclude_net_ids=diff_pair_net_id_set
        )

        if len(swappable_se_nets) >= 2:
            print(f"\nAnalyzing target swaps for {len(swappable_se_nets)} single-ended net(s)...")
            single_ended_target_swaps, single_ended_target_swap_info = apply_single_ended_target_swaps(
                pcb_data, swappable_se_nets, config,
                lambda net_id: get_net_endpoints(pcb_data, net_id, config),
                use_boundary_ordering=mps_unroll
            )

    # Generate boundary debug labels for all diff pairs if not already generated from swappable pairs
    if debug_lines and mps_unroll and not boundary_debug_labels and diff_pair_ids_to_route_set:
        from target_swap import generate_debug_boundary_labels
        from diff_pair_routing import get_diff_pair_endpoints
        boundary_debug_labels = generate_debug_boundary_labels(
            pcb_data, list(diff_pair_ids_to_route_set),
            lambda pair: get_diff_pair_endpoints(pcb_data, pair.p_net_id, pair.n_net_id, config)
        )

    # Upfront layer swap optimization: analyze all diff pairs and apply beneficial swaps
    # BEFORE MPS ordering, so ordering sees correct segment layers
    all_stubs_by_layer = {}
    stub_endpoints_by_layer = {}
    if enable_layer_switch and diff_pair_ids_to_route_set:
        total_layer_swaps, all_stubs_by_layer, stub_endpoints_by_layer = apply_diff_pair_layer_swaps(
            pcb_data, config, diff_pair_ids_to_route_set, diff_pairs,
            can_swap_to_top_layer, all_segment_modifications, all_swap_vias
        )

    # Single-ended layer swap optimization (after diff pair swaps, before MPS ordering)
    single_ended_net_ids = [(name, nid) for name, nid in net_ids if nid not in diff_pair_net_ids]

    if enable_layer_switch and single_ended_net_ids:
        total_layer_swaps += apply_single_ended_layer_swaps(
            pcb_data, config, single_ended_net_ids,
            can_swap_to_top_layer, all_segment_modifications, all_swap_vias,
            verbose=args.verbose
        )

    # Apply net ordering strategy
    if ordering_strategy == "mps":
        # Use Maximum Planar Subset algorithm to minimize crossing conflicts
        print(f"\nUsing MPS ordering strategy...")
        all_net_ids = [nid for _, nid in net_ids]

        # If MPS layer swap is enabled, get extended info for conflict analysis
        if args.mps_layer_swap and enable_layer_switch:
            from mps_layer_swap import try_mps_aware_layer_swaps

            mps_result = compute_mps_net_ordering(
                pcb_data, all_net_ids, diff_pairs=diff_pairs,
                use_boundary_ordering=mps_unroll,
                bga_exclusion_zones=bga_exclusion_zones,
                reverse_rounds=args.mps_reverse_rounds,
                crossing_layer_check=crossing_layer_check,
                return_extended_info=True
            )

            if mps_result.num_rounds > 1:
                print(f"\nMPS detected {mps_result.num_rounds} rounds - attempting layer swaps to reduce crossings...")

                swap_result = try_mps_aware_layer_swaps(
                    pcb_data, config, mps_result, diff_pairs,
                    available_layers=config.layers,
                    can_swap_to_top_layer=can_swap_to_top_layer,
                    all_segment_modifications=all_segment_modifications,
                    all_swap_vias=all_swap_vias,
                    all_stubs_by_layer=all_stubs_by_layer,
                    stub_endpoints_by_layer=stub_endpoints_by_layer,
                    verbose=args.verbose
                )

                if swap_result.swaps_applied > 0:
                    total_layer_swaps += swap_result.swaps_applied
                    # Re-run MPS ordering with updated layer assignments
                    print("Re-running MPS ordering after layer swaps...")
                    ordered_ids = compute_mps_net_ordering(
                        pcb_data, all_net_ids, diff_pairs=diff_pairs,
                        use_boundary_ordering=mps_unroll,
                        bga_exclusion_zones=bga_exclusion_zones,
                        reverse_rounds=args.mps_reverse_rounds,
                        crossing_layer_check=crossing_layer_check
                    )
                else:
                    ordered_ids = mps_result.ordered_ids
            else:
                ordered_ids = mps_result.ordered_ids
        else:
            ordered_ids = compute_mps_net_ordering(
                pcb_data, all_net_ids, diff_pairs=diff_pairs,
                use_boundary_ordering=mps_unroll,
                bga_exclusion_zones=bga_exclusion_zones,
                reverse_rounds=args.mps_reverse_rounds,
                crossing_layer_check=crossing_layer_check
            )

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

    # Generate stub position labels for single-ended nets (when debug_lines enabled)
    if debug_lines and single_ended_nets:
        from target_swap import generate_single_ended_debug_labels
        from routing_utils import get_net_endpoints
        stub_labels = generate_single_ended_debug_labels(
            pcb_data, single_ended_nets,
            lambda net_id: get_net_endpoints(pcb_data, net_id, config),
            use_mps_ordering=mps_unroll
        )
        if stub_labels:
            print(f"Generated {len(stub_labels)} stub position labels for single-ended nets")
            boundary_debug_labels.extend(stub_labels)

    results = []
    pad_swaps = []  # List of (pad1, pad2) tuples for nets that need swapping
    successful = 0
    failed = 0
    total_time = 0
    total_iterations = 0
    # Note: all_swap_vias is initialized at line 476 and populated during layer swaps

    # Skip routing if requested - just write output with swaps and debug info
    if skip_routing:
        print(f"\n--skip-routing: Skipping actual routing of {total_routes} items")
        print("Writing output file with swaps and debug labels only...")
        # Clear the lists so routing loops don't execute
        diff_pair_ids_to_route = []
        single_ended_nets = []
    else:
        print(f"\nRouting {total_routes} items ({len(single_ended_nets)} single-ended nets, {len(diff_pair_ids_to_route)} differential pairs)...")
        print("=" * 60)

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
    # Extra clearance = spacing from centerline to P/N track center
    # (The obstacle formulas already include track_width/2, so we only need spacing)
    diff_pair_extra_clearance = (config.track_width + config.diff_pair_gap) / 2
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

    # Get ALL unrouted nets in the PCB for stub proximity costs
    all_unrouted_net_ids = set(get_all_unrouted_net_ids(pcb_data))
    print(f"Found {len(all_unrouted_net_ids)} unrouted nets in PCB for stub proximity")

    # Get exclusion zone lines for User.5 if debug_lines is enabled
    exclusion_zone_lines = []
    if debug_lines:
        all_unrouted_stubs = get_stub_endpoints(pcb_data, list(all_unrouted_net_ids))
        exclusion_zone_lines = draw_exclusion_zones_debug(config, all_unrouted_stubs)
        print(f"Will draw {len(config.bga_exclusion_zones)} BGA zones and {len(all_unrouted_stubs)} stub proximity circles on User.5")

    # Find GND net ID for GND via obstacle tracking (if GND vias enabled)
    gnd_net_id = None
    if config.gnd_via_enabled:
        for net_id, net in pcb_data.nets.items():
            if net.name.upper() == 'GND':
                gnd_net_id = net_id
                break
        if gnd_net_id:
            print(f"GND net ID: {gnd_net_id} (GND vias will be added as obstacles)")

    # Create routing state object to hold all shared state
    state = create_routing_state(
        pcb_data=pcb_data,
        config=config,
        all_net_ids_to_route=all_net_ids_to_route,
        base_obstacles=base_obstacles,
        diff_pair_base_obstacles=diff_pair_base_obstacles,
        diff_pair_extra_clearance=diff_pair_extra_clearance,
        gnd_net_id=gnd_net_id,
        all_unrouted_net_ids=all_unrouted_net_ids,
        total_routes=total_routes,
        enable_layer_switch=enable_layer_switch,
        debug_lines=debug_lines,
        target_swaps=target_swaps,
        target_swap_info=target_swap_info,
        single_ended_target_swaps=single_ended_target_swaps,
        single_ended_target_swap_info=single_ended_target_swap_info,
        all_segment_modifications=all_segment_modifications,
        all_swap_vias=all_swap_vias,
        total_layer_swaps=total_layer_swaps,
    )

    # Create local aliases for frequently-used state fields (enables gradual migration)
    routed_net_ids = state.routed_net_ids
    routed_net_paths = state.routed_net_paths
    routed_results = state.routed_results
    diff_pair_by_net_id = state.diff_pair_by_net_id
    track_proximity_cache = state.track_proximity_cache
    layer_map = state.layer_map
    reroute_queue = state.reroute_queue
    polarity_swapped_pairs = state.polarity_swapped_pairs
    rip_and_retry_history = state.rip_and_retry_history
    ripup_success_pairs = state.ripup_success_pairs
    rerouted_pairs = state.rerouted_pairs
    remaining_net_ids = state.remaining_net_ids
    results = state.results
    pad_swaps = state.pad_swaps

    # Counters (kept as locals, not aliased from state)
    route_index = 0

    # Route differential pairs first (they're more constrained)
    dp_successful, dp_failed, dp_time, dp_iterations, route_index = route_diff_pairs(
        state, diff_pair_ids_to_route
    )
    successful += dp_successful
    failed += dp_failed
    total_time += dp_time
    total_iterations += dp_iterations

    # Route single-ended nets
    se_successful, se_failed, se_time, se_iterations, route_index, user_quit = route_single_ended_nets(
        state, single_ended_nets,
        visualize=visualize, vis_callback=vis_callback, base_vis_data=base_vis_data,
        route_index_start=route_index
    )
    successful += se_successful
    failed += se_failed
    total_time += se_time
    total_iterations += se_iterations

    # Run reroute loop for nets that were ripped during diff pair or single-ended routing
    rq_successful, rq_failed, rq_time, rq_iterations, route_index = run_reroute_loop(
        state, route_index_start=route_index
    )
    successful += rq_successful
    failed += rq_failed
    total_time += rq_time
    total_iterations += rq_iterations

    # Apply length matching if configured
    if length_match_groups:
        print("\n" + "=" * 60)
        print("Length matching")
        print("=" * 60)

        # Build net_name -> result mapping
        net_name_to_result = {}
        for net_id, result in routed_results.items():
            if net_id in pcb_data.nets:
                net_name = pcb_data.nets[net_id].name
                net_name_to_result[net_name] = result

        all_routed_names = list(net_name_to_result.keys())

        # Track all segments/vias from previously processed groups
        # so that subsequent groups can check clearance against them
        all_processed_segments = []
        all_processed_vias = []

        for group in length_match_groups:
            # Handle "auto" for DDR4 grouping
            if len(group) == 1 and group[0].lower() == 'auto':
                auto_groups = auto_group_ddr4_nets(all_routed_names)
                for auto_group in auto_groups:
                    if len(auto_group) >= 2:
                        net_name_to_result = apply_length_matching_to_group(
                            net_name_to_result, auto_group, config, pcb_data,
                            all_processed_segments, all_processed_vias
                        )
                        # Collect segments/vias from this group for subsequent groups
                        for net_name in auto_group:
                            if net_name in net_name_to_result:
                                result = net_name_to_result[net_name]
                                if result.get('new_segments'):
                                    all_processed_segments.extend(result['new_segments'])
                                if result.get('new_vias'):
                                    all_processed_vias.extend(result['new_vias'])
            else:
                # Find nets matching the patterns in this group
                matching_nets = find_nets_matching_patterns(all_routed_names, group)
                if len(matching_nets) >= 2:
                    print(f"\nLength match group: {group}")
                    print(f"  Matched nets: {matching_nets}")
                    net_name_to_result = apply_length_matching_to_group(
                        net_name_to_result, matching_nets, config, pcb_data,
                        all_processed_segments, all_processed_vias
                    )
                    # Collect segments/vias from this group for subsequent groups
                    for net_name in matching_nets:
                        if net_name in net_name_to_result:
                            result = net_name_to_result[net_name]
                            if result.get('new_segments'):
                                all_processed_segments.extend(result['new_segments'])
                            if result.get('new_vias'):
                                all_processed_vias.extend(result['new_vias'])

    # Notify visualization callback that all routing is complete
    if visualize:
        vis_callback.on_routing_complete(successful, failed, total_iterations)

    # Build summary data
    import json
    routed_diff_pairs = []
    failed_diff_pairs = []
    for pair_name, pair in diff_pair_ids_to_route:
        if pair.p_net_id in routed_results and pair.n_net_id in routed_results:
            routed_diff_pairs.append(pair_name)
        else:
            failed_diff_pairs.append(pair_name)
    routed_single = []
    failed_single = []
    for net_name, net_id in single_ended_nets:
        if net_id in routed_results:
            routed_single.append(net_name)
        else:
            failed_single.append(net_name)
    # Count total vias from results
    total_vias = sum(len(r.get('new_vias', [])) for r in results)

    # Print human-readable summary
    print("\n" + "=" * 60)
    print("Routing complete")
    print("=" * 60)
    if diff_pair_ids_to_route:
        if failed_diff_pairs:
            print(f"  {RED}Diff pairs:    {len(routed_diff_pairs)}/{len(diff_pair_ids_to_route)} routed ({len(failed_diff_pairs)} FAILED){RESET}")
        else:
            print(f"  Diff pairs:    {len(routed_diff_pairs)}/{len(diff_pair_ids_to_route)} routed")
    if single_ended_nets:
        if failed_single:
            print(f"  {RED}Single-ended:  {len(routed_single)}/{len(single_ended_nets)} routed ({len(failed_single)} FAILED){RESET}")
        else:
            print(f"  Single-ended:  {len(routed_single)}/{len(single_ended_nets)} routed")
    if ripup_success_pairs:
        print(f"  Rip-up success: {len(ripup_success_pairs)} (routes that ripped blockers)")
    if rerouted_pairs:
        print(f"  Rerouted:      {len(rerouted_pairs)} (ripped nets re-routed)")
    if polarity_swapped_pairs:
        print(f"  Polarity swaps: {len(polarity_swapped_pairs)}")
    if target_swaps:
        # Print unique swaps (each pair shown once)
        swap_pairs = [(k, v) for k, v in target_swaps.items() if k < v]
        print(f"  Target swaps:  {len(swap_pairs)}")
    print(f"  Total vias:    {total_vias}")
    print(f"  Total time:    {total_time:.2f}s")
    print(f"  Iterations:    {total_iterations:,}")
    summary = {
        'routed_diff_pairs': routed_diff_pairs,
        'failed_diff_pairs': failed_diff_pairs,
        'routed_single': routed_single,
        'failed_single': failed_single,
        'ripup_success_pairs': sorted(ripup_success_pairs),
        'rerouted_pairs': sorted(rerouted_pairs),
        'polarity_swapped_pairs': sorted(polarity_swapped_pairs),
        'target_swaps': [{'pair1': k, 'pair2': v} for k, v in target_swaps.items() if k < v],
        'single_ended_target_swaps': [{'net1': k, 'net2': v} for k, v in single_ended_target_swaps.items() if k < v],
        'layer_swaps': total_layer_swaps,
        'successful': successful,
        'failed': failed,
        'total_time': round(total_time, 2),
        'total_iterations': total_iterations,
        'total_vias': total_vias
    }
    print(f"JSON_SUMMARY: {json.dumps(summary)}")

    # Write output if we have results OR if we have layer swap/target swap modifications to show
    # Also write if skip_routing (to output debug labels even without routing)
    if results or all_segment_modifications or all_swap_vias or target_swap_info or single_ended_target_swap_info or skip_routing:
        print(f"\nWriting output to {output_file}...")
        with open(input_file, 'r', encoding='utf-8') as f:
            content = f.read()

        # Apply target swaps FIRST - layer modifications were recorded with post-swap net IDs,
        # so we need to swap the file content to match before applying layer modifications
        if target_swap_info:
            print(f"Applying {len(target_swap_info)} target swap(s) to output file...")
            for swap in target_swap_info:
                # Get layer info for filtering (prevents swapping stubs that share XY on different layers)
                p1_layer = swap.get('p1_layer')
                p2_layer = swap.get('p2_layer')

                # Swap segments at p1's target: p1 net -> p2 net
                content, p1_p_seg = swap_segment_nets_at_positions(
                    content, swap['p1_p_positions'], swap['p1_p_net_id'], swap['p2_p_net_id'], layer=p1_layer)
                content, p1_n_seg = swap_segment_nets_at_positions(
                    content, swap['p1_n_positions'], swap['p1_n_net_id'], swap['p2_n_net_id'], layer=p1_layer)
                # Swap segments at p2's target: p2 net -> p1 net
                content, p2_p_seg = swap_segment_nets_at_positions(
                    content, swap['p2_p_positions'], swap['p2_p_net_id'], swap['p1_p_net_id'], layer=p2_layer)
                content, p2_n_seg = swap_segment_nets_at_positions(
                    content, swap['p2_n_positions'], swap['p2_n_net_id'], swap['p1_n_net_id'], layer=p2_layer)

                # Swap vias at p1's target
                content, p1_p_via = swap_via_nets_at_positions(
                    content, swap['p1_p_positions'], swap['p1_p_net_id'], swap['p2_p_net_id'])
                content, p1_n_via = swap_via_nets_at_positions(
                    content, swap['p1_n_positions'], swap['p1_n_net_id'], swap['p2_n_net_id'])
                # Swap vias at p2's target
                content, p2_p_via = swap_via_nets_at_positions(
                    content, swap['p2_p_positions'], swap['p2_p_net_id'], swap['p1_p_net_id'])
                content, p2_n_via = swap_via_nets_at_positions(
                    content, swap['p2_n_positions'], swap['p2_n_net_id'], swap['p1_n_net_id'])

                # Swap pads if they exist
                if swap['p1_p_pad'] and swap['p2_p_pad']:
                    print(f"    Swapping P pads in output: {swap['p1_p_pad'].component_ref}:{swap['p1_p_pad'].pad_number} <-> {swap['p2_p_pad'].component_ref}:{swap['p2_p_pad'].pad_number}")
                    content = swap_pad_nets_in_content(content, swap['p1_p_pad'], swap['p2_p_pad'])
                else:
                    print(f"    WARNING: Missing P pads for swap: p1={swap['p1_p_pad']}, p2={swap['p2_p_pad']}")
                if swap['p1_n_pad'] and swap['p2_n_pad']:
                    print(f"    Swapping N pads in output: {swap['p1_n_pad'].component_ref}:{swap['p1_n_pad'].pad_number} <-> {swap['p2_n_pad'].component_ref}:{swap['p2_n_pad'].pad_number}")
                    content = swap_pad_nets_in_content(content, swap['p1_n_pad'], swap['p2_n_pad'])
                else:
                    print(f"    WARNING: Missing N pads for swap: p1={swap['p1_n_pad']}, p2={swap['p2_n_pad']}")

                total_seg = p1_p_seg + p1_n_seg + p2_p_seg + p2_n_seg
                total_via = p1_p_via + p1_n_via + p2_p_via + p2_n_via
                print(f"  {swap['p1_name']} <-> {swap['p2_name']}: {total_seg} segments, {total_via} vias")

        # Apply single-ended target swaps
        if single_ended_target_swap_info:
            print(f"Applying {len(single_ended_target_swap_info)} single-ended target swap(s) to output file...")
            for swap in single_ended_target_swap_info:
                # Swap segments at n1's target: n1 net -> n2 net
                content, n1_seg = swap_segment_nets_at_positions(
                    content, swap['n1_positions'], swap['n1_net_id'], swap['n2_net_id'])
                # Swap segments at n2's target: n2 net -> n1 net
                content, n2_seg = swap_segment_nets_at_positions(
                    content, swap['n2_positions'], swap['n2_net_id'], swap['n1_net_id'])

                # Swap vias at n1's target
                content, n1_via = swap_via_nets_at_positions(
                    content, swap['n1_positions'], swap['n1_net_id'], swap['n2_net_id'])
                # Swap vias at n2's target
                content, n2_via = swap_via_nets_at_positions(
                    content, swap['n2_positions'], swap['n2_net_id'], swap['n1_net_id'])

                # Swap pads if they exist
                if swap['n1_pad'] and swap['n2_pad']:
                    print(f"    Swapping pads in output: {swap['n1_pad'].component_ref}:{swap['n1_pad'].pad_number} <-> {swap['n2_pad'].component_ref}:{swap['n2_pad'].pad_number}")
                    content = swap_pad_nets_in_content(content, swap['n1_pad'], swap['n2_pad'])

                total_seg = n1_seg + n2_seg
                total_via = n1_via + n2_via
                print(f"  {swap['n1_name']} <-> {swap['n2_name']}: {total_seg} segments, {total_via} vias")

        # Apply segment layer modifications from stub layer switching AFTER target swaps
        # (layer mods were recorded with post-swap net IDs, so file must be swapped first)
        if all_segment_modifications:
            content, mod_count = modify_segment_layers(content, all_segment_modifications)
            print(f"Applied {mod_count} segment layer modifications (layer switching)")

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
                    via.layers, via.net_id, getattr(via, 'free', False)
                ) + "\n"

        # Add vias from stub layer swapping
        if all_swap_vias:
            print(f"Adding {len(all_swap_vias)} via(s) from stub layer swapping")
            for via in all_swap_vias:
                routing_text += generate_via_sexpr(
                    via.x, via.y, via.size, via.drill,
                    via.layers, via.net_id, getattr(via, 'free', False)
                ) + "\n"

        # Add debug paths if enabled (using gr_line for User layers)
        if debug_lines:
            print("Adding debug paths to User.3 (connectors), User.4 (stub dirs), User.5 (exclusion zones), User.8 (simplified), User.9 (raw A*)")
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
                                0.05, "User.8"
                            ) + "\n"

                # Connector segments on User.3
                connector_lines = result.get('debug_connector_lines', [])
                for start, end in connector_lines:
                    routing_text += generate_gr_line_sexpr(
                        start, end,
                        0.05, "User.3"
                    ) + "\n"

                # Stub direction arrows on User.4
                stub_arrows = result.get('debug_stub_arrows', [])
                for start, end in stub_arrows:
                    routing_text += generate_gr_line_sexpr(
                        start, end,
                        0.05, "User.4"
                    ) + "\n"

            # Exclusion zones on User.5 (BGA zones + proximity, stub proximity circles)
            for start, end in exclusion_zone_lines:
                routing_text += generate_gr_line_sexpr(
                    start, end,
                    0.05, "User.5"
                ) + "\n"

            # Boundary position labels on User.6 (from mps-unroll)
            if boundary_debug_labels:
                print(f"Adding {len(boundary_debug_labels)} boundary position labels to User.6")
                for label in boundary_debug_labels:
                    routing_text += generate_gr_text_sexpr(
                        label['text'], label['x'], label['y'], label['layer'],
                        size=0.1, angle=label.get('angle', 0)
                    ) + "\n"

        last_paren = content.rfind(')')
        new_content = content[:last_paren] + '\n' + routing_text + '\n' + content[last_paren:]

        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(new_content)

        print(f"Successfully wrote {output_file}")

    return successful, failed, total_time

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
    parser.add_argument("--direction", "-d", choices=["forward", "backward", "random"],
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
    parser.add_argument("--via-cost", type=int, default=50,
                        help="Penalty for placing a via in grid steps (default: 50, doubled for diff pairs)")
    parser.add_argument("--via-proximity-cost", type=int, default=10,
                        help="Via cost multiplier in stub/BGA proximity zones (default: 10, 0=block vias)")
    parser.add_argument("--max-iterations", type=int, default=200000,
                        help="Max A* iterations before giving up (default: 200000)")
    parser.add_argument("--max-probe-iterations", type=int, default=5000,
                        help="Max iterations for quick probe phase per direction (default: 5000)")
    parser.add_argument("--heuristic-weight", type=float, default=1.9,
                        help="A* heuristic weight, higher=faster but less optimal (default: 1.9)")
    parser.add_argument("--turn-cost", type=int, default=1000,
                        help="Penalty for direction changes, encourages straighter paths (default: 1000)")

    # Stub proximity penalty
    parser.add_argument("--stub-proximity-radius", type=float, default=2.0,
                        help="Radius around stubs to penalize routing in mm (default: 2.0)")
    parser.add_argument("--stub-proximity-cost", type=float, default=0.2,
                        help="Cost penalty near stubs in mm equivalent (default: 0.2)")

    # BGA proximity penalty
    parser.add_argument("--bga-proximity-radius", type=float, default=7.0,
                        help="Radius around BGA edges to penalize routing in mm (default: 7.0)")
    parser.add_argument("--bga-proximity-cost", type=float, default=0.2,
                        help="Cost penalty near BGA edges in mm equivalent (default: 0.2)")

    # Track proximity penalty (same layer only)
    parser.add_argument("--track-proximity-distance", type=float, default=2.0,
                        help="Distance around routed tracks to penalize routing on same layer in mm (default: 2.0)")
    parser.add_argument("--track-proximity-cost", type=float, default=0.2,
                        help="Cost penalty near routed tracks in mm equivalent (default: 0.2)")

    # Differential pair routing
    parser.add_argument("--diff-pairs", "-D", nargs="+",
                        help="Glob patterns for nets to route as differential pairs (e.g., '*lvds*')")
    parser.add_argument("--diff-pair-gap", type=float, default=0.101,
                        help="Gap between P and N traces of differential pairs in mm (default: 0.101)")
    parser.add_argument("--diff-pair-centerline-setback", type=float, default=None,
                        help="Distance in front of stubs to start centerline route in mm (default: 2x P-N spacing)")
    parser.add_argument("--min-turning-radius", type=float, default=0.2,
                        help="Minimum turning radius for pose-based routing in mm (default: 0.2)")
    parser.add_argument("--no-fix-polarity", action="store_true",
                        help="Don't swap target pad net assignments if polarity swap is needed (default: fix polarity)")
    parser.add_argument("--no-stub-layer-swap", action="store_true",
                        help="Disable stub layer switching optimization (enabled by default)")
    parser.add_argument("--no-crossing-layer-check", action="store_true",
                        help="Count crossings regardless of layer overlap (by default, only same-layer crossings count)")
    parser.add_argument("--no-gnd-vias", action="store_true",
                        help="Disable GND via placement near diff pair signal vias (enabled by default)")
    parser.add_argument("--can-swap-to-top-layer", action="store_true",
                        help="Allow swapping stubs to F.Cu (top layer). Off by default due to via clearance issues.")
    parser.add_argument("--swappable-nets", nargs="+",
                        help="Glob patterns for diff pair nets that can have targets swapped (e.g., 'rx1_*')")
    parser.add_argument("--crossing-penalty", type=float, default=1000.0,
                        help="Penalty for crossing assignments in target swap optimization (default: 1000.0)")
    parser.add_argument("--mps-reverse-rounds", action="store_true",
                        help="Reverse MPS round order: route most-conflicting groups first instead of least-conflicting")
    parser.add_argument("--mps-layer-swap", action="store_true",
                        help="Enable MPS-aware layer swaps to reduce crossing conflicts by moving Round 2+ nets to different layers")

    # Length matching options
    parser.add_argument("--length-match-group", action="append", nargs="+", dest="length_match_groups",
                        help="Net patterns to length-match as a group (can be repeated). Use 'auto' for DDR4 auto-grouping")
    parser.add_argument("--length-match-tolerance", type=float, default=0.1,
                        help="Acceptable length variance within group in mm (default: 0.1)")
    parser.add_argument("--meander-amplitude", type=float, default=1.0,
                        help="Height of meander perpendicular to trace in mm (default: 1.0)")

    # Rip-up and retry options
    parser.add_argument("--max-ripup", type=int, default=3,
                        help="Maximum blockers to rip up at once during rip-up and retry (default: 3)")
    parser.add_argument("--max-setback-angle", type=float, default=45.0,
                        help="Maximum angle (degrees) for setback position search (default: 45.0)")
    parser.add_argument("--routing-clearance-margin", type=float, default=1.0,
                        help="Multiplier on track-via clearance (1.0 = minimum DRC)")
    parser.add_argument("--max-turn-angle", type=float, default=180.0,
                        help="Max cumulative turn angle (degrees) before reset, to prevent U-turns (default: 180)")

    # Vertical alignment attraction options
    parser.add_argument("--vertical-attraction-radius", type=float, default=1.0,
                        help="Radius in mm for cross-layer track attraction (0 = disabled, default: 1.0)")
    parser.add_argument("--vertical-attraction-cost", type=float, default=0.1,
                        help="Cost bonus in mm equivalent for tracks aligned with other layers (default: 0.1)")

    # Debug options
    parser.add_argument("--debug-lines", action="store_true",
                        help="Output debug geometry on User.3 (connectors), User.4 (stub dirs), User.8 (simplified), User.9 (raw A*)")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Print detailed diagnostic output (setback checks, etc.)")
    parser.add_argument("--skip-routing", action="store_true",
                        help="Skip actual routing, only do swaps and write debug info")

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
                max_probe_iterations=args.max_probe_iterations,
                heuristic_weight=args.heuristic_weight,
                turn_cost=args.turn_cost,
                stub_proximity_radius=args.stub_proximity_radius,
                stub_proximity_cost=args.stub_proximity_cost,
                via_proximity_cost=args.via_proximity_cost,
                bga_proximity_radius=args.bga_proximity_radius,
                bga_proximity_cost=args.bga_proximity_cost,
                track_proximity_distance=args.track_proximity_distance,
                track_proximity_cost=args.track_proximity_cost,
                diff_pair_patterns=args.diff_pairs,
                diff_pair_gap=args.diff_pair_gap,
                diff_pair_centerline_setback=args.diff_pair_centerline_setback,
                min_turning_radius=args.min_turning_radius,
                debug_lines=args.debug_lines,
                verbose=args.verbose,
                fix_polarity=not args.no_fix_polarity,
                max_rip_up_count=args.max_ripup,
                max_setback_angle=args.max_setback_angle,
                enable_layer_switch=not args.no_stub_layer_swap,
                crossing_layer_check=not args.no_crossing_layer_check,
                can_swap_to_top_layer=args.can_swap_to_top_layer,
                swappable_net_patterns=args.swappable_nets,
                crossing_penalty=args.crossing_penalty,
                skip_routing=args.skip_routing,
                routing_clearance_margin=args.routing_clearance_margin,
                max_turn_angle=args.max_turn_angle,
                gnd_via_enabled=not args.no_gnd_vias,
                vertical_attraction_radius=args.vertical_attraction_radius,
                vertical_attraction_cost=args.vertical_attraction_cost,
                length_match_groups=args.length_match_groups,
                length_match_tolerance=args.length_match_tolerance,
                meander_amplitude=args.meander_amplitude,
                vis_callback=vis_callback)

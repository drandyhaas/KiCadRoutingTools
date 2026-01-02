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
from blocking_analysis import analyze_frontier_blocking, print_blocking_analysis
from rip_up_reroute import rip_up_net, restore_net
from polarity_swap import apply_polarity_swap, get_canonical_net_id
from layer_swap_upfront import optimize_diff_pair_layers_upfront, optimize_single_ended_layers_upfront
from output_writer import write_routed_output
from layer_swap_fallback import try_fallback_layer_swap, add_own_stubs_as_obstacles_for_diff_pair
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

            # Generate debug labels for single-ended nets if requested
            if debug_lines and mps_unroll:
                from target_swap import generate_single_ended_debug_labels
                se_labels = generate_single_ended_debug_labels(
                    pcb_data, swappable_se_nets,
                    lambda net_id: get_net_endpoints(pcb_data, net_id, config)
                )
                boundary_debug_labels.extend(se_labels)

    # Generate boundary debug labels for all diff pairs if not already generated from swappable pairs
    if debug_lines and mps_unroll and not boundary_debug_labels and diff_pair_ids_to_route_set:
        from target_swap import generate_debug_boundary_labels
        from diff_pair_routing import get_diff_pair_endpoints
        boundary_debug_labels = generate_debug_boundary_labels(
            pcb_data, list(diff_pair_ids_to_route_set),
            lambda pair: get_diff_pair_endpoints(pcb_data, pair.p_net_id, pair.n_net_id, config)
        )

    # Upfront layer swap optimization for diff pairs
    if enable_layer_switch and diff_pair_ids_to_route_set:
        total_layer_swaps += optimize_diff_pair_layers_upfront(
            pcb_data, config, diff_pair_ids_to_route_set, diff_pairs,
            can_swap_to_top_layer, all_swap_vias, all_segment_modifications
        )

    # Upfront layer swap optimization for single-ended nets
    if enable_layer_switch:
        total_layer_swaps += optimize_single_ended_layers_upfront(
            pcb_data, config, net_ids, diff_pair_net_ids,
            can_swap_to_top_layer, all_swap_vias, all_segment_modifications,
            verbose=args.verbose
        )

    # Apply net ordering strategy
    if ordering_strategy == "mps":
        # Use Maximum Planar Subset algorithm to minimize crossing conflicts
        print(f"\nUsing MPS ordering strategy...")
        all_net_ids = [nid for _, nid in net_ids]
        ordered_ids = compute_mps_net_ordering(pcb_data, all_net_ids, diff_pairs=diff_pairs,
                                               use_boundary_ordering=mps_unroll,
                                               bga_exclusion_zones=bga_exclusion_zones,
                                               reverse_rounds=args.mps_reverse_rounds,
                                               crossing_layer_check=crossing_layer_check)
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

    # Track which nets have been routed (their segments/vias are now in pcb_data)
    routed_net_ids = []
    routed_net_paths = {}  # net_id -> path (grid coords) for blocking analysis
    routed_results = {}  # net_id -> result dict (for rip-up)
    diff_pair_by_net_id = {}  # net_id -> (pair_name, pair) for looking up diff pairs
    # Cache for track proximity costs: net_id -> {layer_idx -> {(gx,gy) -> cost}}
    # Computed once when route succeeds, removed when ripped up
    track_proximity_cache = {}
    layer_map = {name: idx for idx, name in enumerate(config.layers)}
    reroute_queue = []  # List of (pair_name, pair) or (net_name, net_id) for ripped-up nets
    polarity_swapped_pairs = set()  # Track pairs that have already had polarity swap applied
    rip_and_retry_history = set()  # Set of (routing_net_id, frozenset(ripped_net_ids)) to prevent infinite loops
    ripup_success_pairs = set()  # Pairs that succeeded after ripping up blockers
    rerouted_pairs = set()  # Pairs that were ripped up and then successfully rerouted
    remaining_net_ids = list(all_net_ids_to_route)

    # Get ALL unrouted nets in the PCB for stub proximity costs
    # (not just the ones we're routing in this batch)
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

    route_index = 0

    # Route differential pairs first (they're more constrained)
    for pair_name, pair in diff_pair_ids_to_route:
        route_index += 1
        failed_str = f" ({failed} failed)" if failed > 0 else ""
        print(f"\n[{route_index}/{total_routes}{failed_str}] Routing diff pair {pair_name}")
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

        # Add GND vias as obstacles (they were placed with previous diff pair routes)
        if gnd_net_id is not None:
            add_net_vias_as_obstacles(obstacles, pcb_data, gnd_net_id, config, diff_pair_extra_clearance)

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
        # Add track proximity costs for previously routed tracks (same layer only)
        merge_track_proximity_costs(obstacles, track_proximity_cache)

        # Add cross-layer track data for vertical alignment attraction
        # This includes previously routed tracks (newly routed segments are in pcb_data.segments)
        add_cross_layer_tracks(obstacles, pcb_data, config, layer_map,
                                exclude_net_ids={pair.p_net_id, pair.n_net_id})

        # Add same-net via clearance for both P and N
        add_same_net_via_clearance(obstacles, pcb_data, pair.p_net_id, config)
        add_same_net_via_clearance(obstacles, pcb_data, pair.n_net_id, config)

        # Add the diff pair's own stub segments as obstacles to prevent the centerline
        # from routing through them. Exclude the stub endpoints where we need to connect.
        add_own_stubs_as_obstacles_for_diff_pair(obstacles, pcb_data, pair.p_net_id, pair.n_net_id, config, diff_pair_extra_clearance)

        # Get source/target coordinates for blocking analysis (center of P/N endpoints)
        from diff_pair_routing import get_diff_pair_endpoints
        sources, targets, _ = get_diff_pair_endpoints(pcb_data, pair.p_net_id, pair.n_net_id, config)
        source_xy = None
        target_xy = None
        if sources:
            src = sources[0]  # Use first source pair
            # Source center = average of P and N source coords (indices 5,6 and 7,8 are float coords)
            source_xy = ((src[5] + src[7]) / 2, (src[6] + src[8]) / 2)
        if targets:
            tgt = targets[0]  # Use first target pair
            # Target center = average of P and N target coords
            target_xy = ((tgt[5] + tgt[7]) / 2, (tgt[6] + tgt[8]) / 2)

        # Route the differential pair
        # Pass both diff pair obstacles (with extra clearance) and base obstacles (for extension routing)
        # Also pass unrouted stubs for finding clear extension endpoints
        # Note: Layer switching is now done upfront before routing starts
        result = route_diff_pair_with_obstacles(pcb_data, pair, config, obstacles, base_obstacles,
                                                 unrouted_stubs)

        # Handle probe_blocked: try progressive rip-up before full route
        probe_ripup_attempts = 0
        probe_ripped_items = []  # Track all nets ripped during probe phase for potential restore
        while result and result.get('probe_blocked') and probe_ripup_attempts < config.max_rip_up_count:
            blocked_at = result.get('blocked_at', 'unknown')
            blocked_cells = result.get('blocked_cells', [])
            probe_direction = result.get('direction', 'unknown')

            if not blocked_cells:
                print(f"  Probe {probe_direction} blocked at {blocked_at} but no blocked cells to analyze")
                break

            # Analyze which routed nets are blocking
            blockers = analyze_frontier_blocking(
                blocked_cells, pcb_data, config, routed_net_paths,
                exclude_net_ids={pair.p_net_id, pair.n_net_id},
                extra_clearance=config.diff_pair_gap / 2,
                target_xy=target_xy,
                source_xy=source_xy
            )

            # Filter to rippable blockers (only those we've routed)
            rippable_blockers = []
            seen_canonical_ids = set()
            for b in blockers:
                canonical = get_canonical_net_id(b.net_id, diff_pair_by_net_id)
                if canonical not in seen_canonical_ids and b.net_id in routed_results:
                    seen_canonical_ids.add(canonical)
                    rippable_blockers.append(b)

            if not rippable_blockers:
                print(f"  Probe {probe_direction} blocked at {blocked_at} but no rippable blockers found")
                # Convert to failed result so normal failure handling takes over
                result = {
                    'failed': True,
                    'iterations': result.get('iterations', 0),
                    'blocked_cells_forward': blocked_cells if probe_direction == 'forward' else [],
                    'blocked_cells_backward': blocked_cells if probe_direction == 'backward' else [],
                }
                break

            probe_ripup_attempts += 1
            print(f"  Probe blocked at {blocked_at}, rip-up attempt {probe_ripup_attempts}/{config.max_rip_up_count}")

            # Rip up the top blocker
            blocker = rippable_blockers[0]
            print(f"    Ripping up {blocker.net_name} ({blocker.blocked_count} blocked cells)")

            saved_result, ripped_ids, was_in_results = rip_up_net(
                blocker.net_id, pcb_data, routed_net_ids, routed_net_paths,
                routed_results, diff_pair_by_net_id, remaining_net_ids,
                results, config, track_proximity_cache
            )
            # Track this ripped item for potential restore on total failure
            probe_ripped_items.append((blocker, saved_result, ripped_ids, was_in_results))

            # Rebuild obstacles without ripped net
            retry_obstacles = diff_pair_base_obstacles.clone()
            for routed_id in routed_net_ids:
                add_net_stubs_as_obstacles(retry_obstacles, pcb_data, routed_id, config, extra_clearance=config.diff_pair_gap / 2)
                add_net_vias_as_obstacles(retry_obstacles, pcb_data, routed_id, config, extra_clearance=config.diff_pair_gap / 2)
                add_net_pads_as_obstacles(retry_obstacles, pcb_data, routed_id, config, extra_clearance=config.diff_pair_gap / 2)
            # Add GND vias as obstacles
            if gnd_net_id is not None:
                add_net_vias_as_obstacles(retry_obstacles, pcb_data, gnd_net_id, config, extra_clearance=config.diff_pair_gap / 2)
            add_same_net_via_clearance(retry_obstacles, pcb_data, pair.p_net_id, config)
            add_same_net_via_clearance(retry_obstacles, pcb_data, pair.n_net_id, config)
            add_own_stubs_as_obstacles_for_diff_pair(retry_obstacles, pcb_data, pair.p_net_id, pair.n_net_id, config, config.diff_pair_gap / 2)

            # Retry the route
            result = route_diff_pair_with_obstacles(pcb_data, pair, config, retry_obstacles, base_obstacles,
                                                     unrouted_stubs)

            # If retry succeeded, queue ALL ripped nets for re-routing
            if result and not result.get('failed') and not result.get('probe_blocked'):
                # Success! Queue all ripped nets for re-routing later
                for ripped_blocker, ripped_saved, ripped_ids_item, ripped_was_in_results in probe_ripped_items:
                    if ripped_was_in_results:
                        successful -= 1
                    if ripped_blocker.net_id in diff_pair_by_net_id:
                        ripped_pair_name, ripped_pair = diff_pair_by_net_id[ripped_blocker.net_id]
                        reroute_queue.append(('diff_pair', ripped_pair_name, ripped_pair))
                    else:
                        reroute_queue.append(('single', ripped_blocker.net_name, ripped_blocker.net_id))
                ripup_success_pairs.add(pair_name)
                print(f"    Probe rip-up succeeded, {len(probe_ripped_items)} net(s) queued for re-routing")
            elif result and result.get('probe_blocked'):
                # Still probe_blocked, will loop and try next rip-up
                print(f"    Still blocked after rip-up, trying next...")
            else:
                # Route failed completely - restore ALL ripped nets and exit loop
                print(f"    Probe rip-up didn't help, restoring {len(probe_ripped_items)} net(s)")
                for ripped_blocker, ripped_saved, ripped_ids_item, ripped_was_in_results in reversed(probe_ripped_items):
                    if ripped_saved:
                        add_route_to_pcb_data(pcb_data, ripped_saved, debug_lines=config.debug_lines)
                        for rid in ripped_ids_item:
                            if rid not in routed_net_ids:
                                routed_net_ids.append(rid)
                            if rid in remaining_net_ids:
                                remaining_net_ids.remove(rid)
                            routed_results[rid] = ripped_saved
                        if ripped_was_in_results:
                            results.append(ripped_saved)
                        # Restore track proximity cache
                        if track_proximity_cache is not None and layer_map is not None:
                            for rid in ripped_ids_item:
                                track_proximity_cache[rid] = compute_track_proximity_for_net(pcb_data, rid, config, layer_map)
                probe_ripped_items.clear()
                break

        # If probe rip-up exhausted attempts without success, restore all and try full route
        if result and result.get('probe_blocked'):
            print(f"  Probe blocked after {probe_ripup_attempts} rip-up attempts, restoring {len(probe_ripped_items)} net(s), trying full route...")
            for ripped_blocker, ripped_saved, ripped_ids_item, ripped_was_in_results in reversed(probe_ripped_items):
                if ripped_saved:
                    add_route_to_pcb_data(pcb_data, ripped_saved, debug_lines=config.debug_lines)
                    for rid in ripped_ids_item:
                        if rid not in routed_net_ids:
                            routed_net_ids.append(rid)
                        if rid in remaining_net_ids:
                            remaining_net_ids.remove(rid)
                        routed_results[rid] = ripped_saved
                    if ripped_was_in_results:
                        results.append(ripped_saved)
                    # Restore track proximity cache
                    if track_proximity_cache is not None and layer_map is not None:
                        for rid in ripped_ids_item:
                            track_proximity_cache[rid] = compute_track_proximity_for_net(pcb_data, rid, config, layer_map)
            result = {
                'failed': True,
                'iterations': result.get('iterations', 0),
                'blocked_cells_forward': result.get('blocked_cells', []) if result.get('direction') == 'forward' else [],
                'blocked_cells_backward': result.get('blocked_cells', []) if result.get('direction') == 'backward' else [],
            }

        elapsed = time.time() - start_time
        total_time += elapsed

        if result and not result.get('failed'):
            print(f"  SUCCESS: {len(result['new_segments'])} segments, {len(result['new_vias'])} vias, {result['iterations']} iterations ({elapsed:.2f}s)")
            results.append(result)
            successful += 1
            total_iterations += result['iterations']

            # Check if polarity was fixed - need to swap target pad and stub nets
            # Do this BEFORE add_route_to_pcb_data so segment processing sees correct net IDs
            apply_polarity_swap(pcb_data, result, pad_swaps, pair_name, polarity_swapped_pairs)

            add_route_to_pcb_data(pcb_data, result, debug_lines=config.debug_lines)

            if pair.p_net_id in remaining_net_ids:
                remaining_net_ids.remove(pair.p_net_id)
            if pair.n_net_id in remaining_net_ids:
                remaining_net_ids.remove(pair.n_net_id)
            routed_net_ids.append(pair.p_net_id)
            routed_net_ids.append(pair.n_net_id)
            # Compute and cache track proximity costs for the newly routed nets
            track_proximity_cache[pair.p_net_id] = compute_track_proximity_for_net(pcb_data, pair.p_net_id, config, layer_map)
            track_proximity_cache[pair.n_net_id] = compute_track_proximity_for_net(pcb_data, pair.n_net_id, config, layer_map)
            # Track paths for blocking analysis
            if result.get('p_path'):
                routed_net_paths[pair.p_net_id] = result['p_path']
            if result.get('n_path'):
                routed_net_paths[pair.n_net_id] = result['n_path']
            # Track result for rip-up
            routed_results[pair.p_net_id] = result
            routed_results[pair.n_net_id] = result  # Both P and N share the same result
            diff_pair_by_net_id[pair.p_net_id] = (pair_name, pair)
            diff_pair_by_net_id[pair.n_net_id] = (pair_name, pair)
        else:
            iterations = result['iterations'] if result else 0
            print(f"  FAILED: Could not find route ({elapsed:.2f}s)")
            total_iterations += iterations

            # Try rip-up and reroute with progressive N+1
            ripped_up = False
            if routed_net_paths and result:
                # Find the direction that failed faster (likely more constrained)
                fwd_iters = result.get('iterations_forward', 0)
                bwd_iters = result.get('iterations_backward', 0)
                fwd_cells = result.get('blocked_cells_forward', [])
                bwd_cells = result.get('blocked_cells_backward', [])

                # Check for setback failure (no routing iterations but have blocked cells)
                is_setback_failure = (fwd_iters == 0 and bwd_iters == 0 and (fwd_cells or bwd_cells))

                if is_setback_failure:
                    # Combine blocked cells from both directions for setback failures
                    blocked_cells = list(set(fwd_cells + bwd_cells))
                    if blocked_cells:
                        print(f"  Setback blocked ({len(blocked_cells)} blocked cells)")
                elif fwd_iters > 0 and (bwd_iters == 0 or fwd_iters <= bwd_iters):
                    fastest_dir = 'forward'
                    blocked_cells = fwd_cells
                    dir_iters = fwd_iters
                    if blocked_cells:
                        print(f"  {fastest_dir.capitalize()} direction failed fastest ({dir_iters} iterations, {len(blocked_cells)} blocked cells)")
                else:
                    fastest_dir = 'backward'
                    blocked_cells = bwd_cells
                    dir_iters = bwd_iters
                    if blocked_cells:
                        print(f"  {fastest_dir.capitalize()} direction failed fastest ({dir_iters} iterations, {len(blocked_cells)} blocked cells)")

                # Analyze blocking for all failure types (setback, forward, backward)
                if blocked_cells:
                    blockers = analyze_frontier_blocking(
                        blocked_cells, pcb_data, config, routed_net_paths,
                        exclude_net_ids={pair.p_net_id, pair.n_net_id},
                        extra_clearance=diff_pair_extra_clearance,
                        target_xy=target_xy,
                        source_xy=source_xy
                    )
                    print_blocking_analysis(blockers)

                    # Filter to only rippable blockers (those in routed_results)
                    # and deduplicate by diff pair (P and N count as one)
                    rippable_blockers = []
                    seen_canonical_ids = set()
                    for b in blockers:
                        if b.net_id in routed_results:
                            canonical = get_canonical_net_id(b.net_id, diff_pair_by_net_id)
                            if canonical not in seen_canonical_ids:
                                seen_canonical_ids.add(canonical)
                                rippable_blockers.append(b)

                    # Progressive rip-up: try N=1, then N=2, etc up to max_rip_up_count
                    # Keep nets ripped between N levels to avoid redundant restore/re-rip
                    current_canonical = pair.p_net_id  # P is canonical for diff pairs
                    ripped_items = []  # Accumulate ripped items across N levels
                    ripped_canonical_ids = set()  # Track which canonicals have been ripped
                    retry_succeeded = False
                    last_retry_blocked_cells = blocked_cells  # Start with initial failure's blocked cells

                    for N in range(1, config.max_rip_up_count + 1):
                        # For N > 1, re-analyze from the last retry's blocked cells
                        # to find the most blocking net from that specific failure
                        if N > 1 and last_retry_blocked_cells:
                            print(f"  Re-analyzing {len(last_retry_blocked_cells)} blocked cells from N={N-1} retry:")
                            fresh_blockers = analyze_frontier_blocking(
                                last_retry_blocked_cells, pcb_data, config, routed_net_paths,
                                exclude_net_ids={pair.p_net_id, pair.n_net_id},
                                extra_clearance=diff_pair_extra_clearance,
                                target_xy=target_xy,
                                source_xy=source_xy
                            )
                            print_blocking_analysis(fresh_blockers, prefix="    ")
                            # Find the most-blocking net that isn't already ripped
                            next_blocker = None
                            for b in fresh_blockers:
                                if b.net_id in routed_results:
                                    canonical = get_canonical_net_id(b.net_id, diff_pair_by_net_id)
                                    if canonical not in ripped_canonical_ids:
                                        next_blocker = b
                                        break
                            if next_blocker is None:
                                print(f"  No additional rippable blockers from retry analysis")
                                break
                            # Replace the Nth blocker with the one from retry analysis
                            # First check if it's already in rippable_blockers
                            next_canonical = get_canonical_net_id(next_blocker.net_id, diff_pair_by_net_id)
                            if next_canonical not in seen_canonical_ids:
                                seen_canonical_ids.add(next_canonical)
                                rippable_blockers.append(next_blocker)
                            # Find and move it to position N-1 if needed
                            for idx, b in enumerate(rippable_blockers):
                                if get_canonical_net_id(b.net_id, diff_pair_by_net_id) == next_canonical:
                                    if idx != N - 1 and N - 1 < len(rippable_blockers):
                                        # Swap to position N-1
                                        rippable_blockers[idx], rippable_blockers[N-1] = rippable_blockers[N-1], rippable_blockers[idx]
                                    break

                        if N > len(rippable_blockers):
                            break  # Not enough blockers to rip

                        # Build frozenset of all N blocker canonicals for loop check
                        blocker_canonicals = frozenset(
                            get_canonical_net_id(rippable_blockers[i].net_id, diff_pair_by_net_id)
                            for i in range(N)
                        )
                        if (current_canonical, blocker_canonicals) in rip_and_retry_history:
                            # Build informative message showing the loop (use pair names for diff pairs)
                            blocker_names = []
                            for i in range(N):
                                b = rippable_blockers[i]
                                if b.net_id in diff_pair_by_net_id:
                                    blocker_names.append(diff_pair_by_net_id[b.net_id][0])
                                else:
                                    blocker_names.append(b.net_name)
                            if len(blocker_names) == 1:
                                blockers_str = blocker_names[0]
                            else:
                                blockers_str = "{" + ", ".join(blocker_names) + "}"
                            print(f"  Skipping N={N}: already tried ripping {blockers_str} for {pair_name}")
                            continue

                        # Rip up only the new blocker(s) for this N level
                        # (previous blockers are already ripped from N-1)
                        rip_successful = True
                        new_ripped_this_level = []

                        if N == 1:
                            blocker = rippable_blockers[0]
                            if blocker.net_id in diff_pair_by_net_id:
                                ripped_pair_name_tmp, _ = diff_pair_by_net_id[blocker.net_id]
                                print(f"  Ripping up diff pair {ripped_pair_name_tmp} (P and N) to retry...")
                            else:
                                print(f"  Ripping up {blocker.net_name} to retry...")
                        else:
                            blocker = rippable_blockers[N-1]
                            if blocker.net_id in diff_pair_by_net_id:
                                ripped_pair_name_tmp, _ = diff_pair_by_net_id[blocker.net_id]
                                print(f"  Extending to N={N}: ripping diff pair {ripped_pair_name_tmp}...")
                            else:
                                print(f"  Extending to N={N}: ripping {blocker.net_name}...")

                        for i in range(len(ripped_items), N):
                            blocker = rippable_blockers[i]
                            # Skip if already ripped (e.g., as part of a diff pair)
                            if blocker.net_id not in routed_results:
                                continue
                            saved_result, ripped_ids, was_in_results = rip_up_net(
                                blocker.net_id, pcb_data, routed_net_ids, routed_net_paths,
                                routed_results, diff_pair_by_net_id, remaining_net_ids,
                                results, config, track_proximity_cache
                            )
                            if saved_result is None:
                                rip_successful = False
                                break
                            ripped_items.append((blocker.net_id, saved_result, ripped_ids, was_in_results))
                            new_ripped_this_level.append((blocker.net_id, saved_result, ripped_ids, was_in_results))
                            ripped_canonical_ids.add(get_canonical_net_id(blocker.net_id, diff_pair_by_net_id))
                            if was_in_results:
                                successful -= 1
                            if blocker.net_id in diff_pair_by_net_id:
                                ripped_pair_name_tmp, _ = diff_pair_by_net_id[blocker.net_id]
                                print(f"    Ripped diff pair {ripped_pair_name_tmp}")
                            else:
                                print(f"    Ripped {blocker.net_name}")

                        if not rip_successful:
                            # Restore only the nets we just ripped this level
                            for net_id, saved_result, ripped_ids, was_in_results in reversed(new_ripped_this_level):
                                restore_net(net_id, saved_result, ripped_ids, was_in_results,
                                           pcb_data, routed_net_ids, routed_net_paths,
                                           routed_results, diff_pair_by_net_id, remaining_net_ids,
                                           results, config, track_proximity_cache, layer_map)
                                if was_in_results:
                                    successful += 1
                                ripped_items.pop()
                            continue

                        # Rebuild obstacles and retry the current route
                        retry_obstacles = diff_pair_base_obstacles.clone()
                        for routed_id in routed_net_ids:
                            add_net_stubs_as_obstacles(retry_obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
                            add_net_vias_as_obstacles(retry_obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
                            add_net_pads_as_obstacles(retry_obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
                        # Add GND vias as obstacles
                        if gnd_net_id is not None:
                            add_net_vias_as_obstacles(retry_obstacles, pcb_data, gnd_net_id, config, diff_pair_extra_clearance)
                        other_unrouted = [nid for nid in remaining_net_ids
                                         if nid != pair.p_net_id and nid != pair.n_net_id]
                        for other_net_id in other_unrouted:
                            add_net_stubs_as_obstacles(retry_obstacles, pcb_data, other_net_id, config, diff_pair_extra_clearance)
                            add_net_vias_as_obstacles(retry_obstacles, pcb_data, other_net_id, config, diff_pair_extra_clearance)
                            add_net_pads_as_obstacles(retry_obstacles, pcb_data, other_net_id, config, diff_pair_extra_clearance)
                        stub_proximity_net_ids = [nid for nid in all_unrouted_net_ids
                                                   if nid != pair.p_net_id and nid != pair.n_net_id
                                                   and nid not in routed_net_ids]
                        unrouted_stubs = get_stub_endpoints(pcb_data, stub_proximity_net_ids)
                        if unrouted_stubs:
                            add_stub_proximity_costs(retry_obstacles, unrouted_stubs, config)
                        merge_track_proximity_costs(retry_obstacles, track_proximity_cache)
                        add_same_net_via_clearance(retry_obstacles, pcb_data, pair.p_net_id, config)
                        add_same_net_via_clearance(retry_obstacles, pcb_data, pair.n_net_id, config)
                        add_own_stubs_as_obstacles_for_diff_pair(retry_obstacles, pcb_data, pair.p_net_id, pair.n_net_id, config, diff_pair_extra_clearance)

                        retry_result = route_diff_pair_with_obstacles(pcb_data, pair, config, retry_obstacles, base_obstacles, unrouted_stubs)

                        if retry_result and not retry_result.get('failed') and not retry_result.get('probe_blocked'):
                            print(f"  RETRY SUCCESS (N={N}): {len(retry_result['new_segments'])} segments, {len(retry_result['new_vias'])} vias")
                            results.append(retry_result)
                            successful += 1
                            total_iterations += retry_result['iterations']

                            # Apply polarity swap if needed (same as original route path)
                            apply_polarity_swap(pcb_data, retry_result, pad_swaps, pair_name, polarity_swapped_pairs)

                            add_route_to_pcb_data(pcb_data, retry_result, debug_lines=config.debug_lines)
                            if pair.p_net_id in remaining_net_ids:
                                remaining_net_ids.remove(pair.p_net_id)
                            if pair.n_net_id in remaining_net_ids:
                                remaining_net_ids.remove(pair.n_net_id)
                            routed_net_ids.append(pair.p_net_id)
                            routed_net_ids.append(pair.n_net_id)
                            # Compute and cache track proximity costs
                            track_proximity_cache[pair.p_net_id] = compute_track_proximity_for_net(pcb_data, pair.p_net_id, config, layer_map)
                            track_proximity_cache[pair.n_net_id] = compute_track_proximity_for_net(pcb_data, pair.n_net_id, config, layer_map)
                            if retry_result.get('p_path'):
                                routed_net_paths[pair.p_net_id] = retry_result['p_path']
                            if retry_result.get('n_path'):
                                routed_net_paths[pair.n_net_id] = retry_result['n_path']
                            routed_results[pair.p_net_id] = retry_result
                            routed_results[pair.n_net_id] = retry_result
                            diff_pair_by_net_id[pair.p_net_id] = (pair_name, pair)
                            diff_pair_by_net_id[pair.n_net_id] = (pair_name, pair)

                            # Queue all ripped-up nets for rerouting and add to history
                            rip_and_retry_history.add((current_canonical, blocker_canonicals))

                            for net_id, saved_result, ripped_ids, was_in_results in ripped_items:
                                # Decrement successful count - will be re-counted when rerouted
                                if was_in_results:
                                    successful -= 1
                                if net_id in diff_pair_by_net_id:
                                    ripped_pair_name_tmp, ripped_pair_tmp = diff_pair_by_net_id[net_id]
                                    reroute_queue.append(('diff_pair', ripped_pair_name_tmp, ripped_pair_tmp))
                                else:
                                    net = pcb_data.nets.get(net_id)
                                    net_name = net.name if net else f"Net {net_id}"
                                    reroute_queue.append(('single', net_name, net_id))

                            ripped_up = True
                            retry_succeeded = True
                            ripup_success_pairs.add(pair_name)
                            break  # Success! Exit the N loop
                        else:
                            # Retry failed - keep nets ripped for N+1 attempt
                            print(f"  RETRY FAILED (N={N})")

                            # Store blocked cells from retry for next iteration's analysis
                            if retry_result:
                                if retry_result.get('probe_blocked'):
                                    last_retry_blocked_cells = retry_result.get('blocked_cells', [])
                                else:
                                    retry_fwd_cells = retry_result.get('blocked_cells_forward', [])
                                    retry_bwd_cells = retry_result.get('blocked_cells_backward', [])
                                    last_retry_blocked_cells = list(set(retry_fwd_cells + retry_bwd_cells))
                                if last_retry_blocked_cells:
                                    print(f"    Retry had {len(last_retry_blocked_cells)} blocked cells")
                                else:
                                    print(f"    No blocked cells from retry to analyze")

                    # If all N levels failed, restore all ripped nets
                    if not retry_succeeded and ripped_items:
                        print(f"  {RED}All rip-up attempts failed: Restoring {len(ripped_items)} net(s){RESET}")
                        for net_id, saved_result, ripped_ids, was_in_results in reversed(ripped_items):
                            restore_net(net_id, saved_result, ripped_ids, was_in_results,
                                       pcb_data, routed_net_ids, routed_net_paths,
                                       routed_results, diff_pair_by_net_id, remaining_net_ids,
                                       results, config, track_proximity_cache, layer_map)
                            if was_in_results:
                                successful += 1

            # Fallback layer swap for setback failures (after rip-up fails)
            if not ripped_up and is_setback_failure and enable_layer_switch:
                print(f"  Trying fallback layer swap for setback failure...")
                swap_success, swap_result, _, _ = try_fallback_layer_swap(
                    pcb_data, pair, pair_name, config,
                    fwd_cells, bwd_cells,
                    diff_pair_base_obstacles, base_obstacles,
                    routed_net_ids, remaining_net_ids,
                    all_unrouted_net_ids, gnd_net_id,
                    track_proximity_cache, diff_pair_extra_clearance,
                    all_swap_vias, all_segment_modifications,
                    all_stubs_by_layer, stub_endpoints_by_layer,
                    routed_net_paths, routed_results, diff_pair_by_net_id, layer_map,
                    target_swaps, results=results)

                if swap_success and swap_result:
                    print(f"  {GREEN}FALLBACK LAYER SWAP SUCCESS{RESET}")
                    results.append(swap_result)  # Add the main swap result
                    successful += 1
                    total_iterations += swap_result['iterations']

                    apply_polarity_swap(pcb_data, swap_result, pad_swaps, pair_name, polarity_swapped_pairs)
                    add_route_to_pcb_data(pcb_data, swap_result, debug_lines=config.debug_lines)
                    if pair.p_net_id in remaining_net_ids:
                        remaining_net_ids.remove(pair.p_net_id)
                    if pair.n_net_id in remaining_net_ids:
                        remaining_net_ids.remove(pair.n_net_id)
                    routed_net_ids.append(pair.p_net_id)
                    routed_net_ids.append(pair.n_net_id)
                    track_proximity_cache[pair.p_net_id] = compute_track_proximity_for_net(pcb_data, pair.p_net_id, config, layer_map)
                    track_proximity_cache[pair.n_net_id] = compute_track_proximity_for_net(pcb_data, pair.n_net_id, config, layer_map)
                    if swap_result.get('p_path'):
                        routed_net_paths[pair.p_net_id] = swap_result['p_path']
                    if swap_result.get('n_path'):
                        routed_net_paths[pair.n_net_id] = swap_result['n_path']
                    routed_results[pair.p_net_id] = swap_result
                    routed_results[pair.n_net_id] = swap_result
                    diff_pair_by_net_id[pair.p_net_id] = (pair_name, pair)
                    diff_pair_by_net_id[pair.n_net_id] = (pair_name, pair)
                    ripped_up = True  # Prevents "ROUTE FAILED" message

            if not ripped_up:
                print(f"  {RED}ROUTE FAILED - no rippable blockers found{RESET}")
                failed += 1

    # Route single-ended nets
    reroute_index = 0  # Initialize for unified reroute loop at end
    user_quit = False
    for net_name, net_id in single_ended_nets:
        if user_quit:
            break

        route_index += 1
        failed_str = f" ({failed} failed)" if failed > 0 else ""
        print(f"\n[{route_index}/{total_routes}{failed_str}] Routing {net_name} (id={net_id})")
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
        # Use diagonal_margin=0.25 to catch diagonal segments near vias (fixes DRC for single-ended)
        for routed_id in routed_net_ids:
            if visualize:
                add_net_obstacles_with_vis(obstacles, pcb_data, routed_id, config, 0.0,
                                            vis_data.blocked_cells, vis_data.blocked_vias, diagonal_margin=0.25)
            else:
                add_net_stubs_as_obstacles(obstacles, pcb_data, routed_id, config)
                add_net_vias_as_obstacles(obstacles, pcb_data, routed_id, config, diagonal_margin=0.25)
                add_net_pads_as_obstacles(obstacles, pcb_data, routed_id, config)

        # Add GND vias as obstacles (from previous diff pair routing)
        if gnd_net_id is not None:
            add_net_vias_as_obstacles(obstacles, pcb_data, gnd_net_id, config, diagonal_margin=0.25)

        # Add other unrouted nets' stubs, vias, and pads as obstacles (not the current net)
        other_unrouted = [nid for nid in remaining_net_ids if nid != net_id]
        for other_net_id in other_unrouted:
            if visualize:
                add_net_obstacles_with_vis(obstacles, pcb_data, other_net_id, config, 0.0,
                                            vis_data.blocked_cells, vis_data.blocked_vias, diagonal_margin=0.25)
            else:
                add_net_stubs_as_obstacles(obstacles, pcb_data, other_net_id, config)
                add_net_vias_as_obstacles(obstacles, pcb_data, other_net_id, config, diagonal_margin=0.25)
                add_net_pads_as_obstacles(obstacles, pcb_data, other_net_id, config)

        # Add stub proximity costs for ALL unrouted nets in PCB (not just current batch)
        stub_proximity_net_ids = [nid for nid in all_unrouted_net_ids
                                   if nid != net_id and nid not in routed_net_ids]
        unrouted_stubs = get_stub_endpoints(pcb_data, stub_proximity_net_ids)
        if unrouted_stubs:
            add_stub_proximity_costs(obstacles, unrouted_stubs, config)
        # Add track proximity costs for previously routed tracks (same layer only)
        merge_track_proximity_costs(obstacles, track_proximity_cache)

        # Add cross-layer track data for vertical alignment attraction
        add_cross_layer_tracks(obstacles, pcb_data, config, layer_map,
                                exclude_net_ids={net_id})

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
            routed_results[net_id] = result  # Track for summary
            # Track path for blocking analysis
            if result.get('path'):
                routed_net_paths[net_id] = result['path']
            # Add to track proximity cache
            track_proximity_cache[net_id] = compute_track_proximity_for_net(pcb_data, net_id, config, layer_map)
        else:
            iterations = result['iterations'] if result else 0
            print(f"  FAILED: Could not find route ({elapsed:.2f}s)")
            total_iterations += iterations

            # Try rip-up and reroute for single-ended nets (similar to diff pairs)
            ripped_up = False
            if routed_net_paths and result:
                # Find the direction that failed faster (likely more constrained)
                fwd_iters = result.get('iterations_forward', 0)
                bwd_iters = result.get('iterations_backward', 0)
                fwd_cells = result.get('blocked_cells_forward', [])
                bwd_cells = result.get('blocked_cells_backward', [])

                if fwd_iters > 0 and (bwd_iters == 0 or fwd_iters <= bwd_iters):
                    fastest_dir = 'forward'
                    blocked_cells = fwd_cells
                    dir_iters = fwd_iters
                    if blocked_cells:
                        print(f"  {fastest_dir.capitalize()} direction failed fastest ({dir_iters} iterations, {len(blocked_cells)} blocked cells)")
                elif bwd_iters > 0:
                    fastest_dir = 'backward'
                    blocked_cells = bwd_cells
                    dir_iters = bwd_iters
                    if blocked_cells:
                        print(f"  {fastest_dir.capitalize()} direction failed fastest ({dir_iters} iterations, {len(blocked_cells)} blocked cells)")
                else:
                    # Both directions had 0 iterations - combine all blocked cells
                    blocked_cells = list(set(fwd_cells + bwd_cells))

                if blocked_cells:
                    # Get source/target coordinates for blocking analysis
                    from routing_utils import get_net_endpoints as get_single_net_endpoints
                    single_sources, single_targets, _ = get_single_net_endpoints(pcb_data, net_id, config)
                    single_source_xy = None
                    single_target_xy = None
                    if single_sources:
                        single_source_xy = (single_sources[0][3], single_sources[0][4])  # orig_x, orig_y
                    if single_targets:
                        single_target_xy = (single_targets[0][3], single_targets[0][4])  # orig_x, orig_y

                    blockers = analyze_frontier_blocking(
                        blocked_cells, pcb_data, config, routed_net_paths,
                        exclude_net_ids={net_id},
                        target_xy=single_target_xy,
                        source_xy=single_source_xy
                    )
                    print_blocking_analysis(blockers)

                    # Filter to only rippable blockers (those in routed_results)
                    # and deduplicate by diff pair (P and N count as one)
                    rippable_blockers = []
                    seen_canonical_ids = set()
                    for b in blockers:
                        if b.net_id in routed_results:
                            canonical = get_canonical_net_id(b.net_id, diff_pair_by_net_id)
                            if canonical not in seen_canonical_ids:
                                seen_canonical_ids.add(canonical)
                                rippable_blockers.append(b)

                    # Progressive rip-up: try N=1, then N=2, etc up to max_rip_up_count
                    ripped_items = []
                    ripped_canonical_ids = set()  # Track which canonicals have been ripped
                    retry_succeeded = False
                    last_retry_blocked_cells = blocked_cells  # Start with initial failure's blocked cells

                    for N in range(1, config.max_rip_up_count + 1):
                        # For N > 1, re-analyze from the last retry's blocked cells
                        # to find the most blocking net from that specific failure
                        if N > 1 and last_retry_blocked_cells:
                            print(f"  Re-analyzing {len(last_retry_blocked_cells)} blocked cells from N={N-1} retry:")
                            fresh_blockers = analyze_frontier_blocking(
                                last_retry_blocked_cells, pcb_data, config, routed_net_paths,
                                exclude_net_ids={net_id},
                                target_xy=single_target_xy,
                                source_xy=single_source_xy
                            )
                            print_blocking_analysis(fresh_blockers, prefix="    ")
                            # Find the most-blocking net that isn't already ripped
                            next_blocker = None
                            for b in fresh_blockers:
                                if b.net_id in routed_results:
                                    canonical = get_canonical_net_id(b.net_id, diff_pair_by_net_id)
                                    if canonical not in ripped_canonical_ids:
                                        next_blocker = b
                                        break
                            if next_blocker is None:
                                print(f"  No additional rippable blockers from retry analysis")
                                break
                            # Replace the Nth blocker with the one from retry analysis
                            next_canonical = get_canonical_net_id(next_blocker.net_id, diff_pair_by_net_id)
                            if next_canonical not in seen_canonical_ids:
                                seen_canonical_ids.add(next_canonical)
                                rippable_blockers.append(next_blocker)
                            # Find and move it to position N-1 if needed
                            for idx, b in enumerate(rippable_blockers):
                                if get_canonical_net_id(b.net_id, diff_pair_by_net_id) == next_canonical:
                                    if idx != N - 1 and N - 1 < len(rippable_blockers):
                                        rippable_blockers[idx], rippable_blockers[N-1] = rippable_blockers[N-1], rippable_blockers[idx]
                                    break

                        if N > len(rippable_blockers):
                            break  # Not enough blockers to rip

                        # Build frozenset of all N blocker canonicals for loop check
                        blocker_canonicals = frozenset(
                            get_canonical_net_id(rippable_blockers[i].net_id, diff_pair_by_net_id)
                            for i in range(N)
                        )
                        if (net_id, blocker_canonicals) in rip_and_retry_history:
                            blocker_names = []
                            for i in range(N):
                                b = rippable_blockers[i]
                                if b.net_id in diff_pair_by_net_id:
                                    blocker_names.append(diff_pair_by_net_id[b.net_id][0])
                                else:
                                    blocker_names.append(b.net_name)
                            if len(blocker_names) == 1:
                                blockers_str = blocker_names[0]
                            else:
                                blockers_str = "{" + ", ".join(blocker_names) + "}"
                            print(f"  Skipping N={N}: already tried ripping {blockers_str} for {net_name}")
                            continue

                        # Rip up only the new blocker(s) for this N level
                        rip_successful = True
                        new_ripped_this_level = []

                        if N == 1:
                            blocker = rippable_blockers[0]
                            if blocker.net_id in diff_pair_by_net_id:
                                ripped_pair_name_tmp, _ = diff_pair_by_net_id[blocker.net_id]
                                print(f"  Ripping up diff pair {ripped_pair_name_tmp} (P and N) to retry...")
                            else:
                                print(f"  Ripping up {blocker.net_name} to retry...")
                        else:
                            blocker = rippable_blockers[N-1]
                            if blocker.net_id in diff_pair_by_net_id:
                                ripped_pair_name_tmp, _ = diff_pair_by_net_id[blocker.net_id]
                                print(f"  Extending to N={N}: ripping diff pair {ripped_pair_name_tmp}...")
                            else:
                                print(f"  Extending to N={N}: ripping {blocker.net_name}...")

                        for i in range(len(ripped_items), N):
                            blocker = rippable_blockers[i]
                            if blocker.net_id not in routed_results:
                                continue
                            saved_result, ripped_ids, was_in_results = rip_up_net(
                                blocker.net_id, pcb_data, routed_net_ids, routed_net_paths,
                                routed_results, diff_pair_by_net_id, remaining_net_ids,
                                results, config, track_proximity_cache
                            )
                            if saved_result is None:
                                rip_successful = False
                                break
                            ripped_items.append((blocker.net_id, saved_result, ripped_ids, was_in_results))
                            new_ripped_this_level.append((blocker.net_id, saved_result, ripped_ids, was_in_results))
                            ripped_canonical_ids.add(get_canonical_net_id(blocker.net_id, diff_pair_by_net_id))
                            if was_in_results:
                                successful -= 1
                            if blocker.net_id in diff_pair_by_net_id:
                                ripped_pair_name_tmp, _ = diff_pair_by_net_id[blocker.net_id]
                                print(f"    Ripped diff pair {ripped_pair_name_tmp}")
                            else:
                                print(f"    Ripped {blocker.net_name}")

                        if not rip_successful:
                            for rid, saved_result, ripped_ids, was_in_results in reversed(new_ripped_this_level):
                                restore_net(rid, saved_result, ripped_ids, was_in_results,
                                           pcb_data, routed_net_ids, routed_net_paths,
                                           routed_results, diff_pair_by_net_id, remaining_net_ids,
                                           results, config, track_proximity_cache, layer_map)
                                if was_in_results:
                                    successful += 1
                                ripped_items.pop()
                            continue

                        # Rebuild obstacles and retry the current route
                        retry_obstacles = base_obstacles.clone()
                        for routed_id in routed_net_ids:
                            add_net_stubs_as_obstacles(retry_obstacles, pcb_data, routed_id, config)
                            add_net_vias_as_obstacles(retry_obstacles, pcb_data, routed_id, config, diagonal_margin=0.25)
                            add_net_pads_as_obstacles(retry_obstacles, pcb_data, routed_id, config)
                        # Add GND vias as obstacles
                        if gnd_net_id is not None:
                            add_net_vias_as_obstacles(retry_obstacles, pcb_data, gnd_net_id, config, diagonal_margin=0.25)
                        other_unrouted = [nid for nid in remaining_net_ids if nid != net_id]
                        for other_net_id in other_unrouted:
                            add_net_stubs_as_obstacles(retry_obstacles, pcb_data, other_net_id, config)
                            add_net_vias_as_obstacles(retry_obstacles, pcb_data, other_net_id, config, diagonal_margin=0.25)
                            add_net_pads_as_obstacles(retry_obstacles, pcb_data, other_net_id, config)
                        stub_proximity_net_ids = [nid for nid in all_unrouted_net_ids
                                                   if nid != net_id and nid not in routed_net_ids]
                        unrouted_stubs = get_stub_endpoints(pcb_data, stub_proximity_net_ids)
                        if unrouted_stubs:
                            add_stub_proximity_costs(retry_obstacles, unrouted_stubs, config)
                        merge_track_proximity_costs(retry_obstacles, track_proximity_cache)
                        add_same_net_via_clearance(retry_obstacles, pcb_data, net_id, config)

                        retry_result = route_net_with_obstacles(pcb_data, net_id, config, retry_obstacles)

                        if retry_result and not retry_result.get('failed'):
                            print(f"  RETRY SUCCESS (N={N}): {len(retry_result['new_segments'])} segments, {len(retry_result['new_vias'])} vias")
                            results.append(retry_result)
                            successful += 1
                            total_iterations += retry_result['iterations']
                            add_route_to_pcb_data(pcb_data, retry_result, debug_lines=config.debug_lines)
                            if net_id in remaining_net_ids:
                                remaining_net_ids.remove(net_id)
                            routed_net_ids.append(net_id)
                            routed_results[net_id] = retry_result
                            if retry_result.get('path'):
                                routed_net_paths[net_id] = retry_result['path']
                            track_proximity_cache[net_id] = compute_track_proximity_for_net(pcb_data, net_id, config, layer_map)

                            # Queue all ripped-up nets for rerouting and add to history
                            rip_and_retry_history.add((net_id, blocker_canonicals))

                            for rid, saved_result, ripped_ids, was_in_results in ripped_items:
                                if was_in_results:
                                    successful -= 1
                                if rid in diff_pair_by_net_id:
                                    ripped_pair_name_tmp, ripped_pair_tmp = diff_pair_by_net_id[rid]
                                    reroute_queue.append(('diff_pair', ripped_pair_name_tmp, ripped_pair_tmp))
                                else:
                                    ripped_net = pcb_data.nets.get(rid)
                                    ripped_net_name = ripped_net.name if ripped_net else f"Net {rid}"
                                    reroute_queue.append(('single', ripped_net_name, rid))

                            ripped_up = True
                            retry_succeeded = True
                            break  # Success! Exit the N loop
                        else:
                            print(f"  RETRY FAILED (N={N})")

                            # Store blocked cells from retry for next iteration's analysis
                            if retry_result:
                                retry_fwd_cells = retry_result.get('blocked_cells_forward', [])
                                retry_bwd_cells = retry_result.get('blocked_cells_backward', [])
                                last_retry_blocked_cells = list(set(retry_fwd_cells + retry_bwd_cells))
                                if last_retry_blocked_cells:
                                    print(f"    Retry had {len(last_retry_blocked_cells)} blocked cells")
                                else:
                                    print(f"    No blocked cells from retry to analyze")

                    # If all N levels failed, restore all ripped nets
                    if not retry_succeeded and ripped_items:
                        print(f"  {RED}All rip-up attempts failed: Restoring {len(ripped_items)} net(s){RESET}")
                        for rid, saved_result, ripped_ids, was_in_results in reversed(ripped_items):
                            restore_net(rid, saved_result, ripped_ids, was_in_results,
                                       pcb_data, routed_net_ids, routed_net_paths,
                                       routed_results, diff_pair_by_net_id, remaining_net_ids,
                                       results, config, track_proximity_cache, layer_map)
                            if was_in_results:
                                successful += 1

            if not ripped_up:
                print(f"  {RED}ROUTE FAILED - no rippable blockers found{RESET}")
                failed += 1

    # Unified reroute loop - handles all nets ripped during diff pair or single-ended routing
    while reroute_index < len(reroute_queue):
        reroute_item = reroute_queue[reroute_index]
        reroute_index += 1

        if reroute_item[0] == 'single':
            _, ripped_net_name, ripped_net_id = reroute_item

            # Skip if already routed (can happen if rerouted then ripped again, then successfully rerouted by another attempt)
            if ripped_net_id in routed_results:
                continue

            route_index += 1
            current_total = total_routes + len(reroute_queue)
            failed_str = f" ({failed} failed)" if failed > 0 else ""
            print(f"\n[REROUTE {route_index}/{current_total}{failed_str}] Re-routing ripped net {ripped_net_name}")
            print("-" * 40)

            start_time = time.time()
            obstacles = base_obstacles.clone()
            for routed_id in routed_net_ids:
                add_net_stubs_as_obstacles(obstacles, pcb_data, routed_id, config)
                add_net_vias_as_obstacles(obstacles, pcb_data, routed_id, config, diagonal_margin=0.25)
                add_net_pads_as_obstacles(obstacles, pcb_data, routed_id, config)
            # Add GND vias as obstacles
            if gnd_net_id is not None:
                add_net_vias_as_obstacles(obstacles, pcb_data, gnd_net_id, config, diagonal_margin=0.25)
            other_unrouted = [nid for nid in remaining_net_ids if nid != ripped_net_id]
            for other_net_id in other_unrouted:
                add_net_stubs_as_obstacles(obstacles, pcb_data, other_net_id, config)
                add_net_vias_as_obstacles(obstacles, pcb_data, other_net_id, config, diagonal_margin=0.25)
                add_net_pads_as_obstacles(obstacles, pcb_data, other_net_id, config)
            stub_proximity_net_ids = [nid for nid in all_unrouted_net_ids
                                       if nid != ripped_net_id and nid not in routed_net_ids]
            unrouted_stubs = get_stub_endpoints(pcb_data, stub_proximity_net_ids)
            if unrouted_stubs:
                add_stub_proximity_costs(obstacles, unrouted_stubs, config)
            merge_track_proximity_costs(obstacles, track_proximity_cache)
            add_cross_layer_tracks(obstacles, pcb_data, config, layer_map,
                                    exclude_net_ids={ripped_net_id})
            add_same_net_via_clearance(obstacles, pcb_data, ripped_net_id, config)

            result = route_net_with_obstacles(pcb_data, ripped_net_id, config, obstacles)
            elapsed = time.time() - start_time
            total_time += elapsed

            if result and not result.get('failed'):
                print(f"  REROUTE SUCCESS: {len(result['new_segments'])} segments, {len(result['new_vias'])} vias ({elapsed:.2f}s)")
                results.append(result)
                successful += 1
                total_iterations += result['iterations']
                add_route_to_pcb_data(pcb_data, result, debug_lines=config.debug_lines)
                if ripped_net_id in remaining_net_ids:
                    remaining_net_ids.remove(ripped_net_id)
                routed_net_ids.append(ripped_net_id)
                routed_results[ripped_net_id] = result
                if result.get('path'):
                    routed_net_paths[ripped_net_id] = result['path']
                track_proximity_cache[ripped_net_id] = compute_track_proximity_for_net(pcb_data, ripped_net_id, config, layer_map)
            else:
                # Reroute failed - try rip-up and retry
                iterations = result['iterations'] if result else 0
                total_iterations += iterations
                print(f"  REROUTE FAILED: ({elapsed:.2f}s) - attempting rip-up and retry...")

                reroute_succeeded = False
                ripped_items = []  # Initialize here so it's always defined
                if routed_net_paths and result:
                    fwd_cells = result.get('blocked_cells_forward', [])
                    bwd_cells = result.get('blocked_cells_backward', [])
                    blocked_cells = list(set(fwd_cells + bwd_cells))

                    if blocked_cells:
                        # Get source/target coordinates for blocking analysis
                        from routing_utils import get_net_endpoints as get_single_net_endpoints
                        reroute_sources, reroute_targets, _ = get_single_net_endpoints(pcb_data, ripped_net_id, config)
                        reroute_source_xy = None
                        reroute_target_xy = None
                        if reroute_sources:
                            reroute_source_xy = (reroute_sources[0][3], reroute_sources[0][4])  # orig_x, orig_y
                        if reroute_targets:
                            reroute_target_xy = (reroute_targets[0][3], reroute_targets[0][4])  # orig_x, orig_y

                        blockers = analyze_frontier_blocking(
                            blocked_cells, pcb_data, config, routed_net_paths,
                            exclude_net_ids={ripped_net_id},
                            target_xy=reroute_target_xy,
                            source_xy=reroute_source_xy
                        )
                        print_blocking_analysis(blockers)

                        # Filter to only rippable blockers
                        rippable_blockers = []
                        seen_canonical_ids = set()
                        for b in blockers:
                            if b.net_id in routed_results:
                                canonical = get_canonical_net_id(b.net_id, diff_pair_by_net_id)
                                if canonical not in seen_canonical_ids:
                                    seen_canonical_ids.add(canonical)
                                    rippable_blockers.append(b)
                        ripped_canonical_ids = set()  # Track which canonicals have been ripped
                        last_retry_blocked_cells = blocked_cells  # Start with initial failure's blocked cells

                        for N in range(1, config.max_rip_up_count + 1):
                            # For N > 1, re-analyze from the last retry's blocked cells
                            # to find the most blocking net from that specific failure
                            if N > 1 and last_retry_blocked_cells:
                                print(f"  Re-analyzing {len(last_retry_blocked_cells)} blocked cells from N={N-1} retry:")
                                fresh_blockers = analyze_frontier_blocking(
                                    last_retry_blocked_cells, pcb_data, config, routed_net_paths,
                                    exclude_net_ids={ripped_net_id},
                                    target_xy=reroute_target_xy,
                                    source_xy=reroute_source_xy
                                )
                                print_blocking_analysis(fresh_blockers, prefix="    ")
                                # Find the most-blocking net that isn't already ripped
                                next_blocker = None
                                for b in fresh_blockers:
                                    if b.net_id in routed_results:
                                        canonical = get_canonical_net_id(b.net_id, diff_pair_by_net_id)
                                        if canonical not in ripped_canonical_ids:
                                            next_blocker = b
                                            break
                                if next_blocker is None:
                                    print(f"  No additional rippable blockers from retry analysis")
                                    break
                                # Replace the Nth blocker with the one from retry analysis
                                next_canonical = get_canonical_net_id(next_blocker.net_id, diff_pair_by_net_id)
                                if next_canonical not in seen_canonical_ids:
                                    seen_canonical_ids.add(next_canonical)
                                    rippable_blockers.append(next_blocker)
                                # Find and move it to position N-1 if needed
                                for idx, b in enumerate(rippable_blockers):
                                    if get_canonical_net_id(b.net_id, diff_pair_by_net_id) == next_canonical:
                                        if idx != N - 1 and N - 1 < len(rippable_blockers):
                                            rippable_blockers[idx], rippable_blockers[N-1] = rippable_blockers[N-1], rippable_blockers[idx]
                                        break

                            if N > len(rippable_blockers):
                                break

                            blocker_canonicals = frozenset(
                                get_canonical_net_id(rippable_blockers[i].net_id, diff_pair_by_net_id)
                                for i in range(N)
                            )
                            if (ripped_net_id, blocker_canonicals) in rip_and_retry_history:
                                continue

                            rip_successful = True
                            new_ripped_this_level = []

                            if N == 1:
                                blocker = rippable_blockers[0]
                                if blocker.net_id in diff_pair_by_net_id:
                                    ripped_pair_name_tmp, _ = diff_pair_by_net_id[blocker.net_id]
                                    print(f"  Ripping up diff pair {ripped_pair_name_tmp} to retry reroute...")
                                else:
                                    print(f"  Ripping up {blocker.net_name} to retry reroute...")
                            else:
                                blocker = rippable_blockers[N-1]
                                if blocker.net_id in diff_pair_by_net_id:
                                    ripped_pair_name_tmp, _ = diff_pair_by_net_id[blocker.net_id]
                                    print(f"  Extending to N={N}: ripping diff pair {ripped_pair_name_tmp}...")
                                else:
                                    print(f"  Extending to N={N}: ripping {blocker.net_name}...")

                            for i in range(len(ripped_items), N):
                                blocker = rippable_blockers[i]
                                if blocker.net_id not in routed_results:
                                    continue
                                saved_result_tmp, ripped_ids, was_in_results = rip_up_net(
                                    blocker.net_id, pcb_data, routed_net_ids, routed_net_paths,
                                    routed_results, diff_pair_by_net_id, remaining_net_ids,
                                    results, config, track_proximity_cache
                                )
                                if saved_result_tmp is None:
                                    rip_successful = False
                                    break
                                ripped_items.append((blocker.net_id, saved_result_tmp, ripped_ids, was_in_results))
                                new_ripped_this_level.append((blocker.net_id, saved_result_tmp, ripped_ids, was_in_results))
                                ripped_canonical_ids.add(get_canonical_net_id(blocker.net_id, diff_pair_by_net_id))
                                if was_in_results:
                                    successful -= 1

                            if not rip_successful:
                                for net_id_tmp, saved_result_tmp, ripped_ids, was_in_results in reversed(new_ripped_this_level):
                                    restore_net(net_id_tmp, saved_result_tmp, ripped_ids, was_in_results,
                                               pcb_data, routed_net_ids, routed_net_paths,
                                               routed_results, diff_pair_by_net_id, remaining_net_ids,
                                               results, config, track_proximity_cache, layer_map)
                                    if was_in_results:
                                        successful += 1
                                    ripped_items.pop()
                                continue

                            # Rebuild obstacles and retry
                            retry_obstacles = base_obstacles.clone()
                            for routed_id in routed_net_ids:
                                add_net_stubs_as_obstacles(retry_obstacles, pcb_data, routed_id, config)
                                add_net_vias_as_obstacles(retry_obstacles, pcb_data, routed_id, config, diagonal_margin=0.25)
                                add_net_pads_as_obstacles(retry_obstacles, pcb_data, routed_id, config)
                            # Add GND vias as obstacles
                            if gnd_net_id is not None:
                                add_net_vias_as_obstacles(retry_obstacles, pcb_data, gnd_net_id, config, diagonal_margin=0.25)
                            other_unrouted = [nid for nid in remaining_net_ids if nid != ripped_net_id]
                            for other_net_id in other_unrouted:
                                add_net_stubs_as_obstacles(retry_obstacles, pcb_data, other_net_id, config)
                                add_net_vias_as_obstacles(retry_obstacles, pcb_data, other_net_id, config, diagonal_margin=0.25)
                                add_net_pads_as_obstacles(retry_obstacles, pcb_data, other_net_id, config)
                            stub_proximity_net_ids = [nid for nid in all_unrouted_net_ids
                                                       if nid != ripped_net_id and nid not in routed_net_ids]
                            unrouted_stubs = get_stub_endpoints(pcb_data, stub_proximity_net_ids)
                            if unrouted_stubs:
                                add_stub_proximity_costs(retry_obstacles, unrouted_stubs, config)
                            merge_track_proximity_costs(retry_obstacles, track_proximity_cache)
                            add_same_net_via_clearance(retry_obstacles, pcb_data, ripped_net_id, config)

                            retry_result = route_net_with_obstacles(pcb_data, ripped_net_id, config, retry_obstacles)

                            if retry_result and not retry_result.get('failed'):
                                print(f"  REROUTE RETRY SUCCESS (N={N}): {len(retry_result['new_segments'])} segments, {len(retry_result['new_vias'])} vias")
                                results.append(retry_result)
                                successful += 1
                                total_iterations += retry_result['iterations']
                                add_route_to_pcb_data(pcb_data, retry_result, debug_lines=config.debug_lines)
                                if ripped_net_id in remaining_net_ids:
                                    remaining_net_ids.remove(ripped_net_id)
                                routed_net_ids.append(ripped_net_id)
                                routed_results[ripped_net_id] = retry_result
                                if retry_result.get('path'):
                                    routed_net_paths[ripped_net_id] = retry_result['path']
                                track_proximity_cache[ripped_net_id] = compute_track_proximity_for_net(pcb_data, ripped_net_id, config, layer_map)

                                # Queue ripped nets and add to history
                                rip_and_retry_history.add((ripped_net_id, blocker_canonicals))
                                for net_id_tmp, saved_result_tmp, ripped_ids, was_in_results in ripped_items:
                                    if was_in_results:
                                        successful -= 1
                                    if net_id_tmp in diff_pair_by_net_id:
                                        ripped_pair_name_tmp, ripped_pair_tmp = diff_pair_by_net_id[net_id_tmp]
                                        reroute_queue.append(('diff_pair', ripped_pair_name_tmp, ripped_pair_tmp))
                                    else:
                                        net = pcb_data.nets.get(net_id_tmp)
                                        net_name_tmp = net.name if net else f"Net {net_id_tmp}"
                                        reroute_queue.append(('single', net_name_tmp, net_id_tmp))

                                reroute_succeeded = True
                                break
                            else:
                                print(f"  REROUTE RETRY FAILED (N={N})")
                                # Store blocked cells from retry for next iteration's analysis
                                if retry_result:
                                    retry_fwd = retry_result.get('blocked_cells_forward', [])
                                    retry_bwd = retry_result.get('blocked_cells_backward', [])
                                    last_retry_blocked_cells = list(set(retry_fwd + retry_bwd))
                                    if last_retry_blocked_cells:
                                        print(f"    Retry had {len(last_retry_blocked_cells)} blocked cells")
                                    else:
                                        print(f"    No blocked cells from retry to analyze")

                        # If all N levels failed, restore all ripped nets
                        if not reroute_succeeded and ripped_items:
                            print(f"  {RED}All rip-up attempts failed: Restoring {len(ripped_items)} net(s){RESET}")
                            for net_id_tmp, saved_result_tmp, ripped_ids, was_in_results in reversed(ripped_items):
                                restore_net(net_id_tmp, saved_result_tmp, ripped_ids, was_in_results,
                                           pcb_data, routed_net_ids, routed_net_paths,
                                           routed_results, diff_pair_by_net_id, remaining_net_ids,
                                           results, config, track_proximity_cache, layer_map)
                                if was_in_results:
                                    successful += 1

                if not reroute_succeeded:
                    if not ripped_items:
                        print(f"  {RED}ROUTE FAILED - no rippable blockers found{RESET}")
                    failed += 1

        elif reroute_item[0] == 'diff_pair':
            # Handle diff pairs that were ripped during single-ended routing
            _, ripped_pair_name, ripped_pair = reroute_item

            # Skip if already routed (can happen if rerouted then ripped again, then successfully rerouted by another attempt)
            if ripped_pair.p_net_id in routed_results and ripped_pair.n_net_id in routed_results:
                continue

            route_index += 1
            current_total = total_routes + len(reroute_queue)
            failed_str = f" ({failed} failed)" if failed > 0 else ""
            print(f"\n[REROUTE {route_index}/{current_total}{failed_str}] Re-routing ripped diff pair {ripped_pair_name}")
            print("-" * 40)

            start_time = time.time()
            obstacles = diff_pair_base_obstacles.clone()
            for routed_id in routed_net_ids:
                add_net_stubs_as_obstacles(obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
                add_net_vias_as_obstacles(obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
                add_net_pads_as_obstacles(obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
            # Add GND vias as obstacles
            if gnd_net_id is not None:
                add_net_vias_as_obstacles(obstacles, pcb_data, gnd_net_id, config, diff_pair_extra_clearance)
            other_unrouted = [nid for nid in remaining_net_ids
                             if nid != ripped_pair.p_net_id and nid != ripped_pair.n_net_id]
            for other_net_id in other_unrouted:
                add_net_stubs_as_obstacles(obstacles, pcb_data, other_net_id, config, diff_pair_extra_clearance)
                add_net_vias_as_obstacles(obstacles, pcb_data, other_net_id, config, diff_pair_extra_clearance)
                add_net_pads_as_obstacles(obstacles, pcb_data, other_net_id, config, diff_pair_extra_clearance)
            stub_proximity_net_ids = [nid for nid in all_unrouted_net_ids
                                       if nid != ripped_pair.p_net_id and nid != ripped_pair.n_net_id
                                       and nid not in routed_net_ids]
            unrouted_stubs = get_stub_endpoints(pcb_data, stub_proximity_net_ids)
            if unrouted_stubs:
                add_stub_proximity_costs(obstacles, unrouted_stubs, config)
            merge_track_proximity_costs(obstacles, track_proximity_cache)
            add_cross_layer_tracks(obstacles, pcb_data, config, layer_map,
                                    exclude_net_ids={ripped_pair.p_net_id, ripped_pair.n_net_id})
            add_same_net_via_clearance(obstacles, pcb_data, ripped_pair.p_net_id, config)
            add_same_net_via_clearance(obstacles, pcb_data, ripped_pair.n_net_id, config)

            # Get source/target coordinates for blocking analysis
            reroute_sources, reroute_targets, _ = get_diff_pair_endpoints(pcb_data, ripped_pair.p_net_id, ripped_pair.n_net_id, config)
            reroute_source_xy = None
            reroute_target_xy = None
            if reroute_sources:
                src = reroute_sources[0]
                reroute_source_xy = ((src[5] + src[7]) / 2, (src[6] + src[8]) / 2)
            if reroute_targets:
                tgt = reroute_targets[0]
                reroute_target_xy = ((tgt[5] + tgt[7]) / 2, (tgt[6] + tgt[8]) / 2)
            add_own_stubs_as_obstacles_for_diff_pair(obstacles, pcb_data, ripped_pair.p_net_id, ripped_pair.n_net_id, config, diff_pair_extra_clearance)

            result = route_diff_pair_with_obstacles(pcb_data, ripped_pair, config, obstacles, base_obstacles, unrouted_stubs)
            elapsed = time.time() - start_time
            total_time += elapsed

            if result and not result.get('failed') and not result.get('probe_blocked'):
                print(f"  REROUTE SUCCESS: {len(result['new_segments'])} segments, {len(result['new_vias'])} vias ({elapsed:.2f}s)")
                results.append(result)
                successful += 1
                total_iterations += result['iterations']
                rerouted_pairs.add(ripped_pair_name)
                apply_polarity_swap(pcb_data, result, pad_swaps, ripped_pair_name, polarity_swapped_pairs)
                add_route_to_pcb_data(pcb_data, result, debug_lines=config.debug_lines)
                if ripped_pair.p_net_id in remaining_net_ids:
                    remaining_net_ids.remove(ripped_pair.p_net_id)
                if ripped_pair.n_net_id in remaining_net_ids:
                    remaining_net_ids.remove(ripped_pair.n_net_id)
                routed_net_ids.append(ripped_pair.p_net_id)
                routed_net_ids.append(ripped_pair.n_net_id)
                if result.get('p_path'):
                    routed_net_paths[ripped_pair.p_net_id] = result['p_path']
                if result.get('n_path'):
                    routed_net_paths[ripped_pair.n_net_id] = result['n_path']
                routed_results[ripped_pair.p_net_id] = result
                routed_results[ripped_pair.n_net_id] = result
                diff_pair_by_net_id[ripped_pair.p_net_id] = (ripped_pair_name, ripped_pair)
                diff_pair_by_net_id[ripped_pair.n_net_id] = (ripped_pair_name, ripped_pair)
                track_proximity_cache[ripped_pair.p_net_id] = compute_track_proximity_for_net(pcb_data, ripped_pair.p_net_id, config, layer_map)
                track_proximity_cache[ripped_pair.n_net_id] = compute_track_proximity_for_net(pcb_data, ripped_pair.n_net_id, config, layer_map)
            else:
                # Reroute failed - try rip-up and retry
                iterations = result['iterations'] if result else 0
                total_iterations += iterations
                print(f"  REROUTE FAILED: ({elapsed:.2f}s) - attempting rip-up and retry...")

                reroute_succeeded = False
                ripped_items = []  # Initialize here so it's always defined
                if routed_net_paths and result:
                    # Find blocked cells from the failed route
                    if result.get('probe_blocked'):
                        blocked_cells = result.get('blocked_cells', [])
                    else:
                        fwd_iters = result.get('iterations_forward', 0)
                        bwd_iters = result.get('iterations_backward', 0)
                        if fwd_iters > 0 and (bwd_iters == 0 or fwd_iters <= bwd_iters):
                            blocked_cells = result.get('blocked_cells_forward', [])
                        else:
                            blocked_cells = result.get('blocked_cells_backward', [])

                    if not blocked_cells:
                        print(f"  No blocked cells from router - cannot analyze blockers")

                    if blocked_cells:
                        blockers = analyze_frontier_blocking(
                            blocked_cells, pcb_data, config, routed_net_paths,
                            exclude_net_ids={ripped_pair.p_net_id, ripped_pair.n_net_id},
                            extra_clearance=diff_pair_extra_clearance,
                            target_xy=reroute_target_xy,
                            source_xy=reroute_source_xy
                        )
                        print_blocking_analysis(blockers)

                        # Filter to only rippable blockers and deduplicate by diff pair
                        rippable_blockers = []
                        seen_canonical_ids = set()
                        for b in blockers:
                            if b.net_id in routed_results:
                                canonical = get_canonical_net_id(b.net_id, diff_pair_by_net_id)
                                if canonical not in seen_canonical_ids:
                                    seen_canonical_ids.add(canonical)
                                    rippable_blockers.append(b)
                        current_canonical = ripped_pair.p_net_id

                        if blockers and not rippable_blockers:
                            # Show which blockers exist but can't be ripped (not yet routed)
                            unrippable_names = []
                            for b in blockers[:3]:  # Show up to 3
                                if b.net_id in diff_pair_by_net_id:
                                    unrippable_names.append(diff_pair_by_net_id[b.net_id][0])
                                else:
                                    unrippable_names.append(b.net_name)
                            print(f"  No rippable blockers (not yet routed): {', '.join(unrippable_names)}")
                        ripped_canonical_ids = set()  # Track which canonicals have been ripped
                        last_retry_blocked_cells = blocked_cells  # Start with initial failure's blocked cells

                        for N in range(1, config.max_rip_up_count + 1):
                            # For N > 1, re-analyze from the last retry's blocked cells
                            # to find the most blocking net from that specific failure
                            if N > 1 and last_retry_blocked_cells:
                                print(f"  Re-analyzing {len(last_retry_blocked_cells)} blocked cells from N={N-1} retry:")
                                fresh_blockers = analyze_frontier_blocking(
                                    last_retry_blocked_cells, pcb_data, config, routed_net_paths,
                                    exclude_net_ids={ripped_pair.p_net_id, ripped_pair.n_net_id},
                                    extra_clearance=diff_pair_extra_clearance,
                                    target_xy=reroute_target_xy,
                                    source_xy=reroute_source_xy
                                )
                                print_blocking_analysis(fresh_blockers, prefix="    ")
                                # Find the most-blocking net that isn't already ripped
                                next_blocker = None
                                for b in fresh_blockers:
                                    if b.net_id in routed_results:
                                        canonical = get_canonical_net_id(b.net_id, diff_pair_by_net_id)
                                        if canonical not in ripped_canonical_ids:
                                            next_blocker = b
                                            break
                                if next_blocker is None:
                                    print(f"  No additional rippable blockers from retry analysis")
                                    break
                                # Replace the Nth blocker with the one from retry analysis
                                next_canonical = get_canonical_net_id(next_blocker.net_id, diff_pair_by_net_id)
                                if next_canonical not in seen_canonical_ids:
                                    seen_canonical_ids.add(next_canonical)
                                    rippable_blockers.append(next_blocker)
                                # Find and move it to position N-1 if needed
                                for idx, b in enumerate(rippable_blockers):
                                    if get_canonical_net_id(b.net_id, diff_pair_by_net_id) == next_canonical:
                                        if idx != N - 1 and N - 1 < len(rippable_blockers):
                                            rippable_blockers[idx], rippable_blockers[N-1] = rippable_blockers[N-1], rippable_blockers[idx]
                                        break

                            if N > len(rippable_blockers):
                                break

                            blocker_canonicals = frozenset(
                                get_canonical_net_id(rippable_blockers[i].net_id, diff_pair_by_net_id)
                                for i in range(N)
                            )
                            if (current_canonical, blocker_canonicals) in rip_and_retry_history:
                                # Show which blockers we're skipping due to history
                                blocker_names = []
                                for i in range(N):
                                    b = rippable_blockers[i]
                                    if b.net_id in diff_pair_by_net_id:
                                        blocker_names.append(diff_pair_by_net_id[b.net_id][0])
                                    else:
                                        blocker_names.append(b.net_name)
                                print(f"  Skipping rip-up (already tried): {', '.join(blocker_names)}")
                                continue

                            # Rip up blockers
                            rip_successful = True
                            new_ripped_this_level = []

                            if N == 1:
                                blocker = rippable_blockers[0]
                                if blocker.net_id in diff_pair_by_net_id:
                                    ripped_pair_name_tmp, _ = diff_pair_by_net_id[blocker.net_id]
                                    print(f"  Ripping up diff pair {ripped_pair_name_tmp} to retry reroute...")
                                else:
                                    print(f"  Ripping up {blocker.net_name} to retry reroute...")
                            else:
                                blocker = rippable_blockers[N-1]
                                if blocker.net_id in diff_pair_by_net_id:
                                    ripped_pair_name_tmp, _ = diff_pair_by_net_id[blocker.net_id]
                                    print(f"  Extending to N={N}: ripping diff pair {ripped_pair_name_tmp}...")
                                else:
                                    print(f"  Extending to N={N}: ripping {blocker.net_name}...")

                            for i in range(len(ripped_items), N):
                                blocker = rippable_blockers[i]
                                if blocker.net_id not in routed_results:
                                    continue
                                saved_result_tmp, ripped_ids, was_in_results = rip_up_net(
                                    blocker.net_id, pcb_data, routed_net_ids, routed_net_paths,
                                    routed_results, diff_pair_by_net_id, remaining_net_ids,
                                    results, config, track_proximity_cache
                                )
                                if saved_result_tmp is None:
                                    rip_successful = False
                                    break
                                ripped_items.append((blocker.net_id, saved_result_tmp, ripped_ids, was_in_results))
                                new_ripped_this_level.append((blocker.net_id, saved_result_tmp, ripped_ids, was_in_results))
                                ripped_canonical_ids.add(get_canonical_net_id(blocker.net_id, diff_pair_by_net_id))
                                if was_in_results:
                                    successful -= 1

                            if not rip_successful:
                                for net_id_tmp, saved_result_tmp, ripped_ids, was_in_results in reversed(new_ripped_this_level):
                                    restore_net(net_id_tmp, saved_result_tmp, ripped_ids, was_in_results,
                                               pcb_data, routed_net_ids, routed_net_paths,
                                               routed_results, diff_pair_by_net_id, remaining_net_ids,
                                               results, config, track_proximity_cache, layer_map)
                                    if was_in_results:
                                        successful += 1
                                    ripped_items.pop()
                                continue

                            # Rebuild obstacles and retry
                            retry_obstacles = diff_pair_base_obstacles.clone()
                            for routed_id in routed_net_ids:
                                add_net_stubs_as_obstacles(retry_obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
                                add_net_vias_as_obstacles(retry_obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
                                add_net_pads_as_obstacles(retry_obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
                            # Add GND vias as obstacles
                            if gnd_net_id is not None:
                                add_net_vias_as_obstacles(retry_obstacles, pcb_data, gnd_net_id, config, diff_pair_extra_clearance)
                            other_unrouted = [nid for nid in remaining_net_ids
                                             if nid != ripped_pair.p_net_id and nid != ripped_pair.n_net_id]
                            for other_net_id in other_unrouted:
                                add_net_stubs_as_obstacles(retry_obstacles, pcb_data, other_net_id, config, diff_pair_extra_clearance)
                                add_net_vias_as_obstacles(retry_obstacles, pcb_data, other_net_id, config, diff_pair_extra_clearance)
                                add_net_pads_as_obstacles(retry_obstacles, pcb_data, other_net_id, config, diff_pair_extra_clearance)
                            stub_proximity_net_ids = [nid for nid in all_unrouted_net_ids
                                                       if nid != ripped_pair.p_net_id and nid != ripped_pair.n_net_id
                                                       and nid not in routed_net_ids]
                            unrouted_stubs = get_stub_endpoints(pcb_data, stub_proximity_net_ids)
                            if unrouted_stubs:
                                add_stub_proximity_costs(retry_obstacles, unrouted_stubs, config)
                            merge_track_proximity_costs(retry_obstacles, track_proximity_cache)
                            add_same_net_via_clearance(retry_obstacles, pcb_data, ripped_pair.p_net_id, config)
                            add_same_net_via_clearance(retry_obstacles, pcb_data, ripped_pair.n_net_id, config)
                            add_own_stubs_as_obstacles_for_diff_pair(retry_obstacles, pcb_data, ripped_pair.p_net_id, ripped_pair.n_net_id, config, diff_pair_extra_clearance)

                            retry_result = route_diff_pair_with_obstacles(pcb_data, ripped_pair, config, retry_obstacles, base_obstacles, unrouted_stubs)

                            if retry_result and not retry_result.get('failed') and not retry_result.get('probe_blocked'):
                                print(f"  REROUTE RETRY SUCCESS (N={N}): {len(retry_result['new_segments'])} segments, {len(retry_result['new_vias'])} vias")
                                results.append(retry_result)
                                successful += 1
                                total_iterations += retry_result['iterations']
                                rerouted_pairs.add(ripped_pair_name)
                                apply_polarity_swap(pcb_data, retry_result, pad_swaps, ripped_pair_name, polarity_swapped_pairs)
                                add_route_to_pcb_data(pcb_data, retry_result, debug_lines=config.debug_lines)
                                if ripped_pair.p_net_id in remaining_net_ids:
                                    remaining_net_ids.remove(ripped_pair.p_net_id)
                                if ripped_pair.n_net_id in remaining_net_ids:
                                    remaining_net_ids.remove(ripped_pair.n_net_id)
                                routed_net_ids.append(ripped_pair.p_net_id)
                                routed_net_ids.append(ripped_pair.n_net_id)
                                if retry_result.get('p_path'):
                                    routed_net_paths[ripped_pair.p_net_id] = retry_result['p_path']
                                if retry_result.get('n_path'):
                                    routed_net_paths[ripped_pair.n_net_id] = retry_result['n_path']
                                routed_results[ripped_pair.p_net_id] = retry_result
                                routed_results[ripped_pair.n_net_id] = retry_result
                                diff_pair_by_net_id[ripped_pair.p_net_id] = (ripped_pair_name, ripped_pair)
                                diff_pair_by_net_id[ripped_pair.n_net_id] = (ripped_pair_name, ripped_pair)
                                track_proximity_cache[ripped_pair.p_net_id] = compute_track_proximity_for_net(pcb_data, ripped_pair.p_net_id, config, layer_map)
                                track_proximity_cache[ripped_pair.n_net_id] = compute_track_proximity_for_net(pcb_data, ripped_pair.n_net_id, config, layer_map)

                                # Queue ripped nets and add to history
                                rip_and_retry_history.add((current_canonical, blocker_canonicals))
                                for net_id_tmp, saved_result_tmp, ripped_ids, was_in_results in ripped_items:
                                    if was_in_results:
                                        successful -= 1
                                    if net_id_tmp in diff_pair_by_net_id:
                                        ripped_pair_name_tmp, ripped_pair_tmp = diff_pair_by_net_id[net_id_tmp]
                                        reroute_queue.append(('diff_pair', ripped_pair_name_tmp, ripped_pair_tmp))
                                    else:
                                        net = pcb_data.nets.get(net_id_tmp)
                                        net_name_tmp = net.name if net else f"Net {net_id_tmp}"
                                        reroute_queue.append(('single', net_name_tmp, net_id_tmp))

                                reroute_succeeded = True
                                break
                            else:
                                print(f"  REROUTE RETRY FAILED (N={N})")
                                # Store blocked cells from retry for next iteration's analysis
                                if retry_result:
                                    if retry_result.get('probe_blocked'):
                                        last_retry_blocked_cells = retry_result.get('blocked_cells', [])
                                    else:
                                        retry_fwd = retry_result.get('blocked_cells_forward', [])
                                        retry_bwd = retry_result.get('blocked_cells_backward', [])
                                        last_retry_blocked_cells = list(set(retry_fwd + retry_bwd))
                                    if last_retry_blocked_cells:
                                        print(f"    Retry had {len(last_retry_blocked_cells)} blocked cells")
                                    else:
                                        print(f"    No blocked cells from retry to analyze")
                                else:
                                    print(f"    Retry returned no result (endpoint error?)")
                                    last_retry_blocked_cells = []

                        # If all N levels failed, restore all ripped nets
                        if not reroute_succeeded and ripped_items:
                            print(f"  {RED}All rip-up attempts failed: Restoring {len(ripped_items)} net(s){RESET}")
                            for net_id_tmp, saved_result_tmp, ripped_ids, was_in_results in reversed(ripped_items):
                                restore_net(net_id_tmp, saved_result_tmp, ripped_ids, was_in_results,
                                           pcb_data, routed_net_ids, routed_net_paths,
                                           routed_results, diff_pair_by_net_id, remaining_net_ids,
                                           results, config, track_proximity_cache, layer_map)
                                if was_in_results:
                                    successful += 1

                # Fallback layer swap for setback failures in reroute (after rip-up fails)
                if not reroute_succeeded and enable_layer_switch and result:
                    fwd_iters = result.get('iterations_forward', 0)
                    bwd_iters = result.get('iterations_backward', 0)
                    fwd_cells = result.get('blocked_cells_forward', [])
                    bwd_cells = result.get('blocked_cells_backward', [])
                    is_setback_failure = (fwd_iters == 0 and bwd_iters == 0 and (fwd_cells or bwd_cells))

                    if is_setback_failure:
                        print(f"  Trying fallback layer swap for setback failure...")
                        swap_success, swap_result, _, _ = try_fallback_layer_swap(
                            pcb_data, ripped_pair, ripped_pair_name, config,
                            fwd_cells, bwd_cells,
                            diff_pair_base_obstacles, base_obstacles,
                            routed_net_ids, remaining_net_ids,
                            all_unrouted_net_ids, gnd_net_id,
                            track_proximity_cache, diff_pair_extra_clearance,
                            all_swap_vias, all_segment_modifications,
                            all_stubs_by_layer, stub_endpoints_by_layer,
                            routed_net_paths, routed_results, diff_pair_by_net_id, layer_map,
                            target_swaps, results=results)

                        if swap_success and swap_result:
                            print(f"  {GREEN}FALLBACK LAYER SWAP SUCCESS{RESET}")
                            results.append(swap_result)  # Add the main swap result
                            successful += 1
                            total_iterations += swap_result['iterations']
                            rerouted_pairs.add(ripped_pair_name)

                            apply_polarity_swap(pcb_data, swap_result, pad_swaps, ripped_pair_name, polarity_swapped_pairs)
                            add_route_to_pcb_data(pcb_data, swap_result, debug_lines=config.debug_lines)
                            if ripped_pair.p_net_id in remaining_net_ids:
                                remaining_net_ids.remove(ripped_pair.p_net_id)
                            if ripped_pair.n_net_id in remaining_net_ids:
                                remaining_net_ids.remove(ripped_pair.n_net_id)
                            routed_net_ids.append(ripped_pair.p_net_id)
                            routed_net_ids.append(ripped_pair.n_net_id)
                            track_proximity_cache[ripped_pair.p_net_id] = compute_track_proximity_for_net(pcb_data, ripped_pair.p_net_id, config, layer_map)
                            track_proximity_cache[ripped_pair.n_net_id] = compute_track_proximity_for_net(pcb_data, ripped_pair.n_net_id, config, layer_map)
                            if swap_result.get('p_path'):
                                routed_net_paths[ripped_pair.p_net_id] = swap_result['p_path']
                            if swap_result.get('n_path'):
                                routed_net_paths[ripped_pair.n_net_id] = swap_result['n_path']
                            routed_results[ripped_pair.p_net_id] = swap_result
                            routed_results[ripped_pair.n_net_id] = swap_result
                            diff_pair_by_net_id[ripped_pair.p_net_id] = (ripped_pair_name, ripped_pair)
                            diff_pair_by_net_id[ripped_pair.n_net_id] = (ripped_pair_name, ripped_pair)
                            reroute_succeeded = True

                if not reroute_succeeded:
                    if not ripped_items:
                        # No blocked cells or no rippable blockers - print final failure
                        print(f"  {RED}REROUTE FAILED - could not find route{RESET}")
                    # else: Already printed "All rip-up attempts failed" above
                    failed += 1

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

    # Write output file
    write_routed_output(
        input_file, output_file, results,
        all_segment_modifications, all_swap_vias,
        target_swap_info, single_ended_target_swap_info,
        pad_swaps, pcb_data,
        debug_lines=debug_lines,
        exclusion_zone_lines=exclusion_zone_lines,
        boundary_debug_labels=boundary_debug_labels,
        skip_routing=skip_routing
    )

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

    # Stub proximity penalty
    parser.add_argument("--stub-proximity-radius", type=float, default=2.0,
                        help="Radius around stubs to penalize routing in mm (default: 2.0)")
    parser.add_argument("--stub-proximity-cost", type=float, default=0.2,
                        help="Cost penalty near stubs in mm equivalent (default: 0.2)")

    # BGA proximity penalty
    parser.add_argument("--bga-proximity-radius", type=float, default=10.0,
                        help="Radius around BGA edges to penalize routing in mm (default: 10.0)")
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
                vis_callback=vis_callback)

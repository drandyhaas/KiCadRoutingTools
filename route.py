"""
Batch PCB Router using Rust-accelerated A* - Routes single-ended nets sequentially.

For differential pair routing, use route_diff.py instead.

Usage:
    python route.py input.kicad_pcb output.kicad_pcb --nets "Net-(U2A-*)"

Requires the Rust router module. Build it with:
    cd rust_router && cargo build --release
    cp target/release/grid_router.dll grid_router.pyd  # Windows
    cp target/release/libgrid_router.so grid_router.so  # Linux
"""

import sys
import os

# Run startup checks before other imports
from startup_checks import run_all_checks
run_all_checks()

import time
import fnmatch
from typing import List, Optional, Tuple, Dict, Set

from kicad_parser import parse_kicad_pcb, PCBData, Pad
from kicad_writer import (
    generate_segment_sexpr, generate_via_sexpr, generate_gr_line_sexpr, generate_gr_text_sexpr,
    swap_segment_nets_at_positions, swap_via_nets_at_positions, swap_pad_nets_in_content,
    modify_segment_layers
)
from output_writer import write_routed_output
from schematic_updater import apply_swaps_to_schematics

# Import from refactored modules
from routing_config import GridRouteConfig, GridCoord
from connectivity import (
    get_stub_endpoints, find_stub_free_ends, find_connected_groups,
    is_edge_stub, get_net_endpoints, find_connected_segment_positions
)
from net_queries import (
    get_all_unrouted_net_ids, get_chip_pad_positions,
    compute_mps_net_ordering, find_pad_nearest_to_position, find_pad_at_position,
    expand_net_patterns, find_single_ended_nets, identify_power_nets
)
from impedance import calculate_layer_widths_for_impedance, print_impedance_routing_plan
from obstacle_map import (
    build_base_obstacle_map, add_net_stubs_as_obstacles, add_net_pads_as_obstacles,
    add_net_vias_as_obstacles, add_same_net_via_clearance,
    build_base_obstacle_map_with_vis, add_net_obstacles_with_vis, get_net_bounds,
    VisualizationData, draw_exclusion_zones_debug, add_vias_list_as_obstacles, add_segments_list_as_obstacles
)
from obstacle_costs import (
    add_stub_proximity_costs, compute_track_proximity_for_net,
    merge_track_proximity_costs, add_cross_layer_tracks
)
from obstacle_cache import (
    precompute_all_net_obstacles, build_working_obstacle_map, update_net_obstacles_after_routing
)
from single_ended_routing import route_net, route_net_with_obstacles, route_net_with_visualization, route_multipoint_taps
from blocking_analysis import analyze_frontier_blocking, print_blocking_analysis, filter_rippable_blockers
from rip_up_reroute import rip_up_net, restore_net
from layer_swap_optimization import apply_single_ended_layer_swaps
from routing_context import (
    build_single_ended_obstacles,
    record_single_ended_success,
    restore_ripped_net
)
from routing_state import RoutingState, create_routing_state
from memory_debug import (
    get_process_memory_mb, format_memory_stats,
    estimate_net_obstacles_cache_mb, estimate_track_proximity_cache_mb,
    estimate_routed_paths_mb, format_obstacle_map_stats
)
from single_ended_loop import route_single_ended_nets
from reroute_loop import run_reroute_loop
from phase3_routing import run_phase3_tap_routing
from net_ordering import order_nets_mps, order_nets_inside_out
from routing_common import (
    setup_bga_exclusion_zones, resolve_net_ids, filter_already_routed,
    build_obstacle_infrastructure, run_length_matching, sync_pcb_data_segments,
    get_common_config_kwargs
)
import re
from terminal_colors import RED, RESET
from routing_constants import DEFAULT_4_LAYER_STACK, POWER_NET_EXCLUSION_PATTERNS

# Import Rust router (startup_checks ensures it's available and up-to-date)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))
from grid_router import GridObstacleMap, GridRouter


def batch_route(input_file: str, output_file: str, net_names: List[str],
                layers: List[str] = None,
                bga_exclusion_zones: Optional[List[Tuple[float, float, float, float]]] = None,
                direction_order: str = None,
                ordering_strategy: str = "inside_out",
                disable_bga_zones: Optional[List[str]] = None,
                track_width: float = 0.1,
                impedance: Optional[float] = None,
                power_nets: Optional[List[str]] = None,
                power_nets_widths: Optional[List[float]] = None,
                clearance: float = 0.1,
                via_size: float = 0.3,
                via_drill: float = 0.2,
                grid_step: float = 0.1,
                via_cost: int = 50,
                max_iterations: int = 200000,
                max_probe_iterations: int = 5000,
                heuristic_weight: float = 1.9,
                turn_cost: int = 1000,
                stub_proximity_radius: float = 2.0,
                stub_proximity_cost: float = 0.2,
                via_proximity_cost: float = 10.0,
                bga_proximity_radius: float = 7.0,
                bga_proximity_cost: float = 0.2,
                track_proximity_distance: float = 2.0,
                track_proximity_cost: float = 0.2,
                debug_lines: bool = False,
                verbose: bool = False,
                max_rip_up_count: int = 3,
                enable_layer_switch: bool = True,
                crossing_layer_check: bool = True,
                can_swap_to_top_layer: bool = False,
                swappable_net_patterns: Optional[List[str]] = None,
                crossing_penalty: float = 1000.0,
                mps_unroll: bool = True,
                skip_routing: bool = False,
                routing_clearance_margin: float = 1.0,
                hole_to_hole_clearance: float = 0.2,
                board_edge_clearance: float = 0.0,
                vertical_attraction_radius: float = 1.0,
                vertical_attraction_cost: float = 0.1,
                length_match_groups: Optional[List[List[str]]] = None,
                length_match_tolerance: float = 0.1,
                meander_amplitude: float = 1.0,
                debug_memory: bool = False,
                mps_reverse_rounds: bool = False,
                mps_layer_swap: bool = False,
                mps_segment_intersection: bool = False,
                minimal_obstacle_cache: bool = False,
                vis_callback=None,
                schematic_dir: Optional[str] = None,
                layer_costs: Optional[List[float]] = None) -> Tuple[int, int, float]:
    """
    Route single-ended nets using the Rust router.

    For differential pair routing, use route_diff.py instead.

    Args:
        input_file: Path to input KiCad PCB file
        output_file: Path to output KiCad PCB file
        net_names: List of net names to route
        layers: List of copper layers to route on (must be specified - cannot auto-detect
                which layers are ground planes vs signal layers)
        bga_exclusion_zones: Optional list of BGA exclusion zones (auto-detected if None)
        direction_order: Direction search order - "forward" or "backward"
                        (None = use GridRouteConfig default)
        ordering_strategy: Net ordering strategy:
            - "mps": Use Maximum Planar Subset algorithm to minimize crossing conflicts (default)
            - "inside_out": Sort BGA nets by distance from BGA center
            - "original": Keep nets in original order
        track_width: Track width in mm (default: 0.1)
        clearance: Clearance between tracks in mm (default: 0.1)
        via_size: Via outer diameter in mm (default: 0.3)
        via_drill: Via drill size in mm (default: 0.2)
        grid_step: Grid resolution in mm (default: 0.1)
        via_cost: Penalty for placing a via in grid steps (default: 50)
        max_iterations: Max A* iterations before giving up (default: 200000)
        heuristic_weight: A* heuristic weight, higher=faster but less optimal (default: 1.9)
        stub_proximity_radius: Radius around stubs to penalize in mm (default: 2.0)
        stub_proximity_cost: Cost penalty near stubs in mm equivalent (default: 0.2)
        bga_proximity_radius: Radius around BGA edges to penalize in mm (default: 7.0)
        bga_proximity_cost: Cost penalty near BGA edges in mm equivalent (default: 0.2)
        debug_lines: Output debug geometry on User.2/3/8/9 layers
        minimal_obstacle_cache: If True, only build obstacle cache for nets being routed
                               (faster when re-routing a small number of nets)
        vis_callback: Optional visualization callback (implements VisualizationCallback protocol)

    Returns:
        (successful_count, failed_count, total_time)
    """
    visualize = vis_callback is not None

    # Track memory if debug_memory enabled
    mem_start = get_process_memory_mb() if debug_memory else 0.0
    if debug_memory:
        print(format_memory_stats("Initial memory", mem_start))

    print(f"Loading {input_file}...")
    pcb_data = parse_kicad_pcb(input_file)

    # Layers must be specified - we can't auto-detect which are ground planes
    if layers is None:
        layers = DEFAULT_4_LAYER_STACK
    print(f"Using {len(layers)} routing layers: {layers}")

    # Set default layer costs if not specified
    # 4+ layers: all 1.0 (inner layers available for routing)
    # 2 layers: F.Cu=1.0, B.Cu=3.0 (prefer top layer)
    if not layer_costs:
        if len(layers) >= 4:
            layer_costs = [1.0] * len(layers)
        else:
            layer_costs = [1.0 if layer == 'F.Cu' else 3.0 for layer in layers]

    # Validate layer costs are in range [1.0, 1000]
    for i, cost in enumerate(layer_costs):
        if cost < 1.0 or cost > 1000:
            layer_name = layers[i] if i < len(layers) else f"layer {i}"
            print(f"ERROR: Layer cost for {layer_name} must be between 1.0 and 1000, got {cost}")
            sys.exit(1)

    costs_str = ', '.join(f"{layers[i]}={layer_costs[i]}x" for i in range(min(len(layers), len(layer_costs))))
    print(f"  Layer costs: {costs_str}")

    # Calculate layer-specific widths for impedance-controlled routing
    layer_widths = {}
    if impedance is not None:
        if not pcb_data.board_info.stackup:
            print("WARNING: No stackup found in PCB file. Using fixed track width.")
        else:
            print(f"\nCalculating trace widths for {impedance}Ω single-ended impedance...")
            layer_widths = calculate_layer_widths_for_impedance(
                pcb_data, layers, impedance,
                spacing=0.0, is_differential=False,
                fallback_width=track_width,
                min_width=track_width
            )
            print_impedance_routing_plan(pcb_data, layers, impedance, is_differential=False,
                                        min_width=track_width)

    # Auto-detect BGA exclusion zones if not specified
    bga_exclusion_zones = setup_bga_exclusion_zones(pcb_data, disable_bga_zones, bga_exclusion_zones)

    config_kwargs = get_common_config_kwargs(
        track_width=track_width, clearance=clearance, via_size=via_size,
        via_drill=via_drill, grid_step=grid_step, via_cost=via_cost,
        layers=layers, max_iterations=max_iterations,
        max_probe_iterations=max_probe_iterations, heuristic_weight=heuristic_weight,
        turn_cost=turn_cost, bga_exclusion_zones=bga_exclusion_zones,
        stub_proximity_radius=stub_proximity_radius, stub_proximity_cost=stub_proximity_cost,
        via_proximity_cost=via_proximity_cost, bga_proximity_radius=bga_proximity_radius,
        bga_proximity_cost=bga_proximity_cost, track_proximity_distance=track_proximity_distance,
        track_proximity_cost=track_proximity_cost, debug_lines=debug_lines, verbose=verbose,
        max_rip_up_count=max_rip_up_count, crossing_penalty=crossing_penalty,
        crossing_layer_check=crossing_layer_check, routing_clearance_margin=routing_clearance_margin,
        hole_to_hole_clearance=hole_to_hole_clearance, board_edge_clearance=board_edge_clearance,
        vertical_attraction_radius=vertical_attraction_radius,
        vertical_attraction_cost=vertical_attraction_cost, length_match_groups=length_match_groups,
        length_match_tolerance=length_match_tolerance, meander_amplitude=meander_amplitude,
        debug_memory=debug_memory, layer_costs=layer_costs
    )
    if direction_order is not None:
        config_kwargs['direction_order'] = direction_order
    if layer_widths:
        config_kwargs['layer_widths'] = layer_widths
        config_kwargs['impedance_target'] = impedance
    config = GridRouteConfig(**config_kwargs)

    # Identify power nets and set up per-net widths
    if power_nets and power_nets_widths:
        if len(power_nets) != len(power_nets_widths):
            raise ValueError(f"--power-nets ({len(power_nets)}) and --power-nets-widths ({len(power_nets_widths)}) must have same length")
        power_net_widths = identify_power_nets(pcb_data, power_nets, power_nets_widths)
        if power_net_widths:
            config.power_net_widths = power_net_widths
            print(f"\nPower net width assignments ({len(power_net_widths)} nets):")
            # Group by width for summary
            width_groups: Dict[float, List[str]] = {}
            for net_id, width in power_net_widths.items():
                net_name = pcb_data.nets[net_id].name if net_id in pcb_data.nets else f"Net {net_id}"
                if width not in width_groups:
                    width_groups[width] = []
                width_groups[width].append(net_name)
            for width, names in sorted(width_groups.items()):
                if len(names) <= 5:
                    print(f"  {width}mm: {', '.join(names)}")
                else:
                    print(f"  {width}mm: {len(names)} nets ({', '.join(names[:3])}...)")

    # Find net IDs and filter already-routed nets
    net_ids = resolve_net_ids(pcb_data, net_names)
    if not net_ids:
        print("No valid nets to route!")
        return 0, 0, 0.0

    net_ids, _ = filter_already_routed(pcb_data, net_ids, config)
    if not net_ids:
        print("All nets are already fully connected - nothing to route!")
        return 0, 0, 0.0

    # Track all segment layer modifications for file output
    all_segment_modifications = []
    # Track all vias added during stub layer swapping
    all_swap_vias = []
    # Track total number of layer swaps applied
    total_layer_swaps = 0

    # Apply target swaps for single-ended swappable-nets
    single_ended_target_swaps: Dict[str, str] = {}
    single_ended_target_swap_info: List[Dict] = []
    boundary_debug_labels: List[Dict] = []  # Debug labels for boundary positions
    if swappable_net_patterns:
        from target_swap import apply_single_ended_target_swaps

        # Find matching single-ended nets
        swappable_se_nets = find_single_ended_nets(
            pcb_data,
            swappable_net_patterns,
            exclude_net_ids=set()
        )

        if len(swappable_se_nets) >= 2:
            print(f"\nAnalyzing target swaps for {len(swappable_se_nets)} single-ended net(s)...")
            single_ended_target_swaps, single_ended_target_swap_info = apply_single_ended_target_swaps(
                pcb_data, swappable_se_nets, config,
                lambda net_id: get_net_endpoints(pcb_data, net_id, config),
                use_boundary_ordering=mps_unroll
            )

    # Single-ended layer swap optimization (before MPS ordering)
    all_stubs_by_layer = {}
    stub_endpoints_by_layer = {}
    if enable_layer_switch and net_ids:
        total_layer_swaps += apply_single_ended_layer_swaps(
            pcb_data, config, net_ids,
            can_swap_to_top_layer, all_segment_modifications, all_swap_vias,
            verbose=verbose
        )
        # Add stub swap vias to pcb_data so routing and length matching see them as obstacles
        for via in all_swap_vias:
            pcb_data.vias.append(via)

    # Apply net ordering strategy
    if ordering_strategy == "mps":
        net_ids, mps_layer_swaps = order_nets_mps(
            pcb_data=pcb_data,
            net_ids=net_ids,
            diff_pairs={},
            mps_unroll=mps_unroll,
            bga_exclusion_zones=bga_exclusion_zones,
            mps_reverse_rounds=mps_reverse_rounds,
            crossing_layer_check=crossing_layer_check,
            mps_segment_intersection=mps_segment_intersection,
            mps_layer_swap=mps_layer_swap,
            enable_layer_switch=enable_layer_switch,
            config=config,
            can_swap_to_top_layer=can_swap_to_top_layer,
            all_segment_modifications=all_segment_modifications,
            all_swap_vias=all_swap_vias,
            all_stubs_by_layer=all_stubs_by_layer,
            stub_endpoints_by_layer=stub_endpoints_by_layer,
            verbose=verbose
        )
        total_layer_swaps += mps_layer_swaps

    elif ordering_strategy == "inside_out" and bga_exclusion_zones:
        net_ids = order_nets_inside_out(pcb_data, net_ids, bga_exclusion_zones)

    elif ordering_strategy == "original":
        print("\nUsing original net order (no sorting)")

    # All nets are single-ended in this tool
    single_ended_nets = net_ids
    total_routes = len(single_ended_nets)

    # Generate stub position labels for single-ended nets (when debug_lines enabled)
    if debug_lines and single_ended_nets:
        from target_swap import generate_single_ended_debug_labels
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
        single_ended_nets = []
    else:
        print(f"\nRouting {total_routes} single-ended net(s)...")
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
    if debug_memory:
        mem_after_base = get_process_memory_mb()
        print(format_memory_stats("After base obstacle map", mem_after_base, mem_after_base - mem_start))

    # Save original (pre-routing) segment signatures to preserve stubs during sync
    # We use object identity since segments are mutable and could be duplicated
    original_segment_ids = set(id(s) for s in pcb_data.segments)

    # Notify visualization callback that routing is starting
    if visualize:
        vis_callback.on_routing_start(total_routes, layers, grid_step)

    # Get unrouted nets for stub proximity costs
    # Use sorted list for deterministic iteration order
    if minimal_obstacle_cache:
        # Only consider nets we're routing (faster for re-routing a few nets)
        all_unrouted_net_ids = sorted(all_net_ids_to_route)
        print(f"Using minimal obstacle cache for {len(all_unrouted_net_ids)} nets being routed")
    else:
        # All unrouted nets in the PCB (full analysis for stub proximity)
        all_unrouted_net_ids = sorted(set(get_all_unrouted_net_ids(pcb_data)))
        print(f"Found {len(all_unrouted_net_ids)} unrouted nets in PCB for stub proximity")

    # Get exclusion zone lines for User.5 if debug_lines is enabled
    exclusion_zone_lines = []
    if debug_lines:
        all_unrouted_stubs = get_stub_endpoints(pcb_data, list(all_unrouted_net_ids))
        all_chip_pads = get_chip_pad_positions(pcb_data, list(all_unrouted_net_ids))
        all_proximity_points = all_unrouted_stubs + all_chip_pads
        exclusion_zone_lines = draw_exclusion_zones_debug(config, all_proximity_points)
        print(f"Will draw {len(config.bga_exclusion_zones)} BGA zones and {len(all_proximity_points)} stub/pad proximity circles on User.5")

    # Pre-compute net obstacles for caching (speeds up per-route setup)
    print("Pre-computing net obstacle cache...")
    cache_start = time.time()
    net_obstacles_cache = precompute_all_net_obstacles(
        pcb_data, list(all_unrouted_net_ids), config,
        extra_clearance=0.0, diagonal_margin=0.25
    )
    cache_time = time.time() - cache_start
    print(f"Net obstacle cache built in {cache_time:.2f}s ({len(net_obstacles_cache)} nets)")
    if debug_memory:
        mem_after_cache = get_process_memory_mb()
        cache_size = estimate_net_obstacles_cache_mb(net_obstacles_cache)
        print(format_memory_stats("After net obstacles cache", mem_after_cache, mem_after_cache - mem_start))
        print(f"[MEMORY]   Cache estimated size: {cache_size:.1f} MB for {len(net_obstacles_cache)} nets")

    # Build working obstacle map (base + all nets) for incremental updates
    # Uses reference counting in Rust to correctly handle cells blocked by multiple nets
    working_obstacles = build_working_obstacle_map(base_obstacles, net_obstacles_cache)
    # Shrink internal allocations to reduce memory footprint
    working_obstacles.shrink_to_fit()
    if debug_memory:
        mem_after_working = get_process_memory_mb()
        print(format_memory_stats("After working obstacle map", mem_after_working, mem_after_working - mem_start))

    # Create routing state object to hold all shared state
    state = create_routing_state(
        pcb_data=pcb_data,
        config=config,
        all_net_ids_to_route=all_net_ids_to_route,
        base_obstacles=base_obstacles,
        diff_pair_base_obstacles=None,  # Not used for single-ended
        diff_pair_extra_clearance=0.0,
        gnd_net_id=None,  # Not used for single-ended
        all_unrouted_net_ids=all_unrouted_net_ids,
        total_routes=total_routes,
        enable_layer_switch=enable_layer_switch,
        debug_lines=debug_lines,
        target_swaps={},  # Diff pair swaps not used
        target_swap_info=[],
        single_ended_target_swaps=single_ended_target_swaps,
        single_ended_target_swap_info=single_ended_target_swap_info,
        all_segment_modifications=all_segment_modifications,
        all_swap_vias=all_swap_vias,
        total_layer_swaps=total_layer_swaps,
        net_obstacles_cache=net_obstacles_cache,
        working_obstacles=working_obstacles,
    )

    # Create local aliases for frequently-used state fields
    routed_net_ids = state.routed_net_ids
    routed_net_paths = state.routed_net_paths
    routed_results = state.routed_results
    track_proximity_cache = state.track_proximity_cache
    layer_map = state.layer_map
    ripup_success_pairs = state.ripup_success_pairs
    rerouted_pairs = state.rerouted_pairs
    remaining_net_ids = state.remaining_net_ids
    results = state.results

    # Counters (kept as locals, not aliased from state)
    route_index = 0

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
        run_length_matching(routed_results, length_match_groups, config, pcb_data)

    # Sync pcb_data with length-matched segments before Phase 3
    # This ensures tap routes see meanders from other nets as obstacles
    sync_pcb_data_segments(pcb_data, routed_results, original_segment_ids, state, config)

    # Phase 3: Complete multi-point routing (tap connections)
    # This happens AFTER length matching so tap routes connect to meandered main routes
    run_phase3_tap_routing(
        state=state,
        pcb_data=pcb_data,
        config=config,
        base_obstacles=base_obstacles,
        gnd_net_id=None,  # Not used for single-ended
        all_unrouted_net_ids=all_unrouted_net_ids,
        routed_net_ids=routed_net_ids,
        remaining_net_ids=remaining_net_ids,
        routed_net_paths=routed_net_paths,
        routed_results=routed_results,
        diff_pair_by_net_id=state.diff_pair_by_net_id,  # Empty for single-ended
        results=results,
        track_proximity_cache=track_proximity_cache,
        layer_map=layer_map
    )

    # Notify visualization callback that all routing is complete
    if visualize:
        vis_callback.on_routing_complete(successful, failed, total_iterations)

    # Build summary data
    import json
    routed_single = []
    failed_single = []
    for net_name, net_id in single_ended_nets:
        if net_id in routed_results:
            routed_single.append(net_name)
        else:
            failed_single.append(net_name)

    # Collect multi-point tap routing stats
    tap_pads_connected = 0
    tap_pads_total = 0
    tap_edges_routed = 0
    tap_edges_failed = 0
    multipoint_nets = 0
    for net_id, result in routed_results.items():
        if result.get('is_multipoint'):
            multipoint_nets += 1
            tap_pads_connected += result.get('tap_pads_connected', 0)
            tap_pads_total += result.get('tap_pads_total', 0)
            tap_edges_routed += result.get('tap_edges_routed', 0)
            tap_edges_failed += result.get('tap_edges_failed', 0)
    # Count total vias from results
    total_vias = sum(len(r.get('new_vias', [])) for r in results)

    # Print human-readable summary
    print("\n" + "=" * 60)
    print("Routing complete")
    print("=" * 60)
    if single_ended_nets:
        if failed_single:
            print(f"  {RED}Single-ended:  {len(routed_single)}/{len(single_ended_nets)} routed ({len(failed_single)} FAILED){RESET}")
        else:
            print(f"  Single-ended:  {len(routed_single)}/{len(single_ended_nets)} routed")
    if multipoint_nets > 0:
        tap_pads_failed = tap_pads_total - tap_pads_connected
        if tap_pads_failed > 0:
            print(f"  {RED}Multi-point:   {tap_pads_connected}/{tap_pads_total} pads connected ({tap_pads_failed} FAILED){RESET}")
        else:
            print(f"  Multi-point:   {tap_pads_connected}/{tap_pads_total} pads connected ({multipoint_nets} nets)")
    if ripup_success_pairs:
        print(f"  Rip-up success: {len(ripup_success_pairs)} (routes that ripped blockers)")
    if rerouted_pairs:
        print(f"  Rerouted:      {len(rerouted_pairs)} (ripped nets re-routed)")
    if single_ended_target_swaps:
        swap_pairs = [(k, v) for k, v in single_ended_target_swaps.items() if k < v]
        print(f"  Target swaps:  {len(swap_pairs)}")
    print(f"  Total vias:    {total_vias}")
    print(f"  Total time:    {total_time:.2f}s")
    print(f"  Iterations:    {total_iterations:,}")
    summary = {
        'routed_single': routed_single,
        'failed_single': failed_single,
        'multipoint_nets': multipoint_nets,
        'multipoint_pads_connected': tap_pads_connected,
        'multipoint_pads_total': tap_pads_total,
        'multipoint_edges_routed': tap_edges_routed,
        'multipoint_edges_failed': tap_edges_failed,
        'ripup_success_pairs': sorted(ripup_success_pairs),
        'rerouted_pairs': sorted(rerouted_pairs),
        'single_ended_target_swaps': [{'net1': k, 'net2': v} for k, v in single_ended_target_swaps.items() if k < v],
        'layer_swaps': total_layer_swaps,
        'successful': successful,
        'failed': failed,
        'total_time': round(total_time, 2),
        'total_iterations': total_iterations,
        'total_vias': total_vias
    }
    print(f"JSON_SUMMARY: {json.dumps(summary)}")

    # Write output file using extracted output_writer module
    write_routed_output(
        input_file=input_file,
        output_file=output_file,
        results=results,
        all_segment_modifications=all_segment_modifications,
        all_swap_vias=all_swap_vias,
        target_swap_info=[],
        single_ended_target_swap_info=single_ended_target_swap_info,
        pad_swaps=pad_swaps,
        pcb_data=pcb_data,
        debug_lines=debug_lines,
        exclusion_zone_lines=exclusion_zone_lines,
        boundary_debug_labels=boundary_debug_labels,
        skip_routing=skip_routing
    )

    # Update schematics with swap info if directory specified
    if schematic_dir and single_ended_target_swap_info:
        # Convert swap info to format for schematic updater
        schematic_swaps = []
        for info in single_ended_target_swap_info:
            if info.get('n1_pad') and info.get('n2_pad'):
                pad1 = info['n1_pad']
                pad2 = info['n2_pad']
                schematic_swaps.append({
                    'component_ref': pad1.component_ref,
                    'pad1': pad1.pad_number,
                    'pad2': pad2.pad_number
                })
        if schematic_swaps:
            apply_swaps_to_schematics(schematic_dir, schematic_swaps, verbose=verbose)

    # Final memory summary
    if debug_memory:
        final_mem = get_process_memory_mb()
        print("\n" + "=" * 60)
        print("[MEMORY] Final Memory Summary")
        print("=" * 60)
        print(f"  Process RSS: {final_mem:.1f} MB (delta: {final_mem - mem_start:+.1f} MB)")
        print(f"  Net obstacles cache: {estimate_net_obstacles_cache_mb(state.net_obstacles_cache):.1f} MB ({len(state.net_obstacles_cache)} nets)")
        print(f"  Track proximity cache: {estimate_track_proximity_cache_mb(state.track_proximity_cache):.1f} MB ({len(state.track_proximity_cache)} nets)")
        print(f"  Routed paths: {estimate_routed_paths_mb(state.routed_net_paths):.1f} MB ({len(state.routed_net_paths)} nets)")
        print(f"  Routed results: {len(state.routed_results)} nets")
        print(format_obstacle_map_stats(state.working_obstacles))
        print("=" * 60)

    return successful, failed, total_time

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Batch PCB Router - Routes single-ended nets using Rust-accelerated A*. For differential pairs, use route_diff.py.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Wildcard patterns supported:
  "Net-(U2A-DATA_*)"  - matches Net-(U2A-DATA_0), Net-(U2A-DATA_1), etc.
  "Net-(*CLK*)"       - matches any net containing CLK

Examples:
  python route.py fanout_starting_point.kicad_pcb routed.kicad_pcb "Net-(U2A-DATA_*)"
  python route.py input.kicad_pcb output.kicad_pcb "Net-(U2A-DATA_*)" --ordering mps

For differential pair routing, use route_diff.py:
  python route_diff.py input.kicad_pcb output.kicad_pcb --nets "*lvds*"
"""
    )
    parser.add_argument("input_file", help="Input KiCad PCB file")
    parser.add_argument("output_file", nargs="?", help="Output KiCad PCB file (default: input_routed.kicad_pcb)")
    parser.add_argument("net_patterns", nargs="*", help="Net names or wildcard patterns to route (default: '*' = all nets)")
    parser.add_argument("--nets", "-n", nargs="+", help="Net names or wildcard patterns to route (alternative to positional args)")
    parser.add_argument("--overwrite", "-O", action="store_true",
                        help="Overwrite input file instead of creating _routed copy")
    parser.add_argument("--component", "-C", help="Route all nets connected to this component (e.g., U1). Excludes GND/VCC/VDD unless net patterns also specified.")
    # Ordering and strategy options
    parser.add_argument("--ordering", "-o", choices=["inside_out", "mps", "original"],
                        default="mps",
                        help="Net ordering strategy: mps (default, crossing conflicts), inside_out, or original")
    parser.add_argument("--direction", "-d", choices=["forward", "backward"],
                        default=None,
                        help="Direction search order for each net route")
    parser.add_argument("--no-bga-zones", nargs="*", default=None,
                        help="Disable BGA exclusion zones. No args = disable all. With component refs (e.g., U1 U3) = disable only those.")
    parser.add_argument("--layers", "-l", nargs="+",
                        default=['F.Cu', 'B.Cu'],
                        help="Routing layers to use (default: F.Cu B.Cu)")

    # Track and via geometry
    parser.add_argument("--track-width", type=float, default=0.3,
                        help="Track width in mm (default: 0.3). Ignored if --impedance is specified.")
    parser.add_argument("--impedance", type=float, default=None,
                        help="Target single-ended impedance in ohms (e.g., 50). Calculates track width per layer from board stackup.")
    parser.add_argument("--clearance", type=float, default=0.25,
                        help="Clearance between tracks in mm (default: 0.25)")
    parser.add_argument("--via-size", type=float, default=0.5,
                        help="Via outer diameter in mm (default: 0.5)")
    parser.add_argument("--via-drill", type=float, default=0.3,
                        help="Via drill size in mm (default: 0.3)")

    # Power net routing options
    parser.add_argument("--power-nets", nargs="*", default=[],
                        help="Glob patterns for power nets (e.g., '*GND*' '*VCC*'). Must pair with --power-nets-widths.")
    parser.add_argument("--power-nets-widths", nargs="*", type=float, default=[],
                        help="Track widths in mm for each power-net pattern (must match --power-nets length)")

    # Router algorithm parameters
    parser.add_argument("--grid-step", type=float, default=0.1,
                        help="Grid resolution in mm (default: 0.1)")
    parser.add_argument("--via-cost", type=int, default=50,
                        help="Penalty for placing a via in grid steps (default: 50)")
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

    # Layer swap and target swap options
    parser.add_argument("--no-stub-layer-swap", action="store_true",
                        help="Disable stub layer switching optimization (enabled by default)")
    parser.add_argument("--no-crossing-layer-check", action="store_true",
                        help="Count crossings regardless of layer overlap (by default, only same-layer crossings count)")
    parser.add_argument("--can-swap-to-top-layer", action="store_true",
                        help="Allow swapping stubs to F.Cu (top layer). Off by default due to via clearance issues.")
    parser.add_argument("--swappable-nets", nargs="+",
                        help="Glob patterns for nets that can have targets swapped (e.g., '*DATA_*')")
    parser.add_argument("--schematic-dir", default=None,
                        help="Directory containing .kicad_sch files to update with pad swaps (default: no schematic update)")
    parser.add_argument("--crossing-penalty", type=float, default=1000.0,
                        help="Penalty for crossing assignments in target swap optimization (default: 1000.0)")
    parser.add_argument("--mps-reverse-rounds", action="store_true",
                        help="Reverse MPS round order: route most-conflicting groups first instead of least-conflicting")
    parser.add_argument("--mps-layer-swap", action="store_true",
                        help="Enable MPS-aware layer swaps to reduce crossing conflicts")
    parser.add_argument("--mps-segment-intersection", action="store_true",
                        help="Force MPS to use segment intersection for crossing detection (auto-enabled when no BGA chips)")

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
    parser.add_argument("--routing-clearance-margin", type=float, default=1.0,
                        help="Multiplier on track-via clearance (1.0 = minimum DRC)")
    parser.add_argument("--hole-to-hole-clearance", type=float, default=0.2,
                        help="Minimum clearance between drill holes in mm (default: 0.2)")
    parser.add_argument("--board-edge-clearance", type=float, default=0.0,
                        help="Clearance from board edge in mm (default: 0 = use track clearance)")

    # Vertical alignment attraction options
    parser.add_argument("--vertical-attraction-radius", type=float, default=1.0,
                        help="Radius in mm for cross-layer track attraction (0 = disabled, default: 1.0)")
    parser.add_argument("--vertical-attraction-cost", type=float, default=0.1,
                        help="Cost bonus in mm equivalent for tracks aligned with other layers (default: 0.1)")

    # Layer preference options
    parser.add_argument("--layer-costs", nargs="+", type=float, default=[],
                        help="Per-layer cost multipliers (1.0-1000, default: F.Cu=1.0, others=3.0). "
                             "Order matches --layers. Example: --layer-costs 1.0 5.0")

    # Debug options
    parser.add_argument("--debug-lines", action="store_true",
                        help="Output debug geometry on User.3 (connectors), User.4 (stub dirs), User.8 (simplified), User.9 (raw A*)")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Print detailed diagnostic output (setback checks, etc.)")
    parser.add_argument("--skip-routing", action="store_true",
                        help="Skip actual routing, only do swaps and write debug info")
    parser.add_argument("--debug-memory", action="store_true",
                        help="Print memory usage statistics at key points during routing")

    # Visualization options
    parser.add_argument("--visualize", "-V", action="store_true",
                        help="Show real-time visualization of the routing (requires pygame)")
    parser.add_argument("--auto", action="store_true",
                        help="Auto-advance to next net without waiting (with --visualize)")
    parser.add_argument("--display-time", type=float, default=0.0,
                        help="Seconds to display completed route before advancing (with --visualize --auto)")

    args = parser.parse_args()

    # Handle output file: use --overwrite, explicit output, or auto-generate with _routed suffix
    if args.output_file is None:
        if args.overwrite:
            args.output_file = args.input_file
        else:
            # Auto-generate output filename: input.kicad_pcb -> input_routed.kicad_pcb
            base, ext = os.path.splitext(args.input_file)
            args.output_file = base + '_routed' + ext
            print(f"Output file: {args.output_file}")

    # Load PCB to expand wildcards
    print(f"Loading {args.input_file} to expand net patterns...")
    pcb_data = parse_kicad_pcb(args.input_file)

    # Combine positional net_patterns and --nets argument
    all_patterns = list(args.net_patterns) if args.net_patterns else []
    if args.nets:
        all_patterns.extend(args.nets)

    # Default to "*" (all nets) if no patterns and no component specified
    if not all_patterns and not args.component:
        all_patterns = ["*"]

    # Get nets from patterns and/or component
    if all_patterns:
        net_names = expand_net_patterns(pcb_data, all_patterns)
    else:
        net_names = []  # Will be populated by component filter below

    # Filter by component if specified
    if args.component:
        component_nets = set()
        for net_id, pads in pcb_data.pads_by_net.items():
            for pad in pads:
                if pad.component_ref == args.component:
                    net_info = pcb_data.nets.get(net_id)
                    if net_info and net_info.name:
                        component_nets.add(net_info.name)
                    break
        if all_patterns:
            # Intersect with pattern-matched nets
            net_names = [n for n in net_names if n in component_nets]
        else:
            # Use all component nets (excluding power/ground and unconnected pins)
            exclude_patterns = POWER_NET_EXCLUSION_PATTERNS
            filtered = []
            for name in component_nets:
                excluded = False
                for pattern in exclude_patterns:
                    if pattern and fnmatch.fnmatch(name.upper(), pattern.upper()):
                        excluded = True
                        break
                if not excluded:
                    filtered.append(name)
            net_names = sorted(filtered)
        print(f"Filtered to {len(net_names)} nets on component {args.component}")

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
                impedance=args.impedance,
                power_nets=args.power_nets,
                power_nets_widths=args.power_nets_widths,
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
                debug_lines=args.debug_lines,
                verbose=args.verbose,
                max_rip_up_count=args.max_ripup,
                enable_layer_switch=not args.no_stub_layer_swap,
                crossing_layer_check=not args.no_crossing_layer_check,
                can_swap_to_top_layer=args.can_swap_to_top_layer,
                swappable_net_patterns=args.swappable_nets,
                crossing_penalty=args.crossing_penalty,
                skip_routing=args.skip_routing,
                routing_clearance_margin=args.routing_clearance_margin,
                hole_to_hole_clearance=args.hole_to_hole_clearance,
                board_edge_clearance=args.board_edge_clearance,
                vertical_attraction_radius=args.vertical_attraction_radius,
                vertical_attraction_cost=args.vertical_attraction_cost,
                length_match_groups=args.length_match_groups,
                length_match_tolerance=args.length_match_tolerance,
                meander_amplitude=args.meander_amplitude,
                debug_memory=args.debug_memory,
                mps_reverse_rounds=args.mps_reverse_rounds,
                mps_layer_swap=args.mps_layer_swap,
                mps_segment_intersection=args.mps_segment_intersection,
                vis_callback=vis_callback,
                schematic_dir=args.schematic_dir,
                layer_costs=args.layer_costs)

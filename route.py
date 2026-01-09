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
    auto_detect_bga_exclusion_zones, find_components_by_type,
    get_footprint_bounds, detect_bga_pitch
)
from kicad_writer import (
    generate_segment_sexpr, generate_via_sexpr, generate_gr_line_sexpr, generate_gr_text_sexpr,
    swap_segment_nets_at_positions, swap_via_nets_at_positions, swap_pad_nets_in_content,
    modify_segment_layers
)

# Import from refactored modules
from routing_config import GridRouteConfig, GridCoord, DiffPairNet
from routing_utils import (
    find_differential_pairs, get_all_unrouted_net_ids, get_stub_endpoints, get_chip_pad_positions,
    compute_mps_net_ordering, add_route_to_pcb_data, remove_route_from_pcb_data,
    find_pad_nearest_to_position, find_connected_segment_positions,
    find_stub_free_ends, find_connected_groups, pos_key,
    is_edge_stub, find_pad_at_position, expand_net_patterns,
    get_net_endpoints, find_single_ended_nets
)
from obstacle_map import (
    build_base_obstacle_map, add_net_stubs_as_obstacles, add_net_pads_as_obstacles,
    add_net_vias_as_obstacles, add_same_net_via_clearance, add_stub_proximity_costs,
    build_base_obstacle_map_with_vis, add_net_obstacles_with_vis, get_net_bounds,
    VisualizationData, add_connector_region_via_blocking, add_diff_pair_own_stubs_as_obstacles,
    compute_track_proximity_for_net, merge_track_proximity_costs, add_cross_layer_tracks,
    draw_exclusion_zones_debug, add_vias_list_as_obstacles, add_segments_list_as_obstacles,
    precompute_all_net_obstacles, build_working_obstacle_map, precompute_net_obstacles,
    add_net_obstacles_from_cache, remove_net_obstacles_from_cache, update_net_obstacles_after_routing
)
from single_ended_routing import route_net, route_net_with_obstacles, route_net_with_visualization, route_multipoint_taps
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
    apply_length_matching_to_group, find_nets_matching_patterns, auto_group_ddr4_nets,
    apply_intra_pair_length_matching
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
                disable_bga_zones: Optional[List[str]] = None,
                track_width: float = 0.1,
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
                diff_pair_patterns: Optional[List[str]] = None,
                diff_pair_gap: float = 0.101,
                diff_pair_centerline_setback: float = None,
                min_turning_radius: float = 0.2,
                debug_lines: bool = False,
                verbose: bool = False,
                fix_polarity: bool = True,
                max_rip_up_count: int = 3,
                max_setback_angle: float = 45.0,
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
                max_turn_angle: float = 180.0,
                gnd_via_enabled: bool = True,
                vertical_attraction_radius: float = 1.0,
                vertical_attraction_cost: float = 0.1,
                length_match_groups: Optional[List[List[str]]] = None,
                length_match_tolerance: float = 0.1,
                meander_amplitude: float = 1.0,
                diff_chamfer_extra: float = 1.5,
                diff_pair_intra_match: bool = False,
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
        via_cost: Penalty for placing a via in grid steps (default: 50, doubled for diff pairs)
        max_iterations: Max A* iterations before giving up (default: 200000)
        heuristic_weight: A* heuristic weight, higher=faster but less optimal (default: 1.9)
        stub_proximity_radius: Radius around stubs to penalize in mm (default: 2.0)
        stub_proximity_cost: Cost penalty near stubs in mm equivalent (default: 0.2)
        bga_proximity_radius: Radius around BGA edges to penalize in mm (default: 7.0)
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
    if disable_bga_zones is not None:
        if len(disable_bga_zones) == 0:
            # --no-bga-zones with no args: disable all
            bga_exclusion_zones = []
            print("BGA exclusion zones disabled (all)")
        else:
            # --no-bga-zones U1 U3: disable only those components
            bga_components = find_components_by_type(pcb_data, 'BGA')
            bga_exclusion_zones = []
            disabled_refs = set(disable_bga_zones)
            for fp in bga_components:
                if fp.reference not in disabled_refs:
                    bounds = get_footprint_bounds(fp, margin=0.5)
                    pitch = detect_bga_pitch(fp)
                    edge_tolerance = 0.5 + pitch * 1.1
                    bga_exclusion_zones.append((*bounds, edge_tolerance))
            if bga_exclusion_zones:
                print(f"Auto-detected {len(bga_exclusion_zones)} BGA exclusion zone(s) (excluding {', '.join(disable_bga_zones)}):")
                enabled_fps = [fp for fp in bga_components if fp.reference not in disabled_refs]
                for fp, zone in zip(enabled_fps, bga_exclusion_zones):
                    edge_tol = zone[4] if len(zone) > 4 else 1.6
                    print(f"  {fp.reference}: ({zone[0]:.1f}, {zone[1]:.1f}) to ({zone[2]:.1f}, {zone[3]:.1f}), edge_tol={edge_tol:.2f}mm")
            else:
                print(f"BGA exclusion zones disabled for: {', '.join(disable_bga_zones)}")
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
        hole_to_hole_clearance=hole_to_hole_clearance,
        board_edge_clearance=board_edge_clearance,
        max_turn_angle=max_turn_angle,
        gnd_via_enabled=gnd_via_enabled,
        vertical_attraction_radius=vertical_attraction_radius,
        vertical_attraction_cost=vertical_attraction_cost,
        length_match_groups=length_match_groups,
        length_match_tolerance=length_match_tolerance,
        meander_amplitude=meander_amplitude,
        diff_chamfer_extra=diff_chamfer_extra,
        diff_pair_intra_match=diff_pair_intra_match,
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

    # Filter out nets that are already fully connected (no routing needed)
    already_routed = []
    nets_to_route = []
    for net_name, net_id in net_ids:
        _, _, error = get_net_endpoints(pcb_data, net_id, config)
        if error and "already" in error.lower():
            already_routed.append((net_name, error))
        else:
            nets_to_route.append((net_name, net_id))

    if already_routed:
        print(f"\nSkipping {len(already_routed)} already-routed net(s):")
        for net_name, reason in already_routed:
            print(f"  {net_name}: {reason}")

    net_ids = nets_to_route

    if not net_ids:
        print("All nets are already fully connected - nothing to route!")
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
        # Add stub swap vias to pcb_data so routing and length matching see them as obstacles
        for via in all_swap_vias:
            pcb_data.vias.append(via)

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
                return_extended_info=True,
                use_segment_intersection=True if args.mps_segment_intersection else None
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
                        crossing_layer_check=crossing_layer_check,
                        use_segment_intersection=True if args.mps_segment_intersection else None
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
                crossing_layer_check=crossing_layer_check,
                use_segment_intersection=True if args.mps_segment_intersection else None
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

    # Save original (pre-routing) segment signatures to preserve stubs during sync
    # We use object identity since segments are mutable and could be duplicated
    original_segment_ids = set(id(s) for s in pcb_data.segments)

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
    # Use sorted list for deterministic iteration order
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

    # Find GND net ID for GND via obstacle tracking (if GND vias enabled)
    gnd_net_id = None
    if config.gnd_via_enabled:
        for net_id, net in pcb_data.nets.items():
            if net.name.upper() == 'GND':
                gnd_net_id = net_id
                break
        if gnd_net_id:
            print(f"GND net ID: {gnd_net_id} (GND vias will be added as obstacles)")

    # Pre-compute net obstacles for caching (speeds up per-route setup)
    print("Pre-computing net obstacle cache...")
    cache_start = time.time()
    net_obstacles_cache = precompute_all_net_obstacles(
        pcb_data, list(all_unrouted_net_ids), config,
        extra_clearance=0.0, diagonal_margin=0.25
    )
    cache_time = time.time() - cache_start
    print(f"Net obstacle cache built in {cache_time:.2f}s ({len(net_obstacles_cache)} nets)")

    # Build working obstacle map (base + all nets) for incremental updates
    # Uses reference counting in Rust to correctly handle cells blocked by multiple nets
    working_obstacles = build_working_obstacle_map(base_obstacles, net_obstacles_cache)

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
        net_obstacles_cache=net_obstacles_cache,
        working_obstacles=working_obstacles,
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

    # Apply intra-pair P/N length matching if configured
    if config.diff_pair_intra_match:
        print("\n" + "=" * 60)
        print("Intra-pair P/N length matching")
        print("=" * 60)

        # Process each diff pair once (using p_net_id as key to avoid duplicates)
        processed_pairs = set()
        for net_id, result in routed_results.items():
            if not result.get('is_diff_pair'):
                continue
            p_net_id = result.get('p_net_id')
            if p_net_id is None or p_net_id in processed_pairs:
                continue
            processed_pairs.add(p_net_id)

            # Get pair name for logging
            pair_info = diff_pair_by_net_id.get(net_id)
            pair_name = pair_info[0] if pair_info else f"net_{net_id}"

            print(f"\n{pair_name}:")
            seg_count_before = len(result.get('new_segments', []))
            apply_intra_pair_length_matching(result, config, pcb_data)
            seg_count_after = len(result.get('new_segments', []))

    # Sync pcb_data with length-matched segments before Phase 3
    # This ensures tap routes see meanders from other nets as obstacles
    # IMPORTANT: Preserve original stubs (segments from input file) - only replace routed segments
    if routed_results:
        routed_net_ids_set = set(routed_results.keys())
        seg_count_before = len(pcb_data.segments)
        # Remove only ROUTED segments (not original stubs) for routed nets
        # Original stubs have id() in original_segment_ids, routed segments don't
        pcb_data.segments = [s for s in pcb_data.segments
                             if s.net_id not in routed_net_ids_set or id(s) in original_segment_ids]
        seg_count_after_remove = len(pcb_data.segments)
        # Add current (possibly meandered) segments
        total_added = 0
        for net_id, result in routed_results.items():
            for seg in result.get('new_segments', []):
                pcb_data.segments.append(seg)
                total_added += 1
        print(f"\nSync pcb_data: {seg_count_before} -> {seg_count_after_remove} (kept stubs) -> {len(pcb_data.segments)} (after adding {total_added})")

        # Sync working_obstacles with the updated pcb_data
        # This is needed because length matching may have modified segments
        if state.working_obstacles is not None and state.net_obstacles_cache is not None:
            for net_id in routed_net_ids_set:
                # Remove old cache from working
                if net_id in state.net_obstacles_cache:
                    remove_net_obstacles_from_cache(state.working_obstacles, state.net_obstacles_cache[net_id])
                # Recompute cache from current pcb_data
                state.net_obstacles_cache[net_id] = precompute_net_obstacles(pcb_data, net_id, config)
                # Add new cache to working
                add_net_obstacles_from_cache(state.working_obstacles, state.net_obstacles_cache[net_id])

    # Phase 3: Complete multi-point routing (tap connections)
    # This happens AFTER length matching so tap routes connect to meandered main routes
    if state.pending_multipoint_nets:
        # Count total tap edges across all nets for progress display
        total_tap_edges = sum(
            len(result.get('mst_edges', [])) - 1  # -1 for the main edge routed in Phase 1
            for result in state.pending_multipoint_nets.values()
        )
        global_tap_offset = 0
        global_tap_failed = 0

        print("\n" + "=" * 60)
        print(f"Multi-point Phase 3: Routing {total_tap_edges} tap connections")
        print("=" * 60)

        for net_id, main_result in state.pending_multipoint_nets.items():
            net_name = pcb_data.nets[net_id].name if net_id in pcb_data.nets else f"net_{net_id}"
            print(f"\n{net_name} (net {net_id}):")

            # Get the length-matched result (with meanders applied)
            lm_result = routed_results.get(net_id, main_result)
            lm_segments = lm_result.get('new_segments', main_result['new_segments'])
            lm_vias = lm_result.get('new_vias', main_result.get('new_vias', []))

            # Check if length matching modified the segments (different object = modified)
            length_matching_active = lm_segments is not main_result['new_segments']

            # Build obstacles - use incremental approach if no length matching and working map available
            if not length_matching_active and state.working_obstacles is not None and state.net_obstacles_cache:
                from routing_context import build_incremental_obstacles
                obstacles, _ = build_incremental_obstacles(
                    state.working_obstacles, pcb_data, config, net_id,
                    all_unrouted_net_ids, routed_net_ids, track_proximity_cache, layer_map,
                    state.net_obstacles_cache
                )
            else:
                # Full rebuild needed when length matching modified segments
                phase3_routed_ids = [rid for rid in routed_net_ids if rid != net_id]
                obstacles, _ = build_single_ended_obstacles(
                    base_obstacles, pcb_data, config, phase3_routed_ids, remaining_net_ids,
                    all_unrouted_net_ids, net_id, gnd_net_id, track_proximity_cache, layer_map,
                    net_obstacles_cache=state.net_obstacles_cache
                )

            # Build input - use LENGTH-MATCHED segments (so taps connect to actual final route)
            # The segment filtering in route_multipoint_taps will avoid meander regions
            tap_input = dict(main_result)
            tap_input['new_segments'] = lm_segments
            tap_input['new_vias'] = lm_vias

            # Route the tap connections
            completed_result = route_multipoint_taps(
                pcb_data, net_id, config, obstacles, tap_input,
                global_offset=global_tap_offset, global_total=total_tap_edges, global_failed=global_tap_failed
            )

            if completed_result:
                # Update global progress counters
                net_edges_attempted = completed_result.get('tap_edges_routed', 0) + completed_result.get('tap_edges_failed', 0) - 1  # -1 for Phase 1 edge
                global_tap_offset += net_edges_attempted
                global_tap_failed += completed_result.get('tap_edges_failed', 0)

                # Extract only the NEW tap segments (after the length-matched main route)
                tap_segments = completed_result['new_segments'][len(lm_segments):]
                tap_vias = completed_result['new_vias'][len(lm_vias):]

                if tap_segments or tap_vias:
                    tap_result = {'new_segments': tap_segments, 'new_vias': tap_vias}
                    add_route_to_pcb_data(pcb_data, tap_result, debug_lines=config.debug_lines)
                    results.append(tap_result)
                    print(f"  Added {len(tap_segments)} tap segments, {len(tap_vias)} tap vias")

                    # Update working obstacles with tap segments for subsequent nets
                    if state.working_obstacles is not None and state.net_obstacles_cache is not None:
                        # Remove old cache, update with new segments, add back
                        if net_id in state.net_obstacles_cache:
                            remove_net_obstacles_from_cache(state.working_obstacles, state.net_obstacles_cache[net_id])
                        update_net_obstacles_after_routing(pcb_data, net_id, completed_result, config, state.net_obstacles_cache)
                        add_net_obstacles_from_cache(state.working_obstacles, state.net_obstacles_cache[net_id])

                routed_results[net_id] = completed_result

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
        'multipoint_nets': multipoint_nets,
        'multipoint_pads_connected': tap_pads_connected,
        'multipoint_pads_total': tap_pads_total,
        'multipoint_edges_routed': tap_edges_routed,
        'multipoint_edges_failed': tap_edges_failed,
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
    parser.add_argument("net_patterns", nargs="*", help="Net names or wildcard patterns to route (optional if --component or --nets used)")
    parser.add_argument("--nets", "-n", nargs="+", help="Net names or wildcard patterns to route (alternative to positional args)")
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
    parser.add_argument("--mps-segment-intersection", action="store_true",
                        help="Force MPS to use segment intersection for crossing detection (auto-enabled when no BGA chips)")

    # Length matching options
    parser.add_argument("--length-match-group", action="append", nargs="+", dest="length_match_groups",
                        help="Net patterns to length-match as a group (can be repeated). Use 'auto' for DDR4 auto-grouping")
    parser.add_argument("--length-match-tolerance", type=float, default=0.1,
                        help="Acceptable length variance within group in mm (default: 0.1)")
    parser.add_argument("--meander-amplitude", type=float, default=1.0,
                        help="Height of meander perpendicular to trace in mm (default: 1.0)")
    parser.add_argument("--diff-chamfer-extra", type=float, default=1.5,
                        help="Chamfer multiplier for diff pair meanders (default: 1.5, >1 avoids P/N crossings)")
    parser.add_argument("--diff-pair-intra-match", action="store_true",
                        help="Enable intra-pair P/N length matching (add meanders to shorter track of each diff pair)")

    # Rip-up and retry options
    parser.add_argument("--max-ripup", type=int, default=3,
                        help="Maximum blockers to rip up at once during rip-up and retry (default: 3)")
    parser.add_argument("--max-setback-angle", type=float, default=45.0,
                        help="Maximum angle (degrees) for setback position search (default: 45.0)")
    parser.add_argument("--routing-clearance-margin", type=float, default=1.0,
                        help="Multiplier on track-via clearance (1.0 = minimum DRC)")
    parser.add_argument("--hole-to-hole-clearance", type=float, default=0.2,
                        help="Minimum clearance between drill holes in mm (default: 0.2)")
    parser.add_argument("--board-edge-clearance", type=float, default=0.0,
                        help="Clearance from board edge in mm (default: 0 = use track clearance)")
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

    # Combine positional net_patterns and --nets argument
    all_patterns = list(args.net_patterns) if args.net_patterns else []
    if args.nets:
        all_patterns.extend(args.nets)

    # Get nets from patterns and/or component
    if all_patterns:
        net_names = expand_net_patterns(pcb_data, all_patterns)
    elif args.component:
        net_names = []  # Will be populated by component filter below
    else:
        print("Error: Must specify net patterns or --component")
        sys.exit(1)

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
            exclude_patterns = ['*GND*', '*VCC*', '*VDD*', '+*V', '-*V', 'unconnected-*', '']
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
                hole_to_hole_clearance=args.hole_to_hole_clearance,
                board_edge_clearance=args.board_edge_clearance,
                max_turn_angle=args.max_turn_angle,
                gnd_via_enabled=not args.no_gnd_vias,
                vertical_attraction_radius=args.vertical_attraction_radius,
                vertical_attraction_cost=args.vertical_attraction_cost,
                length_match_groups=args.length_match_groups,
                length_match_tolerance=args.length_match_tolerance,
                meander_amplitude=args.meander_amplitude,
                diff_chamfer_extra=args.diff_chamfer_extra,
                diff_pair_intra_match=args.diff_pair_intra_match,
                vis_callback=vis_callback)

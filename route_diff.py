"""
Batch Differential Pair PCB Router using Rust-accelerated A* - Routes differential pairs.

Usage:
    python route_diff.py input.kicad_pcb output.kicad_pcb --nets "*lvds*"

All nets matching the patterns are treated as differential pairs (P/N pairs).
Nets with _P/_N, P/N, or +/- suffixes will be paired and routed together.
Nets only pair within the same suffix convention (e.g. CLK+ pairs with CLK-,
never with an unrelated CLK_N).

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
from routing_config import GridRouteConfig, GridCoord, DiffPairNet
import routing_defaults as defaults
from routing_utils import pos_key
from connectivity import (
    get_stub_endpoints, find_stub_free_ends, find_connected_groups,
    is_edge_stub, get_net_endpoints, find_connected_segment_positions
)
from net_queries import (
    find_differential_pairs, get_all_unrouted_net_ids, get_chip_pad_positions,
    compute_mps_net_ordering, find_pad_nearest_to_position,
    expand_net_patterns
)
from impedance import calculate_layer_widths_for_impedance, print_impedance_routing_plan
from pcb_modification import add_route_to_pcb_data, remove_route_from_pcb_data
from obstacle_map import (
    build_base_obstacle_map, add_net_stubs_as_obstacles, add_net_pads_as_obstacles,
    add_net_vias_as_obstacles, add_same_net_via_clearance,
    build_base_obstacle_map_with_vis, get_net_bounds,
    VisualizationData, add_connector_region_via_blocking, add_diff_pair_own_stubs_as_obstacles,
    draw_exclusion_zones_debug, add_vias_list_as_obstacles, add_segments_list_as_obstacles
)
from obstacle_costs import (
    add_stub_proximity_costs, compute_track_proximity_for_net,
    merge_track_proximity_costs, add_cross_layer_tracks
)
from obstacle_cache import (
    precompute_all_net_obstacles, build_working_obstacle_map, update_net_obstacles_after_routing
)
from diff_pair_routing import route_diff_pair_with_obstacles, get_diff_pair_connector_regions
from blocking_analysis import analyze_frontier_blocking, print_blocking_analysis, filter_rippable_blockers
from rip_up_reroute import rip_up_net, restore_net
from polarity_swap import apply_polarity_swap, get_canonical_net_id
from layer_swap_fallback import try_fallback_layer_swap, add_own_stubs_as_obstacles_for_diff_pair
from layer_swap_optimization import apply_diff_pair_layer_swaps
from routing_context import (
    build_diff_pair_obstacles,
    record_diff_pair_success,
    restore_ripped_net
)
from routing_state import RoutingState, create_routing_state
from memory_debug import (
    get_process_memory_mb, format_memory_stats,
    estimate_net_obstacles_cache_mb, estimate_track_proximity_cache_mb,
    estimate_routed_paths_mb, format_obstacle_map_stats
)
from diff_pair_loop import route_diff_pairs
from reroute_loop import run_reroute_loop
from length_matching import apply_intra_pair_length_matching
from net_ordering import order_nets_mps, order_nets_inside_out, separate_nets_by_type
from routing_common import (
    setup_bga_exclusion_zones, filter_already_routed,
    run_length_matching, sync_pcb_data_segments, get_common_config_kwargs,
    warn_targets_outside_board
)
import re
from terminal_colors import RED, GREEN, YELLOW, RESET

# Import Rust router (startup_checks ensures it's available and up-to-date)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))
from grid_router import GridObstacleMap, GridRouter


def batch_route_diff_pairs(input_file: str, output_file: str, net_names: List[str],
                layers: List[str] = None,
                layer_costs: Optional[List[float]] = None,
                bga_exclusion_zones: Optional[List[Tuple[float, float, float, float]]] = None,
                direction_order: str = None,
                ordering_strategy: str = "inside_out",
                disable_bga_zones: Optional[List[str]] = None,
                track_width: float = defaults.TRACK_WIDTH,
                impedance: Optional[float] = None,
                clearance: float = defaults.CLEARANCE,
                via_size: float = defaults.VIA_SIZE,
                via_drill: float = defaults.VIA_DRILL,
                grid_step: float = defaults.GRID_STEP,
                via_cost: int = defaults.VIA_COST,
                max_iterations: int = defaults.MAX_ITERATIONS,
                max_probe_iterations: int = defaults.MAX_PROBE_ITERATIONS,
                heuristic_weight: float = defaults.HEURISTIC_WEIGHT,
                turn_cost: int = defaults.TURN_COST,
                direction_preference_cost: int = defaults.DIRECTION_PREFERENCE_COST,
                bus_enabled: bool = False,
                bus_detection_radius: float = defaults.BUS_DETECTION_RADIUS,
                bus_attraction_radius: float = defaults.BUS_ATTRACTION_RADIUS,
                bus_attraction_bonus: int = defaults.BUS_ATTRACTION_BONUS,
                bus_min_nets: int = defaults.BUS_MIN_NETS,
                keepout_enabled: bool = False,
                keepout_layer: str = defaults.KEEPOUT_LAYER,
                proximity_heuristic_factor: float = defaults.PROXIMITY_HEURISTIC_FACTOR,
                stub_proximity_radius: float = defaults.STUB_PROXIMITY_RADIUS,
                stub_proximity_cost: float = defaults.STUB_PROXIMITY_COST,
                via_proximity_cost: float = defaults.VIA_PROXIMITY_COST,
                bga_proximity_radius: float = defaults.BGA_PROXIMITY_RADIUS,
                bga_proximity_cost: float = defaults.BGA_PROXIMITY_COST,
                track_proximity_distance: float = defaults.TRACK_PROXIMITY_DISTANCE,
                track_proximity_cost: float = defaults.TRACK_PROXIMITY_COST,
                diff_pair_gap: float = defaults.DIFF_PAIR_GAP,
                diff_pair_centerline_setback: float = None,
                min_turning_radius: float = defaults.DIFF_PAIR_MIN_TURNING_RADIUS,
                debug_lines: bool = False,
                verbose: bool = False,
                fix_polarity: bool = True,
                max_rip_up_count: int = defaults.MAX_RIPUP,
                max_setback_angle: float = defaults.DIFF_PAIR_MAX_SETBACK_ANGLE,
                enable_layer_switch: bool = True,
                crossing_layer_check: bool = True,
                can_swap_to_top_layer: bool = False,
                swappable_net_patterns: Optional[List[str]] = None,
                crossing_penalty: float = defaults.CROSSING_PENALTY,
                mps_unroll: bool = True,
                skip_routing: bool = False,
                routing_clearance_margin: float = defaults.ROUTING_CLEARANCE_MARGIN,
                hole_to_hole_clearance: float = defaults.HOLE_TO_HOLE_CLEARANCE,
                board_edge_clearance: float = defaults.BOARD_EDGE_CLEARANCE,
                max_turn_angle: float = defaults.DIFF_PAIR_MAX_TURN_ANGLE,
                gnd_via_enabled: bool = True,
                vertical_attraction_radius: float = defaults.VERTICAL_ATTRACTION_RADIUS,
                vertical_attraction_cost: float = defaults.VERTICAL_ATTRACTION_COST,
                ripped_route_avoidance_radius: float = defaults.RIPPED_ROUTE_AVOIDANCE_RADIUS,
                ripped_route_avoidance_cost: float = defaults.RIPPED_ROUTE_AVOIDANCE_COST,
                length_match_groups: Optional[List[List[str]]] = None,
                length_match_tolerance: float = defaults.LENGTH_MATCH_TOLERANCE,
                meander_amplitude: float = defaults.MEANDER_AMPLITUDE,
                time_matching: bool = defaults.TIME_MATCHING,
                time_match_tolerance: float = defaults.TIME_MATCH_TOLERANCE,
                diff_chamfer_extra: float = defaults.DIFF_PAIR_CHAMFER_EXTRA,
                diff_pair_intra_match: bool = False,
                ac_couple_match: bool = False,
                debug_memory: bool = False,
                mps_reverse_rounds: bool = False,
                mps_layer_swap: bool = False,
                mps_segment_intersection: bool = False,
                schematic_dir: Optional[str] = None,
                add_teardrops: bool = False,
                return_results: bool = False,
                pcb_data=None,
                cancel_check=None,
                progress_callback=None) -> Tuple[int, int, float]:
    """
    Route differential pairs using the Rust router.

    All nets provided are treated as differential pairs. Nets with _P/_N, P/N, or +/-
    suffixes will be paired and routed together maintaining constant spacing.

    Args:
        input_file: Path to input KiCad PCB file
        output_file: Path to output KiCad PCB file
        net_names: List of net names to route (all treated as differential pairs)
        layers: List of copper layers to route on
        diff_pair_gap: Gap between P and N traces of differential pairs in mm (default: 0.101)
        diff_pair_centerline_setback: Distance in front of stubs to start centerline (default: 2x P-N spacing)
        min_turning_radius: Minimum turning radius for pose-based routing in mm (default: 0.2)
        fix_polarity: Swap target pad net assignments if polarity swap is needed (default: True)
        gnd_via_enabled: Add GND vias near diff pair signal vias (default: True)
        diff_chamfer_extra: Chamfer multiplier for diff pair meanders (default: 1.5)
        diff_pair_intra_match: Enable intra-pair P/N length matching (default: False)
        ac_couple_match: End-to-end length-match AC-coupled diff pairs split by series
            DC-blocking caps, matching the concatenated P vs N path (#196) (default: False)
        return_results: If True, return results data instead of writing to file
        pcb_data: Optional pre-parsed PCBData (if None, loads from input_file)

    Returns:
        If return_results=False: (successful_count, failed_count, total_time)
        If return_results=True: (successful_count, failed_count, total_time, results_data)
    """
    # Track memory if debug_memory enabled
    mem_start = get_process_memory_mb() if debug_memory else 0.0
    if debug_memory:
        print(format_memory_stats("Initial memory", mem_start))

    # Load or use provided pcb_data
    if pcb_data is None:
        print(f"Loading {input_file}...")
        pcb_data = parse_kicad_pcb(input_file, keepout_layer=keepout_layer)
    else:
        print("Using provided PCB data...")

    # Layers must be specified - we can't auto-detect which are ground planes
    if layers is None:
        layers = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']  # Default 4-layer signal stack
    print(f"Using {len(layers)} routing layers: {layers}")

    # Per-layer cost multipliers bias which layer(s) coupled diff pairs prefer
    # (issue #193). Default is 1.0x on every layer so behavior is unchanged unless
    # --layer-costs is given; unlike route.py there is no 2-layer F.Cu/B.Cu default.
    if not layer_costs:
        layer_costs = [1.0] * len(layers)
    for i, cost in enumerate(layer_costs):
        if cost >= 0 and (cost < 1.0 or cost > 1000):
            from routing_exceptions import ConfigurationError
            layer_name = layers[i] if i < len(layers) else f"layer {i}"
            raise ConfigurationError(f"Layer cost for {layer_name} must be negative (forbidden) or "
                                     f"between 1.0 and 1000, got {cost}")
    if any(c != 1.0 for c in layer_costs):
        costs_str = ', '.join(f"{layers[i]}={layer_costs[i]}x" for i in range(min(len(layers), len(layer_costs))))
        print(f"  Layer costs: {costs_str}")

    # Calculate layer-specific widths for impedance-controlled routing
    # For diff pairs, we use the diff_pair_gap as the fixed spacing and calculate width
    layer_widths = {}
    if impedance is not None:
        if not pcb_data.board_info.stackup:
            print("WARNING: No stackup found in PCB file. Using fixed track width.")
        else:
            print(f"\nCalculating trace widths for {impedance}Ω differential impedance...")
            print(f"Using diff pair spacing: {diff_pair_gap}mm ({diff_pair_gap * 39.3701:.2f} mil)")
            layer_widths = calculate_layer_widths_for_impedance(
                pcb_data, layers, impedance,
                spacing=diff_pair_gap, is_differential=True,
                fallback_width=track_width,
                min_width=track_width
            )
            print_impedance_routing_plan(pcb_data, layers, impedance,
                                        spacing=diff_pair_gap, is_differential=True,
                                        min_width=track_width)

    # Auto-detect BGA exclusion zones if not specified
    bga_exclusion_zones = setup_bga_exclusion_zones(pcb_data, disable_bga_zones, bga_exclusion_zones)

    # Build config kwargs from common parameters plus diff-pair specific options
    config_kwargs = get_common_config_kwargs(
        track_width=track_width, clearance=clearance, via_size=via_size,
        via_drill=via_drill, grid_step=grid_step, via_cost=via_cost,
        layers=layers, max_iterations=max_iterations,
        max_probe_iterations=max_probe_iterations, heuristic_weight=heuristic_weight,
        turn_cost=turn_cost, direction_preference_cost=direction_preference_cost,
        bus_enabled=bus_enabled, bus_detection_radius=bus_detection_radius,
        bus_attraction_radius=bus_attraction_radius, bus_attraction_bonus=bus_attraction_bonus,
        bus_min_nets=bus_min_nets, proximity_heuristic_factor=proximity_heuristic_factor,
        keepout_enabled=keepout_enabled, keepout_layer=keepout_layer,
        bga_exclusion_zones=bga_exclusion_zones,
        stub_proximity_radius=stub_proximity_radius, stub_proximity_cost=stub_proximity_cost,
        via_proximity_cost=via_proximity_cost, bga_proximity_radius=bga_proximity_radius,
        bga_proximity_cost=bga_proximity_cost, track_proximity_distance=track_proximity_distance,
        track_proximity_cost=track_proximity_cost, debug_lines=debug_lines, verbose=verbose,
        max_rip_up_count=max_rip_up_count, crossing_penalty=crossing_penalty,
        crossing_layer_check=crossing_layer_check, routing_clearance_margin=routing_clearance_margin,
        hole_to_hole_clearance=hole_to_hole_clearance, board_edge_clearance=board_edge_clearance,
        vertical_attraction_radius=vertical_attraction_radius,
        vertical_attraction_cost=vertical_attraction_cost,
        ripped_route_avoidance_radius=ripped_route_avoidance_radius,
        ripped_route_avoidance_cost=ripped_route_avoidance_cost,
        length_match_groups=length_match_groups,
        length_match_tolerance=length_match_tolerance, meander_amplitude=meander_amplitude,
        time_matching=time_matching, time_match_tolerance=time_match_tolerance,
        debug_memory=debug_memory
    )
    # Add diff-pair specific config options
    config_kwargs.update(
        diff_pair_gap=diff_pair_gap,
        diff_pair_centerline_setback=diff_pair_centerline_setback,
        min_turning_radius=min_turning_radius,
        fix_polarity=fix_polarity,
        max_setback_angle=max_setback_angle,
        max_turn_angle=max_turn_angle,
        gnd_via_enabled=gnd_via_enabled,
        diff_chamfer_extra=diff_chamfer_extra,
        diff_pair_intra_match=diff_pair_intra_match,
        ac_couple_match=ac_couple_match,
    )
    if direction_order is not None:
        config_kwargs['direction_order'] = direction_order
    if layer_widths:
        config_kwargs['layer_widths'] = layer_widths
        config_kwargs['impedance_target'] = impedance
    config_kwargs['layer_costs'] = layer_costs  # per-layer bias for coupled diff routing (#193)
    config = GridRouteConfig(**config_kwargs)

    # Find differential pairs from all provided nets
    diff_pairs: Dict[str, DiffPairNet] = find_differential_pairs(pcb_data, net_names)
    diff_pair_net_ids = set()  # Net IDs that are part of differential pairs

    # Detect AC-coupled XNets (#196): differential pairs split into two base-named
    # pairs by series DC-blocking caps, to be length-matched END-TO-END after
    # routing (see the AC-couple pass below). Pure read of pcb_data -- gathers a
    # side-structure only, never touching diff_pairs / net order / pcb_data, so the
    # routed result is byte-identical when --ac-couple-match is off.
    ac_xnets = []
    if config.ac_couple_match:
        from diff_xnet import find_ac_coupled_xnets
        ac_xnets, ac_warnings = find_ac_coupled_xnets(pcb_data, diff_pairs)
        for _w in ac_warnings:
            print(f"  WARNING: {_w}")
        for _xn in ac_xnets:
            print(f"  AC-coupled XNet: {'+'.join(_xn.base_names)} "
                  f"(coupled via {', '.join(_xn.bridge_refs)})")

    if not diff_pairs:
        print(f"Error: No differential pairs found matching the patterns!")
        print("  Differential pairs must have _P/_N, P/N, or +/- suffixes.")
        print(f"  Patterns provided: {net_names}")
        if return_results:
            return 0, 0, 0.0, {'results': [], 'all_swap_vias': [], 'exclusion_zone_lines': [], 'boundary_debug_labels': []}
        return 0, 0, 0.0

    # Track which net IDs are part of pairs
    for pair in diff_pairs.values():
        diff_pair_net_ids.add(pair.p_net_id)
        diff_pair_net_ids.add(pair.n_net_id)

    print(f"Found {len(diff_pairs)} differential pair(s)")

    # Build the net-id list from the diff pairs we found (both halves), so an
    # explicit base name ('/DVI_CK') or a one-sided glob ('*_P') still routes
    # both halves -- resolve_net_ids only matches exact full net names and would
    # drop the unmatched sibling (or a base name that names no net). (issue #120)
    net_ids = []
    seen_net_ids = set()
    for pair in diff_pairs.values():
        for nid, nname in ((pair.p_net_id, pair.p_net_name),
                           (pair.n_net_id, pair.n_net_name)):
            if nid is not None and nid not in seen_net_ids:
                net_ids.append((nname, nid))
                seen_net_ids.add(nid)
    if not net_ids:
        print("No valid nets to route!")
        if return_results:
            return 0, 0, 0.0, {'results': [], 'all_swap_vias': [], 'exclusion_zone_lines': [], 'boundary_debug_labels': []}
        return 0, 0, 0.0

    # Flag target pads at/over the board edge before routing, so an unroutable
    # off-board pad reads as a clear warning rather than a silent search failure
    # (issue #195).
    _edge_clear = board_edge_clearance if board_edge_clearance > 0 else clearance
    warn_targets_outside_board(pcb_data, net_ids,
                               edge_margin=_edge_clear + track_width / 2)

    net_ids, _ = filter_already_routed(pcb_data, net_ids, config)
    if not net_ids:
        print("All nets are already fully connected - nothing to route!")
        if return_results:
            return 0, 0, 0.0, {'results': [], 'all_swap_vias': [], 'exclusion_zone_lines': [], 'boundary_debug_labels': []}
        return 0, 0, 0.0

    # Track all segment layer modifications for file output
    all_segment_modifications = []
    # Track all vias added during stub layer swapping
    all_swap_vias = []
    # pair_name -> {'vias': [...], 'stubs': [...]} for bare-pad target swaps, so a
    # failed pair can have its swap undone and be re-routed clean (issue #142).
    bare_pad_swaps = {}
    # Track new stub segments synthesized by bare-pad target swaps
    all_swap_segments = []
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

    if not diff_pair_ids_to_route_set:
        print("Error: No differential pairs to route!")
        print("  Ensure your net patterns match nets with _P/_N, P/N, or +/- suffixes.")
        if return_results:
            return 0, 0, 0.0, {'results': [], 'all_swap_vias': [], 'exclusion_zone_lines': [], 'boundary_debug_labels': []}
        return 0, 0, 0.0

    # ----- Fanout DRC pre-check (#242) ------------------------------------
    # A pair whose fanout escape stubs ALREADY pinch their own P/N below clearance
    # (a bga_fanout escape-fan defect) cannot be coupled-routed cleanly: routing it
    # only spreads or relocates the bad copper (a solo source switch silently moves
    # the overlap to another layer). Detect it up front from the existing stub
    # copper, skip the pair entirely (not routed here, and -- by dropping it from
    # the routed-net set -- left as an obstacle and not handed to the single-ended
    # follow-up), and report it. The fanout must be fixed, not routed over.
    from diff_pair_routing import _seg_to_seglist_min_edge as _fanout_edge

    def _fanout_self_overlaps(p_net_id, n_net_id):
        # Match check_drc's seg-seg verdict: it ignores an overlap <= clearance*5%
        # (a coupled escape laid right at the gap dips a hair below clearance at
        # bends but is DRC-clean). Only a REAL violation (e.g. a touching escape
        # fan, edge ~0) should skip the pair.
        thr = config.clearance * 0.95
        p_segs = [s for s in pcb_data.segments if s.net_id == p_net_id]
        n_segs = [s for s in pcb_data.segments if s.net_id == n_net_id]
        if not p_segs or not n_segs:
            return False
        return any(_fanout_edge(s.start_x, s.start_y, s.end_x, s.end_y, s.width, s.layer, n_segs)
                   < thr for s in p_segs)

    skipped_bad_fanout = [name for name, pair in diff_pair_ids_to_route_set
                          if _fanout_self_overlaps(pair.p_net_id, pair.n_net_id)]
    if skipped_bad_fanout:
        skip_set = set(skipped_bad_fanout)
        skip_net_ids = {nid for name, pair in diff_pair_ids_to_route_set if name in skip_set
                        for nid in (pair.p_net_id, pair.n_net_id)}
        diff_pair_ids_to_route_set = [(n, p) for n, p in diff_pair_ids_to_route_set
                                      if n not in skip_set]
        net_ids = [(nm, nid) for nm, nid in net_ids if nid not in skip_net_ids]
        diff_pair_net_ids = {nid for nid in diff_pair_net_ids if nid not in skip_net_ids}
        print("\n" + "=" * 60)
        print(f"Skipping {len(skipped_bad_fanout)} pair(s): fanout escape stubs already pinch "
              f"their own P/N below clearance ({config.clearance}mm) -- fix the fanout (#242):")
        for n in skipped_bad_fanout:
            print(f"  {n}")
        print("=" * 60)

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
        # Probe obstacle map so a bare-pad target swap can fan onto an OPEN
        # launch layer rather than blindly the source layer (issue #121: lpddr4
        # DQ_S0's target was fanned to B.Cu, which is 100% blocked by a connector
        # pad wall, while the inner layers right there were empty).
        _swap_probe_clearance = (config.track_width + config.diff_pair_gap) / 2
        swap_probe_obstacles = build_base_obstacle_map(
            pcb_data, config, [nid for _, nid in net_ids], _swap_probe_clearance)
        total_layer_swaps, all_stubs_by_layer, stub_endpoints_by_layer = apply_diff_pair_layer_swaps(
            pcb_data, config, diff_pair_ids_to_route_set, diff_pairs,
            can_swap_to_top_layer, all_segment_modifications, all_swap_vias,
            all_swap_segments=all_swap_segments, probe_obstacles=swap_probe_obstacles,
            bare_pad_swaps=bare_pad_swaps
        )

    # Add stub swap vias to pcb_data so routing and length matching see them as obstacles
    for via in all_swap_vias:
        pcb_data.vias.append(via)

    # Apply net ordering strategy
    if ordering_strategy == "mps":
        net_ids, mps_layer_swaps = order_nets_mps(
            pcb_data=pcb_data,
            net_ids=net_ids,
            diff_pairs=diff_pairs,
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

    # All nets are diff pairs - separate them from any non-diff-pair nets
    diff_pair_ids_to_route, single_ended_nets = separate_nets_by_type(
        net_ids, diff_pairs, diff_pair_net_ids
    )

    # Warn if any single-ended nets were found (they won't be routed)
    if single_ended_nets:
        print(f"\nWarning: Skipping {len(single_ended_nets)} non-differential-pair net(s):")
        for net_name, _ in single_ended_nets[:5]:
            print(f"  {net_name}")
        if len(single_ended_nets) > 5:
            print(f"  ... and {len(single_ended_nets) - 5} more")
        print("  Use route.py for single-ended routing.")

    total_routes = len(diff_pair_ids_to_route)

    results = []
    pad_swaps = []  # List of (pad1, pad2) tuples for nets that need swapping
    successful = 0
    failed = 0
    total_time = 0
    total_iterations = 0

    # Skip routing if requested - just write output with swaps and debug info
    if skip_routing:
        print(f"\n--skip-routing: Skipping actual routing of {total_routes} diff pairs")
        print("Writing output file with swaps and debug labels only...")
        diff_pair_ids_to_route = []
    else:
        print(f"\nRouting {total_routes} differential pair(s)...")
        print("=" * 60)

    # Build base obstacle map once (excludes all nets we're routing)
    all_net_ids_to_route = [nid for _, nid in net_ids]
    print("Building base obstacle map...")
    base_start = time.time()
    base_obstacles = build_base_obstacle_map(pcb_data, config, all_net_ids_to_route)
    base_elapsed = time.time() - base_start
    print(f"Base obstacle map built in {base_elapsed:.2f}s")
    if debug_memory:
        mem_after_base = get_process_memory_mb()
        print(format_memory_stats("After base obstacle map", mem_after_base, mem_after_base - mem_start))

    # Save original (pre-routing) segment signatures to preserve stubs during sync
    original_segment_ids = set(id(s) for s in pcb_data.segments)

    # Build separate base obstacle map with extra clearance for diff pair centerline routing
    # Extra clearance = spacing from centerline to P/N track center
    diff_pair_extra_clearance = (config.track_width + config.diff_pair_gap) / 2
    print(f"Building diff pair obstacle map (extra clearance: {diff_pair_extra_clearance:.3f}mm)...")
    dp_base_start = time.time()
    diff_pair_base_obstacles = build_base_obstacle_map(pcb_data, config, all_net_ids_to_route, diff_pair_extra_clearance)
    dp_base_elapsed = time.time() - dp_base_start
    print(f"Diff pair obstacle map built in {dp_base_elapsed:.2f}s")
    if debug_memory:
        mem_after_dp = get_process_memory_mb()
        print(format_memory_stats("After diff pair obstacle map", mem_after_dp, mem_after_dp - mem_start))

    # Block connector regions for ALL diff pairs upfront
    # Vias span all layers, so we must block connector regions before any routing starts
    if diff_pair_ids_to_route:
        print(f"Blocking connector regions for {len(diff_pair_ids_to_route)} diff pair(s)...")
        for pair_name, pair in diff_pair_ids_to_route:
            connector_info = get_diff_pair_connector_regions(pcb_data, pair, config)
            if connector_info:
                for end in ('src', 'tgt'):
                    dir_x, dir_y = connector_info[f'{end}_dir']
                    # Synthesized (bare-pad) directions may be flipped by the
                    # router, so block the connector region on both sides
                    signs = (1, -1) if connector_info[f'{end}_dir_synthesized'] else (1,)
                    for sign in signs:
                        add_connector_region_via_blocking(
                            diff_pair_base_obstacles,
                            connector_info[f'{end}_center'][0], connector_info[f'{end}_center'][1],
                            dir_x * sign, dir_y * sign,
                            connector_info[f'{end}_setback'], connector_info['spacing_mm'], config
                        )

    # Get ALL unrouted nets in the PCB for stub proximity costs
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
        extra_clearance=0.0, diagonal_margin=defaults.DIAGONAL_MARGIN
    )
    cache_time = time.time() - cache_start
    print(f"Net obstacle cache built in {cache_time:.2f}s ({len(net_obstacles_cache)} nets)")
    if debug_memory:
        mem_after_cache = get_process_memory_mb()
        cache_size = estimate_net_obstacles_cache_mb(net_obstacles_cache)
        print(format_memory_stats("After net obstacles cache", mem_after_cache, mem_after_cache - mem_start))
        print(f"[MEMORY]   Cache estimated size: {cache_size:.1f} MB for {len(net_obstacles_cache)} nets")

    # Build working obstacle map (base + all nets) for incremental updates
    working_obstacles = build_working_obstacle_map(base_obstacles, net_obstacles_cache)
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
        diff_pair_base_obstacles=diff_pair_base_obstacles,
        diff_pair_extra_clearance=diff_pair_extra_clearance,
        gnd_net_id=gnd_net_id,
        all_unrouted_net_ids=all_unrouted_net_ids,
        total_routes=total_routes,
        enable_layer_switch=enable_layer_switch,
        debug_lines=debug_lines,
        target_swaps=target_swaps,
        target_swap_info=target_swap_info,
        single_ended_target_swaps={},  # Not used for diff pairs
        single_ended_target_swap_info=[],
        all_segment_modifications=all_segment_modifications,
        all_swap_vias=all_swap_vias,
        total_layer_swaps=total_layer_swaps,
        net_obstacles_cache=net_obstacles_cache,
        working_obstacles=working_obstacles,
        cancel_check=cancel_check,
        progress_callback=progress_callback,
    )

    # Create local aliases for frequently-used state fields
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

    route_index = 0

    # Route differential pairs
    dp_successful, dp_failed, dp_time, dp_iterations, route_index = route_diff_pairs(
        state, diff_pair_ids_to_route
    )
    successful += dp_successful
    failed += dp_failed
    total_time += dp_time
    total_iterations += dp_iterations

    # ----- Bare-pad target swap fallback (issue #142) ---------------------
    # A bare-pad target swap fans a surface connector pad onto an inner layer
    # via a synthesized short stub. For some geometries that stub pins the pair
    # into a forced P/N crossing, so the pair FAILS to route -- yet the very
    # same pair routes cleanly straight to the bare pads with a plain via.
    # Detect those failures, undo the swap (drop the synthesized vias + stubs),
    # and re-route the affected pairs once.
    retry_bare_pad = [
        (pair_name, pair)
        for pair_name, pair in diff_pair_ids_to_route
        if pair_name in bare_pad_swaps
        and not (pair.p_net_id in routed_results and pair.n_net_id in routed_results)
        and not (pair.p_net_id in state.diff_pair_single_ended_nets
                 or pair.n_net_id in state.diff_pair_single_ended_nets)
    ]
    if retry_bare_pad:
        print("\n" + "=" * 60)
        print(f"Retrying {len(retry_bare_pad)} pair(s) without bare-pad target swap "
              f"(swap pinned a P/N crossing): "
              + ", ".join(name for name, _ in retry_bare_pad))
        print("=" * 60)

        def _drop(obj_list, doomed):
            doomed_ids = set(id(o) for o in doomed)
            obj_list[:] = [o for o in obj_list if id(o) not in doomed_ids]

        # The synthesized via/stub live on the pair's OWN nets, which are excluded
        # from the obstacle maps while that pair routes - so dropping them from
        # pcb_data (which reverts the pair's endpoints to the bare pads) is enough;
        # no obstacle-map rebuild is needed.
        for pair_name, _pair in retry_bare_pad:
            info = bare_pad_swaps.pop(pair_name)
            _drop(pcb_data.vias, info['vias'])
            _drop(all_swap_vias, info['vias'])
            _drop(pcb_data.segments, info['stubs'])
            _drop(all_swap_segments, info['stubs'])

        rb_s, rb_f, rb_t, rb_i, route_index = route_diff_pairs(state, retry_bare_pad)
        # The retried pairs were already tallied as failures in the first pass;
        # move any that now route from the failed column to the success column.
        successful += rb_s
        failed -= rb_s
        total_time += rb_t
        total_iterations += rb_i

    # Report nets whose far-apart (uncoupled) terminal pads were peeled off the
    # coupled chain (issue #121). Those pads are not a coupled differential
    # connection (e.g. spread-out test points), so they are left for a separate
    # single-ended pass (route.py); the plan-pcb-routing workflow sequences that
    # after this step. Surfaced here and in JSON_SUMMARY so the follow-up knows
    # which nets still need P->P / N->N routing.
    single_ended_followup = sorted(state.diff_pair_single_ended_nets.values())
    if single_ended_followup:
        print("\n" + "=" * 60)
        print(f"{len(single_ended_followup)} net(s) have far-apart terminal pads "
              f"peeled from the coupled chain - route them single-ended next:")
        for nm in single_ended_followup:
            print(f"  {nm}")
        print("  e.g.  route.py <this_output> <out2> --nets " +
              " ".join(f"'{n}'" for n in single_ended_followup[:3]) +
              (" ..." if len(single_ended_followup) > 3 else ""))
        print("=" * 60)

    # Run reroute loop for nets that were ripped during diff pair routing
    rq_successful, rq_failed, rq_time, rq_iterations, route_index = run_reroute_loop(
        state, route_index_start=route_index,
        cancel_check=cancel_check, progress_callback=progress_callback,
        failed_so_far=failed
    )
    successful += rq_successful
    failed += rq_failed
    total_time += rq_time
    total_iterations += rq_iterations

    # Apply length matching if configured
    if length_match_groups:
        run_length_matching(routed_results, length_match_groups, config, pcb_data)

    # Apply intra-pair P/N length matching if configured
    if config.diff_pair_intra_match:
        print("\n" + "=" * 60)
        print("Intra-pair P/N length matching")
        print("=" * 60)

        # Process each diff pair once (using p_net_id as key to avoid duplicates)
        processed_pairs = set()
        # AC-coupled (XNet) member pairs are matched end-to-end below, not per-side;
        # skip them here so the two passes don't fight (double-meander). Gated on
        # the flag so intra-pair behavior is unchanged when --ac-couple-match is off.
        xnet_member_p_net_ids = set()
        if config.ac_couple_match:
            for _xn in ac_xnets:
                for _m in _xn.members:
                    xnet_member_p_net_ids.add(_m.p_net_id)
        for net_id, result in routed_results.items():
            if not result.get('is_diff_pair'):
                continue
            p_net_id = result.get('p_net_id')
            if p_net_id is None or p_net_id in processed_pairs:
                continue
            processed_pairs.add(p_net_id)
            if p_net_id in xnet_member_p_net_ids:
                continue  # matched end-to-end by the AC-couple pass (#196)

            # Get pair name for logging
            pair_info = diff_pair_by_net_id.get(net_id)
            pair_name = pair_info[0] if pair_info else f"net_{net_id}"

            print(f"\n{pair_name}:")
            seg_count_before = len(result.get('new_segments', []))
            apply_intra_pair_length_matching(result, config, pcb_data)
            seg_count_after = len(result.get('new_segments', []))

    # Apply end-to-end AC-coupled (XNet) length matching if configured (#196).
    # Runs AFTER group + intra-pair matching; for its member pairs it supersedes
    # per-side intra-pair (skipped above) by matching the concatenated P vs N path
    # and placing the compensating meanders on whichever segment has room.
    ac_coupled_summary = []
    if config.ac_couple_match and ac_xnets:
        from length_matching import apply_ac_coupled_length_matching
        print("\n" + "=" * 60)
        print("End-to-end AC-coupled diff-pair length matching (#196)")
        print("=" * 60)
        for _xn in ac_xnets:
            _skew = apply_ac_coupled_length_matching(_xn, routed_results, config, pcb_data)
            if _skew is not None:
                ac_coupled_summary.append({
                    'nets': "+".join(_xn.base_names),
                    'skew_mm': round(_skew, 4),
                    'bridges': _xn.bridge_refs,
                })

    # Sync pcb_data with length-matched segments
    sync_pcb_data_segments(pcb_data, routed_results, original_segment_ids, state, config)

    # ----- Post-route cleanup (#215) --------------------------------------
    # Mirror route.py's single-ended cleanup on the diff-pair copper. A redundant
    # connector overshoot can poke within clearance of the PARTNER track (a tight
    # terminal jog, a bare-pad target-swap stub): drop it when the net stays fully
    # connected without it (connectivity-gated, so a load-bearing track is kept),
    # then break any redundant cycle and trim the dead-end spur left behind.
    # Scoped to the diff-pair nets so other copper is untouched.
    from pcb_modification import prune_grazing_segments, prune_redundant_cycles, sweep_dead_ends
    # ONLY fully-routed pairs: a failed / single-ended-deferred pair's fanout stubs
    # have free ends by design (the single-ended follow-up connects them), so a
    # dead-end sweep there would strip copper the next pass needs.
    dp_scope = set()
    for _pn, _pair in diff_pair_ids_to_route:
        if _pair.p_net_id in routed_results and _pair.n_net_id in routed_results:
            dp_scope.add(_pair.p_net_id)
            dp_scope.add(_pair.n_net_id)
    cleanup_input_strip = []
    _gz, _gzn, _gz_in = prune_grazing_segments(
        results, pcb_data, dp_scope, clearance=config.clearance,
        check_foreign_segments=True)
    if _gz:
        print(f"Diff-pair graze prune: removed {_gz} grazing segment(s) across {_gzn} net(s)")
        cleanup_input_strip += _gz_in
    _cy, _cyn, _cy_in = prune_redundant_cycles(
        results, pcb_data, dp_scope, clearance=config.clearance)
    if _cy:
        print(f"Diff-pair cycle prune: removed {_cy} redundant loop segment(s) across {_cyn} net(s)")
        cleanup_input_strip += _cy_in
    _de, _dev, _de_in = sweep_dead_ends(results, pcb_data, dp_scope)
    if _de or _dev:
        print(f"Diff-pair dead-end sweep: trimmed {_de} segment(s), {_dev} via(s)")
        cleanup_input_strip += _de_in

    # Build summary data
    import json
    routed_diff_pairs = []
    failed_diff_pairs = []
    single_ended_diff_pairs = []
    for pair_name, pair in diff_pair_ids_to_route:
        if pair.p_net_id in routed_results and pair.n_net_id in routed_results:
            routed_diff_pairs.append(pair_name)
        elif (pair.p_net_id in state.diff_pair_single_ended_nets
              or pair.n_net_id in state.diff_pair_single_ended_nets):
            # Intentionally left for single-ended routing (electrically short, or
            # fully peeled) - not a coupled route, but not a failure either.
            single_ended_diff_pairs.append(pair_name)
        else:
            failed_diff_pairs.append(pair_name)

    # Count total vias from results
    total_vias = sum(len(r.get('new_vias', [])) for r in results)

    # Print human-readable summary
    print("\n" + "=" * 60)
    print("Routing complete")
    print("=" * 60)
    if failed_diff_pairs:
        print(f"  {RED}Diff pairs:    {len(routed_diff_pairs)}/{len(diff_pair_ids_to_route)} routed ({len(failed_diff_pairs)} FAILED){RESET}")
    else:
        print(f"  Diff pairs:    {len(routed_diff_pairs)}/{len(diff_pair_ids_to_route)} routed")
    if ripup_success_pairs:
        print(f"  Rip-up success: {len(ripup_success_pairs)} (routes that ripped blockers)")
    if rerouted_pairs:
        print(f"  Rerouted:      {len(rerouted_pairs)} (ripped nets re-routed)")
    if polarity_swapped_pairs:
        print(f"  Polarity swaps: {len(polarity_swapped_pairs)}")
    if target_swaps:
        swap_pairs = [(k, v) for k, v in target_swaps.items() if k < v]
        print(f"  Target swaps:  {len(swap_pairs)}")
    if single_ended_diff_pairs:
        print(f"  Single-ended:  {len(single_ended_diff_pairs)} (electrically short - "
              f"deferred to single-ended routing)")
    if skipped_bad_fanout:
        print(f"  {RED}Skipped:       {len(skipped_bad_fanout)} (fanout stubs self-overlap - "
              f"fix the fanout, #242){RESET}")
    print(f"  Total vias:    {total_vias}")
    print(f"  Total time:    {total_time:.2f}s")
    print(f"  Iterations:    {total_iterations:,}")
    summary = {
        'routed_diff_pairs': routed_diff_pairs,
        'failed_diff_pairs': failed_diff_pairs,
        'single_ended_diff_pairs': single_ended_diff_pairs,
        'ripup_success_pairs': sorted(ripup_success_pairs),
        'rerouted_pairs': sorted(rerouted_pairs),
        'polarity_swapped_pairs': sorted(polarity_swapped_pairs),
        'single_ended_followup_nets': sorted(state.diff_pair_single_ended_nets.values()),
        'skipped_bad_fanout': sorted(skipped_bad_fanout),
        'target_swaps': [{'pair1': k, 'pair2': v} for k, v in target_swaps.items() if k < v],
        'layer_swaps': total_layer_swaps,
        'successful': successful,
        'failed': failed,
        'total_time': round(total_time, 2),
        'total_iterations': total_iterations,
        'total_vias': total_vias,
        # Smallest copper clearance any step actually routed at (e.g. fine-pitch
        # taps below the nominal). Grade/check_drc the board at this floor.
        'min_clearance_used': __import__('clearance_ledger').effective(clearance),
    }
    if ac_coupled_summary:
        summary['ac_coupled_xnets'] = ac_coupled_summary
    print(f"JSON_SUMMARY: {json.dumps(summary)}")

    # Write output file or return results for direct application
    if return_results:
        # Return results data for direct application (e.g., KiCad plugin).
        # pad_swaps / target_swap_info / all_segment_modifications must be
        # applied to the live board just like write_routed_output applies them
        # to the output file - otherwise polarity/target swaps leave the routed
        # tracks pointing at pads that still carry the old net.
        results_data = {
            'results': results,
            'all_swap_vias': all_swap_vias,
            'all_swap_segments': all_swap_segments,
            'pad_swaps': pad_swaps,
            'target_swap_info': target_swap_info,
            'all_segment_modifications': all_segment_modifications,
            'exclusion_zone_lines': exclusion_zone_lines if debug_lines else [],
            'boundary_debug_labels': boundary_debug_labels if debug_lines else [],
        }
    else:
        wrote = write_routed_output(
            input_file=input_file,
            output_file=output_file,
            results=results,
            all_segment_modifications=all_segment_modifications,
            all_swap_vias=all_swap_vias,
            all_swap_segments=all_swap_segments,
            target_swap_info=target_swap_info,
            single_ended_target_swap_info=[],
            pad_swaps=pad_swaps,
            pcb_data=pcb_data,
            debug_lines=debug_lines,
            exclusion_zone_lines=exclusion_zone_lines,
            boundary_debug_labels=boundary_debug_labels,
            skip_routing=skip_routing,
            add_teardrops=add_teardrops,
            segments_to_remove=cleanup_input_strip or None
        )
        # When no coupled copper was written -- every pair was deferred to
        # single-ended (electrically short), OR a pair could not be routed at all
        # (terminal/launch escape exhausted, issue #167) -- still pass the board
        # through unchanged so the pipeline never loses its output file. The
        # unrouted nets are picked up by the downstream single-ended route.py pass
        # (the chains route '*' from the diff step's output); without this the
        # whole chain FileNotFoundErrors on the missing board (issues #90, #167).
        if not wrote and output_file:
            import shutil
            shutil.copy(input_file, output_file)
            if state.diff_pair_single_ended_nets:
                print(f"\nAll diff pairs deferred to single-ended (no coupled copper "
                      f"added); wrote board through to {output_file} for the "
                      f"single-ended pass")
            else:
                print(f"\nNo diff pair could be coupled-routed; wrote board through "
                      f"unchanged to {output_file} so the pipeline can continue "
                      f"(route the pair single-ended next)")

    # Update schematics with swap info if directory specified
    if schematic_dir and (target_swap_info or pad_swaps):
        schematic_swaps = []

        # Diff pair target swaps (P pads swap, N pads swap)
        for info in target_swap_info:
            if all(k in info for k in ['p1_p_pad', 'p2_p_pad', 'p1_n_pad', 'p2_n_pad']):
                p1_p = info['p1_p_pad']
                p2_p = info['p2_p_pad']
                p1_n = info['p1_n_pad']
                p2_n = info['p2_n_pad']
                # P pads swap
                if p1_p and p2_p and p1_p.component_ref == p2_p.component_ref:
                    schematic_swaps.append({
                        'component_ref': p1_p.component_ref,
                        'pad1': p1_p.pad_number,
                        'pad2': p2_p.pad_number
                    })
                # N pads swap
                if p1_n and p2_n and p1_n.component_ref == p2_n.component_ref:
                    schematic_swaps.append({
                        'component_ref': p1_n.component_ref,
                        'pad1': p1_n.pad_number,
                        'pad2': p2_n.pad_number
                    })

        # Polarity swaps (P/N swap within same diff pair)
        for swap_info in pad_swaps:
            if 'pad_p' in swap_info and 'pad_n' in swap_info:
                pad_p = swap_info['pad_p']
                pad_n = swap_info['pad_n']
                if pad_p and pad_n and pad_p.component_ref == pad_n.component_ref:
                    schematic_swaps.append({
                        'component_ref': pad_p.component_ref,
                        'pad1': pad_p.pad_number,
                        'pad2': pad_n.pad_number
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

    if return_results:
        return successful, failed, total_time, results_data
    return successful, failed, total_time

if __name__ == "__main__":
    import argparse
    from redo_record import record_invocation
    record_invocation()  # stress-test redo manifest (#132); no-op unless REDO_MANIFEST set

    parser = argparse.ArgumentParser(
        description="Batch Differential Pair PCB Router - Routes differential pairs using Rust-accelerated A*",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Wildcard patterns supported:
  "*lvds*"             - matches any net containing lvds
  "Net-(U1*DQS*)"      - matches Net-(U1-DQS_P), Net-(U1-DQS_N), etc.

All nets matching the patterns are treated as differential pairs.
Nets with _P/_N, P/N, or +/- suffixes will be paired and routed together.

Examples:
  python route_diff.py input.kicad_pcb output.kicad_pcb --nets "*lvds*"
  python route_diff.py input.kicad_pcb output.kicad_pcb --nets "Net-(U1*DQS*)" "Net-(U1*CK_*)"
"""
    )
    parser.add_argument("input_file", help="Input KiCad PCB file")
    parser.add_argument("output_file", nargs="?", help="Output KiCad PCB file (default: input_routed.kicad_pcb)")
    parser.add_argument("--output", metavar="FILE",
                        help="Output KiCad PCB file (named alias for the positional output_file)")
    parser.add_argument("net_patterns", nargs="*", help="Net names or wildcard patterns to route (default: '*' = all nets)")
    parser.add_argument("--nets", "-n", nargs="+", help="Net names or wildcard patterns to route (alternative to positional args)")
    parser.add_argument("--overwrite", "-O", action="store_true",
                        help="Overwrite input file instead of creating _routed copy")

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
    parser.add_argument("--layer-costs", nargs="+", type=float, default=None,
                        help="Per-layer cost multipliers (1.0-1000, or any negative value e.g. -1 = "
                             "forbidden: the layer is an obstacle / via span but carries no routed copper), one per --layers "
                             "entry, to bias which layer(s) coupled diff pairs prefer (e.g. '1 1 1 3' "
                             "to discourage the 4th layer). Default: 1.0 on every layer "
                             "(behavior unchanged). Matches route.py / route_planes.")

    # Track and via geometry
    parser.add_argument("--track-width", type=float, default=0.3,
                        help="Track width in mm (default: 0.3). Ignored if --impedance is specified.")
    parser.add_argument("--impedance", type=float, default=None,
                        help="Target differential impedance in ohms (e.g., 100). Calculates track width per layer from board stackup using --diff-pair-gap as spacing.")
    parser.add_argument("--clearance", type=float, default=0.25,
                        help="Clearance between tracks in mm (default: 0.25)")
    parser.add_argument("--via-size", type=float, default=0.5,
                        help="Via outer diameter in mm (default: 0.5)")
    parser.add_argument("--via-drill", type=float, default=0.3,
                        help="Via drill size in mm (default: 0.3)")

    # Router algorithm parameters
    parser.add_argument("--grid-step", type=float, default=0.1,
                        help="Grid resolution in mm (default: 0.1)")
    parser.add_argument("--via-cost", type=int, default=50,
                        help="Penalty for placing a via, in 0.1mm grid steps (default: 50 = 5mm of path, doubled for diff pairs; mm-equivalent at any --grid-step)")
    parser.add_argument("--via-proximity-cost", type=int, default=10,
                        help="Via cost multiplier in stub/BGA proximity zones (default: 10, 0=block vias)")
    parser.add_argument("--max-iterations", type=int, default=200000,
                        help="Max A* iterations before giving up (default: 200000)")
    parser.add_argument("--max-probe-iterations", type=int, default=5000,
                        help="Max iterations for quick probe phase per direction (default: 5000)")
    parser.add_argument("--heuristic-weight", type=float, default=1.9,
                        help="A* heuristic weight, higher=faster but less optimal (default: 1.9)")
    parser.add_argument("--proximity-heuristic-factor", type=float, default=0.02,
                        help="Factor for proximity-aware A* heuristic (default: 0.02, 0=disabled)")
    parser.add_argument("--turn-cost", type=int, default=1000,
                        help="Penalty for direction changes, encourages straighter paths (default: 1000)")
    parser.add_argument("--direction-preference-cost", type=int, default=defaults.DIRECTION_PREFERENCE_COST,
                        help=f"Penalty for non-preferred layer direction, 0=disabled (default: {defaults.DIRECTION_PREFERENCE_COST})")
    parser.add_argument("--bus", action="store_true",
                        help="Enable auto-detection and routing of bus groups (nets with clustered endpoints)")
    parser.add_argument("--bus-detection-radius", type=float, default=defaults.BUS_DETECTION_RADIUS,
                        help=f"Max endpoint distance to form bus in mm (default: {defaults.BUS_DETECTION_RADIUS})")
    parser.add_argument("--bus-attraction-radius", type=float, default=defaults.BUS_ATTRACTION_RADIUS,
                        help=f"Attraction radius from neighbor track in mm (default: {defaults.BUS_ATTRACTION_RADIUS})")
    parser.add_argument("--bus-attraction-bonus", type=int, default=defaults.BUS_ATTRACTION_BONUS,
                        help=f"Cost bonus for staying near neighbor track (default: {defaults.BUS_ATTRACTION_BONUS})")
    parser.add_argument("--bus-min-nets", type=int, default=defaults.BUS_MIN_NETS,
                        help=f"Minimum nets to form a bus group (default: {defaults.BUS_MIN_NETS})")
    parser.add_argument("--keepout", action="store_true",
                        help="Keep routed tracks out of polygons drawn on a User layer (issue #27)")
    parser.add_argument("--keepout-layer", type=str, default=defaults.KEEPOUT_LAYER,
                        help=f"User layer the keepout polygons are drawn on (default: {defaults.KEEPOUT_LAYER})")

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
                        help="Radius around routed tracks in mm, same layer only (0 = disabled, default: 2.0)")
    parser.add_argument("--track-proximity-cost", type=float, default=0.0,
                        help="Cost penalty near routed tracks (0 = disabled, default: 0.0)")

    # Differential pair routing options
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
    parser.add_argument("--schematic-dir", default=None,
                        help="Directory containing .kicad_sch files to update with pad swaps (default: no schematic update)")
    parser.add_argument("--crossing-penalty", type=float, default=1000.0,
                        help="Penalty for crossing assignments in target swap optimization (default: 1000.0)")
    parser.add_argument("--mps-reverse-rounds", action="store_true",
                        help="Reverse MPS round order: route most-conflicting groups first instead of least-conflicting")
    parser.add_argument("--mps-layer-swap", action="store_true",
                        help="Enable MPS-aware layer swaps to reduce crossing conflicts")
    parser.add_argument("--mps-segment-intersection", action="store_true",
                        help="Force MPS to use segment intersection for crossing detection")

    # Length matching options
    parser.add_argument("--length-match-group", action="append", nargs="+", dest="length_match_groups",
                        help="Net patterns to length-match as a group (can be repeated). Use 'auto' for DDR4 auto-grouping")
    parser.add_argument("--length-match-tolerance", type=float, default=0.1,
                        help="Acceptable length variance within group in mm (default: 0.1)")
    parser.add_argument("--meander-amplitude", type=float, default=1.0,
                        help="Height of meander perpendicular to trace in mm (default: 1.0)")

    # Time matching options (alternative to length matching)
    parser.add_argument("--time-matching", action="store_true",
                        help="Match by propagation time instead of length (accounts for layer dielectric)")
    parser.add_argument("--time-match-tolerance", type=float, default=1.0,
                        help="Acceptable time variance in picoseconds (default: 1.0)")

    parser.add_argument("--diff-chamfer-extra", type=float, default=1.5,
                        help="Chamfer multiplier for diff pair meanders (default: 1.5, >1 avoids P/N crossings)")
    parser.add_argument("--diff-pair-intra-match", action="store_true",
                        help="Enable intra-pair P/N length matching (add meanders to shorter track of each diff pair)")
    parser.add_argument("--ac-couple-match", action="store_true",
                        help="End-to-end length-match AC-coupled differential pairs split by series DC-blocking "
                             "caps (#196): auto-detect the cap chain, match the concatenated P path vs the N path, "
                             "and place the compensating meanders on whichever segment has room. Off by default.")

    # Rip-up and retry options
    parser.add_argument("--max-ripup", type=int, default=3,
                        help="Maximum blockers to rip up at once during rip-up and retry (default: 3)")
    parser.add_argument("--max-setback-angle", type=float, default=45.0,
                        help="Maximum angle (degrees) for setback position search (default: 45.0)")
    parser.add_argument("--routing-clearance-margin", type=float, default=1.0,
                        help="Multiplier on track-via clearance (1.0 = minimum DRC)")
    parser.add_argument("--hole-to-hole-clearance", type=float, default=defaults.HOLE_TO_HOLE_CLEARANCE,
                        help="Minimum clearance between drill holes in mm (default: 0.2)")
    parser.add_argument("--board-edge-clearance", type=float, default=0.0,
                        help="Clearance from board edge in mm (default: 0 = use track clearance)")
    parser.add_argument("--max-turn-angle", type=float, default=180.0,
                        help="Max cumulative turn angle (degrees) before reset, to prevent U-turns (default: 180)")

    # Vertical alignment attraction options
    parser.add_argument("--vertical-attraction-radius", type=float, default=1.0,
                        help="Radius in mm for cross-layer track attraction (0 = disabled, default: 1.0)")
    parser.add_argument("--vertical-attraction-cost", type=float, default=0.0,
                        help="Cost bonus for aligning with tracks on other layers (0 = disabled, default: 0.0)")

    # Ripped route avoidance options
    parser.add_argument("--ripped-route-avoidance-radius", type=float, default=defaults.RIPPED_ROUTE_AVOIDANCE_RADIUS,
                        help=f"Radius in mm around ripped route segments/vias for soft penalty (default: {defaults.RIPPED_ROUTE_AVOIDANCE_RADIUS})")
    parser.add_argument("--ripped-route-avoidance-cost", type=float, default=defaults.RIPPED_ROUTE_AVOIDANCE_COST,
                        help=f"Soft penalty cost for routing through ripped corridors (0 = disabled, default: {defaults.RIPPED_ROUTE_AVOIDANCE_COST})")

    # Debug options
    parser.add_argument("--debug-lines", action="store_true",
                        help="Output debug geometry on User.3 (connectors), User.4 (stub dirs), User.8 (simplified), User.9 (raw A*)")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Print detailed diagnostic output (setback checks, etc.)")
    parser.add_argument("--skip-routing", action="store_true",
                        help="Skip actual routing, only do swaps and write debug info")
    parser.add_argument("--debug-memory", action="store_true",
                        help="Print memory usage statistics at key points during routing")
    parser.add_argument("--add-teardrops", action="store_true",
                        help="Add teardrop settings to all pads in output file")
    parser.add_argument("--no-fix-drc-settings", action="store_true",
                        help="Do not rewrite the output project's DRC design rules to match "
                             "the routing floors (by default they are made consistent so "
                             "KiCad's manual DRC shows only genuine violations; issue #160)")
    parser.add_argument("--keep-thermal", action="store_true",
                        help="When fixing DRC settings, leave thermal-relief severity "
                             "(starved_thermal) untouched instead of demoting it to a warning")

    from fab_tiers import (add_fab_tier_args, fab_tier_from_args, set_default_fab_tier,
                           enforce_fab_floors, count_copper_layers_in_file)
    add_fab_tier_args(parser)
    args = parser.parse_args()
    set_default_fab_tier(*fab_tier_from_args(args))
    _pinned_floors = enforce_fab_floors(
        count_copper_layers_in_file(args.input_file),
        track_width=getattr(args, 'track_width', None),
        clearance=getattr(args, 'clearance', None),
        via_size=getattr(args, 'via_size', None),
        via_drill=getattr(args, 'via_drill', None),
        hole_to_hole_clearance=getattr(args, 'hole_to_hole_clearance', None))
    # Below-floor params are pinned up to the fab floor (warned); apply the clamps.
    for _pname, _pfloor in _pinned_floors.items():
        setattr(args, _pname, _pfloor)

    # --output is a named alias for the positional output_file; reject giving both differently.
    if args.output is not None:
        if args.output_file is not None and args.output_file != args.output:
            parser.error("specify the output path once: positional output_file OR --output, not both")
        args.output_file = args.output

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

    # Accept comma-separated patterns inside one token, e.g.
    # --nets "/DVI_CK_P,/DVI_CK_N" (issue #143): split so each is matched on its
    # own instead of as a single glob containing a comma (which matches nothing).
    all_patterns = [p.strip() for token in all_patterns for p in token.split(',') if p.strip()]

    # Default to "*" (all nets) if no patterns specified
    if not all_patterns:
        all_patterns = ["*"]

    # Warn loudly about any pattern that matches no REAL net, instead of silently
    # routing fewer pairs than asked for (issue #143). expand_net_patterns passes
    # an exact name through even when the board has no such net, so check the
    # expansion against the actual net names.
    real_names = {n.name for n in pcb_data.nets.values() if n.name}
    for pat in all_patterns:
        if pat == "*":
            continue
        if not any(nm in real_names for nm in expand_net_patterns(pcb_data, [pat])):
            print(f"WARNING: net pattern '{pat}' matched no net in this board.")

    # Expand patterns to net names
    net_names = expand_net_patterns(pcb_data, all_patterns)

    if not net_names:
        print("No nets matched the given patterns!")
        sys.exit(1)

    print(f"Routing {len(net_names)} nets as differential pairs: {net_names[:5]}{'...' if len(net_names) > 5 else ''}")

    batch_route_diff_pairs(args.input_file, args.output_file, net_names,
                direction_order=args.direction,
                ordering_strategy=args.ordering,
                disable_bga_zones=args.no_bga_zones,
                layers=args.layers,
                layer_costs=args.layer_costs,
                track_width=args.track_width,
                impedance=args.impedance,
                clearance=args.clearance,
                via_size=args.via_size,
                via_drill=args.via_drill,
                grid_step=args.grid_step,
                via_cost=args.via_cost,
                max_iterations=args.max_iterations,
                max_probe_iterations=args.max_probe_iterations,
                heuristic_weight=args.heuristic_weight,
                proximity_heuristic_factor=args.proximity_heuristic_factor,
                turn_cost=args.turn_cost,
                direction_preference_cost=args.direction_preference_cost,
                bus_enabled=args.bus,
                bus_detection_radius=args.bus_detection_radius,
                bus_attraction_radius=args.bus_attraction_radius,
                bus_attraction_bonus=args.bus_attraction_bonus,
                bus_min_nets=args.bus_min_nets,
                keepout_enabled=args.keepout,
                keepout_layer=args.keepout_layer,
                stub_proximity_radius=args.stub_proximity_radius,
                stub_proximity_cost=args.stub_proximity_cost,
                via_proximity_cost=args.via_proximity_cost,
                bga_proximity_radius=args.bga_proximity_radius,
                bga_proximity_cost=args.bga_proximity_cost,
                track_proximity_distance=args.track_proximity_distance,
                track_proximity_cost=args.track_proximity_cost,
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
                ripped_route_avoidance_radius=args.ripped_route_avoidance_radius,
                ripped_route_avoidance_cost=args.ripped_route_avoidance_cost,
                length_match_groups=args.length_match_groups,
                length_match_tolerance=args.length_match_tolerance,
                meander_amplitude=args.meander_amplitude,
                time_matching=args.time_matching,
                time_match_tolerance=args.time_match_tolerance,
                diff_chamfer_extra=args.diff_chamfer_extra,
                diff_pair_intra_match=args.diff_pair_intra_match,
                ac_couple_match=args.ac_couple_match,
                debug_memory=args.debug_memory,
                mps_reverse_rounds=args.mps_reverse_rounds,
                mps_layer_swap=args.mps_layer_swap,
                mps_segment_intersection=args.mps_segment_intersection,
                schematic_dir=args.schematic_dir,
                add_teardrops=args.add_teardrops)

    # Make the output project's DRC design rules consistent with the floors we
    # just routed to (issue #160), mirroring route.py, so a manual DRC in KiCad
    # flags only genuine problems instead of stock-default noise.
    if not args.no_fix_drc_settings and not args.skip_routing \
            and args.output_file and os.path.isfile(args.output_file):
        try:
            import clearance_ledger
            eff_clearance = clearance_ledger.effective(args.clearance)
            if eff_clearance < args.clearance:
                print(f"  Min clearance used: {eff_clearance:.4g} mm "
                      f"(below nominal {args.clearance:.4g}) - grading at this floor")
            from fix_kicad_drc_settings import fix_project_for_output
            fix_project_for_output(
                args.output_file, input_pcb=args.input_file,
                clearance=eff_clearance, hole_to_hole=args.hole_to_hole_clearance,
                edge_clearance=args.board_edge_clearance, track_width=args.track_width,
                via_diameter=args.via_size, via_drill=args.via_drill,
                diff_pair_gap=args.diff_pair_gap, diff_pair_width=args.track_width,
                keep_thermal=args.keep_thermal)
        except Exception as e:
            print(f"  (skipped DRC-settings fix: {e})")

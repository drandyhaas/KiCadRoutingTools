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
    swap_segment_nets_at_positions, swap_via_nets_at_positions, swap_pad_nets_in_content,
    modify_segment_layers
)

# Import from refactored modules
from routing_config import GridRouteConfig, GridCoord, DiffPair
from routing_utils import (
    find_differential_pairs, get_all_unrouted_net_ids, get_stub_endpoints,
    compute_mps_net_ordering, add_route_to_pcb_data, remove_route_from_pcb_data,
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
from blocking_analysis import analyze_frontier_blocking, print_blocking_analysis
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


def apply_polarity_swap(pcb_data: PCBData, result: dict, pad_swaps: list,
                        pair_name: str = None, already_swapped: set = None) -> bool:
    """
    Apply polarity swap for a diff pair route result.

    When a diff pair route requires P/N polarity swap at the target, this function:
    1. Swaps net IDs of stub segments at the target positions
    2. Swaps net IDs of vias at the target positions
    3. Queues pad swaps for later application to the output file

    If pair_name and already_swapped are provided, skips the swap if this pair
    was already swapped (e.g., before rip-up and reroute).

    Returns True if swap was applied, False if not needed or failed.
    """
    if not result.get('polarity_fixed') or not result.get('swap_target_pads'):
        return False

    # Skip if this pair was already polarity-swapped (prevents double-swap on reroute)
    if pair_name and already_swapped is not None:
        if pair_name in already_swapped:
            print(f"  Polarity swap already applied for {pair_name}, skipping")
            return False
        already_swapped.add(pair_name)

    swap_info = result['swap_target_pads']
    p_pos = swap_info['p_pos']
    n_pos = swap_info['n_pos']
    p_net_id = swap_info['p_net_id']
    n_net_id = swap_info['n_net_id']

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

    # Find the target pads for swap
    pad_p = find_pad_nearest_to_position(pcb_data, p_net_id, p_pos[0], p_pos[1])
    pad_n = find_pad_nearest_to_position(pcb_data, n_net_id, n_pos[0], n_pos[1])

    if pad_p and pad_n:
        pad_swaps.append({
            'pad_p': pad_p,
            'pad_n': pad_n,
            'p_net_id': p_net_id,
            'n_net_id': n_net_id,
            'p_stub_positions': p_stub_positions,
            'n_stub_positions': n_stub_positions,
        })
        print(f"  Polarity fixed: will swap nets of {pad_p.component_ref}:{pad_p.pad_number} <-> {pad_n.component_ref}:{pad_n.pad_number}")
        return True
    else:
        print(f"  WARNING: Could not find target pads to swap for polarity fix")
        if not pad_p:
            print(f"    Missing P pad (net {p_net_id}) near {p_pos}")
        if not pad_n:
            print(f"    Missing N pad (net {n_net_id}) near {n_pos}")
        return False


def get_canonical_net_id(net_id: int, diff_pair_by_net_id: dict) -> int:
    """Get canonical net ID for loop prevention tracking.

    For diff pairs, returns P net_id. For single nets, returns net_id as-is.
    """
    if net_id in diff_pair_by_net_id:
        _, pair = diff_pair_by_net_id[net_id]
        return pair.p_net_id
    return net_id


def rip_up_net(net_id: int, pcb_data, routed_net_ids: list, routed_net_paths: dict,
               routed_results: dict, diff_pair_by_net_id: dict, remaining_net_ids: list,
               results: list, config) -> tuple:
    """Rip up a routed net (or diff pair), removing it from pcb_data and tracking structures.

    Returns:
        tuple: (saved_result, ripped_net_ids, was_in_results) for later restoration
               saved_result: the result dict that was removed
               ripped_net_ids: list of net IDs that were ripped (1 for single, 2 for diff pair)
               was_in_results: True if the result was in the results list
    """
    if net_id not in routed_results:
        return None, [], False

    saved_result = routed_results[net_id]
    ripped_net_ids = []
    was_in_results = saved_result in results

    # Remove from pcb_data
    remove_route_from_pcb_data(pcb_data, saved_result)

    # Remove from results list if present
    if was_in_results:
        results.remove(saved_result)

    # Update tracking structures
    if net_id in diff_pair_by_net_id:
        # It's a diff pair - remove both P and N
        _, ripped_pair = diff_pair_by_net_id[net_id]
        ripped_net_ids = [ripped_pair.p_net_id, ripped_pair.n_net_id]

        if ripped_pair.p_net_id in routed_net_ids:
            routed_net_ids.remove(ripped_pair.p_net_id)
        if ripped_pair.n_net_id in routed_net_ids:
            routed_net_ids.remove(ripped_pair.n_net_id)
        routed_net_paths.pop(ripped_pair.p_net_id, None)
        routed_net_paths.pop(ripped_pair.n_net_id, None)
        routed_results.pop(ripped_pair.p_net_id, None)
        routed_results.pop(ripped_pair.n_net_id, None)
        # Add back to remaining so stubs are treated as obstacles
        if ripped_pair.p_net_id not in remaining_net_ids:
            remaining_net_ids.append(ripped_pair.p_net_id)
        if ripped_pair.n_net_id not in remaining_net_ids:
            remaining_net_ids.append(ripped_pair.n_net_id)
    else:
        # Single-ended net
        ripped_net_ids = [net_id]

        if net_id in routed_net_ids:
            routed_net_ids.remove(net_id)
        routed_net_paths.pop(net_id, None)
        routed_results.pop(net_id, None)
        # Add back to remaining so stubs are treated as obstacles
        if net_id not in remaining_net_ids:
            remaining_net_ids.append(net_id)

    return saved_result, ripped_net_ids, was_in_results


def restore_net(net_id: int, saved_result: dict, ripped_net_ids: list, was_in_results: bool,
                pcb_data, routed_net_ids: list, routed_net_paths: dict,
                routed_results: dict, diff_pair_by_net_id: dict, remaining_net_ids: list,
                results: list, config):
    """Restore a previously ripped net to pcb_data and tracking structures."""

    if saved_result is None:
        return

    # Add back to pcb_data
    add_route_to_pcb_data(pcb_data, saved_result, debug_lines=config.debug_lines)

    # Add back to results list if it was there
    if was_in_results:
        results.append(saved_result)

    # Restore tracking structures
    if net_id in diff_pair_by_net_id:
        # It's a diff pair
        _, ripped_pair = diff_pair_by_net_id[net_id]

        if ripped_pair.p_net_id not in routed_net_ids:
            routed_net_ids.append(ripped_pair.p_net_id)
        if ripped_pair.n_net_id not in routed_net_ids:
            routed_net_ids.append(ripped_pair.n_net_id)
        # Remove from remaining_net_ids since they're back to routed
        if ripped_pair.p_net_id in remaining_net_ids:
            remaining_net_ids.remove(ripped_pair.p_net_id)
        if ripped_pair.n_net_id in remaining_net_ids:
            remaining_net_ids.remove(ripped_pair.n_net_id)
        if saved_result.get('p_path'):
            routed_net_paths[ripped_pair.p_net_id] = saved_result['p_path']
        if saved_result.get('n_path'):
            routed_net_paths[ripped_pair.n_net_id] = saved_result['n_path']
        routed_results[ripped_pair.p_net_id] = saved_result
        routed_results[ripped_pair.n_net_id] = saved_result
    else:
        # Single-ended net
        if net_id not in routed_net_ids:
            routed_net_ids.append(net_id)
        if net_id in remaining_net_ids:
            remaining_net_ids.remove(net_id)
        if saved_result.get('path'):
            routed_net_paths[net_id] = saved_result['path']
        routed_results[net_id] = saved_result


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
                max_probe_iterations: int = 5000,
                heuristic_weight: float = 2.0,
                stub_proximity_radius: float = 2.0,
                stub_proximity_cost: float = 0.2,
                via_proximity_cost: float = 10.0,
                bga_proximity_radius: float = 10.0,
                bga_proximity_cost: float = 0.2,
                diff_pair_patterns: Optional[List[str]] = None,
                diff_pair_gap: float = 0.101,
                diff_pair_centerline_setback: float = None,
                min_turning_radius: float = 0.2,
                debug_lines: bool = False,
                fix_polarity: bool = True,
                max_rip_up_count: int = 3,
                enable_layer_switch: bool = True,
                can_swap_to_top_layer: bool = False,
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
        max_probe_iterations=max_probe_iterations,
        heuristic_weight=heuristic_weight,
        bga_exclusion_zones=bga_exclusion_zones,
        stub_proximity_radius=stub_proximity_radius,
        stub_proximity_cost=stub_proximity_cost,
        via_proximity_cost=via_proximity_cost,
        bga_proximity_radius=bga_proximity_radius,
        bga_proximity_cost=bga_proximity_cost,
        diff_pair_gap=diff_pair_gap,
        diff_pair_centerline_setback=diff_pair_centerline_setback,
        min_turning_radius=min_turning_radius,
        debug_lines=debug_lines,
        fix_polarity=fix_polarity,
        max_rip_up_count=max_rip_up_count,
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

    # Track all segment layer modifications for file output
    all_segment_modifications = []
    # Track all vias added during stub layer swapping
    all_swap_vias = []

    # Upfront layer swap optimization: analyze all diff pairs and apply beneficial swaps
    # BEFORE building obstacle maps, so obstacles reflect correct segment layers
    if enable_layer_switch and diff_pair_ids_to_route:
        from stub_layer_switching import get_stub_info, apply_stub_layer_switch
        from diff_pair_routing import get_diff_pair_endpoints

        print(f"\nAnalyzing layer swaps for {len(diff_pair_ids_to_route)} diff pair(s)...")

        # Build set of pair names being routed
        pairs_being_routed = {name for name, _ in diff_pair_ids_to_route}

        # Collect layer info for pairs we're routing
        pair_layer_info = {}  # pair_name -> (src_layer, tgt_layer, sources, targets)
        for pair_name, pair in diff_pair_ids_to_route:
            sources, targets, error = get_diff_pair_endpoints(pcb_data, pair.p_net_id, pair.n_net_id, config)
            if error or not sources or not targets:
                continue
            src_layer = config.layers[sources[0][4]]
            tgt_layer = config.layers[targets[0][4]]
            pair_layer_info[pair_name] = (src_layer, tgt_layer, sources, targets, pair)

        # Build layer info for ALL diff pairs (for finding swap partners)
        all_pair_layer_info = {}  # pair_name -> (src_layer, tgt_layer, sources, targets, pair)
        for pair_name, pair in diff_pairs.items():
            sources, targets, error = get_diff_pair_endpoints(pcb_data, pair.p_net_id, pair.n_net_id, config)
            if error or not sources or not targets:
                continue
            src_layer = config.layers[sources[0][4]]
            tgt_layer = config.layers[targets[0][4]]
            all_pair_layer_info[pair_name] = (src_layer, tgt_layer, sources, targets, pair)

        # Find pairs that need layer switches (src != tgt layer)
        pairs_needing_via = [(name, info) for name, info in pair_layer_info.items()
                            if info[0] != info[1]]

        # Try to find swap partners for pairs needing via
        applied_swaps = set()
        swap_count = 0

        for pair_name, (src_layer, tgt_layer, sources, targets, pair) in pairs_needing_via:
            if pair_name in applied_swaps:
                continue

            # Get our source stub info
            src_p_stub = get_stub_info(pcb_data, pair.p_net_id,
                                       sources[0][5], sources[0][6], src_layer)
            src_n_stub = get_stub_info(pcb_data, pair.n_net_id,
                                       sources[0][7], sources[0][8], src_layer)

            if not src_p_stub or not src_n_stub:
                continue

            swap_partner = None
            swap_partner_stubs = None

            # Find which nets on target layer actually overlap with our stub segments
            overlapping_nets = set()
            our_stubs = src_p_stub.segments + src_n_stub.segments
            for stub_seg in our_stubs:
                stub_y_min = min(stub_seg.start_y, stub_seg.end_y) - 0.2
                stub_y_max = max(stub_seg.start_y, stub_seg.end_y) + 0.2
                stub_x_min = min(stub_seg.start_x, stub_seg.end_x)
                stub_x_max = max(stub_seg.start_x, stub_seg.end_x)

                for seg in pcb_data.segments:
                    if seg.layer != tgt_layer:
                        continue
                    seg_y_min = min(seg.start_y, seg.end_y)
                    seg_y_max = max(seg.start_y, seg.end_y)
                    seg_x_min = min(seg.start_x, seg.end_x)
                    seg_x_max = max(seg.start_x, seg.end_x)

                    # Check Y and X overlap
                    if seg_y_max >= stub_y_min and seg_y_min <= stub_y_max:
                        if seg_x_max >= stub_x_min and seg_x_min <= stub_x_max:
                            overlapping_nets.add(seg.net_id)

            # Find which diff pair the overlapping nets belong to
            for other_name, other_info in all_pair_layer_info.items():
                if other_name == pair_name:
                    continue
                other_src_layer, other_tgt_layer, other_sources, other_targets, other_pair = other_info

                # Check if their source is on our target layer and overlaps
                if other_src_layer != tgt_layer:
                    continue
                if other_pair.p_net_id not in overlapping_nets and other_pair.n_net_id not in overlapping_nets:
                    continue

                # Get their source stub info
                other_src_p_stub = get_stub_info(pcb_data, other_pair.p_net_id,
                                                 other_sources[0][5], other_sources[0][6], other_src_layer)
                other_src_n_stub = get_stub_info(pcb_data, other_pair.n_net_id,
                                                 other_sources[0][7], other_sources[0][8], other_src_layer)

                if other_src_p_stub and other_src_n_stub:
                    swap_partner = other_name
                    swap_partner_stubs = (other_src_p_stub, other_src_n_stub, other_src_layer)
                    break

            if swap_partner and swap_partner_stubs:
                # Found a swap partner! Swap source layers
                other_src_p_stub, other_src_n_stub, other_src_layer = swap_partner_stubs

                # Check if swap would move stubs to F.Cu (top layer)
                # Skip if can_swap_to_top_layer is False and either destination is F.Cu
                if not can_swap_to_top_layer and (tgt_layer == 'F.Cu' or src_layer == 'F.Cu'):
                    # Skip this swap - would move stubs to top layer
                    continue

                # Our source: src_layer -> tgt_layer
                # Their source: other_src_layer (=tgt_layer) -> src_layer
                vias1, mods1 = apply_stub_layer_switch(pcb_data, src_p_stub, tgt_layer, config, debug=False)
                vias2, mods2 = apply_stub_layer_switch(pcb_data, src_n_stub, tgt_layer, config, debug=False)
                vias3, mods3 = apply_stub_layer_switch(pcb_data, other_src_p_stub, src_layer, config, debug=False)
                vias4, mods4 = apply_stub_layer_switch(pcb_data, other_src_n_stub, src_layer, config, debug=False)
                all_segment_modifications.extend(mods1 + mods2 + mods3 + mods4)
                all_vias = vias1 + vias2 + vias3 + vias4
                all_swap_vias.extend(all_vias)

                applied_swaps.add(pair_name)
                applied_swaps.add(swap_partner)
                swap_count += 1
                via_msg = f", added {len(all_vias)} pad via(s)" if all_vias else ""
                print(f"  Source swap: {pair_name} ({src_layer}->{tgt_layer}) <-> {swap_partner} ({other_src_layer}->{src_layer}){via_msg}")

        if swap_count > 0:
            print(f"Applied {swap_count} source layer swap(s)")

        # Now try target-side segment overlap swaps for remaining pairs
        target_swap_count = 0
        for pair_name, (src_layer, tgt_layer, sources, targets, pair) in pairs_needing_via:
            if pair_name in applied_swaps:
                continue

            # Get our target stub info
            tgt_p_stub = get_stub_info(pcb_data, pair.p_net_id,
                                       targets[0][5], targets[0][6], tgt_layer)
            tgt_n_stub = get_stub_info(pcb_data, pair.n_net_id,
                                       targets[0][7], targets[0][8], tgt_layer)

            if not tgt_p_stub or not tgt_n_stub:
                continue

            swap_partner = None
            swap_partner_stubs = None

            # Find which nets on source layer actually overlap with our target stub segments
            overlapping_nets = set()
            our_stubs = tgt_p_stub.segments + tgt_n_stub.segments
            for stub_seg in our_stubs:
                stub_y_min = min(stub_seg.start_y, stub_seg.end_y) - 0.2
                stub_y_max = max(stub_seg.start_y, stub_seg.end_y) + 0.2
                stub_x_min = min(stub_seg.start_x, stub_seg.end_x)
                stub_x_max = max(stub_seg.start_x, stub_seg.end_x)

                for seg in pcb_data.segments:
                    if seg.layer != src_layer:
                        continue
                    seg_y_min = min(seg.start_y, seg.end_y)
                    seg_y_max = max(seg.start_y, seg.end_y)
                    seg_x_min = min(seg.start_x, seg.end_x)
                    seg_x_max = max(seg.start_x, seg.end_x)

                    # Check Y and X overlap
                    if seg_y_max >= stub_y_min and seg_y_min <= stub_y_max:
                        if seg_x_max >= stub_x_min and seg_x_min <= stub_x_max:
                            overlapping_nets.add(seg.net_id)

            # Find which diff pair the overlapping nets belong to
            for other_name, other_info in all_pair_layer_info.items():
                if other_name == pair_name:
                    continue
                other_src_layer, other_tgt_layer, other_sources, other_targets, other_pair = other_info

                # Check if their target is on our source layer and overlaps
                if other_tgt_layer != src_layer:
                    continue
                if other_pair.p_net_id not in overlapping_nets and other_pair.n_net_id not in overlapping_nets:
                    continue

                # Get their target stub info
                other_tgt_p_stub = get_stub_info(pcb_data, other_pair.p_net_id,
                                                 other_targets[0][5], other_targets[0][6], other_tgt_layer)
                other_tgt_n_stub = get_stub_info(pcb_data, other_pair.n_net_id,
                                                 other_targets[0][7], other_targets[0][8], other_tgt_layer)

                if other_tgt_p_stub and other_tgt_n_stub:
                    swap_partner = other_name
                    swap_partner_stubs = (other_tgt_p_stub, other_tgt_n_stub, other_tgt_layer)
                    break

            if swap_partner and swap_partner_stubs:
                # Found a swap partner! Swap target layers
                other_tgt_p_stub, other_tgt_n_stub, other_tgt_layer = swap_partner_stubs

                # Check if swap would move stubs to F.Cu (top layer)
                # Skip if can_swap_to_top_layer is False and either destination is F.Cu
                if not can_swap_to_top_layer and (src_layer == 'F.Cu' or tgt_layer == 'F.Cu'):
                    # Skip this swap - would move stubs to top layer
                    continue

                # Our target: tgt_layer -> src_layer
                # Their target: other_tgt_layer (=src_layer) -> tgt_layer
                vias1, mods1 = apply_stub_layer_switch(pcb_data, tgt_p_stub, src_layer, config, debug=False)
                vias2, mods2 = apply_stub_layer_switch(pcb_data, tgt_n_stub, src_layer, config, debug=False)
                vias3, mods3 = apply_stub_layer_switch(pcb_data, other_tgt_p_stub, tgt_layer, config, debug=False)
                vias4, mods4 = apply_stub_layer_switch(pcb_data, other_tgt_n_stub, tgt_layer, config, debug=False)
                all_segment_modifications.extend(mods1 + mods2 + mods3 + mods4)
                all_vias = vias1 + vias2 + vias3 + vias4
                all_swap_vias.extend(all_vias)

                applied_swaps.add(pair_name)
                applied_swaps.add(swap_partner)
                target_swap_count += 1
                via_msg = f", added {len(all_vias)} pad via(s)" if all_vias else ""
                print(f"  Target swap: {pair_name} ({tgt_layer}->{src_layer}) <-> {swap_partner} ({other_tgt_layer}->{tgt_layer}){via_msg}")

        if target_swap_count > 0:
            print(f"Applied {target_swap_count} target layer swap(s)")

        # Now try two-pair swaps for remaining pairs that weren't handled
        for pair_name, (src_layer, tgt_layer, sources, targets, pair) in pairs_needing_via:
            if pair_name in applied_swaps:
                continue

            # Look for another pair that also needs a via to swap with
            # Option 1: Swap sources - we want our source to become tgt_layer
            # Option 2: Swap targets - we want our target to become src_layer
            for other_name, other_info in pairs_needing_via:
                if other_name in applied_swaps or other_name == pair_name:
                    continue
                other_src, other_tgt, other_sources, other_targets, other_pair = other_info

                swap_type = None
                our_stubs = None
                their_stubs = None
                our_new_layer = None
                their_new_layer = None

                # Option 1: Swap sources
                # Their source is on our target layer, our source is on their target layer
                if other_src == tgt_layer and src_layer == other_tgt:
                    swap_type = "source"
                    our_new_layer = tgt_layer
                    their_new_layer = src_layer
                    # Our source stubs
                    p_stub = get_stub_info(pcb_data, pair.p_net_id,
                                          sources[0][5], sources[0][6], src_layer)
                    n_stub = get_stub_info(pcb_data, pair.n_net_id,
                                          sources[0][7], sources[0][8], src_layer)
                    # Their source stubs
                    other_p_stub = get_stub_info(pcb_data, other_pair.p_net_id,
                                                other_sources[0][5], other_sources[0][6], other_src)
                    other_n_stub = get_stub_info(pcb_data, other_pair.n_net_id,
                                                other_sources[0][7], other_sources[0][8], other_src)
                    our_stubs = (p_stub, n_stub)
                    their_stubs = (other_p_stub, other_n_stub)

                # Option 2: Swap targets
                # Their target is on our source layer, our target is on their source layer
                elif other_tgt == src_layer and tgt_layer == other_src:
                    swap_type = "target"
                    our_new_layer = src_layer
                    their_new_layer = tgt_layer
                    # Our target stubs
                    p_stub = get_stub_info(pcb_data, pair.p_net_id,
                                          targets[0][5], targets[0][6], tgt_layer)
                    n_stub = get_stub_info(pcb_data, pair.n_net_id,
                                          targets[0][7], targets[0][8], tgt_layer)
                    # Their target stubs
                    other_p_stub = get_stub_info(pcb_data, other_pair.p_net_id,
                                                other_targets[0][5], other_targets[0][6], other_tgt)
                    other_n_stub = get_stub_info(pcb_data, other_pair.n_net_id,
                                                other_targets[0][7], other_targets[0][8], other_tgt)
                    our_stubs = (p_stub, n_stub)
                    their_stubs = (other_p_stub, other_n_stub)

                # Try swap if we have valid stubs, otherwise try the other side
                if swap_type and our_stubs[0] and our_stubs[1] and their_stubs[0] and their_stubs[1]:
                    # Check if swap would move stubs to F.Cu (top layer)
                    if not can_swap_to_top_layer and (our_new_layer == 'F.Cu' or their_new_layer == 'F.Cu'):
                        # Skip this swap - would move stubs to top layer
                        pass
                    else:
                        # Apply swaps
                        _, mods1 = apply_stub_layer_switch(pcb_data, our_stubs[0], our_new_layer, config, debug=False)
                        _, mods2 = apply_stub_layer_switch(pcb_data, our_stubs[1], our_new_layer, config, debug=False)
                        _, mods3 = apply_stub_layer_switch(pcb_data, their_stubs[0], their_new_layer, config, debug=False)
                        _, mods4 = apply_stub_layer_switch(pcb_data, their_stubs[1], their_new_layer, config, debug=False)
                        all_segment_modifications.extend(mods1 + mods2 + mods3 + mods4)

                        applied_swaps.add(pair_name)
                        applied_swaps.add(other_name)
                        swap_count += 1
                        print(f"  Swap {swap_type}s: {pair_name} <-> {other_name}")
                        break
                if swap_type == "source":
                    # Source swap failed, try target swap with same pair
                    if other_tgt == src_layer and tgt_layer == other_src:
                        # Our target stubs
                        p_stub = get_stub_info(pcb_data, pair.p_net_id,
                                              targets[0][5], targets[0][6], tgt_layer)
                        n_stub = get_stub_info(pcb_data, pair.n_net_id,
                                              targets[0][7], targets[0][8], tgt_layer)
                        # Their target stubs
                        other_p_stub = get_stub_info(pcb_data, other_pair.p_net_id,
                                                    other_targets[0][5], other_targets[0][6], other_tgt)
                        other_n_stub = get_stub_info(pcb_data, other_pair.n_net_id,
                                                    other_targets[0][7], other_targets[0][8], other_tgt)

                        if p_stub and n_stub and other_p_stub and other_n_stub:
                            # Check if swap would move stubs to F.Cu (top layer)
                            if not can_swap_to_top_layer and (src_layer == 'F.Cu' or tgt_layer == 'F.Cu'):
                                # Skip this swap - would move stubs to top layer
                                pass
                            else:
                                _, mods1 = apply_stub_layer_switch(pcb_data, p_stub, src_layer, config, debug=False)
                                _, mods2 = apply_stub_layer_switch(pcb_data, n_stub, src_layer, config, debug=False)
                                _, mods3 = apply_stub_layer_switch(pcb_data, other_p_stub, tgt_layer, config, debug=False)
                                _, mods4 = apply_stub_layer_switch(pcb_data, other_n_stub, tgt_layer, config, debug=False)
                                all_segment_modifications.extend(mods1 + mods2 + mods3 + mods4)

                                applied_swaps.add(pair_name)
                                applied_swaps.add(other_name)
                                swap_count += 1
                                print(f"  Swap targets: {pair_name} <-> {other_name}")
                                break

        if swap_count > 0:
            print(f"Applied {swap_count} layer swap(s)")

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
    # Extra clearance = offset from centerline to P/N track outer edge
    # (P/N tracks extend offset + track_width/2 from centerline)
    diff_pair_extra_clearance = (config.track_width + config.diff_pair_gap) / 2 + config.track_width / 2
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
        # Note: Layer switching is now done upfront before routing starts
        result = route_diff_pair_with_obstacles(pcb_data, pair, config, obstacles, base_obstacles,
                                                 unrouted_stubs)
        elapsed = time.time() - start_time
        total_time += elapsed

        if result and not result.get('failed'):
            print(f"  SUCCESS: {len(result['new_segments'])} segments, {len(result['new_vias'])} vias, {result['iterations']} iterations ({elapsed:.2f}s)")
            results.append(result)
            successful += 1
            total_iterations += result['iterations']

            # Check if polarity was fixed - need to swap target pad and stub nets
            # Do this BEFORE add_route_to_pcb_data so collapse_appendices sees correct net IDs
            apply_polarity_swap(pcb_data, result, pad_swaps, pair_name, polarity_swapped_pairs)

            add_route_to_pcb_data(pcb_data, result, debug_lines=config.debug_lines)

            if pair.p_net_id in remaining_net_ids:
                remaining_net_ids.remove(pair.p_net_id)
            if pair.n_net_id in remaining_net_ids:
                remaining_net_ids.remove(pair.n_net_id)
            routed_net_ids.append(pair.p_net_id)
            routed_net_ids.append(pair.n_net_id)
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
                        extra_clearance=diff_pair_extra_clearance
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
                    retry_succeeded = False

                    for N in range(1, config.max_rip_up_count + 1):
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
                            print(f"  Extending to N={N}: ripping additional blocker(s)...")

                        for i in range(len(ripped_items), N):
                            blocker = rippable_blockers[i]
                            # Skip if already ripped (e.g., as part of a diff pair)
                            if blocker.net_id not in routed_results:
                                continue
                            saved_result, ripped_ids, was_in_results = rip_up_net(
                                blocker.net_id, pcb_data, routed_net_ids, routed_net_paths,
                                routed_results, diff_pair_by_net_id, remaining_net_ids,
                                results, config
                            )
                            if saved_result is None:
                                rip_successful = False
                                break
                            ripped_items.append((blocker.net_id, saved_result, ripped_ids, was_in_results))
                            new_ripped_this_level.append((blocker.net_id, saved_result, ripped_ids, was_in_results))
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
                                           results, config)
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
                        add_same_net_via_clearance(retry_obstacles, pcb_data, pair.p_net_id, config)
                        add_same_net_via_clearance(retry_obstacles, pcb_data, pair.n_net_id, config)

                        retry_result = route_diff_pair_with_obstacles(pcb_data, pair, config, retry_obstacles, base_obstacles, unrouted_stubs)

                        if retry_result and not retry_result.get('failed'):
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

                    # If all N levels failed, restore all ripped nets
                    if not retry_succeeded and ripped_items:
                        print(f"  All rip-up attempts failed: Restoring {len(ripped_items)} net(s)")
                        for net_id, saved_result, ripped_ids, was_in_results in reversed(ripped_items):
                            restore_net(net_id, saved_result, ripped_ids, was_in_results,
                                       pcb_data, routed_net_ids, routed_net_paths,
                                       routed_results, diff_pair_by_net_id, remaining_net_ids,
                                       results, config)
                            if was_in_results:
                                successful += 1

            if not ripped_up:
                failed += 1

    # Route ripped-up nets (with rip-up retry support)
    reroute_index = 0
    while reroute_index < len(reroute_queue):
        reroute_item = reroute_queue[reroute_index]
        reroute_index += 1

        if reroute_item[0] == 'diff_pair':
            _, ripped_pair_name, ripped_pair = reroute_item
            route_index += 1
            print(f"\n[REROUTE {route_index}] Re-routing ripped diff pair {ripped_pair_name}")
            print("-" * 40)

            start_time = time.time()
            obstacles = diff_pair_base_obstacles.clone()
            for routed_id in routed_net_ids:
                add_net_stubs_as_obstacles(obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
                add_net_vias_as_obstacles(obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
                add_net_pads_as_obstacles(obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
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
            add_same_net_via_clearance(obstacles, pcb_data, ripped_pair.p_net_id, config)
            add_same_net_via_clearance(obstacles, pcb_data, ripped_pair.n_net_id, config)

            result = route_diff_pair_with_obstacles(pcb_data, ripped_pair, config, obstacles, base_obstacles, unrouted_stubs)
            elapsed = time.time() - start_time
            total_time += elapsed

            if result and not result.get('failed'):
                print(f"  REROUTE SUCCESS: {len(result['new_segments'])} segments, {len(result['new_vias'])} vias ({elapsed:.2f}s)")
                results.append(result)
                successful += 1
                total_iterations += result['iterations']
                rerouted_pairs.add(ripped_pair_name)

                # Apply polarity swap if needed (same as original route path)
                # Pass pair_name and set to avoid double-swap if already done before rip-up
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
            else:
                # Reroute failed - try rip-up and retry with progressive N+1
                iterations = result['iterations'] if result else 0
                total_iterations += iterations
                print(f"  REROUTE FAILED: ({elapsed:.2f}s) - attempting rip-up and retry...")

                reroute_succeeded = False
                if routed_net_paths and result:
                    # Find blocked cells from the failed route
                    fwd_iters = result.get('iterations_forward', 0)
                    bwd_iters = result.get('iterations_backward', 0)
                    if fwd_iters > 0 and (bwd_iters == 0 or fwd_iters <= bwd_iters):
                        blocked_cells = result.get('blocked_cells_forward', [])
                    else:
                        blocked_cells = result.get('blocked_cells_backward', [])

                    if blocked_cells:
                        blockers = analyze_frontier_blocking(
                            blocked_cells, pcb_data, config, routed_net_paths,
                            exclude_net_ids={ripped_pair.p_net_id, ripped_pair.n_net_id},
                            extra_clearance=diff_pair_extra_clearance
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

                        # Progressive rip-up: try N=1, then N=2, etc
                        # Keep nets ripped between N levels to avoid redundant restore/re-rip
                        ripped_items = []  # Accumulate ripped items across N levels

                        for N in range(1, config.max_rip_up_count + 1):
                            if N > len(rippable_blockers):
                                break

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
                                print(f"  Skipping N={N}: already tried ripping {blockers_str} for {ripped_pair_name}")
                                continue

                            # Rip up only the new blocker(s) for this N level
                            rip_successful = True
                            new_ripped_this_level = []

                            if N == 1:
                                blocker = rippable_blockers[0]
                                if blocker.net_id in diff_pair_by_net_id:
                                    ripped_pair_name_tmp, _ = diff_pair_by_net_id[blocker.net_id]
                                    print(f"  Ripping up diff pair {ripped_pair_name_tmp} (P and N) to retry reroute...")
                                else:
                                    print(f"  Ripping up {blocker.net_name} to retry reroute...")
                            else:
                                print(f"  Extending to N={N}: ripping additional blocker(s) for reroute...")

                            for i in range(len(ripped_items), N):
                                blocker = rippable_blockers[i]
                                # Skip if already ripped (e.g., as part of a diff pair)
                                if blocker.net_id not in routed_results:
                                    continue
                                saved_result, ripped_ids, was_in_results = rip_up_net(
                                    blocker.net_id, pcb_data, routed_net_ids, routed_net_paths,
                                    routed_results, diff_pair_by_net_id, remaining_net_ids,
                                    results, config
                                )
                                if saved_result is None:
                                    rip_successful = False
                                    break
                                ripped_items.append((blocker.net_id, saved_result, ripped_ids, was_in_results))
                                new_ripped_this_level.append((blocker.net_id, saved_result, ripped_ids, was_in_results))
                                if was_in_results:
                                    successful -= 1
                                if blocker.net_id in diff_pair_by_net_id:
                                    ripped_pair_name_tmp, _ = diff_pair_by_net_id[blocker.net_id]
                                    print(f"    Ripped diff pair {ripped_pair_name_tmp}")
                                else:
                                    print(f"    Ripped {blocker.net_name}")

                            if not rip_successful:
                                # Restore only the nets we just ripped this level
                                for net_id, saved_result_tmp, ripped_ids, was_in_results in reversed(new_ripped_this_level):
                                    restore_net(net_id, saved_result_tmp, ripped_ids, was_in_results,
                                               pcb_data, routed_net_ids, routed_net_paths,
                                               routed_results, diff_pair_by_net_id, remaining_net_ids,
                                               results, config)
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
                            add_same_net_via_clearance(retry_obstacles, pcb_data, ripped_pair.p_net_id, config)
                            add_same_net_via_clearance(retry_obstacles, pcb_data, ripped_pair.n_net_id, config)

                            retry_result = route_diff_pair_with_obstacles(pcb_data, ripped_pair, config, retry_obstacles, base_obstacles, unrouted_stubs)

                            if retry_result and not retry_result.get('failed'):
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

                                # Queue ripped nets and add to history
                                rip_and_retry_history.add((current_canonical, blocker_canonicals))

                                for net_id, saved_result_tmp, ripped_ids, was_in_results in ripped_items:
                                    if net_id in diff_pair_by_net_id:
                                        ripped_pair_name_tmp, ripped_pair_tmp = diff_pair_by_net_id[net_id]
                                        reroute_queue.append(('diff_pair', ripped_pair_name_tmp, ripped_pair_tmp))
                                    else:
                                        net = pcb_data.nets.get(net_id)
                                        net_name_tmp = net.name if net else f"Net {net_id}"
                                        reroute_queue.append(('single', net_name_tmp, net_id))

                                reroute_succeeded = True
                                break
                            else:
                                # Retry failed - keep nets ripped for N+1 attempt
                                print(f"  REROUTE RETRY FAILED (N={N})")

                        # If all N levels failed, restore all ripped nets
                        if not reroute_succeeded and ripped_items:
                            print(f"  All rip-up attempts failed: Restoring {len(ripped_items)} net(s)")
                            for net_id, saved_result_tmp, ripped_ids, was_in_results in reversed(ripped_items):
                                restore_net(net_id, saved_result_tmp, ripped_ids, was_in_results,
                                           pcb_data, routed_net_ids, routed_net_paths,
                                           routed_results, diff_pair_by_net_id, remaining_net_ids,
                                           results, config)
                                if was_in_results:
                                    successful += 1

                if not reroute_succeeded:
                    failed += 1

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
            # Track path for blocking analysis
            if result.get('path'):
                routed_net_paths[net_id] = result['path']
        else:
            iterations = result['iterations'] if result else 0
            print(f"  FAILED: Could not find route ({elapsed:.2f}s)")
            failed += 1
            total_iterations += iterations
            # Analyze which nets are blocking if we have frontier data
            if routed_net_paths and result:
                for direction in ['forward', 'backward']:
                    blocked_cells = result.get(f'blocked_cells_{direction}', [])
                    dir_iters = result.get(f'iterations_{direction}', 0)
                    if blocked_cells:
                        print(f"  {direction.capitalize()} direction ({dir_iters} iterations, {len(blocked_cells)} blocked cells):")
                        blockers = analyze_frontier_blocking(
                            blocked_cells, pcb_data, config, routed_net_paths,
                            exclude_net_ids={net_id},
                        )
                        print_blocking_analysis(blockers)

    # Notify visualization callback that all routing is complete
    if visualize:
        vis_callback.on_routing_complete(successful, failed, total_iterations)

    print("\n" + "=" * 60)
    print(f"Routing complete: {successful} successful, {failed} failed")
    print(f"Total time: {total_time:.2f}s")
    print(f"Total iterations: {total_iterations}")

    # Build and print JSON summary for reliable parsing
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
    summary = {
        'routed_diff_pairs': routed_diff_pairs,
        'failed_diff_pairs': failed_diff_pairs,
        'routed_single': routed_single,
        'failed_single': failed_single,
        'ripup_success_pairs': sorted(ripup_success_pairs),
        'rerouted_pairs': sorted(rerouted_pairs),
        'polarity_swapped_pairs': sorted(polarity_swapped_pairs),
        'successful': successful,
        'failed': failed,
        'total_time': round(total_time, 2),
        'total_iterations': total_iterations,
        'total_vias': total_vias
    }
    print(f"JSON_SUMMARY: {json.dumps(summary)}")

    # Write output if we have results OR if we have layer swap modifications to show
    if results or all_segment_modifications or all_swap_vias:
        print(f"\nWriting output to {output_file}...")
        with open(input_file, 'r', encoding='utf-8') as f:
            content = f.read()

        # Apply segment layer modifications from stub layer switching BEFORE polarity swaps
        # (so we can find segments by their original net_id)
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
                    via.layers, via.net_id
                ) + "\n"

        # Add vias from stub layer swapping
        if all_swap_vias:
            print(f"Adding {len(all_swap_vias)} via(s) from stub layer swapping")
            for via in all_swap_vias:
                routing_text += generate_via_sexpr(
                    via.x, via.y, via.size, via.drill,
                    via.layers, via.net_id
                ) + "\n"

        # Add debug paths if enabled (using gr_line for User layers)
        if debug_lines:
            print("Adding debug paths to User.3 (connectors), User.4 (stub dirs), User.8 (simplified), User.9 (raw A*)")
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
    parser.add_argument("--via-cost", type=int, default=25,
                        help="Penalty for placing a via in grid steps (default: 25, doubled for diff pairs)")
    parser.add_argument("--max-iterations", type=int, default=200000,
                        help="Max A* iterations before giving up (default: 200000)")
    parser.add_argument("--max-probe-iterations", type=int, default=5000,
                        help="Max iterations for quick probe phase per direction (default: 5000)")
    parser.add_argument("--heuristic-weight", type=float, default=2.0,
                        help="A* heuristic weight, higher=faster but less optimal (default: 2.0)")

    # Stub proximity penalty
    parser.add_argument("--stub-proximity-radius", type=float, default=2.0,
                        help="Radius around stubs to penalize routing in mm (default: 2.0)")
    parser.add_argument("--stub-proximity-cost", type=float, default=0.2,
                        help="Cost penalty near stubs in mm equivalent (default: 0.2)")
    parser.add_argument("--via-proximity-cost", type=float, default=10.0,
                        help="Multiplier on stub-proximity-cost for vias near stubs (0=block, default: 10.0)")

    # BGA proximity penalty
    parser.add_argument("--bga-proximity-radius", type=float, default=10.0,
                        help="Radius around BGA edges to penalize routing in mm (default: 10.0)")
    parser.add_argument("--bga-proximity-cost", type=float, default=0.2,
                        help="Cost penalty near BGA edges in mm equivalent (default: 0.2)")

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
    parser.add_argument("--stub-layer-swap", action="store_true",
                        help="Enable stub layer switching optimization that tries to avoid vias by moving stubs to different layers (experimental)")
    parser.add_argument("--can-swap-to-top-layer", action="store_true",
                        help="Allow swapping stubs to F.Cu (top layer). Off by default due to via clearance issues.")

    # Rip-up and retry options
    parser.add_argument("--max-ripup", type=int, default=3,
                        help="Maximum blockers to rip up at once during rip-up and retry (default: 3)")

    # Debug options
    parser.add_argument("--debug-lines", action="store_true",
                        help="Output debug geometry on User.3 (connectors), User.4 (stub dirs), User.8 (simplified), User.9 (raw A*)")

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
                diff_pair_patterns=args.diff_pairs,
                diff_pair_gap=args.diff_pair_gap,
                diff_pair_centerline_setback=args.diff_pair_centerline_setback,
                min_turning_radius=args.min_turning_radius,
                debug_lines=args.debug_lines,
                fix_polarity=not args.no_fix_polarity,
                max_rip_up_count=args.max_ripup,
                enable_layer_switch=args.stub_layer_swap,
                can_swap_to_top_layer=args.can_swap_to_top_layer,
                vis_callback=vis_callback)

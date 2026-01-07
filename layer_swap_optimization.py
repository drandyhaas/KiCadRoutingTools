"""
Layer swap optimization for PCB routing.

This module handles the upfront layer swap optimization that happens before routing:
- Diff pair source and target layer swaps
- Single-ended source and target layer swaps
- Solo switches and retry loops

The goal is to minimize the number of vias needed by swapping stubs to compatible layers.
"""

from typing import List, Dict, Set, Tuple, Optional
from dataclasses import dataclass

from kicad_parser import PCBData
from routing_config import GridRouteConfig, DiffPairNet
from routing_utils import is_edge_stub
from stub_layer_switching import (
    get_stub_info, apply_stub_layer_switch, collect_stubs_by_layer,
    collect_stub_endpoints_by_layer, validate_swap, validate_single_swap,
    collect_single_ended_stubs_by_layer
)
from diff_pair_routing import get_diff_pair_endpoints
from routing_utils import get_net_endpoints


def _find_blocking_single_ended_nets(
    stub_p, stub_n, dest_layer: str, pcb_data: PCBData, diff_pair_net_ids: Set[int]
) -> List[int]:
    """
    Find single-ended net IDs that block the given stubs from moving to dest_layer.

    Returns list of net IDs that:
    1. Have segments on dest_layer that overlap with stub bounding box
    2. Are NOT part of any diff pair
    """
    our_segments = stub_p.segments + stub_n.segments
    our_net_ids = {stub_p.net_id, stub_n.net_id}
    blocking_nets = set()

    # Use same tolerance as validate_stub_no_overlap
    y_tolerance = 0.2

    for seg in pcb_data.segments:
        if seg.layer != dest_layer:
            continue
        if seg.net_id in our_net_ids:
            continue
        if seg.net_id in diff_pair_net_ids:
            continue

        # Check if segment bounding box overlaps with any of our segments
        for our_seg in our_segments:
            # Segment objects have start_x, start_y, end_x, end_y attributes
            our_x_min = min(our_seg.start_x, our_seg.end_x)
            our_x_max = max(our_seg.start_x, our_seg.end_x)
            our_y_min = min(our_seg.start_y, our_seg.end_y)
            our_y_max = max(our_seg.start_y, our_seg.end_y)

            seg_x_min = min(seg.start_x, seg.end_x)
            seg_x_max = max(seg.start_x, seg.end_x)
            seg_y_min = min(seg.start_y, seg.end_y)
            seg_y_max = max(seg.start_y, seg.end_y)

            # Check if bounding boxes overlap (with y tolerance)
            x_overlap = our_x_min <= seg_x_max and seg_x_min <= our_x_max
            y_overlap = (our_y_min - y_tolerance) <= seg_y_max and seg_y_min <= (our_y_max + y_tolerance)

            if x_overlap and y_overlap:
                blocking_nets.add(seg.net_id)
                break

    return list(blocking_nets)


def _get_single_ended_stub_on_layer(
    pcb_data: PCBData, net_id: int, layer: str, config: GridRouteConfig
):
    """Get stub info for a single-ended net on the given layer."""
    sources, targets, error = get_net_endpoints(pcb_data, net_id, config)
    if error or not sources or not targets:
        return None

    src_layer = config.layers[sources[0][2]]
    tgt_layer = config.layers[targets[0][2]]

    # Check if this net has a stub on the requested layer
    if src_layer == layer:
        return get_stub_info(pcb_data, net_id, sources[0][3], sources[0][4], layer)
    elif tgt_layer == layer:
        return get_stub_info(pcb_data, net_id, targets[0][3], targets[0][4], layer)

    return None


def _validate_single_ended_swap(
    stub, dest_layer: str, pcb_data: PCBData, config: GridRouteConfig,
    exclude_net_ids: Set[int] = None
) -> bool:
    """Validate that a single-ended stub can move to dest_layer without conflicts."""
    from stub_layer_switching import segments_intersect_2d

    if exclude_net_ids is None:
        exclude_net_ids = set()

    # Check for segment intersections on destination layer
    our_segments = stub.segments
    our_net_ids = {stub.net_id} | exclude_net_ids

    for our_seg in our_segments:
        for other in pcb_data.segments:
            if other.layer != dest_layer:
                continue
            if other.net_id in our_net_ids:
                continue
            # Check if segments actually intersect
            if segments_intersect_2d(
                (our_seg.start_x, our_seg.start_y), (our_seg.end_x, our_seg.end_y),
                (other.start_x, other.start_y), (other.end_x, other.end_y)
            ):
                return False

    return True


def _undo_stub_layer_switch(pcb_data: PCBData, mods: List[Dict], vias: List):
    """Undo a stub layer switch by reverting segment layer modifications."""
    # Revert segment layer changes
    for mod in mods:
        seg = mod['segment']
        seg.layer = mod['old_layer']

    # Remove added vias
    for via in vias:
        if via in pcb_data.vias:
            pcb_data.vias.remove(via)


def apply_diff_pair_layer_swaps(
    pcb_data: PCBData,
    config: GridRouteConfig,
    diff_pair_ids_to_route_set: List[Tuple[str, DiffPairNet]],
    diff_pairs: Dict[str, DiffPairNet],
    can_swap_to_top_layer: bool,
    all_segment_modifications: List,
    all_swap_vias: List,
    verbose: bool = False
) -> Tuple[int, Dict, Dict]:
    """
    Apply upfront layer swap optimization for diff pairs.

    Args:
        pcb_data: PCB data structure (modified in place)
        config: Routing configuration
        diff_pair_ids_to_route_set: List of (pair_name, pair) tuples to route
        diff_pairs: Dict of all diff pairs
        can_swap_to_top_layer: Whether stubs can be swapped to F.Cu
        all_segment_modifications: List to append layer modifications (modified in place)
        all_swap_vias: List to append vias from swapping (modified in place)
        verbose: Whether to print verbose output

    Returns:
        (total_layer_swaps, all_stubs_by_layer, stub_endpoints_by_layer)
    """
    print(f"\nAnalyzing layer swaps for {len(diff_pair_ids_to_route_set)} diff pair(s)...")

    # Collect layer info for pairs we're routing
    pair_layer_info = {}  # pair_name -> (src_layer, tgt_layer, sources, targets, pair)
    for pair_name, pair in diff_pair_ids_to_route_set:
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

    # Pre-collect all stub segments by layer for validation
    all_stubs_by_layer = collect_stubs_by_layer(pcb_data, all_pair_layer_info, config)
    # Pre-collect all stub endpoints by layer for proximity checking
    stub_endpoints_by_layer = collect_stub_endpoints_by_layer(pcb_data, all_pair_layer_info, config)

    # Find pairs that need layer switches (src != tgt layer)
    pairs_needing_via = [(name, info) for name, info in pair_layer_info.items()
                        if info[0] != info[1]]

    # Try to find swap partners for pairs needing via
    applied_swaps = set()
    swap_count = 0
    total_layer_swaps = 0

    # Phase 1: Source segment overlap swaps
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

            # IMPORTANT: Don't break a pair that was already OK!
            # After swap, partner's source will be on our src_layer.
            # Partner is OK if: their new source (src_layer) == their target (other_tgt_layer)
            # OR if they already needed a via (can't make it worse)
            partner_already_needs_via = (other_src_layer != other_tgt_layer)
            partner_would_be_ok_after = (src_layer == other_tgt_layer)
            if not partner_already_needs_via and not partner_would_be_ok_after:
                # Partner was OK but swap would break them - skip
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
            _, _, _, _, other_pair = all_pair_layer_info[swap_partner]

            # Validate swap before applying
            our_valid, our_reason = validate_swap(
                src_p_stub, src_n_stub, tgt_layer, all_stubs_by_layer,
                pcb_data, config, swap_partner_name=swap_partner,
                swap_partner_net_ids={other_pair.p_net_id, other_pair.n_net_id},
                stub_endpoints_by_layer=stub_endpoints_by_layer
            )
            partner_valid, partner_reason = validate_swap(
                other_src_p_stub, other_src_n_stub, src_layer, all_stubs_by_layer,
                pcb_data, config, swap_partner_name=pair_name,
                swap_partner_net_ids={pair.p_net_id, pair.n_net_id},
                stub_endpoints_by_layer=stub_endpoints_by_layer
            )

            if not our_valid or not partner_valid:
                reason = our_reason if not our_valid else partner_reason
                print(f"    Source swap validation failed for {pair_name}: {reason}")
                continue  # Try target swap later

            # Check if swap would move stubs to F.Cu (top layer)
            # Skip if can_swap_to_top_layer is False and either destination is F.Cu
            # Exception: allow edge stubs (on BGA boundary) to swap to F.Cu
            if not can_swap_to_top_layer and (tgt_layer == 'F.Cu' or src_layer == 'F.Cu'):
                # Check if stubs moving to F.Cu are edge stubs
                allow_swap = True
                if tgt_layer == 'F.Cu':
                    # Our stubs would move to F.Cu - check if they're edge stubs
                    if not (is_edge_stub(src_p_stub.pad_x, src_p_stub.pad_y, config.bga_exclusion_zones) or
                            is_edge_stub(src_n_stub.pad_x, src_n_stub.pad_y, config.bga_exclusion_zones)):
                        allow_swap = False
                if src_layer == 'F.Cu' and allow_swap:
                    # Their stubs would move to F.Cu - check if they're edge stubs
                    if not (is_edge_stub(other_src_p_stub.pad_x, other_src_p_stub.pad_y, config.bga_exclusion_zones) or
                            is_edge_stub(other_src_n_stub.pad_x, other_src_n_stub.pad_y, config.bga_exclusion_zones)):
                        allow_swap = False
                if not allow_swap:
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

            # Update all_stubs_by_layer to reflect the layer changes
            # pair_name: src_layer -> tgt_layer
            if src_layer in all_stubs_by_layer:
                all_stubs_by_layer[src_layer] = [
                    s for s in all_stubs_by_layer[src_layer] if s[0] != pair_name
                ]
            if tgt_layer not in all_stubs_by_layer:
                all_stubs_by_layer[tgt_layer] = []
            all_stubs_by_layer[tgt_layer].append(
                (pair_name, src_p_stub.segments + src_n_stub.segments)
            )
            # swap_partner: other_src_layer -> src_layer
            if other_src_layer in all_stubs_by_layer:
                all_stubs_by_layer[other_src_layer] = [
                    s for s in all_stubs_by_layer[other_src_layer] if s[0] != swap_partner
                ]
            if src_layer not in all_stubs_by_layer:
                all_stubs_by_layer[src_layer] = []
            all_stubs_by_layer[src_layer].append(
                (swap_partner, other_src_p_stub.segments + other_src_n_stub.segments)
            )

            # Update stub_endpoints_by_layer to reflect the layer changes
            if src_layer in stub_endpoints_by_layer:
                stub_endpoints_by_layer[src_layer] = [
                    e for e in stub_endpoints_by_layer[src_layer] if e[0] != pair_name
                ]
            if tgt_layer not in stub_endpoints_by_layer:
                stub_endpoints_by_layer[tgt_layer] = []
            stub_endpoints_by_layer[tgt_layer].append(
                (pair_name, [(src_p_stub.x, src_p_stub.y), (src_n_stub.x, src_n_stub.y)])
            )
            if other_src_layer in stub_endpoints_by_layer:
                stub_endpoints_by_layer[other_src_layer] = [
                    e for e in stub_endpoints_by_layer[other_src_layer] if e[0] != swap_partner
                ]
            if src_layer not in stub_endpoints_by_layer:
                stub_endpoints_by_layer[src_layer] = []
            stub_endpoints_by_layer[src_layer].append(
                (swap_partner, [(other_src_p_stub.x, other_src_p_stub.y), (other_src_n_stub.x, other_src_n_stub.y)])
            )

            applied_swaps.add(pair_name)
            applied_swaps.add(swap_partner)
            swap_count += 1
            total_layer_swaps += 1
            via_msg = f", added {len(all_vias)} pad via(s)" if all_vias else ""
            print(f"  Source swap: {pair_name} ({src_layer}->{tgt_layer}) <-> {swap_partner} ({other_src_layer}->{src_layer}){via_msg}")

    if swap_count > 0:
        print(f"Applied {swap_count} source layer swap(s)")

    # Phase 2: Solo source layer switches (no partner needed)
    solo_src_count = 0
    for pair_name, (src_layer, tgt_layer, sources, targets, pair) in pairs_needing_via:
        if pair_name in applied_swaps:
            continue

        # Check if we can move source stubs to target layer without a partner
        src_p_stub = get_stub_info(pcb_data, pair.p_net_id,
                                   sources[0][5], sources[0][6], src_layer)
        src_n_stub = get_stub_info(pcb_data, pair.n_net_id,
                                   sources[0][7], sources[0][8], src_layer)

        if not src_p_stub or not src_n_stub:
            continue

        # Check if swap would move stubs to F.Cu (top layer)
        # Exception: allow edge stubs to swap to F.Cu
        if not can_swap_to_top_layer and tgt_layer == 'F.Cu':
            if not (is_edge_stub(src_p_stub.pad_x, src_p_stub.pad_y, config.bga_exclusion_zones) or
                    is_edge_stub(src_n_stub.pad_x, src_n_stub.pad_y, config.bga_exclusion_zones)):
                continue

        # Validate solo switch: source stubs move to target layer
        valid, reason = validate_swap(
            src_p_stub, src_n_stub, tgt_layer, all_stubs_by_layer,
            pcb_data, config, swap_partner_name=None,
            swap_partner_net_ids=set(),
            stub_endpoints_by_layer=stub_endpoints_by_layer
        )

        if valid:
            # Apply solo source switch
            vias1, mods1 = apply_stub_layer_switch(pcb_data, src_p_stub, tgt_layer, config, debug=False)
            vias2, mods2 = apply_stub_layer_switch(pcb_data, src_n_stub, tgt_layer, config, debug=False)
            all_segment_modifications.extend(mods1 + mods2)
            all_vias = vias1 + vias2
            all_swap_vias.extend(all_vias)

            # Update all_stubs_by_layer to reflect the layer change
            # Structure is (pair_name, segments) tuples
            # Remove from old layer and add to new layer
            if src_layer in all_stubs_by_layer:
                all_stubs_by_layer[src_layer] = [
                    s for s in all_stubs_by_layer[src_layer]
                    if s[0] != pair_name  # s[0] is pair_name
                ]
            if tgt_layer not in all_stubs_by_layer:
                all_stubs_by_layer[tgt_layer] = []
            # Add combined segments for this pair on new layer
            combined_segments = src_p_stub.segments + src_n_stub.segments
            all_stubs_by_layer[tgt_layer].append((pair_name, combined_segments))

            # Update stub_endpoints_by_layer
            if src_layer in stub_endpoints_by_layer:
                stub_endpoints_by_layer[src_layer] = [
                    e for e in stub_endpoints_by_layer[src_layer] if e[0] != pair_name
                ]
            if tgt_layer not in stub_endpoints_by_layer:
                stub_endpoints_by_layer[tgt_layer] = []
            stub_endpoints_by_layer[tgt_layer].append(
                (pair_name, [(src_p_stub.x, src_p_stub.y), (src_n_stub.x, src_n_stub.y)])
            )

            applied_swaps.add(pair_name)
            solo_src_count += 1
            total_layer_swaps += 1
            via_msg = f", added {len(all_vias)} pad via(s)" if all_vias else ""
            print(f"  Solo source switch: {pair_name} ({src_layer}->{tgt_layer}){via_msg}")
        else:
            print(f"    Solo source switch validation failed for {pair_name}: {reason}")

    if solo_src_count > 0:
        print(f"Applied {solo_src_count} solo source layer switch(es)")

    # Phase 3: Target-side segment overlap swaps
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

            # IMPORTANT: Don't break a pair that was already OK!
            # After swap, partner's target will be on our tgt_layer.
            # Partner is OK if: their source (other_src_layer) == their new target (tgt_layer)
            # OR if they already needed a via (can't make it worse)
            partner_already_needs_via = (other_src_layer != other_tgt_layer)
            partner_would_be_ok_after = (other_src_layer == tgt_layer)
            if not partner_already_needs_via and not partner_would_be_ok_after:
                # Partner was OK but swap would break them - skip
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
            _, _, _, _, other_pair = all_pair_layer_info[swap_partner]

            # Validate swap before applying
            our_valid, our_reason = validate_swap(
                tgt_p_stub, tgt_n_stub, src_layer, all_stubs_by_layer,
                pcb_data, config, swap_partner_name=swap_partner,
                swap_partner_net_ids={other_pair.p_net_id, other_pair.n_net_id},
                stub_endpoints_by_layer=stub_endpoints_by_layer
            )
            partner_valid, partner_reason = validate_swap(
                other_tgt_p_stub, other_tgt_n_stub, tgt_layer, all_stubs_by_layer,
                pcb_data, config, swap_partner_name=pair_name,
                swap_partner_net_ids={pair.p_net_id, pair.n_net_id},
                stub_endpoints_by_layer=stub_endpoints_by_layer
            )

            if not our_valid or not partner_valid:
                reason = our_reason if not our_valid else partner_reason
                print(f"    Target swap validation failed for {pair_name}: {reason}")
                continue

            # Check if swap would move stubs to F.Cu (top layer)
            # Skip if can_swap_to_top_layer is False and either destination is F.Cu
            # Exception: allow edge stubs to swap to F.Cu
            if not can_swap_to_top_layer and (src_layer == 'F.Cu' or tgt_layer == 'F.Cu'):
                # Check if stubs moving to F.Cu are edge stubs
                allow_swap = True
                if src_layer == 'F.Cu':
                    # Our stubs would move to F.Cu - check if they're edge stubs
                    if not (is_edge_stub(tgt_p_stub.pad_x, tgt_p_stub.pad_y, config.bga_exclusion_zones) or
                            is_edge_stub(tgt_n_stub.pad_x, tgt_n_stub.pad_y, config.bga_exclusion_zones)):
                        allow_swap = False
                if tgt_layer == 'F.Cu' and allow_swap:
                    # Their stubs would move to F.Cu - check if they're edge stubs
                    if not (is_edge_stub(other_tgt_p_stub.pad_x, other_tgt_p_stub.pad_y, config.bga_exclusion_zones) or
                            is_edge_stub(other_tgt_n_stub.pad_x, other_tgt_n_stub.pad_y, config.bga_exclusion_zones)):
                        allow_swap = False
                if not allow_swap:
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

            # Update stub_endpoints_by_layer for both pairs
            # Our targets move from tgt_layer to src_layer
            if tgt_layer in stub_endpoints_by_layer:
                stub_endpoints_by_layer[tgt_layer] = [
                    e for e in stub_endpoints_by_layer[tgt_layer] if e[0] != pair_name
                ]
            if src_layer not in stub_endpoints_by_layer:
                stub_endpoints_by_layer[src_layer] = []
            stub_endpoints_by_layer[src_layer].append(
                (pair_name, [(tgt_p_stub.x, tgt_p_stub.y), (tgt_n_stub.x, tgt_n_stub.y)])
            )
            # Their targets move from other_tgt_layer to tgt_layer
            if other_tgt_layer in stub_endpoints_by_layer:
                stub_endpoints_by_layer[other_tgt_layer] = [
                    e for e in stub_endpoints_by_layer[other_tgt_layer] if e[0] != swap_partner
                ]
            if tgt_layer not in stub_endpoints_by_layer:
                stub_endpoints_by_layer[tgt_layer] = []
            stub_endpoints_by_layer[tgt_layer].append(
                (swap_partner, [(other_tgt_p_stub.x, other_tgt_p_stub.y), (other_tgt_n_stub.x, other_tgt_n_stub.y)])
            )

            applied_swaps.add(pair_name)
            applied_swaps.add(swap_partner)
            target_swap_count += 1
            total_layer_swaps += 1
            via_msg = f", added {len(all_vias)} pad via(s)" if all_vias else ""
            print(f"  Target swap: {pair_name} ({tgt_layer}->{src_layer}) <-> {swap_partner} ({other_tgt_layer}->{tgt_layer}){via_msg}")

    if target_swap_count > 0:
        print(f"Applied {target_swap_count} target layer swap(s)")

    # Phase 4: Solo target layer switches (no partner needed)
    solo_switch_count = 0
    for pair_name, (src_layer, tgt_layer, sources, targets, pair) in pairs_needing_via:
        if pair_name in applied_swaps:
            continue

        # Check if we can move target stubs to source layer without a partner
        tgt_p_stub = get_stub_info(pcb_data, pair.p_net_id,
                                   targets[0][5], targets[0][6], tgt_layer)
        tgt_n_stub = get_stub_info(pcb_data, pair.n_net_id,
                                   targets[0][7], targets[0][8], tgt_layer)

        if not tgt_p_stub or not tgt_n_stub:
            missing = []
            if not tgt_p_stub:
                missing.append("P")
            if not tgt_n_stub:
                missing.append("N")
            print(f"    Solo target switch skipped for {pair_name}: can't find {'+'.join(missing)} stub at target ({targets[0][5]:.2f}, {targets[0][6]:.2f}) on {tgt_layer}")
            continue

        # Check if swap would move stubs to F.Cu (top layer)
        # Exception: allow edge stubs to swap to F.Cu
        if not can_swap_to_top_layer and src_layer == 'F.Cu':
            if not (is_edge_stub(tgt_p_stub.pad_x, tgt_p_stub.pad_y, config.bga_exclusion_zones) or
                    is_edge_stub(tgt_n_stub.pad_x, tgt_n_stub.pad_y, config.bga_exclusion_zones)):
                print(f"    Solo target switch skipped for {pair_name}: would move to F.Cu (top layer)")
                continue

        # Validate solo switch: target stubs move to source layer
        valid, reason = validate_swap(
            tgt_p_stub, tgt_n_stub, src_layer, all_stubs_by_layer,
            pcb_data, config, swap_partner_name=None,
            swap_partner_net_ids=set(),
            stub_endpoints_by_layer=stub_endpoints_by_layer
        )

        if valid:
            # Apply solo target switch
            vias1, mods1 = apply_stub_layer_switch(pcb_data, tgt_p_stub, src_layer, config, debug=False)
            vias2, mods2 = apply_stub_layer_switch(pcb_data, tgt_n_stub, src_layer, config, debug=False)
            all_segment_modifications.extend(mods1 + mods2)
            all_vias = vias1 + vias2
            all_swap_vias.extend(all_vias)

            # Update all_stubs_by_layer to reflect the layer change
            # Structure is (pair_name, segments) tuples
            if tgt_layer in all_stubs_by_layer:
                all_stubs_by_layer[tgt_layer] = [
                    s for s in all_stubs_by_layer[tgt_layer]
                    if s[0] != pair_name
                ]
            if src_layer not in all_stubs_by_layer:
                all_stubs_by_layer[src_layer] = []
            combined_segments = tgt_p_stub.segments + tgt_n_stub.segments
            all_stubs_by_layer[src_layer].append((pair_name, combined_segments))

            # Update stub_endpoints_by_layer
            if tgt_layer in stub_endpoints_by_layer:
                stub_endpoints_by_layer[tgt_layer] = [
                    e for e in stub_endpoints_by_layer[tgt_layer] if e[0] != pair_name
                ]
            if src_layer not in stub_endpoints_by_layer:
                stub_endpoints_by_layer[src_layer] = []
            stub_endpoints_by_layer[src_layer].append(
                (pair_name, [(tgt_p_stub.x, tgt_p_stub.y), (tgt_n_stub.x, tgt_n_stub.y)])
            )

            applied_swaps.add(pair_name)
            solo_switch_count += 1
            total_layer_swaps += 1
            via_msg = f", added {len(all_vias)} pad via(s)" if all_vias else ""
            print(f"  Solo target switch: {pair_name} ({tgt_layer}->{src_layer}){via_msg}")
        else:
            print(f"    Solo target switch validation failed for {pair_name}: {reason}")

    if solo_switch_count > 0:
        print(f"Applied {solo_switch_count} solo target layer switch(es)")

    # Phase 5: Retry solo switches if any progress was made (newly freed layers may allow more switches)
    if solo_src_count > 0 or solo_switch_count > 0:
        retry_round = 1
        while True:
            retry_round += 1
            retry_src_count = 0
            retry_tgt_count = 0

            # Retry solo source switches
            for pair_name, (src_layer, tgt_layer, sources, targets, pair) in pairs_needing_via:
                if pair_name in applied_swaps:
                    continue
                src_p_stub = get_stub_info(pcb_data, pair.p_net_id,
                                           sources[0][5], sources[0][6], src_layer)
                src_n_stub = get_stub_info(pcb_data, pair.n_net_id,
                                           sources[0][7], sources[0][8], src_layer)
                if not src_p_stub or not src_n_stub:
                    continue
                if not can_swap_to_top_layer and tgt_layer == 'F.Cu':
                    if not (is_edge_stub(src_p_stub.pad_x, src_p_stub.pad_y, config.bga_exclusion_zones) or
                            is_edge_stub(src_n_stub.pad_x, src_n_stub.pad_y, config.bga_exclusion_zones)):
                        continue
                valid, reason = validate_swap(
                    src_p_stub, src_n_stub, tgt_layer, all_stubs_by_layer,
                    pcb_data, config, swap_partner_name=None,
                    swap_partner_net_ids=set(),
                    stub_endpoints_by_layer=stub_endpoints_by_layer
                )
                if valid:
                    vias1, mods1 = apply_stub_layer_switch(pcb_data, src_p_stub, tgt_layer, config, debug=False)
                    vias2, mods2 = apply_stub_layer_switch(pcb_data, src_n_stub, tgt_layer, config, debug=False)
                    all_segment_modifications.extend(mods1 + mods2)
                    all_vias = vias1 + vias2
                    all_swap_vias.extend(all_vias)
                    if src_layer in all_stubs_by_layer:
                        all_stubs_by_layer[src_layer] = [s for s in all_stubs_by_layer[src_layer] if s[0] != pair_name]
                    if tgt_layer not in all_stubs_by_layer:
                        all_stubs_by_layer[tgt_layer] = []
                    all_stubs_by_layer[tgt_layer].append((pair_name, src_p_stub.segments + src_n_stub.segments))
                    if src_layer in stub_endpoints_by_layer:
                        stub_endpoints_by_layer[src_layer] = [e for e in stub_endpoints_by_layer[src_layer] if e[0] != pair_name]
                    if tgt_layer not in stub_endpoints_by_layer:
                        stub_endpoints_by_layer[tgt_layer] = []
                    stub_endpoints_by_layer[tgt_layer].append((pair_name, [(src_p_stub.x, src_p_stub.y), (src_n_stub.x, src_n_stub.y)]))
                    applied_swaps.add(pair_name)
                    retry_src_count += 1
                    total_layer_swaps += 1
                    via_msg = f", added {len(all_vias)} pad via(s)" if all_vias else ""
                    print(f"  Solo source switch (round {retry_round}): {pair_name} ({src_layer}->{tgt_layer}){via_msg}")

            # Retry solo target switches
            for pair_name, (src_layer, tgt_layer, sources, targets, pair) in pairs_needing_via:
                if pair_name in applied_swaps:
                    continue
                tgt_p_stub = get_stub_info(pcb_data, pair.p_net_id,
                                           targets[0][5], targets[0][6], tgt_layer)
                tgt_n_stub = get_stub_info(pcb_data, pair.n_net_id,
                                           targets[0][7], targets[0][8], tgt_layer)
                if not tgt_p_stub or not tgt_n_stub:
                    continue
                if not can_swap_to_top_layer and src_layer == 'F.Cu':
                    if not (is_edge_stub(tgt_p_stub.pad_x, tgt_p_stub.pad_y, config.bga_exclusion_zones) or
                            is_edge_stub(tgt_n_stub.pad_x, tgt_n_stub.pad_y, config.bga_exclusion_zones)):
                        continue
                valid, reason = validate_swap(
                    tgt_p_stub, tgt_n_stub, src_layer, all_stubs_by_layer,
                    pcb_data, config, swap_partner_name=None,
                    swap_partner_net_ids=set(),
                    stub_endpoints_by_layer=stub_endpoints_by_layer
                )
                if valid:
                    vias1, mods1 = apply_stub_layer_switch(pcb_data, tgt_p_stub, src_layer, config, debug=False)
                    vias2, mods2 = apply_stub_layer_switch(pcb_data, tgt_n_stub, src_layer, config, debug=False)
                    all_segment_modifications.extend(mods1 + mods2)
                    all_vias = vias1 + vias2
                    all_swap_vias.extend(all_vias)
                    if tgt_layer in all_stubs_by_layer:
                        all_stubs_by_layer[tgt_layer] = [s for s in all_stubs_by_layer[tgt_layer] if s[0] != pair_name]
                    if src_layer not in all_stubs_by_layer:
                        all_stubs_by_layer[src_layer] = []
                    all_stubs_by_layer[src_layer].append((pair_name, tgt_p_stub.segments + tgt_n_stub.segments))
                    if tgt_layer in stub_endpoints_by_layer:
                        stub_endpoints_by_layer[tgt_layer] = [e for e in stub_endpoints_by_layer[tgt_layer] if e[0] != pair_name]
                    if src_layer not in stub_endpoints_by_layer:
                        stub_endpoints_by_layer[src_layer] = []
                    stub_endpoints_by_layer[src_layer].append((pair_name, [(tgt_p_stub.x, tgt_p_stub.y), (tgt_n_stub.x, tgt_n_stub.y)]))
                    applied_swaps.add(pair_name)
                    retry_tgt_count += 1
                    total_layer_swaps += 1
                    via_msg = f", added {len(all_vias)} pad via(s)" if all_vias else ""
                    print(f"  Solo target switch (round {retry_round}): {pair_name} ({tgt_layer}->{src_layer}){via_msg}")

            if retry_src_count == 0 and retry_tgt_count == 0:
                break  # No more progress
            print(f"Applied {retry_src_count + retry_tgt_count} additional solo switch(es) in round {retry_round}")

    # Phase 6: Two-pair swaps for remaining pairs that weren't handled
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
                # Exception: allow edge stubs to swap to F.Cu
                allow_swap = True
                if not can_swap_to_top_layer and (our_new_layer == 'F.Cu' or their_new_layer == 'F.Cu'):
                    if our_new_layer == 'F.Cu':
                        if not (is_edge_stub(our_stubs[0].pad_x, our_stubs[0].pad_y, config.bga_exclusion_zones) or
                                is_edge_stub(our_stubs[1].pad_x, our_stubs[1].pad_y, config.bga_exclusion_zones)):
                            allow_swap = False
                    if their_new_layer == 'F.Cu' and allow_swap:
                        if not (is_edge_stub(their_stubs[0].pad_x, their_stubs[0].pad_y, config.bga_exclusion_zones) or
                                is_edge_stub(their_stubs[1].pad_x, their_stubs[1].pad_y, config.bga_exclusion_zones)):
                            allow_swap = False
                if not allow_swap:
                    pass  # Skip this swap - would move non-edge stubs to top layer
                else:
                    # Validate swap before applying
                    our_valid, our_reason = validate_swap(
                        our_stubs[0], our_stubs[1], our_new_layer, all_stubs_by_layer,
                        pcb_data, config, swap_partner_name=other_name,
                        swap_partner_net_ids={other_pair.p_net_id, other_pair.n_net_id},
                        stub_endpoints_by_layer=stub_endpoints_by_layer
                    )
                    their_valid, their_reason = validate_swap(
                        their_stubs[0], their_stubs[1], their_new_layer, all_stubs_by_layer,
                        pcb_data, config, swap_partner_name=pair_name,
                        swap_partner_net_ids={pair.p_net_id, pair.n_net_id},
                        stub_endpoints_by_layer=stub_endpoints_by_layer
                    )

                    if not our_valid or not their_valid:
                        reason = our_reason if not our_valid else their_reason
                        print(f"    Two-pair {swap_type} swap validation failed for {pair_name}: {reason}")
                        # Don't break - continue to try fallback target swap
                    else:
                        # Apply swaps
                        _, mods1 = apply_stub_layer_switch(pcb_data, our_stubs[0], our_new_layer, config, debug=False)
                        _, mods2 = apply_stub_layer_switch(pcb_data, our_stubs[1], our_new_layer, config, debug=False)
                        _, mods3 = apply_stub_layer_switch(pcb_data, their_stubs[0], their_new_layer, config, debug=False)
                        _, mods4 = apply_stub_layer_switch(pcb_data, their_stubs[1], their_new_layer, config, debug=False)
                        all_segment_modifications.extend(mods1 + mods2 + mods3 + mods4)

                        # Update stub_endpoints_by_layer for both pairs
                        # Our pair moves to our_new_layer
                        our_orig_layer = src_layer if swap_type == "source" else tgt_layer
                        if our_orig_layer in stub_endpoints_by_layer:
                            stub_endpoints_by_layer[our_orig_layer] = [
                                e for e in stub_endpoints_by_layer[our_orig_layer] if e[0] != pair_name
                            ]
                        if our_new_layer not in stub_endpoints_by_layer:
                            stub_endpoints_by_layer[our_new_layer] = []
                        stub_endpoints_by_layer[our_new_layer].append(
                            (pair_name, [(our_stubs[0].x, our_stubs[0].y), (our_stubs[1].x, our_stubs[1].y)])
                        )
                        # Their pair moves to their_new_layer
                        their_orig_layer = other_src if swap_type == "source" else other_tgt
                        if their_orig_layer in stub_endpoints_by_layer:
                            stub_endpoints_by_layer[their_orig_layer] = [
                                e for e in stub_endpoints_by_layer[their_orig_layer] if e[0] != other_name
                            ]
                        if their_new_layer not in stub_endpoints_by_layer:
                            stub_endpoints_by_layer[their_new_layer] = []
                        stub_endpoints_by_layer[their_new_layer].append(
                            (other_name, [(their_stubs[0].x, their_stubs[0].y), (their_stubs[1].x, their_stubs[1].y)])
                        )

                        applied_swaps.add(pair_name)
                        applied_swaps.add(other_name)
                        swap_count += 1
                        total_layer_swaps += 1
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
                        # Exception: allow edge stubs to swap to F.Cu
                        allow_swap = True
                        if not can_swap_to_top_layer and (src_layer == 'F.Cu' or tgt_layer == 'F.Cu'):
                            if src_layer == 'F.Cu':
                                if not (is_edge_stub(p_stub.pad_x, p_stub.pad_y, config.bga_exclusion_zones) or
                                        is_edge_stub(n_stub.pad_x, n_stub.pad_y, config.bga_exclusion_zones)):
                                    allow_swap = False
                            if tgt_layer == 'F.Cu' and allow_swap:
                                if not (is_edge_stub(other_p_stub.pad_x, other_p_stub.pad_y, config.bga_exclusion_zones) or
                                        is_edge_stub(other_n_stub.pad_x, other_n_stub.pad_y, config.bga_exclusion_zones)):
                                    allow_swap = False
                        if not allow_swap:
                            pass  # Skip this swap - would move non-edge stubs to top layer
                        else:
                            # Validate fallback target swap before applying
                            our_valid, our_reason = validate_swap(
                                p_stub, n_stub, src_layer, all_stubs_by_layer,
                                pcb_data, config, swap_partner_name=other_name,
                                swap_partner_net_ids={other_pair.p_net_id, other_pair.n_net_id},
                                stub_endpoints_by_layer=stub_endpoints_by_layer
                            )
                            their_valid, their_reason = validate_swap(
                                other_p_stub, other_n_stub, tgt_layer, all_stubs_by_layer,
                                pcb_data, config, swap_partner_name=pair_name,
                                swap_partner_net_ids={pair.p_net_id, pair.n_net_id},
                                stub_endpoints_by_layer=stub_endpoints_by_layer
                            )

                            if not our_valid or not their_valid:
                                reason = our_reason if not our_valid else their_reason
                                print(f"    Fallback target swap validation failed for {pair_name}: {reason}")
                            else:
                                _, mods1 = apply_stub_layer_switch(pcb_data, p_stub, src_layer, config, debug=False)
                                _, mods2 = apply_stub_layer_switch(pcb_data, n_stub, src_layer, config, debug=False)
                                _, mods3 = apply_stub_layer_switch(pcb_data, other_p_stub, tgt_layer, config, debug=False)
                                _, mods4 = apply_stub_layer_switch(pcb_data, other_n_stub, tgt_layer, config, debug=False)
                                all_segment_modifications.extend(mods1 + mods2 + mods3 + mods4)

                                # Update stub_endpoints_by_layer for both pairs
                                # Our targets move from tgt_layer to src_layer
                                if tgt_layer in stub_endpoints_by_layer:
                                    stub_endpoints_by_layer[tgt_layer] = [
                                        e for e in stub_endpoints_by_layer[tgt_layer] if e[0] != pair_name
                                    ]
                                if src_layer not in stub_endpoints_by_layer:
                                    stub_endpoints_by_layer[src_layer] = []
                                stub_endpoints_by_layer[src_layer].append(
                                    (pair_name, [(p_stub.x, p_stub.y), (n_stub.x, n_stub.y)])
                                )
                                # Their targets move from other_tgt to tgt_layer
                                if other_tgt in stub_endpoints_by_layer:
                                    stub_endpoints_by_layer[other_tgt] = [
                                        e for e in stub_endpoints_by_layer[other_tgt] if e[0] != other_name
                                    ]
                                if tgt_layer not in stub_endpoints_by_layer:
                                    stub_endpoints_by_layer[tgt_layer] = []
                                stub_endpoints_by_layer[tgt_layer].append(
                                    (other_name, [(other_p_stub.x, other_p_stub.y), (other_n_stub.x, other_n_stub.y)])
                                )

                                applied_swaps.add(pair_name)
                                applied_swaps.add(other_name)
                                swap_count += 1
                                total_layer_swaps += 1
                                print(f"  Swap targets: {pair_name} <-> {other_name}")
                                break

    if swap_count > 0:
        print(f"Applied {swap_count} layer swap(s)")

    # Phase 7: Try single-ended swaps to make room for diff pairs that still need vias
    # Build set of all diff pair net IDs for exclusion
    diff_pair_net_ids = set()
    for pair_name, pair in diff_pairs.items():
        diff_pair_net_ids.add(pair.p_net_id)
        diff_pair_net_ids.add(pair.n_net_id)

    single_ended_swap_count = 0
    for pair_name, (src_layer, tgt_layer, sources, targets, pair) in pairs_needing_via:
        if pair_name in applied_swaps:
            continue

        # Try source-side swap: move source stubs to target layer
        src_p_stub = get_stub_info(pcb_data, pair.p_net_id,
                                   sources[0][5], sources[0][6], src_layer)
        src_n_stub = get_stub_info(pcb_data, pair.n_net_id,
                                   sources[0][7], sources[0][8], src_layer)

        if src_p_stub and src_n_stub:
            # Find blocking single-ended nets on target layer
            blocking_nets = _find_blocking_single_ended_nets(
                src_p_stub, src_n_stub, tgt_layer, pcb_data, diff_pair_net_ids
            )

            for blocking_net_id in blocking_nets:
                # Try to find an alternative layer for this single-ended net
                se_stub = _get_single_ended_stub_on_layer(
                    pcb_data, blocking_net_id, tgt_layer, config
                )
                if not se_stub:
                    continue

                # Try each available layer except source and target
                for alt_layer in config.layers:
                    if alt_layer == tgt_layer or alt_layer == src_layer:
                        continue
                    # Don't swap to F.Cu unless it's an edge stub
                    if not can_swap_to_top_layer and alt_layer == 'F.Cu':
                        if not is_edge_stub(se_stub.pad_x, se_stub.pad_y, config.bga_exclusion_zones):
                            continue

                    # Check if single-ended stub can move to alt_layer without conflicts
                    se_valid = _validate_single_ended_swap(
                        se_stub, alt_layer, pcb_data, config, {pair.p_net_id, pair.n_net_id}
                    )
                    if not se_valid:
                        continue

                    # Apply the single-ended swap
                    se_vias, se_mods = apply_stub_layer_switch(
                        pcb_data, se_stub, alt_layer, config, debug=False
                    )
                    all_segment_modifications.extend(se_mods)
                    all_swap_vias.extend(se_vias)

                    # Now retry the diff pair source switch
                    valid, reason = validate_swap(
                        src_p_stub, src_n_stub, tgt_layer, all_stubs_by_layer,
                        pcb_data, config, swap_partner_name=None,
                        swap_partner_net_ids=set(),
                        stub_endpoints_by_layer=stub_endpoints_by_layer
                    )

                    if valid:
                        # Apply diff pair source switch
                        vias1, mods1 = apply_stub_layer_switch(pcb_data, src_p_stub, tgt_layer, config, debug=False)
                        vias2, mods2 = apply_stub_layer_switch(pcb_data, src_n_stub, tgt_layer, config, debug=False)
                        all_segment_modifications.extend(mods1 + mods2)
                        all_swap_vias.extend(vias1 + vias2)

                        # Update tracking structures
                        if src_layer in all_stubs_by_layer:
                            all_stubs_by_layer[src_layer] = [s for s in all_stubs_by_layer[src_layer] if s[0] != pair_name]
                        if tgt_layer not in all_stubs_by_layer:
                            all_stubs_by_layer[tgt_layer] = []
                        all_stubs_by_layer[tgt_layer].append((pair_name, src_p_stub.segments + src_n_stub.segments))
                        if src_layer in stub_endpoints_by_layer:
                            stub_endpoints_by_layer[src_layer] = [e for e in stub_endpoints_by_layer[src_layer] if e[0] != pair_name]
                        if tgt_layer not in stub_endpoints_by_layer:
                            stub_endpoints_by_layer[tgt_layer] = []
                        stub_endpoints_by_layer[tgt_layer].append((pair_name, [(src_p_stub.x, src_p_stub.y), (src_n_stub.x, src_n_stub.y)]))

                        applied_swaps.add(pair_name)
                        single_ended_swap_count += 1
                        total_layer_swaps += 1

                        net = pcb_data.nets.get(blocking_net_id)
                        blocking_name = net.name if net else f"net {blocking_net_id}"
                        via_msg = f", added {len(vias1) + len(vias2)} pad via(s)" if vias1 or vias2 else ""
                        print(f"  Solo source switch: {pair_name} ({src_layer}->{tgt_layer}) after moving {blocking_name} to {alt_layer}{via_msg}")
                        break
                    else:
                        # Undo the single-ended swap since diff pair still can't swap
                        _undo_stub_layer_switch(pcb_data, se_mods, se_vias)

                if pair_name in applied_swaps:
                    break

        # If source side didn't work, try target side
        if pair_name not in applied_swaps:
            tgt_p_stub = get_stub_info(pcb_data, pair.p_net_id,
                                       targets[0][5], targets[0][6], tgt_layer)
            tgt_n_stub = get_stub_info(pcb_data, pair.n_net_id,
                                       targets[0][7], targets[0][8], tgt_layer)

            if tgt_p_stub and tgt_n_stub:
                # Find blocking single-ended nets on source layer
                blocking_nets = _find_blocking_single_ended_nets(
                    tgt_p_stub, tgt_n_stub, src_layer, pcb_data, diff_pair_net_ids
                )

                for blocking_net_id in blocking_nets:
                    se_stub = _get_single_ended_stub_on_layer(
                        pcb_data, blocking_net_id, src_layer, config
                    )
                    if not se_stub:
                        continue

                    for alt_layer in config.layers:
                        if alt_layer == src_layer or alt_layer == tgt_layer:
                            continue
                        if not can_swap_to_top_layer and alt_layer == 'F.Cu':
                            if not is_edge_stub(se_stub.pad_x, se_stub.pad_y, config.bga_exclusion_zones):
                                continue

                        se_valid = _validate_single_ended_swap(
                            se_stub, alt_layer, pcb_data, config, {pair.p_net_id, pair.n_net_id}
                        )
                        if not se_valid:
                            continue

                        se_vias, se_mods = apply_stub_layer_switch(
                            pcb_data, se_stub, alt_layer, config, debug=False
                        )
                        all_segment_modifications.extend(se_mods)
                        all_swap_vias.extend(se_vias)

                        valid, reason = validate_swap(
                            tgt_p_stub, tgt_n_stub, src_layer, all_stubs_by_layer,
                            pcb_data, config, swap_partner_name=None,
                            swap_partner_net_ids=set(),
                            stub_endpoints_by_layer=stub_endpoints_by_layer
                        )

                        if valid:
                            vias1, mods1 = apply_stub_layer_switch(pcb_data, tgt_p_stub, src_layer, config, debug=False)
                            vias2, mods2 = apply_stub_layer_switch(pcb_data, tgt_n_stub, src_layer, config, debug=False)
                            all_segment_modifications.extend(mods1 + mods2)
                            all_swap_vias.extend(vias1 + vias2)

                            if tgt_layer in all_stubs_by_layer:
                                all_stubs_by_layer[tgt_layer] = [s for s in all_stubs_by_layer[tgt_layer] if s[0] != pair_name]
                            if src_layer not in all_stubs_by_layer:
                                all_stubs_by_layer[src_layer] = []
                            all_stubs_by_layer[src_layer].append((pair_name, tgt_p_stub.segments + tgt_n_stub.segments))
                            if tgt_layer in stub_endpoints_by_layer:
                                stub_endpoints_by_layer[tgt_layer] = [e for e in stub_endpoints_by_layer[tgt_layer] if e[0] != pair_name]
                            if src_layer not in stub_endpoints_by_layer:
                                stub_endpoints_by_layer[src_layer] = []
                            stub_endpoints_by_layer[src_layer].append((pair_name, [(tgt_p_stub.x, tgt_p_stub.y), (tgt_n_stub.x, tgt_n_stub.y)]))

                            applied_swaps.add(pair_name)
                            single_ended_swap_count += 1
                            total_layer_swaps += 1

                            net = pcb_data.nets.get(blocking_net_id)
                            blocking_name = net.name if net else f"net {blocking_net_id}"
                            via_msg = f", added {len(vias1) + len(vias2)} pad via(s)" if vias1 or vias2 else ""
                            print(f"  Solo target switch: {pair_name} ({tgt_layer}->{src_layer}) after moving {blocking_name} to {alt_layer}{via_msg}")
                            break
                        else:
                            _undo_stub_layer_switch(pcb_data, se_mods, se_vias)

                    if pair_name in applied_swaps:
                        break

    if single_ended_swap_count > 0:
        print(f"Applied {single_ended_swap_count} single-ended swap(s) to enable diff pair switches")

    # Report pairs that need vias but couldn't be swapped
    for pair_name, (src_layer, tgt_layer, sources, targets, pair) in pairs_needing_via:
        if pair_name not in applied_swaps:
            print(f"  No swap found: {pair_name} ({src_layer}->{tgt_layer}) - will need via")

    return total_layer_swaps, all_stubs_by_layer, stub_endpoints_by_layer


def apply_single_ended_layer_swaps(
    pcb_data: PCBData,
    config: GridRouteConfig,
    single_ended_net_ids: List[Tuple[str, int]],
    can_swap_to_top_layer: bool,
    all_segment_modifications: List,
    all_swap_vias: List,
    all_stubs_by_layer: Optional[Dict] = None,
    verbose: bool = False
) -> int:
    """
    Apply upfront layer swap optimization for single-ended nets.

    Args:
        pcb_data: PCB data structure (modified in place)
        config: Routing configuration
        single_ended_net_ids: List of (net_name, net_id) tuples to route
        can_swap_to_top_layer: Whether stubs can be swapped to F.Cu
        all_segment_modifications: List to append layer modifications (modified in place)
        all_swap_vias: List to append vias from swapping (modified in place)
        all_stubs_by_layer: Optional dict from diff pair layer swaps for validation
        verbose: Whether to print verbose output

    Returns:
        total_layer_swaps: Number of layer swaps applied
    """
    # Collect layer info for single-ended nets
    single_net_layer_info = {}  # net_name -> (src_layer, tgt_layer, sources, targets, net_id)
    for net_name, net_id in single_ended_net_ids:
        sources, targets, error = get_net_endpoints(pcb_data, net_id, config)
        if error or not sources or not targets:
            continue
        src_layer = config.layers[sources[0][2]]
        tgt_layer = config.layers[targets[0][2]]
        if src_layer != tgt_layer:  # Only track nets needing via
            single_net_layer_info[net_name] = (src_layer, tgt_layer, sources, targets, net_id)

    if not single_net_layer_info:
        return 0

    print(f"\nAnalyzing layer swaps for {len(single_net_layer_info)} single-ended net(s) needing via...")

    # Pre-collect single-ended stubs by layer
    single_stubs_by_layer = collect_single_ended_stubs_by_layer(pcb_data, single_net_layer_info, config)

    # Combine with diff pair stubs if they exist
    if all_stubs_by_layer:
        combined_stubs_by_layer = {layer: list(stubs) for layer, stubs in all_stubs_by_layer.items()}
    else:
        combined_stubs_by_layer = {}
    for layer, stubs in single_stubs_by_layer.items():
        combined_stubs_by_layer.setdefault(layer, []).extend(stubs)

    applied_single_swaps = set()
    swap_pair_count = 0
    solo_switch_count = 0
    total_layer_swaps = 0

    # PHASE 1: Try swap pairs first
    # For swap pairs (Net1: src=A,tgt=B and Net2: src=B,tgt=A), we can:
    # Option 1: Both move to layer B (Net1 src A→B, Net2 tgt A→B)
    # Option 2: Both move to layer A (Net1 tgt B→A, Net2 src B→A)
    for net1_name, (src1, tgt1, sources1, targets1, net1_id) in single_net_layer_info.items():
        if net1_name in applied_single_swaps:
            continue
        for net2_name, (src2, tgt2, sources2, targets2, net2_id) in single_net_layer_info.items():
            if net2_name in applied_single_swaps or net1_name == net2_name:
                continue
            # Check if they can help each other: src1==tgt2 and tgt1==src2
            if src1 == tgt2 and tgt1 == src2:
                # Get stubs for both source AND target endpoints
                src1_stub = get_stub_info(pcb_data, net1_id, sources1[0][3], sources1[0][4], src1)
                tgt1_stub = get_stub_info(pcb_data, net1_id, targets1[0][3], targets1[0][4], tgt1)
                src2_stub = get_stub_info(pcb_data, net2_id, sources2[0][3], sources2[0][4], src2)
                tgt2_stub = get_stub_info(pcb_data, net2_id, targets2[0][3], targets2[0][4], tgt2)

                # Try different combinations to find one that works
                # Each net needs to end up with both endpoints on same layer
                swap_options = []
                if src1_stub and src2_stub:
                    # Option A: Net1 src→tgt1, Net2 src→tgt2 (both go to their tgt layers)
                    swap_options.append(('src', 'src', src1_stub, tgt1, src2_stub, tgt2))
                if tgt1_stub and tgt2_stub:
                    # Option B: Net1 tgt→src1, Net2 tgt→src2 (both go to their src layers)
                    swap_options.append(('tgt', 'tgt', tgt1_stub, src1, tgt2_stub, src2))
                if src1_stub and tgt2_stub:
                    # Option C: Net1 src→tgt1, Net2 tgt→src2
                    swap_options.append(('src', 'tgt', src1_stub, tgt1, tgt2_stub, src2))
                if tgt1_stub and src2_stub:
                    # Option D: Net1 tgt→src1, Net2 src→tgt2
                    swap_options.append(('tgt', 'src', tgt1_stub, src1, src2_stub, tgt2))

                for opt_name1, opt_name2, stub1, dest1, stub2, dest2 in swap_options:
                    valid1, reason1 = validate_single_swap(
                        stub1, dest1, combined_stubs_by_layer, pcb_data, config,
                        swap_partner_name=net2_name, swap_partner_net_ids={net2_id}
                    )
                    valid2, reason2 = validate_single_swap(
                        stub2, dest2, combined_stubs_by_layer, pcb_data, config,
                        swap_partner_name=net1_name, swap_partner_net_ids={net1_id}
                    )
                    if valid1 and valid2:
                        # Apply both swaps
                        vias1, mods1 = apply_stub_layer_switch(pcb_data, stub1, dest1, config, debug=False)
                        vias2, mods2 = apply_stub_layer_switch(pcb_data, stub2, dest2, config, debug=False)
                        all_swap_vias.extend(vias1 + vias2)
                        all_segment_modifications.extend(mods1 + mods2)
                        applied_single_swaps.add(net1_name)
                        applied_single_swaps.add(net2_name)
                        swap_pair_count += 1
                        total_layer_swaps += 2
                        print(f"  Swap pair ({opt_name1}/{opt_name2}): {net1_name} <-> {net2_name}")
                        break
                else:
                    continue  # No valid option found, try next partner
                break  # Found a valid option, exit inner loop

    # PHASE 2: Try remaining solo switches (swap pair candidates that failed)
    for net_name, (src_layer, tgt_layer, sources, targets, net_id) in single_net_layer_info.items():
        if net_name in applied_single_swaps:
            continue

        # Try source -> target layer switch first
        src_stub = get_stub_info(pcb_data, net_id, sources[0][3], sources[0][4], src_layer)

        # Check can_swap_to_top_layer restriction
        if src_stub and (can_swap_to_top_layer or tgt_layer != 'F.Cu'):
            valid, reason = validate_single_swap(
                src_stub, tgt_layer, combined_stubs_by_layer, pcb_data, config
            )
            if valid:
                vias, mods = apply_stub_layer_switch(pcb_data, src_stub, tgt_layer, config, debug=False)
                all_swap_vias.extend(vias)
                all_segment_modifications.extend(mods)
                applied_single_swaps.add(net_name)
                solo_switch_count += 1
                total_layer_swaps += 1
                print(f"  Solo source switch: {net_name} ({src_layer}->{tgt_layer})")
                continue

        # Try target -> source layer switch as fallback
        tgt_stub = get_stub_info(pcb_data, net_id, targets[0][3], targets[0][4], tgt_layer)
        if tgt_stub and (can_swap_to_top_layer or src_layer != 'F.Cu'):
            valid, reason = validate_single_swap(
                tgt_stub, src_layer, combined_stubs_by_layer, pcb_data, config
            )
            if valid:
                vias, mods = apply_stub_layer_switch(pcb_data, tgt_stub, src_layer, config, debug=False)
                all_swap_vias.extend(vias)
                all_segment_modifications.extend(mods)
                applied_single_swaps.add(net_name)
                solo_switch_count += 1
                total_layer_swaps += 1
                print(f"  Solo target switch: {net_name} ({tgt_layer}->{src_layer})")
                continue

        # Report if no swap found
        if verbose:
            print(f"  No swap found: {net_name} ({src_layer}->{tgt_layer}) - will need via")

    if swap_pair_count > 0:
        print(f"Applied {swap_pair_count} single-ended swap pair(s) ({swap_pair_count * 2} nets)")
    if solo_switch_count > 0:
        print(f"Applied {solo_switch_count} single-ended solo switch(es)")

    return total_layer_swaps

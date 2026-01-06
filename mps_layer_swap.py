"""
MPS-aware layer swap optimization for PCB routing.

This module handles layer swaps specifically for resolving MPS crossing conflicts.
When MPS ordering puts nets in Round 2+ due to same-layer crossings, this module
attempts layer swaps to eliminate those crossings, then triggers an MPS re-run.
"""

from typing import Dict, Set, List, Tuple, Optional
from dataclasses import dataclass

from kicad_parser import PCBData
from routing_config import GridRouteConfig, DiffPairNet
from routing_utils import MPSResult
from stub_layer_switching import (
    get_stub_info, apply_stub_layer_switch, validate_swap, validate_single_swap,
    StubInfo
)
from diff_pair_routing import get_diff_pair_endpoints
from routing_utils import get_net_endpoints


@dataclass
class MPSLayerSwapResult:
    """Result of MPS-aware layer swap optimization."""
    swaps_applied: int
    nets_swapped: Set[int]


def _get_unit_stub_info(
    pcb_data: PCBData,
    config: GridRouteConfig,
    unit_id: int,
    unit_to_nets: Dict[int, List[int]],
    diff_pairs: Dict[str, DiffPairNet],
    stub_type: str  # 'source' or 'target'
) -> Tuple[Optional[StubInfo], Optional[StubInfo], Optional[str], bool]:
    """
    Get stub info for a routing unit (diff pair or single net).

    Args:
        pcb_data: PCB data
        config: Routing configuration
        unit_id: Unit ID (P net ID for diff pairs, net ID for singles)
        unit_to_nets: Mapping from unit_id to [net_ids]
        diff_pairs: Dict of diff pairs
        stub_type: 'source' or 'target'

    Returns:
        (stub_p, stub_n, current_layer, is_diff_pair)
        For single-ended: stub_n will be None
    """
    net_ids = unit_to_nets.get(unit_id, [unit_id])
    is_diff_pair = len(net_ids) == 2

    if is_diff_pair:
        # Find the diff pair for this unit
        pair = None
        pair_name = None
        for name, p in diff_pairs.items():
            if p.p_net_id in net_ids and p.n_net_id in net_ids:
                pair = p
                pair_name = name
                break

        if not pair:
            return None, None, None, True

        sources, targets, error = get_diff_pair_endpoints(
            pcb_data, pair.p_net_id, pair.n_net_id, config
        )
        if error or not sources or not targets:
            return None, None, None, True

        if stub_type == 'source':
            layer = config.layers[sources[0][4]]
            stub_p = get_stub_info(pcb_data, pair.p_net_id,
                                   sources[0][5], sources[0][6], layer)
            stub_n = get_stub_info(pcb_data, pair.n_net_id,
                                   sources[0][7], sources[0][8], layer)
        else:
            layer = config.layers[targets[0][4]]
            stub_p = get_stub_info(pcb_data, pair.p_net_id,
                                   targets[0][5], targets[0][6], layer)
            stub_n = get_stub_info(pcb_data, pair.n_net_id,
                                   targets[0][7], targets[0][8], layer)

        return stub_p, stub_n, layer, True
    else:
        # Single-ended net
        net_id = net_ids[0]
        sources, targets, error = get_net_endpoints(pcb_data, net_id, config)
        if error or not sources or not targets:
            return None, None, None, False

        if stub_type == 'source':
            # sources[0] is (gx, gy, layer_idx, orig_x, orig_y)
            layer = config.layers[sources[0][2]]
            stub = get_stub_info(pcb_data, net_id,
                                 sources[0][3], sources[0][4], layer)
        else:
            # targets[0] is (gx, gy, layer_idx, orig_x, orig_y)
            layer = config.layers[targets[0][2]]
            stub = get_stub_info(pcb_data, net_id,
                                 targets[0][3], targets[0][4], layer)

        return stub, None, layer, False


def _find_alternative_layers(
    current_layer: str,
    available_layers: List[str],
    can_swap_to_top_layer: bool
) -> List[str]:
    """Get list of alternative layers to try swapping to."""
    alternatives = []
    for layer in available_layers:
        if layer == current_layer:
            continue
        if not can_swap_to_top_layer and layer == 'F.Cu':
            continue
        alternatives.append(layer)
    return alternatives


def _would_eliminate_conflict(
    unit_a_layers: Tuple[Set[str], Set[str]],
    unit_b_layers: Tuple[Set[str], Set[str]],
    swap_unit: str,  # 'a' or 'b'
    swap_type: str,  # 'source', 'target', or 'both'
    from_layer: str,
    to_layer: str
) -> bool:
    """
    Check if swapping a stub would eliminate the layer overlap between two units.

    Returns True if after the swap, the units would have no shared layers.
    """
    # Copy the layer sets
    a_src = set(unit_a_layers[0])
    a_tgt = set(unit_a_layers[1])
    b_src = set(unit_b_layers[0])
    b_tgt = set(unit_b_layers[1])

    # Apply the hypothetical swap
    if swap_unit == 'a':
        if swap_type in ('source', 'both'):
            a_src.discard(from_layer)
            a_src.add(to_layer)
        if swap_type in ('target', 'both'):
            a_tgt.discard(from_layer)
            a_tgt.add(to_layer)
    else:
        if swap_type in ('source', 'both'):
            b_src.discard(from_layer)
            b_src.add(to_layer)
        if swap_type in ('target', 'both'):
            b_tgt.discard(from_layer)
            b_tgt.add(to_layer)

    # Check if there's any layer overlap after the swap
    a_all = a_src | a_tgt
    b_all = b_src | b_tgt
    return len(a_all & b_all) == 0


def try_mps_aware_layer_swaps(
    pcb_data: PCBData,
    config: GridRouteConfig,
    mps_result: MPSResult,
    diff_pairs: Dict[str, DiffPairNet],
    available_layers: List[str],
    can_swap_to_top_layer: bool,
    all_segment_modifications: List,
    all_swap_vias: List,
    all_stubs_by_layer: Dict,
    stub_endpoints_by_layer: Dict,
    max_iterations: int = 10,
    verbose: bool = False
) -> MPSLayerSwapResult:
    """
    Try layer swaps to reduce MPS rounds by eliminating same-layer crossings.

    When two units conflict (cross) AND share layers, attempting to swap one
    to a different layer can eliminate the conflict since crossing_layer_check
    only counts crossings on the same layer.

    Args:
        pcb_data: PCB data (modified in place)
        config: Routing configuration
        mps_result: Extended MPS result with conflicts and layer info
        diff_pairs: Dict of all diff pairs
        available_layers: List of available layer names
        can_swap_to_top_layer: Whether F.Cu swaps are allowed
        all_segment_modifications: List to append layer changes (modified in place)
        all_swap_vias: List to append vias (modified in place)
        all_stubs_by_layer: Pre-computed stubs by layer for validation
        stub_endpoints_by_layer: Pre-computed stub endpoints for validation
        max_iterations: Maximum swap iterations
        verbose: Print verbose output

    Returns:
        MPSLayerSwapResult with number of swaps applied
    """
    swaps_applied = 0
    nets_swapped: Set[int] = set()

    # Only process if there are Round 2+ units (actual crossing conflicts)
    if mps_result.num_rounds <= 1:
        if verbose:
            print("MPS layer swap: No Round 2+ units, nothing to optimize")
        return MPSLayerSwapResult(swaps_applied=0, nets_swapped=set())

    # Find Round 2+ units and their Round 1 conflicts
    round2_plus_units = [
        uid for uid, rnd in mps_result.round_assignments.items()
        if rnd > 1
    ]

    if verbose:
        print(f"MPS layer swap: Found {len(round2_plus_units)} Round 2+ unit(s)")

    for iteration in range(max_iterations):
        made_progress = False

        for r2_unit in round2_plus_units:
            if r2_unit in nets_swapped:
                continue

            # Find Round 1 units this conflicts with
            r1_conflicts = [
                uid for uid in mps_result.conflicts.get(r2_unit, set())
                if mps_result.round_assignments.get(uid, 1) == 1
            ]

            if not r1_conflicts:
                continue

            r2_layers = mps_result.unit_layers.get(r2_unit, (set(), set()))
            r2_all = r2_layers[0] | r2_layers[1]

            for r1_unit in r1_conflicts:
                r1_layers = mps_result.unit_layers.get(r1_unit, (set(), set()))
                r1_all = r1_layers[0] | r1_layers[1]

                shared_layers = r2_all & r1_all
                if not shared_layers:
                    continue  # No actual layer conflict

                r2_name = mps_result.unit_names.get(r2_unit, f"Net {r2_unit}")

                if verbose:
                    r1_name = mps_result.unit_names.get(r1_unit, f"Net {r1_unit}")
                    print(f"  Trying to resolve conflict: {r2_name} (R2) vs {r1_name} (R1) on {shared_layers}")

                # Try swapping stubs to different layers
                # First try R2 unit, then try R1 unit if R2 can't be swapped
                # For each unit, try individual swaps first, then both together
                for swap_unit_id, swap_unit_name, swap_unit_layers in [
                    (r2_unit, r2_name, r2_layers),
                    (r1_unit, mps_result.unit_names.get(r1_unit, f"Net {r1_unit}"), r1_layers)
                ]:
                    if made_progress:
                        break  # Already resolved this conflict

                    other_unit_layers = r1_layers if swap_unit_id == r2_unit else r2_layers
                    swap_which = 'a' if swap_unit_id == r2_unit else 'b'

                    if verbose and swap_unit_id == r1_unit:
                        print(f"  Trying R1 unit {swap_unit_name} instead...")

                    for stub_type in ['source', 'target', 'both']:
                        if stub_type == 'both':
                            # Get both source and target stubs
                            src_stub_p, src_stub_n, src_layer, is_diff_pair = _get_unit_stub_info(
                                pcb_data, config, swap_unit_id,
                                mps_result.unit_to_nets, diff_pairs, 'source'
                            )
                            tgt_stub_p, tgt_stub_n, tgt_layer, _ = _get_unit_stub_info(
                                pcb_data, config, swap_unit_id,
                                mps_result.unit_to_nets, diff_pairs, 'target'
                            )
                            if not src_stub_p or not tgt_stub_p:
                                if verbose:
                                    print(f"    both: could not get stub info")
                                continue
                            # Both must be on the shared layer for 'both' swap to make sense
                            if src_layer != tgt_layer or src_layer not in shared_layers:
                                if verbose:
                                    print(f"    both: source ({src_layer}) and target ({tgt_layer}) not same shared layer")
                                continue
                            current_layer = src_layer
                            stub_p, stub_n = src_stub_p, src_stub_n
                        else:
                            stub_p, stub_n, current_layer, is_diff_pair = _get_unit_stub_info(
                                pcb_data, config, swap_unit_id,
                                mps_result.unit_to_nets, diff_pairs, stub_type
                            )

                            if not stub_p:
                                if verbose:
                                    print(f"    {stub_type}: could not get stub info")
                                continue

                            if current_layer not in shared_layers:
                                if verbose:
                                    print(f"    {stub_type}: current layer {current_layer} not in shared layers {shared_layers}")
                                continue  # This stub isn't on a shared layer

                        if verbose:
                            print(f"    {stub_type}: stub on {current_layer}, trying alternatives...")

                        # Try alternative layers
                        alt_layers = _find_alternative_layers(
                            current_layer, available_layers, can_swap_to_top_layer
                        )

                        for target_layer in alt_layers:
                            # Check if this swap would eliminate the layer overlap
                            would_elim = _would_eliminate_conflict(
                                r2_layers, r1_layers, swap_which, stub_type,
                                current_layer, target_layer
                            )
                            if not would_elim:
                                if verbose:
                                    print(f"      -> {target_layer}: would not eliminate conflict")
                                continue
                            if verbose:
                                print(f"      -> {target_layer}: would eliminate conflict, validating...")

                            # Validate the swap(s)
                            if stub_type == 'both':
                                # Validate both source and target
                                if is_diff_pair and src_stub_n:
                                    valid_src, reason_src = validate_swap(
                                        src_stub_p, src_stub_n, target_layer,
                                        all_stubs_by_layer, pcb_data, config,
                                        stub_endpoints_by_layer=stub_endpoints_by_layer
                                    )
                                else:
                                    valid_src, reason_src = validate_single_swap(
                                        src_stub_p, target_layer,
                                        all_stubs_by_layer, pcb_data, config
                                    )

                                if is_diff_pair and tgt_stub_n:
                                    valid_tgt, reason_tgt = validate_swap(
                                        tgt_stub_p, tgt_stub_n, target_layer,
                                        all_stubs_by_layer, pcb_data, config,
                                        stub_endpoints_by_layer=stub_endpoints_by_layer
                                    )
                                else:
                                    valid_tgt, reason_tgt = validate_single_swap(
                                        tgt_stub_p, target_layer,
                                        all_stubs_by_layer, pcb_data, config
                                    )

                                valid = valid_src and valid_tgt
                                reason = f"src: {reason_src}; tgt: {reason_tgt}" if not valid else ""
                            else:
                                if is_diff_pair and stub_n:
                                    valid, reason = validate_swap(
                                        stub_p, stub_n, target_layer,
                                        all_stubs_by_layer, pcb_data, config,
                                        stub_endpoints_by_layer=stub_endpoints_by_layer
                                    )
                                else:
                                    valid, reason = validate_single_swap(
                                        stub_p, target_layer,
                                        all_stubs_by_layer, pcb_data, config
                                    )

                            if not valid:
                                if verbose:
                                    print(f"      -> {target_layer}: invalid - {reason}")
                                continue

                            # Apply the swap(s)
                            total_vias = []
                            if stub_type == 'both':
                                # Apply source swap
                                new_vias, seg_mods = apply_stub_layer_switch(
                                    pcb_data, src_stub_p, target_layer, config, debug=verbose
                                )
                                all_swap_vias.extend(new_vias)
                                all_segment_modifications.extend(seg_mods)
                                total_vias.extend(new_vias)

                                if is_diff_pair and src_stub_n:
                                    new_vias_n, seg_mods_n = apply_stub_layer_switch(
                                        pcb_data, src_stub_n, target_layer, config, debug=verbose
                                    )
                                    all_swap_vias.extend(new_vias_n)
                                    all_segment_modifications.extend(seg_mods_n)
                                    total_vias.extend(new_vias_n)

                                # Apply target swap
                                new_vias_t, seg_mods_t = apply_stub_layer_switch(
                                    pcb_data, tgt_stub_p, target_layer, config, debug=verbose
                                )
                                all_swap_vias.extend(new_vias_t)
                                all_segment_modifications.extend(seg_mods_t)
                                total_vias.extend(new_vias_t)

                                if is_diff_pair and tgt_stub_n:
                                    new_vias_tn, seg_mods_tn = apply_stub_layer_switch(
                                        pcb_data, tgt_stub_n, target_layer, config, debug=verbose
                                    )
                                    all_swap_vias.extend(new_vias_tn)
                                    all_segment_modifications.extend(seg_mods_tn)
                                    total_vias.extend(new_vias_tn)

                                swaps_applied += 2  # Count as 2 swaps (source + target)
                            else:
                                new_vias, seg_mods = apply_stub_layer_switch(
                                    pcb_data, stub_p, target_layer, config, debug=verbose
                                )
                                all_swap_vias.extend(new_vias)
                                all_segment_modifications.extend(seg_mods)
                                total_vias.extend(new_vias)

                                if is_diff_pair and stub_n:
                                    new_vias_n, seg_mods_n = apply_stub_layer_switch(
                                        pcb_data, stub_n, target_layer, config, debug=verbose
                                    )
                                    all_swap_vias.extend(new_vias_n)
                                    all_segment_modifications.extend(seg_mods_n)
                                    total_vias.extend(new_vias_n)

                                swaps_applied += 1

                            # Update tracking
                            nets_swapped.update(mps_result.unit_to_nets.get(swap_unit_id, [swap_unit_id]))
                            made_progress = True

                            # Update the stubs_by_layer cache (remove old, add new)
                            # This is a simplified update - full update would require re-collecting
                            if current_layer in all_stubs_by_layer:
                                all_stubs_by_layer[current_layer] = [
                                    (name, segs) for name, segs in all_stubs_by_layer[current_layer]
                                    if name != swap_unit_name
                                ]

                            via_msg = f" (added {len(total_vias)} via(s))" if total_vias else ""
                            print(f"  MPS layer swap: {swap_unit_name} {stub_type} {current_layer}->{target_layer}{via_msg}")

                            break  # Stop trying layers for this stub

                        if made_progress:
                            break  # Stop trying stub types

                if made_progress:
                    break  # Stop trying R1 conflicts

            if made_progress:
                break  # Start new iteration

        if not made_progress:
            break  # No more improvements possible

    if swaps_applied > 0:
        print(f"MPS layer swap: Applied {swaps_applied} layer swap(s)")

    return MPSLayerSwapResult(
        swaps_applied=swaps_applied,
        nets_swapped=nets_swapped
    )

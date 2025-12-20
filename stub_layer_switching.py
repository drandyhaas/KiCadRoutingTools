"""
Stub layer switching optimization for differential pair routing.

Provides functions to dynamically switch stub layers to avoid vias when the
source and target stubs are on different layers.
"""

import math
from typing import List, Optional, Tuple, Dict
from dataclasses import dataclass

from kicad_parser import PCBData, Segment, Via, Pad
from routing_config import GridRouteConfig, GridCoord, DiffPair
from routing_utils import get_stub_segments, find_stub_free_ends
from obstacle_map import check_stub_layer_clearance, check_line_clearance

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))
try:
    from grid_router import GridObstacleMap
except ImportError:
    GridObstacleMap = None


@dataclass
class StubInfo:
    """Information about a stub endpoint."""
    net_id: int
    x: float
    y: float
    layer: str
    segments: List[Segment]
    pad_x: float
    pad_y: float
    has_pad_via: bool  # True if pad already has via to other layers


@dataclass
class LayerSwitchOption:
    """A potential layer switch for stub pairs."""
    src_p_stub: StubInfo
    src_n_stub: StubInfo
    tgt_p_stub: StubInfo
    tgt_n_stub: StubInfo
    switch_source: bool  # True if source stubs should switch
    switch_target: bool  # True if target stubs should switch
    new_layer: str
    needs_src_pad_vias: bool
    needs_tgt_pad_vias: bool
    # For swaps with another pair
    swap_pair: Optional['DiffPair'] = None
    swap_pair_stubs: Optional[Tuple[StubInfo, StubInfo]] = None  # (p_stub, n_stub) of swap pair


def get_stub_info(pcb_data: PCBData, net_id: int, stub_x: float, stub_y: float,
                  stub_layer: str, tolerance: float = 0.05) -> Optional[StubInfo]:
    """
    Gather information about a stub for layer switching analysis.

    Args:
        pcb_data: PCB data
        net_id: Net ID of the stub
        stub_x, stub_y: Position of the stub free end
        stub_layer: Current layer of the stub
        tolerance: Distance tolerance for matching

    Returns:
        StubInfo with all relevant data, or None if stub can't be analyzed
    """
    # Get segments forming the stub
    segments = get_stub_segments(pcb_data, net_id, stub_x, stub_y, stub_layer, tolerance)
    if not segments:
        return None

    # Find the pad this stub connects to
    net_pads = pcb_data.pads_by_net.get(net_id, [])
    if not net_pads:
        return None

    # Find which pad the stub connects to (end opposite from stub_x, stub_y)
    pad_x, pad_y = None, None
    for seg in segments:
        # Check which end is farther from stub position
        dist_start = abs(seg.start_x - stub_x) + abs(seg.start_y - stub_y)
        dist_end = abs(seg.end_x - stub_x) + abs(seg.end_y - stub_y)
        if dist_start > dist_end:
            check_x, check_y = seg.start_x, seg.start_y
        else:
            check_x, check_y = seg.end_x, seg.end_y

        for pad in net_pads:
            if abs(pad.global_x - check_x) < tolerance and abs(pad.global_y - check_y) < tolerance:
                pad_x, pad_y = pad.global_x, pad.global_y
                break
        if pad_x is not None:
            break

    if pad_x is None:
        # Use first pad as fallback
        pad_x, pad_y = net_pads[0].global_x, net_pads[0].global_y

    # Check if pad already has a via (connecting to other layers)
    has_pad_via = False
    for via in pcb_data.vias:
        if via.net_id == net_id:
            dist = math.sqrt((via.x - pad_x) ** 2 + (via.y - pad_y) ** 2)
            if dist < tolerance:
                has_pad_via = True
                break

    return StubInfo(
        net_id=net_id,
        x=stub_x,
        y=stub_y,
        layer=stub_layer,
        segments=segments,
        pad_x=pad_x,
        pad_y=pad_y,
        has_pad_via=has_pad_via
    )


def needs_pad_via_for_switch(stub: StubInfo) -> bool:
    """
    Check if switching this stub requires a new pad via.

    A pad via is needed only when:
    1. Stub is currently on F.Cu (top layer)
    2. No via already exists at the pad position

    Returns:
        True if pad via needed for layer switch
    """
    if stub.layer != 'F.Cu':
        return False  # Already has via from pad to stub layer
    return not stub.has_pad_via


def can_switch_stub_to_layer(stub: StubInfo, target_layer: str, target_layer_idx: int,
                              obstacles: GridObstacleMap, config: GridRouteConfig,
                              debug: bool = False) -> bool:
    """
    Check if a stub's segments can exist on the target layer without conflicts.

    Args:
        stub: StubInfo for the stub to check
        target_layer: Target layer name
        target_layer_idx: Target layer index
        obstacles: Obstacle map to check against
        config: Routing configuration
        debug: If True, print which segment is blocked

    Returns:
        True if all stub segments are clear on target layer
    """
    from obstacle_map import check_line_clearance
    coord = GridCoord(config.grid_step)

    for seg in stub.segments:
        if not check_line_clearance(obstacles, seg.start_x, seg.start_y,
                                     seg.end_x, seg.end_y, target_layer_idx, config):
            if debug:
                gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
                gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
                print(f"        Blocked: ({seg.start_x:.2f},{seg.start_y:.2f})->({seg.end_x:.2f},{seg.end_y:.2f}) on {target_layer}")
            return False
    return True


def find_pair_for_net_id(net_id: int, unrouted_pairs: List[Tuple[str, 'DiffPair']]) -> Optional[Tuple[str, 'DiffPair']]:
    """Find the diff pair containing a given net ID."""
    for pair_name, pair in unrouted_pairs:
        if pair.p_net_id == net_id or pair.n_net_id == net_id:
            return (pair_name, pair)
    return None


def get_pair_stubs_on_layer(pcb_data: PCBData, pair: 'DiffPair', layer: str,
                            config: GridRouteConfig) -> Optional[Tuple[StubInfo, StubInfo]]:
    """Get stub info for a diff pair's stubs on a specific layer."""
    from routing_utils import find_stub_free_ends, find_connected_groups

    # Get P net stubs
    p_segments = [s for s in pcb_data.segments if s.net_id == pair.p_net_id]
    if not p_segments:
        return None

    p_groups = find_connected_groups(p_segments)
    if not p_groups:
        return None

    p_pads = pcb_data.pads_by_net.get(pair.p_net_id, [])

    # Find P stub on the specified layer
    p_stub = None
    for group in p_groups:
        free_ends = find_stub_free_ends(group, p_pads)
        for x, y, stub_layer in free_ends:
            if stub_layer == layer:
                p_stub = get_stub_info(pcb_data, pair.p_net_id, x, y, layer)
                break
        if p_stub:
            break

    if not p_stub:
        return None

    # Get N net stubs
    n_segments = [s for s in pcb_data.segments if s.net_id == pair.n_net_id]
    if not n_segments:
        return None

    n_groups = find_connected_groups(n_segments)
    if not n_groups:
        return None

    n_pads = pcb_data.pads_by_net.get(pair.n_net_id, [])

    # Find N stub on the specified layer
    n_stub = None
    for group in n_groups:
        free_ends = find_stub_free_ends(group, n_pads)
        for x, y, stub_layer in free_ends:
            if stub_layer == layer:
                n_stub = get_stub_info(pcb_data, pair.n_net_id, x, y, layer)
                break
        if n_stub:
            break

    if not n_stub:
        return None

    return (p_stub, n_stub)


def can_swap_with_pair(pcb_data: PCBData, our_stubs: Tuple[StubInfo, StubInfo],
                       our_layer: str, their_pair: 'DiffPair', their_layer: str,
                       obstacles: GridObstacleMap, config: GridRouteConfig,
                       debug: bool = False) -> Optional[Tuple[StubInfo, StubInfo]]:
    """
    Check if we can swap layers with another pair.

    We want to move our stubs from our_layer to their_layer.
    They need to move from their_layer to our_layer.

    The key insight: we need to temporarily ignore both pairs' stubs when checking
    clearance, since they'll be moving simultaneously.
    """
    # Get their stubs on their current layer
    their_stubs = get_pair_stubs_on_layer(pcb_data, their_pair, their_layer, config)
    if not their_stubs:
        return None

    their_p_stub, their_n_stub = their_stubs
    our_p_stub, our_n_stub = our_stubs

    # For the swap to work, their stubs need to be clear on OUR layer
    # But we need to exclude OUR stubs from the obstacle check (since we're moving)
    # This is the key: check if their stub positions are clear on our_layer,
    # ignoring our current stubs at that layer

    layer_map = {name: idx for idx, name in enumerate(config.layers)}
    our_layer_idx = layer_map.get(our_layer)
    their_layer_idx = layer_map.get(their_layer)

    if our_layer_idx is None or their_layer_idx is None:
        return None

    # Check if their stubs can exist on our layer
    # Note: We can't easily exclude our stubs from obstacles, so we do a position-based check
    # Their stubs should be at the same position as our stubs (that's why they're blocking)
    # So if our stubs can move to their layer, their stubs can move to our layer

    # Actually, for a proper check we need to verify their stubs aren't blocked by OTHER obstacles
    # For now, assume if they're at similar positions to our stubs, the swap works

    their_p_can = can_switch_stub_to_layer(their_p_stub, our_layer, our_layer_idx, obstacles, config, debug=debug)
    their_n_can = can_switch_stub_to_layer(their_n_stub, our_layer, our_layer_idx, obstacles, config, debug=debug)

    if debug:
        print(f"        Swap check: their pair can move to {our_layer}: P={their_p_can}, N={their_n_can}")

    # If their stubs can move to our layer, the swap is possible
    # (They may be blocked by our stubs, but we're moving too)
    # For a more accurate check, we'd need to rebuild obstacles excluding both pairs

    if their_p_can and their_n_can:
        return their_stubs

    # Stubs can't directly move - check if blocking is specifically from OUR stubs
    # If so, a swap would work since we're moving away
    # Check stub center distances to see if they're mutually blocking
    our_center_x = (our_p_stub.x + our_n_stub.x) / 2
    our_center_y = (our_p_stub.y + our_n_stub.y) / 2
    their_center_x = (their_p_stub.x + their_n_stub.x) / 2
    their_center_y = (their_p_stub.y + their_n_stub.y) / 2

    dist = math.sqrt((our_center_x - their_center_x) ** 2 + (our_center_y - their_center_y) ** 2)

    if debug:
        print(f"        Stub centers distance: {dist:.2f}mm")

    # Only consider swap if stubs are close AND our stubs are specifically blocking them
    if dist < 1.0:
        # Check if the blocking on our_layer near their stubs is from OUR nets specifically
        # Use a tolerance slightly larger than the stub distance to ensure we find nearby stubs
        our_net_ids = {our_p_stub.net_id, our_n_stub.net_id}
        blocking_at_their_pos = find_blocking_stubs_at_position(
            pcb_data, their_center_x, their_center_y, our_layer,
            [their_p_stub.net_id, their_n_stub.net_id], tolerance=max(dist + 0.3, 0.5)
        )

        # Check if blocking is primarily from our nets
        our_blocking_count = sum(1 for net_id, _ in blocking_at_their_pos if net_id in our_net_ids)
        other_blocking_count = len(blocking_at_their_pos) - our_blocking_count

        if debug:
            print(f"        Blocking at their position: {our_blocking_count} from our nets, {other_blocking_count} from others")

        # Only allow swap if blocking is from our nets and no other blocking
        if our_blocking_count > 0 and other_blocking_count == 0:
            if debug:
                print(f"        Mutual blocking confirmed, swap possible")
            return their_stubs
        elif debug:
            print(f"        Swap rejected: blocked by other nets")

    return None


def find_layer_switch_options(pcb_data: PCBData,
                               p_net_id: int, n_net_id: int,
                               src_p_stub: StubInfo, src_n_stub: StubInfo,
                               tgt_p_stub: StubInfo, tgt_n_stub: StubInfo,
                               obstacles: GridObstacleMap,
                               config: GridRouteConfig,
                               unrouted_pairs: List[Tuple[str, 'DiffPair']] = None,
                               debug: bool = False) -> List[LayerSwitchOption]:
    """
    Find valid layer switch options to avoid vias.

    Evaluates:
    1. Can source stubs switch to target layer?
    2. Can target stubs switch to source layer?
    3. If both fail, no switch possible

    Args:
        pcb_data: PCB data
        p_net_id, n_net_id: Net IDs for P and N
        src_p_stub, src_n_stub: Source stub info for P and N
        tgt_p_stub, tgt_n_stub: Target stub info for P and N
        obstacles: Obstacle map for clearance checking
        config: Routing configuration
        debug: If True, print debug info

    Returns:
        List of valid LayerSwitchOption, ordered by preference (simplest first)
    """
    options = []
    layer_map = {name: idx for idx, name in enumerate(config.layers)}

    src_layer = src_p_stub.layer
    tgt_layer = tgt_p_stub.layer

    if debug:
        print(f"      Layer switch analysis: src={src_layer}, tgt={tgt_layer}")

    # Already on same layer - no switch needed
    if src_layer == tgt_layer:
        if debug:
            print(f"      Already on same layer, no switch needed")
        return options

    # Option 1: Switch source stubs to target layer
    tgt_layer_idx = layer_map.get(tgt_layer)
    if tgt_layer_idx is not None:
        src_p_can = can_switch_stub_to_layer(src_p_stub, tgt_layer, tgt_layer_idx, obstacles, config, debug=debug)
        src_n_can = can_switch_stub_to_layer(src_n_stub, tgt_layer, tgt_layer_idx, obstacles, config, debug=debug)

        if debug:
            print(f"      Switch src to tgt layer ({tgt_layer}): P={src_p_can}, N={src_n_can}")

        if src_p_can and src_n_can:
            needs_via = needs_pad_via_for_switch(src_p_stub) or needs_pad_via_for_switch(src_n_stub)
            options.append(LayerSwitchOption(
                src_p_stub=src_p_stub,
                src_n_stub=src_n_stub,
                tgt_p_stub=tgt_p_stub,
                tgt_n_stub=tgt_n_stub,
                switch_source=True,
                switch_target=False,
                new_layer=tgt_layer,
                needs_src_pad_vias=needs_via,
                needs_tgt_pad_vias=False
            ))

    # Option 2: Switch target stubs to source layer
    src_layer_idx = layer_map.get(src_layer)
    if src_layer_idx is not None:
        tgt_p_can = can_switch_stub_to_layer(tgt_p_stub, src_layer, src_layer_idx, obstacles, config, debug=debug)
        tgt_n_can = can_switch_stub_to_layer(tgt_n_stub, src_layer, src_layer_idx, obstacles, config, debug=debug)

        if debug:
            print(f"      Switch tgt to src layer ({src_layer}): P={tgt_p_can}, N={tgt_n_can}")

        if tgt_p_can and tgt_n_can:
            needs_via = needs_pad_via_for_switch(tgt_p_stub) or needs_pad_via_for_switch(tgt_n_stub)
            options.append(LayerSwitchOption(
                src_p_stub=src_p_stub,
                src_n_stub=src_n_stub,
                tgt_p_stub=tgt_p_stub,
                tgt_n_stub=tgt_n_stub,
                switch_source=False,
                switch_target=True,
                new_layer=src_layer,
                needs_src_pad_vias=False,
                needs_tgt_pad_vias=needs_via
            ))

    # Option 3: Try swap with blocking pair (if simple switches failed and we have unrouted pairs info)
    if not options and unrouted_pairs:
        if debug:
            print(f"      Trying swap with blocking pairs...")

        # Try swapping source stubs with a blocking pair on target layer
        blocking = find_blocking_stubs_at_position(
            pcb_data, (src_p_stub.x + src_n_stub.x) / 2, (src_p_stub.y + src_n_stub.y) / 2,
            tgt_layer, [p_net_id, n_net_id], tolerance=1.0
        )

        blocking_pairs_checked = set()
        for blocking_net_id, _ in blocking:
            if blocking_net_id in blocking_pairs_checked:
                continue

            blocking_pair_info = find_pair_for_net_id(blocking_net_id, unrouted_pairs)
            if not blocking_pair_info:
                continue

            blocking_pair_name, blocking_pair = blocking_pair_info
            blocking_pairs_checked.add(blocking_pair.p_net_id)
            blocking_pairs_checked.add(blocking_pair.n_net_id)

            if debug:
                print(f"        Checking swap with {blocking_pair_name}...")

            # Check if we can swap with this pair
            their_stubs = can_swap_with_pair(
                pcb_data, (src_p_stub, src_n_stub), src_layer,
                blocking_pair, tgt_layer, obstacles, config, debug=debug
            )

            if their_stubs:
                needs_our_via = needs_pad_via_for_switch(src_p_stub) or needs_pad_via_for_switch(src_n_stub)
                options.append(LayerSwitchOption(
                    src_p_stub=src_p_stub,
                    src_n_stub=src_n_stub,
                    tgt_p_stub=tgt_p_stub,
                    tgt_n_stub=tgt_n_stub,
                    switch_source=True,
                    switch_target=False,
                    new_layer=tgt_layer,
                    needs_src_pad_vias=needs_our_via,
                    needs_tgt_pad_vias=False,
                    swap_pair=blocking_pair,
                    swap_pair_stubs=their_stubs
                ))
                if debug:
                    print(f"        Found swap option with {blocking_pair_name}")
                break  # Found a swap, stop looking

        # If still no options, try swapping target stubs
        if not options:
            blocking = find_blocking_stubs_at_position(
                pcb_data, (tgt_p_stub.x + tgt_n_stub.x) / 2, (tgt_p_stub.y + tgt_n_stub.y) / 2,
                src_layer, [p_net_id, n_net_id], tolerance=1.0
            )

            blocking_pairs_checked = set()
            for blocking_net_id, _ in blocking:
                if blocking_net_id in blocking_pairs_checked:
                    continue

                blocking_pair_info = find_pair_for_net_id(blocking_net_id, unrouted_pairs)
                if not blocking_pair_info:
                    continue

                blocking_pair_name, blocking_pair = blocking_pair_info
                blocking_pairs_checked.add(blocking_pair.p_net_id)
                blocking_pairs_checked.add(blocking_pair.n_net_id)

                if debug:
                    print(f"        Checking swap with {blocking_pair_name}...")

                # Check if we can swap with this pair
                their_stubs = can_swap_with_pair(
                    pcb_data, (tgt_p_stub, tgt_n_stub), tgt_layer,
                    blocking_pair, src_layer, obstacles, config, debug=debug
                )

                if their_stubs:
                    needs_our_via = needs_pad_via_for_switch(tgt_p_stub) or needs_pad_via_for_switch(tgt_n_stub)
                    options.append(LayerSwitchOption(
                        src_p_stub=src_p_stub,
                        src_n_stub=src_n_stub,
                        tgt_p_stub=tgt_p_stub,
                        tgt_n_stub=tgt_n_stub,
                        switch_source=False,
                        switch_target=True,
                        new_layer=src_layer,
                        needs_src_pad_vias=False,
                        needs_tgt_pad_vias=needs_our_via,
                        swap_pair=blocking_pair,
                        swap_pair_stubs=their_stubs
                    ))
                    if debug:
                        print(f"        Found swap option with {blocking_pair_name}")
                    break  # Found a swap, stop looking

    # Sort options: prefer no pad vias needed, then prefer no swap
    options.sort(key=lambda o: (o.swap_pair is not None, o.needs_src_pad_vias or o.needs_tgt_pad_vias))

    return options


def apply_stub_layer_switch(pcb_data: PCBData, stub: StubInfo, new_layer: str,
                             config: GridRouteConfig, debug: bool = True) -> Tuple[List[Via], List[Dict]]:
    """
    Switch a stub to a new layer by modifying segment layers.

    Args:
        pcb_data: PCB data (segments will be modified in place)
        stub: StubInfo for the stub to switch
        new_layer: Target layer name
        config: Routing configuration
        debug: If True, print debug info about modified segments

    Returns:
        Tuple of (new_vias, segment_modifications):
        - new_vias: List of new vias created (pad vias if switching from F.Cu)
        - segment_modifications: List of dicts with segment layer changes for writing to file
    """
    new_vias = []
    segment_mods = []

    if debug:
        print(f"      apply_stub_layer_switch: net={stub.net_id}, from {stub.layer} to {new_layer}")
        print(f"        Stub at ({stub.x:.2f}, {stub.y:.2f}), pad at ({stub.pad_x:.2f}, {stub.pad_y:.2f})")
        print(f"        Modifying {len(stub.segments)} segments:")

    # Collect modifications and modify segment layers
    for seg in stub.segments:
        old_layer = seg.layer
        if debug:
            print(f"          ({seg.start_x:.2f},{seg.start_y:.2f})->({seg.end_x:.2f},{seg.end_y:.2f}) {old_layer} -> {new_layer}")

        # Record modification for file writing
        segment_mods.append({
            'start': (seg.start_x, seg.start_y),
            'end': (seg.end_x, seg.end_y),
            'net_id': seg.net_id,
            'old_layer': old_layer,
            'new_layer': new_layer
        })

        # Modify in memory
        seg.layer = new_layer

    # Create pad via if needed (switching from F.Cu without existing via)
    if needs_pad_via_for_switch(stub):
        via = Via(
            x=stub.pad_x,
            y=stub.pad_y,
            size=config.via_size,
            drill=config.via_drill,
            layers=['F.Cu', 'B.Cu'],  # Through-hole via
            net_id=stub.net_id
        )
        new_vias.append(via)
        pcb_data.vias.append(via)

    return new_vias, segment_mods


def revert_layer_switch_option(pcb_data: PCBData, option: LayerSwitchOption,
                                config: GridRouteConfig, added_vias: List[Via]) -> None:
    """
    Revert a layer switch option that was applied but routing failed.

    Args:
        pcb_data: PCB data (will be modified in place)
        option: LayerSwitchOption that was applied
        config: Routing configuration
        added_vias: Vias that were added during switch (to be removed)
    """
    # Restore our stubs to their original layers
    if option.switch_source:
        original_layer = option.src_p_stub.layer  # This was the layer BEFORE switch
        for seg in option.src_p_stub.segments:
            seg.layer = original_layer
        for seg in option.src_n_stub.segments:
            seg.layer = original_layer

    if option.switch_target:
        original_layer = option.tgt_p_stub.layer  # This was the layer BEFORE switch
        for seg in option.tgt_p_stub.segments:
            seg.layer = original_layer
        for seg in option.tgt_n_stub.segments:
            seg.layer = original_layer

    # Restore swap pair's stubs if there was a swap
    if option.swap_pair and option.swap_pair_stubs:
        their_p_stub, their_n_stub = option.swap_pair_stubs
        original_layer = their_p_stub.layer  # Their original layer before swap
        for seg in their_p_stub.segments:
            seg.layer = original_layer
        for seg in their_n_stub.segments:
            seg.layer = original_layer

    # Remove added vias
    for via in added_vias:
        if via in pcb_data.vias:
            pcb_data.vias.remove(via)


def apply_layer_switch_option(pcb_data: PCBData, option: LayerSwitchOption,
                               config: GridRouteConfig) -> Tuple[List[Via], str, List[Dict]]:
    """
    Apply a layer switch option to the PCB data.

    Args:
        pcb_data: PCB data (will be modified in place)
        option: LayerSwitchOption to apply
        config: Routing configuration

    Returns:
        Tuple of (new_vias, new_common_layer, segment_modifications)
        - new_vias: Vias created for the switch
        - new_common_layer: The layer both stubs are now on
        - segment_modifications: List of dicts with segment layer changes for writing to file
    """
    new_vias = []
    all_segment_mods = []
    new_common_layer = None

    if option.switch_source:
        vias, mods = apply_stub_layer_switch(pcb_data, option.src_p_stub, option.new_layer, config)
        new_vias.extend(vias)
        all_segment_mods.extend(mods)
        vias, mods = apply_stub_layer_switch(pcb_data, option.src_n_stub, option.new_layer, config)
        new_vias.extend(vias)
        all_segment_mods.extend(mods)
        new_common_layer = option.tgt_p_stub.layer  # Target layer is now common

    if option.switch_target:
        vias, mods = apply_stub_layer_switch(pcb_data, option.tgt_p_stub, option.new_layer, config)
        new_vias.extend(vias)
        all_segment_mods.extend(mods)
        vias, mods = apply_stub_layer_switch(pcb_data, option.tgt_n_stub, option.new_layer, config)
        new_vias.extend(vias)
        all_segment_mods.extend(mods)
        new_common_layer = option.src_p_stub.layer  # Source layer is now common

    # If this is a swap, also switch the other pair's stubs
    if option.swap_pair and option.swap_pair_stubs:
        their_p_stub, their_n_stub = option.swap_pair_stubs
        # They move to our original layer
        if option.switch_source:
            # We moved from src_layer to tgt_layer, they move from tgt_layer to src_layer
            their_new_layer = option.src_p_stub.layer  # Our original source layer
        else:
            # We moved from tgt_layer to src_layer, they move from src_layer to tgt_layer
            their_new_layer = option.tgt_p_stub.layer  # Our original target layer

        vias, mods = apply_stub_layer_switch(pcb_data, their_p_stub, their_new_layer, config)
        new_vias.extend(vias)
        all_segment_mods.extend(mods)
        vias, mods = apply_stub_layer_switch(pcb_data, their_n_stub, their_new_layer, config)
        new_vias.extend(vias)
        all_segment_mods.extend(mods)

    return new_vias, new_common_layer, all_segment_mods


def find_blocking_stubs_at_position(pcb_data: PCBData, x: float, y: float,
                                     layer: str, exclude_net_ids: List[int],
                                     tolerance: float = 0.5) -> List[Tuple[int, Segment]]:
    """
    Find stubs from other nets that are blocking a position on a layer.

    Args:
        pcb_data: PCB data
        x, y: Position to check
        layer: Layer to check
        exclude_net_ids: Net IDs to exclude (our own nets)
        tolerance: Distance tolerance in mm

    Returns:
        List of (net_id, segment) tuples for blocking stubs
    """
    blocking = []
    for seg in pcb_data.segments:
        if seg.net_id in exclude_net_ids:
            continue
        if seg.layer != layer:
            continue

        # Check if segment is near the position
        # Simple distance check from position to segment endpoints
        d1 = math.sqrt((seg.start_x - x) ** 2 + (seg.start_y - y) ** 2)
        d2 = math.sqrt((seg.end_x - x) ** 2 + (seg.end_y - y) ** 2)
        if d1 < tolerance or d2 < tolerance:
            blocking.append((seg.net_id, seg))

    return blocking


def find_layer_switch_for_diff_pair(pcb_data: PCBData,
                                     p_net_id: int, n_net_id: int,
                                     src_layer: str, tgt_layer: str,
                                     p_src_x: float, p_src_y: float,
                                     n_src_x: float, n_src_y: float,
                                     p_tgt_x: float, p_tgt_y: float,
                                     n_tgt_x: float, n_tgt_y: float,
                                     obstacles: GridObstacleMap,
                                     config: GridRouteConfig,
                                     unrouted_pairs: List[Tuple[str, 'DiffPair']] = None,
                                     debug: bool = False) -> Optional[LayerSwitchOption]:
    """
    Find a layer switch option for a differential pair (but don't apply it).

    This is used to find possible layer switches before routing. The switch should
    only be applied after routing succeeds.

    Args:
        pcb_data: PCB data
        p_net_id, n_net_id: Net IDs for P and N
        src_layer: Current source layer
        tgt_layer: Current target layer
        p_src_x, p_src_y: P source stub position
        n_src_x, n_src_y: N source stub position
        p_tgt_x, p_tgt_y: P target stub position
        n_tgt_x, n_tgt_y: N target stub position
        obstacles: Obstacle map
        config: Routing configuration
        unrouted_pairs: List of unrouted pairs for swap detection
        debug: If True, print debug info

    Returns:
        LayerSwitchOption if a switch is possible, None otherwise
    """
    # Already on same layer
    if src_layer == tgt_layer:
        return None

    # Get stub info for all four stubs
    src_p_stub = get_stub_info(pcb_data, p_net_id, p_src_x, p_src_y, src_layer)
    src_n_stub = get_stub_info(pcb_data, n_net_id, n_src_x, n_src_y, src_layer)
    tgt_p_stub = get_stub_info(pcb_data, p_net_id, p_tgt_x, p_tgt_y, tgt_layer)
    tgt_n_stub = get_stub_info(pcb_data, n_net_id, n_tgt_x, n_tgt_y, tgt_layer)

    if not all([src_p_stub, src_n_stub, tgt_p_stub, tgt_n_stub]):
        if debug:
            missing = []
            if not src_p_stub: missing.append("src_p")
            if not src_n_stub: missing.append("src_n")
            if not tgt_p_stub: missing.append("tgt_p")
            if not tgt_n_stub: missing.append("tgt_n")
            print(f"    Layer switch: Could not get stub info for: {', '.join(missing)}")
        return None

    if debug:
        print(f"      src_p: {len(src_p_stub.segments)} segs, pad_via={src_p_stub.has_pad_via}")
        print(f"      src_n: {len(src_n_stub.segments)} segs, pad_via={src_n_stub.has_pad_via}")
        print(f"      tgt_p: {len(tgt_p_stub.segments)} segs, pad_via={tgt_p_stub.has_pad_via}")
        print(f"      tgt_n: {len(tgt_n_stub.segments)} segs, pad_via={tgt_n_stub.has_pad_via}")

    # Find layer switch options
    options = find_layer_switch_options(
        pcb_data, p_net_id, n_net_id,
        src_p_stub, src_n_stub, tgt_p_stub, tgt_n_stub,
        obstacles, config, unrouted_pairs=unrouted_pairs, debug=debug
    )

    if not options:
        if debug:
            print("    Layer switch: No valid options found")
        return None

    if debug:
        best = options[0]
        switch_desc = "source" if best.switch_source else "target"
        swap_desc = f" (swap with {best.swap_pair.base_name})" if best.swap_pair else ""
        print(f"    Layer switch: Found option to switch {switch_desc} stubs to {best.new_layer}{swap_desc}")

    return options[0]


def try_layer_switch_for_diff_pair(pcb_data: PCBData,
                                    p_net_id: int, n_net_id: int,
                                    src_layer: str, tgt_layer: str,
                                    p_src_x: float, p_src_y: float,
                                    n_src_x: float, n_src_y: float,
                                    p_tgt_x: float, p_tgt_y: float,
                                    n_tgt_x: float, n_tgt_y: float,
                                    obstacles: GridObstacleMap,
                                    config: GridRouteConfig,
                                    unrouted_pairs: List[Tuple[str, 'DiffPair']] = None,
                                    debug: bool = False) -> Optional[Tuple[str, List[Via]]]:
    """
    Try to find and apply a layer switch to avoid vias for a differential pair.

    DEPRECATED: Use find_layer_switch_for_diff_pair and apply_layer_switch_option separately
    to only apply after routing succeeds.
    """
    option = find_layer_switch_for_diff_pair(
        pcb_data, p_net_id, n_net_id,
        src_layer, tgt_layer,
        p_src_x, p_src_y, n_src_x, n_src_y,
        p_tgt_x, p_tgt_y, n_tgt_x, n_tgt_y,
        obstacles, config, unrouted_pairs, debug
    )

    if not option:
        return None

    # Apply the option
    new_vias, new_layer = apply_layer_switch_option(pcb_data, option, config)

    if debug:
        switch_desc = "source" if option.switch_source else "target"
        via_desc = f", added {len(new_vias)} pad via(s)" if new_vias else ""
        swap_desc = f" (swapped with {option.swap_pair.base_name})" if option.swap_pair else ""
        print(f"    Layer switch: Applied {switch_desc} stubs to {option.new_layer}{via_desc}{swap_desc}")

    return (new_layer, new_vias)

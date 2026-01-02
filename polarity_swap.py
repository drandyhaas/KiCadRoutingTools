"""
Polarity swap functions for differential pair routing.

This module handles swapping the P/N polarity of differential pairs when
the routing algorithm determines that a swap is needed to achieve a valid route.
"""

from typing import Dict, List, Optional, Set, Tuple

from kicad_parser import PCBData, Pad
from routing_config import DiffPair
from routing_utils import find_pad_nearest_to_position, find_connected_segment_positions, pos_key


def apply_polarity_swap(pcb_data: PCBData, result: dict, pad_swaps: List[dict],
                        pair_name: str = None, already_swapped: Set[str] = None) -> bool:
    """
    Apply polarity swap for a diff pair route result.

    When a diff pair route requires P/N polarity swap at the target, this function:
    1. Swaps net IDs of stub segments at the target positions
    2. Swaps net IDs of vias at the target positions
    3. Queues pad swaps for later application to the output file

    If pair_name and already_swapped are provided, skips the swap if this pair
    was already swapped (e.g., before rip-up and reroute).

    Args:
        pcb_data: The PCB data structure
        result: The routing result dict containing polarity_fixed and swap_target_pads info
        pad_swaps: List to append pad swap info to for later file output
        pair_name: Optional name of the diff pair (for tracking already-swapped pairs)
        already_swapped: Optional set of pair names that have already been swapped

    Returns:
        True if swap was applied, False if not needed or failed.
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
        seg_start = pos_key(seg.start_x, seg.start_y)
        seg_end = pos_key(seg.end_x, seg.end_y)
        if seg.net_id == p_net_id and (seg_start in p_stub_positions or seg_end in p_stub_positions):
            seg.net_id = n_net_id
        elif seg.net_id == n_net_id and (seg_start in n_stub_positions or seg_end in n_stub_positions):
            seg.net_id = p_net_id

    # Also swap via net IDs at stub positions
    for via in pcb_data.vias:
        via_pos = pos_key(via.x, via.y)
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


def get_canonical_net_id(net_id: int, diff_pair_by_net_id: Dict[int, Tuple[str, DiffPair]]) -> int:
    """Get canonical net ID for loop prevention tracking.

    For diff pairs, returns P net_id. For single nets, returns net_id as-is.

    Args:
        net_id: The net ID to get the canonical form of
        diff_pair_by_net_id: Dict mapping net IDs to (pair_name, DiffPair) tuples

    Returns:
        The canonical net ID (P net for diff pairs, unchanged for single nets)
    """
    if net_id in diff_pair_by_net_id:
        _, pair = diff_pair_by_net_id[net_id]
        return pair.p_net_id
    return net_id

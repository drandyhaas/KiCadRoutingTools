"""
Apply routing swap results to a live pcbnew board.

When routing runs inside the KiCad plugin (return_results=True), the results
must be applied to the open board the same way write_routed_output applies
them to an output file:
  - diff pair target swaps (pad + stub nets swapped between two pairs)
  - segment layer modifications (stub layer switching)
  - polarity fix swaps (P/N pad + stub nets swapped within a pair)

Without these, routed tracks point at pads that still carry the old nets,
producing shorts/clearance errors at the swapped pads.

This module deliberately avoids wx so it can be used headless.
"""

from routing_utils import pos_key
from typing import Type


def _to_mm_key(pcbnew, point):
    """Convert a pcbnew VECTOR2I to a pos_key in mm."""
    return pos_key(pcbnew.ToMM(point.x), pcbnew.ToMM(point.y))


def _find_pad(board, component_ref, pad_number):
    """Find a pad on the board by footprint reference and pad number."""
    footprint = board.FindFootprintByReference(component_ref)
    if footprint is None:
        return None
    for pad in footprint.Pads():
        if str(pad.GetNumber()) == str(pad_number):
            return pad
    return None


def _swap_pad_nets(board, pad_info1, pad_info2):
    """Swap the net assignments of two pads (given as kicad_parser Pad objects)."""
    pad1 = _find_pad(board, pad_info1.component_ref, pad_info1.pad_number)
    pad2 = _find_pad(board, pad_info2.component_ref, pad_info2.pad_number)
    if pad1 is None or pad2 is None:
        print(f"  WARNING: could not find pads to swap: "
              f"{pad_info1.component_ref}:{pad_info1.pad_number} <-> "
              f"{pad_info2.component_ref}:{pad_info2.pad_number}")
        return False
    net1 = pad1.GetNetCode()
    pad1.SetNetCode(pad2.GetNetCode())
    pad2.SetNetCode(net1)
    return True


def _collect_items_at(pcbnew, board, positions, net_id, want_via, layer_name=None):
    """Collect tracks (or vias) of net_id with an endpoint at one of the positions."""
    if not positions:
        return []
    layer_id = board.GetLayerID(layer_name) if layer_name else None
    matches = []
    for item in board.GetTracks():
        is_via = item.Type() == pcbnew.PCB_VIA_T
        if is_via != want_via:
            continue
        if item.GetNetCode() != net_id:
            continue
        if is_via:
            keys = [_to_mm_key(pcbnew, item.GetPosition())]
        else:
            if layer_id is not None and item.GetLayer() != layer_id:
                continue
            keys = [_to_mm_key(pcbnew, item.GetStart()), _to_mm_key(pcbnew, item.GetEnd())]
        if any(k in positions for k in keys):
            matches.append(item)
    return matches


def _apply_target_swaps(pcbnew, board, target_swap_info):
    """Apply diff pair target swaps: stub segments/vias and pads between two pairs."""
    for swap in target_swap_info:
        p1_layer = swap.get('p1_layer')
        p2_layer = swap.get('p2_layer')

        # Collect all matches BEFORE mutating so the two swap directions
        # cannot double-swap the same item
        moves = []
        for positions, old_net, new_net, layer in (
                (swap['p1_p_positions'], swap['p1_p_net_id'], swap['p2_p_net_id'], p1_layer),
                (swap['p1_n_positions'], swap['p1_n_net_id'], swap['p2_n_net_id'], p1_layer),
                (swap['p2_p_positions'], swap['p2_p_net_id'], swap['p1_p_net_id'], p2_layer),
                (swap['p2_n_positions'], swap['p2_n_net_id'], swap['p1_n_net_id'], p2_layer)):
            for item in _collect_items_at(pcbnew, board, positions, old_net, want_via=False,
                                          layer_name=layer):
                moves.append((item, new_net))
            for item in _collect_items_at(pcbnew, board, positions, old_net, want_via=True):
                moves.append((item, new_net))
        for item, new_net in moves:
            item.SetNetCode(new_net)

        if swap.get('p1_p_pad') and swap.get('p2_p_pad'):
            _swap_pad_nets(board, swap['p1_p_pad'], swap['p2_p_pad'])
        if swap.get('p1_n_pad') and swap.get('p2_n_pad'):
            _swap_pad_nets(board, swap['p1_n_pad'], swap['p2_n_pad'])
        print(f"  Target swap applied: {swap.get('p1_name', '?')} <-> {swap.get('p2_name', '?')}"
              f" ({len(moves)} items)")


def _apply_single_ended_target_swaps(pcbnew, board, single_ended_target_swap_info):
    """Apply single-ended target swaps: stub segments/vias and pads between two nets."""
    for swap in single_ended_target_swap_info:
        moves = []
        for positions, old_net, new_net in (
                (swap['n1_positions'], swap['n1_net_id'], swap['n2_net_id']),
                (swap['n2_positions'], swap['n2_net_id'], swap['n1_net_id'])):
            for item in _collect_items_at(pcbnew, board, positions, old_net, want_via=False):
                moves.append((item, new_net))
            for item in _collect_items_at(pcbnew, board, positions, old_net, want_via=True):
                moves.append((item, new_net))
        for item, new_net in moves:
            item.SetNetCode(new_net)

        if swap.get('n1_pad') and swap.get('n2_pad'):
            _swap_pad_nets(board, swap['n1_pad'], swap['n2_pad'])
        print(f"  Target swap applied: {swap.get('n1_name', '?')} <-> {swap.get('n2_name', '?')}"
              f" ({len(moves)} items)")


def _apply_segment_layer_mods(pcbnew, board, segment_mods):
    """Apply stub layer-switch modifications to existing board tracks."""
    count = 0
    for mod in segment_mods:
        start_key = pos_key(mod['start'][0], mod['start'][1])
        end_key = pos_key(mod['end'][0], mod['end'][1])
        net_id = mod['net_id']
        old_layer_id = board.GetLayerID(mod['old_layer']) if mod.get('old_layer') else None
        new_layer_id = board.GetLayerID(mod['new_layer'])

        def matches(item, require_net):
            if item.Type() == pcbnew.PCB_VIA_T:
                return False
            if require_net and item.GetNetCode() != net_id:
                return False
            if old_layer_id is not None and item.GetLayer() != old_layer_id:
                return False
            s = _to_mm_key(pcbnew, item.GetStart())
            e = _to_mm_key(pcbnew, item.GetEnd())
            return (s, e) == (start_key, end_key) or (s, e) == (end_key, start_key)

        # Prefer a net-matched segment; fall back to coordinates only (net IDs
        # may have changed due to target/polarity swaps, matching file writer)
        target = next((t for t in board.GetTracks() if matches(t, require_net=True)), None)
        if target is None:
            target = next((t for t in board.GetTracks() if matches(t, require_net=False)), None)
        if target is not None:
            target.SetLayer(new_layer_id)
            count += 1
    if segment_mods:
        print(f"  Applied {count}/{len(segment_mods)} segment layer modifications")
    return count


def _apply_polarity_swaps(pcbnew, board, pad_swaps):
    """Apply polarity fixes: swap P/N pad nets and their stub segment/via nets."""
    for swap in pad_swaps:
        pad_p = swap['pad_p']
        pad_n = swap['pad_n']
        p_net_id = swap['p_net_id']
        n_net_id = swap['n_net_id']

        _swap_pad_nets(board, pad_p, pad_n)
        print(f"  Polarity swap applied: {pad_p.component_ref}:{pad_p.pad_number} <-> "
              f"{pad_n.component_ref}:{pad_n.pad_number}")

        p_positions = swap.get('p_stub_positions') or set()
        n_positions = swap.get('n_stub_positions') or set()

        # Collect both directions before mutating to avoid double-swapping
        moves = []
        for positions, old_net, new_net in ((p_positions, p_net_id, n_net_id),
                                            (n_positions, n_net_id, p_net_id)):
            for item in _collect_items_at(pcbnew, board, positions, old_net, want_via=False):
                moves.append((item, new_net))
            for item in _collect_items_at(pcbnew, board, positions, old_net, want_via=True):
                moves.append((item, new_net))
        for item, new_net in moves:
            item.SetNetCode(new_net)
        if moves:
            print(f"    Stubs: swapped {len(moves)} segments/vias")


def apply_swaps_to_board(board, results_data):
    """Apply target swaps, segment layer modifications, and polarity swaps to
    the live pcbnew board. Mirrors write_routed_output's order of operations."""
    import pcbnew

    target_swap_info = results_data.get('target_swap_info') or []
    single_ended_swaps = results_data.get('single_ended_target_swap_info') or []
    segment_mods = results_data.get('all_segment_modifications') or []
    pad_swaps = results_data.get('pad_swaps') or []

    if target_swap_info:
        _apply_target_swaps(pcbnew, board, target_swap_info)
    if single_ended_swaps:
        _apply_single_ended_target_swaps(pcbnew, board, single_ended_swaps)
    if segment_mods:
        _apply_segment_layer_mods(pcbnew, board, segment_mods)
    if pad_swaps:
        _apply_polarity_swaps(pcbnew, board, pad_swaps)

    return bool(target_swap_info or single_ended_swaps or segment_mods or pad_swaps)

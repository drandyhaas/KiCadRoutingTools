"""
Output file writer for PCB routing results.

This module handles writing the routed PCB output file, including:
- Target swaps (diff pair and single-ended)
- Layer modifications from stub switching
- Polarity fix pad swaps
- New segments and vias from routing
- Debug visualization paths
"""

from typing import List, Dict, Optional
from kicad_writer import (
    generate_segment_sexpr, generate_via_sexpr, generate_gr_line_sexpr,
    generate_gr_text_sexpr, swap_segment_nets_at_positions,
    swap_via_nets_at_positions, swap_pad_nets_in_content, modify_segment_layers,
    move_copper_text_to_silkscreen
)
from connectivity import find_connected_segment_positions


def write_routed_output(
    input_file: str,
    output_file: str,
    results: List[Dict],
    all_segment_modifications: List,
    all_swap_vias: List,
    target_swap_info: List[Dict],
    single_ended_target_swap_info: List[Dict],
    pad_swaps: List[Dict],
    pcb_data,
    debug_lines: bool = False,
    exclusion_zone_lines: List = None,
    boundary_debug_labels: List = None,
    skip_routing: bool = False
) -> bool:
    """
    Write the routed PCB output file.

    Args:
        input_file: Path to input KiCad PCB file
        output_file: Path to output KiCad PCB file
        results: List of routing results with new_segments and new_vias
        all_segment_modifications: Layer modifications from stub switching
        all_swap_vias: Vias added during stub layer swapping
        target_swap_info: Diff pair target swap information
        single_ended_target_swap_info: Single-ended target swap information
        pad_swaps: Polarity fix pad swap information
        pcb_data: PCB data structure (for fallback position lookups)
        debug_lines: Whether to add debug visualization
        exclusion_zone_lines: Debug lines for exclusion zones
        boundary_debug_labels: Debug labels for boundary positions
        skip_routing: Whether routing was skipped (debug only mode)

    Returns:
        True if output was written successfully
    """
    # Check if we have anything to write
    if not (results or all_segment_modifications or all_swap_vias or
            target_swap_info or single_ended_target_swap_info or skip_routing):
        return False

    print(f"\nWriting output to {output_file}...")
    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()

    # Move text from copper layers to silkscreen (prevents routing interference)
    content = move_copper_text_to_silkscreen(content)

    # Apply target swaps FIRST - layer modifications were recorded with post-swap net IDs,
    # so we need to swap the file content to match before applying layer modifications
    content = _apply_diff_pair_target_swaps(content, target_swap_info)
    content = _apply_single_ended_target_swaps(content, single_ended_target_swap_info)

    # Apply segment layer modifications from stub layer switching AFTER target swaps
    # (layer mods were recorded with post-swap net IDs, so file must be swapped first)
    if all_segment_modifications:
        content, mod_count = modify_segment_layers(content, all_segment_modifications)
        print(f"Applied {mod_count} segment layer modifications (layer switching)")

    # Apply pad and stub net swaps for polarity fixes
    content = _apply_polarity_swaps(content, pad_swaps, pcb_data)

    # Generate routing text (new segments and vias)
    routing_text = _generate_routing_text(results, all_swap_vias)

    # Add debug paths if enabled
    if debug_lines:
        routing_text += _generate_debug_paths(
            results, exclusion_zone_lines or [], boundary_debug_labels or []
        )

    # Insert routing text before final closing paren
    last_paren = content.rfind(')')
    new_content = content[:last_paren] + '\n' + routing_text + '\n' + content[last_paren:]

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(new_content)

    print(f"Successfully wrote {output_file}")
    return True


def _apply_diff_pair_target_swaps(content: str, target_swap_info: List[Dict]) -> str:
    """Apply diff pair target swaps to file content."""
    if not target_swap_info:
        return content

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

    return content


def _apply_single_ended_target_swaps(content: str, single_ended_target_swap_info: List[Dict]) -> str:
    """Apply single-ended target swaps to file content."""
    if not single_ended_target_swap_info:
        return content

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

    return content


def _apply_polarity_swaps(content: str, pad_swaps: List[Dict], pcb_data) -> str:
    """Apply polarity fix pad and stub swaps to file content."""
    if not pad_swaps:
        return content

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

    return content


def _generate_routing_text(results: List[Dict], all_swap_vias: List) -> str:
    """Generate routing text for new segments and vias."""
    routing_text = ""

    # Add segments and vias from routing results
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

    return routing_text


def _generate_debug_paths(
    results: List[Dict],
    exclusion_zone_lines: List,
    boundary_debug_labels: List
) -> str:
    """Generate debug visualization paths."""
    routing_text = ""

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

    return routing_text

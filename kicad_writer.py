"""
KiCad PCB Writer - Writes routing results to .kicad_pcb files.
"""

import re
import uuid
from typing import List, Dict, Tuple, Optional

from kicad_parser import Pad


def generate_segment_sexpr(start: Tuple[float, float], end: Tuple[float, float],
                           width: float, layer: str, net_id: int) -> str:
    """Generate KiCad S-expression for a track segment."""
    return f'''	(segment
		(start {start[0]:.6f} {start[1]:.6f})
		(end {end[0]:.6f} {end[1]:.6f})
		(width {width})
		(layer "{layer}")
		(net {net_id})
		(uuid "{uuid.uuid4()}")
	)'''


def generate_gr_line_sexpr(start: Tuple[float, float], end: Tuple[float, float],
                           width: float, layer: str) -> str:
    """Generate KiCad S-expression for a graphic line (for non-copper layers)."""
    return f'''	(gr_line
		(start {start[0]:.6f} {start[1]:.6f})
		(end {end[0]:.6f} {end[1]:.6f})
		(stroke
			(width {width})
			(type solid)
		)
		(layer "{layer}")
		(uuid "{uuid.uuid4()}")
	)'''


def generate_via_sexpr(x: float, y: float, size: float, drill: float,
                       layers: List[str], net_id: int) -> str:
    """Generate KiCad S-expression for a via."""
    layers_str = '" "'.join(layers)
    return f'''	(via
		(at {x:.6f} {y:.6f})
		(size {size})
		(drill {drill})
		(layers "{layers_str}")
		(net {net_id})
		(uuid "{uuid.uuid4()}")
	)'''


def add_tracks_to_pcb(input_path: str, output_path: str, tracks: List[Dict]) -> bool:
    """
    Add track segments to a PCB file.

    Args:
        input_path: Path to original .kicad_pcb file
        output_path: Path for output file
        tracks: List of track dicts with keys: start, end, width, layer, net_id

    Returns:
        True if successful
    """
    with open(input_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Generate segment S-expressions
    segments = []
    for track in tracks:
        seg = generate_segment_sexpr(
            track['start'],
            track['end'],
            track['width'],
            track['layer'],
            track['net_id']
        )
        segments.append(seg)

    routing_text = '\n'.join(segments)

    if not routing_text.strip():
        print("Warning: No routing elements to add")
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(content)
        return True

    # Find the last closing parenthesis
    last_paren = content.rfind(')')

    if last_paren == -1:
        print("Error: Could not find closing parenthesis in PCB file")
        return False

    # Insert routing before the final closing paren
    new_content = content[:last_paren] + '\n' + routing_text + '\n' + content[last_paren:]

    # Write output file
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(new_content)

    return True


def add_tracks_and_vias_to_pcb(input_path: str, output_path: str,
                               tracks: List[Dict], vias: List[Dict] = None,
                               remove_vias: List[Dict] = None) -> bool:
    """
    Add track segments and vias to a PCB file, optionally removing existing vias.

    Args:
        input_path: Path to original .kicad_pcb file
        output_path: Path for output file
        tracks: List of track dicts with keys: start, end, width, layer, net_id
        vias: List of via dicts with keys: x, y, size, drill, layers, net_id
        remove_vias: List of via dicts with keys: x, y (position to match for removal)

    Returns:
        True if successful
    """
    with open(input_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Remove existing vias if specified
    if remove_vias:
        import re
        removed_count = 0
        for via_to_remove in remove_vias:
            x, y = via_to_remove['x'], via_to_remove['y']
            # Match via at this position
            # KiCad format: (via\n\t\t(at X Y)\n\t\t(size ...)\n\t\t(drill ...)\n\t\t(layers ...)\n\t\t(net ...)\n\t\t(uuid ...)\n\t)
            # Build pattern that matches the multi-line via block
            x_str = f"{x:.6f}".rstrip('0').rstrip('.')
            y_str = f"{y:.6f}".rstrip('0').rstrip('.')

            # Also try integer format if coordinates are whole numbers
            x_patterns = [re.escape(x_str)]
            y_patterns = [re.escape(y_str)]
            if x == int(x):
                x_patterns.append(str(int(x)))
            if y == int(y):
                y_patterns.append(str(int(y)))

            # Build pattern that matches via block with any of the coordinate formats
            for x_pat in x_patterns:
                for y_pat in y_patterns:
                    # Match entire via block from opening to closing parenthesis
                    # Use non-greedy match for content between (at ...) and final )
                    pattern = rf'\t\(via\s*\n\s*\(at\s+{x_pat}\s+{y_pat}\)[\s\S]*?\n\t\)'
                    new_content = re.sub(pattern, '', content)
                    if new_content != content:
                        content = new_content
                        removed_count += 1
                        break
                else:
                    continue
                break
        if removed_count > 0:
            print(f"  Removed {removed_count} vias from file")

    elements = []

    # Generate segment S-expressions
    for track in tracks:
        seg = generate_segment_sexpr(
            track['start'],
            track['end'],
            track['width'],
            track['layer'],
            track['net_id']
        )
        elements.append(seg)

    # Generate via S-expressions
    if vias:
        for via in vias:
            v = generate_via_sexpr(
                via['x'],
                via['y'],
                via['size'],
                via['drill'],
                via['layers'],
                via['net_id']
            )
            elements.append(v)

    routing_text = '\n'.join(elements)

    if not routing_text.strip():
        print("Warning: No routing elements to add")
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(content)
        return True

    # Find the last closing parenthesis
    last_paren = content.rfind(')')

    if last_paren == -1:
        print("Error: Could not find closing parenthesis in PCB file")
        return False

    # Insert routing before the final closing paren
    new_content = content[:last_paren] + '\n' + routing_text + '\n' + content[last_paren:]

    # Write output file
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(new_content)

    return True


def swap_segment_nets_at_positions(content: str, positions: set,
                                   old_net_id: int, new_net_id: int) -> Tuple[str, int]:
    """
    Swap net IDs of segments that have endpoints at the given positions.

    Returns (modified_content, count_of_swapped_segments).
    """
    segment_pattern = r'\(segment\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\).*?\(net\s+(\d+)\)'

    count = 0

    def pos_key(x, y):
        return (round(x, 2), round(y, 2))

    def replace_net(match):
        nonlocal count
        start_x, start_y = float(match.group(1)), float(match.group(2))
        end_x, end_y = float(match.group(3)), float(match.group(4))
        seg_net_id = int(match.group(5))

        start_key = pos_key(start_x, start_y)
        end_key = pos_key(end_x, end_y)

        # Check if this segment has endpoints in our position set and correct net ID
        if seg_net_id == old_net_id and (start_key in positions or end_key in positions):
            count += 1
            return match.group(0).replace(f'(net {old_net_id})', f'(net {new_net_id})')
        return match.group(0)

    result = re.sub(segment_pattern, replace_net, content, flags=re.DOTALL)
    return result, count


def swap_via_nets_at_positions(content: str, positions: set,
                               old_net_id: int, new_net_id: int) -> Tuple[str, int]:
    """
    Swap net IDs of vias that are at the given positions.

    Returns (modified_content, count_of_swapped_vias).
    """
    via_pattern = r'\(via\s+\(at\s+([\d.-]+)\s+([\d.-]+)\).*?\(net\s+(\d+)\)'

    count = 0

    def pos_key(x, y):
        return (round(x, 2), round(y, 2))

    def replace_net(match):
        nonlocal count
        via_x, via_y = float(match.group(1)), float(match.group(2))
        via_net_id = int(match.group(3))

        via_key = pos_key(via_x, via_y)

        # Check if this via is at one of our positions and has the correct net ID
        if via_net_id == old_net_id and via_key in positions:
            count += 1
            return match.group(0).replace(f'(net {old_net_id})', f'(net {new_net_id})')
        return match.group(0)

    result = re.sub(via_pattern, replace_net, content, flags=re.DOTALL)
    return result, count


def swap_pad_nets_in_content(content: str, pad1: Pad, pad2: Pad) -> str:
    """
    Swap the net assignments of two pads in the KiCad PCB file content.

    Finds each pad by its component reference and pad number, then swaps their (net ...) declarations.
    """
    def find_pad_net_in_footprint(content: str, component_ref: str, pad_number: str) -> Optional[Tuple[int, int, str]]:
        """Find the (net id "name") part of a pad and return (start, end, match_text)."""
        fp_start_pattern = r'\(footprint\s+"[^"]*"'

        for fp_match in re.finditer(fp_start_pattern, content):
            fp_start = fp_match.start()
            # Find the end of this footprint block
            depth = 0
            fp_end = fp_start
            for i, char in enumerate(content[fp_start:], fp_start):
                if char == '(':
                    depth += 1
                elif char == ')':
                    depth -= 1
                    if depth == 0:
                        fp_end = i + 1
                        break

            fp_text = content[fp_start:fp_end]

            # Check if this footprint has the right Reference
            ref_pattern = rf'\(property\s+"Reference"\s+"{re.escape(component_ref)}"'
            if not re.search(ref_pattern, fp_text):
                continue

            # Find the pad with this number in this footprint
            pad_pattern = rf'\(pad\s+"{re.escape(pad_number)}"\s+\w+\s+\w+'
            for pad_match in re.finditer(pad_pattern, fp_text):
                pad_start_rel = pad_match.start()
                # Find end of this pad block
                depth = 0
                pad_end_rel = pad_start_rel
                for i, char in enumerate(fp_text[pad_start_rel:], pad_start_rel):
                    if char == '(':
                        depth += 1
                    elif char == ')':
                        depth -= 1
                        if depth == 0:
                            pad_end_rel = i + 1
                            break

                pad_text = fp_text[pad_start_rel:pad_end_rel]

                # Find the (net id "name") in this pad
                net_match = re.search(r'\(net\s+\d+\s+"[^"]*"\)', pad_text)
                if net_match:
                    # Convert to absolute positions in content
                    abs_start = fp_start + pad_start_rel + net_match.start()
                    abs_end = fp_start + pad_start_rel + net_match.end()
                    return (abs_start, abs_end, net_match.group())

        return None

    # Find both pads' net declarations
    result1 = find_pad_net_in_footprint(content, pad1.component_ref, pad1.pad_number)
    result2 = find_pad_net_in_footprint(content, pad2.component_ref, pad2.pad_number)

    if not result1 or not result2:
        print(f"  WARNING: Could not find pad(s) to swap nets")
        if not result1:
            print(f"    Missing: {pad1.component_ref} pad {pad1.pad_number}")
        if not result2:
            print(f"    Missing: {pad2.component_ref} pad {pad2.pad_number}")
        return content

    start1, end1, net1_text = result1
    start2, end2, net2_text = result2

    # Swap the net declarations (replace higher position first to preserve indices)
    if start1 < start2:
        content = content[:start2] + net1_text + content[end2:]
        content = content[:start1] + net2_text + content[end1:]
    else:
        content = content[:start1] + net2_text + content[end1:]
        content = content[:start2] + net1_text + content[end2:]

    return content

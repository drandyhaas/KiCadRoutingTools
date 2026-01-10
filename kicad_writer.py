"""
KiCad PCB Writer - Writes routing results to .kicad_pcb files.
"""

import re
import uuid
from typing import List, Dict, Tuple, Optional

from kicad_parser import Pad
from routing_utils import pos_key, POSITION_DECIMALS


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
                       layers: List[str], net_id: int, free: bool = False) -> str:
    """Generate KiCad S-expression for a via.

    Args:
        free: If True, adds (free yes) to prevent KiCad from auto-assigning net based on overlapping tracks.
    """
    layers_str = '" "'.join(layers)
    free_str = "\n\t\t(free yes)" if free else ""
    return f'''	(via
		(at {x:.6f} {y:.6f})
		(size {size})
		(drill {drill})
		(layers "{layers_str}"){free_str}
		(net {net_id})
		(uuid "{uuid.uuid4()}")
	)'''


def generate_gr_text_sexpr(text: str, x: float, y: float, layer: str,
                           size: float = 0.5, angle: float = 0) -> str:
    """Generate KiCad S-expression for a graphic text label."""
    return f'''	(gr_text "{text}"
		(at {x:.6f} {y:.6f} {angle})
		(layer "{layer}")
		(uuid "{uuid.uuid4()}")
		(effects
			(font
				(size {size} {size})
				(thickness 0.1)
			)
		)
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
                via['net_id'],
                via.get('free', False)
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


def modify_segment_layers(content: str, segment_mods: List[Dict]) -> Tuple[str, int]:
    """
    Modify the layer of existing segments in the KiCad PCB content.

    Args:
        content: KiCad PCB file content
        segment_mods: List of dicts with keys:
            - start: (x, y) tuple
            - end: (x, y) tuple
            - net_id: int
            - old_layer: str (optional, for verification)
            - new_layer: str

    Returns:
        (modified_content, count_of_modified_segments)
    """
    if not segment_mods:
        return content, 0

    # Build a lookup for segment modifications
    def coord_key(x, y):
        return (round(x, POSITION_DECIMALS), round(y, POSITION_DECIMALS))

    mod_lookup = {}
    # Secondary lookup by coordinates only (for fallback when net_id doesn't match due to swaps)
    mod_lookup_by_coords = {}
    for mod in segment_mods:
        start_key = coord_key(mod['start'][0], mod['start'][1])
        end_key = coord_key(mod['end'][0], mod['end'][1])
        key = (start_key, end_key, mod['net_id'])
        # Also store reverse order since segment endpoints can be swapped
        key_rev = (end_key, start_key, mod['net_id'])
        mod_lookup[key] = mod
        mod_lookup[key_rev] = mod
        # Coordinate-only lookup (list to handle multiple mods at same coords)
        coord_only_key = (start_key, end_key)
        coord_only_key_rev = (end_key, start_key)
        if coord_only_key not in mod_lookup_by_coords:
            mod_lookup_by_coords[coord_only_key] = []
        mod_lookup_by_coords[coord_only_key].append(mod)
        if coord_only_key_rev not in mod_lookup_by_coords:
            mod_lookup_by_coords[coord_only_key_rev] = []
        mod_lookup_by_coords[coord_only_key_rev].append(mod)

    count = 0

    # Pattern to match segment blocks
    segment_pattern = re.compile(
        r'(\(segment\s*\n?\s*'
        r'\(start\s+([\d.-]+)\s+([\d.-]+)\)\s*\n?\s*'
        r'\(end\s+([\d.-]+)\s+([\d.-]+)\)\s*\n?\s*'
        r'\(width\s+[\d.]+\)\s*\n?\s*'
        r'\(layer\s+")([^"]+)("\)\s*\n?\s*'
        r'\(net\s+(\d+)\))',
        re.MULTILINE
    )

    def replace_layer(match):
        nonlocal count
        full_match = match.group(0)
        start_x = float(match.group(2))
        start_y = float(match.group(3))
        end_x = float(match.group(4))
        end_y = float(match.group(5))
        layer = match.group(6)
        net_id = int(match.group(8))

        start_key = coord_key(start_x, start_y)
        end_key = coord_key(end_x, end_y)
        key = (start_key, end_key, net_id)
        coord_only_key = (start_key, end_key)

        mod = None
        # First try exact match by (coords, net_id)
        if key in mod_lookup:
            mod = mod_lookup[key]
        # Fallback: try coordinate-only match
        # This handles cases where net_id changed due to target swaps
        # Only use fallback if the segment's current layer is one of the expected old_layers
        elif coord_only_key in mod_lookup_by_coords:
            mods_at_coords = mod_lookup_by_coords[coord_only_key]
            # Check if current layer matches any old_layer in the chain
            if any(m.get('old_layer') == layer for m in mods_at_coords):
                # Use the last mod (final target layer for chained swaps)
                mod = mods_at_coords[-1]

        if mod:
            new_layer = mod['new_layer']
            if layer != new_layer:
                count += 1
                # Replace the layer in the match
                return full_match.replace(f'(layer "{layer}")', f'(layer "{new_layer}")')
        return full_match

    result = segment_pattern.sub(replace_layer, content)
    return result, count


def swap_segment_nets_at_positions(content: str, positions: set,
                                   old_net_id: int, new_net_id: int,
                                   layer: str = None) -> Tuple[str, int]:
    """
    Swap net IDs of segments that have endpoints at the given positions.

    Args:
        content: KiCad file content
        positions: Set of position keys to match
        old_net_id: Net ID to replace
        new_net_id: Net ID to replace with
        layer: Optional layer name to filter segments. When specified, only segments
               on this layer are swapped. This prevents incorrectly swapping
               stubs that share XY coordinates but are on different layers.

    Returns (modified_content, count_of_swapped_segments).
    """
    # Pattern now captures layer as well
    segment_pattern = r'\(segment\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)\s+\(width[^)]*\)\s+\(layer\s+"?([^")]+)"?\).*?\(net\s+(\d+)\)'

    count = 0

    def replace_net(match):
        nonlocal count
        start_x, start_y = float(match.group(1)), float(match.group(2))
        end_x, end_y = float(match.group(3)), float(match.group(4))
        seg_layer = match.group(5)
        seg_net_id = int(match.group(6))

        start_key = pos_key(start_x, start_y)
        end_key = pos_key(end_x, end_y)

        # Check if this segment has endpoints in our position set and correct net ID
        # Also filter by layer if specified
        if seg_net_id == old_net_id and (start_key in positions or end_key in positions):
            if layer is None or seg_layer == layer:
                count += 1
                return match.group(0).replace(f'(net {old_net_id})', f'(net {new_net_id})')
        return match.group(0)

    result = re.sub(segment_pattern, replace_net, content, flags=re.DOTALL)
    return result, count


def swap_via_nets_at_positions(content: str, positions: set,
                               old_net_id: int, new_net_id: int,
                               tolerance: float = 0.02) -> Tuple[str, int]:
    """
    Swap net IDs of vias that are at the given positions.

    Uses tolerance-based matching to handle slight coordinate differences
    between vias and segment endpoints (e.g., original vias may be slightly
    off-grid from routed segments).

    Returns (modified_content, count_of_swapped_vias).
    """
    via_pattern = r'\(via\s+\(at\s+([\d.-]+)\s+([\d.-]+)\).*?\(net\s+(\d+)\)'

    count = 0

    def is_near_any_position(x, y, positions, tol):
        """Check if (x, y) is within tolerance of any position in the set."""
        for px, py in positions:
            if abs(x - px) < tol and abs(y - py) < tol:
                return True
        return False

    def replace_net(match):
        nonlocal count
        via_x, via_y = float(match.group(1)), float(match.group(2))
        via_net_id = int(match.group(3))

        # Check if this via is near one of our positions and has the correct net ID
        if via_net_id == old_net_id and is_near_any_position(via_x, via_y, positions, tolerance):
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

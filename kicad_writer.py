"""
KiCad PCB Writer - Writes routing results to .kicad_pcb files.
"""

import uuid
from typing import List, Dict, Tuple


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

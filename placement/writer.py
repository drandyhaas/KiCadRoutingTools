"""
Write placed PCB output by modifying footprint positions in the .kicad_pcb file.

Uses text-based manipulation (same approach as output_writer.py / kicad_writer.py)
to update the (at X Y [rotation]) of each footprint block.
"""
from __future__ import annotations

import re
from typing import List, Dict

from kicad_writer import move_copper_text_to_silkscreen


def _rotate_pad_angles(fp_text: str, delta_rot: float) -> str:
    """Add delta_rot to the (at x y [angle]) of every pad in a footprint block."""

    def fix_pad(m):
        head, x, y, angle = m.group(1), m.group(2), m.group(3), m.group(4)
        new_angle = ((float(angle) if angle else 0.0) + delta_rot) % 360
        if new_angle != 0:
            return f'{head}(at {x} {y} {new_angle:.4g})'
        return f'{head}(at {x} {y})'

    return re.sub(
        r'(\(pad\s+"[^"]*"\s+\S+\s+\S+\s*\n?\s*)'
        r'\(at\s+([\d.-]+)\s+([\d.-]+)(?:\s+([\d.-]+))?\)',
        fix_pad, fp_text)


def write_placed_output(input_file: str, output_file: str,
                        placements: List[Dict],
                        via_moves: List = None,
                        new_segments: List = None,
                        pcb_data=None) -> bool:
    """
    Write a placed PCB file by modifying footprint positions.

    Args:
        input_file: Path to input KiCad PCB file
        output_file: Path to output KiCad PCB file
        placements: List of dicts with keys:
            - reference: str (e.g. "U1")
            - new_x: float (mm)
            - new_y: float (mm)
            - new_rotation: float (degrees)

    Returns:
        True if output was written successfully
    """
    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()

    # Move text from copper layers to silkscreen (prevents routing interference)
    content = move_copper_text_to_silkscreen(content)

    placement_by_ref = {p['reference']: p for p in placements}
    modified_count = 0

    # Find all footprint blocks and modify their (at ...) lines
    footprint_starts = [m.start() for m in re.finditer(r'\(footprint\s+"', content)]

    # Process in reverse order so string indices remain valid after replacements
    for start in reversed(footprint_starts):
        # Find the matching end parenthesis
        depth = 0
        end = start
        for i in range(start, len(content)):
            if content[i] == '(':
                depth += 1
            elif content[i] == ')':
                depth -= 1
                if depth == 0:
                    end = i + 1
                    break

        fp_text = content[start:end]

        # Extract reference from (property "Reference" "XX" ...)
        ref_match = re.search(
            r'\(property\s+"Reference"\s+"([^"]+)"', fp_text)
        if not ref_match:
            continue
        ref = ref_match.group(1)

        if ref not in placement_by_ref:
            continue

        placement = placement_by_ref[ref]

        # Find the footprint's (at X Y [rotation]) - it's the first (at ...) in the block
        at_match = re.search(r'\(at\s+([\d.-]+)\s+([\d.-]+)(?:\s+([\d.-]+))?\)',
                             fp_text)
        if not at_match:
            continue

        # Build replacement (at ...) string
        new_x = placement['new_x']
        new_y = placement['new_y']
        new_rot = placement['new_rotation']
        old_rot = float(at_match.group(3)) if at_match.group(3) else 0.0

        if new_rot != 0:
            new_at = f"(at {new_x:.4g} {new_y:.4g} {new_rot:.4g})"
        else:
            new_at = f"(at {new_x:.4g} {new_y:.4g})"

        new_fp_text = fp_text[:at_match.start()] + new_at + fp_text[at_match.end():]

        # KiCad stores pad angles as footprint rotation + pad-local rotation,
        # so a footprint rotation change must be added to every pad angle.
        delta_rot = (new_rot - old_rot) % 360
        if delta_rot != 0:
            new_fp_text = _rotate_pad_angles(new_fp_text, delta_rot)

        content = content[:start] + new_fp_text + content[end:]
        modified_count += 1

    # Via-nudge rewrites (#313): remove each moved via from the input text and
    # append it at its new position, plus the new connector segments back to
    # the fanout stub start. Attached segments are untouched by design.
    if via_moves or new_segments:
        from plane_io import _remove_vias_at_positions
        from kicad_writer import generate_via_sexpr, generate_segment_sexpr
        n2n = getattr(pcb_data, 'net_id_to_name', None) if pcb_data is not None else None
        elements = []
        if via_moves:
            content, _ = _remove_vias_at_positions(
                content, [(m[0], m[1]) for m in via_moves])
            for m in via_moves:
                v = m[2]
                nm = n2n.get(v['net_id']) if n2n else None
                elements.append(generate_via_sexpr(v['x'], v['y'], v['size'],
                                                   v['drill'], v['layers'],
                                                   v['net_id'], net_name=nm))
        for nsd in (new_segments or []):
            nm = n2n.get(nsd['net_id']) if n2n else None
            elements.append(generate_segment_sexpr(
                nsd['start'], nsd['end'], nsd['width'], nsd['layer'],
                nsd['net_id'], net_name=nm))
        if elements:
            close = content.rindex(')')
            content = content[:close] + '\n'.join(elements) + '\n)' + content[close+1:]

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(content)

    print(f"Modified {modified_count} footprint positions")
    print(f"Successfully wrote {output_file}")
    return True

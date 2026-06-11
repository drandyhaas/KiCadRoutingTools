"""
Write placed PCB output by modifying footprint positions in the .kicad_pcb file.

Uses text-based manipulation (same approach as output_writer.py / kicad_writer.py)
to update the (at X Y [rotation]) of each footprint block.
"""

import re
from typing import List, Dict

from kicad_writer import move_copper_text_to_silkscreen


def write_placed_output(input_file: str, output_file: str,
                        placements: List[Dict]) -> bool:
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

        if new_rot != 0:
            new_at = f"(at {new_x:.4g} {new_y:.4g} {new_rot:.4g})"
        else:
            new_at = f"(at {new_x:.4g} {new_y:.4g})"

        # Replace in the footprint block
        old_at = at_match.group(0)
        new_fp_text = fp_text[:at_match.start()] + new_at + fp_text[at_match.end():]

        content = content[:start] + new_fp_text + content[end:]
        modified_count += 1

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(content)

    print(f"Modified {modified_count} footprint positions")
    print(f"Successfully wrote {output_file}")
    return True

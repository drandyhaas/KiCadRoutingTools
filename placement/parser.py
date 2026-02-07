"""
Parse placement-relevant geometry from KiCad PCB files.

Extracts courtyard boundaries and other footprint geometry that
kicad_parser doesn't provide, for use by the placement tool.
"""

import re
from typing import Dict, Set, Tuple


def extract_locked_refs(pcb_file: str) -> Set[str]:
    """
    Find all footprints marked as locked in the PCB file.

    Returns set of component references (e.g., {"P1", "J1"}).
    """
    with open(pcb_file, 'r', encoding='utf-8') as f:
        content = f.read()

    locked = set()
    footprint_starts = [m.start() for m in re.finditer(r'\(footprint\s+"', content)]

    for start in footprint_starts:
        # Find matching end paren
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

        # Check for (locked yes) before the first pad
        # It appears early in the footprint block, before properties
        first_pad = fp_text.find('(pad ')
        search_region = fp_text[:first_pad] if first_pad > 0 else fp_text[:500]
        if re.search(r'\(locked\s+yes\)', search_region):
            ref_match = re.search(r'\(property\s+"Reference"\s+"([^"]+)"', fp_text)
            if ref_match:
                locked.add(ref_match.group(1))

    return locked


def extract_courtyard_bboxes(pcb_file: str) -> Dict[str, Tuple[float, float, float, float]]:
    """
    Parse courtyard (F.CrtYd / B.CrtYd) extents from each footprint block.

    Returns dict mapping component reference to (local_min_x, local_min_y,
    local_max_x, local_max_y) — the courtyard bounding box in the footprint's
    local coordinate system. These must be transformed to global coordinates
    using the footprint's position and rotation.
    """
    with open(pcb_file, 'r', encoding='utf-8') as f:
        content = f.read()

    result = {}
    footprint_starts = [m.start() for m in re.finditer(r'\(footprint\s+"', content)]

    for start in footprint_starts:
        # Find matching end paren
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

        # Get reference
        ref_match = re.search(r'\(property\s+"Reference"\s+"([^"]+)"', fp_text)
        if not ref_match:
            continue
        ref = ref_match.group(1)

        # Find all fp_line segments on CrtYd layers and collect local coordinate extents
        min_x = float('inf')
        max_x = float('-inf')
        min_y = float('inf')
        max_y = float('-inf')
        found_crtyd = False

        # Match fp_line with start/end coords followed by CrtYd layer
        for m in re.finditer(
            r'\(fp_line\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\).*?\(layer\s+"[FB]\.CrtYd"\)',
            fp_text, re.DOTALL
        ):
            x1, y1 = float(m.group(1)), float(m.group(2))
            x2, y2 = float(m.group(3)), float(m.group(4))
            min_x = min(min_x, x1, x2)
            max_x = max(max_x, x1, x2)
            min_y = min(min_y, y1, y2)
            max_y = max(max_y, y1, y2)
            found_crtyd = True

        # Also check fp_rect on CrtYd
        for m in re.finditer(
            r'\(fp_rect\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\).*?\(layer\s+"[FB]\.CrtYd"\)',
            fp_text, re.DOTALL
        ):
            x1, y1 = float(m.group(1)), float(m.group(2))
            x2, y2 = float(m.group(3)), float(m.group(4))
            min_x = min(min_x, x1, x2)
            max_x = max(max_x, x1, x2)
            min_y = min(min_y, y1, y2)
            max_y = max(max_y, y1, y2)
            found_crtyd = True

        if found_crtyd:
            result[ref] = (min_x, min_y, max_x, max_y)

    return result

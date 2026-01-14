"""
I/O utilities for copper plane generation.

Handles reading zone information from PCB files and writing plane output.
"""

import re
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional

from kicad_parser import PCBData
from kicad_writer import generate_via_sexpr, generate_segment_sexpr


@dataclass
class ZoneInfo:
    """Information about a copper zone/pour."""
    net_id: int
    net_name: str
    layer: str


def extract_zones(pcb_file: str) -> List[ZoneInfo]:
    """Extract zone information from a KiCad PCB file.

    Args:
        pcb_file: Path to the .kicad_pcb file

    Returns:
        List of ZoneInfo objects for each zone found
    """
    with open(pcb_file, 'r', encoding='utf-8') as f:
        content = f.read()

    zones = []
    # Pattern to match zone blocks and extract net_id, net_name, and layer
    zone_pattern = r'\(zone\s+\(net\s+(\d+)\)\s+\(net_name\s+"([^"]+)"\)\s+\(layer\s+"([^"]+)"\)'

    for match in re.finditer(zone_pattern, content):
        zones.append(ZoneInfo(
            net_id=int(match.group(1)),
            net_name=match.group(2),
            layer=match.group(3)
        ))

    return zones


def check_existing_zones(zones: List[ZoneInfo], target_layer: str, target_net_name: str,
                          target_net_id: int, verbose: bool = False) -> Tuple[bool, bool]:
    """Check for existing zones on the target layer.

    Args:
        zones: List of existing zones
        target_layer: Layer we want to create zone on
        target_net_name: Net name we want
        target_net_id: Net ID we want
        verbose: Print verbose output

    Returns:
        (should_create_zone, should_continue) tuple
        - should_create_zone: False if zone already exists on same net
        - should_continue: False if zone exists on different net (error condition)
    """
    for zone in zones:
        if zone.layer == target_layer:
            if zone.net_id == target_net_id or zone.net_name == target_net_name:
                # Same net - warn and skip zone creation but continue with vias
                print(f"Warning: Zone already exists on {target_layer} for net '{zone.net_name}' (ID {zone.net_id})")
                print("  Skipping zone creation, but will place vias to connect to existing zone.")
                return (False, True)
            else:
                # Different net - error
                print(f"Error: Zone already exists on {target_layer} for DIFFERENT net '{zone.net_name}' (ID {zone.net_id})")
                print(f"  Cannot create {target_net_name} zone on same layer. Aborting.")
                return (False, False)

    # No existing zone on this layer
    return (True, True)


def resolve_net_id(pcb_data: PCBData, net_name: str) -> Optional[int]:
    """Resolve a net name to its ID.

    Args:
        pcb_data: Parsed PCB data
        net_name: Net name to look up

    Returns:
        Net ID if found, None otherwise
    """
    # Check in pcb.nets
    for net in pcb_data.nets.values():
        if net.name == net_name:
            return net.net_id

    # Check in pads_by_net (some nets may only appear in pad definitions)
    for net_id, pads in pcb_data.pads_by_net.items():
        for pad in pads:
            if pad.net_name == net_name:
                return net_id

    return None


def filter_nets_from_content(content: str, net_ids_to_exclude: List[int]) -> str:
    """
    Filter out segments and vias for specific net IDs from PCB file content.

    Args:
        content: Raw PCB file content
        net_ids_to_exclude: List of net IDs to remove

    Returns:
        Filtered content with those nets' segments and vias removed
    """
    if not net_ids_to_exclude:
        return content

    net_id_set = set(net_ids_to_exclude)
    lines = content.split('\n')
    result_lines = []

    i = 0
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()

        # Check if this starts a segment or via (may be multi-line)
        if stripped == '(segment' or stripped.startswith('(segment ') or \
           stripped == '(via' or stripped.startswith('(via '):
            # Collect all lines of this element
            element_lines = [line]
            open_parens = line.count('(') - line.count(')')
            while open_parens > 0 and i + 1 < len(lines):
                i += 1
                element_lines.append(lines[i])
                open_parens += lines[i].count('(') - lines[i].count(')')

            # Check if any line contains net ID to exclude
            element_text = '\n'.join(element_lines)
            net_match = re.search(r'\(net\s+(\d+)\)', element_text)
            if net_match:
                element_net_id = int(net_match.group(1))
                if element_net_id in net_id_set:
                    # Skip this element entirely
                    i += 1
                    continue

            # Keep this element
            result_lines.extend(element_lines)
        else:
            result_lines.append(line)

        i += 1

    return '\n'.join(result_lines)


def write_plane_output(
    input_file: str,
    output_file: str,
    zone_sexpr: Optional[str],
    new_vias: List[Dict],
    new_segments: List[Dict],
    exclude_net_ids: List[int] = None
) -> bool:
    """Write the complete output file with zone (optional), vias, and traces.

    Args:
        input_file: Path to input PCB file
        output_file: Path to output PCB file
        zone_sexpr: Zone S-expression to add (or None)
        new_vias: List of via dicts with x, y, size, drill, layers, net_id
        new_segments: List of segment dicts with start, end, width, layer, net_id
        exclude_net_ids: Optional list of net IDs to exclude from output
                         (their segments/vias will be filtered out)

    Returns:
        True if successful, False otherwise
    """
    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()

    # Filter out excluded nets if specified
    if exclude_net_ids:
        content = filter_nets_from_content(content, exclude_net_ids)

    # Build routing text
    elements = []

    # Add zone if provided
    if zone_sexpr:
        elements.append(zone_sexpr)

    # Add vias
    for via in new_vias:
        elements.append(generate_via_sexpr(
            via['x'], via['y'], via['size'], via['drill'],
            via['layers'], via['net_id']
        ))

    # Add segments
    for seg in new_segments:
        elements.append(generate_segment_sexpr(
            seg['start'], seg['end'],
            seg['width'], seg['layer'], seg['net_id']
        ))

    if not elements:
        # Nothing to add, just copy the file
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(content)
        return True

    routing_text = '\n'.join(elements)

    # Insert before final paren
    last_paren = content.rfind(')')
    if last_paren == -1:
        print("Error: Could not find closing parenthesis in PCB file")
        return False

    new_content = content[:last_paren] + '\n' + routing_text + '\n' + content[last_paren:]

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(new_content)

    return True

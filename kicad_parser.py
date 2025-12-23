"""
KiCad PCB Parser - Extracts pads, nets, tracks, vias, and board info from .kicad_pcb files.
"""

import re
import math
import json
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Tuple, Optional
from pathlib import Path


@dataclass
class Pad:
    """Represents a component pad with global board coordinates."""
    component_ref: str
    pad_number: str
    global_x: float
    global_y: float
    local_x: float
    local_y: float
    size_x: float
    size_y: float
    shape: str  # circle, rect, roundrect, etc.
    layers: List[str]
    net_id: int
    net_name: str
    rotation: float = 0.0  # Total rotation in degrees (pad + footprint)
    pinfunction: str = ""
    pintype: str = ""


@dataclass
class Via:
    """Represents a via."""
    x: float
    y: float
    size: float
    drill: float
    layers: List[str]
    net_id: int
    uuid: str = ""


@dataclass
class Segment:
    """Represents a track segment."""
    start_x: float
    start_y: float
    end_x: float
    end_y: float
    width: float
    layer: str
    net_id: int
    uuid: str = ""
    # Original string representations for exact file matching
    start_x_str: str = ""
    start_y_str: str = ""
    end_x_str: str = ""
    end_y_str: str = ""


@dataclass
class Footprint:
    """Represents a component footprint."""
    reference: str
    footprint_name: str
    x: float
    y: float
    rotation: float
    layer: str
    pads: List[Pad] = field(default_factory=list)


@dataclass
class Net:
    """Represents a net (electrical connection)."""
    net_id: int
    name: str
    pads: List[Pad] = field(default_factory=list)


@dataclass
class BoardInfo:
    """Board-level information."""
    layers: Dict[int, str]  # layer_id -> layer_name
    copper_layers: List[str]
    board_bounds: Optional[Tuple[float, float, float, float]] = None  # min_x, min_y, max_x, max_y


@dataclass
class PCBData:
    """Complete parsed PCB data."""
    board_info: BoardInfo
    nets: Dict[int, Net]
    footprints: Dict[str, Footprint]
    vias: List[Via]
    segments: List[Segment]
    pads_by_net: Dict[int, List[Pad]]


def local_to_global(fp_x: float, fp_y: float, fp_rotation_deg: float,
                    pad_local_x: float, pad_local_y: float) -> Tuple[float, float]:
    """
    Transform pad LOCAL coordinates to GLOBAL board coordinates.

    CRITICAL: Negate the rotation angle! KiCad's rotation convention requires
    negating the angle when applying the standard rotation matrix formula.
    """
    rad = math.radians(-fp_rotation_deg)  # CRITICAL: negate the angle
    cos_r = math.cos(rad)
    sin_r = math.sin(rad)

    global_x = fp_x + (pad_local_x * cos_r - pad_local_y * sin_r)
    global_y = fp_y + (pad_local_x * sin_r + pad_local_y * cos_r)

    return global_x, global_y


def parse_s_expression(text: str) -> list:
    """
    Simple S-expression parser - returns nested lists.
    Not used for full file parsing (too slow), but useful for extracting specific elements.
    """
    tokens = re.findall(r'"[^"]*"|\(|\)|[^\s()]+', text)

    def parse_tokens(tokens, idx):
        result = []
        while idx < len(tokens):
            token = tokens[idx]
            if token == '(':
                sublist, idx = parse_tokens(tokens, idx + 1)
                result.append(sublist)
            elif token == ')':
                return result, idx
            else:
                # Remove quotes from strings
                if token.startswith('"') and token.endswith('"'):
                    token = token[1:-1]
                result.append(token)
            idx += 1
        return result, idx

    result, _ = parse_tokens(tokens, 0)
    return result


def extract_layers(content: str) -> BoardInfo:
    """Extract layer information from PCB file."""
    layers = {}
    copper_layers = []

    # Find the layers section
    layers_match = re.search(r'\(layers\s*((?:\([^)]+\)\s*)+)\)', content, re.DOTALL)
    if layers_match:
        layers_text = layers_match.group(1)
        # Parse individual layer entries: (0 "F.Cu" signal)
        layer_pattern = r'\((\d+)\s+"([^"]+)"\s+(\w+)'
        for m in re.finditer(layer_pattern, layers_text):
            layer_id = int(m.group(1))
            layer_name = m.group(2)
            layer_type = m.group(3)
            layers[layer_id] = layer_name
            if layer_type == 'signal' and '.Cu' in layer_name:
                copper_layers.append(layer_name)

    # Extract board bounds from Edge.Cuts
    bounds = extract_board_bounds(content)

    return BoardInfo(layers=layers, copper_layers=copper_layers, board_bounds=bounds)


def extract_board_bounds(content: str) -> Optional[Tuple[float, float, float, float]]:
    """Extract board outline bounds from Edge.Cuts layer."""
    min_x = min_y = float('inf')
    max_x = max_y = float('-inf')
    found = False

    # Look for gr_rect on Edge.Cuts
    # Use [^)]* instead of .*? to avoid catastrophic backtracking with many gr_rect elements
    rect_pattern = r'\(gr_rect\s+\(start\s+([\d.]+)\s+([\d.]+)\)\s+\(end\s+([\d.]+)\s+([\d.]+)\)[^)]*\)[^)]*\)\s+\(layer\s+"Edge\.Cuts"\)'
    for m in re.finditer(rect_pattern, content):
        x1, y1, x2, y2 = float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))
        min_x = min(min_x, x1, x2)
        min_y = min(min_y, y1, y2)
        max_x = max(max_x, x1, x2)
        max_y = max(max_y, y1, y2)
        found = True

    # Look for gr_line on Edge.Cuts
    # Use [^)]* instead of .*? to avoid catastrophic backtracking with many gr_line elements
    line_pattern = r'\(gr_line\s+\(start\s+([\d.]+)\s+([\d.]+)\)\s+\(end\s+([\d.]+)\s+([\d.]+)\)[^)]*\)[^)]*\)\s+\(layer\s+"Edge\.Cuts"\)'
    for m in re.finditer(line_pattern, content):
        x1, y1, x2, y2 = float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))
        min_x = min(min_x, x1, x2)
        min_y = min(min_y, y1, y2)
        max_x = max(max_x, x1, x2)
        max_y = max(max_y, y1, y2)
        found = True

    if found:
        return (min_x, min_y, max_x, max_y)
    return None


def extract_nets(content: str) -> Dict[int, Net]:
    """Extract all net definitions."""
    nets = {}
    net_pattern = r'\(net\s+(\d+)\s+"([^"]*)"\)'

    for m in re.finditer(net_pattern, content):
        net_id = int(m.group(1))
        net_name = m.group(2)
        nets[net_id] = Net(net_id=net_id, name=net_name)

    return nets


def extract_footprints_and_pads(content: str, nets: Dict[int, Net]) -> Tuple[Dict[str, Footprint], Dict[int, List[Pad]]]:
    """Extract footprints and their pads with global coordinates."""
    footprints = {}
    pads_by_net: Dict[int, List[Pad]] = {}

    # Find all footprints - need to handle nested parentheses properly
    # Strategy: find (footprint and then match balanced parens
    footprint_starts = [m.start() for m in re.finditer(r'\(footprint\s+"', content)]

    for start in footprint_starts:
        # Find the matching end parenthesis
        depth = 0
        end = start
        for i, char in enumerate(content[start:], start):
            if char == '(':
                depth += 1
            elif char == ')':
                depth -= 1
                if depth == 0:
                    end = i + 1
                    break

        fp_text = content[start:end]

        # Extract footprint name
        fp_name_match = re.search(r'\(footprint\s+"([^"]+)"', fp_text)
        if not fp_name_match:
            continue
        fp_name = fp_name_match.group(1)

        # Extract position and rotation
        at_match = re.search(r'\(at\s+([\d.-]+)\s+([\d.-]+)(?:\s+([\d.-]+))?\)', fp_text)
        if not at_match:
            continue

        fp_x = float(at_match.group(1))
        fp_y = float(at_match.group(2))
        fp_rotation = float(at_match.group(3)) if at_match.group(3) else 0.0

        # Extract layer
        layer_match = re.search(r'\(layer\s+"([^"]+)"\)', fp_text)
        fp_layer = layer_match.group(1) if layer_match else "F.Cu"

        # Extract reference
        ref_match = re.search(r'\(property\s+"Reference"\s+"([^"]+)"', fp_text)
        reference = ref_match.group(1) if ref_match else "?"

        footprint = Footprint(
            reference=reference,
            footprint_name=fp_name,
            x=fp_x,
            y=fp_y,
            rotation=fp_rotation,
            layer=fp_layer
        )

        # Extract pads
        # Pattern for pad: (pad "num" type shape ... (at x y [rot]) ... (size sx sy) ... (net id "name") ...)
        pad_pattern = r'\(pad\s+"([^"]+)"\s+(\w+)\s+(\w+)(.*?)\)\s*(?=\(pad|\(model|\(zone|\Z|$)'

        # Simpler approach: find pad starts and extract info
        for pad_match in re.finditer(r'\(pad\s+"([^"]+)"\s+(\w+)\s+(\w+)', fp_text):
            pad_start = pad_match.start()
            # Find end of this pad block
            depth = 0
            pad_end = pad_start
            for i, char in enumerate(fp_text[pad_start:], pad_start):
                if char == '(':
                    depth += 1
                elif char == ')':
                    depth -= 1
                    if depth == 0:
                        pad_end = i + 1
                        break

            pad_text = fp_text[pad_start:pad_end]

            pad_num = pad_match.group(1)
            pad_type = pad_match.group(2)  # smd, thru_hole, etc.
            pad_shape = pad_match.group(3)  # circle, rect, roundrect, etc.

            # Extract pad local position and rotation
            pad_at_match = re.search(r'\(at\s+([\d.-]+)\s+([\d.-]+)(?:\s+([\d.-]+))?\)', pad_text)
            if not pad_at_match:
                continue

            local_x = float(pad_at_match.group(1))
            local_y = float(pad_at_match.group(2))
            pad_rotation = float(pad_at_match.group(3)) if pad_at_match.group(3) else 0.0

            # Total rotation = pad rotation + footprint rotation
            total_rotation = (pad_rotation + fp_rotation) % 360

            # Extract size
            size_match = re.search(r'\(size\s+([\d.-]+)\s+([\d.-]+)\)', pad_text)
            if size_match:
                size_x = float(size_match.group(1))
                size_y = float(size_match.group(2))
            else:
                size_x = size_y = 0.5  # default

            # Apply only pad rotation to get board-space dimensions
            # The pad rotation already accounts for orientation relative to footprint,
            # and footprint rotation transforms coordinates but the size in local
            # footprint space after pad rotation gives the board-space dimensions
            pad_rot_normalized = pad_rotation % 180
            if 45 < pad_rot_normalized < 135:  # Close to 90°
                size_x, size_y = size_y, size_x

            # Extract layers
            layers_match = re.search(r'\(layers\s+"([^"]+)"(?:\s+"([^"]+)")*', pad_text)
            pad_layers = []
            if layers_match:
                pad_layers = [g for g in layers_match.groups() if g]

            # Extract net
            net_match = re.search(r'\(net\s+(\d+)\s+"([^"]*)"\)', pad_text)
            if net_match:
                net_id = int(net_match.group(1))
                net_name = net_match.group(2)
            else:
                net_id = 0
                net_name = ""

            # Extract pinfunction
            pinfunc_match = re.search(r'\(pinfunction\s+"([^"]*)"\)', pad_text)
            pinfunction = pinfunc_match.group(1) if pinfunc_match else ""

            # Extract pintype
            pintype_match = re.search(r'\(pintype\s+"([^"]*)"\)', pad_text)
            pintype = pintype_match.group(1) if pintype_match else ""

            # Calculate global coordinates
            global_x, global_y = local_to_global(fp_x, fp_y, fp_rotation, local_x, local_y)

            pad = Pad(
                component_ref=reference,
                pad_number=pad_num,
                global_x=global_x,
                global_y=global_y,
                local_x=local_x,
                local_y=local_y,
                size_x=size_x,
                size_y=size_y,
                shape=pad_shape,
                layers=pad_layers,
                net_id=net_id,
                net_name=net_name,
                rotation=total_rotation,
                pinfunction=pinfunction,
                pintype=pintype
            )

            footprint.pads.append(pad)

            # Add to pads_by_net
            if net_id not in pads_by_net:
                pads_by_net[net_id] = []
            pads_by_net[net_id].append(pad)

            # Also add to Net object
            if net_id in nets:
                nets[net_id].pads.append(pad)

        footprints[reference] = footprint

    return footprints, pads_by_net


def extract_vias(content: str) -> List[Via]:
    """Extract all vias from PCB file."""
    vias = []

    # Find via blocks
    via_pattern = r'\(via\s+\(at\s+([\d.-]+)\s+([\d.-]+)\)\s+\(size\s+([\d.-]+)\)\s+\(drill\s+([\d.-]+)\)\s+\(layers\s+"([^"]+)"\s+"([^"]+)"\)\s+\(net\s+(\d+)\)\s+\(uuid\s+"([^"]+)"\)'

    for m in re.finditer(via_pattern, content, re.DOTALL):
        via = Via(
            x=float(m.group(1)),
            y=float(m.group(2)),
            size=float(m.group(3)),
            drill=float(m.group(4)),
            layers=[m.group(5), m.group(6)],
            net_id=int(m.group(7)),
            uuid=m.group(8)
        )
        vias.append(via)

    return vias


def extract_segments(content: str) -> List[Segment]:
    """Extract all track segments from PCB file."""
    segments = []

    # Find segment blocks
    segment_pattern = r'\(segment\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)\s+\(width\s+([\d.-]+)\)\s+\(layer\s+"([^"]+)"\)\s+\(net\s+(\d+)\)\s+\(uuid\s+"([^"]+)"\)'

    for m in re.finditer(segment_pattern, content, re.DOTALL):
        segment = Segment(
            start_x=float(m.group(1)),
            start_y=float(m.group(2)),
            end_x=float(m.group(3)),
            end_y=float(m.group(4)),
            width=float(m.group(5)),
            layer=m.group(6),
            net_id=int(m.group(7)),
            uuid=m.group(8),
            # Store original strings for exact file matching
            start_x_str=m.group(1),
            start_y_str=m.group(2),
            end_x_str=m.group(3),
            end_y_str=m.group(4)
        )
        segments.append(segment)

    return segments


def parse_kicad_pcb(filepath: str) -> PCBData:
    """
    Parse a KiCad PCB file and extract all routing-relevant information.

    Args:
        filepath: Path to .kicad_pcb file

    Returns:
        PCBData object containing all parsed data
    """
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()

    # Extract components in order
    board_info = extract_layers(content)
    nets = extract_nets(content)
    footprints, pads_by_net = extract_footprints_and_pads(content, nets)
    vias = extract_vias(content)
    segments = extract_segments(content)

    return PCBData(
        board_info=board_info,
        nets=nets,
        footprints=footprints,
        vias=vias,
        segments=segments,
        pads_by_net=pads_by_net
    )


def save_extracted_data(pcb_data: PCBData, output_path: str):
    """Save extracted PCB data to JSON file."""

    def serialize(obj):
        if hasattr(obj, '__dict__'):
            return {k: serialize(v) for k, v in obj.__dict__.items()}
        elif isinstance(obj, dict):
            return {str(k): serialize(v) for k, v in obj.items()}
        elif isinstance(obj, list):
            return [serialize(item) for item in obj]
        else:
            return obj

    data = serialize(pcb_data)

    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2)


def get_nets_to_route(pcb_data: PCBData,
                      net_patterns: Optional[List[str]] = None,
                      exclude_patterns: Optional[List[str]] = None,
                      component_ref: Optional[str] = None) -> List[Net]:
    """
    Get nets that need routing based on filters.

    Args:
        pcb_data: Parsed PCB data
        net_patterns: List of wildcard patterns to match net names (e.g., ["LVDS_*", "DATA_*"])
        exclude_patterns: Patterns to exclude (default: GND, VCC, unconnected)
        component_ref: Only include nets connected to this component

    Returns:
        List of Net objects with 2+ pads that need routing
    """
    import fnmatch

    if exclude_patterns is None:
        exclude_patterns = ['*GND*', '*VCC*', '*VDD*', '*unconnected*', '*NC*', '']

    routes = []

    for net_id, net in pcb_data.nets.items():
        # Skip nets with < 2 pads (nothing to route)
        if len(net.pads) < 2:
            continue

        # Skip based on exclude patterns
        excluded = False
        for pattern in exclude_patterns:
            if fnmatch.fnmatch(net.name.upper(), pattern.upper()):
                excluded = True
                break
        if excluded:
            continue

        # Filter by net patterns if provided
        if net_patterns:
            matched = False
            for pattern in net_patterns:
                if fnmatch.fnmatch(net.name, pattern):
                    matched = True
                    break
            if not matched:
                continue

        # Filter by component if provided
        if component_ref:
            has_component = any(p.component_ref == component_ref for p in net.pads)
            if not has_component:
                continue

        routes.append(net)

    return routes


def detect_package_type(footprint: Footprint) -> str:
    """
    Detect the package type of a footprint based on its characteristics.

    Returns one of: 'BGA', 'QFN', 'QFP', 'SOIC', 'DIP', 'OTHER'

    Detection is based on:
    - Footprint name patterns
    - Pad arrangement (grid vs perimeter)
    - Pad shapes and sizes
    """
    fp_name = footprint.footprint_name.upper()

    # Check footprint name first
    if 'BGA' in fp_name or 'FBGA' in fp_name or 'LFBGA' in fp_name:
        return 'BGA'
    if 'QFN' in fp_name or 'DFN' in fp_name or 'MLF' in fp_name:
        return 'QFN'
    if 'QFP' in fp_name or 'LQFP' in fp_name or 'TQFP' in fp_name:
        return 'QFP'
    if 'SOIC' in fp_name or 'SOP' in fp_name or 'SSOP' in fp_name or 'TSSOP' in fp_name:
        return 'SOIC'
    if 'DIP' in fp_name or 'PDIP' in fp_name:
        return 'DIP'

    # Analyze pad arrangement if name doesn't indicate type
    pads = footprint.pads
    if len(pads) < 4:
        return 'OTHER'

    # Get unique X and Y positions
    x_positions = sorted(set(round(p.global_x, 3) for p in pads))
    y_positions = sorted(set(round(p.global_y, 3) for p in pads))

    # BGA: grid arrangement (multiple rows AND columns of pads)
    # QFN/QFP: perimeter arrangement (pads mostly on edges)

    if len(x_positions) >= 4 and len(y_positions) >= 4:
        # Check if pads form a filled grid (BGA) or just perimeter (QFN/QFP)
        # Count pads in interior vs perimeter
        min_x, max_x = min(x_positions), max(x_positions)
        min_y, max_y = min(y_positions), max(y_positions)

        # Define "interior" as not on the outermost positions
        interior_pads = 0
        perimeter_pads = 0
        tolerance = 0.1

        for pad in pads:
            on_edge = (abs(pad.global_x - min_x) < tolerance or
                      abs(pad.global_x - max_x) < tolerance or
                      abs(pad.global_y - min_y) < tolerance or
                      abs(pad.global_y - max_y) < tolerance)
            if on_edge:
                perimeter_pads += 1
            else:
                interior_pads += 1

        # BGA has many interior pads, QFN/QFP has mostly perimeter pads
        if interior_pads > perimeter_pads:
            return 'BGA'
        elif perimeter_pads > 0:
            # Check pad shapes - QFN typically has rectangular pads, BGA has circular
            circular_pads = sum(1 for p in pads if p.shape in ('circle', 'oval'))
            rect_pads = sum(1 for p in pads if p.shape in ('rect', 'roundrect'))
            if rect_pads > circular_pads:
                return 'QFN'
            else:
                return 'BGA'

    return 'OTHER'


def get_footprint_bounds(footprint: Footprint, margin: float = 0.0) -> Tuple[float, float, float, float]:
    """
    Get the bounding box of a footprint based on its pad positions.

    Args:
        footprint: The footprint to analyze
        margin: Extra margin to add around the bounds (in mm)

    Returns:
        (min_x, min_y, max_x, max_y) tuple
    """
    if not footprint.pads:
        # Fall back to footprint position if no pads
        return (footprint.x - margin, footprint.y - margin,
                footprint.x + margin, footprint.y + margin)

    min_x = min(p.global_x - p.size_x/2 for p in footprint.pads)
    max_x = max(p.global_x + p.size_x/2 for p in footprint.pads)
    min_y = min(p.global_y - p.size_y/2 for p in footprint.pads)
    max_y = max(p.global_y + p.size_y/2 for p in footprint.pads)

    return (min_x - margin, min_y - margin, max_x + margin, max_y + margin)


def find_components_by_type(pcb_data: 'PCBData', package_type: str) -> List[Footprint]:
    """
    Find all components of a specific package type.

    Args:
        pcb_data: Parsed PCB data
        package_type: One of 'BGA', 'QFN', 'QFP', 'SOIC', 'DIP', 'OTHER'

    Returns:
        List of matching Footprint objects
    """
    matches = []
    for ref, fp in pcb_data.footprints.items():
        if detect_package_type(fp) == package_type:
            matches.append(fp)
    return matches


def auto_detect_bga_exclusion_zones(pcb_data: 'PCBData', margin: float = 0.5) -> List[Tuple[float, float, float, float]]:
    """
    Auto-detect BGA exclusion zones from all BGA components in the PCB.

    Via placement should be avoided inside BGA packages to prevent shorts
    with the BGA balls.

    Args:
        pcb_data: Parsed PCB data
        margin: Extra margin around BGA bounds (in mm)

    Returns:
        List of (min_x, min_y, max_x, max_y) tuples for each BGA
    """
    zones = []
    bga_components = find_components_by_type(pcb_data, 'BGA')

    for fp in bga_components:
        bounds = get_footprint_bounds(fp, margin=margin)
        zones.append(bounds)

    return zones


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print("Usage: python kicad_parser.py <input.kicad_pcb> [output.json]")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else input_file.replace('.kicad_pcb', '_extracted.json')

    print(f"Parsing {input_file}...")
    pcb_data = parse_kicad_pcb(input_file)

    print(f"Found:")
    print(f"  - {len(pcb_data.board_info.copper_layers)} copper layers: {pcb_data.board_info.copper_layers}")
    print(f"  - {len(pcb_data.nets)} nets")
    print(f"  - {len(pcb_data.footprints)} footprints")
    print(f"  - {sum(len(fp.pads) for fp in pcb_data.footprints.values())} pads")
    print(f"  - {len(pcb_data.vias)} vias")
    print(f"  - {len(pcb_data.segments)} track segments")
    if pcb_data.board_info.board_bounds:
        bounds = pcb_data.board_info.board_bounds
        print(f"  - Board bounds: ({bounds[0]:.1f}, {bounds[1]:.1f}) to ({bounds[2]:.1f}, {bounds[3]:.1f}) mm")

    print(f"\nSaving extracted data to {output_file}...")
    save_extracted_data(pcb_data, output_file)
    print("Done!")

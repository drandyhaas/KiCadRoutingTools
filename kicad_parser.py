"""
KiCad PCB Parser - Extracts pads, nets, tracks, vias, and board info from .kicad_pcb files.
"""

import re
import math
import json
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Tuple, Optional
from pathlib import Path


# Position rounding precision for coordinate comparisons
# All position-based lookups must use this to ensure consistency
POSITION_DECIMALS = 3


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
    drill: float = 0.0  # Drill size for through-hole pads (0 for SMD)
    pintype: str = ""
    roundrect_rratio: float = 0.0  # Corner radius ratio for roundrect pads


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
    free: bool = False  # If True, KiCad won't auto-assign net based on overlapping tracks


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
class Zone:
    """Represents a filled zone (power plane)."""
    net_id: int
    net_name: str
    layer: str
    polygon: List[Tuple[float, float]]  # List of (x, y) vertices defining the zone outline
    uuid: str = ""


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
    value: str = ""  # Component value (e.g., "MCF5213", "100nF", "10K")


@dataclass
class Net:
    """Represents a net (electrical connection)."""
    net_id: int
    name: str
    pads: List[Pad] = field(default_factory=list)


@dataclass
class StackupLayer:
    """A layer in the board stackup."""
    name: str
    layer_type: str  # 'copper', 'core', 'prepreg', etc.
    thickness: float  # in mm
    epsilon_r: float = 0.0  # Dielectric constant (for dielectric layers)
    loss_tangent: float = 0.0  # Loss tangent (for dielectric layers)
    material: str = ""  # Material name (e.g., "FR4", "S1000-2M")


@dataclass
class BoardInfo:
    """Board-level information."""
    layers: Dict[int, str]  # layer_id -> layer_name
    copper_layers: List[str]
    board_bounds: Optional[Tuple[float, float, float, float]] = None  # min_x, min_y, max_x, max_y
    stackup: List[StackupLayer] = field(default_factory=list)  # ordered top to bottom
    board_outline: List[Tuple[float, float]] = field(default_factory=list)  # Polygon vertices for non-rectangular boards


@dataclass
class PCBData:
    """Complete parsed PCB data."""
    board_info: BoardInfo
    nets: Dict[int, Net]
    footprints: Dict[str, Footprint]
    vias: List[Via]
    segments: List[Segment]
    pads_by_net: Dict[int, List[Pad]]
    zones: List[Zone] = field(default_factory=list)

    def get_via_barrel_length(self, layer1: str, layer2: str) -> float:
        """Calculate the via barrel length between two copper layers.

        Args:
            layer1: First copper layer name (e.g., 'F.Cu', 'In2.Cu')
            layer2: Second copper layer name

        Returns:
            Distance in mm through the board between the two layers
        """
        stackup = self.board_info.stackup
        if not stackup:
            return 0.0

        # Find indices of the two layers in stackup
        idx1 = idx2 = -1
        for i, layer in enumerate(stackup):
            if layer.name == layer1:
                idx1 = i
            elif layer.name == layer2:
                idx2 = i

        if idx1 < 0 or idx2 < 0:
            return 0.0

        # Sum thicknesses between the two layers (exclusive of the layers themselves)
        start_idx = min(idx1, idx2)
        end_idx = max(idx1, idx2)

        total = 0.0
        for i in range(start_idx, end_idx + 1):
            total += stackup[i].thickness

        return total


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

    # Extract board outline polygon for non-rectangular boards
    outline = extract_board_outline(content)

    # Extract stackup information
    stackup = extract_stackup(content)

    return BoardInfo(layers=layers, copper_layers=copper_layers, board_bounds=bounds, stackup=stackup, board_outline=outline)


def extract_stackup(content: str) -> List[StackupLayer]:
    """Extract board stackup information for impedance calculation and via barrel length.

    Extracts copper and dielectric layers with their electrical properties:
    - thickness: layer thickness in mm
    - epsilon_r: dielectric constant (for dielectric layers)
    - loss_tangent: loss tangent (for dielectric layers)
    - material: material name
    """
    stackup = []

    # Find the stackup section
    stackup_match = re.search(r'\(stackup\s+(.*?)\n\s*\(copper_finish', content, re.DOTALL)
    if not stackup_match:
        # Try alternate pattern without copper_finish
        stackup_match = re.search(r'\(stackup\s+(.*?)\n\s*\)\s*\n', content, re.DOTALL)

    if not stackup_match:
        return stackup

    stackup_text = stackup_match.group(1)

    # Parse each layer in stackup
    # Pattern matches: (layer "name" (type "typename") (thickness value) ...)
    # We need to handle multi-line layer definitions
    layer_blocks = re.findall(r'\(layer\s+"([^"]+)"(.*?)(?=\(layer\s+"|$)', stackup_text, re.DOTALL)

    for layer_name, layer_content in layer_blocks:
        # Extract type
        type_match = re.search(r'\(type\s+"([^"]+)"\)', layer_content)
        layer_type = type_match.group(1) if type_match else 'unknown'

        # Extract thickness (in mm)
        thickness_match = re.search(r'\(thickness\s+([\d.]+)\)', layer_content)
        thickness = float(thickness_match.group(1)) if thickness_match else 0.0

        # Extract dielectric constant (epsilon_r)
        epsilon_match = re.search(r'\(epsilon_r\s+([\d.]+)\)', layer_content)
        epsilon_r = float(epsilon_match.group(1)) if epsilon_match else 0.0

        # Extract loss tangent
        loss_match = re.search(r'\(loss_tangent\s+([\d.]+)\)', layer_content)
        loss_tangent = float(loss_match.group(1)) if loss_match else 0.0

        # Extract material name
        material_match = re.search(r'\(material\s+"([^"]+)"\)', layer_content)
        material = material_match.group(1) if material_match else ""

        # Only include copper and dielectric layers (skip mask, silk, paste)
        if layer_type in ('copper', 'core', 'prepreg'):
            stackup.append(StackupLayer(
                name=layer_name,
                layer_type=layer_type,
                thickness=thickness,
                epsilon_r=epsilon_r,
                loss_tangent=loss_tangent,
                material=material
            ))

    return stackup


def extract_board_bounds(content: str) -> Optional[Tuple[float, float, float, float]]:
    """Extract board outline bounds from Edge.Cuts layer."""
    min_x = min_y = float('inf')
    max_x = max_y = float('-inf')
    found = False

    # Look for gr_rect on Edge.Cuts (multi-line format)
    rect_pattern = r'\(gr_rect\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\).*?\(layer\s+"Edge\.Cuts"\)'
    for m in re.finditer(rect_pattern, content, re.DOTALL):
        x1, y1, x2, y2 = float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))
        min_x = min(min_x, x1, x2)
        min_y = min(min_y, y1, y2)
        max_x = max(max_x, x1, x2)
        max_y = max(max_y, y1, y2)
        found = True

    # Look for gr_line on Edge.Cuts (multi-line format)
    line_pattern = r'\(gr_line\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\).*?\(layer\s+"Edge\.Cuts"\)'
    for m in re.finditer(line_pattern, content, re.DOTALL):
        x1, y1, x2, y2 = float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))
        min_x = min(min_x, x1, x2)
        min_y = min(min_y, y1, y2)
        max_x = max(max_x, x1, x2)
        max_y = max(max_y, y1, y2)
        found = True

    if found:
        return (min_x, min_y, max_x, max_y)
    return None


def extract_board_outline(content: str) -> List[Tuple[float, float]]:
    """Extract board outline polygon from Edge.Cuts layer.

    Parses gr_line segments and assembles them into a closed polygon.
    Returns an empty list if no outline is found or if it's a simple rectangle.
    """
    # Collect all line segments as (start, end) tuples
    segments = []

    # Look for gr_line on Edge.Cuts (multi-line format)
    line_pattern = r'\(gr_line\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\).*?\(layer\s+"Edge\.Cuts"\)'
    for m in re.finditer(line_pattern, content, re.DOTALL):
        x1, y1, x2, y2 = float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))
        segments.append(((x1, y1), (x2, y2)))

    if len(segments) < 3:
        return []  # Need at least 3 segments for a polygon

    # Check if this is a simple 4-segment rectangle (no need for polygon handling)
    if len(segments) == 4:
        # Get all unique vertices
        vertices = set()
        for seg in segments:
            vertices.add((round(seg[0][0], 3), round(seg[0][1], 3)))
            vertices.add((round(seg[1][0], 3), round(seg[1][1], 3)))
        if len(vertices) == 4:
            # 4 segments with 4 vertices - likely a simple rectangle
            xs = [v[0] for v in vertices]
            ys = [v[1] for v in vertices]
            # Check if all edges are axis-aligned
            all_axis_aligned = all(
                abs(s[0][0] - s[1][0]) < 0.001 or abs(s[0][1] - s[1][1]) < 0.001
                for s in segments
            )
            if all_axis_aligned:
                return []  # Simple rectangle, use bounding box

    # Build polygon by chaining segments
    # Start with first segment
    polygon = [segments[0][0], segments[0][1]]
    used = {0}

    # Round for comparison
    def approx_equal(p1, p2, tol=0.01):
        return abs(p1[0] - p2[0]) < tol and abs(p1[1] - p2[1]) < tol

    # Chain remaining segments
    max_iterations = len(segments) * 2
    iteration = 0
    while len(used) < len(segments) and iteration < max_iterations:
        iteration += 1
        current_end = polygon[-1]
        found_next = False

        for i, seg in enumerate(segments):
            if i in used:
                continue

            if approx_equal(seg[0], current_end):
                polygon.append(seg[1])
                used.add(i)
                found_next = True
                break
            elif approx_equal(seg[1], current_end):
                polygon.append(seg[0])
                used.add(i)
                found_next = True
                break

        if not found_next:
            break

    # Remove duplicate last point if polygon is closed
    if len(polygon) > 1 and approx_equal(polygon[0], polygon[-1]):
        polygon = polygon[:-1]

    # Only return polygon if we used all segments (complete outline)
    if len(used) == len(segments) and len(polygon) >= 3:
        return polygon

    return []  # Incomplete outline, fall back to bounding box


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

        # Extract value (component part number or value)
        value_match = re.search(r'\(property\s+"Value"\s+"([^"]+)"', fp_text)
        value = value_match.group(1) if value_match else ""

        footprint = Footprint(
            reference=reference,
            footprint_name=fp_name,
            x=fp_x,
            y=fp_y,
            rotation=fp_rotation,
            layer=fp_layer,
            value=value
        )

        # Extract pads
        # Pattern for pad: (pad "num" type shape ... (at x y [rot]) ... (size sx sy) ... (net id "name") ...)
        pad_pattern = r'\(pad\s+"([^"]+)"\s+(\w+)\s+(\w+)(.*?)\)\s*(?=\(pad|\(model|\(zone|\Z|$)'

        # Simpler approach: find pad starts and extract info
        # Note: pad number can be empty string (pad "") so use [^"]* not [^"]+
        for pad_match in re.finditer(r'\(pad\s+"([^"]*)"\s+(\w+)\s+(\w+)', fp_text):
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

            # Extract layers - use findall to get all quoted layer names
            layers_section = re.search(r'\(layers\s+([^)]+)\)', pad_text)
            pad_layers = []
            if layers_section:
                pad_layers = re.findall(r'"([^"]+)"', layers_section.group(1))

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

            # Extract drill size for through-hole pads
            drill_match = re.search(r'\(drill\s+([\d.]+)', pad_text)
            drill_size = float(drill_match.group(1)) if drill_match else 0.0

            # Extract roundrect_rratio for roundrect pads
            rratio_match = re.search(r'\(roundrect_rratio\s+([\d.]+)\)', pad_text)
            roundrect_rratio = float(rratio_match.group(1)) if rratio_match else 0.0

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
                pintype=pintype,
                drill=drill_size,
                roundrect_rratio=roundrect_rratio
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

    # Find via blocks - (free yes) is optional between layers and net
    via_pattern = r'\(via\s+\(at\s+([\d.-]+)\s+([\d.-]+)\)\s+\(size\s+([\d.-]+)\)\s+\(drill\s+([\d.-]+)\)\s+\(layers\s+"([^"]+)"\s+"([^"]+)"\)\s+(?:\(free\s+(yes|no)\)\s+)?\(net\s+(\d+)\)\s+\(uuid\s+"([^"]+)"\)'

    for m in re.finditer(via_pattern, content, re.DOTALL):
        free_value = m.group(7)  # "yes", "no", or None
        via = Via(
            x=float(m.group(1)),
            y=float(m.group(2)),
            size=float(m.group(3)),
            drill=float(m.group(4)),
            layers=[m.group(5), m.group(6)],
            net_id=int(m.group(8)),
            uuid=m.group(9),
            free=(free_value == "yes")
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


def extract_zones(content: str) -> List[Zone]:
    """Extract all filled zones from PCB file.

    Parses zone definitions including their net assignment, layer, and polygon outline.
    These are used for power planes and other filled copper areas.
    """
    zones = []

    # Find each zone block start - zones are at the top level, indented with single tab
    # Use \r?\n to handle both Unix and Windows line endings
    zone_start_pattern = r'\r?\n\t\(zone\s*\r?\n'

    for start_match in re.finditer(zone_start_pattern, content):
        # Find the matching closing paren by counting balanced parens
        start_pos = start_match.start() + len(start_match.group()) - 1  # Position after opening (
        paren_count = 1
        pos = start_match.end()
        zone_end = None

        while pos < len(content) and paren_count > 0:
            char = content[pos]
            if char == '(':
                paren_count += 1
            elif char == ')':
                paren_count -= 1
                if paren_count == 0:
                    zone_end = pos
            pos += 1

        if zone_end is None:
            continue

        zone_content = content[start_match.end():zone_end]

        # Extract net id
        net_match = re.search(r'\(net\s+(\d+)\)', zone_content)
        if not net_match:
            continue
        net_id = int(net_match.group(1))

        # Extract net name
        net_name_match = re.search(r'\(net_name\s+"([^"]*)"\)', zone_content)
        net_name = net_name_match.group(1) if net_name_match else ""

        # Extract layer
        layer_match = re.search(r'\(layer\s+"([^"]+)"\)', zone_content)
        if not layer_match:
            continue
        layer = layer_match.group(1)

        # Extract UUID
        uuid_match = re.search(r'\(uuid\s+"([^"]+)"\)', zone_content)
        uuid = uuid_match.group(1) if uuid_match else ""

        # Extract polygon points - find (pts ...) and extract xy coordinates
        pts_start = zone_content.find('(pts')
        if pts_start < 0:
            continue

        # Find matching closing paren for (pts
        paren_count = 0
        pts_end = pts_start
        for i in range(pts_start, len(zone_content)):
            if zone_content[i] == '(':
                paren_count += 1
            elif zone_content[i] == ')':
                paren_count -= 1
                if paren_count == 0:
                    pts_end = i
                    break

        pts_content = zone_content[pts_start:pts_end + 1]
        # Parse all (xy x y) points
        xy_pattern = r'\(xy\s+([\d.-]+)\s+([\d.-]+)\)'
        polygon = [(float(m.group(1)), float(m.group(2)))
                   for m in re.finditer(xy_pattern, pts_content)]

        if not polygon:
            continue

        zone = Zone(
            net_id=net_id,
            net_name=net_name,
            layer=layer,
            polygon=polygon,
            uuid=uuid
        )
        zones.append(zone)

    return zones


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
    zones = extract_zones(content)

    return PCBData(
        board_info=board_info,
        nets=nets,
        footprints=footprints,
        vias=vias,
        segments=segments,
        pads_by_net=pads_by_net,
        zones=zones
    )


def build_pcb_data_from_board(board) -> PCBData:
    """Build PCBData directly from a pcbnew board object (no file I/O).

    This is much faster than parse_kicad_pcb() since it reads from pcbnew's
    in-memory data structures rather than parsing the file from disk.

    Args:
        board: A pcbnew.BOARD object (from pcbnew.GetBoard())

    Returns:
        PCBData object containing all routing-relevant data
    """
    import pcbnew

    def to_mm(val):
        return pcbnew.ToMM(val)

    # --- Build layer mappings ---
    id_to_name = {pcbnew.F_Cu: 'F.Cu', pcbnew.B_Cu: 'B.Cu'}
    for i in range(1, 31):
        layer_id = getattr(pcbnew, f'In{i}_Cu', None)
        if layer_id is not None:
            id_to_name[layer_id] = f'In{i}.Cu'

    def get_layer_name(layer_id):
        if layer_id in id_to_name:
            return id_to_name[layer_id]
        return board.GetLayerName(layer_id)

    # --- Pad shape mapping ---
    pad_shape_map = {}
    for attr, name in [
        ('PAD_SHAPE_CIRCLE', 'circle'),
        ('PAD_SHAPE_RECT', 'rect'),
        ('PAD_SHAPE_OVAL', 'oval'),
        ('PAD_SHAPE_ROUNDRECT', 'roundrect'),
        ('PAD_SHAPE_TRAPEZOID', 'trapezoid'),
        ('PAD_SHAPE_CUSTOM', 'custom'),
        ('PAD_SHAPE_CHAMFERED_RECT', 'roundrect'),
    ]:
        val = getattr(pcbnew, attr, None)
        if val is not None:
            pad_shape_map[val] = name

    def get_pad_shape_name(shape_enum):
        return pad_shape_map.get(shape_enum, 'rect')

    def get_pad_layers(pad):
        """Get layer names from a pad's layer set, using wildcards to match file format."""
        layer_set = pad.GetLayerSet()
        layers = []

        # Check copper layers - use *.Cu wildcard if pad is on ALL copper layers
        copper_on = [lname for lid, lname in id_to_name.items() if layer_set.Contains(lid)]
        if len(copper_on) == len(id_to_name):
            layers.append('*.Cu')
        elif copper_on:
            layers.extend(copper_on)

        # Check mask layers - use *.Mask wildcard if both present
        f_mask_id = getattr(pcbnew, 'F_Mask', None)
        b_mask_id = getattr(pcbnew, 'B_Mask', None)
        has_f_mask = f_mask_id is not None and layer_set.Contains(f_mask_id)
        has_b_mask = b_mask_id is not None and layer_set.Contains(b_mask_id)
        if has_f_mask and has_b_mask:
            layers.append('*.Mask')
        else:
            if has_f_mask:
                layers.append('F.Mask')
            if has_b_mask:
                layers.append('B.Mask')

        # Check paste layers - use *.Paste wildcard if both present
        f_paste_id = getattr(pcbnew, 'F_Paste', None)
        b_paste_id = getattr(pcbnew, 'B_Paste', None)
        has_f_paste = f_paste_id is not None and layer_set.Contains(f_paste_id)
        has_b_paste = b_paste_id is not None and layer_set.Contains(b_paste_id)
        if has_f_paste and has_b_paste:
            layers.append('*.Paste')
        else:
            if has_f_paste:
                layers.append('F.Paste')
            if has_b_paste:
                layers.append('B.Paste')

        return layers

    # --- Extract board info ---
    layers_dict = {}
    copper_layers = []
    enabled = board.GetEnabledLayers()
    for lid, lname in id_to_name.items():
        if enabled.Contains(lid):
            layers_dict[lid] = lname
            copper_layers.append(lname)

    # Board bounds from Edge.Cuts drawing coordinates (not bounding box which includes line width)
    board_bounds = None
    try:
        edge_cuts_id = getattr(pcbnew, 'Edge_Cuts', None)
        if edge_cuts_id is not None:
            bmin_x = bmin_y = float('inf')
            bmax_x = bmax_y = float('-inf')
            found_edge = False
            for drawing in board.GetDrawings():
                if drawing.GetLayer() != edge_cuts_id:
                    continue
                class_name = drawing.GetClass()
                if class_name in ("PCB_SHAPE", "DRAWSEGMENT"):
                    try:
                        start = drawing.GetStart()
                        end = drawing.GetEnd()
                        sx, sy = to_mm(start.x), to_mm(start.y)
                        ex, ey = to_mm(end.x), to_mm(end.y)
                        bmin_x = min(bmin_x, sx, ex)
                        bmin_y = min(bmin_y, sy, ey)
                        bmax_x = max(bmax_x, sx, ex)
                        bmax_y = max(bmax_y, sy, ey)
                        found_edge = True
                    except Exception:
                        continue
            if found_edge:
                board_bounds = (bmin_x, bmin_y, bmax_x, bmax_y)
    except Exception:
        pass
    # Fallback to bounding box if no Edge.Cuts drawings found
    if board_bounds is None:
        try:
            bbox = board.GetBoardEdgesBoundingBox()
            if bbox.GetWidth() > 0 and bbox.GetHeight() > 0:
                board_bounds = (to_mm(bbox.GetX()), to_mm(bbox.GetY()),
                                to_mm(bbox.GetX() + bbox.GetWidth()),
                                to_mm(bbox.GetY() + bbox.GetHeight()))
        except Exception:
            pass

    # Board outline from Edge.Cuts drawings
    board_outline = _extract_board_outline_from_pcbnew(board, to_mm)

    # Stackup
    stackup = _extract_stackup_from_pcbnew(board, to_mm)

    board_info = BoardInfo(
        layers=layers_dict,
        copper_layers=copper_layers,
        board_bounds=board_bounds,
        stackup=stackup,
        board_outline=board_outline
    )

    # --- Extract nets ---
    nets = {}
    try:
        net_info = board.GetNetInfo()
        # Use NetsByName which is proven to work (see fanout_gui.py)
        nets_by_name = net_info.NetsByName()
        for net_name_wx, net_item in nets_by_name.items():
            net_id = net_item.GetNetCode()
            name = str(net_name_wx)
            nets[net_id] = Net(net_id=net_id, name=name)
    except Exception:
        # Fallback: iterate by net count
        try:
            for i in range(board.GetNetCount()):
                net_item = board.GetNetInfo().GetNetItem(i)
                if net_item:
                    nets[i] = Net(net_id=i, name=net_item.GetNetname())
        except Exception:
            pass

    # --- Extract footprints and pads ---
    footprints = {}
    pads_by_net: Dict[int, List[Pad]] = {}

    for fp in board.GetFootprints():
        reference = fp.GetReference()

        # Get footprint name (library:footprint)
        try:
            fp_name = fp.GetFPID().GetUniStringLibItemName()
        except AttributeError:
            try:
                fp_name = str(fp.GetFPID().GetLibItemName())
            except Exception:
                fp_name = ""

        # Position
        pos = fp.GetPosition()
        fp_x = to_mm(pos.x)
        fp_y = to_mm(pos.y)

        # Rotation
        try:
            fp_rotation = fp.GetOrientationDegrees()
        except AttributeError:
            try:
                fp_rotation = fp.GetOrientation().AsDegrees()
            except Exception:
                fp_rotation = fp.GetOrientation() / 10.0  # Older KiCad: tenths of degrees

        # Layer
        fp_layer = get_layer_name(fp.GetLayer())

        # Value
        try:
            fp_value = fp.GetValue()
        except Exception:
            fp_value = ""

        footprint = Footprint(
            reference=reference,
            footprint_name=fp_name,
            x=fp_x,
            y=fp_y,
            rotation=fp_rotation,
            layer=fp_layer,
            value=fp_value
        )

        # Extract pads
        for pad in fp.Pads():
            pad_num = pad.GetNumber()

            # Global position
            pad_pos = pad.GetPosition()
            global_x = to_mm(pad_pos.x)
            global_y = to_mm(pad_pos.y)

            # Local position (relative to footprint, before footprint rotation)
            try:
                local_pos = pad.GetPos0()
                local_x = to_mm(local_pos.x)
                local_y = to_mm(local_pos.y)
            except Exception:
                # Fallback: compute from global position
                local_x, local_y = _global_to_local(fp_x, fp_y, fp_rotation, global_x, global_y)

            # Size
            pad_size = pad.GetSize()
            size_x = to_mm(pad_size.x)
            size_y = to_mm(pad_size.y)

            # Shape
            shape = get_pad_shape_name(pad.GetShape())

            # Layers
            pad_layers = get_pad_layers(pad)

            # Net
            net_id = pad.GetNetCode()
            net_name = pad.GetNetname()

            # Pad rotation
            try:
                pad_rotation = pad.GetOrientationDegrees()
            except AttributeError:
                try:
                    pad_rotation = pad.GetOrientation().AsDegrees()
                except Exception:
                    pad_rotation = pad.GetOrientation() / 10.0

            total_rotation = (pad_rotation + fp_rotation) % 360

            # Apply only pad rotation to get board-space dimensions
            pad_rot_normalized = pad_rotation % 180
            if 45 < pad_rot_normalized < 135:
                size_x, size_y = size_y, size_x

            # Pin metadata
            try:
                pinfunction = pad.GetPinFunction()
            except Exception:
                pinfunction = ""

            try:
                pintype = pad.GetPinType()
            except Exception:
                pintype = ""

            # Drill
            try:
                drill_size = pad.GetDrillSize()
                drill = to_mm(drill_size.x)
            except Exception:
                drill = 0.0

            # Roundrect ratio
            try:
                roundrect_rratio = pad.GetRoundRectRadiusRatio()
            except Exception:
                roundrect_rratio = 0.0

            pad_obj = Pad(
                component_ref=reference,
                pad_number=pad_num,
                global_x=global_x,
                global_y=global_y,
                local_x=local_x,
                local_y=local_y,
                size_x=size_x,
                size_y=size_y,
                shape=shape,
                layers=pad_layers,
                net_id=net_id,
                net_name=net_name,
                rotation=total_rotation,
                pinfunction=pinfunction,
                pintype=pintype,
                drill=drill,
                roundrect_rratio=roundrect_rratio
            )

            footprint.pads.append(pad_obj)

            # Add to pads_by_net
            if net_id not in pads_by_net:
                pads_by_net[net_id] = []
            pads_by_net[net_id].append(pad_obj)

            # Add to Net object
            if net_id in nets:
                nets[net_id].pads.append(pad_obj)

        footprints[reference] = footprint

    # --- Extract segments and vias (single pass over tracks) ---
    segments = []
    vias = []
    for track in board.GetTracks():
        track_class = track.GetClass()
        if track_class == "PCB_TRACK":
            seg = Segment(
                start_x=to_mm(track.GetStart().x),
                start_y=to_mm(track.GetStart().y),
                end_x=to_mm(track.GetEnd().x),
                end_y=to_mm(track.GetEnd().y),
                width=to_mm(track.GetWidth()),
                layer=get_layer_name(track.GetLayer()),
                net_id=track.GetNetCode(),
            )
            segments.append(seg)
        elif track_class == "PCB_VIA":
            v = Via(
                x=to_mm(track.GetPosition().x),
                y=to_mm(track.GetPosition().y),
                size=to_mm(track.GetWidth()),
                drill=to_mm(track.GetDrill()),
                layers=[get_layer_name(track.TopLayer()), get_layer_name(track.BottomLayer())],
                net_id=track.GetNetCode(),
            )
            vias.append(v)

    # --- Extract zones ---
    zones = _extract_zones_from_pcbnew(board, to_mm, get_layer_name)

    return PCBData(
        board_info=board_info,
        nets=nets,
        footprints=footprints,
        vias=vias,
        segments=segments,
        pads_by_net=pads_by_net,
        zones=zones
    )


def _global_to_local(fp_x, fp_y, fp_rotation_deg, global_x, global_y):
    """Reverse transform: global board coordinates to local footprint coordinates."""
    rad = math.radians(fp_rotation_deg)  # Positive rotation to reverse the transform
    cos_r = math.cos(rad)
    sin_r = math.sin(rad)

    dx = global_x - fp_x
    dy = global_y - fp_y

    local_x = dx * cos_r + dy * sin_r
    local_y = -dx * sin_r + dy * cos_r

    return local_x, local_y


def _extract_board_outline_from_pcbnew(board, to_mm):
    """Extract board outline polygon from Edge.Cuts drawings via pcbnew."""
    import pcbnew

    edge_cuts_id = getattr(pcbnew, 'Edge_Cuts', None)
    if edge_cuts_id is None:
        return []

    segments = []
    for drawing in board.GetDrawings():
        if drawing.GetLayer() != edge_cuts_id:
            continue
        class_name = drawing.GetClass()
        if class_name in ("PCB_SHAPE", "DRAWSEGMENT"):
            try:
                shape_type = drawing.GetShape()
                # Line segment
                if shape_type == getattr(pcbnew, 'SHAPE_T_SEGMENT', getattr(pcbnew, 'S_SEGMENT', -1)):
                    start = drawing.GetStart()
                    end = drawing.GetEnd()
                    segments.append((
                        (to_mm(start.x), to_mm(start.y)),
                        (to_mm(end.x), to_mm(end.y))
                    ))
                elif shape_type == getattr(pcbnew, 'SHAPE_T_RECT', getattr(pcbnew, 'S_RECT', -1)):
                    start = drawing.GetStart()
                    end = drawing.GetEnd()
                    x1, y1 = to_mm(start.x), to_mm(start.y)
                    x2, y2 = to_mm(end.x), to_mm(end.y)
                    segments.append(((x1, y1), (x2, y1)))
                    segments.append(((x2, y1), (x2, y2)))
                    segments.append(((x2, y2), (x1, y2)))
                    segments.append(((x1, y2), (x1, y1)))
            except Exception:
                continue

    if len(segments) < 3:
        return []

    # Check if this is a simple 4-segment rectangle
    if len(segments) == 4:
        vertices = set()
        for seg in segments:
            vertices.add((round(seg[0][0], 3), round(seg[0][1], 3)))
            vertices.add((round(seg[1][0], 3), round(seg[1][1], 3)))
        if len(vertices) == 4:
            all_axis_aligned = all(
                abs(s[0][0] - s[1][0]) < 0.001 or abs(s[0][1] - s[1][1]) < 0.001
                for s in segments
            )
            if all_axis_aligned:
                return []

    # Build polygon by chaining segments
    polygon = [segments[0][0], segments[0][1]]
    used = {0}

    def approx_equal(p1, p2, tol=0.01):
        return abs(p1[0] - p2[0]) < tol and abs(p1[1] - p2[1]) < tol

    max_iterations = len(segments) * 2
    iteration = 0
    while len(used) < len(segments) and iteration < max_iterations:
        iteration += 1
        current_end = polygon[-1]
        found_next = False
        for i, seg in enumerate(segments):
            if i in used:
                continue
            if approx_equal(seg[0], current_end):
                polygon.append(seg[1])
                used.add(i)
                found_next = True
                break
            elif approx_equal(seg[1], current_end):
                polygon.append(seg[0])
                used.add(i)
                found_next = True
                break
        if not found_next:
            break

    if len(polygon) > 1 and approx_equal(polygon[0], polygon[-1]):
        polygon = polygon[:-1]

    if len(used) == len(segments) and len(polygon) >= 3:
        return polygon

    return []


def _extract_stackup_from_pcbnew(board, to_mm):
    """Extract board stackup from pcbnew design settings.

    Tries the SWIG API first, falls back to parsing the board file since
    BOARD_STACKUP bindings aren't fully exposed in all KiCad versions.
    """
    stackup = []

    # First try the full SWIG API (works in some KiCad versions)
    try:
        ds = board.GetDesignSettings()
        stackup_desc = ds.GetStackupDescriptor()
        stack_list = stackup_desc.GetList()

        for item in stack_list:
            try:
                layer_name = item.GetLayerName()
                type_name = item.GetTypeName()
                if type_name not in ('copper', 'core', 'prepreg', 'dielectric'):
                    continue
                thickness = to_mm(item.GetThickness())
                epsilon_r = getattr(item, 'GetEpsilonR', lambda: 0.0)()
                loss_tangent = getattr(item, 'GetLossTangent', lambda: 0.0)()
                material = getattr(item, 'GetMaterial', lambda: "")()
                stackup.append(StackupLayer(
                    name=layer_name, layer_type=type_name, thickness=thickness,
                    epsilon_r=epsilon_r, loss_tangent=loss_tangent, material=material
                ))
            except Exception:
                continue
    except Exception:
        pass

    if stackup:
        return stackup

    # Fallback: parse stackup from the board file
    try:
        board_filename = board.GetFileName()
        if board_filename:
            with open(board_filename, 'r', encoding='utf-8') as f:
                content = f.read(8192)  # Stackup is near the top of the file
            stackup = extract_stackup(content)
    except Exception:
        pass

    return stackup


def _extract_zones_from_pcbnew(board, to_mm, get_layer_name):
    """Extract filled zones from pcbnew board."""
    zones = []

    try:
        for zone in board.Zones():
            net_id = zone.GetNetCode()
            net_name = zone.GetNetname()
            layer = get_layer_name(zone.GetLayer())

            # Extract polygon outline
            polygon = []
            try:
                outline = zone.Outline()
                if outline and outline.OutlineCount() > 0:
                    contour = outline.Outline(0)
                    for j in range(contour.PointCount()):
                        pt = contour.CPoint(j)
                        polygon.append((to_mm(pt.x), to_mm(pt.y)))
            except Exception:
                continue

            if not polygon:
                continue

            zone_obj = Zone(
                net_id=net_id,
                net_name=net_name,
                layer=layer,
                polygon=polygon
            )
            zones.append(zone_obj)
    except Exception:
        pass

    return zones


def compare_pcb_data(from_board: 'PCBData', from_file: 'PCBData', tolerance: float = 0.01) -> List[str]:
    """Compare two PCBData objects and return list of differences.

    Useful for validating that build_pcb_data_from_board() produces the same
    results as parse_kicad_pcb().

    Args:
        from_board: PCBData built from pcbnew SWIG API
        from_file: PCBData parsed from .kicad_pcb file
        tolerance: Position tolerance in mm for coordinate comparisons

    Returns:
        List of difference description strings (empty if identical)
    """
    diffs = []

    def close(a, b):
        return abs(a - b) < tolerance

    # --- Compare board info ---
    bi_b = from_board.board_info
    bi_f = from_file.board_info
    if set(bi_b.copper_layers) != set(bi_f.copper_layers):
        diffs.append(f"Copper layers differ: board={bi_b.copper_layers} file={bi_f.copper_layers}")

    if bi_b.board_bounds and bi_f.board_bounds:
        for i, label in enumerate(['min_x', 'min_y', 'max_x', 'max_y']):
            if not close(bi_b.board_bounds[i], bi_f.board_bounds[i]):
                diffs.append(f"Board bounds {label}: board={bi_b.board_bounds[i]:.3f} file={bi_f.board_bounds[i]:.3f}")
    elif bi_b.board_bounds != bi_f.board_bounds:
        diffs.append(f"Board bounds: board={bi_b.board_bounds} file={bi_f.board_bounds}")

    if len(bi_b.stackup) != len(bi_f.stackup):
        diffs.append(f"Stackup layer count: board={len(bi_b.stackup)} file={len(bi_f.stackup)}")

    # --- Compare nets ---
    board_net_ids = set(from_board.nets.keys())
    file_net_ids = set(from_file.nets.keys())
    if board_net_ids != file_net_ids:
        only_board = board_net_ids - file_net_ids
        only_file = file_net_ids - board_net_ids
        if only_board:
            diffs.append(f"Nets only in board: {sorted(only_board)[:10]}{'...' if len(only_board) > 10 else ''}")
        if only_file:
            diffs.append(f"Nets only in file: {sorted(only_file)[:10]}{'...' if len(only_file) > 10 else ''}")

    for net_id in board_net_ids & file_net_ids:
        bn = from_board.nets[net_id]
        fn = from_file.nets[net_id]
        if bn.name != fn.name:
            diffs.append(f"Net {net_id} name: board='{bn.name}' file='{fn.name}'")
        if len(bn.pads) != len(fn.pads):
            diffs.append(f"Net {net_id} '{bn.name}' pad count: board={len(bn.pads)} file={len(fn.pads)}")

    # --- Compare footprints ---
    board_refs = set(from_board.footprints.keys())
    file_refs = set(from_file.footprints.keys())
    if board_refs != file_refs:
        only_board = board_refs - file_refs
        only_file = file_refs - board_refs
        if only_board:
            diffs.append(f"Footprints only in board: {sorted(only_board)[:10]}")
        if only_file:
            diffs.append(f"Footprints only in file: {sorted(only_file)[:10]}")

    for ref in sorted(board_refs & file_refs):
        bf = from_board.footprints[ref]
        ff = from_file.footprints[ref]

        if not close(bf.x, ff.x) or not close(bf.y, ff.y):
            diffs.append(f"Footprint {ref} position: board=({bf.x:.3f},{bf.y:.3f}) file=({ff.x:.3f},{ff.y:.3f})")
        if not close(bf.rotation % 360, ff.rotation % 360):
            diffs.append(f"Footprint {ref} rotation: board={bf.rotation:.1f} file={ff.rotation:.1f}")
        if len(bf.pads) != len(ff.pads):
            diffs.append(f"Footprint {ref} pad count: board={len(bf.pads)} file={len(ff.pads)}")
        else:
            # Compare individual pads (sorted by pad number for consistency)
            b_pads = sorted(bf.pads, key=lambda p: p.pad_number)
            f_pads = sorted(ff.pads, key=lambda p: p.pad_number)
            for bp, fp in zip(b_pads, f_pads):
                if bp.pad_number != fp.pad_number:
                    diffs.append(f"Footprint {ref} pad number mismatch: board={bp.pad_number} file={fp.pad_number}")
                    continue
                if not close(bp.global_x, fp.global_x) or not close(bp.global_y, fp.global_y):
                    diffs.append(f"Pad {ref}:{bp.pad_number} position: board=({bp.global_x:.3f},{bp.global_y:.3f}) file=({fp.global_x:.3f},{fp.global_y:.3f})")
                if bp.net_id != fp.net_id:
                    diffs.append(f"Pad {ref}:{bp.pad_number} net_id: board={bp.net_id} file={fp.net_id}")
                if bp.shape != fp.shape:
                    diffs.append(f"Pad {ref}:{bp.pad_number} shape: board={bp.shape} file={fp.shape}")
                if not close(bp.size_x, fp.size_x) or not close(bp.size_y, fp.size_y):
                    diffs.append(f"Pad {ref}:{bp.pad_number} size: board=({bp.size_x:.3f},{bp.size_y:.3f}) file=({fp.size_x:.3f},{fp.size_y:.3f})")
                if not close(bp.drill, fp.drill):
                    diffs.append(f"Pad {ref}:{bp.pad_number} drill: board={bp.drill:.3f} file={fp.drill:.3f}")
                # Compare layers (as sets since order may differ)
                if set(bp.layers) != set(fp.layers):
                    diffs.append(f"Pad {ref}:{bp.pad_number} layers: board={bp.layers} file={fp.layers}")

    # --- Compare segments ---
    if len(from_board.segments) != len(from_file.segments):
        diffs.append(f"Segment count: board={len(from_board.segments)} file={len(from_file.segments)}")

    # --- Compare vias ---
    if len(from_board.vias) != len(from_file.vias):
        diffs.append(f"Via count: board={len(from_board.vias)} file={len(from_file.vias)}")

    # --- Compare zones ---
    if len(from_board.zones) != len(from_file.zones):
        diffs.append(f"Zone count: board={len(from_board.zones)} file={len(from_file.zones)}")
    else:
        # Compare zones by net_id and layer
        b_zones = sorted(from_board.zones, key=lambda z: (z.net_id, z.layer))
        f_zones = sorted(from_file.zones, key=lambda z: (z.net_id, z.layer))
        for bz, fz in zip(b_zones, f_zones):
            if bz.net_id != fz.net_id:
                diffs.append(f"Zone net_id mismatch: board={bz.net_id} file={fz.net_id}")
            if bz.layer != fz.layer:
                diffs.append(f"Zone layer mismatch: board={bz.layer} file={fz.layer}")
            if len(bz.polygon) != len(fz.polygon):
                diffs.append(f"Zone net={bz.net_id} layer={bz.layer} vertex count: board={len(bz.polygon)} file={len(fz.polygon)}")

    return diffs


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
    x_positions = sorted(set(round(p.global_x, POSITION_DECIMALS) for p in pads))
    y_positions = sorted(set(round(p.global_y, POSITION_DECIMALS) for p in pads))

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


def detect_bga_pitch(footprint: Footprint) -> float:
    """
    Detect the pitch (pad spacing) of a BGA footprint.

    Returns:
        Pitch in mm, or 1.0 as default if cannot be detected
    """
    if not footprint.pads or len(footprint.pads) < 2:
        return 1.0

    # Get unique x and y positions
    x_positions = sorted(set(p.global_x for p in footprint.pads))
    y_positions = sorted(set(p.global_y for p in footprint.pads))

    pitches = []
    if len(x_positions) > 1:
        x_diffs = [x_positions[i+1] - x_positions[i] for i in range(len(x_positions)-1)]
        pitches.extend(x_diffs)
    if len(y_positions) > 1:
        y_diffs = [y_positions[i+1] - y_positions[i] for i in range(len(y_positions)-1)]
        pitches.extend(y_diffs)

    if pitches:
        # Use minimum pitch (most common spacing)
        return min(pitches)
    return 1.0


def auto_detect_bga_exclusion_zones(pcb_data: 'PCBData', margin: float = 0.5) -> List[Tuple[float, float, float, float, float]]:
    """
    Auto-detect BGA exclusion zones from all BGA components in the PCB.

    Via placement should be avoided inside BGA packages to prevent shorts
    with the BGA balls.

    Returns zones as 5-tuples: (min_x, min_y, max_x, max_y, edge_tolerance)
    where edge_tolerance = margin + pitch * 1.1 (pitch + 10%)

    Args:
        pcb_data: Parsed PCB data
        margin: Extra margin around BGA bounds (in mm)

    Returns:
        List of (min_x, min_y, max_x, max_y, edge_tolerance) tuples for each BGA
    """
    zones = []
    bga_components = find_components_by_type(pcb_data, 'BGA')

    for fp in bga_components:
        bounds = get_footprint_bounds(fp, margin=margin)
        pitch = detect_bga_pitch(fp)
        # Edge tolerance = margin + pitch * 1.1 (pitch + 10% for tolerance)
        edge_tolerance = margin + pitch * 1.1
        zones.append((*bounds, edge_tolerance))

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

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

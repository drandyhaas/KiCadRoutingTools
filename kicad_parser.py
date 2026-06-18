"""
KiCad PCB Parser - Extracts pads, nets, tracks, vias, and board info from .kicad_pcb files.
"""

import os
import re
import math
import json
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Tuple, Optional
from pathlib import Path


# Position rounding precision for coordinate comparisons
# All position-based lookups must use this to ensure consistency
POSITION_DECIMALS = 3

# KiCad 10 removed numeric net IDs from the file format.
# Files with version >= this threshold use name-only nets: (net "name") instead of (net 29 "name").
# KiCad 9 uses version 20241229; KiCad 10 uses version 20260206.
KICAD_10_MIN_VERSION = 20250000


def detect_kicad_version(content: str) -> int:
    """Extract version number from (version YYYYMMDD) header."""
    m = re.search(r'\(version\s+(\d+)\)', content)
    return int(m.group(1)) if m else 0


def is_kicad_10(content: str) -> bool:
    """Check if file content is KiCad 10+ format (name-only nets)."""
    return detect_kicad_version(content) >= KICAD_10_MIN_VERSION


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
    rect_rotation: float = 0.0  # Residual rect tilt in global frame for non-
    # orthogonal pads (deg, in (-90,90]). 0 for axis-aligned pads (the common
    # case) where the rotation is already baked into size_x/size_y. The
    # obstacle/DRC geometry rotates the pad rectangle by this angle.


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
class GuidePath:
    """A user-drawn graphic polyline (e.g. on User.1) used as a routing corridor."""
    layer: str  # e.g. "User.1"
    points: List[Tuple[float, float]]  # ordered (x, y) in mm, >= 2 points
    is_closed: bool = False  # True for closed polygons (gr_poly)


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
    board_cutouts: List[List[Tuple[float, float]]] = field(default_factory=list)  # Interior cutout polygons
    keepouts: List[dict] = field(default_factory=list)  # Keep-out rule areas: {polygon, layers:set, tracks_allowed, vias_allowed}


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
    kicad_version: int = 0  # File format version (e.g., 20241229 for KiCad 9)
    net_id_to_name: Dict[int, str] = field(default_factory=dict)  # Synthetic ID -> net name (for KiCad 10 output)
    # Net class data read from the sibling .kicad_pro file. kipy's
    # BoardDesignRules only exposes board-wide minimums and predefined
    # sizes — per-class settings (track width, clearance, via diameter,
    # diff-pair width/gap) live in the project JSON.
    netclass_params: Dict[str, Dict[str, float]] = field(default_factory=dict)  # class_name -> {param: mm}
    net_to_class: Dict[str, str] = field(default_factory=dict)  # net_name -> class_name
    guide_paths: List[GuidePath] = field(default_factory=list)  # User-drawn guide corridors (issue #7)
    keepout_zones: List[GuidePath] = field(default_factory=list)  # User-drawn keepout polygons (issue #27)

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


# Tolerance (deg) within which a pad rotation is treated as axis-aligned and
# baked into size_x/size_y rather than carried as a residual rect_rotation.
_PAD_ORTHO_TOL = 1.0


def _resolve_pad_rect(size_x: float, size_y: float,
                      pad_rotation_deg: float) -> Tuple[float, float, float]:
    """Resolve a pad rectangle (given in the pad's own frame) into board space.

    pad_rotation_deg is the pad's ABSOLUTE orientation: the KiCad pad ``(at ...)``
    angle already includes the footprint rotation (verified physically -- using
    pad+footprint here makes adjacent pad copper overlap on rotated parts), and
    pcbnew's GetOrientationDegrees() is likewise absolute. The footprint rotation
    still applies to the pad POSITION (see local_to_global) but NOT a second time
    to its orientation.

    Returns (size_x, size_y, rect_rotation): board-axis-aligned dimensions for
    orthogonal pads (0/90/180/270°, the overwhelmingly common case) with
    rect_rotation 0; for genuine diagonal pads the dimensions are left as given
    and rect_rotation carries the residual tilt in the GLOBAL frame, folded to
    (-90, 90]. The global frame negates the KiCad angle (see local_to_global), so
    the geometric tilt is -pad_rotation.
    """
    t = pad_rotation_deg % 180.0  # rect is symmetric under 180°
    if abs(t - 90.0) <= _PAD_ORTHO_TOL:
        return size_y, size_x, 0.0           # ~90°: swap to board axes
    if t <= _PAD_ORTHO_TOL or t >= 180.0 - _PAD_ORTHO_TOL:
        return size_x, size_y, 0.0           # ~0/180°: already board-aligned
    g = (-pad_rotation_deg) % 180.0          # global-frame tilt, folded to [0,180)
    rect_rotation = g if g <= 90.0 else g - 180.0
    return size_x, size_y, rect_rotation


def find_matching_paren(content: str, open_idx: int) -> int:
    """Return the index just past the ``)`` matching the ``(`` at ``open_idx``.

    Counts parentheses but SKIPS those inside quoted strings, honoring KiCad's
    backslash escapes (``\\"`` and ``\\\\``). A naive depth counter breaks when a
    property value contains a lone paren, e.g. an MPN like ``"TCR2EF115,LM(CT"``:
    the unmatched ``(`` makes the scan run far past the block end and swallow
    following footprints' pads (issue #113). Returns ``len(content)`` if no match
    is found.
    """
    depth = 0
    in_string = False
    i = open_idx
    n = len(content)
    while i < n:
        char = content[i]
        if in_string:
            if char == '\\':
                i += 2  # skip escaped char
                continue
            if char == '"':
                in_string = False
        else:
            if char == '"':
                in_string = True
            elif char == '(':
                depth += 1
            elif char == ')':
                depth -= 1
                if depth == 0:
                    return i + 1
        i += 1
    return n


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

    # Find the layers section. Capture lazily up to the block's closing paren on
    # its own line, rather than requiring a run of paren-free (...) entries: layer
    # user-names can contain parentheses (e.g. "In1(GND).Cu", "plate (unused)"),
    # which truncated the old regex and made such boards parse as 0 copper layers.
    layers_match = re.search(r'\(layers\b(.*?)\n[ \t]*\)', content, re.DOTALL)
    if layers_match:
        layers_text = layers_match.group(1)
        # Parse individual layer entries: (0 "F.Cu" signal)
        layer_pattern = r'\((\d+)\s+"([^"]+)"\s+(\w+)'
        for m in re.finditer(layer_pattern, layers_text):
            layer_id = int(m.group(1))
            layer_name = m.group(2)
            layer_type = m.group(3)
            layers[layer_id] = layer_name
            # Any copper layer counts, whatever its declared use. KiCad types
            # plane layers 'power' (and sometimes 'mixed'/'jumper'); restricting
            # to 'signal' dropped real layers, making a 4-layer board look
            # 2-layer (issue #76).
            if '.Cu' in layer_name and layer_type in ('signal', 'power', 'mixed', 'jumper'):
                copper_layers.append(layer_name)

    # Extract board bounds from Edge.Cuts
    bounds = extract_board_bounds(content)

    # Extract board outline polygon and cutouts for non-rectangular boards
    outline, cutouts = extract_board_contours(content)

    # Extract stackup information
    stackup = extract_stackup(content)

    return BoardInfo(layers=layers, copper_layers=copper_layers, board_bounds=bounds, stackup=stackup, board_outline=outline, board_cutouts=cutouts)


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


def _arc_to_segments(start: Tuple[float, float], mid: Tuple[float, float],
                     end: Tuple[float, float], num_segments: int = 16
                     ) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Convert a 3-point arc to a polyline of straight segments.

    Given start, mid (on arc), and end points, computes the circular arc
    and returns it as a list of (start, end) line segment tuples.
    """
    ax, ay = start
    bx, by = mid
    cx, cy = end

    # Find circle center from 3 points using perpendicular bisector intersection
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-10:
        # Degenerate (collinear points) - just return a straight line
        return [(start, end)]

    ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d
    uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d
    radius = math.hypot(ax - ux, ay - uy)

    # Compute angles
    angle_start = math.atan2(ay - uy, ax - ux)
    angle_mid = math.atan2(by - uy, bx - ux)
    angle_end = math.atan2(cy - uy, cx - ux)

    # Determine arc direction: going from start to end, mid must be on the arc
    # Normalize angles relative to start
    def normalize(a, ref):
        a = a - ref
        while a < 0:
            a += 2 * math.pi
        while a >= 2 * math.pi:
            a -= 2 * math.pi
        return a

    mid_ccw = normalize(angle_mid, angle_start)
    end_ccw = normalize(angle_end, angle_start)

    # If going CCW from start, mid should come before end
    if mid_ccw <= end_ccw:
        # CCW direction, sweep = end_ccw
        sweep = end_ccw
    else:
        # CW direction, sweep is negative
        sweep = end_ccw - 2 * math.pi

    # Generate points along the arc, using exact start/end to preserve chaining
    points = [start]
    for i in range(1, num_segments):
        t = i / num_segments
        angle = angle_start + t * sweep
        points.append((ux + radius * math.cos(angle), uy + radius * math.sin(angle)))
    points.append(end)

    # Build segments
    segments = []
    for i in range(len(points) - 1):
        segments.append((points[i], points[i + 1]))
    return segments


# Regex gap that matches anything EXCEPT the start of another graphic element,
# so a lazy match can't run past the current element to a later (layer "...")
# token. A plain `.*?` gap let a silk/fab gr_* element match across element
# boundaries to a later Edge.Cuts token, consuming the real edge elements in
# between (issue #77). Shared by the Edge.Cuts and guide-corridor readers.
_GR_ELEMENT_GAP = r'(?:(?!\(gr_)[\s\S])*?'


def extract_board_bounds(content: str) -> Optional[Tuple[float, float, float, float]]:
    """Extract board outline bounds from Edge.Cuts layer."""
    min_x = min_y = float('inf')
    max_x = max_y = float('-inf')
    found = False

    # Look for gr_rect on Edge.Cuts (multi-line format)
    rect_pattern = r'\(gr_rect\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)' + _GR_ELEMENT_GAP + r'\(layer\s+"Edge\.Cuts"\)'
    for m in re.finditer(rect_pattern, content, re.DOTALL):
        x1, y1, x2, y2 = float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))
        min_x = min(min_x, x1, x2)
        min_y = min(min_y, y1, y2)
        max_x = max(max_x, x1, x2)
        max_y = max(max_y, y1, y2)
        found = True

    # Look for gr_line on Edge.Cuts (multi-line format)
    line_pattern = r'\(gr_line\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)' + _GR_ELEMENT_GAP + r'\(layer\s+"Edge\.Cuts"\)'
    for m in re.finditer(line_pattern, content, re.DOTALL):
        x1, y1, x2, y2 = float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))
        min_x = min(min_x, x1, x2)
        min_y = min(min_y, y1, y2)
        max_x = max(max_x, x1, x2)
        max_y = max(max_y, y1, y2)
        found = True

    # Look for gr_arc on Edge.Cuts (multi-line format)
    arc_pattern = r'\(gr_arc\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(mid\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)' + _GR_ELEMENT_GAP + r'\(layer\s+"Edge\.Cuts"\)'
    for m in re.finditer(arc_pattern, content, re.DOTALL):
        sx, sy = float(m.group(1)), float(m.group(2))
        mx, my = float(m.group(3)), float(m.group(4))
        ex, ey = float(m.group(5)), float(m.group(6))
        # Use linearized arc points for accurate bounds
        for seg in _arc_to_segments((sx, sy), (mx, my), (ex, ey)):
            for px, py in seg:
                min_x = min(min_x, px)
                max_x = max(max_x, px)
                min_y = min(min_y, py)
                max_y = max(max_y, py)
        found = True

    # Look for gr_poly on Edge.Cuts (free-form board outlines)
    for poly in _parse_gr_polys_on_layer(content, "Edge.Cuts"):
        for px, py in poly:
            min_x = min(min_x, px)
            max_x = max(max_x, px)
            min_y = min(min_y, py)
            max_y = max(max_y, py)
        if poly:
            found = True

    if found:
        return (min_x, min_y, max_x, max_y)
    return None


def _collect_edge_cuts_segments(content: str) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Parse all gr_line, gr_arc, and gr_rect segments on Edge.Cuts from file content."""
    segments = []

    # gr_line
    line_pattern = r'\(gr_line\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)' + _GR_ELEMENT_GAP + r'\(layer\s+"Edge\.Cuts"\)'
    for m in re.finditer(line_pattern, content, re.DOTALL):
        x1, y1, x2, y2 = float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))
        segments.append(((x1, y1), (x2, y2)))

    # gr_arc - approximate as polyline
    arc_pattern = r'\(gr_arc\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(mid\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)' + _GR_ELEMENT_GAP + r'\(layer\s+"Edge\.Cuts"\)'
    for m in re.finditer(arc_pattern, content, re.DOTALL):
        sx, sy = float(m.group(1)), float(m.group(2))
        mx, my = float(m.group(3)), float(m.group(4))
        ex, ey = float(m.group(5)), float(m.group(6))
        segments.extend(_arc_to_segments((sx, sy), (mx, my), (ex, ey)))

    # gr_rect - expand to 4 line segments
    rect_pattern = r'\(gr_rect\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)' + _GR_ELEMENT_GAP + r'\(layer\s+"Edge\.Cuts"\)'
    for m in re.finditer(rect_pattern, content, re.DOTALL):
        x1, y1, x2, y2 = float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))
        segments.append(((x1, y1), (x2, y1)))
        segments.append(((x2, y1), (x2, y2)))
        segments.append(((x2, y2), (x1, y2)))
        segments.append(((x1, y2), (x1, y1)))

    # gr_poly - closed polygon outline, expand to line segments
    for poly in _parse_gr_polys_on_layer(content, "Edge.Cuts"):
        for i in range(len(poly)):
            segments.append((poly[i], poly[(i + 1) % len(poly)]))

    return segments


def _parse_gr_polys_on_layer(content: str, layer: str) -> List[List[Tuple[float, float]]]:
    """Return the vertex list of every gr_poly drawn on the given layer."""
    layer_re = re.escape(layer)
    pattern = (
        r'\(gr_poly\s+\(pts\s+((?:\(xy\s+[\d.-]+\s+[\d.-]+\)\s*)+)\)'
        + _GR_ELEMENT_GAP + r'\(layer\s+"' + layer_re + r'"\)'
    )
    polys = []
    for m in re.finditer(pattern, content, re.DOTALL):
        polys.append([(float(px), float(py))
                      for px, py in re.findall(r'\(xy\s+([\d.-]+)\s+([\d.-]+)\)', m.group(1))])
    return polys


def parse_guide_paths(content: str, layer: str) -> List["GuidePath"]:
    """Parse user-drawn graphic polylines on a given layer from file content.

    Reads gr_line and gr_poly graphics on the named layer (e.g. "User.1") and
    returns them as GuidePath objects. Consecutive gr_line segments whose
    endpoints coincide are stitched into a single multi-point path.

    Args:
        content: Raw .kicad_pcb file text.
        layer: Layer name to read from (e.g. "User.1").

    Returns:
        List of GuidePath (mm coordinates). Empty if none found.
    """
    layer_re = re.escape(layer)
    paths: List[GuidePath] = []

    # gr_poly: a closed polygon with a (pts (xy ..) (xy ..) ...) block.
    for points in _parse_gr_polys_on_layer(content, layer):
        if len(points) >= 2:
            paths.append(GuidePath(layer=layer, points=points, is_closed=True))

    # gr_line: collect 2-point segments, then stitch into chains.
    line_pattern = (
        r'\(gr_line\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)'
        + _GR_ELEMENT_GAP + r'\(layer\s+"' + layer_re + r'"\)'
    )
    segments = []
    for m in re.finditer(line_pattern, content, re.DOTALL):
        segments.append(((float(m.group(1)), float(m.group(2))),
                         (float(m.group(3)), float(m.group(4)))))

    paths.extend(_chain_guide_segments(segments, layer))
    return paths


def _chain_guide_segments(segments, layer: str, tol: float = 0.01) -> List["GuidePath"]:
    """Stitch line segments into open polylines by shared endpoints."""
    if not segments:
        return []

    def approx_equal(p1, p2):
        return abs(p1[0] - p2[0]) < tol and abs(p1[1] - p2[1]) < tol

    remaining = list(segments)
    paths: List[GuidePath] = []
    while remaining:
        a, b = remaining.pop(0)
        chain = [a, b]
        extended = True
        while extended:
            extended = False
            for i, (s, e) in enumerate(remaining):
                if approx_equal(chain[-1], s):
                    chain.append(e)
                elif approx_equal(chain[-1], e):
                    chain.append(s)
                elif approx_equal(chain[0], e):
                    chain.insert(0, s)
                elif approx_equal(chain[0], s):
                    chain.insert(0, e)
                else:
                    continue
                remaining.pop(i)
                extended = True
                break
        paths.append(GuidePath(layer=layer, points=chain, is_closed=False))
    return paths


def parse_keepout_zones(content: str, layer: str) -> List["GuidePath"]:
    """Parse user-drawn closed keepout polygons on a given layer from file content.

    Reads gr_poly and gr_rect graphics on the named layer (e.g. "User.2") and
    returns them as closed GuidePath objects (issue #27). Only closed regions are
    returned -- open gr_line polylines are ignored (they don't bound an area).

    Args:
        content: Raw .kicad_pcb file text.
        layer: Layer name to read from (e.g. "User.2").

    Returns:
        List of closed GuidePath (mm coordinates). Empty if none found.
    """
    layer_re = re.escape(layer)
    zones: List[GuidePath] = []

    # gr_poly: a closed polygon with a (pts (xy ..) (xy ..) ...) block.
    for points in _parse_gr_polys_on_layer(content, layer):
        if len(points) >= 3:
            zones.append(GuidePath(layer=layer, points=points, is_closed=True))

    # gr_rect: a rectangle given by opposite corners -> 4-vertex closed box.
    rect_pattern = (
        r'\(gr_rect\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)'
        + _GR_ELEMENT_GAP + r'\(layer\s+"' + layer_re + r'"\)'
    )
    for m in re.finditer(rect_pattern, content, re.DOTALL):
        x1, y1, x2, y2 = (float(m.group(1)), float(m.group(2)),
                          float(m.group(3)), float(m.group(4)))
        zones.append(GuidePath(
            layer=layer,
            points=[(x1, y1), (x2, y1), (x2, y2), (x1, y2)],
            is_closed=True))

    return zones


def _chain_segments_into_contours(segments: List[Tuple[Tuple[float, float], Tuple[float, float]]],
                                   tol: float = 0.01
                                   ) -> List[List[Tuple[float, float]]]:
    """Chain line segments into closed polygon contours.

    Groups segments by connectivity (shared endpoints), chains each group
    into a closed polygon. Returns list of polygons (each a list of vertices).
    """
    if not segments:
        return []

    def approx_equal(p1, p2):
        return abs(p1[0] - p2[0]) < tol and abs(p1[1] - p2[1]) < tol

    # Build connected components using union-find
    parent = list(range(len(segments)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        a, b = find(a), find(b)
        if a != b:
            parent[a] = b

    # Build spatial index: bucket endpoints for fast lookup
    bucket_size = tol * 2
    from collections import defaultdict
    endpoint_buckets = defaultdict(list)
    for i, seg in enumerate(segments):
        for pt in [seg[0], seg[1]]:
            bx = int(pt[0] / bucket_size)
            by = int(pt[1] / bucket_size)
            endpoint_buckets[(bx, by)].append(i)

    # Union segments that share endpoints
    for i, seg in enumerate(segments):
        for pt in [seg[0], seg[1]]:
            bx = int(pt[0] / bucket_size)
            by = int(pt[1] / bucket_size)
            # Check neighboring buckets
            for dbx in [-1, 0, 1]:
                for dby in [-1, 0, 1]:
                    for j in endpoint_buckets.get((bx + dbx, by + dby), []):
                        if j <= i:
                            continue
                        seg_j = segments[j]
                        if (approx_equal(pt, seg_j[0]) or approx_equal(pt, seg_j[1])):
                            union(i, j)

    # Group segments by component
    groups = defaultdict(list)
    for i in range(len(segments)):
        groups[find(i)].append(i)

    # Chain each component into a polygon
    contours = []
    for group_indices in groups.values():
        if len(group_indices) < 3:
            continue

        group_segs = [segments[idx] for idx in group_indices]

        # Build spatial index for this group's segments
        seg_buckets = defaultdict(list)
        for i, seg in enumerate(group_segs):
            for pt in [seg[0], seg[1]]:
                bx = int(pt[0] / bucket_size)
                by = int(pt[1] / bucket_size)
                seg_buckets[(bx, by)].append(i)

        polygon = [group_segs[0][0], group_segs[0][1]]
        used = {0}

        max_iterations = len(group_segs) * 2
        for _ in range(max_iterations):
            if len(used) >= len(group_segs):
                break
            current_end = polygon[-1]
            bx = int(current_end[0] / bucket_size)
            by = int(current_end[1] / bucket_size)
            found_next = False

            # Search nearby buckets for matching endpoint
            for dbx in [-1, 0, 1]:
                if found_next:
                    break
                for dby in [-1, 0, 1]:
                    if found_next:
                        break
                    for i in seg_buckets.get((bx + dbx, by + dby), []):
                        if i in used:
                            continue
                        seg = group_segs[i]
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

        # Remove duplicate closing point
        if len(polygon) > 1 and approx_equal(polygon[0], polygon[-1]):
            polygon = polygon[:-1]

        if len(used) == len(group_segs) and len(polygon) >= 3:
            contours.append(polygon)

    return contours


def extract_board_outline(content: str) -> List[Tuple[float, float]]:
    """Extract board outline polygon from Edge.Cuts layer.

    Parses gr_line, gr_arc, and gr_rect segments and assembles them into
    closed polygons. Returns the largest contour as the board outline.
    Returns an empty list if no outline is found or if it's a simple rectangle.
    """
    outline, _ = extract_board_contours(content)
    return outline


def extract_board_contours(content: str) -> Tuple[List[Tuple[float, float]], List[List[Tuple[float, float]]]]:
    """Extract board outline and cutout polygons from Edge.Cuts layer.

    Returns:
        (outline, cutouts) where outline is the outer boundary polygon vertices
        and cutouts is a list of interior cutout polygons.
        outline is empty if no outline found or if it's a simple axis-aligned rectangle.
    """
    segments = _collect_edge_cuts_segments(content)

    if len(segments) < 3:
        return [], []

    # Check if this is a simple 4-segment rectangle (no need for polygon handling)
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
                return [], []  # Simple rectangle, use bounding box

    contours = _chain_segments_into_contours(segments)
    if not contours:
        return [], []

    # Find the largest contour by bounding box area (outer boundary)
    def contour_bbox_area(contour):
        xs = [p[0] for p in contour]
        ys = [p[1] for p in contour]
        return (max(xs) - min(xs)) * (max(ys) - min(ys))

    contours.sort(key=contour_bbox_area, reverse=True)
    outline = contours[0]
    cutouts = contours[1:]

    return outline, cutouts


def extract_nets(content: str, kicad_version: int = 0) -> Tuple[Dict[int, Net], Dict[str, int]]:
    """Extract all net definitions.

    Returns:
        Tuple of (nets dict keyed by net_id, name_to_id mapping).
        For KiCad 9, net_id comes from the file. For KiCad 10, synthetic IDs are assigned.
    """
    nets = {}
    name_to_id: Dict[str, int] = {}

    if kicad_version >= KICAD_10_MIN_VERSION:
        # KiCad 10 removes the top-level net table entirely.
        # Discover all net names from their usage in pads, segments, vias, and zones.
        # Match (net "name") anywhere in the file — deduplicate to build the net list.
        net_pattern = r'\(net\s+"([^"]*)"\)'
        synthetic_id = 1
        for m in re.finditer(net_pattern, content):
            net_name = m.group(1)
            if net_name in name_to_id:
                continue  # Already seen
            nets[synthetic_id] = Net(net_id=synthetic_id, name=net_name)
            name_to_id[net_name] = synthetic_id
            synthetic_id += 1
    else:
        # KiCad 9: nets are (net <id> "name")
        net_pattern = r'\(net\s+(\d+)\s+"([^"]*)"\)'
        for m in re.finditer(net_pattern, content):
            net_id = int(m.group(1))
            net_name = m.group(2)
            nets[net_id] = Net(net_id=net_id, name=net_name)
            name_to_id[net_name] = net_id

    return nets, name_to_id


def extract_footprints_and_pads(content: str, nets: Dict[int, Net], name_to_id: Dict[str, int] = None) -> Tuple[Dict[str, Footprint], Dict[int, List[Pad]]]:
    """Extract footprints and their pads with global coordinates."""
    footprints = {}
    pads_by_net: Dict[int, List[Pad]] = {}

    # Find all footprints - need to handle nested parentheses properly
    # Strategy: find (footprint and then match balanced parens
    footprint_starts = [m.start() for m in re.finditer(r'\(footprint\s+"', content)]

    for start in footprint_starts:
        # Find the matching end parenthesis (string-aware: a property value with
        # a lone paren must not throw off the count — see find_matching_paren).
        end = find_matching_paren(content, start)

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

        # Extract reference. KiCad 8+ uses (property "Reference" "R1"); KiCad
        # 6/7 use (fp_text reference "R1" ...). Without the fallback every 6/7
        # footprint got reference "?" and collapsed onto one dict key, so a
        # whole board parsed as a single footprint (issue #78).
        ref_match = re.search(r'\(property\s+"Reference"\s+"([^"]+)"', fp_text)
        if not ref_match:
            ref_match = re.search(r'\(fp_text\s+reference\s+"([^"]+)"', fp_text)
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
            # Find end of this pad block (string-aware paren matching).
            pad_end = find_matching_paren(fp_text, pad_start)

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

            # Resolve the pad rectangle into board space. size_x/size_y are in
            # the pad's own frame; its absolute board orientation is the pad
            # ``(at ...)`` angle, which already includes the footprint rotation
            # (keying the swap on a relative angle misses pads whose footprint
            # supplies the 90° and models them 90° off their real shape). For the
            # common orthogonal cases bake the rotation into the dimensions so all
            # downstream axis-aligned geometry stays exact; genuine diagonal pads
            # keep a residual rect_rotation the obstacle/DRC geometry applies.
            size_x, size_y, rect_rotation = _resolve_pad_rect(size_x, size_y, pad_rotation)

            # Extract layers - use findall to get all quoted layer names
            layers_section = re.search(r'\(layers\s+([^)]+)\)', pad_text)
            pad_layers = []
            if layers_section:
                pad_layers = re.findall(r'"([^"]+)"', layers_section.group(1))

            # Extract net - try KiCad 9 format first, then KiCad 10
            net_match = re.search(r'\(net\s+(\d+)\s+"([^"]*)"\)', pad_text)
            if net_match:
                net_id = int(net_match.group(1))
                net_name = net_match.group(2)
            else:
                # KiCad 10: (net "name") with no numeric ID
                net_match_v10 = re.search(r'\(net\s+"([^"]*)"\)', pad_text)
                if net_match_v10 and name_to_id:
                    net_name = net_match_v10.group(1)
                    net_id = name_to_id.get(net_name, 0)
                else:
                    net_id = 0
                    net_name = ""

            # Extract pinfunction
            pinfunc_match = re.search(r'\(pinfunction\s+"([^"]*)"\)', pad_text)
            pinfunction = pinfunc_match.group(1) if pinfunc_match else ""

            # Extract pintype
            pintype_match = re.search(r'\(pintype\s+"([^"]*)"\)', pad_text)
            pintype = pintype_match.group(1) if pintype_match else ""

            # Extract drill size for through-hole pads. Oval/slot drills are
            # (drill oval w h); take max(w, h) so the pad keeps drill>0 (TH)
            # semantics instead of being misread as SMD (issue #106).
            oval_match = re.search(r'\(drill\s+oval\s+([\d.]+)\s+([\d.]+)', pad_text)
            if oval_match:
                drill_size = max(float(oval_match.group(1)), float(oval_match.group(2)))
            else:
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
                roundrect_rratio=roundrect_rratio,
                rect_rotation=rect_rotation
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


def extract_vias(content: str, name_to_id: Dict[str, int] = None) -> List[Via]:
    """Extract all vias from PCB file."""
    vias = []

    # Try KiCad 9 format first: (net <id>)
    # Strict field ordering: at → size → drill → layers → (free?) → net → uuid
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

    if name_to_id:
        # KiCad 10 format: (net "name"). Always run IN ADDITION to the numeric
        # pattern and merge: a file can legally mix both styles (e.g. a tool
        # that writes numeric refs into a v10 board), and each via matches
        # exactly one pattern. An either/or fallback silently dropped every
        # name-style via when any numeric one existed (issue #79).
        # Use flexible matching between layers and net to handle new v10 fields
        # (tenting, covering, plugging, capping, filling) that appear after layers.
        via_pattern_v10 = r'\(via\s+\(at\s+([\d.-]+)\s+([\d.-]+)\)\s+\(size\s+([\d.-]+)\)\s+\(drill\s+([\d.-]+)\)\s+\(layers\s+"([^"]+)"\s+"([^"]+)"\).*?\(net\s+"([^"]*)"\)\s+\(uuid\s+"([^"]+)"\)'
        for m in re.finditer(via_pattern_v10, content, re.DOTALL):
            net_name = m.group(7)
            via = Via(
                x=float(m.group(1)),
                y=float(m.group(2)),
                size=float(m.group(3)),
                drill=float(m.group(4)),
                layers=[m.group(5), m.group(6)],
                net_id=name_to_id.get(net_name, 0),
                uuid=m.group(8),
                free=False  # Parse free from content if present
            )
            vias.append(via)
        # Check for free flag in matched vias
        if vias:
            free_pattern = r'\(via\s+\(at\s+[\d.-]+\s+[\d.-]+\).*?\(free\s+yes\).*?\(uuid\s+"([^"]+)"\)'
            free_uuids = {m.group(1) for m in re.finditer(free_pattern, content, re.DOTALL)}
            for via in vias:
                if via.uuid in free_uuids:
                    via.free = True

    return vias


def extract_segments(content: str, name_to_id: Dict[str, int] = None) -> List[Segment]:
    """Extract all track segments from PCB file."""
    segments = []

    # Try KiCad 9 format first: (net <id>)
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

    if name_to_id:
        # KiCad 10 format: (net "name"). Always run IN ADDITION to the numeric
        # pattern and merge — mixed-style files are legal and each segment
        # matches exactly one pattern (issue #79).
        segment_pattern_v10 = r'\(segment\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)\s+\(width\s+([\d.-]+)\)\s+\(layer\s+"([^"]+)"\)\s+\(net\s+"([^"]*)"\)\s+\(uuid\s+"([^"]+)"\)'
        for m in re.finditer(segment_pattern_v10, content, re.DOTALL):
            net_name = m.group(7)
            segment = Segment(
                start_x=float(m.group(1)),
                start_y=float(m.group(2)),
                end_x=float(m.group(3)),
                end_y=float(m.group(4)),
                width=float(m.group(5)),
                layer=m.group(6),
                net_id=name_to_id.get(net_name, 0),
                uuid=m.group(8),
                start_x_str=m.group(1),
                start_y_str=m.group(2),
                end_x_str=m.group(3),
                end_y_str=m.group(4)
            )
            segments.append(segment)

    # Arc tracks: KiCad routes rounded corners as (arc (start)(mid)(end)...).
    # The parser otherwise drops them, fragmenting arc-routed (human) boards so
    # connectivity/clearance checks see false gaps (KiCad finds them connected).
    # Linearize each arc into straight Segments via the existing helper so the
    # copper graph is complete. (The router never emits arcs, so its own output
    # is unaffected; this only matters when ingesting hand-routed boards.)
    arc_fields = (r'\(arc\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+'
                  r'\(mid\s+([\d.-]+)\s+([\d.-]+)\)\s+'
                  r'\(end\s+([\d.-]+)\s+([\d.-]+)\)\s+'
                  r'\(width\s+([\d.-]+)\)\s+\(layer\s+"([^"]+)"\)\s+\(net\s+')

    def _append_arc(sx, sy, mx, my, ex, ey, width, layer, net_id, uuid):
        for (p0, p1) in _arc_to_segments((sx, sy), (mx, my), (ex, ey)):
            segments.append(Segment(
                start_x=p0[0], start_y=p0[1], end_x=p1[0], end_y=p1[1],
                width=width, layer=layer, net_id=net_id, uuid=uuid,
                start_x_str=repr(p0[0]), start_y_str=repr(p0[1]),
                end_x_str=repr(p1[0]), end_y_str=repr(p1[1])))

    for m in re.finditer(arc_fields + r'(\d+)\)\s+\(uuid\s+"([^"]+)"\)', content, re.DOTALL):
        _append_arc(float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)),
                    float(m.group(5)), float(m.group(6)), float(m.group(7)), m.group(8),
                    int(m.group(9)), m.group(10))
    if name_to_id:
        for m in re.finditer(arc_fields + r'"([^"]*)"\)\s+\(uuid\s+"([^"]+)"\)', content, re.DOTALL):
            _append_arc(float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)),
                        float(m.group(5)), float(m.group(6)), float(m.group(7)), m.group(8),
                        name_to_id.get(m.group(9), 0), m.group(10))

    return segments


def _iter_zone_blocks(content: str):
    """Yield the inner body of each top-level ``(zone ...)`` block.

    Zones are at the top level, indented with a single tab. Uses ``\\r?\\n`` to
    handle both Unix and Windows line endings. Each yielded string is the
    content between the opening ``(zone`` line and its matching closing paren.
    Shared by :func:`extract_zones` and :func:`extract_keepouts`.
    """
    zone_start_pattern = r'\r?\n\t\(zone\s*\r?\n'
    for start_match in re.finditer(zone_start_pattern, content):
        # Find the matching closing paren (string-aware, so a lone paren inside
        # a quoted token cannot run the scan past the zone end — see issue #113).
        # start_match begins at the newline before "(zone"; locate that "(".
        open_idx = content.index('(', start_match.start())
        zone_end = find_matching_paren(content, open_idx) - 1
        if zone_end <= open_idx:
            continue
        yield content[start_match.end():zone_end]


def extract_zones(content: str, name_to_id: Dict[str, int] = None) -> List[Zone]:
    """Extract all filled zones from PCB file.

    Parses zone definitions including their net assignment, layer, and polygon outline.
    These are used for power planes and other filled copper areas.
    """
    zones = []

    for zone_content in _iter_zone_blocks(content):
        # Extract net id - try KiCad 9 format first, then KiCad 10
        net_match = re.search(r'\(net\s+(\d+)\)', zone_content)
        if net_match:
            net_id = int(net_match.group(1))
        elif name_to_id:
            # KiCad 10: (net "name") - first net reference in zone
            net_match_v10 = re.search(r'\(net\s+"([^"]*)"\)', zone_content)
            if not net_match_v10:
                continue
            net_name_v10 = net_match_v10.group(1)
            net_id = name_to_id.get(net_name_v10, 0)
        else:
            continue

        # Extract net name
        net_name_match = re.search(r'\(net_name\s+"([^"]*)"\)', zone_content)
        if net_name_match:
            net_name = net_name_match.group(1)
        elif net_match is None and name_to_id:
            # KiCad 10: net name was already captured above
            net_name = net_name_v10
        else:
            net_name = ""

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


def extract_keepouts(content: str) -> List[dict]:
    """Extract keep-out rule areas (zones with a (keepout ...) clause and no net fill).

    These define regions where tracks and/or vias are not allowed — e.g. an
    antenna-flange RF clearance. Returns a list of dicts:
        {polygon: [(x,y),...], layers: set(layer_names),
         tracks_allowed: bool, vias_allowed: bool}
    """
    keepouts = []
    for zc in _iter_zone_blocks(content):
        # The keepout clause holds nested sub-clauses, e.g.
        #   (keepout (tracks not_allowed) (vias not_allowed) (pads allowed) ...)
        # so capture its full balanced-paren body rather than just the first ).
        ko_start = zc.find('(keepout')
        if ko_start < 0:
            continue
        pc = 0
        ko_end = ko_start
        for i in range(ko_start, len(zc)):
            if zc[i] == '(':
                pc += 1
            elif zc[i] == ')':
                pc -= 1
                if pc == 0:
                    ko_end = i
                    break
        ko_body = zc[ko_start:ko_end + 1]
        tracks_allowed = 'tracks not_allowed' not in ko_body
        vias_allowed = 'vias not_allowed' not in ko_body

        # Layers: (layers "F.Cu" "In1.Cu" ...) or single (layer "F.Cu")
        lm = re.search(r'\(layers\s+([^)]+)\)', zc) or re.search(r'\(layer\s+("[^"]+")\)', zc)
        layers = set(re.findall(r'"([^"]+)"', lm.group(1))) if lm else set()

        # Outline polygons. A rule area can have several (polygon (pts ...))
        # blocks: the first is the outer outline, any further ones are HOLES
        # (even-odd fill). An edge-keepout ring is the board rectangle with an
        # inset hole; parsing only the first polygon collapses the ring to its
        # bounding box and bans the whole board (issue #95). Capture them all.
        polys = []
        search = 0
        while True:
            p_start = zc.find('(polygon', search)
            if p_start < 0:
                break
            pc = 0
            p_end = p_start
            for i in range(p_start, len(zc)):
                if zc[i] == '(':
                    pc += 1
                elif zc[i] == ')':
                    pc -= 1
                    if pc == 0:
                        p_end = i
                        break
            pts = [(float(a), float(b)) for a, b in
                   re.findall(r'\(xy\s+([\d.-]+)\s+([\d.-]+)\)', zc[p_start:p_end + 1])]
            if len(pts) >= 3:
                polys.append(pts)
            search = p_end + 1
        if not polys:
            continue
        keepouts.append({'polygon': polys[0], 'holes': polys[1:], 'layers': layers,
                         'tracks_allowed': tracks_allowed, 'vias_allowed': vias_allowed})
    return keepouts


def parse_kicad_pcb(filepath: str, guide_layer: str = "User.1",
                    keepout_layer: str = "User.2") -> PCBData:
    """
    Parse a KiCad PCB file and extract all routing-relevant information.

    Args:
        filepath: Path to .kicad_pcb file
        guide_layer: User layer to read guide corridor polylines from (issue #7)
        keepout_layer: User layer to read keepout polygons from (issue #27)

    Returns:
        PCBData object containing all parsed data
    """
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()

    kicad_version = detect_kicad_version(content)

    # Extract components in order
    board_info = extract_layers(content)
    nets, name_to_id = extract_nets(content, kicad_version)
    footprints, pads_by_net = extract_footprints_and_pads(content, nets, name_to_id)
    vias = extract_vias(content, name_to_id)
    segments = extract_segments(content, name_to_id)
    zones = extract_zones(content, name_to_id)
    board_info.keepouts = extract_keepouts(content)
    guide_paths = parse_guide_paths(content, guide_layer)
    keepout_zones = parse_keepout_zones(content, keepout_layer)

    # Build net_id_to_name mapping for writer output
    net_id_to_name = {net_id: net.name for net_id, net in nets.items()}

    return PCBData(
        board_info=board_info,
        nets=nets,
        footprints=footprints,
        vias=vias,
        segments=segments,
        pads_by_net=pads_by_net,
        zones=zones,
        kicad_version=kicad_version,
        net_id_to_name=net_id_to_name,
        guide_paths=guide_paths,
        keepout_zones=keepout_zones
    )


def read_pin_metadata_from_kicad_pcb(board_filename: str) -> Dict[Tuple[str, str], Tuple[str, str]]:
    """Extract (component_ref, pad_number) → (pinfunction, pintype) from .kicad_pcb.

    kipy.Pad doesn't expose pin_function or pin_type, but the .kicad_pcb
    file stores them as `(pinfunction "...")` / `(pintype "...")` on each
    pad. This reader walks footprint blocks with a regex pass so the
    power-net / high-speed-net / plan-routing skills get real data
    without parsing the whole board file.

    Returns {} on any failure — callers should treat absence as "all pads
    have empty pinfunction/pintype".
    """
    if not board_filename or not os.path.isfile(board_filename):
        return {}
    try:
        with open(board_filename, "r", encoding="utf-8") as f:
            content = f.read()
    except Exception:
        return {}

    result: Dict[Tuple[str, str], Tuple[str, str]] = {}
    # Walk each (footprint ...) block by paren-matching from the opening
    # token, so we don't get confused by nested s-expressions.
    fp_marker = "(footprint "
    pos = 0
    while True:
        start = content.find(fp_marker, pos)
        if start < 0:
            break
        # Find the matching close-paren for this footprint block.
        depth = 0
        end = start
        for i in range(start, len(content)):
            c = content[i]
            if c == "(":
                depth += 1
            elif c == ")":
                depth -= 1
                if depth == 0:
                    end = i + 1
                    break
        block = content[start:end]
        pos = end

        # Reference: (property "Reference" "U1" ...) — newer KiCad, or
        # (fp_text reference "U1" ...) — older.
        ref_match = (re.search(r'\(property\s+"Reference"\s+"([^"]*)"', block)
                     or re.search(r'\(fp_text\s+reference\s+"([^"]*)"', block))
        if not ref_match:
            continue
        ref = ref_match.group(1)

        # Iterate pad blocks inside this footprint. (pad "N" SHAPE ...).
        # Use the same paren walker scoped to the footprint block.
        ppos = 0
        pad_marker = "(pad "
        while True:
            pstart = block.find(pad_marker, ppos)
            if pstart < 0:
                break
            pdepth = 0
            pend = pstart
            for i in range(pstart, len(block)):
                c = block[i]
                if c == "(":
                    pdepth += 1
                elif c == ")":
                    pdepth -= 1
                    if pdepth == 0:
                        pend = i + 1
                        break
            pad_block = block[pstart:pend]
            ppos = pend

            num_m = re.match(r'\(pad\s+"([^"]*)"', pad_block)
            if not num_m:
                continue
            pad_number = num_m.group(1)
            pinfunc_m = re.search(r'\(pinfunction\s+"([^"]*)"\)', pad_block)
            pintype_m = re.search(r'\(pintype\s+"([^"]*)"\)', pad_block)
            pinfunc = pinfunc_m.group(1) if pinfunc_m else ""
            pintype = pintype_m.group(1) if pintype_m else ""
            if pinfunc or pintype:
                result[(ref, pad_number)] = (pinfunc, pintype)

    return result


def read_netclasses_from_kicad_pro(board_filename: str) -> Tuple[Dict[str, Dict[str, float]], List[Tuple[str, str]], Dict[str, str]]:
    """Parse net-class definitions out of the .kicad_pro project JSON.

    kipy.BoardDesignRules exposes board-wide minimums and predefined sizes
    but NOT per-net-class settings (track width, clearance, via diameter,
    etc.) — those live in the sibling .kicad_pro file under
    `net_settings`. This helper reads them so the routing dialog's
    "use net class definitions" flow can pull real values.

    Args:
        board_filename: Absolute path to a .kicad_pcb file (or its
            project's .kicad_pro). If neither is reachable, returns empty
            mappings so callers can degrade gracefully.

    Returns:
        (classes_by_name, patterns, explicit_assignments) where
          - classes_by_name: dict of class_name -> {track_width, clearance,
                                                     via_size, via_drill,
                                                     diff_pair_width,
                                                     diff_pair_gap, ...}
            (values in mm)
          - patterns: list of (glob_pattern, class_name) for
            netclass_patterns assignments (order matters; first match wins)
          - explicit_assignments: dict net_name -> class_name for any
            explicit netclass_assignments entries
    """
    if not board_filename:
        return {}, [], {}
    # .kicad_pcb → .kicad_pro alongside it.
    pcb_path = Path(board_filename)
    if pcb_path.suffix == ".kicad_pro":
        pro_path = pcb_path
    else:
        pro_path = pcb_path.with_suffix(".kicad_pro")
    if not pro_path.is_file():
        return {}, [], {}

    try:
        with open(pro_path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception as e:
        print(f"read_netclasses_from_kicad_pro: failed to read {pro_path}: {e}")
        return {}, [], {}

    net_settings = (data.get("net_settings") or {})
    raw_classes = net_settings.get("classes") or []
    classes_by_name: Dict[str, Dict[str, float]] = {}
    for c in raw_classes:
        name = c.get("name") or ""
        if not name:
            continue
        # All numeric fields the routing code consumes. Keep keys aligned
        # with what _get_netclass_parameters previously returned.
        params: Dict[str, float] = {}
        for src_key, dst_key in (
            ("track_width", "track_width"),
            ("clearance", "clearance"),
            ("via_diameter", "via_size"),
            ("via_drill", "via_drill"),
            ("diff_pair_width", "diff_pair_width"),
            ("diff_pair_gap", "diff_pair_gap"),
            ("microvia_diameter", "microvia_size"),
            ("microvia_drill", "microvia_drill"),
        ):
            v = c.get(src_key)
            if isinstance(v, (int, float)):
                params[dst_key] = float(v)
        classes_by_name[name] = params

    patterns: List[Tuple[str, str]] = []
    for entry in (net_settings.get("netclass_patterns") or []):
        pat = entry.get("pattern") or ""
        cls = entry.get("netclass") or ""
        if pat and cls:
            patterns.append((pat, cls))

    explicit: Dict[str, str] = {}
    raw_assigns = net_settings.get("netclass_assignments")
    if isinstance(raw_assigns, dict):
        for net_name, cls in raw_assigns.items():
            if isinstance(net_name, str) and isinstance(cls, str):
                explicit[net_name] = cls

    return classes_by_name, patterns, explicit


def _resolve_net_to_class(net_name: str, patterns: List[Tuple[str, str]],
                         explicit: Dict[str, str]) -> str:
    """Match a net to its class using KiCad's lookup order.

    1. explicit assignment in `netclass_assignments`
    2. first matching glob in `netclass_patterns`
    3. fallback 'Default'
    """
    if not net_name:
        return "Default"
    if net_name in explicit:
        return explicit[net_name]
    from fnmatch import fnmatchcase
    for pat, cls in patterns:
        try:
            if fnmatchcase(net_name, pat):
                return cls
        except Exception:
            continue
    return "Default"


def build_pcb_data_from_board(board, guide_layer: str = "User.1",
                              keepout_layer: str = "User.2") -> PCBData:
    """Build PCBData directly from a kipy (KiCad IPC) board object.

    Reads from the running KiCad's in-memory board over the IPC socket
    rather than parsing the .kicad_pcb file from disk. Faster than
    parse_kicad_pcb() and reflects any unsaved changes.

    User-layer guide corridors (#7) and keepout polygons (#27) are read live
    from the running board via the kipy graphics API, so unsaved drawings are
    honoured.

    Args:
        board: A kipy.board.Board object (from KiCad().get_board())
        guide_layer: User layer to read guide corridor polylines from (issue #7)
        keepout_layer: User layer to read keepout polygons from (issue #27)

    Returns:
        PCBData object containing all routing-relevant data
    """
    from kicad_ipc_adapter import layer_maps, layer_name_for, ipc_lock
    # All board.* reads share one ipc_lock acquisition so the snapshot is
    # consistent and we don't interleave with a concurrent apply commit.
    with ipc_lock():
        return _build_pcb_data_from_board_impl(board)


def _build_pcb_data_from_board_impl(board) -> PCBData:
    """Inner impl — caller is responsible for holding ipc_lock."""
    from kicad_ipc_adapter import layer_maps, layer_name_for

    name_to_bl, bl_to_name = layer_maps()

    def get_layer_name(bl):
        return bl_to_name.get(bl) or layer_name_for(bl)

    # --- Build copper-layer info ---
    # `get_enabled_layers()` returns the actual stackup (e.g. just F.Cu+B.Cu
    # on a 2-layer board), which is what the router needs. Fall back to
    # F.Cu/B.Cu only if we can't read the stackup at all.
    layers_dict: Dict[int, str] = {}
    copper_layers: List[str] = []
    enabled = None
    try:
        enabled = set(board.get_enabled_layers() or [])
    except Exception as e:
        print(f"Warning: board.get_enabled_layers() failed ({e}); "
              "defaulting to F.Cu/B.Cu")

    for name, bl in name_to_bl.items():
        if not name.endswith(".Cu"):
            continue
        # Only include layers actually enabled in this board's stackup.
        if enabled is not None:
            if bl not in enabled:
                continue
        else:
            if name not in ("F.Cu", "B.Cu"):
                continue
        layers_dict[int(bl) if hasattr(bl, "__int__") else hash(bl)] = name
        if name not in copper_layers:
            copper_layers.append(name)

    # --- Nets ---
    # kipy.Net.code is deprecated in KiCad 10 and returns unreliable values
    # (often the same code for different nets, or 0 for everything). Key
    # everything by `.name` instead — that's the canonical identifier
    # kicad-python guarantees. We synthesise a unique integer per net so
    # the rest of the codebase, which speaks in net_id ints, keeps working.
    nets: Dict[int, Net] = {}
    net_name_to_id: Dict[str, int] = {"": 0}  # reserve 0 for "no net"
    try:
        net_list = list(board.get_nets())
    except Exception as e:
        print(f"Warning: board.get_nets() failed: {e}")
        net_list = []
    for n in net_list:
        name = getattr(n, "name", "") or ""
        if not name:
            continue
        if name in net_name_to_id:
            continue  # duplicate name from kipy — keep the first
        net_id = 1 + len(net_name_to_id)  # 1, 2, 3, ... (0 reserved)
        net_name_to_id[name] = net_id
        nets[net_id] = Net(net_id=net_id, name=name)
    print(f"build_pcb_data_from_board: loaded {len(nets)} nets, "
          f"{len(copper_layers)} copper layers: {copper_layers}")

    def resolve_net(net_obj) -> Tuple[int, str]:
        """Resolve a kipy Net object to (net_id, net_name) by name lookup.

        Returns (0, '') for the no-net / unconnected case (pads on board
        with no electrical connection assigned).
        """
        if net_obj is None:
            return (0, "")
        name = getattr(net_obj, "name", "") or ""
        return (net_name_to_id.get(name, 0), name)

    # --- Drawings (Edge.Cuts bounding box + outline polygons) ---
    edge_cuts_bl = name_to_bl.get("Edge.Cuts")
    drawings = []
    try:
        drawings = list(board.get_shapes())
    except AttributeError:
        try:
            drawings = list(board.get_drawings())  # type: ignore[attr-defined]
        except Exception:
            drawings = []

    board_bounds = _compute_edge_cuts_bbox_kipy(drawings, edge_cuts_bl)
    board_outline, board_cutouts = _extract_board_contours_kipy(drawings, edge_cuts_bl)

    # Stackup (best-effort via kipy; falls back to parsing the file)
    stackup = _extract_stackup_kipy(board)

    board_info = BoardInfo(
        layers=layers_dict,
        copper_layers=copper_layers,
        board_bounds=board_bounds,
        stackup=stackup,
        board_outline=board_outline,
        board_cutouts=board_cutouts
    )

    # --- Footprints and pads ---
    footprints: Dict[str, Footprint] = {}
    pads_by_net: Dict[int, List[Pad]] = {}

    try:
        fp_iter = board.get_footprints()
    except Exception:
        fp_iter = []

    for fp in fp_iter:
        reference = _fp_reference(fp)
        fp_name = _fp_library_name(fp)
        fp_x, fp_y = _vec_to_xy_mm(_fp_position(fp))
        fp_rotation = _fp_orientation_deg(fp)
        fp_layer = get_layer_name(getattr(fp, "layer", None))
        fp_value = _fp_value(fp)

        footprint = Footprint(
            reference=reference,
            footprint_name=fp_name,
            x=fp_x,
            y=fp_y,
            rotation=fp_rotation,
            layer=fp_layer,
            value=fp_value
        )

        for pad in _fp_pads(fp):
            pad_obj = _build_pad_from_kipy(pad, reference, fp_x, fp_y, fp_rotation,
                                            get_layer_name, resolve_net,
                                            copper_layers)
            if pad_obj is None:
                continue
            footprint.pads.append(pad_obj)
            pads_by_net.setdefault(pad_obj.net_id, []).append(pad_obj)
            if pad_obj.net_id in nets:
                nets[pad_obj.net_id].pads.append(pad_obj)

        footprints[reference] = footprint

    # --- Segments (tracks) ---
    segments: List[Segment] = []
    try:
        track_iter = board.get_tracks()
    except Exception:
        track_iter = []
    for track in track_iter:
        try:
            start_x, start_y = _vec_to_xy_mm(track.start)
            end_x, end_y = _vec_to_xy_mm(track.end)
            net_id, _ = resolve_net(getattr(track, "net", None))
            seg = Segment(
                start_x=start_x, start_y=start_y,
                end_x=end_x, end_y=end_y,
                width=_nm_to_mm(getattr(track, "width", 0)),
                layer=get_layer_name(getattr(track, "layer", None)),
                net_id=net_id,
            )
            segments.append(seg)
        except Exception:
            continue

    # --- Vias ---
    vias: List[Via] = []
    try:
        via_iter = board.get_vias()
    except Exception:
        via_iter = []
    for v in via_iter:
        try:
            x, y = _vec_to_xy_mm(v.position)
            # Via layer span lives on padstack.drill, not directly on the
            # via object. Through-hole vias fall back to F.Cu/B.Cu when
            # the kipy version doesn't expose them.
            top_bl = name_to_bl["F.Cu"]
            bot_bl = name_to_bl["B.Cu"]
            drill_obj = getattr(getattr(v, "padstack", None), "drill", None)
            if drill_obj is not None:
                top_bl = getattr(drill_obj, "start_layer", None) or top_bl
                bot_bl = getattr(drill_obj, "end_layer", None) or bot_bl
            net_id, _ = resolve_net(getattr(v, "net", None))
            vias.append(Via(
                x=x, y=y,
                size=_nm_to_mm(getattr(v, "diameter", 0)),
                drill=_nm_to_mm(getattr(v, "drill_diameter", 0)),
                layers=[get_layer_name(top_bl), get_layer_name(bot_bl)],
                net_id=net_id,
            ))
        except Exception:
            continue

    # --- Zones ---
    zones = _extract_zones_kipy(board, get_layer_name, resolve_net)

    # IPC plugins only run on KiCad 10+, so unconditionally flag this as
    # K10-format data. route_planes / output_writer / route_disconnected_planes
    # gate their "use net names instead of integer net codes" branches on
    # this; without it they fall back to writing integer codes (our
    # synthetic ones, which never match what KiCad expects).
    net_id_to_name = {nid: n.name for nid, n in nets.items()}

    # Net-class settings live in the sibling .kicad_pro file (kipy doesn't
    # expose per-class values). Best-effort; an empty result is fine.
    netclass_params: Dict[str, Dict[str, float]] = {}
    net_to_class: Dict[str, str] = {}
    try:
        from kicad_ipc_adapter import get_board_full_path
        board_path = get_board_full_path()
    except Exception:
        board_path = None
    if board_path:
        try:
            classes_by_name, patterns, explicit = read_netclasses_from_kicad_pro(board_path)
            netclass_params = classes_by_name
            for n in nets.values():
                if n.name:
                    net_to_class[n.name] = _resolve_net_to_class(
                        n.name, patterns, explicit)
        except Exception as e:
            print(f"build_pcb_data_from_board: netclass read failed: {e}")

        # kipy.Pad doesn't expose pin_function / pin_type, so pull them
        # from the .kicad_pcb file. Power-net detection and the
        # high-speed-net / plan-routing skills rely on these.
        try:
            pin_meta = read_pin_metadata_from_kicad_pcb(board_path)
        except Exception as e:
            print(f"build_pcb_data_from_board: pin metadata read failed: {e}")
            pin_meta = {}
        if pin_meta:
            for ref, fp in footprints.items():
                for pad_obj in fp.pads:
                    meta = pin_meta.get((ref, pad_obj.pad_number))
                    if meta:
                        pad_obj.pinfunction, pad_obj.pintype = meta

    # --- User-layer guide corridors (#7) and keepout polygons (#27) ---
    # Read live from the running board over IPC (reflects unsaved edits), via
    # the kipy graphics API. Best-effort; empty on any failure.
    guide_paths = []
    keepout_zones = []
    try:
        from kicad_ipc_adapter import read_guide_paths, read_keepout_zones
        guide_paths = read_guide_paths(board, guide_layer)
        keepout_zones = read_keepout_zones(board, keepout_layer)
    except Exception as e:
        print(f"build_pcb_data_from_board: guide/keepout read failed: {e}")

    return PCBData(
        board_info=board_info,
        nets=nets,
        footprints=footprints,
        vias=vias,
        segments=segments,
        pads_by_net=pads_by_net,
        zones=zones,
        kicad_version=KICAD_10_MIN_VERSION,
        net_id_to_name=net_id_to_name,
        netclass_params=netclass_params,
        net_to_class=net_to_class,
        guide_paths=guide_paths,
        keepout_zones=keepout_zones,
    )


# --- kipy helpers used by build_pcb_data_from_board ----------------------

def _nm_to_mm(value) -> float:
    """Convert a nanometre integer (kipy's internal unit) to millimetres."""
    if value is None:
        return 0.0
    try:
        return float(value) / 1_000_000.0
    except (TypeError, ValueError):
        return 0.0


def _vec_to_xy_mm(vec) -> Tuple[float, float]:
    """Read a kipy Vector2 as (x_mm, y_mm), tolerating nm-only fallbacks."""
    if vec is None:
        return (0.0, 0.0)
    x_mm = getattr(vec, "x_mm", None)
    y_mm = getattr(vec, "y_mm", None)
    if x_mm is not None and y_mm is not None:
        return (float(x_mm), float(y_mm))
    return (_nm_to_mm(getattr(vec, "x", 0)), _nm_to_mm(getattr(vec, "y", 0)))


# NOTE: build_pcb_data_from_board threads a `resolve_net(net) -> (id, name)`
# closure into every helper that needs to translate kipy Net objects into
# our synthetic-id space. kipy.Net.code is deprecated and unreliable in
# KiCad 10 — relying on it silently sends every read to net_id=0.
# Callers MUST always go through `resolve_net`; the helper above exists
# only as a fail-loud guard against future regressions.

def _net_code(_net) -> int:  # pragma: no cover - deliberate footgun guard
    raise RuntimeError(
        "_net_code() was called — this used to rely on kipy.Net.code "
        "which is deprecated in KiCad 10. Use the resolve_net closure "
        "from build_pcb_data_from_board (resolves by net name) instead."
    )


def _fp_reference(fp) -> str:
    """Extract the footprint's reference designator (e.g. 'U1', 'C2').

    kipy chains the reference through three wrappers:
        fp.reference_field            -> Field
        Field.text                    -> BoardText
        BoardText.value               -> str
    Older releases sometimes flatten this. Probe defensively, returning
    the first plain string we find by descending into .value / .text.
    """
    candidates = (
        getattr(fp, "reference", None),
        getattr(fp, "reference_field", None),
    )
    for v in candidates:
        for _ in range(4):  # cap recursion depth
            if v is None:
                break
            if isinstance(v, str):
                return v
            # Prefer .value (BoardText / Field text); fall back to .text
            # (Field wrapping a BoardText).
            nxt = getattr(v, "value", None)
            if nxt is None:
                nxt = getattr(v, "text", None)
            if nxt is v:  # avoid infinite loop if a wrapper is self-referential
                break
            v = nxt
    return ""


def _fp_library_name(fp) -> str:
    """Best-effort 'library:item' name for a kipy footprint."""
    defn = getattr(fp, "definition", None)
    for src in (defn, fp):
        if src is None:
            continue
        for attr in ("library_link", "name", "id", "footprint_id"):
            v = getattr(src, attr, None)
            if isinstance(v, str) and v:
                return v
            if v is not None:
                try:
                    return str(v)
                except Exception:
                    continue
    return ""


def _fp_position(fp):
    return getattr(fp, "position", None)


def _fp_orientation_deg(fp) -> float:
    o = getattr(fp, "orientation", None)
    if o is None:
        return 0.0
    for attr in ("degrees", "as_degrees", "deg"):
        v = getattr(o, attr, None)
        if callable(v):
            try:
                return float(v())
            except Exception:
                continue
        if isinstance(v, (int, float)):
            return float(v)
    try:
        return float(o)
    except (TypeError, ValueError):
        return 0.0


def _fp_value(fp) -> str:
    """Extract the footprint's value field (e.g. '10k', '0.1uF').

    Same Field → BoardText → value chain as _fp_reference.
    """
    candidates = (
        getattr(fp, "value", None),
        getattr(fp, "value_field", None),
    )
    for v in candidates:
        for _ in range(4):
            if v is None:
                break
            if isinstance(v, str):
                return v
            nxt = getattr(v, "value", None)
            if nxt is None:
                nxt = getattr(v, "text", None)
            if nxt is v:
                break
            v = nxt
    return ""


def _fp_pads(fp):
    """Iterate pads on a kipy FootprintInstance regardless of API shape."""
    # Newer kipy: footprint exposes .pads() / .pads attribute directly
    pads = getattr(fp, "pads", None)
    if callable(pads):
        try:
            return list(pads())
        except Exception:
            pass
    elif pads is not None:
        try:
            return list(pads)
        except Exception:
            pass
    # Older / definition-based: pads live under .definition.pads
    defn = getattr(fp, "definition", None)
    if defn is not None:
        defn_pads = getattr(defn, "pads", None)
        if callable(defn_pads):
            try:
                return list(defn_pads())
            except Exception:
                pass
        elif defn_pads is not None:
            try:
                return list(defn_pads)
            except Exception:
                pass
    return []


def _build_pad_from_kipy(pad, reference: str, fp_x: float, fp_y: float,
                         fp_rotation: float, get_layer_name,
                         resolve_net,
                         board_copper_layer_names: List[str]) -> Optional[Pad]:
    """Translate a kipy pad into the project's Pad dataclass.

    board_copper_layer_names is the list of enabled copper layer names on
    the board (e.g. ['F.Cu', 'In1.Cu', 'B.Cu']); through-hole pads
    advertise themselves on every one of these layers.
    """
    try:
        pad_num = str(getattr(pad, "number", "") or "")

        # Position
        pad_pos = getattr(pad, "position", None)
        global_x, global_y = _vec_to_xy_mm(pad_pos)
        local_pos = getattr(pad, "position_relative", None)
        if local_pos is not None:
            local_x, local_y = _vec_to_xy_mm(local_pos)
        else:
            local_x, local_y = _global_to_local(fp_x, fp_y, fp_rotation,
                                                global_x, global_y)

        # kipy's Padstack stores size + shape per-copper-layer on a
        # PadStackLayer (not directly on the padstack). For most pads the
        # F.Cu / B.Cu / inner-Cu copies are identical; we read from the
        # first copper layer.
        padstack = getattr(pad, "padstack", None) or pad
        copper_layers = []
        try:
            copper_layers = list(getattr(padstack, "copper_layers", None) or [])
        except Exception:
            copper_layers = []
        primary_layer = copper_layers[0] if copper_layers else None

        if primary_layer is not None:
            size_vec = getattr(primary_layer, "size", None)
            if size_vec is not None:
                size_x, size_y = _vec_to_xy_mm(size_vec)
            else:
                size_x = size_y = 0.0
            shape_attr = getattr(primary_layer, "shape", None)
            roundrect_rratio = float(getattr(primary_layer, "corner_rounding_ratio", 0.0) or 0.0)
        else:
            # Defensive fallback: older kipy releases or unusual pads where
            # the padstack exposed size directly.
            size = getattr(padstack, "size", None)
            if size is not None:
                size_x, size_y = _vec_to_xy_mm(size)
            else:
                size_x = _nm_to_mm(getattr(padstack, "size_x", 0))
                size_y = _nm_to_mm(getattr(padstack, "size_y", 0))
            shape_attr = getattr(padstack, "shape", None)
            roundrect_rratio = float(getattr(padstack, "corner_radius_ratio", 0.0) or 0.0)

        if shape_attr is None:
            shape = "rect"
        else:
            name = getattr(shape_attr, "name", None)
            if name:
                shape = _pad_shape_label(str(name).lower())
            else:
                shape = _pad_shape_label(str(shape_attr).lower())

        pad_layers = _pad_layer_names(pad, padstack, get_layer_name,
                                       copper_layers, board_copper_layer_names)

        net = getattr(pad, "net", None)
        net_id, net_name = resolve_net(net)

        # kipy's pad rotation lives on padstack.angle (an Angle wrapper);
        # older releases sometimes exposed it as pad.orientation.
        pad_rotation = 0.0
        rot_obj = None
        for src in (padstack, pad):
            if src is None:
                continue
            for attr in ("angle", "orientation"):
                candidate = getattr(src, attr, None)
                if candidate is not None:
                    rot_obj = candidate
                    break
            if rot_obj is not None:
                break
        if rot_obj is not None:
            for attr in ("degrees", "as_degrees", "deg"):
                v = getattr(rot_obj, attr, None)
                if callable(v):
                    try:
                        pad_rotation = float(v())
                        break
                    except Exception:
                        continue
                elif isinstance(v, (int, float)):
                    pad_rotation = float(v)
                    break
            else:
                try:
                    pad_rotation = float(rot_obj)
                except (TypeError, ValueError):
                    pad_rotation = 0.0

        total_rotation = (pad_rotation + fp_rotation) % 360
        # Padstack size is in pad-local coordinates; resolve it into board space
        # (swaps near 90° and carries a residual rect_rotation for diagonal pads),
        # mirroring the text-parser path so GUI/IPC geometry matches the CLI.
        size_x, size_y, rect_rotation = _resolve_pad_rect(size_x, size_y, pad_rotation)

        pinfunction = getattr(pad, "pin_function", "") or ""
        pintype = getattr(pad, "pin_type", "") or ""

        # kipy Padstack: drill = DrillProperties { diameter: Vector2 }.
        # `diameter.x` (== .y for round drills) is the bit diameter.
        drill = 0.0
        drill_obj = getattr(padstack, "drill", None) or getattr(padstack, "drill_diameter", None)
        if drill_obj is not None:
            diameter = getattr(drill_obj, "diameter", None) or drill_obj
            if hasattr(diameter, "x_mm"):
                drill = float(diameter.x_mm)
            elif hasattr(diameter, "x"):
                drill = _nm_to_mm(diameter.x)
            else:
                try:
                    drill = _nm_to_mm(diameter)
                except Exception:
                    drill = 0.0

        return Pad(
            component_ref=reference,
            pad_number=pad_num,
            global_x=global_x, global_y=global_y,
            local_x=local_x, local_y=local_y,
            size_x=size_x, size_y=size_y,
            shape=shape,
            layers=pad_layers,
            net_id=net_id,
            net_name=net_name,
            rotation=total_rotation,
            pinfunction=pinfunction,
            pintype=pintype,
            drill=drill,
            roundrect_rratio=roundrect_rratio,
            rect_rotation=rect_rotation,
        )
    except Exception as e:
        print(f"Warning: failed to read pad {getattr(pad, 'number', '?')}: {e}")
        return None


def _pad_shape_label(raw: str) -> str:
    """Normalise a kipy pad-shape name into the strings the rest of the code uses."""
    raw = raw.lower()
    if "circle" in raw:
        return "circle"
    if "oval" in raw:
        return "oval"
    if "roundrect" in raw or "round_rect" in raw or "chamfered" in raw:
        return "roundrect"
    if "trapezoid" in raw:
        return "trapezoid"
    if "custom" in raw:
        return "custom"
    return "rect"


def _pad_layer_names(pad, padstack, get_layer_name, copper_layers,
                     all_copper_layer_names) -> List[str]:
    """Recover the layer-name list for a kipy pad.

    kipy doesn't expose the wildcard *.Cu / *.Mask / *.Paste forms that the
    .kicad_pcb file format uses. We seed from `padstack.copper_layers[*]`
    (the explicit per-layer entries), then expand for through-hole pads
    where the copper must span every copper layer in the stackup — kipy
    sometimes only stores the F.Cu PadStackLayer for a simple THT pad
    even though the pad punctures every copper layer.
    """
    layers: List[str] = []
    for cl in (copper_layers or []):
        bl = getattr(cl, "layer", None)
        if bl is None:
            continue
        name = get_layer_name(bl)
        if name and name not in layers:
            layers.append(name)

    # Through-hole detection by drill > 0.
    drill_obj = getattr(padstack, "drill", None) or getattr(padstack, "drill_diameter", None)
    drill_mm = 0.0
    if drill_obj is not None:
        diameter = getattr(drill_obj, "diameter", None) or drill_obj
        try:
            if hasattr(diameter, "x_mm"):
                drill_mm = float(diameter.x_mm)
            elif hasattr(diameter, "x"):
                drill_mm = _nm_to_mm(diameter.x)
        except Exception:
            drill_mm = 0.0

    if drill_mm > 0:
        # THT: pad copper exists on every copper layer in the stackup,
        # plus masks on both sides. (No paste — through-hole pads aren't
        # soldered with paste/reflow.)
        for cu_name in (all_copper_layer_names or []):
            if cu_name not in layers:
                layers.append(cu_name)
        for extra in ("F.Mask", "B.Mask"):
            if extra not in layers:
                layers.append(extra)
    else:
        # SMD: mask + paste on the same side as the (single) copper layer.
        if "F.Cu" in layers:
            for extra in ("F.Mask", "F.Paste"):
                if extra not in layers:
                    layers.append(extra)
        elif "B.Cu" in layers:
            for extra in ("B.Mask", "B.Paste"):
                if extra not in layers:
                    layers.append(extra)

    if not layers:
        # Defensive last resort if we couldn't read copper_layers at all.
        layers = ["F.Cu", "F.Mask", "F.Paste"]
    return layers


def _compute_edge_cuts_bbox_kipy(drawings, edge_cuts_bl) -> Optional[Tuple[float, float, float, float]]:
    """Compute bounding box of Edge.Cuts drawings from a list of kipy shapes."""
    if not drawings or edge_cuts_bl is None:
        return None
    bmin_x = bmin_y = float("inf")
    bmax_x = bmax_y = float("-inf")
    found = False
    for d in drawings:
        layer = getattr(d, "layer", None)
        # Compare both as enum and as int value — kipy may return either
        # depending on whether `.layer` is the proto field or a wrapper.
        if not (layer == edge_cuts_bl
                or (hasattr(layer, "value") and hasattr(edge_cuts_bl, "value")
                    and layer.value == edge_cuts_bl.value)
                or (isinstance(layer, int) and hasattr(edge_cuts_bl, "value")
                    and layer == edge_cuts_bl.value)):
            continue
        for x, y in _shape_extreme_points_kipy(d):
            bmin_x = min(bmin_x, x); bmax_x = max(bmax_x, x)
            bmin_y = min(bmin_y, y); bmax_y = max(bmax_y, y)
            found = True
    if not found or bmax_x <= bmin_x or bmax_y <= bmin_y:
        return None
    return (bmin_x, bmin_y, bmax_x, bmax_y)


def _shape_extreme_points_kipy(shape) -> List[Tuple[float, float]]:
    """Yield (x_mm, y_mm) for every vertex on a kipy graphic shape.

    Handles BoardShape segment/arc/rect/circle (start/end/center/mid)
    plus BoardPolygon (.polygons[*].outline as a PolyLine of nodes).
    """
    pts: List[Tuple[float, float]] = []
    for attr in ("start", "end", "center", "mid", "midpoint"):
        v = getattr(shape, attr, None)
        if v is not None and (hasattr(v, "x_mm") or hasattr(v, "x")):
            try:
                pts.append(_vec_to_xy_mm(v))
            except Exception:
                continue
    # BoardPolygon: kipy stores closed polygons as a sequence of
    # PolygonWithHoles, each with a PolyLine outline of PolyLineNodes.
    polys = getattr(shape, "polygons", None)
    if polys is not None:
        try:
            for pwh in polys:
                pts.extend(_polyline_points_mm(getattr(pwh, "outline", None)))
        except Exception as e:
            print(f"_shape_extreme_points: BoardPolygon walk failed: {e}")
    return pts


def _polyline_points_mm(polyline) -> List[Tuple[float, float]]:
    """Extract (x_mm, y_mm) from every node of a kipy PolyLine.

    PolyLineNode wraps a Vector2 — either exposed as .point or directly as
    .x/.y (nm) or .x_mm/.y_mm. Probe defensively across kipy versions.
    """
    out: List[Tuple[float, float]] = []
    if polyline is None:
        return out
    try:
        nodes = list(polyline)
    except TypeError:
        nodes = list(getattr(polyline, "nodes", None) or [])
    for node in nodes:
        candidate = getattr(node, "point", node)
        if hasattr(candidate, "x_mm") and hasattr(candidate, "y_mm"):
            try:
                out.append((float(candidate.x_mm), float(candidate.y_mm)))
                continue
            except Exception:
                pass
        if hasattr(candidate, "x") and hasattr(candidate, "y"):
            try:
                out.append((_nm_to_mm(candidate.x), _nm_to_mm(candidate.y)))
            except Exception:
                continue
    return out


def _extract_board_contours_kipy(drawings, edge_cuts_bl):
    """Chain Edge.Cuts shapes into (outline, [cutouts]) polygons.

    Handles both `BoardShape` (segment/rect/arc/circle, fed through the
    segment chainer) and `BoardPolygon` (already a closed loop of points;
    used as a contour directly).
    """
    if not drawings or edge_cuts_bl is None:
        return [], []

    segments = []
    closed_contours: List[List[Tuple[float, float]]] = []
    for d in drawings:
        if getattr(d, "layer", None) != edge_cuts_bl:
            continue
        # BoardPolygon — the outline is already a closed loop.
        polys = getattr(d, "polygons", None)
        if polys is not None:
            try:
                for pwh in polys:
                    pts = _polyline_points_mm(getattr(pwh, "outline", None))
                    if len(pts) >= 3:
                        closed_contours.append(pts)
            except Exception:
                pass
            continue

        try:
            start = getattr(d, "start", None)
            end = getattr(d, "end", None)
            shape_type = getattr(d, "shape_type", None)
            type_name = getattr(shape_type, "name", "") if shape_type is not None else ""
            type_name = str(type_name).lower()
            if "segment" in type_name and start is not None and end is not None:
                segments.append((_vec_to_xy_mm(start), _vec_to_xy_mm(end)))
            elif "rect" in type_name and start is not None and end is not None:
                x1, y1 = _vec_to_xy_mm(start)
                x2, y2 = _vec_to_xy_mm(end)
                segments.append(((x1, y1), (x2, y1)))
                segments.append(((x2, y1), (x2, y2)))
                segments.append(((x2, y2), (x1, y2)))
                segments.append(((x1, y2), (x1, y1)))
            elif "arc" in type_name and start is not None and end is not None:
                mid = getattr(d, "mid", None) or getattr(d, "midpoint", None)
                if mid is not None:
                    segments.extend(_arc_to_segments(
                        _vec_to_xy_mm(start),
                        _vec_to_xy_mm(mid),
                        _vec_to_xy_mm(end),
                    ))
        except Exception:
            continue

    # Fold any BoardShape segments through the chainer (matches the SWIG-
    # era path where each drawing was a separate segment item).
    if len(segments) >= 3:
        chained = _chain_segments_into_contours(segments)
        if chained:
            closed_contours.extend(chained)

    if not closed_contours:
        return [], []
    closed_contours.sort(
        key=lambda c: (max(p[0] for p in c) - min(p[0] for p in c)) *
                      (max(p[1] for p in c) - min(p[1] for p in c)),
        reverse=True)
    return closed_contours[0], closed_contours[1:]


def _extract_stackup_kipy(board) -> List[StackupLayer]:
    """Stackup extraction via kipy with file-parse fallback.

    BoardStackupLayer in kipy 0.7 exposes:
        layer (BoardLayer enum), user_name (str), thickness (nm int),
        type (BoardStackupLayer.Type enum), material_name (str),
        dielectric (BoardStackupDielectricLayer with epsilon_r,
                    loss_tangent, material_name, thickness)
    We map type → 'copper' / 'core' / 'prepreg' / 'dielectric' and pull
    epsilon_r / loss_tangent off the .dielectric sub-message when present.
    """
    stackup: List[StackupLayer] = []

    try:
        s = board.get_stackup()
        layers = getattr(s, "layers", None) or []
        for item in layers:
            try:
                name = getattr(item, "user_name", "") or ""
                # type is a proto enum; map by its .name (e.g. "BSLT_COPPER").
                type_enum = getattr(item, "type", None)
                if type_enum is None:
                    continue
                type_name = (getattr(type_enum, "name", None) or str(type_enum)).lower()
                if "copper" in type_name:
                    ltype = "copper"
                elif "core" in type_name:
                    ltype = "core"
                elif "prepreg" in type_name:
                    ltype = "prepreg"
                elif "dielec" in type_name:
                    ltype = "dielectric"
                else:
                    # Skip silkscreen / soldermask / paste / etc.
                    continue

                thickness_mm = _nm_to_mm(getattr(item, "thickness", 0) or 0)
                material = str(getattr(item, "material_name", "") or "")

                epsilon_r = 0.0
                loss_tangent = 0.0
                dielectric = getattr(item, "dielectric", None)
                if dielectric is not None:
                    try:
                        epsilon_r = float(getattr(dielectric, "epsilon_r", 0.0) or 0.0)
                        loss_tangent = float(getattr(dielectric, "loss_tangent", 0.0) or 0.0)
                        # Dielectric sub-message may carry its own thickness /
                        # material_name; prefer them if the top-level fields
                        # are empty.
                        if thickness_mm == 0:
                            thickness_mm = _nm_to_mm(getattr(dielectric, "thickness", 0) or 0)
                        if not material:
                            material = str(getattr(dielectric, "material_name", "") or "")
                    except Exception:
                        pass

                stackup.append(StackupLayer(
                    name=name, layer_type=ltype, thickness=thickness_mm,
                    epsilon_r=epsilon_r, loss_tangent=loss_tangent, material=material,
                ))
            except Exception:
                continue
    except Exception:
        pass

    if stackup:
        return stackup

    # Fallback: parse from the .kicad_pcb file using the absolute path.
    try:
        from kicad_ipc_adapter import get_board_full_path
        board_file = get_board_full_path()
    except Exception:
        board_file = None
    if board_file:
        try:
            with open(board_file, "r", encoding="utf-8") as f:
                content = f.read(8192)  # stackup lives near the top of the file
            stackup = extract_stackup(content)
        except Exception:
            pass

    return stackup


def _extract_zones_kipy(board, get_layer_name, resolve_net) -> List[Zone]:
    """Read filled zones via kipy.

    `resolve_net` is the name→synthetic-id resolver from
    build_pcb_data_from_board. Required — without it zones would
    mismatch the rest of PCBData's net ids and "is connected?" would
    silently break for zoned nets.
    """
    zones: List[Zone] = []
    try:
        zone_iter = board.get_zones()
    except AttributeError:
        return zones
    except Exception:
        return zones

    for z in zone_iter:
        try:
            net = getattr(z, "net", None)
            net_id, net_name = resolve_net(net)
            # kipy zones can span multiple layers; emit one Zone entry per
            # layer so callers that filter by single layer match cleanly.
            layers = getattr(z, "layers", None)
            if layers is None:
                layers = [getattr(z, "layer", None)]
            polygon = _extract_zone_polygon_mm(z)
            if not polygon:
                continue
            for bl in layers:
                zones.append(Zone(
                    net_id=net_id,
                    net_name=net_name,
                    layer=get_layer_name(bl),
                    polygon=polygon,
                ))
        except Exception:
            continue

    return zones


def _extract_zone_polygon_mm(zone) -> List[Tuple[float, float]]:
    """Pull a zone's outer outline into a list of (x_mm, y_mm) points.

    kipy zone.outline is a PolygonWithHoles whose .outline is a PolyLine.
    Older releases may expose direct iteration; probe both.
    """
    outline = getattr(zone, "outline", None)
    if outline is None:
        return []
    # PolygonWithHoles.outline → PolyLine
    pwh_outline = getattr(outline, "outline", None)
    if pwh_outline is not None:
        return _polyline_points_mm(pwh_outline)
    # Older / different shape: outline IS a polyline / iterable
    try:
        return _polyline_points_mm(outline)
    except Exception:
        return []


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


def compare_pcb_data(from_board: 'PCBData', from_file: 'PCBData', tolerance: float = 0.01) -> List[str]:
    """Compare two PCBData objects and return list of differences.

    Useful for validating that build_pcb_data_from_board() produces the same
    results as parse_kicad_pcb().

    Args:
        from_board: PCBData built from the live KiCad board over IPC
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


def _is_ball_grid(pads) -> bool:
    """Gate for geometry-based BGA classification: only a genuine ball/pin grid
    qualifies — enough pads, near-uniform pad size, AND a true 2-D matrix.
    Guards against keyswitches, SMD diode pairs, thermal-via exposed pads, and
    connector arrays being classified as BGAs and walled off behind exclusion
    zones (issue #82). BGA/PGA balls are identical; the size-mix false positives
    (switch pins + stabilizer holes, EP via grid + perimeter pads) fail the
    uniform-size test.

    The matrix test additionally rejects wide-pitch through-hole *headers* —
    e.g. the 2-row RPi_Pico_SMD_TH, which is uniform-size but only two columns
    wide (issue #82, set-2 reopen). A real array fills >=3 rows AND >=3 columns
    with several pads each; a 2-row header's columns are deep but it has only
    two of them, and a 1-/2-row header's rows hold just 1-2 pads. Pad-local
    coordinates are used so the test is independent of placement rotation.
    """
    if len(pads) < 16:
        return False
    size_counts = {}
    for p in pads:
        key = (round(p.size_x, 2), round(p.size_y, 2))
        size_counts[key] = size_counts.get(key, 0) + 1
    dominant_size, dominant_n = max(size_counts.items(), key=lambda kv: kv[1])
    if dominant_n < 0.9 * len(pads):
        return False

    # Among the uniform pads, count rows/columns that hold >=3 pads. A genuine
    # ball/pin array populates several of each; a pin header does not.
    uniform = [p for p in pads
               if (round(p.size_x, 2), round(p.size_y, 2)) == dominant_size]
    row_counts = {}
    col_counts = {}
    for p in uniform:
        ry = round(p.local_y, 2)
        rx = round(p.local_x, 2)
        row_counts[ry] = row_counts.get(ry, 0) + 1
        col_counts[rx] = col_counts.get(rx, 0) + 1
    populated_rows = sum(1 for n in row_counts.values() if n >= 3)
    populated_cols = sum(1 for n in col_counts.values() if n >= 3)
    return populated_rows >= 3 and populated_cols >= 3


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

        # BGA has many interior pads, QFN/QFP has mostly perimeter pads.
        # Either way the geometry path only declares BGA for a genuine
        # uniform ball/pin grid (issue #82).
        if interior_pads > perimeter_pads:
            return 'BGA' if _is_ball_grid(pads) else 'OTHER'
        elif perimeter_pads > 0:
            # Check pad shapes - QFN typically has rectangular pads, BGA has circular
            circular_pads = sum(1 for p in pads if p.shape in ('circle', 'oval'))
            rect_pads = sum(1 for p in pads if p.shape in ('rect', 'roundrect'))
            if rect_pads > circular_pads:
                return 'QFN'
            else:
                return 'BGA' if _is_ball_grid(pads) else 'OTHER'

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

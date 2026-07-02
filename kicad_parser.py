"""
KiCad PCB Parser - Extracts pads, nets, tracks, vias, and board info from .kicad_pcb files.
"""
from __future__ import annotations

import re
import math
import json
from dataclasses import dataclass, field
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


def board_uses_name_nets(content: str) -> bool:
    """True if the board references nets by NAME (KiCad 10 style) rather than by
    numeric id (KiCad 9). Detected from the ACTUAL content, not just the version
    header: a KiCad 10+ header, OR name-only refs ``(net "name")`` already present
    (a pre-2025 board a previous pass may have round-tripped). KiCad 9 numeric
    boards only have ``(net <id> "name")`` declarations and ``(net <id>)`` refs,
    neither of which matches ``(net "``.

    Writers use this to keep the output's net-token format consistent with the
    input - never emitting KiCad-10 name nets into a KiCad-9 numeric board (which
    KiCad 9 reads as net-less), nor numeric ids into a name-net board."""
    return is_kicad_10(content) or bool(re.search(r'\(net\s+"', content))


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
    pad_type: str = ""  # KiCad pad kind: 'smd', 'thru_hole', 'np_thru_hole',
    # 'connect'. NPTH pads have NO copper (their size is just a mask opening /
    # keep-out), so copper-vs-copper DRC must skip them; only the hole matters.
    roundrect_rratio: float = 0.0  # Corner radius ratio for roundrect pads
    rect_rotation: float = 0.0  # Residual rect tilt in global frame for non-
    # orthogonal pads (deg, in (-90,90]). 0 for axis-aligned pads (the common
    # case) where the rotation is already baked into size_x/size_y. The
    # obstacle/DRC geometry rotates the pad rectangle by this angle.
    local_clearance: float = 0.0  # Per-pad clearance override (mm) from the
    # pad's own (clearance ...) token. 0 means "no override, use the global
    # clearance". Larger-than-global values (e.g. fiducial keep-clear rings)
    # must widen the obstacle halo or copper routes within the pad's clearance.
    polygons: Optional[List[List[Tuple[float, float]]]] = None  # Real copper
    # outline(s) in GLOBAL board mm for custom comb/finger pads (issue #188).
    # When set, the obstacle map and DRC rasterize these polygons instead of the
    # size_x x size_y bounding box, so the finger channels (and the empty side of
    # an off-anchor pad) stay routable. None for ordinary rect/circle/roundrect
    # pads, which the bounding box models exactly.


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
    dnp: bool = False  # Do-not-populate / no-pop. A no-pop series part is an open
                       # circuit, so its pads do NOT bridge two nets into one signal.


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
    # Smallest copper clearance any routing step actually used on this board this
    # run -- e.g. a fine-pitch tap that escalated below the nominal --clearance.
    # None until a step records one. Routers fold this into the .kicad_pro DRC
    # floor and JSON so check_drc grades at the true routed clearance, not the
    # nominal one (which would flag legitimately tight copper).
    min_clearance_used: Optional[float] = None


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


def _custom_pad_local_extent(pad_text: str) -> Tuple[float, float]:
    """Half-extent (|x|, |y|) of a custom pad's real copper in the pad's local
    frame, from its (primitives ...) block. A custom pad's (size ...) is only the
    anchor; gr_poly/gr_circle/gr_line primitives can reach well beyond it (e.g. a
    resistor pad whose copper extends +0.56mm past the anchor). Modelling only the
    anchor under-blocks the obstacle map, so tracks graze the real copper (#70
    dig). Returns (0, 0) if there are no primitives. Coordinates are pad-local
    (un-rotated); _resolve_pad_rect then orients the enclosing rect like any pad.
    """
    pm = re.search(r'\(primitives\b', pad_text)
    if not pm:
        return 0.0, 0.0
    prim = pad_text[pm.start():]
    xs, ys = [], []
    for m in re.finditer(r'\(xy\s+(-?[\d.]+)\s+(-?[\d.]+)\)', prim):
        xs.append(float(m.group(1))); ys.append(float(m.group(2)))
    # gr_circle: (center x y) (end x y) -> radius reaches center +/- r on both axes
    for m in re.finditer(r'\(gr_circle\s+\(center\s+(-?[\d.]+)\s+(-?[\d.]+)\)\s+\(end\s+(-?[\d.]+)\s+(-?[\d.]+)\)', prim):
        cx, cy, ex, ey = map(float, m.groups())
        r = math.hypot(ex - cx, ey - cy)
        xs += [cx - r, cx + r]; ys += [cy - r, cy + r]
    if not xs:
        return 0.0, 0.0
    # primitive stroke width thickens the copper; add half of the largest.
    widths = [float(w) for w in re.findall(r'\(width\s+(-?[\d.]+)\)', prim)]
    hw = (max(widths) / 2.0) if widths else 0.0
    return max(abs(min(xs)), abs(max(xs))) + hw, max(abs(min(ys)), abs(max(ys))) + hw


def _custom_pad_global_polygons(pad_text: str, global_x: float, global_y: float,
                                pad_abs_rotation_deg: float):
    """Real copper outline(s) of a custom pad in GLOBAL board mm (issue #188).

    A custom pad's exposed source/drain copper is a comb/finger ``gr_poly``
    primitive whose notches our bounding-box model fills in, boxing in the pads
    that sit in the channels. This returns the actual polygon(s) so the obstacle
    map and DRC can rasterize the true copper instead.

    Each primitive vertex is in the pad's local frame (relative to the pad anchor,
    which is at the pad's global position). The pad's primitives are oriented by
    the pad's ABSOLUTE board angle (the file's ``(at x y angle)`` already folds in
    the footprint rotation), applied with KiCad's negate convention -- same as
    local_to_global. Verified against pcbnew GetEffectivePolygon for bitaxe Q1/Q2.

    Handles gr_poly, gr_circle, gr_rect and gr_line primitives -- each becomes one
    polygon (their union is the real copper; the obstacle map and DRC already treat
    polygons as a list and OR/min over them). A round mounting/logo pad drawn as a
    gr_circle thus models its true disc instead of the (size) bounding box, killing
    the phantom over-block on the empty side of the anchor (issue #232).

    Returns a list of vertex lists, or None when the pad has no primitives, only
    sub-3-vertex shapes, or any gr_arc / gr_curve primitive -- those curved shapes
    are still left to the conservative bounding-box model rather than approximated."""
    pm = re.search(r'\(primitives\b', pad_text)
    if not pm:
        return None
    prim = pad_text[pm.start():find_matching_paren(pad_text, pm.start()) - 1]
    # Arcs/beziers aren't approximated yet -- fall back to the bbox for the whole pad.
    if re.search(r'\(gr_(arc|curve)\b', prim):
        return None
    rad = math.radians(-pad_abs_rotation_deg)
    cos_r, sin_r = math.cos(rad), math.sin(rad)

    def to_global(local_pts):
        return [(global_x + (vx * cos_r - vy * sin_r),
                 global_y + (vx * sin_r + vy * cos_r)) for vx, vy in local_pts]

    def _field(block, name, n):
        m = re.search(r'\(' + name + r'\s+' + r'\s+'.join([r'(-?[\d.]+)'] * n) + r'\)', block)
        return tuple(map(float, m.groups())) if m else None

    def _width(block):
        m = re.search(r'\(width\s+(-?[\d.]+)\)', block)
        return float(m.group(1)) if m else 0.0

    polys = []
    # Iterate primitives in order; one polygon per primitive, transformed to global.
    for pm2 in re.finditer(r'\(gr_(poly|circle|rect|line)\b', prim):
        kind = pm2.group(1)
        block = prim[pm2.start():find_matching_paren(prim, pm2.start()) - 1]
        local = None
        if kind == 'poly':
            pts = [(float(a), float(b))
                   for a, b in re.findall(r'\(xy\s+(-?[\d.]+)\s+(-?[\d.]+)\)', block)]
            local = pts if len(pts) >= 3 else None
        elif kind == 'circle':
            c = _field(block, 'center', 2)
            e = _field(block, 'end', 2)
            if c and e:
                # Solid disc out to the outer copper edge (centerline radius + half
                # stroke); a filled circle has the same outer extent. The interior
                # is modelled solid (the union has no holes) -- conservative, and
                # exact for clearance to external copper, which is all that matters.
                R = math.hypot(e[0] - c[0], e[1] - c[1]) + _width(block) / 2.0
                if R > 0:
                    N = 32
                    local = [(c[0] + R * math.cos(2 * math.pi * k / N),
                              c[1] + R * math.sin(2 * math.pi * k / N)) for k in range(N)]
        elif kind == 'rect':
            s = _field(block, 'start', 2)
            e = _field(block, 'end', 2)
            if s and e:
                hw = _width(block) / 2.0
                x0, x1 = min(s[0], e[0]) - hw, max(s[0], e[0]) + hw
                y0, y1 = min(s[1], e[1]) - hw, max(s[1], e[1]) + hw
                local = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        elif kind == 'line':
            s = _field(block, 'start', 2)
            e = _field(block, 'end', 2)
            if s and e:
                hw = _width(block) / 2.0 or 1e-6
                dx, dy = e[0] - s[0], e[1] - s[1]
                L = math.hypot(dx, dy)
                if L > 0:
                    ux, uy = dx / L, dy / L          # along
                    px, py = -uy, ux                 # perpendicular
                    # Capsule approximated by its oriented bounding rectangle:
                    # extend each end by hw and offset +/- hw perpendicular.
                    sx, sy = s[0] - ux * hw, s[1] - uy * hw
                    ex, ey = e[0] + ux * hw, e[1] + uy * hw
                    local = [(sx + px * hw, sy + py * hw), (ex + px * hw, ey + py * hw),
                             (ex - px * hw, ey - py * hw), (sx - px * hw, sy - py * hw)]
        if local:
            polys.append(to_global(local))
    return polys or None


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


def _poly_points_from_drawing(drawing, to_mm) -> List[Tuple[float, float]]:
    """Extract a poly PCB_SHAPE's outline vertices as (x, y) mm tuples."""
    pts = []
    outline = drawing.GetPolyShape().Outline(0)
    for i in range(outline.PointCount()):
        pt = outline.CPoint(i)
        pts.append((to_mm(pt.x), to_mm(pt.y)))
    return pts


def extract_guide_paths_from_board(board, layer_name: str = "User.1") -> List["GuidePath"]:
    """Read user-layer guide polylines from a live pcbnew board (best-effort)."""
    import pcbnew

    try:
        layer_id = board.GetLayerID(layer_name)
    except Exception:
        return []
    if layer_id is None or layer_id < 0:
        return []

    seg_shape = getattr(pcbnew, 'S_SEGMENT', getattr(pcbnew, 'SHAPE_T_SEGMENT', None))
    poly_shape = getattr(pcbnew, 'S_POLYGON', getattr(pcbnew, 'SHAPE_T_POLY', None))
    to_mm = pcbnew.ToMM
    segments = []
    paths: List[GuidePath] = []
    for drawing in board.GetDrawings():
        try:
            if drawing.GetLayer() != layer_id:
                continue
            if drawing.GetClass() not in ("PCB_SHAPE", "DRAWSEGMENT"):
                continue
            shape_type = drawing.GetShape()
        except Exception:
            continue
        if seg_shape is not None and shape_type == seg_shape:
            try:
                s, e = drawing.GetStart(), drawing.GetEnd()
                segments.append(((to_mm(s.x), to_mm(s.y)), (to_mm(e.x), to_mm(e.y))))
            except Exception:
                continue
        elif poly_shape is not None and shape_type == poly_shape:
            try:
                pts = _poly_points_from_drawing(drawing, to_mm)
                if len(pts) >= 2:
                    paths.append(GuidePath(layer=layer_name, points=pts, is_closed=True))
            except Exception:
                continue

    paths.extend(_chain_guide_segments(segments, layer_name))
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


def extract_keepout_zones_from_board(board, layer_name: str = "User.2") -> List["GuidePath"]:
    """Read user-layer closed keepout polygons from a live pcbnew board (best-effort)."""
    import pcbnew

    try:
        layer_id = board.GetLayerID(layer_name)
    except Exception:
        return []
    if layer_id is None or layer_id < 0:
        return []

    poly_shape = getattr(pcbnew, 'S_POLYGON', getattr(pcbnew, 'SHAPE_T_POLY', None))
    rect_shape = getattr(pcbnew, 'S_RECT', getattr(pcbnew, 'SHAPE_T_RECT', None))
    to_mm = pcbnew.ToMM
    zones: List[GuidePath] = []
    for drawing in board.GetDrawings():
        try:
            if drawing.GetLayer() != layer_id:
                continue
            if drawing.GetClass() not in ("PCB_SHAPE", "DRAWSEGMENT"):
                continue
            shape_type = drawing.GetShape()
        except Exception:
            continue
        if poly_shape is not None and shape_type == poly_shape:
            try:
                pts = _poly_points_from_drawing(drawing, to_mm)
                if len(pts) >= 3:
                    zones.append(GuidePath(layer=layer_name, points=pts, is_closed=True))
            except Exception:
                continue
        elif rect_shape is not None and shape_type == rect_shape:
            try:
                s, e = drawing.GetStart(), drawing.GetEnd()
                x1, y1, x2, y2 = to_mm(s.x), to_mm(s.y), to_mm(e.x), to_mm(e.y)
                zones.append(GuidePath(
                    layer=layer_name,
                    points=[(x1, y1), (x2, y1), (x2, y2), (x1, y2)],
                    is_closed=True))
            except Exception:
                continue

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

        # Extract the do-not-populate flag from the footprint (attr ...) token.
        # KiCad 7+ writes a standalone `dnp` keyword among the attr flags
        # (e.g. "(attr smd dnp exclude_from_pos_files)"). A no-pop part is an
        # open circuit, so callers must not treat its pads as bridging two nets.
        attr_match = re.search(r'\(attr\b([^)]*)\)', fp_text)
        is_dnp = bool(attr_match and re.search(r'\bdnp\b', attr_match.group(1)))

        footprint = Footprint(
            reference=reference,
            footprint_name=fp_name,
            x=fp_x,
            y=fp_y,
            rotation=fp_rotation,
            layer=fp_layer,
            value=value,
            dnp=is_dnp
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

            # Custom pads: enclose the real primitive copper, not just the anchor
            # (size ...). Use a centred rect around the connection point that
            # covers the full local extent -- conservative (may over-block the
            # empty side of an asymmetric pad) but DRC-safe; modelling only the
            # anchor lets tracks graze copper that reaches past it.
            if pad_shape == 'custom':
                ext_x, ext_y = _custom_pad_local_extent(pad_text)
                if ext_x > 0:
                    size_x = max(size_x, 2.0 * ext_x)
                if ext_y > 0:
                    size_y = max(size_y, 2.0 * ext_y)

            # Resolve the pad rectangle into board space. size_x/size_y are in
            # the pad's own frame; its absolute board orientation is
            # total_rotation (pad + footprint), NOT pad_rotation alone — keying
            # the swap on pad_rotation misses pads whose footprint supplies the
            # 90° (e.g. a -135° footprint with a 225° pad lands at total 90° and
            # was left un-swapped, modelled 90° off its real shape). For the
            # common orthogonal cases bake the rotation into the dimensions so
            # all downstream axis-aligned geometry stays exact; genuine diagonal
            # pads keep a residual rect_rotation the obstacle/DRC geometry applies.
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

            # Extract per-pad local clearance override, e.g. fiducial keep-clear
            # rings carry (clearance 0.375). The leading "(" avoids matching the
            # footprint's (pad_to_mask_clearance ...) token. 0 = no override.
            clr_match = re.search(r'\(clearance\s+([\d.]+)\)', pad_text)
            local_clearance = float(clr_match.group(1)) if clr_match else 0.0

            # Calculate global coordinates
            global_x, global_y = local_to_global(fp_x, fp_y, fp_rotation, local_x, local_y)

            # Custom comb/finger pads: carry the real copper polygon(s) so the
            # obstacle map / DRC model the notches instead of the bounding box
            # (issue #188). pad_rotation here is the file's pad (at) angle, which
            # is the pad's absolute board orientation (folds in fp rotation).
            pad_polygons = None
            if pad_shape == 'custom':
                pad_polygons = _custom_pad_global_polygons(
                    pad_text, global_x, global_y, pad_rotation)

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
                pad_type=pad_type,
                drill=drill_size,
                roundrect_rratio=roundrect_rratio,
                rect_rotation=rect_rotation,
                local_clearance=local_clearance,
                polygons=pad_polygons
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
        # Check for free flag in matched vias. The free_pattern below has two
        # DOTALL `.*?` gaps, so on a board with NO free via it backtracks from
        # every `(via` to EOF looking for a `(free yes)` that isn't there --
        # O(vias x filesize), ~32s on daisho's 1574 vias (issue #225). Guard on a
        # single linear scan for the token; absent it, no via is free and
        # free_uuids is empty (identical result).
        if vias and re.search(r'\(free\s+yes\)', content):
            free_pattern = r'\(via\s+\(at\s+[\d.-]+\s+[\d.-]+\).*?\(free\s+yes\).*?\(uuid\s+"([^"]+)"\)'
            free_uuids = {m.group(1) for m in re.finditer(free_pattern, content, re.DOTALL)}
            for via in vias:
                if via.uuid in free_uuids:
                    via.free = True

    return vias


def extract_segments(content: str, name_to_id: Dict[str, int] = None) -> List[Segment]:
    """Extract all track segments from PCB file."""
    segments = []

    # Try KiCad 9 format first: (net <id>). (locked yes) is optional and KiCad
    # emits it between width/layer or layer/net, so allow it at both spots -
    # otherwise a locked track parses to nothing, never becomes an obstacle, and
    # the router lays copper straight through it (issue #150).
    segment_pattern = r'\(segment\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)\s+\(width\s+([\d.-]+)\)\s+(?:\(locked\s+yes\)\s+)?\(layer\s+"([^"]+)"\)\s+(?:\(locked\s+yes\)\s+)?\(net\s+(\d+)\)\s+\(uuid\s+"([^"]+)"\)'

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
        segment_pattern_v10 = r'\(segment\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)\s+\(width\s+([\d.-]+)\)\s+(?:\(locked\s+yes\)\s+)?\(layer\s+"([^"]+)"\)\s+(?:\(locked\s+yes\)\s+)?\(net\s+"([^"]*)"\)\s+\(uuid\s+"([^"]+)"\)'
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


def build_pcb_data_from_board(board, guide_layer: str = "User.1",
                              keepout_layer: str = "User.2") -> PCBData:
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

    # Board bounds from Edge.Cuts drawings. We use each drawing's bounding
    # box rather than GetStart()/GetEnd() because the latter only describes
    # the shape's anchor points for non-line shapes (polygons, circles, arcs
    # can have start==end==(0,0) with the geometry stored separately), which
    # produced a degenerate (0,0,0,0) board_bounds.
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
                if class_name not in ("PCB_SHAPE", "DRAWSEGMENT"):
                    continue
                try:
                    bbox = drawing.GetBoundingBox()
                    w, h = bbox.GetWidth(), bbox.GetHeight()
                    if w <= 0 and h <= 0:
                        continue
                    x0, y0 = to_mm(bbox.GetX()), to_mm(bbox.GetY())
                    x1, y1 = to_mm(bbox.GetX() + w), to_mm(bbox.GetY() + h)
                    bmin_x = min(bmin_x, x0, x1)
                    bmin_y = min(bmin_y, y0, y1)
                    bmax_x = max(bmax_x, x0, x1)
                    bmax_y = max(bmax_y, y0, y1)
                    found_edge = True
                except Exception:
                    continue
            if found_edge and bmax_x > bmin_x and bmax_y > bmin_y:
                board_bounds = (bmin_x, bmin_y, bmax_x, bmax_y)
    except Exception:
        pass
    # Fallback to the whole-board edges bounding box (also handles boards
    # whose Edge.Cuts items are stored inside footprints rather than as
    # top-level drawings).
    if board_bounds is None:
        try:
            bbox = board.GetBoardEdgesBoundingBox()
            if bbox.GetWidth() > 0 and bbox.GetHeight() > 0:
                board_bounds = (to_mm(bbox.GetX()), to_mm(bbox.GetY()),
                                to_mm(bbox.GetX() + bbox.GetWidth()),
                                to_mm(bbox.GetY() + bbox.GetHeight()))
        except Exception:
            pass

    # Board outline and cutouts from Edge.Cuts drawings
    board_outline, board_cutouts = _extract_board_contours_from_pcbnew(board, to_mm)

    # Stackup
    stackup = _extract_stackup_from_pcbnew(board, to_mm)

    board_info = BoardInfo(
        layers=layers_dict,
        copper_layers=copper_layers,
        board_bounds=board_bounds,
        stackup=stackup,
        board_outline=board_outline,
        board_cutouts=board_cutouts
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

        # DNP (do-not-populate) flag — kept at parity with the text parser. A
        # no-pop series part is an open circuit, so its pads must not be treated
        # as bridging two nets into one logical net. IsDNP() is KiCad 7+.
        try:
            fp_dnp = bool(fp.IsDNP())
        except Exception:
            fp_dnp = False

        footprint = Footprint(
            reference=reference,
            footprint_name=fp_name,
            x=fp_x,
            y=fp_y,
            rotation=fp_rotation,
            layer=fp_layer,
            value=fp_value,
            dnp=fp_dnp
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

            # Custom pads: GetSize() is only the anchor; enclose the real copper
            # via the effective bounding box, centred on the connection point so
            # all copper is covered (mirrors the text-parser path / #70 dig). The
            # bbox is already board-axis-aligned, so use it directly: do NOT max
            # against the anchor GetSize(), which is in the pad's UN-rotated local
            # frame - for a rotated pad the anchor's long side leaks into the wrong
            # board axis and over-widens the pad (e.g. U9 QFN: narrow 0.28 -> 0.71).
            pad_polygons = None
            if shape == 'custom':
                try:
                    bb = pad.GetBoundingBox()
                    pos = pad.GetPosition()
                    hx = max(abs(to_mm(bb.GetLeft() - pos.x)), abs(to_mm(bb.GetRight() - pos.x)))
                    hy = max(abs(to_mm(bb.GetTop() - pos.y)), abs(to_mm(bb.GetBottom() - pos.y)))
                    size_x, size_y, rect_rotation = 2 * hx, 2 * hy, 0.0
                except Exception:
                    size_x, size_y, rect_rotation = _resolve_pad_rect(size_x, size_y, pad_rotation)
                # Carry the real copper outline so the obstacle map / DRC model the
                # finger channels, not the bounding box (issue #188). pcbnew's
                # effective polygon is already in GLOBAL board coordinates.
                # GetEffectivePolygon needs KiCad's geometry cache (a running
                # wxApp, which the plugin has); if it is unavailable it returns
                # empty and we fall back to the bounding box (today's behaviour --
                # no regression). pcbnew gives the outline in GLOBAL board coords.
                try:
                    poly_set = pad.GetEffectivePolygon(pad.GetLayer())
                    polys = []
                    for oi in range(poly_set.OutlineCount()):
                        ol = poly_set.Outline(oi)
                        pts = [(to_mm(ol.CPoint(i).x), to_mm(ol.CPoint(i).y))
                               for i in range(ol.PointCount())]
                        if len(pts) >= 3:
                            polys.append(pts)
                    pad_polygons = polys or None
                except Exception:
                    pad_polygons = None
            else:
                # Resolve the pad rectangle into board space (keyed on the absolute
                # total rotation, not pad_rotation alone — see the text-parser path).
                size_x, size_y, rect_rotation = _resolve_pad_rect(size_x, size_y, pad_rotation)

            # Pin metadata
            try:
                pinfunction = pad.GetPinFunction()
            except Exception:
                pinfunction = ""

            try:
                pintype = pad.GetPinType()
            except Exception:
                pintype = ""

            # Pad kind at text-parser parity ('smd'/'thru_hole'/'np_thru_hole'/
            # 'connect') so DRC can skip the copper-less NPTH pads.
            try:
                pad_type = {
                    pcbnew.PAD_ATTRIB_SMD: 'smd',
                    pcbnew.PAD_ATTRIB_PTH: 'thru_hole',
                    pcbnew.PAD_ATTRIB_NPTH: 'np_thru_hole',
                    pcbnew.PAD_ATTRIB_CONN: 'connect',
                }.get(pad.GetAttribute(), "")
            except Exception:
                pad_type = ""

            # Drill. Oval/slot drills have distinct x/y; take max(x, y) to match
            # the text parser (issue #106) - using only .x under-reports a rotated
            # slot's hole and risks hole-to-hole DRC.
            try:
                drill_size = pad.GetDrillSize()
                drill = to_mm(max(drill_size.x, drill_size.y))
            except Exception:
                drill = 0.0

            # Roundrect ratio
            try:
                roundrect_rratio = pad.GetRoundRectRadiusRatio()
            except Exception:
                roundrect_rratio = 0.0

            # Per-pad local clearance override (fiducial keep-clear rings etc.).
            # GetLocalClearance() returns the pad's own override in IU, or a
            # falsy/None when unset depending on KiCad version. 0 = no override.
            try:
                lc = pad.GetLocalClearance()
                local_clearance = to_mm(lc) if lc else 0.0
            except Exception:
                local_clearance = 0.0

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
                pad_type=pad_type,
                drill=drill,
                roundrect_rratio=roundrect_rratio,
                rect_rotation=rect_rotation,
                local_clearance=local_clearance,
                polygons=pad_polygons
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

    # --- Extract user-layer guide corridors (issue #7) ---
    guide_paths = extract_guide_paths_from_board(board, guide_layer)

    # --- Extract user-layer keepout polygons (issue #27) ---
    keepout_zones = extract_keepout_zones_from_board(board, keepout_layer)

    # --- Extract KiCad rule-area keep-outs (issue #95) ---
    # board_info.keepouts (the (keepout ...) rule areas the router must avoid)
    # isn't cleanly exposed via the pcbnew zone API, so read them from the board
    # file content exactly as the text parser does (parse_kicad_pcb), keeping the
    # GUI/SWIG path at parity with the CLI. Best-effort: the saved file is the
    # last-saved state, which is fine for rule areas (design-time features, not
    # edited mid-route). Without this the router lays copper through rule areas.
    try:
        board_filename = board.GetFileName()
        if board_filename:
            with open(board_filename, 'r', encoding='utf-8') as f:
                board_info.keepouts = extract_keepouts(f.read())
    except Exception:
        pass

    return PCBData(
        board_info=board_info,
        nets=nets,
        footprints=footprints,
        vias=vias,
        segments=segments,
        pads_by_net=pads_by_net,
        zones=zones,
        guide_paths=guide_paths,
        keepout_zones=keepout_zones
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


def _extract_board_contours_from_pcbnew(board, to_mm):
    """Extract board outline and cutout polygons from Edge.Cuts drawings via pcbnew.

    Returns (outline, cutouts) where outline is the outer boundary polygon
    and cutouts is a list of interior cutout polygons.
    """
    import pcbnew

    edge_cuts_id = getattr(pcbnew, 'Edge_Cuts', None)
    if edge_cuts_id is None:
        return [], []

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
                elif shape_type == getattr(pcbnew, 'SHAPE_T_ARC', getattr(pcbnew, 'S_ARC', -1)):
                    start = drawing.GetStart()
                    mid = drawing.GetArcMid()
                    end = drawing.GetEnd()
                    segments.extend(_arc_to_segments(
                        (to_mm(start.x), to_mm(start.y)),
                        (to_mm(mid.x), to_mm(mid.y)),
                        (to_mm(end.x), to_mm(end.y))
                    ))
            except Exception:
                continue

    if len(segments) < 3:
        return [], []

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
                return [], []

    contours = _chain_segments_into_contours(segments)
    if not contours:
        return [], []

    def contour_bbox_area(contour):
        xs = [p[0] for p in contour]
        ys = [p[1] for p in contour]
        return (max(xs) - min(xs)) * (max(ys) - min(ys))

    contours.sort(key=contour_bbox_area, reverse=True)
    return contours[0], contours[1:]


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

    # --- Compare nets (net 0 = unconnected pseudo-net; one path may list it) ---
    board_net_ids = set(from_board.nets.keys()) - {0}
    file_net_ids = set(from_file.nets.keys()) - {0}
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
            # Pair pads by POSITION, not pad_number: pad numbers are not unique
            # (empty "" thermal/mechanical pads, repeated connector pads), so
            # sorting+zipping on pad_number misaligns a roundrect pad against a
            # circle pad and cascades into spurious diffs. Position is unique per
            # pad and identical (to ~1um) between the board and file parses.
            def _pad_key(p):
                # Include layers: a footprint can stack pads at one position+number
                # (separate copper / paste / mask apertures), so position+number
                # alone is not unique and would pair non-deterministically.
                return (round(p.global_x, 3), round(p.global_y, 3), p.pad_number,
                        tuple(sorted(p.layers)))
            b_pads = sorted(bf.pads, key=_pad_key)
            f_pads = sorted(ff.pads, key=_pad_key)
            for bp, fp in zip(b_pads, f_pads):
                if not close(bp.global_x, fp.global_x) or not close(bp.global_y, fp.global_y):
                    diffs.append(f"Footprint {ref} pad pairing mismatch (position): "
                                 f"board {bp.pad_number}@({bp.global_x:.3f},{bp.global_y:.3f}) "
                                 f"vs file {fp.pad_number}@({fp.global_x:.3f},{fp.global_y:.3f})")
                    continue
                if bp.pad_number != fp.pad_number:
                    diffs.append(f"Pad @({bp.global_x:.3f},{bp.global_y:.3f}) number: board={bp.pad_number} file={fp.pad_number}")
                if bp.net_id != fp.net_id:
                    diffs.append(f"Pad {ref}:{bp.pad_number} net_id: board={bp.net_id} file={fp.net_id}")
                if bp.shape != fp.shape:
                    diffs.append(f"Pad {ref}:{bp.pad_number} shape: board={bp.shape} file={fp.shape}")
                if not close(bp.size_x, fp.size_x) or not close(bp.size_y, fp.size_y):
                    diffs.append(f"Pad {ref}:{bp.pad_number} size: board=({bp.size_x:.3f},{bp.size_y:.3f}) file=({fp.size_x:.3f},{fp.size_y:.3f})")
                # Residual rect tilt - the obstacle/DRC geometry rotates the pad
                # rectangle by this, so a board/file mismatch (e.g. a custom or
                # diagonal pad modelled differently by the two paths) changes the
                # modelled copper footprint.
                br = getattr(bp, 'rect_rotation', 0.0); fr = getattr(fp, 'rect_rotation', 0.0)
                if not close(br, fr):
                    diffs.append(f"Pad {ref}:{bp.pad_number} rect_rotation: board={br:.2f} file={fr:.2f}")
                # Per-pad local clearance override (fiducial keep-clear rings etc.).
                # A pcbnew accessor that silently returns 0 would drop a keep-out the
                # file parse honors - this surfaces that divergence.
                bc = getattr(bp, 'local_clearance', 0.0); fc = getattr(fp, 'local_clearance', 0.0)
                if not close(bc, fc):
                    diffs.append(f"Pad {ref}:{bp.pad_number} local_clearance: board={bc:.3f} file={fc:.3f}")
                # Roundrect corner ratio - only consumed (and only meaningful) for
                # roundrect pads; pcbnew returns a default ratio on every pad shape,
                # so comparing it on circle/rect pads is pure noise.
                if bp.shape == 'roundrect' and fp.shape == 'roundrect':
                    brr = getattr(bp, 'roundrect_rratio', 0.0); frr = getattr(fp, 'roundrect_rratio', 0.0)
                    if not close(brr, frr):
                        diffs.append(f"Pad {ref}:{bp.pad_number} roundrect_rratio: board={brr:.3f} file={frr:.3f}")
                if not close(bp.drill, fp.drill):
                    diffs.append(f"Pad {ref}:{bp.pad_number} drill: board={bp.drill:.3f} file={fp.drill:.3f}")
                # Compare layers (as sets since order may differ)
                if set(bp.layers) != set(fp.layers):
                    diffs.append(f"Pad {ref}:{bp.pad_number} layers: board={bp.layers} file={fp.layers}")

    # --- Compare segments (geometry, not just count) ---
    # Existing tracks are routing obstacles, so a position/width/layer/net
    # divergence changes what the router sees. Match as a multiset of canonical
    # signatures (endpoint order normalised, coords quantised to ~1um) since the
    # board and file orderings differ.
    def _q(v):
        return round(v, 3)

    def _seg_sig(s):
        ends = tuple(sorted([(_q(s.start_x), _q(s.start_y)), (_q(s.end_x), _q(s.end_y))]))
        return (ends, _q(s.width), s.layer, s.net_id)

    def _multiset_diff(board_items, file_items, sig, label, fmt):
        from collections import Counter
        cb = Counter(sig(x) for x in board_items)
        cf = Counter(sig(x) for x in file_items)
        only_board = list((cb - cf).elements())
        only_file = list((cf - cb).elements())
        if only_board or only_file:
            diffs.append(f"{label} count: board={len(board_items)} file={len(file_items)}; "
                         f"{len(only_board)} only in board, {len(only_file)} only in file")
            for s in only_board[:5]:
                diffs.append(f"  {label} only in board: {fmt(s)}")
            for s in only_file[:5]:
                diffs.append(f"  {label} only in file: {fmt(s)}")

    _multiset_diff(from_board.segments, from_file.segments, _seg_sig, "Segment",
                   lambda s: f"ends={s[0]} w={s[1]} layer={s[2]} net={s[3]}")

    # --- Compare vias (geometry, not just count) ---
    def _via_sig(v):
        return (_q(v.x), _q(v.y), _q(v.size), _q(v.drill), v.net_id, tuple(sorted(v.layers)))

    _multiset_diff(from_board.vias, from_file.vias, _via_sig, "Via",
                   lambda v: f"({v[0]},{v[1]}) size={v[2]} drill={v[3]} net={v[4]} layers={list(v[5])}")

    # --- Compare zones (net/layer/vertex count) ---
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

    # --- Compare board outline / cutouts (used for edge & cutout obstacles) ---
    bo_b = bi_b.board_outline or []
    bo_f = bi_f.board_outline or []
    if len(bo_b) != len(bo_f):
        diffs.append(f"Board outline vertex count: board={len(bo_b)} file={len(bo_f)}")
    cut_b = bi_b.board_cutouts or []
    cut_f = bi_f.board_cutouts or []
    if len(cut_b) != len(cut_f):
        diffs.append(f"Board cutout count: board={len(cut_b)} file={len(cut_f)}")

    # --- Compare keepout zones (routing obstacles) ---
    kz_b = from_board.keepout_zones or []
    kz_f = from_file.keepout_zones or []
    if len(kz_b) != len(kz_f):
        diffs.append(f"Keepout zone count: board={len(kz_b)} file={len(kz_f)}")

    # --- Compare guide paths (used by guided routing) ---
    gp_b = from_board.guide_paths or []
    gp_f = from_file.guide_paths or []
    if len(gp_b) != len(gp_f):
        diffs.append(f"Guide path count: board={len(gp_b)} file={len(gp_f)}")

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

    # Check footprint name first. Grid/land-grid/chip-scale arrays all get BGA
    # treatment (bga_fanout escape + BGA exclusion zone): LGA (land grid), CSP/
    # WLCSP/WLP (wafer-level chip-scale = micro-BGA), CGA (column grid). Without
    # this an LGA-12 etc. is too small for the geometric grid gate below and falls
    # through to OTHER, so its interior lands get routed over (issue #144).
    if any(k in fp_name for k in ('BGA', 'FBGA', 'LFBGA', 'LGA', 'CSP', 'WLCSP', 'WLP', 'CGA')):
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


def auto_detect_bga_exclusion_zones(pcb_data: 'PCBData', margin: float = 0.0) -> List[Tuple[float, float, float, float, float]]:
    """
    Auto-detect BGA exclusion zones from all BGA components in the PCB.

    Via placement should be avoided inside BGA packages to prevent shorts
    with the BGA balls.

    The zone is anchored to the BGA's PAD bounding box (get_footprint_bounds is
    pad-edge based), not the courtyard, and defaults to margin=0 so the zone ends
    exactly at the edge of the outermost pads. The fanout escapes every pad to a
    stub that exits PAST the pad edge (>= (pitch - pad_size)/2, and ideally
    BGA_EXIT_MARGIN), so a pad-edge zone leaves those stub tips in open copper --
    a coupled/single-ended leg can then attach to a tip outside the keep-out
    instead of being marooned inside it. The old margin=0.5 pushed the zone ~0.3mm
    PAST where the escapes exit, burying the tips (issue #243).

    Returns zones as 5-tuples: (min_x, min_y, max_x, max_y, edge_tolerance)
    where edge_tolerance = margin + pitch * 1.1 (pitch + 10%)

    Args:
        pcb_data: Parsed PCB data
        margin: Extra margin around the pad bounding box (in mm); 0 = end at pad edge

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

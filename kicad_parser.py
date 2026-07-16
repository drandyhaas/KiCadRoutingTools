"""
KiCad PCB Parser - Extracts pads, nets, tracks, vias, and board info from .kicad_pcb files.
"""
from __future__ import annotations

import re
import math
import json
import routing_defaults as defaults  # fab-floor outline width for 0-stroke copper polys (#337/M2)
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
    rotation: float = 0.0  # Pad's ABSOLUTE board angle in degrees. The file's
    # (at ... angle) / pcbnew GetOrientation already include the footprint's
    # rotation -- do NOT add it again (#369 A4; slot-drill capsules orient by this)
    pinfunction: str = ""
    drill: float = 0.0  # Drill size for through-hole pads (0 for SMD). For an
                         # oval/slot drill this is max(w, h) (back-compat: the
                         # slot's long axis); use pad_drill_capsule()/
                         # pad_drill_circles() for the real slot geometry.
    drill_w: float = 0.0  # Oval/slot drill x-size in the PAD frame (0 = round)
    # Hole position (#324/#325): pads with a copper offset ((drill (offset ..)))
    # have global_x/global_y at the COPPER CENTER (what every clearance/DRC/
    # obstacle consumer wants) while the drill stays at the anchor. None =
    # hole coincides with global_x/global_y (the overwhelmingly common case).
    hole_x: object = None
    hole_y: object = None
    drill_h: float = 0.0  # Oval/slot drill y-size in the PAD frame (0 = round)
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
    # True for copper derived from a gr_line/gr_arc GRAPHIC on a copper layer
    # (#337): real copper for DRC/obstacles, but NOT a track -- cleanup passes
    # must never prune it and writers cannot strip it (there is no (segment)
    # block to match).
    graphic: bool = False


@dataclass
class Zone:
    """Represents a filled zone (power plane)."""
    net_id: int
    net_name: str
    layer: str
    polygon: List[Tuple[float, float]]  # List of (x, y) vertices defining the zone outline
    uuid: str = ""
    priority: int = 0  # (priority N); higher-priority zones win where outlines overlap
    island_removal_mode: int = 0  # 0 = always remove isolated islands (KiCad default,
    # token absent from the file), 1 = never remove, 2 = remove below island_area_min
    island_area_min: float = 0.0  # mm^2 floor for mode 2 (KiCad file units are mm^2)


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
    locked: bool = False  # Footprint (locked yes) flag: the user pinned this part,
                          # so placement passes must not move it and routing/fanout
                          # cannot assume a later step will move it out of the way.
    clearance: float = 0.0  # Footprint-level (clearance ...) override (mm),
    # inherited by every pad of this footprint that has no override of its own
    # (KiCad resolution: pad override, else footprint override, else netclass).
    # The parser RESOLVES the inheritance into each pad's local_clearance at
    # parse time, so clearance consumers only ever read pad.local_clearance;
    # this field records the raw footprint value for fidelity (issue #326).
    net_tie_groups: List[List[str]] = field(default_factory=list)
    # KiCad (net_tie_pad_groups "1,2" ...): pad-number groups this footprint
    # DELIBERATELY shorts (Kelvin shunts, net-tie parts). KiCad exempts copper
    # of the grouped pads' nets from mutual clearance at the footprint, so a
    # sense pad enclosed by its partner's ring is still routable (cynthion
    # R42: the AUX_SENSE- tab sits inside AUX_VBUS_IN's pad -- treating the
    # partner as a hard obstacle seals the net forever). Obstacle builders and
    # check_drc consume this via PCBData.net_tie_exempt_pad_ids().


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
    # ALL outer boundary rings (issue #304). A board can carry several disjoint
    # outlines in one file (split keyboards drawn as left+right halves,
    # panelized boards); board_outline keeps the LARGEST for back-compat, this
    # holds every outer ring. Empty on single-outline/rectangular boards means
    # "use board_outline".
    board_outlines: List[List[Tuple[float, float]]] = field(default_factory=list)
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

    def net_tie_exempt_pad_ids(self, net_id: int):
        """id()s of pads whose keep-out copper of `net_id` may IGNORE.

        For every footprint net-tie group (net_tie_pad_groups) that contains a
        pad on `net_id`, the OTHER pads of that group are deliberate shorts at
        the footprint -- KiCad exempts their mutual clearance, and the partner
        pad often physically encloses the tied pad (Kelvin shunts: cynthion
        R42's AUX_SENSE- tab sits inside AUX_VBUS_IN's pad). Obstacle builders
        skip stamping these pads when routing `net_id`; check_drc skips the
        corresponding pairs. Cached per net; identity keying is safe because
        the pad objects live in this PCBData for its whole lifetime.
        """
        cache = getattr(self, '_net_tie_exempt_cache', None)
        if cache is None:
            cache = {}
            self._net_tie_exempt_cache = cache
        got = cache.get(net_id)
        if got is not None:
            return got
        exempt = set()
        for fp in self.footprints.values():
            if not fp.net_tie_groups:
                continue
            by_num = {}
            for p in fp.pads:
                by_num.setdefault(p.pad_number, []).append(p)
            for group in fp.net_tie_groups:
                members = [p for num in group for p in by_num.get(num, [])]
                if any(p.net_id == net_id for p in members):
                    exempt.update(id(p) for p in members if p.net_id != net_id)
        cache[net_id] = exempt
        return exempt

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


# 3-point arc entry, two spellings with identical fields: (arc ...) inside a
# KiCad 7+ polygon point list, and a standalone (gr_arc ...) pad primitive.
#   (pts (arc (start x y) (mid x y) (end x y)) (xy x y) ...)
_PTS_ARC_RE = (r'\((?:gr_)?arc\s+\(start\s+(-?[\d.]+)\s+(-?[\d.]+)\)\s+'
               r'\(mid\s+(-?[\d.]+)\s+(-?[\d.]+)\)\s+'
               r'\(end\s+(-?[\d.]+)\s+(-?[\d.]+)\)')


def _custom_pad_primitive_points(pad_text: str):
    """Per-primitive (points, half_stroke) tuples from a custom pad's
    (primitives ...) block, in the pad's local frame. Arcs (gr_poly pts arcs
    AND standalone gr_arc) are linearized; gr_circle yields its axis extremes.
    Used by the board-frame extent below."""
    pm = re.search(r'\(primitives\b', pad_text)
    if not pm:
        return []
    prim = pad_text[pm.start():find_matching_paren(pad_text, pm.start()) - 1]
    out = []
    for m2 in re.finditer(r'\(gr_(poly|circle|rect|line|arc)\b', prim):
        block = prim[m2.start():find_matching_paren(prim, m2.start()) - 1]
        wm = re.search(r'\(width\s+(-?[\d.]+)\)', block)
        hw = (float(wm.group(1)) / 2.0) if wm else 0.0
        pts = []
        for m in re.finditer(r'\(xy\s+(-?[\d.]+)\s+(-?[\d.]+)\)', block):
            pts.append((float(m.group(1)), float(m.group(2))))
        for m in re.finditer(_PTS_ARC_RE, block):
            s = (float(m.group(1)), float(m.group(2)))
            mid = (float(m.group(3)), float(m.group(4)))
            e = (float(m.group(5)), float(m.group(6)))
            for p1, p2 in _arc_to_segments(s, mid, e):
                pts.append(p1); pts.append(p2)
        cm = re.search(r'\(center\s+(-?[\d.]+)\s+(-?[\d.]+)\)\s+\(end\s+(-?[\d.]+)\s+(-?[\d.]+)\)', block)
        if cm and m2.group(1) == 'circle':
            cx, cy, ex, ey = map(float, cm.groups())
            r = math.hypot(ex - cx, ey - cy)
            # A circle is rotation-invariant about its centre: represent it as
            # the centre point with the OUTER radius (centreline radius + half
            # the stroke -- KiCad strokes the outline of both filled and
            # unfilled circles) folded into the half-stroke term. The extent
            # consumer then computes |rotated centre| + r + w/2 exactly at ANY
            # pad angle. Sampling the four axis extremes under-covered rotated
            # pads (#418, ottercast MK1: a 315deg unfilled ring lost ~0.4mm of
            # copper per side vs pcbnew).
            out.append(([(cx, cy)], r + hw))
            continue
        elif m2.group(1) in ('rect', 'line'):
            sm = re.search(r'\(start\s+(-?[\d.]+)\s+(-?[\d.]+)\)\s+\(end\s+(-?[\d.]+)\s+(-?[\d.]+)\)', block)
            if sm:
                x1, y1, x2, y2 = map(float, sm.groups())
                pts += [(x1, y1), (x2, y2)] if m2.group(1) == 'line' else \
                       [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
        if pts:
            out.append((pts, hw))
    return out


def _offset_polygon_outward(pts, hw):
    """Offset a closed polygon outward by ``hw`` (mitered per-vertex normals,
    miter capped at 4x for very sharp corners). Winding is derived from the
    shoelace sum, so either vertex order works. Approximate at reflex
    (concave) vertices -- the notch is offset too, mildly over-covering it --
    which is conservative for clearance to external copper."""
    n = len(pts)
    if n < 3 or hw <= 0:
        return pts
    area2 = sum(pts[i][0] * pts[(i + 1) % n][1] - pts[(i + 1) % n][0] * pts[i][1]
                for i in range(n))
    sign = 1.0 if area2 > 0 else -1.0
    normals = []
    for i in range(n):
        dx = pts[(i + 1) % n][0] - pts[i][0]
        dy = pts[(i + 1) % n][1] - pts[i][1]
        length = math.hypot(dx, dy)
        normals.append((sign * dy / length, -sign * dx / length)
                       if length > 1e-12 else None)
    out = []
    for i in range(n):
        adj = [v for v in (normals[i - 1], normals[i]) if v is not None]
        if not adj:
            out.append(pts[i])
            continue
        ax = sum(v[0] for v in adj)
        ay = sum(v[1] for v in adj)
        length = math.hypot(ax, ay)
        if length < 1e-12:
            out.append(pts[i])
            continue
        bx, by = ax / length, ay / length
        cos_half = max(0.25, bx * adj[0][0] + by * adj[0][1])
        out.append((pts[i][0] + bx * hw / cos_half,
                    pts[i][1] + by * hw / cos_half))
    return out


def _custom_pad_board_extent(pad_text: str, abs_rotation_deg: float,
                             anchor_x: float, anchor_y: float) -> Tuple[float, float]:
    """Board-frame half-extents (|x|, |y|) of a custom pad's copper about its
    anchor: primitive points (per-primitive stroke half-width) plus the anchor
    rect's corners, all rotated by the pad's ABSOLUTE angle (KiCad negate
    convention). Matches build_pcb_data_from_board's symmetric bounding box, so
    a 27deg/45deg custom pad models the same axis-aligned copper on both paths
    (sofle_pico D26/LED*, eurorack_pmod U1). Returns (0, 0) with no primitives.
    """
    prims = _custom_pad_primitive_points(pad_text)
    if not prims:
        return 0.0, 0.0
    rad = math.radians(-abs_rotation_deg)
    cos_r, sin_r = math.cos(rad), math.sin(rad)
    ext_x = ext_y = 0.0
    hx, hy = anchor_x / 2.0, anchor_y / 2.0
    prims = prims + [([(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)], 0.0)]
    for pts, hw in prims:
        for vx, vy in pts:
            bx = vx * cos_r - vy * sin_r
            by = vx * sin_r + vy * cos_r
            ext_x = max(ext_x, abs(bx) + hw)
            ext_y = max(ext_y, abs(by) + hw)
    return ext_x, ext_y


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
    # Beziers aren't approximated yet -- fall back to the bbox for the whole pad.
    # (gr_arc strokes ARE handled below: linearized into a width band. sofle_pico's
    # rotated LED lens pads are drawn from gr_arc+gr_line strokes; bailing to the
    # bbox modelled ~10x more copper than exists and made phantom PAD-SEGMENT.)
    if re.search(r'\(gr_curve\b', prim):
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

    def _stroke_band(pts, hw):
        """Polygon for a stroked polyline: offset +-hw using per-vertex averaged
        normals, ends extended by hw (square cap covers the round cap)."""
        if len(pts) < 2 or hw <= 0:
            return None
        # extend ends along their tangents so the cap is covered
        (x0, y0), (x1, y1) = pts[0], pts[1]
        d0 = math.hypot(x1 - x0, y1 - y0) or 1.0
        pts = [(x0 - (x1 - x0) / d0 * hw, y0 - (y1 - y0) / d0 * hw)] + pts[1:]
        (xa, ya), (xb, yb) = pts[-2], pts[-1]
        d1 = math.hypot(xb - xa, yb - ya) or 1.0
        pts = pts[:-1] + [(xb + (xb - xa) / d1 * hw, yb + (yb - ya) / d1 * hw)]
        normals = []
        for i in range(len(pts)):
            acc_x = acc_y = 0.0
            for j in (i - 1, i):
                if 0 <= j < len(pts) - 1:
                    dx = pts[j + 1][0] - pts[j][0]
                    dy = pts[j + 1][1] - pts[j][1]
                    L = math.hypot(dx, dy)
                    if L > 1e-12:
                        acc_x += -dy / L
                        acc_y += dx / L
            L = math.hypot(acc_x, acc_y)
            normals.append((acc_x / L, acc_y / L) if L > 1e-12 else (0.0, 0.0))
        left = [(p[0] + n[0] * hw, p[1] + n[1] * hw) for p, n in zip(pts, normals)]
        right = [(p[0] - n[0] * hw, p[1] - n[1] * hw) for p, n in zip(pts, normals)]
        return left + right[::-1]

    polys = []
    # Iterate primitives in order; one polygon per primitive, transformed to global.
    for pm2 in re.finditer(r'\(gr_(poly|circle|rect|line|arc)\b', prim):
        kind = pm2.group(1)
        block = prim[pm2.start():find_matching_paren(prim, pm2.start()) - 1]
        local = None
        if kind == 'poly':
            # Walk the pts list in order; KiCad 7+ outlines mix (xy ...) with
            # (arc (start)(mid)(end)) entries -- and can be arcs ONLY
            # (thunderscope U11's rounded B0QFN pads). Linearize each arc in
            # place so the polygon follows the real curved outline.
            pts = []
            for m in re.finditer(r'\(xy\s+(-?[\d.]+)\s+(-?[\d.]+)\)|' + _PTS_ARC_RE, block):
                if m.group(1) is not None:
                    pts.append((float(m.group(1)), float(m.group(2))))
                else:
                    s = (float(m.group(3)), float(m.group(4)))
                    a_mid = (float(m.group(5)), float(m.group(6)))
                    e = (float(m.group(7)), float(m.group(8)))
                    segs = _arc_to_segments(s, a_mid, e)
                    pts.append(segs[0][0])
                    for _p1, p2 in segs:
                        pts.append(p2)
            local = pts if len(pts) >= 3 else None
            if local:
                # KiCad strokes the gr_poly OUTLINE with its (width ...) --
                # for filled and unfilled polys alike -- so copper reaches
                # width/2 beyond the outline (#418). Offset the outline
                # outward by that half-stroke. An unfilled poly's interior
                # is not really copper, but the filled-outline model is kept
                # (conservative, and exact for clearance to external copper).
                hw = _width(block) / 2.0
                if hw > 0:
                    local = _offset_polygon_outward(local, hw)
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
        elif kind == 'arc':
            # 3-point arc stroke: linearize the centerline, offset into a band.
            am = re.search(_PTS_ARC_RE, block)
            if am:
                s = (float(am.group(1)), float(am.group(2)))
                a_mid = (float(am.group(3)), float(am.group(4)))
                e = (float(am.group(5)), float(am.group(6)))
                cpts = [s]
                for _p1, p2 in _arc_to_segments(s, a_mid, e):
                    cpts.append(p2)
                local = _stroke_band(cpts, max(_width(block) / 2.0, 1e-3))
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

    # A custom pad's copper is the ANCHOR SHAPE union the primitives (#337).
    # Modeling primitives alone under-covers: urchin's router drilled a via
    # inside D31.2's 0.95x2.29 anchor rect because only the small finger
    # primitive was in the polygon set, and check_drc graded the real contact
    # clean for the same reason. KiCad's default anchor is a circle; the file
    # writes (options ... (anchor rect)) when it is a rectangle.
    sm = re.search(r'\(size\s+(-?[\d.]+)\s+(-?[\d.]+)\)', pad_text)
    if sm:
        ax, ay = float(sm.group(1)), float(sm.group(2))
        if ax > 0 and ay > 0:
            am = re.search(r'\(anchor\s+(\w+)\)', pad_text)
            akind = am.group(1) if am else 'circle'
            if akind == 'rect':
                anchor_local = [(-ax / 2, -ay / 2), (ax / 2, -ay / 2),
                                (ax / 2, ay / 2), (-ax / 2, ay / 2)]
            else:
                r = min(ax, ay) / 2.0
                anchor_local = [(r * math.cos(i * math.pi / 8),
                                 r * math.sin(i * math.pi / 8)) for i in range(16)]
            polys.append(to_global(anchor_local))
    return polys or None


def pad_drill_capsule(pad) -> Tuple[Tuple[float, float], Tuple[float, float], float]:
    """A pad's drill hole as a capsule: (end1, end2, radius) in board mm.

    KiCad slot drills -- ``(drill oval w h)`` -- are milled slots, not round
    holes. Modelling one as a circle of its LONG dimension (the back-compat
    ``pad.drill`` scalar) turns a 30.2x1mm milled slot into a phantom 15mm-
    radius hole that "conflicts" with every track within reach (thunderscope:
    817 phantom copper-to-hole violations; KiCad's own DRC reports none).
    The capsule axis follows the pad's absolute rotation with the same negate
    convention as local_to_global. Round drills (and drills parsed before the
    slot fields existed) return a zero-length capsule at the pad centre.
    """
    w = getattr(pad, 'drill_w', 0.0) or 0.0
    h = getattr(pad, 'drill_h', 0.0) or 0.0
    # Offset pads (#325): the DRILL stays at the anchor (hole_x/hole_y) while
    # global_x/global_y is the copper centre.
    hx = getattr(pad, 'hole_x', None)
    hy = getattr(pad, 'hole_y', None)
    px = hx if hx is not None else pad.global_x
    py = hy if hy is not None else pad.global_y
    if w <= 0 or h <= 0 or abs(w - h) < 1e-9:
        centre = (px, py)
        return centre, centre, (pad.drill or max(w, h)) / 2.0
    radius = min(w, h) / 2.0
    half_span = (max(w, h) - min(w, h)) / 2.0
    # Local long-axis direction: +x when w >= h, else +y; rotate to board frame.
    rad = math.radians(-(getattr(pad, 'rotation', 0.0) or 0.0))
    cos_r, sin_r = math.cos(rad), math.sin(rad)
    if w >= h:
        ux, uy = cos_r, sin_r          # local (1, 0)
    else:
        ux, uy = -sin_r, cos_r         # local (0, 1)
    p1 = (px - ux * half_span, py - uy * half_span)
    p2 = (px + ux * half_span, py + uy * half_span)
    return p1, p2, radius


def pad_is_plated_through(pad) -> bool:
    """True if the pad's hole has a PLATED copper barrel tying every copper
    layer together (a real through-hole pin/via-in-pad). `drill > 0` alone is
    NOT enough: an NPTH pad (`pad_type == 'np_thru_hole'`) has a hole but NO
    copper, so treating it as an all-layer connection point invents copper
    that isn't there (issue #328 -- a net-tied mounting hole would seed a
    plane / offer launch layers on nothing). Use this instead of bare
    `pad.drill > 0` wherever the question is "does this pad connect layers"."""
    return ((getattr(pad, 'drill', 0.0) or 0.0) > 0
            and getattr(pad, 'pad_type', '') != 'np_thru_hole')


def pad_drill_circles(pad, step: float = 0.0) -> List[Tuple[float, float, float]]:
    """A pad's drill hole as (x, y, diameter) circles for circle-based hole
    keep-outs (obstacle maps, via hole-to-hole tests).

    Round drills yield the single (x, y, drill) circle -- identical to the old
    behaviour. Slot drills yield circles of the slot's SHORT dimension sampled
    along the capsule axis (default spacing radius/2, max union sag ~3%% of the
    radius), so the keep-out follows the real slot instead of a long-axis disc.
    """
    p1, p2, r = pad_drill_capsule(pad)
    if r <= 0:
        return []
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    span = math.hypot(dx, dy)
    if span < 1e-9:
        return [(p1[0], p1[1], 2.0 * r)]
    if step <= 0:
        step = r / 2.0
    n = max(1, int(math.ceil(span / step)))
    return [(p1[0] + dx * (i / n), p1[1] + dy * (i / n), 2.0 * r)
            for i in range(n + 1)]


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


def _unescape_kicad_string(s: str) -> str:
    """Undo KiCad s-expression string escapes (backslash and quote).

    The file stores net names like ROT_RDY\\ROT_GPIO0 with the backslash
    escaped; pcbnew returns the unescaped name, so keeping the raw text made
    every such net name diff between the two parsers. Applied to DISPLAY
    names only -- name_to_id lookups key on the raw file text, which every
    in-file reference shares.
    """
    if '\\' not in s:
        return s
    return s.replace('\\\\', '\\').replace('\\"', '"')


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

    # Extract board outline polygon(s) and cutouts for non-rectangular boards
    outers, cutouts = extract_board_contours(content)

    # Extract stackup information
    stackup = extract_stackup(content)

    return BoardInfo(layers=layers, copper_layers=copper_layers, board_bounds=bounds,
                     stackup=stackup, board_outline=(outers[0] if outers else []),
                     board_outlines=outers, board_cutouts=cutouts)


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

    # Adaptive resolution: cap the chord sag at ~5um so a large-radius,
    # wide-sweep arc (nebula_watch's ~350deg, r~20mm watch outline) doesn't
    # under-reach its true extremes -- 16 fixed segments left the board
    # outline 57um short of the real bounding box. Small arcs keep the old
    # cost; the count is capped to stay bounded on huge radii.
    if radius > 0:
        max_step = 2.0 * math.acos(max(-1.0, min(1.0, 1.0 - 0.005 / radius)))
        if max_step > 0:
            num_segments = max(num_segments,
                               min(512, int(math.ceil(abs(sweep) / max_step))))

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
# ... and must not cross a (layer ...)/(layers ...) tag either: the first
# layer tag after an element's fields is that element's own layer, so allowing
# the gap to run past one lets a layer-less element (e.g. a pad-primitive
# gr_line) borrow a LATER element's layer tag and materialize as a phantom
# board graphic.
_GR_ELEMENT_GAP = r'(?:(?!\(gr_|\(layers?\b)[\s\S])*?'


def _mask_pad_primitives(content: str) -> str:
    """Blank out pad ``(primitives ...)`` blocks before board-level graphic scans.

    Custom pads draw their copper with gr_line/gr_arc/gr_poly PRIMITIVES. The
    board-level gr_* scanners (Edge.Cuts bounds/outline, guide corridors,
    keepouts) regex over the whole file with a gap that only refuses to cross
    another ``(gr_`` token -- so a primitive's coordinates could be stitched to
    a LATER element's layer tag (comexpress7: a thermal-spoke gr_line inside an
    F.Paste aperture pad + a footprint fp_line's ``(layer "User.1")`` = a
    phantom guide path with pad-local coordinates). Masking the primitives
    blocks keeps those scans to real board graphics. Returns content unchanged
    when there are no primitives.
    """
    out = []
    pos = 0
    for m in re.finditer(r'\(primitives\b', content):
        if m.start() < pos:
            continue
        end = find_matching_paren(content, m.start())
        if end <= m.start():
            continue
        out.append(content[pos:m.start()])
        # Leave a (gr_ barrier token in place of the block: the gr_* scanners'
        # gap regex refuses to cross (gr_ tokens, and blanking the primitives
        # entirely would REMOVE barriers that stopped an earlier element's
        # scan from bridging across this pad.
        barrier = '(gr_masked)'
        pad_len = end - m.start()
        out.append(barrier + ' ' * (pad_len - len(barrier)) if pad_len >= len(barrier)
                   else ' ' * pad_len)
        pos = end
    if not out:
        return content
    out.append(content[pos:])
    return ''.join(out)


def _footprint_edge_points(content: str) -> List[Tuple[float, float]]:
    """GLOBAL points of footprint-embedded Edge.Cuts shapes (fp_line/fp_rect/
    fp_arc/fp_circle/fp_poly), transformed by each footprint's (at x y rot).

    Some boards keep their entire outline inside a footprint (Adiuvo's
    rp2350_fpga_eensy: fp_lines on Edge.Cuts in an outline footprint); a
    board-level gr_* scan alone reports no bounds, which silently disables
    the routing edge keep-out. KiCad's own edges bounding box includes
    footprint shapes, so these points belong in board_bounds."""
    pts: List[Tuple[float, float]] = []
    if '"Edge.Cuts"' not in content:
        return pts
    for m in re.finditer(r'\(footprint\s+"', content):
        start = m.start()
        end = find_matching_paren(content, start)
        fp_text = content[start:end]
        if '"Edge.Cuts"' not in fp_text:
            continue
        at_match = re.search(r'\(at\s+([\d.-]+)\s+([\d.-]+)(?:\s+([\d.-]+))?\)', fp_text)
        if not at_match:
            continue
        fx, fy = float(at_match.group(1)), float(at_match.group(2))
        frot = float(at_match.group(3)) if at_match.group(3) else 0.0
        local: List[Tuple[float, float]] = []
        for sm in re.finditer(
                r'\(fp_(line|rect)\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+'
                r'\(end\s+([\d.-]+)\s+([\d.-]+)\)' + _GR_ELEMENT_GAP +
                r'\(layer\s+"Edge\.Cuts"\)', fp_text, re.DOTALL):
            x1, y1, x2, y2 = (float(sm.group(i)) for i in range(2, 6))
            if sm.group(1) == 'rect':
                # all four corners: under rotation the rect tilts, and the
                # two derived corners bound it where start/end alone don't
                local += [(x1, y1), (x2, y2), (x1, y2), (x2, y1)]
            else:
                local += [(x1, y1), (x2, y2)]
        for sm in re.finditer(
                r'\(fp_arc\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+'
                r'\(mid\s+([\d.-]+)\s+([\d.-]+)\)\s+'
                r'\(end\s+([\d.-]+)\s+([\d.-]+)\)' + _GR_ELEMENT_GAP +
                r'\(layer\s+"Edge\.Cuts"\)', fp_text, re.DOTALL):
            sx, sy, mx, my, ex, ey = (float(sm.group(i)) for i in range(1, 7))
            for seg in _arc_to_segments((sx, sy), (mx, my), (ex, ey)):
                local += list(seg)
        for lx, ly in local:
            pts.append(local_to_global(fx, fy, frot, lx, ly))
        for sm in re.finditer(
                r'\(fp_circle\s+\(center\s+([\d.-]+)\s+([\d.-]+)\)\s+'
                r'\(end\s+([\d.-]+)\s+([\d.-]+)\)' + _GR_ELEMENT_GAP +
                r'\(layer\s+"Edge\.Cuts"\)', fp_text, re.DOTALL):
            cx, cy, ex, ey = (float(sm.group(i)) for i in range(1, 5))
            r = math.hypot(ex - cx, ey - cy)
            gcx, gcy = local_to_global(fx, fy, frot, cx, cy)
            # circle bounds are rotation-invariant about the moved center
            pts += [(gcx - r, gcy - r), (gcx + r, gcy + r)]
        for pm in re.finditer(r'\(fp_poly\s*\(pts', fp_text):
            p_end = find_matching_paren(fp_text, pm.start())
            poly_text = fp_text[pm.start():p_end]
            if not re.search(r'\(layer\s+"Edge\.Cuts"\)', poly_text):
                continue
            for xm in re.finditer(r'\(xy\s+([\d.-]+)\s+([\d.-]+)\)', poly_text):
                pts.append(local_to_global(fx, fy, frot,
                                           float(xm.group(1)), float(xm.group(2))))
    return pts


def extract_board_bounds(content: str) -> Optional[Tuple[float, float, float, float]]:
    """Extract board outline bounds from Edge.Cuts layer."""
    content = _mask_pad_primitives(content)  # pad primitives are not board graphics
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

    # Footprint-embedded Edge.Cuts (fp_* shapes, transformed to global):
    # boards whose outline lives in a footprint otherwise report no bounds
    # and route with no edge keep-out. Mirrored in build_pcb_data_from_board.
    for px, py in _footprint_edge_points(content):
        min_x = min(min_x, px)
        max_x = max(max_x, px)
        min_y = min(min_y, py)
        max_y = max(max_y, py)
        found = True

    if found:
        return (min_x, min_y, max_x, max_y)
    return None


def _bezier_to_segments(p0, p1, p2, p3, num_segments: int = 16):
    """Cubic bezier -> consecutive line segments (de Casteljau sampling)."""
    pts = []
    for i in range(num_segments + 1):
        t = i / num_segments
        mt = 1.0 - t
        x = (mt ** 3) * p0[0] + 3 * (mt ** 2) * t * p1[0] + 3 * mt * (t ** 2) * p2[0] + (t ** 3) * p3[0]
        y = (mt ** 3) * p0[1] + 3 * (mt ** 2) * t * p1[1] + 3 * mt * (t ** 2) * p2[1] + (t ** 3) * p3[1]
        pts.append((x, y))
    return list(zip(pts, pts[1:]))


def _circle_to_segments(cx, cy, r, num_segments: int = 64):
    """Circle -> closed ring of line segments."""
    pts = [(cx + r * math.cos(2 * math.pi * k / num_segments),
            cy + r * math.sin(2 * math.pi * k / num_segments))
           for k in range(num_segments + 1)]
    return list(zip(pts, pts[1:]))


def _collect_edge_cuts_segments(content: str) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Parse all gr_line, gr_arc, and gr_rect segments on Edge.Cuts from file content."""
    content = _mask_pad_primitives(content)  # pad primitives are not board graphics
    segments = []

    # gr_line
    line_pattern = r'\(gr_line\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)' + _GR_ELEMENT_GAP + r'\(layer\s+"Edge\.Cuts"\)'
    for m in re.finditer(line_pattern, content, re.DOTALL):
        x1, y1, x2, y2 = float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))
        # Skip degenerate zero-length lines (matches the pcbnew extraction):
        # zynq_ad9364 has dot-lines on Edge.Cuts that only duplicate vertices.
        if abs(x2 - x1) < 0.001 and abs(y2 - y1) < 0.001:
            continue
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

    # gr_curve - cubic bezier corner/edge (#304: crkbd's outline corners are
    # 100 gr_curves; without them the outline never chains closed, the halves'
    # small slots masquerade as the whole outline, and every track "leaves the
    # board"). Control points are (pts (xy)x4) in GLOBAL coords.
    curve_pattern = (r'\(gr_curve\s+\(pts\s+'
                     r'\(xy\s+([\d.-]+)\s+([\d.-]+)\)\s*\(xy\s+([\d.-]+)\s+([\d.-]+)\)\s*'
                     r'\(xy\s+([\d.-]+)\s+([\d.-]+)\)\s*\(xy\s+([\d.-]+)\s+([\d.-]+)\)\s*\)'
                     + _GR_ELEMENT_GAP + r'\(layer\s+"Edge\.Cuts"\)')
    for m in re.finditer(curve_pattern, content, re.DOTALL):
        g = [float(v) for v in m.groups()]
        segments.extend(_bezier_to_segments((g[0], g[1]), (g[2], g[3]),
                                            (g[4], g[5]), (g[6], g[7])))

    # gr_circle - a standalone ring (mounting hole / round cutout)
    circle_pattern = (r'\(gr_circle\s+\(center\s+([\d.-]+)\s+([\d.-]+)\)\s+'
                      r'\(end\s+([\d.-]+)\s+([\d.-]+)\)' + _GR_ELEMENT_GAP
                      + r'\(layer\s+"Edge\.Cuts"\)')
    for m in re.finditer(circle_pattern, content, re.DOTALL):
        cx, cy, ex, ey = (float(v) for v in m.groups())
        r = math.hypot(ex - cx, ey - cy)
        if r > 1e-6:
            segments.extend(_circle_to_segments(cx, cy, r))

    # Footprint-embedded Edge.Cuts (#304): reversible split keyboards draw
    # per-LED cutout windows as fp_lines on Edge.Cuts inside the LED
    # footprints (crkbd: 184 of them). File-local coords are already
    # side-resolved; transform = footprint position + rotation with the KiCad
    # negate convention (verified against pcbnew GraphicalItems on crkbd
    # LED16, flipped + rot 180).
    segments.extend(_collect_footprint_edge_segments(content))

    return segments


def _collect_footprint_edge_segments(content: str):
    """Edge.Cuts fp_line/fp_arc/fp_circle/fp_rect segments inside footprints,
    transformed to global coordinates (issue #304)."""
    out = []
    for fm in re.finditer(r'\(footprint\s+"[^"]*"', content):
        end = find_matching_paren(content, fm.start())
        block = content[fm.start():end]
        if '"Edge.Cuts"' not in block:
            continue
        at = re.search(r'\(at\s+([-\d.]+)\s+([-\d.]+)(?:\s+([-\d.]+))?\)', block)
        if not at:
            continue
        fx, fy = float(at.group(1)), float(at.group(2))
        rot = float(at.group(3)) if at.group(3) else 0.0
        rad = math.radians(-rot)
        cos_r, sin_r = math.cos(rad), math.sin(rad)

        def to_g(x, y):
            return (fx + x * cos_r - y * sin_r, fy + x * sin_r + y * cos_r)

        gap = _GR_ELEMENT_GAP
        for m in re.finditer(r'\(fp_line\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)'
                             + gap + r'\(layer\s+"Edge\.Cuts"\)', block, re.DOTALL):
            a = to_g(float(m.group(1)), float(m.group(2)))
            b = to_g(float(m.group(3)), float(m.group(4)))
            if abs(b[0] - a[0]) >= 0.001 or abs(b[1] - a[1]) >= 0.001:
                out.append((a, b))
        for m in re.finditer(r'\(fp_rect\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)'
                             + gap + r'\(layer\s+"Edge\.Cuts"\)', block, re.DOTALL):
            x1, y1, x2, y2 = (float(v) for v in m.groups())
            c = [to_g(x1, y1), to_g(x2, y1), to_g(x2, y2), to_g(x1, y2)]
            out.extend([(c[0], c[1]), (c[1], c[2]), (c[2], c[3]), (c[3], c[0])])
        for m in re.finditer(r'\(fp_arc\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(mid\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)'
                             + gap + r'\(layer\s+"Edge\.Cuts"\)', block, re.DOTALL):
            g = [float(v) for v in m.groups()]
            for p1, p2 in _arc_to_segments((g[0], g[1]), (g[2], g[3]), (g[4], g[5])):
                out.append((to_g(*p1), to_g(*p2)))
        for m in re.finditer(r'\(fp_circle\s+\(center\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)'
                             + gap + r'\(layer\s+"Edge\.Cuts"\)', block, re.DOTALL):
            cx, cy, ex, ey = (float(v) for v in m.groups())
            r = math.hypot(ex - cx, ey - cy)
            if r > 1e-6:
                for p1, p2 in _circle_to_segments(cx, cy, r):
                    out.append((to_g(*p1), to_g(*p2)))
    return out


def _parse_gr_polys_on_layer(content: str, layer: str) -> List[List[Tuple[float, float]]]:
    """Return the vertex list of every gr_poly drawn on the given layer."""
    content = _mask_pad_primitives(content)  # pad primitives are not board graphics
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
    content = _mask_pad_primitives(content)  # pad primitives are not board graphics
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
    content = _mask_pad_primitives(content)  # pad primitives are not board graphics
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

        # Loose ends = endpoints with no partner within tol. A group with
        # exactly two is an OPEN chain: KiCad's outline assembler
        # (ConvertOutlineToPolygon) closes it straight back to its start
        # rather than discarding it -- bus_pirate5's two panel slots close
        # across a 60mm synthesized edge. Walk such a chain from a loose
        # end (starting mid-chain consumes only one side and drops the
        # group); the self-intersection guard below then mirrors pcbnew's
        # normalization dropping sliver rings whose closure crosses their
        # own chain (the same board's panel-frame strips).
        endpoints = []
        for seg in group_segs:
            endpoints.append(seg[0])
            endpoints.append(seg[1])
        loose = [p for p in endpoints
                 if sum(1 for q in endpoints if approx_equal(p, q)) == 1]
        start_idx, start_flip = 0, False
        if len(loose) == 2:
            for i, seg in enumerate(group_segs):
                if approx_equal(seg[0], loose[0]):
                    start_idx, start_flip = i, False
                    break
                if approx_equal(seg[1], loose[0]):
                    start_idx, start_flip = i, True
                    break
        s0 = group_segs[start_idx]
        polygon = [s0[1], s0[0]] if start_flip else [s0[0], s0[1]]
        used = {start_idx}

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
        gap_filled = False
        if len(polygon) > 1 and approx_equal(polygon[0], polygon[-1]):
            polygon = polygon[:-1]
        elif len(loose) == 2 and len(used) == len(group_segs):
            gap_filled = True  # open chain: ring closes polygon[-1]->polygon[0]

        if len(used) != len(group_segs) or len(polygon) < 3:
            continue
        if gap_filled:
            # pcbnew parity: a gap-filled ring whose synthesized closing edge
            # properly CROSSES its own chain is a degenerate sliver (an open
            # panel-frame strip, not a real cutout) -- polygon normalization
            # drops it on the pcbnew side, so drop it here too. "Properly"
            # means penetrating deeper than the chaining tolerance: a real
            # slot's chain can graze its closure line by a sub-micron
            # excursion at the loose ends (bus_pirate5: 0.8um), while a
            # sliver strip oscillates across it by its full ~0.1mm width.
            c1, c2 = polygon[-1], polygon[0]
            clen = ((c2[0] - c1[0]) ** 2 + (c2[1] - c1[1]) ** 2) ** 0.5
            crosses = False
            if clen > tol:
                ux, uy = (c2[0] - c1[0]) / clen, (c2[1] - c1[1]) / clen
                for k in range(1, len(polygon) - 2):
                    p1, p2 = polygon[k], polygon[k + 1]
                    # signed perpendicular distance to the closure line
                    s1 = (p1[0] - c1[0]) * uy - (p1[1] - c1[1]) * ux
                    s2 = (p2[0] - c1[0]) * uy - (p2[1] - c1[1]) * ux
                    if (s1 > 0) == (s2 > 0) or min(abs(s1), abs(s2)) <= tol:
                        continue  # same side, or grazing shallower than tol
                    # crossing point must lie within the closure segment
                    t = s1 / (s1 - s2)
                    cx = p1[0] + (p2[0] - p1[0]) * t
                    cy = p1[1] + (p2[1] - p1[1]) * t
                    along = (cx - c1[0]) * ux + (cy - c1[1]) * uy
                    if tol < along < clen - tol:
                        crosses = True
                        break
            if crosses:
                continue
            area = 0.0
            perim = 0.0
            for k in range(len(polygon)):
                x1, y1 = polygon[k]
                x2, y2 = polygon[(k + 1) % len(polygon)]
                area += x1 * y2 - x2 * y1
                perim += ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
            area = abs(area) / 2.0
            # Mean-thickness sliver rule: a gap-filled ring thinner than any
            # manufacturable cutout (fab min slot ~0.8mm) is a V-cut / tab
            # guide LINE, not a hole -- pcbnew's normalization drops these
            # (bus_pirate5's three panel-frame strips, 0.013-0.24mm thick,
            # vs its real slots at ~3.8mm; the rule only applies to rings we
            # closed OURSELVES, never to deliberately drawn thin rings).
            if perim < tol or (2.0 * area) / perim < 0.5:
                continue
        contours.append(polygon)

    return contours


def _pt_in_ring(x: float, y: float, ring) -> bool:
    """Ray-cast point-in-polygon for a vertex ring."""
    inside = False
    n = len(ring)
    j = n - 1
    for i in range(n):
        xi, yi = ring[i]
        xj, yj = ring[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def _classify_contours(contours):
    """Split closed Edge.Cuts contours into (outers, cutouts) by CONTAINMENT.

    The old rule -- largest contour is THE outline, everything else a cutout --
    breaks boards with several disjoint outlines in one file (split keyboards:
    crkbd's right half became a "cutout", so every track in it graded as a
    board-edge violation, issue #304). A contour is a CUTOUT only if it lies
    inside another contour; disjoint top-level contours are all outer
    boundaries. Containment is decided by majority vote over sampled vertices
    (robust to shared/touching vertices). Nested islands (ring inside a cutout)
    are rare and classified as cutouts -- conservative.
    Returns (outers sorted largest-first, cutouts).
    """
    def bbox_area(c):
        xs = [p[0] for p in c]
        ys = [p[1] for p in c]
        return (max(xs) - min(xs)) * (max(ys) - min(ys))

    outers, cutouts = [], []
    for i, c in enumerate(contours):
        samples = c[:: max(1, len(c) // 5)][:5]
        contained = False
        for j, other in enumerate(contours):
            if j == i or len(other) < 3:
                continue
            votes = sum(1 for (px, py) in samples if _pt_in_ring(px, py, other))
            if votes * 2 > len(samples):
                contained = True
                break
        (cutouts if contained else outers).append(c)
    outers.sort(key=bbox_area, reverse=True)
    return outers, cutouts


def extract_board_contours(content: str) -> Tuple[List[List[Tuple[float, float]]], List[List[Tuple[float, float]]]]:
    """Extract board outline and cutout polygons from Edge.Cuts layer.

    Returns:
        (outers, cutouts): outers is the list of OUTER boundary rings (largest
        first -- several on split-keyboard/panelized boards, issue #304), and
        cutouts the truly-contained interior rings. outers is empty if no
        outline is found or if it's a simple axis-aligned rectangle.
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

    outers, cutouts = _classify_contours(contours)
    if not outers:
        return [], []
    return outers, cutouts


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
        # (net "") is the canonical NO-NET (net 0), not a real net: pcbnew maps
        # it to net code 0, so synthesizing an id for it split no-net copper
        # onto a phantom net (comexpress7's two dangling F.Cu segments).
        name_to_id[""] = 0
        synthetic_id = 1
        for m in re.finditer(net_pattern, content):
            net_name = m.group(1)
            if net_name in name_to_id:
                continue  # Already seen
            nets[synthetic_id] = Net(net_id=synthetic_id,
                                     name=_unescape_kicad_string(net_name))
            name_to_id[net_name] = synthetic_id
            synthetic_id += 1
    else:
        # KiCad 9: nets are (net <id> "name")
        net_pattern = r'\(net\s+(\d+)\s+"([^"]*)"\)'
        for m in re.finditer(net_pattern, content):
            net_id = int(m.group(1))
            net_name = m.group(2)
            # Display name unescaped (pcbnew parity); raw name keeps resolving
            # in-file references.
            nets[net_id] = Net(net_id=net_id, name=_unescape_kicad_string(net_name))
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

        # Extract footprint name. May be EMPTY: KiCad writes (footprint "")
        # for reference-less drill/graphic footprints (thunderscope's 86 locked
        # NPTH dots) -- requiring a non-empty name silently dropped them and
        # their hole obstacles.
        fp_name_match = re.search(r'\(footprint\s+"([^"]*)"', fp_text)
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
        if ref_match:
            reference = ref_match.group(1)
        else:
            # Reference-less footprint (e.g. a locked NPTH drill dot). The
            # footprints dict is keyed by reference, so a shared '' / '?' key
            # would collapse them all onto one entry and lose the rest's pads
            # (86 NPTH holes on thunderscope). Key by the footprint's uuid --
            # build_pcb_data_from_board synthesizes the same key, so the GUI
            # and file models stay comparable.
            uid_match = re.search(r'\(uuid\s+"([^"]+)"', fp_text)
            reference = "#" + uid_match.group(1) if uid_match else "?"

        # Extract value (component part number or value)
        value_match = re.search(r'\(property\s+"Value"\s+"([^"]+)"', fp_text)
        value = value_match.group(1) if value_match else ""

        # Extract the do-not-populate flag from the footprint (attr ...) token.
        # KiCad 7+ writes a standalone `dnp` keyword among the attr flags
        # (e.g. "(attr smd dnp exclude_from_pos_files)"). A no-pop part is an
        # open circuit, so callers must not treat its pads as bridging two nets.
        attr_match = re.search(r'\(attr\b([^)]*)\)', fp_text)
        is_dnp = bool(attr_match and re.search(r'\bdnp\b', attr_match.group(1)))

        # Footprint-level (locked yes): appears in the block header, before the
        # first nested element. Limit the search there so a locked PAD or
        # graphic inside the footprint doesn't read as a locked footprint.
        _hdr_end = len(fp_text)
        for _tok in ('(property', '(pad', '(fp_'):
            _i = fp_text.find(_tok)
            if _i != -1:
                _hdr_end = min(_hdr_end, _i)
        is_locked = bool(re.search(r'\(locked\s+yes\)', fp_text[:_hdr_end]))

        # Footprint-level (clearance ...) override (issue #326). KiCad writes it
        # in the footprint header, after the properties but before any graphic/
        # pad/zone child, so bound the search there (a PAD's own (clearance ...)
        # or a custom pad's (clearance outline) must not match). Negative
        # overrides (KiCad allows them to SHRINK the netclass clearance) are
        # clamped to 0: this codebase models overrides only as keep-out floors,
        # so a shrinking override safely degrades to "no override".
        _clr_end = len(fp_text)
        for _tok in ('(pad', '(fp_', '(zone', '(model'):
            _i = fp_text.find(_tok)
            if _i != -1:
                _clr_end = min(_clr_end, _i)
        fp_clr_match = re.search(r'\(clearance\s+(-?[\d.]+)\)', fp_text[:_clr_end])
        fp_clearance = max(0.0, float(fp_clr_match.group(1))) if fp_clr_match else 0.0

        # Net-tie pad groups: (net_tie_pad_groups "1, 2" "3, 4") -- each quoted
        # string is one comma-separated group of pad numbers this footprint
        # deliberately shorts (Kelvin shunt / net-tie). Whitespace around the
        # numbers varies by KiCad version; strip it.
        net_tie_groups: List[List[str]] = []
        ntpg_match = re.search(r'\(net_tie_pad_groups\b([^)]*)\)', fp_text)
        if ntpg_match:
            for grp in re.findall(r'"([^"]*)"', ntpg_match.group(1)):
                pads_in_group = [p.strip() for p in grp.split(',') if p.strip()]
                if len(pads_in_group) >= 2:
                    net_tie_groups.append(pads_in_group)

        footprint = Footprint(
            reference=reference,
            footprint_name=fp_name,
            x=fp_x,
            y=fp_y,
            rotation=fp_rotation,
            layer=fp_layer,
            value=value,
            dnp=is_dnp,
            locked=is_locked,
            clearance=fp_clearance,
            net_tie_groups=net_tie_groups
        )

        # Extract pads
        # Pattern for pad: (pad "num" type shape ... (at x y [rot]) ... (size sx sy) ... (net id "name") ...)
        pad_pattern = r'\(pad\s+"([^"]*)"\s+(\w+)\s+(\w+)(.*?)\)\s*(?=\(pad|\(model|\(zone|\Z|$)'

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

            # #369 A4: the file's (at ... angle) is the pad's ABSOLUTE board
            # angle (3d197c0, verified physically) -- adding fp_rotation
            # double-counted it, so Pad.rotation was wrong on every rotated
            # footprint and pad_drill_capsule oriented slot drills 90° off
            # (size resolution below was already fixed to the absolute angle).
            total_rotation = pad_rotation % 360

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
            custom_resolved = False
            if pad_shape == 'custom':
                # pad_rotation: the file's (at ...) angle is the pad's ABSOLUTE
                # board angle (same convention as _custom_pad_global_polygons /
                # _resolve_pad_rect).
                ext_bx, ext_by = _custom_pad_board_extent(
                    pad_text, pad_rotation, size_x, size_y)
                if ext_bx > 0 or ext_by > 0:
                    # Board-frame symmetric box about the anchor, matching the
                    # pcbnew builder's convention exactly (rect_rotation 0) --
                    # a rotated custom pad otherwise modelled a different
                    # rectangle on the two paths.
                    size_x, size_y = 2.0 * ext_bx, 2.0 * ext_by
                    rect_rotation = 0.0
                    custom_resolved = True

            # Resolve the pad rectangle into board space. size_x/size_y are in
            # the pad's own frame; its absolute board orientation is the file's
            # pad_rotation (the (at ...) angle already includes the footprint's
            # rotation -- 3d197c0). For the common orthogonal cases bake the
            # rotation into the dimensions so all downstream axis-aligned
            # geometry stays exact; genuine diagonal pads keep a residual
            # rect_rotation the obstacle/DRC geometry applies.
            if not custom_resolved:
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
                net_name = _unescape_kicad_string(net_match.group(2))
            else:
                # KiCad 10: (net "name") with no numeric ID
                net_match_v10 = re.search(r'\(net\s+"([^"]*)"\)', pad_text)
                if net_match_v10 and name_to_id:
                    net_id = name_to_id.get(net_match_v10.group(1), 0)
                    net_name = _unescape_kicad_string(net_match_v10.group(1))
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
            oval_match = re.search(r'\(drill\s+oval\s+([\d.]+)(?:\s+([\d.]+))?', pad_text)
            drill_w = drill_h = 0.0
            if oval_match:
                # (drill oval w) with a single value is legal and means w x w
                # (dilemma J102) -- requiring two numbers parsed it as drill 0.
                drill_w = float(oval_match.group(1))
                drill_h = float(oval_match.group(2)) if oval_match.group(2) else drill_w
                drill_size = max(drill_w, drill_h)
            else:
                drill_match = re.search(r'\(drill\s+([\d.]+)', pad_text)
                drill_size = float(drill_match.group(1)) if drill_match else 0.0

            # Pad copper offset (#324): (drill (offset x y)) displaces the
            # pad COPPER relative to the pad anchor/hole (castellated module
            # paddles: RPi_Pico's 3.5x1.7 SMD pads carry (offset 0.9 0)).
            # Ignoring it modelled the copper up to its full offset away from
            # reality -- the router laid tracks THROUGH real pad copper and
            # check_drc graded the shorts clean (piantor).
            offset_match = re.search(r'\(offset\s+([-\d.]+)\s+([-\d.]+)\)', pad_text)
            pad_offset_x = float(offset_match.group(1)) if offset_match else 0.0
            pad_offset_y = float(offset_match.group(2)) if offset_match else 0.0

            # Extract roundrect_rratio for roundrect pads
            rratio_match = re.search(r'\(roundrect_rratio\s+([\d.]+)\)', pad_text)
            roundrect_rratio = float(rratio_match.group(1)) if rratio_match else 0.0

            # Extract per-pad local clearance override, e.g. fiducial keep-clear
            # rings carry (clearance 0.375). The leading "(" avoids matching the
            # footprint's (pad_to_mask_clearance ...) token. 0 = no override
            # (KiCad's explicit "(clearance 0)" also means inherit). A pad with
            # no override of its own inherits the FOOTPRINT-level override
            # (issue #326) -- resolved here so every consumer of
            # pad.local_clearance honors both. Negative (shrinking) overrides
            # clamp to 0 = no override (see the footprint-level comment).
            clr_match = re.search(r'\(clearance\s+(-?[\d.]+)\)', pad_text)
            local_clearance = max(0.0, float(clr_match.group(1))) if clr_match else 0.0
            if local_clearance == 0.0:
                local_clearance = fp_clearance

            # Calculate global coordinates
            global_x, global_y = local_to_global(fp_x, fp_y, fp_rotation, local_x, local_y)

            # Fold the copper offset into the pad position (#324/#325): the
            # shifted rect IS the pad copper, so every clearance/DRC/obstacle
            # consumer gets it for free. The offset rotates with the pad's
            # ABSOLUTE angle (the file's (at) angle; same negate convention as
            # local_to_global). For drilled pads the HOLE stays at the anchor:
            # recorded in hole_x/hole_y for the drill-geometry consumers
            # (pad_drill_capsule / through-hole cells). caravel U8 proved the
            # drilled case is not rare: its THT pins carry a 0.3mm copper
            # offset, and modelling the copper at the hole shipped 89 real
            # cross-net shorts that check_drc graded clean.
            pad_hole_x = pad_hole_y = None
            if pad_offset_x or pad_offset_y:
                if drill_size > 0:
                    pad_hole_x, pad_hole_y = global_x, global_y
                _orad = math.radians(-pad_rotation)
                global_x += pad_offset_x * math.cos(_orad) - pad_offset_y * math.sin(_orad)
                global_y += pad_offset_x * math.sin(_orad) + pad_offset_y * math.cos(_orad)

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
                drill_w=drill_w,
                drill_h=drill_h,
                roundrect_rratio=roundrect_rratio,
                rect_rotation=rect_rotation,
                local_clearance=local_clearance,
                polygons=pad_polygons,
                hole_x=pad_hole_x,
                hole_y=pad_hole_y
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
    # (via blind ...) / (via micro ...): the type token precedes (at ...)
    via_pattern = r'\(via\s+(?:blind\s+|micro\s+)?\(at\s+([\d.-]+)\s+([\d.-]+)\)\s+\(size\s+([\d.-]+)\)\s+\(drill\s+([\d.-]+)\)\s+\(layers\s+"([^"]+)"\s+"([^"]+)"\)\s+(?:\(free\s+(yes|no)\)\s+)?\(net\s+(\d+)\)\s+\(uuid\s+"([^"]+)"\)'

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
        via_pattern_v10 = r'\(via\s+(?:blind\s+|micro\s+)?\(at\s+([\d.-]+)\s+([\d.-]+)\)\s+\(size\s+([\d.-]+)\)\s+\(drill\s+([\d.-]+)\)\s+\(layers\s+"([^"]+)"\s+"([^"]+)"\).*?\(net\s+"([^"]*)"\)\s+\(uuid\s+"([^"]+)"\)'
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
    # (locked yes) is optional at the same two spots the segment patterns
    # allow it (#150 / #369 A7) -- without it a LOCKED arc parses to nothing,
    # never becomes an obstacle, and the router routes straight through it.
    arc_fields = (r'\(arc\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+'
                  r'\(mid\s+([\d.-]+)\s+([\d.-]+)\)\s+'
                  r'\(end\s+([\d.-]+)\s+([\d.-]+)\)\s+'
                  r'\(width\s+([\d.-]+)\)\s+(?:\(locked\s+yes\)\s+)?'
                  r'\(layer\s+"([^"]+)"\)\s+(?:\(locked\s+yes\)\s+)?\(net\s+')

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

    # Net-tied copper GRAPHICS (#337): KiCad renders gr_line / gr_arc drawn on
    # a copper layer as real copper (optionally carrying a (net ...)). They are
    # invisible as (segment) records, so without this pass both check_drc and
    # the router's obstacle model were blind to them -- openstint's router
    # placed a via 62um from a hand-drawn GND gr_line it could not see, and
    # kicad-cli flagged dozens of such contacts we graded clean. Parse them
    # into Segments tagged graphic=True (obstacle/DRC-real, never prunable).
    def _resolve_net(tok):
        if tok is None:
            return 0
        tok = tok.strip()
        if tok.startswith('"'):
            return name_to_id.get(tok[1:-1], 0) if name_to_id else 0
        try:
            return int(tok)
        except ValueError:
            return 0

    # #337: model copper GRAPHICS (gr_line/gr_arc/gr_poly/gr_rect/gr_circle on a
    # copper layer) as graphic=True copper. BLOCK-BASED extraction -- find each
    # (gr_* ...) block, then pull each field from within it regardless of ORDER
    # or extra tokens. A fixed field-order regex silently dropped a graphic with
    # a (locked yes) token, KiCad-6/7 (tstamp ..) instead of (uuid ..), net-
    # before-layer ordering, or extra stroke fields -- a silent miss in a
    # "never miss real copper" pass. Lines/arcs need a real stroke (width>0);
    # filled poly/rect/circle default the outline to the fab track width.
    def _emit_outline(pts, w, layer, nid, uuid, closed=True):
        if len(pts) < 2:
            return
        ew = w if w > 0 else defaults.TRACK_WIDTH
        seq = pts + [pts[0]] if closed else pts
        for a, b in zip(seq, seq[1:]):
            segments.append(Segment(
                start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1],
                width=ew, layer=layer, net_id=nid, uuid=uuid, graphic=True))

    def _blk_fields(blk):
        lm = re.search(r'\(layer\s+"([^"]+)"\)', blk)
        wm = re.search(r'\(width\s+([-\d.]+)\)', blk)
        nm = re.search(r'\(net\s+("[^"]*"|\d+)\)', blk)
        um = (re.search(r'\(uuid\s+"([^"]+)"\)', blk)
              or re.search(r'\(tstamp\s+([-\w]+)\)', blk))
        layer = lm.group(1) if lm else None
        return (layer, float(wm.group(1)) if wm else 0.0,
                _resolve_net(nm.group(1)) if nm else 0,
                um.group(1) if um else '')

    def _xy(blk, name):
        m = re.search(r'\(' + name + r'\s+([-\d.]+)\s+([-\d.]+)\)', blk)
        return (float(m.group(1)), float(m.group(2))) if m else None

    for tag in ('gr_line', 'gr_arc', 'gr_poly', 'gr_rect', 'gr_circle'):
        needle = '(' + tag
        pos = 0
        while True:
            i = content.find(needle, pos)
            if i < 0:
                break
            nxt = content[i + len(needle): i + len(needle) + 1]
            if nxt and (nxt.isalnum() or nxt == '_'):
                pos = i + len(needle)
                continue
            j = find_matching_paren(content, i)
            blk = content[i:j]
            pos = j
            layer, w, nid, uuid = _blk_fields(blk)
            if not layer or not layer.endswith('.Cu'):
                continue
            if tag == 'gr_line':
                if w <= 0:
                    continue
                a, b = _xy(blk, 'start'), _xy(blk, 'end')
                if a and b:
                    segments.append(Segment(
                        start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1],
                        width=w, layer=layer, net_id=nid, uuid=uuid, graphic=True))
            elif tag == 'gr_arc':
                if w <= 0:
                    continue
                a, mid, b = _xy(blk, 'start'), _xy(blk, 'mid'), _xy(blk, 'end')
                if a and mid and b:
                    for p0, p1 in _arc_to_segments(a, mid, b):
                        segments.append(Segment(
                            start_x=p0[0], start_y=p0[1], end_x=p1[0], end_y=p1[1],
                            width=w, layer=layer, net_id=nid, uuid=uuid, graphic=True))
            elif tag == 'gr_poly':
                pts = [(float(x), float(y)) for x, y in
                       re.findall(r'\(xy\s+([-\d.]+)\s+([-\d.]+)\)', blk)]
                _emit_outline(pts, w, layer, nid, uuid)
            elif tag == 'gr_rect':
                a, b = _xy(blk, 'start'), _xy(blk, 'end')
                if a and b:
                    _emit_outline([(a[0], a[1]), (b[0], a[1]),
                                   (b[0], b[1]), (a[0], b[1])], w, layer, nid, uuid)
            elif tag == 'gr_circle':
                c, e = _xy(blk, 'center'), _xy(blk, 'end')
                if c and e:
                    r = math.hypot(e[0] - c[0], e[1] - c[1])
                    _emit_outline([(c[0] + r * math.cos(k * math.pi / 8),
                                    c[1] + r * math.sin(k * math.pi / 8))
                                   for k in range(16)], w, layer, nid, uuid)

    return segments


def _iter_zone_blocks(content: str):
    """Yield the inner body of each top-level ``(zone ...)`` block.

    Zones are at the top level, indented with a single tab. Uses ``\\r?\\n`` to
    handle both Unix and Windows line endings. Each yielded string is the
    content between the opening ``(zone`` line and its matching closing paren.
    Shared by :func:`extract_zones` and :func:`extract_keepouts`.
    """
    # #369 A5: match the zone header token itself, not "(zone" + NEWLINE --
    # KiCad v5/v6 write zones single-line ("(zone (net 0) (layer F.Cu) ...")
    # and the newline-anchored pattern parsed ZERO zones/keepouts from those
    # files (every pour and rule area silently dropped).
    zone_start_pattern = r'\r?\n\t\(zone(?=[\s(])'
    for start_match in re.finditer(zone_start_pattern, content):
        # Find the matching closing paren (string-aware, so a lone paren inside
        # a quoted token cannot run the scan past the zone end — see issue #113).
        # start_match begins at the newline before "(zone"; locate that "(".
        open_idx = content.index('(', start_match.start())
        zone_end = find_matching_paren(content, open_idx) - 1
        body_start = open_idx + len('(zone')
        if zone_end <= body_start:
            continue
        yield content[body_start:zone_end]


def extract_zones(content: str, name_to_id: Dict[str, int] = None) -> List[Zone]:
    """Extract all filled zones from PCB file.

    Parses zone definitions including their net assignment, layer, and polygon outline.
    These are used for power planes and other filled copper areas.
    """
    zones = []

    for zone_content in _iter_zone_blocks(content):
        # Rule areas are routing restrictions, not copper -- skip regardless
        # of net clause. v5/v6 keepouts DO carry (net 0), so the old skip
        # (inside the no-numeric-net branch only) would have modeled them as
        # phantom net-0 pours once single-line zones parse (#369 A5).
        if '(keepout' in zone_content:
            continue
        # Extract net id - try KiCad 9 format first, then KiCad 10
        net_match = re.search(r'\(net\s+(\d+)\)', zone_content)
        net_match_v10 = None
        if net_match:
            net_id = int(net_match.group(1))
        else:
            # KiCad 10: (net "name") - first net reference in zone. Unescape
            # before the lookup: name_to_id is keyed by UNESCAPED Net.name
            # (#369 A12 -- escaped zone names resolved to net 0).
            if name_to_id:
                net_match_v10 = re.search(r'\(net\s+"((?:[^"\\]|\\.)*)"\)', zone_content)
            if net_match_v10:
                net_name_v10 = _unescape_kicad_string(net_match_v10.group(1))
                net_id = name_to_id.get(net_name_v10, 0)
            else:
                # No net clause at all: a NO-NET copper pour (net 0). It still
                # pours real copper (nitrokey/vfo_ctrl decorative fills), and
                # pcbnew keeps it -- dropping it under-modelled the board.
                net_id = 0

        # Extract net name. Unescaped like every other net_name in the model
        # (#369 A12: zones kept the RAW file text, so backslash/quote-named
        # plane nets evaded --nets filters and zone dedup keys).
        net_name_match = re.search(r'\(net_name\s+"((?:[^"\\]|\\.)*)"\)', zone_content)
        if net_name_match:
            net_name = _unescape_kicad_string(net_name_match.group(1))
        elif net_match_v10 is not None:
            # KiCad 10: net name was already captured (and unescaped) above
            net_name = net_name_v10
        else:
            net_name = ""

        # Extract layer(s). A zone can span several copper layers via
        # (layers "F.Cu" "In2.Cu" ...) -- emit one Zone per copper layer so the
        # obstacle/connectivity model sees its copper everywhere it exists
        # (single-layer (layer "...") zones are the common case). Search the
        # zone HEADER only: each (filled_polygon ...) inside the zone carries
        # its own (layer "...") tag, which would mask the (layers ...) clause
        # and pin a 5-layer zone to just its first filled layer.
        _hdr_end = len(zone_content)
        for _tok in ('(polygon', '(filled_polygon', '(keepout'):
            _i = zone_content.find(_tok)
            if _i != -1:
                _hdr_end = min(_hdr_end, _i)
        zone_header = zone_content[:_hdr_end]
        # Layer tokens are UNQUOTED in v5/v6 files ((layer F.Cu)) -- accept
        # both spellings (#369 A5).
        layer_match = re.search(r'\(layer\s+"?([^")\s]+)"?\)', zone_header)
        if layer_match:
            zone_layers = [layer_match.group(1)]
        else:
            layers_match = re.search(r'\(layers\s+([^)]+)\)', zone_header)
            if not layers_match:
                continue
            _toks = [t.strip('"') for t in
                     re.findall(r'"[^"]+"|\S+', layers_match.group(1))]
            zone_layers = [l for l in _toks
                           if l.endswith('.Cu') or l == '*.Cu']
            if not zone_layers:
                continue
        layer = zone_layers[0]

        # Extract UUID
        uuid_match = re.search(r'\(uuid\s+"([^"]+)"\)', zone_content)
        uuid = uuid_match.group(1) if uuid_match else ""

        # Fill semantics (#350). All three live before the polygon blocks, so
        # the header slice covers them: (priority N) is a direct zone child;
        # island_removal_mode / island_area_min sit inside the (fill ...) block.
        # Absent tokens mean KiCad defaults: priority 0, mode 0 (always remove
        # isolated islands).
        prio_match = re.search(r'\(priority\s+(\d+)\)', zone_header)
        priority = int(prio_match.group(1)) if prio_match else 0
        irm_match = re.search(r'\(island_removal_mode\s+(\d+)\)', zone_header)
        island_removal_mode = int(irm_match.group(1)) if irm_match else 0
        iam_match = re.search(r'\(island_area_min\s+([\d.]+)\)', zone_header)
        island_area_min = float(iam_match.group(1)) if iam_match else 0.0

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

        for zl in zone_layers:
            zones.append(Zone(
                net_id=net_id,
                net_name=net_name,
                layer=zl,
                polygon=polygon,
                uuid=uuid,
                priority=priority,
                island_removal_mode=island_removal_mode,
                island_area_min=island_area_min
            ))

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

        # Layers: (layers "F.Cu" "In1.Cu" ...) or single (layer "F.Cu").
        # v5/v6 write tokens UNQUOTED ((layers F&B.Cu)) -- accept both (#369 A5).
        lm = re.search(r'\(layers\s+([^)]+)\)', zc) or re.search(r'\(layer\s+("?[^")\s]+"?)\)', zc)
        layers = ({t.strip('"') for t in re.findall(r'"[^"]+"|\S+', lm.group(1))}
                  if lm else set())

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
    # copper_id_to_name: COPPER ONLY -- the copper-layer enumeration and the
    # pad '*.Cu' wildcard collapse iterate this; mixing non-copper tokens in
    # broke both. id_to_name (below) adds the non-copper tokens for
    # get_layer_name.
    copper_id_to_name = {pcbnew.F_Cu: 'F.Cu', pcbnew.B_Cu: 'B.Cu'}
    for i in range(1, 31):
        layer_id = getattr(pcbnew, f'In{i}_Cu', None)
        if layer_id is not None:
            copper_id_to_name[layer_id] = f'In{i}.Cu'
    id_to_name = dict(copper_id_to_name)

    # Non-copper layers: the .kicad_pcb s-expression always uses the CANONICAL
    # short tokens (F.SilkS, B.Mask, Dwgs.User, ...) even when the user renamed
    # the layer and even though pcbnew's display names are the modern long
    # forms (F.Silkscreen) or the user names ("Bottom Solder, Bot Mask") -- a
    # silk/mask zone's layer diffed against the file parse for exactly that.
    for attr, token in (('F_SilkS', 'F.SilkS'), ('B_SilkS', 'B.SilkS'),
                        ('F_Mask', 'F.Mask'), ('B_Mask', 'B.Mask'),
                        ('F_Paste', 'F.Paste'), ('B_Paste', 'B.Paste'),
                        ('F_Adhes', 'F.Adhes'), ('B_Adhes', 'B.Adhes'),
                        ('F_Fab', 'F.Fab'), ('B_Fab', 'B.Fab'),
                        ('F_CrtYd', 'F.CrtYd'), ('B_CrtYd', 'B.CrtYd'),
                        ('Dwgs_User', 'Dwgs.User'), ('Cmts_User', 'Cmts.User'),
                        ('Eco1_User', 'Eco1.User'), ('Eco2_User', 'Eco2.User'),
                        ('Edge_Cuts', 'Edge.Cuts'), ('Margin', 'Margin')):
        layer_id = getattr(pcbnew, attr, None)
        if layer_id is not None:
            id_to_name[layer_id] = token

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
        copper_on = [lname for lid, lname in copper_id_to_name.items() if layer_set.Contains(lid)]
        if len(copper_on) == len(copper_id_to_name):
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

        # Silk layers - TH connector pads often list F.SilkS (a silk aperture
        # ring); dropping them made every such pad diff against the file parse.
        f_silk_id = getattr(pcbnew, 'F_SilkS', None)
        b_silk_id = getattr(pcbnew, 'B_SilkS', None)
        has_f_silk = f_silk_id is not None and layer_set.Contains(f_silk_id)
        has_b_silk = b_silk_id is not None and layer_set.Contains(b_silk_id)
        if has_f_silk and has_b_silk:
            layers.append('*.SilkS')
        else:
            if has_f_silk:
                layers.append('F.SilkS')
            if has_b_silk:
                layers.append('B.SilkS')

        # Remaining documentation layers a pad can legally list (Dwgs.User etc.)
        for attr, lname in (('Dwgs_User', 'Dwgs.User'), ('Cmts_User', 'Cmts.User'),
                            ('Eco1_User', 'Eco1.User'), ('Eco2_User', 'Eco2.User'),
                            ('F_Adhes', 'F.Adhes'), ('B_Adhes', 'B.Adhes'),
                            ('F_Fab', 'F.Fab'), ('B_Fab', 'B.Fab'),
                            ('F_CrtYd', 'F.CrtYd'), ('B_CrtYd', 'B.CrtYd')):
            lid = getattr(pcbnew, attr, None)
            if lid is not None and layer_set.Contains(lid):
                layers.append(lname)

        return layers

    # --- Extract board info ---
    layers_dict = {}
    copper_layers = []
    enabled = board.GetEnabledLayers()
    for lid, lname in id_to_name.items():
        if enabled.Contains(lid):
            layers_dict[lid] = lname

    # Copper layers must be in PHYSICAL STACKUP order (F.Cu, In1..InN, B.Cu) to
    # match the text parser (parse_kicad_pcb reads them in file/stackup order).
    # Iterating pcbnew layer-IDs instead puts B.Cu SECOND (its id is below the
    # inner-layer ids), and the fanout/router consume the layer list in order as
    # the layer-preference: with B.Cu 2nd, the BGA fanout piles ~all escapes onto
    # B.Cu instead of distributing across the inner layers, congesting the board
    # so the later signal route drops many nets. This was the GUI-only leg of the
    # rp2350 GUI-vs-CLI connectivity gap: build_pcb_data_from_board must stay at
    # parity with the text parser here. LSET.CuStack() yields front->back order.
    try:
        for lid in enabled.CuStack():
            lname = id_to_name.get(lid)
            if lname is None:
                try:
                    lname = board.GetLayerName(lid)
                except Exception:
                    lname = None
            if lname is not None:
                copper_layers.append(lname)
    except Exception:
        # Fallback to layer-id order if CuStack is unavailable on this build.
        for lid, lname in id_to_name.items():
            if enabled.Contains(lid) and lid in copper_id_to_name:
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
            # Footprint-embedded Edge.Cuts shapes count too (a board whose
            # whole outline lives in a footprint, e.g. rp2350_fpga_eensy) --
            # same shapes _footprint_edge_points reads on the text side.
            # FP_SHAPE is the KiCad 6/7 footprint-shape class.
            fp_items = [item for fp in board.GetFootprints()
                        for item in fp.GraphicalItems()]
            for drawing in list(board.GetDrawings()) + fp_items:
                if drawing.GetLayer() != edge_cuts_id:
                    continue
                class_name = drawing.GetClass()
                if class_name not in ("PCB_SHAPE", "DRAWSEGMENT", "FP_SHAPE"):
                    continue
                try:
                    bbox = drawing.GetBoundingBox()
                    w, h = bbox.GetWidth(), bbox.GetHeight()
                    if w <= 0 and h <= 0:
                        continue
                    x0, y0 = to_mm(bbox.GetX()), to_mm(bbox.GetY())
                    x1, y1 = to_mm(bbox.GetX() + w), to_mm(bbox.GetY() + h)
                    # GetBoundingBox is inflated by the stroke half-width; the
                    # board edge is the CENTERLINE of the Edge.Cuts stroke (the
                    # fab cuts on the line), which is also what the text parser
                    # returns. Deflate so GUI and CLI agree on board_bounds.
                    try:
                        hs = to_mm(drawing.GetWidth()) / 2.0
                    except Exception:
                        hs = 0.0
                    if hs > 0:
                        # per-axis: a straight line's bbox is exactly one
                        # stroke wide across it -- collapse that axis to the
                        # centerline instead of leaving it inflated.
                        if (x1 - x0) > 2 * hs:
                            x0, x1 = x0 + hs, x1 - hs
                        else:
                            x0 = x1 = (x0 + x1) / 2.0
                        if (y1 - y0) > 2 * hs:
                            y0, y1 = y0 + hs, y1 - hs
                        else:
                            y0 = y1 = (y0 + y1) / 2.0
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

    # Board outline(s) and cutouts from Edge.Cuts drawings
    board_outlines, board_cutouts = _extract_board_contours_from_pcbnew(board, to_mm)
    board_outline = board_outlines[0] if board_outlines else []

    # Stackup
    stackup = _extract_stackup_from_pcbnew(board, to_mm)

    board_info = BoardInfo(
        layers=layers_dict,
        copper_layers=copper_layers,
        board_bounds=board_bounds,
        stackup=stackup,
        board_outline=board_outline,
        board_outlines=board_outlines,
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
        if not reference:
            # Reference-less footprint: key by uuid, mirroring the text parser
            # (see extract_footprints_and_pads) so both models keep every such
            # footprint (NPTH drill dots) instead of collapsing them onto ''.
            try:
                reference = "#" + fp.m_Uuid.AsString()
            except Exception:
                reference = "?"

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

        # Locked flag — parity with the text parser's footprint (locked yes).
        try:
            fp_locked = bool(fp.IsLocked())
        except Exception:
            fp_locked = False

        # Footprint-level clearance override — parity with the text parser
        # (issue #326). GetLocalClearance() returns IU or an optional/None on
        # KiCad 8+; falsy = no override. Negative (shrinking) overrides clamp
        # to 0 like the text parser (overrides act only as keep-out floors).
        try:
            _fc = fp.GetLocalClearance()
            fp_clearance = max(0.0, to_mm(_fc)) if _fc else 0.0
        except Exception:
            fp_clearance = 0.0

        # Net-tie pad groups — parity with the text parser (Kelvin shunts /
        # net-tie parts; KiCad exempts the grouped pads' mutual clearance).
        # GetNetTiePadGroups() returns a vector of "1, 2"-style strings.
        try:
            fp_net_tie = []
            for _grp in fp.GetNetTiePadGroups():
                _nums = [p.strip() for p in str(_grp).split(',') if p.strip()]
                if len(_nums) >= 2:
                    fp_net_tie.append(_nums)
        except Exception:
            fp_net_tie = []

        footprint = Footprint(
            reference=reference,
            footprint_name=fp_name,
            x=fp_x,
            y=fp_y,
            rotation=fp_rotation,
            layer=fp_layer,
            value=fp_value,
            dnp=fp_dnp,
            locked=fp_locked,
            clearance=fp_clearance,
            net_tie_groups=fp_net_tie
        )

        # Extract pads
        for pad in fp.Pads():
            pad_num = pad.GetNumber()

            # Global position
            pad_pos = pad.GetPosition()
            global_x = to_mm(pad_pos.x)
            global_y = to_mm(pad_pos.y)

            # Pad copper offset (#324/#325), parity with the text parser:
            # fold the (rotated) copper offset into the position for ALL pads;
            # drilled pads record the hole (anchor) in hole_x/hole_y. pcbnew's
            # GetPosition() is the anchor/hole; GetOffset() is the copper
            # displacement in the pad frame, rotated by the pad's absolute
            # orientation.
            pcb_hole_x = pcb_hole_y = None
            try:
                _off = pad.GetOffset()
                if _off.x or _off.y:
                    if pad.GetDrillSize().x:
                        pcb_hole_x, pcb_hole_y = global_x, global_y
                    _oa = math.radians(-pad.GetOrientationDegrees())
                    _ox, _oy = to_mm(_off.x), to_mm(_off.y)
                    global_x += _ox * math.cos(_oa) - _oy * math.sin(_oa)
                    global_y += _ox * math.sin(_oa) + _oy * math.cos(_oa)
            except Exception:
                pass

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

            # #369 A4: pcbnew's GetOrientation is the pad's ABSOLUTE board
            # angle (GetFPRelativeOrientation is the relative one) -- adding
            # fp_rotation double-counted, mirroring the text-parser bug.
            total_rotation = pad_rotation % 360

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
                if drill_size.x != drill_size.y:
                    drill_w, drill_h = to_mm(drill_size.x), to_mm(drill_size.y)
                else:
                    drill_w = drill_h = 0.0
            except Exception:
                drill = 0.0
                drill_w = drill_h = 0.0

            # Roundrect ratio
            try:
                roundrect_rratio = pad.GetRoundRectRadiusRatio()
            except Exception:
                roundrect_rratio = 0.0

            # Per-pad local clearance override (fiducial keep-clear rings etc.).
            # GetLocalClearance() returns the pad's own override in IU, or a
            # falsy/None when unset depending on KiCad version. 0 = no override.
            # Pads without an override inherit the footprint-level one, matching
            # the text parser's resolution (issue #326); negatives clamp to 0.
            try:
                lc = pad.GetLocalClearance()
                local_clearance = max(0.0, to_mm(lc)) if lc else 0.0
            except Exception:
                local_clearance = 0.0
            if local_clearance == 0.0:
                local_clearance = fp_clearance

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
                drill_w=drill_w,
                drill_h=drill_h,
                roundrect_rratio=roundrect_rratio,
                rect_rotation=rect_rotation,
                local_clearance=local_clearance,
                polygons=pad_polygons,
                hole_x=pcb_hole_x,
                hole_y=pcb_hole_y
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
        elif track_class == "PCB_ARC":
            # Arc tracks: linearize with the same helper (and the same three
            # defining points) the text parser uses, so GUI and CLI see the
            # identical copper graph. Skipping them left the GUI's obstacle /
            # connectivity model blind to arc-routed copper (thunderscope:
            # 19,520 file-only segments in validate-board).
            try:
                s = (to_mm(track.GetStart().x), to_mm(track.GetStart().y))
                a_mid = (to_mm(track.GetMid().x), to_mm(track.GetMid().y))
                e = (to_mm(track.GetEnd().x), to_mm(track.GetEnd().y))
            except Exception:
                continue
            for p0, p1 in _arc_to_segments(s, a_mid, e):
                segments.append(Segment(
                    start_x=p0[0], start_y=p0[1], end_x=p1[0], end_y=p1[1],
                    width=to_mm(track.GetWidth()),
                    layer=get_layer_name(track.GetLayer()),
                    net_id=track.GetNetCode(),
                ))
        elif track_class == "PCB_VIA":
            # KiCad 9/10 padstack vias can refuse layerless GetWidth() with
            # 'result with an error set' (seen on vias ADDED in-session by
            # the plugin, then re-synced); GetFrontWidth() is the stable
            # accessor for the outer-annulus size.
            try:
                _vw = track.GetWidth()
            except Exception:
                _vw = track.GetFrontWidth()
            v = Via(
                x=to_mm(track.GetPosition().x),
                y=to_mm(track.GetPosition().y),
                size=to_mm(_vw),
                drill=to_mm(track.GetDrill()),
                layers=[get_layer_name(track.TopLayer()), get_layer_name(track.BottomLayer())],
                net_id=track.GetNetCode(),
            )
            vias.append(v)

    # --- Extract zones ---
    zones = _extract_zones_from_pcbnew(board, to_mm, get_layer_name)

    # --- Extract user-layer guide corridors (issue #7) ---
    # --- Net-tied copper GRAPHICS (#337), builder side ---
    # Parity with the text parser's gr_line/gr_arc pass: PCB_SHAPE lines/arcs
    # on copper layers render as real copper (obstacles + DRC), tagged
    # graphic=True (immutable input art -- never ripped/pruned/stripped).
    try:
        import pcbnew as _pcbnew_g
        for _d in board.GetDrawings():
            if _d.GetClass() not in ("PCB_SHAPE", "DRAWSEGMENT"):
                continue
            _ln = get_layer_name(_d.GetLayer())
            if not (_ln or '').endswith('.Cu'):
                continue
            try:
                _shape = _d.GetShape()
                _w = to_mm(_d.GetWidth())
            except Exception:
                continue
            _nid = _d.GetNetCode() if hasattr(_d, 'GetNetCode') else 0
            _POLY = getattr(_pcbnew_g, 'SHAPE_T_POLY', -10)
            _RECT = getattr(_pcbnew_g, 'SHAPE_T_RECT', -11)
            _CIRC = getattr(_pcbnew_g, 'SHAPE_T_CIRCLE', -12)

            def _emit_outline_b(pts, ew):
                seq = list(pts) + [pts[0]]
                for _a, _b in zip(seq, seq[1:]):
                    segments.append(Segment(
                        start_x=_a[0], start_y=_a[1], end_x=_b[0], end_y=_b[1],
                        width=ew, layer=_ln, net_id=_nid, graphic=True))

            if _shape == getattr(_pcbnew_g, 'SHAPE_T_SEGMENT', 0):
                if _w <= 0:
                    continue
                segments.append(Segment(
                    start_x=to_mm(_d.GetStart().x), start_y=to_mm(_d.GetStart().y),
                    end_x=to_mm(_d.GetEnd().x), end_y=to_mm(_d.GetEnd().y),
                    width=_w, layer=_ln, net_id=_nid, graphic=True))
            elif _shape == getattr(_pcbnew_g, 'SHAPE_T_ARC', 2):
                if _w <= 0:
                    continue
                try:
                    _s0 = (to_mm(_d.GetStart().x), to_mm(_d.GetStart().y))
                    _m0 = (to_mm(_d.GetArcMid().x), to_mm(_d.GetArcMid().y))
                    _e0 = (to_mm(_d.GetEnd().x), to_mm(_d.GetEnd().y))
                except Exception:
                    continue
                for _p0, _p1 in _arc_to_segments(_s0, _m0, _e0):
                    segments.append(Segment(
                        start_x=_p0[0], start_y=_p0[1], end_x=_p1[0], end_y=_p1[1],
                        width=_w, layer=_ln, net_id=_nid, graphic=True))
            elif _shape in (_POLY, _RECT, _CIRC):
                # FILLED copper areas (#337): outline as graphic segments (parity
                # with the text parser). Filled shapes may have 0 stroke width.
                _ow = _w if _w > 0 else defaults.TRACK_WIDTH
                try:
                    if _shape == _POLY:
                        _ps = _d.GetPolyShape()
                        _ol = _ps.Outline(0)
                        _pts = [(to_mm(_ol.CPoint(k).x), to_mm(_ol.CPoint(k).y))
                                for k in range(_ol.PointCount())]
                        if len(_pts) >= 2:
                            _emit_outline_b(_pts, _ow)
                    elif _shape == _RECT:
                        _s0 = _d.GetStart(); _e0 = _d.GetEnd()
                        _x1, _y1 = to_mm(_s0.x), to_mm(_s0.y)
                        _x2, _y2 = to_mm(_e0.x), to_mm(_e0.y)
                        _emit_outline_b([(_x1, _y1), (_x2, _y1),
                                         (_x2, _y2), (_x1, _y2)], _ow)
                    elif _shape == _CIRC:
                        _c = _d.GetCenter(); _cx, _cy = to_mm(_c.x), to_mm(_c.y)
                        _r = to_mm(_d.GetRadius())
                        _emit_outline_b(
                            [(_cx + _r * math.cos(k * math.pi / 8),
                              _cy + _r * math.sin(k * math.pi / 8))
                             for k in range(16)], _ow)
                except Exception:
                    pass
    except Exception:
        pass  # older pcbnew APIs: best-effort

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

    Returns (outers, cutouts): the OUTER boundary rings (largest first --
    several on multi-outline boards, issue #304) and the truly-contained
    interior cutout rings.
    """
    import pcbnew

    edge_cuts_id = getattr(pcbnew, 'Edge_Cuts', None)
    if edge_cuts_id is None:
        return [], []

    def _shape_segments(drawing):
        """Segments for one Edge.Cuts PCB_SHAPE (line/rect/arc/circle/bezier)."""
        segs = []
        shape_type = drawing.GetShape()
                # Line segment
        if shape_type == getattr(pcbnew, 'SHAPE_T_SEGMENT', getattr(pcbnew, 'S_SEGMENT', -1)):
            start = drawing.GetStart()
            end = drawing.GetEnd()
            x1, y1 = to_mm(start.x), to_mm(start.y)
            x2, y2 = to_mm(end.x), to_mm(end.y)
            # Degenerate zero-length lines (zynq_ad9364 has a (0,0)-(0,0)
            # dot and two point-lines on Edge.Cuts) duplicate contour
            # endpoints and break the chain into no closed outline at all.
            if abs(x2 - x1) >= 0.001 or abs(y2 - y1) >= 0.001:
                segs.append(((x1, y1), (x2, y2)))
        elif shape_type == getattr(pcbnew, 'SHAPE_T_RECT', getattr(pcbnew, 'S_RECT', -1)):
            start = drawing.GetStart()
            end = drawing.GetEnd()
            x1, y1 = to_mm(start.x), to_mm(start.y)
            x2, y2 = to_mm(end.x), to_mm(end.y)
            segs.append(((x1, y1), (x2, y1)))
            segs.append(((x2, y1), (x2, y2)))
            segs.append(((x2, y2), (x1, y2)))
            segs.append(((x1, y2), (x1, y1)))
        elif shape_type == getattr(pcbnew, 'SHAPE_T_ARC', getattr(pcbnew, 'S_ARC', -1)):
            start = drawing.GetStart()
            mid = drawing.GetArcMid()
            end = drawing.GetEnd()
            segs.extend(_arc_to_segments(
                (to_mm(start.x), to_mm(start.y)),
                (to_mm(mid.x), to_mm(mid.y)),
                (to_mm(end.x), to_mm(end.y))
            ))
        elif shape_type == getattr(pcbnew, 'SHAPE_T_CIRCLE', getattr(pcbnew, 'S_CIRCLE', -1)):
            c = drawing.GetCenter()
            e = drawing.GetEnd()
            r = math.hypot(to_mm(e.x) - to_mm(c.x), to_mm(e.y) - to_mm(c.y))
            if r > 1e-6:
                segs.extend(_circle_to_segments(to_mm(c.x), to_mm(c.y), r))
        elif shape_type == getattr(pcbnew, 'SHAPE_T_BEZIER', getattr(pcbnew, 'S_CURVE', -1)):
            p0, p3 = drawing.GetStart(), drawing.GetEnd()
            c1, c2 = drawing.GetBezierC1(), drawing.GetBezierC2()
            segs.extend(_bezier_to_segments(
                (to_mm(p0.x), to_mm(p0.y)), (to_mm(c1.x), to_mm(c1.y)),
                (to_mm(c2.x), to_mm(c2.y)), (to_mm(p3.x), to_mm(p3.y))))
        return segs

    segments = []
    for drawing in board.GetDrawings():
        if drawing.GetLayer() != edge_cuts_id:
            continue
        if drawing.GetClass() in ("PCB_SHAPE", "DRAWSEGMENT"):
            try:
                segments.extend(_shape_segments(drawing))
            except Exception:
                continue
    # Footprint-embedded Edge.Cuts (#304): per-LED cutout windows etc. live as
    # fp_ shapes inside footprints; GraphicalItems() returns them in absolute
    # board coordinates, so the same shape handler applies.
    try:
        for fp in board.GetFootprints():
            for g in fp.GraphicalItems():
                if g.GetLayer() != edge_cuts_id or g.GetClass() not in ("PCB_SHAPE", "DRAWSEGMENT"):
                    continue
                try:
                    segments.extend(_shape_segments(g))
                except Exception:
                    continue
    except Exception:
        pass

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

    # Containment-based classification (issue #304) -- same rule as the text
    # parser, so split-keyboard second halves stay OUTER boundaries.
    outers, cutouts = _classify_contours(contours)
    return outers, cutouts


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
            # Keepout / rule areas are routing restrictions, NOT copper. The
            # text parser's zones are filled copper only (a rule area has no
            # (net ...) clause and is skipped there), and every zone consumer
            # (obstacle map, plane connectivity) treats pcb.zones as copper --
            # including rule areas over-blocked the GUI's model and made
            # validate-board report phantom zone-count diffs (comexpress7: 14).
            try:
                if zone.GetIsRuleArea():
                    continue
            except Exception:
                pass
            net_id = zone.GetNetCode()
            net_name = zone.GetNetname()
            # Fill semantics, mirroring the text parser (#350). pcbnew's
            # ISLAND_REMOVAL_MODE enum matches the file token values
            # (0 always / 1 never / 2 area); GetMinIslandArea is IU^2 -> mm^2.
            try:
                priority = int(zone.GetAssignedPriority())
            except Exception:
                try:
                    priority = int(zone.GetPriority())  # pre-KiCad-7 name
                except Exception:
                    priority = 0
            try:
                island_removal_mode = int(zone.GetIslandRemovalMode())
            except Exception:
                island_removal_mode = 0
            try:
                mm_per_iu = float(to_mm(1))
                island_area_min = float(zone.GetMinIslandArea()) * mm_per_iu * mm_per_iu
            except Exception:
                island_area_min = 0.0
            # A zone can span several copper layers; GetLayer() returns
            # UNDEFINED for those. Emit one Zone per copper layer (the text
            # parser does the same for (layers ...) zones).
            try:
                import pcbnew as _pcbnew
                zone_layers = [get_layer_name(l) for l in zone.GetLayerSet().Seq()
                               if _pcbnew.IsCopperLayer(l)]
            except Exception:
                zone_layers = []
            if not zone_layers:
                zone_layers = [get_layer_name(zone.GetLayer())]

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

            for zl in zone_layers:
                zones.append(Zone(
                    net_id=net_id,
                    net_name=net_name,
                    layer=zl,
                    polygon=polygon,
                    priority=priority,
                    island_removal_mode=island_removal_mode,
                    island_area_min=island_area_min
                ))
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
        # Footprint-level clearance override (#326): a divergence here shifts
        # EVERY inheriting pad's resolved local_clearance between the two paths.
        bfc = getattr(bf, 'clearance', 0.0); ffc = getattr(ff, 'clearance', 0.0)
        if not close(bfc, ffc):
            diffs.append(f"Footprint {ref} clearance: board={bfc:.3f} file={ffc:.3f}")
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
            # Sub-micron board/file coordinate differences can straddle the
            # rounding above (e.g. -11.7875 -> -11.787 vs -11.788) and scramble
            # the zip pairing into cascade mismatches (crkbd EXSW). Re-pair by
            # nearest same-number pad within a small tolerance first; anything
            # left over keeps the sorted order.
            unmatched_f = list(f_pads)
            pairs = []
            leftovers_b = []
            for bp in b_pads:
                best = None
                best_d = 0.02  # mm; well above float noise, below any pitch
                for fp2 in unmatched_f:
                    if fp2.pad_number != bp.pad_number:
                        continue
                    d = max(abs(fp2.global_x - bp.global_x), abs(fp2.global_y - bp.global_y))
                    if d < best_d:
                        best, best_d = fp2, d
                if best is not None:
                    unmatched_f.remove(best)
                    pairs.append((bp, best))
                else:
                    leftovers_b.append(bp)
            pairs.extend(zip(leftovers_b, unmatched_f))
            for bp, fp in pairs:
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

    def _net_label(pcb, nid):
        # Compare by NAME, not raw id: for nets carried only by copper
        # GRAPHICS (#337, no pads/tracks anchor them), pcbnew's renumbered
        # code and the file's net-table id legitimately differ (openstint
        # /A-: board 3 vs file 14) while naming the same net.
        n = pcb.nets.get(nid)
        return n.name if n is not None else nid

    def _seg_sig_for(pcb):
        def _seg_sig(s):
            ends = tuple(sorted([(_q(s.start_x), _q(s.start_y)), (_q(s.end_x), _q(s.end_y))]))
            if getattr(s, 'graphic', False):
                # KiCad RECOMPUTES a copper graphic's net from connectivity on
                # load (openstint: file attribute /A-, pcbnew says GND for the
                # same art). Same copper either way -- compare geometry only.
                return (ends, _q(s.width), s.layer, '<graphic>')
            return (ends, _q(s.width), s.layer, _net_label(pcb, s.net_id))
        return _seg_sig

    def _multiset_diff_2sig(board_items, file_items, sig_b, sig_f, label, fmt):
        from collections import Counter
        cb = Counter(sig_b(x) for x in board_items)
        cf = Counter(sig_f(x) for x in file_items)
        only_board = list((cb - cf).elements())
        only_file = list((cf - cb).elements())
        if only_board or only_file:
            diffs.append(f"{label} count: board={len(board_items)} file={len(file_items)}; "
                         f"{len(only_board)} only in board, {len(only_file)} only in file")
            for s in only_board[:5]:
                diffs.append(f"  {label} only in board: {fmt(s)}")
            for s in only_file[:5]:
                diffs.append(f"  {label} only in file: {fmt(s)}")

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

    _multiset_diff_2sig(from_board.segments, from_file.segments,
                        _seg_sig_for(from_board), _seg_sig_for(from_file), "Segment",
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
            # Fill semantics (#350): both fronts must agree or the priority-
            # exact fill-point rejection / island-patch gating diverge CLI vs GUI.
            if getattr(bz, 'priority', 0) != getattr(fz, 'priority', 0):
                diffs.append(f"Zone net={bz.net_id} layer={bz.layer} priority: board={bz.priority} file={fz.priority}")
            if getattr(bz, 'island_removal_mode', 0) != getattr(fz, 'island_removal_mode', 0):
                diffs.append(f"Zone net={bz.net_id} layer={bz.layer} island_removal_mode: board={bz.island_removal_mode} file={fz.island_removal_mode}")
            # island_area_min is only meaningful in mode 2: pcbnew's
            # GetMinIslandArea() reports the C++ default (10mm^2) even when
            # the file carries no token (mode 0/1 zones), so comparing it
            # unconditionally manufactured phantom parity diffs (free_dap).
            if (getattr(bz, 'island_removal_mode', 0) == 2
                    and abs(getattr(bz, 'island_area_min', 0.0) - getattr(fz, 'island_area_min', 0.0)) > 0.01):
                diffs.append(f"Zone net={bz.net_id} layer={bz.layer} island_area_min: board={bz.island_area_min} file={fz.island_area_min}")

    # --- Compare board outline / cutouts (used for edge & cutout obstacles) ---
    bo_b = bi_b.board_outline or []
    bo_f = bi_f.board_outline or []
    if len(bo_b) != len(bo_f):
        diffs.append(f"Board outline vertex count: board={len(bo_b)} file={len(bo_f)}")
    cut_b = bi_b.board_cutouts or []
    cut_f = bi_f.board_cutouts or []
    if len(cut_b) != len(cut_f):
        diffs.append(f"Board cutout count: board={len(cut_b)} file={len(cut_f)}")
    # Multi-outline boards (#304): both sides must agree on the OUTER ring set
    outs_b = sorted(len(o) for o in (getattr(bi_b, 'board_outlines', None) or []))
    outs_f = sorted(len(o) for o in (getattr(bi_f, 'board_outlines', None) or []))
    if outs_b != outs_f:
        diffs.append(f"Board outer-ring set: board={outs_b} file={outs_f}")

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

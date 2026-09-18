"""Solder-paste aperture geometry (#962).

Nothing in this toolchain used to model the paste stencil. The router could lay
a via inside an aperture, and no repo checker could see it. Measured on
esp_prog U2, a SOT-89:
- the land tab is a net-0 F.Cu `fp_poly`;
- the stencil opening is a separate F.Paste `fp_poly`;
- the declared pad 2 is on F.Cu ONLY.

So the only object that describes where solder goes is a GRAPHIC. Eleven
routing arms out of eleven put a same-net via inside it, and so did
`--same-net-pad-clearance 0.4`, because that flag measured the declared pad
rectangle.

This module answers three questions, for BOTH parse paths (the text parser and
`build_pcb_data_from_board` feed it the same inputs, so parity holds by
construction):

1. **Where are the apertures?** `build_paste_apertures`:
   - every pad on a paste layer, inflated by its resolved paste margin
     (`resolve_paste_margin`: pad, then footprint, then board, exactly
     KiCad's `PAD::GetSolderPasteMargin` precedence);
   - every footprint/board paste graphic.
2. **How far is a point from one?** `aperture_distance` (0 inside).
3. **Which nets does one belong to?** `aperture_nets` / `apertures_for_net`.
   A pad aperture belongs to its pad's net. A graphic belongs to the nets of
   the owner's copper pads it overlaps, plus the nets the #908 own-pad lift
   frees on the owner's overlapping graphic copper. For U2 that is
   `Net-(C1-Pad1)` alone; GND's pad 1 has its own opening.

Leaf module. `check_drc` and `kicad_parser` are imported INSIDE the functions
that need them, because both import this one.
"""
from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass, field
from typing import Dict, FrozenSet, List, Optional, Sequence, Tuple

PASTE_LAYERS = ('F.Paste', 'B.Paste')
PASTE_TO_COPPER = {'F.Paste': 'F.Cu', 'B.Paste': 'B.Cu'}

#: KiCad clamps a pad's paste margin so the opening never inverts:
#: `margin.x >= -size.x / 2` (PAD::GetSolderPasteMargin). Custom pads are exempt
#: there; they are here too, because their polygons are not shrunk at all
#: (see `pad_aperture`).
_EPS = 1e-9


@dataclass
class PasteAperture:
    """One solder-paste opening, in GLOBAL board millimetres.

    `source` says where the opening comes from:
    - ``'pad'``: a copper pad on a paste layer. It is inflated by `margin`, and
      `shape_pad` is that inflated pad, so distances use the exact pad geometry
      (`check_drc.point_to_pad_distance`) instead of a polygonised copy.
    - ``'paste_only_pad'``: a pad with no copper, only a paste layer (a QFN
      windowpane). KiCad applies NO margin to these.
    - ``'graphic'``: an `fp_*` / `gr_*` shape drawn on F.Paste/B.Paste.
      `rings` / `circle` / `closed` / `filled` / `width` describe it.
    """
    owner_ref: str
    layer: str
    source: str
    pad_number: str = ""
    net_id: int = 0
    rings: List[List[Tuple[float, float]]] = field(default_factory=list)
    closed: bool = True
    filled: bool = True
    width: float = 0.0
    circle: Optional[Tuple[float, float, float]] = None
    uuid: str = ""
    margin: Tuple[float, float] = (0.0, 0.0)
    shape_pad: object = None
    approximations: Tuple[str, ...] = ()
    bounds: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)

    def label(self) -> str:
        """How a report names this opening: `U2 F.Paste (graphic)`,
        `C1.1 F.Paste (pad)`."""
        who = self.owner_ref or 'board'
        if self.pad_number and self.source != 'graphic':
            who = '%s.%s' % (who, self.pad_number)
        return '%s %s (%s)' % (who, self.layer, self.source)


# -- pad membership -----------------------------------------------------------

def pad_paste_layers(pad) -> List[str]:
    """The paste layers a pad opens, `*.Paste` expanded."""
    out = []
    for ln in (getattr(pad, 'layers', None) or []):
        for one in (PASTE_LAYERS if ln == '*.Paste' else (ln,)):
            if one in PASTE_LAYERS and one not in out:
                out.append(one)
    return out


def pad_has_copper(pad) -> bool:
    """Does the pad carry copper? NPTH pads never do (CLAUDE.md), even when
    their layer list names `*.Cu`."""
    if getattr(pad, 'pad_type', '') == 'np_thru_hole':
        return False
    return any(ln.endswith('.Cu') for ln in (getattr(pad, 'layers', None) or []))


def pad_on_copper_side(pad, copper_layer: str) -> bool:
    """Is the pad's copper on `copper_layer`? Through-hole copper reaches every
    layer; `*.Cu` / `F&B.Cu` spell that too."""
    if not pad_has_copper(pad):
        return False
    lys = getattr(pad, 'layers', None) or []
    if copper_layer in lys or '*.Cu' in lys or 'F&B.Cu' in lys:
        return True
    return getattr(pad, 'drill', 0.0) > 0 and getattr(pad, 'pad_type', '') == 'thru_hole'


# -- margin -------------------------------------------------------------------

def resolve_paste_margin(pad, fp, board_info) -> Tuple[float, float]:
    """Per-axis paste margin (mm) of `pad`, KiCad precedence.

    `PAD::GetSolderPasteMargin`:
    - A pad with no copper layer gets (0, 0): its own shape IS the opening.
    - margin = pad override, else footprint override, else board
      `pad_to_paste_clearance`. The ratio is resolved independently, in the
      same order.
    - Per axis: `margin + size * ratio`.
    - Clamped at `-size / 2` for every shape except custom.

    Board-space `size_x`/`size_y` are what we have. That is the right frame:
    the margin is per axis of the pad's own frame, and the parser already swaps
    `size_x`/`size_y` for a ~90-degree pad, so the pairing survives.
    """
    if not pad_has_copper(pad):
        return 0.0, 0.0

    def pick(attr, board_attr):
        v = getattr(pad, attr, None)
        if v is None and fp is not None:
            v = getattr(fp, attr, None)
        if v is None:
            v = getattr(board_info, board_attr, None) if board_info is not None else None
        return float(v or 0.0)

    m = pick('paste_margin', 'pad_to_paste_clearance')
    r = pick('paste_margin_ratio', 'pad_to_paste_clearance_ratio')
    mx = m + pad.size_x * r
    my = m + pad.size_y * r
    if getattr(pad, 'shape', '') != 'custom':
        mx = max(mx, -pad.size_x / 2.0)
        my = max(my, -pad.size_y / 2.0)
    return mx, my


def _inflated_pad(pad, mx: float, my: float):
    """A copy of `pad` whose copper IS the paste opening.

    - rect / roundrect / oval / circle: the size grows by 2·margin per axis. A
      roundrect's corner radius grows by the smaller margin (Minkowski sum of
      a rounded rect); a stadium stays a stadium, because
      `point_to_pad_distance` rounds ovals by min(size)/2.
    - circle: KiCad sizes it from size.x.
    - custom: the polygons are grown by a positive margin. A NEGATIVE margin is
      left un-shrunk, which is a superset: a via can only be flagged more,
      never less. The caller discloses it.
    """
    sx = pad.size_x + 2.0 * mx
    sy = pad.size_y + 2.0 * my
    if getattr(pad, 'shape', '') == 'circle':
        sy = sx
    changes = {'size_x': sx, 'size_y': sy}
    if getattr(pad, 'shape', '') == 'roundrect':
        r0 = pad.roundrect_rratio * min(pad.size_x, pad.size_y)
        r1 = max(0.0, r0 + min(mx, my))
        changes['roundrect_rratio'] = (r1 / min(sx, sy)) if min(sx, sy) > 0 else 0.0
    polys = getattr(pad, 'polygons', None)
    if polys and mx > 0:
        from kicad_parser import _offset_polygon_outward
        changes['polygons'] = [_offset_polygon_outward(list(p), mx) for p in polys]
    return dataclasses.replace(pad, **changes)


# -- construction ---------------------------------------------------------------

def _ring_bounds(rings, circle, width):
    hw = width / 2.0
    if circle is not None:
        cx, cy, r = circle
        return (cx - r - hw, cy - r - hw, cx + r + hw, cy + r + hw)
    xs = [p[0] for ring in rings for p in ring]
    ys = [p[1] for ring in rings for p in ring]
    if not xs:
        return (0.0, 0.0, 0.0, 0.0)
    return (min(xs) - hw, min(ys) - hw, max(xs) + hw, max(ys) + hw)


def _pad_bounds(shape_pad):
    """Axis-aligned bounds of a pad's copper, computed analytically.

    Exact for an axis-aligned pad. For a rotated rounded pad it is the rotated
    RECT's bounds, a slight superset. The bounds are only a pre-filter and a
    comparison key, and both parse paths share this function, so a superset
    costs nothing. Sampling the perimeter instead cost 90 ms a parse on glasgow.
    """
    polys = getattr(shape_pad, 'polygons', None)
    if polys:
        xs = [p[0] for poly in polys for p in poly]
        ys = [p[1] for poly in polys for p in poly]
        if xs:
            return (min(xs), min(ys), max(xs), max(ys))
    hx, hy = shape_pad.size_x / 2.0, shape_pad.size_y / 2.0
    th = math.radians(getattr(shape_pad, 'rect_rotation', 0.0) or 0.0)
    c, s = abs(math.cos(th)), abs(math.sin(th))
    ex, ey = hx * c + hy * s, hx * s + hy * c
    return (shape_pad.global_x - ex, shape_pad.global_y - ey,
            shape_pad.global_x + ex, shape_pad.global_y + ey)


def pad_aperture(pad, fp, board_info, layer: str) -> Optional[PasteAperture]:
    """The opening `pad` makes on `layer`, or None when the margin closes it."""
    copper = pad_has_copper(pad)
    mx, my = resolve_paste_margin(pad, fp, board_info)
    if pad.size_x + 2 * mx <= _EPS or pad.size_y + 2 * my <= _EPS:
        return None
    shape_pad = _inflated_pad(pad, mx, my) if (mx or my) else pad
    approx = tuple(getattr(pad, 'geometry_approximations', ()) or ())
    if getattr(pad, 'polygons', None) and mx < 0:
        approx = approx + ('custom pad paste reduction not applied (superset)',)
    return PasteAperture(
        owner_ref=getattr(pad, 'component_ref', '') or '',
        layer=layer,
        source='pad' if copper else 'paste_only_pad',
        pad_number=str(getattr(pad, 'pad_number', '') or ''),
        net_id=int(getattr(pad, 'net_id', 0) or 0) if copper else 0,
        margin=(mx, my),
        shape_pad=shape_pad,
        approximations=approx,
        bounds=_pad_bounds(shape_pad),
    )


def graphic_aperture(g: dict) -> Optional[PasteAperture]:
    """A paste GRAPHIC, as both parse paths describe it:

    `{owner_ref, layer, kind: poly|rect|circle|line|arc, points, center,
    radius, width, filled, uuid}`, with points in GLOBAL mm.

    Polys and rects are closed. Lines and arcs are open strokes.
    """
    kind = g.get('kind')
    w = float(g.get('width') or 0.0)
    filled = bool(g.get('filled'))
    common = dict(owner_ref=g.get('owner_ref', '') or '', layer=g['layer'],
                  source='graphic', width=w, uuid=g.get('uuid', '') or '')
    if kind == 'circle':
        c = g.get('center')
        r = float(g.get('radius') or 0.0)
        if c is None or (r <= 0 and w <= 0):
            return None
        circle = (float(c[0]), float(c[1]), r)
        return PasteAperture(circle=circle, closed=True, filled=filled,
                             bounds=_ring_bounds([], circle, w), **common)
    pts = [(float(x), float(y)) for x, y in (g.get('points') or [])]
    # A closed poly often REPEATS its first vertex; drop the duplicate, as the
    # copper emitter does.
    if len(pts) > 2 and pts[0] == pts[-1]:
        pts = pts[:-1]
    if len(pts) < 2:
        return None
    closed = kind in ('poly', 'rect')
    if not closed and w <= 0:
        return None         # an unstroked line opens nothing
    if closed and not filled and w <= 0:
        return None
    return PasteAperture(rings=[pts], closed=closed, filled=filled and closed,
                         bounds=_ring_bounds([pts], None, w), **common)


def _aperture_sort_key(ap: PasteAperture):
    b = ap.bounds
    return (ap.owner_ref, ap.layer, ap.source, ap.pad_number, ap.uuid,
            round(b[0], 4), round(b[1], 4), round(b[2], 4), round(b[3], 4))


def build_paste_apertures(footprints: Dict[str, object], board_info,
                          graphics: Sequence[dict]) -> List[PasteAperture]:
    """Every paste opening on the board: pad openings plus paste graphics.

    Sorted, so both parse paths publish the same list in the same order.
    """
    out: List[PasteAperture] = []
    for fp in (footprints or {}).values():
        for pad in getattr(fp, 'pads', None) or []:
            for layer in pad_paste_layers(pad):
                ap = pad_aperture(pad, fp, board_info, layer)
                if ap is not None:
                    out.append(ap)
    for g in graphics or []:
        ap = graphic_aperture(g)
        if ap is not None:
            out.append(ap)
    out.sort(key=_aperture_sort_key)
    return out


# -- distance -------------------------------------------------------------------

def _seg_dist(px, py, a, b):
    from geometry_utils import point_to_segment_distance
    return point_to_segment_distance(px, py, a[0], a[1], b[0], b[1])


def _point_in_ring(x, y, ring) -> bool:
    inside = False
    n = len(ring)
    j = n - 1
    for i in range(n):
        xi, yi = ring[i]
        xj, yj = ring[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def _outline_distance(x, y, ring, closed) -> float:
    n = len(ring)
    edges = n if closed else n - 1
    best = float('inf')
    for i in range(edges):
        d = _seg_dist(x, y, ring[i], ring[(i + 1) % n])
        if d < best:
            best = d
    return best


def aperture_distance(x: float, y: float, ap: PasteAperture) -> float:
    """Distance (mm) from a point to the paste opening; 0 inside it."""
    if ap.shape_pad is not None:
        from check_drc import point_to_pad_distance
        return max(0.0, point_to_pad_distance(x, y, ap.shape_pad))
    hw = ap.width / 2.0
    if ap.circle is not None:
        cx, cy, r = ap.circle
        d = math.hypot(x - cx, y - cy)
        edge = max(0.0, d - r) if ap.filled else abs(d - r)
        return max(0.0, edge - hw)
    best = float('inf')
    for ring in ap.rings:
        if ap.filled and ap.closed and _point_in_ring(x, y, ring):
            return 0.0
        d = _outline_distance(x, y, ring, ap.closed)
        if d < best:
            best = d
    return max(0.0, best - hw)


def via_paste_penetration(x: float, y: float, via_size: float,
                          ap: PasteAperture) -> float:
    """How far a via barrel of diameter `via_size` reaches INTO the opening.

    Positive means they overlap. This is the same barrel rule `fab_notes`
    uses for via-in-pad (#695): an off-centre barrel wicks solder just the
    same.
    """
    return via_size / 2.0 - aperture_distance(x, y, ap)


# -- association ---------------------------------------------------------------

def _bounds_overlap(a, b, grow=0.0) -> bool:
    return not (a[2] + grow < b[0] or b[2] + grow < a[0]
                or a[3] + grow < b[1] or b[3] + grow < a[1])


def _pad_overlaps(ap: PasteAperture, pad) -> bool:
    """Does the pad's COPPER overlap the opening?

    Tested three ways: the pad centre, the pad's perimeter samples, and the
    opening's own vertices against the pad.
    """
    from check_drc import _pad_perimeter_points, point_to_pad_distance
    if aperture_distance(pad.global_x, pad.global_y, ap) <= 1e-9:
        return True
    for (px, py) in _pad_perimeter_points(pad, 8):
        if aperture_distance(px, py, ap) <= 1e-9:
            return True
    for ring in ap.rings:
        for (vx, vy) in ring:
            if point_to_pad_distance(vx, vy, pad) <= 1e-9:
                return True
    if ap.circle is not None:
        cx, cy, _r = ap.circle
        if point_to_pad_distance(cx, cy, pad) <= 1e-9:
            return True
    return False


def _pad_copper_bounds(pad):
    return _pad_bounds(pad)


def aperture_nets(pcb_data, ap: PasteAperture) -> FrozenSet[int]:
    """The nets whose vias this opening concerns.

    - pad opening: the pad's own net.
    - graphic / paste-only-pad opening, owned by footprint F: the nets of F's
      copper pads on the same side that the opening overlaps, SMD or
      through-hole (pin-in-paste), plus the nets that
      `check_drc.graphic_own_pad_nets` frees on F's graphic copper where it
      overlaps the opening (the #908 own-pad lift). That second set is how
      esp_prog U2's tab reaches `Net-(C1-Pad1)`.
    - board-level graphic (no owner): any footprint's pads it overlaps.

    Deliberately NOT every net of the footprint: at
    `--same-net-pad-clearance 0.4` that would wall every fine-pitch part off
    from its own vias.
    """
    if ap.source == 'pad':
        return frozenset({ap.net_id}) if ap.net_id else frozenset()
    copper = PASTE_TO_COPPER.get(ap.layer)
    fps = getattr(pcb_data, 'footprints', None) or {}
    owner = fps.get(ap.owner_ref) if ap.owner_ref else None
    candidates = owner.pads if owner is not None else [
        p for f in fps.values() for p in (getattr(f, 'pads', None) or [])]
    nets = set()
    for pad in candidates:
        if not pad.net_id or not pad_on_copper_side(pad, copper):
            continue
        if not _bounds_overlap(ap.bounds, _pad_copper_bounds(pad)):
            continue
        if _pad_overlaps(ap, pad):
            nets.add(pad.net_id)
    if owner is not None:
        from check_drc import graphic_own_pad_nets
        lifted = _own_pad_lift_cache(pcb_data, graphic_own_pad_nets)
        for seg in getattr(pcb_data, 'segments', None) or []:
            if (not getattr(seg, 'graphic', False)
                    or getattr(seg, 'owner_ref', '') != ap.owner_ref
                    or seg.layer != copper):
                continue
            got = lifted.get(id(seg))
            if not got:
                continue
            hw = seg.width / 2.0
            if (aperture_distance(seg.start_x, seg.start_y, ap) <= hw
                    or aperture_distance(seg.end_x, seg.end_y, ap) <= hw):
                nets |= set(got)
    return frozenset(nets)


def _own_pad_lift_cache(pcb_data, fn):
    key = (id(getattr(pcb_data, 'segments', None)),
           len(getattr(pcb_data, 'segments', None) or []))
    hit = getattr(pcb_data, '_paste_own_pad_lift', None)
    if hit is not None and hit[0] == key:
        return hit[1]
    got = fn(pcb_data)
    try:
        pcb_data._paste_own_pad_lift = (key, got)
    except Exception:
        pass
    return got


def _index_key(pcb_data):
    aps = getattr(pcb_data, 'paste_apertures', None) or []
    segs = getattr(pcb_data, 'segments', None) or []
    return (id(aps), len(aps), id(segs), len(segs))


def apertures_by_net(pcb_data) -> Dict[int, List[PasteAperture]]:
    """{net_id: [the openings that concern that net's vias]}, memoised on
    `pcb_data` (the key covers the aperture and segment lists)."""
    key = _index_key(pcb_data)
    hit = getattr(pcb_data, '_paste_net_index', None)
    if hit is not None and hit[0] == key:
        return hit[1]
    index: Dict[int, List[PasteAperture]] = {}
    for ap in getattr(pcb_data, 'paste_apertures', None) or []:
        for nid in aperture_nets(pcb_data, ap):
            index.setdefault(nid, []).append(ap)
    try:
        pcb_data._paste_net_index = (key, index)
    except Exception:
        pass
    return index


def apertures_for_net(pcb_data, net_id: int) -> List[PasteAperture]:
    """The openings a via of `net_id` must keep out of (or be protected in)."""
    return apertures_by_net(pcb_data).get(net_id, [])

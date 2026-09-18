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
    - An explicit 0 counts as UNSET. The KiCad 10 loader returns None for
      `(solder_paste_margin 0)`, and the parsers mirror that, storing None.
    - Per axis: `margin + size * ratio`, clamped at `-size / 2` for every
      shape except custom.

    The size frame depends on the pad:
    - Ordinary pads use board-space `size_x`/`size_y`. The parser swaps those
      for a ~90-degree pad, so the pairing with KiCad's pad-frame axes
      survives.
    - Custom pads use the ANCHOR `(size ...)` in the PAD frame
      (`Pad.anchor_size`), because `size_x`/`size_y` there are the primitive
      extent. KiCad sizes the ratio term from the anchor (measured, #962 phase-1
      verification: anchor 0.5, primitive 2x1, ratio -0.1 -> (-0.10, -0.10)).
      So for a custom pad the returned tuple is in the pad frame, as KiCad
      returns it.
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
    if getattr(pad, 'shape', '') == 'custom':
        ax, ay = getattr(pad, 'anchor_size', None) or (pad.size_x, pad.size_y)
        return m + ax * r, m + ay * r
    mx = m + pad.size_x * r
    my = m + pad.size_y * r
    mx = max(mx, -pad.size_x / 2.0)
    my = max(my, -pad.size_y / 2.0)
    return mx, my


def _inflated_pad(pad, mx: float, my: float):
    """A COPY of `pad` whose copper IS the paste opening.

    Always a copy, even at zero margin: the opening is a snapshot taken at
    parse time and must not move if a later pass moves the live pad.

    - rect / roundrect / oval / circle: the size grows by 2*margin per axis. A
      roundrect's corner radius grows by the smaller margin (the Minkowski sum
      of a rounded rect). A stadium stays a stadium, because
      `point_to_pad_distance` rounds ovals by min(size)/2. KiCad sizes a
      circle from size.x.
    - custom: KiCad inflates the primitive outline uniformly by the PAD-frame x
      margin. A positive margin grows the polygons. A negative one leaves them
      un-shrunk, which is a superset (a via can only be flagged more, never
      less), and the caller discloses it.
    """
    if getattr(pad, 'shape', '') == 'custom':
        polys = getattr(pad, 'polygons', None)
        changes = {}
        if polys and mx > 0:
            from kicad_parser import _offset_polygon_outward
            changes['polygons'] = [_offset_polygon_outward(list(p), mx) for p in polys]
        elif not polys and mx > 0:
            changes = {'size_x': pad.size_x + 2.0 * mx, 'size_y': pad.size_y + 2.0 * mx}
        return dataclasses.replace(pad, **changes)
    sx = pad.size_x + 2.0 * mx
    sy = pad.size_y + 2.0 * my
    if getattr(pad, 'shape', '') == 'circle':
        sy = sx
    changes = {'size_x': sx, 'size_y': sy}
    if getattr(pad, 'shape', '') == 'roundrect':
        r0 = pad.roundrect_rratio * min(pad.size_x, pad.size_y)
        r1 = max(0.0, r0 + min(mx, my))
        changes['roundrect_rratio'] = (r1 / min(sx, sy)) if min(sx, sy) > 0 else 0.0
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
    custom = getattr(pad, 'shape', '') == 'custom'
    mx, my = resolve_paste_margin(pad, fp, board_info)
    # KiCad does not clamp a custom pad, and a negative margin on one is not
    # applied here (superset, disclosed). Only an ordinary pad can close.
    if not custom and (pad.size_x + 2 * mx <= _EPS or pad.size_y + 2 * my <= _EPS):
        return None
    shape_pad = _inflated_pad(pad, mx, my)
    approx = tuple(getattr(pad, 'geometry_approximations', ()) or ())
    if custom and mx < 0:
        approx = approx + ('custom pad paste reduction not applied (superset)',)
    return PasteAperture(
        owner_ref=getattr(pad, 'component_ref', '') or '',
        layer=layer,
        source='pad' if copper else 'paste_only_pad',
        # pcbnew blanks the number of a copper-less (aperture) pad on load, so
        # the opening carries none on either path (#962 phase-1 verification).
        pad_number=str(getattr(pad, 'pad_number', '') or '') if copper else '',
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


def _boundary_samples(ap: PasteAperture, step: float = 0.05):
    """Points along an opening's boundary, spaced <= `step` mm, plus its centre.

    Dense rather than vertex-only. A strip or a stroked line that crosses a pad
    has no vertex inside the pad and no pad vertex inside it (#962 phase-1
    verification, S6), so a vertex test would call them disjoint.
    """
    from check_drc import _pad_perimeter_points
    b = ap.bounds
    pts = [((b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0)]
    if ap.shape_pad is not None:
        sp = ap.shape_pad
        n = max(8, int(max(sp.size_x, sp.size_y) / step) + 1)
        pts += _pad_perimeter_points(sp, n)
        pts.append((sp.global_x, sp.global_y))
        return pts
    if ap.circle is not None:
        cx, cy, r = ap.circle
        n = max(16, int(2 * math.pi * r / step) + 1)
        pts += [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n))
                for k in range(n)]
        pts.append((cx, cy))
        return pts
    for ring in ap.rings:
        n = len(ring)
        edges = n if ap.closed else n - 1
        for i in range(edges):
            (x1, y1), (x2, y2) = ring[i], ring[(i + 1) % n]
            k = max(1, int(math.hypot(x2 - x1, y2 - y1) / step) + 1)
            pts += [(x1 + (x2 - x1) * t / k, y1 + (y2 - y1) * t / k) for t in range(k)]
        if not ap.closed and ring:
            pts.append(ring[-1])
    return pts


def _pad_overlaps(ap: PasteAperture, pad) -> bool:
    """Does the pad's COPPER overlap the opening?

    Both ways round, densely:
    - the pad's centre and perimeter samples inside the opening;
    - the opening's centre and boundary samples on the pad's copper.

    The second direction is the one a QFN windowpane needs: the pane sits
    entirely inside the exposed pad, and the EP's own perimeter and centre can
    all miss it. A copper-to-opening distance within half a stroke counts as
    overlap for a stroked shape.
    """
    from check_drc import _pad_perimeter_points, point_to_pad_distance
    if aperture_distance(pad.global_x, pad.global_y, ap) <= 1e-9:
        return True
    n = max(8, int(max(pad.size_x, pad.size_y) / 0.05) + 1)
    for (px, py) in _pad_perimeter_points(pad, n):
        if aperture_distance(px, py, ap) <= 1e-9:
            return True
    reach = ap.width / 2.0 if ap.shape_pad is None else 0.0
    for (vx, vy) in _boundary_samples(ap):
        if point_to_pad_distance(vx, vy, pad) <= reach + 1e-9:
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
        samples = None
        for ring, closed, segs in _owner_copper_shapes(pcb_data, ap.owner_ref, copper):
            got = set()
            for sg in segs:
                got |= set(lifted.get(id(sg), ()))
            if not got or got <= nets:
                continue
            if samples is None:
                samples = _boundary_samples(ap)
            if _shape_meets_opening(ring, closed, segs, ap, samples):
                nets |= got
    return frozenset(nets)


def _owner_copper_shapes(pcb_data, owner_ref: str, copper_layer: str):
    """The owner's graphic copper on `copper_layer`, as `(ring, closed, segs)`.

    The regrouping into shapes is check_drc's `graphic_copper_shapes`, the
    same grouping the off-outline census uses, so the two cannot disagree
    about where one shape ends. A shape counts as closed when it is a poly,
    rect or circle.
    """
    return _shapes_by_owner(pcb_data).get((owner_ref, copper_layer), [])


def _shape_meets_opening(ring, closed, segs, ap, samples) -> bool:
    """Does a copper shape (area when closed, stroke otherwise) meet the opening?"""
    for (x, y) in samples:
        if closed and len(ring) >= 3 and _point_in_ring(x, y, ring):
            return True
        for sg in segs:
            if _seg_dist(x, y, (sg.start_x, sg.start_y), (sg.end_x, sg.end_y)) <= sg.width / 2.0:
                return True
    for sg in segs:
        if aperture_distance(sg.start_x, sg.start_y, ap) <= sg.width / 2.0:
            return True
    return False


#: Stand-ins for a missing or empty container. Module constants, so a memo keyed
#: on container IDENTITY hits on a board with no segments. A fresh `[]` per call
#: missed on every lookup (#962 phase-1 verification, round 2: 70 ms a call on
#: orangecrab with `segments=[]`). Never mutated.
_EMPTY_LIST: list = []
_EMPTY_DICT: dict = {}


def _graphic_sig(segs):
    """Ids of the GRAPHIC segments: the only segments the association reads."""
    return tuple(id(sg) for sg in segs if getattr(sg, 'graphic', False))


def _memo_sig(objs):
    """Cheap signature of the containers a memo depends on.

    Each container is HELD by the memo entry, so its id cannot be reused while
    the entry lives (#977's `id(map)` bug class, #962 phase-1 verification S4).
    The aperture list also carries its element ids, which catches an in-place
    replacement at unchanged length; it is ~1000 entries, so that is cheap.
    The segment list is keyed on its LENGTH here. When the length moves (a
    routed track was added), `_memo_get` checks the graphic segments
    themselves before rebuilding. Routing adds tracks all the time and never
    touches graphic copper, and a rebuild costs ~0.1 s.
    """
    aps, segs, fps = objs
    return (len(aps), tuple(map(id, aps)), len(segs), len(fps))


def _memo_get(pcb_data, attr, objs):
    hit = getattr(pcb_data, attr, None)
    if (hit is None or len(hit[0]) != len(objs)
            or not all(a is b for a, b in zip(hit[0], objs))):
        return False, None
    sig = _memo_sig(objs)
    if hit[1] == sig:
        return True, hit[2]
    # Only the segment COUNT moved: if the graphic copper is the same objects,
    # the answer still holds. Refresh the cheap key and reuse it.
    if hit[1][:2] == sig[:2] and hit[1][3] == sig[3] and hit[3] == _graphic_sig(objs[1]):
        try:
            setattr(pcb_data, attr, (hit[0], sig, hit[2], hit[3]))
        except Exception:
            pass
        return True, hit[2]
    return False, None


def _memo_put(pcb_data, attr, objs, value):
    try:
        setattr(pcb_data, attr, (tuple(objs), _memo_sig(objs), value,
                                 _graphic_sig(objs[1])))
    except Exception:
        pass


def _memo_objs(pcb_data):
    return (getattr(pcb_data, 'paste_apertures', None) or _EMPTY_LIST,
            getattr(pcb_data, 'segments', None) or _EMPTY_LIST,
            getattr(pcb_data, 'footprints', None) or _EMPTY_DICT)


def _own_pad_lift_cache(pcb_data, fn):
    objs = _memo_objs(pcb_data)
    ok, val = _memo_get(pcb_data, '_paste_own_pad_lift', objs)
    if ok:
        return val
    got = fn(pcb_data)
    _memo_put(pcb_data, '_paste_own_pad_lift', objs, got)
    return got


def _shapes_by_owner(pcb_data):
    """{(owner, copper layer): [(ring, closed, segs)]}, memoised like the index."""
    objs = _memo_objs(pcb_data)
    ok, val = _memo_get(pcb_data, '_paste_shape_index', objs)
    if ok:
        return val
    from check_drc import graphic_copper_shapes
    idx: Dict[Tuple[str, str], list] = {}
    for sh in graphic_copper_shapes(pcb_data):
        segs = sh['segs']
        idx.setdefault((sh['owner'], sh['layer']), []).append(
            ([(sg.start_x, sg.start_y) for sg in segs],
             sh['kind'] in ('poly', 'rect', 'circle'), segs))
    _memo_put(pcb_data, '_paste_shape_index', objs, idx)
    return idx


def apertures_by_net(pcb_data) -> Dict[int, List[PasteAperture]]:
    """{net_id: [the openings that concern that net's vias]}, memoised on
    `pcb_data` and revalidated against the containers it depends on (see
    `_memo_sig` / `_memo_get`)."""
    objs = _memo_objs(pcb_data)
    ok, val = _memo_get(pcb_data, '_paste_net_index', objs)
    if ok:
        return val
    index: Dict[int, List[PasteAperture]] = {}
    for ap in objs[0]:
        for nid in aperture_nets(pcb_data, ap):
            index.setdefault(nid, []).append(ap)
    _memo_put(pcb_data, '_paste_net_index', objs, index)
    return index


def apertures_for_net(pcb_data, net_id: int) -> List[PasteAperture]:
    """The openings a via of `net_id` must keep out of (or be protected in)."""
    return apertures_by_net(pcb_data).get(net_id, [])

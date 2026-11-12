"""Solder-mask artwork as copper keep-outs.

A graphic drawn on F.Mask or B.Mask is a mask OPENING: whatever copper lies
under it ships bare. A logo, a label or an exposed-copper window drawn there
must therefore not get a track or a via of that side's copper routed through
it -- the router had no notion of the mask layers and routed straight across.

`extract_mask_artwork` turns every such graphic into a keep-out dict of the
same shape `kicad_parser.extract_keepouts` returns (tracks and vias not
allowed, on F.Cu for F.Mask and B.Cu for B.Mask), so the one native keep-out
rasteriser (`obstacle_map.add_rule_area_keepout_obstacles`) stamps it. They
live in their own list, `board_info.mask_artwork`, NOT in
`board_info.keepouts` (which stays the board's KiCad rule areas, as DRC,
the census and the rule-area audit read it). `routing_keepouts` joins the
two for the consumers that keep NEW copper out: the stamper and the
octolinear smoother's shortcut check.

Covered: gr_/fp_ rect, poly, circle, line, arc and text. A filled shape keeps
copper out of its area, an unfilled one out of its stroke. Every emitted
polygon CONTAINS its true shape: circles are circumscribed polygons (their
holes inscribed), arc strokes are chord bands widened by the chord's sagitta,
and text is a generous bounding box (KiCad's stroke font: at most
_TEXT_EM_PER_CHAR em per character, 1.6 em line pitch, plus stroke and an
italic slant allowance).

Pad openings (ownership rule): a graphic that meets the copper of a pad with a
net on the same side (its real, possibly rotated rectangle) is that pad's
opening only where it lies within OWN_MARGIN of the pad. A graphic wholly
inside the pad grown by OWN_MARGIN is the pad's own opening (a mask-defined
pad, a test point) and keeps nothing out; anything larger keeps its area
minus each touching pad grown by OWN_MARGIN, so a broad opening still keeps
copper out everywhere but next to its pads. A pad that is merely near a
graphic exempts nothing.

Scope: the keep-outs reach the consumers that read `routing_keepouts` (the
obstacle stamper and the octolinear smoother). Other copper writers (bridge
and cleanup passes, copper_keepout_gate) read the rule areas only, and the
GUI reads the artwork from the saved board file, not the live board.
"""
import math
import re
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

Point = Tuple[float, float]

_MASK_TO_CU = {'F.Mask': 'F.Cu', 'B.Mask': 'B.Cu'}
# Pads use `(layers ...)`; only a graphic carries a singular mask `(layer ..)`.
_MASK_LAYER_RE = re.compile(r'\(layer\s+"([FB]\.Mask)"')
_GR_RE = re.compile(r'\((gr_(?:rect|poly|circle|line|arc|text))\s')
_FP_RE = re.compile(r'\((fp_(?:rect|poly|circle|line|arc|text))\s')
_NUM = r'(-?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)'
_XY_RE = {k: re.compile(r'\(' + k + r'\s+' + _NUM + r'\s+' + _NUM)
          for k in ('start', 'end', 'center', 'mid')}
_AT_RE = re.compile(r'\(at\s+' + _NUM + r'\s+' + _NUM + r'(?:\s+' + _NUM + r')?\s*\)')
_WIDTH_RE = re.compile(r'\(width\s+' + _NUM + r'\s*\)')
_FILL_RE = re.compile(r'\(fill\s+(\w+)')
_SIZE_RE = re.compile(r'\(size\s+' + _NUM + r'\s+' + _NUM + r'\s*\)')
_THICK_RE = re.compile(r'\(thickness\s+' + _NUM + r'\s*\)')
_JUSTIFY_RE = re.compile(r'\(justify([^()]*)\)')
_STR_RE = re.compile(r'"((?:[^"\\]|\\.)*)"')
_CIRCLE_SEGS = 32
_ARC_SEGS = 16
# A graphic within this distance of a netted pad's copper belongs to that pad's
# opening (KiCad's mask expansions and mask-defined pads stay well inside it).
OWN_MARGIN = 0.5
# Widest stroke-font glyph advance (W, M) in em, generous.
_TEXT_EM_PER_CHAR = 1.3


def _filled(block: str) -> bool:
    m = _FILL_RE.search(block)
    return bool(m) and m.group(1) in ('solid', 'yes')


def _stroke(block: str) -> float:
    m = _WIDTH_RE.search(block)
    return float(m.group(1)) if m else 0.0


def _xy(block: str, key: str) -> Optional[Point]:
    m = _XY_RE[key].search(block)
    return (float(m.group(1)), float(m.group(2))) if m else None


def _rot(p: Point, deg: float) -> Point:
    """KiCad's angle (counter-clockwise on screen, y down)."""
    if not deg:
        return p
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return (p[0] * c + p[1] * s, -p[0] * s + p[1] * c)


def _band(a: Point, b: Point, half: float) -> List[Point]:
    """The stroke of segment a-b, square caps: a rectangle `2*half` wide."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    n = math.hypot(dx, dy)
    if n == 0:
        return [(a[0] - half, a[1] - half), (a[0] + half, a[1] - half),
                (a[0] + half, a[1] + half), (a[0] - half, a[1] + half)]
    ux, uy = dx / n * half, dy / n * half
    return [(a[0] - ux - uy, a[1] - uy + ux), (b[0] + ux - uy, b[1] + uy + ux),
            (b[0] + ux + uy, b[1] + uy - ux), (a[0] - ux + uy, a[1] - uy - ux)]


def _rect(x1, y1, x2, y2) -> List[Point]:
    return [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]


def _circle(c: Point, r: float, outside: bool = True) -> List[Point]:
    """A _CIRCLE_SEGS-gon of the circle: circumscribed (contains the circle)
    when `outside`, else inscribed (inside it, for holes)."""
    if outside:
        r = r / math.cos(math.pi / _CIRCLE_SEGS)
    return [(c[0] + r * math.cos(2 * math.pi * i / _CIRCLE_SEGS),
             c[1] + r * math.sin(2 * math.pi * i / _CIRCLE_SEGS)) for i in range(_CIRCLE_SEGS)]


def _arc_points(s: Point, m: Point, e: Point) -> Tuple[List[Point], float]:
    """(points along the arc, sagitta of one chord between them)."""
    ax, ay = s
    bx, by = m
    cx, cy = e
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12:
        return [s, e], 0.0
    ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay)
          + (cx * cx + cy * cy) * (ay - by)) / d
    uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx)
          + (cx * cx + cy * cy) * (bx - ax)) / d
    r = math.hypot(ax - ux, ay - uy)
    a0 = math.atan2(ay - uy, ax - ux)
    am = math.atan2(by - uy, bx - ux)
    a1 = math.atan2(cy - uy, cx - ux)
    sweep = (a1 - a0) % (2 * math.pi)
    if (am - a0) % (2 * math.pi) > sweep:  # the arc runs the other way round
        sweep -= 2 * math.pi
    sag = r * (1.0 - math.cos(abs(sweep) / _ARC_SEGS / 2.0))
    return ([(ux + r * math.cos(a0 + sweep * i / _ARC_SEGS),
              uy + r * math.sin(a0 + sweep * i / _ARC_SEGS)) for i in range(_ARC_SEGS + 1)],
            sag)


def _text_box(block: str) -> Tuple[Optional[Point], float, List[Point]]:
    """(anchor, angle, box corners relative to the anchor before rotation)."""
    from kicad_parser import find_matching_paren
    at = _AT_RE.search(block)
    s = _STR_RE.search(block)
    if not at or not s:
        return None, 0.0, []
    lines = s.group(1).split('\\n')
    eff = block.find('(effects')
    eff_txt = block[eff:find_matching_paren(block, eff)] if eff >= 0 else ''
    sz = _SIZE_RE.search(eff_txt)
    h, w = (float(sz.group(1)), float(sz.group(2))) if sz else (1.0, 1.0)  # (size H W)
    th = _THICK_RE.search(eff_txt)
    pad = (float(th.group(1)) if th else 0.15 * h) / 2.0
    width = max(len(ln) for ln in lines) * w * _TEXT_EM_PER_CHAR + 0.25 * h  # + italic slant
    height = h + (len(lines) - 1) * 1.6 * h
    j = _JUSTIFY_RE.search(eff_txt)
    just = j.group(1).split() if j else []
    if 'mirror' in just:
        just = ['right' if t == 'left' else 'left' if t == 'right' else t for t in just]
    x0 = 0.0 if 'left' in just else -width if 'right' in just else -width / 2
    y0 = 0.0 if 'top' in just else -height if 'bottom' in just else -height / 2
    box = _rect(x0 - pad, y0 - pad, x0 + width + pad, y0 + height + pad)
    return ((float(at.group(1)), float(at.group(2))),
            float(at.group(3) or 0.0), box)


def _shapes(kind: str, block: str) -> List[Tuple[List[Point], List[List[Point]]]]:
    """[(outline, holes)] in the block's own coordinates, text excepted."""
    from kicad_parser import find_matching_paren, kicad_poly_shape_points
    half = _stroke(block) / 2.0
    filled = _filled(block)
    out = []
    if kind == 'rect':
        a, b = _xy(block, 'start'), _xy(block, 'end')
        if not a or not b:
            return []
        x1, x2 = sorted((a[0], b[0]))
        y1, y2 = sorted((a[1], b[1]))
        outer = _rect(x1 - half, y1 - half, x2 + half, y2 + half)
        inner = ([] if filled or x2 - x1 <= 2 * half or y2 - y1 <= 2 * half
                 else [_rect(x1 + half, y1 + half, x2 - half, y2 - half)])
        out.append((outer, inner))
    elif kind == 'circle':
        c, e = _xy(block, 'center'), _xy(block, 'end')
        if not c or not e:
            return []
        r = math.hypot(e[0] - c[0], e[1] - c[1])
        inner = [] if filled or r <= half else [_circle(c, r - half, outside=False)]
        out.append((_circle(c, r + half), inner))
    elif kind == 'poly':
        p = block.find('(pts')
        if p < 0:
            return []
        pts = kicad_poly_shape_points(block[p:find_matching_paren(block, p)])
        if len(pts) < 2:
            return []
        if filled and len(pts) >= 3:
            out.append((pts, []))
        if half > 0:
            ring = list(pts) + ([pts[0]] if len(pts) >= 3 else [])
            out += [(_band(a, b, half), []) for a, b in zip(ring, ring[1:])]
    elif kind == 'line':
        a, b = _xy(block, 'start'), _xy(block, 'end')
        if a and b and half > 0:
            out.append((_band(a, b, half), []))
    elif kind == 'arc':
        a, m, b = _xy(block, 'start'), _xy(block, 'mid'), _xy(block, 'end')
        if a and m and b and half > 0:
            pts, sag = _arc_points(a, m, b)
            out += [(_band(p, q, half + sag), []) for p, q in zip(pts, pts[1:])]
    return out


def _point_in(p: Point, poly: Sequence[Point]) -> bool:
    x, y = p
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def _seg_dist(p: Point, a: Point, b: Point) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    n = dx * dx + dy * dy
    t = 0.0 if n == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / n))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def _pad_shapes(footprints) -> Dict[str, list]:
    """{copper layer: [shapely polygon of the pad's copper rectangle]} of every
    pad with a net (its real size and tilt; round and rounded pads by their
    enclosing rectangle)."""
    from shapely.geometry import box
    from shapely import affinity
    shapes: Dict[str, list] = {'F.Cu': [], 'B.Cu': []}
    for fp in (footprints or {}).values():
        for pad in getattr(fp, 'pads', ()) or ():
            if not getattr(pad, 'net_id', 0):
                continue
            x, y = pad.global_x, pad.global_y
            hx, hy = pad.size_x / 2.0, pad.size_y / 2.0
            g = box(x - hx, y - hy, x + hx, y + hy)
            rr = getattr(pad, 'rect_rotation', 0.0) or 0.0
            if rr:
                g = affinity.rotate(g, rr, origin=(x, y))
            lays = set(pad.layers or ())
            for cu in ('F.Cu', 'B.Cu'):
                if cu in lays or '*.Cu' in lays or 'F&B.Cu' in lays:
                    shapes[cu].append(g)
    return shapes


def _rings(geom):
    """[(outline, holes)] of a shapely (Multi)Polygon, as point lists."""
    polys = getattr(geom, 'geoms', [geom])
    out = []
    for g in polys:
        if g.is_empty or g.geom_type != 'Polygon' or g.area <= 1e-9:
            continue
        out.append((list(g.exterior.coords)[:-1],
                    [list(h.coords)[:-1] for h in g.interiors]))
    return out


def _unowned(poly, holes, pads) -> list:
    """The parts of a mask graphic that are no pad's opening (module doc)."""
    from shapely.geometry import Polygon
    g = Polygon(poly, holes)
    if not g.is_valid:
        g = g.buffer(0)
    touching = [p for p in pads if p.intersects(g)]
    if not touching:
        return [(poly, holes)]
    grown = [p.buffer(OWN_MARGIN, join_style=2) for p in touching]
    if any(g.within(z) for z in grown):
        return []
    rest = g
    for z in grown:
        rest = rest.difference(z)
    return _rings(rest)


def routing_keepouts(board_info) -> List[dict]:
    """Rule-area keep-outs, plus mask artwork when the run opted in
    (board_info.mask_keepouts, set by batch_route's mask_keepouts)."""
    keepouts = list(getattr(board_info, 'keepouts', None) or [])
    if getattr(board_info, 'mask_keepouts', False):
        keepouts += list(getattr(board_info, 'mask_artwork', None) or [])
    return keepouts


def _entry(poly, holes, cu, owner, mask):
    return {'polygon': poly, 'holes': holes, 'layers': {cu},
            'tracks_allowed': False, 'vias_allowed': False,
            'copper_pour_allowed': True, 'pads_allowed': True,
            'footprints_allowed': True, 'in_footprint': bool(owner),
            'owner': owner, 'name': mask + ' artwork', 'mask_artwork': True}


def _graphics(text: str, regex, xform, owner: str):
    """(polygon, holes, mask layer, owner) per mask graphic in `text`."""
    from kicad_parser import find_matching_paren
    for m in regex.finditer(text):
        end = find_matching_paren(text, m.start())
        block = text[m.start():end]
        lm = _MASK_LAYER_RE.search(block)
        if not lm:
            continue
        kind = m.group(1).split('_', 1)[1]
        if kind == 'text':
            anchor, ang, box = _text_box(block)
            if anchor is None:
                continue
            # The text's own angle is absolute in the file (KiCad 6+), so only
            # its anchor follows the footprint pose.
            ax, ay = xform([anchor])[0]
            shapes = [([(ax + x, ay + y) for x, y in (_rot(p, ang) for p in box)], [])]
        else:
            shapes = [(xform(o), [xform(h) for h in hs]) for o, hs in _shapes(kind, block)]
        for poly, holes in shapes:
            if len(poly) >= 3:
                yield poly, holes, lm.group(1), owner


def extract_mask_artwork(content: str, footprints=None) -> List[dict]:
    """Keep-out dicts for the F.Mask / B.Mask artwork of a board (see module
    doc). `footprints` (parse_kicad_pcb's) supply the pads whose own mask
    openings are exempt; None exempts nothing."""
    if not _MASK_LAYER_RE.search(content):
        return []
    from kicad_parser import (footprint_at_match, iter_footprint_blocks,
                              _mask_footprint_blocks, local_to_global)

    def board(pts):
        return list(pts)

    found = list(_graphics(_mask_footprint_blocks(content), _GR_RE, board, ''))
    for _s, _e, fp_text, _raw, key in iter_footprint_blocks(content):
        if not _MASK_LAYER_RE.search(fp_text):
            continue
        at = footprint_at_match(fp_text)
        fx, fy, fr = ((float(at.group(1)), float(at.group(2)), float(at.group(3) or 0.0))
                      if at else (0.0, 0.0, 0.0))

        def fp_xform(pts, fx=fx, fy=fy, fr=fr):
            return [local_to_global(fx, fy, fr, x, y) for x, y in pts]
        found += list(_graphics(fp_text, _FP_RE, fp_xform, key))

    pads = _pad_shapes(footprints)
    out = []
    for poly, holes, mask, owner in found:
        cu = _MASK_TO_CU[mask]
        for keep, keep_holes in _unowned(poly, holes, pads[cu]):
            out.append(_entry(keep, keep_holes, cu, owner, mask))
    return out

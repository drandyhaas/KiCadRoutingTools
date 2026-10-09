"""
Parse placement-relevant geometry from KiCad PCB files.

Extracts courtyard boundaries and other footprint geometry that
kicad_parser doesn't provide, for use by the placement tool.

Both readers here split footprint blocks with ``kicad_parser.find_matching_paren``
rather than a naive paren counter: a property value containing a lone paren (an
MPN like ``"TCR2EF115,LM(CT"``) makes a naive scan run past the block end and
swallow the following footprints (issue #113).

The courtyard reader deliberately mirrors ``kicad_parser._footprint_edge_points``,
which solved the same problems one layer up for Edge.Cuts: every shape kind
(fp_line / fp_rect / fp_arc / fp_circle / fp_poly), and an element gap that
cannot cross into the next element (see ``_FP_ELEMENT_GAP``).
"""
from __future__ import annotations

import math
import re
from typing import Dict, Optional, Set, Tuple

from kicad_parser import (_arc_to_segments, find_matching_paren,
                          footprint_head_flags, iter_footprint_blocks,
                          strip_bare_shape_locks, upgrade_legacy_arcs)

Bbox = Tuple[float, float, float, float]

# Regex gap that matches anything EXCEPT the start of another footprint element
# or a layer tag, so a lazy match cannot run past the current element to a later
# ``(layer ...)`` token. This is the footprint-level twin of kicad_parser's
# _GR_ELEMENT_GAP (issue #77, board graphics).
#
# With a plain `.*?` here, a match could START on a silk fp_line and BORROW the
# layer tag of a later courtyard element, dragging the silk element's
# coordinates into the courtyard bbox: with the standard element order (silk,
# courtyard, fab) a 0402 whose true courtyard is (-0.5,-0.4)-(0.5,0.4) read as
# (-2.0,-0.9)-(2.0,0.4). Usually that only INFLATES a part (spurious clearance
# rejections, phantom halo cost), but on a non-rectangular courtyard it can lose
# an extreme and UNDER-estimate, letting real overlap validate; it also made the
# swap-identity guard see two instances of one footprint as different shapes
# purely because their neighbouring silk differed (#456 item 3).
_FP_ELEMENT_GAP = r'(?:(?!\(fp_|\(pad\b|\(layers?\b)[\s\S])*?'

_CRTYD_LAYER = r'\(layer\s+"([FB])\.CrtYd"\)'
_FAB_LAYER = r'\(layer\s+"([FB])\.Fab"\)'
# #896. The LAST resort before the pad bbox, and the only drawn geometry a
# hand-rolled library may carry at all: on esp_prog (OLIMEX) not one of 21
# footprints draws a courtyard and six -- CON1, CON2, U1, U2, Q1, Q2 -- draw no
# .Fab either, so silk is their only body. Read through the same
# `_courtyard_points_by_side`, so `_FP_ELEMENT_GAP` protects it too; a
# hand-written silk regex is exactly how #456 item 3 comes back.
_SILK_LAYER = r'\(layer\s+"([FB])\.SilkS"\)'
_NUM = r'([\d.eE+-]+)'


def _footprint_blocks(content: str):
    """Yield (key, footprint_text) for every footprint block.

    The key is the parser's own (#726), so a courtyard, a fab outline and a
    lock all land under the name `pcb.footprints` uses. This mattered as soon
    as duplicates stopped collapsing: `placement/labels.py` looks a courtyard
    up as `courtyard_sides.get(fp.reference)`, so a `TP4~2` against a
    `TP4`-keyed map would silently find nothing and fall back to the pad bbox.

    Two behaviour changes come with delegating, both in the conservative
    direction. The old regex required a non-empty `(property "Reference" ...)`,
    so it skipped reference-LESS blocks and KiCad 6/7 `(fp_text reference ...)`
    blocks ENTIRELY -- those now appear, under their `#uuid` / 6-7 names.
    `extract_locked_refs` therefore starts returning `#uuid` keys for locked
    NPTH drill dots (thunderscope has 86), which adds to lock sets and never
    subtracts.
    """
    for _start, _end, fp_text, _raw_ref, key in iter_footprint_blocks(content):
        yield key, fp_text


#: #1094: `(path, mtime, size) -> [(key, footprint_text)]`. Every reader
#: below splits the whole file into footprint blocks, and one
#: check_assembly grade calls five of them twice each (occupancy and fab
#: bodies): the split was 5 of its 7 seconds on glasgow_revC.
_BLOCK_CACHE: Dict[tuple, list] = {}


def _file_blocks(pcb_file: str) -> list:
    """`_footprint_blocks` of a board FILE, split once per file version."""
    import os as _os
    try:
        st = _os.stat(pcb_file)
        key = (_os.path.abspath(pcb_file), st.st_mtime_ns, st.st_size)
    except OSError:
        key = None
    if key is not None and key in _BLOCK_CACHE:
        return _BLOCK_CACHE[key]
    with open(pcb_file, 'r', encoding='utf-8') as f:
        # A pre-6.0 file's center/angle fp_arcs read as start/mid/end, the
        # only arc form the outline readers here match (read-only copy).
        blocks = list(_footprint_blocks(strip_bare_shape_locks(upgrade_legacy_arcs(f.read()))))
    if key is not None:
        if len(_BLOCK_CACHE) > 8:
            _BLOCK_CACHE.clear()
        _BLOCK_CACHE[key] = blocks
    return blocks


def extract_locked_refs(pcb_file: str) -> Set[str]:
    """
    Find all footprints marked as locked in the PCB file.

    Returns set of component references (e.g., {"P1", "J1"}).
    """
    blocks = _file_blocks(pcb_file)

    locked = set()
    for ref, fp_text in blocks:
        # Check for (locked yes) before the first pad
        # It appears early in the footprint block, before properties
        first_pad = fp_text.find('(pad ')
        search_region = fp_text[:first_pad] if first_pad > 0 else fp_text[:500]
        if (re.search(r'\(locked(?:\s+yes)?\)', search_region)  # (locked) 2021 nightlies
                or 'locked' in footprint_head_flags(fp_text)):  # KiCad 6: bare
            locked.add(ref)
    return locked


def _full_circle_arc(sx, sy, mx, my, ex, ey, grid=1e-6):
    """`((cx, cy), r)` when an fp_arc's start and end coincide (in KiCad's
    1 nm unit, `grid`) but its mid does not: KiCad draws that as a FULL
    circle whose diameter is start-mid, and closes it (kicad-cli 10, final
    P1 verifier: such an arc alone over a pin reports pth_inside_courtyard).
    `_arc_to_segments` reads it as the zero-length chord start-end. None
    otherwise -- ends a few nm apart are an ordinary (near-full) arc."""
    def q(v):
        return round(v / grid)
    if (q(sx), q(sy)) != (q(ex), q(ey)) or (q(sx), q(sy)) == (q(mx), q(my)):
        return None
    return ((sx + mx) / 2.0, (sy + my) / 2.0), math.hypot(mx - sx, my - sy) / 2


def _courtyard_points_by_side(fp_text: str,
                              _CRTYD_LAYER: str = _CRTYD_LAYER
                              ) -> Dict[str, list]:
    """Local-frame outline points of one footprint, keyed by 'F' / 'B'.

    Defaults to the courtyard layers; pass a different single-group layer
    regex (e.g. `_FAB_LAYER`) to read another outline layer with the same
    geometry rules (run-6: the fab BODY channel). The parameter shadows the
    module constant so the regex sites below stay untouched."""
    pts: Dict[str, list] = {}

    def add(side, *xy_pairs):
        pts.setdefault(side, []).extend(xy_pairs)

    for m in re.finditer(
            r'\(fp_(line|rect)\s+\(start\s+' + _NUM + r'\s+' + _NUM + r'\)\s+'
            r'\(end\s+' + _NUM + r'\s+' + _NUM + r'\)' + _FP_ELEMENT_GAP
            + _CRTYD_LAYER, fp_text, re.DOTALL):
        x1, y1, x2, y2 = (float(m.group(i)) for i in range(2, 6))
        if m.group(1) == 'rect':
            # all four corners: a rect's start/end alone don't bound it once the
            # footprint rotates (same reason as _footprint_edge_points)
            add(m.group(6), (x1, y1), (x2, y2), (x1, y2), (x2, y1))
        else:
            add(m.group(6), (x1, y1), (x2, y2))

    for m in re.finditer(
            r'\(fp_arc\s+\(start\s+' + _NUM + r'\s+' + _NUM + r'\)\s+'
            r'\(mid\s+' + _NUM + r'\s+' + _NUM + r'\)\s+'
            r'\(end\s+' + _NUM + r'\s+' + _NUM + r'\)' + _FP_ELEMENT_GAP
            + _CRTYD_LAYER, fp_text, re.DOTALL):
        sx, sy, mx, my, ex, ey = (float(m.group(i)) for i in range(1, 7))
        full = _full_circle_arc(sx, sy, mx, my, ex, ey)
        if full is not None:
            (cx, cy), r = full
            add(m.group(7), (cx - r, cy - r), (cx + r, cy + r))
            continue
        for seg in _arc_to_segments((sx, sy), (mx, my), (ex, ey)):
            add(m.group(7), *seg)

    for m in re.finditer(
            r'\(fp_circle\s+\(center\s+' + _NUM + r'\s+' + _NUM + r'\)\s+'
            r'\(end\s+' + _NUM + r'\s+' + _NUM + r'\)' + _FP_ELEMENT_GAP
            + _CRTYD_LAYER, fp_text, re.DOTALL):
        cx, cy, ex, ey = (float(m.group(i)) for i in range(1, 5))
        r = math.hypot(ex - cx, ey - cy)
        # the disc's bbox; rotation-invariant about the centre
        add(m.group(5), (cx - r, cy - r), (cx + r, cy + r))

    # fp_poly carries its vertices in a nested (pts (xy ...) ...) block, so it is
    # read as a balanced sub-expression rather than by a flat regex.
    for pm in re.finditer(r'\(fp_poly\b', fp_text):
        poly_text = fp_text[pm.start():find_matching_paren(fp_text, pm.start())]
        lm = re.search(_CRTYD_LAYER, poly_text)
        if not lm:
            continue
        for xm in re.finditer(r'\(xy\s+' + _NUM + r'\s+' + _NUM + r'\)', poly_text):
            add(lm.group(1), (float(xm.group(1)), float(xm.group(2))))

    return pts


def _bbox(points) -> Bbox:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return (min(xs), min(ys), max(xs), max(ys))


def extract_courtyard_sides(pcb_file: str) -> Dict[str, Dict[str, Bbox]]:
    """
    Parse courtyard extents from each footprint block, kept PER SIDE.

    Returns dict mapping component reference to {'F': bbox} / {'B': bbox} /
    both, where each bbox is (local_min_x, local_min_y, local_max_x,
    local_max_y) in the footprint's local coordinate system. These must be
    transformed to global coordinates using the footprint's position and
    rotation. A reference with no courtyard on any layer is absent entirely.

    Side matters because a back-side part and a front-side part overlap in XY
    without overlapping in copper; `extract_courtyard_bboxes` keeps the legacy
    union-of-sides view for callers that don't model side.
    """
    blocks = _file_blocks(pcb_file)

    result: Dict[str, Dict[str, Bbox]] = {}
    for ref, fp_text in blocks:
        if '.CrtYd"' not in fp_text:
            continue
        by_side = _courtyard_points_by_side(fp_text)
        if by_side:
            result[ref] = {side: _bbox(pts) for side, pts in by_side.items()}
    return result


def extract_fab_sides(pcb_file: str) -> Dict[str, Dict[str, Bbox]]:
    """Per-side F/B.Fab BODY-outline bboxes, `extract_courtyard_sides`'s
    sibling (run-6). The fab outline is the drawn component BODY with no
    courtyard margin, so a cross-footprint fab intersection is two parts
    physically colliding -- the discriminating channel between a real stack
    and a legitimate shell-overhang courtyard kiss."""
    blocks = _file_blocks(pcb_file)
    result: Dict[str, Dict[str, Bbox]] = {}
    for ref, fp_text in blocks:
        if '.Fab"' not in fp_text:
            continue
        by_side = _courtyard_points_by_side(fp_text, _FAB_LAYER)
        if by_side:
            result[ref] = {side: _bbox(pts) for side, pts in by_side.items()}
    return result


def extract_silk_sides(pcb_file: str) -> Dict[str, Dict[str, Bbox]]:
    """Per-side F/B.SilkS bboxes -- `extract_fab_sides`'s sibling (#896).

    THIS IS NOT A BODY ON ITS OWN, and callers must not treat it as one. On a
    stock KiCad footprint silk is a pair of clipped side ticks that bracket the
    pads on one axis and are cut away on the other: measured on esp_prog, all 10
    footprints drawing both fab and silk have a silk bbox NARROWER than the fab
    body along the pad axis (Y1: 0.508mm against a 3.200mm body) and WIDER
    across it. There is no offset that reconciles the two -- the sign of the
    error differs per axis -- which is why `placement.body` unions this with the
    pad bbox rather than substituting it, and why nothing here applies an
    expansion. `placement.body.body_geometry` is the only intended consumer.
    """
    blocks = _file_blocks(pcb_file)
    result: Dict[str, Dict[str, Bbox]] = {}
    for ref, fp_text in blocks:
        if '.SilkS"' not in fp_text:
            continue
        by_side = _courtyard_points_by_side(fp_text, _SILK_LAYER)
        if by_side:
            result[ref] = {side: _bbox(pts) for side, pts in by_side.items()}
    return result


#: Coordinates are snapped to this grid (mm) before an outline is noded, so two
#: segment ends KiCad wrote as 1.2500001 and 1.25 still meet.
_OUTLINE_SNAP_MM = 1e-4
#: A polygonised outline must contain every drawn vertex to within this (mm);
#: otherwise part of the drawing did not close and the convex hull is used.
_OUTLINE_COVER_TOL_MM = 1e-3
#: Segment ends within this of each other are joined on a second attempt
#: when a drawing does not close as written (#1094 verifier: ulx3s BAT1).
#: KiCad's own courtyard chaining epsilon (`BuildCourtyardCaches`, 0.02 mm):
#: a drawing it closes is a courtyard it tests pins against, and one this
#: called open read `courtyard_malformed` and gated nothing (fa10 P1 verifier:
#: synthetic gaps of 15 and 19 um close in KiCad, 30 um does not). A drawing
#: joined at this distance is also ACCEPTED at it (`tol` below): judged at
#: `_OUTLINE_COVER_TOL_MM`, the joined shape's moved corner always failed,
#: which is how glasgow J4's 8 um corner gap read open while kicad-cli
#: closes it.
_OUTLINE_JOIN_MM = 0.02
#: ...as far as it can be seen through the `_OUTLINE_SNAP_MM` grid: snapping
#: both ends can grow a gap by up to sqrt(2) grid steps (final P1 verifiers:
#: a 19.97 um diagonal gap off the grid snapped to 20.08 and read open, while
#: KiCad closes it), and KiCad rounds a rotated footprint's ends to its 1 nm
#: unit first, which can shrink a gap by up to sqrt(2) nm more (a 20.001 um
#: gap at 15 degrees: KiCad 19.9999, closed). Generous, like the join
#: itself: a gap between 20 and ~20.14 um closes here and not in KiCad
#: (KNOWN_CONSERVATIVE in test_1212).
_OUTLINE_JOIN_REACH_MM = _OUTLINE_JOIN_MM + math.sqrt(2) * (
    _OUTLINE_SNAP_MM + 1e-6)
OUTLINE_POLYGON = 'polygon'
OUTLINE_HULL = 'hull'
#: One read of a board's outlines per (path, mtime, size, layer): a single
#: check_assembly reads them twice (occupancy and fab), and the placement
#: tools grade the same file again and again.
_SHAPE_CACHE: Dict[tuple, Dict[str, Dict[str, tuple]]] = {}


def _nested_even_odd(geoms):
    """One outline from closed contours the way KiCad assembles a courtyard
    (`ConvertOutlineToPolygon`): a contour enclosed by an ODD number of the
    others is a hole in its parent, one enclosed by an even number an
    outline. So two concentric circles are a RING -- glasgow MK1-MK4's
    mounting-hole keep-outs, 34 mm2 in KiCad, which a plain union filled to
    78.5 mm2. Contours that overlap without one enclosing the other are
    united, and an exact duplicate counts once.
    """
    from shapely.geometry import Polygon
    from shapely.ops import unary_union

    contours = []
    for g in geoms:
        stack = [g]
        while stack:
            h = stack.pop()
            if h is None or h.is_empty:
                continue
            if h.geom_type == 'Polygon':
                c = Polygon(h.exterior)
                if c.area > 0 and not any(c.equals(q) for q in contours):
                    contours.append(c)
            else:
                stack.extend(getattr(h, 'geoms', ()))
    if not contours:
        return None
    depth = [sum(1 for j, q in enumerate(contours)
                 if j != i and q.contains(c))
             for i, c in enumerate(contours)]
    shape = None
    for d in sorted(set(depth)):
        level = unary_union([c for c, k in zip(contours, depth) if k == d])
        if shape is None:
            shape = level
        elif d % 2:
            shape = shape.difference(level)
        else:
            shape = shape.union(level)
    return shape


def _outline_shapes_by_side(fp_text: str, layer_re: str,
                            even_odd: bool = False) -> Dict[str, tuple]:
    """The TRUE drawn outline of one footprint on `layer_re`, per side (#1094).

    `{side: (shapely geometry in the local frame, how)}`, where `how` is
    `OUTLINE_POLYGON` when the drawing closes and `OUTLINE_HULL` when it does
    not (an open courtyard drawing, a missing segment) and the convex hull of
    every drawn vertex stands in for it. Never smaller than what was drawn.

    `_courtyard_points_by_side` reads the same elements but keeps only their
    points, which is all a bbox needs; a bbox cannot survive a 45 degree
    rotation (StickHub: 74 phantom courtyard pairs, KiCad finds 0), and an
    oriented bbox cannot either, because a stepped courtyard such as StickHub
    U1's 24 segments still over-states its corners by 2 mm2. This keeps the
    shape.

    `even_odd` assembles nested contours as KiCad assembles a COURTYARD
    (`_nested_even_odd`: a contour inside another is a hole). Off, every
    closed contour is united -- a .Fab drawing's inner circle is a detail
    of the body (a button, a lens), not a hole through it.
    """
    from shapely.geometry import LineString, Point, Polygon, box
    from shapely.ops import polygonize, unary_union

    def snap(v):
        return round(v / _OUTLINE_SNAP_MM) * _OUTLINE_SNAP_MM

    lines: Dict[str, list] = {}
    areas: Dict[str, list] = {}
    verts: Dict[str, list] = {}
    rings: Dict[str, list] = {}   # full-circle arcs: (circle, start)

    def seg(side, a, b):
        # A zero-length element (a line, or an arc whose start, mid and end
        # coincide) is no part of the outline: KiCad drops it (kicad-cli 10:
        # a square plus a zero-length fp_line, fp_arc or fp_rect is NOT
        # malformed, and a pin under the square is reported), so its point
        # must not be a vertex the outline has to cover. An arc whose ends
        # meet but whose mid does not is a full circle (`_full_circle_arc`).
        a = (snap(a[0]), snap(a[1]))
        b = (snap(b[0]), snap(b[1]))
        if a == b:
            return
        verts.setdefault(side, []).extend((a, b))
        lines.setdefault(side, []).append(LineString((a, b)))

    for m in re.finditer(
            r'\(fp_(line|rect)\s+\(start\s+' + _NUM + r'\s+' + _NUM + r'\)\s+'
            r'\(end\s+' + _NUM + r'\s+' + _NUM + r'\)' + _FP_ELEMENT_GAP
            + layer_re, fp_text, re.DOTALL):
        x1, y1, x2, y2 = (float(m.group(i)) for i in range(2, 6))
        side = m.group(6)
        if m.group(1) == 'rect':
            if (snap(x1), snap(y1)) == (snap(x2), snap(y2)):
                continue  # zero-size: dropped, as `seg` drops a point
            r = box(min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2))
            areas.setdefault(side, []).append(r)
            verts.setdefault(side, []).extend(r.exterior.coords)
        else:
            seg(side, (x1, y1), (x2, y2))

    for m in re.finditer(
            r'\(fp_arc\s+\(start\s+' + _NUM + r'\s+' + _NUM + r'\)\s+'
            r'\(mid\s+' + _NUM + r'\s+' + _NUM + r'\)\s+'
            r'\(end\s+' + _NUM + r'\s+' + _NUM + r'\)' + _FP_ELEMENT_GAP
            + layer_re, fp_text, re.DOTALL):
        sx, sy, mx, my, ex, ey = (float(m.group(i)) for i in range(1, 7))
        ring = _full_circle_arc(sx, sy, mx, my, ex, ey)
        if ring is not None:
            c = Point(*ring[0]).buffer(ring[1], 32)
            rings.setdefault(m.group(7), []).append((c, (snap(sx), snap(sy))))
            verts.setdefault(m.group(7), []).extend(c.exterior.coords)
            continue
        for a, b in _arc_to_segments((sx, sy), (mx, my), (ex, ey)):
            seg(m.group(7), a, b)

    for m in re.finditer(
            r'\(fp_circle\s+\(center\s+' + _NUM + r'\s+' + _NUM + r'\)\s+'
            r'\(end\s+' + _NUM + r'\s+' + _NUM + r'\)' + _FP_ELEMENT_GAP
            + layer_re, fp_text, re.DOTALL):
        cx, cy, ex, ey = (float(m.group(i)) for i in range(1, 5))
        # A circle stays a circle: its bbox corners rotate off the disc.
        c = Point(cx, cy).buffer(math.hypot(ex - cx, ey - cy), 32)
        areas.setdefault(m.group(5), []).append(c)
        verts.setdefault(m.group(5), []).extend(c.exterior.coords)

    for pm in re.finditer(r'\(fp_poly\b', fp_text):
        poly_text = fp_text[pm.start():find_matching_paren(fp_text, pm.start())]
        lm = re.search(layer_re, poly_text)
        if not lm:
            continue
        pts = [(snap(float(xm.group(1))), snap(float(xm.group(2))))
               for xm in re.finditer(r'\(xy\s+' + _NUM + r'\s+' + _NUM + r'\)',
                                     poly_text)]
        if len(pts) >= 3:
            p = Polygon(pts)
            if not p.is_valid:
                p = p.buffer(0)
            areas.setdefault(lm.group(1), []).append(p)
        verts.setdefault(lm.group(1), []).extend(pts)

    def covers(shape, pts, tol=_OUTLINE_COVER_TOL_MM):
        if shape is None or shape.is_empty:
            return False
        import shapely
        return float(shapely.distance(shape, shapely.points(pts)).max()
                     ) <= tol

    def joined(segs):
        """`segs` with each DANGLING end -- one no other end meets exactly
        -- moved onto the nearest other dangling end within
        `_OUTLINE_JOIN_REACH_MM`, nearest pairs first: a chain's loose ends
        bridged, the way KiCad chains a courtyard. Ends that already meet
        are left alone, so an arc's short chords and a side drawn in short
        pieces keep their shape (a 20 um merge of ALL ends collapsed a
        0.1 mm fillet's chords); a 15 um piece between two 10 um gaps still
        closes (its four loose ends pair up). A segment is never joined to
        ITSELF: a 12 um piece between two 15 um gaps would otherwise pair
        its own ends first and vanish, leaving both gaps open (KiCad
        closes it)."""
        ends = [tuple(ln.coords[k]) for ln in segs for k in (0, -1)]
        seen: Dict[tuple, int] = {}
        for p in ends:
            k = (round(p[0], 6), round(p[1], 6))
            seen[k] = seen.get(k, 0) + 1
        free = [i for i, p in enumerate(ends)
                if seen[(round(p[0], 6), round(p[1], 6))] == 1]
        cands = []
        for a, i in enumerate(free):
            for j in free[a + 1:]:
                if i // 2 == j // 2:
                    continue
                d = math.hypot(ends[i][0] - ends[j][0],
                               ends[i][1] - ends[j][1])
                if d <= _OUTLINE_JOIN_REACH_MM + 1e-9:
                    cands.append((d, i, j))
        moved: Dict[int, tuple] = {}
        used: set = set()
        for _d, i, j in sorted(cands):
            if i in used or j in used:
                continue
            used.update((i, j))
            moved[j] = ends[i]
        out = []
        for k in range(len(segs)):
            a = moved.get(2 * k, ends[2 * k])
            b = moved.get(2 * k + 1, ends[2 * k + 1])
            if a != b:
                out.append(LineString((a, b)))
        return out

    # A full-circle arc nests like any contour (a hole inside an outline)
    # -- unless its START lies on another contour. KiCad decides hole or
    # outline from a contour's FIRST point, and a point on an edge is not
    # inside it: a circle drawn from the square's edge inward is outline,
    # the same circle drawn from its far side (its mid on the edge) a hole
    # (third fix verifier, kicad-cli 10). Such a circle is united here.
    united: Dict[str, list] = {}
    for side, rs in rings.items():
        for k, (c, start) in enumerate(rs):
            p = Point(start)
            others = (list(lines.get(side, ()))
                      + [a.boundary for a in areas.get(side, ())]
                      + [o.boundary for j, (o, _s) in enumerate(rs) if j != k])
            if any(g.distance(p) <= _OUTLINE_SNAP_MM for g in others):
                united.setdefault(side, []).append(c)
            else:
                areas.setdefault(side, []).append(c)

    compose = _nested_even_odd if even_odd else unary_union

    def composed(geoms, side):
        g = compose(geoms) if geoms else None
        if united.get(side):
            g = unary_union(([g] if g is not None else []) + united[side])
        return g

    out: Dict[str, tuple] = {}
    for side in set(verts) | set(areas):
        parts = list(areas.get(side, []))
        if lines.get(side):
            parts.extend(polygonize(unary_union(lines[side])))
        how = OUTLINE_POLYGON
        shape = composed(parts, side)
        pts = verts.get(side, [])
        tol = _OUTLINE_COVER_TOL_MM
        if pts and lines.get(side) and not covers(shape, pts):
            # Ends that miss each other by a few microns (ulx3s BAT1's
            # courtyard, glasgow J4's 8 um corner: KiCad closes them, a
            # strict join does not). Two ways to close them, tried in turn:
            # snap the drawing onto itself (an end within `_OUTLINE_JOIN_MM`
            # of another segment moves onto it), then bridge each loose end
            # to the nearest loose end (`joined`: a 15 um piece between two
            # 10 um gaps).
            # The first that covers every drawn vertex to within the join
            # distance is the outline.
            #
            # GENEROUS BY DESIGN. KiCad's own chaining (end to end, nearest
            # first) is not reproduced exactly: each stricter emulation
            # tried here missed drawings KiCad closes (corner-touching
            # squares, a nested square, an arc beside a corner: the pin
            # under them is a real pth_inside_courtyard, and reading them
            # open made it pass). Closing generously errs the other way --
            # a T that stops 15 um short, crossing ends, a stray parallel
            # line read closed here while KiCad flags malformed_courtyard
            # (it still tests a pin against the contours that do close).
            # tests/fixtures/1212_courtyard_chaining.json holds kicad-cli's
            # verdict on 178 drawings; the test asserts no drawing KiCad
            # closes reads open here. NOT modelled: a drawing that closes
            # plus debris no join absorbs (a stray piece outside it, a
            # there-and-back spike, a flat fp_rect or 2-point fp_poly)
            # reads as the hull here, its pins `courtyard_malformed`
            # (listed, not gating), where KiCad flags it malformed AND
            # tests its pins against the square.
            import shapely
            from shapely.geometry import MultiLineString
            ml = MultiLineString([list(ln.coords) for ln in lines[side]])
            for cand_lines in (shapely.snap(ml, ml, _OUTLINE_JOIN_REACH_MM),
                               joined(lines[side])):
                retry = list(areas.get(side, [])) + list(
                    polygonize(unary_union(cand_lines)))
                if retry:
                    cand = composed(retry, side)
                    if covers(cand, pts, _OUTLINE_JOIN_REACH_MM):
                        shape, tol = cand, _OUTLINE_JOIN_REACH_MM
                        break
        if pts and not covers(shape, pts, tol):
            from shapely.geometry import MultiPoint
            shape, how = MultiPoint(pts).convex_hull, OUTLINE_HULL
        if shape is None or shape.is_empty or shape.area <= 0:
            continue
        out[side] = (shape, how)
    return out


def _extract_outline_shapes(pcb_file: str, layer_re: str, marker: str,
                            even_odd: bool = False
                            ) -> Dict[str, Dict[str, tuple]]:
    import os as _os
    try:
        st = _os.stat(pcb_file)
        key = (_os.path.abspath(pcb_file), st.st_mtime_ns, st.st_size,
               layer_re, even_odd)
    except OSError:
        key = None
    if key is not None and key in _SHAPE_CACHE:
        return dict(_SHAPE_CACHE[key])
    result = _read_outline_shapes(pcb_file, layer_re, marker, even_odd)
    if key is not None:
        if len(_SHAPE_CACHE) > 16:
            _SHAPE_CACHE.clear()
        _SHAPE_CACHE[key] = result
    return dict(result)


def _read_outline_shapes(pcb_file: str, layer_re: str, marker: str,
                         even_odd: bool = False
                         ) -> Dict[str, Dict[str, tuple]]:
    blocks = _file_blocks(pcb_file)
    result: Dict[str, Dict[str, tuple]] = {}
    for ref, fp_text in blocks:
        if marker not in fp_text:
            continue
        by_side = _outline_shapes_by_side(fp_text, layer_re, even_odd)
        if by_side:
            result[ref] = by_side
    return result


def extract_courtyard_shapes(pcb_file: str) -> Dict[str, Dict[str, tuple]]:
    """Per-side courtyard OUTLINES, `{ref: {side: (geometry, how)}}` (#1094).

    `extract_courtyard_sides`' shape-keeping twin: same footprints, same keys,
    same elements, local frame. The bbox of each geometry is the bbox that
    function returns, so a consumer can use the bbox as a broad phase and this
    as the exact test.
    """
    return _extract_outline_shapes(pcb_file, _CRTYD_LAYER, '.CrtYd"',
                                   even_odd=True)


def extract_fab_shapes(pcb_file: str) -> Dict[str, Dict[str, tuple]]:
    """Per-side .Fab body OUTLINES; `extract_fab_sides`' twin (#1094)."""
    return _extract_outline_shapes(pcb_file, _FAB_LAYER, '.Fab"')


def extract_courtyard_bboxes(pcb_file: str) -> Dict[str, Bbox]:
    """
    Parse courtyard (F.CrtYd / B.CrtYd) extents from each footprint block.

    Returns dict mapping component reference to (local_min_x, local_min_y,
    local_max_x, local_max_y) — the courtyard bounding box in the footprint's
    local coordinate system, unioned over both sides. These must be transformed
    to global coordinates using the footprint's position and rotation.

    See `extract_courtyard_sides` for the per-side view.
    """
    out: Dict[str, Bbox] = {}
    for ref, sides in extract_courtyard_sides(pcb_file).items():
        boxes = list(sides.values())
        out[ref] = (min(b[0] for b in boxes), min(b[1] for b in boxes),
                    max(b[2] for b in boxes), max(b[3] for b in boxes))
    return out


def courtyard_for_side(sides: Optional[Dict[str, Bbox]],
                       side: str) -> Optional[Bbox]:
    """The courtyard a part on `side` should use: its own side's if drawn,
    else the other side's (a footprint mounted on B whose library drew only
    F.CrtYd is common), else None so the caller falls back to the pad bbox."""
    if not sides:
        return None
    return sides.get(side) or next(iter(sides.values()))


def warn_missing_courtyards(refs, label: str = 'placement',
                            sources: Optional[Dict[str, str]] = None) -> None:
    """Print a one-line warning naming refs that have no courtyard at all.

    Those parts fall back to `compute_footprint_bbox_local`, which is the union
    of PAD rects with no courtyard margin at all — a 0402's IPC courtyard is
    ~2.9x1.5mm against a ~1.9x0.9mm pad box — so the part is modelled smaller
    than it is and can be packed to a courtyard violation. Silence was the
    complaint in #456 item 3; the geometry itself is unchanged.

    `sources` ({ref: occupancy source}, `placement.body`) is the ARMED case
    (`body_model`, #1182): a courtyard-less part with a drawn body does not
    fall back to its pads, so only `pad_bbox` parts are named, and the line
    says what the others are spaced on instead.
    """
    refs = sorted(refs)
    if not refs:
        return
    if sources is not None:
        drawn = sorted(r for r in refs
                       if sources.get(r) not in (None, 'pad_bbox'))
        if drawn:
            mix: Dict[str, int] = {}
            for r in drawn:
                mix[sources[r]] = mix.get(sources[r], 0) + 1
            print(f"  NOTE [{label}]: {len(drawn)} footprint(s) without a "
                  f"courtyard are spaced on their drawn body united with "
                  f"their pads (body model: "
                  + ', '.join(f"{n} {s}" for s, n in sorted(mix.items()))
                  + ")")
        refs = [r for r in refs if r not in drawn]
        if not refs:
            return
    shown = ', '.join(refs[:12]) + (', ...' if len(refs) > 12 else '')
    print(f"  WARNING [{label}]: {len(refs)} footprint(s) have no courtyard "
          f"(F/B.CrtYd) and fall back to their pad bounding box, which carries "
          f"no courtyard margin: {shown}")

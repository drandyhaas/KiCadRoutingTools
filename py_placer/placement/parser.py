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
                          iter_footprint_blocks)

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


def extract_locked_refs(pcb_file: str) -> Set[str]:
    """
    Find all footprints marked as locked in the PCB file.

    Returns set of component references (e.g., {"P1", "J1"}).
    """
    with open(pcb_file, 'r', encoding='utf-8') as f:
        content = f.read()

    locked = set()
    for ref, fp_text in _footprint_blocks(content):
        # Check for (locked yes) before the first pad
        # It appears early in the footprint block, before properties
        first_pad = fp_text.find('(pad ')
        search_region = fp_text[:first_pad] if first_pad > 0 else fp_text[:500]
        if re.search(r'\(locked\s+yes\)', search_region):
            locked.add(ref)
    return locked


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
    with open(pcb_file, 'r', encoding='utf-8') as f:
        content = f.read()

    result: Dict[str, Dict[str, Bbox]] = {}
    for ref, fp_text in _footprint_blocks(content):
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
    with open(pcb_file, 'r', encoding='utf-8') as f:
        content = f.read()
    result: Dict[str, Dict[str, Bbox]] = {}
    for ref, fp_text in _footprint_blocks(content):
        if '.Fab"' not in fp_text:
            continue
        by_side = _courtyard_points_by_side(fp_text, _FAB_LAYER)
        if by_side:
            result[ref] = {side: _bbox(pts) for side, pts in by_side.items()}
    return result


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


def warn_missing_courtyards(refs, label: str = 'placement') -> None:
    """Print a one-line warning naming refs that have no courtyard at all.

    Those parts fall back to `compute_footprint_bbox_local`, which is the union
    of PAD rects with no courtyard margin at all — a 0402's IPC courtyard is
    ~2.9x1.5mm against a ~1.9x0.9mm pad box — so the part is modelled smaller
    than it is and can be packed to a courtyard violation. Silence was the
    complaint in #456 item 3; the geometry itself is unchanged.
    """
    refs = sorted(refs)
    if not refs:
        return
    shown = ', '.join(refs[:12]) + (', ...' if len(refs) > 12 else '')
    print(f"  WARNING [{label}]: {len(refs)} footprint(s) have no courtyard "
          f"(F/B.CrtYd) and fall back to their pad bounding box, which carries "
          f"no courtyard margin: {shown}")

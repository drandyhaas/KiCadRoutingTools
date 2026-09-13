"""Connected pads on a row that faces the board outline with nothing beyond it.

The one placement fact run 26 got wrong that no instrument could name: a
SOT-89 regulator seated with its three pins 0.4 mm from the north edge, every
one of its nets forced to leave under its own body or on the back, and the
review wrote "the only free side in that corner -- PASS" because criterion 4
of the boundary review (facing) had no number. This module is that number,
shared by three consumers so they cannot disagree about it:

  * `placement_score.edge_facing`   -- the report-only placement term;
  * `floorplan.rule_pins_to_edge`   -- the WARN a close-out grade prints;
  * `seeder` (opt-in, #C5)          -- the tie-break among rotations that fit.

DEFINITION. For one part: its connected pads (`net_id > 0`), the rectangle
those pad centres span, and the board outline's bounds. A pad is ON a face
when its centre lies within `max(pitch / 2, INTERIOR_EPS)` of that face's
segment; a face MEETS THE OUTLINE when its outward gap to the board bounds is
at most `edge_mm`; and a pad on such a face counts unless one of ITS net's
partner pads on another part lies beyond that face (`bearing_face` from the
part's centre), because then the row faces what it is wired to -- the
connector case. Each pad counts at most once.

Why `escape.assign_faces` is NOT called here, although it is "THE face rule"
for escape lanes: it assigns each pad exactly ONE face, and sends a single
row's END pads to the row's ends. Measured on run 26's context sheet
(`context_close.json`), U2's three-pin north row reads
`{east: 1, north: 1, west: 1}` under it, so a term built on that answer would
report the regulator as 1 pad of 3 facing the edge where the layout shows all
three. That rule answers "which lane does this pad escape through"; this one
asks "which side of the part is this pad on", and a pad at the corner of a
part is on two sides. Same face geometry (`escape._face_geometry`), same
pitch, different question -- said here the way `plane_cut_proxy` says why it
does not call `corridor_cut_mm`.
"""

from typing import Dict, Iterable, List, Sequence, Tuple

from .escape import FACES, INTERIOR_EPS, _face_geometry
from .pose_ops import bearing_face

#: A face whose outward gap to the outline is at most this is "at the edge":
#: nothing can be seated between the row and the board's boundary, so the
#: pads on it can only be reached along the edge strip or from under the
#: part. 2 mm is two 0402s or one lane pair with clearance; a judgement, and
#: stated as one.
EDGE_MM = 2.0

#: A coordinate step below this is a STAGGER inside a row, not a lattice
#: pitch. Measured on esp_prog U2 (SOT-89): its middle pin sits 0.1 mm off
#: its two row mates, and taking that as the pitch made the on-face
#: tolerance 0.05 mm, so the term read the regulator as 2 pins of 3 facing
#: the edge where the layout shows all three. No real land pattern has a
#: pitch under 0.2 mm (0.35 is the finest QFN in the corpus).
MIN_PITCH_STEP = 0.2

#: A part counts when it has at least this many CONNECTED pads: a two-pad
#: passive has no row to face anything with. ONE constant for the term
#: (`placement_score.EDGE_FACING_MIN_PADS`), the rule
#: (`floorplan.PINS_TO_EDGE_MIN_PADS`) and the seeder's opt-in tie-break
#: (`seeder._facing_rank`), so what the search prefers is what the term
#: then reports -- three spellings of `3` drifted apart in review.
MIN_PADS = 3

Point = Tuple[float, float]


def pitch_of(points: Iterable[Point]) -> float:
    """The minimum spacing of a point lattice along either axis, in mm,
    ignoring steps under `MIN_PITCH_STEP`; inf when no axis has two distinct
    coordinates that far apart. The same reading `escape.pad_pitch` takes off
    a footprint's local pads, here on any coordinates so the seeder and the
    term measure one pitch."""
    pts = list(points)
    xs = sorted({round(x, 3) for x, _ in pts})
    ys = sorted({round(y, 3) for _, y in pts})

    def step(v):
        return min((b - a for a, b in zip(v, v[1:])
                    if b - a >= MIN_PITCH_STEP - INTERIOR_EPS),
                   default=float('inf'))
    return min(step(xs), step(ys))


def _seg_dist(p: Point, seg: Tuple[float, float, float, float]) -> float:
    """Distance from a point to an axis-aligned segment."""
    x, y = p
    x1, y1, x2, y2 = seg
    if abs(x1 - x2) <= INTERIOR_EPS:          # vertical
        dy = 0.0 if min(y1, y2) <= y <= max(y1, y2) else min(abs(y - y1), abs(y - y2))
        return max(abs(x - x1), dy)
    dx = 0.0 if min(x1, x2) <= x <= max(x1, x2) else min(abs(x - x1), abs(x - x2))
    return max(abs(y - y1), dx)


def count_pads_to_edge(pads: Sequence[Tuple[float, float, int]],
                       rect: Tuple[float, float, float, float],
                       bounds: Tuple[float, float, float, float],
                       partners_by_net: Dict[int, List[Point]],
                       centre: Point, *, edge_mm: float = EDGE_MM,
                       pitch: float) -> dict:
    """The count for ONE part. See the module docstring for the definition.

    `pads` are `(x, y, net_id)` for the part's connected pads in board
    coordinates; `rect` the rectangle the term measures faces on (the pad
    centre bbox, `escape._part_rect`); `bounds` the outline bounds;
    `partners_by_net` maps a net id to the pads of OTHER parts on it;
    `centre` is the point `bearing_face` looks out from. Returns
    `{'pads', 'to_edge', 'faces': {face: n}, 'gaps': {face: mm}}`.
    """
    x0, y0, x1, y1 = bounds
    minx, miny, maxx, maxy = rect
    gaps = {'north': miny - y0, 'south': y1 - maxy,
            'west': minx - x0, 'east': x1 - maxx}
    edge_faces = [f for f in FACES if gaps[f] <= edge_mm + INTERIOR_EPS]
    tol = max((pitch / 2.0) if pitch != float('inf') else 0.0, INTERIOR_EPS)
    faces_hit: Dict[str, int] = {}
    to_edge = 0
    for x, y, nid in pads:
        counted = False
        for f in edge_faces:
            if _seg_dist((x, y), _face_geometry(rect, f)) > tol:
                continue
            beyond = any(bearing_face(centre, p) == f
                         for p in partners_by_net.get(nid, ()))
            if beyond:
                continue
            faces_hit[f] = faces_hit.get(f, 0) + 1
            counted = True
        if counted:
            to_edge += 1
    return {'pads': len(pads), 'to_edge': to_edge, 'faces': faces_hit,
            'gaps': {f: round(gaps[f], 4) for f in FACES},
            'edge_mm': edge_mm}


def part_inputs(pcb_data, ref: str):
    """`(pads, rect, partners_by_net, centre, pitch)` for `count_pads_to_edge`,
    read off a parsed board; None when the part has fewer than one connected
    pad. Partners are every pad of every OTHER footprint on the same net."""
    fp = (pcb_data.footprints or {}).get(ref)
    if fp is None:
        return None
    pads = [(float(p.global_x), float(p.global_y), int(getattr(p, 'net_id', 0) or 0))
            for p in (fp.pads or ())
            if (getattr(p, 'net_id', 0) or 0) > 0]
    if not pads:
        return None
    all_pads = [(float(p.global_x), float(p.global_y)) for p in (fp.pads or ())]
    xs = [x for x, _ in all_pads]
    ys = [y for _, y in all_pads]
    rect = (min(xs), min(ys), max(xs), max(ys))
    centre = ((rect[0] + rect[2]) / 2.0, (rect[1] + rect[3]) / 2.0)
    nets = {nid for _, _, nid in pads}
    partners: Dict[int, List[Point]] = {}
    for other, ofp in (pcb_data.footprints or {}).items():
        if other == ref:
            continue
        for p in (ofp.pads or ()):
            nid = getattr(p, 'net_id', 0) or 0
            if nid in nets:
                partners.setdefault(nid, []).append(
                    (float(p.global_x), float(p.global_y)))
    return pads, rect, partners, centre, pitch_of(all_pads)

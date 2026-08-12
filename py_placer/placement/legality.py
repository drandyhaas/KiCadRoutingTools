"""Placement legality geometry: board side, true board outline, courtyard rects.

The single home for the geometric predicates that decide whether a placement is
LEGAL, as opposed to merely cheap. Both placement engines consume it:

- ``placement/quench.py`` -- the perturbative nudge/rotate/swap optimizer
- ``placement/fanout_clearance.py`` -- the decoupling-cap repair pass

and so can a placement grader: `placement_overlap_area` (total courtyard overlap
area) and `placement_out_of_board` are the two legality metrics the placement
scorecard needs (#456, feeding #411/#110), computed by the same code the
optimizers gate on rather than by a second implementation that can disagree.

Two things here are easy to get wrong and are therefore centralized:

**Board side.** Parts on opposite sides of the board overlap in XY without
overlapping in copper -- a back-side decoupling cap legitimately sits under a
front-side BGA. A side-blind clearance check calls that a collision, and since
only CANDIDATE poses are validated (never the incumbent), every candidate near
such a part is rejected and the part freezes in place. But a THROUGH-HOLE part
does occupy both sides: its leads pass through the board. So a part occupies its
own side with its full courtyard, and the opposite side only with the bounding
box of its drilled pads (`sides_occupied` / `rect_on` / `pair_min_gap`).

**The board is not its bounding box.** `board_bounds` is an axis-aligned bbox
(kicad_parser), so an inset of it is blind to an L-shaped outline, a notch, or an
interior cutout -- a part gets nudged into the hole. `BoardOutlineGate` measures
against the real Edge.Cuts rings, with the three-level short-circuit that makes
it affordable: board-level opt-out when the outline IS its bbox, a cached
per-part reachable-disk prune, then the exact ring test.
"""
from __future__ import annotations

import math
from typing import Dict, Iterable, List, NamedTuple, Optional, Sequence, Tuple

EPS = 1e-6

# Run-6: a courtyard covering at least this fraction of the board bbox is a
# CONTAINER (a module-outline footprint hosting the design -- a frame, not a
# body). Calibration: rp2350_fpga_eensy U8 = 1.13x board area; the largest
# non-container anywhere in the 33-board corpus = 0.29 (sonde_u J1). Pairs
# with a container member are exempt from the courtyard channels everywhere;
# the pad layer applies in full.
CONTAINER_RATIO = 0.5

BOTH_SIDES = frozenset(('F', 'B'))


# --- rect primitives ---------------------------------------------------------
# rotate_local_bounds and rect_gap lived in duplicate in quench.py and
# fanout_clearance.py; both now re-export from here.

def rotate_local_bounds(lmin_x, lmin_y, lmax_x, lmax_y, rotation):
    """Rotate a local bounding box by the footprint rotation (KiCad sign)."""
    rot = rotation % 360
    if abs(rot) < 0.01:
        return lmin_x, lmin_y, lmax_x, lmax_y
    angle = math.radians(-rot)
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    corners = [(lmin_x, lmin_y), (lmax_x, lmin_y),
               (lmin_x, lmax_y), (lmax_x, lmax_y)]
    xs = [x * cos_a - y * sin_a for x, y in corners]
    ys = [x * sin_a + y * cos_a for x, y in corners]
    return min(xs), min(ys), max(xs), max(ys)


def rect_gap(a, b):
    """Smallest axis-aligned gap between two rects (negative if overlapping)."""
    dx = max(a[0] - b[2], b[0] - a[2])
    dy = max(a[1] - b[3], b[1] - a[3])
    if dx < 0 and dy < 0:
        return max(dx, dy)  # overlap amount (negative)
    return math.hypot(max(dx, 0), max(dy, 0)) if (dx > 0 and dy > 0) else max(dx, dy)


def rect_overlap_shortfall(a, b, clearance):
    """Clearance shortfall between two rects (0 when they are clear)."""
    return max(0.0, clearance - rect_gap(a, b))


def rect_overlap_area(a, b):
    """Area of the intersection of two rects (0 when disjoint).

    The OO ("total component overlap area") legality metric's primitive: unlike
    rect_gap this is a real area, so it stays comparable across boards.
    """
    w = min(a[2], b[2]) - max(a[0], b[0])
    if w <= 0.0:
        return 0.0
    h = min(a[3], b[3]) - max(a[1], b[1])
    return w * h if h > 0.0 else 0.0


def rect_area(rect):
    return max(0.0, rect[2] - rect[0]) * max(0.0, rect[3] - rect[1])


def point_to_seg_dist(px, py, x1, y1, x2, y2):
    """Distance from a point to a line segment."""
    dx, dy = x2 - x1, y2 - y1
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L2))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


def ring_is_rect(ring):
    """True when an outline ring is an axis-aligned rectangle equal to its own
    bbox (shoelace area == bbox area) -- then the bbox inset test is exact and
    the per-candidate ring checks can be skipped entirely (#370 B2)."""
    xs = [p[0] for p in ring]
    ys = [p[1] for p in ring]
    bbox_area = (max(xs) - min(xs)) * (max(ys) - min(ys))
    if bbox_area <= 0:
        return False
    a = 0.0
    n = len(ring)
    for i in range(n):
        x1, y1 = ring[i]
        x2, y2 = ring[(i + 1) % n]
        a += x1 * y2 - x2 * y1
    return abs(abs(a) / 2.0 - bbox_area) <= max(1e-6, 1e-3 * bbox_area)


# --- board side model --------------------------------------------------------

def footprint_side(fp) -> str:
    """'F' or 'B' from a footprint's layer. Anything not B.* reads as front,
    matching how the rest of the package resolves side from `fp.layer`."""
    return 'B' if (getattr(fp, 'layer', '') or '').startswith('B') else 'F'


def footprint_has_through_pads(fp) -> bool:
    """True when any pad is drilled, so the part's leads occupy BOTH sides.

    Deliberately plain `drill > 0`, not `pad_is_plated_through`: the question
    here is physical body/lead obstruction, not electrical layer-tying, so an
    unplated hole blocks the far side exactly as a plated one does.
    """
    return any((getattr(p, 'drill', 0) or 0) > 0 for p in (fp.pads or []))


def sides_occupied(side: str, has_tht: bool) -> frozenset:
    """The board sides a part physically obstructs."""
    return BOTH_SIDES if has_tht else frozenset((side,))


def rect_on(side_wanted: str, own_side: str, courtyard_rect, tht_rect):
    """The rect a part presents on `side_wanted`.

    Its own side gets the full courtyard; the opposite side gets only the
    drilled-pad bounding box (None when the part has no drilled pads, i.e. it
    does not reach that side at all).
    """
    return courtyard_rect if side_wanted == own_side else tht_rect


def pair_min_gap(a_sides, a_side, a_rect, a_tht,
                 b_sides, b_side, b_rect, b_tht) -> Optional[float]:
    """Smallest rect gap between two parts over the sides they SHARE.

    None when they share no side -- then they cannot interact at all and every
    consumer (hard clearance, halo penalty, overlap grader) must skip the pair.
    On a single-sided board the shared set is always the one side and this
    reduces to plain courtyard-vs-courtyard, which is what keeps the existing
    behaviour (and the existing tests) unchanged.
    """
    best = None
    for s in (a_sides & b_sides):
        ra = rect_on(s, a_side, a_rect, a_tht)
        rb = rect_on(s, b_side, b_rect, b_tht)
        if ra is None or rb is None:
            continue
        g = rect_gap(ra, rb)
        if best is None or g < best:
            best = g
    return best


def pair_overlap_area(a_sides, a_side, a_rect, a_tht,
                      b_sides, b_side, b_rect, b_tht) -> float:
    """Largest per-shared-side overlap area between two parts (0 if none).

    Max rather than sum: on a THT/THT pair the same physical interference shows
    up on both sides, and counting it twice would double-charge one overlap.
    """
    worst = 0.0
    for s in (a_sides & b_sides):
        ra = rect_on(s, a_side, a_rect, a_tht)
        rb = rect_on(s, b_side, b_rect, b_tht)
        if ra is None or rb is None:
            continue
        worst = max(worst, rect_overlap_area(ra, rb))
    return worst


# --- board outline gate ------------------------------------------------------

class BoardOutlineGate:
    """Board-containment tests against the REAL Edge.Cuts outline (#370 B2).

    Callers keep using the cheap `usable` bbox inset as a first filter and reach
    for this only when it is not exact. Three levels of short-circuit:

    1. `active` is False when the board's outline IS its bounding box (one ring,
       rectangular, no cutouts) -- then the inset test is already exact, and it
       is also False when the parser found no usable ring at all, in which case
       the bbox is all we have and behaviour is unchanged.
    2. `edges_near(key, seed_rect, travel)` -- cached per key; a part that cannot
       reach any ring from its seed never pays for the exact test, and one that
       can gets the short list of edges to measure against rather than all of
       them. Pass that list back into the level-3 calls; skipping this prune is
       correct but roughly an order of magnitude slower in a candidate loop.
    3. `rect_blocked(rect)` / `rect_outside_amount(rect)` -- the exact tests.

    Caching note: a key's answer is computed from the pose and budget of the
    FIRST call, so a caller whose budget grows later must use a distinct key.
    Over-estimating `travel` is always safe (more edges, same verdict).
    """

    def __init__(self, board_info, margin: float):
        # Function-local import: check_drc pulls the DRC stack, and this module
        # is imported at the top of both placement engines. Same reason
        # fanout_clearance imported it locally.
        from check_drc import board_edge_geometry
        rings, outer, cutouts = board_edge_geometry(board_info)
        self.rings = rings
        self.outer = outer
        self.cutouts = cutouts
        self.margin = margin
        self.bounds = getattr(board_info, 'board_bounds', None)
        self.usable = None
        if self.bounds is not None:
            self.usable = (self.bounds[0] + margin, self.bounds[1] + margin,
                           self.bounds[2] - margin, self.bounds[3] - margin)
        self.active = bool(rings) and (
            bool(cutouts) or len(rings) > 1 or not ring_is_rect(rings[0]))
        self._near: Dict[object, list] = {}
        self._edges = None

    def edges(self):
        """Every ring edge as (x1, y1, x2, y2), built once."""
        if self._edges is None:
            out = []
            for ring in self.rings:
                n = len(ring)
                for i in range(n):
                    out.append((ring[i][0], ring[i][1],
                                ring[(i + 1) % n][0], ring[(i + 1) % n][1]))
            self._edges = out
        return self._edges

    # -- level 2: cached reachability prune
    def edges_near(self, key, seed_rect, travel: float, center=None) -> list:
        """Cached: the ring edges a part seeded at `seed_rect` could ever bring
        a pose within the edge margin of, given `travel` of displacement budget.

        Keyed on the SEED pose, so the reachable disk covers every pose the part
        can ever take and the answer holds for the whole run. Empty means the
        exact ring test can be skipped entirely; otherwise this is the short list
        the exact test should measure against instead of every ring, which is
        what keeps a dense candidate loop affordable (the segment-to-ring
        distances dominated a naive port).

        `center` is the part's pose ORIGIN, and callers whose parts can ROTATE
        must pass it. Measuring the span from the seed rect's own centre assumes
        the rect stays centred there, which is false for a courtyard that is
        off-centre from the footprint origin: a 90/180/270 pose swings it about
        the origin, and the reachable disk drawn around the seed centre can then
        miss an edge the part really can reach. Rotation preserves distance FROM
        THE ORIGIN, so the max origin-to-corner distance bounds every pose
        exactly. (Omitting it is safe only for parts that never rotate, or whose
        courtyard is origin-symmetric -- the fanout caps this prune came from.)
        """
        near = self._near.get(key)
        if near is None:
            if center is not None:
                cx, cy = center
                span = max(math.hypot(px - cx, py - cy)
                           for px in (seed_rect[0], seed_rect[2])
                           for py in (seed_rect[1], seed_rect[3]))
            else:
                cx = (seed_rect[0] + seed_rect[2]) / 2.0
                cy = (seed_rect[1] + seed_rect[3]) / 2.0
                span = math.hypot(seed_rect[2] - cx, seed_rect[3] - cy)
            reach = travel + span + self.margin + EPS
            # An edge farther than reach from that centre cannot come within
            # margin of any reachable pose: every point of a reachable rect lies
            # within travel + span of it.
            near = [e for e in self.edges()
                    if point_to_seg_dist(cx, cy, *e) <= reach]
            self._near[key] = near
        return near

    def may_reach(self, key, seed_rect, travel: float, center=None) -> bool:
        return bool(self.edges_near(key, seed_rect, travel, center))

    # -- level 3: the exact tests
    def rect_blocked(self, rect, edges=None) -> bool:
        """True when a rect leaves the real outline, enters a cutout, or comes
        within the edge margin of either.

        `edges` restricts the distance test to a pre-filtered edge list from
        `edges_near`; omitted, every ring edge is measured.
        """
        from check_drc import _point_on_board, _seg_seg_dist_coords
        x0, y0, x1, y1 = rect
        for (px, py) in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
            if not _point_on_board(px, py, self.outer, self.cutouts):
                return True
        for (ax, ay, bx, by) in ((x0, y0, x1, y0), (x1, y0, x1, y1),
                                 (x1, y1, x0, y1), (x0, y1, x0, y0)):
            for (ex1, ey1, ex2, ey2) in (self.edges() if edges is None else edges):
                if _seg_seg_dist_coords(ax, ay, bx, by,
                                        ex1, ey1, ex2, ey2) < self.margin:
                    return True
        # A cutout ring FULLY INSIDE the rect evades both tests above.
        for cut in self.cutouts:
            cx, cy = cut[0]
            if x0 <= cx <= x1 and y0 <= cy <= y1:
                return True
        return False

    def bbox_outside_amount(self, rect) -> float:
        """Total per-axis overshoot of the `usable` bbox inset (0 when inside).

        This is the whole board test on a rectangular board, and the fallback
        when the parser produced no outline rings.
        """
        if self.usable is None:
            return 0.0
        u = self.usable
        return (max(0.0, u[0] - rect[0]) + max(0.0, u[1] - rect[1])
                + max(0.0, rect[2] - u[2]) + max(0.0, rect[3] - u[3]))

    def rect_outside_amount(self, rect, exact: bool = True, edges=None) -> float:
        """Magnitude of a rect's board-boundary violation; 0 iff fully legal.

        Zero exactly when the bbox inset holds AND `rect_blocked` is False, so
        it can drive a "may not worsen" acceptance rule and a grader from one
        definition. It is a distance-like magnitude, not an area -- see
        `out_of_board_area` for an area.

        `exact=False` skips the ring terms, for a caller that has already
        established via `may_reach` that this rect cannot come near a ring. The
        ring terms are ~100x the cost of the bbox term, and in a hot candidate
        loop most parts are nowhere near an edge.
        """
        amt = self.bbox_outside_amount(rect)
        if not (self.active and exact):
            return amt
        from check_drc import _point_on_board, _point_to_rings_distance, \
            _seg_seg_dist_coords
        use = self.edges() if edges is None else edges
        x0, y0, x1, y1 = rect
        for (px, py) in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
            if not _point_on_board(px, py, self.outer, self.cutouts):
                # How far the stray corner must come back, floored at the margin
                # so an off-board corner always outweighs a mere margin graze.
                amt += max(self.margin,
                           _point_to_rings_distance(px, py, self.rings))
        for (ax, ay, bx, by) in ((x0, y0, x1, y0), (x1, y0, x1, y1),
                                 (x1, y1, x0, y1), (x0, y1, x0, y0)):
            d = min((_seg_seg_dist_coords(ax, ay, bx, by, *e) for e in use),
                    default=float('inf'))
            if d < self.margin:
                amt += self.margin - d
        for cut in self.cutouts:
            cx, cy = cut[0]
            if x0 <= cx <= x1 and y0 <= cy <= y1:
                amt += self.margin
        return amt

    def edge_clearance(self, rect, edges=None) -> float:
        """Distance from a rect to the nearest board edge (rings when we have
        them, else the raw bbox). Negative is not reported -- a rect straddling
        an edge reads as 0. Drives the SOFT edge-margin penalty, which otherwise
        measures to the bounding box and ignores notches entirely.

        `edges` restricts the measurement to a pre-filtered list; a caller that
        knows the rect is farther than it cares about from every edge should not
        call this at all.
        """
        if self.active:
            from check_drc import _seg_seg_dist_coords
            use = self.edges() if edges is None else edges
            x0, y0, x1, y1 = rect
            best = float('inf')
            for (ax, ay, bx, by) in ((x0, y0, x1, y0), (x1, y0, x1, y1),
                                     (x1, y1, x0, y1), (x0, y1, x0, y0)):
                for e in use:
                    d = _seg_seg_dist_coords(ax, ay, bx, by, *e)
                    if d < best:
                        best = d
            return max(0.0, best)
        if self.bounds is None:
            return float('inf')
        b = self.bounds
        return max(0.0, min(rect[0] - b[0], rect[1] - b[1],
                            b[2] - rect[2], b[3] - rect[3]))

    def out_of_board_area(self, rect) -> float:
        """Area of `rect` outside the usable bbox inset (exact on a rectangular
        board; a lower bound on a notched one, where the rings also bite)."""
        if self.usable is None:
            return 0.0
        inside = rect_overlap_area(rect, self.usable)
        return max(0.0, rect_area(rect) - inside)


# --- graders (#456 comment: shared with the #411/#110 placement scorecard) ----

class GradedPart(NamedTuple):
    """One placed part, as the graders want it: absolute rects, resolved side."""
    ref: str
    side: str                       # 'F' or 'B'
    rect: Tuple[float, float, float, float]      # courtyard, own side
    tht_rect: Optional[Tuple[float, float, float, float]] = None
    has_tht: bool = False

    @property
    def sides(self) -> frozenset:
        return sides_occupied(self.side, self.has_tht)


class OutOfBoard(NamedTuple):
    count: int          # parts with any board-boundary violation
    amount: float       # summed rect_outside_amount (distance-like)
    area: float         # summed out_of_board_area (mm^2, bbox-exact)


def placement_overlap_area(parts: Sequence[GradedPart]) -> float:
    """OO: total pairwise courtyard overlap area, side-aware (mm^2).

    Zero on a legal placement. Cross-side pairs contribute nothing unless a
    through-hole part's lead field reaches the other side.
    """
    total = 0.0
    items = list(parts)
    for i, a in enumerate(items):
        for b in items[i + 1:]:
            total += pair_overlap_area(a.sides, a.side, a.rect, a.tht_rect,
                                       b.sides, b.side, b.rect, b.tht_rect)
    return total


def placement_out_of_board(parts: Sequence[GradedPart], board_info,
                           margin: float) -> OutOfBoard:
    """OoB: how many parts leave the real board outline, and by how much."""
    gate = BoardOutlineGate(board_info, margin)
    count = 0
    amount = 0.0
    area = 0.0
    for p in parts:
        amt = gate.rect_outside_amount(p.rect)
        if amt > EPS:
            count += 1
            amount += amt
            area += gate.out_of_board_area(p.rect)
    return OutOfBoard(count=count, amount=amount, area=area)


# --- body-overlap channel (run-6: the assembly gate) --------------------------
# Run 5 shipped a board with two 0402s STACKED (C14 on R14): their pads
# physically intersected by 0.25 mm^2 on the same side, and every gate in the
# chain was blind by construction -- pair_shortfall deliberately skips same-net
# pairs (it is a SHORT detector), the aggregate overlap_area has a legitimate
# nonzero floor on human boards (mount-hole courtyards under connector shells),
# and nothing anywhere reported overlap as PAIRS. This channel is the fix,
# CALIBRATED on the 33-board corpus (measured, not assumed):
#
#   pad_intersection  cross-footprint pad rects INTERSECTING on a shared side,
#                     ANY net (same-net included -- two different footprints'
#                     copper in the same space is an assembly impossibility
#                     regardless of net). Exact-verified. Corpus census: ZERO
#                     pairs on all 33 healthy boards. THE blocking channel;
#                     never waivable.
#   fab               F/B.Fab BODY-outline intersection pairs (no courtyard
#                     margin). Corpus census: 4 pairs, all by-design
#                     (fiducials under connector bodies; an HDMI shell kissing
#                     neighbors) -- one of which no class evidence can waive
#                     (ulx3s GPDI1/J5, a custom-lib receptacle) -- so this is
#                     an ADVISORY channel: reported with waiver labels, a fix
#                     target for the placement loop, never blocking.
#   courtyard         side-aware courtyard intersection pairs. Corpus census:
#                     6 real boards carry them (shells over low parts, tight
#                     watch layouts) -- ADVISORY, same policy as fab. This is
#                     the run-4 lesson holding: courtyards carry their own
#                     margin, and a courtyard kiss is not proof of a defect.
#
# Waiver ladder for the advisory channels (never for pad_intersection):
# member classifies mount_hole/fiducial/testpoint (pose-independent part_class
# KB), member classifies an edge class (shells overhang neighbors by design),
# or the pair is declared in the intent's overlap_waivers (authored only).

class BodyOverlapPair(NamedTuple):
    a: str
    b: str
    kind: str            # 'pad_intersection' | 'courtyard'
    area_mm2: float      # intersection area on the worst shared side
    side: str            # the shared side it occurs on ('F'/'B'; worst side)
    waived: bool
    waiver: str          # 'mount_hole_class' | 'intent_declared' | ''
    # Run-7 filed a report saying this channel false-positives on SAME-NET
    # contact, because DRC exempts it. Re-measured: the disputed pair really
    # was same-net (a 0402's whole pad, 0.83mm2, inside a connector pad on the
    # same GND) -- and it is still a defect. DRC exempts same-net because DRC
    # grades electrical clearance; this channel grades whether two parts can
    # both be built, and two parts cannot occupy the same copper whatever the
    # net. That is precisely the case the channel was created for: the pair of
    # stacked 0402s it was calibrated on were same-net too. Finding REFUTED,
    # policy unchanged.
    #
    # What the report DID expose is that a pair says nothing about which pads
    # touched, so a reader had to re-derive it to judge. `shorts` names up to
    # three DIFFERENT-net pad intersections when they exist -- those are a
    # short on top of the assembly defect, and they should never have to be
    # re-derived. Empty is the common case and means every verified
    # intersection was same-net.
    shorts: Tuple[str, ...] = ()      # e.g. ('C1.1:+3V3 <-> U1.8:/USB_P',)
    # Either member carries KiCad's own (locked yes). A locked part's pose is
    # a decision somebody made; copper landing on it is never dispositionable
    # by a placement search (run-8 E6).
    locked_ref: str = ''


def body_overlap_pairs(parts: Sequence[GradedPart]) -> List[BodyOverlapPair]:
    """Per-PAIR side-aware courtyard intersections (kind='courtyard').

    The pair decomposition of `placement_overlap_area`: same side rules
    (cross-side pairs contribute nothing unless a THT lead field reaches the
    far side), but reported as pairs so a gate can charge the DEFECT and a
    waiver can bless the by-design cases. Waivers are the CALLER's job --
    this function reports raw geometry (waived=False throughout).
    """
    out: List[BodyOverlapPair] = []
    items = list(parts)
    for i, a in enumerate(items):
        for b in items[i + 1:]:
            worst = 0.0
            worst_side = ''
            for s in (a.sides & b.sides):
                ra = rect_on(s, a.side, a.rect, a.tht_rect)
                rb = rect_on(s, b.side, b.rect, b.tht_rect)
                if ra is None or rb is None:
                    continue
                area = rect_overlap_area(ra, rb)
                if area > worst:
                    worst = area
                    worst_side = s
            if worst > EPS:
                out.append(BodyOverlapPair(
                    a=min(a.ref, b.ref), b=max(a.ref, b.ref),
                    kind='courtyard', area_mm2=round(worst, 4),
                    side=worst_side, waived=False, waiver=''))
    out.sort(key=lambda p: (-p.area_mm2, p.a, p.b))
    return out


def graded_parts_from_file(pcb_data, pcb_file: Optional[str] = None
                           ) -> List[GradedPart]:
    """`GradedPart` records at the FILE's own poses.

    Same geometry rules as the quench state (#456: one definition of legal):
    courtyard for the part's own side via the text parser, pad-bbox fallback
    when the footprint draws none, drilled-pad box on the far side. Needs the
    board FILE for courtyards (the text parser reads it); falls back to
    `pcb_data.source_path`, then to pad bboxes everywhere.
    """
    from placement.parser import courtyard_for_side, extract_courtyard_sides
    from placement.quench import _through_pad_bounds_local
    from placement.utility import compute_footprint_bbox_local

    path = pcb_file or getattr(pcb_data, 'source_path', None)
    sides: Dict[str, Dict] = {}
    if path:
        try:
            sides = extract_courtyard_sides(path)
        except Exception:
            sides = {}
    out: List[GradedPart] = []
    for ref, fp in sorted((pcb_data.footprints or {}).items()):
        own = footprint_side(fp)
        local = None
        if sides.get(ref):
            local = courtyard_for_side(sides[ref], own)
        if local is None:
            try:
                local = compute_footprint_bbox_local(fp)
            except Exception:
                local = None
        if local is None:
            continue
        rot = fp.rotation or 0.0
        lx0, ly0, lx1, ly1 = rotate_local_bounds(*local, rot)
        rect = (fp.x + lx0, fp.y + ly0, fp.x + lx1, fp.y + ly1)
        tht = None
        has_tht = footprint_has_through_pads(fp)
        if has_tht:
            tlb = _through_pad_bounds_local(fp)
            if tlb is not None:
                tx0, ty0, tx1, ty1 = rotate_local_bounds(*tlb, rot)
                tht = (fp.x + tx0, fp.y + ty0, fp.x + tx1, fp.y + ty1)
        out.append(GradedPart(ref=ref, side=own, rect=rect,
                              tht_rect=tht, has_tht=has_tht))
    return out


def grade_body_overlap(pcb_data, clearance: float,
                       intent_waivers: Sequence[Sequence[str]] = (),
                       pcb_file: Optional[str] = None) -> Dict[str, object]:
    """Board-level ASSEMBLY audit at the file's own poses.

    Returns {'blocking': int, 'advisory': int, 'waived': int,
    'pairs': [BodyOverlapPair...], 'blocking_pairs': [...],
    'advisory_pairs': [...]}. `blocking` counts pad_intersection pairs only
    (the corpus-calibrated hard channel); `advisory` counts UNWAIVED fab and
    courtyard pairs -- fix targets for the placement loop, dispositioned by
    the boundary verifier, never a gate by themselves (see the module
    comment's corpus census for why).
    """
    from placement.part_class import classify_part

    fps = pcb_data.footprints or {}
    waiver_sets = {frozenset(p) for p in intent_waivers if len(p) == 2}
    _MARKER = ('mount_hole', 'fiducial', 'testpoint')
    _EDGE = ('edge_receptacle', 'edge_actuator')

    def _class_of(ref: str) -> Optional[str]:
        fp = fps.get(ref)
        if fp is None:
            return None
        try:
            return classify_part(fp, ref).name
        except Exception:
            return None

    _containers: set = set()
    bb = getattr(getattr(pcb_data, 'board_info', None), 'board_bounds', None)
    if bb:
        _barea = max(1e-9, (bb[2] - bb[0]) * (bb[3] - bb[1]))
        for g in graded_parts_from_file(pcb_data, pcb_file):
            if rect_area(g.rect) >= CONTAINER_RATIO * _barea:
                _containers.add(g.ref)

    def _waiver_for(a: str, b: str) -> str:
        if a in _containers or b in _containers:
            return 'container_class'
        ca, cb = _class_of(a), _class_of(b)
        if ca in _MARKER or cb in _MARKER:
            return 'marker_class'
        if ca in _EDGE or cb in _EDGE:
            return 'edge_class'
        if frozenset((a, b)) in waiver_sets:
            return 'intent_declared'
        return ''

    pairs: List[BodyOverlapPair] = []

    # -- courtyard channel (advisory) -----------------------------------------
    for p in body_overlap_pairs(graded_parts_from_file(pcb_data, pcb_file)):
        waiver = _waiver_for(p.a, p.b)
        pairs.append(p._replace(waived=bool(waiver), waiver=waiver))

    # -- fab BODY channel (advisory) ------------------------------------------
    path = pcb_file or getattr(pcb_data, 'source_path', None)
    if path:
        try:
            from placement.parser import extract_fab_sides
            fab = extract_fab_sides(path)
        except Exception:
            fab = {}
        fab_parts = []
        for ref, fp in sorted(fps.items()):
            sides = fab.get(ref)
            if not sides:
                continue
            own = footprint_side(fp)
            lb = sides.get(own) or next(iter(sides.values()))
            rot = fp.rotation or 0.0
            x0, y0, x1, y1 = rotate_local_bounds(*lb, rot)
            fab_parts.append((ref, own,
                              (fp.x + x0, fp.y + y0, fp.x + x1, fp.y + y1)))
        for i, (ra, sa, rca) in enumerate(fab_parts):
            for rb, sb, rcb in fab_parts[i + 1:]:
                if sa != sb:
                    continue
                ov = rect_overlap_area(rca, rcb)
                if ov > EPS:
                    waiver = _waiver_for(ra, rb)
                    pairs.append(BodyOverlapPair(
                        a=min(ra, rb), b=max(ra, rb), kind='fab',
                        area_mm2=round(ov, 4), side=sa,
                        waived=bool(waiver), waiver=waiver))

    # -- pad_intersection channel (never waivable) ----------------------------
    # AABB broad phase in the gate currency, then exact re-verification at
    # clearance 0 (an intersection is a clearance-0 violation) so round or
    # rotated pads produce no bbox phantoms in a BLOCKING claim.
    parts = build_part_pads(fps, clearance)
    pads_by_ref = {ref: [p for p in fp.pads] for ref, fp in fps.items()}
    routing_layers = list(getattr(pcb_data.board_info, 'copper_layers', [])
                          or [])
    check_exact = None
    try:
        from check_drc import check_pad_pad_overlap
        check_exact = check_pad_pad_overlap
    except Exception:
        check_exact = None
    cell = 4.0
    grid: Dict[Tuple[int, int], set] = {}
    # KiCad's own (locked yes) stamps, for the E6 channel below. Best-effort:
    # the file is optional here, and a missing or unreadable one simply means
    # no pair is marked locked.
    locked_refs = set()
    if pcb_file:
        try:
            from .parser import extract_locked_refs
            locked_refs = set(extract_locked_refs(pcb_file) or ())
        except Exception:
            locked_refs = set()

    def _pad_label(pads, idx):
        try:
            return pads[idx].pad_number or '?'
        except Exception:
            return '?'

    entries = {}
    for ref, pp in parts.items():
        fp = fps[ref]
        rects = pp.pad_rects(fp.x, fp.y, fp.rotation or 0.0)
        entries[ref] = rects
        ext = pp.extent(fp.x, fp.y, fp.rotation or 0.0)
        if ext is None:
            continue
        for gx in range(int(ext[0] // cell), int(ext[2] // cell) + 1):
            for gy in range(int(ext[1] // cell), int(ext[3] // cell) + 1):
                grid.setdefault((gx, gy), set()).add(ref)
    seen = set()
    court_keys = {(p.a, p.b) for p in pairs}
    for ref in sorted(parts):
        fp = fps[ref]
        pp = parts[ref]
        ext = pp.extent(fp.x, fp.y, fp.rotation or 0.0)
        if ext is None:
            continue
        near = set()
        for gx in range(int(ext[0] // cell), int(ext[2] // cell) + 1):
            for gy in range(int(ext[1] // cell), int(ext[3] // cell) + 1):
                near |= grid.get((gx, gy), set())
        near.discard(ref)
        for other in near:
            key = (ref, other) if ref <= other else (other, ref)
            if key in seen:
                continue
            seen.add(key)
            area = 0.0
            side = ''
            shorts = []
            for ai, (a0, a1, a2, a3, na, sa) in enumerate(entries[ref]):
                for bi, (b0, b1, b2, b3, nb, sb) in enumerate(entries[other]):
                    if not _sides_interact(sa, sb):
                        continue
                    ov = rect_overlap_area((a0, a1, a2, a3),
                                           (b0, b1, b2, b3))
                    if ov <= EPS:
                        continue
                    if check_exact is not None:
                        # check_pad_pad_overlap's perimeter distance clamps
                        # at 0 for interior points, so clearance-0 can never
                        # register an intersection. Ask at a tiny epsilon and
                        # require the FULL-epsilon shortfall: over >= eps
                        # <=> exact edge distance <= 0 <=> real intersection
                        # (merely-near pads at 0 < gap < eps read over < eps
                        # and are skipped).
                        eps = 1e-3
                        pa = _pad_with_copper(pads_by_ref[ref], ai, clearance)
                        pb = _pad_with_copper(pads_by_ref[other], bi,
                                              clearance)
                        if pa is not None and pb is not None:
                            hit, over, _pt = check_exact(
                                pa, pb, eps, routing_layers,
                                clearance_margin=0.0)
                            if not (hit and over >= eps - 1e-9):
                                continue
                    if ov > area:
                        area = ov
                        side = sa if sa in ('F', 'B') else ''
                    # Different-net copper touching is a SHORT on top of the
                    # assembly defect. Record one example per pair so a reader
                    # cannot mistake the pair for exempt same-net contact.
                    if na and nb and na != nb and len(shorts) < 3:
                        pa_num = _pad_label(pads_by_ref[ref], ai)
                        pb_num = _pad_label(pads_by_ref[other], bi)
                        shorts.append(f'{ref}.{pa_num}:{na} <-> '
                                      f'{other}.{pb_num}:{nb}')
            if area > EPS:
                locked_ref = ' '.join(sorted(
                    r for r in (key[0], key[1]) if r in locked_refs))
                pairs.append(BodyOverlapPair(
                    a=key[0], b=key[1], kind='pad_intersection',
                    area_mm2=round(area, 4), side=side,
                    waived=False, waiver='',
                    shorts=tuple(shorts), locked_ref=locked_ref))

    pairs.sort(key=lambda p: (p.waived, -p.area_mm2, p.a, p.b))
    blocking = [p for p in pairs if p.kind == 'pad_intersection']
    advisory = [p for p in pairs
                if p.kind != 'pad_intersection' and not p.waived]
    return {'blocking': len(blocking),
            'advisory': len(advisory),
            'waived': sum(1 for p in pairs if p.waived),
            'pairs': pairs,
            'blocking_pairs': blocking,
            'advisory_pairs': advisory,
            # E6: pairs where copper lands on a part KiCad marks (locked yes),
            # of ANY channel including waived ones. A locked pose is a decision
            # somebody made -- a mounting hole against an enclosure, a
            # connector against a panel cut-out -- so a placement search may
            # not resolve the contact by moving the other part somewhere it
            # likes better, and a waiver class chosen for unlocked parts does
            # not apply. Measured on a wrong-basin placement: fires there,
            # silent on the truth board and on every healthy corpus board.
            'locked_contact_pairs': [p for p in pairs if p.locked_ref]}


# --- pad + drill legality layer ----------------------------------------------
# The courtyard model above cannot see pad copper or drill holes: a part with
# no drawn courtyard falls back to its pad bbox with zero margin, the swap
# phase exchanges nets under identical courtyards, and an NPTH mounting hole
# is invisible on its own side. This layer adds the missing geometry in the
# GATE currency (rotation-inflated axis-aligned pad rects, the proven
# fanout_clearance._Cap pattern): conservative -- it can falsely reject, never
# falsely accept. Exact pad geometry (check_drc) is used only by
# `grade_pad_legality` for REPORTS, so summaries carry no AABB phantoms.
#
# Baseline semantics: engines gate moves per-PAIR against the SEED poses
# ("never worse than the board you were handed"), so pre-existing violations
# on dense or by-design-overhanging boards do not freeze the optimizer, while
# a NEW different-net pad intersection (a short) is never admitted.

class PairShortfall(NamedTuple):
    """Pad/hole interference between two parts, in the AABB gate currency."""
    pad: float          # summed different-net pad clearance shortfall (mm)
    pad_overlap: bool   # any different-net pad rects INTERSECT (a short)
    hole: float         # summed pad-copper-into-hole-keepout penetration (mm)
    stack: bool = False  # ANY-net cross-footprint pad rects INTERSECT --
    #                      two different parts' copper in the same space is an
    #                      assembly impossibility regardless of net (run-6:
    #                      the C14-on-R14 class the same-net skip is blind to)


ZERO_SHORTFALL = PairShortfall(0.0, False, 0.0, False)


class PartPads:
    """Pose-owning pad/hole model for one footprint.

    Offsets are captured in a seed-relative frame and re-derived per DELTA
    rotation (the `_Cap.pad_rects` pattern) because parser `pad.global_x` is
    stale after an in-memory footprint move -- this class owns the pose->pad
    transform. Half-extents fold `rect_rotation` into an axis-aligned bbox,
    exact for the axis-aligned pads that dominate placement and conservative
    for the rest.
    """

    __slots__ = ('ref', 'side', 'has_tht', 'seed_rot', 'pads_local',
                 'holes_local', 'n_pads', '_pad_cache', '_hole_cache',
                 '_ext_cache')

    def __init__(self, fp, clearance: float):
        # Local imports: check_drc pulls the whole DRC stack (see the
        # BoardOutlineGate note); kicad_parser is cheap but keeps the module's
        # import surface unchanged for existing consumers.
        from check_drc import _pad_has_no_copper
        from kicad_parser import pad_drill_circles
        import routing_defaults as defaults

        self.ref = fp.reference
        self.side = footprint_side(fp)
        self.has_tht = footprint_has_through_pads(fp)
        self.seed_rot = (fp.rotation or 0.0) % 360
        self.pads_local = []    # (off_x, off_y, half_x, half_y, net_id, pside)
        self.holes_local = []   # (off_x, off_y, radius) -- NPTH keepouts, inflated
        npth_grow = max(0.0, defaults.NPTH_TO_TRACK_CLEARANCE - clearance)
        for p in fp.pads:
            if _pad_has_no_copper(p):
                # Copper-less drilled pad (NPTH mounting hole): the DRILL still
                # removes copper closer than the NPTH-to-track floor. Inflation
                # matches fanout_clearance's foreign-pad keepouts.
                if (p.drill or 0) > 0:
                    for hx, hy, hd in pad_drill_circles(p):
                        self.holes_local.append(
                            (hx - fp.x, hy - fp.y, hd / 2.0 + npth_grow))
                continue
            copper = [l for l in p.layers if str(l).endswith('.Cu')]
            if not copper:
                continue    # paste/mask-only aperture
            through = (p.drill or 0) > 0
            pside = None if through else (
                'B' if any(str(l).startswith('B') for l in copper) else 'F')
            tilt = math.radians(getattr(p, 'rect_rotation', 0.0) or 0.0)
            c, s = abs(math.cos(tilt)), abs(math.sin(tilt))
            hx, hy = p.size_x / 2.0, p.size_y / 2.0
            self.pads_local.append((p.global_x - fp.x, p.global_y - fp.y,
                                    hx * c + hy * s, hx * s + hy * c,
                                    p.net_id, pside))
        self.n_pads = len(self.pads_local)
        self._pad_cache: Dict[float, list] = {}
        self._hole_cache: Dict[float, list] = {}
        self._ext_cache: Dict[float, Tuple[float, float, float, float]] = {}

    def _delta_key(self, rot: float) -> float:
        return round(((rot or 0.0) - self.seed_rot) % 360, 3)

    def _rotated(self, rot: float):
        key = self._delta_key(rot)
        cache = self._pad_cache.get(key)
        if cache is None:
            rad = math.radians(-key)
            c, s = math.cos(rad), math.sin(rad)
            swap = round(key) % 180 == 90
            cache = []
            for ox, oy, hx, hy, net, pside in self.pads_local:
                rx = ox * c - oy * s
                ry = ox * s + oy * c
                HX, HY = (hy, hx) if swap else (hx, hy)
                cache.append((rx, ry, HX, HY, net, pside))
            self._pad_cache[key] = cache
        return cache

    def pad_rects(self, x: float, y: float, rot: float):
        """[(x0, y0, x1, y1, net_id, pside)] at the given pose."""
        out = []
        for ox, oy, HX, HY, net, pside in self._rotated(rot):
            cx, cy = x + ox, y + oy
            out.append((cx - HX, cy - HY, cx + HX, cy + HY, net, pside))
        return out

    def hole_circles(self, x: float, y: float, rot: float):
        """[(cx, cy, radius)] NPTH keepout circles at the given pose."""
        key = self._delta_key(rot)
        cache = self._hole_cache.get(key)
        if cache is None:
            rad = math.radians(-key)
            c, s = math.cos(rad), math.sin(rad)
            cache = [(ox * c - oy * s, ox * s + oy * c, r)
                     for ox, oy, r in self.holes_local]
            self._hole_cache[key] = cache
        return [(x + ox, y + oy, r) for ox, oy, r in cache]

    def extent_local(self, rot: float):
        """Bbox over pads and holes at delta-rot, in the part frame (cached).
        None when the part has neither copper pads nor holes."""
        key = self._delta_key(rot)
        ext = self._ext_cache.get(key)
        if ext is None:
            xs0, ys0, xs1, ys1 = [], [], [], []
            for ox, oy, HX, HY, _n, _s in self._rotated(rot):
                xs0.append(ox - HX); ys0.append(oy - HY)
                xs1.append(ox + HX); ys1.append(oy + HY)
            for cx, cy, r in self.hole_circles(0.0, 0.0, rot):
                xs0.append(cx - r); ys0.append(cy - r)
                xs1.append(cx + r); ys1.append(cy + r)
            if not xs0:
                ext = ()
            else:
                ext = (min(xs0), min(ys0), max(xs1), max(ys1))
            self._ext_cache[key] = ext
        return ext or None

    def extent(self, x: float, y: float, rot: float):
        e = self.extent_local(rot)
        if e is None:
            return None
        return (x + e[0], y + e[1], x + e[2], y + e[3])


def build_part_pads(footprints: Dict[str, object],
                    clearance: float) -> Dict[str, 'PartPads']:
    """PartPads for every footprint that has any pad (copper or NPTH)."""
    out = {}
    for ref, fp in footprints.items():
        if not getattr(fp, 'pads', None):
            continue
        pp = PartPads(fp, clearance)
        if pp.n_pads or pp.holes_local:
            out[ref] = pp
    return out


def _sides_interact(a, b) -> bool:
    """Do two pad sides share copper? None = through (both sides)."""
    return a is None or b is None or a == b


def _circle_rect_penetration(cx, cy, r, rect) -> float:
    """How deep a circle penetrates a rect's keepout (0 when clear)."""
    dx = max(rect[0] - cx, 0.0, cx - rect[2])
    dy = max(rect[1] - cy, 0.0, cy - rect[3])
    return max(0.0, r - math.hypot(dx, dy))


# Above this many pad-pair tests the per-pad loop is skipped and the pair is
# gated on extents + its seed baseline alone (BGA-vs-BGA); the exact report
# still surfaces anything a candidate loop misses.
PAIR_TEST_CAP = 4096


class LegalityContext:
    """Pad/hole legality for one engine run.

    The engine supplies `pose_of(ref) -> (x, y, rot)` for CURRENT poses and
    `seed_of(ref) -> (x, y, rot)` for the input board's poses; baselines are
    computed lazily from seeds and never invalidated (seeds are constant for
    the run, unlike incumbents).
    """

    def __init__(self, part_pads: Dict[str, PartPads],
                 gate: Optional[BoardOutlineGate], clearance: float,
                 pose_of, seed_of):
        self.parts = part_pads
        self.gate = gate
        self.clearance = clearance
        self.pose_of = pose_of
        self.seed_of = seed_of
        self._baselines: Dict[Tuple[str, str], PairShortfall] = {}

    # -- pair measurement ------------------------------------------------------
    def pair_shortfall(self, a: str, b: str, pose_a=None,
                       pose_b=None) -> PairShortfall:
        pa = self.parts.get(a)
        pb = self.parts.get(b)
        if pa is None or pb is None:
            return ZERO_SHORTFALL
        xa, ya, ra = pose_a if pose_a is not None else self.pose_of(a)
        xb, yb, rb = pose_b if pose_b is not None else self.pose_of(b)
        ea = pa.extent(xa, ya, ra)
        eb = pb.extent(xb, yb, rb)
        if ea is None or eb is None:
            return ZERO_SHORTFALL
        if rect_gap(ea, eb) >= self.clearance - EPS:
            return ZERO_SHORTFALL
        if pa.n_pads * pb.n_pads > PAIR_TEST_CAP:
            # Extent-level verdict only: charge the extent shortfall as pad
            # shortfall so the baseline comparison still constrains the pair.
            g = rect_gap(ea, eb)
            return PairShortfall(max(0.0, self.clearance - g), g < 0.0, 0.0,
                                 g < 0.0)
        rects_a = pa.pad_rects(xa, ya, ra)
        rects_b = pb.pad_rects(xb, yb, rb)
        pad_short = 0.0
        overlap = False
        stack = False
        for a0, a1, a2, a3, na, sa in rects_a:
            for b0, b1, b2, b3, nb, sb in rects_b:
                if not _sides_interact(sa, sb):
                    continue
                g = rect_gap((a0, a1, a2, a3), (b0, b1, b2, b3))
                if g < 0.0:
                    # any-net physical intersection: the assembly channel,
                    # measured BEFORE the same-net skip below (which exists
                    # for the SHORT semantics only)
                    stack = True
                if na == nb and na > 0:
                    continue
                if g < self.clearance - EPS:
                    pad_short += self.clearance - g
                    if g < 0.0:
                        overlap = True
        hole = 0.0
        for cx, cy, r in pb.hole_circles(xb, yb, rb):
            for a0, a1, a2, a3, _na, _sa in rects_a:
                hole += _circle_rect_penetration(cx, cy, r, (a0, a1, a2, a3))
        for cx, cy, r in pa.hole_circles(xa, ya, ra):
            for b0, b1, b2, b3, _nb, _sb in rects_b:
                hole += _circle_rect_penetration(cx, cy, r, (b0, b1, b2, b3))
        return PairShortfall(pad_short, overlap, hole, stack)

    def seed_baseline(self, a: str, b: str) -> PairShortfall:
        key = (a, b) if a <= b else (b, a)
        base = self._baselines.get(key)
        if base is None:
            base = self.pair_shortfall(key[0], key[1],
                                       pose_a=self.seed_of(key[0]),
                                       pose_b=self.seed_of(key[1]))
            self._baselines[key] = base
        return base

    # -- the gate --------------------------------------------------------------
    def pads_ok(self, ref: str, x: float, y: float, rot: float,
                neighbors: Iterable[str], exclude=None) -> bool:
        """May `ref` take this pose? Per neighbor: no worse than the SEED
        baseline, and a NEW different-net pad intersection is never admitted."""
        if ref not in self.parts:
            return True
        pose = (x, y, rot)
        for nb in neighbors:
            if nb == ref or (exclude is not None and nb in exclude):
                continue
            cur = self.pair_shortfall(ref, nb, pose_a=pose)
            if cur is ZERO_SHORTFALL:
                continue
            base = self.seed_baseline(ref, nb)
            if cur.pad > base.pad + EPS:
                return False
            if cur.pad_overlap and not base.pad_overlap:
                return False
            # run-6: a NEW any-net pad stack (two footprints' copper in the
            # same space) is never admitted -- the same-net C14-on-R14 class
            # the short conjunct above cannot see
            if cur.stack and not base.stack:
                return False
            if cur.hole > base.hole + EPS:
                return False
        return True

    def pad_oob_amount(self, ref: str, x: float, y: float, rot: float,
                       exact: bool = True, edges=None) -> float:
        """Board-boundary violation of the part's PAD extent (catches pads off
        the board even when a bad courtyard rect is inside)."""
        pp = self.parts.get(ref)
        if pp is None or self.gate is None:
            return 0.0
        ext = pp.extent(x, y, rot)
        if ext is None:
            return 0.0
        return self.gate.rect_outside_amount(ext, exact=exact, edges=edges)


def format_oob_clause(report, limit: int = 6) -> str:
    """One line naming the off-board parts AND which measure produced them.

    Lives beside the measure so the CLIs that print it cannot drift -- hand
    copying a basis string into each is the Class-2 CLI drift CLAUDE.md warns
    about, and no parity gate covers a print.

    Sorted by AMOUNT, worst first. Sorting by ref and then truncating hides the
    offenders that matter: on one board the four mounting holes at 3.0mm sat
    behind five fiducials at 0.875mm. That is the run-4 census-cap defect,
    which this module already records elsewhere.
    """
    refs = list(report.get('oob_pad_refs') or [])
    if not refs:
        return ''
    refs.sort(key=lambda ra: -ra[1])
    shown = ', '.join('{} ({}mm)'.format(r, a) for r, a in refs[:limit])
    more = ' ... showing {} of {}'.format(limit, len(refs)) if len(refs) > limit else ''
    basis = (
        "    (part pad AABB -- pads PLUS NPTH drill circles, summed over the "
        "union rect's four sides -- against an outline inflated by the GRADING "
        "CLEARANCE. It moves when --clearance moves, so a hit here is not "
        "necessarily copper off the outline. The per-pad, margin-0 outline "
        "measure is render_placement's checklist.a_off_outline.pad_copper.)")
    return shown + more + "\n" + basis


def grade_pad_legality(pcb_data, clearance: float, exact: bool = True,
                       edge_margin: Optional[float] = None,
                       worst_n: int = 10) -> Dict[str, object]:
    """Board-level pad/hole legality audit at the FILE's own poses.

    AABB broad phase over all cross-footprint pad pairs; with `exact` (the
    default) every AABB hit is re-verified with check_drc's exact pad geometry
    so the report carries no bbox phantoms on round/rotated pads. Returns::

        {'pad_conflicts': int, 'pad_shortfall': mm, 'hole_conflicts': int,
         'oob_pad_count': int, 'oob_pad_amount': mm,
         'worst': [(refA, refB, mm), ...], 'exact': bool}

    Consumers: place_optimize / place_seed JSON summaries, the render
    legality overlay, and the reconstruct gate.
    """
    fps = pcb_data.footprints
    pads_by_ref = {ref: [p for p in fp.pads] for ref, fp in fps.items()}
    parts = build_part_pads(fps, clearance)
    routing_layers = list(getattr(pcb_data.board_info, 'copper_layers', []) or [])
    check_exact = None
    if exact:
        try:
            from check_drc import check_pad_pad_overlap
            check_exact = check_pad_pad_overlap
        except Exception:
            check_exact = None

    # cell hash over pad rects at file poses
    cell = 4.0
    grid: Dict[Tuple[int, int], set] = {}
    entries = {}
    for ref, pp in parts.items():
        fp = fps[ref]
        rects = pp.pad_rects(fp.x, fp.y, fp.rotation or 0.0)
        holes = pp.hole_circles(fp.x, fp.y, fp.rotation or 0.0)
        entries[ref] = (rects, holes)
        ext = pp.extent(fp.x, fp.y, fp.rotation or 0.0)
        if ext is None:
            continue
        for gx in range(int(ext[0] // cell), int(ext[2] // cell) + 1):
            for gy in range(int(ext[1] // cell), int(ext[3] // cell) + 1):
                grid.setdefault((gx, gy), set()).add(ref)

    def near_refs(ref):
        fp = fps[ref]
        pp = parts[ref]
        ext = pp.extent(fp.x, fp.y, fp.rotation or 0.0)
        if ext is None:
            return ()
        out = set()
        m = clearance + 0.5
        for gx in range(int((ext[0] - m) // cell), int((ext[2] + m) // cell) + 1):
            for gy in range(int((ext[1] - m) // cell), int((ext[3] + m) // cell) + 1):
                out |= grid.get((gx, gy), set())
        out.discard(ref)
        return out

    pad_conflicts = 0
    pad_shortfall = 0.0
    hole_conflicts = 0
    worst: List[Tuple[str, str, float]] = []
    seen_pairs = set()
    for ref in sorted(parts):
        rects_a, holes_a = entries[ref]
        for other in near_refs(ref):
            key = (ref, other) if ref <= other else (other, ref)
            if key in seen_pairs:
                continue
            seen_pairs.add(key)
            rects_b, holes_b = entries[other]
            pair_mm = 0.0
            pair_hit = False
            for ai, (a0, a1, a2, a3, na, sa) in enumerate(rects_a):
                for bi, (b0, b1, b2, b3, nb, sb) in enumerate(rects_b):
                    if na == nb and na > 0:
                        continue
                    if not _sides_interact(sa, sb):
                        continue
                    g = rect_gap((a0, a1, a2, a3), (b0, b1, b2, b3))
                    if g >= clearance - EPS:
                        continue
                    if check_exact is not None:
                        pa = _pad_with_copper(pads_by_ref[ref], ai, clearance)
                        pb = _pad_with_copper(pads_by_ref[other], bi, clearance)
                        if pa is not None and pb is not None:
                            hit, over, _pt = check_exact(pa, pb, clearance,
                                                         routing_layers,
                                                         clearance_margin=0.0)
                            if not hit:
                                continue
                            pair_mm += over
                            pair_hit = True
                            continue
                    pair_mm += clearance - g
                    pair_hit = True
            if pair_hit:
                pad_conflicts += 1
                pad_shortfall += pair_mm
                worst.append((key[0], key[1], round(pair_mm, 4)))
            hole_pen = 0.0
            for cx, cy, r in holes_b:
                for a0, a1, a2, a3, _na, _sa in rects_a:
                    hole_pen += _circle_rect_penetration(cx, cy, r,
                                                         (a0, a1, a2, a3))
            for cx, cy, r in holes_a:
                for b0, b1, b2, b3, _nb, _sb in rects_b:
                    hole_pen += _circle_rect_penetration(cx, cy, r,
                                                         (b0, b1, b2, b3))
            if hole_pen > EPS:
                hole_conflicts += 1

    oob_count = 0
    oob_amount = 0.0
    # NAME them. This returned a bare count, so the three CLIs that print it
    # ("N part(s) with pad copper off-board") could not say WHICH part -- one
    # run deduced the single ref by elimination.
    oob_refs = []
    board_info = getattr(pcb_data, 'board_info', None)
    if board_info is not None and getattr(board_info, 'board_bounds', None):
        gate = BoardOutlineGate(board_info,
                                edge_margin if edge_margin is not None
                                else clearance)
        for ref, pp in parts.items():
            fp = fps[ref]
            ext = pp.extent(fp.x, fp.y, fp.rotation or 0.0)
            if ext is None:
                continue
            amt = gate.rect_outside_amount(ext)
            if amt > EPS:
                oob_count += 1
                oob_amount += amt
                oob_refs.append([ref, round(amt, 4)])
    worst.sort(key=lambda t: -t[2])
    return {'pad_conflicts': pad_conflicts,
            'pad_shortfall': round(pad_shortfall, 4),
            'hole_conflicts': hole_conflicts,
            'oob_pad_count': oob_count,
            'oob_pad_amount': round(oob_amount, 4),
            'oob_pad_refs': sorted(oob_refs),
            # WHICH QUANTITY THIS IS. Three tools print "pad copper
            # off-board" for three different measurements. This one is the
            # part's pad AABB against an outline inflated by the GRADING
            # CLEARANCE -- so it moves when --clearance moves, and a rotated
            # part reports a breach no pad makes. render_placement's
            # `checklist.a_off_outline.pad_copper` is the per-PAD, margin-0
            # outline measure the docs designate as authoritative.
            'oob_pad_basis': ('part pad AABB vs outline inflated by the '
                              'grading clearance (NOT the per-pad outline '
                              'measure; see render_placement '
                              'checklist.a_off_outline.pad_copper)'),
            # worst_n <= 0 = list ALL (run-4 F5: the repair rung's census
            # was silently capped at 10 movers on a 20-pair board).
            'worst': worst[:worst_n] if worst_n and worst_n > 0 else worst,
            'exact': check_exact is not None}


def _pad_with_copper(pads, copper_index: int, clearance: float):
    """The Nth COPPER pad of a footprint's pad list (grade_pad_legality's
    PartPads indices count copper pads only, in construction order)."""
    from check_drc import _pad_has_no_copper
    n = -1
    for p in pads:
        if _pad_has_no_copper(p):
            continue
        if not any(str(l).endswith('.Cu') for l in p.layers):
            continue
        n += 1
        if n == copper_index:
            return p
    return None

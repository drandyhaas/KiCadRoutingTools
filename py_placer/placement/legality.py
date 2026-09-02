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
# #834: interned so `PartPads.pad_sides` allocates none of these per part --
# `pair_shortfall` intersects them in `quench.candidate_valid`'s inner loop,
# and a board contributes a few hundred parts.
FRONT_ONLY = frozenset(('F',))
BACK_ONLY = frozenset(('B',))
NO_SIDES = frozenset()


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


def pad_half_extents(pad):
    """(half_x, half_y) of a pad's axis-aligned bbox, honouring `rect_rotation`.

    `pad.size_x/size_y` are already resolved to board space for an orthogonal
    pad; `rect_rotation` is the residual tilt for one on a non-orthogonal angle
    (in (-90, 90]), and ignoring it UNDER-sizes the pad. THREE copies lived in
    `fanout_clearance` alone -- `_Cap`'s pad list, its via pass, and its rect
    builder -- so it lives here now, beside the gap function its callers pair
    it with.

    Two near neighbours are deliberately NOT folded in, and naming them is the
    point of this paragraph: `reachability._point_rect_dist` rotates the query
    POINT into the pad frame instead, which is an exact rotated-rect distance
    rather than a bbox; and `py_router/check_drc._pad_half_extents` computes
    the same numbers but lives across the package boundary and also handles
    custom-pad polygons. A first draft of this docstring listed `reachability`
    as one of the three and did not mention `check_drc` at all -- which is
    exactly the map a later reader hunting duplicates would have trusted.

    The bbox OVER-states a tilted pad, so a gap computed from it UNDER-states
    the true clearance: the safe direction for a proximity rule, and the same
    convention `reachability` already records -- modelling one pad two ways puts
    the naming and the number in different geometries.
    """
    tilt = math.radians(getattr(pad, 'rect_rotation', 0.0) or 0.0)
    c, s = abs(math.cos(tilt)), abs(math.sin(tilt))
    hx, hy = pad.size_x / 2.0, pad.size_y / 2.0
    return hx * c + hy * s, hx * s + hy * c


def pad_rect(pad, margin: float = 0.0):
    """A pad's axis-aligned copper rect in BOARD coordinates, plus `margin`.

    `global_x/global_y` is always the copper centre even when the drill is
    offset from it, so this is the right anchor for a clearance question and the
    wrong one for a drill question.
    """
    hx, hy = pad_half_extents(pad)
    hx += margin
    hy += margin
    return (pad.global_x - hx, pad.global_y - hy,
            pad.global_x + hx, pad.global_y + hy)


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


#: A body overlap at or above this fraction of the SMALLER body is a
#: CONTAINMENT rather than a kiss. Corpus-calibrated on the 33 boards in
#: kicad_files/, measured (not assumed), in the FAB currency:
#:
#:     FID2 / J5   (orangecrab_ext_pll)  frac 1.000  2.25 mm2  marker-exempt
#:     FID1 / J4   (orangecrab_ext_pll)  frac 0.867  1.95 mm2  marker-exempt
#:     GPDI1 / J5  (ulx3s)               frac 0.011  0.22 mm2  non-exempt
#:     GPDI1 / SW1 (ulx3s)               frac 0.001  0.085 mm2 non-exempt
#:
#: Those 4 pairs are the entire fab census. NON-EXEMPT fab containment at
#: frac >= 0.5 is ZERO on all 33 healthy boards, and the worst healthy
#: non-exempt fab frac is 0.011 against a measured defect of 1.000 -- a ~90x
#: separation, the same standard that licensed `stacks` to gate.
CONTAINMENT_FRAC = 0.5

#: Courtyard BLOCKING policy (run-23). The courtyard channel stayed advisory
#: because area alone cannot tell a by-design kiss from an interpenetration:
#: run-23's final board shipped D3<->SW1 at 0.059 mm2 / depth 0.02 (a sliver a
#: healthy board carries) NEXT TO J4<->U6 at 5.56 mm2 / depth 0.90 (two parts
#: in the same space) and every gate passed both. A pair gates only when BOTH
#: exceed these floors -- area says the overlap is substantial, depth says it
#: is an interpenetration rather than an edge graze -- and the pair is
#: unwaived (marker/edge/container/intent ladders still apply) and both
#: courtyards are REAL geometry (see GradedPart.synthetic: a zero-pad
#: footprint's fictional +/-0.5mm box manufactures phantom pairs).
COURTYARD_BLOCKING_MIN_MM2 = 0.5
COURTYARD_BLOCKING_MIN_DEPTH_MM = 0.3
#: The RELATIVE floor (run-23, user finding #2): an absolute area floor is
#: blind to small parts -- J4<->R21 measured 0.445mm2 (11% under the area
#: floor) while a QUARTER of R21's courtyard sat inside J4's. A pair also
#: gates when the overlap consumes this fraction of the SMALLER courtyard,
#: the same denominator containment_frac uses and for the same reason: area
#: is everything to a 0402 and nothing to a connector.
COURTYARD_BLOCKING_MIN_FRAC = 0.25


def containment_frac(area, ra, rb):
    """How much of the SMALLER of two bodies the overlap `area` consumes.

    The number `area_mm2` alone cannot supply, and the reason run-22 shipped a
    board every gate called buildable: a connector shell KISSING a neighbour
    and a part sitting WHOLLY INSIDE another part produce the same
    `area_mm2` on different-sized parts, so a reader could not tell an
    intended overhang from an assembly impossibility without re-deriving the
    geometry by hand (the run wrote a throwaway probe to do exactly that).

    Measured on that board: RN3 inside U5 and RN7 inside U6 both reported
    `fab 2.0 mm2` -- identical to a large shell's legitimate graze -- while
    being 1.000 of the smaller body.

    Smaller-body denominator, because containment is asymmetric: a 2 mm2
    overlap is everything to a 0402 and nothing to a connector. Returns
    0.0..1.0, or None when either body has no area (nothing to be inside of).
    """
    small = min(rect_area(ra), rect_area(rb))
    if small <= EPS:
        return None
    return round(min(1.0, area / small), 4)


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


def container_refs(pcb_data, graded) -> set:
    """Refs whose courtyard covers at least `CONTAINER_RATIO` of the board.

    A frame, not a body -- see the constant. Extracted from
    `grade_body_overlap` (#835) so the escape ledger decides who is a
    container the same way the courtyard channel does; two copies of this
    arithmetic is how the two channels would come to disagree about rp2350's
    U8, which is the part the constant was calibrated on.

    `graded` is a `graded_parts_from_file(...)` sequence. Empty when the board
    declares no bounds, which is the conservative answer: nothing is exempt.
    """
    bb = getattr(getattr(pcb_data, 'board_info', None), 'board_bounds', None)
    if not bb:
        return set()
    barea = max(1e-9, (bb[2] - bb[0]) * (bb[3] - bb[1]))
    return {g.ref for g in graded
            if rect_area(g.rect) >= CONTAINER_RATIO * barea}


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
        # Milled interior contours (#505). Already inside `rings` (that is what
        # board_edge_geometry does with them), so the distance tests see them;
        # kept separately because the SWALLOW probe needs them too (#628).
        self.milled = [c for c in
                       (getattr(board_info, 'board_edge_contours', None) or [])
                       if len(c) >= 3]
        # One representative vertex per interior ring, for the swallow probe in
        # rect_blocked / rect_outside_amount. Any vertex serves: a ring wholly
        # inside the rect has every vertex inside it, and a ring only partly
        # inside is already caught by the corner and edge tests.
        #
        # DELIBERATELY NOT a containment rule. A milled contour's interior is
        # NOT declared off-board here, and must not be: board_edge_contours
        # mixes a real inner OUTLINE (interior = board -- crkbd's nested half,
        # bus_pirate5's 60.8x80.2mm inner ring) with a milled SLOT (interior =
        # not board), and the parser cannot tell them apart. Calling the
        # interior off-board re-opens #291, where bus_pirate5 lost all 870 pads
        # and the run broke the chain. "A part may not swallow a milled ring"
        # is true for both readings, which is exactly why it is safe.
        #
        # Carries the ring's IDENTITY, not just its vertex: a part whose own
        # pads sit inside a milled ring must be allowed to swallow THAT ring
        # (see rings_enclosing / skip_rings below). Ids are indices into
        # cutouts + milled, in that order.
        self._swallow_pts = (
            [(i, r[0][0], r[0][1]) for i, r in enumerate(self.cutouts)]
            + [(len(self.cutouts) + i, r[0][0], r[0][1])
               for i, r in enumerate(self.milled)])
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

    def _ring_edge_set(self, skip_rings) -> frozenset:
        """The EDGES belonging to `skip_rings`, as a set for exact filtering.

        Ring ids index `cutouts + milled` (the `_swallow_pts` convention), which
        is NOT the ordering of `self.rings` -- so the mapping is done from the
        source lists rather than by index into `edges()`. Edges are built from
        the same vertex tuples in both places, so equality is exact; both
        orientations are stored because `edges()` walks each ring in one
        direction only and a future caller might not.

        Cached: `rect_blocked` is called in the innermost candidate loop.
        """
        if not skip_rings:
            return frozenset()
        key = frozenset(skip_rings)
        cache = getattr(self, '_ring_edge_cache', None)
        if cache is None:
            cache = self._ring_edge_cache = {}
        hit = cache.get(key)
        if hit is not None:
            return hit
        out = set()
        for rid in key:
            if rid < len(self.cutouts):
                ring = self.cutouts[rid]
            elif rid - len(self.cutouts) < len(self.milled):
                ring = self.milled[rid - len(self.cutouts)]
            else:
                continue
            n = len(ring)
            for i in range(n):
                a, b = ring[i], ring[(i + 1) % n]
                out.add((a[0], a[1], b[0], b[1]))
                out.add((b[0], b[1], a[0], a[1]))
        hit = frozenset(out)
        cache[key] = hit
        return hit

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

    def rings_enclosing(self, points) -> frozenset:
        """Ids of the MILLED rings enclosing any of `points` -- a part's OWN
        milled relief, for the `skip_rings` argument of the two probes.

        Only milled contours can be owned, by construction:
        `drop_pad_containing_cutouts` moves every ring enclosing >= 2 pad
        centres OUT of board_cutouts and into board_edge_contours, so no ring
        left in `self.cutouts` encloses a pad and no cutout id can ever land in
        a skip set. That is what keeps the genuine-cutout half of the swallow
        probe untouchable rather than merely untouched.

        >= 1 enclosed pad, not >= 2: two parts with one pad each inside a ring
        are both partial owners and both must be exempt. Pad CENTRES and the
        same containment test the parser used to reclassify the ring in the
        first place (kicad_parser.drop_pad_containing_cutouts -> _pt_in_ring).
        """
        if not self.milled:
            return frozenset()
        from check_drc import _point_in_poly
        base = len(self.cutouts)
        owned = set()
        for i, ring in enumerate(self.milled):
            for (px, py) in points:
                if _point_in_poly(px, py, ring):
                    owned.add(base + i)
                    break
        return frozenset(owned)

    # -- level 2b: per-rect bbox prefilter, exact
    def _edges_touching(self, rect, edges):
        """The subset of `edges` that can possibly come within `margin` of
        `rect`.

        EXACT, not a heuristic, and that is the whole reason it is safe to put
        in front of the threshold tests: if an edge's bounding box is
        separated from the rect's bbox by more than `margin` on EITHER axis,
        then every point of that edge differs from every point of the rect by
        more than `margin` on that axis alone, so their distance exceeds
        `margin` and the edge cannot contribute.

        It is not usable in front of `edge_clearance`, which reports the true
        minimum distance rather than testing it against a threshold -- the
        nearest edge there may be arbitrarily far away.

        Why it matters: `edges_near` prunes per PART, sized by a displacement
        budget, and the seeder's state has no budget at all (pose_score builds
        its QuenchState without `build_neighbor_lists`, so `_travel_budget` is
        infinite and `edges_near` returns every edge). `_try_place` calls
        `rect_outside_amount` once per candidate offset, and `_offsets(16,
        0.25)` alone is 16,641 offsets.

        Measured on run 19's urchin, a 638-edge outline with one milled ring:
        7x to 14x depending on machine load and on which rect population is
        swept (a sweep near an edge keeps more edges than one over open
        board). The conservative end is the number to quote. An independent
        measurement that neutralised this method by monkeypatching it to the
        identity -- so the ONLY difference between arms was the prefilter --
        put it at 11.98x on two separate runs.
        """
        m = self.margin
        lo_x, lo_y, hi_x, hi_y = rect[0] - m, rect[1] - m, rect[2] + m, rect[3] + m
        out = []
        for e in edges:
            ax, ay, bx, by = e
            if (ax if ax > bx else bx) < lo_x:
                continue
            if (ax if ax < bx else bx) > hi_x:
                continue
            if (ay if ay > by else by) < lo_y:
                continue
            if (ay if ay < by else by) > hi_y:
                continue
            out.append(e)
        return out

    # -- level 3: the exact tests
    def rect_blocked(self, rect, edges=None, skip_rings=None) -> bool:
        """True when a rect leaves the real outline, enters a cutout, or comes
        within the edge margin of either.

        `edges` restricts the distance test to a pre-filtered edge list from
        `edges_near`; omitted, every ring edge is measured.

        `skip_rings` (from `rings_enclosing`) exempts the tested part's OWN
        milled rings -- a connector over its own milled relief may swallow it,
        and without this it is judged board-violating at its own hand-placed
        pose.

        THE EXEMPTION COVERS THE EDGE-MARGIN TEST TOO, not only the swallow
        probe. It did not, and the omission made the exemption almost useless:
        a part whose own pads caused a contour to be reclassified as an inner
        milled edge still had every pose within `margin` of that contour
        vetoed -- which, for a part sitting INSIDE its own relief, is every
        pose there is. Measured on run 20's SW2, which owns the strap slot its
        two NPTH posts sit in: 0 legal poses of 14884 at the board's own
        floors, and at margin 0, 508 of 9604 -- all at rot 0/180, none at SW2's
        own rot 270. `place_optimize` with SW2 free and 83 refs locked moved it
        0 mm with the edge term as the entire objective; `place_reconstruct`
        reported "no legal pose within any cap". The run had to declare SW2 an
        `edge_actuator` and waive it permanently.

        PER-PART, never global: the ring stays a hard edge for every other
        part, and `rings_enclosing` only ever returns MILLED ids, so the outer
        outline and the genuine cutouts can never be exempted by this path.
        """
        from check_drc import _point_on_board, _seg_seg_dist_coords
        x0, y0, x1, y1 = rect
        for (px, py) in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
            if not _point_on_board(px, py, self.outer, self.cutouts):
                return True
        near = self._edges_touching(
            rect, self.edges() if edges is None else edges)
        _own = self._ring_edge_set(skip_rings)
        if _own:
            near = [e for e in near if e not in _own]
        for (ax, ay, bx, by) in ((x0, y0, x1, y0), (x1, y0, x1, y1),
                                 (x1, y1, x0, y1), (x0, y1, x0, y0)):
            for (ex1, ey1, ex2, ey2) in near:
                if _seg_seg_dist_coords(ax, ay, bx, by,
                                        ex1, ey1, ex2, ey2) < self.margin:
                    return True
        # An interior ring FULLY INSIDE the rect evades both tests above -- no
        # corner is off-board and no rect edge comes near a ring edge (#628).
        for (rid, cx, cy) in self._swallow_pts:
            if skip_rings and rid in skip_rings:
                continue  # the part's own milled relief
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

    def rect_outside_amount(self, rect, exact: bool = True, edges=None,
                            skip_rings=None) -> float:
        """Magnitude of a rect's board-boundary violation; 0 iff fully legal.

        Zero exactly when the bbox inset holds AND `rect_blocked` is False, so
        it can drive a "may not worsen" acceptance rule and a grader from one
        definition. It is a distance-like magnitude, not an area -- see
        `out_of_board_area` for an area.

        `exact=False` skips the ring terms, for a caller that has already
        established via `may_reach` that this rect cannot come near a ring. The
        ring terms are ~100x the cost of the bbox term, and in a hot candidate
        loop most parts are nowhere near an edge.

        `skip_rings` must match what `rect_blocked` is given for the same rect,
        or the two stop agreeing on legality (see the swallow probe below).
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
        # Only edges that can come within `margin` can contribute a term, and
        # `_edges_touching` selects exactly those -- so this stays the same
        # number while measuring against a short list. See its docstring.
        near = self._edges_touching(rect, use)
        # Same exemption as rect_blocked's, and it MUST be the same: the two
        # functions' agreement is what makes `violation() == 0` imply `not
        # rect_blocked()`. Exempting the owned ring in one and not the other
        # would give a part a legal verdict and a non-zero cost at the same
        # pose, which every acceptance rule downstream reads as a regression.
        _own = self._ring_edge_set(skip_rings)
        if _own:
            near = [e for e in near if e not in _own]
        for (ax, ay, bx, by) in ((x0, y0, x1, y0), (x1, y0, x1, y1),
                                 (x1, y1, x0, y1), (x0, y1, x0, y0)):
            d = min((_seg_seg_dist_coords(ax, ay, bx, by, *e) for e in near),
                    default=float('inf'))
            if d < self.margin:
                amt += self.margin - d
        # Swallowed interior ring (cutout or milled contour), same probe and
        # same charge as rect_blocked's -- the two must agree on legality or
        # `violation()==0` stops implying `not rect_blocked()` (#628).
        for (rid, cx, cy) in self._swallow_pts:
            if skip_rings and rid in skip_rings:
                continue  # the part's own milled relief
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
    # True when `rect` is the +/-0.5mm FICTION compute_footprint_bbox_local
    # returns for a footprint with no courtyard AND no pads (logos, graphics).
    # Such a rect is not geometry anyone drew, so overlap against it must
    # never gate -- run-23's G***<->J5 "0.537 mm2" pair was exactly this
    # artifact. A pad-bbox fallback (pads exist, courtyard missing) stays
    # False: those bounds are real copper.
    synthetic: bool = False

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
    kind: str            # 'pad_intersection' | 'courtyard' | 'fab'
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
    # How much of the SMALLER body this overlap consumes, 0..1 -- see
    # `containment_frac`. THE field `area_mm2` alone cannot supply: run-22
    # shipped RN3-in-U5 and RN7-in-U6 at `fab 2.0 mm2`, which is also what a
    # large connector's by-design graze measures. Area cannot tell a KISS from
    # a part WHOLLY INSIDE another; this can.
    #
    # None = NOT MEASURED, not "no containment". The pad_intersection channel
    # has no body rect in scope, so it reports None rather than a 0.0 that
    # would read as a measurement someone took.
    contained_frac: Optional[float] = None
    # Penetration depth: the SHORTER axis extent of the intersection rect, mm
    # (0.0 = not measured; the pad_intersection channel has no body rects).
    # Area cannot tell a long thin kiss from a real interpenetration --
    # run-23: D3<->SW1 0.059mm2 at depth 0.02 vs J4<->U6 5.56mm2 at depth
    # 0.90 -- and the courtyard blocking policy keys on this axis.
    depth_mm: float = 0.0
    # WHERE the overlap is: the intersection rect (x0, y0, x1, y1), None =
    # not measured. run-23 user finding #1: the edge-class waiver needs the
    # REGION, not only the pair -- J1's mating overhang legitimately covers
    # the strip at/over the outline, and nothing else, yet R5 sat 45.7%
    # inside J1's INTERIOR courtyard (under the connector body, 1.5mm inside
    # the board) behind the same waiver.
    overlap_rect: Optional[Tuple[float, float, float, float]] = None
    # `contained_frac >= CONTAINMENT_FRAC`, **on the fab channel only**.
    #
    # Why fab-only, measured on the same 33 boards: the COURTYARD currency
    # ships frac-1.0 containment on four healthy boards -- esp_prog (a part
    # inside USB1), orangecrab_ext_pll (R9, R12 inside J5),
    # rp2350_fpga_eensy_prePlane (U3 inside J2), ulx3s (R24, R22, C18 inside
    # GPDI1, OSHW inside U9). A courtyard is body PLUS margin PLUS shell
    # overhang volume, so a small part living under a connector's courtyard is
    # ordinary and correct. The .Fab outline is the real body, and there a
    # non-exempt containment is unknown on the corpus.
    #
    # So `contained_frac` is measured on both body channels because it is free
    # and true, while `contained` -- the flag anything downstream acts on --
    # is asserted only where the corpus says it discriminates.
    contained: bool = False


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
            worst_rects = None
            for s in (a.sides & b.sides):
                ra = rect_on(s, a.side, a.rect, a.tht_rect)
                rb = rect_on(s, b.side, b.rect, b.tht_rect)
                if ra is None or rb is None:
                    continue
                area = rect_overlap_area(ra, rb)
                if area > worst:
                    worst = area
                    worst_side = s
                    worst_rects = (ra, rb)
            if worst > EPS:
                ra, rb = worst_rects
                ix = (max(ra[0], rb[0]), max(ra[1], rb[1]),
                      min(ra[2], rb[2]), min(ra[3], rb[3]))
                _dx, _dy = ix[2] - ix[0], ix[3] - ix[1]
                out.append(BodyOverlapPair(
                    a=min(a.ref, b.ref), b=max(a.ref, b.ref),
                    kind='courtyard', area_mm2=round(worst, 4),
                    side=worst_side, waived=False, waiver='',
                    contained_frac=containment_frac(worst, *worst_rects),
                    depth_mm=round(max(0.0, min(_dx, _dy)), 4),
                    overlap_rect=tuple(round(v, 4) for v in ix)))
    out.sort(key=lambda p: (-p.area_mm2, p.a, p.b))
    return out


class LocalBounds(NamedTuple):
    """One part's UNROTATED local geometry, before any pose is applied.

    The half of `graded_parts_from_file` that does not depend on where the part
    currently sits: which box the courtyard is, whether it is the pad-bbox
    fallback or the zero-pad fiction, and the drilled-pad box. Split out because
    a consumer asking "could this part fit HERE, at any rotation" needs the
    local box at a rotation the file does not carry, and re-deriving that chain
    is exactly the second implementation #456 exists to prevent.

    `local` / `tht_local` are in the footprint's own frame; rotate them with
    `rotate_local_bounds` and offset by the pose to get board coordinates.
    """
    ref: str
    side: str
    local: Tuple[float, float, float, float]
    tht_local: Optional[Tuple[float, float, float, float]]
    has_tht: bool
    # True when `local` is the +/-0.5mm FICTION for a footprint with no
    # courtyard AND no pads. See `GradedPart.synthetic`: not geometry anyone
    # drew, so it must never gate.
    synthetic: bool
    # False when the courtyard was missing and `local` is the PAD bbox, which
    # carries no courtyard margin. NOT a safe direction, contrary to an earlier
    # comment here: a smaller part is more permissive only WITHIN a fixed
    # anchor mode, and shrinking can flip `zone_is_anchor` True->False, whose
    # non-anchor branch has a strictly smaller admissible origin box. Measured:
    # zone [0,0,5,2] tol 0, keep-out [0,0.9,2,1.1] -- a 6x3 courtyard is
    # `seated` and its 4x1 pad bbox is REFUSED. So a consumer refusing on this
    # geometry may differ in EITHER direction, which is why it is recorded
    # per part rather than assumed away.
    from_courtyard: bool


def part_local_bounds(pcb_data, pcb_file: Optional[str] = None
                      ) -> Dict[str, LocalBounds]:
    """THE local-bounds chain, once: `{ref: LocalBounds}`.

    Same geometry rules as the quench state (#456: one definition of legal):
    courtyard for the part's own side via the text parser, pad-bbox fallback
    when the footprint draws none, drilled-pad box on the far side. Needs the
    board FILE for courtyards (the text parser reads it); falls back to
    `pcb_data.source_path`, then to pad bboxes everywhere.

    A ref is ABSENT when even the pad fallback raised -- the same parts
    `graded_parts_from_file` skips, so both consumers see one universe.
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
    out: Dict[str, LocalBounds] = {}
    for ref, fp in sorted((pcb_data.footprints or {}).items()):
        own = footprint_side(fp)
        local = None
        synthetic = False
        from_courtyard = False
        if sides.get(ref):
            local = courtyard_for_side(sides[ref], own)
            from_courtyard = local is not None
        if local is None:
            try:
                local = compute_footprint_bbox_local(fp)
                # No courtyard AND no pads: the +/-0.5mm fiction, not geometry.
                synthetic = not (fp.pads or ())
            except Exception:
                local = None
        if local is None:
            continue
        tht_local = None
        has_tht = footprint_has_through_pads(fp)
        if has_tht:
            tht_local = _through_pad_bounds_local(fp)
        out[ref] = LocalBounds(ref=ref, side=own, local=tuple(local),
                               tht_local=(tuple(tht_local)
                                          if tht_local is not None else None),
                               has_tht=has_tht, synthetic=synthetic,
                               from_courtyard=from_courtyard)
    return out


def graded_parts_from_file(pcb_data, pcb_file: Optional[str] = None
                           ) -> List[GradedPart]:
    """`GradedPart` records at the FILE's own poses.

    The pose half of `part_local_bounds`: rotate each part's local box by its
    own rotation and offset it. The chain itself lives there, so a consumer
    that needs the box at ANOTHER rotation does not grow a second copy of it.
    """
    out: List[GradedPart] = []
    for ref, lb in part_local_bounds(pcb_data, pcb_file).items():
        fp = pcb_data.footprints[ref]
        rot = fp.rotation or 0.0
        lx0, ly0, lx1, ly1 = rotate_local_bounds(*lb.local, rot)
        rect = (fp.x + lx0, fp.y + ly0, fp.x + lx1, fp.y + ly1)
        tht = None
        if lb.has_tht and lb.tht_local is not None:
            tx0, ty0, tx1, ty1 = rotate_local_bounds(*lb.tht_local, rot)
            tht = (fp.x + tx0, fp.y + ty0, fp.x + tx1, fp.y + ty1)
        out.append(GradedPart(ref=ref, side=lb.side, rect=rect,
                              tht_rect=tht, has_tht=lb.has_tht,
                              synthetic=lb.synthetic))
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

    _graded = graded_parts_from_file(pcb_data, pcb_file)
    # Refs whose courtyard rect is the zero-pad +/-0.5mm fiction: their pairs
    # may inform but must never gate (run-23's phantom G***<->J5).
    _synthetic_refs = {g.ref for g in _graded if g.synthetic}
    # `bb` stays bound here: the edge-waiver and off-board arms below read it.
    bb = getattr(getattr(pcb_data, 'board_info', None), 'board_bounds', None)
    _containers = container_refs(pcb_data, _graded)

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
    # Refs the fab channel could not judge (no .Fab geometry). A board with no
    # readable source path leaves this empty AND judges nothing -- both are
    # reported rather than silently conflated with "clean".
    fab_unjudged: set = set()

    # -- courtyard channel (advisory + the run-23 blocking policy below) ------
    for p in body_overlap_pairs(_graded):
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
                fab_unjudged.add(ref)
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
                    _cf = containment_frac(ov, rca, rcb)
                    _dx = min(rca[2], rcb[2]) - max(rca[0], rcb[0])
                    _dy = min(rca[3], rcb[3]) - max(rca[1], rcb[1])
                    pairs.append(BodyOverlapPair(
                        a=min(ra, rb), b=max(ra, rb), kind='fab',
                        area_mm2=round(ov, 4), side=sa,
                        waived=bool(waiver), waiver=waiver,
                        contained_frac=_cf, contained=_cf is not None
                        and _cf >= CONTAINMENT_FRAC,
                        depth_mm=round(max(0.0, min(_dx, _dy)), 4)))

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
    contained = [p for p in pairs if p.contained]
    # The subset that GATES. A containment is by-design when a MARKER
    # (mount_hole/fiducial/testpoint) or a CONTAINER (courtyard >= half the
    # board) is involved -- orangecrab ships FID2 wholly inside J5 at frac
    # 1.000 and FID1 inside J4 at 0.867, and both are correct -- or when an
    # operator has named the pair in the intent's `overlap_waivers`, which is
    # AUTHORED and recorded.
    #
    # `edge_class` is deliberately NOT an exemption, and that is the whole
    # point. It is a pure part-class lookup with no geometry in it, and it is
    # the waiver that hid run-22's D4-wholly-inside-SW2 (SW2 is a declared
    # edge_actuator) and C26/FB3/R1 inside J1's 65mm2 USB-C body on the board
    # that run shipped as `buildable`. An operator may still accept those --
    # by writing the pair into the intent, where the acceptance is visible --
    # but never by inheriting a class waiver nobody authored.
    #
    # Corpus effect, measured: ZERO boards gate on this.
    _GATE_EXEMPT = ('marker_class', 'container_class', 'intent_declared')
    containment_blocking = [p for p in contained
                            if p.waiver not in _GATE_EXEMPT]
    # Courtyard BLOCKING (run-23): the subset of courtyard pairs that gates.
    # Both floors must trip -- area >= COURTYARD_BLOCKING_MIN_MM2 and depth
    # >= COURTYARD_BLOCKING_MIN_DEPTH_MM -- so by-design slivers a healthy
    # board carries stay advisory, and a synthetic (+/-0.5mm fiction)
    # courtyard never gates anything.
    #
    # The waiver ladder applies WITH ONE GEOMETRY CONDITION on edge_class
    # (the run-22 containment lesson, in its courtyard form): an edge-class
    # waiver is a claim that the part's shell/actuator legitimately overhangs
    # -- which is only TRUE of a part that is actually AT an edge. Run-23's
    # board waived every SW1/SW2 collision this way (SW1<->U1 1.74mm2,
    # FB1<->SW2 0.70mm2 with real body contact) while SW2 sat 8.33mm
    # INTERIOR, where the overhang story is geometrically dead. The waiver
    # now stands only for a member whose pose is edge-LIVE: overhanging the
    # outline, or within SEAT_TOL_MM of an edge. J1<->SW1 stays waived (J1
    # is a receptacle overhanging at the edge; its courtyard includes the
    # mating volume); marker/container/intent waivers are untouched --
    # geometry cannot invalidate "this is a fiducial" or an authored intent.
    _EDGE_W = ('edge_receptacle', 'edge_actuator')
    _edge_live_cache: Dict[str, bool] = {}

    def _edge_waiver_live(ref: str) -> bool:
        if ref in _edge_live_cache:
            return _edge_live_cache[ref]
        live = False
        if _class_of(ref) in _EDGE_W and bb:
            g = next((x for x in _graded if x.ref == ref), None)
            if g is not None:
                try:
                    from .part_class import SEAT_TOL_MM
                    _og = BoardOutlineGate(pcb_data.board_info, 0.0)
                    live = (_og.rect_outside_amount(g.rect) > EPS
                            or _og.edge_clearance(g.rect) <= SEAT_TOL_MM)
                except Exception:                            # noqa: BLE001
                    live = True     # unmeasurable geometry never UN-waives
        _edge_live_cache[ref] = live
        return live

    # Marker classes whose courtyard is NON-PHYSICAL and may keep the blanket
    # blocking exemption. A mount_hole is deliberately absent (run-23, user
    # finding #3): its courtyard is the SCREW-HEAD/standoff keepout -- J3's
    # header sat 1.47mm inside locked H2's courtyard behind the same
    # 'marker_class' label a fiducial gets, and a screw in H2 lands on J3's
    # pin row. Fiducials and testpoints have nothing above board level;
    # mounting holes do.
    _MARKER_NONPHYSICAL = ('fiducial', 'testpoint')

    def _blocking_waived(p) -> bool:
        if not p.waived:
            return False
        # An AUTHORED waiver outranks everything below: it is a recorded
        # human decision about this exact pair.
        if p.waiver == 'intent_declared':
            return True
        # No CLASS waiver blesses contact with a KiCad-LOCKED part (the
        # run-8 E6 principle, extended to the courtyard channel): a locked
        # pose is a decision somebody made, and a class label chosen for
        # unlocked parts does not apply to copper or courtyards landing on
        # it. The pair still faces the floors + the moved gate like any
        # unwaived pair.
        if p.a in locked_refs or p.b in locked_refs:
            return False
        if p.waiver == 'marker_class':
            return (_class_of(p.a) in _MARKER_NONPHYSICAL
                    or _class_of(p.b) in _MARKER_NONPHYSICAL)
        if p.waiver != 'edge_class':
            return True
        if not (_edge_waiver_live(p.a) or _edge_waiver_live(p.b)):
            return False
        # run-23 user finding #1: an edge-LIVE part's waiver covers its
        # MATING ZONE, never its whole courtyard. The overhang story is
        # about the strip at/over the outline; an overlap sitting INSIDE
        # the board is under the part's BODY, whoever is at the edge --
        # R5 sat 45.7% inside J1's interior courtyard behind this waiver.
        # The overlap region must leave the outline or hug the edge
        # (within SEAT_TOL_MM); unmeasurable geometry never UN-waives.
        r = p.overlap_rect
        if r is None or not bb:
            return True
        try:
            from .part_class import SEAT_TOL_MM
            _og = BoardOutlineGate(pcb_data.board_info, 0.0)
            return (_og.rect_outside_amount(r) > EPS
                    or _og.edge_clearance(r) <= SEAT_TOL_MM)
        except Exception:                                    # noqa: BLE001
            return True

    courtyard_blocking = [
        p for p in pairs
        if p.kind == 'courtyard' and not _blocking_waived(p)
        # ABSOLUTE floor or RELATIVE floor (user finding #2: 0.445mm2 was
        # 25.5% of R21's courtyard and slid under the absolute floor).
        and (p.area_mm2 >= COURTYARD_BLOCKING_MIN_MM2
             or (p.contained_frac or 0.0) >= COURTYARD_BLOCKING_MIN_FRAC)
        and p.depth_mm >= COURTYARD_BLOCKING_MIN_DEPTH_MM
        and p.a not in _synthetic_refs and p.b not in _synthetic_refs]
    return {'blocking': len(blocking),
            'advisory': len(advisory),
            'waived': sum(1 for p in pairs if p.waived),
            'pairs': pairs,
            'blocking_pairs': blocking,
            'advisory_pairs': advisory,
            # Body containment: one part's .Fab outline (near-)wholly inside
            # another's. DISCLOSURE ONLY -- deliberately NOT folded into
            # `blocking`, because the corpus ships legitimate frac-1.0
            # containments (fiducials under connector bodies, measured on
            # orangecrab_ext_pll: FID2/J5 at 1.000, FID1/J4 at 0.867).
            #
            # NOT filtered by `waived`, and that one omission is the whole
            # point. `_waiver_for` is pure class membership and geometry-blind,
            # so a part sitting WHOLLY inside an `edge_actuator` gets the same
            # `edge_class` label as a 0.01 mm2 graze and then leaves
            # `advisory`, `advisory_pairs` AND `new_advisory_pairs` in one
            # step. Run-22 lost D4-inside-SW2 exactly that way, twice, and
            # nothing in the chain could report it. This is the list a waiver
            # cannot empty.
            'contained': len(contained),
            'containment_pairs': contained,
            'containment_blocking': len(containment_blocking),
            'containment_blocking_pairs': containment_blocking,
            # Parts whose footprint draws no .Fab outline at all, so the fab
            # channel CANNOT judge them (parser.extract_fab_sides skips them
            # and the loop above continues past them). An unjudged part is not
            # a clean part, and saying nothing would claim it was.
            #
            # MEASURED across all 33 corpus boards, so nobody has to re-derive
            # it: 144 of 1583 footprints are unjudged, and 144 of 144 draw ZERO
            # .Fab geometric primitives. They are mounting holes, testpoints,
            # logos, fiducials and panel tabs. There is NO footprint anywhere
            # in the corpus whose .Fab geometry the parser fails to read --
            # every primitive kind used on that layer (fp_line, fp_arc,
            # fp_circle, fp_poly, fp_rect) is already handled, and there are no
            # degenerate bboxes.
            #
            # So a parser tolerance or polygon-closure fix moves ZERO
            # footprints out of this set. (313 footprints do have non-closing
            # .Fab line chains -- tigard Q1's left edge stops 0.02mm short --
            # and every one of them is judged correctly, because a bbox is a
            # min/max over points and the gap is interior to it.)
            #
            # And the one change that WOULD close the hole is measured
            # dangerous: giving a bodyless part a fallback body (its courtyard,
            # else its pad bbox) adds 77 new fab pairs corpus-wide, 65 of them
            # above CONTAINMENT_FRAC -- dominated by rp2350's Teensy40, a
            # bodyless module whose courtyard swallows 56 neighbours at frac
            # 1.0. It would also break the 4-pair calibration gate outright.
            #
            # DISCLOSURE IS THE ANSWER HERE, not a fix. Report the count and
            # let a reader judge.
            'fab_unjudged': len(fab_unjudged),
            'fab_unjudged_refs': sorted(fab_unjudged),
            # Run-23 courtyard blocking channel -- see the selection above.
            # NOTE these pairs also remain in `advisory`/`advisory_pairs`
            # (that count's meaning is unchanged for its existing consumers);
            # a pair can be both advisory-listed and courtyard-blocking.
            'courtyard_blocking': len(courtyard_blocking),
            'courtyard_blocking_pairs': courtyard_blocking,
            # Refs graded on the zero-pad +/-0.5mm fictional box: their pairs
            # are disclosure, never gates (run-23's phantom G***<->J5).
            'courtyard_synthetic_refs': sorted(_synthetic_refs),
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


# Local import at module scope would drag the whole DRC stack in at import
# time (see the BoardOutlineGate note); bound once on first use instead of once
# per pad, which is where the old inline copy had it.
def _pad_has_no_copper(p):
    global _PAD_HAS_NO_COPPER
    if _PAD_HAS_NO_COPPER is None:
        from check_drc import _pad_has_no_copper as _f
        _PAD_HAS_NO_COPPER = _f
    return _PAD_HAS_NO_COPPER(p)


_PAD_HAS_NO_COPPER = None


def _pad_carries_copper(p) -> bool:
    """Does this pad put copper on a copper layer?

    The predicate `PartPads` builds its pad list from, `_pad_with_copper` walks
    its index with, and `PadClearanceModel` tests inertness by. It has to be ONE
    function: they are the same two conditions, and they must agree or the
    rect index stops addressing the pad. Measured when they did not: watchy
    declares `local_clearance` on 8 NPTH pads that carry no copper, so a raw
    scan of `fp.pads` reported the board as having overrides while its parts'
    `max_floor` was 0.0 -- the board lost its inertness for nothing.
    """
    if _pad_has_no_copper(p):
        return False
    return any(str(l).endswith('.Cu') for l in (getattr(p, 'layers', None) or ()))


class PadFloor(NamedTuple):
    """One pad's required-clearance inputs, resolved once at build time.

    `ncl` is its net's non-Default net-class clearance, `lc` its own (or its
    footprint's, already resolved by the parser) `local_clearance` override,
    and `layers` the copper layers its copper occupies -- carried only when the
    board has .kicad_dru rules to scope, else None.
    """
    ncl: float
    lc: float
    layers: object = None


class PadClearanceModel:
    """The per-pair required clearance, resolved exactly as check_drc does.

    #697: the placement census used to price every pad pair at one flat scalar,
    so a board that could not pass DRC reported nothing to fix. Measured: a
    fiducial keep-clear pad (`local_clearance` 1.016mm) 0.94mm from a connector
    pad -- `check_drc` flagged it, while `grade_pad_legality` and
    `place_reconstruct --stages legalize` both reported 0 conflict pairs.

    The formula is check_drc's, and deliberately CALLS its code rather than
    mirroring it (`check_drc.pads_shared_layer_clearance`, `pad_copper_layers`)::

        base = max(global clearance, netclass(a), netclass(b))
        eff  = <.kicad_dru layer rules over the SHARED copper layers>   # REPLACES
        eff  = max(eff, lc_a, lc_b)                                     # override wins

    #768: when the caller passes a `ceiling` -- a `--clearance` it will then
    CLAMP the output project's classes down to -- the netclass term alone is
    capped at it, in `for_board`'s map and nowhere else. The dru rules and the
    pad overrides run afterwards and keep outranking it, because the writeback
    does not touch either. `ceiling=None` (the default) is the documented
    "--clearance OMITTED" branch and leaves every tier as it was, which is why
    the two callers that write no project are untouched by it.

    Note the override enters as a max over BOTH pads: check_drc needs a second
    max at its own pad-pad call site for the same reason, because a pair is
    equally in violation when only the SECOND pad carries the keep-clear.

    `active` is False -- and every consumer then takes its original flat-scalar
    path unchanged -- when the board declares no netclass, no dru rule and no
    pad override. That is the common case, and the reason this cannot perturb
    an ordinary board.

    #735: the same `.kicad_dru`'s TRACK-scoped rules are carried too, in
    `track_rules` / `net_classes`, and answered by `track_pair` -- which is a
    separate resolver from `pair` and is **not** part of `active`. Both halves
    of that are deliberate. A track rule binds `A.Type=='track' &&
    B.Type=='track'`, so it can never price a pad pair, which is all `pair`
    is asked for; and `active` is the switch consumers use to drop this object
    entirely, so admitting a track-only board there would move
    `grade_pad_legality`, `quench` and every broad phase in
    `fanout_clearance` onto their resolved path to answer a question none of
    them asks. The one caller that does ask reads `track_rules` for itself.
    """

    __slots__ = ('base', 'net_floor', 'layer_rules', 'board_copper',
                 'active', 'notes', 'ceiling', '_pair_cache',
                 'track_rules', 'net_classes')

    def __init__(self, base: float, net_floor=None, layer_rules=None,
                 board_copper=(), has_overrides: bool = False,
                 ceiling: Optional[float] = None,
                 track_rules=None, net_classes=None):
        self.base = float(base)
        # #768: the --clearance ceiling this model was built under, or None.
        # Recorded rather than re-derived: `net_floor` is already capped by the
        # time anyone can look at it, so a value equal to the ceiling and a
        # value that was never above it are indistinguishable afterwards.
        self.ceiling = None if ceiling is None else float(ceiling)
        self.net_floor = dict(net_floor or {})
        self.layer_rules = dict(layer_rules or {})
        self.board_copper = list(board_copper or [])
        # #735: the TRACK-scoped .kicad_dru channel and the class memberships
        # that are its binding key. A separate tier from the three above, and
        # deliberately NOT part of `active`: those three price the pair kinds
        # every consumer of this model asks about, while a track rule binds
        # track-vs-track and nothing else (KiCad's `A.Type=='track' &&
        # B.Type=='track'`). Folding it in would move `grade_pad_legality`,
        # `quench` and every broad phase in `fanout_clearance` off their inert
        # path to answer a question none of them asks. The one consumer that
        # DOES ask reads `track_rules` directly -- see `track_pair`.
        self.track_rules = list(track_rules or [])
        self.net_classes = dict(net_classes or {})
        self.active = bool(self.net_floor or self.layer_rules or has_overrides)
        self.notes = []
        self._pair_cache = {}

    # -- construction ---------------------------------------------------------
    @classmethod
    def for_board(cls, pcb_data, clearance: float, pcb_file: str = None,
                  ceiling: Optional[float] = None):
        """Resolve from the board's own sibling files.

        Path discovery is the #498 rule: the caller's `pcb_file` when it has
        one, else `PCBData.source_path` (engines whose signature carries no
        input file). A missing sibling is a strict no-op, never an error -- the
        model simply carries fewer sources.

        `ceiling` (#768) caps the NETCLASS tier at a `--clearance` that will
        be clamped into the output project. The test is whether THE RUN THIS
        PRICES FOR writes that clamp, not whether this process does: pass it
        when the clamp lands, leave it None otherwise, or the pass prices at a
        class KiCad will still enforce.

        The distinction is not pedantic -- `animate_fanout_clearance.py` passes
        a ceiling and writes no project at all, because it exists to VISUALISE
        `place_fanout_clearance.py` and must price identically or the GIF shows
        a repair the tool does not perform. `grade_pad_legality` and `quench`
        leave it None for the opposite reason: nothing downstream of them
        clamps anything.
        """
        path = pcb_file or getattr(pcb_data, 'source_path', '') or ''
        fps = getattr(pcb_data, 'footprints', None) or {}
        has_overrides = any(
            (getattr(p, 'local_clearance', 0.0) or 0.0) > 0.0
            and _pad_carries_copper(p)
            for fp in fps.values()
            for p in (getattr(fp, 'pads', None) or ()))
        board_copper = list(
            getattr(getattr(pcb_data, 'board_info', None), 'copper_layers', None)
            or [])
        net_floor = {}
        layer_rules = {}
        track_rules = []
        net_classes = {}
        notes = []
        if path:
            nets = getattr(pcb_data, 'nets', None) or {}
            try:
                # check_drc's map, NOT the router's `net_clearance_map_by_id`.
                # That one omits every net resolving only to Default and takes
                # the MAX over all matching classes -- both correct for a
                # router (a Default net routes at config.clearance;
                # over-blocking is safe) and both wrong for a grader, which has
                # to agree with check_drc pair for pair. Dropping Default
                # re-created this very issue whenever an explicit --clearance
                # sits BELOW the board's Default class, which check_assembly
                # tells users they may pass; taking the max over glob
                # memberships invented requirements check_drc grades clean.
                from list_nets import net_clearance_map
                by_name = net_clearance_map(
                    path, [n.name for n in nets.values()
                           if getattr(n, 'name', None)]) or {}
                if ceiling is not None:
                    # #768: --clearance is a CEILING on the netclass tier and
                    # nothing above it. min() BEFORE the admission test below,
                    # so a class at or under the ceiling still raises normally
                    # and one above it collapses to the ceiling -- which is
                    # `clearance` itself, so it never enters the map at all.
                    #
                    # Capped HERE and not in `pair_with_source` for two
                    # reasons. This map IS this pass's netclass tier, and it is
                    # the same place the router caps (route.py pre-caps the
                    # netclass map before set_net_clearances installs it), so
                    # the two halves of the toolchain do the same thing in the
                    # same place. And the dru REPLACE and the pad override run
                    # after it in `pair_with_source`, which is what keeps them
                    # outranking the ceiling -- they must, because the project
                    # writeback clamps neither.
                    _capped = {}
                    for _v in by_name.values():
                        if _v > ceiling + 1e-9:
                            _k = round(_v, 6)
                            _capped[_k] = _capped.get(_k, 0) + 1
                    if _capped:
                        notes.append(
                            'net classes capped at the %gmm --clearance '
                            'ceiling: %s' % (ceiling, ', '.join(
                                '%g -> %g (%d net%s)'
                                % (_v, ceiling, _c, '' if _c == 1 else 's')
                                for _v, _c in sorted(_capped.items()))))
                    by_name = {n: min(v, ceiling) for n, v in by_name.items()}
                # Same admission rule as check_drc: a class at or below the
                # board-wide floor cannot raise anything, so it never enters.
                net_floor = {nid: by_name[n.name]
                             for nid, n in nets.items()
                             if getattr(n, 'name', None) in by_name
                             and by_name[n.name] > clearance}
            except Exception as exc:                            # noqa: BLE001
                net_floor = {}
                notes.append('net classes unread (%s: %s)'
                             % (type(exc).__name__, exc))
            try:
                from kicad_dru import read_board_layer_clearances
                layer_rules, dru_notes = read_board_layer_clearances(
                    path, board_copper)
                notes.extend('.kicad_dru: ' + n for n in dru_notes)
            except Exception as exc:                            # noqa: BLE001
                layer_rules = {}
                notes.append('.kicad_dru unread (%s: %s)'
                             % (type(exc).__name__, exc))
            # #735: the SAME .kicad_dru's track-scoped rules, plus the class
            # memberships that bind them. `board_track_rules` is quiet by
            # construction and never raises -- it answers ([], {}) for a
            # missing file, an unparsable one, and the case where the rules
            # parse but memberships cannot be read (keeping rules there would
            # grade every pair as a NON-member, which is a different answer,
            # not a degraded one; check_drc drops them for the same reason).
            #
            # NOTHING IS ADDED TO `notes` HERE, and that is deliberate twice
            # over.
            #
            # The parse notes would DUPLICATE. `read_board_layer_clearances`
            # and `read_board_track_clearances` are two calls into the same
            # `kicad_dru._parse_dru`, differing only in the copper list they
            # pass -- and no note site in that function depends on the copper
            # list, so the two note lists come back byte-identical (measured
            # on a six-rule dru). The layer read above already filed them.
            #
            # And a note SAYING WHICH RULES WERE HONOURED would be a lie in
            # two of this constructor's three callers. `notes` reaches the
            # operator as `pad clearance: ...` from `grade_pad_legality` and
            # from the quench census, and NEITHER reads `track_rules` --
            # a track rule binds no pad pair, which is the whole reason it is
            # a separate tier. The pass that does honour it discloses it where
            # it acts, in the nudger's own fallback line.
            try:
                from kicad_dru import board_track_rules
                track_rules, net_classes = board_track_rules(pcb_data, path)
            except Exception as exc:                            # noqa: BLE001
                track_rules, net_classes = [], {}
                notes.append('.kicad_dru track rules unread (%s: %s)'
                             % (type(exc).__name__, exc))
        model = cls(clearance, net_floor, layer_rules, board_copper,
                    track_rules=track_rules, net_classes=net_classes,
                    has_overrides=has_overrides, ceiling=ceiling)
        model.notes = notes
        return model

    # -- per-pad --------------------------------------------------------------
    def pad_floor(self, pad):
        """This pad's clearance inputs.

        `getattr` on local_clearance, never a bare attribute read: the placement
        tests build duck-typed pad fakes that do not carry the field, and a hard
        read would turn a missing attribute into a crash instead of the "no
        override" it means.
        """
        lc = getattr(pad, 'local_clearance', 0.0) or 0.0
        ncl = self.net_floor.get(getattr(pad, 'net_id', 0) or 0, 0.0)
        layers = None
        if self.layer_rules:
            from check_drc import pad_copper_layers
            layers = frozenset(pad_copper_layers(pad, self.board_copper))
        return PadFloor(float(ncl), float(lc), layers)

    def max_floor(self, floor) -> float:
        """An UPPER BOUND on what this pad can require of any partner, for the
        broad phases only. Never the requirement itself: a dru rule REPLACES, so
        it can also LOWER the pair value below `base`, and a broad phase that
        under-reaches drops real pairs (over-reaching only costs a test)."""
        v = max(floor.ncl, floor.lc)
        if self.layer_rules and floor.layers:
            for l in floor.layers:
                r = self.layer_rules.get(l)
                if r is not None and r > v:
                    v = r
        return v

    # -- the pair -------------------------------------------------------------
    def pair(self, fa, fb) -> float:
        """The required clearance between two pads (mm)."""
        return self.pair_with_source(fa, fb)[0]

    def pair_with_source(self, fa, fb):
        """(required clearance in mm, what set it).

        The source is recorded AT the assignment, never reconstructed from the
        final value. Reconstruction is wrong under the dru's REPLACE semantics:
        with netclass 0.5 and a layer rule replacing it to 0.3, the value 0.3
        still satisfies `max(ncl) >= eff`, so an after-the-fact test attributes
        it to the net class when the RULE set it. Disclosure is the whole point
        of carrying this, so it has to be exact.

        The empty source means "the board-wide floor" -- the same test
        check_drc's `_mark_required` applies before disclosing anything.
        """
        eff = self.base
        src = ''
        if fa.ncl > eff:
            eff, src = fa.ncl, 'netclass'
        if fb.ncl > eff:
            eff, src = fb.ncl, 'netclass'
        if self.layer_rules:
            key = (fa.layers, fb.layers, eff)
            cached = self._pair_cache.get(key)
            if cached is None:
                from check_drc import pads_shared_layer_clearance
                cached = pads_shared_layer_clearance(
                    eff, self.layer_rules, fa.layers or (), fb.layers or ())
                self._pair_cache[key] = cached
            if cached != eff:
                # REPLACE: a rule may raise OR lower, and either way it is the
                # rule that decided the value.
                eff, src = cached, 'layer rule'
        if fa.lc > eff:
            eff, src = fa.lc, 'pad override'
        if fb.lc > eff:
            eff, src = fb.lc, 'pad override'
        if eff <= self.base + 1e-9:
            # check_drc's `_mark_required` threshold exactly, not legality's
            # 1e-6 EPS: a requirement between the two would be disclosed by one
            # grader and not the other.
            src = ''
        return eff, src

    # -- the TRACK-vs-track pair (#735) ---------------------------------------
    def track_pair(self, net_a: int, net_b: int, resolved: float):
        """(required mm, the TrackRule that raised it or None) for one pair of
        TRACKS -- `resolved` raised by any binding `.kicad_dru` track rule.

        A SEPARATE resolver from `pair`, not a flag on it, because this is the
        only pair kind a track rule can bind: KiCad's condition is
        `A.Type=='track' && B.Type=='track'`, so a pad or a via on either side
        exempts the pair, and every other requirement this model answers has
        one. Callers pass the value `pair()` already gave them: the rule is the
        LAST tier and raise-only, exactly the order check_drc composes it in.

        Keyed on NETS, never on floors, so it is correct to call with a
        `resolved` that came from the flat fallback -- which is what happens on
        a board whose only declaration IS a track rule, where the model is
        `active is False` and no floor resolves at all.
        """
        if not self.track_rules:
            return resolved, None
        from kicad_dru import track_pair_clearance
        return track_pair_clearance(self.track_rules,
                                    self.net_classes.get(net_a, ()),
                                    self.net_classes.get(net_b, ()), resolved)


def resolve_npth_floor(pcb_data, pcb_file: str = None,
                       notes: list = None) -> float:
    """The board's copper-to-NPTH-hole FLOOR: the JLC fab floor, raised to the
    board's own declared `min_hole_clearance` when it declares more (#761).

    The one resolver, and it lives OUTSIDE `PartPads` on purpose. That class
    takes a footprint and scalars and reads no board -- a contract
    `tests/test_730_...py` pins, because six call sites build parts and a
    board read inside would change what each of them means. So the board is
    read here, once, by the caller that has one, and the resolved float is
    passed down. That is exactly the shape `fanout_clearance` already uses
    (`self.npth_floor`, fanout_clearance.py:664-666).

    `check_drc`'s requirement is
    `max(clearance, NPTH_TO_TRACK_CLEARANCE, hole_clearance, lc)`
    (check_drc.py:2714, :2733); this supplies the middle two terms.

    Measured over the 22 tracked boards: `flat_hierarchy` is the ONLY one
    declaring above the fab floor (0.2500), and all 6 of its NPTH pads carry
    `local_clearance` 0.0 -- so it is the single witness that this term fires
    at all, and every other board resolves to 0.2000.

    `pcb_file` follows the #498 rule every sibling here follows -- the
    CALLER's path when it has one, else `PCBData.source_path`. It is not
    decoration: a board staged into a temp dir and parsed from there carries a
    `source_path` that is not the board the caller means, and flat_hierarchy
    staged that way resolves 0.2000 where the real path resolves 0.2500 --
    which is exactly the 2.3500 -> 2.4000 radius this term exists for.

    Returns the plain fab floor when the board cannot be read, which is what a
    caller with no board gets by default. `notes` collects WHY, if a list is
    given: a silent fallback here drops the modelled floor by up to the
    declared value and reports a byte-identical census, which is the shape of
    silence this issue was filed about.
    """
    import routing_defaults as defaults
    floor = float(defaults.NPTH_TO_TRACK_CLEARANCE)
    if pcb_data is None:
        return floor
    try:
        from obstacle_map import resolve_hole_clearance
        return max(floor,
                   float(resolve_hole_clearance(pcb_data, None, pcb_file)))
    except Exception as e:                                       # noqa: BLE001
        if notes is not None:
            notes.append('copper-to-hole floor unresolved (%s: %s); the NPTH '
                         'keep-out falls back to the %.2fmm fab floor'
                         % (type(e).__name__, e, floor))
        return floor


class PartPads:
    """Pose-owning pad/hole model for one footprint.

    Offsets are captured in a seed-relative frame and re-derived per DELTA
    rotation (the `_Cap.pad_rects` pattern) because parser `pad.global_x` is
    stale after an in-memory footprint move -- this class owns the pose->pad
    transform. Half-extents fold `rect_rotation` into an axis-aligned bbox,
    exact for the axis-aligned pads that dominate placement and conservative
    for the rest.

    With a `PadClearanceModel` (#697) each copper pad also carries its
    required-clearance inputs in `pad_floors`, INDEX-ALIGNED with `pads_local`
    and therefore with everything `pad_rects` emits. Without one -- the default,
    and what the silk (labels.py) and extent-only (routability.py) consumers
    pass -- `pad_floors` is empty and `max_floor` is 0.0, so every caller keeps
    its original flat-scalar behaviour exactly.

    `pad_rects`' 6-tuple is deliberately NOT widened to carry the floor: it is a
    public shape (render_placement slices `r[:4]`, two test modules unpack it
    positionally) and the rect index already addresses the pad.

    NPTH holes are carried TWICE (#730), index-aligned: `holes_local` at the
    copper KEEP-OUT radius that `hole_circles()` serves, and `holes_extent` at
    the radius `extent_local()` needs. They differ only by the hole pad's own
    `local_clearance`, and only when a caller asks for it -- but the two
    questions are different in kind ("how close may copper come" versus "where
    is this part"), and answering the second with the first lets an author's
    keep-clear declaration push a part off the board outline.
    """

    __slots__ = ('ref', 'side', 'has_tht', 'seed_rot', 'pads_local',
                 'holes_local', 'holes_extent', 'n_pads', '_pad_cache',
                 '_hole_cache', '_keepout_cache', '_ext_cache', 'pad_floors',
                 'max_floor', 'clearance', 'hole_reach', 'holes_req',
                 'pad_sides', '_ext_side_cache')

    def __init__(self, fp, clearance: float, model=None,
                 copper_holes: bool = True, npth_floor: float = None):
        # Local imports: check_drc pulls the whole DRC stack (see the
        # BoardOutlineGate note); kicad_parser is cheap but keeps the module's
        # import surface unchanged for existing consumers.
        from kicad_parser import pad_drill_circles
        import routing_defaults as defaults

        self.ref = fp.reference
        self.side = footprint_side(fp)
        self.has_tht = footprint_has_through_pads(fp)
        self.seed_rot = (fp.rotation or 0.0) % 360
        # The flat clearance this part was built at. Stored (#761) because the
        # NPTH keep-out radius is only complete WITH it -- see hole_keepouts --
        # and a consumer that had to supply it separately is a consumer that
        # can forget to, which is exactly how the keep-out came to be graded
        # without one.
        self.clearance = float(clearance)
        self.pads_local = []    # (off_x, off_y, half_x, half_y, net_id, pside)
        self.holes_local = []   # (off_x, off_y, radius) -- NPTH keepouts, inflated
        self.holes_extent = []  # ...the same holes at their EXTENT radius (#730)
        self.holes_req = []     # ...and the REQUIREMENT each one resolved to (#761)
        self.pad_floors = []    # PadFloor per copper pad, index-aligned (#697)
        self.max_floor = 0.0    # upper bound on this part's pad requirements
        # `copper_holes` gates the BOARD floor exactly as it gates the
        # pad's own override, and for the same reason: a declared
        # `min_hole_clearance` is a COPPER rule, and KiCad has no silk-to-hole
        # rule at all.
        #
        # It is HARDENING, not a defect fix, and the commit that added it
        # (#761) claimed otherwise -- corrected here rather than left
        # standing. `labels.py:303`, the only `copper_holes=False` caller,
        # passes no `npth_floor` at all, so silk took the flat fab floor with
        # or without this gate; measured, silk output is byte-identical across
        # the whole change. What the silk arm caught is a latent hole that any
        # future silk caller supplying a floor would fall into, which is worth
        # closing but is not what the commit said it was.  The arm still
        # asserts against an OFFERED floor rather than the default, because
        # that is the only way to see this branch at all.
        _npth_floor = (defaults.NPTH_TO_TRACK_CLEARANCE
                       if (npth_floor is None or not copper_holes)
                       else max(defaults.NPTH_TO_TRACK_CLEARANCE,
                                float(npth_floor)))
        for p in fp.pads:
            if not _pad_carries_copper(p):
                # Copper-less drilled pad (NPTH mounting hole): the DRILL still
                # removes copper closer than the NPTH-to-track floor. A
                # paste/mask-only aperture has neither copper nor a hole and
                # simply drops out.
                #
                # The inflation matches fanout_clearance's foreign-pad
                # keepouts -- and the inflation is only HALF of the rule
                # (#761). What is stored here is the growth ABOVE `clearance`;
                # the requirement is that radius PLUS `clearance`, which each
                # sibling adds at its own gap test: fanout_clearance through
                # `_pair_or_flat` (an NPTH tuple files floor `None`, so it
                # hands back the flat scalar, fanout_clearance.py:894/1306),
                # and labels.py by comparing against `silk_pad_clearance`
                # (labels.py:399). legality compared this circle against RAW
                # pad rects and added nothing, so its modelled standoff was
                # `max(0, requirement - clearance)` -- which collapses to ZERO
                # once `clearance` reaches the requirement, at 0.20 for a plain
                # hole and 0.40 for ulx3s's AUDIO1. The inflation matched; the
                # COMPARISON did not, and only the inflation was ever checked.
                # `hole_keepouts()` is now the one place that composes both.
                #
                # #730: ...or closer than the pad's OWN `(clearance ...)`
                # override, when that is larger. That is what KiCad enforces
                # and what check_drc grades (`max(npth_clr, lc)`,
                # check_drc.py:2733). PER-PAD, so it is computed per pad and no
                # longer hoisted above the loop -- a footprint may carry
                # several holes with different overrides, and a hoisted value
                # would give them all the first one's answer. Raise-only:
                # 20 of the 22 tracked boards carry no NPTH override at all,
                # and watchy's 8 are 0.100, under the 0.20 fab floor, so it is
                # the negative control rather than a second example. The board
                # that moves is ulx3s -- AUDIO1's two 1.7mm holes at lc 0.400,
                # radius 0.950 -> 1.150 at --clearance 0.1.
                #
                # `copper_holes` is False for a SILK caller and that is not a
                # tuning knob. `hole_circles()` answers two different
                # questions: legality's copper consumers want the override,
                # and labels.py uses the same circles to keep a reference
                # designator's INK off a drill. `local_clearance` is a copper
                # rule -- KiCad has no silk-to-hole rule at all -- so applying
                # it there would push ulx3s's labels 0.20mm further from
                # AUDIO1 for a reason that does not exist. Naming the question
                # in the signature rather than inferring it from `model`:
                # measured, only two of the six `build_part_pads` call sites
                # pass one (grade_pad_legality and quench), so a model gate
                # would leave legality's own pad-overlap path, routability and
                # seeder unfixed while looking principled -- and it would key
                # a COPPER-vs-SILK question on an argument that answers a
                # different one.
                #
                # The BOARD's declared min_hole_clearance, which
                # check_drc's `npth_clr` carries, arrives as the resolved
                # scalar `npth_floor` (#761) -- NOT as a board pointer. This
                # class still reads no board; `resolve_npth_floor` does, once,
                # in the caller that has one. It defaults to the fab floor, so
                # a caller that passes nothing keeps exactly its previous
                # answer.
                if _pad_has_no_copper(p) and (p.drill or 0) > 0:
                    _lc = ((getattr(p, 'local_clearance', 0.0) or 0.0)
                           if copper_holes else 0.0)
                    npth_grow = max(0.0, max(_npth_floor, _lc) - clearance)
                    # ...and the SAME holes without the override, for extents.
                    # `holes_local` is a copper KEEP-OUT and `extent_local` is
                    # a question about where the part physically IS -- an
                    # author's copper keep-clear must not make a part read as
                    # hanging off the board outline. Measured on
                    # splitflap_driver at --clearance 0.25, injecting lc on
                    # H6/H7's NPTH pads only: without this split, lc 0.90 takes
                    # `oob_pad_count` from 0 to 2 and lc 0.95 puts both refs in
                    # the seeder's ZERO-margin off-board census, which
                    # CLAUDE.md calls the top-priority placement defect. This
                    # keeps every extent byte-identical to pre-#730.
                    # The FLAT floor, deliberately (#761): a board's
                    # declared copper-to-hole rule is a copper rule, exactly
                    # like `local_clearance` above, and `extent_local` asks
                    # where the part physically IS. Keeping it out here keeps
                    # every extent byte-identical on every board, which is what
                    # stops a declaration from pushing a part off the outline.
                    ext_grow = max(0.0, defaults.NPTH_TO_TRACK_CLEARANCE
                                   - clearance)
                    # The resolved requirement itself, for DISCLOSURE
                    # (#761). Derivable from the radius only with `hd/2` in
                    # hand, which no consumer has -- and a census that counts
                    # a conflict without naming what it required is the same
                    # shape of silence #697 fixed for the pad channel.
                    npth_req = max(clearance, _npth_floor, _lc)
                    for hx, hy, hd in pad_drill_circles(p):
                        self.holes_local.append(
                            (hx - fp.x, hy - fp.y, hd / 2.0 + npth_grow))
                        self.holes_extent.append(
                            (hx - fp.x, hy - fp.y, hd / 2.0 + ext_grow))
                        self.holes_req.append(npth_req)
                continue
            copper = [l for l in p.layers if str(l).endswith('.Cu')]
            through = (p.drill or 0) > 0
            # #834: a pad on BOTH faces reads as through, not as back-only.
            # An SMD pad listed on `*.Cu` (or explicitly on F.Cu and B.Cu) is
            # not drilled, so the old expression fell to its `'B'` arm and
            # declared the F copper away. That was harmless while the
            # over-cap branch ignored sides entirely; once `pair_shortfall`
            # SKIPS a pair whose pad sides are disjoint, it becomes a false
            # ACCEPT -- which this module's contract forbids outright (see the
            # class docstring: it may falsely reject, never falsely accept).
            # Measured a no-op on the corpus: zero such pads on any of the 22
            # tracked boards, so this is hardening, not a behaviour change.
            # `*.Cu` is KiCad's ALL-copper wildcard, so it is both faces on its
            # own -- and it read as FRONT, because it does not start with 'B'.
            _star = any(str(l).startswith('*') for l in copper)
            _b = any(str(l).startswith('B') for l in copper)
            _f = any(not str(l).startswith(('B', '*')) for l in copper)
            pside = (None if (through or _star or (_b and _f))
                     else ('B' if _b else 'F'))
            # THE one formula (see its docstring): PartPads was a fourth
            # copy of it, and `part_copper_geometry` keys a pad's own rect
            # on the same call, so a lookup cannot disagree with the box it
            # is looking up.
            phx, phy = pad_half_extents(p)
            self.pads_local.append((p.global_x - fp.x, p.global_y - fp.y,
                                    phx, phy, p.net_id, pside))
            if model is not None:
                floor = model.pad_floor(p)
                self.pad_floors.append(floor)
                mf = model.max_floor(floor)
                if mf > self.max_floor:
                    self.max_floor = mf
        self.n_pads = len(self.pads_local)
        # How far this part's hole keep-outs reach BEYOND its own extent
        # (#761) -- the broad-phase bound, 0.0 with no holes.
        #
        # Both early-outs drop a pair on the gap between EXTENT boxes, and
        # `max_floor` is built from copper pads only: an NPTH pad `continue`s
        # above, before `pad_floors.append`, so a hole's requirement reaches
        # neither bound. Measured on ulx3s: AUDIO1.max_floor is 0.0 while its
        # holes require 0.400, so at `model.base` 0.25 every pair at an extent
        # gap in [0.25, 0.40) returned ZERO_SHORTFALL before the hole loop ran
        # -- i.e. the keep-out fix above is INERT without this term.
        #
        # Soundness: an extent box contains its hole at `extent_r`, so
        # `rect_gap(ea, eb) >= dist(hole_centre, foreign_rect) - extent_r`,
        # while a penetration needs `dist < keepout_r`. Dropping at
        # `gap >= reach` is therefore safe exactly when
        # `reach >= keepout_r - extent_r`, which is this.
        self.hole_reach = 0.0
        for (_hx, _hy, _kr), (_ex, _ey, _er) in zip(self.holes_local,
                                                    self.holes_extent):
            over = (_kr + self.clearance) - _er
            if over > self.hole_reach:
                self.hole_reach = over
        # #834: the board sides this part's COPPER PADS occupy -- a through
        # pad (`pside is None`) occupies both. Deliberately NOT
        # `sides_occupied(self.side, self.has_tht)`, which answers the BODY
        # question and is always a superset: glasgow_revC's J5 has pads on F
        # only but two NPTH holes, so its body occupies both faces, and
        # pricing pad clearance at the body's answer would make this fix inert
        # on the one over-cap part in the corpus that carries holes.
        _sides = set()
        for _t in self.pads_local:
            if _t[5] is None:
                _sides = BOTH_SIDES
                break
            _sides.add(_t[5])
        self.pad_sides = (BOTH_SIDES if _sides is BOTH_SIDES else
                          FRONT_ONLY if _sides == {'F'} else
                          BACK_ONLY if _sides == {'B'} else
                          BOTH_SIDES if _sides else NO_SIDES)
        self._pad_cache: Dict[float, list] = {}
        self._hole_cache: Dict[float, list] = {}
        self._keepout_cache: Dict[float, list] = {}
        self._ext_cache: Dict[float, Tuple[float, float, float, float]] = {}
        self._ext_side_cache: Dict[Tuple[float, str], tuple] = {}

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
        """[(cx, cy, radius)] NPTH hole circles at the given pose, at the
        INFLATION radius -- the growth above `clearance`, not the requirement.

        A copper consumer wants `hole_keepouts` instead. This accessor exists
        for the caller that adds its own term at its own gap test: labels.py
        (silk, `< config.silk_pad_clearance`, labels.py:399).
        """
        key = self._delta_key(rot)
        cache = self._hole_cache.get(key)
        if cache is None:
            rad = math.radians(-key)
            c, s = math.cos(rad), math.sin(rad)
            cache = [(ox * c - oy * s, ox * s + oy * c, r)
                     for ox, oy, r in self.holes_local]
            self._hole_cache[key] = cache
        return [(x + ox, y + oy, r) for ox, oy, r in cache]

    def hole_keepouts(self, x: float, y: float, rot: float):
        """[(cx, cy, radius)] NPTH COPPER keep-out circles at the given pose.

        The one resolver for "how close may foreign copper come to this hole"
        (#761). It is `hole_circles`' radius PLUS the flat clearance, because
        the stored growth is `max(0, requirement - clearance)`; adding the
        clearance back composes, per hole, to

            hd/2 + max(clearance, NPTH floor, the hole pad's local_clearance)

        -- a MAX rather than a sum, since the growth is already floored at 0.
        That is `check_drc`'s own requirement (`max(npth_clr, lc)`,
        check_drc.py:2733) and what fanout_clearance's cap gate charges.

        Tested against RAW pad rects, so the whole requirement lives in this
        radius: `_circle_rect_penetration` stays a pure geometric predicate
        with no clearance parameter to forget.
        """
        key = self._delta_key(rot)
        cache = self._keepout_cache.get(key)
        if cache is None:
            clr = self.clearance
            cache = [(ox, oy, r + clr)
                     for ox, oy, r in self.hole_circles(0.0, 0.0, rot)]
            self._keepout_cache[key] = cache
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
            # #730: the EXTENT radii, not the keep-out ones -- see the note
            # beside holes_extent. `_rotated`-style rotation done inline
            # because holes rotate about the part origin exactly as
            # hole_circles rotates them.
            rad = math.radians(-key)
            hc, hs = math.cos(rad), math.sin(rad)
            for ox, oy, r in self.holes_extent:
                cx, cy = ox * hc - oy * hs, ox * hs + oy * hc
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

    def extent_local_side(self, rot: float, side: str):
        """`extent_local`, restricted to the pads that occupy `side` (#834).

        The over-cap branch of `pair_shortfall` grades a pair on the gap
        between EXTENT boxes, and a whole-part extent spans both faces -- so a
        BGA on F was charged for a part on B whenever the pad-pair product
        crossed `PAIR_TEST_CAP`, and only then. This is the same box, built
        over the pads that `_sides_interact` would have admitted.

        Holes go into BOTH sides' boxes: a drill removes copper on every
        layer, so it is not a side's to exclude. That also makes this box
        equal to `extent_local` for a part whose pads all occupy one side --
        which is why no same-side pair can move (measured: every one of the 35
        same-side over-cap pairs on the tracked corpus, at four rotations, is
        bit-identical).

        `holes_extent` is rotated INLINE here for the same reason
        `extent_local` does it: `hole_circles()` serves `holes_local`, the
        keep-OUT radii, and substituting them would inflate every extent on
        every board.
        """
        key = (self._delta_key(rot), side)
        ext = self._ext_side_cache.get(key)
        if ext is None:
            xs0, ys0, xs1, ys1 = [], [], [], []
            for ox, oy, HX, HY, _n, _s in self._rotated(rot):
                if not _sides_interact(_s, side):
                    continue
                xs0.append(ox - HX); ys0.append(oy - HY)
                xs1.append(ox + HX); ys1.append(oy + HY)
            rad = math.radians(-key[0])
            hc, hs = math.cos(rad), math.sin(rad)
            for ox, oy, r in self.holes_extent:
                cx, cy = ox * hc - oy * hs, ox * hs + oy * hc
                xs0.append(cx - r); ys0.append(cy - r)
                xs1.append(cx + r); ys1.append(cy + r)
            ext = () if not xs0 else (min(xs0), min(ys0), max(xs1), max(ys1))
            self._ext_side_cache[key] = ext
        return ext or None

    def extent_side(self, x: float, y: float, rot: float, side: str):
        e = self.extent_local_side(rot, side)
        if e is None:
            return None
        return (x + e[0], y + e[1], x + e[2], y + e[3])


def build_part_pads(footprints: Dict[str, object],
                    clearance: float, model=None,
                    copper_holes: bool = True,
                    npth_floor: float = None,
                    tolerant: bool = False) -> Dict[str, 'PartPads']:
    """PartPads for every footprint that has any pad (copper or NPTH).

    `model` is an optional `PadClearanceModel` (#697); without it the parts
    carry no per-pad clearance floors and every consumer behaves exactly as
    before.

    `tolerant` (#841) SKIPS a footprint whose pad model cannot be built rather
    than losing the whole map to it. Default False, so the six existing call
    sites are bit-identical: a gate that silently drops a part it could not
    model is worse than one that raises. It exists for the lane ledgers, whose
    callers include hand-built test fixtures that carry no `reference` -- there
    a missing part degrades to the pad-centre box it always had, and
    `CopperGeometry.modelled` says which parts those are.
    """
    out = {}
    for ref, fp in footprints.items():
        if not getattr(fp, 'pads', None):
            continue
        try:
            pp = PartPads(fp, clearance, model, copper_holes, npth_floor)
        except Exception:                                    # noqa: BLE001
            if not tolerant:
                raise
            continue
        if pp.n_pads or pp.holes_local:
            out[ref] = pp
    return out


class CopperGeometry(NamedTuple):
    """One part's copper, as both lane ledgers charge it (#841)."""
    ref: str
    #: `PartPads.extent`: pad copper UNION the NPTH hole extents. This is the
    #: part's obstruction box -- what it contributes as a NEIGHBOUR, and the
    #: box its own faces are measured on, so two parts facing each other
    #: cannot disagree about where the channel between them is.
    rect: Tuple[float, float, float, float]
    #: The union of the PAD rects alone, never wider than `rect`. Face
    #: ASSIGNMENT is measured against this one, because every edge of it is
    #: attained by some pad -- which is what keeps an edge pad at distance
    #: exactly 0. `rect` does not have that property: measured, on 12 of the
    #: 388 edges of the corpus's fine-pitch parts the extreme edge is set by
    #: an NPTH hole and no pad reaches it -- watchy SW1-SW4 (8 edges, 0.400mm)
    #: and rp2350 J2 (4, up to 1.372mm). Regenerate with
    #: `tests/test_841_obstruction_rect.py`, which prints the count.
    #:
    #: It read 14 for one commit, with ulx3s U1 (2 edges, 0.1905mm) named as a
    #: third hole part. U1 has no holes: those two edges were `copper` losing
    #: four stacked pad rects to a lookup keyed on the centre alone, and the
    #: number was corrected in the docstring before the bug behind it was
    #: found. A number that disagrees with another number is not always the
    #: one to change.
    copper: Tuple[float, float, float, float]
    #: `{(cx, cy, half_x, half_y): (x0, y0, x1, y1)}` -- each copper pad's own
    #: rect at this pose, keyed by its centre AND its half-extents (rounded to
    #: 1e-4 mm) so a caller holding a `Pad` can find it. NOT indexed by
    #: position: `PartPads` drops pads that carry no copper, so its order does
    #: not align with `fp.pads`.
    #:
    #: The size is in the key because the centre alone is not unique. Pads
    #: STACKED at one point are ordinary -- a cross-shaped alignment mark, a
    #: thermal pad drawn as several rects -- and keying on the centre gave the
    #: last one to whichever pad asked: measured, glasgow_revC U1's 0.6mm
    #: GND pad 57 was handed a co-located 6.22mm box, and rp2350 U6's pad 61
    #: a 3.4mm one.
    #:
    #: Measured over the tracked corpus, 383 of 9491 pads have no rect and
    #: every one of them is copper-less; no NETTED pad misses, and none is now
    #: handed another pad's box.
    pads: Dict[Tuple[float, float], Tuple[float, float, float, float]]
    #: False when the pad model could not be built and `rect`/`copper` are the
    #: pad-CENTRE bbox instead -- today's answer, kept for the caller that has
    #: no `PartPads` behind its footprints.
    modelled: bool


def part_copper_geometry(footprints: Dict[str, object], clearance: float, *,
                         parts: Optional[Dict[str, 'PartPads']] = None
                         ) -> Dict[str, CopperGeometry]:
    """{ref: CopperGeometry} at each footprint's own pose -- THE lane-ledger
    obstruction geometry (#841).

    One definition, because two lane ledgers asking "what stops a track
    leaving this face" answered it with two different rectangles: `escape`
    charged the bbox of pad CENTRES (a 0.9mm passive terminal contributing a
    zero-width body), `routability.face_lane_ledger` charged the COURTYARD (an
    assembly keep-out a track may legally run under). Neither is copper.

    `clearance` is NOT decoration. `holes_extent` carries
    `max(0, NPTH_TO_TRACK_CLEARANCE - clearance)` (see `extent_local`), so
    below 0.20mm the hole box grows: measured over the 22 tracked boards, 19
    parts on 6 boards differ between clearance 0.05 and 0.20, by up to
    0.150mm, and nothing differs between 0.20 and 0.40. Pass the clearance you
    priced the LANE at -- `check_channels` runs the corpus at 0.09, squarely
    inside that regime, so two ledgers on different clearances would disagree
    exactly where the starved-face gate lives.

    A pad-LESS footprint is absent from the result (as it is from
    `build_part_pads`). That is deliberate and it closes #841's own hazard:
    the +/-0.5mm `synthetic` fiction `part_local_bounds` invents for a
    footprint with neither courtyard nor pads can no longer reach a lane
    SUPPLY, which `options.move_blocker` turns into an instruction to move a
    part. Measured over the whole tracked corpus, the refs this drops from
    `face_lane_ledger`'s neighbour list are EXACTLY that board's `synthetic`
    set -- set equality, on every board that has one: ulx3s 9, glasgow_revC 8,
    orangecrab 3, tigard 3, esp_prog 3, watchy 2, and interf_u_unrouted /
    interf_u_unrouted_placed 1 each. Those last two carry no fine-pitch part,
    so no default run reaches them; they are named because "and none
    elsewhere" is what was written here first, and it was wrong.
    """
    if parts is None:
        parts = build_part_pads(footprints, clearance, tolerant=True)
    else:
        # `parts` decides the NPTH hole growth, not `clearance` -- the boxes
        # are already built. A caller handing in a map built at a different
        # clearance would get hole extents up to 0.2mm off the value it just
        # asked for, silently, and the docstring above spends a paragraph
        # promising the opposite. `PartPads` records the clearance it was
        # built at, so the disagreement is checkable rather than trusted.
        for _pp in parts.values():
            if abs(float(_pp.clearance) - float(clearance)) > 1e-9:
                raise ValueError(
                    'part_copper_geometry: parts were built at clearance '
                    '{} but {} was requested; the NPTH hole extents would be '
                    'the former and every caller would read the latter'
                    .format(_pp.clearance, clearance))
            break
    out: Dict[str, CopperGeometry] = {}
    for ref, fp in footprints.items():
        if not getattr(fp, 'pads', None):
            continue
        pp = parts.get(ref)
        ext = None if pp is None else pp.extent(fp.x, fp.y, fp.rotation or 0.0)
        if pp is None or ext is None:
            xs = [p.global_x for p in fp.pads]
            ys = [p.global_y for p in fp.pads]
            centre = (min(xs), min(ys), max(xs), max(ys))
            out[ref] = CopperGeometry(ref=ref, rect=centre, copper=centre,
                                      pads={}, modelled=False)
            continue
        boxes = [(r[0], r[1], r[2], r[3])
                 for r in pp.pad_rects(fp.x, fp.y, fp.rotation or 0.0)]
        # The UNION comes from the full list, never from the lookup dict.
        # Keying it by centre alone collapsed pads STACKED at one point -- a
        # cross-shaped alignment pad, a thermal pad drawn as several rects --
        # so `copper` silently lost them: measured, ulx3s U1 dropped 4 of its
        # 389 rects and came out 0.1905mm short on two sides, and glasgow U1's
        # 0.6mm pad 57 was handed a co-located 6.22mm box. A dict cannot be
        # the union AND the lookup; it is now only the lookup.
        if boxes:
            copper = (min(b[0] for b in boxes), min(b[1] for b in boxes),
                      max(b[2] for b in boxes), max(b[3] for b in boxes))
        else:
            copper = ext                      # holes only: no pad copper at all
        # ...and the lookup key carries the pad's SIZE as well as its centre,
        # so two pads at one point are distinguishable and a hit is the pad's
        # own box. BOTH sides of that key come from `pad_half_extents(pad)` --
        # here and in `pad_box` -- rather than one from the pad and one from
        # the finished rect: measured, re-deriving the half-extent as
        # `(x1 - x0) / 2` puts esp_prog U2 pad 2 (1.5019mm wide) at 0.7509
        # against the pad's own 0.751, and it stops being findable.
        #
        # `pads_local` is built by walking `fp.pads` and keeping the pads that
        # carry copper, so `pad_rects` is index-aligned with that same filter;
        # the zip below is that correspondence, and the length check is what
        # refuses to guess if it ever stops holding.
        copper_pads = [q for q in fp.pads if _pad_carries_copper(q)]
        rects = {}
        if len(copper_pads) == len(boxes):
            for q, box in zip(copper_pads, boxes):
                qhx, qhy = pad_half_extents(q)
                rects[(round(q.global_x, 4), round(q.global_y, 4),
                       round(qhx, 4), round(qhy, 4))] = box
        out[ref] = CopperGeometry(ref=ref, rect=ext, copper=copper,
                                  pads=rects, modelled=True)
    return out


def pad_box(geom: CopperGeometry, pad) -> Optional[Tuple[float, float,
                                                         float, float]]:
    """`pad`'s own copper rect within `geom`, or None when it has none.

    The one lookup, so a caller does not re-derive a pad's half-extents from
    `size_x`/`size_y`/`rect_rotation` and end up with a box that disagrees with
    the `copper` union built from `PartPads`. None means the pad carries no
    copper (a paste aperture, an NPTH mounting hole) -- measured, no NETTED pad
    on the tracked corpus answers None.
    """
    if not geom.pads:
        # An unmodelled part (`modelled=False`) has no pad boxes at all, and
        # asking one for its half-extents is how this reached out and touched
        # a `size_x` the caller's footprints do not carry -- the hand-built
        # fixtures in `tests/test_escape_ledger.py` are exactly that caller.
        return None
    hx, hy = pad_half_extents(pad)
    return geom.pads.get((round(pad.global_x, 4), round(pad.global_y, 4),
                          round(hx, 4), round(hy, 4)))


def _sides_interact(a, b) -> bool:
    """Do two pad sides share copper? None = through (both sides)."""
    return a is None or b is None or a == b


def _hole_shortfall(pa, xa, ya, ra, rects_b, pb, xb, yb, rb, rects_a):
    """Total penetration of either part's NPTH keep-outs into the other's
    copper (#761). ONE resolver, because `pair_shortfall` reaches it from two
    branches -- the full pad sweep and the PAIR_TEST_CAP extent branch -- and
    a second copy is how the cap branch came to report a hardcoded zero.

    Keep-outs, not `hole_circles`: the requirement lives in the radius, and
    the rects are raw.
    """
    hole = 0.0
    for cx, cy, r in pb.hole_keepouts(xb, yb, rb):
        for a0, a1, a2, a3, _na, _sa in rects_a:
            hole += _circle_rect_penetration(cx, cy, r, (a0, a1, a2, a3))
    for cx, cy, r in pa.hole_keepouts(xa, ya, ra):
        for b0, b1, b2, b3, _nb, _sb in rects_b:
            hole += _circle_rect_penetration(cx, cy, r, (b0, b1, b2, b3))
    return hole


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
                 pose_of, seed_of, model=None):
        self.parts = part_pads
        self.gate = gate
        self.clearance = clearance
        # #697: the per-pair required clearance, when the board declares one
        # above the flat scalar (pad override / netclass / .kicad_dru rule).
        # None -- the common case, and every board that declares nothing -- is
        # the original flat-scalar path, unchanged and untouched.
        self._floors = model if (model is not None and model.active) else None
        self.pose_of = pose_of
        self.seed_of = seed_of
        self._baselines: Dict[Tuple[str, str], PairShortfall] = {}
        # run-19: a PILE seed is not a license. Every conjunct of pads_ok is
        # relative to seed_baseline, and on a pile the seed pair carries huge
        # base.pad with pad_overlap and stack both True -- so ANY smaller
        # intersection passed (measured: the SW23/SW28<->U2 blocker pairs).
        # A seed bucket of >= 3 parts at one round(coord, 3) point is the
        # pile signature; every part in one loses its baseline license and
        # must be absolutely clean toward every neighbor. Deliberately above
        # 2: a legitimate two-part by-design overlap (run 18's under-module
        # R1<->U2 pairs) KEEPS its license.
        _buckets: Dict[Tuple[float, float], List[str]] = {}
        for _ref in part_pads:
            _sx, _sy = seed_of(_ref)[:2]
            _buckets.setdefault((round(_sx, 3), round(_sy, 3)),
                                []).append(_ref)
        self._degenerate_refs = frozenset(
            r for refs in _buckets.values() if len(refs) >= 3 for r in refs)

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
        model = self._floors
        # The pair's REACH: how far apart these two parts can still interact.
        # With per-pad floors it is bounded by THESE TWO PARTS' own maxima, not
        # by the board-wide maximum -- this test runs per candidate pose
        # (quench.candidate_valid -> pads_ok), so a board-wide bound would slow
        # every pair on the board for the sake of one fiducial.
        pad_reach = (self.clearance if model is None
                     else max(self.clearance, model.base,
                              pa.max_floor, pb.max_floor))
        # ...and the hole keep-outs, which no pad floor accounts for (#761).
        # Kept as a SEPARATE name: the cap branch below charges its shortfall
        # into the PAD channel, and folding the hole requirement into the
        # number it charges bills one physical violation to two independent
        # gates (`pads_ok` tests `cur.pad` and `cur.hole` as separate
        # conjuncts, and quench sums `sf.hole` on its own). Measured on one
        # hole at lc 1.0 with pads 0.30 off the wall, the only difference
        # being the pad-pair product: 60x60 -> pad 0.0000, 70x70 -> pad
        # 0.7000, hole 0.7000 in both.
        reach = max(pad_reach, pa.hole_reach, pb.hole_reach)
        if rect_gap(ea, eb) >= reach - EPS:
            return ZERO_SHORTFALL
        rects_a = pa.pad_rects(xa, ya, ra)
        rects_b = pb.pad_rects(xb, yb, rb)
        # #834: two parts whose pads share no copper face cannot interact
        # through the PAD channel at all, so answer it here rather than in
        # either branch below.
        #
        # This is EXACT, not conservative, which is what licenses it without
        # an EPS. `_sides_interact(a, b)` is `a is None or b is None or
        # a == b`, and a member of `pad_sides` is never None, so: if some pad
        # pair passes that filter then either one side is None (and that pad
        # contributes BOTH sides) or the two are equal -- either way the sets
        # intersect; and if a side s is in both sets then each part has a pad
        # with pside in {None, s}, which `_sides_interact` admits. Disjoint
        # therefore holds if and only if the per-pad sweep below would find
        # zero interacting pairs, so this returns exactly what it returns.
        #
        # Placed AFTER the broad-phase early-out and after the rect lists so
        # only pairs that were about to run the O(n*m) loop pay for the
        # intersection -- measured on ulx3s, ~0.14us against 310us for one
        # 778-pad-pair sweep it replaces.
        #
        # The HOLE channel is still measured: `reach` above includes
        # `hole_reach`, so a cross-side pair that interacts ONLY through a
        # drill survives the broad phase and lands here, and an NPTH hole
        # pierces both faces (see `footprint_has_through_pads`). Returning a
        # blanket ZERO_SHORTFALL here would delete #761's channel for exactly
        # those pairs. The singleton is returned only when neither part has a
        # hole, where `_hole_shortfall` is 0.0 by construction -- and that is
        # worth doing, because `pads_ok` short-circuits on its IDENTITY.
        if not (pa.pad_sides & pb.pad_sides):
            if not pa.holes_local and not pb.holes_local:
                return ZERO_SHORTFALL
            return PairShortfall(0.0, False,
                                 _hole_shortfall(pa, xa, ya, ra, rects_b,
                                                 pb, xb, yb, rb, rects_a),
                                 False)
        if pa.n_pads * pb.n_pads > PAIR_TEST_CAP:
            # Extent-level verdict for the PAD channel only: charge the extent
            # shortfall as pad shortfall so the baseline comparison still
            # constrains the pair.
            #
            # The HOLE channel is measured anyway (#761). The cap exists for
            # the pad x pad product; the hole loops are holes x pads, an order
            # smaller on the one part that trips it -- glasgow_revC's J5 is 44
            # pads and 2 NPTH drill circles against a 121-pad neighbour that
            # has none, so 5324 pad pairs against 242 hole tests, a factor of
            # 22. (First written as "4 holes / 660 tests / two orders", which
            # a fact-check re-measured; the numbers are here rather than in a
            # commit message so the next reader can check them.) Returning a
            # hardcoded `hole=0.0` here made `cur.hole > base.hole` unable to
            # fire for exactly the dense connectors that carry mounting holes,
            # and that is corpus-reachable, not theoretical: J5 is the one part
            # on the 22 tracked boards whose product exceeds the cap.
            #
            # ...which was wrong, and is corrected here (#834): 38 pairs on
            # NINE of those boards exceed the cap, and J5 is not even the only
            # over-cap part on glasgow (U1 x U30 is 83 x 121). The hole
            # argument above still holds -- J5 remains the only over-cap part
            # that CARRIES holes, which is what that paragraph is really
            # about. Re-derive both with
            # `tests/measure_834_835_side_awareness.py --table A`.
            #
            # The gap is taken PER SHARED SIDE, because the extent boxes above
            # span both faces and a pair reaching this branch may share only
            # one. Charging the whole-extent gap here priced a part on B
            # against a BGA on F -- the same pair the per-pad sweep below
            # discards on `_sides_interact` -- so the verdict turned on the
            # pad-pair PRODUCT, a performance switch, rather than on physics.
            # `min` because the pair interacts on whichever shared face brings
            # them closest; `default` because a bare `min()` over an empty
            # generator raises inside `quench.candidate_valid`'s inner loop,
            # and "the disjoint case already returned above" is exactly the
            # kind of unreachability that ships as a crash.
            #
            # `pad_overlap` and `stack` follow the same g. `stack` becoming
            # side-aware is the correction, not a side effect: the per-pad
            # sweep below sets it only AFTER `_sides_interact`, and
            # `render_placement` already prints front-to-back overlaps as
            # "opposite faces, NOT a conflict" while this branch refused them.
            g = min((rect_gap(pa.extent_side(xa, ya, ra, s),
                              pb.extent_side(xb, yb, rb, s))
                     for s in ('F', 'B')
                     if pa.extent_side(xa, ya, ra, s) is not None
                     and pb.extent_side(xb, yb, rb, s) is not None),
                    default=rect_gap(ea, eb))
            return PairShortfall(max(0.0, pad_reach - g), g < 0.0,
                                 _hole_shortfall(pa, xa, ya, ra, rects_b,
                                                 pb, xb, yb, rb, rects_a),
                                 g < 0.0)
        floors_a = pa.pad_floors if model is not None else None
        floors_b = pb.pad_floors if model is not None else None
        pad_short = 0.0
        overlap = False
        stack = False
        clr = self.clearance
        for ai, (a0, a1, a2, a3, na, sa) in enumerate(rects_a):
            fa = floors_a[ai] if floors_a else None
            for bi, (b0, b1, b2, b3, nb, sb) in enumerate(rects_b):
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
                # Cheap pre-reject before resolving the pair's requirement: it
                # can never exceed `reach`, so a gap at or beyond it is clear
                # whatever the two pads declare.
                if g >= reach - EPS:
                    continue
                eff = clr if fa is None else model.pair(fa, floors_b[bi])
                if g < eff - EPS:
                    pad_short += eff - g
                    if g < 0.0:
                        overlap = True
        # #761: keep-outs, not inflations -- the requirement is the stored
        # growth PLUS the flat clearance, and the rects below are raw.
        return PairShortfall(pad_short, overlap,
                             _hole_shortfall(pa, xa, ya, ra, rects_b,
                                             pb, xb, yb, rb, rects_a),
                             stack)

    def seed_baseline(self, a: str, b: str) -> PairShortfall:
        # The single choke point every consumer routes through (pads_ok and
        # swap_pads_ok): a part from a degenerate seed bucket gets the ZERO
        # baseline, which is exactly the semantics a pile deserves.
        if a in self._degenerate_refs or b in self._degenerate_refs:
            return ZERO_SHORTFALL
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


def format_required_clause(report, limit: int = 6) -> str:
    """One line naming the pairs graded ABOVE the board-wide clearance, and WHY.

    Lives beside the measure for the same reason `format_oob_clause` does: a
    printed basis hand-copied into each CLI is the Class-2 drift CLAUDE.md
    warns about, and no parity gate covers a print.

    Without this the #697 fix would trade one confusing report for another --
    the census would start counting a pair that is nowhere near the `--clearance`
    the run announced, with nothing on screen to explain the gap. Returns ''
    when every pair is graded at the flat scalar, which is the common case.
    """
    rows = list(report.get('required') or [])
    if not rows:
        return ''
    shown = ', '.join('{}<->{} requires {:g}mm ({})'.format(a, b, mm, src)
                      for a, b, mm, src in rows[:limit])
    more = ' ... showing {} of {}'.format(limit, len(rows)) if len(rows) > limit else ''
    return shown + more


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
                       worst_n: int = 10,
                       pcb_file: str = None) -> Dict[str, object]:
    """Board-level pad/hole legality audit at the FILE's own poses.

    AABB broad phase over all cross-footprint pad pairs; with `exact` (the
    default) every AABB hit is re-verified with check_drc's exact pad geometry
    so the report carries no bbox phantoms on round/rotated pads. Returns::

        {'pad_conflicts': int, 'pad_shortfall': mm, 'hole_conflicts': int,
         'oob_pad_count': int, 'oob_pad_amount': mm,
         'worst': [(refA, refB, mm), ...],
         'required': [[refA, refB, mm, source], ...], 'exact': bool}

    `clearance` is the BOARD-WIDE floor, not the whole requirement. Each pair is
    graded at check_drc's own value -- max over the two nets' net classes, the
    board's .kicad_dru per-layer rules, and either pad's `local_clearance`
    override -- resolved by `PadClearanceModel` (#697). Pairs whose requirement
    exceeds `clearance` are named in `required` with the source that raised
    them, mirroring check_drc's `required_mm` disclosure; a board that declares
    none of the three grades exactly as it did before.

    `worst` deliberately stays a 3-tuple: seeder's repair census unpacks it
    positionally, and the disclosure rides the separate `required` list.

    Consumers: place_optimize / place_seed JSON summaries, the render
    legality overlay, and the reconstruct gate.
    """
    fps = pcb_data.footprints
    pads_by_ref = {ref: [p for p in fp.pads] for ref, fp in fps.items()}
    model = PadClearanceModel.for_board(pcb_data, clearance, pcb_file)
    # Keep the notes even when the model is dropped: a source that FAILED to
    # read is exactly what makes the model look inert, so reading them off the
    # dropped object loses them in the one case they matter.
    clearance_notes = list(model.notes)
    if not model.active:
        model = None
    # The census reads hole keep-outs, so it carries the board's own
    # copper-to-hole floor. The three call sites that read only pad rects and
    # extents (legality's pad_intersection channel, routability, seeder's
    # off-board census) deliberately do not: for them the term would resolve a
    # board and change nothing.
    parts = build_part_pads(
        fps, clearance, model,
        npth_floor=resolve_npth_floor(pcb_data, pcb_file, clearance_notes))
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
        holes = pp.hole_keepouts(fp.x, fp.y, fp.rotation or 0.0)  # #761
        entries[ref] = (rects, holes)
        ext = pp.extent(fp.x, fp.y, fp.rotation or 0.0)
        if ext is None:
            continue
        for gx in range(int(ext[0] // cell), int(ext[2] // cell) + 1):
            for gy in range(int(ext[1] // cell), int(ext[3] // cell) + 1):
                grid.setdefault((gx, gy), set()).add(ref)

    # The census reach. Unlike the per-candidate gate this runs ONCE, and the
    # requirement is a max over BOTH members of a pair -- so the halo must cover
    # the largest floor any partner could contribute, which is the board-wide
    # maximum. Under-reaching here is how the original bug hid: a fiducial
    # keep-clear 1.016mm wide never entered a census bounded at 0.15 + 0.5.
    board_max_floor = max([pp.max_floor for pp in parts.values()] or [0.0])
    # ...plus the hole keep-outs (#761): same argument, and the same reason
    # the per-candidate gate needs it -- an NPTH hole contributes to no pad
    # floor, so a halo bounded by floors alone never reaches one.
    board_max_hole = max([pp.hole_reach for pp in parts.values()] or [0.0])
    census_reach = max(clearance, board_max_floor, board_max_hole,
                       model.base if model is not None else 0.0)

    def near_refs(ref):
        fp = fps[ref]
        pp = parts[ref]
        ext = pp.extent(fp.x, fp.y, fp.rotation or 0.0)
        if ext is None:
            return ()
        out = set()
        m = census_reach + 0.5
        for gx in range(int((ext[0] - m) // cell), int((ext[2] + m) // cell) + 1):
            for gy in range(int((ext[1] - m) // cell), int((ext[3] + m) // cell) + 1):
                out |= grid.get((gx, gy), set())
        out.discard(ref)
        return out

    pad_conflicts = 0
    pad_shortfall = 0.0
    hole_conflicts = 0
    worst: List[Tuple[str, str, float]] = []
    required: List[list] = []
    seen_pairs = set()
    for ref in sorted(parts):
        rects_a, holes_a = entries[ref]
        for other in near_refs(ref):
            key = (ref, other) if ref <= other else (other, ref)
            if key in seen_pairs:
                continue
            seen_pairs.add(key)
            rects_b, holes_b = entries[other]
            floors_a = parts[ref].pad_floors if model is not None else None
            floors_b = parts[other].pad_floors if model is not None else None
            pair_reach = (clearance if model is None else
                          max(clearance, model.base, parts[ref].max_floor,
                              parts[other].max_floor))
            pair_mm = 0.0
            pair_hit = False
            pair_required = 0.0
            pair_source = ''
            for ai, (a0, a1, a2, a3, na, sa) in enumerate(rects_a):
                fa = floors_a[ai] if floors_a else None
                for bi, (b0, b1, b2, b3, nb, sb) in enumerate(rects_b):
                    if na == nb and na > 0:
                        continue
                    if not _sides_interact(sa, sb):
                        continue
                    g = rect_gap((a0, a1, a2, a3), (b0, b1, b2, b3))
                    if g >= pair_reach - EPS:
                        continue
                    if fa is None:
                        eff, src = clearance, ''
                    else:
                        eff, src = model.pair_with_source(fa, floors_b[bi])
                    if g >= eff - EPS:
                        continue
                    if check_exact is not None:
                        pa = _pad_with_copper(pads_by_ref[ref], ai, clearance)
                        pb = _pad_with_copper(pads_by_ref[other], bi, clearance)
                        if pa is not None and pb is not None:
                            hit, over, _pt = check_exact(pa, pb, eff,
                                                         routing_layers,
                                                         clearance_margin=0.0)
                            if not hit:
                                continue
                            pair_mm += over
                            pair_hit = True
                            if eff > pair_required:
                                pair_required, pair_source = eff, src
                            continue
                    pair_mm += eff - g
                    pair_hit = True
                    if eff > pair_required:
                        pair_required, pair_source = eff, src
            if pair_hit:
                pad_conflicts += 1
                pad_shortfall += pair_mm
                worst.append((key[0], key[1], round(pair_mm, 4)))
                if pair_source:
                    required.append([key[0], key[1],
                                     round(pair_required, 4), pair_source])
            # #761: the hole channel discloses its requirement too. It used
            # to report a bare `hole_conflicts` count with the mm thrown away,
            # so a user saw that a mounting hole was too close to something
            # without ever being told how close it was allowed to be -- and
            # that number is not the board-wide clearance whenever the hole
            # pad declares an override or the board declares a floor.
            hole_pen = 0.0
            hole_req = 0.0
            for (cx, cy, r), req in zip(holes_b, parts[other].holes_req):
                for a0, a1, a2, a3, _na, _sa in rects_a:
                    pen = _circle_rect_penetration(cx, cy, r,
                                                   (a0, a1, a2, a3))
                    hole_pen += pen
                    if pen > EPS and req > hole_req:
                        hole_req = req
            for (cx, cy, r), req in zip(holes_a, parts[ref].holes_req):
                for b0, b1, b2, b3, _nb, _sb in rects_b:
                    pen = _circle_rect_penetration(cx, cy, r,
                                                   (b0, b1, b2, b3))
                    hole_pen += pen
                    if pen > EPS and req > hole_req:
                        hole_req = req
            if hole_pen > EPS:
                hole_conflicts += 1
                # Above the board-wide clearance only, so a plain hole at the
                # flat scalar stays quiet. NOT literally the pad channel's
                # bar, which is `pair_source != ''` and therefore sits at
                # `model.base` -- the two differ when a netclass lifts
                # `model.base` above `--clearance`, and "the same bar" would
                # be wrong in exactly that case.
                #
                # DISCLOSED, not fixed: the hole channel files into `required`
                # but not into `worst`, so a hole-ONLY conflict is printed by
                # `format_required_clause` and is invisible to
                # `floorplan.suspect_pairs`, which is built from `worst`.
                # Widening `worst` changes what `place_seed --repair` chooses
                # to move -- a placement-behaviour change needing its own
                # before/after, not a rider on an arithmetic fix.
                if hole_req > clearance + 1e-9:
                    required.append([key[0], key[1], round(hole_req, 4),
                                     'NPTH hole'])

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
            # A part whose own pads sit in a milled relief may span it -- e.g. a
            # connector with pins around its shield slot, whose pad EXTENT
            # swallows the ring. Ownership from the FILE's own poses, which is
            # what this function grades (#628).
            own = gate.rings_enclosing(
                [(p.global_x, p.global_y) for p in pads_by_ref.get(ref, ())])
            amt = gate.rect_outside_amount(ext, skip_rings=own)
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
            #
            # HOW FAR APART THEY ACTUALLY LAND, measured over #703's 120
            # placements (#788, docs/placement-predictors.md): graded at each
            # board's own netclass floor the two agree in sign on 119 of 119
            # and exactly on 118; graded at the 0.25 fallback, 112 of 120 and
            # 104. Every one of the 8 disagreements is THIS count reporting a
            # breach the per-pad measure does not, and seven of them are watchy
            # ROWS -- which are one placement, not seven: the human-authored
            # board plus six perturbations that did not move it, where SW1..SW4
            # sit 0.1696mm inside the inflated outline and no pad crosses the
            # real one. The number is not wrong; it is a different question,
            # and it is the one loop_driver's L2 gate refuses on.
            'oob_pad_basis': ('part pad AABB vs outline inflated by the '
                              'grading clearance (NOT the per-pad outline '
                              'measure; see render_placement '
                              'checklist.a_off_outline.pad_copper)'),
            # worst_n <= 0 = list ALL (run-4 F5: the repair rung's census
            # was silently capped at 10 movers on a 20-pair board).
            'worst': worst[:worst_n] if worst_n and worst_n > 0 else worst,
            # #697: pairs whose REQUIREMENT exceeds the board-wide `clearance`,
            # with what raised it -- check_drc's `required_mm` disclosure, so a
            # conflict the flat scalar could not explain does not read as an
            # unexplained number. Empty on a board that declares no netclass,
            # no .kicad_dru rule and no pad override.
            'required': sorted(required, key=lambda r: (-r[2], r[0], r[1])),
            # #697: anything that went WRONG resolving the requirement (an
            # unreadable .kicad_pro / .kicad_dru, a dru note). Silence there
            # drops the census back to the flat scalar and reports 0 conflicts
            # -- the exact silence this issue was filed for.
            'clearance_notes': clearance_notes,
            'exact': check_exact is not None}


def _pad_with_copper(pads, copper_index: int, clearance: float):
    """The Nth COPPER pad of a footprint's pad list (grade_pad_legality's
    PartPads indices count copper pads only, in construction order).

    The skip conditions here MIRROR `PartPads.__init__`'s, and that mirroring is
    what makes the index valid. Since #697 the same index also addresses
    `PartPads.pad_floors`, so a change to either skip rule must change both.
    """
    n = -1
    for p in pads:
        if not _pad_carries_copper(p):
            continue
        n += 1
        if n == copper_index:
            return p
    return None

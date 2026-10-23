"""QuenchState: a board's parts, their poses, and the legality around them.

The placement state every placement tool builds (`pose_score.make_state`):
each part's footprint geometry at every quarter turn, its pads and nets, its
lock, the board outline, the intent's keep-outs and zones, and the airwires
and their crossings. The simple path (`place_simple.py`) prices its own
objective (`simple.py`) on this state's live poses and asks it for legality.

Legality (hard constraints, and the whole of what candidate_valid decides) lives
in placement/legality.py, shared with the fanout-clearance repair pass: parts
only collide with parts that share a board SIDE (a back-side decap under a
front-side BGA is not an overlap; a through-hole part's lead field does reach the
far side), and board containment measures against the real Edge.Cuts outline, not
its bounding box.
"""
from __future__ import annotations

import os
import math
from typing import Dict, List, NamedTuple, Sequence, Tuple, Set, Optional

import numpy as np

from kicad_parser import IU_PER_MM, PCBData, mm_to_iu
from paste_apertures import pad_has_copper as _pad_has_copper
from connectivity import compute_mst_edges
import sys as _sys
_sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..',
                                                 'rust_router')))
import grid_router  # noqa: E402  the placement kernels: crossing_pairs, mst_prim
from placement.parser import (courtyard_for_side, extract_courtyard_sides,
                              extract_locked_refs, warn_missing_courtyards)
from placement.utility import compute_footprint_bbox_local
from placement import legality
from placement.legality import (CONTAINER_RATIO, CONTAINMENT_FRAC,
                                BoardOutlineGate, containment_frac,
                                footprint_has_through_pads,
                                footprint_side, pair_min_gap, rect_gap,
                                rect_overlap_area,
                                rotate_local_bounds, sides_occupied)

ROTATIONS = [0.0, 90.0, 180.0, 270.0]

#: The body-containment conjunct of `candidate_valid`, on. A module constant
#: so a test can switch it off and show that this conjunct is the one that
#: refuses a pose.
_CONTAINMENT_GATE = True
EPS_IMPROVE = 1e-6


_SEEDER = None


def _seeder():
    """`placement.seeder`, imported on first use (it imports this module).
    `candidate_valid` reads the #1257 pad conjunct through the module
    object, so a test that patches `seeder._pad_copper_overhang` switches
    the seat gate's and the state's copies off together."""
    global _SEEDER
    if _SEEDER is None:
        from . import seeder
        _SEEDER = seeder
    return _SEEDER


#: mm2 of .Fab body overlap `_body_overlap_at` counts: bodies that abut
#: (a shared edge, area 0) are not overlapping.
_BODY_OVERLAP_EPS = 1e-6


class _IntentTerm(NamedTuple):
    """One declared claim binding one ref, frozen at state construction.

    ONE TERM PER (ref, ENTRY) -- never per (ref, rule). Aggregating a rule's
    entries to a single scalar per ref is the subtle way to break the gate: two
    keep-outs would report `1.0 -> 1.0` for circles (or `5mm2 -> 5mm2` for
    rects) as a part moved from one into the other, and a monotone rule reads
    that as "no worse" and ADMITS it. The part hops between two regions it is
    graded on. Same for a ref that resolves into two zones.

    `threshold` is in this term's OWN currency: mm of zone escape, mm2 of
    intrusion, or `keepout_hit`'s fabricated circle marker. They are compared
    termwise and NEVER summed -- a sum lets a part buy 1mm of zone escape with
    1mm2 of intrusion.
    """
    rule: str                 # one of INTENT_ENFORCED_RULES
    name: str                 # block or keep-out name -- the NAMED verdict
    rect: Optional[Tuple[float, float, float, float]]
    threshold: float
    anchor: bool              # zone_containment: grade the courtyard CENTRE
    entry: Optional[Dict]     # keepout: the raw entry `keepout_hit` reads
    # zone_containment (#1174): sides the zone does not bound for this ref --
    # its declared edge, where the zone ends at the outline. The grade's own
    # `zone_open_sides`, carried in the zone row, so the gate opens what the
    # grade opens and nothing else.
    open_sides: Tuple[str, ...] = ()


class _TetherTerm(NamedTuple):
    """One declared part-vs-part claim, frozen at state construction (#1043).

    `refs` is every ref whose pose the measurement reads, so a move of ANY of
    them is checked against it: a cap's move and the move of any chip on its
    rail (the decap term re-elects among them per pose), both halves
    of a swap, a proximity subject and its partner. One term per claim the
    grade can report ONCE -- a (cap, IC) pair, an (IC, supply pin), a
    proximity subject pad (or the pair, when no pads are declared) -- so the
    monotone rule counts in `grade_delta`'s currency.
    """
    rule: str                 # one of floorplan.TETHER_RULES
    name: str
    refs: Tuple[str, ...]
    threshold: float
    kind: str                 # 'decap' | 'pin' | 'prox_pad' | 'prox_body'
    data: Dict


# --------------------------------------------------------------------------
# The declared-claim measurement, as three free functions (#698): separable
# from the state that enforces it, so there is exactly ONE of it.
# --------------------------------------------------------------------------

def build_zone_spec(zones, parts, refs=None
                    ) -> Dict[str, Tuple[_IntentTerm, ...]]:
    """The pose-INVARIANT zone terms binding each ref: `{ref: (term, ...)}`.

    Everything that does not depend on a pose is settled here so the per-pose
    cost is the geometry and nothing else: block membership, the
    exclusive-zone side filter, the tolerance, and the `zone_fits_courtyard`
    anchor decision (which reads only w/h and tests both orders, so no rotation
    in this engine's lattice can flip it).

    `refs` limits the walk to a subset -- the re-seat measures its scope, not
    the board. `None` means every part, which is what a `QuenchState` wants.

    The KEEP-OUT half is deliberately NOT here: it is resolved live, per call,
    by `intent_spec` -- see that function for why freezing it breaks #701's
    census.
    """
    spec: Dict[str, Tuple[_IntentTerm, ...]] = {}
    if not zones:
        return spec
    from . import floorplan as _fp
    items = (parts.items() if refs is None
             else ((r, parts[r]) for r in refs if r in parts))
    for _ref, _p in items:
        _terms: List[_IntentTerm] = []
        for _z in zones:
            _tol = float(_z['tolerance_mm'])
            if _ref in _z['refs']:
                # At the ORIGIN: `zone_fits_courtyard` reads only w/h, so
                # position is irrelevant and passing 0,0 says so. Both
                # rotations, so a turn cannot move a part between branches.
                _anchor = not any(
                    _fp.zone_fits_courtyard(
                        _z['rect'], _p.grade_rect(0.0, 0.0, _r), _tol)
                    for _r in (_p.rot % 360, (_p.rot + 90) % 360))
                _terms.append(_IntentTerm(
                    'zone_containment', _z['name'], tuple(_z['rect']),
                    _tol, _anchor, None,
                    tuple((_z.get('open_sides') or {}).get(_ref, ()))))
            elif _z['exclusive'] and (not _z['side']
                                      or _p.side == _z['side']):
                # `elif`, not `if`: `rule_zone_exclusive` skips members of the
                # block that owns the zone. Membership and the side filter are
                # both pose-invariant, so the set of rects a stranger must
                # avoid is resolved here.
                _terms.append(_IntentTerm(
                    'zone_exclusive', _z['name'], tuple(_z['rect']),
                    legality.EPS, False, None))
        if _terms:
            spec[_ref] = tuple(_terms)
    return spec


def exclusive_spec(zones, parts, refs=None
                   ) -> Dict[str, Tuple[_IntentTerm, ...]]:
    """The `zone_exclusive` slice of `build_zone_spec`, and NOTHING else (#797).

    A SLICE rather than a second walk, so membership, the `z.side` filter and
    the load-bearing `elif` that exempts a block's own members are resolved in
    exactly ONE place, and the seat gate and `candidate_valid`'s #702
    channel read the same construction rather than two that happen to agree.

    ONE DELIBERATE DIVERGENCE: the unresolved-zone filter below applies to the
    SEAT gate and not to the #702 channel. It is a policy about which claims
    may STRAND a part. `floorplan.rule_zone_exclusive` carries the SAME
    filter, so the seat gate and the GRADE -- the pair whose disagreement is an
    exit 4 on a correct board -- agree exactly.

    THE `zone_containment` TERMS ARE DROPPED HERE. Containment is gated only
    through `intent_zones`, by `candidate_valid`'s MONOTONE `intent_ok` (a pose
    inside the zone, or no further outside it than the part's current pose),
    never absolutely. `zone_exclusive` is the opposite shape -- a
    must-be-OUTSIDE claim, whose target is clean by definition -- which is
    why it alone is gated absolutely here.
    """
    # A zone whose members did not RESOLVE binds nobody here. "Stranger" means
    # "not a member", so with an empty member set EVERY part on the board is
    # one, and enforcing such a zone evicts the very parts it was drawn
    # around. Measured on `fanout_output1.kicad_pcb`, whose block is
    # `group:`-shaped: resolved at bare `()` sources the seat gate refused all
    # 5 of its OWN members at their own poses, and a repair then walked 4 of
    # them 4.00mm out of the region their block reserved.
    #
    # Nothing is silently admitted by this: `resolve_blocks` already reports an
    # unresolved block as `block_unresolved`, an ERROR, so such a board fails
    # on the real finding rather than on a rule that cannot tell a member from
    # a stranger.
    #
    # Applied HERE rather than in `build_zone_spec`, so the #702 channel is
    # untouched -- this is a SEAT-gate policy.
    # Membership is tested against the PARTS THIS WALK CAN SEE, not against
    # the mere presence of a ref string. A first version asked only
    # `z.get('refs')`, and a blind review found the hole: `QuenchState.parts`
    # drops a footprint with no pads and no courtyard, so a block whose only
    # member is one of those RESOLVES non-empty, keeps its zone, and then
    # finds no member here -- the same inversion one step further along, and
    # with no `block_unresolved` finding to point at it.
    zones = tuple(z for z in (zones or ())
                  if not z.get('exclusive')
                  or any(r in parts for r in (z.get('refs') or ())))
    full = build_zone_spec(zones, parts, refs)
    out: Dict[str, Tuple[_IntentTerm, ...]] = {}
    for _ref, _terms in full.items():
        _keep = tuple(t for t in _terms if t.rule == 'zone_exclusive')
        if _keep:
            out[_ref] = _keep
    return out


def intent_spec(zone_spec, keepouts_for, ref) -> Tuple[_IntentTerm, ...]:
    """The claims binding `ref` right now: frozen zone terms, plus keep-out
    terms derived LIVE from `keepouts_for`.

    The keep-out slice is deliberately not frozen: a caller that lifts
    keep-out X by removing it from `state.keepouts_for[ref]` must see the
    lift, and a frozen copy would defeat it silently, because `pose_ok`
    reaches this gate through `candidate_valid`.

    Still pose-INVARIANT and still resolved once: `keepouts_for` is the cached
    resolution, and this only reads it.
    """
    zones = zone_spec.get(ref, ())
    kos = keepouts_for.get(ref, ())
    if not kos:
        return zones
    return zones + tuple(
        _IntentTerm('keepout', str(k.get('name') or '<unnamed>'),
                    None, 0.0, False, k)
        for k in kos)


def intent_term_values(spec: Tuple[_IntentTerm, ...], rects
                       ) -> Tuple[float, ...]:
    """This pose measured against every term in `spec`, in the spec's order.

    A VECTOR, never a scalar -- see `_IntentTerm`.
    """
    from . import floorplan as _fp   # lazy: see seeder.pose_ok's reason
    out = []
    for t in spec:
        if t.rule == 'keepout':
            # BOTH rects, because `rule_keepout` grades both: a THT part's
            # leads pierce a keep-out from the far side.
            out.append(_fp.keepout_hit(t.entry, rects))
        elif t.rule == 'zone_exclusive':
            # COURTYARD ONLY -- `rule_zone_exclusive` reads `part.rect` and
            # never `tht_rect`. Matching the grade includes matching what it
            # declines to measure.
            out.append(rect_overlap_area(rects[0], t.rect))
        else:
            out.append(_fp.zone_escape(t.rect, rects[0], t.anchor,
                                       t.open_sides)[0])
    return tuple(out)


# Lives in placement/legality.py, the single home shared with fanout_clearance
# (which carried a byte-identical copy). Kept as a module-level alias: it is
# part of this module's de-facto surface (tests import it).
_rotate_local_bounds = rotate_local_bounds


def _airwires_for_points(points: List[Tuple[float, float]], net_id: int):
    """MST airwires for one net: list of (x1, y1, x2, y2, net_id)."""
    if len(points) < 2:
        return []
    if len(points) == 2:
        (x1, y1), (x2, y2) = points
        return [(x1, y1, x2, y2, net_id)]
    edges = _mst_edges(points)
    return [(points[i][0], points[i][1], points[j][0], points[j][1], net_id)
            for i, j, _ in edges]


#: A net with at least this many pads takes its tree from `_mst_edges_sparse`;
#: a smaller one from the Rust Prim, which is faster there (Rust Prim against
#: sparse: 5 ms against 7 ms at 1,200 pads, 48 ms against 26 ms at 4,000).
SPARSE_MST_MIN_POINTS = 2500


def _mst_edges(points):
    """`compute_mst_edges(points)` -- the same edges, in the same order, by
    the same floats: the Rust kernel `grid_router.mst_prim` (the same Prim,
    in O(n) memory), or over the Delaunay edges for a net of
    `SPARSE_MST_MIN_POINTS` pads or more, where O(n log n) wins."""
    if len(points) >= SPARSE_MST_MIN_POINTS:
        got = _mst_edges_sparse(points)
        if got is not None:
            return got
    if len(points) < 3:
        return compute_mst_edges(points, use_manhattan=False)
    pts = np.asarray(points, dtype=np.float64)
    return grid_router.mst_prim(np.ascontiguousarray(pts[:, 0]),
                                np.ascontiguousarray(pts[:, 1]))


def _mst_edges_sparse(points):
    """Prim's tree of `points` exactly as `compute_mst_edges` builds it, run
    over the Delaunay triangulation's edges instead of every pair; None when
    the triangulation cannot be had (fewer than three distinct points, all of
    them on a line, a point qhull set aside), and the caller falls back.

    Why the sparse graph gives the SAME tree, ties included. Dense Prim adds,
    at every step, a lightest edge leaving the tree (the lowest-numbered
    vertex among equal keys, joined to the earliest tree vertex at that
    distance). Every lightest edge leaving a tree is in some minimum spanning
    tree; every edge of a minimum spanning tree is a relative-neighbourhood
    edge (no third point is strictly nearer both ends, or the tree would trade
    it for a shorter one); and every such edge has an empty CLOSED diametral
    circle (a point on or in it sees the edge at 90 degrees or more, so it is
    strictly nearer both ends), which puts it in every Delaunay triangulation.
    So every edge dense Prim could choose, and every tie it breaks, is among
    the Delaunay edges, and a heap keyed (distance, vertex) with the same
    strict-less key update makes the same choice at every step. Distances are
    `sqrt(dx*dx + dy*dy)`, the dense matrix's own expression, so they are the
    same floats. Coincident pads share one triangulation vertex, and every
    copy takes that vertex's neighbours and each other."""
    import heapq
    n = len(points)
    pts = np.asarray(points, dtype=np.float64)
    if not np.isfinite(pts).all():
        return None
    uniq, first, rep = np.unique(pts, axis=0, return_index=True,
                                 return_inverse=True)
    rep = rep.reshape(-1)
    if len(uniq) < 3:
        return None
    try:
        from scipy.spatial import Delaunay
        tri = Delaunay(uniq)
    except Exception:                                         # noqa: BLE001
        return None
    if len(getattr(tri, 'coplanar', ())):
        return None
    sx = tri.simplices
    ue = np.concatenate([sx[:, [0, 1]], sx[:, [1, 2]], sx[:, [0, 2]]])
    ue = np.unique(np.sort(ue, axis=1), axis=0)
    # Unique-vertex edges back onto the original points, plus each set of
    # coincident copies with one another.
    members: Dict[int, List[int]] = {}
    for i, r in enumerate(rep.tolist()):
        members.setdefault(r, []).append(i)
    ea, eb = [], []
    for u, v in ue.tolist():
        for i in members[u]:
            for j in members[v]:
                ea.append(i)
                eb.append(j)
    for grp in members.values():
        for x in range(len(grp)):
            for y in range(x + 1, len(grp)):
                ea.append(grp[x])
                eb.append(grp[y])
    ea = np.array(ea, dtype=np.intp)
    eb = np.array(eb, dtype=np.intp)
    xs, ys = pts[:, 0], pts[:, 1]
    dx = xs[eb] - xs[ea]
    dy = ys[eb] - ys[ea]
    w = np.sqrt(dx * dx + dy * dy).tolist()
    adj: List[List[Tuple[float, int]]] = [[] for _ in range(n)]
    for a_, b_, d in zip(ea.tolist(), eb.tolist(), w):
        adj[a_].append((d, b_))
        adj[b_].append((d, a_))
    inf = float('inf')
    key = [inf] * n
    near = [0] * n
    done = [False] * n
    done[0] = True
    heap = []
    for d, v in adj[0]:
        if d < key[v]:
            key[v] = d
            heapq.heappush(heap, (d, v))
    edges = []
    while heap:
        d, v = heapq.heappop(heap)
        if done[v] or d != key[v]:
            continue
        done[v] = True
        edges.append((near[v], v, d))
        for dv, u in adj[v]:
            if not done[u] and dv < key[u]:
                key[u] = dv
                near[u] = v
                heapq.heappush(heap, (dv, u))
    return edges if len(edges) == n - 1 else None


def _aw_array(airwires) -> np.ndarray:
    if not airwires:
        return np.zeros((0, 5))
    return np.asarray(airwires, dtype=float)


def _count_crossings_np(a: np.ndarray, b: np.ndarray,
                        net_w: Optional[np.ndarray] = None):
    """Count crossings between airwire sets a (n,5) and b (m,5), skipping
    same-net pairs and pairs sharing an endpoint (within 1um).

    Returns (count, weighted). `count` is the raw unweighted pair count, kept
    as an int for reporting. `weighted` prices each crossing at
    max(net_w[net_a], net_w[net_b]), the per-net weighting from #458, where
    net_w is a net-id-indexed weight lookup. With net_w
    None the two are equal and `weighted` is exactly float(count), so
    unweighted callers get bit-identical costs.

    Only a pair whose bounding boxes meet can cross: a crossing, or a
    touching endpoint, puts a point of both segments in both boxes. So the
    orientation tests run on those pairs alone, taken in the row-major order
    the full matrix would list them in, so the weighted sum adds the same
    values in the same order. On a big board most pairs are far apart
    (glasgow_revC: 97 of a seed's 243 s were this function over the full
    matrix). The box margin is far above float error. The one pair this
    counts differently is two COLLINEAR segments that do not touch: their
    orientation products are rounding noise, which the full matrix could
    read as a crossing, and their boxes do not meet.
    """
    if len(a) == 0 or len(b) == 0:
        return 0, 0.0
    _ii, _jj, w = _cross_pairs(a, b, net_w)
    count = len(w)
    if net_w is None:
        return count, float(count)
    # The weights in row-major pair order, summed by numpy as they always
    # were, so the total is the same float.
    return count, float(np.sum(w))


def _cross_pairs(a: np.ndarray, b: np.ndarray, net_w: Optional[np.ndarray] = None):
    """(ii, jj, w): every (a row, b row) pair of airwires that cross, in
    row-major order, with each pair's weight max(net_w[net_a], net_w[net_b])
    (1.0 without `net_w`; column 4 carries the net id, and every id that can
    appear there is a key of QuenchState.net_airwires, which net_w covers).
    The Rust kernel `grid_router.crossing_pairs`: the box test, the
    orientation test and the exclusions above, operation for operation."""
    return grid_router.crossing_pairs(
        np.ascontiguousarray(a, dtype=np.float64),
        np.ascontiguousarray(b, dtype=np.float64),
        None if net_w is None else np.ascontiguousarray(net_w, dtype=np.float64))


class _CorridorBox(NamedTuple):
    """A corridor frozen into the form the chord kernel wants.

    `skip` is a dense net-id-indexed bool: the corridor's OWN nets (a bus inside
    its own lane is the point, not a cost), plus the ignored and high-fanout nets
    `foreign_crossings` drops for the same reason it drops them there.
    """
    ax: float
    ay: float
    ux: float
    uy: float
    length: float
    half_w: float
    skip: np.ndarray


def _corridor_cut_np(a: np.ndarray, boxes) -> float:
    """Total length of `a`'s airwires lying INSIDE the corridor rectangles.

    Why a chord and not the crossing count `foreign_crossings` uses: a count is
    piecewise constant in pose, so its gradient is zero almost everywhere and a
    greedy descent gets no direction from it -- a part can slide halfway out of a
    corridor and the count never moves until it pops out entirely. The chord is
    piecewise linear in pose, and it prices obliqueness for free (a wire crossing
    a width-w lane at angle theta cuts w/sin(theta), so cutting a lane
    diagonally costs more than crossing it square, which is exactly the physical
    truth on a plane).

    What it is a bound on: on a 2-layer board the bus owns the top layer inside
    its own lane, so a foreign net crossing the lane runs underneath for the
    shared span, and the chord is the geometric lower bound on reference-plane
    copper that span removes.

    Exact, not sampled: each segment is rotated into the corridor's own frame
    (an isometry, so lengths carry over unchanged) where the rectangle is
    axis-aligned, and clipped with Liang-Barsky. Vectorised over airwires, one
    pass per corridor -- boards declare a handful of corridors, not thousands.
    """
    if len(a) == 0 or not boxes:
        return 0.0
    total = 0.0
    for live, chord in _corridor_chords(a, boxes):
        total += float(np.sum(chord))
    return total


def _corridor_chords(a: np.ndarray, boxes):
    """Per corridor that `a` reaches: (the rows' mask, each row's chord
    inside it), as `_corridor_cut_np` sums them."""
    x1, y1, x2, y2 = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    nets = a[:, 4].astype(np.intp)
    seg_len = np.hypot(x2 - x1, y2 - y1)
    for box in boxes:
        live = ~box.skip[np.clip(nets, 0, len(box.skip) - 1)]
        # A net id beyond the lookup was never in net_airwires, so it cannot be
        # one of this corridor's own nets: clip-and-test would read the wrong
        # slot, so drop those explicitly rather than trusting the clamp.
        live &= (nets >= 0) & (nets < len(box.skip))
        if not live.any():
            continue
        dx1, dy1 = x1[live] - box.ax, y1[live] - box.ay
        dx2, dy2 = x2[live] - box.ax, y2[live] - box.ay
        # Corridor frame: u along the axis, v along the normal (-uy, ux).
        u1 = dx1 * box.ux + dy1 * box.uy
        v1 = -dx1 * box.uy + dy1 * box.ux
        u2 = dx2 * box.ux + dy2 * box.uy
        v2 = -dx2 * box.uy + dy2 * box.ux
        du, dv = u2 - u1, v2 - v1
        t0 = np.zeros(len(u1))
        t1 = np.ones(len(u1))
        alive = np.ones(len(u1), dtype=bool)
        h = box.half_w
        for p, q in ((-du, u1), (du, box.length - u1),
                     (-dv, v1 + h), (dv, h - v1)):
            par = np.abs(p) < 1e-12          # parallel to this edge
            alive &= ~(par & (q < 0.0))      # ...and outside it: no chord
            with np.errstate(divide='ignore', invalid='ignore'):
                r = np.where(par, 0.0, q / np.where(par, 1.0, p))
            enter = (~par) & (p < 0.0)
            t0 = np.where(enter, np.maximum(t0, r), t0)
            t1 = np.where((~par) & (p > 0.0), np.minimum(t1, r), t1)
        frac = np.where(alive, np.maximum(t1 - t0, 0.0), 0.0)
        yield live, frac * seg_len[live]


def _count_crossings_within(a: np.ndarray,
                            net_w: Optional[np.ndarray] = None):
    """Crossings among one airwire set (each unordered pair counted once).
    Returns (count, weighted), as _count_crossings_np."""
    if len(a) < 2:
        return 0, 0.0
    total, weighted = _count_crossings_np(a, a, net_w)
    half = total // 2
    if net_w is None:
        # Keep the historical integer halving bit-exact. The ccw predicate for
        # (i,j) and (j,i) are different floating point expressions of the same
        # orientation, so `total` is not PROVABLY even and float(total)/2 is
        # not always float(total // 2). Do not "simplify" this branch away.
        return half, float(half)
    # max(w_i, w_j) is symmetric and so is the crossing relation, so every
    # unordered pair contributes twice with the same weight; dividing by 2.0
    # is exact in binary floating point.
    return half, weighted / 2.0


#: #1152: WHY a part entered a `QuenchState` locked, as the phrase a refusal
#: names (`QuenchState.lock_reasons`). `_Part.locked` is one boolean with
#: several causes, and naming every one of them "locked in the board file"
#: sends the author of a logo's fixed pose to unlock a board file that locks
#: nothing. Recorded where each lock is applied, the first cause winning; a
#: part absent from the map entered free.
LOCK_FILE = 'locked in the board file'
LOCK_CALLER = "locked by the caller (the intent's fixed_poses[])"
LOCK_OUTLINE = 'draws the board outline (#829)'
LOCK_PADLESS = 'has no pad: a static obstacle by its courtyard'
LOCK_SEATED_PLUG = 'a PCB-edge plug seated at its edge (#1098)'


#: `_NearIndex` widens every query by this much past the clearance, so the
#: prune stays a prune at the last ulp: `rect_gap`'s diagonal branch is a
#: `math.hypot`, exact to an ulp rather than to the bit, and the grid's cell
#: arithmetic divides.
_NEAR_SLACK_MM = 1e-6

#: `_NearIndex`'s bucket edge. Parts are a few mm across, so a query box
#: usually touches a handful of cells.
_NEAR_CELL_MM = 4.0

#: A footprint spanning more cells than this (a wild pose, an infinite box)
#: is not bucketed: it is returned for every query instead.
_NEAR_MAX_CELLS = 4096

#: Below this many parts a full sweep costs no more than the index does,
#: and `_sweep_others` keeps it.
_NEAR_INDEX_MIN_PARTS = 32

#: A box reaching farther from the origin than this (mm) is not bucketed: a
#: footprint is returned for every query, a query sweeps everything.
#: `_NEAR_SLACK_MM` covers the rounding of the cell arithmetic only while an
#: ulp is far below it -- about 1e-10 mm here, where it passes 1e-6 near
#: 1e10 mm.
_NEAR_MAX_COORD_MM = 1e6


class _NearIndex:
    """Every part's NEIGHBOUR FOOTPRINT -- the box spanning `rect()` and
    `tht_rect()` at its live pose, the two rects the courtyard pair tests
    read -- bucketed on a uniform grid, so the full-board sweeps of
    `violation_parts` and `candidate_valid` visit only the parts that can
    come within the clearance. The placer asks both at every candidate pose
    against every part on the board.

    A PRUNE, never a verdict. `near` returns a part unless its footprint
    lies at least `reach` from the query box along one axis. Every gap the
    sweeps compute (`rect_gap`, `far_gap`, `pair_min_gap`) is bounded below
    by the axis separation of the boxes it compares, both of which sit
    inside the two footprints, and IEEE subtraction rounds monotonically, so
    a dropped part could neither fail the clearance nor add a shortfall. The
    grid only narrows which footprints that exact test is asked about: a
    footprint in no cell the query reaches is farther than `reach` already
    (`_cells` widens the query by `_NEAR_SLACK_MM` against the rounding of
    its division). The survivors come back in `parts` order, so the loops
    see the sequence they always saw, less parts that would have contributed
    nothing -- the same first violator, the same float sum, the same early
    exit. A footprint with a NaN or infinite coordinate -- in any box it
    spans, a far side's clusters included -- is in every answer, unpruned;
    one past `_NEAR_MAX_COORD_MM` is in every answer and pruned by the exact
    test alone.

    A stale footprint would be a silent wrong answer, so staleness is not
    left to call sites: every write to a field the footprint reads
    (`_NearWatched._GEOM_FIELDS`, through its `__setattr__`) and every entry
    `ensure_rotation` adds marks the part here, and `near` re-buckets marked
    parts before it reads any."""

    __slots__ = ('src', 'items', 'pos', 'boxes', 'spans', 'cells', 'always',
                 'dirty')

    def __init__(self, parts):
        self.src = parts
        self.items = list(parts.items())
        self.pos = {ref: i for i, (ref, _p) in enumerate(self.items)}
        self.boxes = [None] * len(self.items)
        self.spans = [None] * len(self.items)
        self.cells: Dict[Tuple[int, int], Set[int]] = {}
        self.always: Set[int] = set()
        self.dirty = set(self.pos)
        for _ref, p in self.items:
            try:
                p._ix_watch.append(self.dirty)
            except AttributeError:
                p._ix_watch = [self.dirty]

    @classmethod
    def build(cls, parts):
        """An index over `parts`, or None when one is not a `_Part` (it
        could be moved without marking the index)."""
        if not all(type(p) is _Part for p in parts.values()):
            return None
        return cls(parts)

    def current(self, parts) -> bool:
        """Whether this index still describes `parts` (the same dict, the
        same members -- QuenchState never adds or removes one)."""
        return parts is self.src and len(parts) == len(self.items)

    def detach(self):
        """Stop watching: a superseded index's dirty set leaves every part's
        watch list, so later moves stop feeding a set nothing reads."""
        mine = self.dirty
        for _ref, p in self.items:
            watch = getattr(p, '_ix_watch', None)
            if watch:
                watch[:] = [d for d in watch if d is not mine]

    @staticmethod
    def _cells(x0, y0, x1, y1):
        """The cell range (cx0, cy0, cx1, cy1) a box touches, or None when it
        is not finite, reaches past `_NEAR_MAX_COORD_MM`, is inverted, or
        spans more than `_NEAR_MAX_CELLS` cells."""
        g, e, m = _NEAR_CELL_MM, _NEAR_SLACK_MM, _NEAR_MAX_COORD_MM
        if not (-m <= x0 <= m and -m <= y0 <= m and -m <= x1 <= m
                and -m <= y1 <= m):
            return None                                # far, or NaN
        try:
            r = (math.floor((x0 - e) / g), math.floor((y0 - e) / g),
                 math.floor((x1 + e) / g), math.floor((y1 + e) / g))
        except (OverflowError, ValueError):            # inf, NaN
            return None
        if (r[2] < r[0] or r[3] < r[1]
                or (r[2] - r[0] + 1) * (r[3] - r[1] + 1) > _NEAR_MAX_CELLS):
            return None
        return r

    def _refresh(self):
        cells, always = self.cells, self.always
        for ref in self.dirty:
            i = self.pos.get(ref)
            if i is None:
                continue
            old = self.spans[i]
            if old is None:
                always.discard(i)
            else:
                for cx in range(old[0], old[2] + 1):
                    for cy in range(old[1], old[3] + 1):
                        cells[(cx, cy)].discard(i)
            p = self.items[i][1]
            try:
                box = _footprint_box(p.rect(), p.tht_rect())
            except Exception:                              # noqa: BLE001
                # Unmeasurable now: the sweep measures it itself, so it
                # raises (or not) exactly where it always did.
                box = None
            span = None if box is None else self._cells(*box)
            self.boxes[i] = box
            self.spans[i] = span
            if span is None:
                always.add(i)
            else:
                for cx in range(span[0], span[2] + 1):
                    for cy in range(span[1], span[3] + 1):
                        c = cells.get((cx, cy))
                        if c is None:
                            c = cells[(cx, cy)] = set()
                        c.add(i)
        self.dirty.clear()

    def near(self, q, reach):
        """[(ref, part)] in `parts` order, less every part whose footprint
        is at least `reach` from box `q` along x or y; None when `q` cannot
        be bucketed (the caller sweeps everything)."""
        if self.dirty:
            self._refresh()
        span = self._cells(q[0] - reach, q[1] - reach, q[2] + reach,
                           q[3] + reach)
        if span is None:
            return None
        cells = self.cells
        got = set(self.always)
        for cx in range(span[0], span[2] + 1):
            for cy in range(span[1], span[3] + 1):
                c = cells.get((cx, cy))
                if c:
                    got |= c
        q0, q1, q2, q3 = q
        boxes, items = self.boxes, self.items
        out = []
        for i in sorted(got):
            b = boxes[i]
            if b is not None and (q0 - b[2] >= reach or b[0] - q2 >= reach
                                  or q1 - b[3] >= reach
                                  or b[1] - q3 >= reach):
                continue
            out.append(items[i])
        return out


def _footprint_box(rect, tht):
    """The box spanning a courtyard rect and an optional far-side rect (a
    `FarSide` IS its union box), or None unless every coordinate is finite.

    Every coordinate means every box the far side stands for
    (`legality.far_boxes`), not just its four union numbers: those are a
    min/max over its clusters, and `min`/`max` skip a NaN that is not
    first, while `rect_gap` returns `max(dx, NaN) == dx` -- a cluster NaN in
    y is compared in x alone, so the union would prove a separation the
    sweep does not see. An infinity is refused with it: `inf - inf` is the
    other way a gap goes NaN, and an inverted box can reach it. With every
    coordinate finite no gap the sweeps take is NaN, and each is at least
    every axis separation of the boxes it compares."""
    if tht is None:
        box = tuple(rect)
        s = box[0] + box[1] + box[2] + box[3]
    else:
        box = (min(rect[0], tht[0]), min(rect[1], tht[1]),
               max(rect[2], tht[2]), max(rect[3], tht[3]))
        s = (rect[0] + rect[1] + rect[2] + rect[3]
             + sum(map(sum, legality.far_boxes(tht))))
    # `s - s` is 0.0 exactly when `s` is finite, and `s` is finite only
    # when every term is: a NaN or an infinity propagates (an overflow is
    # refused too, which only costs the prune).
    if s - s != 0.0:
        return None
    return box


class _NearWatched:
    """The `_NearIndex` side of `_Part`: the indexes watching this part, and
    the write hook that marks it stale in them. A base class, so `_Part`'s
    own slots stay as they were."""

    __slots__ = ('_ix_watch',)

    #: What `rect()` / `tht_rect()` read, and so what a `_NearIndex`
    #: footprint is computed from: a write to any of them marks the part
    #: stale in every index watching it. Reads are unaffected -- only
    #: assignment goes through `__setattr__`, and the memo slots
    #: (`_pg_rot`, `_pg_pose`, `_live_rects`) are written with
    #: `object.__setattr__` to skip it (~5% of a pad_globals miss).
    #: tests/test_near_index_prune.py reads `rect()` / `tht_rect()`'s
    #: source and fails when they read a field missing here.
    _GEOM_FIELDS = frozenset(('x', 'y', 'rot', 'bounds_by_rot',
                              'tht_by_rot', 'side', 'sides', 'has_tht'))

    def __setattr__(self, name, value):
        object.__setattr__(self, name, value)
        if name in _NearWatched._GEOM_FIELDS:
            # getattr: construction, a copy or an unpickle may write a
            # geometry field before the watch list exists.
            for dirty in getattr(self, '_ix_watch', ()):
                dirty.add(self.ref)

    def _near_stale(self):
        """Mark this part stale in every index watching it."""
        for dirty in getattr(self, '_ix_watch', ()):
            dirty.add(self.ref)


def locked_body_locals(pcb_data, pcb_file
                       ) -> Dict[str, Tuple[float, float, float, float]]:
    """`{ref: occupancy_local}` for each part whose drawn body stands in for a
    missing courtyard: a FAB body (`placement.body`), united with the pads,
    on a part that is no container (`legality._container_kind`).

    `QuenchState` gives it to a LOCKED part without a courtyard. Such a part
    is otherwise spaced on its pad box, while check_assembly grades every
    moved neighbour against its body: on ulx3s the ESP32 module U9 (locked,
    no courtyard) has a fab body 4 mm past its pads at the antenna end, and
    six parts seated there made every grouped candidate NOT BUILDABLE. A
    MOVABLE part keeps its pad box -- the body model as a seat currency for
    every part (#1182) was A/B'd and rejected -- and a silk outline or a
    pin-1 marking is no body (#896, #1201), nor is a container's outline,
    which other parts sit inside by design."""
    from placement import body as _body
    out = {}
    for ref, g in _body.board_bodies(pcb_data, pcb_file).items():
        if (g.source == _body.SOURCE_FAB and g.occupancy_local is not None
                and not legality._container_kind(pcb_data, ref, g.occupancy_local,
                                                 g.source, False)):
            out[ref] = tuple(g.occupancy_local)
    return out


class _Part(_NearWatched):
    __slots__ = ('ref', 'pads_local', 'pin_count', 'bounds_by_rot',
                 'seed_x', 'seed_y', 'x', 'y', 'rot', 'locked',
                 'nets', 'footprint_name', 'orig_rot',
                 'side', 'has_tht', 'sides', 'tht_by_rot', 'padbox_local',
                 'padbox_by_rot', 'grade_by_rot', '_live_rects',
                 '_pg_rot', '_pg_pose', 'drawn_local')

    def __init__(self, ref, fp, courtyard_sides, locked, body_local=None,
                 drawn_sides=None):
        self._live_rects = None
        self.ref = ref
        self.footprint_name = fp.footprint_name
        self.pads_local = [(p.local_x, p.local_y, p.net_id)
                           for p in fp.pads if p.net_id > 0]
        self.pin_count = len(self.pads_local)
        # Board side, and the sides this part physically obstructs: its own
        # always, both when it has drilled pads (#456 item 1).
        self.side = footprint_side(fp)
        self.has_tht = footprint_has_through_pads(fp)
        self.sides = sides_occupied(self.side, self.has_tht)
        # `body_local`, None for nearly every part, is the occupancy of a
        # LOCKED part with no courtyard: the body check_assembly grades it on
        # (`locked_body_locals`), so a part seated on it is a blocking pair.
        # `pose_ok`/`candidate_valid` seat against `rect`, "what does this
        # part occupy".
        #
        # #1182: the GRADE ladder (courtyard, else pad bbox) is kept beside
        # it, because the floorplan grade -- zones, keep-outs, edge claims,
        # the board term -- asks `grade_rect`, so the search cannot refuse a
        # zone seat the grade accepts. Without a body the two are ONE dict.
        grade_lb = courtyard_for_side(courtyard_sides.get(ref), self.side)
        if grade_lb is None:
            grade_lb = compute_footprint_bbox_local(fp)
        lb = body_local
        if lb is None:
            lb = grade_lb
        self.bounds_by_rot = {r: _rotate_local_bounds(*lb, r) for r in ROTATIONS}
        self.grade_by_rot = (self.bounds_by_rot if lb is grade_lb else
                             {r: _rotate_local_bounds(*grade_lb, r)
                              for r in ROTATIONS})
        # #1300: when `courtyard_sides` carries an ESTIMATED courtyard for a
        # part that draws none, `drawn_sides` is the board's own map, and
        # `drawn_local` the box KiCad would judge the part on (its pad box):
        # `drawn_rect` hands it to the one test that judges two declared
        # poses as KiCad does. None when the part's courtyard is drawn.
        self.drawn_local = None
        if drawn_sides is not None and ref in courtyard_sides \
                and ref not in drawn_sides:
            self.drawn_local = tuple(compute_footprint_bbox_local(fp))
        # #1101: the PAD copper box, for a board whose project waives the
        # courtyard rule -- the seat then spaces pads, not courtyards. None
        # for a pad-less footprint (a logo occupies no copper).
        # `fp.pads`, not `non_aperture_pads` (#1143, deliberately): an
        # aperture-only part stays a MOVABLE quench part (see the zero-pad
        # branch in QuenchState), so it keeps a pad box -- the bbox
        # fallback, since its apertures are not extent.
        self.padbox_local = (compute_footprint_bbox_local(fp)
                             if fp.pads else None)
        self.padbox_by_rot: Dict[float, Tuple[float, float, float, float]] = {}
        # #1206: one box per cluster of drilled pads (a FarSide when there
        # are several) -- the grader's far side, so the seat and the grade
        # agree on what a part presents through the board.
        # `courtyard_sides` is the BOARD's map: this part's entry is what
        # names its far courtyard (the second phase-2 verifier: handed the
        # whole map, `far_courtyard_of` answered None for every part, and the
        # search admitted a 0603 under a drawn B.CrtYd the grader flags).
        tlb = (legality.far_side_local(fp, legality.far_courtyard_of(
            courtyard_sides.get(ref), self.side)) if self.has_tht else None)
        self.tht_by_rot = ({r: legality.rotate_far(tlb, r) for r in ROTATIONS}
                           if tlb is not None else None)
        # A non-90-degree seed rotation brings its WHOLE 90-degree lattice:
        # those are the turns a seat can try for such a part.
        base = fp.rotation % 90
        if base:
            for r in ROTATIONS:
                rot = (base + r) % 360
                self.bounds_by_rot[rot] = _rotate_local_bounds(*lb, rot)
                if self.grade_by_rot is not self.bounds_by_rot:
                    self.grade_by_rot[rot] = _rotate_local_bounds(*grade_lb,
                                                                  rot)
                if self.tht_by_rot is not None:
                    self.tht_by_rot[rot] = legality.rotate_far(tlb, rot)
        self.seed_x, self.seed_y = fp.x, fp.y
        self.x, self.y, self.rot = fp.x, fp.y, fp.rotation % 360
        self.orig_rot = fp.rotation % 360
        self.locked = locked
        self.nets = sorted({n for _, _, n in self.pads_local})
        # `pad_globals` memos -- see there. Both carry the `pads_local` list
        # they were built from, so a reassigned list can never be served a
        # stale answer.
        self._pg_rot = None
        self._pg_pose = None

    def ensure_rotation(self, rot):
        """Fill both rotation caches for `rot`, as given (a caller that
        wants the normalised key normalises first). THE one place an entry is
        made after construction (#1206): the far side is turned cluster by
        cluster (`legality.rotate_far`), so no cache can flatten a FarSide
        to its union box, which an inline copy of this fill could.

        An entry it adds can change what `rect()` answers for a part already
        AT that angle (a missing key falls back to the 0-degree bounds), so
        it marks the part stale in every `_NearIndex` watching it."""
        added = False
        if rot not in self.bounds_by_rot:
            self.bounds_by_rot[rot] = _rotate_local_bounds(
                *self.bounds_by_rot[0.0], rot)
            added = True
        if rot not in self.grade_by_rot:
            self.grade_by_rot[rot] = _rotate_local_bounds(
                *self.grade_by_rot[0.0], rot)
        if self.tht_by_rot is not None and rot not in self.tht_by_rot:
            self.tht_by_rot[rot] = legality.rotate_far(self.tht_by_rot[0.0],
                                                       rot)
            added = True
        if added:
            self._near_stale()

    def rect(self, x=None, y=None, rot=None):
        x = self.x if x is None else x
        y = self.y if y is None else y
        rot = self.rot if rot is None else rot
        b = self.bounds_by_rot.get(rot % 360)
        if b is None:
            b = self.bounds_by_rot[0.0]
        return (x + b[0], y + b[1], x + b[2], y + b[3])

    def grade_rect(self, x=None, y=None, rot=None):
        """The rect the floorplan GRADE reads for this part at a pose:
        courtyard, else pad bbox (#1182). `rect()` itself unless the part
        carries a body (`body_local`), which `rect` reads and this does
        not."""
        x = self.x if x is None else x
        y = self.y if y is None else y
        rot = self.rot if rot is None else rot
        b = self.grade_by_rot.get(rot % 360)
        if b is None:
            b = self.grade_by_rot[0.0]
        return (x + b[0], y + b[1], x + b[2], y + b[3])

    def grade_rects(self, x=None, y=None, rot=None):
        """`rects()` on the grade ladder: (grade rect, far-side rect)."""
        if self.tht_by_rot is None:
            return self.grade_rect(x, y, rot), None
        return self.grade_rect(x, y, rot), self.tht_rect(x, y, rot)

    def drawn_rect(self, x=None, y=None, rot=None):
        """The rect KiCad judges this part on at a pose: `grade_rect`, except
        that a part seated on an ESTIMATED courtyard (#1300) answers its pad
        box, as it did before the estimate -- for `seeder._courtyard_overlap`
        between two parts that both stand where they were declared or
        locked, so a declared arrangement is never refused on an estimate."""
        if self.drawn_local is None:
            return self.grade_rect(x, y, rot)
        x = self.x if x is None else x
        y = self.y if y is None else y
        rot = self.rot if rot is None else rot
        b = _rotate_local_bounds(*self.drawn_local, rot % 360)
        return (x + b[0], y + b[1], x + b[2], y + b[3])

    def padbox(self, x=None, y=None, rot=None):
        """The part's pad-copper box at a pose (#1101), or None (no pads)."""
        if self.padbox_local is None:
            return None
        x = self.x if x is None else x
        y = self.y if y is None else y
        rot = (self.rot if rot is None else rot) % 360
        b = self.padbox_by_rot.get(rot)
        if b is None:
            b = self.padbox_by_rot[rot] = _rotate_local_bounds(
                *self.padbox_local, rot)
        return (x + b[0], y + b[1], x + b[2], y + b[3])

    def tht_rect(self, x=None, y=None, rot=None):
        """The part's obstruction on the OPPOSITE side; None when it has no
        drilled pads and therefore does not reach the far side at all."""
        if self.tht_by_rot is None:
            return None
        x = self.x if x is None else x
        y = self.y if y is None else y
        rot = self.rot if rot is None else rot
        b = self.tht_by_rot.get(rot % 360)
        if b is None:
            b = self.tht_by_rot[0.0]
        return legality.offset_far(b, x, y)

    def rects(self, x=None, y=None, rot=None):
        """(courtyard rect, far-side rect) at a pose -- what a pair test needs.

        The far-side rect is None for the overwhelming majority of parts (no
        drilled pads), and the pair tests fast-path on exactly that, so this
        stays a single rect() call for them.
        """
        if self.tht_by_rot is None:
            return self.rect(x, y, rot), None
        if x is None and y is None and rot is None:
            # A drilled part's rects at its LIVE pose, kept until it moves (or
            # a rotation table grows): every neighbour test against it asks
            # them, and its far side is a fresh `FarSide` per call -- on
            # ulx3s's pile the decap rung built 7.8M of them, a fifth of its
            # time. The key is everything the two reads depend on.
            key = (self.x, self.y, self.rot, len(self.bounds_by_rot),
                   len(self.tht_by_rot))
            hit = self._live_rects
            if hit is not None and hit[0] == key:
                return hit[1]
            got = (self.rect(), self.tht_rect())
            object.__setattr__(self, '_live_rects', (key, got))  # not geometry
            return got
        return self.rect(x, y, rot), self.tht_rect(x, y, rot)

    def gap_to(self, other, self_rects=None, other_rects=None):
        """Smallest gap to another part over the board sides they SHARE, or None
        when they share none -- then they cannot interact at all and every
        consumer must skip the pair.

        Both parts' rect pairs are passed in so a caller can hoist them out of a
        loop; each defaults to the part's live pose.
        """
        sr = self.rects() if self_rects is None else self_rects
        orr = other.rects() if other_rects is None else other_rects
        return pair_min_gap(self.sides, self.side, sr[0], sr[1],
                            other.sides, other.side, orr[0], orr[1])

    def _pad_offsets(self, rot):
        """[(dx, dy, net)]: each connected pad's offset from the origin at
        rotation `rot` -- `local_to_global`'s rotation half, by the SAME
        expressions in the same order, so `x + dx` is the very sum it forms.

        Memoised per rotation: the trig and the products depend on the
        rotation alone, and a quench asks for the same handful of rotations
        at every candidate position. A key that compares equal but differs in
        the sign of a zero (0.0 / -0.0) changes only the sign of a zero
        offset, which the integer-nm snap in `pad_globals` erases.
        """
        memo = self._pg_rot
        if memo is None or memo[0] is not self.pads_local:
            memo = (self.pads_local, {})
            object.__setattr__(self, '_pg_rot', memo)      # not geometry
        offs = memo[1].get(rot)
        if offs is None:
            rad = math.radians(-rot)  # CRITICAL: negate the angle (as there)
            cos_r = math.cos(rad)
            sin_r = math.sin(rad)
            offs = memo[1][rot] = [(lx * cos_r - ly * sin_r,
                                    lx * sin_r + ly * cos_r, n)
                                   for lx, ly, n in self.pads_local]
        return offs

    def pad_globals(self, x=None, y=None, rot=None):
        """[(gx, gy, net)] of every connected pad at a pose (default: live).

        BIT-IDENTICAL to `local_to_global(x, y, rot, lx, ly)` per pad -- the
        offset from `_pad_offsets`, then the same integer-nm snap
        (`mm_to_iu(...) / IU_PER_MM`) -- and
        tests/test_quench_pad_globals_memo.py holds it to that, bit for bit.

        The last pose's answer is memoised: a candidate is evaluated net by
        net and asks for the same pose once per net, and `_net_points` asks
        every OTHER part on a net for its live pose. Measured on
        routed_output, this transform was 78% of a placement run. Every call still returns a NEW list, so a caller that
        edits its copy cannot reach the memo.
        """
        x = self.x if x is None else x
        y = self.y if y is None else y
        rot = self.rot if rot is None else rot
        key = (x, y, rot)
        memo = self._pg_pose
        if (memo is not None and memo[1] is self.pads_local
                and memo[0] == key):
            return list(memo[2])
        out = [(mm_to_iu(x + dx) / IU_PER_MM, mm_to_iu(y + dy) / IU_PER_MM, n)
               for dx, dy, n in self._pad_offsets(rot)]
        object.__setattr__(self, '_pg_pose', (key, self.pads_local, out))
        return list(out)


class QuenchState:
    """Current placement plus cached airwires: the seat gate
    (`candidate_valid`) and the measurements the graders read."""

    def __init__(self, pcb_data: PCBData, pcb_file: str,
                 clearance: float, board_edge_clearance: float,
                 ignore_net_ids: Optional[Set[int]] = None,
                 extra_locked_refs: Optional[Set[str]] = None,
                 # --- #701 intent keep-outs, empty by default so a state that
                 # does not ask for them -- INCLUDING the one `floorplan.grade`
                 # builds, which must keep measuring independently of the seat
                 # gate -- has none. Consumed by `seeder.pose_ok` under an
                 # ABSOLUTE policy, and since #702 by `candidate_valid` under a
                 # MONOTONE one: a hard gate on which poses exist, not a cost.
                 keepouts: Optional[Sequence[Dict]] = None,
                 # --- #702 declared zones, empty by default for the same
                 # reason. Plain data from `floorplan.resolve_intent_gate`,
                 # never an `Intent`: the engine must not import that schema
                 # to run, and `floorplan.grade` builds a state of its own that
                 # has to keep measuring independently of whatever a seat was
                 # gated on.
                 intent_zones: Optional[Sequence[Dict]] = None,
                 # --- declared EXCLUSIVE zones, for the SEAT predicate
                 # (`exclusive_clear`). A SEPARATE parameter from
                 # `intent_zones`: this one carries the must-be-OUTSIDE slice
                 # alone, gated ABSOLUTELY, while `intent_zones` feeds
                 # `candidate_valid`'s MONOTONE `intent_ok`. Same plain data
                 # (`floorplan.zone_entries`), empty by default.
                 exclusive_zones: Optional[Sequence[Dict]] = None,
                 # --- #1300: a part that draws no courtyard is seated on an
                 # ESTIMATED one (`body.estimated_courtyards`). Off by
                 # default, so a state that grades what the board draws --
                 # INCLUDING the one `floorplan.grade` builds -- keeps doing
                 # so; `place_simple` and the renderer that draws its model
                 # ask for it.
                 estimate_courtyards: bool = False,
                 # ...except these refs, which keep the basis the board
                 # draws (place_simple's fallback for a part the estimates
                 # leave no legal pose)
                 estimate_skip=()):
        bounds = pcb_data.board_info.board_bounds
        if bounds is None:
            raise ValueError("No board boundary (Edge.Cuts) found")
        self.board = bounds
        margin = max(clearance, board_edge_clearance)
        self.usable = (bounds[0] + margin, bounds[1] + margin,
                       bounds[2] - margin, bounds[3] - margin)
        # Real board outline / cutouts (#456 item 2): `usable` is a bbox inset,
        # so on an L-shaped outline or a board with interior cutouts it would
        # admit parts in the notch or the hole. The gate measures against the
        # true Edge.Cuts rings and self-disables when the bbox inset is already
        # exact (single rectangular ring, no cutouts) or when the parser found no
        # usable ring at all -- in which case behaviour is unchanged.
        self.edge_gate = BoardOutlineGate(pcb_data.board_info, margin)
        self.clearance = clearance
        # #975: the edge floor ITSELF. `edge_gate.margin` is the max of the two
        # floors, so the pad-copper edge check an edge seat makes (a floor, not
        # a copper clearance) cannot be recovered from it.
        self.board_edge_clearance = board_edge_clearance

        # Kept so the FAB body channel can be resolved lazily -- see
        # `fab_rect`. Nothing is parsed unless something asks.
        self.pcb_file = pcb_file
        self.pcb_data = pcb_data
        self._fab_local = None
        self._fab_cache = {}

        # #1300: with `estimate_courtyards`, every pad-bearing part that
        # draws no courtyard enters this map with an ESTIMATED one -- the
        # occupancy check_assembly grades it on (its .Fab body, else silk,
        # else pads, united with the pads), with no margin round it -- so
        # `rect`, `grade_rect` and every seat test below see it
        # where they would see a drawn courtyard. Before, such a part was
        # seated on its PAD box: on multichannel_mixer an electrolytic's
        # 4.0 x 1.5 mm pads stand in a 5 mm body, and every seed packed caps
        # into each other's bodies, NOT BUILDABLE. `_drawn_courtyards` keeps
        # what the board draws, for the one test that must judge it as KiCad
        # does (`_Part.drawn_rect`, a declared pose against a declared pose).
        self._drawn_courtyards = extract_courtyard_sides(pcb_file)
        courtyards = self._drawn_courtyards
        self.courtyard_estimates: Dict[str, object] = {}
        if estimate_courtyards:
            from placement.body import courtyard_sides_with_estimates
            courtyards, self.courtyard_estimates = (
                courtyard_sides_with_estimates(pcb_data, pcb_file,
                                               drawn=self._drawn_courtyards,
                                               skip=frozenset(estimate_skip)))
        file_locked = set(extract_locked_refs(pcb_file))
        locked_refs = set(file_locked)
        if extra_locked_refs:
            locked_refs |= extra_locked_refs
        ignore = ignore_net_ids or set()

        locked_bodies: Optional[Dict[str, Tuple[float, float, float, float]]] = None
        self.parts: Dict[str, _Part] = {}
        #: #1152: {ref: LOCK_*} for every part entered locked -- see LOCK_FILE.
        self.lock_reasons: Dict[str, str] = {}
        no_courtyard = []
        outline_locked = []   # #829, reported below
        for ref, fp in pcb_data.footprints.items():
            # `fp.pads`, not `non_aperture_pads` -- the one #1143 site left
            # reading every pad, deliberately. A part whose only pads are
            # paste/mask apertures (a logo) stays MOVABLE here, seated by its
            # courtyard when the intent fixes its pose. A truly pad-less
            # footprint is locked below, under LOCK_PADLESS: a policy, not a
            # pose -- a declared fixed pose still seats it by the same
            # courtyard (#1152).
            if not fp.pads:
                # Zero-pad footprints (graphics-only mechanical parts, logos
                # with a courtyard) enter as locked static obstacles when
                # they draw a courtyard; without one there is no geometry to
                # respect.
                if ref in courtyards:
                    self.parts[ref] = _Part(ref, fp, courtyards, True)
                    self.lock_reasons[ref] = (
                        LOCK_FILE if ref in file_locked else
                        LOCK_CALLER if ref in locked_refs else LOCK_PADLESS)
                continue
            # #829: a footprint that draws part of the BOARD's own boundary is
            # never this tool's to move -- its pose transforms that Edge.Cuts
            # geometry, so moving it resizes the board, which is a mechanical
            # decision the user owns. A lock INPUT like the file's and the
            # caller's: no un-lock operator, so it can never override what a
            # caller asked for.
            #
            # `owns_board_outline`, NOT `owns_edge_cuts`. Geometry parented to
            # a footprint is usually a relief the designer bound to the part so
            # it travels with it -- crkbd draws 184 per-LED windows that way,
            # and #628 exempts a part's own milled ring from the edge-margin
            # test precisely to keep such a part placeable (without it run 20's
            # SW2 had 0 legal poses of 14884). Only geometry lying outside the
            # board-level outline is the board's.
            owns_outline = getattr(fp, 'owns_board_outline', False)
            locked = ref in locked_refs or owns_outline
            if owns_outline:
                outline_locked.append(ref)
            body_local = None
            if ref not in courtyards:
                no_courtyard.append(ref)
                # A LOCKED part without a courtyard is an obstacle by the
                # body check_assembly grades it on (`locked_body_locals`),
                # not by its pad box: nothing moves it, so its body is the
                # board's, and a part seated on it is a blocking pair.
                if locked:
                    if locked_bodies is None:
                        locked_bodies = locked_body_locals(pcb_data, pcb_file)
                    body_local = locked_bodies.get(ref)
            self.parts[ref] = _Part(
                ref, fp, courtyards, locked, body_local,
                drawn_sides=(self._drawn_courtyards
                             if self.courtyard_estimates else None))
            if locked:
                self.lock_reasons[ref] = (
                    LOCK_FILE if ref in file_locked else
                    LOCK_CALLER if ref in locked_refs else LOCK_OUTLINE)
            # Ignored nets (e.g. plane-routed power) don't contribute airwires
            self.parts[ref].nets = [n for n in self.parts[ref].nets
                                    if n not in ignore]
        # #829. DISCLOSED, never silent: a freeze nobody is told about is
        # the failure mode lock_advisor's 'advice, never action' rule
        # exists to prevent. The reader needs the ref AND why, because
        # the remedy is not in this toolchain -- part and outline have to
        # move together, in KiCad.
        self.outline_locked = sorted(outline_locked)
        if self.outline_locked:
            print(f"  Locked because they draw the board outline (#829): "
                  f"{', '.join(self.outline_locked)} -- moving one would"
                  f" resize the board. Edit the part and the outline"
                  f" together in KiCad if it really must move.")
        warn_missing_courtyards(no_courtyard, 'placement')
        if self.courtyard_estimates:
            from placement.body import format_estimates
            print(format_estimates(self.courtyard_estimates, 'placement'))

        # #701 intent keep-outs, resolved ONCE per state rather than once per
        # candidate pose. Neither an `allow` fnmatch against a reference nor
        # the set of faces a part occupies changes when the part moves, so the
        # only pose-dependent work left for the seat predicate is the geometry
        # itself. The placer evaluates thousands of poses per part, so this
        # is the difference between a conjunct and a regression.
        #
        # `keepouts_for` is EMPTY on every board that declares no keep-out --
        # which is every board in the corpus today -- and `pose_ok` guards on
        # that emptiness, so the whole channel is inert unless asked for.
        # #1098: plus the mating region of every PCB-edge plug on the board
        # (derived from the footprint; a declared `mating:<ref>` wins), so
        # no move puts a part on a USB tongue whatever the intent says. Empty on a board with no such plug.
        from . import floorplan as _fpk
        # #1101: the board's OWN `courtyards_overlap` severity, read the way
        # check_assembly reads it (#1095, `legality.courtyard_severity_of`:
        # an `ignore` this repo's old route steps wrote is not the author's).
        # At `ignore` the author said courtyards may overlap and KiCad checks
        # none; refusing them here made StickHub's port capacitors unseatable
        # -- the column between the JST ports is 1.14 mm between courtyards,
        # and the human's caps overlap them by 0.68 mm. Pads, holes and .Fab
        # bodies are still checked. Inert on every board that does not say
        # `ignore` (none in the tracked corpus).
        try:
            from .legality import courtyard_severity_of as _cso
            self.courtyards_ignored = _cso(pcb_file)[0] == 'ignore'
        except Exception:                                    # noqa: BLE001
            self.courtyards_ignored = False
        self.keepouts = _fpk.with_derived_keepouts(keepouts, pcb_data,
                                                   pcb_file)
        # A plug SEATED at its edge is the mechanical fact its keep-out is
        # derived from: moved inland, the region would go with it and the
        # parts it kept off the tongue would be free to return. So the
        # placer never moves it -- as with a KiCad lock. Only while it IS
        # seated (`seated_plugs`): a plug a declared `mating:` keep-out names
        # but which sits in the staging pile, or hangs across an edge, must
        # stay free, or it would stay where it is.
        for _ref in _fpk.seated_plugs(self.keepouts, pcb_data, pcb_file):
            if _ref in self.parts:
                self.parts[_ref].locked = True
                # Every earlier lock recorded its own cause, which stands.
                self.lock_reasons.setdefault(_ref, LOCK_SEATED_PLUG)
        self.keepouts_for: Dict[str, Tuple[Dict, ...]] = {}
        if self.keepouts:
            from . import floorplan as _fp
            for _ref, _p in self.parts.items():
                _binding = _fp.keepouts_for_ref(self.keepouts, _ref, _p.sides)
                if _binding:
                    self.keepouts_for[_ref] = _binding

        # --- #702 declared claims, resolved ONCE per state ------------------
        # The zone half is built by the free `build_zone_spec` (#698), the one
        # construction of which parts a block binds.
        #
        # `_intent_active` is False on every board that declares nothing --
        # which is every board in the corpus today -- and `candidate_valid`
        # guards on it before any arithmetic, so the whole channel costs one
        # bool load and one branch.
        self.intent_zones = tuple(intent_zones or ())
        self._intent_spec: Dict[str, Tuple[_IntentTerm, ...]] = build_zone_spec(
            self.intent_zones, self.parts)
        self._intent_active = bool(self._intent_spec or self.keepouts_for)

        # --- #797 declared EXCLUSIVE zones, for the SEAT predicate ----------
        # Resolved once per state, like `keepouts_for`, and for the same
        # reason: membership and the side filter are pose-invariant, and the
        # placer evaluates thousands of poses per part.
        #
        # DELIBERATELY NOT FOLDED INTO `_intent_spec`, and not counted in
        # `_intent_active`. Those drive `candidate_valid`'s MONOTONE
        # `intent_ok`, which admits any pose termwise no worse than the one the
        # part is IN -- and before a part is seated, that is its generator
        # pile coordinate. A stranger whose pile coordinate already sits inside
        # a reserved zone would then be admitted to every pose no worse than
        # that, i.e. seated inside it: #797's own bug, back through the door
        # `pose_ok`'s docstring names for keep-outs. Folding it in would also
        # change `intent_spec_for`'s arity, which `_inc_intent` is keyed on.
        #
        # Empty on every board that declares no exclusive zone -- which is
        # every board in the corpus today -- and `exclusive_clear` guards on
        # that emptiness, so the channel is inert unless asked for.
        self.exclusive_zones = tuple(exclusive_zones or ())
        self.exclusive_for: Dict[str, Tuple[_IntentTerm, ...]] = exclusive_spec(
            self.exclusive_zones, self.parts)
        # ref -> the incumbent pose's term vector. Cleared beside
        # `_inc_violation` on every move, for the same reason. Never computed
        # on a compliant board: `intent_ok` returns on its absolute branch.
        self._inc_intent: Dict[str, Tuple[float, ...]] = {}

        # Run-6 CONTAINER exemption: a courtyard covering most of the board
        # is a FRAME (a module-outline footprint hosting the whole design),
        # not a body -- measured on rp2350_fpga_eensy: U8's courtyard is
        # 1.13x the board area and the courtyard-hard gate refused EVERY
        # pose on the board (13 unrepairable, 0.00mm moved), while the next
        # largest ratio anywhere in the 33-board corpus is 0.29 (a
        # connector). Pairs with a container member skip the courtyard
        # channels; the PAD layer (pads_ok) still applies in full -- the
        # module's pads are real obstacles.
        #
        # fa10 P1 (#1184, #1212): WHO is a container is the grader's own
        # decision (`legality.container_kinds`): a pin frame or a pad-less
        # outline, never a big BODY and never a drawn courtyard. Area alone
        # called One-Air-Max's 18650 holder a frame and the seed put DC1
        # wholly inside it, while check_assembly gated the pair. A pin
        # frame's PINS still bind: `_pin_conflict_at`, the grader's
        # `pin_hits`, refuses a part on one (KiCad's pth_inside_courtyard).
        try:
            _kinds = legality.container_kinds(
                pcb_data, legality.part_local_bounds(pcb_data, pcb_file))
        except Exception:                                    # noqa: BLE001
            _kinds = {}
        self.container_kinds = {r: k for r, k in _kinds.items()
                                if r in self.parts}
        self.container_refs = set(self.container_kinds)
        self.pin_frame_refs = {r for r, k in self.container_kinds.items()
                               if k == 'pin_frame'}
        self._pin_census_obj = None
        self._pin_copper = {}
        if self.container_refs:
            print(f"  container footprint(s) (>= {CONTAINER_RATIO:.0%} of "
                  f"the board, a pin frame or an outline -- frame, not "
                  f"body): " + ', '.join(
                      f"{r} ({k})"
                      for r, k in sorted(self.container_kinds.items())))

        # --- pad + drill legality (gate currency; see placement/legality.py).
        # pose_of/seed_of read the live _Part records, so the context follows
        # every apply_move with no invalidation; baselines key off SEED poses.
        # #697: the per-pair required clearance (pad overrides, net
        # classes, .kicad_dru layer rules), resolved from the board's own
        # siblings exactly as check_drc does. Inert -- and every gate below
        # then behaves as it did -- on a board that declares none of them.
        pad_model = legality.PadClearanceModel.for_board(
            pcb_data, clearance, pcb_file)
        pad_model = pad_model if pad_model.active else None
        # #761: the board's own copper-to-NPTH-hole floor, resolved
        # once. This context serves `pair_shortfall`, which reads hole
        # keep-outs, so it is one of the two call sites that need it.
        #
        # The notes are PRINTED rather than returned: unlike
        # `grade_pad_legality` this constructor has no report to file them
        # into, and a silent fallback drops the modelled floor to the flat
        # fab value while every downstream number looks normal. Empty on
        # every board that resolves cleanly.
        _npth_notes = []
        _npth = legality.resolve_npth_floor(pcb_data, pcb_file,
                                            _npth_notes)
        for _n in _npth_notes:
            print('WARNING: %s' % _n)
        part_pads = legality.build_part_pads(
            {ref: pcb_data.footprints[ref] for ref in self.parts
             if ref in pcb_data.footprints}, clearance, pad_model,
            npth_floor=_npth)
        self.legality_ctx = legality.LegalityContext(
            part_pads, self.edge_gate, clearance,
            pose_of=lambda r: (self.parts[r].x, self.parts[r].y,
                               self.parts[r].rot),
            seed_of=lambda r: (self.parts[r].seed_x, self.parts[r].seed_y,
                               self.parts[r].orig_rot),
            model=pad_model,
            # #1031: the board's rule-area keep-outs; inert (None
            # inside) on a board that declares none with tracks
            # forbidden.
            keepouts=legality.RuleAreaKeepouts.for_board(
                pcb_data, clearance, pcb_file),
            # #1266: footprint graphic copper (a SOT-89 tab), charged
            # against other parts' pads; empty on most boards.
            graphics=legality.build_part_graphics(
                pcb_data, refs=set(part_pads)))

        # net -> refs touching it, as a SORTED LIST, not a set (#457).
        #
        # This order reaches compute_mst_edges as the point order, and Prim's
        # tie-break there is first-index-wins (seed node 0, argmin, and a strict
        # `<` on the frontier update). Equidistant pads are the norm on a real
        # board -- uniform-pitch GND arrays, identical decaps on a grid,
        # symmetric connectors -- so a different order builds a different tree
        # of the same total length, which changes the crossing count, which
        # changes which moves get accepted. Set-of-STRING iteration order is
        # randomized per process (PYTHONHASHSEED), so identical inputs gave
        # different boards: interf_u_unrouted scored 447 / 457 / 450 crossings
        # under three seeds before a single move was made.
        #
        # Sorting (rather than merely fixing an insertion order) makes the
        # labelling a property of the NET, not of whichever front enumerated the
        # footprints -- the same argument connectivity.py's
        # get_multipoint_net_pads makes for sorting by position. Geometry is
        # untouched, so this cannot pick a worse tree; it only settles which of
        # several equivalent trees everyone agrees on.
        by_net: Dict[int, List[str]] = {}
        for ref, part in self.parts.items():
            for n in part.nets:
                by_net.setdefault(n, []).append(ref)
        self.net_refs: Dict[int, List[str]] = {
            n: sorted(refs) for n, refs in by_net.items()}

        # Per-net airwires cache
        self.net_airwires: Dict[int, List] = {}
        for net_id in self.net_refs:
            self.net_airwires[net_id] = self._build_net_airwires(net_id)

        # The full-sweep prune (`_sweep_others`), built on first use; False
        # when `parts` cannot be indexed.
        self._near_ix = None
        # ref -> violation of its CURRENT pose; whole-dict invalidated on any
        # move, since a move changes its neighbours' violations too.
        self._inc_violation: Dict[str, float] = {}
        # #1259: ref -> (x, y, rot), the pose `candidate_valid`'s off-board
        # escape branch measures as the INCUMBENT -- `_incumbent_violation`
        # and the #1257 pad-copper conjunct both read it (`_incumbent_pose`).
        # Empty, the incumbent is the part's current pose, as it is for every
        # caller but one: place_pose's snap ranker (`pose_ops._ranker_state`)
        # moves the part to the REQUEST before ranking, and an off-board
        # request as the incumbent licensed every pose merely less off the
        # board than it, each one refused by the re-grade against the input.
        # Not cleared by a move: it is a fact about the caller's baseline.
        self.incumbent_at: Dict[str, Tuple[float, float, float]] = {}
        # #1277: refs whose escape branch a caller has CLOSED, so
        # `candidate_valid` admits for them only what the ordinary gate
        # admits. Empty for every caller but one: the snap ranker
        # (`pose_ops._rank`), which ranks a part whose input pose is off the
        # board once with the branch closed, ahead of the licensed ranking.
        self.escape_closed: Set[str] = set()
        # ref -> ids of the milled rings its OWN pads sit inside (#628). Keyed
        # on the SEED pose, so unlike _inc_violation this NEVER invalidates.
        self._owned_rings_cache: Dict[str, frozenset] = {}

    # ----- airwire helpers -------------------------------------------------

    def _net_points(self, net_id, override_ref=None, override_pads=None):
        """A net's pad points, with `override_ref` held at a HYPOTHETICAL
        pose whose pad globals are `override_pads` (#459).

        The `for ref in self.net_refs[net_id]` iteration is load-bearing:
        net_refs is a SORTED list and this order becomes the point order handed
        to compute_mst_edges, whose tie-break is first-index-wins. Changing it
        makes quench output vary across processes again (#457).
        """
        pts = []
        for ref in self.net_refs[net_id]:
            pads = override_pads if ref == override_ref else None
            if pads is None:
                pads = self.parts[ref].pad_globals()
            pts.extend((gx, gy) for gx, gy, n in pads if n == net_id)
        return pts

    def _build_net_airwires(self, net_id, override_ref=None, override_pads=None):
        return _airwires_for_points(
            self._net_points(net_id, override_ref, override_pads), net_id)

    # ----- legality -------------------------------------------------------

    def _sweep_others(self, rects, clr):
        """The full-board sweep's `(ref, part)` pairs for a part presenting
        `rects` (its `rects()` at the pose asked about) at clearance `clr`:
        `parts` in order, less the parts `_NearIndex` proves stay `clr`
        clear of it. For the courtyard loops of `violation_parts` and
        `candidate_valid` only -- a pair test with another reach (pad floors,
        bodies) needs its own bound."""
        if len(self.parts) < _NEAR_INDEX_MIN_PARTS:
            return self.parts.items()
        ix = self._near_ix
        if ix is None or (ix is not False and not ix.current(self.parts)):
            if ix is not None:
                ix.detach()                     # superseded: stop feeding it
            ix = self._near_ix = _NearIndex.build(self.parts) or False
        if ix is False:
            return self.parts.items()
        q = _footprint_box(rects[0], rects[1])
        near = None if q is None else ix.near(q, clr + _NEAR_SLACK_MM)
        return self.parts.items() if near is None else near

    def violation_parts(self, ref, x=None, y=None, rot=None,
                        exclude: Optional[Set[str]] = None,
                        limit: Optional[float] = None):
        """(board violation, overlap violation) of a pose; (0, 0) when legal.

        Kept apart because only the BOARD term drives the unfreezing rule in
        candidate_valid. The overlap term is the summed clearance shortfall
        against every part this one shares a side with -- a DISTANCE, whereas the
        overlap metric a placement is graded on is an AREA, and the two do not
        move together: trading one deep narrow overlap for a shallow wide one
        reduces the shortfall while increasing the area. So it can order poses,
        but it must not be used to license a move.

        `limit` lets the caller stop as soon as the running total exceeds it and
        return some value above it -- the accept test only asks "is this worse
        than X", and without the early exit an obviously-worse candidate still
        pays a full neighbour sweep. Never pass a limit when you need the value.
        """
        part = self.parts[ref]
        rects = part.rects(x, y, rot)
        # Same reachability prune as candidate_valid: a part that cannot come
        # near a ring pays only the bbox term (the ring terms cost ~100x), and
        # one that can measures only against the edges it can actually reach.
        # The BOARD term on the grade ladder (#1182), as candidate_valid.
        near = self._edges_near(ref) if self.edge_gate.active else None
        board = self.edge_gate.rect_outside_amount(
            part.grade_rect(x, y, rot), exact=bool(near), edges=near,
            skip_rings=self._owned_rings(ref))
        overlap = 0.0
        if limit is not None and board > limit:
            return board, overlap
        others = self.parts.items()
        if getattr(self, 'courtyards_ignored', False):
            # #1104: the courtyard is waived, so the overlap term is the pad
            # and hole one the waived seat asks (#1101), absolutely -- and a
            # frame's PIN still counts: it is pth/npth_inside_courtyard, not
            # the waived rule, and without it the escape branch below seated
            # a part on a pin the grader gates (second phase-3 verifier).
            overlap = self._waived_overlap(ref, x, y, rot, others, exclude,
                                           limit, board)
            if limit is not None and board + overlap > limit:
                return board, overlap
            return board, overlap + self._pin_violation(ref, x, y, rot,
                                                        exclude)
        clr = self.clearance
        others = self._sweep_others(rects, clr)
        rect = rects[0]
        tht = part.has_tht
        skip_containers = (ref in self.container_refs)
        for other_ref, other in others:
            if other_ref == ref or (exclude and other_ref in exclude):
                continue
            if skip_containers or other_ref in self.container_refs:
                continue    # run-6: container = frame, not body
            if tht or other.has_tht:
                gap = part.gap_to(other, rects)
                if gap is None:
                    continue
            else:
                if other.side != part.side:
                    continue
                r = other.rect()
                if (rect[2] + clr <= r[0] or r[2] + clr <= rect[0]
                        or rect[3] + clr <= r[1] or r[3] + clr <= rect[1]):
                    continue        # clear, so no shortfall to add
                gap = rect_gap(rect, r)
            if gap < clr:
                overlap += clr - gap
                if limit is not None and board + overlap > limit:
                    return board, overlap
        return board, overlap + self._pin_violation(ref, x, y, rot, exclude)

    def _pin_violation(self, ref, x, y, rot, exclude=None) -> float:
        """#1212: a part on a frame's pin is a violation too -- the
        clearance, as for any refused pair -- so `violation() == 0` keeps
        implying `candidate_valid` admits it, on the waived path as well."""
        if not self.pin_frame_refs:
            return 0.0
        part = self.parts[ref]
        _px = part.x if x is None else x
        _py = part.y if y is None else y
        _pr = part.rot if rot is None else rot
        if self._pin_conflict_at(ref, _px, _py, _pr, exclude) is not None:
            return self.clearance
        return 0.0

    def _pin_conflict_at(self, ref, x, y, rot, exclude=None):
        """The first GATING pin_in_courtyard pair `ref` makes at this pose,
        as the GRADER finds it (`legality.CourtyardCensus.pin_hits`, the
        function check_assembly's channel runs), or None. Frames sit at their
        current poses; a frame being moved is graded against every part."""
        for q in self._pin_hits_at(ref, x, y, rot):
            other = q.b if q.a == ref else q.a
            if exclude and other in exclude:
                continue
            return q
        return None

    def pin_hole_boxes(self, frame) -> list:
        """The bounds of pin frame `frame`'s drilled holes at its current
        pose, as the grader tests them (`CourtyardCensus._frame_holes`): what
        place_simple's anneal prices a courtyard over (#1307)."""
        if self._pin_census_obj is None:
            self._pin_census_obj = legality.CourtyardCensus(self.pcb_data,
                                                            self.pcb_file)
        p = self.parts[frame]
        return [hb for _n, _s, hb, _k in
                self._pin_census_obj._frame_holes(frame, (p.x, p.y, p.rot))]

    def pin_copper_boxes(self, frame) -> list:
        """[(x0, y0, x1, y1, sides)] of pin frame `frame`'s copper pads at
        its current pose, each grown by the clearance: what place_simple's
        anneal keeps a part's pad copper off, beside the holes
        (`pin_hole_boxes`). A castellated pin's copper reaches well past its
        hole -- a 2.07 x 1.6 mm edge pad and a 1.6 mm ring around 1.1 mm
        drills -- so pricing the holes alone let a capacitor's pads settle on
        a pin's copper with the hole clear, and the legality pass after the
        anneal found the capacitor no legal pose in a crowded corner."""
        p = self.parts[frame]
        key = (frame, p.x, p.y, p.rot)
        hit = self._pin_copper.get(key)
        if hit is not None:
            return hit
        c = self.clearance
        out = []
        for q in legality.pads_at_pose(self.pcb_data.footprints[frame],
                                       (p.x, p.y, p.rot)):
            if q.pad_type == 'np_thru_hole':
                continue                    # a hole, priced by pin_hole_boxes
            if q.pad_type == 'thru_hole':
                sides = legality.BOTH_SIDES
            else:
                lay = [str(x) for x in (q.layers or ())]
                sides = frozenset(s for s in ('F', 'B')
                                  if f'{s}.Cu' in lay or '*.Cu' in lay)
            if not sides:
                continue
            cx, cy, hx, hy = legality.pad_copper_box(q)
            out.append((cx - hx - c, cy - hy - c, cx + hx + c, cy + hy + c, sides))
        if len(self._pin_copper) > 64:
            self._pin_copper = {}
        self._pin_copper[key] = out
        return out

    def _pin_hits_at(self, ref, x, y, rot, poses=None) -> list:
        """Every GATING pin_in_courtyard pair `ref` makes at this pose.
        `poses` ({ref: (x, y, rot)}) overrides the parts' current poses --
        a declared pose is judged against its obstacles' DECLARED poses
        (`seeder._fixed_pose_check`), not wherever they sit right now."""
        # NOT skipped under `courtyards_ignored`: that is courtyards_overlap,
        # and a pin is KiCad's pth/npth_inside_courtyard, whose own project
        # severity the grader's pairs already honour (phase-3 verifier).
        if not self.pin_frame_refs:
            return []
        if self._pin_census_obj is None:
            self._pin_census_obj = legality.CourtyardCensus(self.pcb_data,
                                                            self.pcb_file)
        if poses is None:
            refs = (self.parts if ref in self.pin_frame_refs
                    else self.pin_frame_refs)
            poses = {r: (self.parts[r].x, self.parts[r].y,
                         self.parts[r].rot) for r in refs}
        return self._pin_census_obj.pin_hits(ref, (x, y, rot), poses)

    def intent_spec_for(self, ref) -> Tuple[_IntentTerm, ...]:
        """The claims binding `ref` right now: frozen zone terms, plus keep-out
        terms derived LIVE from `keepouts_for`.

        The keep-out slice is deliberately not frozen -- see `intent_spec`.

        Still pose-INVARIANT and still resolved once: `keepouts_for` is the
        cached resolution, and this only reads it.
        """
        return intent_spec(self._intent_spec, self.keepouts_for, ref)

    def intent_terms(self, ref, rects) -> Tuple[float, ...]:
        """This pose measured against every declared claim binding `ref`, in
        the fixed order `intent_spec_for` returns.

        A VECTOR, never a scalar -- see `_IntentTerm`.
        """
        return intent_term_values(self.intent_spec_for(ref), rects)

    def intent_clear(self, ref, rects) -> bool:
        """ABSOLUTE: every term at or below its own threshold.

        The SEAT policy. Placement from scratch has no incumbent worth
        improving on (`seeder.pose_ok`), so the seat gate demands cleanliness
        rather than non-worsening: a part whose current pose is fully inside a
        keep-out is REFUSED, which the monotone rule below would admit.
        """
        spec = self.intent_spec_for(ref)
        if not spec:
            return True
        return all(v <= t.threshold
                   for v, t in zip(self.intent_terms(ref, rects), spec))

    def _waived_overlap(self, ref, x, y, rot, others, exclude, limit, board):
        """`violation_parts`' overlap term on a courtyard-waived project
        (#1104): per neighbour on a shared face, the absolute pad + hole
        shortfall (`pair_shortfall`), a pad short or stack counted as the
        clearance, and a stacked drill hole likewise -- the questions the
        #1101 waived seat asks, as a distance rather than a verdict."""
        from .seeder import _drill_conflict
        part = self.parts[ref]
        x = part.x if x is None else x
        y = part.y if y is None else y
        rot = part.rot if rot is None else rot
        ctx = self.legality_ctx
        drilled = self._drilled_refs()
        clr = self.clearance
        overlap = 0.0
        for other_ref, other in others:
            if other_ref == ref or (exclude and other_ref in exclude):
                continue
            if not (part.sides & other.sides):
                continue
            if (ref in drilled and other_ref in drilled and _drill_conflict(
                    self, ref, (x, y, rot), other_ref,
                    (other.x, other.y, other.rot))):
                overlap += clr
            if ctx is not None:
                sf = ctx.pair_shortfall(ref, other_ref, pose_a=(x, y, rot))
                overlap += max(0.0, sf.pad) + max(0.0, sf.hole)
                if sf.pad_overlap or sf.stack:
                    overlap += clr
            else:
                # The waived seat's own fallback: pad boxes at the clearance.
                mine, ob = part.padbox(x, y, rot), other.padbox()
                if mine is not None and ob is not None:
                    gap = rect_gap(mine, ob)
                    if gap < clr:
                        overlap += clr - gap
            if limit is not None and board + overlap > limit:
                break
        return overlap

    def _drilled_refs(self):
        """Refs with any drilled pad (plated or not), cached -- the parts a
        hole-to-hole check can concern (#1101)."""
        got = getattr(self, '_drilled_cache', None)
        if got is None:
            got = frozenset(
                r for r, fp in (self.pcb_data.footprints or {}).items()
                if any(max(float(getattr(p, 'drill', 0) or 0),
                           float(getattr(p, 'drill_w', 0) or 0)) > 0
                       for p in fp.pads))
            self._drilled_cache = got
        return got

    def keepout_clear(self, ref, rects) -> bool:
        """The keep-out slice of `intent_clear`, absolute (#701's policy).

        One loop, so `seeder.pose_ok` owns no copy -- the doctrine
        `floorplan.keepout_hit`'s own header states.
        """
        if not self.keepouts_for:
            return True
        from . import floorplan as _fp
        for k in self.keepouts_for.get(ref, ()):
            if _fp.keepout_hit(k, rects):
                return False
        return True

    def keepout_blockers(self, ref, rects) -> List[str]:
        """Names of the keep-outs `ref` is in at this pose. #701's doctrine is
        that a claim which strands a part is a NAMED verdict."""
        if not self.keepouts_for:
            return []
        from . import floorplan as _fp
        return [str(k.get('name') or '<unnamed>')
                for k in self.keepouts_for.get(ref, ())
                if _fp.keepout_hit(k, rects)]

    def exclusive_clear(self, ref, rects) -> bool:
        """The zone_exclusive slice of `intent_clear`, ABSOLUTE (#797).

        `keepout_clear`'s policy, one rule over, and for the same reason:
        placement from scratch has no incumbent worth improving on, so a seat
        search demands cleanliness rather than non-worsening. See
        `seeder.pose_ok`, which states the two-policies-one-loop split.

        Measured through `intent_term_values`, never a local `rect_overlap_area`
        call: that function's `zone_exclusive` branch is where the decision to
        read the COURTYARD ONLY lives ("matching the grade includes matching
        what it declines to measure"), and a second copy here is how the seat
        gate would come to grade a through-hole part's leads that
        `rule_zone_exclusive` does not.
        """
        spec = self.exclusive_for.get(ref) if self.exclusive_for else None
        if not spec:
            return True
        return all(v <= t.threshold
                   for v, t in zip(intent_term_values(spec, rects), spec))

    def exclusive_blockers(self, ref, rects) -> List[str]:
        """Names of the BLOCKS whose exclusive zone `ref` intrudes on at this
        pose. #701's doctrine one rule over: a claim that strands a part is a
        NAMED verdict, not a silent missing pose."""
        spec = self.exclusive_for.get(ref) if self.exclusive_for else None
        if not spec:
            return []
        return [t.name for v, t in zip(intent_term_values(spec, rects), spec)
                if v > t.threshold]

    def _incumbent_intent(self, ref) -> Tuple[float, ...]:
        """The term vector of the pose `ref` is IN, cached until it moves.

        No `exclude` key, unlike `_incumbent_violation`: the intent terms are
        part-vs-DECLARED-GEOMETRY, never part-vs-part, so nothing another part
        does can change them. That is also why they are the right gate for the
        swap phase, where `candidate_valid` is not.
        """
        v = self._inc_intent.get(ref)
        if v is None:
            v = self.intent_terms(ref, self.parts[ref].grade_rects())
            self._inc_intent[ref] = v
        return v

    def intent_ok(self, ref, x, y, rot, rects=None) -> bool:
        """MONOTONE: a pose is admitted when every term is clean, or --
        TERMWISE -- no worse than the pose the part is in.

        Termwise and never summed, and never traded across terms: a part may
        not buy its way into keep-out B by leaving keep-out A.

        The incumbent is computed only on the branch that needs it, and cached
        -- the same trade `candidate_valid` makes at its own escape branch
        ("Only now, on a rejected candidate, is the incumbent's legality worth
        computing"). On a compliant board it is never computed at all.

        NON-STRICT (`<=`), unlike the off-board branch's strict compare. That
        branch needs strictness because it hands out a licence to be ILLEGAL;
        this one does not: it is a FILTER on which poses exist, and the
        caller's own acceptance rule chooses between them. Strictness here
        would be a bug: `keepout_hit` reports a circle as a fabricated 1.0
        marker, so `<` would freeze a part already inside a circle unless it
        could clear the whole circle in a single move.

        THE TRADE THIS MAKES, STATED. Because the conjunct sits ABOVE the #456
        off-board escape branch and returns rather than setting a flag, a part
        that is OFF THE OUTLINE and whose only reachable homeward poses lie in
        a declared keep-out stays off the outline. That trades a `keepout`
        finding for an off-board part, and CLAUDE.md ranks a part whose pad copper lies
        outside the outline as the TOP-priority placement defect, because it
        converts one-for-one into unrouted and broken nets.

        It is nonetheless the right ordering, for one reason: the alternative
        is a gate that can be defeated by first walking a part off the board.
        A declared keep-out that stops applying under some other violation is
        not a hard constraint. The honest handling is disclosure, not a
        loophole -- `intent_blockers` names the claim that stranded the part,
        and an intent whose keep-outs leave a part no way home is a
        contradiction its author has to see. `zone_covered_by_keepout` catches
        the total-coverage case at load time; the partial case is not caught
        yet, and is filed rather than silently accepted.
        """
        spec = self.intent_spec_for(ref)
        if not spec:
            return True
        if rects is None:
            rects = self.parts[ref].grade_rects(x, y, rot)
        cand = self.intent_terms(ref, rects)
        if all(v <= t.threshold for v, t in zip(cand, spec)):
            return True
        cur = self._incumbent_intent(ref)
        return all(c <= u + legality.EPS for c, u in zip(cand, cur))

    def intent_blockers(self, ref, x, y, rot, rects=None):
        """[(rule, name, measured, incumbent)] for the terms `intent_ok`
        refuses at this pose. DIAGNOSTIC only, and the reason the refusal can
        be reported by NAME rather than as a silent missing pose."""
        spec = self.intent_spec_for(ref)
        if not spec:
            return []
        if rects is None:
            rects = self.parts[ref].grade_rects(x, y, rot)
        cand = self.intent_terms(ref, rects)
        cur = self._incumbent_intent(ref)
        return [(t.rule, t.name, round(c, 4), round(u, 4))
                for c, u, t in zip(cand, cur, spec)
                if c > t.threshold and c > u + legality.EPS]

    #: #1113: the check that refused the candidate `candidate_veto` is
    #: asking about, filled by `_veto` on candidate_valid's rejection paths.
    #: None outside a `candidate_veto` call, so the labels cost one attribute
    #: test on a rejection and nothing on an admission.
    _why = None

    def _veto(self, check, blocker=None, **detail):
        """Record `check` as the reason the current candidate is refused,
        unless an earlier conjunct already gave one (#1113)."""
        if self._why is not None and 'check' not in self._why:
            self._why.update(detail, check=check, blocker=blocker)

    def candidate_veto(self, ref, x, y, rot,
                       exclude: Optional[Set[str]] = None):
        """`None` when `candidate_valid` admits the pose, else `(check,
        blocker)`: the label of the conjunct that refused it and the part it
        was refused against, when there is one. It CALLS
        candidate_valid, so the two can never disagree; the labels are
        written on candidate_valid's own rejection paths."""
        self._why = {}
        try:
            if self.candidate_valid(ref, x, y, rot, exclude):
                return None
            why = self._why
        finally:
            self._why = None
        return (why.get('check', 'unattributed'), why.get('blocker'))

    def candidate_valid(self, ref, x, y, rot, exclude: Optional[Set[str]] = None):
        """True when the pose is legal, or -- when the part sits OFF THE BOARD --
        when it moves strictly back toward the board without overlapping anything.

        The second branch exists because only candidates were ever validated,
        never the incumbent: a part outside the bbox inset, or (now that the real
        outline is enforced) inside a notch or a cutout, had every candidate
        rejected and could never move at all, not even toward the board (#456
        item 1).

        It is deliberately limited to the BOARD term. Extending it to overlaps --
        "any pose no worse than the one you are in" -- measurably destroys the
        constraint on a dense board: on watchy 81 of 82 parts start in violation
        (its hand placement is tighter than the 0.25mm courtyard clearance quench
        asks for), so almost every part gets a licence to slide, and total
        courtyard overlap went 9.1 -> 37.9mm2 instead of the 9.1 -> 0.04mm2 the
        strict gate achieves. Requiring a strict DECREASE instead only softened
        that to 16.8mm2, because the violation measure is a distance while the
        thing being wrecked is an area: trading one deep narrow overlap for a
        shallow wide one lowers the shortfall and raises the area. An overlapping
        part therefore keeps the original rule -- it may move only to a pose that
        is fully legal.
        """
        part = self.parts[ref]
        rects = part.rects(x, y, rot)
        rect = rects[0]
        # #1182: the intent and the board ask the GRADE ladder (what the
        # floorplan grade reads); the neighbour layers below ask `rect`, the
        # occupancy. One object for a part without a body.
        grects = part.grade_rects(x, y, rot)
        grect = grects[0]
        # DECLARED INTENT (#702) -- FIRST, and a `return`, not `legal = False`.
        #
        # First, for the ordering reason `seeder.pose_ok` gives for its own
        # keep-out conjunct: a handful of float compares against a usually-
        # empty tuple, where the neighbour loop below is O(neighbours) and
        # `pads_ok` is another sweep. On a pose the intent refuses this
        # REPLACES that work rather than adding to it.
        #
        # A `return`, because the escape branch at the bottom of this function
        # is a licence to be worse on the BOARD term, for a part coming home
        # from off the board -- and a licence must not compose into a licence
        # to be worse on a DECLARED one. Written as `legal = False` this would
        # be silently overturned there. Nothing that can return True may ever
        # be inserted above this line.
        if self._intent_active and not self.intent_ok(ref, x, y, rot, grects):
            if self._why is not None:
                # #1113: labelled by the first claim the pose breaks.
                b = next(iter(self.intent_blockers(ref, None, None, None,
                                                   grects)), None)
                self._veto('intent', b and f"{b[0]}:{b[1]}")
            return False
        # The board, to `legality.BOARD_TOL`: the rounding `seeder.pose_ok`
        # allows the same rect, so a courtyard flush with `usable` is on the
        # board here too, whichever way its last ulp went.
        u, tol = self.usable, legality.BOARD_TOL
        legal = not (grect[0] < u[0] - tol or grect[1] < u[1] - tol
                     or grect[2] > u[2] + tol or grect[3] > u[3] + tol)
        if not legal and self._why is not None:
            self._veto('board_bbox')
        # Real outline / cutout gate, three-level short-circuit: board-level
        # opt-out, cached per-part reachable-edge list, then the exact test
        # against only those edges.
        if legal and self.edge_gate.active:
            near = self._edges_near(ref)
            if near and self.edge_gate.rect_blocked(
                    grect, edges=near, skip_rings=self._owned_rings(ref),
                    tol=tol):
                legal = False
                if self._why is not None:
                    self._veto('outline')
        # #1257: and the PAD COPPER of a part whose pads reach past its
        # courtyard by more than the margin the two tests above hold it to --
        # the polish pushed `_scoped`'s R2 (a pad 4 mm east of its courtyard)
        # 1.5 mm past the outline with the courtyard legal. ABSOLUTE, like the
        # outline; a part coming home from off the board gets the escape
        # branch's own, monotone, form below.
        if legal and self._pad_copper_past(ref, x, y, rot) > EPS_IMPROVE:
            legal = False
            if self._why is not None:
                self._veto('pad_copper_outline')
        # #962: and the copper the footprint DRAWS (esp_prog U2's F.Cu tab),
        # which reaches past the courtyard the same way, and which the grade
        # place_pose re-grades with refuses past the outline.
        if legal and self._graphic_copper_past(ref, x, y, rot) > EPS_IMPROVE:
            legal = False
            if self._why is not None:
                self._veto('graphic_copper_outline')
        if legal and getattr(self, 'courtyards_ignored', False):
            # #1101: courtyards waived by the project, so the neighbour test
            # asks the PAD question instead, ABSOLUTELY and at each pair's own
            # requirement (net class, pad override -- what the grade prices),
            # never at the seat's ladder clearance: pricing pad boxes at
            # `self.clearance` let four StickHub pairs sit under their 0.15 mm
            # net class at --clearance 0.1 (#1101 verifier). The seed-relative
            # `pads_ok` below cannot stand in: on a pile the seed poses
            # already overlap, so it admits anything.
            mine = part.padbox(x, y, rot)
            if mine is not None:
                others = self.parts.items()
                ctx = self.legality_ctx
                clr = self.clearance
                drilled = self._drilled_refs()
                from .seeder import _drill_conflict
                for other_ref, other in others:
                    if other_ref == ref or (exclude and other_ref in exclude):
                        continue
                    if not (part.sides & other.sides):
                        continue
                    # Hole to hole is not the pad question and the courtyard
                    # that used to keep holes apart is waived (#1101 review).
                    if (ref in drilled and other_ref in drilled
                            and _drill_conflict(
                                self, ref, (x, y, rot), other_ref,
                                (other.x, other.y, other.rot))):
                        legal = False
                        if self._why is not None:
                            self._veto('waived_drill', other_ref)
                        break
                    if ctx is not None:
                        # No padbox prefilter: a padbox is built from pad
                        # ANCHORS, and offset-drill copper sits millimetres
                        # past it; pair_shortfall early-outs on its own
                        # copper extent (#1101 review).
                        sf = ctx.pair_shortfall(ref, other_ref,
                                                pose_a=(x, y, rot))
                        if (sf.pad > EPS_IMPROVE or sf.pad_overlap
                                or sf.stack or sf.hole > EPS_IMPROVE):
                            legal = False
                            if self._why is not None:
                                self._veto('waived_pads', other_ref)
                            break
                    else:
                        ob = other.padbox()
                        if ob is not None and rect_gap(mine, ob) < clr:
                            legal = False
                            if self._why is not None:
                                self._veto('waived_pads', other_ref)
                            break
            # The courtyard is body + margin, and the project waived the
            # MARGIN. Two bodies in one place are still two parts colliding,
            # and the containment conjunct below only refuses half of a body
            # or more: without this, two parts seated with 40% of their
            # .Fab bodies overlapping, pads clear (#1101 review).
            if legal and self._body_overlap_at(ref, x, y, rot, exclude):
                legal = False
        elif legal:
            clr = self.clearance
            others = self._sweep_others(rects, clr)
            tht = part.has_tht
            skip_containers = (ref in self.container_refs)
            for other_ref, other in others:
                if other_ref == ref or (exclude and other_ref in exclude):
                    continue
                if skip_containers or other_ref in self.container_refs:
                    continue    # run-6: container = frame, not body
                if tht or other.has_tht:
                    # Either part reaches the far side: fall through to the
                    # shared-side rule, which needs both parts' rect pairs.
                    gap = part.gap_to(other, rects)
                    if gap is not None and gap < clr:
                        legal = False
                        if self._why is not None:
                            self._veto('courtyard', other_ref, path='tht')
                        break
                    continue
                # Fast path: two plain SMD parts, same side. The per-axis test is
                # an early-OUT, not the verdict -- clearing it on any axis proves
                # the true gap clears too, but failing it does not prove the
                # reverse, because the gap is EUCLIDEAN. Two rects offset
                # diagonally by (0.2, 0.2) at clearance 0.25 fail every axis while
                # rect_gap is hypot(0.2,0.2)=0.283, i.e. legal. Using the axis
                # test as the answer made candidate_valid REJECT poses that
                # violation_parts scores as legal -- so a zero violation did
                # not imply the hard gate passes, and the unfreeze branch could
                # walk a part into a pose the ordinary gate forbids.
                if other.side != part.side:
                    continue
                r = other.rect()
                if (rect[2] + clr <= r[0] or r[2] + clr <= rect[0]
                        or rect[3] + clr <= r[1] or r[3] + clr <= rect[1]):
                    continue                    # provably clear
                if rect_gap(rect, r) < clr:
                    legal = False
                    if self._why is not None:
                        self._veto('courtyard', other_ref, path='smd')
                    break
        if legal and self.pin_frame_refs:
            # #1212: a pin frame's PINS. Its rect is skipped above (a frame,
            # not a body), so without this a part was seated on a Teensy pin
            # (rp2350 seed 4: SW1 over U8 pins 16/17, which KiCad reports and
            # no repo checker saw). The grader's own pairs decide.
            _hit = self._pin_conflict_at(ref, x, y, rot, exclude)
            if _hit is not None:
                legal = False
                if self._why is not None:
                    self._veto('container_pin',
                               _hit.b if _hit.a == ref else _hit.a)
        if legal:
            # BODY layer. A pose that buries this part inside another part's
            # .Fab body is not a trade-off to be priced -- it is illegal.
            # Without it here, the placer could put a part straight back
            # inside the body it was charged for.
            #
            # FAB currency, and only fab. The courtyard would be a false-veto
            # machine: four healthy corpus boards ship frac-1.0 COURTYARD
            # containment (esp_prog, orangecrab, rp2350, ulx3s). The docstring
            # above warns that on watchy 81 of 82 parts start in violation of
            # the courtyard clearance -- that warning is about the courtyard
            # currency and about RELAXING the gate, and it does not transfer:
            # on the fab currency the whole 33-board corpus carries 4 pairs
            # with a maximum non-exempt frac of 0.011.
            #
            # Same marker/container exemption as the prevention gate, or a
            # displaced fiducial could never come home under a connector.
            legal = not self._body_contained_at(ref, x, y, rot, exclude)
            # #1106: the checker grades a body-less part's pads under a
            # drawn body at EVERY severity, so the seat refuses it here, on
            # both the waived and the courtyard path --
            # a body drawn larger than its courtyard leaves room the
            # courtyard test above does not see (review: J9 under a
            # +-3.5 mm body behind a +-1 mm courtyard, at `error`).
            if legal and self._pads_under_body_at(ref, x, y, rot, exclude):
                legal = False
        if legal and self.legality_ctx is not None:
            # Pad+drill layer: courtyard-clear does not imply pad-clear (pads
            # overhanging courtyards, exchanged nets, NPTH holes). Baseline-
            # relative: the pose may not worsen any pair vs the SEED, and a
            # NEW different-net pad intersection is never admitted.
            legal = self.legality_ctx.pads_ok(
                ref, x, y, rot, self._pad_neighbors(ref), exclude=exclude,
                why=self._why)
        if legal:
            return True
        # Only now, on a rejected candidate, is the incumbent's legality worth
        # computing -- and it is cached, because it is the same answer for every
        # candidate of this part until something moves. Without the cache this
        # branch runs a full neighbour sweep per rejected candidate, which on a
        # dense board is most of them.
        cur_board = self.escape_board_term(ref, exclude=exclude)
        if cur_board is None:
            # Overlapping, already legal, or closed by the caller (#1277):
            # original rule, legal poses only.
            return False
        cand_board, cand_overlap = self.violation_parts(
            ref, x, y, rot, exclude=exclude, limit=cur_board)
        if not (cand_overlap <= EPS_IMPROVE
                and cand_board < cur_board - EPS_IMPROVE):
            if (self._why is not None and cand_overlap > EPS_IMPROVE
                    and self._why.get('check') in ('board_bbox', 'outline',
                                                   'pad_copper_outline',
                                                   'graphic_copper_outline')):
                # #1113: a part coming home from off the board, refused for
                # OVERLAP -- the board term the ordinary path named is not
                # what stopped it (a courtyard label keeps its blocker).
                self._why.clear()
                self._veto('escape_overlap')
            return False
        # #1113: past here the ESCAPE rule's own conjuncts decide, so a
        # refusal below is theirs, not the ordinary path's board term.
        if self._why is not None:
            self._why.clear()
        # #1257: coming home, the pad copper of a part `_pad_copper_past`
        # measures may not end further past the outline than it starts --
        # the courtyard term above can improve while a pad reaching past the
        # courtyard moves off the board.
        cand_pad = self._pad_copper_past(ref, x, y, rot)
        if cand_pad > EPS_IMPROVE and cand_pad > self._pad_copper_past(
                ref, *self._incumbent_pose(ref)) + EPS_IMPROVE:
            if self._why is not None:
                self._veto('pad_copper_outline')
            return False
        cand_gfx = self._graphic_copper_past(ref, x, y, rot)
        if cand_gfx > EPS_IMPROVE and cand_gfx > self._graphic_copper_past(
                ref, *self._incumbent_pose(ref)) + EPS_IMPROVE:
            if self._why is not None:
                self._veto('graphic_copper_outline')
            return False
        # #1101: ...and the same BODY conjunct the ordinary path has. Coming in
        # from the pile, a part whose courtyard is small and whose drawn body
        # is large (StickHub's lying-down electrolytic C38: a 6.3 x 11.5 mm
        # .Fab over a pad-sized courtyard) overlaps no courtyard, so this
        # branch seated it on top of eleven parts -- and, before #1101, inside
        # Y1's body. The branch never asked about bodies.
        if self._body_contained_at(ref, x, y, rot, exclude):
            return False
        # #1106: and the body-less half of it. Measured on run 38's pile:
        # the waived seat refused J9 under U1's LQFP body, then this branch
        # accepted the same pose for a part "coming home" from the pile.
        if self._pads_under_body_at(ref, x, y, rot, exclude):
            return False
        # The unfreeze branch gets the SAME pad/hole conjunct: a part may move
        # back toward the board only without worsening any pad pair.
        if self.legality_ctx is not None and not self.legality_ctx.pads_ok(
                ref, x, y, rot, self._pad_neighbors(ref), exclude=exclude,
                why=self._why):
            return False
        return True

    # ----- #1043 tethers ----------------------------------------------------
    # The part-vs-PART claims (`decap_distance`, `decap_pin_distance`,
    # `proximity`), which is why they are not folded into `_intent_spec`:
    # that channel's terms are part-vs-declared-geometry, and its incumbent
    # cache (`_incumbent_intent`) is keyed by the moving ref on exactly that
    # assumption. A tether term reads the poses of every ref it names, so its
    # incumbent is cached PER TERM and a move of any of them invalidates it.
    #
    # The pairings are elected ONCE, by `floorplan.tether_pairings` -- the
    # grader's own election on the board as it stands; a decap term re-runs
    # the cap's election per pose over the chips on its rail, because the
    # grade re-elects -- and every value is measured by CALLING the grader's
    # functions on footprints posed at the live (or candidate) poses:
    # `groups.elect_live`, `floorplan.nearest_rail_cap`,
    # `floorplan.proximity_reaches`.
    #
    # These methods run on a `TetherGateView` (#1067), which binds them and
    # carries the state they read; a QuenchState gates no tethers itself.

    def _build_tethers(self) -> None:
        """Elect the pairings once and freeze them as the GATE's
        `_TetherTerm`s (`tether_terms_for`, with locked terms dropped)."""
        terms = self.tether_terms_for(self.tethers)
        by_ref: Dict[str, List[int]] = {}
        for i, t in enumerate(terms):
            for r in set(t.refs):
                by_ref.setdefault(r, []).append(i)
        self._tether_terms = terms
        self._tethers_of = {r: tuple(v) for r, v in by_ref.items()}

    def tether_terms_for(self, tethers: Dict) -> List[_TetherTerm]:
        """The `_TetherTerm`s `tethers` (`floorplan.tether_gate_spec`) elects
        on the board as it stands. Assigns NOTHING on the state.

        A term none of whose refs can move is dropped, since no move of this
        engine changes it and it cannot refuse anything. A proximity term is expanded to one per REACH
        the rule reports (per declared subject pad, or one for the pair),
        counted on the board as it stands.
        """
        from . import floorplan as _fp
        rows = _fp.tether_pairings(tethers, self.pcb_data)
        nets = {n.name: nid for nid, n in (self.pcb_data.nets or {}).items()}
        terms: List[_TetherTerm] = []
        for row in rows:
            refs = tuple(row['refs'])
            if not any(r in self.parts and not self.parts[r].locked
                       for r in refs):
                continue
            rule, name, lim = row['rule'], row['name'], float(row['limit'])
            if rule == 'decap_distance':
                terms.append(_TetherTerm(rule, name, refs, lim, 'decap', {
                    'cap': row['cap'], 'ic': row['ic'],
                    'rail': tuple(row['rail']),
                    'rail_set': frozenset(row['rail']),
                    'graded': row['graded'], 'radius': row['radius'],
                    'stranded_error': bool(row.get('stranded_error'))}))
            elif rule == 'decap_pin_distance':
                terms.append(_TetherTerm(rule, name, refs, lim, 'pin', {
                    'ic': row['ic'], 'pad_index': row['pad_index'],
                    'net_id': nets.get(row['net']),
                    'caps': tuple(row['caps']),
                    'caps_set': frozenset(row['caps'])}))
            elif row['basis'] == 'body':
                terms.append(_TetherTerm(rule, name, refs, lim, 'prox_body',
                                         {'claim': row['claim']}))
            else:
                a, b = refs
                n = len(_fp.proximity_reaches(*_fp.proximity_pads(
                    row['claim'], self._posed_fp(a), self._posed_fp(b))))
                for k in range(n):
                    terms.append(_TetherTerm(
                        rule, f"{name}#{k}" if n > 1 else name, refs, lim,
                        'prox_pad', {'claim': row['claim'], 'slot': k}))
        return terms

    def _pose_of(self, ref, override=None):
        if override and ref in override:
            return override[ref]
        p = self.parts.get(ref)
        if p is not None:
            return (p.x, p.y, p.rot)
        fp = self.pcb_data.footprints[ref]
        return (fp.x, fp.y, (fp.rotation or 0.0) % 360)

    def _posed_fp(self, ref, override=None):
        """`ref`'s footprint at its live (or overridden) pose, through
        `legality.footprint_at_pose` -- the posing `floorplan.PoseGrader`
        grades with. Cached by pose, so an incumbent is posed once."""
        pose = self._pose_of(ref, override)
        key = (ref,) + tuple(pose)
        fp = self._posed.get(key)
        if fp is None:
            if len(self._posed) > 4096:
                self._posed.clear()
            fp = legality.footprint_at_pose(self.pcb_data.footprints[ref],
                                            pose)
            self._posed[key] = fp
        return fp

    def _chip_bounds(self, ref, override=None):
        """`groups.chip_bounds_of` for `ref` at its live (or overridden)
        pose, cached by pose like `_posed_fp`."""
        pose = self._pose_of(ref, override)
        key = (ref,) + tuple(pose)
        b = self._bounds.get(key)
        if b is None:
            if len(self._bounds) > 16384:
                self._bounds.clear()
            from . import groups as _groups
            b = _groups.chip_bounds_of(self._posed_fp(ref, override))
            self._bounds[key] = b
        return b

    def _tether_value(self, i: int, override=None) -> float:
        """Term `i` measured with `override` poses over the live board.

        Every number comes from a grader function, never a copy of one. The
        only thing added here is the CACHE of a pin term's static caps: when
        the IC does not move, `nearest_rail_cap` over the caps that do not
        move either is the same answer for every candidate of the one that
        does, and the minimum of two minima is the minimum.
        """
        return self._tether_measure(self._tether_terms[i], i, override)

    def _tether_measure(self, t: _TetherTerm, i: int,
                        override=None) -> float:
        """`_tether_value`'s body, for term `t`, the gate's term `i`."""
        _fp, _groups = _FP_GROUPS or _load_fp_groups()
        # `override` is one or two refs, so membership is asked of IT, never
        # by walking a term's (long) rail.
        moving = override or {}
        if t.kind == 'decap':
            # The grade's LIVE election: the nearest chip on the cap's rail at
            # these poses, not the IC elected at state build. A cap elected
            # beyond the radius of one IC can walk into the radius of another
            # (run 32: C26, 7.20mm from U15, walked to 3.07mm from U36), and
            # a frozen pair would read that move as clean.
            cap, rail = t.data['cap'], t.data['rail']
            if cap not in moving and moving:
                # The cap stays: the chips that do not move are one fixed
                # minimum. If it is within the limit already, no chip's move
                # can take the election past it (same bound as the pin term).
                key = (i, tuple(sorted(r for r in moving
                                       if r in t.data['rail_set'])))
                static = self._tgap.get(key)
                if static is None:
                    static = _groups.elect_live(
                        self._posed_fp(cap),
                        [(r, self._chip_bounds(r)) for r in rail
                         if r not in moving])[1]
                    self._tgap[key] = static
                if static is not None and static <= t.threshold + legality.EPS:
                    return static
            if any(r in t.data['rail_set'] for r in moving):
                cands = [(r, self._chip_bounds(r, override)) for r in rail]
            else:
                # No chip on the rail moves: their live bounds are the same
                # for every candidate until the next applied move.
                cands = self._tgap.get(('rail', i))
                if cands is None:
                    cands = [(r, self._chip_bounds(r)) for r in rail]
                    self._tgap[('rail', i)] = cands
            _ic, d = _groups.elect_live(self._posed_fp(cap, override), cands)
            if d is None:
                return 0.0
            if (not (t.data['graded'] or t.data.get('stranded_error'))
                    and d > t.data['radius'] + legality.EPS):
                # Outside the radius the grade calls it `decap_ungraded`
                # (warn): not a finding this term counts. INSIDE it is graded,
                # so a pair elected beyond the radius may not walk in past
                # the limit -- that would be a new error the grade sees. A
                # pair graded at build stays measured beyond it: leaving the
                # radius is not how a cap may stop being too far. So does a
                # cap STRANDED beyond it whose `decap_ungraded` is an error
                # (#1150 c: a --decaps-from reference holds it, #1142): it
                # read 0 here, so the gate let it drift further out; the
                # monotone gate now lets it come home and nothing else.
                return 0.0
            return d
        if t.kind == 'pin':
            ic, caps = t.data['ic'], t.data['caps']
            if ic in moving:
                # The gate asks only whether some cap is within the limit, so
                # the first one that is answers it (exact past the limit, as
                # the static path below).
                pin = self._posed_fp(ic, override).pads[t.data['pad_index']]
                got = _fp.nearest_rail_cap(
                    pin, (self._posed_fp(c, override) for c in caps),
                    stop_at=t.threshold + legality.EPS)
                return got[0] if got is not None else 0.0
            live = tuple(sorted(c for c in moving if c in t.data['caps_set']))
            key = (i, live)
            static = self._tgap.get(key)
            if static is None:
                pin = self._posed_fp(ic).pads[t.data['pad_index']]
                static = _fp.nearest_rail_cap(
                    pin, [self._posed_fp(c) for c in caps if c not in moving])
                self._tgap[key] = static
            best = static[0] if static is not None else None
            if best is not None and best <= t.threshold + legality.EPS:
                # A cap that does not move already satisfies the pin, and the
                # term is a MINIMUM, so no pose of the moving cap can take it
                # past the limit. Returned without measuring the moving cap:
                # the value is then an upper bound, exact whenever it is past
                # the limit -- the only case the gate compares. (The incumbent
                # has no moving cap, so it is always exact.)
                return best
            if live:
                pin = self._posed_fp(ic).pads[t.data['pad_index']]
                got = _fp.nearest_rail_cap(
                    pin, [self._posed_fp(c, override) for c in live])
                if got is not None and (best is None or got[0] < best):
                    best = got[0]
            return best if best is not None else 0.0
        a, b = t.refs
        fa, fb = self._posed_fp(a, override), self._posed_fp(b, override)
        if t.kind == 'prox_pad':
            reaches = _fp.proximity_reaches(
                *_fp.proximity_pads(t.data['claim'], fa, fb))
            k = t.data['slot']
            return reaches[k][0] if k < len(reaches) else 0.0
        if self._tether_bodies is None:
            from .body import board_bodies
            self._tether_bodies = board_bodies(self.pcb_data, self.pcb_file)
        ra, _sa = _fp.drawn_body_rect(self._tether_bodies.get(a), fa)
        rb, _sb = _fp.drawn_body_rect(self._tether_bodies.get(b), fb)
        if ra is None or rb is None:
            return 0.0
        return rect_gap(ra, rb)

    def _incumbent_tether(self, i: int) -> float:
        v = self._inc_tval.get(i)
        if v is None:
            v = self._tether_value(i)
            self._inc_tval[i] = v
        return v

    def tether_failures(self, override) -> List[Tuple]:
        """[(rule, name, measured, incumbent)] for every term touching a ref
        in `override` that the candidate breaks: past its limit AND worse
        than the live board. Termwise, per claim -- see `tether_ok`."""
        return list(self._iter_tether_failures(override))

    def _iter_tether_failures(self, override):
        """`tether_failures`, lazily, so a yes/no caller stops at the first."""
        if len(override) == 1:
            idx = self._tethers_of.get(next(iter(override)), ())
        else:
            idx = sorted({i for r in override
                          for i in self._tethers_of.get(r, ())})
        for i in idx:
            t = self._tether_terms[i]
            c = self._tether_value(i, override)
            if c <= t.threshold + legality.EPS:
                continue
            u = self._incumbent_tether(i)
            if c > u + legality.EPS:
                yield (t.rule, t.name, round(c, 4), round(u, 4))

    def tether_ok(self, override) -> bool:
        """MONOTONE per CLAIM: every tether term touching a moving ref is
        within its limit or no worse than on the live board.

        Per term, not all-or-nothing like `intent_ok`'s vector: a term here is
        one finding `grade_delta` counts (a cap, an IC's pin, a proximity
        pad), and an IC with one pin already past its limit must still be
        allowed to move in ways that keep every OTHER pin within limit.
        `override` maps each moving ref to its candidate pose."""
        if not self._tether_active:
            return True
        return next(self._iter_tether_failures(override), None) is None

    def _pad_neighbors(self, ref):
        """Neighbor refs for the pad gate: everyone else."""
        return [o for o in self.parts if o != ref]

    def _incumbent_violation(self, ref, exclude=None):
        """The incumbent pose's violation, cached per (ref, exclude).

        Keyed on `exclude` too, so a caller that always passes one (the
        unplaced pile) still hits the cache: each miss walks the neighbours
        and the outline. The key is a frozenset, so it costs O(|exclude|) hashing against a
        full neighbour-and-outline sweep -- cheap by a wide margin. The whole
        cache is cleared on every move (see apply_move), which is what makes
        an incumbent answer safe to hold at all.

        The incumbent is the part's current pose unless the caller pinned it
        in `incumbent_at` (#1259, `_incumbent_pose`).
        """
        key = (ref, frozenset(exclude) if exclude else None)
        v = self._inc_violation.get(key)
        if v is None:
            at = self.incumbent_at.get(ref)
            v = (self.violation_parts(ref, exclude=exclude) if at is None
                 else self.violation_parts(ref, *at, exclude=exclude))
            self._inc_violation[key] = v
        return v

    def escape_board_term(self, ref, exclude=None) -> Optional[float]:
        """The incumbent's board term when `candidate_valid`'s off-board
        escape branch is OPEN for `ref`, else None (#1277).

        Open means the incumbent (`_incumbent_pose`) is off the board without
        overlapping anything, and no caller has closed the branch
        (`escape_closed`). Only then can the branch admit a pose the ordinary
        gate refuses: a pose with a smaller board term than this value. It is
        the branch's own test, so a caller asking "is my part's input pose
        off the board" gets the answer the ranking will act on."""
        if ref in self.escape_closed:
            return None
        cur_board, cur_overlap = self._incumbent_violation(ref,
                                                           exclude=exclude)
        if cur_overlap > EPS_IMPROVE or cur_board <= EPS_IMPROVE:
            return None
        return cur_board

    def _incumbent_pose(self, ref) -> Tuple[float, float, float]:
        """#1259: the pose `candidate_valid`'s escape branch measures as the
        incumbent -- `incumbent_at[ref]` when a caller set it, else the
        part's current pose."""
        at = self.incumbent_at.get(ref)
        if at is not None:
            return at
        part = self.parts[ref]
        return (part.x, part.y, part.rot)

    def _pad_copper_past(self, ref, x, y, rot) -> float:
        """#1257: how far `ref`'s pad copper reaches past the outline at the
        pose by check_assembly's gate, for a part whose pads reach past its
        courtyard by more than `edge_gate.margin`; 0.0 for every other part,
        whose copper a courtyard held inside at that margin keeps on the
        board, after one cached compare. `seeder.pose_ok`'s conjunct, through
        the seeder's own two functions (`_pad_copper_overhang`,
        `_pad_copper_off_board`) rather than a copy."""
        sd = _seeder()
        if sd._pad_copper_overhang(self, ref) <= self.edge_gate.margin:
            return 0.0
        return sd._pad_copper_off_board(self, ref, x, y, rot)

    def _graphic_copper_past(self, ref, x, y, rot) -> float:
        """#962: `_pad_copper_past` for the copper the footprint draws: how
        far it reaches past the outline at the pose by the grade's measure,
        for a part whose drawn copper reaches past its courtyard by more than
        `edge_gate.margin`; 0.0 for every other part, after one cached
        compare. Through the seeder's own two functions
        (`_graphic_copper_overhang`, `_graphic_copper_off_board`)."""
        sd = _seeder()
        if sd._graphic_copper_overhang(self, ref) <= self.edge_gate.margin:
            return 0.0
        return sd._graphic_copper_off_board(self, ref, x, y, rot)

    def _edges_near(self, ref) -> list:
        """Cached: the Edge.Cuts edges this part can touch -- every edge for
        a movable part (its travel is unbounded), only those at its seed pose
        for a locked one. Empty means the exact ring test is skippable for
        every pose it can take."""
        part = self.parts[ref]
        travel = 0.0 if part.locked else float('inf')
        # center= the pose ORIGIN: a part ROTATES about it, so an off-centre
        # courtyard's rect swings and the seed-rect-centred disk can miss an edge.
        return self.edge_gate.edges_near(
            ref, part.rect(part.seed_x, part.seed_y, part.orig_rot), travel,
            center=(part.seed_x, part.seed_y))

    def _owned_rings(self, ref) -> frozenset:
        """Cached: the milled rings this part's OWN pads sit inside (#628).

        A milled contour is reclassified out of board_cutouts precisely BECAUSE
        it encloses >= 2 pad centres, so such a ring always has a part living on
        it -- a connector over its own milled relief. Without this exemption the
        swallow probe judges that part board-violating at its own hand-placed
        pose, and because a genuinely sub-clearance edge pose then scores LOWER,
        the unfreeze branch below walks it off the board edge.

        Keyed on the SEED pose and never invalidated, like _edges_near: seed
        ownership is what the reclassification itself was computed from, and it
        is the anti-gaming choice -- ownership taken at the CANDIDATE pose would
        let any part claim a ring merely by moving onto it.
        """
        owned = self._owned_rings_cache.get(ref)
        if owned is None:
            part = self.parts[ref]
            pts = [(gx, gy) for (gx, gy, _net) in
                   part.pad_globals(part.seed_x, part.seed_y, part.orig_rot)]
            owned = self.edge_gate.rings_enclosing(pts) if pts else frozenset()
            self._owned_rings_cache[ref] = owned
        return owned

    # ----- bodies and reporting ---------------------------------------------

    def fab_rect(self, ref, x=None, y=None, rot=None):
        """The part's .Fab BODY rect at a pose, or None when its footprint
        draws no .Fab geometry.

        The body currency for CONTAINMENT tests. It must never fall back to
        the courtyard: a courtyard is body + margin + shell-overhang volume,
        and the 33-board corpus ships frac-1.0 COURTYARD containment on four
        healthy boards (esp_prog, orangecrab_ext_pll,
        rp2350_fpga_eensy_prePlane, ulx3s) against ZERO non-exempt fab
        containment. A courtyard-based containment test is a false-veto
        machine; this one is not.

        None means UNJUDGED, not clear -- a pose inside a bodyless part
        cannot be refused, and callers disclose that rather than assuming
        coverage they do not have.

        Lazy: nothing is parsed until the first call, so a caller that never
        asks pays nothing.
        """
        if self._fab_local is None:
            try:
                from placement.parser import extract_fab_sides
                self._fab_local = extract_fab_sides(self.pcb_file) or {}
            except Exception:
                self._fab_local = {}
        p = self.parts.get(ref)
        if p is None or ref in self._fab_rejected_refs():
            return None
        sides = self._fab_local.get(ref)
        if not sides:
            return None
        x = p.x if x is None else x
        y = p.y if y is None else y
        rot = p.rot if rot is None else rot
        key = (ref, rot % 360)
        local = self._fab_cache.get(key)
        if local is None:
            own = self._fab_side(p)
            lb = sides.get(own) or next(iter(sides.values()))
            local = rotate_local_bounds(*lb, rot)
            self._fab_cache[key] = local
        return (x + local[0], y + local[1], x + local[2], y + local[3])

    def _board_bodies(self):
        """`body.board_bodies` for this state's board, read once (or None
        when it cannot be read)."""
        got = getattr(self, '_board_bodies_cache', False)
        if got is False:
            try:
                from placement.body import board_bodies
                got = board_bodies(self.pcb_data, self.pcb_file)
            except Exception:                                # noqa: BLE001
                got = None
            self._board_bodies_cache = got
        return got

    def _fab_rejected_refs(self):
        """Parts whose .Fab drawing `body.fab_cannot_be_body` rejects
        (#1201): `fab_rect` / `fab_shape` answer None for them, so the
        quench's body vetoes read the checker's body and never a pin-1 dot.
        Their own .Fab reader is kept, rather than replaced by the ladder's
        `drawn_local`, so every other part reads exactly what it did."""
        got = getattr(self, '_fab_rejected_cache', None)
        if got is None:
            bodies = self._board_bodies() or {}
            got = frozenset(r for r, g in bodies.items()
                            if getattr(g, 'fab_rejected', False))
            self._fab_rejected_cache = got
        return got

    @staticmethod
    def _fab_side(p):
        """The side whose .Fab drawing `fab_rect` and `fab_shape` read:
        the part's already-resolved `side`, upper-cased defensively."""
        return 'B' if str(getattr(p, 'side', 'F')).upper().startswith('B') \
            else 'F'

    def fab_shape(self, ref, x=None, y=None, rot=None):
        """The part's DRAWN .Fab body at a pose (board-frame geometry), on the
        side `fab_rect` picks, or None when its footprint draws no .Fab.
        Lazy, like `fab_rect`: nothing is read until the first call."""
        if getattr(self, '_fab_shapes', None) is None:
            try:
                from placement.parser import extract_fab_shapes
                self._fab_shapes = extract_fab_shapes(self.pcb_file) or {}
            except Exception:                                # noqa: BLE001
                self._fab_shapes = {}
        p = self.parts.get(ref)
        sides = self._fab_shapes.get(ref)
        if p is None or not sides or ref in self._fab_rejected_refs():
            return None
        own = self._fab_side(p)
        got = sides.get(own) or next(iter(sides.values()))
        return legality.place_local_shape(
            got[0], p.x if x is None else x, p.y if y is None else y,
            p.rot if rot is None else rot)

    def _body_overlap_at(self, ref, x, y, rot, exclude=None):
        """Would this pose put any of `ref`'s .Fab body over a same-side
        neighbour's? `fab_rect` is the broad phase and the drawn bodies
        decide, as check_assembly's fab channel measures them (#1094), so a
        diagonal part is not refused on its box. Marker and container parts
        are exempt (`body_exempt_refs`); a part with no .Fab is unjudged.

        For a board whose project waives the courtyard rule (#1101), where
        the courtyard no longer keeps bodies apart."""
        exempt = self.body_exempt_refs()
        if ref in exempt:
            return False
        ra = self.fab_rect(ref, x, y, rot)
        if ra is None:
            return False
        part = self.parts[ref]
        others = list(self.parts.items())
        mine = None
        for other_ref, other in others:
            if other_ref == ref or (exclude and other_ref in exclude):
                continue
            if other_ref in exempt or other.side != part.side:
                continue
            rb = self.fab_rect(other_ref)
            if rb is None or rect_overlap_area(ra, rb) <= _BODY_OVERLAP_EPS:
                continue
            if mine is None:
                mine = self.fab_shape(ref, x, y, rot)
            theirs = self.fab_shape(other_ref)
            if mine is None or theirs is None:
                if self._why is not None:
                    self._veto('body_overlap', other_ref)
                return True            # unmeasurable: the rects overlap
            if mine.intersection(theirs).area > _BODY_OVERLAP_EPS:
                if self._why is not None:
                    self._veto('body_overlap', other_ref)
                return True
        return False

    def _bodyless_refs(self):
        """Pad-bearing parts that draw NO body (#1106), cached -- the
        checker's own set: `grade_body_overlap` judges a part by its drawn
        body (.Fab, else a usable silk outline, `body.board_bodies`) and
        asks the pads-under-body question only of a part with neither. A
        part with a silk body but no .Fab is NOT body-less here: reading
        .Fab alone put esp_prog's silk-only U2 in this set and refused quench
        moves the checker allows (8 esp_prog poses moved, review)."""
        got = getattr(self, '_bodyless_cache', None)
        if got is None:
            fps = getattr(self.pcb_data, 'footprints', {}) or {}
            bodies = self._board_bodies()
            out = set()
            for r in self.parts:
                fp = fps.get(r)
                if fp is None or not any(
                        _pad_has_copper(p) for p in fp.pads or ()):
                    continue
                if bodies is None:
                    drawn = self.fab_rect(r)
                else:
                    g = bodies.get(r)
                    drawn = g.drawn_local if g is not None else None
                if drawn is None:
                    out.add(r)
            got = frozenset(out)
            self._bodyless_cache = got
        return got

    def _pads_under_body_at(self, ref, x, y, rot, exclude=None):
        """Would `ref` at this pose put a BODY-LESS part's pad copper under a
        same-face drawn body (#1106)? Either direction of the move: `ref`
        body-less landing under a neighbour's body, or `ref`'s body landing
        over a body-less neighbour. `legality.pads_under_body_frac` at
        `CONTAINMENT_FRAC`, the checker's predicate; marker and container
        parts are exempt, as in the checker's gate. The body-less set is the
        checker's own (`_bodyless_refs`); the covering body is read from .Fab
        only, which is still the checker's gate, since a pair whose body
        came from silk never gates. Rect broad phase first, so a board with
        no body-less part pays a set lookup."""
        bodyless = self._bodyless_refs()
        if not bodyless:
            return False
        exempt = self.body_exempt_refs()
        if ref in exempt:
            return False
        fps = getattr(self.pcb_data, 'footprints', {}) or {}
        part = self.parts[ref]
        if ref in bodyless:
            box = part.padbox(x, y, rot)
            if box is None:
                return False
            mine_pads = None
            for other_ref, other in self.parts.items():
                if (other_ref == ref or other_ref in bodyless
                        or (exclude and other_ref in exclude)
                        or other_ref in exempt
                        or not (part.sides & other.sides)):
                    continue
                orect = self.fab_rect(other_ref)
                if orect is None or rect_gap(box, orect) > 0.0:
                    continue
                theirs = self.fab_shape(other_ref)
                if theirs is None:
                    continue
                if mine_pads is None:
                    mine_pads = legality.bodyless_pad_shape(fps[ref], x, y,
                                                            rot)
                if (legality.pads_under_body_frac(mine_pads, theirs)
                        >= legality.CONTAINMENT_FRAC):
                    if self._why is not None:
                        self._veto('pads_under_body', other_ref)
                    return True
            return False
        mrect = self.fab_rect(ref, x, y, rot)
        if mrect is None:
            return False
        mine_body = None
        for other_ref in bodyless:
            if (other_ref == ref or (exclude and other_ref in exclude)
                    or other_ref in exempt):
                continue
            other = self.parts[other_ref]
            if not (part.sides & other.sides):
                continue
            ob = other.padbox()
            if ob is None or rect_gap(mrect, ob) > 0.0:
                continue
            if mine_body is None:
                mine_body = self.fab_shape(ref, x, y, rot)
                if mine_body is None:
                    return False
            theirs_pads = legality.bodyless_pad_shape(
                fps[other_ref], other.x, other.y, other.rot)
            if (legality.pads_under_body_frac(theirs_pads, mine_body)
                    >= legality.CONTAINMENT_FRAC):
                if self._why is not None:
                    self._veto('pads_under_body', other_ref)
                return True
        return False

    def body_exempt_refs(self):
        """Refs whose body may legitimately swallow or be swallowed.

        MARKER (mount_hole/fiducial/testpoint) and CONTAINER only -- the same
        set reconstruct._body_exempt_refs builds, and deliberately NOT the
        edge classes. Measured: orangecrab ships FID2 wholly inside J5 at frac
        1.000 and FID1 inside J4 at 0.867, so without the marker exemption a
        displaced fiducial could never come home under a connector.
        """
        cached = getattr(self, '_body_exempt', None)
        if cached is not None:
            return cached
        exempt = set(getattr(self, 'container_refs', ()) or ())
        try:
            from placement.part_class import classify_part
            fps = getattr(getattr(self, 'pcb_data', None), 'footprints', {})
            for ref in self.parts:
                fp = (fps or {}).get(ref)
                if fp is None:
                    continue
                try:
                    if classify_part(fp, ref).name in ('mount_hole', 'fiducial',
                                                       'testpoint'):
                        exempt.add(ref)
                except Exception:
                    continue
        except Exception:
            pass
        self._body_exempt = exempt
        return exempt

    def _body_contained_at(self, ref, x, y, rot, exclude=None):
        """Would this pose put `ref`'s body inside a neighbour's, or vice
        versa? Fab currency; `None` from fab_rect means UNJUDGED, never clear.
        """
        if not _CONTAINMENT_GATE:
            return False
        exempt = self.body_exempt_refs()
        if ref in exempt:
            return False
        ra = self.fab_rect(ref, x, y, rot)
        if ra is None:
            return False
        part = self.parts[ref]
        others = list(self.parts.items())
        for other_ref, other in others:
            if other_ref == ref or (exclude and other_ref in exclude):
                continue
            if other_ref in exempt or other.side != part.side:
                continue
            rb = self.fab_rect(other_ref)
            if rb is None:
                continue
            area = rect_overlap_area(ra, rb)
            if area <= 1e-9:
                continue
            frac = containment_frac(area, ra, rb)
            if frac is not None and frac >= CONTAINMENT_FRAC:
                if self._why is not None:
                    self._veto('body_contained', other_ref)
                return True
        return False

    def wire_metrics(self):
        """`{'length', 'crossings', 'hpwl'}` of the current placement: the
        airwire MST length (mm), the crossing count among those airwires,
        and `hpwl`. What render_placement's metrics strip and its JSON
        report; nothing optimises on them here."""
        all_aw = _aw_array([aw for lst in self.net_airwires.values() for aw in lst])
        length = (float(np.sum(np.hypot(all_aw[:, 2] - all_aw[:, 0],
                                        all_aw[:, 3] - all_aw[:, 1])))
                  if len(all_aw) else 0.0)
        crossings, _w = _count_crossings_within(all_aw)
        return {'length': length, 'crossings': crossings, 'hpwl': self.hpwl()}

    def hpwl(self):
        """Half-perimeter wirelength: sum over nets of the pad bbox's width plus
        height (mm). The classic placement-quality proxy, and one of the columns
        a placement scorecard wants (#411).

        Its value here is that it is airwire-ORDER-INVARIANT by construction: it
        reads only the extremes of each net's pad positions, so unlike the MST
        length and the crossing count it cannot move when a tie-break resolves
        differently. That makes it the witness for #457 -- two runs whose HPWL
        agrees but whose crossing count does not differ in tie-breaks, not in
        placement quality. (After the sorted-net_refs fix all three agree; HPWL
        is what tells you WHICH kind of difference you are looking at if one
        ever reappears.)
        """
        total = 0.0
        for net_id, refs in self.net_refs.items():
            xs, ys = [], []
            for ref in refs:
                for gx, gy, n in self.parts[ref].pad_globals():
                    if n == net_id:
                        xs.append(gx)
                        ys.append(gy)
            if len(xs) > 1:
                total += (max(xs) - min(xs)) + (max(ys) - min(ys))
        return total

    def graded_parts(self):
        """The placement as `legality.GradedPart` records, for the graders.

        Lets a scorecard compute OO (overlap area) and OoB from the same
        geometry and the same side rules the gate uses, rather than
        re-deriving courtyards and disagreeing about what legal means (#456).
        """
        return [legality.GradedPart(ref=ref, side=p.side, rect=p.rect(),
                                    tht_rect=p.tht_rect(), has_tht=p.has_tht)
                for ref, p in self.parts.items()]

    def legality_metrics(self):
        """{'overlap_area', 'oob_count', 'oob_amount', 'oob_area', 'hpwl'} for
        the current placement. Zero across the legality keys means fully legal;
        `hpwl` is a quality number, not a legality one, and is included because a
        scorecard wants both from one call (#411)."""
        parts = self.graded_parts()
        oob_count = 0
        oob_amount = 0.0
        oob_area = 0.0
        for p in parts:
            # skip_rings: a part over its own milled relief is legal there, and
            # this is the #456 scorecard -- without it a correct hand placement
            # reports oob_count 1 (#628).
            amt = self.edge_gate.rect_outside_amount(
                p.rect, skip_rings=self._owned_rings(p.ref))
            if amt > legality.EPS:
                oob_count += 1
                oob_amount += amt
                oob_area += self.edge_gate.out_of_board_area(p.rect)
        # A container's RECT is not a body (fa10 P1): the grader dropped a
        # pin frame's rect pairs and waives an outline's, so this number
        # leaves them out too -- rp2350's was mostly U8's frame.
        _cont = getattr(self, 'container_refs', ()) or ()
        overlap = legality.placement_overlap_area(
            [g for g in parts if g.ref not in _cont])
        out = {'overlap_area': overlap,
               'oob_count': oob_count, 'oob_amount': oob_amount,
               'oob_area': oob_area, 'hpwl': self.hpwl()}
        if getattr(self, 'courtyards_ignored', False):
            # The project waives courtyard overlap
            # (`courtyards_ignored`), so it is not a legality cost here; the
            # measurement is kept, under its own name, for disclosure.
            out['overlap_area'] = 0.0
            out['overlap_area_waived'] = overlap
        out.update(self.pad_legality_metrics())
        return out

    def pad_legality_metrics(self):
        """AABB-currency pad/hole tallies of the CURRENT placement (the gate
        currency -- conservative; the CLIs re-grade the written file with
        exact geometry via legality.grade_pad_legality). Empty when the pad
        layer is off."""
        if self.legality_ctx is None:
            return {}
        refs = sorted(self.parts)
        pairs = 0
        short = 0.0
        overlaps = 0
        holes = 0.0
        stacks = 0
        locked_contacts = 0
        for i, a in enumerate(refs):
            a_locked = self.parts[a].locked
            for b in refs[i + 1:]:
                sf = self.legality_ctx.pair_shortfall(a, b)
                if sf.pad > legality.EPS:
                    pairs += 1
                    short += sf.pad
                if sf.pad_overlap:
                    overlaps += 1
                if sf.stack:
                    stacks += 1
                holes += sf.hole
                # E6, riding along in the loop that is already walking every
                # pair: copper touching a part KiCad marks (locked yes). A
                # locked pose is a decision made outside this toolchain, so a
                # search may not settle the contact by moving the other part.
                if (a_locked or self.parts[b].locked) and (
                        sf.stack or sf.hole > legality.EPS
                        or sf.pad > legality.EPS):
                    locked_contacts += 1
        # #1031: parts with an ILLEGAL pad in a rule-area keep-out band, at
        # the current poses (0 on a board without such a keep-out).
        ko_parts = 0
        ko_amount = 0.0
        if self.legality_ctx.keepouts is not None:
            for r in refs:
                amt = self.legality_ctx.keepout_amount(r, *self.legality_ctx.pose_of(r))
                if amt > legality.EPS:
                    ko_parts += 1
                    ko_amount += amt
        return {'pad_conflict_pairs': pairs,
                'keepout_pad_parts': ko_parts,
                'keepout_pad_amount': round(ko_amount, 4),
                'pad_shortfall': round(short, 4),
                'pad_overlap_pairs': overlaps,
                # run-6: ANY-net cross-footprint pad intersections -- the
                # assembly (stacked-parts) channel, corpus-calibrated 0 on
                # every healthy board in both exact and AABB currencies
                'pad_intersection_pairs': stacks,
                'locked_contact_pairs': locked_contacts,
                'hole_shortfall': round(holes, 4)}

    # ----- move application -------------------------------------------------

    def apply_move(self, ref, x, y, rot):
        part = self.parts[ref]
        part.x, part.y, part.rot = x, y, rot
        self._inc_violation.clear()
        self._inc_intent.clear()
        for net_id in part.nets:
            self.net_airwires[net_id] = self._build_net_airwires(net_id)

class TetherGateView:
    """QuenchState's #1043 tether gate on parts that are not a QuenchState
    (#1067: `place_fanout_clearance`'s near-BGA caps).

    The methods ARE QuenchState's -- bound here as class attributes, not
    copied -- so a candidate is judged by the same measurement and the same
    per-claim rule everywhere: past its limit AND worse than the live board
    refuses (`tether_failures` / `tether_ok`). `parts` maps each MOVABLE
    ref to an object with `x`, `y`, `rot` and `locked`; every other part is
    read at its file pose, which is right for an engine that moves only those
    parts. `note_move()` must follow every applied move: it clears the
    incumbent values and the static partial minima.
    """

    _build_tethers = QuenchState._build_tethers
    tether_terms_for = QuenchState.tether_terms_for
    _pose_of = QuenchState._pose_of
    _posed_fp = QuenchState._posed_fp
    _chip_bounds = QuenchState._chip_bounds
    _tether_value = QuenchState._tether_value
    _tether_measure = QuenchState._tether_measure
    _incumbent_tether = QuenchState._incumbent_tether
    tether_failures = QuenchState.tether_failures
    _iter_tether_failures = QuenchState._iter_tether_failures
    tether_ok = QuenchState.tether_ok

    def __init__(self, pcb_data, pcb_file, parts, tethers):
        self.pcb_data = pcb_data
        self.pcb_file = pcb_file
        self.parts = parts
        self._tether_terms = []
        self._tethers_of = {}
        self._inc_tval = {}
        self._tgap = {}
        self._posed = {}
        self._bounds = {}
        self._tether_bodies = None
        self.tethers = dict(tethers or {})
        if self.tethers:
            self._build_tethers()
        self._tether_active = bool(self._tether_terms)

    def note_move(self):
        """A part in `parts` moved: the incumbent values and the static
        partial minima are stale."""
        self._inc_tval.clear()
        self._tgap.clear()


_FP_GROUPS = None


def _load_fp_groups():
    """(floorplan, groups), imported once: the tether gate measures through
    them on every candidate, and an import statement there is a lookup per
    call."""
    global _FP_GROUPS
    from . import floorplan, groups
    _FP_GROUPS = (floorplan, groups)
    return _FP_GROUPS


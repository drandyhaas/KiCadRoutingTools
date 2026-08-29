"""
Greedy quench placement optimizer: perturbative refinement of an existing
(hand- or AI-made) placement to improve routability.

Starts from the current placement and repeatedly tries, for each component,
small moves within --max-displacement of its seed position plus 90-degree
rotations and same-footprint swaps, accepting only improvements
(zero-temperature anneal). Same-footprint swaps are accepted only if both
parts land within swap_max_displacement (default: max_displacement) of
their own seed positions. Locked components never move.

Cost = total airwire length
     + crossing_penalty * airwire crossings
     + halo penalty (soft whitespace around parts, scaled by pin count)
     + edge penalty (soft margin inside the board edge)

Legality (hard constraints, and the whole of what candidate_valid decides) lives
in placement/legality.py, shared with the fanout-clearance repair pass: parts
only collide with parts that share a board SIDE (a back-side decap under a
front-side BGA is not an overlap; a through-hole part's lead field does reach the
far side), and board containment measures against the real Edge.Cuts outline, not
its bounding box. A part whose SEED pose is already illegal is not frozen: it may
take any candidate that strictly reduces its violation.

The halo term spreads apart parts that are not pulled together by shared
nets — things that *can* be far apart may as well be, to leave routing room,
especially around high-pin-count parts.

Both airwire terms accept optional per-net weights (`net_weights`): a net's
airwire length is scaled by its weight, and each crossing is priced by the
larger of the two crossing nets' weights, so place_route_loop.py can bias the
whole objective toward the nets the router failed on. An unweighted board is
untouched, since max(1, 1) = 1.
"""
from __future__ import annotations

import os
import math
from typing import Dict, List, NamedTuple, Sequence, Tuple, Set, Optional

import numpy as np

from kicad_parser import PCBData, local_to_global
from connectivity import compute_mst_edges
from placement.parser import (courtyard_for_side, extract_courtyard_sides,
                              extract_locked_refs, warn_missing_courtyards)
from placement.utility import compute_footprint_bbox_local, snap_to_grid
from placement import legality
from placement.legality import (CONTAINER_RATIO, CONTAINMENT_FRAC,
                                BoardOutlineGate, containment_frac,
                                footprint_has_through_pads,
                                footprint_side, pair_min_gap, rect_gap,
                                rect_overlap_area,
                                rotate_local_bounds, sides_occupied)

ROTATIONS = [0.0, 90.0, 180.0, 270.0]

#: Kill switch for the body-containment conjunct in `candidate_valid`. It is a
#: HARD gate on real boards, so a way to isolate it without a git stash is
#: worth the one line -- `KRT_NO_CONTAINMENT_GATE=1` restores the pre-2026-08
#: behaviour exactly. Not a supported flag; a debugging lever.
_CONTAINMENT_GATE = os.environ.get('KRT_NO_CONTAINMENT_GATE', '') != '1'
EPS_IMPROVE = 1e-6

# Both helpers now live in placement/legality.py, the single home shared with
# fanout_clearance (which carried byte-identical copies). Kept as module-level
# aliases: they are part of this module's de-facto surface (tests import them).
_rotate_local_bounds = rotate_local_bounds
_rect_gap = rect_gap

# Two courtyard boxes are the "same shape" for swap purposes. 1nm: far below any
# real courtyard difference (KiCad writes 6 decimals of mm), far above the float
# wobble that made two instances of one library footprint compare unequal.
_BOUNDS_EPS = 1e-6


def _bounds_match(a, b):
    return all(abs(x - y) <= _BOUNDS_EPS for x, y in zip(a, b))


def _airwires_for_points(points: List[Tuple[float, float]], net_id: int):
    """MST airwires for one net: list of (x1, y1, x2, y2, net_id)."""
    if len(points) < 2:
        return []
    if len(points) == 2:
        (x1, y1), (x2, y2) = points
        return [(x1, y1, x2, y2, net_id)]
    edges = compute_mst_edges(points, use_manhattan=False)
    return [(points[i][0], points[i][1], points[j][0], points[j][1], net_id)
            for i, j, _ in edges]


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
    net_w is a net-id-indexed weight lookup (QuenchState._net_w). With net_w
    None the two are equal and `weighted` is exactly float(count), so
    unweighted callers get bit-identical costs.
    """
    if len(a) == 0 or len(b) == 0:
        return 0, 0.0
    eps = 0.001
    a1x = a[:, 0][:, None]; a1y = a[:, 1][:, None]
    a2x = a[:, 2][:, None]; a2y = a[:, 3][:, None]
    b1x = b[:, 0][None, :]; b1y = b[:, 1][None, :]
    b2x = b[:, 2][None, :]; b2y = b[:, 3][None, :]

    same_net = a[:, 4][:, None] == b[:, 4][None, :]
    shared = (
        ((np.abs(a1x - b1x) < eps) & (np.abs(a1y - b1y) < eps)) |
        ((np.abs(a1x - b2x) < eps) & (np.abs(a1y - b2y) < eps)) |
        ((np.abs(a2x - b1x) < eps) & (np.abs(a2y - b1y) < eps)) |
        ((np.abs(a2x - b2x) < eps) & (np.abs(a2y - b2y) < eps))
    )

    def ccw(ax, ay, bx, by, cx, cy):
        return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax)

    inter = (
        (ccw(a1x, a1y, b1x, b1y, b2x, b2y) != ccw(a2x, a2y, b1x, b1y, b2x, b2y)) &
        (ccw(a1x, a1y, a2x, a2y, b1x, b1y) != ccw(a1x, a1y, a2x, a2y, b2x, b2y))
    )
    hits = inter & ~same_net & ~shared
    count = int(np.count_nonzero(hits))
    if net_w is None:
        return count, float(count)
    # Column 4 carries the net id as a float; every id that can appear there
    # is a key of QuenchState.net_airwires, which net_w is sized to cover.
    wa = net_w[a[:, 4].astype(np.intp)]
    wb = net_w[b[:, 4].astype(np.intp)]
    return count, float(np.sum(np.maximum(wa[:, None], wb[None, :])[hits]))


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
    x1, y1, x2, y2 = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    nets = a[:, 4].astype(np.intp)
    seg_len = np.hypot(x2 - x1, y2 - y1)
    total = 0.0
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
        total += float(np.sum(frac * seg_len[live]))
    return total


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


def _through_pad_bounds_local(fp):
    """Local bbox over a footprint's DRILLED pads, or None if it has none.

    This is the footprint's footprint on the OPPOSITE board side: its body and
    courtyard live on its own side, but its leads pass through, so a part on the
    far side may not sit inside this box (#456 item 1). Deliberately the drill
    hole's own extent rather than the pad copper's -- the far side sees the
    barrel and the lead, and the annular ring on that side is part of it.
    """
    # `local_x/local_y` is the pad ANCHOR in the footprint's frame, and for a
    # drilled pad the anchor IS the hole: kicad_parser records hole_x/hole_y as
    # the pre-offset position and only then shifts global_x/global_y to the
    # copper centre. So the hole needs no offset correction at all -- an earlier
    # version "corrected" local_* by (hole - global), which is minus the offset,
    # landing the box one full offset on the WRONG side of the hole.
    #
    # size_x/size_y are BOARD-axis resolved (kicad_parser swaps them for a pad at
    # ~90 degrees), so they cannot be used as local half-extents directly -- that
    # transposed the box on every 90/270-degree footprint (kit-dev SW_ONOFF201
    # modelled 2.54 x 13.97 where the truth is 3.81 x 12.70, under-blocking
    # 1.27mm). Project them through the pad's local tilt exactly as
    # placement/utility.compute_footprint_bbox_local does.
    xs, ys = [], []
    for p in (fp.pads or []):
        d = getattr(p, 'drill', 0) or 0
        if d <= 0:
            continue
        local_tilt = math.radians((getattr(p, 'rect_rotation', 0.0) or 0.0)
                                  + (fp.rotation or 0.0))
        c, s = abs(math.cos(local_tilt)), abs(math.sin(local_tilt))
        hx, hy = (getattr(p, 'size_x', 0) or 0) / 2.0, (getattr(p, 'size_y', 0) or 0) / 2.0
        # An oval/slotted hole is bounded by the pad extent it sits in; taking
        # the larger of drill radius and half pad size keeps a slot covered.
        r = d / 2.0
        rx = max(r, hx * c + hy * s)
        ry = max(r, hx * s + hy * c)
        xs += [p.local_x - rx, p.local_x + rx]
        ys += [p.local_y - ry, p.local_y + ry]
    if not xs:
        return None
    return (min(xs), min(ys), max(xs), max(ys))


class _Part:
    __slots__ = ('ref', 'pads_local', 'pin_count', 'bounds_by_rot',
                 'seed_x', 'seed_y', 'x', 'y', 'rot', 'locked',
                 'nets', 'halo', 'footprint_name', 'orig_rot',
                 'side', 'has_tht', 'sides', 'tht_by_rot')

    def __init__(self, ref, fp, courtyard_sides, locked, halo_base, halo_coef):
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
        lb = courtyard_for_side(courtyard_sides.get(ref), self.side)
        if lb is None:
            lb = compute_footprint_bbox_local(fp)
        self.bounds_by_rot = {r: _rotate_local_bounds(*lb, r) for r in ROTATIONS}
        tlb = _through_pad_bounds_local(fp) if self.has_tht else None
        self.tht_by_rot = ({r: _rotate_local_bounds(*tlb, r) for r in ROTATIONS}
                           if tlb is not None else None)
        # A non-90-degree seed rotation brings its WHOLE 90-degree lattice:
        # those are the poses _candidate_rotations offers such a part, and
        # build_neighbor_lists unions bounds_by_rot over the movable
        # same-footprint group, so recording them here is what keeps the
        # pruning boxes covering every pose the group can reach.
        base = fp.rotation % 90
        if base:
            for r in ROTATIONS:
                rot = (base + r) % 360
                self.bounds_by_rot[rot] = _rotate_local_bounds(*lb, rot)
                if self.tht_by_rot is not None:
                    self.tht_by_rot[rot] = _rotate_local_bounds(*tlb, rot)
        self.seed_x, self.seed_y = fp.x, fp.y
        self.x, self.y, self.rot = fp.x, fp.y, fp.rotation % 360
        self.orig_rot = fp.rotation % 360
        self.locked = locked
        self.nets = sorted({n for _, _, n in self.pads_local})
        self.halo = halo_base + halo_coef * math.sqrt(max(self.pin_count, 1))

    def rect(self, x=None, y=None, rot=None):
        x = self.x if x is None else x
        y = self.y if y is None else y
        rot = self.rot if rot is None else rot
        b = self.bounds_by_rot.get(rot % 360)
        if b is None:
            b = self.bounds_by_rot[0.0]
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
        return (x + b[0], y + b[1], x + b[2], y + b[3])

    def rects(self, x=None, y=None, rot=None):
        """(courtyard rect, far-side rect) at a pose -- what a pair test needs.

        The far-side rect is None for the overwhelming majority of parts (no
        drilled pads), and the pair tests fast-path on exactly that, so this
        stays a single rect() call for them.
        """
        if self.tht_by_rot is None:
            return self.rect(x, y, rot), None
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

    def pad_globals(self, x=None, y=None, rot=None):
        x = self.x if x is None else x
        y = self.y if y is None else y
        rot = self.rot if rot is None else rot
        return [(*local_to_global(x, y, rot, lx, ly), n)
                for lx, ly, n in self.pads_local]


class QuenchState:
    """Current placement plus cached airwires and cost terms."""

    def __init__(self, pcb_data: PCBData, pcb_file: str,
                 clearance: float, board_edge_clearance: float,
                 crossing_penalty: float,
                 halo_base: float, halo_coef: float, halo_weight: float,
                 edge_halo: float, edge_weight: float,
                 grid_step: float, length_weight: float = 1.0,
                 ignore_net_ids: Optional[Set[int]] = None,
                 extra_locked_refs: Optional[Set[str]] = None,
                 move_refs: Optional[Set[str]] = None,
                 net_weights: Optional[Dict[int, float]] = None,
                 # --- #548, APPENDED after net_weights on purpose. Three test
                 # files bind this constructor with TWELVE positional arguments
                 # (test_458_quench_net_weights.py:131,
                 # test_458_quench_rotations.py:252 and :284); inserting
                 # anywhere earlier rebinds them silently.
                 align_weight: float = 0.0,
                 align_radius: float = 0.5,
                 align_span: float = 20.0,
                 orient_weight: float = 0.0,
                 # --- pad+drill legality layer, APPENDED for the same
                 # positional-binding reason as the #548 block above.
                 pad_legality: bool = True,
                 # True HERE (no freeze): the zero-net freeze is an OPTIMIZER
                 # policy, applied by quench() -- the seeder places mounting
                 # holes from an intent and must not find them pre-locked
                 # (measured: the freeze-in-state broke test_place_seed's
                 # declared-edge H5 seat).
                 move_unconnected: bool = True,
                 # --- corridor cut. Appended for the same reason as #548's
                 # four, and OFF by default: at weight 0.0 no corridor is ever
                 # built and the objective is bit-identical.
                 corridor_weight: float = 0.0,
                 corridor_specs: Optional[Sequence[Dict]] = None,
                 corridor_max_fanout: int = 20,
                 # --- #701 intent keep-outs. Appended for the same
                 # positional-binding reason as the #548 block above, and
                 # empty by default so every state in the tree that does not
                 # ask for them -- INCLUDING the one `floorplan.grade` builds,
                 # which must keep measuring independently of the seat gate --
                 # is bit-identical. Consumed by `seeder.pose_ok` and
                 # `seeder.edge_seat_ok`; the quench objective never reads it.
                 keepouts: Optional[Sequence[Dict]] = None):
        bounds = pcb_data.board_info.board_bounds
        if bounds is None:
            raise ValueError("No board boundary (Edge.Cuts) found")
        self.board = bounds
        margin = max(clearance, board_edge_clearance)
        self.usable = (bounds[0] + margin, bounds[1] + margin,
                       bounds[2] - margin, bounds[3] - margin)
        # Real board outline / cutouts (#456 item 2): `usable` is a bbox inset,
        # so on an L-shaped outline or a board with interior cutouts it happily
        # nudges parts into the notch or the hole. The gate measures against the
        # true Edge.Cuts rings and self-disables when the bbox inset is already
        # exact (single rectangular ring, no cutouts) or when the parser found no
        # usable ring at all -- in which case behaviour is unchanged.
        self.edge_gate = BoardOutlineGate(pcb_data.board_info, margin)
        self.clearance = clearance
        self.crossing_penalty = crossing_penalty
        self.length_weight = length_weight
        self.net_weights = net_weights or {}
        self.halo_weight = halo_weight
        self.edge_halo = edge_halo
        self.edge_weight = edge_weight
        self.grid_step = grid_step

        # Kept so the FAB body channel can be resolved lazily -- see
        # `fab_rect`. Nothing is parsed unless something asks.
        self.pcb_file = pcb_file
        self.pcb_data = pcb_data
        self._fab_local = None
        self._fab_cache = {}

        courtyards = extract_courtyard_sides(pcb_file)
        locked_refs = set(extract_locked_refs(pcb_file))
        if extra_locked_refs:
            locked_refs |= extra_locked_refs
        ignore = ignore_net_ids or set()

        self.parts: Dict[str, _Part] = {}
        no_courtyard = []
        for ref, fp in pcb_data.footprints.items():
            if not fp.pads:
                # Zero-pad footprints (graphics-only mechanical parts, logos
                # with a courtyard) used to be dropped entirely -- neither
                # movable NOR an obstacle, so the optimizer walked parts onto
                # them. With a drawn courtyard they now enter as locked static
                # obstacles; without one there is no geometry to respect.
                if ref in courtyards:
                    self.parts[ref] = _Part(ref, fp, courtyards, True,
                                            halo_base, halo_coef)
                continue
            locked = (ref in locked_refs
                      or (move_refs is not None and ref not in move_refs))
            if ref not in courtyards:
                no_courtyard.append(ref)
            self.parts[ref] = _Part(ref, fp, courtyards, locked,
                                    halo_base, halo_coef)
            # A part with NO connected pins (mounting hole, NPTH, fiducial) is
            # invisible to the airwire cost -- only halo/edge decide where it
            # goes, which is how holes wander. Frozen by default; the caller
            # frees them explicitly with move_unconnected (--move-unconnected).
            if self.parts[ref].pin_count == 0 and not move_unconnected:
                self.parts[ref].locked = True
            # Ignored nets (e.g. plane-routed power) don't contribute airwires
            self.parts[ref].nets = [n for n in self.parts[ref].nets
                                    if n not in ignore]
        warn_missing_courtyards(no_courtyard, 'quench')

        # #701 intent keep-outs, resolved ONCE per state rather than once per
        # candidate pose. Neither an `allow` fnmatch against a reference nor
        # the set of faces a part occupies changes when the part moves, so the
        # only pose-dependent work left for the seat predicate is the geometry
        # itself. `_try_place` evaluates thousands of poses per part, so this
        # is the difference between a conjunct and a regression.
        #
        # `keepouts_for` is EMPTY on every board that declares no keep-out --
        # which is every board in the corpus today -- and `pose_ok` guards on
        # that emptiness, so the whole channel is inert unless asked for.
        self.keepouts = tuple(keepouts or ())
        self.keepouts_for: Dict[str, Tuple[Dict, ...]] = {}
        if self.keepouts:
            from . import floorplan as _fp
            for _ref, _p in self.parts.items():
                _binding = _fp.keepouts_for_ref(self.keepouts, _ref, _p.sides)
                if _binding:
                    self.keepouts_for[_ref] = _binding

        # Run-6 CONTAINER exemption: a courtyard covering most of the board
        # is a FRAME (a module-outline footprint hosting the whole design),
        # not a body -- measured on rp2350_fpga_eensy: U8's courtyard is
        # 1.13x the board area and the courtyard-hard gate refused EVERY
        # pose on the board (13 unrepairable, 0.00mm moved), while the next
        # largest ratio anywhere in the 33-board corpus is 0.29 (a
        # connector). Pairs with a container member skip the courtyard
        # channels; the PAD layer (pads_ok) still applies in full -- the
        # module's pads are real obstacles.
        barea = max(1e-9, (bounds[2] - bounds[0]) * (bounds[3] - bounds[1]))
        self.container_refs = set()
        for ref, p in self.parts.items():
            r = p.rect()
            if (r[2] - r[0]) * (r[3] - r[1]) >= CONTAINER_RATIO * barea:
                self.container_refs.add(ref)
        if self.container_refs:
            print(f"  container footprint(s) (courtyard >= "
                  f"{CONTAINER_RATIO:.0%} of the board -- frame, not body): "
                  f"{', '.join(sorted(self.container_refs))}")

        # --- pad + drill legality (gate currency; see placement/legality.py).
        # pose_of/seed_of read the live _Part records, so the context follows
        # every apply_move with no invalidation; baselines key off SEED poses.
        self.pad_legality = bool(pad_legality)
        self.legality_ctx = None
        if self.pad_legality:
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
                model=pad_model)

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

        # Dense net_id -> weight lookup for the crossing kernel, derived once
        # from net_weights (which is not mutated after construction) and sized
        # to cover every net id that can appear in an airwire array's column 4.
        # None when there is nothing to weight, which short-circuits the
        # crossing kernel back onto its exact integer path.
        self._net_w = None
        if self.net_weights:
            size = max(max(self.net_weights),
                       max(self.net_airwires, default=-1)) + 1
            self._net_w = np.ones(size)
            for net_id, w in self.net_weights.items():
                if net_id >= 0:
                    self._net_w[net_id] = w

        # Optional pruned neighbour lists (see build_neighbor_lists)
        self._neighbors = None
        # Displacement budget, for the outline gate's reachability prune. Unknown
        # until build_neighbor_lists is told it, and UNBOUNDED until then so the
        # prune can only ever be conservative (every part pays for the exact ring
        # test) rather than skip a part that can in fact reach an edge.
        self._travel_budget = float('inf')
        # ref -> violation of its CURRENT pose; whole-dict invalidated on any
        # move, since a move changes its neighbours' violations too.
        self._inc_violation: Dict[str, float] = {}
        # ref -> ids of the milled rings its OWN pads sit inside (#628). Keyed
        # on the SEED pose, so unlike _inc_violation this NEVER invalidates.
        self._owned_rings_cache: Dict[str, frozenset] = {}

        # --- #548: alignment and orientation, both OFF unless asked for ------
        self.align_weight = float(align_weight)
        self.align_radius = float(align_radius)
        self.align_span = float(align_span)
        self.orient_weight = float(orient_weight)
        self._peers = self._build_peers(self.align_span) if (
            self.align_weight > 0.0) else {}
        # (ref, net_id) -> centroid of that net's pads owned by OTHER parts, or
        # None when this part is the net's only owner. Cleared beside
        # _inc_violation on every move.
        self._anchors: Dict[Tuple[str, int], Optional[Tuple[float, float]]] = {}

        # --- corridor cut, OFF unless a weight was asked for -----------------
        self.corridor_weight = float(corridor_weight)
        self._corridor_boxes: List[_CorridorBox] = []
        if self.corridor_weight > 0.0 and corridor_specs:
            self._corridor_boxes = self._freeze_corridors(
                pcb_data, corridor_specs, ignore, corridor_max_fanout)

    def _freeze_corridors(self, pcb_data, specs, ignore_net_ids, max_fanout):
        """Corridor rectangles, built ONCE and never rebuilt.

        This is the load-bearing property of the whole term, so it is enforced
        structurally rather than by discipline: `_cluster_ends` derives a
        corridor's endpoints from LIVE pad positions, so a corridor recomputed
        after a move would make the objective non-stationary -- the cost of a
        pose would depend on when it was evaluated, `apply_move`'s accepted gain
        would not match the recomputed total, and a greedy descent could cycle.
        Freezing at construction makes that impossible rather than merely
        avoided. It also makes the model independent of the poses it scores,
        which is what lets `check_floorplan --health` re-derive corridors from
        the FINAL placement and act as an honest check on the result.
        """
        from .routability import corridors_from_intent
        boxes: List[_CorridorBox] = []
        size = max(max(self.net_airwires, default=-1),
                   max(pcb_data.nets, default=-1)) + 1
        base = np.zeros(max(size, 1), dtype=bool)
        for nid in ignore_net_ids:
            if 0 <= nid < len(base):
                base[nid] = True
        if max_fanout:
            for nid, refs in self.net_refs.items():
                if len(refs) > max_fanout and 0 <= nid < len(base):
                    base[nid] = True
        for cor in corridors_from_intent(self, pcb_data, specs):
            n = cor.length_mm
            if n < 1e-9 or cor.width_mm <= 0.0:
                continue
            skip = base.copy()
            for nid in cor.net_ids:
                if 0 <= nid < len(skip):
                    skip[nid] = True
            boxes.append(_CorridorBox(
                ax=cor.a[0], ay=cor.a[1],
                ux=(cor.b[0] - cor.a[0]) / n, uy=(cor.b[1] - cor.a[1]) / n,
                length=n, half_w=cor.width_mm / 2.0, skip=skip))
        return boxes

    def _build_peers(self, span: float) -> Dict[str, List[str]]:
        """{ref: sorted peer refs} -- same footprint_name, SEED centres within
        `span`.

        Built HERE and not in `build_neighbor_lists`, deliberately.
        `tests/test_quench_neighbor_lists.py:60` asserts a state that never
        called `build_neighbor_lists` has `_neighbors is None`, and :79-82
        asserts that state's `part_geometry_cost` equals the built one's
        EXACTLY. A peer index that only existed on the built path would break
        that equality as a correctness failure, not a baseline drift.

        It also cannot ride on `_neighbors` at all: that prune is a 2-D BOX
        overlap test, so a pair must be near in BOTH axes. Alignment is
        inherently long-range ALONG the shared axis -- two caps 50mm apart in x
        and 0.1mm apart in y ARE aligned -- and no widening of the box margin
        covers that without inflating every neighbour list board-wide.

        Seed-relative on purpose: "align to the peers you started next to" is
        what `your placement, nudged` means. A pair whose seeds are just over
        `span` apart and that later drift within it is NOT picked up. That makes
        this a LOSSY prune, unlike `_neighbors`' exact one, and it is said out
        loud here rather than left to be discovered.
        """
        by_fp: Dict[str, List[str]] = {}
        for ref in sorted(self.parts):
            by_fp.setdefault(self.parts[ref].footprint_name, []).append(ref)
        out: Dict[str, List[str]] = {}
        for group in by_fp.values():
            if len(group) < 2:
                continue
            seeds = {r: (self.parts[r].seed_x, self.parts[r].seed_y)
                     for r in group}
            for ref in group:
                sx, sy = seeds[ref]
                near = [o for o in group
                        if o != ref
                        and math.hypot(seeds[o][0] - sx, seeds[o][1] - sy) <= span]
                if near:
                    out[ref] = near
        return out

    # ----- #548 cost terms --------------------------------------------------

    def _align_pair_penalty(self, part_a, rect_a, part_b, rect_b) -> float:
        """Off-axis misalignment between two PEER parts.

            d   = min(|cx_a - cx_b|, |cy_a - cy_b|)     # the nearer shared axis
            pen = align_weight * min(d, align_radius) ** 2

        Three properties chosen deliberately:

        CONTINUOUS at `align_radius`. The obvious "charge inside the radius,
        zero outside" shape has a cliff there, which pays a part to FLEE the row
        rather than join it -- the exact opposite of the intent.

        SATURATING beyond it, so every distant peer contributes the same
        constant and it cancels between one part's candidate poses instead of
        dragging it across the board toward a peer it will never reach.

        ZERO exactly on a shared axis, so a tidy row costs nothing.

        Peers means identical `footprint_name`: centre-to-centre is the right
        anchor for two instances of one library footprint and the wrong one for
        an 0402 against a BGA. It is also the pairing the swap phase already
        indexes, so the notion is not new to this module.
        """
        d = min(abs((rect_a[0] + rect_a[2]) - (rect_b[0] + rect_b[2])),
                abs((rect_a[1] + rect_a[3]) - (rect_b[1] + rect_b[3]))) * 0.5
        if d >= self.align_radius:
            d = self.align_radius
        return self.align_weight * d * d

    def _align_cost(self, ref, rect, exclude: Optional[Set[str]] = None
                    ) -> float:
        if self.align_weight <= 0.0:
            return 0.0
        peers = self._peers.get(ref)
        if not peers:
            return 0.0
        part = self.parts[ref]
        pen = 0.0
        for other_ref in peers:
            if exclude and other_ref in exclude:
                continue
            other = self.parts.get(other_ref)
            if other is None:
                continue
            pen += self._align_pair_penalty(part, rect, other, other.rect())
        return pen

    def _net_anchor(self, ref, net_id):
        """Centroid of `net_id`'s pads owned by parts OTHER than `ref`."""
        key = (ref, net_id)
        if key in self._anchors:
            return self._anchors[key]
        xs = ys = 0.0
        n = 0
        # net_refs values are SORTED, so the float summation order is a property
        # of the net rather than of dict iteration (#457).
        for other in self.net_refs.get(net_id, ()):
            if other == ref:
                continue
            part = self.parts.get(other)
            if part is None:
                continue
            for gx, gy, pn in part.pad_globals():
                if pn == net_id:
                    xs += gx
                    ys += gy
                    n += 1
        val = (xs / n, ys / n) if n else None
        self._anchors[key] = val
        return val

    def _orient_cost(self, ref, x=None, y=None, rot=None) -> float:
        """Reward a pose whose pads FACE the nets they serve (#548 item 2).

        With `o` the pose origin, `r = pad_global - o` and `u` the unit vector
        from `o` toward that net's anchor:

            cost = orient_weight * sum_p ( |r| - r . u )

        Zero when a pad points exactly at its anchor, `2|r|` when exactly away,
        so the whole term is bounded by `2 * orient_weight * sum|r|` -- mm-scale
        per part. Bounded on purpose: this should break a rotation TIE, never
        outrank a real length win.

        Why this rather than "score airwires from the actual pad", as #548
        proposes: the cost path ALREADY does that. `_net_points` emits one MST
        node per connected pad from `pad_globals()`, full rotation applied;
        there is no centroid anywhere in the objective. The gap is NUMERIC -- a
        ~1mm pad offset perturbs a ~20mm net's MST length by a fraction of a mm,
        less once the tree re-roots -- so the directional signal is present and
        drowned. This extracts the same signal at part scale and gives it its
        own weight.

        Two things it is NOT, worth stating: it is not purely rotational, since
        moving the part also changes `u`, so a large weight pulls a part toward
        its nets and overlaps the length term; and through `part_geometry_cost`
        it also reaches the group phase, so a rigid block translate is steered
        by it too. Both intended, neither obvious.
        """
        if self.orient_weight <= 0.0:
            return 0.0
        part = self.parts[ref]
        ox = part.x if x is None else x
        oy = part.y if y is None else y
        pen = 0.0
        for gx, gy, net_id in part.pad_globals(x, y, rot):
            anchor = self._net_anchor(ref, net_id)
            if anchor is None:
                continue
            rx, ry = gx - ox, gy - oy
            rn = math.hypot(rx, ry)
            if rn < 1e-9:
                continue
            ax, ay = anchor[0] - ox, anchor[1] - oy
            an = math.hypot(ax, ay)
            if an < 1e-9:
                continue
            pen += rn - (rx * ax + ry * ay) / an
        return self.orient_weight * pen

    # ----- airwire helpers -------------------------------------------------

    def _net_points(self, net_id, override_ref=None, override_pads=None,
                    overrides=None):
        """A net's pad points, with any number of parts held at a HYPOTHETICAL
        pose (#459).

        `overrides` is {ref: pad_globals}; `override_ref`/`override_pads` are the
        single-part spelling every existing caller uses. A rigid block move has
        to override N parts at once, and the swap phase already needed two --
        which it solved by hand-inlining a second copy of this loop, with a
        comment warning that the two must not drift apart. This is that one
        implementation.

        The `for ref in self.net_refs[net_id]` iteration is load-bearing:
        net_refs is a SORTED list and this order becomes the point order handed
        to compute_mst_edges, whose tie-break is first-index-wins. Changing it
        makes quench output vary across processes again (#457).
        """
        if overrides is None:
            overrides = ({override_ref: override_pads}
                         if override_ref is not None else {})
        pts = []
        for ref in self.net_refs[net_id]:
            pads = overrides.get(ref)
            if pads is None:
                pads = self.parts[ref].pad_globals()
            pts.extend((gx, gy) for gx, gy, n in pads if n == net_id)
        return pts

    def _build_net_airwires(self, net_id, override_ref=None, override_pads=None,
                            overrides=None):
        return _airwires_for_points(
            self._net_points(net_id, override_ref, override_pads, overrides),
            net_id)

    def airwires_excluding(self, nets: Set[int]) -> np.ndarray:
        aws = []
        for net_id, lst in self.net_airwires.items():
            if net_id not in nets:
                aws.extend(lst)
        return _aw_array(aws)

    # ----- cost terms ------------------------------------------------------

    def _halo_pair_penalty(self, part_a: _Part, rect_a, part_b: _Part, rect_b,
                           rects_a=None, rects_b=None):
        """Whitespace-shortfall penalty between two parts.

        Zero for a cross-side pair that shares no board side: their whitespace is
        not shared, so pushing them apart buys no routing room (#456 item 1). The
        explicit rect_a / rect_b stay in the signature because every caller has
        them already; rects_a / rects_b carry the far-side rects when the caller
        has them (defaulting to the parts' live poses).
        """
        if (part_a.ref in self.container_refs
                or part_b.ref in self.container_refs):
            return 0.0    # run-6: container = frame, not body
        required = part_a.halo + part_b.halo
        if not (part_a.has_tht or part_b.has_tht):
            # Fast path (nearly every pair): plain SMD parts interact only when
            # they are on the same side, and a per-axis separation of `required`
            # already proves the true gap clears it -- so the exact gap is only
            # computed for pairs that can actually be charged.
            if part_a.side != part_b.side:
                return 0.0
            if (rect_a[2] + required <= rect_b[0]
                    or rect_b[2] + required <= rect_a[0]
                    or rect_a[3] + required <= rect_b[1]
                    or rect_b[3] + required <= rect_a[1]):
                return 0.0
            gap = rect_gap(rect_a, rect_b)
        else:
            gap = part_a.gap_to(
                part_b,
                (rect_a, part_a.tht_rect()) if rects_a is None else rects_a,
                (rect_b, part_b.tht_rect()) if rects_b is None else rects_b)
            if gap is None:
                return 0.0
        if gap >= required:
            return 0.0
        # No clamp at zero: a deeper overlap must cost MORE than a shallow one,
        # or nothing in the objective ever repairs an existing overlap (the
        # clamp made a 2mm-deep overlap price identically to a touching pair).
        # Bit-identical on any board with no overlapping pair -- the hard gate
        # forbids CREATING one, so only seeded-in violations see the change.
        short = required - gap
        return self.halo_weight * short * short

    def _edge_penalty(self, rect, ref=None):
        """Soft margin inside the board edge.

        Measured to the real outline when we have one, so a part sitting in a
        notch is charged for the notch's edges rather than for a bounding box it
        is nowhere near (#456 item 2). Falls back to the four bbox gaps.
        """
        if self.edge_gate.active:
            # Same prefilter as the hard gate, but sized to edge_halo, which is
            # the radius this soft term cares about and is usually WIDER than the
            # hard margin. Empty list = nothing within edge_halo, penalty 0.
            near = (self.edge_gate.edges() if ref is None
                    else self._edges_near_halo(ref))
            if not near:
                return 0.0
            g = self.edge_gate.edge_clearance(rect, edges=near)
            if g >= self.edge_halo:
                return 0.0
            short = self.edge_halo - max(g, 0.0)
            # One charge on the NEAREST edge, matching what the per-axis sum
            # below charges a part near a single edge -- which is the ordinary
            # case the weights were tuned against. (Charging per-direction would
            # need four directional distances to the outline; multiplying this
            # one by four instead would bill every edge-adjacent part as though
            # it were boxed in on all sides, a 4x distortion of the term.)
            return self.edge_weight * short * short
        pen = 0.0
        gaps = (rect[0] - self.board[0], rect[1] - self.board[1],
                self.board[2] - rect[2], self.board[3] - rect[3])
        for g in gaps:
            if g < self.edge_halo:
                short = self.edge_halo - max(g, 0.0)
                pen += self.edge_weight * short * short
        return pen

    def part_geometry_cost(self, ref, x=None, y=None, rot=None,
                           exclude: Optional[Set[str]] = None):
        """Halo + edge penalty contributions of one part at a position."""
        part = self.parts[ref]
        rects = part.rects(x, y, rot)
        rect = rects[0]
        pen = self._edge_penalty(rect, ref)
        if self._neighbors is not None and ref in self._neighbors:
            others = ((o, self.parts[o]) for o in self._neighbors[ref])
        else:
            others = self.parts.items()
        for other_ref, other in others:
            if other_ref == ref or (exclude and other_ref in exclude):
                continue
            pen += self._halo_pair_penalty(part, rect, other, other.rect(),
                                           rects_a=rects)
        # #548. Hooked HERE because this is the one function all three
        # evaluators call -- the nudge pass, the group pass and the swap pass --
        # and it already carries the `exclude` semantics the latter two need.
        # Both return 0.0 before touching any geometry when their weight is 0,
        # so a default run is bit-identical and pays nothing.
        pen += self._align_cost(ref, rect, exclude)
        pen += self._orient_cost(ref, x, y, rot)
        return pen

    def violation(self, ref, x=None, y=None, rot=None,
                  exclude: Optional[Set[str]] = None,
                  limit: Optional[float] = None) -> float:
        """Total illegality of a pose: 0.0 exactly when it is legal.

        The sum of the two terms `violation_parts` returns; see there. This is
        the number `placement/legality.py`'s graders report, so the optimizer and
        the scorecard cannot disagree about what legal means.
        """
        board, overlap = self.violation_parts(ref, x, y, rot, exclude, limit)
        return board + overlap

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
        near = self._edges_near(ref) if self.edge_gate.active else None
        board = self.edge_gate.rect_outside_amount(
            rects[0], exact=bool(near), edges=near,
            skip_rings=self._owned_rings(ref))
        overlap = 0.0
        if limit is not None and board > limit:
            return board, overlap
        if self._neighbors is not None and ref in self._neighbors:
            others = ((o, self.parts[o]) for o in self._neighbors[ref])
        else:
            others = self.parts.items()
        clr = self.clearance
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
        return board, overlap

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
        legal = not (rect[0] < self.usable[0] or rect[1] < self.usable[1]
                     or rect[2] > self.usable[2] or rect[3] > self.usable[3])
        # Real outline / cutout gate, three-level short-circuit: board-level
        # opt-out, cached per-part reachable-edge list, then the exact test
        # against only those edges.
        if legal and self.edge_gate.active:
            near = self._edges_near(ref)
            if near and self.edge_gate.rect_blocked(
                    rect, edges=near, skip_rings=self._owned_rings(ref)):
                legal = False
        if legal:
            if self._neighbors is not None and ref in self._neighbors:
                others = ((o, self.parts[o]) for o in self._neighbors[ref])
            else:
                others = self.parts.items()
            clr = self.clearance
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
                        break
                    continue
                # Fast path: two plain SMD parts, same side. The per-axis test is
                # an early-OUT, not the verdict -- clearing it on any axis proves
                # the true gap clears too, but failing it does not prove the
                # reverse, because the gap is EUCLIDEAN. Two rects offset
                # diagonally by (0.2, 0.2) at clearance 0.25 fail every axis while
                # rect_gap is hypot(0.2,0.2)=0.283, i.e. legal. Using the axis
                # test as the answer made candidate_valid REJECT poses that
                # violation_parts (:577) and _halo_pair_penalty (:447) both score
                # as legal -- so violation()==0 did not imply the hard gate
                # passes, and the unfreeze branch could walk a part into a pose
                # the ordinary gate forbids.
                if other.side != part.side:
                    continue
                r = other.rect()
                if (rect[2] + clr <= r[0] or r[2] + clr <= rect[0]
                        or rect[3] + clr <= r[1] or r[3] + clr <= rect[1]):
                    continue                    # provably clear
                if rect_gap(rect, r) < clr:
                    legal = False
                    break
        if legal:
            # BODY layer. A pose that buries this part inside another part's
            # .Fab body is not a trade-off to be priced -- it is illegal, the
            # same verdict reconstruct._pair_conflicts already returns for the
            # assign/exchange ILP. Without it here, `legalize` and every
            # quench pass could re-seat a charged part straight back inside
            # the body it was charged for, and _try_place's clearance ladder
            # (full, full/2, 0.02) makes such a seat MORE reachable, not less.
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
        if legal and self.legality_ctx is not None:
            # Pad+drill layer: courtyard-clear does not imply pad-clear (pads
            # overhanging courtyards, exchanged nets, NPTH holes). Baseline-
            # relative: the pose may not worsen any pair vs the SEED, and a
            # NEW different-net pad intersection is never admitted.
            legal = self.legality_ctx.pads_ok(
                ref, x, y, rot, self._pad_neighbors(ref), exclude=exclude)
        if legal:
            return True
        # Only now, on a rejected candidate, is the incumbent's legality worth
        # computing -- and it is cached, because it is the same answer for every
        # candidate of this part until something moves. Without the cache this
        # branch runs a full neighbour sweep per rejected candidate, which on a
        # dense board is most of them.
        cur_board, cur_overlap = self._incumbent_violation(ref,
                                                           exclude=exclude)
        if cur_overlap > EPS_IMPROVE or cur_board <= EPS_IMPROVE:
            # Overlapping, or already legal: original rule, legal poses only.
            return False
        cand_board, cand_overlap = self.violation_parts(
            ref, x, y, rot, exclude=exclude, limit=cur_board)
        if not (cand_overlap <= EPS_IMPROVE
                and cand_board < cur_board - EPS_IMPROVE):
            return False
        # The unfreeze branch gets the SAME pad/hole conjunct: a part may move
        # back toward the board only without worsening any pad pair.
        if self.legality_ctx is not None:
            return self.legality_ctx.pads_ok(
                ref, x, y, rot, self._pad_neighbors(ref), exclude=exclude)
        return True

    def _pad_neighbors(self, ref):
        """Neighbor refs for the pad gate: the pruned list when built, else
        everyone. The pruning boxes are widened with pad/hole extents in
        build_neighbor_lists, so the list stays a superset of interacting
        pairs."""
        if self._neighbors is not None and ref in self._neighbors:
            return self._neighbors[ref]
        return [o for o in self.parts if o != ref]

    def swap_pads_ok(self, ra, rb):
        """May two parts exchange poses, pad/hole-wise? The exchange preserves
        courtyard occupancy but NOT nets -- identical copper, exchanged net
        assignments can land a pad inside a foreign (or locked) part's
        clearance that the old net shared. Each part is tested at its partner's
        pose against its own neighbors, plus the pair itself with both at
        their new poses."""
        if self.legality_ctx is None:
            return True
        pa, pb = self.parts[ra], self.parts[rb]
        pose_a = (pb.x, pb.y, pb.rot)
        pose_b = (pa.x, pa.y, pa.rot)
        if not self.legality_ctx.pads_ok(ra, *pose_a,
                                         self._pad_neighbors(ra),
                                         exclude={rb}):
            return False
        if not self.legality_ctx.pads_ok(rb, *pose_b,
                                         self._pad_neighbors(rb),
                                         exclude={ra}):
            return False
        cur = self.legality_ctx.pair_shortfall(ra, rb, pose_a=pose_a,
                                               pose_b=pose_b)
        base = self.legality_ctx.seed_baseline(ra, rb)
        if cur.pad > base.pad + EPS_IMPROVE:
            return False
        if cur.pad_overlap and not base.pad_overlap:
            return False
        return cur.hole <= base.hole + EPS_IMPROVE

    def _incumbent_violation(self, ref, exclude=None):
        """The incumbent pose's violation, cached per (ref, exclude).

        The `exclude` half is why this exists. `candidate_valid`'s rejection
        path guarded the cache with `if exclude:` and fell through to an
        uncached `violation_parts` whenever one was supplied -- and the seeder
        ALWAYS supplies one (the unplaced pile), so on the path that matters
        the cache never ran. Measured over 30s of a real seeding run: 61,119
        incumbent-pose calls, of which 61,092 (99.96%) were exact repeats of
        the same (ref, clearance, exclude). Each one walks the neighbours and
        the outline; the comment three lines above the guard already said this
        must not happen.

        The key is a frozenset, so it costs O(|exclude|) hashing against a
        full neighbour-and-outline sweep -- cheap by a wide margin. The whole
        cache is cleared on every move (see apply_move), which is what makes
        an incumbent answer safe to hold at all.
        """
        key = (ref, frozenset(exclude) if exclude else None)
        v = self._inc_violation.get(key)
        if v is None:
            v = self.violation_parts(ref, exclude=exclude)
            self._inc_violation[key] = v
        return v

    def _edges_near(self, ref) -> list:
        """Cached: the Edge.Cuts edges this part's reachable disk can touch.
        Empty means the exact ring test is skippable for every pose it can take."""
        part = self.parts[ref]
        travel = 0.0 if part.locked else self._travel_budget
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

    def _edges_near_halo(self, ref) -> list:
        """Like _edges_near but sized to the SOFT edge_halo radius. Inflating
        `travel` by edge_halo is the conservative way to widen the gate's
        margin-based reach without a second reach parameter."""
        part = self.parts[ref]
        travel = 0.0 if part.locked else self._travel_budget
        return self.edge_gate.edges_near(
            (ref, 'halo'), part.rect(part.seed_x, part.seed_y, part.orig_rot),
            travel + self.edge_halo, center=(part.seed_x, part.seed_y))

    def _may_reach_edge(self, ref) -> bool:
        return bool(self._edges_near(ref))

    def _weighted_length(self, arr: np.ndarray) -> float:
        if len(arr) == 0:
            return 0.0
        lengths = np.hypot(arr[:, 2] - arr[:, 0], arr[:, 3] - arr[:, 1])
        if self.net_weights:
            w = np.array([self.net_weights.get(int(n), 1.0)
                          for n in arr[:, 4]])
            lengths = lengths * w
        return float(np.sum(lengths))

    def nets_cost(self, net_airwires_subset: Dict[int, List],
                  other_airwires: np.ndarray):
        """Length + crossing cost of the given nets' airwires, counting
        crossings against `other_airwires` and among themselves.

        Returns (cost, crossings). The cost prices each crossing at the larger
        of the two nets' weights (#458), so a weighted net's crossings, not
        just its far cheaper length, carry the weight; `crossings` stays the
        raw unweighted count for reporting."""
        own = []
        for lst in net_airwires_subset.values():
            own.extend(lst)
        own_arr = _aw_array(own)
        length = self._weighted_length(own_arr)
        n_out, w_out = _count_crossings_np(own_arr, other_airwires, self._net_w)
        n_in, w_in = _count_crossings_within(own_arr, self._net_w)
        cost = (self.length_weight * length
                + self.crossing_penalty * (w_out + w_in))
        if self._corridor_boxes:
            # Only the SUBSET's chords: `other_airwires` is fixed across the
            # candidate poses this cost ranks, so its cut is a constant that
            # cancels in the argmin -- the same reason its length is not summed
            # here either.
            cost += self.corridor_weight * _corridor_cut_np(
                own_arr, self._corridor_boxes)
        return cost, n_out + n_in

    # ----- full cost (for reporting) ---------------------------------------

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

        Lazy: nothing is parsed until the first call, so every existing
        quench/seeder path pays nothing.
        """
        if self._fab_local is None:
            try:
                from placement.parser import extract_fab_sides
                self._fab_local = extract_fab_sides(self.pcb_file) or {}
            except Exception:
                self._fab_local = {}
        p = self.parts.get(ref)
        if p is None:
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
            own = 'B' if str(getattr(p, 'side', 'F')).upper().startswith('B')                 else 'F'
            lb = sides.get(own) or next(iter(sides.values()))
            local = rotate_local_bounds(*lb, rot)
            self._fab_cache[key] = local
        return (x + local[0], y + local[1], x + local[2], y + local[3])

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
        if self._neighbors is not None and ref in self._neighbors:
            others = [(o, self.parts[o]) for o in self._neighbors[ref]]
        else:
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
                return True
        return False

    def total_cost(self):
        all_aw = _aw_array([aw for lst in self.net_airwires.values() for aw in lst])
        length = self._weighted_length(all_aw)
        # `crossings` stays the raw unweighted count so the pass banner and
        # any count-based expectation are unchanged; `total` is the objective
        # the quench actually minimizes, which is weighted (#458), matching
        # `length`, which has always been weighted.
        crossings, w_crossings = _count_crossings_within(all_aw, self._net_w)
        halo = 0.0
        edge = 0.0
        align = 0.0
        orient = 0.0
        refs = list(self.parts)
        peers = self._peers
        for i, ra in enumerate(refs):
            pa = self.parts[ra]
            rect_a = pa.rect()
            edge += self._edge_penalty(rect_a, ra)
            orient += self._orient_cost(ra)
            near = peers.get(ra) if peers else None
            for rb in refs[i + 1:]:
                pb = self.parts[rb]
                halo += self._halo_pair_penalty(pa, rect_a, pb, pb.rect())
                # #548. Counted over UNORDERED pairs here, matching halo, so the
                # report shows each physical pair once. part_geometry_cost sums
                # one part's pairs from that part's side, which is the factor of
                # 2 the evaluators need and which cancels between candidates.
                if near and rb in near:
                    align += self._align_pair_penalty(pa, rect_a, pb, pb.rect())
        cut = (_corridor_cut_np(all_aw, self._corridor_boxes)
               if self._corridor_boxes else 0.0)
        total = (self.length_weight * length
                 + self.crossing_penalty * w_crossings + halo + edge
                 + align + orient + self.corridor_weight * cut)
        return {'total': total, 'length': length, 'crossings': crossings,
                'halo': halo, 'edge': edge, 'hpwl': self.hpwl(),
                'align': align, 'orient': orient, 'corridor_cut': cut}

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
        geometry and the same side rules the optimizer gated on, rather than
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
        out = {'overlap_area': legality.placement_overlap_area(parts),
               'oob_count': oob_count, 'oob_amount': oob_amount,
               'oob_area': oob_area, 'hpwl': self.hpwl()}
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
        return {'pad_conflict_pairs': pairs,
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
        # #548: a move changes which pads other parts see, so every
        # net anchor computed against this part is now stale.
        self._anchors.clear()
        for net_id in part.nets:
            self.net_airwires[net_id] = self._build_net_airwires(net_id)

    def apply_group_move(self, refs, dx, dy):
        """Translate a whole block rigidly by (dx, dy) (#459).

        Every member's pose is set FIRST, then each affected net is rebuilt once.
        Calling apply_move per member would be correct but rebuilds a shared net
        once per member that touches it, which on a 20-part block is most of the
        cost of evaluating the move again.
        """
        nets = set()
        for ref in refs:
            part = self.parts[ref]
            part.x += dx
            part.y += dy
            nets.update(part.nets)
        self._inc_violation.clear()
        # #548: a move changes which pads other parts see, so every
        # net anchor computed against this part is now stale.
        self._anchors.clear()
        for net_id in nets:
            self.net_airwires[net_id] = self._build_net_airwires(net_id)

    def group_move_valid(self, refs, dx, dy):
        """Is a rigid translate of `refs` by (dx, dy) legal?

        Intra-group pairs are EXCLUDED. Under a rigid translate the block's
        internal geometry is invariant, so those pairs contribute exactly what
        they did before the move -- but candidate_valid re-tests every pair, and
        on a real board members routinely sit at sub-clearance courtyard gaps
        already (watchy seeds 81 of 82 parts in violation), so without the
        exclusion a block would veto its own every candidate. The swap phase uses
        `exclude` in exactly this way.

        Each member must also stay within its own seed cap, which is what keeps
        build_neighbor_lists' pruning exact and the outline gate's cached reach
        valid -- see the group phase in quench().
        """
        others = set(refs)
        for ref in refs:
            part = self.parts[ref]
            if not self.candidate_valid(ref, part.x + dx, part.y + dy, part.rot,
                                        exclude=others):
                return False
        return True

    def build_neighbor_lists(self, travel_budget):
        """Per-movable-part pruned neighbour lists (perf, mirrors the
        fanout_clearance pattern from #213 profiling). A movable part's live
        position stays within travel_budget of its seed (nudge candidates are
        radius-checked against the seed; swaps are capped by swap_cap <=
        max_displacement), and rect() only ever reads bounds_by_rot, so the
        union-of-rotations box at the seed inflated by the budget contains
        every rect the part can ever occupy. Any pair whose per-axis seed gap
        exceeds both budgets plus the largest interaction reach (hard
        clearance / summed halos / either part's largest PAD requirement) can
        NEVER interact -- excluding it is exact for candidate_valid AND
        part_geometry_cost, not an approximation.

        The pad term is what #697 added: a pad's required clearance can exceed
        the hard clearance (a fiducial keep-clear, a net class, a .kicad_dru
        rule), and a reach that ignored it would quietly make this prune LOSSY
        for the pad gate -- dropping exactly the pairs that gate exists to
        catch. `PartPads.max_floor` is an upper bound per part, so folding it in
        keeps the claim above literally true; it is 0.0 on a board that
        declares nothing.

        Exactness survives the side rule unchanged: the side filter only ever
        REMOVES pairs from consideration, so an XY-only prune stays a superset of
        what the checkers consult. It is deliberately not folded in here -- the
        courtyard box is the pruning box, and a same-footprint swap must keep the
        pair whether or not the pose it lands on shares a side."""
        self._travel_budget = travel_budget
        # A swap can hand a part any rotation currently held by a movable
        # same-footprint partner (the swap path adds the bounds entry lazily,
        # after this build), so each part's union box must cover its whole
        # group's rotation set, not just its own bounds_by_rot entries. Since
        # _Part records the full 90-degree lattice of a non-orthogonal seed,
        # that union is exactly {seed angle + 90k} over every movable part of
        # the group, which is the closure of what a swap can hand over and a
        # nudge can then rotate within.
        group_rots: Dict[str, set] = {}
        for p in self.parts.values():
            if not p.locked:
                group_rots.setdefault(p.footprint_name, set()).update(
                    p.bounds_by_rot)
        # Pad/hole extents must also be bounded by the pruning boxes, or the
        # pad gate's neighbor lists silently become lossy for parts whose pads
        # overhang their courtyard (THT connectors are the common case).
        def _pad_ext_boxes(ref, rots):
            if self.legality_ctx is None:
                return []
            pp = self.legality_ctx.parts.get(ref)
            if pp is None:
                return []
            out = []
            for r in rots:
                e = pp.extent_local(r)
                if e is not None:
                    out.append(e)
            return out

        geom = {}
        for ref, p in self.parts.items():
            if p.locked:
                lr = p.rect()
                tr = p.tht_rect()
                box = (lr if tr is None else
                       (min(lr[0], tr[0]), min(lr[1], tr[1]),
                        max(lr[2], tr[2]), max(lr[3], tr[3])))
                if self.legality_ctx is not None:
                    pp = self.legality_ctx.parts.get(ref)
                    pe = pp.extent(p.x, p.y, p.rot) if pp is not None else None
                    if pe is not None:
                        box = (min(box[0], pe[0]), min(box[1], pe[1]),
                               max(box[2], pe[2]), max(box[3], pe[3]))
                geom[ref] = (box, 0.0)
            else:
                boxes = [p.bounds_by_rot[r] if r in p.bounds_by_rot
                         else _rotate_local_bounds(*p.bounds_by_rot[0.0], r)
                         for r in group_rots[p.footprint_name]]
                # A THT part also presents its lead field on the far side; that
                # box is normally inside the courtyard, but a badly drawn
                # courtyard can be smaller, and the pruning box must bound EVERY
                # rect the pair tests can ask about or the prune stops being exact.
                if p.tht_by_rot is not None:
                    boxes += [p.tht_by_rot[r] if r in p.tht_by_rot
                              else _rotate_local_bounds(*p.tht_by_rot[0.0], r)
                              for r in group_rots[p.footprint_name]]
                boxes += _pad_ext_boxes(ref, group_rots[p.footprint_name])
                u0 = min(b[0] for b in boxes)
                u1 = min(b[1] for b in boxes)
                u2 = max(b[2] for b in boxes)
                u3 = max(b[3] for b in boxes)
                geom[ref] = ((p.seed_x + u0, p.seed_y + u1,
                              p.seed_x + u2, p.seed_y + u3), travel_budget)
        self._neighbors = {}
        for ref, p in self.parts.items():
            if p.locked:
                continue
            ra, ba = geom[ref]
            lst = []
            for oref, (rb, bb) in geom.items():
                if oref == ref:
                    continue
                reach = self.clearance
                if self.legality_ctx is not None:
                    pa = self.legality_ctx.parts.get(ref)
                    pb = self.legality_ctx.parts.get(oref)
                    if pa is not None and pa.max_floor > reach:
                        reach = pa.max_floor
                    if pb is not None and pb.max_floor > reach:
                        reach = pb.max_floor
                m = ba + bb + max(reach,
                                  p.halo + self.parts[oref].halo) + 1e-9
                if (ra[2] + m >= rb[0] and rb[2] + m >= ra[0]
                        and ra[3] + m >= rb[1] and rb[3] + m >= ra[1]):
                    lst.append(oref)
            self._neighbors[ref] = lst


def _group_offsets(state, refs, max_disp: float, step: float, grid_step: float):
    """Rigid (dx, dy) offsets a whole block may take (#459).

    The block translates as one body, so a single offset applies to every
    member -- and the offset is admissible only if it keeps EVERY member within
    `max_disp` of ITS OWN seed. That per-member seed cap is what makes the block
    move safe to add without touching anything else:

      * build_neighbor_lists' exactness argument is stated per part ("a movable
        part's live position stays within travel_budget of its seed"), and stays
        true, so the pruned neighbour lists remain exact rather than lossy;
      * BoardOutlineGate.edges_near caches its reachable-edge list per ref on
        first call, sized by that same budget -- a block allowed to travel
        further would silently outrun the cache and skip the exact ring test;
      * test_quench_swap_cap's no-stranding invariant ("no part further than
        max_displacement + grid snap from where it started") keeps holding
        unmodified.

    Lifting that cap is what makes #459's 80mm relocation a separate piece of
    work rather than a bigger number here.
    """
    n = int(max_disp / step)
    if n <= 0:
        return
    seen = set()
    for ix in range(-n, n + 1):
        for iy in range(-n, n + 1):
            if ix == 0 and iy == 0:
                continue
            dx, dy = ix * step, iy * step
            if math.hypot(dx, dy) > max_disp + 1e-9:
                continue
            ok = True
            for ref in refs:
                p = state.parts[ref]
                nx = snap_to_grid(p.x + dx, grid_step)
                ny = snap_to_grid(p.y + dy, grid_step)
                if math.hypot(nx - p.seed_x, ny - p.seed_y) > max_disp + 1e-9:
                    ok = False
                    break
            if not ok:
                continue
            # Snap the OFFSET, so the block stays rigid: snapping each member
            # independently would shear it by up to a grid step.
            sdx = snap_to_grid(dx, grid_step)
            sdy = snap_to_grid(dy, grid_step)
            if (sdx, sdy) in seen or (sdx == 0.0 and sdy == 0.0):
                continue
            seen.add((sdx, sdy))
            yield sdx, sdy


def _candidate_positions(part: _Part, max_disp: float, step: float,
                         grid_step: float):
    """Grid of candidate centers within max_disp of the seed position."""
    seen = set()
    out = []
    n = int(max_disp / step)
    for ix in range(-n, n + 1):
        for iy in range(-n, n + 1):
            cx = part.seed_x + ix * step
            cy = part.seed_y + iy * step
            if math.hypot(cx - part.seed_x, cy - part.seed_y) > max_disp + 1e-9:
                continue
            cx = snap_to_grid(cx, grid_step)
            cy = snap_to_grid(cy, grid_step)
            key = (round(cx, 4), round(cy, 4))
            if key not in seen:
                seen.add(key)
                out.append((cx, cy))
    return out


def _candidate_rotations(part: _Part, allow_rotations: bool) -> List[float]:
    """Rotation candidates for a nudge move.

    The 90-degree lattice through the part's CURRENT angle, plus the lattice
    through its seed angle when a swap has moved it off that one. Two
    consequences beyond the old "the four axis rotations, but only if the seed
    is orthogonal" rule: a part placed at 45 degrees can rotate at all, to
    135/225/315, staying on its own lattice rather than being snapped onto the
    axes; and the current pose is always among the candidates, so such a part
    can be MOVED while KEEPING its angle.

    Generated in ROTATIONS order from a base of rot % 90, so for any part
    sitting at a multiple of 90, which is every part on a board with only
    orthogonal footprint rotations, this returns exactly ROTATIONS: same
    values, same order. The order is load-bearing, not style: the caller keeps
    the FIRST strict minimum, so a reordered list would silently change which
    pose wins a tie. Do not rewrite this as a set.
    """
    if not allow_rotations:
        return [part.rot]
    bases = [part.rot % 90]
    if part.orig_rot % 90 != bases[0]:
        bases.append(part.orig_rot % 90)
    return [(b + r) % 360 for b in bases for r in ROTATIONS]


def quench(pcb_data: PCBData, pcb_file: str,
           max_displacement: float = 10.0,
           swap_max_displacement: Optional[float] = None,
           step: float = 1.0,
           grid_step: float = 0.1,
           clearance: float = 0.25,
           board_edge_clearance: float = 0.55,
           crossing_penalty: float = 10.0,
           length_weight: float = 1.0,
           halo_base: float = 0.5,
           halo_coef: float = 0.25,
           halo_weight: float = 2.0,
           edge_halo: float = 2.0,
           edge_weight: float = 2.0,
           allow_rotations: bool = True,
           allow_swaps: bool = True,
           max_passes: int = 10,
           ignore_nets: Optional[List[str]] = None,
           lock_refs: Optional[List[str]] = None,
           move_refs: Optional[Set[str]] = None,
           net_weights: Optional[Dict[int, float]] = None,
           metrics_out: Optional[Dict] = None,
           groups: Optional[Dict[str, List[str]]] = None,
           align_weight: float = 0.0,
           align_radius: float = 0.5,
           align_span: float = 20.0,
           orient_weight: float = 0.0,
           verbose: bool = False,
           pad_legality: bool = True,
           min_gain_per_mm: float = 0.1,
           move_unconnected: bool = False,
           corridor_weight: float = 0.0,
           corridor_specs: Optional[Sequence[Dict]] = None,
           cancel_check=None,
           progress_callback=None) -> List[Dict]:
    """Greedy quench: iterate over parts, accept only cost-reducing moves.

    align_weight / align_radius / align_span, orient_weight: the #548 tidiness
    terms, BOTH OFF by default. See QuenchState for what they measure and why
    they default to zero -- at 0.0 they return before touching any geometry and
    the output is bit-identical to a build without them.

    corridor_weight / corridor_specs: price the LENGTH each foreign airwire cuts
    through a declared bus corridor, rather than merely counting that it does.
    Off by default -- at 0.0 no corridor is built at all -- and NOT ADOPTED:
    measured on three boards it improved the re-derived corridor signal on one
    of them. The corridors must be frozen at construction (an unfrozen corridor
    makes the objective non-stationary), but a corridor is DEFINED by its bus's
    pads, so moving parts moves the corridor and the minimised gain does not
    survive re-derivation. Read `check_floorplan --health`'s `cut_mm` as a
    diagnostic instead. See docs/placement-optimization.md.

    net_weights: optional {net_id: weight} priority multipliers. A weighted
    net's airwire length is scaled by its weight and every crossing it takes
    part in is priced at max(weight_a, weight_b). Absent or all-ones leaves
    the cost exactly unchanged.

    metrics_out: optional dict, filled in place with the ratsnest and legality
    numbers this function already computes and used to print and discard (#504):

        {'before': {...}, 'after': {...}, 'legality': {...}}

    where before/after are `QuenchState.total_cost()` -- length, crossings,
    halo, edge, hpwl, total -- and legality is `legality_metrics()`. An
    out-param rather than a changed return type on purpose: the return is
    consumed positionally by both CLIs and four test files, and one of those
    binds this signature with inspect.signature. Same note/consume shape as
    plane_resistance's consume_resistance_results (#487).

    Reading them: `crossings` and `hpwl` are UNWEIGHTED (crossings is a raw
    count by contract; hpwl is pure pad geometry), so they are comparable
    across calls. `length` and `total` are scaled by net_weights, so they are
    only comparable between the before/after of the SAME call.

    groups: optional {block name: [reference]} from placement.groups. Each block
    gains a RIGID TRANSLATE move -- the whole body shifts by one offset, capped
    so every member stays within max_displacement of its own seed (#459). Absent
    or empty means no group phase runs and the result is byte-identical to the
    ungrouped engine, which is why grouping is opt-in.

    Returns a list of placement dicts (reference/new_x/new_y/new_rotation)
    for every movable part, whether or not it moved.
    """
    if swap_max_displacement is not None:
        if swap_max_displacement < 0:
            raise ValueError("swap_max_displacement must be >= 0")
        if swap_max_displacement > max_displacement + 1e-9:
            raise ValueError(
                "swap_max_displacement must be <= max_displacement "
                "(each part must stay within max_displacement of its seed)")
    swap_cap = (max_displacement if swap_max_displacement is None
                else swap_max_displacement)

    ignore_net_ids: Set[int] = set()
    if ignore_nets:
        import fnmatch
        for net_id, net in pcb_data.nets.items():
            if any(fnmatch.fnmatch(net.name, pat) for pat in ignore_nets):
                ignore_net_ids.add(net_id)
        print(f"Ignoring {len(ignore_net_ids)} nets for airwire scoring")

    extra_locked: Set[str] = set()
    if lock_refs:
        import fnmatch
        for ref in pcb_data.footprints:
            if any(fnmatch.fnmatch(ref, pat) for pat in lock_refs):
                extra_locked.add(ref)
        print(f"Locked via --lock: {', '.join(sorted(extra_locked))}")

    state = QuenchState(pcb_data, pcb_file, clearance, board_edge_clearance,
                        crossing_penalty, halo_base, halo_coef, halo_weight,
                        edge_halo, edge_weight, grid_step, length_weight,
                        ignore_net_ids=ignore_net_ids,
                        extra_locked_refs=extra_locked,
                        move_refs=move_refs,
                        net_weights=net_weights,
                        align_weight=align_weight, align_radius=align_radius,
                        align_span=align_span, orient_weight=orient_weight,
                        pad_legality=pad_legality,
                        move_unconnected=move_unconnected,
                        corridor_weight=corridor_weight,
                        corridor_specs=corridor_specs)
    state.build_neighbor_lists(max_displacement + grid_step)

    before = state.total_cost()
    print(f"Initial: length={before['length']:.1f}mm "
          f"crossings={before['crossings']} halo={before['halo']:.1f} "
          f"edge={before['edge']:.1f} hpwl={before['hpwl']:.1f}mm "
          f"total={before['total']:.1f}")

    movable = [r for r, p in state.parts.items() if not p.locked]
    movable.sort(key=lambda r: state.parts[r].pin_count, reverse=True)

    # --- placement blocks (#459) ---
    # A block moves as one rigid body, which is the move the per-part nudge
    # cannot express: an IC and its decoupling caps that need to travel together
    # fight each other one part at a time, because moving either alone worsens
    # the pair. Empty unless the caller asked for grouping, and when it is empty
    # the group phase never runs and output is byte-identical to before.
    blocks: Dict[str, List[str]] = {}
    if groups:
        movable_set = set(movable)
        blocks = {name: [r for r in refs if r in movable_set]
                  for name, refs in groups.items()}
        blocks = {n: r for n, r in blocks.items() if len(r) >= 2}
        if blocks and verbose:
            from placement.groups import describe
            print(describe(blocks))

    stopped = False
    for pass_num in range(1, max_passes + 1):
        # COOPERATIVE STOP. This engine had no clock of any kind -- no cancel
        # hook, no progress, no iteration bound but max_passes -- and it is
        # what `place_optimize` and `place_portfolio` run. A 217-part board hung
        # a whole run behind it, indistinguishable from slow work because the
        # only output is one line per COMPLETED pass.
        #
        # A partial is coherent here by construction: apply_move mutates the
        # state in place and the return below reads state.parts, so stopping
        # between parts yields a valid, less-optimised board -- never a torn
        # one. There is no staging step to invalidate.
        if cancel_check is not None and cancel_check():
            print(f"  quench: stopping at pass {pass_num} (budget)")
            break
        improved = 0.0
        moves = 0
        group_moves = 0
        swaps_skipped = 0
        swaps_skipped_shape = 0

        # --- rigid block translation (#459) ---
        # Coarse before fine: a block that wants to be 2mm left is cheaper to fix
        # here than by nudging twenty parts individually, and the per-part pass
        # below then polishes inside the relocated block.
        for name in sorted(blocks):
            refs = blocks[name]
            involved = set()
            for r in refs:
                involved.update(state.parts[r].nets)
            other_aw = state.airwires_excluding(involved)
            member_set = set(refs)

            def eval_group(ddx, ddy):
                ov = {r: state.parts[r].pad_globals(
                    state.parts[r].x + ddx, state.parts[r].y + ddy,
                    state.parts[r].rot) for r in refs}
                subset = {n: state._build_net_airwires(n, overrides=ov)
                          for n in sorted(involved)}
                net_cost, _ = state.nets_cost(subset, other_aw)
                # Intra-group geometry is invariant under a rigid translate, so
                # excluding those pairs both avoids double-counting each internal
                # halo pair and keeps the term comparable across candidates.
                geo = sum(state.part_geometry_cost(
                    r, state.parts[r].x + ddx, state.parts[r].y + ddy,
                    state.parts[r].rot, exclude=member_set - {r}) for r in refs)
                return net_cost + geo

            base_cost = eval_group(0.0, 0.0)
            best = (base_cost, 0.0, 0.0)
            for ddx, ddy in _group_offsets(state, refs, max_displacement, step,
                                           grid_step):
                if not state.group_move_valid(refs, ddx, ddy):
                    continue
                c = eval_group(ddx, ddy)
                if c < best[0] - EPS_IMPROVE:
                    best = (c, ddx, ddy)
            # Displacement-scaled acceptance: motion must buy objective. At 0
            # this is exactly the old EPS_IMPROVE rule.
            if (base_cost - best[0] > EPS_IMPROVE
                    + min_gain_per_mm * math.hypot(best[1], best[2])
                    and (best[1], best[2]) != (0.0, 0.0)):
                improved += base_cost - best[0]
                moves += 1
                group_moves += 1
                state.apply_group_move(refs, best[1], best[2])
                if verbose:
                    print(f"  block {name}: {len(refs)} parts moved "
                          f"({best[1]:+.2f}, {best[2]:+.2f})mm "
                          f"gain={base_cost - best[0]:.1f}")

        # --- single-part moves (nudge + rotate) ---
        for _mi, ref in enumerate(movable):
            # Per PART, not per candidate pose: one monotonic read per
            # violator is the right granularity, and a part
            # costs O(candidates x rotations x parts) so it is a real unit.
            if cancel_check is not None and cancel_check():
                print(f"  quench: stopping mid-pass {pass_num} at "
                      f"{_mi}/{len(movable)} (budget)")
                stopped = True
                break
            if progress_callback is not None:
                progress_callback(_mi, len(movable), f'quench pass {pass_num}')
            part = state.parts[ref]
            involved = set(part.nets)
            other_aw = state.airwires_excluding(involved)

            def eval_at(x, y, rot):
                subset = {n: state._build_net_airwires(
                    n, override_ref=ref,
                    override_pads=part.pad_globals(x, y, rot))
                    for n in involved}
                net_cost, _ = state.nets_cost(subset, other_aw)
                geo_cost = state.part_geometry_cost(ref, x, y, rot)
                return net_cost + geo_cost

            current_cost = eval_at(part.x, part.y, part.rot)
            rotations = _candidate_rotations(part, allow_rotations)
            for rot in rotations:
                # A swap can hand a part an angle from ANOTHER seed's lattice,
                # in a group holding two different non-orthogonal seeds. Add
                # the bounds entry so rect() does not silently fall back to
                # rot-0 geometry. Every such angle is already inside the group
                # closure build_neighbor_lists unioned, so this cannot
                # invalidate the pruning. No-op for orthogonal parts.
                if rot not in part.bounds_by_rot:
                    part.bounds_by_rot[rot] = _rotate_local_bounds(
                        *part.bounds_by_rot[0.0], rot)

            best = (current_cost, part.x, part.y, part.rot)
            for cx, cy in _candidate_positions(part, max_displacement, step,
                                               grid_step):
                for rot in rotations:
                    if (cx, cy, rot) == (part.x, part.y, part.rot):
                        continue
                    if not state.candidate_valid(ref, cx, cy, rot):
                        continue
                    c = eval_at(cx, cy, rot)
                    if c < best[0] - EPS_IMPROVE:
                        best = (c, cx, cy, rot)

            moved_dist = math.hypot(best[1] - part.x, best[2] - part.y)
            rot_charge = 0.5 if best[3] != part.rot else 0.0
            if (current_cost - best[0] > EPS_IMPROVE
                    + min_gain_per_mm * (moved_dist + rot_charge)):
                gain = current_cost - best[0]
                improved += gain
                moves += 1
                state.apply_move(ref, best[1], best[2], best[3])
                if verbose:
                    dx = best[1] - part.seed_x
                    dy = best[2] - part.seed_y
                    print(f"  {ref:>6s}: moved to ({best[1]:.2f}, {best[2]:.2f})"
                          f" rot={best[3]:.0f} (d=({dx:+.1f},{dy:+.1f}))"
                          f" gain={gain:.1f}")

        # --- same-footprint swap moves ---
        if allow_swaps:
            by_fp: Dict[str, List[str]] = {}
            for ref in movable:
                by_fp.setdefault(state.parts[ref].footprint_name, []).append(ref)
            for fp_name, refs in by_fp.items():
                if len(refs) < 2:
                    continue
                # The swap phase is O(n^2) per footprint group and runs AFTER
                # the per-part sweep, so a budget spent above must not be
                # re-spent here.
                if cancel_check is not None and cancel_check():
                    stopped = True
                    break
                for i in range(len(refs)):
                    for j in range(i + 1, len(refs)):
                        ra, rb = refs[i], refs[j]
                        pa, pb = state.parts[ra], state.parts[rb]
                        # A swap exchanges FULL poses, rotation included, so a
                        # mixed-angle pair rotates both parts. --no-rotate
                        # promises that no move changes any part's rotation:
                        # restrict swaps to pairs that already share one, which
                        # also keeps the exchange rotation-neutral and so
                        # preserves the occupied-space invariance that lets
                        # swaps skip candidate_valid. Checked before the cap so
                        # swaps_skipped keeps counting cap rejections only.
                        if not allow_rotations and abs(pa.rot - pb.rot) > 1e-9:
                            continue
                        # Swapping must keep each part within swap_cap of
                        # its OWN seed position
                        if (math.hypot(pb.x - pa.seed_x, pb.y - pa.seed_y) > swap_cap + 1e-9
                                or math.hypot(pa.x - pb.seed_x, pa.y - pb.seed_y) > swap_cap + 1e-9):
                            swaps_skipped += 1
                            continue
                        # A swap exchanges poses within one board side; parts on
                        # opposite sides (or one THT and one not) do not present
                        # the same obstruction, so the occupied-space invariance
                        # that lets swaps skip candidate_valid would not hold.
                        if pa.side != pb.side or pa.has_tht != pb.has_tht:
                            swaps_skipped_shape += 1
                            continue
                        # Courtyards are extracted per-ref, so the same
                        # footprint_name doesn't guarantee identical bounds.
                        # Compared with a tolerance, not ==: two instances of one
                        # library footprint should differ by nothing, and when
                        # they differ by a float wobble refusing the swap costs a
                        # free win silently (#456 item 3 made this fire on
                        # identical parts whose neighbouring silk differed).
                        if not _bounds_match(pa.bounds_by_rot[0.0],
                                             pb.bounds_by_rot[0.0]):
                            swaps_skipped_shape += 1
                            continue
                        # rect() falls back to rot-0 bounds for unknown
                        # rotations; add the partner's rotation lazily so
                        # non-90-degree swaps use correct geometry
                        for p_dst, inherited in ((pa, pb.rot % 360), (pb, pa.rot % 360)):
                            if inherited not in p_dst.bounds_by_rot:
                                p_dst.bounds_by_rot[inherited] = _rotate_local_bounds(
                                    *p_dst.bounds_by_rot[0.0], inherited)
                            if (p_dst.tht_by_rot is not None
                                    and inherited not in p_dst.tht_by_rot):
                                p_dst.tht_by_rot[inherited] = _rotate_local_bounds(
                                    *p_dst.tht_by_rot[0.0], inherited)
                        involved = set(pa.nets) | set(pb.nets)
                        other_aw = state.airwires_excluding(involved)

                        def eval_pair(ax, ay, arot, bx, by, brot):
                            # Two-part override through the shared helper. This
                            # used to be a hand-inlined second copy of
                            # _net_points, carrying a comment warning that the
                            # two must not drift apart; _net_points now takes N
                            # overrides (#459), so there is one implementation
                            # and the sorted net_refs order (#457) is inherited
                            # rather than re-typed.
                            ov = {ra: pa.pad_globals(ax, ay, arot),
                                  rb: pb.pad_globals(bx, by, brot)}
                            subset = {n: state._build_net_airwires(n, overrides=ov)
                                      for n in sorted(involved)}
                            net_cost, _ = state.nets_cost(subset, other_aw)
                            geo = (state.part_geometry_cost(ra, ax, ay, arot,
                                                            exclude={rb})
                                   + state.part_geometry_cost(rb, bx, by, brot,
                                                              exclude={ra})
                                   + state._halo_pair_penalty(
                                       pa, pa.rect(ax, ay, arot),
                                       pb, pb.rect(bx, by, brot),
                                       rects_a=pa.rects(ax, ay, arot),
                                       rects_b=pb.rects(bx, by, brot)))
                            # #548: the a-b ALIGN pair, added back for the same
                            # reason the halo pair above is -- both
                            # part_geometry_cost calls exclude the other part.
                            #
                            # It is tempting to skip this on the grounds that a
                            # swap exchanges two poses so the pair term cancels.
                            # It does not always: rect() centres depend on
                            # rotation when a courtyard is off-centre from the
                            # footprint origin, and a swap exchanges rotations
                            # too.
                            if (state.align_weight > 0.0
                                    and rb in state._peers.get(ra, ())):
                                geo += state._align_pair_penalty(
                                    pa, pa.rect(ax, ay, arot),
                                    pb, pb.rect(bx, by, brot))
                            return net_cost + geo

                        # Pad/hole gate BEFORE the expensive pair evaluation:
                        # identical copper, exchanged nets can short against a
                        # neighbor the old assignment shared a net with.
                        if not state.swap_pads_ok(ra, rb):
                            swaps_skipped_shape += 1
                            continue
                        cur = eval_pair(pa.x, pa.y, pa.rot, pb.x, pb.y, pb.rot)
                        swapped = eval_pair(pb.x, pb.y, pb.rot,
                                            pa.x, pa.y, pa.rot)
                        swap_dist = 2.0 * math.hypot(pa.x - pb.x, pa.y - pb.y)
                        if cur - swapped > EPS_IMPROVE + min_gain_per_mm * swap_dist:
                            gain = cur - swapped
                            improved += gain
                            moves += 1
                            ax, ay, arot = pa.x, pa.y, pa.rot
                            state.apply_move(ra, pb.x, pb.y, pb.rot)
                            state.apply_move(rb, ax, ay, arot)
                            if verbose:
                                da = math.hypot(pa.x - pa.seed_x, pa.y - pa.seed_y)
                                db = math.hypot(pb.x - pb.seed_x, pb.y - pb.seed_y)
                                print(f"  swap {ra} <-> {rb} gain={gain:.1f}"
                                      f" (d[{ra}]={da:.1f}mm, d[{rb}]={db:.1f}mm)")

        stats = state.total_cost()
        group_note = f" blocks={group_moves}" if group_moves else ""
        swap_note = (f" swap-capped={swaps_skipped}"
                     if verbose and swaps_skipped else "")
        # Shape/side mismatches were silent before: two instances of one
        # footprint that never swap look exactly like a pair with nothing to gain.
        if verbose and swaps_skipped_shape:
            swap_note += f" swap-mismatched={swaps_skipped_shape}"
        print(f"Pass {pass_num}: {moves} moves, gain {improved:.1f} -> "
              f"length={stats['length']:.1f}mm crossings={stats['crossings']} "
              f"halo={stats['halo']:.1f} edge={stats['edge']:.1f} "
              f"total={stats['total']:.1f}{group_note}{swap_note}")
        if stopped:
            break
        if moves == 0:
            break

    after = state.total_cost()
    print(f"Quench complete: length {before['length']:.1f} -> {after['length']:.1f}mm, "
          f"crossings {before['crossings']} -> {after['crossings']}, "
          f"hpwl {before['hpwl']:.1f} -> {after['hpwl']:.1f}mm, "
          f"total {before['total']:.1f} -> {after['total']:.1f}")

    if metrics_out is not None:
        # #504: hand back what we just printed, instead of discarding it. The
        # caller owns the dict, so this cannot change the return contract.
        metrics_out['before'] = dict(before)
        metrics_out['after'] = dict(after)
        metrics_out['legality'] = state.legality_metrics()

    return [{'reference': ref,
             'new_x': p.x, 'new_y': p.y, 'new_rotation': p.rot}
            for ref, p in state.parts.items()
            if not p.locked and (p.x != p.seed_x or p.y != p.seed_y
                                 or p.rot != p.orig_rot % 360)]

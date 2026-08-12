"""Structure-level placement reconstruction -- the "puzzle" solver.

For a board whose placement is WRONG (not merely rough): mechanically
determined parts first, then a rigid-displacement hypothesis, then an exact
simultaneous assignment. The pipeline (place_reconstruct.py drives it):

  classify          tiers: locked/statics, zero-net (frozen frame pieces),
                    anchors (large extents), smalls. The user's puzzle order.
  fit_pattern       propose-only: corner-inset fit on zero-net drilled parts
                    (mounting holes). Emits proposed positions, never applies.
                    (Bbox symmetry-transform slates: deferred to v2.)
  rigid_vector      offsets between current and proposed poses, agreeing up
                    to SIGN (a swap's signature), become the +/-v candidate
                    vectors -- reuse of run-2's R4, productized.
  assign            ONE simultaneous solve over each part's small candidate
                    set {stay, +v, -v, proposed slots}: an Assignment Problem
                    with Conflicts as a small ILP (scipy.optimize.milp /
                    HiGHS, in scipy >= 1.9). Colliding candidate pairs are
                    exclusion rows, so the squatter deadlock (a big part
                    evacuated because its home slot is occupied by parts that
                    would only move later) is structurally impossible: the
                    solver trades the squatters' small moves against the big
                    part's placement in one shot. Falls back to a
                    breakout-weighted coordinate descent (Morris, AAAI 1993)
                    without scipy.milp.
  legalize_residue  violation-driven minimal-move sweep (seeder.repair_
                    placement), escalating displacement caps.

Every stage is gated: it is APPLIED only if the lexicographic legality tuple
(pad conflict pairs, hole shortfall, banded pad-oob, stack pairs, hpwl,
courtyard overlap area -- see measure() for the run-4/run-6 ordering
rationale) does not worsen -- the run-2 lesson that a bare conflict-count
gate is gameable by pushing parts off the board, extended in run 6 with the
assembly (stacked-parts) conjunct hpwl-gaming cannot see.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Set, Tuple

from placement import legality

GRID_TOL = 0.05          # pattern-fit residual tolerance (one routing grid)
MOVE_PENALTY_MM = 0.75   # ILP objective: flat cost per moved part (mm-equiv)
PATTERN_BONUS_MM = 2.0   # pattern-proposed slots are EVIDENCE: taking one is
                         # rewarded, not charged (a conflict-free displaced
                         # mounting hole has no other reason to go home)
DIST_TIEBREAK_PER_MM = 0.02
# ^ run-4 F1: nearest-slot tiebreak, scale-free (per mm of displacement).
# Zero-net pattern parts have NO net-anchor cost -- _net_anchor_cost's loop
# body never runs -- so two mounting holes and two free corners were exactly
# cost-degenerate and HiGHS picked arbitrarily: run 3 shipped H1/H3 at each
# other's corners, ~40 mm from home each, mechanically equivalent and
# recovery-visible (worth ~0.16 of recovery on that board). Nearest-slot is
# the minimal-perturbation choice; the weight stays far below
# PATTERN_BONUS_MM and MOVE_PENALTY_MM at real move scales (0.32 mm-equiv
# for a 15.8 mm move), so it can only break ties, never fight the evidence.
# This deliberately REVERSES placement/README.md's earlier position that
# hole-slot degeneracy is acceptable: equivalent to the board is not
# equivalent to recovery.


# --------------------------------------------------------------------------
# measurement -- the stage gate
# --------------------------------------------------------------------------

def _bbox_outside(ext, b) -> float:
    """Per-axis overshoot of the raw board bbox at ZERO margin. Exact on a
    rectangular outline; a consistent lower bound on a notched one -- the
    gate tuple only needs stage-to-stage comparability."""
    return (max(0.0, b[0] - ext[0]) + max(0.0, ext[2] - b[2])
            + max(0.0, b[1] - ext[1]) + max(0.0, ext[3] - b[3]))


def pad_oob_amount(state, edge_bands=None) -> float:
    """Summed pad/hole-extent off-board amount over all parts (zero margin).

    Run-4 F2: a ref declared in ``edge_bands`` ({ref: band_max_mm}, from the
    intent's edge_connectors) charges only the EXCESS past its band -- its
    declared overhang is by design, and without the allowance the gate tuple
    itself would revert a correct edge-homecoming (the move RAISES board-wide
    oob exactly by the legitimate overhang).

    Run-7 finding: this used to measure against the board's BOUNDING RECTANGLE
    while every other instrument used the real Edge.Cuts outline. On a
    non-rectangular board the two disagree completely -- a candidate that swept
    nine parts into a notch measured `oob 0.0` here and 14.29mm everywhere
    else, so the gate accepted an evacuation it was built to reject. The
    legality context already owns the outline-aware measurement (rings,
    cutouts, notches); use it, and keep the bbox form only as the fallback for
    a state that has no context.
    """
    oob = 0.0
    if state.legality_ctx is None:
        return oob
    bands = edge_bands or {}
    ctx = state.legality_ctx
    outline_aware = getattr(ctx, 'gate', None) is not None
    for ref, pp in ctx.parts.items():
        p = state.parts.get(ref)
        if p is None:
            continue
        if outline_aware:
            amt = ctx.pad_oob_amount(ref, p.x, p.y, p.rot, exact=True)
        else:
            ext = pp.extent(p.x, p.y, p.rot)
            if ext is None:
                continue
            amt = _bbox_outside(ext, state.board)
        oob += max(0.0, amt - bands.get(ref, 0.0))
    return oob


def _cross_group_contact(state, groups, snap):
    """First (a, b) newly in contact across two DIFFERENT move groups, or None.

    `groups` maps a multiplier k to the refs moved by k*v, so refs in
    different groups moved by different vectors. `snap` holds their poses
    before the move, used to skip pairs that already conflicted.
    """
    ctx = state.legality_ctx
    if ctx is None or len(groups) < 2:
        return None
    keys = sorted(groups)
    for i, ka in enumerate(keys):
        for kb in keys[i + 1:]:
            for a in sorted(groups[ka]):
                for b in sorted(groups[kb]):
                    sf = ctx.pair_shortfall(a, b)
                    if not (sf.stack or sf.pad > legality.EPS or sf.hole > legality.EPS):
                        continue
                    # Was it already touching before this move? Restore the
                    # two poses, re-ask, put them back.
                    keep = {r: (state.parts[r].x, state.parts[r].y,
                                state.parts[r].rot) for r in (a, b)}
                    for r in (a, b):
                        if r in snap:
                            state.apply_move(r, *snap[r])
                    before = ctx.pair_shortfall(a, b)
                    for r, pose in keep.items():
                        state.apply_move(r, *pose)
                    if not (before.stack or before.pad > legality.EPS
                            or before.hole > legality.EPS):
                        return (a, b)
    return None


#: The gate tuple's terms, in order, as ONE definition.
#:
#: These names used to be repeated in the f-strings that print the gate. When
#: `locked_contacts` was prepended to `measure()`, those f-strings kept their
#: old indices and every term printed under its NEIGHBOUR's name: an executor
#: reading `oob=0.0` was reading the hole shortfall, on a board carrying a part
#: 44 mm off the outline. That matters beyond cosmetics -- the reconstruct
#: ladder's apply rule is "improves the violation count AND does not increase
#: the off-board amount", so the mislabelled term is a hard conjunct someone
#: has to compare by eye. Print via `format_gate` and the labels cannot drift
#: from the tuple again.
GATE_TERMS = ('locked_contacts', 'pad_pairs', 'hole', 'oob', 'stacks',
              'hpwl', 'overlap')


def format_gate(t) -> str:
    """One labelled line for a gate tuple, or a loud one if it has changed."""
    if len(t) != len(GATE_TERMS):
        return (f'GATE TUPLE CHANGED ({len(t)} terms, {len(GATE_TERMS)} names '
                f'-- update GATE_TERMS): ' + ' '.join(f'[{i}]={v!r}'
                                                      for i, v in enumerate(t)))
    return ' '.join(f'{n}={v:g}' if isinstance(v, (int, float)) else f'{n}={v}'
                    for n, v in zip(GATE_TERMS, t))


def measure(state, edge_bands=None) -> Tuple:
    """The lexicographic gate tuple. Smaller-or-equal is acceptable.

    Order (run-8): LOCKED CONTACTS first, then pad conflicts, hole shortfall,
    banded pad-oob, then STACK PAIRS (any-net cross-footprint pad
    intersections -- the assembly channel), then hpwl, then courtyard overlap
    as the LAST tiebreak.

    Locked contacts lead because they are the one term nothing below may buy.
    A locked pose is a decision made outside this toolchain -- a standoff
    against an enclosure, a connector against a panel cut-out -- so a candidate
    that lands copper on one has not found a trade-off, it has broken a
    premise. Placing it first makes it a hard conjunct in the machinery that
    already exists: any candidate that creates one is strictly worse and is
    rejected, whatever it wins on hpwl. On a healthy board the term is 0 and
    the order below it is unchanged (measured: the corpus sweep does not
    move).

    Two measured lessons live in this order. Run 4 demoted the AGGREGATE
    overlap_area below hpwl (it had vetoed a 44 mm hpwl homecoming over
    +0.73 mm^2 of courtyard kiss; run 2 measured it POSITIVELY correlated
    with distance-to-truth, r = +0.72 -- courtyards carry their own margin
    and human boards have a nonzero floor). Run 5 then shipped two 0402s
    STACKED because nothing above hpwl could see them: the stack-pair COUNT
    is the non-gameable per-pair channel (corpus-calibrated ZERO on all 33
    healthy boards, exact and AABB currencies), so it sits ABOVE hpwl where
    the aggregate never can."""
    m = state.pad_legality_metrics() if state.legality_ctx is not None else {}
    leg = state.legality_metrics()
    return (m.get('locked_contact_pairs', 0),
            m.get('pad_conflict_pairs', 0),
            round(m.get('hole_shortfall', 0.0), 4),
            round(pad_oob_amount(state, edge_bands), 4),
            m.get('pad_intersection_pairs', 0),
            round(leg.get('hpwl', 0.0), 3),
            round(leg.get('overlap_area', 0.0), 4))


# --------------------------------------------------------------------------
# classify
# --------------------------------------------------------------------------

class Tiers:
    __slots__ = ('locked', 'zero_net', 'anchors', 'smalls', 'threshold',
                 'edge')

    def as_dict(self):
        return {'locked': sorted(self.locked),
                'zero_net': sorted(self.zero_net),
                'anchors': sorted(self.anchors),
                'smalls': sorted(self.smalls),
                'edge': sorted(self.edge),
                'anchor_extent_mm': self.threshold}


def part_extent_mm(state, ref: str) -> float:
    """Max pad-extent dimension of a part at its current rotation (mm)."""
    if state.legality_ctx is not None:
        pp = state.legality_ctx.parts.get(ref)
        if pp is not None:
            e = pp.extent_local(state.parts[ref].rot)
            if e is not None:
                return max(e[2] - e[0], e[3] - e[1])
    r = state.parts[ref].rect()
    return max(r[2] - r[0], r[3] - r[1])


def classify(state, intent=None, anchor_extent='auto') -> Tiers:
    """The puzzle tiers. Frame first: locked + zero-net (mechanical) parts;
    anchors = large pad extents (edge-connector intent refs always anchors);
    everything else is a small."""
    t = Tiers()
    t.locked = {r for r, p in state.parts.items() if p.locked}
    # A ref the intent declares must_lock belongs in the frame too. This used
    # to read the file's stamps only, so the three placement tools disagreed
    # about the same word: the grader demanded the stamp, the seeder treated it
    # as permission to move, and reconstruct did not look. A hand-written
    # must_lock is a statement that the ref's pose is a decision, which is
    # exactly what the frame tier means.
    if intent is not None and getattr(intent, 'must_lock', None):
        import fnmatch as _fn
        refs_all = sorted(state.parts)
        for pat in intent.must_lock:
            t.locked |= set(_fn.filter(refs_all, pat))
    # UNWIRED, not pin-count-zero (run-10 A1). The frame tier means "this
    # part's position is not a netlist question". `pin_count == 0` reads that
    # off the pad count, which assumes a net-less mounting hole is also
    # PAD-less -- true for a bare NPTH, false for every `MountingHole_*_Pad*`
    # footprint, whose plated pad KiCad gives a real, non-zero net id under an
    # `unconnected-(REF-PadN)` placeholder NAME. `quench` builds `pads_local`
    # with `net_id > 0`, so such a hole counts a pin and misses this tier.
    #
    # Measured on the run-10 subject, whose 9 mounting holes are all of that
    # kind: `zero_net` came out EMPTY, and two things followed silently.
    # `fit_corner_insets` scans `zero_net | locked`, so it saw only the 2
    # holes the FILE happened to lock, found no group of >=2 in distinct
    # corners, and returned {} -- disabling the hole-pattern fit, the one rung
    # that can carry a part an arbitrary distance home. `rigid_vectors` is
    # pattern-gated, so it then had nothing either, and the whole structural
    # ladder was inert while `legalize` (capped at --max-move) was left to do
    # the work alone. The other 7 holes also landed in `smalls`, i.e. free for
    # a search to move a mounting hole.
    #
    # Ask the netlist instead: a pad whose net touches no OTHER part is not a
    # connection -- which is the same >=2-parts rule the routable denominator
    # uses. Strictly wider than the old test (a part with no pads has no nets,
    # so `all(...)` is vacuously true) and it frames only parts nothing wires:
    # on the subject board exactly the 9 holes plus 6 `Z*` mechanical parts,
    # and none of the 11 parts the damage displaced.
    _net_parts: Dict[int, set] = {}
    for _r, _p in state.parts.items():
        for _nid in (getattr(_p, 'nets', ()) or ()):
            _net_parts.setdefault(_nid, set()).add(_r)
    #
    # AND DRILLED, when it has pads at all (run-10 W17). "Nothing wires it" is
    # necessary and not sufficient: on a single-part fixture board the sole
    # part's net-neighbourhood is trivially solo, so the unwired test alone
    # framed the one part the board exists to place -- measured on three
    # in-repo QFN fanout fixtures. A part carrying pads earns the frame only
    # if those pads are DRILLED, which is what separates a plated mounting
    # hole from an unwired SMD part, and is the same `has_tht` filter
    # `fit_corner_insets` applies to whatever this tier hands it. A part with
    # no pads at all keeps the old unconditional pass.
    t.zero_net = {r for r, p in state.parts.items()
                  if r not in t.locked
                  and all(len(_net_parts.get(nid, ())) < 2
                          for nid in (getattr(p, 'nets', ()) or ()))
                  and (p.pin_count == 0 or getattr(p, 'has_tht', False))}
    free = [r for r in state.parts if r not in t.locked | t.zero_net]
    exts = sorted(part_extent_mm(state, r) for r in free)
    if anchor_extent == 'auto':
        p75 = exts[int(0.75 * (len(exts) - 1))] if exts else 3.5
        thr = max(3.5, p75)
    else:
        thr = float(anchor_extent)
    t.threshold = round(thr, 3)
    edge_refs = ({c['ref'] for c in intent.edge_connectors}
                 if intent is not None else set())
    t.edge = edge_refs & set(state.parts)   # run-4 F2: kept, not discarded
    t.anchors = {r for r in free
                 if part_extent_mm(state, r) >= thr or r in edge_refs}
    t.smalls = {r for r in free if r not in t.anchors}
    return t


# --------------------------------------------------------------------------
# fit_pattern (propose-only)
# --------------------------------------------------------------------------

def _slot_on_board(state, x: float, y: float) -> bool:
    """Is a constructed slot actually on the board?

    Slots are built from the bounding box, and a notched or rounded board has
    bbox corners that are not on it. Enumerating those offers a hole a seat it
    could never occupy.
    """
    b = state.board
    if not (b[0] - 1e-6 <= x <= b[2] + 1e-6 and b[1] - 1e-6 <= y <= b[3] + 1e-6):
        return False
    gate = getattr(getattr(state, 'legality_ctx', None), 'gate', None)
    outer = getattr(gate, 'outer', None)
    if not outer:
        return True                       # bbox IS the outline; already checked
    try:
        from check_drc import _point_on_board
        return bool(_point_on_board(x, y, outer, gate.cutouts))
    except Exception:
        return True


def _pattern_slots(state, inset_x: float, inset_y: float, mid_edges: bool):
    """The seats a hole family at this inset can occupy: 4 corners, +4 mid-edges.

    A slot is (edge-perpendicular inset, along-edge position). A CORNER fixes
    the along-position to the inset from the adjacent perpendicular edge; a
    MID-EDGE slot fixes it to that edge's midpoint, at the same perpendicular
    inset. Both are derived from the one inset the survivors over-determine --
    a mid-edge slot invents no new parameter.
    """
    b = state.board
    mx, my = (b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0
    slots = {'SW': (b[0] + inset_x, b[1] + inset_y),
             'NW': (b[0] + inset_x, b[3] - inset_y),
             'SE': (b[2] - inset_x, b[1] + inset_y),
             'NE': (b[2] - inset_x, b[3] - inset_y)}
    if mid_edges:
        slots.update({'W': (b[0] + inset_x, my), 'E': (b[2] - inset_x, my),
                      'S': (mx, b[1] + inset_y), 'N': (mx, b[3] - inset_y)})
    return {k: (round(x, 4), round(y, 4)) for k, (x, y) in slots.items()
            if _slot_on_board(state, x, y)}


def fit_corner_insets(state, tiers: Tiers) -> Dict[str, List[Tuple[float, float]]]:
    """Hole-pattern fit over zero-net DRILLED parts (mounting holes).

    Survivors: holes whose (inset_x, inset_y) agree with a common inset within
    GRID_TOL, in DISTINCT corners. >= 2 survivors over-determine a
    translation; holes seated by no family get every FREE slot of every fitted
    family as proposed positions. Propose-only: the assign stage (or its gate)
    decides.

    Four things this gets right that the corner-only version did not, each
    measured on a board damaged by a 66mm rigid swap:

    EVERY GROUP, not the largest. A board can carry two legitimate hole
    families (one corpus board has {3.302, 3.302} and {7.62, 1.27}). Fitting
    only the largest makes the second family's members read as displaced, and
    they then compete for the first family's free seat -- which is how two
    holes that had never moved became claimants.

    MID-EDGE SLOTS. A hole one inset in from an edge, mid-span along it, is an
    ordinary mounting pattern and the corner model has no hypothesis for it. On
    the measured board the displaced hole's true home is exactly the north
    edge's midpoint at the fitted inset; with no slot there its offset agreed
    with nothing, the support >= 2 rule discarded the one correct offset along
    with three wrong ones, and the whole ladder produced no vector.

    ...but ONLY WHEN OVER-SUBSCRIBED. Mid-edge slots sit nearer the board
    centre than the corners, so on a board whose corner model is sufficient
    they hand DIST_TIEBREAK_PER_MM a closer wrong answer. Enumerate them only
    when the corner model cannot seat everyone: survivors + unseated unlocked
    holes > 4. Counting only UNLOCKED holes is load-bearing -- one corpus board
    carries five locked zero-net through-hole connectors that would otherwise
    inflate demand on every board it appears with.

    AT SEAT IS GLOBAL. A hole matching any fitted family's inset is at seat: it
    is not a candidate, and its slot is taken. Scoping that to one family is
    what let a correct hole be offered another family's corner.
    """
    from collections import defaultdict
    b = state.board
    corners = {'SW': (b[0], b[1]), 'NW': (b[0], b[3]),
               'SE': (b[2], b[1]), 'NE': (b[2], b[3])}
    holes = []
    for ref in sorted(tiers.zero_net | (tiers.locked & set(state.parts))):
        p = state.parts[ref]
        if not p.has_tht:
            continue
        best = min(corners.items(),
                   key=lambda kv: abs(p.x - kv[1][0]) + abs(p.y - kv[1][1]))
        holes.append((ref, best[0], abs(p.x - best[1][0]),
                      abs(p.y - best[1][1])))
    groups = defaultdict(list)
    for ref, corner, ix, iy in holes:
        # PER-AXIS conformance (run-8 A1). This used to demand ix ~= iy, i.e.
        # a SQUARE inset, so an ordinary asymmetric pattern -- a hole 7.62mm
        # in from one edge and 1.27mm from the other, which is just an
        # imperial layout -- never conformed and the fit found nothing at all.
        # Group on the inset PAIR instead: the two axes have to agree across
        # holes, not with each other.
        groups[(round(ix / GRID_TOL), round(iy / GRID_TOL))].append(
            (ref, corner, (ix, iy)))

    fits = []
    for _key, members in sorted(groups.items(), key=lambda kv: -len(kv[1])):
        distinct = {m[1] for m in members}
        if len(members) < 2 or len(distinct) != len(members):
            continue
        fits.append((sum(m[2][0] for m in members) / len(members),
                     sum(m[2][1] for m in members) / len(members),
                     {m[0] for m in members}))
    if not fits:
        return {}

    # RECOGNISE GENEROUSLY, OFFER CONSERVATIVELY.
    #
    # "At seat" is positional -- is this hole standing on a pattern seat? -- and
    # is asked against every slot the families could have, mid-edge included,
    # whether or not those slots are offered to anyone. Asking it about INSETS
    # instead gets a mid-edge hole wrong: its nearest-corner inset is the
    # along-edge distance (99.06mm on the measured board), which matches no
    # family, so a hole standing exactly on its own seat reads as displaced --
    # and the corpus sweep duly caught this proposing a healthy board's hole a
    # move to the position it was already in.
    #
    # Being generous here is the safe direction: it only ever REMOVES
    # candidates. Offering a slot is the risky direction, and that stays behind
    # the pigeonhole gate below.
    every_slot = []
    for inset_x, inset_y, _s in fits:
        every_slot.extend(_pattern_slots(state, inset_x, inset_y, True).values())

    def at_seat(ref) -> bool:
        p = state.parts[ref]
        return any(abs(p.x - sx) <= GRID_TOL and abs(p.y - sy) <= GRID_TOL
                   for sx, sy in every_slot)

    # Demand is board-wide, so a hole seated by family B never inflates family
    # A's pigeonhole count.
    candidates = [ref for ref, _c, _ix, _iy in holes
                  if ref not in tiers.locked and not at_seat(ref)]

    proposals: Dict[str, List[Tuple[float, float]]] = defaultdict(list)
    for inset_x, inset_y, survivors in fits:
        over = len(survivors) + len(candidates) > 4
        slots = _pattern_slots(state, inset_x, inset_y, over)
        taken = {name for name, (sx, sy) in slots.items()
                 if any(abs(state.parts[r].x - sx) <= GRID_TOL
                        and abs(state.parts[r].y - sy) <= GRID_TOL
                        for r, _c, _ix, _iy in holes)}
        free = [xy for name, xy in sorted(slots.items()) if name not in taken]
        for ref in candidates:
            proposals[ref].extend(free)
    return {r: sorted(set(c)) for r, c in proposals.items() if c}


# --------------------------------------------------------------------------
# family orbits -- a DETECTOR, deliberately not a proposal source
# --------------------------------------------------------------------------

#: A family member's radius must agree with the consensus to this (mm).
ORBIT_RADIUS_TOL_MM = 0.20
#: ...and its angular residue with its class to this (degrees).
ORBIT_ANGLE_TOL_DEG = 0.75
#: Fewest members that may define an orbit. Measured: the corpus no-op below
#: holds at 5, 4 and 3 -- 5 is chosen because the fit has four continuous
#: parameters (cx, cy, r, theta0) plus a discrete m, and five members give ten
#: observations.
ORBIT_MIN_INLIERS = 5
#: Rotational orders tried. 2 is excluded by the CURVATURE guard: a 2-fold
#: "orbit" is a point reflection and has no curvature at all, so any two parts
#: anywhere define one.
ORBIT_M_RANGE = range(3, 25)
#: OVER-DETERMINATION: distinct SEATS a residue class must occupy. Seats, not
#: members -- two co-located parts are one piece of evidence about a pattern.
ORBIT_MIN_SEATS_PER_CLASS = 3
#: OCCUPANCY: fraction of the orbit's slots that must actually be filled.
ORBIT_MIN_OCCUPANCY = 0.60
#: SCALE: the fitted circle must be at most this fraction of the board
#: diagonal, and its centre on the board. A near-collinear family fits an
#: enormous circle exactly as a long airwire fits a damage vector.
ORBIT_MAX_RADIUS_FRAC = 0.75
#: Deterministic bound on circumcentre hypotheses per family. A 100-member
#: family has 161700 triples; the stride covers the whole family rather than a
#: prefix of it, so a ring whose refs sort late is not missed.
ORBIT_MAX_HYPOTHESES = 4000


class OrbitFit:
    """One fitted rotational orbit of a footprint family."""
    __slots__ = ('family', 'cx', 'cy', 'r', 'm', 'residues', 'slots',
                 'inliers', 'free_slots')

    def __init__(self, family, cx, cy, r, m, residues, slots, inliers,
                 free_slots):
        self.family = family
        self.cx, self.cy, self.r, self.m = cx, cy, r, m
        self.residues = residues
        self.slots = slots
        self.inliers = inliers
        self.free_slots = free_slots

    def as_dict(self):
        return {'family': self.family,
                'centre': [round(self.cx, 4), round(self.cy, 4)],
                'radius_mm': round(self.r, 4), 'm': self.m,
                'residues_deg': [round(a, 4) for a in self.residues],
                'slots': self.slots, 'inliers': list(self.inliers),
                'free_slots': [[round(x, 4), round(y, 4)]
                               for x, y in self.free_slots]}

    def __repr__(self):                     # pragma: no cover - diagnostics
        return (f'OrbitFit({self.family} m={self.m}x{len(self.residues)} '
                f'r={self.r:.4f} c=({self.cx:.4f},{self.cy:.4f}) '
                f'{len(self.inliers)}/{self.slots})')


def _circumcentre(p1, p2, p3):
    ax, ay = p1
    bx, by = p2
    cx, cy = p3
    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-9:
        return None                     # collinear: no finite circumcentre
    a2, b2, c2 = ax * ax + ay * ay, bx * bx + by * by, cx * cx + cy * cy
    ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d
    uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d
    return (ux, uy)


def fit_family_orbits(state, tiers=None, *,
                      min_inliers: int = ORBIT_MIN_INLIERS
                      ) -> List[OrbitFit]:
    """Footprint families that lie on a regular rotational orbit.

    A DETECTOR and nothing else. It proposes no poses and contributes no gate
    term; its whole product is the fact "this part is AT SEAT on a fitted
    orbit, corroborated by N others". Two consumers want that fact and neither
    needs a proposal: `lock_advisor` (protect an intact array from a legality
    search) and any model that would otherwise claim an at-seat part is
    displaced -- `fit_corner_insets` has no hypothesis for a rotational
    pattern, and on the measured board read H2, standing on an exact 3-fold
    orbit with H1/H3 (radii agreeing to 0.8 nanometres), as displaced and
    offered it a 5.2 mm move.

    THE GUARDS ARE THE DESIGN. Un-guarded this is the shape that got
    `airwire_cluster_vectors` refuted (`:1045`,
    `tests/test_run8_airwire_refuted.py`), so the firing rate on the 33 HEALTHY
    in-repo corpus boards was measured guard by guard BEFORE shipping any of
    it, and `tests/test_orbit_fit_noop.py` pins the whole matrix:

        no guards at all                                 28 of 33 fire
        + scale        (r <= 0.75*diag, centre on board) 28 of 33
        + curvature    (m >= 3)                          28 of 33
        + over-determination (>= 3 distinct SEATS/class) 14 of 33
        + occupancy    (>= 0.60 of slots filled)          7 of 33
        + min_inliers 4                                   2 of 33
        + min_inliers 5   <- SHIPPED                      0 of 33

      scale               r <= ORBIT_MAX_RADIUS_FRAC * board diagonal, centre
                          inside the bbox
      curvature           m >= 3 (a 2-fold orbit is a point reflection)
      over-determination  every residue class occupies >= 3 distinct SEATS
      occupancy           filled seats >= ORBIT_MIN_OCCUPANCY * slots
      min_inliers         >= 5 members on the fitted orbit

    TWO CORRECTIONS TO THE INVESTIGATION THAT PROPOSED THIS, both measured
    here and both in the direction that flattered the proposal:

    1. It reported the un-guarded rate as 6 of 33 and claimed the guarded rate
       stayed 0 with min_inliers lowered to 4 and to 3. Neither reproduces: the
       un-guarded rate is 28 of 33, and relaxing min_inliers to 4 and 3 fires
       on 2 and 7 boards. `min_inliers` is a load-bearing guard here, not a
       formality, and lowering it is a regression rather than a free widening.
    2. Over-determination MUST count distinct seats, not members. Counting
       members, glasgow_revC's six `Fiducial_0.75mm_Mask1.5mm` -- which are
       three positions with a front/back pair at each -- read as six
       corroborations of a 10-fold orbit through three points, and that fit
       passed every other guard. Two co-located parts are one piece of evidence
       about a pattern.

    WHAT THE THRESHOLD COSTS, stated rather than left to be discovered: a
    3- or 4-member orbit is below `min_inliers` and is NOT detected. On the
    measured board that means the exact 3-fold mounting-hole orbit H1/H2/H3
    (radii agreeing to 0.8 nanometres) is invisible here, so this detector does
    NOT refute `fit_corner_insets`' claim that H2 -- which is at seat -- is
    displaced. That refutation needs either a hole-specific relaxation or a
    different model; it is not delivered by this one.

    A near-collinear line of parts is REFUSED, not fitted -- which is correct
    behaviour and not a gap: the case that motivates wanting one (six 0603s at
    1.5 mm pitch, all six displaced together) has no surviving member to anchor
    a line, and a pattern fitter with no survivors must propose nothing.
    Re-seating owns that case (`seeder.reseat_scope`).

    `tiers` is accepted for symmetry with `fit_corner_insets` and is unused:
    an orbit is a property of a family's geometry, not of a part's size tier.
    """
    from collections import defaultdict
    from itertools import combinations

    b = state.board
    diag = math.hypot(b[2] - b[0], b[3] - b[1])
    r_max = ORBIT_MAX_RADIUS_FRAC * diag

    families: Dict[str, List[str]] = defaultdict(list)
    for ref in sorted(state.parts):
        fam = getattr(state.parts[ref], 'footprint_name', None)
        if fam:
            families[fam].append(ref)

    fits: List[OrbitFit] = []
    for fam in sorted(families):
        members = families[fam]
        if len(members) < min_inliers:
            continue
        pts = {r: (state.parts[r].x, state.parts[r].y) for r in members}

        # ---- centre hypotheses: circumcentres of member triples ------------
        total = len(members) * (len(members) - 1) * (len(members) - 2) // 6
        stride = max(1, total // ORBIT_MAX_HYPOTHESES)
        seen_c = set()
        centres = []
        for i, tri in enumerate(combinations(members, 3)):
            if i % stride:
                continue
            c = _circumcentre(*(pts[t] for t in tri))
            if c is None:
                continue
            key = (round(c[0] / 0.01), round(c[1] / 0.01))
            if key in seen_c:
                continue
            seen_c.add(key)
            centres.append(c)

        best = None
        for (cx, cy) in centres:
            if not (b[0] <= cx <= b[2] and b[1] <= cy <= b[3]):
                continue                                        # scale guard
            radii = sorted((math.hypot(pts[r][0] - cx, pts[r][1] - cy), r)
                           for r in members)
            # Consensus radius = the largest run within tolerance.
            i = 0
            while i < len(radii):
                j = i
                while (j + 1 < len(radii)
                       and radii[j + 1][0] - radii[i][0] <= ORBIT_RADIUS_TOL_MM):
                    j += 1
                run = radii[i:j + 1]
                i = j + 1
                if len(run) < min_inliers:
                    continue
                rr = sum(t[0] for t in run) / len(run)
                if rr > r_max or rr < 1e-6:
                    continue                                    # scale guard
                cand = _fit_orbit_m(fam, cx, cy, rr,
                                    [t[1] for t in run], pts, min_inliers)
                if cand is None:
                    continue
                key = (-len(cand.inliers), cand.slots - len(cand.inliers),
                       -cand.m)
                if best is None or key < best[0]:
                    best = (key, cand)
        if best is not None:
            fits.append(best[1])
    return fits


def _fit_orbit_m(family, cx, cy, r, refs, pts, min_inliers):
    """Best rotational order for `refs` on the circle (cx, cy, r), or None."""
    angles = {ref: math.degrees(math.atan2(pts[ref][1] - cy,
                                           pts[ref][0] - cx)) % 360.0
              for ref in refs}
    best = None
    for m in ORBIT_M_RANGE:
        step = 360.0 / m
        # Residue classes: the angle reduced mod the rotational step. A
        # complete orbit of an m-fold pattern with k independent seeds has
        # exactly k classes, each occupied m times.
        buckets: Dict[int, List[str]] = {}
        bucket_res: Dict[int, float] = {}       # each class's REFERENCE residue
        for ref, a in sorted(angles.items()):
            res = a % step
            hit = None
            for key, r0 in bucket_res.items():
                d = abs(res - r0)
                if min(d, step - d) <= ORBIT_ANGLE_TOL_DEG:
                    hit = key
                    break
            if hit is None:
                hit = round(res / ORBIT_ANGLE_TOL_DEG)
                bucket_res[hit] = res
            buckets.setdefault(hit, []).append(ref)
        # OVER-DETERMINATION, counted in SLOTS rather than in members. A
        # residue class seen once or twice is a fitted free parameter, not
        # evidence -- and the members holding it must be at DIFFERENT seats.
        # Measured: glasgow_revC carries six `Fiducial_0.75mm_Mask1.5mm` at
        # three positions (a front/back pair at each), and counting members
        # made 3 distinct points look like 6 corroborations, which was enough
        # for a 10-fold "orbit" through them to survive every other guard. Two
        # co-located parts are one piece of evidence about a pattern.
        #
        # The slot index is taken RELATIVE TO THE CLASS'S REFERENCE RESIDUE,
        # never by flooring the raw angle: a residue that wraps (an angle of
        # 179.99999 deg on a 45 deg step has residue 44.99999, which IS the
        # same class as 0.0) floors into the PREVIOUS slot, so an exact 8-fold
        # ring read as two members sharing a seat and was mis-fitted as 4-fold
        # with two residue classes.
        occupied = {}                   # (class, k) -> refs on that seat
        for k, v in sorted(buckets.items()):
            for ref in v:
                slot = int(round((angles[ref] - bucket_res[k]) / step)) % m
                occupied.setdefault((k, slot), []).append(ref)
        seats_per_class: Dict[int, set] = {}
        for (k, slot) in occupied:
            seats_per_class.setdefault(k, set()).add(slot)
        classes = {k: v for k, v in buckets.items()
                   if len(seats_per_class.get(k, ()))
                   >= ORBIT_MIN_SEATS_PER_CLASS}
        if not classes:
            continue
        inliers = sorted(ref for v in classes.values() for ref in v)
        if len(inliers) < min_inliers:
            continue
        slots = len(classes) * m
        filled = sum(len(seats_per_class[k]) for k in classes)
        # OCCUPANCY: 5 parts spread over a 21-slot orbit is a coincidence, and
        # so are 3 seats of 10.
        if filled < ORBIT_MIN_OCCUPANCY * slots:
            continue
        # Circular mean about the class reference, for the same wrap reason:
        # averaging 0.0 and 44.99999 arithmetically gives 22.5, which is not a
        # residue any member has.
        residues = tuple(sorted(
            (bucket_res[k] + sum(((angles[ref] % step) - bucket_res[k]
                                  + step / 2.0) % step - step / 2.0
                                 for ref in v) / len(v)) % step
            for k, v in classes.items()))
        # Pick the m with the FEWEST FREE SLOTS; ties go to the larger m.
        key = (slots - filled, -m)
        if best is None or key < best[0]:
            free = []
            seated = [angles[ref] for ref in inliers]
            for res in residues:
                for k in range(m):
                    a = (res + k * (360.0 / m)) % 360.0
                    if any(min(abs(a - oa), 360.0 - abs(a - oa))
                           <= ORBIT_ANGLE_TOL_DEG for oa in seated):
                        continue
                    free.append((cx + r * math.cos(math.radians(a)),
                                 cy + r * math.sin(math.radians(a))))
            best = (key, OrbitFit(family, cx, cy, r, m, residues, slots,
                                  tuple(inliers), tuple(sorted(free))))
    return None if best is None else best[1]


def orbit_seats(fits: Sequence['OrbitFit'], min_corroborating: int = 3
                ) -> Dict[str, 'OrbitFit']:
    """{ref: the fit it is AT SEAT on}, for fits with enough corroboration.

    `min_corroborating` counts the OTHER members holding the same orbit, so 3
    means a part plus three others agreeing on centre, radius and pitch. That
    is the threshold at which the fit stops being a description of the part
    itself: four points over-determine a circle's three parameters."""
    out: Dict[str, OrbitFit] = {}
    for f in fits:
        if len(f.inliers) < min_corroborating + 1:
            continue
        for ref in f.inliers:
            out.setdefault(ref, f)
    return out


# --------------------------------------------------------------------------
# rigid_vector
# --------------------------------------------------------------------------

def rigid_vectors(state, proposals: Dict[str, List[Tuple[float, float]]],
                  grid_tol: float = GRID_TOL) -> List[Tuple[float, float]]:
    """Candidate group vectors from the pattern proposals: current - proposed,
    deduped up to SIGN (a swap displaces two groups by +v and -v).

    Run-4 F3(a): a vector is kept only with support from >= 2 DISTINCT refs
    -- R4's own letter ("two or more agree, up to sign"), which the code did
    not implement: every proposal x free-corner pairing minted a vector, and
    run 3's spurious cross-corner vector mis-moved J7 to 31.6 mm from home,
    WORSE than its 15.8 mm input.

    Run-5: the symmetric survivor of that rule is gone too. A symmetric
    cross-pairing (two holes x two corners) self-supports with 2 refs: the
    same refs mapped onto the same slots CROSSED mint a second, spurious
    vector. Competing pairings of the SAME ref-set onto the SAME slot-set
    are one assignment question, and only the minimal-mean-displacement
    pairing is evidence (the F1 nearest-slot principle applied at the
    vector layer); the crossed pairing ships each part ~2x farther. The
    post-assign prune sweep (`prune_assignment`) remains the backstop."""
    support: Dict[Tuple[float, float], Set[str]] = {}
    pairs: Dict[Tuple[float, float],
                List[Tuple[str, Tuple[float, float], float]]] = {}
    order: List[Tuple[float, float]] = []
    for ref, cands in sorted(proposals.items()):
        p = state.parts[ref]
        for (px, py) in cands:
            v = (p.x - px, p.y - py)
            d = math.hypot(*v)
            if d < grid_tol:
                continue
            slot = (round(px, 4), round(py, 4))
            canon = v if (v[1], v[0]) > (0, 0) else (-v[0], -v[1])
            for w in order:
                if math.hypot(canon[0] - w[0], canon[1] - w[1]) <= 2 * grid_tol:
                    support[w].add(ref)
                    pairs[w].append((ref, slot, d))
                    break
            else:
                key = (round(canon[0], 4), round(canon[1], 4))
                order.append(key)
                support[key] = {ref}
                pairs[key] = [(ref, slot, d)]
    kept = [v for v in order if len(support[v]) >= 2]
    groups: Dict[Tuple[frozenset, frozenset], List[Tuple[float, float]]] = {}
    for v in kept:
        gk = (frozenset(r for r, _s, _d in pairs[v]),
              frozenset(s for _r, s, _d in pairs[v]))
        groups.setdefault(gk, []).append(v)
    out = []
    for v in kept:
        gk = (frozenset(r for r, _s, _d in pairs[v]),
              frozenset(s for _r, s, _d in pairs[v]))
        grp = groups[gk]
        if len(grp) > 1:
            best = min(grp, key=lambda w: (
                sum(d for _r, _s, d in pairs[w]) / len(pairs[w]), w))
            if v != best:
                continue
        out.append(v)
    return out


EDGE_PREF_WEIGHT = 10.0
# ^ run-4 F2: mm-equiv charged per mm an edge-class part's pose sits from an
# edge, INSIDE the joint solve. Offering a displaced receptacle its band
# candidate is not enough (measured: J1's candidate was offered and never
# taken -- the net-anchor proxy pulls toward its partners, themselves
# misplaced), and seating it AFTER the solve fails too (measured: the seat
# collided with squatters the joint solve would have co-moved: 5 pad pairs,
# overlap +70). R1's letter -- an edge part's position is not a netlist
# question -- so for declared edge-less receptacles the edge metric REPLACES
# the net-anchor cost in the objective, and the exclusion rows resolve the
# squatters in the same solve. Generic: the weight scales a geometric metric,
# no board constants.


def edge_metric(state, ref: str, x: float, y: float,
                band: float) -> Optional[float]:
    """How far a pose is from being edge-seated, courtyard channel: 0 for an
    in-band overhang, else the courtyard edge clearance; None = past the
    band (illegal as a seat)."""
    p = state.parts[ref]
    rect = p.rect(x, y, p.rot)
    over = state.edge_gate.rect_outside_amount(rect)
    if over > 1e-6:
        return 0.0 if over <= band + 1e-6 else None
    return state.edge_gate.edge_clearance(rect)


def damage_witnesses(state) -> Dict[str, str]:
    """Refs carrying a NAMED structural witness that they are misplaced.

    Not a vector source. The off-board idea was measured as one and refuted:
    containment leaves a box 30x11mm wide with sixty conflict-free candidate
    vectors, hpwl's minimum among them sits 7.6mm from the truth and scores the
    truth WORSE, and on a swap the joint box is infeasible. Same shape as the
    airwire refutation -- sound detector, ordinary-layout chooser.

    As a WITNESS it is sound, and the no-op is a property rather than a
    threshold: a pad centre off the outline is the negation of a
    manufacturability invariant, because you cannot solder to air. Measured
    across 33 corpus boards plus 5 controls: ZERO witnesses.

    The form matters and only one form has that guarantee. Measured on the same
    boards, a pad-EXTENT bounding box against the outline fires on a healthy
    module wider than its own board, and against a ROUNDED outline it fires on
    four locked mounting holes whose round pads are entirely inside but whose
    AABB corners clear the corner arc. Pad CENTRES fire on none of them.
    """
    out: Dict[str, str] = {}
    ctx = getattr(state, 'legality_ctx', None)
    if ctx is None:
        return out
    for ref, pp in sorted((ctx.parts or {}).items()):
        p = state.parts.get(ref)
        if p is None:
            continue
        try:
            rects = pp.pad_rects(p.x, p.y, p.rot)
        except Exception:                                   # noqa: BLE001
            continue
        for rect in rects or ():
            x0, y0, x1, y1 = rect[0], rect[1], rect[2], rect[3]
            if not _slot_on_board(state, (x0 + x1) / 2.0, (y0 + y1) / 2.0):
                out[ref] = 'off_board'
                break
    return out


def evidenced_moves(state, old: Dict[str, Tuple[float, float, float]],
                    vectors, tol: float = 2 * GRID_TOL) -> Set[str]:
    """Refs whose displacement AGREES with a kept vector, up to sign and step.

    The damage hypothesis is "parts displaced by k*v"; a move that matches it
    is evidence, and a move that matches nothing is a seat the solver liked.
    Both look identical to a board-wide gate tuple when the part carries no
    net, which is why this exists as a separate question from the tuple.
    """
    out: Set[str] = set()
    steps = (-2, -1, 1, 2)
    for ref, (x, y, _r) in (old or {}).items():
        p = state.parts.get(ref)
        if p is None:
            continue
        dx, dy = p.x - x, p.y - y
        if math.hypot(dx, dy) < GRID_TOL:
            continue                    # did not move; nothing to justify
        for vx, vy in (vectors or ()):
            if any(math.hypot(dx - k * vx, dy - k * vy) <= tol
                   for k in steps):
                out.add(ref)
                break
    return out


def prune_assignment(state, old: Dict[str, Tuple[float, float, float]],
                     notes: Optional[List[str]] = None,
                     edge_bands: Optional[Dict[str, float]] = None,
                     exempt: Optional[Set[str]] = None,
                     evidenced: Optional[Set[str]] = None) -> List[str]:
    """Per-part revert sweep after an ACCEPTED assignment (run-4 F3b).

    The stage gate is one board-wide lexicographic tuple, so an assignment
    that repairs the frame can smuggle individual mis-moves past it -- run
    3's spurious vector carried J7 to 31.6 mm from home (worse than its
    input) inside a hugely-improving move set, and the global gate cannot
    see per-part worsening. Walk the moved parts by DESCENDING displacement,
    tentatively restore each input pose, and keep the revert iff the gate
    tuple STRICTLY improves. Board-only, monotone by construction: the
    sweep can only improve the tuple, and a revert that would reintroduce a
    conflict is rejected by the same tuple.

    `evidenced` names the refs whose chosen pose came from real evidence -- a
    corroborated pattern slot, or a +/-v offset of a kept vector. For anything
    NOT in that set, an EQUAL tuple is also grounds to revert, and that is the
    hole a measured 26.27 mm move went through: a mounting hole was carried to
    another hole's seat, the gate tuple came out byte-identical (a zero-net
    part in free space touches no term the tuple has), so `< base` never fired
    and the move survived a sweep written to catch exactly that. A move nothing
    measured justifies is not a tie to be broken in its favour.

    Strictness is deliberately NOT applied to the round accept upstream: a
    displaced hole coming home is gate-neutral by construction for the same
    reason, so requiring strict improvement there would reject the homecoming
    the whole fit exists to produce.
    """
    evidenced = evidenced or set()
    pruned: List[str] = []

    def moved_dist(item):
        ref, (x, y, _r) = item
        p = state.parts[ref]
        return math.hypot(p.x - x, p.y - y)

    for ref, (x, y, rot) in sorted(old.items(), key=moved_dist, reverse=True):
        if exempt and ref in exempt:
            # F2: an edge-class seat is hpwl-worse BY DESIGN (the netlist
            # proxy is what its class overrules) -- pruning it back would
            # undo the seat one stage later.
            continue
        p = state.parts[ref]
        if math.hypot(p.x - x, p.y - y) < 1e-9 and abs(p.rot - rot) < 1e-9:
            continue
        base = measure(state, edge_bands)
        cur = (p.x, p.y, p.rot)
        state.apply_move(ref, x, y, rot)
        after = measure(state, edge_bands)
        if after < base or (after == base and ref not in evidenced):
            pruned.append(ref)
        else:
            state.apply_move(ref, *cur)
    if notes is not None and pruned:
        notes.append(f"prune: reverted {len(pruned)} per-part mis-move(s) "
                     f"the global gate could not see: {', '.join(pruned)}")
    return pruned


# --------------------------------------------------------------------------
# exchange -- the joint +/-v lattice ILP (run-5 F4)
# --------------------------------------------------------------------------

EXCHANGE_LATTICE_TOL = 0.1
# ^ how close current-minus-input must sit to an integer multiple of the
# pattern vector for a part to be on the damage lattice (mm). Parts off the
# lattice (seeded elsewhere, nudged by another tool) are stay-only.


def _exchange_solve(state, v, exclude, edge_bands):
    """One joint solve of the exchange ILP for pattern vector ``v``.

    Every eligible part chooses an option k with pose = current + k*v,
    where the options are the +/-1 V-LATTICE AROUND ITS INPUT POSE
    (seed_x/seed_y): j_cur = round offset of current from input in v units;
    k is offered iff j_cur + k is in {-1, 0, +1} and the target pad extent
    stays on-board (banded). Rationale, all measured on the swap corpus:

    - The damage hypothesis is "parts displaced by +/-v"; poses more than
      one lattice step from the INPUT are not reachable by any such
      hypothesis. Offering current-relative +/-2 instead let the solver
      "improve" hpwl by relocating 13 never-displaced parts 31.6 mm out --
      board-only metrics all improved while correct structure broke, and
      no gate conjunct can see that. Input-anchoring makes every such move
      structurally unreachable while still letting the solver UNWIND or
      COMPLETE a part the ladder itself mis-moved (a part at input-v may
      take +2: undo the mis-move and come home).
    - Constraints are pairwise-absolute pad/hole legality on every option
      combo (skipping stay-stay: the status quo stays feasible), so a
      solution can only remove or keep existing conflicts, never add one.
    - The objective is EXACT per-net bbox hpwl at the chosen poses
      (linearized: per-net min/max vars over option-shifted pad bboxes)
      plus MOVE_PENALTY_MM per moved part. A pairwise spring proxy was
      measured WRONG here (valued the true exchange at -0.56 while real
      hpwl gained 60): only the gate's own currency can price a joint
      move.

    Returns {ref: k} for the chosen non-stay options ({} = all-stay or no
    solver).
    """
    try:
        import numpy as np
        from scipy.optimize import milp, LinearConstraint, Bounds
    except ImportError:
        return None
    vx, vy = v
    bands = edge_bands or {}
    excl = exclude or set()
    ctx = state.legality_ctx
    m = state.clearance + 0.1

    opts: Dict[str, List[int]] = {}
    for r, p in sorted(state.parts.items()):
        if r in excl or p.locked:
            opts[r] = [0]
            continue
        dx0, dy0 = p.x - p.seed_x, p.y - p.seed_y
        j_cur = None
        for j in (-1, 0, 1):
            if math.hypot(dx0 - j * vx, dy0 - j * vy) <= EXCHANGE_LATTICE_TOL:
                j_cur = j
                break
        if j_cur is None:
            opts[r] = [0]
            continue
        ss = [0]
        pp = ctx.parts.get(r)
        for k in (-2, -1, 1, 2):
            if j_cur + k not in (-1, 0, 1):
                continue
            ext = (pp.extent(p.x + k * vx, p.y + k * vy, p.rot)
                   if pp is not None else None)
            if ext is None or (_bbox_outside(ext, state.board)
                               <= bands.get(r, 0.0) + 1e-6):
                ss.append(k)
        opts[r] = ss

    refs = sorted(opts)
    ext_of = {}
    for r in refs:
        p = state.parts[r]
        pp = ctx.parts.get(r)
        for k in opts[r]:
            ext_of[(r, k)] = (pp.extent(p.x + k * vx, p.y + k * vy, p.rot)
                              if pp is not None else None)

    def near(ea, eb):
        return (ea is None or eb is None
                or (ea[2] + m >= eb[0] and eb[2] + m >= ea[0]
                    and ea[3] + m >= eb[1] and eb[3] + m >= ea[1]))

    conflict = []
    for i, a in enumerate(refs):
        for b in refs[i + 1:]:
            if len(opts[a]) == 1 and len(opts[b]) == 1:
                continue
            pa, pb = state.parts[a], state.parts[b]
            for ka in opts[a]:
                for kb in opts[b]:
                    if ka == 0 and kb == 0:
                        continue
                    if not near(ext_of[(a, ka)], ext_of[(b, kb)]):
                        continue
                    if _pair_conflicts(
                            state, a, (pa.x + ka * vx, pa.y + ka * vy),
                            b, (pb.x + kb * vx, pb.y + kb * vy)):
                        conflict.append((a, ka, b, kb))

    xidx = {}
    for r in refs:
        for k in opts[r]:
            xidx[(r, k)] = len(xidx)
    nx = len(xidx)

    nets: Dict[int, Dict[str, List[float]]] = {}
    for r, p in state.parts.items():
        for gx, gy, nid in p.pad_globals():
            d = nets.setdefault(nid, {})
            bb = d.get(r)
            if bb is None:
                d[r] = [gx, gy, gx, gy]
            else:
                bb[0] = min(bb[0], gx)
                bb[1] = min(bb[1], gy)
                bb[2] = max(bb[2], gx)
                bb[3] = max(bb[3], gy)
    net_ids = sorted(n for n, d in nets.items() if len(d) >= 2)

    nv = nx + 4 * len(net_ids)
    cost = np.zeros(nv)
    lb = np.zeros(nv)
    ub = np.ones(nv)
    for kk in range(len(net_ids)):
        for j, sgn in ((0, 1.0), (1, -1.0), (2, 1.0), (3, -1.0)):
            i = nx + 4 * kk + j
            cost[i] = sgn
            lb[i] = -np.inf
            ub[i] = np.inf
    for (r, k), i in xidx.items():
        if k != 0:
            cost[i] += MOVE_PENALTY_MM

    rows = []
    rl = []
    ru = []
    for r in refs:
        row = np.zeros(nv)
        for k in opts[r]:
            row[xidx[(r, k)]] = 1.0
        rows.append(row)
        rl.append(1.0)
        ru.append(1.0)
    for (a, ka, b, kb) in conflict:
        row = np.zeros(nv)
        row[xidx[(a, ka)]] = 1.0
        row[xidx[(b, kb)]] = 1.0
        rows.append(row)
        rl.append(-np.inf)
        ru.append(1.0)
    for kk, nid in enumerate(net_ids):
        iXmax, iXmin = nx + 4 * kk, nx + 4 * kk + 1
        iYmax, iYmin = nx + 4 * kk + 2, nx + 4 * kk + 3
        for r, bb in sorted(nets[nid].items()):
            kx = [(xidx[(r, k)], k * vx) for k in opts.get(r, [0]) if k != 0]
            ky = [(xidx[(r, k)], k * vy) for k in opts.get(r, [0]) if k != 0]
            for (idx, bound, coefs, upper) in (
                    (iXmax, bb[2], kx, True), (iXmin, bb[0], kx, False),
                    (iYmax, bb[3], ky, True), (iYmin, bb[1], ky, False)):
                row = np.zeros(nv)
                row[idx] = 1.0
                for i, c_ in coefs:
                    row[i] = -c_
                rows.append(row)
                if upper:
                    rl.append(bound)
                    ru.append(np.inf)
                else:
                    rl.append(-np.inf)
                    ru.append(bound)

    A = np.vstack(rows)
    integ = np.zeros(nv)
    integ[:nx] = 1
    res = milp(c=cost, constraints=LinearConstraint(A, rl, ru),
               integrality=integ, bounds=Bounds(lb, ub))
    if res.x is None:
        return {}
    return {r: k for (r, k), i in xidx.items() if res.x[i] > 0.5 and k != 0}


def exchange_stage(state, vectors: Sequence[Tuple[float, float]],
                   exclude: Optional[Set[str]] = None,
                   edge_bands: Optional[Dict[str, float]] = None,
                   notes: Optional[List[str]] = None,
                   max_iter: int = 10):
    """Gated joint +/-v group exchange along the pattern vectors.

    The measured limit this removes (run-4 F4): a self-consistent displaced
    ISLAND -- every member conflict-free where it sits, net-anchored to the
    other members -- presents no per-part gradient, so the assign ILP's
    static anchor proxy prices every member against stay and the
    simultaneous exchange is invisible. Every local mechanism probed on the
    swap corpus failed measurably (single-sign translate sets cascade to
    nothing against the OPPOSITE group's squatters; seeded best-response
    has a wrong attractor -- home parts chasing displaced partners beat
    the true exchange 315 to 60 in raw hpwl; interlock 2-coloring floods).
    Only the JOINT solve works: one ILP over the +/-1 input-lattice with
    absolute pairwise legality rows and the exact bbox-hpwl objective
    (see _exchange_solve).

    Each vector iterates solve -> apply -> measure-gate -> until all-stay
    or a gate rejection (exact snapshot restore). Healthy-board no-op by
    construction: candidate vectors exist only where the corner-pattern
    fit found displaced pattern parts (rigid_vectors is pattern-gated).

    Returns (report, old_poses): old_poses maps each moved ref to its
    pre-move pose, in prune_assignment's format.
    """
    report: Dict = {'attempts': 0, 'accepted': [], 'solver': True}
    old: Dict[str, Tuple[float, float, float]] = {}
    if not vectors:
        return report, old
    base = measure(state, edge_bands)
    for (vx, vy) in vectors:
        for _ in range(max_iter):
            report['attempts'] += 1
            sigma = _exchange_solve(state, (vx, vy), exclude, edge_bands)
            if sigma is None:
                report['solver'] = False
                if notes is not None:
                    notes.append('exchange: scipy.optimize.milp unavailable'
                                 ' -- stage skipped')
                return report, old
            if not sigma:
                break
            groups: Dict[int, List[str]] = {}
            for r, k in sigma.items():
                groups.setdefault(k, []).append(r)
            snap = {r: (state.parts[r].x, state.parts[r].y,
                        state.parts[r].rot) for r in sigma}
            for k, refs_ in sorted(groups.items()):
                state.apply_group_move(sorted(refs_), k * vx, k * vy)
            after = measure(state, edge_bands)
            # E7, in the one place the engine KNOWS the applied vectors.
            # Members displaced by the same k*v keep their relative geometry
            # exactly, so they cannot newly touch each other; a new contact
            # between two members of DIFFERENT groups means this assignment is
            # not a rigid restore but a search result that happens to fit.
            # Checked only across groups, and only for pairs that were clear
            # before -- a conflict the input already had is not this move's.
            cross_group_hit = _cross_group_contact(state, groups, snap)
            if cross_group_hit and notes is not None:
                notes.append(
                    f'exchange: REJECTED an assignment along '
                    f'({vx:.3f}, {vy:.3f}) -- it puts {cross_group_hit[0]} '
                    f'and {cross_group_hit[1]} in contact although they moved '
                    f'by different vectors, which a rigid restore cannot do')
            # STRICT improvement: an exchange the tuple cannot see is churn.
            if after < base and not cross_group_hit:
                base = after
                old.update(snap)
                report['accepted'].append(
                    {'v': [round(vx, 4), round(vy, 4)],
                     'moves': {str(k): sorted(refs_)
                               for k, refs_ in sorted(groups.items())},
                     'gate': list(after)})
                if notes is not None:
                    notes.append(
                        'exchange: '
                        + ' / '.join(f'{len(refs_)} part(s) by {k:+d}v'
                                     for k, refs_ in sorted(groups.items()))
                        + f' along ({vx:.3f}, {vy:.3f})')
            else:
                # exact snapshot restore -- apply_group_move's `+= dx`
                # would leave float residue on an inverse move
                for r, (x, y, rot) in snap.items():
                    state.apply_move(r, x, y, rot)
                break
    return report, old


# --------------------------------------------------------------------------
# assign -- the ILP
# --------------------------------------------------------------------------

def _net_anchor_cost(state, ref: str, x: float, y: float,
                     fixed: Set[str]) -> float:
    """Linear wirelength proxy: distance from the pose to each of the part's
    nets' centroids over OTHER parts' current poses (those parts move too, so
    this is a proxy -- the exclusions carry the hard constraints; this breaks
    ties). `fixed` weights: a centroid built only from frame parts would be
    empty on most nets, so all others count, frame parts double."""
    part = state.parts[ref]
    cost = 0.0
    for net in part.nets:
        sx = sy = w = 0.0
        for other in state.net_refs.get(net, ()):
            if other == ref:
                continue
            op = state.parts[other]
            ww = 2.0 if other in fixed else 1.0
            sx += ww * op.x
            sy += ww * op.y
            w += ww
        if w > 0:
            cost += math.hypot(x - sx / w, y - sy / w)
    return cost


def conflict_offset_vectors(state, *, cluster_tol: float = 1.5,
                            min_support: int = 3,
                            top_k: int = 3) -> List[Dict]:
    """Candidate rigid-translate vectors from CONFLICT-PAIR offsets (run-7
    A1: the vector source for boards with no mounting-hole pattern).

    The geometry: a translate-displaced part lands ON or NEAR stationary
    geometry (stacks, pad conflicts), and for every such pair the offset
    a.pos - b.pos approximates the damage vector to within ~part size --
    so the offsets of the damage's conflict pairs CLUSTER while a dense
    design's own incidental conflicts scatter. Clusters with
    ``min_support`` pairs (canonicalized up to sign, like rigid_vectors)
    become candidate vectors, largest support first.

    Design history, measured: the first A1 attempt fit block vectors from
    net-anchor targets and was abandoned -- healthy boards read up to 60mm
    of pure design slack (a part legitimately sits far from its net
    centroid) and healthy-vs-damaged rp2350 were indistinguishable
    (12.9 vs 12.0). Conflict offsets have the property that matters
    instead: NO conflicts, NO vectors -- the healthy-board no-op guarantee
    is structural, not a threshold.

    The vectors are COARSE (part-size error), which the consumers absorb:
    build_candidates offers +/-v poses whose residual mis-fit the
    legalize/repair rungs clean, and the exchange lattice stays consistent
    because the same vector list is used for the whole run.
    """
    ctx = state.legality_ctx
    if ctx is None:
        return []
    refs = sorted(state.parts)
    offsets = []
    for i, a in enumerate(refs):
        pa = state.parts[a]
        for b in refs[i + 1:]:
            pb = state.parts[b]
            sf = ctx.pair_shortfall(a, b)
            # STACK/HOLE pairs only -- these are clearance-independent
            # (physical intersection), so the no-op guarantee holds at ANY
            # grading clearance. Clearance-SHORTFALL pairs are excluded:
            # packed healthy designs read shortfalls at a too-strict
            # clearance (measured: the corpus sweep at the 0.25 default
            # minted a vector on a healthy board and moved it).
            if not (sf.stack or sf.hole > legality.EPS):
                continue
            offsets.append((pa.x - pb.x, pa.y - pb.y))
    clusters: List[List[Tuple[float, float]]] = []
    for (dx, dy) in offsets:
        if math.hypot(dx, dy) < 2.0:
            # small offsets are intra-pile NEIGHBOR pairs (two displaced
            # parts conflicting each other at design-scale spacing), not
            # displaced-onto-stationary evidence -- measured: they dominated
            # the clusters at ~1mm and drowned the damage vector
            continue
        canon = (dx, dy) if (dy, dx) > (0, 0) else (-dx, -dy)
        for cl in clusters:
            cx = sum(p[0] for p in cl) / len(cl)
            cy = sum(p[1] for p in cl) / len(cl)
            if math.hypot(canon[0] - cx, canon[1] - cy) <= cluster_tol:
                cl.append(canon)
                break
        else:
            clusters.append([canon])
    out = []
    for cl in clusters:
        if len(cl) < min_support:
            continue
        vx = sum(p[0] for p in cl) / len(cl)
        vy = sum(p[1] for p in cl) / len(cl)
        out.append({'v': (round(vx, 4), round(vy, 4)), 'support': len(cl)})
    out.sort(key=lambda d: -d['support'])
    return out[:top_k]





def airwire_cluster_vectors(state, *, cluster_tol: float = 1.5,
                            min_support: int = 3, top_k: int = 3,
                            min_airwire_mm: float = 8.0,
                            max_net_pads: int = 4) -> List[Dict]:
    """REFUTED (run-8 E5). Kept, unwired, as the record of a measurement.

    The proposal, endorsed after run 7: a third vector source for the case the
    other two cannot see -- a translate that produced no conflicts and left no
    hole pattern to fit. On a small net a displaced pad's airwire to its
    partners is stretched by roughly the damage vector, so those stretch
    vectors should CLUSTER on the damage while a healthy design's long
    airwires point in unrelated directions.

    They do not. Measured over the 33 in-repo boards and two recorded
    perturbed ones, at the endorsed filters (nets of 2-4 pads, airwires over
    8mm, three agreeing within 1.5mm):

        healthy boards firing          6 of 33, with support up to 112
        a board translated by (4.5, -2.4), |v| = 5.1
                                       top cluster (6.7, 21.4), support 28
                                       -- not the damage, and not close

    The reason is the same one that killed the first A1 design (net-anchor
    least squares, which read up to 60mm of pure design slack on healthy
    boards): a long airwire is ORDINARY LAYOUT. A net legitimately spanning
    the board produces exactly the signal this looks for, in quantity, and no
    threshold separates it from damage because there is no separation to find.
    Restricting to small nets narrows the population without changing its
    nature.

    What the two SHIPPED sources have and this does not is a structural no-op
    guarantee. A hole-pattern fit needs a hole pattern; conflict offsets need
    conflicts. Both are silent on a healthy board by construction rather than
    by threshold. Connectivity has no such property.

    Left in the tree, not called: tests/test_run8_airwire_refuted.py pins these
    numbers so a future attempt starts from the measurement instead of
    repeating it, and so a change that alters the behaviour is visible.
    """
    parts = state.parts
    pads_by_net: Dict[int, List[Tuple[str, float, float]]] = {}
    for ref in sorted(parts):
        for nid in sorted(getattr(parts[ref], 'nets', ()) or ()):
            pads_by_net.setdefault(nid, []).append(
                (ref, parts[ref].x, parts[ref].y))
    stretches = []
    for nid, members in sorted(pads_by_net.items()):
        if nid <= 0 or not (2 <= len(members) <= max_net_pads):
            continue
        for i, (ra, ax, ay) in enumerate(members):
            for (rb, bx, by) in members[i + 1:]:
                if ra == rb:
                    continue
                dx, dy = ax - bx, ay - by
                if math.hypot(dx, dy) < min_airwire_mm:
                    continue
                stretches.append((dx, dy))
    clusters: List[List[Tuple[float, float]]] = []
    for (dx, dy) in stretches:
        canon = (dx, dy) if (dy, dx) > (0, 0) else (-dx, -dy)
        for cl in clusters:
            cx = sum(p[0] for p in cl) / len(cl)
            cy = sum(p[1] for p in cl) / len(cl)
            if math.hypot(canon[0] - cx, canon[1] - cy) <= cluster_tol:
                cl.append(canon)
                break
        else:
            clusters.append([canon])
    out = []
    for cl in clusters:
        if len(cl) < min_support:
            continue
        vx = sum(p[0] for p in cl) / len(cl)
        vy = sum(p[1] for p in cl) / len(cl)
        out.append({'v': (round(vx, 4), round(vy, 4)), 'support': len(cl)})
    out.sort(key=lambda d: -d['support'])
    return out[:top_k]


def build_candidates(state, tiers: Tiers,
                     vectors: Sequence[Tuple[float, float]],
                     proposals: Dict[str, List[Tuple[float, float]]],
                     edge_bands: Optional[Dict[str, float]] = None
                     ) -> Dict[str, List[Tuple[float, float]]]:
    """Per-part candidate positions: stay (always index 0), +/-each vector
    (kept only when the pad extent stays on-board), and any pattern-proposed
    slots. Locked parts get only stay.

    Run-4 F2: the anti-evacuation cull compares each candidate's off-board
    amount against the part's CURRENT pose -- and a DISPLACED edge part sits
    interior (cur_oob = 0), so its true home, which overhangs BY DESIGN, was
    culled unconditionally (run 3's J1 was never offered its edge slot).
    Refs declared in ``edge_bands`` ({ref: band_max_mm}) may overhang up to
    their band; 'stay' stays index 0 -- a declaration never forces a move.
    """
    out: Dict[str, List[Tuple[float, float]]] = {}
    pattern: Dict[str, Set[Tuple[float, float]]] = {}
    bands = edge_bands or {}
    for ref in sorted(state.parts):
        p = state.parts[ref]
        cands = [(p.x, p.y)]
        if not p.locked:
            for (vx, vy) in vectors:
                for sx, sy in ((vx, vy), (-vx, -vy)):
                    cands.append((round(p.x + sx, 4), round(p.y + sy, 4)))
            for slot in proposals.get(ref, ()):
                pattern.setdefault(ref, set()).add(slot)
                if slot not in cands:
                    cands.append(slot)
        kept = []
        pp = (state.legality_ctx.parts.get(ref)
              if state.legality_ctx is not None else None)
        cur_oob = None
        for i, (x, y) in enumerate(cands):
            if i > 0 and pp is not None:
                ext = pp.extent(x, y, p.rot)
                if ext is not None:
                    if cur_oob is None:
                        e0 = pp.extent(p.x, p.y, p.rot)
                        cur_oob = (_bbox_outside(e0, state.board)
                                   if e0 is not None else 0.0)
                    # Never offer a candidate whose pad copper leaves the
                    # board MORE than the part already does (S1: the
                    # conflict gate must not be satisfiable by evacuation)
                    # -- except up to a declared edge band (run-4 F2).
                    allow = max(cur_oob, bands.get(ref, 0.0))
                    if _bbox_outside(ext, state.board) > allow + 1e-6:
                        continue
            kept.append((x, y))
        out[ref] = kept
    return out, pattern


def solve_assignment(state, candidates: Dict[str, List[Tuple[float, float]]],
                     tiers: Tiers,
                     move_penalty: float = MOVE_PENALTY_MM,
                     notes: Optional[List[str]] = None,
                     pattern: Optional[Dict[str, Set]] = None,
                     edge_pref: Optional[Dict[str, float]] = None
                     ) -> Dict[str, int]:
    """Choose one candidate per part. Exact ILP when scipy.optimize.milp is
    available; breakout-weighted min-conflicts descent otherwise. Returns
    {ref: chosen index}.

    ``edge_pref`` ({ref: band_max_mm}): declared edge-class refs whose edge
    could not be named -- their objective term is the EDGE metric instead of
    the net-anchor proxy (see EDGE_PREF_WEIGHT)."""
    try:
        return _solve_ilp(state, candidates, tiers, move_penalty, notes,
                          pattern or {}, edge_pref or {})
    except ImportError:
        if notes is not None:
            notes.append('scipy.optimize.milp unavailable -- using the '
                         'breakout-descent fallback')
        return _solve_breakout(state, candidates, tiers, notes,
                               edge_pref=edge_pref or {})


def _pair_conflicts(state, a: str, pos_a, b: str, pos_b) -> bool:
    """Do these two candidate poses conflict, ABSOLUTELY? Repair semantics:
    unlike the quench's baseline-relative gate, the assign stage exists to
    REMOVE existing conflicts, and baseline-relative exclusions make the
    damaged status quo feasible at zero cost (measured: the ILP chose
    all-stay on the swap corpus). The outer stage gate still reverts any
    application that worsens the board."""
    ctx = state.legality_ctx
    pa, pb = state.parts[a], state.parts[b]
    cur = ctx.pair_shortfall(a, b, pose_a=(pos_a[0], pos_a[1], pa.rot),
                             pose_b=(pos_b[0], pos_b[1], pb.rot))
    # run-6: `stack` makes ANY-net pad intersection a conflict here too --
    # without it the assign/exchange solvers could co-place two same-net
    # parts in the same space (the shipped C14-on-R14 class: R14's
    # homecoming landed on squatting C14 and no exclusion row existed).
    return (cur.pad > legality.EPS or cur.hole > legality.EPS or cur.stack)


def _interacting_pairs(state, candidates):
    """Part pairs whose candidate extents can come near each other."""
    ctx = state.legality_ctx
    reach: Dict[str, Tuple[float, float, float, float]] = {}
    for ref, cands in candidates.items():
        pp = ctx.parts.get(ref)
        if pp is None:
            continue
        boxes = []
        for (x, y) in cands:
            e = pp.extent(x, y, state.parts[ref].rot)
            if e is not None:
                boxes.append(e)
        if boxes:
            reach[ref] = (min(b[0] for b in boxes), min(b[1] for b in boxes),
                          max(b[2] for b in boxes), max(b[3] for b in boxes))
    refs = sorted(reach)
    m = state.clearance + 0.1
    for i, a in enumerate(refs):
        ra = reach[a]
        for b in refs[i + 1:]:
            rb = reach[b]
            if (ra[2] + m >= rb[0] and rb[2] + m >= ra[0]
                    and ra[3] + m >= rb[1] and rb[3] + m >= ra[1]):
                yield a, b


def _solve_ilp(state, candidates, tiers, move_penalty, notes, pattern,
               edge_pref=None):
    import numpy as np
    from scipy.optimize import milp, LinearConstraint, Bounds
    from scipy.sparse import lil_matrix

    edge_pref = edge_pref or {}
    refs = sorted(candidates)
    var_of: Dict[Tuple[str, int], int] = {}
    costs: List[float] = []
    fixed = tiers.locked | tiers.zero_net
    for ref in refs:
        p = state.parts[ref]
        for k, (x, y) in enumerate(candidates[ref]):
            var_of[(ref, k)] = len(costs)
            if ref in edge_pref:
                # F2: class outranks the netlist proxy for edge parts (R1).
                m = edge_metric(state, ref, x, y, edge_pref[ref])
                c = EDGE_PREF_WEIGHT * (m if m is not None else 100.0)
            else:
                c = _net_anchor_cost(state, ref, x, y, fixed)
            # F1 nearest-slot tiebreak (see DIST_TIEBREAK_PER_MM). Applies to
            # every candidate; the stay pose charges 0 by construction.
            c += DIST_TIEBREAK_PER_MM * math.hypot(x - p.x, y - p.y)
            if (x, y) in pattern.get(ref, ()):
                c -= PATTERN_BONUS_MM       # evidence, not a cost
            elif k > 0:
                c += move_penalty
            costs.append(c)
    n = len(costs)

    rows: List[Tuple[List[int], float, float]] = []
    # one candidate per part
    for ref in refs:
        idx = [var_of[(ref, k)] for k in range(len(candidates[ref]))]
        rows.append((idx, 1.0, 1.0))
    # a pattern SLOT takes at most one part (two displaced holes must not both
    # claim the same corner -- holes carry no copper, so the pad exclusions
    # cannot see that collision)
    slot_users: Dict[Tuple[float, float], List[int]] = {}
    for ref in refs:
        for k, pos in enumerate(candidates[ref]):
            if pos in pattern.get(ref, ()):
                slot_users.setdefault(pos, []).append(var_of[(ref, k)])
    for pos, idx in sorted(slot_users.items()):
        if len(idx) > 1:
            rows.append((idx, 0.0, 1.0))
    # pairwise exclusions between conflicting candidate poses
    n_excl = 0
    # Stay-stay rows are kept SEPARATE. Forbidding the status quo is what makes
    # the solve strong -- two parts that already conflict are not allowed to
    # both sit still, so the solver has to fix them. But when neither has an
    # alternative pose, that same row makes the whole ILP infeasible, the
    # solver falls back to a descent, and the simultaneous assignment is lost
    # for every OTHER part too (measured on two run-7 boards: "ILP status 2
    # Infeasible" on every lap).
    #
    # So: solve with them, and only if that is infeasible solve again without
    # them, before giving up on the ILP entirely. Strongest formulation first,
    # the status quo as the fallback, the descent as the last resort.
    stay_rows = []
    for a, b in _interacting_pairs(state, candidates):
        for ka, pos_a in enumerate(candidates[a]):
            for kb, pos_b in enumerate(candidates[b]):
                if not _pair_conflicts(state, a, pos_a, b, pos_b):
                    continue
                row = ([var_of[(a, ka)], var_of[(b, kb)]], 0.0, 1.0)
                if ka == 0 and kb == 0:
                    stay_rows.append(row)
                else:
                    rows.append(row)
                    n_excl += 1
    rows.extend(stay_rows)
    if notes is not None:
        notes.append(f'ILP: {n} binaries, {len(rows)} rows '
                     f'({n_excl} exclusions)')

    def _solve(active_rows):
        A = lil_matrix((len(active_rows), n))
        lb = np.zeros(len(active_rows))
        ub = np.zeros(len(active_rows))
        for i, (idx, lo, hi) in enumerate(active_rows):
            for j in idx:
                A[i, j] = 1.0
            lb[i] = lo
            ub[i] = hi
        return milp(c=np.asarray(costs),
                    constraints=LinearConstraint(A.tocsr(), lb, ub),
                    integrality=np.ones(n), bounds=Bounds(0, 1))

    res = _solve(rows)
    if (res.status != 0 or res.x is None) and stay_rows:
        # Relax only the status-quo rows and try again: a board whose own
        # pre-existing conflict has no alternative pose should still get its
        # simultaneous assignment for everything else.
        relaxed = [r for r in rows if r not in stay_rows]
        if notes is not None:
            notes.append(f'ILP infeasible with {len(stay_rows)} status-quo '
                         f'row(s); retrying with the pre-existing conflicts '
                         f'priced but not forbidden')
        res = _solve(relaxed)
    if res.status != 0 or res.x is None:
        # Infeasible (exclusions + one-per-part cannot all hold): fall back.
        if notes is not None:
            notes.append(f'ILP status {res.status} ({res.message}) -- '
                         f'breakout-descent fallback')
        return _solve_breakout(state, candidates, tiers, notes)
    choice: Dict[str, int] = {}
    for ref in refs:
        for k in range(len(candidates[ref])):
            if res.x[var_of[(ref, k)]] > 0.5:
                choice[ref] = k
                break
        else:
            choice[ref] = 0
    return choice


def _solve_breakout(state, candidates, tiers, notes, max_sweeps: int = 60,
                    edge_pref=None):
    """Min-conflicts coordinate descent with breakout constraint weighting
    (Morris 1993): at a local minimum every currently-conflicting pair's
    weight is incremented, so chronic squatters eventually move first.
    Deterministic."""
    refs = sorted(candidates)
    choice = {r: 0 for r in refs}
    weights: Dict[Tuple[str, str], float] = {}
    pairs = list(_interacting_pairs(state, candidates))
    by_ref: Dict[str, List[Tuple[str, str]]] = {r: [] for r in refs}
    for a, b in pairs:
        by_ref[a].append((a, b))
        by_ref[b].append((a, b))

    def pos(ref):
        return candidates[ref][choice[ref]]

    def pair_bad(a, b):
        return _pair_conflicts(state, a, pos(a), b, pos(b))

    def ref_cost(ref, k):
        x, y = candidates[ref][k]
        w = 0.0
        for (a, b) in by_ref[ref]:
            other = b if a == ref else a
            oxy = pos(other)
            if _pair_conflicts(state, ref,
                               (x, y), other, oxy):
                w += weights.get((a, b), 1.0)
        p = state.parts[ref]
        if edge_pref and ref in edge_pref:
            m = edge_metric(state, ref, x, y, edge_pref[ref])
            base_cost = EDGE_PREF_WEIGHT * (m if m is not None else 100.0)
        else:
            base_cost = _net_anchor_cost(state, ref, x, y,
                                         tiers.locked | tiers.zero_net)
        return (w, base_cost
                # F1 nearest-slot tiebreak, mirrored from _solve_ilp
                + DIST_TIEBREAK_PER_MM * math.hypot(x - p.x, y - p.y)
                + (MOVE_PENALTY_MM if k else 0.0))

    for sweep in range(max_sweeps):
        changed = 0
        for ref in refs:
            if state.parts[ref].locked or len(candidates[ref]) < 2:
                continue
            best = min(range(len(candidates[ref])),
                       key=lambda k: ref_cost(ref, k) + ((0.0, 0.0)
                                                         if k == choice[ref]
                                                         else (0.0, 1e-9)))
            if best != choice[ref]:
                choice[ref] = best
                changed += 1
        bad = [(a, b) for a, b in pairs if pair_bad(a, b)]
        if not bad:
            break
        if changed == 0:
            for key in bad:
                weights[key] = weights.get(key, 1.0) + 1.0
    if notes is not None:
        residual = sum(1 for a, b in pairs if pair_bad(a, b))
        notes.append(f'breakout descent: {residual} residual conflicting '
                     f'pair(s)')
    return choice

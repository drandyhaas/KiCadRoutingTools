"""Generate an initial placement from a declared floorplan intent.

The placement stack refines; it deliberately does not place from scratch
UNAIDED (placement_state.py:13, docs/placement-optimization.md) -- handed a
pile of parts it has nothing to inherit constraints from. This module is the
aided path: the intent file IS the constraint carrier (zones, edge bands,
locks, decap rules), so a board whose repo declares one can get a legal,
deterministic, seeded starting placement instead of a refusal.

What each intent construct becomes, in placement order:

  1. ``edge_connectors``   the declared edge, overhang centered in the band,
                           distributed evenly along the edge. Placed WITHOUT
                           the legality gate: overhanging the outline is the
                           point, and candidate_valid would veto it.
  2. single-ref zones      the zone center -- the "few hundred microns around
                           the spec coordinate" pattern for spec-pinned parts.
  3. multi-ref zones       members packed radially from the zone center,
                           highest pin count first, constrained to the zone
                           rect plus its declared tolerance.
  4. everything else       nearest legal pose to the centroid of its already-
                           placed partners (fanout-capped, so GND does not
                           drag everything to the board middle) -- which is
                           also what lands a decap next to its IC.

Rotations: the input rotation is tried IN FULL first and kept when it fits;
a part with no contained legal pose at it falls back to its 90-degree
lattice, and the note names the change. The intent schema cannot express a
rotation, so a part whose rotation is a DECISION (pin order, the U3 rot-180
case) must be locked -- an unlocked load-bearing rotation was never
protected from the quench either. Explore rotations deliberately with
place_portfolio's `poses` strategy.

Determinism: the only randomness is ``random.Random(f"{seed}")`` -- it breaks
ties in the packing order and jitters non-spec targets, so different seeds
give genuinely different (still legal) seeds while the same seed reproduces
byte for byte. Everything else iterates sorted (#457).
"""
from __future__ import annotations

import fnmatch
import itertools
import math
import random
import re
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

# Ring-search enumeration: nearest-first out to this radius, then a FINE ring
# near the target, then a coarse whole-board sweep. The fine pass exists
# because a packed board's remaining windows can be sub-millimeter -- measured:
# the 51x21 board's LDO had exactly one fully-legal window left, 0.09mm tall,
# which both a 1.0mm ring and a 2.0mm sweep step straight over. A part that
# finds nothing anywhere is reported UNSEATED, never silently dropped.
SEARCH_RADIUS_MM = 30.0
SEARCH_STEP_MM = 1.0
SEARCH_FINE_RADIUS_MM = 16.0
SEARCH_FINE_STEP_MM = 0.25
# Run-7 A3: a third, grid-step ring near the target -- the scar above says
# 0.25 still steps over the last legal pocket on a packed board (the
# 0.09mm window). Small radius keeps it affordable.
SEARCH_XFINE_RADIUS_MM = 4.0
FALLBACK_STEP_MM = 2.0
TARGET_JITTER_MM = 1.5


def _rect_inside(rect, outer, tol: float) -> bool:
    return (rect[0] >= outer[0] - tol and rect[1] >= outer[1] - tol
            and rect[2] <= outer[2] + tol and rect[3] <= outer[3] + tol)


def pose_ok(state, ref: str, x: float, y: float, rot: float,
            exclude: Set[str]) -> bool:
    """THE seat predicate: fully contained, and clear of everything not in
    `exclude`.

    Lifted out of `_try_place` so a POSE COUNTER can use the identical test.
    `count_legal_poses` below answers "how many seats would lifting X free",
    and that answer is only worth anything if a positive count implies the
    seat would really have been taken -- which needs the same predicate, not
    a second copy of it that drifts.

    Note `candidate_valid` alone is not enough: for a part whose incumbent
    pose is off the board, its #456 branch accepts poses that move strictly
    TOWARD the board while still outside it. Placement from scratch has no
    incumbent worth improving on, so full containment is demanded explicitly.

    The third conjunct is the intent's declared KEEP-OUTS (#701). Before it,
    `rule_keepout` graded a region the seat search walked parts into, forever:
    a declared keep-out was enforced by nothing at all. It sits BEFORE
    `candidate_valid` for the same reason `count_legal_poses` puts the zone
    gate first -- a handful of float compares against a usually-empty tuple,
    where `candidate_valid` ends in the neighbour loop.
    """
    part = state.parts[ref]
    r, tht = part.rects(x, y, rot)
    if state.edge_gate.rect_outside_amount(r) > 1e-9:
        return False
    # BOTH rects, because `rule_keepout` grades both: a through-hole part's
    # leads pass through a keep-out even when its body sits on the far side,
    # so a courtyard-only seat would be accepted here and flagged there --
    # exit 4 on a board this function placed correctly.
    # ABSOLUTE, via the state's shared loop. `edge_seat_ok` below had a second
    # copy of this and #702 gave `candidate_valid` a third -- with a MONOTONE
    # policy, which this predicate must not inherit: seeding from scratch has
    # no incumbent worth improving on, and `test_701_keepout_predicate.py`
    # seats a part whose current pose is fully inside a keep-out and asserts
    # refusal, which "no worse than where you already are" would admit. One
    # loop, two policies, both named.
    if not state.keepout_clear(ref, (r, tht)):
        return False
    return state.candidate_valid(ref, x, y, rot, exclude=exclude)


# A pose census is a DIAGNOSTIC, not a search: it answers "is there room"
# and must cost a fraction of the seat attempt that already failed. The cap
# is what bounds it -- run 19's measured answers were 46 and 32 poses freed,
# so a cap well above those still distinguishes "none" from "plenty".
CENSUS_RADIUS_MM = 16.0
CENSUS_STEP_MM = 1.0
CENSUS_CAP = 64


def zone_gate(part, constraint, tol: float):
    """`(predicate, anchor_zone)` for "is this pose inside the zone".

    ONE definition, shared by the seat search and the pose census. It used to
    live as a closure inside `_try_place`, which meant the census -- the
    thing that tells a plan author WHY a part could not be seated -- had no
    zone concept at all. Measured on splitflap_driver: three parts packed
    into a 2x2mm zone were refused, and the census answered over an
    unconstrained 3mm disc, so one verdict read "64 legal poses with nothing
    lifted" about a part the same pass had just refused to seat, and two
    more advised lifting U1 when the zone was the problem.

    The ANCHOR relaxation is the half that must not be dropped when copying.
    A zone smaller than the courtyard cannot contain it at any rotation --
    the spec-COORDINATE pattern -- so containment relaxes to
    anchor-point-in-zone, which makes such zones satisfiable by
    construction; `floorplan.rule_zone_containment` grades them the same
    way. A census that mirrors only the strict branch under-counts exactly
    where the relaxation applies: C1 into a 0.4mm zone at its own pose
    censuses 0 with the strict rule and 294 with this one, while
    `_try_place` seats it either way. That is a confident zero, which is the
    one census answer worse than no census -- so this returns the same pair
    of branches to both callers rather than letting a second copy drift.
    """
    if constraint is None:
        return (lambda x, y, rot: True), False
    from placement.floorplan import zone_fits_courtyard
    anchor = not any(
        zone_fits_courtyard(constraint, part.rect(0.0, 0.0, r), tol)
        for r in (part.rot % 360, (part.rot + 90) % 360))

    if anchor:
        def _in(x, y, rot):
            return (constraint[0] - tol <= x <= constraint[2] + tol
                    and constraint[1] - tol <= y <= constraint[3] + tol)
    else:
        def _in(x, y, rot):
            return _rect_inside(part.rect(x, y, rot), constraint, tol)
    return _in, anchor


# A census sweep never exceeds this many locations. It is the ONLY bound on
# the cost, so it is stated as a count rather than left to fall out of a
# radius: (2*25+1)^2 = 2601 locations, x4 rotations = ~10k `pose_ok` calls
# worst case, for the blocked part that has to exhaust the ladder.
CENSUS_MAX_LOCATIONS = 2601


def _feasible_centre_box(part, constraint, tol, anchor):
    """Where `part`'s CENTRE may sit for the zone to hold it, over rotations.

    Derived, not approximated. Containment at rotation r is
    `zone[0]-tol <= x + b_r[0]` and `x + b_r[2] <= zone[2]+tol`, so
    `x` in `[zone[0]-tol-b_r[0], zone[2]+tol-b_r[2]]`, and the UNION over
    rotations is `[zone[0]-tol-max_r(b_r[0]), zone[2]+tol-min_r(b_r[2])]`.

    THE COURTYARD IS NOT CENTRED ON THE FOOTPRINT ORIGIN -- `part.rect` is
    `(x+b[0], y+b[1], x+b[2], y+b[3])` with `b` the raw local bounds, and 17
    of 65 parts on splitflap_driver and 6 of 89 on tigard have an offset
    centre (up to 10.15mm on tigard J3). A symmetric half-extent deflation
    therefore SHIFTS the box: measured, J18/J19/J20 censused 0 while
    `_try_place` seated them at full clearance. Two earlier forms of this
    function were wrong here in opposite directions (max half-extent = a
    subset, min half-extent = a shifted box), which is why this is now the
    algebra rather than a bound.
    """
    x0, y0, x1, y1 = (float(v) for v in constraint)
    if anchor:
        # Anchor zones constrain the ANCHOR POINT, so the centre box is the
        # zone itself; no courtyard term enters.
        return x0 - tol, y0 - tol, x1 + tol, y1 + tol
    max_b0x = max_b0y = -float('inf')
    min_b2x = min_b2y = float('inf')
    for rot in (part.rot, part.rot + 90.0, part.rot + 180.0, part.rot + 270.0):
        b = part.rect(0.0, 0.0, rot % 360)
        max_b0x = max(max_b0x, b[0])
        max_b0y = max(max_b0y, b[1])
        min_b2x = min(min_b2x, b[2])
        min_b2y = min(min_b2y, b[3])
    return (x0 - tol - max_b0x, y0 - tol - max_b0y,
            x1 + tol - min_b2x, y1 + tol - min_b2y)


def zone_census_offsets(part, constraint, tol, tx, ty, grid_step=0.1,
                        max_disp=None):
    """`(step, [(dx, dy), ...], reach_mm)` for a census under a ZONE.

    A disc around the target is the wrong sample set once a constraint
    applies: the reachable poses are bounded by the zone, so a disc census
    spends its whole location cap on ground the zone excludes and has to
    coarsen its step to afford it. Measured on splitflap_driver, R1 packed
    into a 2x2mm zone: a 0.25mm disc lattice counts ZERO with the blocker
    lifted, because the zone leaves a feasible x-window for R1's centre
    **0.07mm wide**, while `_try_place` reaches that window on its 0.1mm
    ring and seats. Threading the constraint without this moves the
    confident zero from the relaxation axis to the lattice axis instead of
    removing it.

    So: enumerate the feasible-CENTRE box instead, on the lattices
    `_try_place` sweeps AT EACH DISTANCE. That last clause is the half an
    earlier version got wrong: it chose one step by location count alone, so
    a large zone fell to the 1.0mm ring (a ring that exists to cross the
    board cheaply, not to decide whether a part fits, so a verdict rendered
    on it is a confident zero) and a distant zone was sampled at 0.1mm where
    the search only reaches 0.25mm -- 4 of 8 censused parts in one ordinary
    zone pack then promised poses the retry could never collect.

    `reach_mm` is how far from the target the sweep actually got:
    `_try_place` skips its whole-board fallback whenever `max_disp` is set,
    so nothing beyond `SEARCH_FINE_RADIUS_MM` is reachable anyway, and the
    location cap can bite before even that.
    """
    grid = max(0.05, float(grid_step or 0.1))
    _in, anchor = zone_gate(part, constraint, tol)
    lo_x, lo_y, hi_x, hi_y = _feasible_centre_box(part, constraint, tol,
                                                 anchor)
    # Nothing past the fine ring is reachable once `max_disp` is set, so the
    # box is clipped there BEFORE it is materialised -- that is also the
    # guard against a hostile zone (a 2000mm rect used to build a 4,004,001
    # entry list, once per censused part, uncached).
    reach = SEARCH_FINE_RADIUS_MM if max_disp is None \
        else min(float(max_disp), SEARCH_FINE_RADIUS_MM)
    lo_x, hi_x = max(lo_x, tx - reach), min(hi_x, tx + reach)
    lo_y, hi_y = max(lo_y, ty - reach), min(hi_y, ty + reach)
    if hi_x < lo_x or hi_y < lo_y:
        # The zone cannot hold this part at all, or the budget cannot reach
        # it. ONE offset, so the caller still evaluates the target itself and
        # the answer is a measured zero rather than an empty sweep that never
        # ran -- and `reach_mm` 0.0 says the sweep never left the target.
        return grid, [(0.0, 0.0)], 0.0

    def _lattice(step, rmax):
        """Target-aligned points of `step` inside the box and within `rmax`."""
        i0 = int(math.ceil((lo_x - tx) / step - 1e-9))
        i1 = int(math.floor((hi_x - tx) / step + 1e-9))
        j0 = int(math.ceil((lo_y - ty) / step - 1e-9))
        j1 = int(math.floor((hi_y - ty) / step + 1e-9))
        pts = []
        for i in range(i0, i1 + 1):
            dx = i * step
            if abs(dx) > rmax + 1e-9:
                continue
            for j in range(j0, j1 + 1):
                dy = j * step
                if dx * dx + dy * dy <= rmax * rmax + 1e-9:
                    pts.append((round(dx, 6), round(dy, 6)))
        return pts

    # TWO lattices, exactly as `_try_place` does it: `grid_step` out to
    # SEARCH_XFINE_RADIUS_MM, then SEARCH_FINE_STEP_MM out to the reach.
    # The 1.0mm ring is deliberately excluded -- a census must not render a
    # verdict on it (see the docstring).
    seen, out = set(), []
    for step, rmax in ((grid, min(reach, SEARCH_XFINE_RADIUS_MM)),
                       (max(grid, SEARCH_FINE_STEP_MM), reach)):
        for d in _lattice(step, rmax):
            if d not in seen:
                seen.add(d)
                out.append(d)
    if not out:
        return grid, [(0.0, 0.0)], 0.0
    out.sort(key=lambda d: (d[0] * d[0] + d[1] * d[1]))
    if len(out) > CENSUS_MAX_LOCATIONS:
        out = out[:CENSUS_MAX_LOCATIONS]
    reach = math.sqrt(max(d[0] * d[0] + d[1] * d[1] for d in out))
    step_used = grid if any(
        abs(d[0]) <= SEARCH_XFINE_RADIUS_MM
        and abs(d[1]) <= SEARCH_XFINE_RADIUS_MM for d in out) else \
        max(grid, SEARCH_FINE_STEP_MM)
    return step_used, out, round(reach, 6)


# How many incumbents one stuck part may censused against. Bounded because
# each candidate costs a census sweep, and because a part is blocked by its
# NEIGHBOURS -- a part on the far side of the board cannot be in the way, and
# the geometry below proves it rather than assuming it.
EVICT_MAX_BLOCKERS = 8

# Depth 2 lifts a PAIR (#699). A rung that only ever lifts ONE neighbour
# records "immovable" for a part two neighbours jointly block, and that
# verdict is true only of the basin the board happens to be in: the reporter's
# connector censused 8 neighbours, none of which frees a pose alone, while the
# truth arrangement of the same board seats it by moving two of them together.
#
# The bound is a COUNT, deliberately -- a wall-clock budget would make the
# same board place differently on a slow machine and a fast one (#621, the
# reason `--deadline` was deleted repo-wide). ONE live bound, not two: a
# second "only pair up the nearest K" cap set to C(K,2) can never bite, and a
# bound that cannot bite is a comment pretending to be a limit.
#
# 16 of the C(8,2)=28 pairs, in "both blockers close to the contested region"
# order -- by (i+j) over the nearest-first candidate list, so the truncation
# drops the far-far pairs rather than starving one candidate of partners.
# Plain `combinations` order would spend the whole budget pairing the single
# nearest blocker with everything. What is dropped is REPORTED, never silent.
EVICT_MAX_PAIRS = 16


def _evict_candidates(state, ref: str, tx: float, ty: float,
                      placed: Set[str], immovable,
                      constraint=None, tol: float = 0.5,
                      info: Optional[Dict] = None) -> List[str]:
    """Seated, movable parts that could possibly be in `ref`'s way at (tx,ty).

    `immovable` is every seated ref the rung may not lift: the intent's
    `must_lock` set and its DECLARED EDGE CONNECTORS. A file-locked part is
    skipped here as well. It may be a plain set, or a {ref: source} mapping,
    in which case the source is what `info['frozen']` reports. The edge connectors are excluded because the rung
    re-seats a blocker through `_try_place`, which demands full containment
    -- measured, it lifted a stage-1 connector off its edge band and seated
    it inland, on top of another part. An edge seat is `_seat_edge`'s to
    make, and sliding a connector along its edge to free a pocket is not
    what this rung does.

    A superset, not a heuristic: the box is every pose `ref` can take within
    the census radius, inflated by its own reach and the clearance, so a part
    whose own inflated extent misses it cannot be within clearance of ANY
    candidate pose and would free exactly zero poses by construction. That is
    `build_neighbor_lists`' pruning argument (quench.py) with the travel
    budget replaced by the census radius.

    Then nearest-first, capped: the cap is the only approximation, and it is
    reported rather than hidden -- pass `info` and it comes back carrying
    `boxed` / `movable` / `frozen` / `truncated`, which is what makes
    "censused 8 neighbour(s)" auditable against how many there really were.
    """
    part = state.parts.get(ref)
    if part is None:
        return []
    r = part.rect(tx, ty, part.rot)
    reach = max(r[2] - r[0], r[3] - r[1]) / 2.0 + CENSUS_RADIUS_MM
    clr = state.clearance
    locked = set(immovable or ())
    # UNDER A ZONE THE TARGET IS NOT THE CENTRE OF THE QUESTION. The zone
    # stage jitters each member's target, so a target can sit wholly outside
    # a small zone (measured: 3.3mm out on a 2x2 zone), and both the box and
    # the nearest-first cap were keyed on it -- so the 8 chosen
    # need not be the 8 that overlap the zone the part must actually reach.
    # Box on the union, rank by distance to the region being contested.
    bx0, by0, bx1, by1 = tx - reach, ty - reach, tx + reach, ty + reach
    cx, cy = tx, ty
    if constraint is not None:
        bx0 = min(bx0, constraint[0] - tol - reach)
        by0 = min(by0, constraint[1] - tol - reach)
        bx1 = max(bx1, constraint[2] + tol + reach)
        by1 = max(by1, constraint[3] + tol + reach)
        cx = min(max(tx, constraint[0]), constraint[2])
        cy = min(max(ty, constraint[1]), constraint[3])
    out: List[Tuple[float, str]] = []
    frozen: Dict[str, str] = {}
    # THE BOX FIRST, then the freeze. Reversed (as this read until #699) a
    # locked neighbour never reaches the box test, so "no candidates" cannot
    # be told apart from "every neighbour that could be in the way is one
    # nobody may move" -- two verdicts with two different answers for the
    # reader, recorded as the same empty dict.
    for other in sorted(placed):
        if other == ref or other not in state.parts:
            continue
        op = state.parts[other]
        orect = op.rect(op.x, op.y, op.rot)
        if (orect[2] + clr < bx0 or orect[0] - clr > bx1
                or orect[3] + clr < by0 or orect[1] - clr > by1):
            continue
        if op.locked or other in locked:
            # NAME THE SOURCE. "frozen" collapses three different decisions
            # -- a lock in the FILE, the intent's must_lock, a declared edge
            # connector -- and the reader's next move differs for each.
            frozen[other] = ('file-locked' if op.locked else
                             (immovable.get(other)
                              if isinstance(immovable, dict) else None)
                             or 'immovable')
            continue          # not this tool's to move -- see reseat_scope
        out.append((math.hypot(op.x - cx, op.y - cy), other))
    out.sort()
    picked = [b for _d, b in out[:EVICT_MAX_BLOCKERS]]
    if info is not None:
        # The docstring above has always promised the cap is "reported by the
        # caller rather than hidden". Until #699 nothing reported it: eight
        # entries in `no_pose_blockers` and no way to learn there were twelve.
        info['boxed'] = len(out) + len(frozen)
        # `movable` is how many COULD have been censused; `censused` is how
        # many were. Reporting the pre-cap number as the censused one is the
        # exact inversion of what the cap disclosure is for -- "censused 12
        # neighbour(s) ... 4 not censused" over a sweep that tested 8.
        info['movable'] = len(out)
        info['censused'] = len(picked)
        info['frozen'] = dict(sorted(frozen.items()))
        info['truncated'] = max(0, len(out) - len(picked))
    return picked


def count_legal_poses(state, ref: str, tx: float, ty: float,
                      exclude: Set[str], *,
                      radius: float = CENSUS_RADIUS_MM,
                      step: float = CENSUS_STEP_MM,
                      cap: int = CENSUS_CAP,
                      max_disp: Optional[float] = None,
                      rotations: Optional[Sequence[float]] = None,
                      constraint=None, tol: float = 0.5,
                      without_keepouts: Sequence[str] = ()) -> int:
    """How many legal poses `ref` has near (tx, ty), counting at most `cap`.

    This is issue #629's measurement. Three consecutive sweeps in run 19
    returned a bare "no legal pose anywhere on the board" for SW17/SW34, and
    when the same question was finally asked in scoped form the engine
    answered precisely: with D14 in place 0 poses, with D14 lifted 46; with
    D31, 0 then 32. A verdict that names its blockers is the next move; a
    bare verdict is a dead end.

    Counts only -- no `apply_move`, no cost evaluation. It uses `pose_ok`,
    the same predicate `_try_place` seats on, at the state's own clearance
    (NOT the relaxed ladder): a count is meant to say whether there is room,
    and counting poses that only exist at a 0.02mm floor would promise seats
    the ordinary search does not take. Measured cost of that divergence on
    splitflap_driver: 4 of 65 movable parts census 0 while `_try_place`
    seats them on a relaxed rung, 6 of 65 with a zone. Reported, not fixed --
    see the paragraph above.

    **With a `constraint`, the zone is honoured and the SAMPLE SET comes
    from the zone too** (`zone_census_offsets`). Both halves are needed: the
    predicate alone leaves the census answering over a disc the zone
    excludes, and the disc alone leaves it counting poses that are outside
    the zone the seat had to satisfy. `radius`/`step` are ignored when a
    constraint is given -- the zone supersedes them.

    `without_keepouts` names declared keep-outs to LIFT for the sweep (#701).
    A keep-out is measured the way a blocker is measured -- count the poses
    with it out of the way -- so that "this keep-out is what refuses the
    part" is a NUMBER from the same predicate the seat search uses, not a
    static "does the zone intersect a keep-out" test computed some other way.
    A verdict derived from a different question is the reported-field trap
    one level up.
    """
    from pose_score import _offsets
    part = state.parts[ref]
    rots = list(rotations) if rotations is not None \
        else [part.rot] + [(part.rot + d) % 360 for d in (90.0, 180.0, 270.0)]
    in_zone, _anchor = zone_gate(part, constraint, tol)
    if constraint is None:
        offsets = _offsets(radius, step)
    else:
        _step, offsets, _reach = zone_census_offsets(
            part, constraint, tol, tx, ty,
            getattr(state, 'grid_step', 0.1), max_disp)
    # Lift the named keep-outs for the duration of the sweep, the same way
    # `_try_place` swaps `state.clearance` and `_evict_trade` swaps a pose:
    # a try/finally around the state, so the count comes from `pose_ok`
    # itself rather than from a second, keep-out-blind predicate.
    lift = set(without_keepouts or ())
    saved = None
    if lift and state.keepouts_for.get(ref):
        saved = state.keepouts_for[ref]
        # The gate derives its keep-out terms from `keepouts_for` per
        # call, so the lift below is honoured -- but the INCUMBENT
        # vector is cached, and under a lift it has the wrong arity.
        state._inc_intent.clear()
        kept = tuple(k for k in saved if k['name'] not in lift)
        if kept:
            state.keepouts_for[ref] = kept
        else:
            del state.keepouts_for[ref]
    try:
        n = 0
        for dx, dy in offsets:
            if max_disp is not None and math.hypot(dx, dy) > max_disp + 1e-9:
                continue
            x, y = round(tx + dx, 3), round(ty + dy, 3)
            for rot in rots:
                # Zone first: it is four float compares, where `pose_ok` ends
                # in `candidate_valid`. Measured 19x faster on a small zone.
                if not in_zone(x, y, rot):
                    continue
                if pose_ok(state, ref, x, y, rot, exclude):
                    n += 1
                    if n >= cap:
                        return n
        return n
    finally:
        if saved is not None:
            state.keepouts_for[ref] = saved
            state._inc_intent.clear()


#: The verdicts a part with no legal pose can be given (#699). Two of them
#: used to be the SAME ledger entry -- an empty `no_pose_blockers[ref]` --
#: and they have opposite answers for the reader: NO_MOVABLE_NEIGHBOUR says
#: the geometry refuses the part with nothing in the way, IMMOVABLE_GIVEN
#: _FROZEN says the neighbours that ARE in the way are ones somebody locked,
#: so the next move is to relax a lock, not to re-place anything.
NO_POSE_VERDICTS = (
    'seated_after_eviction',    # not unseated after all: a trade worked
    'no_target_recorded',       # the rung never got to ask (no seat context)
    'no_movable_neighbour',     # nothing seated is anywhere near it
    'immovable_given_frozen',   # only locked / declared-edge neighbours are
    'no_single_lift_frees',     # movable neighbours censused, none frees one
    'no_pair_lift_frees',       # ... and no PAIR of them frees one either
    'blocker_available',        # a lift WOULD free a pose; the depth said no
    'trade_reverted',           # a trade was tried and put back
    # #701. Before it, a part a declared KEEP-OUT refuses reported
    # `no_movable_neighbour`, whose prose says "the outline, the zone or its
    # own size refuses it, not a neighbour" -- false, and the reader's next
    # move is different: move the keep-out, or add the part to its `allow`.
    'keepout_blocks',
)


def _empty_census() -> Dict:
    """A census record with every sub-key present, so no consumer needs a
    defaulting `.get` that quietly reads as a real measurement.

    A FUNCTION, not a module-level dict copied with `dict()`: that copy is
    shallow, so every record built from it would share one `frozen` dict and
    a single write into any of them would corrupt the template for the whole
    process.
    """
    return {'boxed': 0, 'movable': 0, 'censused': 0, 'frozen': {},
            'truncated': 0, 'baseline': 0, 'pairs_total': 0,
            'pairs_censused': 0, 'pairs_truncated': 0, 'best_pair': None,
            # #701: poses freed by lifting EVERY bound keep-out at once, for
            # a part no single one explains. 0 means the keep-outs are not
            # jointly what refuses it.
            'keepouts_joint': 0,
            # #701: {keep-out name: poses freed by lifting it}. Present and
            # empty on every part, per this function's whole rationale --
            # a consumer must never need a defaulting `.get` to tell "no
            # keep-out is in the way" from "keep-outs were not considered".
            'keepouts_freeing': {}}


def _verdict_for(cands: Sequence[str], census: Dict) -> str:
    """The verdict for a part still unseated after the census."""
    if census.get('keepouts_freeing') or census.get('keepouts_joint'):
        # #701, and FIRST: a declared keep-out that frees poses when lifted
        # is the answer, whatever the neighbours look like. It outranks
        # `no_movable_neighbour`, whose prose would actively mislead here,
        # and it sits below `blocker_available` for free -- this function is
        # only reached when no trade was chosen.
        v = 'keepout_blocks'
    elif not cands:
        v = ('immovable_given_frozen' if census.get('frozen')
             else 'no_movable_neighbour')
    else:
        v = ('no_pair_lift_frees' if census.get('pairs_censused')
             else 'no_single_lift_frees')
    # The vocabulary is only a contract if something checks it: a typo'd
    # verdict string is invisible to every consumer that switches on it.
    assert v in NO_POSE_VERDICTS, v
    return v


def _no_pose_note(ref: str, verdict: str, census: Dict,
                  evict_depth: int = 0) -> str:
    """The one place the verdict's prose is written.

    A verdict string in the JSON and a differently-worded sentence on stdout
    is the next thing to drift apart, so both come from here -- the same
    argument `zone_gate` makes for having ONE definition shared by the seat
    search and the pose census.
    """
    frozen = census.get('frozen') or {}
    # The number ACTUALLY censused, never the number that could have been:
    # "lifting any ONE of them frees no pose" is a claim about the refs the
    # sweep tested.
    n = census.get('censused', 0)
    trunc = census.get('truncated', 0)
    tail = (f" ({trunc} further movable neighbour(s) not censused, cap "
            f"EVICT_MAX_BLOCKERS={EVICT_MAX_BLOCKERS})" if trunc else "")
    # A movable neighbour AND a frozen one is the interesting mixed case:
    # the verdict is about what could be lifted, but the reader's cheapest
    # move may well be to unfreeze the other one.
    if frozen and verdict in ('no_single_lift_frees', 'no_pair_lift_frees'):
        tail += ("; also in the way, and not this rung's to move: "
                 + ', '.join(f"{r} ({why})" for r, why in sorted(frozen.items())))
    if verdict == 'keepout_blocks':
        freeing = census.get('keepouts_freeing') or {}
        if freeing:
            who = ', '.join(f"{n!r} (frees {c})"
                            for n, c in sorted(freeing.items()))
            what = (f"the DECLARED KEEP-OUT(S) {who} are what refuse it -- "
                    f"not a neighbour")
        else:
            # The JOINT case: no single keep-out frees a pose, all of them
            # together do. Naming one here would be false of every individual
            # one, which is why the sentence does not.
            what = (f"the declared keep-outs are JOINTLY what refuse it -- "
                    f"no single one frees a pose, lifting all of them frees "
                    f"{census.get('keepouts_joint', 0)}, and no neighbour is "
                    f"involved")
        return (f"{ref}: no legal pose, and {what}, and not this rung's to "
                f"lift. Move a keep-out, or add {ref} to an `allow` list if "
                f"it is the part that owns one")
    if verdict == 'no_movable_neighbour':
        return (f"{ref}: no legal pose, and NOTHING seated is near enough to "
                f"be in the way -- the outline, the zone or its own size "
                f"refuses it, not a neighbour")
    if verdict == 'immovable_given_frozen':
        who = ', '.join(f"{r} ({why})" for r, why in sorted(frozen.items()))
        return (f"{ref}: no legal pose, and every neighbour that could be in "
                f"the way is one this rung may not move: {who}. Immovable "
                f"GIVEN those, not immovable")
    if verdict == 'no_single_lift_frees':
        # Only suggest the depth the run is not already at -- at depth 2 with
        # fewer than two movable candidates there is no pair to try, and
        # telling the reader to pass the flag they passed is noise.
        hint = ("; --evict-depth 2 also tries pairs" if evict_depth < 2
                else "; fewer than two movable neighbours, so there is no "
                     "pair to try")
        return (f"{ref}: censused {n} neighbour(s); lifting any ONE of them "
                f"frees no pose{tail}{hint}")
    if verdict == 'no_pair_lift_frees':
        p = census.get('pairs_censused', 0)
        pt = census.get('pairs_truncated', 0)
        return (f"{ref}: censused {n} neighbour(s) and {p} pair(s); lifting "
                f"no one or two of them frees a pose{tail}"
                + (f" ({pt} further pair(s) not censused, cap "
                   f"EVICT_MAX_PAIRS={EVICT_MAX_PAIRS})" if pt else ""))
    return ""


def _seated_violations(state, seated: Set[str]) -> Tuple[int, float]:
    """`(violating parts-or-pairs, courtyard overlap mm2)` over the SEATED
    parts only.

    The pile is not measured. An unseated part sits at a coordinate that
    means nothing, and a measure that includes it is wrong in whichever
    direction it is read: counting its overlaps makes any move out of the
    pile look like a repair, while `reconstruct.measure`, which ranks HPWL
    above overlap area, makes every legal seat look like a loss because the
    pile's HPWL is artificially short. The first version of the eviction
    rung gated on that tuple and refused every legal trade (measured:
    `[.. 3.502, 1.0] -> [.. 12.706, 0.0]` rejected) while accepting the one
    that stacked the blocker on the part it had just seated.

    A pair counts when its courtyards intersect on a shared side, or, when
    the pad layer is on, when pads or holes intersect; a part counts when
    its body is contained in another's (#680's fab-currency test, with its
    marker/container exemptions). Container parts are skipped as obstacles,
    as `candidate_valid` skips them. Clearance SHORTFALL is deliberately not
    counted: `_try_place` seats at a relaxed clearance by design when
    nothing else fits, and a rung that refused what the ordinary stages
    accept would trade a seated part for an unseated one.
    """
    refs = sorted(r for r in seated if r in state.parts)
    containers = set(getattr(state, 'container_refs', ()) or ())
    ctx = state.legality_ctx
    others = set(state.parts) - set(refs)
    count = 0
    area = 0.0
    for i, a in enumerate(refs):
        pa = state.parts[a]
        if a not in containers:
            try:
                if state._body_contained_at(a, None, None, None,
                                            exclude=others):
                    count += 1
            except Exception:          # noqa: BLE001 -- unjudged, not clear
                count += 1
        ra = pa.rects()
        for b in refs[i + 1:]:
            if a in containers or b in containers:
                continue
            pb = state.parts[b]
            gap = pa.gap_to(pb, ra)
            bad = gap is not None and gap < -1e-9
            if bad and pa.side == pb.side:
                r1, r2 = ra[0], pb.rect()
                area += (max(0.0, min(r1[2], r2[2]) - max(r1[0], r2[0]))
                         * max(0.0, min(r1[3], r2[3]) - max(r1[1], r2[1])))
            if not bad and ctx is not None:
                sf = ctx.pair_shortfall(a, b)
                bad = bool(sf.pad_overlap or sf.stack or sf.hole > 1e-6)
            if bad:
                count += 1
    return count, round(area, 4)


def _evict_trade(state, ref: str, blockers: Sequence[str],
                 tx: float, ty: float, constraint, tol: float,
                 blocker_zones: Sequence[Tuple[Any, float]],
                 placed: Set[str], unplaced: Set[str]) -> Dict:
    """Lift every ref in `blockers`, seat `ref` at the target it was refused
    at, put the blockers back; keep the trade only under the rule below, else
    restore all of them.

    `blockers` is ONE ref at depth 1 and TWO at depth 2 (#699). Nothing in
    the rule below is per-blocker-count: the same three conjuncts decide a
    pair, over a bigger snapshot. `blocker_zones` is the matching
    `(constraint_rect, tol)` for each blocker, in the same order.

    THE ACCEPTANCE RULE, in this order, every conjunct required:

      1. every seat was found -- `_try_place` seated `ref` (under its own
         zone) against the board with the blockers lifted, then seated each
         blocker (under ITS own zone, searching out from its old pose)
         against the board with `ref` in it. The blockers go back HARDEST
         FIRST (descending courtyard extent, then name), so the part with
         the least choice picks while the board is emptiest, and each one
         that lands is an obstacle to the next;
      2. all of them are legal against the FULL seated set, re-checked here
         with `pose_ok` at the clearance the seats were found at. The only
         parts excluded are the ones still in the pile, so `ref` and every
         blocker are obstacles to each other. This is the conjunct that does
         not trust the bookkeeping: the first version of this rung re-seated
         the blocker with `ref` still in its exclude set and landed it on top
         of `ref`, 100% inside its courtyard, and reported `unseated 0`;
      3. `_seated_violations` over the seated parts, `ref` now among them,
         has not increased against the board before the trade. This is the
         only ABSOLUTE pad-layer check in the rule: conjunct 2's pad test
         (`candidate_valid` -> `pads_ok`) is baseline-relative, "no worse
         than the SEED pose", and two parts that started stacked have a seed
         baseline that already contains a pad intersection -- so a re-seat
         whose courtyards are clear but whose pads intersect passes 2 and is
         refused here.

    HPWL is NOT a conjunct and is not a tie-break between anything, because
    there is one trade per part: it is recorded in the returned record for
    the reader. A gate that ranks it vetoes every legal seat over an
    unseated pile (see `_seated_violations`).

    Returns the eviction record. `accepted` says which branch ran; on the
    reverted branch every part is back at its snapshot pose and `reason`
    names the conjunct that failed. `blocker` is the FIRST blocker and is
    never None -- consumers union these into ref sets; `blockers` is the
    authoritative list at every depth. The caller
    owns `placed`/`unplaced`; this function reads them and restores every
    blocker to `placed` either way.
    """
    from placement.reconstruct import part_extent_mm
    blockers = list(blockers)
    zones = dict(zip(blockers, blocker_zones))
    snapshot = {r: (state.parts[r].x, state.parts[r].y, state.parts[r].rot)
                for r in [ref] + blockers}
    seated_before = set(placed) - {ref}
    viol_before = _seated_violations(state, seated_before)
    hpwl_before = round(state.hpwl(), 3)
    pile = set(unplaced) - {ref} - set(blockers)
    # 1. lift, seat the blocked part, put the blockers back with it in place.
    for b in blockers:
        unplaced.add(b)
        placed.discard(b)
    lifted = set(blockers)
    clr_ref = _try_place(state, ref, tx, ty, pile | lifted,
                         constraint=constraint, tol=tol)
    clr_back: Dict[str, Optional[float]] = {b: None for b in blockers}
    tried: List[str] = []
    if clr_ref is not None:
        # Hardest first: the biggest courtyard has the fewest pockets left
        # once `ref` is in, and a small part squeezed in first can leave the
        # big one nowhere to go. Same ordering the anchor rounds use.
        for b in sorted(blockers, key=lambda r: (-part_extent_mm(state, r), r)):
            lifted.discard(b)
            tried.append(b)
            bx, by, _brot = snapshot[b]
            bz, btol = zones.get(b, (None, 0.5))
            clr_back[b] = _try_place(state, b, bx, by, pile | lifted,
                                     constraint=bz, tol=btol)
            if clr_back[b] is None:
                break
    ok = clr_ref is not None and all(c is not None
                                     for c in clr_back.values())
    # 2. legal against the full seated set, independently of (1).
    #
    # The re-check clearance is the MINIMUM over every seat found, which gets
    # weaker as the trade grows: one part seated on `_try_place`'s 0.02mm
    # floor drags the re-check for all the others down to that floor. Keeping
    # `min` for every N is deliberate -- a per-part clearance here would be a
    # SECOND rule, and it would silently change which depth-1 trades are
    # accepted -- but a relaxed re-check is now reported (`relaxed`) instead
    # of being invisible.
    legal = False
    relaxed = False
    if ok:
        full = state.clearance
        try:
            state.clearance = min([clr_ref] + list(clr_back.values()))
            relaxed = state.clearance < full - 1e-9
            state._inc_violation.clear()
            legal = all(pose_ok(state, r, state.parts[r].x, state.parts[r].y,
                                state.parts[r].rot, pile)
                        for r in [ref] + blockers)
        finally:
            state.clearance = full
            state._inc_violation.clear()
    # 3. the seated board did not get worse.
    viol_after = (_seated_violations(state, seated_before | {ref})
                  if ok else None)
    accepted = bool(ok and legal and viol_after <= viol_before)
    if not accepted:
        for r, pose in snapshot.items():
            state.apply_move(r, *pose)
    for b in blockers:
        placed.add(b)
        unplaced.discard(b)
    # Singular wording is kept verbatim for the one-blocker case: it is what
    # the depth-1 notes and their tests read.
    one = len(blockers) == 1
    if not ok:
        if clr_ref is None:
            reason = ('the blocked part still had no legal pose with the '
                      + ('blocker lifted' if one else 'blockers lifted'))
        elif one:
            reason = ('the blocker had no legal pose to return to with the '
                      'part in place')
        else:
            # Only the one that FAILED. `clr_back` is None both for the
            # blocker with no pose and for every blocker the `break` never
            # asked, and naming the untried ones reports a measurement that
            # was not taken.
            stuck = [b for b in tried if clr_back[b] is None]
            untried = [b for b in blockers if b not in tried]
            reason = (f"{', '.join(sorted(stuck))} had no legal pose to "
                      f"return to with the part in place"
                      + (f" ({', '.join(sorted(untried))} not attempted)"
                         if untried else ""))
    elif not legal:
        reason = 'a seat was not legal against the full seated set'
    elif not accepted:
        reason = (f'violations rose {list(viol_before)} -> '
                  f'{list(viol_after)}')
    else:
        reason = ''
    return {'ref': ref,
            # The FIRST blocker, never None: consumers union these into ref
            # sets and a None there is a landmine. `blockers` is the
            # authoritative list at every depth.
            'blocker': blockers[0],
            'blockers': list(blockers),
            'accepted': accepted,
            # In the order the returns were ATTEMPTED, after `clr_ref`, so
            # a None can be read as "this one failed" rather than conflated
            # with a blocker the break never reached (`attempted` names them).
            'clearance': [clr_ref] + [clr_back[b] for b in tried],
            'attempted': list(tried),
            'violations_before': list(viol_before),
            'violations_after': (None if viol_after is None
                                 else list(viol_after)),
            'hpwl_before': hpwl_before,
            'hpwl_after': round(state.hpwl(), 3),
            # What the trade actually COST, per evicted part: how far it was
            # pushed and whether it came back turned. HPWL is not a conjunct
            # (see above) and one trade may now displace two parts, so the
            # bet has to be visible rather than merely bounded.
            'moved': {b: round(math.hypot(state.parts[b].x - snapshot[b][0],
                                          state.parts[b].y - snapshot[b][1]),
                               3) for b in blockers} if accepted else {},
            'rotated': {b: [snapshot[b][2], state.parts[b].rot]
                        for b in blockers
                        if accepted
                        and abs(state.parts[b].rot - snapshot[b][2]) > 1e-9},
            # The conjunct-2 re-check ran below the board's clearance.
            'relaxed': relaxed,
            'reason': reason}


def _try_place(state, ref: str, tx: float, ty: float, exclude: Set[str],
               constraint=None, tol: float = 0.5,
               max_disp: Optional[float] = None,
               info: Optional[Dict] = None) -> Optional[float]:
    """Nearest FULLY-CONTAINED legal pose to (tx, ty); applies the move and
    returns True.

    `exclude` carries the not-yet-placed refs: the pile they still form at
    their meaningless input coordinates must not veto real poses.

    candidate_valid alone is not the right gate here: for a part whose
    INCUMBENT pose is off the board (a generator's default position can be),
    its #456 branch accepts poses that move strictly TOWARD the board while
    still outside it -- measured: a free LDO seeded 2.7mm outside the
    outline, "placed", unseated 0. Placement from scratch has no incumbent
    worth improving on, so full containment is demanded explicitly; the only
    deliberate off-board poses are the edge connectors, which stage 1 places
    without this helper.

    The part's CURRENT rotation is tried in full first, then the rest of its
    90-degree lattice: an unplaced pile's rotation is a generator default,
    not a decision, and a large part can have NO contained legal pose at it
    while fitting fine turned 90 (measured: the same LDO, 0 poses at rot 0
    against 3 at rot 90 on a packed 51x21 board). A part whose rotation IS a
    decision must be locked -- an unlocked "load-bearing rotation" was never
    protected from the quench either (the U3 lesson). The caller can see a
    fallback fired by comparing the part's rot before and after.

    Returns the courtyard clearance the pose was found at, or None. The full
    clearance is demanded first; when the whole board offers nothing, the
    search reruns at half, then at a 0.02mm floor -- dense boards carry
    sub-clearance courtyard pairs BY DESIGN (the reference hand seed for the
    51x21 board places its LDO 0.04mm from a locked decap; a 0.05 floor
    still refused that board), and refusing to seed what a human
    deliberately packs would fail real boards. Courtyards carry their own
    margin, so a small courtyard-to-courtyard gap is not a copper hazard. A
    relaxed placement is a NOTE for the caller, never silent."""
    from pose_score import _offsets
    part = state.parts[ref]

    def _ok(x, y, rot):
        return pose_ok(state, ref, x, y, rot, exclude)

    _in_zone, anchor_zone = zone_gate(part, constraint, tol)
    if anchor_zone and info is not None:
        info['anchor_zone'] = True

    full = state.clearance
    try:
        for clr in (full, full / 2.0, min(0.02, full)):
            # candidate_valid reads state.clearance; the incumbent-violation
            # cache is keyed on it implicitly, so clear it on every change.
            state.clearance = clr
            state._inc_violation.clear()
            for rot in [part.rot] + [(part.rot + d) % 360
                                     for d in (90.0, 180.0, 270.0)]:
                xfine = max(0.05, getattr(state, 'grid_step', 0.1) or 0.1)
                for radius, step in ((SEARCH_RADIUS_MM, SEARCH_STEP_MM),
                                     (SEARCH_FINE_RADIUS_MM,
                                      SEARCH_FINE_STEP_MM),
                                     (SEARCH_XFINE_RADIUS_MM, xfine)):
                    if max_disp is not None and max_disp < step - 1e-9:
                        # run-7 A3: a ring whose step exceeds the cap can
                        # contribute nothing but used to burn a full sweep
                        continue
                    # Budget check at the RING head (36 per call: 3 clearance
                    # levels x 4 rotations x 3 bands) -- negligible, and it
                    # bounds one pathological part's overrun to a single ring
                    # sweep instead of a whole cap ladder. Deliberately NOT in
                    # the `for dx, dy in _offsets(...)` loop below: that is the
                    # innermost loop and _offsets already materialises ~3700
                    # tuples per call, so the band check bounds it adequately.
                    for dx, dy in _offsets(radius, step):
                        if (max_disp is not None
                                and math.hypot(dx, dy) > max_disp + 1e-9):
                            continue
                        x, y = round(tx + dx, 3), round(ty + dy, 3)
                        if not _in_zone(x, y, rot):
                            continue
                        if _ok(x, y, rot):
                            state.apply_move(ref, x, y, rot)
                            return clr
                if constraint is not None:
                    continue    # a zone-constrained part stays in its zone
                if max_disp is not None:
                    continue    # a capped repair never sweeps the whole board
                u = state.usable
                grid = []
                nx = max(1, int((u[2] - u[0]) / FALLBACK_STEP_MM))
                ny = max(1, int((u[3] - u[1]) / FALLBACK_STEP_MM))
                for i in range(nx + 1):
                    for j in range(ny + 1):
                        x = round(u[0] + i * FALLBACK_STEP_MM, 3)
                        y = round(u[1] + j * FALLBACK_STEP_MM, 3)
                        grid.append(((x - tx) ** 2 + (y - ty) ** 2, x, y))
                grid.sort()
                for _, x, y in grid:
                    if _ok(x, y, rot):
                        state.apply_move(ref, x, y, rot)
                        return clr
    finally:
        state.clearance = full
        state._inc_violation.clear()
    return None


def _edge_pose(part, bounds, edge: str, frac: float, overhang: float
               ) -> Tuple[float, float]:
    """Center coordinates that put the part's courtyard `overhang` mm past
    the named edge of the BOUNDING BOX, at fraction `frac` along it. A first
    guess only -- see _edge_correct for why it cannot be the answer."""
    lx0, ly0, lx1, ly1 = part.rect(0.0, 0.0, part.rot)
    x0, y0, x1, y1 = bounds
    if edge == 'north':
        return x0 + (x1 - x0) * frac, y0 - overhang - ly0
    if edge == 'south':
        return x0 + (x1 - x0) * frac, y1 + overhang - ly1
    if edge == 'west':
        return x0 - overhang - lx0, y0 + (y1 - y0) * frac
    if edge == 'east':
        return x1 + overhang - lx1, y0 + (y1 - y0) * frac
    raise ValueError(f"unknown edge {edge!r}")


def _edge_correct(state, ref: str, edge: str, x: float, y: float,
                  target: float) -> Tuple[float, float, bool]:
    """Walk the pose along the edge normal until the MEASURED overhang hits
    `target`. The analytic pose measures against the bounding box, but the
    grade's rule_edge_connector measures rect_outside_amount against the real
    Edge.Cuts rings -- on a non-rectangular outline the two differ by the
    local inset, and a seed placed by the bbox grades over its declared band
    (measured on splitflap: 4 connectors 0.1-0.2mm past their max).

    Returns (x, y, converged). **The third element is not decoration.** This
    walk moves along ONE axis while `rect_outside_amount` is a SUM over all
    four sides (`legality.EdgeGate.rect_outside_amount`), so any along-edge
    overshoot is a
    constant term the walk cannot cancel -- it subtracts it again every
    iteration and marches the part inland past the far edge. Measured on a
    41.16mm connector on a 50.8mm edge: frac 0.70 -> y 88.767 (off the
    opposite side), frac 0.90 -> y -2.443. It used to return that pose
    indistinguishably from a converged one, and the caller seated it.
    """
    part = state.parts[ref]
    converged = False
    for _ in range(4):
        amt = state.edge_gate.rect_outside_amount(part.rect(x, y, part.rot))
        err = target - amt
        if abs(err) < 0.02:
            converged = True
            break
        if edge == 'north':
            y -= err
        elif edge == 'south':
            y += err
        elif edge == 'west':
            x -= err
        else:
            x += err
    else:
        # Ran out of iterations. One last measurement decides it -- a walk
        # that happened to land on its target on the final step is converged.
        amt = state.edge_gate.rect_outside_amount(part.rect(x, y, part.rot))
        converged = abs(target - amt) < 0.02
    return x, y, converged


def edge_seat_ok(state, part, x: float, y: float, edge: str,
                 lo: float, hi: float,
                 reasons: Optional[List[str]] = None) -> bool:
    """Is this edge pose a seat, or is the part off the board?

    An edge seat is the one seat in the system that cannot use `pose_ok` --
    it overhangs by design, so full containment is the wrong predicate. This
    is the predicate it uses instead, and it has TWO parts because either
    alone was measured to accept an off-board part:

    * the measured overhang lies in the DECLARED band. Same quantity
      `floorplan.rule_edge_connector` grades on, so a seat accepted here is
      not a violation there.
    * EVERY PAD lands on the board. That is the invariant that actually
      matters -- CLAUDE.md calls pad copper outside the outline the
      top-priority placement defect, because it converts 1:1 into unrouted
      nets -- and it is the one a band cannot argue with. A band is an
      author's declaration and a wrong one is not rare: `{min: 3, max: 4}` on
      a 3.0mm-deep connector seated it with 1.7% of its courtyard on the
      board, and `{min: 20, max: 21}` was accepted on the east and west edges
      with 8 of 16 pads off.

    The pad test uses the gate's own containment, so a board with cutouts or
    milled rings is measured properly. A bounding-box test is not enough and
    was measured dropping 24 of 26 pads into a milled slot while reporting a
    seat: the pads were inside the bbox and inside a hole.

    An edge connector's BODY overhangs by design; its PADS do not. That
    asymmetry is what makes this checkable at all.

    A THIRD conjunct, since #701: a declared KEEP-OUT. An edge connector's
    body may leave the outline; it may not enter a region the intent
    reserved, and neither of the other two conjuncts can see that -- a
    mounting-hole keep-out on the north edge leaves the band satisfied and
    every pad on the board. This is the only place it can go: `pose_ok`
    demands full containment and an edge seat overhangs by design, so this
    predicate deliberately bypasses it, and BOTH edge paths come through
    here -- `_seat_edge`'s `on_board`, and stage 1 of `seed_from_intent`,
    which runs no legality gate at all by design. `reasons`, when given,
    collects WHY, so a refusal can name the keep-out instead of sending the
    reader to look at an outline that is not the problem.
    """
    r, tht = part.rects(x, y, part.rot)
    amt = state.edge_gate.rect_outside_amount(r)
    if not ((lo - 0.02) <= amt <= (hi + 0.02)):
        return False
    _blockers = state.keepout_blockers(part.ref, (r, tht))
    if _blockers:
        if reasons is not None:
            reasons.extend(f"keep-out {n!r}" for n in _blockers)
        return False
    gate = state.edge_gate
    for px, py, _sz in part.pad_globals(x, y, part.rot):
        # A zero-size rect at the pad centre: "is this point on the board",
        # asked through the gate so cutouts and milled rings count.
        if gate.rect_outside_amount((px, py, px, py)) > 1e-9:
            return False
    return True


def _edge_frac_bounds(part, bounds, edge: str) -> Tuple[float, float]:
    """The fractions along `edge` at which the part is still ON the board.

    `_edge_pose` places the part's CENTRE at `frac` along the edge's own span,
    and the old clamp was a bare [0.05, 0.95] that knew nothing of the part's
    width -- so a 41.16mm connector on a 50.80mm edge was slid to frac 0.70
    and hung 5.34mm past the end while reporting a seat. Only
    **[0.405, 0.595]** keeps that part on the board: the centre may range over
    span - width = 9.64mm, i.e. (width/2)/span = 0.405 in from each end.

    Returns (lo, hi); lo > hi means the part is wider than the edge, which is
    a real answer and the caller must treat it as "no legal fraction".
    """
    lx0, ly0, lx1, ly1 = part.rect(0.0, 0.0, part.rot)
    x0, y0, x1, y1 = bounds
    if edge in ('north', 'south'):
        span, a, b = (x1 - x0), lx0, lx1
    else:
        span, a, b = (y1 - y0), ly0, ly1
    span = max(1e-9, span)
    return (-a) / span, 1.0 - (b / span)


#: "Is this part over the boundary at all", in mm. MIRRORS `legality.EPS`
#: (1e-6), which is this repo's zero for a distance like this; spelled here
#: because `seeder` does not import `legality`, and reaching for a name this
#: module does not have is how the first version of `_already_on_its_edge`
#: came to answer False for every part on every board.
_ON_EDGE_EPS_MM = 1e-6


def _declared_frac(entry: Dict) -> Optional[float]:
    """The along-edge fraction the INTENT declares, or None (#706 + #712).

    `center_on_edge` is the midpoint. `along_edge_band`'s MIDPOINT, not an
    end: the caller's +/-0.05, +/-0.1 ... ladder searches outward, so starting
    at the middle reaches both ends of any band the ladder could have reached
    from either end, and it does so symmetrically.

    None whenever nothing is declared, which is what makes every consumer of
    this bit-identical to the code that predates it.
    """
    if entry.get('center_on_edge') is not None:
        return 0.5
    band = entry.get('along_edge_band')
    if band is not None:
        return (float(band['from']) + float(band['to'])) / 2.0
    return None


def _declared_frac_window(entry: Dict, span: float):
    """(lo, hi) fractions a declared claim allows, or None.

    `center_on_edge`'s tolerance is in MILLIMETRES and the window is in
    fractions, so it needs the span; a caller with no span passes 0 and gets
    None rather than a window computed from a number that is not there.
    """
    band = entry.get('along_edge_band')
    if band is not None:
        return float(band['from']), float(band['to'])
    centre = entry.get('center_on_edge')
    if centre is not None and span > 1e-9:
        t = float(centre['tolerance_mm']) / span
        return max(0.0, 0.5 - t), min(1.0, 0.5 + t)
    return None


def _centre_offset_frac(part, bounds, edge: str) -> float:
    """(rect centre - origin) along `edge`, as a fraction of the edge span.

    THE TWO SIDES SPEAK DIFFERENT CURRENCIES, and this is the conversion.
    `_edge_pose` positions the part's ORIGIN, while `rule_edge_connector`
    measures the courtyard RECT CENTRE -- and a footprint origin is usually
    not the centre of its own courtyard.

    Measured, and this is why the conversion exists rather than being assumed
    unnecessary: seating splitflap_driver's J17 into a declared band of
    0.80-0.90 put its origin at exactly frac 0.900 and its rect centre at
    0.9128 -- 2.54mm out -- so the seat search accepted a pose the grade then
    flagged as "91% along the edge, outside the declared band 80%-90%". That
    is the seeder placing to a rule the grader does not have, which is the
    failure #701 and #702 both exist to prevent, and it was caught by the
    seat-versus-grade agreement sweep rather than by reading the code.
    """
    lx0, ly0, lx1, ly1 = part.rect(0.0, 0.0, part.rot)
    x0, y0, x1, y1 = bounds
    if edge in ('north', 'south'):
        span, a, b = (x1 - x0), lx0, lx1
    else:
        span, a, b = (y1 - y0), ly0, ly1
    return ((a + b) / 2.0) / max(1e-9, span)


def _already_on_its_edge(state, part) -> bool:
    """Is this part already crossing the boundary it was declared on?

    Then its ORIENTATION is right and its refusal is a band, keep-out or
    neighbour problem -- turning it is guaranteed to be the wrong answer.

    Thresholdless: `> EPS` is the module's own zero. Measured on the tracked
    corpus, three parts have `_edge_frac_bounds` refusing at their current
    rotation while already overhanging their boundary --

        ulx3s   J1  (f_lo, f_hi) = (0.034, 0.015)  overhang 11.99mm  rot 270
        ulx3s   J2                (0.985, 0.966)   overhang 11.99mm  rot  90
        sonde_u J1                (0.626, 0.374)   overhang 26.55mm  rot 270

    -- all three full board-width connectors. A bare "no seat at the current
    rotation, so try 90/180/270" gate would turn every one of them sideways,
    which is a worse answer than today's honest refusal. With this guard the
    rotation loop is INERT on every tracked board.

    NO `try/except` here, and that is the point. The first version of this
    function wrapped the measurement in a bare `except Exception: return
    False`, and `legality` is not imported in this module -- so `legality.EPS`
    raised `NameError`, the except swallowed it, and the guard answered "not
    on its edge" for every part on every board. It read exactly like a guard
    and never fired once. A measurement that cannot be taken must raise, not
    return the permissive answer.
    """
    return state.edge_gate.rect_outside_amount(part.rect()) > _ON_EDGE_EPS_MM


def _seat_edge(state, ref: str, entry: Dict, must_lock: Set[str],
               notes: List[str], target=None, exclude=None) -> bool:
    """Seat a DECLARED edge part on its edge band, minimal-move (run-4 B-6).

    Repair could never do this: `_try_place._ok` demands full containment,
    and an edge seat overhangs by design -- so declared edge refs were
    exempt-only and a misplaced one was simply unrepairable here. Reuses the
    stage-1 geometry (`_edge_pose` + `_edge_correct`); the along-edge
    position starts at the part's CURRENT projection (minimal move) and
    slides outward until the seat is pad/hole-conflict-free against every
    other part. Board-only; the band comes from the intent."""
    part = state.parts[ref]
    edge = entry['edge']
    band = entry.get('overhang_mm') or {}
    lo = float(band.get('min', 0.0))
    hi = band.get('max')
    overhang = (lo + float(hi)) / 2.0 if hi is not None else max(lo, 0.5)
    x0, y0, x1, y1 = state.board

    # Along-edge start, in precedence order (#706):
    #   the intent's DECLARED position  >  the zone centre  >  minimal move.
    # Before #712 there was nothing to declare, so `df = 0.0` meant "keep the
    # along-edge coordinate you came in with" -- and on a repaired board that
    # coordinate is the damaged pose's own, carried verbatim into the output.
    declared = _declared_frac(entry)
    # A declared fraction is about the part's COURTYARD CENTRE, which is what
    # the grade measures; the ladder below positions its ORIGIN. Convert once,
    # here, so every use downstream is in the ladder's own currency.
    _c_off = _centre_offset_frac(part, state.board, edge)
    if declared is not None:
        cur = declared - _c_off
    else:
        ax, ay = target if target is not None else (part.x, part.y)
        if edge in ('north', 'south'):
            cur = (ax - x0) / max(1e-9, (x1 - x0))
        else:
            cur = (ay - y0) / max(1e-9, (y1 - y0))

    # Clamp by the part's OWN half-extent, not a bare [0.05, 0.95]. A part
    # wider than its edge has no legal fraction at all; say so rather than
    # sliding it off the end and reporting a seat.
    f_lo, f_hi = _edge_frac_bounds(part, state.board, edge)
    if f_lo > f_hi:
        notes.append(f"{ref}: is wider than the {edge} edge "
                     f"({f_lo:.2f} > {f_hi:.2f} of its span), so no along-edge "
                     f"position keeps it on the board")
        return False
    # A DECLARED window narrows the legal one. Without this the +/-0.4 ladder
    # could find a seat OUTSIDE the declared band, which `rule_edge_connector`
    # would then flag -- the search accepting a pose the grade refuses is the
    # round-trip break `edge_seat_ok` and `keepout_hit` both exist to prevent.
    span = ((x1 - x0) if edge in ('north', 'south') else (y1 - y0))
    win = _declared_frac_window(entry, span)
    if win is not None:
        # Same conversion as `cur`: the declared window is about the rect
        # centre, `f_lo`/`f_hi` are about the origin.
        w_lo, w_hi = win[0] - _c_off, win[1] - _c_off
        n_lo, n_hi = max(f_lo, w_lo), min(f_hi, w_hi)
        if n_lo > n_hi:
            notes.append(
                f"{ref}: the declared along-edge window "
                f"[{win[0]:.3f}, {win[1]:.3f}] does not intersect the legal one "
                f"[{f_lo + _c_off:.3f}, {f_hi + _c_off:.3f}] for this part "
                f"on the {edge} edge -- "
                f"widen the declaration, or the part is too wide for the "
                f"position it is declared at")
            return False
        f_lo, f_hi = n_lo, n_hi
    cur = min(f_hi, max(f_lo, cur))

    # The declared band. `hi` None means "no stated maximum" -- allow twice the
    # midpoint target, which is what `overhang` was derived from, rather than
    # allowing anything.
    hi_eff = float(hi) if hi is not None else max(2.0 * overhang, lo + 1.0)

    ctx = state.legality_ctx
    ex = set(exclude or ())

    def conflict_free(px, py, rot):
        if ctx is None:
            return True
        for other in ctx.parts:
            # `ex` is the pile: parts whose input coordinates are meaningless.
            # Without it a pile at the board centre vetoes the honest edge
            # poses and the loop SLIDES the part along the edge until one is
            # "free" -- measured, that is how a connector reached frac 0.70
            # and hung off the end.
            if other == ref or other in ex or other not in state.parts:
                continue
            sf = ctx.pair_shortfall(ref, other, pose_a=(px, py, rot))
            if sf.pad > 1e-6 or sf.hole > 1e-6:
                return False
        return True

    # #701: WHY the band refused, when it was a declared keep-out rather than
    # the outline. "no conflict-free seat found on the declared north edge"
    # sends the reader to look at the outline and the neighbours, neither of
    # which is the problem, and the next move is different: move the keep-out
    # or add an `allow`.
    refused: List[str] = []

    def on_board(px, py):
        return edge_seat_ok(state, part, px, py, edge, lo, hi_eff,
                            reasons=refused)

    # The ladder's steps are fractions of the WHOLE edge, which is the right
    # scale when it is sliding a part along a free edge and much too coarse
    # when it is searching inside a declared window. Measured: with a declared
    # band of 0.85-0.95 on splitflap_driver's 198.12mm north edge, the +/-0.05
    # rungs are 9.9mm apart and clamp to the window's two ends, so the ladder
    # tries THREE distinct positions in the band and misses the one legal seat
    # between them -- the feature reported "no seat" on every band of that
    # board. Scaling the rungs to the window searches it at the same relative
    # resolution the ladder gives a whole edge. 0.8 is the ladder's own full
    # sweep (+/-0.4), so an undeclared seat has scale exactly 1.0 and is
    # unchanged.
    _step = ((f_hi - f_lo) / 0.8) if win is not None else 1.0

    def try_rot(rot):
        """The seat ladder at one rotation. (x, y) or None.

        Extracted verbatim so the rotation loop below can re-run it; at the
        part's own rotation this executes exactly the statements the inline
        version did, against the same state, so a board that seats today
        seats identically.
        """
        for df in (0.0, 0.05, -0.05, 0.1, -0.1, 0.15, -0.15,
                   0.2, -0.2, 0.3, -0.3, 0.4, -0.4):
            frac = min(f_hi, max(f_lo, cur + df * _step))
            x, y = _edge_pose(part, state.board, edge, frac, overhang)
            x, y, converged = _edge_correct(state, ref, edge, x, y, overhang)
            if not converged or not on_board(x, y):
                continue
            if conflict_free(x, y, rot):
                return x, y
        return None

    seat = try_rot(part.rot)
    if seat is not None:
        state.apply_move(ref, round(seat[0], 3), round(seat[1], 3), part.rot)
        return True

    # #706, the rotation half. THREE gates, and the third is a deliberate
    # addition to what the issue asked for:
    #
    #  1. the ladder above already ran at the part's own rotation and found
    #     nothing, so this can only fire where the function returns False
    #     today -- the issue's own guard against turning a part whose
    #     orientation was deliberate;
    #  2. `_already_on_its_edge` -- see its docstring for the three corpus
    #     parts a bare gate would have turned sideways;
    #  3. the intent must actually DECLARE a position for this part.
    #
    # (3) is not in the issue, and it was added because the probe caught the
    # loop firing without it: driven over a lattice of incoming poses for an
    # UNDECLARED splitflap_driver J17, the seat at start fraction 0.3 came
    # back rotated 0 -> 90 degrees. Nothing in this tree knows which way a
    # mating face must point -- `edge_seat_ok` tests the overhang band, the
    # pads being on the board and the keep-outs, and `part_class` has no
    # orientation concept at all -- so turning a connector is only defensible
    # where a human declared where it belongs, and that declaration is the
    # evidence that its current orientation is not itself the requirement.
    # Without (3) this PR's contract, inert unless something is declared,
    # would be false.
    #
    # Stay on the part's OWN 90-degree lattice (`rot + 90k`, not a fixed
    # list): a part seeded at 45 degrees must rotate to 135/225/315, not be
    # snapped onto the axes.
    if declared is not None and not _already_on_its_edge(state, part):
        # Captured BEFORE the loop: `state.apply_move` mutates `part.rot`, so
        # reading it afterwards reports the new angle as the old one and the
        # note says "at its own rotation 90deg; seated at 90deg".
        was_rot = part.rot
        for rot in ((was_rot + 90) % 360, (was_rot + 180) % 360,
                    (was_rot + 270) % 360):
            seat = try_rot(rot)
            if seat is None:
                continue
            state.apply_move(ref, round(seat[0], 3), round(seat[1], 3), rot)
            notes.append(
                f"{ref}: no seat existed on the declared {edge} edge at its "
                f"own rotation {was_rot:g}deg; seated at {rot:g}deg. CHECK "
                f"THIS -- a rotation changes the PART, not only where it is, "
                f"and nothing here knows which way the mating face must point")
            return True
    if refused:
        # Sorted+deduped: the ladder tries up to 13 fractions and would
        # otherwise name the same keep-out 13 times.
        notes.append(f"{ref}: every position on the declared {edge} edge band "
                     f"is refused by " + ', '.join(sorted(set(refused)))
                     + " -- move the keep-out, or add this ref to its `allow`")
    return False


def _partner_centroid(state, ref: str, placed: Set[str],
                      max_fanout: int = 20) -> Optional[Tuple[float, float]]:
    """Centroid of already-placed partners on shared nets: ONE vote per
    (partner footprint, net), each vote the mean of that partner's matching
    pads. Voting per PAD (the pre-run-7 behavior) let a partner with
    duplicated pins outvote one with a single pin -- a USB-C receptacle's
    doubled A6/B6 DP pads pulled the 27R series pair 2:1 toward the
    connector, seating R7 15.3mm from the U1 face the intent named. Nets
    owned by more than `max_fanout` parts are excluded for the
    routability.py reason: they reach everywhere by design and would
    collapse every centroid onto the board middle. Plane nets are NOT
    otherwise excluded here -- for a decap, the rail net is exactly what
    tethers it to its IC."""
    part = state.parts.get(ref)
    if part is None:
        return None
    xs: List[float] = []
    ys: List[float] = []
    for nid in part.nets:
        owners = state.net_refs.get(nid, ())
        if len(owners) > max_fanout:
            continue
        for other in owners:
            if other == ref or other not in placed:
                continue
            pxs = [gx for gx, gy, pn in state.parts[other].pad_globals()
                   if pn == nid]
            pys = [gy for gx, gy, pn in state.parts[other].pad_globals()
                   if pn == nid]
            if pxs:
                xs.append(sum(pxs) / len(pxs))
                ys.append(sum(pys) / len(pys))
    if not xs:
        return None
    return sum(xs) / len(xs), sum(ys) / len(ys)


def seed_from_intent(pcb_data, pcb_file: str, intent, rng: random.Random, *,
                     group_sources: Sequence[str] = (),
                     clearance: float = 0.25,
                     board_edge_clearance: float = 0.55,
                     grid_step: float = 0.1,
                     seed_refs: Optional[Set[str]] = None,
                     anchors_first: bool = False,
                     anchor_rounds: int = 1,
                     evict_depth: int = 0,
                     immovable_extra: Sequence[str] = ()) -> Dict:
    """Compute a full placement for an unplaced board from its intent.

    Returns {'placements': [...], 'lock_refs': [...], 'unseated': [...],
    'notes': [...]}. `placements` covers every ref that was placed (writer
    format); `unseated` names parts NO legal pose was found for -- the caller
    reports them and the grade fails, deliberately.

    `seed_refs`, when given, scopes the seeding to exactly those refs: every
    other part is treated as authoritatively placed where it stands (the
    PARTIALLY-unplaced case -- a stacked pile beside a real placement, where
    re-deriving the placed parts would discard someone's work, and the
    LIFT-AND-RE-SEAT case -- see `reseat_scope`).

    `evict_depth` arms the eviction rung (stage 3c, #630): 0 (the default)
    censuses the blockers of every unseated part and moves nothing; 1 also
    trades the best SINGLE blocker out and back under `_evict_trade`'s
    acceptance rule; 2 additionally censuses PAIRS when no single lift frees
    a pose, and trades the best pair (#699). Opt-in until an A/B row on three
    boards exists (CLAUDE.md, "A new PLACEMENT objective term"). Nothing
    deeper is defined -- depth 3 raises rather than silently meaning 2.

    `immovable_extra` names seated refs the eviction rung may not lift on top
    of the intent's own locks. It exists because a caller's lock is not
    always IN the intent: `reseat_scope` resolves `--lock` globs into its own
    state and the seeder builds a fresh one, so without this the rung would
    happily evict a ref the user locked by name. It is deliberately NOT
    laundered through `must_lock`, which also drives stage 1.5, the
    file-lock/zone contradiction note and the `lock_refs` this returns (which
    `place_seed` STAMPS into the board).
    """
    if evict_depth not in (0, 1, 2):
        raise ValueError(
            f"evict_depth must be 0, 1 or 2, got {evict_depth!r}")
    import pose_score
    from placement import floorplan

    state = pose_score.make_state(
        pcb_data, pcb_file, clearance=clearance,
        board_edge_clearance=board_edge_clearance, grid_step=grid_step,
        # #701: the declared keep-outs reach the SEAT PREDICATE here, and
        # every `_try_place` / `count_legal_poses` / `_evict_trade` site in
        # this module inherits them through `pose_ok`.
        keepouts=intent.keepouts if intent else ())
    bounds = state.board
    refs_all = sorted(pcb_data.footprints)
    notes: List[str] = []

    blocks, block_problems = floorplan.resolve_blocks(
        intent, pcb_data, group_sources)
    for v in block_problems:
        notes.append(v.message)
    zones_by_name = {z.name: z for z in intent.blocks if z.rect is not None}

    lock_refs: List[str] = sorted({
        r for pat in intent.must_lock for r in fnmatch.filter(refs_all, pat)})

    placed: Set[str] = set()
    unplaced: Set[str] = {r for r, p in state.parts.items()}
    unseated: List[str] = []
    # ref -> (target_x, target_y, constraint_rect, tol) for the seat that
    # failed. The eviction rung (3c) retries at exactly the target the part
    # was refused at; a rung that re-derived one would be answering a
    # different question from the one that failed.
    unseated_ctx: Dict[str, Tuple[float, float, Any, float]] = {}
    evictions: List[Dict] = []
    no_pose_blockers: Dict[str, Dict[str, int]] = {}
    # #699: WHY a part has no pose, and what the census actually looked at.
    # `no_pose_blockers` alone cannot say it -- an empty dict there means
    # both "nothing is near it" and "everything near it is locked".
    no_pose_verdict: Dict[str, str] = {}
    no_pose_census: Dict[str, Dict] = {}
    # A part locked IN THE FILE is already authoritatively placed -- a caller
    # that pre-placed its spec-fixed parts and stamped them (locked yes) must
    # not have the seeder re-derive them. Treated as placed from the start:
    # they anchor the connectivity centroids and obstruct packing, and every
    # later stage (edge connectors included) skips them. The same applies to
    # every ref outside an explicit `seed_refs` scope.
    for ref in sorted(state.parts):
        if state.parts[ref].locked or (seed_refs is not None
                                       and ref not in seed_refs):
            placed.add(ref)
            unplaced.discard(ref)
    # Deterministic tie-break values, drawn once in sorted order so the
    # stream never depends on set iteration.
    tiebreak = {r: rng.random() for r in sorted(state.parts)}

    def _order(refs):
        return sorted((r for r in refs if r in unplaced),
                      key=lambda r: (-state.parts[r].pin_count, tiebreak[r]))

    def _jitter():
        return (rng.uniform(-TARGET_JITTER_MM, TARGET_JITTER_MM),
                rng.uniform(-TARGET_JITTER_MM, TARGET_JITTER_MM))

    # ---- 1. edge connectors: spec geometry, no legality gate ---------------
    # edge_claims(), not the raw key: a connector_affinity entry declares a
    # class and makes no seat claim, so stage 1 has nothing to seat it from.
    # Reading the raw key printed "edge connector J7: no edge declared" at a
    # part that never claimed one.
    by_edge: Dict[str, List[Dict]] = {}
    for c in intent.edge_claims():
        if c['ref'] not in state.parts:
            notes.append(f"edge connector {c['ref']} is not on this board")
        elif c['ref'] in unplaced:
            if not c.get('edge'):
                # Run-4 A: an entry with no edge used to default to SOUTH --
                # an auto-declared receptacle whose true edge is underivable
                # (implausible pose, run 3's J1) would have been seated on a
                # wrong edge silently. No edge, no seat: say so and leave the
                # part to the later stages / reconstruct.
                notes.append(f"edge connector {c['ref']}: no edge declared; "
                             f"stage 1 will not guess one (it used to default "
                             f"to south) -- the centroid stage places it")
                continue
            by_edge.setdefault(c['edge'], []).append(c)
    for edge in sorted(by_edge):
        specs = sorted(by_edge[edge], key=lambda c: c['ref'])
        for k, c in enumerate(specs):
            ref = c['ref']
            part = state.parts[ref]
            band = c.get('overhang_mm') or {}
            lo = float(band.get('min', 0.0))
            hi = band.get('max')
            overhang = (lo + float(hi)) / 2.0 if hi is not None else max(lo, 0.5)
            # Even distribution, but clamped by the part's own half-extent:
            # 3 connectors on one edge get fracs 0.25/0.5/0.75, and a wide
            # part at 0.25 hangs off the end. This stage runs no legality
            # gate at all (by design), so nothing downstream would catch it.
            f_lo, f_hi = _edge_frac_bounds(part, bounds, edge)
            # #706/#712. A DECLARED position outranks the even distribution.
            # Stage 1 is the from-scratch path and it never calls `_seat_edge`,
            # so without this a declared `center_on_edge` would be seated at
            # (k+1)/(n+1) -- on splitflap_driver's six north connectors that is
            # 1/7 .. 6/7 and NONE of them is 0.5 -- and `place_seed` would then
            # exit 4 grading its own output against the intent it was built
            # from. Inert when nothing is declared: `_declared_frac` is None.
            _dec = _declared_frac(c)
            frac = _dec if _dec is not None else (k + 1) / (len(specs) + 1)
            if f_lo > f_hi:
                notes.append(f"edge connector {ref}: wider than the {edge} "
                             f"edge, so stage 1 leaves it to the later stages")
                continue
            _win = _declared_frac_window(
                c, (bounds[2] - bounds[0]) if edge in ('north', 'south')
                else (bounds[3] - bounds[1]))
            if _win is not None:
                _n_lo, _n_hi = max(f_lo, _win[0]), min(f_hi, _win[1])
                if _n_lo > _n_hi:
                    notes.append(
                        f"edge connector {ref}: the declared along-edge window "
                        f"[{_win[0]:.3f}, {_win[1]:.3f}] does not intersect the "
                        f"legal one [{f_lo:.3f}, {f_hi:.3f}] on the {edge} "
                        f"edge, so stage 1 leaves it to the later stages")
                    continue
                f_lo, f_hi = _n_lo, _n_hi
            frac = min(f_hi, max(f_lo, frac))
            # #701: SLIDE along the edge when a declared keep-out refuses the
            # even-distribution position, using the same ladder `_seat_edge`
            # already uses. Without it, one keep-out over the middle of an
            # edge sent the connector to the ordinary stages, which park it in
            # the board INTERIOR -- measured: J1 written at (11.22, 6.589) on
            # a board whose south edge is y=14, trading a `keepout` grade
            # error for an `edge_connector` one, while 26 clear south-edge
            # seats existed. The ladder is skipped entirely when nothing is
            # declared, so a board with no keep-out is unchanged.
            # #706: armed for a DECLARED position too, not only a keep-out.
            # Two connectors declared at overlapping positions must slide off
            # each other rather than one being dropped to the later stages,
            # which park a connector in the board INTERIOR. Still skipped
            # entirely when nothing is declared and no keep-out exists, so a
            # board that declares nothing is unchanged.
            _slide = ((0.0,) if not (state.keepouts_for.get(ref)
                                     or _dec is not None) else
                      (0.0, 0.05, -0.05, 0.1, -0.1, 0.15, -0.15,
                       0.2, -0.2, 0.3, -0.3, 0.4, -0.4))
            _base_frac = frac
            _why: List[str] = []
            for _df in _slide:
                frac = min(f_hi, max(f_lo, _base_frac + _df))
                _x, _y = _edge_pose(part, bounds, edge, frac, overhang)
                _x, _y, _conv = _edge_correct(state, ref, edge, _x, _y,
                                              overhang)
                _why = []
                if _conv and edge_seat_ok(state, part, _x, _y, edge, lo,
                                          float(hi) if hi is not None
                                          else max(2.0 * overhang, lo + 1.0),
                                          reasons=_why):
                    break
            x, y = _edge_pose(part, bounds, edge, frac, overhang)
            x, y, converged = _edge_correct(state, ref, edge, x, y, overhang)
            if not converged:
                # The walk diverged (it drives a scalar SUM along one axis, so
                # an along-edge overshoot never cancels). It used to
                # apply_move unconditionally, which is how a diverged stage-1
                # seat reached the board silently.
                notes.append(f"edge connector {ref}: the overhang walk did "
                             f"not converge on the {edge} edge, so stage 1 "
                             f"left it for the later stages")
                continue
            # The SAME containment predicate _seat_edge uses. Stage 1 got the
            # fraction clamp and the convergence skip but not this, and was
            # measured still seating a connector at (159.909, 132.830) with
            # 16 of 16 pads 18.45mm off a board ending at 114.38 -- the exact
            # pose the fix elsewhere refuses. Stage 1 runs no legality gate at
            # all by design, so nothing downstream catches it.
            hi_eff = float(hi) if hi is not None else max(2.0 * overhang,
                                                          lo + 1.0)
            _why: List[str] = []
            if not edge_seat_ok(state, part, x, y, edge, lo, hi_eff,
                                reasons=_why):
                # #701: a keep-out refusal has a DIFFERENT next move from an
                # off-board one -- move the keep-out, not the band -- so it is
                # named rather than folded into "would put it off the board".
                notes.append(f"edge connector {ref}: "
                             + (f"the {edge} band is refused by "
                                + ', '.join(sorted(set(_why))) if _why else
                                f"the {edge} band would put it off the board")
                             + ", so stage 1 left it for the later stages")
                continue
            state.apply_move(ref, round(x, 3), round(y, 3), part.rot)
            placed.add(ref)
            unplaced.discard(ref)

    # ---- 1.5 must_lock parts seat FIRST, in place when possible ------------
    # Under --force, previously-good must_lock parts used to be re-derived at
    # connectivity centroids with everything else. They are the spec-fixed
    # parts: seat them before anything else, targeted at their CURRENT pose
    # (the nearest-first ring keeps a legal current pose at 0mm), constrained
    # to their declared zone when they have one; fall back to the zone center
    # when the current pose is nowhere near the zone.
    ref_zone: Dict[str, object] = {}
    for z in intent.blocks:
        if z.rect is None:
            continue
        for r in blocks.get(z.name, ()):
            ref_zone.setdefault(r, z)
    for ref in _order(lock_refs):
        part = state.parts[ref]
        z = ref_zone.get(ref)
        rect = z.rect if z is not None else None
        tol = intent.zone_tolerance(z) if z is not None else 0.5
        info: Dict = {}
        clr = _try_place(state, ref, part.x, part.y, unplaced - {ref},
                         constraint=rect, tol=tol, info=info)
        if clr is None and z is not None:
            zx = (z.rect[0] + z.rect[2]) / 2.0
            zy = (z.rect[1] + z.rect[3]) / 2.0
            clr = _try_place(state, ref, zx, zy, unplaced - {ref},
                             constraint=rect, tol=tol, info=info)
        if clr is not None:
            placed.add(ref)
            unplaced.discard(ref)
            if info.get('anchor_zone'):
                notes.append(f"{ref}: zone smaller than the courtyard -- "
                             f"seated by anchor point (spec-coordinate zone)")
        # An unseated must_lock ref falls through to the ordinary stages and,
        # failing there too, lands in `unseated` with its own note.

    # A part locked IN THE FILE that violates its declared zone is a
    # contradiction this tool must not resolve by force: the file lock is the
    # user's. Say it precisely instead of failing the grade mysteriously.
    for ref in sorted(set(ref_zone) - unplaced - set(lock_refs)):
        if ref not in state.parts or not state.parts[ref].locked:
            continue
        z = ref_zone[ref]
        part = state.parts[ref]
        tol = intent.zone_tolerance(z)
        from placement.floorplan import zone_fits_courtyard, _rect_escape
        r = part.rect()
        if zone_fits_courtyard(z.rect, r, tol):
            out, _axis = _rect_escape(z.rect, r)
        else:
            cx, cy = (r[0] + r[2]) / 2.0, (r[1] + r[3]) / 2.0
            out, _axis = _rect_escape(z.rect, (cx, cy, cx, cy))
        if out > tol:
            notes.append(
                f"{ref} is (locked yes) IN THE FILE at a pose violating its "
                f"declared zone {z.name!r} by {out:.2f}mm -- the file lock is "
                f"not this tool's to override; unlock it or fix the zone")

    # Decap-governed caps are claimed by stage 2.5, never zone-packed: a
    # zone is a REGION and the decap rule is a distance to a specific pin --
    # a cap packed anywhere in a 15x9 zone routinely lands >3mm from the pin
    # it exists to serve (measured: the flash decap, zone-packed, graded
    # 3.5mm from the flash's VCC pin).
    decap_spec = getattr(intent, 'decaps', None) or {}
    decap_scope: Set[str] = set()
    if decap_spec.get('max_distance_mm') is not None:
        exempt = tuple(decap_spec.get('exempt') or ())
        decap_scope = {r for r in state.parts
                       if r[0] == 'C' and state.parts[r].pin_count == 2
                       and not any(fnmatch.fnmatch(r, pat) for pat in exempt)}

    # ---- 2. zoned blocks: radial pack from the zone center -----------------
    # A single-member zone is the spec-coordinate pattern (a rect a few
    # hundred microns wide around where the spec pins the part), so it gets
    # the exact center; multi-member zones jitter each target so different
    # seeds pack differently.
    for name in sorted(zones_by_name):
        z = zones_by_name[name]
        members = [r for r in _order(blocks.get(name, ()))
                   if r not in decap_scope]
        if not members:
            continue
        cx = (z.rect[0] + z.rect[2]) / 2.0
        cy = (z.rect[1] + z.rect[3]) / 2.0
        tol = intent.zone_tolerance(z)
        for ref in members:
            jx, jy = (0.0, 0.0) if len(members) == 1 else _jitter()
            rot_before = state.parts[ref].rot
            zinfo: Dict = {}
            clr = _try_place(state, ref, cx + jx, cy + jy, unplaced - {ref},
                             constraint=z.rect, tol=tol, info=zinfo)
            if clr is not None:
                placed.add(ref)
                unplaced.discard(ref)
                if zinfo.get('anchor_zone'):
                    notes.append(f"{ref}: zone {name!r} smaller than the "
                                 f"courtyard -- seated by anchor point")
                if state.parts[ref].rot != rot_before:
                    notes.append(f"{ref}: rotated {rot_before:g} -> "
                                 f"{state.parts[ref].rot:g} (no contained "
                                 f"pose at the input rotation)")
                if clr < state.clearance:
                    notes.append(f"{ref}: placed at reduced courtyard "
                                 f"clearance {clr:g} (none at "
                                 f"{state.clearance:g})")
            else:
                unseated.append(ref)
                unseated_ctx[ref] = (cx + jx, cy + jy, z.rect, tol)
                notes.append(f"{ref}: no legal pose inside zone {name!r}")

    # ---- 2.5 decap-governed caps: one cap per supply PIN -------------------
    # A 100nF's two nets are a rail and GND -- both usually above the fanout
    # cap -- so the generic centroid stage would park every decap mid-board
    # and a pin-exact decap gate (3mm pad-edge per SUPPLY PIN) would fail.
    # The iteration is PIN-FIRST, not cap-first: a cap-first greedy spends
    # every cap on the biggest IC's pads and starves the flash (measured --
    # all ten caps claimed U1 pads, U3.8 graded 3.5mm). Pins are the rail
    # pads of PLACED ICs (U-prefix: a castellated row carries the rail too
    # and must not eat a claim), pair-collapsed (adjacent same-rail pins
    # under 1mm share one cap by design), biggest owner first; each pin
    # takes a matching-rail cap, preferring one whose declared zone CONTAINS
    # the pin so a zone-member cap serves its own block.
    if decap_scope:
        avail = [r for r in _order(sorted(unplaced)) if r in decap_scope]
        rail_of: Dict[str, int] = {}
        for ref in avail:
            rail = min((nid for nid in state.parts[ref].nets
                        if len(state.net_refs.get(nid, ())) >= 2),
                       key=lambda nid: (len(state.net_refs[nid]), nid),
                       default=None)
            if rail is not None:
                rail_of[ref] = rail
        rails = set(rail_of.values())
        pins: List[Tuple[int, str, float, float, int]] = []
        for owner in sorted(placed):
            if owner[0] != 'U':
                continue
            o = state.parts[owner]
            for gx, gy, pn in o.pad_globals():
                if pn in rails:
                    pins.append((-o.pin_count, owner, round(gx, 3),
                                 round(gy, 3), pn))
        pins.sort()
        zone_of_cap = {}
        for name in sorted(zones_by_name):
            for r in blocks.get(name, ()):
                if r in decap_scope and r not in zone_of_cap:
                    zone_of_cap[r] = zones_by_name[name]

        def _seat(ref, tx, ty, owner, pn, constraint=None, tol=0.5):
            clr = _try_place(state, ref, tx, ty, unplaced - {ref},
                             constraint=constraint, tol=tol)
            if clr is None and constraint is not None:
                clr = _try_place(state, ref, tx, ty, unplaced - {ref})
            if clr is None:
                return False
            avail.remove(ref)
            placed.add(ref)
            unplaced.discard(ref)
            p2 = state.parts[ref]
            net = getattr(pcb_data.nets.get(pn), 'name', pn)
            notes.append(f"{ref}: decap for {owner} pad(s) near ({tx}, {ty})"
                         f" [{net}], landed "
                         f"{math.hypot(p2.x - tx, p2.y - ty):.2f}mm"
                         + (f" at reduced clearance {clr:g}"
                            if clr < state.clearance else ""))
            return True

        # Pass 1: a cap declared in a zone serves a pin INSIDE that zone --
        # the flash's own decap covers the flash, whatever the owner-size
        # ordering says.
        remaining = []
        for key in pins:
            _, owner, x, y, pn = key
            hit = next((r for r in avail if rail_of.get(r) == pn
                        and zone_of_cap.get(r) is not None
                        and zone_of_cap[r].rect[0] <= x <= zone_of_cap[r].rect[2]
                        and zone_of_cap[r].rect[1] <= y <= zone_of_cap[r].rect[3]),
                       None)
            if hit is not None and _seat(
                    hit, x, y, owner, pn,
                    constraint=zone_of_cap[hit].rect,
                    tol=intent.zone_tolerance(zone_of_cap[hit])):
                continue
            remaining.append(key)

        # Pass 2, per rail: CLUSTER the remaining pins until they fit the
        # remaining caps, then seat one cap per cluster centroid. Twelve
        # graded pins over ten caps is the DESIGNED shape -- adjacent supply
        # pins share a cap -- so the collapse radius grows (1.0 -> 3.0mm)
        # until every pin belongs to a served cluster; a fixed radius either
        # starves the last pin or wastes two caps on one pair (both
        # measured).
        for rail in sorted(rails):
            pins_r = [k for k in remaining if k[4] == rail]
            caps_r = [r for r in avail if rail_of.get(r) == rail]
            if not pins_r or not caps_r:
                continue
            radius = 1.0
            while True:
                clusters: List[List[Tuple]] = []
                for key in pins_r:
                    _, owner, x, y, pn = key
                    home = next((c for c in clusters
                                 if c[0][1] == owner
                                 and math.hypot(x - c[0][2], y - c[0][3])
                                 <= radius), None)
                    (home.append(key) if home is not None
                     else clusters.append([key]))
                if len(clusters) <= len(caps_r) or radius >= 3.0:
                    break
                radius += 0.5
            for cluster, ref in zip(clusters, list(caps_r)):
                cx2 = round(sum(k[2] for k in cluster) / len(cluster), 3)
                cy2 = round(sum(k[3] for k in cluster) / len(cluster), 3)
                _seat(ref, cx2, cy2, cluster[0][1], rail)
            # pins beyond the cap supply, and caps no pin wanted, fall
            # through to the generic stage, which reports honestly

    # ---- 3. the rest: connectivity centroid --------------------------------
    # --anchors-first (run-4 C): the default queue is pin-count descending,
    # which seeds a LARGE low-pin part (a connector shell, a big switch)
    # late, after the smalls have claimed its space -- and "nothing in the
    # placement code orders by size" was the skill's own measured complaint.
    # The mode seeds the anchor tier (pad-extent >= the P75 threshold, the
    # same tiering reconstruct uses) by DESCENDING EXTENT first; the smalls
    # are already parked as non-obstacles by the existing `exclude` set, so
    # anchors place against anchors only. Everything else is unchanged.
    center = ((bounds[0] + bounds[2]) / 2.0, (bounds[1] + bounds[3]) / 2.0)
    queue = _order(sorted(unplaced))
    if anchors_first and unplaced:
        from placement.reconstruct import part_extent_mm
        exts = sorted(part_extent_mm(state, r) for r in unplaced)
        thr = max(3.5, exts[int(0.75 * (len(exts) - 1))]) if exts else 3.5
        anchors = sorted((r for r in unplaced
                          if part_extent_mm(state, r) >= thr),
                         key=lambda r: -part_extent_mm(state, r))
        notes.append(f"anchors-first: {len(anchors)} anchor(s) (extent >= "
                     f"{thr:.2f}mm) seed before {len(unplaced) - len(anchors)}"
                     f" small(s): {', '.join(anchors)}")
        queue = anchors + [r for r in queue if r not in set(anchors)]
    for ref in queue:
        target = _partner_centroid(state, ref, placed) or center
        jx, jy = _jitter()
        rot_before = state.parts[ref].rot
        clr = _try_place(state, ref, target[0] + jx, target[1] + jy,
                         unplaced - {ref})
        if clr is not None:
            placed.add(ref)
            unplaced.discard(ref)
            if state.parts[ref].rot != rot_before:
                notes.append(f"{ref}: rotated {rot_before:g} -> "
                             f"{state.parts[ref].rot:g} (no contained pose "
                             f"at the input rotation)")
            if clr < state.clearance:
                notes.append(f"{ref}: placed at reduced courtyard clearance "
                             f"{clr:g} (none at {state.clearance:g})")
        else:
            unseated.append(ref)
            # setdefault: a zone member that failed its zone stage keeps THAT
            # context, so the rung retries it inside its zone rather than at
            # this unconstrained centroid (a seat outside the zone would fail
            # the grade anyway).
            unseated_ctx.setdefault(
                ref, (target[0] + jx, target[1] + jy, None, 0.5))
            notes.append(f"{ref}: no legal pose anywhere on the board")

    # ---- 3c. eviction rung (#630): census the blockers, evict, retry --------
    # A part with no legal pose is not necessarily a part with no ROOM. Run 19
    # measured the difference: three sweeps returned a bare "no legal pose
    # anywhere on the board" for SW17/SW34, and when the question was finally
    # asked in scoped form the engine answered precisely -- with D14 in place
    # 0 poses, with D14 lifted 46; with D31, 0 then 32. One eviction each and
    # both seated. That verdict was reachable the whole time; nothing asked.
    #
    # So: for each part this seed could not seat, count its poses with each
    # nearby incumbent lifted in turn (the CENSUS, which runs at every depth
    # and is what `no_pose_blockers` reports), and at depth 1 evict the one
    # that frees the most and retry. THE ORDERING IS LOAD-BEARING -- the
    # blocked part is seated FIRST, against a board the blocker is lifted
    # out of, and the blocker is re-seated afterwards with it as an
    # obstacle. Run 19's one-call reseat got a null three times precisely
    # because its queue re-seated the blockers first, back into the pockets
    # they block. The trade itself, and the rule that accepts or reverts
    # it, is `_evict_trade`.
    #
    # At depth 2 the same question is asked of PAIRS (#699), but ONLY for a
    # part no single lift helped: a rung that lifts one neighbour at a time
    # writes down "immovable" for a part two neighbours jointly block, and
    # that verdict is true only of the basin the board is in. The pair sweep
    # cannot be pruned to the candidates that scored well singly -- in the
    # case it exists for, every single lift frees exactly zero.
    #
    # Bounded on every axis: no recursion at either depth (a blocker's own
    # blocker is not chased), at most EVICT_MAX_BLOCKERS candidates per part,
    # at most EVICT_MAX_PAIRS of the pairs they form, ONE trade per part (a
    # single lift that was useful but whose trade reverted does NOT fall back
    # to a pair), and the census counts to a cap.
    if unseated:
        # Not this rung's to lift: the intent's locks, and its declared edge
        # connectors (see `_evict_candidates`). An edge_connector entry with
        # no `edge` key is not protected, consistent with stage 1: it seats
        # no such entry ("no edge, no seat") and leaves it to the ordinary
        # stages, so it is an ordinary part here too.
        # {ref: why}, not a bare set, so a frozen neighbour can be reported
        # with the decision that froze it -- the reader's next move differs
        # for a must_lock and for a declared edge connector.
        immovable = {r: 'must_lock' for r in lock_refs}
        immovable.update({c['ref']: 'edge_connector'
                          for c in intent.edge_claims() if c.get('edge')})
        immovable.update({r: 'lock-glob' for r in immovable_extra
                          if r not in immovable})
        still: List[str] = []
        # DEDUPED, and placed-aware. A zone member that fails its zone stage
        # stays in `unplaced`, so stage 3 tries it again and appends it a
        # SECOND time -- `unseated` can name one part twice. Iterating that
        # raw ran the rung again on a part the first pass had just seated,
        # where the trade is now a pure loss, and the revert put it back in
        # `unseated`: a success undone by a duplicate.
        seen: Set[str] = set()
        for ref in list(unseated):
            if ref in seen:
                continue
            seen.add(ref)
            if ref in placed:
                continue          # an earlier pass of this rung seated it
            if ref not in unseated_ctx or ref not in state.parts:
                # The rung never got to ask. This landed in `unseated` with
                # NO ledger entry at all before #699 -- the same "a verdict
                # you cannot act on" the census exists to end, one level down.
                still.append(ref)
                no_pose_verdict[ref] = 'no_target_recorded'
                no_pose_census[ref] = _empty_census()
                continue
            tx, ty, constraint, tol = unseated_ctx[ref]
            base_excl = unplaced - {ref}
            # The census and the retry answer the SAME question: both carry
            # the zone the part was refused in. A census over the open board
            # about a part that must land in a zone is a different question,
            # and this one reaches `place_seed`'s JSON_SUMMARY.
            zkw = dict(constraint=constraint, tol=tol)
            baseline = count_legal_poses(state, ref, tx, ty, base_excl, **zkw)
            # #701: which DECLARED KEEP-OUT is refusing this part, measured
            # the way a blocker is -- count the poses with it lifted. Only
            # when the part has no pose at all (nothing to explain otherwise)
            # and only over the keep-outs that BIND it, so a board declaring
            # none pays nothing and each extra census is already capped by
            # CENSUS_CAP.
            keepouts_freeing: Dict[str, int] = {}
            keepouts_joint = 0
            if not baseline:
                _bound = state.keepouts_for.get(ref, ())
                for _k in _bound:
                    _n = count_legal_poses(state, ref, tx, ty, base_excl,
                                           without_keepouts=(_k['name'],),
                                           **zkw)
                    if _n > baseline:
                        keepouts_freeing[_k['name']] = _n
                # JOINTLY blocked: two keep-outs that overlap over the part's
                # feasible region each free nothing ALONE, so the per-keep-out
                # sweep above reports {} and the verdict would fall back to
                # `no_movable_neighbour` -- whose prose ("nothing seated is
                # near enough to be in the way -- the outline, the zone or its
                # own size refuses it") is exactly the misleading answer this
                # whole disclosure exists to replace. Measured on a nested
                # enclosure+boss pair. One extra census, only for a part no
                # single lift explained, mirroring the blocker side's own
                # single-then-pair escalation.
                if not keepouts_freeing and len(_bound) > 1:
                    keepouts_joint = count_legal_poses(
                        state, ref, tx, ty, base_excl,
                        without_keepouts=tuple(k['name'] for k in _bound),
                        **zkw)
            cinfo: Dict = {}
            cands = _evict_candidates(state, ref, tx, ty, placed, immovable,
                                      constraint=constraint, tol=tol,
                                      info=cinfo)
            freed = {b: count_legal_poses(state, ref, tx, ty,
                                          base_excl | {b}, **zkw)
                     for b in cands}
            no_pose_blockers[ref] = dict(freed)
            # Stored by reference on purpose: the pair sweep below fills
            # its `pairs_*` / `best_pair` into this same object.
            census = _empty_census()
            census.update({'boxed': cinfo.get('boxed', 0),
                           'movable': cinfo.get('movable', 0),
                           'censused': cinfo.get('censused', len(cands)),
                           'frozen': cinfo.get('frozen') or {},
                           'truncated': cinfo.get('truncated', 0),
                           'baseline': baseline,
                           'keepouts_freeing': keepouts_freeing,
                           'keepouts_joint': keepouts_joint})
            no_pose_census[ref] = census
            useful = sorted((n, b) for b, n in freed.items() if n > baseline)
            if not evict_depth:
                still.append(ref)
                if useful:
                    no_pose_verdict[ref] = 'blocker_available'
                    notes.append(
                        f"{ref}: no legal pose, and lifting {useful[-1][1]} "
                        f"would free {useful[-1][0]} -- not evicted "
                        f"(--evict-depth 0)")
                else:
                    # Depth 0 printed NOTHING here: no `useful` blocker, no
                    # note, and an empty dict in the JSON that could mean
                    # three different things. Every unseated part now leaves
                    # a sentence and a verdict, at every depth.
                    no_pose_verdict[ref] = _verdict_for(cands, census)
                    notes.append(_no_pose_note(
                        ref, no_pose_verdict[ref], census, evict_depth))
                continue
            # Depth 2 (#699): when NO single lift frees a pose, ask the same
            # question of pairs. The pair sweep CANNOT be pruned by the
            # single-lift counts -- in the case it exists for every one of
            # them is zero -- so it is ordered geometrically instead.
            chosen: List[str] = [useful[-1][1]] if useful else []
            chosen_freed = useful[-1][0] if useful else 0
            if not useful and evict_depth >= 2 and len(cands) >= 2:
                pairs = sorted(
                    itertools.combinations(range(len(cands)), 2),
                    key=lambda ij: (ij[0] + ij[1], ij[1]))
                pairs = [(cands[i], cands[j]) for i, j in pairs]
                census['pairs_total'] = len(pairs)
                census['pairs_truncated'] = max(0, len(pairs)
                                                - EVICT_MAX_PAIRS)
                pairs = pairs[:EVICT_MAX_PAIRS]
                census['pairs_censused'] = len(pairs)
                freed2 = {pr: count_legal_poses(state, ref, tx, ty,
                                                base_excl | set(pr), **zkw)
                          for pr in pairs}
                # Ties go to the NEAREST pair, not the alphabetically last
                # one: `count_legal_poses` saturates at CENSUS_CAP, so on a
                # roomy board several pairs come back with the identical
                # count and a plain `sorted(...)[-1]` would pick by ref name
                # -- throwing away the (i + j) ordering this sweep just went
                # to the trouble of building. `rank` is the enumeration
                # index, so the key is (count desc, rank asc).
                rank = {pr: i for i, pr in enumerate(pairs)}
                useful2 = sorted(((n, -rank[pr], pr)
                                  for pr, n in freed2.items()
                                  if n > baseline))
                if useful2:
                    chosen = list(useful2[-1][2])
                    chosen_freed = useful2[-1][0]
                    # The winning pair, as a RECORD. A tuple key is not
                    # JSON, and stringifying it ("S1+S2") invents a
                    # separator that a real reference may contain.
                    census['best_pair'] = {'blockers': list(chosen),
                                           'freed': chosen_freed}
            if not chosen:
                still.append(ref)
                no_pose_verdict[ref] = _verdict_for(cands, census)
                notes.append(_no_pose_note(
                    ref, no_pose_verdict[ref], census, evict_depth))
                continue
            zinfo = []
            for b in chosen:
                bz = ref_zone.get(b)
                zinfo.append(
                    (bz.rect if bz is not None else None,
                     intent.zone_tolerance(bz) if bz is not None else 0.5))
            rec = _evict_trade(state, ref, chosen, tx, ty, constraint, tol,
                               zinfo, placed, unplaced)
            rec.update({'poses_freed': chosen_freed, 'poses_before': baseline,
                        'depth': len(chosen)})
            evictions.append(rec)
            names = ', '.join(chosen)
            no_pose_verdict[ref] = ('seated_after_eviction' if rec['accepted']
                                    else 'trade_reverted')
            if rec['accepted']:
                placed.add(ref)
                unplaced.discard(ref)
                extra = ''
                if rec.get('moved'):
                    extra += ('; moved ' + ', '.join(
                        f"{b} {d:g}mm" for b, d in
                        sorted(rec['moved'].items())))
                if rec.get('rotated'):
                    extra += ('; ROTATED ' + ', '.join(
                        f"{b} {a:g}->{c:g}" for b, (a, c) in
                        sorted(rec['rotated'].items())))
                if rec.get('relaxed'):
                    extra += ('; the legality re-check ran at a reduced '
                              'clearance (a seat was found below the board '
                              'floor)')
                notes.append(
                    f"{ref}: seated after evicting {names} (poses at its "
                    f"target: {baseline} before, {chosen_freed} with "
                    f"{' + '.join(chosen)} lifted); violations "
                    f"{rec['violations_before']} -> "
                    f"{rec['violations_after']}, hpwl {rec['hpwl_before']} "
                    f"-> {rec['hpwl_after']}" + extra)
            else:
                still.append(ref)
                notes.append(f"{ref}: evicting {names} REVERTED -- "
                             f"{rec['reason']}")
        unseated = still

    # ---- 3b. anchor rounds (run-4 C): gated re-seat passes ------------------
    # Round 1 seeded anchors against anchors only; now that the smalls
    # exist, an anchor's partner centroid is truer, and a small seeded
    # around a provisional anchor pose may sit better re-derived. Each
    # round re-seats anchors (extent desc) then smalls at their partner
    # centroids over the FULL placement, and is kept only if the
    # reconstruct gate tuple does not worsen -- otherwise the whole round
    # reverts. Stops early when a round moves nothing.
    if anchors_first and anchor_rounds > 1 and placed:
        from placement.reconstruct import measure, part_extent_mm
        for rnd in range(2, max(2, anchor_rounds) + 1):
            baseline = measure(state)
            snapshot = {r: (state.parts[r].x, state.parts[r].y,
                            state.parts[r].rot) for r in placed}
            moved_n = 0
            order2 = sorted(placed,
                            key=lambda r: -part_extent_mm(state, r))
            for ref in order2:
                if state.parts[ref].locked:
                    continue
                target = _partner_centroid(state, ref, placed - {ref})
                if target is None:
                    continue
                ox, oy = state.parts[ref].x, state.parts[ref].y
                if _try_place(state, ref, target[0], target[1],
                              set()) is not None:
                    if math.hypot(state.parts[ref].x - ox,
                                  state.parts[ref].y - oy) > 1e-6:
                        moved_n += 1
            after = measure(state)
            if after <= baseline:
                notes.append(f"anchor round {rnd}: {moved_n} part(s) "
                             f"re-seated; gate {list(baseline)} -> "
                             f"{list(after)}")
                if moved_n == 0:
                    break
            else:
                for r, (x, y, rot) in snapshot.items():
                    state.apply_move(r, x, y, rot)
                notes.append(f"anchor round {rnd} REVERTED: gate worsened "
                             f"{list(baseline)} -> {list(after)}")
                break

    placements = [{'reference': ref,
                   'new_x': state.parts[ref].x, 'new_y': state.parts[ref].y,
                   'new_rotation': state.parts[ref].rot}
                  for ref in sorted(placed)]
    # Deduped: a zone member that also fails stage 3 is appended twice, and
    # `unseated: 2` for one part is a miscount every consumer inherits --
    # place_seed's summary, its exit code, and any gate reading the number.
    return {'placements': placements, 'lock_refs': lock_refs,
            'unseated': sorted(set(unseated)), 'notes': notes,
            # #629: a no-pose verdict that NAMES its blockers, with the count
            # each one frees. Present at every evict_depth. An empty dict for
            # a ref means the census ran and found no movable neighbour; a
            # ref absent from the dict had no recorded target.
            'no_pose_blockers': no_pose_blockers,
            # One record per attempted trade, accepted or reverted -- see
            # `_evict_trade`. `blockers` holds one ref at depth 1 and two at
            # depth 2.
            'evictions': evictions,
            # #699: WHY, not just WHO. `no_pose_blockers[ref] == {}` says
            # both "nothing is near it" and "everything near it is locked",
            # and those have opposite answers for the reader. One of
            # NO_POSE_VERDICTS per ref the rung reached, with the counts it
            # reached them by -- including the neighbours and pairs it did
            # NOT census, so a cap can never read as a complete sweep.
            'no_pose_verdict': no_pose_verdict,
            'no_pose_census': no_pose_census}


def stamp_locked(board_file: str, refs: Sequence[str]) -> int:
    """Insert `(locked yes)` into the named footprints, in place.

    Inserted immediately after the footprint's opening token, which is before
    the first pad -- the position placement/parser.extract_locked_refs (and
    KiCad itself) reads it from. The grade's must_lock rule demands the lock
    IN THE FILE, so writing the intent's locks here is what makes the emitted
    seed grade clean rather than merely hoped-correct."""
    from kicad_parser import find_matching_paren
    with open(board_file, 'r', encoding='utf-8') as f:
        content = f.read()
    want = set(refs)
    count = 0
    starts = [m.start() for m in re.finditer(r'\(footprint\s+"', content)]
    for start in reversed(starts):
        end = find_matching_paren(content, start)
        fp_text = content[start:end]
        m = re.search(r'\(property\s+"Reference"\s+"([^"]+)"', fp_text)
        if not m or m.group(1) not in want:
            continue
        if re.search(r'\(locked\s+yes\)', fp_text[:fp_text.find('(pad')
                                                  if '(pad' in fp_text else len(fp_text)]):
            continue
        open_m = re.match(r'\(footprint\s+"[^"]*"', fp_text)
        if not open_m:
            continue
        at = open_m.end()
        content = (content[:start + at] + '\n\t\t(locked yes)'
                   + content[start + at:])
        count += 1
    with open(board_file, 'w', encoding='utf-8') as f:
        f.write(content)
    return count


#: What a containment charges. Flat, not area-scaled: a 0402 wholly
#: inside a TSSOP measures 0.5mm2 and an area charge would floor it to
#: the 1.0mm budget, while a large part half-swallowed would outrank
#: it. Containment is a yes/no fact about whether a part can be built.
#: 2.0 buys the full cap ladder (budget = max(1.0, 8.0 * charge)).
CONTAINMENT_CHARGE_MM = 2.0

REPAIR_CAPS_MM = (0.5, 1.0, 2.0, 5.0)
# A repair move must be PROPORTIONATE to the violation it clears. The cap
# ladder escalates 0.5 -> 5.0mm hunting any legal seat, and on a board damaged
# by ~1.2mm it relocated parts 4.3-5.8mm: those few parts carried the whole of
# that run's negative recovery (excluding three of them took it from -0.296 to
# -0.058). A seat further than this multiple of the charged violation is not a
# repair, it is a different placement -- so it is refused and reported, and the
# floor keeps a sub-millimetre violation fixable by a sensible move.
DISPROPORTION_RATIO = 8.0
DISPROPORTION_FLOOR_MM = 1.0


def repair_placement(pcb_data, pcb_file: str, intent, *,
                     lock_globs: Optional[Sequence[str]] = None,
                     group_sources: Sequence[str] = (),
                     clearance: float = 0.25,
                     board_edge_clearance: float = 0.55,
                     grid_step: float = 0.1,
                     caps: Sequence[float] = REPAIR_CAPS_MM) -> Dict:
    """Violation-driven minimal-move repair of a PLACED board (#place_seed
    --repair). Everything clean freezes; only violators move, worst first,
    each seated by the seeder's own search targeted at its CURRENT pose with
    an escalating displacement cap. The opposite contract of --force (which
    re-derives everything).

    Violators: intent grade errors with a ref (zone/edge/decap...), pad/hole
    legality conflicts (the movable member of each pair), and parts off the
    board outline -- except refs the intent declares as edge connectors,
    whose overhang is by design.

    A must_lock ref is seeder-owned: its file lock is lifted in-memory for
    seating (the stamp in the file survives the positional rewrite, so no
    re-stamping is needed). A file-locked ref OUTSIDE must_lock is not this
    tool's to move: reported in `unrepairable`.

    NOTE this sweep has no internal bound -- its cost is violators x caps x 36
    ring sweeps x O(parts) per candidate, and on a 217-part board it ran 46
    minutes (run 9). Scope it with the violator set, not with a clock.
    """
    import pose_score
    from placement import floorplan, legality as _leg

    # A caller's --lock globs, on top of the file's own (locked yes) stamps.
    # These two entry points RE-PARSE the board and build their own state, so
    # a lock resolved by the CLI never reached them -- `--lock` was honoured
    # by five reconstruct stages and silently ignored by the two that move the
    # most parts. Feeding it here means the existing `.locked` checks below
    # (the unrepairable filter, and reseat's refusal list) pick it up for free.
    _extra_locked = {r for pat in (lock_globs or [])
                     for r in fnmatch.filter(sorted(pcb_data.footprints), pat)}
    state = pose_score.make_state(
        pcb_data, pcb_file, clearance=clearance,
        board_edge_clearance=board_edge_clearance, grid_step=grid_step,
        extra_locked_refs=_extra_locked or None,
        # #701: the declared keep-outs reach the SEAT PREDICATE (see
        # `seed_from_intent`). `intent` is optional on this path, so the
        # inert default is what a caller without one gets.
        keepouts=intent.keepouts if intent else ())
    refs_all = sorted(pcb_data.footprints)
    notes: List[str] = []
    must_lock = {r for pat in intent.must_lock
                 for r in fnmatch.filter(refs_all, pat)} if intent else set()
    # {ref: declared band max mm}. The off-board census below charges only the
    # EXCESS past the band, not nothing at all -- see the note there.
    edge_band: Dict[str, float] = {}
    if intent:
        for _c in intent.edge_claims():   # seat claims only; see edge_claims
            edge_band[_c['ref']] = float(
                (_c.get('overhang_mm') or {}).get('max') or 0.0)

    blocks, _probs = floorplan.resolve_blocks(intent, pcb_data, group_sources) \
        if intent else ({}, [])
    ref_zone: Dict[str, object] = {}
    if intent:
        for z in intent.blocks:
            if z.rect is None:
                continue
            for r in blocks.get(z.name, ()):
                ref_zone.setdefault(r, z)

    # ---- violator census ---------------------------------------------------
    weight: Dict[str, float] = {}
    # The OTHER member of each conflicting pair, kept as a fallback rather than
    # charged. Run-7 shipped a pair whose preferred mover had no legal seat
    # anywhere while its partner had one, and the partner was never tried: the
    # pair was reported unrepairable. Charging both instead is churn -- measured
    # on a corpus board with one genuine 0.04mm pair, it moved a second part
    # 0.50mm for no change in the graded result. So: try the partner only when
    # the preferred mover FAILS.
    partner_of: Dict[str, List[str]] = {}

    def _charge(ref, amt):
        if ref in state.parts:
            weight[ref] = weight.get(ref, 0.0) + amt

    # Refs known to be misplaced on structural grounds, not inferred from size.
    try:
        from placement import reconstruct as _recon
        witnesses = set(_recon.damage_witnesses(state))
    except Exception:                                       # noqa: BLE001
        witnesses = set()

    def _marker_or_container(r):
        """`reconstruct._body_exempt_refs`, resolved lazily and cached there."""
        try:
            from placement import reconstruct as _recon
            return r in _recon._body_exempt_refs(state)
        except Exception:                                       # noqa: BLE001
            return r in (getattr(state, 'container_refs', ()) or ())

    def _mover_key(r):
        """Which member of a conflicting pair should move.

        Pin count is a proxy for "the small part is the cheap one to move",
        and on a DAMAGED board it is the wrong question: it will happily move
        a small connector that is exactly where it belongs out from under a
        large part that is not. A ref carrying a structural witness -- a pad
        centre off the outline, which a shipped board cannot have -- is KNOWN
        to be misplaced, so it sorts first whatever its size.

        On a board with no witnesses this changes nothing, and that is every
        healthy board in the corpus: measured zero witnesses on all 33.
        """
        return (0 if r in witnesses else 1, state.parts[r].pin_count, r)

    def _keepclear_mover_key(r):
        """`_mover_key` for a pair a KEEP-CLEAR raised (#697), where a MARKER
        (fiducial / mount hole / test point) or a board-sized CONTAINER sorts
        LAST instead of first.

        Such a pair is characteristically a 1-pad fiducial against a many-pad
        connector, so pin count points the repair straight at the one part
        whose position is a mechanical fact. This is the set, and the argument,
        `reconstruct._body_exempt_refs` already makes for the body channel --
        "a displaced fiducial could never come home under a connector".

        Deliberately NOT the blanket ordering: as a term above pin count it is
        not inert. Measured on an undamaged orangecrab_ext_pll it flipped two
        PRE-EXISTING pairs (C67<->TP30, TP28<->U8) from moving a 1-pad test
        point to moving a 2-pad cap and a 14-pin IC -- pairs this issue did not
        surface and whose ordering it has no business changing.

        The witness term still outranks it, so a marker KNOWN to be misplaced
        keeps moving first, and a marker that is the only unlocked member of a
        pair still moves: this orders candidates, it does not veto one.
        """
        return (0 if r in witnesses else 1,
                1 if _marker_or_container(r) else 0,
                state.parts[r].pin_count, r)

    graded = None
    if intent is not None:
        graded = floorplan.grade(intent, pcb_data, pcb_file,
                                 group_sources=group_sources,
                                 clearance=clearance,
                                 board_edge_clearance=board_edge_clearance)
        for v in graded.errors:
            if v.ref:
                _charge(v.ref, float((v.measured or {}).get('outside_mm', 1.0)
                                     or 1.0))

    # worst_n=0: the FULL pair census (run-4 F5). The default cap of 10
    # bounded one repair pass at 10 pair-movers on a 20-pair board -- the
    # summary said 20 conflicts while only 10 got charged.
    pads = _leg.grade_pad_legality(pcb_data, clearance, worst_n=0,
                                   pcb_file=pcb_file)
    print(f"  Repair census: {pads['pad_conflicts']} conflict pair(s), "
          f"all listed")
    # #697: a pair can be graded ABOVE `clearance` (a pad keep-clear override, a
    # net class, a .kicad_dru rule). Say so, or the census reports a conflict at
    # a gap the announced clearance says is fine.
    _rq = _leg.format_required_clause(pads)
    if _rq:
        print(f"    above the {clearance}mm floor: {_rq}")
    for _n in (pads.get('clearance_notes') or ()):
        notes.append(f"pad clearance: {_n}")
    # Pairs a pad keep-clear / net class / dru rule raised above `clearance`.
    # Only these get the marker-last mover rule; see _keepclear_mover_key.
    _raised = {tuple(sorted((r[0], r[1])))
               for r in (pads.get('required') or ())}
    for (ra, rb, mm) in pads['worst']:
        free = [r for r in (ra, rb)
                if r in state.parts and not state.parts[r].locked]
        if not free:
            notes.append(f"pad conflict {ra}<->{rb} ({mm}mm): both file-locked"
                         f" -- not repairable here")
            continue
        # Charge the PREFERRED mover fully and its partner partially, instead
        # of charging one and forgetting the other. Run-7 shipped a pair whose
        # chosen mover had no legal seat anywhere while the other member had
        # one available -- the partner was never tried, and the pair was
        # reported unrepairable. Both are candidates now; the weights keep the
        # preferred one first in the worst-first order.
        ordered = sorted(free, key=(_keepclear_mover_key
                                    if tuple(sorted((ra, rb))) in _raised
                                    else _mover_key))
        _charge(ordered[0], mm)
        for partner in ordered[1:]:
            partner_of.setdefault(ordered[0], []).append(partner)

    # Run-6: the ASSEMBLY census. Blocking body pairs (any-net cross-
    # footprint pad intersections -- the shipped C14-on-R14 class that the
    # different-net-only census above skips by design) charge the same
    # mover rule, so the repair machinery can actually seat the squatter.
    body = _leg.grade_body_overlap(pcb_data, clearance, pcb_file=pcb_file)
    if body['blocking']:
        print(f"  Assembly census: {body['blocking']} blocking body "
              f"pair(s), all listed")
    for bp in body['blocking_pairs']:
        free = [r for r in (bp.a, bp.b)
                if r in state.parts and not state.parts[r].locked]
        if not free:
            notes.append(f"body stack {bp.a}<->{bp.b} ({bp.area_mm2}mm2): "
                         f"both file-locked -- not repairable here")
            continue
        ordered = sorted(free, key=_mover_key)
        # mm2 -> a strong mm-equivalent charge: a stack is never cosmetic
        _charge(ordered[0], max(1.0, bp.area_mm2))
        for partner in ordered[1:]:
            partner_of.setdefault(ordered[0], []).append(partner)

    # CONTAINMENT census (run-22). The body census above reads
    # `blocking_pairs`, which is pad_intersection ONLY -- so a part sitting
    # WHOLLY INSIDE another part's .Fab body produces no pad-intersection area,
    # is never charged, and legalize leaves it there. Prevention shipped
    # (reconstruct._pair_conflicts refuses such a pose, candidate_valid refuses
    # such a seat); this is the repair half, which did not.
    #
    # EXEMPTION: the engine's set, not the pair's `waiver` field.
    # `containment_pairs` is deliberately unfiltered by `waived` -- that is the
    # point of that list -- so iterating it raw would try to move orangecrab's
    # FID2 out from under J5 (frac 1.000, by design). And filtering by
    # `p.waived` instead would exempt `edge_class`, silently re-admitting the
    # very defect the channel exists to catch (run 22's D4 wholly inside SW2,
    # a declared edge_actuator).
    _cont_exempt = set()
    try:
        from placement import reconstruct as _recon
        _cont_exempt = _recon._body_exempt_refs(state)
    except Exception:
        _cont_exempt = set(getattr(state, 'container_refs', ()) or ())
    _cont = [q for q in body.get('containment_pairs', ())
             if q.a not in _cont_exempt and q.b not in _cont_exempt]
    if _cont:
        print(f"  Containment census: {len(_cont)} part(s) inside another "
              f"part's body, all listed")
    for q in _cont:
        free = [r for r in (q.a, q.b)
                if r in state.parts and not state.parts[r].locked]
        if not free:
            notes.append(f"containment {q.a}<->{q.b} "
                         f"({q.contained_frac:.0%} of the smaller body): "
                         f"both file-locked -- not repairable here")
            continue
        ordered = sorted(free, key=_mover_key)
        # CHARGE: the same mm-equivalent scale the body stack above uses, so a
        # containment buys at least the full cap ladder. Deliberately NOT
        # area_mm2 -- a 0402 wholly inside a TSSOP measures 0.5mm2 and would
        # be floored to a 1.0mm budget, while a large part half-swallowed
        # would outrank it. Containment is a yes/no fact about whether a part
        # can be built, so it charges a flat strong value rather than a size.
        _charge(ordered[0], CONTAINMENT_CHARGE_MM)
        for partner in ordered[1:]:
            partner_of.setdefault(ordered[0], []).append(partner)

    # Off-board census on PAD/HOLE extents at ZERO margin -- copper or drill
    # off the outline is a fab defect; a COURTYARD poking past the edge is
    # cosmetic and common on legitimate boards (tigard's own corner mounting
    # holes overhang by courtyard; the human original grades oob 7). The
    # margined courtyard test flagged and "repaired" exactly those.
    zero_gate = _leg.BoardOutlineGate(pcb_data.board_info, 0.0)
    part_pads = _leg.build_part_pads(pcb_data.footprints, clearance)
    #
    # A DECLARED edge ref is exempt only WHILE ITS OVERHANG IS INSIDE ITS
    # DECLARED BAND; past that the excess is charged like anyone else's. The
    # exemption used to be unbounded (`if ref in edge_refs: continue`), which
    # is safe only while the bands are a spec. Once `check_floorplan
    # --emit-intent` is run on a DAMAGED board they are not: run 10 emitted
    # bands equal to each part's damage displacement (up to 160 mm on an 81 mm
    # board) and this census then skipped all ELEVEN off-board parts -- the
    # repair reported 5 violators, none of them the ones whose pads were in the
    # air, and all 13 unrouted nets had a pad on one of them. floorplan's
    # EDGE_BAND_SANITY_MM stops such a band being emitted; this stops one that
    # already exists (an older intent, a hand-written one) from blinding the
    # census.
    for ref, part in state.parts.items():
        pp = part_pads.get(ref)
        if pp is None:
            continue
        fp = pcb_data.footprints[ref]
        ext = pp.extent(fp.x, fp.y, fp.rotation or 0.0)
        if ext is None:
            continue
        amt = zero_gate.rect_outside_amount(ext)
        band = edge_band.get(ref)
        if band is not None:
            excess = amt - band
            if excess > 1e-6:
                notes.append(
                    f"{ref}: overhangs {amt:.3f}mm against a declared band of "
                    f"{band:.3f}mm -- the {excess:.3f}mm EXCESS is charged "
                    f"(a declared band exempts the overhang it declares, not "
                    f"any overhang)")
                _charge(ref, excess)
            continue
        if amt > 1e-6:
            _charge(ref, amt)

    # A file-locked part is never this tool's to move (run-7 finding).
    #
    # `must_lock` used to be an exemption here: a ref inside it was treated as
    # "seeder-owned", its lock lifted in memory, and it was repaired like any
    # other violator. That is safe while must_lock is a hand-written
    # REQUIREMENT ("these refs must end up locked"), and catastrophic once
    # check_floorplan --emit-intent started filling must_lock with the board's
    # OWN file-locked set: auto-intent + --repair then resolved to "unlock
    # exactly the parts the user locked, and move them". Measured on two run-7
    # boards, one of which walked a locked part 31mm.
    #
    # Seeder-ownership is a SEEDING concept -- when place_seed builds a
    # placement from an intent it owns every pose it creates, including the
    # locks it stamps on its own output. Repair edits a board somebody else
    # placed, so the file's locks outrank the intent. must_lock keeps its
    # grading meaning (floorplan's rule_must_lock still demands the stamp).
    unrepairable = [r for r in sorted(weight) if state.parts[r].locked]
    for r in unrepairable:
        why = ("in must_lock, which grades the lock rather than licensing a "
               "move" if r in must_lock else "not in must_lock")
        notes.append(f"{r} violates but is (locked yes) in the file "
                     f"({why}) -- not this tool's to move")
    violators = [r for r in sorted(weight, key=lambda r: -weight[r])
                 if r not in unrepairable]

    # ---- seat violators, worst first, escalating cap -----------------------
    # edge_claims(), and this one is a REGRESSION FIX, not tidiness. The
    # dispatch below short-circuits every entry in this map that carries no
    # `edge` into a refusal, so reading the raw key took the run-23
    # connector_affinity declarations -- which never carry an edge, by
    # design -- straight out of the ordinary `_try_place` loop. Measured on
    # tests/fixtures/run23/tigard_damaged.kicad_pcb with its own
    # --declare-classes intent: J5, J6 and J7 were refused with "declared
    # edge part misplaced, but no edge is declared" (which also mislabels
    # them), where upstream main tries them like any other violator and
    # reports the honest "no legal pose within any cap". A declaration must
    # not remove a part from repair.
    edge_entry = ({c['ref']: c for c in intent.edge_claims()}
                  if intent else {})
    repaired: List[str] = []
    failed: List[str] = []
    zero_move: List[str] = []   # run-7 A2: honesty re-grade candidates
    moves: List[Dict] = []
    for ref in violators:
        part = state.parts[ref]
        was_locked = part.locked      # always False here: locked refs are
                                      # already in `unrepairable` (see above)

        # Run-4 F2/B-6: a DECLARED edge part charged by the proximity rule
        # cannot be seated by _try_place (its _ok demands full containment,
        # and an edge seat overhangs by design). Seat it on its declared
        # edge band instead -- or refuse honestly when no edge is declared
        # (an implausibly-posed receptacle: reconstruct derives the edge).
        ec = edge_entry.get(ref)
        if ec is not None:
            part.locked = was_locked
            if not ec.get('edge'):
                failed.append(ref)
                notes.append(
                    f"{ref}: declared edge part misplaced, but no edge is "
                    f"declared (implausible pose, none derivable here) -- "
                    f"place_reconstruct derives edge slots; repair will not "
                    f"guess one")
                continue
            zt = ref_zone.get(ref)
            tgt = None
            if zt is not None and zt.rect is not None:
                tgt = ((zt.rect[0] + zt.rect[2]) / 2.0,
                       (zt.rect[1] + zt.rect[3]) / 2.0)
            ok = _seat_edge(state, ref, ec, must_lock, notes, target=tgt)
            if ok:
                d = math.hypot(part.x - part.seed_x, part.y - part.seed_y)
                moves.append({'reference': ref, 'new_x': part.x,
                              'new_y': part.y, 'new_rotation': part.rot})
                repaired.append(ref)
                notes.append(f"{ref}: seated on the {ec['edge']} edge band "
                             f"({d:.2f}mm from its input pose)")
            else:
                failed.append(ref)
                notes.append(f"{ref}: no conflict-free seat found on the "
                             f"declared {ec['edge']} edge band")
            continue

        z = ref_zone.get(ref)
        rect = z.rect if z is not None else None
        tol = intent.zone_tolerance(z) if (intent and z is not None) else 0.5
        ox, oy, orot = part.x, part.y, part.rot
        placed_at = None
        for cap in caps:
            info: Dict = {}
            clr = _try_place(state, ref, ox, oy, set(), constraint=rect,
                             tol=tol, max_disp=cap, info=info)
            if clr is not None:
                placed_at = cap
                if info.get('anchor_zone'):
                    notes.append(f"{ref}: spec-coordinate zone -- seated by "
                                 f"anchor point")
                break
        if placed_at is None and z is not None:
            # current pose may be far from the zone: target the zone center
            zx = (z.rect[0] + z.rect[2]) / 2.0
            zy = (z.rect[1] + z.rect[3]) / 2.0
            clr = _try_place(state, ref, zx, zy, set(), constraint=rect,
                             tol=tol)
            if clr is not None:
                placed_at = 'zone'
        part.locked = was_locked
        if placed_at is None:
            # Before giving up on the pair, try its OTHER member -- the mover
            # rule picked this one, but "preferred" is not "the only one that
            # can move".
            seated_partner = None
            for partner in partner_of.get(ref, []):
                if partner not in state.parts or state.parts[partner].locked:
                    continue
                pp = state.parts[partner]
                pox, poy, porot = pp.x, pp.y, pp.rot
                for cap in caps:
                    if _try_place(state, partner, pox, poy, set(),
                                  max_disp=cap) is not None:
                        pd = math.hypot(pp.x - pox, pp.y - poy)
                        if pd > 1e-9:
                            seated_partner = (partner, pd)
                        break
                if seated_partner:
                    break
                state.apply_move(partner, pox, poy, porot)
            if seated_partner:
                pname, pd = seated_partner
                pp = state.parts[pname]
                moves.append({'reference': pname, 'new_x': pp.x,
                              'new_y': pp.y, 'new_rotation': pp.rot})
                repaired.append(pname)
                notes.append(
                    f"{ref}: no legal pose within any cap {tuple(caps)}mm -- "
                    f"seated its pair partner {pname} instead "
                    f"({pd:.2f}mm from its pose)")
                continue
            failed.append(ref)
            notes.append(f"{ref}: no legal pose within any cap "
                         f"{tuple(caps)}mm of its current pose"
                         + (f" (partner{'s' if len(partner_of.get(ref, [])) > 1 else ''} "
                            f"{', '.join(partner_of.get(ref, []))} tried too)"
                            if partner_of.get(ref) else ''))
            continue
        d = math.hypot(part.x - ox, part.y - oy)
        budget = max(DISPROPORTION_FLOOR_MM,
                     DISPROPORTION_RATIO * weight.get(ref, 0.0))
        if d > budget:
            state.apply_move(ref, ox, oy, orot)
            failed.append(ref)
            notes.append(
                f"{ref}: the only legal seat is {d:.2f}mm away, which is "
                f"disproportionate to the {weight.get(ref, 0.0):.2f}mm "
                f"violation it clears (budget {budget:.2f}mm) -- left in "
                f"place. A move this size is a different placement, not a "
                f"repair; reconstruct or re-arrange instead.")
            continue
        if d > 1e-9 or part.rot != orot:
            moves.append({'reference': ref, 'new_x': part.x, 'new_y': part.y,
                          'new_rotation': part.rot})
            repaired.append(ref)
            notes.append(f"{ref}: re-seated {d:.2f}mm from its pose "
                         f"(cap {placed_at})")
        else:
            # Zero-move "repair": _try_place accepted the CURRENT pose.
            # Legitimate when another mover already cleared the pair;
            # run-6 measured the OTHER case looping forever ("5 repaired,
            # 0 moved" every lap): the census charged a pad/body/grade
            # violation while _try_place's courtyard test passed at the
            # standing pose (metric mismatch). Classify honestly below.
            repaired.append(ref)
            zero_move.append(ref)

    # Run-7 A2: honesty re-grade. A violator counts as repaired only if the
    # charged violation classes actually IMPROVED; zero-move violators on a
    # board whose pad/body census did not move are UNRESOLVED, so the fix
    # loop can see it stalled instead of believing "repaired" forever.
    unresolved = []
    if zero_move and state.legality_ctx is not None:
        # Post-repair poses live on the STATE (pcb_data still holds the
        # file's input poses), so the re-grade uses the state's own pair
        # machinery in the gate currency.
        ctx = state.legality_ctx
        for ref in zero_move:
            still = False
            # CONTAINMENT is checked FIRST, and it has to be: PairShortfall
            # carries pad/hole/stack and no body term, so a part charged for
            # sitting inside another body, which then could not move, would
            # pass this loop and be reported `repaired`. That is the exact
            # metric mismatch this re-grade was written to stop, one channel
            # later.
            #
            # It FAILS LOUD. `state` is always a QuenchState (it comes from
            # pose_score.make_state), so the method always exists, and
            # fab_rect already swallows its own parse failure and returns None
            # = UNJUDGED. Anything still raising here is a real bug -- and
            # swallowing it would convert that bug into exactly the false
            # `repaired` this check exists to prevent. So an error means NOT
            # repaired, said out loud, rather than a silent pass.
            try:
                if state._body_contained_at(ref, None, None, None):
                    still = True
            except Exception as exc:
                still = True
                notes.append(f'{ref}: could not verify containment after the '
                             f'repair ({exc.__class__.__name__}: {exc}) -- '
                             f'NOT reported repaired')
            for other in sorted(state.parts):
                if still:
                    break
                if other == ref:
                    continue
                sf = ctx.pair_shortfall(ref, other)
                if sf.stack or sf.pad > 1e-6 or sf.hole > 1e-6:
                    still = True
                    break
            if still:
                unresolved.append(ref)
                repaired.remove(ref)
                notes.append(
                    f"{ref}: UNRESOLVED -- pose is courtyard-legal but the "
                    f"charged pad/body violation persists (metric mismatch; "
                    f"was reported 'repaired' before run-7 A2)")
    return {'moves': moves, 'repaired': repaired, 'unrepairable':
            unrepairable + failed, 'unresolved': unresolved,
            'violators': violators, 'notes': notes,
            'pad_report_before': {k: pads[k] for k in
                                  ('pad_conflicts', 'hole_conflicts',
                                   'oob_pad_count')},
            'grade_errors_before': len(graded.errors) if graded else None}


def eviction_licence_ok(before: Sequence[float],
                        after: Sequence[float]) -> bool:
    """May a re-seat that moved parts OUTSIDE its scope be accepted?

    `reseat_scope`'s own gate compares `oob` and the witness count, and that
    is sufficient while the pass only moves parts it was asked about. Once
    `--evict-depth` lets it trade out a part nobody named, it is not: the gate
    tuple is lexicographic and `oob` moves hugely in this pass's own favour,
    so a new stack or a pile of overlap sits below it and is never read.

    So: stacks and overlap must not RISE. Both terms are already in the tuple,
    which is why this costs nothing.

    It is the JOINT check. `prune_assignment` runs first and reverts per-part
    mis-moves the global gate cannot see -- measured, it catches an injected
    "blocker parked on the part just seated" before this is reached -- but its
    sweep restores ONE pose at a time, so a pair of moves that is individually
    neutral and jointly worse is exactly what it cannot see and this can.
    Tested directly (`tests/test_630_seeder_eviction.py`) rather than through
    a fixture, because a fixture that reaches it has to defeat prune first.
    """
    from placement import reconstruct as _recon
    _stk = _recon.GATE_TERMS.index('stacks')
    _ovl = _recon.GATE_TERMS.index('overlap')
    return (after[_stk] <= before[_stk]
            and after[_ovl] <= before[_ovl] + 1e-9)


# --------------------------------------------------------------------------
# #698: what an EXPLICIT re-seat may be accepted on
#
# The auto scope and an explicit scope differ in one structural way, and every
# decision below follows from it. On `auto:damage_witnesses` the pass's win IS
# `oob`, which sits at index 3 of the gate tuple -- ABOVE `hpwl` -- so the
# lexicographic compare already sees it, prune cannot revert a genuine
# homecoming, and `after[oob] < before[oob]` is a complete rule. On an explicit
# scope the win is a declared claim or the scope's own wirelength, which the
# tuple cannot see AT ALL. That is why the explicit branch needs a term-wise
# safety condition plus a SEPARATE trigger, and why the answer is not an eighth
# tuple term (`measure` has no intent to measure, and
# tests/test_run8_gate_conjuncts.py pins the arity at 7 with its own reason).
# --------------------------------------------------------------------------

#: The scope-relevant terms an explicit re-seat may be accepted ON, in the order
#: they are tried and reported. Exported so a test reads the vocabulary FROM the
#: engine rather than restating it -- `quench.INTENT_ENFORCED_RULES`'s device.
#: Severity order, weakest last: `scope_hpwl` is a netlist PROXY, and the whole
#: point of a declared claim is that it overrules the proxy.
RESEAT_BASES = ('locked_contacts', 'pad_pairs', 'hole', 'oob', 'intent',
                'stacks', 'overlap', 'scope_hpwl')

#: Each basis in its OWN currency. There is no exchange rate between them and
#: `--reseat-min-gain` does not invent one -- see `reseat_accept`.
RESEAT_BASIS_UNITS = {'locked_contacts': 'count', 'pad_pairs': 'count',
                      'hole': 'mm', 'oob': 'mm', 'intent': 'count',
                      'stacks': 'count', 'overlap': 'mm2',
                      'scope_hpwl': 'mm'}

#: The ONE gate term an explicit re-seat is licensed to worsen, and the reason
#: it is not also a basis: a seat made for a declared reason is hpwl-worse BY
#: CONSTRUCTION, because hpwl is the netlist proxy the declaration overrules.
#: `placement/README.md` says it in the same words for the evicted-part
#: exemption -- "an edge-class seat is hpwl-worse BY DESIGN". Measured on
#: `tests/test_698_reseat_acceptance.py`'s keep-out fixture, swept over 20
#: seeds: escaping keep-out `hot` costs hpwl on 20 of 20 (6.82 to 12.39 mm --
#: a different value per seed, since the seat search is seeded), so any rule
#: that forbids hpwl rising refuses exactly the case this exists for. `arm_B`
#: computes that counterfactual rather than asserting it.
RESEAT_LICENSED_TERM = 'hpwl'

#: The last digit `reconstruct.measure` keeps for the continuous LEGALITY terms
#: -- `hole`, `oob` and `overlap` are rounded to 4 decimals (`hpwl` to 3). A
#: gain must EXCEED this to count: a change in the last representable digit of
#: a 4dp aggregate is not evidence of anything, and these bases are otherwise
#: ungated, so nothing else would stop rounding from carrying a pass.
MEASURE_QUANTUM = 1e-4


def reseat_safety_ok(before: Sequence[float],
                     after: Sequence[float]) -> Tuple[bool, List[str]]:
    """(ok, the GATE_TERMS that ROSE). TERM-WISE, deliberately not lexicographic.

    A lexicographic `after <= before` reads `hpwl` at index 5 and so refuses
    every declared-claim escape -- the trap this whole change exists to avoid.
    Term-wise with one licensed term says the intended thing instead: the pass
    may pay wirelength for a claim, and may pay NOTHING else.

    `oob` is hard but is NOT required to improve, and that asymmetry is issue
    #698 in one line: requiring it to improve is what makes the pass a no-op
    for any part already on the board, while requiring it not to worsen keeps
    the defect CLAUDE.md ranks first -- pad copper off the outline -- unbuyable.
    """
    from placement import reconstruct as _recon
    idx = _recon.GATE_TERMS.index
    rose: List[str] = []
    for n in ('locked_contacts', 'pad_pairs', 'stacks'):
        if after[idx(n)] > before[idx(n)]:          # integer counts, exact
            rose.append(n)
    for n in ('hole', 'oob', 'overlap'):
        # 1e-9, the SAME epsilon `eviction_licence_ok` uses. Two licences on
        # one pass must not carry two tolerances, and `measure` rounds these
        # to 4 decimals anyway.
        if after[idx(n)] > before[idx(n)] + 1e-9:
            rose.append(n)
    return (not rose), rose


def scope_hpwl(state, refs) -> float:
    """HPWL over the NETS a scope ref has a pad on (mm).

    Narrower than the tuple's board-wide `hpwl`, which is the point: a net that
    touches no scope ref cannot move in this pass, so including it only adds a
    constant that hides the signal.

    It is NOT "the scope's own contribution", and the difference matters at
    `--evict-depth >= 1`. The sum runs over every pad on those nets, so an
    evicted neighbour sharing a net with the scope can supply part or all of
    the gain while the ref the operator named got worse. Measured on the
    `plain_board` fixture with scope `{U1}`, moving only CON2: `scope_hpwl`
    62.0 -> 8.0. What stops that being a licence is elsewhere -- the intent
    probe covers the whole board (see `reseat_scope`), and
    `eviction_licence_ok` refuses any eviction that raised stacks or overlap --
    not this function, which is only a wirelength number over a net set.
    """
    nets = set()
    for r in refs:
        p = state.parts.get(r)
        if p is None:
            continue
        for _gx, _gy, n in p.pad_globals():
            if n > 0:
                nets.add(n)
    return round(state.hpwl(nets), 3)


def reseat_bases(gate: Sequence[float], intent_count: int,
                 hpwl_scope: float) -> Dict[str, float]:
    """{basis -> value} for all of `RESEAT_BASES` at one measurement point."""
    from placement import reconstruct as _recon
    idx = _recon.GATE_TERMS.index
    out = {n: gate[idx(n)] for n in RESEAT_BASES
           if n in _recon.GATE_TERMS}
    out['intent'] = intent_count
    out['scope_hpwl'] = hpwl_scope
    return out


def basis_skeleton(scope_source: str, *, policy: str,
                   witnesses_before=(), witnesses_after=(),
                   hpwl_before: float = 0.0, hpwl_after: float = 0.0,
                   min_gain: float = 0.0) -> Dict:
    """The `accept_basis` key set, in ONE place.

    Every return path of `reseat_scope` -- the seated one, the early-out, and
    both policies -- carries the same keys because they all come from here. The
    early-out used to hand back 6 of the 15 under a comment promising parity:
    exactly the schema split that early-out's own census keys exist to prevent,
    one field down. (It did NOT crash anything -- `reseat_refusal_note` reads
    every field with `.get`, and no engine path passes an empty-scope basis to
    it. The defect is the broken promise, which a consumer outside this repo is
    entitled to rely on, not a live traceback.)
    """
    return {
        'scope_source': scope_source,
        'policy': policy,
        # Present on EVERY path, so the seated path cannot carry a key the
        # early-out lacks. `reseat_scope` overwrites it with False when the
        # licence refuses; None means the question never arose.
        'eviction_licence': None,
        'witness_ok': True,
        'witnesses_before': len(witnesses_before),
        'witnesses_after': len(witnesses_after),
        'hpwl_before': hpwl_before, 'hpwl_after': hpwl_after,
        'hpwl_delta': round(hpwl_after - hpwl_before, 3),
        'min_gain': float(min_gain), 'min_gain_units': 'mm',
        'min_gain_applies_to': 'scope_hpwl',
        'fired': None, 'terms': [],
        # Measured-and-clean must not look like never-measured: a path that
        # does not run these leaves them None rather than reporting a clean
        # pass over nothing.
        'safety': None, 'intent_licence': None,
    }


def reseat_accept(before: Sequence[float], after: Sequence[float], *,
                  scope_source: str,
                  witnesses_before, witnesses_after,
                  bases_before: Optional[Dict[str, float]] = None,
                  bases_after: Optional[Dict[str, float]] = None,
                  intent_risen: Sequence = (),
                  min_gain: float = 0.0) -> Tuple[bool, Dict]:
    """THE re-seat acceptance rule. (accepted, accept_basis).

    Plain numbers, no state -- `eviction_licence_ok`'s shape and its reason:
    the whole policy becomes directly testable without a fixture that has to
    defeat `prune_assignment` first.

    `--reseat-min-gain` is MILLIMETRES and gates the `scope_hpwl` basis ONLY.
    A single scalar compared against a count, a millimetre, an mm2 of intrusion
    and `keepout_hit`'s fabricated circle marker is the summing error wearing a
    threshold's clothes: it asserts an exchange rate between "half a millimetre
    of wire" and "half a keep-out violation". `_IntentTerm` refuses that rate
    inside one ref's vector, and the accept basis refuses it across bases.
    Count bases threshold at ONE WHOLE defect, which is the only figure their
    currency has; the remaining continuous bases are legality terms, where any
    strict improvement is real and a shuffle cannot manufacture one.

    `scope_hpwl` is where the sideways shuffle lives and is therefore both LAST
    and the only gated basis.
    """
    from placement import reconstruct as _recon
    _oob = _recon.GATE_TERMS.index('oob')
    _hp = _recon.GATE_TERMS.index(RESEAT_LICENSED_TERM)
    witness_ok = len(witnesses_after) <= len(witnesses_before)

    basis = basis_skeleton(
        scope_source, policy='', witnesses_before=witnesses_before,
        witnesses_after=witnesses_after, hpwl_before=before[_hp],
        hpwl_after=after[_hp], min_gain=min_gain)
    basis['witness_ok'] = witness_ok

    if scope_source != 'explicit':
        basis['policy'] = 'auto:oob-strict'
        accepted = (after[_oob] < before[_oob] and witness_ok)
        basis['terms'] = [{
            'term': 'oob', 'units': 'mm', 'before': before[_oob],
            'after': after[_oob],
            'gain': round(before[_oob] - after[_oob], 4),
            'min_gain_applies': False,
            'would_fire': after[_oob] < before[_oob],
            'first': after[_oob] < before[_oob]}]
        if accepted:
            basis['fired'] = 'oob'
        return accepted, basis

    basis['policy'] = 'explicit:one-term-strict'
    safe, rose = reseat_safety_ok(before, after)
    basis['safety'] = {'ok': safe, 'worsened': rose,
                       'licensed': RESEAT_LICENSED_TERM}
    risen = [tuple(r) for r in intent_risen]
    basis['intent_licence'] = {'ok': not risen, 'risen': risen}

    bb = bases_before or {}
    ba = bases_after or {}
    fired = None
    for name in RESEAT_BASES:
        b, a = bb.get(name), ba.get(name)
        if b is None or a is None:
            continue
        gain = b - a
        units = RESEAT_BASIS_UNITS[name]
        gated = (name == 'scope_hpwl')
        if units == 'count':
            ok = gain >= 1
        elif gated and float(min_gain) > MEASURE_QUANTUM:
            # `>=`, not `>`. The flag's help calls it "the smallest win that
            # COUNTS as a re-seat", and `scope_hpwl` is quantised to 3dp, so
            # exact equality with a round threshold is the common case rather
            # than a corner: `--reseat-min-gain 0.5` refusing a gain of 0.5
            # would be the flag not doing what it says.
            #
            # The guard is `> MEASURE_QUANTUM`, not a bare truthiness test, and
            # that is the whole reason this branch is written out. A threshold
            # BELOW the quantum must not reach it: with `elif gated and
            # min_gain` a `--reseat-min-gain 1e-12` sent a gain of EXACTLY ZERO
            # down this path and accepted it -- a looser gate from a stricter
            # flag, admitting the very sideways shuffle the basis exists to
            # refuse, and reachable (10 of the 16 corpus rows in
            # `tests/measure_698_min_gain.py` have a gain of exactly 0.000). A
            # negative `min_gain` did the same through the kwarg, which the CLI
            # validator cannot see. Falling through to the floor below makes
            # the rule MONOTONE in `min_gain`: a bigger threshold is never
            # looser, and no threshold is ever looser than no threshold.
            ok = gain >= float(min_gain) - 1e-9
        else:
            # `MEASURE_QUANTUM`, not an epsilon. These bases are ungated by
            # `min_gain`, so without a floor the old `gain > 1e-9` fired on a
            # change in the LAST DIGIT `measure` keeps -- an `overlap` gain of
            # 1e-4 mm2 was enough to accept a pass, and since `hpwl` is the
            # licensed term such a pass may be arbitrarily worse on wirelength.
            # Run 4 demoted `overlap` below `hpwl` in GATE_TERMS because
            # 0.73mm2 of kiss had vetoed a 44mm homecoming; 0.0001mm2 must not
            # buy one in the other direction.
            ok = gain > MEASURE_QUANTUM
        first = ok and fired is None
        if first:
            fired = name
        # `would_fire` is per-term and says only "this basis improved enough".
        # The DECISION is the top-level `fired`, which is None unless the
        # safety half also held -- so a refused pass still discloses which
        # basis would have carried it, rather than reporting a bare no.
        basis['terms'].append({
            'term': name, 'units': units,
            'before': b, 'after': a,
            'gain': round(gain, 4) if units != 'count' else gain,
            'min_gain_applies': gated,
            'would_fire': bool(ok), 'first': bool(first)})

    accepted = bool(witness_ok and safe and not risen and fired is not None)
    basis['fired'] = fired if accepted else None
    return accepted, basis


def reseat_refusal_note(n_scope: int, basis: Dict) -> str:
    """Why the pass was refused, in the terms it was actually judged on.

    The old note said "did not strictly improve the off-board amount" on EVERY
    path, which on an explicit scope names a term the operator never asked
    about and cannot move -- that sentence, quoted back with `(2.15 -> 2.15)`
    in it, is what issue #698 was filed with. A refusal that misnames its own
    reason sends the reader to fix the wrong thing.
    """
    head = f"REVERTED: re-seating {n_scope} part(s) was refused"
    # FIRST, on BOTH policies. `reseat_scope` sets this flag on either scope,
    # and the auto branch below returns -- so an evicted auto pass used to
    # print "the off-board amount strictly improves (9.65 -> 0.0) and the
    # witness count does not grow (2 -> 0)" as its REASON for refusing, with
    # both conjuncts visibly satisfied in the sentence stating them. That is
    # the defect this function exists to prevent, one branch over.
    if basis.get('eviction_licence') is False:
        return (f"{head}: the eviction licence -- moving parts outside the "
                f"scope raised the stack count or the overlap area. See the "
                f"note above for the figures.")
    if basis.get('policy') == 'auto:oob-strict':
        t = (basis.get('terms') or [{}])[0]
        return (f"{head}: on the AUTO scope the rule is that the off-board "
                f"amount strictly improves ({t.get('before')} -> "
                f"{t.get('after')}) and the witness count does not grow "
                f"({basis.get('witnesses_before')} -> "
                f"{basis.get('witnesses_after')}). Both conjuncts are "
                f"required: the gate tuple is lexicographic, so a large oob "
                f"win would hide a new stack or an hpwl blow-up below it, and "
                f"a sideways move that changes neither is not a re-seat.")
    if not basis.get('witness_ok', True):
        return (f"{head}: the off-outline part count GREW "
                f"({basis.get('witnesses_before')} -> "
                f"{basis.get('witnesses_after')}), which no basis may buy.")
    safety = basis.get('safety') or {}
    if not safety.get('ok', True):
        return (f"{head}: it worsened {', '.join(safety.get('worsened') or [])}"
                f". An explicit re-seat is licensed to pay "
                f"{safety.get('licensed')} for a claim -- a seat made for a "
                f"declared reason is hpwl-worse by construction -- and is "
                f"licensed to pay nothing else.")
    lic = basis.get('intent_licence') or {}
    if not lic.get('ok', True):
        why = "; ".join(f"{r} {rule} {name!r} {b:g} -> {a:g}"
                        for r, rule, name, b, a in (lic.get('risen') or []))
        return (f"{head}: a declared claim got WORSE ({why}). Measured "
                f"termwise and never summed, so a part cannot leave one "
                f"keep-out by entering another and report no change.")
    gains = ", ".join(
        f"{t['term']} {t['before']}->{t['after']} ({t['units']})"
        for t in (basis.get('terms') or []))
    mg = basis.get('min_gain') or 0.0
    tail = (f" `scope_hpwl` additionally had to beat --reseat-min-gain "
            f"{mg:g}mm." if mg else "")
    return (f"{head}: nothing improved. An explicit scope is accepted when no "
            f"hard term and no declared claim got worse AND at least one "
            f"scope-relevant term strictly improved; none did. Considered: "
            f"{gains}.{tail}")


def reseat_scope(pcb_data, pcb_file: str, intent, *,
                 lock_globs: Optional[Sequence[str]] = None,
                 refs: Optional[Sequence[str]] = None,
                 group_sources: Sequence[str] = (),
                 clearance: float = 0.25,
                 board_edge_clearance: float = 0.55,
                 grid_step: float = 0.1,
                 seed: int = 0,
                 evict_depth: int = 0,
                 min_gain: float = 0.0,
                 edge_bands: Optional[Dict[str, float]] = None) -> Dict:
    """LIFT a subset of parts and re-seat them FROM SCRATCH at their net
    centroids, holding every other part fixed as an obstacle.

    `evict_depth` (#699) is the ONE exception to "every other part fixed",
    and it is off by default. At depth >= 1 a scope ref with no legal pose
    may have a seated NON-SCOPE neighbour traded out from under it, under
    `_evict_trade`'s acceptance rule. Those refs are named in `evicted`, in a
    NOTE, and in `moves` -- they have to be in `moves`, because `moves` is
    the whole of what gets written, so an evicted part left out of it would
    be written at its OLD pose while the scope ref takes its pocket. At
    depth 0 the fixed-obstacle contract holds exactly and every code path
    below reduces to what it was.

    The contract that distinguishes this from `repair_placement`: **the part's
    current pose is never consulted.** Every other repair path in this stack --
    `--repair`'s cap ladder, `place_optimize`'s nudge, `place_portfolio`'s
    strategies, reconstruct's `{stay, +v, -v, pattern slot}` candidate sets --
    searches outward from where the part IS. That is the right question for a
    part that is nearly home and no question at all for one that is tens of
    millimetres out: a pose 30 mm from where a part belongs carries no
    information about where it belongs, and the cost of hunting from it grows
    with the cap while the chance of a hit does not. Measured on a 107-part
    board with 11 parts 7-32 mm off the outline: `--repair` spent 4 m 55 s and
    attempted none of them (its ladder tops out at 5 mm from the wrong centre);
    `place_reconstruct --max-move 40` ran over 8.5 min; this pass seated 11 of
    11 in 6.3 s.

    Not a recovery pass. It puts parts where the NETLIST wants them, not where
    they were, so `recovery` will not improve and `collateral_pad_rms` will
    grow. The number to judge it on is `witnesses_after` -- the count of parts
    whose pad centres are still off the outline -- because that is the one that
    predicts routability: on the board above, the same 11 refs carried a pad on
    every one of the 13 nets the router could not attempt, one for one.

    Scope: `refs` (fnmatch globs over the board's references), else AUTO =
    `reconstruct.damage_witnesses` -- refs with a pad CENTRE off the outline,
    which is the negation of a manufacturability invariant (you cannot solder
    to air) and is corpus-calibrated to ZERO on all 33 healthy boards. An empty
    scope returns `{'reseated': []}` and is a RESULT, not a failure.

    Every refusal is named in `notes`: a ref absent from the board, and a ref
    locked in the file or in the intent's `must_lock` (a locked pose is not
    this tool's to move -- the same rule `repair_placement` applies, for the
    run-7 reason recorded there).

    The scope's own `edge_connectors` declarations are DROPPED before seeding,
    and this is mandatory rather than hygiene: `seed_from_intent`'s stage 1
    places a declared edge connector with NO legality gate, at the middle of
    its band, and then walks it outward. Measured with the bands left in, on an
    intent auto-emitted from the damaged board, it threw TP4 to y = 30255 and
    R12 to x = -3971 -- thirty metres off an 81 mm board. A ref being re-seated
    has forfeited its band anyway: the band was measured off the pose being
    discarded.

    Gate, three conjuncts, because one lexicographic tuple is not enough here:

      1. Per-seat and structural, already free: `_try_place` demands full
         containment and `candidate_valid` -> `pads_ok` refuses any pose that
         worsens a pad pair, introduces an any-net stack, or worsens a hole
         shortfall.
      2. Board-wide `reconstruct.measure` with `edge_bands` computed EXCLUDING
         the scope, then a per-part `prune_assignment` sweep with
         `evidenced=scope` (a part coming back onto the board is gate-neutral
         on several terms by construction, exactly like the mounting-hole
         homecoming that rule was written for).
      3. A pass-specific conjunct the tuple cannot express, and it depends on
         `scope_source` (#698). See `reseat_accept`, which is the rule; in
         outline:

         * **AUTO scope** -- unchanged: `oob` must STRICTLY improve AND the
           witness count must not rise. The tuple's lexicographic comparison
           stops at the first differing term, and this pass moves `oob`
           (index 3) hugely in its own favour, which would HIDE a new stack,
           an hpwl blow-up or piled-on overlap below it.
         * **EXPLICIT scope** -- the same rule is unsatisfiable here, because a
           part that is legal and ON the board cannot move `oob` at all, so an
           explicitly named ref could never be re-seated whatever the search
           found. Instead: the witness count must not rise, no HARD gate term
           may worsen (`hpwl` is the one licensed term -- a seat made for a
           declared reason is hpwl-worse by construction), no declared claim
           may worsen termwise, and at least one basis in `RESEAT_BASES` must
           strictly improve. What stops a sideways move 'succeeding' is that
           `oob` is no longer the only thing measured, not that it is required.

    Returns `{'moves', 'reseated', 'refused', 'unseated', 'evicted', 'scope',
    'scope_source', 'notes', 'gate_before', 'gate_after', 'accepted',
    'accept_basis', 'witnesses_before', 'witnesses_after',
    'edge_bands_dropped', 'pruned'}`.
    `reseated` stays the SCOPE parts that moved; an evicted part is not a
    re-seat and is counted separately. `accept_basis` is the whole verdict --
    which basis carried the pass, what the safety half saw, and every basis
    that did not fire -- and it is present on EVERY return path, including the
    empty-scope early-out.

    NOT `placement/reseat.py`, which is a different mechanism for a different
    problem (Hungarian re-assignment of a proximity-tethered decap cluster onto
    rings around its anchor IC, refusing outright when the members' nets are
    rails). Do not merge them.
    """
    import dataclasses
    import pose_score
    from placement import floorplan, reconstruct as _recon

    if intent is None:
        intent = floorplan.empty_intent(pcb_file)

    # A caller's --lock globs, on top of the file's own (locked yes) stamps.
    # These two entry points RE-PARSE the board and build their own state, so
    # a lock resolved by the CLI never reached them -- `--lock` was honoured
    # by five reconstruct stages and silently ignored by the two that move the
    # most parts. Feeding it here means the existing `.locked` checks below
    # (the unrepairable filter, and reseat's refusal list) pick it up for free.
    _extra_locked = {r for pat in (lock_globs or [])
                     for r in fnmatch.filter(sorted(pcb_data.footprints), pat)}
    state = pose_score.make_state(
        pcb_data, pcb_file, clearance=clearance,
        board_edge_clearance=board_edge_clearance, grid_step=grid_step,
        extra_locked_refs=_extra_locked or None,
        # #701: the declared keep-outs reach the SEAT PREDICATE (see
        # `seed_from_intent`). `intent` is optional on this path, so the
        # inert default is what a caller without one gets.
        keepouts=intent.keepouts if intent else ())
    refs_all = sorted(pcb_data.footprints)
    notes: List[str] = []
    must_lock = {r for pat in intent.must_lock
                 for r in fnmatch.filter(refs_all, pat)}

    # ---- scope resolution --------------------------------------------------
    witnesses_before = _recon.damage_witnesses(state)
    if refs is None:
        scope = set(witnesses_before)
        scope_source = 'auto:damage_witnesses'
    else:
        scope = set()
        scope_source = 'explicit'
        for pat in refs:
            hits = fnmatch.filter(refs_all, pat)
            if not hits:
                notes.append(f"{pat}: matches no reference on this board")
            scope.update(hits)

    refused: Dict[str, str] = {}
    for ref in sorted(scope):
        if ref not in state.parts:
            refused[ref] = 'not a movable part on this board'
        elif state.parts[ref].locked:
            # NAME THE SOURCE. This said "(locked yes) in the file"
            # unconditionally, which is false for a ref locked by --lock -- and
            # sends the reader hunting the board for a stamp that is not there.
            if ref in _extra_locked:
                refused[ref] = ("locked by --lock on this invocation -- not "
                                "this tool's to move")
            else:
                why = ("in must_lock, which grades the lock rather than "
                       "licensing a move" if ref in must_lock
                       else "not in must_lock")
                refused[ref] = (f"(locked yes) in the file ({why}) -- not "
                                f"this tool's to move")
    for ref, why in sorted(refused.items()):
        notes.append(f"{ref}: {why}")
    scope -= set(refused)

    def _empty(reason: str) -> Dict:
        notes.append(reason)
        _empty_gate = _recon.measure(state, edge_bands or {})
        return {'moves': [], 'reseated': [], 'refused': sorted(refused),
                'intent_used': intent,
                'unseated': [], 'scope': [], 'scope_source': scope_source,
                'notes': notes, 'reason': reason,
                # The SAME keys as the seated path below. This early-out
                # omitted every census key, and `place_seed` papered over it
                # with defaulting `.get`s -- so one function returned two
                # different schemas and a reader could not tell "the census
                # found nothing" from "no census ran".
                'no_pose_blockers': {}, 'no_pose_verdict': {},
                'no_pose_census': {}, 'evicted': [],
                'evictions': 0, 'evictions_reverted': 0,
                'gate_before': list(_empty_gate),
                'gate_after': list(_empty_gate),
                # #698: the SAME keys as the seated path, for the reason the
                # census keys above are here -- one function must not return
                # two schemas. Built from the shared skeleton so the promise is
                # structural, not a literal someone has to keep in step.
                # `min_gain` is the caller's, not a fabricated 0.0: reporting a
                # threshold that was never the one in force is the same defect
                # as reporting a census that never ran.
                'accept_basis': basis_skeleton(
                    scope_source, policy='empty',
                    witnesses_before=witnesses_before,
                    witnesses_after=witnesses_before,
                    hpwl_before=_empty_gate[_recon.GATE_TERMS.index('hpwl')],
                    hpwl_after=_empty_gate[_recon.GATE_TERMS.index('hpwl')],
                    min_gain=min_gain),
                'accepted': True, 'pruned': [],
                'witnesses_before': sorted(witnesses_before),
                'witnesses_after': sorted(witnesses_before),
                'edge_bands_dropped': {}}

    if not scope:
        # A no-op is a RESULT. On a healthy board the auto scope is empty by
        # construction (zero witnesses on all 33 corpus boards), and that is
        # the property that makes this pass safe to put in a default ladder.
        # "No part NEEDS re-seating" is a claim about the board. When every
        # candidate was refused for being locked, the truthful claim is about
        # the LOCKS -- one run reported this while its own census still said
        # `OFF-OUTLINE PARTS 1 -> 1`.
        if refused:
            return _empty(f'{len(refused)} candidate(s) were refused (locked), '
                          f'so nothing was left to re-seat -- this is not '
                          f'"no part needs it"')
        return _empty('no part needs re-seating'
                      if refs is None else
                      'every named ref was refused or matched nothing')

    # ---- edge bands: the scope forfeits its own ----------------------------
    if edge_bands is None:
        edge_bands = {}
        # edge_claims(): the `or 2.0` default below is an off-outline
        # allowance for a part whose seat overhangs by design. A
        # connector_affinity entry carries `overhang_mm` with only a `min`,
        # so the raw key handed every generic header 2.0mm of licence in the
        # gate tuple -- the exact trap test_run23_connector_affinity's
        # edge-band test names.
        for c in intent.edge_claims():
            if c['ref'] in state.parts:
                band = c.get('overhang_mm') or {}
                edge_bands[c['ref']] = float(band.get('max') or 2.0)
    gate_bands = {r: m for r, m in edge_bands.items() if r not in scope}

    dropped = {}
    keep = []
    # Only an edge CLAIM has a band to forfeit, and only a claim is seated by
    # stage 1 without a legality gate (the 30km probe below). A
    # connector_affinity entry declares a class and no band, so it rides
    # through into `intent2` untouched rather than being reported as a
    # dropped declaration it never made.
    _claims = {c['ref'] for c in intent.edge_claims()}
    # The raw key here is deliberate: `intent2` below must stay a faithful
    # copy of the declaration list minus only the bands the scope forfeits,
    # and filtering it would silently strip every connector_affinity entry
    # from the sub-intent. Every OTHER engine read goes through
    # edge_claims(); the marker on the next line is what the source guard in
    # tests/test_run23_connector_affinity.py accepts, and it asserts this is
    # the ONLY one in the tree.
    for c in intent.edge_connectors:   # edge-claims-exempt: faithful copy
        if c['ref'] in scope and c['ref'] in _claims:
            dropped[c['ref']] = float((c.get('overhang_mm') or {}).get('max')
                                      or 0.0)
        else:
            keep.append(c)
    if dropped:
        notes.append(
            "dropped the edge declaration of " + ", ".join(
                f"{r} (band {m:g}mm)" for r, m in sorted(dropped.items()))
            + " -- a ref being re-seated has forfeited its band, which was "
              "measured off the pose being discarded. Stage 1 seats a declared "
              "edge connector with NO legality gate; leaving these in threw a "
              "part 30km off the board in a measured probe.")
    intent2 = dataclasses.replace(intent, edge_connectors=tuple(keep))

    # ---- the declared-claim probe (#698) ------------------------------------
    # Explicit scope only. On the auto scope the pass's win is `oob`, which the
    # gate tuple already ranks above `hpwl`, so nothing here is needed and
    # nothing here runs -- `place_reconstruct`'s ladder rung is bit-identical.
    #
    # MEASUREMENT ONLY. `state` was built by `pose_score.make_state`, which
    # hands the re-seat `keepouts` and deliberately withholds `intent_zones`
    # (pose_score.py:84-90) -- arming the monotone zone gate would make this
    # pass refuse its own target. `IntentProbe` assigns to neither
    # `state._intent_spec` nor `state._intent_active`.
    probe = None
    if scope_source == 'explicit':
        from placement.groups import parse_sources as _parse_sources
        from placement import quench as _q
        # `or auto`: resolving at a bare `()` makes every `group:`-shaped block
        # resolve to NOTHING, silently -- `cli_gates.resolve_intent_gate_for_cli`
        # states the same rule and the same reason.
        _srcs = tuple(group_sources) or _parse_sources('auto')
        _bundle, _problems = floorplan.resolve_intent_gate(
            intent, pcb_data, _srcs)
        for _v in _problems:
            # Reported, never dropped: a block that resolves to nothing gates
            # nobody and looks identical to a gate that is working.
            notes.append(f"intent gate: [{_v.rule}] {_v.message}")
        # The WHOLE BOARD, not `refs=scope`. The scope is not the set of parts
        # this pass can move: at `--evict-depth >= 1` it trades out neighbours
        # nobody named, and those refs are not known until `seed_from_intent`
        # has run -- after the "before" snapshot has to be taken. Scoped to the
        # named refs, the licence could not see an evicted stranger pushed INTO
        # a keep-out: the scope ref's own escape fires the `intent` basis, no
        # gate term moves, and the pass is accepted having created the
        # violation it was run to remove. Only claim-bound refs get terms, so
        # on a board that declares nothing this is still empty.
        probe = _q.IntentProbe(state, zones=_bundle['zones'])

    # ---- seat ---------------------------------------------------------------
    before = _recon.measure(state, gate_bands)
    intent_before = probe.snapshot() if probe is not None else None
    bases_before = (reseat_bases(before, intent_before['count'],
                                 scope_hpwl(state, scope))
                    if probe is not None else None)
    old = {r: (state.parts[r].x, state.parts[r].y, state.parts[r].rot)
           for r in sorted(scope)}
    res = seed_from_intent(
        pcb_data, pcb_file, intent2, random.Random(f"{seed}"),
        group_sources=group_sources, clearance=clearance,
        board_edge_clearance=board_edge_clearance, grid_step=grid_step,
        seed_refs=set(scope), evict_depth=evict_depth,
        # The seeder builds its OWN state, so `--lock` -- which this pass
        # resolved into ITS state as extra_locked_refs -- is invisible to the
        # eviction rung. Without this it would cheerfully trade out a ref the
        # user locked by name.
        immovable_extra=sorted(_extra_locked))
    notes.extend(res['notes'])

    # A part the rung evicted is OUTSIDE the scope, and `moves` is the whole
    # of what gets written: left out of it, the blocker is written at its old
    # pose while the scope ref takes the pocket it vacated -- overlapping
    # copper, exit 0. Snapshot them here, BEFORE any pose is applied, so the
    # revert below can put them back too.
    evicted = sorted({b for e in (res.get('evictions') or [])
                      if e.get('accepted')
                      for b in (e.get('blockers') or [e.get('blocker')])
                      if b and b not in scope})
    for r in evicted:
        if r in state.parts:
            pp = state.parts[r]
            old.setdefault(r, (pp.x, pp.y, pp.rot))
    adopt = set(scope) | set(evicted)

    # `placements` covers every PLACED ref -- 101 of 107 on the measured board
    # -- and `make_state` normalises rotation mod 360, so returning all of them
    # rewrites -112.5 -> 247.5 on parts this pass never touched and pollutes
    # every diff, every movie frame and every recovery measurement. Filter --
    # to the scope AND to the parts the rung was licensed to evict, which is
    # the only widening of this filter that does not bring the churn back.
    seated = {p['reference']: p for p in res['placements']
              if p['reference'] in adopt}
    for ref, p in sorted(seated.items()):
        state.apply_move(ref, p['new_x'], p['new_y'], p['new_rotation'])

    # ---- gate ---------------------------------------------------------------
    # An evicted part is EXEMPT from the per-part sweep, and that is the only
    # thing that makes the trade atomic. `evidenced` is not enough: it gates
    # only the EQUAL case (`reconstruct.py`: `after < base or (after == base
    # and ref not in evidenced)`), so a STRICT improvement reverts an
    # evidenced ref anyway -- and reverting an evicted blocker back into the
    # pocket the trade just gave away IS a strict improvement, because
    # GATE_TERMS ranks `hpwl` above `overlap`. Measured: prune put S2 back
    # inside BIG's courtyard on hpwl 24.4 -> 14.4, the eviction licence then
    # correctly refused the damaged board, and a legal pair trade
    # (violations [0,0.0] -> [0,0.0], oob 9.65 -> 0) was thrown away whole.
    #
    # `exempt` is the right mechanism and not a workaround: its own rationale
    # is "an edge-class seat is hpwl-worse BY DESIGN -- pruning it back would
    # undo the seat one stage later", which is exactly an evicted blocker
    # pushed to the rim. Its move is not unexamined; it passed
    # `_evict_trade`'s three conjuncts, and if the trade hurt the board the
    # whole-pass gate below throws it out rather than half of it.
    pruned = _recon.prune_assignment(state, old, notes,
                                     edge_bands=gate_bands,
                                     exempt=set(evicted),
                                     evidenced=set(scope),
                                     # #698: the sweep's tuple has no intent
                                     # term, so a seat that cleared a declared
                                     # keep-out reads as a pure hpwl loss and
                                     # is reverted before the gate below ever
                                     # runs. `None` on the auto scope, where
                                     # the pass's win IS in the tuple.
                                     intent_probe=(probe.terms if probe
                                                   is not None else None))
    after = _recon.measure(state, gate_bands)
    witnesses_after = _recon.damage_witnesses(state)
    _oob = _recon.GATE_TERMS.index('oob')
    intent_after = probe.snapshot() if probe is not None else None
    bases_after = (reseat_bases(after, intent_after['count'],
                                scope_hpwl(state, scope))
                   if probe is not None else None)
    _risen = ()
    if probe is not None:
        _lic_ok, _risen = probe.licence(intent_before, intent_after)
    accepted, accept_basis = reseat_accept(
        before, after, scope_source=scope_source,
        witnesses_before=witnesses_before, witnesses_after=witnesses_after,
        bases_before=bases_before, bases_after=bases_after,
        intent_risen=_risen, min_gain=min_gain)
    if evicted and not eviction_licence_ok(before, after):
        accepted = False
        accept_basis['fired'] = None
        accept_basis['eviction_licence'] = False
        _stk = _recon.GATE_TERMS.index('stacks')
        _ovl = _recon.GATE_TERMS.index('overlap')
        notes.append(
            f"the eviction licence is REFUSED: moving "
            f"{', '.join(evicted)} outside the scope raised stacks "
            f"{before[_stk]:g} -> {after[_stk]:g} or overlap "
            f"{before[_ovl]:g} -> {after[_ovl]:g}")
    if not accepted:
        for ref, (x, y, rot) in old.items():
            state.apply_move(ref, x, y, rot)
        witnesses_after = _recon.damage_witnesses(state)
        after = _recon.measure(state, gate_bands)
        notes.append(reseat_refusal_note(len(scope), accept_basis))

    if evicted:
        notes.append(
            (f"the eviction rung moved {len(evicted)} part(s) OUTSIDE the "
             f"scope to seat it: {', '.join(evicted)}. That is what "
             f"--evict-depth licenses; at depth 0 no part outside the scope "
             f"is touched.") if accepted else
            (f"the eviction rung traded {', '.join(evicted)} out of the "
             f"scope, but the pass was REFUSED and nothing was written -- "
             f"they are back where they started."))
    moves = []
    if accepted:
        # `adopt`, not `scope`: see the snapshot above -- an evicted part
        # missing from `moves` is written at its old pose.
        for ref in sorted(adopt):
            p = state.parts[ref]
            ox, oy, orot = old[ref]
            if (math.hypot(p.x - ox, p.y - oy) > 1e-9
                    or abs(p.rot - orot) > 1e-9):
                moves.append({'reference': ref, 'new_x': p.x, 'new_y': p.y,
                              'new_rotation': p.rot})

    return {'moves': moves,
            # The intent with the scope's edge declarations removed. A caller
            # that GRADES the result must grade against this, not the intent it
            # loaded: an entry declaring a 160 mm band was measured off the pose
            # this pass just discarded, so grading the homecoming against it
            # charges the repair for repairing (measured: 10 `R10 sits nearest
            # the west edge but is declared on the east edge` errors on a board
            # whose 11 off-outline parts had all come home). That is the same
            # laundering as the band itself, one step later.
            'intent_used': intent2,
            # SCOPE parts that moved. An evicted part is not a re-seat and
            # must not inflate the count this pass is judged by.
            'reseated': sorted(m['reference'] for m in moves
                               if m['reference'] in scope),
            # Parts this pass moved outside its scope IN THE BOARD IT WROTE.
            # Gated on `accepted` deliberately: `evicted` is the disclosure
            # channel for "the held-fixed contract was relaxed", and a
            # refused pass wrote nothing, so reporting it there would have a
            # consumer conclude the contract broke when the board is
            # untouched. The attempt is still visible in `evictions`.
            'evicted': evicted if accepted else [],
            'refused': sorted(refused),
            'unseated': sorted(r for r in res['unseated'] if r in scope),
            # The census travels with the verdict here too (#630): a scope
            # ref with no legal pose names what is in its way. The
            # eviction counts are 0 unless the caller passed `evict_depth`
            # (#699); they are carried either way so a reader sees the same
            # keys on both paths.
            'no_pose_blockers': {r: v for r, v in
                                 (res.get('no_pose_blockers') or {}).items()
                                 if r in scope},
            # #699's ledger travels with the verdict here too, same filter.
            'no_pose_verdict': {r: v for r, v in
                                (res.get('no_pose_verdict') or {}).items()
                                if r in scope},
            'no_pose_census': {r: v for r, v in
                               (res.get('no_pose_census') or {}).items()
                               if r in scope},
            'evictions': sum(1 for e in (res.get('evictions') or [])
                             if e.get('accepted')),
            'evictions_reverted': sum(1 for e in (res.get('evictions') or [])
                                      if not e.get('accepted')),
            'scope': sorted(scope), 'scope_source': scope_source,
            'notes': notes,
            'gate_before': list(before), 'gate_after': list(after),
            # #698: which scope-relevant term carried the pass, what the safety
            # half saw, and every basis that did NOT fire. All three bases are
            # always reported: a basis that measured nothing and a basis that
            # measured no change must not look alike.
            'accept_basis': accept_basis,
            'accepted': accepted, 'pruned': sorted(pruned),
            'witnesses_before': sorted(witnesses_before),
            'witnesses_after': sorted(witnesses_after),
            'edge_bands_dropped': {r: m for r, m in sorted(dropped.items())}}

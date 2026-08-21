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


def _keepout_entry(entry):
    """(rect, sides, allow) from either a bare rect or a full declaration.

    A bare 4-tuple stays legal because that is what a plan writes in the
    common case, and requiring the long form everywhere would make the
    simple keepout the awkward one.
    """
    if isinstance(entry, dict):
        return (tuple(entry['rect']), entry.get('sides') or None,
                entry.get('allow') or ())
    if len(entry) == 4 and all(isinstance(v, (int, float)) for v in entry):
        return tuple(entry), None, ()
    rect, sides, allow = entry
    return tuple(rect), (tuple(sides) if sides else None), tuple(allow or ())


def pose_ok(state, ref: str, x: float, y: float, rot: float,
            exclude: Set[str], forbid: Sequence = ()) -> bool:
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
    """
    part = state.parts[ref]
    r = part.rect(x, y, rot)
    if state.edge_gate.rect_outside_amount(r) > 1e-9:
        return False
    # DECLARED KEEPOUTS. The intent schema has had a `keepouts` rule since
    # #549 and it is GRADED and honoured by nothing: `keepout` appears in
    # floorplan.py's rule and in no seeding module, so a reserved strip could
    # only ever be reported after the fact. Checked here, in the one predicate
    # every seat goes through, so it cannot be honoured by some ops and not
    # others. Default empty, so every existing path is byte-identical.
    #
    # Each entry is (rect, sides, allow) with the same meaning the intent's
    # `keepouts` rule gives them: `sides` limits it to one face, and `allow`
    # names the refs it does NOT apply to -- a mounting-hole keepout exists
    # so the mounting hole can sit there, and one that excluded MH1 would be
    # the wrong shape of correct.
    for entry in forbid:
        k, sides, allow = _keepout_entry(entry)
        if allow and any(fnmatch.fnmatch(ref, pat) for pat in allow):
            continue
        if sides and getattr(part, 'side', None) not in sides:
            continue
        if not (r[2] <= k[0] or k[2] <= r[0] or r[3] <= k[1] or k[3] <= r[1]):
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
    into a 2x2mm zone parked, and the census answered over an unconstrained
    3mm disc, so one park read `baseline_poses: 64` ("64 legal poses with
    nothing lifted") about a part the same op had just refused to seat, and
    two more advised lifting U1 when the zone was the problem.

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


def census_window(within, grid_step=0.1):
    """The `(radius, step)` a pose census should sample a budget at.

    Lives here, beside `_try_place`, because it encodes what `_try_place`
    actually searches and must move with it.

    **Take the FINEST lattice whose sweep fits the budget of locations, and
    never a lattice coarser than `SEARCH_FINE_STEP_MM`.** The rule this
    replaces was `within / 8`, which lands on no lattice `_try_place` visits
    (0.1125 at `within=0.9`, 0.1875 at 1.5, 0.4875 at 3.9), so a park could
    report "lifting X frees 34 poses" and the retry find not one of them.

    The 1.0mm ring is deliberately excluded even though `_try_place` sweeps
    it. That ring exists to cross 30mm cheaply, not to decide whether a part
    fits, and a census that renders a verdict on it produces the one output
    worse than no census: a confident zero. Measured on splitflap_driver
    with an earlier "coarsest lattice with >= 6 samples" rule -- which chose
    1.0mm the moment the budget reached 6.0 -- a part that `_try_place`
    seats at FULL clearance censused `baseline 0, every blocker 0,
    censused: true` at `within=6.0`, while `within=5.99` counted 44. The
    cliff was not removed by that rule, only moved from 4.0 to 6.0.

    **The radius is the budget, clamped twice**: to `SEARCH_RADIUS_MM`,
    beyond which `_try_place` does not look at all once `max_disp` is set
    (so counting there is counting poses the retry can never visit); and to
    whatever `CENSUS_MAX_LOCATIONS` affords. `count_legal_poses` applies
    `max_disp` as a `continue` AFTER `_offsets` materialises the disc, so an
    unclamped radius is a list-building cost with nothing to show for it --
    a fixed 16mm built 103,041 offsets to hand back 81 at `within=0.5`, and
    an unclamped `within=500` built 1,002,001 (~72MB, 28s) before probing a
    single pose.

    **A TRUNCATED radius is not an error but it is not the whole answer** --
    `radius < within` means the census looked at the near field only, and
    the caller must say so rather than report the count bare.

    A census is not comparable ACROSS budgets -- a coarser lattice counts
    fewer poses over more ground -- which is why the window is reported with
    the count. What must be commensurate is baseline vs each blocker, and
    that holds because one census uses one window throughout.

    **THIS COSTS MORE THAN THE RULE IT REPLACES, AND THAT IS THE TRADE.**
    Measured, ten censused parks on splitflap_driver, before -> after:
    `within` 0.5 3.19s -> 1.41s (the radius clamp; 2.3x FASTER), 1.4 3.99 ->
    5.72, 2.0 5.36 -> 10.99 (the worst, 2.05x), 3.0 10.28 -> 11.16, 5.0
    16.85 -> 22.52. Total 40.9s -> 53.0s, **1.30x**. The extra time buys a
    lattice the seat search actually visits and the removal of the confident
    zero; `CENSUS_CAP` still short-circuits the case where poses exist, so
    the full price is only paid by the genuinely blocked part -- which is
    the one whose answer matters.
    """
    grid = max(0.05, float(grid_step or 0.1))
    if within is None:
        return CENSUS_RADIUS_MM, CENSUS_STEP_MM
    try:
        budget = float(within)
    except (TypeError, ValueError):          # noqa: BLE001
        budget = 0.0
    # NaN before the clamp: `min(nan, x)` is nan, and `math.ceil(inf)` raises
    # OverflowError straight out of `op_place_lift`, which has no census
    # guard. `plan_ops` validates `within > 0` and json.loads accepts
    # `Infinity`, so both are plan-reachable.
    if math.isnan(budget) or budget < 0.0:
        budget = 0.0
    budget = min(budget, SEARCH_RADIUS_MM)

    # Finest first. `grid` is what `_try_place` reaches inside
    # SEARCH_XFINE_RADIUS_MM; SEARCH_FINE_STEP_MM is the floor on coarseness.
    cands = sorted({s for s in (grid, SEARCH_FINE_STEP_MM) if s >= grid - 1e-9})
    max_rings = max(1, (math.isqrt(CENSUS_MAX_LOCATIONS) - 1) // 2)
    step = cands[-1]
    for s in cands:
        if max(1, int(math.ceil(budget / s - 1e-9))) <= max_rings:
            step = s
            break
    rings = min(max(1, int(math.ceil(budget / step - 1e-9))), max_rings)
    return round(rings * step, 6), step


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

    A disc of `within` is the wrong sample set once a constraint applies:
    the reachable poses are bounded by the zone, not by the budget, so
    `census_window` spends its whole location cap on ground the zone
    excludes and then has to coarsen the step to afford it. Measured on
    splitflap_driver, R1 packed into a 2x2mm zone: `census_window(3.0, 0.1)`
    picks step 0.25, and the zone leaves a feasible x-window for R1's centre
    **0.07mm wide** -- so a census that filters that disc by the zone counts
    ZERO with the blocker lifted, while `_try_place` reaches the window on
    its 0.1mm ring and seats. Threading the constraint without this moves
    the confident zero from the relaxation axis to the lattice axis instead
    of removing it.

    So: enumerate the feasible-CENTRE box instead, on the lattices
    `_try_place` sweeps AT EACH DISTANCE. That last clause is the half an
    earlier version got wrong: it chose one step by location count alone, so
    a large zone fell to the 1.0mm ring (the verdict `census_window` refuses
    to render) and a distant zone was sampled at 0.1mm where the search only
    reaches 0.25mm -- 4 of 8 parks in one ordinary `place_pack` then promised
    poses the retry could never collect.

    `reach_mm` is how far from the target the sweep actually got. Smaller
    than the budget means the near field only, and the caller must say so:
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
    # entry list, ten times per park, uncached).
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
    # verdict on it (see `census_window`).
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


def _evict_candidates(state, ref: str, tx: float, ty: float,
                      placed: Set[str], lock_refs: Sequence[str],
                      constraint=None, tol: float = 0.5) -> List[str]:
    """Seated, movable parts that could possibly be in `ref`'s way at (tx,ty).

    A superset, not a heuristic: the box is every pose `ref` can take within
    the census radius, inflated by its own reach and the clearance, so a part
    whose own inflated extent misses it cannot be within clearance of ANY
    candidate pose and would free exactly zero poses by construction. That is
    `build_neighbor_lists`' pruning argument (quench.py) with the travel
    budget replaced by the census radius.

    Then nearest-first, capped: the cap is the only approximation, and it is
    reported by the caller rather than hidden.
    """
    part = state.parts.get(ref)
    if part is None:
        return []
    r = part.rect(tx, ty, part.rot)
    reach = max(r[2] - r[0], r[3] - r[1]) / 2.0 + CENSUS_RADIUS_MM
    clr = state.clearance
    locked = set(lock_refs or ())
    # UNDER A ZONE THE TARGET IS NOT THE CENTRE OF THE QUESTION. `place_pack`
    # lays its members out on their own stride, so a member's target can sit
    # wholly outside the zone (measured: 3.3mm out on a 2x2 zone), and both
    # the box and the nearest-first cap were keyed on it -- so the 8 chosen
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
    for other in sorted(placed):
        if other == ref or other not in state.parts:
            continue
        op = state.parts[other]
        if op.locked or other in locked:
            continue          # not this tool's to move -- see reseat_scope
        orect = op.rect(op.x, op.y, op.rot)
        if (orect[2] + clr < bx0 or orect[0] - clr > bx1
                or orect[3] + clr < by0 or orect[1] - clr > by1):
            continue
        out.append((math.hypot(op.x - cx, op.y - cy), other))
    out.sort()
    return [b for _d, b in out[:EVICT_MAX_BLOCKERS]]


def count_legal_poses(state, ref: str, tx: float, ty: float,
                      exclude: Set[str], *,
                      radius: float = CENSUS_RADIUS_MM,
                      step: float = CENSUS_STEP_MM,
                      cap: int = CENSUS_CAP,
                      max_disp: Optional[float] = None,
                      rotations: Optional[Sequence[float]] = None,
                      forbid: Sequence = (),
                      constraint=None, tol: float = 0.5) -> int:
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
    n = 0
    for dx, dy in offsets:
        if max_disp is not None and math.hypot(dx, dy) > max_disp + 1e-9:
            continue
        x, y = round(tx + dx, 3), round(ty + dy, 3)
        for rot in rots:
            # Zone first: it is four float compares, where `pose_ok` ends in
            # `candidate_valid`. Measured 19x faster on a small zone.
            if not in_zone(x, y, rot):
                continue
            if pose_ok(state, ref, x, y, rot, exclude, forbid):
                n += 1
                if n >= cap:
                    return n
    return n


def near_miss_poses(state, ref: str, tx: float, ty: float,
                    exclude: Set[str], *,
                    radius: float = CENSUS_RADIUS_MM,
                    step: float = CENSUS_STEP_MM,
                    max_disp: Optional[float] = None,
                    rotations: Optional[Sequence[float]] = None,
                    forbid: Sequence = (), k: int = 3) -> List[Dict]:
    """The K nearest-to-legal poses, for a census that just counted ZERO.

    run-23: `count_legal_poses` == 0 was a dead end three times (J5 "0 legal
    poses in a 20mm disc", U6, J1), and each dead end was resolved by an
    UNCHECKED place_fixed assert that shipped an overlap. The information
    that would have resolved it honestly -- "the best pose in budget overlaps
    only SW1, by 0.8mm2; move SW1 and the seat exists" -- was never computed.

    Report-only: never applies a move. One entry per DISTINCT blocker set
    (three poses all blocked by the same part teach nothing extra), ranked by
    residual overlap area, side-aware, off-outline poses skipped (an
    off-board pose is a different problem, not a near-miss). Same lattice
    family as count_legal_poses; runs only on its zero path, which is
    already the slow case.
    """
    part = state.parts[ref]
    from pose_score import _offsets
    rots = list(rotations) if rotations is not None \
        else [part.rot] + [(part.rot + d) % 360 for d in (90.0, 180.0, 270.0)]
    obstacles = []
    for other, p2 in state.parts.items():
        if other == ref or other in exclude:
            continue
        if not (part.sides & p2.sides):
            continue
        obstacles.append((other, p2.rect(p2.x, p2.y, p2.rot)))
    best = []
    for dx, dy in _offsets(radius, step):
        if max_disp is not None and math.hypot(dx, dy) > max_disp + 1e-9:
            continue
        x, y = round(tx + dx, 3), round(ty + dy, 3)
        for rot in rots:
            r = part.rect(x, y, rot)
            if state.edge_gate.rect_outside_amount(r) > 1e-9:
                continue
            ov = 0.0
            hits = set()
            for other, r2 in obstacles:
                w = min(r[2], r2[2]) - max(r[0], r2[0])
                h = min(r[3], r2[3]) - max(r[1], r2[1])
                if w > 0 and h > 0:
                    ov += w * h
                    hits.add(other)
            if ov <= 1e-9:
                continue      # legal: count_legal_poses' business, not ours
            best.append((ov, x, y, rot, tuple(sorted(hits))))
    best.sort(key=lambda t: (t[0], t[1], t[2]))
    seen = set()
    out: List[Dict] = []
    for ov, x, y, rot, hits in best:
        if hits in seen:
            continue
        seen.add(hits)
        out.append({'pose': [x, y, rot], 'overlap_mm2': round(ov, 3),
                    'hits': list(hits)})
        if len(out) >= k:
            break
    return out


def _try_place(state, ref: str, tx: float, ty: float, exclude: Set[str],
               constraint=None, tol: float = 0.5,
               max_disp: Optional[float] = None,
               info: Optional[Dict] = None,
               deadline=None, forbid: Sequence = ()) -> Optional[float]:
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
        return pose_ok(state, ref, x, y, rot, exclude, forbid)

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
                    # Returns None rather than raising, so the caller's existing
                    # "no legal pose" path handles it with no new control flow --
                    # but `info` records WHY, because reporting an unfinished
                    # search as a measured failure is the same silent lie this
                    # whole change exists to remove.
                    if deadline is not None and deadline.expired():
                        if info is not None:
                            info['deadline'] = True
                        return None
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
    four sides (`legality.py:438-448`), so any along-edge overshoot is a
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
                 lo: float, hi: float) -> bool:
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
    """
    r = part.rect(x, y, part.rot)
    amt = state.edge_gate.rect_outside_amount(r)
    if not ((lo - 0.02) <= amt <= (hi + 0.02)):
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

    # Along-edge start: the declared zone center when one exists (the R2
    # spec-coordinate pattern -- a DERIVED home outranks minimal-move), else
    # the part's current projection (minimal move).
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
    cur = min(f_hi, max(f_lo, cur))

    # The declared band. `hi` None means "no stated maximum" -- allow twice the
    # midpoint target, which is what `overhang` was derived from, rather than
    # allowing anything.
    hi_eff = float(hi) if hi is not None else max(2.0 * overhang, lo + 1.0)

    ctx = state.legality_ctx
    ex = set(exclude or ())

    def conflict_free(px, py):
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
            sf = ctx.pair_shortfall(ref, other, pose_a=(px, py, part.rot))
            if sf.pad > 1e-6 or sf.hole > 1e-6:
                return False
        return True

    def on_board(px, py):
        return edge_seat_ok(state, part, px, py, edge, lo, hi_eff)

    for df in (0.0, 0.05, -0.05, 0.1, -0.1, 0.15, -0.15,
               0.2, -0.2, 0.3, -0.3, 0.4, -0.4):
        frac = min(f_hi, max(f_lo, cur + df))
        x, y = _edge_pose(part, state.board, edge, frac, overhang)
        x, y, converged = _edge_correct(state, ref, edge, x, y, overhang)
        if not converged or not on_board(x, y):
            continue
        if conflict_free(x, y):
            state.apply_move(ref, round(x, 3), round(y, 3), part.rot)
            return True
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
                     evict_depth: int = 1,
                     deadline=None, progress=None) -> Dict:
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

    `deadline` is an optional `krt_deadline.Deadline`, checked between parts in
    the connectivity-centroid stage (the only unbounded one -- stages 1/1.5/2
    are O(declared entries)). The partial is coherent by construction: a part
    is either fully seated, and in `placements`, or untouched. Untouched parts
    are named in `deadline_skipped` and are NOT `unseated` -- a search that ran
    out of clock has measured nothing. `progress` is an optional
    `(current, total, label)` callback.
    """
    import pose_score
    from placement import floorplan

    state = pose_score.make_state(
        pcb_data, pcb_file, clearance=clearance,
        board_edge_clearance=board_edge_clearance, grid_step=grid_step)
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
    # Parts a budget ran out before REACHING. Never `unseated`: a search that
    # never ran has measured nothing, and reporting it as a failure is a
    # verdict nobody earned. Declared here because the zone stage can fill it
    # too, long before stage 3.
    deadline_skipped: List[str] = []
    # ref -> (target_x, target_y, constraint_rect, tol) for the seat that
    # failed. The eviction rung (3c) retries at exactly the target the part
    # was refused at; a rung that re-derived one would be answering a
    # different question from the one that failed.
    unseated_ctx: Dict[str, Tuple[float, float, Any, float]] = {}
    evictions: List[Dict] = []
    no_pose_blockers: Dict[str, Dict[str, int]] = {}
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
    by_edge: Dict[str, List[Dict]] = {}
    for c in intent.edge_connectors:
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
            frac = (k + 1) / (len(specs) + 1)
            if f_lo > f_hi:
                notes.append(f"edge connector {ref}: wider than the {edge} "
                             f"edge, so stage 1 leaves it to the later stages")
                continue
            frac = min(f_hi, max(f_lo, frac))
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
            if not edge_seat_ok(state, part, x, y, edge, lo, hi_eff):
                notes.append(f"edge connector {ref}: the {edge} band would "
                             f"put it off the board, so stage 1 left it for "
                             f"the later stages")
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
                         constraint=rect, tol=tol, info=info,
                         deadline=deadline)
        if clr is None and z is not None:
            zx = (z.rect[0] + z.rect[2]) / 2.0
            zy = (z.rect[1] + z.rect[3]) / 2.0
            clr = _try_place(state, ref, zx, zy, unplaced - {ref},
                             constraint=rect, tol=tol, info=info,
                             deadline=deadline)
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
            # The zone stage was UNBOUNDED. `seed_from_intent`'s contract says
            # the deadline is checked "between parts in the connectivity-
            # centroid stage (the only unbounded one -- stages 1/1.5/2 are
            # O(declared entries))", and the entry COUNT is indeed bounded --
            # but each entry pays a full `_try_place` ladder, so on a board
            # whose intent zones every part (run 19's urchin: 85 refs in two
            # half-blocks) the whole run is stage 2 and the clock never got a
            # chance to fire. Measured: a 30s budget ran past 200s.
            if deadline is not None and deadline.expired():
                for r in members[members.index(ref):]:
                    if r in unplaced and r not in deadline_skipped:
                        deadline_skipped.append(r)
                notes.append(
                    f"deadline reached in zone {name!r}; "
                    f"{len(deadline_skipped)} part(s) left at their input "
                    f"poses and reported in deadline_skipped (NOT unseated -- "
                    f"they were never tried)")
                break
            jx, jy = (0.0, 0.0) if len(members) == 1 else _jitter()
            rot_before = state.parts[ref].rot
            zinfo: Dict = {}
            clr = _try_place(state, ref, cx + jx, cy + jy, unplaced - {ref},
                             constraint=z.rect, tol=tol, info=zinfo,
                             deadline=deadline)
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
    for _qi, ref in enumerate(queue):
        if deadline is not None and deadline.check('seed'):
            deadline_skipped.extend(r for r in queue[_qi:]
                                    if r not in deadline_skipped)
            notes.append(
                f"deadline reached after {_qi}/{len(queue)} part(s); "
                f"{len(deadline_skipped)} left at their input poses and "
                f"reported in deadline_skipped (NOT unseated -- they were "
                f"never tried)")
            break
        if progress is not None:
            try:
                progress(_qi, len(queue), f'seat {ref}')
            except Exception:                                   # noqa: BLE001
                pass
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
            unseated_ctx[ref] = (target[0] + jx, target[1] + jy, None, 0.5)
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
    # nearby incumbent lifted in turn, evict the one that frees the most, and
    # retry. THE ORDERING IS LOAD-BEARING -- the blocked part is seated FIRST,
    # against a board the blocker is lifted out of, and the blocker is
    # re-seated afterwards with it as an obstacle. Run 19's one-call reseat
    # got a null three times precisely because its queue re-seated the
    # blockers first, back into the pockets they block.
    #
    # Bounded on every axis: depth 1 (a blocker's own blocker is not chased),
    # at most EVICT_MAX_BLOCKERS candidates per part, and the census counts to
    # a cap. Gated exactly like the anchor rounds -- `measure` before and
    # after, with a snapshot revert -- so a trade that does not improve the
    # board leaves it byte-identical.
    # The CENSUS runs whenever anything is unseated; only the EVICTION is
    # gated on depth. #629 asks for a verdict that names its blockers, and
    # that is worth having on its own -- `--evict-depth 0` means "tell me
    # what is in the way, move nothing", which is the honest default for
    # anyone who wants to make the call themselves.
    if unseated:
        from placement.reconstruct import measure
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
                still.append(ref)
                continue
            # Checked between PARTS, like the stage-3 queue: one census is
            # `1 + candidates` bounded sweeps, so the grain is fine enough,
            # and a rung that overran its budget would be adding unbounded
            # work to the one path that had none.
            if deadline is not None and deadline.expired():
                still.extend(r for r in list(unseated)
                             if r not in seen and r not in placed)
                notes.append(
                    f"eviction rung: deadline reached; {len(still)} part(s) "
                    f"left uncensused (their bare verdict stands)")
                break
            tx, ty, constraint, tol = unseated_ctx[ref]
            base_excl = unplaced - {ref}
            # The constraint has been unpacked here since this rung was
            # written and was then dropped on the next two lines, so a part
            # that failed to seat INSIDE A ZONE was censused over the open
            # board. The retry below already passes it (`constraint=
            # constraint` at the `_try_place` call), so the measurement and
            # the retry were answering different questions -- and this one
            # reaches `place_seed`'s JSON_SUMMARY, which the placement skill
            # tells an agent to read.
            zkw = dict(constraint=constraint, tol=tol)
            baseline = count_legal_poses(state, ref, tx, ty, base_excl, **zkw)
            cands = _evict_candidates(state, ref, tx, ty, placed, lock_refs,
                                      constraint=constraint, tol=tol)
            freed = {b: count_legal_poses(state, ref, tx, ty,
                                          base_excl | {b}, **zkw)
                     for b in cands}
            no_pose_blockers[ref] = dict(freed)
            useful = sorted((n, b) for b, n in freed.items() if n > baseline)
            if not evict_depth:
                still.append(ref)
                if useful:
                    notes.append(
                        f"{ref}: no legal pose, and lifting {useful[-1][1]} "
                        f"would free {useful[-1][0]} -- not evicted "
                        f"(--evict-depth 0)")
                continue
            if not useful:
                still.append(ref)
                if cands:
                    notes.append(
                        f"{ref}: censused {len(cands)} neighbour(s); lifting "
                        f"none of them frees a pose, so they are not what is "
                        f"in the way")
                continue
            best = useful[-1][1]
            snapshot = {r: (state.parts[r].x, state.parts[r].y,
                            state.parts[r].rot) for r in (ref, best)}
            before = measure(state)
            # Lift, seat the blocked part, then put the blocker back with it
            # in place.
            unplaced.add(best)
            placed.discard(best)
            ok = _try_place(state, ref, tx, ty, (unplaced - {ref}) | {best},
                            constraint=constraint, tol=tol) is not None
            bx, by, brot = snapshot[best]
            ok = ok and _try_place(state, best, bx, by,
                                   unplaced - {best}) is not None
            after = measure(state) if ok else None
            if ok and after <= before:
                placed.add(ref)
                placed.add(best)
                unplaced.discard(ref)
                unplaced.discard(best)
                evictions.append({
                    'ref': ref, 'blocker': best, 'poses_freed': freed[best],
                    'poses_before': baseline, 'depth': 1, 'accepted': True,
                    'gate_before': list(before), 'gate_after': list(after)})
                notes.append(
                    f"{ref}: seated after evicting {best} (poses at its "
                    f"target: {baseline} before, {freed[best]} with {best} "
                    f"lifted); gate {list(before)} -> {list(after)}")
            else:
                for r, pose in snapshot.items():
                    state.apply_move(r, *pose)
                placed.add(best)
                unplaced.discard(best)
                still.append(ref)
                evictions.append({
                    'ref': ref, 'blocker': best, 'poses_freed': freed[best],
                    'poses_before': baseline, 'depth': 1, 'accepted': False,
                    'gate_before': list(before),
                    'gate_after': None if after is None else list(after),
                    'reason': ('the blocker had nowhere to go' if not ok
                               else 'the trade did not improve the board')})
                notes.append(
                    f"{ref}: evicting {best} REVERTED -- "
                    + ('it had no legal pose to return to'
                       if not ok else
                       f"gate worsened {list(before)} -> {list(after)}"))
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
            # each one frees. An empty dict for a ref means the census ran and
            # found no movable neighbour; a ref absent from the dict was never
            # censused (the rung was off, or it had no recorded target).
            'no_pose_blockers': no_pose_blockers,
            'evictions': evictions,
            'deadline_skipped': deadline_skipped,
            'complete': not deadline_skipped}


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
                     caps: Sequence[float] = REPAIR_CAPS_MM,
                     deadline=None, progress=None) -> Dict:
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

    `deadline` is an optional `krt_deadline.Deadline`. This sweep has no
    internal bound -- its cost is violators x caps x 36 ring sweeps x O(parts)
    per candidate -- and on a 217-part board it ran 46 minutes without
    terminating (run 9). When a budget is supplied the loop stops between
    violators, keeps every seat it already made, and reports the untouched ones
    in `deadline_skipped`. Those are NOT `unrepairable`: a search that ran out
    of clock has measured nothing, and filing it as a failure is the same class
    of lie as the silent hang.

    `progress` is an optional `(current, total, label)` callback -- the sweep is
    otherwise completely silent between its census line and its final line.
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
        extra_locked_refs=_extra_locked or None)
    refs_all = sorted(pcb_data.footprints)
    notes: List[str] = []
    must_lock = {r for pat in intent.must_lock
                 for r in fnmatch.filter(refs_all, pat)} if intent else set()
    # {ref: declared band max mm}. The off-board census below charges only the
    # EXCESS past the band, not nothing at all -- see the note there.
    edge_band: Dict[str, float] = {}
    if intent:
        for _c in intent.edge_connectors:
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
    pads = _leg.grade_pad_legality(pcb_data, clearance, worst_n=0)
    print(f"  Repair census: {pads['pad_conflicts']} conflict pair(s), "
          f"all listed")
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
        ordered = sorted(free, key=_mover_key)
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
    edge_entry = ({c['ref']: c for c in intent.edge_connectors}
                  if intent else {})
    repaired: List[str] = []
    failed: List[str] = []
    zero_move: List[str] = []   # run-7 A2: honesty re-grade candidates
    moves: List[Dict] = []
    deadline_skipped: List[str] = []
    for _vi, ref in enumerate(violators):
        # Budget check at the VIOLATOR head: one monotonic read per violator,
        # and each violator costs seconds (caps x 36 ring sweeps x O(parts)).
        # The partial is coherent by construction -- a violator is either fully
        # seated, with its move in `moves`, or untouched.
        if deadline is not None and deadline.check('legalize'):
            deadline_skipped = list(violators[_vi:])
            notes.append(
                f"deadline reached after {_vi}/{len(violators)} violator(s); "
                f"{len(deadline_skipped)} left untouched and reported in "
                f"deadline_skipped (NOT unrepairable -- they were never tried)")
            break
        if progress is not None:
            try:
                progress(_vi, len(violators), f'repair {ref}')
            except Exception:                                  # noqa: BLE001
                pass
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
                             tol=tol, max_disp=cap, info=info,
                             deadline=deadline)
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
                             tol=tol, deadline=deadline)
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
                                  max_disp=cap,
                                  deadline=deadline) is not None:
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
    if deadline_skipped:
        # Skip the honesty re-grade on a partial run. It is O(zero_move x parts)
        # with pair_shortfall, it can only DOWNGRADE `repaired`, and a partial
        # run's `repaired` is already qualified by complete:false -- so paying
        # for it here would just spend the clock we already ran out of.
        notes.append("unresolved re-grade skipped: run stopped on its deadline")
    elif zero_move and state.legality_ctx is not None:
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
            'deadline_skipped': deadline_skipped,
            'complete': not deadline_skipped,
            'pad_report_before': {k: pads[k] for k in
                                  ('pad_conflicts', 'hole_conflicts',
                                   'oob_pad_count')},
            'grade_errors_before': len(graded.errors) if graded else None}


def reseat_scope(pcb_data, pcb_file: str, intent, *,
                 lock_globs: Optional[Sequence[str]] = None,
                 refs: Optional[Sequence[str]] = None,
                 group_sources: Sequence[str] = (),
                 clearance: float = 0.25,
                 board_edge_clearance: float = 0.55,
                 grid_step: float = 0.1,
                 seed: int = 0,
                 edge_bands: Optional[Dict[str, float]] = None,
                 deadline=None, progress=None) -> Dict:
    """LIFT a subset of parts and re-seat them FROM SCRATCH at their net
    centroids, holding every other part fixed as an obstacle.

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
      3. A pass-specific conjunct the tuple cannot express: `oob` must STRICTLY
         improve AND the witness count must not rise. The tuple's lexicographic
         comparison stops at the first differing term, and a re-seat moves
         `oob` (index 3) hugely in its own favour -- which would HIDE a new
         stack, an hpwl blow-up or piled-on overlap below it. This conjunct is
         what stops the pass 'succeeding' by moving a part sideways.

    Returns `{'moves', 'reseated', 'refused', 'unseated', 'scope',
    'scope_source', 'notes', 'gate_before', 'gate_after', 'accepted',
    'witnesses_before', 'witnesses_after', 'edge_bands_dropped', 'pruned',
    'deadline_skipped', 'complete'}`.

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
        extra_locked_refs=_extra_locked or None)
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
        return {'moves': [], 'reseated': [], 'refused': sorted(refused),
                'intent_used': intent,
                'unseated': [], 'scope': [], 'scope_source': scope_source,
                'notes': notes, 'reason': reason,
                'gate_before': list(_recon.measure(state, edge_bands or {})),
                'gate_after': list(_recon.measure(state, edge_bands or {})),
                'accepted': True, 'pruned': [],
                'witnesses_before': sorted(witnesses_before),
                'witnesses_after': sorted(witnesses_before),
                'edge_bands_dropped': {}, 'deadline_skipped': [],
                'complete': True}

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
        for c in intent.edge_connectors:
            if c['ref'] in state.parts:
                band = c.get('overhang_mm') or {}
                edge_bands[c['ref']] = float(band.get('max') or 2.0)
    gate_bands = {r: m for r, m in edge_bands.items() if r not in scope}

    dropped = {}
    keep = []
    for c in intent.edge_connectors:
        if c['ref'] in scope:
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

    # ---- seat ---------------------------------------------------------------
    before = _recon.measure(state, gate_bands)
    old = {r: (state.parts[r].x, state.parts[r].y, state.parts[r].rot)
           for r in sorted(scope)}
    res = seed_from_intent(
        pcb_data, pcb_file, intent2, random.Random(f"{seed}"),
        group_sources=group_sources, clearance=clearance,
        board_edge_clearance=board_edge_clearance, grid_step=grid_step,
        seed_refs=set(scope), deadline=deadline, progress=progress)
    notes.extend(res['notes'])

    # `placements` covers every PLACED ref -- 101 of 107 on the measured board
    # -- and `make_state` normalises rotation mod 360, so returning all of them
    # rewrites -112.5 -> 247.5 on parts this pass never touched and pollutes
    # every diff, every movie frame and every recovery measurement. Filter.
    seated = {p['reference']: p for p in res['placements']
              if p['reference'] in scope}
    for ref, p in sorted(seated.items()):
        state.apply_move(ref, p['new_x'], p['new_y'], p['new_rotation'])

    # ---- gate ---------------------------------------------------------------
    pruned = _recon.prune_assignment(state, old, notes,
                                     edge_bands=gate_bands,
                                     evidenced=set(scope))
    after = _recon.measure(state, gate_bands)
    witnesses_after = _recon.damage_witnesses(state)
    _oob = _recon.GATE_TERMS.index('oob')
    accepted = (after[_oob] < before[_oob]
                and len(witnesses_after) <= len(witnesses_before))
    if not accepted:
        for ref, (x, y, rot) in old.items():
            state.apply_move(ref, x, y, rot)
        witnesses_after = _recon.damage_witnesses(state)
        after = _recon.measure(state, gate_bands)
        notes.append(
            f"REVERTED: re-seating {len(scope)} part(s) did not strictly "
            f"improve the off-board amount ({before[_oob]:g} -> "
            f"{after[_oob]:g}) or raised the witness count "
            f"({len(witnesses_before)} -> {len(witnesses_after)}). Both "
            f"conjuncts are required: the gate tuple is lexicographic, so a "
            f"large oob win would hide a new stack or an hpwl blow-up below "
            f"it, and a sideways move that changes neither is not a re-seat.")

    moves = []
    if accepted:
        for ref in sorted(scope):
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
            'reseated': sorted(m['reference'] for m in moves),
            'refused': sorted(refused),
            'unseated': sorted(r for r in res['unseated'] if r in scope),
            'scope': sorted(scope), 'scope_source': scope_source,
            'notes': notes,
            'gate_before': list(before), 'gate_after': list(after),
            'accepted': accepted, 'pruned': sorted(pruned),
            'witnesses_before': sorted(witnesses_before),
            'witnesses_after': sorted(witnesses_after),
            'edge_bands_dropped': {r: m for r, m in sorted(dropped.items())},
            'deadline_skipped': sorted(r for r in
                                       res.get('deadline_skipped') or ()
                                       if r in scope),
            'complete': res.get('complete', True)}

"""Generate an initial placement from a declared floorplan intent.

The placement stack refines; it deliberately does not place from scratch
UNAIDED (placement_state.py:13, docs/placement-optimization.md) -- handed a
pile of parts it has nothing to inherit constraints from. This module is the
aided path: the intent file IS the constraint carrier (zones, edge bands,
locks, decap rules), so a board whose repo declares one can get a legal,
deterministic, seeded starting placement instead of a refusal.

IT PLACES THE RESIDUE, NOT THE DECISIONS. Everything below is greedy
first-fit: a zone is packed radially from its centre, anything unzoned lands
on its connectivity centroid, and the first rotation that fits is kept. That
is the right shape for the many small parts and it is not a chooser -- it has
no representation for a pose that is a DECISION. An edge band says which edge
and not where along it; nothing says which way a mating face points, so a
connector is seated at the band's midpoint at whatever angle it came in with,
which on a pile is a generator default. Measured on esp_prog (run 27): both
free connectors came out at rotation 0, and one of them put the band midpoint
through a fixed socket's ground tab on every one of ten seeds. Place and lock
the parts whose pose is a decision first; seed what is left. The placement
driver's P1 enforces that, and `--waive seed-connectors:<why>` is how a
caller deliberately hands a connector to this module instead.

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

Rotations: UNDECLARED, the input rotation is tried IN FULL first and kept
when it fits; a part with no contained legal pose at it falls back to its
90-degree lattice, and the note names the change.

Since #893 the intent CAN express a rotation, which is what a part whose
rotation is a DECISION (pin order, the U3 rot-180 case) should use.
`blocks[].rotation` is honoured exactly -- a part that does not fit at it is
reported UNSEATED in `rotation_unseated`, never quietly turned -- and
`blocks[].rotation_candidates` narrows the ladder to the author's set, in the
author's order, because this search keeps the FIRST pose that fits. Every
`_try_place` seat builds that ladder with `floorplan.declared_ladder` --
including `place_seed`'s post-polish re-seat, which until #1117 searched the
fallback lattice and could turn a declared part. Stage 1's edge seat, which
calls no seat search, applies a set itself since #1120: the part's own angle
when that is a member that fits the edge, else the first member that does.

Note what a declared rotation deliberately does NOT do: it does not lock the
part. The advice this paragraph used to give -- lock it -- costs the part its
POSITION too, because `_Part.locked` is one boolean covering both, and
`place_seed` stamps it into the board. The angle is held by handing
`_try_place` a one-element ladder instead. `place_portfolio`'s `poses`
strategy is still how you EXPLORE rotations (within a declaration, since
#1121); this is how you FIX one.

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
from typing import Any, Dict, List, NamedTuple, Optional, Sequence, Set, Tuple

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
    # The grade ladder (#1182): board, keep-out and exclusive zone are the
    # floorplan grade's questions; `candidate_valid` asks the neighbours.
    r, tht = part.grade_rects(x, y, rot)
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
    # #797, the FOURTH conjunct. `rule_zone_exclusive` graded a reserved
    # rectangle that the seat search walked STRANGERS into: `place_seed` could
    # seat an unrelated part in the middle of a region the intent reserved and
    # then exit 4 against the intent it was built from. That is the round trip
    # the keep-out conjunct above closed, one rule over.
    #
    # ABSOLUTE, for the reason the paragraph above gives, and BEFORE
    # `candidate_valid` for the reason the keep-out one is: a handful of float
    # compares against a usually-empty dict, where `candidate_valid` ends in
    # the neighbour loop.
    #
    # BOTH rects are handed over and the answer is nonetheless COURTYARD ONLY.
    # `intent_term_values` takes `rects[0]` for a zone_exclusive term because
    # `rule_zone_exclusive` reads `part.rect` and never `tht_rect`; passing
    # `(r,)` here would put a SECOND copy of that decision in this file, and
    # the two would drift the first time one of them was revisited.
    if not state.exclusive_clear(ref, (r, tht)):
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
    from placement import floorplan as _fp
    # The anchor decision moved to `floorplan.zone_is_anchor` (#799) so the
    # load-time contradiction check asks the SAME question rather than a
    # re-derivation of it. Resolved through the module object, not a
    # `from ... import`, so a test that patches the decision is observed here.
    anchor = _fp.zone_is_anchor(constraint, part, tol)

    if anchor:
        def _in(x, y, rot):
            return (constraint[0] - tol <= x <= constraint[2] + tol
                    and constraint[1] - tol <= y <= constraint[3] + tol)
    else:
        def _in(x, y, rot):
            return _rect_inside(part.grade_rect(x, y, rot), constraint, tol)
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
    # The per-rotation algebra is `floorplan.zone_origin_box` since #799, which
    # needs it one rotation at a time; the union is this function's own shape.
    # BIT-IDENTICAL to the max/min form it replaces: subtraction is monotone in
    # its second operand, so `min_r(x0 - tol - b0_r)` is `x0 - tol - max_r(b0_r)`
    # evaluated with the same operands in the same order.
    from placement import floorplan as _fp
    boxes = [_fp.zone_origin_box(constraint, part.grade_rect(0.0, 0.0, rot % 360), tol)
             for rot in (part.rot, part.rot + 90.0,
                         part.rot + 180.0, part.rot + 270.0)]
    return (min(b[0] for b in boxes), min(b[1] for b in boxes),
            max(b[2] for b in boxes), max(b[3] for b in boxes))


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
        # #1101: a part on the OTHER face cannot be in this one's way unless
        # one of them is drilled; the census named StickHub's back-side U1,
        # C23, C27 as blockers of a front-side cap.
        if not (part.sides & op.sides):
            continue
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
                      without_keepouts: Sequence[str] = (),
                      without_exclusive: Sequence[str] = ()) -> int:
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

    `without_exclusive` is the same lever for declared EXCLUSIVE zones (#797),
    naming BLOCKS rather than keep-outs. Same argument: "this reserved zone is
    what refuses the part" has to be a NUMBER from the seat predicate, not a
    static "does the part's feasible region intersect the rect" test computed
    some other way.
    """
    from pose_score import _offsets
    part = state.parts[ref]
    rots = list(rotations) if rotations is not None \
        else [part.rot] + [(part.rot + d) % 360 for d in (90.0, 180.0, 270.0)]
    if rotations is None and getattr(state, 'diagonal_fallback', False):
        # #1099: `_try_place`'s fallback pass, so a census of legal poses
        # counts what the seat search can really reach.
        rots += [(part.rot + d) % 360 for d in (45.0, 135.0, 225.0, 315.0)]
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
    # #797: the same lift for exclusive zones, as a SECOND independent
    # save/restore rather than a shared one -- the two channels are lifted by
    # different callers for different questions, and one `saved` sentinel
    # covering both would restore a channel that was never swapped.
    #
    # No `_inc_intent.clear()` here, unlike the keep-out lift above, and the
    # difference is not an oversight: that cache holds the incumbent vector for
    # `intent_spec_for`, whose ARITY changes when a keep-out is lifted.
    # `exclusive_for` feeds no cached incumbent at all -- `exclusive_clear` is
    # absolute and reads the candidate rects only -- so clearing it here would
    # be harmless but would falsely signal that the two channels are coupled.
    lift_z = set(without_exclusive or ())
    saved_z = None
    if lift_z and state.exclusive_for.get(ref):
        saved_z = state.exclusive_for[ref]
        kept_z = tuple(t for t in saved_z if t.name not in lift_z)
        if kept_z:
            state.exclusive_for[ref] = kept_z
        else:
            del state.exclusive_for[ref]
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
        if saved_z is not None:
            state.exclusive_for[ref] = saved_z


#: The declared-intent rules the SEAT SEARCH enforces, as `quench.py` states
#: its own `INTENT_ENFORCED_RULES` for the per-move gate (#797).
#:
#: This exists because `docs/floorplan-intent.md` carries a "Which rules the
#: SEARCH can see" table whose seat-search column was RE-TYPED prose, and it
#: sat reading "no" against `zone_exclusive` for as long as it took someone to
#: notice -- which is this issue. A tuple the tests can read makes that column
#: derivable from the engine instead of remembered about it.
#:
#: Each rule reaches the search by a DIFFERENT mechanism, and the differences
#: are the reason the other six are absent rather than an oversight:
#:   zone_containment  the per-call `constraint` of `zone_gate`, anchor-aware
#:                     and per-part -- never a state-wide gate, because a
#:                     monotone one would make a repair refuse its own target
#:                     (pose_score.make_state, and test_698 arm H)
#:   zone_exclusive    `QuenchState.exclusive_clear`, ABSOLUTE, over the
#:                     zone_exclusive slice of the same `build_zone_spec` the
#:                     quench gate reads
#:   keepout           `QuenchState.keepout_clear`, ABSOLUTE, over
#:                     `floorplan.keepout_hit`
#: The remaining rules are graded and not gated, each for a stated reason --
#: see that document's table, whose "if not, why not" column is the record.
SEAT_ENFORCED_RULES = ('zone_containment', 'zone_exclusive', 'keepout')


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
    # #1213. Before it, a part refused by a LOCKED neighbour's pads read
    # `no_single_lift_frees` -- a sentence about the movable neighbours,
    # which were all at 0 -- and never named the part that refused it:
    # rp2350's U6 against U8, the locked Teensy frame. Measured, not
    # inferred: the poses are recounted with each frozen neighbour lifted.
    'frozen_blocks',
    # #701. Before it, a part a declared KEEP-OUT refuses reported
    # `no_movable_neighbour`, whose prose says "the outline, the zone or its
    # own size refuses it, not a neighbour" -- false, and the reader's next
    # move is different: move the keep-out, or add the part to its `allow`.
    'keepout_blocks',
    # #797, the same story one rule over. Before it, a part a declared
    # EXCLUSIVE zone refuses reported `no_movable_neighbour` (or, once
    # neighbours were censused, `no_single_lift_frees` -- "lifting any ONE of
    # them frees no pose", about a board where no neighbour was ever the
    # problem). The reader's next move is different again: there is no `allow`
    # list for an exclusive zone, because MEMBERSHIP is the allow list.
    'zone_exclusive_blocks',
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
            'keepouts_freeing': {},
            # #797: the same pair for declared EXCLUSIVE zones. Named after
            # the RULE rather than `zones_*`, which would collide with the
            # other zone in this file -- the part's OWN zone, which is the
            # per-call `constraint` and is not a census channel at all.
            'zone_exclusive_joint': 0,
            'zone_exclusive_freeing': {},
            # #1213: {frozen neighbour: poses freed by lifting it}, zeros
            # included, so "censused and frees 0" is not "never censused".
            # Filled only for a part with no pose at all. The rung still may
            # not move these parts; the count says which one is refusing.
            'frozen_lifted': {},
            # #1213: {frozen neighbour: poses legal with it as the ONLY
            # neighbour present}, against `open_poses` (every neighbour
            # lifted). 0 of a non-zero `open_poses` means that part alone
            # refuses every pose the outline and zone allow.
            'frozen_alone': {},
            'open_poses': 0,
            'frozen_truncated': 0}


def _frozen_refusers(census: Dict) -> List[Tuple[str, str]]:
    """`[(frozen ref, how it refuses)]`, the strongest first (#1213).

    A frozen neighbour refuses the part when lifting it frees poses
    (`frozen_lifted` above `baseline`), or when it ALONE -- every other
    neighbour lifted -- admits none of a non-zero `open_poses`. The second
    is the rp2350 case: U8 refused U6 everywhere, but by U6's turn the frame
    was full, so lifting U8 alone freed nothing."""
    base = census.get('baseline', 0)
    open_ = census.get('open_poses', 0)
    out = []
    for r, n in sorted((census.get('frozen_lifted') or {}).items(),
                       key=lambda kv: (-kv[1], kv[0])):
        if n > base:
            out.append((r, f"frees {n} pose(s) when lifted"))
    named = {r for r, _h in out}
    if open_:
        cap = (f"all of the first {open_} open poses censused (the census "
               f"cap)" if open_ >= CENSUS_CAP else
               f"all {open_} open pose(s)")
        for r, n in sorted((census.get('frozen_alone') or {}).items()):
            if n == 0 and r not in named:
                out.append((r, f"alone refuses {cap}"))
    return out


def _verdict_for(cands: Sequence[str], census: Dict) -> str:
    """The verdict for a part still unseated after the census."""
    if census.get('keepouts_freeing') or census.get('keepouts_joint'):
        # #701, and FIRST: a declared keep-out that frees poses when lifted
        # is the answer, whatever the neighbours look like. It outranks
        # `no_movable_neighbour`, whose prose would actively mislead here,
        # and it sits below `blocker_available` for free -- this function is
        # only reached when no trade was chosen.
        v = 'keepout_blocks'
    elif (census.get('zone_exclusive_freeing')
          or census.get('zone_exclusive_joint')):
        # #797, and BELOW `keepout_blocks` on purpose rather than by accident.
        # When both explain the refusal, name the one the reader CANNOT
        # change: a keep-out is usually a mechanical fact (a boss, a shell, an
        # enclosure wall), while an exclusive zone is a policy its author can
        # relax with one key. Telling someone to drop `exclusive` while a
        # heatsink boss also covers the pocket sends them to do work that will
        # not seat the part.
        #
        # Above `no_movable_neighbour` / `immovable_given_frozen` /
        # `no_single_lift_frees` for #701's reason: all three describe
        # NEIGHBOURS, and on a board where a declared claim is what refuses,
        # their prose is not merely unhelpful but false.
        v = 'zone_exclusive_blocks'
    elif not cands:
        v = ('immovable_given_frozen' if census.get('frozen')
             else 'no_movable_neighbour')
    elif _frozen_refusers(census):
        # #1213. Only where there ARE movable neighbours: there the old
        # verdicts below are sentences about THEM -- "lifting any one frees
        # no pose" -- and on rp2350 they were all at 0 while a frozen part
        # (U8) refused the seat everywhere. With no movable neighbour at all,
        # `immovable_given_frozen` (#699) already says the frozen ones are
        # what is in the way, and its note carries these counts too.
        v = 'frozen_blocks'
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
    if verdict == 'zone_exclusive_blocks':
        freeing = census.get('zone_exclusive_freeing') or {}
        if freeing:
            who = ', '.join(f"{n!r} (frees {c})"
                            for n, c in sorted(freeing.items()))
            what = (f"the EXCLUSIVE ZONE(S) of block(s) {who} are what refuse "
                    f"it -- {ref} is not a member of them, and no neighbour "
                    f"is involved")
        else:
            # The JOINT case, as `keepout_blocks` has: naming one here would
            # be false of every individual one, which is why the sentence does
            # not.
            what = (f"the declared exclusive zones are JOINTLY what refuse it "
                    f"-- no single one frees a pose, lifting all of them frees "
                    f"{census.get('zone_exclusive_joint', 0)}, and no "
                    f"neighbour is involved")
        # The reader's next move is NOT a keep-out's. There is no `allow` list
        # for an exclusive zone -- membership IS the allow list -- so all three
        # options are edits to the intent file and all three are named.
        return (f"{ref}: no legal pose, and {what}. Add {ref} to the block "
                f"that owns the zone, move the zone, or drop its `exclusive` "
                f"flag")
    if verdict == 'frozen_blocks':
        refusers = _frozen_refusers(census)
        f, how = refusers[0]
        why = frozen.get(f, 'immovable')
        step = ("unlock it, or check how it is modelled -- a frame's pin "
                "ring is not its body" if why == 'file-locked'
                else "relax the intent clause that froze it")
        also = ''
        if len(refusers) > 1:
            also = '; also ' + ', '.join(f"{r} ({h})"
                                         for r, h in refusers[1:])
        # What the movable census found as well, in the words its own
        # verdicts use, so this note never says less than they would.
        if census.get('pairs_censused'):
            movable = (f"censused {n} movable neighbour(s) and "
                       f"{census.get('pairs_censused', 0)} pair(s); lifting "
                       f"no one or two of them frees a pose")
        else:
            movable = (f"censused {n} movable neighbour(s); lifting any ONE "
                       f"of them frees no pose"
                       + ("; --evict-depth 2 also tries pairs"
                          if evict_depth < 2 else
                          "; fewer than two movable neighbours, so there is "
                          "no pair to try" if n < 2 else ""))
        return (f"{ref}: no legal pose; {f} ({why}) {how} -- {ref}/{f} is "
                f"the refusing pair, and {f} is not this rung's to "
                f"move{also}. Next: {step}. ({movable}{tail})")
    if verdict == 'no_movable_neighbour':
        return (f"{ref}: no legal pose, and NOTHING seated is near enough to "
                f"be in the way -- the outline, the zone or its own size "
                f"refuses it, not a neighbour")
    if verdict == 'immovable_given_frozen':
        who = ', '.join(f"{r} ({why})" for r, why in sorted(frozen.items()))
        measured = _frozen_refusers(census)
        tail = ('; measured: ' + ', '.join(f"{r} {how}"
                                          for r, how in measured)
                if measured else '')
        return (f"{ref}: no legal pose, and every neighbour that could be in "
                f"the way is one this rung may not move: {who}. Immovable "
                f"GIVEN those, not immovable{tail}")
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


#: Dispositions of a part the seed could not seat (#1151).
UNSEATED_DISPOSITIONS = ('locked_at_input', 'off_board', 'clear_at_input',
                         'staged', 'not_modelled')

#: Gap between the board (or the lowest part rect) and the staging row, and
#: between staged parts on it, in mm.
STAGING_GAP_MM = 5.0
STAGING_PITCH_GAP_MM = 1.0


def _dispose_unseated(state, refs: Sequence[str],
                      locked=(), waivers=(),
                      keepouts=()) -> Dict[str, Dict]:
    """Decide where each part the seed could not seat is WRITTEN (#1151).

    A part with no seat used to keep the pose it came in with, while every
    later seat excluded it as part of "the pile" -- so its neighbours were
    packed onto copper that was then written exactly there. Measured: all six
    of StickHub's OFF-seed `body_blocking` pairs involve J6, which the seed
    never moved, and 14 of rp2350's 15 OFF-seed stacks involve U6.

    The fix is applied AFTER the search, so every seated pose stays exactly
    what the search chose. Making the input pose an obstacle instead was
    measured worse (#982: ulx3s 20 -> 25 unseated over ten seeds, rp2350
    1 -> 3). In sorted order, each part is:

      * `locked_at_input` -- locked, in the file or by the seed (`locked`:
        the intent's must_lock and seated fixed poses, which the caller
        stamps `(locked yes)`): never moved -- a staged part stamped locked
        would be frozen off the board;
      * `off_board`       -- its rect is already wholly outside the board;
      * `clear_at_input`  -- at its input pose it makes no HARD conflict with
        what was seated or left before it (`_graded_input_conflicts`): left
        where it is;
      * `staged`          -- it does: moved to a deterministic row below the
        board, rotation kept, and written there;
      * `not_modelled`    -- not a search part (nothing to decide).

    HARD means what check_assembly gates whatever moved, between the part and
    a neighbour, READ FROM check_assembly's OWN CHANNELS on the board as it
    would be written (`_graded_input_conflicts`): a pad intersection, a
    gating containment (pads under a body included), a pin frame's pin under
    its courtyard, a plug's mating region, a coincident origin -- with the
    intent's `overlap_waivers` honoured as check_assembly honours them. NOT
    the seat predicate (`pose_ok`), which also demands full containment and a
    clearance gap: measured on ulx3s, it called the designer's own edge
    connectors J1/J2 illegal at their overhanging edge poses and staged them
    off the board -- 36% more crossings and four edge_connector intent errors
    on a seed with no stack at all. Nor a mirror of the grader built from the
    search's own predicates: that one was stricter (AABB pads, a rect
    containment test, no waivers) and looser (no mating region, no
    coincident origin) at once (phase-2 verifier).

    Returns `{ref: {disposition, input, written, refused_by}}`; `refused_by`
    is `candidate_veto`'s `(check, blocker)` at the input pose for a staged
    part, the conjunct and the part that refused leaving it there. The
    caller writes every `staged` part at `written`; the unseated count and
    the exit code do not change -- a staged part is still unseated."""
    from .legality import rect_overlap_area
    out: Dict[str, Dict] = {}
    todo = [r for r in sorted(set(refs))]
    bb = getattr(getattr(state, 'pcb_data', None), 'board_info', None)
    bb = getattr(bb, 'board_bounds', None)
    staged: List[str] = []
    # One grade of the board with every part in `refs` at its INPUT pose and
    # every other at the pose the search gave it; each part below reads its
    # own conflicts from it, partners filtered by what is decided so far.
    _pending = [r for r in todo if r in state.parts
                and not (state.parts[r].locked or r in locked)]
    graded = (_graded_input_conflicts(state, _pending, waivers=waivers,
                                      keepouts=keepouts)
              if _pending else {})
    for i, ref in enumerate(todo):
        part = state.parts.get(ref)
        if part is None:
            out[ref] = {'disposition': 'not_modelled', 'input': None,
                        'written': None, 'refused_by': None}
            continue
        pose = (part.seed_x, part.seed_y, part.orig_rot)
        rec = {'input': [round(v, 4) for v in pose], 'written':
               [round(v, 4) for v in pose], 'refused_by': None}
        r = part.rect(*pose)
        if part.locked or ref in locked:
            rec['disposition'] = 'locked_at_input'
        elif bb and rect_overlap_area(r, bb) <= 1e-9:
            rec['disposition'] = 'off_board'
        else:
            # The parts still to decide are not obstacles yet; the ones
            # already LEFT where they are now are, so two unseated parts are
            # never both left on one spot.
            undecided = set(todo[i + 1:]) | set(staged)
            if graded is None:
                conflict = _input_pose_conflict(state, ref, pose, undecided)
            else:
                conflict = next((c for c in graded.get(ref, ())
                                 if c[1] not in undecided), None)
            if conflict is None:
                rec['disposition'] = 'clear_at_input'
            else:
                rec['refused_by'] = list(conflict)
                rec['disposition'] = 'staged'
                staged.append(ref)
        if rec['disposition'] != 'staged' and (part.x, part.y, part.rot) \
                != pose:
            state.apply_move(ref, pose[0], pose[1], pose[2])
        out[ref] = rec
    if staged:
        # Below the board AND below every part rect, so a staged part can
        # never land on anything -- including a pile that sits off-board.
        lowest = max(p.rect()[3] for p in state.parts.values())
        floor = max(bb[3], lowest) if bb else lowest
        cursor = bb[0] if bb else min(p.rect()[0]
                                      for p in state.parts.values())
        for ref in staged:
            part = state.parts[ref]
            rot = part.orig_rot
            lx0, ly0, lx1, _ly1 = part.rect(0.0, 0.0, rot)
            x = cursor - lx0
            y = floor + STAGING_GAP_MM - ly0
            state.apply_move(ref, x, y, rot)
            out[ref]['written'] = [round(x, 4), round(y, 4), rot]
            cursor = x + lx1 + STAGING_PITCH_GAP_MM
    return out


def _graded_input_conflicts(state, refs, waivers=(), keepouts=()):
    """`{ref: [(channel, other), ...]}`: every HARD finding check_assembly
    makes between a part in `refs` -- written at its INPUT pose -- and
    another part, every other part at the pose the search gave it (#1151).

    The grader is CALLED, on the board as it would be written: written to a
    scratch copy (siblings carried, so the project's courtyard severity and
    rules hold) and graded with `grade_body_overlap` (pad intersections,
    gating containments -- pads under a body included -- and a pin frame's
    pins), `floorplan.mating_keepout_findings` (a plug's mating region; the
    `other` is the keep-out's name) and `placement_state.
    coincident_stack_groups`. Sorted, so the first conflict a part reports
    is not a hash accident. None when the board cannot be written or graded
    (no source file): the caller falls back to `_input_pose_conflict`."""
    import contextlib
    import io
    import os
    import shutil
    import tempfile
    from kicad_parser import parse_kicad_pcb
    from . import floorplan as _fp
    from .legality import grade_body_overlap
    from .placement_state import coincident_stack_groups, is_assembly_marker
    from .writer import write_placed_output
    src = getattr(state, 'pcb_file', None)
    if not src or not os.path.isfile(src):
        return None
    refs = set(refs)
    placements = []
    for r, p in sorted(state.parts.items()):
        x, y, rot = ((p.seed_x, p.seed_y, p.orig_rot) if r in refs
                     else (p.x, p.y, p.rot))
        placements.append({'reference': r, 'new_x': x, 'new_y': y,
                           'new_rotation': rot})
    td = tempfile.mkdtemp(prefix='dispose_')
    try:
        dst = os.path.join(td, os.path.basename(src))
        with contextlib.redirect_stdout(io.StringIO()):
            if not write_placed_output(src, dst, placements):
                return None
        from copy_board import SIBLING_EXTS          # ONE list (#711)
        for ext in SIBLING_EXTS:
            sib = os.path.splitext(src)[0] + ext
            if os.path.isfile(sib):
                shutil.copy2(sib, os.path.splitext(dst)[0] + ext)
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(dst)
            g = grade_body_overlap(pcb, getattr(state, 'clearance', 0.2),
                                   intent_waivers=tuple(waivers or ()),
                                   pcb_file=dst)
            mating = _fp.mating_keepout_findings(
                pcb, dst, declared=tuple(keepouts or ()))
            stacks = coincident_stack_groups(pcb, dst)
    except Exception:                                        # noqa: BLE001
        return None
    finally:
        shutil.rmtree(td, ignore_errors=True)
    out: Dict[str, List[Tuple[str, str]]] = {}

    def _add(ref, channel, other):
        if ref in refs and other != ref:
            out.setdefault(ref, []).append((channel, other))
    for channel, key in (('pads', 'blocking_pairs'),
                         ('containment', 'containment_blocking_pairs'),
                         ('pin_in_courtyard', 'pin_in_courtyard_pairs')):
        for q in g.get(key) or ():
            _add(q.a, channel, q.b)
            _add(q.b, channel, q.a)
    for m in mating or ():
        _add(m.get('ref'), 'mating', str(m.get('keepout')))
    for grp in stacks or ():
        parts = [r for r in grp['refs'] if not is_assembly_marker(pcb, r)]
        for r in parts:
            for o in parts:
                _add(r, 'coincident', o)
    return {r: sorted(set(v)) for r, v in out.items()}


def _input_pose_conflict(state, ref: str, pose, exclude: Set[str]):
    """`(channel, other)` for the first HARD conflict `ref` makes at `pose`
    with a part not in `exclude`, or None (#1151), from the SEARCH's own
    predicates: pad copper overlapping or stacked (`pair_shortfall`'s
    `pad_overlap` / `stack`), a body contained in a body, pads under a body.
    The FALLBACK of `_dispose_unseated`, for a state with no board file to
    grade -- `_graded_input_conflicts` is the answer whenever there is one,
    because this mirror is both stricter and looser than check_assembly."""
    ctx = getattr(state, 'legality_ctx', None)
    if ctx is not None:
        for other in sorted(state.parts):
            if other == ref or other in exclude:
                continue
            sf = ctx.pair_shortfall(ref, other, pose_a=pose)
            if sf.pad_overlap or sf.stack:
                return ('pads', other)
    if state._body_contained_at(ref, *pose, exclude=exclude):
        return ('containment', None)
    if state._pads_under_body_at(ref, *pose, exclude=exclude):
        return ('pads_under_body', None)
    return None


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
    # #1212: a seated part on a seated frame's pin.
    if getattr(state, 'pin_frame_refs', None):
        for a in refs:
            if a in containers:
                continue
            pa = state.parts[a]
            if state._pin_conflict_at(a, pa.x, pa.y, pa.rot,
                                      exclude=others) is not None:
                count += 1
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
            # #1104: on a courtyard-waived project a courtyard intersection is
            # not a violation; the pad/hole arm below still is.
            gap = (None if getattr(state, 'courtyards_ignored', False)
                   else pa.gap_to(pb, ra))
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
                 placed: Set[str], unplaced: Set[str],
                 rot_ladder=None) -> Dict:
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
    # #893. A declared rotation binds the EVICTED part and every blocker
    # put back after it, not just the parts the ordinary stages seat. Without
    # this the rung is a hole in the claim: a trade that turns a declared part
    # keeps it, silently, and the note the stages emit is not even printed
    # here. `rot_ladder` is `seed_from_intent._rot_ladder`; None (every other
    # caller) keeps the fallback ladder exactly.
    _ladder = rot_ladder if rot_ladder is not None else (lambda _r: None)
    clr_ref = _try_place(state, ref, tx, ty, pile | lifted,
                         constraint=constraint, tol=tol,
                         rotations=_ladder(ref))
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
                                     constraint=bz, tol=btol,
                                     rotations=_ladder(b))
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


def _materialise_rotation(part, rot: float) -> float:
    """Make `part.bounds_by_rot` (and `tht_by_rot`) hold an entry for `rot`.

    Returns the normalised angle, which is the key `_Part.rect` looks up.

    A DECLARED angle need not lie on the part's 90-degree lattice, and
    `_Part.rect` silently falls back to `bounds_by_rot[0.0]` for an angle it
    has no entry for -- so a declared 45 on a part seeded at 0 would be judged
    for overlap, halo and containment on the UNROTATED box and then written
    out at 45. The quench's nudge loop materialises the entry before using it
    for exactly this reason; every seat search must too.

    KEYED BY THE NORMALISED ANGLE, which is what `_Part.rect` looks up
    (`bounds_by_rot.get(rot % 360)`). `_try_place` fills by the RAW angle
    instead (`_Part.ensure_rotation(_r)`), so a part whose board spells its rotation
    -45 gets an entry at -45 that `rect` never reads and is judged on the
    unrotated box anyway. That is a real bug and it is deliberately NOT fixed
    here: 14 of 40 corpus boards spell a rotation outside [0, 360), fixing it
    moves placement on them, and it is measurable -- with `_try_place` routed
    through this function, `mutate_711`'s `both-along-edge-forms-allowed` row
    flips from KILLED to SURVIVED. It owes its own change and its own A/B.
    Declared angles reach here already normalised (`floorplan._rotation`), so
    the two callers cannot disagree on anything a declaration can express.
    """
    rot = rot % 360.0
    part.ensure_rotation(rot)
    return rot


def _facing_rank(state, ref: str, tx: float, ty: float, rot: float,
                 exclude: Set[str], *, edge_refs: Set[str]) -> int:
    """How many of `ref`'s connected pads would sit on a row facing the
    outline with nothing beyond it, at pose (tx, ty, rot). The seeder's
    opt-in rotation tie-break; the geometry is `placement.edge_facing`, the
    same core `placement_score.edge_facing` and `floorplan.rule_pins_to_edge`
    read, so what the search prefers is what the term then reports.

    A declared edge connector ranks 0 at every angle: its mating row SHOULD
    face the edge, and the edge stage seats it by its band anyway. Partners
    are read over PLACED parts only -- the pile (`exclude`) sits at one
    meaningless coordinate -- so early parts see few partners and the count
    is an over-estimate for them; a tie-break, not a cost.
    """
    if ref in edge_refs:
        return 0
    from placement.edge_facing import MIN_PADS, count_pads_to_edge, pitch_of
    part = state.parts[ref]
    pads_all = part.pad_globals(tx, ty, rot)
    pads = [(x, y, n) for x, y, n in pads_all if n > 0]
    if len(pads) < MIN_PADS:
        return 0
    xs = [x for x, _, _ in pads_all]
    ys = [y for _, y, _ in pads_all]
    rect = (min(xs), min(ys), max(xs), max(ys))
    centre = ((rect[0] + rect[2]) / 2.0, (rect[1] + rect[3]) / 2.0)
    nets = {n for _, _, n in pads}
    partners: Dict[int, List[Tuple[float, float]]] = {}
    for n in nets:
        for other in state.net_refs.get(n, ()):
            if other == ref or other in exclude:
                continue
            op = state.parts.get(other)
            if op is None:
                continue
            for px, py, pn in op.pad_globals():
                if pn == n:
                    partners.setdefault(n, []).append((px, py))
    return count_pads_to_edge(
        pads, rect, state.board, partners, centre,
        pitch=pitch_of((x, y) for x, y, _ in pads_all))['to_edge']


def seat_clearances(full: float) -> Tuple[float, float, float]:
    """The courtyard-clearance ladder every seat search walks: the full
    clearance, then half, then a 0.02mm floor (`_try_place`'s docstring says
    why a dense board needs the floor). One tuple, so a single-part seat and
    a row seat (#1051) relax in the same steps."""
    return (full, full / 2.0, min(0.02, full))


def _set_seat_clearance(state, clr: float) -> None:
    """Set the state's clearance for one rung of `seat_clearances`.
    candidate_valid reads state.clearance; the incumbent-violation cache is
    keyed on it implicitly, so clear it on every change."""
    state.clearance = clr
    state._inc_violation.clear()


_RING_CACHE: Dict[Tuple[float, float], Tuple[Tuple[float, float], ...]] = {}


def _ring_offsets(radius: float, step: float):
    """`pose_score._offsets(radius, step)`, built once per (radius, step).
    Pure and pose-free, so caching it changes no order; it is ~40ms a call
    and a seed makes one per ring per rotation per clearance rung."""
    key = (float(radius), float(step))
    got = _RING_CACHE.get(key)
    if got is None:
        from pose_score import _offsets
        got = tuple(_offsets(radius, step))
        _RING_CACHE[key] = got
    return got


def seat_candidates(state, tx: float, ty: float, *,
                    max_disp: Optional[float] = None, sweep: bool = True):
    """Yield candidate (x, y) seats around (tx, ty), nearest-first, in the
    order the seat search has always tried them: a 1.0mm ring out to
    SEARCH_RADIUS_MM, a FINE ring near the target, an XFINE (grid-step) ring,
    then -- when `sweep` and no `max_disp` cap -- a whole-board sweep at
    FALLBACK_STEP_MM, nearest the target first.

    Lifted out of `_try_place`'s `_first_fit` closure (#1051) so the row seat
    (`_seat_block`) walks the SAME offsets for a row's anchor rather than a
    second copy that drifts. Lazy: a caller that stops at the first fit pays
    only for the candidates it looked at, as the closure did. Positions are
    rounded to 3dp, the pose `apply_move` writes.

    `sweep` is False for a zone-constrained search (a part stays in its
    zone); `max_disp` (a capped repair) never sweeps the whole board either.
    """
    for _name, band in seat_candidate_bands(state, tx, ty, max_disp=max_disp,
                                            sweep=sweep):
        yield from band()


def seat_candidate_bands(state, tx: float, ty: float, *,
                         max_disp: Optional[float] = None,
                         sweep: bool = True):
    """`[(name, band)]`: `seat_candidates`' bands, each a zero-argument
    callable yielding its positions lazily -- 'ring' (1.0mm), 'fine',
    'xfine', then 'sweep' when it applies. `seat_candidates` chains them in
    that order; the row seat (`_seat_block`) walks the SAME bands in its own
    order, so the offsets are shared and only the order differs."""
    out = []
    xfine = max(0.05, getattr(state, 'grid_step', 0.1) or 0.1)
    for name, radius, step in (('ring', SEARCH_RADIUS_MM, SEARCH_STEP_MM),
                               ('fine', SEARCH_FINE_RADIUS_MM,
                                SEARCH_FINE_STEP_MM),
                               ('xfine', SEARCH_XFINE_RADIUS_MM, xfine)):
        if max_disp is not None and max_disp < step - 1e-9:
            # run-7 A3: a ring whose step exceeds the cap can contribute
            # nothing but used to burn a full sweep
            continue

        def _ring(radius=radius, step=step):
            for dx, dy in _ring_offsets(radius, step):
                if (max_disp is not None
                        and math.hypot(dx, dy) > max_disp + 1e-9):
                    continue
                yield round(tx + dx, 3), round(ty + dy, 3)
        out.append((name, _ring))
    if sweep and max_disp is None:
        out.append(('sweep', lambda: _sweep_positions(state, tx, ty)))
    return out


def _sweep_positions(state, tx: float, ty: float):
    """The whole-board sweep at FALLBACK_STEP_MM, nearest the target first."""
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
        yield x, y


def _try_place(state, ref: str, tx: float, ty: float, exclude: Set[str],
               constraint=None, tol: float = 0.5,
               max_disp: Optional[float] = None,
               info: Optional[Dict] = None,
               rotations: Optional[Sequence[float]] = None) -> Optional[float]:
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
    against 3 at rot 90 on a packed 51x21 board).

    `rotations` REPLACES that ladder with a declared one (#893) -- a single
    angle for `blocks[].rotation`, the author's set for
    `rotation_candidates` -- so a part whose rotation is a decision is seated
    at it or not at all, and the quench's gate pins it there afterwards. This
    paragraph used to end "a part whose rotation IS a decision must be locked";
    that advice froze the part's POSITION as well, which is exactly what the
    declaration exists to avoid. The caller can see a
    fallback fired by comparing the part's rot before and after. Every
    production caller passes a ladder -- `floorplan.declared_ladder(...)`,
    or stage 2.5's chip lattice for an undeclared cap (#1099) -- so
    None reaches here only for an undeclared part; a call with no
    `rotations=` at all is what #1117 was, and test_893 refuses one anywhere
    in the source trees.

    Returns the courtyard clearance the pose was found at, or None. The full
    clearance is demanded first; when the whole board offers nothing, the
    search reruns at half, then at a 0.02mm floor -- dense boards carry
    sub-clearance courtyard pairs BY DESIGN (the reference hand seed for the
    51x21 board places its LDO 0.04mm from a locked decap; a 0.05 floor
    still refused that board), and refusing to seed what a human
    deliberately packs would fail real boards. Courtyards carry their own
    margin, so a small courtyard-to-courtyard gap is not a copper hazard. A
    relaxed placement is a NOTE for the caller, never silent."""
    part = state.parts[ref]

    def _ok(x, y, rot):
        return pose_ok(state, ref, x, y, rot, exclude)

    _in_zone, anchor_zone = zone_gate(part, constraint, tol)
    if anchor_zone and info is not None:
        info['anchor_zone'] = True

    # #1099: PREFER, THEN FALL BACK. The 90-degree lattice (or the
    # declared ladder) is searched at every clearance step first, exactly
    # as before; only when it seats NOTHING anywhere does a second pass try
    # the diagonals -- so a part that fits orthogonally lands where it
    # always did, and the diagonals can only turn an unseated part into a
    # seated one. A declared ladder is the author's decision and gets no
    # fallback. StickHub's human packs 39 parts at +-45/+-135 degrees
    # around its diagonal QFP; the seeder could not produce one.
    _passes = [rotations]
    if rotations is None and getattr(state, 'diagonal_fallback', False):
        _passes.append([(part.rot + d) % 360
                        for d in (45.0, 135.0, 225.0, 315.0)])
    full = state.clearance
    try:
        for _pass_i, _pass_rots in enumerate(_passes):
            if _pass_i and info is not None:
                info['diagonal_fallback'] = True
            for clr in seat_clearances(full):
                _set_seat_clearance(state, clr)
                # #893. `rotations` is the DECLARED ladder when an intent gave
                # this ref one -- a single angle for `blocks[].rotation`, the
                # author's set for `rotation_candidates` -- in the author's order,
                # because this search keeps the FIRST pose that fits and a
                # reordered ladder changes which angle wins. None keeps the
                # fallback ladder every caller had before #893, byte for byte.
                _ladder_rots = (list(_pass_rots) if _pass_rots is not None
                                else [part.rot] + [(part.rot + d) % 360
                                                   for d in (90.0, 180.0, 270.0)])
                # #893 (PR932 form, VERBATIM -- see the commit message).
                for _r in _ladder_rots:
                    part.ensure_rotation(_r)
                # OPT-IN (`seed_from_intent(rotate_by_facing=True)`): let every
                # angle of the ladder find its own first fit, and keep the pose
                # with the fewest connected pads on a row facing the outline
                # with nothing beyond; a tie keeps #893's author order, and with
                # the preference unset the search below is the one it always
                # was. Why not a cost: this search has none (it keeps the first
                # pose that fits), and the quench that follows it runs with its
                # facing terms at zero (measured to fail a 4-board A/B, #932).
                # MEASURED (tests/test_placement_ab.py, the facing-seed rows):
                # the count it ranks by falls on two boards of three, and a
                # guard rises on both of them (pin-order inversions on both;
                # crossings and wire length on one), so it is REJECTED as a
                # default and stays opt-in for a caller who has read that
                # trade. The numbers are in the baseline file.
                _pref = getattr(state, 'rotation_prefer', None)

                def _first_fit(rot):
                    """The first legal (x, y) for `rot` in the order
                    `seat_candidates` yields, or None. A closure so the
                    preferred-rotation path below can ask it once per angle; the
                    ORDER lives in `seat_candidates`, which the row seat
                    (`_seat_block`, #1051) walks too."""
                    # The rings, then (unconstrained and uncapped only) the
                    # whole-board sweep -- PER ANGLE. The sweep is part of "first
                    # fit": the first lift of the rings into a closure left it
                    # outside the OFF path, and splitflap's default seed went from
                    # 0 to 6 unseated parts. `_in_zone` is trivially true on the
                    # sweep, which is only reached with no constraint.
                    for x, y in seat_candidates(state, tx, ty, max_disp=max_disp,
                                                sweep=constraint is None):
                        if not _in_zone(x, y, rot):
                            continue
                        if _ok(x, y, rot):
                            return x, y
                    return None

                if _pref is None or len(_ladder_rots) < 2:
                    # The search as it has always been: the first angle of the
                    # ladder that fits anywhere (rings, then the sweep) wins.
                    for rot in _ladder_rots:
                        hit = _first_fit(rot)
                        if hit is not None:
                            state.apply_move(ref, hit[0], hit[1], rot)
                            return clr
                else:
                    # OPT-IN (`seed_from_intent(rotate_by_facing=True)`): every
                    # angle finds ITS OWN first fit, and the pose with the fewest
                    # connected pads on a row facing the outline wins; ties keep
                    # #893's ladder order. Ranked at the pose each angle actually
                    # takes, not at the target: the first form of this ranked the
                    # ladder at (tx, ty) and then let the search seat the winner
                    # anywhere -- measured on esp_prog, the count it was chosen
                    # for did not move (3 -> 3) while crossings, hpwl and
                    # inversions all worsened. Costs up to four searches per part
                    # instead of one.
                    best = None
                    for i, rot in enumerate(_ladder_rots):
                        hit = _first_fit(rot)
                        if hit is None:
                            continue
                        key = (_pref(ref, hit[0], hit[1], rot, exclude), i)
                        if best is None or key < best[0]:
                            best = (key, hit[0], hit[1], rot)
                    if best is not None:
                        state.apply_move(ref, best[1], best[2], best[3])
                        return clr
    finally:
        _set_seat_clearance(state, full)
    return None


def _edge_pose(part, bounds, edge: str, frac: float, overhang: float
               ) -> Tuple[float, float]:
    """Center coordinates that put the part's courtyard `overhang` mm past
    the named edge of the BOUNDING BOX, at fraction `frac` along it. A first
    guess only -- see _edge_correct for why it cannot be the answer."""
    lx0, ly0, lx1, ly1 = part.grade_rect(0.0, 0.0, part.rot)
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
                  target: float, band=None) -> Tuple[float, float, bool]:
    """Walk the pose along the edge normal until the MEASURED overhang hits
    `target`. The analytic pose measures against the bounding box, but the
    grade's rule_edge_connector measures rect_outside_amount against the real
    Edge.Cuts rings (since #961 it grades the drawn body instead wherever one
    can be measured, which `band` and `_body_band_correct` follow) -- on a
    non-rectangular outline the two differ by the
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
        amt = state.edge_gate.rect_outside_amount(part.grade_rect(x, y, part.rot))
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
        amt = state.edge_gate.rect_outside_amount(part.grade_rect(x, y, part.rot))
        converged = abs(target - amt) < 0.02
    if band is not None and converged:
        return _body_band_correct(state, ref, edge, x, y, target, band)
    return x, y, converged


def _body_band_correct(state, ref: str, edge: str, x: float, y: float,
                       target: float, band) -> Tuple[float, float, bool]:
    """#961: the second rung of `_edge_correct`, taken only when the first
    rung's pose would be REFUSED by the band `edge_seat_ok` now grades.

    The walk above converges `rect_outside_amount` -- the occupancy reading
    at the gate's margin -- on `target`. Where the part's drawn body can be
    measured, the band is graded on the body instead (see
    `connector_geometry`), and the two disagree by the margin and by any gap
    between courtyard and body: esp_prog's USB1 has a pad-box courtyard 1.6 mm
    inboard of a body flush with the west edge. A pose the walk converged on
    and the body band accepts is returned UNCHANGED, so every seat upstream
    produced that is still legal is bit-identical. Only a pose the band would
    refuse is moved, analytically, along the declared edge's normal, to put
    that edge's signed position on `target`; a body that cannot be measured
    leaves the walk's pose alone. The convergence check is on the SUMMED
    overhang, so a corner part -- whose second edge one normal cannot fix --
    is reported unconverged rather than seated.

    What that does NOT promise: that every seat is the one upstream chose.
    Where the walk's pose was REFUSED, this rung can make it legal, so a
    ladder that used to fall through to a later rung, rotation or stage can
    now seat at the earlier one. The Round3 test whose name ends
    "ladder_seats_on_the_body_band" is that case at its simplest: a body
    reaching 3 mm west of its only pad seats at x 2.0 here, and nowhere at
    all without this rung.
    """
    from .connector_geometry import geometry_for
    part = state.parts[ref]
    geometry = geometry_for(state, state.pcb_data, state.pcb_file)
    row = geometry.measure(ref, edge, (x, y, part.rot))
    lo, hi = band
    if (not row['body_measured']
            or (lo - 0.02) <= row['body_outside_mm'] <= (hi + 0.02)):
        return x, y, True
    err = target - row['body_signed_position_mm']
    if edge == 'north':
        y -= err
    elif edge == 'south':
        y += err
    elif edge == 'west':
        x -= err
    else:
        x += err
    row = geometry.measure(ref, edge, (x, y, part.rot))
    return x, y, (row['body_measured']
                  and abs(target - row['body_outside_mm']) < 0.02)


#: #1044: `edge_seat_ok`'s rule-area band conjunct. On in every production
#: path; `tests/test_placement_ab.py` turns it off for its OFF arm only
#: (`seed_from_intent(_edge_band_gate=False)`), which is why it is module
#: state rather than a parameter threaded through every edge caller.
_edge_band_gate = True


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

    Five conjuncts in all: the band, pads on the board, and (below) a
    declared keep-out, an exclusive zone and a rule-area band (#1044).

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
    r, tht = part.grade_rects(x, y, part.rot)
    amt = state.edge_gate.rect_outside_amount(r)
    # #961: the band in the currency `rule_edge_connector` now grades it in
    # -- the drawn body at zero margin where it can be measured, `amt` itself
    # where it cannot -- so this predicate and the rule read one number, as
    # they did before (agreeing up to this check's own +/-0.02 tolerance,
    # which the rule does not share, exactly as upstream).
    from .connector_geometry import band_amount, geometry_for
    geometry = geometry_for(state, state.pcb_data, state.pcb_file)
    amt, _basis, _body = band_amount(geometry, part.ref, edge, amt,
                                     state.edge_gate.margin,
                                     pose=(x, y, part.rot))
    if not ((lo - 0.02) <= amt <= (hi + 0.02)):
        return False
    if _body.get('body_measured'):
        # The band used to be read off the COURTYARD, which on a connector
        # that draws none is the pad box itself, so it usually carried pad
        # copper past the outline. The drawn body never does, and the rule
        # now names that copper (#961 round 3) -- so this predicate must see
        # it too, or the seat accepts a pose the grade refuses, which is
        # exactly what the pad conjunct below exists to prevent. The case is
        # committed as the Round3 test whose name ends
        # "refuses_a_pose_whose_pad_copper_is_off_the_board": a body flush
        # with the edge while a pad sits 0.75 mm past it. CONTAINMENT only,
        # at zero margin -- the edge-clearance floor is check_drc's question.
        from .connector_geometry import pad_copper_outside
        from .legality import BoardOutlineGate
        zero = getattr(state, '_zero_edge_gate', None)
        if zero is None:
            zero = BoardOutlineGate(state.pcb_data.board_info, 0.0)
            state._zero_edge_gate = zero
        off = pad_copper_outside(geometry, zero, part.ref, (x, y, part.rot))
        if off > 1e-9:
            if reasons is not None:
                reasons.append(f'pad copper {off:.3f}mm past the outline')
            return False
    _blockers = state.keepout_blockers(part.ref, (r, tht))
    if _blockers:
        if reasons is not None:
            reasons.extend(f"keep-out {n!r}" for n in _blockers)
        return False
    # A FOURTH conjunct, since #797, and here for the same reason the keep-out
    # one is: this predicate deliberately bypasses `pose_ok`, so a rule added
    # there is absent here unless it is added here too. An edge connector's
    # body may leave the OUTLINE; it may not enter a region some other block
    # reserved, and neither the band nor the pad test can see that.
    #
    # THE TRADE THIS MAKES, stated as #701 stated its own: a stranger edge
    # connector whose entire declared band lies inside a reserved zone will
    # slide along the edge, exhaust, and fall through to the ordinary stages
    # -- trading a `zone_exclusive` error for an `edge_connector` one. That is
    # a worse-looking grade about a better board, and it is reported BY NAME
    # through `reasons` rather than as a silent missing pose.
    _zblockers = state.exclusive_blockers(part.ref, (r, tht))
    if _zblockers:
        if reasons is not None:
            reasons.extend(f"exclusive zone of block {n!r}" for n in _zblockers)
        return False
    # A FIFTH conjunct (#1044), for the same reason as the fourth: this
    # predicate bypasses `pose_ok`, and so bypassed `pads_ok`'s #1031 check
    # of the board's rule-area keep-out bands. Stage 1 and `_seat_edge` could
    # put an SMD connector's pad copper in a `(tracks not_allowed)` band --
    # a pad no track can reach -- and the polish quench then took that pose
    # as the seed's licence. ABSOLUTE, like `_fixed_pose_check`'s: an edge
    # seat is chosen, not inherited, so a band pose has no incumbent to be
    # "no worse than". The trade is the keep-out conjunct's: a connector
    # whose whole band lies in the rule area is left to the later stages,
    # named in `reasons`.
    if _edge_band_gate:
        ctx = getattr(state, 'legality_ctx', None)
        if ctx is not None and getattr(ctx, 'keepouts', None) is not None:
            ko = ctx.keepout_amount(part.ref, x, y, part.rot)
            if ko > 1e-6:
                if reasons is not None:
                    reasons.append(f"pad copper {ko:.3f}mm into a rule-area "
                                   f"keep-out band")
                return False
    gate = state.edge_gate
    for px, py, _sz in part.pad_globals(x, y, part.rot):
        # A zero-size rect at the pad centre: "is this point on the board",
        # asked through the gate so cutouts and milled rings count.
        if gate.rect_outside_amount((px, py, px, py)) > 1e-9:
            return False
    return True


# ---- #975: the pad-copper edge floor, as a PREFERENCE of the edge seat ------
#
# `edge_seat_ok` never measured pad copper against the board-edge floor: its
# pad conjunct asks whether each pad CENTRE is on the board at the gate margin,
# so a seat could leave a connector's shield tabs inside the floor that
# `grade_pad_legality`'s `pad_edge` then reports (tigard J7 0.073 mm short,
# ulx3s AUDIO1 0.385). It is not added as a fifth conjunct. A refusal turns a
# DRC shortfall into an unseated connector, and an unseated connector is an
# unrouted one, so the seat PREFERS a pose whose copper clears the floor and
# otherwise keeps the pose it always chose, disclosed in `edge_floor_fallback`.
#
# The reading is `legality.EdgeCopperContext.pose_copper`, the grader's own
# per-pad arithmetic at the trial pose with the board's files read once per
# state, so the seat and `pad_edge` agree by construction. A pad whose amount
# cannot be measured (an unsupported shape) keeps only the checks it had.

#: The grader names sides by coordinate (min-y is 'bottom'); the seeder names
#: them by compass (min-y is north). One table, so no call site re-derives it.
_GRADE_SIDE = {'left': 'west', 'right': 'east', 'bottom': 'north', 'top': 'south'}
#: The unit step that moves a part INTO the board from each edge.
_INWARD = {'north': (0.0, 1.0), 'south': (0.0, -1.0),
           'west': (1.0, 0.0), 'east': (-1.0, 0.0)}
#: Added to a derived shift so it survives `apply_move`'s 3-dp rounding, which
#: can take back up to half a micron per axis.
_FLOOR_SHIFT_GUARD_MM = 0.001
#: #983: the step a rung is moved along its edge when the pose it would
#: WRITE falls outside its declared window. A rung clamped to a window end
#: puts the courtyard centre exactly on it, and `round(x, 3)` then moves it up
#: to half a micron either way against the grade's 1 nm EPS; one step of the
#: 0.001 mm grid the pose is written on puts it back at least half a micron
#: inside.
_WINDOW_GUARD_MM = 0.001
#: A record names at most this many pads; the counts beside it are complete.
_FLOOR_RECORD_PADS = 4
#: The same bound for `grade_delta` ROWS, which are not pads: a row is a
#: rule the move would break, and `n_grade_delta` counts them all, so a
#: fifth one is summarised rather than silently dropped.
_FLOOR_ROWS = 4
#: The reasons a first seat stays short for which the ladder WALKS to later
#: rungs. A shortfall on another side than the seated one (a pad past its
#: courtyard at a corner), or on a sampled outline, is one only an along-edge
#: rung can change. Every other reason -- the inward move refused by the band,
#: a setback, a keep-out, a neighbour, still short, the grade's nearest edge
#: or along-edge window -- is about the MOVE. A later rung can still help
#: there, by having a different move accepted: away from a corner its gap to
#: the seated edge is the same, but near one `_edge_correct` converges on the
#: sum of every side and the gap varies from rung to rung. A random search
#: found grade-clean seats given up on 153 of 20,000 inputs (51 of them within
#: 1 mm; 102 were nearest-edge refusals, 23 still short). But walking trades
#: the connector's along-edge position for 0.1 mm-scale copper, and the first
#: version of this ladder, walking on every reason, slid one connector 10 mm
#: to clear 0.03 mm. So this is a CHOICE to stay put, not a claim that nothing
#: along the edge would do.
_SLIDE_HELPS = frozenset(('along_edge', 'outline_sampled'))


class _Floor(NamedTuple):
    """The floor reading of one pose: `short` is `(amount, index, side,
    gap, pad_number)` per measured pad over the floor, worst first."""
    required: float
    short: Tuple
    unmeasured: Tuple[int, ...]


def _floor_at(state, ref: str, x: float, y: float, rot: float):
    """`_Floor` for `ref` at the pose `apply_move` would WRITE, or None when the
    board's constants cannot be read (the seat then behaves as it always did).
    """
    from .legality import EPS, edge_copper_for
    ctx, _err = edge_copper_for(state, state.pcb_data, state.pcb_file,
                                state.clearance, state.board_edge_clearance)
    fp = state.pcb_data.footprints.get(ref) if ctx is not None else None
    if fp is None:
        return None
    reading = ctx.pose_copper(fp, (round(x, 3), round(y, 3), rot))
    short = sorted(((r.amount_mm, r.index, _GRADE_SIDE.get(r.edge), r.gap_mm,
                     r.pad.pad_number)
                    for r in reading.pads
                    if r.amount_mm is not None and r.amount_mm > EPS),
                   key=lambda s: (-s[0], s[1]))
    return _Floor(ctx.required, tuple(short), reading.fallback)


def _floor_context_note(state, notes: List[str]) -> None:
    """Say ONCE per state that the floor preference is off, and why."""
    from .legality import edge_copper_for
    ctx, err = edge_copper_for(state, state.pcb_data, state.pcb_file,
                               state.clearance, state.board_edge_clearance)
    if ctx is None and not getattr(state, '_edge_floor_noted', False):
        state._edge_floor_noted = True
        notes.append(f"edge seats: the board-edge copper floor could not be "
                     f"read ({err}), so seats were chosen without it")


def _faces_its_edge(state, part, entry: Dict, edge: str, x: float, y: float) -> bool:
    """Does `rule_edge_connector` read this pose as nearest its declared edge?

    Asked of every pose the floor preference picks over the seat the ladder
    always chose, because that seat is the only one the grade has already been
    measured against. The rect and its basis are the rule's own
    (`floorplan.edge_seat_rect`, the drawn body of a receptacle) at the pose
    `apply_move` writes."""
    from .floorplan import _nearest_edge, drawn_body_rect, edge_seat_rect
    bounds = state.pcb_data.board_info.board_bounds
    if not bounds:
        return True
    px, py = round(x, 3), round(y, 3)

    def body():
        bodies = getattr(state, '_edge_seat_bodies', None)
        if bodies is None:
            from .body import board_bodies
            try:
                bodies = board_bodies(state.pcb_data, state.pcb_file)
            except Exception:                                 # noqa: BLE001
                bodies = {}
            state._edge_seat_bodies = bodies
        fp = state.pcb_data.footprints.get(part.ref)
        if fp is None:
            return None, 'none'
        from copy import copy
        moved = copy(fp)
        moved.x, moved.y, moved.rotation = px, py, part.rot
        return drawn_body_rect(bodies.get(part.ref), moved)

    rect, _basis = edge_seat_rect(entry, part.grade_rect(px, py, part.rot), body)
    return _nearest_edge(rect, tuple(round(v, 6) for v in bounds)) == edge


def _carries_setback(entry: Dict) -> bool:
    """Does `rule_edge_connector` grade this entry's SETBACK? It does for an
    explicit `max_setback_mm` and for the `edge_receptacle` and
    `connector_affinity` classes, and only once the part has no overhang."""
    return (entry.get('max_setback_mm') is not None
            or entry.get('class') in ('edge_receptacle', 'connector_affinity'))


def _band_reading(state, part, edge: str, x: float, y: float):
    """`(amount, basis, occupancy)` at the pose `apply_move` WRITES: the band
    reading `rule_edge_connector` grades -- the drawn body's overhang where it
    can be measured, else the occupancy reading at the gate margin -- and that
    occupancy reading itself, which is what gates the setback."""
    from .connector_geometry import band_amount, geometry_for
    px, py = round(x, 3), round(y, 3)
    legacy = state.edge_gate.rect_outside_amount(part.grade_rect(px, py, part.rot))
    amount, basis, _row = band_amount(
        geometry_for(state, state.pcb_data, state.pcb_file), part.ref, edge,
        legacy, state.edge_gate.margin, pose=(px, py, part.rot))
    return amount, basis, legacy


def _grade_band_refuses(state, part, entry: Dict, edge: str, lo: float,
                        x: float, y: float):
    """`(reason, detail)`: would `rule_edge_connector` refuse this pose's
    overhang band, or charge its setback? `reason` is None when it would not.

    Asked of every pose the floor preference picks over the seat the ladder
    always chose. The ladder's own band test (`edge_seat_ok`) allows 0.02 mm
    either side of the band, and a moved pose or a later rung can land in
    that margin where the first seat did not (measured: a later rung read
    0.23 on a 0.25 minimum and turned `--repair` from rc 0 to rc 4). A FIRST
    seat could sit there too; since #987 every rung is settled into the band
    at the grade's own bounds first (`_band_settle`), and this remains the
    check on what the settle could not move.
    The setback is charged only once the occupancy reading is <= EPS, which
    an inward move is exactly what produces, so any such pose on an entry
    that carries one is refused -- conservatively, since the grade then also
    needs the body too far in.
    """
    from .legality import EPS
    amount, basis, legacy = _band_reading(state, part, edge, x, y)
    detail = {'overhang_after_mm': round(amount, 4), 'band_min_mm': lo,
              'basis': basis}
    hi = (entry.get('overhang_mm') or {}).get('max')
    if amount < lo - EPS:
        return 'band_min', detail
    if hi is not None and amount > float(hi) + EPS:
        return 'band_max', detail
    if _carries_setback(entry) and legacy <= EPS:
        return 'setback', detail
    return None, detail


def _outside_its_along_edge_claim(state, part, entry: Dict, edge: str,
                                  x: float, y: float) -> bool:
    """Would `rule_edge_connector`'s along-edge conjunct flag this pose?

    Asked by CALLING the rule's own conjunct (`floorplan._grade_along_edge`)
    on the pose `apply_move` writes, with the edge span the seat ladder
    already uses (`_declared_edge_span`'s outline stand-in), so the two
    cannot disagree about the window. The ladder clamps its rungs to the
    declared window, but a rung ON a window end is written to 3 decimals and
    can land half a micron outside it, where the grade's EPS is 1 nm --
    measured: a one-footprint board whose seed went rc 0 -> 4, and 70 of
    2880 blocker positions in #983. `_window_nudge` asks this of every rung
    and steps the ones it flags inside the window; what is still flagged at
    the written pose is named by `_window_miss_note`."""
    if entry.get('center_on_edge') is None and entry.get('along_edge_band') is None:
        return False
    from types import SimpleNamespace
    from placement import floorplan as _fp
    bounds = state.pcb_data.board_info.board_bounds
    if not bounds:
        return False
    outline = {'simple_rectangle': not getattr(state.edge_gate, 'rings', None),
               'cutouts': 0, 'edge_segments': 0}
    ctx = SimpleNamespace(gate=state.edge_gate, outline=outline,
                          outline_bounds=tuple(round(v, 6) for v in bounds),
                          edge_seating=[], abstain=lambda key, why: None)
    probe = SimpleNamespace(rect=part.grade_rect(round(x, 3), round(y, 3), part.rot))
    return any(True for _ in _fp._grade_along_edge(ctx, dict(entry, edge=edge),
                                                   part.ref, probe, 'error'))


#: #987: the furthest `_band_settle` moves a seat along its edge normal: the
#: seat's own 0.02 mm band tolerance plus the half micron the write rounds
#: away, rounded up to the 0.001 mm grid (0.021), plus one grid step. It can
#: only close the gap that tolerance opened.
_BAND_SETTLE_CAP_MM = 0.022


def _band_settle(state, part, entry: Dict, edge: str, lo: float, x: float, y: float,
                 seats=None) -> Tuple[float, float]:
    """#987: `(x, y)`, unless the pose it WRITES reads outside the declared
    overhang band; then that pose moved along the edge normal until it reads
    inside, by whole 0.001 mm grid steps and one step more.

    The seat accepts a band reading within 0.02 mm of the band
    (`_body_band_correct`, `edge_seat_ok`) and stops the overhang walk within
    0.02 mm of its target (`_edge_correct`); the grade accepts one only within
    EPS. So a seat could be written up to 0.02 mm outside its band -- a drawn
    body past its courtyard, a target ON a band end (`max(lo, 0.5)` with no
    `max`), a gate margin under 0.02 mm. The reading here is the grade's own
    (`_band_reading`) at the written pose, so a rung it accepts is returned
    BIT-IDENTICAL. The along-edge coordinate is not touched.

    A PREFERENCE, and a narrow one:
      * only a rung that already seats is moved (`seats(x, y)` is not None),
        so a correction never makes a seat of a rung the seat refused;
      * the move is at most `_BAND_SETTLE_CAP_MM`;
      * the moved pose must seat, and `_no_worse` must accept it: no pad
        further inside the #975 floor (measured, moving a body 0.01 mm out to
        meet a 0.3 minimum put two pads 0.006 mm inside it), no courtyard
        overlap the grade would newly report, pair by pair, and none doubled
        (measured: 0.0072 mm2 bought with a locked part), and with a grader
        no intent-grade error it did not have;
      * an INWARD move that takes the occupancy reading to <= EPS is refused
        on an entry that carries a setback, because that is exactly when the
        grade starts charging it (measured: a courtyard-only receptacle on a
        {0, 0} band traded "past the declared maximum" for "seated 0.55mm
        from the nearest edge with no overhang");
      * the moved pose must still face its edge if the raw one did.
    Anything that fails, or raises, leaves the raw pose, which the grade then
    reports as it always did. It is not asked `_grade_worse`, whose
    unconditional off-board reading would refuse every outward move; the
    overlap and grade halves are asked through `_no_worse` instead.
    """
    from .legality import EPS
    try:
        hi = (entry.get('overhang_mm') or {}).get('max')
        top = None if hi is None else float(hi) + EPS

        def inside(a):
            return a >= lo - EPS and (top is None or a <= top)
        amount, _basis, _legacy = _band_reading(state, part, edge, x, y)
        if inside(amount):
            return x, y
        raw = seats(x, y) if seats is not None else (0, 0.0, {}, None, None)
        if raw is None:
            return x, y
        # Outward (more overhang) when short of the minimum, inward past the max.
        sign = 1.0 if amount > lo else -1.0
        ix, iy = _INWARD[edge]
        rx, ry = round(x, 3), round(y, 3)
        moved, reading = 0.0, amount
        for _ in range(3):
            gap = (reading - float(hi)) if sign > 0 else (lo - reading)
            moved += (math.ceil(gap / _WINDOW_GUARD_MM - 1e-9) + 1) * _WINDOW_GUARD_MM
            if moved > _BAND_SETTLE_CAP_MM + 1e-9:
                return x, y
            nx = round(rx + sign * ix * moved, 3) if ix else x
            ny = round(ry + sign * iy * moved, 3) if iy else y
            reading, _basis, legacy = _band_reading(state, part, edge, nx, ny)
            if inside(reading):
                break
        else:
            return x, y
        if sign > 0 and _carries_setback(entry) and legacy <= EPS:
            return x, y
        if (_faces_its_edge(state, part, entry, edge, x, y)
                and not _faces_its_edge(state, part, entry, edge, nx, ny)):
            return x, y
        if seats is not None:
            new = seats(nx, ny)
            if new is None or (raw is not None and not _no_worse(new, raw)):
                return x, y
        return nx, ny
    except Exception:                                   # noqa: BLE001
        return x, y


#: The courtyard overlap a pair must reach to count as one: `overlap_area`
#: is reported to 4 decimals, so below this the grade prints 0.0000 and a
#: pair the correction pushes past it is NEW overlap in the grade's own words.
_OVERLAP_REPORTED_MM2 = 5e-5


def _overlap_at(state, part, x: float, y: float, others) -> Dict[str, float]:
    """{neighbour: courtyard overlap (mm^2)} of `part` at the pose `apply_move`
    WRITES, for every part in `others` it touches -- side-aware, in the
    currency of `legality_metrics`' `overlap_area`
    (`legality.pair_overlap_area`). PER PAIR: a sum lets an overlap one
    neighbour already has hide a new one with another."""
    from .legality import pair_overlap_area
    if getattr(state, 'courtyards_ignored', False):
        return {}      # #1104: the project waives courtyard overlap
    px, py = round(x, 3), round(y, 3)
    rect, tht = part.rect(px, py, part.rot), part.tht_rect(px, py, part.rot)
    out: Dict[str, float] = {}
    for ref in others:
        q = state.parts.get(ref)
        if q is None or q is part:
            continue
        area = pair_overlap_area(part.sides, part.side, rect, tht,
                                 q.sides, q.side, q.rect(), q.tht_rect())
        if area > 0.0:
            out[ref] = area
    return out


def _seat_reading(state, part, ref: str, x: float, y: float, rot: float,
                  others, grade=None, exclude=()):
    """What a #983/#987 correction compares, at the pose `apply_move` WRITES:
    `(pads short of the floor, worst shortfall, {neighbour: courtyard
    overlap} over `others`, intent-grade errors, interior-contour split)`. The
    last two are None without a grader."""
    floor = _floor_key(_floor_at(state, ref, x, y, rot))
    overlap = _overlap_at(state, part, x, y, others)
    if grade is None:
        return floor + (overlap, None, None)
    pose = {ref: (round(x, 3), round(y, 3), rot)}
    return floor + (overlap, grade.violations(exclude=exclude, poses=pose),
                    grade.interior_split(pose))


def _no_worse(new, raw) -> bool:
    """May a correction trade `raw` for `new` (`_seat_reading` tuples)?

    Only if it costs NOTHING the seat already had, each count on its own:
      * no more pads short of the #975 floor, and the worst no shorter;
      * pair by pair, no courtyard overlap the grade would newly report: a
        pair under `_OVERLAP_REPORTED_MM2` (the resolution `overlap_area` is
        printed at) may not cross it, nor grow past EPS of float slack; one
        the grade already reports may deepen, but never to double (measured
        on #983's whole lattice through `repair_placement`: of the 655 seats
        already overlapping the blocker, 16 deepen, by 0.0002-0.0020 mm2 on
        0.014-0.35 mm2, at most 2.7 %) -- refusing that would keep the
        grade ERROR the correction exists to remove. The comparison is
        against the rung as the ladder found it, so a settle and a step on
        one rung together still may not double it;
      * with a grader, no intent-grade error the raw pose does not have
        (`floorplan.grade_delta`, as #975's `_grade_worse`) -- which also
        refuses an existing overlap deepened past a declared budget -- and
        the two poses must read the board's interior contours alike, or they
        are not comparable at all.
    """
    from .legality import EPS
    n_pads, n_worst, n_ov, n_err, n_split = new
    r_pads, r_worst, r_ov, r_err, r_split = raw
    if n_pads > r_pads or n_worst > r_worst + EPS:
        return False
    for ref, grown in n_ov.items():
        was = r_ov.get(ref, 0.0)
        if was < _OVERLAP_REPORTED_MM2 <= grown:
            return False
        if grown <= was + EPS:
            continue
        if was < _OVERLAP_REPORTED_MM2 or grown >= 2.0 * was:
            return False
    if n_err is not None and r_err is not None:
        from placement import floorplan as _fp
        if n_split != r_split or list(_fp.grade_delta(r_err, n_err)):
            return False
    return True


def _floor_key(floor) -> Tuple[int, float]:
    """(pads short of the floor, worst shortfall): smaller is better, and a
    reading that could not be taken counts as clear -- the seat then behaves
    as it always did."""
    if floor is None or not floor.short:
        return (0, 0.0)
    return (len(floor.short), floor.short[0][0])


def _window_nudge(state, part, entry: Dict, edge: str, x: float, y: float,
                  seats=None, origin=None) -> Tuple[float, float]:
    """#983: `(x, y)`, unless the pose it WRITES is outside the declared
    along-edge window; then that pose one 0.001 mm grid step along the edge,
    into the window.

    A rung clamped to a window END puts the courtyard centre exactly on it,
    and `round(x, 3)` then lands it up to half a micron outside, where the
    grade's EPS is 1 nm. The window is asked through the grade's own conjunct
    at the written pose, so a rung the grade accepts is returned
    BIT-IDENTICAL -- including a window-end rung that already lands on the
    grid. Only the along-edge coordinate moves, from its written value by
    exactly one grid step (`_WINDOW_GUARD_MM`), so it is the same micron at
    every rotation and on a notched outline, and the normal coordinate the
    overhang walk converged on is untouched: re-deriving it would move the
    part by a rounding step AWAY from the edge floor as often as towards it
    (measured: a pad already past the outline went 0.132 -> 0.133 mm).

    A PREFERENCE, so it may not cost a seat or anything else the seat had:
    `seats(x, y)`, when given, is None for a pose that does not seat (off
    the board, crowding a neighbour) and its `_seat_reading` otherwise. Only
    a rung that already seats is moved, and the nudged pose is taken only
    when it seats and `_no_worse` accepts it (floor, new overlap, grade). A window no
    grid pose meets (one step is not enough: `center_on_edge` with
    `tolerance_mm: 0` and a courtyard centre off the grid) keeps the raw pose
    too; `_window_miss_note` names what is left. So does anything raised.
    """
    try:
        return _window_step(state, part, entry, edge, x, y, seats, origin)
    except Exception:                                   # noqa: BLE001
        # A preference may not cost a seat: anything raised while asking
        # leaves the rung as the ladder had it, which the grade then reports.
        return x, y


def _window_step(state, part, entry, edge, x, y, seats, origin=None):
    """`_window_nudge`'s body, outside its catch-all. `origin`, when given,
    is the rung before `_band_settle` moved it: what the step is judged
    against, so the two corrections together cost no more than one may."""
    if not _outside_its_along_edge_claim(state, part, entry, edge, x, y):
        return x, y
    e_lo, e_hi, _ = _declared_edge_span(state, state.board, edge)
    win = _declared_frac_window(entry, e_hi - e_lo)
    if win is None:
        return x, y
    ax = _axis_of(edge)
    r = part.grade_rect(round(x, 3), round(y, 3), part.rot)
    centre = (r[ax] + r[ax + 2]) / 2.0
    mid = e_lo + (win[0] + win[1]) / 2.0 * (e_hi - e_lo)
    step = _WINDOW_GUARD_MM if mid > centre else -_WINDOW_GUARD_MM
    if ax == 0:
        nx, ny = round(round(x, 3) + step, 3), y
    else:
        nx, ny = x, round(round(y, 3) + step, 3)
    if _outside_its_along_edge_claim(state, part, entry, edge, nx, ny):
        return x, y
    if seats is not None:
        raw = seats(*(origin or (x, y)))
        new = seats(nx, ny)
        if raw is None or new is None or not _no_worse(new, raw):
            return x, y
    return nx, ny


def _window_miss_note(state, part, entry: Dict, edge: str, prefix: str):
    """The NOTE for a pose WRITTEN outside its declared along-edge window, or
    None. What is left after `_window_nudge` is a window the 0.001 mm grid a
    pose is written on cannot meet (`center_on_edge` with `tolerance_mm: 0`
    and a courtyard centre off that grid), or a rung whose step was refused
    -- it would not seat, or `_no_worse` refused it, or asking raised -- or
    one that never seated to be stepped (a crowding fallback); either way
    the grade reports it, and this says so at the seat rather than leaving
    the reader to find it in the grade."""
    if not _outside_its_along_edge_claim(state, part, entry, edge, part.x, part.y):
        return None
    return (f"{prefix}{part.ref}: written outside its declared along-edge "
            f"window on the {edge} edge, and the grade reports it -- the "
            f"window is narrower than the 0.001mm grid a pose is written on, "
            f"or one grid step inside it would have cost the seat something")


def _grade_accepts(state, part, entry: Dict, edge: str, lo: float,
                   x: float, y: float) -> bool:
    """The grade's own conjuncts a preferred pose must pass beyond the seat
    predicate (which already holds the pad copper past the outline): the band
    and setback at the grade's bounds, the nearest edge, and the declared
    along-edge window."""
    return (_grade_band_refuses(state, part, entry, edge, lo, x, y)[0] is None
            and _faces_its_edge(state, part, entry, edge, x, y)
            and not _outside_its_along_edge_claim(state, part, entry, edge, x, y))


#: The legality readings a pose comparison may not let rise (`_grade_worse`,
#: the decap rung): the quench's rect overlap, the exact one the `legality`
#: budget is graded on since #1162, and the off-board pair.
LEGALITY_COMPARE_KEYS = ('overlap_area', 'overlap_area_exact', 'oob_amount',
                         'oob_count')


def _grade_worse(grade, ref: str, rot: float, first, seat, exclude, memo):
    """What the intent grade adds when `ref` sits at `seat` instead of `first`
    -- the seat the ladder always chose -- or () when nothing.

    Asked of every pose the floor preference would take over that seat, after
    its own conjuncts pass, because those conjuncts are a LIST and a list
    misses a rule: a move that clears the floor can raise the courtyard
    overlap budget, leave the part's own zone, or break a proximity claim, and
    place_seed then exits 4 on a board the first seat would have passed. The
    grade is `floorplan.PoseGrader`: the RULES `grade` runs, on this search's
    own board at the two poses, with `exclude` (the pile, whose coordinates
    mean nothing yet) left out of both so the difference is this part's. A
    pose the grade cannot be asked about is not taken. `memo` keeps the first
    seat's grade for one rotation.

    It is asked PER SEAT, at the moment of that seat, so an error only a later
    part's seat will produce -- two connectors sharing an edge, where this
    move leaves room the next one then wants -- is outside it by construction.
    `place_seed`'s own end-of-run grade still reports that one, and still sets
    the exit code from it.

    Beside the grade, the placement's own legality numbers (courtyard overlap
    and off-board) are compared UNCONDITIONALLY through `legality_at`, because
    the `legality` rule is skipped when the intent declares no
    `legality_budget` -- and `emit_intent` withholds that budget on exactly the
    boards where the risk is real. Without that, a move could buy courtyard
    interpenetration with a locked part and raise nothing: measured on a
    fixture, 0.18 mm2, which no pad or hole predicate can see.

    The two grades are comparable only while they describe the same board, and
    they stop doing so when the poses fall on opposite sides of the parser's
    two-pad-centre threshold for an interior Edge.Cuts contour: the grader
    holds the classification its state was built with, while a grade of the
    board that would be WRITTEN re-derives it (`PoseGrader.interior_split`).
    That is not a delta of zero, it is a delta of nothing, so it is reported
    unavailable and the seat is kept. Measured by a verifier on a synthetic
    board: a sub-millimetre move of the declared connector (0.5 mm on each
    axis) took the written board's `oob_count` 2 -> 0 while the masked reading
    held at 2."""
    if grade is None:
        return ()
    from placement import floorplan as _fp

    def at(pose):
        return (round(pose[0], 3), round(pose[1], 3), rot)
    try:
        if (grade.interior_split({ref: at(first)})
                != grade.interior_split({ref: at(seat)})):
            return ({'unavailable': "the two poses do not classify the board's "
                                    "interior contours alike"},)
        if memo.get('pose') != at(first):
            memo['errors'] = grade.violations(exclude=exclude, poses={ref: at(first)})
            memo['legality'] = grade.legality_at(exclude=exclude,
                                                 poses={ref: at(first)})
            memo['pose'] = at(first)
        after = grade.violations(exclude=exclude, poses={ref: at(seat)})
        rows = list(_fp.grade_delta(memo['errors'], after))
        # The grade alone is not enough: `legality` is skipped when the intent
        # declares no `legality_budget`, and `emit_intent` withholds
        # `overlap_area` on exactly the boards where courtyard interpenetration
        # is the live risk, so a move could buy overlap with a LOCKED part and
        # raise no error at all (measured on a fixture: 0.18mm2, undisclosed).
        # These numbers are read whatever the intent says.
        from .legality import EPS as _eps
        moved = grade.legality_at(exclude=exclude, poses={ref: at(seat)})
        # Whatever the armed `legality` rule already said, said once: a budget
        # it reported growing is the same finding as the reading below.
        said = {r.get('budget') for r in rows if r.get('rule') == 'legality'}
        # #1162: the budget is graded on `overlap_area_exact` now, so a rule
        # row for 'overlap_area' already said what that reading would.
        if 'overlap_area' in said:
            said.add('overlap_area_exact')
        for key in LEGALITY_COMPARE_KEYS:
            if key in said:
                continue
            was, now = memo['legality'].get(key), moved.get(key)
            if not (isinstance(was, (int, float))
                    and isinstance(now, (int, float))):
                continue
            if now > was + (0 if isinstance(now, int) else _eps):
                rows.append({'rule': 'legality', 'budget': key,
                             'before': round(float(was), 4),
                             'after': round(float(now), 4)})
        return tuple(rows)
    except Exception as exc:                                   # noqa: BLE001
        return ({'unavailable': f'{type(exc).__name__}: {exc}'},)


def _floor_rung(state, part, entry: Dict, edge: str, lo: float, hi: float,
                x: float, y: float, crowds):
    """One ladder rung, already a legal conflict-free seat, asked about the
    floor. Returns `(seat, floor, why)`:

    * `seat` -- `(x, y)` when this rung, or this rung moved inward, clears the
      floor, else None;
    * `floor` -- the reading at the rung itself (None: no floor context);
    * `why` -- when `seat` is None, a dict saying why moving inward did not
      help, for the disclosure.

    The inward move is DERIVED, not searched. On a rectangular outline every
    copper gap to the seated edge grows by exactly the distance the part moves
    inward, so the largest shortfall on that edge is the distance to move.
    Nothing else about the pose changes, and the move is then re-checked: the
    band and setback at the grade's own bounds (`_grade_band_refuses`),
    `edge_seat_ok`, the neighbours, the floor again, the grade's nearest edge
    and along-edge window. The caller then asks the whole intent grade
    (`_grade_worse`), which is what a list of conjuncts cannot promise.
    """
    floor = _floor_at(state, part.ref, x, y, part.rot)
    if floor is None or not floor.short:
        return (x, y), floor, None
    sides = {s[2] for s in floor.short}
    if sides != {edge}:
        return None, floor, {'why': ('along_edge' if None not in sides
                                     else 'outline_sampled'),
                             'sides': sorted(str(s) for s in sides)}
    shift = max(s[0] for s in floor.short) + _FLOOR_SHIFT_GUARD_MM
    ux, uy = _INWARD[edge]
    sx, sy = round(x + ux * shift, 3), round(y + uy * shift, 3)
    reason, detail = _grade_band_refuses(state, part, entry, edge, lo, sx, sy)
    why = dict({'shift_mm': round(shift, 4)}, **detail)
    if reason is not None:
        return None, floor, dict(why, why=reason)
    refused: List[str] = []
    if not edge_seat_ok(state, part, sx, sy, edge, lo, hi, reasons=refused):
        return None, floor, dict(why, why='refused',
                                 refused_by=sorted(set(refused))[:3])
    if crowds(sx, sy):
        return None, floor, dict(why, why='crowds')
    moved = _floor_at(state, part.ref, sx, sy, part.rot)
    if moved is None or moved.short:
        return None, floor, dict(why, why='still_short')
    if not _faces_its_edge(state, part, entry, edge, sx, sy):
        return None, floor, dict(why, why='nearest_edge')
    if _outside_its_along_edge_claim(state, part, entry, edge, sx, sy):
        return None, floor, dict(why, why='along_edge_window')
    return (sx, sy), floor, None


def _floor_record(ref: str, edge: str, kept: str, pose, floor: _Floor,
                  why: Optional[Dict]) -> Optional[Dict]:
    """The `edge_floor_fallback` record for a KEPT pose that is short on a
    measured pad, or None when it is not. Bounded: at most
    `_FLOOR_RECORD_PADS` pads by name, with the full counts beside them."""
    if floor is None or not floor.short:
        return None
    worst = floor.short[0]
    record = {
        'edge': edge, 'kept': kept,
        'pose': [round(pose[0], 3), round(pose[1], 3), pose[2]],
        'required_mm': floor.required,
        'shortfall_mm': worst[0], 'min_gap_mm': worst[3],
        'n_pads_short': len(floor.short),
        'pads': [{'pad_ref': f'{ref}.{num}', 'pad_index': index, 'side': side,
                  'gap_mm': gap, 'shortfall_mm': amount}
                 for amount, index, side, gap, num
                 in floor.short[:_FLOOR_RECORD_PADS]],
        'n_unmeasured_pads': len(floor.unmeasured),
        'unmeasured_pads': list(floor.unmeasured[:_FLOOR_RECORD_PADS]),
    }
    record.update(why or {'why': kept})
    return record


def _grade_delta_phrase(entry: Dict) -> str:
    """One `grade_delta` row in words."""
    if 'unavailable' in entry:
        return f"the grade could not be asked ({entry['unavailable']})"
    if 'budget' in entry:
        return (f"{entry['rule']} {entry['budget']} {entry['before']} -> "
                f"{entry['after']}")
    who = entry.get('ref') or entry.get('block') or 'board'
    return f"{entry['added']} more {entry['rule']} on {who}"


def _floor_note(prefix: str, ref: str, record: Dict) -> str:
    """One NOTE for a disclosed floor shortfall, shared by every seat path."""
    pad = record['pads'][0]
    gap = ('' if pad['gap_mm'] is None
           else f" is {pad['gap_mm']:.3f}mm from the {pad['side']} edge")
    why = record.get('why')
    because = {
        'band_min': (f"clearing it would take the overhang to "
                     f"{record.get('overhang_after_mm', 0.0):.3f}mm, under the "
                     f"declared minimum {record.get('band_min_mm', 0.0):g}mm"),
        'setback': ("clearing it would leave the part with no overhang, where "
                    "its seat setback is graded"),
        'along_edge': ("the short copper faces a side that moving the part "
                       "inward does not clear"),
        'outline_sampled': ("the outline is sampled, so no inward distance can "
                            "be derived"),
        'refused': ("the pose that clears it is refused by "
                    + ', '.join(record.get('refused_by') or ['the seat'])),
        'crowds': "the pose that clears it crowds a part already placed",
        'still_short': "moving the part inward did not clear it",
        'band_max': ("the pose that clears it would overhang past the declared "
                     "maximum"),
        'along_edge_window': ("the pose that clears it would sit outside the "
                              "declared along-edge window"),
        # Not "error(s) this seat does not have": the same channel carries a
        # budget this seat is already over and the move would grow -- an error
        # the seat DOES have -- and a grade that could not be asked at all.
        # Each row says which; the sentence must not overwrite them.
        'grade_delta': (("the pose that clears it could not be compared with "
                         "this seat: "
                         if any('unavailable' in d
                                for d in record.get('grade_delta') or [])
                         else "the pose that clears it does not pass the intent "
                              "grade beside this seat: ")
                        + '; '.join(_grade_delta_phrase(d)
                                    for d in record.get('grade_delta') or [])),
        'nearest_edge': ("the pose that clears it would read nearest another "
                         "edge than the declared one"),
        'crowding': ("no seat on this band clears the parts already placed, so "
                     "the floor was not searched"),
    }.get(why, str(why))
    return (f"{prefix}{ref}: its seat on the {record['edge']} edge leaves pad "
            f"copper inside the {record['required_mm']:g}mm board-edge floor -- "
            f"{pad['pad_ref']}{gap} ({record['shortfall_mm']:.3f}mm short); "
            f"{because}. Kept the seat rather than leave the connector "
            f"unseated, which leaves its nets unrouted (edge_floor_fallback)")


def _floor_records_at_final_pose(state, records: Dict[str, Dict]) -> Dict[str, Dict]:
    """The records whose part still sits at the pose each describes. A later
    stage (stage 3b's anchor rounds) can re-seat a part, and a record about a
    pose that was not written would describe copper that is not there."""
    return floor_records_at_poses(
        records, {ref: (p.x, p.y, p.rot) for ref, p in state.parts.items()})


def floor_records_at_poses(records: Dict[str, Dict], poses) -> Dict[str, Dict]:
    """The `edge_floor_fallback` records whose ref sits, in `poses`
    ({ref: (x, y, rot)}), at the pose the record describes. `place_seed` asks
    it of the WRITTEN board, after passes the seeder never saw."""
    out = {}
    for ref, record in sorted((records or {}).items()):
        pose = poses.get(ref)
        if pose is None:
            continue
        x, y, rot = record['pose']
        turn = ((pose[2] or 0.0) - rot) % 360.0
        # 1e-3 degrees, not float equality: the writer prints an angle with
        # `%g`, so a part seated at 0.1234567 degrees is WRITTEN at 0.123457.
        if ((round(pose[0], 3), round(pose[1], 3)) == (x, y)
                and min(turn, 360.0 - turn) < 1e-3):
            out[ref] = record
    return out


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
    lx0, ly0, lx1, ly1 = part.grade_rect(0.0, 0.0, part.rot)
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


def _declared_edge_span(state, bounds, edge: str):
    """(lo, hi, basis) for the edge a DECLARED fraction is a fraction OF.

    The grade resolves this with `floorplan.edge_span`, which prefers the
    outline RING because the bounding box is wrong on a notched board -- on
    `interf_u_unrouted_placed` the bbox south side spans 115.57mm where the
    board's real south edge spans 81.28mm. If the seat search kept using the
    bbox the two would target centres 5.715mm apart on that board, and the
    seeder would deterministically place what the grade then flags. So both
    sides call the SAME function; a grader with its own idea of the geometry
    grades the reimplementation.

    Falls back to the bounding box when the outline cannot be resolved, which
    is what the ladder used before any of this and is correct wherever the
    grade abstains (there is then no declared verdict to disagree with).
    """
    from placement import floorplan as _fp   # NOT inside a try: an
    # ImportError here would make every board answer `bbox` on every edge,
    # silently, which is the "guard that never fires" shape
    # `_already_on_its_edge` below was written to stop repeating.
    x0, y0, x1, y1 = bounds
    bbox = ((x0, x1) if edge in ('north', 'south') else (y0, y1))
    outline = {'simple_rectangle': True, 'cutouts': 0, 'edge_segments': 0}
    gate = state.edge_gate
    if getattr(gate, 'rings', None):
        # Rings exist, so `edge_span` takes its ring branch and never reads
        # these flags. When there are none it takes the bbox branch, which is
        # correct here for the same reason it is correct there: the parser
        # publishes no ring for a plain rectangle, and the bbox IS the
        # outline. The seeder can therefore answer `bbox` only where the
        # GRADE would abstain -- never the reverse, since a ring run can
        # never exceed its own bbox side.
        outline = {'simple_rectangle': False, 'cutouts': 0,
                   'edge_segments': 0}
    lo, hi, basis = _fp.edge_span(gate, bounds, edge, outline)
    if lo is None:
        return bbox[0], bbox[1], 'bbox'
    return lo, hi, basis


def _axis_of(edge: str) -> int:
    return 0 if edge in ('north', 'south') else 1


def _centre_offset_mm(part, edge: str) -> float:
    """(rect centre - origin) along `edge`, in MILLIMETRES.

    Turns with the part, so it must be read at the rotation in use.
    """
    lx0, ly0, lx1, ly1 = part.grade_rect(0.0, 0.0, part.rot)
    a, b = ((lx0, lx1) if _axis_of(edge) == 0 else (ly0, ly1))
    return (a + b) / 2.0


class _AtRotation:
    """`part` as it would stand at `rot`, for READING its geometry only.

    `_edge_frac_bounds`, `declared_to_ladder_frac` and
    `ladder_to_declared_frac` read exactly two things off a part: `rot`, and
    `rect(x, y, rot)`. This answers both for another angle without turning the
    part in the state, so a caller that measures and then skips the part has
    not moved it. (An off-lattice angle's rotated box is cached by
    `_materialise_rotation` first, as the #893 block would cache it anyway.)
    """
    __slots__ = ('_part', 'rot')

    def __init__(self, part, rot):
        self._part, self.rot = part, rot

    def rect(self, x, y, rot):
        return self._part.rect(x, y, rot)

    def grade_rect(self, x, y, rot):
        # #1182: the edge-claim readers ask the GRADE ladder.
        return self._part.grade_rect(x, y, rot)


def _stage1_geometry_rot(part, claim, fits=None):
    """#988: the rotation stage 1 measures an edge connector's geometry at.

    Stage 1 applies a DECLARED rotation (#893) only after it has converted the
    declared window and clamped by the part's extents -- all of which turn
    with the part. Measured at the input rotation, splitflap_driver's J5
    (input 180, `center_on_edge` 1.0 mm) was written 10.00 mm off centre when
    declared at 0, 5.65 mm at 90 and 4.60 mm at 270, and a part that fits the
    edge only at its declared angle was refused as "wider than the edge". So:
    the declared angle when one is declared, else the part's own.

    #1120: a candidate SET is applied too. It used not to be, and no later
    stage re-seats a connector stage 1 seats, so J5 declared `[0, 90]` was
    written at its input 180, ungraded. The part's own angle when it is a
    member that `fits` (stage 1 does not turn a part already at a member
    that fits; one at a member that does not is turned to one that does),
    else the first member in the AUTHOR's order that fits -- `fits(rot)`
    is stage 1's own two pre-turn refusals (`_stage1_fits`). With no member
    that fits, the first member: stage 1 then refuses the part at a declared
    angle and leaves it unturned, and the later stages seat it at a member or
    report it in `rotation_unseated`. `fits=None` admits every member.
    """
    if claim is not None and claim[0] is not None:
        return claim[0] % 360.0
    if claim is not None and claim[1]:
        set1120 = [c % 360.0 for c in claim[1]]
        fit1120 = [r for r in set1120 if fits is None or fits(r)]
        # quench._same_angle's tolerance, so a part `poses` reads as AT a
        # member is not turned here (#1120 verifier: 1e-9 vs 1e-6).
        if any(abs((r - part.rot + 180.0) % 360.0 - 180.0) < 1e-6
               for r in fit1120):
            return part.rot
        return fit1120[0] if fit1120 else set1120[0]
    return part.rot


def _stage1_walk_member(part, claim, tried, fits=None):
    """#1125: the next member of a candidate set stage 1 tries after the
    angles in `tried` -- `_stage1_geometry_rot`'s own choice with the tried
    members taken out, so the walk follows the order that choice does (the
    part's own angle, then the author's), and anything that replaces the
    choice replaces the walk with it. None when no untried member fits."""
    def _new(r):
        return not any(abs((r - t + 180.0) % 360.0 - 180.0) < 1e-6
                       for t in tried)

    def _left(r):
        return _new(r) and (fits is None or fits(r))
    nxt = _stage1_geometry_rot(part, claim, fits=_left)
    return nxt if _left(nxt) else None


def _stage1_fits(state, part, entry, bounds, edge, rot) -> bool:
    """Would stage 1 seat `part` on `edge` at `rot`? Its two refusals before
    any turn, repeated: the part is wider than the edge, or the declared
    along-edge window does not intersect the legal one (#1120).

    A second copy of two predicates is a second chance for them to drift, and
    the originals are anchored lines, so they cannot be shared; test_983's
    C10 pins that this answer and stage 1's own skip notes agree.
    """
    if rot != part.rot:
        rot = _materialise_rotation(part, rot)
    geo1120 = _AtRotation(part, rot)
    lo1120, hi1120 = _edge_frac_bounds(geo1120, bounds, edge)
    if lo1120 > hi1120:
        return False
    e_lo, e_hi, _ = _declared_edge_span(state, bounds, edge)
    win1120 = _declared_frac_window(entry, e_hi - e_lo)
    if win1120 is None:
        return True
    w_lo = declared_to_ladder_frac(geo1120, bounds, edge, e_lo, e_hi,
                                   win1120[0])
    w_hi = declared_to_ladder_frac(geo1120, bounds, edge, e_lo, e_hi,
                                   win1120[1])
    return max(lo1120, w_lo) <= min(hi1120, w_hi)


def declared_to_ladder_frac(part, bounds, edge, e_lo, e_hi, declared):
    """A DECLARED fraction -> the fraction the seat ladder works in.

    TWO conversions, and both are needed or the seat lands somewhere the
    grade did not ask for:

      * the declaration is about the part's courtyard CENTRE (what
        `rule_edge_connector` measures) and `_edge_pose` positions its
        ORIGIN -- 2.54mm apart on splitflap_driver's J17;
      * the declaration is a fraction of the EDGE's span, which on a notched
        board is not the bounding box -- 81.28mm against 115.57mm on
        `interf_u_unrouted_placed` -- while `_edge_pose` interpolates the
        bounding box.

    Everything meets in millimetres along the edge axis, which is the only
    currency both sides can state without ambiguity.
    """
    ax = _axis_of(edge)
    bb_lo, bb_hi = ((bounds[0], bounds[2]) if ax == 0
                    else (bounds[1], bounds[3]))
    centre_pos = e_lo + declared * (e_hi - e_lo)
    origin_pos = centre_pos - _centre_offset_mm(part, edge)
    return (origin_pos - bb_lo) / max(1e-9, bb_hi - bb_lo)


def ladder_to_declared_frac(part, bounds, edge, e_lo, e_hi, frac):
    """The inverse, so a refusal can report the legal window in the SAME
    currency as the declaration it is refusing. Two numbers in one sentence
    that mean different things is how the first version of this message read.
    """
    ax = _axis_of(edge)
    bb_lo, bb_hi = ((bounds[0], bounds[2]) if ax == 0
                    else (bounds[1], bounds[3]))
    origin_pos = bb_lo + frac * (bb_hi - bb_lo)
    centre_pos = origin_pos + _centre_offset_mm(part, edge)
    return (centre_pos - e_lo) / max(1e-9, e_hi - e_lo)


def _already_on_its_edge(state, part) -> bool:
    """Is this part already crossing the boundary it was declared on?

    Then its ORIENTATION is right and its refusal is a band, keep-out or
    neighbour problem -- turning it is the wrong answer.

    MEASURED, and the measurement is J17 on splitflap_driver rather than the
    three parts an earlier version of this docstring named. With a declared
    `along_edge_band` of 0.10-0.20, J17 at its home pose (overhanging its
    north edge by 0.40mm) is refused at rot 0 WITH this guard and rotates to
    90 without it -- so the guard is what keeps it upright, and
    `tests/test_706_seat_edge_target.py` pins exactly that counterfactual.

    THE CLAIM HERE HAS BEEN WRONG TWICE, and both are worth recording.

    First it said a bare gate would turn ulx3s J1/J2 and sonde_u J1 -- three
    full board-width connectors overhanging 11.99, 11.99 and 26.55mm. The
    overhangs are real, the consequence was not: those three returned at the
    `f_lo > f_hi` refusal before this guard was consulted, and I had read a
    True from calling this function DIRECTLY as if it had been reached
    through `_seat_edge`.

    Then the correction itself went stale in the commit that wrote it. Moving
    the rotation-dependent geometry into `_geometry` moved that refusal too,
    so it no longer returns from `_seat_edge` -- the guard IS reached for all
    three now (measured: `guard_calls=[(J1, True), (J2, True), (J1, True)]`),
    and it is what keeps them upright after all. The conclusion held while
    the mechanism was wrong in both directions, which is exactly why the test
    beside this asserts the REASON and not just the outcome.

    A real consequence of that move, stated because it is a contract change:
    both hard refusals now fall THROUGH to the rotation loop instead of
    returning. A declared part too wide for its edge at one rotation, and not
    already overhanging, can now be turned. That is what #706's rotation half
    is for, and it is corpus-inert -- measured, all six too-wide (part, edge)
    pairs across splitflap/ulx3s/sonde_u/esp_prog/interf_u are already on
    their edge, so none rotates.

    NO `try/except` here, and that is the point. The first version wrapped the
    measurement in a bare `except Exception: return False`, and `seeder` has
    no module-level `legality` binding -- so `legality.EPS` raised NameError,
    the except swallowed it, and the guard answered "not on its edge" for
    EVERY part on EVERY board. It read exactly like a guard and never fired
    once. A measurement that cannot be taken must raise, not return the
    permissive answer.
    """
    return state.edge_gate.rect_outside_amount(part.grade_rect()) > _ON_EDGE_EPS_MM


def _seat_edge(state, ref: str, entry: Dict, must_lock: Set[str],
               notes: List[str], target=None, exclude=None,
               rotations=None, disclose=None, grade=None) -> bool:
    """Seat a DECLARED edge part on its edge band, minimal-move (run-4 B-6).

    Repair could never do this: `_try_place._ok` demands full containment,
    and an edge seat overhangs by design -- so declared edge refs were
    exempt-only and a misplaced one was simply unrepairable here. Reuses the
    stage-1 geometry (`_edge_pose` + `_edge_correct`); the along-edge
    position starts at the part's CURRENT projection (minimal move) and
    slides outward until the seat is pad/hole-conflict-free against every
    other part. Board-only; the band comes from the intent.

    `rotations` is the DECLARED ladder (#893), or None. When it is given it
    REPLACES the part's incoming angle rather than backing it up: the seat is
    tried at each declared angle in the author's order and the part is refused
    by name if none of them seats. The part's own angle is a candidate only
    when the author declared it.

    #975: at each rotation the ladder PREFERS a seat whose pad copper clears
    the board-edge floor -- a rung, or a rung moved inward by the derived
    distance (`_floor_rung`) -- and otherwise keeps the first seat it would
    always have kept. A rotation is never changed to clear the floor, so which
    rotations seat is exactly what it was. A kept seat that is short of the
    floor gets a NOTE and, when `disclose` is a dict, an `edge_floor_fallback`
    record under `ref`."""
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
    # the grade measures; the ladder below positions its ORIGIN. The
    # conversion lives in `_geometry` rather than here because it TURNS WITH
    # THE PART -- computing it once, at the incoming rotation, is what made
    # the rotation branch validate one rectangle and apply another.
    # The EDGE's span, not the bounding box's. `_declared_frac_window` turns
    # `center_on_edge.tolerance_mm` into a fraction OF THE SPAN IT IS HANDED,
    # and `_geometry` then reads that fraction as a fraction of the edge --
    # so handing it the bbox silently shrinks the declared tolerance by the
    # ratio of the two. Measured on `interf_u_unrouted_placed`'s south edge
    # (bbox 115.570mm, real edge 81.280mm) with `tolerance_mm: 1.0`: the
    # window came out +/-0.7033mm, 30% tighter than declared, which makes the
    # "does not intersect the legal one" refusal fire on windows that DO
    # intersect and drops the connector to the later stages -- and those park
    # a connector in the board interior. Stage 1 was fixed and this was not;
    # both now measure the same quantity.
    _e_lo0, _e_hi0, _ = _declared_edge_span(state, state.board, edge)
    win = _declared_frac_window(entry, _e_hi0 - _e_lo0)

    def _geometry(rot, complain):
        """(cur, f_lo, f_hi, step) at rotation `rot`, or None.

        EVERY quantity here depends on the rotation, which is why it is a
        function of one rather than four values computed once. `part.rect`
        turns with the part, so its half-extent (`_edge_frac_bounds`), the
        offset from its origin to its courtyard centre (`_centre_offset_mm`,
        via `declared_to_ladder_frac`)
        and therefore the window intersection are all different at 90 degrees.

        `complain` is True only for the part's OWN rotation: a refusal note is
        about the pose the caller asked for, and the rotation loop would
        otherwise append the same sentence up to four times.
        """
        e_lo, e_hi, _ = _declared_edge_span(state, state.board, edge)

        def to_ladder(f):
            return declared_to_ladder_frac(part, state.board, edge,
                                           e_lo, e_hi, f)

        def to_declared(f):
            return ladder_to_declared_frac(part, state.board, edge,
                                           e_lo, e_hi, f)

        # Clamp by the part's OWN half-extent, not a bare [0.05, 0.95]. A part
        # wider than its edge has no legal fraction at all; say so rather than
        # sliding it off the end and reporting a seat.
        f_lo, f_hi = _edge_frac_bounds(part, state.board, edge)
        if f_lo > f_hi:
            if complain:
                notes.append(f"{ref}: at rotation {rot:g}deg it is wider "
                             f"than the {edge} edge ({f_lo:.2f} > {f_hi:.2f} "
                             f"of its span), so no along-edge position keeps "
                             f"it on the board")
            return None
        # A DECLARED window narrows the legal one. Without this the +/-0.4
        # ladder could find a seat OUTSIDE the declared band, which
        # `rule_edge_connector` would then flag -- the search accepting a pose
        # the grade refuses is the round-trip break `edge_seat_ok` and
        # `keepout_hit` both exist to prevent.
        if win is not None:
            w_lo, w_hi = to_ladder(win[0]), to_ladder(win[1])
            n_lo, n_hi = max(f_lo, w_lo), min(f_hi, w_hi)
            if n_lo > n_hi:
                if complain:
                    notes.append(
                        f"{ref}: the declared along-edge window "
                        f"[{win[0]:.3f}, {win[1]:.3f}] does not intersect the "
                        f"legal one [{to_declared(f_lo):.3f}, "
                        f"{to_declared(f_hi):.3f}] "
                        f"for this part on the {edge} edge -- "
                        f"widen the declaration, or the part is too wide for "
                        f"the position it is declared at")
                return None
            f_lo, f_hi = n_lo, n_hi
        if declared is not None:
            cur = to_ladder(declared)
        else:
            ax, ay = target if target is not None else (part.x, part.y)
            if edge in ('north', 'south'):
                cur = (ax - x0) / max(1e-9, (x1 - x0))
            else:
                cur = (ay - y0) / max(1e-9, (y1 - y0))
        cur = min(f_hi, max(f_lo, cur))
        # The ladder's steps are fractions of the WHOLE edge, which is the
        # right scale when it is sliding a part along a free edge and much too
        # coarse when it is searching inside a declared window. Measured: with
        # a declared band of 0.85-0.95 on splitflap_driver's 198.12mm north
        # edge, the +/-0.05 rungs are 9.9mm apart and clamp to the window's
        # two ends, so the ladder tries THREE distinct positions in the band
        # and misses the one legal seat between them -- the feature reported
        # "no seat" on every band of that board. Scaling the rungs to the
        # window searches it at the same relative resolution the ladder gives
        # a whole edge. 0.8 is the ladder's own full sweep (+/-0.4), so an
        # undeclared seat has scale exactly 1.0 and is unchanged.
        step = ((f_hi - f_lo) / 0.8) if win is not None else 1.0
        return cur, f_lo, f_hi, step

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

    def try_rot(rot):
        """The seat ladder at one rotation. (x, y, record) or None; `record` is
        the `edge_floor_fallback` record when the seat is short of the floor.

        #975: ONE walk. Every rung is tested as it always was, and the first
        conflict-free one is the seat the ladder always chose. It is kept if
        its copper clears the floor, or replaced by its inward move when that
        move passes every re-check (`_floor_rung`). Otherwise the walk goes on
        only when the shortfall is one a later rung can change
        (`_SLIDE_HELPS`), and takes a later rung, or its move, only when it
        clears the floor and passes the grade's edge conjuncts
        (`_grade_accepts`: band, setback, nearest edge, along-edge window).
        Every pose other than the first seat must then add no intent-grade
        error to that seat's (`_grade_worse`). Failing all that, the first
        seat is kept and disclosed. A rung that is not already a conflict-free
        seat is never asked, so a rotation that seated nowhere still seats
        nowhere.

        `part.rot` is SET for the duration, and that is the whole point rather
        than a shortcut. `_edge_pose`, `_edge_correct` and `edge_seat_ok` all
        read `part.rot` internally -- the first version of this threaded the
        trial rotation into `conflict_free` alone, so the overhang band, the
        convergence walk and the pads-on-board check were all measured on the
        rectangle the part had BEFORE the turn, and then the turn was applied.
        Measured on the fixture this file's own rotation test uses: the seat
        checked 0.50mm of overhang at rot 0 and delivered 1.92mm at rot 90 --
        0.92mm past the declared maximum, `edge_seat_ok` False at the pose
        actually written, and an along-edge fraction outside the declared band
        as well. Exactly the "search accepts what the grade refuses" break the
        window intersection exists to prevent.
        """
        saved = part.rot
        part.rot = rot
        try:
            geom = _geometry(rot, complain=(rot == saved))
            if geom is None:
                return None
            cur, f_lo, f_hi, step = geom

            def seats(sx, sy):
                # What `_band_settle` and `_window_nudge` compare: None when
                # the pose is not a seat, else how far short of the floor it
                # leaves the copper and how much courtyard it overlaps -- the
                # neighbours `conflict_free` asks about, pile left out.
                if not (edge_seat_ok(state, part, sx, sy, edge, lo, hi_eff)
                        and conflict_free(sx, sy, rot)):
                    return None
                return _seat_reading(state, part, ref, sx, sy, rot,
                                     [o for o in state.parts if o != ref and o not in ex],
                                     grade, ex - {ref})
            first = None
            graded = {}
            for df in (0.0, 0.05, -0.05, 0.1, -0.1, 0.15, -0.15,
                       0.2, -0.2, 0.3, -0.3, 0.4, -0.4):
                frac = min(f_hi, max(f_lo, cur + df * step))
                x, y = _edge_pose(part, state.board, edge, frac, overhang)
                x, y, converged = _edge_correct(state, ref, edge, x, y,
                                                overhang, band=(lo, hi_eff))
                if converged:
                    rung = (x, y)
                    x, y = _band_settle(state, part, entry, edge, lo, x, y, seats)
                    x, y = _window_nudge(state, part, entry, edge, x, y, seats, rung)
                if not converged or not on_board(x, y):
                    continue
                if not conflict_free(x, y, rot):
                    continue
                if first is not None and first[3] not in _SLIDE_HELPS:
                    break
                try:
                    seat, floor, why = _floor_rung(
                        state, part, entry, edge, lo, hi_eff, x, y,
                        lambda a, b: not conflict_free(a, b, rot))
                except Exception as exc:               # noqa: BLE001
                    # A PREFERENCE may not cost a seat. Anything raised while
                    # measuring the floor leaves this rung exactly as the
                    # ladder had it before #975 -- today's pose, kept -- rather
                    # than propagating out of `_seat_edge` and abandoning the
                    # part, which would be worse than the shortfall.
                    notes.append(f"{ref}: the board-edge copper floor could "
                                 f"not be measured at this seat "
                                 f"({type(exc).__name__}: {exc}); the seat is "
                                 f"unchanged")
                    seat, floor, why = None, None, None
                # The first seat, unmoved, is the one the ladder always chose,
                # and is taken without asking anything more.
                if seat is not None and first is None and seat == (x, y):
                    return x, y, None
                # Anything else is a pose the grade has not been measured on:
                # the grade's own edge conjuncts are asked of it (a move asked
                # them already), then the whole intent grade against the
                # first seat's.
                if seat is not None and (first is None or seat != (x, y)
                                         or _grade_accepts(state, part, entry,
                                                           edge, lo, x, y)):
                    worse = _grade_worse(grade, ref, rot,
                                         (x, y) if first is None else first[:2],
                                         seat, ex - {ref}, graded)
                    if not worse:
                        return seat[0], seat[1], None
                    if first is None:
                        why = dict(why or {}, why='grade_delta',
                                   n_grade_delta=len(worse),
                                   grade_delta=list(worse[:_FLOOR_ROWS]))
                if first is None:
                    first = (x, y, _floor_record(
                        ref, edge, 'conflict_free', (x, y, rot), floor, why),
                        (why or {}).get('why'))
            return first[:3] if first is not None else None
        finally:
            part.rot = saved

    def keep(seat, rot):
        """Apply a seat, disclosing a floor shortfall it carries."""
        state.apply_move(ref, round(seat[0], 3), round(seat[1], 3), rot)
        missed = _window_miss_note(state, part, entry, edge, '')
        if missed:
            notes.append(missed)
        if seat[2] is not None:
            notes.append(_floor_note('', ref, seat[2]))
            if disclose is not None:
                disclose[ref] = seat[2]

    _floor_context_note(state, notes)

    # #893, the REPAIR half, and it has to run before the minimal-move seat
    # below rather than after it. Wiring `rotations` into the #706 fallback
    # ladder alone left the declaration unreachable in the ordinary case: the
    # seat at `part.rot` succeeds, this function returns True, and the ladder
    # is never consulted. Measured on splitflap_driver with an angle declared
    # 90deg off the board's: 17 of 17 J-refs seated at the INPUT angle and the
    # claim was dropped in silence -- the fourth time #893's ladder reached
    # some seating sites and not others, and the one shape the `_try_place`
    # AST gate cannot see, because `_seat_edge` is not a `_try_place` site.
    #
    # The part's own angle is NOT a fallback here. `rotation` is a decision and
    # `rotation_candidates` a set; an angle outside either is a pose the author
    # said they did not want, so a part that seats at no declared angle is
    # refused and named, exactly as the seed path refuses it into
    # `rotation_unseated`. That also makes repair agree with `_try_place`,
    # which has replaced the ladder outright since #893 -- before this, a
    # declared NON-edge part was corrected by repair and a declared EDGE part
    # was not.
    #
    # The three #706 gates below do not apply and are deliberately skipped: all
    # three exist because "nothing in this tree knows which way a mating face
    # must point". A declared rotation is precisely that knowledge, stated by
    # the author, so the guards protecting an UNDECLARED orientation have
    # nothing left to protect.
    if rotations is not None:
        was_rot = part.rot
        ladder = []
        for _r in rotations:
            _r = _materialise_rotation(part, _r)
            if _r not in ladder:      # author order; see `_rotation_candidates`
                ladder.append(_r)
        for rot in ladder:
            seat = try_rot(rot)
            if seat is None:
                continue
            keep(seat, rot)
            if abs((rot - was_rot) % 360.0) > 1e-9:
                notes.append(
                    f"{ref}: seated on the declared {edge} edge at the "
                    f"DECLARED rotation {rot:g}deg; it arrived at "
                    f"{was_rot:g}deg. Unlike a rotation this tool chooses, "
                    f"this one is the author's claim -- fix the declaration, "
                    f"not the board, if it is wrong")
            return True
        if refused:
            notes.append(f"{ref}: every position on the declared {edge} edge "
                         f"band is refused by " + _edge_refusal_tail(refused))
        notes.append(
            f"{ref}: no seat exists on the declared {edge} edge at any "
            f"declared rotation ({', '.join(f'{r:g}' for r in ladder)}deg) "
            f"-- NOT turned to an undeclared angle. Revisit the declaration "
            f"or the edge band")
        return False

    seat = try_rot(part.rot)
    if seat is not None:
        keep(seat, part.rot)
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
        # #893 handles a DECLARED rotation above and returns, so this ladder
        # is reached only for a part whose angle nobody declared -- which is
        # what its three gates assume. The first fix put the declared ladder
        # HERE, where the early return above meant it almost never ran.
        for rot in ((was_rot + 90) % 360, (was_rot + 180) % 360,
                    (was_rot + 270) % 360):
            seat = try_rot(rot)
            if seat is None:
                continue
            keep(seat, rot)
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
                     f"is refused by " + _edge_refusal_tail(refused))
    return False


def _edge_refusal_tail(refused) -> str:
    """The reasons an edge band was refused, deduplicated, and the next move
    each kind has. A declared keep-out has an `allow` list; a board rule-area
    band (#1044) does not, so its advice differs -- and it is reported ONCE,
    at its deepest, rather than once per ladder rung's depth."""
    band = [r for r in refused if 'rule-area keep-out band' in r]
    other = sorted(set(r for r in refused if r not in band))
    parts = list(other)
    if band:
        depth = max(float(r.split('pad copper ')[1].split('mm')[0])
                    for r in band)
        parts.append(f"pad copper up to {depth:.3f}mm into a rule-area "
                     f"keep-out band")
    advice = []
    if any(r.startswith('keep-out ') for r in other):
        advice.append("move the keep-out, or add this ref to its `allow`")
    if band:
        advice.append("move the board's rule area, or the connector's "
                      "declared edge / band")
    return ', '.join(parts) + (" -- " + '; '.join(advice) if advice else '')


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


#: #1051: the most ROW poses (an anchor position at one rotation, axis and
#: clearance rung) one array's seat may try. A COUNT, never a clock (the
#: maintainer's rule: determinism over deadlines), and the only bound on the
#: row search's cost: each pose costs up to one `pose_ok` per member but
#: exits at the first member that fails, which near the target is almost
#: always the first. One ring ladder at one angle is ~27k anchors
#: (`seat_candidates`), so the cap lets a row walk well past the rings at its
#: first angle and axis while a row that fits nowhere stops at a known cost.
#: Hitting it is disclosed (`array_unseated[name].capped`).
ARRAY_SEAT_POSE_CAP = 20000

#: #1051: `pitch_mm: auto` is the widest member's courtyard extent along the
#: row plus the board clearance plus THIS, rounded up to 0.01mm. Measured,
#: not cosmetic: at exactly extent + clearance the siblings' courtyard gap is
#: the clearance to within float noise (0.19999... < 0.2), the with-siblings
#: re-check refuses every anchor at the full rung, and on glasgow 11 of 24
#: rows burned the whole pose cap that way.
ROW_PITCH_MARGIN_MM = 0.01


def _row_offsets(state, members: Sequence[str], rot: float, axis: str,
                 pitch: float) -> List[Tuple[float, float]]:
    """Origin offsets from the row's centre that put each member's COURTYARD
    centre on one line along `axis`, `pitch` apart, in `members` order.

    The courtyard centre, not the footprint origin, because that is what
    `arrays.formation` measures in the grade (`rule_array_formation` reads
    the grader's own rect): a part whose origin is off its courtyard centre
    would otherwise sit on the line by origin and off it by the grade."""
    n = len(members)
    out = []
    for i, m in enumerate(members):
        b = state.parts[m].rect(0.0, 0.0, rot)
        cx, cy = (b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0
        s = (i - (n - 1) / 2.0) * pitch
        out.append((s - cx, -cy) if axis == 'x' else (-cx, s - cy))
    return out


def _row_extent(state, members: Sequence[str], rot: float, axis: str) -> float:
    """The largest member courtyard extent ALONG the row at `rot`."""
    ext = 0.0
    for m in members:
        b = state.parts[m].rect(0.0, 0.0, rot)
        ext = max(ext, (b[2] - b[0]) if axis == 'x' else (b[3] - b[1]))
    return ext


def _seat_block(state, members: Sequence[str], rot_options: Sequence[float],
                pitch_spec, axes: Sequence[str], target: Tuple[float, float],
                exclude: Set[str], *, constraint=None, tol: float = 0.5,
                cap: int = ARRAY_SEAT_POSE_CAP,
                reverse_for=None) -> Dict[str, object]:
    """Seat `members` as ONE row (#1051): a common axis, `members` order, one
    rotation, one pitch. Applies the moves on success.

    For each rung of `seat_clearances`, BAND-major over the SAME offsets a
    single part walks (`seat_candidate_bands`: the 1mm ring, then the
    whole-board sweep, then the fine and grid-step rings), and within each
    band rotation-major: each rotation in `rot_options`, each axis in
    `axes`, the row's centre walks the band around `target`. (Why band-
    major rather than a single part's per-angle order: see the loop.) At each anchor the members are checked in order with
    `pose_ok` and the pose is abandoned at the first that fails (EARLY EXIT).
    Every member's check EXCLUDES its unplaced siblings (they are in
    `exclude`, as the pile always is), so no member vetoes another at its
    pile coordinate; the sibling-vs-sibling question is the cheap PITCH
    pre-check instead -- a pitch below the courtyard extent plus the rung's
    clearance cannot seat at any anchor and is refused once, not per pose --
    and, on a hit, every member is re-checked with its siblings SEATED
    before the row is kept (a hit that fails that is reverted and the walk
    goes on).

    `pitch_spec` 'auto' is the largest courtyard extent along the axis plus
    the state's (full) clearance plus `ROW_PITCH_MARGIN_MM`, rounded up to
    0.01mm, so the gaps clear the board clearance at the widest member; a
    number is used as declared. `reverse_for(axis, order)`
    may flip the order along an axis (the served pins' direction).
    `constraint`/`tol`: a zone the whole row must sit in, through
    `zone_gate`, per member.

    The pose count is capped at `cap` (anchors tried, over every rung,
    rotation and axis). Returns `{'ok', 'poses_tried', 'capped', 'rot',
    'axis', 'pitch_mm', 'anchor', 'order', 'clearance', 'reasons'}`.
    """
    full = state.clearance
    gates = {m: zone_gate(state.parts[m], constraint, tol)[0]
             for m in members}
    tried = 0
    reasons: List[str] = []
    out = {'ok': False, 'poses_tried': 0, 'capped': False, 'reasons': reasons}
    for rot in rot_options:
        for m in members:
            _materialise_rotation(state.parts[m], rot)
    # The (rotation, axis) combos, rotation-major, each with its order,
    # offsets and pitch -- or skipped once by the pitch pre-check.
    def _combos(clr):
        got = []
        for rot in rot_options:
            for axis in axes:
                ext = _row_extent(state, members, rot, axis)
                pitch = (math.ceil((ext + full + ROW_PITCH_MARGIN_MM)
                                   * 100.0 - 1e-6) / 100.0
                         if pitch_spec == 'auto' else float(pitch_spec))
                if pitch - ext < clr + 1e-6:
                    why = (f"pitch {pitch:g}mm is below the courtyard "
                           f"extent {ext:.3f}mm + clearance {clr:g} along "
                           f"{axis} at {rot:g}deg")
                    if why not in reasons:
                        reasons.append(why)
                    continue
                order = list(members)
                if reverse_for is not None and reverse_for(axis, order):
                    order.reverse()
                got.append((rot, axis, pitch, order,
                            _row_offsets(state, order, rot, axis, pitch)))
        return got

    bands = dict(seat_candidate_bands(state, target[0], target[1],
                                      sweep=constraint is None))
    try:
        for clr in seat_clearances(full):
            _set_seat_clearance(state, clr)
            combos = _combos(clr)
            # BAND-major: the coarse ring at every angle and axis, then the
            # whole-board sweep, then the fine rings. A single part walks
            # ring -> fine -> xfine -> sweep at ONE angle before the next;
            # a row that did the same spent the whole pose cap on its first
            # angle's 16k-position fine ring (splitflap U4:47k, 7 members,
            # capped at 20000 once 2.4 seated the bigger parts first). The
            # fine rings exist for sub-mm windows a single part can use; a
            # row needs a strip, which the coarse bands find.
            for bname in ('ring', 'sweep', 'fine', 'xfine'):
                if bname not in bands:
                    continue
                for rot, axis, pitch, order, offs in combos:
                    for ax, ay in bands[bname]():
                        if tried >= cap:
                            out['capped'] = True
                            out['poses_tried'] = tried
                            return out
                        tried += 1
                        poses = []
                        for m, (ox, oy) in zip(order, offs):
                            x, y = round(ax + ox, 3), round(ay + oy, 3)
                            if not gates[m](x, y, rot) or not pose_ok(
                                    state, m, x, y, rot, exclude - {m}):
                                poses = None
                                break
                            poses.append((m, x, y))
                        if poses is None:
                            continue
                        before = {m: (state.parts[m].x, state.parts[m].y,
                                      state.parts[m].rot) for m in order}
                        for m, x, y in poses:
                            state.apply_move(m, x, y, rot)
                        if _siblings_ok(state, poses, rot,
                                        exclude - set(order)):
                            out.update(ok=True, poses_tried=tried, rot=rot,
                                       axis=axis, pitch_mm=pitch,
                                       anchor=[ax, ay], order=order,
                                       clearance=clr)
                            return out
                        for m, (bx, by, br) in before.items():
                            state.apply_move(m, bx, by, br)
    finally:
        _set_seat_clearance(state, full)
    out['poses_tried'] = tried
    if not reasons:
        reasons.append('no anchor seats every member')
    return out


def _siblings_ok(state, poses, rot: float, exclude: Set[str]) -> bool:
    """The row's re-check with its siblings SEATED: every member `pose_ok`
    at its applied pose against the rest of the row. The per-member check
    during the walk excludes the unplaced siblings, so this is the only
    place sibling pads and courtyards meet; `_seat_block` reverts the row
    and walks on when it fails."""
    return all(pose_ok(state, m, x, y, rot, exclude) for m, x, y in poses)


def _formation_at(state, pcb_data, spec: Dict, members: Sequence[str]
                  ) -> Dict[str, object]:
    """`arrays.formation` over `members` at their CURRENT state poses, in the
    grade's own measurement (courtyard centre, board rotation, copper pad
    count) -- the seeder's self-check that the row it just seated is the row
    the grader will call formed. One predicate, called, never copied."""
    from . import arrays as arr
    return arr.formation_at_state(state, pcb_data, spec, members)


def _seat_array(state, pcb_data, intent, spec: Dict, zone, placed: Set[str],
                unplaced: Set[str], center, rot_ladder, cap: int,
                formed: Dict[str, Dict], unseated: Dict[str, Dict],
                notes: List[str]) -> None:
    """Stage 2.45 for one declared array (#1051): resolve the row's order,
    rotations, axes and target, then `_seat_block`.

    * ORDER: the served part's pin order for `order: "pin"` (members the
      pin order cannot place follow, in declared order; the grade discloses
      them), the declared list for `"declared"` and for `"unknown"`.
    * ROTATION: a number is binding. `"shared"` and `"unknown"` both seat
      ONE angle for the row -- the seeder owns the shared rotation -- tried
      in the first member's `_rot_ladder` order (its declared ladder, else
      its own 90-degree lattice), narrowed to the angles every member's
      declared ladder allows (`arrays.allowed_angles`), and deduplicated
      modulo 180 when every member is a <= 2-pad part.
    * AXIS: a declared `x` / `y` is used alone. `"auto"` tries both, in a
      DETERMINISTIC order: the row is first laid ACROSS the line from the
      served part to the TARGET (below) -- the target's offset from the
      host's courtyard centre, |dx| >= |dy| (east or west of it) giving 'y'
      first, else 'x' -- and then the other axis. With members that have no
      far-side partner the target is on the host's pin side, so the row
      lies parallel to that side. With no placed host, or no member
      reaching its pads, 'x' then 'y'.
    * DIRECTION: along the chosen axis the order is flipped when the first
      member's served pins lie further along the axis than the last's, so
      the row runs the way the pins do.
    * TARGET: the mean of the members' `_partner_centroid`s (every placed
      partner, host and far side alike); with none, the centroid of the
      served part's pads the members reach by their OWN nets (a net every
      member shares is a rail, and lands on every supply pin); else the
      board centre. A zoned row's target is clamped into its zone, and the
      whole row must sit in it.
    """
    from . import arrays as arr
    name = spec['name']
    members = list(spec['present'])
    order_refs = spec.get('order_refs')
    if spec['order'] in ('pin', 'declared') and order_refs:
        order = ([m for m in order_refs if m in members]
                 + [m for m in members if m not in order_refs])
    else:
        order = list(members)
    pads = {m: arr._copper_pad_count(pcb_data.footprints[m]) for m in members}
    if arr.is_number(spec['rotation']):
        rots = [float(spec['rotation']) % 360.0]
    else:
        p0 = state.parts[order[0]]
        base = rot_ladder(order[0]) or (
            [p0.rot] + [(p0.rot + d) for d in (90.0, 180.0, 270.0)])
        rots = []
        for r in base:
            r = float(r) % 360.0
            if r not in rots:
                rots.append(r)
        common = None
        for m in members:
            lad = rot_ladder(m)
            if lad is not None:
                a = arr.allowed_angles(lad, pads[m])
                common = a if common is None else (common & a)
        if common is not None:
            rots = ([r for r in rots
                     if any(arr._ang_diff(r, c) < 1e-6 for c in common)]
                    or sorted(common))
        if all(arr.rotation_period(pads[m]) == 180.0 for m in members):
            ded: List[float] = []
            for r in rots:
                if not any(arr._ang_diff(r, d, 180.0) < 1e-6 for d in ded):
                    ded.append(r)
            rots = ded

    serves = spec.get('serves')
    host = (serves if serves not in (None, 'unknown') and serves in placed
            and serves in state.parts else None)
    host_pin: Dict[str, Tuple[float, float]] = {}
    if host is not None:
        nets = {m: set(state.parts[m].nets) for m in members}
        hpads = state.parts[host].pad_globals()
        for m in members:
            others = set().union(*(nets[o] for o in members if o != m))
            use = (nets[m] - others) or nets[m]
            pts = [(gx, gy) for gx, gy, pn in hpads if pn and pn in use]
            if pts:
                host_pin[m] = (sum(p[0] for p in pts) / len(pts),
                               sum(p[1] for p in pts) / len(pts))
    # The target is the mean of the members' `_partner_centroid`s -- ALL
    # their placed partners, the host pins AND each member's far side --
    # which is what a member seated alone would aim at. The host pins alone
    # (the first form) pulled the row off its far-side nets: on glasgow the
    # airwire length on the members' nets rose 3264 -> 3718mm (phase-3
    # verifier). The host pins still decide the DIRECTION; the axis is laid
    # across the host-to-target line (AXIS, above).
    cs = [c for c in (_partner_centroid(state, m, placed) for m in members)
          if c is not None]
    if cs:
        tx = sum(c[0] for c in cs) / len(cs)
        ty = sum(c[1] for c in cs) / len(cs)
    elif host_pin:
        tx = sum(p[0] for p in host_pin.values()) / len(host_pin)
        ty = sum(p[1] for p in host_pin.values()) / len(host_pin)
    else:
        tx, ty = center
    constraint, tol = None, 0.5
    if zone is not None:
        constraint, tol = zone.rect, intent.zone_tolerance(zone)
        tx = min(max(tx, zone.rect[0]), zone.rect[2])
        ty = min(max(ty, zone.rect[1]), zone.rect[3])
    tx, ty = round(tx, 3), round(ty, 3)
    if spec['axis'] in ('x', 'y'):
        axes = [spec['axis']]
    else:
        first = 'x'
        if host is not None and host_pin:
            hr = state.parts[host].rect()
            dx = tx - (hr[0] + hr[2]) / 2.0
            dy = ty - (hr[1] + hr[3]) / 2.0
            first = 'y' if abs(dx) >= abs(dy) else 'x'
        axes = [first, 'y' if first == 'x' else 'x']

    def _reverse(axis, seq):
        k = 0 if axis == 'x' else 1
        ends = [host_pin[m][k] for m in seq if m in host_pin]
        return len(ends) >= 2 and ends[0] > ends[-1]

    res = _seat_block(state, order, rots, spec['pitch_mm'], axes, (tx, ty),
                      set(unplaced), constraint=constraint, tol=tol, cap=cap,
                      reverse_for=_reverse)
    if not res['ok']:
        why = (f"the pose cap ({cap}) was reached before any anchor seated "
               f"every member" if res['capped']
               else '; '.join(res['reasons']))
        unseated[name] = {'members': members,
                          'poses_tried': res['poses_tried'],
                          'capped': res['capped'], 'reason': why,
                          'target': [tx, ty]}
        notes.append(f"array {name}: NOT seated as a row after "
                     f"{res['poses_tried']} pose(s) -- {why}; its members "
                     f"are seated one by one")
        if zone is not None:
            # Stage 2 stepped aside for this row; put its members into their
            # zone one by one, as stage 2 would have, rather than leave them
            # to the unconstrained centroid stage.
            zx = (zone.rect[0] + zone.rect[2]) / 2.0
            zy = (zone.rect[1] + zone.rect[3]) / 2.0
            missed = []
            for ref in sorted(members, key=lambda r: (
                    -state.parts[r].pin_count, r)):
                if _try_place(state, ref, zx, zy, unplaced - {ref},
                              constraint=zone.rect, tol=tol,
                              rotations=rot_ladder(ref)) is not None:
                    placed.add(ref)
                    unplaced.discard(ref)
                else:
                    missed.append(ref)
                    notes.append(f"{ref}: array {name}'s zone fallback "
                                 f"found no pose in zone {zone.name!r} -- "
                                 f"left to the centroid stage")
            if missed:
                unseated[name]['zone_unseated'] = missed
        return
    for m in res['order']:
        placed.add(m)
        unplaced.discard(m)
    v = _formation_at(state, pcb_data, spec, res['order'])
    rects = [state.parts[m].rect() for m in res['order']]
    cxs = [(r[0] + r[2]) / 2.0 for r in rects]
    cys = [(r[1] + r[3]) / 2.0 for r in rects]
    formed[name] = {
        'serves': serves, 'members': list(res['order']),
        'rot': res['rot'], 'pitch_mm': res['pitch_mm'], 'axis': res['axis'],
        'anchor': [round(sum(cxs) / len(cxs), 3),
                   round(sum(cys) / len(cys), 3)],
        'target': [tx, ty], 'zone': getattr(zone, 'name', None),
        'poses_tried': res['poses_tried'], 'clearance': res['clearance'],
        'verdict': 'formed' if v['formed'] else 'broken',
        'failed': list(v['failed']), 'unchecked': list(v['unchecked'])}
    notes.append(
        f"array {name}: seated {len(members)} member(s) as one row along "
        f"{res['axis']} at {res['rot']:g}deg, pitch {res['pitch_mm']:g}mm, "
        f"after {res['poses_tried']} pose(s)"
        + (f" at reduced courtyard clearance {res['clearance']:g}"
           if res['clearance'] < state.clearance else '')
        + (" -- formation self-check PASSED" if v['formed'] else
           f" -- formation self-check FAILED: {', '.join(v['failed'])}"))


#: #1054: how close a part already standing where a fixed pose puts it must
#: be to count as AT that pose -- the writer's 3dp rounding, not a tolerance.
FIXED_POSE_EPS_MM = 1e-3


def _holes_outside(state, gate, ref: str, x: float, y: float,
                   rot: float) -> Tuple[int, int]:
    """`(holes, outside)`: how many drill holes (plated and NPTH) `ref` has,
    and how many leave `gate`'s outline at (x, y, rot) -- the hole's whole
    bounding square, so a hole straddling the edge counts. The pad frame
    transform is `connector_geometry.pad_copper_outside`'s own."""
    fp = (state.pcb_data.footprints or {}).get(ref)
    if fp is None:
        return 0, 0
    c, s = math.cos(math.radians(rot)), math.sin(math.radians(rot))
    n = out = 0
    for p in fp.pads:
        d = float(getattr(p, 'drill', 0) or 0)
        if d <= 0:
            continue
        n += 1
        lx, ly = float(p.local_x), float(p.local_y)
        px, py = x + c * lx + s * ly, y - s * lx + c * ly
        h = d / 2.0
        if gate.rect_outside_amount((px - h, py - h, px + h, py + h)) > 1e-9:
            out += 1
    return n, out


#: #1054: a courtyard overlap AREA (mm^2) above this is an overlap; below it
#: is float noise on two courtyards that abut. KiCad's `courtyards_overlap`
#: is an intersection test, so abutting courtyards (gap 0) are legal.
FIXED_OVERLAP_EPS_MM2 = 1e-6


def _courtyard_overlap(state, a: str, pose_a, b: str, pose_b):
    """`(area mm^2, w, h)` of the courtyard overlap of `a` at `pose_a` with
    `b` at `pose_b`. The VERDICT is `legality.pair_overlap_area` -- the
    side-aware measure -- on each part's DRAWN courtyard, the way KiCad
    judges a declared pose (`grade_rect`), and where those overlap,
    `legality.pair_overlap_area_exact` on the drawn outlines (#1094:
    StickHub's declared -135 degree human poses were refused on rects
    alone). A part that draws NO courtyard is screened on its occupancy
    instead (`occupancy_rect_at(courtyard_less_only=True)`, #1182): its
    ladder rect is a pad box, and two fab bodies meeting outside their pads
    read 0 while check_assembly grades them. `w` x `h` is the rects'
    intersection, for the refusal's text only."""
    from .legality import (graded_part_at_pose, occupancy_rect_at,
                           pair_overlap_area, pair_overlap_area_exact)
    if getattr(state, 'courtyards_ignored', False):
        # #1101: the project waives KiCad's courtyard rule, so a declared
        # pose is not refused for one (its drill holes still are, below).
        return 0.0, 0.0, 0.0
    pa, pb = state.parts[a], state.parts[b]
    cache = state.__dict__.setdefault('_exact_overlap_cache', {})
    pcb_file = getattr(state, 'pcb_file', None)
    ta, tb = pa.tht_rect(*pose_a), pb.tht_rect(*pose_b)
    ra = occupancy_rect_at(state.pcb_data, a, pose_a, pa.grade_rect(*pose_a),
                           pcb_file, cache, courtyard_less_only=True)
    rb = occupancy_rect_at(state.pcb_data, b, pose_b, pb.grade_rect(*pose_b),
                           pcb_file, cache, courtyard_less_only=True)
    area = pair_overlap_area(pa.sides, pa.side, ra, ta,
                             pb.sides, pb.side, rb, tb)
    if area > FIXED_OVERLAP_EPS_MM2:
        ga = graded_part_at_pose(state.pcb_data, a, pose_a, pa.side, ra, ta,
                                 ta is not None, pcb_file, cache)
        gb = graded_part_at_pose(state.pcb_data, b, pose_b, pb.side, rb, tb,
                                 tb is not None, pcb_file, cache)
        if ga.poly is not None or gb.poly is not None:
            area = pair_overlap_area_exact(ga, gb)
    w = max(0.0, min(ra[2], rb[2]) - max(ra[0], rb[0]))
    h = max(0.0, min(ra[3], rb[3]) - max(ra[1], rb[1]))
    return area, w, h


def _drill_conflict(state, a: str, pose_a, b: str, pose_b) -> Optional[str]:
    """The closest pair of DRILL holes of `a` and `b` (plated or not) at their
    poses, as a refusal, when they are closer than the board's own
    `min_hole_to_hole` (else touching). None when clear.

    The courtyard branch of `_fixed_pose_check` was the only thing that
    caught two holes stacked on each other -- `pair_shortfall` checks a hole
    against the other part's COPPER, never against its hole -- so a courtyard
    waiver must re-ask it (#1060, phase-4 verifier: two coincident NPTH drills
    seated under `accept_courtyard_overlap`)."""
    from .legality import footprint_at_pose

    def drills(ref, pose):
        fp = (state.pcb_data.footprints or {}).get(ref)
        if fp is None:
            return []
        out = []
        for p in footprint_at_pose(fp, pose).pads:
            d = max(float(getattr(p, 'drill', 0) or 0),
                    float(getattr(p, 'drill_w', 0) or 0),
                    float(getattr(p, 'drill_h', 0) or 0))
            if d <= 0:
                continue
            hx = p.hole_x if getattr(p, 'hole_x', None) is not None \
                else p.global_x
            hy = p.hole_y if getattr(p, 'hole_y', None) is not None \
                else p.global_y
            out.append((hx, hy, d / 2.0, str(p.pad_number)))
        return out
    da, db = drills(a, pose_a), drills(b, pose_b)
    if not da or not db:
        return None
    # Read once per state: the waived-courtyard seat (#1101) calls this per
    # candidate pose, and the constraint is a file read.
    floor = getattr(state, '_h2h_floor', None)
    if floor is None:
        floor = 0.0
        try:
            from list_nets import board_constraint
            floor = float(board_constraint(state.pcb_file,
                                           'min_hole_to_hole') or 0.0)
        except Exception:                                  # noqa: BLE001
            floor = 0.0
        try:
            state._h2h_floor = floor
        except Exception:                                  # noqa: BLE001
            pass
    worst = None
    for ax, ay, ar, an in da:
        for bx, by, br, bn in db:
            gap = math.hypot(ax - bx, ay - by) - ar - br
            if gap < floor - 1e-6 and (worst is None or gap < worst[0]):
                worst = (gap, an, bn)
    if worst is None:
        return None
    def name(ref, num):
        return f"{ref}.{num}" if num else f"{ref}'s hole"
    return (f"drill {name(a, worst[1])} is {worst[0]:.3f}mm from "
            f"{name(b, worst[2])} (hole-to-hole floor {floor:g}mm; a negative "
            f"gap is holes overlapping)")


def _waived_what(m: Dict) -> str:
    """One waived pair's measurement, as `_fixed_pose_check` recorded it:
    a courtyard overlap's box and area, a frame pin under the courtyard
    (#1212), or both."""
    what = []
    if 'area_mm2' in m:
        what.append(f"{m['w_mm']:.2f}x{m['h_mm']:.2f}mm, "
                    f"{m['area_mm2']:.3f}mm2")
    if m.get('pins'):
        what.append(f"pin(s) {', '.join(m['pins'])}")
    return '; '.join(what)


def _fixed_pose_check(state, ref: str, pose, obstacles: Dict[str, Tuple],
                      waived=frozenset(),
                      waived_out: Optional[Dict[str, Dict]] = None
                      ) -> Tuple[Optional[str], List[str], Dict[str, str]]:
    """`(how, reasons, conflicts)` for seating `ref` EXACTLY at `pose`.

    A CHECK, never a search (#1054): the pose is a mechanical fact, so a
    pose that fails is refused with its reasons and never nudged.

    `obstacles` is `{ref: pose}`: every part already placed, at its pose,
    AND every other declared fixed pose at ITS declared pose -- so two
    declarations are judged against each other symmetrically, whatever
    order they are seated in. `conflicts` is `{other: reason}` for the
    obstacles this pose collides with, so the caller can refuse both halves
    of a clash between two declarations.

    COURTYARDS, the way KiCad judges them: an OVERLAP (area above
    `FIXED_OVERLAP_EPS_MM2`, `legality.pair_overlap_area`) is illegal;
    courtyards that ABUT (gap >= 0) are legal. Only for a DECLARED pose:
    every searched seat keeps the board clearance and its 0.02mm floor
    (`seat_clearances`), which buy margin where the seeder chooses. A
    human layout packs courtyards edge to edge (glasgow's RN banks and
    SN74LVC1T45 buffers: 14 pairs at exactly 0.000mm, which kicad-cli's DRC
    accepts) and a fixed pose exists to express THAT arrangement -- under
    the searched-seat floor stage 0 refused 15 of 30 human glasgow poses,
    each "within 0.02mm".

    Everything else at its normal rule, ABSOLUTE (a declared pose has no
    incumbent to be "no worse than"): the outline (containment at the
    board-edge margin), declared keep-outs and exclusive zones, #1031's
    rule-area keep-out band (`legality_ctx.keepout_amount`), and to every
    obstacle `legality_ctx.pair_shortfall`'s pad clearance, hole clearance,
    pad short and cross-part pad stack -- `pads_ok`'s conjuncts, called.

    `waived` (#1060) is a set of unordered pairs `frozenset({a, b})` whose
    COURTYARD overlap is declared -- `fixed_poses[].accept_courtyard_overlap`
    and `overlap_waivers[]`. For such a pair the courtyard branch alone is
    skipped, and the measurement is recorded in `waived_out[other]` instead
    of refusing; pad clearance, pad shorts, hole clearance, the keep-out band
    and the outline stay absolute, because a waiver is a claim about two
    courtyards and nothing else.

    * `'contained'`: inside the outline, and nothing above fails.
    * `'overhang'`: the courtyard leaves the outline -- a connector or a
      mounting part may overhang by design, which is stage 1's exemption --
      accepted only when every pad's copper is on the board at zero margin
      and every drill hole (NPTH included) too, and a part with neither is
      judged by its courtyard at zero margin (the pad test is vacuous for a
      part with no copper).

    `how` None means refused; `reasons` then says why, with the measurement.
    """
    part = state.parts[ref]
    x, y, rot = pose
    reasons: List[str] = []
    conflicts: Dict[str, str] = {}
    r, tht = part.grade_rects(x, y, rot)
    outside = state.edge_gate.rect_outside_amount(r) > 1e-9
    reasons.extend(f"keep-out {n!r}"
                   for n in state.keepout_blockers(ref, (r, tht)))
    reasons.extend(f"exclusive zone {n!r}"
                   for n in state.exclusive_blockers(ref, (r, tht)))
    containers = getattr(state, 'container_refs', ()) or ()
    ctx = state.legality_ctx
    # #1212: a declared pose on a frame's pin is refused like a courtyard
    # overlap -- the frame's rect is skipped below, its pins are not. Judged
    # against the obstacles at THEIR poses, and, like a courtyard overlap,
    # recorded instead of refused when the pair's overlap is declared: the
    # grader waives an intent-declared pin pair (`intent_declared`) too.
    if getattr(state, 'pin_frame_refs', None):
        for _hit in state._pin_hits_at(
                ref, *pose, poses={o: tuple(p) for o, p in obstacles.items()
                                   if o != ref}):
            _other = _hit.b if _hit.a == ref else _hit.a
            if _other not in obstacles:
                continue
            _pins = ', '.join(_hit.pins)
            if frozenset((ref, _other)) in waived:
                if waived_out is not None:
                    waived_out.setdefault(_other, {})['pins'] = list(
                        _hit.pins)
                continue
            conflicts[_other] = (
                f"sits on {_other}'s pin(s) {_pins} (pin_in_courtyard)"
                if _other in state.pin_frame_refs else
                f"pin(s) {_pins} under {_other}'s courtyard "
                f"(pin_in_courtyard)")
    for other in sorted(obstacles):
        if other == ref or other not in state.parts:
            continue
        opose = obstacles[other]
        if ref not in containers and other not in containers:
            area, w, h = _courtyard_overlap(state, ref, pose, other, opose)
            if area > FIXED_OVERLAP_EPS_MM2:
                if frozenset((ref, other)) in waived:
                    if waived_out is not None:
                        waived_out.setdefault(other, {}).update(
                            {'area_mm2': round(area, 4),
                             'w_mm': round(w, 3), 'h_mm': round(h, 3)})
                    # The courtyard is waived; its holes are not.
                    _dh = _drill_conflict(state, ref, pose, other, opose)
                    if _dh:
                        conflicts[other] = _dh
                else:
                    conflicts[other] = (f"courtyard overlaps {other} by "
                                        f"{w:.2f}x{h:.2f}mm ({area:.3f}mm2)")
            elif getattr(state, 'courtyards_ignored', False):
                # #1101: the project waives the courtyard, and with it the
                # only branch above that asked about stacked holes; holes are
                # not waived (review: two coincident NPTH seated).
                _dh = _drill_conflict(state, ref, pose, other, opose)
                if _dh:
                    conflicts[other] = _dh
        if ctx is not None:
            sf = ctx.pair_shortfall(ref, other, pose_a=pose, pose_b=opose)
            # `pads_ok`'s conjuncts, ABSOLUTE rather than seed-relative: a
            # declared pose has no incumbent to be "no worse than".
            what = []
            if sf.pad_overlap:
                what.append(f"pads short {other}'s (different-net copper "
                            f"intersects)")
            elif sf.stack:
                what.append(f"pads stack on {other}'s copper")
            if sf.pad > 1e-6:
                what.append(f"pad clearance to {other} short by "
                            f"{sf.pad:.3f}mm")
            if sf.hole > 1e-6:
                what.append(f"hole clearance to {other} short by "
                            f"{sf.hole:.3f}mm")
            if what:
                conflicts[other] = '; '.join(
                    ([conflicts[other]] if other in conflicts else [])
                    + what)
    reasons.extend(conflicts[o] for o in sorted(conflicts))
    # #1031's rule-area keep-out band, ABSOLUTE (`keepout_ok` is seed-
    # relative, and a pile seed is no licence): pad copper the band forbids
    # at this pose refuses it, measured in the band's own currency. Without
    # it stage 0 seated test_1031's R2 at (37.6, 15) -- a pose place_pose
    # refuses and grade_pad_legality reports under oob_keepout_copper_refs.
    if ctx is not None:
        ko = ctx.keepout_amount(ref, x, y, rot)
        if ko > 1e-6:
            reasons.append(f"pad copper {ko:.3f}mm into a rule-area "
                           f"keep-out band")
    if outside:
        from .connector_geometry import (geometry_for, pad_boxes,
                                         pad_copper_outside)
        from .legality import BoardOutlineGate
        zero = getattr(state, '_zero_edge_gate', None)
        if zero is None:
            zero = BoardOutlineGate(state.pcb_data.board_info, 0.0)
            state._zero_edge_gate = zero
        geometry = geometry_for(state, state.pcb_data, state.pcb_file)
        # A pad-less part is judged by its HOLES (the whole drill circle,
        # zero margin), else by its courtyard at zero margin -- not by the
        # margin-gated courtyard: a mounting hole's courtyard legitimately
        # crosses the edge (tigard H1 at its own human pose is 1.3mm past
        # the margin gate, and must seat). Without this a part with no copper
        # passed "every pad on the board" vacuously (tigard H1 at (300, 300)).
        n_holes, holes_out = _holes_outside(state, zero, ref, x, y, rot)
        if not pad_boxes(geometry, ref) and not n_holes:
            amt = zero.rect_outside_amount(r)
            if amt > 1e-9:
                reasons.append(f"no copper pad or hole to anchor an "
                               f"overhang, and its courtyard is {amt:.3f}mm "
                               f"past the outline")
        off = pad_copper_outside(geometry, zero, ref, (x, y, rot))
        if off > 1e-9:
            reasons.append(f"pad copper {off:.3f}mm past the outline")
        if holes_out:
            reasons.append(f"{holes_out} of {n_holes} drill hole(s) past "
                           f"the outline")
        return (None if reasons else 'overhang'), reasons, conflicts
    return (None if reasons else 'contained'), reasons, conflicts


def _fixed_pose_prep(state, pcb_data, f: Dict, placed: Set[str],
                     held: Set[str], seated: Dict[str, Dict],
                     refused: Dict[str, Dict], lock: Set[str],
                     notes: List[str]):
    """The per-entry half of stage 0: resolve one `fixed_poses[]` entry's
    pose, and settle the entries no geometry check is needed for. Returns
    `(ref, pose, rec)` for an entry to check, else None.

    `rot` / `side` absent or `"unknown"` keep the part's CURRENT rotation /
    side, and the record says so (`rot_kept`, `side_kept`) -- the author
    declared that they do not know, so nothing is guessed. A declared side
    the part is not on is REFUSED: this search has no flip move, and seating
    the front-side geometry at a back-side pose would grade a part that is
    not the one written.
    """
    from .legality import footprint_side
    ref = str(f['ref'])
    x, y = round(float(f['x']), 3), round(float(f['y']), 3)
    if ref not in state.parts:
        why = ('not on this board' if ref not in (pcb_data.footprints or {})
               else 'the placement state carries no geometry for it '
                    '(pad-less)')
        refused[ref] = {'reason': why, 'pose': [x, y, f.get('rot')]}
        notes.append(f"fixed pose {ref}: REFUSED -- {why}")
        return None
    # #1060: a waiver of a part the board does not have waives nothing, and
    # a typo there would read as a pose that seated clean.
    absent = [o for o in (f.get('accept_courtyard_overlap') or ())
              if o not in (pcb_data.footprints or {})]
    if absent:
        why = (f"accept_courtyard_overlap names {', '.join(absent)}, which "
               f"{'is' if len(absent) == 1 else 'are'} not on this board")
        refused[ref] = {'reason': why, 'pose': [x, y, f.get('rot')]}
        notes.append(f"fixed pose {ref}: REFUSED -- {why}")
        return None
    part = state.parts[ref]
    rot_decl = f.get('rot')
    rot_kept = rot_decl is None or rot_decl == 'unknown'
    rot = (part.rot % 360.0) if rot_kept else float(rot_decl) % 360.0
    side_decl = f.get('side')
    side_kept = side_decl is None or side_decl == 'unknown'
    side_now = footprint_side(pcb_data.footprints[ref])
    rec = {'x': x, 'y': y, 'rot': rot, 'side': side_now,
           'basis': f.get('basis'), 'rot_kept': rot_kept,
           'side_kept': side_kept}
    if f.get('accept_courtyard_overlap'):
        rec['waives'] = sorted(f['accept_courtyard_overlap'])
    if not side_kept and side_decl != side_now:
        held.add(ref)
        refused[ref] = dict(rec, reason=(
            f"declared side {side_decl}, and the part is on {side_now}: the "
            f"seeder has no flip move, so it cannot seat it there"))
        notes.append(f"fixed pose {ref}: REFUSED -- {refused[ref]['reason']}"
                     f"; flip the part on the board first")
        return None
    if ref in placed:
        # Authoritative already: locked in the FILE, or outside an explicit
        # `seed_refs` scope. Not this stage's to move -- the file lock is the
        # user's. At the pose it is simply recorded; anywhere else is a
        # contradiction to name, never to resolve by force.
        at = (math.hypot(part.x - x, part.y - y) <= FIXED_POSE_EPS_MM
              and _ang_close(part.rot, rot))
        if at:
            seated[ref] = dict(rec, how='already_there')
            if not part.locked:
                lock.add(ref)
            return None
        refused[ref] = dict(rec, reason=(
            f"already placed at ({part.x:g}, {part.y:g}, {part.rot:g}deg) -- "
            + ("locked in the board file" if part.locked
               else "outside the seed scope")
            + ", and not this stage's to move"))
        notes.append(f"fixed pose {ref}: REFUSED -- {refused[ref]['reason']}"
                     f" (declared ({x:g}, {y:g}, {rot:g}deg))")
        return None
    rot = _materialise_rotation(part, rot)
    return ref, (x, y, rot), rec


def _seat_fixed_poses(state, pcb_data, entries, placed: Set[str],
                      unplaced: Set[str], held: Set[str],
                      seated: Dict[str, Dict], refused: Dict[str, Dict],
                      lock: Set[str], notes: List[str],
                      waived=frozenset()) -> None:
    """Stage 0 (#1054): every `fixed_poses[]` entry, judged as ONE batch.

    ORDER-INDEPENDENT. Each declared pose is checked against the parts
    already placed AND every other declared pose at its declared pose, and
    only then are the survivors seated -- so the verdict on A never depends
    on whether B's ref sorts first. Seating in ref order and checking each
    against the ones already seated refused glasgow's human poses in an
    alternating pattern (phase-6 verifier).

    A clash between two DECLARATIONS refuses BOTH, each naming the other
    and the measurement. Neither declaration outranks the other -- both are
    the author's statement of where a part IS -- so keeping one would pick
    a winner by reference name, and a refusal that names the pair is what
    the author needs to fix the one that is wrong.

    `waived` (#1060): the declared courtyard waivers as unordered pairs, so
    a waiver on EITHER entry of a pair covers both checks -- checked one way
    only, FID8 declared beside U30 would still refuse both halves.
    """
    declared: Dict[str, Tuple] = {}
    recs: Dict[str, Dict] = {}
    for f in sorted(entries, key=lambda f: str(f['ref'])):
        got = _fixed_pose_prep(state, pcb_data, f, placed, held, seated,
                               refused, lock, notes)
        if got is not None:
            ref, pose, rec = got
            declared[ref] = pose
            recs[ref] = rec
    fixed_obstacles = {r: (state.parts[r].x, state.parts[r].y,
                           state.parts[r].rot) for r in placed
                       if r in state.parts}
    verdicts = {}
    waived_hits: Dict[str, Dict[str, Dict]] = {}
    for ref in sorted(declared):
        obstacles = dict(fixed_obstacles)
        obstacles.update({o: p for o, p in declared.items() if o != ref})
        waived_hits[ref] = {}
        verdicts[ref] = _fixed_pose_check(state, ref, declared[ref],
                                          obstacles, waived=waived,
                                          waived_out=waived_hits[ref])
    for ref in sorted(declared):
        how, reasons, conflicts = verdicts[ref]
        x, y, rot = declared[ref]
        rec = recs[ref]
        clash = sorted(o for o in conflicts if o in declared)
        if how is None:
            held.add(ref)
            refused[ref] = dict(rec, reason='; '.join(reasons),
                                conflicts_with_declared=clash)
            notes.append(
                f"fixed pose {ref}: REFUSED at ({x:g}, {y:g}, {rot:g}deg) -- "
                f"{refused[ref]['reason']}."
                + (f" {', '.join(clash)} {'is' if len(clash) == 1 else 'are'}"
                   f" ALSO a declared fixed pose, so both declarations are "
                   f"refused: they cannot both be true" if clash else '')
                + " The pose is a fact, so it is never nudged: fix the pose "
                  "or what it collides with")
            continue
        state.apply_move(ref, x, y, rot)
        placed.add(ref)
        unplaced.discard(ref)
        lock.add(ref)
        seated[ref] = dict(rec, how=how)
        if waived_hits.get(ref):
            # Disclosed, never silent: the measured overlap each waiver
            # accepted, in the record `JSON_SUMMARY.fixed_seated` carries.
            seated[ref]['courtyard_waived'] = waived_hits[ref]
            notes.append(
                f"fixed pose {ref}: courtyard overlap WAIVED with "
                + ', '.join(f"{o} ({_waived_what(m)})"
                            for o, m in sorted(waived_hits[ref].items()))
                + " -- declared by accept_courtyard_overlap / "
                  "overlap_waivers; pads, holes, keep-outs and the outline "
                  "were still checked")
        unused = sorted(o for o in (recs[ref].get('waives') or ())
                        if o not in waived_hits.get(ref, {}))
        if unused:
            notes.append(f"fixed pose {ref}: accept_courtyard_overlap names "
                         f"{', '.join(unused)}, and stage 0 measured no "
                         f"courtyard overlap with "
                         f"{'it' if len(unused) == 1 else 'them'} (not yet "
                         f"placed, or not overlapping: the grade reports "
                         f"which)")
        notes.append(f"fixed pose {ref}: seated exactly at ({x:g}, {y:g}, "
                     f"{rot:g}deg)"
                     + (" overhanging the outline (pads on the board)"
                        if how == 'overhang' else '')
                     + (" at its current rotation (declared unknown)"
                        if rec['rot_kept'] else '')
                     + " -- locked")


def _ang_close(a: float, b: float, eps: float = 1e-6) -> bool:
    d = abs(float(a) - float(b)) % 360.0
    return min(d, 360.0 - d) <= eps


#: #1099: after the 90-degree lattice seats nothing, try the diagonals (and
#: seat a cap on a diagonal chip on the chip's lattice first). OFF by
#: default: tests/test_placement_ab.py's `diag-seed-*` rows wrote identical
#: poses in both arms on all five boards -- none of their unseated parts is
#: one the diagonals seat -- so the table has no evidence to make it a
#: default. `place_seed --diagonal-rotations` opts in, for a caller whose
#: reference placement is diagonal; no skill passes it.
DIAGONAL_ROTATIONS_DEFAULT = False

#: #1105: stage 3.5 -- the per-supply-pin claim run AGAIN inside stage 3, at
#: the first scoped cap after the queue's last owner IC, over the owner ICs
#: stage 3 itself seated. On a pile or a flat board nothing seats an owner IC
#: before stage 2.5, so 2.5 claims nothing and every decap fell through to the
#: generic centroid seat. Stage 3 seats by pin count, so every IC is seated
#: before any 2-pin cap, and the claim draws no RNG: every part seated before
#: it is bit-identical to the stage-off seed. Both earlier attempts seated
#: the owner ICs EARLY instead -- #1059's `seat_owners_first` (removed in
#: f77b0acd) and the `--decap-owners-first` stage 2.5a PR #1110 proposed
#: (e4317972, held out of the merge by a14f68f3) -- which moved the ICs and
#: regressed on 4 of 4 A/B boards. Set by tests/test_placement_ab.py's `decap-after-ics-*` rows;
#: `place_seed --decap-claim-after-ics` / `--no-decap-claim-after-ics`
#: override it.
DECAP_CLAIM_AFTER_ICS_DEFAULT = False

#: #1105: stage 3.5 undoes a seat that lands farther from its pin target than
#: the declared `decaps.max_distance_mm`, and the cap keeps its own centroid
#: turn. Two adjacent pins on different rails send their caps to one spot, and
#: the second cap then lands mm away from the pin it claimed -- measured on
#: the run-29 pile: C4 5.00mm from its target and graded 3.85mm from U1,
#: where its own centroid turn seated it inside the limit.
DECAP_LATE_WITHIN_LIMIT = False

#: #1105: where stage 3.5 runs in stage 3's queue. 'after_last_owner' is the
#: first scoped cap after the last owner IC; 'after_queue' holds every scoped
#: cap back until the rest of the queue is seated, so a claimed cap never
#: takes a pose a later resistor or LED would have had.
DECAP_LATE_AT = 'after_last_owner'


def _decap_owner_ok(ref: str, chips: Optional[Set[str]]) -> bool:
    """Whether the pin stages may serve `ref`'s supply pins (#1105).

    One spelling for stage 2.5, stage 3.5 and `decap_pin_forecast`: the
    grouper's chips under `decap_owner_chips`, else a U-prefixed ref (a
    castellated row carries the rail too and must not eat a claim)."""
    return (ref in chips) if chips is not None else (ref[0:1] == 'U')


def decap_graded_distance(pcb_data, state, cap: str, chips, placed
                          ) -> Tuple[Optional[str], Optional[float]]:
    """`(chip, mm)`: how far `cap` is from its IC AS THE GRADE MEASURES IT,
    at the state's live poses -- the cap's pad centroid to the nearest pad
    box among the `chips` that are `placed` (`groups.elect_live`, the
    election `rule_decap_distance` reads and the quench's #1043 gate calls
    per pose). An unplaced chip still sits at its staging pose, which is not
    where the grade will find it, so it is not a candidate. `(None, None)`
    when no placed chip is given, and then the grade has no tether to grade
    either.

    Stage 3.5's within-limit check reads this. It used to read the distance
    from the cap to the PIN TARGET it was aimed at, which is never shorter
    than the distance to the pin's own IC's pad box, so it declined seats the
    grade accepts (#1141: seeding a watchy pile with after_queue at upstream
    main 055fa9e1 undid 16 seats, 9 of them inside the limit as graded)."""
    from . import groups as _g
    from .legality import footprint_at_pose

    def _posed(ref):
        p = state.parts[ref]
        return footprint_at_pose(pcb_data.footprints[ref], (p.x, p.y, p.rot))
    cands = []
    for c in chips:
        if c not in placed:
            continue
        b = _g.chip_bounds_of(_posed(c))
        if b is not None:
            cands.append((c, b))
    return _g.elect_live(_posed(cap), cands)


def _decap_rail(nets, net_refs) -> Optional[int]:
    """A decap's rail: of its nets with two or more owners, the one with the
    FEWEST owners (GND has the most), ties by net id (#1105: shared by the
    pin stages and `decap_pin_forecast`)."""
    return min((nid for nid in nets if len(net_refs.get(nid, ())) >= 2),
               key=lambda nid: (len(net_refs[nid]), nid), default=None)


def decap_pin_forecast(pcb_data, intent, blocks, *,
                       standing: Sequence[str] = (),
                       owner_chips: bool = False,
                       claim_after_ics: Optional[bool] = None) -> Dict:
    """What the pin stages CAN claim when this intent seeds this board (#1105).

    A forecast of PINS, not of seats: a cap counts as claimable when an
    owner IC carries its rail, and the seed may still decline it (no legal
    pose, more caps than pin clusters). `early` caps have an owner that is
    seated before stage 2.5 -- one in `standing` (file-locked, or outside a
    partially-unplaced board's pile), a fixed pose, an edge claim, a
    must_lock part, a zoned block member, or a declared row's `serves`
    (ASSUMING those seats succeed: an edge claim's walk can fail, and then
    the cap falls to its `backup_owners`, the stage-3 owners of its rail);
    `late` caps have owners only the centroid stage seats, which stage 3.5
    serves when armed (`late_armed`); `ownerless` caps have a rail no
    U-prefixed part (the grouper's chips under `owner_chips`) carries, and
    neither stage can claim them. Same scope, rail and owner rules as the
    stages themselves (`_decap_rail`, `_decap_owner_ok`).
    """
    from placement import floorplan
    from placement import groups as _g
    spec = getattr(intent, 'decaps', None) or {}
    out: Dict[str, Any] = {
        'armed': spec.get('max_distance_mm') is not None,
        'owner_rule': 'chips' if owner_chips else 'U-prefixed',
        'scope': 0, 'exempt': [], 'array_members': [], 'ownerless': [],
        'early': [], 'early_owners': [], 'late': [], 'late_owners': [],
        'backup_owners': [],
        'late_armed': (DECAP_CLAIM_AFTER_ICS_DEFAULT if claim_after_ics is None
                       else bool(claim_after_ics))}
    if not out['armed']:
        return out
    fps = pcb_data.footprints
    near, beyond, _orphans = _g.decap_populations(pcb_data)
    tethered = ({c for caps in near.values() for c, _d in caps}
                | {c for c, _ic, _d in beyond})
    exempt = tuple(spec.get('exempt') or ())
    out['exempt'] = sorted(r for r in tethered
                           if any(fnmatch.fnmatchcase(r, p) for p in exempt))
    scope = {r for r in tethered if r in fps and r not in out['exempt']}
    arrays = (floorplan.resolved_arrays(intent, pcb_data)
              if getattr(intent, 'arrays', ()) else ())
    members = {m for a in arrays for m in a.get('present') or ()}
    out['array_members'] = sorted(scope & members)
    scope -= members
    out['scope'] = len(scope)

    nets_of: Dict[str, List[int]] = {}
    by_net: Dict[int, List[str]] = {}
    for ref, fp in fps.items():
        nets = sorted({p.net_id for p in fp.pads if p.net_id > 0})
        nets_of[ref] = nets
        for n in nets:
            by_net.setdefault(n, []).append(ref)
    net_refs = {n: sorted(r) for n, r in by_net.items()}
    chips = _g.chip_refs(pcb_data) if owner_chips else None

    refs_all = sorted(fps)
    zoned = {m for z in intent.blocks if z.rect is not None
             for m in blocks.get(z.name, ())}
    early_set = (set(standing)
                 | {str(f['ref']) for f in intent.fixed_poses}
                 | {str(c['ref']) for c in intent.edge_claims()}
                 | {r for p in intent.must_lock
                    for r in refs_all if fnmatch.fnmatchcase(r, p)}
                 | zoned
                 | {str(a['serves']) for a in arrays
                    if a.get('serves') not in (None, 'unknown')
                    and not (set(a.get('present') or ()) & zoned)})
    early_owners: Set[str] = set()
    late_owners: Set[str] = set()
    backup_owners: Set[str] = set()
    for cap in sorted(scope):
        rail = _decap_rail(nets_of.get(cap, ()), net_refs)
        owners = [r for r in net_refs.get(rail, ())
                  if r != cap and _decap_owner_ok(r, chips)] if rail else []
        if not owners:
            out['ownerless'].append(cap)
        elif any(o in early_set for o in owners):
            out['early'].append(cap)
            early_owners.update(o for o in owners if o in early_set)
            # An early seat can fail (an edge claim's walk, a refused fixed
            # pose); the cap then falls to the stage-3 owners of its rail.
            backup_owners.update(o for o in owners if o not in early_set)
        else:
            out['late'].append(cap)
            late_owners.update(owners)
    out['early_owners'] = sorted(early_owners)
    out['late_owners'] = sorted(late_owners)
    out['backup_owners'] = sorted(backup_owners)
    return out


def seed_from_intent(pcb_data, pcb_file: str, intent, rng: random.Random, *,
                     group_sources: Sequence[str] = (),
                     clearance: float = 0.25,
                     board_edge_clearance: float = 0.55,
                     grid_step: float = 0.1,
                     seed_refs: Optional[Set[str]] = None,
                     anchors_first: bool = False,
                     anchor_rounds: int = 1,
                     evict_depth: int = 0,
                     decap_owner_chips: bool = False,
                     immovable_extra: Sequence[str] = (),
                     body_model: bool = False,
                     rotate_by_facing: bool = False,
                     array_pose_cap: int = ARRAY_SEAT_POSE_CAP,
                     diagonal_rotations: Optional[bool] = None,
                     decap_claim_after_ics: Optional[bool] = None,
                     dispose_unseated: bool = True) -> Dict:
    """Compute a full placement for an unplaced board from its intent.

    `dispose_unseated` (#1151): stage, below the board, a part it could not
    seat whose input pose makes a hard conflict (`_dispose_unseated`). True
    for a seed; `reseat_scope` passes False -- it re-seats a SCOPE inside a
    placed board and keeps a scope ref it cannot seat where it was, and a
    staging row there cost a valid re-seat of the rest of the scope (the
    phase-1 verifier: base `pad_pairs 3 -> 1`, staged: the pass refused).

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

    `decap_claim_after_ics` arms stage 3.5 (#1105): the per-supply-pin decap
    claim, run again inside stage 3 once the centroid stage has seated the
    owner ICs that stage 2.5 found unplaced. None takes
    `DECAP_CLAIM_AFTER_ICS_DEFAULT`; what it did is `decap_stage['late']`.
    """
    if evict_depth not in (0, 1, 2):
        raise ValueError(
            f"evict_depth must be 0, 1 or 2, got {evict_depth!r}")
    import pose_score
    from placement import floorplan

    # Hoisted above `make_state` (#797): the exclusive zones the seat predicate
    # gates on are built from these blocks, and `resolve_blocks` needs no
    # state. `notes` is filled from `block_problems` below, unchanged.
    blocks, block_problems = floorplan.resolve_blocks(
        intent, pcb_data, group_sources)

    state = pose_score.make_state(
        pcb_data, pcb_file, clearance=clearance,
        board_edge_clearance=board_edge_clearance, grid_step=grid_step,
        # #701: the declared keep-outs reach the SEAT PREDICATE here, and
        # every `_try_place` / `count_legal_poses` / `_evict_trade` site in
        # this module inherits them through `pose_ok`.
        keepouts=intent.keepouts if intent else (),
        # #797: and the declared EXCLUSIVE zones, the same way. Stage 3 is the
        # STRANGER path -- it passes no `constraint` at all, so before this
        # nothing asked whether the region it was aiming at belonged to
        # somebody else.
        exclusive_zones=(floorplan.zone_entries(intent, blocks)
                         if intent else ()),
        # #916. Reaches `pose_ok` through the state, which is the search
        # this issue is actually about. False by default.
        body_model=body_model)
    # #975: the grade a preferred edge seat is compared on (`_grade_worse`).
    # Reads nothing until a seat is short of the floor.
    pose_grader = floorplan.PoseGrader(
        intent, state, blocks=blocks, clearance=clearance,
        board_edge_clearance=board_edge_clearance)
    bounds = state.board
    refs_all = sorted(pcb_data.footprints)
    notes: List[str] = []

    for v in block_problems:
        notes.append(v.message)
    zones_by_name = {z.name: z for z in intent.blocks if z.rect is not None}

    lock_refs: List[str] = sorted({
        r for pat in intent.must_lock for r in refs_all if fnmatch.fnmatchcase(r, pat)})
    # #893. {ref: (declared rotation, declared candidates)}. NOTE these refs
    # are deliberately NOT added to `lock_refs`: that flag becomes
    # `_Part.locked`, one boolean covering position AND rotation, and
    # `place_seed` stamps it into the board -- so locking a part to hold its
    # angle would also freeze wherever the seeder first dropped it, which is
    # the very trade `_try_place`'s docstring told authors to accept for want
    # of anything better. The angle is held by handing `_try_place` a
    # one-element ladder instead.
    declared_rot = floorplan.rotations_for_ref(intent, blocks) if intent else {}
    # #1051: the declared rows, resolved against the board by the gate's own
    # resolver (members present, expected order). Empty unless declared.
    arrays_resolved = (floorplan.resolved_arrays(intent, pcb_data)
                       if intent is not None and getattr(intent, 'arrays', ())
                       else ())

    # Opt-in (OFF by default until `tests/test_placement_ab.py` pins its
    # rows): `_try_place` reads `state.rotation_prefer` and, when set, ranks
    # its rotation ladder by `_facing_rank`. Unset, the ladder is untouched.
    # #1099: the diagonal fallback pass in `_try_place`, and a decoupling
    # cap on a diagonal chip trying the chip's lattice first (stage 2.5).
    state.diagonal_fallback = (DIAGONAL_ROTATIONS_DEFAULT
                               if diagonal_rotations is None
                               else bool(diagonal_rotations))
    late_on = (DECAP_CLAIM_AFTER_ICS_DEFAULT if decap_claim_after_ics is None
               else bool(decap_claim_after_ics))
    if rotate_by_facing:
        import functools
        state.rotation_prefer = functools.partial(
            _facing_rank, state,
            edge_refs={c['ref'] for c in (intent.edge_claims() if intent else ())})
        notes.append('rotate-by-facing: the rotation ladder is ranked by '
                     'pads-facing-the-outline (placement.edge_facing) '
                     'before the first fit is kept')

    def _rot_why(ref):
        """Why a seat turned `ref` (#1113: a declared rotation used to be
        reported as a containment failure)."""
        claim = declared_rot.get(ref)
        if claim is None:
            return "(no contained pose at the input rotation)"
        return ("(its declared rotation)" if claim[0] is not None else
                "(the first of its declared rotation_candidates that fits)")

    def _rot_ladder(ref):
        """The declared ladder for `ref`, or None for the fallback one
        (`floorplan.declared_ladder`, shared with every seat search, #1117)."""
        return floorplan.declared_ladder(declared_rot.get(ref))

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
    # #975: stage 1's edge seats that keep pad copper inside the board-edge
    # floor because no pose the ladder tried clears it -- see `_floor_rung`.
    edge_floor_fallback: Dict[str, Dict] = {}
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

    # #1054: a fixed pose the seeder REFUSED. The part stays in `unplaced`
    # (so its pile coordinate never vetoes anyone's seat) but no later stage
    # may seat it: a refused mechanical fact placed "somewhere near" is the
    # nudge the refusal exists to prevent. Empty unless `fixed_poses` is.
    held: Set[str] = set()

    def _order(refs):
        return sorted((r for r in refs if r in unplaced and r not in held),
                      key=lambda r: (-state.parts[r].pin_count, tiebreak[r]))

    def _jitter():
        return (rng.uniform(-TARGET_JITTER_MM, TARGET_JITTER_MM),
                rng.uniform(-TARGET_JITTER_MM, TARGET_JITTER_MM))

    # ---- 0. fixed poses (#1054): the EXACT pose, checked, never searched ---
    # Before every other stage, so stage 1's edge ladder (`_shorted_by`, its
    # slide arming) and every later seat see these parts as placed obstacles.
    # Refs go to `fixed_lock` -> the returned `lock_refs` -> `stamp_locked`,
    # NOT into `lock_refs` here: that list drives stage 1.5's must_lock
    # re-seat and the eviction rung's 'must_lock' label, and a fixed pose is
    # deliberately not must_lock (docs/design-brief.md: filling must_lock
    # made `--repair` lift the user's locks). A file-locked part is outside
    # `--repair`'s reach and `--force`'s re-derivation alike, which is what
    # keeps a seated fixed pose where it was put.
    fixed_seated: Dict[str, Dict] = {}
    fixed_refused: Dict[str, Dict] = {}
    fixed_lock: Set[str] = set()
    _waiver_pairs = getattr(intent, 'courtyard_waiver_pairs', None)
    _seat_fixed_poses(state, pcb_data,
                      getattr(intent, 'fixed_poses', ()) or (), placed,
                      unplaced, held, fixed_seated, fixed_refused,
                      fixed_lock, notes,
                      waived=frozenset(frozenset(p) for p in
                                       (_waiver_pairs() if _waiver_pairs
                                        else ())))

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
    if by_edge:
        _floor_context_note(state, notes)
    for edge in sorted(by_edge):
        specs = sorted(by_edge[edge], key=lambda c: c['ref'])

        def _stage1_one(k, c, _member1125=None):
            # #1125: ONE stage-1 attempt at one connector, at the member
            # `_stage1_geometry_rot` picks, or at `_member1125` when the walk
            # below tries a later member of a candidate set. Returns
            # (outcome, the angle measured at): 'skip' -- wider than the edge
            # or outside its window, the two refusals `_stage1_fits` already
            # applied to every member; 'skip_late' -- refused after the turn;
            # 'crowded' -- seated at the crowding fallback; 'clean'.
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
            #
            # Everything measured below turns with the part -- its extents, the
            # offset from its origin to its courtyard centre, so the declared
            # start and window too -- and a DECLARED rotation is only applied
            # further down (#893). `_geo` is the part at the rotation this
            # stage will write, read-only, so a part skipped before that
            # block has not been turned (`_stage1_geometry_rot` says why this
            # matters). An angle the part already has is read exactly as
            # before, cache and all: only a declared one is materialised.
            _geo_rot = _stage1_geometry_rot(
                part, declared_rot.get(ref),
                fits=lambda r: _stage1_fits(state, part, c, bounds, edge, r))
            if _member1125 is not None:
                _geo_rot = _member1125
            if _geo_rot != part.rot:
                _geo_rot = _materialise_rotation(part, _geo_rot)
            _geo = _AtRotation(part, _geo_rot)
            _claim1120 = declared_rot.get(ref)
            if (_claim1120 is not None and _claim1120[0] is None
                    and _claim1120[1]
                    and not _stage1_fits(state, part, c, bounds, edge,
                                         _geo_rot)):
                notes.append(
                    f"edge connector {ref}: none of its declared "
                    f"rotation_candidates "
                    f"{[float(r) for r in _claim1120[1]]} fits the {edge} "
                    f"edge, so stage 1 leaves it, unturned, to the later "
                    f"stages -- which seat it only at one of them, or report "
                    f"it in rotation_unseated")
            f_lo, f_hi = _edge_frac_bounds(_geo, bounds, edge)
            # #706/#712. A DECLARED position outranks the even distribution.
            # Stage 1 is the from-scratch path and it never calls `_seat_edge`,
            # so without this a declared `center_on_edge` would be seated at
            # (k+1)/(n+1) -- on splitflap_driver's six north connectors that is
            # 1/7 .. 6/7 and NONE of them is 0.5 -- and `place_seed` would then
            # exit 4 grading its own output against the intent it was built
            # from. Inert when nothing is declared: `_declared_frac` is None.
            _dec = _declared_frac(c)
            # THE SAME TWO CONVERSIONS `_seat_edge` MAKES, and stage 1 needs
            # both for the same reasons. Without them this path seats a
            # declared claim wrong and `place_seed` exits 4 grading its own
            # output -- the very failure the comment above is about.
            #
            #  * the declared fraction is about the courtyard CENTRE and this
            #    ladder positions the ORIGIN. Measured on splitflap_driver
            #    J17 with `center_on_edge {tolerance_mm: 1.0}`: origin at
            #    frac 0.5 puts the rect centre at 0.51282, i.e. +2.54mm, a
            #    violation of any tolerance under 2.54mm;
            #  * the fraction is of the EDGE's span, and on a notched board
            #    that is not the bounding box -- the reason `edge_span`
            #    exists at all.
            _e_lo, _e_hi, _ = _declared_edge_span(state, bounds, edge)
            frac = ((declared_to_ladder_frac(_geo, bounds, edge,
                                             _e_lo, _e_hi, _dec))
                    if _dec is not None else (k + 1) / (len(specs) + 1))
            if f_lo > f_hi:
                notes.append(f"edge connector {ref}: wider than the {edge} "
                             f"edge, so stage 1 leaves it to the later stages")
                return 'skip', _geo_rot
            _win = _declared_frac_window(c, _e_hi - _e_lo)
            if _win is not None:
                _w_lo = declared_to_ladder_frac(_geo, bounds, edge,
                                                _e_lo, _e_hi, _win[0])
                _w_hi = declared_to_ladder_frac(_geo, bounds, edge,
                                                _e_lo, _e_hi, _win[1])
                _n_lo, _n_hi = max(f_lo, _w_lo), min(f_hi, _w_hi)
                if _n_lo > _n_hi:
                    notes.append(
                        f"edge connector {ref}: the declared along-edge window "
                        f"[{_win[0]:.3f}, {_win[1]:.3f}] does not intersect "
                        f"the legal one ["
                        f"{ladder_to_declared_frac(_geo, bounds, edge, _e_lo, _e_hi, f_lo):.3f}, "
                        f"{ladder_to_declared_frac(_geo, bounds, edge, _e_lo, _e_hi, f_hi):.3f}] "
                        f"on the {edge} edge, so stage 1 leaves it to the "
                        f"later stages")
                    return 'skip', _geo_rot
                f_lo, f_hi = _n_lo, _n_hi
            frac = min(f_hi, max(f_lo, frac))
            # #893. An edge connector is the class whose rotation is most often
            # a DECISION, and stage 1 never turns a part -- it seats at
            # `part.rot`. So without this a declared angle was simply ignored
            # here: not turned away silently, but seated at the INPUT angle,
            # which is the same broken promise wearing a different face. Set it
            # first, so `_edge_pose` and `_edge_correct` compute the overhang
            # and the correction for the geometry that will actually be
            # written. A candidate SET is applied as well (#1120): `_geo_rot`
            # is the member `_stage1_geometry_rot` chose -- the part's own
            # angle when that is a member that fits, else the first member
            # in the author's order that fits. It used not to be, and no
            # later stage re-seats an edge connector THIS stage seats, so one
            # whose input angle was outside its set was written at that input
            # angle, ungraded. One this stage skips is seated later, ladder
            # and all.
            _edge_decl = declared_rot.get(ref)
            if _edge_decl is not None:
                _want = (_edge_decl[0] if _edge_decl[0] is not None
                         else _geo_rot) % 360.0
                if abs((part.rot % 360.0) - _want) > 1e-9:
                    part.ensure_rotation(_want)
                    notes.append(
                        f"edge connector {ref}: seated at the declared "
                        f"rotation {_want:g}deg (input was {part.rot:g}deg)"
                        + (f", the first of its rotation_candidates "
                           f"{[float(r) for r in _edge_decl[1]]} "
                           + ("that fits " if _member1125 is None else
                              "that seats clear of what is placed on ")
                           + f"the {edge} edge" if _edge_decl[0] is None
                           else ''))
                    state.apply_move(ref, part.x, part.y, _want)
            # #701: SLIDE along the edge when a declared keep-out refuses the
            # even-distribution position, using the same ladder `_seat_edge`
            # already uses. Without it, one keep-out over the middle of an
            # edge sent the connector to the ordinary stages, which park it in
            # the board INTERIOR -- measured: J1 written at (11.22, 6.589) on
            # a board whose south edge is y=14, trading a `keepout` grade
            # error for an `edge_connector` one, while 26 clear south-edge
            # seats existed. The ladder is skipped entirely when nothing is
            # declared, so a board with no keep-out is unchanged.
            #
            # #797 arms it for a declared EXCLUSIVE ZONE too. `edge_seat_ok`
            # got the exclusive conjunct, but this ladder was still keyed on
            # keep-outs alone -- so identical geometry cost the connector its
            # declared edge when declared one way and not the other. Measured
            # on a 20x14 fixture with a rect over the WEST HALF of the south
            # band: as a keep-out J1 slid to (14.0, 12.5), still on the south
            # edge; as an exclusive zone it got one fraction, was refused, and
            # fell through to the ordinary stages, which parked it at
            # (7.489, 8.34) -- the board interior. Same rect, same free strip
            # to the east, two different answers. (Both numbers are arm E2's
            # own output; an earlier draft of this comment said 13.0, which
            # was never measured anywhere.)
            #
            # #706: armed for a DECLARED position too, not only a keep-out.
            # Two connectors declared at overlapping positions must slide off
            # each other rather than one being dropped to the later stages,
            # which park a connector in the board INTERIOR. Still skipped
            # entirely when nothing is declared and no keep-out exists, so a
            # board that declares nothing is unchanged.
            #
            # The four arming conditions are a UNION: #701's keep-out,
            # #706's declared position, #797's exclusive zone and run 27's
            # already-placed neighbour each need the ladder, and any one of
            # them alone leaves the others parking a connector in the
            # interior or on top of a fixed part.
            #
            # RUN 27, the fourth: `placed` at this point is the parts whose
            # pose is AUTHORITATIVE -- locked in the file, or outside an
            # explicit `seed_refs` scope -- and this stage seats a connector
            # without looking at any of them. Measured on esp_prog seeded
            # from a zone plan: CON2, declared on the south edge with band
            # 0.25-0.75, took the band's midpoint and put pin 1 through the
            # fixed USB socket's ground tab (0.198mm2 of pad intersection, a
            # short). Every one of ten seeds did it, the seed gate passed
            # them all because a pad conflict is not a budgeted channel, and
            # `check_assembly` then called each one NOT BUILDABLE. The band
            # had a clear seat the whole time -- run 26 found it by hand and
            # narrowed the declaration to 0.52-0.75 to force it.
            _slide = ((0.0,) if not (state.keepouts_for.get(ref)
                                     or _dec is not None
                                     or state.exclusive_for.get(ref)
                                     or placed) else
                      (0.0, 0.05, -0.05, 0.1, -0.1, 0.15, -0.15,
                       0.2, -0.2, 0.3, -0.3, 0.4, -0.4))

            def _shorted_by(px, py):
                """Already-placed refs this seat comes within clearance of.

                `pair_shortfall` measures a CLEARANCE shortfall, not contact,
                so a seat that merely crowds a placed part arms the slide too.
                Deliberately the wider predicate: the seat is free to move
                along its own band, so preferring a pose that is legal over
                one that is merely not-touching costs nothing.

                `placed`, never `state.parts`: a part still in the pile sits
                at one meaningless coordinate, and vetoing an honest edge
                seat against it is what `_seat_edge`'s `exclude` comment
                warns about -- the connector slides along the edge until one
                fraction is "free" and hangs off the end. Grows as this stage
                seats each connector, so two on one edge see each other.
                """
                ctx = state.legality_ctx
                if ctx is None:
                    return []
                hit = []
                for other in sorted(placed):
                    if other == ref or other not in state.parts:
                        continue
                    # The pose as it will be WRITTEN (`apply_move` rounds to
                    # 3dp below), so the predicate and the seat cannot differ
                    # by half a micron against a 1e-6 threshold.
                    sf = ctx.pair_shortfall(
                        ref, other,
                        pose_a=(round(px, 3), round(py, 3), part.rot))
                    if sf.pad > 1e-6 or sf.hole > 1e-6:
                        hit.append(other)
                return hit
            # SCALED to the declared window, exactly as `_seat_edge`'s ladder
            # is. Unscaled, a `center_on_edge {tolerance_mm: 1.0}` window on
            # splitflap's 198.12mm north edge is 0.0101 wide and every
            # +/-0.05 rung clamps to an end -- three distinct positions, the
            # defect `_seat_edge`'s `step` comment documents, in this path.
            _sstep = ((f_hi - f_lo) / 0.8) if _win is not None else 1.0

            def _s1_seats(sx, sy):
                # As `_seat_edge`'s `seats`, for `_band_settle` (#987) and
                # `_window_nudge` (#983): None when the pose is not a seat
                # clear of what is placed.
                _hi = float(hi) if hi is not None else max(2.0 * overhang, lo + 1.0)
                if (not edge_seat_ok(state, part, sx, sy, edge, lo, _hi)
                        or _shorted_by(sx, sy)):
                    return None
                return _seat_reading(state, part, ref, sx, sy, part.rot, sorted(placed),
                                     pose_grader, set(unplaced) - {ref})
            _base_frac = frac
            _why: List[str] = []
            # The FIRST rung that is a legal seat but crowds a placed part --
            # the declared position when rung 0.0 was legal, the nearest legal
            # rung to it otherwise. So a band with no clear seat anywhere
            # still gets its declared edge instead of being dropped to the
            # stages that park a connector in the interior. The conflict is
            # named on the record and `place_seed`'s gate refuses it; trading
            # the declared edge for it would lose both.
            _fallback = None
            # #975, the order of preference, tier by tier:
            #   1. a conflict-free rung, or that rung moved inward, whose pad
            #      copper clears the board-edge floor -- the first in rung
            #      order (`_pick`);
            #   2. the first conflict-free rung, which is what this ladder
            #      always kept (`_kept`), disclosed when it is short;
            #   3. the crowding `_fallback` above, unchanged: with no clear
            #      seat anywhere the floor is not searched, only reported.
            # A floor shortfall does not ARM the slide (an unarmed slide has
            # one rung and no later ones), a later rung is only walked to when
            # the first seat's shortfall is one a rung can change
            # (`_SLIDE_HELPS`), and any pose other than the first seat must
            # also pass the grade's edge conjuncts (`_grade_accepts`) and add
            # no intent-grade error to the first seat's (`_grade_worse`, with
            # the pile -- everything stage 1 has not placed -- left out).
            _kept = None
            _kept_xy = None
            _pick = None
            _graded = {}
            for _df in _slide:
                frac = min(f_hi, max(f_lo, _base_frac + _df * _sstep))
                _x, _y = _edge_pose(part, bounds, edge, frac, overhang)
                _x, _y, _conv = _edge_correct(
                    state, ref, edge, _x, _y, overhang,
                    band=(lo, float(hi) if hi is not None
                          else max(2.0 * overhang, lo + 1.0)))
                if _conv:
                    _rung = (_x, _y)
                    _x, _y = _band_settle(state, part, c, edge, lo, _x, _y, _s1_seats)
                    _x, _y = _window_nudge(state, part, c, edge, _x, _y, _s1_seats, _rung)
                _why = []
                if _conv and edge_seat_ok(state, part, _x, _y, edge, lo,
                                          float(hi) if hi is not None
                                          else max(2.0 * overhang, lo + 1.0),
                                          reasons=_why):
                    _hit = _shorted_by(_x, _y)
                    if not _hit:
                        if (_kept is not None and (_kept[2] or {}).get('why')
                                not in _SLIDE_HELPS):
                            break
                        _seat, _floor, _fwhy = _floor_rung(
                            state, part, c, edge, lo,
                            float(hi) if hi is not None
                            else max(2.0 * overhang, lo + 1.0),
                            _x, _y, lambda a, b: bool(_shorted_by(a, b)))
                        if _seat is not None and _kept is None and _seat == (_x, _y):
                            _pick = _seat
                            break
                        if _seat is not None and (
                                _kept is None or _seat != (_x, _y)
                                or _grade_accepts(state, part, c, edge, lo, _x, _y)):
                            # The pile is everything stage 1 has not placed.
                            _worse = _grade_worse(
                                pose_grader, ref, part.rot,
                                (_x, _y) if _kept is None else _kept_xy, _seat,
                                set(unplaced) - {ref}, _graded)
                            if not _worse:
                                _pick = _seat
                                break
                            if _kept is None:
                                _fwhy = dict(_fwhy or {}, why='grade_delta',
                                             n_grade_delta=len(_worse),
                                             grade_delta=list(_worse[:_FLOOR_ROWS]))
                        if _kept is None:
                            _kept = (frac, _floor, _fwhy)
                            _kept_xy = (_x, _y)
                        continue
                    if _fallback is None:
                        _fallback = (frac, _hit)
            # Without a pick the only early exit leaves `_kept` set, so this is
            # the old `for ... else`: the loop ran out, or tier 2 stopped it.
            if _pick is None:
                if _kept is not None:
                    frac = _kept[0]
                elif _fallback is not None:
                    frac, _hit = _fallback
                    notes.append(
                        f"edge connector {ref}: no seat on the {edge} band "
                        f"clears {', '.join(_hit)}, so it keeps the nearest "
                        f"legal seat to its declared position and the "
                        f"conflict is left for the gate -- narrow the band, "
                        f"or move what it crowds")
            x, y = _edge_pose(part, bounds, edge, frac, overhang)
            x, y, converged = _edge_correct(
                state, ref, edge, x, y, overhang,
                band=(lo, float(hi) if hi is not None
                      else max(2.0 * overhang, lo + 1.0)))
            if converged and _pick is not None:
                # The same rung, moved inward when that is what cleared it --
                # the walk above already checked this exact pose.
                x, y = _pick
            elif converged and _kept is not None:
                # Likewise the kept rung: the pose the ladder chose, not one
                # re-derived from `frac`, which would drop a #983 window step
                # or a #987 band settle. (The crowding fallback needs no such
                # line: neither correction moves a rung that does not seat.)
                x, y = _kept_xy
            if not converged:
                # The walk diverged (it drives a scalar SUM along one axis, so
                # an along-edge overshoot never cancels). It used to
                # apply_move unconditionally, which is how a diverged stage-1
                # seat reached the board silently.
                notes.append(f"edge connector {ref}: the overhang walk did "
                             f"not converge on the {edge} edge, so stage 1 "
                             f"left it for the later stages")
                return 'skip_late', _geo_rot
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
                return 'skip_late', _geo_rot
            state.apply_move(ref, round(x, 3), round(y, 3), part.rot)
            _missed = _window_miss_note(state, part, c, edge, 'edge connector ')
            if _missed:
                notes.append(_missed)
            placed.add(ref)
            unplaced.discard(ref)
            if _pick is None:
                if _kept is not None:
                    _record = _floor_record(ref, edge, 'conflict_free',
                                            (x, y, part.rot), _kept[1], _kept[2])
                else:
                    _record = _floor_record(
                        ref, edge, 'crowding', (x, y, part.rot),
                        _floor_at(state, ref, x, y, part.rot),
                        {'why': 'crowding'} if _fallback is not None else None)
                if _record is not None:
                    edge_floor_fallback[ref] = _record
                    notes.append(_floor_note('edge connector ', ref, _record))
            return ('crowded' if (_pick is None and _kept is None
                                  and _fallback is not None) else 'clean',
                    _geo_rot)

        def _stage1_undo(ref, pose, n0):
            """#1125: take back one stage-1 attempt -- its pose, its notes,
            its floor record and its seat -- so the next member is tried on
            the board the first one saw. The `bounds_by_rot` / `tht_by_rot`
            entries an attempt adds stay: they cache the part's own extents
            at an angle, the same whoever asks."""
            state.apply_move(ref, *pose)
            del notes[n0:]
            edge_floor_fallback.pop(ref, None)
            placed.discard(ref)
            unplaced.add(ref)

        for k, c in enumerate(specs):
            # #1125: a candidate SET is walked by the SEAT each member gets,
            # not only by whether it fits. The member `_stage1_geometry_rot`
            # picks is tried first, exactly as before; only when that seat
            # crowds what is placed (or is refused after the turn) are the
            # set's other fitting members tried, in the same order, and the
            # first that seats clear is kept. When none does, the first
            # member that SEATS at all is made again -- a crowded seat on its
            # declared edge beats the interior the later stages would park it
            # in, stage 1's own crowding-fallback rule -- and when none
            # seats, attempt 1 is, which leaves the part to the later stages
            # exactly as before. splitflap's J5 declared [180, 90] kept 180,
            # which only crowds J17, where 90 seats clear.
            _ref1125 = c['ref']
            _part1125 = state.parts[_ref1125]
            _pose1125 = (_part1125.x, _part1125.y, _part1125.rot)
            _n1125 = len(notes)
            _out, _used = _stage1_one(k, c)
            _claim = declared_rot.get(_ref1125)
            if (_out not in ('crowded', 'skip_late') or _claim is None
                    or _claim[0] is not None or not _claim[1]):
                continue
            _tried = [_used]
            _won = None
            _crowded = _used if _out == 'crowded' else None
            while True:
                _next = _stage1_walk_member(
                    _part1125, _claim, _tried,
                    fits=lambda r, _p=_part1125, _c=c: _stage1_fits(
                        state, _p, _c, bounds, edge, r))
                if _next is None:
                    break
                _stage1_undo(_ref1125, _pose1125, _n1125)
                _tried.append(_next)
                _o, _u = _stage1_one(k, c, _member1125=_next)
                if _o == 'clean':
                    _won = _u
                    break
                if _o == 'crowded' and _crowded is None:
                    _crowded = _u
            if _won is not None:
                notes.append(
                    f"edge connector {_ref1125}: its rotation_candidates "
                    f"member {_used:g}deg "
                    + ("only crowded what is placed" if _out == 'crowded'
                       else "was refused after the turn")
                    + f", so stage 1 seated it at {_won:g}deg, the next "
                    f"member that seats clear on the {edge} edge (#1125)")
                continue
            if len(_tried) > 1:
                _stage1_undo(_ref1125, _pose1125, _n1125)
                if _crowded is not None and _crowded != _used:
                    _stage1_one(k, c, _member1125=_crowded)
                else:
                    _stage1_one(k, c)
                _set = [float(r) for r in _claim[1]]
                notes.append(
                    f"edge connector {_ref1125}: no member of its "
                    f"rotation_candidates {_set} seats clear on the {edge} "
                    f"edge, so "
                    + (f"it keeps the crowded seat of {_crowded:g}deg, the "
                       f"first member that seats there at all (#1125)"
                       if _crowded is not None else
                       f"none seats there and it is left to the later "
                       f"stages (#1125)"))

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
                         rotations=_rot_ladder(ref))
        if clr is None and z is not None:
            zx = (z.rect[0] + z.rect[2]) / 2.0
            zy = (z.rect[1] + z.rect[3]) / 2.0
            clr = _try_place(state, ref, zx, zy, unplaced - {ref},
                             constraint=rect, tol=tol, info=info,
                             rotations=_rot_ladder(ref))
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
        # NARROWED by #792 to the caps that ELECT A TETHER at any distance --
        # `near | beyond`, i.e. the scope minus the caps whose rail no chip
        # carries.
        #
        # It used to be a syntactic test (`r[0] == 'C'` and two net-bearing
        # pads) -- a third spelling of the grouper's predicate and, more to the
        # point, an answer to the WRONG QUESTION. This stage seats a cap AT A
        # CHIP'S PIN. A cap whose rail no >=4-pad non-collinear part touches
        # has no such pin, ever, on any board, at any run time -- so evicting
        # it from zone packing below is a promise the stage is structurally
        # incapable of keeping. It then fell through to the generic centroid
        # stage, whose fanout cap nulls out a rail net, and landed near the
        # board middle.
        #
        # Measured, ulx3s: ten caps -- C3 C4 C22 on /power/P1V1, C7 C8 C24 on
        # /power/P3V3, C11 C12 C23 on /power/P2V5, C14 on /power/SHUT. Those
        # rails are owned only by two-pad passives (the caps, L1-L3, RA*/RP*),
        # so they are bulk and filter caps upstream of an LC network, not
        # decouplers. cap_chain 2 of 2, flat_hierarchy 3, watchy 2.
        #
        # The three syntactic spellings name the SAME parts on every tracked
        # board (`tests/test_792_decap_predicate.py`), so unifying them would
        # have preserved this bug rather than fixed it. The defect was never
        # the spelling; it was one predicate asked two different questions.
        from placement import groups as _groups
        near, beyond, _orphans = _groups.decap_populations(pcb_data)
        tethered = ({c for caps in near.values() for c, _d in caps}
                    | {c for c, _ic, _d in beyond})
        decap_scope = {r for r in tethered
                       if r in state.parts
                       and not any(fnmatch.fnmatchcase(r, pat) for pat in exempt)}

    # ---- #1051: which declared rows stage 2.45 will seat ----------------------
    # Decided HERE, before stage 2, because two earlier stages must step
    # aside for a row: stage 2 does not zone-pack the members of a row whose
    # members all sit in one zoned block (the row is seated into that zone
    # whole, through `zone_gate`), and the decap pin stage does not claim a
    # row member (the array wins; disclosed in `decap_stage`). A row the
    # intent check refuses (`floorplan.array_problems`: a missing or
    # file-locked member, mixed footprints, a rotation or zone conflict), or
    # one with a member already placed, is not attempted; its members are
    # ordinary parts and the reason is in `array_unseated`.
    arrays_formed: Dict[str, Dict] = {}
    array_unseated: Dict[str, Dict] = {}
    array_try: List[Dict] = []
    array_zone: Dict[str, object] = {}
    array_members: Set[str] = set()
    decap_array_skipped: List[str] = []
    if arrays_resolved:
        _aprobs: Dict[str, List[str]] = {}
        for _v in floorplan.array_problems(intent, pcb_data, blocks):
            _aprobs.setdefault(str(_v.block), []).append(_v.message)
        for spec in arrays_resolved:
            _an = spec['name']
            _am = list(spec['present'])
            if _an in _aprobs:
                array_unseated[_an] = {
                    'members': _am, 'poses_tried': 0, 'capped': False,
                    'reason': ('refused by the intent check: '
                               + '; '.join(_aprobs[_an]))}
            elif any(m not in unplaced or m in held for m in _am):
                _gone = [m for m in _am if m not in unplaced or m in held]
                array_unseated[_an] = {
                    'members': _am, 'poses_tried': 0, 'capped': False,
                    'reason': (f"member(s) {', '.join(_gone)} already placed "
                               f"(locked in the file or outside the seed "
                               f"scope) -- a row is seated as one piece")}
            else:
                _zs = [zn for zn in sorted(zones_by_name)
                       if _am[0] in blocks.get(zn, ())]
                if _zs:
                    array_zone[_an] = zones_by_name[_zs[0]]
                array_try.append(spec)
                array_members.update(_am)
                continue
            notes.append(f"array {_an}: not seated as a row -- "
                         f"{array_unseated[_an]['reason']}; its members are "
                         f"seated one by one")
        decap_array_skipped = sorted(decap_scope & array_members)
        if decap_array_skipped:
            decap_scope -= array_members
            notes.append(f"decap stage: {len(decap_array_skipped)} cap(s) "
                         f"are declared array members, and the array wins -- "
                         f"the pin stage skips "
                         + ', '.join(decap_array_skipped))
    array_zoned_members = {m for spec in array_try
                           if spec['name'] in array_zone
                           for m in spec['present']}
    center = ((bounds[0] + bounds[2]) / 2.0, (bounds[1] + bounds[3]) / 2.0)

    # ---- 2. zoned blocks: radial pack from the zone center -----------------
    # A single-member zone is the spec-coordinate pattern (a rect a few
    # hundred microns wide around where the spec pins the part), so it gets
    # the exact center; multi-member zones jitter each target so different
    # seeds pack differently.
    for name in sorted(zones_by_name):
        z = zones_by_name[name]
        # #1051: a row zoned HERE is seated before the zone's other members
        # -- a row needs a contiguous strip, and the radial pack fills the
        # zone around it (measured on glasgow: RN5+RN6 seated into a packed
        # sheet zone only at clearance 0.1, after 243k poses; into the
        # empty one first). It aims at `_seat_array`'s target (its members'
        # placed partners, else the served pins, else the board centre),
        # clamped into the zone.
        for spec in array_try:
            if getattr(array_zone.get(spec['name']), 'name', None) == name:
                _seat_array(state, pcb_data, intent, spec, z, placed,
                            unplaced, center, _rot_ladder, array_pose_cap,
                            arrays_formed, array_unseated, notes)
        members = [r for r in _order(blocks.get(name, ()))
                   if r not in decap_scope and r not in array_zoned_members]
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
                             rotations=_rot_ladder(ref),
                             constraint=z.rect, tol=tol, info=zinfo)
            if clr is not None:
                placed.add(ref)
                unplaced.discard(ref)
                if zinfo.get('anchor_zone'):
                    notes.append(f"{ref}: zone {name!r} smaller than the "
                                 f"courtyard -- seated by anchor point")
                if state.parts[ref].rot != rot_before:
                    notes.append(f"{ref}: rotated {rot_before:g} -> "
                                 f"{state.parts[ref].rot:g} "
                                 + _rot_why(ref))
                if clr < state.clearance:
                    notes.append(f"{ref}: placed at reduced courtyard "
                                 f"clearance {clr:g} (none at "
                                 f"{state.clearance:g})")
            else:
                unseated.append(ref)
                unseated_ctx[ref] = (cx + jx, cy + jy, z.rect, tol)
                notes.append(f"{ref}: no legal pose inside zone {name!r}")

    # ---- the connectivity-centroid seat, shared by 2.4 and 3 --------------
    # ONE body, so stage 2.4 seats an IC exactly as stage 3 would have, only
    # earlier (#1053: "the two paths cannot diverge"). The target, the jitter
    # draw, the ladder and the notes are stage 3's, in stage 3's order.

    def _centroid_seat(ref, jit=None):
        """`(clearance or None, target, jx, jy)`; seats on success. `jit`
        is a jitter drawn earlier for this ref (stage 3's `q_jit`); None
        draws it here, as stage 2.4 does."""
        target = _partner_centroid(state, ref, placed) or center
        jx, jy = _jitter() if jit is None else jit
        rot_before = state.parts[ref].rot
        clr = _try_place(state, ref, target[0] + jx, target[1] + jy,
                         unplaced - {ref},
                         rotations=_rot_ladder(ref))
        if clr is not None:
            placed.add(ref)
            unplaced.discard(ref)
            if state.parts[ref].rot != rot_before:
                notes.append(f"{ref}: rotated {rot_before:g} -> "
                             f"{state.parts[ref].rot:g} " + _rot_why(ref))
            if clr < state.clearance:
                notes.append(f"{ref}: placed at reduced courtyard clearance "
                             f"{clr:g} (none at {state.clearance:g})")
        return clr, target, jx, jy

    # ---- 2.4 declared rows: each row's served part, then the row ---------
    # A non-zoned declared row aims at the part it serves, so that part is
    # seated first, in STAGE 3's order and with stage 3's own seat
    # (`_centroid_seat`: "the two paths cannot diverge"), and each row at its
    # members' rank in that order. Zoned rows are seated in stage 2.
    #
    # A ROW MEMBER is never seated alone here, whatever else it is. A part
    # that finds no seat is left to stage 3, which reports it. No non-zoned
    # row declared: skipped, and the seed is bit-identical.
    served_first: List[str] = []
    early_order: List[str] = []
    rows_early = [sp for sp in array_try if sp['name'] not in array_zone]
    if rows_early:
        # Only rows this seed will actually seat: a refused row's host is
        # an ordinary part and keeps its stage-3 turn.
        want24: Set[str] = {str(a['serves']) for a in rows_early
                            if a.get('serves') not in (None, 'unknown')}
        # A ROW MEMBER is never seated alone here, even when it is another
        # row's host: it is seated with its row.
        want24 -= array_members
        # (key, kind, payload): parts in `_order`'s key, rows at their rank.
        items = [((-state.parts[r].pin_count, tiebreak[r]), 0, r)
                 for r in _order(sorted(r for r in want24
                                        if r in state.parts))]
        items += [((-max(state.parts[m].pin_count for m in sp['present']),
                    tiebreak[sp['present'][0]]), 1, k)
                  for k, sp in enumerate(rows_early)]
        items.sort(key=lambda it: (it[0], it[1]))
        for _key, kind, what in items:
            if kind == 1:
                sp = rows_early[what]
                early_order.append(f"array:{sp['name']}")
                _seat_array(state, pcb_data, intent, sp, None, placed,
                            unplaced, center, _rot_ladder, array_pose_cap,
                            arrays_formed, array_unseated, notes)
                continue
            early_order.append(what)
            clr, _t, _jx, _jy = _centroid_seat(what)
            if clr is not None:
                served_first.append(what)
            else:
                notes.append(f"{what}: stage 2.4 (a declared row's served "
                             f"part) found no seat -- left to the centroid "
                             f"stage")
        if served_first:
            notes.append(f"stage 2.4: seated {len(served_first)} part(s) "
                         f"that declared rows serve, before the decap/array "
                         f"stages (" + ', '.join(served_first) + ")")

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
    # #1053: what the pin stage did, and WHY when it claimed nothing. A
    # silent zero is the defect this record exists to end.
    decap_claimed: List[str] = []
    decap_put_back: List[str] = []
    decap_pins = 0
    decap_rails = 0
    # #1105: the owners stage 2.5 served, so stage 3.5 never serves a pin
    # twice.
    decap_owners_early: Set[str] = set()
    from placement import groups as _g
    # Built ONCE: `chip_refs` walks every footprint's pads, the owner loop
    # runs per placed part, and stage 3.5 asks again.
    chips = (_g.chip_refs(pcb_data) if decap_owner_chips and decap_scope
             else None)
    zone_of_cap = {}
    for name in sorted(zones_by_name):
        for r in blocks.get(name, ()):
            if r in decap_scope and r not in zone_of_cap:
                zone_of_cap[r] = zones_by_name[name]

    def _decap_pin_claim(owner_pool, claimed, tag='', decline_beyond=None,
                         declined=None):
        """Seat one scoped cap per supply pin of the owner ICs in
        `owner_pool`, appending each to `claimed`; returns `(avail, pins,
        rails, the owners whose pins it found)`. Stage 2.5 runs it over the
        parts placed before it, stage 3.5 (#1105) over the owner ICs the
        centroid stage seated since. `tag` marks which stage wrote a note.
        `decline_beyond` (stage 3.5 under `DECAP_LATE_WITHIN_LIMIT`) undoes a
        seat whose cap lands farther than that from its IC as the GRADE
        measures it (`decap_graded_distance`: pad centroid to the elected
        chip's pad box, over the placed chips on its rail), adding the cap to
        `declined`: it keeps its own centroid turn instead."""
        _last = {'declined': False}   # did the latest `_seat` decline?
        _rail_chips: Dict[str, List[str]] = {}   # cap -> its rail's chips
        avail = [r for r in _order(sorted(unplaced)) if r in decap_scope]
        rail_of: Dict[str, int] = {}
        for ref in avail:
            rail = _decap_rail(state.parts[ref].nets, state.net_refs)
            if rail is not None:
                rail_of[ref] = rail
        rails = set(rail_of.values())
        pins: List[Tuple[int, str, float, float, int]] = []
        # WHAT IS AN IC gets one answer, and it is the grouper's (#792).
        # `owner[0] != 'U'` and `groups._pads_are_collinear` are two answers to
        # the SAME documented problem -- "a castellated row carries the rail
        # too and must not eat a claim" -- and only the grouper's carries the
        # measurement behind it ("Measured on one board it captured three, and
        # the grader then reported 'C12 is 3.30mm from CN2, the IC it
        # decouples'"). A ref prefix also refuses a real IC for its NAME:
        # measured, caps whose rail gains a pin source under the grouper's
        # answer -- watchy +8, kit-dev +5, lvds +4, orangecrab +3, ulx3s +2,
        # glasgow +1, tigard +1, and zero losses IN THAT METRIC. The new
        # sources are IC*, J*, VR*, SD* and a crystal: exactly the parts
        # `decap_tethers` already tethers caps to, so this makes the seeder
        # AGREE with the grader.
        #
        # "Zero losses" is true of pin SOURCES and false of the outcome,
        # and the first draft of this comment said the former while
        # meaning the latter. Measured on the rows this PR commits, the
        # widened arm STRANDS four parts across three boards that the
        # control seats (orangecrab U4, rp2350 L1, tigard H1 and H3) and
        # worsens `pin_gap_sum` on glasgow (346.63 -> 404.80) and rp2350
        # (86.99 -> 91.30). Coverage is not seating. That is why the flag
        # is OFF, and `test_792_seeding_claims.py` asserts the stranding
        # so it cannot be flipped on without confronting it.
        #
        # Behind a flag until the A/B rows run, following `evict_depth`'s
        # precedent: it changes where parts go, and this file's own rule is
        # that such a change is opt-in until three boards say otherwise.
        for owner in sorted(owner_pool):
            if not _decap_owner_ok(owner, chips):
                continue
            o = state.parts[owner]
            for gx, gy, pn in o.pad_globals():
                if pn in rails:
                    pins.append((-o.pin_count, owner, round(gx, 3),
                                 round(gy, 3), pn))
        pins.sort()

        def _cap_ladder(ref, owner):
            """#1099: a cap on a chip seated OFF the 90-degree lattice tries
            the chip's own lattice first -- StickHub's U1 at -135 degrees
            has every strap resistor and decap at +-45/+-135 -- then its own
            orthogonal one. A declared ladder wins; a chip on the lattice
            leaves the default search untouched."""
            declared = _rot_ladder(ref)
            if declared is not None or not state.diagonal_fallback:
                return declared
            o = state.parts.get(owner)
            if o is None or abs(((o.rot % 90.0) + 45.0) % 90.0 - 45.0) < 1e-6:
                return None
            p = state.parts[ref]
            chip = [(o.rot + d) % 360 for d in (0.0, 90.0, 180.0, 270.0)]
            own = [(p.rot + d) % 360 for d in (0.0, 90.0, 180.0, 270.0)]
            return chip + [r for r in own if r not in chip]

        def _seat(ref, tx, ty, owner, pn, constraint=None, tol=0.5):
            _last['declined'] = False
            _el = _off = None
            _ladder = _cap_ladder(ref, owner)
            _was = (state.parts[ref].x, state.parts[ref].y,
                    state.parts[ref].rot)
            clr = _try_place(state, ref, tx, ty, unplaced - {ref},
                             constraint=constraint, tol=tol,
                             rotations=_ladder)
            if clr is None and constraint is not None:
                clr = _try_place(state, ref, tx, ty, unplaced - {ref},
                                 rotations=_ladder)
            if clr is None:
                return False
            if decline_beyond is not None:
                if ref not in _rail_chips:
                    _rail_chips[ref] = _g.rail_chips(pcb_data, ref)
                _el, _off = decap_graded_distance(
                    pcb_data, state, ref, _rail_chips[ref], placed)
                if _off is None:
                    _off = 0.0    # no placed chip on its rail: no tether to grade
                if _off > decline_beyond:
                    state.apply_move(ref, *_was)
                    _last['declined'] = True
                    if ref not in declined:
                        declined.append(ref)
                    notes.append(
                        f"{ref}: stage 3.5 declined its seat for {owner} -- "
                        f"it landed {_off:.2f}mm from {_el} as the grade "
                        f"measures it, past the {decline_beyond:g}mm decap "
                        f"limit")
                    return False
            avail.remove(ref)
            placed.add(ref)
            unplaced.discard(ref)
            claimed.append(ref)
            if declined and ref in declined:
                declined.remove(ref)    # declined at one pin, claimed at another
            p2 = state.parts[ref]
            net = getattr(pcb_data.nets.get(pn), 'name', pn)
            notes.append(f"{ref}: decap for {owner} pad(s) near ({tx}, {ty})"
                         f" [{net}], landed "
                         f"{math.hypot(p2.x - tx, p2.y - ty):.2f}mm"
                         + (f" at reduced clearance {clr:g}"
                            if clr < state.clearance else "")
                         + (f", {_off:.2f}mm from {_el} as graded"
                            if _el is not None else "") + tag)
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
                if not _seat(ref, cx2, cy2, cluster[0][1], rail):
                    # SILENTLY lost before #792. `caps_r` is snapshotted above
                    # while `_seat` mutates `avail`, so the zip consumes the
                    # cluster whether or not the seat succeeded: the cap stayed
                    # in `avail`, the pins went unserved, and NOTHING said so --
                    # while the comment below claimed the fall-through "reports
                    # honestly". It does now.
                    if _last['declined']:
                        continue    # #1105: `_seat` said why
                    net = getattr(pcb_data.nets.get(rail), 'name', rail)
                    notes.append(
                        f"{ref}: no legal pose at {cluster[0][1]}'s {net} pin "
                        f"cluster ({cx2}, {cy2}) -- falls through to the "
                        f"zone/centroid stages" + tag)
            # ...and the caps NO CLUSTER WANTED. `zip` truncates to the
            # shorter list, so a cap past the cluster count is never reached
            # by the loop above and was dropped without a word -- a SECOND
            # silent path, distinct from the `_seat` failure, and the one the
            # comment below has always described as reporting honestly.
            # Measured on a three-caps-two-pins fixture: the third cap is
            # seated by the generic stage and nothing said the pin stage had
            # passed it over.
            for ref in caps_r[len(clusters):]:
                if ref in avail:
                    net = getattr(pcb_data.nets.get(rail), 'name', rail)
                    # WORDED so it does not contain "falls through". The first
                    # draft ended with the same phrase the `_seat`-failure note
                    # uses, and the arm that matched on that substring then
                    # accepted EITHER note as coverage -- which silently
                    # un-tested the 2.6 put-back: a battery row that had been
                    # KILLED started SURVIVING, and the headline #792 fix
                    # became deletable with every arm still green. Two paths,
                    # two texts, so an arm can name exactly one.
                    notes.append(
                        f"{ref}: no {net} pin cluster left for it "
                        f"({len(clusters)} cluster(s), {len(caps_r)} cap(s)) "
                        f"-- left to the zone/centroid stages" + tag)
            # pins beyond the cap supply, and caps no pin wanted, fall
            # through to the generic stage, which reports honestly
        return avail, len(pins), len(rails), sorted({k[1] for k in pins})

    if decap_scope:
        decap_owners_early = set(placed)
        avail, decap_pins, decap_rails, _owners25 = _decap_pin_claim(
            decap_owners_early, decap_claimed)

        # ---- 2.6 put back what the pin stage DECLINED (#792) ---------------
        # Narrowing the scope is the predictive half and it is a theorem; this
        # is the run-time safety net, and it is much the bigger of the two.
        # Measured with `seed_from_intent`: splitflap_driver has 12 tethered
        # caps and this stage claims ZERO; watchy 26 tethered, ZERO; kit-dev 53
        # in scope, 12 claimed. The residue the stage declines at run time --
        # the owner not yet placed, the cap supply exhausted, `_seat` refusing
        # -- dwarfs the orphan population.
        #
        # NOT "re-run stage 2": that path appends a failure to `unseated`
        # (see above), and UNSEATED is worse than the board centre. This is
        # `_try_place` inside the declared zone, falling through to stage 3 on
        # refusal, which makes the whole put-back MONOTONE -- no cap can end up
        # worse placed than it is today, and none can become newly unseated.
        # That is what lets it ship on without an A/B row.
        for ref in _order(sorted(unplaced & decap_scope)):
            z = zone_of_cap.get(ref)
            if z is None or not getattr(z, 'rect', None):
                continue
            zx0, zy0, zx1, zy1 = z.rect
            clr = _try_place(state, ref, round((zx0 + zx1) / 2.0, 3),
                             round((zy0 + zy1) / 2.0, 3), unplaced - {ref},
                             constraint=z.rect,
                             tol=intent.zone_tolerance(z),
                             rotations=_rot_ladder(ref))
            if clr is None:
                notes.append(f"{ref}: the pin stage declined it and its zone "
                             f"{z.name!r} has no legal pose either -- falls "
                             f"through to the centroid stage")
                continue
            placed.add(ref)
            unplaced.discard(ref)
            if ref in avail:
                avail.remove(ref)
            decap_put_back.append(ref)
            notes.append(f"{ref}: zone-packed into {z.name!r} after the pin "
                         f"stage declined it")

    if decap_spec.get('max_distance_mm') is None:
        decap_stage = {'armed': False, 'scope': 0, 'claimed': 0,
                       'reason': 'decaps.max_distance_mm is not declared'}
    else:
        _why = None
        if not decap_scope:
            _why = 'no cap in scope elects a tether (or every one is exempt)'
        elif not decap_claimed:
            if not decap_rails:
                _why = 'no cap in scope carries a rail net'
            elif not decap_pins:
                _why = ("no PLACED IC carries a scoped cap's rail (pins 0) "
                        "-- no owner IC is seated before this stage (a "
                        "fixed pose, must_lock, a zoned block or a declared "
                        "row's `serves` seats one earlier)"
                        + ('' if decap_owner_chips else
                           '; owners must be U-prefixed unless '
                           'decap_owner_chips'))
            else:
                _why = (f"{decap_pins} pin(s) found, and no cap found a "
                        f"legal seat at any of them")
        decap_stage = {'armed': True, 'scope': len(decap_scope),
                       'claimed': len(decap_claimed),
                       'put_back': len(decap_put_back),
                       'pins': decap_pins, 'reason': _why,
                       'served_first': list(served_first),
                       'array_members_skipped': list(decap_array_skipped)}
        if decap_scope and not decap_claimed:
            notes.append(f"decap stage 2.5: {len(decap_scope)} cap(s) in "
                         f"scope, 0 claimed at a supply pin -- {_why}. "
                         + ("Stage 3.5 retries the claim once the centroid "
                            "stage has seated their owner ICs" if late_on
                            else "They fall through to the centroid stage"))

    # ---- 3. the rest: connectivity centroid --------------------------------
    # --anchors-first (run-4 C): the default queue is pin-count descending,
    # which seeds a LARGE low-pin part (a connector shell, a big switch)
    # late, after the smalls have claimed its space -- and "nothing in the
    # placement code orders by size" was the skill's own measured complaint.
    # The mode seeds the anchor tier (pad-extent >= the P75 threshold, the
    # same tiering reconstruct uses) by DESCENDING EXTENT first; the smalls
    # are already parked as non-obstacles by the existing `exclude` set, so
    # anchors place against anchors only. Everything else is unchanged.
    queue = _order(sorted(unplaced))
    if anchors_first and unplaced:
        from placement.reconstruct import part_extent_mm
        exts = sorted(part_extent_mm(state, r) for r in unplaced)
        thr = max(3.5, exts[int(0.75 * (len(exts) - 1))]) if exts else 3.5
        # `held` (#1054): a REFUSED fixed pose stays in `unplaced` so its
        # pile coordinate never vetoes a seat, and must not become an anchor
        # -- this queue is built from `unplaced` directly, not `_order`.
        # Ties broken by ref: `unplaced` is a SET, and equal-extent parts
        # (identical footprints, the common case) otherwise came out in
        # string-hash order -- a different seed per PYTHONHASHSEED.
        anchors = sorted((r for r in unplaced
                          if r not in held
                          and part_extent_mm(state, r) >= thr),
                         key=lambda r: (-part_extent_mm(state, r), r))
        notes.append(f"anchors-first: {len(anchors)} anchor(s) (extent >= "
                     f"{thr:.2f}mm) seed before {len(unplaced) - len(anchors)}"
                     f" small(s): {', '.join(anchors)}")
        queue = anchors + [r for r in queue if r not in set(anchors)]
    # ---- 3.5 (#1105): the pin claim again, once its owner ICs are seated ---
    # On a pile or a flat board stage 2.5 finds no placed owner IC, so it
    # claims nothing and every decap was left to the centroid seat below. The
    # claim runs again HERE, at the first scoped cap after the last queue
    # entry that can own one of their rails, over the owner ICs this stage
    # seated (never one 2.5 served: the pools are disjoint, so no pin is
    # served twice). It draws no RNG, so every part seated before it is
    # bit-identical to the stage-off seed; a cap it declines keeps its own
    # turn in the queue. Anchors-first can put a big cap ahead of the ICs,
    # which is why the trigger is "after the last owner", not "the first cap".
    late_claimed: List[str] = []
    late: Dict[str, Any] = {'armed': late_on, 'at': None, 'owners': [],
                            'pins': 0, 'caps': [], 'claimed': 0,
                            'declined': [], 'reason': None}
    late_left = len(unplaced & decap_scope)
    late_from = None
    # #1105: every queue entry's jitter, drawn HERE in queue order and before
    # any reorder. A cap stage 3.5 claims skips its centroid turn, and
    # `after_queue` moves the caps to the end; drawing at the turn shifted the
    # RNG stream for every part after either, so the stage's A/B measured a
    # re-roll of their targets as well as the claim. With the stage off every
    # entry reaches its turn in this order, so the draws are the ones the
    # inline `_jitter()` made -- the same values, bit for bit.
    q_jit = {r: _jitter() for r in queue}
    if late_on and late_left and DECAP_LATE_AT == 'after_queue':
        queue = ([r for r in queue if r not in decap_scope]
                 + [r for r in queue if r in decap_scope])
    if late_on and late_left:
        _late_rails = {_decap_rail(state.parts[r].nets, state.net_refs)
                       for r in queue if r in decap_scope} - {None}
        _own = [i for i, r in enumerate(queue) if _decap_owner_ok(r, chips)
                and any(n in _late_rails for n in state.parts[r].nets)]
        late_from = (_own[-1] + 1) if _own else None
    for i, ref in enumerate(queue):
        if (late_from is not None and late['at'] is None and i >= late_from
                and ref in decap_scope):
            late['at'] = ref
            late_left = len(unplaced & decap_scope)
            _a, late['pins'], _r, late['owners'] = _decap_pin_claim(
                set(placed) - decap_owners_early, late_claimed,
                tag=' (stage 3.5)',
                decline_beyond=(float(decap_spec['max_distance_mm'])
                                if DECAP_LATE_WITHIN_LIMIT else None),
                declined=late['declined'])
        if ref not in unplaced:
            continue    # #1105: claimed by stage 3.5 just above
        clr, target, jx, jy = _centroid_seat(ref, jit=q_jit[ref])
        if clr is None:
            unseated.append(ref)
            # setdefault: a zone member that failed its zone stage keeps THAT
            # context, so the rung retries it inside its zone rather than at
            # this unconstrained centroid (a seat outside the zone would fail
            # the grade anyway).
            unseated_ctx.setdefault(
                ref, (target[0] + jx, target[1] + jy, None, 0.5))
            notes.append(f"{ref}: no legal pose anywhere on the board")
    if decap_stage['armed']:
        late['caps'] = list(late_claimed)
        late['claimed'] = len(late_claimed)
        if not late_on:
            late['reason'] = 'off -- place_seed --decap-claim-after-ics arms it'
        elif not late_left:
            late['reason'] = ('every scoped cap was claimed or put back '
                              'before the centroid stage')
        elif late_from is None:
            late['reason'] = ("no owner IC carrying a remaining cap's rail is "
                              "in the centroid stage's queue (owners must be "
                              "U-prefixed unless decap_owner_chips)")
        elif late['at'] is None:
            late['reason'] = ('no scoped cap reaches the centroid stage after '
                              'its last owner IC')
        elif not late['pins']:
            late['reason'] = ("the owner IC(s) the centroid stage seated carry "
                              "no remaining cap's rail (pins 0)")
        elif not late_claimed and late['declined']:
            late['reason'] = (f"every seat found at the {late['pins']} pin(s) "
                              f"landed past the decap limit and was declined")
        elif not late_claimed:
            late['reason'] = (f"{late['pins']} pin(s) found, and no cap found "
                              f"a legal seat at any of them")
        decap_stage['late'] = late
        if late_on and late_left:
            notes.append(
                f"decap stage 3.5: {len(late_claimed)} of {late_left} cap(s) "
                f"left in scope claimed at a supply pin of the owner IC(s) "
                f"the centroid stage seated ("
                + (', '.join(late['owners']) or 'none') + ")"
                + (f", before {late['at']}'s turn" if late['at'] else "")
                + (f" -- {late['reason']}" if late['reason'] else "")
                + ". The rest keep their centroid-stage turn")

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
        # #1054: a seated fixed pose is a fact, not a neighbour to trade.
        immovable.update({r: 'fixed_pose' for r in fixed_seated
                          if r not in immovable})
        # #1051: a formed row's member moves only with its row; evicting one
        # alone would break the row the seed just formed, silently.
        immovable.update({m: f"array:{n}" for n, rec in arrays_formed.items()
                          for m in rec['members'] if m not in immovable})
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
            # #797: which declared EXCLUSIVE zone is refusing this part,
            # measured the same way. A SIBLING of the keep-out sweep above and
            # never nested inside it: a stranger can be refused by a reserved
            # zone on a board that declares no keep-out anywhere, and nesting
            # would make this channel depend on that one being non-empty.
            #
            # Same cost model, and the same reasons it is affordable: only for
            # a part with no pose at all, only over the zones that BIND it, and
            # every census is already capped at CENSUS_CAP.
            zx_freeing: Dict[str, int] = {}
            zx_joint = 0
            if not baseline:
                _zb = state.exclusive_for.get(ref, ())
                for _t in _zb:
                    _n = count_legal_poses(state, ref, tx, ty, base_excl,
                                           without_exclusive=(_t.name,),
                                           **zkw)
                    if _n > baseline:
                        zx_freeing[_t.name] = _n
                # JOINTLY blocked, exactly as the keep-out side: two zones
                # that each cross the part's whole feasible region free
                # NOTHING alone, so the per-zone sweep reports {} and the
                # verdict would fall back to a sentence about neighbours.
                #
                # KNOWN GAP, disclosed rather than silently accepted: this
                # joint sweep is per-RULE. A part refused by a keep-out AND an
                # exclusive zone over the same pocket frees nothing under
                # either one, so both channels report {} and the verdict falls
                # back to `no_movable_neighbour`. Closing it needs a
                # cross-rule joint census and a fourth verdict tier, which is
                # disproportionate for a case needing both declarations over
                # one pocket. Filed rather than fixed here.
                if not zx_freeing and len(_zb) > 1:
                    zx_joint = count_legal_poses(
                        state, ref, tx, ty, base_excl,
                        without_exclusive=tuple(t.name for t in _zb), **zkw)
            cinfo: Dict = {}
            cands = _evict_candidates(state, ref, tx, ty, placed, immovable,
                                      constraint=constraint, tol=tol,
                                      info=cinfo)
            freed = {b: count_legal_poses(state, ref, tx, ty,
                                          base_excl | {b}, **zkw)
                     for b in cands}
            no_pose_blockers[ref] = dict(freed)
            # #1213: the FROZEN neighbours, measured the same way and never
            # moved. A part refused by a locked neighbour used to be told
            # about its movable neighbours only, all at 0, and the census
            # never named the part that refused it. Only for a part with no
            # pose at all, nearest first, at the blocker cap.
            #
            # Two numbers per frozen part, because one is not enough. Lifting
            # it ALONE (`frozen_lifted`) answers "would unlocking it seat the
            # part now" -- and on rp2350 that is 0 for U8, because by U6's
            # turn the other parts had filled the frame. `frozen_alone` asks
            # the question the refusal is actually about: with EVERY other
            # neighbour lifted, how many of the `open_poses` the outline and
            # zone allow does this part still admit? U8 admitted none of
            # them: it alone refused U6 everywhere.
            frozen_lifted: Dict[str, int] = {}
            frozen_alone: Dict[str, int] = {}
            open_poses = 0
            frozen_trunc = 0
            _fz = cinfo.get('frozen') or {}
            if not baseline and _fz:
                _order = sorted(
                    (f for f in _fz if f in state.parts),
                    key=lambda f: (math.hypot(state.parts[f].x - tx,
                                              state.parts[f].y - ty), f))
                frozen_trunc = max(0, len(_order) - EVICT_MAX_BLOCKERS)
                for _f in _order[:EVICT_MAX_BLOCKERS]:
                    frozen_lifted[_f] = count_legal_poses(
                        state, ref, tx, ty, base_excl | {_f}, **zkw)
                _everyone = set(state.parts) - {ref}
                open_poses = count_legal_poses(state, ref, tx, ty, _everyone,
                                               **zkw)
                if open_poses:
                    for _f in _order[:EVICT_MAX_BLOCKERS]:
                        frozen_alone[_f] = count_legal_poses(
                            state, ref, tx, ty, _everyone - {_f}, **zkw)
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
                           'keepouts_joint': keepouts_joint,
                           'zone_exclusive_freeing': zx_freeing,
                           'zone_exclusive_joint': zx_joint,
                           'frozen_lifted': frozen_lifted,
                           'frozen_alone': frozen_alone,
                           'open_poses': open_poses,
                           'frozen_truncated': frozen_trunc})
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
                               zinfo, placed, unplaced,
                               rot_ladder=_rot_ladder)
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
    # #1051: the anchor rounds re-seat ONE part at a time, which would pull a
    # formed row apart; its members sit the rounds out.
    row_members = {m for rec in arrays_formed.values() for m in rec['members']}
    if anchors_first and anchor_rounds > 1 and placed:
        from placement.reconstruct import measure, part_extent_mm
        for rnd in range(2, max(2, anchor_rounds) + 1):
            baseline = measure(state)
            snapshot = {r: (state.parts[r].x, state.parts[r].y,
                            state.parts[r].rot) for r in placed}
            moved_n = 0
            # Tie-broken by ref, for the reason given at the anchors queue.
            order2 = sorted(placed,
                            key=lambda r: (-part_extent_mm(state, r), r))
            for ref in order2:
                if (state.parts[ref].locked or ref in fixed_seated
                        or ref in row_members):
                    continue
                target = _partner_centroid(state, ref, placed - {ref})
                if target is None:
                    continue
                ox, oy = state.parts[ref].x, state.parts[ref].y
                if _try_place(state, ref, target[0], target[1],
                              set(),
                              rotations=_rot_ladder(ref)) is not None:
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

    # #1151: where each part the seed could not seat is WRITTEN. After every
    # seat above, so not one seated pose depends on it. A staged part gets a
    # placement row; one left at its input pose needs none.
    disposition = (_dispose_unseated(
        state, [r for r in set(unseated) | set(held) if r not in placed],
        locked=set(lock_refs) | fixed_lock,
        waivers=(intent.waiver_pairs() if hasattr(intent, 'waiver_pairs')
                 else ()),
        keepouts=tuple(getattr(intent, 'keepouts', None) or ()))
        if dispose_unseated else {})
    staged = {r for r, d in disposition.items()
              if d['disposition'] == 'staged'}
    for ref in sorted(staged):
        notes.append(f"{ref}: unseated, and its input pose is not legal "
                     f"against what was seated (refused: "
                     f"{disposition[ref]['refused_by']}) -- STAGED off the "
                     f"board at {disposition[ref]['written'][:2]} so it is "
                     f"not written on top of a neighbour")
    placements = [{'reference': ref,
                   'new_x': state.parts[ref].x, 'new_y': state.parts[ref].y,
                   'new_rotation': state.parts[ref].rot}
                  for ref in sorted(set(placed) | staged)]
    # Deduped: a zone member that also fails stage 3 is appended twice, and
    # `unseated: 2` for one part is a miscount every consumer inherits --
    # place_seed's summary, its exit code, and any gate reading the number.
    # #1054: a refused fixed pose is UNSEATED -- added here, after the
    # eviction rung, which has no target for it and must not trade for it.
    unseated = list(unseated) + sorted(held)
    return {'placements': placements,
            # #1151: {ref: {disposition, input, written, refused_by}} for
            # every part the seed could not seat (`UNSEATED_DISPOSITIONS`).
            # `staged` ones are in `placements` at `written`; the rest are
            # written where they came in.
            'unseated_disposition': disposition,
            'lock_refs': sorted(set(lock_refs) | fixed_lock),
            'unseated': sorted(set(unseated)), 'notes': notes,
            # #1054: {ref: {x, y, rot, side, basis, how, rot_kept,
            # side_kept}} for every fixed pose this seed honoured (`how`:
            # 'contained', 'overhang', or 'already_there' for a part the file
            # already held at it), and {ref: {..., reason,
            # conflicts_with_declared}} for every one it refused (the other
            # DECLARED poses it clashes with; each of those is refused too).
            # Both empty when nothing is declared.
            'fixed_seated': fixed_seated,
            'fixed_refused': fixed_refused,
            # #1053: {armed, scope, claimed, put_back, pins, reason,
            # served_first} -- the pin stage's own count and, whenever it
            # claimed nothing with a non-empty scope, why. #1105: `late`
            # ({armed, at, owners, pins, caps, claimed, reason}) is stage
            # 3.5's, present whenever the stage is armed by the intent.
            'decap_stage': decap_stage,
            # #1051: {name: {serves, members (in row order), rot, pitch_mm,
            # axis, anchor, target, zone, poses_tried, clearance, verdict,
            # failed, unchecked}} for every declared row seated whole;
            # `verdict` is `arrays.formation`'s at the SEEDED poses (a later
            # polish can move them -- place_seed re-grades at the written
            # ones). And {name: {members, reason, poses_tried, capped}} for
            # every row not seated whole, whose members were seated one by one
            # (with `zone_unseated`: the members a zoned row's fallback could
            # not seat in its zone).
            'arrays_formed': arrays_formed,
            'array_unseated': array_unseated,
            # Stage 2.4's seat order: each declared row's served part, and
            # `array:<name>` where each row fell among them. Empty when no
            # non-zoned row is declared.
            'early_order': early_order,
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
            # #893. Declared rotations that could NOT be seated, by ref and
            # angle. The refusal is structural rather than a check: a declared
            # ladder has only the declared angle in it, so a part that does not
            # fit at it reaches `unseated` instead of being quietly turned --
            # which is what happened before, with a note nobody gated on. This
            # key exists so a caller can say WHICH claim it could not meet
            # rather than reporting a bare unseated ref.
            'rotation_unseated': {
                r: (declared_rot[r][0] if declared_rot[r][0] is not None
                    else list(declared_rot[r][1]))
                for r in unseated if r in declared_rot},
            'no_pose_census': no_pose_census,
            # #975. Declared edge connectors seated with pad copper inside the
            # board-edge floor because no pose the ladder tried clears it, by ref: the
            # pose, the floor, the worst pads and why the seat could not move.
            # Only records still describing the written pose are kept.
            'edge_floor_fallback': _floor_records_at_final_pose(
                state, edge_floor_fallback)}


#: KiCad 6's bare footprint lock, ``(footprint "X" locked ...``; group 1 is
#: everything before the word, which the unlock keeps.
_BARE_FP_LOCK_RE = re.compile(
    r'^(\(\s*(?:footprint|module)\s+(?:"(?:[^"\\]|\\.)*"|[^\s()"]+)'
    r'(?:\s+[A-Za-z_]+)*?)\s+locked\b')


def stamp_locked(board_file: str, refs: Sequence[str]) -> int:
    """Insert `(locked yes)` into the named footprints, in place.

    Inserted immediately after the footprint's opening token, which is before
    the first pad -- the position placement/parser.extract_locked_refs (and
    KiCad itself) reads it from. The grade's must_lock rule demands the lock
    IN THE FILE, so writing the intent's locks here is what makes the emitted
    seed grade clean rather than merely hoped-correct."""
    from kicad_parser import footprint_head_flags, iter_footprint_blocks
    with open(board_file, 'r', encoding='utf-8') as f:
        content = f.read()
    want = set(refs)
    count = 0
    # Named by the parser's own resolver (#726), so locking `TP4` stamps the
    # one block the parser calls `TP4`. It used to match the reference STRING,
    # so on watchy it locked both test points from one name -- and the refs
    # handed in here come from `pcb.footprints` keys, which now address the
    # twins separately. Reverse order keeps the spans valid as text is inserted.
    for start, end, fp_text, _raw_ref, key in reversed(
            list(iter_footprint_blocks(content))):
        if key not in want:
            continue
        if (re.search(r'\(locked(?:\s+yes)?\)', fp_text[:fp_text.find('(pad')
                                                   if '(pad' in fp_text else len(fp_text)])
                or 'locked' in footprint_head_flags(fp_text)):  # KiCad 6: bare
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


def stamp_unlocked(board_file: str, refs: Sequence[str]) -> int:
    """Remove `(locked yes)` from the named footprints, in place (#892).

    The inverse of `stamp_locked`, and it lives beside it deliberately: this
    repo had a stamper and no un-stamper, so a model that locked a rotation
    decision could not change its mind without hand-editing the board -- which
    is the class of hand script #892 exists to remove. This half reads exactly
    the window `stamp_locked` writes into -- the header, up to `(pad` -- and
    addresses blocks by the parser's own key (#726), so lock and unlock cannot
    disagree about which block they mean.

    Note for anyone tightening this: `placement/parser.extract_locked_refs`
    cuts at `'(pad '` WITH the space and falls back to the first 500
    characters when a block has no pad, so a header containing a token like
    `(padstack` is read differently there than here. Both stamping halves
    share that divergence and it predates them. Measured over the 22 boards in
    `kicad_files/` (1349 footprint blocks): 0 blocks contain `(pad` before
    `(pad `, and 0 disagree about a `(locked yes)`. 30 blocks DO take the
    reader's 500-character fallback (they have no pad at all) -- they simply
    carry no late lock, so the two windows still agree today. It is called out
    because the
    consequence is asymmetric: the reader would call a footprint locked that
    this cannot unlock, which is why `pose_ops.apply_poses` VERIFIES the
    unlock on the staged board before promoting anything.

    Returns the number of footprints actually changed; a ref that was not
    locked contributes 0 rather than raising, so unlocking twice is idempotent.
    Only KiCad's footprint `(locked yes)` is touched -- locked SEGMENTS and
    VIAS are copper, read by a different rule (#521), and are not footprint
    blocks, so nothing here can reach them.
    """
    from kicad_parser import footprint_head_flags, iter_footprint_blocks
    with open(board_file, 'r', encoding='utf-8') as f:
        content = f.read()
    want = set(refs)
    count = 0
    # Reverse order keeps the spans valid as text is removed, exactly as the
    # stamping half relies on it while text is inserted.
    for start, end, fp_text, _raw_ref, key in reversed(
            list(iter_footprint_blocks(content))):
        if key not in want:
            continue
        head_end = fp_text.find('(pad') if '(pad' in fp_text else len(fp_text)
        head, tail = fp_text[:head_end], fp_text[head_end:]
        new_head, n = re.subn(r'\s*\(locked(?:\s+yes)?\)', '', head)
        # KiCad 6 writes the lock as a bare word after the name:
        # (footprint "X" locked (layer ... -- leave that and KiCad keeps it locked.
        new_head, n_bare = _BARE_FP_LOCK_RE.subn(r'\1', new_head, count=1)
        n += n_bare
        if not n:
            continue
        content = content[:start] + new_head + tail + content[end:]
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

#: The rules `--repair-decaps` (#1066) seats a cap FOR. Each names its cap and
#: its IC (`decap_distance`: ref = the cap, measured `ic`;
#: `decap_pin_distance`: ref = the IC, measured `cap` and `pad`).
DECAP_RUNG_RULES = ('decap_distance', 'decap_pin_distance')


def _decap_target(state, pcb_data, cap: str, ic: str, pad_number=None):
    """`(x, y)` the decap rung seats `cap` toward: the IC's declared supply
    pad (`decap_pin_distance`), or the IC pad on the cap's RAIL nearest the
    cap (`decap_distance`) -- the rail being the cap's smallest multi-ref
    net, stage 2.5's own choice (`rail_of`). That keeps the main ground
    off the target, not every ground: on a split-ground board the smallest
    net can be a local one (glasgow's C76 -> /GNDPLL0), and a non-decoupling
    cap aims at a signal pad -- harmless, since the grade decides what is
    kept. None when the IC carries no such pad."""
    from .legality import footprint_at_pose
    part, chip = state.parts[cap], state.parts[ic]
    fp = footprint_at_pose(pcb_data.footprints[ic], (chip.x, chip.y, chip.rot))
    if pad_number is not None:
        pads = [p for p in fp.pads if str(p.pad_number) == str(pad_number)]
    else:
        rail = min((nid for nid in part.nets
                    if len(state.net_refs.get(nid, ())) >= 2),
                   key=lambda nid: (len(state.net_refs[nid]), nid),
                   default=None)
        pads = [p for p in fp.pads if rail is not None and p.net_id == rail]
        if not pads:
            # The cap's smallest net is not on this IC (watchy C14: its
            # smallest net is an LED's, and it decouples U1 on +3V3): aim at
            # the smallest net the two DO share -- the grade decides.
            shared = sorted((len(state.net_refs.get(p.net_id, ())), p.net_id)
                            for p in fp.pads if p.net_id in part.nets
                            and len(state.net_refs.get(p.net_id, ())) >= 2)
            if shared:
                pads = [p for p in fp.pads if p.net_id == shared[0][1]]
    if not pads:
        return None
    best = min(pads, key=lambda p: (math.hypot(p.global_x - part.x,
                                               p.global_y - part.y),
                                    str(p.pad_number)))
    return best.global_x, best.global_y


def _repair_decap_rung(state, pcb_data, graded, grader, limits, rot_ladder,
                       notes) -> Dict[str, Dict]:
    """#1066 (b): seat each cap a decap rule charges at its IC's pin.

    `repair_placement`'s ordinary seat is `_try_place` at the part's CURRENT
    pose, which knows nothing of a decap target and accepts the pose it
    stands at -- which is why a decap violator never moved. This is the
    missing actor: for every `decap_distance` / `decap_pin_distance` error,
    the CAP (never the IC, which carries every other claim on its pins) is
    searched for the nearest legal pose within the rule's own limit of its
    target pad (`_decap_target`), by the same `_try_place` every seat uses.

    Accepted only when the grade says it is a fix: the charged claim is gone
    AND no finding is new or worse (`new_or_worse`, per finding -- stricter
    than `floorplan.grade_delta`, which counts claims) AND the placement's own
    overlap / off-board numbers did not grow, taken on the whole board before
    and after. Anything else is reverted, and said.
    A claim an earlier seat already cleared is skipped. Opt-in
    (`repair_decaps`), because it moves parts the repair did not move before.

    Returns `{cap: {'moved': bool, 'tried': [row, ...]}}`."""
    from placement import floorplan as _fp
    from .legality import EPS as _eps
    tasks = []
    for v in graded.errors:
        if v.rule not in DECAP_RUNG_RULES or not v.ref:
            continue
        m = v.measured or {}
        if v.rule == 'decap_distance':
            cap, ic, pad = v.ref, m.get('ic'), None
        else:
            cap, ic, pad = m.get('cap'), v.ref, m.get('pad')
        if cap in state.parts and ic in state.parts:
            amt = m.get('distance_mm', m.get('gap_mm'))
            excess = (float(amt) - float(limits.get(v.rule, 0.0))
                      if isinstance(amt, (int, float)) else 0.0)
            # The FINDING, not the claim: every uncovered pin of one IC
            # shares a claim, so asking the claim whether THIS pin is fixed
            # read a cap that cleared pin 1 as failing while pin 6 was still
            # charged -- two caps serving two pins of one IC reverted each
            # other (phase-3 verifier: glasgow C14/C16 on U36, 11 such).
            tasks.append((str(cap), str(ic), pad, finding_key(v),
                          max(0.0, excess)))
    out: Dict[str, Dict] = {}
    for cap, ic, pad, claim, excess in sorted(
            tasks, key=lambda t: (t[0], t[1], str(t[2]))):
        rec = out.setdefault(cap, {'moved': False, 'tried': []})
        row = {'rule': claim[0], 'ref': claim[1], 'ic': ic, 'pad': pad}
        rec['tried'].append(row)
        part = state.parts[cap]
        if part.locked:
            row['result'] = 'locked'
            continue
        try:
            before = grader.violations()
            leg0 = grader.legality_at()
        except (_fp.UntrustworthyOutline, ValueError) as exc:
            row['result'] = f'unavailable: {type(exc).__name__}'
            notes.append(f"decap rung: the grade is unavailable "
                         f"({type(exc).__name__}: {exc}) -- nothing seated")
            break
        if claim not in findings_of(before):
            row['result'] = 'already_cleared'
            continue
        target = _decap_target(state, pcb_data, cap, ic, pad)
        if target is None:
            row['result'] = 'no_target_pad'
            continue
        ox, oy, orot = part.x, part.y, part.rot
        # The rule measures to the chip's pad BOX (or the cap's own pads),
        # not to the target pad's centre, so a pose farther than the limit
        # from that centre can satisfy it: the search widens once, to twice
        # the limit, and the grade below is what decides.
        got = None
        for disp in (float(limits[claim[0]]), 2.0 * float(limits[claim[0]])):
            got = _try_place(state, cap, target[0], target[1], set(),
                             max_disp=disp, rotations=rot_ladder(cap))
            if got is not None:
                break
        if got is None:
            row['result'] = 'no_legal_pose_within_limit'
            continue
        after = grader.violations()
        leg1 = grader.legality_at()
        # Per FINDING (`new_or_worse`), not per claim: `grade_delta` cannot
        # see a second pin stranded under a claim its IC already carries, or
        # a finding that only grew -- the two ways a cap moved TO one pin
        # can hurt another.
        added = sorted({f"{v.rule} on {v.ref}"
                        + ('' if how == 'new' else ' (worse)')
                        for v, how in new_or_worse(findings_of(before),
                                                   after)})
        for key in LEGALITY_COMPARE_KEYS:
            was, now = leg0.get(key), leg1.get(key)
            if (isinstance(was, (int, float)) and isinstance(now, (int, float))
                    and now > was + (0 if isinstance(now, int) else _eps)):
                added.append(f'legality.{key}')
        # Past the decap search radius the finding is not cleared, it stops
        # being GRADED: `decap_distance` (error) becomes `decap_ungraded`
        # (a warn `findings_of` does not read -- or, since #1142, an error it
        # does read for a cap a --decaps-from reference holds). Still open, as the
        # honesty re-grade already says (round-2 verifier: tigard C18 moved
        # 0.85mm to 5.21mm from U3 and read "cleared").
        still = (claim in findings_of(after)
                 or (claim[0] == 'decap_distance'
                     and any(v.rule == 'decap_ungraded' and v.ref == cap
                             for v in after)))
        d = math.hypot(part.x - ox, part.y - oy)
        # The ordinary repair's proportion rule, in the violation's own
        # currency: a cap 0.125mm past its limit is not moved 9.9mm to fix
        # it (esp_prog C3, measured) -- that is a different placement.
        budget = max(DISPROPORTION_FLOOR_MM, DISPROPORTION_RATIO * excess)
        if d > budget:
            state.apply_move(cap, ox, oy, orot)
            row['result'] = 'disproportionate'
            row['moved_mm'] = round(d, 3)
            notes.append(
                f"{cap}: decap rung's only fixing pose is {d:.2f}mm away, "
                f"disproportionate to the {excess:.3f}mm it is past its "
                f"limit (budget {budget:.2f}mm) -- left in place")
            continue
        if added or still:
            state.apply_move(cap, ox, oy, orot)
            row['result'] = 'reverted'
            row['added'] = added
            row['still'] = still
            notes.append(
                f"{cap}: decap rung found a pose {d:.2f}mm away at {ic}"
                + (f" pad {pad}" if pad is not None else '')
                + " and REVERTED it -- "
                + '; '.join(([f"it would add {', '.join(added)}"]
                             if added else [])
                            + ([f"{claim[0]} would remain"] if still
                               else [])))
            continue
        rec['moved'] = True
        row['result'] = 'seated'
        row['moved_mm'] = round(d, 3)
        notes.append(f"{cap}: decap rung seated it {d:.2f}mm from its pose, "
                     f"toward {ic}" + (f" pad {pad}" if pad is not None
                                       else '')
                     + f" -- {claim[0]} cleared, nothing added")
    return out

#: The measured value a grade finding gets WORSE along, first key found
#: (#1066). A distance or an escape: larger is worse.
FINDING_AMOUNT_KEYS = ('gap_mm', 'distance_mm', 'outside_mm', 'area_mm2',
                       'intrusion_mm2', 'overlap_mm2')

#: A finding's amount must grow by more than this to be "worse": the grade
#: rounds its measurements to 3-4 decimals.
FINDING_WORSE_EPS_MM = 1e-3


def finding_key(v) -> Tuple:
    """One grade FINDING's identity: `floorplan.violation_claim` plus the
    pad and net it is about. The claim alone is per (rule, ref, block), so
    an IC with one supply pin already past its limit reads a SECOND pin
    stranded by a move as the same claim -- a new finding, invisible (the
    phase-1 round-2 verifier: watchy U4 pad 20, created by moving C5, hidden
    behind U4 pad 46)."""
    from placement import floorplan as _fp
    m = v.measured or {}
    return _fp.violation_claim(v) + (str(m.get('pad', '')),
                                     str(m.get('net', '')))


def finding_amount(v) -> Optional[float]:
    m = v.measured or {}
    for k in FINDING_AMOUNT_KEYS:
        x = m.get(k)
        if isinstance(x, (int, float)) and not isinstance(x, bool):
            return float(x)
    return None


def findings_of(violations) -> Dict[Tuple, Optional[float]]:
    """{finding_key: amount} over the ERRORS in `violations` (the largest
    amount when a key repeats)."""
    from placement import floorplan as _fp
    out: Dict[Tuple, Optional[float]] = {}
    for v in violations:
        if v.severity != _fp.ERROR:
            continue
        k, a = finding_key(v), finding_amount(v)
        if k not in out or (a is not None and (out[k] is None or a > out[k])):
            out[k] = a
    return out


def new_or_worse(before: Dict[Tuple, Optional[float]], after_violations
                 ) -> List[Tuple[object, str]]:
    """`[(violation, 'new' | 'worse')]`: every ERROR in `after_violations`
    that `before` (a `findings_of`) does not have, or has with a smaller
    amount. Stricter than `floorplan.grade_delta`, which counts claims and so
    is blind to a second finding under one claim and to a finding that only
    grew -- a cap moved further from the only IC it decouples (watchy C12,
    U3 VBUS 7.78 -> 11.10mm) is a regression, not a repair."""
    from placement import floorplan as _fp
    out = []
    for v in after_violations:
        if v.severity != _fp.ERROR:
            continue
        k = finding_key(v)
        if k not in before:
            out.append((v, 'new'))
            continue
        a, b = finding_amount(v), before[k]
        if a is not None and b is not None and a > b + FINDING_WORSE_EPS_MM:
            out.append((v, 'worse'))
    return out
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
                     repair_decaps: bool = False,
                     baseline_file: Optional[str] = None,
                     body_model: bool = False) -> Dict:
    """Violation-driven minimal-move repair of a PLACED board (#place_seed
    --repair). Everything clean freezes; only violators move, worst first,
    each seated by the seeder's own search targeted at its CURRENT pose with
    an escalating displacement cap. The opposite contract of --force (which
    re-derives everything).

    Violators: intent grade errors with a ref (zone/edge/decap...; never
    `decap_ungraded`, which no move here can target -- #1142), pad/hole
    legality conflicts (the movable member of each pair), and parts off the
    board outline -- except refs the intent declares as edge connectors,
    whose overhang is by design.

    A must_lock ref is seeder-owned: its file lock is lifted in-memory for
    seating (the stamp in the file survives the positional rewrite, so no
    re-stamping is needed). A file-locked ref OUTSIDE must_lock is not this
    tool's to move: reported in `unrepairable`.

    `repair_decaps` (#1066 b, `--repair-decaps`, off by default) adds the
    decap rung, `_repair_decap_rung`: each cap a decap rule charges is seated
    at its IC's pin, kept only when the grade calls it a fix. Without it a
    decap violator is never moved (the ordinary seat has no decap target),
    and the honesty re-grade reports it `unresolved`.

    NOTE this sweep has no internal bound -- its cost is violators x caps x 36
    ring sweeps x O(parts) per candidate, and on a 217-part board it ran 46
    minutes (run 9). Scope it with the violator set, not with a clock.
    """
    import os
    import pose_score
    from placement import floorplan, legality as _leg

    # A caller's --lock globs, on top of the file's own (locked yes) stamps.
    # These two entry points RE-PARSE the board and build their own state, so
    # a lock resolved by the CLI never reached them -- `--lock` was honoured
    # by five reconstruct stages and silently ignored by the two that move the
    # most parts. Feeding it here means the existing `.locked` checks below
    # (the unrepairable filter, and reseat's refusal list) pick it up for free.
    _extra_locked = {r for pat in (lock_globs or [])
                     for r in sorted(pcb_data.footprints) if fnmatch.fnmatchcase(r, pat)}
    # #1054: a `fixed_poses[]` ref is a mechanical fact the seed put at its
    # exact pose and stamped (locked yes). A repair never nudges one, stamped
    # or not: an unstamped copy of the board must not turn the fact into a
    # violator to move. It lands in `unrepairable` if it violates anything.
    _extra_locked |= {str(f['ref'])
                      for f in (getattr(intent, 'fixed_poses', ()) or ())
                      if str(f['ref']) in (pcb_data.footprints or {})}
    # Hoisted above `make_state` (#797), as in `seed_from_intent`; the
    # `ref_zone` join below reads the same `blocks`.
    blocks, _probs = floorplan.resolve_blocks(intent, pcb_data, group_sources) \
        if intent else ({}, [])
    # #893. A repair must honour a declared rotation for the same reason a seed
    # must: the claim is about the BOARD, not about which entry point touched
    # it last. Without this, `place_optimize --repair` would quietly undo an
    # angle `place_seed` had just been told to hold.
    _declared_rot = floorplan.rotations_for_ref(intent, blocks) if intent else {}

    def _rot_ladder(ref):
        return floorplan.declared_ladder(_declared_rot.get(ref))
    state = pose_score.make_state(
        pcb_data, pcb_file, clearance=clearance,
        board_edge_clearance=board_edge_clearance, grid_step=grid_step,
        extra_locked_refs=_extra_locked or None,
        # #701: the declared keep-outs reach the SEAT PREDICATE (see
        # `seed_from_intent`). `intent` is optional on this path, so the
        # inert default is what a caller without one gets.
        keepouts=intent.keepouts if intent else (),
        # #797: and the exclusive zones. Arming the REPAIR path is what lets
        # `--repair` fix a zone_exclusive breach at all -- without it the
        # repair's own `_try_place` is free to re-seat the stranger straight
        # back into the zone it was moved out of. Safe here because
        # `_try_place` is a nearest-first RING search, not a bounded nudge: it
        # finds the nearest fully clear pose directly and never needs a
        # partly-still-inside stepping stone, which is exactly the property
        # the quench lacks and why the quench's gate has to be monotone.
        exclusive_zones=(floorplan.zone_entries(intent, blocks)
                         if intent else ()),
        # #1182: armed, the seat search spaces parts on check_assembly's
        # occupancy, so a courtyard pair this repair is charged for can be
        # cleared rather than re-seated onto the same pad-box-legal overlap.
        body_model=body_model)
    # #975: see `seed_from_intent`. Without an intent there is no grade to
    # compare on, and an edge seat keeps today's preference guards only.
    pose_grader = (floorplan.PoseGrader(
        intent, state, blocks=blocks, clearance=clearance,
        board_edge_clearance=board_edge_clearance) if intent else None)
    refs_all = sorted(pcb_data.footprints)
    notes: List[str] = []
    must_lock = {r for pat in intent.must_lock
                 for r in refs_all if fnmatch.fnmatchcase(r, pat)} if intent else set()
    # {ref: declared band max mm}. The off-board census below charges only the
    # EXCESS past the band, not nothing at all -- see the note there.
    edge_band: Dict[str, float] = {}
    if intent:
        for _c in intent.edge_claims():   # seat claims only; see edge_claims
            edge_band[_c['ref']] = float(
                (_c.get('overhang_mm') or {}).get('max') or 0.0)

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
    # #1066: every grade error a ref is CHARGED for, by `violation_claim`, so
    # the honesty re-grade after the moves can ask whether THAT finding is
    # still on the board -- not merely whether the part moved.
    charged_claims: Dict[str, List[Tuple]] = {}
    if intent is not None:
        graded = floorplan.grade(intent, pcb_data, pcb_file,
                                 group_sources=group_sources,
                                 clearance=clearance,
                                 board_edge_clearance=board_edge_clearance)
        for v in graded.errors:
            # #1142: a held cap stranded beyond the tether radius
            # (`decap_ungraded`, an ERROR under a --decaps-from intent) is
            # charged to no one. This repair seats a violator at its nearest
            # LEGAL pose, not toward its IC, so charging one nudged the cap
            # further out and shipped the worse pose (final review: esp_prog
            # C2 5.64 -> 5.69 mm). The finding stays in the grade, and in its
            # exit code; `--repair-decaps` has no rung for it either. The same
            # holds for a #1102 board-wide `severity.decap_ungraded: error`.
            # (`decap_distance` / `decap_pin_distance` are still charged and
            # can be nudged the same way -- older than #1142, filed as
            # #1150.) Said in `notes`, so a --dry-run, which has no final
            # grade, still names the cap.
            if v.rule == 'decap_ungraded':
                notes.append(
                    f"{v.ref}: not charged -- decap_ungraded (a cap the "
                    f"reference holds, stranded past the tether radius, "
                    f"#1142) has no repair that moves it toward its IC; "
                    f"the finding stays in the grade")
                continue
            if v.ref:
                _charge(v.ref, float((v.measured or {}).get('outside_mm', 1.0)
                                     or 1.0))
                if v.ref in state.parts:
                    charged_claims.setdefault(v.ref, []).append(
                        floorplan.violation_claim(v))
    # The claims the POSE grader reproduces at the input poses. A charged
    # claim it does not reproduce comes from a part of `grade` outside the
    # rules loop (intent validation, block resolution), which no move can
    # clear -- so after the moves it is counted as still present. Measured
    # before any move, while the state still holds the input poses; also the
    # BASELINE a move's NEW finding is told apart by (see the re-grade).
    claims_regradable = None
    findings_before = None
    regrade_error = None
    if pose_grader is not None:
        try:
            _before = pose_grader.violations()
            claims_regradable = {floorplan.violation_claim(v)
                                 for v in _before
                                 if v.severity == floorplan.ERROR}
            findings_before = findings_of(_before)
        except (floorplan.UntrustworthyOutline, ValueError) as exc:
            regrade_error = exc

    # worst_n=0: the FULL pair census (run-4 F5). The default cap of 10
    # bounded one repair pass at 10 pair-movers on a 20-pair board -- the
    # summary said 20 conflicts while only 10 got charged.
    pads = _leg.grade_pad_legality(pcb_data, clearance, worst_n=0,
                                   edge_margin=board_edge_clearance,
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

    # PIN census (fa10 P1, #1212): a part whose courtyard covers a pin
    # frame's drilled pin -- KiCad's pth_inside_courtyard, check_assembly's
    # absolute `pin_in_courtyard` conjunct. The frame itself never moves for
    # it (its rect is not a body, and it is usually the board's host): the
    # OTHER member is charged, at the containment scale, because a pin under
    # a courtyard is the same yes/no buildability fact. A pair the intent
    # declares is the designer's and is skipped, as check_assembly skips it.
    _pin_frames = set(body.get('containers') or ())
    _declared = {frozenset(w) for w in (intent.waiver_pairs()
                                        if intent is not None
                                        and hasattr(intent, 'waiver_pairs')
                                        else ())}
    _pins = [q for q in body.get('pin_in_courtyard_pairs', ())
             if frozenset((q.a, q.b)) not in _declared]
    if _pins:
        print(f"  Pin census: {len(_pins)} part(s) over a pin frame's "
              f"pin(s), all listed")
    for q in _pins:
        frame, other = ((q.a, q.b) if q.a in _pin_frames else (q.b, q.a))
        if other not in state.parts or state.parts[other].locked:
            notes.append(f"pin_in_courtyard {other} over {frame} pin(s) "
                         f"{', '.join(q.pins)}: {other} is file-locked -- "
                         f"not repairable here")
            continue
        _charge(other, CONTAINMENT_CHARGE_MM)

    # COURTYARD census (#1182): check_assembly's own courtyard channel
    # (`CourtyardCensus`, with the intent's waivers), and its GATE. The
    # repair used to read no courtyard pair at all -- One-Air-Max's s180_0q
    # printed `Repair census: 0 conflict pair(s)` while check_assembly
    # --baseline gated C27/L1, U5/U8, D6/D7 and JP3/U12. check_assembly gates
    # a courtyard pair only when a member MOVED against a baseline, so the
    # repair charges only then: with no `baseline_file` it reports and does
    # not charge (charging absolutely churned by-design pairs on 5 of 34
    # healthy boards). The member that moved is the one charged -- our move
    # put it there -- by `_mover_key`, weight its depth (>= 1 mm).
    cy_census = None
    cy_base = None
    cy_gating_before: List = []
    cy_charged: Dict[str, Set] = {}
    try:
        cy_census = _leg.CourtyardCensus(
            pcb_data, pcb_file,
            intent_waivers=(intent.waiver_pairs() if intent else ()))
        if baseline_file:
            from kicad_parser import parse_kicad_pcb as _parse
            cy_base = _parse(baseline_file)
        _moved0 = (_leg.moved_refs(pcb_data, cy_base)
                   if cy_base is not None else None)
        _cg = cy_census.grade(moved=_moved0)
        cy_gating_before = list(_cg.gating or ())
        print(f"  Courtyard census: {len(_cg.blocking)} blocking pair(s), "
              + (f"{len(cy_gating_before)} gating against "
                 f"{os.path.basename(baseline_file)}, charged"
                 if cy_base is not None else
                 "none charged (no --baseline: check_assembly gates a "
                 "courtyard pair only when a member moved against one)"))
    except Exception as exc:                                 # noqa: BLE001
        cy_census = None
        notes.append(f"courtyard census unavailable ({type(exc).__name__}: "
                     f"{exc}) -- no courtyard pair charged")
    for q in cy_gating_before:
        free = [r for r in (q.a, q.b)
                if r in state.parts and not state.parts[r].locked]
        mine = [r for r in free if r in (_moved0 or ())] or free
        if not mine:
            notes.append(f"courtyard {q.a}<->{q.b} ({q.area_mm2}mm2): both "
                         f"file-locked -- not repairable here")
            continue
        ordered = sorted(mine, key=_mover_key)
        _charge(ordered[0], max(1.0, float(q.depth_mm or 0.0)))
        cy_charged.setdefault(ordered[0], set()).add(frozenset((q.a, q.b)))
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
    edge_floor_fallback: Dict[str, Dict] = {}   # #975, see `_floor_rung`
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
            ok = _seat_edge(state, ref, ec, must_lock, notes, target=tgt,
                            rotations=_rot_ladder(ref),
                            disclose=edge_floor_fallback, grade=pose_grader)
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
            clr = _try_place(state, ref, ox, oy, set(),
                             rotations=_rot_ladder(ref), constraint=rect,
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
                             tol=tol, rotations=_rot_ladder(ref))
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
                                  rotations=_rot_ladder(partner)) is not None:
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

    # #1066 (b): the decap rung, opt-in. After the ordinary seats, so it
    # measures the board they left; before both re-grades, which then judge
    # its seats like any other move.
    decap_rung: Dict[str, Dict] = {}
    if repair_decaps and graded is not None and pose_grader is not None:
        _dc = dict(getattr(intent, 'decaps', None) or {})
        _limits = {'decap_distance': _dc.get('max_distance_mm'),
                   'decap_pin_distance': _dc.get('max_pin_distance_mm')}
        decap_rung = _repair_decap_rung(
            state, pcb_data, graded, pose_grader,
            {k: v for k, v in _limits.items() if v is not None},
            _rot_ladder, notes)
        for cap, rec in sorted(decap_rung.items()):
            if not rec['moved']:
                continue
            p = state.parts[cap]
            moves[:] = [m for m in moves if m['reference'] != cap]
            moves.append({'reference': cap, 'new_x': p.x, 'new_y': p.y,
                          'new_rotation': p.rot})
            zero_move[:] = [r for r in zero_move if r != cap]
            failed[:] = [r for r in failed if r != cap]
            if cap not in repaired:
                repaired.append(cap)

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

    # #1066: the INTENT half of the same honesty re-grade. The loop above
    # re-checks pads, holes and containment only, so a part charged for a
    # grade error -- a decap too far from its IC, a part out of its zone --
    # that `_try_place` left where it stood (it searches for the nearest
    # LEGAL pose and has no intent target) was reported repaired while the
    # finding stayed on the board: glasgow, 33 repaired, 0 moved, errors
    # 46 -> 46. Every charged ref is re-graded, MOVED OR NOT: a cap moved
    # 0.3mm and still too far from its IC is no more repaired than one that
    # did not move. A ref is repaired only when every claim it was charged
    # for is gone.
    #
    # And the move must not have MADE one. A cap charged for a pad conflict,
    # moved off it and out of its decap limit, cleared its charge and read
    # repaired while the board gained an error (phase-1 verifier: watchy
    # 9 -> 19 errors, splitflap C8 re-seated 2.00mm to 3.29mm from U9). An
    # error the input poses did not have is attributed to every MOVED ref it
    # names -- its own ref, or the cap / IC / partner its measurement names,
    # since `decap_pin_distance` is charged to the IC a moved cap stranded.
    unresolved_claims: Dict[str, List[str]] = {}
    moved_refs = {m['reference'] for m in moves}
    check_refs = [r for r in dict.fromkeys(repaired)
                  if r in charged_claims or r in moved_refs]
    if check_refs and pose_grader is not None:
        after = None
        if regrade_error is None and claims_regradable is not None:
            try:
                after = pose_grader.violations()
            except (floorplan.UntrustworthyOutline, ValueError) as exc:
                regrade_error = exc
        after_claims = ({floorplan.violation_claim(v) for v in after
                         if v.severity == floorplan.ERROR}
                        if after is not None else None)
        # A cap pushed past the decap search radius does not clear its
        # `decap_distance` charge, it stops being GRADED: the finding becomes
        # `decap_ungraded` (warn, or per cap an error under a --decaps-from
        # intent, #1142) under a different claim key. Read as the
        # charge persisting, or leaving the radius would be a way to be fixed.
        ungraded = ({v.ref for v in after if v.rule == 'decap_ungraded'}
                    if after is not None else set())
        # A finding the input poses did not have, or had SMALLER, charged to
        # the moved ref that caused it. The ref it NAMES is not enough: a pin
        # a cap's move stranded names the IC and whichever cap is now
        # nearest, never the cap that left (round-2 verifier: watchy C5). So
        # a finding naming no moved ref is attributed by COUNTERFACTUAL --
        # each moved ref restored alone to its input pose, the finding
        # disappearing or shrinking back names the move that made it.
        created: Dict[str, List[str]] = {}
        unattributed: List[str] = []
        restored: Dict[str, Dict[Tuple, Optional[float]]] = {}
        for v, how in (new_or_worse(findings_before, after)
                       if after is not None else ()):
            label = v.rule if how == 'new' else f"{v.rule} (made worse)"
            m = v.measured or {}
            names = {v.ref} | {m.get(k) for k in ('cap', 'ic', 'near')}
            who = sorted(names & moved_refs)
            if not who:
                k, amt = finding_key(v), finding_amount(v)
                for r in sorted(moved_refs):
                    if r not in restored:
                        fp0 = pcb_data.footprints[r]
                        try:
                            restored[r] = findings_of(pose_grader.violations(
                                poses={r: (fp0.x, fp0.y,
                                           (fp0.rotation or 0.0) % 360.0)}))
                        except (floorplan.UntrustworthyOutline,
                                ValueError):
                            restored[r] = None
                    got = restored[r]
                    if got is None:
                        continue
                    if k not in got or (
                            amt is not None and got[k] is not None
                            and got[k] < amt - FINDING_WORSE_EPS_MM):
                        who.append(r)
            if not who:
                unattributed.append(f"{label} on {v.ref}")
            for r in who:
                created.setdefault(r, []).append(label)
        if unattributed:
            notes.append(
                "repair: " + '; '.join(sorted(set(unattributed)))
                + " -- not present before the repair, and no single move "
                  "restored alone clears it (a joint effect of several)")
        for ref in check_refs:
            charged = charged_claims.get(ref, ())
            made: List[str] = []
            if after_claims is None:
                if not charged:
                    continue
                still = sorted({c[0] for c in charged})
                why = (f"the grade could not be re-run after the repair "
                       f"({regrade_error.__class__.__name__}: "
                       f"{regrade_error})" if regrade_error is not None
                       else "the grade could not be re-run after the repair")
            else:
                still = sorted({c[0] for c in charged
                                if c in after_claims
                                or c not in claims_regradable
                                or (c[0] == 'decap_distance'
                                    and ref in ungraded)})
                made = sorted(set(created.get(ref, ()))
                              - {r for r in still})
                why = None
                if ('decap_distance' in still and ref in ungraded
                        and ('decap_distance', ref) not in
                        {(c[0], c[1]) for c in after_claims}):
                    still = [r if r != 'decap_distance' else
                             'decap_distance (moved past the decap search '
                             'radius: now decap_ungraded, not cleared)'
                             for r in still]
            if not still and not made:
                continue
            repaired[:] = [r for r in repaired if r != ref]
            if ref not in unresolved:
                unresolved.append(ref)
            unresolved_claims[ref] = sorted(
                {r.split(' ')[0] for r in list(still) + list(made)})
            said = []
            if why:
                said.append(f"{why}, so the charged {', '.join(still)} "
                            f"cannot be shown cleared")
            elif still:
                said.append(f"still carries {', '.join(still)} after the "
                            f"repair")
            if made:
                said.append(f"its move created {', '.join(made)}, which the "
                            f"input poses did not have at that size")
            notes.append(
                f"{ref}: UNRESOLVED -- " + '; '.join(said)
                + ((" (it moved, and the move did not clear it)" if still
                    else " (it moved)") if ref in moved_refs
                   else " (it did not move)")
                + " -- NOT reported repaired")

    # #1182: the COURTYARD half of the honesty re-grade, on check_assembly's
    # channel at the FINAL poses and against the same baseline. A part
    # charged for a gating pair that still gates, or a part this repair
    # moved that is now in a gating pair the input did not have, is not
    # `repaired`. Unarmed, the seat search spaced pad boxes, which is how a
    # move can leave a courtyard pair exactly where it was: say so.
    courtyard_after = None
    if cy_census is not None and cy_base is not None:
        _final = {r: (p.x, p.y, p.rot) for r, p in state.parts.items()}
        _cga = cy_census.grade(_final, moved=_leg.moved_refs_at(
            pcb_data, cy_base, _final))
        courtyard_after = len(_cga.gating or ())
        _before = {frozenset((q.a, q.b)) for q in cy_gating_before}
        for q in (_cga.gating or ()):
            key = frozenset((q.a, q.b))
            for r in sorted(key):
                if key in _before and key in cy_charged.get(r, ()):
                    why = (f"its courtyard pair with "
                           f"{(set(key) - {r}).pop()} still gates "
                           f"({q.area_mm2}mm2)")
                elif key not in _before and r in moved_refs:
                    why = (f"its move created a gating courtyard pair with "
                           f"{(set(key) - {r}).pop()} ({q.area_mm2}mm2)")
                else:
                    continue
                repaired[:] = [x for x in repaired if x != r]
                if r not in unresolved:
                    unresolved.append(r)
                unresolved_claims.setdefault(r, [])
                if 'courtyard_blocking' not in unresolved_claims[r]:
                    unresolved_claims[r] = sorted(
                        unresolved_claims[r] + ['courtyard_blocking'])
                hint = ("" if body_model else
                        " (the seat search spaced pad boxes; --body-model "
                        "seats on check_assembly's occupancy)")
                notes.append(f"{r}: UNRESOLVED -- {why}{hint} -- NOT "
                             f"reported repaired")
    return {'moves': moves, 'repaired': repaired, 'unrepairable':
            unrepairable + failed, 'unresolved': unresolved,
            'violators': violators, 'notes': notes,
            # #1066: {ref: [rule, ...]} -- the grade claims that kept each
            # unresolved ref out of `repaired`: charged ones still present,
            # and ones its move created. Refs the run-7 pad/body re-grade
            # made unresolved are in `unresolved` but not here.
            'unresolved_claims': unresolved_claims,
            # #1182: check_assembly's gating courtyard pairs against the
            # baseline, before and after; None without a baseline.
            'courtyard_gating_before': (len(cy_gating_before)
                                        if cy_base is not None else None),
            'courtyard_gating_after': courtyard_after,
            'pad_report_before': {k: pads[k] for k in
                                  ('pad_conflicts', 'hole_conflicts',
                                   'oob_pad_count')},
            'grade_errors_before': len(graded.errors) if graded else None,
            # #975: edge seats kept short of the board-edge floor, by ref.
            'edge_floor_fallback': _floor_records_at_final_pose(
                state, edge_floor_fallback),
            # #1066 (b): {cap: {moved, tried: [...]}}; {} when the rung is
            # off or nothing was charged to a decap rule.
            'decap_rung': decap_rung}


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
        # #1068: the rules the `intent` basis counts -- what `intent 0->0`
        # is a count OF. Empty where no probe ran.
        'intent_rules': [],
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
                 edge_bands: Optional[Dict[str, float]] = None,
                 decap_claim_after_ics: Optional[bool] = None,
                 body_model: bool = False) -> Dict:
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
                     for r in sorted(pcb_data.footprints) if fnmatch.fnmatchcase(r, pat)}
    state = pose_score.make_state(
        pcb_data, pcb_file, clearance=clearance,
        board_edge_clearance=board_edge_clearance, grid_step=grid_step,
        extra_locked_refs=_extra_locked or None,
        # #701: the declared keep-outs reach the SEAT PREDICATE (see
        # `seed_from_intent`). `intent` is optional on this path, so the
        # inert default is what a caller without one gets.
        keepouts=intent.keepouts if intent else (),
        body_model=body_model)
    # #1182: the gate's overlap term on the GRADER's ruler -- check_assembly's
    # courtyard channel at the state's poses -- not on this state's rects,
    # which on a courtyard-less library are pad boxes: One-Air-Max's
    # `--reseat C27 D6 JP3 U8` was `GATE REFUSED, overlap 0.4323->0.4323`
    # while four courtyard pairs gated. Every `measure` below passes it.
    from placement import legality as _leg_r
    _cy_census = _leg_r.CourtyardCensus(
        pcb_data, pcb_file,
        intent_waivers=(intent.waiver_pairs() if intent else ()))

    def _cy_overlap(s):
        return _cy_census.grade({r: (p.x, p.y, p.rot)
                                 for r, p in s.parts.items()}).overlap_exact
    # NO `exclusive_zones=` here, and that is measured rather than assumed.
    # A draft of #797 passed one, with a paragraph explaining why it was safe.
    # A blind review showed this state is never consulted by a seat predicate
    # at all -- `reseat_scope` seats through `seed_from_intent`, which builds
    # its own -- so the channel was inert, and setting it to `()` changed
    # nothing across all three of this area's batteries. It also cost a second
    # `resolve_blocks` per call, whose `block_unresolved` problems were then
    # discarded unread. Dead code carrying a comment that asserts it matters is
    # worse than no code: anyone adding a seat call here must think about the
    # channel, and an inert kwarg would tell them it was already handled.
    refs_all = sorted(pcb_data.footprints)
    notes: List[str] = []
    must_lock = {r for pat in intent.must_lock
                 for r in refs_all if fnmatch.fnmatchcase(r, pat)}

    # ---- scope resolution --------------------------------------------------
    witnesses_before = _recon.damage_witnesses(state)
    if refs is None:
        scope = set(witnesses_before)
        scope_source = 'auto:damage_witnesses'
    else:
        scope = set()
        scope_source = 'explicit'
        for pat in refs:
            hits = [r for r in refs_all if fnmatch.fnmatchcase(r, pat)]
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
            if ref in getattr(state, 'outline_locked', ()):
                # #829, and it belongs FIRST: an outline owner is locked by
                # QuenchState, not by a stamp or a flag, so both arms below
                # would name a source that is not there. That is the same
                # defect this block's own comment records having fixed for
                # --lock, recurring for a new lock source -- so the rule is
                # not "special-case --lock", it is "every source names
                # itself".
                refused[ref] = ("draws the board outline -- moving it would "
                                "resize the board, which is not this tool's "
                                "to change (#829). Edit part and outline "
                                "together in KiCad if it must move")
            elif ref in _extra_locked:
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
        _empty_gate = _recon.measure(state, edge_bands or {},
                                     overlap=_cy_overlap)
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
                'no_pose_census': {}, 'evicted': [], 'edge_floor_fallback': {},
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
        #
        # #1068: and the TETHER rules (decap_distance, decap_pin_distance,
        # proximity) the quench's gate holds -- the same `_bundle['tethers']`.
        # Without them the `intent` basis read 0 -> 0 on a board printing
        # decap GRADE ERRORs on the very refs re-seated, and prune reverted a
        # seat made for a decap reason as a pure hpwl loss.
        probe = _q.IntentProbe(state, zones=_bundle['zones'],
                               tethers=_bundle.get('tethers'))

    # ---- seat ---------------------------------------------------------------
    before = _recon.measure(state, gate_bands, overlap=_cy_overlap)
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
        decap_claim_after_ics=decap_claim_after_ics,
        # #1151: a scope ref this pass cannot seat stays where it was; the
        # seed's staging row is for a board seeded from scratch.
        dispose_unseated=False,
        body_model=body_model,
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
                                     overlap=_cy_overlap,
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
    after = _recon.measure(state, gate_bands, overlap=_cy_overlap)
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
    if probe is not None:
        accept_basis['intent_rules'] = list(probe.rules)
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
        after = _recon.measure(state, gate_bands, overlap=_cy_overlap)
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
            # #975, same filter, and only for a board this pass wrote. Empty
            # in practice: re-seat drops its scope's edge declarations, so no
            # stage-1 seat reaches a scope ref. Carried so both paths, and
            # every place_seed summary, have the key.
            'edge_floor_fallback': ({r: v for r, v in
                                     (res.get('edge_floor_fallback') or {}).items()
                                     if r in scope} if accepted else {}),
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

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
from paste_apertures import pad_has_copper as _pad_has_copper
from connectivity import compute_mst_edges
from placement.parser import (courtyard_for_side, extract_courtyard_sides,
                              extract_locked_refs, warn_missing_courtyards)
from placement.utility import compute_footprint_bbox_local, snap_to_grid
from placement.pair_order import pair_inversions, ref_inversions
from placement.board_grid import (describe as describe_lattice,
                                  resolve_snap_lattice)
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

#: #1113: every label `QuenchState.candidate_veto` can return, in the order
#: candidate_valid asks (intent first, the tether last); 'unattributed' is a
#: refusal no label covered and is a bug for `tests/test_1113_pose_veto.py`.
VETO_CHECKS = ('intent', 'board_bbox', 'outline', 'waived_drill',
               'waived_pads', 'body_overlap', 'courtyard', 'container_pin',
               'body_contained', 'pads_under_body', 'keepout_band', 'pads',
               'tether', 'escape_overlap')
#: mm2 of .Fab body overlap `_body_overlap_at` counts: bodies that abut
#: (a shared edge, area 0) are not overlapping.
_BODY_OVERLAP_EPS = 1e-6

#: The floorplan rules the quench ENFORCES per move (#702), as opposed to the
#: ones it is merely graded on afterwards. Exported so `test_placement_ab.py`
#: and the docs detector read the enforced set FROM the engine instead of
#: re-typing it: a rule added here enters the A/B signal automatically, and a
#: rule removed to flatter a row trips a test rather than passing quietly.
#:
#: The three TETHER rules (#1043) are enforced through a separate channel,
#: `QuenchState._tether_terms`, because they are part-vs-PART: each is
#: measured by CALLING the grader's own function (`groups.elect_live`,
#: `floorplan.nearest_rail_cap`, `floorplan.proximity_reaches`) on footprints
#: posed at the live poses, over pairings `floorplan.tether_pairings` elects
#: once. Each is armed only by its own declared limit at error severity.
#:
#: The other NINE of the fifteen floorplan rules are deliberately absent, each
#: for its own reason -- `must_lock` and `edge_connector` are enforced by
#: FREEZING the ref (no pose satisfies or violates them), `zone_side` is
#: invariant under every move this engine can make and `assembly_side` (#837)
#: is invariant for the same reason one level up, `envelope` is a claim about
#: the intent file, `decap_ungraded` is a claim about what the GRADE covers
#: rather than about any pose (since #1142 an ERROR for a cap a --decaps-from
#: reference holds, which this gate still does not hold -- docs/floorplan-
#: intent.md, the quench table), `legality` is a whole-board budget rather than
#: a per-pose predicate, `pins_to_edge` is always-warn advice for a reviewer,
#: and `array_formation` (#1051) is held by construction -- a declared array
#: is a rigid group that only translates -- rather than priced per pose.
#:
#: The count said "six" and named six while nine were absent, and then said
#: "NINE" while eleven were (`proximity` and `pins_to_edge` arrived without
#: being added here). Stated as a number AND an enumeration
#: so the next addition is visibly missing from both.
INTENT_ENFORCED_RULES = ('zone_containment', 'zone_exclusive', 'keepout',
                         'decap_distance', 'decap_pin_distance', 'proximity')


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
# The declared-claim measurement, as three free functions (#698)
#
# `QuenchState` is not the only consumer any more. `seeder.reseat_scope` has to
# ask "is this part further outside its declared claims than it was" WITHOUT
# arming the per-pose seat gate -- `pose_score.make_state` passes that state
# `keepouts` and deliberately withholds `intent_zones`, because the re-seat's
# whole job is to move a part that is ALREADY violating and a monotone gate
# would make it refuse its own target. So the measurement is separable from the
# state that enforces it, and there is still exactly ONE of it.
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
                # rotations, matching `seeder.zone_gate`'s own form, so the two
                # cannot disagree about which branch a part is on.
                _anchor = not any(
                    _fp.zone_fits_courtyard(
                        _z['rect'], _p.grade_rect(0.0, 0.0, _r), _tol)
                    for _r in (_p.rot % 360, (_p.rot + 90) % 360))
                _terms.append(_IntentTerm(
                    'zone_containment', _z['name'], tuple(_z['rect']),
                    _tol, _anchor, None))
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
    exactly ONE place, and the seat gate and the quench gate read the same
    construction rather than two that happen to agree today.

    ONE DELIBERATE DIVERGENCE, stated because the sentence above used to claim
    there were none: the unresolved-zone filter below applies to the SEAT gate
    and not to the quench's #702 channel. It is a policy about which claims may
    STRAND a part, and the quench cannot strand one -- it only declines moves.
    Widening it to `build_zone_spec` would change #702's admitted-move set
    inside the wrong issue. `floorplan.rule_zone_exclusive` carries the SAME
    filter, so the seat gate and the GRADE -- the pair whose disagreement is an
    exit 4 on a correct board -- still agree exactly.

    THE `zone_containment` TERMS ARE DROPPED HERE, AND MUST NEVER REACH A SEAT
    GATE. `pose_score.make_state` withholds `intent_zones` from every seat
    state, and `tests/test_698_reseat_acceptance.py` arm H parses `seeder.py`
    to enforce that, because a MONOTONE containment gate would make a repair
    refuse its own target: the re-seat's whole job is to move a part that is
    already outside its zone back into it. `zone_exclusive` is the opposite
    shape -- a must-be-OUTSIDE claim, whose target is clean by definition --
    which is why it can be gated absolutely and containment cannot. The
    seeder's containment channel remains `zone_gate` plus the per-call
    `constraint`, which is anchor-aware and per-part.

    This filter is the one line that keeps that argument true. Widening it back
    to every term would re-open the bug arm H exists to prevent, so
    `test_698`'s runtime half asserts on the RESULT -- every term here is a
    `zone_exclusive` one, and a member binds none -- rather than on this
    source.
    """
    # A zone whose members did not RESOLVE binds nobody here. "Stranger" means
    # "not a member", so with an empty member set EVERY part on the board is
    # one, and enforcing such a zone evicts the very parts it was drawn
    # around. Measured on `fanout_output1.kicad_pcb`, whose block is
    # `group:`-shaped: resolved at bare `()` sources the seat gate refused all
    # 5 of its OWN members at their own poses, and `repair_placement` then
    # walked 4 of them 4.00mm out of the region their block reserved.
    #
    # Nothing is silently admitted by this: `resolve_blocks` already reports an
    # unresolved block as `block_unresolved`, an ERROR, so such a board fails
    # on the real finding rather than on a rule that cannot tell a member from
    # a stranger.
    #
    # Applied HERE rather than in `build_zone_spec`, so the quench's #702 gate
    # is untouched -- this is a SEAT-gate policy, and widening it would be a
    # behaviour change shipped inside the wrong issue.
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

    The keep-out slice is deliberately not frozen. `seeder.count_legal_poses`
    answers "how many seats would lifting keep-out X free" by temporarily
    removing X from `state.keepouts_for[ref]` and recounting -- and a frozen
    copy defeats that lift silently, because `pose_ok` reaches this gate
    through `candidate_valid`. Measured when it WAS frozen: the #701 census
    went `lifted=49` to `lifted=0` on arm Q's fixture, and a stranded part's
    verdict degraded from `keepout_blocks` to `no_movable_neighbour`, whose
    prose -- "NOTHING seated is near enough to be in the way" -- is verbatim
    the misleading answer that disclosure exists to replace.

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
            out.append(_fp.zone_escape(t.rect, rects[0], t.anchor)[0])
    return tuple(out)


class IntentProbe:
    """MEASUREMENT-ONLY declared-claim vectors for a state whose GATE has none.

    `pose_score.make_state` hands the re-seat `keepouts` and deliberately
    WITHHOLDS `intent_zones` (pose_score.py:84-90): the re-seat's job is to move
    a part that is ALREADY violating, and a monotone per-pose zone gate would
    make the repair refuse its own target. That argument is about the SEAT
    PREDICATE. It says nothing against measuring the same claims once before and
    once after the pass, which is what this does. It assigns to neither
    `state._intent_spec` nor `state._intent_active`, so `candidate_valid` sees
    exactly what it saw before -- and a source guard in
    `tests/test_698_reseat_acceptance.py` pins that, because the tempting
    "simplification" is to pass `intent_zones=` to `make_state` and re-open the
    bug `pose_score.py` describes.

    The spec is FROZEN here, unlike `intent_spec_for` -- which is deliberately
    live so `seeder.count_legal_poses` can lift a keep-out and recount
    (see `intent_spec`). Different consumer, opposite requirement: a
    before/after comparison of two vectors of DIFFERENT LENGTH is not a
    comparison at all, and a lift landing between the two snapshots would
    produce one silently.

    THE TETHER RULES (#1068), `decap_distance`, `decap_pin_distance` and
    `proximity`, from `tethers` (`floorplan.tether_gate_spec`, the same
    spec the quench's gate holds). Before this the probe measured the three
    zone/keep-out rules only, so the re-seat's `intent` basis read `0 -> 0`
    on a board printing four decap GRADE ERRORs on the very refs re-seated,
    and prune reverted a seat made for a decap reason as a pure hpwl loss.
    They are part-vs-PART, so they are held apart from `spec`: ONE list,
    each term counted ONCE however many refs it binds (a cap and every chip
    on its rail) -- appended per ref, a shared cap would count once per IC.
    Built by `QuenchState.tether_terms_for(keep_locked=True)` (the grade
    counts a claim whose refs are all locked; the gate drops it because it
    cannot refuse anything) and measured by `tether_graded_value`, which
    reads and writes none of the gate's caches. Nothing here assigns
    `state._tether_terms`, `state._tether_active` or `state.tethers`, so
    `_tether_gate` stays exactly as armed as it was.
    """

    def __init__(self, state, zones: Sequence[Dict] = (), refs=None,
                 tethers: Optional[Dict] = None) -> None:
        self.state = state
        rs = (sorted(state.parts) if refs is None
              else sorted(r for r in refs if r in state.parts))
        zone_spec = build_zone_spec(zones, state.parts, refs=rs)
        self.spec: Dict[str, Tuple[_IntentTerm, ...]] = {}
        for r in rs:
            s = intent_spec(zone_spec, state.keepouts_for, r)
            if s:
                self.spec[r] = s
        self.refs: Tuple[str, ...] = tuple(rs)
        want = set(rs)
        self.tethers: Tuple[_TetherTerm, ...] = tuple(
            t for t in (state.tether_terms_for(tethers, keep_locked=True)
                        if tethers else ())
            if want & set(t.refs))
        self._tethers_of: Dict[str, Tuple[int, ...]] = {}
        for i, t in enumerate(self.tethers):
            for r in set(t.refs):
                self._tethers_of[r] = self._tethers_of.get(r, ()) + (i,)

    @property
    def active(self) -> bool:
        return bool(self.spec) or bool(self.tethers)

    @property
    def rules(self) -> Tuple[str, ...]:
        """The rules this probe measures -- what its `count` is a count OF.
        A consumer printing the count prints these beside it."""
        got = {t.rule for ts in self.spec.values() for t in ts}
        got |= {t.rule for t in self.tethers}
        return tuple(r for r in INTENT_ENFORCED_RULES if r in got)

    def _tether_values(self) -> Tuple[float, ...]:
        """The COUNT's view: as the grade reads each term."""
        return tuple(self.state.tether_graded_value(t) for t in self.tethers)

    def _tether_guard_values(self) -> Tuple[float, ...]:
        """The LICENCE's view: as the gate reads each term. They differ for a
        decap pair past the search radius -- the grade stops grading it
        (`decap_ungraded`: a warn, or per cap an error the tether count
        still does not see, #1142), so the count drops; the gate keeps
        measuring it, so the licence sees a cap that walked further from its
        IC as the regression it is, not as a fix (phase-2 verifier: esp_prog
        C3, radius 2.2, moved 1mm out, read `1 -> 0` and licensed)."""
        return tuple(self.state.tether_gate_view_value(t)
                     for t in self.tethers)

    def terms(self, ref) -> Tuple[float, ...]:
        """`ref`'s claim vector at its CURRENT pose: its zone/keep-out terms,
        then every tether term that binds it.

        The zone terms are part-vs-DECLARED-GEOMETRY, so nothing another part
        does changes them. The tether terms are part-vs-PART, so another
        part's move DOES change them -- which is still legal to hand to
        `reconstruct.prune_assignment` as a per-ref callable, because prune
        samples it either side of restoring `ref` ALONE: nothing else moves
        between the two samples, so a rise is `ref`'s doing.

        A tether term enters as its EXCESS over its limit, never its raw
        distance: prune refuses a revert on any rise, and a cap moving from
        1.0 to 1.5mm under a 2mm limit is no finding -- only a revert that
        leaves a term further past its limit is. That is `tether_ok`'s rule
        (within the limit, or no worse), in the vector prune compares.
        """
        s = self.spec.get(ref)
        out = intent_term_values(s, self.state.parts[ref].grade_rects()) if s \
            else ()
        idx = self._tethers_of.get(ref, ())
        if idx:
            out = tuple(out) + tuple(
                max(0.0, self.state.tether_gate_view_value(self.tethers[i])
                    - self.tethers[i].threshold - legality.EPS)
                for i in idx)
        return out

    def snapshot(self) -> Dict:
        """Every bound ref's vector, plus the BREACH COUNT and its by-rule split.

        A term is breached when its value is above its own `threshold` -- the
        comparison `intent_clear` makes, and the same event `floorplan.grade`
        raises a `Violation` for, so the count is in the GRADE's currency while
        every underlying compare stays in the TERM's.

        The count is only ever a TRIGGER, never a guard. On its own it carries
        the aggregation trap `_IntentTerm` names: a part hopping from keep-out A
        into keep-out B reads `1 -> 1`, which a monotone rule would admit. The
        guard is `licence()` below, on the VECTORS.
        """
        vecs = {r: intent_term_values(self.spec[r],
                                      self.state.parts[r].grade_rects())
                for r in sorted(self.spec)}
        count = 0
        by_rule: Dict[str, int] = {}
        for r, vals in vecs.items():
            for v, t in zip(vals, self.spec[r]):
                if v > t.threshold:
                    count += 1
                    by_rule[t.rule] = by_rule.get(t.rule, 0) + 1
        # The grade's own comparison for these rules: past the limit by more
        # than EPS (`rule_decap_distance`, `rule_decap_pin_distance`,
        # `rule_proximity`), the same one `tether_failures` makes.
        tvals = self._tether_values()
        for v, t in zip(tvals, self.tethers):
            if v > t.threshold + legality.EPS:
                count += 1
                by_rule[t.rule] = by_rule.get(t.rule, 0) + 1
        return {'count': count, 'by_rule': by_rule, 'terms': vecs,
                'tethers': self._tether_guard_values()}

    def licence(self, before: Dict, after: Dict) -> Tuple[bool, List[Tuple]]:
        """(ok, risen) -- no declared term binding a probed ref may RISE.

        TERMWISE and never summed (`_IntentTerm`), which is what makes
        `snapshot()['count']` safe to use as the acceptance trigger: the A -> B
        keep-out hop that the count cannot see is a term that ROSE, and this
        refuses it. `risen` names each one `(ref, rule, name, before, after)` --
        the #701 doctrine that a claim which refuses is a NAMED verdict.
        """
        risen: List[Tuple] = []
        b_terms, a_terms = before.get('terms', {}), after.get('terms', {})
        for ref in sorted(self.spec):
            bv, av = b_terms.get(ref, ()), a_terms.get(ref, ())
            if len(bv) != len(av):
                # Cannot happen with a frozen spec; if it ever does, refusing is
                # the only honest answer -- see the class docstring.
                risen.append((ref, 'spec', 'length-changed', len(bv), len(av)))
                continue
            for t, b, a in zip(self.spec[ref], bv, av):
                if a > b + legality.EPS:
                    risen.append((ref, t.rule, t.name, b, a))
        bt, at = before.get('tethers', ()), after.get('tethers', ())
        if len(bt) != len(at):
            risen.append(('*', 'spec', 'tethers-length-changed', len(bt),
                          len(at)))
        else:
            # Per TERM, like the gate's `tether_ok`: a term within its limit
            # after the pass is no finding, however it moved; one past it
            # must not have got worse.
            for t, b, a in zip(self.tethers, bt, at):
                if a > t.threshold + legality.EPS and a > b + legality.EPS:
                    risen.append((','.join(t.refs[:2]), t.rule, t.name, b, a))
        return (not risen), risen


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


class _Part:
    __slots__ = ('ref', 'pads_local', 'pin_count', 'bounds_by_rot',
                 'seed_x', 'seed_y', 'x', 'y', 'rot', 'locked',
                 'nets', 'halo', 'footprint_name', 'orig_rot',
                 'side', 'has_tht', 'sides', 'tht_by_rot', 'padbox_local',
                 'padbox_by_rot', 'grade_by_rot')

    def __init__(self, ref, fp, courtyard_sides, locked, halo_base, halo_coef,
                 body_local=None):
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
        # #916. `body_local` is `placement.body`'s `occupancy_local` for this
        # ref, supplied by QuenchState under `body_model=True`. None keeps the
        # inlined ladder below, which is what every caller got before #916 and
        # what the default still gets -- so the OFF arm of the A/B is this
        # file unchanged, not a re-derivation that happens to agree.
        #
        # OCCUPANCY, not the bare body, and not the drawn body: this rect is
        # what `pose_ok`/`candidate_valid` seat against, i.e. "what does this
        # part occupy". `legality.part_local_bounds` already answers the same
        # question with `occupancy_local` (legality.py:1213-1216), so before
        # this the GRADER and the ENFORCER measured different rectangles for
        # the same part -- the grader courtyard-union-pads, the search bare
        # courtyard or a pad box. `QuenchState.fab_rect` deliberately does NOT
        # move: containment is a different question with a fab-only
        # calibration, and its own docstring calls a courtyard-based
        # containment test a false-veto machine.
        #
        # #1182: the GRADE ladder (courtyard, else pad bbox) is kept beside
        # it, because the floorplan grade -- zones, keep-outs, edge claims,
        # the board term -- reads a DEFAULT state's rects (`_grade_ctx` builds
        # one). Armed, only the neighbour/body currency moves to occupancy;
        # every intent question asks `grade_rect`, so the search cannot refuse
        # a zone seat the grade accepts. Unarmed the two are ONE dict, so the
        # default path is this file before #1182.
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
        # those are the poses _candidate_rotations offers such a part, and
        # build_neighbor_lists unions bounds_by_rot over the movable
        # same-footprint group, so recording them here is what keeps the
        # pruning boxes covering every pose the group can reach.
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
        self.halo = halo_base + halo_coef * math.sqrt(max(self.pin_count, 1))

    def ensure_rotation(self, rot):
        """Fill both rotation caches for `rot`, as given (a caller that
        wants the normalised key normalises first). THE one place an entry is
        made after construction (#1206): the far side is turned cluster by
        cluster (`legality.rotate_far`), so no cache can flatten a FarSide
        to its union box -- four inline copies of this fill (three in the
        seeder, one in the swap) each could."""
        if rot not in self.bounds_by_rot:
            self.bounds_by_rot[rot] = _rotate_local_bounds(
                *self.bounds_by_rot[0.0], rot)
        if rot not in self.grade_by_rot:
            self.grade_by_rot[rot] = _rotate_local_bounds(
                *self.grade_by_rot[0.0], rot)
        if self.tht_by_rot is not None and rot not in self.tht_by_rot:
            self.tht_by_rot[rot] = legality.rotate_far(self.tht_by_rot[0.0],
                                                       rot)

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
        courtyard, else pad bbox (#1182). `rect()` itself under
        `body_model=False`; the body/neighbour currency (`rect`) becomes
        occupancy under `body_model=True` and this does not."""
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

    #: #1043: measure every tether term exactly, skipping the bound shortcuts
    #: in `_tether_value`. Off in production; the test that proves the
    #: shortcuts change no decision runs a quench both ways.
    _exact_tethers = False

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
                 # `seeder.edge_seat_ok` under an ABSOLUTE policy, and since
                 # #702 by `candidate_valid` under a MONOTONE one. The quench
                 # OBJECTIVE still never reads it: this is a hard gate on which
                 # poses exist, not a term in the cost.
                 keepouts: Optional[Sequence[Dict]] = None,
                 # --- #702 declared zones. APPENDED after `keepouts` for the
                 # same positional-binding reason as the #548 block above, and
                 # empty by default for the same bit-identity reason. Plain
                 # data from `floorplan.resolve_intent_gate`, never an
                 # `Intent`: the engine must not import that schema to run, and
                 # `floorplan.grade` builds a state of its own that has to keep
                 # measuring independently of whatever the optimizer was gated
                 # on (tests/test_701_keepout_predicate.py:395).
                 intent_zones: Optional[Sequence[Dict]] = None,
                 # --- #797 declared EXCLUSIVE zones, for the SEAT predicate.
                 # A SEPARATE parameter from `intent_zones` on purpose, and not
                 # merely a different spelling of it: this one carries the
                 # must-be-OUTSIDE slice alone, so it can be gated ABSOLUTELY
                 # without arming the monotone containment gate that
                 # `pose_score.make_state` and test_698 arm H exist to keep off
                 # the seat paths. Same plain data (`floorplan.zone_entries`),
                 # same empty-by-default bit-identity, and the quench itself
                 # never passes it -- its zone_exclusive enforcement stays
                 # where #702 put it, in `intent_ok`.
                 exclusive_zones: Optional[Sequence[Dict]] = None,
                 # --- #916. The SEARCH's body currency. APPENDED for the same
                 # positional-binding reason as the #548 block above, and False
                 # by default so this commit moves NO number: at False every
                 # part takes the inlined courtyard-or-pad-box ladder it took
                 # before, so the A/B's OFF arm is the old code rather than a
                 # re-derivation that happens to agree. #896 wired every
                 # GRADING consumer to `placement.body` and deliberately left
                 # the search behind, because `pose_ok` reads these baked
                 # bounds and so the change moves which BASIN the anneal lands
                 # in -- an engine change owing its own A/B, which is what
                 # flipping this default is gated on.
                 body_model: bool = False,
                 # --- #893 pin-order facing term. APPENDED for the same
                 # positional-binding reason as the #548 block above, and 0.0
                 # by default: see `_facing_cost` for why the default is a
                 # measurement question and not timidity.
                 facing_weight: float = 0.0,
                 # --- #893 declared rotations, from the intent gate. APPENDED
                 # for the same positional-binding reason as the #548 block
                 # above, and empty by default so an undeclared board keeps the
                 # full lattice and is bit-identical.
                 declared_rotations: Optional[Dict] = None,
                 # --- #1043 declared tethers, from the intent gate's
                 # `tethers` key. APPENDED for the same positional-binding
                 # reason, and empty by default: no term is built, the gate
                 # is one bool load, and the quench is bit-identical.
                 tethers: Optional[Dict] = None):
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
        # #975: the edge floor ITSELF. `edge_gate.margin` is the max of the two
        # floors, so the pad-copper edge check an edge seat makes (a floor, not
        # a copper clearance) cannot be recovered from it.
        self.board_edge_clearance = board_edge_clearance
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
        # #916. One read for the whole board when armed, never per part:
        # `board_bodies` makes three regex passes over the file, and the
        # seeder builds states repeatedly. `{}` when off, so `.get(ref)`
        # below yields None and `_Part` keeps its own ladder.
        self.body_model = bool(body_model)
        self.facing_weight = facing_weight
        #: #893. {ref: (rotation, candidates)} from the intent gate. Empty when
        #: nothing is declared, and every consumer falls back to the lattice --
        #: so a board with no declaration is bit-identical.
        self.declared_rotations: Dict[str, object] = dict(
            declared_rotations or {})
        body_locals: Dict[str, object] = {}
        body_sources: Optional[Dict[str, str]] = None
        if self.body_model:
            from placement import body as _body
            body_sources = {}
            for _ref, _geom in _body.board_bodies(pcb_data, pcb_file).items():
                if _geom.occupancy_local is not None:
                    body_locals[_ref] = _geom.occupancy_local
                    body_sources[_ref] = _geom.source
        locked_refs = set(extract_locked_refs(pcb_file))
        if extra_locked_refs:
            locked_refs |= extra_locked_refs
        ignore = ignore_net_ids or set()

        self.parts: Dict[str, _Part] = {}
        no_courtyard = []
        outline_locked = []   # #829, reported below
        for ref, fp in pcb_data.footprints.items():
            # `fp.pads`, not `non_aperture_pads` -- the one #1143 site left
            # reading every pad, deliberately. A part whose only pads are
            # paste/mask apertures (a logo) stays MOVABLE here: the seeder
            # seats such a part by its courtyard when the intent fixes its
            # pose (test_1051_hardening's copperless logo), and taking it
            # down this branch would lock it as a static obstacle, which the
            # fixed-pose stage then refuses as "already placed" (Phase-1
            # verifier). A truly pad-less footprint is refused that way too,
            # which is a separate, older question.
            if not fp.pads:
                # Zero-pad footprints (graphics-only mechanical parts, logos
                # with a courtyard) used to be dropped entirely -- neither
                # movable NOR an obstacle, so the optimizer walked parts onto
                # them. With a drawn courtyard they now enter as locked static
                # obstacles; without one there is no geometry to respect.
                if ref in courtyards:
                    self.parts[ref] = _Part(ref, fp, courtyards, True,
                                            halo_base, halo_coef,
                                            body_locals.get(ref))
                continue
            # #829: a footprint that draws part of the BOARD's own boundary is
            # never this tool's to move -- its pose transforms that Edge.Cuts
            # geometry, so moving it resizes the board, which is a mechanical
            # decision the user owns. A fourth lock INPUT with the same
            # property the other three have (see the union note in `quench()`):
            # no un-lock operator, so it can never override what a caller
            # asked for.
            #
            # `owns_board_outline`, NOT `owns_edge_cuts`. Geometry parented to
            # a footprint is usually a relief the designer bound to the part so
            # it travels with it -- crkbd draws 184 per-LED windows that way,
            # and #628 exempts a part's own milled ring from the edge-margin
            # test precisely to keep such a part placeable (without it run 20's
            # SW2 had 0 legal poses of 14884). Only geometry lying outside the
            # board-level outline is the board's.
            owns_outline = getattr(fp, 'owns_board_outline', False)
            locked = (ref in locked_refs
                      or owns_outline
                      or (move_refs is not None and ref not in move_refs))
            if owns_outline:
                outline_locked.append(ref)
            if ref not in courtyards:
                no_courtyard.append(ref)
            self.parts[ref] = _Part(ref, fp, courtyards, locked,
                                    halo_base, halo_coef,
                                    body_locals.get(ref))
            # A part with NO connected pins (mounting hole, NPTH, fiducial) is
            # invisible to the airwire cost -- only halo/edge decide where it
            # goes, which is how holes wander. Frozen by default; the caller
            # frees them explicitly with move_unconnected (--move-unconnected).
            if self.parts[ref].pin_count == 0 and not move_unconnected:
                self.parts[ref].locked = True
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
        warn_missing_courtyards(no_courtyard, 'quench',
                                sources=body_sources)

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
        # #1098: plus the mating region of every PCB-edge plug on the board
        # (derived from the footprint; a declared `mating:<ref>` wins), so
        # no seat, nudge or swap puts a part on a USB tongue whatever the
        # intent says. Empty on a board with no such plug.
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
        # parts it kept off the tongue would be free to return (#1098
        # review). So the search never moves it -- as with a KiCad lock.
        # Only while it IS seated (`seated_plugs`): a plug a declared
        # `mating:` keep-out names but which sits in the staging pile, or
        # hangs across an edge, must stay free, or the seeder writes it back
        # where it was and reports nothing unseated.
        for _ref in _fpk.seated_plugs(self.keepouts, pcb_data, pcb_file):
            if _ref in self.parts:
                self.parts[_ref].locked = True
        self.keepouts_for: Dict[str, Tuple[Dict, ...]] = {}
        if self.keepouts:
            from . import floorplan as _fp
            for _ref, _p in self.parts.items():
                _binding = _fp.keepouts_for_ref(self.keepouts, _ref, _p.sides)
                if _binding:
                    self.keepouts_for[_ref] = _binding

        # --- #702 declared claims, resolved ONCE per state ------------------
        # The zone half is built by the free `build_zone_spec` (#698), which is
        # the SAME construction `seeder.reseat_scope` uses for its acceptance
        # measurement -- a second copy here is how the two would come to
        # disagree about which parts a block binds.
        #
        # `_intent_active` is False on every board that declares nothing --
        # which is every board in the corpus today -- and `candidate_valid`
        # guards on it before any arithmetic, so the whole channel costs one
        # bool load and one branch and the objective is bit-identical.
        self.intent_zones = tuple(intent_zones or ())
        self._intent_spec: Dict[str, Tuple[_IntentTerm, ...]] = build_zone_spec(
            self.intent_zones, self.parts)
        self._intent_active = bool(self._intent_spec or self.keepouts_for)

        # --- #797 declared EXCLUSIVE zones, for the SEAT predicate ----------
        # Resolved once per state, like `keepouts_for`, and for the same
        # reason: membership and the side filter are pose-invariant, and
        # `_try_place` evaluates thousands of poses per part.
        #
        # DELIBERATELY NOT FOLDED INTO `_intent_spec`, and not counted in
        # `_intent_active`. Those drive `candidate_valid`'s MONOTONE
        # `intent_ok`, which admits any pose termwise no worse than the one the
        # part is IN -- and before a part is seated, that is its generator
        # pile coordinate. A stranger whose pile coordinate already sits inside
        # a reserved zone would then be admitted to every pose no worse than
        # that, i.e. seated inside it: #797's own bug, back through the door
        # `pose_ok`'s docstring names for keep-outs. Folding it in would also
        # change `intent_spec_for`'s arity, which is the coupling
        # `seeder.count_legal_poses` clears `_inc_intent` for.
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
        # Refusal tally, for `metrics_out['intent_gate']`. Without it, "the
        # gate refused nothing" and "the gate is not wired" are the same
        # observation -- which is the whole anti-vacuity device for #702.
        self.intent_rejected: Dict[str, int] = {}
        self.intent_rejected_by_site: Dict[str, int] = {}

        # --- #1043 declared TETHERS -----------------------------------------
        # The part-vs-PART claims (`decap_distance`, `decap_pin_distance`,
        # `proximity`), which is why they are not folded into `_intent_spec`:
        # that channel's terms are part-vs-declared-geometry, and its
        # incumbent cache (`_incumbent_intent`) is keyed by the moving ref on
        # exactly that assumption. A tether term reads the poses of every ref
        # it names, so its incumbent is cached PER TERM and a move of any of
        # them invalidates it (both caches are cleared on every move).
        #
        # The pairings are elected ONCE, here, by `floorplan.tether_pairings`
        # -- the grader's own election on the board as it stands; a decap
        # term re-runs the cap's election per pose over the chips on its
        # rail, because the grade re-elects -- and every
        # value is measured by CALLING the grader's functions on footprints
        # posed at the live (or candidate) poses: `groups.elect_live`,
        # `floorplan.nearest_rail_cap`, `floorplan.proximity_reaches`.
        #
        # Empty unless the intent declares a tether limit at error severity,
        # and then `_tether_active` is False and no path below reads anything.
        self._tether_terms: List[_TetherTerm] = []
        self._tethers_of: Dict[str, Tuple[int, ...]] = {}
        self._inc_tval: Dict[int, float] = {}
        self._tgap: Dict[Tuple, object] = {}
        self._posed: Dict[Tuple, object] = {}
        self._bounds: Dict[Tuple, object] = {}
        self._tether_override: Optional[Dict[str, Tuple[float, float,
                                                        float]]] = None
        self._tether_bodies = None
        self.tethers = dict(tethers or {})
        if self.tethers:
            self._build_tethers()
        self._tether_active = bool(self._tether_terms)

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
        if self.container_refs:
            print(f"  container footprint(s) (>= {CONTAINER_RATIO:.0%} of "
                  f"the board, a pin frame or an outline -- frame, not "
                  f"body): " + ', '.join(
                      f"{r} ({k})"
                      for r, k in sorted(self.container_kinds.items())))

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
                model=pad_model,
                # #1031: the board's rule-area keep-outs; inert (None
                # inside) on a board that declares none with tracks
                # forbidden.
                keepouts=legality.RuleAreaKeepouts.for_board(
                    pcb_data, clearance, pcb_file))

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

    def _facing_cost(self, ref, x=None, y=None, rot=None,
                     exclude: Optional[Set[str]] = None) -> float:
        """Price the pin-ORDER a pose forces (#893). Off at weight 0.0.

        `pair_order.ref_inversions` -- the SAME lower bound
        `placement_score.pin_order_crossings` reports, called rather than
        re-derived. For each partner sharing >= 2 scoring nets, project both
        parts' escape pads onto the channel cross-section and count order
        inversions: in a two-sided channel each inverted pair must cross at
        least once, so this is a floor on the crossings any router must pay.

        WHY THIS IS NOT `_orient_cost`, which already "rewards a pose whose
        pads FACE the nets they serve": that term is a DIRECTION, summed per
        pad against a net centroid, and it is blind to ORDER. Two parts can
        point their pads straight at each other and still have every net
        crossed -- which is exactly run 5's U3, where rotating 180 degrees took
        the same nets from 4/7 routed to 7/7 while the airwire lengths barely
        moved. `_orient_cost` cannot see that; this can. They are complementary
        and both are off by default.

        WHY IT IS OFF BY DEFAULT, and why that is not timidity:
        `pair_order`'s own header records the standing decision that these
        metrics "deliberately do NOT join quench.total_cost", and
        `docs/placement-optimization.md` is a file-length negative result about
        adding proxies to this objective -- "proxy-routability correlation is
        weak", "proxies propose, the router disposes". A lower bound is a
        better citizen than a correlational proxy (improving it cannot be
        gamed), but "better citizen" is an argument, not a measurement. The
        weight is the way to MEASURE it; `tests/test_placement_ab.py` is the
        way to decide it.

        Cost: `ref_inversions` is ~150-800 us/call depending on the board even
        after the #893 hot-path work, so this is the most expensive term in
        `part_geometry_cost` by an order of magnitude. It returns before
        touching anything at weight 0, so a default run pays nothing.
        """
        if self.facing_weight <= 0.0:
            return 0.0
        if exclude:
            # NUDGE ONLY, and this is a correctness bound rather than a
            # simplification. Inversions are a PAIR quantity evaluated against
            # the partner's LIVE pose, so the two multi-part evaluators cannot
            # price it:
            #
            # * the SWAP passes `exclude={partner}` to each half and adds the
            #   a-b halo and align pairs back at the candidate poses (see the
            #   add-back below the swap loop). There is no such add-back for a
            #   term that needs BOTH parts moved at once, and scoring `a` at
            #   its candidate against `b` still at its PRE-swap pose is simply
            #   the wrong geometry.
            # * the GROUP translate passes `exclude=members - {r}` precisely
            #   because intra-group geometry is invariant under a rigid move.
            #   Facing is not: `r` would be displaced while its in-group
            #   partners are not, turning an invariant into a bias on block
            #   moves.
            #
            # `exclude` is non-empty only on those two paths, so returning 0.0
            # here leaves the single-part nudge -- where `ref_inversions` IS
            # the exact delta -- as the only consumer, and leaves the group and
            # swap objectives bit-identical to their unarmed selves.
            return 0.0
        return self.facing_weight * ref_inversions(self, ref, x, y, rot)

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
        # #893. Same hook, same contract: 0.0 before touching geometry when the
        # weight is 0, so a default run is bit-identical. Deliberately NOT a
        # separate move phase -- a phase minimising `nets+geo+facing` while the
        # nudge minimises `nets+geo` gives the loop two objectives, and the
        # nudge's own rotation loop then reverts every rotation the phase makes
        # (its candidate list always contains the current angle), so `moves`
        # never reaches 0 and `improved` sums two currencies.
        pen += self._facing_cost(ref, x, y, rot, exclude)
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
        # The BOARD term on the grade ladder (#1182), as candidate_valid.
        near = self._edges_near(ref) if self.edge_gate.active else None
        board = self.edge_gate.rect_outside_amount(
            part.grade_rect(x, y, rot), exact=bool(near), edges=near,
            skip_rings=self._owned_rings(ref))
        overlap = 0.0
        if limit is not None and board > limit:
            return board, overlap
        if self._neighbors is not None and ref in self._neighbors:
            others = ((o, self.parts[o]) for o in self._neighbors[ref])
        else:
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

        The keep-out slice is deliberately not frozen. `seeder.count_legal_poses`
        answers "how many seats would lifting keep-out X free" by temporarily
        removing X from `state.keepouts_for[ref]` and recounting -- and a frozen
        copy defeats that lift silently, because `pose_ok` reaches this gate
        through `candidate_valid`. Measured when it WAS frozen: the #701 census
        went `lifted=49` to `lifted=0` on arm Q's fixture, and a stranded
        part's verdict degraded
        from `keepout_blocks` to `no_movable_neighbour`, whose prose --
        "NOTHING seated is near enough to be in the way" -- is verbatim the
        misleading answer that disclosure exists to replace.

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
        improving on (`seeder.pose_ok`), so a seat search demands cleanliness
        rather than non-worsening -- and `tests/test_701_keepout_predicate.py`
        seats a part whose current pose is fully inside a keep-out and asserts
        REFUSAL, which the monotone rule below would admit.
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

        One loop, so `seeder.pose_ok` and `seeder.edge_seat_ok` stop owning a
        copy each -- the doctrine `floorplan.keepout_hit`'s own header states.
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
        """MONOTONE: the QUENCH policy. A pose is admitted when every term is
        clean, or -- TERMWISE -- no worse than the pose the part is in.

        Termwise and never summed, and never traded across terms: a part may
        not buy its way into keep-out B by leaving keep-out A.

        The incumbent is computed only on the branch that needs it, and cached
        -- the same trade `candidate_valid` makes at its own escape branch
        ("Only now, on a rejected candidate, is the incumbent's legality worth
        computing"). On a compliant board it is never computed at all.

        NON-STRICT (`<=`), unlike the #456 off-board branch's strict compare.
        That branch needs strictness because it hands out a licence to be
        ILLEGAL; this one does not, because acceptance is still governed by
        `current - best > EPS + min_gain_per_mm * dist`, a strictly decreasing
        potential. The gate is a FILTER on which poses exist, not a descent
        direction, so equality cannot cycle. Strictness here would instead be a
        bug: `keepout_hit` reports a circle as a fabricated 1.0 marker, so
        `<` would freeze a part already inside a circle unless it could clear
        the whole circle in a single nudge.

        THE TRADE THIS MAKES, STATED. Because the conjunct sits ABOVE the #456
        off-board escape branch and returns rather than setting a flag, a part
        that is OFF THE OUTLINE and whose only reachable homeward poses lie in
        a declared keep-out stays off the outline. Measured on a fixture: the
        ungated run brings it 1.50mm inside the usable inset, the gated run
        leaves it 2.50mm outside. That trades a `keepout` finding for an
        off-board part, and CLAUDE.md ranks a part whose pad copper lies
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

    def _note_intent_refusal(self, ref, site, rects=None,
                             x=None, y=None, rot=None) -> None:
        """Tally ONE refusal against `site`, and the rules behind it.

        `by_site` counts REFUSALS and `by_rule` counts BLOCKING TERMS, so the
        two do not sum to each other in either direction: one refused pose can
        break two claims at once, and a pose refused for a reason this call was
        not given a ref for contributes to `by_site` alone. Stated here because
        a reader will otherwise assume `by_rule` partitions `rejected`.
        """
        self.intent_rejected_by_site[site] = (
            self.intent_rejected_by_site.get(site, 0) + 1)
        _first = None
        for rule, _name, _c, _u in self.intent_blockers(ref, x, y, rot, rects):
            self.intent_rejected[rule] = self.intent_rejected.get(rule, 0) + 1
            _first = _first or f"{rule}:{_name}"
        if self._why is not None and site == 'candidate_valid':
            self._veto('intent', _first)    # #1113

    def _note_swap_refusal(self, ra, rb) -> None:
        """Attribute a refused swap to whichever HALF of it was refused.

        Asking only about `ra` loses the case where the partner's claims are
        what refused: `by_site` showed the swap and `by_rule` stayed empty,
        which reads as a refusal with no reason. Both halves are checked
        because either can be the one that failed, and both can.
        """
        pa, pb = self.parts[ra], self.parts[rb]
        self.intent_rejected_by_site['swap'] = (
            self.intent_rejected_by_site.get('swap', 0) + 1)
        for who, (x, y, rot), cur in ((ra, (pb.x, pb.y, pb.rot), pa.rot),
                                      (rb, (pa.x, pa.y, pa.rot), pb.rot)):
            for rule, _n, _c, _u in self.intent_blockers(who, x, y, rot):
                self.intent_rejected[rule] = self.intent_rejected.get(rule, 0) + 1
            # #1117: the declared-rotation half has no intent_blockers rule.
            if not _declared_admits(self.declared_rotations.get(who), rot,
                                    cur):
                self.intent_rejected['rotation'] = (
                    self.intent_rejected.get('rotation', 0) + 1)
        if self._tether_active:
            # #1043: both halves at once, since a tether between the two (two
            # caps on one pin's rail) reads both poses.
            for rule, _n, _c, _u in self.tether_failures(
                    {ra: (pb.x, pb.y, pb.rot), rb: (pa.x, pa.y, pa.rot)}):
                self.intent_rejected[rule] = self.intent_rejected.get(rule, 0) + 1

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
        blocker)`: the conjunct that refused it (`VETO_CHECKS`) and the part
        it was refused against, when there is one (#1113). It CALLS
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
        # floorplan grade reads); the neighbour layers below ask `rect`, which
        # is occupancy under body_model. One object unarmed.
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
            self._note_intent_refusal(ref, 'candidate_valid', grects)
            return False
        legal = not (grect[0] < self.usable[0] or grect[1] < self.usable[1]
                     or grect[2] > self.usable[2] or grect[3] > self.usable[3])
        if not legal and self._why is not None:
            self._veto('board_bbox')
        # Real outline / cutout gate, three-level short-circuit: board-level
        # opt-out, cached per-part reachable-edge list, then the exact test
        # against only those edges.
        if legal and self.edge_gate.active:
            near = self._edges_near(ref)
            if near and self.edge_gate.rect_blocked(
                    grect, edges=near, skip_rings=self._owned_rings(ref)):
                legal = False
                if self._why is not None:
                    self._veto('outline')
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
                if self._neighbors is not None and ref in self._neighbors:
                    others = ((o, self.parts[o]) for o in self._neighbors[ref])
                else:
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
            # #1043: the tether conjunct LAST, at every `return True`, not
            # beside the #702 check above. It is the one conjunct that poses
            # footprints and walks pad pairs, so it runs only on a pose every
            # cheaper test already admitted. It is still a conjunct on the
            # escape branch below: a part coming home from off the board may
            # not strand its caps on the way.
            return self._tether_gate(ref, x, y, rot)
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
            if (self._why is not None and cand_overlap > EPS_IMPROVE
                    and self._why.get('check') in ('board_bbox', 'outline')):
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
        return self._tether_gate(ref, x, y, rot)

    def swap_intent_ok(self, ra, rb) -> bool:
        """May these two parts exchange poses, declared-intent-wise? (#702)

        The swap phase does not call `candidate_valid` on ANY path, and that is
        deliberate: a swap preserves the OCCUPIED SPACE, so the geometry every
        other part sees is unchanged. That argument is true of clearance and
        FALSE of a claim that binds a REF -- exchanging two identical decaps
        moves A to B's pose, where B's zone, B's keep-out bindings and B's
        exclusive-zone exemptions applied, not A's. Occupied space cannot see
        that, so this is its own conjunct rather than a relaxation of one.

        Each part's OWN claims at its PARTNER's pose, each against its OWN
        incumbent. Atomic: both halves must hold and nothing is applied, so
        there is no ordering hazard between them.
        """
        pa, pb = self.parts[ra], self.parts[rb]
        # #1117: a declared ROTATION binds a ref the same way. The swap hands
        # each part the other's angle, and the #893 pin lived only in the
        # nudge's candidate list, so two parts of one footprint declared at
        # different angles traded them and the seed graded clean (there is
        # no rule_rotation to catch it).
        if self.declared_rotations and not (
                _declared_admits(self.declared_rotations.get(ra), pb.rot,
                                 pa.rot)
                and _declared_admits(self.declared_rotations.get(rb), pa.rot,
                                     pb.rot)):
            return False
        return (self.intent_ok(ra, pb.x, pb.y, pb.rot)
                and self.intent_ok(rb, pa.x, pa.y, pa.rot)
                and (not self._tether_active
                     or self.tether_ok({ra: (pb.x, pb.y, pb.rot),
                                        rb: (pa.x, pa.y, pa.rot)})))

    # ----- #1043 tethers ----------------------------------------------------

    def _build_tethers(self) -> None:
        """Elect the pairings once and freeze them as the GATE's
        `_TetherTerm`s (`tether_terms_for`, with locked terms dropped)."""
        terms = self.tether_terms_for(self.tethers, keep_locked=False)
        by_ref: Dict[str, List[int]] = {}
        for i, t in enumerate(terms):
            for r in set(t.refs):
                by_ref.setdefault(r, []).append(i)
        self._tether_terms = terms
        self._tethers_of = {r: tuple(v) for r, v in by_ref.items()}

    def tether_terms_for(self, tethers: Dict, *, keep_locked: bool
                         ) -> List[_TetherTerm]:
        """The `_TetherTerm`s `tethers` (`floorplan.tether_gate_spec`) elects
        on the board as it stands. Assigns NOTHING on the state, so a
        measurement-only caller (`IntentProbe`, #1068) can build them without
        arming `_tether_gate`.

        `keep_locked=False` is the gate's choice: a term none of whose refs
        can move is dropped, since no move of this engine changes it and it
        cannot refuse anything. A MEASUREMENT keeps it (`True`), because the
        grade still counts it. A proximity term is expanded to one per REACH
        the rule reports (per declared subject pad, or one for the pair),
        counted on the board as it stands.
        """
        from . import floorplan as _fp
        rows = _fp.tether_pairings(tethers, self.pcb_data)
        nets = {n.name: nid for nid, n in (self.pcb_data.nets or {}).items()}
        terms: List[_TetherTerm] = []
        for row in rows:
            refs = tuple(row['refs'])
            if not keep_locked and not any(
                    r in self.parts and not self.parts[r].locked
                    for r in refs):
                continue
            rule, name, lim = row['rule'], row['name'], float(row['limit'])
            if rule == 'decap_distance':
                terms.append(_TetherTerm(rule, name, refs, lim, 'decap', {
                    'cap': row['cap'], 'ic': row['ic'],
                    'rail': tuple(row['rail']),
                    'rail_set': frozenset(row['rail']),
                    'graded': row['graded'], 'radius': row['radius']}))
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

    def tether_graded_value(self, t: _TetherTerm) -> float:
        """Term `t` at the LIVE poses, exactly, as the GRADE reads it (#1068):
        the measurement `IntentProbe` counts, never the gate's. No cache is
        read or written (the gate's caches are keyed by ITS term index), and
        a decap pair the live election puts beyond the search radius reads 0,
        because the grade calls it `decap_ungraded` (a warn, or per cap an
        error under a --decaps-from intent, #1142) however it was
        elected at build -- the gate deliberately keeps measuring that pair,
        which is stricter than the grade and therefore not a count of it."""
        return self._tether_measure(t, None, None, grade_view=True)

    def tether_gate_view_value(self, t: _TetherTerm) -> float:
        """Term `t` at the LIVE poses, exactly, as the GATE reads it: a pair
        graded at build stays measured past the search radius, because
        leaving the radius is not how a cap may stop being too far (#1043).
        What `IntentProbe`'s LICENCE and prune vector read (#1068); its COUNT
        reads `tether_graded_value`. No cache is read or written."""
        return self._tether_measure(t, None, None, grade_view=False)

    def _tether_measure(self, t: _TetherTerm, i: Optional[int],
                        override=None, grade_view: bool = False) -> float:
        """`_tether_value`'s body, for term `t`. `i` None: no cache."""
        from . import floorplan as _fp
        from . import groups as _groups
        # `override` is one or two refs on every path but a group move, so
        # membership is asked of IT, never by walking a term's (long) rail.
        moving = override or {}
        if t.kind == 'decap':
            # The grade's LIVE election: the nearest chip on the cap's rail at
            # these poses, not the IC elected at state build. A cap elected
            # beyond the radius of one IC can walk into the radius of another
            # (run 32: C26, 7.20mm from U15, walked to 3.07mm from U36), and
            # a frozen pair would read that move as clean.
            cap, rail = t.data['cap'], t.data['rail']
            if (cap not in moving and moving and not self._exact_tethers
                    and i is not None):
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
            if any(r in t.data['rail_set'] for r in moving) or i is None:
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
            if ((grade_view or not t.data['graded'])
                    and d > t.data['radius'] + legality.EPS):
                # Outside the radius the grade calls it `decap_ungraded`
                # (warn): not a finding this term counts. INSIDE it is graded,
                # so a pair elected beyond the radius may not walk in past
                # the limit -- that would be a new error the grade sees. A
                # pair graded at build stays measured beyond it: leaving the
                # radius is not how a cap may stop being too far.
                return 0.0
            return d
        if t.kind == 'pin':
            ic, caps = t.data['ic'], t.data['caps']
            if ic in moving:
                pin = self._posed_fp(ic, override).pads[t.data['pad_index']]
                got = _fp.nearest_rail_cap(
                    pin, [self._posed_fp(c, override) for c in caps])
                return got[0] if got is not None else 0.0
            live = tuple(sorted(c for c in moving if c in t.data['caps_set']))
            key = (i, live)
            static = self._tgap.get(key) if i is not None else None
            if static is None:
                pin = self._posed_fp(ic).pads[t.data['pad_index']]
                static = _fp.nearest_rail_cap(
                    pin, [self._posed_fp(c) for c in caps if c not in moving])
                if i is not None:
                    self._tgap[key] = static
            best = static[0] if static is not None else None
            if (best is not None and best <= t.threshold + legality.EPS
                    and not self._exact_tethers):
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
        if self._tether_override:
            # A group move: the OTHER members move too, so they are posed at
            # their shifted poses -- but only this call's refs' terms are
            # checked; the other members are checked by their own calls.
            override = dict(self._tether_override, **override)
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
        `override` maps each moving ref to its candidate pose; a group
        move's shifted members ride in `_tether_override`."""
        if not self._tether_active:
            return True
        return next(self._iter_tether_failures(override), None) is None

    def _tether_gate(self, ref, x, y, rot, site='candidate_valid') -> bool:
        """`candidate_valid`'s tether conjunct, tallied like `intent_ok`'s."""
        if not self._tether_active or ref not in self._tethers_of:
            return True
        # ONE full evaluation: an admitted pose needs every term anyway, and
        # a refused one needs every blocking term for the by-rule tally, so
        # stopping at the first failure would only buy a second pass.
        fails = self.tether_failures({ref: (x, y, rot)})
        if not fails:
            return True
        if self._why is not None:
            self._veto('tether', f"{fails[0][0]}:{fails[0][1]}")
        tally = self.intent_rejected_by_site
        tally[site] = tally.get(site, 0) + 1
        for rule, _n, _c, _u in fails:
            self.intent_rejected[rule] = self.intent_rejected.get(rule, 0) + 1
        return False

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
            own = self._fab_side(p)
            lb = sides.get(own) or next(iter(sides.values()))
            local = rotate_local_bounds(*lb, rot)
            self._fab_cache[key] = local
        return (x + local[0], y + local[1], x + local[2], y + local[3])

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
        if p is None or not sides:
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
        if self._neighbors is not None and ref in self._neighbors:
            others = [(o, self.parts[o]) for o in self._neighbors[ref]]
        else:
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
            try:
                from placement.body import board_bodies
                bodies = board_bodies(self.pcb_data, self.pcb_file)
            except Exception:                                # noqa: BLE001
                bodies = None
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
                if self._why is not None:
                    self._veto('body_contained', other_ref)
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
        # #893. Counted over UNORDERED pairs, like `halo` and `align` above and
        # for the same reason: `ref_inversions` sums a symmetric PAIR quantity
        # from one part's side, so summing it over every ref would count each
        # physical pair twice. `part_geometry_cost` does sum from one side --
        # that is the factor of 2 the evaluators need, and it cancels between
        # candidates -- but a REPORT must show each pair once.
        facing = (self.facing_weight
                  * sum(m['inversions']
                        for m in pair_inversions(self).values())
                  if self.facing_weight > 0.0 else 0.0)
        total = (self.length_weight * length
                 + self.crossing_penalty * w_crossings + halo + edge
                 + align + orient + facing + self.corridor_weight * cut)
        return {'total': total, 'length': length, 'crossings': crossings,
                'halo': halo, 'edge': edge, 'hpwl': self.hpwl(),
                'align': align, 'orient': orient, 'facing': facing,
                'corridor_cut': cut}

    def hpwl(self, nets=None):
        """Half-perimeter wirelength: sum over nets of the pad bbox's width plus
        height (mm). The classic placement-quality proxy, and one of the columns
        a placement scorecard wants (#411).

        `nets` restricts the sum to a net-id subset -- what
        `seeder.reseat_scope` needs to price the wirelength of the nets ITS
        scope touches (#698). `None`, the default, is every net and is the
        loop this method has always run, so `legality_metrics` and therefore
        `reconstruct.measure`'s `hpwl` term are bit-identical. One optional
        argument rather than a second HPWL in `seeder.py`: two implementations
        of one number is how they come to disagree.

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
        items = (self.net_refs.items() if nets is None else
                 [(n, self.net_refs[n]) for n in sorted(nets)
                  if n in self.net_refs])
        for net_id, refs in items:
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
        # A container's RECT is not a body (fa10 P1): the grader dropped a
        # pin frame's rect pairs and waives an outline's, so the optimizer's
        # number leaves them out too -- rp2350's was mostly U8's frame.
        _cont = getattr(self, 'container_refs', ()) or ()
        overlap = legality.placement_overlap_area(
            [g for g in parts if g.ref not in _cont])
        out = {'overlap_area': overlap,
               'oob_count': oob_count, 'oob_amount': oob_amount,
               'oob_area': oob_area, 'hpwl': self.hpwl()}
        if getattr(self, 'courtyards_ignored', False):
            # #1104: the project waives courtyard overlap (#1101's
            # predicate), so it is not a legality cost here -- the decap
            # rung, the reseat/evict gates and the portfolio all compare this
            # key, and each refused the moves the waiver exists for. The
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
        self._inc_tval.clear()
        self._tgap.clear()
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
        self._inc_intent.clear()
        self._inc_tval.clear()
        self._tgap.clear()
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
        if self._tether_active:
            # #1043: a tether between two members is invariant under the
            # shift, so each member's terms are read with the WHOLE block at
            # its shifted pose -- otherwise an IC translating with its caps
            # would be refused for "leaving" caps that travel with it.
            self._tether_override = {
                r: (self.parts[r].x + dx, self.parts[r].y + dy,
                    self.parts[r].rot) for r in refs}
        try:
            for ref in refs:
                part = self.parts[ref]
                if not self.candidate_valid(ref, part.x + dx, part.y + dy,
                                            part.rot, exclude=others):
                    return False
            return True
        finally:
            self._tether_override = None

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


class TetherGateView:
    """QuenchState's #1043 tether gate on parts that are not a QuenchState
    (#1067: `place_fanout_clearance`'s near-BGA caps).

    The methods ARE QuenchState's -- bound here as class attributes, not
    copied -- so a candidate is judged by the same measurement and the same
    per-claim rule the quench applies: past its limit AND worse than the live
    board refuses (`tether_failures` / `tether_ok`). `parts` maps each MOVABLE
    ref to an object with `x`, `y`, `rot` and `locked`; every other part is
    read at its file pose, which is right for an engine that moves only those
    parts. `note_move()` must follow every applied move: it clears the two
    caches QuenchState's `apply_move` clears.
    """

    _exact_tethers = False
    _build_tethers = QuenchState._build_tethers
    tether_terms_for = QuenchState.tether_terms_for
    _pose_of = QuenchState._pose_of
    _posed_fp = QuenchState._posed_fp
    _chip_bounds = QuenchState._chip_bounds
    _tether_value = QuenchState._tether_value
    tether_graded_value = QuenchState.tether_graded_value
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
        self._tether_override = None
        self._tether_bodies = None
        self.tethers = dict(tethers or {})
        if self.tethers:
            self._build_tethers()
        self._tether_active = bool(self._tether_terms)

    def note_move(self):
        """A part in `parts` moved: the incumbent values and the static
        partial minima are stale (QuenchState.apply_move's two clears)."""
        self._inc_tval.clear()
        self._tgap.clear()


def merge_groups(groups: Dict[str, List[str]], rigid: Dict[str, List[str]],
                 clusters: Dict[str, List[str]], movable_set: Set[str],
                 parts: Dict[str, '_Part']):
    """The quench's group phase over three sources, deduped (#1051/#1052/#1043).

    Claim order: the RIGID groups (declared arrays, then `rigid: true` blocks,
    in the gate bundle's order), then the tether clusters, then the caller's
    `--group-by` groups. A ref named by several keeps the FIRST group that
    claims it and is removed from the rest, each removal disclosed. Without
    this a ref in two groups is translated twice in one pass.

    A rigid group translates only when EVERY member present on the board can
    move: moving the movable part of a partly-locked row would shear it.
    Such a group is `anchored` -- its movable members are held still (they
    stay out of the single-part nudge) and it takes no translate.

    Returns (blocks, info): `blocks` is what the translate loop moves, with
    the plain filter every caller group always had (movable members, >= 2);
    `info` carries `groups` (every rigid group, present refs), `held`
    ({ref: rigid group}), `anchored` and `deduped`.
    """
    claimed: Dict[str, str] = {}
    dropped: Dict[str, List[str]] = {}
    kept: Dict[str, Tuple[str, List[str]]] = {}
    sources = ([(n, r, 'rigid') for n, r in rigid.items()]
               + [(n, r, 'tether') for n, r in clusters.items()]
               + [(n, r, 'caller') for n, r in groups.items()])
    for name, refs, kind in sources:
        mine: List[str] = []
        for ref in refs:
            owner = claimed.get(ref)
            if owner is None:
                claimed[ref] = name
                mine.append(ref)
            elif owner != name:
                dropped.setdefault(ref, []).append(name)
        if name in kept:            # one name from two sources: one group
            kept[name][1].extend(mine)
        else:
            kept[name] = (kind, mine)
    blocks: Dict[str, List[str]] = {}
    info: Dict[str, object] = {'groups': {}, 'held': {}, 'anchored': {},
                               'clusters_dropped': [],
                               'deduped': [
                                   {'ref': r, 'kept': claimed[r],
                                    'dropped_from': sorted(n)}
                                   for r, n in sorted(dropped.items())]}
    for name, (kind, refs) in kept.items():
        present = [r for r in refs if r in parts]
        mov = [r for r in present if r in movable_set]
        if kind == 'rigid':
            if len(present) < 2:
                continue            # one part is not a formation to hold
            info['groups'][name] = present
            fixed = [r for r in present if r not in movable_set]
            if fixed:
                info['anchored'][name] = fixed
            elif len(mov) >= 2:
                blocks[name] = mov
            for r in mov:
                info['held'][r] = name
        elif kind == 'tether' and name.split(':', 1)[1] not in refs:
            # Its IC was claimed by an earlier (rigid) group: what is left is
            # caps with no IC, and translating them together moves them OFF
            # their IC rather than with it. Dropped, and disclosed.
            info['clusters_dropped'].append(name)
        elif len(mov) >= 2:
            blocks[name] = mov
    return blocks, info


def _rigid_swap_ok(held: Dict[str, str], array_order: Dict[str, Set[str]],
                   ra: str, rb: str) -> bool:
    """May two parts exchange poses when at least one is a held rigid member?

    Only inside ONE group. Inside a declared array, only when neither ref has
    an expected position along the row (`order_refs`): an array whose order is
    `pin` or `declared` is graded on that order by `arrays.formation`, and a
    swap of two positioned members breaks it; `order: "unknown"` positions
    nobody, so any two members may trade. Inside a `rigid: true` block, always:
    the block declares WHICH parts travel together, not where each sits in
    it, and a same-footprint swap preserves the occupied space exactly.
    """
    ga, gb = held.get(ra), held.get(rb)
    if ga is None or ga != gb:
        return False
    order = array_order.get(ga)
    if order is None:
        return True                 # a rigid block
    return ra not in order and rb not in order


def _clause_failing(state, ref, override=None, exclude=None) -> Optional[str]:
    """The first clause `ref` fails at its pose in `override` (else its live
    pose), or None. Absolute, not monotone: a release is about a pose that is
    WRONG, not one that is merely no better. Intra-group pairs are `exclude`d:
    a formation's own spacing is invariant under every block move and is the
    seeder's decision, not a reason to break the formation up."""
    part = state.parts[ref]
    x, y, rot = (override or {}).get(ref, (part.x, part.y, part.rot))
    rects = part.grade_rects(x, y, rot)
    spec = state.intent_spec_for(ref)
    if spec:
        for v, t in zip(state.intent_terms(ref, rects), spec):
            if v > t.threshold:
                return f"intent:{t.rule}"
    if state._tether_active:
        for i in state._tethers_of.get(ref, ()):
            t = state._tether_terms[i]
            if state._tether_value(i, override) > t.threshold + legality.EPS:
                return f"intent:{t.rule}"
    board, overlap = state.violation_parts(ref, x, y, rot, exclude=exclude)
    if board > EPS_IMPROVE or overlap > EPS_IMPROVE:
        return 'legality'
    return None


def _release_clause(state, ref, members, block_refs, max_disp, step, lattice
                    ) -> Optional[str]:
    """The clause that releases `ref` from its rigid group, or None.

    Released only when its INCUMBENT pose fails a clause (`_clause_failing`)
    AND no admissible block offset clears it: an offset the group phase could
    take (`group_move_valid`) at which the member's clause no longer fails.
    An anchored group (a member cannot move) has no offsets, so a failing
    member of one is released directly. `members` is the FORMATION -- the
    group minus its released members, `_formation` -- whose pairs are
    excluded from the legality clause; the rejoin test excludes exactly the
    same set, or a released sibling sitting on a member would count against
    it here and be ignored there, and the member would flip every pass.
    `block_refs` are the members the translate moves, or None for an
    anchored group.
    """
    members = set(members)
    clause = _clause_failing(state, ref, exclude=members)
    if clause is None:
        return None
    if block_refs:
        # A PROBE, not a search step: the refusals `group_move_valid` tallies
        # here are not refusals of a move the search considered, so the
        # `intent_gate` tallies are restored afterwards.
        saved = (dict(state.intent_rejected),
                 dict(state.intent_rejected_by_site))
        try:
            for dx, dy in _group_offsets(state, block_refs, max_disp, step,
                                         lattice):
                if not state.group_move_valid(block_refs, dx, dy):
                    continue
                shifted = {r: (state.parts[r].x + dx,
                               state.parts[r].y + dy,
                               state.parts[r].rot) for r in block_refs}
                if _clause_failing(state, ref, shifted,
                                   exclude=members) is None:
                    return None
        finally:
            state.intent_rejected, state.intent_rejected_by_site = saved
    return clause


def _public(rec: Dict[str, object]) -> Dict[str, object]:
    """A release record without its private rejoin bookkeeping."""
    return {k: v for k, v in rec.items() if not k.startswith('_')}


def _slot_of(state, ref, anchor):
    """`ref`'s offset from `anchor` (pose), the row slot a rejoin checks."""
    p, a = state.parts[ref], state.parts[anchor]
    return (round(p.x - a.x, 6), round(p.y - a.y, 6), round(p.rot % 360, 6))


def _formation(rigid_info, name, released) -> Set[str]:
    """Group `name` minus its currently released members: the parts whose
    mutual spacing is the formation's own, excluded from a member's legality
    clause by BOTH the release and the rejoin test."""
    out = {r['ref'] for r in released if r['group'] == name}
    return set(rigid_info['groups'][name]) - out


def _rebuild_block(blocks, held, rigid_info, name) -> None:
    """`blocks[name]`: the group's members still `held`, in group order, when
    at least two are -- one part is no formation to translate, and stays
    still, holding its slot for a rejoin. An anchored group never has a
    block. Rebuilt from `held` on every release and rejoin, so a member
    rejoining a row that a release shrank to one part re-forms the block
    with the sibling that stayed, not alone."""
    if name in rigid_info['anchored']:
        return
    refs = [r for r in rigid_info['groups'][name] if held.get(r) == name]
    if len(refs) >= 2:
        blocks[name] = refs
    else:
        blocks.pop(name, None)


def _update_releases(state, held, blocks, rigid_info, released, rejoined,
                     pass_num, max_disp, step, lattice) -> bool:
    """End-of-pass release and rejoin (see the call site). Mutates `held`,
    `blocks`, `released` and `rejoined`; True when anything changed, so the
    pass loop runs once more for the change to act."""
    changed = False
    for ref in sorted(held):
        name = held[ref]
        clause = _release_clause(state, ref,
                                 _formation(rigid_info, name, released),
                                 blocks.get(name), max_disp, step, lattice)
        if clause is None:
            continue
        rest = [r for r in rigid_info['groups'][name]
                if r != ref and r in state.parts]
        anchor = rest[0] if rest else None
        released.append({'ref': ref, 'group': name, 'clause': clause,
                         'pass': pass_num, '_anchor': anchor,
                         '_slot': (_slot_of(state, ref, anchor)
                                   if anchor else None)})
        del held[ref]
        _rebuild_block(blocks, held, rigid_info, name)
        changed = True
        print(f"  NOTE: {ref} released from rigid group {name} ({clause}) "
              f"after pass {pass_num}: its pose still fails it after every "
              f"other part had the pass to clear it, and no block move "
              f"clears it, so it may move alone")
    for rec in list(released):
        # Hysteresis: never in the pass of the release nor the next one. A
        # member released after pass N moves alone in pass N+1; judging its
        # rejoin before it has had that pass would decide on the very state
        # that released it.
        if pass_num < rec['pass'] + 2 or rec['_anchor'] is None:
            continue
        ref, name = rec['ref'], rec['group']
        if _slot_of(state, ref, rec['_anchor']) != rec['_slot']:
            continue                    # it moved alone: stays released
        # The SAME exclusion as the release test: the formation it would
        # rejoin, plus itself. Its released siblings are ordinary parts to
        # both decisions.
        if _clause_failing(state, ref, exclude=_formation(
                rigid_info, name, released) | {ref}) is not None:
            continue
        released.remove(rec)
        rejoined.append(dict(rec, rejoined_after_pass=pass_num))
        held[ref] = name
        _rebuild_block(blocks, held, rigid_info, name)
        changed = True
        print(f"  NOTE: {ref} rejoins rigid group {name} after pass "
              f"{pass_num}: it is clean again and still in its slot")
    return changed


#: The `metrics_out` keys a caller's JSON_SUMMARY carries verbatim (#1043,
#: #1051, #1052). Each is present only when its channel was declared, so an
#: undeclared run's summary is unchanged.
DISCLOSURE_KEYS = ('rigid', 'rigid_released', 'groups_deduped', 'tethers')


def disclosure(metrics_out: Dict) -> Dict[str, object]:
    """The `DISCLOSURE_KEYS` present in `metrics_out`, for a JSON_SUMMARY."""
    return {k: metrics_out[k] for k in DISCLOSURE_KEYS if k in metrics_out}


def _tether_over_limit(state) -> Dict[str, int]:
    """{rule: terms past their limit} on the live poses (#1043)."""
    out: Dict[str, int] = {}
    for i, t in enumerate(state._tether_terms):
        if state._incumbent_tether(i) > t.threshold + legality.EPS:
            out[t.rule] = out.get(t.rule, 0) + 1
    return out


def _group_offsets(state, refs, max_disp: float, step: float, lattice: float):
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
            # Snap the OFFSET, so the block stays rigid: snapping each member
            # independently would shear it by up to a lattice step. Snapping to
            # the BOARD's lattice rather than the raster is #708: an offset
            # that is a whole number of the designer's grid units carries every
            # member from an on-lattice pose to another one.
            sdx = snap_to_grid(ix * step, lattice)
            sdy = snap_to_grid(iy * step, lattice)
            if math.hypot(sdx, sdy) > max_disp + 1e-9:
                continue
            if (sdx, sdy) in seen or (sdx == 0.0 and sdy == 0.0):
                continue
            # Probe the pose that will actually be APPLIED. This used to snap
            # the ABSOLUTE p.x + dx while the emitted offset was snap(dx), so
            # the per-member seed cap -- the thing this docstring says keeps
            # build_neighbor_lists' pruning exact and edges_near's cache valid
            # -- was tested against a pose the block never takes. The gap was
            # under 0.07mm at the raster and is under a lattice step now, but
            # it was always the wrong quantity.
            ok = True
            for ref in refs:
                p = state.parts[ref]
                if math.hypot(p.x + sdx - p.seed_x,
                              p.y + sdy - p.seed_y) > max_disp + 1e-9:
                    ok = False
                    break
            if not ok:
                continue
            seen.add((sdx, sdy))
            yield sdx, sdy


def _candidate_positions(part: _Part, max_disp: float, step: float,
                         lattice: float):
    """Grid of candidate centers within max_disp of the seed position.

    The snap is on the OFFSET, not on the absolute position, and that
    distinction is the whole of #708. `seed_x + ix*step` already carries
    whatever phase the designer laid the board out on; snapping the SUM to a
    lattice through board ORIGIN discards it, because seed_x is not generally a
    multiple of anything. Snapping the offset instead inherits the seed's
    phase, so a part on a 0.3175mm (12.5 mil) lattice is offered only poses on
    that same lattice.

    Two measurements behind this, both in `tests/measure_708_lattice.py`:

      * the old absolute snap removed ZERO candidates at every step the tool
        ships -- it was a pure translation of the whole set by the seed's
        residue -- so nothing is lost by dropping it;
      * snapping the offset to the RASTER would not be enough. At step=1.0 the
        raster offsets are {0, +/-1.0, +/-2.0, ...} and 1.0 is not a multiple
        of 0.3175, so only the zero offset lands back on the board's lattice.
        Snapping to the lattice gives {0, +/-0.9525, +/-1.905, ...} and every
        candidate stays on the seed's coset of it.

    Reach is comparable but NOT uniformly better, and the honest bound is worth
    stating: sweeping max_disp 0.5..19.5 at step=1.0 on a 0.3175 lattice, 12
    values gain candidates, 17 tie and 10 LOSE -- worst 81 -> 69 at
    max_disp=5.0, because an offset the raster admitted at exactly the cap can
    snap UP past it. That is the price of the exact cap below, paid knowingly.
    At the shipped default (10.0 / 1.0) it is 317 -> 325.

    `lattice` is the board's own pitch when one can be read off it and the
    `grid_step` raster otherwise, so a board with no inferable lattice keeps
    exactly the offsets it has always had (see `placement/board_grid.py`).

    The radius test runs AFTER the snap, so `max_disp` is an exact cap rather
    than one overshot by up to lattice*sqrt(2)/2.
    """
    seen = set()
    out = []
    n = int(max_disp / step)
    for ix in range(-n, n + 1):
        for iy in range(-n, n + 1):
            dx = snap_to_grid(ix * step, lattice)
            dy = snap_to_grid(iy * step, lattice)
            if math.hypot(dx, dy) > max_disp + 1e-9:
                continue
            cx = part.seed_x + dx
            cy = part.seed_y + dy
            key = (round(cx, 4), round(cy, 4))
            if key not in seen:
                seen.add(key)
                out.append((cx, cy))
    return out


def _candidate_rotations(part: _Part, allow_rotations: bool,
                         declared=None) -> List[float]:
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
    if declared is not None:
        # #893. A DECLARED rotation outranks the lattice. `[angle]` for a
        # decision -- the move loop then has no rotation to choose and the
        # part keeps the angle the seeder was told to give it -- and the
        # author's SET, order preserved, for `rotation_candidates`. This is
        # what makes the declaration survive `place_seed -> place_optimize`;
        # without it the intent was honoured once and undone by the next step,
        # which is weaker than the locking it replaced.
        rot, cands = declared
        if rot is not None:
            return [rot % 360]
        return [c % 360 for c in cands]
    bases = [part.rot % 90]
    if part.orig_rot % 90 != bases[0]:
        bases.append(part.orig_rot % 90)
    return [(b + r) % 360 for b in bases for r in ROTATIONS]


def _same_angle(a, b) -> bool:
    return abs((a - b + 180.0) % 360.0 - 180.0) < 1e-6


def _declared_admits(declared, rot, current=None) -> bool:
    """Whether a declared rotation claim (#893; `(rotation, candidates)`, or
    None for no claim) admits the angle `rot` -- the swap phase's half of the
    pin `_candidate_rotations` puts on the nudge (#1117). A part handed its
    `current` angle is never refused: a swap that changes no angle cannot
    make a declaration worse. An earlier version of this gate refused it --
    even under `--no-rotate`, where no move can turn anything -- and lost a
    16 mm wirelength win (#1117's second verifier)."""
    if declared is None:
        return True
    if current is not None and _same_angle(rot, current):
        return True
    return any(_same_angle(rot, a)
               for a in _candidate_rotations(None, True, declared))


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
           intent_gate: Optional[Dict[str, object]] = None,
           cancel_check=None,
           progress_callback=None,
           # #916: the SEARCH's body currency. Appended, and False by default
           # -- see QuenchState.__init__. Flipping it is an engine change
           # gated on tests/test_placement_ab.py, not a tidy-up.
           body_model: bool = False,
           # #893: the pin-order facing term. Appended, 0.0 by default.
           facing_weight: float = 0.0) -> List[Dict]:
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
            if any(fnmatch.fnmatchcase(net.name, pat) for pat in ignore_nets):
                ignore_net_ids.add(net_id)
        print(f"Ignoring {len(ignore_net_ids)} nets for airwire scoring")

    extra_locked: Set[str] = set()
    if lock_refs:
        import fnmatch
        for ref in pcb_data.footprints:
            if any(fnmatch.fnmatchcase(ref, pat) for pat in lock_refs):
                extra_locked.add(ref)
        print(f"Locked via --lock: {', '.join(sorted(extra_locked))}")

    # #702: `must_lock` and the intent's EDGE CLAIMS are enforced by freezing
    # the ref, not by a per-pose term -- neither is a property of a pose. The
    # merge is a UNION of three sources (the file's own locks, --lock, and
    # these), and none of the three has an un-lock operator, so a conflict is
    # impossible by construction and nothing a caller asked for is overridden.
    #
    # Printed under its OWN name rather than folded into the line above, which
    # would otherwise become a lie about where a frozen part came from.
    _intent_locked = set((intent_gate or {}).get('lock_refs') or ())
    _intent_locked &= set(pcb_data.footprints)
    if _intent_locked:
        extra_locked |= _intent_locked
        print(f"Locked via intent (must_lock / edge claims): "
              f"{', '.join(sorted(_intent_locked))}")

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
                        corridor_specs=corridor_specs,
                        keepouts=(intent_gate or {}).get('keepouts'),
                        intent_zones=(intent_gate or {}).get('zones'),
                        declared_rotations=(intent_gate or {}).get('rotations'),
                        body_model=body_model,
                        facing_weight=facing_weight,
                        tethers=(intent_gate or {}).get('tethers'))
    # #708: the lattice candidate OFFSETS are multiples of. The board's own
    # pitch when one can be read off it, the `grid_step` raster otherwise.
    # There is deliberately no flag: the fallback IS the off state and the
    # board picks it. Note the fallback restores the OFFSET GRANULARITY, not
    # the old poses -- the seed-relative half applies either way, which is the
    # bug fix.
    lattice, lattice_evidence = resolve_snap_lattice(pcb_data, grid_step)
    # A lattice COARSER than the search step would quantize the search rather
    # than merely phase it: `snap(0.1, 0.3175)` is 0.0, so at `--step 0.1` on an
    # imperial board every +/-1 offset collapses onto the zero offset and
    # `_group_offsets` discards it as "no move at all". The finest rung of the
    # search would vanish silently. `--step` is a plain float on three CLIs with
    # nothing relating it to the board, so the guard lives here.
    if lattice > step + 1e-9:
        lattice_evidence = dict(lattice_evidence, source='grid_step',
                                resolved=grid_step,
                                reason='inferred %g mm is coarser than --step '
                                       '%g mm, which would quantize the search'
                                       % (lattice, step))
        lattice = grid_step
    print(describe_lattice(lattice_evidence))

    # Unchanged, and now conservative rather than exact: the offsets are
    # snapped BEFORE the radius test, so a candidate is within max_displacement
    # of its seed rather than up to a snap diagonal past it. The old
    # `+ grid_step` slack covered that overshoot and now simply exceeds it.
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
    movable_set = set(movable)
    # #1051/#1052: the intent's RIGID groups (declared arrays, and blocks that
    # declare `rigid: true`) join the group phase whatever --group-by says,
    # and #1043's IC+caps clusters join it when a decap tether is armed, so an
    # IC can still travel WITH its caps now that the gate holds each cap near
    # it. All three are empty on an intent that declares none of them, and
    # `merge_groups` then returns the caller's groups exactly as the plain
    # filter below always did.
    rigid_in = dict((intent_gate or {}).get('rigid_blocks') or {})
    # A declared ARRAY is held rigid only while it is a formed row at the
    # poses this quench starts from (`arrays.formation`, the grader's own
    # predicate): holding a row the seeder could not seat (its members were
    # seated one by one, `array_unseated`), or one a later edit broke, would
    # weld unrelated poses together. Disclosed in `rigid.unformed`, with the
    # checks it fails. Blocks declaring `rigid: true` are held as declared.
    rigid_unformed: Dict[str, List[str]] = {}
    if any(n.startswith('array:') for n in rigid_in):
        from placement import arrays as _arr
        _specs = {f"array:{a.get('name')}": a
                  for a in (intent_gate or {}).get('arrays') or ()}
        for name in sorted(rigid_in):
            if not name.startswith('array:'):
                continue
            mem = [r for r in rigid_in[name] if r in state.parts]
            spec = _specs.get(name)
            if spec is None or len(mem) < 2:
                continue
            v = _arr.formation_at_state(state, pcb_data, spec, mem)
            if not v['formed']:
                rigid_unformed[name] = list(v['failed'])
                del rigid_in[name]
                print(f"  NOTE: {name} is not a formed row here "
                      f"({', '.join(v['failed'])} failed), so it is not "
                      f"held rigid -- its members move as single parts")
    clusters: Dict[str, List[str]] = {}
    if state._tether_active and any(
            t.rule in ('decap_distance', 'decap_pin_distance')
            for t in state._tether_terms):
        from placement.groups import derive_groups as _derive
        for name, refs in _derive(pcb_data, ('decap',),
                                  movable=movable_set).items():
            ic = name.split(':', 1)[1]
            if ic in refs:              # a cluster without its IC moves caps
                clusters[f"tether:{ic}"] = list(refs)   # off it, not with it
    rigid_info = None
    if rigid_in or clusters:
        blocks, rigid_info = merge_groups(groups or {}, rigid_in, clusters,
                                          movable_set, state.parts)
        for d in rigid_info['deduped']:
            print(f"  NOTE: {d['ref']} is in {', '.join(d['dropped_from'])} "
                  f"and in {d['kept']}; it moves with {d['kept']} only")
        for name, locked_refs in sorted(rigid_info['anchored'].items()):
            print(f"  NOTE: rigid group {name} cannot translate -- "
                  f"{', '.join(locked_refs)} cannot move; its movable "
                  f"members are held where they are")
        if rigid_info['groups']:
            print("Rigid groups (#1051/#1052): "
                  + ', '.join(f"{n} ({len(r)})" for n, r in
                              sorted(rigid_info['groups'].items())))
        if clusters:
            print(f"Tether clusters (#1043): "
                  f"{len(clusters) - len(rigid_info['clusters_dropped'])} "
                  f"IC+caps group(s) join the rigid translate"
                  + (f"; dropped {', '.join(rigid_info['clusters_dropped'])}"
                     f" -- a rigid group claimed the IC"
                     if rigid_info['clusters_dropped'] else ''))
    elif groups:
        blocks = {name: [r for r in refs if r in movable_set]
                  for name, refs in groups.items()}
        blocks = {n: r for n, r in blocks.items() if len(r) >= 2}
    if blocks and verbose:
        from placement.groups import describe
        print(describe(blocks))
    #: ref -> the rigid group holding it out of the single-part nudge.
    held: Dict[str, str] = dict((rigid_info or {}).get('held') or {})
    released: List[Dict[str, object]] = []
    rejoined: List[Dict[str, object]] = []
    moved_as_block: Dict[str, int] = {}
    array_order = {f"array:{a.get('name')}": set(a.get('order_refs') or ())
                   for a in (intent_gate or {}).get('arrays') or ()}
    swaps_skipped_rigid_total = 0
    if state._tether_active:
        _rules = sorted({t.rule for t in state._tether_terms})
        print(f"Tethers (#1043): {len(state._tether_terms)} term(s) held "
              f"per move ({', '.join(_rules)})")

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
        swaps_skipped_intent = 0
        swaps_skipped_rigid = 0

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
                                           lattice):
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
                moved_as_block[name] = moved_as_block.get(name, 0) + 1
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
            if ref in held:
                continue            # #1052: a rigid member moves with its group
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
            rotations = _candidate_rotations(
                part, allow_rotations, state.declared_rotations.get(ref))
            for rot in rotations:
                # A swap can hand a part an angle from ANOTHER seed's lattice,
                # in a group holding two different non-orthogonal seeds. Add
                # the bounds entry so rect() does not silently fall back to
                # rot-0 geometry. Every such angle is already inside the group
                # closure build_neighbor_lists unioned, so this cannot
                # invalidate the pruning. No-op for orthogonal parts. The far
                # side too (#1206): it used to keep only the bounds, so such
                # an angle read its drilled pads at rot 0.
                part.ensure_rotation(rot)

            best = (current_cost, part.x, part.y, part.rot)
            for cx, cy in _candidate_positions(part, max_displacement, step,
                                               lattice):
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
                        if held and (ra in held or rb in held) and not \
                                _rigid_swap_ok(held, array_order, ra, rb):
                            swaps_skipped_rigid += 1
                            continue
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
                            p_dst.ensure_rotation(inherited)
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
                        # Declared intent (#702). Counted and PRINTED under its
                        # own name: this file already carries the scar for a
                        # silent swap rejection -- "two instances of one
                        # footprint that never swap look exactly like a pair
                        # with nothing to gain".
                        if ((state._intent_active or state._tether_active
                             or state.declared_rotations)
                                and not state.swap_intent_ok(ra, rb)):
                            state._note_swap_refusal(ra, rb)
                            swaps_skipped_intent += 1
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

        # --- rigid releases and rejoins (#1051/#1052) ---
        # At the END of the pass, not at first sight: every movable part
        # outside the group has had this pass's nudge and swap phases to
        # clear the violation from its side, so a member is not broken out of
        # its row for a clash its neighbour could have resolved (the
        # verifier's case: an UNLOCKED part dropped on a row member). A
        # member is released only when its current pose still fails a clause
        # and no admissible block offset clears it; it moves alone from the
        # NEXT pass. A released member that is clean again and still sits in
        # its slot of the row (it never moved alone, or came back) REJOINS --
        # release is not a verdict for the rest of the run. One that moved
        # stays released: pulling it back into the row would be a move no
        # objective chose.
        changed = _update_releases(state, held, blocks, rigid_info, released,
                                   rejoined, pass_num, max_displacement,
                                   step, lattice) if (held or released)             else False

        stats = state.total_cost()
        group_note = f" blocks={group_moves}" if group_moves else ""
        swap_note = (f" swap-capped={swaps_skipped}"
                     if verbose and swaps_skipped else "")
        # Shape/side mismatches were silent before: two instances of one
        # footprint that never swap look exactly like a pair with nothing to gain.
        if verbose and swaps_skipped_shape:
            swap_note += f" swap-mismatched={swaps_skipped_shape}"
        # NOT gated on `verbose`: a swap the declared intent killed is
        # a constraint doing its job, and the run that most needs to
        # know is the one nobody ran with -v.
        if swaps_skipped_intent:
            swap_note += f" swap-intent={swaps_skipped_intent}"
        if swaps_skipped_rigid:
            swap_note += f" swap-rigid={swaps_skipped_rigid}"
            swaps_skipped_rigid_total += swaps_skipped_rigid
        print(f"Pass {pass_num}: {moves} moves, gain {improved:.1f} -> "
              f"length={stats['length']:.1f}mm crossings={stats['crossings']} "
              f"halo={stats['halo']:.1f} edge={stats['edge']:.1f} "
              f"total={stats['total']:.1f}{group_note}{swap_note}")
        if stopped:
            break
        if moves == 0 and not changed:
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
        # #708: which lattice the candidate offsets were multiples of, and how
        # that was decided. `source` distinguishes "the board declared one"
        # from "it did not and we used the raster" -- without it, a run on a
        # board with no lattice and a run where the inference was never wired
        # report the same thing.
        metrics_out['board_grid'] = dict(lattice_evidence,
                                         resolved=lattice)
        # #702: what the declared-intent gate actually DID. Always present when
        # a gate was built, and `rejected: 0` is a real answer -- without this
        # key, "the gate refused nothing" and "the gate was never wired" are
        # the same observation, which is how a constraint ships inert.
        # Gated on "a gate was HANDED IN", not on `_intent_active`. An intent
        # whose zone refs resolve to nothing on this board, or whose only
        # keep-out allows every ref, builds an empty spec -- and reporting no
        # key for that puts back exactly the ambiguity this key removes: "the
        # gate refused nothing" and "there was no gate" become the same
        # observation again, in the one case where the difference matters most.
        if intent_gate is not None:
            metrics_out['intent_gate'] = {
                'rejected': sum(state.intent_rejected_by_site.values()),
                'by_rule': dict(sorted(state.intent_rejected.items())),
                'by_site': dict(sorted(state.intent_rejected_by_site.items())),
                # Both derived through `intent_spec_for`, NOT off
                # `_intent_spec`. That dict holds the ZONE terms only -- the
                # keep-out slice is derived live from `keepouts_for` so the
                # #701 census lift keeps working -- so reading it directly
                # made a keep-out-ONLY intent (the shape #701 exists for)
                # report `refs_bound: 0, rules_enforced: []` while refusing
                # hundreds of poses. `place_optimize` then printed the
                # self-contradictory line "enforced  over 0 bound part(s);
                # refused 360 candidate pose(s)", and shipped the same
                # nonsense in JSON_SUMMARY.
                # #1043: the tether terms' refs and rules count too -- a
                # decaps-only intent must not report `rules_enforced: []`
                # while refusing poses on `decap_distance`.
                # #1117: and the declared rotations (of parts this state
                # holds), which the swap refuses on as rule `rotation` -- a
                # GATE rule, not a grade rule: floorplan has no
                # rule_rotation. Without them an earlier version of this
                # gate reported "enforced over 0 bound part(s)" on a
                # rotation-only intent while refusing swaps.
                'refs_bound': len(set(state._intent_spec)
                                  | set(state.keepouts_for)
                                  | set(state._tethers_of)
                                  | (set(state.declared_rotations)
                                     & set(state.parts))),
                'rules_enforced': sorted(
                    {t.rule for ref in (set(state._intent_spec)
                                        | set(state.keepouts_for))
                     for t in state.intent_spec_for(ref)}
                    | {t.rule for t in state._tether_terms}
                    | ({'rotation'} if state.declared_rotations else set())),
            }
        if state._tether_active:
            by_rule: Dict[str, int] = {}
            for t in state._tether_terms:
                by_rule[t.rule] = by_rule.get(t.rule, 0) + 1
            metrics_out['tethers'] = {
                'armed': sorted((intent_gate or {}).get('tethers') or ()),
                'terms': by_rule,
                'refs_bound': len(state._tethers_of),
                'clusters': {n: list(r) for n, r in sorted(clusters.items())
                             if n not in (rigid_info or {}).get(
                                 'clusters_dropped', ())},
                'clusters_dropped': list((rigid_info or {}).get(
                    'clusters_dropped', ())),
                # Past its limit on the WRITTEN poses, per rule: what the
                # gate held (never worse than the input) made visible.
                'over_limit_after': _tether_over_limit(state),
            }
        if rigid_info is None and rigid_unformed:
            metrics_out['rigid'] = {'groups': {}, 'moved_as_block': {},
                                    'anchored': {}, 'released': [],
                                    'rejoined': [], 'swaps_refused': 0,
                                    'unformed': rigid_unformed}
        if rigid_info is not None:
            metrics_out['rigid'] = {
                'unformed': rigid_unformed,
                'groups': {n: list(r) for n, r in
                           sorted(rigid_info['groups'].items())},
                'moved_as_block': {n: moved_as_block.get(n, 0)
                                   for n in sorted(rigid_info['groups'])},
                'anchored': {n: list(r) for n, r in
                             sorted(rigid_info['anchored'].items())},
                'released': [_public(r) for r in released],
                'rejoined': [_public(r) for r in rejoined],
                'swaps_refused': swaps_skipped_rigid_total,
            }
            metrics_out['rigid_released'] = [_public(r) for r in released]
            metrics_out['groups_deduped'] = [dict(d) for d in
                                             rigid_info['deduped']]

    return [{'reference': ref,
             'new_x': p.x, 'new_y': p.y, 'new_rotation': p.rot}
            for ref, p in state.parts.items()
            if not p.locked and (p.x != p.seed_x or p.y != p.seed_y
                                 or p.rot != p.orig_rot % 360)]

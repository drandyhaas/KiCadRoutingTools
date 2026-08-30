"""Bounded block relocation as a constraint solve (#554, part 2 of #459).

#459: *"Relocation is a constraint problem, not an optimization problem. Move only
diagnosed blocks; keep the relative order of everything else as a hard constraint;
minimize displacement."* And: *"The constraint graph doubles as the explanation of
why a block sits where it does, which is what an AI director needs to justify a
move."*

The quench's rigid block translate (#538) caps every member at `max_displacement`
from its own seed and freezes everybody else. Measured with the neighbours frozen,
the furthest a block travels TOWARD its connectivity target is a **median of
0.00 mm against a median want of 10.36 mm**, over the 24 measurable blocks on the
9 boards that have one (`tests/stress/relocation_reach.py`; a tenth board,
glasgow_revC, contributes only refusals -- its blocks contain KiCad-locked
parts). A shipped board has no vacancy to translate into; the only way a block
moves is if its neighbours yield.

THE FORMULATION
---------------
One variable per **rigid unit** per axis, holding a SHIFT from the incumbent pose.

* **Rigidity is variable sharing, not constraints.** Every member of a block reads
  the same variable, so the block translates rigidly at zero constraint cost -- and
  so does every untouched decap cluster.
* **`s = 0` is the incumbent board, so identity is always feasible.** Every gap
  requirement is `min(clearance, the separation the pair already has)`, which is
  character-for-character `LegalityContext.pads_ok`'s rule ("no worse than the SEED
  baseline"). That is not a softening invented here: without it a board that ships
  81 of 82 parts inside the courtyard clearance (watchy) would be infeasible at its
  own incumbent, and this pass would refuse every board for a reason that has
  nothing to do with room.
* **Non-overlap becomes a frozen order.** Each interacting pair contributes ONE
  difference constraint, in the axis where it is currently more separated:
  `s_hi - s_lo >= gap - current_separation`. Freezing the disjunction costs a
  board whose only room needs a part to slide around a corner; that answer is
  `no_room_at_any_dose`, which is a correct answer to the question #554 asked.

The whole system is difference constraints, so its solutions form a lattice: the
componentwise maximum is itself feasible, which is why `reach` is exactly the
block's own upper envelope over THIS SYSTEM.

THE GRAPH IS NOT A CONSERVATIVE MODEL OF LEGALITY. THE EXACT CHECK IS.
----------------------------------------------------------------------
An earlier revision of this docstring claimed the frozen disjunction made the
system "a RESTRICTION of the true feasible set, never a relaxation -- it can
refuse a legal answer, never admit an illegal one". **That is false, and it was
falsified by measurement rather than by reading.** Two mechanisms, both
demonstrated on shipped corpus boards at the lattice minimum:

1. **A gap is Euclidean; a constraint is per-axis.** `rect_gap` is
   `hypot(sep_x, sep_y)` when both separations are positive, and the graph
   constrains only the larger axis, leaving the other free to shrink. A pair at
   separations (0.24, 0.24) has a true gap of 0.339 -- legal at 0.25 clearance --
   and an assignment violating NONE of the graph's edges can drive it to 0.240.
   Measured live: watchy `AE1/C7` 0.2388 -> 0.2250, glasgow_revC `R23/U7`
   0.1769 -> 0.1300, ulx3s `R56/U10` 0.2420 -> 0.2300.
2. **Admission geometry is not pricing geometry.** A pair is admitted through
   `gap_to` -> `pair_min_gap`, which measures over the sides the two parts SHARE
   and includes a THT part's far-side lead field; the edge is then priced from
   the two COURTYARD rects. For a front part against a back-side THT part the
   courtyards overlap freely, so the requirement is vacuous: tigard `JP1/SW1` has
   a real `pair_min_gap` of +1.05 mm and a graph floor of -1.25 mm.

Censused over 54,841 inter-unit pairs on ten boards: **73 exposed**, of which 51
are currently clear and could be driven sub-clearance; worst exposure 2.97 mm.

`identity_violations` cannot see any of this: it checks that `s = 0` satisfies
the GRAPH, not that the graph implies the GEOMETRY. It is a change detector for a
broken gap clamp -- which is what it was added for, and what it catches -- and
nothing more.

What makes the shipped pass safe is therefore **not** the graph but
`exact_refusal`, which re-measures every moved part with the real gate and
refuses the dose outright if any pair worsened. Nothing is ever applied on the
LP's word. The consequence for callers: a `Relocation` is trustworthy, and a bare
`reach()` is an upper bound on the SYSTEM, not a promise about the board.

`binding_path` is the shortest-path predecessor chain that produced that envelope --
a NAMED chain of refs and gaps ending at the wall or the locked part that stopped
the block. That is the explanation #459 asked for, and it falls out of the solve
rather than being reconstructed afterwards.

THE CONTRACT THAT KEEPS THIS SAFE
---------------------------------
**`build_neighbor_lists` is never called.** The travel budget then stays infinite,
`BoardOutlineGate.edges_near`'s per-ref cache is sized conservatively and the exact
ring test is never skipped -- which is precisely why the pruning caveat in
`quench._group_offsets` (its docstring explains that lifting the per-member seed cap
makes the pruned lists lossy) does not apply to anything here. `perturb.py` and
`recovery.py` state the same contract for the same reason, and
`tests/test_quench_neighbor_lists.py` already runs a full `quench()` with the
builder disabled and asserts identical output. So #554's three named invariants are
not "unlocked" here; they are **not engaged**, and no line of `quench.py` or
`legality.py` changes.

**The state must be built with `move_refs=None`.** `QuenchState.__init__` turns
every ref OUTSIDE `move_refs` into a locked part, and `place_route_loop` passes its
target selection as `move_refs`. Handed such a state, this solve would have zero
free variables, would report `no_room_at_any_dose` on every board, and would look
exactly like a correct negative result. `assert_relocatable_state` refuses it.

WHAT IS NOT HERE, AND WHY
-------------------------
* **Any judgement about routability.** No hpwl, no crossings, no MST, no
  `nets_cost`. The objective is millimetres. This is deliberate, and it is the
  reason `reconstruct._net_anchor_cost` is not reused: that objective was measured
  to price a self-consistent displaced island at all-stay ("a pairwise spring proxy
  was measured WRONG here ... only the gate's own currency can price a joint move"),
  and a diagnosed block IS such an island by construction. A relocation that
  shortens displacement and worsens routing is *supposed* to be rejected -- by the
  router, downstream, and by nothing in this module.
* **Choosing the block.** `placement.diagnosis` does. Every proposal therefore
  carries TWO disclosures: this module's `NO_EFFICACY_CLAIM` (nothing shows a
  relocated board routes better) and the diagnosis's own (nothing shows its
  selection routes better than a pin count). They are different gaps and neither
  substitutes for the other.
* **Block rotation.** `perturb` refuses it as a damage arm for a measured reason: on
  any non-square block the result is dominated by courtyard damage rather than
  displacement. The same geometry refuses it as a repair, and a rotating rectangle
  breaks the frozen-disjunction argument outright.
* **Swapping.** A swap FLIPS a relative order, which is the negation of the hard
  constraint, and it moves an undiagnosed second block. It is named here because it
  is the measured arm that reaches 14-66 mm where a rigid translate reaches ~0.1 mm:
  **when this solve answers `no_room_at_any_dose`, a swap may still exist and this
  module will never find it.** A known gap, not an oversight.
* **Legalizing the incumbent.** Baseline-relative gaps hold every existing violation
  constant. This pass may not deepen one and does not promise to heal one; that is
  `seeder.repair_placement` and `fanout_clearance`.
* **The board outline as a solve constraint.** The system knows the usable bbox and
  nothing else. Rings, cutouts, notches and milled slots are non-convex and enter
  only at the exact re-check.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

SCHEMA = 1

#: Coordinates are rounded here before any comparison or dict key, so a result
#: cannot reorder between processes on floating-point noise.
ROUND_MM = 6

#: What has and has not been measured about this pass. Carried on every
#: `Relocation` and stamped into the loop's verdict as `relocate_efficacy`, so a
#: machine consumer cannot read the numbers without the sentence.
#:
#: It is DELIBERATELY not `diagnosis.NO_EFFICACY_CLAIM` reused verbatim. That
#: sentence is about a selector -- "no paired routed A/B of pins vs diagnosis
#: exists" -- and pasting it here would name the wrong comparison while looking
#: like a disclosure. The two are both carried, because a relocation of a block
#: the diagnosis chose inherits BOTH gaps.
NO_EFFICACY_CLAIM = (
    'NOT MEASURED: no paired routed A/B of relocate-on vs relocate-off exists, '
    'so nothing shows a relocated board ROUTES better. What IS measured is the '
    'mechanism -- letting neighbours yield in preserved order buys a block more '
    'travel than freezing them, on 11 of the 24 measurable blocks, spanning 6 '
    'of the 9 boards that have one '
    '(tests/stress/relocation_reach.py). Reach is not routability.')

#: The two pinned wall nodes per axis, and the super-source. Names chosen so they
#: can never collide with a KiCad reference (which cannot contain a space).
SOURCE = '<source>'
WALL_LO = '<wall lo>'
WALL_HI = '<wall hi>'
_RESERVED = (SOURCE, WALL_LO, WALL_HI)

AXES = ('x', 'y')


class RelocateError(ValueError):
    """A caller handed this module a state or a block it cannot work with."""


# ---------------------------------------------------------------------------
# records
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class OrderEdge:
    """`s[hi] - s[lo] >= slack` on one axis: the pair's frozen relative order.

    `slack = gap_mm - separation_mm`, and is <= 0 exactly because `gap_mm` is
    `min(clearance, separation_mm)` -- which is what makes `s = 0` feasible.
    """
    axis: str
    lo: str            # unit on the low side of the axis
    hi: str            # unit on the high side
    gap_mm: float      # what the pair must keep
    sep_mm: float      # what it has now
    slack: float       # gap_mm - sep_mm, <= 0
    source: str        # 'clearance' | 'baseline' | 'wall'
    lo_ref: str = ''   # the parts the pair came from, for the explanation
    hi_ref: str = ''

    def to_dict(self):
        return {'axis': self.axis, 'lo': self.lo, 'hi': self.hi,
                'gap_mm': round(self.gap_mm, 4), 'sep_mm': round(self.sep_mm, 4),
                'slack': round(self.slack, 4), 'source': self.source,
                'lo_ref': self.lo_ref, 'hi_ref': self.hi_ref}


@dataclass(frozen=True)
class Units:
    """The rigid-unit partition. Every movable part is in exactly one unit."""
    of_ref: Dict[str, str]                  # ref -> unit name
    members: Dict[str, Tuple[str, ...]]     # unit name -> sorted refs
    pinned: frozenset                       # units that cannot move at all

    @property
    def free(self) -> Tuple[str, ...]:
        return tuple(u for u in sorted(self.members) if u not in self.pinned)


@dataclass
class Reach:
    """How far one unit may travel, and the chain of parts that stopped it."""
    unit: str
    lo: Dict[str, float] = field(default_factory=dict)   # axis -> min shift
    hi: Dict[str, float] = field(default_factory=dict)   # axis -> max shift
    binding: Dict[str, Tuple] = field(default_factory=dict)  # axis -> path rows
    reach_mm: float = 0.0        # along the requested direction
    #: WHICH axis produced `reach_mm`. Reporting the x chain unconditionally
    #: names the wrong parts whenever y is the binding one -- measured on 8 of
    #: the 24 live corpus cells, and on all 8 the x chain differs -- including
    #: the headline kit-dev-coldfire/net:JTAG, where
    #: the printed chain was `FB101 -> C114 -> C119` and the chain that actually
    #: stopped it was `Y101 -> R207 -> <wall hi>`. An explanation that names the
    #: wrong parts is worse than none.
    binding_axis: str = ''
    refusal: str = ''

    @property
    def binding_chain(self) -> Tuple:
        """The chain on the axis that actually bound, nearest-first."""
        return self.binding.get(self.binding_axis, ())

    def to_dict(self):
        return {'unit': self.unit,
                'lo': {a: round(v, 4) for a, v in sorted(self.lo.items())},
                'hi': {a: round(v, 4) for a, v in sorted(self.hi.items())},
                'binding': {a: [list(r) for r in p]
                            for a, p in sorted(self.binding.items())},
                'binding_axis': self.binding_axis,
                'reach_mm': round(self.reach_mm, 4),
                'refusal': self.refusal}


# ---------------------------------------------------------------------------
# preconditions
# ---------------------------------------------------------------------------

def assert_relocatable_state(state) -> None:
    """Refuse a state whose free variables were removed before we got it.

    `QuenchState.__init__` locks every ref outside `move_refs`. A state built that
    way has no corridor at all, so the solve would answer `no_room_at_any_dose` on
    every board -- indistinguishable, in the output, from a correct negative. There
    is no public flag recording that `move_refs` was passed, so the check is on the
    consequence: a board on which nothing but the block may yield is either that
    mistake or a genuinely fully-locked board, and the caller must say which.
    """
    movable = sum(1 for p in state.parts.values() if not p.locked)
    if movable < 2:
        raise RelocateError(
            'relocate needs a state whose neighbours may yield: %d of %d parts are '
            'movable. Build it with pose_score.make_state(..., move_refs=None) -- '
            'QuenchState locks every ref OUTSIDE move_refs, and a relocation solve '
            'handed such a state reports "no room" on every board.'
            % (movable, len(state.parts)))


# ---------------------------------------------------------------------------
# units
# ---------------------------------------------------------------------------

def pin_all_but(units: Units, unit: str) -> Units:
    """The same partition with every OTHER unit pinned -- the paired control.

    `reach` on this answers "how far does the block go with the neighbours frozen",
    under the identical feasibility rule, the identical graph and the identical
    solver. That is the only honest comparison for the claim this module exists to
    make: the arms then differ in exactly one thing, whether neighbours may yield.

    It is deliberately NOT the same number as
    `tests/stress/relocation_reach.py`'s slide-until-contact `frozen_mm`, and the
    difference is worth stating because the two disagreed and the disagreement
    looked like a bug: the slide gates on the block's TOTAL violation being no
    worse, so one pair may worsen while another improves, while this gates PER
    PAIR and additionally forbids two parts exchanging sides. The slide is the
    looser rule, so it can and does report further travel on some cells. Neither
    is wrong; they answer different questions, and only this one is paired.
    """
    return Units(of_ref=units.of_ref, members=units.members,
                 pinned=frozenset(u for u in units.members if u != unit))


def rigid_units(state, blocks: Optional[Dict[str, Sequence[str]]] = None) -> Units:
    """Partition every part into rigid units.

    A block whose members are not all present, or which shares a member with an
    earlier block, keeps only the members it uniquely owns -- `derive_groups`
    already guarantees a ref lands in at most one group, so this is a guard against
    a hand-built dict, not a routine path.

    A unit is PINNED when any member is locked. That is deliberately all-or-nothing
    and matches `perturb.candidate_blocks`' rule: a block containing a locked part
    is refused whole rather than silently shrunk, because a block that cannot
    translate rigidly is not the block that was named.
    """
    of_ref: Dict[str, str] = {}
    members: Dict[str, Tuple[str, ...]] = {}
    for name in sorted(blocks or {}):
        if name in _RESERVED:
            raise RelocateError('block name %r collides with a reserved node' % name)
        refs = sorted(r for r in blocks[name]
                      if r in state.parts and r not in of_ref)
        if len(refs) < 2:
            continue
        members[name] = tuple(refs)
        for r in refs:
            of_ref[r] = name
    for ref in sorted(state.parts):
        if ref in of_ref:
            continue
        members[ref] = (ref,)
        of_ref[ref] = ref
    pinned = frozenset(u for u, refs in members.items()
                       if any(state.parts[r].locked for r in refs))
    return Units(of_ref=of_ref, members=members, pinned=pinned)


# ---------------------------------------------------------------------------
# the order graph
# ---------------------------------------------------------------------------

def _axis_sep(ra, rb, axis):
    """Signed separation of two rects on one axis, and which is on the low side.

    Positive when disjoint on that axis; negative by the overlap depth when not.
    """
    i = 0 if axis == 'x' else 1
    j = i + 2
    if ra[i] + ra[j] <= rb[i] + rb[j]:          # a's centre is lower
        return rb[i] - ra[j], True
    return ra[i] - rb[j], False


def order_graph(state, units: Units, *, clearance: Optional[float] = None
                ) -> List[OrderEdge]:
    """One difference constraint per interacting pair, plus the walls.

    Pair admission and the required gap come from `_Part.gap_to` ->
    `legality.pair_min_gap`, CALLED rather than mirrored: it is the authority on
    the board-side rule and on a THT part's far-side lead field, and a `None`
    return means the two parts share no side and cannot interact at all.

    Intra-unit pairs contribute nothing: a rigid translate leaves their geometry
    invariant, which is the same reason `quench.group_move_valid` excludes them.
    """
    clr = state.clearance if clearance is None else clearance
    refs = sorted(state.parts)
    rects = {r: state.parts[r].rects() for r in refs}
    edges: List[OrderEdge] = []
    seen = set()

    for i, a in enumerate(refs):
        pa = state.parts[a]
        ua = units.of_ref[a]
        for b in refs[i + 1:]:
            ub = units.of_ref[b]
            if ua == ub:
                continue
            pb = state.parts[b]
            gap = pa.gap_to(pb, rects[a], rects[b])
            if gap is None:            # no shared board side: no interaction
                continue
            ra, rb = rects[a][0], rects[b][0]
            sep_x, a_low_x = _axis_sep(ra, rb, 'x')
            sep_y, a_low_y = _axis_sep(ra, rb, 'y')
            # The disjunction, frozen from the incumbent: constrain the axis the
            # pair is currently MORE separated on. Ties go to x, then to the ref
            # pair, so the choice cannot vary between processes.
            if round(sep_x, ROUND_MM) >= round(sep_y, ROUND_MM):
                axis, sep, a_low = 'x', sep_x, a_low_x
            else:
                axis, sep, a_low = 'y', sep_y, a_low_y
            lo_ref, hi_ref = (a, b) if a_low else (b, a)
            lo, hi = units.of_ref[lo_ref], units.of_ref[hi_ref]
            if lo == hi:
                continue
            # min(): what makes s = 0 feasible, and what stops this pass
            # legalizing (or deepening) a violation it did not create.
            need = min(clr, sep)
            key = (axis, lo, hi, round(need - sep, ROUND_MM))
            if key in seen:
                continue
            seen.add(key)
            edges.append(OrderEdge(
                axis=axis, lo=lo, hi=hi, gap_mm=need, sep_mm=sep,
                slack=need - sep,
                source='clearance' if need >= clr - 1e-12 else 'baseline',
                lo_ref=lo_ref, hi_ref=hi_ref))

    edges.extend(_wall_edges(state, units))
    edges.sort(key=lambda e: (e.axis, e.lo, e.hi, e.lo_ref, e.hi_ref))
    return edges


def _wall_edges(state, units: Units) -> List[OrderEdge]:
    """Keep every unit inside the usable bbox, as two difference constraints.

    The walls are the ONLY outline knowledge in the solve. Rings, cutouts and
    notches are non-convex, have no difference-constraint form, and are enforced at
    the exact re-check instead.

    The required margin is `min(0, current)`, exactly as a pair's is
    `min(clearance, current)` -- and for the same load-bearing reason. A board that
    ships copper outside the usable inset is ordinary (edge connectors, castellated
    rows, card edges; `perturb.max_feasible_dose` measures against each member's own
    baseline for this reason). Requiring a flat 0 there would give the wall a
    POSITIVE slack, `s = 0` would violate it, and the identity-is-always-feasible
    guarantee the whole formulation rests on would be false. Measured when it was:
    watchy's `decap:U1` reported a reach of 0.00 mm against a frozen slide of
    1.45 mm -- the solve claiming less room than the same block has with every
    neighbour nailed down, which is impossible and was the tell.
    """
    usable = getattr(state, 'usable', None)
    if not usable:
        return []
    out: List[OrderEdge] = []
    for unit in sorted(units.members):
        if unit in units.pinned:
            continue
        rs = [state.parts[r].rect() for r in units.members[unit]]
        for axis in AXES:
            i = 0 if axis == 'x' else 1
            lo_edge = min(r[i] for r in rs)
            hi_edge = max(r[i + 2] for r in rs)
            first = units.members[unit][0]
            # unit's low edge must stay at or above the low wall
            sep = lo_edge - usable[i]
            need = min(0.0, sep)
            out.append(OrderEdge(axis=axis, lo=WALL_LO, hi=unit,
                                 gap_mm=need, sep_mm=sep, slack=need - sep,
                                 source='wall', lo_ref=WALL_LO, hi_ref=first))
            # ... and its high edge at or below the high wall
            sep = usable[i + 2] - hi_edge
            need = min(0.0, sep)
            out.append(OrderEdge(axis=axis, lo=unit, hi=WALL_HI,
                                 gap_mm=need, sep_mm=sep, slack=need - sep,
                                 source='wall', lo_ref=first, hi_ref=WALL_HI))
    return out


# ---------------------------------------------------------------------------
# the envelope, by shortest path
# ---------------------------------------------------------------------------

def identity_violations(edges, tol: float = 1e-9) -> List[OrderEdge]:
    """Edges that `s = 0` does NOT satisfy. Must always be empty.

    Every gap is `min(what is required, what the pair already has)`, so every slack
    is <= 0 and the incumbent board is feasible by construction. This is not a
    belt-and-braces check: the property is what makes `reach` an envelope rather
    than a guess, what stops the pass legalizing a board it did not damage, and
    what guarantees a refusal is a refusal rather than an exception. It shipped
    false once -- the wall edges were not baseline-clamped -- and the symptom was a
    reach SMALLER than the same block's frozen slide, which no reader would have
    attributed to the graph.
    """
    return [e for e in edges if e.slack > tol]


def _pinned_nodes(units: Units) -> List[str]:
    return sorted(set(units.pinned) | {WALL_LO, WALL_HI})


def _spfa(nodes, adj, sources):
    """Shortest path from a virtual super-source; None on a negative cycle.

    Returns `(dist, pred)`. SPFA rather than plain Bellman-Ford because the graph is
    sparse and nearly acyclic in practice; the per-node relaxation counter is the
    negative-cycle guard and it is exact, not a heuristic bound.
    """
    dist = {n: math.inf for n in nodes}
    pred: Dict[str, Optional[Tuple]] = {n: None for n in nodes}
    q = deque()
    inq = set()
    for s in sources:
        dist[s] = 0.0
        q.append(s)
        inq.add(s)
    count = {n: 0 for n in nodes}
    limit = len(nodes) + 1
    while q:
        u = q.popleft()
        inq.discard(u)
        du = dist[u]
        for v, w, why in adj.get(u, ()):  # already in a deterministic order
            nd = du + w
            if nd < dist[v] - 1e-12:
                dist[v] = nd
                pred[v] = (u, w, why)
                if v not in inq:
                    count[v] += 1
                    if count[v] > limit:
                        return None, None
                    q.append(v)
                    inq.add(v)
    return dist, pred


def _adjacency(edges, axis, fixed, *, upper: bool):
    """Edges for one axis, oriented for the upper or the lower envelope.

    Every constraint is `s_hi - s_lo >= slack`.
      upper: `s_lo <= s_hi - slack`  -> relax hi -> lo with weight `-slack`
      lower: `s_hi >= s_lo + slack`  -> negate and relax lo -> hi with `-slack`,
             so the same shortest-path routine yields `-lo`.

    A FIXED node (a locked unit, a wall, or -- in the paired control -- every unit
    but one) is a CONSTANT, not a variable, so it takes no incoming edge. Seeding
    it as a source at 0 is not enough on its own: a negative-weight path could
    relax it below 0, and the answer would then quietly describe a board on which a
    locked part moved. Dropping the incoming edge is what makes `hi = lo = 0`
    hold exactly for everything that may not move.
    """
    adj: Dict[str, List[Tuple[str, float, Tuple]]] = {}
    for e in edges:
        if e.axis != axis:
            continue
        src, dst = (e.hi, e.lo) if upper else (e.lo, e.hi)
        if dst in fixed:
            continue
        why = (e.hi_ref if upper else e.lo_ref, round(e.gap_mm, 4), e.source)
        adj.setdefault(src, []).append((dst, -e.slack, why))
    for k in adj:
        adj[k].sort(key=lambda t: (t[0], t[1]))
    return adj


def envelope(state, units: Units, edges) -> Tuple[Dict, Dict, Dict, str]:
    """Per-axis `(hi, lo, binding, refusal)` for every unit.

    The solution set of a difference-constraint system is a lattice, so the
    componentwise maximum is itself a FEASIBLE assignment -- which is why `hi` is
    the block's true reach and not an optimistic bound.
    """
    bad = identity_violations(edges)
    if bad:
        e = bad[0]
        return {}, {}, {}, (
            'order_graph_infeasible: %d edge(s) refuse the INCUMBENT board, e.g. '
            '%s %s->%s needs %.4f but has %.4f. Every gap must be clamped to what '
            'the pair already has, or s = 0 is not feasible and nothing below '
            'means what it says.' % (len(bad), e.axis, e.lo, e.hi, e.gap_mm,
                                     e.sep_mm))
    nodes = sorted(set(units.members) | {WALL_LO, WALL_HI})
    sources = _pinned_nodes(units)
    fixed = frozenset(sources)
    hi: Dict[str, Dict[str, float]] = {}
    lo: Dict[str, Dict[str, float]] = {}
    binding: Dict[str, Dict[str, Tuple]] = {}
    for axis in AXES:
        du, pu = _spfa(nodes, _adjacency(edges, axis, fixed, upper=True), sources)
        if du is None:
            return {}, {}, {}, ('order_graph_infeasible: a negative cycle on the '
                                '%s axis -- identity should always be feasible, so '
                                'this is a bug in the graph, not a board' % axis)
        dl, _pl = _spfa(nodes, _adjacency(edges, axis, fixed, upper=False), sources)
        if dl is None:
            return {}, {}, {}, ('order_graph_infeasible: a negative cycle on the '
                                '%s axis (lower envelope)' % axis)
        hi[axis] = {n: du[n] for n in nodes}
        lo[axis] = {n: -dl[n] for n in nodes}
        binding[axis] = pu
    return hi, lo, binding, ''


def _path_to(pred, node, limit=64):
    """The chain that bound `node`, nearest-first. A NAMED chain, not a number."""
    out = []
    seen = set()
    cur = node
    while cur is not None and cur not in seen and len(out) < limit:
        seen.add(cur)
        step = pred.get(cur)
        if step is None:
            break
        prev, _w, why = step
        out.append((why[0], why[1], why[2]))
        cur = prev
    return tuple(out)


def reach(state, units: Units, edges, unit: str,
          direction: Tuple[float, float]) -> Reach:
    """How far `unit` travels along `direction` with neighbours yielding in order.

    This is the number the whole feature rests on. Its control is
    `tests/stress/relocation_reach.py`'s `frozen_mm`, the same question asked with
    every non-member frozen.

    TWO THINGS IT IS NOT, both measured rather than argued:

    * **It is board-frame TRAVEL, not closure on the target.** The direction comes
      from `net_centroid`, which is computed from pads on parts OUTSIDE the block
      -- and those parts move too. So a block travelling `reach_mm` does not
      close `reach_mm` of the gap to what it connects to; the target drifts with
      the corridor. Bounded on the corpus, the true closure is smaller on most
      cells and LARGER on a few (a neighbour yielding can carry the target
      toward the block): kit-dev-coldfire `net:JTAG` travels 16.78 mm and closes
      at most 13.05 mm, while splitflap `decap:U10` travels 0.00 and could close
      5.72. Those figures come from a review-time probe that is NOT in this repo
      and that nothing re-derives -- an order-of-magnitude illustration, not a
      committed measurement. The aggregate mechanism verdict survives the
      substitution; no individual cell's number is the closure.
    * **It is not a legality guarantee.** See the graph-is-not-conservative
      section in the module docstring. Only `exact_refusal` gates a board.
    """
    r = Reach(unit=unit)
    if unit in units.pinned:
        r.refusal = 'block_member_locked: the unit contains a locked part'
        return r
    hi, lo, binding, refusal = envelope(state, units, edges)
    if refusal:
        r.refusal = refusal
        return r
    ux, uy = direction
    norm = math.hypot(ux, uy)
    if norm <= 0:
        r.refusal = 'block_has_no_target: the direction is a zero vector'
        return r
    ux, uy = ux / norm, uy / norm
    r.hi = {a: hi[a][unit] for a in AXES}
    r.lo = {a: lo[a][unit] for a in AXES}
    r.binding = {a: _path_to(binding[a], unit) for a in AXES}
    caps = []
    for axis, comp in (('x', ux), ('y', uy)):
        if abs(comp) < 1e-12:
            continue
        bound = r.hi[axis] if comp > 0 else r.lo[axis]
        if not math.isfinite(bound):
            continue
        caps.append(bound / comp)
    if not caps:
        # Nothing bounds this unit on either axis -- no wall, no neighbour it
        # shares a side with. Returning `inf` would be true and useless: it
        # serialises as the literal `Infinity`, which is not valid JSON, so it
        # would poison any baseline that recorded it. Name it instead.
        r.refusal = ('unbounded: no wall and no interacting neighbour limits '
                     'this unit, so "how far can it go" has no answer')
        return r
    # FLOOR, not round-to-nearest. `reach_mm` is fed straight back in as a dose
    # ceiling, and rounding UP puts it past the envelope it came from: measured,
    # 7 of 24 corpus cells reported a value 3e-6..4e-5 mm above the exact bound,
    # and HiGHS answered `Infeasible` for splitflap_driver/decap:U3 at its own
    # published reach of 11.0236 (exact 11.023577). A reported reach must always
    # be achievable.
    r.reach_mm = max(0.0, math.floor(min(caps) * 10000) / 10000)
    r.binding_axis = min(
        (('x', ux), ('y', uy)),
        key=lambda ac: (((r.hi[ac[0]] if ac[1] > 0 else r.lo[ac[0]]) / ac[1])
                        if abs(ac[1]) >= 1e-12 else math.inf))[0]
    return r


# ---------------------------------------------------------------------------
# minimum perturbation
# ---------------------------------------------------------------------------

def _axis_rows(edges, axis, fixed_values):
    """(free unit names, rows) for one axis. A row is (coeffs, rhs) for >= rhs.

    A FIXED node contributes a constant, moved to the right-hand side, rather
    than a column. That is what pins the block at its dose and the walls at zero
    in the same mechanism, with no special case for either.
    """
    free = sorted({u for e in edges if e.axis == axis
                   for u in (e.lo, e.hi) if u not in fixed_values})
    idx = {u: i for i, u in enumerate(free)}
    rows = []
    for e in edges:
        if e.axis != axis:
            continue
        coeffs = {}
        rhs = e.slack
        for u, sign in ((e.hi, 1.0), (e.lo, -1.0)):
            if u in fixed_values:
                rhs -= sign * fixed_values[u]
            else:
                coeffs[idx[u]] = coeffs.get(idx[u], 0.0) + sign
        if not coeffs:
            # Both ends fixed: nothing to solve, but it still has to HOLD, or
            # the dose is simply not admissible.
            if rhs > 1e-9:
                return free, None
            continue
        rows.append((coeffs, rhs))
    return free, rows


def _lp_min_perturbation(free, rows, lo, hi, names):
    """Minimise sum |s| subject to the rows. `None` when scipy is absent."""
    try:
        import numpy as np
        from scipy.optimize import milp, LinearConstraint, Bounds
        from scipy.sparse import lil_matrix
    except ImportError:
        return None
    n = len(free)
    if n == 0:
        return {}
    # s = p - q, both >= 0; minimising p + q minimises |s| exactly.
    A = lil_matrix((len(rows), 2 * n))
    lb = np.empty(len(rows))
    ub = np.full(len(rows), np.inf)
    for i, (coeffs, rhs) in enumerate(rows):
        for j, c in coeffs.items():
            A[i, j] = c
            A[i, n + j] = -c
        lb[i] = rhs
    ubs = np.empty(2 * n)
    lbs = np.zeros(2 * n)
    for j, u in enumerate(free):
        ubs[j] = max(0.0, hi.get(u, math.inf))
        ubs[n + j] = max(0.0, -lo.get(u, -math.inf))
    ubs = np.where(np.isfinite(ubs), ubs, np.inf)
    res = milp(c=np.ones(2 * n),
               constraints=LinearConstraint(A.tocsr(), lb, ub),
               integrality=np.zeros(2 * n), bounds=Bounds(lbs, ubs))
    if res.status != 0 or res.x is None:
        return False        # infeasible, as distinct from "no solver"
    return {u: round(res.x[j] - res.x[n + j], ROUND_MM)
            for j, u in enumerate(free)}


def _sweep_min_perturbation(free, rows, lo, hi, names, max_passes=200):
    """Raise-to-satisfy fallback. A HEURISTIC: feasible, not minimal.

    Starts every unit at 0 -- the value it would keep if nothing forced it -- and
    repeatedly raises whichever side of an unsatisfied row is cheapest to raise,
    which terminates because values only increase and are capped by `hi`. It can
    move more parts, and further, than the LP would; it can never return an
    infeasible assignment, because it checks before it returns. Reported as
    `solver: 'sweep'` so a reader can tell "the heuristic found less room" from
    "there is less room".
    """
    s = [0.0] * len(free)
    cap = [hi.get(u, math.inf) for u in free]
    for _ in range(max_passes):
        moved = False
        for coeffs, rhs in rows:
            val = sum(c * s[j] for j, c in coeffs.items())
            if val >= rhs - 1e-9:
                continue
            need = rhs - val
            # Raise the positive-coefficient column with the most headroom.
            best, best_room = None, 0.0
            for j, c in coeffs.items():
                if c <= 0:
                    continue
                room = cap[j] - s[j]
                if room > best_room:
                    best, best_room = j, room
            if best is None or best_room < need / max(
                    coeffs.get(best, 1.0), 1e-9) - 1e-9:
                return False
            s[best] += need / coeffs[best]
            moved = True
        if not moved:
            break
    for coeffs, rhs in rows:
        if sum(c * s[j] for j, c in coeffs.items()) < rhs - 1e-6:
            return False
    return {u: round(s[j], ROUND_MM) for j, u in enumerate(free)}


def min_perturbation(state, units: Units, edges, block: str,
                     shift: Tuple[float, float], pinned=()
                     ) -> Tuple[Optional[Dict[str, Tuple[float, float]]], str]:
    """Smallest total neighbour displacement that lets `block` take `shift`.

    Returns `({unit: (dx, dy)}, solver)`, or `(None, reason)`. The corridor is an
    OUTPUT: `{u for u, d in result.items() if d != (0, 0)}` is the set of parts
    that had to yield, and it is minimal in millimetres -- there is no radius
    knob and no scope glob, because a second definition of the same thing is a
    second thing to get wrong.
    """
    hi, lo, _b, refusal = envelope(state, units, edges)
    if refusal:
        return None, refusal
    fixed_extra = {u: 0.0 for u in pinned}
    out: Dict[str, Dict[str, float]] = {}
    solver = 'milp'
    for axis, comp in zip(AXES, shift):
        fixed = {WALL_LO: 0.0, WALL_HI: 0.0, block: comp}
        fixed.update({u: 0.0 for u in units.pinned})
        fixed.update(fixed_extra)
        free, rows = _axis_rows(edges, axis, fixed)
        if rows is None:
            return None, ('no_room_at_any_dose: two fixed parts alone refuse '
                          'the %s shift' % axis)
        got = _lp_min_perturbation(free, rows, lo[axis], hi[axis], free)
        if got is None:
            solver = 'sweep'
            got = _sweep_min_perturbation(free, rows, lo[axis], hi[axis], free)
        if got is False:
            # The refusal has to name the SWEEP when the sweep produced it.
            # Measured with scipy absent, 5 of 12 corpus cells flip from a real
            # relocation to "no room" and a 6th collapses to the frozen answer
            # with an empty corridor -- because the sweep only ever RAISES a
            # variable, so any system needing a negative shift is infeasible to
            # it. Without this, `rel.solver` stays '' on the refusal path and a
            # reader cannot tell "the heuristic found less room" from "there is
            # less room" -- which is exactly what this module's own docstring
            # promised they could.
            return None, (
                'no_room_at_any_dose: the %s system is infeasible at this dose%s'
                % (axis,
                   ' -- SOLVED BY THE FALLBACK SWEEP, not the LP: scipy is '
                   'unavailable, and the sweep can only raise variables, so a '
                   'system needing a negative shift is infeasible to it. This '
                   'is not necessarily a board with no room.'
                   if solver == 'sweep' else ''))
        for u, v in got.items():
            out.setdefault(u, {})[axis] = v
    moves = {u: (round(d.get('x', 0.0), ROUND_MM), round(d.get('y', 0.0), ROUND_MM))
             for u, d in out.items()}
    moves[block] = (round(shift[0], ROUND_MM), round(shift[1], ROUND_MM))
    for u in pinned:
        moves[u] = (0.0, 0.0)
    return moves, solver


# ---------------------------------------------------------------------------
# the exact re-check
# ---------------------------------------------------------------------------

def exact_refusal(state, units: Units, moves, tol: float = 1e-6) -> str:
    """'' when every shifted part is acceptable at its new pose, else a NAMED reason.

    The LP is not a uniform relaxation of the gate and both directions matter:

    * **Stricter** on the courtyard conjunct -- it measures per-axis edge
      separation while the gate measures the Euclidean `rect_gap`, so a pair
      offset diagonally by (0.2, 0.2) at clearance 0.25 fails every axis test
      while its true gap is 0.283 and legal. The solve therefore refuses some
      legal shifts. That costs reach and can never ship an illegal board.
    * **Looser** on four conjuncts it has no representation of: the real
      Edge.Cuts rings and cutouts, pad/hole pairs, FAB body containment, and
      declared intent. Those are why nothing may be applied on the LP's word.

    **The geometry conjunct is BASELINE-RELATIVE, and that is not a softening.**
    `candidate_valid` asks "is this pose fully legal", and measured on this
    corpus it answers NO at the INCUMBENT pose for 35% of esp_prog's parts, 61%
    of tigard's, 49% of splitflap's and 99% of watchy's -- human placements sit
    below the 0.25 mm courtyard clearance the optimizer asks for. Gated on it,
    this pass refuses every shift on every real board, for a reason that has
    nothing to do with the shift; that is exactly the artefact that made the
    first version of `tests/stress/relocation_reach.py` report a phantom null.
    So the rule is the one the order graph is already built on and the one
    `pads_ok` already applies: **no worse than the board we were handed.**

    The conjuncts that are NOT relaxed, because they are declarations rather
    than incumbent geometry: keep-outs and declared intent zones. A re-seat
    withholds those (arming them would make it refuse its own target); a
    relocation moves a COMPLIANT block, so a declared zone is exactly what
    should stop it.

    Every moved part is `exclude`d from its own check, because `state.parts`
    still holds the OLD poses: testing a new pose against stale neighbours tests
    a board that will never exist -- and the baseline is measured with the same
    exclusion, so the two sides are like for like. Moved-vs-moved pairs are then
    checked separately at both new poses, through `gap_to`.
    """
    moved = {r: (state.parts[r].x + d[0], state.parts[r].y + d[1])
             for u, d in moves.items() if d != (0.0, 0.0)
             for r in units.members.get(u, ())}
    if not moved:
        return ''
    ex = set(moved)
    ctx = getattr(state, 'legality_ctx', None)
    for ref in sorted(moved, key=lambda r: (-abs(moved[r][0] - state.parts[r].x)
                                            - abs(moved[r][1] - state.parts[r].y),
                                            r)):
        nx, ny = moved[ref]
        part = state.parts[ref]
        before = sum(state.violation_parts(ref, part.x, part.y, part.rot,
                                           exclude=ex))
        after = sum(state.violation_parts(ref, nx, ny, part.rot, exclude=ex))
        if after > before + tol:
            return 'geometry_worsened:%s' % ref
        rects = part.rects(nx, ny, part.rot)
        keepout_clear = getattr(state, 'keepout_clear', None)
        if keepout_clear is not None and not keepout_clear(ref, rects):
            # MONOTONE, matching the declared-intent gate's own rule: it
            # prevents a part being walked INTO a keep-out, it does not freeze
            # one that is already in it. The absolute form looks stricter and is
            # worse: a single neighbour sitting inside a declared zone would
            # then be unable to yield at all, and one declaration would freeze
            # the whole corridor -- measured on a fixture where the keep-out
            # overlapped the very neighbour the block needed to push.
            if keepout_clear(ref, part.rects(part.x, part.y, part.rot)):
                blockers = getattr(state, 'keepout_blockers', None)
                named = ','.join(blockers(ref, rects)) if blockers else ''
                return 'declared_keepout_refused_a_shift:%s:%s' % (ref, named)
        intent_ok = getattr(state, 'intent_ok', None)
        if intent_ok is not None and not intent_ok(ref, nx, ny, part.rot):
            return 'declared_claim_refused_a_shift:%s' % ref
        if ctx is not None:
            pads_ok = getattr(ctx, 'pads_ok', None)
            nbrs = [r for r in state.parts if r != ref and r not in ex]
            if pads_ok is not None and not pads_ok(ref, nx, ny, part.rot, nbrs):
                return 'pad_gate_refused_a_shift:%s' % ref
    order = sorted(moved)
    for i, a in enumerate(order):
        pa = state.parts[a]
        ra = pa.rects(moved[a][0], moved[a][1], pa.rot)
        for b in order[i + 1:]:
            if units.of_ref[a] == units.of_ref[b]:
                continue        # rigid: their geometry did not change
            pb = state.parts[b]
            gap = pa.gap_to(pb, ra, pb.rects(moved[b][0], moved[b][1], pb.rot))
            if gap is None:
                continue
            was = pa.gap_to(pb)
            if gap < min(state.clearance, was if was is not None else 0.0) - tol:
                return 'moved_pair_worsened:%s:%s' % (a, b)
            # ... and the PAD/HOLE currency for the same pair, which the
            # per-part sweep above cannot reach: it passes `pads_ok` a neighbour
            # list with every moved ref REMOVED (their stored poses are stale),
            # so a pair where BOTH parts move is checked on courtyards only.
            #
            # That is not hypothetical. Measured on esp_prog/decap:U1, the LP
            # drives a courtyard gap to EXACTLY the clearance -- so the test
            # above is satisfied to the micron -- while the pads, which overhang
            # the courtyard, cross: C4/Q2 and U1/U2 each gained a pad conflict
            # that `grade_pad_legality` reports and nothing here asked about.
            # The shortfalls were 80 nm and 50 nm, under the ~8 um grid noise
            # CLAUDE.md says to filter, so the harm was small -- but "can never
            # ship an illegal board" was false in KIND, and that is the claim.
            if ctx is not None and hasattr(ctx, 'pair_shortfall'):
                pose_a = (moved[a][0], moved[a][1], pa.rot)
                pose_b = (moved[b][0], moved[b][1], pb.rot)
                try:
                    now = ctx.pair_shortfall(a, b, pose_a=pose_a, pose_b=pose_b)
                    base = ctx.seed_baseline(a, b)
                except Exception:                          # noqa: BLE001
                    continue     # no pad model for this pair; courtyards stand
                for field in ('pad', 'pad_overlap', 'hole', 'stack'):
                    n_v = getattr(now, field, 0.0) or 0.0
                    b_v = getattr(base, field, 0.0) or 0.0
                    if n_v > b_v + tol:
                        return 'moved_pair_pad_worsened:%s:%s:%s' % (a, b, field)
    return ''


# ---------------------------------------------------------------------------
# the pass
# ---------------------------------------------------------------------------

#: Fractions of the reach-clamped target, largest first, first survivor wins.
#: At most `len(DOSE_LADDER) x (MAX_CUTS + 1)` = 16 solves, and measured on the
#: corpus exactly `len(DOSE_LADDER)` or fewer, because the cut loop has never
#: fired (see MAX_CUTS). Against a grid: the largest reach anywhere in the study
#: is 16.78 mm, where `_group_offsets` at `--step 0.5` enumerates ~3,540
#: offsets; the 80 mm case #459 was filed about would be ~103,000.
DOSE_LADDER = (1.0, 0.75, 0.5, 0.25)

#: How many times a refusing unit may be pinned and the system re-solved.
#:
#: MEASURED: it has never fired. Over 24 live corpus cells, every one of the 13
#: accepted proposals came back with `cuts == 0`, and the three cells that DID
#: reach a pinned re-solve were infeasible at every one of them. The structural
#: reason is that the unit the exact check refuses is, by construction, one that
#: had to yield -- so pinning it at 0 makes that same dose infeasible. What
#: recovers a cell is the LADDER, not the cuts. Kept because it is cheap and
#: bounded, and recorded here rather than presented as a working mechanism.
MAX_CUTS = 3


@dataclass
class Relocation:
    """One proposal: where the block goes, who yielded, and what stopped it."""
    block: str = ''
    members: Tuple[str, ...] = ()
    direction: Tuple[float, float] = (0.0, 0.0)
    want_mm: float = 0.0
    reach_mm: float = 0.0
    frozen_reach_mm: float = 0.0
    dose_mm: float = 0.0
    shift: Tuple[float, float] = (0.0, 0.0)
    corridor: Tuple[str, ...] = ()
    corridor_mm: float = 0.0
    binding_path: Tuple = ()
    pinned: Tuple[str, ...] = ()
    cuts: int = 0
    solver: str = ''
    moves: Tuple = ()          # writer-shaped, ready for write_placed_output
    applied: bool = False
    refusal: str = ''
    disclosures: Tuple[str, ...] = ()

    def to_dict(self):
        return {'schema': SCHEMA, 'block': self.block,
                'members': list(self.members),
                'direction': [round(v, 4) for v in self.direction],
                'want_mm': round(self.want_mm, 4),
                'reach_mm': round(self.reach_mm, 4),
                'frozen_reach_mm': round(self.frozen_reach_mm, 4),
                'dose_mm': round(self.dose_mm, 4),
                'shift': [round(v, 4) for v in self.shift],
                'corridor': list(self.corridor),
                'corridor_mm': round(self.corridor_mm, 4),
                'binding_path': [list(r) for r in self.binding_path],
                'pinned': list(self.pinned), 'cuts': self.cuts,
                'solver': self.solver, 'moves': [dict(m) for m in self.moves],
                'applied': self.applied, 'refusal': self.refusal,
                'disclosures': list(self.disclosures)}


def relocate_block(state, blocks, block: str, direction: Tuple[float, float], *,
                   want_mm: Optional[float] = None,
                   doses: Sequence[float] = DOSE_LADDER,
                   max_cuts: int = MAX_CUTS,
                   max_corridor_mm: Optional[float] = None,
                   clearance: Optional[float] = None) -> Relocation:
    """Propose a bounded relocation of `block` toward `direction`. Never writes.

    Nothing here judges routability, and that is the point: the objective is
    millimetres and the router is the judge (#459). A proposal that shortens
    displacement and worsens the route is SUPPOSED to be reverted upstream.
    """
    from placement.diagnosis import NO_EFFICACY_CLAIM as DIAG_NO_EFFICACY
    # BOTH: this pass's own gap, and the selector's, because a relocation of a
    # block the diagnosis chose inherits the diagnosis's measured null too.
    rel = Relocation(block=block, direction=tuple(direction),
                     disclosures=(NO_EFFICACY_CLAIM, DIAG_NO_EFFICACY))
    assert_relocatable_state(state)
    units = rigid_units(state, blocks)
    if block not in units.members:
        rel.refusal = 'no_diagnosed_block: %r is not a derived unit' % block
        return rel
    rel.members = units.members[block]
    edges = order_graph(state, units, clearance=clearance)
    bad = identity_violations(edges)
    if bad:
        rel.refusal = ('order_graph_infeasible: %d edge(s) refuse the incumbent '
                       'board' % len(bad))
        return rel

    ry = reach(state, units, edges, block, direction)
    if ry.refusal:
        rel.refusal = ry.refusal
        return rel
    rf = reach(state, pin_all_but(units, block), edges, block, direction)
    rel.reach_mm = ry.reach_mm
    rel.frozen_reach_mm = 0.0 if rf.refusal else rf.reach_mm
    rel.binding_path = ry.binding_chain
    norm = math.hypot(*direction) or 1.0
    u = (direction[0] / norm, direction[1] / norm)
    rel.want_mm = float(want_mm if want_mm is not None else norm)
    ceiling = min(rel.reach_mm, rel.want_mm)
    if ceiling <= 0:
        rel.refusal = ('no_room_at_any_dose: the block cannot travel toward its '
                       'target at all, even with the neighbours yielding')
        return rel

    last_why = ''
    pinned: List[str] = []
    for frac in doses:
        dose = round(ceiling * frac, ROUND_MM)
        if dose <= 0:
            continue
        # Pins are per RUNG, not carried down the ladder. A unit pinned because
        # it could not yield enough for a long dose has no reason to be frozen
        # at a short one -- carrying them forward made the second rung
        # infeasible for the first rung's reason, so every rung after the first
        # reported "two fixed parts alone refuse the shift" and the ladder was a
        # ladder in name only. Termination is already guaranteed by `max_cuts`.
        pinned = []
        for _cut in range(max_cuts + 1):
            moves, solver = min_perturbation(
                state, units, edges, block, (u[0] * dose, u[1] * dose),
                pinned=tuple(pinned))
            if moves is None:
                # `solver` carries the reason on this path.
                last_why = solver
                break
            # QUANTISE ONCE, BEFORE THE CHECK. The emitted board rounds each
            # unit's shift to 4 dp, and an earlier revision checked the
            # UNROUNDED solve and then wrote the rounded one -- so the board
            # that shipped was not the board that was verified. Measured on
            # esp_prog/decap:U1: the exact check passed, and the written board
            # graded two NEW pad conflicts (80 nm and 50 nm) that
            # `grade_pad_legality` reported and the solve had never proposed.
            # Sub-micron, and still a gate that validated a different artifact.
            moves = {u: (round(d[0], 4), round(d[1], 4))
                     for u, d in moves.items()}
            why = exact_refusal(state, units, moves)
            if not why:
                rel.dose_mm = dose
                rel.shift = (round(u[0] * dose, 4), round(u[1] * dose, 4))
                rel.solver = solver
                rel.pinned = tuple(sorted(pinned))
                rel.cuts = len(pinned)
                rel.corridor = tuple(sorted(
                    x for x, d in moves.items()
                    if x != block and d != (0.0, 0.0)))
                rel.corridor_mm = round(sum(
                    math.hypot(*moves[x]) for x in rel.corridor), 4)
                if (max_corridor_mm is not None
                        and rel.corridor_mm > max_corridor_mm + 1e-9):
                    # Legal, minimal for this dose, and still too disturbing.
                    # #554 is "move only diagnosed blocks"; a solve that buys
                    # travel by walking 17 neighbours is the loop, which already
                    # exists. Refuse this rung and try a shorter one.
                    last_why = ('corridor_over_budget: %.2f mm of neighbour '
                                'travel over the %.2f mm allowed, at dose '
                                '%.2f mm' % (rel.corridor_mm, max_corridor_mm,
                                             dose))
                    rel.corridor, rel.corridor_mm = (), 0.0
                    rel.dose_mm, rel.shift, rel.solver = 0.0, (0.0, 0.0), ''
                    rel.pinned, rel.cuts = (), 0
                    break
                # ROUND THE SHIFT, NEVER THE RESULTING COORDINATE. Rounding
                # `x + dx` per member snaps each one independently and SHEARS
                # the block by up to half a step -- `quench._group_offsets`
                # states the same rule for the same reason ("snapping each
                # member independently would shear it by up to a grid step").
                # Measured: rounding the coordinate to 4 dp sheared esp_prog's
                # decap:U1 enough to create two pad conflicts of 80 nm and
                # 50 nm that `grade_pad_legality` reported and the solve had
                # not proposed. Sub-micron, and still a rigid body that was not
                # rigid.
                rel.moves = tuple(
                    {'reference': r,
                     'new_x': state.parts[r].x + moves[uu][0],
                     'new_y': state.parts[r].y + moves[uu][1],
                     'new_rotation': state.parts[r].rot}
                    for uu, d in sorted(moves.items()) if d != (0.0, 0.0)
                    for r in units.members[uu])
                return rel
            # Pin the unit the exact check refused and re-solve around it. A
            # WALK-BACK -- retrying that unit at successively smaller shifts --
            # would find more room and is deliberately not implemented; pinning
            # is conservative, terminates, and every dose it costs is reported
            # rather than hidden.
            last_why = why
            ref = why.split(':')[1] if ':' in why else ''
            unit = units.of_ref.get(ref)
            if unit is None or unit == block or unit in pinned:
                # Nothing left to pin at this dose. Fall to the NEXT rung rather
                # than giving up -- the ladder exists precisely because a shorter
                # move can clear a gate a longer one cannot, and returning here
                # is a bug this pass shipped once: `reach` read 11.02 mm on
                # splitflap while every dose reported "no room", because the
                # first refusal ended the search before the 0.75 rung ran.
                break
            pinned.append(unit)
    # The machine fields must agree with the sentence. They were assigned only
    # on the SUCCESS path, so a refusal whose text said "1 unit(s) pinned"
    # serialised `pinned: [], cuts: 0` -- and the count was the LAST rung's, not
    # the run's, so four rungs each pinning one unit reported as one.
    rel.pinned = tuple(sorted(pinned))
    rel.cuts = len(pinned)
    rel.refusal = ('no_room_at_any_dose: every rung of the dose ladder was '
                   'refused (reach %.2f mm; the last rung pinned %d unit(s) by '
                   'the exact check%s)'
                   % (rel.reach_mm, len(pinned),
                      '; last: ' + last_why if last_why else ''))
    return rel


def format_text(rel: Relocation) -> str:
    """One human-readable block. The binding chain is NAMED, never a number."""
    if rel.refusal:
        return 'relocate %s: %s' % (rel.block, rel.refusal)
    chain = ' -> '.join('%s (%.2f %s)' % (r, g, s)
                        for r, g, s in rel.binding_path[:6]) or '(unbound)'
    return ('relocate %s (%d parts): %.2f mm of %.2f wanted; reach %.2f with '
            'neighbours yielding vs %.2f frozen.\n'
            '  %d part(s) yielded, %.2f mm total. Bound by: %s\n'
            '  solver=%s cuts=%d'
            % (rel.block, len(rel.members), rel.dose_mm, rel.want_mm,
               rel.reach_mm, rel.frozen_reach_mm, len(rel.corridor),
               rel.corridor_mm, chain, rel.solver, rel.cuts))


__all__ = ['SCHEMA', 'AXES', 'NO_EFFICACY_CLAIM', 'SOURCE', 'WALL_LO', 'WALL_HI', 'RelocateError',
           'DOSE_LADDER', 'MAX_CUTS', 'OrderEdge', 'Units', 'Reach',
           'Relocation', 'assert_relocatable_state', 'rigid_units',
           'pin_all_but', 'order_graph', 'identity_violations', 'envelope',
           'reach', 'min_perturbation', 'exact_refusal', 'relocate_block',
           'format_text']

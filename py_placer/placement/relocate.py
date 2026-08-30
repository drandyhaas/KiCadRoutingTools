"""Bounded block relocation as a constraint solve (#554, part 2 of #459).

#459: *"Relocation is a constraint problem, not an optimization problem. Move only
diagnosed blocks; keep the relative order of everything else as a hard constraint;
minimize displacement."* And: *"The constraint graph doubles as the explanation of
why a block sits where it does, which is what an AI director needs to justify a
move."*

The quench's rigid block translate (#538) caps every member at `max_displacement`
from its own seed and freezes everybody else. Measured with the neighbours frozen,
the furthest a block travels TOWARD its connectivity target is a **median 0.1 mm
against a median want of 6.1 mm** over ten corpus boards
(`tests/stress/relocation_reach.py`). A shipped board has no vacancy to translate
into; the only way a block moves is if its neighbours yield.

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
  `s_hi - s_lo >= gap - current_separation`. Freezing the disjunction from the
  incumbent board is a RESTRICTION of the true feasible set, never a relaxation --
  it can refuse a legal answer, never admit an illegal one. What it costs is a
  board whose only room requires a part to slide around a corner; that answer is
  `no_room_at_any_dose`, which is a correct answer to the question #554 asked.

The whole system is difference constraints, so its solutions form a lattice: the
componentwise maximum is itself feasible, which is why `reach` is exactly the
block's own upper envelope and not an optimistic bound.

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
* **Choosing the block.** `placement.diagnosis` does, and its `NO_EFFICACY_CLAIM`
  travels with every relocation proposed here, verbatim.
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
    refusal: str = ''

    def to_dict(self):
        return {'unit': self.unit,
                'lo': {a: round(v, 4) for a, v in sorted(self.lo.items())},
                'hi': {a: round(v, 4) for a, v in sorted(self.hi.items())},
                'binding': {a: [list(r) for r in p]
                            for a, p in sorted(self.binding.items())},
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
    r.reach_mm = max(0.0, round(min(caps), 4)) if caps else math.inf
    return r


__all__ = ['SCHEMA', 'AXES', 'SOURCE', 'WALL_LO', 'WALL_HI', 'RelocateError',
           'OrderEdge', 'Units', 'Reach', 'assert_relocatable_state',
           'rigid_units', 'pin_all_but', 'order_graph', 'identity_violations',
           'envelope', 'reach']

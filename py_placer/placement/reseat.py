"""Re-seat a constrained cluster by assignment, instead of freezing it.

The problem this removes: parts under a proximity rule (a decap that must stay
within N mm of the pin it serves, a part zoned into a region) get **locked**,
because the quench has no proximity term and will otherwise walk one past the
limit every run -- lock one and it moves the next. Locking works, and it
freezes exactly the parts whose re-seating produced most of a run's placement
wins.

The trick that removes the need for both the locks and a proximity cost term:
**the proximity rule becomes the definition of the slot pool.** Slots are
generated around the ANCHOR (the IC, or the zone rect), so every slot satisfies
the constraint by construction; nothing can propose an illegal pose, so nothing
has to price one.

What remains is then an assignment problem -- which member goes in which slot --
and it is exactly additive, so it is Hungarian-legal:

* each member is scored with **every other cluster member's pads overridden to
  an empty list**, which removes them from the airwire model entirely. That is
  why there is no member-member term to make the matrix non-additive, and it
  needs no change to `_net_points`: it reads `overrides.get(ref)`, and `[]` is
  not `None`.
* the constant every row shares (the rest of the board's contribution to the
  same nets) is uniform across the matrix, and Hungarian is invariant to adding
  a constant to every entry.

Three properties that make it safe to run:

* **Identity is always feasible.** Each member's incumbent pose is appended to
  the slot pool, so the assignment can always return "leave everything alone".
* **Acceptance is on the EXACT cluster objective**, re-evaluated with all
  members at their assigned poses -- never on the additive surrogate the
  matrix was built from. A surrogate that lies can therefore waste time, but
  can never ship a worse board.
* **No RNG.** Slots come from a deterministic lattice, ties break on index, and
  the solver is exact. Re-running gives the same answer (#457).

Hungarian returns distinct slot INDICES, not non-overlapping courtyards, so the
assignment is applied one member at a time with a legality re-check and a
per-row walk to the next-best legal slot; what that costs is reported as
`repairs` rather than hidden.

**What it cannot do, stated rather than discovered.** The objective is the
cluster's airwire cost, so a cluster has to carry a net worth scoring. The only
tether source in this repo is `decap_tethers`, and a decoupling cap's two nets
are RAILS -- which the fanout cut correctly drops, because scoring a pose
against a 96-part GND MST measures distance from the middle of the board (and
costs an MST over a hundred points per candidate). Such a cluster reports
`every member net is an ignored or high-fanout rail` and is left alone. That is
the honest answer, not a gap to paper over: where a decap sits is governed by
its pin, and the slot pool already guarantees that. The mechanism earns its
keep on clusters whose members carry SIGNAL nets -- series terminations, filter
networks, a part tethered to a zone -- and it is generic over
`(member, anchor, radius)` so a future tether source needs no changes here.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Set, Tuple

from placement.utility import snap_to_grid

# Slot lattice. Deliberately coarse: this is a RE-SEAT, choosing which side of
# an anchor a part sits on, not a sub-grid polish -- the quench does that after.
DEFAULT_RING_STEP_MM = 0.5
DEFAULT_ROTATIONS = (0.0, 90.0, 180.0, 270.0)
EPS_IMPROVE = 1e-6
# Every position costs one MST rebuild plus one crossing sweep against the rest
# of the board, PER member, so the pool has to be bounded: a wide IC contributes
# a ring set per relevant pin and reaches thousands of positions, at which point
# one cluster takes minutes. This is the cost/quality knob -- raise it when a
# cluster matters enough to pay for it. Measured on ulx3s: ~17ms per slot per
# member, so 32 positions x 4 rotations is a second or two per cluster.
MAX_POSITIONS = 32


@dataclass
class Cluster:
    """Members that must stay near `anchor`, and the region that means."""
    name: str
    anchor: str
    members: Tuple[str, ...]
    radius_mm: float
    rect: Optional[Tuple[float, float, float, float]] = None   # x, y, w, h

    def __post_init__(self):
        self.members = tuple(sorted(self.members))


@dataclass
class ReseatResult:
    cluster: str
    moved: Tuple[str, ...] = ()
    before: float = 0.0
    after: float = 0.0
    slots: int = 0
    repairs: int = 0          # members bumped off their assigned slot
    accepted: bool = False
    reason: str = ''

    def to_dict(self):
        return {'cluster': self.cluster, 'moved': list(self.moved),
                'before': round(self.before, 4), 'after': round(self.after, 4),
                'gain': round(self.before - self.after, 4),
                'slots': self.slots, 'repairs': self.repairs,
                'accepted': self.accepted, 'reason': self.reason}


def clusters_from_tethers(pcb_data, state, radius_mm,
                          movable: Optional[Set[str]] = None
                          ) -> List[Cluster]:
    """One cluster per anchor that has tethered members, from `decap_tethers`.

    Generic over (member, anchor, radius) rather than special-cased to decaps:
    `decap_tethers` is simply the tether source this repo already has, and it
    already refuses the traps (a cap near an IC it shares no net with; a
    collinear 1xN row masquerading as an IC).
    """
    from .groups import decap_tethers
    if movable is None:
        movable = {r for r, p in state.parts.items() if not p.locked}
    out: List[Cluster] = []
    for anchor, tethered in sorted(decap_tethers(pcb_data, movable).items()):
        members = tuple(r for r, _d in tethered if r in state.parts)
        if not members or anchor not in state.parts:
            continue
        out.append(Cluster(name=f"tether:{anchor}", anchor=anchor,
                           members=members, radius_mm=radius_mm))
    return out


def _anchor_pin_points(state, cluster) -> List[Tuple[float, float]]:
    """Where a member could usefully sit: the anchor's pads on nets the members
    share with it.

    Ringing the anchor's CENTRE would send a decap to the middle of the die.
    The rule these parts live under is proximity to a PIN, so the slot pool is
    built around the pins that matter.
    """
    anchor = state.parts[cluster.anchor]
    member_nets: Set[int] = set()
    for m in cluster.members:
        member_nets.update(state.parts[m].nets)
    pts = [(gx, gy) for gx, gy, nid in anchor.pad_globals()
           if nid in member_nets]
    return pts or [(anchor.x, anchor.y)]


def slot_pool(state, cluster, ring_step=DEFAULT_RING_STEP_MM,
              rotations=DEFAULT_ROTATIONS, grid_step=0.1,
              max_positions=MAX_POSITIONS
              ) -> List[Tuple[float, float, float]]:
    """Candidate (x, y, rot) poses. Every one satisfies the proximity rule.

    Generated on rings around the anchor's relevant pins out to `radius_mm`, so
    membership in the pool IS the constraint -- there is nothing to gate and no
    proximity term to weight. Snapped to `grid_step` and de-duplicated in a
    deterministic order.
    """
    seen = set()
    out: List[Tuple[float, float, float]] = []

    # #708: snap the OFFSET FROM THE ANCHOR, not the absolute position, and
    # through the shared primitive rather than a second spelling of it.
    #
    # The phase to preserve is the ANCHOR's, not the pad's: these rings are
    # generated around `_anchor_pin_points`, and a pad sits at its footprint
    # origin plus a package pitch (0.5mm, 0.65mm) that is on no board lattice.
    # The anchor's own origin is what the designer placed, so an offset that is
    # a whole number of raster units from it lands a member on the same pitch
    # the anchor sits on. Snapping the absolute point instead quantized to a
    # lattice through board origin and threw that away.
    #
    # The RASTER, not the board's inferred lattice: like the fanout cap repair,
    # this pool is a proximity constraint satisfied by construction, and a
    # coarse lattice would both thin it and inflate the `snap` shrink below.
    anchor_part = state.parts[cluster.anchor]
    ax, ay = anchor_part.x, anchor_part.y

    def add(x, y, rot):
        sx = ax + snap_to_grid(x - ax, grid_step)
        sy = ay + snap_to_grid(y - ay, grid_step)
        key = (round(sx / grid_step), round(sy / grid_step), round(rot, 3))
        if key not in seen:
            seen.add(key)
            out.append((sx, sy, rot))

    # Snapping to the grid moves a point by up to half a cell on each axis, so
    # a ring generated AT the radius lands outside it. Shrink by the snap
    # diagonal, or the "every slot satisfies the rule by construction" claim is
    # simply false at the outer ring -- which is exactly where a re-seat wants
    # to look, and exactly where a proximity clause is about to be graded.
    snap = grid_step * math.sqrt(2.0) / 2.0
    r_max = max(grid_step, cluster.radius_mm - snap)
    n_rings = max(1, int(r_max / ring_step))
    for px, py in _anchor_pin_points(state, cluster):
        for k in range(1, n_rings + 1):
            r = min(r_max, k * ring_step)
            n_pts = max(4, int(round(2 * math.pi * r / ring_step)))
            for i in range(n_pts):
                th = 2 * math.pi * i / n_pts
                add(px + r * math.cos(th), py + r * math.sin(th), 0.0)
    # An anchor contributes one ring set PER relevant pin, so a wide IC can
    # generate thousands of positions and each one costs a full MST rebuild per
    # member. Keep the nearest `max_positions` to the members' own centroid --
    # deterministic (distance, then coordinate), and the near ones are the ones
    # a proximity-constrained part can actually use.
    if len(out) > max_positions:
        cx = sum(state.parts[m].x for m in cluster.members) / len(
            cluster.members)
        cy = sum(state.parts[m].y for m in cluster.members) / len(
            cluster.members)
        out.sort(key=lambda p: (math.hypot(p[0] - cx, p[1] - cy), p[0], p[1]))
        out = out[:max_positions]
    # Rotations are applied to every position, after the positions are settled,
    # so the lattice is the same set whatever the rotation list is.
    out = [(x, y, rot) for (x, y, _r) in out for rot in rotations]
    # Identity last but always present: the assignment must be able to return
    # "leave it alone", and a member's own pose need not be on the lattice.
    for m in cluster.members:
        p = state.parts[m]
        out.append((p.x, p.y, p.rot))
    return out


def _member_costs(state, cluster, slots, member, others_blank, other_aw,
                  involved):
    """One row of the matrix: `member` at each slot, everyone else removed.

    Returns (costs, legal) where an illegal slot carries `inf` -- kept in the
    matrix rather than dropped so every row is the same length and the column
    indices stay comparable.
    """
    part = state.parts[member]
    exclude = set(cluster.members) - {member}
    costs: List[float] = []
    for (x, y, rot) in slots:
        if rot not in part.bounds_by_rot:
            costs.append(float('inf'))
            continue
        if not state.candidate_valid(member, x, y, rot, exclude=exclude):
            costs.append(float('inf'))
            continue
        ov = dict(others_blank)
        ov[member] = part.pad_globals(x, y, rot)
        subset = {n: state._build_net_airwires(n, overrides=ov)
                  for n in involved}
        net_cost, _ = state.nets_cost(subset, other_aw)
        costs.append(net_cost + state.part_geometry_cost(
            member, x, y, rot, exclude=exclude))
    return costs


def _assign(cost: List[List[float]]) -> List[Tuple[int, int]]:
    """Min-cost bipartite matching; scipy when present, exact otherwise.

    KiCad's bundled Python has no scipy, and this runs on the same code paths,
    so the fallback is not optional. Same shape as
    `diff_pair_multipoint._min_cost_matching`; kept separate rather than
    imported because that module pulls in the whole diff-pair stack.
    """
    rows = len(cost)
    cols = len(cost[0]) if rows else 0
    if not rows or not cols:
        return []
    try:
        import numpy as np
        from scipy.optimize import linear_sum_assignment
        arr = np.array(cost, dtype=float)
        # Hungarian rejects an all-inf row; a huge finite stand-in keeps the
        # solver happy and still loses to any legal slot.
        big = np.nanmax(arr[np.isfinite(arr)]) if np.isfinite(arr).any() else 1.0
        arr[~np.isfinite(arr)] = big * 1e6 + 1e9
        ri, ci = linear_sum_assignment(arr)
        return list(zip(ri.tolist(), ci.tolist()))
    except ImportError:
        pass
    # Greedy, deterministic: cheapest legal (row, col) first. Suboptimal in
    # principle; the acceptance test below is on the exact objective, so a
    # worse assignment costs a wasted evaluation and never a worse board.
    used_r, used_c, pairs = set(), set(), []
    order = sorted(((cost[r][c], r, c) for r in range(rows)
                    for c in range(cols) if math.isfinite(cost[r][c])))
    for _v, r, c in order:
        if r in used_r or c in used_c:
            continue
        used_r.add(r)
        used_c.add(c)
        pairs.append((r, c))
    return pairs


def _rects_overlap(a, b) -> bool:
    return not (a[2] <= b[0] or b[2] <= a[0] or a[3] <= b[1] or b[3] <= a[1])


def _clashes_with_seated(state, ref, pose, poses) -> bool:
    """Does `ref` at `pose` overlap a cluster member already seated?

    Needed because the seated members are not in `state.parts` at their new
    poses yet, so `candidate_valid` cannot see them. Same courtyard/far-side
    pair the legality gate uses, so "legal" means the same thing here as there.
    """
    if not poses:
        return False
    part = state.parts[ref]
    a_ct, a_tht = part.rects(*pose)
    for other, opose in poses.items():
        op = state.parts[other]
        b_ct, b_tht = op.rects(*opose)
        if part.side == op.side and _rects_overlap(a_ct, b_ct):
            return True
        # A through-hole part obstructs the FAR side too, so a same-cluster
        # pair on opposite sides can still clash through the board.
        if a_tht is not None and _rects_overlap(a_tht, b_ct):
            return True
        if b_tht is not None and _rects_overlap(a_ct, b_tht):
            return True
    return False


def scored_nets(state, cluster, ignore_net_ids=None,
                max_fanout=None) -> Set[int]:
    """The cluster's nets, minus the ones that say nothing about a seat.

    Same cut as `block_displacements` and `net_affinity`, and it matters more
    here than anywhere: these clusters are usually tethered to RAILS. GND owns
    96 of ulx3s's parts, so scoring a decap's pose against GND's MST both costs
    an MST over a hundred points per candidate slot AND measures "distance from
    the middle of the board" -- the exact fiction the fanout cut exists for.
    Filtered, a cluster evaluates in a fraction of a second against nets whose
    centroid means something.
    """
    from .routability import DISPLACEMENT_MAX_FANOUT
    if max_fanout is None:
        max_fanout = DISPLACEMENT_MAX_FANOUT
    ignored = set(ignore_net_ids or ())
    if max_fanout:
        ignored.update(nid for nid, refs in state.net_refs.items()
                       if len(refs) > max_fanout)
    out: Set[int] = set()
    for m in cluster.members:
        if m in state.parts:
            out.update(n for n in state.parts[m].nets if n not in ignored)
    return out


def cluster_cost(state, cluster, poses=None, involved=None) -> float:
    """The EXACT objective for this cluster: every member at its real pose.

    This is what acceptance is measured on. The matrix's additive surrogate is
    only a way to CHOOSE a candidate; it never decides whether one ships.
    """
    if involved is None:
        involved = scored_nets(state, cluster)
    if not involved:
        return 0.0
    other_aw = state.airwires_excluding(involved)
    ov = {}
    for m in cluster.members:
        p = state.parts[m]
        x, y, rot = (poses or {}).get(m, (p.x, p.y, p.rot))
        ov[m] = p.pad_globals(x, y, rot)
    subset = {n: state._build_net_airwires(n, overrides=ov) for n in involved}
    net_cost, _ = state.nets_cost(subset, other_aw)
    geo = 0.0
    member_set = set(cluster.members)
    for m in cluster.members:
        p = state.parts[m]
        x, y, rot = (poses or {}).get(m, (p.x, p.y, p.rot))
        # Intra-cluster pairs are excluded on BOTH sides, so each is counted
        # once by neither -- the term is comparable across candidates, which is
        # all it has to be.
        geo += state.part_geometry_cost(m, x, y, rot,
                                        exclude=member_set - {m})
    return net_cost + geo


def reseat_cluster(state, cluster, *, ring_step=DEFAULT_RING_STEP_MM,
                   rotations=DEFAULT_ROTATIONS, grid_step=0.1,
                   max_positions=MAX_POSITIONS, ignore_net_ids=None,
                   max_fanout=None, apply=True) -> ReseatResult:
    """Re-assign one cluster's members to slots around their anchor.

    Reports rather than acts when there is nothing to decide. In particular a
    cluster whose members carry ONLY ignored nets -- a decap pair on rails the
    state was built with `ignore_net_ids` for, which is the normal
    configuration -- has no scored airwire to improve, and says so instead of
    shuffling parts on a geometry term that was never the reason they were
    grouped.
    """
    members = [m for m in cluster.members if m in state.parts]
    if not members:
        return ReseatResult(cluster=cluster.name, reason='no members')

    slots = slot_pool(state, cluster, ring_step, rotations, grid_step,
                      max_positions)
    if not slots:
        return ReseatResult(cluster=cluster.name, reason='no slots')

    involved = scored_nets(state, cluster, ignore_net_ids, max_fanout)
    if not involved:
        return ReseatResult(
            cluster=cluster.name, slots=len(slots),
            reason='every member net is an ignored or high-fanout rail -- '
                   'there is no scored airwire here to improve')
    other_aw = state.airwires_excluding(involved)
    # The additivity trick: everyone else contributes NO pads.
    blank = {m: [] for m in members}

    matrix = []
    for m in members:
        others = dict(blank)
        others.pop(m, None)
        matrix.append(_member_costs(state, cluster, slots, m, others,
                                    other_aw, involved))

    before = cluster_cost(state, cluster, involved=involved)
    pairs = _assign(matrix)
    if not pairs:
        return ReseatResult(cluster=cluster.name, before=before, after=before,
                            slots=len(slots), reason='no assignment')

    # Distinct slot indices are not non-overlapping courtyards. Seat one member
    # at a time against the poses already seated, and walk a bumped member down
    # its own row to the next-best legal slot.
    member_set = set(cluster.members)
    poses: Dict[str, Tuple[float, float, float]] = {}
    repairs = 0
    for r, c in sorted(pairs):
        m = members[r]
        row = matrix[r]
        part = state.parts[m]
        order = [c] + sorted((i for i in range(len(slots)) if i != c),
                             key=lambda i: (row[i], i))
        placed = False
        for idx in order:
            if not math.isfinite(row[idx]):
                continue
            x, y, rot = slots[idx]
            # Every cluster member is excluded from candidate_valid, because
            # `state.parts` still holds their OLD poses -- nothing has been
            # applied yet, so checking against them would test the wrong
            # geometry both ways. The members already seated are then checked
            # explicitly, at the poses they are actually going to.
            if not state.candidate_valid(m, x, y, rot, exclude=member_set):
                continue
            if _clashes_with_seated(state, m, (x, y, rot), poses):
                continue
            poses[m] = (x, y, rot)
            placed = True
            if idx != c:
                repairs += 1
            break
        if not placed:
            poses[m] = (part.x, part.y, part.rot)   # identity: always feasible
            repairs += 1

    after = cluster_cost(state, cluster, poses, involved=involved)
    moved = tuple(m for m in members
                  if abs(poses[m][0] - state.parts[m].x) > 1e-6
                  or abs(poses[m][1] - state.parts[m].y) > 1e-6
                  or abs(poses[m][2] - state.parts[m].rot) > 1e-6)
    res = ReseatResult(cluster=cluster.name, moved=moved, before=before,
                       after=after, slots=len(slots), repairs=repairs)
    if after < before - EPS_IMPROVE and moved:
        # `after` above is a PREDICTION, and it is blind in one direction: it
        # overrides the members' pads on the SCORED nets, but `other_aw` is
        # built from the live board, so the move's effect on a member's
        # UNSCORED nets is not in it. A decap on GND and one signal has its GND
        # airwire priced at the pose it is leaving. Measured on splitflap's
        # tether:B0 (C60, nets 1 and 278, only 278 scored): predicted 1214.28,
        # actual 1274.28 -- six crossings, 60.0 of cost, all of it invisible.
        # So seat it and ASK, rather than predict and hope. The contract this
        # restores is the one `test_reseat` states: acceptance is on the exact
        # objective, re-evaluated with all members in place.
        prior = {m: (state.parts[m].x, state.parts[m].y, state.parts[m].rot)
                 for m in moved}
        for m in moved:
            state.apply_move(m, *poses[m])
        seated = cluster_cost(state, cluster, involved=involved)
        if seated < before - EPS_IMPROVE:
            res.accepted = True
            res.after = seated
            if not apply:
                for m in moved:
                    state.apply_move(m, *prior[m])
        else:
            for m in moved:
                state.apply_move(m, *prior[m])
            res.after = seated
            res.reason = ('the seated cost is worse than the prediction '
                          '(%.4f vs %.4f): the members carry nets this '
                          'cluster does not score' % (seated, after))
    else:
        res.reason = ('no improvement on the exact objective'
                      if moved else 'assignment returned the incumbent')
    return res


def reseat(state, clusters: Sequence[Cluster], **kw) -> List[ReseatResult]:
    """Re-seat every cluster, in a deterministic order. Returns one row each."""
    return [reseat_cluster(state, c, **kw)
            for c in sorted(clusters, key=lambda c: c.name)]

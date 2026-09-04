"""What tells you the FLOORPLAN is wrong, rather than the routing (#549, #407).

Discussion #407 names the unsolved problem exactly: *knowing when to stop routing
and go move something*. The two scars in it are both floorplan errors that a
router can only grind against --

    a magnetics block sitting ~80mm from BOTH its own endpoints, so 8 of its
    traces cut straight through the camera-bus corridor;

    ~22 residual nets that would not route whatever knob was turned, because
    the fix was re-floorplanning a quadrant.

Both were visible from move zero to anyone looking at the right number. The
signals below are those numbers. They are deliberately NOT part of the intent
rule set: an intent violation says "this is not the floorplan you declared", a
health signal says "this floorplan will fight the router whatever you declared".

Kept in its own module, not inside `floorplan.py`, because #459's mover
selection wants the displacement metric without the intent machinery.

HONEST COMPUTABILITY -- stated because the temptation is to ship all four:

  block displacement   computable today, from geometry alone. This IS #459's
                       "connectivity-centroid displacement".
  bus crossings        computable pre-route, but the corridor is a MODEL (a
                       straight rectangle between two pad clusters), not the
                       route. Screening signal, not a verdict.
  convergence          needs a declared list of critical net classes. Placement
                       has no net-class notion, and "critical" is design intent
                       rather than a fact in the file, so it comes from the
                       intent or it does not run. No silent heuristic.
  blocked-cell share   NOT computable pre-route at all. It needs #409's blocker
                       JSON, which only exists after a routing attempt.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


# A net owned by more parts than this reaches everywhere by design -- GND, a
# supply rail -- so it says nothing about where any one block belongs. Measured
# on ulx3s: 329 nets, MEDIAN 2 owning parts, and exactly two above 20 (GND at
# 96, +3V3 at 45). Without this cut those two dominate every block's foreign-pad
# set, the "net centroid" collapses onto the board centroid, and the metric
# degenerates into "distance from the middle of the board" -- which is not a
# floorplan signal at all. Same reasoning as the plane-net exclusion the
# optimizer already wants from `--ignore-nets`.
DISPLACEMENT_MAX_FANOUT = 20


@dataclass(frozen=True)
class BlockDisplacement:
    """One block, and how far it sits from the parts it actually connects to."""
    block: str
    members: Tuple[str, ...]
    centroid: Tuple[float, float]
    net_centroid: Tuple[float, float]
    distance_mm: float
    foreign_pads: int
    nets: int
    nets_ignored: int = 0

    def to_dict(self):
        return {'block': self.block, 'members': len(self.members),
                'centroid': [round(v, 3) for v in self.centroid],
                'net_centroid': [round(v, 3) for v in self.net_centroid],
                'distance_mm': round(self.distance_mm, 4),
                'foreign_pads': self.foreign_pads, 'nets': self.nets,
                'nets_ignored': self.nets_ignored}


def block_displacements(state, blocks: Dict[str, Sequence[str]],
                        ignore_net_ids: Optional[Sequence[int]] = None,
                        max_fanout: int = DISPLACEMENT_MAX_FANOUT
                        ) -> List[BlockDisplacement]:
    """How far each block sits from the centroid of what it connects to.

    For a block B: the centroid of B's own connected pads, against the centroid
    of every pad on B's nets that belongs to a part OUTSIDE B. A large distance
    means the block was placed away from everything it talks to -- the 80mm
    magnetics case from #407, and the one failure mode a 3mm nudge provably
    cannot fix.

    Power and ground are excluded, by `ignore_net_ids` (pass the plane-net set,
    as `--ignore-nets` does elsewhere) and by `max_fanout` as a backstop. See
    DISPLACEMENT_MAX_FANOUT: without the cut this measures distance from the
    middle of the board.

    A block left with no foreign pads is OMITTED rather than reported as 0.0.
    "Connects to nothing outside itself" and "sits exactly on its partners" are
    different facts, and a zero reads as the second.
    """
    ignored = set(ignore_net_ids or ())
    if max_fanout:
        ignored.update(nid for nid, refs in state.net_refs.items()
                       if len(refs) > max_fanout)

    out: List[BlockDisplacement] = []
    for name in sorted(blocks):
        members = [r for r in sorted(blocks[name]) if r in state.parts]
        if not members:
            continue
        member_set = set(members)

        own: List[Tuple[float, float]] = []
        nets: set = set()
        dropped = 0
        for ref in members:
            for gx, gy, nid in state.parts[ref].pad_globals():
                if nid in ignored:
                    dropped += 1
                    continue
                own.append((gx, gy))
                nets.add(nid)
        if not own:
            continue

        foreign: List[Tuple[float, float]] = []
        for nid in sorted(nets):
            for ref in state.net_refs.get(nid, ()):
                if ref in member_set or ref not in state.parts:
                    continue
                for gx, gy, pn in state.parts[ref].pad_globals():
                    if pn == nid:
                        foreign.append((gx, gy))
        if not foreign:
            continue

        cx = sum(p[0] for p in own) / len(own)
        cy = sum(p[1] for p in own) / len(own)
        fx = sum(p[0] for p in foreign) / len(foreign)
        fy = sum(p[1] for p in foreign) / len(foreign)
        out.append(BlockDisplacement(
            block=name, members=tuple(members), centroid=(cx, cy),
            net_centroid=(fx, fy), distance_mm=math.hypot(fx - cx, fy - cy),
            foreign_pads=len(foreign), nets=len(nets), nets_ignored=dropped))
    return out


@dataclass(frozen=True)
class Corridor:
    """A straight rectangular channel between a bus's two pad clusters.

    A MODEL of where the bus wants to run, not where it will. Real routes bend
    around obstacles; this is a screening signal for "something large crosses
    the path this bus obviously wants", which is the `CSI-2 x MDI-Ethernet`
    smell from #407.
    """
    name: str
    net_ids: Tuple[int, ...]
    a: Tuple[float, float]
    b: Tuple[float, float]
    width_mm: float

    @property
    def length_mm(self) -> float:
        return math.hypot(self.b[0] - self.a[0], self.b[1] - self.a[1])

    def side_edges(self) -> List[Tuple[float, float, float, float]]:
        """The two long sides, as segments.

        This is what makes the whole thing cheap: "a foreign net pierces this
        corridor" becomes a crossing count against two segments, so it reuses
        `quench._count_crossings_np` instead of introducing a polygon clipper.
        """
        dx, dy = self.b[0] - self.a[0], self.b[1] - self.a[1]
        n = math.hypot(dx, dy)
        if n < 1e-9:
            return []
        ux, uy = -dy / n, dx / n            # unit normal
        h = self.width_mm / 2.0
        return [(self.a[0] + ux * h, self.a[1] + uy * h,
                 self.b[0] + ux * h, self.b[1] + uy * h),
                (self.a[0] - ux * h, self.a[1] - uy * h,
                 self.b[0] - ux * h, self.b[1] - uy * h)]

    def to_dict(self):
        return {'name': self.name, 'nets': len(self.net_ids),
                'a': [round(v, 3) for v in self.a],
                'b': [round(v, 3) for v in self.b],
                'width_mm': self.width_mm, 'length_mm': round(self.length_mm, 3)}


def _cluster_ends(state, net_ids: Sequence[int]):
    """Both endpoint clusters of a bus, as centroids.

    Each net contributes its two extreme pads (the ones furthest apart), and the
    clusters are the centroids of the two ends. Crude on purpose: a corridor is
    a screening model.
    """
    starts, ends = [], []
    for nid in net_ids:
        pts = []
        for ref in state.net_refs.get(nid, ()):
            if ref not in state.parts:
                continue
            pts.extend((gx, gy) for gx, gy, pn in state.parts[ref].pad_globals()
                       if pn == nid)
        if len(pts) < 2:
            continue
        best, pa, pb = -1.0, None, None
        for i, p in enumerate(pts):
            for q in pts[i + 1:]:
                d = (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2
                if d > best:
                    best, pa, pb = d, p, q
        if pa[0] > pb[0] or (pa[0] == pb[0] and pa[1] > pb[1]):
            pa, pb = pb, pa            # orient deterministically (#457)
        starts.append(pa)
        ends.append(pb)
    if not starts:
        return None, None
    return ((sum(p[0] for p in starts) / len(starts),
             sum(p[1] for p in starts) / len(starts)),
            (sum(p[0] for p in ends) / len(ends),
             sum(p[1] for p in ends) / len(ends)))


def corridors_from_intent(state, pcb_data, specs: Sequence[Dict]
                          ) -> List[Corridor]:
    """Corridors for the bus specs an intent declares.

    Each spec is {'name', 'nets': [glob...], 'width_mm'}; net globs use the same
    dialect as `route.py --nets`, via the shared matcher, so a pattern means
    here what it means there.
    """
    from net_queries import matches_net_filter
    out: List[Corridor] = []
    for spec in specs:
        pats = list(spec.get('nets') or ())
        if not pats:
            continue
        ids = tuple(sorted(
            nid for nid, n in pcb_data.nets.items()
            if nid > 0 and n.name and matches_net_filter(n.name, pats)))
        if len(ids) < 2:
            continue
        a, b = _cluster_ends(state, ids)
        if a is None:
            continue
        out.append(Corridor(name=spec.get('name') or ','.join(pats),
                            net_ids=ids, a=a, b=b,
                            width_mm=float(spec.get('width_mm', 8.0))))
    return out


def foreign_crossings(state, corridor: Corridor,
                      ignore_net_ids: Optional[Sequence[int]] = None,
                      max_fanout: int = DISPLACEMENT_MAX_FANOUT
                      ) -> Tuple[int, List[int]]:
    """How many airwires of OTHER nets cross into this corridor, and whose.

    Counted against the corridor's two long sides with the quench's own crossing
    kernel, so the geometry is the one the placement cost already uses.

    Power and ground are excluded for the same reason as in
    `block_displacements`, and it matters more here: GND on ulx3s owns 96 parts,
    so its MST sprays airwires over the whole board and crosses EVERY corridor.
    Measured on the sdram corridor, the unfiltered top three offenders were GND,
    +5V and +3V3 -- a fiction, since those rails route on a plane rather than as
    traces through the channel. Left in, the signal reports "every bus is
    crossed by power" on every board and distinguishes nothing.
    """
    from .quench import _aw_array, _count_crossings_np
    sides = corridor.side_edges()
    if not sides:
        return 0, []
    own = set(corridor.net_ids)
    skip = set(ignore_net_ids or ())
    if max_fanout:
        skip.update(nid for nid, refs in state.net_refs.items()
                    if len(refs) > max_fanout)
    wall = _aw_array([(x1, y1, x2, y2, -1.0) for x1, y1, x2, y2 in sides])

    total = 0
    guilty: Dict[int, int] = {}
    for nid, aws in sorted(state.net_airwires.items()):
        if nid in own or nid in skip or not aws:
            continue
        n, _w = _count_crossings_np(_aw_array(aws), wall)
        if n:
            guilty[nid] = n
            total += n
    return total, [nid for nid, _n in sorted(guilty.items(),
                                             key=lambda kv: (-kv[1], kv[0]))]


# Below this share of a bus's pads sitting near either endpoint, the corridor
# is a phantom: `_cluster_ends` averaged clusters that are not there. Measured
# on the corpus -- a real point-to-point bus scores 0.8-1.0 (ulx3s SDRAM_A* is
# 1.0), while N repeated LOCAL sub-blocks under one name score ~0.0 and still
# produce a confident-looking rectangle (splitflap OUT_* : 24 nets, cover 0.0).
CORRIDOR_MIN_COVER = 0.5


def corridor_cover(state, corridor: Corridor) -> float:
    """Fraction of the bus's own pads that sit at one of the corridor's ends.

    The honesty check on a declared corridor. `corridors_from_intent` will build
    a rectangle for ANY set of nets, including one whose "bus" is really six
    identical motor channels scattered over the board -- their endpoint
    centroids average to the middle of nothing, and every signal computed
    against that rectangle is measuring a fiction. This is what lets a consumer
    say so instead of grading it.

    "At an end" means within a quarter of the corridor's length of either
    endpoint, which is a shape test rather than a tuned distance: it scales with
    the corridor and is unit-free.
    """
    n = corridor.length_mm
    if n < 1e-9:
        return 0.0
    tol = n / 4.0
    near = total = 0
    for nid in corridor.net_ids:
        for ref in state.net_refs.get(nid, ()):
            if ref not in state.parts:
                continue
            for gx, gy, pn in state.parts[ref].pad_globals():
                if pn != nid:
                    continue
                total += 1
                if (math.hypot(gx - corridor.a[0], gy - corridor.a[1]) <= tol
                        or math.hypot(gx - corridor.b[0],
                                      gy - corridor.b[1]) <= tol):
                    near += 1
    return near / total if total else 0.0


def corridor_cut_mm(state, corridor: Corridor,
                    ignore_net_ids: Optional[Sequence[int]] = None,
                    max_fanout: int = DISPLACEMENT_MAX_FANOUT) -> float:
    """Total LENGTH foreign airwires spend inside this corridor.

    The same event `foreign_crossings` counts, measured instead of tallied. It
    is the better number of the two to READ, for a reason the count cannot
    express: a wire crossing a width-w lane at angle theta runs w/sin(theta)
    under the bus, so a shallow diagonal does several times the damage of a
    square crossing while both score 1. On two layers that length is the
    geometric lower bound on reference-plane copper the crossing removes, which
    is the quantity `check_impedance.py`'s void-run counter later grades.

    It is NOT a useful thing to minimise by nudging parts, and that is measured,
    not assumed -- see `docs/placement-optimization.md`. A wire that fully
    traverses the lane pays w/sin(theta) wherever along the lane it crosses, so
    the chord is POSITION-INVARIANT and a small move slides the crossing point
    without changing the toll.
    """
    from .quench import _CorridorBox, _aw_array, _corridor_cut_np
    import numpy as np
    n = corridor.length_mm
    if n < 1e-9 or corridor.width_mm <= 0.0:
        return 0.0
    size = max(max(state.net_airwires, default=-1),
               max(corridor.net_ids, default=-1),
               *(ignore_net_ids or (-1,))) + 1
    skip = np.zeros(max(size, 1), dtype=bool)
    for nid in (ignore_net_ids or ()):
        if 0 <= nid < len(skip):
            skip[nid] = True
    if max_fanout:
        for nid, refs in state.net_refs.items():
            if len(refs) > max_fanout and 0 <= nid < len(skip):
                skip[nid] = True
    for nid in corridor.net_ids:
        if 0 <= nid < len(skip):
            skip[nid] = True
    box = _CorridorBox(ax=corridor.a[0], ay=corridor.a[1],
                       ux=(corridor.b[0] - corridor.a[0]) / n,
                       uy=(corridor.b[1] - corridor.a[1]) / n,
                       length=n, half_w=corridor.width_mm / 2.0, skip=skip)
    aw = [a for lst in state.net_airwires.values() for a in lst]
    return _corridor_cut_np(_aw_array(aw), [box])


@dataclass(frozen=True)
class PartAffinity:
    """One part that carries most of a net its own block sits away from."""
    ref: str
    block: str
    net: str
    net_id: int
    share: float             # fraction of the net's MST length incident on ref
    length_mm: float         # that length
    reach_norm: float        # length_mm / board diagonal -- board-comparable
    pierced: Tuple[str, ...]  # declared corridors this part's edges cross
    recoverable_mm: float    # MST length freed by moving ref to its net centroid
    recoverable_norm: float
    locked: bool

    def to_dict(self):
        return {'ref': self.ref, 'block': self.block, 'net': self.net,
                'share': round(self.share, 4),
                'length_mm': round(self.length_mm, 3),
                'reach_norm': round(self.reach_norm, 4),
                'pierced': list(self.pierced),
                'recoverable_mm': round(self.recoverable_mm, 3),
                'recoverable_norm': round(self.recoverable_norm, 4),
                'locked': self.locked}


# A part "dominates" a net when it carries more than half of it. Necessary but
# NOT sufficient: a part sitting in the middle of a net's span is incident on
# the edges either side of it, so it scores 1.0 while being exactly where it
# belongs. Dominance says "this part carries the net"; it takes the recoverable
# test below to say "and it should not".
AFFINITY_DOMINANT_SHARE = 0.5

# Moving the part onto the centroid of what it talks to must free at least this
# fraction of what it carries, or the length is topology rather than
# misplacement. A hub frees ~0; a part zoned into the wrong block frees nearly
# all of it (measured: 31.0 of 31.0 mm on the fixture, 21.7 of 21.7 on a real
# board).
AFFINITY_MIN_RECOVERABLE_FRACTION = 0.10


def _incident_edges(state, ref: str, net_id: int):
    """The net's MST edges that touch one of `ref`'s pads, each once."""
    pts = {(round(gx, 6), round(gy, 6))
           for gx, gy, n in state.parts[ref].pad_globals() if n == net_id}
    if not pts:
        return []
    out = []
    for aw in state.net_airwires.get(net_id, ()):
        x1, y1, x2, y2 = aw[0], aw[1], aw[2], aw[3]
        if ((round(x1, 6), round(y1, 6)) in pts
                or (round(x2, 6), round(y2, 6)) in pts):
            out.append(aw)
    return out


def _mst_length(airwires) -> float:
    return sum(math.hypot(a[2] - a[0], a[3] - a[1]) for a in airwires)


def net_affinity(state, pcb_data, blocks: Dict[str, Sequence[str]],
                 corridors: Optional[Sequence[Corridor]] = None,
                 ignore_net_ids: Optional[Sequence[int]] = None,
                 max_fanout: int = DISPLACEMENT_MAX_FANOUT,
                 exempt_net_ids: Optional[Sequence[int]] = None
                 ) -> List[PartAffinity]:
    """Parts that carry a net their own block sits away from.

    `block_displacements` asks the block-level question: how far is this block
    from what it talks to. This is the per-PART inverse, and it catches a
    failure the block metric averages away -- ONE member seated by block
    membership while a net's mass sits elsewhere becomes the sole carrier of
    that net's traverse. Measured on a real board: a series resistor zoned into
    a far-edge block owned 85.7% of a critical bus net's routed length, forcing
    ten drop-vias and eight reference-plane voids. The intent's own zone caused
    it, and four runs went by without anything measuring it.

    Reported per (part, net), ranked, advisory. The entry conditions are the
    false-positive killers, and none of them is a tuned constant:

      * the same fanout / ignore-net cut `block_displacements` uses, so a rail
        or a ground net -- which reaches everywhere by design -- never appears;
      * the part must belong to a block THAT HAS A ZONE. Without a declared
        seat there is no membership to blame, and blaming a part for being
        where nothing asked it to be is noise;
      * `share > AFFINITY_DOMINANT_SHARE`, a definition rather than a
        threshold: below half, no single part carries the net;
      * nets whose every owner sits inside the part's own block are skipped --
        there is no membership conflict to report.

    Locked parts are reported WITH a flag, not suppressed: "this cannot move"
    is triage, not absence.

    `recoverable_mm` is a mechanical counterfactual, not a guess -- the net's
    MST is rebuilt with the part translated onto the centroid of the pads it
    talks to, using the same override primitive the quench scores moves with.
    It is an UPPER BOUND that ignores legality: nothing checks that the part
    may actually sit there.
    """
    ignored = set(ignore_net_ids or ())
    if max_fanout:
        ignored.update(nid for nid, refs in state.net_refs.items()
                       if len(refs) > max_fanout)
    ignored.update(exempt_net_ids or ())

    block_of: Dict[str, str] = {}
    for name in sorted(blocks):
        for ref in blocks[name]:
            block_of.setdefault(ref, name)

    diag = _board_diagonal(state, pcb_data)
    out: List[PartAffinity] = []

    for ref in sorted(block_of):
        part = state.parts.get(ref)
        if part is None:
            continue
        own_block = block_of[ref]
        members = set(blocks.get(own_block) or ())
        for net_id in sorted({n for _x, _y, n in part.pad_globals()
                              if n and n not in ignored}):
            owners = set(state.net_refs.get(net_id, ()))
            if owners <= members:
                continue                      # wholly inside its own block
            if len(owners) < 3:
                # A two-owner net has ONE MST edge, incident on both parts, so
                # `share` is 1.0 for each of them and "dominates" means
                # nothing -- it would only be restating that the net is long.
                # That is block_displacement's question, and it already
                # answers it. Dominance needs a net one part can dominate.
                continue
            edges = _incident_edges(state, ref, net_id)
            if not edges:
                continue
            total = _mst_length(state.net_airwires.get(net_id, ()))
            if total <= 0:
                continue
            mine = _mst_length(edges)
            share = mine / total
            if share <= AFFINITY_DOMINANT_SHARE:
                continue

            pierced = tuple(c.name for c in (corridors or ())
                            if net_id not in c.net_ids
                            and _edges_cross_corridor(edges, c))

            recoverable = _recoverable(state, ref, net_id)
            if recoverable <= AFFINITY_MIN_RECOVERABLE_FRACTION * mine:
                # A part in the MIDDLE of a net's span is incident on both of
                # its edges, so it scores share 1.0 while being exactly where
                # it belongs -- share alone cannot tell a misplacement from a
                # topological hub. Moving a hub onto its own net's centroid
                # frees nothing; moving a misplaced part frees most of what it
                # carries. That difference is the discriminator, not `share`.
                continue
            out.append(PartAffinity(
                ref=ref, block=own_block,
                net=pcb_data.nets[net_id].name if net_id in pcb_data.nets
                    else str(net_id),
                net_id=net_id, share=share, length_mm=mine,
                reach_norm=mine / diag if diag else 0.0,
                pierced=pierced,
                recoverable_mm=recoverable,
                recoverable_norm=recoverable / diag if diag else 0.0,
                locked=bool(getattr(part, 'locked', False))))

    # Rank, do not threshold: corridors pierced first (the near-zero
    # false-positive signal), then how much a move could win, then dominance.
    out.sort(key=lambda a: (-len(a.pierced), -round(a.recoverable_norm, 4),
                            -round(a.share, 4), a.ref))
    return out


def _board_diagonal(state, pcb_data) -> float:
    """Board diagonal, for normalising a length into something comparable
    across boards. mm never is."""
    b = getattr(getattr(pcb_data, 'board_info', None), 'board_bounds', None)
    if b:
        return math.hypot(b[2] - b[0], b[3] - b[1])
    pts = [(p.x, p.y) for p in state.parts.values()]
    if len(pts) < 2:
        return 0.0
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return math.hypot(max(xs) - min(xs), max(ys) - min(ys))


def _edges_cross_corridor(edges, corridor: Corridor) -> bool:
    from .quench import _aw_array, _count_crossings_np
    sides = corridor.side_edges()
    if not sides or not edges:
        return False
    wall = _aw_array([(x1, y1, x2, y2, -1.0) for x1, y1, x2, y2 in sides])
    n, _w = _count_crossings_np(_aw_array(list(edges)), wall)
    return bool(n)


def _recoverable(state, ref: str, net_id: int) -> float:
    """MST length freed by moving `ref` onto the centroid of what it talks to.

    Upper bound: legality is not checked. Uses the quench's own override
    primitive so the number is the one the optimizer would see.
    """
    part = state.parts[ref]
    foreign = [(gx, gy) for other in state.net_refs.get(net_id, ())
               if other != ref and other in state.parts
               for gx, gy, n in state.parts[other].pad_globals() if n == net_id]
    if not foreign:
        return 0.0
    cx = sum(p[0] for p in foreign) / len(foreign)
    cy = sum(p[1] for p in foreign) / len(foreign)
    before = _mst_length(state.net_airwires.get(net_id, ()))
    moved = part.pad_globals(cx, cy, part.rot)
    after = _mst_length(state._build_net_airwires(
        net_id, override_ref=ref, override_pads=moved))
    return max(0.0, before - after)


def corridor_intrusions(state, corridor: Corridor,
                        refs: Optional[Sequence[str]] = None
                        ) -> List[Dict]:
    """Parts whose COURTYARD physically sits inside a corridor, with the depth.

    `foreign_crossings` sees airwires; it cannot see a part BODY parked in the
    channel (test-board run 5: R10's courtyard reached 1.03 mm INSIDE the QSPI
    corridor and every escape had to bend around it -- the watcher measured it
    by hand from pad coordinates). This makes that number mechanical: for each
    part (default: all movable+locked parts not owning a corridor net), test
    courtyard-rect vs corridor-rect by separating axes (exact for two rects),
    and report the overlap thickness along the corridor's cross-section --
    "how deep into the channel the body reaches", in mm.

    Returns [{'ref', 'depth_mm', 'along_mm'}] sorted deepest first. `along_mm`
    is the longitudinal overlap, so a part clipping a corner shows a small
    number on both axes.
    """
    dxc, dyc = corridor.b[0] - corridor.a[0], corridor.b[1] - corridor.a[1]
    length = math.hypot(dxc, dyc)
    if length < 1e-9:
        return []
    ux, uy = dxc / length, dyc / length          # along the channel
    vx, vy = -uy, ux                             # cross-section
    h = corridor.width_mm / 2.0
    own = set(corridor.net_ids)
    out = []
    for ref in sorted(refs if refs is not None else state.parts):
        part = state.parts.get(ref)
        if part is None or set(part.nets) & own:
            continue                             # its pads belong in the channel
        x1, y1, x2, y2 = part.rect()
        corners = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
        us = [(cx - corridor.a[0]) * ux + (cy - corridor.a[1]) * uy
              for cx, cy in corners]
        vs = [(cx - corridor.a[0]) * vx + (cy - corridor.a[1]) * vy
              for cx, cy in corners]
        along = min(max(us), length) - max(min(us), 0.0)
        depth = min(max(vs), h) - max(min(vs), -h)
        if along <= 0 or depth <= 0:
            continue
        # The interval tests above are a 2-axis SAT (corridor frame); finish
        # the rect-vs-rect test on the courtyard's own axes so a diagonal
        # corridor can't flag a part that only overlaps the bounding box.
        cor_pts = [(corridor.a[0] + vx * s * h, corridor.a[1] + vy * s * h)
                   for s in (-1, 1)] + \
                  [(corridor.b[0] + vx * s * h, corridor.b[1] + vy * s * h)
                   for s in (-1, 1)]
        xs = [p[0] for p in cor_pts]
        ys = [p[1] for p in cor_pts]
        if min(xs) >= x2 or max(xs) <= x1 or min(ys) >= y2 or max(ys) <= y1:
            continue
        out.append({'ref': ref, 'depth_mm': round(depth, 3),
                    'along_mm': round(along, 3)})
    out.sort(key=lambda r: (-r['depth_mm'], r['ref']))
    return out


def corridor_convergence(state, corridors: Sequence[Corridor],
                         class_nets: Dict[str, Sequence[int]],
                         ignore_net_ids: Optional[Sequence[int]] = None,
                         max_fanout: int = DISPLACEMENT_MAX_FANOUT
                         ) -> List[Tuple[str, List[str]]]:
    """Which declared critical classes crowd into each corridor.

    `class_nets` must come from the intent. Placement has no net-class notion,
    and "critical" is a design decision rather than a fact in the board file --
    guessing it would be exactly the silent heuristic this module refuses.
    """
    out = []
    for c in corridors:
        here = []
        for cls in sorted(class_nets):
            ids = set(class_nets[cls])
            if not ids or ids <= set(c.net_ids):
                continue
            n, guilty = foreign_crossings(state, c, ignore_net_ids,
                                          max_fanout)
            if n and ids & set(guilty):
                here.append(cls)
        if here:
            out.append((c.name, here))
    return out


def health(state, pcb_data, blocks: Dict[str, Sequence[str]],
           spec: Optional[Dict] = None) -> Dict[str, object]:
    """The composite. Each signal reports, or says why it did not run."""
    spec = spec or {}
    out: Dict[str, object] = {'skipped': {}}

    disp = block_displacements(
        state, blocks, ignore_net_ids=spec.get('ignore_net_ids'),
        max_fanout=int(spec.get('max_fanout', DISPLACEMENT_MAX_FANOUT)))
    if blocks:
        limit = spec.get('block_displacement_mm')
        out['block_displacement'] = [d.to_dict() for d in
                                     sorted(disp, key=lambda d: -d.distance_mm)]
        out['block_displacement_max_mm'] = (
            round(max((d.distance_mm for d in disp), default=0.0), 4))
        if limit is not None:
            out['blocks_displaced'] = sum(1 for d in disp
                                          if d.distance_mm > float(limit))
    else:
        out['skipped']['block_displacement'] = (
            'no blocks resolved; derive them with --group-by')

    # Escape lanes. Reported for every fine-pitch part the board HAS, with no
    # declaration needed -- a face in deficit is a fact about geometry, not
    # about intent, and requiring a declaration would mean it is only ever
    # measured on boards where someone already suspected it.
    try:
        from . import escape as _escape
        led = _escape.escape_ledger(
            pcb_data, pcb_file=getattr(pcb_data, 'source_path', None),
            ignore_net_ids=spec.get('ignore_net_ids'),
            # #700: name the plane layers the way `ignore_net_ids` names the
            # plane NETS. Left undeclared the ledger observes pours, and a
            # board being placed has none yet -- so the declaration is the
            # channel that actually carries an answer here.
            plane_layers=spec.get('plane_layers'))
    except Exception as exc:                      # noqa: BLE001
        out['skipped']['escape_lanes'] = f'not computed: {exc}'
        led = []
    if led:
        short = [p for p in led if p.worst]
        out['escape_lanes'] = [p.to_dict() for p in led[:10]]
        out['escape_parts'] = len(led)
        out['escape_deficit_parts'] = len(short)
        out['escape_worst_deficit'] = (led[0].worst.deficit
                                       if led[0].worst else 0)
        # #700's layer-aware pair, reported BESIDE the own-layer numbers and
        # never in place of them. `escape_worst_deficit_floor` is a LOWER
        # bound -- short even using every other signal layer -- so > 0 is a
        # strictly stronger verdict than the own-layer one, and == 0 proves
        # nothing. Nothing gates on either; see this function's own contract.
        floor_short = [p for p in led if p.worst_floor]
        out['escape_deficit_parts_all_layers'] = len(floor_short)
        out['escape_worst_deficit_floor'] = max(
            [p.worst_floor.deficit_floor for p in floor_short], default=0)
        f0 = led[0].faces[0] if led[0].faces else None
        out['escape_signal_layers'] = f0.signal_layers if f0 else 1
        out['escape_signal_layers_source'] = (f0.signal_layers_source if f0
                                              else 'unknown')
        # WHICH term bounds the relief, over the faces actually in deficit.
        # `via_slots` is layer-independent, so this is usually 'via_slots' --
        # which is the finding, not a defect: more layers do not help a face
        # whose via row is full.
        # `max(set(...), key=count)` would be PYTHONHASHSEED-dependent on a
        # tie -- set iteration over strings is randomised, so a published key
        # would differ run to run. Unreachable today (every board yields one
        # bound value) and fixed anyway: a nondeterministic output is a bug
        # whether or not a fixture reaches it.
        bounds = [f.supply_bound for p in led for f in p.faces if f.deficit]
        out['escape_supply_bound'] = (
            max(sorted(set(bounds)), key=bounds.count) if bounds else None)
        # WHO to move, deduped across parts in face order. This is the field
        # that turns "a face is short" into an action.
        blockers: List[str] = []
        for p in short:
            for f in p.faces:
                for b in f.blockers:
                    if f.deficit and b not in blockers:
                        blockers.append(b)
        out['escape_blockers'] = blockers[:10]
    elif 'escape_lanes' not in out['skipped']:
        out['skipped']['escape_lanes'] = 'no fine-pitch parts on this board'

    specs = spec.get('bus_corridors') or []
    corridors_for_affinity: List[Corridor] = []
    if specs:
        corridors = corridors_from_intent(state, pcb_data, specs)
        corridors_for_affinity = list(corridors)
        ign = spec.get('ignore_net_ids')
        fan = int(spec.get('max_fanout', DISPLACEMENT_MAX_FANOUT))
        rows = []
        for c in corridors:
            n, guilty = foreign_crossings(state, c, ign, fan)
            # run-6: part BODIES parked in the channel, deepest first (airwire
            # counts cannot see them). The row carries the five deepest for
            # display AND the untruncated count: consumers that scored
            # len(intrusions) were saturating at 5 per corridor, so a channel
            # with twelve parts in it ranked level with one that had five.
            intr = corridor_intrusions(state, c)
            rows.append({**c.to_dict(), 'foreign_crossings': n,
                         'cut_mm': round(corridor_cut_mm(state, c, ign, fan), 3),
                         'cover': round(corridor_cover(state, c), 3),
                         'worst_nets': [pcb_data.nets[i].name
                                        for i in guilty[:5]
                                        if i in pcb_data.nets],
                         'intrusions': intr[:5],
                         'intrusions_total': len(intr)})
        out['bus_corridors'] = rows
        out['bus_foreign_crossings'] = sum(r['foreign_crossings'] for r in rows)
        out['bus_cut_mm'] = round(sum(r['cut_mm'] for r in rows), 3)
        # A corridor whose own pads do not sit at its ends is a FICTION -- the
        # endpoints are an average of clusters that are not there. Surfaced so a
        # declared corridor can be disbelieved, rather than silently graded.
        phantom = [r['name'] for r in rows if r['cover'] < CORRIDOR_MIN_COVER]
        if phantom:
            out['bus_corridors_phantom'] = phantom
        classes = spec.get('classes') or {}
        if classes:
            from net_queries import matches_net_filter
            cn = {cls: [nid for nid, n in pcb_data.nets.items()
                        if nid > 0 and n.name
                        and matches_net_filter(n.name, list(pats))]
                  for cls, pats in classes.items()}
            out['convergence'] = [
                {'corridor': name, 'classes': cls}
                for name, cls in corridor_convergence(state, corridors, cn,
                                                      ign, fan)]
        else:
            out['skipped']['convergence'] = (
                'health.classes not declared; placement has no net-class '
                'notion and "critical" is design intent, not a board fact')
    else:
        out['skipped']['bus_corridors'] = 'health.bus_corridors not declared'
        out['skipped']['convergence'] = 'no corridors to converge in'

    # Per-part net affinity: which single part carries a net its own block
    # sits away from. Needs ZONED blocks -- a block with no declared seat
    # cannot be blamed for where its members ended up.
    zoned = spec.get('zoned_blocks')
    zoned_blocks = ({k: v for k, v in blocks.items() if k in set(zoned)}
                    if zoned is not None else blocks)
    if zoned_blocks:
        exempt = spec.get('affinity_exempt_net_ids')
        rows_aff = net_affinity(
            state, pcb_data, zoned_blocks, corridors_for_affinity,
            ignore_net_ids=spec.get('ignore_net_ids'),
            max_fanout=int(spec.get('max_fanout', DISPLACEMENT_MAX_FANOUT)),
            exempt_net_ids=exempt)
        out['net_affinity'] = [a.to_dict() for a in rows_aff[:5]]
        out['net_affinity_rows'] = len(rows_aff)
        # The near-zero-false-positive count: dominates a net AND pierces a
        # declared corridor. That pair is what turned into ten drop-vias and
        # eight plane voids on a real board.
        out['net_affinity_offenders'] = sum(1 for a in rows_aff if a.pierced)
        out['net_affinity_worst_norm'] = round(
            max((a.recoverable_norm for a in rows_aff), default=0.0), 4)
        if not corridors_for_affinity:
            out['skipped']['net_affinity_corridors'] = (
                'no corridors declared; affinity ranked on recoverable '
                'length alone, without the pierced-corridor signal')
    else:
        out['skipped']['net_affinity'] = (
            'no zoned blocks; without a declared seat there is no membership '
            'to blame for where a part sits')

    out['skipped']['blocked_cell_share'] = (
        'needs a routing attempt: the blocker JSON (#409) only exists after '
        'one, so this cannot be computed pre-route')
    return out


# --------------------------------------------------------------------------
# run-6: the per-face lane ledger and part-pair channels
# --------------------------------------------------------------------------
# SKILL.md specified the lane ledger in prose across three sections and no
# code existed ("the single highest-leverage productization target" per the
# run-6 exploration). V1 scope, stated honestly:
#   - supply counts lanes a face can pass at the routed pitch AND at the
#     finest legal grid (the run-5 grid confound, productized);
#   - lanes eaten by NEIGHBOR COPPER inside the escape band are charged
#     per neighbor (`eaten_by`), via `legality.part_copper_geometry` -- the
#     same box `escape` charges, since #841. It was the courtyard, justified
#     by consistency with the ASSEMBLY channel rather than by what obstructs
#     a track; see the note beside the kernel call below;
#   - supply-tap/via consumption is NOT modeled (v2; every consumer output
#     says so rather than faking it).
# Report-only: these numbers feed the placement fix loop's classification
# ("deficit at the finest grid = floorplan/placement-shaped by
# measurement"), never a gate by themselves.

FINEST_LEGAL_GRID_MM = 0.025


def _quantized_pitch(track_width: float, clearance: float,
                     grid_step: float) -> float:
    """One routed lane's pitch at a grid: track + clearance, rounded UP to
    the grid (a router cannot place copper between grid points)."""
    raw = track_width + clearance
    if grid_step <= 0:
        return raw
    return math.ceil(raw / grid_step - 1e-9) * grid_step


class LaneContext:
    """The whole-board geometry a lane ledger grades ONE ref against (#849).

    `face_lane_ledger` answers a per-REF question out of five WHOLE-BOARD
    objects, and it rebuilt all five on every call -- including
    `graded_parts_from_file`, which re-reads the `.kicad_pcb` from disk and
    re-parses every footprint's courtyards. `check_channels` calls the ledger
    once per auto-detected fine-pitch ref, and a SECOND time per ref with
    `--baseline`, so the sweep spent more time re-deriving geometry it already
    had than computing lane supply.

    This is the hoist `escape_ledger` already does for its own three board
    maps (`board_side_map`, `board_container_refs`, `board_copper_geometry`),
    and it is threaded the same way: an optional keyword that DEFAULTS to
    None and is built inside the ledger, so every direct caller and every
    test keeps working unchanged and byte-identical.

    Each member is built on FIRST USE rather than at construction, because
    the ledger's own early returns (a ref with no footprint, no pad model, or
    no extent) reach only `parts`; building the rest for them would make a
    bogus ref MORE expensive than it is today.

    WHAT MAKES SHARING SAFE is narrower than "nothing mutates". Four of the
    five are freshly built containers the ledger only reads. The fifth is
    not: `pp.extent(...)` writes into `PartPads._ext_cache` AND, via
    `PartPads._rotated`, into `_pad_cache` -- measured on rp2350, one ledger
    call grows BOTH on 61 of 61 parts. Both are pure memoization keyed on the
    pose delta, whose values are functions of `(pads_local, holes_extent,
    rot)` alone, so a shared map accumulates entries, answers identically,
    and makes the sweep faster still. Count them before repeating the
    sentence: "the one write that does happen" was wrong by one.

    The members are deliberately NOT a public tuple to unpack: `containers`
    is derived from `graded` and `geom` from `parts`, so a caller that hoisted
    four of five, or built `parts` at a different clearance than it priced the
    lane at, would get a silently wrong answer. `part_copper_geometry` refuses
    that same mismatch rather than trusting it (see `resolved_for`).
    """

    __slots__ = ('pcb_data', 'pcb_file', 'clearance', '_parts', '_net_owners',
                 '_graded', '_containers', '_geom')

    def __init__(self, pcb_data, clearance: float,
                 pcb_file: Optional[str] = None):
        self.pcb_data = pcb_data
        self.pcb_file = pcb_file
        self.clearance = float(clearance)
        self._parts = None
        self._net_owners = None
        self._graded = None
        self._containers = None
        self._geom = None

    @property
    def parts(self):
        """`{ref: PartPads}` at this context's clearance.

        Built WITHOUT `tolerant=True`, which is what the ledger has always
        done -- `part_copper_geometry`'s own fallback uses the tolerant form,
        so the two differ and a caller hand-mirroring the builder would
        silently change which parts are modelled.
        """
        if self._parts is None:
            from placement.legality import build_part_pads
            self._parts = build_part_pads(self.pcb_data.footprints or {},
                                          self.clearance)
        return self._parts

    @property
    def net_owners(self):
        """`{net_id: {ref, ...}}` -- which parts own each net.

        Whole-board, and a function of `pcb_data` alone: the demand half asks
        "does this net have somewhere to escape TO", which is the same answer
        for every ref on the board.
        """
        if self._net_owners is None:
            owners: Dict[int, set] = {}
            for r2, f2 in (self.pcb_data.footprints or {}).items():
                for pad in (f2.pads or []):
                    if pad.net_id:
                        owners.setdefault(pad.net_id, set()).add(r2)
            self._net_owners = owners
        return self._net_owners

    @property
    def graded(self):
        """`graded_parts_from_file(...)` -- THE `.kicad_pcb` courtyard parse.

        This is the one #849 is named for: `part_local_bounds` reaches
        `parser.extract_courtyard_sides`, which opens and regex-walks the
        whole board file. Once per ref is once per ref too many.
        """
        if self._graded is None:
            from placement.legality import graded_parts_from_file
            self._graded = graded_parts_from_file(self.pcb_data, self.pcb_file)
        return self._graded

    @property
    def containers(self):
        """The refs the courtyard channel treats as frames, not bodies."""
        if self._containers is None:
            from placement.legality import container_refs
            self._containers = container_refs(self.pcb_data, self.graded)
        return self._containers

    @property
    def geom(self):
        """`{ref: CopperGeometry}` -- the #841 obstruction rectangle."""
        if self._geom is None:
            from placement.legality import part_copper_geometry
            self._geom = part_copper_geometry(
                self.pcb_data.footprints or {}, self.clearance,
                parts=self.parts)
        return self._geom

    def resolved_for(self, pcb_data, clearance: float,
                     pcb_file: Optional[str],
                     caller: str = 'face_lane_ledger') -> 'LaneContext':
        """This context, or a `ValueError` naming the disagreement.

        Three things are checked because all three can silently produce a
        plausible wrong number, and the BOARD is the dangerous one:
        `check_channels` ledgers two different boards in one run (the board
        and its `--baseline`), so a context hoisted to the wrong scope would
        grade the baseline's refs against the primary board's geometry and
        fabricate a delta -- which `--gate` turns into exit 4 on a board that
        did not regress.

        `part_copper_geometry` already refuses a `parts=` map built at the
        wrong clearance, with the reasoning spelled out there; this is the
        same refusal one level up, for the same reason: a disagreement that
        is checkable should not be trusted instead.

        `caller` names the function in the message, because a message that
        names the WRONG function is worse than one that names none: two
        consumers reach here, and a hardcoded `face_lane_ledger:` reported
        refusals out of `pair_channel_widths` calls. It defaults to the
        common caller rather than being required, so a third consumer that
        forgets it gets a stale name -- pass it.

        TWO LIMITS, stated rather than implied:

        * It is NOT a total gate. `face_lane_ledger` returns `[]` for a ref
          the board does not have BEFORE it gets here, so a mismatched
          context handed a bogus ref is never noticed. That matches the old
          code's output exactly; it just is not a check.
        * The board test is identity, not equality, so a board MUTATED IN
          PLACE (a part moved) keeps passing while `graded` and `geom` stay
          frozen at the poses they were built from -- where the un-hoisted
          code would have rebuilt. No caller in this repo does that; one that
          moves parts between ledger calls must build a new context.
        """
        if self.pcb_data is not pcb_data:
            raise ValueError(
                '{}: the lane context was built for a DIFFERENT board object '
                '({} footprints, from {!r}) than the one being graded ({} '
                'footprints, from {!r}); its neighbours, courtyards and net '
                "owners are that board's, so this ref would be graded against "
                "another board's geometry".format(
                    caller, len(getattr(self.pcb_data, 'footprints', None) or {}),
                    self.pcb_file,
                    len(getattr(pcb_data, 'footprints', None) or {}), pcb_file))
        if self.pcb_file != pcb_file:
            raise ValueError(
                '{}: the lane context was built from '
                '{!r} but {!r} was passed; the courtyards come from the FILE '
                'and the two need not agree'.format(caller, self.pcb_file,
                                                    pcb_file))
        if abs(self.clearance - float(clearance)) > 1e-9:
            raise ValueError(
                '{}: the lane context was built at clearance '
                '{} but {} was requested; the pad boxes carry NPTH hole '
                'growth at the former and every face would be priced at the '
                'latter'.format(caller, self.clearance, clearance))
        return self


def board_lane_context(pcb_data, clearance: float, *,
                       pcb_file: Optional[str] = None) -> LaneContext:
    """The per-board half of `face_lane_ledger`, resolved ONCE (#849).

    Build one above a loop over refs and pass it to every call; see
    `LaneContext`. Measured by `tests/measure_849_hoist.py` (min-of-3, one
    process, the context built INSIDE the timed region because one per run is
    what the caller does) at clearance 0.09 / track 0.127 / grid 0.05:

        board                     basis  refs    sweep            parses
        tigard                    cli       2   0.147s -> 0.068s   2 -> 1
        rp2350_fpga_eensy_prePl.  cli       7   0.370s -> 0.061s   7 -> 1
        glasgow_revC              cli       9   2.568s -> 0.301s   9 -> 1
        glasgow_revC              fine     24   6.374s -> 0.262s  24 -> 1

    TWO BASES, and mixing them is the mistake this table exists to prevent.
    `cli` is what `check_channels` auto-detects and therefore what the TOOL
    costs. `fine` is `escape.fine_pitch_parts`, which is the basis #849
    measured -- and on that basis its "~97% of the sweep is avoidable"
    REPRODUCES here (95.9% removed, 24.3x), even though its absolute
    12.6-15.8s does not. A draft of this docstring compared the issue's
    24-ref sweep against a 9-ref number and concluded the opposite.

    The parse count is the load-independent half, and it is what
    `tests/test_849_lane_context.py` pins: seconds move with the machine,
    "once per ref" against "once per board" does not.

    tigard's 2.2x is the shape of the win, not a disappointment -- two refs
    is two rebuilds, so there is almost nothing to hoist, and it is the least
    stable figure here (independent re-measurement put it anywhere in
    1.5x-2.5x). The saving is per EXTRA ref, which is why a dense board in
    the fix loop feels it and a two-part board does not.
    """
    return LaneContext(pcb_data, clearance, pcb_file=pcb_file)


def face_lane_ledger(pcb_data, ref: str, *, clearance: float,
                     track_width: float, grid_step: float,
                     escape_band_mm: Optional[float] = None,
                     pcb_file: Optional[str] = None,
                     context: Optional[LaneContext] = None) -> List[Dict]:
    """Per-face escape supply/demand for one part (SKILL's lane ledger, v1).

    For each face (N/S/E/W of the part's pad extent):
      demand        signal nets with a pad ON this face and at least one pad
                    on another part (they must escape somewhere, and this
                    face is where their pad points). "On" is
                    `escape.assign_faces` (#850), the rule the escape ledger
                    uses: the pad's own copper edge against the part's copper
                    box, within `max(pad_pitch/2, INTERIOR_EPS)`. A pad boxed
                    in on every side is INTERIOR -- it cannot leave sideways
                    at any pitch and needs a via -- and counts toward no
                    face's demand. It was "NEAREST this face", which has no
                    interior case at all, so a BGA's inner balls were demand
                    on whichever edge they happened to sit closest to.
      interior      how many pads that was, and how many nets the faces lost
                    to them (`interior_pads` / `interior_nets` /
                    `interior_demand_nets`).
      supply        face length / lane pitch, minus lanes covered by
                    NEIGHBOR bodies inside the escape band. Reported at
                    the ROUTED grid and at the FINEST legal grid -- a
                    deficit that survives the finest grid is a floorplan
                    fact, not a parameter fact.
      eaten_by      [(neighbor ref, lanes covered)], the fix targets.

    Rail nets (fanout > DISPLACEMENT_MAX_FANOUT) are not demand: planes
    feed them, not face escapes. Tap/via lane consumption is v2.
    """
    from placement.legality import (footprint_has_through_pads,
                                    footprint_side, rect_on_sides,
                                    sides_occupied)
    from .escape import (FACE_LETTER, assign_faces as _assign_faces,
                         escape_band as _escape_band, span_eaten)

    fps = pcb_data.footprints or {}
    fp = fps.get(ref)
    if fp is None:
        return []
    # #849: the five whole-board objects below are resolved ONCE per BOARD,
    # not once per ref. A caller sweeping refs (`check_channels`, and its
    # `--baseline` sweep separately) builds one `board_lane_context` above its
    # loop and passes it; a direct caller passes nothing and gets one built
    # here, so this stays a standalone entry point. See `LaneContext`.
    ctx = (board_lane_context(pcb_data, clearance, pcb_file=pcb_file)
           if context is None
           else context.resolved_for(pcb_data, clearance, pcb_file))
    pp = ctx.parts.get(ref)
    if pp is None:
        return []
    ext = pp.extent(fp.x, fp.y, fp.rotation or 0.0)
    if ext is None:
        return []

    net_owners = ctx.net_owners

    faces = {'N': (ext[0], ext[1], ext[2], ext[1]),
             'S': (ext[0], ext[3], ext[2], ext[3]),
             'W': (ext[0], ext[1], ext[0], ext[3]),
             'E': (ext[2], ext[1], ext[2], ext[3])}
    pitch_routed = _quantized_pitch(track_width, clearance, grid_step)

    # #850: THE FACE RULE IS `escape.assign_faces`, the one this ledger's
    # counterpart uses. What stood here was a third answer to the same
    # question: `min` over `abs(pad CENTRE - ext edge)`, with no tolerance and
    # no interior bucket, so EVERY netted pad was demand on some face -- a
    # BGA's inner balls included, which cannot escape sideways at any pitch
    # and need a via. `escape` reports those separately and says why: rolling
    # one into a face's demand blames the face for a fanout problem.
    #
    # Three things change with the rule, and they do not all push one way:
    #   * the pad is measured by its own COPPER EDGE against
    #     `CopperGeometry.copper`, not by its centre against the hole-inclusive
    #     `extent`. Note both halves -- measuring a CENTRE against a copper box
    #     is the mis-pairing `escape.face_of` measured at 244 interior pads on
    #     one board, and it reads as a fix.
    #   * a `max(pad_pitch/2, INTERIOR_EPS)` tolerance, so a pad row that is
    #     not perfectly collinear still counts.
    #   * ties resolve in `escape.FACES` order (N, E, S, W) rather than by this
    #     dict's insertion order (N, S, W, E).
    # The first two can only move a pad OFF a face; the last MOVES it between
    # two, so a face's demand can rise. The row order below is unchanged.
    #
    # `ext` is deliberately still the whole-part `PartPads.extent`: the face
    # GEOMETRY, its length and the obstruction band are what they were, and
    # only assignment moved. That separation is what makes it checkable that
    # this commit changed the demand model and nothing else.
    own = ctx.geom.get(ref)
    # #862: `lane_mm` stays the PITCH fallback only. The enclosure basis is
    # the raw `track_width` / `clearance` this ledger was called with, never
    # `pitch_routed` -- that one is grid-quantized (#847), and a structural
    # verdict that moves with `--grid-step` is exactly the confound the two
    # supply columns exist to keep out. `escape.part_escape` resolves the
    # same pair, so the two ledgers price one corridor.
    asg = _assign_faces(fp, own, lane_mm=pitch_routed, fallback_rect=ext,
                        clearance=clearance, track_width=track_width)

    demand: Dict[str, set] = {f: set() for f in faces}
    interior_pads = 0
    interior_nets: set = set()
    interior_demand: set = set()
    for pad, face in asg.faces:
        nid = pad.net_id
        if not nid:
            continue
        # GEOMETRY FIRST, THE NET FILTER SECOND. The other way round, the
        # published interior count would be over the owner-filtered subset and
        # would not be comparable with `escape.PartEscape.interior_pads`,
        # which is #850's own stated verification. So `interior_pads` counts
        # the same population escape counts, and `interior_demand_nets` is the
        # part of it this ledger's demand model would otherwise have charged.
        if face is None:
            interior_pads += 1
            interior_nets.add(nid)
        owners = net_owners.get(nid, set())
        if len(owners) < 2:
            continue                      # nowhere to escape to
        if len(owners) > DISPLACEMENT_MAX_FANOUT:
            continue                      # rail: plane-fed, not face-fed
        if face is None:
            interior_demand.add(nid)
            continue
        demand[FACE_LETTER[face]].add(nid)
    # A net with one pad interior and another on a face is NOT lost demand --
    # it still has to leave through that face. Only the nets with no pad on
    # any face are.
    interior_demand -= set().union(*demand.values()) if demand else set()

    pitch_fine = _quantized_pitch(track_width, clearance,
                                  FINEST_LEGAL_GRID_MM)
    # #847: ONE resolver, shared with `escape.part_escape`. Both open-coded
    # `max(1.0, 4 * pitch)` -- this one on the grid-QUANTIZED pitch, `escape`
    # on the raw `track + clearance` -- so the two ledgers looked DIFFERENT
    # depths off the same face and no file said so. Measured over the 22
    # tracked boards they disagree on 19 (2.2mm against 2.4mm at the
    # routing_defaults fallback). The basis stays each ledger's own here, and
    # is now a reported field rather than an invisible difference; unifying it
    # moves published deficits on every dense board and is its own decision.
    _band = _escape_band(pitch_routed, basis='quantized_lane',
                         override=escape_band_mm)
    band = _band.mm

    # #835: the SYMMETRIC side test, `own & other`, not `own_side in g.sides`.
    # The one-sided form charges a through-hole part only for neighbours on
    # its `footprint_side`, though its leads occupy both faces -- measured on
    # rp2350, that drops the B-side U1 from the drilled U6's blockers, losing
    # a real obstruction. `sides_occupied` is what every other pair predicate
    # in the package uses (`legality.pair_min_gap`, `pair_channel_widths`
    # below).
    #
    # ...and containers are exempt here for the same reason they are exempt in
    # the courtyard channel and now in `escape`: a module-outline footprint
    # covering half the board is a frame the part sits inside, not a body
    # parked off its face.
    own_sides = sides_occupied(footprint_side(fp),
                               footprint_has_through_pads(fp))
    _graded = ctx.graded
    _containers = ctx.containers
    # #841: the neighbour RECT is the pad COPPER, not the courtyard. The
    # SIDE and CONTAINER questions stay on `_graded`: "which faces does this
    # part occupy" and "is this footprint a frame" are both properties of the
    # drawn outline, and `CONTAINER_RATIO` was calibrated on courtyard AREA.
    # A ref with no copper geometry is dropped -- measured, that set is
    # exactly the board's `synthetic` refs on every tracked board that has one
    # (ulx3s 9, glasgow 8, orangecrab 3, tigard 3, esp_prog 3, watchy 2, and
    # the two interf_u boards 1 each), the +/-0.5mm fiction
    # `grade_body_overlap` already refuses to gate on. See
    # `legality.part_copper_geometry` for the full census.
    #
    # #848: and what a charged neighbour CONTRIBUTES is the box over the sides
    # it SHARES with the escaping part, not over all its pads. The filter one
    # line down is already per-side; the rectangle was not, so an F.Cu part
    # escaping past a B.Cu connector with a few through pins was charged the
    # connector's whole back-side pad field -- copper these tracks never have
    # to share. `rect_on_sides` unions the shared sides and falls back to the
    # whole box whenever either side is unknown. Holes are in BOTH side boxes
    # (a drill removes copper on every layer), which is why this is very nearly
    # inert: 18 (ref, side) boxes differ corpus-wide, 3 are ever charged
    # against a face in deficit, and no board's deficit moves.
    _geom = ctx.geom
    neighbors = []
    for g in _graded:
        if g.ref == ref or g.ref in _containers or g.ref not in _geom:
            continue
        shared = own_sides & g.sides
        if not shared:
            continue
        neighbors.append((g.ref, rect_on_sides(_geom[g.ref], shared)))

    out = []
    for fname, (x0, y0, x1, y1) in faces.items():
        horiz = (y0 == y1)
        length = (x1 - x0) if horiz else (y1 - y0)
        lo, hi = (x0, x1) if horiz else (y0, y1)
        if horiz:
            band_across = ((y0 - band, y0) if fname == 'N'
                           else (y0, y0 + band))
        else:
            band_across = ((x0 - band, x0) if fname == 'W'
                           else (x0, x0 + band))
        # #835: ONE obstruction kernel, shared with `escape._blocked_span`.
        # What it fixes here is the double charge: this loop accumulated
        # `covered += span` per neighbour, so two bodies covering the same
        # stretch were billed twice and a face could be reported as more than
        # fully blocked -- measured, glasgow_revC's J1 east accumulated
        # 11.82mm of cover on a 9.64mm face, which only `max(0.0, length -
        # covered)` kept from going negative. The kernel unions the intervals
        # and clamps to the span, so an impossible total is impossible rather
        # than absorbed.
        #
        # ...and the same neighbour RECTANGLE as `escape`, since #841: the
        # pad-copper box `part_copper_geometry` resolves once above. The two
        # ledgers now agree on every input to this kernel.
        covered, order = span_eaten(lo, hi, band_across, horiz, neighbors)
        eaten = [(r, round(mm / pitch_routed, 2)) for r, mm in order]
        usable = max(0.0, length - covered)
        supply_routed = int(usable // pitch_routed)
        supply_fine = int(usable // pitch_fine)
        n_demand = len(demand[fname])
        out.append({'face': fname,
                    'length_mm': round(length, 3),
                    'demand_nets': n_demand,
                    'supply_routed_grid': supply_routed,
                    'supply_finest_grid': supply_fine,
                    'deficit_routed_grid': max(0, n_demand - supply_routed),
                    'deficit_finest_grid': max(0, n_demand - supply_fine),
                    'eaten_by': eaten[:8],
                    'taps_not_modeled': True,
                    # #847: the band this row was measured at, and the term
                    # that set it ('caller' / 'lanes' / 'floor'). Two ledgers
                    # graded at different bands are not comparable, and until
                    # now nothing in the output said which band either used.
                    'escape_band_mm': round(_band.mm, 4),
                    'escape_band_source': _band.source,
                    'escape_band_basis': _band.basis,
                    # #850: PART-level facts, repeated on all four rows the
                    # way `escape_band_mm` above already is, because the row
                    # is what `check_channels --json` publishes and a caller
                    # holding one row must be able to read them.
                    #
                    # `interior_pads` counts netted pads that escape through
                    # NO face -- the same population `escape.PartEscape.
                    # interior_pads` counts, and equal to it for the same ref
                    # at the same clearance. That equality is the assertion
                    # that says these two instruments were reconciled rather
                    # than merely moved closer.
                    #
                    # `interior_demand_nets` is what the four faces LOST: the
                    # nets that clear this ledger's own 2..DISPLACEMENT_MAX_
                    # FANOUT owner filter and have no pad on any face. It is
                    # not decoration. Demand falling with nothing in the
                    # output naming where it went is how a ledger stops
                    # looking and reads as a fix.
                    'interior_pads': interior_pads,
                    'interior_nets': len(interior_nets),
                    'interior_demand_nets': len(interior_demand),
                    # ...and the tolerance the face rule ran at. The two
                    # ledgers resolve different LANES (#847) and this is a
                    # second, separate basis: the part's own pad pitch, or
                    # this ledger's quantized lane when the pad lattice
                    # yields none. Reported for the same reason the band is.
                    'face_pitch_mm': round(asg.pitch_mm, 4),
                    'face_pitch_source': asg.pitch_source})
    # SHARED WITH `escape` (#835): the obstruction arithmetic -- the
    # interval union, the clamp, the symmetric side test and the container
    # exemption -- is one kernel, `escape.span_eaten`, because the two ledgers
    # were answering the same question three different ways. What the union
    # fixes here: this loop billed `covered += span` per neighbour, so a face
    # could be reported as more than fully blocked and `max(0.0, length -
    # covered)` absorbed the impossible total as supply 0 rather than refusing
    # it. The witness is ulx3s H4 south -- two neighbours covering 6.16mm of a
    # 5.50mm face; unioned it is 3.71mm and supply 4. (It was glasgow_revC's
    # J1 east, 12.42mm on 9.64mm, supply 3. At the pad-copper rect that face
    # is supply 8 whether summed or unioned, so it is history, not an example
    # -- see `escape.span_eaten`, which carries the same note.)
    #
    # ...and since #841 the neighbour RECTANGLE is shared too, which is what
    # settles the question this block used to leave open. What obstructs a
    # TRACK is foreign COPPER; what obstructs a PART is a foreign BODY, and a
    # courtyard is an assembly keep-out drawn deliberately beyond the copper.
    # A lane ledger wants the first. Both ledgers charge
    # `legality.part_copper_geometry`.
    #
    # This ledger's own deficit at the finest grid, over the fine-pitch parts,
    # at clearance 0.2 / track 0.2 / grid 0.05 -- courtyard (before) -> pad
    # copper (now), with the bbox of pad CENTRES for scale, since that is what
    # `escape` charged and it is NOT the same box:
    #
    #   glasgow_revC  116 ->  75 (37)     orangecrab_ext_pll 163 -> 147 (87)
    #   rp2350         67 ->  53 (30)     tigard              54 ->  37  (8)
    #   watchy         31 ->  10  (2)     ulx3s               72 ->  68 (65)
    #   kit-dev / splitflap / esp_prog: 0 throughout
    #
    # ONE CONSEQUENCE, disclosed rather than buried: this takes
    # `check_channels --gate` from exit 4 to exit 0 on the only wrong-basin
    # fixture the starved-face gate has. The detection turned on U30's 1.325mm
    # courtyard skirt reaching into a 1.0mm escape band its copper does not.
    # `tests/test_run8_starved_face_gate.py` records the measurement, pins the
    # band dependence, and says the band needs its own re-derivation -- the
    # band floor and the neighbour rect are one model.
    #
    # ...and since #850 the FACE RULE is shared too -- `escape.assign_faces`,
    # so a pad points at the same face in both instruments and an interior pad
    # is interior in both. `interior_pads` on the rows above equals
    # `escape.PartEscape.interior_pads` for the same ref at the same
    # clearance, which is the assertion that says so.
    #
    # Still different, and deliberately: WHICH NETS each ledger asks about.
    # This one takes nets with >= 2 owners and <= DISPLACEMENT_MAX_FANOUT;
    # escape takes an `ignore_net_ids` list from its caller. Those are two
    # answers to "which nets need a lane", not two answers to "where does this
    # pad point", and only the second was a disagreement. Also still separate:
    # the grid quantization below, and the FACE pitch the tolerance is
    # resolved from -- reported per row as `face_pitch_mm` /
    # `face_pitch_source`, the same treatment #847 gave the band.
    #
    # NO LAYER TERM HERE, deliberately (#700). `escape.FaceLedger` gained
    # `via_slots` / `supply_other_max` / `deficit_floor`; this ledger did not,
    # because its supplies are GRID-QUANTIZED (`_quantized_pitch` above) while
    # a via pitch is not. Pairing the two would give a `supply_bound` that
    # flips with `--grid-step` -- a structural verdict moving with a raster
    # setting, which is the confound `check_channels.py` documents at its
    # floor-wrapping block. Adding it here needs
    # `_quantized_pitch(via_pitch, 0.0, grid_step)` and its own thought.
    # (`escape_band_mm` above was long described here as "a public keyword no
    # caller has ever passed, in either of this function's two call sites or
    # its tests" -- already false when it was written, since
    # `tests/test_run8_starved_face_gate.py` passes it. Since #847 it is also
    # reachable from the shipped CLI as `check_channels --escape-band`, and
    # every row reports the band it was graded at.)
    return out


def pair_channel_widths(pcb_data, *, clearance: float,
                        min_extent_mm: float = 3.5,
                        radius_mm: float = 15.0,
                        pcb_file: Optional[str] = None,
                        context: Optional[LaneContext] = None) -> List[Dict]:
    """Measured channel width between ANCHOR-sized parts, plus what lives
    in the channel -- the run-2 corridor law's inputs, measured instead of
    declared (Corridor.width_mm has only ever been an intent input).

    The corridor is for the PARTS (measured against THE GAP A HUMAN LEFT,
    r=+0.41..+0.90 on 8/8 boards -- never against routed `blocking`, which no
    predictor here has been correlated with; see docs/placement-predictors.md):
    each row reports the gap and the small parts whose bodies sit inside
    the channel rectangle between the two anchors.

    `context` (#849) is the same `LaneContext` the lane ledger takes, and is
    used ONLY for its `graded` -- this function's whole per-board input.
    `check_channels` calls this once, right after a sweep that already built
    that list, so without it the tool parses the board's courtyards one more
    time than it has refs. Defaults to None and rebuilds, and the context is
    checked against this call's board and file before it is believed.
    """
    from placement.legality import graded_parts_from_file, rect_gap

    # `clearance` is not part of the check: this function never touches the
    # pad boxes it prices, so a context built at another clearance carries
    # exactly the `graded` list this wants. Passing `context.clearance`
    # asserts the pair that matters -- board and file -- and nothing it would
    # be lying about.
    parts = (graded_parts_from_file(pcb_data, pcb_file) if context is None
             else context.resolved_for(pcb_data, context.clearance,
                                       pcb_file,
                                       caller='pair_channel_widths').graded)
    big = [g for g in parts
           if max(g.rect[2] - g.rect[0], g.rect[3] - g.rect[1])
           >= min_extent_mm]
    big_refs = {g.ref for g in big}
    small = [g for g in parts if g.ref not in big_refs]
    rows = []
    for i, a in enumerate(big):
        for b in big[i + 1:]:
            if not (a.sides & b.sides):
                continue
            gap = rect_gap(a.rect, b.rect)
            if gap <= 0 or gap > radius_mm:
                continue
            chan = (max(min(a.rect[0], b.rect[0]),
                        min(a.rect[2], b.rect[2])),
                    max(min(a.rect[1], b.rect[1]),
                        min(a.rect[3], b.rect[3])),
                    min(max(a.rect[0], b.rect[0]),
                        max(a.rect[2], b.rect[2])),
                    min(max(a.rect[1], b.rect[1]),
                        max(a.rect[3], b.rect[3])))
            inside = [s.ref for s in small
                      if (s.sides & a.sides)
                      and rect_gap(s.rect, chan) < 0]
            rows.append({'a': a.ref, 'b': b.ref,
                         'channel_mm': round(gap, 3),
                         'parts_in_channel': sorted(inside)[:12]})
    rows.sort(key=lambda r: r['channel_mm'])
    return rows


def relief_move(rect_a, rect_b, need_mm, limit_mm=5.0, tol_mm=1e-6):
    """How far must B move, per axis direction, for the A-B gap to reach need_mm?

    Answers the question a throat measurement leaves open: "so what do I DO?".
    Run 20 measured a 0.409 mm gap against a 0.450 mm need and then had to work
    out by hand that moving R7 east by ~0.042 mm would clear it.

    Bisection on `rect_gap`, which already returns the diagonal corner distance
    for rects that miss on both axes -- i.e. exactly the geometry being measured.
    Four axis directions; a direction that cannot reach `need_mm` within
    `limit_mm` is omitted rather than reported as a large number.

    Every result is stamped `bound: "lower"` and that is not decoration. This is
    a TWO-BODY answer on a board with hundreds of bodies: moving B by this much
    clears THIS pair and may create another. It is the smallest move that could
    possibly work, never a solution.
    """
    from placement.legality import rect_gap

    def shifted(d, t):
        dx, dy = {'east': (t, 0.0), 'west': (-t, 0.0),
                  'north': (0.0, -t), 'south': (0.0, t)}[d]
        return (rect_b[0] + dx, rect_b[1] + dy,
                rect_b[2] + dx, rect_b[3] + dy)

    out = []
    for d in ('east', 'west', 'north', 'south'):
        if rect_gap(rect_a, shifted(d, limit_mm)) < need_mm - tol_mm:
            continue                       # unreachable within the cap: say nothing
        lo, hi = 0.0, limit_mm
        if rect_gap(rect_a, rect_b) >= need_mm - tol_mm:
            out.append({'dir': d, 'min_mm': 0.0, 'bound': 'lower'})
            continue
        for _ in range(60):
            mid = (lo + hi) / 2.0
            if rect_gap(rect_a, shifted(d, mid)) >= need_mm:
                hi = mid
            else:
                lo = mid
            if hi - lo < tol_mm:
                break
        out.append({'dir': d, 'min_mm': round(hi, 6), 'bound': 'lower'})
    out.sort(key=lambda r: r['min_mm'])
    return out

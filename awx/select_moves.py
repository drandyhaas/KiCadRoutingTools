"""Choose one escape move per berth pad from the menu.

The braid's own entry assignment only ever considered moves that go WEST
-- toward the corridor -- and when a net could not take one it was split
off into a separate hand-written flow. The menu (escape_moves.py) offers
every direction, so the choice becomes a real decision:

  * a move costs its own vias, plus the corridor length needed to reach
    its exit point from the net's launch;
  * moves compete for the same channels -- one net per row gap, one net
    per column gap, one via per inter-ball cell -- so the choice is an
    assignment problem, not a per-net one.

Selection is greedy over a cost, in most-constrained-first order, which
is enough to reproduce the old westward choices when only westward moves
are offered (`only_dirs={'left'}`) and to spread onto the other edges
when they are not. It reports WHY each net got what it got, because a
silent assignment is impossible to audit.
"""
from __future__ import annotations

import math
from typing import (Callable, Dict, Iterable, List, Optional, Sequence,
                    Tuple)

from escape_moves import Move

Pt = Tuple[float, float]


def _lane_span(m: Move) -> Tuple[Tuple, float, float]:
    """The channel this move occupies and the interval it takes along
    it: a row gap (left/right escape) or a column gap (up/down), on its
    own layer, from where the escape enters the gap to where it leaves
    the array."""
    src = m.site if m.site is not None else m.legs[0][0]
    if m.direction in ('left', 'right'):
        key = ('row', round(m.exit_pt[1], 3), m.layer)
        a, b = src[0], m.exit_pt[0]
    else:
        key = ('col', round(m.exit_pt[0], 3), m.layer)
        a, b = src[1], m.exit_pt[1]
    return key, min(a, b), max(a, b)


def _length(m: Move) -> float:
    return sum(math.hypot(q[0] - p[0], q[1] - p[1])
               for (p, q, _L) in m.legs)


def _seg_hits_box(a: Pt, b: Pt, box) -> bool:
    """Does segment a-b pass through the axis-aligned box?"""
    x0, y0, x1, y1 = box
    # Liang-Barsky
    dx, dy = b[0] - a[0], b[1] - a[1]
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, a[0] - x0), (dx, x1 - a[0]),
                 (-dy, a[1] - y0), (dy, y1 - a[1])):
        if abs(p) < 1e-12:
            if q < 0:
                return False
            continue
        r = q / p
        if p < 0:
            if r > t1:
                return False
            if r > t0:
                t0 = r
        else:
            if r < t0:
                return False
            if r < t1:
                t1 = r
    return t0 < t1


def around_box_path(a: Pt, b: Pt, box, pad: float = 0.3):
    """The polyline `around_box` measures: the straight line when it
    misses the box, otherwise the shorter way round its padded corners.
    Returned so the corridor leg can be DRAWN, not just priced."""
    x0, y0, x1, y1 = box
    bx = (x0 - pad, y0 - pad, x1 + pad, y1 + pad)
    if not _seg_hits_box(a, b, bx):
        return [a, b]
    x0, y0, x1, y1 = bx
    corners = ((x0, y0), (x1, y0), (x0, y1), (x1, y1))
    best, path = float('inf'), [a, b]
    for c1 in corners:
        for c2 in corners:
            if _seg_hits_box(a, c1, bx) or _seg_hits_box(c2, b, bx):
                continue
            if c1 != c2 and _seg_hits_box(c1, c2, bx):
                continue
            d = (math.hypot(c1[0] - a[0], c1[1] - a[1])
                 + math.hypot(c2[0] - c1[0], c2[1] - c1[1])
                 + math.hypot(b[0] - c2[0], b[1] - c2[1]))
            if d < best:
                best = d
                path = [a, c1, c2, b] if c1 != c2 else [a, c1, b]
    return path


# The router's own via/length exchange rate: a via costs 75 units on
# the 0.1 mm grid, i.e. one via == 7.5 mm of track. Every objective
# that mixes vias with distance converts at this ONE rate.
VIA_MM = 7.5


def ride_mm(sel: Dict[str, 'Move'], launch: Dict[str, Pt],
            keep_out) -> float:
    """Total corridor ride length the destination choice implies: each
    net's around-the-array distance from its launch point to its berth
    exit. The judged plan objective adds this at VIA_MM per via so a
    berth on a far face pays its wrap -- the general cost that replaces
    face restrictions (a floor-only objective walked berths to far
    faces for free and unrestricted plans routed WORSE than restricted
    ones)."""
    return sum(around_box(launch[n], m.exit_pt, keep_out)
               for n, m in sel.items() if n in launch)


def around_box(a: Pt, b: Pt, box, pad: float = 0.3) -> float:
    """Distance from a to b that does not cross `box`. The straight
    line when it misses; otherwise the shorter of the two ways round,
    bending at the padded corners. The corridor cannot cross the array,
    so a straight-line reach through it is not a distance the router
    could ever realise."""
    x0, y0, x1, y1 = box
    box = (x0 - pad, y0 - pad, x1 + pad, y1 + pad)
    if not _seg_hits_box(a, b, box):
        return math.hypot(b[0] - a[0], b[1] - a[1])
    x0, y0, x1, y1 = box
    corners = ((x0, y0), (x1, y0), (x0, y1), (x1, y1))
    best = float('inf')
    for c1 in corners:
        for c2 in corners:
            if _seg_hits_box(a, c1, box) or _seg_hits_box(c2, b, box):
                continue
            if c1 != c2 and _seg_hits_box(c1, c2, box):
                continue
            d = (math.hypot(c1[0] - a[0], c1[1] - a[1])
                 + math.hypot(c2[0] - c1[0], c2[1] - c1[1])
                 + math.hypot(b[0] - c2[0], b[1] - c2[1]))
            best = min(best, d)
    if best == float('inf'):
        # both endpoints inside the padded box: fall back to straight
        return math.hypot(b[0] - a[0], b[1] - a[1])
    return best


def _site_key(m: Move) -> Optional[Tuple]:
    if m.site is None:
        return None
    return (round(m.site[0], 3), round(m.site[1], 3))


def side_capacity(menu: Dict[str, List[Move]],
                  bus: Sequence[str], side: str) -> int:
    """How many of this bus can leave on `side` at once: the number of
    distinct (exit line, layer) slots its members can actually reach.
    This is the cut the bundle has to cross."""
    axis = 1 if side in ('left', 'right') else 0
    slots = set()
    for n in bus:
        for m in menu.get(n, ()):
            if m.direction == side:
                slots.add((round(m.exit_pt[axis], 3), m.layer))
    return len(slots)


def certify(menu: Dict[str, List[Move]], bus: Sequence[str],
            side: str) -> Tuple[bool, str]:
    """Maley-style capacity check: every member must have a move on
    this side, and the side must offer at least as many distinct
    (line, layer) slots as there are members. Returns (ok, why)."""
    missing = [n for n in bus
               if not any(m.direction == side for m in menu.get(n, ()))]
    if missing:
        return False, (f'{len(missing)} member(s) have no {side} move '
                       f'({",".join(missing[:4])})')
    cap = side_capacity(menu, bus, side)
    if cap < len(bus):
        return False, (f'cut too narrow: {cap} slots for {len(bus)} nets')
    return True, f'{cap} slots for {len(bus)} nets'


def bus_sides(menu: Dict[str, List[Move]],
              launch: Dict[str, Pt],
              buses: Sequence[Sequence[str]],
              cost_fn, geo: Optional['Corridor'] = None,
              cross_weight: float = 6.0, log=None) -> Dict[str, str]:
    """One exit side per bus: the cheapest side that PASSES the capacity
    certificate AND does not cut through the corridors already placed.

    Certifying first means a side that cannot hold the bundle is never
    offered, instead of being discovered one unplaceable net at a time
    inside greedy assignment.

    The crossing term is the whole point of doing this in order rather
    than per bus. Corridors are as expensive to each other as their
    members are among themselves -- measured at K21, 30 crossings
    BETWEEN corridors against 34 within them -- and a per-bus choice
    cannot see that, because the cost of a side depends on what is
    already there. A 2-net bus sent `up` around the array cost 30
    crossings against two bundles it had no business touching, and
    scored as the cheapest side available.

    Buses are placed largest first, so the big bundles lay down the
    reference and the small ones fit around them, rather than a two-net
    bus dictating terms to an eleven-net one.
    """
    out: Dict[str, str] = {}
    placed: List[List[Pt]] = []
    for bus in sorted(buses, key=len, reverse=True):
        scored = []
        for d in ('left', 'right', 'up', 'down'):
            ok, why = certify(menu, bus, d)
            if not ok:
                if log:
                    log(f'  bus[{len(bus)}] {d}: REFUSED -- {why}')
                continue
            pick, s = {}, 0.0
            for n in bus:
                m = min((m for m in menu[n] if m.direction == d),
                        key=lambda m: cost_fn(n, m))
                pick[n] = m
                s += cost_fn(n, m)
            xs = 0
            if geo is not None and placed:
                for n in bus:
                    leg = geo.leg(n, pick[n])
                    xs += sum(1 for other in placed
                              if geo.paths_cross(leg, other))
            scored.append((s + cross_weight * xs, d, why, xs, pick))
        if not scored:
            continue
        scored.sort(key=lambda r: r[0])
        s, best, why, xs, pick = scored[0]
        if log:
            alt = ', '.join(f'{d}:{c:.0f}(+{x} cross)'
                            for c, d, _w, x, _p in scored[1:])
            log(f'  bus[{len(bus)}] -> {best} ({why}, cost {s:.0f}, '
                f'{xs} crossings into placed corridors)'
                + (f'   over {alt}' if alt else ''))
        for n in bus:
            out[n] = best
        if geo is not None:
            placed.extend(geo.leg(n, pick[n]) for n in bus)
    return out


def _other(layer: str, layers=('F.Cu', 'B.Cu')) -> str:
    return layers[1] if layer == layers[0] else layers[0]


def _proper_cross(p1: Pt, p2: Pt, p3: Pt, p4: Pt) -> bool:
    """Do segments p1p2 and p3p4 properly intersect? Shared endpoints
    and collinear touching do not count."""
    def d(a, b, c):
        return ((b[0] - a[0]) * (c[1] - a[1])
                - (b[1] - a[1]) * (c[0] - a[0]))
    d1, d2 = d(p3, p4, p1), d(p3, p4, p2)
    d3, d4 = d(p1, p2, p3), d(p1, p2, p4)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


class Corridor:
    """How a corridor is ordered, and which of its nets can stay put.

    The via floor is 2*(K - a), where `a` is the largest set of legs
    that pairwise do not cross: everyone else has to dive and come back.
    Reading that off a 1-D permutation is only valid when a coordinate
    exists that really does order the crossings, and the obvious
    candidates were each measured wrong on this bench:

      launch y / exit axis  assumes the launches are a VERTICAL comb.
                            The `down` corridor launches from a
                            horizontal comb south-west of the array,
                            every member within 0.3 mm of one y, so
                            ordering by y is a coin flip -- it called
                            9 of 28 pairs crossing where 3 do.
      angle about the array assumes the bundle WRAPS the array. That
                            corridor approaches the bottom edge head-on
                            from the south and wraps nothing: 6 pairs
                            predicted, 5 of them wrong.

    So the projection is used only to PROPOSE an order -- the transverse
    axis of the bundle's own mean travel, which needs no comb
    orientation and no side cases, and which measured best of the five
    tried (2 wrong of 28 on `down`, 0 of 55 on `left`, 0 of 1 on `up`).
    The proposal is then CHECKED against the drawn legs and any pair
    that really crosses is dropped. So the kept set is always genuinely
    non-crossing, and the floor it gives is never optimistic -- which is
    the failure that mattered, a floor of 0 reported for four nets piled
    on one exit point.
    """

    def __init__(self, box, launch: Dict[str, Pt], pad: float = 0.3,
                 cache: Optional[Dict[str, Dict]] = None):
        self.box = box
        self.pad = pad
        self.launch = launch
        # the caches key on the LAUNCH point as well as the exit, so a
        # caller that rebuilds the frame with one net moved -- which is
        # every step of a source-side search -- keeps the other nets'
        # work instead of paying O(N^2) crossings again per candidate
        if cache is None:
            cache = {}
        self._legs = cache.setdefault('legs', {})
        self._x = cache.setdefault('x', {})

    def leg(self, n: str, m) -> List[Pt]:
        """The polyline the corridor must draw for this net: launch to
        the escape's exit, around the array rather than through it."""
        pt = m if isinstance(m, tuple) else m.exit_pt
        lp = self.launch[n]
        key = (n, round(lp[0], 4), round(lp[1], 4),
               round(pt[0], 4), round(pt[1], 4), self.pad)
        hit = self._legs.get(key)
        if hit is None:
            hit = around_box_path(self.launch[n], pt, self.box, self.pad)
            self._legs[key] = hit
        return hit

    def axis(self, grp: Sequence[str], sel: Dict[str, Move]) -> Pt:
        """Unit vector ACROSS the bundle -- perpendicular to where it is
        on average going."""
        dx = dy = 0.0
        for n in grp:
            dx += sel[n].exit_pt[0] - self.launch[n][0]
            dy += sel[n].exit_pt[1] - self.launch[n][1]
        h = math.hypot(dx, dy) or 1.0
        return (-dy / h, dx / h)

    def launch_key(self, n: str, t: Pt) -> float:
        return self.launch[n][0] * t[0] + self.launch[n][1] * t[1]

    def exit_key(self, n: str, m, t: Pt) -> float:
        pt = m if isinstance(m, tuple) else m.exit_pt
        return pt[0] * t[0] + pt[1] * t[1]

    def order(self, grp: Sequence[str], sel: Dict[str, Move],
              t: Optional[Pt] = None) -> List[str]:
        t = t or self.axis(grp, sel)
        return sorted(grp, key=lambda n: self.launch_key(n, t))

    @staticmethod
    def paths_cross(pa: Sequence[Pt], pb: Sequence[Pt]) -> bool:
        return any(_proper_cross(p, q, r, s)
                   for p, q in zip(pa, pa[1:])
                   for r, s in zip(pb, pb[1:]))

    def crosses(self, a: str, b: str, sel: Dict[str, Move]) -> bool:
        ea, eb = sel[a].exit_pt, sel[b].exit_pt
        la, lb = self.launch[a], self.launch[b]
        key = (a, round(la[0], 4), round(la[1], 4),
               round(ea[0], 4), round(ea[1], 4),
               b, round(lb[0], 4), round(lb[1], 4),
               round(eb[0], 4), round(eb[1], 4), self.pad)
        hit = self._x.get(key)
        if hit is None:
            hit = self.paths_cross(self.leg(a, sel[a]), self.leg(b, sel[b]))
            self._x[key] = hit
        return hit

    def keep(self, grp: Sequence[str], sel: Dict[str, Move],
             weight: Optional[Dict[str, float]] = None) -> List[str]:
        """The nets that can travel the corridor without diving: the
        proposal from the transverse order, pruned until no two of them
        actually cross."""
        import braid as _te
        if len(grp) < 2:
            return list(grp)
        t = self.axis(grp, sel)
        lo = self.order(grp, sel, t)
        li = {n: i for i, n in enumerate(lo)}
        tgt = sorted(grp, key=lambda n: (round(self.exit_key(n, sel[n], t),
                                               6), li[n]))
        tr = {n: i for i, n in enumerate(tgt)}
        ranks = [tr[n] for n in lo]
        if weight:
            idx = _te.lis_keep_weighted(ranks, [weight.get(n, 0.0)
                                                for n in lo])
        else:
            idx = _te.lis_keep(ranks)
        kept = [lo[i] for i in sorted(idx)]
        # prune: drop the worst offender until the set really is
        # crossing-free. The projection is a proposal, not a proof.
        while True:
            bad: Dict[str, int] = {}
            for i, a in enumerate(kept):
                for b in kept[i + 1:]:
                    if self.crosses(a, b, sel):
                        bad[a] = bad.get(a, 0) + 1
                        bad[b] = bad.get(b, 0) + 1
            if not bad:
                return kept
            worst = max(bad, key=lambda n: (bad[n], -(weight or {}).get(n, 0)))
            kept = [n for n in kept if n != worst]


def corridor_groups(choice: Dict[str, Move]) -> List[List[str]]:
    """The unit a corridor actually routes: EVERY net leaving on one
    side, not one taut-path cluster.

    Clusters answer "which nets are going the same way", which is the
    right question for CHOOSING a side. But two clusters that pick the
    same side share one channel and one permutation, so their mutual
    crossings are real and a per-cluster floor cannot see them. Measured
    at K21: three clusters (5/4/2 nets) all left, 23 crossings between
    them, and the summed per-cluster floor understated the truth by 8."""
    g: Dict[str, List[str]] = {}
    for n, m in choice.items():
        g.setdefault(m.direction, []).append(n)
    return [v for _k, v in sorted(g.items())]


def delivered_layers(choice: Dict[str, Move], groups, geo: 'Corridor',
                     tooth_layer: Dict[str, str]) -> Dict[str, str]:
    """The layer the corridor hands each net over on: its tooth layer
    if the permutation makes it a keeper, the other one if it must
    dive."""
    out: Dict[str, str] = {}
    for bus in groups:
        if not all(n in choice for n in bus):
            continue
        # among the equally-good keep sets, take the one that holds on
        # to the nets whose escape starts on their TOOTH layer -- those
        # are the ones that pair for free with staying put
        w = {n: (1.0 if choice[n].layer == tooth_layer.get(n, 'F.Cu')
                 else 0.0) for n in bus}
        kept = set(geo.keep(bus, choice, w))
        for n in bus:
            L = tooth_layer.get(n, 'F.Cu')
            out[n] = L if n in kept else _other(L)
    return out


def true_vias(choice: Dict[str, Move], groups, geo: 'Corridor',
              tooth_layer: Dict[str, str]) -> int:
    """The vias a route ACTUALLY needs, per net:

        1 if the corridor makes it dive at the tooth
      + 1 if the layer it is handed over on is not the one its escape
          starts on
      + the escape's own vias

    which gives 0 for a keeper taking a surface escape and 2 for
    everything else -- an aligned diver's dive and its escape's surface
    via being the SAME two, not four. Counting escape vias and the
    corridor floor as independent totals double-counts exactly that
    merge, and would score an aligned plan as though nothing had been
    saved."""
    dl = delivered_layers(choice, groups, geo, tooth_layer)
    n_v = 0
    for n, m in choice.items():
        if n not in dl:
            continue
        n_v += 1 if dl[n] != tooth_layer.get(n, 'F.Cu') else 0
        n_v += 1 if dl[n] != m.layer else 0
        n_v += m.vias
    return n_v


def score(choice: Dict[str, Move], groups, geo: 'Corridor',
          tooth_layer: Dict[str, str]) -> Tuple[int, int, int]:
    """(true vias, corridor via floor, layer mismatches). The first is
    what a round is judged on; the others are reported to show where it
    came from."""
    tv = true_vias(choice, groups, geo, tooth_layer)
    fl = sum(_floor(b, choice, geo) for b in groups
             if len(b) >= 2 and all(n in choice for n in b))
    dl = delivered_layers(choice, groups, geo, tooth_layer)
    mm = sum(1 for n, m in choice.items()
             if dl.get(n) and dl[n] != m.layer)
    return tv, fl, mm


def _site_blocks(site: Optional[Pt], key: Tuple, a: float, b: float,
                 tol: float = 0.05) -> bool:
    """Does a via SITE sit in the stretch [a, b] of the lane `key`? A
    via is on every layer, so a surface escape running through a row
    gap cannot pass a dogbone's site in that gap whatever layer either
    is on -- the plan let a B dogbone and an F surface escape share a
    0.65 mm gap (K15 SDQM0/SDQ15: 9 grazes on the re-fanned source)."""
    if site is None:
        return False
    axis, coord, _L = key
    along, across = (site[0], site[1]) if axis == 'row' else (site[1], site[0])
    return abs(across - coord) <= tol and a - tol <= along <= b + tol


def _conflict(m: Move, om: Move, tol: float = 0.16, strict: bool = True) -> bool:
    """Two moves that cannot both be laid: a shared lane stretch or a
    shared site -- and, `strict`, a lane matched within `tol` (half a
    fine-pitch gap) or one's via site in the other's lane. The strict
    form is the SOURCE refinement's: it re-fans real moves against stubs
    already on the board, which wander a few tens of microns off their
    gap's centreline, and a dogbone's via is on every layer (K15
    SDQM0/SDQ15: an F surface escape planned through a B dogbone's gap,
    9 grazes). The DESTINATION choice is not strict: the fanout takes
    only the plan's DIRECTION from it, never its move geometry, so the
    strict test only removes moves the plan was free to use -- applied
    there it took the restricted K19 plan from floor 12 to 20, moved
    SDQ0 from the south face to the west, split the corridor in two and
    left 5 lanes open (2026-08-30, measured after the fact: the ladder
    had been run on fanout boards recorded before the change)."""
    key, a, b = _lane_span(m)
    ok, oa, ob = _lane_span(om)
    if strict:
        same_lane = (ok[0] == key[0] and ok[2] == key[2]
                     and abs(ok[1] - key[1]) < tol)
    else:
        same_lane = ok == key
    if same_lane and a < ob and oa < b:
        return True
    if m.site is not None and _site_key(om) == _site_key(m):
        return True
    if strict and (_site_blocks(om.site, key, a, b, tol)
                   or _site_blocks(m.site, ok, oa, ob, tol)):
        return True
    return False


def lanes_free(m: Move, sel: Dict[str, Move], me: str,
               strict: bool = True) -> bool:
    """Is this move's channel and via site free, ignoring the net's own
    current claim? The same check select() applies inside its greedy
    pass (non-strict there), exposed so a later refinement cannot
    quietly propose a move that two nets would have to share."""
    return not any(_conflict(m, om, strict=strict) for other, om in sel.items()
                   if other != me)


def plan_floor(sel: Dict[str, Move], geo: 'Corridor') -> int:
    """The via floor for the WHOLE plan, corridor boundaries ignored.

    A per-corridor floor prices only the crossings inside a corridor,
    which makes any change that pushes crossings across a corridor
    boundary look free. It is not: the board is one surface, and the
    corridor split is our decomposition for search, not an accounting
    boundary. Measured at K32, moving from no crossing pricing to
    cross_weight 6 cut total crossings 241 -> 112 while the per-corridor
    via count ROSE 37 -> 45, purely because the crossings it removed
    were the ones nobody was charging for."""
    nets = list(sel)
    return 2 * (len(nets) - len(geo.keep(nets, sel)))


def _floor(bus: Sequence[str], sel: Dict[str, Move],
           geo: 'Corridor') -> int:
    """The corridor's via floor: 2 vias for every net that cannot stay
    on the layer it arrives on, i.e. everything outside the largest
    crossing-free set."""
    return 2 * (len(bus) - len(geo.keep(bus, sel)))


def refine_lis(choice: Dict[str, Move], groups, menu, geo: 'Corridor',
               free: Callable[[Move, Dict[str, Move], str], bool],
               rounds: int = 6, prefer_layer=None,
               log=None) -> Dict[str, Move]:
    """Choose exit coordinates to MAXIMISE the corridor's LIS, exactly.

    The corridor's via floor is 2*(K - LIS) of the launch->exit
    permutation, and the floor depends only on the ORDER of the exits.
    So walk the bus in launch order and solve for the longest
    non-decreasing chain of exit coordinates by dynamic programming --
    dp[c] = longest chain ending at coordinate c -- which is optimal
    for this objective. Hill-climbing on single-net flips was tried
    first and stalls: reaching the best assignment needs several nets
    to move together, so it sat at the floor it started from.

    The chain runs over SLOTS -- (exit line, layer) -- and must increase
    STRICTLY through them, which is what makes the answer physical. With
    a merely non-decreasing chain over exit coordinates the DP has a
    trivial optimum: put every net on ONE exit line, where all the ties
    count as ordered, and report LIS = K and a floor of 0. It did
    exactly that, and nothing downstream objected, because the chain
    nets were also applied WITHOUT the channel check the other nets go
    through. Measured at K21: four nets of the `down` corridor assigned
    to the single point (140.73, 68.16), and a floor of 0 that no route
    could ever realise. Strict slot order caps a line at one net per
    layer, and the chain is now applied through `free` like everything
    else, so an unrealisable chain loses its members instead of being
    reported as an achievement.
    """
    for bus in groups:
        if len(bus) < 3 or any(n not in choice for n in bus):
            continue
        side = choice[bus[0]].direction
        t = geo.axis(bus, choice)
        lo = geo.order(bus, choice, t)
        opts = {}
        for n in lo:
            seen, keep = {}, []
            # Collapse each SLOT to one representative -- but rank a
            # move that starts on the layer the corridor will deliver
            # FIRST. Ranking purely by via count kept the 0-via surface
            # move at every coordinate and threw the dive escape away,
            # which silently undid the layer alignment: SA7 had a
            # matching dogbone at cost 27.5 against its 29.0 surface,
            # and never got to keep it.
            pl = (prefer_layer or {}).get(n)

            def _rank(m, _pl=pl):
                return (0 if _pl and m.layer == _pl else 1,
                        m.vias, _length(m))

            for m in sorted(menu[n], key=_rank):
                if m.direction != side:
                    continue
                s = (round(geo.exit_key(n, m, t), 6), m.layer)
                if s not in seen:
                    seen[s] = m
                    keep.append((s, m))
            opts[n] = sorted(keep)
        if not all(opts[n] for n in lo):
            continue
        # dp over nets in launch order: longest STRICTLY increasing
        # chain of slots
        best_end = {}          # slot -> (len, net index, move)
        back = {}
        for i, n in enumerate(lo):
            cur = {}
            for s, m in opts[n]:
                bl, bi, bs = 0, None, None
                for s2, (l2, i2, _m2) in best_end.items():
                    if s2 < s and l2 > bl:
                        bl, bi, bs = l2, i2, s2
                cur[s] = (bl + 1, i, m)
                back[(i, s)] = (bi, bs)
            for s, v in cur.items():
                if s not in best_end or v[0] > best_end[s][0]:
                    best_end[s] = v
        if not best_end:
            continue
        ends = max(best_end, key=lambda s: best_end[s][0])
        chain = {}
        i, s = best_end[ends][1], ends
        while i is not None:
            chain[lo[i]] = dict(opts[lo[i]])[s]
            i, s = back[(i, s)]
        # apply: chain first, then the rest -- but every net through the
        # SAME channel check, chain members included
        trial = {n: m for n, m in choice.items() if n not in bus}
        for n in lo:
            if n in chain and free(chain[n], trial, n):
                trial[n] = chain[n]
        for n in lo:
            if n in trial:
                continue
            for s, m in opts[n]:
                if free(m, trial, n):
                    trial[n] = m
                    break
            else:
                trial[n] = choice[n]        # nothing free: keep what it had
        if len(trial) != len(choice):
            continue
        before = _floor(bus, choice, geo)
        after = _floor(bus, trial, geo)
        if after < before:
            for n in bus:
                choice[n] = trial[n]
            if log:
                log(f'  LIS: bus[{len(bus)}] floor {before} -> {after}')
    return choice


def select(menu: Dict[str, List[Move]],
           launch: Dict[str, Pt],
           only_dirs: Optional[Iterable[str]] = None,
           via_weight: float = 3.0,
           channel_weight: float = 2.0,
           keep_out=None,
           buses: Optional[Sequence[Sequence[str]]] = None,
           side_weight: float = 6.0,
           tooth_layer: Optional[Dict[str, str]] = None,
           mismatch_weight: float = 4.0,
           # 6.0 measured over the whole coherent ladder: it is the
           # only value tried that improves the WHOLE-PLAN floor at
           # every checkpoint (K11 12->10, K21 18->16, K32 46->38,
           # K47 70->62, K51 72->64). 12.0 is better at K32 and a
           # wash at K11; 24.0 regresses at K51.
           cross_weight: float = 6.0,
           align_rounds: int = 4,
           log=None, geo: Optional['Corridor'] = None
           ) -> Tuple[Dict[str, Move], List[str]]:
    """Pick one move per net. `launch[n]` is where the net enters the
    corridor, used to price how far the corridor must carry it to reach
    a move's exit point. `geo`: the order model to use for the floor,
    the LIS refinement and the layer alignment (plan_order.BraidOrder,
    the braid's own rules) instead of this module's projection.
    Returns (choice, unplaced)."""
    dirs = set(only_dirs) if only_dirs else None
    cand = {}
    for n, ms in menu.items():
        ms = [m for m in ms if dirs is None or m.direction in dirs]
        cand[n] = ms
    if geo is None:
        geo = Corridor(keep_out, launch) if keep_out else None

    def cost(n: str, m: Move) -> float:
        lx, ly = launch[n]
        if keep_out is None:
            reach = math.hypot(m.exit_pt[0] - lx, m.exit_pt[1] - ly)
        else:
            # the corridor cannot cross the array: measure the reach it
            # would actually have to travel
            reach = around_box((lx, ly), m.exit_pt, keep_out)
        # the move's own run occupies a channel INSIDE the array, which
        # is scarcer than corridor length -- weight it above `reach`
        return via_weight * m.vias + channel_weight * _length(m) + reach

    # a bus enters the destination on ONE side, certified for capacity
    # before it is committed there; deviating from it means crossing
    # your own bundle, which nothing else in this cost sees
    side = (bus_sides(menu, launch, buses, cost, geo=geo,
                      cross_weight=cross_weight, log=log)
            if buses else {})
    # filled by the alignment loop below; empty on the first pass, so
    # the penalty is inert until there is a diver set to align with
    want_layer: Dict[str, str] = {}

    # legs of the nets chosen so far, so a candidate can be charged for
    # the corridors it would cut through. Pricing this only at the BUS
    # level is not enough: bus_sides stopped sending anyone `up`, and
    # three nets then deviated there one at a time inside the greedy
    # pass -- each deviation cheap on its own, 38 inter-corridor
    # crossings between them.
    placed_legs: List[List[Pt]] = []

    def total(n: str, m: Move) -> float:
        c = cost(n, m)
        if side.get(n) and m.direction != side[n]:
            c += side_weight
        # prefer an escape that starts on the layer the corridor will
        # actually hand this net over on; a mismatch costs a via
        if want_layer.get(n) and m.layer != want_layer[n]:
            c += mismatch_weight
        if geo is not None and placed_legs and cross_weight:
            leg = geo.leg(n, m)
            c += cross_weight * sum(1 for o in placed_legs
                                    if geo.paths_cross(leg, o))
        return c

    taken: List[Move] = []          # the moves laid so far this pass

    def lane_free(m: Move) -> bool:
        return not any(_conflict(m, om, strict=False) for om in taken)

    choice: Dict[str, Move] = {}
    unplaced: List[str] = []
    # most constrained first: a net with few options must choose before
    # a net with many takes its only lane
    order = sorted(cand, key=lambda n: len(cand[n]))
    for n in order:
        best = None
        for m in sorted(cand[n], key=lambda m: total(n, m)):
            if not lane_free(m):
                continue
            best = m
            break
        if best is None:
            unplaced.append(n)
            continue
        if log and side.get(n) and best.direction != side[n]:
            log(f'  {n}: leaves its bus side {side[n]} for '
                f'{best.direction} (no room on the bus side)')
        choice[n] = best
        if geo is not None:
            placed_legs.append(geo.leg(n, best))
        taken.append(best)
        if log:
            log(f'  {n}: {best}  (of {len(cand[n])} candidates)')

    def _free(m: Move, sel: Dict[str, Move], me: str) -> bool:
        """Is this move's channel free, ignoring the net's OWN current
        claim? The LIS pass swaps one net at a time -- without
        excluding `me`, a net is blocked from changing gap by the gap
        it is already sitting in."""
        return lanes_free(m, sel, me, strict=False)

    # From here on the grouping is the CORRIDOR -- every net leaving on
    # one side -- not the taut-path cluster. The cluster chose the side
    # (above); the permutation, the via floor and the diver set all
    # belong to whatever ends up sharing that one channel, and are
    # recomputed from `choice` because the greedy pass is allowed to
    # send a net off its bus side when the side is full.
    if buses and geo:
        refine_lis(choice, corridor_groups(choice), menu, geo, _free, log=log)

    # --- fixed point: exits decide the divers, divers decide the
    # preferred escape layer, which decides the exits
    if buses and tooth_layer and geo:
        best = dict(choice)
        best_s = score(best, corridor_groups(best), geo, tooth_layer)
        if log:
            log(f'  align round 0: vias {best_s[0]}, floor {best_s[1]}, '
                f'mismatch {best_s[2]}')
        for r in range(align_rounds):
            want_layer.clear()
            want_layer.update(delivered_layers(choice, corridor_groups(choice),
                                               geo, tooth_layer))
            taken.clear()
            placed_legs.clear()
            trial: Dict[str, Move] = {}
            for n in order:
                pick = None
                for m in sorted(cand[n], key=lambda m: total(n, m)):
                    if not lane_free(m):
                        continue
                    pick = m
                    break
                if pick is None:
                    continue
                trial[n] = pick
                if geo is not None:
                    placed_legs.append(geo.leg(n, pick))
                taken.append(pick)
            if len(trial) < len(choice):
                break                      # lost a net: reject the round
            refine_lis(trial, corridor_groups(trial), menu, geo, _free,
                       prefer_layer=want_layer)
            s = score(trial, corridor_groups(trial), geo, tooth_layer)
            if log:
                log(f'  align round {r + 1}: vias {s[0]}, floor {s[1]}, '
                    f'mismatch {s[2]}')
            if s[0] < best_s[0]:        # judge on the true via count
                best, best_s = dict(trial), s
            if trial == choice:
                break
            choice = trial
        choice = best
        if log:
            log(f'  aligned: vias {best_s[0]}, floor {best_s[1]}, '
                f'mismatch {best_s[2]}')
    return choice, unplaced


def summarise(choice: Dict[str, Move]) -> str:
    by_dir, by_kind = {}, {}
    for m in choice.values():
        by_dir[m.direction] = by_dir.get(m.direction, 0) + 1
        by_kind[m.kind] = by_kind.get(m.kind, 0) + 1
    vias = sum(m.vias for m in choice.values())
    return (f'{len(choice)} placed, {vias} escape vias; '
            f'dirs ' + ','.join(f'{k}:{v}' for k, v in sorted(by_dir.items()))
            + '; kinds ' + ','.join(f'{k}:{v}'
                                    for k, v in sorted(by_kind.items())))

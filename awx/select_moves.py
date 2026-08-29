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
              cost_fn, log=None) -> Dict[str, str]:
    """One exit side per bus: the cheapest side that PASSES the
    capacity certificate. Certifying first means a side that cannot
    hold the bundle is never offered, instead of being discovered one
    unplaceable net at a time inside greedy assignment."""
    out: Dict[str, str] = {}
    for bus in buses:
        scored = []
        for d in ('left', 'right', 'up', 'down'):
            ok, why = certify(menu, bus, d)
            if not ok:
                if log:
                    log(f'  bus[{len(bus)}] {d}: REFUSED -- {why}')
                continue
            s = sum(min(cost_fn(n, m) for m in menu[n]
                        if m.direction == d) for n in bus)
            scored.append((s, d, why))
        if not scored:
            continue
        scored.sort()
        s, best, why = scored[0]
        if log:
            log(f'  bus[{len(bus)}] -> {best} ({why}, cost {s:.0f})')
        for n in bus:
            out[n] = best
    return out


def _floor(bus: Sequence[str], sel: Dict[str, Move],
           launch: Dict[str, Pt]) -> int:
    """2*(K - LIS) of the launch->exit permutation: the corridor's via
    floor for this bus."""
    import topo_emit as _te
    lo = sorted(bus, key=lambda n: launch[n][1])
    axis = 1 if sel[bus[0]].direction in ('left', 'right') else 0
    li = {n: i for i, n in enumerate(lo)}
    tgt = sorted(bus, key=lambda n: (round(sel[n].exit_pt[axis], 3), li[n]))
    tr = {n: i for i, n in enumerate(tgt)}
    ranks = [tr[n] for n in lo]
    return 2 * (len(bus) - len(_te.lis_keep(ranks)))


def refine_lis(choice: Dict[str, Move], buses, menu, launch,
               free: Callable[[Move, Dict[str, Move], str], bool],
               rounds: int = 6, log=None) -> Dict[str, Move]:
    """Choose exit coordinates to MAXIMISE the corridor's LIS, exactly.

    The corridor's via floor is 2*(K - LIS) of the launch->exit
    permutation, and the floor depends only on the ORDER of the exits.
    So walk the bus in launch order and solve for the longest
    non-decreasing chain of exit coordinates by dynamic programming --
    dp[c] = longest chain ending at coordinate c -- which is optimal
    for this objective. Hill-climbing on single-net flips was tried
    first and stalls: reaching the best assignment needs several nets
    to move together, so it sat at the floor it started from.

    Nets that are not on the chain keep the cheapest exit whose channel
    is still free; ties are fine, since two nets on one exit line but
    different layers do not cross.
    """
    for bus in buses:
        if len(bus) < 3 or any(n not in choice for n in bus):
            continue
        side = choice[bus[0]].direction
        axis = 1 if side in ('left', 'right') else 0
        lo = sorted(bus, key=lambda n: launch[n][1])
        opts = {}
        for n in lo:
            seen, keep = {}, []
            for m in sorted(menu[n], key=lambda m: (m.vias, _length(m))):
                if m.direction != side:
                    continue
                c = round(m.exit_pt[axis], 3)
                if c not in seen:
                    seen[c] = m
                    keep.append((c, m))
            opts[n] = sorted(keep)
        if not all(opts[n] for n in lo):
            continue
        # dp over nets in launch order: (chain length, coordinate)
        NEG = (-1, None, None)
        best_end = {}          # coord -> (len, net index, move)
        back = {}
        for i, n in enumerate(lo):
            cur = {}
            for c, m in opts[n]:
                bl, bi, bc = 0, None, None
                for c2, (l2, i2, _m2) in best_end.items():
                    if c2 <= c + 1e-9 and l2 > bl:
                        bl, bi, bc = l2, i2, c2
                cur[c] = (bl + 1, i, m)
                back[(i, c)] = (bi, bc)
            for c, v in cur.items():
                if c not in best_end or v[0] > best_end[c][0]:
                    best_end[c] = v
        if not best_end:
            continue
        endc = max(best_end, key=lambda c: best_end[c][0])
        chain = {}
        i, c = best_end[endc][1], endc
        while i is not None:
            chain[lo[i]] = dict(opts[lo[i]])[c]
            i, c = back[(i, c)]
        # apply: chain nets take their chain move, the rest keep the
        # cheapest still-free option
        trial = dict(choice)
        for n in lo:
            if n in chain:
                trial[n] = chain[n]
        for n in lo:
            if n in chain:
                continue
            for c, m in opts[n]:
                if free(m, trial, n):
                    trial[n] = m
                    break
        before = _floor(bus, choice, launch)
        after = _floor(bus, trial, launch)
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
           log=None) -> Tuple[Dict[str, Move], List[str]]:
    """Pick one move per net. `launch[n]` is where the net enters the
    corridor, used to price how far the corridor must carry it to reach
    a move's exit point. Returns (choice, unplaced)."""
    dirs = set(only_dirs) if only_dirs else None
    cand = {}
    for n, ms in menu.items():
        ms = [m for m in ms if dirs is None or m.direction in dirs]
        cand[n] = ms

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
    side = bus_sides(menu, launch, buses, cost, log=log) if buses else {}

    def total(n: str, m: Move) -> float:
        c = cost(n, m)
        if side.get(n) and m.direction != side[n]:
            c += side_weight
        return c

    taken_lane: Dict[Tuple, List[Tuple[float, float]]] = {}
    taken_site = set()

    def lane_free(m: Move) -> bool:
        key, a, b = _lane_span(m)
        for (u, v) in taken_lane.get(key, ()):
            if a < v and u < b:            # intervals overlap
                return False
        return True

    choice: Dict[str, Move] = {}
    unplaced: List[str] = []
    # most constrained first: a net with few options must choose before
    # a net with many takes its only lane
    order = sorted(cand, key=lambda n: len(cand[n]))
    for n in order:
        best = None
        for m in sorted(cand[n], key=lambda m: total(n, m)):
            sk = _site_key(m)
            if not lane_free(m):
                continue
            if sk is not None and sk in taken_site:
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
        _k, _a, _b = _lane_span(best)
        taken_lane.setdefault(_k, []).append((_a, _b))
        sk = _site_key(best)
        if sk is not None:
            taken_site.add(sk)
        if log:
            log(f'  {n}: {best}  (of {len(cand[n])} candidates)')

    def _free(m: Move, sel: Dict[str, Move], me: str) -> bool:
        """Is this move's channel free, ignoring the net's OWN current
        claim? The LIS pass swaps one net at a time -- without
        excluding `me`, a net is blocked from changing gap by the gap
        it is already sitting in."""
        key, a, b = _lane_span(m)
        for other, om in sel.items():
            if other == me:
                continue
            ok, oa, ob = _lane_span(om)
            if ok == key and a < ob and oa < b:
                return False
            if m.site is not None and _site_key(om) == _site_key(m):
                return False
        return True

    if buses:
        refine_lis(choice, buses, menu, launch, _free, log=log)
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

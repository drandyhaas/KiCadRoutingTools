"""sched_first.py -- the destination chosen SCHEDULE-FIRST (README TODO 12
and 13, 2026-09-11).

The recorded planner picks one escape move per net greedily (escape vias,
channel, reach, 6 per geometric crossing whatever the layers) and only
then asks the braid's schedule what that choice costs; the judge cannot
restructure the greedy's result, and a per-pair crossing price cannot see
that the two-page schedule must two-colour EVERY crossing pair at once.
Measured on the K41 chain: 121 inversions among front-born front-stub
lanes (the human's plan: 0), 13-19 swimmers, 82-131 vias for the human's
70.

This module inverts the order. Its objective is the braid's: ZERO
inversions between the launch order and the target order WITHIN EACH
PAGE (back and front alike -- a page is the layer a lane runs on, and a
same-page inversion is a crossing only a swimmer's two vias can pay
for), at the least vias and channel. Concretely:

  * the LAUNCH order is fixed by the teeth as they stand (the source is
    frozen in the one-pass chain): nets by their launch key across the
    bundle;
  * the TARGET key of a candidate move follows the braid's own target
    order round the array (the wrap): the front face by position across
    the bundle, each side face nested first-exiter-innermost (the exit
    nearest the front takes the innermost slot), the far face outermost
    of its side;
  * PER PAGE, the nets on that page must take moves whose target keys are
    non-decreasing along the launch order -- a monotone assignment, solved
    exactly by a dynamic programme over the nets in launch order (state:
    the previous placed net's move), each move priced by its escape vias,
    the tooth and berth mismatches with the page, its channel and its
    reach; a net that cannot fit the chain is SKIPPED at a swimmer's price
    (SF_SWIM) and takes its cheapest move;
  * PAGES start as the tooth layers; every skipped net is then tried on
    the other page (a dive at the tooth is one via, a swimmer two and the
    refusals it seeds), the pair of chains re-solved, the change kept when
    the total falls;
  * moves that cannot both be laid (select_moves._conflict: a shared lane
    or site, an exit point shared) are repaired by banning the later net's
    move and re-solving its page.

What comes out is verified by the braid's own planner (plan_braid: the
Schedule's pages and swimmers) exactly as the greedy's choice was; the
count of same-page inversions in the result is printed and should be 0
among the placed nets. Opt-in: SCHED_FIRST=1 in the fanout stage.
"""
from __future__ import annotations

import math
import os
from typing import Dict, List, Optional, Sequence, Tuple

import select_moves as sm
from escape_moves import Move

Pt = Tuple[float, float]

SF_SWIM = float(os.environ.get('SF_SWIM', '12.0'))    # a net left off its page's chain
SF_NOCONFLICT = int(os.environ.get('SF_NOCONFLICT', '0'))   # DIAGNOSTIC: ignore lane conflicts (the order's own ceiling)
SF_REPAIR = int(os.environ.get('SF_REPAIR', '60'))     # conflict-repair rounds (each re-solves ONE page)
SF_REPAIR_S = float(os.environ.get('SF_REPAIR_S', '12'))  # ...and their time budget per choose(), seconds
# SF_BRAID_ORDER=1 (2026-09-11, README TODO 20): the launch order is the
# braid's own (plan_braid on a provisional plan: joiners by their join
# blocks, not by the teeth's projection across the bundle -- at K15 the
# two are the REVERSE of each other for the 4 joiners and the copper
# follows the braid), and a far-face berth whose stub points along the
# spine is keyed as the braid classes it: a HEAD-ON arrival at its row,
# through the array, not a wrap round the outside.
SF_BRAID_ORDER = int(os.environ.get('SF_BRAID_ORDER', '0'))
# SF_FAR_HEADON=1: a far-face berth keyed as a head-on arrival at its row
# (the braid's class for an east-pointing stub). Measured at K15 with the
# braid's launch order: the braid then SPLIT the corridor (SA9/SA7 and
# SCAS in corridors of their own, cross-corridor dives), 20-22 vias for
# 14. Off, a far-face berth wraps round the outside, outermost.
SF_FAR_HEADON = int(os.environ.get('SF_FAR_HEADON', '0'))
# SF_STAYERS=1 (2026-09-11, README TODO 20, the human's rule): the monotone
# chain binds only the STAYERS -- nets that keep their tooth layer to a
# berth on that same layer -- and a net that does not fit its layer's
# chain becomes a one-change DIVER: a berth on the OTHER layer, priced at
# its real cost (the berth's escape via plus one change), with no order
# constraint at all (two divers cross by diving at different points; the
# residue that cannot is judged afterwards, one_dive). Before this, every
# net sat in a per-page chain and a misfit was "skipped" at SF_SWIM and
# dumped on its cheapest berth wherever that lay (K28: SCKE1/SA6 at the
# far end of the up face, 14 same-layer crossings from two nets).
SF_STAYERS = int(os.environ.get('SF_STAYERS', '0'))
# SF_JOINT=1 (2026-09-11): BOTH chains chosen at once. A dynamic programme
# over the nets in launch order whose state is (last key on the front
# chain, last key on the back chain), keys binned at SF_BIN mm: every net
# takes a berth on the front chain, a berth on the back chain, or swims;
# the two chains are monotone TOGETHER, which the sequential choice (the
# front chain first, the divers from what is left) cannot see -- K28:
# twelve stayers chosen first left divers landing in scrambled order
# (SDQM1 1st -> 14th, SDQ9 2nd -> 15th), 11 swimmers, 56 vias for a plan
# the one-change check scored 0; the human's divers land in launch order.
SF_JOINT = int(os.environ.get('SF_JOINT', '0'))
SF_BIN = float(os.environ.get('SF_BIN', '0.2'))
# ^ in the greedy's units (a via 3, a mm of channel 2, a mm of reach 1): a
# swimmer's two vias, the refusals it seeds, and the route's own chances
VIA_W = 3.0                                          # the greedy's weights, kept
CHAN_W = 2.0
LAYERS = ('F.Cu', 'B.Cu')
DIRS = {'right': (1.0, 0.0), 'left': (-1.0, 0.0), 'up': (0.0, -1.0), 'down': (0.0, 1.0)}


def _other(L: str) -> str:
    return LAYERS[1] if L == LAYERS[0] else LAYERS[0]


class Frame:
    """The bundle's frame at the destination: u along the bundle (launch
    centroid to the box centre), t across it (u's left normal); the
    box's half extents along and across."""

    def __init__(self, launch: Dict[str, Pt], box):
        x0, y0, x1, y1 = box
        self.box = box
        self.c = ((x0 + x1) / 2, (y0 + y1) / 2)
        lx = sum(p[0] for p in launch.values()) / len(launch)
        ly = sum(p[1] for p in launch.values()) / len(launch)
        ux, uy = self.c[0] - lx, self.c[1] - ly
        h = math.hypot(ux, uy) or 1.0
        self.u = (ux / h, uy / h)
        self.t = (-self.u[1], self.u[0])
        hx, hy = (x1 - x0) / 2, (y1 - y0) / 2
        self.Hu = abs(hx * self.u[0]) + abs(hy * self.u[1])
        self.Ht = abs(hx * self.t[0]) + abs(hy * self.t[1])

    def along(self, p: Pt) -> float:
        return (p[0] - self.c[0]) * self.u[0] + (p[1] - self.c[1]) * self.u[1]

    def across(self, p: Pt) -> float:
        return (p[0] - self.c[0]) * self.t[0] + (p[1] - self.c[1]) * self.t[1]

    def face(self, m: Move) -> Tuple[str, int]:
        """('front' | 'far' | 'side', sign across) of a move's face, from
        its direction's outward normal against the bundle."""
        n = DIRS[m.direction]
        d = n[0] * self.u[0] + n[1] * self.u[1]
        if d < -0.5:
            return 'front', 0
        if d > 0.5:
            return 'far', (1 if self.across(m.exit_pt) >= 0 else -1)
        s = n[0] * self.t[0] + n[1] * self.t[1]
        return 'side', (1 if s >= 0 else -1)

    def key(self, m: Move) -> float:
        """The move's place in the braid's target order (ascending across
        the bundle): the front face by its position across; a side face
        beyond the front keys, the exit nearest the front innermost; the
        far face outermost of its side, the exit nearest its corner
        innermost."""
        kind, sg = self.face(m)
        a, x = self.along(m.exit_pt), self.across(m.exit_pt)
        if kind == 'front' or (kind == 'far' and SF_FAR_HEADON):
            return x
        if kind == 'side':
            return sg * (self.Ht + 0.5 + (a + self.Hu))
        return sg * (self.Ht + 0.5 + 2 * self.Hu + 1.0 + (self.Ht - sg * x))


def monotone_assign(order: Sequence[str], cands: Dict[str, List[Tuple[Move, float, float]]],
                    swim: float, skip_cost: Optional[Dict[str, float]] = None
                    ) -> Tuple[Dict[str, Move], List[str], float]:
    """Nets in launch order; cands[n] = [(move, key, cost)]. The min-cost
    choice with non-decreasing keys along the order; a net SKIPPED off
    the chain pays its own cheapest move plus `swim` (it still takes a
    move -- the cheapest -- and swims). Returns (choice for the placed
    nets, skipped nets, total)."""
    N = len(order)
    base = [(skip_cost[n] if skip_cost is not None and n in skip_cost
             else min((c for (_m, _k, c) in cands[n]), default=0.0) + swim) for n in order]
    pre = [0.0]
    for b in base:
        pre.append(pre[-1] + b)          # pre[i] = skip cost of nets 0..i-1

    def skip(i2: int, i: int) -> float:  # nets i2+1 .. i-1 skipped
        return pre[i] - pre[i2 + 1]
    f: List[List[float]] = []
    back: List[List[Optional[Tuple[int, int]]]] = []
    for i, n in enumerate(order):
        fi, bi = [], []
        for j, (m, k, c) in enumerate(cands[n]):
            best, arg = pre[i], None             # every earlier net skipped
            for i2 in range(i):
                gap = skip(i2, i)
                for j2, (m2, k2, c2) in enumerate(cands[order[i2]]):
                    if k2 <= k + 1e-9 and f[i2][j2] + gap < best \
                            and (SF_NOCONFLICT or not sm._conflict(m2, m, strict=False)):
                        # the previous PLACED net's move must be layable
                        # beside this one: two nets on one exit lane have
                        # equal keys and are chain neighbours, so the
                        # transition sees every such collision
                        best, arg = f[i2][j2] + gap, (i2, j2)
            fi.append(best + c)
            bi.append(arg)
        f.append(fi)
        back.append(bi)
    best, arg = pre[N], None
    for i in range(N):
        for j in range(len(cands[order[i]])):
            v = f[i][j] + (pre[N] - pre[i + 1])
            if v < best:
                best, arg = v, (i, j)
    choice: Dict[str, Move] = {}
    while arg is not None:
        i, j = arg
        choice[order[i]] = cands[order[i]][j][0]
        arg = back[i][j]
    skipped = [n for n in order if n not in choice]
    return choice, skipped, best


def joint_assign(order: Sequence[str], cands: Dict[str, List[Tuple[Move, float, float, float]]],
                 skip_cost: Dict[str, float], bin_mm: float = 0.2):
    """cands[n] = [(move, key, cost on the front chain, cost on the back
    chain)]. The least-cost assignment of every net to the front chain,
    the back chain or a swim (skip_cost[n]) with both chains' keys
    non-decreasing along `order`, keys binned at `bin_mm`. Returns
    (choice, page {net: 'F.Cu' | 'B.Cu'}, skipped, total)."""
    import numpy as np
    keys = sorted({k for n in order for (_m, k, _a, _b) in cands[n]})
    if not keys:
        return {}, {}, list(order), sum(skip_cost.values())
    k0 = keys[0]
    nb = int((keys[-1] - k0) / bin_mm) + 3
    def bin_of(k): return min(nb - 1, int((k - k0) / bin_mm + 1e-9) + 1)   # bin 0 = nothing placed
    INF = 1e18
    f = np.zeros((nb, nb))               # before the first net: nothing placed
    hist = []                             # per net: (kind, arg) arrays for backtracking
    for n in order:
        # prefix minima along each axis, with their arg
        cf = np.minimum.accumulate(f, axis=0)
        af = np.zeros_like(f, dtype=np.int32)
        run = np.arange(nb, dtype=np.int32)[:, None] * np.ones((1, nb), dtype=np.int32)
        eq = f <= cf                      # where f attains the running min
        af = np.where(eq, run, 0)
        af = np.maximum.accumulate(af, axis=0)
        cb = np.minimum.accumulate(f, axis=1)
        eqb = f <= cb
        runb = np.ones((nb, 1), dtype=np.int32) * np.arange(nb, dtype=np.int32)[None, :]
        ab = np.maximum.accumulate(np.where(eqb, runb, 0), axis=1)
        g = f + skip_cost[n]
        kind = np.full((nb, nb), -1, dtype=np.int32)     # -1 skip, else move index
        side = np.zeros((nb, nb), dtype=np.int8)         # 0 front, 1 back
        for j, (m, k, cF, cB) in enumerate(cands[n]):
            b = bin_of(k)
            # front chain: state a' = b, from any state with a STRICTLY
            # smaller front key (an equal key is the same exit lane on the
            # same layer: a collision, not a chain), any back key
            if b > 0:
                cand = cf[b - 1, :] + cF
                better = cand < g[b, :]
                g[b, better] = cand[better]; kind[b, better] = j; side[b, better] = 0
            # back chain: state b' = b, any front key
            if b > 0:
                cand = cb[:, b - 1] + cB
                better = cand < g[:, b]
                g[better, b] = cand[better]; kind[better, b] = j; side[better, b] = 1
        hist.append((kind, side, af, ab))
        f = g
    a, b = np.unravel_index(int(np.argmin(f)), f.shape)
    total = float(f[a, b])
    choice: Dict[str, Move] = {}
    page: Dict[str, str] = {}
    for n, (kind, side, af, ab) in zip(reversed(order), reversed(hist)):
        j = int(kind[a, b])
        if j < 0:
            continue
        choice[n] = cands[n][j][0]
        if side[a, b] == 0:
            page[n] = 'F.Cu'; a = int(af[a - 1, b])   # predecessor's front key (< this one)
        else:
            page[n] = 'B.Cu'; b = int(ab[a, b - 1])
    skipped = [n for n in order if n not in choice]
    return choice, page, skipped, total


def _setup(st, menu: Dict[str, List[Move]]):
    """What every entry of this module shares (choose and judge): the nets
    with a menu, the bundle's frame at the destination, the launch order
    (nets by their launch key across the bundle) and the cost of a move on
    a page -- escape vias, tooth and berth mismatches with the page, the
    channel and the reach, in the greedy's units."""
    launch: Dict[str, Pt] = st['launch']
    tooth: Dict[str, str] = st['tooth0']
    box = st['dgrid'].bbox
    names = [n for n in launch if menu.get(n)]
    fr = Frame({n: launch[n] for n in names}, box)
    order = sorted(names, key=lambda n: fr.across(launch[n]))

    def cost(n: str, m: Move, page: str) -> float:
        v = m.vias + (1 if tooth.get(n, 'F.Cu') != page else 0) + (1 if m.layer != page else 0)
        reach = sm.around_box(launch[n], m.exit_pt, box)
        return VIA_W * v + CHAN_W * sm._length(m) + reach
    return names, fr, order, cost


def judge(st, moves: Dict[str, Optional[Move]], bin_mm: Optional[float] = None) -> dict:
    """The planner's OWN verdict on one assignment -- a plan, or the berths
    a fanout LAID matched to moves (fanout_from_plan.achieved_move): the
    least cost of putting every net on the front chain, the back chain or
    a swim (joint_assign with a single candidate per net, pages free), i.e.
    the objective choose() minimises, evaluated on given moves. A laid
    fanout and its plan are thereby judged on ONE scale, which is what
    "equivalent to the plan" means (2026-09-11): a berth the engine walked
    a gap along its face costs nothing here if the chains stay monotone,
    and a berth laid at the exact gap on the wrong side of a neighbour is
    a swimmer. Nets whose move is None (no copper) are left out and
    reported in 'unlaid'. Returns {'cost', 'page', 'skipped', 'vias',
    'n', 'unlaid'} -- 'vias' the planner's count: escape vias, one per
    tooth or berth off its page, two per swimmer."""
    menu = {n: [m] for n, m in moves.items() if m is not None}
    names, fr, order, cost = _setup(st, menu)
    tooth = st['tooth0']
    cands = {n: [(menu[n][0], fr.key(menu[n][0]),
                  cost(n, menu[n][0], 'F.Cu'), cost(n, menu[n][0], 'B.Cu'))] for n in order}
    skip = {n: min(cands[n][0][2], cands[n][0][3]) + SF_SWIM for n in order}
    choice, page, skipped, total = joint_assign(order, cands, skip, bin_mm or SF_BIN)
    vias = (sum(menu[n][0].vias for n in order)
            + sum(1 for n in choice if tooth.get(n, 'F.Cu') != page[n])
            + sum(1 for n in choice if choice[n].layer != page[n])
            + 2 * len(skipped))
    return {'cost': total, 'page': page, 'skipped': skipped, 'vias': vias,
            'n': len(order), 'unlaid': [n for n, m in moves.items() if m is None]}


def choose(st, menu: Dict[str, List[Move]], log=None, rounds: int = 3,
           launch_order: Optional[Sequence[str]] = None, fixed=None, learned=None):
    """The schedule-first destination choice on a plan state (fanout_from_plan
    .plan_state). `launch_order`: the braid's own launch order for these
    nets (SF_BRAID_ORDER), oriented here to match the frame's `across`.
    `fixed` {net: move signature}: berths the engine has LAID EXACTLY on an
    earlier pass -- each such net's menu is that one move, and the engine's
    verdict outranks the model: two fixed moves are never a conflict here.
    Without it the re-plan after a refusal moved many nets at once and met
    new refusals every pass (K41: 5-9 per pass, 8 passes, never converged).
    Returns (choice, report lines)."""
    import itertools, time
    import source_realize as _sr
    learned = learned or set()      # {frozenset({sig_a, sig_b})}: pairs the ENGINE refused together
    launch: Dict[str, Pt] = st['launch']
    tooth: Dict[str, str] = st['tooth0']
    box = st['dgrid'].bbox
    fixed = dict(fixed or {})
    if fixed:
        import source_realize as _sr
        menu = dict(menu)
        for n, sig in list(fixed.items()):
            hit = [m for m in menu.get(n, []) if _sr.move_sig(m) == sig]
            if hit:
                menu[n] = hit[:1]
            else:
                del fixed[n]
    names, fr, order, cost = _setup(st, menu)
    rep: List[str] = []
    if launch_order:
        lo = [n for n in launch_order if n in menu and menu[n]]
        if len(lo) >= 2:
            # the braid's frame may be the mirror of this one: orient its
            # order by the majority of pairs against `across`
            pos = {n: i for i, n in enumerate(lo)}
            conc = sum(1 for a, b in itertools.combinations(lo, 2)
                       if (fr.across(launch[a]) < fr.across(launch[b])) == (pos[a] < pos[b]))
            if conc * 2 < len(lo) * (len(lo) - 1) / 2:
                lo.reverse()
            for n in [n for n in names if n not in pos]:
                x = fr.across(launch[n])
                at = 0
                for i, m in enumerate(lo):
                    if fr.across(launch[m]) <= x:
                        at = i + 1
                lo.insert(at, n)
            rep.append(f'  schedule-first: launch order from the braid (SF_BRAID_ORDER), '
                       f'{sum(1 for i, n in enumerate(order) if lo[i] != n)} nets moved against the teeth\'s projection')
            order = lo
    pos = {n: i for i, n in enumerate(order)}

    banned: Dict[str, set] = {n: set() for n in names}
    cache: Dict[tuple, tuple] = {}

    def cands_for(n: str, page: str, stay: Optional[bool] = None):
        """The net's candidate moves on `page`: all of them, or (SF_STAYERS)
        `stay` True the berths on the page's layer, False the others."""
        out = [(m, fr.key(m), cost(n, m, page)) for m in menu[n]
               if id(m) not in banned[n] and (stay is None or (m.layer == page) == stay)]
        out.sort(key=lambda t: t[1])
        return out

    def solve_page(P: str, grp: List[str]):
        key = (P, tuple(grp), frozenset((n, i) for n in grp for i in banned[n]))
        if key not in cache:
            if SF_STAYERS:
                # the chain over same-layer berths only; leaving it costs
                # the cheapest other-layer berth (a diver: its own escape
                # via plus the one change, both already in cost())
                stay = {n: cands_for(n, P, True) for n in grp}
                dive = {n: min((c for (_m, _k, c) in cands_for(n, P, False)), default=None) for n in grp}
                skip = {n: (dive[n] if dive[n] is not None
                            else min((c for (_m, _k, c) in stay[n]), default=0.0) + SF_SWIM)
                        for n in grp}
                cache[key] = monotone_assign(grp, stay, SF_SWIM, skip_cost=skip)
            else:
                cache[key] = monotone_assign(grp, {n: cands_for(n, P) for n in grp}, SF_SWIM)
        return cache[key]

    def solve_divers(P: str, dv: List[str]):
        """SF_STAYERS=2: the page's divers in a monotone chain of their
        own over the OTHER layer's berths, a misfit priced at a swimmer's
        two extra vias -- the braid as it stands holds only a crossing-
        free set of divers on its back page and makes the rest swim
        (K28: 16 divers with free keys, 127 crossings, 11 swimmers at 3
        changes each, 57 vias for a plan the one-change check scores 0)."""
        key = ('divers', P, tuple(dv), frozenset((n, i) for n in dv for i in banned[n]))
        if key not in cache:
            cache[key] = monotone_assign(dv, {n: cands_for(n, P, False) for n in dv}, VIA_W * 2)
        return cache[key]

    def solve(page: Dict[str, str]):
        choice: Dict[str, Move] = {}
        skipped: List[str] = []
        total = 0.0
        if SF_JOINT:
            key = ('joint', frozenset((n, i) for n in names for i in banned[n]))
            if key not in cache:
                cands = {n: [(m, fr.key(m), cost(n, m, 'F.Cu'), cost(n, m, 'B.Cu'))
                             for m in menu[n] if id(m) not in banned[n]] for n in order}
                skip = {n: min((min(a, b) for (_m, _k, a, b) in cands[n]), default=0.0) + SF_SWIM
                        for n in order}
                cache[key] = joint_assign(order, cands, skip, SF_BIN)
            ch, pg, sk, tot = cache[key]
            choice.update(ch)
            for n, P in pg.items():
                page[n] = P
            skipped += sk
            total += tot
        for P in ([] if SF_JOINT else LAYERS):
            grp = [n for n in order if page[n] == P]
            if not grp:
                continue
            ch, sk, tot = solve_page(P, grp)
            choice.update(ch)
            if SF_STAYERS >= 2 and sk:
                # the stayers' chain priced each diver at its cheapest
                # other-layer berth; the divers' own chain re-prices them
                dv_cost = {n: min((c for (_m, _k, c) in cands_for(n, P, False)), default=None) for n in sk}
                tot -= sum(c for c in dv_cost.values() if c is not None)
                dv = [n for n in sk if dv_cost[n] is not None]
                if dv:
                    ch2, sk2, tot2 = solve_divers(P, dv)
                    choice.update(ch2)
                    tot += tot2
                    sk = [n for n in sk if n not in ch2]
            skipped += sk
            total += tot
        # a skipped net takes the cheapest move that can be laid beside
        # every placed one (it still swims; it must not also collide)
        placed = list(choice.values())
        for n in sorted(skipped, key=lambda n: pos[n]):
            cs = cands_for(n, page[n], False) if SF_STAYERS else []
            cs = cs or cands_for(n, page[n])
            if not cs:
                continue
            ok = [t for t in cs if SF_NOCONFLICT
                  or not any(sm._conflict(t[0], m, strict=False) for m in placed)]
            pick = min(ok or cs, key=lambda t: t[2])[0]
            choice[n] = pick
            placed.append(pick)
        return choice, skipped, total

    def conflicts(choice: Dict[str, Move]) -> Dict[str, List[str]]:
        bad: Dict[str, List[str]] = {}
        if SF_NOCONFLICT:
            return bad
        sig = {n: _sr.move_sig(choice[n]) for n in choice} if learned else {}
        for a, b in itertools.combinations([n for n in order if n in choice], 2):
            if a in fixed and b in fixed:
                continue
            if (learned and frozenset((sig[a], sig[b])) in learned) \
                    or sm._conflict(choice[a], choice[b], strict=False):
                bad.setdefault(a, []).append(b)
                bad.setdefault(b, []).append(a)
        return bad

    def repair(page, choice, skipped, total):
        """Every pair of chosen moves that cannot both be laid (any two
        nets, either page, skipped or not): the net in the most such
        pairs (the later one on a tie) loses its move and its page is
        re-solved, until none remain or the rounds run out."""
        for _k in range(SF_REPAIR):
            bad = conflicts(choice)
            if not bad:
                return choice, skipped, total, True
            if time.time() - t_start > SF_REPAIR_S:
                break
            # a fixed net keeps its move: the other side of the pair loses
            free = [n for n in bad if n not in fixed] or list(bad)
            worst = max(free, key=lambda n: (len(bad[n]), pos[n]))
            banned[worst].add(id(choice[worst]))
            rep.append(f'  schedule-first: {worst} cannot be laid beside {bad[worst]} '
                       f'-- loses {choice[worst]}')
            choice, skipped, total = solve(page)
        return choice, skipped, total, False

    t_start = time.time()
    page = {n: tooth.get(n, 'F.Cu') for n in names}
    choice, skipped, total = solve(page)
    if fixed:
        rep.append(f'  schedule-first: {len(fixed)} berth(s) fixed as laid on the earlier pass')
    rep.append(f'  schedule-first: pages by tooth: {len(names) - len(skipped)} on chains, '
               f'{len(skipped)} skipped {skipped}, cost {total:.1f}')
    clean = True
    for _r in range(0 if (SF_STAYERS or SF_JOINT) else rounds):
        # conflicts first, then a page flip for each net still skipped
        # (a flip is judged on the repaired chains; before 2026-09-11 the
        # flips ran BEFORE the repair, so a net the repair skipped never
        # had its trial: K15 SDQ15)
        choice, skipped, total, clean = repair(page, choice, skipped, total)
        improved = False
        for n in list(skipped):
            trial = dict(page)
            trial[n] = _other(page[n])
            ch2, sk2, tot2 = solve(trial)
            if tot2 < total - 1e-9:
                page, choice, skipped, total = trial, ch2, sk2, tot2
                improved = True
                rep.append(f'  schedule-first: {n} -> page {page[n][0]}: {len(sk2)} skipped {sk2}, cost {tot2:.1f}')
        if not improved:
            break
    choice, skipped, total, clean = repair(page, choice, skipped, total)
    if not clean:
        rep.append(f'  schedule-first: conflicts REMAIN after {SF_REPAIR} repair rounds / '
                   f'{time.time() - t_start:.0f} s: {sorted(conflicts(choice))}')
    # the result's same-page inversions, the objective
    inv = 0
    for i, a in enumerate(order):
        for b in order[i + 1:]:
            if a in choice and b in choice and page[a] == page[b] \
                    and fr.key(choice[a]) > fr.key(choice[b]) + 1e-9:
                inv += 1
    rep.append(f'  schedule-first: {len(choice)} placed, {len(skipped)} skipped {skipped}, '
               f'pages F {sum(1 for n in choice if page[n] == "F.Cu")} / B '
               f'{sum(1 for n in choice if page[n] == "B.Cu")}, same-page inversions {inv}, '
               f'cost {total:.1f}')
    if SF_STAYERS:
        stay = [n for n in choice if choice[n].layer == page[n]]
        div = [n for n in choice if choice[n].layer != page[n]]
        inv_s = sum(1 for i, a in enumerate(order) for b in order[i + 1:]
                    if a in stay and b in stay and page[a] == page[b]
                    and fr.key(choice[a]) > fr.key(choice[b]) + 1e-9)
        rep.append(f'  schedule-first (SF_STAYERS): {len(stay)} stayers '
                   f'(F {sum(1 for n in stay if page[n] == "F.Cu")} / B {sum(1 for n in stay if page[n] == "B.Cu")}), '
                   f'{len(div)} one-change divers, stayer-stayer inversions {inv_s}; '
                   f'vias {sum(choice[n].vias for n in choice) + len(div)}')
    if os.environ.get('SF_PLAN'):
        # DIAGNOSTIC (2026-09-11): a plan from a file {net: {kind, direction,
        # layer, exit:[x,y]}} replaces the choice, each move matched in the
        # net's menu -- to lay a plan found OUTSIDE this module (an exact
        # ILP optimum, a hand plan) through the same fanout and braid
        import json
        ext = json.load(open(os.environ['SF_PLAN']))
        n_hit = 0
        for n, d in ext.items():
            for m in menu.get(n, []):
                if (m.kind == d['kind'] and m.direction == d['direction'] and m.layer == d['layer']
                        and abs(m.exit_pt[0] - d['exit'][0]) < 1e-3 and abs(m.exit_pt[1] - d['exit'][1]) < 1e-3):
                    choice[n] = m; n_hit += 1
                    break
            else:
                rep.append(f'  schedule-first: SF_PLAN move for {n} not in its menu: {d}')
        rep.append(f'  schedule-first: SF_PLAN {os.path.basename(os.environ["SF_PLAN"])}: {n_hit}/{len(ext)} moves taken from the file')
    if log:
        for line in rep:
            log(line)
    return choice, rep

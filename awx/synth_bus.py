#!/usr/bin/env python3
"""synth_bus.py -- generated bus articles with KNOWN ANSWERS (#622).

Every number in this campaign comes from ONE bench (`fb_t2q_fresh`, an H3
to a DDR3) graded against ONE human layout, so the ladder and
`judge_gate.py` see a comparison but never an OPTIMUM. This writes bus
problems whose optimum is known by construction, so "how far off is the
plan" becomes a number rather than a ranking.

    python3 synth_bus.py OUT.kicad_pcb --k 8 --pattern sorted
    python3 synth_bus.py OUT.kicad_pcb --k 15 --pattern blocks --blocks 2
    python3 synth_bus.py OUT.kicad_pcb --k 28 --pattern shuffle --seed 3

What it writes: a 2-layer `.kicad_pcb` with two BGA arrays (`SU1` source,
`SD1` destination) facing each other across a channel, K two-pad nets
between them in a chosen PIN PATTERN, optional foreign parts, and a
sidecar `<stem>.truth.json` carrying the planted permutation and the
optimum computed from it. `make_bench.py` then prepares it for the chain
exactly as it prepares a corpus pair (source fanned out, DRC floor
stamped, coherent ladder written).

THE TRUTH MODEL, and what it is exact for
-----------------------------------------
Bus balls are drawn from the `--depth` columns nearest the channel;
at the default depth 1 every used ball is PERIPHERAL on the facing face,
so each escapes straight into the channel on F with no via, and both of a
lane's ends are on F.  Then, in the channel:

* two lanes CROSS iff their order is inverted between the two faces
  (this is the crossing graph of the permutation -- a permutation graph);
* two lanes that cross must be on opposite layers where they cross;
* a lane that is wholly on F pays 0 vias; a lane wholly on B pays exactly
  2 (one at each end, because both its pads are on F).

There are THREE answers, and they bracket each other. Every case carries
all three, and `--self-test` asserts the bracket on every pattern.

1. LOWER BOUND, at any K: ``2 * (K - LIS(pi))``. LIS is the longest
   increasing subsequence, which for a permutation graph IS the maximum
   independent set -- the largest set of lanes that can share layer F.
   Every other lane leaves F and must come back, at 2 vias. Valid for any
   permutation, and still valid with an obstacle in the channel (an
   obstacle can only add vias).

2. WHOLE-LANE OPTIMUM: the best solution in which a lane keeps one layer
   end to end. Those are exactly the proper 2-colourings of the crossing
   graph, so it is
   ``2 * sum over components of min(|part A|, |part B|)`` -- and it does
   not exist at all when the crossing graph has an odd cycle.

3. EXACT OPTIMUM (`exact_dp`), for K up to the `--dp-cap`: the best over
   ALL two-layer routings, mid-channel layer changes included, by a DP
   over the crossing events. This is the answer the whole-lane model
   cannot reach, and it is the one to grade against.

Measured by `--self-test`: on the planted-bipartite families (`sorted`,
`blocks`, `interleave`, `riffle`) the whole-lane optimum and the exact
optimum AGREE -- mid-channel changes buy nothing there, which is what
makes those cases clean planted optima. On `reversed` the LIS bound is
TIGHT at every K tested (4, 6, 8, 9, 12), so ``2*(K-1)`` is the answer
even though the crossing graph is a clique and the whole-lane model
returns nothing. On `shuffle` all three can differ: K=15 seed 1 measures
LIS bound 20, whole-lane none, exact 24.

| pattern | crossing graph | answer |
|---|---|---|
| `sorted` | empty | **0 vias** |
| `blocks b=2` (sizes a, b) | complete bipartite K(a,b) | **2*min(a,b)** |
| `interleave` | the de-interleave's sparse graph | whole-lane = exact |
| `riffle` | bipartite by construction, random-looking | whole-lane = exact |
| `reversed` | complete K(K) | **2*(K-1)**, from the DP |
| `shuffle` | whatever the seed gives | the DP, else the bracket |

What the model does NOT price: the source fanout's own vias (fixed by
the engine before the chain starts -- the driver MEASURES them off the
fanout board and adds them), any berth via the chain chooses to pay at
the destination, and realization slack (the corpus bench measures that at
~6 vias whoever routes it).  `--depth > 1` deliberately breaks the
peripheral precondition to make an escape field: the driver then reads
each tooth's real layer off the board and prices the weighted colouring
instead, and the answer is exact only for the ends as the engine laid
them.  `--obstacle-w/-h` puts a through-hole part IN the channel: the
three answers are then the CLEAR-CHANNEL ones, so the excess over them is
what the chain pays for the obstruction.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys

# JEDEC ball rows: the alphabet without I, O, Q, S, X, Z (they read as
# 1/0/2/5 on a package), then doubled -- AA, AB, ... -- exactly as KiCad's
# Package_BGA footprints number a tall array.
ROW_LETTERS = [c for c in 'ABCDEFGHJKLMNPRTUVWY']

PITCH = 0.8          # ball pitch, mm (the bench's DDR3 is 0.8)
PAD = 0.4            # ball pad diameter, mm
VERSION = 20260206   # the board format the bench carries


def row_name(i: int) -> str:
    """Row `i` (0-based) as a BGA row letter: A..Y, then AA..AY, ..."""
    n = len(ROW_LETTERS)
    if i < n:
        return ROW_LETTERS[i]
    return ROW_LETTERS[i // n - 1] + ROW_LETTERS[i % n]


def uid(*parts) -> str:
    """A deterministic uuid: the board must be byte-identical per seed."""
    h = hashlib.md5('|'.join(str(p) for p in parts).encode()).hexdigest()
    return f'{h[:8]}-{h[8:12]}-{h[12:16]}-{h[16:20]}-{h[20:32]}'


# --- the pin patterns -------------------------------------------------------

def pattern_perm(kind: str, K: int, seed: int = 0, blocks: int = 2,
                 inversions: int | None = None):
    """pi[i] = the DESTINATION rank of the lane whose SOURCE rank is i.

    Every pattern is a function of (kind, K, seed, blocks, inversions)
    alone -- no clock, no board, no global state."""
    if kind == 'sorted':
        return list(range(K))
    if kind == 'reversed':
        return list(range(K - 1, -1, -1))
    if kind == 'blocks':
        # b contiguous blocks of source ranks, delivered in REVERSE block
        # order, each block's internal order kept. b=2 is the classic
        # bus swap: the crossing graph is complete bipartite.
        b = max(2, blocks)
        cut = [round(K * i / b) for i in range(b + 1)]
        groups = [list(range(cut[i], cut[i + 1])) for i in range(b)]
        order = [n for g in reversed(groups) for n in g]
        pi = [0] * K
        for rank, n in enumerate(order):
            pi[n] = rank
        return pi
    if kind == 'interleave':
        # the DDR-like de-interleave: even source ranks land in the first
        # half of the destination in order, odd ranks in the second half.
        ev = [i for i in range(K) if i % 2 == 0]
        od = [i for i in range(K) if i % 2 == 1]
        pi = [0] * K
        for rank, n in enumerate(ev + od):
            pi[n] = rank
        return pi
    if kind == 'riffle':
        # RANDOM but 2-increasing by construction: each lane is dealt to
        # one of two decks, each deck keeps its source order at the
        # destination, and the two decks are riffled together. Every deck
        # is a crossing-free set, so the crossing graph is bipartite and
        # the case carries an exact optimum however random it looks --
        # the only random family here that does.
        import random
        rng = random.Random(seed * 7919 + K)
        deck = [rng.random() < 0.5 for _ in range(K)]
        a_src = [i for i in range(K) if not deck[i]]
        b_src = [i for i in range(K) if deck[i]]
        slots = [False] * len(a_src) + [True] * len(b_src)
        rng.shuffle(slots)
        ia = ib = 0
        order = []
        for s in slots:
            if s:
                order.append(b_src[ib])
                ib += 1
            else:
                order.append(a_src[ia])
                ia += 1
        pi = [0] * K
        for rank, n in enumerate(order):
            pi[n] = rank
        return pi
    if kind == 'shuffle':
        import random
        rng = random.Random(seed * 1000003 + K)
        pi = list(range(K))
        if inversions is None:
            rng.shuffle(pi)
            return pi
        # a CONTROLLED inversion count: random adjacent transpositions,
        # each accepted only while it moves the count towards the target
        want = inversions
        cur = 0
        guard = 0
        while cur != want and guard < 200 * K * K:
            guard += 1
            i = rng.randrange(K - 1)
            up = pi[i] < pi[i + 1]
            if (up and cur < want) or (not up and cur > want):
                pi[i], pi[i + 1] = pi[i + 1], pi[i]
                cur += 1 if up else -1
        return pi
    raise SystemExit(f'unknown pattern {kind!r}')


# --- the truth --------------------------------------------------------------

def crossing_edges(order_src, order_dst):
    """Pairs (i, j) of lane IDS whose order is inverted between the two
    faces -- the crossing graph. `order_*` are lists of lane ids in the
    order they appear along each face."""
    rs = {n: i for i, n in enumerate(order_src)}
    rd = {n: i for i, n in enumerate(order_dst)}
    ids = list(rs)
    out = []
    for a in range(len(ids)):
        for b in range(a + 1, len(ids)):
            i, j = ids[a], ids[b]
            if (rs[i] - rs[j]) * (rd[i] - rd[j]) < 0:
                out.append((i, j))
    return out


def lis_len(seq):
    """Longest strictly increasing subsequence (patience sorting)."""
    import bisect
    tails = []
    for v in seq:
        k = bisect.bisect_left(tails, v)
        if k == len(tails):
            tails.append(v)
        else:
            tails[k] = v
    return len(tails)


def two_colour(ids, edges):
    """(bipartite, colour, components). `colour[n]` in {0, 1}; components
    is a list of [part0, part1] lane-id lists (an isolated lane is its own
    component with an empty part 1)."""
    adj = {n: [] for n in ids}
    for i, j in edges:
        adj[i].append(j)
        adj[j].append(i)
    colour, comps, ok = {}, [], True
    for s in ids:
        if s in colour:
            continue
        colour[s] = 0
        stack, part = [s], ([s], [])
        while stack:
            u = stack.pop()
            for v in adj[u]:
                if v not in colour:
                    colour[v] = 1 - colour[u]
                    part[colour[v]].append(v)
                    stack.append(v)
                elif colour[v] == colour[u]:
                    ok = False
        comps.append([sorted(part[0]), sorted(part[1])])
    return ok, colour, comps


def optimum(ids, edges, tooth_layer=None, berth_layer=None):
    """The corridor optimum in VIAS, and how it was established.

    `tooth_layer` / `berth_layer` map lane id -> 'F.Cu' | 'B.Cu' (the ends
    as the engine actually laid them). Omitted = the peripheral case, both
    ends on F. A lane paged on layer p pays [tooth != p] + [berth != p],
    so a component is priced twice (the colouring and its swap) and the
    cheaper is taken -- exact, because a connected component's proper
    2-colouring is unique up to that swap."""
    ok, colour, comps = two_colour(ids, edges)
    tl = tooth_layer or {n: 'F.Cu' for n in ids}
    bl = berth_layer or {n: 'F.Cu' for n in ids}

    def price(n, page):
        return (0 if tl.get(n, 'F.Cu') == page else 1) + \
               (0 if bl.get(n, 'F.Cu') == page else 1)

    if not ok:
        return None, comps, 'NOT BIPARTITE (an odd cycle: a lane must change layer mid-channel)'
    tot = 0
    for p0, p1 in comps:
        a = sum(price(n, 'F.Cu') for n in p0) + sum(price(n, 'B.Cu') for n in p1)
        b = sum(price(n, 'B.Cu') for n in p0) + sum(price(n, 'F.Cu') for n in p1)
        tot += min(a, b)
    return tot, comps, 'exact (min over the proper 2-colourings)'


def lower_bound(order_src, order_dst):
    """2 * (K - LIS): at most LIS lanes can share layer F (a set of lanes
    with no crossing is an increasing subsequence), and every other lane
    leaves F and must come back, so it pays at least 2 vias. Holds for any
    permutation, bipartite or not -- and still holds with an obstacle in
    the channel, which can only ADD vias."""
    rd = {n: i for i, n in enumerate(order_dst)}
    seq = [rd[n] for n in order_src]
    return 2 * (len(seq) - lis_len(seq)), lis_len(seq)


# --- the PLANNER'S OWN model, solved exactly ---------------------------------
#
# The three answers above price the PROBLEM. This prices the MODEL: the best
# plan `pages_first.py` is able to express at all. Its variables are a page
# per net and an end per net, and its one hard rule is that two nets on the
# same page are never inverted; a net that fits neither page is left to SWIM
# at a fixed price (`PLAN_PAGES_SWIM`, 100 vias as shipped).
#
# Two lanes are inverted exactly when they cross, so "a page" is a
# crossing-free set is an INCREASING SUBSEQUENCE of the destination ranks in
# source order. The model's optimum is therefore: cover the lanes with two
# increasing subsequences, pay each lane its page's end-mismatch price, and
# pay `swim` for every lane left over. That is an exact O(K^3) DP -- state is
# the last destination rank placed on each page, because nothing earlier can
# constrain a later lane -- so it answers at K=51 in milliseconds where
# `exact_dp` stops at 22.
#
# Why it is worth a column. `planner gap = plan - optimum` cannot say WHY the
# plan is off, and the two causes want opposite work:
#
#     MODEL error  = the best plan the model can express  - the true optimum
#     SEARCH error = the plan the planner found           - that best plan
#
# If the model error dominates, more solving is wasted (and, at a swim price
# of 100 vias against a swimmer's true cost of 2, actively harmful -- the
# solve will distort every other choice to page one more lane). If the search
# error dominates, a better solver pays. The campaign has been buying search.

def pages_model(order_src, order_dst, tooth_layer=None, berth_layer=None,
                swim=100.0, mismatch=1.0):
    """The exact optimum of the two-page model, in the model's own units.

    `swim` is the model's price for a lane it cannot page (vias);
    `mismatch` the price of one end whose layer differs from its page --
    both as `pages_first` spells them. Returns a dict:

        cost      the model's objective at its optimum (vias)
        page      lane id -> 'F.Cu' | 'B.Cu' | 'swim'
        n_f/n_b   lanes on each page, n_swim lanes left over
        paged     n_f + n_b -- and with a swim price above the span of the
                  end prices this is MAXIMAL, so it equals the Greene
                  number lambda1 + lambda2 (the self-test checks that)
        true_lb   a LOWER bound on what a routing of this plan can cost in
                  real vias: the paged lanes at their end prices, plus 2 a
                  swimmer (a lane whose ends share a layer must leave it
                  and come back). NOT the model's `cost`, which counts a
                  swimmer at `swim`.
    """
    rd = {n: i + 1 for i, n in enumerate(order_dst)}     # 1-based; 0 = empty
    tl, bl = tooth_layer or {}, berth_layer or {}
    # `swim` may be a per-lane mapping as well as a scalar. That is what
    # turns this function into a LOWER BOUND (see channel_lower_bound):
    # price each unpaged lane at the least it could possibly cost, and the
    # DP's answer is below every routing's.
    swim_of = (swim.get if hasattr(swim, 'get') else (lambda n, _d=None: swim))

    def price(n, page):
        return mismatch * ((0 if tl.get(n, 'F.Cu') == page else 1) +
                           (0 if bl.get(n, 'F.Cu') == page else 1))

    def c_min(n):
        """The least a lane that does NOT hold one page can cost: it makes
        at least one layer change, and an even number of them when its two
        ends share a layer."""
        return 2.0 if tl.get(n, 'F.Cu') == bl.get(n, 'F.Cu') else 1.0

    # state (a, b) -> (cost, true, choices): the highest destination rank
    # already placed on page F and on page B. A later lane is legal on a
    # page iff its rank is above that page's last, so the two numbers are
    # the whole of the state.
    cur = {(0, 0): (0.0, 0.0, ())}
    for n in order_src:
        v = rd[n]
        pf, pb = price(n, 'F.Cu'), price(n, 'B.Cu')
        nxt = {}
        for (a, b), (c, t, ch) in cur.items():
            # the CHARGED price (what the model pays) and the TRUE floor
            # (what such a lane cannot cost less than) are different
            # numbers: a lane that leaves its tooth's layer and must come
            # back to the same one pays 2, and 1 when its ends differ.
            opts = [((a, b), swim_of(n, 0.0), c_min(n), 'swim')]
            if v > a:
                opts.append(((v, b), pf, pf, 'F.Cu'))
            if v > b:
                opts.append(((a, v), pb, pb, 'B.Cu'))
            for key, dc, dt, tag in opts:
                cc = c + dc
                old = nxt.get(key)
                if old is None or cc < old[0] - 1e-12:
                    nxt[key] = (cc, t + dt, ch + ((n, tag),))
        cur = nxt
    (cost, true_lb, ch) = min(cur.values(), key=lambda v: v[0])
    page = dict(ch)
    n_f = sum(1 for p in page.values() if p == 'F.Cu')
    n_b = sum(1 for p in page.values() if p == 'B.Cu')
    n_s = sum(1 for p in page.values() if p == 'swim')
    return dict(cost=cost, page=page, n_f=n_f, n_b=n_b, n_swim=n_s,
                paged=n_f + n_b, true_lb=true_lb)


def channel_lower_bound(order_src, order_dst, tooth_layer=None,
                        berth_layer=None):
    """A LOWER BOUND on the channel's exact optimum, at ANY K.

    `exact_dp` is exact but costs 2**(free lanes), so at K=41 or 51 --
    the rungs that matter -- there has never been a number to compare the
    routed board against. This gives one, in milliseconds, and it is a
    bound rather than an estimate:

    THE ARGUMENT. Any routing splits the lanes into three sets: A, the
    lanes that hold layer F for the whole channel; B, those that hold
    layer B; and S, the rest. Then

      * A is crossing-free and so is B -- two lanes that share a layer
        everywhere cannot cross -- so each is an increasing subsequence of
        the destination ranks in source order;
      * a lane in A costs exactly `price(n, F)` and one in B exactly
        `price(n, B)`, its end mismatches and nothing else;
      * a lane in S changes layer at least once, so it costs at least 1,
        and at least 2 when its two ends share a layer (an odd number of
        changes could not return it).

    So the routing's cost is at least the minimum, over all valid (A, B),
    of that sum -- which is exactly `pages_model` with every lane's swim
    price set to its own floor. The O(K^3) DP computes that minimum
    exactly, so the number is a true bound and not a heuristic.

    WHAT IT IS A BOUND FOR, said plainly: the channel-confined homotopy
    class, where every inverted pair crosses once and no other pair
    crosses. A lane that reaches its pad THROUGH or AROUND an array is
    outside it and can legitimately beat this -- which is what the
    ladder's `thru` column counts. On a board with `thru > 0` read it as
    the bound for the lanes that stayed in the channel.

    Returns the `pages_model` dict; `cost` is the bound.
    """
    tl, bl = tooth_layer or {}, berth_layer or {}
    floor = {n: (2.0 if tl.get(n, 'F.Cu') == bl.get(n, 'F.Cu') else 1.0)
             for n in order_src}
    return pages_model(order_src, order_dst, tooth_layer, berth_layer,
                       swim=floor)


def rsk_shape(seq):
    """The first two rows of the RSK insertion tableau's shape.

    Greene's theorem: the largest union of k increasing subsequences of a
    sequence is the sum of the k largest parts of that shape. So
    `lambda1 + lambda2` is, independently of the DP above, the most lanes
    two pages can hold -- and lambda1 alone is the LIS. Used only by the
    self-test, as a second opinion computed a different way."""
    import bisect
    # NOTE bisect_left and bisect_right agree on a permutation (all values
    # distinct), so a mutation between them is equivalent and the self-test
    # cannot tell them apart -- recorded rather than papered over.
    rows = []
    for v in seq:
        cur = v
        for r in rows:
            k = bisect.bisect_left(r, cur)
            if k == len(r):
                r.append(cur)
                cur = None
                break
            r[k], cur = cur, r[k]
        if cur is not None:
            rows.append([cur])
    return [len(r) for r in rows]


# --- the EXACT optimum, over pages AND mid-channel layer changes -----------
#
# The whole-lane model above prices only solutions where a lane keeps one
# layer end to end, so it returns nothing at all for a crossing graph with
# an odd cycle (`reversed`, most `shuffle` seeds) -- exactly the cases
# where the interesting answer lives. This prices the general model.

DP_INF = 1 << 20


def crossing_events(ids, s, d):
    """The inverted pairs IN THE ORDER THEY CROSS along the channel.

    Lane n is the straight segment from cross-axis coordinate `s[n]` at the
    source face to `d[n]` at the destination face, so lanes i and j cross at

        t = (s_i - s_j) / ((s_i - s_j) - (d_i - d_j))       in (0, 1)

    which is defined exactly when the pair is inverted. Returns a list of
    (index_i, index_j) into `ids`, sorted by t.

    TIES ARE THE CAVEAT. On a perfectly regular article several pairs cross
    at the same t (`reversed` crosses every pair at the midpoint), and the
    order among them is then this function's deterministic tie-break rather
    than a fact about the problem: a real router perturbs them. So the DP
    below is exact for ONE crossing order -- see `exact_dp`."""
    out = []
    for a in range(len(ids)):
        for b in range(a + 1, len(ids)):
            i, j = ids[a], ids[b]
            ds, dd = s[i] - s[j], d[i] - d[j]
            if ds * dd >= 0:
                continue                      # not inverted: no crossing
            out.append(((ds / (ds - dd)), a, b))
    out.sort()                                # ties break on (a, b): deterministic
    return [(a, b) for _t, a, b in out]


def _relax_hamming(cost, K):
    """min over y of cost[y] + hamming(x, y), in place over the hypercube.

    Hamming is a sum over coordinates, so the distance transform is
    SEPARABLE: one exact 1-D pass per bit, in any order, is the whole
    answer. The 1-D transform on {0, 1} with cost |a - b| is just
    `min(c, c_flipped + 1)`."""
    try:
        import numpy as np
    except ImportError:
        np = None
    if np is not None and isinstance(cost, np.ndarray):
        v = cost.reshape([2] * K) if K else cost
        for b in range(K):
            other = np.flip(v, axis=b)
            np.minimum(v, other + 1, out=v)
        return v.reshape(-1)
    for b in range(K):
        bit = 1 << b
        for x in range(len(cost)):
            if x & bit:
                continue
            a, c = cost[x], cost[x | bit]
            if a + 1 < c:
                cost[x | bit] = a + 1
            elif c + 1 < a:
                cost[x] = c + 1
    return cost


def exact_dp(ids, s, d, tooth_layer=None, berth_layer=None, cap=22, fixed=None):
    """The minimum via count over ALL two-layer routings of this channel
    whose inverted pairs each cross once, in `crossing_events`' order --
    mid-channel layer changes included.

    State = which layer each lane is on (one bit a lane, 0 = F). Walk the
    crossing events in order; at each, states where the crossing pair
    shares a layer are forbidden; between events a lane may flip for one
    via, which is the Hamming distance transform above. The teeth fix the
    entry state and the berths the exit state.

    Cost: K * 2**K per event, so it is capped. Returns (vias, how) or
    (None, why-not).

    What it is exact FOR, said plainly: one crossing order and one
    homotopy class (every inverted pair crosses exactly once, no
    non-inverted pair crosses at all). A router that takes a lane the long
    way round an array is outside the model and can beat it -- which is
    what the harness's `thru` column exists to catch.

    `fixed` (lane id -> 'F.Cu' | 'B.Cu') PINS those lanes to one layer for
    the whole channel. That is what a whole-lane PLAN is, so
    `exact_dp(fixed=the model's pages) - exact_dp()` is the exact price of
    planning in whole lanes -- the MODEL error, measured in the same units
    and the same homotopy class as the optimum it is compared to. A pinned
    lane is not a variable, so the walk runs over the FREE lanes only and
    the cap applies to those: a plan that pages most of a K=41 bus is
    answered even though the free problem at K=41 is not.
    """
    K = len(ids)
    if K == 0:
        return 0, 'exact (no lanes)'
    tl = tooth_layer or {}
    bl = berth_layer or {}
    fx = {n: l for n, l in (fixed or {}).items() if l in ('F.Cu', 'B.Cu')}
    free = [n for n in ids if n not in fx]
    F = len(free)
    if F > cap:
        return None, (f'not computed ({F} free lane(s) > the 2**n cap of {cap}'
                      + (f'; {K - F} of {K} were pinned)' if fx else ')'))
    try:
        import numpy as np
    except ImportError:
        np = None
    if np is None and F > 12:
        return None, f'not computed (no numpy, and {F} free lanes > 12 in pure python)'
    bit = {n: i for i, n in enumerate(free)}
    pin = {n: (1 if fx[n] == 'B.Cu' else 0) for n in fx}
    # the pinned lanes' own cost is fixed: one via for each end whose layer
    # is not the page they are held on
    base = 0
    for n, p in fx.items():
        base += (0 if tl.get(n, 'F.Cu') == p else 1)
        base += (0 if bl.get(n, 'F.Cu') == p else 1)
    start = sum(1 << bit[n] for n in free if tl.get(n, 'F.Cu') != 'F.Cu')
    goal = sum(1 << bit[n] for n in free if bl.get(n, 'F.Cu') != 'F.Cu')
    events = crossing_events(ids, s, d)
    N = 1 << F
    how = ('exact over pages AND mid-channel changes, for the '
           'straight-line crossing order')
    if fx:
        how = (f'exact for the {F} free lane(s), with {K - F} pinned to their '
               'page for the whole channel')
    # each event is one of three kinds once the pins are known
    plan_events = []
    for (a, b) in events:
        na, nb = ids[a], ids[b]
        pa, pb = pin.get(na), pin.get(nb)
        if pa is not None and pb is not None:
            if pa == pb:
                return None, ('two lanes pinned to the same page cross: the '
                              'plan is not a plan')
            continue                      # different pages: always legal
        if pa is not None:
            plan_events.append(('one', bit[nb], pa))
        elif pb is not None:
            plan_events.append(('one', bit[na], pb))
        else:
            plan_events.append(('two', bit[na], bit[nb]))
    if F == 0:
        return base, how
    if np is not None:
        x = np.arange(N, dtype=np.int32)
        cost = np.zeros(N, dtype=np.int32)
        for i in range(F):
            cost += ((x >> i) & 1) ^ ((start >> i) & 1)
        for kind, i, j in plan_events:
            if kind == 'two':
                bad = (((x >> i) & 1) == ((x >> j) & 1))
            else:
                bad = (((x >> i) & 1) == j)     # j is the pinned layer bit
            cost = np.where(bad, DP_INF, cost)
            if cost.min() >= DP_INF:
                return None, 'no two-layer routing of this crossing order exists'
            cost = _relax_hamming(cost, F)
        for i in range(F):
            cost += ((x >> i) & 1) ^ ((goal >> i) & 1)
        best = int(cost.min())
    else:
        cost = [bin(v ^ start).count('1') for v in range(N)]
        for kind, i, j in plan_events:
            bi = 1 << i
            for v in range(N):
                if (bool(v & bi) == bool(v & (1 << j))) if kind == 'two' \
                        else (bool(v & bi) == bool(j)):
                    cost[v] = DP_INF
            if min(cost) >= DP_INF:
                return None, 'no two-layer routing of this crossing order exists'
            _relax_hamming(cost, F)
        best = min(c + bin(v ^ goal).count('1') for v, c in enumerate(cost))
    if best >= DP_INF:
        return None, 'no two-layer routing of this crossing order exists'
    return best + base, how


# --- is the objective pointing the right way? -------------------------------
#
# `pages_model` answers "what is the best plan this model can express". This
# answers the question behind it: **does the model's objective RANK plans the
# way the truth does?** A solver can only ever find the best point of the
# objective it is given, so an objective that ranks wrongly cannot be fixed
# by more search -- and the campaign's headline finding (solving the K41 plan
# to proven optimality routes twelve vias WORSE) is exactly that shape.
#
# The measurement: build a pool of plans the model considers feasible, score
# each one both ways -- the model's own objective, and the exact via count of
# the best routing consistent with it -- and correlate. No routing, no chain,
# milliseconds. On a case with a known optimum the pool also brackets it.

def cap_floor(ids, s, d, tooth_layer=None, berth_layer=None, cap=None):
    """The channel's minimum vias when EVERY lane is capped at `cap` of them.

    WHY THIS EXISTS. `channel_lower_bound` prices a swimmer at its own
    floor -- 2 when its ends share a layer -- and that is a correct LOWER
    bound. It is routinely read as a target ("if every net paid at most
    two, the board would be 78"), and that reading is a different claim
    altogether: that a two-via-per-net routing EXISTS. It often does not.
    Measured on uniform-random permutations, a clean channel stops being
    two-via-feasible at about K=16 and is never feasible from K=20 up,
    while the same channel is always feasible at four. So the planner's
    directive "no net may need more than two vias" is not a hard rule of
    the channel -- it is a property a particular arrangement may or may
    not have, and at the campaign's K it is only reachable by leaving the
    channel (un-crossing pairs by going around an array).

    THE MODEL is the same one `exact_dp` walks -- a layer per lane at each
    crossing, opposite layers at a crossing, ends pinned to the pads --
    written as a MILP so a per-lane bound can be added. With `cap=None` it
    must therefore agree with `exact_dp` exactly, and the self-test checks
    that on every small case; the two use entirely different algorithms,
    so an agreement is real evidence.

    Returns (vias, per-lane vias, status). `vias` is None when the cap
    makes the system infeasible -- which is a RESULT, and the strongest
    one available: no realization of this channel meets the cap.
    """
    try:
        import numpy as np
        from scipy.optimize import milp, Bounds, LinearConstraint
        from scipy.sparse import coo_matrix
    except ImportError as e:            # pragma: no cover - environment
        return None, {}, f'not computed (needs scipy/numpy: {e})'
    tl, bl = tooth_layer or {}, berth_layer or {}
    events = crossing_events(ids, s, d)
    sites = {n: [] for n in ids}
    for ei, (a, b) in enumerate(events):
        sites[ids[a]].append(ei)
        sites[ids[b]].append(ei)
    idx = {}
    for n in ids:
        for ei in sites[n]:
            idx[(n, ei)] = len(idx)
    ny = len(idx)
    rows, lo, hi = [], [], []

    def add(co, a, b):
        rows.append(co); lo.append(a); hi.append(b)

    for ei, (a, b) in enumerate(events):          # a crossing: opposite layers
        add({idx[(ids[a], ei)]: 1, idx[(ids[b], ei)]: 1}, 1, 1)
    nv = ny
    dcost = []
    lane_d = {n: [] for n in ids}
    INF = float('inf')
    for n in ids:
        chain = sites[n]
        p0 = 1 if tl.get(n, 'F.Cu') == 'B.Cu' else 0
        p1 = 1 if bl.get(n, 'F.Cu') == 'B.Cu' else 0
        for pos, ei in enumerate(chain):
            v = idx[(n, ei)]
            dv = nv; nv += 1; dcost.append(1.0); lane_d[n].append(dv)
            if pos == 0:
                add({dv: 1, v: -1}, -p0, INF); add({dv: 1, v: 1}, p0, INF)
            else:
                u = idx[(n, chain[pos - 1])]
                add({dv: 1, v: -1, u: 1}, 0, INF)
                add({dv: 1, v: 1, u: -1}, 0, INF)
        dv = nv; nv += 1; dcost.append(1.0); lane_d[n].append(dv)
        if chain:
            v = idx[(n, chain[-1])]
            add({dv: 1, v: -1}, -p1, INF); add({dv: 1, v: 1}, p1, INF)
        else:
            add({dv: 1}, abs(p1 - p0), INF)
        if cap is not None:
            add({x: 1 for x in lane_d[n]}, 0, float(cap))
    cvec = np.concatenate([np.zeros(ny), np.asarray(dcost, float)])
    integ = np.concatenate([np.ones(ny), np.zeros(len(dcost))])
    ri, ci, vi = [], [], []
    for i, co in enumerate(rows):
        for k, v in co.items():
            ri.append(i); ci.append(k); vi.append(float(v))
    A = coo_matrix((vi, (ri, ci)), shape=(max(1, len(rows)), nv)).tocsr()
    res = milp(cvec, constraints=LinearConstraint(A, np.asarray(lo, float),
                                                  np.asarray(hi, float)),
               integrality=integ, bounds=Bounds(0, 1))
    if res.x is None:
        # a missing solution is not evidence of infeasibility: say which
        why = ('infeasible' if getattr(res, 'status', None) == 2
               else f'no solution (status {getattr(res, "status", "?")})')
        return None, {}, why
    per = {n: int(round(sum(res.x[x] for x in lane_d[n]))) for n in ids}
    return int(round(float(cvec @ res.x))), per, 'optimal'


def cap_sat_feasible(ids, s, d, tooth_layer=None, berth_layer=None, cap=2,
                     workers=8, det_time=120.0):
    """Is a routing with at most `cap` vias a lane POSSIBLE? -- CP-SAT.

    The same question as `cap_floor`, as satisfiability rather than
    optimisation, because that is what scales: at K=48 the MILP ran over
    thirty minutes and was killed, while this proves the real bench
    channel infeasible in about a minute (and a second on an easier one).
    Use `cap_floor` when the COST matters, this when only the answer does.

    Returns one of 'FEASIBLE', 'INFEASIBLE', 'UNKNOWN', or a 'not
    computed...' string. **UNKNOWN is not a negative** -- it means the
    deterministic budget ran out, and the self-test only compares the two
    backends where this one is decisive.

    Budgeted in DETERMINISTIC time, never wall clock (the campaign rule:
    a clock budget makes a slow machine answer DIFFERENTLY, not later).
    """
    try:
        from ortools.sat.python import cp_model
    except ImportError as e:            # pragma: no cover - environment
        return f'not computed (needs ortools: {e})'
    tl, bl = tooth_layer or {}, berth_layer or {}
    events = crossing_events(ids, s, d)
    sites = {n: [] for n in ids}
    for ei, (a, b) in enumerate(events):
        sites[ids[a]].append(ei)
        sites[ids[b]].append(ei)
    m = cp_model.CpModel()
    y = {(n, k): m.NewBoolVar(f'y{n}_{k}') for n in ids for k in sites[n]}
    for ei, (a, b) in enumerate(events):        # a crossing: opposite layers
        ya, yb = y[(ids[a], ei)], y[(ids[b], ei)]
        m.AddBoolOr([ya, yb])
        m.AddBoolOr([ya.Not(), yb.Not()])
    for n in ids:
        p0 = 1 if tl.get(n, 'F.Cu') == 'B.Cu' else 0
        p1 = 1 if bl.get(n, 'F.Cu') == 'B.Cu' else 0
        seq = [p0] + [y[(n, k)] for k in sites[n]] + [p1]
        fixed, free = 0, []
        for i in range(len(seq) - 1):
            a, b = seq[i], seq[i + 1]
            if isinstance(a, int) and isinstance(b, int):
                fixed += int(a != b)
                continue
            dv = m.NewBoolVar(f'd{n}_{i}')
            if isinstance(a, int):
                m.Add(dv == (b if a == 0 else 1 - b))
            elif isinstance(b, int):
                m.Add(dv == (a if b == 0 else 1 - a))
            else:
                # dv = a XOR b, i.e. a XOR b XOR (NOT dv) is true. Spelt by
                # hand this is easy to invert, and an inverted dv bounds the
                # NON-transitions -- which every alternating assignment
                # satisfies, so every infeasible case reads FEASIBLE. That
                # bug is why the self-test below compares the two backends.
                m.AddBoolXOr([a, b, dv.Not()])
            free.append(dv)
        if free:
            m.Add(sum(free) <= cap - fixed)
        elif fixed > cap:
            return 'INFEASIBLE'
    sol = cp_model.CpSolver()
    sol.parameters.num_workers = workers
    sol.parameters.max_deterministic_time = det_time
    st = sol.Solve(m)
    return {cp_model.OPTIMAL: 'FEASIBLE', cp_model.FEASIBLE: 'FEASIBLE',
            cp_model.INFEASIBLE: 'INFEASIBLE'}.get(st, 'UNKNOWN')


def cap_survey(seeds=5, ks=(8, 10, 12, 14, 16, 20, 24, 32), cap=2):
    """Is "no lane over `cap` vias" reachable at all, as K grows?

    The answer is the SOLVER STATUS, not the presence of a solution: a
    missing answer can mean a limit was hit, and reading that as "no such
    routing exists" would be the strongest possible claim drawn from the
    weakest possible evidence. Anything that is neither optimal nor
    infeasible is printed as itself.
    """
    print(f'clean channel, uniform-random permutation, every pad on F.Cu; '
          f'{seeds} seed(s) a rung')
    print(f'{"K":>4}  {"<=%d vias a lane" % cap:22s}  {"at %d" % (2 * cap):>14s}'
          f'  {"LIS":>5}')
    worst = 0
    rows = []
    for K in ks:
        col, four, lis_ = [], [], []
        for seed in range(seeds):
            pi = pattern_perm('shuffle', K, seed=seed)
            ids = list(range(K))
            s_ = {n: float(n) for n in ids}
            d_ = {n: float(pi[n]) for n in ids}
            _lb, l_ = lower_bound(ids, sorted(ids, key=lambda n: d_[n]))
            lis_.append(l_)
            v, _p, st = cap_floor(ids, s_, d_, cap=cap)
            if st.startswith('not computed'):
                print(f'  {st}')
                return 0
            col.append('Y' if v is not None else
                       ('N' if st == 'infeasible' else '?'))
            v4, _p4, st4 = cap_floor(ids, s_, d_, cap=2 * cap)
            four.append('Y' if v4 is not None else
                        ('N' if st4 == 'infeasible' else '?'))
        if 'N' in col:
            first_fail = K if worst == 0 else worst
            worst = first_fail
        rows.append((K, col, four))
        print(f'{K:4d}  {" ".join(col):22s}  {" ".join(four):>14s}'
              f'  {sum(lis_) / len(lis_):5.1f}')
    # every sentence below is read off the table just printed, so it cannot
    # drift from it -- the first draft of this summary asserted "4 always
    # holds" on a run whose own K=32 column had already refuted it
    never = [K for K, col, _f in rows if 'Y' not in col]
    ever = [K for K, col, _f in rows if 'Y' in col]
    f_bad = [K for K, _c, four in rows if 'N' in four]
    print(f'\n  Y = a routing with at most {cap} vias a lane EXISTS; '
          f'N = the solver proved none does.')
    if worst:
        print(f'  First rung with a failing seed: K={worst}.', end=' ')
    if never:
        print(f'No seed succeeds from K={min(never)} up'
              f'{" (highest rung with any feasible seed: K=%d)" % max(ever) if ever else ""}.')
    else:
        print('Every rung tested had a feasible seed.')
    print(f'  So the directive is a property of the ARRANGEMENT, not a rule '
          f'of the channel.')
    if f_bad:
        print(f'  A cap of {2 * cap} is not free either -- it failed at '
              f'K={", ".join(str(k) for k in f_bad)}.')
    else:
        print(f'  A cap of {2 * cap} held at every rung tested.')
    print(f'  Above the failing K the directive is reachable only by LEAVING '
          f'the channel --')
    print(f'  un-crossing pairs by going around an array, which this model '
          f'does not contain.')
    return 0


def plan_pool(order_src, order_dst, tooth_layer=None, berth_layer=None,
              n=400, seed=0, swims=(2.0, 3.0, 6.0, 20.0, 100.0)):
    """Distinct model-feasible plans: lane id -> 'F.Cu' | 'B.Cu' | 'swim'.

    Random walks in source order (each lane takes a uniformly random legal
    option) give the spread; the model's optimum at each price in `swims`
    is added so the pool always contains the points a solver would actually
    reach. Deterministic in `seed`."""
    import random
    rng = random.Random(seed * 104729 + len(order_src))
    rd = {nm: i + 1 for i, nm in enumerate(order_dst)}
    out = {}
    for _ in range(n):
        a = b = 0
        plan = {}
        for nm in order_src:
            v = rd[nm]
            opts = ['swim'] + (['F.Cu'] if v > a else []) + (['B.Cu'] if v > b else [])
            pick = rng.choice(opts)
            if pick == 'F.Cu':
                a = v
            elif pick == 'B.Cu':
                b = v
            plan[nm] = pick
        out[tuple(sorted(plan.items()))] = plan
    for sp in swims:
        m = pages_model(order_src, order_dst, tooth_layer, berth_layer, swim=sp)
        out[tuple(sorted(m['page'].items()))] = m['page']
    return list(out.values())


def plan_objective(plan, tooth_layer=None, berth_layer=None,
                   swim=100.0, mismatch=1.0):
    """`pages_first`'s objective for one plan, in vias, with the ends fixed.

    That is the whole of it on a peripheral bus: the per-net berth and tooth
    terms are constant when the ends cannot move, so what the solver is
    ranking by is the end-mismatch price plus the swimmers."""
    tl, bl = tooth_layer or {}, berth_layer or {}
    tot = 0.0
    for nm, page in plan.items():
        if page == 'swim':
            tot += swim
            continue
        tot += mismatch * ((0 if tl.get(nm, 'F.Cu') == page else 1) +
                           (0 if bl.get(nm, 'F.Cu') == page else 1))
    return tot


def spearman(xs, ys):
    """Rank correlation, average ranks for ties. None when either side is
    constant -- a constant column has no ranking to agree with, and
    reporting 0 for it would read as "uncorrelated" when it is "undefined"."""
    def ranks(v):
        order = sorted(range(len(v)), key=lambda i: v[i])
        r = [0.0] * len(v)
        i = 0
        while i < len(order):
            j = i
            while j + 1 < len(order) and v[order[j + 1]] == v[order[i]]:
                j += 1
            avg = (i + j) / 2.0 + 1
            for k in range(i, j + 1):
                r[order[k]] = avg
            i = j + 1
        return r
    if len(xs) < 2 or len(set(xs)) < 2 or len(set(ys)) < 2:
        return None
    rx, ry = ranks(xs), ranks(ys)
    mx, my = sum(rx) / len(rx), sum(ry) / len(ry)
    num = sum((a - mx) * (b - my) for a, b in zip(rx, ry))
    dx = sum((a - mx) ** 2 for a in rx) ** 0.5
    dy = sum((b - my) ** 2 for b in ry) ** 0.5
    return None if not dx or not dy else num / (dx * dy)


def judge_report(ids, s, d, order_src, order_dst, tooth_layer=None,
                 berth_layer=None, n=400, seed=0, swims=(2.0, 3.0, 6.0, 20.0, 100.0),
                 cap=22):
    """For each candidate swim price: how well does the objective at that
    price rank the pool against the TRUE cost of each plan, and what does
    the plan it would choose really cost?"""
    pool = plan_pool(order_src, order_dst, tooth_layer, berth_layer,
                     n=n, seed=seed, swims=swims)
    truth, keep = [], []
    for plan in pool:
        fixed = {nm: l for nm, l in plan.items() if l != 'swim'}
        v, _how = exact_dp(ids, s, d, tooth_layer=tooth_layer,
                           berth_layer=berth_layer, fixed=fixed, cap=cap)
        if v is not None:
            truth.append(v)
            keep.append(plan)
    if not keep:
        return None
    best_true = min(truth)
    rows = []
    for sp in swims:
        objs = [plan_objective(p, tooth_layer, berth_layer, swim=sp) for p in keep]
        # the plan the objective would choose, and what it really costs --
        # ties broken by the WORST truth, because a solver picking among
        # equal-objective plans gives no guarantee about which it returns
        lo = min(objs)
        tied = [t for o, t in zip(objs, truth) if o == lo]
        chosen = max(tied)
        rows.append({'swim': sp, 'rho': spearman(objs, truth),
                     'chosen': chosen, 'regret': chosen - best_true,
                     'ties': len(tied),
                     # THE number: among the plans this objective cannot
                     # tell apart, how far apart are they really? A solver
                     # returns one of them and no search can prefer the
                     # right one, so this is the floor on what solving this
                     # objective can guarantee -- and it is why proving
                     # optimality moved the routed board by twelve vias.
                     'tie_spread': max(tied) - min(tied),
                     'tie_best': min(tied)})
    return {'pool': len(keep), 'best_true': best_true, 'rows': rows}


# --- the board --------------------------------------------------------------

HEAD = '''(kicad_pcb
\t(version {version})
\t(generator "synth_bus")
\t(generator_version "10.0")
\t(general
\t\t(thickness 1.6)
\t\t(legacy_teardrops no)
\t)
\t(paper "A4")
\t(layers
\t\t(0 "F.Cu" signal)
\t\t(2 "B.Cu" signal)
\t\t(9 "F.Adhes" user "F.Adhesive")
\t\t(11 "B.Adhes" user "B.Adhesive")
\t\t(13 "F.Paste" user)
\t\t(15 "B.Paste" user)
\t\t(5 "F.SilkS" user "F.Silkscreen")
\t\t(7 "B.SilkS" user "B.Silkscreen")
\t\t(1 "F.Mask" user)
\t\t(3 "B.Mask" user)
\t\t(17 "Dwgs.User" user "User.Drawings")
\t\t(19 "Cmts.User" user "User.Comments")
\t\t(25 "Edge.Cuts" user)
\t\t(27 "Margin" user)
\t\t(31 "F.CrtYd" user "F.Courtyard")
\t\t(29 "B.CrtYd" user "B.Courtyard")
\t\t(35 "F.Fab" user)
\t\t(33 "B.Fab" user)
\t)
\t(setup
\t\t(stackup
\t\t\t(layer "F.SilkS"
\t\t\t\t(type "Top Silk Screen")
\t\t\t)
\t\t\t(layer "F.Paste"
\t\t\t\t(type "Top Solder Paste")
\t\t\t)
\t\t\t(layer "F.Mask"
\t\t\t\t(type "Top Solder Mask")
\t\t\t\t(thickness 0.01)
\t\t\t)
\t\t\t(layer "F.Cu"
\t\t\t\t(type "copper")
\t\t\t\t(thickness 0.035)
\t\t\t)
\t\t\t(layer "dielectric 1"
\t\t\t\t(type "core")
\t\t\t\t(thickness 1.51)
\t\t\t\t(material "FR4")
\t\t\t\t(epsilon_r 4.5)
\t\t\t\t(loss_tangent 0.02)
\t\t\t)
\t\t\t(layer "B.Cu"
\t\t\t\t(type "copper")
\t\t\t\t(thickness 0.035)
\t\t\t)
\t\t\t(layer "B.Mask"
\t\t\t\t(type "Bottom Solder Mask")
\t\t\t\t(thickness 0.01)
\t\t\t)
\t\t\t(layer "B.Paste"
\t\t\t\t(type "Bottom Solder Paste")
\t\t\t)
\t\t\t(layer "B.SilkS"
\t\t\t\t(type "Bottom Silk Screen")
\t\t\t)
\t\t\t(copper_finish "None")
\t\t\t(dielectric_constraints no)
\t\t)
\t\t(pad_to_mask_clearance 0)
\t\t(allow_soldermask_bridges_in_footprints no)
\t\t(tenting
\t\t\t(front yes)
\t\t\t(back yes)
\t\t)
\t)
'''


def pad_block(name, lx, ly, rot, net_id, net_name, side, key, pad=PAD):
    """One ball. A net-0 ball carries NO `(net ...)` node, exactly as the
    bench's unconnected H3 balls do."""
    lay = 'F' if side == 'F' else 'B'
    # the KiCad 10 net dialect, which is the bench's: a pad names its net and
    # there is NO top-level net table (`extract_nets` synthesises the ids from
    # first appearance). A `(net N "NAME")` node here parses as no net at all.
    net = f'\n\t\t\t(net "{net_name}")' if net_id else ''
    return (f'\t\t(pad "{name}" smd circle\n'
            f'\t\t\t(at {lx:.4f} {ly:.4f}{"" if not rot else " %g" % rot})\n'
            f'\t\t\t(size {pad} {pad})\n'
            f'\t\t\t(layers "{lay}.Cu" "{lay}.Mask" "{lay}.Paste"){net}\n'
            f'\t\t\t(uuid "{uid(key, name)}")\n'
            f'\t\t)\n')


def array_block(ref, cx, cy, rows, cols, rot, side, assign, pitch=PITCH, pad=PAD,
                pad_inner=None):
    """A BGA array footprint. `assign` maps (row, col) -> (net_id, net_name);
    every other ball is net 0 -- a real package's power and ground balls,
    and an obstacle either way."""
    w, h = (cols - 1) * pitch, (rows - 1) * pitch
    key = ref
    out = [f'\t(footprint "Synth:BGA-{rows}x{cols}_P{pitch}mm"\n'
           f'\t\t(layer "{side}.Cu")\n'
           f'\t\t(uuid "{uid(key)}")\n'
           f'\t\t(at {cx:.4f} {cy:.4f}{"" if not rot else " %g" % rot})\n'
           f'\t\t(descr "synthetic BGA, {rows}x{cols} layout, {pitch}mm pitch")\n'
           f'\t\t(tags "BGA {rows * cols} {pitch}")\n'
           f'\t\t(property "Reference" "{ref}"\n'
           f'\t\t\t(at 0 {-h / 2 - 1.2:.4f} {rot:g})\n'
           f'\t\t\t(layer "{side}.SilkS")\n'
           f'\t\t\t(uuid "{uid(key, "ref")}")\n'
           f'\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1 1)\n'
           f'\t\t\t\t\t(thickness 0.15)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n'
           f'\t\t(property "Value" "SYNTH-BGA"\n'
           f'\t\t\t(at 0 {h / 2 + 1.2:.4f} {rot:g})\n'
           f'\t\t\t(layer "{side}.Fab")\n'
           f'\t\t\t(uuid "{uid(key, "val")}")\n'
           f'\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1 1)\n'
           f'\t\t\t\t\t(thickness 0.15)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n'
           f'\t\t(attr smd)\n']
    # a silkscreen box, so the renders show where the package is
    for (x1, y1, x2, y2) in [(-w / 2 - 0.5, -h / 2 - 0.5, w / 2 + 0.5, h / 2 + 0.5)]:
        out.append(f'\t\t(fp_rect\n\t\t\t(start {x1:.4f} {y1:.4f})\n'
                   f'\t\t\t(end {x2:.4f} {y2:.4f})\n'
                   f'\t\t\t(stroke\n\t\t\t\t(width 0.12)\n\t\t\t\t(type solid)\n\t\t\t)\n'
                   f'\t\t\t(fill no)\n\t\t\t(layer "{side}.SilkS")\n'
                   f'\t\t\t(uuid "{uid(key, "box")}")\n\t\t)\n')
    for r in range(rows):
        for c in range(cols):
            nid, nname = assign.get((r, c), (0, ''))
            lx = -w / 2 + c * pitch
            ly = -h / 2 + r * pitch
            out.append(pad_block(f'{row_name(r)}{c + 1}', lx, ly, rot, nid, nname,
                                 side, key,
                                 pad if nid else (pad_inner or pad)))
    out.append('\t)\n')
    return ''.join(out)


def cap_block(ref, cx, cy, side, key):
    """A 0402 two-pad foreign part -- the row of decoupling caps a real
    bench carries on the far face, an obstacle the escape must respect."""
    lay = 'F' if side == 'F' else 'B'
    out = [f'\t(footprint "Synth:C_0402"\n\t\t(layer "{lay}.Cu")\n'
           f'\t\t(uuid "{uid(key)}")\n\t\t(at {cx:.4f} {cy:.4f})\n'
           f'\t\t(attr smd)\n'
           f'\t\t(property "Reference" "{ref}"\n\t\t\t(at 0 -1 0)\n'
           f'\t\t\t(layer "{lay}.SilkS")\n\t\t\t(uuid "{uid(key, "ref")}")\n'
           f'\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 0.6 0.6)\n'
           f'\t\t\t\t\t(thickness 0.1)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n']
    for i, dx in ((1, -0.48), (2, 0.48)):
        out.append(f'\t\t(pad "{i}" smd roundrect\n\t\t\t(at {dx} 0)\n'
                   f'\t\t\t(size 0.54 0.64)\n'
                   f'\t\t\t(layers "{lay}.Cu" "{lay}.Mask" "{lay}.Paste")\n'
                   f'\t\t\t(roundrect_rratio 0.25)\n'
                   f'\t\t\t(uuid "{uid(key, i)}")\n\t\t)\n')
    out.append('\t)\n')
    return ''.join(out)


def obstacle_block(ref, cx, cy, w, h, key, pitch=1.0):
    """A foreign part sitting IN THE CHANNEL: a grid of net-0 THROUGH-HOLE
    pads filling `w` x `h` mm at `pitch`.

    Through-hole is the point. An SMD blocker is dodged by a layer change,
    which is a via the model already prices; a plated barrel blocks BOTH
    layers (CLAUDE.md: "Even unconnected through-hole pads (net_id=0)
    physically block tracks"), so the bus has no choice but to fan IN
    above and below it and fan back out -- parallel bends at the lane
    pitch, which is the geometry the session-11 island-stack finding says
    goes infeasible. Nothing about it is keyed to a bench: it is a
    rectangle of copper in the way."""
    nx = max(1, int(round(w / pitch)) + 1)
    ny = max(1, int(round(h / pitch)) + 1)
    sx = -(nx - 1) * pitch / 2
    sy = -(ny - 1) * pitch / 2
    out = [f'\t(footprint "Synth:BLOCKER_{nx}x{ny}"\n\t\t(layer "F.Cu")\n'
           f'\t\t(uuid "{uid(key)}")\n\t\t(at {cx:.4f} {cy:.4f})\n'
           f'\t\t(attr through_hole)\n'
           f'\t\t(property "Reference" "{ref}"\n\t\t\t(at 0 {-h / 2 - 1:.3f} 0)\n'
           f'\t\t\t(layer "F.SilkS")\n\t\t\t(uuid "{uid(key, "ref")}")\n'
           f'\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 0.8 0.8)\n'
           f'\t\t\t\t\t(thickness 0.12)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n'
           f'\t\t(fp_rect\n\t\t\t(start {sx - 0.4:.3f} {sy - 0.4:.3f})\n'
           f'\t\t\t(end {-sx + 0.4:.3f} {-sy + 0.4:.3f})\n'
           f'\t\t\t(stroke\n\t\t\t\t(width 0.12)\n\t\t\t\t(type solid)\n\t\t\t)\n'
           f'\t\t\t(fill no)\n\t\t\t(layer "F.SilkS")\n'
           f'\t\t\t(uuid "{uid(key, "box")}")\n\t\t)\n']
    for r in range(ny):
        for c in range(nx):
            out.append(f'\t\t(pad "{r * nx + c + 1}" thru_hole circle\n'
                       f'\t\t\t(at {sx + c * pitch:.4f} {sy + r * pitch:.4f})\n'
                       f'\t\t\t(size 0.7 0.7)\n\t\t\t(drill 0.4)\n'
                       f'\t\t\t(layers "*.Cu" "*.Mask")\n'
                       f'\t\t\t(uuid "{uid(key, r, c)}")\n\t\t)\n')
    out.append('\t)\n')
    return ''.join(out)


def build(a):
    """The board text and the truth sidecar for one case."""
    K = a.k
    depth = max(1, a.depth)
    # rows-2 usable slots a column (the corners are skipped, see face_slots)
    off = max(0, getattr(a, 'row_offset', 0) or 0)
    rows, cols = a.rows or -(-K // depth) + 2 + 2 * off, a.cols
    if (rows - 2 - 2 * off) * depth < K:
        raise SystemExit(f'--rows {rows} (minus 2 corner rows and 2x{off} free) x '
                         f'--depth {depth} < K={K}: not enough non-corner balls '
                         f'on the facing face')
    pi = pattern_perm(a.pattern, K, seed=a.seed, blocks=a.blocks,
                      inversions=a.inversions)
    assert sorted(pi) == list(range(K)), 'the pattern is not a permutation'

    # Geometry: source west, destination east, the channel along +x.
    # Bus balls are drawn from the `depth` columns nearest the channel,
    # walking the facing column top to bottom first (so depth 1 is one
    # clean column and the order along the face is the ball order).
    def face_slots(n, cols_, east):
        """The n ball (row, col) slots nearest the channel, in order along
        the face. `east` = the facing side is the array's east column.

        The CORNER balls (first and last row) are deliberately left out:
        `bga_fanout.is_edge_pad` calls a ball an edge pad when it is within
        0.01 mm of ANY of the four grid lines, and a corner ball is on two
        of them -- so its escape direction is the engine's choice, not the
        channel's, and the clean "both ends on F, straight into the
        channel" precondition the truth model rests on would not hold."""
        out = []
        for d in range(depth):
            c = (cols_ - 1 - d) if east else d
            for r in range(1 + off, rows - 1 - off):
                out.append((r, c))
                if len(out) == n:
                    return out
        return out[:n]

    src_slots = face_slots(K, cols, east=True)
    dst_slots = face_slots(K, cols, east=False)
    names = [f'SYN{i:02d}' for i in range(K)]
    # lane i: source rank i -> destination rank pi[i]
    src_assign = {src_slots[i]: (i + 1, names[i]) for i in range(K)}
    dst_assign = {dst_slots[pi[i]]: (i + 1, names[i]) for i in range(K)}

    h = (rows - 1) * PITCH
    wsrc = (cols - 1) * PITCH
    sx, sy = 0.0, 0.0
    dx = wsrc / 2 + a.gap + wsrc / 2
    txt = [HEAD.format(version=VERSION)]
    # `--margin-y` walls the channel: generous by default (no escape is
    # edge-clearance bound), small enough and a lane cannot ride round an
    # array at all, which is what the channel-confined optimum assumes
    m, my = 6.0, a.margin_y
    x0, y0 = sx - wsrc / 2 - m, sy - h / 2 - my
    x1, y1 = sx + dx + wsrc / 2 + m, sy + h / 2 + my
    txt.append(f'\t(gr_rect\n\t\t(start {x0:.3f} {y0:.3f})\n'
               f'\t\t(end {x1:.3f} {y1:.3f})\n'
               f'\t\t(stroke\n\t\t\t(width 0.1)\n\t\t\t(type default)\n\t\t)\n'
               f'\t\t(fill no)\n\t\t(layer "Edge.Cuts")\n\t\t(uuid "{uid("edge")}")\n\t)\n')
    txt.append(array_block(a.src, sx, sy, rows, cols, 0, 'F', src_assign, PITCH,
                           a.pad, a.pad_inner))
    txt.append(array_block(a.dst, sx + dx, sy, rows, cols, a.dst_rot, 'F', dst_assign,
                           PITCH, a.pad, a.pad_inner))
    # foreign parts: a row of caps on the BACK face under each array, as
    # the corpus bench carries -- they take room out of the B page's
    # escape field without touching a bus net
    for i in range(a.caps):
        f = i / max(1, a.caps - 1) if a.caps > 1 else 0.5
        cy = sy - h / 2 + f * h
        txt.append(cap_block(f'C{i + 1}', sx, cy, 'B', f'cap{i}'))
        txt.append(cap_block(f'C{a.caps + i + 1}', sx + dx, cy, 'B', f'capd{i}'))
    # the part IN the corridor: the bus has to fan in around it
    obs = None
    if a.obstacle_h > 0 and a.obstacle_w > 0:
        ox = sx + wsrc / 2 + a.obstacle_x * a.gap
        oy = sy + a.obstacle_y
        txt.append(obstacle_block(a.obstacle_ref, ox, oy, a.obstacle_w,
                                  a.obstacle_h, 'obs'))
        # the room left for the bus on each side of it, inside the outline
        obs = {'w': a.obstacle_w, 'h': a.obstacle_h, 'x': ox, 'y': oy,
               'room_above': round((oy - a.obstacle_h / 2) - y0, 3),
               'room_below': round(y1 - (oy + a.obstacle_h / 2), 3)}
    txt.append(')\n')

    # --- the truth, from the planted pattern ---
    ids = list(range(K))
    order_src = list(range(K))                  # source rank order
    order_dst = sorted(ids, key=lambda i: pi[i])
    edges = crossing_edges(order_src, order_dst)
    opt, comps, how = optimum(ids, edges)
    lb, lis = lower_bound(order_src, order_dst)
    # the general optimum: pages AND mid-channel changes, from the planted
    # ranks (source rank i, destination rank pi[i]) as the straight lanes
    rank_s = {i: i for i in ids}
    rank_d = {i: pi[i] for i in ids}
    dp, dp_how = exact_dp(ids, rank_s, rank_d, cap=a.dp_cap)
    if obs and dp is not None:
        dp_how += ' -- CLEAR-CHANNEL: the obstacle is not priced'
    truth = {
        'k': K, 'pattern': a.pattern, 'seed': a.seed, 'blocks': a.blocks,
        'inversions_asked': a.inversions,
        'perm': pi, 'names': names,
        'src': a.src, 'dst': a.dst,
        'rows': rows, 'cols': cols, 'depth': depth, 'gap': a.gap,
        'row_offset': off,
        'pad': a.pad, 'pad_inner': a.pad_inner, 'margin_y': a.margin_y,
        'dst_rot': a.dst_rot, 'caps': a.caps,
        'crossings': len(edges),
        'inversions': len(edges),
        'lis': lis,
        'corridor_lower_bound': lb,
        'corridor_optimum': opt,
        'optimum_how': how,
        'corridor_dp': dp,
        'dp_how': dp_how,
        'bipartite': opt is not None,
        'exact': opt is not None and opt == lb,
        'components': [[len(p0), len(p1)] for p0, p1 in comps],
        'peripheral': depth == 1,
        'obstacle': obs,
        'channel_clear': obs is None,
    }
    # the three answers must bracket, or one of them is wrong
    assert dp is None or dp >= lb, f'dp {dp} < lower bound {lb}'
    assert dp is None or opt is None or dp <= opt, \
        f'dp {dp} > whole-lane optimum {opt} (the whole-lane solutions are a subset)'
    return ''.join(txt), truth


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out', nargs='?', help='the .kicad_pcb to write (omit with --self-test)')
    ap.add_argument('--k', type=int, default=8, help='bus width')
    ap.add_argument('--pattern', default='sorted',
                    choices=('sorted', 'reversed', 'blocks', 'interleave', 'riffle', 'shuffle'))
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--blocks', type=int, default=2)
    ap.add_argument('--inversions', type=int, default=None)
    ap.add_argument('--rows', type=int, default=None)
    ap.add_argument('--row-offset', type=int, default=0,
                    help='leave this many ball rows FREE at each end of the '
                         'facing column, above and below the bus (the array '
                         'grows to fit). The bus otherwise fills the facing '
                         'column from the first non-corner row to the last, '
                         'so a launch has nowhere along the face to move TO '
                         'and an end-of-face climb (fanout_from_plan '
                         'SRC_CLIMB_END, the group move) has no room to exist '
                         '-- the harness is inert for it. With free rows the '
                         'move is available and the planted optimum is '
                         'unchanged, which is what makes the pair a '
                         'measurement: the same case at offset 0 is the '
                         'negative control.')
    ap.add_argument('--cols', type=int, default=4)
    ap.add_argument('--depth', type=int, default=1,
                    help='how many columns deep the bus balls are drawn from '
                         '(1 = peripheral, the clean known answer)')
    ap.add_argument('--gap', type=float, default=12.0, help='channel width, mm')
    ap.add_argument('--pad', type=float, default=PAD,
                    help='ball diameter, mm (default 0.4). At 0.8 pitch the '
                         'inter-column gap is then 0.4 mm and a 0.127 track '
                         'with 0.105 clearance FITS THROUGH THE ARRAY: 0.5 '
                         'closes it (0.3 mm < 0.337 needed)')
    ap.add_argument('--pad-inner', type=float, default=None,
                    help='diameter of the UNUSED (net-0) balls, mm; default = '
                         '--pad. 0.6 at 0.8 pitch closes the array interior to '
                         'F traffic (0.3 mm gap < the 0.337 a 0.127/0.105 track '
                         'needs) WITHOUT touching the bus balls escape, which '
                         'leaves the facing column at --pad and runs outward. '
                         'That is what makes the channel-confined optimum an '
                         'optimum: a lane can no longer un-cross a partner by '
                         'reaching its ball through the array from the far side')
    ap.add_argument('--margin-y', type=float, default=6.0,
                    help='board outline margin above and below the arrays, mm. '
                         'Generous by default; a small value walls the channel '
                         'so a lane cannot ride around an array')
    ap.add_argument('--dst-rot', type=float, default=0.0)
    ap.add_argument('--caps', type=int, default=0)
    ap.add_argument('--src', default='SU1')
    ap.add_argument('--dst', default='SD1')
    ap.add_argument('--obstacle-w', type=float, default=0.0,
                    help='width (along the channel) of a THROUGH-HOLE blocker '
                         'placed IN the channel, mm. 0 = no obstacle')
    ap.add_argument('--obstacle-h', type=float, default=0.0,
                    help='its height across the channel, mm. This is the knob '
                         'that makes the case hard: the bus must fan in past '
                         'it on both sides, and K lanes need K*(track+clearance) '
                         'of room split between them')
    ap.add_argument('--obstacle-x', type=float, default=0.5,
                    help='where along the channel it sits, as a fraction of '
                         '--gap from the source array (default mid-channel)')
    ap.add_argument('--obstacle-y', type=float, default=0.0,
                    help='its offset across the channel from the bus centre, mm')
    ap.add_argument('--obstacle-ref', default='XB1')
    ap.add_argument('--dp-cap', type=int, default=22,
                    help='largest K for which the exact over-all-routings '
                         'optimum is computed. Cost is K*2**K per crossing: '
                         'measured 0.02 s at K=15, 0.8 s at 20, 4.5 s at 22, '
                         '12 s at 23 -- and K=28 is out of reach, so the big '
                         'cases are graded on the other two answers')
    ap.add_argument('--cap-survey', action='store_true',
                    help='sweep K and report whether a clean channel over a '
                         'random permutation can be routed with at most two '
                         'vias a lane at all -- the planner directive, asked '
                         'rather than assumed. Prints FEASIBLE/INFEASIBLE '
                         'from the solver status, never from a missing answer')
    ap.add_argument('--cap-survey-seeds', type=int, default=5)
    ap.add_argument('--self-test', action='store_true',
                    help='check the truth sources against each other on '
                         'every pattern and exit')
    ap.add_argument('--judge', action='store_true',
                    help='score the PLANNER\'S OBJECTIVE against the truth on '
                         'generated cases and exit -- no board, no chain, '
                         'seconds. For each candidate swim price: the rank '
                         'correlation between the objective and the real cost '
                         'of the plan, and the REGRET and DEGENERACY of the '
                         'plan that objective would choose')
    ap.add_argument('--judge-k', default='10,12,15,18',
                    help='bus widths for --judge')
    ap.add_argument('--judge-swims', default='2,3,6,20,100',
                    help='swim prices to score, in vias')
    ap.add_argument('--judge-pool', type=int, default=250,
                    help='plans sampled per case')
    ap.add_argument('--judge-jitter', type=float, default=0.30,
                    help='perturb the end positions by this much (in lane '
                         'pitches) before scoring, so no two pairs cross at '
                         'the same point. 0 uses the article exactly as drawn '
                         '-- which on a REGULAR one is degenerate: `reversed` '
                         'crosses every pair at the midpoint, so the crossing '
                         'ORDER is crossing_events\' tie-break rather than a '
                         'fact, and a spread read off it is partly an artifact '
                         '(measured at K=10: 14 vias degenerate, 6..12 across '
                         'jitter seeds). Non-zero is the honest default')
    a = ap.parse_args(argv)
    if a.cap_survey:
        return cap_survey(seeds=a.cap_survey_seeds)
    if a.self_test:
        return self_test()
    if a.judge:
        return judge_main(a)
    if not a.out:
        ap.error('an output path is required (or use --self-test)')
    out = a.out if a.out.endswith('.kicad_pcb') else a.out + '.kicad_pcb'
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    txt, truth = build(a)
    with open(out, 'w', encoding='utf-8') as f:
        f.write(txt)
    tp = out[:-len('.kicad_pcb')] + '.truth.json'
    with open(tp, 'w', encoding='utf-8') as f:
        json.dump(truth, f, indent=2)
    print(f'{os.path.basename(out)}: K={truth["k"]} {truth["pattern"]} '
          f'{truth["rows"]}x{truth["cols"]} depth={truth["depth"]} '
          f'gap={truth["gap"]} -- crossings {truth["crossings"]}, LIS {truth["lis"]}, '
          f'lower bound {truth["corridor_lower_bound"]}, whole-lane optimum '
          f'{truth["corridor_optimum"]}, exact {truth["corridor_dp"]} '
          f'[{truth["dp_how"]}]'
          + ('' if not truth['obstacle'] else
             f' -- OBSTACLE {truth["obstacle"]["w"]}x{truth["obstacle"]["h"]} mm, '
             f'{truth["obstacle"]["room_above"]}/{truth["obstacle"]["room_below"]} mm '
             f'of room past it'))
    return 0


def judge_blind(ids, s, d, order_src, order_dst, tooth_layer=None,
                berth_layer=None, n=300, seed=0, swim=100.0, cap=22):
    """How much of the answer is INVISIBLE to the objective, with the swim
    price taken out of the comparison.

    Plans are grouped by how many lanes they swim; inside one group every
    plan pays the same swim total, so the price cancels and what is left is
    the objective's opinion about WHICH lanes to page. Reports the largest
    group: the span of the objective over it, the span of the TRUE cost,
    and their rank correlation.

    This is the arm that does not rest on a swimmer being free: whatever a
    swimmer really costs, it costs the same in every plan here."""
    pool = plan_pool(order_src, order_dst, tooth_layer, berth_layer,
                     n=n, seed=seed)
    groups = {}
    for plan in pool:
        fixed = {nm: l for nm, l in plan.items() if l != 'swim'}
        v, _how = exact_dp(ids, s, d, tooth_layer=tooth_layer,
                           berth_layer=berth_layer, fixed=fixed, cap=cap)
        if v is None:
            continue
        ns = sum(1 for l in plan.values() if l == 'swim')
        groups.setdefault(ns, []).append(
            (plan_objective(plan, tooth_layer, berth_layer, swim=swim), v))
    if not groups:
        return None
    ns, g = max(groups.items(), key=lambda kv: len(kv[1]))
    if len(g) < 5:
        return None
    objs = [o for o, _ in g]
    tr = [t for _, t in g]
    return {'n': len(g), 'swimmers': ns, 'rho': spearman(objs, tr),
            'true_lo': min(tr), 'true_hi': max(tr),
            'obj_lo': min(objs), 'obj_hi': max(objs),
            'blind': max(tr) - min(tr) if min(objs) == max(objs) else 0}


def judge_main(a):
    """Is the objective pointing the right way? -- printed.

    THE ONE CAVEAT, and it decides how far this can be read: in the model
    below a SWIMMER is free, because `exact_dp` routes the unpinned lanes
    optimally. The real braid routes a swimmer outside its page chains and
    may pay far more, or fail. So this measures whether the objective
    RANKS plans the way the truth does, and it measures the DEGENERACY of
    its optimum -- both of which are properties of the objective alone.
    It does NOT set the swim price: an objective that prefers swimmers
    scores well here by construction, and only a chain run can say what a
    swimmer really costs.
    """
    import random
    swims = tuple(float(x) for x in a.judge_swims.split(','))
    Ks = [int(x) for x in a.judge_k.split(',')]
    kinds = ('blocks', 'interleave', 'riffle', 'reversed', 'shuffle')
    jit = float(a.judge_jitter)

    def ends(ids, pi, seed):
        """The lane ends, perturbed. The PERMUTATION is untouched -- only the
        positions move, and by less than half a pitch, so no pair's order
        changes and the crossing GRAPH is identical. What changes is that
        each crossing gets its own place along the channel."""
        rng = random.Random(seed * 2654435761 + len(ids))
        j = (lambda: rng.uniform(-jit, jit)) if jit else (lambda: 0.0)
        return ({i: i + j() for i in ids}, {i: pi[i] + j() for i in ids})
    if jit:
        print(f'(ends jittered by +-{jit:g} of a pitch, so a regular article '
              f'does not answer from a tie-break -- --judge-jitter 0 to '
              f'disable)\n')
    print('Spearman(the planner objective, the TRUE cost of that plan) over a '
          'pool of model-feasible\nplans, the REGRET of the plan the objective '
          'would choose (ties broken pessimistically,\nbecause nothing in the '
          'objective prefers one tied plan over another), and the SPREAD of\n'
          'true cost inside the objective\'s own optimum set.\n')
    print(f'{"case":20} {"pool":>5} {"opt":>4} | '
          + '  '.join(f'{"swim=" + f"{p:g}":>20}' for p in swims))
    print(f'{"":20} {"":5} {"":4} | '
          + '  '.join(f'{"rho  regret  ties":>20}' for p in swims))
    agg = {p: [0.0, 0, 0, 0, 0] for p in swims}
    for K in Ks:
        for kind in kinds:
            for seed in (0, 1, 2):
                pi = pattern_perm(kind, K, seed=seed)
                ids = list(range(K))
                o_s, o_d = ids, sorted(ids, key=lambda i: pi[i])
                s, d = ends(ids, pi, seed)
                r = judge_report(ids, s, d, o_s, o_d, n=a.judge_pool,
                                 seed=seed, swims=swims, cap=a.dp_cap)
                if r is None:
                    continue
                cells = []
                for row in r['rows']:
                    g = agg[row['swim']]
                    if row['rho'] is not None:
                        g[0] += row['rho']
                        g[1] += 1
                    g[2] += row['regret']
                    g[3] += row['tie_spread']
                    g[4] += 1
                    rho = f"{row['rho']:+.2f}" if row['rho'] is not None else ' -- '
                    cells.append(f'{rho} {row["regret"]:+5d} {row["ties"]:5}')
                if seed == 0:
                    print(f'{kind + "_k" + str(K):20} {r["pool"]:5} '
                          f'{r["best_true"]:4} | '
                          + '  '.join(f'{c:>20}' for c in cells))
    print(f'\nover every case ({agg[swims[0]][4]} of them):')
    for p in swims:
        g = agg[p]
        print(f'   swim={p:6g}: mean rho {g[0] / max(g[1], 1):+.3f}   '
              f'total regret {g[2]:+5d} vias   mean spread inside the optimum '
              f'{g[3] / max(g[4], 1):.1f}')
    print('\nRead the regret as a STEP, not a curve: a swimmer that routes '
          'freely costs 2, so\nevery price below 2 buys the same plan and '
          'every price above it buys the other one.\nAnd read nothing here as '
          'the right price -- see judge_main\'s docstring.')
    # ---- the arm the blind spot cannot reach
    print('\nAnd with the swim price taken OUT of the comparison -- only plans '
          'that swim the same\nnumber of lanes, so whatever a swimmer really '
          'costs it costs the same in all of them:\n')
    print(f'{"case":20} {"plans":>6} {"swims":>6} {"rho":>7} '
          f'{"TRUE spans":>12} {"objective spans":>16}')
    rows, blind = [], []
    for K in Ks:
        for kind in kinds:
            for seed in (0, 1, 2):
                pi = pattern_perm(kind, K, seed=seed)
                ids = list(range(K))
                o_s, o_d = ids, sorted(ids, key=lambda i: pi[i])
                s, d = ends(ids, pi, seed)
                b = judge_blind(ids, s, d, o_s, o_d, n=a.judge_pool, seed=seed,
                                swim=max(swims), cap=a.dp_cap)
                if b is None:
                    continue
                rows.append(b)
                if b['blind']:
                    blind.append((f'{kind}_k{K}_s{seed}', b['blind']))
                if seed == 0:
                    r = f'{b["rho"]:+.2f}' if b['rho'] is not None else '  -- '
                    print(f'{kind + "_k" + str(K):20} {b["n"]:6} {b["swimmers"]:6} '
                          f'{r:>7} {str(b["true_lo"]) + ".." + str(b["true_hi"]):>12} '
                          f'{str(int(b["obj_lo"])) + ".." + str(int(b["obj_hi"])):>16}')
    if rows:
        rho = [b['rho'] for b in rows if b['rho'] is not None]
        print(f'\nmean rho inside a fixed swimmer count: '
              f'{sum(rho) / max(len(rho), 1):+.3f} over {len(rho)} group(s) '
              f'({len(rows) - len(rho)} where the objective is CONSTANT and has '
              f'no ranking at all); mean true span inside a group '
              f'{sum(b["true_hi"] - b["true_lo"] for b in rows) / len(rows):.1f} '
              f'vias.')
        if blind:
            worst = max(b for _t, b in blind)
            print(f'{len(blind)} case(s) where the objective gives ONE value to '
                  f'plans whose real costs differ by up to {worst} vias. No '
                  f'budget, solver or tie-break inside this model can choose '
                  f'between them.')
    return 0


def self_test():
    """The three truth sources must bracket on every pattern, and the two
    planted families must hit their closed forms. A silent disagreement
    here is a harness bug, and the whole point of the harness is that its
    answers are not guesses."""
    bad = 0
    for K in (4, 6, 8, 9, 12):
        for kind in ('sorted', 'reversed', 'blocks', 'interleave', 'riffle',
                     'shuffle'):
            for seed in (0, 1, 2):
                pi = pattern_perm(kind, K, seed=seed, blocks=2)
                ids = list(range(K))
                o_s, o_d = ids, sorted(ids, key=lambda i: pi[i])
                edges = crossing_edges(o_s, o_d)
                opt, _c, _h = optimum(ids, edges)
                lb, _lis = lower_bound(o_s, o_d)
                dp, how = exact_dp(ids, {i: i for i in ids},
                                   {i: pi[i] for i in ids})
                tag = f'{kind} K={K} seed={seed}'
                if dp is None:
                    print(f'FAIL {tag}: no exact answer ({how})')
                    bad += 1
                    continue
                if dp < lb:
                    print(f'FAIL {tag}: exact {dp} below the LIS bound {lb}')
                    bad += 1
                if opt is not None and dp > opt:
                    print(f'FAIL {tag}: exact {dp} above the whole-lane {opt}')
                    bad += 1
                # the closed forms the planted patterns were designed to
                # have. `interleave` is deliberately NOT here: the
                # de-interleave's crossing graph is sparse, not the
                # complete bipartite one it looks like, so its optimum is
                # what `optimum()` computes and not a formula.
                want = None
                if kind == 'sorted':
                    want = 0
                elif kind == 'blocks':
                    want = 2 * min(K // 2, K - K // 2)   # complete bipartite
                if want is not None and dp != want:
                    print(f'FAIL {tag}: exact {dp}, closed form {want}')
                    bad += 1
                if kind == 'reversed' and seed == 0:
                    print(f'  note {tag}: LIS bound {lb}, exact {dp}'
                          + ('  (the bound is TIGHT)' if dp == lb else
                             '  (the bound is NOT tight)'))
                # The planted-bipartite families agree with the exact
                # answer AT THESE SEEDS -- and that is all it is. `riffle`
                # K=8 seed=4 is a counter-example (whole-lane 8, exact 6),
                # so this is an empirical agreement over the tested range
                # and NOT the property the README once implied. The
                # witnesses below pin the counter-example so it stays true.
                if kind in ('sorted', 'blocks', 'interleave', 'riffle') \
                        and opt != dp:
                    print(f'FAIL {tag}: planted-bipartite whole-lane {opt} '
                          f'!= exact {dp} (mid-channel changes should not help '
                          f'at the seeds this loop covers)')
                    bad += 1
                # --- `cap_floor` walks the SAME model as `exact_dp` by an
                # entirely different algorithm (a MILP over per-crossing
                # layer bits against a DP over the layer hypercube), so an
                # agreement between them is real evidence about both. It
                # is the uncapped call that is comparable; a cap can only
                # raise the answer, never lower it.
                cf, per_l, st = cap_floor(ids, {i: i for i in ids},
                                          {i: pi[i] for i in ids})
                if cf is None:
                    if 'needs scipy' not in st:
                        print(f'FAIL {tag}: cap_floor gave nothing ({st})')
                        bad += 1
                else:
                    if cf != dp:
                        print(f'FAIL {tag}: cap_floor {cf} != exact_dp {dp} '
                              f'-- the same model answered two ways must agree')
                        bad += 1
                    if sum(per_l.values()) != cf:
                        print(f'FAIL {tag}: cap_floor per-lane sums to '
                              f'{sum(per_l.values())}, total says {cf}')
                        bad += 1
                    # a cap at the worst lane's own cost must stay feasible
                    # and must not change the answer
                    wide = max(per_l.values()) if per_l else 0
                    cw, _p, sw = cap_floor(ids, {i: i for i in ids},
                                           {i: pi[i] for i in ids}, cap=wide)
                    if cw != cf:
                        print(f'FAIL {tag}: cap={wide} gave {cw} ({sw}), '
                              f'uncapped {cf} -- a non-binding cap changed it')
                        bad += 1
                # --- the PLANNER'S model, checked three ways. It is solved by
                # an O(K^3) DP, so it needs an independent opinion: the
                # brute force over F/B/swim (3**K, small K only) and Greene's
                # theorem, which says the most lanes two increasing
                # subsequences can hold is lambda1 + lambda2 of the RSK shape.
                m = pages_model(o_s, o_d)
                rd = {n: i for i, n in enumerate(o_d)}
                lam = rsk_shape([rd[n] for n in o_s])
                if m['paged'] != sum(lam[:2]):
                    print(f'FAIL {tag}: the model pages {m["paged"]} lanes, '
                          f'Greene says {sum(lam[:2])} (shape {lam})')
                    bad += 1
                if lam[0] != _lis:
                    print(f'FAIL {tag}: RSK lambda1 {lam[0]} != LIS {_lis}')
                    bad += 1
                if K <= 8:
                    import itertools
                    eset = {tuple(sorted(e)) for e in edges}
                    best = None
                    for asg in itertools.product('FBS', repeat=K):
                        if any(asg[i] == asg[j] != 'S' for i, j in eset):
                            continue
                        c = sum(0.0 if a == 'F' else (2.0 if a == 'B' else 100.0)
                                for a in asg)
                        best = c if best is None else min(best, c)
                    if abs(m['cost'] - best) > 1e-9:
                        print(f'FAIL {tag}: model DP {m["cost"]}, brute {best}')
                        bad += 1
                # pinning the model's pages can never beat the free optimum,
                # and on a case it pages entirely it must reproduce the
                # whole-lane answer exactly
                fixed = {n: l for n, l in m['page'].items() if l != 'swim'}
                mdp, mhow = exact_dp(ids, {i: i for i in ids},
                                     {i: pi[i] for i in ids}, fixed=fixed)
                if mdp is None:
                    print(f'FAIL {tag}: the model\'s own plan does not route '
                          f'({mhow})')
                    bad += 1
                elif mdp < dp:
                    print(f'FAIL {tag}: pinned {mdp} below the free optimum {dp}')
                    bad += 1
                elif kind == 'reversed' and m['true_lb'] != dp:
                    # On a clique crossing graph the model pages exactly one
                    # lane per page and swims the rest, so its plan really
                    # costs 0 + 2 + 2*(K-2) = 2*(K-1), which is the exact
                    # optimum. That pins `true_lb`'s swimmer term to a closed
                    # form -- the one place it can be checked and not merely
                    # bounded, and `true_lb` is the ONLY model-cost number
                    # available past the DP's K cap.
                    print(f'FAIL {tag}: the model true_lb {m["true_lb"]} != '
                          f'the exact {dp} on a clique, where they must agree')
                    bad += 1
                elif m['true_lb'] > mdp + 1e-9:
                    # `true_lb` prices a swimmer at 2 and must therefore be a
                    # BOUND on what the model's plan really costs. Asserted
                    # because a field nothing checks is a field that drifts
                    # (this one survived the whole battery until it was).
                    print(f'FAIL {tag}: the model true_lb {m["true_lb"]} is '
                          f'above what its plan really costs ({mdp})')
                    bad += 1
                elif m['n_swim'] == 0 and opt is not None and mdp != opt:
                    print(f'FAIL {tag}: a fully paged plan costs {mdp}, '
                          f'whole-lane says {opt}')
                    bad += 1
                # --- the LOWER BOUND. Two properties, and the first is an
                # exact agreement between two unrelated algorithms: with
                # both ends of every lane on F the bound's DP minimises
                # 2*(K - |A|) over increasing subsequences A, so it must
                # land exactly on the LIS formula that patience sorting
                # computes. The second is the property that makes it a
                # bound at all.
                clb = channel_lower_bound(o_s, o_d)
                if clb['cost'] != lb:
                    print(f'FAIL {tag}: the channel bound {clb["cost"]} != the '
                          f'LIS formula {lb}, which it must equal when every '
                          f'end is on F')
                    bad += 1
                if dp is not None and clb['cost'] > dp:
                    print(f'FAIL {tag}: the channel bound {clb["cost"]} is '
                          f'ABOVE the exact optimum {dp} -- it is not a bound')
                    bad += 1
                # ...and with the ends MOVED it must still bound. Flip a
                # third of the teeth to B: the LIS formula stops applying
                # (it assumes F ends) but the bound does not.
                tl2 = {n: ('B.Cu' if n % 3 == 0 else 'F.Cu') for n in ids}
                clb2 = channel_lower_bound(o_s, o_d, tooth_layer=tl2)
                dp2, _ = exact_dp(ids, {i: i for i in ids},
                                  {i: pi[i] for i in ids}, tooth_layer=tl2)
                if dp2 is not None and clb2['cost'] > dp2:
                    print(f'FAIL {tag}: with teeth moved, the bound '
                          f'{clb2["cost"]} is above the exact {dp2}')
                    bad += 1
                # ...and the ends must REACH it. Every berth on B against
                # every tooth on F: each lane must change layer exactly
                # once wherever it runs, so the answer is K whatever the
                # permutation -- while a bound that ignored the ends would
                # answer 2*(K - LIS), which is 0 on `sorted`.
                bl3 = {n: 'B.Cu' for n in ids}
                clb3 = channel_lower_bound(o_s, o_d, berth_layer=bl3)
                if clb3['cost'] != K:
                    print(f'FAIL {tag}: with every berth on B the bound is '
                          f'{clb3["cost"]}, must be {K} (one change a lane)')
                    bad += 1
                # ...and the pin must BITE. Everything above passes a pin
                # that does nothing: on these cases the model's own pages
                # are the optimum's, so an inert pin reproduces it (measured
                # -- `pin_mask |= 0` survived the whole battery). These two
                # ask for an answer only a working pin can give.
                if opt is not None:
                    # `optimum` chooses the cheaper orientation PER
                    # COMPONENT, so the pin must be built the same way --
                    # one global flip cannot reach its answer, which is
                    # what the first version of this check got wrong.
                    _o, comps, _h2 = optimum(ids, edges)
                    full = {}
                    for p0, p1 in comps:
                        a = len(p1) * 2      # part 1 on B: 2 vias a lane
                        b = len(p0) * 2      # the swap
                        for n in p0:
                            full[n] = 'F.Cu' if a <= b else 'B.Cu'
                        for n in p1:
                            full[n] = 'B.Cu' if a <= b else 'F.Cu'
                    v2, _ = exact_dp(ids, {i: i for i in ids},
                                     {i: pi[i] for i in ids}, fixed=full)
                    if v2 != opt:
                        print(f'FAIL {tag}: the whole-lane colouring pinned '
                              f'costs {v2}, whole-lane says {opt}')
                        bad += 1
                if kind == 'sorted':
                    # a bus with no crossings: every lane on F is 0 vias, and
                    # pinning ONE lane to B is 2 -- one via at each end. A pin
                    # that is not applied answers 0.
                    for pick in (0, K // 2, K - 1):
                        v4, _ = exact_dp(ids, {i: i for i in ids},
                                         {i: pi[i] for i in ids},
                                         fixed={pick: 'B.Cu'})
                        if v4 != 2:
                            print(f'FAIL {tag}: lane {pick} pinned to B on a '
                                  f'crossing-free bus costs {v4}, must be 2')
                            bad += 1
    # --- WITNESSES: named cases whose answer is known and DIFFERENT, so
    # that a mechanism which quietly stops working is caught by a number
    # and not by an absence. Each was found by search and is recorded with
    # what it proves; the loop above cannot supply them because at its
    # seeds every family happens to agree.
    for kind, K, seed, want_opt, want_dp in (
            # the whole-lane model is STRICTLY worse than the truth here, so
            # pinning its colouring must answer `opt` and not `dp`. Without
            # this a pin that leaks mid-channel passes the whole battery
            # (measured: dropping the in-loop mask survived everything else).
            ('riffle', 8, 4, 8, 6),
            ('riffle', 11, 7, 10, 8),
            ('riffle', 12, 11, 10, 8),
    ):
        pi = pattern_perm(kind, K, seed=seed)
        ids = list(range(K))
        o_s, o_d = ids, sorted(ids, key=lambda i: pi[i])
        edges = crossing_edges(o_s, o_d)
        s, d = {i: i for i in ids}, {i: pi[i] for i in ids}
        opt, comps, _h = optimum(ids, edges)
        dp, _ = exact_dp(ids, s, d)
        tag = f'witness {kind} K={K} seed={seed}'
        if (opt, dp) != (want_opt, want_dp):
            print(f'FAIL {tag}: whole-lane/exact {opt}/{dp}, recorded '
                  f'{want_opt}/{want_dp} -- the witness has moved')
            bad += 1
            continue
        full = {}
        for p0, p1 in comps:
            a, b = len(p1) * 2, len(p0) * 2
            for n in p0:
                full[n] = 'F.Cu' if a <= b else 'B.Cu'
            for n in p1:
                full[n] = 'B.Cu' if a <= b else 'F.Cu'
        v, _ = exact_dp(ids, s, d, fixed=full)
        if v != opt:
            print(f'FAIL {tag}: the whole-lane colouring pinned costs {v}, '
                  f'must be {opt} -- a pinned lane is changing layer '
                  f'mid-channel, which is what a whole-lane PLAN forbids')
            bad += 1
        else:
            print(f'  note {tag}: whole-lane {opt} against exact {dp} -- the '
                  f'two-page model cannot express the answer, and the pin '
                  f'holds it to {v}')
    # ...and the same again for cases the model must SWIM, where the pin
    # holds only the paged lanes. This is the arm that catches a pin which
    # leaks mid-channel: with the swimmers free, a leak lets the PAGED
    # lanes drift too and the model error collapses (measured on the first
    # of these: 34 with the pin held, 22 with it dropped).
    for kind, K, seed, want_dp, want_paged, want_swim, want_mdp in (
            ('shuffle', 12, 0, 20, 8, 4, 34),
            ('shuffle', 15, 0, 30, 9, 6, 40),
            ('shuffle', 18, 0, 28, 13, 5, 32),
    ):
        pi = pattern_perm(kind, K, seed=seed)
        ids = list(range(K))
        o_s, o_d = ids, sorted(ids, key=lambda i: pi[i])
        s, d = {i: i for i in ids}, {i: pi[i] for i in ids}
        dp, _ = exact_dp(ids, s, d)
        m = pages_model(o_s, o_d)
        fixed = {n: l for n, l in m['page'].items() if l != 'swim'}
        mdp, _ = exact_dp(ids, s, d, fixed=fixed)
        tag = f'witness {kind} K={K} seed={seed}'
        got = (dp, m['paged'], m['n_swim'], mdp)
        want = (want_dp, want_paged, want_swim, want_mdp)
        if got != want:
            print(f'FAIL {tag}: (exact, paged, swimmers, model-pinned) = {got}, '
                  f'recorded {want}')
            bad += 1
        else:
            print(f'  note {tag}: the model pages {m["paged"]} and swims '
                  f'{m["n_swim"]} at a price of {m["cost"]:.0f} vias; that plan '
                  f'really costs {mdp} against an optimum of {dp} '
                  f'(MODEL error +{mdp - dp})')
    # --- the REFUSALS. `pages_model` never builds a plan that pins two
    # crossing lanes to one page, so the guard that catches it is a branch
    # no other check reaches (measured: removing it survived the whole
    # battery). Hand it one on purpose, and assert the REASON rather than
    # a bare None -- a crash and a refusal both return nothing.
    pi = pattern_perm('reversed', 6, seed=0)
    ids = list(range(6))
    s, d = {i: i for i in ids}, {i: pi[i] for i in ids}
    v, why = exact_dp(ids, s, d, fixed={0: 'F.Cu', 1: 'F.Cu'})   # 0 and 1 cross
    if v is not None or 'same page' not in why:
        print(f'FAIL refusal: two crossing lanes pinned to one page answered '
              f'{v} ({why}) instead of refusing')
        bad += 1
    else:
        print(f'  note refusal: two crossing lanes on one page -> "{why}"')
    # and the K cap must refuse by its own reason, not by crashing
    big = list(range(40))
    v, why = exact_dp(big, {i: i for i in big}, {i: 39 - i for i in big}, cap=22)
    if v is not None or 'cap' not in why:
        print(f'FAIL refusal: 40 free lanes past a cap of 22 answered {v} ({why})')
        bad += 1
    # ...and the SAME instance with all but 10 pinned must be answered,
    # because the cap is on the free lanes. A cap that still counted K
    # would refuse this, and the model-error column would be blank at
    # exactly the sizes it was built for.
    pins = {i: ('B.Cu' if i % 2 else 'F.Cu') for i in big[:30]}
    v, why = exact_dp(big, {i: i for i in big}, {i: 39 - i for i in big},
                      cap=22, fixed=pins)
    if v is None and 'cap' in (why or ''):
        print(f'FAIL refusal: 10 free lanes of 40 refused by the cap ({why})')
        bad += 1
    # --- the two-via cap, checked by TWO independent solvers.
    #
    # These witnesses are INFEASIBLE cases on purpose. A cross-check run
    # only on feasible channels is vacuous: a SAT model with its
    # transition literal inverted bounds the NON-transitions instead, which
    # every alternating assignment satisfies, so it calls everything
    # feasible and agrees with the MILP on every feasible case. Measured --
    # with that exact bug in place, a self-test whose cases were all
    # K <= 12 (all feasible) printed ALL PASS. The infeasible rows are the
    # test.
    CAP_WITNESS = [           # (K, seed, cap, feasible?)
        (12, 0, 2, True),
        (16, 1, 2, False),
        (16, 2, 2, False),
        (20, 0, 2, False),
        (16, 1, 4, True),     # the same channel is fine at four
    ]
    for K, seed, cp, want in CAP_WITNESS:
        pi = pattern_perm('shuffle', K, seed=seed)
        ids = list(range(K))
        s_ = {i: float(i) for i in ids}
        d_ = {i: float(pi[i]) for i in ids}
        tag = f'cap witness shuffle K={K} seed={seed} cap={cp}'
        v, _p, st = cap_floor(ids, s_, d_, cap=cp)
        if st.startswith('not computed'):
            print(f'  note {tag}: skipped ({st})')
            continue
        if (v is not None) != want:
            print(f'FAIL {tag}: MILP says '
                  f'{"feasible" if v is not None else "infeasible"}, '
                  f'expected {"feasible" if want else "infeasible"}')
            bad += 1
        sat = cap_sat_feasible(ids, s_, d_, cap=cp)
        if sat.startswith('not computed'):
            print(f'  note {tag}: CP-SAT skipped ({sat})')
        elif sat == 'UNKNOWN':
            print(f'  note {tag}: CP-SAT UNKNOWN (a budget, not a verdict)')
        elif (sat == 'FEASIBLE') != want:
            print(f'FAIL {tag}: CP-SAT says {sat}, expected '
                  f'{"FEASIBLE" if want else "INFEASIBLE"}')
            bad += 1
    print(f'  note the two-via cap: {len(CAP_WITNESS)} witness(es), '
          f'{sum(1 for w in CAP_WITNESS if not w[3])} of them INFEASIBLE -- '
          f'which is the half that can fail')
    print('self-test: ' + ('ALL PASS' if not bad else f'{bad} FAILURE(S)'))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())

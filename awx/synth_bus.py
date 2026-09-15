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


def exact_dp(ids, s, d, tooth_layer=None, berth_layer=None, cap=22):
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
    what the harness's `thru` column exists to catch."""
    K = len(ids)
    if K == 0:
        return 0, 'exact (no lanes)'
    if K > cap:
        return None, f'not computed (K {K} > the 2**K cap of {cap})'
    try:
        import numpy as np
    except ImportError:
        np = None
    if np is None and K > 12:
        return None, f'not computed (no numpy, and K {K} > 12 in pure python)'
    tl = tooth_layer or {}
    bl = berth_layer or {}
    start = sum(1 << b for b, n in enumerate(ids) if tl.get(n, 'F.Cu') != 'F.Cu')
    goal = sum(1 << b for b, n in enumerate(ids) if bl.get(n, 'F.Cu') != 'F.Cu')
    events = crossing_events(ids, s, d)
    N = 1 << K
    if np is not None:
        x = np.arange(N, dtype=np.int32)
        cost = np.zeros(N, dtype=np.int32)
        for b in range(K):                     # hamming(x, start)
            cost += ((x >> b) & 1) ^ ((start >> b) & 1)
        for (a, b) in events:
            same = (((x >> a) & 1) == ((x >> b) & 1))
            cost = np.where(same, DP_INF, cost)
            if cost.min() >= DP_INF:
                return None, 'no two-layer routing of this crossing order exists'
            cost = _relax_hamming(cost, K)
        for b in range(K):                     # + hamming(x, goal), read at goal
            cost += ((x >> b) & 1) ^ ((goal >> b) & 1)
        best = int(cost.min())
    else:
        cost = [bin(v ^ start).count('1') for v in range(N)]
        for (a, b) in events:
            ba, bb = 1 << a, 1 << b
            for v in range(N):
                if bool(v & ba) == bool(v & bb):
                    cost[v] = DP_INF
            if min(cost) >= DP_INF:
                return None, 'no two-layer routing of this crossing order exists'
            _relax_hamming(cost, K)
        best = min(c + bin(v ^ goal).count('1') for v, c in enumerate(cost))
    if best >= DP_INF:
        return None, 'no two-layer routing of this crossing order exists'
    return best, ('exact over pages AND mid-channel changes, for the '
                  'straight-line crossing order')


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
    ap.add_argument('--self-test', action='store_true',
                    help='check the three truth sources against each other on '
                         'every pattern and exit')
    a = ap.parse_args(argv)
    if a.self_test:
        return self_test()
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
                if kind in ('sorted', 'blocks', 'interleave', 'riffle') \
                        and opt != dp:
                    print(f'FAIL {tag}: planted-bipartite whole-lane {opt} '
                          f'!= exact {dp} (mid-channel changes should not help)')
                    bad += 1
    print('self-test: ' + ('ALL PASS' if not bad else f'{bad} FAILURE(S)'))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())

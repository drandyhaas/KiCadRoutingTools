#!/usr/bin/env python3
"""schedule.py -- the braid's order combinatorics, with no geometry in it.

Given a corridor's LAUNCH order and TARGET order (two orderings of the
same nets across the bundle), decide which nets dive and when they
cross whom: the divers are the complement of the longest increasing
subsequence of the launch->target permutation, a SERIAL PASS fixes who
passes whom and in which direction, and the WAVE SCHEDULE turns the
inversions into columns of concurrent adjacent swaps, gated so every
diver-diver crossing has exactly one side on the back layer. A column
is a position along the corridor; where that is in millimetres is the
corridor's business, not this module's.
"""
from __future__ import annotations

import os
from typing import Dict, List, Optional, Sequence, Set, Tuple


def lis_keep(ranks: Sequence[int]) -> Set[int]:
    n = len(ranks)
    if n == 0:
        return set()
    best = [1] * n
    prev = [-1] * n
    for i in range(n):
        for j in range(i):
            if ranks[j] < ranks[i] and best[j] + 1 > best[i]:
                best[i] = best[j] + 1
                prev[i] = j
    i = max(range(n), key=lambda k: best[k])
    keep = set()
    while i >= 0:
        keep.add(i)
        i = prev[i]
    return keep


def lis_keep_weighted(ranks: Sequence[int], weight) -> Set[int]:
    """A maximum-length increasing subsequence, choosing among the
    equally-long ones the one of greatest total weight (the plan uses
    it to prefer keepers whose escape starts on the tooth layer)."""
    n = len(ranks)
    if n == 0:
        return set()
    W = [1.0 + 0.5 * float(weight[i]) for i in range(n)]
    best = [(1, W[i]) for i in range(n)]
    prev = [-1] * n
    for i in range(n):
        for j in range(i):
            if ranks[j] < ranks[i]:
                cand = (best[j][0] + 1, best[j][1] + W[i])
                if cand > best[i]:
                    best[i] = cand
                    prev[i] = j
    i = max(range(n), key=lambda k: best[k])
    keep = set()
    while i >= 0:
        keep.add(i)
        i = prev[i]
    return keep


class Schedule:
    """Divers, passes and the wave schedule for one corridor."""

    def __init__(self, launch: Sequence[str], target: Sequence[str],
                 tooth_layer: Optional[Dict[str, str]] = None, log=None,
                 dest_layer: Optional[Dict[str, str]] = None,
                 pages: Optional[Dict[str, Optional[str]]] = None):
        self.launch = list(launch)
        self.target = list(target)
        assert sorted(self.launch) == sorted(self.target)
        self.trank = {nm: i for i, nm in enumerate(self.target)}
        self.lidx = {nm: i for i, nm in enumerate(self.launch)}
        ranks = [self.trank[nm] for nm in self.launch]
        tl = tooth_layer or {}
        dl = dest_layer or {}
        # birth layers: what a swimmer is on before its first crossing
        self.tl = {nm: tl.get(nm, 'F.Cu') for nm in self.launch}

        def on(nm, L):
            # how many of the net's two ends already sit on L: a page
            # lane pays a via at each end that does not
            return ((1.0 if tl.get(nm, 'F.Cu') == L else 0.0)
                    + (1.0 if dl.get(nm, 'F.Cu') == L else 0.0))
        # TWO PAGES. The F-page is the largest crossing-free set (the
        # LIS of the launch -> target permutation, preferring teeth
        # already on F); the B-page is the largest crossing-free set of
        # the REST (preferring teeth on B). A page lane keeps its layer
        # through the whole schedule region, so an F-page lane and a
        # B-page lane cross for FREE -- no layer rule, no via, no room
        # -- which is how the human routes a corridor (every net on one
        # layer end to end, two vias, both escapes). What is left are
        # the SWIMMERS: they take the other layer from whichever page
        # lane they cross and pay a via at each change. Measured with
        # Greene's count, 21-23 of K28's 27 nets fit two pages; the
        # single-page floor made 13 of them dive. TWO_PAGE=0 keeps the
        # single page (every non-keeper a swimmer).
        self.two_page = os.environ.get('TWO_PAGE', '0') == '1'
        if self.two_page and pages is not None:
            # the PLAN's page assignment, used verbatim (the chain
            # writes it beside the fanout board), so plan and braid
            # can never disagree about who is a swimmer. A member
            # that crosses an earlier member of its own page is
            # demoted to a swimmer: the assignment came from another
            # geometry's orders, and a page must stay crossing-free
            # under THIS corridor's orders.
            keep = set()
            self.page = {nm: None for nm in self.launch}
            for L in ('F.Cu', 'B.Cu'):
                kept: List[str] = []
                for i, nm in enumerate(self.launch):
                    if pages.get(nm) != L:
                        continue
                    if all(not self.inverted(nm, om) for om in kept):
                        kept.append(nm)
                        self.page[nm] = L
                        if L == 'F.Cu':
                            keep.add(i)
            if log:
                demoted = [nm for nm in self.launch
                           if pages.get(nm) in ('F.Cu', 'B.Cu')
                           and self.page[nm] is None]
                if demoted:
                    log(f'  pages sidecar: demoted to swimmers '
                        f'(cross their own page here): {demoted}')
        else:
            # single page: the plain LIS (the form every recorded
            # ladder was routed with); two pages: weighted by the
            # ends' layers
            keep = (lis_keep_weighted(ranks,
                                      [on(nm, 'F.Cu') for nm in self.launch])
                    if self.two_page else lis_keep(ranks))
            self.page = {nm: None for nm in self.launch}
            for i in keep:
                self.page[self.launch[i]] = 'F.Cu'
            if self.two_page:
                rest = [nm for i, nm in enumerate(self.launch)
                        if i not in keep]
                if len(rest) >= 2:
                    # TWO_PAGE_B default is 'worst' (2026-08-31): with
                    # the diamond reservation + free swimmers, the
                    # worst-crosser B page ties 'lis' at every K below
                    # 28 (4/10/24/32 vias, all complete) and beats it
                    # at K28 (50/0 vs 56/0, human 46) -- the demoted
                    # risers it strands are exactly the class the last
                    # call rescues at 2 vias. 'lis' selects the old
                    # length-first page, 'wmax' the weighted middle.
                    if os.environ.get('TWO_PAGE_B', 'worst') == 'wmax':
                        # MAX-WEIGHT increasing subsequence of the rest,
                        # weight = 1 + inversions: between 'lis' (length
                        # first -- keeps the mutually-increasing risers,
                        # strands the worst crossers: K28 49v/3 open
                        # SDQ0/SDQ15/SWE) and 'worst' (crossers first --
                        # rescues those, demotes the risers: K28 46v/4
                        # open SDQM1/SDQ9/11/12). A heavy crosser is
                        # worth a page slot exactly as much as the
                        # swimmers it would otherwise refuse into.
                        inv = {nm: sum(1 for om in self.launch
                                       if om != nm and self.inverted(nm, om))
                               for nm in rest}
                        rr = [self.trank[nm] for nm in rest]
                        W = [1.0 + inv[rest[i]] for i in range(len(rest))]
                        n_ = len(rest)
                        bestw = [(W[i], ) for i in range(n_)]
                        prev = [-1] * n_
                        for i in range(n_):
                            for j in range(i):
                                if rr[j] < rr[i] and \
                                        bestw[j][0] + W[i] > bestw[i][0]:
                                    bestw[i] = (bestw[j][0] + W[i], )
                                    prev[i] = j
                        i = max(range(n_), key=lambda k: bestw[k][0])
                        while i >= 0:
                            self.page[rest[i]] = 'B.Cu'
                            i = prev[i]
                    elif os.environ.get('TWO_PAGE_B', 'worst') == 'worst':
                        # the B-page RESCUES THE WORST CROSSERS first
                        # -- the human's constant-layer SWE idiom. On
                        # an all-F-escape fanout this measured WORSE
                        # (K11 10/11 -> 7/11: it demotes the mutually-
                        # increasing risers to swimmers against a
                        # saturated F layer); with escapes BY PAGE
                        # (step 3) it is the arm to re-try, hence a
                        # knob rather than a default.
                        inv = {nm: sum(1 for om in self.launch
                                       if om != nm and self.inverted(nm, om))
                               for nm in rest}
                        page_b: List[str] = []
                        for nm in sorted(rest,
                                         key=lambda n: (-inv[n],
                                                        -on(n, 'B.Cu'))):
                            if all(not self.inverted(nm, om)
                                   for om in page_b):
                                page_b.append(nm)
                        for nm in page_b:
                            self.page[nm] = 'B.Cu'
                    else:
                        keep_b = lis_keep_weighted(
                            [self.trank[nm] for nm in rest],
                            [on(nm, 'B.Cu') for nm in rest])
                        for i in keep_b:
                            self.page[rest[i]] = 'B.Cu'
                elif rest:
                    self.page[rest[0]] = 'B.Cu'
        self.b_page = [nm for nm in self.launch if self.page[nm] == 'B.Cu']
        self.swimmers = [nm for nm in self.launch if self.page[nm] is None]
        self.divers = {self.launch[i] for i in range(len(self.launch))
                       if i not in keep}
        trank, lidx, divers = self.trank, self.lidx, self.divers
        ups = sorted((d for d in divers if trank[d] < lidx[d]),
                     key=lambda nm: trank[nm])
        downs = sorted((d for d in divers if trank[d] >= lidx[d]),
                       key=lambda nm: -trank[nm])
        # a diver whose tooth already sits on B.Cu is BORN diving: it
        # needs no dive via if it reaches its first own crossing on B.
        # It takes no special place in the priority: moving B-born
        # divers to the front made the serial pass place them relative
        # to divers not yet moved (K32: the pass did not reach the
        # target).
        self.birth_b = {d for d in divers if tl.get(d) == 'B.Cu'}
        # swimmers first (the most constrained), then the B-page
        self.priority = ([d for d in ups + downs if self.page[d] is None]
                         + [d for d in ups + downs if self.page[d] == 'B.Cu'])

        # serial pass fixes WHO passes WHOM -- and in which DIRECTION.
        # The target-vs-launch index says where a diver ends up overall,
        # but the other divers' passes shift it on the way: rot90 K15's
        # SDQ12 ends 3 slots below its launch (a "down" diver) yet must
        # pass SDQ11 going UP, once the up-divers have carried SDQ11
        # below it. The serial pass sees the sequence as it stands when
        # the diver moves, so its direction is the one the wave must
        # use; the global one deadlocked the wave.
        sseq = list(self.launch)
        placed = set(nm for nm in self.launch if nm not in divers)
        for d in self.priority:
            i = sseq.index(d)
            rest = sseq[:i] + sseq[i + 1:]
            # the slot: after the last PLACED element (a keeper, or a
            # diver already moved) of smaller rank. A diver not yet
            # moved is no anchor -- K32: SCKE1 was placed after SDQ7, an
            # unmoved diver then sitting to the right of SA0, and so
            # never passed SA0.
            want = 0
            for j, e in enumerate(rest):
                if e in placed and trank[e] < trank[d]:
                    want = j + 1
            sseq = rest[:want] + [d] + rest[want:]
            placed.add(d)
        assert sseq == self.target, (sseq, self.target)
        if log:
            if self.two_page:
                log(f'  page F: {[n for n in self.launch if self.page[n] == "F.Cu"]}')
                log(f'  page B: {self.b_page}   swimmers: {self.swimmers}')
            log(f'  launch order: {self.launch}')
            log(f'  target order: {self.target}  ranks: {ranks}')
            log(f'  divers ({len(divers)}): '
                f'{sorted(divers, key=lambda n: trank[n])}')
            if self.birth_b:
                log(f'  B-birth divers: {sorted(self.birth_b)}')

    def inverted(self, anm: str, bnm: str) -> bool:
        return ((self.lidx[anm] < self.lidx[bnm])
                != (self.trank[anm] < self.trank[bnm]))

    def is_free(self, d: str, p: str) -> bool:
        """A crossing of two page lanes: no layer rule, no via room."""
        return bool(self.page[d]) and bool(self.page[p])

    def ensure_recolor(self, cols) -> None:
        """BRAID_WORD=1 (#622 braid-word move): swimmer x swimmer
        crossings have a FREE orientation -- the crossing only needs
        the two on OPPOSITE layers, but pair_layers hardcodes
        mover-on-B/passed-on-F, so a swimmer on F crossing one on B
        had BOTH flipped (two layer changes bought nothing). Walk the
        fixed column order tracking each swimmer's current layer;
        where a swimmer pair is already opposite, keep both. Every
        other branch reproduces pair_layers exactly, and the column
        ORDER is untouched (the fa4..fa9 lesson: order is the draw)."""
        if os.environ.get('BRAID_WORD', '0') != '1':
            self._recolor = {}
            return
        key = (id(cols), len(cols))
        if getattr(self, '_recolor_key', None) == key:
            return
        cur = {nm: self.tl[nm] for nm in self.swimmers}
        out = {}
        for k, col in enumerate(cols):
            for (m, p) in col:
                pm, pp = self.page[m], self.page[p]
                if pm and pp:
                    continue                    # free crossing
                if pm or pp:                    # swimmer x page: forced
                    Lm, Lp = self.pair_layers(m, p)
                    if pm is None:
                        cur[m] = Lm
                    else:
                        cur[p] = Lp
                    continue
                cm = cur.get(m, self.tl.get(m, 'F.Cu'))
                cp = cur.get(p, self.tl.get(p, 'F.Cu'))
                if cm != cp:
                    Lm, Lp = cm, cp             # already opposite: free
                else:
                    Lm, Lp = 'B.Cu', 'F.Cu'     # pair_layers' own rule
                out[(k, m, p)] = (Lm, Lp)
                cur[m], cur[p] = Lm, Lp
        self._recolor = out
        self._recolor_key = key
        diff = sum(1 for kk, v in out.items()
                   if v != self.pair_layers(kk[1], kk[2]))
        print(f'  braid-word recolor: {len(out)} swimmer-pair '
              f'crossing(s), {diff} reoriented', flush=True)

    def col_layers(self, k: int, d: str, p: str) -> Tuple[str, str]:
        """pair_layers, column-aware: under BRAID_WORD=1 the
        recolored orientation for swimmer pairs; identical otherwise."""
        r = getattr(self, '_recolor', None)
        if r:
            v = r.get((k, d, p))
            if v is not None:
                return v
        return self.pair_layers(d, p)

    def pair_layers(self, d: str, p: str) -> Tuple[str, str]:
        """(layer of the mover, layer of the passed) at a column's
        crossing. A page lane is on its page; a swimmer takes the other
        layer from the page lane it crosses; of two swimmers the mover
        is on B and the passed on F."""
        pd, pp = self.page[d], self.page[p]
        if pd and pp:
            return pd, pp
        if pp:
            return ('B.Cu' if pp == 'F.Cu' else 'F.Cu'), pp
        if pd:
            return pd, ('B.Cu' if pd == 'F.Cu' else 'F.Cu')
        return 'B.Cu', 'F.Cu'

    def columns(self, gaps: Dict[str, int], lead: Dict[str, int],
                gate: Optional[str] = None
                ) -> List[List[Tuple[str, str]]]:
        """The schedule as columns of concurrent adjacent swaps: a
        transposition sort. Each column swaps a maximal set of disjoint
        ADJACENT INVERTED pairs (the upper one has the larger target
        rank), so every swap is a real inversion and every pair is
        swapped exactly once; any permutation sorts in at most n
        columns. In each swap the MOVER is a diver (the LIS keepers are
        never movers, so they never via); when both are divers, the one
        with the larger remaining displacement moves. The gap rules
        make a layer change a column of its own: after being passed
        (front layer) a diver waits `gaps[d]` columns before it moves
        (back layer), and after moving it waits as long before it can
        be passed; `lead[d]` = columns it waits from the launch before
        its first swap (room for its dive via when nobody passes it
        first). The router's refusals raise both.

        (The earlier wave schedule derived each diver's passes from a
        serial pass over the sequence, which mis-placed a diver whose
        launch slot already was its target slot -- K32's, and the merged
        corridor's, "phantom swap". Sorting by adjacent inversions has no
        such state.)

        `gate`: the take-off rule. 'strict' holds a take-off while ANY
        diver in flight still has to cross the candidate. 'last' lets it
        go when the flying diver's only crossing left IS the candidate:
        that crossing is the flyer's landing whichever layer it happens
        on, so nothing is saved by waiting -- and the flyer cannot land
        until the candidate reaches it, which the hold forbids, so the
        strict rule deadlocked until the empty-column override (K28:
        SDQ0's seven-column chain serialised sixteen columns behind
        SDQ15's for a crossing that cost no via). 'off' never gates:
        columns for vias, when the corridor is out of columns."""
        gate = gate or os.environ.get('SCHED_GATE', 'last')
        assert gate in ('strict', 'last', 'off'), gate
        self.surfaced: List[str] = []
        trank, divers, page = self.trank, self.divers, self.page
        swim = set(self.swimmers)
        seq = list(self.launch)
        cols: List[List[Tuple[str, str]]] = []
        # per swimmer: the column and layer of its last constrained
        # crossing (a change of layer between two of its crossings
        # needs `gaps` columns between them for the via)
        last_col: Dict[str, int] = {}
        last_layer: Dict[str, str] = {}
        on_b: Set[str] = set()          # swimmers in flight (on the back layer)
        # the inverted pairs still to be swapped, per net
        todo: Dict[str, Set[str]] = {nm: set() for nm in seq}
        for i, a in enumerate(seq):
            for b in seq[i + 1:]:
                if trank[a] > trank[b]:
                    todo[a].add(b)
                    todo[b].add(a)
        guard = 0
        override = False
        while seq != self.target:
            col: List[Tuple[str, str]] = []
            used: Set[str] = set()
            gap_block = gate_block = phase_block = False
            i = 0
            while i < len(seq) - 1:
                a, b = seq[i], seq[i + 1]
                if a in used or trank[a] < trank[b]:
                    i += 1
                    continue
                # an inverted adjacent pair. Two page lanes: a FREE
                # crossing (they are on different pages by construction;
                # the B-page lane is called the mover). A page lane and a
                # swimmer: the swimmer moves, on the other layer from the
                # page. Two swimmers: the MOVER (back layer) is the one in
                # flight (a flying swimmer passed by a grounded one would
                # have to surface); of two grounded or two flying, the one
                # with more crossings left (it flies longest).
                pa, pb = page[a], page[b]
                if pa and pb:
                    assert pa != pb, f'two {pa} page lanes inverted: {a}/{b}'
                    m, p = (a, b) if pa == 'B.Cu' else (b, a)
                    free = True
                elif pa or pb:
                    m, p = (a, b) if pa is None else (b, a)
                    free = False
                else:
                    fa, fb = a in on_b, b in on_b
                    if fa != fb:
                        m, p = (a, b) if fa else (b, a)
                    else:
                        m, p = (a, b) if len(todo[a]) >= len(todo[b]) else (b, a)
                    free = False
                if free:
                    seq[i], seq[i + 1] = b, a
                    col.append((m, p))
                    used.update((a, b))
                    todo[a].discard(b)
                    todo[b].discard(a)
                    i += 2
                    continue
                Lm, Lp = self.pair_layers(m, p)
                if m in swim and m not in on_b and Lm == 'B.Cu':
                    # a take-off: wait while a swimmer in flight still has
                    # to cross this one (it would be crossed mid-flight
                    # and pay two vias) -- unless waiting stalls the
                    # whole schedule, when the flight goes anyway
                    if not override and gate != 'off' and any(
                            e in on_b and (gate == 'strict' or len(todo[e]) > 1)
                            for e in todo[m] if e in swim):
                        gate_block = True
                        i += 1
                        continue
                    if len(cols) < lead.get(m, 0):
                        gap_block = True
                        i += 1
                        continue
                if self.two_page:
                    # PHASE GROUPING. A swimmer's layer changes are the
                    # only priced events, so group its crossings by
                    # layer: defer a crossing that would change its
                    # layer while it still has PAGE partners crossable
                    # on the layer it is on (a partner of the other
                    # page keeps it). Swimmer partners do NOT hold a
                    # deferral -- counted as crossable-on-current they
                    # kept every deferral alive to the end and the
                    # overrides fired chaotically (K28: SWE 6 changes,
                    # SCAS 5, 8 surfacings). Two same-page partners
                    # cross in a fixed order -- only an inverted F x B
                    # pair between them can be re-ordered -- so a
                    # forced interleave ends in an empty column and
                    # phase_free lets ONE change through, then re-arms:
                    # a whole ungated column let every deferred swap
                    # proceed at once.
                    defer = False
                    for nm, L in ((m, Lm), (p, Lp)):
                        if nm not in swim:
                            continue
                        cur = last_layer.get(nm, self.tl[nm])
                        if L == cur:
                            continue
                        want = 'B.Cu' if cur == 'F.Cu' else 'F.Cu'
                        other = p if nm == m else m
                        if any(page.get(y) == want
                               for y in todo[nm] if y != other):
                            defer = True
                    if defer and not override:
                        phase_block = True
                        i += 1
                        continue
                # a layer change needs columns of its own: a swimmer
                # crossing on the other layer from its last crossing
                # waits gaps[d] columns after it (room for the via)
                blocked = False
                for nm, L in ((m, Lm), (p, Lp)):
                    if nm in swim and nm in last_layer and last_layer[nm] != L \
                            and len(cols) <= last_col[nm] + gaps.get(nm, 1):
                        blocked = True
                if blocked:
                    gap_block = True
                    i += 1
                    continue
                seq[i], seq[i + 1] = b, a
                col.append((m, p))
                used.update((a, b))
                todo[a].discard(b)
                todo[b].discard(a)
                for nm, L in ((m, Lm), (p, Lp)):
                    if nm in swim:
                        if L == 'B.Cu':
                            on_b.add(nm)
                        else:
                            if nm in on_b and todo[nm]:
                                # surfaced in flight with crossings still
                                # to make: it dives again, two more vias
                                self.surfaced.append(nm)
                            on_b.discard(nm)
                        last_col[nm] = len(cols)
                        last_layer[nm] = L
                        if not todo[nm]:
                            on_b.discard(nm)     # landed
                i += 2
            if not col and (gate_block or phase_block) and not gap_block:
                override = True                 # retry this column ungated
                continue
            override = False
            assert col or gap_block, ('schedule stuck', seq, self.target)
            cols.append(col)
            guard += 1
            assert guard < 4 * len(seq) + 20 + 4 * sum(gaps.values()), \
                ('schedule runaway', seq, self.target)
        # sanity: every pair swapped once, every swap a real inversion
        seen_pairs = set()
        for col in cols:
            for (d, e) in col:
                pair = frozenset((d, e))
                assert pair not in seen_pairs, f'pair {d}/{e} swapped twice'
                seen_pairs.add(pair)
                assert self.inverted(d, e), f'phantom swap {d}/{e}'
        return cols

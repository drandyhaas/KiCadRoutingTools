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
                 dest_layer: Optional[Dict[str, str]] = None):
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
        # single-page floor made 13 of them dive.
        # the LIS weighted by the ends' layers
        # ends' layers
        keep = lis_keep_weighted(ranks,
                                 [on(nm, 'F.Cu') for nm in self.launch])
        self.page = {nm: None for nm in self.launch}
        for i in keep:
            self.page[self.launch[i]] = 'F.Cu'
        rest = [nm for i, nm in enumerate(self.launch)
                if i not in keep]
        if len(rest) >= 2:
            # the B page policy is 'worst' (2026-08-31): with
            # the diamond reservation + free swimmers, the
            # worst-crosser B page ties 'lis' at every K below
            # 28 (4/10/24/32 vias, all complete) and beats it
            # at K28 (50/0 vs 56/0, human 46) -- the demoted
            # risers it strands are exactly the class the last
            # call rescues at 2 vias. 'lis' selects the old
            # length-first page, 'wmax' the weighted middle.
            if True:
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






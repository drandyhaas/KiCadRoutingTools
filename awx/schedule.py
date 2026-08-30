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
                 tooth_layer: Optional[Dict[str, str]] = None, log=None):
        self.launch = list(launch)
        self.target = list(target)
        assert sorted(self.launch) == sorted(self.target)
        self.trank = {nm: i for i, nm in enumerate(self.target)}
        self.lidx = {nm: i for i, nm in enumerate(self.launch)}
        ranks = [self.trank[nm] for nm in self.launch]
        keep = lis_keep(ranks)
        self.divers = {self.launch[i] for i in range(len(self.launch))
                       if i not in keep}
        trank, lidx, divers = self.trank, self.lidx, self.divers
        ups = sorted((d for d in divers if trank[d] < lidx[d]),
                     key=lambda nm: trank[nm])
        downs = sorted((d for d in divers if trank[d] >= lidx[d]),
                       key=lambda nm: -trank[nm])
        self.dirs = {d: (-1 if trank[d] < lidx[d] else 1) for d in divers}
        # a diver whose tooth already sits on B.Cu is BORN diving: it
        # needs no dive via if it reaches its first own crossing on B.
        # It takes no special place in the priority: moving B-born
        # divers to the front made the serial pass place them relative
        # to divers not yet moved (K32: the pass did not reach the
        # target).
        tl = tooth_layer or {}
        self.birth_b = {d for d in divers if tl.get(d) == 'B.Cu'}
        self.priority = ups + downs

        # serial pass fixes WHO passes WHOM -- and in which DIRECTION.
        # The target-vs-launch index says where a diver ends up overall,
        # but the other divers' passes shift it on the way: rot90 K15's
        # SDQ12 ends 3 slots below its launch (a "down" diver) yet must
        # pass SDQ11 going UP, once the up-divers have carried SDQ11
        # below it. The serial pass sees the sequence as it stands when
        # the diver moves, so its direction is the one the wave must
        # use; the global one deadlocked the wave.
        self.mover_set: Dict[str, List[str]] = {d: [] for d in self.priority}
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
            passed_ser = rest[want:i] if want <= i else rest[i:want]
            self.mover_set[d] = list(passed_ser)
            if want != i:
                self.dirs[d] = -1 if want < i else 1
            sseq = rest[:want] + [d] + rest[want:]
            placed.add(d)
        assert sseq == self.target, (sseq, self.target)
        if log:
            log(f'  launch order: {self.launch}')
            log(f'  target order: {self.target}  ranks: {ranks}')
            log(f'  divers ({len(divers)}): '
                f'{sorted(divers, key=lambda n: trank[n])}')
            if self.birth_b:
                log(f'  B-birth divers: {sorted(self.birth_b)}')

    def inverted(self, anm: str, bnm: str) -> bool:
        return ((self.lidx[anm] < self.lidx[bnm])
                != (self.trank[anm] < self.trank[bnm]))

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
        trank, divers = self.trank, self.divers
        seq = list(self.launch)
        cols: List[List[Tuple[str, str]]] = []
        last_passed: Dict[str, int] = {}
        last_move: Dict[str, int] = {}
        on_b: Set[str] = set()          # divers in flight (on the back layer)
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
            gap_block = gate_block = False
            i = 0
            while i < len(seq) - 1:
                a, b = seq[i], seq[i + 1]
                if a in used or trank[a] < trank[b]:
                    i += 1
                    continue
                # an inverted adjacent pair. The MOVER (back layer at
                # this crossing): the diver; of two divers the one in
                # flight (a flying diver passed by a grounded one would
                # have to surface); of two grounded or two flying, the
                # one with more crossings left (it flies longest)
                if a in divers and b in divers:
                    fa, fb = a in on_b, b in on_b
                    if fa != fb:
                        m, p = (a, b) if fa else (b, a)
                    else:
                        m, p = (a, b) if len(todo[a]) >= len(todo[b]) else (b, a)
                elif a in divers or b in divers:
                    m, p = (a, b) if a in divers else (b, a)
                else:
                    raise AssertionError(f'two keepers inverted: {a}/{b}')
                if m not in on_b:
                    # a take-off: wait while a diver in flight still has
                    # to cross this one (it would be crossed mid-flight
                    # and pay two vias) -- unless waiting stalls the
                    # whole schedule, when the flight goes anyway
                    if not override and gate != 'off' and any(
                            e in on_b and (gate == 'strict' or len(todo[e]) > 1)
                            for e in todo[m] if e in divers):
                        gate_block = True
                        i += 1
                        continue
                    if len(cols) < lead.get(m, 0):
                        gap_block = True
                        i += 1
                        continue
                # a layer change needs a column of its own: after being
                # passed (front) a diver waits gaps[d] columns before it
                # moves (back), and after moving before it is passed
                if m in last_passed and len(cols) <= last_passed[m] + gaps[m]:
                    gap_block = True
                    i += 1
                    continue
                if p in divers and p in last_move and \
                        len(cols) <= last_move[p] + gaps[p]:
                    gap_block = True
                    i += 1
                    continue
                seq[i], seq[i + 1] = b, a
                col.append((m, p))
                used.update((a, b))
                todo[a].discard(b)
                todo[b].discard(a)
                on_b.add(m)
                last_move[m] = len(cols)
                if p in divers:
                    on_b.discard(p)             # it surfaced to be passed
                    last_passed[p] = len(cols)
                if not todo[m]:
                    on_b.discard(m)             # landed
                i += 2
            if not col and gate_block and not gap_block:
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

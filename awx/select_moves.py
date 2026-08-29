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
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

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


def _site_key(m: Move) -> Optional[Tuple]:
    if m.site is None:
        return None
    return (round(m.site[0], 3), round(m.site[1], 3))


def select(menu: Dict[str, List[Move]],
           launch: Dict[str, Pt],
           only_dirs: Optional[Iterable[str]] = None,
           via_weight: float = 3.0,
           channel_weight: float = 2.0,
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
        reach = math.hypot(m.exit_pt[0] - lx, m.exit_pt[1] - ly)
        # the move's own run occupies a channel INSIDE the array, which
        # is scarcer than corridor length -- weight it above `reach`
        return via_weight * m.vias + channel_weight * _length(m) + reach

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
        for m in sorted(cand[n], key=lambda m: cost(n, m)):
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
        choice[n] = best
        _k, _a, _b = _lane_span(best)
        taken_lane.setdefault(_k, []).append((_a, _b))
        sk = _site_key(best)
        if sk is not None:
            taken_site.add(sk)
        if log:
            log(f'  {n}: {best}  (of {len(cand[n])} candidates)')
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

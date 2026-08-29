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


def _lane_key(m: Move) -> Tuple:
    """The shared resource this move consumes: a row gap (for a
    left/right escape) or a column gap (up/down), on its own layer."""
    if m.direction in ('left', 'right'):
        return ('row', round(m.exit_pt[1], 3), m.layer, m.direction)
    return ('col', round(m.exit_pt[0], 3), m.layer, m.direction)


def _site_key(m: Move) -> Optional[Tuple]:
    if m.site is None:
        return None
    return (round(m.site[0], 3), round(m.site[1], 3))


def select(menu: Dict[str, List[Move]],
           launch: Dict[str, Pt],
           only_dirs: Optional[Iterable[str]] = None,
           via_weight: float = 3.0,
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
        return via_weight * m.vias + reach

    taken_lane = {}
    taken_site = set()
    choice: Dict[str, Move] = {}
    unplaced: List[str] = []
    # most constrained first: a net with few options must choose before
    # a net with many takes its only lane
    order = sorted(cand, key=lambda n: len(cand[n]))
    for n in order:
        best = None
        for m in sorted(cand[n], key=lambda m: cost(n, m)):
            lk = _lane_key(m)
            sk = _site_key(m)
            if lk in taken_lane:
                continue
            if sk is not None and sk in taken_site:
                continue
            best = m
            break
        if best is None:
            unplaced.append(n)
            continue
        choice[n] = best
        taken_lane[_lane_key(best)] = n
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

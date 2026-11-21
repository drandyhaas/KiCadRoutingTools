"""Pair-unit lap: two legs of a pair that rip each other across laps.

A connector that interleaves the two legs of a pair (USB-C: D+ A6, D- A7,
D+ B6, D- B7 in one row) needs both legs to join their own pads in the row,
which leaves each leg one exit. When a third net (VBUS) crossed the row
before the pair routed, the second leg's exit is taken by the first leg's
join, the first leg's by the crosser, and every lap swaps which leg ships
open: the open leg blames its partner, the partner's rip blames the
crosser, and the crosser's rip is restored because the crosser cannot reroute
while both legs still stand.

The lap here breaks the cycle as one unit: strip both legs and the crossers
inside the row's box, route the two legs on the cleared row, then route the
crossers around them. route.py keeps the result only when both sub-runs
close every net they were given at the run's own rules. A leg that is
protected, locked or otherwise not rippable, a pour net or a supply rail is
never stripped: the pair is skipped.

Pure functions only (names, boxes, copper lists); route.py does the I/O.
"""
from __future__ import annotations

from typing import Dict, Iterable, List, Optional, Sequence, Tuple

# Suffix pairs, longest first so '_P' wins over 'P'.
_SUFFIX_PAIRS: Tuple[Tuple[str, str], ...] = (
    ('_P', '_N'), ('_N', '_P'),
    ('DP', 'DM'), ('DM', 'DP'),
    ('+', '-'), ('-', '+'),
    ('P', 'N'), ('N', 'P'),
)

CROSSER_CAP = 4
PAIR_CAP = 2
BOX_MARGIN_MM = 0.5


def partner_name(name: str, names: Iterable[str]) -> Optional[str]:
    """The other leg of `name`'s pair among `names`, or None."""
    pool = set(names)
    for a, b in _SUFFIX_PAIRS:
        if name.endswith(a) and len(name) > len(a):
            cand = name[: -len(a)] + b
            if cand != name and cand in pool:
                return cand
    return None


def find_exchanges(targets_left_open: Dict[str, dict],
                   names: Iterable[str]) -> List[Tuple[str, str]]:
    """(open leg, partner) for every open target whose partner is either
    open too or named among its blockers -- the exchange signature. Each
    pair is reported once, open leg first; at most PAIR_CAP pairs."""
    pool = list(names)
    seen = set()
    out: List[Tuple[str, str]] = []
    for x in sorted(targets_left_open or {}):
        y = partner_name(x, pool)
        if not y:
            continue
        key = frozenset((x, y))
        if key in seen:
            continue
        blk = (targets_left_open.get(x) or {}).get('blockers') or []
        if y in blk or y in targets_left_open:
            seen.add(key)
            out.append((x, y))
        if len(out) >= PAIR_CAP:
            break
    return out


def shared_pad_box(pads_x: Sequence[Tuple[str, float, float, float, float]],
                   pads_y: Sequence[Tuple[str, float, float, float, float]],
                   margin: float = BOX_MARGIN_MM
                   ) -> Optional[Tuple[float, float, float, float]]:
    """The box around both legs' pads on the ONE footprint where they meet
    most: the shared footprint carrying the most pads of the two legs (the
    interleaved row), the smallest box on a tie.

    Each pad is (footprint ref, x, y, half width, half height). Returns
    (x0, y0, x1, y1) grown by `margin`, or None when the legs share no
    footprint (no row to clear). One footprint only: a pair also shares its
    far part (the IC), and a box over both parts would span the board
    between them, so its "crossers" would be whatever has the most copper
    there rather than the nets crossing the row."""
    refs = {p[0] for p in pads_x} & {p[0] for p in pads_y}
    if not refs:
        return None
    every = list(pads_x) + list(pads_y)

    def _box(ref):
        sel = [p for p in every if p[0] == ref]
        return (min(p[1] - p[3] for p in sel) - margin,
                min(p[2] - p[4] for p in sel) - margin,
                max(p[1] + p[3] for p in sel) + margin,
                max(p[2] + p[4] for p in sel) + margin)

    def _rank(ref):
        b = _box(ref)
        return (-sum(1 for p in every if p[0] == ref),
                (b[2] - b[0]) * (b[3] - b[1]), ref)

    return _box(min(refs, key=_rank))


def is_power_leg(pintypes: Iterable[str]) -> bool:
    """Whether a "leg" is a supply rail rather than a signal: every pad
    names a power pin type. `V+`/`V-` and `VP`/`VN` rails match the suffix
    pairs, but a rail is not a pair to re-route as a unit."""
    kinds = [t for t in pintypes if t]
    return bool(kinds) and all(t.startswith('power') for t in kinds)


def _seg_hits_box(sx, sy, ex, ey, box) -> bool:
    """Whether segment (sx,sy)-(ex,ey) meets the box (Liang-Barsky clip)."""
    x0, y0, x1, y1 = box
    dx, dy = ex - sx, ey - sy
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, sx - x0), (dx, x1 - sx), (-dy, sy - y0), (dy, y1 - sy)):
        if p == 0:
            if q < 0:
                return False
            continue
        t = q / p
        if p < 0:
            if t > t1:
                return False
            t0 = max(t0, t)
        else:
            if t < t0:
                return False
            t1 = min(t1, t)
    return t0 <= t1


def crossers_in_box(segments: Iterable[Tuple[str, float, float, float, float]],
                    vias: Iterable[Tuple[str, float, float]],
                    box: Tuple[float, float, float, float],
                    exclude: Iterable[str]) -> List[str]:
    """Names of the nets with copper inside `box`, besides `exclude`, in
    order of how much copper each has there (most first)."""
    ex = set(exclude)
    hits: Dict[str, int] = {}
    for name, sx, sy, exx, eyy in segments:
        if name in ex or not name:
            continue
        if _seg_hits_box(sx, sy, exx, eyy, box):
            hits[name] = hits.get(name, 0) + 1
    x0, y0, x1, y1 = box
    for name, vx, vy in vias:
        if name in ex or not name:
            continue
        if x0 <= vx <= x1 and y0 <= vy <= y1:
            hits[name] = hits.get(name, 0) + 1
    return [n for n, _c in sorted(hits.items(), key=lambda kv: (-kv[1], kv[0]))]

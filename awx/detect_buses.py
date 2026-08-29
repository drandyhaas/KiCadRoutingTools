"""Detect buses / rivers from the geometry instead of naming them.

The braid splits nets into flows with a hand-written rule -- "stub below
the destination's bottom row goes to the south river" -- which is a fact
about one board, not a routing concept. But the split itself is real:
turning it off makes the entry assignment fail outright. So the groups
need to be FOUND, not declared.

Detection: pre-route every net as a TAUT string (topo_strings.relax --
the shortest obstacle-aware path it can be pulled to, ignoring the other
bus nets), then cluster the nets by how much of their length actually
runs TOGETHER. Two nets share a bus when a long stretch of one path
stays within a corridor width of the other; nets whose taut paths
diverge belong to different buses.

This is the same string machinery the campaign already uses, and it
answers the question the hand-written rule was standing in for: which
nets are going the same way?
"""
from __future__ import annotations

import math
from typing import Callable, Dict, List, Sequence, Tuple

import topo_strings as ts

Pt = Tuple[float, float]


def taut_paths(nets: Sequence[str],
               ends: Dict[str, Tuple[Pt, Pt, str]],
               obs_for: Callable[[str], 'ts.Obstacles'],
               log=None) -> Dict[str, List[Pt]]:
    """One taut string per net, tooth -> ball, avoiding static copper."""
    out = {}
    for nm in nets:
        pts, iters = ts.relax(ends[nm][0], ends[nm][1], obs_for(nm))
        out[nm] = pts
        if log:
            log(f'  {nm}: {ts.polyline_len(pts):.2f} mm, {len(pts)} pts, '
                f'{iters} iters')
    return out


def _resample(pts: List[Pt], step: float = 0.25) -> List[Pt]:
    if len(pts) < 2:
        return list(pts)
    out = [pts[0]]
    carry = 0.0
    for a, b in zip(pts, pts[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if seg < 1e-9:
            continue
        t = step - carry
        while t <= seg:
            out.append((a[0] + (b[0] - a[0]) * t / seg,
                        a[1] + (b[1] - a[1]) * t / seg))
            t += step
        carry = (carry + seg) % step
    out.append(pts[-1])
    return out


def togetherness(pa: List[Pt], pb: List[Pt], width: float) -> float:
    """Fraction of the SHORTER path's length that runs within `width`
    of the other. 1.0 = they travel together the whole way."""
    ra, rb = _resample(pa), _resample(pb)
    if not ra or not rb:
        return 0.0
    short, other = (ra, rb) if len(ra) <= len(rb) else (rb, ra)
    near = 0
    for p in short:
        best = min((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 for q in other)
        if best <= width * width:
            near += 1
    return near / float(len(short))


def cluster(nets: Sequence[str], paths: Dict[str, List[Pt]],
            width: float = 1.5, thresh: float = 0.55
            ) -> List[List[str]]:
    """Single-link clustering on togetherness. Single-link is the right
    join here: a bus is a chain of neighbours, and two nets at opposite
    edges of a wide bus need not be near each other, only near the ones
    between them."""
    idx = {n: i for i, n in enumerate(nets)}
    parent = list(range(len(nets)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i, a in enumerate(nets):
        for b in nets[i + 1:]:
            if togetherness(paths[a], paths[b], width) >= thresh:
                ra, rb = find(idx[a]), find(idx[b])
                if ra != rb:
                    parent[ra] = rb
    groups: Dict[int, List[str]] = {}
    for n in nets:
        groups.setdefault(find(idx[n]), []).append(n)
    return sorted(groups.values(), key=len, reverse=True)

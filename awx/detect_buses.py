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
    import taut_clean as tc
    out = {}
    for nm in nets:
        # CLEANLINESS, not convergence (user, 0902): relax can settle
        # in a stable cycle THROUGH a thin foreign capsule and report
        # success; relax_clean asserts point_violation None along the
        # whole string and reseeds around the offender (wrong SECTOR,
        # never a realisation problem). An INVALID string is a loud
        # line, never a silent spine input.
        pts, iters, status, n_re = tc.relax_clean(
            ends[nm][0], ends[nm][1], obs_for(nm))
        out[nm] = pts
        if log:
            log(f'  {nm}: {ts.polyline_len(pts):.2f} mm, {len(pts)} pts, '
                f'{iters} iters'
                + (f', {status} ({n_re} reseed)' if status != 'clean'
                   else ''))
        if status == 'reseeded':
            print(f'TAUT RESEEDED: {nm} ({n_re} reseed(s))', flush=True)
        elif status == 'violating':
            print(f'TAUT VIOLATING: {nm} -- {tc.relax_clean.last} '
                  '(assert-only; TAUT_RESEED=1 reseeds)', flush=True)
        elif status == 'tolerated':
            print(f'TAUT tolerated: {nm} -- {tc.relax_clean.last}',
                  flush=True)
        elif status == 'INVALID':
            print(f'TAUT INVALID: {nm} -- no clean homotopy sector found '
                  f'after {n_re} reseed(s); spine input violates copper: '
                  f'{tc.relax_clean.last}', flush=True)
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
            width: float = 1.5, thresh: float = 0.55,
            linkage: str = 'average') -> List[List[str]]:
    """Cluster nets by how much of their length runs together.

    Single link was the first choice -- a bus is a chain of neighbours,
    and two nets at opposite edges of a wide bus need not be near each
    other, only near the ones between. It CHAINS: measured on the
    coherent ladder it gives 3 sensible buses at K21 but collapses to
    ONE bus of 32 at K32 and one of 47 at K47, merging the west bundle
    with the group that approaches from below. Adding nets adds links,
    and single link needs only one.

    `average` requires a net to run with the cluster as a whole, not
    with one member of it, which is what stops the chain."""
    idx = {n: i for i, n in enumerate(nets)}
    parent = list(range(len(nets)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    sim = {}
    for i, a in enumerate(nets):
        for b in nets[i + 1:]:
            sim[(a, b)] = sim[(b, a)] = togetherness(paths[a], paths[b],
                                                     width)
    if linkage == 'single':
        for i, a in enumerate(nets):
            for b in nets[i + 1:]:
                if sim[(a, b)] >= thresh:
                    ra, rb = find(idx[a]), find(idx[b])
                    if ra != rb:
                        parent[ra] = rb
        groups: Dict[int, List[str]] = {}
        for n in nets:
            groups.setdefault(find(idx[n]), []).append(n)
        return sorted(groups.values(), key=len, reverse=True)

    # agglomerative with average linkage: merge the two clusters whose
    # MEAN pairwise togetherness is highest, while it clears `thresh`
    clus = [[n] for n in nets]

    def link(ca, cb):
        return sum(sim[(x, y)] for x in ca for y in cb) / (len(ca) * len(cb))

    while len(clus) > 1:
        best, bi, bj = -1.0, None, None
        for i in range(len(clus)):
            for j in range(i + 1, len(clus)):
                v = link(clus[i], clus[j])
                if v > best:
                    best, bi, bj = v, i, j
        if best < thresh:
            break
        clus[bi] = clus[bi] + clus[bj]
        clus.pop(bj)
    return sorted(clus, key=len, reverse=True)

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

import json
import os
import topo_strings as ts

Pt = Tuple[float, float]

_TAUT_MEMO: Dict[str, List[Pt]] = {}
_TAUT_MEMO_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               'tmp', 'taut_memo.json')
_TAUT_MEMO_LOADED = False


def _memo_load():
    """The taut-path memo persists across processes (the fanout loop and
    the braid are separate runs on the same ends and copper)."""
    global _TAUT_MEMO_LOADED
    if _TAUT_MEMO_LOADED:
        return
    _TAUT_MEMO_LOADED = True
    try:
        with open(_TAUT_MEMO_PATH, encoding='utf-8') as f:
            _TAUT_MEMO.update({k: [tuple(p) for p in v]
                               for k, v in json.load(f).items()})
    except (OSError, ValueError):
        pass


def _memo_save():
    try:
        os.makedirs(os.path.dirname(_TAUT_MEMO_PATH), exist_ok=True)
        tmp = _TAUT_MEMO_PATH + f'.{os.getpid()}.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump({k: [list(p) for p in v] for k, v in _TAUT_MEMO.items()}, f)
        os.replace(tmp, _TAUT_MEMO_PATH)
    except OSError:
        pass


def taut_paths(nets: Sequence[str],
               ends: Dict[str, Tuple[Pt, Pt, str]],
               obs_for: Callable[[str], 'ts.Obstacles'],
               log=None) -> Dict[str, List[Pt]]:
    """One taut string per net, tooth -> ball, avoiding static copper."""
    import taut_clean as tc
    out = {}
    dirty = False
    for nm in nets:
        # CLEANLINESS, not convergence (user, 0902): relax can settle
        # in a stable cycle THROUGH a thin foreign capsule and report
        # success; relax_clean asserts point_violation None along the
        # whole string and reseeds around the offender (wrong SECTOR,
        # never a realisation problem). An INVALID string is a loud
        # line, never a silent spine input.
        # MEMO (run-wide): a taut path is a pure function of its two ends
        # and the static copper it relaxes against (the run's own nets'
        # copper is excluded from that model), and the plan loop asks for
        # the same paths again at every judgment -- 14 times at K15, 82 %
        # of the fanout stage. Same inputs, same answer, no recomputation.
        obs = obs_for(nm)
        _memo_load()
        key = (f'{ends[nm][0][0]:.4f},{ends[nm][0][1]:.4f}>'
               f'{ends[nm][1][0]:.4f},{ends[nm][1][1]:.4f}@{obs.signature()}')
        hit = _TAUT_MEMO.get(key)
        if hit is not None:
            out[nm] = list(hit)
            continue
        pts, iters, status, n_re = tc.relax_clean(
            ends[nm][0], ends[nm][1], obs)
        _TAUT_MEMO[key] = [tuple(p) for p in pts]
        dirty = True
        out[nm] = pts
        if status == 'reseeded':
            print(f'TAUT RESEEDED: {nm} ({n_re} reseed(s))', flush=True)
        elif status == 'violating':
            print(f'TAUT VIOLATING: {nm} -- {tc.relax_clean.last} '
                  '(assert-only)', flush=True)
        elif status == 'tolerated':
            print(f'TAUT tolerated: {nm} -- {tc.relax_clean.last}',
                  flush=True)
        elif status == 'INVALID':
            print(f'TAUT INVALID: {nm} -- no clean homotopy sector found '
                  f'after {n_re} reseed(s); spine input violates copper: '
                  f'{tc.relax_clean.last}', flush=True)
    if dirty:
        _memo_save()
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
            width: float = 1.5, thresh: float = 0.55) -> List[List[str]]:
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
    sim = {}
    for i, a in enumerate(nets):
        for b in nets[i + 1:]:
            sim[(a, b)] = sim[(b, a)] = togetherness(paths[a], paths[b],
                                                     width)
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

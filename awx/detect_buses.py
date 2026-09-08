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

import numpy as np
from typing import Callable, Dict, List, Sequence, Tuple

import json
import os
import topo_strings as ts

Pt = Tuple[float, float]

# THE TAUT MEMO, sharded. A taut path is a pure function of its two ends
# and the obstacle model it relaxes against, so it is memoised on
# `ends@signature` and persists across processes (the fanout loop and the
# braid are separate runs on the same ends and copper). One file held
# every entry ever computed: 158 MB, 31,000 entries, loaded in full by
# every process (1.6 s, two processes per K) and REWRITTEN in full each
# time a run added an entry -- a cold K41 dumped it twenty times. Now a
# file per two-hex-digit prefix of the signature under tmp/taut_memo/,
# loaded on first touch, only dirty shards written, merged with what is
# on disk first (a parallel chain's additions survive), entries untouched
# for TAUT_MAX_AGE days dropped at write time. The old single file is
# migrated into shards once and renamed.
_TAUT_MEMO_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              'tmp', 'taut_memo')
_TAUT_MEMO_LEGACY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 'tmp', 'taut_memo.json')
TAUT_MAX_AGE = 14 * 86400
_TAUT_SHARDS: Dict[str, dict] = {}
_TAUT_DIRTY = set()
_TAUT_MIGRATED = False


def _shard_of(key):
    return key.rsplit('@', 1)[-1][:2] or '00'


def _shard_path(prefix):
    return os.path.join(_TAUT_MEMO_DIR, prefix + '.json')


def _read_shard(prefix):
    try:
        with open(_shard_path(prefix), encoding='utf-8') as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def _memo_migrate():
    """The single-file memo into shards, once: every entry it holds is
    still a valid answer, and a board that has been run pays nothing."""
    global _TAUT_MIGRATED
    if _TAUT_MIGRATED:
        return
    _TAUT_MIGRATED = True
    if not os.path.exists(_TAUT_MEMO_LEGACY):
        return
    import time as _t
    try:
        with open(_TAUT_MEMO_LEGACY, encoding='utf-8') as f:
            old = json.load(f)
    except (OSError, ValueError):
        return
    os.makedirs(_TAUT_MEMO_DIR, exist_ok=True)
    now = _t.time()
    by = {}
    for k, v in old.items():
        by.setdefault(_shard_of(k), {})[k] = {'p': v, 't': now}
    for prefix, d in by.items():
        cur = _read_shard(prefix)
        cur.update(d)
        _write_shard(prefix, cur)
    os.replace(_TAUT_MEMO_LEGACY, _TAUT_MEMO_LEGACY + '.migrated')
    print(f'taut memo: {len(old)} entries migrated into {len(by)} shards '
          f'under {os.path.relpath(_TAUT_MEMO_DIR)}', flush=True)


def _write_shard(prefix, d):
    tmp = _shard_path(prefix) + f'.{os.getpid()}.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(d, f)
    os.replace(tmp, _shard_path(prefix))


def _memo_load():
    """Kept for callers that poke the memo (probes): nothing to do, the
    shards load on first touch."""
    _memo_migrate()


def _memo_shard(prefix):
    d = _TAUT_SHARDS.get(prefix)
    if d is None:
        _memo_migrate()
        d = _TAUT_SHARDS[prefix] = _read_shard(prefix)
    return d


def _memo_get(key):
    e = _memo_shard(_shard_of(key)).get(key)
    if e is None:
        return None
    pts = e['p'] if isinstance(e, dict) else e
    return [tuple(p) for p in pts]


def _memo_put(key, pts):
    import time as _t
    prefix = _shard_of(key)
    _memo_shard(prefix)[key] = {'p': [list(p) for p in pts], 't': _t.time()}
    _TAUT_DIRTY.add(prefix)


def _memo_save():
    """Dirty shards only, merged with the shard on disk (another process
    may have added to it meanwhile), stale entries dropped."""
    import time as _t
    if not _TAUT_DIRTY:
        return
    now = _t.time()
    try:
        os.makedirs(_TAUT_MEMO_DIR, exist_ok=True)
        for prefix in sorted(_TAUT_DIRTY):
            disk = _read_shard(prefix)
            mine = _TAUT_SHARDS.get(prefix, {})
            for k, v in mine.items():
                if isinstance(v, dict):
                    disk[k] = v
                elif k not in disk:
                    disk[k] = {'p': v, 't': now}
            keep = {k: v for k, v in disk.items()
                    if not isinstance(v, dict) or now - v.get('t', now) <= TAUT_MAX_AGE}
            _write_shard(prefix, keep)
            _TAUT_SHARDS[prefix] = keep
    except OSError:
        pass
    _TAUT_DIRTY.clear()


def taut_paths(nets: Sequence[str],
               ends: Dict[str, Tuple[Pt, Pt, str]],
               obs_for: Callable[[str], 'ts.Obstacles'],
               log=None) -> Dict[str, List[Pt]]:
    """One taut string per net, tooth -> ball, avoiding static copper."""
    import taut_clean as tc
    out = {}
    dirty = False
    # The batched, convergent relaxation (taut_fast) is the default since
    # 2026-09-08: every string missing from the memo relaxed together in
    # one array, contact as a constraint (no wedged oscillation), a string
    # that touches nothing leaves after one block. Its strings are not the
    # old relaxation's, so its memo entries carry their own tag and never
    # mix with the old algorithm's inside a run. TAUT_FAST=0 keeps the old
    # per-string relaxation reachable for comparison.
    fast = os.environ.get('TAUT_FAST', '1') != '0'
    tag = '#fast' if fast else ''
    if fast:
        import taut_fast as tf
        todo = []
        for nm in nets:
            obs = obs_for(nm)
            key = (f'{ends[nm][0][0]:.4f},{ends[nm][0][1]:.4f}>'
                   f'{ends[nm][1][0]:.4f},{ends[nm][1][1]:.4f}@{obs.signature()}{tag}')
            hit = _memo_get(key)
            if hit is not None:
                out[nm] = hit
            else:
                todo.append((nm, key, obs))
        if todo:
            res = tf.relax_many([(ends[nm][0], ends[nm][1], obs) for (nm, _k, obs) in todo])
            for (nm, key, obs), (pts, it) in zip(todo, res):
                pts, it, status, n_re = tc.assess(pts, it, obs)
                _memo_put(key, pts)
                dirty = True
                out[nm] = pts
                if status == 'violating':
                    print(f'TAUT VIOLATING: {nm} -- {tc.relax_clean.last} (assert-only)', flush=True)
                elif status == 'tolerated':
                    print(f'TAUT tolerated: {nm} -- {tc.relax_clean.last}', flush=True)
        if dirty:
            _memo_save()
        return out
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
        key = (f'{ends[nm][0][0]:.4f},{ends[nm][0][1]:.4f}>'
               f'{ends[nm][1][0]:.4f},{ends[nm][1][1]:.4f}@{obs.signature()}')
        hit = _memo_get(key)
        if hit is not None:
            out[nm] = hit
            continue
        pts, iters, status, n_re = tc.relax_clean(
            ends[nm][0], ends[nm][1], obs)
        _memo_put(key, pts)
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


def togetherness(pa: List[Pt], pb: List[Pt], width: float,
                 ra=None, rb=None) -> float:
    """Fraction of the SHORTER path's length that runs within `width`
    of the other. 1.0 = they travel together the whole way. `ra` / `rb`:
    the paths already resampled (cluster resamples each net once).

    Vectorised: the squared distances are the same two subtractions,
    two squares and one sum per pair of points, in the same order, so
    the per-point minimum and the width test are bit-identical to the
    scalar loop (cluster measured 4.2 s of a 50 s K35 fanout stage,
    35M generator steps, 2026-09-06)."""
    if ra is None:
        ra = _resample(pa)
    if rb is None:
        rb = _resample(pb)
    if not ra or not rb:
        return 0.0
    short, other = (ra, rb) if len(ra) <= len(rb) else (rb, ra)
    S = np.asarray(short, dtype=float)
    O = np.asarray(other, dtype=float)
    dx = S[:, None, 0] - O[None, :, 0]
    dy = S[:, None, 1] - O[None, :, 1]
    best = (dx * dx + dy * dy).min(axis=1)
    near = int(np.count_nonzero(best <= width * width))
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
    rs = {n: _resample(paths[n]) for n in nets}
    for i, a in enumerate(nets):
        for b in nets[i + 1:]:
            sim[(a, b)] = sim[(b, a)] = togetherness(paths[a], paths[b],
                                                     width, rs[a], rs[b])
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

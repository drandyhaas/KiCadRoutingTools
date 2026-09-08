"""taut_fast.py -- the taut-string relaxation, vectorised and convergent.
The DEFAULT relaxation of detect_buses.taut_paths since 2026-09-08
(TAUT_FAST=0 selects the old per-string topo_strings.relax).

The same model as topo_strings.relax (curve shortening on a densified
polyline against discs and capsules, the ends frozen, a shortcut every
so often), with three changes that were measured to matter
(README, "The taut memo, sharded; and why the relaxation itself is
not fast", 2026-09-08):

1. CONTACT IS A CONSTRAINT, NOT A COLLISION. A point pushed out of an
   obstacle lands ON its boundary (plus a micron), not 0.01 mm past it.
   The old overshoot made every contact point move by at least 0.01 mm
   every round -- the smoothing pulled it in, the push sent it out --
   so the exit test (total movement below 1e-4 mm per point) never
   fired for a string that touched anything, and every such string ran
   all 400 rounds at a limit cycle. Without the overshoot, smooth-then-
   project is a projected gradient step: the normal part of the
   smoothing is removed by the projection, the tangential part slides
   the point along the boundary, and the displacement goes to zero.

2. A CAPSULE THE STRING CROSSES IS TRANSPARENT. A foreign track the
   string crosses transversally is a dive on a two-layer ribbon (the
   cleanliness check already classifies those as legitimate, assert-
   only); pushing the crossing points sideways every round was the
   other limit cycle. A capsule whose axis is more than ~30 degrees off
   the string's local direction no longer pushes; one the string runs
   along still does, so the mean spine stays off foreign tracks.

3. EVERY ROUND IS A FEW NUMPY OPERATIONS over all points and all nearby
   obstacles at once (a Jacobi sweep, diffusion number 1/4, stable),
   coarse to fine: 0.48 mm spacing first, then 0.12 mm.

Same inputs, same kind of answer (the same sector: an explicit step
never moves a point further than a thin obstacle's half-width, so the
string cannot jump across one); the paths differ from the old ones by
the old oscillation's amplitude and by straight instead of wiggled
crossings. Deterministic and translation-invariant.
"""
from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np

import topo_strings as ts

Pt = Tuple[float, float]

FREEZE = ts.FREEZE           # no moves this close to an endpoint
STEP = ts.STEP               # fine spacing (mm)
COARSE = 4 * ts.STEP         # coarse spacing (mm)
MARGIN = 2.0                 # candidate obstacles: within this of the string's box (mm)
OUT = 1e-3                   # a projected point sits this far outside the boundary (mm)
CROSS_SIN = 0.5              # |sin(angle)| above which a capsule is a crossing, not a wall
PUSHES = 6                   # projections per round for a point wedged between obstacles


def _arrays(obs):
    """The (surviving) discs and capsules of an obstacle model as arrays.
    A derived model (Obstacles.exclude) shares the base's lists and
    rewrites its cells, so the survivors are the cells' contents."""
    di = sorted({i for v in getattr(obs, '_near_d', {}).values() for i in v})
    ci = sorted({i for v in getattr(obs, '_near_c', {}).values() for i in v})
    D = np.array([(obs.discs[i][0], obs.discs[i][1], obs.discs[i][2]) for i in di],
                 dtype=float).reshape(-1, 3)
    C = np.array([(a[0], a[1], b[0] - a[0], b[1] - a[1], r)
                  for i in ci for (a, b, r, _n) in (obs.caps[i],)],
                 dtype=float).reshape(-1, 5)
    return D, C


def _near(D, C, P):
    """The candidates whose inflated box meets the string's box + MARGIN."""
    x0, y0 = P.min(0) - MARGIN
    x1, y1 = P.max(0) + MARGIN
    if len(D):
        md = ((D[:, 0] + D[:, 2] >= x0) & (D[:, 0] - D[:, 2] <= x1)
              & (D[:, 1] + D[:, 2] >= y0) & (D[:, 1] - D[:, 2] <= y1))
        Dn = D[md]
    else:
        Dn = D
    if len(C):
        cx0 = np.minimum(C[:, 0], C[:, 0] + C[:, 2]) - C[:, 4]
        cx1 = np.maximum(C[:, 0], C[:, 0] + C[:, 2]) + C[:, 4]
        cy0 = np.minimum(C[:, 1], C[:, 1] + C[:, 3]) - C[:, 4]
        cy1 = np.maximum(C[:, 1], C[:, 1] + C[:, 3]) + C[:, 4]
        mc = (cx1 >= x0) & (cx0 <= x1) & (cy1 >= y0) & (cy0 <= y1)
        Cn = C[mc]
    else:
        Cn = C
    return Dn, Cn


def _deepest(Q, T, D, C):
    """Per point: the deepest violated obstacle -> (depth, nx, ny), depth
    <= 0 where none. `T` is the unit tangent per point, for the capsule
    crossing rule."""
    n = len(Q)
    best = np.full(n, -np.inf)
    nx = np.ones(n)
    ny = np.zeros(n)
    if len(D):
        ex = Q[:, None, 0] - D[None, :, 0]
        ey = Q[:, None, 1] - D[None, :, 1]
        d = np.hypot(ex, ey)
        depth = D[None, :, 2] - d
        j = depth.argmax(1)
        ar = np.arange(n)
        bd = depth[ar, j]
        dd = d[ar, j]
        safe = np.where(dd < 1e-9, 1.0, dd)
        ux = np.where(dd < 1e-9, 1.0, ex[ar, j] / safe)
        uy = np.where(dd < 1e-9, 0.0, ey[ar, j] / safe)
        m = bd > best
        best[m], nx[m], ny[m] = bd[m], ux[m], uy[m]
    if len(C):
        ax, ay, sx, sy, r = C.T
        L2 = sx * sx + sy * sy
        L = np.sqrt(np.where(L2 > 0, L2, 1.0))
        px = Q[:, None, 0] - ax[None]
        py = Q[:, None, 1] - ay[None]
        t = (px * sx[None] + py * sy[None]) / np.where(L2 > 1e-12, L2, 1.0)[None]
        t = np.clip(np.where(L2[None] > 1e-12, t, 0.0), 0.0, 1.0)
        ex = px - t * sx[None]
        ey = py - t * sy[None]
        d = np.hypot(ex, ey)
        depth = r[None] - d
        # a capsule crossed transversally is a dive: transparent
        ux_ = (sx / L)[None]
        uy_ = (sy / L)[None]
        cross = np.abs(T[:, None, 0] * uy_ - T[:, None, 1] * ux_)
        depth = np.where(cross > CROSS_SIN, -np.inf, depth)
        j = depth.argmax(1)
        ar = np.arange(n)
        bd = depth[ar, j]
        dd = d[ar, j]
        safe = np.where(dd < 1e-9, 1.0, dd)
        # the normal: away from the closest axis point, or the axis normal when on it
        nax = (-sy / L)[j]
        nay = (sx / L)[j]
        ux = np.where(dd < 1e-9, nax, ex[ar, j] / safe)
        uy = np.where(dd < 1e-9, nay, ey[ar, j] / safe)
        m = bd > best
        best[m], nx[m], ny[m] = bd[m], ux[m], uy[m]
    return best, nx, ny


def _tangents(P):
    T = np.empty_like(P)
    T[1:-1] = P[2:] - P[:-2]
    T[0] = P[1] - P[0]
    T[-1] = P[-1] - P[-2]
    L = np.hypot(T[:, 0], T[:, 1])
    L = np.where(L < 1e-12, 1.0, L)
    return T / L[:, None]


CAP = 0.05        # a point moves at most this per round: below a thin
                  # capsule's radius, so a step can never jump one
REACH = 0.9       # per-point candidate obstacles: within this of the point
                  # when the lists were built (valid while it has moved less
                  # than REACH - the largest inflated radius; rebuilt sooner)
KMAX = 16         # candidates kept per point (the nearest)


def _candidates(P, D, C):
    """Per point, the indices of the nearest obstacles within REACH, as
    padded (N, KMAX) index arrays (-1 = none) -- the spatial hash, in
    numpy, rebuilt at every resample."""
    n = len(P)
    if len(D):
        dd = np.hypot(P[:, None, 0] - D[None, :, 0], P[:, None, 1] - D[None, :, 1]) - D[None, :, 2]
        dd = np.where(dd <= REACH, dd, np.inf)
        k = min(KMAX, dd.shape[1])
        od = np.argsort(dd, axis=1)[:, :k]
        vd = np.take_along_axis(dd, od, 1) < np.inf
        od = np.where(vd, od, -1)
    else:
        od = np.full((n, 0), -1)
    if len(C):
        ax, ay, sx, sy, r = C.T
        L2 = np.where(sx * sx + sy * sy > 1e-12, sx * sx + sy * sy, 1.0)
        px = P[:, None, 0] - ax[None]
        py = P[:, None, 1] - ay[None]
        t = np.clip((px * sx[None] + py * sy[None]) / L2[None], 0.0, 1.0)
        dc = np.hypot(px - t * sx[None], py - t * sy[None]) - r[None]
        dc = np.where(dc <= REACH, dc, np.inf)
        k = min(KMAX, dc.shape[1])
        oc = np.argsort(dc, axis=1)[:, :k]
        vc = np.take_along_axis(dc, oc, 1) < np.inf
        oc = np.where(vc, oc, -1)
    else:
        oc = np.full((n, 0), -1)
    return od, oc


def _deepest_k(Q, T, D, C, od, oc):
    """_deepest over each point's own candidates."""
    n = len(Q)
    best = np.full(n, -np.inf)
    nx = np.ones(n)
    ny = np.zeros(n)
    ar = np.arange(n)
    if od.shape[1]:
        valid = od >= 0
        Ds = D[np.where(valid, od, 0)]                     # (n, k, 3)
        ex = Q[:, None, 0] - Ds[:, :, 0]
        ey = Q[:, None, 1] - Ds[:, :, 1]
        d = np.hypot(ex, ey)
        depth = np.where(valid, Ds[:, :, 2] - d, -np.inf)
        j = depth.argmax(1)
        bd = depth[ar, j]
        dd = d[ar, j]
        safe = np.where(dd < 1e-9, 1.0, dd)
        ux = np.where(dd < 1e-9, 1.0, ex[ar, j] / safe)
        uy = np.where(dd < 1e-9, 0.0, ey[ar, j] / safe)
        m = bd > best
        best[m], nx[m], ny[m] = bd[m], ux[m], uy[m]
    if oc.shape[1]:
        valid = oc >= 0
        Cs = C[np.where(valid, oc, 0)]                     # (n, k, 5)
        ax, ay, sx, sy, r = (Cs[:, :, i] for i in range(5))
        L2 = sx * sx + sy * sy
        L = np.sqrt(np.where(L2 > 0, L2, 1.0))
        px = Q[:, None, 0] - ax
        py = Q[:, None, 1] - ay
        t = np.clip(np.where(L2 > 1e-12, (px * sx + py * sy) / np.where(L2 > 1e-12, L2, 1.0), 0.0), 0.0, 1.0)
        ex = px - t * sx
        ey = py - t * sy
        d = np.hypot(ex, ey)
        depth = r - d
        cross = np.abs(T[:, None, 0] * (sy / L) - T[:, None, 1] * (sx / L))
        depth = np.where(valid & (cross <= CROSS_SIN), depth, -np.inf)
        j = depth.argmax(1)
        bd = depth[ar, j]
        dd = d[ar, j]
        safe = np.where(dd < 1e-9, 1.0, dd)
        nax = (-sy / L)[ar, j]
        nay = (sx / L)[ar, j]
        ux = np.where(dd < 1e-9, nax, ex[ar, j] / safe)
        uy = np.where(dd < 1e-9, nay, ey[ar, j] / safe)
        m = bd > best
        best[m], nx[m], ny[m] = bd[m], ux[m], uy[m]
    return best, nx, ny


def _project(P, free, D, C, od, oc, T=None):
    """Push the free points out of their obstacles, onto the boundary
    plus OUT; up to PUSHES times for a point wedged between two."""
    idx = np.nonzero(free)[0]
    if T is None:
        T = _tangents(P)
    for _k in range(PUSHES):
        if not len(idx):
            break
        depth, ux, uy = _deepest_k(P[idx], T[idx], D, C, od[idx], oc[idx])
        v = depth > 0.0
        if not v.any():
            break
        sub = idx[v]
        P[sub, 0] += ux[v] * (depth[v] + OUT)
        P[sub, 1] += uy[v] * (depth[v] + OUT)
        idx = sub
    return P


def _free_mask(P, ends):
    n = len(P)
    free = np.ones(n, dtype=bool)
    free[0] = free[-1] = False
    d0 = np.hypot(P[:, 0] - ends[0][0], P[:, 1] - ends[0][1])
    d1 = np.hypot(P[:, 0] - ends[1][0], P[:, 1] - ends[1][1])
    return free & (d0 >= FREEZE) & (d1 >= FREEZE)


def _simplify(P, tol=2e-4):
    """Douglas-Peucker: the vertices of P that matter at `tol`. A relaxed
    string is straight between its contacts, and the chord along a
    straight run is the run itself, so the shortcut need not test it;
    this is what turned the shortcut from 64,000 clearance tests into a
    few hundred (profile, 2026-09-08)."""
    n = len(P)
    if n < 3:
        return P
    keep = np.zeros(n, dtype=bool)
    keep[0] = keep[-1] = True
    stack = [(0, n - 1)]
    while stack:
        a, b = stack.pop()
        if b - a < 2:
            continue
        ax, ay = P[a]
        bx, by = P[b]
        dx, dy = bx - ax, by - ay
        L = math.hypot(dx, dy)
        seg = P[a + 1:b]
        if L < 1e-12:
            d = np.hypot(seg[:, 0] - ax, seg[:, 1] - ay)
        else:
            d = np.abs((seg[:, 0] - ax) * dy - (seg[:, 1] - ay) * dx) / L
        k = int(d.argmax())
        if d[k] > tol:
            m = a + 1 + k
            keep[m] = True
            stack.append((a, m))
            stack.append((m, b))
    return P[keep]


def _resample(P, obs, step):
    return np.array(ts.densify(ts.shortcut([tuple(p) for p in _simplify(P)], obs), step), dtype=float)


def _hausdorff(A, B):
    d = np.hypot(A[:, None, 0] - B[None, :, 0], A[:, None, 1] - B[None, :, 1])
    return max(d.min(1).max(), d.min(0).max())


BLOCK = 25        # rounds between shortcut-and-densify resamples, as before


def _relax_level(P, ends, D, C, rounds, tol_h, step, obs):
    """Blocks of BLOCK Jacobi rounds, a shortcut-and-densify after each
    (the global straightening the diffusion cannot do), done when the
    resampled polyline stops moving (Hausdorff below tol_h) or `rounds`
    are spent. Returns (points, rounds used)."""
    used = 0
    prev = None
    while used < rounds:
        od, oc = _candidates(P, D, C)
        anchor = P.copy()
        for _it in range(BLOCK):
            used += 1
            n = len(P)
            if n < 3:
                break
            free = _free_mask(P, ends)
            Q = P.copy()
            Q[1:-1] = 0.5 * P[1:-1] + 0.25 * (P[:-2] + P[2:])
            dx = Q[:, 0] - P[:, 0]
            dy = Q[:, 1] - P[:, 1]
            mag = np.hypot(dx, dy)
            sc = np.where(mag > CAP, CAP / np.where(mag > 0, mag, 1.0), 1.0)
            Q[:, 0] = P[:, 0] + dx * sc
            Q[:, 1] = P[:, 1] + dy * sc
            Q[~free] = P[~free]
            Q = _project(Q, free, D, C, od, oc, _tangents(Q))
            P = Q
            if np.hypot(P[:, 0] - anchor[:, 0], P[:, 1] - anchor[:, 1]).max() > 0.5:
                od, oc = _candidates(P, D, C)
                anchor = P.copy()
        R = _resample(P, obs, step)
        if prev is not None and len(prev) and _hausdorff(R, prev) < tol_h:
            P = R
            break
        prev = R
        P = R
    return P, used


def _batch_candidates(P, own, D, C, dnet, cnet):
    """Per point, the nearest obstacles within REACH that are not the
    point's own net, as padded (M, KMAX) index arrays (-1 = none)."""
    m = len(P)
    if len(D):
        dd = np.hypot(P[:, None, 0] - D[None, :, 0], P[:, None, 1] - D[None, :, 1]) - D[None, :, 2]
        dd = np.where((dd <= REACH) & (dnet[None, :] != own[:, None]), dd, np.inf)
        k = min(KMAX, dd.shape[1])
        od = np.argpartition(dd, k - 1, axis=1)[:, :k] if dd.shape[1] > k else np.argsort(dd, axis=1)
        vd = np.take_along_axis(dd, od, 1) < np.inf
        od = np.where(vd, od, -1)
    else:
        od = np.full((m, 0), -1)
    if len(C):
        ax, ay, sx, sy, r = C.T
        L2 = np.where(sx * sx + sy * sy > 1e-12, sx * sx + sy * sy, 1.0)
        px = P[:, None, 0] - ax[None]
        py = P[:, None, 1] - ay[None]
        t = np.clip((px * sx[None] + py * sy[None]) / L2[None], 0.0, 1.0)
        dc = np.hypot(px - t * sx[None], py - t * sy[None]) - r[None]
        dc = np.where((dc <= REACH) & (cnet[None, :] != own[:, None]), dc, np.inf)
        k = min(KMAX, dc.shape[1])
        oc = np.argpartition(dc, k - 1, axis=1)[:, :k] if dc.shape[1] > k else np.argsort(dc, axis=1)
        vc = np.take_along_axis(dc, oc, 1) < np.inf
        oc = np.where(vc, oc, -1)
    else:
        oc = np.full((m, 0), -1)
    return od, oc


class _Batch:
    """Every active string's points in one array."""

    def __init__(self, polys, ends, nids):
        self.polys = [np.asarray(p, dtype=float) for p in polys]
        self.ends = ends
        self.nids = nids
        self._pack()

    def _pack(self):
        lens = [len(p) for p in self.polys]
        self.off = np.concatenate([[0], np.cumsum(lens)])
        self.P = np.concatenate(self.polys) if lens else np.zeros((0, 2))
        m = len(self.P)
        self.sid = np.repeat(np.arange(len(lens)), lens)
        self.own = np.array([self.nids[s] for s in self.sid], dtype=int) if m else np.zeros(0, dtype=int)
        idx = np.arange(m)
        first = self.off[self.sid]
        last = self.off[self.sid + 1] - 1
        self.prev = np.maximum(idx - 1, first)
        self.next = np.minimum(idx + 1, last)
        free = (idx != first) & (idx != last)
        e0 = np.array([self.ends[s][0] for s in self.sid]).reshape(-1, 2) if m else np.zeros((0, 2))
        e1 = np.array([self.ends[s][1] for s in self.sid]).reshape(-1, 2) if m else np.zeros((0, 2))
        d0 = np.hypot(self.P[:, 0] - e0[:, 0], self.P[:, 1] - e0[:, 1])
        d1 = np.hypot(self.P[:, 0] - e1[:, 0], self.P[:, 1] - e1[:, 1])
        self.free = free & (d0 >= FREEZE) & (d1 >= FREEZE)

    def split(self):
        return [self.P[self.off[s]:self.off[s + 1]] for s in range(len(self.polys))]


def _tangents_b(P, prev, nxt):
    T = P[nxt] - P[prev]
    L = np.hypot(T[:, 0], T[:, 1])
    L = np.where(L < 1e-12, 1.0, L)
    return T / L[:, None]


def _round_b(B, D, C, od, oc):
    """One Jacobi round with the trust region, then the projection, over
    the whole batch. Returns the per-point displacement."""
    P = B.P
    Q = 0.5 * P + 0.25 * (P[B.prev] + P[B.next])
    dx = Q[:, 0] - P[:, 0]
    dy = Q[:, 1] - P[:, 1]
    mag = np.hypot(dx, dy)
    sc = np.where(mag > CAP, CAP / np.where(mag > 0, mag, 1.0), 1.0)
    Q = P + np.stack([dx * sc, dy * sc], 1)
    Q[~B.free] = P[~B.free]
    T = _tangents_b(Q, B.prev, B.next)
    idx = np.nonzero(B.free)[0]
    for _k in range(PUSHES):
        if not len(idx):
            break
        depth, ux, uy = _deepest_k(Q[idx], T[idx], D, C, od[idx], oc[idx])
        v = depth > 0.0
        if not v.any():
            break
        sub = idx[v]
        Q[sub, 0] += ux[v] * (depth[v] + OUT)
        Q[sub, 1] += uy[v] * (depth[v] + OUT)
        idx = sub
    moved = np.hypot(Q[:, 0] - P[:, 0], Q[:, 1] - P[:, 1])
    B.P = Q
    return moved


def _level_many(polys, ends, nids, obss, D, C, dnet, cnet, rounds, tol_h, step):
    """One spacing level for every string at once. Strings drop out of
    the batch as they converge. Returns (polys, rounds used per string)."""
    n = len(polys)
    done = [None] * n
    used = [0] * n
    prev = [None] * n
    active = list(range(n))
    cur = list(polys)
    while active:
        B = _Batch([cur[s] for s in active], [ends[s] for s in active], [nids[s] for s in active])
        od, oc = _batch_candidates(B.P, B.own, D, C, dnet, cnet)
        anchor = B.P.copy()
        still = np.ones(len(B.P), dtype=bool)
        for _it in range(BLOCK):
            moved = _round_b(B, D, C, od, oc)
            for k, s in enumerate(active):
                used[s] += 1
            if np.hypot(B.P[:, 0] - anchor[:, 0], B.P[:, 1] - anchor[:, 1]).max() > 0.5:
                od, oc = _batch_candidates(B.P, B.own, D, C, dnet, cnet)
                anchor = B.P.copy()
            if moved.max() < 1e-6:
                break      # nothing touched anything: straight, and done
        parts = B.split()
        nxt_active = []
        for k, s in enumerate(active):
            R = _resample(parts[k], obss[s], step)
            settled = (prev[s] is not None and len(prev[s]) and _hausdorff(R, prev[s]) < tol_h) \
                or (used[s] >= rounds) or (moved[B.off[k]:B.off[k + 1]].max() < 1e-6)
            prev[s] = R
            cur[s] = R
            if settled:
                done[s] = R
            else:
                nxt_active.append(s)
        active = nxt_active
    return [done[s] if done[s] is not None else cur[s] for s in range(n)], used


def relax_many(items, rounds: int = 400, start=None):
    """[(src, dst, obs)] -> [(points, rounds used)], every string relaxed
    together. The models must derive from one base (they share their
    disc and capsule lists); each string's own net is masked out."""
    if not items:
        return []
    base = items[0][2]
    D = np.array([(x, y, r) for (x, y, r, _n) in base.discs], dtype=float).reshape(-1, 3)
    C = np.array([(a[0], a[1], b[0] - a[0], b[1] - a[1], r) for (a, b, r, _n) in base.caps],
                 dtype=float).reshape(-1, 5)
    dnet = np.array([(-1 if v is None else v) for v in base.dnets], dtype=int)
    cnet = np.array([(-1 if v is None else v) for v in base.cnets], dtype=int)
    ends = [(src, dst) for (src, dst, _o) in items]
    obss = [o for (_s, _d, o) in items]
    nids = [_own_net(o) for o in obss]
    # `start`: a polyline per item to relax from instead of the chord (the
    # exact solver's string, to be polished where its contacts did not settle)
    polys = [np.array(ts.densify(list(start[k]) if start and start[k] is not None else [src, dst], COARSE), dtype=float)
             for k, (src, dst, _o) in enumerate(items)]
    polys, u1 = _level_many(polys, ends, nids, obss, D, C, dnet, cnet, rounds, 0.02, COARSE)
    polys = [np.array(ts.densify([tuple(p) for p in P], STEP), dtype=float) for P in polys]
    polys, u2 = _level_many(polys, ends, nids, obss, D, C, dnet, cnet, rounds, 0.02, STEP)
    out = []
    for s, P in enumerate(polys):
        R = _resample(P, obss[s], STEP)
        B = _Batch([R], [ends[s]], [nids[s]])
        od, oc = _batch_candidates(B.P, B.own, D, C, dnet, cnet)
        B.P = B.P.copy()
        T = _tangents_b(B.P, B.prev, B.next)
        idx = np.nonzero(B.free)[0]
        for _k in range(PUSHES):
            if not len(idx):
                break
            depth, ux, uy = _deepest_k(B.P[idx], T[idx], D, C, od[idx], oc[idx])
            v = depth > 0.0
            if not v.any():
                break
            sub = idx[v]
            B.P[sub, 0] += ux[v] * (depth[v] + OUT)
            B.P[sub, 1] += uy[v] * (depth[v] + OUT)
            idx = sub
        out.append(([tuple(map(float, q)) for q in B.P], u1[s] + u2[s]))
    return out


def _own_net(obs):
    """The net a derived model excludes: the one whose items are in the
    base lists but in none of its cells."""
    xd = getattr(obs, '_xd', None)
    if xd:
        for i in xd:
            if obs.dnets[i] is not None:
                return int(obs.dnets[i])
    xc = getattr(obs, '_xc', None)
    if xc:
        for i in xc:
            if obs.cnets[i] is not None:
                return int(obs.cnets[i])
    return -2


def relax(src: Pt, dst: Pt, obs, rounds: int = 400) -> Tuple[List[Pt], int]:
    """Drop-in for topo_strings.relax: (points, rounds used)."""
    return relax_many([(src, dst, obs)], rounds)[0]

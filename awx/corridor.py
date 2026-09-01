#!/usr/bin/env python3
"""corridor.py -- the corridor frame: a SPINE, and (s, o) coordinates along it.

A corridor is a group of nets that flow together (detect_buses clusters
them by how much of their taut paths run side by side). Its spine is
the bundle's medial line -- the mean of the members' taut paths, relaxed
as a string against the static copper and against the corridors already
laid down, with the obstacles inflated by the bundle's half-width so a
corridor that turns a chip's corner turns it with room for its inner
lane. The spine is an arbitrary polyline: nothing here reads a face, an
axis, or a chip's orientation.

Every quantity the braid reasons about lives in the spine's own frame:
`s` = arc-length along the spine (the role x plays in a straight
corridor), `o` = signed offset across it (positive to the RIGHT of the
direction of travel, which in KiCad's y-down frame is south for a
corridor heading east). A lane is a polyline in (s, o); a schedule
column is a value of s; the neighbour bands, the required-layer
intervals and the virtual copper are all functions of s. The mapping
back to the board is `project` (board -> (s, o), vectorised for the
router's grid cells) and `lane_xy` ((s, o) polyline -> board polyline).

At a corner of the spine the mapping is what a bundle of tracks
actually does: on the OUTER side of the turn a lane at offset o rounds
the corner on an arc of radius |o| (every cell in that wedge projects
to the corner's s, at its own radius), on the INNER side the two offset
lines meet at the mitre point. Adjacent lanes stay a pitch apart
through the turn either way. No swap column is ever placed inside a
corner's wedge, so a lane's o is constant through it.
"""
from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

import topo_strings as ts

Pt = Tuple[float, float]


# ---------------------------------------------------------------- polylines

def simplify(pts: Sequence[Pt], tol: float = 0.03) -> List[Pt]:
    """Douglas-Peucker, keeping both ends."""
    pts = [tuple(p) for p in pts]
    if len(pts) < 3:
        return pts

    def rec(a, b):
        best, bi = 0.0, -1
        for i in range(a + 1, b):
            d = ts.seg_pt_dist(pts[a], pts[b], pts[i])
            if d > best:
                best, bi = d, i
        if best <= tol:
            return [pts[a], pts[b]]
        return rec(a, bi)[:-1] + rec(bi, b)
    out = rec(0, len(pts) - 1)
    # drop consecutive duplicates
    keep = [out[0]]
    for p in out[1:]:
        if math.hypot(p[0] - keep[-1][0], p[1] - keep[-1][1]) > 1e-6:
            keep.append(p)
    return keep


def resample(pts: Sequence[Pt], n: int) -> List[Pt]:
    """n points at equal fractions of the polyline's length."""
    if len(pts) == 1:
        return [tuple(pts[0])] * n
    seg = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])]
    L = sum(seg)
    if L < 1e-9:
        return [tuple(pts[0])] * n
    out = []
    for k in range(n):
        t = L * k / (n - 1)
        acc = 0.0
        for (a, b), sl in zip(zip(pts, pts[1:]), seg):
            if acc + sl >= t - 1e-12 and sl > 0:
                u = (t - acc) / sl
                out.append((a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1])))
                break
            acc += sl
        else:
            out.append(tuple(pts[-1]))
    return out


def mean_path(paths: Sequence[Sequence[Pt]], n: int = 60) -> List[Pt]:
    """The pointwise mean of the paths, each resampled by fraction of
    its length: the bundle's medial line, in the homotopy class the
    members' taut paths chose (which side of a chip they went)."""
    rs = [resample(p, n) for p in paths]
    return [(sum(r[k][0] for r in rs) / len(rs),
             sum(r[k][1] for r in rs) / len(rs)) for k in range(n)]


def polyline_len(pts) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(pts, pts[1:]))


def point_before_end(pts: Sequence[Pt], back: float) -> Pt:
    """The point `back` mm before the end of the polyline, along it."""
    acc = 0.0
    for a, b in zip(reversed(pts[:-1]), reversed(pts[1:])):
        sl = math.hypot(b[0] - a[0], b[1] - a[1])
        if acc + sl >= back:
            u = (back - acc) / max(sl, 1e-12)
            return (b[0] + u * (a[0] - b[0]), b[1] + u * (a[1] - b[1]))
        acc += sl
    return tuple(pts[0])


def point_after_start(pts: Sequence[Pt], fwd: float) -> Pt:
    return point_before_end(list(reversed(pts)), fwd)


def cluster_corridors(names: Sequence[str], paths, teeth, stubs,
                      pad_clear, D: float = 4.0, log=None,
                      spine_fn=None, dest_ref=None, centres=None,
                      src_centres=None) -> List[List[str]]:
    """Group nets into corridors.

    Two nets share a corridor when their stub ends are on the same
    destination array within `D` of each other and their taut paths
    arrive from the same side of it -- single linkage, so a face of
    stubs chains up whatever its length. A group is then
    checked against the spine it would get: a member whose stub lies
    past that spine's end, or whose tooth lies behind its start, is
    kept only if its OWN lane can run that stretch without crossing a
    pad field (a stub along the next face of the chip can: the lane
    runs past the corner outside the array; a stub on the far face
    cannot). The run is tried at the end's own offset and then stepped
    OUTWARD from its array (`centres` / `src_centres`, the arrays'
    centres per net) by a lane pitch at a time, up to a block's width:
    a flank end never runs at its own offset -- the braid jogs it out
    into a join or exit block first -- and at its own offset a flank
    tooth one row further in grazes the array's outer pads (K28 SODT1,
    0.06 mm from U1's outer row, split into a corridor of its own that
    then collided with the main one; at K21 the same tooth passed
    because the spine leaned 2 degrees the other way). The others are
    split off and regroup among themselves. Nothing here names a face:
    the test is whether one spine can serve every member."""
    idx = {n: i for i, n in enumerate(names)}
    parent = list(range(len(names)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(a, b):
        ra, rb = find(idx[a]), find(idx[b])
        if ra != rb:
            parent[ra] = rb
    # the direction each net ARRIVES from, seen from its destination
    # array's centre: its taut path 2 mm before the stub. Two stubs link
    # when they are close and arrive from within 60 degrees of each
    # other -- a face of stubs chains along, a corner is crossed in two
    # or three links, the far face is never reached. (Rungs between the
    # stubs themselves were tried first and clip the neighbouring pads
    # of whatever row a stub exits along.)
    appr = {nm: point_before_end(paths[nm], 2.0) for nm in names}
    arr = {}
    for nm in names:
        c = centres[nm]
        arr[nm] = _unit((appr[nm][0] - c[0], appr[nm][1] - c[1]))
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if dest_ref[a] != dest_ref[b]:
                continue
            dist = math.hypot(stubs[a][0] - stubs[b][0], stubs[a][1] - stubs[b][1])
            if dist > D:
                continue
            ang = _angle(arr[a], arr[b])
            if ang <= 60.0:
                union(a, b)
            elif log and dist < 2.0:
                log(f'  no link {a}-{b} ({dist:.2f} mm apart): arrive '
                    f'{ang:.0f} deg apart')
    groups: dict = {}
    for n in names:
        groups.setdefault(find(idx[n]), []).append(n)
    todo = sorted(groups.values(), key=len, reverse=True)
    out = []
    while todo:
        grp = todo.pop(0)
        if len(grp) == 1:
            out.append(grp)
            continue
        if spine_fn is not None:
            sp = spine_fn(grp)
        else:
            sp = Spine(simplify(mean_path([paths[nm] for nm in grp]), 0.1))
        keep, split = [], []
        P0, d0 = sp.P[0], sp.d[0]
        Pn, dn = sp.P[-1], sp.d[-1]

        def run_clear(end, far, d, centre):
            """Can a lane run from `end` to `far` (the stretch along
            the spine direction `d`) -- at the end's own offset, or
            jogged outward from the array centred at `centre` by up
            to a block's width, the jog itself clear too."""
            if pad_clear(end, far):
                return True
            if centre is None:
                return False
            nx, ny = end[0] - centre[0], end[1] - centre[1]
            # the outward component across the spine direction
            along = nx * d[0] + ny * d[1]
            nx, ny = nx - along * d[0], ny - along * d[1]
            h = math.hypot(nx, ny)
            if h < 1e-9:
                return False
            nx, ny = nx / h, ny / h
            for k in range(1, 4):
                off = 0.35 * k
                e2 = (end[0] + off * nx, end[1] + off * ny)
                f2 = (far[0] + off * nx, far[1] + off * ny)
                if pad_clear(end, e2) and pad_clear(e2, f2):
                    return True
            return False

        for nm in grp:
            # how far past the spine's end the stub lies, and how far
            # behind its start the tooth lies, along the spine; a member
            # is reachable when its OWN lane can run that stretch
            # without crossing a pad field (a stub one face round the
            # corner reads as "past the end, through the chip"; a flank
            # tooth reads as "behind the start, clear")
            t_e = ((stubs[nm][0] - Pn[0]) * dn[0] + (stubs[nm][1] - Pn[1]) * dn[1])
            t_0 = ((teeth[nm][0] - P0[0]) * d0[0] + (teeth[nm][1] - P0[1]) * d0[1])
            ok = True
            if t_e > 1.0:
                back_pt = (stubs[nm][0] - t_e * dn[0], stubs[nm][1] - t_e * dn[1])
                ok = run_clear(stubs[nm], back_pt, dn,
                               (centres or {}).get(nm))
            if ok and t_0 < -1.0:
                fwd_pt = (teeth[nm][0] - t_0 * d0[0], teeth[nm][1] - t_0 * d0[1])
                ok = run_clear(teeth[nm], fwd_pt, d0,
                               (src_centres or {}).get(nm))
            (keep if ok else split).append(nm)
        if split and keep:
            if log:
                log(f'  corridor split: {split} cannot be reached by the '
                    f'spine of {keep}')
            out.append(keep)
            # the split-off nets regroup among themselves by proximity
            sub = cluster_corridors(split, paths, teeth, stubs, pad_clear, D,
                                    spine_fn=spine_fn, dest_ref=dest_ref,
                                    centres=centres, src_centres=src_centres)
            todo = sorted(todo + sub, key=len, reverse=True)
        else:
            out.append(grp)
    return sorted(out, key=len, reverse=True)


# ---------------------------------------------------------------- the spine

class Spine:
    """A polyline with arc-length and a right-hand normal per segment."""

    def __init__(self, pts: Sequence[Pt]):
        P = [tuple(p) for p in pts]
        clean = [P[0]]
        for p in P[1:]:
            if math.hypot(p[0] - clean[-1][0], p[1] - clean[-1][1]) > 1e-6:
                clean.append(p)
        assert len(clean) >= 2, 'a spine needs two distinct points'
        self.pts = clean
        self.P = np.array(clean, dtype=float)
        D = self.P[1:] - self.P[:-1]
        self.len = np.hypot(D[:, 0], D[:, 1])
        self.d = D / self.len[:, None]
        self.nrm = np.stack([-self.d[:, 1], self.d[:, 0]], axis=1)
        self.S = np.concatenate([[0.0], np.cumsum(self.len)])
        self.L = float(self.S[-1])
        self.n = len(self.len)
        # signed turn at each interior vertex: > 0 = a right turn
        # (toward +o), < 0 = left; the inner side of the turn is the
        # side the spine turns toward
        cr = (self.d[:-1, 0] * self.d[1:, 1] - self.d[:-1, 1] * self.d[1:, 0])
        dt = (self.d[:-1] * self.d[1:]).sum(axis=1)
        self.turn = np.degrees(np.arctan2(cr, dt))          # per vertex 1..n-1

    # -- scalar helpers
    def seg_of(self, s: float) -> int:
        k = int(np.searchsorted(self.S, s, side='right') - 1)
        return max(0, min(self.n - 1, k))

    def xy(self, s: float, o: float) -> Pt:
        k = self.seg_of(s)
        t = s - self.S[k]
        return (float(self.P[k, 0] + t * self.d[k, 0] + o * self.nrm[k, 0]),
                float(self.P[k, 1] + t * self.d[k, 1] + o * self.nrm[k, 1]))

    def dir(self, s: float) -> Pt:
        k = self.seg_of(s)
        return (float(self.d[k, 0]), float(self.d[k, 1]))

    def corners(self, min_deg: float = 5.0) -> List[Tuple[int, float, float]]:
        """(vertex index, s, turn degrees) for every real corner."""
        return [(j + 1, float(self.S[j + 1]), float(self.turn[j]))
                for j in range(self.n - 1) if abs(self.turn[j]) >= min_deg]

    # -- vectorised projection
    def project(self, xs, ys):
        """(s, o) of every point; arrays of one shape. The nearest point
        of the spine decides: a point whose nearest point is an interior
        VERTEX is in that corner's outer wedge, at s = the corner's s and
        o = its signed distance from the corner (the arc radius)."""
        X = np.asarray(xs, dtype=float)
        Y = np.asarray(ys, dtype=float)
        shape = X.shape
        X = X.ravel()
        Y = Y.ravel()
        best_d2 = np.full(X.shape, np.inf)
        best_s = np.zeros(X.shape)
        best_o = np.zeros(X.shape)
        best_k = np.zeros(X.shape, dtype=int)
        best_t = np.zeros(X.shape)
        for k in range(self.n):
            px, py = self.P[k]
            dx, dy = self.d[k]
            nx, ny = self.nrm[k]
            Lk = self.len[k]
            t = (X - px) * dx + (Y - py) * dy
            tc = np.clip(t, 0.0, Lk)
            fx = px + tc * dx
            fy = py + tc * dy
            d2 = (X - fx) ** 2 + (Y - fy) ** 2
            o = (X - px) * nx + (Y - py) * ny
            better = d2 < best_d2 - 1e-12
            best_d2 = np.where(better, d2, best_d2)
            best_s = np.where(better, self.S[k] + tc, best_s)
            best_o = np.where(better, o, best_o)
            best_k = np.where(better, k, best_k)
            best_t = np.where(better, tc, best_t)
        # outer wedges: nearest point is an interior vertex
        at_end = (best_t >= self.len[best_k] - 1e-9) & (best_k < self.n - 1)
        at_start = (best_t <= 1e-9) & (best_k > 0)
        wedge = at_end | at_start
        if wedge.any():
            vk = np.where(at_end, best_k + 1, best_k)
            # a point whose nearest point is a vertex lies on the OUTER
            # side of that corner (an inner point always has a foot on
            # one of the two legs), so its sign is the outer side's:
            # opposite to the turn
            outer = -np.sign(self.turn[np.clip(vk - 1, 0, self.n - 2)])
            outer = np.where(outer == 0, 1.0, outer)
            r = np.sqrt(best_d2)
            best_o = np.where(wedge, outer * r, best_o)
            best_s = np.where(wedge, self.S[vk], best_s)
        return best_s.reshape(shape), best_o.reshape(shape)

    def project_pt(self, p: Pt) -> Tuple[float, float]:
        s, o = self.project(np.array([p[0]]), np.array([p[1]]))
        return float(s[0]), float(o[0])

    def extend(self, back: float, fwd: float) -> 'Spine':
        """The same spine with its first leg extended backwards by
        `back` and its last leg forwards by `fwd`."""
        pts = list(self.pts)
        if back > 0:
            dx, dy = float(self.d[0, 0]), float(self.d[0, 1])
            pts[0] = (pts[0][0] - back * dx, pts[0][1] - back * dy)
        if fwd > 0:
            dx, dy = float(self.d[-1, 0]), float(self.d[-1, 1])
            pts[-1] = (pts[-1][0] + fwd * dx, pts[-1][1] + fwd * dy)
        return Spine(pts)

    def lane_xy(self, so: Sequence[Tuple[float, float]]) -> List[Pt]:
        """Board polyline of a lane given as (s, o) points with s
        non-decreasing. Corners of the spine crossed by a piece are
        rendered as the offset curve really is: an arc of radius |o| on
        the outer side, the mitre point on the inner side."""
        out: List[Pt] = []

        def push(p):
            if not out or math.hypot(p[0] - out[-1][0],
                                     p[1] - out[-1][1]) > 1e-6:
                out.append(p)
        for (sa, oa), (sb, ob) in zip(so, so[1:]):
            push(self.xy(sa, oa))
            # vertices strictly inside (sa, sb)
            for j in range(1, self.n):
                Sj = self.S[j]
                if not (sa + 1e-9 < Sj < sb - 1e-9):
                    continue
                if abs(self.turn[j - 1]) < 1e-6:
                    continue
                o = oa + (ob - oa) * (Sj - sa) / max(sb - sa, 1e-12)
                n1 = self.nrm[j - 1]
                n2 = self.nrm[j]
                V = self.P[j]
                if o * self.turn[j - 1] > 0:
                    # inner side: the mitre of the two offset lines
                    den = 1.0 + float(n1 @ n2)
                    m = (n1 + n2) / max(den, 1e-6)
                    push((float(V[0] + o * m[0]), float(V[1] + o * m[1])))
                elif abs(o) > 1e-9:
                    # outer side: an arc from o*n1 to o*n2 about V
                    a1 = math.atan2(n1[1], n1[0])
                    ang = math.radians(self.turn[j - 1])
                    steps = max(1, int(math.ceil(abs(ang) / math.radians(30))))
                    for q in range(steps + 1):
                        a = a1 + ang * q / steps
                        push((float(V[0] + o * math.cos(a)),
                              float(V[1] + o * math.sin(a))))
                else:
                    push((float(V[0]), float(V[1])))
            push(self.xy(sb, ob))
        if len(out) == 1:
            out.append(out[0])
        return out


# ---------------------------------------------------------------- obstacles

class RampedObstacles:
    """The obstacle set a spine is relaxed against: the base obstacles
    (static copper), inflated by up to `H` -- the bundle's half-width --
    with the inflation ramping from nothing within R0 of either endpoint
    to full at R1, because a bundle is not yet a bundle where its lanes
    are still joining or already leaving. `extra` obstacles (the tubes
    of corridors already laid) are ramped the same way, all the way to
    zero radius at the ends: a corridor's exits may legitimately sit
    among another corridor's stubs."""

    def __init__(self, base: 'ts.Obstacles', ends: Tuple[Pt, Pt], H: float,
                 extra: Optional[List[Tuple[Pt, Pt, float]]] = None,
                 R0: Optional[float] = None, R1: Optional[float] = None):
        self.base = base
        self.ends = ends
        self.H = H
        self.R0 = H if R0 is None else R0
        self.R1 = 2 * H if R1 is None else R1
        # the extra capsules go into a hashed Obstacles at their FULL
        # inflated radius; a query shrinks them back by the part of the
        # inflation the ramp has not reached yet
        self.extra = None
        if extra:
            self.extra = ts.Obstacles()
            for (a, b, rad) in extra:
                self.extra.add_cap(a, b, rad + H, 'laid')
            self.extra.build()

    def ramp(self, p: Pt) -> float:
        d = min(math.hypot(p[0] - e[0], p[1] - e[1]) for e in self.ends)
        if self.R1 <= self.R0:
            return 1.0 if d >= self.R0 else 0.0
        return max(0.0, min(1.0, (d - self.R0) / (self.R1 - self.R0)))

    def point_violation(self, p: Pt, pad: float = 0.0):
        r = self.ramp(p)
        worst = self.base.point_violation(p, pad=pad + self.H * r)
        if self.extra is not None and r > 0:
            # radius held: rad + H; wanted: (rad + H) * r
            # -> shrink by (rad + H) * (1 - r); rad is per capsule, so
            # approximate with the common H (rad << H for a lane tube)
            v = self.extra.point_violation(p, pad=pad - self.H * (1.0 - r))
            if v is not None and (worst is None or v[0] > worst[0]):
                worst = v
        return worst

    def seg_clear(self, a: Pt, b: Pt) -> bool:
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(L / 0.25))
        for k in range(n + 1):
            t = k / n
            p = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
            if self.point_violation(p) is not None:
                return False
        return True


def relax_path(init: Sequence[Pt], obs, rounds: int = 150) -> List[Pt]:
    """topo_strings.relax from a given initial polyline (it starts from
    the chord): the elastic band settles in the homotopy class of
    `init`, pushed out of `obs`."""
    pts = ts.densify(list(init))
    ends = (pts[0], pts[-1])
    it = 0
    for it in range(rounds):
        moved = 0.0
        for i in range(1, len(pts) - 1):
            p = pts[i]
            if ts.d2(p, ends[0]) < ts.FREEZE ** 2 or \
                    ts.d2(p, ends[1]) < ts.FREEZE ** 2:
                continue
            q = (0.5 * p[0] + 0.25 * pts[i - 1][0] + 0.25 * pts[i + 1][0],
                 0.5 * p[1] + 0.25 * pts[i - 1][1] + 0.25 * pts[i + 1][1])
            for _k in range(6):
                v = obs.point_violation(q)
                if v is None:
                    break
                depth, (ux, uy) = v
                q = (q[0] + ux * (depth + 0.01), q[1] + uy * (depth + 0.01))
            moved += math.hypot(q[0] - p[0], q[1] - p[1])
            pts[i] = q
        if it % 25 == 24:
            pts = ts.densify(ts.shortcut(pts, obs))
        if moved < 1e-4 * len(pts):
            break
    return ts.densify(ts.shortcut(pts, obs))


def _unit(v: Pt) -> Pt:
    h = math.hypot(v[0], v[1])
    return (v[0] / h, v[1] / h) if h > 1e-12 else (1.0, 0.0)


def _angle(u: Pt, v: Pt) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, u[0] * v[0] + u[1] * v[1]))))


def flow_dir(path: Sequence[Pt], end_dir: Pt, at_start: bool,
             probe: float = 1.5, snap_deg: float = 45.0) -> Pt:
    """The direction a net FLOWS as it leaves its tooth (or arrives at
    its stub): the stub's own escape direction when the taut path keeps
    to it, else the taut path's direction `probe` mm in -- the net
    turns at once, and where it turns to is the flow (a flank tooth
    points south and flows east). No face is read: `end_dir` comes from
    the copper of the stub itself."""
    if polyline_len(path) < 2 * probe:
        q = path[-1] if at_start else path[0]
        p = path[0] if at_start else path[-1]
        taut = _unit((q[0] - p[0], q[1] - p[1]))
    elif at_start:
        q = point_after_start(path, probe)
        taut = _unit((q[0] - path[0][0], q[1] - path[0][1]))
    else:
        q = point_before_end(path, probe)
        taut = _unit((path[-1][0] - q[0], path[-1][1] - q[1]))
    return end_dir if _angle(taut, end_dir) <= snap_deg else taut


def build_spine(paths: Sequence[Sequence[Pt]], base_obs: 'ts.Obstacles',
                H: float, extra=None, log=None,
                teeth: Optional[Sequence[Pt]] = None,
                stubs: Optional[Sequence[Pt]] = None,
                tooth_dirs: Optional[Sequence[Pt]] = None,
                stub_dirs: Optional[Sequence[Pt]] = None,
                relax: bool = True) -> Spine:
    """The corridor's spine.

    Its two END LEGS follow the flow at the teeth and at the stubs
    (flow_dir): from the teeth's centroid along the launch flow for as
    far as the teeth spread, and into the stubs' centroid along the
    arrival flow. When the two flows are parallel the spine is ONE
    straight line through the midpoint (the channel between two facing
    arrays, however their centroids are offset -- the offset is the
    lanes' morph, not a tilt of the frame). The middle is the members'
    mean path relaxed against the ramped obstacles, so it bends only
    where something is in the way; a straight channel stays straight."""
    if teeth is None or stubs is None or not tooth_dirs or not stub_dirs:
        init = mean_path(paths)
        obs = RampedObstacles(base_obs, (init[0], init[-1]), H, extra=extra)
        pts = relax_path(init, obs) if relax else list(init)
        return Spine(simplify(pts, 0.08))
    Ct = (sum(p[0] for p in teeth) / len(teeth), sum(p[1] for p in teeth) / len(teeth))
    Cs = (sum(p[0] for p in stubs) / len(stubs), sum(p[1] for p in stubs) / len(stubs))
    ut = [flow_dir(pth, d, True) for pth, d in zip(paths, tooth_dirs)]
    us = [flow_dir(pth, (-d[0], -d[1]), False) for pth, d in zip(paths, stub_dirs)]
    u_t = _unit((sum(u[0] for u in ut), sum(u[1] for u in ut)))
    u_s = _unit((sum(u[0] for u in us), sum(u[1] for u in us)))
    if log:
        log(f'    flow: launch ({u_t[0]:.2f},{u_t[1]:.2f}) arrival '
            f'({u_s[0]:.2f},{u_s[1]:.2f})'
            + ('' if _angle(u_t, u_s) <= 30.0 else f' -- bends {_angle(u_t, u_s):.0f} deg'))

    def spread(pts, c, u):
        return max([0.0] + [(p[0] - c[0]) * u[0] + (p[1] - c[1]) * u[1] for p in pts])
    straight = _angle(u_t, u_s) <= 30.0
    if straight:
        u = _unit((u_t[0] + u_s[0], u_t[1] + u_s[1]))
        mid = ((Ct[0] + Cs[0]) / 2, (Ct[1] + Cs[1]) / 2)
        ta = (Ct[0] - mid[0]) * u[0] + (Ct[1] - mid[1]) * u[1]
        tb = (Cs[0] - mid[0]) * u[0] + (Cs[1] - mid[1]) * u[1]
        a = (mid[0] + ta * u[0], mid[1] + ta * u[1])
        b = (mid[0] + tb * u[0], mid[1] + tb * u[1])
        u_t = u_s = u
    else:
        a, b = Ct, Cs
    R_t = spread(teeth, a, u_t) + 0.5
    R_s = spread(stubs, b, (-u_s[0], -u_s[1])) + 0.5
    p1 = (a[0] + R_t * u_t[0], a[1] + R_t * u_t[1])
    p2 = (b[0] - R_s * u_s[0], b[1] - R_s * u_s[1])
    gap = (p2[0] - p1[0]) * u_t[0] + (p2[1] - p1[1]) * u_t[1]
    if gap <= 0.5:
        # the two zones overlap: nothing left to relax
        return Spine(simplify([a, b], 0.08))
    obs = RampedObstacles(base_obs, (Ct, Cs), H, extra=extra)
    if straight:
        init = [p1, p2]
        # a clear straight channel needs no relaxing (and relaxing it
        # against the ramped obstacles can only add wiggles)
        if obs.seg_clear(p1, p2):
            relax = False
    else:
        M = mean_path(paths)
        keep = [p for p in M
                if math.hypot(p[0] - Ct[0], p[1] - Ct[1]) > R_t
                and math.hypot(p[0] - Cs[0], p[1] - Cs[1]) > R_s]
        init = [p1] + keep + [p2]
    pts = relax_path(init, obs) if relax else list(init)
    sp = simplify([a] + list(pts) + [b], 0.08)
    # a bundle never doubles back: a vertex the string folded at (a
    # string stuck between two pushes) is dropped, and the polyline
    # re-simplified, until every turn is a real corner
    while len(sp) > 2:
        s_ = Spine(sp)
        bad = [j + 1 for j in range(s_.n - 1) if abs(s_.turn[j]) > 120.0]
        if not bad:
            break
        sp = simplify([p for j, p in enumerate(sp) if j not in set(bad)], 0.08)
    # a spine with MANY near-right-angle corners is not a corridor
    # axis -- it is the elastic band OSCILLATING between pushes, which
    # the fold filter (>120 deg) never catches (K35 corridor
    # SA6/SA4/SBA1: 56 mean pts -> 1023 relaxed -> 34 vertices with 32
    # corners, 47 mm of spine for a ~20 mm run -- rendered as a white
    # scribble-ball, and every lane's frame-mapped centreline curled
    # with it). The frame is a coordinate AXIS, not a route: fall back
    # to the straight chord and let the lanes morph.
    if sum(1 for _i, _s, t in Spine(sp).corners() if abs(t) > 80.0) > 4:
        if log:
            log(f'    spine DEGENERATE ({len(sp)} vertices, '
                f'{polyline_len(sp):.1f} mm) -- straight-chord fallback')
        sp = simplify([a, b], 0.08)
    if log:
        log(f'    spine: {len(init)} mean pts -> {len(pts)} relaxed -> '
            f'{len(sp)} vertices, {polyline_len(sp):.2f} mm, '
            f'corners {[round(t) for _i, _s, t in Spine(sp).corners()]}')
    return Spine(sp)


# ---------------------------------------------------------------- distances


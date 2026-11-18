#!/usr/bin/env python3
"""corridor.py -- the corridor frame: a SPINE, and (s, o) coordinates along it.

A spine is the polyline a bundle of lanes runs along: the whole route's
trunk (whole_frame), or a ring wound round a destination's pads
(build_wrap_spine). It is an arbitrary polyline: nothing here reads a
face, an axis, or a chip's orientation.

Every quantity of a bundle lives in the spine's own frame:
`s` = arc-length along the spine (the role x plays in a straight
corridor), `o` = signed offset across it (positive to the RIGHT of the
direction of travel, which in KiCad's y-down frame is south for a
corridor heading east). A lane is a polyline in (s, o). The mapping
back to the board is `project` (board -> (s, o), vectorised for the
router's grid cells), and `lane_line` and `lane_xy` ((s, o) polyline ->
board polyline).

At a corner of the spine the offset lines of a lane at o meet at the
MITRE point on both sides (the corner displaced along the bisector by
o / cos(turn/2)); a cell in the outer wedge projects to the corner's s
at the larger of its two offsets from the legs' lines, which is the
offset polyline that passes through it. Adjacent lanes stay a pitch
apart along the legs and 1/cos(turn/2) of it across the mitre. The
outer side used to be an ARC of radius |o| (what a bundle of curved
tracks does); on an octilinear router the band of an arc is a
staircase, and every lane took one round every corner (channel article
K28: 4021 segments for 28 lanes against 1166 on the chord).

cluster_corridors groups nets by where they arrive at an array (the
fanout's planned buses), and the functions after the Spine read a part
against the frames of a trunk and its rings (whole_geo).
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

    # iterative (a spine of thousands of relaxed points overflowed the
    # recursive form: RecursionError in a K51 corner corridor)
    keep_idx = {0, len(pts) - 1}
    stack = [(0, len(pts) - 1)]
    while stack:
        a, b = stack.pop()
        best, bi = 0.0, -1
        for i in range(a + 1, b):
            d = ts.seg_pt_dist(pts[a], pts[b], pts[i])
            if d > best:
                best, bi = d, i
        if best > tol:
            keep_idx.add(bi)
            stack.append((a, bi))
            stack.append((bi, b))
    out = [pts[i] for i in sorted(keep_idx)]
    # drop consecutive duplicates
    keep = [out[0]]
    for p in out[1:]:
        if math.hypot(p[0] - keep[-1][0], p[1] - keep[-1][1]) > 1e-6:
            keep.append(p)
    return keep


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
        sp = spine_fn(grp)
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

        def wrap_clear(end, d, centre):
            """A stub on the array's FAR face. Its lane cannot run back
            along the spine into the pad field, but it can arrive from
            BEYOND the array: the bundle's outermost lane continues past
            the far face and a leg turns in along that face to the stub
            -- the human's homotopy for K35's SA9/SA13/SA8 (corridor 0's
            outermost lanes round DU1's east corner), which a corridor
            of their own, spined straight through the main bundle,
            could only plan as fiction (all three refused in-band every
            attempt, re-laid at last call; K41: three of the eight open
            nets). Reachable when a short run FORWARD from the stub
            (away from the array, a pitch to a block's width) is clear
            and so is a leg from there, out across the array's width on
            the side the stub is on (either side when it is centred)."""
            if centre is None:
                return False
            nx, ny = end[0] - centre[0], end[1] - centre[1]
            along = nx * d[0] + ny * d[1]
            nx, ny = nx - along * d[0], ny - along * d[1]
            h = math.hypot(nx, ny)
            sides = [(nx / h, ny / h), (-nx / h, -ny / h)] if h > 1e-9 \
                else [(-d[1], d[0]), (d[1], -d[0])]
            for k in range(1, 4):
                off = 0.35 * k
                e2 = (end[0] + off * d[0], end[1] + off * d[1])
                if not pad_clear(end, e2):
                    continue
                for (ox, oy) in sides:
                    f2 = (e2[0] + 2 * D * ox, e2[1] + 2 * D * oy)
                    if pad_clear(e2, f2):
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
                               (centres or {}).get(nm)) \
                    or wrap_clear(stubs[nm], dn, (centres or {}).get(nm))
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
            d2 = (X - fx) * (X - fx) + (Y - fy) * (Y - fy)
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
            # opposite to the turn; its offset is the larger of its
            # offsets from the two legs' lines -- the mitred offset
            # polyline that passes through it
            ka = np.clip(vk - 1, 0, self.n - 1)
            kb = np.clip(vk, 0, self.n - 1)
            outer = -np.sign(self.turn[np.clip(vk - 1, 0, self.n - 2)])
            outer = np.where(outer == 0, 1.0, outer)
            o1 = (X - self.P[ka, 0]) * self.nrm[ka, 0] + (Y - self.P[ka, 1]) * self.nrm[ka, 1]
            o2 = (X - self.P[kb, 0]) * self.nrm[kb, 0] + (Y - self.P[kb, 1]) * self.nrm[kb, 1]
            r = np.maximum(np.abs(o1), np.abs(o2))
            best_o = np.where(wedge, outer * r, best_o)
            best_s = np.where(wedge, self.S[vk], best_s)
        return best_s.reshape(shape), best_o.reshape(shape)

    def project_pt(self, p: Pt) -> Tuple[float, float]:
        s, o = self.project(np.array([p[0]]), np.array([p[1]]))
        return float(s[0]), float(o[0])

    def lane_line(self, so: Sequence[Tuple[float, float]], fixed=()) -> List[Tuple[float, float, float]]:
        """Board polyline of a lane given by its COLUMN points so [(s, o)], s increasing: [(x, y, s)], each column on
        its own leg, and at each corner of the spine the lane's two legs MEET: a lane on the corner's outer side at the
        mitre of its offset there (every column drawn), a lane inside it where its own two lines cross -- the line
        through its last two points before the corner and the line through its first two after it. A point past that
        crossing is left out: a lane inside a corner lies, near it, beyond where its line meets the other leg's (by
        |o| tan(turn / 2) at a fixed offset, sooner where its offset grows into the corner), and drawn there it
        stepped past the corner and back. The constant-offset mitre lane_xy puts in is that crossing only for a lane
        whose offset does not change. Points whose index is in `fixed` (terminals, layer changes) are never left out;
        where one would have to be, or the two lines cross farther than a turn of 120 degrees would put them, the
        lane runs straight across the corner."""
        pts = [(float(s), float(o), i) for i, (s, o) in enumerate(so)]
        at = {}                                         # pts index -> the corner put in after it
        for j in range(1, self.n):
            if abs(float(self.turn[j - 1])) < 1e-6:
                continue
            Sj = float(self.S[j])
            # (a column ON the corner is drawn on the leg after it, as Spine.xy draws it: one of the points after)
            b = next((k for k, p in enumerate(pts) if p[0] >= Sj - 1e-9), None)
            if b is None or b == 0:
                continue
            a = b - 1
            X = None
            # a lane on the corner's OUTER side (its offset at the corner of the sign away from the turn): its legs meet
            # at the mitre of that offset, between its last column before the corner and its first after it, and every
            # column is drawn. Outside a corner the offset lines part, so no column lies past the meeting -- but the
            # meeting of a lane's OWN two lines does, where its offset changes across the corner: the zynq DDR's RAS,
            # 3.8 mm outside the north ring's 36-degree corner and moving in 0.07 mm a column, met its other leg 2.3 mm
            # along it, its three columns there were left out, and the line drawn to that point crossed ODT's, which
            # stood a pitch inside it at every column. The mitres of two lanes' offsets keep the order of their columns
            oc = pts[a][1] + (pts[b][1] - pts[a][1]) * (Sj - pts[a][0]) / max(pts[b][0] - pts[a][0], 1e-12)
            if oc * float(self.turn[j - 1]) < 0:
                n1, n2 = self.nrm[j - 1], self.nrm[j]
                m = (n1 + n2) / max(1.0 + float(n1 @ n2), 1e-6)
                X = (float(self.P[j, 0] + oc * m[0]), float(self.P[j, 1] + oc * m[1]))
                P1, Q0 = self.xy(*pts[a][:2]), self.xy(*pts[b][:2])
                gap = math.hypot(Q0[0] - P1[0], Q0[1] - P1[1])
                if math.hypot(X[0] - P1[0], X[1] - P1[1]) + math.hypot(Q0[0] - X[0], Q0[1] - X[1]) <= 2.0 * gap + 1e-9:
                    at[pts[a][2]] = (X[0], X[1], Sj)
                continue                                # (past a turn of 120 degrees: straight across)
            while a >= 1 and b + 1 < len(pts) and self.seg_of(pts[a - 1][0]) == j - 1 and self.seg_of(pts[b + 1][0]) == j:
                P0, P1 = self.xy(*pts[a - 1][:2]), self.xy(*pts[a][:2])
                Q0, Q1 = self.xy(*pts[b][:2]), self.xy(*pts[b + 1][:2])
                r = (P1[0] - P0[0], P1[1] - P0[1]); q = (Q1[0] - Q0[0], Q1[1] - Q0[1])
                den = r[0] * q[1] - r[1] * q[0]
                if abs(den) < 1e-12:
                    break                               # the two lines parallel: straight across
                w = (Q0[0] - P0[0], Q0[1] - P0[1])
                t = (w[0] * q[1] - w[1] * q[0]) / den   # P0 + t r = Q0 + u q
                u = (w[0] * r[1] - w[1] * r[0]) / den
                if t < 1.0 - 1e-9 and pts[a][2] not in fixed:
                    del pts[a]; a -= 1; b -= 1          # the last point before the corner lies past the crossing
                    continue
                if u > 1e-9 and pts[b][2] not in fixed:
                    del pts[b]                          # the first one after it lies before the crossing
                    continue
                if t >= 1.0 - 1e-9 and u <= 1e-9:
                    X = (P0[0] + t * r[0], P0[1] + t * r[1])
                    gap = math.hypot(Q0[0] - P1[0], Q0[1] - P1[1])
                    if math.hypot(X[0] - P1[0], X[1] - P1[1]) + math.hypot(Q0[0] - X[0], Q0[1] - X[1]) > 2.0 * gap + 1e-9:
                        X = None
                break
            if X is not None:
                at[pts[a][2]] = (X[0], X[1], Sj)
        out = []
        for s, o, i in pts:
            x, y = self.xy(s, o)
            out.append((x, y, s))
            if i in at:
                out.append(at[i])
        return out

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
                if abs(o) > 1e-9:
                    # the mitre of the two offset lines, either side --
                    # unless it lies BEHIND the piece's start along the
                    # incoming leg or past its end along the outgoing one:
                    # an inner offset near a vertex put it there and the
                    # line stepped back (HHa's north wrap, SDQ10: 0.05 mm
                    # back at the corner, two 170-degree turns)
                    den = 1.0 + float(n1 @ n2)
                    m = (n1 + n2) / max(den, 1e-6)
                    mp = (float(V[0] + o * m[0]), float(V[1] + o * m[1]))
                    a_ = out[-1] if out else self.xy(sa, oa)
                    b_ = self.xy(sb, ob)
                    d1_, d2_ = self.d[j - 1], self.d[j]
                    if ((mp[0] - a_[0]) * d1_[0] + (mp[1] - a_[1]) * d1_[1] > 0
                            and (b_[0] - mp[0]) * d2_[0] + (b_[1] - mp[1]) * d2_[1] > 0):
                        push(mp)
                else:
                    push((float(V[0]), float(V[1])))
            push(self.xy(sb, ob))
        if len(out) == 1:
            out.append(out[0])
        return out


# ---------------------------------------------------------------- a part seen from several spines
# The whole route's geometry (whole_geo) lays each lane in frames -- the trunk, a ring round the destination -- each a
# spine whose offset o runs across it. A part a lane must pass is one thing on the board, and which side of it the lane
# passes is decided once there (decide_sides) and carried into every frame (side_carry); each frame only measures it,
# column by column, along the column's own offset line (line_extent) -- the box of its four corners projected into a
# BENT frame is far larger than its copper (zynq K44: C98 at the corner where the trunk hands its lanes to the ring).

def line_extent(sp: Spine, s: float, rects, g: float) -> Optional[Tuple[float, float]]:
    """(lo, hi): the offsets where spine `sp`'s offset line at `s` -- sp.xy(s, o) for every o -- meets the rectangles
    `rects` [(x0, y0, x1, y1)] grown by `g` on every side; None where it misses them all. The line is infinite: a
    caller bounds it (whole_geo gates it by the part's span and its box, and the lane's reach)."""
    ax, ay = sp.xy(s, 0.0)
    nx, ny = (float(v) for v in sp.nrm[sp.seg_of(s)])
    lo, hi = math.inf, -math.inf
    for (x0, y0, x1, y1) in rects:
        a_, b_ = -math.inf, math.inf
        for p0, p1, q, d in ((x0 - g, x1 + g, ax, nx), (y0 - g, y1 + g, ay, ny)):
            if abs(d) < 1e-12:
                if not (p0 <= q <= p1):
                    a_, b_ = 1.0, -1.0
                continue
            t0, t1 = sorted(((p0 - q) / d, (p1 - q) / d))
            a_, b_ = max(a_, t0), min(b_, t1)
        if a_ <= b_:
            lo, hi = min(lo, a_), max(hi, b_)
    return (lo, hi) if lo <= hi else None


def round_cover(x: float, y: float, rx: float, ry: float):
    """a ROUND pad or hole (centre x, y, half-sizes rx, ry) as rectangles for line_extent: the cross of two, each its
    full size one way and cos 45 of it the other. It covers the circle and reaches 0.22 of its radius past it at its
    corners, where the square of its box reached 0.41 -- a hole beside a ring's handoff held the ring's lanes in past
    the ring's start by that much (synth_handoff npth_corner)"""
    c = math.sqrt(0.5)
    return [(x - rx, y - c * ry, x + rx, y + c * ry), (x - c * rx, y - ry, x + c * rx, y + ry)]


def pads_across(spines, pads) -> bool:
    """whether a two-pad part's pads (centres) stand ACROSS the lanes -- one beside the other in its home frame's offset,
    so a lane through the gap between them runs along the frame -- rather than one after the other along it, where a
    lane "between" them would cross the part's axis through the gap (K51 C12 along DU1's south face: lanes told to
    pass between its pads, 0.21 mm short, and the loop did not converge). `spines` {frame: Spine}"""
    so = {f: [sp.project_pt(p) for p in pads] for f, sp in spines.items()}
    c = ((pads[0][0] + pads[1][0]) / 2, (pads[0][1] + pads[1][1]) / 2)
    h = part_home(spines, {f: (min(q[0] for q in v), max(q[0] for q in v)) for f, v in so.items()}, c)
    (s1, o1), (s2, o2) = so[h]
    return abs(o1 - o2) > abs(s1 - s2)


def part_home(spines, spans, centre: Pt):
    """the frame that sees a part WHOLE -- its span (sa, sb), the s of its four corners there, not cut at the frame's
    ends -- nearest its spine; `spines` {frame: Spine}, `spans` {frame: (sa, sb)}"""
    return min((not (spans[f][0] > 1e-6 and spans[f][1] < spines[f].L - 1e-6), abs(spines[f].project_pt(centre)[1]), f)
               for f in sorted(spines))[2]


def side_carry(sp_from: Spine, sp_to: Spine, centre: Pt, o_hi: float, room: float) -> int:
    """+1 or -1: a part's +1 side in spine `sp_from` (its offsets above `o_hi`, the top of its box there) as a side in
    `sp_to` -- a point `room` beyond it at the centre's s, against the centre, both measured in `sp_to`"""
    s_c, _o = sp_from.project_pt(centre)
    up = sp_from.xy(s_c, o_hi + room)
    return 1 if sp_to.project_pt(up)[1] >= sp_to.project_pt(centre)[1] else -1


def decide_sides(meets, home, split_side, box_mid, carry):
    """{(lane, part): side in the part's HOME frame's terms} for every lane meeting a part in two frames or more.
    `meets` {(lane, part): {frame: [the lane's offsets at its columns meeting the part]}}; `home` {part: frame};
    `split_side` {(frame, lane, part): side}, the frames' own splits; `box_mid` {(frame, part): the middle of the
    part's offsets there}; `carry` {(part, frame): +1 / -1, side_carry from the home frame}. The home frame's split
    where the lane is in it -- a split reads the lane order at the part's middle column, where a lane handing off
    before it is absent -- else the frame the lane meets the part in over most columns (the home first among equals):
    that frame's split, else its mean offset against the part's middle there"""
    out = {}
    for (n, ii), byf in meets.items():
        if len(byf) < 2:
            continue
        h = home[ii]
        if (h, n, ii) in split_side:
            src, sd = h, split_side[(h, n, ii)]
        else:
            src = max(sorted(byf), key=lambda f: (len(byf[f]), f == h))
            sd = split_side.get((src, n, ii))
            if sd is None:
                sd = -1 if float(np.mean(byf[src])) < box_mid[(src, ii)] else 1
        out[(n, ii)] = sd * carry[(ii, src)]
    return out


def piece_u(u: float, trunk: bool, handoff: float) -> float:
    """the place along a lane at which its layers are read on one of its pieces (whole_geo.lay_u), a point of it there
    at `u`: its changes up to its ring's `handoff` are the trunk's and the later ones the ring's, as their vias are
    drawn -- so on the trunk (`trunk`) no later than the handoff, and on the ring no earlier. A ring piece starts where
    its trunk ENDS, which may be short of the ring's origin, its first columns' u short of the handoff"""
    h = handoff + 1e-9
    return min(u, h) if trunk else max(u, h)


def held_interval(iv, ref: float, was, term):
    """(lo, hi): the free interval of a column a lane is bounded to (whole_geo's bound rows). `iv` the column's free
    offset intervals outside the arrays' boxes, `ref` the lane's reference there, `was` the interval it was held to a
    column before (None at its first), `term` the offset of the terminal AHEAD of it -- the end it must reach (None: no
    end fixed).
    The interval the reference lies in; where it lies in none -- it cuts a box's corner -- the one the lane was in a
    column before, which it cannot leave across the box (K41: at the column clipping the source's south-east corner,
    the interval nearest the south-face lanes' references was the one NORTH of the box, and six lanes were bounded 2.7
    to 3.9 mm off where they ran). Where that one stood on both sides of the box here -- the box's corner starts at
    this column -- the side of its nearer terminal, which it cannot reach across the box: by the most overlap it was
    the larger side, and zynq U2's top-face berths, which stand on the destination's grown box, had A8 and CKE held
    SOUTH of the corner they reached them round, their last columns run across the west column's balls (A8 into A6's
    via in the gap). The nearest one only with neither.
    A lane held a column before keeps to the intervals that overlap that one: a reference that runs INTO the box and
    out of its far side does not carry the lane across it (zynq LVDS K47: CTRL_IN0's reference, tooth to handoff,
    crossed U1's east corner, and at the column it came out north of the box the bound jumped 3.3 mm across it --
    the lane paid 5 to 7 mm of bound, and the polish that pushed it out squeezed its neighbour off its layer). And at
    such a split the side of the terminal ahead whether or not the reference lies in the other: a reference a hair
    inside one side may run on into the box (K47's RX_D0, 0.15 mm north of the destination's corner where it started,
    its berth 1.1 mm south of it: held north, it paid 1.3 mm of bound)"""
    over = [q for q in iv if was is not None and min(q[1], was[1]) > max(q[0], was[0])]
    if len(over) > 1 and term is not None:
        return min(over, key=lambda q: max(q[0] - term, term - q[1], 0.0))
    inside = [q for q in (over or iv) if q[0] <= ref <= q[1]]
    if inside:
        return inside[0]
    if over:
        return max(over, key=lambda q: min(q[1], was[1]) - max(q[0], was[0]))
    return min(iv, key=lambda q: min(abs(ref - q[0]), abs(ref - q[1])))


def _unit(v: Pt) -> Pt:
    h = math.hypot(v[0], v[1])
    return (v[0] / h, v[1] / h) if h > 1e-12 else (1.0, 0.0)


def _angle(u: Pt, v: Pt) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, u[0] * v[0] + u[1] * v[1]))))


def octo_hull(pts: Sequence[Pt], margin: float) -> List[Pt]:
    """The octilinear hull of `pts` (the intersection of the half-planes of
    its eight support lines, at 0, 45, .. 315 degrees) pushed out by
    `margin`: eight vertices in increasing-angle order."""
    dirs = [(math.cos(k * math.pi / 4), math.sin(k * math.pi / 4)) for k in range(8)]
    h = [max(p[0] * d[0] + p[1] * d[1] for p in pts) + margin for d in dirs]
    V = []
    for k in range(8):
        (a1, b1), c1 = dirs[k], h[k]
        (a2, b2), c2 = dirs[(k + 1) % 8], h[(k + 1) % 8]
        det = a1 * b2 - a2 * b1
        V.append(((c1 * b2 - c2 * b1) / det, (a1 * c2 - a2 * c1) / det))
    return V


def build_wrap_spine(dest_pts: Sequence[Pt], stubs: Sequence[Pt],
                     teeth: Sequence[Pt], ccw: bool, arrive: Pt,
                     margin: float, reach_deg: float = 8.0,
                     per_edge: int = 40, hull_extra: Sequence[Pt] = (), wrap_side: bool = False) -> Spine:
    """A WRAP corridor's spine: the bundle comes in from its teeth, meets
    the destination array at the corner where the face the incoming
    bundle meets ends (the straight corridor's face), and runs round the
    array -- along the octilinear hull of its pads and this group's stubs,
    `margin` outside -- in the group's wrap direction (`ccw`: the angle
    round the array's centre increasing) to just past the farthest stub,
    so every stub lies on the spine's inner side and the lanes peel off
    in arc order, innermost first, as a human's ring does. `arrive` is
    the incoming bundle's direction at the array. `hull_extra`: more copper
    the ring must pass outside (the trunk's head-on lanes on their way to
    their berths), in the hull only."""
    V = octo_hull(list(dest_pts) + list(stubs) + list(hull_extra), margin)
    cen = (sum(p[0] for p in dest_pts) / len(dest_pts),
           sum(p[1] for p in dest_pts) / len(dest_pts))
    Ct = (sum(p[0] for p in teeth) / len(teeth), sum(p[1] for p in teeth) / len(teeth))
    ring = []
    for k in range(8):
        a, b = V[k], V[(k + 1) % 8]
        for t in np.linspace(0, 1, per_edge, endpoint=False):
            ring.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
    ang = lambda p: math.atan2(p[1] - cen[1], p[0] - cen[0])

    def outward_dot(k):
        a, b = V[k], V[(k + 1) % 8]
        n_ = (b[1] - a[1], -(b[0] - a[0]))
        L_ = math.hypot(*n_) or 1.0
        return (n_[0] * arrive[0] + n_[1] * arrive[1]) / L_
    k_front = min(range(8), key=outward_dot)       # the face the bundle meets
    step = 1 if ccw else -1
    # angles are measured from the corner where that face ends in the wrap
    # direction: every stub of the group lies ahead of it
    i_f = (((k_front + 1) % 8) * per_edge) if ccw else (k_front * per_edge)
    a0 = ang(ring[i_f])

    def swept(p):
        d = ang(p) - a0
        return d % (2 * math.pi) if ccw else (-d) % (2 * math.pi)
    # (a WOUND ring's stubs, `wrap_side`, lie anywhere ahead short of the face the bundle meets, whose far corner is
    # its bound: a bundle arriving at a slant meets the hull's diagonal face, and its wound stubs on the side face
    # beyond the far one lie 1.6 pi round -- past one and a half pi the start was backed up to just short of them
    # and the ring ran straight from the trunk THROUGH the destination, zynq's Ethernet bus's MDC and MDIO to U3)
    j_b = (k_front * per_edge) if ccw else (((k_front + 1) % 8) * per_edge)
    for _ in range(len(ring)):
        lim = swept(ring[j_b]) if wrap_side else 1.5 * math.pi
        if all(swept(s) < lim for s in stubs):
            break
        i_f = (i_f - step) % len(ring)
        a0 = ang(ring[i_f])
    a_end = max(swept(s) for s in stubs) + math.radians(reach_deg)
    # the lead-in is pulled TAUT: straight from the teeth to where it grazes
    # the hull on the wrap side (the tangent point), never along the face the
    # bundle meets; stubs short of the tangent point are peeled off the lead-in
    ahead = [i % len(ring) for i in range(i_f, i_f + step * len(ring), step)
             if swept(ring[i % len(ring)]) <= a_end]
    cx_, cy_ = cen[0] - Ct[0], cen[1] - Ct[1]

    def bearing(q):
        # signed angle at the teeth between the array's centre and q
        qx, qy = q[0] - Ct[0], q[1] - Ct[1]
        return math.atan2(cx_ * qy - cy_ * qx, cx_ * qx + cy_ * qy)
    # (`wrap_side`: the tangent on the side the ring wraps -- a ring wrapping with the angle rising grazes the hull at
    # the least bearing -- not the stubs' side: a WOUND ring's stubs lie past the array's far side, and their bearings
    # pointed the lead-in at the facing face, a hook back down it, whole_frame's winding cut)
    side = (-1.0 if ccw else 1.0) if wrap_side else sum(bearing(s) for s in stubs)
    i_t = (max(ahead, key=lambda i: bearing(ring[i])) if side > 0
           else min(ahead, key=lambda i: bearing(ring[i])))
    path = [Ct]
    for i in ahead[ahead.index(i_t):]:
        path.append(ring[i])
    return Spine(simplify(path, 0.02))


# ---------------------------------------------------------------- distances


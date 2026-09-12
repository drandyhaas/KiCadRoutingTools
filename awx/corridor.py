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

At a corner of the spine the offset lines of a lane at o meet at the
MITRE point on both sides (the corner displaced along the bisector by
o / cos(turn/2)); a cell in the outer wedge projects to the corner's s
at the larger of its two offsets from the legs' lines, which is the
offset polyline that passes through it. Adjacent lanes stay a pitch
apart along the legs and 1/cos(turn/2) of it across the mitre. The
outer side used to be an ARC of radius |o| (what a bundle of curved
tracks does); on an octilinear router the band of an arc is a
staircase, and every lane took one round every corner (channel article
K28: 4021 segments for 28 lanes against 1166 on the chord). The spine's
legs are octilinear (octilinearise), so mitred lanes are grid legs too.
"""
from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

import taut_fast as tf
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
                      src_centres=None, same_line=None) -> List[List[str]]:
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
            elif same_line is not None and same_line(a, b):
                # two stubs on one BAND LINE (a banded destination): a
                # comb along the band whatever the bearings from the
                # array's centre say (inside the band the centre is
                # beside the stubs, and two of them 4 mm apart read as
                # arriving 110 degrees apart: two corridors, the second
                # laid through the first's lanes)
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
                                    centres=centres, src_centres=src_centres,
                                    same_line=same_line)
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
                    # the mitre of the two offset lines, either side
                    den = 1.0 + float(n1 @ n2)
                    m = (n1 + n2) / max(den, 1e-6)
                    push((float(V[0] + o * m[0]), float(V[1] + o * m[1])))
                else:
                    push((float(V[0]), float(V[1])))
            push(self.xy(sb, ob))
        if len(out) == 1:
            out.append(out[0])
        return out


# ---------------------------------------------------------------- obstacles

class RampedObstacles:
    """The obstacle set a spine is relaxed against: the base obstacles
    (the big parts' pads) and the TUBES of the corridors already laid --
    each one's spine at a lane pitch's radius -- every obstacle inflated
    by its own amount `infl`: the bundle's half-width H for a part, H
    plus the OTHER corridor's half-width for a tube, so two corridors
    never overlap along their length (a later corridor runs beside an
    earlier one, or crosses it transversally). The inflation is RAMPED
    by the distance to the nearer end: nothing within `infl` of an end,
    the full amount from 2*infl, because a bundle is not yet a bundle
    where its lanes are still joining or already leaving, and a
    corridor's exits may legitimately sit among another corridor's
    stubs. The ramp scales with each obstacle's OWN inflation, so a fat
    neighbour tube never reaches a thin corridor's end zone.
    `extra`: [(a, b, rad[, infl])] capsules; infl defaults to H."""

    def __init__(self, base: 'ts.Obstacles', ends: Tuple[Pt, Pt], H,
                 extra: Optional[List[tuple]] = None, zones=None):
        self.base = base
        self.ends = ends
        # `zones`: ((C, u, R) at the launch, (C, u, R) at the arrival) --
        # the end centroid, the unit flow INTO the corridor, the end
        # zone's length along it. With them the PARTS ramp ALONG THE
        # FLOW: nothing inside an end zone (a part beside the teeth is
        # the lanes' business, and the string's frozen end is never
        # inside an inflated obstacle), then one millimetre of inflation
        # per millimetre of run past it, up to H -- the fastest a ribbon
        # of 45-degree legs can shift sideways, so a part the string can
        # reach is inflated exactly as much as the lanes can honour. The
        # radial ramp (distance to the centroid, nothing within H of it)
        # zeroed the inflation for a part 7 mm ahead of the teeth when H
        # was 5.9, and the top lanes ran into it. Without zones the
        # parts ramp radially like the tubes.
        self.zones = zones
        # `H`: the bundle's half-width the parts are inflated by (ramped).
        # A RIBBON model -- the parts inflated by the ribbon's half-extent
        # interpolated from the teeth's spread to the stubs', UNRAMPED --
        # was measured on 2026-09-08 and LOST: full-width inflation from
        # the launch makes the string wander into a sawtooth of eleven
        # grid legs (both channel articles at K28: 3 open, 3 to 5 of 28
        # lanes in band, against 0 open with the ramp). The ramp stays.
        self.H = H
        self.extra_raw = []
        for tup in (extra or []):
            a, b, rad = tup[0], tup[1], tup[2]
            infl = tup[3] if len(tup) > 3 else H
            self.extra_raw.append((a, b, rad, infl))

    def d_end(self, p: Pt) -> float:
        return min(math.hypot(p[0] - e[0], p[1] - e[1]) for e in self.ends)

    def along(self, p: Pt) -> float:
        """How far past the nearer end zone the point is, along that
        end's flow (negative inside or behind a zone)."""
        return min((p[0] - c[0]) * u[0] + (p[1] - c[1]) * u[1] - r
                   for (c, u, r) in self.zones)

    def part_inflation(self, p: Pt) -> float:
        if self.zones is None:
            return self.inflation(self.d_end(p), self.H)
        return max(0.0, min(self.H, self.along(p)))


    @staticmethod
    def inflation(d: float, infl: float) -> float:
        """How much an obstacle inflated by `infl` is inflated at a point
        `d` from the nearer end: 0 within infl, infl from 2*infl."""
        return max(0.0, min(infl, d - infl))

    def point_violation(self, p: Pt, pad: float = 0.0):
        d = self.d_end(p)
        worst = self.base.point_violation(p, pad=pad + self.part_inflation(p))
        for (a, b, rad, infl) in self.extra_raw:
            sx, sy = b[0] - a[0], b[1] - a[1]
            L2 = sx * sx + sy * sy
            u = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((p[0] - a[0]) * sx + (p[1] - a[1]) * sy) / L2))
            ex, ey = p[0] - (a[0] + u * sx), p[1] - (a[1] + u * sy)
            dist = math.hypot(ex, ey)
            depth = rad + self.inflation(d, infl) + pad - dist
            if depth > 0 and (worst is None or depth > worst[0]):
                if dist > 1e-9:
                    nrm = (ex / dist, ey / dist)
                else:
                    L = math.sqrt(L2) if L2 > 1e-12 else 1.0
                    nrm = (-sy / L, sx / L)
                worst = (depth, nrm)
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




def octilinearise(pts: Sequence[Pt], obs, flows=None, tol: float = 1.0) -> List[Pt]:
    """A relaxed spine as a few OCTILINEAR legs. The relaxation hands
    back an arc round the obstacle -- ten vertices turning 10 or 15
    degrees each -- and a lane in the band of such a leg is a shallow
    line on the router's grid: a staircase of sub-pitch octilinear
    steps (measured on the channel article at K15: 2683 segments for
    15 lanes against 590 on the straight chord, 2501 of them under
    0.3 mm). A bundle turns a part the way a human's does, in legs at
    0, 45 or 90 degrees. So: the polyline simplified at `tol`, then
    every interior leg off the grid directions replaced by the two
    grid legs that span it (the leg's vector decomposed on the two
    octilinear directions that bracket it), in whichever order keeps
    the intermediate vertex clear of the ramped obstacles -- the
    order that folds INTO the part is the other one.  is COARSE
    on purpose: at 0.25 mm an arc of radius 4 mm kept eight legs and
    each became a 45-degree pair, a sawtooth the lanes could not
    follow (channel article K15: 5 of 15 in band); at 1 mm the arc is
    one or two chords, and the clear order of their grid legs lies
    OUTSIDE the arc (the chord cuts inside it, the grid legs round it),
    so the frame ends up no closer to the part than the relaxed line. A leg neither
    order can clear keeps its own direction. A first or last leg that
    still runs along its end zone's FLOW (`flows` = launch, arrival
    unit vectors; within 3 degrees) is left as it is -- the flow may
    sit off the grid (the bench's chord is 2 degrees off) and the end
    zones are the teeth's and berths' business; a straight corridor
    never comes here."""
    pts = simplify(pts, tol)
    if len(pts) < 3:
        return list(pts)
    out = [pts[0]]

    def bad(a, b):
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(L / 0.25))
        return sum(1 for k in range(n + 1)
                   if obs.point_violation((a[0] + (b[0] - a[0]) * k / n,
                                           a[1] + (b[1] - a[1]) * k / n)) is not None)
    for i, (P, Q) in enumerate(zip(pts, pts[1:])):
        vx, vy = Q[0] - P[0], Q[1] - P[1]
        ang = math.degrees(math.atan2(vy, vx))
        k = round(ang / 45.0)
        on_flow = False
        if flows is not None:
            f = flows[0] if i == 0 else (flows[1] if i == len(pts) - 2 else None)
            if f is not None:
                on_flow = _angle(_unit((vx, vy)), f) <= 3.0
        if on_flow or abs(ang - 45.0 * k) < 1.0:
            out.append(Q)
            continue
        lo = 45.0 * math.floor(ang / 45.0)
        d1 = (math.cos(math.radians(lo)), math.sin(math.radians(lo)))
        d2 = (math.cos(math.radians(lo + 45.0)), math.sin(math.radians(lo + 45.0)))
        det = d1[0] * d2[1] - d1[1] * d2[0]
        al = (vx * d2[1] - vy * d2[0]) / det
        be = (d1[0] * vy - d1[1] * vx) / det
        cands = []
        for order in ((d1, al), (d2, be)):
            d, m = order
            M = (P[0] + m * d[0], P[1] + m * d[1])
            cands.append((bad(P, M) + bad(M, Q), M))
        own = bad(P, Q)
        best = min(cands, key=lambda c: c[0])
        if best[0] <= own:
            out.append(best[1])
        out.append(Q)
    return simplify(out, 0.08)


def merge_short_legs(pts: Sequence[Pt], min_len: float) -> List[Pt]:
    """An interior leg shorter than `min_len` is a quantisation notch
    (the channel article's K15 spine: flat, a 0.55 mm step down, then
    the climb), not a bend the lanes should follow: it is removed and
    its neighbours extended to where their lines meet. Neighbours that
    are parallel keep the notch (there is no meeting point). The first
    and last legs are the flows and are never merged."""
    P = [tuple(p) for p in pts]
    changed = True
    while changed and len(P) > 3:
        changed = False
        for i in range(1, len(P) - 2):
            a, b = P[i], P[i + 1]
            if math.hypot(b[0] - a[0], b[1] - a[1]) >= min_len:
                continue
            d1 = _unit((a[0] - P[i - 1][0], a[1] - P[i - 1][1]))
            d2 = _unit((P[i + 2][0] - b[0], P[i + 2][1] - b[1]))
            det = d1[0] * d2[1] - d1[1] * d2[0]
            if abs(det) < 1e-9:
                continue
            s = ((b[0] - P[i - 1][0]) * d2[1] - (b[1] - P[i - 1][1]) * d2[0]) / det
            m = (P[i - 1][0] + s * d1[0], P[i - 1][1] + s * d1[1])
            # the meeting point must lie ahead on the previous leg and
            # behind on the next, or the merge would fold the polyline
            if s <= 0 or ((P[i + 2][0] - m[0]) * d2[0] + (P[i + 2][1] - m[1]) * d2[1]) <= 0:
                continue
            P[i:i + 2] = [m]
            changed = True
            break
    return P


def clear_legs(pts: Sequence[Pt], obs, rounds: int = 8, step: float = 0.1) -> List[Pt]:
    """The octilinear polyline pushed OUT of the ramped obstacles, leg by
    leg, its directions kept. A grid leg is a chord of the relaxed arc
    and a chord cuts inside it: on the channel article the 45-degree leg
    into the bottom of the dip lay 1.8 mm nearer the header than the
    string had settled, and the island logic then saw the part 3.55 mm
    from the spine where the relaxation had cleared 5.9. Each round
    samples every interior leg against the model; a violating leg is
    moved along its own normal, in the direction the deepest violation
    pushes, by that depth, and its ends re-cut where its line meets the
    neighbouring legs' lines (their directions kept, so the polyline
    stays octilinear). The first and last legs are anchored at the
    ends: the string never bends inside the end zones (no inflation
    there), so a violation on them is left to the lanes. Converges in a
    few rounds (every move is outward); `rounds` bounds it."""
    P = [tuple(p) for p in pts]
    if len(P) < 3:
        return P

    def worst_on(a, b):
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(L / step))
        w = None
        for k in range(n + 1):
            q = (a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
            v = obs.point_violation(q)
            if v is not None and (w is None or v[0] > w[0]):
                w = v
        return w

    def line_isect(p, d, q, e):
        det = d[0] * e[1] - d[1] * e[0]
        if abs(det) < 1e-9:
            return None
        s = ((q[0] - p[0]) * e[1] - (q[1] - p[1]) * e[0]) / det
        return (p[0] + s * d[0], p[1] + s * d[1])
    for _r in range(rounds):
        moved = False
        for i in range(1, len(P) - 2):
            a, b = P[i], P[i + 1]
            w = worst_on(a, b)
            if w is None:
                continue
            depth, (ux, uy) = w
            d = _unit((b[0] - a[0], b[1] - a[1]))
            nrm = (-d[1], d[0])
            sgn = 1.0 if (nrm[0] * ux + nrm[1] * uy) >= 0 else -1.0
            shift = (depth + 0.02) * sgn
            a2 = (a[0] + shift * nrm[0], a[1] + shift * nrm[1])
            dp = _unit((a[0] - P[i - 1][0], a[1] - P[i - 1][1]))
            dn = _unit((P[i + 2][0] - b[0], P[i + 2][1] - b[1]))
            na = line_isect(a2, d, P[i - 1], dp)
            nb = line_isect(a2, d, P[i + 2], dn)
            if na is None or nb is None:
                continue
            P[i], P[i + 1] = na, nb
            moved = True
        if not moved:
            break
    out = [P[0]]
    for q in P[1:]:
        if math.hypot(q[0] - out[-1][0], q[1] - out[-1][1]) > 1e-6:
            out.append(q)
    return out


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
        # BOTH ends measure FORWARD along the path: at_start wants
        # (further in) - (start), at the arrival end (end) - (further
        # back). The two were written as one expression with the operands
        # swapped for the arrival case, so any taut path shorter than
        # 2*probe reported its arrival flow REVERSED -- verified: a 5 mm
        # east path gives (1,0) and a 1 mm east path gave (-1,0). A
        # corridor of short lanes then built its spine against a u_s
        # pointing back into the destination array.
        p, q = (path[0], path[-1]) if at_start else (path[0], path[-1])
        taut = _unit((q[0] - p[0], q[1] - p[1]))
    elif at_start:
        q = point_after_start(path, probe)
        taut = _unit((q[0] - path[0][0], q[1] - path[0][1]))
    else:
        q = point_before_end(path, probe)
        taut = _unit((path[-1][0] - q[0], path[-1][1] - q[1]))
    return end_dir if _angle(taut, end_dir) <= snap_deg else taut


def align_tail(sp_pts: Sequence[Pt], dest_box, margin: float = 0.5) -> List[Pt]:
    """The spine's TAIL aligned with the destination array's own axis
    (SPLIT_BLOCKS, 2026-09-10). A corridor's spine is the chord (or
    medial line) from the teeth's centroid to the STUBS' centroid, and
    with berths on three faces of the destination that centroid pulls
    the chord across the array at an angle (the bench's K28: 13 degrees
    through DU1). Every lane past the schedule region runs at a
    constant offset from the spine, so inside the array -- a band comb,
    a flank block -- it drifts across the band or the ball rows by the
    tilt times its length. From where the spine first reaches the
    array's near face (the padded box), it runs on along the axis the
    chord points nearest to, for the chord's remaining length along
    that axis: the lanes at the destination then run parallel to the
    faces and the bands, as the human's do. Unchanged when the spine
    never reaches the box."""
    if dest_box is None or len(sp_pts) < 2:
        return list(sp_pts)
    x0, y0, x1, y1 = dest_box
    bx = (x0 - margin, y0 - margin, x1 + margin, y1 + margin)
    a, b = sp_pts[0], sp_pts[-1]
    u = _unit((b[0] - a[0], b[1] - a[1]))
    u_snap = (1.0 if u[0] > 0 else -1.0, 0.0) if abs(u[0]) >= abs(u[1]) \
        else (0.0, 1.0 if u[1] > 0 else -1.0)
    # the face the spine arrives at, and the coordinate of its line
    if u_snap[0]:
        face = bx[0] if u_snap[0] > 0 else bx[2]
    else:
        face = bx[1] if u_snap[1] > 0 else bx[3]
    ax = 0 if u_snap[0] else 1
    # the tail is the axis line THROUGH THE STUBS' CENTROID (the spine's
    # end), from the face to that end -- not the chord's own face
    # crossing: a steep chord (two band stubs reached from the source's
    # far corner) meets the face inside the next block and its tail ran
    # along a ball row. The spine keeps its points before the face and
    # bends to the tail's start on the face line.
    # ...on the array's CENTRE line (a banded array's band, a solid
    # array's middle), not the stubs' centroid's: a corridor with berths
    # on three faces has its centroid line on a ball row, the corridor's
    # tube then holds that row's balls as static islands, and every lane
    # is deflected round them. The tail runs from the face to the
    # chord's end projected onto that line.
    cy = ((y0 + y1) / 2) if ax == 0 else ((x0 + x1) / 2)
    pf = (face, cy) if ax == 0 else (cy, face)
    end = (b[0], cy) if ax == 0 else (cy, b[1])
    rest = (end[0] - pf[0]) * u_snap[0] + (end[1] - pf[1]) * u_snap[1]
    if rest <= 0.1:
        return list(sp_pts)
    out = [sp_pts[0]]
    for q in sp_pts[1:]:
        if (q[ax] - face) * u_snap[ax] >= 0:
            break
        out.append(q)
    if math.hypot(pf[0] - out[-1][0], pf[1] - out[-1][1]) > 1e-6:
        out.append(pf)
    out.append(end)
    return out


def build_spine(paths: Sequence[Sequence[Pt]], base_obs: 'ts.Obstacles',
                H: float, extra=None, log=None,
                teeth: Optional[Sequence[Pt]] = None,
                stubs: Optional[Sequence[Pt]] = None,
                dest_box=None,
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

    def spread_across(pts, c, u):
        return max([0.0] + [abs((p[0] - c[0]) * u[1] - (p[1] - c[1]) * u[0]) for p in pts])
    straight = _angle(u_t, u_s) <= 30.0
    obs = None
    if straight:
        # ONE straight line through the midpoint: the channel between
        # two facing arrays, however their centroids are offset -- the
        # offset is the lanes' morph, not a tilt of the frame
        u = _unit((u_t[0] + u_s[0], u_t[1] + u_s[1]))
        mid = ((Ct[0] + Cs[0]) / 2, (Ct[1] + Cs[1]) / 2)
        ta = (Ct[0] - mid[0]) * u[0] + (Ct[1] - mid[1]) * u[1]
        tb = (Cs[0] - mid[0]) * u[0] + (Cs[1] - mid[1]) * u[1]
        a = (mid[0] + ta * u[0], mid[1] + ta * u[1])
        b = (mid[0] + tb * u[0], mid[1] + tb * u[1])
        R_t = spread(teeth, a, u) + 0.5
        R_s = spread(stubs, b, (-u[0], -u[1])) + 0.5
        p1 = (a[0] + R_t * u[0], a[1] + R_t * u[1])
        p2 = (b[0] - R_s * u[0], b[1] - R_s * u[1])
        gap = (p2[0] - p1[0]) * u[0] + (p2[1] - p1[1]) * u[1]
        if gap <= 0.5:
            # the two zones overlap: nothing left to relax
            return Spine(align_tail(simplify([a, b], 0.08), dest_box))
        obs = RampedObstacles(base_obs, (Ct, Cs), H, extra=extra)
        if obs.seg_clear(p1, p2):
            # a clear straight channel: the chord, not relaxed (relaxing
            # it against the ramped obstacles can only add wiggles)
            sp = simplify([a, p1, p2, b], 0.08)
            sp = align_tail(sp, dest_box)
            if log:
                log(f'    spine: 2 mean pts -> 2 relaxed (0 rounds) -> '
                    f'{len(sp)} vertices, {polyline_len(sp):.2f} mm, corners []'
                    + ('  tail on the array axis' if dest_box is not None else ''))
            return Spine(sp)
    # THE MIDDLE, when the flows BEND (more than 30 degrees between
    # launch and arrival) or a big part / a laid corridor stands in the
    # chord: the members' MEAN TAUT PATH between the two end zones (the
    # bundle's medial line, in the homotopy class the taut paths chose:
    # which side of a part they went), from the teeth's centroid to the
    # stubs' centroid along their own flows, relaxed as a string against
    # the ramped obstacles by taut_fast.relax_spine (the strings' own
    # rules -- contact a constraint, a tube crossed transversally
    # transparent -- with the ramp as a per-point inflation), then made
    # OCTILINEAR (octilinearise: a bundle turns a part in legs at 0, 45
    # or 90 degrees, as a human's does; an arc's bands sit at shallow
    # angles to the router's grid and every lane became a staircase).
    # The inflation is the larger of the nominal half-width and the
    # ends' actual spread across their flows: a face of 15 teeth at the
    # ball pitch is 3.5 mm wide where the lane pitch says 2.8, and a dip
    # sized for the spine alone left the top lanes in the part (channel
    # article K15). Until 2026-09-08 the bent branch was the chord with
    # two corners: the mean-path relaxation had been pruned as never
    # reached at K28, and its absence crashed K51's singleton.
    a, b = Ct, Cs
    R_t = spread(teeth, a, u_t) + 0.5
    R_s = spread(stubs, b, (-u_s[0], -u_s[1])) + 0.5
    p1 = (a[0] + R_t * u_t[0], a[1] + R_t * u_t[1])
    p2 = (b[0] - R_s * u_s[0], b[1] - R_s * u_s[1])
    if math.hypot(p2[0] - p1[0], p2[1] - p1[1]) <= 0.5:
        return Spine(align_tail(simplify([a, b], 0.08), dest_box))
    H_eff = max(H, spread_across(teeth, Ct, u_t) + 0.1,
                spread_across(stubs, Cs, u_s) + 0.1)
    obs = RampedObstacles(base_obs, (Ct, Cs), H_eff, extra=extra,
                          zones=((Ct, u_t, R_t), (Cs, (-u_s[0], -u_s[1]), R_s)))
    M = mean_path(paths)
    keep = [p for p in M
            if math.hypot(p[0] - Ct[0], p[1] - Ct[1]) > R_t
            and math.hypot(p[0] - Cs[0], p[1] - Cs[1]) > R_s]
    init = [p1] + keep + [p2]
    used = 0
    if relax:
        pts, used = tf.relax_spine(init, obs)
        n_arc = len(simplify(pts, 0.08))
        mid = octilinearise(pts, obs, flows=(u_t, u_s))
        mid = merge_short_legs([p1] + mid[1:-1] + [p2], 2 * 0.35)
        mid = clear_legs(mid, obs)
    else:
        pts = list(init)
        n_arc = len(pts)
        mid = pts
    sp = simplify([a] + list(mid) + [b], 0.08)
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
    # axis -- it is the string OSCILLATING between pushes, which the
    # fold filter (>120 deg) never catches (take4, K35 corridor
    # SA6/SA4/SBA1: 34 vertices with 32 corners, 47 mm of spine for a
    # ~20 mm run). The frame is a coordinate AXIS, not a route: fall
    # back to the chord and let the lanes morph.
    if sum(1 for _i, _s, t_ in Spine(sp).corners() if abs(t_) > 80.0) > 4:
        if log:
            log(f'    spine DEGENERATE ({len(sp)} vertices, '
                f'{polyline_len(sp):.1f} mm) -- straight-chord fallback')
        sp = simplify([a, b], 0.08)
    if log:
        log(f'    spine: {len(init)} mean pts -> {len(pts)} relaxed '
            f'({used} rounds, H {H:.2f} -> {H_eff:.2f}) -> {n_arc} vertices -> '
            f'{len(sp)} octilinear, {polyline_len(sp):.2f} mm, '
            f'corners {[round(t_) for _i, _s, t_ in Spine(sp).corners()]}'
            + (f'  {[(round(x, 2), round(y, 2)) for x, y in sp]}' if len(sp) <= 8 else ''))
    return Spine(align_tail(sp, dest_box))


# ---------------------------------------------------------------- distances


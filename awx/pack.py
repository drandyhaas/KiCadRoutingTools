#!/usr/bin/env python3
"""pack.py -- every routed lane of a corridor pulled taut against its
neighbour, vias included, and re-emitted as copper (README TODO 8).

After a corridor's lanes are laid, each lane's copper -- every run on
either layer and every via joining them -- is one polyline from tooth
to landing. Lanes are taken in order across the corridor, and each is
relaxed as a taut string between its two ends (which never move):
curve shortening against every obstacle -- a point on a layer against
that layer's pads, foreign copper, other lanes, vias and the board
edge, inflated by the clearance and half a track, so contact IS the
clearance; a VIA point against BOTH layers at the via's own radius --
with a FOLLOW force snapping each track point into the tube of the
settled copper on its layer (the lane packed just before it, from far;
anything else settled, from near), wherever the string runs alongside
it, on the packing side, in plain sight. A via moves with its lane:
nothing pulls it directly, the string does, and it settles where a via
may sit. The relaxed string is emitted run by run as OCTILINEAR copper
where a build clears -- a hug is a line in the exact direction of the
copper it hugs, at the distance the string settled at; a free stretch
is the grid legs of its chords -- else as the string's own chords, and
validated piece by piece with exact clearance geometry at the true
radii. A lane whose copper fails that keeps the router's; every lane
is packed against the board as it stands, so a kept lane is legal
beside the packed ones.

Why the string re-emits rather than steering the router (2026-09-08):
the first port relaxed the run's string and re-routed it inside a tube
round it; the string reached the pitch in nearly every run and the
router then refused the tube in 13 of 47 (a lattice pinch at the exact
pitch) or laid the far side. A per-piece octilinear shove of the
router's copper moved almost nothing: the router's runs are staircases
of a hundred 0.05 mm pieces. And a string per RUN with the vias frozen
packed each run between joints the router had placed for the old,
spread lanes.

General: no face, axis or board name; the pitch and window arrive by
env.

    BRAID_PACK=1        pack every corridor at write time (braid.py)
    BRAID_PACK_PITCH    the packed pitch (default track + clearance + the margin)
    BRAID_PACK_WINDOW   how far settled copper attracts (1.5)
    BRAID_PACK_DEBUG=1  one line per lane
"""
import copy as _copy
import math
import os
import time as _time

import numpy as np

from kicad_parser import Segment
import topo_strings as ts
import taut_fast as tf

DEBUG = os.environ.get('BRAID_PACK_DEBUG') == '1'
STEP = 0.08                   # spacing of the string (mm)
FREEZE = ts.FREEZE            # no moves this close to a lane's end (0.35)
CAP = 0.05                    # a point moves at most this per round: below a
                              # thin capsule's radius, so it never jumps one
PUSHES = 6                    # projections per round for a wedged point
VIA_SPAN = 5                  # a via's smoothing pull comes from the points this
                              # many steps away on either side (0.4 mm): a joint
                              # feels the angle between its two stretches, not the
                              # 0.08 mm wiggle beside it (K41's SA12 via crept down
                              # its slot at 0.017 mm a round and the convergence
                              # test stopped it there, 2026-09-09)
VIA_EXIT = 2e-3               # ...and a round that moves a via more than this is
                              # not a converged one
OUT = 1e-3                    # a projected point sits this far outside
MARGIN_R = 0.008              # the relaxation's extra inflation, the budget
                              # for what the emitted copper loses: the chords
                              # of a densified arc cut inside the true circle
                              # by up to STEP^2 / (8 r) ~ 3.3 um, and the
                              # simplification moves a point up to EMIT_TOL
EMIT_TOL = 0.003              # the finest Douglas-Peucker tolerance of an
                              # emitted string (the last resort)
DP_LADDER = (0.05, 0.02, 0.008)   # coarser any-angle emissions, tried first
                              # and kept when they validate at the true radii
EXIT = 1e-3                   # a round that moves less than this per point ends
                              # the relaxation
NEAR = 2.0                    # candidate obstacles: within this of the box
PAR_SIN = 0.6                 # |sin| of the tangent-vs-neighbour angle below
                              # which the string runs ALONGSIDE (37 degrees):
                              # also the angle a transition into a hug settles at
FOLD_COS = 0.0                # a point where the string turns by more than 90
                              # degrees (the cosine of the turn below this) is a
                              # FOLD: its tangent is meaningless and it is
                              # alongside nothing. K41's SA12 hairpin tip passed
                              # the alongside test against a track it lay one
                              # pitch below, and the follow held it there against
                              # the tension (2026-09-09). By angle, not by the
                              # chord between the neighbours: a hairpin whose legs
                              # are unevenly spaced has a long chord (SA9)
MAX_EXIT = 0.01               # ...and no point may move ACROSS the string by more
                              # than this in a round that counts as converged: the
                              # sum over a lane's 500 points hid one apex collapsing
                              # at 0.1 mm a round (K41's SCS0, SA9, 2026-09-09).
                              # Across, not along: points drift along a straight
                              # stretch for hundreds of rounds (the smoothing
                              # evens their spacing) without changing its shape
BLOCK = 25                    # rounds between straightenings of the free stretches
WINDOW_PRED = 6.0             # the lane packed just before attracts from this far
                              # (3 and 2 mm measured the same on K28/K41, 2026-09-09)
PRED_END_KEEP = 0.3           # no following within this of the predecessor's ends
PRED_SIN = 0.87               # the predecessor is followed up to 60 degrees off the
                              # string's tangent (a wrap is steep; a departure steeper)
EPS = 1e-6
SLACK_TOL = 0.005             # emitted copper may sit this far inside the
                              # relaxation's clearance (0.105): the router's own
                              # copper beside a diagonal neighbour is at 0.100
VIA = -1                      # the class of a via point (a layer class is its
                              # index in the board's copper layers)
CHORD_TOL = 0.03              # settled copper simplified at this before a hug
                              # reads its direction: a router staircase of
                              # 0.05 mm pieces is ONE chord, a packed lane's
                              # legs are their own
HUG_MAX = 0.8                 # a point hugs copper within this (the pitch, or a
                              # plateau past a via); farther is a free stretch
OCT_FREE_TOLS = (0.4, 0.15, 0.06)   # a free stretch simplified at these, coarsest first
OCT_DTOL = 0.03               # a hug at a distance this different is another line
OCT_SNAP = 2.0                # degrees: a chord this close to a grid direction IS one
SNAP_DEV = 0.01               # ...if the snap moves its far end by no more than this
MIN_LEG = 0.05                # a grid leg shorter than this is dropped
JOG_MIN = 0.06                # parallel lines offset by less than this are one line
REPAIR_TOL = 0.012            # the string's chords that replace an unclear leg:
                              # MARGIN_R + SLACK_TOL is what the true-radius
                              # validation allows a chord to cut
OCT_GIVE = 0.005              # a grid leg may cut this far into the INFLATED model
                              # (MARGIN_R less the chord sagitta): a string in contact
                              # sits at the inflated radius, its legs a hair inside


# ------------------------------------------------------------ chaining

def chain_segs(segs, start):
    """Order a lane's segments into a polyline from `start`. Returns
    (points, layers, seg_objects, left): layers[i] and seg_objects[i]
    belong to the piece points[i] -> points[i+1]; `left` holds the
    segments that did not chain (a branch: the lane is skipped)."""
    def k(x, y):
        return (round(x, 4), round(y, 4))
    left = list(segs)
    pts, lays, objs = [k(*start)], [], []
    cur = pts[0]
    while left:
        nxt = None
        for i, sg in enumerate(left):
            a, b = k(sg.start_x, sg.start_y), k(sg.end_x, sg.end_y)
            if a == cur:
                nxt = (i, b)
                break
            if b == cur:
                nxt = (i, a)
                break
        if nxt is None:
            break
        i, other = nxt
        lays.append(left[i].layer)
        objs.append(left[i])
        pts.append(other)
        cur = other
        left.pop(i)
    return pts, lays, objs, left


def poly_len(pts):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def seg_len(segs):
    return sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) for s in segs)


# ------------------------------------------------------------ geometry

def _pt_seg_d2(P, S0, S1):
    """Squared distance from each point of P (n, 2) to each segment
    S0->S1 (m, 2): (n, m)."""
    dx = S1[:, 0] - S0[:, 0]
    dy = S1[:, 1] - S0[:, 1]
    ll = np.maximum(dx * dx + dy * dy, 1e-12)
    px = P[:, None, 0] - S0[None, :, 0]
    py = P[:, None, 1] - S0[None, :, 1]
    t = np.clip((px * dx[None] + py * dy[None]) / ll[None], 0.0, 1.0)
    ex = px - t * dx[None]
    ey = py - t * dy[None]
    return ex * ex + ey * ey


def _orient(P, Q, R):
    return ((Q[..., 0] - P[..., 0]) * (R[..., 1] - P[..., 1])
            - (Q[..., 1] - P[..., 1]) * (R[..., 0] - P[..., 0]))


def seg_seg_dist(A0, A1, B0, B1):
    """Distance between each segment A0->A1 (n, 2) and each B0->B1
    (m, 2): (n, m); 0 where they cross."""
    if not len(A0) or not len(B0):
        return np.zeros((len(A0), len(B0)))
    d2 = np.minimum(np.minimum(_pt_seg_d2(A0, B0, B1), _pt_seg_d2(A1, B0, B1)),
                    np.minimum(_pt_seg_d2(B0, A0, A1).T, _pt_seg_d2(B1, A0, A1).T))
    a0, a1 = A0[:, None, :], A1[:, None, :]
    b0, b1 = B0[None, :, :], B1[None, :, :]
    o1 = _orient(a0, a1, b0)
    o2 = _orient(a0, a1, b1)
    o3 = _orient(b0, b1, a0)
    o4 = _orient(b0, b1, a1)
    x = (o1 * o2 < 0) & (o3 * o4 < 0)
    return np.sqrt(np.where(x, 0.0, d2))


def _pt_seg_d2_pairs(P, S0, S1):
    """Squared distance from P[k] to the segment S0[k]->S1[k], per pair
    (a zero-length segment is a point)."""
    dx = S1[:, 0] - S0[:, 0]
    dy = S1[:, 1] - S0[:, 1]
    ll = dx * dx + dy * dy
    px = P[:, 0] - S0[:, 0]
    py = P[:, 1] - S0[:, 1]
    t = np.clip((px * dx + py * dy) / np.where(ll > 1e-12, ll, 1.0), 0.0, 1.0)
    t = np.where(ll > 1e-12, t, 0.0)
    ex = px - t * dx
    ey = py - t * dy
    return ex * ex + ey * ey, ex, ey, t


def _box_pairs(ax0, ay0, ax1, ay1, bx0, by0, bx1, by1):
    """(i, j) of the boxes a[i] and b[j] that overlap."""
    m = ((ax0[:, None] <= bx1[None, :]) & (ax1[:, None] >= bx0[None, :])
         & (ay0[:, None] <= by1[None, :]) & (ay1[:, None] >= by0[None, :]))
    return np.nonzero(m)


def _cap_boxes(C):
    """Boxes of relaxation capsules (ax, ay, dx, dy, r), inflated by r."""
    x0 = np.minimum(C[:, 0], C[:, 0] + C[:, 2]) - C[:, 4]
    x1 = np.maximum(C[:, 0], C[:, 0] + C[:, 2]) + C[:, 4]
    y0 = np.minimum(C[:, 1], C[:, 1] + C[:, 3]) - C[:, 4]
    y1 = np.maximum(C[:, 1], C[:, 1] + C[:, 3]) + C[:, 4]
    return x0, y0, x1, y1


class Caps:
    """Discs (x, y, r) and relaxation capsules (ax, ay, dx, dy, r) as ONE
    capsule array `M` (a disc a zero-length capsule) with its boxes
    computed once: the model a lane relaxes and emits against is static
    for the lane, and every test starts from the boxes."""

    def __init__(self, D, C):
        C = np.asarray(C, dtype=float).reshape(-1, 5)
        if len(D):
            Dc = np.zeros((len(D), 5))
            Dc[:, 0:2] = D[:, 0:2]
            Dc[:, 4] = D[:, 2]
            C = np.concatenate([C, Dc]) if len(C) else Dc
        self.M = C
        self.boxes = _cap_boxes(C) if len(C) else None

    def __len__(self):
        return len(self.M)


def _as_caps(D, C):
    return Caps(D, C)


def _segs_hit(A0, A1, caps, tol=EPS):
    """Per segment A0[i]->A1[i]: True when it comes within (r - tol) of a
    capsule of `caps` (a Caps), a crossing included -- the dense
    seg_seg_dist test on the pairs whose boxes overlap only (a pull
    segment meets a handful of the lane's hundreds of obstacles)."""
    n = len(A0)
    hit = np.zeros(n, dtype=bool)
    if not n or not len(caps):
        return hit
    M = caps.M
    i, j = _box_pairs(np.minimum(A0[:, 0], A1[:, 0]), np.minimum(A0[:, 1], A1[:, 1]),
                      np.maximum(A0[:, 0], A1[:, 0]), np.maximum(A0[:, 1], A1[:, 1]),
                      *caps.boxes)
    if not len(i):
        return hit
    a0, a1 = A0[i], A1[i]
    b0 = M[j, 0:2]
    b1 = b0 + M[j, 2:4]
    d2 = np.minimum(np.minimum(_pt_seg_d2_pairs(a0, b0, b1)[0], _pt_seg_d2_pairs(a1, b0, b1)[0]),
                    np.minimum(_pt_seg_d2_pairs(b0, a0, a1)[0], _pt_seg_d2_pairs(b1, a0, a1)[0]))
    x = ((_orient(a0, a1, b0) * _orient(a0, a1, b1) < 0)
         & (_orient(b0, b1, a0) * _orient(b0, b1, a1) < 0))
    hit[i[x | (d2 < (M[j, 4] - tol) ** 2)]] = True
    return hit


class World:
    """The obstacles a class of point is held off, at their TRUE
    required distances: capsules (ax, ay, bx, by, r) with r = the
    obstacle's half-width + clearance + the point's own half-width
    (a track's, or a via's), discs (x, y, r) likewise, and the board
    edge as bare segments with its own clearance. `slack` grades
    emitted copper; `arrays(extra)` hands the relaxation the same model
    inflated by `extra`."""

    def __init__(self, caps, discs, edges, edge_need):
        self.C = np.asarray(caps, dtype=float).reshape(-1, 5)
        self.D = np.asarray(discs, dtype=float).reshape(-1, 3)
        self.E = np.asarray(edges, dtype=float).reshape(-1, 4)
        self.edge_need = edge_need

    def grown(self, extra, edge_extra):
        """The same world for a fatter point (a via): every radius +
        `extra`."""
        C = self.C.copy()
        D = self.D.copy()
        if len(C):
            C[:, 4] += extra
        if len(D):
            D[:, 2] += extra
        w = World([], [], self.E, self.edge_need + edge_extra)
        w.C, w.D = C, D
        return w

    @staticmethod
    def union(a, b):
        w = World([], [], a.E, max(a.edge_need, b.edge_need))
        w.C = np.concatenate([a.C, b.C]) if len(a.C) or len(b.C) else a.C
        w.D = np.concatenate([a.D, b.D]) if len(a.D) or len(b.D) else a.D
        return w

    def slack(self, S):
        """Per segment of S (n, 4): the smallest (distance - required)
        over every obstacle; negative = a violation. A via is a
        zero-length segment."""
        S = np.asarray(S, dtype=float).reshape(-1, 4)
        n = len(S)
        out = np.full(n, np.inf)
        if not n:
            return out
        A0, A1 = S[:, :2], S[:, 2:]
        if len(self.C):
            d = seg_seg_dist(A0, A1, self.C[:, :2], self.C[:, 2:4]) - self.C[None, :, 4]
            out = np.minimum(out, d.min(1))
        if len(self.D):
            d = np.sqrt(_pt_seg_d2(self.D[:, :2], A0, A1)).T - self.D[None, :, 2]
            out = np.minimum(out, d.min(1))
        if len(self.E):
            d = seg_seg_dist(A0, A1, self.E[:, :2], self.E[:, 2:4]) - self.edge_need
            out = np.minimum(out, d.min(1))
        return out

    def worst_kind(self, pc):
        """What one piece violates most, for a log line."""
        S = np.asarray([pc], dtype=float)
        A0, A1 = S[:, :2], S[:, 2:]
        out = []
        if len(self.C):
            d = seg_seg_dist(A0, A1, self.C[:, :2], self.C[:, 2:4])[0] - self.C[:, 4]
            j = int(d.argmin())
            out.append((d[j], f'capsule ({self.C[j, 0]:.2f},{self.C[j, 1]:.2f})-'
                              f'({self.C[j, 2]:.2f},{self.C[j, 3]:.2f}) r{self.C[j, 4]:.3f}'))
        if len(self.D):
            d = np.sqrt(_pt_seg_d2(self.D[:, :2], A0, A1))[:, 0] - self.D[:, 2]
            j = int(d.argmin())
            out.append((d[j], f'disc ({self.D[j, 0]:.2f},{self.D[j, 1]:.2f}) r{self.D[j, 2]:.3f}'))
        if len(self.E):
            d = seg_seg_dist(A0, A1, self.E[:, :2], self.E[:, 2:4])[0] - self.edge_need
            out.append((float(d.min()), 'edge'))
        return min(out)[1] if out else '-'

    def arrays(self, extra):
        """(D, C) for the relaxation: discs (x, y, r) and capsules (ax,
        ay, dx, dy, r), every radius + `extra`, the edge as capsules."""
        D = self.D.copy()
        if len(D):
            D[:, 2] += extra
        C = np.zeros((len(self.C) + len(self.E), 5))
        if len(self.C):
            C[:len(self.C), 0:2] = self.C[:, 0:2]
            C[:len(self.C), 2:4] = self.C[:, 2:4] - self.C[:, 0:2]
            C[:len(self.C), 4] = self.C[:, 4] + extra
        if len(self.E):
            C[len(self.C):, 0:2] = self.E[:, 0:2]
            C[len(self.C):, 2:4] = self.E[:, 2:4] - self.E[:, 0:2]
            C[len(self.C):, 4] = self.edge_need + extra
        return D, C


# ------------------------------------------------------------ the string

def _nearest_on_segments(Q, T):
    """Per point of Q (n, 2): the distance to the nearest of the segments
    T (m, 4: ax ay bx by), the foot point, the unit normal from the foot
    to the point, and the segment's unit direction."""
    ax, ay, bx, by = T[:, 0], T[:, 1], T[:, 2], T[:, 3]
    dx, dy = bx - ax, by - ay
    ll = np.maximum(dx * dx + dy * dy, 1e-12)
    t = np.clip(((Q[:, 0:1] - ax) * dx + (Q[:, 1:2] - ay) * dy) / ll, 0.0, 1.0)
    fx = ax + t * dx
    fy = ay + t * dy
    ex = Q[:, 0:1] - fx
    ey = Q[:, 1:2] - fy
    d2 = ex * ex + ey * ey
    j = d2.argmin(1)
    ar = np.arange(len(Q))
    d = np.sqrt(d2[ar, j])
    safe = np.where(d < 1e-9, 1.0, d)
    L = np.sqrt(ll[j])
    return d, fx[ar, j], fy[ar, j], ex[ar, j] / safe, ey[ar, j] / safe, dx[j] / L, dy[j] / L, j


def _deepest_plain(Q, caps, margin=0.0):
    """Per point: the deepest violated capsule of `caps` -> (depth, nx, ny),
    depth <= 0 where none. taut_fast._deepest WITHOUT the crossing rule:
    on one layer nothing a packed run meets is a dive, every capsule
    pushes. Evaluated on the (point, capsule) pairs whose boxes overlap
    (the boxes grown by `margin`: with one, a LEGAL point within it of
    a capsule reports that capsule as its nearest wall, depth < 0); a
    point on a capsule's axis is pushed along its normal, a point at a
    disc's centre along +x."""
    n = len(Q)
    best = np.full(n, -np.inf)
    nx = np.ones(n)
    ny = np.zeros(n)
    if not n or not len(caps):
        return best, nx, ny
    M = caps.M
    bx0, by0, bx1, by1 = caps.boxes
    if margin:
        bx0, by0, bx1, by1 = bx0 - margin, by0 - margin, bx1 + margin, by1 + margin
    i, j = _box_pairs(Q[:, 0], Q[:, 1], Q[:, 0], Q[:, 1], bx0, by0, bx1, by1)
    if not len(i):
        return best, nx, ny
    b0 = M[j, 0:2]
    d2, ex, ey, _t = _pt_seg_d2_pairs(Q[i], b0, b0 + M[j, 2:4])
    d = np.sqrt(d2)
    safe = np.where(d < 1e-9, 1.0, d)
    sx, sy = M[j, 2], M[j, 3]
    L2 = sx * sx + sy * sy
    L = np.sqrt(np.where(L2 > 0, L2, 1.0))
    dep = M[j, 4] - d
    NX = np.where(d < 1e-9, np.where(L2 > 0, -sy / L, 1.0), ex / safe)
    NY = np.where(d < 1e-9, np.where(L2 > 0, sx / L, 0.0), ey / safe)
    # the deepest per point: pairs ordered by point then depth descending
    order = np.lexsort((-dep, i))
    first = np.ones(len(order), dtype=bool)
    first[1:] = i[order][1:] != i[order][:-1]
    sel = order[first]
    best[i[sel]] = dep[sel]
    nx[i[sel]] = NX[sel]
    ny[i[sel]] = NY[sel]
    return best, nx, ny


def _nearest_cap(Q, caps, margin):
    """Per point: the nearest capsule of `caps` within `margin` of it ->
    (depth, j, t): depth = r - distance (negative outside), j its row
    (-1 when none), t the parameter of the nearest point on its axis
    (0 or 1 at an end; a disc is all end)."""
    n = len(Q)
    depth = np.full(n, -np.inf)
    jj = np.full(n, -1, dtype=int)
    tt = np.zeros(n)
    if not n or not len(caps):
        return depth, jj, tt
    M = caps.M
    bx0, by0, bx1, by1 = caps.boxes
    i, j = _box_pairs(Q[:, 0], Q[:, 1], Q[:, 0], Q[:, 1],
                      bx0 - margin, by0 - margin, bx1 + margin, by1 + margin)
    if not len(i):
        return depth, jj, tt
    b0 = M[j, 0:2]
    d2, _ex, _ey, t = _pt_seg_d2_pairs(Q[i], b0, b0 + M[j, 2:4])
    dep = M[j, 4] - np.sqrt(d2)
    order = np.lexsort((-dep, i))
    first = np.ones(len(order), dtype=bool)
    first[1:] = i[order][1:] != i[order][:-1]
    sel = order[first]
    depth[i[sel]] = dep[sel]
    jj[i[sel]] = j[sel]
    tt[i[sel]] = t[sel]
    return depth, jj, tt


def _near_T(T, P):
    if not len(T):
        return T
    x0, y0 = P.min(0) - NEAR
    x1, y1 = P.max(0) + NEAR
    lo_x = np.minimum(T[:, 0], T[:, 2])
    hi_x = np.maximum(T[:, 0], T[:, 2])
    lo_y = np.minimum(T[:, 1], T[:, 3])
    hi_y = np.maximum(T[:, 1], T[:, 3])
    m = (hi_x >= x0) & (lo_x <= x1) & (hi_y >= y0) & (lo_y <= y1)
    return T[m]


def _tangents(P, folds=False):
    """Unit tangents (the chord between a point's two neighbours); with
    `folds`, also the mask of points whose chord is under FOLD."""
    T = np.empty_like(P)
    T[1:-1] = P[2:] - P[:-2]
    T[0] = P[1] - P[0]
    T[-1] = P[-1] - P[-2]
    L = np.hypot(T[:, 0], T[:, 1])
    U = T / np.where(L < 1e-12, 1.0, L)[:, None]
    if not folds:
        return U
    fold = np.zeros(len(P), dtype=bool)
    u = P[1:-1] - P[:-2]
    v = P[2:] - P[1:-1]
    lu = np.hypot(u[:, 0], u[:, 1])
    lv = np.hypot(v[:, 0], v[:, 1])
    cosang = (u[:, 0] * v[:, 0] + u[:, 1] * v[:, 1]) / np.where((lu * lv) < 1e-12, 1.0, lu * lv)
    fold[1:-1] = cosang < FOLD_COS
    return U, fold


def _chord_clear(a, b, M, tol=1e-6):
    """True when the chord a->b clears every capsule of the (inflated)
    model M, to `tol` inside it."""
    return not _segs_hit(np.asarray([a], dtype=float), np.asarray([b], dtype=float), M, tol)[0]


STR_TOL = 0.06                # a free stretch's vertices for the taut path: its
                              # Douglas-Peucker corners at this tolerance...
STR_SUB = 0.5                 # ...plus a vertex every this far along a long
                              # straight piece, so the path can leave a leg
                              # partway (from the foot of SCKE0's east leg every
                              # chord grazed a neighbour's end by a micron; from
                              # 0.3 mm up it was free)
STR_MAX = 48                  # ...at most this many (the tolerance doubles until)


def _dp_indices(seg, tol):
    """Douglas-Peucker on seg (k, 2): the indices kept, first and last
    included, in order."""
    n = len(seg)
    keep = np.zeros(n, dtype=bool)
    keep[0] = keep[-1] = True
    stack = [(0, n - 1)]
    while stack:
        a, b = stack.pop()
        if b - a < 2:
            continue
        ax, ay = seg[a]
        dx, dy = seg[b, 0] - ax, seg[b, 1] - ay
        L = math.hypot(dx, dy)
        px = seg[a + 1:b, 0] - ax
        py = seg[a + 1:b, 1] - ay
        if L < 1e-12:
            d = np.hypot(px, py)
        else:
            d = np.abs(px * dy - py * dx) / L
        j = int(np.argmax(d))
        if d[j] > tol:
            k = a + 1 + j
            keep[k] = True
            stack.append((a, k))
            stack.append((k, b))
    return np.nonzero(keep)[0]


def _straighten(P, cls, free, hug, Ms):
    """Every maximal stretch of free points of ONE class that are NOT
    hugging a neighbour is pulled TAUT: its vertices (Douglas-Peucker at
    STR_TOL, plus its two anchoring neighbours) form a visibility graph
    -- an edge where the chord clears the class's inflated model -- and
    the string is re-laid along the shortest path through it. This is
    the tension the diffusion cannot apply in a hundred rounds; a greedy
    "farthest clear chord by halving" before it landed on the index
    midpoint of a detour, the detour's own apex, and re-laid the detour
    as two chords every time (K41's SCKE0 ride, 3 mm up the board's edge
    and back, 2026-09-09). Hugging points, frozen points and vias (their
    own class, never a stretch) stay."""
    n = len(P)
    i = 0
    while i < n:
        c = cls[i]
        if not free[i] or hug[i] or c == VIA:
            i += 1
            continue
        j = i
        while j + 1 < n and free[j + 1] and not hug[j + 1] and cls[j + 1] == c:
            j += 1
        lo, hi = i - 1, j + 1
        M = Ms[c]
        seg = P[lo:hi + 1]
        tol = STR_TOL
        V = _dp_indices(seg, tol)
        while len(V) > STR_MAX:
            tol *= 2.0
            V = _dp_indices(seg, tol)
        # a vertex every STR_SUB along a long straight piece
        extra = []
        for a, b in zip(V, V[1:]):
            L = math.hypot(seg[b, 0] - seg[a, 0], seg[b, 1] - seg[a, 1])
            k_ = int(L // STR_SUB)
            if k_ >= 1 and b - a > k_:
                extra.extend(a + (b - a) * t // (k_ + 1) for t in range(1, k_ + 1))
        if extra:
            V = np.unique(np.concatenate([V, np.asarray(extra, dtype=int)]))
        V = V + lo
        m = len(V)
        if m >= 2:
            # edges: a clear chord between any two vertices (re-laid
            # straight), and always the string's own arc between
            # consecutive vertices (kept as it is) -- a chord across a
            # contact arc never clears, so the arc is the fallback. Every
            # pair is tested in one sparse call
            ia, ib = np.triu_indices(m, 1)
            clear = ~_segs_hit(P[V[ia]], P[V[ib]], M)
            ok = np.zeros((m, m), dtype=bool)
            ok[ia, ib] = clear
            best = np.full(m, np.inf)
            prev = np.full(m, -1, dtype=int)
            straight = np.zeros(m, dtype=bool)
            best[0] = 0.0
            for a in range(m):
                pa = P[V[a]]
                if a + 1 < m:
                    arc = poly_len([tuple(p_) for p_ in P[V[a]:V[a + 1] + 1]])
                    if best[a] + arc < best[a + 1]:
                        best[a + 1] = best[a] + arc
                        prev[a + 1] = a
                        straight[a + 1] = False
                for b in range(a + 1, m):
                    if not ok[a, b]:
                        continue
                    pb = P[V[b]]
                    d = best[a] + math.hypot(pb[0] - pa[0], pb[1] - pa[1])
                    if d < best[b] - 1e-9:
                        best[b] = d
                        prev[b] = a
                        straight[b] = True
            path = [m - 1]
            while prev[path[-1]] >= 0:
                path.append(int(prev[path[-1]]))
            path.reverse()
            for a, b in zip(path, path[1:]):
                if not straight[b]:
                    continue
                ka, kb = int(V[a]), int(V[b])
                mm = kb - ka
                for s_ in range(1, mm):
                    P[ka + s_] = P[ka] + (P[kb] - P[ka]) * (s_ / mm)
        i = j + 1
    return P


def _side_of(P, cls, Ts, window):
    """The side (+1 left / -1 right of the string's direction) the
    settled copper lies on, by majority over the track points that have
    any within `window` on their layer; 0 when none."""
    tg = _tangents(P)
    v = 0.0
    for c, T in enumerate(Ts):
        m = cls == c
        if not len(T) or m.sum() < 2:
            continue
        d, fx, fy, _ux, _uy, _vx, _vy, _j = _nearest_on_segments(P[m], T)
        s = tg[m, 0] * (fy - P[m, 1]) - tg[m, 1] * (fx - P[m, 0])
        s = s[d < window]
        if len(s):
            v += float(np.sign(s).sum())
    return 0 if v == 0 else (1 if v > 0 else -1)



TRACE = set(int(x) for x in os.environ.get('BRAID_PACK_TRACE', '').split(',') if x)
DUMP = set(x for x in os.environ.get('BRAID_PACK_DUMP', '').split(',') if x)   # lanes whose
                              # relaxed string is saved to tmp/dump_<net>.npz


def relax_lane(P, cls, anchors, Ds, Cs, Ts, pitch, window, rounds=250, gain=1.0, side=0,
               Tp=None, window_pred=None):
    """The lane's string relaxed: Jacobi rounds that smooth, snap every
    track point that runs alongside settled copper on its layer and on
    the packing `side` into that copper's tube (at `pitch`), cap the
    step, project every point out of its class's obstacles (a via out
    of both layers' at the via's radius) -- a point still wedged after
    PUSHES stays where it was, so the string is legal after every round
    -- and straighten the free stretches every BLOCK rounds. Nothing
    within FREEZE of an anchor moves. Returns (P, rounds used)."""
    n = len(P)
    free = np.ones(n, dtype=bool)
    free[0] = free[-1] = False
    for a in anchors:
        free &= np.hypot(P[:, 0] - a[0], P[:, 1] - a[1]) >= FREEZE
    idx = np.nonzero(free)[0]
    if not len(idx):
        return P, 0
    near = [tf._near(D, C, P) for D, C in zip(Ds, Cs)]
    Ms = [_as_caps(Dn, Cn) for Dn, Cn in near]     # one capsule array per class
    Tn = [_near_T(T, P) for T in Ts]
    # the predecessor is known by name: it attracts from `window_pred`
    # (far), the rest of the settled copper from `window`. The outer
    # lanes of a river kept their dip when the lane packed before them
    # had moved beyond 1.5 mm (K28's bottom river, 2026-09-08)
    Tpn = [_near_T(T, P) for T in Tp] if Tp is not None else None
    wp = window_pred if window_pred else window
    # the predecessor's free ends per layer: endpoints its segments share
    # with no other segment of its (a tooth, a landing, a via)
    pend = []
    for T in (Tpn or []):
        pts_ = {}
        for (ax_, ay_, bx_, by_) in T:
            for q_ in ((round(ax_, 4), round(ay_, 4)), (round(bx_, 4), round(by_, 4))):
                pts_[q_] = pts_.get(q_, 0) + 1
        pend.append(np.asarray([q_ for q_, k_ in pts_.items() if k_ == 1], dtype=float).reshape(-1, 2))
    hug = np.zeros(n, dtype=bool)
    vias_i = [int(i) for i in np.nonzero(cls == VIA)[0] if free[i]]
    # the pull gain per point: halved each time a point's move reverses
    # between rounds (a snap toward the tube answered by a push out of an
    # obstacle, or two targets taking turns): thirteen K41 lanes ran to
    # the round cap with a few points ping-ponging by the full step
    # (2026-09-09); and the convergence test reads the NET move over two
    # rounds, which a damped oscillator passes
    gain_pt = np.full(n, gain)
    pmx = np.zeros(len(idx))
    pmy = np.zeros(len(idx))
    P2 = P.copy()
    it = 0
    for it in range(rounds):
        Q = P.copy()
        Q[1:-1] = 0.5 * P[1:-1] + 0.25 * (P[:-2] + P[2:])
        for i in vias_i:
            k = min(VIA_SPAN, i, n - 1 - i)
            Q[i] = 0.5 * P[i] + 0.25 * (P[i - k] + P[i + k])
        hug[:] = False
        tg, fold = _tangents(P, folds=True)
        for c, T in enumerate(Tn):
            sel = idx[cls[idx] == c]
            if not len(sel) or not len(T):
                continue
            Qf = Q[sel]
            d, fx, fy, ux, uy, vx, vy, _j = _nearest_on_segments(Qf, T)
            tgc = tg[sel]
            flat = ~fold[sel]
            along = (np.abs(tgc[:, 0] * vy - tgc[:, 1] * vx) < PAR_SIN) & (d < window) & flat
            if Tpn is not None and c < len(Tpn) and len(Tpn[c]):
                dp, fxp, fyp, uxp, uyp, vxp, vyp, _jp = _nearest_on_segments(Qf, Tpn[c])
                # the predecessor is followed wherever it is within reach,
                # whatever the string's own tangent (a string already
                # wrapping a via is steep exactly where the plateau must
                # replace the wrap) -- except where the foot sits at one of
                # the predecessor's ends, which would drag the string into
                # a hook
                alp = (dp < wp) & (np.abs(tgc[:, 0] * vyp - tgc[:, 1] * vxp) < PRED_SIN) & flat
                if len(pend[c]):
                    de = np.hypot(fxp[:, None] - pend[c][None, :, 0],
                                  fyp[:, None] - pend[c][None, :, 1]).min(1)
                    alp &= de > PRED_END_KEEP
                use = alp & (~along | (dp <= d + EPS))
                d = np.where(use, dp, d)
                fx = np.where(use, fxp, fx)
                fy = np.where(use, fyp, fy)
                ux = np.where(use, uxp, ux)
                uy = np.where(use, uyp, uy)
                vx = np.where(use, vxp, vx)
                vy = np.where(use, vyp, vy)
                along = along | alp
            sd = side[c] if isinstance(side, (list, tuple)) else side
            if sd:
                # one side only: a lane with settled copper on both sides
                # was snapped to each in turn and came out wavy
                sgn = tgc[:, 0] * (fy - Qf[:, 1]) - tgc[:, 1] * (fx - Qf[:, 0])
                along &= (np.sign(sgn) == sd)
            hug[sel[along]] = True
            # THE TARGET DISTANCE, PER STRETCH NOT PER POINT: the pitch, or
            # what clears every disc (a via, a pad) standing between the
            # neighbour's line and the lane at that cross-section -- and
            # that requirement spread under 45-degree ramps along the
            # string (its upper envelope with unit slope), so the lane jogs
            # out once, runs STRAIGHT at the via's distance past the whole
            # cluster and jogs back, instead of wrapping each via (the
            # bottom bundle's wave, copied outward lane by lane)
            Dn = near[c][0]
            req = np.full(len(sel), pitch)
            if len(Dn) and along.any():
                ai = np.nonzero(along)[0]
                gx = Dn[None, :, 0] - fx[ai, None]
                gy = Dn[None, :, 1] - fy[ai, None]
                # (along, across) of each disc about the foot, across
                # positive toward the lane
                al = gx * vx[ai, None] + gy * vy[ai, None]
                ac = gx * ux[ai, None] + gy * uy[ai, None]
                r = Dn[None, :, 2]
                hit = (np.abs(al) <= r) & (ac > 0) & (ac - r < pitch + EPS)
                need = np.where(hit, ac + r, 0.0).max(1)
                req[ai] = np.maximum(req[ai], need)
            if (req > pitch + EPS).any():
                # the envelope under unit-slope ramps, along the string:
                # req_i = max_j (req_j - |s_i - s_j|), s = index * STEP
                s_ = sel.astype(float) * STEP
                env = (req[None, :] - np.abs(s_[:, None] - s_[None, :])).max(1)
                req = np.maximum(req, env)
            # two-sided: a point alongside sits ON the profile -- pulled in
            # when farther, pushed out onto the plateau when nearer (the
            # one-sided tube let the string curve back in past a via and
            # wrap it)
            pull = along & (np.abs(d - req) > 1e-4)
            if not pull.any():
                continue
            tx = fx + ux * req
            ty = fy + uy * req
            # only toward a target in plain sight: the way from the
            # point to its packed position must cross no obstacle (a
            # stub behind a via pulled strings round the via, +2 mm)
            pi = np.nonzero(pull)[0]
            A0 = Qf[pi]
            A1 = np.stack([tx[pi], ty[pi]], 1)
            blocked = _segs_hit(A0, A1, Ms[c])
            if TRACE:
                for k_, gi in enumerate(sel[pi]):
                    if int(gi) in TRACE:
                        print(f'      trace r{it} i={gi} c{c} at ({Qf[pi[k_],0]:.3f},{Qf[pi[k_],1]:.3f}) '
                              f'-> target ({tx[pi[k_]]:.3f},{ty[pi[k_]]:.3f}) d {d[pi[k_]]:.3f} req {req[pi[k_]]:.3f} '
                              f'{"BLOCKED" if blocked[k_] else "pulled"}')
            # a point whose pull is refused hugs nothing: marked hugging it
            # would anchor the straightening (K41's SA9 kept a spike into a
            # pad gap for 250 rounds, its base points "hugging" a track a
            # millimetre away they could never reach, 2026-09-09)
            hug[sel[pi[blocked]]] = False
            pi = pi[~blocked]
            g = gain_pt[sel[pi]]
            Q[sel[pi], 0] += g * (tx[pi] - Qf[pi, 0])
            Q[sel[pi], 1] += g * (ty[pi] - Qf[pi, 1])
        # the step capped; then every class projected out of its world
        mv = Q[idx] - P[idx]
        mag = np.hypot(mv[:, 0], mv[:, 1])
        big = mag > CAP
        if big.any():
            mv[big] *= (CAP / mag[big])[:, None]
        Qf = P[idx] + mv
        for c in range(len(Ds)):
            cm = (cls[idx] == (VIA if c == len(Ds) - 1 else c))
            sub = np.nonzero(cm)[0]
            if not len(sub):
                continue
            s2 = sub
            for _k in range(PUSHES):
                if not len(s2):
                    break
                depth, nx, ny = _deepest_plain(Qf[s2], Ms[c])
                v = depth > 0.0
                if not v.any():
                    s2 = s2[:0]
                    break
                s3 = s2[v]
                Qf[s3, 0] += nx[v] * (depth[v] + OUT)
                Qf[s3, 1] += ny[v] * (depth[v] + OUT)
                s2 = s3
            if len(s2):
                depth, _nx, _ny = _deepest_plain(Qf[s2], Ms[c])
                stuck = s2[depth > 0.0]
                if len(stuck):
                    # WEDGED: the move pressed the point into a wall and
                    # the pushes ping-ponged (a via in the slot between two
                    # pads has 2 um of room in this model). SLIDE instead:
                    # the move less its component INTO the nearest wall at
                    # the point's legal position -- the projected step along
                    # the wall -- kept when the projection converges from
                    # there, else the point stays (K41's SA12 via sat wedged
                    # between R4's pads for 150 rounds with the slot open
                    # below it, 2026-09-09)
                    P0 = P[idx[stuck]]
                    _d, wx, wy = _deepest_plain(P0, Ms[c], margin=CAP + OUT)
                    mvs = mv[stuck]
                    into = np.minimum(mvs[:, 0] * wx + mvs[:, 1] * wy, 0.0)
                    Qs = P0 + mvs - into[:, None] * np.stack([wx, wy], 1)
                    s4 = np.arange(len(stuck))
                    for _k in range(PUSHES):
                        if not len(s4):
                            break
                        depth, nx, ny = _deepest_plain(Qs[s4], Ms[c])
                        v = depth > 0.0
                        if not v.any():
                            s4 = s4[:0]
                            break
                        s5 = s4[v]
                        Qs[s5, 0] += nx[v] * (depth[v] + OUT)
                        Qs[s5, 1] += ny[v] * (depth[v] + OUT)
                        s4 = s5
                    ok = np.ones(len(stuck), dtype=bool)
                    if len(s4):
                        depth, _nx, _ny = _deepest_plain(Qs[s4], Ms[c])
                        ok[s4[depth > 0.0]] = False
                    Qf[stuck[ok]] = Qs[ok]
                    Qf[stuck[~ok]] = P0[~ok]              # wedged: stay legal
        mvx = Qf[:, 0] - P[idx, 0]
        mvy = Qf[:, 1] - P[idx, 1]
        dm = np.hypot(mvx, mvy)
        flip = (mvx * pmx + mvy * pmy < 0) & (dm > 0.005) & (np.hypot(pmx, pmy) > 0.005)
        gain_pt[idx[flip]] = np.maximum(gain_pt[idx[flip]] * 0.5, 0.02)
        pmx, pmy = mvx, mvy
        moved = float(dm.sum())
        via_moved = float(dm[cls[idx] == VIA].max()) if vias_i else 0.0
        P[idx] = Qf
        nx2 = P[idx, 0] - P2[idx, 0]
        ny2 = P[idx, 1] - P2[idx, 1]
        across = np.abs(nx2 * tg[idx, 1] - ny2 * tg[idx, 0])       # net, over two rounds
        P2 = P.copy()
        if it % BLOCK == BLOCK - 1:
            P = _straighten(P, cls, free, hug, Ms)
        if moved < EXIT * n and via_moved < VIA_EXIT and float(across.max()) < MAX_EXIT \
                and it >= BLOCK:
            break
        if DEBUG and it == rounds - 1:
            top = np.argsort(-across)[:3]
            print('      at the round cap, across-movers: ' + '; '.join(
                f'i={idx[k]} c{cls[idx[k]]} ({P[idx[k], 0]:.3f},{P[idx[k], 1]:.3f}) across {across[k]:.4f} '
                f'hug {bool(hug[idx[k]])}' for k in top))
    P = _straighten(P, cls, free, hug, Ms)
    return P, it + 1, hug


def lane_string(pts, lays, layers):
    """A chained lane as a string: points every STEP with a class per
    point (the layer index of the piece it lies on; VIA at a vertex
    where the layer changes). Vertices are kept."""
    P, cls = [tuple(pts[0])], [layers.index(lays[0])]
    for i, (a, b) in enumerate(zip(pts, pts[1:])):
        c = layers.index(lays[i])
        if i:
            cls[-1] = VIA if lays[i - 1] != lays[i] else c
        n = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / STEP)))
        for k in range(1, n + 1):
            P.append((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n))
            cls.append(c)
    return np.asarray(P, dtype=float), np.asarray(cls, dtype=int)



# ------------------------------------------------------------ the emitter

_DIRS = [(math.cos(math.radians(45.0 * k)), math.sin(math.radians(45.0 * k))) for k in range(8)]


def _grid_dir(ux, uy, L=0.0):
    """The grid direction within OCT_SNAP degrees of (ux, uy) when a line
    of length `L` turned onto it stays within SNAP_DEV of where it was
    (a 2-degree snap moves the end of a 6 mm hug by 0.2 mm, into the
    neighbour it hugs); else None."""
    ang = math.degrees(math.atan2(uy, ux))
    k = int(round(ang / 45.0)) % 8
    dev = abs(((ang - 45.0 * k) + 180.0) % 360.0 - 180.0)
    if dev <= OCT_SNAP and L * math.sin(math.radians(dev)) <= SNAP_DEV:
        return _DIRS[k]
    return None


def _isect(p, u, q, v):
    den = u[0] * v[1] - u[1] * v[0]
    if abs(den) < 1e-9:
        return None
    s = ((q[0] - p[0]) * v[1] - (q[1] - p[1]) * v[0]) / den
    return (p[0] + s * u[0], p[1] + s * u[1])


def _is_grid(a, b, tol=2.5e-4):
    """On a grid direction, to the writer's 0.1 um rounding of both ends."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    return abs(dx) < tol or abs(dy) < tol or abs(abs(dx) - abs(dy)) < tol


def _chords_of(T):
    """Settled segments (m, 4) chained at shared endpoints into polylines
    and simplified at CHORD_TOL: the lines a hug follows. A router
    staircase of 0.05 mm pieces is one chord; a packed lane's legs are
    their own. A junction or a free end starts a path."""
    if not len(T):
        return np.zeros((0, 4))

    def k(x, y):
        return (round(float(x), 4), round(float(y), 4))
    adj = {}
    for i, (ax, ay, bx, by) in enumerate(T):
        adj.setdefault(k(ax, ay), []).append(i)
        adj.setdefault(k(bx, by), []).append(i)
    used = set()
    paths = []

    def walk(start, i):
        path = [start]
        cur, ci = start, i
        while True:
            used.add(ci)
            a, b = k(T[ci, 0], T[ci, 1]), k(T[ci, 2], T[ci, 3])
            nxt = b if a == cur else a
            path.append(nxt)
            cur = nxt
            lst = adj[cur]
            if len(lst) != 2:
                break
            ci = lst[0] if lst[1] == ci else lst[1]
            if ci in used:
                break
        return path
    for key, lst in adj.items():
        if len(lst) != 2:
            for i in lst:
                if i not in used:
                    paths.append(walk(key, i))
    for i in range(len(T)):
        if i not in used:                    # a closed loop of degree-2 nodes
            paths.append(walk(k(T[i, 0], T[i, 1]), i))
    out = []
    for path in paths:
        Q = tf._simplify(np.asarray(path, dtype=float), CHORD_TOL)
        out.extend((Q[j, 0], Q[j, 1], Q[j + 1, 0], Q[j + 1, 1]) for j in range(len(Q) - 1))
    return np.asarray(out, dtype=float).reshape(-1, 4)


def _elbow_lines(a, b, M, depth=0):
    """The chord a->b as grid legs: its vector decomposed on the two
    grid directions that bracket it, in the order whose corner clears
    the (inflated) model and deviates least from the chord; a chord
    neither order can clear is halved and each half decomposed; past
    the depth, kept as it is. Returns [(point, dir)] lines."""
    vx, vy = b[0] - a[0], b[1] - a[1]
    L = math.hypot(vx, vy)
    if L < 1e-9:
        return []
    ang = math.degrees(math.atan2(vy, vx))
    lo = 45.0 * math.floor(ang / 45.0)
    d1 = (math.cos(math.radians(lo)), math.sin(math.radians(lo)))
    d2 = (math.cos(math.radians(lo + 45.0)), math.sin(math.radians(lo + 45.0)))
    det = d1[0] * d2[1] - d1[1] * d2[0]
    al = (vx * d2[1] - vy * d2[0]) / det
    be = (d1[0] * vy - d1[1] * vx) / det
    cands = []
    for d, m, e in ((d1, al, d2), (d2, be, d1)):
        Mp = (a[0] + m * d[0], a[1] + m * d[1])
        dev = abs((Mp[0] - a[0]) * vy - (Mp[1] - a[1]) * vx) / L
        if _chord_clear(a, Mp, M, OCT_GIVE) and _chord_clear(Mp, b, M, OCT_GIVE):
            cands.append((dev, [(a, d), (Mp, e)]))
    if cands:
        return min(cands, key=lambda c: c[0])[1]
    if depth < 6:
        m = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        return _elbow_lines(a, m, M, depth + 1) + _elbow_lines(m, b, M, depth + 1)
    if DEBUG:
        _elbow_lines.notes.append(
            f'chord ({a[0]:.2f},{a[1]:.2f})-({b[0]:.2f},{b[1]:.2f}) L {L:.2f} kept any-angle')
    return [(a, (vx / L, vy / L))]


_elbow_lines.notes = []


def _lines_to_poly(lines, start, end):
    """Consecutive lines [(point, dir)] met at their intersections into a
    polyline from `start` to `end`; the first line is moved through
    `start` and the last through `end` (a hug line passes within the
    emit tolerance of them). Parallel same-direction neighbours are one
    line when offset under JOG_MIN, else joined by a 45-degree jog; a leg
    that reverses, or an interior leg under MIN_LEG, drops its line and
    the rest are met again -- every pass drops a line, so it ends. None
    when two neighbours run opposite ways or nothing is left."""
    lines = [(tuple(p), tuple(d)) for p, d in lines]
    _lines_to_poly.why = ''
    while True:
        if not lines:
            _lines_to_poly.why = 'no lines'
            return None
        lines[0] = (tuple(start), lines[0][1])
        lines[-1] = (tuple(end), lines[-1][1])
        merged = []
        for (p, d) in lines:
            if merged:
                p0, d0 = merged[-1]
                if abs(d0[0] * d[1] - d0[1] * d[0]) < 1e-9:
                    if d0[0] * d[0] + d0[1] * d[1] < 0:
                        _lines_to_poly.why = 'anti-parallel lines'
                        return None
                    off = (p[0] - p0[0]) * -d0[1] + (p[1] - p0[1]) * d0[0]
                    if abs(off) < JOG_MIN:
                        if (p, d) == (tuple(end), lines[-1][1]):
                            merged[-1] = (p, d)      # the run's end wins
                        continue
                    sgn = 1.0 if off > 0 else -1.0
                    ang = math.degrees(math.atan2(d0[1], d0[0])) + sgn * 45.0
                    dj = (math.cos(math.radians(ang)), math.sin(math.radians(ang)))
                    foot = (p[0] - off * -d0[1], p[1] - off * d0[0])
                    merged.append((foot, dj))
            merged.append((p, d))
        lines = merged
        if len(lines) == 1:
            return [tuple(start), tuple(end)]
        pts = [tuple(start)]
        for (p0, d0), (p1, d1) in zip(lines, lines[1:]):
            x = _isect(p0, d0, p1, d1)
            if x is None:
                _lines_to_poly.why = 'parallel lines meet'
                return None
            pts.append(x)
        pts.append(tuple(end))
        bad = None
        for j, (p, d) in enumerate(lines):
            adv = (pts[j + 1][0] - pts[j][0]) * d[0] + (pts[j + 1][1] - pts[j][1]) * d[1]
            if adv < -EPS or (0 < j < len(lines) - 1 and adv < MIN_LEG):
                bad = j
                break
        if bad is None:
            out = [pts[0]]
            for p in pts[1:]:
                if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > 1e-6:
                    out.append(p)
            return out
        if len(lines) <= 2:
            _lines_to_poly.why = f'2 lines, leg {bad} reversed/short'
            return None
        del lines[bad]


ARC_TOL = 0.03                # a point within this of a disc's inflated circle
                              # (or a capsule end's) is ON its arc
ARC_MIN = 3                   # ...and this many in a row are a wrap...
ARC_SPAN = math.radians(30)   # ...that turns at least this much: a graze past
                              # a track's end is a free stretch, not a wrap
ARC_IN = math.radians(15)     # a tangent is emitted only where its tangent point
                              # lies this far inside the arc's span: the last
                              # tangent of a wrap that ended 5 degrees past it
                              # ran a millimetre before the exit chord met it
                              # (SODT1's V at the source, 2026-09-09)


def _arc_lines(q, i0, i1, M, dep, jj, tt):
    """The free points q[i0..i1] split into WRAPS -- runs of ARC_MIN or
    more consecutive points on the inflated circle of one disc or one
    capsule end -- and the rest. A wrap is emitted as the CHAMFER of
    that circle: the tangent lines in the grid directions whose tangent
    point lies inside the arc's angular span, in travel order (none
    inside: the one at the grid direction nearest the arc's middle). The
    lines meet at the circumscribed octagon's corners, 8 % of the
    radius outside the circle -- room the lane packed against this one
    left, since it hugged the same chamfer. Returns [(kind, i0, i1,
    lines)] with kind 'arc' (lines) or 'free' (no lines)."""
    out = []
    i = i0
    while i <= i1:
        j = int(jj[i])
        on = (j >= 0 and dep[i] > -ARC_TOL and dep[i] < ARC_TOL
              and (M[j, 2] == 0 and M[j, 3] == 0 or tt[i] <= 1e-6 or tt[i] >= 1 - 1e-6))
        if not on:
            k = i
            while k + 1 <= i1:
                j2 = int(jj[k + 1])
                on2 = (j2 >= 0 and -ARC_TOL < dep[k + 1] < ARC_TOL
                       and (M[j2, 2] == 0 and M[j2, 3] == 0 or tt[k + 1] <= 1e-6 or tt[k + 1] >= 1 - 1e-6))
                if on2:
                    break
                k += 1
            out.append(('free', i, k, None))
            i = k + 1
            continue
        end = 0.0 if tt[i] <= 0.5 else 1.0
        k = i
        while k + 1 <= i1 and int(jj[k + 1]) == j and -ARC_TOL < dep[k + 1] < ARC_TOL \
                and ((M[j, 2] == 0 and M[j, 3] == 0) or abs(tt[k + 1] - end) <= 1e-6):
            k += 1
        if k - i + 1 < ARC_MIN:
            out.append(('free', i, k, None))
            i = k + 1
            continue
        C = (M[j, 0] + end * M[j, 2], M[j, 1] + end * M[j, 3])
        R = M[j, 4]
        ph = np.unwrap(np.arctan2(q[i:k + 1, 1] - C[1], q[i:k + 1, 0] - C[0]))
        if abs(ph[-1] - ph[0]) < ARC_SPAN:
            out.append(('free', i, k, None))
            i = k + 1
            continue
        ccw = ph[-1] > ph[0]
        lo, hi = (ph[0], ph[-1]) if ccw else (ph[-1], ph[0])
        lines = []
        for kk in range(8):
            al = math.radians(45.0 * kk)
            # the travel direction along the circle at angle phi is
            # (-sin, cos) counter-clockwise, (sin, -cos) clockwise: a line
            # in direction al is tangent where phi = al -/+ 90 degrees
            phi = al - math.pi / 2 if ccw else al + math.pi / 2
            # into the arc's unwrapped range
            while phi < lo - 1e-9:
                phi += 2 * math.pi
            while phi > lo + 2 * math.pi:
                phi -= 2 * math.pi
            if lo + ARC_IN <= phi <= hi - ARC_IN:
                lines.append((phi, (C[0] + R * math.cos(phi), C[1] + R * math.sin(phi)),
                              (math.cos(al), math.sin(al))))
        if not lines:
            # no grid tangent well inside the arc: a free stretch after all
            out.append(('free', i, k, None))
            i = k + 1
            continue
        lines.sort(key=lambda L_: L_[0], reverse=not ccw)
        out.append(('arc', i, k, [(p_, d_) for _phi, p_, d_ in lines]))
        i = k + 1
    return out


def _hug_groups(q, TC, hug_j, hug_d):
    """The run's points as maximal groups: ('hug', i0, i1) where they
    follow one chord at one distance (within OCT_DTOL), ('free', i0, i1)
    elsewhere. A hug of one or two points is a flicker and joins the
    free stretch round it."""
    n = len(q)
    hj = np.array(hug_j, dtype=int)
    hd = np.array(hug_d, dtype=float)
    for i in range(1, n - 1):                  # a one-point hole closes
        if hj[i] < 0 and hj[i - 1] >= 0 and hj[i - 1] == hj[i + 1]:
            hj[i] = hj[i - 1]
            hd[i] = 0.5 * (hd[i - 1] + hd[i + 1])
    groups = []
    i = 0
    while i < n:
        if hj[i] >= 0:
            j = i
            ref = hd[i]
            while j + 1 < n and hj[j + 1] == hj[i] and abs(hd[j + 1] - ref) <= OCT_DTOL:
                j += 1
            groups.append(['hug', i, j])
        else:
            j = i
            while j + 1 < n and hj[j + 1] < 0:
                j += 1
            groups.append(['free', i, j])
        i = j + 1
    merged = []
    for g in groups:
        if g[0] == 'hug' and g[2] - g[1] < 2:
            g = ['free', g[1], g[2]]
        if merged and merged[-1][0] == 'free' and g[0] == 'free':
            merged[-1][2] = g[2]
        else:
            merged.append(g)
    return [tuple(g) for g in merged], hd


def _octilinear_run(q, M, TC, hug_j, hug_d, free_tol):
    """A run's string as an exactly octilinear polyline between its two
    ends, from the string's OWN structure: every maximal group of points
    hugging one chord of the settled copper (same chord, same distance)
    becomes ONE line in that chord's direction -- snapped to the grid
    when it is within OCT_SNAP of it, oriented along the string -- at
    the distance the string settled at; every free group becomes the
    grid legs of its chords (simplified at `free_tol`; a chord within
    OCT_SNAP of a grid direction is one line, any other the two legs
    that span it). The lines are met (_lines_to_poly); the result must
    clear the (inflated) model, else None."""
    n = len(q)
    if n < 2:
        return None
    _elbow_lines.notes = []
    groups, hd = _hug_groups(q, TC, hug_j, hug_d)
    qa = np.asarray(q, dtype=float)
    dep_, jj_, tt_ = _nearest_cap(qa, M, ARC_TOL)
    Mm = M.M
    # free groups split into wraps (chamfers) and the rest
    split = []
    for (kind, i0, i1) in groups:
        if kind == 'hug':
            split.append((kind, i0, i1, None))
        else:
            split.extend(_arc_lines(qa, i0, i1, Mm, dep_, jj_, tt_))
    groups = split
    lines = []
    m = len(groups)
    for gi, (kind, i0, i1, arc) in enumerate(groups):
        if kind == 'arc':
            lines.extend(arc)
            _octilinear_run.arcs += 1
            continue
        if kind == 'hug':
            ax, ay, bx, by = TC[int(hug_j[i0])]
            ux, uy = bx - ax, by - ay
            L = math.hypot(ux, uy)
            if L < 1e-9:
                kind = 'free'
            else:
                ux, uy = ux / L, uy / L
                tx, ty = q[i1][0] - q[i0][0], q[i1][1] - q[i0][1]
                if tx * ux + ty * uy < 0:              # along the string
                    ux, uy = -ux, -uy
                g = _grid_dir(ux, uy, math.hypot(tx, ty))
                if g is not None:
                    ux, uy = g
                nx, ny = -uy, ux
                mid = q[(i0 + i1) // 2]
                s = (mid[0] - ax) * nx + (mid[1] - ay) * ny
                off = float(np.median(hd[i0:i1 + 1])) * (1.0 if s > 0 else -1.0)
                lines.append(((ax + off * nx, ay + off * ny), (ux, uy)))
                continue
        pts = [tuple(map(float, p)) for p in q[max(i0 - 1, 0):min(i1 + 1, n - 1) + 1]]
        Qs = [tuple(map(float, p)) for p in tf._simplify(np.asarray(pts, dtype=float), free_tol)]
        for ci, (a, b) in enumerate(zip(Qs, Qs[1:])):
            vx, vy = b[0] - a[0], b[1] - a[1]
            L = math.hypot(vx, vy)
            if L < 1e-9:
                continue
            g = _grid_dir(vx / L, vy / L, L / 2)
            if g is not None:
                through = a if (gi == 0 and ci == 0) else (
                    b if (gi == m - 1 and ci == len(Qs) - 2) else ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2))
                lines.append((through, g))
            else:
                lines.extend(_elbow_lines(a, b, M))
    if not lines:
        return None
    out = _lines_to_poly(lines, tuple(map(float, q[0])), tuple(map(float, q[-1])))
    if out is None or len(out) < 2:
        _octilinear_run.note = (f'no build ({_lines_to_poly.why}; {len(lines)} lines '
                                f'from {len(groups)} groups)')
        return None
    # THE STRING'S OWN DEPTH: where the router laid copper at exactly the
    # clearance, the string sits inside this inflated model (by the 5 um
    # the pack's clearance adds plus MARGIN_R) and, boxed in, never
    # moves. A leg through such a stretch may cut as deep as the string
    # does there -- the copper is never worse than the string, and the
    # string never worse than the router's -- else every corner the
    # router threaded tight fell to coarse any-angle chords (K41's SCKE0
    # and SA13, 2026-09-09)
    dq = np.maximum(_deepest_plain(np.asarray(q, dtype=float), M)[0], 0.0)

    def allow(a, b, base):
        ia = int(np.argmin(np.hypot(q[:, 0] - a[0], q[:, 1] - a[1])))
        ib = int(np.argmin(np.hypot(q[:, 0] - b[0], q[:, 1] - b[1])))
        lo, hi = min(ia, ib), max(ia, ib)
        return base + float(dq[lo:hi + 1].max())
    # a corner that lands INSIDE the model (a chamfer's corner into a
    # neighbour that is still an arc, an elbow's corner into a via)
    # becomes the nearest string point first: the repair keeps a leg's
    # ends, and a leg starting inside an obstacle stays unclear however
    # it is patched (2026-09-09, the chamfer round)
    if len(out) > 2:
        V = np.asarray(out[1:-1], dtype=float)
        dv, _nx, _ny = _deepest_plain(V, M)
        for k_ in np.nonzero(dv > OCT_GIVE)[0]:
            ia = int(np.argmin(np.hypot(q[:, 0] - V[k_, 0], q[:, 1] - V[k_, 1])))
            if dv[k_] > OCT_GIVE + dq[ia]:
                out[k_ + 1] = (float(q[ia, 0]), float(q[ia, 1]))
        out = [p_ for i_, p_ in enumerate(out) if i_ == 0 or math.hypot(p_[0] - out[i_ - 1][0], p_[1] - out[i_ - 1][1]) > 1e-6]
    bad = [(a, b) for a, b in zip(out, out[1:]) if not _chord_clear(a, b, M, allow(a, b, OCT_GIVE))]
    if bad:
        out = _repair(out, q, M, dq)
        # the repair's own chords cut up to REPAIR_TOL into the inflated
        # model by construction (what the true-radius validation allows):
        # judged at OCT_GIVE they failed their own check and the whole run
        # fell to coarse any-angle chords (K41's SA0 ride, SA13, SCKE0:
        # "first unclear" was a 0.05 mm repair chord every time, 2026-09-09)
        still = [(a, b) for a, b in zip(out, out[1:])
                 if not _chord_clear(a, b, M, allow(a, b, REPAIR_TOL + 1e-3))]
        if still:
            a, b = still[0]
            _octilinear_run.note = (f'{len(bad)} legs unclear, {len(still)}/{len(out) - 1} after '
                                    f'the repair, first ({a[0]:.2f},{a[1]:.2f})-({b[0]:.2f},'
                                    f'{b[1]:.2f}) {"grid" if _is_grid(a, b) else "any-angle"}')
            return None
        _octilinear_run.repaired += len(bad)
    return out


_octilinear_run.repaired = 0
_octilinear_run.arcs = 0


def _repair(out, q, M, dq):
    """Every leg of `out` that cuts the (inflated) model replaced by the
    string's own chords between the string points nearest its ends (at
    REPAIR_TOL, less the string's own depth `dq` there -- a patch through
    a stretch the router laid at the exact clearance reproduces the
    string): a run mostly octilinear, any-angle only where a leg had no
    room -- an arc round a via between two hug lines, an elbow no order
    could clear."""
    pts = [out[0]]
    for a, b in zip(out, out[1:]):
        ia = int(np.argmin(np.hypot(q[:, 0] - a[0], q[:, 1] - a[1])))
        ib = int(np.argmin(np.hypot(q[:, 0] - b[0], q[:, 1] - b[1])))
        lo, hi = min(ia, ib), max(ia, ib)
        if _chord_clear(a, b, M, OCT_GIVE + float(dq[lo:hi + 1].max())):
            pts.append(b)
            continue
        if ib <= ia:
            pts.append(b)
            continue
        tol = max(0.001, REPAIR_TOL - float(dq[ia:ib + 1].max()))
        # only the string points that lie BETWEEN the leg's ends along it:
        # the nearest string point to an end can sit just behind it, and a
        # patch that steps back to it leaves a 40 um spur (K35's SA8, a
        # same-net soft-joint flag, 2026-09-09)
        vx, vy = b[0] - a[0], b[1] - a[1]
        L2 = vx * vx + vy * vy or 1e-12
        for p in tf._simplify(np.asarray(q[ia:ib + 1], dtype=float), tol):
            t = ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / L2
            if 1e-6 < t < 1.0 - 1e-6:
                pts.append((float(p[0]), float(p[1])))
        pts.append(b)
    dd = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - dd[-1][0], p[1] - dd[-1][1]) > 1e-6:
            dd.append(p)
    return dd


_octilinear_run.note = ''


def _hug_of(q, TC):
    """Per point of the run: the chord of the settled copper it hugs
    (nearest, alongside within PAR_SIN, within HUG_MAX) or -1, and the
    distance to it."""
    n = len(q)
    hj = np.full(n, -1, dtype=int)
    hd = np.zeros(n)
    if not len(TC) or n < 2:
        return hj, hd
    tg, fold = _tangents(q, folds=True)
    d, fx, fy, ux, uy, vx, vy, jj = _nearest_on_segments(q, TC)
    along = (np.abs(tg[:, 0] * vy - tg[:, 1] * vx) < PAR_SIN) & (d < HUG_MAX) & ~fold
    hj[along] = jj[along]
    hd[along] = d[along]
    return hj, hd


def _despike(q, turn_cos=-0.94, short=0.12):
    """A vertex where the run doubles back on itself (turn over ~160
    degrees) with a short leg on either side is a spur, not copper:
    dropped, and the run re-checked until none is left."""
    changed = True
    while changed and len(q) > 2:
        changed = False
        for i in range(1, len(q) - 1):
            ux, uy = q[i][0] - q[i - 1][0], q[i][1] - q[i - 1][1]
            vx, vy = q[i + 1][0] - q[i][0], q[i + 1][1] - q[i][1]
            lu, lv = math.hypot(ux, uy), math.hypot(vx, vy)
            if lu < 1e-9 or lv < 1e-9:
                del q[i]
                changed = True
                break
            if (ux * vx + uy * vy) / (lu * lv) < turn_cos and min(lu, lv) < short:
                del q[i]
                changed = True
                break
    return q


def emit_lane(P, cls, layers, models, worlds):
    """The string as copper: the runs between vias, each the first of
    -- octilinear (_octilinear_run over a ladder of free-stretch
    tolerances, coarsest first: a wiggly stretch decomposed chord by
    chord is a sawtooth no cleanup survives) against its layer's
    inflated model `models[c]` = (capsules, chords); its own chords at the
    DP_LADDER tolerances, kept when they clear the TRUE world
    `worlds[c]`; its chords at EMIT_TOL -- as [(layer, points)], and
    the via points."""
    cuts = [0] + [i for i in range(1, len(P) - 1) if cls[i] == VIA] + [len(P) - 1]
    runs, vias = [], []
    emit_lane.notes = []
    emit_lane.repaired = 0
    emit_lane.arcs = 0
    for a, b in zip(cuts, cuts[1:]):
        seg = P[a:b + 1]
        c = int(cls[a + 1]) if b > a + 1 else int(cls[a] if cls[a] != VIA else cls[b])
        M, TC = models[c]
        hj, hd = _hug_of(seg, TC)
        Q = None
        notes = []
        _octilinear_run.repaired = 0
        _octilinear_run.arcs = 0
        for tol in OCT_FREE_TOLS:
            _octilinear_run.note = ''
            Q = _octilinear_run(seg, M, TC, hj, hd, tol)
            if Q is not None:
                break
            notes.append(f'{tol}: {_octilinear_run.note}')
        if Q is None:
            if DEBUG:
                emit_lane.notes.append(f'{layers[c][0]}: ' + ' | '.join(notes))
            w = worlds[c]
            for tol in DP_LADDER:
                cand = tf._simplify(np.asarray(seg, dtype=float), tol)
                pcs = [(p[0], p[1], r[0], r[1]) for p, r in zip(cand, cand[1:])]
                if pcs and not (w.slack(pcs) < -SLACK_TOL).any():
                    Q = cand
                    break
            if Q is None:
                Q = tf._simplify(np.asarray(seg, dtype=float), EMIT_TOL)
        q = [(round(float(x), 4), round(float(y), 4)) for x, y in Q]
        q = _despike([p for i, p in enumerate(q) if i == 0 or p != q[i - 1]])
        emit_lane.repaired += _octilinear_run.repaired
        emit_lane.arcs += _octilinear_run.arcs
        runs.append((layers[c], q))
    for i in cuts[1:-1]:
        vias.append((round(float(P[i, 0]), 4), round(float(P[i, 1]), 4)))
    return runs, vias


# ------------------------------------------------------------ the pack

def _edges_of(pcb):
    bi = pcb.board_info
    rings = list(getattr(bi, 'board_outlines', None) or [])
    if not rings and getattr(bi, 'board_outline', None):
        rings = [bi.board_outline]
    E = []
    for ring in rings:
        for a, b in zip(ring, ring[1:] + ring[:1]):
            E.append((a[0], a[1], b[0], b[1]))
    if not E and bi.board_bounds:
        x0, y0, x1, y1 = bi.board_bounds
        E = [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]
    return E


def _pad_shapes(p, m, D, C):
    """A pad's clearance region, exactly: a circle as a disc, an oval as
    a capsule, a roundrect as the four edge capsules of its inset
    rectangle at radius m + the corner radius (the Minkowski sum of a
    rounded rectangle and a disc IS that), anything else as a rectangle
    (conservative at a chamfered or custom corner). A tilted rectangle
    is rotated by its residual rect_rotation."""
    x, y = p.global_x, p.global_y
    sx, sy = p.size_x, p.size_y
    if p.shape == 'circle' or (p.shape == 'oval' and abs(sx - sy) < 1e-9):
        D.append((x, y, max(sx, sy) / 2 + m))
        return
    if p.shape == 'oval':
        r = min(sx, sy) / 2
        h = (max(sx, sy) - min(sx, sy)) / 2
        if sx >= sy:
            C.append((x - h, y, x + h, y, r + m))
        else:
            C.append((x, y - h, x, y + h, r + m))
        return
    rc = 0.0
    if p.shape == 'roundrect' and p.roundrect_rratio > 0:
        rc = min(p.roundrect_rratio * min(sx, sy), min(sx, sy) / 2)
    hx, hy = sx / 2 - rc, sy / 2 - rc
    ang = math.radians(getattr(p, 'rect_rotation', 0.0) or 0.0)
    ca, sa = math.cos(ang), math.sin(ang)
    corners = []
    for ux, uy in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
        corners.append((x + ux * ca - uy * sa, y + ux * sa + uy * ca))
    if hx <= 1e-9 and hy <= 1e-9:
        D.append((x, y, rc + m))
        return
    for a, b in zip(corners, corners[1:] + corners[:1]):
        C.append((a[0], a[1], b[0], b[1], rc + m))


def _static_of(pcb, kids, layer, m):
    """The static copper on `layer`: every pad (drilled: on every
    layer), every foreign segment as a capsule, every foreign via as a
    disc, inflated by clearance + half a track; each row ends with its
    net id (-1 for none), so a net's own pads can be left out of its
    world."""
    D, C = [], []
    for ref, fp in pcb.footprints.items():
        for p in fp.pads:
            on_layer = any(L == layer or '*' in L for L in p.layers)
            if p.drill and p.drill > 0:
                on_layer = True
            if not on_layer:
                continue
            if p.pad_type == 'np_thru_hole' and p.drill:
                D.append((p.global_x, p.global_y, p.drill / 2 + m, -1))
                continue
            d_, c_ = [], []
            _pad_shapes(p, m, d_, c_)
            net = p.net_id if p.net_id else -1
            D.extend(r + (net,) for r in d_)
            C.extend(r + (net,) for r in c_)
    for s in pcb.segments:
        if s.net_id in kids or s.layer != layer:
            continue
        C.append((s.start_x, s.start_y, s.end_x, s.end_y, s.width / 2 + m, s.net_id))
    for v in pcb.vias:
        if v.net_id in kids:
            continue
        D.append((v.x, v.y, v.size / 2 + m, v.net_id))
    return D, C


def _pitch_stat(P, T, window=1.5, step=0.12):
    """Median distance from the run's points (sampled every `step`) to
    the nearest other lane on its layer, over the points that have one
    within `window`. The tidiness number beside a render."""
    if not len(T) or len(P) < 2:
        return None
    Q = np.asarray(ts.densify([tuple(p) for p in P], step), dtype=float)
    d = np.sqrt(_pt_seg_d2(Q, T[:, :2], T[:, 2:]).min(1))
    d = d[d < window]
    return float(np.median(d)) if len(d) else None


def _room(P, cls, T, side, cap=3.0):
    """How much room a lane has on `side` (+1 left / -1 right of its
    direction): the median distance from its track points to the nearest
    settled copper on that side, `cap` where none is within it."""
    if not len(T) or len(P) < 2:
        return cap
    tg = _tangents(P)
    d, fx, fy, _ux, _uy, _vx, _vy, _j = _nearest_on_segments(P, T)
    s = tg[:, 0] * (fy - P[:, 1]) - tg[:, 1] * (fx - P[:, 0])
    on = (np.sign(s) == side) & (d < cap)
    return float(np.median(d[on])) if on.any() else cap


def pack_order(c, lanes, settled_T, layers, log):
    """The lanes of corridor `c` in packing order: across the corridor
    by the plan's target slots, from
    the outer lane with MORE room on its outer side, inward. The first
    lane hugs whatever settled copper stands beside it and otherwise
    goes taut; every next lane hugs the one packed just before it, on
    the roomy side, and its elbows bulge into the room that lane
    vacated -- a bundle of router staircases across a corridor becomes
    nested elbows only in this order, each lane moving before the lane
    it would cross. Packing from the wall inward gives shorter copper
    (K41 -1.6 % against +1.7 %) and tighter rivers, but every lane
    copies the ragged static copper it packs against, and the river
    reads as a wave (K28 1484 segments against 952, 2026-09-09, the
    renders tmp/t6w_* against tmp/t6_*); the centre outward leaves the
    centre a staircase every hug copies."""
    across = [nm for nm in c.target if nm in lanes]
    if not across:
        return []
    if len(across) < 3:
        order = across
    else:
        # the room outside each outer lane, on its layer(s), away from
        # the next lane in
        room = []
        for outer, inner in ((across[0], across[1]), (across[-1], across[-2])):
            pts, lays, _o, left = chain_segs(lanes[outer][0], c.ctx.ends[outer][0])
            if left or len(pts) < 2:
                room.append(0.0)
                continue
            P, cls = lane_string(pts, lays, layers)
            Ti = [np.array([(s.start_x, s.start_y, s.end_x, s.end_y)
                            for s in lanes[inner][0] if s.layer == L], dtype=float).reshape(-1, 4)
                  for L in layers]
            s_in = _side_of(P, cls, Ti, 1e9)
            if not s_in:
                room.append(0.0)
                continue
            vals = []
            for ci_, L in enumerate(layers):
                sel = cls == ci_
                if sel.sum() > 1 and len(settled_T[ci_]):
                    vals.append(_room(P[sel], cls[sel], _near_T(settled_T[ci_], P[sel]), -s_in))
            room.append(min(vals) if vals else 3.0)
        order = across if room[0] > room[1] else across[::-1]
        log(f'  pack: room outside {across[0]} {room[0]:.2f} mm, outside {across[-1]} '
            f'{room[1]:.2f} mm -> first {order[0]}')
    return order


def pack_corridor(c, log, pitch=None, window=None, tag=''):
    """Pack the routed lanes of corridor `c` (a braid.Corridor) in
    place: `c.out_segs`, `c.out_vias`, `ctx.pcb.segments` and
    `ctx.pcb.vias` carry the packed copper after. Every lane is packed
    against the board as it stands, so a lane that keeps the router's
    copper is legal beside the packed ones. Returns (lanes packed,
    lanes)."""
    import braid as br
    ctx = c.ctx
    t0 = _time.time()
    layers = list(ctx.cfg.layers)
    m = br.CLEAR + br.TRACK / 2
    via_r = br.VIA_SIZE / 2
    via_extra = via_r - br.TRACK / 2          # a via's radius over a track's
    if pitch is None:
        pitch = float(os.environ.get('BRAID_PACK_PITCH', '0') or 0) \
            or (br.TRACK + br.CLEAR + MARGIN_R)
    if window is None:
        window = float(os.environ.get('BRAID_PACK_WINDOW', '1.5'))
    kids = set(ctx.kids)
    lanes = {nm: (list(c.out_segs.get(nm) or []), list(c.out_vias.get(nm) or []))
             for nm in c.members if c.out_segs.get(nm)}
    if not lanes:
        return 0, 0
    edges = _edges_of(ctx.pcb)
    edge_need = float(getattr(ctx.cfg, 'board_edge_clearance', 0.0) or br.CLEAR) + br.TRACK / 2
    static = {L: _static_of(ctx.pcb, kids, L, m) for L in layers}
    own_ids = {id(s) for nm in lanes for s in lanes[nm][0]}

    def world_of(L, nid):
        """Everything on L but the net's own copper, at a track's
        radii: the static model plus every other kid's live copper and
        every kid via."""
        D, C = static[L]
        C = [r[:5] for r in C if r[5] != nid] \
            + [(s.start_x, s.start_y, s.end_x, s.end_y, s.width / 2 + m)
               for s in ctx.pcb.segments
               if s.layer == L and s.net_id in kids and s.net_id != nid]
        D = [r[:3] for r in D if r[3] != nid] \
            + [(v.x, v.y, v.size / 2 + m)
               for v in ctx.pcb.vias if v.net_id in kids and v.net_id != nid]
        return World(C, D, edges, edge_need)

    def settled(L, nid, unpacked_ids):
        """The copper on L a lane may follow: everything settled -- not
        this net, not this corridor's lanes still to move."""
        return np.array([(s.start_x, s.start_y, s.end_x, s.end_y)
                         for s in ctx.pcb.segments
                         if s.layer == L and s.net_id != nid and id(s) not in unpacked_ids],
                        dtype=float).reshape(-1, 4)
    order = pack_order(c, lanes, [settled(L, -1, own_ids) for L in layers], layers, log)
    if not order:
        return 0, 0
    tot0 = sum(seg_len(v[0]) for v in lanes.values())
    n_lanes = n_packed = 0
    stat0, stat1 = [], []
    n_oct = n_runs = 0
    n_rep = [0]
    n_arc = [0]
    why = {}
    packed = []

    def pack_one(nm, follow=True):
        """Pack lane `nm` against the board as it stands; with `follow`
        False the string hugs nothing and only goes taut. Returns a
        reason string when the lane is not kept."""
        nonlocal n_packed, n_oct, n_runs
        nid, net = ctx.byname[nm]
        segs, vias = lanes[nm]
        pts, lays, objs, left = chain_segs(segs, ctx.ends[nm][0])
        if left or len(pts) < 2 or any(L not in layers for L in lays):
            return 'unchained'
        L0 = poly_len(pts)
        if L0 < 2 * FREEZE + 0.3:
            return 'short'
        P, cls = lane_string(pts, lays, layers)
        own_vias = {}
        anchors = [tuple(pts[0]), tuple(pts[-1])]
        for i in np.nonzero(cls == VIA)[0]:
            hit = [v for v in vias if math.hypot(v.x - P[i, 0], v.y - P[i, 1]) < 1e-3]
            if hit:
                own_vias[int(i)] = hit[0]
            else:
                anchors.append((float(P[i, 0]), float(P[i, 1])))
        worlds = [world_of(L, nid) for L in layers]
        wvia = worlds[0].grown(via_extra, via_r - br.TRACK / 2)
        for w in worlds[1:]:
            wvia = World.union(wvia, w.grown(via_extra, via_r - br.TRACK / 2))
        arrs = [w.arrays(MARGIN_R) for w in worlds] + [wvia.arrays(MARGIN_R)]
        Ds = [a_ for a_, _c in arrs]
        Cs = [c_ for _a, c_ in arrs]
        # the FOLLOW targets per layer: every piece of settled copper on
        # it -- the lanes packed before this one, the other corridors'
        # lanes, the fanout's stubs, foreign tracks -- and not this
        # corridor's lanes still to pack
        unp = {id(s) for om in lanes if om not in packed and om != nm for s in lanes[om][0]}
        Ts = [settled(L, nid, unp) for L in layers] + [np.zeros((0, 4))]
        Tall = {L: np.array([(s.start_x, s.start_y, s.end_x, s.end_y)
                             for s in ctx.pcb.segments
                             if s.layer == L and s.net_id in kids and s.net_id != nid
                             and id(s) not in unp], dtype=float).reshape(-1, 4) for L in layers}
        # THE PREDECESSOR, PER LAYER: the lane packed before this one --
        # nearest in the order -- that has copper on that layer (a page
        # lane on the back has nothing to hug in a front lane's copper);
        # and THE SIDE it lies on, per layer: where the PLAN put this
        # lane relative to it (its planned centreline against the
        # predecessor's), which the packed copper cannot confuse; the
        # copper's own reading when the plan has no lines
        before = order[:order.index(nm)]
        pl = getattr(c, 'lane_xy', {})
        T_pred, side = [], []
        empty = np.zeros((0, 4))
        for ci_, L in enumerate(layers):
            pn = next((om for om in reversed(before)
                       if any(s.layer == L for s in lanes[om][0])), None)
            T = np.array([(s.start_x, s.start_y, s.end_x, s.end_y)
                          for s in (lanes[pn][0] if pn else ()) if s.layer == L],
                         dtype=float).reshape(-1, 4)
            T_pred.append(T)
            # the side the predecessor's COPPER lies on, by majority over
            # the track points that have it within the window, first; the
            # plan's lines only when the copper says nothing (a far-face
            # ride runs where the plan drew no lane: SRST's plan line lay
            # below SA0's, its ride ran below SA0's ride too, but the
            # plan said +1 and the copper -1, and it never hugged -- an
            # 8 mm chord at 4 degrees, 2026-09-09)
            only = [_near_T(T, P) if c_ == ci_ else empty for c_ in range(len(layers))]
            sd = _side_of(P, cls, only, window)
            if not sd and pn and pl.get(nm) and pl.get(pn):
                A = np.asarray(ts.densify([tuple(q) for q in pl[nm]], 0.2), dtype=float)
                B = np.asarray(pl[pn], dtype=float)
                TB = np.concatenate([B[:-1], B[1:]], 1)
                sd = _side_of(A, np.zeros(len(A), dtype=int), [TB], 1e9)
            if not sd:
                sd = _side_of(P, cls, [_near_T(Ts[c_], P) if c_ == ci_ else empty
                                       for c_ in range(len(layers))], window)
            side.append(sd)
        p0 = {L: _pitch_stat([p for p, l_ in zip(pts, lays + [lays[-1]]) if l_ == L], Tall[L])
              for L in layers}
        if not follow:
            Ts = [np.zeros((0, 4)) for _ in Ts]
            T_pred = [np.zeros((0, 4)) for _ in T_pred]
        P0_ = P.copy()
        P, rounds, _hug = relax_lane(P, cls, anchors, Ds, Cs, Ts, pitch, window, side=side,
                                     Tp=T_pred, window_pred=WINDOW_PRED)
        if nm in DUMP:
            # per point: the nearest settled segment on its layer (predecessor
            # first, then the rest) and the distance -- what a hug point holds
            tgt = np.full((len(P), 5), np.nan)
            for ci_ in range(len(layers)):
                sel_ = np.nonzero(cls == ci_)[0]
                for T_, wname in ((T_pred[ci_], 1.0), (Ts[ci_], 0.0)):
                    if len(sel_) and len(T_):
                        d_, fx_, fy_, _ux, _uy, _vx, _vy, j_ = _nearest_on_segments(P[sel_], T_)
                        for k_, i_ in enumerate(sel_):
                            if np.isnan(tgt[i_, 0]) or d_[k_] < tgt[i_, 0]:
                                tgt[i_] = (d_[k_], fx_[k_], fy_[k_], wname, j_[k_])
            np.savez(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tmp', f'dump_{nm}.npz'),
                     P0=P0_, P=P, cls=cls, hug=_hug, anchors=np.asarray(anchors, dtype=float),
                     tgt=tgt, side=np.asarray(side, dtype=float))
        models = [(_as_caps(*tf._near(Ds[i], Cs[i], P)), _chords_of(_near_T(Ts[i], P)))
                  for i in range(len(layers))]
        runs, new_vias = emit_lane(P, cls, layers, models, worlds)
        L1 = sum(poly_len(q) for _L, q in runs)
        orig_pc = {(round(s.start_x, 4), round(s.start_y, 4), round(s.end_x, 4), round(s.end_y, 4))
                   for s in segs}
        orig_pc |= {(k[2], k[3], k[0], k[1]) for k in orig_pc}
        if any(len(q) < 2 for _L, q in runs) or runs[0][1][0] != tuple(pts[0]) \
                or runs[-1][1][-1] != tuple(pts[-1]):
            return 'ends moved'
        if L1 > L0 * 1.1 + 0.3:
            return f'longer {L0:.2f} -> {L1:.2f}'
        for L, q in runs:
            pieces = [(a_[0], a_[1], b_[0], b_[1]) for a_, b_ in zip(q, q[1:])]

            def _frozen(pc):
                return any(max(math.hypot(pc[0] - a_[0], pc[1] - a_[1]),
                               math.hypot(pc[2] - a_[0], pc[3] - a_[1])) <= FREEZE + EPS
                           for a_ in anchors)
            fresh = [pc for pc in pieces if pc not in orig_pc and not _frozen(pc)]
            if not fresh:
                continue
            w = worlds[layers.index(L)]
            sl = w.slack(fresh)
            if (sl < -SLACK_TOL).any():
                j = int(sl.argmin())
                return (f'clearance {sl[j]:+.4f} on {L[0]} at ({fresh[j][0]:.2f},'
                        f'{fresh[j][1]:.2f})-({fresh[j][2]:.2f},{fresh[j][3]:.2f}) '
                        f'{w.worst_kind(fresh[j])}')
        for (x, y) in new_vias:
            sl = wvia.slack([(x, y, x, y)])
            if (sl < -SLACK_TOL).any():
                return f'via {sl[0]:+.4f} at ({x:.2f},{y:.2f}) {wvia.worst_kind((x, y, x, y))}'
        for i, (La, qa) in enumerate(runs):
            for j, (Lb, qb) in enumerate(runs):
                if j <= i or La != Lb or len(qa) < 2 or len(qb) < 2:
                    continue
                A = np.asarray(qa, dtype=float)
                B = np.asarray(qb, dtype=float)
                if (seg_seg_dist(A[:-1], A[1:], B[:-1], B[1:]) < EPS).any():
                    return 'self-crossing'
        new_segs = [Segment(a_[0], a_[1], b_[0], b_[1], br.TRACK, L, nid)
                    for L, q in runs for a_, b_ in zip(q, q[1:])]
        moved_vias = []
        for (x, y), i in zip(new_vias, [i for i in range(1, len(P) - 1) if cls[i] == VIA]):
            old = own_vias.get(int(i))
            if old is None:
                continue
            v = _copy.copy(old)
            v.x, v.y = x, y
            moved_vias.append((old, v))
        own_seg_ids = {id(s) for s in segs}
        old_via_ids = {id(o) for o, _v in moved_vias}
        ctx.pcb.segments = [s for s in ctx.pcb.segments if id(s) not in own_seg_ids] + new_segs
        ctx.pcb.vias = [v for v in ctx.pcb.vias if id(v) not in old_via_ids] \
            + [v for _o, v in moved_vias]
        kept_vias = [v for v in vias if id(v) not in old_via_ids] + [v for _o, v in moved_vias]
        lanes[nm] = (new_segs, kept_vias)
        n_packed += 1
        p1 = {L: _pitch_stat(q, Tall[L]) for L, q in runs}
        for L in layers:
            if p0.get(L) is not None:
                stat0.append(p0[L])
            if p1.get(L) is not None:
                stat1.append(p1[L])
        oct_ = sum(1 for _L, q in runs if all(_is_grid(a_, b_) for a_, b_ in zip(q, q[1:])))
        n_oct += oct_
        n_runs += len(runs)
        n_rep[0] += emit_lane.repaired
        n_arc[0] += emit_lane.arcs
        if DEBUG:
            dv = max((math.hypot(v.x - o.x, v.y - o.y) for o, v in moved_vias), default=0.0)
            for note in getattr(emit_lane, 'notes', ()):
                log(f'      oct fallback {note}')
            log(f'    pack {nm}{"" if follow else " (taut, no follow)"}: {len(pts) - 1} -> '
                f'{sum(len(q) - 1 for _L, q in runs)} pieces '
                f'({oct_}/{len(runs)} runs octilinear), {L0:.2f} -> {L1:.2f} mm in {rounds} '
                f'rounds, {len(moved_vias)} via(s) moved (max {dv:.2f} mm), side '
                f'{"/".join(f"{sd:+d}" for sd in side)}; '
                'nearest lane '
                + ' '.join(f'{L[0]} {"-" if p0.get(L) is None else f"{p0[L]:.3f}"}->'
                           f'{"-" if p1.get(L) is None else f"{p1[L]:.3f}"}' for L in layers))
        return None

    failed = []
    n_taut = 0
    for nm in order:
        n_lanes += 1
        reason = pack_one(nm)
        if reason is not None and reason.startswith('longer'):
            # the follow would drag the lane onto a longer arc (packing
            # toward the roomy side pulls an inner lane out onto the outer
            # lane's detour): the lane goes TAUT instead, hugging nothing --
            # still re-emitted clean, and what packs after it hugs a line,
            # not the router's staircase
            r2 = pack_one(nm, follow=False)
            if r2 is None:
                n_taut += 1
                reason = None
            elif DEBUG:
                log(f'    pack {nm}: taut retry NOT KEPT ({r2})')
        if reason is not None:
            failed.append(nm)
            if DEBUG:
                log(f'    pack {nm}: NOT KEPT ({reason}) -- retried at the end')
        packed.append(nm)
    still = []
    for nm in failed:
        reason = pack_one(nm)
        if reason is not None and reason.startswith('longer'):
            reason = pack_one(nm, follow=False)
        if reason is not None:
            still.append(nm)
            why[reason.split()[0]] = why.get(reason.split()[0], 0) + 1
            log(f'    pack {nm}: NOT KEPT on the retry ({reason})')
    # A SECOND PASS (opt-in, BRAID_PACK_PASS2=1), every lane again in the
    # same order against the board as it now stands: an early lane packed
    # against neighbours still at their router positions -- it went taut
    # round copper that then moved, and kept the spike, hook or zigzag
    # that copper forced (K41's SA0, the tenth of 41, with a 0.6 mm spike
    # nothing held once the lanes after it had packed, 2026-09-09). It
    # takes those out and packs tighter (median 0.47 -> 0.32 mm), and
    # measured WORSE on the emission (K41 1948 -> 2272 segments, 45 -> 49
    # long any-angle pieces) at twice the time: every wrap it tightens
    # is one more arc to emit. Off until the emission earns it.
    n2 = 0
    for nm in (order if os.environ.get('BRAID_PACK_PASS2', '0') == '1' else ()):
        if nm in still:
            continue
        if pack_one(nm) is None:
            n2 += 1
    if still:
        log(f'  PACK{tag}: {len(still)} lane(s) keep the router\'s copper: {", ".join(still)}')
    tot1 = sum(seg_len(v[0]) for v in lanes.values())
    for nm in lanes:
        c.out_segs[nm] = lanes[nm][0]
        c.out_vias[nm] = lanes[nm][1]
    ctx.base_segments = list(ctx.pcb.segments)
    ctx.base_vias = list(ctx.pcb.vias)
    med0 = f'{np.median(stat0):.3f}' if stat0 else '-'
    med1 = f'{np.median(stat1):.3f}' if stat1 else '-'
    log(f'  PACK{tag}: {n_packed} of {n_lanes} lanes re-emitted from their string '
        f'({n2} re-packed on the second pass; {n_taut} taut without a follow; '
        f'{n_oct}/{n_runs} runs octilinear, '
        f'{n_arc[0]} wraps as chamfers, {n_rep[0]} legs repaired any-angle; '
        f'pitch {pitch:.3f}, window {window:.1f}); '
        f'lane length {tot0:.1f} -> {tot1:.1f} mm; '
        f'median neighbour distance {med0} -> {med1} mm'
        + (f'; not kept: {", ".join(f"{k} {v}" for k, v in sorted(why.items()))}' if why else '')
        + f'  ({_time.time() - t0:.1f} s)')
    return n_packed, n_lanes

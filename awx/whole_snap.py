"""whole_snap.py PLAN.json OUT.json -- a smooth plan that FITS (whole_polish, passing the audit), made octilinear on
the router's grid, one lane at a time.

Each lane in turn, the pairs first (the widest): a grid search in a band round its smooth line -- moves of 0, 45 or
90 degrees between grid points (a pair's 45 at most); cost = length + bends + distance from its smooth line. Its layer
changes only near the plan's own vias, at a grid point where the via clears everything -- a pair's dive as its two
barrels across the way it arrives, as the audit draws them. Hard clearance at the audit's own bars to every static
object of other nets and to every lane and via already placed (a placed pair as its two legs, mitred at its corners
as the audit draws them).

A lane not yet placed keeps its SHARE of every gap: a cell is n's only where its distance to the neighbour's smooth
line exceeds its distance to n's own by the bar -- for its track, and for its via by the via's bar. A pair is priced
at its legs' reach at a 45-degree corner, plus the half step its off-grid legs take. The smooth plan stands its lines
that bar and a grid step apart (whole_polish), so each share holds a grid row, and a lane placed first cannot drift
into the room a later one needs.

At the ends: an off-grid tooth or berth joins the nearest grid point whose join folds neither against its stub nor
against the lane's own way (a pair's: within 45 degrees of its stub), nudged a row off where the nearest would come
under a neighbouring terminal's bar; every move within a track width of an end runs within 90 degrees of its stub
(so the lane's first and last track width cannot fold against it). A lane with no such approach is laid without that
rule and NAMED (OUT's 'folded', which the gate fails); a terminal join is never merged with the grid run beside it.

Then SWEEPS: every lane lifted and laid again against the others' real copper, which leaves room for clean jogs where
the shares made a lane staircase. A lane that cannot be laid in its band FAILS, named (exit 3). The lanes placed later
hug the ones before, so a bundle turns together, mitred."""
import sys, json, math, heapq, time
import numpy as np

import whole_ctx
import braid as bd
import pairs as _pairs
from fab_tiers import min_via_center_distance

plan = json.load(open(sys.argv[1]))
OUT = sys.argv[2]
log = lambda *a: print(*a, flush=True)
ctx, cs = whole_ctx.plan()
cfg = ctx.cfg
g = cfg.grid_step
TW, CL, VR = cfg.track_width, cfg.clearance, cfg.via_size / 2
h2h = getattr(cfg, 'hole_to_hole_clearance', 0.0) or 0.0
prs = getattr(ctx, 'pairs', {}) or {}
HALF = _pairs.pitch(TW) / 2
M = list(plan['lanes'])
HALF_SNAP = HALF / math.cos(math.pi / 8)                  # a pair's legs from its centreline at a 45-degree corner
hw = {n: (HALF_SNAP if n in prs else 0.0) for n in M}
VX = {n: (_pairs.dive_offset(cfg, HALF) if n in prs else 0.0) for n in M}
OFFG = {n: (1 if n in prs else 0) for n in M}             # a pair's legs are off the grid: half a step on its bars
VVB = min_via_center_distance(cfg.via_size, CL, cfg.via_drill, h2h)
EPS = 1e-3                                                 # a diagonal between two grid points, a hair of room
BAND = 2 * bd.LANE_MIN                                     # how far a lane may stray from its smooth line
RVIA = 2 * bd.LANE_MIN                                     # how far a via may move from the plan's
W_BEND = 4 * g                                             # a 45-degree bend, in mm of length
W_DEV = 0.5                                                # per mm of length, per mm from the smooth line
SWEEPS = 2                                                 # clean-up sweeps against the others' real copper
W_VIA = 1.0                                                # per mm a via stands from the plan's
W_SCALE = TW + CL                                          # the audit measures a turn over this much lane
DIRS = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]

# ------------------------------------------------------------------ the plan's lanes: polyline, layers, vias in order
LANE = {}
for n in M:
    pcs = plan['lanes'][n]['pieces']
    pts = [(pcs[0][0], pcs[0][1])] + [(p[2], p[3]) for p in pcs]
    lays = [p[4] for p in pcs]
    vias = [(pcs[i][0], pcs[i][1]) for i in range(1, len(pcs)) if pcs[i][4] != pcs[i - 1][4]]
    LANE[n] = dict(pts=np.array(pts, float), lays=lays, vias=vias, L0=lays[0])
    cl = [((p[0], p[1]), (p[2], p[3]), p[4]) for p in pcs]
    LANE[n]['barrels'] = [b_ for v in vias for b_ in (_pairs.dive_barrels(v, cl, VX[n]) if n in prs else [v])]

# ------------------------------------------------------------------ static copper of other nets
OWN = {n: {ctx.byname[n][0]} | {ctx.byname[leg][0] for leg in prs.get(n, ()) if leg in ctx.byname} for n in M}
STATIC = []          # (kind, layers, net, data): 'rect' cx cy hx hy | 'circ' cx cy r | 'seg' x0 y0 x1 y1 r
for fp in ctx.pcb.footprints.values():
    for pd in fp.pads:
        if pd.pad_type == 'np_thru_hole':
            STATIC.append(('circ', {'F.Cu', 'B.Cu'}, pd.net_id, (pd.global_x, pd.global_y, (pd.drill or 0) / 2)))
            continue
        Ls = {'F.Cu', 'B.Cu'} if (pd.drill and pd.drill > 0) or any(L.startswith('*') for L in pd.layers) \
            else {L for L in pd.layers if L in ('F.Cu', 'B.Cu')}
        if not Ls:
            continue
        if pd.shape == 'circle':
            STATIC.append(('circ', Ls, pd.net_id, (pd.global_x, pd.global_y, pd.size_x / 2)))
        else:
            STATIC.append(('rect', Ls, pd.net_id, (pd.global_x, pd.global_y, pd.size_x / 2, pd.size_y / 2)))
for s in ctx.base_segments:
    STATIC.append(('seg', {s.layer}, s.net_id, (s.start_x, s.start_y, s.end_x, s.end_y, s.width / 2)))
for v in ctx.base_vias:
    STATIC.append(('circ', {'F.Cu', 'B.Cu'}, v.net_id, (v.x, v.y, v.size / 2)))


def bbox(kind, d):
    if kind == 'circ':
        return d[0] - d[2], d[1] - d[2], d[0] + d[2], d[1] + d[2]
    if kind == 'rect':
        return d[0] - d[2], d[1] - d[3], d[0] + d[2], d[1] + d[3]
    return min(d[0], d[2]) - d[4], min(d[1], d[3]) - d[4], max(d[0], d[2]) + d[4], max(d[1], d[3]) + d[4]


def edge_dist(kind, d, X, Y):
    """distance from grid points (X, Y) to the object's copper edge (0 inside)"""
    if kind == 'circ':
        return np.maximum(np.hypot(X - d[0], Y - d[1]) - d[2], 0.0)
    if kind == 'rect':
        dx = np.maximum(np.abs(X - d[0]) - d[2], 0.0)
        dy = np.maximum(np.abs(Y - d[1]) - d[3], 0.0)
        return np.hypot(dx, dy)
    x0, y0, x1, y1, r = d
    vx, vy = x1 - x0, y1 - y0
    L2 = vx * vx + vy * vy
    t = np.clip(((X - x0) * vx + (Y - y0) * vy) / L2, 0.0, 1.0) if L2 > 1e-18 else np.zeros_like(X)
    return np.maximum(np.hypot(X - (x0 + t * vx), Y - (y0 + t * vy)) - r, 0.0)


def seg_pts_dist(P, Q, X, Y):
    vx, vy = Q[0] - P[0], Q[1] - P[1]
    L2 = vx * vx + vy * vy
    t = np.clip(((X - P[0]) * vx + (Y - P[1]) * vy) / L2, 0.0, 1.0) if L2 > 1e-18 else np.zeros_like(X)
    return np.hypot(X - (P[0] + t * vx), Y - (P[1] + t * vy))


# ------------------------------------------------------------------ the stub's last segment at each end
def last_dir(net, pt, layer):
    return whole_ctx.stub_dir(ctx, net, pt, layer)


def end_dirs(n):
    """(out of the tooth, into the berth) as unit vectors, from the stubs' last segments"""
    L0, L1 = LANE[n]['lays'][0], LANE[n]['lays'][-1]
    if n in prs:
        (sp_, _sn), (tp_, _tn) = ctx.pair_ends[n]
        leg = prs[n][0]
        a, b = last_dir(leg, sp_, L0), last_dir(leg, tp_, L1)
    else:
        a, b = last_dir(n, ctx.ends[n][0], L0), last_dir(n, ctx.ends[n][1], L1)
    P = LANE[n]['pts']
    a = a or tuple((P[1] - P[0]) / np.linalg.norm(P[1] - P[0]))
    b = (-b[0], -b[1]) if b else tuple((P[-1] - P[-2]) / np.linalg.norm(P[-1] - P[-2]))
    return a, b


def dir_index(v):
    a = math.atan2(v[1], v[0])
    return int(round(a / (math.pi / 4))) % 8


def term_cell(p, e, way, start, n, layer, free):
    """the grid point an off-grid terminal joins: the nearest whose join folds neither against the stub's way e nor
    against the lane's own way (a pair's: within 45 degrees of e -- it turns no more at a time), where an on-grid track fits (free: clear of static copper and placed lanes),
    nudged a row off where the nearest would come under the bar of a neighbouring terminal it was not already under
    (rounded toward each other, SDQ15 and SDQ13 left teeth 0.254 apart at 0.242 -- the plan's doing, not the
    fanout's). Only a neighbour within its bar counts: SDQS1's tooth 0.44 away is no reason to move"""
    nbr = [(tuple(LANE[m]['pts'][k_]), LANE[m]['lays'][k_], m) for m in M if m != n for k_ in (0, -1)]
    # the bar two JOINS (both off the grid) keep: a lane's, plus half a step for each
    nbr = [(q_, TW + CL + hw[n] + hw[m] + g) for q_, L_, m in nbr if L_ == layer]
    best = None
    for i in range(int(math.floor(p[0] / g)) - 2, int(math.ceil(p[0] / g)) + 3):
        for j in range(int(math.floor(p[1] / g)) - 2, int(math.ceil(p[1] / g)) + 3):
            v = np.array([i * g - p[0], j * g - p[1]])
            v = v if start else -v
            L = float(np.hypot(*v))
            out45 = L > 1e-9 and float(v @ np.array(e)) < L * math.cos(math.pi / 4) - 1e-9
            if out45 and (n in prs or float(v @ np.array(e)) < -1e-9):
                continue
            fold = L > 1e-9 and float(v @ np.array(way)) < -1e-9
            closer = any(math.hypot(i * g - q_[0], j * g - q_[1]) < min(bar_, math.hypot(p[0] - q_[0], p[1] - q_[1])) - 1e-9
                         for q_, bar_ in nbr)
            key_ = (not free(i, j), closer, fold, L)
            if best is None or key_ < best[0]:
                best = (key_, i, j)
    return best[1], best[2]


# ------------------------------------------------------------------ what is placed so far
PLACED = []          # (n, layer, P, Q)  lane centreline segments
LEGS = []            # (n, layer, P, Q)  a placed PAIR's two legs as the audit draws them (mitred at its corners)
PVIAS = []           # (n, x, y): every placed via's barrels (a pair's two)


def lane_bar(n, m):
    return TW + CL + hw[n] + hw[m] + g / 2 * (OFFG[n] + OFFG[m])


def build(n):
    """lane n's search window: band cells, their distance from the smooth line and arc position, the forbidden
    cells per layer for its centre, the cells where its via clears, the soft price of lanes not yet placed"""
    P = LANE[n]['pts']
    x0, y0 = P.min(axis=0) - BAND - g
    x1, y1 = P.max(axis=0) + BAND + g
    i0, j0 = int(math.floor(x0 / g)), int(math.floor(y0 / g))
    i1, j1 = int(math.ceil(x1 / g)), int(math.ceil(y1 / g))
    xs, ys = np.arange(i0, i1 + 1) * g, np.arange(j0, j1 + 1) * g
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    # distance to the smooth line and arc position of the nearest point
    dist = np.full(X.shape, np.inf)
    arc = np.zeros(X.shape)
    s0 = 0.0
    for a_, b_ in zip(P, P[1:]):
        d_ = seg_pts_dist(a_, b_, X, Y)
        L_ = float(np.hypot(*(b_ - a_)))
        t_ = np.clip(((X - a_[0]) * (b_[0] - a_[0]) + (Y - a_[1]) * (b_[1] - a_[1])) / max(L_ * L_, 1e-18), 0, 1)
        better = d_ < dist
        dist[better] = d_[better]
        arc[better] = s0 + t_[better] * L_
        s0 += L_
    band = dist <= BAND
    bad = {L: np.zeros(X.shape, bool) for L in ('F.Cu', 'B.Cu')}
    lo_x, lo_y = xs[0], ys[0]

    def window(bb, rch, ox=0.0, oy=0.0):
        """index ranges of the cells whose point (x + ox, y + oy) lies within rch of the box bb, or None"""
        a0 = max(0, int(math.floor((bb[0] - rch - ox - lo_x) / g))); a1 = min(len(xs) - 1, int(math.ceil((bb[2] + rch - ox - lo_x) / g)))
        b0 = max(0, int(math.floor((bb[1] - rch - oy - lo_y) / g))); b1 = min(len(ys) - 1, int(math.ceil((bb[3] + rch - oy - lo_y) / g)))
        return None if a1 < a0 or b1 < b0 else (slice(a0, a1 + 1), slice(b0, b1 + 1))

    def mark(arr, kind, d, bar, dfun=None, ox=0.0, oy=0.0):
        w = window(bbox(kind, d) if kind else d, bar, ox, oy)
        if w is None:
            return
        Xs, Ys = X[w] + ox, Y[w] + oy
        dd = dfun(Xs, Ys) if dfun else edge_dist(kind, d, Xs, Ys)
        arr[w] |= dd < bar + EPS

    def share(arr, bar, dfun, bb, ox=0.0, oy=0.0):
        """the cells whose point (x + ox, y + oy) stands nearer an unplaced lane's smooth copper than bar more than
        the cell stands from n's own line: that lane's share"""
        w = window(bb, bar + BAND, ox, oy)
        if w is not None:
            arr[w] |= (dfun(X[w] + ox, Y[w] + oy) - dist[w]) < bar - EPS

    seg_d = lambda p_, q_: (lambda Xs, Ys: seg_pts_dist(p_, q_, Xs, Ys))
    pt_d = lambda x_, y_: (lambda Xs, Ys: np.hypot(Xs - x_, Ys - y_))
    box = lambda p_, q_: (min(p_[0], q_[0]), min(p_[1], q_[1]), max(p_[0], q_[0]), max(p_[1], q_[1]))
    lane_st = TW / 2 + CL + hw[n] + g / 2 * OFFG[n]
    for kind, Ls, net, d in STATIC:
        if net in OWN[n]:
            continue
        for L in Ls:
            if L in bad:
                mark(bad[L], kind, d, lane_st)
    # placed copper: a single lane's centreline; a pair's two LEGS (its corners' mitres are wider than its pitch); every
    # placed via as its barrels (a pair's two, across the way it arrived)
    placed = [(e_[0], e_[1], e_[2], e_[3], hw[e_[0]], OFFG[e_[0]]) for e_ in PLACED if e_[0] not in prs] + \
             [(e_[0], e_[1], e_[2], e_[3], 0.0, 1) for e_ in LEGS]
    for (m, L, p_, q_, hm, om) in placed:
        mark(bad[L], None, box(p_, q_), TW + CL + hw[n] + hm + g / 2 * (OFFG[n] + om), seg_d(p_, q_))
    for (m, bx_, by_) in PVIAS:
        for L in bad:
            mark(bad[L], None, (bx_, by_, bx_, by_), VR + CL + TW / 2 + hw[n] + g / 2 * (OFFG[n] + OFFG[m]), pt_d(bx_, by_))
    # the lanes not yet placed keep their SHARE of every gap: a cell is n's only where its distance to the neighbour's
    # smooth line exceeds its distance to n's own by the bar (the smooth plan stands its lines a bar and a grid step
    # apart, so each share holds at least one grid row). Left to a soft price, SDQ6 and SDQ7 drifted to 0.225 of
    # SDQ3's line and left it a channel only its exact slanted line fits
    unplaced = [m for m in M if m != n and m not in res['lanes']]
    for m in unplaced:
        Pm, Lm = LANE[m]['pts'], LANE[m]['lays']
        for (p_, q_), L in zip(zip(Pm, Pm[1:]), Lm):
            share(bad[L], lane_bar(n, m), seg_d(p_, q_), box(p_, q_))
        for (bx_, by_) in LANE[m]['barrels']:
            for L in bad:
                share(bad[L], VR + CL + TW / 2 + hw[n] + g / 2 * (OFFG[n] + OFFG[m]), pt_d(bx_, by_), (bx_, by_, bx_, by_))

    def vfield(ox, oy):
        """the cells where a barrel of n's via at (x + ox, y + oy) does not clear: other nets' static copper, placed
        copper and barrels, and the unplaced lanes' shares (a via is wider than its track, and one slid along the line
        out of the room the plan gave it takes the neighbour's row: SDQ14's via, 0.053 on, closed SDQ12)"""
        vb = np.zeros(X.shape, bool)
        for kind, Ls, net, d in STATIC:
            if net not in OWN[n]:
                mark(vb, kind, d, VR + CL + g / 2 * OFFG[n], ox=ox, oy=oy)
        for (m, L, p_, q_, hm, om) in placed:
            mark(vb, None, box(p_, q_), VR + CL + TW / 2 + hm + g / 2 * (OFFG[n] + om), seg_d(p_, q_), ox, oy)
        for (m, bx_, by_) in PVIAS:
            mark(vb, None, (bx_, by_, bx_, by_), VVB + g / 2 * (OFFG[n] + OFFG[m]), pt_d(bx_, by_), ox, oy)
        for m in unplaced:
            Pm = LANE[m]['pts']
            for p_, q_ in zip(Pm, Pm[1:]):
                share(vb, VR + CL + TW / 2 + hw[m] + g / 2 * (OFFG[n] + OFFG[m]), seg_d(p_, q_), box(p_, q_), ox, oy)
            for (bx_, by_) in LANE[m]['barrels']:
                share(vb, VVB + g / 2 * (OFFG[n] + OFFG[m]), pt_d(bx_, by_), (bx_, by_, bx_, by_), ox, oy)
        return vb
    if n in prs:
        # a pair dives as two barrels across the way it ARRIVES (as the audit draws them): one field per axis
        vbad = []
        for a in range(4):
            ux, uy = -DIRS[a][1], DIRS[a][0]
            ul = math.hypot(ux, uy)
            ox, oy = VX[n] * ux / ul, VX[n] * uy / ul
            vbad.append(vfield(ox, oy) | vfield(-ox, -oy))
    else:
        vbad = [vfield(0.0, 0.0)] * 4
    return dict(i0=i0, j0=j0, xs=xs, ys=ys, dist=dist, arc=arc, band=band, bad=bad, vbad=vbad)


def via_arcs(n):
    """the arc position of each of the plan's vias along its smooth line"""
    P = LANE[n]['pts']
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(P, axis=0).T))])
    out = []
    for vp in LANE[n]['vias']:
        k = int(np.argmin(np.hypot(P[:, 0] - vp[0], P[:, 1] - vp[1])))
        out.append(float(s[k]))
    return out, float(s[-1])


def route(n, strict=True):
    """A* over (cell, direction, vias taken): the lane's octilinear grid path, or None. strict: every move within a
    track width of an end runs within 90 degrees of its stub's own way"""
    W = build(n)
    i0, j0, band, bad, vbad, dist, arc = W['i0'], W['j0'], W['band'], W['bad'], W['vbad'], W['dist'], W['arc']
    NI, NJ = band.shape
    a_out, a_in = end_dirs(n)
    P = LANE[n]['pts']
    L_s, L_e = LANE[n]['lays'][0], LANE[n]['lays'][-1]
    free_ = lambda L_: (lambda i, j: 0 <= i - i0 < NI and 0 <= j - j0 < NJ and not bad[L_][i - i0, j - j0])
    s_ = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(P, axis=0).T))])
    w0 = P[min(int(np.searchsorted(s_, W_SCALE)), len(P) - 1)] - P[0]            # the lane's own way at each end,
    w1 = P[-1] - P[max(int(np.searchsorted(s_, s_[-1] - W_SCALE)) - 1, 0)]      # over the audit's scale for a turn
    si, sj = term_cell(tuple(P[0]), a_out, w0, True, n, L_s, free_(L_s))
    ei, ej = term_cell(tuple(P[-1]), a_in, w1, False, n, L_e, free_(L_e))
    d0, dN = dir_index(a_out), dir_index(a_in)
    lays = [LANE[n]['L0']]
    for _v in LANE[n]['vias']:
        lays.append('B.Cu' if lays[-1] == 'F.Cu' else 'F.Cu')
    K = len(LANE[n]['vias'])
    varc, total = via_arcs(n)
    gate = [(-math.inf if k == 0 else varc[k - 1] - 3 * bd.LANE_MIN,
             math.inf if k == K else varc[k] + 3 * bd.LANE_MIN) for k in range(K + 1)]
    vpts = LANE[n]['vias']
    no90 = n in prs
    ok = lambda i, j: 0 <= i < NI and 0 <= j < NJ
    S = (si - i0, sj - j0)
    E = (ei - i0, ej - j0)
    if not (ok(*S) and ok(*E)):
        return None, 'terminal outside the window'
    jS = ((S[0] + i0) * g - P[0][0], (S[1] + j0) * g - P[0][1])           # the join out of the tooth
    jE = (P[-1][0] - (E[0] + i0) * g, P[-1][1] - (E[1] + j0) * g)         # the join into the berth
    if math.hypot(*jS) < 1e-9:
        jS = a_out
    if math.hypot(*jE) < 1e-9:
        jE = a_in
    h = lambda i, j: math.hypot(i - E[0], j - E[1]) * g
    start = (S[0], S[1], d0, 0)
    best = {start: 0.0}
    prev = {}
    pq = [(h(*S), 0.0, start)]
    goal = None
    npop = 0
    while pq:
        f_, c_, st = heapq.heappop(pq)
        if best.get(st, math.inf) < c_ - 1e-12:
            continue
        npop += 1
        i, j, d, k = st
        if (i, j) == E and k == K:
            goal = st
            break
        L = lays[k]
        # a via here: the next layer, near the plan's via, where a via clears
        if k < K and not vbad[d % 4][i, j]:
            x_, y_ = (i + i0) * g, (j + j0) * g
            dv = math.hypot(x_ - vpts[k][0], y_ - vpts[k][1])
            if dv <= RVIA and not bad[lays[k + 1]][i, j]:
                nst = (i, j, d, k + 1)
                nc = c_ + W_VIA * dv
                if nc < best.get(nst, math.inf) - 1e-12:
                    best[nst] = nc; prev[nst] = st
                    heapq.heappush(pq, (nc + h(i, j), nc, nst))
        for nd in range(8):
            bend = min(abs(nd - d), 8 - abs(nd - d))
            if bend >= 3 or (no90 and bend >= 2):
                continue
            di, dj = DIRS[nd]
            ni, nj = i + di, j + dj
            if not ok(ni, nj) or not band[ni, nj]:
                continue
            # no fold at either end: the first move within 90 degrees of the join out of the tooth, the last within
            # 90 of the join into the berth (the audit folds a turn over 100)
            if (i, j) == S and k == 0 and DIRS[nd][0] * jS[0] + DIRS[nd][1] * jS[1] < -1e-9:
                continue
            if (ni, nj) == E and k == K and DIRS[nd][0] * jE[0] + DIRS[nd][1] * jE[1] < -1e-9:
                continue
            # ...and every move within a track width of either end runs within 90 degrees of its stub's own way (a sum
            # of such moves cannot fold, so neither can the lane's first or last track width: SDQ4's stub runs 14
            # degrees off north, and a last move east read as 90 degrees against the router direction)
            if strict and k == 0 and math.hypot(i - S[0], j - S[1]) * g <= TW and DIRS[nd][0] * a_out[0] + DIRS[nd][1] * a_out[1] < -1e-9:
                continue
            if strict and k == K and math.hypot(ni - E[0], nj - E[1]) * g <= TW and DIRS[nd][0] * a_in[0] + DIRS[nd][1] * a_in[1] < -1e-9:
                continue
            if (ni, nj) != E and bad[L][ni, nj]:
                continue
            a_ = arc[ni, nj]
            if not (gate[k][0] <= a_ <= gate[k][1]):
                continue
            step = g * (math.sqrt(2) if di and dj else 1.0)
            nc = c_ + step * (1 + W_DEV * dist[ni, nj]) + W_BEND * bend
            if (ni, nj) == E:
                eb = min(abs(nd - dN), 8 - abs(nd - dN))
                if no90 and eb >= 2:
                    continue
                nc += W_BEND * eb
            nst = (ni, nj, nd, k)
            if nc < best.get(nst, math.inf) - 1e-12:
                best[nst] = nc; prev[nst] = st
                heapq.heappush(pq, (nc + h(ni, nj), nc, nst))
    if goal is None:
        return None, f'no path in its band ({npop} states searched)'
    path = [goal]
    while path[-1] in prev:
        path.append(prev[path[-1]])
    path.reverse()
    pts, vias = [], []
    for (i, j, d, k) in path:
        xy = ((i + i0) * g, (j + j0) * g)
        if pts and pts[-1][0] == xy and pts[-1][1] != k:
            vias.append(xy)
        pts.append((xy, k))
    pieces = [(tuple(P[0]), pts[0][0], lays[0])]
    for (a_, ka), (b_, kb) in zip(pts, pts[1:]):
        if a_ != b_:
            pieces.append((a_, b_, lays[kb]))
    pieces.append((pts[-1][0], tuple(P[-1]), lays[K]))
    merged = []
    for q_, (a_, b_, L) in enumerate(pieces):
        if math.hypot(b_[0] - a_[0], b_[1] - a_[1]) < 1e-12:
            continue
        # a terminal JOIN (the first and last piece: the only ones off the grid) is never merged with the grid run
        # beside it, or the audit grades the whole run off the grid (SCAS's diagonal tooth join took 1.6 mm with it)
        if merged and merged[-1][2] == L and q_ < len(pieces) - 1 and len(merged) > 1:
            p0, p1, _ = merged[-1]
            cr = (p1[0] - p0[0]) * (b_[1] - p0[1]) - (p1[1] - p0[1]) * (b_[0] - p0[0])
            dot = (p1[0] - p0[0]) * (b_[0] - p1[0]) + (p1[1] - p0[1]) * (b_[1] - p1[1])
            if abs(cr) < 1e-12 and dot > 0 and p1 == a_:
                merged[-1] = (p0, b_, L)
                continue
        merged.append((a_, b_, L))
    return (merged, vias), f'{npop} states'


# ------------------------------------------------------------------ one lane at a time
order = [n for n in M if n in prs] + [n for n in M if n not in prs]
DONE = set()
res = {'lanes': {}, 'vias': [], 'conflicts': [], 'rules': {'grid': g, 'track': TW, 'clear': CL, 'lane_min': bd.LANE_MIN}}
t_all = time.time()
def place(n, out):
    pieces, vias = out
    for (a_, b_, L) in pieces:
        PLACED.append((n, L, a_, b_))
    if n in prs:
        for lg in bd._pair_legs([(a_, b_, L) for (a_, b_, L) in pieces], HALF):
            (a_, b_, L) = lg[:3]
            LEGS.append((n, L, tuple(map(float, a_)), tuple(map(float, b_))))
    for v in vias:
        for b_ in (_pairs.dive_barrels(v, pieces, VX[n]) if n in prs else [v]):
            PVIAS.append((n, b_[0], b_[1]))
    res['lanes'][n] = {'xy': [list(pieces[0][0])] + [list(p[1]) for p in pieces],
                       'pieces': [[a_[0], a_[1], b_[0], b_[1], L] for (a_, b_, L) in pieces], 'vias': [list(v) for v in vias]}


def lift(m):
    """take lane m's copper off the board (a sweep lays it again)"""
    PLACED[:] = [e_ for e_ in PLACED if e_[0] != m]
    LEGS[:] = [e_ for e_ in LEGS if e_[0] != m]
    PVIAS[:] = [e_ for e_ in PVIAS if e_[0] != m]
    res['lanes'].pop(m, None)


failed = {}
folded = {}
for n in order:
    t0 = time.time()
    out, why = route(n)
    if out is None:
        # no approach to a stub within 90 degrees of it (SCAS: its berth between a capacitor's pads, reachable at this
        # clearance only descending into it): laid without that rule, and NAMED -- the lint reports the fold
        out, why2 = route(n, strict=False)
        if out is not None:
            folded[n] = why
            why = why2 + ' (NO approach within 90 degrees of a stub: laid folding, named)'
    DONE.add(n)
    if out is None:
        failed[n] = why
        log(f'  {n:7s} FAILED: {why}')
        continue
    place(n, out)
    log(f'  {n:7s} {len(out[0]):3d} pieces {len(out[1])} via(s)  {why}  {time.time() - t0:.1f} s')
# SWEEPS: every lane lifted and laid again against the others' REAL copper (no shares left): where a gap's share
# made it staircase, the neighbours' actual lines usually leave room for one clean jog. Its old path is still there,
# so a sweep never fails a lane.
for sw in range(SWEEPS):
    nb0 = sum(len(L_['pieces']) for L_ in res['lanes'].values())
    for n in order:
        if n not in res['lanes'] or n in failed:
            continue
        keep = res['lanes'][n]
        old = ([(tuple(p_[0:2]), tuple(p_[2:4]), p_[4]) for p_ in keep['pieces']], [tuple(v) for v in keep['vias']])
        lift(n)
        out, why = route(n, strict=n not in folded)
        place(n, out if out is not None else old)
    nb1 = sum(len(L_['pieces']) for L_ in res['lanes'].values())
    log(f'  sweep {sw + 1}: pieces {nb0} -> {nb1}')
    if nb1 >= nb0:
        break
for n, why in failed.items():
    res['conflicts'].append({'frame': 'snap', 'xy': list(map(float, LANE[n]['pts'][0])), 'lanes': [n],
                             'charged': [{'kind': 'snap', 'text': f'{n}: {why}', 'short': 1.0, 'lanes': [n],
                                          'xy': list(map(float, LANE[n]['pts'][0]))}], 'hard': []})
res['vias'] = [[n, v[0], v[1]] for n, L_ in res['lanes'].items() for v in L_['vias']]
res['tdir'] = {n: [list(end_dirs(n)[0]), [-v for v in end_dirs(n)[1]], list(LANE[n]['pts'][0]), list(LANE[n]['pts'][-1])]
               for n in M}
res['failed'] = bool(res['conflicts'])
res['folded'] = sorted(folded)
json.dump(res, open(OUT, 'w'))
log(f'snap: {len(res["lanes"])}/{len(M)} lanes laid, {len(res["vias"])} vias, {time.time() - t_all:.0f} s -> {OUT}')
if folded:
    log(f'snap: {len(folded)} lane(s) with no approach within 90 degrees of a stub, laid folding: {", ".join(sorted(folded))}')
if res['conflicts']:
    log(f'SNAP FAILED: {len(res["conflicts"])} lane(s) could not be laid: {", ".join(c["lanes"][0] for c in res["conflicts"])}')
    sys.exit(3)

"""whole_polish.py GEO.json OUT.json -- the geometry LP's lanes made to meet the AUDIT's own measures, in board xy.

The column LP (whole_geo) measures in its spine frames, which are not Euclidean off a curved spine and linearise slopes
with tangent cuts, so its lanes come out a few um short of the audit's bars in dozens of places. This polish works on
the lanes' board polylines directly: every near-violation of the audit's rules becomes a linear constraint on small
vertex moves (the normal taken from the current geometry), a small LP over the vertices near violations finds the
least movement that meets them all, and that repeats (re-linearised) until the audit's measures hold or no move helps.

  pitch    same-layer lanes: track + clearance + grid (+ a pair's legs' reach, below)
  dives    a via (a pair: its two barrels, pairs.dive_offset across the arriving direction) vs other lanes' lines on
           either layer, other lanes' vias, static copper of other nets
  static   lane vs other nets' pads / stubs / vias on its layer: track/2 + clearance + grid/2
  shape    a moved vertex keeps its turn under 90 degrees at the lane's scale; the AUDIT's own shape findings (a
           turn over 100 degrees at a vertex or over the lane's scale, a notch) straightened before the rounds and
           after them, and the rounds run again
  joins    a single's last track + clearance at either end inside the arc of router directions that each keep
           within 90 degrees of its stub: the moves the snap may make there (no fold where lane and stub meet)

Every bar is the snap's (whole_snap) plus a grid step, so the gaps the snap splits each hold a grid row: a pair is
priced at its legs' reach at the snap's 45-degree corners (HALF_SNAP) plus the half step its off-grid legs and
barrels take. A pair's end stretch -- the pair router's first setback from its tips -- is laid straight within 45
degrees of its stub and held (pair_approaches).

Lane ends (tooth, berth) never move. What a constraint still needs after the last round (its slack) is a rule the
geometry cannot meet here without breaking another: reported, for the solve -- and a lane held off an island's side
(a static clearance, or a stub join it cannot meet) is written as a side FLIP for the next geometry."""
import sys, json, math, collections
import numpy as np
from scipy.optimize import linprog

import whole_ctx
import plan_audit as pa
import pairs as _pairs
from fab_tiers import min_via_center_distance

geo = json.load(open(sys.argv[1]))
OUT = sys.argv[2]
ROUNDS = 12
CYCLES = 3                         # rounds, then the shape measures; at most this many times
ctx, cs = whole_ctx.plan()
cfg = ctx.cfg
TRUST = 2 * cfg.grid_step          # the most a vertex moves in one round
TW, CL, VR, g2 = cfg.track_width, cfg.clearance, cfg.via_size / 2, cfg.grid_step / 2
h2h = getattr(cfg, 'hole_to_hole_clearance', 0.0) or 0.0
prs = getattr(ctx, 'pairs', {}) or {}
HALF = _pairs.pitch(TW) / 2
OFF = _pairs.dive_offset(cfg, HALF) if prs else 0.0
# the audit's bars (plan_audit) for items OFF the grid, as every piece of a smooth plan is: planned vs planned a whole
# grid step, planned vs static half
BLOCK = TW + CL + 2 * g2
NEED_VL = VR + CL + TW / 2 + 2 * g2
NEED_VV = min_via_center_distance(cfg.via_size, CL, cfg.via_drill, h2h) + 2 * g2
NEED_ST = TW / 2 + CL + g2
NEED_VST = VR + CL + g2
MARGIN = cfg.grid_step           # constraints within this of their bar are kept in the LP (so a move breaks none)
EPS = 1e-4                       # met with this much to spare (the audit compares at 1e-6)
# the snap (whole_snap) lays a pair's centreline on the grid with 45-degree corners, where its legs stand HALF_SNAP
# from it, and its legs off the grid: a pair's bars here carry both, so every gap the snap splits keeps a grid row
# of slack (a single's already do: BLOCK is its on-grid bar plus a step)
HALF_SNAP = HALF / math.cos(math.pi / 8)
hw = {n: (HALF_SNAP + g2 if n in prs else 0.0) for n in geo['lanes']}
LNAME = {'F': 'F.Cu', 'B': 'B.Cu'}
log = lambda *a: print(*a, flush=True)


# ------------------------------------------------------------------ the lanes as vertices + per-segment layers
def densify(pts, lays, dmax):
    X, Ls = [pts[0]], []
    for (p, q), L in zip(zip(pts, pts[1:]), lays):
        n = max(1, int(math.ceil(math.hypot(q[0] - p[0], q[1] - p[1]) / dmax)))
        for i in range(1, n + 1):
            X.append((p[0] + (q[0] - p[0]) * i / n, p[1] + (q[1] - p[1]) * i / n)); Ls.append(L)
    return X, Ls


LANES = {}
for n, v in geo['lanes'].items():
    pcs = v['pieces']
    pts = [(pcs[0][0], pcs[0][1])] + [(pc[2], pc[3]) for pc in pcs]
    X, Ls = densify(pts, [pc[4] for pc in pcs], 4 * cfg.grid_step)
    LANES[n] = dict(X=np.array(X, float), L=Ls)


def via_idx(n):
    """indices of the vertices where lane n changes layer (its vias)"""
    Ls = LANES[n]['L']
    return [i for i in range(1, len(Ls)) if Ls[i] != Ls[i - 1]]


PIN = collections.defaultdict(lambda: [0, 0])   # lane -> vertices held after its first / before its last (a pair's approach)
OCT = [np.array(v, float) / math.hypot(*v) for v in ((1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1))]


def octi(u):
    """the router direction (0, 45, 90 ... degrees) nearest u"""
    return max(OCT, key=lambda o: float(o @ np.asarray(u, float)))


def stub_ways(n, router=True):
    """(the way lane n leaves its tooth, the way it arrives at its berth) from its stubs' last segments -- each the
    router direction nearest it, or (router False) the segment's own -- None where the stub ends in a via (the lane
    lands on it and continues no line)"""
    L0, L1 = LANES[n]['L'][0], LANES[n]['L'][-1]
    if n in prs:
        (sp_, _sn), (tp_, _tn) = ctx.pair_ends[n]
        a, b = whole_ctx.stub_dir(ctx, prs[n][0], sp_, L0), whole_ctx.stub_dir(ctx, prs[n][0], tp_, L1)
    else:
        a, b = whole_ctx.stub_dir(ctx, n, ctx.ends[n][0], L0), whole_ctx.stub_dir(ctx, n, ctx.ends[n][1], L1)
    w = octi if router else (lambda u: np.asarray(u, float))
    return (w(a) if a else None), (-w(b) if b else None)


def fixed(n, i):
    return i <= PIN[n][0] or i >= len(LANES[n]['X']) - 1 - PIN[n][1]


def turn_deg(X, i):
    d1, d2 = X[i] - X[i - 1], X[i + 1] - X[i]
    l1, l2 = np.linalg.norm(d1), np.linalg.norm(d2)
    if l1 < 1e-9 or l2 < 1e-9:
        return 0.0
    return math.degrees(math.atan2(d1[0] * d2[1] - d1[1] * d2[0], d1 @ d2))


W_SCALE = TW + CL                  # a turn is measured at the lane's own scale (the audit's shape check does the same)


def arclen(X):
    return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(X, axis=0), axis=1))])


def scale_window(s, i):
    """the vertices W_SCALE back and ahead of vertex i along the lane (the nearest at or beyond that reach)"""
    j0 = int(np.searchsorted(s, s[i] - W_SCALE, side='right')) - 1
    j1 = int(np.searchsorted(s, s[i] + W_SCALE, side='left'))
    return max(0, min(j0, i - 1)), min(len(s) - 1, max(j1, i + 1))


NOTCH = TW + cfg.grid_step          # the audit's: a step or a dip shorter than a track and a grid step


def shape_faults(n):
    """the audit's own shape findings on lane n (plan_audit.check_shape, on each same-layer stretch): a turn over 100
    degrees at a vertex or measured over W_SCALE, a notch -- as (first, last) vertex index of each"""
    X, Ls = LANES[n]['X'], LANES[n]['L']
    out = []
    a = 0
    while a < len(Ls):
        b = a
        while b + 1 < len(Ls) and Ls[b + 1] == Ls[a]:
            b += 1
        idx = {tuple(X[i]): i for i in range(a, b + 2)}
        pts = [tuple(X[i]) for i in range(a, b + 2)]
        tr = pa._turns(pts)
        out += [(idx[v], idx[v]) for v, t_, _l in tr if abs(t_) > 100]
        out += [(idx[v], idx[v]) for v, t_ in pa._scale_turns(pts, W_SCALE) if abs(t_) > 100]
        out += [(idx[v1], idx[v2]) for (v1, t1, l1), (v2, t2, _l) in zip(tr, tr[1:])
                if abs(t1) > 60 and abs(t2) > 60 and t1 * t2 < 0 and l1 < NOTCH]
        a = b + 1
    return sorted(set(out))


def drop_faults():
    """each shape fault straightened: the vertices within W_SCALE of it replaced by the straight line across -- never
    across a layer change, an end or a held approach. Repeatedly; the clearance rounds put the room back. What cannot
    be straightened is left for the audit to name."""
    dropped = collections.Counter()
    for n, ln in LANES.items():
        for _ in range(50):
            X, Ls = ln['X'], ln['L']
            s = arclen(X)
            vs = via_idx(n)
            hit = None
            for i0, i1 in shape_faults(n):
                j0 = scale_window(s, i0)[0]
                j1 = scale_window(s, i1)[1]
                j0 = max([j0, PIN[n][0]] + [v for v in vs if v <= i0])
                j1 = min([j1, len(X) - 1 - PIN[n][1]] + [v for v in vs if v >= i1])
                if j1 - j0 >= 2:
                    hit = (j0, j1)
                    break
            if hit is None:
                break
            j0, j1 = hit
            ln['X'] = np.concatenate([X[:j0 + 1], X[j1:]]); ln['L'] = Ls[:j0] + Ls[j1 - 1:]
            dropped[n] += j1 - j0 - 1
    return dropped


# ------------------------------------------------------------------ static copper of other nets, per layer
STATIC = []          # (kind, layers, net, data): pads ('rect' cx cy hx hy | 'circ' cx cy r), segs (x0 y0 x1 y1 hw), vias (x y r)
for fp in ctx.pcb.footprints.values():
    for pd in fp.pads:
        if pd.pad_type == 'np_thru_hole':
            STATIC.append(('circ', {'F.Cu', 'B.Cu'}, pd.net_id, (pd.global_x, pd.global_y, (pd.drill or 0) / 2), f'hole {fp.reference}.{pd.pad_number}'))
            continue
        Ls = {'F.Cu', 'B.Cu'} if (pd.drill and pd.drill > 0) or any(L.startswith('*') for L in pd.layers) \
            else {L for L in pd.layers if L in ('F.Cu', 'B.Cu')}
        if not Ls:
            continue
        if pd.shape == 'circle':
            STATIC.append(('circ', Ls, pd.net_id, (pd.global_x, pd.global_y, pd.size_x / 2), f'pad {fp.reference}.{pd.pad_number}'))
        else:
            STATIC.append(('rect', Ls, pd.net_id, (pd.global_x, pd.global_y, pd.size_x / 2, pd.size_y / 2), f'pad {fp.reference}.{pd.pad_number}'))
for s in ctx.base_segments:
    STATIC.append(('seg', {s.layer}, s.net_id, (s.start_x, s.start_y, s.end_x, s.end_y, s.width / 2), 'copper'))
for v in ctx.base_vias:
    STATIC.append(('circ', {'F.Cu', 'B.Cu'}, v.net_id, (v.x, v.y, v.size / 2), 'via'))
OWN = {n: {ctx.byname[n][0]} | {ctx.byname[leg][0] for leg in prs.get(n, ()) if leg in ctx.byname} for n in LANES}
# static objects binned by bounding box; a query looks SREACH round its point: the widest bar to static copper
SCELL = 2 * _pairs.pitch(TW)
SREACH = max(NEED_VST + OFF + g2, NEED_ST + HALF_SNAP + g2, BLOCK + 2 * (HALF_SNAP + g2)) + MARGIN
SGRID = collections.defaultdict(list)
for k_, (kind, Ls, net, d, lab) in enumerate(STATIC):
    if kind == 'circ':
        lo, hi = (d[0] - d[2], d[1] - d[2]), (d[0] + d[2], d[1] + d[2])
    elif kind == 'rect':
        lo, hi = (d[0] - d[2], d[1] - d[3]), (d[0] + d[2], d[1] + d[3])
    else:
        lo, hi = (min(d[0], d[2]) - d[4], min(d[1], d[3]) - d[4]), (max(d[0], d[2]) + d[4], max(d[1], d[3]) + d[4])
    for cx in range(int(math.floor(lo[0] / SCELL)), int(math.floor(hi[0] / SCELL)) + 1):
        for cy in range(int(math.floor(lo[1] / SCELL)), int(math.floor(hi[1] / SCELL)) + 1):
            SGRID[(cx, cy)].append(k_)


def static_near(P, L, own):
    """(distance to the object's edge, the object's nearest point) for every static object of other nets on L near P"""
    out = []
    ks = set()
    for cx in range(int(math.floor((P[0] - SREACH) / SCELL)), int(math.floor((P[0] + SREACH) / SCELL)) + 1):
        for cy in range(int(math.floor((P[1] - SREACH) / SCELL)), int(math.floor((P[1] + SREACH) / SCELL)) + 1):
            ks.update(SGRID.get((cx, cy), ()))
    for k_ in ks:
        kind, Ls, net, d, lab = STATIC[k_]
        if L not in Ls or net in own:
            continue
        if kind == 'circ':
            cx, cy, r = d
            if abs(P[0] - cx) > 1.5 or abs(P[1] - cy) > 1.5:
                continue
            v = P - np.array([cx, cy]); dd = np.linalg.norm(v)
            if dd < 1e-9:
                continue
            out.append((dd - r, np.array([cx, cy]) + v / dd * r, lab))
        elif kind == 'rect':
            cx, cy, hx, hy = d
            if abs(P[0] - cx) > hx + 1.5 or abs(P[1] - cy) > hy + 1.5:
                continue
            q = np.array([min(max(P[0], cx - hx), cx + hx), min(max(P[1], cy - hy), cy + hy)])
            dd = np.linalg.norm(P - q)
            if dd < 1e-9:
                out.append((-min(hx - abs(P[0] - cx), hy - abs(P[1] - cy)), None, lab))   # inside: no normal
                continue
            out.append((dd, q, lab))
        else:
            x0, y0, x1, y1, r = d
            if min(x0, x1) - 1.5 > P[0] or P[0] > max(x0, x1) + 1.5 or min(y0, y1) - 1.5 > P[1] or P[1] > max(y0, y1) + 1.5:
                continue
            a, b = np.array([x0, y0]), np.array([x1, y1]); ab = b - a; l2 = ab @ ab
            t = 0.0 if l2 < 1e-12 else max(0.0, min(1.0, ((P - a) @ ab) / l2))
            c = a + ab * t; v = P - c; dd = np.linalg.norm(v)
            if dd < 1e-9:
                continue
            out.append((dd - r, c + v / dd * r, lab))
    return out


def mitre(n, i, s_hint=None):
    """half a pair's pitch, grown at a bend to where the INNER leg's mitre stands (HALF / cos(turn / 2)), over the
    two vertices of segment i -- at least where it stands at the snap's 45-degree corners, plus the half step its
    off-grid legs take; 0 for a single"""
    if n not in prs:
        return 0.0
    X = LANES[n]['X']
    worst = 0.0
    for v in (i, i + 1):
        if 0 < v < len(X) - 1:
            worst = max(worst, abs(turn_deg(X, v)))
    return max(HALF / max(math.cos(math.radians(min(worst, 120.0)) / 2), 0.3), HALF_SNAP) + g2


def static_seg(p0, p1, L, own):
    """(edge distance, parameter on p0p1, the object's nearest point) for other nets' static copper on L near the
    segment p0p1 -- exact: circles and stubs by segment-segment distance, rectangles by their four edges"""
    out = []
    mid = (p0 + p1) / 2
    reach = np.linalg.norm(p1 - p0) / 2 + SREACH
    ks = set()
    for cx in range(int(math.floor((mid[0] - reach) / SCELL)), int(math.floor((mid[0] + reach) / SCELL)) + 1):
        for cy in range(int(math.floor((mid[1] - reach) / SCELL)), int(math.floor((mid[1] + reach) / SCELL)) + 1):
            ks.update(SGRID.get((cx, cy), ()))
    for k_ in ks:
        kind, Ls, net, d, lab = STATIC[k_]
        if L not in Ls or net in own:
            continue
        if kind == 'circ':
            c = np.array([d[0], d[1]])
            dd, s, _t = seg_seg(p0, p1, c, c)
            P = p0 + (p1 - p0) * s
            v = P - c; nv_ = np.linalg.norm(v)
            out.append((dd - d[2], s, (c + v / nv_ * d[2]) if nv_ > 1e-9 else None, lab))
        elif kind == 'seg':
            a, b = np.array([d[0], d[1]]), np.array([d[2], d[3]])
            dd, s, t_ = seg_seg(p0, p1, a, b)
            P, C = p0 + (p1 - p0) * s, a + (b - a) * t_
            v = P - C; nv_ = np.linalg.norm(v)
            out.append((dd - d[4], s, (C + v / nv_ * d[4]) if nv_ > 1e-9 else None, lab))
        else:
            cx, cy, hx, hy = d
            corners = [np.array(q) for q in ((cx - hx, cy - hy), (cx + hx, cy - hy), (cx + hx, cy + hy), (cx - hx, cy + hy))]
            inside = [s for s in (0.0, 0.5, 1.0) if abs((p0 + (p1 - p0) * s)[0] - cx) <= hx and abs((p0 + (p1 - p0) * s)[1] - cy) <= hy]
            if inside:
                out.append((-1.0, inside[0], None, lab)); continue
            best = None
            for e in range(4):
                a, b = corners[e], corners[(e + 1) % 4]
                dd, s, t_ = seg_seg(p0, p1, a, b)
                if best is None or dd < best[0]:
                    best = (dd, s, a + (b - a) * t_)
            out.append((best[0], best[1], best[2], lab))
    return out


# ------------------------------------------------------------------ geometry helpers
def seg_seg(p0, p1, q0, q1):
    """closest points of segments p0p1, q0q1: (distance, s on p, t on q)"""
    d1, d2, r = p1 - p0, q1 - q0, p0 - q0
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    if a < 1e-14 and e < 1e-14:
        return float(np.linalg.norm(r)), 0.0, 0.0
    if a < 1e-14:
        s, t = 0.0, min(max(f / e, 0.0), 1.0)
    else:
        c = d1 @ r
        if e < 1e-14:
            t, s = 0.0, min(max(-c / a, 0.0), 1.0)
        else:
            b = d1 @ d2; den = a * e - b * b
            s = min(max((b * f - c * e) / den, 0.0), 1.0) if den > 1e-14 else 0.0
            t = (b * s + f) / e
            if t < 0:
                t, s = 0.0, min(max(-c / a, 0.0), 1.0)
            elif t > 1:
                t, s = 1.0, min(max((b - c) / a, 0.0), 1.0)
    return float(np.linalg.norm(p0 + d1 * s - q0 - d2 * t)), s, t


def barrels(n, i):
    """a via vertex's barrels: itself, or a pair's two across its arriving direction -- as (vertex, offset vector)"""
    X = LANES[n]['X']
    if n not in prs:
        return [np.zeros(2)]
    d = X[i] - X[i - 1]
    if np.linalg.norm(d) < 1e-9:
        d = X[i + 1] - X[i]
    d = d / max(np.linalg.norm(d), 1e-12)
    nn = np.array([-d[1], d[0]])
    return [nn * OFF, -nn * OFF]


# ------------------------------------------------------------------ the constraints of one round
def gather():
    """every rule within MARGIN of its bar, linearised: (terms [(lane, vertex, coefficient-vector)], rhs, kind, label,
    current value) meaning sum coef . dX >= rhs"""
    rows = []
    segs = collections.defaultdict(list)       # grid cell -> (lane, i) segments i..i+1
    CELL = SCELL
    for n, ln in LANES.items():
        X = ln['X']
        for i in range(len(X) - 1):
            lo, hi = np.minimum(X[i], X[i + 1]), np.maximum(X[i], X[i + 1])
            for cx in range(int(math.floor(lo[0] / CELL)), int(math.floor(hi[0] / CELL)) + 1):
                for cy in range(int(math.floor(lo[1] / CELL)), int(math.floor(hi[1] / CELL)) + 1):
                    segs[(cx, cy)].append((n, i))

    def near_segs(P, r):
        out = set()
        for cx in range(int(math.floor((P[0] - r) / CELL)), int(math.floor((P[0] + r) / CELL)) + 1):
            for cy in range(int(math.floor((P[1] - r) / CELL)), int(math.floor((P[1] + r) / CELL)) + 1):
                out.update(segs.get((cx, cy), ()))
        return sorted(out)          # the LP's row order: a set's would follow the hash seed

    seen = set()
    # pitch: same-layer lane segments
    for n, ln in LANES.items():
        X, Ls = ln['X'], ln['L']
        for i in range(len(X) - 1):
            mid = (X[i] + X[i + 1]) / 2
            r = np.linalg.norm(X[i + 1] - X[i]) / 2 + BLOCK + 2 * (HALF_SNAP + g2) + MARGIN
            for (m, j) in near_segs(mid, r):
                if m <= n or LANES[m]['L'][j] != Ls[i]:
                    continue
                Y = LANES[m]['X']
                need = BLOCK + mitre(n, i, s_hint=None) + mitre(m, j, s_hint=None)
                d, s, t = seg_seg(X[i], X[i + 1], Y[j], Y[j + 1])
                if d >= need + MARGIN or d < 1e-9:
                    continue
                P, Q = X[i] + (X[i + 1] - X[i]) * s, Y[j] + (Y[j + 1] - Y[j]) * t
                nv = (P - Q) / d
                key = ('p', n, i, m, j)
                if key in seen:
                    continue
                seen.add(key)
                rows.append(([(n, i, nv * (1 - s)), (n, i + 1, nv * s), (m, j, -nv * (1 - t)), (m, j + 1, -nv * t)],
                             need + EPS - d, 'pitch', f'{n}/{m} {Ls[i][0]}', d - need))
    # vias: each barrel vs other lanes' segments (either layer), other lanes' barrels, static of other nets
    VIAS = [(n, i) for n in LANES for i in via_idx(n)]
    for (n, i) in VIAS:
        X = LANES[n]['X']
        vx = g2 if n in prs else 0.0                 # a pair's barrels stay off the grid when the snap lays its dive
        for bo in barrels(n, i):
            B = X[i] + bo
            for (m, j) in near_segs(B, NEED_VL + vx + HALF_SNAP + g2 + MARGIN):
                if m == n:
                    continue
                Y = LANES[m]['X']
                d, _s, t = seg_seg(B, B, Y[j], Y[j + 1])
                need = NEED_VL + vx + mitre(m, j, s_hint=None)
                if d >= need + MARGIN or d < 1e-9:
                    continue
                Q = Y[j] + (Y[j + 1] - Y[j]) * t
                nv = (B - Q) / d
                rows.append(([(n, i, nv), (m, j, -nv * (1 - t)), (m, j + 1, -nv * t)], need + EPS - d, 'via-lane',
                             f'{n}~{m}', d - need))
            for (m, k) in VIAS:
                if m == n or (m, k) < (n, i):
                    continue
                Y = LANES[m]['X']
                need = NEED_VV + vx + (g2 if m in prs else 0.0)
                for bo2 in barrels(m, k):
                    C = Y[k] + bo2
                    d = float(np.linalg.norm(B - C))
                    if d >= need + MARGIN or d < 1e-9:
                        continue
                    nv = (B - C) / d
                    rows.append(([(n, i, nv), (m, k, -nv)], need + EPS - d, 'via-via', f'{n}~{m}', d - need))
            for L in ('F.Cu', 'B.Cu'):
                for dd, q, lab in static_near(B, L, OWN[n]):
                    need = NEED_VST + vx
                    if dd >= need + MARGIN:
                        continue
                    if q is None:
                        rows.append(([], 1.0, 'via-static', f'{n} inside {lab}', dd - need)); continue
                    nv = (B - q) / np.linalg.norm(B - q)
                    rows.append(([(n, i, nv)], need + EPS - dd, 'via-static', f'{n}~{lab}', dd - need))
    # the joins: a single's last W_SCALE at either end runs inside the arc of router directions that each keep within
    # 90 degrees of its stub (two half-planes per segment) -- the moves the snap may make there. Within 90 degrees of
    # the stub alone is not enough: SDQ4's stub runs 14 degrees off north, and a line heading east-north-east is
    # followed by east steps, 104 degrees off it. (A pair's approach is laid within 45 and held: pair_approaches)
    for n, ways in JOIN_WAYS.items():
        X = LANES[n]['X']
        s = arclen(X)
        for end, nrm in enumerate(ways):
            for u in nrm:
                for i in range(len(X) - 1):
                    if (s[i] < W_SCALE) if end == 0 else (s[-1] - s[i + 1] < W_SCALE):
                        val = float((X[i + 1] - X[i]) @ u)
                        if val < MARGIN:
                            rows.append(([(n, i + 1, u), (n, i, -u)], EPS - val, 'stub join', f'{n}{"<>"[end]}', val))
    # static: each lane SEGMENT vs other nets' copper on its layer, at the exact nearest points
    for n, ln in LANES.items():
        X, Ls = ln['X'], ln['L']
        for i in range(len(X) - 1):
            for dd, s, q, lab in static_seg(X[i], X[i + 1], Ls[i], OWN[n]):
                need = NEED_ST + hw[n]
                if dd >= need + MARGIN:
                    continue
                P = X[i] + (X[i + 1] - X[i]) * s
                if q is None or np.linalg.norm(P - q) < 1e-9:
                    rows.append(([], 1.0, 'static', f'{n} inside {lab}', dd - need)); continue
                nv = (P - q) / np.linalg.norm(P - q)
                rows.append(([(n, i, nv * (1 - s)), (n, i + 1, nv * s)], need + EPS - dd, 'static',
                             f'{n}~{lab}', dd - need))
    return rows


def measure(rows):
    bad = [r for r in rows if r[4] < -1e-6]
    return collections.Counter(r[2] for r in bad), bad


# ------------------------------------------------------------------ one LP round
def solve_round(rows, trust):
    active = [r for r in rows if r[0]]
    movable = set()
    for terms, *_ in active:
        for (n, i, _c) in terms:
            if not fixed(n, i):
                movable.add((n, i))
    # neighbours move too (a lane shifts as a stretch, not a spike); vias' neighbours so its legs follow
    for (n, i) in list(movable):
        for di in (-2, -1, 1, 2):
            j = i + di
            if 0 < j < len(LANES[n]['X']) - 1:
                movable.add((n, j))
    idx = {v: k for k, v in enumerate(sorted(movable))}
    nv = len(idx)
    if not nv:
        return None
    # variables: dx, dy per vertex (2 nv), |dx|,|dy| aux (2 nv), smooth aux (2 nv per interior), slack per row
    cols = 2 * nv
    cost = [0.0] * cols
    b = []
    bounds = [(-trust, trust)] * (2 * nv)

    def newv(c, lo=0.0, hi=None):
        nonlocal cols
        cost.append(c); bounds.append((lo, hi)); cols += 1
        return cols - 1
    rowsA = []
    for (n, i), k in idx.items():
        for ax in (0, 1):
            a = newv(1.0)                                   # |d| >= +-d
            rowsA.append(({2 * k + ax: 1.0, a: -1.0}, 0.0)); rowsA.append(({2 * k + ax: -1.0, a: -1.0}, 0.0))
    W_SM = 2.0
    for (n, i), k in idx.items():
        kp, kn = idx.get((n, i - 1)), idx.get((n, i + 1))
        for ax in (0, 1):
            terms = {2 * k + ax: 1.0}
            for kk in (kp, kn):
                if kk is not None:
                    terms[2 * kk + ax] = terms.get(2 * kk + ax, 0.0) - 0.5
            a = newv(W_SM)
            rowsA.append(({**terms, a: -1.0}, 0.0))
            rowsA.append(({**{c: -v for c, v in terms.items()}, a: -1.0}, 0.0))
    slack_of = []
    for terms, rhs, kind, lab, cur in active:
        coef = collections.defaultdict(float)
        for (n, i, cvec) in terms:
            k = idx.get((n, i))
            if k is None:
                continue
            coef[2 * k] += float(cvec[0]); coef[2 * k + 1] += float(cvec[1])
        sl = newv(1e4)
        slack_of.append((sl, kind, lab))
        # sum coef.d + slack >= rhs   ->   -sum coef.d - slack <= -rhs
        rowsA.append(({**{c: -v for c, v in coef.items()}, sl: -1.0}, -rhs))
    # shape: a moved vertex keeps its turn under 90 degrees at the lane's own scale (d1 . d2 >= 0, linearised, the
    # directions taken W_SCALE back and ahead) -- a per-vertex limit let a fold through in two steps
    S_ = {n: arclen(LANES[n]['X']) for n in {n for (n, _i) in idx}}
    for (n, i), k in idx.items():
        X = LANES[n]['X']
        if fixed(n, i):
            continue
        jb, jf = scale_window(S_[n], i)
        d1, d2 = X[i] - X[jb], X[jf] - X[i]
        kp, kn = idx.get((n, jb)), idx.get((n, jf))
        # (d1 + D_i - D_b) . (d2 + D_f - D_i) >= 0  ~  d1.d2 + d2.(D_i - D_b) + d1.(D_f - D_i) >= 0
        coef = collections.defaultdict(float)
        for ax in (0, 1):
            coef[2 * k + ax] += d2[ax] - d1[ax]
            if kp is not None:
                coef[2 * kp + ax] -= d2[ax]
            if kn is not None:
                coef[2 * kn + ax] += d1[ax]
        rowsA.append(({c: -v for c, v in coef.items()}, float(d1 @ d2)))
    from scipy.sparse import coo_matrix
    ri, ci, vv = [], [], []
    for r_, (terms, rhs) in enumerate(rowsA):
        for c, v in terms.items():
            ri.append(r_); ci.append(c); vv.append(v)
        b.append(rhs)
    Aub = coo_matrix((vv, (ri, ci)), shape=(len(rowsA), cols)).tocsr()
    res = linprog(np.array(cost), A_ub=Aub, b_ub=np.array(b), bounds=bounds, method='highs')
    if res.status != 0:
        log(f'  LP status {res.status}: {res.message}')
        return None
    x = res.x
    for (n, i), k in idx.items():
        LANES[n]['X'][i] += np.array([x[2 * k], x[2 * k + 1]])
    paid = [(float(x[sl]), kind, lab) for sl, kind, lab in slack_of if x[sl] > 1e-5]
    return nv, len(active), paid


def pair_approaches():
    """a PAIR's end stretch -- the pair router's first setback from its tips -- laid straight, within 45 degrees of its
    stub (the chord's own direction, turned into that cone when it lies outside), and held: the pair router launches
    from a pose ON the plan and turns at most 45 degrees at a time. At a ring berth the frame runs across the stub
    (SCK came along the ring and turned 90 degrees into its berth), which no frame offset can express."""
    laid = []
    cone = math.radians(45.0)
    for n in [n for n in LANES if n in prs]:
        ln = LANES[n]
        for end in (0, 1):
            X, Ls = (ln['X'], ln['L']) if end == 0 else (ln['X'][::-1], ln['L'][::-1])
            tips = ctx.pair_ends[n][end]
            sb = _pairs.approach_setback(tips, TW, cfg.grid_step)
            s = arclen(X)
            if s[-1] < 3 * sb:
                continue                                 # too short to hold both ends apart
            k = int(np.searchsorted(s, sb))
            vs = [v if end == 0 else len(Ls) - v for v in via_idx(n)]
            if any(0 < v <= k for v in vs):
                continue                                 # a dive inside the setback: the plan's own, left as it is
            chord = X[k] - X[0]
            c = chord / np.linalg.norm(chord)
            u = stub_ways(n)[end]
            u = None if u is None else (u if end == 0 else -u)       # walking out of this end
            if u is not None:
                ang = math.atan2(u[0] * c[1] - u[1] * c[0], u[0] * c[0] + u[1] * c[1])
                a_ = max(-cone, min(cone, ang))
                c = np.array([u[0] * math.cos(a_) - u[1] * math.sin(a_), u[0] * math.sin(a_) + u[1] * math.cos(a_)])
            A = X[0] + c * float(np.linalg.norm(chord))
            new = X.copy()
            for i in range(1, k + 1):
                new[i] = X[0] + (A - X[0]) * (i / k)
            # the next setback of lane runs straight into the approach (not held): turning the chord into the cone moves
            # its far end sideways, and the lane behind it would jog to meet it (a notch, SCK)
            k2 = int(np.searchsorted(s, s[k] + sb))
            if k2 < len(X) - 1 and not any(k < v <= k2 for v in vs):
                for i in range(k + 1, k2):
                    new[i] = A + (X[k2] - A) * ((i - k) / (k2 - k))
            ln['X'] = new if end == 0 else new[::-1]
            PIN[n][end] = k
            laid.append(f'{n}{"<>"[end]} {math.degrees(abs(ang)) if u is not None else 0:.0f}deg')
    return laid


# ------------------------------------------------------------------ run
dr = drop_faults()
if dr:
    log(f'shape faults straightened: {dict(dr)}')
pa_ = pair_approaches()
if pa_:
    log(f'pair approaches laid (the chord off its stub): {", ".join(pa_)}')


def join_arc(u):
    """the inward normals of the arc of router directions within 90 degrees of the stub's own way u (the snap's rule
    for every move near an end): one normal when the arc is a half-plane, two otherwise; none at a via end"""
    if u is None:
        return []
    ok = [k for k in range(8) if float(OCT[k] @ u) >= -1e-9]
    lo = next(k for k in ok if (k - 1) % 8 not in ok)             # the arc's first direction, counter-clockwise
    hi = next(k for k in ok if (k + 1) % 8 not in ok)             # ... and its last
    n1 = np.array([-OCT[lo][1], OCT[lo][0]])
    n2 = np.array([OCT[hi][1], -OCT[hi][0]])
    return [n1] if float(n1 @ n2) > 1 - 1e-9 else [n1, n2]


JOIN_WAYS = {n: [join_arc(u) for u in stub_ways(n, router=False)] for n in LANES if n not in prs}
# the rounds, then the audit's shape measures again: a round that bends a lane past them is straightened and the
# rounds run again (SA6 overshot its berth and doubled back 114 degrees; the scale measure alone read 82)
for cycle in range(1, CYCLES + 1):
    rows = gather()
    cnt, bad = measure(rows)
    log(f'round 0: {sum(cnt.values())} short ({dict(cnt)})')
    for rd in range(1, ROUNDS + 1):
        out = solve_round(rows, TRUST)
        if out is None:
            break
        nv, na, paid = out
        rows = gather()
        cnt, bad = measure(rows)
        log(f'round {rd}: moved {nv} vertices against {na} rules; {sum(cnt.values())} short ({dict(cnt)})'
            + (f'; the LP paid {len(paid)} (max {max(p[0] for p in paid):.3f})' if paid else ''))
        if not bad:
            break
    dr = drop_faults()
    if not dr:
        break
    log(f'shape faults after the rounds, straightened: {dict(dr)}' + (' -- the rounds again' if cycle < CYCLES else ''))
    if cycle == CYCLES:
        rows = gather()
        cnt, bad = measure(rows)
for r in sorted(bad, key=lambda r: r[4])[:25]:
    log(f'  SHORT {r[2]:10s} {r[3]:32s} {r[4]:+.4f}')

# ------------------------------------------------------------------ write: pieces from the vertices, same-layer collinear runs merged
res = dict(geo)
res['lanes'], res['vias'] = {}, []
for n, ln in LANES.items():
    X, Ls = ln['X'], ln['L']
    pieces = []
    for i in range(len(X) - 1):
        p, q = X[i], X[i + 1]
        if pieces and pieces[-1][4] == Ls[i]:
            a = pieces[-1]
            cr = (a[2] - a[0]) * (q[1] - a[1]) - (a[3] - a[1]) * (q[0] - a[0])
            if abs(cr) < 1e-7:
                pieces[-1] = (a[0], a[1], float(q[0]), float(q[1]), a[4]); continue
        pieces.append((float(p[0]), float(p[1]), float(q[0]), float(q[1]), Ls[i]))
    res['lanes'][n] = {'xy': [(pieces[0][0], pieces[0][1])] + [(pc[2], pc[3]) for pc in pieces], 'pieces': pieces}
    for i in via_idx(n):
        res['vias'].append((n, float(X[i][0]), float(X[i][1])))
# the lane-island clearances the polish could not meet on the side the geometry chose: fed back as side flips
flips = {tuple(x) for x in geo.get('flips', [])}
for r in bad:
    if r[2] != 'static':
        continue
    lane_, _, what = r[3].replace(' inside ', '~').partition('~')
    if what.startswith('pad '):
        flips.add((lane_, what[4:].split('.')[0]))
# ...and a stub join the polish could not meet: the pad island nearest the lane's end stretch holds its approach off
# the stub on this side (SCAS ran north of C6 and could only descend into its berth; south of it, up through the
# pads' gap, it arrives the stub's way)
for r in bad:
    if r[2] != 'stub join':
        continue
    lane_, end = r[3][:-1], r[3][-1]
    X, Ls = LANES[lane_]['X'], LANES[lane_]['L']
    s = arclen(X)
    near = []
    for i in range(len(X)):
        if (s[i] < 2 * W_SCALE) if end == '<' else (s[-1] - s[i] < 2 * W_SCALE):
            L = Ls[min(i, len(Ls) - 1)]
            near += [(dd, lab) for dd, _q, lab in static_near(X[i], L, OWN[lane_]) if lab.startswith('pad ')]
    if near:
        flips.add((lane_, min(near)[1][4:].split('.')[0]))
res['flips'] = sorted(flips)
if flips - {tuple(x) for x in geo.get('flips', [])}:
    log(f'side flips for the next geometry: {sorted(flips - {tuple(x) for x in geo.get("flips", [])})}')
json.dump(res, open(OUT, 'w'))
log(f'wrote {OUT}: {len(res["lanes"])} lanes, {len(res["vias"])} vias')

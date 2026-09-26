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

A plan's HELD lanes (the pairs, laid by whole_snap --pairs) keep every vertex; the singles are fitted round them.
A pair not yet laid has each dive laid straight and held: one router heading through it, the pair router's straight
run (pairs.via_straight) and a grid step more on each side.

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
STILL = 1e-6                       # mm: a round moving no vertex further than this (KiCad's own unit, a nanometre)
                                   # has settled -- the rounds after it change no board
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
RING, RING_PAIR = _pairs.via_ring(cfg), _pairs.via_ring(cfg, HALF)      # a via's reach into a track's grid cells
NEED_VV = min_via_center_distance(cfg.via_size, CL, cfg.via_drill, h2h) + 2 * g2
NEED_ST = TW / 2 + CL + g2
NEED_VST = VR + CL + g2
MARGIN = cfg.grid_step           # constraints within this of their bar are kept in the LP (so a move breaks none)
EPS = 1e-4                       # met with this much to spare (the audit compares at 1e-6)
# the snap (whole_snap) lays a pair's centreline on the grid with 45-degree corners, where its legs stand HALF_SNAP
# from it, and its legs off the grid: a pair's bars here carry both, so every gap the snap splits keeps a grid row
# of slack (a single's already do: BLOCK is its on-grid bar plus a step)
HALF_SNAP = HALF / math.cos(math.pi / 8)
# the pair router tests a dive at three cells, the centre and SPC grid steps either way across its heading, on the
# pair's map: each a via's half, a track's half, the clearance and half the pair's pitch from another lane's line, a
# via and the clearance from its via site (whole_snap.pfield)
SPC = _pairs.pose_via_cells(cfg, HALF)
R_LINE = VR + TW / 2 + CL + HALF
R_VIA = 2 * VR + CL
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
    xo = v.get('cross')
    mid = set()
    if xo:
        # a CROSSED pair (whole_snap): its centreline between the crossover's two poses is no copper either -- the
        # crossover's legs and barrels are (below, as static copper)
        bef, aft = _pairs.cut_span([((pc[0], pc[1]), (pc[2], pc[3]), pc[4]) for pc in pcs], xo['poses'][0],
                                   xo['poses'][1])
        span = [(tuple(xo['poses'][0]), tuple(xo['at']), xo['layers'][0]),
                (tuple(xo['at']), tuple(xo['poses'][1]), xo['layers'][1])]
        pcs = [(a_[0], a_[1], b_[0], b_[1], L_) for a_, b_, L_ in bef + span + aft]
        mid = {len(bef), len(bef) + 1}
    pts = [(pcs[0][0], pcs[0][1])] + [(pc[2], pc[3]) for pc in pcs]
    X, Ls = densify(pts, [pc[4] for pc in pcs], 4 * cfg.grid_step)
    LANES[n] = dict(X=np.array(X, float), L=Ls, H=np.zeros(len(X), bool))     # H: vertices held where they are laid
    # a pair with END CONNECTORS (whole_snap): its first and last pieces join its tips' midpoints to its poses and are
    # no copper -- its end legs are (below, as static copper): their segments take part in no row
    nc, xv = set(), set()
    if v.get('ends') or xo:
        seg_n = [max(1, int(math.ceil(math.hypot(pc[2] - pc[0], pc[3] - pc[1]) / (4 * cfg.grid_step)))) for pc in pcs]
        off_ = np.concatenate([[0], np.cumsum(seg_n)])
        if v.get('ends'):
            nc = set(range(seg_n[0])) | set(range(len(X) - 1 - seg_n[-1], len(X) - 1))
        for k_ in mid:
            nc |= set(range(int(off_[k_]), int(off_[k_ + 1])))
        if mid:
            xv = {int(off_[min(mid) + 1])}                # the crossover's centre: no pair dive, no barrels of its own
    LANES[n]['NC'] = nc
    LANES[n]['XV'] = xv
# a plan's HELD lanes (the pairs, laid as the pair router moves by whole_snap --pairs) stay exactly where they are:
# the singles are fitted round them
HELD = set(geo.get('held', []))
for n in HELD:
    LANES[n]['H'][:] = True


def via_idx(n):
    """indices of the vertices where lane n changes layer (its vias)"""
    Ls = LANES[n]['L']
    return [i for i in range(1, len(Ls)) if Ls[i] != Ls[i - 1]]


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
    return i == 0 or i == len(LANES[n]['X']) - 1 or bool(LANES[n]['H'][i])


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
                held = np.flatnonzero(ln['H'])
                j0 = max([j0] + [int(h) for h in held if h <= i0] + [v for v in vs if v <= i0])
                j1 = min([j1] + [int(h) for h in held if h >= i1] + [v for v in vs if v >= i1])
                if j1 - j0 >= 2:
                    hit = (j0, j1)
                    break
            if hit is None:
                break
            j0, j1 = hit
            ln['X'] = np.concatenate([X[:j0 + 1], X[j1:]]); ln['L'] = Ls[:j0] + Ls[j1 - 1:]
            ln['H'] = np.concatenate([ln['H'][:j0 + 1], ln['H'][j1:]])
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
# a held pair's END LEGS are laid where they are drawn: copper the singles are fitted round -- and a crossed pair's
# crossover, its legs and its two barrels
for n in HELD:
    for e_ in geo['lanes'][n].get('ends', []):
        for pts, leg in zip(e_['legs'], prs[n]):
            for a_, b_ in zip(pts, pts[1:]):
                STATIC.append(('seg', {e_['layer']}, ctx.byname[leg][0], (a_[0], a_[1], b_[0], b_[1], TW / 2),
                               f'{n} end leg'))
    xo = geo['lanes'][n].get('cross')
    if xo:
        legs_ = dict(zip(('P', 'N'), prs[n]))
        for k_, runs in xo['legs'].items():
            for pts, L_ in runs:
                for a_, b_ in zip(pts, pts[1:]):
                    STATIC.append(('seg', {L_}, ctx.byname[legs_[k_]][0], (a_[0], a_[1], b_[0], b_[1], TW / 2),
                                   f'{n} crossover leg'))
        for vx_, vy_, k_ in xo['vias']:
            STATIC.append(('circ', {'F.Cu', 'B.Cu'}, ctx.byname[legs_[k_]][0], (vx_, vy_, VR), f'{n} crossover via'))
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
            v = P - np.array([cx, cy]); dd = np.linalg.norm(v)
            if dd < 1e-9:
                continue
            out.append((dd - r, np.array([cx, cy]) + v / dd * r, lab))
        elif kind == 'rect':
            cx, cy, hx, hy = d
            q = np.array([min(max(P[0], cx - hx), cx + hx), min(max(P[1], cy - hy), cy + hy)])
            dd = np.linalg.norm(P - q)
            if dd < 1e-9:
                out.append((-min(hx - abs(P[0] - cx), hy - abs(P[1] - cy)), None, lab))   # inside: no normal
                continue
            out.append((dd, q, lab))
        else:
            x0, y0, x1, y1, r = d
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


def pose_cells(n, i):
    """the offsets from a pair's via vertex of the three cells the pair router checks for its dive: the centre, and SPC
    grid steps either way along the integer perpendicular of its heading (the arriving direction, to the nearest of
    the eight), so sqrt(2) further on a diagonal"""
    X = LANES[n]['X']
    d = X[i] - X[i - 1]
    if np.linalg.norm(d) < 1e-9:
        d = X[i + 1] - X[i]
    k = int(round(math.atan2(d[1], d[0]) / (math.pi / 4))) % 8
    ux, uy = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)][k]
    o = np.array([-uy, ux], float) * SPC * 2 * g2
    return [np.zeros(2), o, -o]


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
            if i in ln['NC']:
                continue
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
            if i in ln['NC']:
                continue
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
    VIAS = [(n, i) for n in LANES for i in via_idx(n) if i not in LANES[n]['XV']]
    for (n, i) in VIAS:
        X = LANES[n]['X']
        for bo in barrels(n, i):
            B = X[i] + bo
            # how far the barrel stands off the grid point the router rounds it to: a laid (held) pair's exactly; a
            # pair's not yet laid half a step (its barrels stay off the grid when the snap lays its dive)
            vx = (math.hypot(B[0] - round(B[0] / (2 * g2)) * 2 * g2, B[1] - round(B[1] / (2 * g2)) * 2 * g2)
                  if n in HELD else g2 if n in prs else 0.0)
            for (m, j) in near_segs(B, RING_PAIR + vx + 2 * g2 + MARGIN):
                if m == n:
                    continue
                Y = LANES[m]['X']
                d, _s, t = seg_seg(B, B, Y[j], Y[j + 1])
                # the router's RING round the via (pairs.via_ring: rounded up to whole cells; a pair's centreline
                # by the pair's), plus a grid step: the snap's share of the gap holds a row
                need = (RING_PAIR if m in prs else RING) + vx + 2 * g2
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
        if n in prs:
            # the cells the pair router checks for the dive, each kept off the other lanes by its bar plus a grid step
            # (the snap's share of the gap holds a row); a pair not yet laid half a step more (the snap rounds its dive)
            vc = 0.0 if n in HELD else g2
            for co in pose_cells(n, i):
                B = X[i] + co
                for (m, j) in near_segs(B, R_LINE + HALF_SNAP + 3 * g2 + vc + MARGIN):
                    if m == n:
                        continue
                    Y = LANES[m]['X']
                    d, _s, t = seg_seg(B, B, Y[j], Y[j + 1])
                    need = R_LINE + hw[m] + 2 * g2 + vc
                    if d >= need + MARGIN or d < 1e-9:
                        continue
                    Q = Y[j] + (Y[j + 1] - Y[j]) * t
                    nv = (B - Q) / d
                    rows.append(([(n, i, nv), (m, j, -nv * (1 - t)), (m, j + 1, -nv * t)], need + EPS - d,
                                 'dive-lane', f'{n}~{m}', d - need))
                for (m, k) in VIAS:
                    if m == n:
                        continue
                    C = LANES[m]['X'][k]
                    d = float(np.linalg.norm(B - C))
                    need = R_VIA + 2 * g2 + vc
                    if d >= need + MARGIN or d < 1e-9:
                        continue
                    nv = (B - C) / d
                    rows.append(([(n, i, nv), (m, k, -nv)], need + EPS - d, 'dive-via', f'{n}~{m}', d - need))
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
            if i in ln['NC']:
                continue
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
            if 0 < j < len(LANES[n]['X']) - 1 and not fixed(n, j):     # a held vertex stays held
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


DIVE_U = {}          # (lane, via vertex) -> the heading its dive is laid on (pair_approaches: joined to an end run)


def pair_approaches():
    """a PAIR's end stretch -- its END RUN (pairs.end_run) -- laid straight and held: its END CONNECTOR
    (pairs.end_connector) along the stub's own way, then the straight the pair router probes past its pose, within
    max_setback_angle of that way (the chord's own direction, turned into that cone when it lies outside). A connector
    at an angle leaves the pose off the plan (SDQS1's berth run left at 45 degrees; in a band that follows the plan its
    pose had no cell); a whole end run along the stub pinned the lanes round SCK's ends (four pitch findings round 1,
    not converging). Without a stub's way, the chord's own direction, straight. At a ring berth the frame runs across
    the stub (SCK came along the ring and turned 90 degrees into its berth), which no frame offset can express."""
    laid = []
    cone = math.radians(cfg.max_setback_angle)
    for n in [n for n in LANES if n in prs and n not in HELD]:
        ln = LANES[n]
        for end in (0, 1):
            X, Ls = (ln['X'], ln['L']) if end == 0 else (ln['X'][::-1], ln['L'][::-1])
            tips = ctx.pair_ends[n][end]
            sb = _pairs.end_run(cfg, tips)
            s = arclen(X)
            if s[-1] < 3 * sb:
                continue                                 # too short to hold both ends apart
            k = int(np.searchsorted(s, sb))
            vs = [v if end == 0 else len(Ls) - v for v in via_idx(n)]
            if stub_ways(n)[end] is None and any(0 < v <= k for v in vs):
                continue                                 # a dive inside the end run with no way to lay it: left as it is
            chord = X[k] - X[0]
            c = chord / np.linalg.norm(chord)
            u = stub_ways(n)[end]
            u = None if u is None else (u if end == 0 else -u)       # walking out of this end
            if u is None:
                A0 = X[0]                                # no stub's way: the chord straight from the end
                A = X[0] + c * float(np.linalg.norm(chord))
            else:
                u = np.asarray(u, float) / float(np.linalg.norm(u))
                A0 = X[0] + u * _pairs.end_connector(cfg, tips)      # the end connector, along the stub
                ch2 = X[k] - A0
                c2 = ch2 / max(float(np.linalg.norm(ch2)), 1e-12)
                ang = math.atan2(u[0] * c2[1] - u[1] * c2[0], u[0] * c2[0] + u[1] * c2[1])
                a_ = max(-cone, min(cone, ang))
                c2 = np.array([u[0] * math.cos(a_) - u[1] * math.sin(a_), u[0] * math.sin(a_) + u[1] * math.cos(a_)])
                A = A0 + c2 * max(float(np.linalg.norm(ch2)), 0.0)    # the probe, within the cone
            # a dive of its own just past the end run: the router runs straight on from its pose into the via,
            # so the probe and the dive's straight run are ONE line -- the router heading within the cone
            # nearest the way to the via, the via moved onto it (laid apart, SDQS1 folded 122 degrees between them)
            # (a dive inside the end run as well: the end run is short, a grid step or two past the connector's pose,
            # and the dive is moved out along the line to where the router can make it)
            Lst = _pairs.via_straight(cfg, (1.0, 1.0)) + cfg.grid_step
            vn = sorted(v for v in vs if v > 1)
            joined = None
            if u is not None and vn and s[vn[0]] - Lst <= sb + Lst:
                v0 = vn[0]
                w = X[v0] - A0
                c2 = max([o for o in OCT if float(o @ u) >= math.cos(cone) - 1e-9], key=lambda o: float(o @ w))
                sbk = sb - _pairs.end_connector(cfg, tips)               # the probe's own length
                A = A0 + c2 * sbk
                # the router may dive once its probe past the pose and its straight run into a via are both behind it
                # -- both are counted from the pose (whole_snap's search: its straight count starts there)
                Pv = A0 + c2 * max(float(w @ c2), sbk, _pairs.via_straight(cfg, c2) + cfg.grid_step)
                joined = (v0, Pv)
                DIVE_U[(n, v0 if end == 0 else len(Ls) - v0)] = c2 if end == 0 else -c2
            new = X.copy()
            # the end run's vertices -- and a joined dive's, up to its via: one line from the pose on
            kk, Aend = (joined[0], joined[1]) if joined is not None else (k, A)
            l0, l1 = float(np.linalg.norm(A0 - X[0])), float(np.linalg.norm(Aend - A0))
            # a vertex ON the corner between connector and the line; the others spread evenly either side of it
            i0 = min(kk - 1, max(1, int(round(kk * l0 / max(l0 + l1, 1e-12))))) if l0 > 1e-12 else 0
            for i in range(1, kk + 1):
                new[i] = (X[0] + (A0 - X[0]) * (i / i0)) if i <= i0 else (A0 + (Aend - A0) * ((i - i0) / (kk - i0)))
            # the next setback of lane runs straight into the setback (not held): turning the chord into the cone moves
            # its far end sideways, and the lane behind it would jog to meet it (a notch, SCK)
            k2 = int(np.searchsorted(s, s[k] + sb))
            if joined is None and k2 < len(X) - 1 and not any(k < v <= k2 for v in vs):
                for i in range(k + 1, k2):
                    new[i] = A + (X[k2] - A) * ((i - k) / (k2 - k))
            ln['X'] = new if end == 0 else new[::-1]
            if end == 0:
                ln['H'][:kk + 1] = True
            else:
                ln['H'][len(X) - 1 - kk:] = True
            laid.append(f'{n}{"<>"[end]} {math.degrees(abs(ang)) if u is not None else 0:.0f}deg')
    return laid


VCUT = []           # the via cuts the polish sends the solve (pair_dive_straights)


def via_cut(n, v, L):
    """a via cut for lane n's change at its vertex v: its route coordinate from the geometry (the solve's changes, in
    order along the lane), the stretch either side of it that the change must leave"""
    ch = geo.get('changes', {}).get(n)
    vs = via_idx(n)
    if ch is None or len(ch) != len(vs) or v not in vs:
        return
    u = float(ch[vs.index(v)])
    if not any(c_['lane'] == n and abs(c_['u'] - u) < 1e-9 for c_ in VCUT):     # one cut per change
        VCUT.append({'lane': n, 'u': u, 'w': float(L)})


def pair_dive_straights():
    """a PAIR's dive laid straight and held: one heading through the via (the router direction nearest the lane's
    own there), the pair router's straight run (pairs.via_straight) and a grid step more on each side -- it neither
    turns at a via nor within that many steps of one (pose_router.rs straight_after_via; SDQS0 turned 45 degrees at
    its dive, SCK 0.035 mm before its). The next stretch on each side runs straight into it (not held). A dive JOINED
    to its end run (pair_approaches) has its end's side laid already, on that line: only its other side is laid. A
    dive that cannot be laid so -- an end or a held stretch within its straight run, or a straight that folds the lane
    where it joins it (the audit's own shape rule) -- is left as it is and sent to the solve as a VIA CUT (its lane, its
    change's route coordinate, the straight run either side): the change moves off that stretch."""
    laid = []
    for n in [n for n in LANES if n in prs and n not in HELD]:
        ln = LANES[n]
        for v in via_idx(n):
            X, s = ln['X'], arclen(ln['X'])
            reach = _pairs.via_straight(cfg, (1.0, 1.0)) + cfg.grid_step          # the longer (diagonal) run
            at = lambda u_: np.array([np.interp(u_, s, X[:, 0]), np.interp(u_, s, X[:, 1])])
            joined = (n, v) in DIVE_U
            u = DIVE_U.get((n, v), octi(at(s[v] + reach) - at(s[v] - reach)))
            L = _pairs.via_straight(cfg, u) + cfg.grid_step
            ib = int(np.searchsorted(s, s[v] - L, side='left'))
            ia = int(np.searchsorted(s, s[v] + L, side='right')) - 1
            lay_b = not (joined and ln['H'][v - 1])
            lay_a = not (joined and ln['H'][min(v + 1, len(X) - 1)])
            if (lay_b and (s[v] - L < 0 or ib >= v or ln['H'][ib:v].any())) \
                    or (lay_a and (s[v] + L > s[-1] or ia <= v or ln['H'][v + 1:ia + 1].any())) \
                    or (not joined and ln['H'][v]):
                laid.append(f'{n}@{v} not laid (an end or a held stretch within {L:.3f})')
                via_cut(n, v, L)
                continue
            new = X.copy()
            P0, Pb, Pa = X[v], X[v] - u * L, X[v] + u * L
            if lay_b:
                for i in range(ib, v):
                    new[i] = Pb + (P0 - Pb) * ((s[i] - s[ib]) / max(s[v] - s[ib], 1e-12))
                new[ib] = Pb
            if lay_a:
                for i in range(v + 1, ia + 1):
                    new[i] = P0 + (Pa - P0) * ((s[i] - s[v]) / max(s[ia] - s[v], 1e-12))
                new[ia] = Pa
            # the lane runs straight into the held stretch from a straight run's length further on each side
            jb = int(np.searchsorted(s, s[ib] - L, side='left'))
            ja = int(np.searchsorted(s, s[ia] + L, side='right')) - 1
            if lay_b and 0 < jb < ib and not ln['H'][jb:ib].any() and not any(jb < w < ib for w in via_idx(n)):
                for i in range(jb + 1, ib):
                    new[i] = X[jb] + (Pb - X[jb]) * ((i - jb) / (ib - jb))
            if lay_a and ia < ja < len(X) - 1 and not ln['H'][ia + 1:ja + 1].any() and not any(ia < w < ja for w in via_idx(n)):
                for i in range(ia + 1, ja):
                    new[i] = Pa + (X[ja] - Pa) * ((i - ia) / (ja - ia))
            # laid only if it folds nothing where it joins the lane (the audit's shape rule, on the stretch it moved)
            lo_, hi_ = min(jb, ib) if lay_b else v, max(ja, ia) if lay_a else v
            before_ = {f for f in shape_faults(n) if f[1] >= lo_ - 1 and f[0] <= hi_ + 1}
            ln['X'] = new
            after_ = {f for f in shape_faults(n) if f[1] >= lo_ - 1 and f[0] <= hi_ + 1}
            if len(after_) > len(before_):
                ln['X'] = X
                laid.append(f'{n}@{v} not laid (its straight folds the lane)')
                via_cut(n, v, L)
                continue
            ln['H'][(ib if lay_b else v):(ia if lay_a else v) + 1] = True
            laid.append(f'{n}@({P0[0]:.2f},{P0[1]:.2f}) {L:.3f}')
    return laid


# ------------------------------------------------------------------ run
dr = drop_faults()
if dr:
    log(f'shape faults straightened: {dict(dr)}')
pa_ = pair_approaches()
if pa_:
    log(f'pair approaches laid (the chord off its stub): {", ".join(pa_)}')
pd_ = pair_dive_straights()
if pd_:
    log(f'pair dives laid straight: {", ".join(pd_)}')


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
        before_ = {n: ln['X'].copy() for n, ln in LANES.items()}
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
        if all(LANES[n]['X'].shape == X0.shape and float(np.max(np.abs(LANES[n]['X'] - X0), initial=0.0)) <= STILL
               for n, X0 in before_.items()):
            log(f'round {rd} moved no vertex more than {STILL * 1e6:.0f} nm: the rounds have settled')
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
    if n in HELD:
        res['lanes'][n] = dict(geo['lanes'][n])          # laid as it was given (its ends and joins with it)
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
# ...and every change the rounds could not give its room (a via or a pair's dive against a lane, a via against a via):
# a via cut on its own change, a via's room wide (as the geometry cuts the via rows it had to pay)
for r in bad:
    if r[2] in ('via-lane', 'dive-lane', 'via-via'):
        n_, i_ = r[0][0][0], r[0][0][1]
        via_cut(n_, i_, VR + (HALF if n_ in prs else 0.0))
res['vcuts'] = VCUT
if VCUT:
    log(f'via cuts for the solve: {[(c_["lane"], round(c_["u"], 2), round(c_["w"], 3)) for c_ in VCUT]}')
json.dump(res, open(OUT, 'w'))
log(f'wrote {OUT}: {len(res["lanes"])} lanes, {len(res["vias"])} vias')

"""whole_polish.py GEO.json OUT.json -- the geometry LP's lanes made to meet the AUDIT's own measures, in board xy.

The column LP (whole_geo) measures in its spine frames, which are not Euclidean off a curved spine and linearise slopes
with tangent cuts, so its lanes come out a few um short of the audit's bars in dozens of places. This polish works on
the lanes' board polylines directly: every near-violation of the audit's rules becomes a linear constraint on small
vertex moves (the normal taken from the current geometry), a small LP over the vertices near violations finds the
least movement that meets them all, and that repeats (re-linearised) until the audit's measures hold or no move helps.

  pitch    same-layer lanes: track + clearance + grid (+ a pair's legs' reach, below)
  dives    a via (a pair: its two barrels, pairs.dive_offset across the arriving direction) vs other lanes' lines on
           either layer, other lanes' vias, static copper of other nets
  static   lane vs other nets' pads / stubs / vias on its layer: track/2 + clearance + grid/2 (a pad as KiCad draws
           it, rounded corners and all, and a single's in its corner zone the router's corner buffer further:
           pairs.pad_corner_buffer)
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

import awx_settings
import detmath
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
# how far off its grid point a barrel of a dive not yet laid can stand, once the snap lays the dive on the grid: OFF
# across each of the router's eight headings, the worst of them -- the snap bars it twice that (the router rings the
# point it rounds to), so a bar here of that and a grid step leaves the snap its row (half a step, the old allowance,
# is short of it on a diagonal)
_goff = lambda v: abs(v - round(v / cfg.grid_step) * cfg.grid_step)
VX_OFF = 2 * max(math.hypot(_goff(OFF * -dy / math.hypot(dx, dy)), _goff(OFF * dx / math.hypot(dx, dy)))
                 for dx, dy in [(1, 0), (1, 1), (0, 1), (-1, 1)]) if prs else 0.0
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
DIAG = (math.sqrt(0.5), math.sqrt(0.5))   # a diagonal heading, as a UNIT vector (via_straight and the crossover's runs
                                         # read u's length: (1, 1) gave the axis run, 0.225 where the diagonal's is 0.318)
# the pair router tests a dive at three cells, the centre and SPC grid steps either way across its heading, on the
# pair's map: each a via's half, a track's half, the clearance and half the pair's pitch from another lane's line, a
# via and the clearance from its via site (whole_snap.pfield)
SPC = _pairs.pose_via_cells(cfg, HALF)
R_LINE = VR + TW / 2 + CL + HALF
R_VIA = 2 * VR + CL
hw = {n: (HALF_SNAP + g2 if n in prs else 0.0) for n in geo['lanes']}
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


# a crossed pair's CROSSOVER not yet laid (pairs.opposite_hands: its legs swap once, at its first dive): its two barrels
# on ONE side, staggered along it, and its own straight runs either way (pairs.crossover_shape) -- on the side with
# the more room from the other lanes when first asked (whole_snap lays it on the side that leaves them the most), as the
# audit measures it
XO_SIDE = {}


def xo_at(n, i):
    """is lane n's via vertex i its crossover (a crossed pair's first dive, not yet laid)?"""
    return n in prs and n not in HELD and _pairs.opposite_hands(ctx, n) and bool(via_idx(n)) and i == via_idx(n)[0]


def xo_runs(u, side=1):
    """a crossover's straight runs (before, after) on heading u, and a grid step more each way, as a dive's"""
    sh = _pairs.crossover_shape(cfg, (float(u[0]), float(u[1])), side)
    L_ = _pairs.via_straight(cfg, u)
    return ((sh[1] if sh else L_) + cfg.grid_step, (sh[2] if sh else L_) + cfg.grid_step)


def octi(u):
    """the router direction (0, 45, 90 ... degrees) nearest u"""
    return max(OCT, key=lambda o: float(o @ np.asarray(u, float)))


def stub_ways(n, router=True):
    """(the way lane n leaves its tooth, the way it arrives at its berth) from its stubs' last segments -- each the
    router direction nearest it, or (router False) the segment's own -- None where the stub ends in a via (the lane
    lands on it and continues no line). A PAIR's are the pair's own escape directions, as the snap's (end_dirs) and
    the frame's landing (whole_frame): one leg's last stub segment can converge on the tips at 45 degrees (SCKP at
    K28's berth), and held along it the pair folded back onto its landing"""
    L0, L1 = LANES[n]['L'][0], LANES[n]['L'][-1]
    if n in prs:
        a, b = ctx.tooth_dir.get(n), ctx.stub_dir.get(n)
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
STATIC = []          # (kind, layers, net, data): pads ('rect' cx cy hx hy cr), holes and vias ('circ' cx cy r), segs (x0 y0 x1 y1 hw)
RL = set(cfg.layers)  # the routing layers (connect.make_config): a hole's, a drilled pad's and a via's copper on every one
for ref_, fp in ctx.pcb.footprints.items():            # (labelled by the board's key: a repeated reference has its own)
    for pd in fp.pads:
        if pd.pad_type == 'np_thru_hole':
            STATIC.append(('circ', set(RL), pd.net_id, (pd.global_x, pd.global_y, (pd.drill or 0) / 2), f'hole {ref_}.{pd.pad_number}'))
            continue
        Ls = set(RL) if (pd.drill and pd.drill > 0) or '*.Cu' in pd.layers \
            else {L for L in pd.layers if L in RL}
        if not Ls:
            continue
        # a pad as KiCad draws its copper: a rectangle with its corners rounded (a circle's and an oval's by half its
        # width), where the router's corner buffer takes over from the flat bar (pad_dist)
        STATIC.append(('rect', Ls, pd.net_id, (pd.global_x, pd.global_y, pd.size_x / 2, pd.size_y / 2,
                                               _pairs.pad_corner_radius(pd)), f'pad {ref_}.{pd.pad_number}'))
for s in ctx.base_segments:
    STATIC.append(('seg', {s.layer}, s.net_id, (s.start_x, s.start_y, s.end_x, s.end_y, s.width / 2), 'copper'))
for v in ctx.base_vias:
    STATIC.append(('circ', set(RL), v.net_id, (v.x, v.y, v.size / 2), 'via'))
# a held pair's END LEGS are laid where they are drawn: copper the singles are fitted round -- and a crossed pair's
# crossover, its legs and its two barrels (in name order: a set's would follow the hash seed, and this order is the
# static list's and so the LP's rows')
for n in sorted(HELD):
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
            STATIC.append(('circ', set(RL), ctx.byname[legs_[k_]][0], (vx_, vy_, VR), f'{n} crossover via'))
OWN = {n: {ctx.byname[n][0]} | {ctx.byname[leg][0] for leg in prs.get(n, ()) if leg in ctx.byname} for n in LANES}
# static objects binned by bounding box; a query looks SREACH round its point: the widest bar to static copper
SCELL = 2 * _pairs.pitch(TW)
# the layers a via's barrel meets: the two pages, then a board's inner layers, whose copper no lane meets and every via
# does (a 2-layer board: the pages alone, in the order the rows were always built)
VIA_LAYERS = ('F.Cu', 'B.Cu') + tuple(L for L in ctx.pcb.board_info.copper_layers if L not in ('F.Cu', 'B.Cu'))
SREACH = (max(NEED_VST + OFF + g2, NEED_ST + HALF_SNAP + g2, BLOCK + 2 * (HALF_SNAP + g2)) + MARGIN
          + _pairs.corner_buffer(cfg.grid_step))            # (a pad's corner buffer: pad_dist)
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


def pad_dist(P, d, pair):
    """(distance from P to a pad's copper, less -- a single's (not `pair`) -- the router's corner buffer where P stands
    in its corner zone (pairs.pad_corner_buffer), the pad's nearest point -- None inside), for a pad d = (cx, cy, hx, hy,
    cr): the rounded rectangle is its inner rectangle grown by cr"""
    cx, cy, hx, hy, cr = d
    ix, iy = hx - cr, hy - cr
    q = np.array([min(max(P[0], cx - ix), cx + ix), min(max(P[1], cy - iy), cy + iy)])
    v = P - q; dd = float(np.linalg.norm(v))
    cb = 0.0 if pair else float(_pairs.pad_corner_buffer(P[0] - cx, P[1] - cy, hx, hy, cr, cfg.grid_step))
    if dd <= 1e-9:                      # inside the inner rectangle: no normal
        return -min(hx - abs(P[0] - cx), hy - abs(P[1] - cy)) - cb, None
    return dd - cr - cb, q + v / dd * cr    # (within the rounding, inside or out: its signed distance and its normal)


def static_near(P, L, own, pair=False):
    """(distance to the object's edge, the object's nearest point) for every static object of other nets on L near P
    (a pad's as pad_dist reads it; `pair`: a pair's copper)"""
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
            dd, q = pad_dist(P, d, pair)
            out.append((dd, q, lab))                     # inside: no normal (q None)
        else:
            x0, y0, x1, y1, r = d
            a, b = np.array([x0, y0]), np.array([x1, y1]); ab = b - a; l2 = ab @ ab
            t = 0.0 if l2 < 1e-12 else max(0.0, min(1.0, ((P - a) @ ab) / l2))
            c = a + ab * t; v = P - c; dd = np.linalg.norm(v)
            if dd < 1e-9:
                continue
            out.append((dd - r, c + v / dd * r, lab))
    return out


def mitre(n, i):
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


def static_seg(p0, p1, L, own, cut=math.inf, pair=False):
    """(edge distance, parameter on p0p1, the object's nearest point) for other nets' static copper on L near the
    segment p0p1 -- exact: circles and stubs by segment-segment distance, a pad by its inner rectangle's four edges
    grown by its corner radius (seg_rrect), and -- a single's (not `pair`) -- the part of the segment in each of the
    pad's corner zones again, less the router's corner buffer (pad_dist), the worst of them. An object whose bounding
    box stands further than cut (and a pad's buffer) from the segment's is further still, and left out"""
    out = []
    sx0, sy0, sx1, sy1 = min(p0[0], p1[0]), min(p0[1], p1[1]), max(p0[0], p1[0]), max(p0[1], p1[1])
    cut2 = (cut + 1e-9) * (cut + 1e-9)

    def far(bx0, by0, bx1, by1, grow):
        gx, gy = max(0.0, bx0 - grow - sx1, sx0 - bx1 - grow), max(0.0, by0 - grow - sy1, sy0 - by1 - grow)
        return gx * gx + gy * gy > cut2
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
            if far(d[0], d[1], d[0], d[1], d[2]):
                continue
            c = np.array([d[0], d[1]])
            dd, s, _t = seg_seg(p0, p1, c, c)
            P = p0 + (p1 - p0) * s
            v = P - c; nv_ = np.linalg.norm(v)
            out.append((dd - d[2], s, (c + v / nv_ * d[2]) if nv_ > 1e-9 else None, lab))
        elif kind == 'seg':
            if far(min(d[0], d[2]), min(d[1], d[3]), max(d[0], d[2]), max(d[1], d[3]), d[4]):
                continue
            a, b = np.array([d[0], d[1]]), np.array([d[2], d[3]])
            dd, s, t_ = seg_seg(p0, p1, a, b)
            P, C = p0 + (p1 - p0) * s, a + (b - a) * t_
            v = P - C; nv_ = np.linalg.norm(v)
            out.append((dd - d[4], s, (C + v / nv_ * d[4]) if nv_ > 1e-9 else None, lab))
        else:
            cx, cy, hx, hy, cr = d
            if far(cx - hx, cy - hy, cx + hx, cy + hy, _pairs.corner_buffer(cfg.grid_step)):
                continue
            inside = [s for s in (0.0, 0.5, 1.0)       # (in the copper, its corners rounded: not the box)
                      if _pairs.pad_distance(*(p0 + (p1 - p0) * s - np.array([cx, cy])), hx, hy, cr) <= 0.0]
            if inside:
                out.append((-1.0, inside[0], None, lab)); continue
            # the rounded rectangle's distance, and within each corner's zone (the router's corner buffer: pad_dist)
            # the part of the segment there, less the buffer -- the worst of them
            best = seg_rrect(p0, p1, d)
            ax_, ay_ = _pairs.corner_zone(hx, hy, cr, cfg.grid_step)
            cb = 0.0 if pair else _pairs.corner_buffer(cfg.grid_step)
            for sx in ((-1.0, 1.0) if cb > 0 else ()):
                for sy in (-1.0, 1.0):
                    t = clip_quadrant(p0, p1, cx + sx * ax_, cy + sy * ay_, sx, sy)
                    if t is None:
                        continue
                    a_, b_ = p0 + (p1 - p0) * t[0], p0 + (p1 - p0) * t[1]
                    dd, s_, q = seg_rrect(a_, b_, d)
                    if dd - cb < best[0]:
                        best = (dd - cb, t[0] + s_ * (t[1] - t[0]), q)
            out.append((best[0], best[1], best[2], lab))
    return out


def seg_rrect(p0, p1, d):
    """(distance, parameter on p0p1, nearest point of the pad) from segment p0p1 to a pad d = (cx, cy, hx, hy, cr)
    outside it: its inner rectangle's four edges, grown by cr"""
    cx, cy, hx, hy, cr = d
    ix, iy = hx - cr, hy - cr
    corners = [np.array(q) for q in ((cx - ix, cy - iy), (cx + ix, cy - iy), (cx + ix, cy + iy), (cx - ix, cy + iy))]
    best = None
    for e in range(4):
        a, b = corners[e], corners[(e + 1) % 4]
        dd, s, t_ = seg_seg(p0, p1, a, b)
        if best is None or dd < best[0]:
            best = (dd, s, a + (b - a) * t_)
    dd, s, qi = best
    P = p0 + (p1 - p0) * s
    v = P - qi; nv_ = np.linalg.norm(v)
    return dd - cr, s, (qi + v / nv_ * cr) if nv_ > 1e-9 else qi


def clip_quadrant(p0, p1, x0, y0, sx, sy):
    """the parameter interval (t0, t1) of segment p0p1 inside the quadrant sx * (x - x0) > 0, sy * (y - y0) > 0, or
    None"""
    lo, hi = 0.0, 1.0
    for c0, dc in ((sx * (p0[0] - x0), sx * (p1[0] - p0[0])), (sy * (p0[1] - y0), sy * (p1[1] - p0[1]))):
        if abs(dc) < 1e-15:
            if c0 <= 0:
                return None
            continue
        t = -c0 / dc
        if dc > 0:
            lo = max(lo, t)
        else:
            hi = min(hi, t)
    return (lo, hi) if hi - lo > 1e-12 else None


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
    if xo_at(n, i):
        if n not in XO_SIDE:
            def room_(side):
                sh = _pairs.crossover_shape(cfg, (float(d[0]), float(d[1])), side)
                if sh is None:
                    return -math.inf
                return min(float(np.linalg.norm(X[i] + a_ * d + c_ * nn - q_)) for a_, c_ in sh[0]
                           for m, v_ in LANES.items() if m != n for q_ in v_['X'])
            XO_SIDE[n] = max((1, -1), key=room_)
        sh = _pairs.crossover_shape(cfg, (float(d[0]), float(d[1])), XO_SIDE[n])
        if sh is not None:
            return [a_ * d + c_ * nn for a_, c_ in sh[0]]
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
    # each segment's bounding box and its mitre, once: a pair of segments whose boxes stand further apart than its bar
    # (and the margin) is further apart still, and needs no closer look
    BOX = {n: [(min(X_[i][0], X_[i + 1][0]), min(X_[i][1], X_[i + 1][1]), max(X_[i][0], X_[i + 1][0]),
                max(X_[i][1], X_[i + 1][1])) for i in range(len(X_) - 1)] for n, X_ in ((n, ln['X'].tolist()) for n, ln in LANES.items())}
    MIT = {n: [mitre(n, i) for i in range(len(ln['X']) - 1)] for n, ln in LANES.items()}
    # pitch: same-layer lane segments
    for n, ln in LANES.items():
        X, Ls = ln['X'], ln['L']
        for i in range(len(X) - 1):
            if i in ln['NC']:
                continue
            mid = (X[i] + X[i + 1]) / 2
            r = np.linalg.norm(X[i + 1] - X[i]) / 2 + BLOCK + 2 * (HALF_SNAP + g2) + MARGIN
            ax0, ay0, ax1, ay1 = BOX[n][i]
            for (m, j) in near_segs(mid, r):
                if m <= n or LANES[m]['L'][j] != Ls[i]:
                    continue
                need = BLOCK + MIT[n][i] + MIT[m][j]
                bx0, by0, bx1, by1 = BOX[m][j]
                gx, gy = max(0.0, bx0 - ax1, ax0 - bx1), max(0.0, by0 - ay1, ay0 - by1)
                if gx * gx + gy * gy > (need + MARGIN + 1e-9) * (need + MARGIN + 1e-9):
                    continue
                Y = LANES[m]['X']
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
                  if n in HELD else VX_OFF if n in prs else 0.0)
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
            for L in VIA_LAYERS:
                for dd, q, lab in static_near(B, L, OWN[n], pair=n in prs):
                    need = NEED_VST + vx
                    if lab.endswith((' end leg', ' crossover leg')):
                        # a held pair's leg is a TRACK, off the grid: the router keeps the via's ring (pairs.via_ring,
                        # rounded up to whole cells) and half a step from it, as the audit and the snap bar it -- the
                        # via-to-track clearance alone stood 0.306 where they ask 0.319
                        need = RING - TW / 2 + g2 + vx
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
    # a held pair's CROSSOVER barrels (laid where drawn; no vertex of the lane stands for them) are vias as its dive
    # barrels are: each other lane's line kept outside the router's ring round it, the barrel's own offset from its
    # grid point and a grid step -- as static copper alone they stood at the via-to-track clearance, a ring short
    # (SA6 0.306 from SCK's barrel where the audit asks 0.325)
    for n in sorted(HELD):
        xo = geo['lanes'][n].get('cross')
        for vx_, vy_, _k in (xo['vias'] if xo else []):
            B = np.array([vx_, vy_], float)
            vx = math.hypot(B[0] - round(B[0] / (2 * g2)) * 2 * g2, B[1] - round(B[1] / (2 * g2)) * 2 * g2)
            for (m, j) in near_segs(B, RING_PAIR + vx + 2 * g2 + MARGIN):
                if m == n:
                    continue
                Y = LANES[m]['X']
                d, _s, t = seg_seg(B, B, Y[j], Y[j + 1])
                need = (RING_PAIR if m in prs else RING) + vx + 2 * g2
                if d >= need + MARGIN or d < 1e-9:
                    continue
                Q = Y[j] + (Y[j + 1] - Y[j]) * t
                nv = (B - Q) / d
                rows.append(([(m, j, -nv * (1 - t)), (m, j + 1, -nv * t)], need + EPS - d, 'via-lane', f'{n}~{m}',
                             d - need))
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
        # (a pair not yet laid: its centreline's first and last stretch, from its tips' midpoint onto its pose, is its
        # END CONNECTOR, no copper -- its legs converge there from the tips, either side of what stands between them,
        # and the audit measures them. Measured as a line with a rounded end on the midpoint, every pair's end read
        # 0.113 short of the ball between its tips (SDQS0 and DU1.F1, its legs 0.27 clear), a row no move could meet)
        cu = None
        if n in prs and n not in HELD and not geo['lanes'][n].get('ends'):
            s_ = arclen(X)
            cu = (_pairs.end_connector(cfg, ctx.pair_ends[n][0]), s_[-1] - _pairs.end_connector(cfg, ctx.pair_ends[n][1]))
        for i in range(len(X) - 1):
            if i in ln['NC']:
                continue
            t0, t1 = 0.0, 1.0
            if cu is not None:
                if s_[i + 1] <= cu[0] + 1e-9 or s_[i] >= cu[1] - 1e-9:
                    continue
                sl_ = s_[i + 1] - s_[i]
                t0, t1 = max(0.0, (cu[0] - s_[i]) / sl_), min(1.0, (cu[1] - s_[i]) / sl_)
                if t1 <= t0:                     # (a lane no longer than its two connectors: nothing between them)
                    continue
            A_, B_ = X[i] + (X[i + 1] - X[i]) * t0, X[i] + (X[i + 1] - X[i]) * t1
            for dd, s, q, lab in static_seg(A_, B_, Ls[i], OWN[n], cut=NEED_ST + hw[n] + MARGIN, pair=n in prs):
                s = t0 + s * (t1 - t0)
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
    # one optimum, not a face, and without the solver's last bits: the same polish on every run (detmath)
    res = linprog(np.array(cost) + detmath.lp_tie_break(len(cost)), A_ub=Aub, b_ub=np.array(b), bounds=bounds, method='highs')
    if res.status != 0:
        log(f'  LP status {res.status}: {res.message}')
        return None
    x = detmath.lp_round(res.x)
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
    pose had no cell); along the stub for its connector and probe only, not its whole end run, which pins the lanes
    round a pair's ends. Without a stub's way, the chord's own direction, straight. At a ring berth the frame runs across
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
            Lst = _pairs.via_straight(cfg, DIAG) + cfg.grid_step
            if _pairs.opposite_hands(ctx, n):
                Lst = max(Lst, max(xo_runs(DIAG)))               # (its first dive a crossover)
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
                vi_ = v0 if end == 0 else len(Ls) - v0                 # (the vertex in the lane's own order)
                Lv_ = xo_runs(c2)[end] if xo_at(n, vi_) else _pairs.via_straight(cfg, c2) + cfg.grid_step
                Pv = A0 + c2 * max(float(w @ c2), sbk, Lv_)
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
            reach = _pairs.via_straight(cfg, DIAG) + cfg.grid_step          # the longer (diagonal) run
            at = lambda u_: np.array([np.interp(u_, s, X[:, 0]), np.interp(u_, s, X[:, 1])])
            joined = (n, v) in DIVE_U
            u = DIVE_U.get((n, v), octi(at(s[v] + reach) - at(s[v] - reach)))
            L = _pairs.via_straight(cfg, u) + cfg.grid_step
            Lb_, La_ = xo_runs(u, XO_SIDE.get(n, 1)) if xo_at(n, v) else (L, L)      # (a crossover's own runs)
            L = max(Lb_, La_)
            ib = int(np.searchsorted(s, s[v] - Lb_, side='left'))
            ia = int(np.searchsorted(s, s[v] + La_, side='right')) - 1
            lay_b = not (joined and ln['H'][v - 1])
            lay_a = not (joined and ln['H'][min(v + 1, len(X) - 1)])
            if (lay_b and (s[v] - Lb_ < 0 or ib >= v or ln['H'][ib:v].any())) \
                    or (lay_a and (s[v] + La_ > s[-1] or ia <= v or ln['H'][v + 1:ia + 1].any())) \
                    or (not joined and ln['H'][v]):
                laid.append(f'{n}@{v} not laid (an end or a held stretch within {L:.3f})')
                via_cut(n, v, L)
                continue
            new = X.copy()
            P0, Pb, Pa = X[v], X[v] - u * Lb_, X[v] + u * La_
            if lay_b:
                for i in range(ib, v):
                    new[i] = Pb + (P0 - Pb) * ((s[i] - s[ib]) / max(s[v] - s[ib], 1e-12))
                new[ib] = Pb
            if lay_a:
                for i in range(v + 1, ia + 1):
                    new[i] = P0 + (Pa - P0) * ((s[i] - s[v]) / max(s[ia] - s[v], 1e-12))
                new[ia] = Pa
            # the lane runs straight into the held stretch from a straight run's length further on each side
            jb = int(np.searchsorted(s, s[ib] - Lb_, side='left'))
            ja = int(np.searchsorted(s, s[ia] + La_, side='right')) - 1
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
# the lane-island clearances the polish could not meet on the side the geometry chose: fed back as side flips -- of the
# parts the geometry holds lanes to one side of (whole_geo's islands: every part's pads and holes but the source's and
# the destination's; a flip of either would change nothing and cost a round)
_SRC = collections.Counter(ctx.src_ref[n] for n in geo['lanes']).most_common(1)[0][0]
_DST = awx_settings.req('DEST')


ISLAND = whole_ctx.part_islands(ctx, skip=(_SRC, _DST))


def island_of(lab):
    """the island a static label names ('pad REF.N ...', 'hole REF.N': that pad's, whole_ctx.part_islands), or None"""
    for pre in ('pad ', 'hole '):
        if lab.startswith(pre):
            ref, _, num = lab[len(pre):].split(' ')[0].rpartition('.')
            if ref in (_SRC, _DST) or ref not in ctx.pcb.footprints:
                return None
            if geo.get('islands') is not None:        # the islands the geometry held lanes to
                return geo['islands'].get(f'{ref}.{num}')
            return next((ISLAND[(ref, i)] for i, p in enumerate(ctx.pcb.footprints[ref].pads)
                         if str(p.pad_number) == num and (ref, i) in ISLAND), None)
    return None


flips = {tuple(x) for x in geo.get('flips', [])}
LCUT = []
_NL, _ibx = len(cfg.layers), geo.get('island_boxes') or {}
for r in bad:
    if r[2] != 'static':
        continue
    lane_, _, what = r[3].replace(' inside ', '~').partition('~')
    isl_ = island_of(what)
    if isl_ and _NL > 2 and isl_ in _ibx and len(_ibx[isl_][4]) < _NL:
        # (more routing layers than two: an island not on every one is answered under it -- the lane held off its
        # layers there, a LAYER cut -- not by sending the lane round it)
        c_ = {'lane': lane_, 'island': isl_, 'layer': 1 - min(_ibx[isl_][4]), 'blocked': list(_ibx[isl_][4]),
              'box': list(_ibx[isl_][:4])}
        if c_ not in LCUT:
            LCUT.append(c_)
        continue
    if isl_:
        flips.add((lane_, isl_))
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
            near += [(dd, island_of(lab)) for dd, _q, lab in static_near(X[i], L, OWN[lane_], pair=lane_ in prs) if island_of(lab)]
    if near:
        flips.add((lane_, min(near)[1]))
res['flips'] = sorted(flips)
if flips - {tuple(x) for x in geo.get('flips', [])}:
    log(f'side flips for the next geometry: {sorted(flips - {tuple(x) for x in geo.get("flips", [])})}')
# ...and every change the rounds could not give its room (a via or a pair's dive against a lane, a via against a via):
# a via cut on its own change, a via's room wide (as the geometry cuts the via rows it had to pay)
for r in bad:
    # (a via against static copper too, where the row names its change: one inside the copper names none)
    if r[2] in ('via-lane', 'dive-lane', 'via-via') or (r[2] == 'via-static' and r[0]):
        n_, i_ = r[0][0][0], r[0][0][1]
        via_cut(n_, i_, VR + (HALF if n_ in prs else 0.0))
res['vcuts'] = VCUT
res['lcuts'] = LCUT
if LCUT:
    log(f'layer cuts for the solve: {[(c_["lane"], c_["island"]) for c_ in LCUT]}')
if VCUT:
    log(f'via cuts for the solve: {[(c_["lane"], round(c_["u"], 2), round(c_["w"], 3)) for c_ in VCUT]}')
json.dump(res, open(OUT, 'w'))
log(f'wrote {OUT}: {len(res["lanes"])} lanes, {len(res["vias"])} vias')

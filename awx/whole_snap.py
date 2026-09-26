"""whole_snap.py PLAN.json OUT.json [--pairs] -- a smooth plan that FITS (whole_polish, passing the audit), made
octilinear on the router's grid, one lane at a time.

--pairs lays only the PAIRS, against static copper and each other, and writes them HELD with the singles still
smooth: the polish then fits the singles round them, and a second snap places the held pairs as given and lays the
singles. A pair moves as the pair router does (pose_router.rs, its two counters in the search's state): 45-degree
turns at most, each followed by pairs.turn_straight_steps straight steps; a via only after pairs.via_straight_steps
straight steps and followed by as many, one heading through it.

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
from plane_pad_tap import make_local_window
from obstacle_map import (build_base_obstacle_map, add_same_net_via_clearance, add_same_net_pad_drill_via_clearance,
                          same_net_pad_via_keepout_cells)
from routing_context import _add_free_via_positions

plan = json.load(open(sys.argv[1]))
OUT = sys.argv[2]
# --pairs: only the PAIRS laid (as the pair router moves), against static copper and each other; the singles are left
# smooth for the polish to fit round them. A plan's 'held' lanes are placed exactly as it gives them.
PAIRS_ONLY = '--pairs' in sys.argv[3:]
HELD = set(plan.get('held', []))
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
RING, RING_PAIR = _pairs.via_ring(cfg), _pairs.via_ring(cfg, HALF)      # a via's reach into a track's grid cells
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
SPC = _pairs.pose_via_cells(cfg, HALF)                    # a pair's barrels, in cells across its heading
LIDX = {L: k for k, L in enumerate(cfg.layers)}
STATIC = {}


def static_masks(n, i0, j0, band):
    """the board's fixed copper as the router itself stamps it: its base obstacle map (obstacle_map.
    build_base_obstacle_map: pads with their corner buffers, other nets' stubs, vias and holes, the board edge) over
    lane n's window with n's own nets excluded -- a pair's with the pair's extra clearance, half its pitch, as the
    pair step builds it -- and n's own pads' via keep-outs. Read at the cells n can use: track cells per layer, and
    the cells where a via of n clears (a pair's: the centre and the two barrels the pair router checks, SPC cells
    along each heading's integer perpendicular). Cached: static copper does not move between sweeps"""
    key = (n, i0, j0, band.shape)
    if key in STATIC:
        return STATIC[key]
    NI, NJ = band.shape
    R = SPC if n in prs else 0
    x0, y0, x1, y1 = (i0 - R) * g, (j0 - R) * g, (i0 + NI + R) * g, (j0 + NJ + R) * g
    # the window's own edge is stamped as a board edge: keep it well clear of every cell read (a few lane pitches, as
    # the audit's static windows)
    win = make_local_window(ctx.pcb, (x0 + x1) / 2, (y0 + y1) / 2, max(x1 - x0, y1 - y0) / 2 + 4 * bd.LANE_MIN)
    own = sorted(OWN[n])
    obs = build_base_obstacle_map(win, cfg, own, HALF if n in prs else 0.0, static_base=True)
    _add_free_via_positions(obs, win, own, cfg)
    for nid in own:
        add_same_net_via_clearance(obs, win, nid, cfg)
        add_same_net_pad_drill_via_clearance(obs, win, nid, cfg)
        keep = same_net_pad_via_keepout_cells(ctx.pcb, nid, cfg)
        if len(keep):
            obs.add_blocked_vias_batch(keep)
    real = obs.unwrap() if hasattr(obs, 'unwrap') else obs
    trk = {L: np.ones((NI, NJ), bool) for L in ('F.Cu', 'B.Cu')}
    for L, arr in trk.items():
        arr[band] = read_cells(real, np.argwhere(band) + (i0, j0), LIDX[L])
    # via cells: the band, grown by the barrels' reach for a pair
    reach = np.zeros((NI + 2 * R, NJ + 2 * R), bool)
    for di in range(-R, R + 1):
        for dj in range(-R, R + 1):
            reach[R + di:R + di + NI, R + dj:R + dj + NJ] |= band
    vb = np.ones(reach.shape, bool)
    vb[reach] = read_cells(real, np.argwhere(reach) + (i0 - R, j0 - R), None)
    if n in prs:
        via = []
        for a in range(4):
            px, py = -DIRS[a][1] * SPC, DIRS[a][0] * SPC
            c_ = vb[R:R + NI, R:R + NJ]
            via.append(c_ | vb[R + px:R + px + NI, R + py:R + py + NJ] | vb[R - px:R - px + NI, R - py:R - py + NJ])
    else:
        via = [vb] * 4
    STATIC[key] = (trk, via)
    return STATIC[key]


def read_cells(real, cells, layer):
    """blocked flags of the map at grid cells [(gx, gy)] on a layer (None: a via's)"""
    if layer is None:
        f = real.is_via_blocked
        return np.fromiter((f(int(a), int(b)) for a, b in cells), bool, len(cells))
    f = real.is_blocked
    return np.fromiter((f(int(a), int(b), layer) for a, b in cells), bool, len(cells))


def seg_pts_dist(P, Q, X, Y):
    vx, vy = Q[0] - P[0], Q[1] - P[1]
    L2 = vx * vx + vy * vy
    t = np.clip(((X - P[0]) * vx + (Y - P[1]) * vy) / L2, 0.0, 1.0) if L2 > 1e-18 else np.zeros_like(X)
    return np.hypot(X - (P[0] + t * vx), Y - (P[1] + t * vy))


# ------------------------------------------------------------------ the stub's last segment at each end
def last_dir(net, pt, layer):
    return whole_ctx.stub_dir(ctx, net, pt, layer)


def end_dirs(n):
    """(out of the tooth, into the berth) as unit vectors, from the stubs' last segments -- a PAIR's from the pair's
    own escape directions, the ones the pair step works from (a leg's own last stub segment can converge on the tips
    at 45 degrees: SDQS1's berth)"""
    L0, L1 = LANE[n]['lays'][0], LANE[n]['lays'][-1]
    if n in prs:
        a, b = ctx.tooth_dir.get(n), ctx.stub_dir.get(n)
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


# ------------------------------------------------------------------ a pair's end connectors
STATIC_NEAR = {}


def static_near(n, end):
    """other nets' copper near a pair's end: static_around its tips' midpoint"""
    tips = ctx.pair_ends[n][end]
    return static_around(_pairs.mid(tips[0], tips[1]), end_reach(tips) + 4 * bd.LANE_MIN)


def end_reach(tips):
    """how far from a pair's tips its end connector's pose is looked for: its end run and two pair pitches more (the
    shortest clear connector is taken)"""
    return _pairs.end_run(cfg, tips) + 2 * _pairs.pitch(TW)


def static_around(m, R):
    """the board's copper within R of m, per net it is foreign to: (pads [(pad, layers, kind)], segments, vias) -- a
    leg or a barrel is measured against every one not of its own net, the pair's other leg's stub included"""
    key = (round(m[0] / g), round(m[1] / g), round(R / g))
    if key in STATIC_NEAR:
        return STATIC_NEAR[key]
    near = lambda x, y: math.hypot(x - m[0], y - m[1]) < R
    pads = []
    for fp in ctx.pcb.footprints.values():
        for pd in fp.pads:
            if not near(pd.global_x, pd.global_y):
                continue
            if pd.pad_type == 'np_thru_hole':
                pads.append((pd, {'F.Cu', 'B.Cu'}, 'hole'))
            elif (pd.drill and pd.drill > 0) or any(L_.startswith('*') for L_ in pd.layers):
                pads.append((pd, {'F.Cu', 'B.Cu'}, 'pad'))
            else:
                pads.append((pd, {L_ for L_ in pd.layers if L_ in ('F.Cu', 'B.Cu')}, 'pad'))
    segs = [s_ for s_ in ctx.base_segments if near(s_.start_x, s_.start_y) or near(s_.end_x, s_.end_y)]
    vias = [v_ for v_ in ctx.base_vias if near(v_.x, v_.y)]
    STATIC_NEAR[key] = (pads, segs, vias)
    return STATIC_NEAR[key]


def leg_clear(pts, L, net, n, near):
    """a leg (points, layer, its net) clear of every other net's copper at half a track and the clearance -- pads as
    drawn, the other leg's stub among them -- and of the copper placed so far (a placed pair's legs off the grid by
    half a step), exactly: the pair step lays it where it is"""
    pads, segs, vias = near
    bar = TW / 2 + CL
    step = g / 4
    samples = []
    for a_, b_ in zip(pts, pts[1:]):
        k = max(1, int(math.ceil(math.hypot(b_[0] - a_[0], b_[1] - a_[1]) / step)))
        samples += [(a_[0] + (b_[0] - a_[0]) * t / k, a_[1] + (b_[1] - a_[1]) * t / k) for t in range(k + 1)]
    S_ = np.array(samples)
    for pd, Ls, kind in pads:
        if pd.net_id == net or L not in Ls:
            continue
        if kind == 'hole':
            d = np.hypot(S_[:, 0] - pd.global_x, S_[:, 1] - pd.global_y) - (pd.drill or 0) / 2
        else:
            d = _pairs.pad_distance(S_[:, 0] - pd.global_x, S_[:, 1] - pd.global_y, pd.size_x / 2, pd.size_y / 2,
                                    _pairs.pad_corner_radius(pd))
        if float(np.min(d)) < bar + step / 2:
            return False
    for s_ in segs:
        if s_.net_id == net or s_.layer != L:
            continue
        if _pairs.poly_dist(pts, [(s_.start_x, s_.start_y), (s_.end_x, s_.end_y)]) < bar + s_.width / 2 - 1e-9:
            return False
    for v_ in vias:
        if v_.net_id != net and min(_pairs._pt_seg((v_.x, v_.y), a_, b_) for a_, b_ in zip(pts, pts[1:])) \
                < bar + v_.size / 2 - 1e-9:
            return False
    for (m_, L_, p_, q_) in LEGS:
        if m_ != n and L_ == L and _pairs.poly_dist(pts, [p_, q_]) < TW + CL + g / 2 - 1e-9:
            return False
    for (m_, L_, p_, q_) in PLACED:
        if m_ != n and m_ not in prs and L_ == L and _pairs.poly_dist(pts, [p_, q_]) < TW + CL - 1e-9:
            return False
    for (m_, bx_, by_) in PVIAS:
        if m_ != n and min(_pairs._pt_seg((bx_, by_), a_, b_) for a_, b_ in zip(pts, pts[1:])) < VR + TW / 2 + CL - 1e-9:
            return False
    return True


def via_clear(x, y, net, n, near):
    """a barrel of net at (x, y), laid as drawn, clear of every other net's copper on both layers -- pads as drawn,
    holes by their drill -- and of what is placed (a placed pair's legs off the grid by half a step)"""
    pads, segs, vias = near
    for pd, Ls, kind in pads:
        if pd.net_id == net:
            continue
        if kind == 'hole':
            if math.hypot(x - pd.global_x, y - pd.global_y) < cfg.via_drill / 2 + (pd.drill or 0) / 2 + h2h - 1e-9:
                return False
            continue
        d = float(_pairs.pad_distance(x - pd.global_x, y - pd.global_y, pd.size_x / 2, pd.size_y / 2,
                                      _pairs.pad_corner_radius(pd)))
        if d < VR + CL - 1e-9:
            return False
    for s_ in segs:
        if s_.net_id != net and _pairs._pt_seg((x, y), (s_.start_x, s_.start_y), (s_.end_x, s_.end_y)) \
                < VR + s_.width / 2 + CL - 1e-9:
            return False
    for v_ in vias:
        if v_.net_id != net and math.hypot(x - v_.x, y - v_.y) < max(VR + v_.size / 2 + CL,
                                                                        cfg.via_drill / 2 + v_.drill / 2 + h2h) - 1e-9:
            return False
    for (m_, L_, p_, q_) in LEGS:
        if m_ != n and _pairs._pt_seg((x, y), p_, q_) < VR + TW / 2 + CL + g / 2 - 1e-9:
            return False
    for (m_, L_, p_, q_) in PLACED:
        if m_ != n and m_ not in prs and _pairs._pt_seg((x, y), p_, q_) < VR + TW / 2 + CL - 1e-9:
            return False
    for (m_, bx_, by_) in PVIAS:
        if m_ != n and math.hypot(x - bx_, y - by_) < VVB - 1e-9:
            return False
    return True


def is_crossed(n):
    """an OPPOSITE-HANDS pair: P on one side of its travel at its tooth, on the other arriving at its berth (pairs.hand)
    -- its legs must swap sides once, at a dive"""
    (tp, tn), (sp, sn) = ctx.pair_ends[n]
    a = _pairs.hand(ctx.tooth_dir.get(n), tp, tn)
    b = _pairs.hand(ctx.stub_dir.get(n), sp, sn, arriving=True)
    return a != 0 and b != 0 and a != b


def crossover_at(n, V, d, hand0, L1, L2):
    """the pair's crossover at V on heading DIRS[d] (pairs.crossover: P or N diving first, the first clean), its legs
    and barrels clear of everything, or None"""
    pn, nn = prs[n]
    ids = {'P': ctx.byname[pn][0], 'N': ctx.byname[nn][0]}
    near = static_around(V, 4 * bd.LANE_MIN)
    for first in ('P', 'N'):
        c = _pairs.crossover(V, DIRS[d], hand0, HALF, cfg.via_size, VX[n], TW, CL, g, L1, L2, first=first,
                             floor=_pairs.handover_setback(cfg))
        if c is None:
            continue
        if all(leg_clear(pts, L, ids[k], n, near) for k, v in c['legs'].items() for pts, L in v) \
                and all(via_clear(x, y, ids[k], n, near) for x, y, k in c['vias']):
            c['first'] = first
            c['at'] = [V[0], V[1]]
            c['heading'] = list(DIRS[d])
            c['layers'] = [L1, L2]
            return c
    return None


def pair_end_cands(n, end, W, esc):
    """lane n's end connectors at one end (0 its tooth, 1 its berth), shortest first: (length, pose cell (i, j),
    heading index (walking into the lane), the end as the plan records it -- pose, heading, layer, the P and N legs
    from the tips to the pose's legs, the router's own straight onto the pose included)"""
    tips = ctx.pair_ends[n][end]
    L = LANE[n]['lays'][0] if end == 0 else LANE[n]['lays'][-1]
    pn, nn = prs[n]
    nets = (ctx.byname[pn][0], ctx.byname[nn][0])
    m = _pairs.mid(tips[0], tips[1])
    el = math.hypot(*esc)
    e = (esc[0] / el, esc[1] / el)
    floor_ = _pairs.handover_setback(cfg)                 # the router takes over this far past the legs' ends
    reach = end_reach(tips)
    i0, j0, band, bad = W['i0'], W['j0'], W['band'], W['bad']
    out = []
    for (ii, jj) in np.argwhere(band):
        i, j = int(ii) + i0, int(jj) + j0
        x, y = i * g, j * g
        ahead = (x - m[0]) * e[0] + (y - m[1]) * e[1]
        if ahead < max(floor_, g) or math.hypot(x - m[0], y - m[1]) > reach or bad[L][ii, jj]:
            continue
        for k in range(8):
            hx, hy = DIRS[k]
            hl = math.hypot(hx, hy)
            u = (hx / hl, hy / hl)
            if u[0] * e[0] + u[1] * e[1] < -1e-9:
                continue
            q = (x - u[0] * floor_, y - u[1] * floor_)
            legs = _pairs.end_legs(tips, e, q, u, HALF, TW + CL, g, reach)
            if legs is None:
                continue
            full = []
            for pts, net in zip(legs, nets):
                E_ = pts[-1]
                full.append(list(pts) + ([(E_[0] + u[0] * floor_, E_[1] + u[1] * floor_)] if floor_ > 0 else []))
            if not all(leg_clear(pts, L, net, n, static_near(n, end)) for pts, net in zip(full, nets)):
                continue
            ln = sum(math.hypot(b_[0] - a_[0], b_[1] - a_[1]) for pts in full for a_, b_ in zip(pts, pts[1:]))
            out.append((ln, (i, j), k, {'pose': [x, y], 'heading': [u[0], u[1]], 'layer': L,
                                        'legs': [[list(p_) for p_ in pts] for pts in legs],
                                        'handover': [list(pts[-1]) for pts in legs]}))
    return sorted(out, key=lambda c_: (c_[0], c_[1], c_[2]))


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

    def mark(arr, bb, bar, dfun, ox=0.0, oy=0.0):
        """the cells whose point (x + ox, y + oy) lies within bar of the copper dfun measures (inside the box bb)"""
        w = window(bb, bar, ox, oy)
        if w is not None:
            arr[w] |= dfun(X[w] + ox, Y[w] + oy) < bar + EPS

    def share(arr, bar, dfun, bb, ox=0.0, oy=0.0):
        """the cells whose point (x + ox, y + oy) stands nearer an unplaced lane's smooth copper than bar more than
        the cell stands from n's own line: that lane's share"""
        w = window(bb, bar + BAND, ox, oy)
        if w is not None:
            arr[w] |= (dfun(X[w] + ox, Y[w] + oy) - dist[w]) < bar - EPS

    seg_d = lambda p_, q_: (lambda Xs, Ys: seg_pts_dist(p_, q_, Xs, Ys))
    pt_d = lambda x_, y_: (lambda Xs, Ys: np.hypot(Xs - x_, Ys - y_))
    box = lambda p_, q_: (min(p_[0], q_[0]), min(p_[1], q_[1]), max(p_[0], q_[0]), max(p_[1], q_[1]))
    # the board's fixed copper: the router's own base map (static_masks)
    st_trk, st_via = static_masks(n, i0, j0, band)
    for L in bad:
        bad[L] |= st_trk[L]
    # placed copper: a single lane's centreline; a pair's two LEGS (its corners' mitres are wider than its pitch); every
    # placed via as its barrels (a pair's two, across the way it arrived)
    placed = [(e_[0], e_[1], e_[2], e_[3], hw[e_[0]], OFFG[e_[0]]) for e_ in PLACED if e_[0] not in prs] + \
             [(e_[0], e_[1], e_[2], e_[3], 0.0, 1) for e_ in LEGS]
    for (m, L, p_, q_, hm, om) in placed:
        mark(bad[L], box(p_, q_), TW + CL + hw[n] + hm + g / 2 * (OFFG[n] + om), seg_d(p_, q_))
    # a placed via keeps n's track cells out of its RING (pairs.via_ring: the via-to-track clearance rounded up to
    # whole cells plus a quarter, about the grid point the via rounds to, and as far again as it stands off it; a
    # pair's centreline by the pair's) -- the flat clearance left 0.300 where the router blocks to 0.306 (8 singles
    # refused) and 0.450 where it blocks a pair to 0.456 (SDQS0 behind SDQ4's via)
    ring_n = RING_PAIR if n in prs else RING
    for (m, bx_, by_) in PVIAS:
        rx, ry = round(bx_ / g) * g, round(by_ / g) * g
        for L in bad:
            mark(bad[L], (rx, ry, rx, ry), ring_n + math.hypot(bx_ - rx, by_ - ry), pt_d(rx, ry))
    # the lanes not yet placed keep their SHARE of every gap: a cell is n's only where its distance to the neighbour's
    # smooth line exceeds its distance to n's own by the bar (the smooth plan stands its lines a bar and a grid step
    # apart, so each share holds at least one grid row). Left to a soft price, SDQ6 and SDQ7 drifted to 0.225 of
    # SDQ3's line and left it a channel only its exact slanted line fits
    unplaced = [m for m in M if m != n and m not in res['lanes'] and (m in prs or not PAIRS_ONLY)]
    for m in unplaced:
        Pm, Lm = LANE[m]['pts'], LANE[m]['lays']
        for (p_, q_), L in zip(zip(Pm, Pm[1:]), Lm):
            share(bad[L], lane_bar(n, m), seg_d(p_, q_), box(p_, q_))
        for (bx_, by_) in LANE[m]['barrels']:
            for L in bad:
                share(bad[L], ring_n + g / 2 * OFFG[m], pt_d(bx_, by_), (bx_, by_, bx_, by_))

    def vfield(ox, oy):
        """the cells where a barrel of n's via at (x + ox, y + oy) does not clear the lanes: placed copper and
        barrels, and the unplaced lanes' shares (a via is wider than its track, and one slid along the line
        out of the room the plan gave it takes the neighbour's row: SDQ14's via, 0.053 on, closed SDQ12)"""
        vb = np.zeros(X.shape, bool)
        # a via stands outside the RING of every other lane's track (pairs.via_ring about the grid point the via
        # rounds to: the single's round a single's line, the pair's round a pair's centreline) -- whichever routes
        # later meets it that way; a barrel off the grid by its own offset, twice (the router rings its rounded point)
        boff = 2 * math.hypot(ox - round(ox / g) * g, oy - round(oy / g) * g)
        for (m, L, p_, q_) in PLACED:
            mark(vb, box(p_, q_), (RING_PAIR if m in prs else RING) + boff, seg_d(p_, q_), ox, oy)
        for (m, bx_, by_) in PVIAS:
            mark(vb, (bx_, by_, bx_, by_), VVB + g / 2 * (OFFG[n] + OFFG[m]), pt_d(bx_, by_), ox, oy)
        for m in unplaced:
            Pm = LANE[m]['pts']
            for p_, q_ in zip(Pm, Pm[1:]):
                share(vb, (RING_PAIR if m in prs else RING) + boff, seg_d(p_, q_), box(p_, q_), ox, oy)
            for (bx_, by_) in LANE[m]['barrels']:
                share(vb, VVB + g / 2 * (OFFG[n] + OFFG[m]), pt_d(bx_, by_), (bx_, by_, bx_, by_), ox, oy)
        return vb
    def pfield(ox, oy):
        """the pair router's own test of a dive against the other lanes, at one of the three cells it checks (the
        centre, and SPC cells either way along the heading's integer perpendicular): on the pair's map, where every
        other lane is its reservation -- a line a track wide (a pair's two legs), a via at each of its via sites (a
        pair's dive as one) -- a via cell is blocked within a via's half, a track's half, the clearance and the pair's
        extra (half its pitch) of a line, and within a via and the clearance of a via site. The barrels' own field
        (vfield) keeps the other lanes off the barrels where they stand; this keeps the barrels off the lanes where
        the router looks for them (SDQS0's dive, 0.354 from SDQ12's line where the router asks 0.430)"""
        pb = np.zeros(X.shape, bool)
        R_LINE = VR + TW / 2 + CL + HALF
        R_VIA = 2 * VR + CL
        for (m, L, p_, q_) in PLACED:
            if m not in prs:
                mark(pb, box(p_, q_), R_LINE, seg_d(p_, q_), ox, oy)
        for (m, L, p_, q_) in LEGS:
            mark(pb, box(p_, q_), R_LINE, seg_d(p_, q_), ox, oy)
        for m, v_ in res['lanes'].items():
            for (vx_, vy_) in v_['vias']:
                mark(pb, (vx_, vy_, vx_, vy_), R_VIA + math.hypot(vx_ - round(vx_ / g) * g, vy_ - round(vy_ / g) * g),
                     pt_d(vx_, vy_), ox, oy)
        for m in unplaced:
            Pm = LANE[m]['pts']
            for p_, q_ in zip(Pm, Pm[1:]):
                share(pb, R_LINE + hw[m] + g / 2 * OFFG[m], seg_d(p_, q_), box(p_, q_), ox, oy)
            for (vx_, vy_) in LANE[m]['vias']:
                share(pb, R_VIA + g / 2, pt_d(vx_, vy_), (vx_, vy_, vx_, vy_), ox, oy)
        return pb
    if n in prs:
        # a pair dives as two barrels across the way it ARRIVES (as the audit draws them): one field per axis; and
        # the pair router checks its centre and SPC cells either way across that heading (pfield)
        vbad = []
        P0 = pfield(0.0, 0.0)
        for a in range(4):
            ux, uy = -DIRS[a][1], DIRS[a][0]
            ul = math.hypot(ux, uy)
            ox, oy = VX[n] * ux / ul, VX[n] * uy / ul
            px, py = ux * SPC * g, uy * SPC * g
            vbad.append(vfield(ox, oy) | vfield(-ox, -oy) | st_via[a] | P0 | pfield(px, py) | pfield(-px, -py))
    else:
        vbad = [vfield(0.0, 0.0) | st_via[0]] * 4
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
    # a PAIR moves as the pair router does (pose_router.rs), its two counters in the state: straight steps still owed
    # and straight steps taken. After a 45-degree turn RT straight steps before the next (its turning radius); a via
    # only with none owed and ST taken, and ST owed after it; one heading through the via. A single owes nothing.
    ST = _pairs.via_straight_steps(cfg) if no90 else 0
    CAP = max(ST, _pairs.pose_probe_steps(cfg)) if no90 else 0     # the straight count's ceiling: the most any rule asks
    RT = _pairs.turn_straight_steps(cfg) if no90 else 0
    # an OPPOSITE-HANDS pair (P on one side of its travel at its tooth, the other at its berth) swaps its legs at its
    # dive with a CROSSOVER (pairs.crossover), laid as drawn: its dive only where the crossover clears everything, and
    # straight on its heading from the pair router's probe before the crossover's entry pose to the probe after its
    # exit pose (the pair router routes up to the one and on from the other)
    cross = no90 and is_crossed(n)
    XO = {}
    if cross:
        hand0 = _pairs.hand(ctx.tooth_dir.get(n), *ctx.pair_ends[n][0])
        PR = _pairs.pose_probe_steps(cfg)

        def xo(i, j, d):
            key = (i, j, d)
            if key not in XO:
                XO[key] = crossover_at(n, ((i + i0) * g, (j + j0) * g), d, hand0, lays[0], lays[-1])
            return XO[key]

        def xo_steps(d):
            dx_, dy_ = DIRS[d]
            step_ = g * math.hypot(dx_, dy_)
            c_ = _pairs.crossover((0.0, 0.0), (dx_, dy_), hand0, HALF, cfg.via_size, VX[n], TW, CL, g, 'F.Cu', 'B.Cu',
                                  floor=_pairs.handover_setback(cfg))
            return int(round(-c_['span'][0] / step_)) + PR, int(round(c_['span'][1] / step_)) + PR
        XS = {d: xo_steps(d) for d in range(8)}
        CAP = max(CAP, max(v[0] for v in XS.values()))
    # ...and dives no nearer either end than its DIVE ROOM (pairs.dive_room: its end connector onto the pose, then the
    # router's straight from the pose into the via)
    room0 = _pairs.dive_room(cfg, ctx.pair_ends[n][0], a_out) if no90 else 0.0
    room1 = _pairs.dive_room(cfg, ctx.pair_ends[n][1], a_in) if no90 else 0.0
    def search(S, d0, E, dN, poses=False):
        """the A* from grid cell S, leaving on heading d0, to E: its states, or (None, states searched). poses: S and E
        are a pair's POSES past its end runs, not terminal joins -- no join rules, E reached on its heading dN and
        clear of everything"""
        h = lambda i, j: math.hypot(i - E[0], j - E[1]) * g
        # from a pose, the pair router looks pairs.pose_probe_steps straight ahead before it accepts it: that many
        # straight steps owed at the start, and taken into the far pose
        PR = _pairs.pose_probe_steps(cfg) if poses else 0
        start = (S[0], S[1], d0, 0, (PR, 0))
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
            i, j, d, k, sc = st
            if (i, j) == E and k == K and (not poses or sc[1] >= PR):
                goal = st
                break
            L = lays[k]
            # a via here: the next layer, near the plan's via, where a via clears
            if cross and k < K:
                if sc[0] <= 0 and sc[1] >= XS[d][0] and room0 <= arc[i, j] <= total - room1 \
                        and not bad[lays[k + 1]][i, j] and xo(i, j, d) is not None:
                    x_, y_ = (i + i0) * g, (j + j0) * g
                    dv = math.hypot(x_ - vpts[k][0], y_ - vpts[k][1])
                    if dv <= RVIA:
                        nst = (i, j, d, k + 1, (XS[d][1], 0))
                        nc = c_ + W_VIA * dv
                        if nc < best.get(nst, math.inf) - 1e-12:
                            best[nst] = nc; prev[nst] = st
                            heapq.heappush(pq, (nc + h(i, j), nc, nst))
            elif k < K and not vbad[d % 4][i, j] and sc[0] <= 0 and sc[1] >= ST and room0 <= arc[i, j] <= total - room1:
                x_, y_ = (i + i0) * g, (j + j0) * g
                dv = math.hypot(x_ - vpts[k][0], y_ - vpts[k][1])
                if dv <= RVIA and not bad[lays[k + 1]][i, j]:
                    nst = (i, j, d, k + 1, (ST, 0))
                    nc = c_ + W_VIA * dv
                    if nc < best.get(nst, math.inf) - 1e-12:
                        best[nst] = nc; prev[nst] = st
                        heapq.heappush(pq, (nc + h(i, j), nc, nst))
            for nd in range(8):
                bend = min(abs(nd - d), 8 - abs(nd - d))
                if bend >= 3 or (no90 and bend >= 2) or (bend and sc[0] > 0):
                    continue
                di, dj = DIRS[nd]
                ni, nj = i + di, j + dj
                if not ok(ni, nj) or not band[ni, nj]:
                    continue
                if not poses:
                    # no fold at either end: the first move within 90 degrees of the join out of the tooth, the last
                    # within 90 of the join into the berth (the audit folds a turn over 100)
                    if (i, j) == S and k == 0 and DIRS[nd][0] * jS[0] + DIRS[nd][1] * jS[1] < -1e-9:
                        continue
                    if (ni, nj) == E and k == K and DIRS[nd][0] * jE[0] + DIRS[nd][1] * jE[1] < -1e-9:
                        continue
                    # ...and every move within a track width of either end runs within 90 degrees of its stub's own
                    # way (a sum of such moves cannot fold, so neither can the lane's first or last track width:
                    # SDQ4's stub runs 14 degrees off north, and a last move east read as 90 degrees against the
                    # router direction)
                    if strict and k == 0 and math.hypot(i - S[0], j - S[1]) * g <= TW and DIRS[nd][0] * a_out[0] + DIRS[nd][1] * a_out[1] < -1e-9:
                        continue
                    if strict and k == K and math.hypot(ni - E[0], nj - E[1]) * g <= TW and DIRS[nd][0] * a_in[0] + DIRS[nd][1] * a_in[1] < -1e-9:
                        continue
                if ((ni, nj) != E or poses) and bad[L][ni, nj]:
                    continue
                a_ = arc[ni, nj]
                if not (gate[k][0] <= a_ <= gate[k][1]):
                    continue
                step = g * (math.sqrt(2) if di and dj else 1.0)
                nc = c_ + step * (1 + W_DEV * dist[ni, nj]) + W_BEND * bend
                if (ni, nj) == E:
                    eb = min(abs(nd - dN), 8 - abs(nd - dN))
                    if (no90 and eb >= 2) or (poses and eb):
                        continue
                    nc += W_BEND * eb
                nst = (ni, nj, nd, k, ((max(sc[0] - 1, 0), min(sc[1] + 1, CAP)) if not bend else (RT, 1)) if no90 else (0, 0))
                if nc < best.get(nst, math.inf) - 1e-12:
                    best[nst] = nc; prev[nst] = st
                    heapq.heappush(pq, (nc + h(ni, nj), nc, nst))
        if goal is None:
            return None, npop
        path = [goal]
        while path[-1] in prev:
            path.append(prev[path[-1]])
        return path[::-1], npop

    # a PAIR's two ends are its END CONNECTORS (pairs.end_legs): from its tips, two legs to a POSE on the grid where the
    # pair router takes over, heading along a router direction -- the pair step lays these legs as they are and runs
    # the pair router from the pose (no end search of its own), so the plan and the router share one end. Each end's
    # candidates are the band's cells ahead of its tips, each heading within 90 degrees of the escape, whose legs
    # (and the router's own straight onto the pose) clear other nets' copper, the other leg's stub and what is
    # placed; the body is searched pose to pose, the shortest ends first
    ends_out = None
    if no90:
        cands = [pair_end_cands(n, 0, W, (a_out[0], a_out[1])), pair_end_cands(n, 1, W, (-a_in[0], -a_in[1]))]
        combos = sorted(((c0[0] + c1[0], x0, x1) for x0, c0 in enumerate(cands[0][:12]) for x1, c1 in enumerate(cands[1][:12])))
        path = None
        for _tot, x0, x1 in combos:
            c0, c1 = cands[0][x0], cands[1][x1]
            path, npop = search((c0[1][0] - i0, c0[1][1] - j0), c0[2], (c1[1][0] - i0, c1[1][1] - j0), (c1[2] + 4) % 8,
                                poses=True)
            if path is not None:
                ends_out = [c0[3], c1[3]]
                break
        if path is None:
            return None, (f'no end connector with a body between them ({len(cands[0])} x {len(cands[1])} candidate '
                          f'poses, {len(combos)} tried)')
    else:
        path, npop = search(S, d0, E, dN)
    if path is None:
        return None, f'no path in its band ({npop} states searched)'
    cross_out = None
    if cross:
        for (i, j, d, k, _sc), (i2, j2, d2, k2, _s2) in zip(path, path[1:]):
            if (i, j) == (i2, j2) and k2 == k + 1:
                cross_out = XO[(i, j, d)]
                break
    pts, vias = [], []
    for (i, j, d, k, _sc) in path:
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
    return (merged, vias, {'ends': ends_out, 'cross': cross_out} if no90 else None), f'{npop} states'


# ------------------------------------------------------------------ one lane at a time
order = [n for n in M if n in prs] + [n for n in M if n not in prs]
DONE = set()
res = {'lanes': {}, 'vias': [], 'conflicts': [], 'pairs': sorted(n for n in M if n in prs),
       'rules': {'grid': g, 'track': TW, 'clear': CL, 'lane_min': bd.LANE_MIN,
                 'pair_turn_steps': _pairs.turn_straight_steps(cfg), 'pair_via_steps': _pairs.via_straight_steps(cfg)}}
t_all = time.time()
def place(n, out):
    pieces, vias = out[0], out[1]
    extra = (out[2] if len(out) > 2 else None) or {}
    ends, cross = extra.get('ends'), extra.get('cross')
    # a pair with END CONNECTORS: its body runs pose to pose (its first and last pieces join its tips' midpoints to
    # them, no copper); its copper is the body's two legs and the end legs -- and, crossed, the crossover's legs and
    # barrels in place of the body's legs between the crossover's poses
    body = pieces[1:-1] if ends else pieces
    for (a_, b_, L) in body:
        PLACED.append((n, L, a_, b_))
    if n in prs:
        parts = list(_pairs.cut_span(body, cross['poses'][0], cross['poses'][1])) if cross else [body]
        for part in parts:
            for lg in bd._pair_legs([(a_, b_, L) for (a_, b_, L) in part], HALF):
                (a_, b_, L) = lg[:3]
                LEGS.append((n, L, tuple(map(float, a_)), tuple(map(float, b_))))
        for e_ in (ends or []):
            for pts in e_['legs']:
                for a_, b_ in zip(pts, pts[1:]):
                    LEGS.append((n, e_['layer'], tuple(map(float, a_)), tuple(map(float, b_))))
        for k_, v_ in ((cross or {}).get('legs') or {}).items():
            for pts, L in v_:
                for a_, b_ in zip(pts, pts[1:]):
                    LEGS.append((n, L, tuple(map(float, a_)), tuple(map(float, b_))))
    if cross:
        for (x_, y_, _k) in cross['vias']:
            PVIAS.append((n, x_, y_))
    else:
        for v in vias:
            for b_ in (_pairs.dive_barrels(v, pieces, VX[n]) if n in prs else [v]):
                PVIAS.append((n, b_[0], b_[1]))
    res['lanes'][n] = {'xy': [list(pieces[0][0])] + [list(p[1]) for p in pieces],
                       'pieces': [[a_[0], a_[1], b_[0], b_[1], L] for (a_, b_, L) in pieces], 'vias': [list(v) for v in vias]}
    if ends:
        res['lanes'][n]['ends'] = ends
    if cross:
        res['lanes'][n]['cross'] = json.loads(json.dumps(cross))


def lift(m):
    """take lane m's copper off the board (a sweep lays it again)"""
    PLACED[:] = [e_ for e_ in PLACED if e_[0] != m]
    LEGS[:] = [e_ for e_ in LEGS if e_[0] != m]
    PVIAS[:] = [e_ for e_ in PVIAS if e_[0] != m]
    res['lanes'].pop(m, None)


failed = {}
folded = {}
for n in [n for n in order if n in HELD]:
    place(n, ([((p_[0], p_[1]), (p_[2], p_[3]), p_[4]) for p_ in plan['lanes'][n]['pieces']], list(LANE[n]['vias']),
              {'ends': plan['lanes'][n].get('ends'), 'cross': plan['lanes'][n].get('cross')}))
    log(f'  {n:7s} held as the plan lays it')
lay = [n for n in order if n not in HELD and (n in prs or not PAIRS_ONLY)]
for n in lay:
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
    # a lane the first pass could not lay is tried again against the others' REAL copper: a share is the room a lane
    # not yet laid might need, and where a gap holds just one row a turn can close it (SDQ0 at SA4's share)
    relaid = []
    for n in [n for n in lay if n in failed]:
        out, why = route(n)
        if out is None:
            out, why2 = route(n, strict=False)
            if out is not None:
                folded[n] = why
        if out is not None:
            place(n, out)
            del failed[n]
            relaid.append(n)
    if relaid:
        log(f'  sweep {sw + 1}: laid against the placed copper: {", ".join(relaid)}')
    nb0 = sum(len(L_['pieces']) for L_ in res['lanes'].values())
    for n in lay:
        if n not in res['lanes'] or n in failed:
            continue
        keep = res['lanes'][n]
        old = ([(tuple(p_[0:2]), tuple(p_[2:4]), p_[4]) for p_ in keep['pieces']], [tuple(v) for v in keep['vias']])
        lift(n)
        out, why = route(n, strict=n not in folded)
        place(n, out if out is not None else old)
    nb1 = sum(len(L_['pieces']) for L_ in res['lanes'].values())
    log(f'  sweep {sw + 1}: pieces {nb0} -> {nb1}')
    if nb1 >= nb0 and not relaid:
        break
for n, why in failed.items():
    res['conflicts'].append({'frame': 'snap', 'xy': list(map(float, LANE[n]['pts'][0])), 'lanes': [n],
                             'charged': [{'kind': 'snap', 'text': f'{n}: {why}', 'short': 1.0, 'lanes': [n],
                                          'xy': list(map(float, LANE[n]['pts'][0]))}], 'hard': []})
if PAIRS_ONLY:
    # the singles as the smooth plan has them, the pairs held: the plan the polish fits the singles into
    for n in M:
        if n not in res['lanes'] and n not in failed:
            res['lanes'][n] = {'xy': plan['lanes'][n]['xy'], 'pieces': plan['lanes'][n]['pieces'],
                               'vias': [list(v) for v in LANE[n]['vias']]}
    res['held'] = sorted(n for n in res['lanes'] if n in prs)
    for k_ in ('flips', 'cuts', 'vcuts', 'paid'):
        if k_ in plan:
            res[k_] = plan[k_]
res['vias'] = [[n, v[0], v[1]] for n, L_ in res['lanes'].items() for v in L_['vias']]
res['tdir'] = {n: [list(end_dirs(n)[0]), [-v for v in end_dirs(n)[1]], list(LANE[n]['pts'][0]), list(LANE[n]['pts'][-1])]
               for n in M}
res['failed'] = bool(res['conflicts'])
res['folded'] = sorted(folded)
json.dump(res, open(OUT, 'w'))
log(f'snap: {len([n for n in lay if n in res["lanes"]])}/{len(lay)} lanes laid' + (f' (+ {len(HELD)} held)' if HELD else '')
    + (' -- the pairs only' if PAIRS_ONLY else '') + f', {len(res["vias"])} vias, {time.time() - t_all:.0f} s -> {OUT}')
if folded:
    log(f'snap: {len(folded)} lane(s) with no approach within 90 degrees of a stub, laid folding: {", ".join(sorted(folded))}')
if res['conflicts']:
    log(f'SNAP FAILED: {len(res["conflicts"])} lane(s) could not be laid: {", ".join(c["lanes"][0] for c in res["conflicts"])}')
    sys.exit(3)

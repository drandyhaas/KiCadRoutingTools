"""whole_ctx.py -- the bench every whole_* tool plans, and the braid's plan of it.

The whole-route tools read one bench from the environment, like the rest of the
chain: BENCH (the board, fanned out: teeth and berths laid), NETS (N1,N2,.. or
@FILE) and DEST (the destination part's reference). Under the chain's own plan
environment -- PLAN_PAGES=1 PLAN_JUDGE=count PLAN_JUDGE_LEN=lane BRAID_PAIRS=1
PLAN_PAIRS=1 BRAID_EXACT_PAGES=0 PLAN_PAGES_SIDERS=2 -- plan() returns the
corridors exactly as braid.run and plan_audit plan them.

install(ctx, corridor, geo) puts a whole-route plan (a geometry, a polish or a snap: per-lane board polylines with
per-piece layers, via sites) in place of the braid's own, through the things the router and plan_audit read from a
plan: virtual_of (a lane's reserved lines on their layers; a pair's two legs), virtual_vias_of (its planned via sites),
band_of (a tube round its own lines on each layer, both layers round its via sites; a snapped lane's just its own grid
line), layer_profile (its layer runs) and lane_xy (its search window); every lane a page lane (nothing weaves). The
audit (whole_audit) and the router (route_lanes --plan) install the same plan the same way."""
import collections
import contextlib
import io
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
os.chdir(HERE)

import plan_audit as pa  # noqa: E402


def bench():
    """(board, nets, dest) from the environment, or a usage stop"""
    miss = [k for k in ('BENCH', 'NETS', 'DEST') if not os.environ.get(k)]
    if miss:
        raise SystemExit(f'set {", ".join(miss)}: the bench board, its nets (N1,N2,.. or @FILE) and the '
                         f'destination part')
    return os.environ['BENCH'], pa.read_nets(os.environ['NETS']), os.environ['DEST']


def stub_dir(ctx, net, pt, layer):
    """the direction of net's stub's LAST segment at its free end pt ON THE LANE'S LAYER, pointing out of the stub (the
    way a lane leaves that end); None when the stub reaches pt on the other layer -- it ends in a via there, and the
    lane lands on the via and continues no line (SDQM1's F remnant at its berth is not its stub)"""
    tw = ctx.cfg.track_width
    nid = ctx.byname[net][0]
    if any(v_.net_id == nid and math.hypot(v_.x - pt[0], v_.y - pt[1]) < tw / 2 for v_ in ctx.base_vias):
        return None
    best = None
    for s_ in ctx.base_segments:
        if s_.net_id != nid or s_.layer != layer:
            continue
        for (ax, ay), (bx_, by_) in (((s_.start_x, s_.start_y), (s_.end_x, s_.end_y)),
                                     ((s_.end_x, s_.end_y), (s_.start_x, s_.start_y))):
            d_ = math.hypot(bx_ - pt[0], by_ - pt[1])
            L_ = math.hypot(bx_ - ax, by_ - ay)
            if L_ > 1e-6 and (best is None or d_ < best[0]):
                best = (d_, ((bx_ - ax) / L_, (by_ - ay) / L_))
    return best[1] if best is not None and best[0] < tw / 2 else None


def plan(quiet=True):
    """(ctx, corridors): the braid's plan of the bench"""
    board, nets, dest = bench()
    if quiet:
        with contextlib.redirect_stdout(io.StringIO()):
            ctx, cs, _logs = pa.plan(board, nets, dest)
    else:
        ctx, cs, _logs = pa.plan(board, nets, dest)
    return ctx, cs


def _seg_dist(X, Y, p, q):
    dx, dy = q[0] - p[0], q[1] - p[1]
    l2 = dx * dx + dy * dy
    if l2 < 1e-18:
        return np.hypot(X - p[0], Y - p[1])
    t = np.clip(((X - p[0]) * dx + (Y - p[1]) * dy) / l2, 0.0, 1.0)
    return np.hypot(X - p[0] - t * dx, Y - p[1] - t * dy)


def _simplify(pts, tol):
    """Douglas-Peucker: the fewest vertices within tol of the polyline (ends kept)"""
    if len(pts) < 3:
        return list(pts)
    a, b = np.asarray(pts[0]), np.asarray(pts[-1])
    P = np.asarray(pts[1:-1])
    d = _seg_dist(P[:, 0], P[:, 1], a, b)
    k = int(np.argmax(d))
    if d[k] <= tol:
        return [pts[0], pts[-1]]
    left = _simplify(pts[:k + 2], tol)
    return left[:-1] + _simplify(pts[k + 1:], tol)


def install(ctx, c, geo):
    import braid as bd
    import pairs as _pairs
    TUBE = bd.TRACK + bd.CLEAR           # a lane's band: a track and a clearance either side of its own line
    VZONE = bd.VIA_NEED                  # both layers open a via's room round its via site
    prs = getattr(ctx, 'pairs', {}) or {}
    half = _pairs.pitch(bd.TRACK) / 2
    P = {n: [((a, b), (cx, dy), L) for a, b, cx, dy, L in v['pieces']] for n, v in geo['lanes'].items()}
    V = collections.defaultdict(list)
    for n, x, y in geo['vias']:
        V[n].append((float(x), float(y)))

    def virtual_of(unrouted):
        out = []
        for om in unrouted:
            if om in P:
                out.extend(bd._pair_legs(P[om], half) if om in prs else P[om])
        return out

    def virtual_vias_of(unrouted):
        return [p for om in unrouted for p in V.get(om, ())]

    # a SNAPPED lane (every piece but its two terminal joins on the router's grid: whole_snap) is handed a band just
    # wide enough to hold it: its own line within half a grid step, a grid step and a half round each end (the cell
    # the router starts from), its via cells. A snapped plan is legal on the grid as it stands, and a lane free to
    # roam a track and a clearance either side takes the next lane's row: all at once, 39 of 48 lanes kept their
    # band so, 43 with the singles' bands narrowed (their copper within 0.02 of the plan, from 0.23). A PAIR's band
    # also opens a grid step either side along its end run (the pose the router launches from rounds onto the grid
    # from the tips' midpoint) and its approach legs' end cells (the pair step checks those against the band)
    g = ctx.cfg.grid_step
    snapped = {n for n, v in geo['lanes'].items()
               if all(pa._on_grid(np.array(p_[:2], float), np.array(p_[2:4], float), g) for p_ in v['pieces'][1:-1])}
    dirs = (getattr(ctx, 'tooth_dir', {}) or {}, getattr(ctx, 'stub_dir', {}) or {})

    def narrow(nm, slack, open_layers):
        segs, vias = P.get(nm, []), V.get(nm, [])
        w = g / 2 + slack
        ends = [segs[0][0], segs[-1][1]]
        legs, runs = [], []
        if nm in prs and nm in getattr(ctx, 'pair_ends', {}):
            for k_, tips in enumerate(ctx.pair_ends[nm]):
                d = dirs[k_].get(nm)
                mid = ((tips[0][0] + tips[1][0]) / 2, (tips[0][1] + tips[1][1]) / 2)
                runs.append((ends[k_], _pairs.end_run(ctx.cfg, tips) + g))
                if d is not None:
                    dl = float(np.hypot(*d))
                    u, nn = (d[0] / dl, d[1] / dl), (-d[1] / dl, d[0] / dl)
                    A = _pairs.approach_len(ctx.cfg)
                    c_ = (mid[0] + u[0] * A, mid[1] + u[1] * A)
                    legs += [(c_[0] + sg * nn[0] * half, c_[1] + sg * nn[1] * half) for sg in (1, -1)]

        def band(xs, ys, L):
            X, Y = np.meshgrid(np.asarray(xs, float), np.asarray(ys, float), indexing='ij')
            ok = np.zeros(X.shape, dtype=bool)
            near_end = [np.hypot(X - e[0], Y - e[1]) <= r_ for e, r_ in runs]
            for (p, q, L_) in segs:
                if not open_layers and L_ != L:
                    continue
                d_ = _seg_dist(X, Y, p, q)
                ok |= d_ <= w + 1e-9
                for ne in near_end:
                    ok |= ne & (d_ <= g + slack + 1e-9)
            for e in ends:
                ok |= np.hypot(X - e[0], Y - e[1]) <= 1.5 * g + slack + 1e-9
            for (vx, vy) in vias:
                ok |= np.hypot(X - vx, Y - vy) <= w + 1e-9
            for (lx, ly) in legs:
                ok |= np.hypot(X - lx, Y - ly) <= g + slack + 1e-9
            return ok
        return band

    def band_of(nm, slack=0.0, open_layers=False):
        if nm in snapped:
            return narrow(nm, slack, open_layers)
        segs, vias = P.get(nm, []), V.get(nm, [])
        w = TUBE + slack + (half if nm in prs else 0.0)

        def band(xs, ys, L):
            xs, ys = np.asarray(xs, float), np.asarray(ys, float)
            ok = np.zeros((len(xs), len(ys)), dtype=bool)
            for (p, q, L_) in segs:
                if not open_layers and L_ != L:
                    continue
                i0, i1 = np.searchsorted(xs, min(p[0], q[0]) - w), np.searchsorted(xs, max(p[0], q[0]) + w)
                j0, j1 = np.searchsorted(ys, min(p[1], q[1]) - w), np.searchsorted(ys, max(p[1], q[1]) + w)
                if i1 <= i0 or j1 <= j0:
                    continue
                X, Y = np.meshgrid(xs[i0:i1], ys[j0:j1], indexing='ij')
                ok[i0:i1, j0:j1] |= _seg_dist(X, Y, p, q) <= w
            r = VZONE + slack
            for (vx, vy) in vias:
                i0, i1 = np.searchsorted(xs, vx - r), np.searchsorted(xs, vx + r)
                j0, j1 = np.searchsorted(ys, vy - r), np.searchsorted(ys, vy + r)
                if i1 > i0 and j1 > j0:
                    X, Y = np.meshgrid(xs[i0:i1], ys[j0:j1], indexing='ij')
                    ok[i0:i1, j0:j1] |= np.hypot(X - vx, Y - vy) <= r
            return ok
        return band

    def layer_profile(nm):
        """the lane's own layer runs, [(spine s where each starts, layer)] -- what a lane's layer changes are counted
        from (route_lanes' plan vias, plan_audit near)"""
        out = []
        for i, p_ in enumerate(geo['lanes'].get(nm, {}).get('pieces', [])):
            if i == 0 or p_[4] != geo['lanes'][nm]['pieces'][i - 1][4]:
                s_, _o = c.spine.project(np.array([p_[0]]), np.array([p_[1]]))
                out.append((float(np.asarray(s_).ravel()[0]), p_[4]))
        return out

    c.virtual_of = virtual_of
    c._virtual_of_plain = lambda unrouted: [pc for om in unrouted for pc in P.get(om, ())]   # a pair's centreline
    c.layer_profile = layer_profile
    c.virtual_vias_of = virtual_vias_of
    c.band_of = band_of
    for n, v in geo['lanes'].items():
        # the lane's search window and the pair step's connector points read RUNS off lane_xy ("about a millimetre
        # along the run leaving the end"); a polyline sampled every 0.1 mm has none, so it is simplified to real runs
        # (Douglas-Peucker, half a grid step -- far inside the band); the reservations and bands keep the exact geometry
        c.lane_xy[n] = _simplify([tuple(map(float, p)) for p in v['xy']], ctx.cfg.grid_step / 2)
    sc = getattr(c, 'sched_cur', None)
    if sc is not None:
        for n in c.members:
            if sc.page.get(n) is None:
                sc.page[n] = 0
    c._geo = geo
    return c

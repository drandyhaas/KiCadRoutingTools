"""whole_audit.py -- install a whole-route geometry (per-lane board polylines with per-piece layers, via sites: from
whole_geo, whole_polish or whole_snap) into a planned corridor, through the four things the router and plan_audit read from a plan: virtual_of (a lane's reserved lines on
their layers; a pair's two legs), virtual_vias_of (its planned via sites), band_of (a tube round its own lines on each
layer, both layers round its via sites) and lane_xy (its search window); every lane a page lane (nothing weaves).

usage as a driver: whole_audit.py GEO.json [checks]   -- plans the bench (whole_ctx: BENCH / NETS / DEST) as
plan_audit does, installs, runs plan_audit's checks (default: pitch dives static shape bands swim). whole_gate.py
reads the output."""
import sys, collections, json
import numpy as np


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

    def band_of(nm, slack=0.0, open_layers=False):
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

    c.virtual_of = virtual_of
    c._virtual_of_plain = lambda unrouted: [pc for om in unrouted for pc in P.get(om, ())]   # a pair's centreline
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


if __name__ == '__main__':
    import whole_ctx
    import plan_audit as pa
    geo = json.load(open(sys.argv[1]))
    checks = sys.argv[2].split(',') if len(sys.argv) > 2 else ['pitch', 'dives', 'static', 'shape', 'bands', 'swim']
    ctx, cs = whole_ctx.plan()
    install(ctx, cs[0], geo)
    cs = cs[:1]
    if 'pitch' in checks:
        pa.check_pitch(ctx, cs)
    if 'dives' in checks:
        pa.check_dives(ctx, cs)
    if 'static' in checks:
        pa.check_static(ctx, cs)
    if 'shape' in checks:
        pa.check_shape(ctx, cs)
    if 'bands' in checks:
        pa.check_bands(ctx, cs)
    if 'swim' in checks:
        pa.check_swim(ctx, cs)

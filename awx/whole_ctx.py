"""whole_ctx.py -- the bench every whole_* tool plans, and the braid's plan of it.

The whole-route tools read one bench from the environment, like the rest of the
chain: BENCH (the board, fanned out: teeth and berths laid), NETS (N1,N2,.. or
@FILE) and DEST (the destination part's reference). Under the braid's plan
environment the planning reads -- BRAID_PAIRS=1 BRAID_EXACT_PAGES=0
PLAN_PAGES_SIDERS=2 (whole_loop.sh sets it) -- plan() returns the corridors
exactly as braid.run and plan_audit plan them.

install(ctx, corridor, geo) puts a whole-route plan (a geometry, a polish or a snap: per-lane board polylines with
per-piece layers, via sites) in place of the braid's own, through the things the router and plan_audit read from a
plan: virtual_of (a lane's reserved lines on their layers; a pair's two legs), virtual_vias_of (its planned via sites),
band_of (a tube round its own lines on each layer, both layers round its via sites; a snapped lane's just its own grid
line), layer_profile (its layer runs) and lane_xy (its search window); every lane a page lane (nothing weaves). The
audit (whole_audit) and the router (route_lanes --plan) install the same plan the same way."""
import collections
import contextlib
import hashlib
import importlib
import io
import json
import marshal
import math
import os
import pickle
import sys
import types

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
    """(ctx, corridors): the braid's plan of the bench -- planned once, and SAVED: every whole_* stage plans the same
    bench (4.5 s of each, a fifth of a loop), so the plan is kept under tmp/ctx_cache, keyed as a stage is
    (stage_cache: the environment, which names the bench and its nets, by content) and restored while every file the
    planning read -- the board, its siblings, every module loaded -- is unchanged and every file it looked for and did
    not find is still absent, as a stage is. Only with STAGE_CACHE=1 (a harness redoing the bench); without it every
    stage plans the bench itself."""
    board, nets, dest = bench()
    import stage_cache as sc
    if not sc.enabled():
        return _guard(_plan(board, nets, dest, quiet), nets, dest)
    key = hashlib.sha256(json.dumps({'py': [sys.version, sys.executable], 'env': sc.env_key(STAGE_VARS)},
                                    sort_keys=True).encode()).hexdigest()[:24]
    pk, mt = os.path.join(CTX_CACHE, key + '.pkl'), os.path.join(CTX_CACHE, key + '.json')
    if os.path.isfile(mt) and os.path.isfile(pk) and sc.still(json.load(open(mt))):
        with open(pk, 'rb') as f_:
            return _guard(pickle.load(f_), nets, dest)
    with sc.recording() as rec:
        out = _plan(board, nets, dest, quiet)
    ev = sc.evidence(rec)
    if ev is not None:          # (None: a file it read changed while it planned -- not saved)
        os.makedirs(CTX_CACHE, exist_ok=True)
        if os.path.exists(mt):  # the old meta out first: a half-written entry is never restored
            os.remove(mt)
        with open(pk + '.tmp', 'wb') as f_:
            _Pickler(f_, protocol=pickle.HIGHEST_PROTOCOL).dump(out)
        os.replace(pk + '.tmp', pk)
        json.dump(ev, open(mt + '.tmp', 'w'), indent=0)
        os.replace(mt + '.tmp', mt)
    return _guard(out, nets, dest)


def _guard(out, nets, dest):
    """the plan, on a bench the whole route is written for -- else a stop that says why: the CANONICAL FRAME (the run's
    source-to-destination direction along +x, so the trunk arrives at the destination's west face and the rings run
    north and south of it: flow_frame.py turns a board into it) and TWO copper layers (a lane's layer is one bit)"""
    ctx, _cs = out
    import flow_frame
    k, _cx, _cy = flow_frame.quarter_of(ctx.pcb, dest, set(nets))
    if k != 0:
        raise SystemExit(f'whole route: the bench is not in the canonical frame (its source-to-destination direction '
                         f'is a quarter turn {k} from +x) -- turn it with flow_frame.py first')
    cu = list(ctx.pcb.board_info.copper_layers)
    if sorted(cu) != ['B.Cu', 'F.Cu']:
        raise SystemExit(f'whole route: the bench has copper layers {cu} -- the whole route plans two (F.Cu, B.Cu)')
    return out


def _plan(board, nets, dest, quiet):
    if quiet:
        with contextlib.redirect_stdout(io.StringIO()):
            ctx, cs, _logs = pa.plan(board, nets, dest)
    else:
        ctx, cs, _logs = pa.plan(board, nets, dest)
    return ctx, cs


# ---- saving the plan: pickle, with the functions pickle cannot name (the braid's closures, a lambda) saved BY VALUE --
# their code, and their cells filled once the function exists, so a closure that refers back to itself or to what
# holds it comes back whole. The plan fills only caches as it goes (the braid's obstacle models, the taut memo's
# shards: pure functions of their keys, rebuilt when asked), so a restored plan is the plan.
CTX_CACHE = os.path.join(HERE, 'tmp', 'ctx_cache')
# what the loop hands one stage and not the next, read only by whole_solve and whole_geo once the bench is planned:
# not in the plan's key, or each stage would plan the same bench again
STAGE_VARS = ('HINT', 'CUTS', 'HIST', 'GEO_FLIPS_FROM', 'SEED_FLIPS', 'SEED_CUTS', 'SEED_HIST', 'WHOLE_SOLVE_BATCHES')


class _NoCell:
    """an empty closure cell, in a saved function"""


def _fn_new(code, module, name, ncells):
    return types.FunctionType(marshal.loads(code), importlib.import_module(module).__dict__, name, None,
                              tuple(types.CellType() for _ in range(ncells)))


def _fn_fill(fn, state):
    defaults, kwdefaults, cells, fdict, qualname = state
    fn.__defaults__, fn.__kwdefaults__, fn.__qualname__ = defaults, kwdefaults, qualname
    for c_, v_ in zip(fn.__closure__ or (), cells):
        if v_ is not _NoCell:
            c_.cell_contents = v_
    fn.__dict__.update(fdict)


class _Pickler(pickle.Pickler):
    def reducer_override(self, obj):
        if isinstance(obj, types.FunctionType) and '<' in obj.__qualname__:
            cells = []
            for c_ in obj.__closure__ or ():
                try:
                    cells.append(c_.cell_contents)
                except ValueError:
                    cells.append(_NoCell)
            return (_fn_new, (marshal.dumps(obj.__code__), obj.__module__, obj.__name__, len(obj.__closure__ or ())),
                    (obj.__defaults__, obj.__kwdefaults__, tuple(cells), dict(obj.__dict__), obj.__qualname__),
                    None, None, _fn_fill)
        return NotImplemented


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

    # a pair with END CONNECTORS (whole_snap: its legs from the tips to a pose on the grid, which the pair step lays
    # as they are): its copper is its body's two legs (pose to pose; the first and last pieces join the tips'
    # midpoints to the poses and are no copper) and the end legs, exact
    ENDS = {n: v['ends'] for n, v in geo['lanes'].items() if v.get('ends')}
    CROSS = {n: v['cross'] for n, v in geo['lanes'].items() if v.get('cross')}
    END_LEGS = {n: [((a_[0], a_[1]), (b_[0], b_[1]), e_['layer']) for e_ in es for pts in e_['legs']
                    for a_, b_ in zip(pts, pts[1:])] for n, es in ENDS.items()}
    # ...and, crossed (an opposite-hands pair, whole_snap), the crossover's legs and barrels in place of the body's
    # legs between the crossover's poses
    for n, xo in CROSS.items():
        END_LEGS.setdefault(n, []).extend(((a_[0], a_[1]), (b_[0], b_[1]), L_) for v_ in xo['legs'].values()
                                          for pts, L_ in v_ for a_, b_ in zip(pts, pts[1:]))

    def body_parts(om):
        body = P[om][1:-1] if om in ENDS else P[om]
        if om not in CROSS:
            return [body]
        return list(_pairs.cut_span(body, CROSS[om]['poses'][0], CROSS[om]['poses'][1]))

    def virtual_of(unrouted):
        out = []
        for om in unrouted:
            if om not in P:
                continue
            if om in prs:
                for part in body_parts(om):
                    out.extend(bd._pair_legs(part, half))
                out.extend(END_LEGS.get(om, []))
            else:
                out.extend(P[om])
        return out

    def virtual_vias_of(unrouted):
        out = []
        for om in unrouted:
            xo = CROSS.get(om)
            for p in V.get(om, ()):
                if xo is not None and math.hypot(p[0] - xo['at'][0], p[1] - xo['at'][1]) < 1e-6:
                    out.extend((vx, vy) for vx, vy, _k in xo['vias'])    # the crossover's two barrels
                else:
                    out.append(p)
        return out

    # a SNAPPED lane (every piece but its two terminal joins on the router's grid: whole_snap) is handed a band just
    # wide enough to hold it: its own line within half a grid step, a grid step and a half round each end (the cell
    # the router starts from), its via cells. A snapped plan is legal on the grid as it stands, and a lane free to
    # roam a track and a clearance either side takes the next lane's row: all at once, 39 of 48 lanes kept their
    # band so, 43 with the singles' bands narrowed (their copper within 0.02 of the plan, from 0.23). A PAIR's band
    # runs pose to pose (its end connectors and a crossover are laid as drawn): its body, its poses, and the end cells
    # of the legs it takes over from (the pair step checks those against the band)
    g = ctx.cfg.grid_step
    snapped = {n for n, v in geo['lanes'].items()                  # (a lane of its two joins alone is not snapped)
               if len(v['pieces']) > 2
               and all(pa._on_grid(np.array(p_[:2], float), np.array(p_[2:4], float), g) for p_ in v['pieces'][1:-1])}

    def narrow(nm, slack, open_layers):
        segs, vias = P.get(nm, []), V.get(nm, [])
        w = g / 2 + slack
        ends = [segs[0][0], segs[-1][1]]
        legs = []
        if nm in ENDS:
            # a pair with END CONNECTORS: its body, its poses, its legs' end cells (the pair step checks those) and the
            # router's own run from the legs' ends onto each pose (it checks that line on this map)
            segs = segs[1:-1] + [(((e_['handover'][0][0] + e_['handover'][1][0]) / 2,
                                   (e_['handover'][0][1] + e_['handover'][1][1]) / 2), tuple(e_['pose']), e_['layer'])
                                 for e_ in ENDS[nm]]
            ends = [tuple(e_['pose']) for e_ in ENDS[nm]]
            legs = [tuple(h_) for e_ in ENDS[nm] for h_ in e_['handover']]
            if nm in CROSS:
                # the crossover's poses and its handover points, the pair router's ends either side of it
                xo = CROSS[nm]
                ends += [tuple(p_) for p_ in xo['poses']]
                legs += [tuple(xo['entry']['P']), tuple(xo['entry']['N']), tuple(xo['exit']['P']), tuple(xo['exit']['N'])]

        def band(xs, ys, L):
            X, Y = np.meshgrid(np.asarray(xs, float), np.asarray(ys, float), indexing='ij')
            ok = np.zeros(X.shape, dtype=bool)
            for (p, q, L_) in segs:
                if not open_layers and L_ != L:
                    continue
                d_ = _seg_dist(X, Y, p, q)
                ok |= d_ <= w + 1e-9
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

    c.end_legs_of = lambda nm: list(END_LEGS.get(nm, []))
    c.cross_of = lambda nm: CROSS.get(nm)
    c.ends_of = lambda nm: ENDS.get(nm)
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

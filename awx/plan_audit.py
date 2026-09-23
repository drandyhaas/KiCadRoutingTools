"""plan_audit.py -- audits of a braid PLAN, before anything is routed.

Every corridor is planned exactly as braid.run plans it (braid.plan_corridors,
under the caller's environment: BRAID_BRANCH, ...), and the
plan is checked against the rules the router will apply to it. A plan that
fails one of these hands the router a lane it cannot route in band, so each
failure here is a refusal found without paying for the route.

usage: plan_audit.py CHECK --board B --nets N1,N2,..|@FILE [--dest REF] [--png OUT]

  pitch   two lanes' reservations (virtual_of: exactly what the router stamps
          for a lane not routed yet) nearer than track/2 + clearance + track/2
          on one layer: a planned line the router cannot sit on. Also lists
          lanes with a reserved piece off the board's bounds.
  dives   every planned via site (virtual_vias_of: exit corners, split legs, a
          swimmer's reserved diamonds) against static copper of other
          nets (pads, teeth, berth stubs), other lanes' planned lines and other
          lanes' via sites.
  bands   each lane's band -- the one route_lane hands the router -- against
          its planned line: the planned length no layer's band covers, and
          whether the band connects tooth to berth at all (a flood over band
          cells, no obstacles). --only N1,N2 restricts; --png DIR renders.
  near NET X,Y [R]   every other lane's reservation within R (0.45) of (X, Y),
          and the plan facts of NET and of those lanes (page, layers, slots,
          layer profile).

--png OUT (pitch: a file; bands: a directory) draws the finding over the plan.
"""
import argparse
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
import braid as bd  # noqa: E402


def read_nets(arg):
    """A comma list, or @FILE holding one."""
    if arg.startswith('@'):
        arg = open(arg[1:]).read()
    return [n.strip() for n in arg.replace('\n', ',').split(',') if n.strip()]


def plan(board, nets, dest, quiet=True):
    """(ctx, corridors, log lines): the braid's plan of `nets`, every corridor
    planned as braid.run plans it, the board's copper reset to the base."""
    logs = []
    cm = contextlib.redirect_stdout(io.StringIO()) if quiet else contextlib.nullcontext()
    with cm:
        ctx, corridors = bd.plan_corridors(board, nets, dest, logs.append)
    return ctx, corridors, logs


def dseg(x, y, p, q):
    dx, dy = q[0] - p[0], q[1] - p[1]
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 < 1e-12 else max(0.0, min(1.0, ((x - p[0]) * dx + (y - p[1]) * dy) / l2))
    return math.hypot(x - p[0] - t * dx, y - p[1] - t * dy)


def _samples(segs, L, step=0.025):
    out = []
    for p, q, L_ in segs:
        if L_ != L:
            continue
        n = max(1, int(np.hypot(*(q - p)) / step))
        t = np.linspace(0, 1, n + 1)[:, None]
        out.append(p + (q - p) * t)
    return np.concatenate(out) if out else np.zeros((0, 2))


def _dist_to_segs(P, segs, L):
    S = [(p, q) for p, q, L_ in segs if L_ == L]
    if not S or not len(P):
        return np.full(len(P), 1e9)
    A_ = np.array([p for p, q in S])
    B_ = np.array([q for p, q in S])
    best = np.full(len(P), 1e9)
    for i in range(0, len(A_), 256):
        a, b = A_[i:i + 256], B_[i:i + 256]
        d = b - a
        l2 = np.maximum((d ** 2).sum(1), 1e-12)
        t = np.clip(((P[:, None, :] - a[None]) * d[None]).sum(2) / l2[None], 0, 1)
        proj = a[None] + t[..., None] * d[None]
        best = np.minimum(best, np.hypot(*(P[:, None, :] - proj).transpose(2, 0, 1)).min(1))
    return best


def check_pitch(ctx, corridors, png=None):
    TW, CL = ctx.cfg.track_width, ctx.cfg.clearance
    block = TW / 2 + CL + TW / 2
    bx0, by0, bx1, by1 = ctx.pcb.board_info.board_bounds
    hits, V_all = [], {}
    for c in corridors:
        s_h = getattr(c, 'handoff_s', None)
        V = {nm: [(np.array(p, float), np.array(q, float), L) for (p, q, L) in c.virtual_of([nm])]
             for nm in c.members}
        V_all.update(V)
        off = collections.Counter()
        for nm, v in V.items():
            for p, q, _L in v:
                if any(not (bx0 <= x <= bx1 and by0 <= y <= by1)
                       for x, y in (p + (q - p) * t for t in np.linspace(0, 1, 11))):
                    off[nm] += 1
        if off:
            print(f'PITCH corridor {c.idx}: lanes with a reserved piece off the board bounds: {dict(off)}')
        mem = list(c.members)
        for i, nm in enumerate(mem):
            for L in ctx.cfg.layers:
                P = _samples(V[nm], L)
                if not len(P):
                    continue
                for om in mem[i + 1:]:
                    d = _dist_to_segs(P, V[om], L)
                    bad = d < block - 0.005
                    if bad.any():
                        k = int(np.argmin(d))
                        x, y = P[k]
                        S, _O = c.spine.project(np.array([x]), np.array([y]))
                        S = float(np.asarray(S).ravel()[0])
                        where = 'branch' if s_h is not None and S >= s_h else 'trunk'
                        hits.append((float(d[k]), nm, om, L, (float(x), float(y)), S, where,
                                     float(bad.sum()) * 0.025, c.idx))
    hits.sort()
    for (d, a, b, L, xy, S, where, ln, ci) in hits:
        print(f'PITCH {a:7s} {b:7s} {L[0]} min {d:.3f}  over {ln:4.2f} mm  at ({xy[0]:.2f},{xy[1]:.2f})  '
              f'corridor {ci} s {S:6.2f} {where}')
    print(f'PITCH {len(hits)} pair(s) within {block:.3f} mm on one layer: '
          f'{dict(collections.Counter(h[6] for h in hits))}')
    if png:
        from route_render import BoardRenderer
        from kicad_parser import Segment as _S
        allp = [p for v in V_all.values() for (p, q, L) in v] + [q for v in V_all.values() for (p, q, L) in v]
        view = (min(p[0] for p in allp) - 1, min(p[1] for p in allp) - 1,
                max(p[0] for p in allp) + 1, max(p[1] for p in allp) + 1)
        r = BoardRenderer(ctx.pcb, size=2000, supersample=2, show_zones=False, view=view, layer_alpha=70)

        def ov(dr, rr):
            for v in V_all.values():
                rr._draw_segments(dr, [_S(p[0], p[1], q[0], q[1], 0.04, L, 0) for p, q, L in v if L == 'F.Cu'],
                                  color=(230, 200, 60))
                rr._draw_segments(dr, [_S(p[0], p[1], q[0], q[1], 0.04, L, 0) for p, q, L in v if L != 'F.Cu'],
                                  color=(80, 160, 255))
            for h in hits:
                xy = h[4]
                rr._draw_segments(dr, [_S(xy[0] - 0.12, xy[1], xy[0] + 0.12, xy[1], 0.24, 'F.Cu', 0)],
                                  color=(255, 0, 0))
        r.frame(segments=[], vias=[], overlays=[ov],
                label=f'reservations F yellow, B blue | red: two lanes within {block:.3f} mm on one layer '
                      f'({len(hits)} pairs)').save(png)
        print('wrote', png)
    return hits


def _pad_edge(x, y, pd):
    if pd.shape == 'circle':
        return math.hypot(x - pd.global_x, y - pd.global_y) - pd.size_x / 2
    dx = abs(x - pd.global_x) - pd.size_x / 2
    dy = abs(y - pd.global_y) - pd.size_y / 2
    return math.hypot(max(dx, 0), max(dy, 0)) + min(max(dx, dy), 0)


def check_dives(ctx, corridors, show_all=False):
    cfg = ctx.cfg
    VR, CL, TW = cfg.via_size / 2, cfg.clearance, cfg.track_width
    need_static, need_lane, need_via = VR + CL, VR + CL + TW / 2, 2 * VR + CL
    pairs = getattr(ctx, 'pairs', {}) or {}
    name = lambda i: (ctx.pcb.nets[i].name.split('/')[-1] if i in ctx.pcb.nets else str(i))
    pads = [(fp.reference, pd) for fp in ctx.pcb.footprints.values() for pd in fp.pads
            if pd.pad_type != 'np_thru_hole' and any(L.endswith('.Cu') for L in pd.layers)]
    fail = collections.Counter()
    n_sites = 0
    for c in corridors:
        M = list(c.members)
        own = {nm: {ctx.byname[nm][0]} | {ctx.byname[leg][0] for leg in pairs.get(nm, ()) if leg in ctx.byname}
               for nm in M}
        sites = {nm: [tuple(p) for p in c.virtual_vias_of([nm])] for nm in M}
        lines = {nm: c.virtual_of([nm]) for nm in M}
        for nm in M:
            for (x, y) in sites[nm]:
                n_sites += 1
                hits = []
                near = lambda px, py: abs(px - x) < 2 and abs(py - y) < 2
                st = min([(_pad_edge(x, y, pd), f'pad {ref}.{pd.pad_number} {name(pd.net_id)}')
                          for ref, pd in pads if pd.net_id not in own[nm] and near(pd.global_x, pd.global_y)]
                         + [(dseg(x, y, (s.start_x, s.start_y), (s.end_x, s.end_y)) - s.width / 2,
                             f'{s.layer[0]} copper {name(s.net_id)}')
                            for s in ctx.base_segments if s.net_id not in own[nm] and near(s.start_x, s.start_y)]
                         + [(math.hypot(v.x - x, v.y - y) - v.size / 2, f'via {name(v.net_id)}')
                            for v in ctx.base_vias if v.net_id not in own[nm] and near(v.x, v.y)],
                         default=(9, ''))
                if st[0] < need_static - 1e-6:
                    hits.append(f'static {st[0]:+.3f}/{need_static:.3f} ({st[1]})')
                    fail['static'] += 1
                ln = min(((dseg(x, y, p, q), om, L) for om in M if om != nm for (p, q, L) in lines[om]),
                         default=(9, '', ''))
                if ln[0] < need_lane - 1e-6:
                    hits.append(f'lane {ln[0]:.3f}/{need_lane:.3f} ({ln[1]} {ln[2][0]})')
                    fail['lane'] += 1
                vv = min(((math.hypot(px - x, py - y), om) for om in M if om != nm for (px, py) in sites[om]),
                         default=(9, ''))
                if vv[0] < need_via - 1e-6:
                    hits.append(f'via {vv[0]:.3f}/{need_via:.3f} ({vv[1]})')
                    fail['via'] += 1
                if hits or show_all:
                    print(f'DIVE {nm:7s} ({x:7.2f},{y:6.2f})  ' + ('; '.join(hits) if hits else 'ok'))
    print(f'DIVE {n_sites} planned via sites; failing: {dict(fail)}')
    return fail


def router_band(c, nm):
    """(kind, band): the band route_lane hands the router for `nm`'s search
    (a swimmer is searched free: None)."""
    if c.sched_cur.page.get(nm) is None:
        return 'free', None
    return 'page', c.band_of(nm)


def _flood(ok, xs, ys, layers, a, aL, b, bL, G):
    ij = lambda p: (int(round((p[0] - xs[0]) / G)), int(round((p[1] - ys[0]) / G)))
    li = {L: k for k, L in enumerate(layers)}
    open_at = lambda c_: 0 <= c_[0] < len(xs) and 0 <= c_[1] < len(ys) and ok[layers[c_[2]]][c_[0], c_[1]]

    def seed(c_):
        if open_at(c_):
            return c_
        r = int(0.3 / G)
        best = None
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                cc = (c_[0] + di, c_[1] + dj, c_[2])
                if open_at(cc) and (best is None or di * di + dj * dj < best[0]):
                    best = (di * di + dj * dj, cc)
        return best[1] if best else None
    s, t = seed(ij(a) + (li[aL],)), seed(ij(b) + (li[bL],))
    if s is None:
        return 'TOOTH-OUT'
    if t is None:
        return 'BERTH-OUT'
    seen = {s}
    q = collections.deque([s])
    while q:
        c_ = q.popleft()
        if c_ == t:
            return 'connected'
        i, j, L = c_
        for n in [(i + di, j + dj, L) for di in (-1, 0, 1) for dj in (-1, 0, 1) if di or dj] + [(i, j, 1 - L)]:
            if n not in seen and open_at(n):
                seen.add(n)
                q.append(n)
    return 'BROKEN'


def check_bands(ctx, corridors, only=None, png_dir=None, G=0.05):
    layers = list(ctx.cfg.layers)
    tot_out = 0.0
    for c in corridors:
        for nm in c.members:
            if only and nm not in only:
                continue
            if nm not in c.lane_xy:
                print(f'BAND {nm:7s} no planned line')
                continue
            kind, band = router_band(c, nm)
            if band is None:
                print(f'BAND {nm:7s} {kind:5s} no band (free)')
                continue
            w = list(c.lane_xy[nm])
            a, b = c.teeth[nm], c.stubs[nm]
            P = w + [a, b]
            x0, x1 = min(p[0] for p in P) - 1.2, max(p[0] for p in P) + 1.2
            y0, y1 = min(p[1] for p in P) - 1.2, max(p[1] for p in P) + 1.2
            xs, ys = np.arange(x0, x1, G), np.arange(y0, y1, G)
            ok = {L: np.asarray(band(xs, ys, L), dtype=bool) for L in layers}
            pts = []
            for p, q in zip(w, w[1:]):
                n = max(1, int(math.hypot(q[0] - p[0], q[1] - p[1]) / G))
                pts += [(p[0] + (q[0] - p[0]) * k / n, p[1] + (q[1] - p[1]) * k / n) for k in range(n)]
            runs, cur, s, out, prev = [], None, 0.0, 0.0, pts[0]
            for (x, y) in pts:
                ds = math.hypot(x - prev[0], y - prev[1])
                s += ds
                prev = (x, y)
                i, j = int(round((x - xs[0]) / G)), int(round((y - ys[0]) / G))
                st = ''.join(L[0] if 0 <= i < len(xs) and 0 <= j < len(ys) and ok[L][i, j] else '-'
                             for L in layers)
                if st == '-' * len(layers):
                    out += ds
                if cur is None or cur[0] != st:
                    if cur:
                        runs.append(cur)
                    cur = [st, s, s]
                else:
                    cur[2] = s
            runs.append(cur)
            tot_out += out
            fl = _flood(ok, xs, ys, layers, a, ctx.tooth_layer[nm], b, ctx.dest_layer[nm], G)
            gaps = [r for r in runs if r[0] == '-' * len(layers) and r[2] - r[1] > 0.05]
            print(f'BAND {nm:7s} {kind:5s} plan {s:5.1f} mm  outside {out:4.1f} mm  {fl:9s} '
                  + ' '.join(f'out:{r[1]:.1f}-{r[2]:.1f}' for r in gaps))
            if png_dir:
                from route_render import BoardRenderer
                from kicad_parser import Segment as _S
                os.makedirs(png_dir, exist_ok=True)
                r = BoardRenderer(ctx.pcb, size=1400, supersample=2, show_zones=False,
                                  view=(x0, y0, x1, y1), layer_alpha=110)

                def ov(d, rr, ok=ok, xs=xs, ys=ys, w=w, a=a, b=b):
                    both = ok[layers[0]] & ok[layers[1]]
                    for L, col in zip(layers, ((0, 150, 60), (170, 40, 170))):
                        bi, bj = np.nonzero(ok[L] & ~both)
                        rr._draw_segments(d, [_S(xs[i], ys[j], xs[i] + 0.001, ys[j], 0.03, 'F.Cu', 0)
                                              for i, j in zip(bi, bj)], color=col)
                    bi, bj = np.nonzero(both)
                    rr._draw_segments(d, [_S(xs[i], ys[j], xs[i] + 0.001, ys[j], 0.03, 'F.Cu', 0)
                                          for i, j in zip(bi, bj)], color=(90, 110, 160))
                    rr._draw_segments(d, [_S(p[0], p[1], q[0], q[1], 0.05, 'F.Cu', 0) for p, q in zip(w, w[1:])],
                                      color=(255, 255, 255))
                    for p, col in ((a, (255, 80, 80)), (b, (80, 160, 255))):
                        rr._draw_segments(d, [_S(p[0] - 0.08, p[1], p[0] + 0.08, p[1], 0.16, 'F.Cu', 0)], color=col)
                r.frame(segments=[], vias=[], overlays=[ov],
                        label=f'{nm} {kind} band: F only green, B only magenta, both blue | plan white | '
                              f'tooth red, berth blue | {fl}').save(os.path.join(png_dir, f'{nm}.png'))
    print(f'BAND total planned length outside its band: {tot_out:.1f} mm')
    return tot_out


def show_near(ctx, corridors, nm, P, R=0.45):
    c = next((c for c in corridors if nm in c.members), None)
    if c is None:
        print(f'{nm} is not planned')
        return
    rows, seen = [], set()
    for d, om, L in sorted((round(dseg(P[0], P[1], p, q), 3), om, L[0]) for om in c.members if om != nm
                           for p, q, L in c.virtual_of([om]) if dseg(P[0], P[1], p, q) < R):
        if (om, L) not in seen:
            seen.add((om, L))
            rows.append((d, om, L))
    print(f'near ({P[0]}, {P[1]}) within {R}: ' + ', '.join(f'{om} {L} {d}' for d, om, L in rows))
    sc = c.sched_cur
    for om in [nm] + sorted({r[1] for r in rows}):
        try:
            prof = [(round(s, 2), L[0]) for s, L in c.layer_profile(om)]
        except Exception:
            prof = None
        print(f'  {om:7s} page {sc.page.get(om)} tooth {ctx.tooth_layer[om][0]} dest {ctx.dest_layer[om][0]} '
              f'st {tuple(round(v, 2) for v in c.st[om])} launch {c.launch_o[om]:.2f} '
              f'target {c.target_o[om]:.2f} profile {prof}')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('check', choices=('pitch', 'dives', 'bands', 'near'))
    ap.add_argument('args', nargs='*', help='near: NET X,Y [R]')
    ap.add_argument('--board', required=True)
    ap.add_argument('--nets', required=True, help='N1,N2,.. or @FILE')
    ap.add_argument('--dest', default='DU1')
    ap.add_argument('--only', default='', help='bands: these lanes only')
    ap.add_argument('--png', default='')
    ap.add_argument('--all', action='store_true', help='dives: print every site, not only the failing ones')
    a = ap.parse_args(argv)
    ctx, corridors, _logs = plan(a.board, read_nets(a.nets), a.dest)
    if a.check == 'pitch':
        check_pitch(ctx, corridors, a.png or None)
    elif a.check == 'dives':
        check_dives(ctx, corridors, a.all)
    elif a.check == 'bands':
        check_bands(ctx, corridors, set(a.only.split(',')) - {''} or None, a.png or None)
    else:
        if len(a.args) < 2:
            ap.error('near: NET X,Y [R]')
        show_near(ctx, corridors, a.args[0], tuple(map(float, a.args[1].split(','))),
                  float(a.args[2]) if len(a.args) > 2 else 0.45)


if __name__ == '__main__':
    main()

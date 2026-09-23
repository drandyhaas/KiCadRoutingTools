"""one_net.py -- route chosen lanes of a braid plan ONE AT A TIME, in band.

The bench is planned exactly as braid.run plans it (plan_audit.plan, the
caller's environment), then ONLY the named lanes are routed, in the braid's
own attempt-0 order (pairs first, the pages in target order, the
swimmers), each through its corridor's own route_lane (a
pair through the braid's pair step) against every other lane's reservation --
no rescue, no last call, no econ re-lay. What the router does with ONE lane
in its band is then visible apart from what the rest of the run does to it.

usage: one_net.py NETS|all --board B --nets N1,..|@FILE [--dest REF]
                  [--mode alone|seq] [--png DIR] [--probe X,Y;..|stops]
                  [--box X0,Y0,X1,Y1] [--viacheck]

  --mode alone  the copper is reset before each lane (each alone against the plan)
         seq    the lanes accumulate in order (the kept attempt, these lanes only)

Per lane: IN BAND / REFUSED, vias (routed vs the plan's layer changes), length
vs the plan, the copper's distance from the planned line (max, and the share
within 0.3 mm); on a refusal the router's split probe frontiers (forward from
the tooth, backward from the berth: iterations and how far along the plan
each got, and where it stopped).
  --png DIR   one render per lane: band F green / B magenta / both blue, plan
              white, reservations F yellow / B orange, reserved vias orange
              rings, the lane's copper (F red, B cyan), frontiers fwd cyan /
              bwd pink. --box fixes the view and draws the reservations at
              their blocking width.
  --probe     on a refusal, the refused call's own obstacle map around each
              point (or at the frontiers' stops): where a via or a track on
              each layer is legal, and the nearest obstacles.
  --viacheck  each routed via's distance to the reservations and copper.
"""
import argparse
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
import connect as cn  # noqa: E402
from plan_audit import plan, read_nets, dseg  # noqa: E402


def along(w, P):
    """(position along the polyline w, distance from it) of every point of P,
    and w's length."""
    w = np.asarray(w, float)
    P = np.asarray(P, float)
    if len(w) < 2 or not len(P):
        return np.zeros(len(P)), np.full(len(P), 1e9), 0.0
    seg = w[1:] - w[:-1]
    L = np.hypot(seg[:, 0], seg[:, 1])
    cum = np.concatenate([[0], np.cumsum(L)])
    best = np.full(len(P), 1e9)
    at = np.zeros(len(P))
    for i in range(len(seg)):
        t = np.clip(((P[:, 0] - w[i, 0]) * seg[i, 0] + (P[:, 1] - w[i, 1]) * seg[i, 1])
                    / max(L[i] ** 2, 1e-12), 0, 1)
        d = np.hypot(w[i, 0] + t * seg[i, 0] - P[:, 0], w[i, 1] + t * seg[i, 1] - P[:, 1])
        m = d < best
        best[m] = d[m]
        at[m] = cum[i] + t[m] * L[i]
    return at, best, cum[-1]


class Recorder:
    """Every connect call's inputs, and the router's result and obstacle map
    for it (the split probe frontiers live in the result)."""

    def __init__(self):
        self.calls = []
        self._rno, self._connect = cn.route_net_with_obstacles, cn.connect
        cn.route_net_with_obstacles, cn.connect = self.rno, self.connect

    def rno(self, *a_, **k_):
        r_ = self._rno(*a_, **k_)
        if self.calls:
            self.calls[-1].update(rno=r_, win=a_[0],
                                  obs=a_[3] if len(a_) > 3 else k_.get('obstacles'))
        return r_

    def connect(self, pcb, net_id, a, a_layer, b, b_layer, cfg, **kw):
        self.calls.append(dict(a=a, aL=a_layer, b=b, bL=b_layer, kw=kw, cfg=cfg, nid=net_id))
        res = self._connect(pcb, net_id, a, a_layer, b, b_layer, cfg, **kw)
        self.calls[-1]['ok'] = res is not None
        return res


def attempt0_order(ctx, corridors):
    """[(corridor, lane)] in the braid's attempt-0 order."""
    out = []
    for c in corridors:
        sched = c.sched_cur
        sw_ = [nm for nm in c.members if sched.page.get(nm) is None]
        ti = {nm: i for i, nm in enumerate(c.target)}
        order = (sorted((nm for nm in c.target if nm not in sw_), key=lambda nm: ti[nm])
                 + sorted(sw_, key=lambda nm: -abs(c.launch_o[nm] - c.target_o[nm])))
        prs = [nm for nm in order if nm in getattr(ctx, 'pairs', {})]
        out += [(c, nm) for nm in prs + [nm for nm in order if nm not in prs]]
    return out


def probe_at(ctx, cl, x, y, R=0.35):
    """Around (x, y), in the refused call's own obstacle map: where a via or a
    track on each layer is legal, and the obstacles nearest the point."""
    obs, cfg = cl.get('obs'), cl['cfg']
    if obs is None:
        print('    probe: no obstacle map captured')
        return
    g = cfg.grid_step
    gx0, gy0, n = int(round(x / g)), int(round(y / g)), int(R / g)
    print(f'    probe ({x:.2f},{y:.2f}) +-{R} mm, step {2 * g:.3f}: V via ok, b both tracks, '
          f'F/B one layer, . none (rows = y down)')
    for dj in range(-n, n + 1, 2):
        row = ''
        for di in range(-n, n + 1, 2):
            gx, gy = gx0 + di, gy0 + dj
            v = not obs.is_via_blocked(gx, gy)
            f, b = not obs.is_blocked(gx, gy, 0), not obs.is_blocked(gx, gy, 1)
            row += 'V' if v and f and b else 'b' if f and b else 'F' if f else 'B' if b else '.'
        print('      ' + row)
    name = lambda i: (ctx.pcb.nets[i].name.split('/')[-1] if i in ctx.pcb.nets else str(i))
    win, kw, nid = cl.get('win'), cl['kw'], cl.get('nid')
    near = [(dseg(x, y, p, q) - cfg.track_width / 2, 'reserved', L[0]) for (p, q, L) in (kw.get('virtual') or [])]
    near += [(math.hypot(x - vx, y - vy) - cfg.via_size / 2, 'reserved via', '*')
             for (vx, vy) in (kw.get('virtual_vias') or [])]
    if win is not None:
        near += [(dseg(x, y, (s.start_x, s.start_y), (s.end_x, s.end_y)) - s.width / 2, f'seg {name(s.net_id)}',
                  s.layer[0]) for s in win.segments if s.net_id not in (nid, cn.VIRTUAL_NET)]
        near += [(math.hypot(x - v.x, y - v.y) - v.size / 2, f'via {name(v.net_id)}', '*')
                 for v in win.vias if v.net_id not in (nid, cn.VIRTUAL_NET)]
        for fp in win.footprints.values():
            for pd in fp.pads:
                d_ = math.hypot(x - pd.global_x, y - pd.global_y) - max(pd.size_x, pd.size_y) / 2
                if pd.net_id != nid and d_ < 0.6:
                    near.append((d_, f'pad {fp.reference}.{pd.pad_number} {name(pd.net_id)}',
                                 ''.join(L[0] for L in pd.layers if L.endswith('.Cu'))[:2]))
    near.sort()
    print('    nearest obstacles (to their copper edge): '
          + ', '.join(f'{d_:.3f} {w} {L}' for d_, w, L in near[:8]))


def render(ctx, c, nm, cl, segs_nm, vias_nm, out, box=None):
    from route_render import BoardRenderer
    from kicad_parser import Segment as _S
    kw, cfg = cl['kw'], cl['cfg']
    w = list(c.lane_xy.get(nm) or [])
    P = w + [cl['a'], cl['b']]
    x0, y0, x1, y1 = box or (min(p[0] for p in P) - 1.0, min(p[1] for p in P) - 1.0,
                             max(p[0] for p in P) + 1.0, max(p[1] for p in P) + 1.0)
    band = kw.get('band')
    G = 0.05 if (x1 - x0) > 6 else 0.025
    xs, ys = np.arange(x0, x1, G), np.arange(y0, y1, G)
    ok = ({L: np.asarray(band(xs, ys, L), dtype=bool) for L in ('F.Cu', 'B.Cu')}
          if callable(band) else None)
    g = cfg.grid_step
    dot = lambda x, y, w_=0.035: _S(x, y, x + 0.001, y, w_, 'F.Cu', 0)

    def ov(d, rr):
        if ok is not None:
            both = ok['F.Cu'] & ok['B.Cu']
            for L, col in (('F.Cu', (0, 120, 50)), ('B.Cu', (130, 30, 130))):
                bi, bj = np.nonzero(ok[L] & ~both)
                rr._draw_segments(d, [dot(xs[i], ys[j]) for i, j in zip(bi, bj)], color=col)
            bi, bj = np.nonzero(both)
            rr._draw_segments(d, [dot(xs[i], ys[j]) for i, j in zip(bi, bj)], color=(70, 90, 140))
        virt = kw.get('virtual') or []
        if box:
            bw = cfg.track_width + 2 * cfg.clearance
            for L, col in (('F.Cu', (90, 80, 10)), ('B.Cu', (100, 50, 10))):
                rr._draw_segments(d, [_S(p[0], p[1], q[0], q[1], bw, L, 0) for (p, q, L_) in virt if L_ == L],
                                  color=col)
        for L, col in (('F.Cu', (230, 200, 0)), ('B.Cu', (255, 120, 0))):
            rr._draw_segments(d, [_S(p[0], p[1], q[0], q[1], 0.04, L, 0) for (p, q, L_) in virt if L_ == L],
                              color=col)
        for (vx, vy) in (kw.get('virtual_vias') or []):
            ring = [(vx + 0.2 * math.cos(2 * math.pi * k / 12), vy + 0.2 * math.sin(2 * math.pi * k / 12))
                    for k in range(13)]
            rr._draw_segments(d, [_S(p[0], p[1], q[0], q[1], 0.03, 'F.Cu', 0) for p, q in zip(ring, ring[1:])],
                              color=(255, 120, 0))
        rr._draw_segments(d, [_S(p[0], p[1], q[0], q[1], 0.03, 'F.Cu', 0) for p, q in zip(w, w[1:])],
                          color=(255, 255, 255))
        rno = cl.get('rno') or {}
        for key_, col in (('blocked_cells_forward', (0, 255, 255)), ('blocked_cells_backward', (255, 110, 200))):
            rr._draw_segments(d, [dot(cc[0] * g, cc[1] * g, 0.03) for cc in list(rno.get(key_) or [])[:20000]],
                              color=col)
        for L, col in (('F.Cu', (255, 60, 60)), ('B.Cu', (60, 220, 255))):
            rr._draw_segments(d, [_S(s.start_x, s.start_y, s.end_x, s.end_y, max(s.width, 0.08), s.layer, 0)
                                  for s in segs_nm if s.layer == L], color=col)
        for v in vias_nm:
            rr._draw_segments(d, [_S(v.x - 0.08, v.y, v.x + 0.08, v.y, 0.2, 'F.Cu', 0)], color=(255, 255, 255))
        for p, col in ((cl['a'], (255, 80, 80)), (cl['b'], (80, 160, 255))):
            rr._draw_segments(d, [_S(p[0] - 0.07, p[1], p[0] + 0.07, p[1], 0.14, 'F.Cu', 0)], color=col)
    nid = ctx.byname[nm][0]
    r = BoardRenderer(ctx.pcb, size=1500, supersample=2, show_zones=False, view=(x0, y0, x1, y1), layer_alpha=90)
    img = r.frame(segments=[s for s in ctx.pcb.segments if s.net_id != nid
                            and x0 - 1 < s.start_x < x1 + 1 and y0 - 1 < s.start_y < y1 + 1],
                  vias=[v for v in ctx.pcb.vias if v.net_id != nid and x0 < v.x < x1 and y0 < v.y < y1],
                  overlays=[ov], label=f'{nm}: band F green B magenta both blue | plan white | reserved F yellow '
                                       f'B orange, vias rings | lane F red B cyan | frontier fwd cyan bwd pink')
    img.save(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('only', help='the lanes to route (N1,N2,..) or all')
    ap.add_argument('--board', required=True)
    ap.add_argument('--nets', required=True, help='the bench nets: N1,N2,.. or @FILE')
    ap.add_argument('--dest', default='DU1')
    ap.add_argument('--mode', choices=('alone', 'seq'), default='alone')
    ap.add_argument('--png', default='', help='a directory: one render per lane')
    ap.add_argument('--probe', default='', help="X,Y;X,Y;.. or 'stops'")
    ap.add_argument('--box', default='', help='X0,Y0,X1,Y1: the render view')
    ap.add_argument('--viacheck', action='store_true')
    a = ap.parse_args(argv)
    box = tuple(map(float, a.box.split(','))) if a.box else None
    ctx, corridors, logs = plan(a.board, read_nets(a.nets), a.dest)
    base_s, base_v = list(ctx.base_segments), list(ctx.base_vias)
    ctx.pcb.segments, ctx.pcb.vias = list(base_s), list(base_v)
    order = attempt0_order(ctx, corridors)
    chosen = [p for p in order if a.only == 'all' or p[1] in a.only.split(',')]
    miss = set(a.only.split(',')) - {nm for _c, nm in chosen} - {'all'}
    if miss:
        print('not planned:', sorted(miss))
    rec = Recorder()
    if a.png:
        os.makedirs(a.png, exist_ok=True)
    routed = set()
    n_ok = n_free = v_tot = v_plan = 0
    print(f'{a.mode}: {len(chosen)} lane(s), branch={bd.BRANCH}')
    for (c, nm) in chosen:
        if a.mode == 'alone':
            ctx.pcb.segments, ctx.pcb.vias = list(base_s), list(base_v)
            routed = set()
        k0 = len(rec.calls)
        how_ = 'IN BAND ok  '
        n_s, n_v = len(ctx.pcb.segments), len(ctx.pcb.vias)
        try:
            if nm in getattr(ctx, 'pairs', {}):
                # a pair routes FIRST by the braid's own pair step: its ends own
                # the fan-in, only the others' stubs are reserved there
                n_log = len(logs)
                quiet = (contextlib.nullcontext() if os.environ.get('BRAID_PAIR_DEBUG')
                         else contextlib.redirect_stdout(io.StringIO()))
                with quiet:
                    done_ = bd._route_pairs_planned_in_order(ctx, corridors, logs.append, [nm])
                how_ = 'IN BAND ok  '
                for l_ in logs[n_log:]:
                    if ('pair ' + nm) in l_:
                        print('    ' + l_.strip()[:200])
                        if 'routed FIRST' in l_ and 'planned band' not in l_:
                            how_ = 'FREE        '            # a pair off its band: the pair step's own rung
                res = done_.get(nm)
                if res is not None:
                    del ctx.pcb.segments[n_s:]
                    del ctx.pcb.vias[n_v:]
                    ctx.pcb.segments.extend(res[0])
                    ctx.pcb.vias.extend(res[1])
            else:
                unrouted = [om for om in c.members if om != nm and om not in routed]
                res = c.route_lane(nm, c.virtual_of(unrouted), c.virtual_vias_of(unrouted))
        except Exception as ex:
            res = None
            print(f'{nm:7s} ERROR {ex}')
        calls = rec.calls[k0:]
        kind = ('pair' if nm in getattr(ctx, 'pairs', {})
                else 'swim' if c.sched_cur.page.get(nm) is None else 'page')
        exp = max(0, len(c.layer_profile(nm)) - 1)
        w = list(c.lane_xy.get(nm) or [])
        Lp = sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(w, w[1:]))
        if res is not None:
            segs_nm, vias_nm = res
            routed.add(nm)
            n_ok += how_.startswith('IN BAND')
            n_free += not how_.startswith('IN BAND')
            v_tot += len(vias_nm)
            v_plan += exp
            Lr = sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) for s in segs_nm)
            if kind == 'pair':
                Lr /= 2                     # per leg, as the plan's line is
            pts = []
            for s in segs_nm:
                n = max(1, int(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) / 0.05))
                pts += [(s.start_x + (s.end_x - s.start_x) * t / n, s.start_y + (s.end_y - s.start_y) * t / n)
                        for t in range(n + 1)]
            _at, dd, _L = along(w, pts)
            print(f'{nm:7s} {kind:4s} {how_} vias {len(vias_nm)} (plan {exp})  len {Lr:5.1f} '
                  f'(plan {Lp:5.1f})  off-plan max {dd.max():.2f} mm, within 0.3: {100 * (dd < 0.3).mean():3.0f}%  '
                  f'calls {len(calls)}')
            if a.viacheck and calls:
                kw_, cfg_ = calls[-1]['kw'], calls[-1]['cfg']
                for v in vias_nm:
                    near = sorted((dseg(v.x, v.y, p, q), L[0]) for (p, q, L) in (kw_.get('virtual') or []))[:3]
                    vv = sorted(math.hypot(v.x - x, v.y - y) for (x, y) in (kw_.get('virtual_vias') or []))[:1]
                    print(f'    via ({v.x:.3f},{v.y:.3f}): nearest reserved lines '
                          f'{[(round(d, 3), L) for d, L in near]} (need >= '
                          f'{v.size / 2 + cfg_.clearance + cfg_.track_width / 2:.3f}); reserved via '
                          f'{[round(d, 3) for d in vv]}')
        else:
            del ctx.pcb.segments[n_s:]
            del ctx.pcb.vias[n_v:]
            segs_nm, vias_nm = [], []
            line = f'{nm:7s} {kind:4s} REFUSED      (plan {exp} vias, {Lp:5.1f} mm)  calls {len(calls)}'
            cl = calls[-1] if calls else None
            if cl and cl.get('rno'):
                rno, g = cl['rno'], cl['cfg'].grid_step
                parts = []
                for lab, key_, itk in (('fwd', 'blocked_cells_forward', 'iterations_forward'),
                                       ('bwd', 'blocked_cells_backward', 'iterations_backward')):
                    cells = list(rno.get(key_) or [])
                    if cells:
                        sv, _d, _Lw = along(w or [cl['a'], cl['b']], [(cc[0] * g, cc[1] * g) for cc in cells])
                        k_ = int(np.argmax(sv)) if lab == 'fwd' else int(np.argmin(sv))
                        stop = (cells[k_][0] * g, cells[k_][1] * g)
                        cl.setdefault('stops', []).append(stop)
                        parts.append(f'{lab} {rno.get(itk)} it, s {sv.min():.1f}..{sv.max():.1f} '
                                     f'(stops at {stop[0]:.2f},{stop[1]:.2f})')
                    else:
                        parts.append(f'{lab} {rno.get(itk)} it')
                line += f'  [probe {cl["cfg"].max_probe_iterations}: ' + ' | '.join(parts) + ']'
            print(line)
            if a.probe and cl:
                pts_ = (cl.get('stops', []) if a.probe == 'stops'
                        else [tuple(map(float, pt.split(','))) for pt in a.probe.split(';')])
                for pt in pts_:
                    probe_at(ctx, cl, *pt)
        if a.png and calls:
            render(ctx, c, nm, calls[-1], segs_nm, vias_nm, os.path.join(a.png, f'{nm}.png'), box)
    print(f'SUMMARY {a.mode}: {n_ok}/{len(chosen)} in band' + (f' (+{n_free} free)' if n_free else '')
          + f', {v_tot} vias (plan {v_plan} for those)')


if __name__ == '__main__':
    main()

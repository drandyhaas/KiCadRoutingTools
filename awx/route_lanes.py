"""route_lanes.py -- route chosen lanes of a plan ONE AT A TIME, each in its band.

The bench is planned exactly as braid.run plans it (plan_audit.plan, the
caller's environment), then ONLY the named lanes are routed, in the braid's
own attempt-0 order (pairs first, the pages in target order, the
swimmers), each through its corridor's own route_lane (a
pair through the braid's pair step) against every other lane's reservation --
no rescue, no last call, no econ re-lay. What the router does with ONE lane
in its band is then visible apart from what the rest of the run does to it.

usage: route_lanes.py NETS|all --board B --nets N1,..|@FILE [--dest REF]
                  [--plan PLAN.json] [--mode alone|seq] [--write OUT.kicad_pcb]
                  [--png DIR] [--probe X,Y;..|stops] [--box X0,Y0,X1,Y1] [--viacheck]

  --plan PLAN   a WHOLE-ROUTE plan (whole_snap's), routed in place of the braid's
                plan: the bench read as braid.setup reads it and the plan's
                lanes installed in the whole route's own frame (whole_ctx.lanes,
                no braid corridor) -- its reservations, via sites, bands and
                search windows are what the lanes route against; pairs first,
                then the singles, in the berths' order round the destination
  --mode alone  the copper is reset before each lane (each alone against the plan)
         seq    the lanes accumulate in order (the kept attempt, these lanes only)
  --write OUT   the board the run leaves -- the bench plus every segment and via
                it added (seq: all the lanes together), its project stamped at
                the routed floor -- for check_connected / check_drc: a lane the
                router reports routed is not proof its net connects

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
KRT_TOOL = {'scope': [], 'kind': 'conditional'}   # #937: a research tool (awx), catalogued, shown at no door
import argparse
import contextlib
import io
import json
import math
import os
import awx_settings
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
import braid as bd  # noqa: E402
import connect as cn  # noqa: E402
from plan_audit import plan, read_nets, dseg  # noqa: E402
# WALL_LIFT=1: a ball of the bus's parts on no bus net that the lay WALLED IN (the step's own cage test,
# route_bus.walls_made) has the lanes walling it lifted and routed again round its way kept -- the via site it escapes
# by with them gone, reserved as a lane's line is (lift_walls). 0 = no test
WALL_LIFT = (awx_settings.get('WALL_LIFT') or '1') == '1'


def lift_walls(ctx, laid, routed, board, names, dest, base_s, base_v):
    """The balls of the source and `dest`'s parts on no bus net the lay walled in, against the bench as it came
    (`base_s`, `base_v`): each one's walling lanes (`laid`'s singles) lifted, the ball's way -- the via site its layer
    reaches with their lanes gone and their fanout standing, route_bus.ball_way -- reserved as a line from the ball and
    a via, and they routed again,
    in their order, round it. Kept when every one of them routes and the ball is reached, else all as it was. A two-row
    connector's ball between a neighbour's lane run along the row's outer ends and an along-strip lane between the
    rows was reached from nowhere, and its net shipped open"""
    import copy
    import route_bus as rb
    from coherent_nets import by_key
    byk = by_key(ctx.pcb)
    full = {n: byk[n][1].name for n in names if n in byk}
    short = {v: k for k, v in full.items()}
    refs = sorted(set(getattr(ctx, 'src_ref', {}).values()) | set(dest.split('+')))
    before = copy.copy(ctx.pcb)
    before.segments, before.vias = list(base_s), list(base_v)
    walls = rb.walls_made(before, ctx.pcb, refs, list(full.values()), [full[m] for m in laid if m in full],
                          project=board)
    rung = rb._cage_rung(ctx.pcb)
    for k, who in walls.items():
        lanes = [short[x] for x in who if short.get(x) in laid]
        r, n = k.split('.', 1)
        pad = next(p for p in ctx.pcb.footprints[r].pads if p.pad_number == n)
        if not lanes or len(lanes) < len(who):
            print(f'{k} WALLED IN by {", ".join(who) or "no lane alone"}: not lifted (not every one is a laid single)')
            continue
        fixed = rb._fixed_nets(ctx.pcb, board, full.values())
        # (the way on the board as the lanes are routed again on it: their lane copper gone, their fanout's -- which
        # stays -- still a wall; with the whole net gone, a ball took a site under the walling lane's own escape on an
        # inner layer, which no lane routed again could clear)
        drop_ = {id(o) for m in lanes for o in laid[m][1] + laid[m][2]}
        gone = copy.copy(ctx.pcb)
        gone.segments = [s for s in ctx.pcb.segments if id(s) not in drop_]
        gone.vias = [v for v in ctx.pcb.vias if id(v) not in drop_]
        way = rb.ball_way(gone, pad, fixed, rung)
        if way is None:
            print(f'{k} WALLED IN by {", ".join(lanes)}: no via site reached with them gone either')
            continue
        (sx, sy), home = way
        vline = [((pad.global_x, pad.global_y), (sx, sy), home)] if math.hypot(sx - pad.global_x,
                                                                              sy - pad.global_y) > 1e-6 else []
        keep_s, keep_v = list(ctx.pcb.segments), list(ctx.pcb.vias)
        ctx.pcb.segments = [s for s in keep_s if id(s) not in drop_]
        ctx.pcb.vias = [v for v in keep_v if id(v) not in drop_]
        routed.difference_update(lanes)
        got, ok_, how = {}, True, []
        for m in lanes:
            cm = laid[m][0]
            later = [om for om in cm.members if om != m and om not in routed]
            # (in its band, else in none: a band that hugs a row's ends has no room round a via site off them)
            r_ = None
            for free_ in (False, True):
                try:
                    r_ = cm.route_lane(m, list(cm.virtual_of(later)) + vline,
                                       list(cm.virtual_vias_of(later) or []) + [(sx, sy)], free=free_)
                except Exception:
                    r_ = None
                if r_ is not None:
                    how.append(f'{m} {"free" if free_ else "in band"}')
                    break
            if r_ is None:
                ok_ = False
                break
            got[m] = r_
            routed.add(m)
        if ok_ and not rb._caged_on(ctx.pcb, {r: [n]}, rb._fixed_nets(ctx.pcb, board, full.values()), rung):
            for m in lanes:
                laid[m] = (laid[m][0], list(got[m][0]), list(got[m][1]))
            print(f'{k} WALLED IN by {", ".join(lanes)}: lifted and routed again round its via site '
                  f'({sx:.3f},{sy:.3f}) -- reached ({", ".join(how)})')
        else:
            ctx.pcb.segments, ctx.pcb.vias = keep_s, keep_v
            routed.update(lanes)
            print(f'{k} WALLED IN by {", ".join(lanes)}: ' + ('routed again round its via site, still walled'
                                                             if ok_ else 'not routed again round its via site')
                  + f' ({sx:.3f},{sy:.3f} on {home}) -- left as laid')


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
                    / max(L[i] * L[i], 1e-12), 0, 1)
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
    name = lambda i: (ctx.net_key(ctx.pcb.nets[i].name) if i in ctx.pcb.nets else str(i))
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
    ap.add_argument('--dest', required=True, help="the destination part's reference (several, 'U4+U3': one source, "
                                                  "several ends -- the trunk's first)")
    ap.add_argument('--mode', choices=('alone', 'seq'), default='alone')
    ap.add_argument('--png', default='', help='a directory: one render per lane')
    ap.add_argument('--probe', default='', help="X,Y;X,Y;.. or 'stops'")
    ap.add_argument('--box', default='', help='X0,Y0,X1,Y1: the render view')
    ap.add_argument('--viacheck', action='store_true')
    ap.add_argument('--plan', default='', help='a whole-route plan (JSON) to route instead of the braid\'s')
    ap.add_argument('--write', default='', help='the board the run leaves (.kicad_pcb)')
    a = ap.parse_args(argv)
    for k in ('board', 'plan', 'write', 'png'):          # whole_ctx works from awx/: every path absolute first
        if getattr(a, k):
            setattr(a, k, os.path.abspath(getattr(a, k)))
    if a.nets.startswith('@'):
        a.nets = '@' + os.path.abspath(a.nets[1:])
    box = tuple(map(float, a.box.split(','))) if a.box else None
    if a.plan:
        # a whole-route plan: the bench as the braid's setup reads it, the plan's lanes in the whole route's own frame
        # (whole_ctx.lanes) -- none of the braid's corridors -- routed pairs first, then the singles, each set in the
        # berths' order round the destination
        import whole_ctx
        geo = json.load(open(a.plan))
        ctx, _groups = whole_ctx.bench_with(a.board, read_nets(a.nets), a.dest)
        logs = []
        lanes_ = whole_ctx.lanes(ctx, a.dest, geo, logs.append)
        corridors = [lanes_]
        prs_ = getattr(ctx, 'pairs', {}) or {}
        order = ([(lanes_, nm) for nm in lanes_.order if nm in prs_ and nm in geo['lanes']]
                 + [(lanes_, nm) for nm in lanes_.order if nm not in prs_ and nm in geo['lanes']])
        print(f'installed {os.path.basename(a.plan)}: {len(geo["lanes"])} lanes, {len(geo["vias"])} vias')
    else:
        ctx, corridors, logs = plan(a.board, read_nets(a.nets), a.dest)
        order = attempt0_order(ctx, corridors)
    base_s, base_v = list(ctx.base_segments), list(ctx.base_vias)
    ctx.pcb.segments, ctx.pcb.vias = list(base_s), list(base_v)
    chosen = [p for p in order if a.only == 'all' or p[1] in a.only.split(',')]
    miss = set(a.only.split(',')) - {nm for _c, nm in chosen} - {'all'}
    if miss:
        print('not planned:', sorted(miss))
    rec = Recorder()
    if a.png:
        os.makedirs(a.png, exist_ok=True)
    routed = set()
    n_ok = n_free = v_tot = v_plan = 0
    errors = []                 # lanes whose routing RAISED: a crash, not a routing result
    laid = {}                   # (a plan's singles, seq: each laid lane's corridor and copper -- lift_walls')
    print(f'{a.mode}: {len(chosen)} lane(s), branch={bd.BRANCH}')
    # (a whole-route PLAN: every lane in its own band first -- a pair by its band rungs alone -- and a pair refused there
    # tried in a WIDER band, then FREE of the plan, only after them, round what they laid. Free in its turn, ahead of the
    # singles, a pair took the room the plan gave them: zynq LVDS round 3 with RX_D4's dive laid, RX_D1 routed free
    # 2.8 mm off its line, DATA_CLK and RX_D5 free after it, and 9 singles refused -- 13/24 in band where 22/23 were)
    work = [(c, nm, not a.plan) for (c, nm) in chosen]
    wi = 0
    while wi < len(work):
        c, nm, free_ok = work[wi]
        wi += 1
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
                quiet = (contextlib.nullcontext() if awx_settings.get('BRAID_PAIR_DEBUG')
                         else contextlib.redirect_stdout(io.StringIO()))
                with quiet:
                    # (a pair tried after the others: its band WIDENED too, a pair pitch and two -- round what they
                    # laid, as the free rungs are)
                    done_ = bd._route_pairs_planned_in_order(
                        ctx, corridors, logs.append, [nm], free=free_ok,
                        slacks=(sorted(set(bd.pair_slacks()) | {bd.PP, 2 * bd.PP}) if free_ok and a.plan else None))
                how_ = 'IN BAND ok  '
                for l_ in logs[n_log:]:
                    if ('pair ' + nm) in l_:
                        print('    ' + l_.strip()[:200])
                        if 'routed FIRST' in l_ and 'planned band' not in l_:
                            how_ = 'FREE        '            # a pair off its band: the pair step's own rung
                res = done_.get(nm)
                if res is None and not free_ok:
                    del ctx.pcb.segments[n_s:]
                    del ctx.pcb.vias[n_v:]
                    work.append((c, nm, True))
                    print(f'{nm:7s} pair refused in its band -- tried wider, then free, after the lanes in theirs')
                    continue
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
            errors.append(nm)
            print(f'{nm:7s} ERROR {type(ex).__name__}: {ex}')
        calls = rec.calls[k0:]
        kind = ('pair' if nm in getattr(ctx, 'pairs', {})
                else 'swim' if c.sched_cur.page.get(nm) is None else 'page')
        exp = max(0, len(c.layer_profile(nm)) - 1) * (2 if kind == 'pair' else 1)   # a pair's change: a via per leg
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
        if a.plan and a.mode == 'seq' and kind != 'pair' and res is not None:
            laid[nm] = (c, list(segs_nm), list(vias_nm))
    if a.plan and a.mode == 'seq' and laid and WALL_LIFT:
        lift_walls(ctx, laid, routed, a.board, read_nets(a.nets), a.dest, base_s, base_v)
    print(f'SUMMARY {a.mode}: {n_ok}/{len(chosen)} in band' + (f' (+{n_free} free)' if n_free else '')
          + f', {v_tot} vias (plan {v_plan} for those)' + (f'; {len(errors)} ERROR(S): {errors}' if errors else ''))
    if a.write:
        write(ctx, a.board, a.write, base_s, base_v)
    if errors:
        sys.exit(1)


def write(ctx, board, out, base_s, base_v):
    """the bench plus every segment and via the run added, its project stamped at the routed floor (as braid.run
    stamps its own)"""
    from kicad_writer import add_tracks_and_vias_to_pcb
    from fix_kicad_drc_settings import fix_project_for_output
    from bga_fanout.flip_frame import other_layer
    cfg = ctx.cfg
    old_s, old_v = set(map(id, base_s)), set(map(id, base_v))
    segs = [s_ for s_ in ctx.pcb.segments if id(s_) not in old_s]
    vias = [v_ for v_ in ctx.pcb.vias if id(v_) not in old_v]
    # the copper back in the board's own frame, as braid.run writes it: the braid's setup turns a board of the other
    # chirality over (ctx.M maps a point of the turned board back, and the layers swap), and every lane here was laid
    # on the turned board -- written as laid, it lands mirrored (K15's own fanout: four nets open, a lane over a pad)
    M, OL = (ctx.M, other_layer) if getattr(ctx, 'M', None) is not None else ((lambda x, y: (x, y)), (lambda L: L))
    add_tracks_and_vias_to_pcb(board, out,
                               [dict(start=M(s_.start_x, s_.start_y), end=M(s_.end_x, s_.end_y), width=s_.width,
                                     layer=OL(s_.layer), net_id=s_.net_id) for s_ in segs],
                               # (every via a lane lays is a THROUGH via: written F.Cu to B.Cu whatever layers it
                               # joins -- KiCad's via names its span's two ends, and one written with every routing
                               # layer was a barrel kicad_parser, and so every check here, could not read)
                               [dict(x=M(v_.x, v_.y)[0], y=M(v_.x, v_.y)[1], size=v_.size, drill=v_.drill,
                                     layers=['F.Cu', 'B.Cu'], net_id=v_.net_id) for v_ in vias],
                               net_id_to_name={i: n.name for i, n in ctx.pcb.nets.items()})
    from copy_board import copy_siblings
    copy_siblings(board, out)           # (the project and the .kicad_dru's per-layer rules with it)
    fix_project_for_output(out, board, clearance=cfg.clearance, track_width=cfg.track_width,
                           via_diameter=cfg.via_size, via_drill=cfg.via_drill, verbose=False)
    # (a via laid in a pad or a paste opening declares Type VII, as braid.run's and the fanout's do (#962): a lane's
    # change right at its tooth stood 0.15 mm into its own ball's paste opening on the zynq LVDS bus, and check_drc
    # graded the board's only violation, via-in-paste)
    import ship_vias
    ship_vias.stamp(out, 'route_lanes', print)
    print(f'wrote {out}: {len(segs)} segment(s), {len(vias)} via(s) added, routed at clearance {cfg.clearance}')


if __name__ == '__main__':
    main()

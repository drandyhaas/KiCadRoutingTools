"""wall_probe.py -- track-level wall census for ONE lane of a chain run.

Run from awx/ on a chain tag (`bash chain_k.sh TAG K` first). In the braid's attempt-0
world: intercept the router at the lane's first call, flood-fill the
free cells from its tooth (per layer, 8-connected, inside a box), and
attribute every wall cell on the pocket's boundary to what blocks it --
a VIRTUAL lane (by owner net + layer + which piece), REAL copper (net,
layer), a via, or the BAND alone (no copper within reach). Prints the
pocket size, whether the target is reachable, and the wall census.

usage: wall_probe.py K TAG NET [R=2.0] [--png OUT] [--call last]
"""
import sys, os, json, math
from collections import Counter, deque
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'py_router'))
import braid as te
import connect as cn
from coherent_nets import coherent_nets

K = int(sys.argv[1]); tag = sys.argv[2]; NET = sys.argv[3]
R = float(sys.argv[4]) if len(sys.argv) > 4 and not sys.argv[4].startswith('--') else 2.0
png = None
if '--png' in sys.argv:
    png = sys.argv[sys.argv.index('--png') + 1]
# --call last: run the corridor to completion, record every router call
# for NET, then analyze the LAST CALL's world (first rung: band slack 0.3
# with margin 2.0; and the final free rung at margin 6.0)
CALL = 'first'
if '--call' in sys.argv:
    CALL = sys.argv[sys.argv.index('--call') + 1]
RECORDED = []
names = coherent_nets(K)
fo = f'tmp/{tag}_fo_k{K}.kicad_pcb'
plan = json.load(open(f'tmp/{tag}_fo_k{K}.plan.json'))
logs = []
ctx, groups = te.setup(fo, names, 'DU1', logs.append, plan=plan)
nid = ctx.byname[NET][0]
tooth = ctx.ends[NET][0]
stub = ctx.ends[NET][1]
orig = cn.route_net_with_obstacles
id2name = {v[0]: k for k, v in ctx.byname.items()}


class Done(Exception):
    pass


def seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def make_attrib(cfg, vsegs, rsegs, vvias, rvias, vv_own, Lname_of, window=None, band_fn=None):
    tw = float(cfg.track_width); clr = float(cfg.clearance)
    zseg = tw / 2 + clr + tw / 2
    zvia = float(cfg.via_size) / 2 + clr + tw / 2

    def attrib(px, py, L):
        Lname = Lname_of[L]
        best, who = 1e9, 'band/pad'
        for sg, o in vsegs:
            if sg.layer != Lname:
                continue
            d = seg_dist(px, py, sg.start_x, sg.start_y, sg.end_x, sg.end_y) - zseg
            if d < best:
                best, who = d, f'virtual {o} {Lname[0]}'
        for sg in rsegs:
            if sg.layer != Lname:
                continue
            d = seg_dist(px, py, sg.start_x, sg.start_y, sg.end_x, sg.end_y) - (sg.width / 2 + clr + tw / 2)
            if d < best:
                best, who = d, f'real {id2name.get(sg.net_id, sg.net_id)} {Lname[0]}'
        for v in rvias:
            d = math.hypot(px - v.x, py - v.y) - (v.size / 2 + clr + tw / 2)
            if d < best:
                best, who = d, f'via {id2name.get(v.net_id, v.net_id)}'
        for v in vvias:
            d = math.hypot(px - v.x, py - v.y) - zvia
            if d < best:
                best, who = d, 'virtual via ' + vv_own.get((round(v.x, 4), round(v.y, 4)), '?')
        for fp in getattr(window, 'footprints', {}).values():
            for pd in fp.pads:
                if Lname not in pd.layers and not (pd.drill and pd.drill > 0):
                    continue
                d = math.hypot(px - pd.global_x, py - pd.global_y) - (max(pd.size_x, pd.size_y) / 2 + clr + tw / 2)
                if d < best:
                    best, who = d, f'pad {fp.reference}.{pd.pad_number}'
        if best > 0.03:
            who = 'band' if band_fn is not None and not band_fn(np.array([px]), np.array([py]), Lname)[0, 0] else f'?? nearest {who} at {best:+.2f}'
        return who
    return attrib


def spy(window, net_id, cfg, obstacles, **kw):
    if net_id != nid:
        return orig(window, net_id, cfg, obstacles, **kw)
    cc = [c_ for c_ in corridors if NET in c_.members][0]
    if CALL == 'last':
        x0_, y0_, x1_, y1_ = window.board_info.board_bounds
        RECORDED.append((window, cfg, obstacles, dict(kw), set(cc.out_segs.keys()),
                         f'window {x1_ - x0_:.1f} mm, max_iter {cfg.max_iterations}'))
        return orig(window, net_id, cfg, obstacles, **kw)
    return analyze(window, cfg, obstacles, kw, cc, set(cc.out_segs.keys()), 'first call')


def analyze(window, cfg, obstacles, kw, cc, routed, label):
    print(f'\n===== {NET}: {label}')
    coord = cn.GridCoord(cfg.grid_step)
    G = cfg.grid_step
    # per-owner virtual copper (same call as the braid's, one owner at a time)
    sc = cc.sched_cur
    unrouted = [om for om in cc.members if om != NET and om not in routed]
    own = {}
    for om in unrouted:
        for (p, q, L) in cc.virtual_of([om]):
            own[(round(p[0], 4), round(p[1], 4), round(q[0], 4), round(q[1], 4), L)] = om
    # the reserve pass's pieces, per owner (ends of every unrouted lane,
    # 1.5 mm on the END layers): computed one owner at a time
    allm = set()
    for c_ in corridors:
        allm |= set(c_.members)
    saved = set(ctx.landed)
    for om in sorted(allm):
        if om == NET or om in saved:
            continue
        ctx.landed = allm - {om}
        for (p, q, L) in te.cross_reserve(ctx, NET):
            own[(round(p[0], 4), round(p[1], 4), round(q[0], 4), round(q[1], 4), L)] = 'RESERVE ' + om
    ctx.landed = saved
    vsegs = []
    for sg in window.segments:
        if sg.net_id == cn.VIRTUAL_NET:
            key = (round(sg.start_x, 4), round(sg.start_y, 4), round(sg.end_x, 4), round(sg.end_y, 4), sg.layer)
            vsegs.append((sg, own.get(key, '?')))
    vv_own = {}
    for om in unrouted:
        if om in cc.exit_leg_s:
            p = cc.spine.xy(cc.exit_leg_s[om], cc.exit_block[om])
            vv_own[(round(p[0], 4), round(p[1], 4))] = 'corner ' + om
        for p in getattr(cc, 'hops', {}).get(om, ()):
            vv_own[(round(p[0], 4), round(p[1], 4))] = 'hop ' + om
    sp_ = cc.spine
    print(f'  spine: from {tuple(round(v,3) for v in sp_.xy(0.0, 0.0))} dir s -> {tuple(round(v,3) for v in sp_.xy(1.0, 0.0))}, +o -> {tuple(round(v,3) for v in sp_.xy(0.0, 1.0))}')
    rsegs = [sg for sg in window.segments if sg.net_id != cn.VIRTUAL_NET and sg.net_id != nid]
    vvias = [v for v in window.vias if v.net_id == cn.VIRTUAL_NET]
    rvias = [v for v in window.vias if v.net_id != cn.VIRTUAL_NET and v.net_id != nid]
    print(f'{NET}: tooth {tooth} on {ctx.tooth_layer[NET]}, stub {stub} on {ctx.dest_layer[NET]}; '
          f'page {sc.page.get(NET)}; routed before it: {sorted(routed)}')
    print(f'  unrouted others: {unrouted}')
    print(f'  window virtual segs {len(vsegs)} (owners {Counter(o for _, o in vsegs)})')
    band_fn = None if (sc.page.get(NET) is None or (window.board_info.board_bounds[2] - window.board_info.board_bounds[0]) > 14) else cc.band_of(NET)
    attrib = make_attrib(cfg, vsegs, rsegs, vvias, rvias, vv_own, list(cfg.layers), window, band_fn)
    # CHANNEL PROFILE along the planned path: at each s, the free run of
    # cells across the lane (perpendicular to the spine) that contains the
    # planned centreline, its width, and the blockers just outside it
    ms_ = [p[0] for p in cc.mid[NET]]; mo_ = [p[1] for p in cc.mid[NET]]
    print('  channel profile along the planned lane (F unless said; width of the free run holding the '
          'centreline, else nearest free run; blockers either side):')
    Lp = 0 if ctx.tooth_layer[NET] == 'F.Cu' else 1
    prev = None
    rows = []
    for s_ in np.arange(ms_[0], min(ms_[-1], cc.s1 + 1.5) + 1e-9, 0.05):
        o_c = float(np.interp(s_, ms_, mo_))
        Lhere = Lp
        if not cc.allowed(NET, s_, cfg.layers[Lp]):
            Lhere = 1 - Lp
        ks = list(range(-28, 29))
        free = []
        for k in ks:
            x_, y_ = sp_.xy(s_, o_c + k * G)
            gx_, gy_ = coord.to_grid(x_, y_)
            free.append(not obstacles.is_blocked(gx_, gy_, Lhere))
        # run containing k=0
        i0 = ks.index(0)
        if free[i0]:
            a = i0
            while a - 1 >= 0 and free[a - 1]:
                a -= 1
            b = i0
            while b + 1 < len(ks) and free[b + 1]:
                b += 1
        else:
            # nearest free run
            cand = [i for i in range(len(ks)) if free[i]]
            if not cand:
                a = b = None
            else:
                i1 = min(cand, key=lambda i: abs(i - i0))
                a = b = i1
                while a - 1 >= 0 and free[a - 1]:
                    a -= 1
                while b + 1 < len(ks) and free[b + 1]:
                    b += 1
        if a is None:
            who0 = attrib(*sp_.xy(s_, o_c), Lhere)
            whoN = attrib(*sp_.xy(s_, o_c - 0.3), Lhere)
            whoS = attrib(*sp_.xy(s_, o_c + 0.3), Lhere)
            row = (s_, o_c, Lhere, 0.0, None, None, f'ALL BLOCKED: at line {who0}; -0.3 {whoN}', f'+0.3 {whoS}')
        else:
            w_ = (b - a + 1) * G
            lo_k, hi_k = ks[a], ks[b]
            blk_lo = attrib(*sp_.xy(s_, o_c + (lo_k - 1) * G), Lhere) if a - 1 >= 0 else '(box)'
            blk_hi = attrib(*sp_.xy(s_, o_c + (hi_k + 1) * G), Lhere) if b + 1 < len(ks) else '(box)'
            row = (s_, o_c, Lhere, w_, lo_k * G, hi_k * G, blk_lo, blk_hi)
        rows.append(row)
    minw = min(r[3] for r in rows)
    print(f'    min free width along the path = {minw:.3f} mm')
    last = None
    for r in rows:
        s_, o_c, Lh, w_, lo_, hi_, bl, bh = r
        key = (Lh, bl, bh, w_ < 0.1)
        if key != last or w_ < 0.1:
            x_, y_ = sp_.xy(s_, o_c)
            off = '' if lo_ is None else f'run o[{lo_:+.3f},{hi_:+.3f}]'
            print(f'    s={s_:6.2f} ({x_:.2f},{y_:.2f}) {cfg.layers[Lh][0]} width {w_:.3f} {off:24s} '
                  f'{"-o: " + bl:38s} +o: {bh}')
            last = key
    # FRONTIER: flood the whole window from the tooth on the lane's
    # planned layer (page lanes may not change layer inside the band),
    # project what was reached onto the spine, and name the wall at the
    # farthest s reached -- the closure a box round the tooth cannot see
    wx0, wy0, wx1, wy1 = window.board_info.board_bounds
    gwx0, gwy0 = coord.to_grid(wx0, wy0)
    gwx1, gwy1 = coord.to_grid(wx1, wy1)
    NX, NY = gwx1 - gwx0 + 1, gwy1 - gwy0 + 1
    Lf = 0 if (sc.page.get(NET) or ctx.tooth_layer[NET]) == 'F.Cu' else 1
    blk = np.zeros((NX, NY), dtype=bool)
    for i in range(NX):
        for j in range(NY):
            blk[i, j] = obstacles.is_blocked(gwx0 + i, gwy0 + j, Lf)
    tx_, ty_ = coord.to_grid(*tooth)
    si, sj = tx_ - gwx0, ty_ - gwy0
    seen = np.zeros((NX, NY), dtype=bool)
    if 0 <= si < NX and 0 <= sj < NY:
        q = deque([(si, sj)]); seen[si, sj] = True
        while q:
            i, j = q.popleft()
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    a, b = i + di, j + dj
                    if 0 <= a < NX and 0 <= b < NY and not seen[a, b] and not blk[a, b]:
                        seen[a, b] = True; q.append((a, b))
    ii, jj = np.nonzero(seen)
    xs_r = np.array([coord.to_float(int(gwx0 + i), 0)[0] for i in ii]) if len(ii) else np.array([])
    ys_r = np.array([coord.to_float(0, int(gwy0 + j))[1] for j in jj]) if len(jj) else np.array([])
    tgx, tgy = coord.to_grid(*stub)
    reached_stub = bool(0 <= tgx - gwx0 < NX and 0 <= tgy - gwy0 < NY and seen[tgx - gwx0, tgy - gwy0])
    if len(ii):
        S_r, O_r = sp_.project(xs_r, ys_r)
        s_max = float(S_r.max())
        print(f'  FRONTIER on {cfg.layers[Lf]}: {len(ii)} cells reached from the tooth, farthest s = {s_max:.2f} '
              f'(stub at s {sp_.project_pt(stub)[0]:.2f}); stub cell reached: {reached_stub}')
        # wall cells adjacent to reached cells with s within 0.4 of the frontier
        near = S_r > s_max - 0.4
        wall = Counter(); pts = {}
        for i, j in zip(ii[near], jj[near]):
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    a, b = i + di, j + dj
                    if 0 <= a < NX and 0 <= b < NY and blk[a, b] and not seen[a, b]:
                        px_ = coord.to_float(int(gwx0 + a), 0)[0]; py_ = coord.to_float(0, int(gwy0 + b))[1]
                        who = attrib(px_, py_, Lf)
                        wall[who] += 1
                        pts.setdefault(who, []).append((px_, py_))
        for who, c in wall.most_common(8):
            P = pts[who]
            so_ = [sp_.project_pt(p) for p in P]
            print(f'      frontier wall {c:5d} cells  {who:34s} s {min(v[0] for v in so_):.2f}..{max(v[0] for v in so_):.2f} '
                  f'o {min(v[1] for v in so_):+.2f}..{max(v[1] for v in so_):+.2f}')
    x0, y0 = tooth[0] - R, tooth[1] - R
    n = int(2 * R / G) + 1
    gx0, gy0 = coord.to_grid(x0, y0)
    tx, ty = coord.to_grid(*tooth)
    reach = float(cfg.track_width) / 2 + float(cfg.clearance) + 0.02
    for L in (0, 1):
        Lname = cfg.layers[L]
        blocked = np.zeros((n, n), dtype=bool)
        for i in range(n):
            for j in range(n):
                blocked[i, j] = obstacles.is_blocked(gx0 + i, gy0 + j, L)
        si, sj = tx - gx0, ty - gy0
        if not (0 <= si < n and 0 <= sj < n):
            continue
        if blocked[si, sj]:
            print(f'  layer {Lname}: the TOOTH cell itself is blocked')
        seen = np.zeros((n, n), dtype=bool)
        q = deque([(si, sj)])
        seen[si, sj] = True
        cnt = 0
        while q:
            i, j = q.popleft()
            cnt += 1
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    a, b = i + di, j + dj
                    if 0 <= a < n and 0 <= b < n and not seen[a, b] and not blocked[a, b]:
                        seen[a, b] = True
                        q.append((a, b))
        ii, jj = np.nonzero(seen)
        if cnt == 0:
            continue
        bx = (x0 + ii.min() * G, x0 + ii.max() * G)
        by = (y0 + jj.min() * G, y0 + jj.max() * G)
        touches_edge = ii.min() == 0 or jj.min() == 0 or ii.max() == n - 1 or jj.max() == n - 1
        print(f'  layer {Lname}: pocket from the tooth = {cnt} cells, x {bx[0]:.3f}..{bx[1]:.3f} '
              f'y {by[0]:.3f}..{by[1]:.3f}{"  (reaches the box edge: OPEN)" if touches_edge else "  (CLOSED pocket)"}')
        # wall census: blocked cells 8-adjacent to the pocket
        wall = Counter()
        wall_pts = {}
        for i, j in zip(ii, jj):
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    a, b = i + di, j + dj
                    if 0 <= a < n and 0 <= b < n and blocked[a, b] and not seen[a, b]:
                        px, py = x0 + a * G, y0 + b * G
                        who = attrib(px, py, L)
                        wall[who] += 1
                        wall_pts.setdefault(who, []).append((px, py))
        if png and L == 0:
            from PIL import Image, ImageDraw
            S_ = 4
            im = Image.new('RGB', (n * S_, n * S_), (20, 22, 20))
            d = ImageDraw.Draw(im)
            for i in range(n):
                for j in range(n):
                    if seen[i, j]:
                        d.rectangle([i * S_, j * S_, i * S_ + S_ - 1, j * S_ + S_ - 1], fill=(40, 160, 60))
                    elif blocked[i, j]:
                        d.rectangle([i * S_, j * S_, i * S_ + S_ - 1, j * S_ + S_ - 1], fill=(120, 30, 30))
            def P(x, y):
                return ((x - x0) / G * S_, (y - y0) / G * S_)
            for sg, o in vsegs:
                if sg.layer != Lname:
                    continue
                col = (255, 200, 0) if o.startswith('RESERVE') else (120, 200, 255)
                d.line([P(sg.start_x, sg.start_y), P(sg.end_x, sg.end_y)], fill=col, width=2)
            for sg in rsegs:
                if sg.layer != Lname:
                    continue
                d.line([P(sg.start_x, sg.start_y), P(sg.end_x, sg.end_y)], fill=(230, 230, 230), width=2)
            for v in vvias:
                cx_, cy_ = P(v.x, v.y)
                d.ellipse([cx_ - 6, cy_ - 6, cx_ + 6, cy_ + 6], outline=(255, 120, 255), width=2)
            for v in rvias:
                cx_, cy_ = P(v.x, v.y)
                d.ellipse([cx_ - 6, cy_ - 6, cx_ + 6, cy_ + 6], outline=(255, 255, 255), width=2)
            cx_, cy_ = P(*tooth)
            d.ellipse([cx_ - 5, cy_ - 5, cx_ + 5, cy_ + 5], outline=(255, 255, 0), width=2)
            for mm in range(int(math.floor(x0)), int(x0 + 2 * R) + 2):
                px_ = (mm - x0) / G * S_
                d.line([(px_, 0), (px_, n * S_)], fill=(70, 70, 70)); d.text((px_ + 2, 2), str(mm), fill=(200, 200, 200))
            for mm in range(int(math.floor(y0)), int(y0 + 2 * R) + 2):
                py_ = (mm - y0) / G * S_
                d.line([(0, py_), (n * S_, py_)], fill=(70, 70, 70)); d.text((2, py_ + 2), str(mm), fill=(200, 200, 200))
            im.save(png)
            print(f'  wrote {png} (F: pocket green, blocked red, reserve yellow, virtual blue, real white, virtual vias magenta)')
        for who, c in wall.most_common():
            pts = wall_pts[who]
            xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
            print(f'      wall {c:5d} cells  {who:24s}  x {min(xs):.2f}..{max(xs):.2f} y {min(ys):.2f}..{max(ys):.2f}')
    # the lane's own plan geometry near the tooth
    print('  own plan: mid (s,o) =', [(round(s, 2), round(o, 3)) for s, o in cc.mid[NET]])
    print('  own req =', [(round(a, 2), round(b, 2), L[0]) for a, b, L in sorted(cc.req.get(NET, ()))])
    print('  own bwin =', [(round(a, 2) if abs(a) < 1e8 else a, round(b, 2) if abs(b) < 1e8 else b) for a, b in cc.bwin.get(NET, ())])
    print(f'  s0={cc.s0:.2f} s1={cc.s1:.2f}; launch_o={cc.launch_o[NET]:.3f} target_o={cc.target_o[NET]:.3f}; tooth st={tuple(round(v,3) for v in cc.st[NET])}')
    # the virtual owners near the tooth with their plan
    print('  virtual pieces within 1.0 mm of the tooth:')
    for sg, o in sorted(vsegs, key=lambda t: t[1]):
        d = seg_dist(tooth[0], tooth[1], sg.start_x, sg.start_y, sg.end_x, sg.end_y)
        if d < 1.0:
            sa = cc.spine.project_pt((sg.start_x, sg.start_y))
            sb = cc.spine.project_pt((sg.end_x, sg.end_y))
            print(f'     {o:6s} {sg.layer[0]} ({sg.start_x:.3f},{sg.start_y:.3f})->({sg.end_x:.3f},{sg.end_y:.3f})'
                  f'  s {sa[0]:.2f}..{sb[0]:.2f} o {sa[1]:.3f}..{sb[1]:.3f}  d={d:.3f}')
    for o in sorted(set(o for _, o in vsegs)):
        if o in cc.members and o != NET:
            print(f'     {o:6s} page {sc.page.get(o)} tooth {ctx.tooth_layer[o][0]} req {[(round(a,2), round(b,2), L[0]) for a,b,L in sorted(cc.req.get(o, ()))]}'
                  f' bwin {[(round(a,2) if abs(a)<1e8 else a, round(b,2) if abs(b)<1e8 else b) for a,b in cc.bwin.get(o, ())]}'
                  f' mid {[(round(s,2), round(oo,3)) for s, oo in cc.mid[o]]}')
    if CALL != 'last':
        raise Done()


cn.route_net_with_obstacles = spy
corridors = [te.Corridor(ci, g, ctx, logs.append) for ci, g in enumerate(groups)]
ctx.corridors = corridors
try:
    for c in corridors:
        # every corridor before NET's is routed for real first (its
        # copper is the world the later corridor sees), as main() does
        c.run()
        ctx.corr_done.add(c.idx)
        if NET in c.members:
            break
except Done:
    pass
if CALL == 'last':
    cc = [c_ for c_ in corridors if NET in c_.members][0]
    print(f'{NET}: {len(RECORDED)} router calls recorded:')
    for i, r in enumerate(RECORDED):
        print(f'   call {i}: {r[5]}, routed before it: {len(r[4])}')
    print('refused at the end:', cc.refused, ' log tail:', [l for l in logs if NET in l][-6:])
    picks = [-5, -1] if len(RECORDED) >= 5 else [-1]
    for k in picks:
        w, cfg, ob, kw, routed, lab = RECORDED[k]
        analyze(w, cfg, ob, kw, cc, routed, f'call {len(RECORDED) + k} ({lab})')

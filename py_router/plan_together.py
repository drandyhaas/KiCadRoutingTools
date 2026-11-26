"""A RUN'S NETS PLANNED TOGETHER (KICAD_PLAN_TOGETHER=1): whole routes for the run's signal nets, found with one
another in view, before any of them is routed.

Routed one after another, each net takes the way that is best for it alone, and the ways close regions on one
another: a track crosses in front of a neighbour's stub and seals it, two tracks fence a pad between them. Each net
routes alone; together some stay open, whatever the order, and ripping one for another trades which.

Here every net's whole route is found with all the others' before any is laid:

- THE BOARD is the router's own: the run's base obstacle map (every pad, track, via, hole, edge and guarded pour of
  the nets not planned, each at its clearance), and the copper the planned nets already have, each open to its own
  net alone.
- A ROUTE runs from the largest piece of its net's copper to the net's other pieces, one leg after another: a leg
  from any point of the copper and of the legs before it to any point of the nearest piece left.
- THE ROUTES ARE NEGOTIATED: a route is priced for each cell it takes within a track pitch of another route (a via for
  the routes in its reach), and for each cell that was shared in a round before. A route is found again while it
  shares a cell. The price of sharing grows a round at a time, so two routes that each want the other's ground give
  way where it costs them least; the history sends them round ground that stays contested.
- WHAT IS KEPT stands as drawn: each route clear of every route kept before it at the track pitch, a via its reach
  from a track, two vias their spacing. Of the routes still on one another's ground after the rounds, as many as can
  stand together are kept; the rest, and any net with no way at all, are left to the router's search.

A route kept is routed by its plan: the run takes it at the net's turn in place of a search (single_ended_loop), so
it is committed, tidied, checked and written as any route the run found itself. The planned nets go first and are not
ripped for a net searched after them. A net with a pour, a width or a clearance of its own, or with many pads (a
rail), is not planned.
"""
import math
import heapq
import time
import collections

import numpy as np

import env_knobs

SQ2 = math.sqrt(2.0)
INF = float('inf')
MAX_PADS = 6           # a net of more pads is a rail: left to the router
ROUNDS = 60
# What a route pays for standing on another's ground, by what it takes to part the two (the module's text):
SQUEEZE = 5            # a via among another route's tracks costs 1/SQUEEZE of a via for each such route
CROSS = 600            # a step onto another route's line costs CROSS cells within its pitch
LANE = 10              # ground of a LANE (copper laid before, which gives way only if laid again) costs LANE times a route's
KEPT = set()           # the names of the nets this process's run routed by its plan: no lap of it rips one
TAP_REACH = 1.0        # (mm) how far from its pad a supply pad's via may stand


def _on(layers_of, name):
    return name in layers_of or '*.Cu' in layers_of


def _paint(pcb, nets, layers, track, clearance, h, I0, J0, W, H, pads_only=False):
    """The copper of the nets `nets` on a grid (cell (i, j) of the window at ((I0 + i) h, (J0 + j) h)), each piece at
    the reach a track's centre keeps from it, per layer of `layers`: 0 free, a net's id (its own copper's reach
    alone), -1 two nets' reaches."""
    nL = len(layers)
    G = lambda v: int(round(v / h))  # noqa: E731
    own = np.zeros((nL, H, W), dtype=np.int32)
    r_t = clearance + track / 2.0

    def mark(li, a0, b0, a1, b1, fn, nid):
        a0, b0, a1, b1 = max(a0, 0), max(b0, 0), min(a1, W - 1), min(b1, H - 1)
        if a0 > a1 or b0 > b1:
            return
        X = (np.arange(a0, a1 + 1) + I0) * h
        Y = (np.arange(b0, b1 + 1) + J0) * h
        m = fn(X[None, :], Y[:, None])
        sub = own[li, b0:b1 + 1, a0:a1 + 1]
        sub[m] = np.where(sub == 0, nid, np.where(sub == nid, nid, -1))[m]

    def win(x0, y0, x1, y1, r):
        return G(x0 - r) - I0 - 1, G(y0 - r) - J0 - 1, G(x1 + r) - I0 + 1, G(y1 + r) - J0 + 1

    def disc(li, x, y, r, nid):
        mark(li, *win(x, y, x, y, r), lambda X, Y: (X - x) ** 2 + (Y - y) ** 2 <= r * r, nid)

    def capsule(li, x1, y1, x2, y2, r, nid):
        vx, vy = x2 - x1, y2 - y1
        L2 = vx * vx + vy * vy

        def fn(X, Y):
            u = np.clip(((X - x1) * vx + (Y - y1) * vy) / L2, 0.0, 1.0) if L2 > 0 else 0.0
            return (X - x1 - u * vx) ** 2 + (Y - y1 - u * vy) ** 2 <= r * r
        mark(li, *win(min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2), r), fn, nid)

    def rect(li, x, y, sx, sy, rot, r, nid):
        c, s = math.cos(math.radians(rot)), math.sin(math.radians(rot))
        R = math.hypot(sx, sy) / 2

        def fn(X, Y):
            u, v = (X - x) * c + (Y - y) * s, -(X - x) * s + (Y - y) * c
            return np.maximum(np.abs(u) - sx / 2, 0.0) ** 2 + np.maximum(np.abs(v) - sy / 2, 0.0) ** 2 <= r * r
        mark(li, *win(x - R, y - R, x + R, y + R, r), fn, nid)

    wx0, wy0, wx1, wy1 = I0 * h, J0 * h, (I0 + W) * h, (J0 + H) * h
    near = lambda x, y, r: wx0 - r <= x <= wx1 + r and wy0 - r <= y <= wy1 + r  # noqa: E731
    for nid in nets:
        net = pcb.nets.get(nid)
        for p in (net.pads if net is not None else ()):
            if p.pad_type == 'np_thru_hole' or not near(p.global_x, p.global_y, max(p.size_x, p.size_y) + 1.0):
                continue
            r = max(clearance, getattr(p, 'local_clearance', 0.0) or 0.0) + track / 2.0
            for li in range(nL):
                if not _on(p.layers, layers[li]):
                    continue
                if p.shape == 'circle':
                    disc(li, p.global_x, p.global_y, p.size_x / 2 + r, nid)
                elif p.shape == 'oval':
                    d = (p.size_x - p.size_y) / 2
                    if d >= 0:
                        capsule(li, p.global_x - d, p.global_y, p.global_x + d, p.global_y, p.size_y / 2 + r, nid)
                    else:
                        capsule(li, p.global_x, p.global_y + d, p.global_x, p.global_y - d, p.size_x / 2 + r, nid)
                else:
                    rect(li, p.global_x, p.global_y, p.size_x, p.size_y, getattr(p, 'rect_rotation', 0.0) or 0.0, r, nid)
    if pads_only:
        return own
    for s in pcb.segments:
        if s.net_id in nets and s.layer in layers and (near(s.start_x, s.start_y, 1.0) or near(s.end_x, s.end_y, 1.0)):
            capsule(layers.index(s.layer), s.start_x, s.start_y, s.end_x, s.end_y, s.width / 2 + r_t, s.net_id)
    for v in pcb.vias:
        if v.net_id in nets and near(v.x, v.y, 1.0):
            for li in range(nL):
                disc(li, v.x, v.y, v.size / 2 + r_t, v.net_id)
    return own


def _base_blocked(base, layers_on, nL, I0, J0, W, H):
    """the run's base obstacle map read into arrays over a window: (track cells blocked per layer [nL, H, W] -- a
    layer not routed on all blocked --, via cells blocked [H, W])"""
    blk = np.ones((nL, H, W), dtype=bool)
    vblk = np.ones((H, W), dtype=bool)
    is_b, is_v = base.is_blocked, base.is_via_blocked
    for j in range(H):
        gy = J0 + j
        for i in range(W):
            gx = I0 + i
            vblk[j, i] = is_v(gx, gy)
            for li in layers_on:
                blk[li, j, i] = is_b(gx, gy, li)
    return blk, vblk


def plan_routes(pcb, config, net_ids, base, rounds=ROUNDS, log=None, search=None, dbg=None, soft=None,
                start_reach=None, whole=None, exits=0.0, taps=None):
    """Whole routes for the nets `net_ids` of `pcb`, found together (the module's text), on the base obstacle map
    `base` (built without those nets), at the run's sizes, layers, layer costs and via cost (`config`).

    -> (routes {net id: [(way [(gx, gy, layer index), ...], the points of the net's copper the way begins at and
    arrives at), ...]} -- a net's legs, on the router's grid, a change of layer two points at one place --, report: nets,
    planned, boxed (names: no way), shared (names: on another route's ground after the rounds), close (names: as
    drawn, closer to a route kept than the spacing), partial (names: not every piece reached), rounds, vias, seconds). `search`: 'py' for this module's own search (the crate's
    plan_search is the same search, and is used where the crate has it)."""
    from routing_config import REFERENCE_GRID_STEP
    t0 = time.time()
    log = log or (lambda *_a: None)
    layers = list(config.layers)
    nL = len(layers)
    h = config.grid_step
    track, clearance, via = config.track_width, config.clearance, config.via_size
    LC = [c / 1000.0 if c > 0 else -1.0 for c in config.get_layer_costs()]
    routable = [i for i in range(nL) if LC[i] > 0]
    rep = dict(nets=0, planned=0, boxed=[], shared=[], close=[], partial=[], rounds=0, vias=0, seconds=0.0, asks={},
               ask_legs={})
    soft = set(soft or ()) & set(net_ids)
    bounds = pcb.board_info.board_bounds
    if not routable or not bounds:
        return {}, rep
    cmin = min(LC[i] for i in routable)
    via_mm = config.via_cost * REFERENCE_GRID_STEP    # (what the router charges a via, as millimetres of path)
    want = set(net_ids)
    # THE SUPPLY PADS' VIAS (`taps` {a poured net's id: [(a layer it pours on, where: has .contains_xy or None)]}):
    # each piece of such a net's copper that no via and no plated hole joins to its pours is a route of the plan too
    # -- from the piece to a layer the net pours on, within TAP_REACH of its pad, or to a via the net has there.
    # Laid after the signals, a pad's via stood where a signal ran and one of the two was given up; here each gives
    # way to the other before either is laid.
    taps = {n: v for n, v in (taps or {}).items() if n not in want}
    tapn = set(taps)
    segs, vias = collections.defaultdict(list), collections.defaultdict(list)
    for s in pcb.segments:
        if s.net_id in want | tapn and not getattr(s, 'graphic', False):
            segs[s.net_id].append(s)
    for v in pcb.vias:
        if v.net_id in want | tapn:
            vias[v.net_id].append(v)
    K = lambda x, y: (round(x, 3), round(y, 3))  # noqa: E731
    lidx = {name: i for i, name in enumerate(layers)}

    # ---- the nets: each net's pieces (pads the copper it has already joins); its route begins on the largest
    starts = []
    for nid in sorted(want | tapn):
        net = pcb.nets.get(nid)
        if net is None:
            continue
        link = collections.defaultdict(list)
        for s in segs[nid]:
            a, b = K(s.start_x, s.start_y), K(s.end_x, s.end_y)
            li = lidx.get(s.layer, -1)
            link[a, li].append((b, li)); link[b, li].append((a, li))
        via_at = {K(v.x, v.y) for v in vias[nid]}

        def reach(q):
            got, todo = set(), [(K(q.global_x, q.global_y), li) for li in range(nL) if _on(q.layers, layers[li])]
            while todo:
                st = todo.pop()
                if st in got:
                    continue
                got.add(st)
                todo += link.get(st, ())
                if st[0] in via_at:
                    todo += [(st[0], k) for k in range(nL)]
            return got
        pieces, left = [], [q for q in net.pads if q.pad_type != 'np_thru_hole']
        while left:
            q = left.pop(0)
            got = reach(q)
            # (another pad of the net the piece's copper stands on, ON THAT PAD'S LAYER: a pad on the far face
            # beneath a ball is no pad of the ball's piece)
            same = [r for r in left if any(_on(r.layers, layers[li]) and abs(r.global_x - x) <= r.size_x / 2 + 1e-6
                                           and abs(r.global_y - y) <= r.size_y / 2 + 1e-6 for (x, y), li in got)]
            left = [r for r in left if not any(r is t for t in same)]
            pieces.append(([q] + same, got))
        if nid in tapn:
            zl = [(lidx[L], where) for L, where in taps[nid] if L in lidx and LC[lidx[L]] > 0]
            at_via = lambda seen: any(pt in via_at for pt, _li in seen)                    # noqa: E731
            for pads_, seen in pieces:
                if not zl or at_via(seen) or any(q.drill for q in pads_) or \
                        any(_on(q.layers, layers[li]) for q in pads_ for li, _w in zl):
                    continue                # (joined to every layer, or on a layer its net pours on)
                at_pad = {K(q.global_x, q.global_y) for q in pads_}
                starts.append(dict(
                    nid=nid, name=f'{net.name} at {pads_[0].component_ref}.{pads_[0].pad_number}', parts=[],
                    own=([(sg.start_x, sg.start_y, sg.end_x, sg.end_y, lidx[sg.layer]) for sg in segs[nid]
                          if sg.layer in lidx and (K(sg.start_x, sg.start_y), lidx[sg.layer]) in seen],
                         [(pt, li) for pt, li in seen if pt in at_pad and li in routable]),
                    tap=(zl, [(q.global_x, q.global_y) for q in pads_], [(v.x, v.y) for v in vias[nid]])))
            continue
        if len(pieces) < 2:
            continue
        k0 = max(range(len(pieces)), key=lambda k: (len(pieces[k][1]), -k))
        def copper(pads_, seen):
            """a piece's copper: (its tracks [(x1, y1, x2, y2, layer index)], its pads' and vias' points on the
            layers routed [((x, y), layer index)])"""
            at_pad = {K(q.global_x, q.global_y) for q in pads_}
            return ([(sg.start_x, sg.start_y, sg.end_x, sg.end_y, lidx[sg.layer]) for sg in segs[nid]
                     if sg.layer in lidx and (K(sg.start_x, sg.start_y), lidx[sg.layer]) in seen],
                    [(pt, li) for pt, li in seen if (pt in via_at or pt in at_pad) and li in routable])
        starts.append(dict(nid=nid, name=net.name, own=copper(*pieces[k0]),
                           parts=[copper(*pc) for k, pc in enumerate(pieces) if k != k0]))
    rep['nets'] = sum(1 for s in starts if not s.get('tap'))
    rep['taps'] = dict(asked=sum(1 for s in starts if s.get('tap')), kept=0, left=[])
    rep['tap_routes'] = []
    if not starts:
        return {}, rep

    # ---- the grid: the router's own, over the board and a margin
    G = lambda v: int(round(v / h))  # noqa: E731
    M = 12
    I0, J0 = G(bounds[0]) - M, G(bounds[1]) - M
    W, H = G(bounds[2]) + M - I0 + 1, G(bounds[3]) + M - J0 + 1
    N = W * H
    inz = np.zeros((H, W), dtype=np.uint8)
    inz[M:H - M, M:W - M] = 1
    # the board: the base map's cells, and the planned nets' own copper open to its net alone
    blk, vblk = _base_blocked(base, routable, nL, I0, J0, W, H)
    # (a LANE net's tracks and vias are no wall: ground a route may take at a price, `soft`; its pads are walls)
    own = _paint(pcb, (want | tapn) - soft, layers, track, clearance, h, I0, J0, W, H)
    if soft:
        own_p = _paint(pcb, soft, layers, track, clearance, h, I0, J0, W, H, pads_only=True)
        own = np.where(own == 0, own_p, np.where(own_p == 0, own, np.where(own == own_p, own, -1)))
    # where a via may stand: its barrel clear of the planned nets' copper on every layer (drawn as the walls are: a
    # place is closed where a via's centre there is nearer the copper than the spacing)
    vown = np.zeros((H, W), dtype=np.int32)
    p_v = _paint(pcb, (want | tapn) - soft, layers, via, clearance, h, I0, J0, W, H)
    if soft:
        p_s = _paint(pcb, soft, layers, via, clearance, h, I0, J0, W, H, pads_only=True)
        p_v = np.where(p_v == 0, p_s, np.where(p_s == 0, p_v, np.where(p_v == p_s, p_v, -1)))
    for li in range(nL):
        a_ = p_v[li]
        vown = np.where(a_ == 0, vown, np.where(vown == 0, a_, np.where(vown == a_, vown, -1)))
    del p_v
    vown[vblk] = -1
    # (no via on a pad of the planned nets, the net's own included: a pad's reach for a via's rim, and a cell more)
    vown[(_paint(pcb, want, layers, via, h, h, I0, J0, W, H, pads_only=True) != 0).any(axis=0)] = -1
    own[blk] = -1
    if exits > 0:
        # THE WAY OUT OF EVERY OTHER ESCAPE kept: ahead of a track's loose end (no pad, via or track of its net at
        # it) of a net not planned here, `exits` mm on along the track, is closed to these routes on its layer and
        # to their vias -- a start stands in no other escape's mouth
        import types as _types
        ends_ = collections.Counter()
        for sg in pcb.segments:
            if sg.net_id not in want and not getattr(sg, 'graphic', False):
                ends_[sg.net_id, K(sg.start_x, sg.start_y), sg.layer] += 1
                ends_[sg.net_id, K(sg.end_x, sg.end_y), sg.layer] += 1
        held = {(v.net_id, K(v.x, v.y)) for v in pcb.vias} | {
            (q.net_id, K(q.global_x, q.global_y)) for n_ in pcb.nets.values() for q in n_.pads}
        ghosts = []
        for sg in pcb.segments:
            if sg.net_id in want or sg.layer not in lidx or getattr(sg, 'graphic', False):
                continue
            for (ax, ay), (bx, by) in (((sg.start_x, sg.start_y), (sg.end_x, sg.end_y)),
                                       ((sg.end_x, sg.end_y), (sg.start_x, sg.start_y))):
                L_ = math.hypot(bx - ax, by - ay)
                if L_ < 1e-6 or ends_[sg.net_id, K(bx, by), sg.layer] != 1 or (sg.net_id, K(bx, by)) in held:
                    continue
                ghosts.append(_types.SimpleNamespace(
                    start_x=bx, start_y=by, end_x=bx + (bx - ax) / L_ * exits, end_y=by + (by - ay) / L_ * exits,
                    width=sg.width, layer=sg.layer, net_id=-7))
        if ghosts:
            fake = _types.SimpleNamespace(nets={}, segments=ghosts, vias=[])
            own[_paint(fake, {-7}, layers, track, clearance, h, I0, J0, W, H) != 0] = -1
            vown[(_paint(fake, {-7}, layers, via, clearance, h, I0, J0, W, H) != 0).any(axis=0)] = -1
        rep['exits'] = len(ghosts)
    OWN = own.reshape(-1).tolist()
    VOWN = vown.reshape(-1).tolist()
    INZ = inz.reshape(-1).tolist()
    # The search is the crate's where it has one (plan_search: search_leg below, ported exactly); it reads the same
    # tables as arrays, kept in step with the lists.
    fast = None
    if search != 'py':
        try:
            import grid_router
            fast = getattr(grid_router, 'plan_search', None)
        except ImportError:
            fast = None
    if fast is not None:
        OWN_A = np.ascontiguousarray(own.reshape(-1), dtype=np.int32)
        VOWN_A = np.ascontiguousarray(vown.reshape(-1), dtype=np.int32)
        INZ_A = np.ascontiguousarray(inz.reshape(-1), dtype=np.uint8)
        USE_A, VUSE_A = np.zeros(nL * N, dtype=np.int32), np.zeros(N, dtype=np.int32)
        HIST_A, VHIST_A = np.zeros(nL * N, dtype=np.float64), np.zeros(N, dtype=np.float64)
    del own, vown, blk, vblk

    # Two routes are compared cell to cell at the track pitch (what is kept is checked as drawn, at the end). A via
    # keeps its own reach from a track, and from another route's via the larger of the copper's and the holes' spacing.
    sep = track + clearance
    ks = int(sep / h) + 1
    tr_off = [dj * W + di for di in range(-ks, ks + 1) for dj in range(-ks, ks + 1) if math.hypot(di, dj) * h < sep - 1e-9]
    r_vs = via / 2.0 + clearance + track / 2.0
    kv = int(r_vs / h) + 1
    vs_off = [dj * W + di for di in range(-kv, kv + 1) for dj in range(-kv, kv + 1) if math.hypot(di, dj) * h < r_vs - 1e-9]
    r_vv = max(via + clearance, (getattr(config, 'via_drill', 0.0) or 0.0)
               + (getattr(config, 'hole_to_hole_clearance', 0.0) or 0.0))
    if start_reach is not None:
        r_vv = max(r_vv, via + 2 * clearance + track)   # (two starts' vias a track's room apart, on every layer)
    kw = int(r_vv / h) + 1
    vv_off = [dj * W + di for di in range(-kw, kw + 1) for dj in range(-kw, kw + 1) if math.hypot(di, dj) * h < r_vv - 1e-9]
    MOVES = [(1, h, 0, 0), (-1, h, 0, 0), (W, h, 0, 0), (-W, h, 0, 0),
             (W + 1, h * SQ2, 1, W), (W - 1, h * SQ2, -1, W), (-W + 1, h * SQ2, 1, -W), (-W - 1, h * SQ2, -1, -W)]
    REACH = [0] * (nL * N)                            # (routes within a pitch of the cell, a via's reach of it)
    CORE = [0] * (nL * N)                             # (routes whose line the cell is on)
    VNEAR = [0] * N                                   # (routes with a via within a via's spacing of the place)
    LREACH, LTN, LVN = ([0] * (nL * N), [0] * N, [0] * N) if soft else (None, None, None)   # (the same, of lanes)
    TNEAR = [0] * N                                   # (routes with a track, on any layer, within a via's reach of it)
    # what the search is charged, from those: a cell SQUEEZE for each route in reach and SQUEEZE * CROSS for each
    # route whose line it is on; a via 1 for each route whose track it would stand among, 2 for each via too near
    USE = [0] * (nL * N) if fast is None else None
    VUSE = [0] * N if fast is None else None
    hist = {}

    def cell(x, y):
        return (G(y) - J0) * W + (G(x) - I0)

    def cells_of(cop, dots):
        """a piece's cells: {cell: the point of the piece's copper it stands for} -- every cell along its tracks, and
        its pads' and vias' own (a route leaves a piece, or arrives on it, at any of them)"""
        at = {}
        for x1, y1, x2, y2, li in cop:
            if LC[li] > 0:
                k = max(1, int(math.hypot(x2 - x1, y2 - y1) / h * 2))
                for t in range(k + 1):
                    pt = (x1 + (x2 - x1) * t / k, y1 + (y2 - y1) * t / k)
                    at.setdefault(li * N + cell(*pt), pt)
        for pt, li in dots:
            at[li * N + cell(*pt)] = pt
        return at

    for s in starts:
        srcpt = cells_of(*s['own'])               # (a cell of the net's copper -> the point it stands for)
        s['src'] = sorted(srcpt)
        s['tparts'] = []
        for cop, dots in s['parts']:
            at = cells_of(cop, dots)
            srcpt.update((u, pt) for u, pt in at.items() if u not in srcpt)
            # (the estimate to go: to the piece's pads, vias and track ends -- near enough to its nearest copper)
            ends = {pt for pt, _li in dots} | {(x, y) for x1, y1, x2, y2, _li in cop for x, y in ((x1, y1), (x2, y2))}
            s['tparts'].append((set(at), sorted({(G(x) - I0, G(y) - J0) for x, y in ends})))
        s['srcpt'] = srcpt
        s['mine'] = set(srcpt)                    # (the cells of the copper the net has: no route's to dispute)
        s['path'], s['stamp'], s['full'] = None, set(), False
        if s.get('tap'):
            # A SUPPLY PAD'S VIA: from the piece to any place within TAP_REACH of a pad of it on a layer its net
            # pours on (where the pour is the net's), or to a via the net has within the reach
            zl, pts, own_v = s['tap']
            kr = max(1, int(round(TAP_REACH / h)))
            tgt = set()
            for x, y in pts:
                i_, j_ = G(x) - I0, G(y) - J0
                jj, ii = np.meshgrid(np.arange(max(j_ - kr, 0), min(j_ + kr, H - 1) + 1),
                                     np.arange(max(i_ - kr, 0), min(i_ + kr, W - 1) + 1), indexing='ij')
                jj, ii = jj.reshape(-1), ii.reshape(-1)
                for li, where in zl:
                    ok = np.ones(len(jj), dtype=bool) if where is None else \
                        where.contains_xy((ii + I0) * h, (jj + J0) * h)
                    tgt.update((li * N + jj[ok] * W + ii[ok]).tolist())
                for vx, vy in own_v:
                    if math.hypot(vx - x, vy - y) <= TAP_REACH and any(
                            where is None or bool(where.contains_xy(np.array([vx]), np.array([vy]))[0])
                            for _li, where in zl):
                        tgt.update(L * N + cell(vx, vy) for L in routable)
            tgt -= set(srcpt)
            s['tparts'] = [(tgt, sorted({(G(x) - I0, G(y) - J0) for x, y in pts}))] if tgt else []
            if not tgt:
                s['src'] = []
        if start_reach is not None and s['nid'] not in (whole or ()):
            # A START alone: from the net's piece on ONE layer (an escape with no via) to another layer, within
            # `start_reach` of that piece -- the via that opens every layer to it, and no more of its route
            one = [(cop, dots) for cop, dots in [s['own']] + list(s['parts'])
                   if len({li for *_p, li in cop} | {li for _pt, li in dots}) == 1]
            if not one:
                s['src'], s['tparts'] = [], []
                continue
            at = cells_of(*one[0])
            li0 = next(iter(at)) // N
            kr = max(1, int(round(start_reach / h)))
            tgt = set()
            for u in at:
                j_, i_ = divmod(u % N, W)
                for L in routable:
                    if L != li0:
                        tgt.update(L * N + jj * W + ii for jj in range(max(j_ - kr, 0), min(j_ + kr, H - 1) + 1)
                                   for ii in range(max(i_ - kr, 0), min(i_ + kr, W - 1) + 1))
            s['src'] = sorted(at)
            s['tparts'] = [(tgt, sorted({(u % N % W, u % N // W) for u in at}))]

    def lane(s, sign):
        """a lane's copper (`s`, of `lanes`) on the tables (made once): its tracks' and vias' reach, as a route's, at
        LANE times a route's price -- taken off while its own net's route is found"""
        if 'l_st' not in s:
            # (as the walls are drawn: a cell is in the lane's reach where a track's centre -- a via's -- there is
            # nearer the lane's copper than the spacing)
            one = {s['nid']}
            p_t = _paint(pcb, one, layers, track, clearance, h, I0, J0, W, H)
            p_v = (_paint(pcb, one, layers, via, clearance, h, I0, J0, W, H) != 0).any(axis=0)
            p_t[[li for li in range(nL) if LC[li] <= 0]] = 0
            core, vst = set(), set()
            for x1, y1, x2, y2, li in s['tracks']:
                if LC[li] <= 0:
                    continue
                k = max(1, int(math.hypot(x2 - x1, y2 - y1) / h * 2))
                for t in range(k + 1):
                    u = li * N + cell(x1 + (x2 - x1) * t / k, y1 + (y2 - y1) * t / k)
                    core.update((u, u + 1, u - 1, u + W, u - W))
            for x, y in s['vias']:
                idx = cell(x, y)
                vst.update(idx + o for o in vv_off)
            s['l_st'] = set(np.flatnonzero(p_t.reshape(-1)).tolist())
            s['l_core'] = {u for u in core if 0 <= u < nL * N}
            s['l_vst'] = {i for i in vst if 0 <= i < N}
            s['l_tn'] = set(np.flatnonzero(p_v.reshape(-1)).tolist())
        for u in s['l_st']:
            LREACH[u] += sign
        for i in s['l_tn']:
            LTN[i] += sign
        for i in s['l_vst']:
            LVN[i] += sign
        for key, k_ in (('l_st', SQUEEZE * LANE), ('l_core', SQUEEZE * CROSS * LANE)):
            if fast is not None:
                USE_A[np.fromiter(s[key], dtype=np.int64, count=len(s[key]))] += sign * k_
            else:
                for u in s[key]:
                    USE[u] += sign * k_
        for key, k_ in (('l_vst', 2 * LANE), ('l_tn', LANE)):
            if fast is not None:
                if s[key]:
                    VUSE_A[np.fromiter(s[key], dtype=np.int64, count=len(s[key]))] += sign * k_
            else:
                for i in s[key]:
                    VUSE[i] += sign * k_

    # (every lane net's copper, the nets with nothing left to route among them)
    lane_of = {nid: dict(nid=nid, name=pcb.nets[nid].name,
                         tracks=[(sg.start_x, sg.start_y, sg.end_x, sg.end_y, lidx[sg.layer]) for sg in segs[nid]
                                 if sg.layer in lidx], vias=[(v.x, v.y) for v in vias[nid]])
               for nid in sorted(soft) if nid in pcb.nets}
    lanes = list(lane_of.values())
    for s in lanes:
        lane(s, +1)

    def search(s, pf):
        """the net's route: its legs' cells one leg after another, None between two"""
        out, src, left = [], list(s['src']), list(s['tparts'])
        if not src:
            return None
        while left:
            path = search_leg(s['nid'], pf / SQUEEZE, src, set().union(*(t for t, _x in left)),
                              [xy for _t, x in left for xy in x])
            if path is None or len(path) < 2:
                break
            out += ([None] if out else []) + path
            done = [t for t, _x in left if path[-1] in t]
            left = [(t, x) for t, x in left if path[-1] not in t]
            src = src + path + [u for t in done for u in t]
        s['full'] = bool(out) and not left
        return out or None

    def search_leg(nid, pf, src, tgt, txy):
        if fast is not None:
            return fast(OWN_A, VOWN_A, USE_A, VUSE_A, HIST_A, VHIST_A, INZ_A, LC, W, nid, h, cmin, via_mm * cmin, pf,
                        list(src), list(tgt), list(txy), [])

        def hf(idx):
            j, i = divmod(idx, W)
            d = 10 ** 9
            for ti, tj in txy:
                di, dj = abs(ti - i), abs(tj - j)
                d = min(d, max(di, dj) + (SQ2 - 1) * min(di, dj))
            return d * h * cmin
        g, par, heap = {}, {}, []
        for u in src:
            g[u] = 0.0
            heapq.heappush(heap, (hf(u % N), 0.0, u))
        best, best_end = INF, None
        while heap:
            f, gu, u = heapq.heappop(heap)
            if f >= best:
                break
            if gu > g.get(u, INF):
                continue
            if u in tgt:
                best, best_end = gu, u
                continue
            li, idx = divmod(u, N)
            lc = LC[li]
            for d, sl, o1, o2 in MOVES:
                v = u + d
                o = OWN[v]
                if o != 0 and o != nid and v not in tgt:
                    continue
                if o1:
                    a, b = OWN[u + o1], OWN[u + o2]
                    if (a != 0 and a != nid) or (b != 0 and b != nid):
                        continue
                gv = gu + sl * lc * (1.0 + hist.get(v, 0.0)) * (1.0 + pf * USE[v])
                vi = idx + d
                if not INZ[vi]:
                    continue
                if gv < g.get(v, INF):
                    g[v] = gv
                    par[v] = u
                    heapq.heappush(heap, (gv + hf(vi), gv, v))
            vo = VOWN[idx]
            if vo == 0 or vo == nid:
                gv = gu + via_mm * cmin * (1.0 + hist.get(('v', idx), 0.0)) * (1.0 + pf * VUSE[idx])
                hv = hf(idx)
                for k in routable:
                    v = k * N + idx
                    if k != li and gv < g.get(v, INF) and (OWN[v] == 0 or OWN[v] == nid):
                        g[v] = gv
                        par[v] = u
                        heapq.heappush(heap, (gv + hv, gv, v))
        if best_end is None:
            return None
        path = [best_end]
        u = par.get(best_end)
        while u is not None:
            path.append(u)
            u = par.get(u)
        return path[::-1]

    def stamp(s, sign):
        if sign > 0:
            st, core, vst, vat, tn = set(), set(), set(), set(), set()
            for p in _legs(s['path']):
                for k, u in enumerate(p):
                    idx = u % N
                    if (k > 0 and p[k - 1] % N == idx and p[k - 1] != u) or (k + 1 < len(p) and p[k + 1] % N == idx):
                        st.update(q * N + idx + o for q in range(nL) for o in vs_off)   # (a via: on every layer)
                        core.add(u)
                        if idx not in vat:
                            vat.add(idx)
                            vst.update(idx + o for o in vv_off)
                    else:
                        st.update(u + o for o in tr_off)
                        tn.update(idx + o for o in vs_off)
                        # (the route's LINE: its cells, and the two beside a diagonal step, so that no route steps
                        # across it but onto it)
                        core.add(u)
                        d = u - p[k - 1] if k > 0 and p[k - 1] // N == u // N else 0
                        if abs(d) in (W + 1, W - 1):
                            core.add(p[k - 1] + (1 if d in (W + 1, 1 - W) else -1))
                            core.add(p[k - 1] + (W if d > 0 else -W))
            s['stamp'] = {u for u in st if 0 <= u < nL * N}
            s['core'] = {u for u in core if 0 <= u < nL * N}
            s['vstamp'] = {i for i in vst if 0 <= i < N}
            s['tn'] = {i for i in tn if 0 <= i < N}
            s['vat'] = vat
        for u in s['stamp']:
            REACH[u] += sign
        for u in s.get('core', ()):
            CORE[u] += sign
        for i in s.get('vstamp', ()):
            VNEAR[i] += sign
        for i in s.get('tn', ()):
            TNEAR[i] += sign
        if fast is not None:
            for key, arr, k_ in (('stamp', USE_A, SQUEEZE), ('core', USE_A, SQUEEZE * CROSS), ('vstamp', VUSE_A, 2),
                                 ('tn', VUSE_A, 1)):
                if s.get(key):
                    arr[np.fromiter(s[key], dtype=np.int64, count=len(s[key]))] += sign * k_
        else:
            for u in s['stamp']:
                USE[u] += sign * SQUEEZE
            for u in s.get('core', ()):
                USE[u] += sign * SQUEEZE * CROSS
            for i in s.get('vstamp', ()):
                VUSE[i] += sign * 2
            for i in s.get('tn', ()):
                VUSE[i] += sign
        if sign < 0:
            s['stamp'], s['core'], s['vstamp'], s['tn'], s['vat'] = set(), set(), set(), set(), set()

    def shared(s, lanes_too=True):
        """where the net's route stands on another's ground: its cells within a pitch of another route (the copper
        the net has, apart), and its vias within a via's spacing of another's or with another route's track in reach
        -- and, `lanes_too`, the same of a lane's copper (not its own net's)"""
        tn, mine = s.get('tn', ()), s['mine']
        out = [u for u in (s['path'] or ())[1:] if u is not None and REACH[u] >= 2 and u not in mine] \
            + [('v', i) for i in s.get('vat', ()) if VNEAR[i] >= 2 or TNEAR[i] - (i in tn) >= 1]
        if soft and lanes_too and s['path']:
            mine_l = lane_of.get(s['nid']) or {}
            l_st, l_tn, l_vst = mine_l.get('l_st', ()), mine_l.get('l_tn', ()), mine_l.get('l_vst', ())
            got = set(out)
            out += [u for u in s['path'][1:] if u is not None and LREACH[u] and u not in mine and u not in got
                    and LREACH[u] - (u in l_st) >= 1]
            out += [('v', i) for i in s.get('vat', ()) if ('v', i) not in got
                    and (LTN[i] - (i in l_tn) >= 1 or LVN[i] - (i in l_vst) >= 1)]
        return out

    # The present price grows a round at a time and a shared cell's history a step a round: two routes that each
    # need the other's ground give way through the history, not through a price too high to cross.
    pf = 0.5
    for it in range(rounds):
        todo = [s for s in starts if it == 0 or (s['path'] and shared(s))]
        if not todo:
            break
        rep['rounds'] = it + 1
        for s in todo:
            stamp(s, -1)
            if s['nid'] in lane_of:
                lane(lane_of[s['nid']], -1)
            s['path'] = search(s, pf)
            if s['nid'] in lane_of:
                lane(lane_of[s['nid']], +1)
            if s['path']:
                stamp(s, +1)
        n_sh = n_x = 0
        for s in starts:
            if s['path']:
                n_x += sum(1 for u in s['path'] if u is not None and u not in s['mine'] and CORE[u] >= 2)
            for u in shared(s):
                hist[u] = hist.get(u, 0.0) + 1.0
                if fast is not None:
                    if isinstance(u, tuple):
                        VHIST_A[u[1]] = hist[u]
                    else:
                        HIST_A[u] = hist[u]
                n_sh += 1
        log(f'  plan round {it + 1}: {len(todo)} route(s) found, {n_sh} shared cell(s), {n_x} of them on another '
            f"route's line, {sum(1 for s in starts if s['path'] and shared(s))} route(s) on shared ground, "
            f'{time.time() - t0:.1f} s')
        pf *= 1.3

    # ---- what is kept. A net with no way, or with a piece its route does not reach, is the router's. Of the routes
    # still on one another's ground as many as can stand together are kept: the one in conflict with the fewest
    # others first, the ones it conflicts with then out of the count (tried last: they fall where a route kept
    # stands). A route on nobody's ground comes before them all, the shortest first.
    if dbg is not None:
        dbg.update(N=N, W=W, H=H, I0=I0, J0=J0, h=h, layers=layers, OWN=OWN, VOWN=VOWN, hist=dict(hist),
                   nets=[dict(nid=s['nid'], name=s['name'], path=list(s['path'] or ()), full=s['full'],
                              shared=shared(s) if s['path'] else [], src=list(s['src']),
                              tparts=[sorted(t) for t, _x in s['tparts']]) for s in starts])
    for s in starts:
        if not s['path']:
            (rep['taps']['left'] if s.get('tap') else rep['boxed']).append(s['name'])
        elif not s['full']:
            rep['partial'].append(s['name'])
            stamp(s, -1)
            s['path'] = None
    held = [(s, list(s['path'])) for s in starts if s['path']]
    rank = {}
    tied = [s for s, _p in held if shared(s, False)]
    if len(tied) > 1:
        cells = {id(s): {u for u in s['path'] if u is not None} for s in tied}
        foes = {id(s): set() for s in tied}
        for a in tied:
            for b in tied:
                if a is not b and (cells[id(a)] & b['stamp'] or a.get('vat', set()) & b.get('vstamp', set())
                                   or a.get('vat', set()) & b.get('tn', set())):
                    foes[id(a)].add(id(b))
                    foes[id(b)].add(id(a))
        left = {id(s): s for s in tied}
        while left:
            k = min(left, key=lambda i: (len(foes[i] & set(left)), len(left[i]['path']), left[i]['nid']))
            rank[k] = len(rank)
            for i in foes[k] & set(left):
                rank.setdefault(i, 10 ** 6 + len(rank))
                left.pop(i)
            left.pop(k, None)
    for s, _p in held:
        stamp(s, -1)
    routes = {}
    pitch, v_t = track + clearance, via / 2.0 + clearance + track / 2.0
    kept = []                                     # (runs, vias) of the routes kept, as drawn
    for s, path in sorted(held, key=lambda t: (id(t[0]) in rank, rank.get(id(t[0]), 0), len(t[1]), t[0]['nid'])):
        s['path'] = path
        stamp(s, +1)
        if shared(s, False):
            u0 = next((u for u in shared(s, False) if not isinstance(u, tuple)), None)
            if u0 is not None:
                log(f"  plan: {s['name']} left out, on another route's ground at ({(u0 % N % W + I0) * h:.2f}, "
                    f"{(u0 % N // W + J0) * h:.2f}) {layers[u0 // N]}")
            stamp(s, -1)
            (rep['taps']['left'] if s.get('tap') else rep['shared']).append(s['name'])
            continue
        legs = []
        for leg in _legs(path):
            w_ = []
            for u in leg:
                li, idx = divmod(u, N)
                j, i = divmod(idx, W)
                w_.append((i + I0, j + J0, li))
            legs.append((w_, s['srcpt'].get(leg[0]), s['srcpt'].get(leg[-1])))
        # AS DRAWN the route keeps the pitch from every route kept before it (its cells do; a run between two cells
        # may pass a little closer), a via its reach from a run, and two vias their spacing
        runs, vs = [], []
        for w_, e_, f_ in legs:
            r_, v_ = _shape(w_, e_, h, f_)
            runs += r_
            vs += v_
        if any(any(l1 == l2 and _seg_dist(a, b, c, d) < pitch - 1e-6 for a, b, l1 in runs for c, d, l2 in oruns)
               or any(_seg_dist(v, v, c, d) < v_t - 1e-6 for v in vs for c, d, _l in oruns)
               or any(_seg_dist(v, v, a, b) < v_t - 1e-6 for v in ovs for a, b, _l in runs)
               or any(math.dist(v, w) < r_vv - 1e-6 for v in vs for w in ovs) for oruns, ovs in kept):
            stamp(s, -1)
            (rep['taps']['left'] if s.get('tap') else rep['close']).append(s['name'])
            continue
        kept.append((runs, vs))
        if s.get('tap'):
            rep['tap_routes'].append((s['nid'], s['name'], legs))
            rep['taps']['kept'] += 1
            continue
        if soft and shared(s):
            # (clear of every route kept, and on a lane's ground: the route stands if the lane is laid again round it)
            cells_, vat_ = {u for u in path if u is not None}, s.get('vat', set())
            rep['asks'][s['name']] = sorted(o['name'] for o in lanes if o['nid'] != s['nid'] and (
                cells_ & o['l_st'] or vat_ & o['l_tn'] or vat_ & o['l_vst']))
            rep['ask_legs'][s['nid']] = legs
            continue
        routes[s['nid']] = legs
        rep['vias'] += len(vs)
    rep.update(planned=len(routes), seconds=round(time.time() - t0, 1))
    return routes, rep


def _legs(path):
    """a route's cells as its legs (None stands between two)"""
    out, cur = [], []
    for u in path or ():
        if u is None:
            out.append(cur)
            cur = []
        else:
            cur.append(u)
    if cur:
        out.append(cur)
    return out


def _shape(way, e, g, f=None):
    """a way as drawn: (straight runs [((x1, y1), (x2, y2), layer index)], vias [(x, y)]), mm; `e`, `f`: the points
    of the net's own copper it begins at and arrives at (None: the cell's own)"""
    pts = [(x * g, y * g) for x, y, _li in way]
    runs, vias = [], []
    if e is not None and math.hypot(e[0] - pts[0][0], e[1] - pts[0][1]) > 1e-6:
        runs.append(((e[0], e[1]), pts[0], way[0][2]))
    i = 0
    while i < len(way) - 1:
        (x1, y1, l1), (x2, y2, l2) = way[i], way[i + 1]
        if l1 != l2:
            vias.append(pts[i])
            i += 1
            continue
        d, j = (x2 - x1, y2 - y1), i + 1
        while j + 1 < len(way) and way[j + 1][2] == l1 and (way[j + 1][0] - way[j][0], way[j + 1][1] - way[j][1]) == d:
            j += 1
        runs.append((pts[i], pts[j], l1))
        i = j
    if f is not None and math.hypot(f[0] - pts[-1][0], f[1] - pts[-1][1]) > 1e-6:
        runs.append((pts[-1], (f[0], f[1]), way[-1][2]))
    return runs, vias


def _seg_dist(a, b, c, d):
    """the distance between the segments a-b and c-d"""
    def pt(p, q, r):
        vx, vy = r[0] - q[0], r[1] - q[1]
        L2 = vx * vx + vy * vy
        u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - q[0]) * vx + (p[1] - q[1]) * vy) / L2))
        return math.hypot(p[0] - q[0] - u * vx, p[1] - q[1] - u * vy)
    o = lambda p, q, r: (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])  # noqa: E731
    if (o(a, b, c) > 0) != (o(a, b, d) > 0) and (o(c, d, a) > 0) != (o(c, d, b) > 0):
        return 0.0
    return min(pt(a, c, d), pt(b, c, d), pt(c, a, b), pt(d, a, b))


def route_results(config, routes):
    """The planned routes as route results: {net id: a result as the router's own -- new_segments, new_vias, path,
    iterations 0, planned True}: a track a straight run of a leg on one layer, a via where a leg changes layer."""
    from kicad_parser import Segment, Via
    from via_span import via_layer_pair, via_kind
    out = {}
    g = config.grid_step
    for nid, legs in routes.items():
        runs, vs = [], []
        for way, e, f in legs:
            r_, v_ = _shape(way, e, g, f)
            runs += r_
            vs += v_
        segs = [Segment(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1],
                        width=config.get_net_track_width(nid, config.layers[li]), layer=config.layers[li], net_id=nid)
                for a, b, li in runs if math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-9]
        vias = [Via(x=x, y=y, size=config.via_size, drill=config.via_drill, layers=via_layer_pair(config),
                    via_type=via_kind(config), net_id=nid) for x, y in vs]
        out[nid] = {'new_segments': segs, 'new_vias': vias, 'iterations': 0, 'path': [tuple(c) for c in legs[0][0]],
                    'planned': True}
    return out


def tap_nets(pcb, tap_ids, mode=1):
    """THE NETS WHOSE PADS' VIAS THE PLAN LAYS {net id: [(a layer it pours on, where its pour is its own: an object
    with contains_xy(xs, ys), or None for anywhere)]} of the poured nets among `tap_ids`: a zone's outline less the
    zones of other nets drawn over it on its layer at a higher priority. `mode` 2: of a net poured on an inner layer
    too, the inner layers alone -- its outer pours are cut by every track laid across them."""
    zones = [z for z in (getattr(pcb, 'zones', None) or ()) if z.net_id and len(z.polygon or ()) >= 3]
    if not zones:
        return {}
    import shapely
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    cu = list(pcb.board_info.copper_layers)
    outer = {cu[0], cu[-1]} if len(cu) > 2 else set()

    class Where:
        # (inside the outline, and on the main body of the fill: the zone's outline holds islands too -- round an
        # array the lanes cut a pour into slivers no via joins to the plane, and a tap there is no tap)
        def __init__(self, geom, fill=None):
            self.geom = geom
            shapely.prepare(self.geom)
            self.fill = fill
            self.main = fill.largest_component() if fill is not None else 0

        def contains_xy(self, xs, ys):
            xs, ys = np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)
            ok = shapely.contains_xy(self.geom, xs, ys)
            if self.fill is not None and self.main:
                f = self.fill
                i = ((xs - f.x0) / f.cell).astype(int)
                j = ((ys - f.y0) / f.cell).astype(int)
                inb = (i >= 0) & (i < f.nx) & (j >= 0) & (j < f.ny)
                lab = np.zeros(len(xs), dtype=np.int64)
                lab[inb] = f.labels[i[inb], j[inb]]
                ok = ok & (lab == self.main)
            return ok
    from plane_fill_model import get_zone_model
    out = collections.defaultdict(list)
    for z in zones:
        if z.net_id not in set(tap_ids):
            continue
        g = Polygon(z.polygon, [h_ for h_ in (z.holes or ()) if len(h_) >= 3])
        over = [Polygon(o.polygon) for o in zones if o is not z and o.layer == z.layer and o.net_id != z.net_id
                and (o.priority or 0) > (z.priority or 0)]
        if over:
            g = g.difference(unary_union(over))
        if not g.is_empty:
            out[z.net_id].append((z.layer, Where(g.buffer(0), get_zone_model(pcb, z))))
    if mode >= 2:
        out = {n: ([lw for lw in v if lw[0] not in outer] or v) for n, v in out.items()}
    return dict(out)


def tap_results(config, tap_routes):
    """the supply pads' vias the plan kept as route results, as route_results makes a net's: [a result -- its
    tracks at the run's own track width, the plan's pitch --, 'tap': what it joins]"""
    from kicad_parser import Segment, Via
    from via_span import via_layer_pair, via_kind
    out, g = [], config.grid_step
    for nid, name, legs in tap_routes:
        runs, vs = [], []
        for way, e, f in legs:
            r_, v_ = _shape(way, e, g, f)
            runs += r_
            vs += v_
        segs = [Segment(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1], width=config.track_width,
                        layer=config.layers[li], net_id=nid)
                for a, b, li in runs if math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-9]
        vias = [Via(x=x, y=y, size=config.via_size, drill=config.via_drill, layers=via_layer_pair(config),
                    via_type=via_kind(config), net_id=nid) for x, y in vs]
        out.append({'new_segments': segs, 'new_vias': vias, 'iterations': 0, 'path': [tuple(c) for c in legs[0][0]],
                    'planned': True, 'tap': name, 'net_id': nid})
    return out


def plan_run(pcb_data, config, net_ids, net_clearances=None, tap_ids=()):
    """A ROUTE RUN'S PLAN (KICAD_PLAN_TOGETHER=1): whole routes for its nets `net_ids` [(name, id)], left on
    `config._planned_routes` for the run's loop to take (the module's text). Made on a base obstacle map of its own,
    built as the run's is and holding every net but those planned: call it once the run's rules stand (its
    clearances, its guarded pours)."""
    if not env_knobs.PLAN_TOGETHER or not net_ids:
        return
    from obstacle_map import build_base_obstacle_map
    t0 = time.time()
    # The routes are laid at the run's own track and clearance: a net with a pour, a width or a clearance of its own,
    # or with many pads (a rail), is left to the router
    poured = {z.net_id for z in (getattr(pcb_data, 'zones', None) or ())}
    own_w = set(getattr(config, 'power_net_widths', None) or ()) | set(getattr(config, 'net_layer_widths', None) or ())
    ncl = net_clearances if net_clearances is not None else (config.net_clearances or {})
    ids = [nid for _name, nid in net_ids
           if nid in pcb_data.nets and nid not in poured and nid not in own_w
           and ncl.get(nid, config.clearance) <= config.clearance + 1e-9
           and 2 <= len(pcb_data.nets[nid].pads) <= (env_knobs.PLAN_MAX_PADS or MAX_PADS)]
    if len(ids) < 2 and not (env_knobs.PLAN_ONLY and ids):
        if env_knobs.PLAN_ONLY:
            # (asked for the verdict alone, with no net to plan: none is without a way -- said, where saying nothing
            # read as "the check did not run")
            import json
            with open(env_knobs.PLAN_ONLY, 'w', encoding='utf-8') as f:
                json.dump(dict(nets=0, planned=0, boxed=[], shared=[], close=[], partial=[], rounds=0), f)
            raise SystemExit(0)
        return
    taps = tap_nets(pcb_data, [n for n in tap_ids if n not in ids], env_knobs.PLAN_TAPS) if env_knobs.PLAN_TAPS \
        else {}
    base = build_base_obstacle_map(pcb_data, config, ids + sorted(taps), net_clearances=ncl)
    routes, rep = plan_routes(pcb_data, config, ids, base, rounds=env_knobs.PLAN_ROUNDS or ROUNDS,
                              log=print if config.verbose else None, taps=taps)
    config._planned_taps = tap_results(config, rep.pop('tap_routes', []))
    if env_knobs.PLAN_ONLY:
        # (the plan's verdict alone, for a caller that asks which nets have no way on this board: written, and the
        # run ends before it routes anything)
        import json
        with open(env_knobs.PLAN_ONLY, 'w', encoding='utf-8') as f:
            json.dump({k: rep[k] for k in ('nets', 'planned', 'boxed', 'shared', 'close', 'partial', 'rounds')}, f)
        raise SystemExit(0)
    config._planned_routes = route_results(config, routes)
    left = [f"{len(v)} {what} ({', '.join(n.split('/')[-1] for n in v[:6])}{', ...' if len(v) > 6 else ''})"
            for what, v in (('with no way', rep['boxed']), ('on ground another route keeps', rep['shared']),
                            ('too close to a route kept, as drawn', rep['close']),
                            ('not reaching every pad', rep['partial'])) if v]
    print(f"Plan: {rep['planned']} of {rep['nets']} net(s) routed by a plan made of them together "
          f"({rep['rounds']} round(s), {rep['vias']} via(s), {time.time() - t0:.1f} s)"
          + (f"; left to the search: {', '.join(left)}" if left else ''), flush=True)
    if rep.get('taps', {}).get('asked'):
        tl = rep['taps']['left']
        print(f"Plan: {rep['taps']['kept']} of {rep['taps']['asked']} supply pad(s) with no via given one by the same "
              f"plan" + (f"; left to the plane step: {', '.join(n.split('/')[-1] for n in tl[:8])}"
                         f"{', ...' if len(tl) > 8 else ''}" if tl else ''), flush=True)

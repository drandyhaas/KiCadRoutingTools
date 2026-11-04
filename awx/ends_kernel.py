"""ends_kernel -- the ends model's estimate (whole_ends.Ends.score) and the ends search's loop (Ends.search), in Rust
(grid_router.EndsKernel) and nowhere else: two routing layers or more, one destination or a GROUP of them, a plane's
routing layer priced (route_layers.plane_kinds). The ends search scores some 400 000 states a choice.

`build` reads every table the score reads per state off the model, once: each option's point, layer, layer domain
(Ends.end_domain), legs, conflict keys (Ends.conf) and their lanes, hand, far face and fence; each tooth and berth's
ride together (Ends.ride, wound_ride's turns, the far face's ride) and the lane's length (its tooth's exit to its
berth's, what a plane's layer prices it); each pair of options' stacked exits, their runs met (_runs_meet,
_meet_nbrs); the plane ways each option kills (plane_ways); the feedback's tables; a GROUP's walk (whole_frame.Group)
and each berth option's way in -- its place on the walk, the pins its leg passes, the pocket options it cannot be laid
with -- with each part's copper box. The route EXACT on a state's orders is CP-SAT (Ends.exact_route_nl,
exact_route): the kernel calls it back where the estimate would stand (Handle.parts).

A binary without the kernel is refused: rebuild it (python3 build_router.py)."""
import collections
import math
import os
import sys

import pairs as _pairs

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'rust_router')))
try:
    from grid_router import EndsKernel as _EndsKernel
except ImportError as exc:
    raise SystemExit(f'ends_kernel: grid_router has no EndsKernel ({exc}) -- rebuild it: python3 build_router.py')
if not hasattr(_EndsKernel, 'parts'):
    raise SystemExit('ends_kernel: grid_router\'s EndsKernel is older than this awx (no parts) -- rebuild it: '
                     'python3 build_router.py')


def _wcut(E):
    """the model's winding cut as the kernel reads it: None, or (face, coordinate) -- a far-face cut, a bare y, on 'E'"""
    w_ = E.wcut
    if w_ is None:
        return None
    if isinstance(w_, (int, float)):
        return ('E', float(w_))
    return (str(w_[0]), float(w_[1]))


class Handle:
    """the kernel of one model, shared by its copies (cut_phase's), and the learned prices it was last given"""

    def __init__(self, kern, lanes):
        self.k, self.lanes, self.fp = kern, lanes, None

    def sync(self, E):
        fp = (len(E.x_learned), len(E.opp_learned), len(E.end_learned), sum(E.x_learned.values()),
              sum(E.opp_learned.values()), sum(E.end_learned.values()))
        if fp != self.fp:
            li = {l_: i for i, l_ in enumerate(self.lanes)}
            x = [(li[a], li[b], float(v)) for k, v in E.x_learned.items() for a, b in [tuple(k)]
                 if a in li and b in li]
            self.k.set_learned(x, [(li[l_], float(v)) for l_, v in E.opp_learned.items() if l_ in li],
                               bool(E.x_learned), bool(E.opp_learned),
                               [(li[l_], float(v)) for l_, v in E.end_learned.items() if l_ in li],
                               bool(E.end_learned))
            self.fp = fp

    def objective(self, E, state):
        self.sync(E)
        return self.k.objective([state[l_] for l_ in self.lanes],
                                _wcut(E))

    def search(self, E, state, sweeps, tabu):
        """Ends.search in the kernel: (state, objective, [(each sweep's best, its state)])"""
        self.sync(E)
        li = {l_: i for i, l_ in enumerate(self.lanes)}
        st, best, sweeps_ = self.k.search([state[l_] for l_ in self.lanes], sweeps,
                                          _wcut(E),
                                          [(li[l_], e_, i_) for l_, e_, i_ in tabu])
        back = lambda s_: {l_: tuple(s_[i]) for i, l_ in enumerate(self.lanes)}
        return back(st), best, [(b_, back(s_)) for b_, s_ in sweeps_]

    def parts(self, E, state, exact=None):
        """(objective, the kernel's parts by lane index) of `state`; `exact` the exact route's callback"""
        self.sync(E)
        return self.k.parts([state[l_] for l_ in self.lanes],
                            _wcut(E), exact)


def stacked_exits(lanes, opts):
    """{(lane index, option): [(later lane index, its option, weight)]}: the STACKED exits at one end of the lanes
    `lanes` [(lane, legs)] with their options `opts` {lane: options} -- two lanes' moves within DUP_TOL on different
    layers, a weight each pair of options (two where either lane is a pair), the later lane's option listed under the
    earlier's. The kernel counts a stack where either lane changes layer (on an escape state, every stack)"""
    import whole_ends as we
    li = {l_: i for i, (l_, _lg) in enumerate(lanes)}
    npair = [len(lg) for _l, lg in lanes]
    cells = collections.defaultdict(list)
    for la, _lga in lanes:
        for ia, oa in enumerate(opts[la]):
            for m in oa[0]:
                cells[(int(math.floor(m.exit_pt[0] / 0.25)), int(math.floor(m.exit_pt[1] / 0.25)))].append(
                    (li[la], ia, m))
    w_ = collections.Counter()
    for (cx, cy), here in cells.items():
        near = [e for dx in (-1, 0, 1) for dy in (-1, 0, 1) for e in cells.get((cx + dx, cy + dy), ())]
        for a, ia, ma in here:
            for b, ib, mb in near:
                if b <= a or ma.layer == mb.layer:
                    continue
                if math.hypot(ma.exit_pt[0] - mb.exit_pt[0], ma.exit_pt[1] - mb.exit_pt[1]) > we.DUP_TOL:
                    continue
                w_[(a, ia, b, ib)] += 2 if npair[a] > 1 or npair[b] > 1 else 1
    out = collections.defaultdict(list)
    for (a, ia, b, ib), w in sorted(w_.items()):
        out[(a, ia)].append((b, ib, w))
    return out


def build(E):
    """the kernel of model E (an Ends), its tables read off it"""
    import rules as _rules
    import select_moves as _sm
    import whole_ends as we
    import whole_frame as _wf
    import route_layers as _rl
    import braid as _bd
    t0_ = __import__('time').time()
    lanes = [l_ for l_, _lg in E.lanes]
    li = {l_: i for i, l_ in enumerate(lanes)}
    npair = [len(lg) for _l, lg in E.lanes]
    lids, keys = {}, {}
    lid = lambda L: lids.setdefault(L, len(lids))
    for _l, lg in E.lanes:
        for k, opts in ((0, E.T), (1, E.B)):
            for o in opts[_l]:
                for leg, m in zip(lg, o[0]):
                    keys.setdefault((leg, id(m)), len(keys))
    # the runs met (Ends.end_clashes): a via end against each other lane's fixed end at its array, on that end's layer
    # -- the pairs whose run boxes (each grown by the fanout's rule) meet, asked of the Python's own test
    meet = {0: [], 1: []}
    r_ = _rules.active()
    bar = r_.fan_track + r_.fan_clear - 0.01           # (_runs_hit's: a pair whose legs' boxes stand this far apart
    #                                                     along an axis has no two segments nearer)

    def legs_box(moves, layer):
        pts = [p for m in moves for a, b, L_ in (getattr(m, 'legs', None) or ())
               if L_ == (layer if layer is not None else m.layer) for p in (a, b)]
        return (min(p[0] for p in pts), min(p[1] for p in pts), max(p[0] for p in pts), max(p[1] for p in pts)) \
            if pts else None
    two = E.NL == 2       # (two routing layers: _score_at's own route reads no layer domain and no runs met)
    if two:
        assert E.RL[:2] == ['F.Cu', 'B.Cu'], E.RL      # (the kernel reads an end's layer as F.Cu 0, B.Cu 1)
    for k, opts in ((0, E.T), (1, E.B)) if not two else ():
        for la, lga in E.lanes:
            for ia, oa in enumerate(opts[la]):
                fa, _na, oka = E.end_domain(oa, k, lga)
                if fa is not None:
                    continue
                own_a = E.RL.index(oa[2]) if oa[2] in E.RL else None
                ba = legs_box(oa[0], None)
                if ba is None:
                    continue
                for lb, lgb in E.lanes:
                    if lb == la:
                        continue
                    for ib, ob in enumerate(opts[lb]):
                        q, _nb, _okb = E.end_domain(ob, k, lgb)
                        if q is None or q not in oka or q == own_a:
                            continue
                        bb = legs_box(ob[0], E.RL[q])
                        if bb is None or bb[0] - ba[2] >= bar or ba[0] - bb[2] >= bar or bb[1] - ba[3] >= bar \
                                or ba[1] - bb[3] >= bar:
                            continue
                        if E._runs_meet(oa[0], ob[0], E.RL[q]):
                            meet[k].append((li[la], ia, li[lb], ib))
    # the stacked exits: two lanes' moves within DUP_TOL on different layers (stacked_exits)
    stack_of = {k: stacked_exits(E.lanes, opts) for k, opts in ((0, E.T), (1, E.B))}
    # the plane ways (plane_ways): each option's kills, each ball's drops and straps
    pw = E.pw
    bkeys = {}
    balls = []
    if pw is not None and pw.balls:
        for b, _dw, _sw in pw.balls:
            bkeys.setdefault(b, len(bkeys))
        balls = [(bkeys[b], [int(w) for w in dw], [(int(w), bkeys.get(to, -1)) for w, to in sw])
                 for b, dw, sw in pw.balls]
    fence = E.fence
    rows = {}
    for k, opts in ((0, E.T), (1, E.B)):
        rows[k] = []
        for la, lg in E.lanes:
            lane_rows = []
            for i, o in enumerate(opts[la]):
                f_, n_, ok_ = E.end_domain(o, k, lg) if not two else (0, None, None)
                hand = 0
                if len(lg) == 2:
                    (p_, q_) = o[0]
                    hand = int(_pairs.hand(p_.direction, p_.exit_pt, q_.exit_pt, arriving=(k == 1)) or 0)
                d_ = o[0][0].direction if o[0] else None
                far = k == 0 and d_ == E.far_dir
                side_hit = k == 0 and not far and d_ in ('up', 'down') and \
                    any(x_ >= o[1][0] for x_ in fence.get('N' if d_ == 'up' else 'S', ()))
                nbrs = []
                if f_ is None:
                    for lb, ibs in E._meet_nbrs(la, k, i).items():
                        nbrs.append((li[lb], sorted(ibs)))
                kills = sorted(int(w) for w in (pw.kill.get((la, k, i), ()) if pw is not None else ()))
                lane_rows.append((
                    (float(o[1][0]), float(o[1][1]), E.RL.index(o[2]) if o[2] in E.RL else -1, lid(o[2]), int(o[3]),
                     bool(o[4])),
                    (-1 if f_ is None else int(f_), -1 if n_ is None else int(n_),
                     sum(1 << q for q in (ok_ or ())), bool(we._via_end(o)), hand, bool(far), bool(side_hit)),
                    [(float(m.exit_pt[0]), float(m.exit_pt[1]), int(m.vias)) for m in o[0]],
                    [keys[(leg, id(m))] for leg, m in zip(lg, o[0])],
                    kills, nbrs, stack_of[k].get((li[la], i), []),
                    sorted((li[ln], j) for ln, j in E.oconf.get((la, k, i), ()))))
            rows[k].append(lane_rows)
    conf = [[] for _ in keys]
    for key, kid in keys.items():
        conf[kid] = sorted(keys[c] for c in E.conf.get(key, ()) if c in keys)
    # the rides: each tooth with each berth, the short way (None) and wound round the north and the south; the wound
    # turns (wound_ride's) and the far face's extra (_far_ride's term in its brackets) each side of the source's cut
    import plan_ends as pe
    sb = E.st['sgrid'].bbox
    pad = _bd.LPITCH
    ride, nturn, farx = [], [], []
    for la, lg in E.lanes:
        r_, t_, f_ = [], [], []
        for ti, to_ in enumerate(E.T[la]):
            for bi, bo_ in enumerate(E.B[la]):
                r_ += [E.ride(la, len(lg), ti, bi, s) for s in (None, 'N', 'S')]
                t_ += [int(we.wound_ride(to_[1], bo_[1], E.st['dboxes'], s, pad=E.ring_d0, turns=True)[1])
                       for s in ('N', 'S')]
                if to_[0][0].direction == E.far_dir:
                    a, b = to_[1], bo_[1]
                    # (and the laid lanes of another bus that side of the cut passes, FENCE_LAID each, as ride)
                    for c, nx in zip(((sb[0] - pad, sb[1] - pad), (sb[0] - pad, sb[3] + pad)), E.laid_x[la][ti]):
                        f_.append(math.hypot(c[0] - a[0], c[1] - a[1]) + pe.sm.around_box(c, b, sb)
                                  - pe.sm.around_box(a, b, sb) + we.FENCE_LAID * we.VIA_MM * nx)
                else:
                    f_ += [0.0, 0.0]
        ride.append(r_); nturn.append(t_); farx.append(f_)
    lane_of_leg = {leg: li[l_] for l_, lg in E.lanes for leg in lg}
    key_lane = [0] * len(keys)
    for (leg, _mid), kid in keys.items():
        key_lane[kid] = lane_of_leg[leg]
    # a destination GROUP (whole_frame.Group): its walk, and each berth option's way in
    G = E.group
    parts_ = sorted({E.dest_of[l_] for l_ in lanes}) if G is not None else []
    gspec = dict(g_on=False, g_len=0.0, g_f0=0.0, g_f1=0.0, g_bbox=(0.0, 0.0, 0.0, 0.0), g_s=[], g_pass=[], g_pconf=[],
                 g_part=[], g_pbox=[], g_leg_w=0.0)
    if G is not None:
        gspec = dict(
            g_on=True, g_len=float(G.L), g_f0=float(G.f0), g_f1=float(G.f1), g_bbox=tuple(map(float, G.bbox)),
            g_s=[[float(o[5][1]) for o in E.B[l_]] for l_ in lanes],
            g_pass=[[int(o[5][5]) for o in E.B[l_]] for l_ in lanes],
            g_pconf=[[sorted((li[o_], j_) for o_, j_ in E.pconf.get((l_, i), ())) for i in range(len(E.B[l_]))]
                     for l_ in lanes],
            g_part=[parts_.index(E.dest_of[l_]) for l_ in lanes],
            g_pbox=[tuple(map(float, _wf.box_of(E.st['pcb'], r_))) for r_ in parts_],
            g_leg_w=float(_wf.GROUP_LEG_PASS))
    dense = lambda l_, k, ix: [float(ix.get(i, 0.0)) for i in range(len((E.T if k == 0 else E.B)[l_]))]
    # (the fronts, a side-face tooth's laid lanes of another bus, FENCE_LAID each, and its legs out of the array's other
    # half, AWAY_FACE each: one more price per option)
    front = dict(E.front)
    for l_, lg in E.lanes:
        lx_ = {i: we.FENCE_LAID * len(lg) * n_ for i, (n_, _n) in E.laid_x[l_].items()
               if n_ and not we._far_opt(E, E.T[l_][i])}
        for i, n_ in (E.away.get(l_) or {}).items():
            lx_[i] = lx_.get(i, 0.0) + we.AWAY_FACE * n_
        if lx_:
            fr_ = dict(front.get((l_, 0)) or {})
            for i, v_ in lx_.items():
                fr_[i] = fr_.get(i, 0.0) + v_
            front[(l_, 0)] = fr_
    spec = dict(
        nl=E.NL, nlanes=len(lanes), npair=npair,
        tie=[sum(E.tie[g] for g in lg) for _l, lg in E.lanes],
        fb_over=[int(E.fb_over.get(l_, 0)) for l_ in lanes], has_fb_over=bool(E.fb_over),
        teeth=rows[0], berths=rows[1], conf=conf, nkeys=len(keys), meet0=meet[0], meet1=meet[1],
        ride=ride, nturn=nturn, farx=farx,
        balls=balls, nball_keys=len(bkeys), nways=(len(pw.ways) if pw is not None else 0),
        has_pw=bool(pw is not None and pw.balls),
        fb_avoid=[(li[l_], k, dense(l_, k, ix)) for (l_, k), ix in E.fb_avoid.items()],
        fb_pairs=[(li[a], li[b], k, dense(a, k, ia), dense(b, k, ib)) for a, b, k, ia, ib in E.fb_pairs],
        front=[(li[l_], k, dense(l_, k, c_)) for (l_, k), c_ in front.items()],
        far_w=[float(E.far_w[l_]) for l_ in lanes],
        sbox=tuple(map(float, E.sbox)), dbox=tuple(map(float, E.dbox)),
        fence=(len(fence.get('N', ())), len(fence.get('S', ()))), escape=bool(E.st.get('escape')),
        neumaier=sys.version_info >= (3, 12),   # (sum() of floats compensated: the kernel sums as this Python does)
        pk=[{'-': 0, 'pwr': 1, 'gnd': 2}[k_] for k_ in E.PK], p_gnd=float(_rl.gnd_run_vias(E.inner_signal)),
        p_pwr=float(_rl.run_price('pwr', E.inner_signal)), key_lane=key_lane, **gspec,
        consts=[we.W_OVER, we.VIA_PREF, we.STACK_COST, we.VIA_MM, we.LOAD_OK, we.W_CONG, we.X_TRUNK, we.FB_AVOID,
                we.FB_PAIR, we.EPS_X, we.BIG, we.WALLED, we.FENCE_VIAS, we.OPP_DIVE,
                we._LANE_PITCH, we._CHG_ROOM, _bd.LPITCH, we.TIE_MM])
    h = Handle(_EndsKernel(spec), lanes)
    h.lid_name = {v: k for k, v in lids.items()}     # (a layer id's name: the two-layer exact route reads names)
    h.secs = __import__('time').time() - t0_
    return h

"""ends_kernel -- the ends model's estimate (whole_ends.Ends._score: two routing layers or more, one destination)
scored in Rust (grid_router.EndsKernel). The ends search scores some 400 000 states a choice -- zynq7020_som's first,
200 of its 240 CPU seconds at half a millisecond each in Python, its four threads taking turns at the GIL.

`build` reads every table the score reads per state off the model, once: each option's point, layer, layer domain
(Ends.end_domain), legs, conflict keys (Ends.conf), hand, far face and fence; each tooth and berth's ride together
(Ends.ride, wound_ride's turns, the far face's ride); each pair of options' stacked exits (_stacks), their runs met
(_runs_meet, _meet_nbrs); the plane ways each option kills (plane_ways); the feedback's tables. The kernel forms the
objective from them with the same operations in the same order as the Python (rust_router/src/ends_kernel.rs), so the
same doubles.

ENDS_KERNEL: '1' (default) the kernel where it reaches -- Ends.objective, the search's question; the parts of a state
kept (Ends.score) are still the Python's -- '0' the Python alone, 'check' both on every state, a difference raised
with the two sets of terms. On two routing layers it scores _score_at's own route (the cover, the settling, the
coupling); it does not reach a branched bus, the risk price (ENDS_CORNER) or the exact route: the Python scores those."""
import collections
import math
import os
import sys

import awx_settings
import pairs as _pairs

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'rust_router')))
try:
    from grid_router import EndsKernel as _EndsKernel
except ImportError:          # (an older binary: the Python scores)
    _EndsKernel = None

MODE = awx_settings.get('ENDS_KERNEL') or '1'
TERMS = ('objective', 'fan', 'route', 'over', 'ride', 'cong', 'feedback', 'crossings', 'conflicts', 'splits', 'refused',
         'stacks', 'front', 'walled', 'learned', 'far', 'fence')


def reaches(E):
    import whole_ends as we
    return _EndsKernel is not None and MODE != '0' and E.NL >= 2 and not E.multi and not we.CORNER_ON


class Handle:
    """the kernel of one model, shared by its copies (cut_phase's), and the learned prices it was last given"""

    def __init__(self, kern, lanes):
        self.k, self.lanes, self.fp = kern, lanes, None

    def sync(self, E):
        fp = (len(E.x_learned), len(E.opp_learned), sum(E.x_learned.values()), sum(E.opp_learned.values()))
        if fp != self.fp:
            li = {l_: i for i, l_ in enumerate(self.lanes)}
            x = [(li[a], li[b], float(v)) for k, v in E.x_learned.items() for a, b in [tuple(k)]
                 if a in li and b in li]
            self.k.set_learned(x, [(li[l_], float(v)) for l_, v in E.opp_learned.items() if l_ in li],
                               bool(E.x_learned), bool(E.opp_learned))
            self.fp = fp

    def objective(self, E, state):
        self.sync(E)
        return self.k.objective([state[l_] for l_ in self.lanes],
                                None if E.wcut is None else (str(E.wcut[0]), float(E.wcut[1])))

    def search(self, E, state, sweeps, tabu):
        """Ends.search in the kernel: (state, objective, [(each sweep's best, its state)])"""
        self.sync(E)
        li = {l_: i for i, l_ in enumerate(self.lanes)}
        st, best, sweeps_ = self.k.search([state[l_] for l_ in self.lanes], sweeps,
                                          None if E.wcut is None else (str(E.wcut[0]), float(E.wcut[1])),
                                          [(li[l_], e_, i_) for l_, e_, i_ in tabu])
        back = lambda s_: {l_: tuple(s_[i]) for i, l_ in enumerate(self.lanes)}
        return back(st), best, [(b_, back(s_)) for b_, s_ in sweeps_]

    def terms(self, E, state):
        self.sync(E)
        return self.k.terms([state[l_] for l_ in self.lanes],
                            None if E.wcut is None else (str(E.wcut[0]), float(E.wcut[1])))


def build(E):
    """the kernel of model E (an Ends), its tables read off it"""
    import rules as _rules
    import select_moves as _sm
    import whole_ends as we
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
    # the stacked exits (_stacks): two lanes' moves within DUP_TOL on different layers, a weight each pair (two where
    # either lane is a pair), per pair of options -- the later lane's option listed under the earlier's
    stack_of = {0: collections.defaultdict(list), 1: collections.defaultdict(list)}
    for k, opts in ((0, E.T), (1, E.B)):
        cells = collections.defaultdict(list)
        for la, lga in E.lanes:
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
        for (a, ia, b, ib), w in sorted(w_.items()):
            stack_of[k][(a, ia)].append((b, ib, w))
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
                    for c in ((sb[0] - pad, sb[1] - pad), (sb[0] - pad, sb[3] + pad)):
                        f_.append(math.hypot(c[0] - a[0], c[1] - a[1]) + pe.sm.around_box(c, b, sb)
                                  - pe.sm.around_box(a, b, sb))
                else:
                    f_ += [0.0, 0.0]
        ride.append(r_); nturn.append(t_); farx.append(f_)
    dense = lambda l_, k, ix: [float(ix.get(i, 0.0)) for i in range(len((E.T if k == 0 else E.B)[l_]))]
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
        front=[(li[l_], k, dense(l_, k, c_)) for (l_, k), c_ in E.front.items()],
        sbox=tuple(map(float, E.sbox)), dbox=tuple(map(float, E.dbox)),
        fence=(len(fence.get('N', ())), len(fence.get('S', ()))), escape=bool(E.st.get('escape')),
        neumaier=sys.version_info >= (3, 12),   # (sum() of floats compensated: the kernel sums as this Python does)
        consts=[we.W_OVER, we.VIA_PREF, we.STACK_COST, we.VIA_MM, we.LOAD_OK, we.W_CONG, we.X_TRUNK, we.FB_AVOID,
                we.FB_PAIR, we.EPS_X, we.BIG, we.WALLED, we.FAR_TOOTH, we.FENCE_VIAS, we.OPP_DIVE, we.CORNER,
                we._LANE_PITCH, we._CHG_ROOM, _bd.LPITCH, we.TIE_MM])
    h = Handle(_EndsKernel(spec), lanes)
    h.secs = __import__('time').time() - t0_
    return h


def check(E, state, v_py, parts):
    """ENDS_KERNEL=check: the kernel's objective of `state` against the Python's `v_py` (its `parts`), exactly"""
    t = E._kh.terms(E, state)
    if t[0] == v_py:
        return
    py = dict(objective=v_py, fan=parts.get('fan'), route=parts.get('route'), over=parts.get('over'),
              ride=parts.get('ride'), cong=parts.get('cong'), feedback=parts.get('feedback'),
              crossings=parts.get('crossings'), conflicts=parts.get('conflicts'), splits=parts.get('splits'),
              refused=parts.get('refused'), stacks=parts.get('stacks'), front=parts.get('front'),
              walled=len(parts.get('walled') or ()), learned=parts.get('learned'), far=parts.get('far'),
              fence=parts.get('fence'))
    rows = '; '.join(f'{n} {py[n]!r} vs {t[i]!r}' for i, n in enumerate(TERMS))
    raise SystemExit(f'ends_kernel: the kernel scored {t[0]!r} where the Python scored {v_py!r} (wcut {E.wcut}, '
                     f'state {state}): {rows}')

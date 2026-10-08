"""whole_ends.py -- the whole route's own choice of ENDS: each net's tooth at the source and its berth at the
destination, chosen together from the fanout's escape menus (fanout_from_plan.plan_state: the destination menu, the
source menu, the teeth as laid), so the braid they make is cheap. fanout_from_plan's PLAN_JUDGE=ends chooses and
judges with it.

The ends decide the braid: the order of the teeth round the source and of the berths round the destination (read as
the whole frame reads them: whole_frame's boxes, cut and unrolled perimeters) fix every crossing, and the layers at
the two ends fix what a crossing costs. Two lanes cross only on different layers, so a crossing between two lanes on
ONE layer end to end sends one of them off its layer and back -- two changes; a lane whose end layers differ changes
once and can take its crossings on either side of that change; an OPPOSITE-HANDS pair on one layer end to end crosses
over at a dive (two changes) and takes its crossings there. A pair's change is two vias.

The route's changes are ESTIMATED from the ends alone, for the search: the end-layer changes and crossovers; two per
lane of a least-weight cover of the crossings between lanes on one layer end to end (exact: a permutation graph); then
the SETTLING of each lane whose end layers differ -- its one change serves its crossings with lanes on one layer end
to end only if none it must meet on its end layer is met, in a forced order, before one it must meet on its start
layer; broken, it takes an excursion or the fewest of those partners dive (an exact cut) -- and the COUPLING of two
such lanes that do not cross each other, when a pair of partners crossing each other must be met by them in orders no
single place of the partners' own crossing gives. Two lanes whose ends stand at one place round a box (an F end
stacked over a B one) have no order there. The best few states the searches reach are then ranked on the route EXACT
on their orders (exact_route: the whole solve's order model without its lengths, CP-SAT on one worker to a
deterministic work limit), and a state's judged objective is that one.

The objective is one sum in vias, as the whole solve prices them: the vias -- the ends' own (a tie via at a ball with
a pad of its own under it among them) and the route's -- with W_OVER more for each via a net carries past two on the
board (its stubs' own vias and its lane's changes; the tie via not counted toward the two), each lane's ride (round the
two arrays, and each leg's straight stubs from its balls to its exits) at select_moves.VIA_MM per via, CONGESTION on
the trunk (each lane's load there past what the solve takes freely, and each crossing there), STACKING (two lanes' ends
at one point on different layers where either changes layer, a pair's twice), the whole route's FEEDBACK (ends it
found crowded or named, whole_feedback: priced by place, less on the other layer there and falling off with distance,
and doubled each later round an end is named again), and the crossings' count as the tie-break.

Searched locally -- one lane's tooth or berth at a time, one other lane
ejected where it is in the way, each improving change taken as it is found, sweep after sweep until none is left,
then iterated from random kicks until ILS_PATIENCE kicks in a row find nothing, then from BAN kicks (the most crossed
lanes' current ends banned) -- first with the teeth as laid, then with the teeth free from the berths just chosen; tooth
moves are asked only when they save more than TOOTH_GAIN. A state whose exact route finds no plan ranks after every
one whose does. Never taken: two moves the fanout cannot lay together
(select_moves._conflict, an F exit stacked over a B one allowed; a tooth move through another run net's LAID tooth;
two berths a destination pass laid in violation of each other); a pair's legs on different faces or layers, or exits
not neighbours; copper of a net outside the run between a pair's tips; a tooth on the source's far face (the whole
frame has no way round the source); a pair with another lane's end on its layer between its tips; a pair's joint move
the fanout refused.

usage: whole_ends.py BOARD NETS|@FILE  -- the ends on the teeth as laid, and with the teeth free, the objective's parts
for each (the fanout's menus as PLAN_JUDGE=ends builds them)
"""
import collections
import itertools
import math
import os
import awx_settings
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))

import pairs as _pairs  # noqa: E402
import braid as _bd  # noqa: E402
import select_moves as _sm  # noqa: E402
_LANE_PITCH = _bd.LANE_MIN                   # a crossing's length along a lane holding its line
_CHG_ROOM = 2 * 1.1 * _bd.VIA_NEED           # a layer change's room along its lane (whole_solve: VR_STAY x a via's room)

EPS_X = 0.01              # a crossing: the tie-break between ends of one via count
TOOTH_GAIN = 0.5          # tooth moves are asked only for at least half a via (a realize costs a fanout run)
DUP_TOL = _sm._STACK_PITCH / 2   # two exits this close are one (two distinct ones on a layer stand a pitch apart)
ILS_ROUNDS = 12           # iterated local search: kicks from the best state (a count, never a clock)
ILS_KICK = 3              # ... each moving this many lanes' ends to random options
ILS_PATIENCE = 4          # ... stopping after this many kicks in a row that find nothing (K15: none of 12 did)
BAN_KICKS = 8             # then BAN kicks (ban_kicks): the most crossed lanes' current ends banned, searched again ...
BAN_LANES = 3             # ... this many lanes a kick
BAN_PATIENCE = 4          # ... stopping after this many kicks in a row that find nothing
ILS_SEED = 622
VIA_PREF = 2              # no more than two vias on a net where that can be had (whole_solve's first objective)
W_OVER = 5.0              # each via a net carries past two costs this many MORE (nets of 0..4 vias: 0, 1, 2, 8, 14) --
#                           a price, never a cap, the same as the whole solve's
STACK_COST = 5.0          # two lanes' ends at one point on different layers where either changes layer (_stacks)
VIA_MM = _sm.VIA_MM       # a via is worth this much ride (the one exchange rate)
_PARTS_KEEP = 256          # score: the states whose parts are kept (the search's kept states' are asked again soon)
EXACT_TOP = 6             # the search's best distinct ends re-ranked on the exact route (best_exact) ...
EXACT_MARGIN = 4.0        # ... those within this much of the best by the estimate
EXACT_KMAX = 3            # a lane's changes at most in the exact route (the whole solve's KMAX)
WIND_TRY = 3              # WINDING (WIND_CUT): the cuts off the far face searched again, the best few by the estimate
WIND_GAIN = 1.0           # ...a cut taken only when its ends lay fewer vias and beat the far face's by this much: within
#                           it, the model's own error on a wound plan (its rides, the stubs the fanout lays) -- h3 K28
#                           took two cuts at 1.2 and 0.18 and laid 30 vias and 584 mm against the far cut's 32 and 544
EXACT_WORK = 60.0         # CP-SAT's deterministic work limit for one exact route (a machine's deterministic time is
#                           its own: H3 K51's best ends proved at 17 on a Mac and stopped unproved at 20 on Linux)
LOAD_OK = 0.5             # a lane's congestion on the trunk (score) the solve takes freely ...
W_CONG = 20.0             # ... past it, vias per the square of the overload
X_TRUNK = 0.2             # a crossing on the trunk, in vias
FB_AVOID = 5.0            # an end the whole route's audits found crowded (whole_feedback), in vias ...
FB_PAIR = 10.0            # ... two ends found crowded together -- each by place (fb_weights): in full at the place ...
FB_OTHER_LAYER = 0.5      # ... this much on the other layer there (a layer change is some answer), and at most this
#                           much off the place (moving away a better one) ...
FB_RADIUS = 0.76          # ... falling to nothing this far beyond the place ...
FB_ESCALATE = 2.0         # ... and this many times more each later round it is named again
BIG = 100000.0            # a conflict, a split pair or a refused end: never taken
FRONT_REACH = 1.2         # a single lane's exit: its straight run out, this far along its escape (as a pair's,
#                           fanout_from_plan.PAIR_EXIT_REACH), against copper outside the run on its layer (exit_front) --
FRONT_VIA = 1.0           # ... blocked, with room for a via before the block: the lane changes layer there ...
FRONT_BLOCKED = 1000.0    # ... blocked with no room for one: past any via count, yet no refusal (a lane with no other end
#                           keeps it)
WALLED = 1000.0           # a plane ball of the arrays its ends leave no way down (the joint fanout: plane_ways) -- past
#                           any via count, yet no refusal
FAR = {'left': (-1, 0), 'right': (1, 0), 'up': (0, -1), 'down': (0, 1)}


def _seg_d(a, b, p, q):
    """the least distance between segments ab and pq"""
    def pt(u, v, w):
        dx, dy = w[0] - v[0], w[1] - v[1]
        L = dx * dx + dy * dy
        t = 0.0 if L == 0 else max(0.0, min(1.0, ((u[0] - v[0]) * dx + (u[1] - v[1]) * dy) / L))
        return math.hypot(v[0] + t * dx - u[0], v[1] + t * dy - u[1])

    def cross(a, b, p, q):
        o = lambda u, v, w: (v[0] - u[0]) * (w[1] - u[1]) - (v[1] - u[1]) * (w[0] - u[0])
        return o(a, b, p) * o(a, b, q) < 0 and o(p, q, a) * o(p, q, b) < 0
    if cross(a, b, p, q):
        return 0.0
    return min(pt(a, p, q), pt(b, p, q), pt(p, a, b), pt(q, a, b))


def min_cut_cover(edges, wt):
    """the least-weight vertex cover of bipartite `edges` [(left, right)] (weights `wt`): a minimum s-t cut"""
    cap = collections.defaultdict(float)
    adj = collections.defaultdict(set)
    left, right = sorted({a for a, _b in edges}), sorted({b for _a, b in edges})

    def add(u, v, c):
        cap[(u, v)] += c; adj[u].add(v); adj[v].add(u)
    for a in left:
        add('s', ('L', a), wt[a])
    for b in right:
        add(('R', b), 't', wt[b])
    for a, b in edges:
        add(('L', a), ('R', b), math.inf)
    while True:
        prev = {'s': None}
        q = collections.deque(['s'])
        while q and 't' not in prev:
            u = q.popleft()
            for v in sorted(adj[u], key=repr):
                if v not in prev and cap[(u, v)] > 1e-12:
                    prev[v] = u; q.append(v)
        if 't' not in prev:
            break
        f, v = math.inf, 't'
        while prev[v] is not None:
            f = min(f, cap[(prev[v], v)]); v = prev[v]
        v = 't'
        while prev[v] is not None:
            cap[(prev[v], v)] -= f; cap[(v, prev[v])] += f; v = prev[v]
    return [a for a in left if ('L', a) not in prev] + [b for b in right if ('R', b) in prev]


class Ends:
    """the lanes of one plan state (a pair one lane) with their tooth and berth options, and the objective"""

    def __init__(self, st, src_free=True, fixed=None, learned=None, free_teeth=None):
        import pages_first as pf
        import source_realize as sr
        self.st = st
        names = [n for n in st['launch'] if st['dmenu'].get(n)]
        prs = {}
        if int(awx_settings.get('PLAN_PAIRS', awx_settings.get('BRAID_PAIRS', '0')) or 0):
            prs = {b: pr for b, pr in _pairs.pair_names(names).items() if pr[0] in names and pr[1] in names}
        legs = {l_ for pr in prs.values() for l_ in pr}
        self.lanes = [(n, (n,)) for n in names if n not in legs] + [(b, pr) for b, pr in prs.items()]
        self.legs_of = dict(self.lanes)
        sb, db = st['sgrid'].bbox, st['dgrid'].bbox
        cs = ((sb[0] + sb[2]) / 2, (sb[1] + sb[3]) / 2)
        cd = ((db[0] + db[2]) / 2, (db[1] + db[3]) / 2)
        if cd[0] <= cs[0] or abs(cd[1] - cs[1]) > cd[0] - cs[0]:
            raise SystemExit('whole_ends: the bench is not in the canonical frame (source to destination along +x)')
        far_dir = min(FAR, key=lambda d_: FAR[d_][0] * (cd[0] - cs[0]) + FAR[d_][1] * (cd[1] - cs[1]))
        cur = {n: pf.current_tooth(st, n) for n in names}
        # ---- each lane's options at each end: (moves per leg, point, layer, vias)
        rt = 1.3 * max(st['sgrid'].pitch_x, st['sgrid'].pitch_y)
        rb = 1.3 * max(st['dgrid'].pitch_x, st['dgrid'].pitch_y)

        def combos(menus, reach, forced=False):
            # (moves per leg, point, layer, vias, refused); `forced`: the ends as they stand, one option, REFUSED
            # when the whole route cannot take it (a pair's legs on different faces or layers, or not neighbours)
            if len(menus) == 1:
                return [((m,), m.exit_pt, m.layer, m.vias, False) for m in menus[0]]
            out = []
            for a in menus[0]:
                for b in menus[1]:
                    d = math.hypot(a.exit_pt[0] - b.exit_pt[0], a.exit_pt[1] - b.exit_pt[1])
                    ok = (a.direction, a.layer) == (b.direction, b.layer) and DUP_TOL < d <= reach
                    if ok or forced:
                        out.append(((a, b), ((a.exit_pt[0] + b.exit_pt[0]) / 2, (a.exit_pt[1] + b.exit_pt[1]) / 2),
                                    a.layer, a.vias + b.vias, not ok))
            return out
        fixed = fixed or {}
        banned = st.get('banned') or set()

        def unbanned(lg, opts):
            # a pair's options without the joint moves the fanout would not lay (fanout_from_plan.ban_moves)
            if len(lg) < 2:
                return opts
            return [o for o in opts if ('pair', lg[0], lg[1], sr.move_sig(o[0][0]), sr.move_sig(o[0][1])) not in banned
                    and ('pairclass', lg[0], lg[1], sr.move_class(o[0][0]), sr.move_class(o[0][1])) not in banned]

        def same_as_laid(m, c):
            # a menu move that IS the laid tooth: its kind, face and layer, its exit within DUP_TOL (the menu's exit
            # stands on the array's edge line, the laid stub's end a hair off it) -- asked, it was realized as it
            # stood, three rounds running at K15 ('= original')
            return (c is not None and m.kind == c.kind and m.direction == c.direction and m.layer == c.layer
                    and math.hypot(m.exit_pt[0] - c.exit_pt[0], m.exit_pt[1] - c.exit_pt[1]) <= DUP_TOL)
        self.T, self.B = {}, {}
        import route_layers
        self.RL = list(route_layers.layers())           # (the routing layers: F.Cu, B.Cu, the inner ones)
        self.NL = len(self.RL)
        self._runs_ok = {}                              # (more than two) a via end's moves -> the layers its runs fit
        self._meet = {}                                 # (more than two) two ends' moves -> their runs within the rule
        vias_only = route_layers.escape_vias('src')
        for lane, lg in self.lanes:
            # (`free_teeth`: an incremental fanout frees only the teeth its feedback names; the rest stand as laid)
            held = not src_free or (free_teeth is not None and lane not in free_teeth)
            if held and all(cur[l_] is not None for l_ in lg):
                # the teeth as laid: the one option, refused on the source's far face
                # (ESCAPE_VIAS: a tooth laid as a surface escape refused, so a re-fan through a via is not judged
                # worse than it for the via it adds)
                self.T[lane] = [o[:4] + (o[4] or o[0][0].direction == far_dir
                                         or (vias_only and any(m.kind == 'surface' for m in o[0])),)
                                for o in combos([[cur[l_]] for l_ in lg], rt, forced=True)]
            else:
                # (ESCAPE_VIAS: a tooth laid as a surface escape is no option where its menu has a via's)
                tm = [[m for m in ([cur[l_]] if cur[l_] is not None and not (
                       vias_only and cur[l_].kind == 'surface' and st['smenu'].get(l_)) else [])
                       + [m for m in st['smenu'].get(l_, []) if not same_as_laid(m, cur[l_])]
                       if m.direction != far_dir] for l_ in lg]
                self.T[lane] = unbanned(lg, combos(tm, rt))
                if not self.T[lane]:
                    # nothing clean to offer (a pair's legs with no clean combination, every joint move banned, the laid
                    # tooth on the far face): the options as they stand, REFUSED -- a lane with none stops the search
                    self.T[lane] = (combos(tm, rt, forced=True) if all(tm) else []) or \
                        ([o[:4] + (True,) for o in combos([[cur[l_]] for l_ in lg], rt, forced=True)]
                         if all(cur[l_] is not None for l_ in lg) else [])
            dm = [[m for m in st['dmenu'][l_] if l_ not in fixed or sr.move_sig(m) == fixed[l_]] or list(st['dmenu'][l_])
                  for l_ in lg]
            forced = all(l_ in fixed for l_ in lg)
            self.B[lane] = (combos(dm, rb, forced=forced) if forced else
                            unbanned(lg, combos(dm, rb)) or combos(dm, rb, forced=True))     # (nothing clean: refused)
        # a PAIR's end is refused where copper of a net OUTSIDE the run stands between its two tips on its layer: the
        # pair cannot be coupled there. A run net's stub there is no refusal: it moves with the search, and the score's
        # split count sees whichever of its options stands between the tips (the bench's SA6 stub between SCK's teeth
        # refused the teeth the human laid, and SA6's own menu has the move that clears them)
        run_ids = {st['byname'][l_][0] for _lane, lg in self.lanes for l_ in lg}
        fx = [(sg.layer, (sg.start_x, sg.start_y), (sg.end_x, sg.end_y), sg.width / 2, sg.net_id)
              for sg in st['pcb'].segments if sg.net_id not in run_ids]
        fx += [(L, (v.x, v.y), (v.x, v.y), v.size / 2, v.net_id) for v in st['pcb'].vias if v.net_id not in run_ids
               for L in route_layers.layers()]

        def between_tips(o, lg):
            (a, b), L = [m.exit_pt for m in o[0]], o[2]
            own = {st['byname'][l_][0] for l_ in lg}
            for L_, p, q, r, nid in fx:
                if L_ != L or nid in own:
                    continue
                if _seg_d(a, b, p, q) < _bd.TRACK / 2 + _bd.SPEC_CLEARANCE + r:
                    return True
            return False
        for lane, lg in self.lanes:
            if len(lg) == 2:
                self.T[lane] = [o[:4] + (o[4] or between_tips(o, lg),) for o in self.T[lane]]
                self.B[lane] = [o[:4] + (o[4] or between_tips(o, lg),) for o in self.B[lane]]
        # a SINGLE lane's end priced by what stands in front of its exit on its layer (exit_front): a pair leg's menu
        # keeps only exits with room for the pair there (fanout_from_plan.pair_exit_clear), a single's was checked
        # only to its tooth's tip -- and the loop's audits then found it against a passive the solve cannot move it
        # off (zynq: C105 0.6 mm in front of the source teeth, C98 beside the berths -- most of the static findings)
        self.front, self.run_ids = {}, run_ids
        for lane, lg in self.lanes:
            if len(lg) != 1:
                continue
            for end, opts in ((0, self.T[lane]), (1, self.B[lane])):
                c_ = {i: (0.0, FRONT_VIA, FRONT_BLOCKED)[exit_front(st['pcb'], st['byname'][lg[0]][0], o[0][0],
                                                                    run_ids)]
                      for i, o in enumerate(opts)}
                if any(c_.values()):
                    self.front[(lane, end)] = c_
        self.cur = cur
        # (the joint fanout's plane balls of the arrays, each kept a way down: plane_ways)
        self.pw = plane_ways(st, self.lanes, self.T, self.B, cur)
        # a TIE VIA at the ball where a pad of the net's own lies under it on the other layer (fanout_from_plan.
        # tie_vias_under): one more via on that leg whatever its escapes
        self.tie = {l_: int(any(_pairs.under_pad(st['dst_pad'][l_], q, _bd.VIA_SIZE) for q in st['byname'][l_][1].pads))
                    for _l, lg in self.lanes for l_ in lg}
        # ---- the pad boxes the orders run round (grown, per state, by where its ends stand -- whole_frame's own)
        import whole_frame
        self.sbox, self.dbox = whole_frame.box_of(st['pcb'], st['sref']), whole_frame.box_of(st['pcb'], st['dref'])
        # ---- the moves the fanout cannot lay together
        kept = {0: collections.defaultdict(dict), 1: collections.defaultdict(dict)}
        for lane, lg in self.lanes:
            for end, opts in ((0, self.T[lane]), (1, self.B[lane])):
                for o in opts:
                    for leg, m in zip(lg, o[0]):
                        kept[end][leg][id(m)] = m
        conf = collections.defaultdict(set)
        for cands, strict in (({n: list(kept[0][n].values()) for n in names}, True),
                              ({n: list(kept[1][n].values()) for n in names}, bool(pf.PAGES_STRICT))):
            # stacked: the whole route orders each layer's lanes on their own, so an F exit over a B one is
            # no conflict
            for (a, i, b, j) in pf._conflicts(cands, strict, stack=True):
                conf[(a, id(cands[a][i]))].add((b, id(cands[b][j])))
                conf[(b, id(cands[b][j]))].add((a, id(cands[a][i])))
        # ...and a tooth move through another run net's LAID tooth (fanout_from_plan: sblock): a conflict with that
        # laid tooth alone -- the net's other options are free of it
        for (a, ida), bs in st.get('sblock', {}).items():
            for b in bs:
                if b != a and cur.get(b) is not None:
                    conf[(a, ida)].add((b, id(cur[b])))
                    conf[(b, id(cur[b]))].add((a, ida))
        # ...and the berth pairs a destination pass laid exactly but in violation of each other (`learned`: frozensets
        # of two move signatures, fanout_from_plan.fanout_destination): those two moves together, not either alone
        if learned:
            by_sig = collections.defaultdict(list)
            for n in names:
                for m in st['dmenu'][n]:
                    by_sig[sr.move_sig(m)].append((n, id(m)))
            for pr in learned:
                if len(pr) != 2:
                    continue
                sa, sb = tuple(pr)
                for (a, ia) in by_sig.get(sa, ()):
                    for (b, ib) in by_sig.get(sb, ()):
                        if a != b:
                            conf[(a, ia)].add((b, ib))
                            conf[(b, ib)].add((a, ia))
        self.conf = conf
        # ...and per OPTION: the other lanes' options (same end) it cannot be laid with -- a hit test is then a lookup
        where = collections.defaultdict(list)
        for lane, lg in self.lanes:
            for end, opts in ((0, self.T[lane]), (1, self.B[lane])):
                for i, o in enumerate(opts):
                    for leg, m in zip(lg, o[0]):
                        where[(leg, id(m), end)].append((lane, i))
        self.oconf = {}
        for lane, lg in self.lanes:
            for end, opts in ((0, self.T[lane]), (1, self.B[lane])):
                for i, o in enumerate(opts):
                    hit = set()
                    for leg, m in zip(lg, o[0]):
                        for (ol, oid) in conf.get((leg, id(m)), ()):
                            hit.update((ln, j) for ln, j in where.get((ol, oid, end), ()) if ln != lane)
                    self.oconf[(lane, end, i)] = hit
        self._ride = {}
        # ...and the RING's standoff round the destination's ball box, for a WOUND lane's ride (wound_ride): its berths'
        # exits stand outside the box by their stubs (the median, whole_frame.grown's measure), and the ring a lane pitch
        # past them (whole_frame: braid.ring_spine round the hull of the pads and the stubs). Priced at the box's own 0.3
        # mm, as around_box prices the short way, a lane wound round three faces came 5 to 12 mm longer laid (s4_bulgeW)
        bx_ = st['dboxes']
        out_ = sorted(v for lane, _lg in self.lanes for o in self.B[lane]
                      if (v := max(bx_[0] - o[1][0], o[1][0] - bx_[2], bx_[1] - o[1][1], o[1][1] - bx_[3])) > 0)
        self.ring_d0 = (out_[len(out_) // 2] if out_ else 0.0) + _bd.LPITCH
        # ---- the whole route's FEEDBACK (whole_feedback: ends its audits found crowded, FEEDBACK= to the fanout): an
        # end to avoid costs FB_AVOID vias when chosen, a pair of ends FB_PAIR when both are -- each option weighed by
        # its lane, its layer and ANY leg's exit's distance to the end's points as laid (fb_weights), so a pair cannot
        # leave the price by moving one leg and keeping the other where it was found crowded; two items naming one
        # option add
        self.fb_avoid, self.fb_pairs = collections.defaultdict(dict), []

        def matches(it):
            if it['lane'] not in self.legs_of:
                return {}
            return fb_weights(self.T[it['lane']] if it['end'] == 0 else self.B[it['lane']], it)
        fb = st.get('feedback') or {}
        for it in fb.get('avoid', ()):
            w_ = self.fb_avoid[(it['lane'], it['end'])]
            for i, w in matches(it).items():
                w_[i] = w_.get(i, 0.0) + w
        for a_, b_ in fb.get('pairs', ()):
            ma, mb = matches(a_), matches(b_)
            if ma and mb and a_['end'] == b_['end']:
                self.fb_pairs.append((a_['lane'], b_['lane'], a_['end'], ma, mb))
        # ...and the lanes the whole solve could not keep to two vias on an earlier round's ends ('over': whole_route,
        # an unproved plan's nets over two): each counted over by at least that much on every option, so the ends no
        # longer pay vias for a promise the solve could not keep (K51: ends at 92 vias promising SCK at two, chosen over
        # ends at 78 with SCK over; the solve held SCK over and 51 route vias, proved nothing, and laid nothing)
        self.fb_over = {ln: int(k) for ln, k in (fb.get('over') or {}).items() if ln in self.legs_of and int(k) > 0}
        self._xroute = {}          # the exact route per state (exact_route)
        self.wcut = None           # the destination's cut when WINDING moved it off the far face (choose: cut_phase)
        self.pool = {}             # every search's result: its state -> its objective by the estimate
        self._memo = {}            # every state scored: its objective -- the search asks a third of them again
        self._parts = collections.OrderedDict()     # the last _PARTS_KEEP states' parts (score), the oldest dropped
        self._perim = {}           # a point's place round a grown box: one lane's move rarely moves the box

    def ride(self, lane, npair, ti, bi, side=None):
        """a lane's ride in mm, each leg's: round the destination and the source from its tooth to its berth
        (select_moves.ride_mm's own measure), and its STUBS -- each leg's straight run from its ball to its tooth's
        and its berth's exit (a laid tooth has no legs of its own to measure; the straight run measures every option
        alike). Without them a tooth that ran 5 mm inside the source's balls to leave by another face cost nothing
        (K15 SDQS1, via-in-pad on B from the east rows out through the north face). `side`: a WOUND lane's ('N' or
        'S', _wound_sides) -- round the destination the long way, that side (wound_ride)"""
        k = (lane, ti, bi, side)
        if k not in self._ride:
            import plan_ends as pe
            a, b = self.T[lane][ti][1], self.B[lane][bi][1]
            d = pe.sm.around_box(a, b, self.st['dboxes']) if side is None else \
                wound_ride(a, b, self.st['dboxes'], side, pad=self.ring_d0)
            d += pe.sm.around_box(a, b, self.st['sgrid'].bbox) - math.hypot(b[0] - a[0], b[1] - a[1])
            lg = dict(self.lanes)[lane]
            stubs = 0.0
            for leg, tm, bm in zip(lg, self.T[lane][ti][0], self.B[lane][bi][0]):
                sp_, dp_ = self.st['src_pad'][leg], self.st['dst_pad'][leg]
                stubs += math.hypot(tm.exit_pt[0] - sp_.global_x, tm.exit_pt[1] - sp_.global_y)
                stubs += math.hypot(bm.exit_pt[0] - dp_.global_x, bm.exit_pt[1] - dp_.global_y)
            self._ride[k] = npair * d + stubs
        return self._ride[k]

    def _wound_sides(self, lanes, bo, DB, cut):
        """{lane: 'N' | 'S'} of the lanes WOUND in a state: its winding cut (cut_phase; `cut`) sends a lane's berth round
        another ring than the far face's own cut (the widest gap, whole_frame.cut) would -- read on the state's grown box
        DB as the frame reads it (whole_frame.ring_side). Read on the pad box with the short way's side, a berth on the
        box's south-west corner tied with the facing face, and s4_bulgeW's three lanes the frame laid wound (40 mm each)
        were priced the short way (16 to 18 mm). Empty with no winding cut"""
        if self.wcut is None:
            return {}
        import whole_frame
        far = whole_frame.cut([bo[l_][1][1] for l_ in lanes if whole_frame.face(bo[l_][1], DB) == 'E'], DB[1], DB[3])
        out = {}
        for l_ in lanes:
            s_w = whole_frame.ring_side(bo[l_][1], DB, cut)
            if s_w is not None and s_w != whole_frame.ring_side(bo[l_][1], DB, far):
                out[l_] = s_w
        return out

    def _wind_stack(self, lanes, to, bo, kd, dcls, npair, wound):
        """the ride each WOUND lane (`wound`, _wound_sides) adds by riding OUTSIDE its ring's other lanes (wound_ride
        prices it at the ring's own standoff): round each corner it turns, a quarter turn, half the ring's lanes inside
        it -- those its ring reaches before it, which peel off as it goes -- a lane pitch apart"""
        extra = 0.0
        for l_, side in wound.items():
            _d, nturn = wound_ride(to[l_][1], bo[l_][1], self.st['dboxes'], side, pad=self.ring_d0, turns=True)
            inside = sum(1 for o in lanes if o != l_ and dcls[o] == side
                         and (kd[o] > kd[l_] if side == 'N' else kd[o] < kd[l_]))
            extra += npair[l_] * nturn * (math.pi / 2) * (inside / 2) * _LANE_PITCH
        return extra

    # ---- the objective
    def score(self, state, exact=False):
        """(objective, parts) of a state {lane: (tooth option index, berth option index)}: each state's objective kept
        (objective), its parts for the last _PARTS_KEEP states asked -- an older state's computed again, the same (the
        search reads the parts of the states it keeps, not of the ones it tries: kept for every state, at ~4.6 KB a
        state at K41, a 200 000-state search held 0.9 GB of them)"""
        key = (tuple(state[l_] for l_, _lg in self.lanes), exact, self.wcut)
        p = self._parts.get(key)
        if p is not None:
            self._parts.move_to_end(key)
            return self._memo[key], p
        v, p = self._score(state, exact)
        self._memo[key] = v
        self._keep_parts(key, p)
        return v, p

    def objective(self, state, exact=False):
        """a state's objective alone (score's first): each state scored once (the local search asks a third of its
        states again -- an ejection's winner is scored in the min and then again, a sweep revisits the kicks' states)"""
        key = (tuple(state[l_] for l_, _lg in self.lanes), exact, self.wcut)
        v = self._memo.get(key)
        if v is None:
            v, p = self._score(state, exact)
            self._memo[key] = v
            self._keep_parts(key, p)
        return v

    def _keep_parts(self, key, p):
        self._parts[key] = p
        self._parts.move_to_end(key)
        if len(self._parts) > _PARTS_KEEP:
            self._parts.popitem(last=False)

    def _score(self, state, exact=False):
        """(objective, parts) of a state: at the far face's cut (the widest gap between its berths), or, where other
        gaps tie with it within a lane's pitch, at the tied cut the state scores best at (the widest on a tie) -- a
        gap a hair wider is no evidence: the synth wind_rot_e2 split its face one way on two layers and the other on
        three, 32 crossings against 36, 8 vias against 12"""
        import whole_frame
        if self.wcut is not None:
            return self._score_at(state, exact, self.wcut)      # (winding's cut, choose: cut_phase)
        pts = [self.B[l_][state[l_][1]][1] for l_, _lg in self.lanes]
        DB = whole_frame.grown(self.dbox, pts)
        cuts = whole_frame.cuts_tied([p[1] for p in pts if whole_frame.face(p, DB) == 'E'], DB[1], DB[3], _bd.LPITCH)
        if len(cuts) < 2:
            return self._score_at(state, exact)
        best = None
        for c in cuts:
            got = self._score_at(state, exact, c)
            if best is None or got[0] < best[0] - 1e-9:
                best = got
        return best

    def _score_at(self, state, exact=False, cut_at=None):
        """_score's at one cut (`cut_at`: a far-face y, None its widest gap -- or anywhere round the destination,
        whole_frame.cut_point's, WINDING the berths past it round the other side)"""
        import whole_frame
        T, B = self.T, self.B
        lanes = [l_ for l_, _lg in self.lanes]
        npair = {l_: len(lg) for l_, lg in self.lanes}
        to = {l_: T[l_][state[l_][0]] for l_ in lanes}
        bo = {l_: B[l_][state[l_][1]] for l_ in lanes}
        fan = sum(to[l_][3] + bo[l_][3] + sum(self.tie[g] for g in lg) for l_, lg in self.lanes)
        # the moves chosen, for the conflicts
        chosen = set()
        for l_, lg in self.lanes:
            for leg, m in zip(lg, to[l_][0]):
                chosen.add((leg, id(m)))
            for leg, m in zip(lg, bo[l_][0]):
                chosen.add((leg, id(m)))
        nconf = sum(1 for c in chosen for o in self.conf.get(c, ()) if o in chosen) // 2
        lane_of = {leg: l_ for l_, lg in self.lanes for leg in lg}
        nref = sum(1 for l_ in lanes if to[l_][4] or bo[l_][4])
        # the orders, as the whole frame reads them (whole_frame.build)
        SB = whole_frame.grown(self.sbox, [to[l_][1] for l_ in lanes])
        DB = whole_frame.grown(self.dbox, [bo[l_][1] for l_ in lanes])
        pc = self._perim

        def dface(p):
            k = ('face', p, DB)
            if k not in pc:
                pc[k] = whole_frame.face(p, DB)
            return pc[k]
        # (the far face's cut, from the model's own exits: it rides the plan sidecar to the solve's frame, dest_cut)
        cut = cut_at if cut_at is not None else \
            whole_frame.cut([bo[l_][1][1] for l_ in lanes if dface(bo[l_][1]) == 'E'], DB[1], DB[3])
        if isinstance(cut, (tuple, list)) and cut[0] == 'E':
            cut = float(cut[1])                 # (a far-face cut is its y, as the sidecar has always carried it)

        def psrc(p):
            k = (p, SB)
            if k not in pc:
                pc[k] = whole_frame.perim_s(p, SB)
            return pc[k]

        def pdst(p):
            k = (p, DB, cut)
            if k not in pc:
                pc[k] = whole_frame.perim_c(p, DB, cut)
            return pc[k]
        ps = {l_: psrc(to[l_][1]) for l_ in lanes}
        pd = {l_: pdst(bo[l_][1]) for l_ in lanes}
        tl = {l_: to[l_][2] for l_ in lanes}
        dl = {l_: bo[l_][2] for l_ in lanes}
        # strict orders: a tie broken by the lane's place in the run, the same way in both
        idx = {l_: i for i, l_ in enumerate(lanes)}
        kp = {l_: (ps[l_], idx[l_]) for l_ in lanes}
        kd = {l_: (pd[l_], idx[l_]) for l_ in lanes}
        inv = [(a, b) for a, b in itertools.combinations(lanes, 2) if (kp[a] < kp[b]) != (kd[a] < kd[b])]
        flat = {l_: tl[l_] == dl[l_] for l_ in lanes}
        same = [(a, b) for a, b in inv if flat[a] and flat[b] and tl[a] == tl[b]]
        # split pairs: another lane's end on the pair's layer between its two tips, at either end
        nsplit = 0
        if any(len(lg) > 1 for _l, lg in self.lanes):
            # every leg's place round each box, once
            legs_at = [(to, {l_: [psrc(m.exit_pt) for m in to[l_][0]] for l_ in lanes}, SB),
                       (bo, {l_: [pdst(m.exit_pt) for m in bo[l_][0]] for l_ in lanes}, DB)]
            for l_, lg in self.lanes:
                if len(lg) < 2:
                    continue
                for opt, pv, B_ in legs_at:
                    whole = 2 * (B_[2] - B_[0] + B_[3] - B_[1])
                    a_, b_ = sorted(pv[l_])
                    L = opt[l_][2]
                    for o in lanes:
                        if o == l_ or opt[o][2] != L:
                            continue
                        # (more routing layers than two: a via end's layer is the solve's, which holds an end between
                        # a pair's tips off the pair's layer -- whole_solve's tip_between -- a split only of two
                        # surface escapes)
                        if self.NL > 2 and (_via_end(opt[l_]) or _via_end(opt[o])):
                            continue
                        if any(whole_frame.between(a_, b_, v, whole) for v in pv[o]):
                            nsplit += 1
        parity = sum(npair[l_] for l_ in lanes if not flat[l_])
        # an OPPOSITE-HANDS pair (pairs.opposite_hands: P on one side of its travel at its tooth, on the other arriving
        # at its berth) crosses over at a dive: on one layer end to end, that is two changes -- and a lane already off
        # its layer and back takes its crossings there at no more
        xo = {}
        for l_, lg in self.lanes:
            xo[l_] = 0
            if len(lg) == 2 and flat[l_]:
                (tp, tn), (bp, bn) = to[l_][0], bo[l_][0]
                a_ = _pairs.hand(tp.direction, tp.exit_pt, tn.exit_pt)
                b_ = _pairs.hand(bp.direction, bp.exit_pt, bn.exit_pt, arriving=True)
                xo[l_] = 2 if a_ and b_ and a_ != b_ else 0
        same = [(a, b) for a, b in same if not xo[a] and not xo[b]]
        # a net's vias on the board: its stubs' own (a pair's leg the more) and its lane's changes -- one for ends on
        # two layers, two more for a lane leaving its layer to cross; how far that goes over two is priced first,
        # as the whole solve does. A TIE via (a ball's via to a pad of its own under it) is not counted toward the
        # two: it serves that pad, not the lane
        sv = {l_: max(t_.vias + b_.vias for t_, b_ in zip(to[l_][0], bo[l_][0])) for l_, lg in self.lanes}
        if self.NL > 2:
            return self._score_nl(state, exact, lanes, npair, to, bo, fan, nconf, nref, nsplit, lane_of, chosen, kp, kd,
                                  inv, tl, dl, sv, dface, cut, SB, DB)
        base = {l_: sv[l_] + (0 if flat[l_] else 1) + xo[l_] for l_ in lanes}
        ov = lambda l_, k_: max(0, base[l_] + k_ - VIA_PREF)
        w = {l_: 2 * npair[l_] + W_OVER * (ov(l_, 2) - ov(l_, 0)) for l_ in lanes}
        cov = self._cover([l_ for l_ in lanes if flat[l_] and not xo[l_]], tl, kp, kd, w)
        cover = sum(npair[l_] for l_ in cov)
        cross = sum(npair[l_] * xo[l_] for l_ in lanes)
        # each lane's changes so far: one for ends on two layers, two for a crossover pair, two for a lane of the cover
        chg = {l_: (0 if flat[l_] else 1) + xo[l_] + (2 if l_ in cov else 0) for l_ in lanes}
        # ...then each lane whose ends differ SETTLES the crossings its one change cannot serve. It crosses the lanes on
        # one layer end to end on the other layer: before its change those on its end layer, after it those on its
        # start layer. Two of them met in the wrong order break that -- when the order is forced: two lanes that do
        # not cross each other are met in the order the straight lines from launch rank to final rank meet them (two
        # that do, the solve orders). Broken, either the lane takes one excursion more (two changes) or the fewest of
        # those partners dive (an exact cut: the broken pairs are bipartite) -- fewer nets over two vias first, then
        # fewer vias, as the whole solve decides (K15: SDQ15 met four lanes on its end layer, then SDQS1 on its start
        # layer; the bound said 4 route vias, the solve paid 12)
        rl = {l_: i for i, l_ in enumerate(sorted(lanes, key=lambda l_: kp[l_]))}
        rf = {l_: i for i, l_ in enumerate(sorted(lanes, key=lambda l_: kd[l_]))}
        xs = collections.defaultdict(list)
        for a, b in inv:
            xs[a].append(b); xs[b].append(a)
        cset = {frozenset(e) for e in inv}
        tcr = lambda n, m: (rl[m] - rl[n]) / ((rl[m] - rl[n]) - (rf[m] - rf[n]))
        ovk = lambda l_, k_: max(0, sv[l_] + k_ - VIA_PREF)
        settle = 0
        for n in sorted((l_ for l_ in lanes if not flat[l_]), key=lambda l_: rl[l_]):
            ps = [(m, 'B.Cu' if tl[m] == 'F.Cu' else 'F.Cu') for m in sorted(xs[n], key=lambda m: tcr(n, m))
                  if flat[m] and chg[m] == 0]
            bad = [(a, b) for (a, na), (b, nb) in itertools.combinations(ps, 2)
                   if na == dl[n] and nb == tl[n] and frozenset((a, b)) not in cset]
            if not bad:
                continue
            wt = {m: W_OVER * (ovk(m, 2) - ovk(m, 0)) + 2 * npair[m] for e in bad for m in e}
            div = min_cut_cover(bad, wt)
            if W_OVER * (ovk(n, chg[n] + 2) - ovk(n, chg[n])) + 2 * npair[n] <= sum(wt[m] for m in div):
                chg[n] += 2
                settle += 2 * npair[n]
            else:
                for m in div:
                    chg[m] = 2
                    settle += 2 * npair[m]
        # ...then the lanes whose settling SHARES a choice. Two partners on one layer end to end that cross each other
        # (a, b; on different layers, neither diving) are met by a lane crossing both in the order the partners'
        # own crossing P decides: a lane that a meets before P meets first the partner it would meet with a and b
        # still uncrossed -- the nearer at launch, or a when they start either side of it -- and one met after P the
        # other. Two lanes that do not cross each other are met by a in a forced order, so the first cannot need P
        # after it while the second needs P before it (K15: SDQ11 needed SDQ15 first, SDQ9 met after it needed
        # SDQS1 first; each alone settled, together one of the four took an excursion). Each such conflict is
        # resolved by one of its four lanes -- an excursion (two changes) or a partner's dive -- the cheapest first,
        # fewer nets over two vias before fewer vias
        couple = 0
        conflicts = []
        # two lanes whose ends stand at one place round a box (an F tooth stacked over a B one) have no order
        # there: whether they cross is the route's to choose, so no order through them is forced
        tie = lambda x, y: abs(kp[x][0] - kp[y][0]) < 1e-3 or abs(kd[x][0] - kd[y][0]) < 1e-3
        for a, b in inv:
            if not (flat[a] and flat[b] and chg[a] == 0 and chg[b] == 0 and tl[a] != tl[b]) or tie(a, b):
                continue
            ns = [n for n in set(xs[a]) & set(xs[b]) if not flat[n] and chg[n] == 1
                  and not tie(n, a) and not tie(n, b)]
            if len(ns) < 2:
                continue
            want = {}
            for n in ns:
                first = a if tl[n] != tl[a] else b             # the partner asking n's start layer
                if (rl[a] - rl[n]) * (rl[b] - rl[n]) > 0:
                    pre = a if abs(rl[a] - rl[n]) < abs(rl[b] - rl[n]) else b
                else:
                    pre = a
                want[n] = first == pre                           # True: n needs P after it along a
            ns.sort(key=lambda n: tcr(a, n))
            for i, n1 in enumerate(ns):
                for n2 in ns[i + 1:]:
                    if frozenset((n1, n2)) not in cset and not tie(n1, n2) and not want[n1] and want[n2]:
                        conflicts.append((a, b, n1, n2))
        while conflicts:
            def fix_cost(x):
                return (W_OVER * (ovk(x, chg[x] + 2) - ovk(x, chg[x])) + 2 * npair[x]) if not flat[x] else \
                    (W_OVER * (ovk(x, 2) - ovk(x, 0)) + 2 * npair[x])
            cand = sorted({x for c in conflicts for x in c}, key=lambda x: idx[x])
            x = min(cand, key=lambda x: (fix_cost(x) / sum(1 for c in conflicts if x in c), idx[x]))
            chg[x] = chg[x] + 2 if not flat[x] else 2
            couple += 2 * npair[x]
            conflicts = [c for c in conflicts if x not in c]
        over = sum(ovk(l_, chg[l_]) for l_ in lanes)
        route = sum(npair[l_] * chg[l_] for l_ in lanes)
        route_est = route
        # ---- CONGESTION on the trunk (the whole frame's straight run between the two arrays, whole_frame): a crossing
        # with a lane off the lane's own ring happens there and takes a lane pitch along it, and a lane that berths on
        # the destination's near face changes layer there, a change taking its room; a lane's LOAD is what it needs
        # over the trunk's length. The solve can take a load to LOAD_OK freely (K28's ends: 0.5 at most, solved in 6
        # s); past it each lane pays W_CONG vias per the square of its overload (K35's: 0.8, 173 crossings on a 6.5
        # mm trunk, the solve found no plan)
        dcls = {}
        for l_ in lanes:
            f_ = dface(bo[l_][1])
            dcls[l_] = (('N' if bo[l_][1][1] < cut else 'S') if f_ == 'E' else f_) if whole_frame.far_cut(cut) \
                else (whole_frame.ring_side(bo[l_][1], DB, cut) or 'W')
        gT = max(DB[0] - SB[2], _LANE_PITCH)
        xT = collections.Counter()
        for a, b in inv:
            if not (dcls[a] == dcls[b] and dcls[a] != 'W'):
                xT[a] += 1; xT[b] += 1
        # ...and each crossing on the trunk its price, X_TRUNK vias, whatever the load: the solve's work grows with the
        # crossings it must place there, and ends with fewer of them are the easy ones (the human's K35 ends: 48 on
        # the trunk against our 94-173, for two vias more, by berths on the destination's far face)
        cong = X_TRUNK * sum(xT.values()) / 2
        loads = {}
        for l_ in lanes:
            load = (xT[l_] * _LANE_PITCH + (chg[l_] if dcls[l_] == 'W' else 0) * _CHG_ROOM) / gT
            loads[l_] = load
            over_ = max(0.0, load - LOAD_OK)
            cong += npair[l_] * W_CONG * over_ * over_
        exact_failed = False
        if exact:
            # the route EXACT on the orders (Ends.exact_route): what the estimate above approximates
            xr = self.exact_route(state, lanes, kp, kd, tl, dl, sv, xo, npair)
            if xr is not None:
                route, over, chg = xr
            else:
                exact_failed = True
        if self.fb_over:
            over = sum(max(ovk(l_, chg[l_]), self.fb_over.get(l_, 0)) for l_ in lanes)
        wound = self._wound_sides(lanes, bo, DB, cut)
        ride = sum(self.ride(l_, npair[l_], state[l_][0], state[l_][1], wound.get(l_)) for l_ in lanes) + \
            self._wind_stack(lanes, to, bo, kd, dcls, npair, wound)
        stacks = _stacks(self.lanes, to, bo, chg)
        fbk = (FB_AVOID * sum(ix.get(state[l_][k_], 0.0) for (l_, k_), ix in self.fb_avoid.items())
               + FB_PAIR * sum(ia.get(state[a_][k_], 0.0) * ib.get(state[b_][k_], 0.0)
                               for a_, b_, k_, ia, ib in self.fb_pairs))
        front = sum(c_.get(state[l_][k_], 0.0) for (l_, k_), c_ in self.front.items())
        # (each chosen end blocked with room for a via before the block, where an earlier round's audits found it
        # crowded: that block's span along its escape, for the solve to hold the lane on the other layer across it --
        # fanout_from_plan writes it, whole_solve reads it. An end nobody found crowded goes round the block on its
        # own layer, as a lane goes round a part: a change planned there cost two vias where none were needed)
        lane_front = collections.defaultdict(dict)
        for (l_, k_), c_ in self.front.items():
            if (l_, k_) not in self.fb_avoid:
                continue
            i_ = state[l_][k_]
            if c_.get(i_) == FRONT_VIA:
                o_ = (self.T if k_ == 0 else self.B)[l_][i_]
                sp_ = front_span(self.st['pcb'], self.st['byname'][self.legs_of[l_][0]][0], o_[0][0], self.run_ids)
                if sp_ is not None:
                    lane_front[l_][k_] = sp_
        walled = walled_balls(self.pw, self.lanes, state)
        obj = (W_OVER * over + fan + route + ride / VIA_MM + cong + fbk + EPS_X * len(inv) + BIG * (nconf + nsplit + nref)
               + STACK_COST * stacks + front + WALLED * len(walled))
        # (each lane's share, for a round the whole route laid nothing on or left nets open -- whole_feedback --name
        # names the lanes to free from these: its nets over two, its load on the trunk, its crossings -- and the lanes
        # still in a conflict, which a destination re-plan frees with their neighbours)
        xl = collections.Counter(l_ for e in inv for l_ in e)
        lane_over = {l_: k_ for l_ in lanes if (k_ := max(ovk(l_, chg[l_]), self.fb_over.get(l_, 0)))}
        return obj, dict(fan=fan, parity=parity, crossover=cross, cover=cover, settle=settle, couple=couple,
                         route=route, route_est=route_est, over=over, cong=round(cong, 2), feedback=fbk,
                         ride=round(ride, 1), chg=chg,
                         crossings=len(inv), same=len(same), conflicts=nconf, splits=nsplit, refused=nref,
                         lane_over=lane_over, lane_load={l_: round(v_, 3) for l_, v_ in loads.items()},
                         lane_x=dict(xl), stacks=stacks, exact_failed=exact_failed, cut=cut, front=front,
                         lane_front=dict(lane_front), walled=walled,
                         conf_lanes=(sorted({lane_of[c[0]] for c in chosen for o in self.conf.get(c, ())
                                             if o in chosen}) if nconf else []))

    # ---- more routing layers than two
    def end_domain(self, opt, k_, lg):
        """(fixed, neck, allowed) of an end option `opt` (k_ 0 a tooth, 1 a berth; `lg` its lane's legs), as routing-
        layer indices: a SURFACE escape is FIXED on its layer (neck and allowed None); a VIA end -- every leg a dog-bone
        or a via in its pad -- is free (fixed None) on the layers its runs can lie on (`allowed`, via_run_layers), its
        via joining nothing, and dropped, on its NECK's own layer (the ball's pad's; None where a leg's pad is not on one
        routing layer)"""
        moves = opt[0]
        if not moves or any(getattr(m, 'kind', 'surface') == 'surface' for m in moves):
            return (self.RL.index(opt[2]) if opt[2] in self.RL else 0), None, None
        necks = set()
        for leg in lg:
            pad = self.st['src_pad' if k_ == 0 else 'dst_pad'].get(leg)
            cu = {L for L in (getattr(pad, 'layers', None) or ()) if L in self.RL}
            necks.add(next(iter(cu)) if len(cu) == 1 else None)
        neck = next(iter(necks)) if len(necks) == 1 else None
        return None, (self.RL.index(neck) if neck is not None else None), self.via_run_layers(moves, lg)

    def via_run_layers(self, moves, lg):
        """the routing layers (indices) a via end's runs can lie on: its own layer, and each other one where every
        leg's run out of its via -- the move's legs on its own layer -- clears that layer's copper (braid.build_obstacles,
        the run's nets free: they move with the search), as whole_solve's VBAN reads the laid runs"""
        key = tuple(id(m) for m in moves)
        got = self._runs_ok.get(key)
        if got is not None:
            return got[1]
        ok = set()
        for q, L in enumerate(self.RL):
            fine = True
            for leg, m in zip(lg, moves):
                if L == m.layer:
                    continue
                nid = self.st['byname'][leg][0]
                o = _bd.build_obstacles(self.st['pcb'], nid, self.run_ids, L)
                if any(not o.seg_clear(a, b) for a, b, L_ in (getattr(m, 'legs', None) or ()) if L_ == m.layer):
                    fine = False
                    break
            if fine:
                ok.add(q)
        got = frozenset(ok)
        self._runs_ok[key] = (tuple(moves), got)
        return got

    def end_clashes(self, ends, to, bo):
        """(ends, sep): the chosen ends' runs against EACH OTHER, as the whole solve reads the laid board (relayer.clashes:
        its VBAN and VSEP) -- via_run_layers tests a via end's runs against the copper on the board, where the other
        lanes' chosen ends are not yet. A via end loses each layer, not its own, where another lane's FIXED end there
        comes within the rule of its run; two via ends whose runs come within the rule of each other, laid on two
        layers, end on different ones (sep [((lane, k), (lane, k))]). The synth ring_s4 on three layers: a berth on B.Cu
        ending at the point its neighbour's F.Cu berth does, free to drop onto F alone"""
        opt = lambda l_, k_: (to if k_ == 0 else bo)[l_]
        own = lambda l_, k_: self.RL.index(opt(l_, k_)[2]) if opt(l_, k_)[2] in self.RL else None
        vend = [(l_, k_) for l_, _lg in self.lanes for k_ in (0, 1) if ends[l_][k_][0] is None]
        out, sep = dict(ends), []
        for l_, k_ in vend:
            f_, n_, ok_ = out[l_][k_]
            lose = set()
            for l2, _lg2 in self.lanes:
                q = ends[l2][k_][0] if l2 != l_ else None
                if q is None or q not in ok_ or q in lose or q == own(l_, k_):
                    continue
                if self._runs_meet(opt(l_, k_)[0], opt(l2, k_)[0], self.RL[q]):
                    lose.add(q)
            if lose:
                e_ = list(out[l_])
                e_[k_] = (f_, n_, ok_ - lose)
                out[l_] = tuple(e_)
        for i, (la, ka) in enumerate(vend):
            for lb, kb in vend[i + 1:]:
                if la != lb and ka == kb and own(la, ka) != own(lb, kb) \
                        and self._runs_meet(opt(la, ka)[0], opt(lb, kb)[0], None):
                    sep.append(((la, ka), (lb, kb)))
        return out, sep

    def _runs_meet(self, ma, mb, layer):
        """whether via end `ma`'s runs (each move's legs on its own layer) come within the fanout's rule (its track,
        its clearance) of `mb`'s -- a fixed end's legs on `layer`, or (None) a via end's runs"""
        key = (tuple(id(m) for m in ma), tuple(id(m) for m in mb), layer)
        got = self._meet.get(key)
        if got is not None:
            return got[2]
        import rules as _rules
        r = _rules.active()
        bar = r.fan_track + r.fan_clear - 0.01          # (relayer.clashes' SLACK: the fanout's grid a hair under)
        ra = [(a, b) for m in ma for a, b, L_ in (getattr(m, 'legs', None) or ()) if L_ == m.layer]
        rb = [(a, b) for m in mb for a, b, L_ in (getattr(m, 'legs', None) or ())
              if L_ == (layer if layer is not None else m.layer)]
        hit = any(_seg_d(a, b, p, q) < bar for a, b in ra for p, q in rb)
        self._meet[key] = (tuple(ma), tuple(mb), hit)
        return hit

    def _colour(self, lanes, inv, ends, npair):
        """(changes, drops) per lane: the crossing graph COLOURED on the routing layers, the ESTIMATE the search steers by
        -- each lane, most crossed first, on the layer its crossed lanes left it that costs it least: a change for each
        fixed end not on it, less a via dropped for each via end on its neck there; a lane its neighbours left no layer
        crosses on the way, leaving its cheapest and coming back (two changes more). Two lanes that cross on different
        layers need nothing; a via end's lane takes any layer (the human's way: a dog-bone each end, one layer a net)"""
        nb = collections.defaultdict(set)
        for a, b in inv:
            nb[a].add(b); nb[b].add(a)
        col, chg, drp = {}, {}, {}

        def price(l_, c):
            # (a fixed end not on c a change; a via end whose runs cannot lie on c a change too, near it; a via end on
            # its neck's layer, its runs allowed there, a via dropped)
            ch = dr = 0
            for f_, n_, ok_ in ends[l_]:
                if f_ is not None:
                    ch += c != f_
                else:
                    ch += c not in ok_
                    dr += c == n_ and c in ok_
            return ch, dr
        for l_ in sorted(lanes, key=lambda l_: (-len(nb[l_]), lanes.index(l_))):
            used = {col[o] for o in nb[l_] if o in col}
            free = [c for c in range(self.NL) if c not in used]
            pick = min(free or range(self.NL), key=lambda c: (price(l_, c)[0] - price(l_, c)[1], c))
            ch, dr = price(l_, pick)
            col[l_] = pick if free else None
            chg[l_] = ch + (0 if free else 2)
            drp[l_] = dr
        return chg, drp, col

    def _score_nl(self, state, exact, lanes, npair, to, bo, fan, nconf, nref, nsplit, lane_of, chosen, kp, kd, inv,
                  tl, dl, sv, dface, cut, SB, DB):
        """the objective of a state on MORE ROUTING LAYERS THAN TWO (_score's, with its N-layer route): each end's
        layer domain (end_domain), the route estimated by a colouring of the crossing graph (_colour) or, exact, by
        the whole solve's own model (exact_route_nl); an OPPOSITE-HANDS pair (pairs.opposite_hands) a change at least, at
        its dive; the dropped vias off the fanout's count; the trunk's load shared among the layers. Two lanes that cross
        cost nothing where they can stand on two layers -- the two-layer arithmetic (parity, a single chain of lanes on
        one layer, settling) charged a via end its via and the crossings both, and steered to surface escapes"""
        import whole_frame
        ends = {}
        for l_, lg in self.lanes:
            ends[l_] = (self.end_domain(to[l_], 0, lg), self.end_domain(bo[l_], 1, lg))
        ends, sep = self.end_clashes(ends, to, bo)
        chg, drp, col = self._colour(lanes, inv, ends, npair)
        xo = {}
        for l_, lg in self.lanes:
            xo[l_] = 0
            if len(lg) == 2:
                (tp, tn), (bp, bn) = to[l_][0], bo[l_][0]
                a_ = _pairs.hand(tp.direction, tp.exit_pt, tn.exit_pt)
                b_ = _pairs.hand(bp.direction, bp.exit_pt, bn.exit_pt, arriving=True)
                if a_ and b_ and a_ != b_ and chg[l_] == 0:
                    (tf, _tn, _ta), (df, _dn, _da) = ends[l_]
                    xo[l_] = 2 if (tf is not None and df is not None) else 1     # (a free end takes the second)
                    chg[l_] += xo[l_]
        fan -= sum(npair[l_] * drp[l_] for l_ in lanes)
        svd = {l_: max(0, sv[l_] - drp[l_]) for l_ in lanes}
        ovk = lambda l_, k_: max(0, svd[l_] + k_ - VIA_PREF)
        route = sum(npair[l_] * chg[l_] for l_ in lanes)
        route_est = route
        over = sum(ovk(l_, chg[l_]) for l_ in lanes)
        # (the trunk: as on two layers, each crossing there its price, and each lane's load -- shared among the layers,
        # two lanes crossing on different ones standing one over the other)
        dcls = {}
        for l_ in lanes:
            f_ = dface(bo[l_][1])
            dcls[l_] = (('N' if bo[l_][1][1] < cut else 'S') if f_ == 'E' else f_) if whole_frame.far_cut(cut) \
                else (whole_frame.ring_side(bo[l_][1], DB, cut) or 'W')
        gT = max(DB[0] - SB[2], _LANE_PITCH)
        xT = collections.Counter()
        for a, b in inv:
            if not (dcls[a] == dcls[b] and dcls[a] != 'W'):
                xT[a] += 1; xT[b] += 1
        cong = X_TRUNK * sum(xT.values()) / 2
        loads = {}
        for l_ in lanes:
            load = (xT[l_] * _LANE_PITCH / (self.NL - 1) + (chg[l_] if dcls[l_] == 'W' else 0) * _CHG_ROOM) / gT
            loads[l_] = load
            over_ = max(0.0, load - LOAD_OK)
            cong += npair[l_] * W_CONG * over_ * over_
        exact_failed = False
        if exact:
            xr = self.exact_route_nl(state, lanes, kp, kd, ends, svd, xo, npair, sep)
            if xr is not None:
                route, over, chg, drx = xr
                fan += sum(npair[l_] * (drp[l_] - drx[l_]) for l_ in lanes)     # (its own drops, not the estimate's)
            else:
                exact_failed = True
        if self.fb_over:
            over = sum(max(ovk(l_, chg[l_]), self.fb_over.get(l_, 0)) for l_ in lanes)
        wound = self._wound_sides(lanes, bo, DB, cut)
        ride = sum(self.ride(l_, npair[l_], state[l_][0], state[l_][1], wound.get(l_)) for l_ in lanes) + \
            self._wind_stack(lanes, to, bo, kd, dcls, npair, wound)
        stacks = _stacks(self.lanes, to, bo, chg)
        fbk = (FB_AVOID * sum(ix.get(state[l_][k_], 0.0) for (l_, k_), ix in self.fb_avoid.items())
               + FB_PAIR * sum(ia.get(state[a_][k_], 0.0) * ib.get(state[b_][k_], 0.0)
                               for a_, b_, k_, ia, ib in self.fb_pairs))
        front = sum(c_.get(state[l_][k_], 0.0) for (l_, k_), c_ in self.front.items())
        lane_front = collections.defaultdict(dict)
        for (l_, k_), c_ in self.front.items():
            if (l_, k_) not in self.fb_avoid:
                continue
            i_ = state[l_][k_]
            if c_.get(i_) == FRONT_VIA:
                o_ = (self.T if k_ == 0 else self.B)[l_][i_]
                sp_ = front_span(self.st['pcb'], self.st['byname'][self.legs_of[l_][0]][0], o_[0][0], self.run_ids)
                if sp_ is not None:
                    lane_front[l_][k_] = sp_
        obj = (W_OVER * over + fan + route + ride / VIA_MM + cong + fbk + EPS_X * len(inv) + BIG * (nconf + nsplit + nref)
               + STACK_COST * stacks + front + WALLED * len(walled := walled_balls(self.pw, self.lanes, state)))
        xl = collections.Counter(l_ for e in inv for l_ in e)
        lane_over = {l_: k_ for l_ in lanes if (k_ := max(ovk(l_, chg[l_]), self.fb_over.get(l_, 0)))}
        same = sum(1 for a, b in inv if col.get(a) is not None and col.get(a) == col.get(b))
        return obj, dict(fan=fan, parity=sum(npair[l_] for l_ in lanes if ends[l_][0][0] is not None
                                             and ends[l_][1][0] is not None and ends[l_][0][0] != ends[l_][1][0]),
                         crossover=sum(npair[l_] * xo[l_] for l_ in lanes), cover=0, settle=0, couple=0,
                         route=route, route_est=route_est, over=over, cong=round(cong, 2), feedback=fbk,
                         ride=round(ride, 1), chg=chg,
                         crossings=len(inv), same=same, conflicts=nconf, splits=nsplit, refused=nref,
                         lane_over=lane_over, lane_load={l_: round(v_, 3) for l_, v_ in loads.items()},
                         lane_x=dict(xl), stacks=stacks, exact_failed=exact_failed, cut=cut, front=front,
                         lane_front=dict(lane_front), dropped=sum(npair[l_] * drp[l_] for l_ in lanes), walled=walled,
                         conf_lanes=(sorted({lane_of[c[0]] for c in chosen for o in self.conf.get(c, ())
                                             if o in chosen}) if nconf else []))

    def exact_route_nl(self, state, lanes, kp, kd, ends, sv, xo, npair, sep=()):
        """(route vias, nets over two, changes per lane, vias dropped per lane) of a state EXACT on its orders on MORE
        ROUTING LAYERS THAN TWO -- the whole solve's N-layer model without its lengths: every inverted pair crossing once,
        the braid triple rule, each lane's runs between its changes on one layer each (y), two crossing lanes on
        different layers there, a fixed end's run on its layer and a via end's on any (its via dropped on its neck's),
        an opposite-hands pair a change at least, two via ends whose runs meet (`sep`, end_clashes) on two layers --
        CP-SAT on the solve's objective (nets over two first, then vias).
        One worker, a deterministic work limit; None when it finds no plan in that work"""
        key = ('nl', self.wcut) + tuple(sorted(state.items()))
        if key in self._xroute:
            return self._xroute[key]
        from ortools.sat.python import cp_model
        idx = {l_: i for i, l_ in enumerate(lanes)}
        rkd = {l_: i for i, l_ in enumerate(sorted(lanes, key=lambda l_: kd[l_]))}
        rkp = {l_: i for i, l_ in enumerate(sorted(lanes, key=lambda l_: kp[l_]))}
        Ln = sorted(lanes, key=lambda l_: (round(kp[l_][0], 3), rkd[l_], idx[l_]))
        Fn = sorted(lanes, key=lambda l_: (round(kd[l_][0], 3), rkp[l_], idx[l_]))
        li = {l_: i for i, l_ in enumerate(Ln)}
        fi = {l_: i for i, l_ in enumerate(Fn)}
        X = [(a, b) for a, b in itertools.combinations(Ln, 2) if (li[a] < li[b]) == (fi[a] > fi[b])]
        m = cp_model.CpModel()
        H = 2 * len(X) + 2
        t = {k: m.NewIntVar(1, H, '') for k in X}
        for i, j, k in itertools.combinations(Ln, 3):
            ij, ik, jk = (i, j) in t, (i, k) in t, (j, k) in t
            if ij and ik and jk:
                b_ = m.NewBoolVar('')
                m.Add(t[(i, j)] < t[(i, k)]).OnlyEnforceIf(b_); m.Add(t[(i, k)] < t[(j, k)]).OnlyEnforceIf(b_)
                m.Add(t[(j, k)] < t[(i, k)]).OnlyEnforceIf(b_.Not()); m.Add(t[(i, k)] < t[(i, j)]).OnlyEnforceIf(b_.Not())
            elif ij and ik:
                m.Add(t[(i, j)] < t[(i, k)])
            elif ik and jk:
                m.Add(t[(j, k)] < t[(i, k)])
        ev = {l_: [k for k in X if l_ in k] for l_ in lanes}
        for l_ in lanes:
            if len(ev[l_]) > 1:
                m.AddAllDifferent([t[k] for k in ev[l_]])
        before, ys, acts, drops, cost = {}, {}, {}, {}, []
        for l_ in lanes:
            cs = [m.NewIntVar(0, H + 1, '') for _ in range(EXACT_KMAX)]
            act = [m.NewBoolVar('') for _ in range(EXACT_KMAX)]
            for k in range(EXACT_KMAX):
                m.Add(cs[k] <= H).OnlyEnforceIf(act[k]); m.Add(cs[k] == H + 1).OnlyEnforceIf(act[k].Not())
                if k:
                    m.Add(cs[k] > cs[k - 1]).OnlyEnforceIf(act[k]); m.AddImplication(act[k], act[k - 1])
            for key_ in ev[l_]:
                bits = []
                for k in range(EXACT_KMAX):
                    bb = m.NewBoolVar('')
                    m.Add(cs[k] < t[key_]).OnlyEnforceIf(bb); m.Add(cs[k] > t[key_]).OnlyEnforceIf([bb.Not(), act[k]])
                    m.AddImplication(bb, act[k]); bits.append(bb)
                before[(l_, key_)] = bits
            y = [m.NewIntVar(0, self.NL - 1, '') for _ in range(EXACT_KMAX + 1)]
            dr_ = []
            for y_e, (fx, nk, ok_) in ((y[0], ends[l_][0]), (y[EXACT_KMAX], ends[l_][1])):
                if fx is not None:
                    m.Add(y_e == fx)
                    continue
                for q in range(self.NL):
                    if q not in ok_:
                        m.Add(y_e != q)          # (a layer its runs cannot lie on)
                if nk is not None and nk in ok_:
                    d_ = m.NewBoolVar('')
                    m.Add(y_e == nk).OnlyEnforceIf(d_); m.Add(y_e != nk).OnlyEnforceIf(d_.Not())
                    dr_.append(d_)
            for k in range(EXACT_KMAX):
                m.Add(y[k + 1] != y[k]).OnlyEnforceIf(act[k]); m.Add(y[k + 1] == y[k]).OnlyEnforceIf(act[k].Not())
            if xo.get(l_):
                m.Add(act[0] == 1)              # an opposite-hands pair crosses over at a dive
            ys[l_], acts[l_], drops[l_] = y, act, dr_
            n_ = sum(act)
            ov = m.NewIntVar(0, EXACT_KMAX + 8, '')
            m.Add(ov >= sv[l_] + n_ - sum(dr_) - VIA_PREF)
            cost.append(int(W_OVER) * ov + npair[l_] * (n_ - sum(dr_)))
        for (la_, ka_), (lb_, kb_) in sep:
            if la_ in ys and lb_ in ys:
                m.Add(ys[la_][0 if ka_ == 0 else EXACT_KMAX] != ys[lb_][0 if kb_ == 0 else EXACT_KMAX])
        for key_ in X:
            a, b = key_
            la = []
            for l_ in (a, b):
                i_ = m.NewIntVar(0, EXACT_KMAX, '')
                m.Add(i_ == sum(before[(l_, key_)]))
                L_ = m.NewIntVar(0, self.NL - 1, '')
                m.AddElement(i_, ys[l_], L_)
                la.append(L_)
            m.Add(la[0] != la[1])
        m.Minimize(sum(cost))
        sol = cp_model.CpSolver()
        sol.parameters.num_workers = 1
        sol.parameters.max_deterministic_time = EXACT_WORK
        r = sol.Solve(m)
        out = None
        if r in (cp_model.OPTIMAL, cp_model.FEASIBLE):
            chg = {l_: sum(sol.Value(a) for a in acts[l_]) for l_ in lanes}
            drx = {l_: sum(sol.Value(d_) for d_ in drops[l_]) for l_ in lanes}
            out = (sum(npair[l_] * chg[l_] for l_ in lanes),
                   sum(max(0, sv[l_] + chg[l_] - drx[l_] - VIA_PREF) for l_ in lanes), chg, drx)
        self._xroute[key] = out
        return out

    def exact_route(self, state, lanes, kp, kd, tl, dl, sv, xo, npair):
        """(route vias, nets over two, changes per lane) of a state EXACT on its orders: the whole solve's order model
        without its lengths -- every inverted pair crossing once at a position, the braid triple rule, each lane's
        changes between its crossings, two crossing lanes on different layers there, an opposite-hands pair crossed
        over at a dive -- CP-SAT on the solve's own objective (nets over two vias first, then vias, a pair's change two):
        its optimum, or the best plan it finds within its work limit. Two lanes at one place round a box (an F end stacked over a B one) are taken in the order
        that does not cross them there. One worker and a deterministic work limit: the same answer on every machine;
        None when it finds no plan in that work (the estimate stands)"""
        key = (self.wcut,) + tuple(sorted(state.items()))
        if key in self._xroute:
            return self._xroute[key]
        from ortools.sat.python import cp_model
        idx = {l_: i for i, l_ in enumerate(lanes)}
        rkd = {l_: i for i, l_ in enumerate(sorted(lanes, key=lambda l_: kd[l_]))}
        rkp = {l_: i for i, l_ in enumerate(sorted(lanes, key=lambda l_: kp[l_]))}
        Ln = sorted(lanes, key=lambda l_: (round(kp[l_][0], 3), rkd[l_], idx[l_]))
        Fn = sorted(lanes, key=lambda l_: (round(kd[l_][0], 3), rkp[l_], idx[l_]))
        li = {l_: i for i, l_ in enumerate(Ln)}
        fi = {l_: i for i, l_ in enumerate(Fn)}
        X = [(a, b) for a, b in itertools.combinations(Ln, 2) if (li[a] < li[b]) == (fi[a] > fi[b])]
        m = cp_model.CpModel()
        H = 2 * len(X) + 2
        t = {k: m.NewIntVar(1, H, '') for k in X}
        for i, j, k in itertools.combinations(Ln, 3):
            ij, ik, jk = (i, j) in t, (i, k) in t, (j, k) in t
            if ij and ik and jk:
                b_ = m.NewBoolVar('')
                m.Add(t[(i, j)] < t[(i, k)]).OnlyEnforceIf(b_); m.Add(t[(i, k)] < t[(j, k)]).OnlyEnforceIf(b_)
                m.Add(t[(j, k)] < t[(i, k)]).OnlyEnforceIf(b_.Not()); m.Add(t[(i, k)] < t[(i, j)]).OnlyEnforceIf(b_.Not())
            elif ij and ik:
                m.Add(t[(i, j)] < t[(i, k)])
            elif ik and jk:
                m.Add(t[(j, k)] < t[(i, k)])
        ev = {l_: [k for k in X if l_ in k] for l_ in lanes}
        for l_ in lanes:
            if len(ev[l_]) > 1:
                m.AddAllDifferent([t[k] for k in ev[l_]])
        F_ = lambda L: int(L == 'B.Cu')
        before, act_of, cost = {}, {}, []
        for l_ in lanes:
            cs = [m.NewIntVar(0, H + 1, '') for _ in range(EXACT_KMAX)]
            act = [m.NewBoolVar('') for _ in range(EXACT_KMAX)]
            for k in range(EXACT_KMAX):
                m.Add(cs[k] <= H).OnlyEnforceIf(act[k]); m.Add(cs[k] == H + 1).OnlyEnforceIf(act[k].Not())
                if k:
                    m.Add(cs[k] > cs[k - 1]).OnlyEnforceIf(act[k]); m.AddImplication(act[k], act[k - 1])
            for key_ in ev[l_]:
                bits = []
                for k in range(EXACT_KMAX):
                    bb = m.NewBoolVar('')
                    m.Add(cs[k] < t[key_]).OnlyEnforceIf(bb); m.Add(cs[k] > t[key_]).OnlyEnforceIf([bb.Not(), act[k]])
                    m.AddImplication(bb, act[k]); bits.append(bb)
                before[(l_, key_)] = bits
            m.AddBoolXOr(act + ([m.NewConstant(1)] if tl[l_] == dl[l_] else []))
            if xo.get(l_):
                m.Add(act[0] == 1)              # an opposite-hands pair crosses over at a dive
            n_ = sum(act)
            ov = m.NewIntVar(0, EXACT_KMAX + 8, '')
            m.Add(ov >= sv[l_] + n_ - VIA_PREF)
            act_of[l_] = act
            cost.append(int(W_OVER) * ov + npair[l_] * n_)
        for key_ in X:
            a, b = key_
            lits = before[(a, key_)] + before[(b, key_)]
            if F_(tl[a]) ^ F_(tl[b]):
                lits = lits + [m.NewConstant(1)]
            m.AddBoolXOr(lits)
        m.Minimize(sum(cost))
        sol = cp_model.CpSolver()
        sol.parameters.num_workers = 1
        sol.parameters.max_deterministic_time = EXACT_WORK
        r = sol.Solve(m)
        out = None
        if r in (cp_model.OPTIMAL, cp_model.FEASIBLE):
            chg = {l_: sum(sol.Value(a) for a in act_of[l_]) for l_ in lanes}
            out = (sum(npair[l_] * chg[l_] for l_ in lanes),
                   sum(max(0, sv[l_] + chg[l_] - VIA_PREF) for l_ in lanes), chg)
        self._xroute[key] = out
        return out

    def best_exact(self, state, log=print):
        """(state, (objective, parts)): of `state` and the searches' best distinct ends so far (the pool, within
        EXACT_MARGIN of the best by the estimate, EXACT_TOP of them), the best by the EXACT route. The estimate steers
        the search; the orders it cannot see (a crossing pair met by two lanes in the orders each needs, a cover's
        excursion that cannot hold every crossing it must) decide among its best"""
        ranked = sorted(self.pool.items(), key=lambda kv: (kv[1], kv[0]))
        v0 = ranked[0][1] if ranked else None
        cands = [dict(k) for k, v in ranked[:EXACT_TOP] if v <= v0 + EXACT_MARGIN]
        if dict(state) not in cands:
            cands.append(dict(state))
        best = None
        for s_ in cands:
            v_, p_ = self.score(s_, exact=True)
            # (a state the exact route found no plan for ranks after every one it did: its objective is the estimate's,
            # the optimistic one, for the ends the solve will find hardest)
            k_ = (bool(p_.get('exact_failed')), v_)
            if best is None or k_ < (bool(best[1][1].get('exact_failed')), best[1][0] - 1e-9):
                best = (s_, (v_, p_))
        if best[0] != dict(state):
            log(f'  whole ends: by the exact route, another of the search\'s best: {_fmt(best[1][1])}')
        return best

    @staticmethod
    def _cover(flat_lanes, layer, kp, kd, w):
        """the lanes of least weight `w` covering every crossing between two of `flat_lanes` on one layer: EXACT. On a
        layer those crossings are the inversions between the two orders -- a permutation graph -- so what needs no
        cover is the heaviest chain rising in both, and the cover is the rest (an O(n^2) chain, per layer)"""
        out = set()
        for L in sorted({layer[l_] for l_ in flat_lanes}):
            ls = sorted((l_ for l_ in flat_lanes if layer[l_] == L), key=lambda l_: kp[l_])
            best, prev = {}, {}
            for i, a in enumerate(ls):
                best[a], prev[a] = w[a], None
                for b in ls[:i]:
                    if kd[b] < kd[a] and best[b] + w[a] > best[a]:
                        best[a], prev[a] = best[b] + w[a], b
            if not ls:
                continue
            chain, x = set(), max(ls, key=lambda l_: (best[l_], -kp[l_][1]))
            while x is not None:
                chain.add(x); x = prev[x]
            out |= set(ls) - chain
        return out

    # ---- the search
    def start(self, seed=None):
        """the teeth as laid (their own option, else the cheapest clear one), the berths at `seed` {net: Move}
        (default: the fanout's greedy selection, plan_ends.sm.select -- every berth clear of the others), a pair at
        the combination nearest its legs' there; a lane the seed leaves out at its cheapest berth clear of those
        taken"""
        import plan_ends as pe
        import source_realize as sr
        st = self.st
        if seed is None:
            pads = {nm: (st['dst_pad'][nm].global_x, st['dst_pad'][nm].global_y) for nm in st['dst_pad']}
            seed, _un = pe.sm.select(st['dmenu'], st['launch'], keep_out=st['dboxes'], buses=st['buses'],
                                     tooth_layer=st['tooth0'], log=None, pads=pads, chi=st['chi'])
        greedy = seed
        state = {l_: (None, None) for l_, _lg in self.lanes}
        for lane, lg in self.lanes:
            ti = next((i for i, o in enumerate(self.T[lane]) if all(m is self.cur.get(l_) for l_, m in zip(lg, o[0]))),
                      None)
            want = [greedy.get(l_) for l_ in lg]
            if all(w is not None for w in want):
                bi = min(range(len(self.B[lane])), default=None,
                         key=lambda i: sum(math.hypot(m.exit_pt[0] - w.exit_pt[0], m.exit_pt[1] - w.exit_pt[1])
                                           + (0 if sr.move_sig(m) == sr.move_sig(w) else 1e-3)
                                           for m, w in zip(self.B[lane][i][0], want)))
            else:
                bi = None
            state[lane] = (ti, bi)
        for lane, lg in self.lanes:
            for end in (0, 1):
                if state[lane][end] is None:
                    opts = self.T[lane] if end == 0 else self.B[lane]
                    free = [i for i in range(len(opts)) if self._clear(state, lane, end, i)] or list(range(len(opts)))
                    i = min(free, key=lambda i: opts[i][3]) if free else None
                    state[lane] = (i, state[lane][1]) if end == 0 else (state[lane][0], i)
        return state

    def _hits(self, state, lane, end, i):
        """the other lanes whose chosen moves (at the same end) a lane's option i conflicts with"""
        return {o for o, j in self.oconf[(lane, end, i)] if state[o][end] == j}

    def _clear(self, state, lane, end, i):
        return not self._hits(state, lane, end, i)

    def search(self, state=None, sweeps=20, log=print):
        """each improving change of one lane's tooth or berth (one other lane ejected where it is in the way), taken as
        it is found, sweep after sweep until a sweep finds none"""
        state = dict(state or self.start())
        missing = [l_ for l_ in state if state[l_][0] is None or state[l_][1] is None]
        if missing:
            raise SystemExit(f'whole_ends: no tooth or berth option for {missing}')
        best, parts = self.score(state)
        tabu = getattr(self, 'tabu', None) or set()     # (lane, end, option) a ban kick forbids (ban_kicks)
        set_ = lambda s_, l_, e_, i_: {**s_, l_: ((s_[l_][0], i_) if e_ else (i_, s_[l_][1]))}
        for sw in range(sweeps):
            improved = False
            for lane, _lg in self.lanes:
                for end in (1, 0):
                    opts = self.B[lane] if end else self.T[lane]
                    for i in range(len(opts)):
                        if i == state[lane][end] or (lane, end, i) in tabu:
                            continue
                        hits = self._hits(state, lane, end, i)
                        if len(hits) > 1:
                            continue
                        trial = set_(state, lane, end, i)
                        if hits:
                            # an EJECTION: the one lane in the way moves to its best option clear of the others
                            (o,) = hits
                            oo = self.B[o] if end else self.T[o]
                            cands = [j for j in range(len(oo)) if self._clear(trial, o, end, j)
                                     and (o, end, j) not in tabu]
                            if not cands:
                                continue
                            trial = min((set_(trial, o, end, j) for j in cands), key=self.objective)
                        v = self.objective(trial)
                        if v < best - 1e-9:
                            best, parts, state, improved = v, self.score(trial)[1], trial, True
            log(f'  whole ends: sweep {sw}: objective {best:.2f} {parts}')
            if not improved:
                break
        self.pool[tuple(sorted(state.items()))] = best
        return state

    def iterate(self, state, rounds=None, log=print):
        """ITERATED local search from `state` (already a local optimum): `rounds` times, the best state with KICK lanes'
        ends moved to random options (a fixed seed: the same answer on every machine), searched again, kept when
        better. A one-lane search stops in the first basin it reaches: at K41 it found 74 vias from the greedy's berths
        and 66 from the human's"""
        import random
        rounds = ILS_ROUNDS if rounds is None else rounds
        rng = random.Random(ILS_SEED)
        best, (bv, bp) = dict(state), self.score(state)
        # (only lanes that can move: an incremental round holds most berths at one option, and a kick there does nothing)
        lanes = [l_ for l_, _lg in self.lanes if len(self.T[l_]) > 1 or len(self.B[l_]) > 1]
        if not lanes:
            return best
        idle = 0
        for r in range(rounds):
            if idle >= ILS_PATIENCE:
                break                   # that many kicks in a row found nothing: the basin is the best one near
            s_ = dict(best)
            for l_ in rng.sample(lanes, min(ILS_KICK, len(lanes))):
                e_ = rng.randrange(2) if len(self.T[l_]) > 1 else 1
                opts = self.T[l_] if e_ == 0 else self.B[l_]
                i_ = rng.randrange(len(opts))
                s_[l_] = (i_, s_[l_][1]) if e_ == 0 else (s_[l_][0], i_)
            s_ = self.search(s_, log=lambda *a: None)
            v_, p_ = self.score(s_)
            if v_ < bv - 1e-9:
                best, bv, bp = s_, v_, p_
                idle = 0
                log(f'  whole ends: kick {r}: objective {bv:.2f} {bp}')
            else:
                idle += 1
        return best

    def ban_kicks(self, state, log=print):
        """STRUCTURED kicks, after the random ones: BAN_KICKS times, the best state's BAN_LANES most crossed lanes not
        yet kicked (and able to move) have their current ends banned -- each moved to its cheapest other option clear
        of the rest -- the search run with the bans, then again without them, and the better state kept; BAN_PATIENCE
        kicks in a row finding nothing stop. A random kick moves lanes the search returns to; a banned end makes it
        find the next basin (K41: a fanout refusing one berth moved the ends from 191.1 to 187.5)"""
        best, (bv, bp) = dict(state), self.score(state)
        kicked, idle = set(), 0
        for k in range(BAN_KICKS):
            if idle >= BAN_PATIENCE:
                break
            x = bp.get('lane_x') or {}
            group = sorted((l_ for l_, _lg in self.lanes if l_ not in kicked
                            and (len(self.T[l_]) > 1 or len(self.B[l_]) > 1)),
                           key=lambda l_: (-x.get(l_, 0), l_))[:BAN_LANES]
            if not group:
                break
            kicked |= set(group)
            s_ = dict(best)
            self.tabu = set()
            for l_ in group:
                for e_ in ((0, 1) if len(self.T[l_]) > 1 else (1,)):
                    opts = self.T[l_] if e_ == 0 else self.B[l_]
                    cur = s_[l_][e_]
                    others = [i for i in range(len(opts)) if i != cur]
                    if not others:
                        continue
                    self.tabu.add((l_, e_, cur))
                    free = [i for i in others if self._clear(s_, l_, e_, i)] or others
                    i = min(free, key=lambda i: opts[i][3])
                    s_[l_] = (i, s_[l_][1]) if e_ == 0 else (s_[l_][0], i)
            try:
                s_ = self.search(s_, log=lambda *a: None)
            finally:
                self.tabu = set()
            s_ = self.search(s_, log=lambda *a: None)
            v_, p_ = self.score(s_)
            better = v_ < bv - 1e-9
            log(f'  whole ends: ban kick {k} ({", ".join(group)}): objective {v_:.2f}' + (' (best)' if better else ''))
            if better:
                best, bv, bp, idle = s_, v_, p_, 0
            else:
                idle += 1
        return best

    def choice(self, state):
        """(the berths {net: Move}, the teeth to MOVE {net: Move}) of a state"""
        dst, src = {}, {}
        for lane, lg in self.lanes:
            for leg, m in zip(lg, self.B[lane][state[lane][1]][0]):
                dst[leg] = m
            for leg, m in zip(lg, self.T[lane][state[lane][0]][0]):
                if m is not self.cur.get(leg):
                    src[leg] = m
        return dst, src


def wound_ride(a, b, box, side, pad=0.3, turns=False):
    """the ride from a to b round `box` grown by `pad` the long way, round its `side` ('N' or 'S') from the facing face:
    to that side's near corner, then corner to corner round the box until the face b stands on -- a WOUND lane's
    (cut_phase), which around_box, the shorter way past at most two corners, cannot price. `turns`: (the ride, the
    corners it turns)"""
    x0, y0, x1, y1 = box
    x0, y0, x1, y1 = x0 - pad, y0 - pad, x1 + pad, y1 + pad
    seq = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)] if side == 'N' else [(x0, y1), (x1, y1), (x1, y0), (x0, y0)]
    # (the faces reached after each corner, round that side: north / far / south, or south / far / north)
    faces = ['N', 'E', 'S', 'W'] if side == 'N' else ['S', 'E', 'N', 'W']
    d = {'E': abs(b[0] - x1), 'N': abs(b[1] - y0), 'W': abs(b[0] - x0), 'S': abs(b[1] - y1)}
    fb = min(d, key=d.get)
    path = [a]
    for c, f in zip(seq, faces):
        path.append(c)
        if f == fb:
            break
    path.append(b)
    d_ = sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(path, path[1:]))
    return (d_, len(path) - 2) if turns else d_


def wind_on():
    """WIND_CUT: the destination's cut anywhere round it (cut_phase), '1', or on its far face alone, '0'. Unset: on with
    more routing layers than two (route_layers), where a wound family takes its own layer through the channel beside
    the destination (synth wind_rot16_g3); off on two"""
    import route_layers
    v = awx_settings.get('WIND_CUT') or ('1' if len(route_layers.layers()) > 2 else '0')
    if v not in ('0', '1'):
        raise SystemExit(f'WIND_CUT={v!r}: expected 0 or 1')
    return v == '1'


WIND_SEEN = []      # the cuts off the far face the cut phase chose in this process, for judge to price a choice at


def cut_phase(E, state, vp, log=print):
    """WINDING: the destination's cut moved off its far face. A lane's way round the destination is fixed by where the
    cut stands: a berth past it is reached round the other side, so a cut on the destination's south face sends the
    lanes ending on the south face beyond it, and on the far face, round the north (a human's lanes from the source's
    northern rows looping round the destination to enter it from the south: zynq's U1 to U5). One cut for every lane:
    measured on the human's own route there, one cut leaves as few lanes out of order on a layer as a cut per layer.
    Each gap between the berths of `state` (the best ends on the far face's cut, `vp` their (objective, parts)) off the
    far face is a cut to try -- one a gap (the far face's own gaps are _score's tied cuts); each scored on those ends
    by the estimate, the best WIND_TRY searched again from them with the cut held (search, iterate) and ranked on the
    exact route; a cut is taken only when its ends beat the far cut's by WIND_GAIN. Returns (state, (objective,
    parts)) and leaves E.wcut at the cut taken (None: the far face's)"""
    import whole_frame
    quiet = lambda *a: None
    v0, p0 = vp
    pts = [E.B[l_][state[l_][1]][1] for l_, _lg in E.lanes]
    DB = whole_frame.grown(E.dbox, pts)
    seen = {whole_frame.cut_sig(pts, DB, p0.get('cut'))}
    cands = []
    E.wcut = None
    exits = [o[1] for l_, _lg in E.lanes for o in E.B[l_]] + [m.exit_pt for l_, _lg in E.lanes for o in E.B[l_]
                                                               for m in o[0]]
    for c in whole_frame.cut_gaps(pts, DB, avoid=exits):
        if whole_frame.far_cut(c):
            continue                    # (the far face's own gaps are _score's tied cuts: winding is a cut OFF it)
        sg = whole_frame.cut_sig(pts, DB, c)
        if sg in seen:
            continue
        seen.add(sg)
        E.wcut = c
        cands.append((E.objective(state), c))
    E.wcut = None
    best = (state, (v0, p0), None)
    rank = lambda vp_: (bool(vp_[1].get('exact_failed')), vp_[0])
    for v_est, c in sorted(cands, key=lambda t: (t[0], str(t[1])))[:WIND_TRY]:
        t_c = time.time()
        E.wcut = c
        E.pool = {}
        s_ = E.iterate(E.search(state, log=quiet), log=quiet)
        t_s = time.time() - t_c
        s_, vp_ = E.best_exact(s_, log=quiet)
        pv = lambda p_: (f'{p_["fan"] + p_["route"]} v, ride {p_["ride"]}, cong {p_.get("cong", 0)}, '
                         f'{p_["crossings"]} x')
        log(f'  whole ends: winding, the cut at {c[0]} {c[1]:.3f} (estimate {v_est:.2f} on the far cut\'s ends): '
            f'objective {vp_[0]:.2f} ({pv(vp_[1])}) against the far cut\'s {v0:.2f} ({pv(p0)}); '
            f'searched in {t_s:.0f} s, routed exact in {time.time() - t_c - t_s:.0f} s')
        # (taken when it lays FEWER vias than the far cut's ends (the fanout's and the route's, predicted) and beats
        # their objective by WIND_GAIN -- winding trades length for vias; a cut that saves none won on the estimate's
        # other terms alone, which its own search, held at another cut, had one more chance to move: shuf_k12s3_g3,
        # four layers, its ends wound for 16 vias against the far cut's 14, chosen for the far ends' two stacked ends
        # (10 in the objective) -- the far plan laid 14, the wound one 16. Where the far ends' exact route found no plan,
        # a wound one that has one is taken)
        vias_ = lambda p_: p_['fan'] + p_['route']
        better_ = rank(vp_) < (rank(best[1])[0], rank(best[1])[1] - (WIND_GAIN if best[2] is None else 1e-9))
        saves_ = vias_(vp_[1]) < vias_(p0) or (bool(p0.get('exact_failed')) and not vp_[1].get('exact_failed'))
        if better_ and saves_:
            best = (s_, vp_, c)
    E.wcut = best[2]
    if best[2] is not None:
        WIND_SEEN.append(best[2])
        log(f'  whole ends: WOUND -- the destination cut at {best[2][0]} {best[2][1]:.3f}: {_fmt(best[1][1])}')
    return best[0], best[1]


def choose(st, log=print, src_free=True, fixed=None, seed=None, learned=None, free_teeth=None):
    """the whole route's ends on a plan state: (berths {net: Move}, teeth to move {net: Move}). The berths are first
    searched against the teeth AS LAID (the laid objective, `choose.last['laid']`: what a realized board is judged
    on); with `src_free` the teeth are searched too, from there, and their moves asked only when that is better.
    `fixed` {net: move signature}: berths held; `seed` {net: Move}: the berths to start from; `learned`: berth pairs
    not to be laid together (Ends); `free_teeth` {lane}: with `src_free`, only these lanes' teeth move"""
    quiet = lambda *a: None
    E = Ends(st, src_free=False, fixed=fixed, learned=learned)
    if E.pw is not None:
        log(f'  whole ends: {len(E.pw.balls)} plane ball(s) of the arrays kept a way down ({E.pw.n_ways} ways, {E.pw.secs:.0f} s)')
    sL = E.ban_kicks(E.iterate(E.search(E.start(seed), log=quiet), log=quiet), log=log)
    sL, (vL, pL) = E.best_exact(sL, log=log)
    if wind_on():
        sL, (vL, pL) = cut_phase(E, sL, (vL, pL), log=log)
    out, vF, pF = E.choice(sL), None, None
    log(f'  whole ends: on the teeth as laid: {_fmt(pL)}')
    if src_free:
        E2 = Ends(st, src_free=True, fixed=fixed, learned=learned, free_teeth=free_teeth)
        # a local search from the berths just chosen on the teeth as laid, iterated
        sF = E2.ban_kicks(E2.iterate(E2.search(E2.start(out[0]), log=quiet), log=quiet), log=log)
        sF, (vF, pF) = E2.best_exact(sF, log=log)
        if wind_on():
            # (the teeth searched on the far face's cut first, then its own winding: held at the berths' cut, the teeth
            # missed the moves the far cut's ends wanted -- s4_bulge's five teeth, 12 vias to 8)
            sF, (vF, pF) = cut_phase(E2, sF, (vF, pF), log=log)
        if vF < vL - TOOTH_GAIN:
            out = E2.choice(sF)
            log(f'  whole ends: teeth moved ({len(out[1])}): {_fmt(pF)}')
        else:
            log('  whole ends: no tooth move is better')
    # (`st`: the plan state it chose on, and whether it asked tooth moves -- asked none, its berths ARE the choice on
    # that board's teeth as laid, and fanout_from_plan need not ask again)
    choose.last = dict(laid=vL, laid_parts=pL, free=vF, free_parts=pF, moved=bool(out[1]), st=id(st),
                       cut=(E2.wcut if out[1] else E.wcut))
    return out


def _corner_blocked(pcb, nid, layer, pts, bar):
    """[bool per point]: within `bar` of a foreign pad's copper on `layer` -- its copper as KiCad draws it, and in its
    corner's zone the router's corner buffer further (pairs.pad_corner_buffer), as the router keeps a single's track
    or via off it and the audit measures it (plan_audit.check_static); an unplated hole by its drill"""
    import numpy as _np
    P = _np.asarray(pts, float)
    out = _np.zeros(len(P), bool)
    x0, y0 = P.min(0) - bar - 3.0
    x1, y1 = P.max(0) + bar + 3.0
    for fp in pcb.footprints.values():
        for pd in fp.pads:
            if pd.net_id == nid and nid or not (x0 <= pd.global_x <= x1 and y0 <= pd.global_y <= y1):
                continue
            if pd.pad_type == 'np_thru_hole':
                d = _np.hypot(P[:, 0] - pd.global_x, P[:, 1] - pd.global_y) - (pd.drill or 0) / 2
            elif (pd.drill and pd.drill > 0) or '*.Cu' in pd.layers or layer in pd.layers:
                cr = _pairs.pad_corner_radius(pd)
                dx, dy = P[:, 0] - pd.global_x, P[:, 1] - pd.global_y
                d = (_pairs.pad_distance(dx, dy, pd.size_x / 2, pd.size_y / 2, cr)
                     - _pairs.pad_corner_buffer(dx, dy, pd.size_x / 2, pd.size_y / 2, cr, _bd.GRID))
            else:
                continue
            out |= d < bar - 1e-9
    return out


def _via_end(opt):
    """an end option (moves per leg, ...) whose every leg leaves by a via -- a dog-bone or a via in its pad: its lane's
    layer there the route's to choose (more routing layers than two)"""
    return bool(opt[0]) and all(getattr(m, 'kind', 'surface') != 'surface' for m in opt[0])


def _front(pcb, nid, m, kids):
    """(level, span) of a single lane's exit `m`: what stands in the straight run out of it, FRONT_REACH along its
    escape direction on its layer, against copper outside the run (braid.build_obstacles: every foreign pad, segment
    and via, inflated by clearance and half a track; the segments of the run's nets `kids` left out, as they move with
    the search; a pad also by the router's bar at its corners, _corner_blocked). Level 0 where the run is clear; 1
    where it is blocked but a via fits on the way before the block (clear of that copper on every layer by a via's
    room, a barrel piercing them all); 2 where none does. `span`, the layer change the solve plans there (whole_solve's
    LAYER cuts): (via_near, via_far, near, far) in mm from the exit -- where such a via stands with the run on another
    layer clear from it across the block, and the span of the copper blocking the track beyond it (the first step that
    meets it, the first past the last within the reach that does); None where no via stands so, a block on every layer
    (a barrel, a hole) among them: the lane goes round that, priced as the via its bend is like. Kept on the board per
    exit"""
    from escape_moves import DIRS
    d = DIRS.get(getattr(m, 'direction', None))
    if d is None or getattr(m, 'exit_pt', None) is None:
        return 0, None
    key = (nid, m.layer, m.direction, round(m.exit_pt[0], 4), round(m.exit_pt[1], 4))
    memo = pcb.__dict__.setdefault('_exit_front', {})
    if key in memo:
        return memo[key]
    kids = frozenset(kids) | {nid}
    step, (x0, y0) = 0.05, m.exit_pt
    at = lambda k: (x0 + d[0] * step * k, y0 + d[1] * step * k)
    n = int(FRONT_REACH / step + 1e-9)
    pts = [at(k) for k in range(n + 1)]
    layers = list(getattr(pcb.board_info, 'copper_layers', None) or ('F.Cu', 'B.Cu'))
    t_bar, v_bar = _bd.CLEAR + _bd.TRACK / 2, _bd.VIA_SIZE / 2 + _bd.CLEAR
    cnr = {L: _corner_blocked(pcb, nid, L, pts, t_bar) for L in layers}
    track = _bd.build_obstacles(pcb, nid, kids, m.layer)
    blocked = lambda o, L, k: o.point_violation(at(k)) or cnr[L][k]
    hits = [k for k in range(1, n + 1) if blocked(track, m.layer, k)]
    import route_layers
    RL = [L for L in route_layers.layers() if L in layers]
    if hits and len(RL) > 2 and getattr(m, 'kind', 'surface') != 'surface':
        # (a VIA END on more routing layers than two: its run out, as the relayer lays it, on any of them -- clear where
        # any one is)
        for L_ in RL:
            if L_ != m.layer and not any(blocked(_bd.build_obstacles(pcb, nid, kids, L_), L_, k) for k in range(1, n + 1)):
                hits = []
                break
    k0 = hits[0] if hits else None
    out = (0, None)
    if k0 is not None:
        k1 = min(hits[-1] + 1, n)          # (past the last step within the reach that meets copper)
        vias = [_bd.build_obstacles(pcb, nid, kids, L, margin=v_bar) for L in layers]
        vcn = [_corner_blocked(pcb, nid, L, pts[:k0], v_bar) for L in layers]
        fit = [k for k in range(k0) if not any(v.point_violation(at(k)) or c[k] for v, c in zip(vias, vcn))]
        # (more routing layers than two: the layer past its via one of the ROUTING layers -- a copper layer the route
        # does not lay on is no way past)
        others = [(_bd.build_obstacles(pcb, nid, kids, L), L) for L in (RL if len(RL) > 2 else layers) if L != m.layer]
        # (the lane past its via on another layer, clear from the via across the block: a layer it can stand on)
        on = [k for k in fit if any(not any(blocked(o, L, j) for j in range(k, k1 + 1)) for o, L in others)]
        out = (1 if fit else 2, (round(min(on) * step, 3), round(max(on) * step, 3), round(k0 * step, 3),
                                 round(k1 * step, 3)) if on else None)
    memo[key] = out
    return out


def plane_ways(st, lanes, T, B, cur):
    """The joint fanout's PLANE balls kept a way down by the ends (FANOUT_JOINT; None without it). Each plane ball of
    the two arrays (the spec's `drops`) has its WAYS: its drops as the joint escape plans them (joint_escape._drops: a
    via behind a stub in a diagonal gap or straight off an edge, at the rung's via or a finer rung's, or a via in its
    pad), measured on the board without the run's copper, and its straps to a neighbouring ball of its net
    (joint_escape._straps), which serve it while that ball has a way of its own. Each end option KILLS the ways its
    copper stands on -- its legs at the fan track (a laid tooth's: its net's copper at the array), its via, and its
    lane's straight run past its exit, held off a drop's via by the joint escape's own bar (joint_escape._lanes_clear).
    Unmeasured, the ends ran the zynq DDR's lanes over every way down three of U1's VCC_1V5 balls had (a bus track on
    B.Cu under each pad, bus stubs in every gap round it), and the joint escape found them walled in.

    Returns a namespace: `balls` [(ball, [drop ways], [(strap way, the ball it joins)])] -- only the balls with a way
    on the board without the run; a ball with none is walled whatever the ends -- `kill` {(lane, end, option
    index): frozenset of ways}, `ways` (each way by its index: its array `end`, its via's `site`, `r` and `dr`, its
    `stub` or a strap's on `layer`), and `secs` (its own build time)"""
    t0 = time.time()
    path = awx_settings.get('FANOUT_JOINT')
    if not path:
        return None
    import copy
    import json
    import types
    import escape_moves as em
    import joint_escape as je
    import pair_teeth as pt
    import source_realize as sr
    from escape_moves import DIRS
    with open(path, encoding='utf-8') as f:
        spec = json.load(f)
    pcb = st['pcb']
    run = {l_ for _l, lg in lanes for l_ in lg}
    run_ids = {st['byname'][l_][0] for l_ in run}
    pcb0 = copy.copy(pcb)
    pcb0.segments = [s for s in pcb.segments if s.net_id not in run_ids]
    pcb0.vias = [v for v in pcb.vias if v.net_id not in run_ids]
    ways, balls, at = [], [], {}
    for end, ref in ((0, st['sref']), (1, st['dref'])):
        ar = next((a for a in spec.get('arrays', ()) if a['ref'] == ref), None)
        foot = pcb0.footprints.get(ref)
        if ar is None or foot is None:
            continue
        drop_s = ({je.short_name(n) for n in ar.get('drops', ())} - {je.short_name(n) for n in ar.get('others', ())}
                  - run)
        items = {f'{je.short_name(p.net_name)}#{p.pad_number}': (je.short_name(p.net_name), p)
                 for p in foot.pads if p.net_id and p.net_name and je.short_name(p.net_name) in drop_s}
        if not items:
            continue
        grid = em.grid_of(foot)
        sz = je._sizes(pcb0, foot)
        skip = je.movable_refs(pcb0, ref)
        cache = {}

        def obs(nid, layer, via=False, _c=cache, _sz=sz, _skip=skip):
            k = (nid, layer, via)
            if k not in _c:
                _c[k] = _bd.build_obstacles(pcb0, nid, {nid}, layer, margin=_sz['cl'] + _sz['tw'] / 2,
                                            skip_refs=_skip if via else ())
            return _c[k]
        regions = je.zone_regions(pcb0)
        dw = {}
        for key, (_nm, p) in sorted(items.items()):
            dw[key] = []
            for d in je._drops(pcb0, grid, p, obs, sz, foot, (), regions):
                dw[key].append(len(ways))
                ways.append(dict(end=end, site=tuple(d.site), r=d.r, dr=d.dr, stub=d.stub, layer=d.layer))
        sw = collections.defaultdict(list)
        for key, ss in je._straps(pcb0, grid, items, obs).items():
            for s in ss:
                sw[key].append((len(ways), (end, s.to)))
                ways.append(dict(end=end, stub=(tuple(s.a), tuple(s.b)), layer=s.layer))
        for key in sorted(items):
            if dw[key] or sw[key]:
                balls.append(((end, key), dw[key], sw[key]))
        at[end] = dict(sz=sz, box=grid.bbox, half=max(grid.pitch_x, grid.pitch_y) / 2.0)
    if not balls:
        return types.SimpleNamespace(balls=[], kill={}, ways=ways, n_ways=0, secs=time.time() - t0)
    # the ways by 1 mm cell, per array, for the options' copper to find
    cells = collections.defaultdict(list)
    for i, w in enumerate(ways):
        pts = [w['site']] if 'site' in w else list(w['stub'])
        for q in pts + (list(w['stub']) if w.get('stub') else []):
            c = (w['end'], int(math.floor(q[0])), int(math.floor(q[1])))
            if i not in cells[c]:
                cells[c].append(i)
    hw_fan, cl = sr.FAN_TRACK / 2.0, sr.FAN_CLEAR
    bar = _bd.TRACK / 2 + _bd.CLEAR + _bd.GRID / 2          # (joint_escape._lanes_clear's)
    by_net = collections.defaultdict(lambda: ([], []))
    for s in pcb.segments:
        if s.net_id in run_ids:
            by_net[s.net_id][0].append(((s.start_x, s.start_y), (s.end_x, s.end_y), s.layer, s.width / 2.0))
    for v in pcb.vias:
        if v.net_id in run_ids:
            by_net[v.net_id][1].append(((v.x, v.y), v.size / 2.0, (v.drill or 0.0) / 2.0))

    def copper(end, leg, m):
        segs, vias = [], []
        pad = (st['src_pad'] if end == 0 else st['dst_pad']).get(leg)
        if m.legs or m is not cur.get(leg):
            segs = [(tuple(a), tuple(b), L, hw_fan) for a, b, L in (m.legs or [])]
            if m.site is not None:
                own = pad is not None and math.hypot(m.site[0] - pad.global_x, m.site[1] - pad.global_y) < 1e-6
                r, dr = at[end]['sz']['inpad'](pad) if own else (at[end]['sz']['vr'], at[end]['sz']['vdr'])
                vias = [(tuple(m.site), r, dr)]
        else:
            # (a tooth as laid: its net's copper at the array, read off the board)
            x0, y0, x1, y1 = at[end]['box']
            near = lambda q: x0 - 2 <= q[0] <= x1 + 2 and y0 - 2 <= q[1] <= y1 + 2
            sg, vs = by_net[st['byname'][leg][0]]
            segs = [s for s in sg if near(s[0]) or near(s[1])]
            vias = [v for v in vs if near(v[0])]
        d = DIRS.get(getattr(m, 'direction', None))
        ray = None
        if d is not None and getattr(m, 'exit_pt', None) is not None:
            e = tuple(m.exit_pt)
            ray = (e, (e[0] + d[0] * je.LANE_REACH, e[1] + d[1] * je.LANE_REACH))
        return segs, vias, ray

    def kills(w, segs, vias, ray, h2h):
        if 'site' in w:
            s, r = w['site'], w['r']
            if any(_seg_d(s, s, a, b) < r + hw + cl - 1e-9 for a, b, _L, hw in segs):
                return True
            for c, rv, dv in vias:
                dd = math.hypot(c[0] - s[0], c[1] - s[1])
                if dd < r + rv + cl - 1e-9 or dd < w['dr'] + dv + h2h - 1e-9:
                    return True
            if ray is not None and _seg_d(s, s, ray[0], ray[1]) - r < bar - 1e-9:
                return True
        if w.get('stub'):
            a0, b0 = w['stub']
            if any(L == w['layer'] and _seg_d(a0, b0, a, b) < hw_fan + hw + cl - 1e-9 for a, b, L, hw in segs):
                return True
            if any(_seg_d(c, c, a0, b0) < rv + hw_fan + cl - 1e-9 for c, rv, _dv in vias):
                return True
        return False
    kill = {}
    for lane, lg in lanes:
        for end, opts in ((0, T[lane]), (1, B[lane])):
            if end not in at:
                continue
            for i, o in enumerate(opts):
                segs, vias, rays = [], [], []
                for leg, m in zip(lg, o[0]):
                    s_, v_, r_ = copper(end, leg, m)
                    segs += s_
                    vias += v_
                    if r_ is not None:
                        rays.append(r_)
                pts = [q for a, b, _L, _h in segs for q in (a, b)] + [c for c, _r, _d in vias] + \
                    [q for r_ in rays for q in r_]
                if not pts:
                    continue
                cand = set()
                for cx in range(int(math.floor(min(q[0] for q in pts))) - 1, int(math.floor(max(q[0] for q in pts))) + 2):
                    for cy in range(int(math.floor(min(q[1] for q in pts))) - 1,
                                    int(math.floor(max(q[1] for q in pts))) + 2):
                        cand.update(cells.get((end, cx, cy), ()))
                h2h = at[end]['sz']['h2h']
                # (a pair's two exits by one face on one layer: its POCKET too, the pair's room to close past them,
                # where the joint escape lays nothing -- joint_escape.laid_pockets)
                pk = None
                if len(lg) == 2 and o[0][0].direction == o[0][1].direction and o[0][0].layer == o[0][1].layer \
                        and DIRS.get(o[0][0].direction) is not None:
                    pk = pt.pockets({0: (0, 1)}, {0: (o[0][0].exit_pt, DIRS[o[0][0].direction], o[0][0].layer),
                                                  1: (o[0][1].exit_pt, DIRS[o[0][1].direction], o[0][1].layer)},
                                    at[end]['half'])[0]

                def in_pk(w):
                    if pk is None:
                        return False
                    if 'site' in w and pt.in_pocket(pk[2], w['site']):
                        return True
                    return bool(w.get('stub')) and w['layer'] == pk[0] and pt.in_pocket(pk[2], *w['stub'])
                k_ = frozenset(wi for wi in cand if kills(ways[wi], segs, vias, None, h2h)
                               or any(kills(ways[wi], [], [], r_, h2h) for r_ in rays) or in_pk(ways[wi]))
                if k_:
                    kill[(lane, end, i)] = k_
    return types.SimpleNamespace(balls=balls, kill=kill, ways=ways, n_ways=len(ways), secs=time.time() - t0)


def walled_balls(pw, lanes, state):
    """the plane balls `pw` (plane_ways) the ends `state` leave no way down, as (array: 0 source, 1 destination,
    NET#PAD): every drop of theirs killed by a chosen end, and every strap either killed or to a ball with no drop of
    its own left"""
    if not pw or not pw.balls:
        return []
    killed = set()
    for l_, _lg in lanes:
        killed |= pw.kill.get((l_, 0, state[l_][0]), frozenset())
        killed |= pw.kill.get((l_, 1, state[l_][1]), frozenset())
    alive = {b: any(w not in killed for w in dw) for b, dw, _sw in pw.balls}
    return [b for b, dw, sw in pw.balls if not alive[b] and not any(w not in killed and alive.get(to) for w, to in sw)]


def exit_front(pcb, nid, m, kids):
    """the level of what stands in front of a single lane's exit `m` (_front): 0 clear, 1 blocked with room for a via
    before the block, 2 blocked with none"""
    return _front(pcb, nid, m, kids)[0]


def front_span(pcb, nid, m, kids):
    """(via_near, via_far, near, far) of a single lane's exit `m` blocked where a layer change answers it (_front):
    where its via stands and where the copper blocking it does, in mm from the exit; None for any other exit"""
    return _front(pcb, nid, m, kids)[1]


def _fb_nl():
    import route_layers
    return len(route_layers.layers())


def fb_weights(opts, item):
    """{option: weight}: what a feedback ITEM (whole_feedback: {'layer', 'points', 'times'}) prices each of a lane's
    options at one end (`opts`: (moves per leg, point, layer, ...)) at, by PLACE -- in full where ANY leg's exit stands
    at ANY of its points (within DUP_TOL); off the place at most FB_OTHER_LAYER of that, falling linearly to nothing
    FB_RADIUS beyond; and FB_OTHER_LAYER of either on the other layer. A layer change at the place is some answer and
    moving away a better one, the nearest other exit included (zynq K44: with the fall alone, a tooth one exit over,
    0.4 mm, paid more than the other layer at the place, and round 2 moved nothing); a pair cannot leave the price by
    moving one leg and keeping the other where it was found crowded -- and by ESCALATION: FB_ESCALATE times more for
    each later round it was named again"""
    esc = FB_ESCALATE ** (int(item.get('times', 1)) - 1)
    _FB_NL = _fb_nl()
    out = {}
    for i, o in enumerate(opts):
        d = min((math.hypot(m.exit_pt[0] - p_[0], m.exit_pt[1] - p_[1]) for m in o[0] for p_ in item['points']),
                default=math.inf)
        place = 1.0 if d <= DUP_TOL else FB_OTHER_LAYER * max(0.0, 1.0 - (d - DUP_TOL) / FB_RADIUS)
        if place > 0.0:
            # (more routing layers than two: a via end's layer is the route's to choose -- no other layer at the place)
            other_ok = not (_FB_NL > 2 and _via_end(o))
            out[i] = esc * place * (1.0 if o[2] == item['layer'] or not other_ok else FB_OTHER_LAYER)
    return out


def _stacks(lanes, to, bo, chg):
    """the STACKED pairs of ends -- two lanes' legs at one point (within DUP_TOL) on different layers, at the teeth or
    at the berths -- where either lane changes layer; one where either lane is a PAIR counts twice (a pair's dive is two
    barrels side by side under the other lane's via: K51, SDQS1 under SDQ8)"""
    npair = {l_: len(lg) for l_, lg in lanes}
    n = 0
    for opt in (to, bo):
        cells = {}
        for l_, _lg in lanes:
            for m in opt[l_][0]:
                cells.setdefault((int(math.floor(m.exit_pt[0] / 0.25)), int(math.floor(m.exit_pt[1] / 0.25))),
                                 []).append((l_, m))
        seen = set()
        for (cx, cy), here in cells.items():
            near = [e for dx in (-1, 0, 1) for dy in (-1, 0, 1) for e in cells.get((cx + dx, cy + dy), ())]
            for la, ma in here:
                for lb, mb in near:
                    if la == lb or ma.layer == mb.layer or not (chg.get(la) or chg.get(lb)):
                        continue
                    if math.hypot(ma.exit_pt[0] - mb.exit_pt[0], ma.exit_pt[1] - mb.exit_pt[1]) > DUP_TOL:
                        continue
                    key = tuple(sorted([(la, id(ma)), (lb, id(mb))]))
                    if key not in seen:
                        seen.add(key)
                        n += 2 if npair[la] > 1 or npair[lb] > 1 else 1
    return n


def judge(st, choice):
    """(objective, parts) of berths `choice` {net: Move} against the teeth as laid -- a net `choice` leaves out at
    its greedy berth"""
    import source_realize as sr
    E = Ends(st, src_free=False, fixed={nm: sr.move_sig(m) for nm, m in choice.items()})
    s0 = E.start()
    best = E.score(s0, exact=True)
    # (WINDING: at the best of the far face's cut and the cuts the cut phase took in this process -- the model's own
    # best cut, which the plan sidecar then carries to the whole route's frame)
    for c in dict.fromkeys(WIND_SEEN):
        E.wcut = c
        got = E.score(s0, exact=True)
        if (bool(got[1].get('exact_failed')), got[0]) < (bool(best[1].get('exact_failed')), best[0] - 1e-9):
            best = got
    return best


def _fmt(p):
    return (f'{p["fan"] + p["route"]} vias predicted (fanout {p["fan"]}, route >= {p["route"]}: {p["parity"]} end-layer '
            f'changes + {p["crossover"]} crossover + 2 x {p["cover"]} + {p["settle"]} settling + {p["couple"]} coupled), '
            f'{p["over"]} over two on a net, '
            f'ride {p["ride"]} mm, congestion {p.get("cong", 0)}, ' + (f'feedback {p["feedback"]}, ' if p.get('feedback') else '') +
            (f'blocked in front {p["front"]}, ' if p.get('front') else '') +
            f'{p["crossings"]} crossings ({p["same"]} on one layer)' + ''.join(f', {p[k]} {k}' for k in ('conflicts', 'splits', 'refused', 'stacks') if p.get(k))
            + (f', {len(p["walled"])} plane ball(s) walled in ('
               + ', '.join(('source ' if e == 0 else 'destination ') + k for e, k in p['walled']) + ')'
               if p.get('walled') else '')
            + (f' [the route exact on the orders: {p["route"]}, estimated {p["route_est"]}]'
               if p.get('route_est') is not None and p['route_est'] != p['route'] else ''))


if __name__ == '__main__':
    os.environ.setdefault('PLAN_JUDGE', 'ends')      # (read by fanout_from_plan at import: the menus the ends model takes)
    from kicad_parser import parse_kicad_pcb
    import fanout_from_plan as F
    board, nets = sys.argv[1], sys.argv[2]
    names = [x for x in (open(nets[1:]).read().split() if nets.startswith('@') else nets.split(',')) if x]
    st = F.plan_state(parse_kicad_pcb(board), names)
    dst, src = choose(st)
    print(f'  {len(src)} tooth move(s): {sorted(src)}')

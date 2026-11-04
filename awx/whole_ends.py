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
import contextlib
import copy
import io
import itertools
import json
import math
import os
import awx_settings
import cp_seed  # noqa: E402  (every CP-SAT solve under the bus step's seed)
import sys
import time
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))

import pairs as _pairs  # noqa: E402
import braid as _bd  # noqa: E402
import select_moves as _sm  # noqa: E402
import ends_kernel as _ek  # noqa: E402
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
POCKET_PASSES = 2         # (a group) every pocket split again, a pocket's lanes at once (pocket_kicks), this many passes
ILS_SEED = int(awx_settings.get('ENDS_ILS_SEED', '622'))   # the kicks' seed (a setting: one bench, other trajectories)
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
ONE_WINDING = (awx_settings.get('ENDS_ONE_WINDING') or '0') == '1'   # (probe) no second search with the teeth
#                           free where no tooth is laid (an escape base): its model is the first's
WIND_GAIN = 1.0           # ...a cut taken only when its ends lay fewer vias and beat the far face's by this much: within
#                           it, the model's own error on a wound plan (its rides, the stubs the fanout lays) -- h3 K28
#                           took two cuts at 1.2 and 0.18 and laid 30 vias and 584 mm against the far cut's 32 and 544
EXACT_WORK = 60.0         # CP-SAT's deterministic work limit for one exact route (a machine's deterministic time is
#                           its own, so the limit stands far above a proof's: H3 K51's best ends prove within 1.1)
_EXACT_MEMO = {}          # an exact route's model (its text's hash) -> its answer: one model is one answer, solved once in
#                           the process -- a re-plan asks the first plan's models again (the two-end USB and Ethernet
#                           bus's fanout: 7 of its 27 four-layer exact routes, at 3 to 14 s each)


def _exact_solve(m):
    """(status, the value of each variable by index, or None) of the exact route's model `m`: one worker, the work
    EXACT_WORK -- the same answer on every run -- searching CORE-GUIDED (the objective is a sum of the lanes' changes and
    dropped vias, nearly all of them none: the bound raised by the cores of 'none here' that cannot all hold). The
    default search routed h616_lpddr4's thirteen states of one choice in 963 s, five of them unproved; core-guided, in
    192 s, every one proved, and in 29 s on orders (_crossing_orders) with the layers bools. Memoised on the model
    itself (_EXACT_MEMO)"""
    import hashlib
    from ortools.sat.python import cp_model
    key = hashlib.sha1(str(m.Proto()).encode()).hexdigest()
    if key not in _EXACT_MEMO:
        sol = cp_seed.solver()
        sol.parameters.num_workers = 1
        sol.parameters.max_deterministic_time = EXACT_WORK
        sol.parameters.optimize_with_core = True
        r = sol.Solve(m)
        _EXACT_MEMO[key] = (r, [sol.Value(m.GetIntVarFromProtoIndex(i)) for i in range(len(m.Proto().variables))]
                            if r in (cp_model.OPTIMAL, cp_model.FEASIBLE) else None)
    return _EXACT_MEMO[key]


def _crossing_orders(m, lanes, Ln, X):
    """(before, act) of an exact route's model `m` on its crossings X (the inverted pairs of the order Ln), ordered
    WITHOUT positions: a bool per fully inverted triple orders its three crossings (the braid triple rule: ij, ik, jk
    or the reverse), a triple of two inversions its two, and each lane's crossings, so ordered, transitively
    (consistent on every triple, a lane's order is an arrangement's: Goodman and Pollack's local sequences). Each
    lane's EXACT_KMAX changes act[k] (in turn: act[k] needs act[k - 1]) and before[(lane, crossing)][k], change k
    before the crossing, never falling along the lane's order. Positions (an integer per crossing, and per change)
    held one order in every relabelling of them: thirteen states of a choice of h616_lpddr4's routed in 192 s on
    positions, 99 s on orders"""
    Xs = set(X)
    rel = {}                            # (lane, p, q): its crossing with p before its crossing with q
    neg = lambda x: (not x) if isinstance(x, bool) else x.Not()

    def order(l_, p, q, lit):
        rel[(l_, p, q)], rel[(l_, q, p)] = lit, neg(lit)
    for i, j, k in itertools.combinations(Ln, 3):
        ij, ik, jk = (i, j) in Xs, (i, k) in Xs, (j, k) in Xs
        if ij and ik and jk:
            b_ = m.NewBoolVar('')       # ij < ik < jk, else jk < ik < ij
            order(i, j, k, b_); order(j, i, k, b_); order(k, i, j, b_)
        elif ij and ik:
            order(i, j, k, True)
        elif ik and jk:
            order(k, j, i, True)

    def clause(lits):
        if not any(x is True for x in lits):
            m.AddBoolOr([x for x in lits if x is not False])
    ev = {l_: [] for l_ in lanes}
    for key_ in X:
        for l_ in key_:
            ev[l_].append(key_)
    other = lambda l_, key_: key_[1] if key_[0] == l_ else key_[0]
    before, acts = {}, {}
    for l_ in lanes:
        ps = [other(l_, key_) for key_ in ev[l_]]
        for p, q, r in itertools.combinations(ps, 3):
            x, y, z = rel[(l_, p, q)], rel[(l_, q, r)], rel[(l_, p, r)]
            if not all(isinstance(v, bool) for v in (x, y, z)):
                clause([neg(x), neg(y), z]); clause([x, y, neg(z)])
        act = [m.NewBoolVar('') for _ in range(EXACT_KMAX)]
        for k in range(1, EXACT_KMAX):
            m.AddImplication(act[k], act[k - 1])
        for key_ in ev[l_]:
            bits = [m.NewBoolVar('') for _ in range(EXACT_KMAX)]
            for k in range(EXACT_KMAX):
                m.AddImplication(bits[k], act[k])
                if k:
                    m.AddImplication(bits[k], bits[k - 1])
            before[(l_, key_)] = bits
        for k1, k2 in itertools.combinations(ev[l_], 2):
            r_ = rel[(l_, other(l_, k1), other(l_, k2))]
            for k in range(EXACT_KMAX):
                clause([neg(r_), before[(l_, k1)][k].Not(), before[(l_, k2)][k]])
                clause([r_, before[(l_, k2)][k].Not(), before[(l_, k1)][k]])
        acts[l_] = act
    return before, acts


def _clique_bound(m, act, lanes, X, NL):
    """each maximal clique of an exact route's crossing graph (lanes every two of which cross) past NL lanes: that many
    of them change layer at least (`act`: each lane's changes), for lanes that keep one layer and all cross need a
    layer each. The graph is a permutation's, so perfect: without those cliques its lanes could keep a layer each.
    Redundant, the colouring's pigeonhole, which the core-guided search otherwise proves by search -- and on
    h616_lpddr4's thirteen models of one choice, under one of five solver seeds, did not within EXACT_WORK (a bound
    stuck at its least); with it, all thirteen proved under each of the five"""
    ix = {l_: i for i, l_ in enumerate(lanes)}
    nb = {l_: set() for l_ in lanes}
    for a, b in X:
        nb[a].add(b); nb[b].add(a)
    srt = lambda S: sorted(S, key=ix.get)  # (lane order, never a set's: the same model on every run)

    def bk(R, P, Q):                    # (Bron and Kerbosch's, pivoting)
        if not P and not Q:
            if len(R) > NL:
                m.Add(sum(act[l_][0] for l_ in srt(R)) >= len(R) - NL)
            return
        u = max(srt(P | Q), key=lambda v: len(nb[v] & P))
        for v in srt(P - nb[u]):
            bk(R | {v}, P & nb[v], Q & nb[v])
            P, Q = P - {v}, Q | {v}
    bk(set(), set(lanes), set())



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
TIE_MM = 0.005            # exits nearer than this round a box are ONE exit, whose lanes cross no one there (whole_frame's)
FRONT_REACH = 1.2         # a single lane's exit: its straight run out, this far along its escape (as a pair's,
#                           fanout_from_plan.PAIR_EXIT_REACH), against copper outside the run on its layer (exit_front) --
FRONT_VIA = 1.0           # ... blocked, with room for a via before the block: the lane changes layer there ...
FRONT_BLOCKED = 1000.0    # ... blocked with no room for one: past any via count, yet no refusal (a lane with no other end
#                           keeps it)
FAR_TOOTH = 1000.0        # a tooth on the source's FAR face (the one
#                           turned away from the destination): the lane goes round the source (whole_frame's lead-in),
#                           so past any via count, yet no refusal -- taken only by a lane with no other way out of the
#                           source (sdi_mipi_conv's SDI bus on U11's columns beside its far face)
FENCE_OUT = 1.5           # the band round a source side face its lanes ride in: this far outside its pad box, mm
FENCE_VIAS = 2.0          # a lane round a FENCED side (other nets' copper across that band: side_fences) -- a far-face
#                           lane's side of the cut, a side face's tooth -- a layer change under the fence and one back
FENCE_LAID = 1.0          # ...and each LAID lane of another bus across that band (an earlier step's locked copper off
#                           a shared array) between the lane's exit and its side's corner, a via more: where another bus
#                           fans out of the array, a lane crossing it there finds the least room for a via. A lane whose
#                           every way out but the far face crosses one is not held to the far face's last resort
#                           (Ends.far_w: FAR_TOOTH waived) -- it goes round the side the other bus leaves free
AWAY_FACE = 5.0           # a tooth on a side face of the source's other half from its ball -- neither toward the
#                           destination nor on the ball's own half (joint_escape's esc_faces, an escape bus's rule):
#                           its escape runs under the array through the other half's balls' way out, each leg so
POCKET_ROW = 0.01         # (plane_ways) a pair's pocket starts this much short of the outer ball row's centres
WALLED = 1000.0           # a plane ball of the arrays its ends leave no way down (the joint fanout: plane_ways) -- past
#                           any via count, yet no refusal
FAR = {'left': (-1, 0), 'right': (1, 0), 'up': (0, -1), 'down': (0, 1)}


def side_fences(pcb, box, skip, out=FENCE_OUT):
    """{'N': [x, ..], 'S': [x, ..]}: where other nets' copper -- routed copper, a bus an earlier step laid -- crosses
    the line `out` outside the source's pad box's north face and its south face, corner to corner, on the routing
    layers: its TRACKS across the line, and its VIAS within half that of it, a barrel on every layer (`skip`: the run's
    net names). What a lane round that side crosses: sdi_mipi_conv's DDR3 bus leaves U11's south face on four of its
    eight layers and drops to the planes there, and the SDI lanes sent round the south -- far-face lanes, and teeth on
    the south face -- were found against it every round (the north side free)"""
    import route_layers
    lays = set(route_layers.layers())
    x0, x1 = box[0] - out, box[2] + out
    from coherent_nets import namer
    sn = namer(pcb)
    name = lambda i: sn(pcb.nets[i].name) if i in pcb.nets else ''
    res = {}
    for side, y in (('N', box[1] - out), ('S', box[3] + out)):
        xs = []
        for s_ in pcb.segments:
            if s_.layer not in lays or getattr(s_, 'graphic', False):
                continue
            nm = name(s_.net_id)
            if not nm or nm in skip or (s_.start_y - y) * (s_.end_y - y) > 0 or s_.start_y == s_.end_y:
                continue
            x = s_.start_x + (y - s_.start_y) / (s_.end_y - s_.start_y) * (s_.end_x - s_.start_x)
            if x0 <= x <= x1:
                xs.append(x)
        xs += [v_.x for v_ in pcb.vias if x0 <= v_.x <= x1 and abs(v_.y - y) <= out / 2
               and name(v_.net_id) and name(v_.net_id) not in skip]
        res[side] = sorted(xs)
    return res


def _box_run(a, b, box, n=64):
    """the length of the straight a->b inside box"""
    k = 0
    for j in range(n):
        t = (j + 0.5) / n
        x, y = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
        if box[0] < x < box[2] and box[1] < y < box[3]:
            k += 1
    return k / n * math.hypot(b[0] - a[0], b[1] - a[1])


def corner_slope(spine, p0, p1, box, tol=0.05):
    """the least slope |do/ds| on `spine` a lane from p0 to p1 needs PAST the corner of `box` it must go round -- its
    straight through the box, the corner the shorter detour outside it takes; 0 where the straight is clear, inf where
    p1 stands no further along the spine than that corner"""
    if _box_run(p0, p1, box) < tol:
        return 0.0
    best = None
    for c in ((box[0], box[1]), (box[0], box[3]), (box[2], box[1]), (box[2], box[3])):
        if _box_run(p0, c, box) > tol or _box_run(c, p1, box) > tol:
            continue
        d = math.hypot(c[0] - p0[0], c[1] - p0[1]) + math.hypot(p1[0] - c[0], p1[1] - c[1])
        if best is None or d < best[0]:
            best = (d, c)
    if best is None:
        return math.inf
    sc, oc = spine.project_pt(best[1])
    s1, o1 = spine.project_pt(p1)
    return float(abs(o1 - oc) / (s1 - sc)) if s1 - sc > 1e-6 else math.inf


def frame_spine(SB, DB):
    """the whole frame's straight trunk spine (whole_frame.build's) from the two grown boxes"""
    import numpy as np
    import corridor as _cr
    ca = np.array([(SB[0] + SB[2]) / 2, (SB[1] + SB[3]) / 2])
    cb = np.array([(DB[0] + DB[2]) / 2, (DB[1] + DB[3]) / 2])
    d = (cb - ca) / np.hypot(*(cb - ca))
    back = math.hypot(SB[2] - SB[0], SB[3] - SB[1]) / 2 + _bd.LPITCH
    fwd = math.hypot(DB[2] - DB[0], DB[3] - DB[1]) / 2 + _bd.LPITCH
    return _cr.Spine([tuple(ca - d * back), tuple(cb + d * fwd)])



OPP_DIVE = float(awx_settings.get('ENDS_OPP_DIVE') or '0')   # (probe) vias for a crossover riding on a pair's dive


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


_pt_seg = lambda p, a, b: _seg_d(p, p, a, b)        # (the least distance from point p to segment ab)



class Ends:
    """the lanes of one plan state (a pair one lane) with their tooth and berth options, and the objective"""

    def __init__(self, st, src_free=True, fixed=None, learned=None, free_teeth=None):
        import pages_first as pf
        import source_realize as sr
        self.st = st
        # (a lane with no option at an end is no lane -- the run leaves it out, and names it: on an escape base its
        # teeth are its menu's alone, so with no tooth escape it is none either -- zynq U1's CTRL_OUT0, its via
        # walled in, crashed the joint ends)
        names = [n for n in st['launch'] if st['dmenu'].get(n) and (st['smenu'].get(n) or not st.get('escape'))]
        prs = {}
        if int(awx_settings.get('PLAN_PAIRS', awx_settings.get('BRAID_PAIRS', '0')) or 0):
            prs = {b: pr for b, pr in _pairs.pair_names(names).items() if pr[0] in names and pr[1] in names}
        legs = {l_ for pr in prs.values() for l_ in pr}
        self.lanes = [(n, (n,)) for n in names if n not in legs] + [(b, pr) for b, pr in prs.items()]
        self.legs_of = dict(self.lanes)
        # (DEST naming several parts, one source and several ends: ONE destination, the GROUP of them -- each lane's own
        # part, and each berth option once for each WAY its lane enters the group, its place on the group's walk the
        # way's: whole_frame.Group, the frame's own)
        import whole_frame
        dests_ = list(st.get('dests') or [st['dref']])
        self.multi = len(dests_) > 1
        self.dest_of = {l_: (st['dst_pad'][lg[0]].component_ref if self.multi else st['dref']) for l_, lg in self.lanes}
        _sb = whole_frame.box_of(st['pcb'], st['sref'])
        self.group = whole_frame.Group(st['pcb'], dests_, ((_sb[0] + _sb[2]) / 2, (_sb[1] + _sb[3]) / 2)) \
            if self.multi else None
        self.prim = [l_ for l_, _lg in self.lanes if self.dest_of[l_] == st['dref']]
        self.lbox = {l_: (st['dboxes'] if self.dest_of[l_] == st['dref'] else st['dbox_of'][self.dest_of[l_]])
                     for l_, _lg in self.lanes}
        sb, db = st['sgrid'].bbox, st['dgrid'].bbox
        cs = ((sb[0] + sb[2]) / 2, (sb[1] + sb[3]) / 2)
        cd = ((db[0] + db[2]) / 2, (db[1] + db[3]) / 2)
        if cd[0] <= cs[0] or abs(cd[1] - cs[1]) > cd[0] - cs[0]:
            raise SystemExit('whole_ends: the bench is not in the canonical frame (source to destination along +x)')
        far_dir = min(FAR, key=lambda d_: FAR[d_][0] * (cd[0] - cs[0]) + FAR[d_][1] * (cd[1] - cs[1]))
        self.far_dir = far_dir
        # (a far-face tooth still inside the source's array -- a ball's via, its lane not out of the field -- has no way
        # round the source: the frame refuses it, whole_frame.FarFace)
        far_in = lambda m: m.direction == far_dir and m.exit_pt[0] >= sb[0] - 1e-6
        cur = {n: pf.current_tooth(st, n) for n in names}
        # ---- each lane's options at each end: (moves per leg, point, layer, vias)
        rt = 1.3 * max(st['sgrid'].pitch_x, st['sgrid'].pitch_y)
        rb = 1.3 * max(st['dgrid'].pitch_x, st['dgrid'].pitch_y)
        rb_of = {l_: (rb if self.dest_of[l_] == st['dref'] else
                      1.3 * max(st['dgrid_of'][self.dest_of[l_]].pitch_x, st['dgrid_of'][self.dest_of[l_]].pitch_y))
                 for l_, _lg in self.lanes}

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
        # (more than two: each routing layer's plane, its run priced as the solve prices it -- route_layers.plane_kinds;
        # every layer a plane's, none priced)
        kinds_ = route_layers.plane_kinds(st['pcb'], self.RL) if self.NL > 2 else {}
        self.PK = [kinds_.get(L_, '-') for L_ in self.RL]
        if '-' not in self.PK:
            self.PK = ['-'] * self.NL
        self.inner_signal = route_layers.inner_signal(dict(zip(self.RL, self.PK)))   # (route_layers.run_price)
        self._runs_ok = {}                              # (more than two) a via end's moves -> the layers its runs fit
        self._meet = {}                                 # (more than two) two ends' moves -> their runs within the rule
        self._nbrs = {}                                 # (more than two) an end option -> the options its runs meet
        self._rbox = {}                                 # ...an option's runs' box, grown by the rule
        self._dom = {}                                  # (more than two) an end option -> its layer domain
        vias_only = route_layers.escape_vias('src')
        for lane, lg in self.lanes:
            # (`free_teeth`: an incremental fanout frees only the teeth its feedback names; the rest stand as laid)
            held = not src_free or (free_teeth is not None and lane not in free_teeth)
            if held and all(cur[l_] is not None for l_ in lg):
                # the teeth as laid: the one option, refused on the source's far face inside the array (a far-face
                # tooth out of it is priced, FAR_TOOTH)
                # (ESCAPE_VIAS: a tooth laid as a surface escape refused, so a re-fan through a via is not judged
                # worse than it for the via it adds)
                self.T[lane] = [o[:4] + (o[4] or any(far_in(m) for m in o[0])
                                         or (vias_only and any(m.kind == 'surface' for m in o[0])),)
                                for o in combos([[cur[l_]] for l_ in lg], rt, forced=True)]
            else:
                # (ESCAPE_VIAS: a tooth laid as a surface escape is no option where its menu has a via's)
                tm = [[m for m in ([cur[l_]] if cur[l_] is not None and not (
                       vias_only and cur[l_].kind == 'surface' and st['smenu'].get(l_)) else [])
                       + [m for m in st['smenu'].get(l_, []) if not same_as_laid(m, cur[l_])]
                       if not far_in(m)] for l_ in lg]
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
            rb_ = rb_of[lane]
            self.B[lane] = (combos(dm, rb_, forced=forced) if forced else
                            unbanned(lg, combos(dm, rb_)) or combos(dm, rb_, forced=True))  # (nothing clean: refused)
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
        if self.group is not None:
            # (a GROUP's berth option once per way in, the way its sixth element: (way, its place on the walk, its
            # pocket's mouth or None, its length, its pocket) -- Group.options, the shortest GROUP_OPTIONS of them)
            for lane, lg in self.lanes:
                pads_ = [st['dst_pad'][l_] for l_ in lg]
                px_, py_ = sum(p_.global_x for p_ in pads_) / len(pads_), sum(p_.global_y for p_ in pads_) / len(pads_)
                self.B[lane] = [o + (w_,) for o in self.B[lane]
                                for w_ in self.group.options(o[1], (o[1][0] - px_, o[1][1] - py_), self.dest_of[lane],
                                                             o[2], o[3] > 0)]
        # a SINGLE lane's end priced by what stands in front of its exit on its layer (exit_front): a pair leg's menu
        # keeps only exits with room for the pair there (fanout_from_plan.pair_exit_clear), a single's was checked
        # only to its tooth's tip -- and the loop's audits then found it against a passive the solve cannot move it
        # off (zynq: C105 0.6 mm in front of the source teeth, C98 beside the berths -- most of the static findings)
        self.front, self.run_ids = {}, run_ids
        # (the tracks across the source's side bands, read on the board the bus's routed neighbours stand on -- the
        # shared field where the ends are chosen on the bus's own: escape_plan, `fence_pcb`)
        import whole_frame as _wf
        skip_ = {l_ for _lane, lg in self.lanes for l_ in lg}
        fpcb_ = st.get('fence_pcb') or st['pcb']
        self.fence = side_fences(fpcb_, _wf.box_of(st['pcb'], st['sref']), skip_)
        # (the LAID lanes of another bus among them -- locked copper: FENCE_LAID each one a lane passes, (north, south)
        # per tooth option, a far-face one's by the side of the cut it goes round; and a lane whose every other way out
        # passes one is not held to the far face's last resort, FAR_TOOTH)
        laid_ = copy.copy(fpcb_)
        laid_.segments = [s_ for s_ in fpcb_.segments if s_.locked]
        laid_.vias = [v_ for v_ in fpcb_.vias if v_.locked]
        lf_ = side_fences(laid_, _wf.box_of(st['pcb'], st['sref']), skip_)
        self.laid_x, self.far_w = {}, {}
        for lane, _lg in self.lanes:
            xs_ = {}
            for i, o in enumerate(self.T[lane]):
                d_ = o[0][0].direction if o[0] else None
                if d_ == far_dir:
                    xs_[i] = (len(lf_['N']), len(lf_['S']))
                elif d_ in ('up', 'down'):
                    n_ = sum(1 for x_ in lf_['N' if d_ == 'up' else 'S'] if x_ >= o[1][0])
                    xs_[i] = (n_, n_)
                else:
                    xs_[i] = (0, 0)
            self.laid_x[lane] = xs_
            other_ = [xs_[i] for i, o in enumerate(self.T[lane]) if not o[4] and not _far_opt(self, o)]
            self.far_w[lane] = 0.0 if other_ and all(min(c_) for c_ in other_) else FAR_TOOTH
        # (each tooth option's legs that leave by a side face of the array's other half from their ball: AWAY_FACE each)
        toward_dir = max(FAR, key=lambda d_: FAR[d_][0] * (cd[0] - cs[0]) + FAR[d_][1] * (cd[1] - cs[1]))
        self.away = {}
        for lane, lg in self.lanes:
            ax_ = {}
            for i, o in enumerate(self.T[lane]):
                n_ = 0
                for leg, m in zip(lg, o[0]):
                    d_ = m.direction
                    if d_ in (far_dir, toward_dir) or d_ not in FAR or leg not in st['src_pad']:
                        continue
                    p_ = st['src_pad'][leg]
                    k_ = 0 if FAR[d_][0] else 1
                    if ((p_.global_x, p_.global_y)[k_] - cs[k_]) * FAR[d_][k_] < -1e-6:
                        n_ += 1
                if n_:
                    ax_[i] = n_
            if ax_:
                self.away[lane] = ax_
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
        # (a laid tooth's STUB as laid -- its net's copper inside the source's box a pitch out -- for the ride, which
        # measures a menu move's stub along its legs: a tooth laid from the plan has no legs of its own)
        sg_ = st['sgrid']
        bx_ = (sg_.bbox[0] - sg_.pitch_x, sg_.bbox[1] - sg_.pitch_y, sg_.bbox[2] + sg_.pitch_x, sg_.bbox[3] + sg_.pitch_y)
        inb_ = lambda x, y: bx_[0] <= x <= bx_[2] and bx_[1] <= y <= bx_[3]  # noqa: E731
        by_net_ = collections.defaultdict(float)
        for sg in st['pcb'].segments:
            if inb_(sg.start_x, sg.start_y) and inb_(sg.end_x, sg.end_y):
                by_net_[sg.net_id] += math.hypot(sg.end_x - sg.start_x, sg.end_y - sg.start_y)
        self._laid_len = {leg: by_net_.get(st['byname'][leg][0], 0.0) for leg, m in cur.items()
                          if m is not None and not m.legs and leg in st['byname']}
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
        # (the escape fanout lays its moves exactly as they are: a via keeps another lane's run off by the rule, not by
        # the router's grid ring -- rules.fan_via_clear)
        import rules as _rules
        reach_ = _rules.fan_via_clear(_rules.active()) if st.get('escape') else None
        for cands, strict in (({n: list(kept[0][n].values()) for n in names}, True),
                              ({n: list(kept[1][n].values()) for n in names}, bool(pf.PAGES_STRICT))):
            # stacked: the whole route orders each layer's lanes on their own, so an F exit over a B one is
            # no conflict
            for (a, i, b, j) in pf._conflicts(cands, strict, stack=True, via_reach=reach_):
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
        # ...but two teeth AS LAID -- copper on the board, which the fanout's own check laid clean -- are not kept a via's
        # REACH apart: the reach asks whether a run can still be laid beside a via, and both are. Their exits, their
        # stacks and their lanes stay the rule. The engine lays a tooth a grid step off the move asked (`gap ok` in the
        # realize audit), and read at the reach the realized board's own teeth conflicted where the moves had not: the
        # zynq six-rail DDR3's DQS1_N dog-bone 0.375 from DQS1_P's via in pad (0.400 asked) -- every realized board
        # then judged worse than the one it came from by a conflict nobody could move, and reverted
        import select_moves as _sm
        laid_ = {(l_, id(m)): m for l_, m in cur.items() if m is not None}
        for k_, m_ in laid_.items():
            for o_ in [o_ for o_ in conf.get(k_, ()) if o_ in laid_]:
                if not _sm._conflict(m_, laid_[o_], strict=True, stack=True, reach=1e-6):
                    conf[k_].discard(o_)
                    conf[o_].discard(k_)
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
        # ...and a GROUP's lanes in one POCKET: in their order along it from its first mouth, the ones out of that
        # mouth, then the ones straight under, then the ones out of the other -- a lane out of a mouth past one that
        # goes another way crosses it in the pocket, where no place on the walk shows it (Group.units)
        self.pconf = collections.defaultdict(set)
        if self.group is not None:
            pk_ = collections.defaultdict(list)
            for lane, _lg in self.lanes:
                for j, o in enumerate(self.B[lane]):
                    if o[5][4] is not None:
                        pk_[o[5][4]].append((lane, j))
            for key_, opts_ in pk_.items():
                m1_ = key_[0]
                rank_ = lambda w_: 0 if (w_[0] == 'mouth' and w_[2] == m1_) else (1 if w_[0] == 'straight' else 2)
                dep_ = {(l_, j): _pt_seg(self.B[l_][j][1], m1_[0], m1_[1]) for l_, j in opts_}
                for (a, i) in opts_:
                    for (b, j) in opts_:
                        if a != b and dep_[(a, i)] < dep_[(b, j)] and \
                                rank_(self.B[a][i][5]) > rank_(self.B[b][j][5]):
                            self.pconf[(a, i)].add((b, j))
                            self.pconf[(b, j)].add((a, i))
                            self.oconf[(a, 1, i)].add((b, j))
                            self.oconf[(b, 1, j)].add((a, i))
            # ...and a way UNDER parts (straight under the part in front, or under its own) through another lane's
            # berth VIA: the straight from the berth to its place passes within a via's reach of it, and a via stands
            # on every layer. The berths' vias along a row part's face stand a pin pitch or two apart, a fence no lane
            # passes under the part through -- the ways under it priced as open on every layer its pads leave free,
            # eight lanes to a connector's pins went straight under a QFN across a pocket, through the QFN's own
            # berth vias, and the three the gaps had no room for were never laid
            if (awx_settings.get('ENDS_UNDER_VIAS') or '1') == '1':
                import numpy as np
                import rules as _rules_u
                R_ = _rules_u.fan_via_clear(_rules_u.active())
                sites_ = [(l_, j, m.site) for l_, _lg in self.lanes for j, o in enumerate(self.B[l_])
                          for m in o[0] if m.site is not None]
                if sites_:
                    SP_ = np.array([s_[2] for s_ in sites_], float)
                    n_u = 0
                    for a, _lg in self.lanes:
                        for i, o in enumerate(self.B[a]):
                            if o[5][0] not in ('straight', 'under'):
                                continue
                            p_, q_ = np.array(o[1], float), np.array(self.group.pt(o[5][1]), float)
                            d_ = q_ - p_
                            L2_ = float(d_ @ d_)
                            t_ = np.clip(((SP_ - p_) @ d_) / L2_, 0.0, 1.0) if L2_ > 1e-12 else np.zeros(len(SP_))
                            dist_ = np.hypot(*(SP_ - (p_ + t_[:, None] * d_)).T)
                            for k_ in np.nonzero(dist_ < R_)[0]:
                                b, j, _s = sites_[k_]
                                if b != a:
                                    self.oconf[(a, 1, i)].add((b, j))
                                    self.oconf[(b, 1, j)].add((a, i))
                                    n_u += 1
                    self.under_vias = n_u
        self._ride = {}
        # ...and the RING's standoff round the destination's ball box, for a WOUND lane's ride (wound_ride): its berths'
        # exits stand outside the box by their stubs (the median, whole_frame.grown's measure), and the ring a lane pitch
        # past them (whole_frame: braid.ring_spine round the hull of the pads and the stubs). Priced at the box's own 0.3
        # mm, as around_box prices the short way, a lane wound round three faces came 5 to 12 mm longer laid (s4_bulgeW)
        bx_ = st['dboxes']
        out_ = sorted(v for lane in self.prim for o in self.B[lane]
                      if (v := max(bx_[0] - o[1][0], o[1][0] - bx_[2], bx_[1] - o[1][1], o[1][1] - bx_[3])) > 0)
        self.ring_d0 = (out_[len(out_) // 2] if out_ else 0.0) + _bd.LPITCH
        # ---- the whole route's FEEDBACK (whole_feedback: ends its audits found crowded, FEEDBACK= to the fanout): an
        # end to avoid costs FB_AVOID vias when chosen, a pair of ends FB_PAIR when both are -- each option weighed by
        # its lane, its layer and ANY leg's exit's distance to the end's points as laid (fb_weights), so a pair cannot
        # leave the price by moving one leg and keeping the other where it was found crowded; two items naming one
        # option add
        fb = st.get('feedback') or {}
        self.fb_avoid, self.fb_pairs = fb_tables(fb, self.T, self.B)
        # ...and the lanes the whole solve could not keep to two vias on an earlier round's ends ('over': whole_route,
        # an unproved plan's nets over two): each counted over by at least that much on every option, so the ends no
        # longer pay vias for a promise the solve could not keep (K51: ends at 92 vias promising SCK at two, chosen over
        # ends at 78 with SCK over; the solve held SCK over and 51 route vias, proved nothing, and laid nothing)
        self.fb_over = {ln: int(k) for ln, k in (fb.get('over') or {}).items() if ln in self.legs_of and int(k) > 0}
        self._xroute = {}          # the exact route per state (exact_route)
        self.x_learned = {}        # {frozenset of two lanes: vias}: crossings the whole solve could not place (checked)
        self.opp_learned = {}      # {lane: vias}: a pair's opposite hands the solve could not cross over (checked, rules)
        self.end_learned = {}      # {lane: vias}: a lane's fixed end layers the solve could not hold (checked, rules)
        self.wcut = None           # the destination's cut when WINDING moved it off the far face (choose: cut_phase)
        self.pool = {}             # every search's result: its state -> its objective by the estimate
        self.pool_all = set()      # ...and every search's result whatever cut it was searched at (ENDS_PORTFOLIO)
        self._memo = {}            # every state scored: its objective -- the search asks a third of them again
        self._parts = collections.OrderedDict()     # the last _PARTS_KEEP states' parts (score), the oldest dropped

    @staticmethod
    def _stub_len(m, pad, laid=None):
        """a stub's length from its pad to its exit: along its LEGS (a climb's run along the array before it leaves),
        never less than the straight; a laid tooth's (no legs) its copper as laid, `laid`. The straight alone priced a
        climb 11 mm long out of the array's far face as 8.4"""
        straight = math.hypot(m.exit_pt[0] - pad.global_x, m.exit_pt[1] - pad.global_y)
        if m.legs:
            return max(straight, sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b, _L in m.legs))
        return max(straight, laid or 0.0)

    def ride(self, lane, npair, ti, bi, side=None):
        """a lane's ride in mm, each leg's: round the destination and the source from its tooth to its berth
        (select_moves.ride_mm's own measure), and its STUBS -- each leg's straight run from its ball to its tooth's
        and its berth's exit (a laid tooth has no legs of its own to measure; the straight run measures every option
        alike). Without them a tooth that ran 5 mm inside the source's balls to leave by another face cost nothing
        (K15 SDQS1, via-in-pad on B from the east rows out through the north face). `side`: a WOUND lane's ('N' or
        'S', as the kernel reads it off a winding cut) -- round the destination the long way, that side (wound_ride)"""
        k = (lane, ti, bi, side)
        if k not in self._ride:
            import plan_ends as pe
            a, b = self.T[lane][ti][1], self.B[lane][bi][1]
            way_ = 0.0
            G = self.group
            ring = None
            if G is not None and self.B[lane][bi][5][0] != 'on':
                # (a GROUP lane entering by another way than its berth's own place: to where it enters, then that way)
                w_ = self.B[lane][bi][5]
                b, way_ = G.pt(w_[1]), w_[3]
            elif G is not None and side is None and not G.in_front(self.B[lane][bi][5][1]):
                # (a GROUP berth OFF THE FRONT, on its own place: its lane goes round the group's outline to it from the
                # front's nearer end -- the frame's ring -- where the straight from its tooth ran through the parts. A
                # back-row pin's lane round a connector's far end was priced as the straight under it, and the ring cost
                # the ends nothing beside the way straight under the connector)
                s_ = self.B[lane][bi][5][1]
                ring = min(math.hypot(e_[0] - a[0], e_[1] - a[1]) + w for w, e_ in
                           (((s_ - G.f1) % G.L, G.pt(G.f1)), ((G.f0 - s_) % G.L, G.pt(G.f0))))
                q_ = G.pt(s_)
                ring += math.hypot(b[0] - q_[0], b[1] - q_[1])
            d = (ring if ring is not None else pe.sm.around_box(a, b, self.lbox[lane])) if side is None else \
                wound_ride(a, b, self.st['dboxes'], side, pad=self.ring_d0)
            d += way_
            d += pe.sm.around_box(a, b, self.st['sgrid'].bbox) - math.hypot(b[0] - a[0], b[1] - a[1])
            lg = dict(self.lanes)[lane]
            stubs = 0.0
            for leg, tm, bm in zip(lg, self.T[lane][ti][0], self.B[lane][bi][0]):
                sp_, dp_ = self.st['src_pad'][leg], self.st['dst_pad'][leg]
                stubs += self._stub_len(tm, sp_, self._laid_len.get(leg))
                stubs += self._stub_len(bm, dp_, None)
            self._ride[k] = npair * d + stubs
        return self._ride[k]


    def arrange_ways(self, state, prefer=None):
        """a GROUP's state with each lane's WAY into the group chosen again, its tooth and berth held: the ways and the
        cut that cross the fewest against the teeth's order (Group.arrange, the frame's own). The berths read back as
        laid hold each lane to its berth's ways, and a start took each its shortest -- 542 crossings where the search's
        own ways on the same ends made 451. `prefer` {lane: its way's point} (the search's own, its leg prices in
        them): each lane it names takes its berth's way nearest that point, and only the others are arranged"""
        if self.group is None:
            return state
        import whole_frame
        lanes = [l_ for l_, _lg in self.lanes]
        SB = whole_frame.grown(self.sbox, [self.T[l_][state[l_][0]][1] for l_ in lanes])
        lkey = {l_: (whole_frame.perim_s(self.T[l_][state[l_][0]][1], SB, None), i) for i, l_ in enumerate(lanes)}
        idx, opts = {}, {}
        for l_ in lanes:
            b0 = self.B[l_][state[l_][1]]
            idx[l_] = [j for j, o in enumerate(self.B[l_]) if o[0] == b0[0] and o[1] == b0[1]]
            opts[l_] = [self.B[l_][j][5] for j in idx[l_]]
        start = None
        if prefer:
            start = {l_: min(range(len(opts[l_])), key=lambda i, l_=l_: math.dist(self.group.pt(opts[l_][i][1]),
                                                                                    prefer[l_]))
                     for l_ in lanes if l_ in prefer}
            if len(start) == len(lanes):
                return {l_: (state[l_][0], idx[l_][start[l_]]) for l_ in lanes}
        ch, _c, _x = self.group.arrange(opts, lkey)
        if start:
            ch.update(start)
        return {l_: (state[l_][0], idx[l_][ch[l_]]) for l_ in lanes}


    def _entry_out(self, bo):
        """{lane: (x, y)} -- each GROUP lane's way in, as the plan sidecar carries it: its place's point on the walk (the
        frame takes the way nearest it); None off a group"""
        if self.group is None:
            return None
        return {l_: tuple(float(v) for v in self.group.pt(o[5][1])) for l_, o in bo.items()}


    def tooth_price(self, lane, i):
        """a tooth option's own price past its vias, as the joint models read it: the far face's (far_w), the laid
        lanes of another bus it passes (FENCE_LAID each, a far-face option's by the side that passes fewer -- the
        kernel's by the side of the cut) and its legs out of the array's other half (AWAY_FACE each)"""
        o = self.T[lane][i]
        return (self.far_w[lane] if _far_opt(self, o) else 0.0) + \
            FENCE_LAID * len(self.legs_of[lane]) * min(self.laid_x[lane][i]) + \
            AWAY_FACE * (self.away.get(lane) or {}).get(i, 0)

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
        v, p = self._score_k(state, exact)
        self._memo[key] = v
        self._keep_parts(key, p)
        return v, p

    def objective(self, state, exact=False):
        """a state's objective alone (score's first): each state scored once (the local search asks a third of its
        states again -- an ejection's winner is scored in the min and then again, a sweep revisits the kicks' states)"""
        key = (tuple(state[l_] for l_, _lg in self.lanes), exact, self.wcut)
        v = self._memo.get(key)
        if v is None:
            if exact:
                v = self.score(state, exact)[0]
            else:
                v = self._kernel().objective(self, state)
            self._memo[key] = v
        return v

    def _kernel(self):
        """the estimate's Rust kernel of this model (ends_kernel), built on its first question and shared by its copies
        (cut_phase's)"""
        kh = self.__dict__.get('_kh')
        if kh is None:
            kh = self._kh = _ek.build(self)
        return kh

    def _score_k(self, state, exact=False):
        """(objective, parts) of a state, scored by the kernel (ends_kernel) -- the route EXACT on its orders, where
        asked, CP-SAT here, called back from the kernel at each cut it scores -- its parts named for the whole route"""
        kh = self._kernel()
        v, raw = kh.parts(self, state, self._exact_cb(state, kh) if exact else None)
        lanes = [l_ for l_, _lg in self.lanes]
        bo = {l_: self.B[l_][state[l_][1]] for l_ in lanes}
        p = dict(raw)
        p['chg'] = {lanes[i]: c_ for i, c_ in enumerate(raw['chg'])}
        p['lane_over'] = {lanes[i]: k_ for i, k_ in enumerate(raw['lane_over']) if k_}
        p['lane_load'] = {lanes[i]: round(v_, 3) for i, v_ in enumerate(raw['lane_load'])}
        p['lane_x'] = {lanes[i]: n_ for i, n_ in enumerate(raw['lane_x']) if n_}
        p['cong'], p['ride'] = round(raw['cong'], 2), round(raw['ride'], 1)
        p['plane'], p['learned'] = round(raw['plane'], 2), round(raw['learned'], 2)
        p['front'] = round(raw['front'], 2)
        code, c_ = raw['cut']
        p['cut'] = (c_ if code == 0 else (('N', 'S')[code - 1], c_) if code in (1, 2)
                    else ('P',) + tuple(float(v_) for v_ in self.group.pt(c_)))
        p['entry'] = self._entry_out(bo)
        p['walled'] = [self.pw.balls[i][0] for i in raw['walled']]
        p['conf_lanes'] = sorted(lanes[i] for i in raw['conf_lanes']) if raw['conflicts'] else []
        # (each chosen end blocked with room for a via before the block, where an earlier round's audits found it
        # crowded: that block's span along its escape, for the solve to hold the lane on the other layer across it --
        # fanout_from_plan writes it, whole_solve reads it. An end nobody found crowded goes round the block on its
        # own layer, as a lane goes round a part: a change planned there cost two vias where none were needed)
        lane_front = collections.defaultdict(dict)
        for (l_, k_), c2_ in self.front.items():
            if (l_, k_) not in self.fb_avoid:
                continue
            i_ = state[l_][k_]
            if c2_.get(i_) == FRONT_VIA:
                o_ = (self.T if k_ == 0 else self.B)[l_][i_]
                sp_ = front_span(self.st['pcb'], self.st['byname'][self.legs_of[l_][0]][0], o_[0][0], self.run_ids)
                if sp_ is not None:
                    lane_front[l_][k_] = sp_
        p['lane_front'] = dict(lane_front)
        return v, p

    def _exact_cb(self, state, kh):
        """the exact route's callback the kernel calls at each cut it scores: its orders and ends by lane index in,
        (route, over, changes, drops, plane price) by lane index out -- exact_route_nl on more routing layers than two,
        exact_route on two; None where it finds no plan"""
        lanes = [l_ for l_, _lg in self.lanes]
        npair = {l_: len(lg) for l_, lg in self.lanes}
        to = {l_: self.T[l_][state[l_][0]] for l_ in lanes}
        bo = {l_: self.B[l_][state[l_][1]] for l_ in lanes}

        def cb(kp, kd, ends, sv, xo, sep, tl, dl):
            kp_ = {lanes[i]: (v_, i) for i, (v_, _i) in enumerate(kp)}
            kd_ = {lanes[i]: (v_, i) for i, (v_, _i) in enumerate(kd)}
            sv_ = {lanes[i]: v_ for i, v_ in enumerate(sv)}
            xo_ = {lanes[i]: v_ for i, v_ in enumerate(xo)}
            if self.NL > 2:
                def dom(e):
                    f_, n_, ok_ = e
                    if f_ >= 0:
                        return (f_, None, None)
                    return (None, None if n_ < 0 else n_, frozenset(q for q in range(self.NL) if ok_ >> q & 1))
                ends_ = {lanes[i]: (dom(a_), dom(b_)) for i, (a_, b_) in enumerate(ends)}
                sep_ = [((lanes[a_], ka_), (lanes[b_], kb_)) for (a_, ka_), (b_, kb_) in sep]
                xr = self.exact_route_nl(state, lanes, kp_, kd_, ends_, sv_, xo_, npair, sep_)
                if xr is None:
                    return None
                route, over, chg, drx, plane = xr
                return (route, over, [chg[l_] for l_ in lanes], [drx[l_] for l_ in lanes], float(plane))
            tl_ = {lanes[i]: kh.lid_name[v_] for i, v_ in enumerate(tl)}
            dl_ = {lanes[i]: kh.lid_name[v_] for i, v_ in enumerate(dl)}
            xr = self.exact_route(state, lanes, kp_, kd_, tl_, dl_, sv_, xo_, npair)
            if xr is None:
                return None
            route, over, chg = xr
            return (route, over, [chg[l_] for l_ in lanes], [], 0.0)
        return cb

    def _keep_parts(self, key, p):
        self._parts[key] = p
        self._parts.move_to_end(key)
        if len(self._parts) > _PARTS_KEEP:
            self._parts.popitem(last=False)


    # ---- more routing layers than two
    def end_domain(self, opt, k_, lg):
        """(fixed, neck, allowed) of an end option `opt` (k_ 0 a tooth, 1 a berth; `lg` its lane's legs), as routing-
        layer indices: a SURFACE escape is FIXED on its layer (neck and allowed None); a VIA end -- every leg a dog-bone
        or a via in its pad -- is free (fixed None) on the layers its runs can lie on (`allowed`, via_run_layers), its
        via joining nothing, and dropped, on its NECK's own layer (the ball's pad's; None where a leg's pad is not on one
        routing layer)"""
        got = self._dom.get((k_, id(opt)))
        if got is not None and got[0] is opt:
            return got[1]
        got = self._end_domain(opt, k_, lg)
        self._dom[(k_, id(opt))] = (opt, got)       # (an option is its lane's for good: T and B stand as built)
        return got

    def _end_domain(self, opt, k_, lg):
        moves = opt[0]
        if not moves or any(getattr(m, 'kind', 'surface') == 'surface' for m in moves):
            return (self.RL.index(opt[2]) if opt[2] in self.RL else 0), None, None
        necks = set()
        for leg in lg:
            pad = self.st['src_pad' if k_ == 0 else 'dst_pad'].get(leg)
            cu = {L for L in (getattr(pad, 'layers', None) or ()) if L in self.RL}
            necks.add(next(iter(cu)) if len(cu) == 1 else None)
        neck = next(iter(necks)) if len(necks) == 1 else None
        ok = self.via_run_layers(moves, lg)
        return None, (self.RL.index(neck) if neck is not None else None), ok

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


    def _run_box(self, k_, l_, i_):
        """the box of option i_'s runs (lane l_'s, k_ 0 a tooth, 1 a berth: each move's legs on its own layer), grown
        by the fanout's rule; None with no run"""
        key = (k_, l_, i_)
        if key not in self._rbox:
            import rules as _rules
            r = _rules.active()
            bar = r.fan_track + r.fan_clear
            pts = [p for m in (self.T if k_ == 0 else self.B)[l_][i_][0]
                   for a, b, L_ in (getattr(m, 'legs', None) or ()) if L_ == m.layer for p in (a, b)]
            self._rbox[key] = (min(p[0] for p in pts) - bar, min(p[1] for p in pts) - bar,
                               max(p[0] for p in pts) + bar, max(p[1] for p in pts) + bar) if pts else None
        return self._rbox[key]

    def _meet_nbrs(self, l_, k_, i_):
        """{lane: frozenset of option indices}: the other lanes' options at the same array (k_ 0 the teeth, 1 the
        berths) whose runs come within the fanout's rule of option i_ of lane l_'s -- _runs_meet(.., None), each pair
        tested once; a lane none of whose options meet it is not a key"""
        key = (l_, k_, i_)
        got = self._nbrs.get(key)
        if got is not None:
            return got
        opts = self.T if k_ == 0 else self.B
        ba = self._run_box(k_, l_, i_)
        got = {}
        if ba is not None:
            for l2, _lg2 in self.lanes:
                if l2 == l_:
                    continue
                hit = []
                for i2 in range(len(opts[l2])):
                    bb = self._run_box(k_, l2, i2)
                    if bb is None or bb[0] > ba[2] or ba[0] > bb[2] or bb[1] > ba[3] or ba[1] > bb[3]:
                        continue
                    if self._runs_hit(opts[l_][i_][0], opts[l2][i2][0], None):
                        hit.append(i2)
                if hit:
                    got[l2] = frozenset(hit)
        self._nbrs[key] = got
        return got

    def meet_all(self, k_):
        """_meet_nbrs for EVERY option at array k_ (0 the teeth, 1 the berths) at once, into its cache: the same runs,
        the same boxes and the same distance arithmetic (_seg_d's, as numpy arrays) -- the joint ends ask every option's
        neighbours, where the local search asks a few at a time: in Python that was 23 of a joint build's 24 s (zynq
        LVDS K32, 6.9 M segment pairs)"""
        import numpy as np
        import rules as _rules
        r = _rules.active()
        bar = r.fan_track + r.fan_clear - 0.01          # (_runs_hit's)
        opts = self.T if k_ == 0 else self.B
        keys, lane_of, boxes, sa, sb, own = [], [], [], [], [], []
        for li, (l_, _lg) in enumerate(self.lanes):
            for i_ in range(len(opts[l_])):
                q = len(keys)
                keys.append((l_, i_))
                lane_of.append(li)
                bx = self._run_box(k_, l_, i_)
                boxes.append(bx if bx is not None else (np.inf, np.inf, -np.inf, -np.inf))
                for m in opts[l_][i_][0]:
                    for a, b, L_ in (getattr(m, 'legs', None) or ()):
                        if L_ == m.layer:
                            sa.append(a); sb.append(b); own.append(q)
        if not keys:
            return
        lane_of, boxes = np.array(lane_of), np.array(boxes, dtype=float)
        A, B, own = np.array(sa, dtype=float).reshape(-1, 2), np.array(sb, dtype=float).reshape(-1, 2), np.array(own)
        first = np.searchsorted(own, np.arange(len(keys) + 1))     # (each option's runs, contiguous)

        def pt(u, v, w):
            # (_seg_d's pt, elementwise: u a point, vw a segment)
            dx, dy = w[..., 0] - v[..., 0], w[..., 1] - v[..., 1]
            L = dx * dx + dy * dy
            with np.errstate(divide='ignore', invalid='ignore'):
                t = np.where(L == 0, 0.0, np.maximum(0.0, np.minimum(1.0, ((u[..., 0] - v[..., 0]) * dx
                                                                           + (u[..., 1] - v[..., 1]) * dy) / L)))
            return np.hypot(v[..., 0] + t * dx - u[..., 0], v[..., 1] + t * dy - u[..., 1])
        o = lambda u, v, w: (v[..., 0] - u[..., 0]) * (w[..., 1] - u[..., 1]) - (v[..., 1] - u[..., 1]) * (w[..., 0] - u[..., 0])
        for q, (l_, i_) in enumerate(keys):
            if (l_, k_, i_) in self._nbrs:
                continue
            got = {}
            if first[q] < first[q + 1]:
                ba = boxes[q]
                near = (lane_of != lane_of[q]) & ~((boxes[:, 0] > ba[2]) | (ba[0] > boxes[:, 2])
                                                   | (boxes[:, 1] > ba[3]) | (ba[1] > boxes[:, 3]))
                sel = np.flatnonzero(near[own])
                if sel.size:
                    a, b = A[first[q]:first[q + 1], None, :], B[first[q]:first[q + 1], None, :]
                    p, qq = A[None, sel, :], B[None, sel, :]
                    crs = (o(a, b, p) * o(a, b, qq) < 0) & (o(p, qq, a) * o(p, qq, b) < 0)
                    d = np.minimum(np.minimum(pt(a, p, qq), pt(b, p, qq)), np.minimum(pt(p, a, b), pt(qq, a, b)))
                    hit = (crs | (d < bar)).any(axis=0)
                    hq = sorted(set(own[sel[hit]].tolist()))
                    by = collections.defaultdict(list)
                    for h in hq:
                        by[keys[h][0]].append(keys[h][1])
                    got = {l2: frozenset(by[l2]) for l2, _lg2 in self.lanes if l2 in by}
            self._nbrs[(l_, k_, i_)] = got

    def _runs_meet(self, ma, mb, layer):
        """whether via end `ma`'s runs (each move's legs on its own layer) come within the fanout's rule (its track,
        its clearance) of `mb`'s -- a fixed end's legs on `layer`, or (None) a via end's runs"""
        key = (tuple(id(m) for m in ma), tuple(id(m) for m in mb), layer)
        got = self._meet.get(key)
        if got is not None:
            return got[2]
        hit = self._runs_hit(ma, mb, layer)
        self._meet[key] = (tuple(ma), tuple(mb), hit)
        return hit

    @staticmethod
    def _runs_hit(ma, mb, layer):
        """_runs_meet's test itself, uncached"""
        import rules as _rules
        r = _rules.active()
        bar = r.fan_track + r.fan_clear - 0.01          # (relayer.clashes' SLACK: the fanout's grid a hair under)
        ra = [(a, b) for m in ma for a, b, L_ in (getattr(m, 'legs', None) or ()) if L_ == m.layer]
        rb = [(a, b) for m in mb for a, b, L_ in (getattr(m, 'legs', None) or ())
              if L_ == (layer if layer is not None else m.layer)]
        return any(_seg_d(a, b, p, q) < bar for a, b in ra for p, q in rb)


    def exact_route_nl(self, state, lanes, kp, kd, ends, sv, xo, npair, sep=()):
        """(route vias, nets over two, changes per lane, vias dropped per lane, plane price) of a state EXACT on its
        orders on MORE ROUTING LAYERS THAN TWO -- the whole solve's N-layer model without its lengths: every inverted
        pair crossing once in an order (_crossing_orders), each lane's runs between its changes on one layer each (y, a
        bool a layer), two crossing lanes on different layers there, a fixed end's run on its layer and a via end's on
        any (its via dropped on its neck's), an opposite-hands pair a change at least, two via ends whose runs meet
        (`sep`, the kernel's) on two layers -- CP-SAT on the solve's objective (nets over two first, then vias, and each
        run on a plane's layer its price, route_layers.run_price). All of it bools: the layers as integers (an element of y at the count of changes
        before each crossing) took 99 s where these take 29. One worker, a deterministic work limit; None when it finds
        no plan in that work"""
        import route_layers
        from ortools.sat.python import cp_model
        idx = {l_: i for i, l_ in enumerate(lanes)}
        rkd = {l_: i for i, l_ in enumerate(sorted(lanes, key=lambda l_: kd[l_]))}
        rkp = {l_: i for i, l_ in enumerate(sorted(lanes, key=lambda l_: kp[l_]))}
        Ln = sorted(lanes, key=lambda l_: (round(kp[l_][0], 3), rkd[l_], idx[l_]))
        Fn = sorted(lanes, key=lambda l_: (round(kd[l_][0], 3), rkp[l_], idx[l_]))
        # (kept by what the model is built from, never by the state alone: _score scores one state at each far-face cut
        # that ties, each cut with its own destination order, crossings and estimate. Kept by the state and the winding
        # cut, a tied cut's route was the first cut's answer -- on zynq_ad9364's DDR fanout, all 38 times one was met)
        # (each lane's run on each plane's layer, in tenths of a via)
        in_ = self.inner_signal
        pc10 = {l_: [int(round(10 * route_layers.run_price(self.PK[q], in_))) for q in range(self.NL)] for l_ in lanes}
        key = ('nl', self.NL, tuple(lanes), tuple(Ln), tuple(Fn),
               tuple((l_, tuple((f_, n_, None if ok_ is None else tuple(sorted(ok_))) for f_, n_, ok_ in ends[l_]),
                      sv[l_], xo.get(l_), npair[l_], tuple(pc10[l_])) for l_ in lanes), tuple(sep))
        if key in self._xroute:
            return self._xroute[key]
        li = {l_: i for i, l_ in enumerate(Ln)}
        fi = {l_: i for i, l_ in enumerate(Fn)}
        X = [(a, b) for a, b in itertools.combinations(Ln, 2) if (li[a] < li[b]) == (fi[a] > fi[b])]
        m = cp_model.CpModel()
        before, acts = _crossing_orders(m, lanes, Ln, X)
        ys, drops, cost, pruns = {}, {}, [], []
        for l_ in lanes:
            act = acts[l_]
            y = [[m.NewBoolVar('') for _ in range(self.NL)] for _ in range(EXACT_KMAX + 1)]
            for k in range(EXACT_KMAX + 1):
                m.AddExactlyOne(y[k])           # (y[k][q]: the run after change k on layer q)
            for k in range(EXACT_KMAX):
                for q in range(self.NL):        # a change to another layer; none, the same one
                    m.AddBoolOr([act[k], y[k][q].Not(), y[k + 1][q]])
                    m.AddBoolOr([act[k], y[k][q], y[k + 1][q].Not()])
                    m.AddBoolOr([act[k].Not(), y[k][q].Not(), y[k + 1][q].Not()])
            dr_ = []
            for y_e, (fx, nk, ok_) in ((y[0], ends[l_][0]), (y[EXACT_KMAX], ends[l_][1])):
                if fx is not None:
                    m.Add(y_e[fx] == 1)
                    continue
                for q in range(self.NL):
                    if q not in ok_:
                        m.Add(y_e[q] == 0)      # (a layer its runs cannot lie on)
                if nk is not None and nk in ok_:
                    dr_.append(y_e[nk])
            if xo.get(l_):
                m.Add(act[0] == 1)              # an opposite-hands pair crosses over at a dive
            ys[l_], drops[l_] = y, dr_
            n_ = sum(act)
            ov = m.NewIntVar(0, EXACT_KMAX + 8, '')
            m.Add(ov >= sv[l_] + n_ - sum(dr_) - VIA_PREF)
            cost.append(10 * (int(W_OVER) * ov + npair[l_] * (n_ - sum(dr_))))
            for k in range(EXACT_KMAX + 1):
                for q in range(self.NL):
                    if pc10[l_][q]:
                        # (run k laid on q: k is 0, or its change is active)
                        z_ = m.NewBoolVar('')
                        m.AddBoolOr([y[k][q].Not(), z_] + ([act[k - 1].Not()] if k else []))
                        cost.append(npair[l_] * pc10[l_][q] * z_)
                        pruns.append((l_, k, q, z_))
        for (la_, ka_), (lb_, kb_) in sep:
            if la_ in ys and lb_ in ys:
                for q in range(self.NL):
                    m.AddBoolOr([ys[la_][0 if ka_ == 0 else EXACT_KMAX][q].Not(),
                                 ys[lb_][0 if kb_ == 0 else EXACT_KMAX][q].Not()])
        lay = {}
        for (l_, key_), bits in before.items():
            on = [m.NewBoolVar('') for _ in range(self.NL)]     # (on[q]: the lane on q at the crossing)
            for k in range(EXACT_KMAX + 1):
                at = ([bits[k - 1].Not()] if k else []) + ([bits[k]] if k < EXACT_KMAX else [])
                for q in range(self.NL):
                    m.AddBoolOr(at + [ys[l_][k][q].Not(), on[q]])
            lay[(l_, key_)] = on
        for key_ in X:
            for q in range(self.NL):
                m.AddBoolOr([lay[(key_[0], key_)][q].Not(), lay[(key_[1], key_)][q].Not()])
        _clique_bound(m, acts, lanes, X, self.NL)
        m.Minimize(sum(cost))
        _r, vals = _exact_solve(m)
        out = None
        if vals is not None:
            chg = {l_: sum(vals[a.Index()] for a in acts[l_]) for l_ in lanes}
            drx = {l_: sum(vals[d_.Index()] for d_ in drops[l_]) for l_ in lanes}
            out = (sum(npair[l_] * chg[l_] for l_ in lanes),
                   sum(max(0, sv[l_] + chg[l_] - drx[l_] - VIA_PREF) for l_ in lanes), chg, drx,
                   sum(npair[l_] * pc10[l_][q] * vals[z_.Index()] for l_, _k, q, z_ in pruns) / 10)
        self._xroute[key] = out
        return out

    def exact_route(self, state, lanes, kp, kd, tl, dl, sv, xo, npair):
        """(route vias, nets over two, changes per lane) of a state EXACT on its orders: the whole solve's order model
        without its lengths -- every inverted pair crossing once in an order (_crossing_orders), each lane's
        changes between its crossings, two crossing lanes on different layers there, an opposite-hands pair crossed
        over at a dive -- CP-SAT on the solve's own objective (nets over two vias first, then vias, a pair's change two):
        its optimum, or the best plan it finds within its work limit. Two lanes at one place round a box (an F end stacked over a B one) are taken in the order
        that does not cross them there. One worker and a deterministic work limit: the same answer on every machine;
        None when it finds no plan in that work (the estimate stands)"""
        from ortools.sat.python import cp_model
        idx = {l_: i for i, l_ in enumerate(lanes)}
        rkd = {l_: i for i, l_ in enumerate(sorted(lanes, key=lambda l_: kd[l_]))}
        rkp = {l_: i for i, l_ in enumerate(sorted(lanes, key=lambda l_: kp[l_]))}
        Ln = sorted(lanes, key=lambda l_: (round(kp[l_][0], 3), rkd[l_], idx[l_]))
        Fn = sorted(lanes, key=lambda l_: (round(kd[l_][0], 3), rkp[l_], idx[l_]))
        # (kept by what the model is built from, as exact_route_nl's: a far-face cut that ties has orders of its own)
        key = ('2l', tuple(lanes), tuple(Ln), tuple(Fn),
               tuple((l_, tl[l_], dl[l_], sv[l_], xo.get(l_), npair[l_]) for l_ in lanes))
        if key in self._xroute:
            return self._xroute[key]
        li = {l_: i for i, l_ in enumerate(Ln)}
        fi = {l_: i for i, l_ in enumerate(Fn)}
        X = [(a, b) for a, b in itertools.combinations(Ln, 2) if (li[a] < li[b]) == (fi[a] > fi[b])]
        m = cp_model.CpModel()
        before, act_of = _crossing_orders(m, lanes, Ln, X)
        F_ = lambda L: int(L == 'B.Cu')
        cost = []
        for l_ in lanes:
            act = act_of[l_]
            m.AddBoolXOr(act + ([m.NewConstant(1)] if tl[l_] == dl[l_] else []))
            if xo.get(l_):
                m.Add(act[0] == 1)              # an opposite-hands pair crosses over at a dive
            n_ = sum(act)
            ov = m.NewIntVar(0, EXACT_KMAX + 8, '')
            m.Add(ov >= sv[l_] + n_ - VIA_PREF)
            cost.append(int(W_OVER) * ov + npair[l_] * n_)
        for key_ in X:
            a, b = key_
            lits = before[(a, key_)] + before[(b, key_)]
            if F_(tl[a]) ^ F_(tl[b]):
                lits = lits + [m.NewConstant(1)]
            m.AddBoolXOr(lits)
        _clique_bound(m, act_of, lanes, X, 2)
        m.Minimize(sum(cost))
        _r, vals = _exact_solve(m)
        out = None
        if vals is not None:
            chg = {l_: sum(vals[a.Index()] for a in act_of[l_]) for l_ in lanes}
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
        # (the candidates' exact routes AT ONCE on the ends model's pool, the calling thread among them -- each on a copy
        # of the model sharing only the caches written once per key, as cut_phase's cuts: one CP-SAT worker each, at its
        # deterministic work limit, so the same answers in the time of the slowest. A candidate no worker has started
        # when its turn comes is routed here: no thread waits on a task nobody runs, and best_exact runs on the pool
        # itself, inside the cuts' searches. One after another they were a third of the zynq DDR3 bus's fanout)
        def one(s_):
            Ec = copy.copy(self)
            Ec._memo, Ec._parts = {}, collections.OrderedDict()
            return Ec.score(s_, exact=True)
        key_ = lambda s_: (tuple(s_[l_] for l_, _lg in self.lanes), True, self.wcut)
        futs = {i: _pool().submit(one, s_) for i, s_ in enumerate(cands) if i and key_(s_) not in self._memo}
        got = {}
        if cands:
            got[0] = self.score(cands[0], exact=True)
        for i, f_ in futs.items():
            got[i] = one(cands[i]) if f_.cancel() else f_.result()
            self._memo[key_(cands[i])] = got[i][0]
            self._keep_parts(key_(cands[i]), got[i][1])
        best = None
        for i, s_ in enumerate(cands):
            v_, p_ = got[i] if i in got else self.score(s_, exact=True)
            # (a state the exact route found no plan for ranks after every one it did: its objective is the estimate's,
            # the optimistic one, for the ends the solve will find hardest)
            k_ = (bool(p_.get('exact_failed')), v_)
            if best is None or k_ < (bool(best[1][1].get('exact_failed')), best[1][0] - 1e-9):
                best = (s_, (v_, p_))
        if best[0] != dict(state):
            log(f'  whole ends: by the exact route, another of the search\'s best: {_fmt(best[1][1])}')
        return best


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
        tabu = getattr(self, 'tabu', None) or set()     # (lane, end, option) a ban kick forbids (ban_kicks)
        # (the loop in the kernel, ends_kernel: each sweep's line said with its state's parts)
        state, best, sweeps_ = self._kernel().search(self, state, sweeps, tabu)
        for sw, (b_, s_) in enumerate(sweeps_):
            log(f'  whole ends: sweep {sw}: objective {b_:.2f} {self.score(s_)[1]}')
        self.pool[tuple(sorted(state.items()))] = best
        self.pool_all.add(tuple(sorted(state.items())))
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

    def kind_kicks(self, state, log=print):
        """the escapes of ONE PART's lanes at one end switched in KIND at once: every lane to the part with a via there,
        or every one without -- the other kind banned, each lane on its cheapest option of the kind, searched with the
        ban and again without it, kept when better. A part's lanes share their style -- a human dog-bones every pin of a
        QFN or none -- and one lane alone with the other kind pays for the change and buys nothing: on a group bus the
        one-lane search kept a QFN's lanes on surface berths, where with every berth a via its objective was 5.4
        lower (the exact route's vias one fewer)"""
        best, (bv, _bp) = dict(state), self.score(state)
        lanes = [l_ for l_, _lg in self.lanes]
        parts = sorted({self.dest_of[l_] for l_ in lanes}) if getattr(self, 'dest_of', None) else []
        for part in parts:
            members = [l_ for l_ in lanes if self.dest_of[l_] == part]
            for end in (0, 1):
                for want_via in (True, False):
                    tb = set()
                    for l_ in members:
                        opts = self.T[l_] if end == 0 else self.B[l_]
                        tb |= {(l_, end, i) for i, o in enumerate(opts) if ((o[3] or 0) > 0) != want_via}
                    s_ = dict(best)
                    moved = 0
                    for l_ in members:
                        if (l_, end, s_[l_][end]) not in tb:
                            continue
                        opts = self.T[l_] if end == 0 else self.B[l_]
                        ok = [i for i in range(len(opts)) if (l_, end, i) not in tb]
                        if not ok:
                            break                   # (a lane with no option of that kind: the group cannot switch)
                        put = lambda i, l_=l_: {**s_, l_: (i, s_[l_][1]) if end == 0 else (s_[l_][0], i)}
                        s_ = put(min(ok, key=lambda i: self.score(put(i))[0]))
                        moved += 1
                    else:
                        if not moved:
                            continue
                        self.tabu = tb
                        try:
                            s_ = self.search(s_, log=lambda *a: None)
                        finally:
                            self.tabu = set()
                        s_ = self.search(s_, log=lambda *a: None)
                        v_, _p = self.score(s_)
                        if v_ < bv - 1e-9:
                            log(f'  whole ends: {part}\'s {("teeth", "berths")[end]} all '
                                f'{"with" if want_via else "without"} a via ({moved} moved): objective {bv:.2f} -> {v_:.2f}')
                            best, bv = s_, v_
        return best

    def pocket_kicks(self, state, log=print):
        """a GROUP's POCKETS split again, a pocket's lanes at once: each split Group.arrange searches (Group.units --
        the lanes nearest one mouth out of it, the ones nearest the other out of that, the rest straight under), each
        lane on its own berth, the best split by the objective searched again and kept when better. A lane out of a
        mouth past one that goes straight under crosses it in the pocket (pconf, a conflict), so the one-lane search
        never moved a pocket's lanes from straight under to a mouth: a group bus's lanes to a connector's pins
        across a pocket from a QFN went straight under the QFN, every one, where a human took the six nearest the
        far mouth round the QFN, and the via fence under it left three of them no room"""
        if self.group is None:
            return state
        best, (bv, _bp) = dict(state), self.score(state)
        lanes = [l_ for l_, _lg in self.lanes]
        for _pass in range(POCKET_PASSES):
            idx, opts = {}, {}
            for l_ in lanes:
                b0 = self.B[l_][best[l_][1]]
                idx[l_] = [j for j, o in enumerate(self.B[l_]) if o[0] == b0[0] and o[1] == b0[1]]
                opts[l_] = [self.B[l_][j][5] for j in idx[l_]]
            moved = False
            for alts in self.group.units(opts, lanes):
                if not alts or len(alts[0]) < 2:
                    continue                    # (a lane alone: the one-lane search's)
                cands = []
                for alt in alts:
                    s_ = dict(best)
                    for l_, i in alt.items():
                        s_[l_] = (s_[l_][0], idx[l_][i])
                    if s_ != best:
                        cands.append((self.score(s_)[0], sorted(alt.items()), s_))
                if not cands:
                    continue
                _v, _k, s_ = min(cands, key=lambda c_: (c_[0], c_[1]))
                s_ = self.search(s_, log=lambda *a: None)
                v_, p_ = self.score(s_)
                if v_ < bv - 1e-9:
                    log(f'  whole ends: pocket split ({len(alts[0])} lanes): objective {bv:.2f} -> {v_:.2f}')
                    best, bv, moved = s_, v_, True
            if not moved:
                break
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
THREADS = 4         # the ends model's threads, the calling one counted: the most exact routes (one CP-SAT worker
#                     each) at once -- the winding's cuts, and the teeth's search beside the berths' (choose)
_POOL = []


def _pool():
    """the ends model's pool, THREADS - 1 workers (the calling thread the last), made once in the process. Only the
    teeth's search, run on it, waits on others (its own winding's cuts), and they wait on nothing: no task is starved"""
    if not _POOL:
        _POOL.append(ThreadPoolExecutor(THREADS - 1))
    return _POOL[0]


def cut_phase(E, state, vp, log=print, into=None):
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
    if E.group is not None:
        E.wcut = None
        return state, vp                # (a GROUP's cut stands anywhere off its front already: Group.score)
    pts = [E.B[l_][state[l_][1]][1] for l_ in E.prim]
    DB = whole_frame.grown(E.dbox, pts)
    seen = {whole_frame.cut_sig(pts, DB, p0.get('cut'))}
    cands = []
    E.wcut = None
    exits = [o[1] for l_ in E.prim for o in E.B[l_]] + [m.exit_pt for l_ in E.prim for o in E.B[l_] for m in o[0]]
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
    tried = sorted(cands, key=lambda t: (t[0], str(t[1])))[:WIND_TRY]

    def one(c):
        # (each cut on a copy of the model of its own -- its cut, its search's pool, the memo as it stood -- sharing
        # only caches written once per key, a value set twice set the same: the cuts searched and routed AT ONCE, each
        # exact route a CP-SAT worker that gives up the GIL as it solves. The same ends as one cut after another, in
        # the time of the slowest -- the two-end USB and Ethernet bus's fanout spent 110 s of its 160 s here)
        Ec = copy.copy(E)
        Ec.wcut, Ec.pool = c, {}
        Ec._memo, Ec._parts = dict(E._memo), collections.OrderedDict(E._parts)
        t_c = time.time()
        s_ = Ec.iterate(Ec.pocket_kicks(Ec.search(state, log=quiet), log=quiet), log=quiet)
        t_s = time.time() - t_c
        # (the searched ends routed exact first, and the search's other best ends (best_exact) only when these come
        # within EXACT_MARGIN of the far cut's less WIND_GAIN -- the pool stands within EXACT_MARGIN of them by the
        # estimate. A cut far from winning spent up to seven exact routes at the work limit to lose: h616_lpddr4's
        # LPDDR4 on four layers tried 57 cuts in its two fanouts, none taken, 1726 s of its 3318)
        v1, p1 = Ec.score(s_, exact=True)
        far_ = not p0.get('exact_failed') and not p1.get('exact_failed') and v1 >= v0 - WIND_GAIN + EXACT_MARGIN
        s_, vp_ = (s_, (v1, p1)) if far_ else Ec.best_exact(s_, log=quiet)
        return Ec, s_, vp_, t_s, time.time() - t_c - t_s, far_
    # (the first cut on this thread, the others on the ends model's pool -- THREADS in all)
    futs = [_pool().submit(one, c) for _v, c in tried[1:]]
    runs = ([one(tried[0][1])] if tried else []) + [f_.result() for f_ in futs]
    for (v_est, c), (Ec, s_, vp_, t_s, t_x, far_) in zip(tried, runs):
        E._memo.update(Ec._memo)
        E.pool = Ec.pool
        pv = lambda p_: (f'{p_["fan"] + p_["route"]} v, ride {p_["ride"]}, cong {p_.get("cong", 0)}, '
                         f'{p_["crossings"]} x')
        log(f'  whole ends: winding, the cut at {c[0]} {c[1]:.3f} (estimate {v_est:.2f} on the far cut\'s ends): '
            f'objective {vp_[0]:.2f} ({pv(vp_[1])}) against the far cut\'s {v0:.2f} ({pv(p0)}); '
            f'searched in {t_s:.0f} s, routed exact in {t_x:.0f} s' + (' (its searched ends alone)' if far_ else ''))
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
        (WIND_SEEN if into is None else into).append(best[2])
        log(f'  whole ends: WOUND -- the destination cut at {best[2][0]} {best[2][1]:.3f}: {_fmt(best[1][1])}')
    return best[0], best[1]


def _searched(E, s0, log=print):
    """the ends search (search, iterated, ban kicks) from `s0`"""
    quiet = lambda *a: None
    return E.kind_kicks(E.ban_kicks(E.iterate(E.pocket_kicks(E.search(s0, log=quiet), log=log), log=quiet), log=log),
                        log=log)


# THE JOINT ENDS (an escape base; ENDS_JOINT=0: the local search alone): every lane's tooth and berth chosen TOGETHER by CP-SAT over the ends
# model's own menus, CROSSINGS FIRST -- the local search's objective prices a stack at five vias and a crossing at a
# hundredth of one, so it buys stacks back with crossings the whole solve then cannot place: on the zynq LVDS bus's
# escapes 117 of its local optima were put to the solve's strict check, and at K47 none of 78 had a plan, where the
# joint ends had one at the first check on all six round-one frames (K32/K40/K47, edge-free and not), in 10-70 s
JOINT = (awx_settings.get('ENDS_JOINT') or '1') == '1'
JOINT_X = 10.0             # a crossing, in vias
JOINT_LEARN = 20.0         # a crossing in a failing check's core: this many vias more, doubled each time it is again
JOINT_TRIES = 5            # the joint ends put to the check at most this many times; none with a plan, the last
#                           state stands -- and a round left with nets open is followed by the next seed's (whole_route)
JOINT_SOFT = 3             # the joint ends solved again at most this many times, each core of conflicts no state holds priced
JOINT_BATCHES = 40         # CP-SAT's interleaved batches, the solve's budget (reproducible)
JOINT_SEED = awx_settings.get('JOINT_SEED') or ''   # (probe) CP-SAT's random_seed for the joint ends: another trajectory
START_WORK = 30.0          # the search's start, the nearest state the rules allow: CP-SAT's deterministic seconds, one worker


JOINT_SWEEPS = 30          # the descent's sweeps at most (joint_descent)


def _far_opt(E, o):
    """a tooth option on the source's far face (whole_ends.FAR_TOOTH: the lane goes round the source)"""
    return o[0][0].direction == E.far_dir


def _place_b(E, o, DB, cut):
    """a berth option's place round the destination at the cut: round its grown box DB (whole_frame.perim_c), or a
    GROUP's -- its way's place on the walk, unrolled at the cut's point ('P', x, y)"""
    import whole_frame
    if E.group is None:
        return whole_frame.perim_c(o[1], DB, cut)
    return E.group.unroll(o[5][1], E.group.s(cut[1:]))


def joint_boxes(E, s):
    """(SB, DB): the two pad boxes grown by the ends of state `s` (whole_frame.grown), the orders' frame"""
    import whole_frame
    return (whole_frame.grown(E.sbox, [E.T[l_][s[l_][0]][1] for l_, _lg in E.lanes]),
            whole_frame.grown(E.dbox, [E.B[l_][s[l_][1]][1] for l_, _lg in E.lanes]))


def joint_descent(E, s0, boxes, cut, learned=None):
    """(state, cost, crossings) -- the JOINT ENDS' objective DESCENDED from `s0` at the cut `cut` (boxes (SB, DB)):
    each lane in turn takes its best tooth and berth TOGETHER against every other lane's ends as they stand -- the
    fanout's vias, JOINT_X a crossing and `learned` more, a single's blocked front and the feedback's avoided ends; the
    conflicts with the others' ends, a pair's opposite hands and the refused options kept out -- sweep after sweep until
    none moves (JOINT_SWEEPS at most). CP-SAT's hint, and how the cuts are ranked: the greedy start's crossings are a
    hundred and more, and from there a reproducible solve spends its budget coming down (zynq LVDS K47: 40 batches, 78 s,
    50 crossings left and no plan)"""
    import numpy as np
    import whole_frame
    SB, DB = boxes
    lanes = [l_ for l_, _lg in E.lanes]
    n = len(lanes)
    Q = lambda v_: int(round(v_ * 1000))
    TQ = Q(TIE_MM)
    cut_ = float(cut[1]) if isinstance(cut, (tuple, list)) and cut[0] == 'E' else cut
    # (the source's far-face cut: the start's, as the model reads it -- a far-face tooth's side round the source)
    scut = E.score(s0)[1].get('scut')
    PT = {l_: np.array([Q(whole_frame.perim_s(o[1], SB, scut)) for o in E.T[l_]]) for l_ in lanes}
    PB = {l_: np.array([Q(_place_b(E, o, DB, cut_)) for o in E.B[l_]]) for l_ in lanes}
    lin = {}
    for l_ in lanes:
        for k_, O_ in ((0, E.T[l_]), (1, E.B[l_])):
            c_ = np.array([np.inf if o[4] else float(o[3]) + (E.tooth_price(l_, i_) if k_ == 0 else 0.0)
                           for i_, o in enumerate(O_)])
            fr, fb = E.front.get((l_, k_)) or {}, E.fb_avoid.get((l_, k_)) or {}
            c_ += np.array([fr.get(i, 0.0) + FB_AVOID * fb.get(i, 0.0) for i in range(len(O_))])
            lin[(l_, k_)] = c_
    bad = {l_: np.array([[not _same_hands(E, l_, i, j) for j in range(len(E.B[l_]))] for i in range(len(E.T[l_]))])
           for l_, lg in E.lanes if len(lg) == 2}
    W = {l_: np.array([0.0 if o == l_ else JOINT_X + (learned or {}).get(frozenset((l_, o)), 0.0) for o in lanes])
         for l_ in lanes}
    cf = {(l_, k_, i): [(o, j) for o, j in E.oconf.get((l_, k_, i), ())]
          for l_ in lanes for k_, O_ in ((0, E.T[l_]), (1, E.B[l_])) for i in range(len(O_))}
    s = dict(s0)

    def costs(l_):
        # (every (tooth, berth) of lane l_ against the others' ends as they stand: crossed where the orders are STRICTLY
        # opposite, a tie deciding nothing, as the model's)
        ps = np.array([PT[o][s[o][0]] for o in lanes])
        pd = np.array([PB[o][s[o][1]] for o in lanes])
        bs_, as_ = PT[l_][:, None] < ps[None, :] - TQ, PT[l_][:, None] > ps[None, :] + TQ
        bd_, ad_ = PB[l_][:, None] < pd[None, :] - TQ, PB[l_][:, None] > pd[None, :] + TQ
        xx_ = (bs_[:, None, :] & ad_[None, :, :]) | (as_[:, None, :] & bd_[None, :, :])
        c_ = lin[(l_, 0)][:, None] + lin[(l_, 1)][None, :] + xx_.astype(float) @ W[l_]
        for k_ in (0, 1):
            hit = [i for i in range(len((E.T if k_ == 0 else E.B)[l_])) if any(s[o][k_] == j for o, j in cf[(l_, k_, i)])]
            if hit:
                if k_ == 0:
                    c_[hit, :] = np.inf
                else:
                    c_[:, hit] = np.inf
        if l_ in bad:
            c_[bad[l_]] = np.inf
        return c_
    for _sw in range(JOINT_SWEEPS):
        moved = False
        for l_ in lanes:
            c_ = costs(l_)
            i, j = np.unravel_index(int(np.argmin(c_)), c_.shape)
            if c_[i, j] < c_[s[l_]] - 1e-9:
                s[l_] = (int(i), int(j))
                moved = True
        if not moved:
            break
    tot, nx = 0.0, 0
    for a in range(n):
        la = lanes[a]
        tot += lin[(la, 0)][s[la][0]] + lin[(la, 1)][s[la][1]]
        for b in range(a + 1, n):
            lb = lanes[b]
            ds_, dd_ = PT[la][s[la][0]] - PT[lb][s[lb][0]], PB[la][s[la][1]] - PB[lb][s[lb][1]]
            if (ds_ < -TQ and dd_ > TQ) or (ds_ > TQ and dd_ < -TQ):
                nx += 1
                tot += W[la][b]
    return s, tot, nx


def _last_ends():
    """{lane: [tooth signature, berth signature]}: the joint ends the round before chose (its plan sidecar's
    `joint_ends`, INCREMENTAL=), or None"""
    import json
    p_ = awx_settings.get('INCREMENTAL')
    if not p_ or not os.path.isfile(p_):
        return None
    try:
        return json.load(open(p_, encoding='utf-8')).get('joint_ends') or None
    except (OSError, ValueError):
        return None


def _opt_sig(o):
    """an end option's signature, its legs' moves -- the same end on a field built again"""
    import source_realize as sr
    return tuple(sr.move_sig(m_) for m_ in o[0])


def _stars(pairs, idx):
    """{(a, b): [(side, centre option, [the other lane's options])]}: a relation between two lanes' options (`pairs`
    [(lane, option, lane, option)]) covered by stars round the side with the fewer centres"""
    rel = collections.defaultdict(set)
    for a, i, b, j in pairs:
        if idx[a] > idx[b]:
            a, i, b, j = b, j, a, i
        rel[(a, b)].add((i, j))
    out = {}
    for (a, b), ps in sorted(rel.items(), key=lambda kv: (idx[kv[0][0]], idx[kv[0][1]])):
        fa, fb = collections.defaultdict(list), collections.defaultdict(list)
        for i, j in sorted(ps):
            fa[i].append(j)
            fb[j].append(i)
        out[(a, b)] = [(0, i, js) for i, js in fa.items()] if len(fa) <= len(fb) else \
            [(1, j, is_) for j, is_ in fb.items()]
    return out


def joint_state(E, hint, learned=None, log=print, boxes=None):
    """(state, how) -- _joint_state; and where it has NO STATE because two or more lanes' options cannot all be held
    apart (two balls hemmed in side by side, every tooth of one conflicting with every tooth of the other), the
    conflicts that leave no state (its CORE: _joint_state(core=True), each pair's conflicts behind an assumption)
    PRICED instead of forbidden -- BIG each, as the local search prices a conflict it cannot avoid -- and the joint
    ends solved again, up to JOINT_SOFT times. Forbidden, a 41-lane escape bus's first ends had no state and the local
    search took them: 380 s of windings alone, its ends 129 crossings; the pair priced, the joint ends' whole choice,
    its check with it, 277 s and 22 crossings"""
    soft = set(getattr(E, '_soft', ()))         # (a core priced in one try, priced in the later ones: joint_choose)
    for _k in range(JOINT_SOFT + 1):
        s, how = _joint_state(E, hint, learned, log, boxes, soft=soft)
        if s is not None or _k == JOINT_SOFT or 'INFEASIBLE in' not in how:
            break
        core = _joint_state(E, hint, learned, log, boxes, soft=soft, core=True)
        if not core:
            break
        soft |= core
        log(f'  whole ends: the joint ends: no state -- {len(core)} lane pair(s) whose options cannot all be held '
            f'apart priced as the local search prices a conflict: '
            + ', '.join(f'{a}/{b} at the {"teeth" if k_ == 0 else "berths"}' for k_, a, b in sorted(core)))
    E._soft = soft
    return s, how


def _joint_state(E, hint, learned=None, log=print, boxes=None, soft=(), core=False):
    """(state, how) -- THE JOINT ENDS on `E` at its cut (E.wcut, held: the far face's or a winding's), hinted from
    `hint` (`boxes` (SB, DB) the orders' frame, else the hint's: joint_boxes): one boolean per lane and option (refused
    ones left out); a lane's conflicts with another's, and a pair's opposite hands, forbidden; each end on a layer of
    its domain (end_domain: a surface end fixed, a via end on the layers its runs clear, its via dropped on its neck's);
    two lanes whose orders round the two pad boxes are STRICTLY opposite (a tie, one exit on two layers, decides nothing:
    TIE_MM) CROSS, and where neither changes layer their ends stand on different layers; two via ends whose runs meet,
    and two legs stacked at one exit, on different layers; and what the check's refusals taught (joint_choose: E._nogoods,
    E._nochg, E._nocross). Minimized: a lane changing layer FIRST (a leg each, lexicographic), then the fanout's vias
    (less those dropped), JOINT_X a crossing and `learned` {frozenset of two lanes: vias} more for those, a single's
    blocked front, the whole route's feedback (FB_AVOID, FB_PAIR) and a walled plane ball (WALLED) as the objective
    prices them.

    Each rule is written per pair of LANES, not per pair of options -- the model a sixth the size, built and presolved
    in a second where the per-option form took fifteen: a lane's conflicts with another's an at-most-one round each
    option of the side with fewer; each end's layer one-hot (a layer only where the chosen option allows it); two ends
    whose runs meet or stack one literal per pair of lanes, raised by any such pair of options; the crossings on
    ORDER-ENCODED places (each lane's [place >= v] literals, chained), a lane before another a clause per threshold, a
    pair whose order the places settle a constant. The search starts from a FEASIBLE state: the descent's ends (`hint`,
    which breaks rules the descent does not see -- CP-SAT ignores a hint that does) and the joint ends the round before
    chose (_last_ends), each moved to the nearest state every rule allows (fewest ends moved, START_WORK), its layers
    and crossings then solved for the objective, the better one the hint of every variable. (None, why) with no state.
    `soft`: (end, lane, lane) pairs whose conflicts are priced (BIG each), not forbidden; `core`: only the model, its
    other pairs' conflicts each behind an assumption -- the set of those a state cannot hold (CP-SAT's sufficient
    assumptions), empty when it has one"""
    import bisect
    import whole_frame
    import whole_solve
    from ortools.sat.python import cp_model
    t0 = time.time()
    lanes = [l_ for l_, _lg in E.lanes]
    LG = dict(E.lanes)
    npair = {l_: len(LG[l_]) for l_ in lanes}
    idx = {l_: k for k, l_ in enumerate(lanes)}
    OT = {l_: [i for i, o in enumerate(E.T[l_]) if not o[4]] for l_ in lanes}
    OB = {l_: [j for j, o in enumerate(E.B[l_]) if not o[4]] for l_ in lanes}
    bare = [l_ for l_ in lanes if not OT[l_] or not OB[l_]]
    if bare:
        return None, f'no clean option for {bare}'
    SB, DB = boxes or joint_boxes(E, hint)
    cut = E.wcut
    if isinstance(cut, (tuple, list)) and cut[0] == 'E':
        cut = float(cut[1])
    Q = lambda v_: int(round(v_ * 1000))
    O_ = {0: OT, 1: OB}
    opts = lambda k_: E.T if k_ == 0 else E.B
    NL = E.NL
    m = cp_model.CpModel()
    # (every set the model is built from walked SORTED: a lane's name hashes differently in every process, and a model
    # built in another order breaks CP-SAT's ties another way -- three runs of one frame, three ends, all optimal)
    xs = {k_: {l_: {i: m.NewBoolVar('') for i in O_[k_][l_]} for l_ in lanes} for k_ in (0, 1)}
    for k_ in (0, 1):
        for l_ in lanes:
            m.AddExactlyOne(xs[k_][l_].values())
    # every state the whole solve's check refused (joint_choose), by the LANES its core named: not all of them on the
    # ends they had there again
    for ng_ in getattr(E, '_nogoods', ()):
        lits_ = []
        for l_, (i_, j_) in sorted(ng_.items()):
            if l_ not in idx:
                continue
            if i_ not in xs[0][l_] or j_ not in xs[1][l_]:
                lits_ = None
                break
            lits_ += [xs[0][l_][i_].Not(), xs[1][l_][j_].Not()]
        if lits_:
            m.AddBoolOr(lits_)
    cost = []
    # a lane's layer chosen at its ends: a leg changing layer between them ranked before everything else the model
    # prices (lexicographic, below) -- a via at an escape stands in the array's via grid, a change in the channel needs
    # room beside it there (the human's bus: 51 of 54 nets two escape vias and one layer between)
    chg_cost = []
    # the conflicts, per pair of lanes
    cpairs = {0: [], 1: []}
    held = {}               # (core: each pair's assumption -- its conflicts held)
    for k_ in (0, 1):
        for l_ in lanes:
            for i in xs[k_][l_]:
                for o, j in sorted(E.oconf.get((l_, k_, i), ())):
                    if idx[o] > idx[l_] and j in xs[k_][o]:
                        cpairs[k_].append((l_, i, o, j))
        for (a, b), st_ in _stars(cpairs[k_], idx).items():
            gate = None
            if (k_, a, b) in soft:
                gate = m.NewBoolVar('')             # (its conflicts held, else BIG paid)
                cost.append(Q(BIG) * gate.Not())
            elif core:
                gate = held[(k_, a, b)] = m.NewBoolVar('')
            for side, c, others in st_:
                ca, ob = (a, b) if side == 0 else (b, a)
                ct_ = m.Add(xs[k_][ca][c] + sum(xs[k_][ob][j] for j in others) <= 1)
                if gate is not None:
                    ct_.OnlyEnforceIf(gate)
    # a pair's opposite hands
    for l_ in lanes:
        if npair[l_] == 2:
            for i in OT[l_]:
                js = [j for j in OB[l_] if not _same_hands(E, l_, i, j)]
                if js:
                    m.Add(xs[0][l_][i] + sum(xs[1][l_][j] for j in js) <= 1)
    # each end's layer, one-hot; the via dropped on its neck's layer
    dom = {(k_, l_, i): E.end_domain(opts(k_)[l_][i], k_, LG[l_]) for k_ in (0, 1) for l_ in lanes for i in xs[k_][l_]}
    y = {k_: {l_: [m.NewBoolVar('') for _ in range(NL)] for l_ in lanes} for k_ in (0, 1)}
    for k_ in (0, 1):
        for l_ in lanes:
            m.AddExactlyOne(y[k_][l_])
            for q in range(NL):
                ok = [v for i, v in xs[k_][l_].items()
                      if (dom[(k_, l_, i)][0] == q if dom[(k_, l_, i)][0] is not None else q in dom[(k_, l_, i)][2])]
                if len(ok) < len(xs[k_][l_]):
                    m.Add(y[k_][l_][q] <= sum(ok))
            byneck = collections.defaultdict(list)
            for i, v in xs[k_][l_].items():
                fx_, nk_, ok_ = dom[(k_, l_, i)]
                if fx_ is None and nk_ is not None and nk_ in ok_:
                    byneck[nk_].append(v)
            if byneck:
                d_ = m.NewBoolVar('')
                es = []
                for q, vs in sorted(byneck.items()):
                    e_ = m.NewBoolVar('')
                    m.Add(e_ <= y[k_][l_][q])
                    m.Add(e_ <= sum(vs))
                    es.append(e_)
                m.Add(d_ <= sum(es))
                cost.append(-Q(1.0) * npair[l_] * d_)
    # a change where the two ends' layers differ: a leg each, ranked first (below)
    chg = {}
    for l_ in lanes:
        c_ = m.NewBoolVar('')
        for q in range(NL):
            m.Add(c_ >= y[0][l_][q] - y[1][l_][q])
            m.Add(c_ + y[0][l_][q] + y[1][l_][q] <= 2)
        chg[l_] = c_
        chg_cost.append(npair[l_] * c_)
    # a lane the whole solve's check found no room for a change in (joint_choose): one layer end to end, on any ends --
    # its options stand along the same two faces, its route as short on each
    for l_ in sorted(getattr(E, '_nochg', ())):
        if l_ in idx:
            m.Add(chg[l_] == 0)
    # two via ends whose runs meet on different layers (the estimate's sep), and two legs stacked at one exit:
    # per pair of lanes, then different layers
    own_ = lambda k_, l_, i: (lambda L_: E.RL.index(L_) if L_ in E.RL else None)(opts(k_)[l_][i][2])
    for k_ in (0, 1):
        sp = []
        E.meet_all(k_)
        for la in lanes:
            for ia in xs[k_][la]:
                if dom[(k_, la, ia)][0] is not None:
                    continue
                for lb, ibs in E._meet_nbrs(la, k_, ia).items():
                    if idx[lb] <= idx[la]:
                        continue
                    for ib in sorted(ibs):
                        if ib in xs[k_][lb] and own_(k_, la, ia) != own_(k_, lb, ib) and dom[(k_, lb, ib)][0] is None:
                            sp.append((la, ia, lb, ib))
        cells = collections.defaultdict(list)
        for l_ in lanes:
            for i in xs[k_][l_]:
                for m_ in opts(k_)[l_][i][0]:
                    cells[(math.floor(m_.exit_pt[0] / 0.25), math.floor(m_.exit_pt[1] / 0.25))].append((l_, i, m_))
        for (cx, cy), here in sorted(cells.items()):
            near = [e for dx in (-1, 0, 1) for dy in (-1, 0, 1) for e in cells.get((cx + dx, cy + dy), ())]
            for la, ia, ma in here:
                for lb, ib, mb in near:
                    if idx[la] >= idx[lb] or ma.layer == mb.layer or \
                            math.hypot(ma.exit_pt[0] - mb.exit_pt[0], ma.exit_pt[1] - mb.exit_pt[1]) > DUP_TOL:
                        continue
                    sp.append((la, ia, lb, ib))
        for (a, b), st_ in _stars(sp, idx).items():
            z_ = m.NewBoolVar('')
            for side, c, others in st_:
                ca, ob = (a, b) if side == 0 else (b, a)
                m.Add(z_ >= xs[k_][ca][c] + sum(xs[k_][ob][j] for j in others) - 1)
            for q in range(NL):
                m.Add(y[k_][a][q] + y[k_][b][q] + z_ <= 2)
    # the crossings: places round the two boxes (whole_frame's perimeters), order-encoded
    scut = E.score(hint)[1].get('scut')     # (the source's far-face cut, the hint's: whole_frame.src_cut)
    pos = {(0, l_, i): Q(whole_frame.perim_s(E.T[l_][i][1], SB, scut)) for l_ in lanes for i in OT[l_]}
    pos.update({(1, l_, j): Q(_place_b(E, E.B[l_][j], DB, cut)) for l_ in lanes for j in OB[l_]})
    pv = {(k_, l_): sorted({pos[(k_, l_, i)] for i in O_[k_][l_]}) for k_ in (0, 1) for l_ in lanes}
    H = {}
    for (k_, l_), vs in pv.items():
        h_ = [1] + [m.NewBoolVar('') for _ in vs[1:]] + [0]
        for t_, v_ in enumerate(vs):
            m.Add(h_[t_] - h_[t_ + 1] == sum(xs[k_][l_][i] for i in O_[k_][l_] if pos[(k_, l_, i)] == v_))
        H[(k_, l_)] = h_
    TQ = Q(TIE_MM)

    def ge(k_, l_, v_):
        """[lane l_'s place at end k_ >= v_]: a literal, or 0/1"""
        vs = pv[(k_, l_)]
        t_ = bisect.bisect_left(vs, v_)
        return H[(k_, l_)][t_] if t_ < len(vs) else 0

    neg = lambda e_: 1 - e_ if isinstance(e_, int) else e_.Not()
    Bc = {}

    def before(k_, a, b):
        """[a's place < b's - TQ at end k_]: raised wherever the places say so; or 0/1"""
        key = (k_, a, b)
        if key not in Bc:
            va, vb = pv[(k_, a)], pv[(k_, b)]
            if va[-1] + TQ < vb[0]:
                Bc[key] = 1
            elif vb[-1] - va[0] <= TQ:
                Bc[key] = 0
            else:
                rows = {}
                for p_ in va:
                    g_ = ge(k_, b, p_ + TQ + 1)
                    if isinstance(g_, int):
                        if g_ == 0:
                            continue
                        rows[('c', g_)] = (p_, g_)
                    else:
                        rows[g_.Index()] = (p_, g_)
                lit = m.NewBoolVar('')
                for p_, g_ in rows.values():
                    lea = neg(ge(k_, a, p_ + 1))
                    if any(isinstance(e_, int) and e_ == 0 for e_ in (lea, g_)):
                        continue
                    m.AddBoolOr([lit] + [neg(e_) for e_ in (lea, g_) if not isinstance(e_, int)])
                Bc[key] = lit
        return Bc[key]
    X, const = {}, 0
    for a, b in itertools.combinations(lanes, 2):
        w_ = Q(JOINT_X + (learned or {}).get(frozenset((a, b)), 0.0))
        terms = [(before(0, a, b), before(1, b, a)), (before(0, b, a), before(1, a, b))]
        if any(all(isinstance(e_, int) and e_ == 1 for e_ in t_) for t_ in terms):
            const += w_
            xv = 1
        elif all(any(isinstance(e_, int) and e_ == 0 for e_ in t_) for t_ in terms):
            continue
        else:
            xv = m.NewBoolVar('')
            for t_ in terms:
                if any(isinstance(e_, int) and e_ == 0 for e_ in t_):
                    continue
                m.AddBoolOr([xv] + [neg(e_) for e_ in t_ if not isinstance(e_, int)])
            cost.append(w_ * xv)
            if frozenset((a, b)) in getattr(E, '_nocross', ()):
                m.Add(xv == 0)                     # (a crossing whose window the check could not hold: not again)
        X[(a, b)] = xv
        for q in range(NL):
            m.Add(y[0][a][q] + y[0][b][q] + xv - chg[a] - chg[b] <= 2)
    # the options' own costs: the fanout's vias, a single's blocked front, a far-face tooth, the feedback's avoided ends
    for k_ in (0, 1):
        for l_ in lanes:
            fr, fb = E.front.get((l_, k_)) or {}, E.fb_avoid.get((l_, k_)) or {}
            for i, v in xs[k_][l_].items():
                o = opts(k_)[l_][i]
                c_ = Q(1.0) * o[3] + Q(fr.get(i, 0.0)) + Q(FB_AVOID * fb.get(i, 0.0))
                if k_ == 0:
                    c_ += Q(E.tooth_price(l_, i))
                if c_:
                    cost.append(c_ * v)
    for a_, b_, k_, ia, ib in E.fb_pairs:
        for i, wa in ia.items():
            for j, wb in ib.items():
                if i in xs[k_][a_] and j in xs[k_][b_] and wa * wb:
                    z_ = m.NewBoolVar('')
                    m.Add(z_ >= xs[k_][a_][i] + xs[k_][b_][j] - 1)
                    cost.append(Q(FB_PAIR * wa * wb) * z_)
    # a plane ball of the arrays its ends leave no way down (walled_balls): every drop killed, and every strap killed or
    # to a ball with none left
    if E.pw is not None and E.pw.balls:
        kw_ = {}

        def killed(w_):
            if w_ not in kw_:
                kw_[w_] = m.NewBoolVar('')
            return kw_[w_]
        for (l_, k_, i), ws_ in sorted(E.pw.kill.items(), key=lambda kv: repr(kv[0])):
            if i in xs[k_].get(l_, {}):
                for w_ in sorted(ws_, key=repr):
                    m.AddImplication(xs[k_][l_][i], killed(w_))
        alive = {b_: m.NewBoolVar('') for b_, _dw, _sw in E.pw.balls}
        for b_, dw, _sw in E.pw.balls:
            m.AddBoolOr([killed(w_).Not() for w_ in sorted(dw, key=repr)] + [alive[b_].Not()])
        for b_, dw, sw in E.pw.balls:
            outs = []
            for w_, to in sorted(sw, key=repr):
                if to in alive:
                    s_ = m.NewBoolVar('')
                    m.AddImplication(s_, killed(w_).Not()); m.AddImplication(s_, alive[to])
                    outs.append(s_)
            wl_ = m.NewBoolVar('')
            m.AddBoolOr([alive[b_], wl_] + outs)
            cost.append(Q(WALLED) * wl_)
    m.Minimize(sum(cost) + const)
    if chg_cost:
        # (the changes' weight: one more than the most the rest of the objective can take, read off the model itself)
        pr_ = m.Proto()
        ob_ = pr_.floating_point_objective if len(pr_.floating_point_objective.vars) else pr_.objective
        hi_ = 0
        for v_, c_ in zip(ob_.vars, ob_.coeffs):
            dm_ = list(pr_.variables[v_ if v_ >= 0 else -v_ - 1].domain)     # (this proto's lists take no index -1)
            hi_ += abs(c_) * max(abs(dm_[0]), abs(dm_[-1]))
        W_LEX = int(math.ceil(hi_)) + 1
        m.Minimize(W_LEX * sum(chg_cost) + sum(cost) + const)
        log(f'  whole ends: changes ranked first (a leg {W_LEX} against the rest at most {hi_})')
    if core:
        mc_ = m.Clone()
        mc_.ClearObjective()
        mc_.AddAssumptions([mc_.GetBoolVarFromProtoIndex(v_.Index()) for v_ in held.values()])
        sc_ = cp_seed.solver()
        sc_.parameters.num_workers = 1
        sc_.parameters.max_deterministic_time = START_WORK
        if sc_.Solve(mc_) != cp_model.INFEASIBLE:
            return set()
        by_ = {v_.Index(): key_ for key_, v_ in held.items()}
        return {by_[i_] for i_ in sc_.SufficientAssumptionsForInfeasibility() if i_ in by_}
    t1 = time.time()
    # THE START: the descent's ends and the last round's, each moved to the nearest state the rules allow and completed
    sv_val = lambda sv_, mm_, v_: sv_.Value(mm_.GetBoolVarFromProtoIndex(v_.Index()))
    starts = [('the descent\'s', {l_: tuple(hint[l_]) for l_ in lanes})]
    last = _last_ends()
    if last:
        import json
        ls_ = {}
        for l_ in lanes:
            e_ = []
            for k_ in (0, 1):
                # (a signature by its JSON text: the sidecar's lists and the options' tuples alike)
                sig = json.dumps((last.get(l_) or (None, None))[k_])
                e_.append(next((i for i in O_[k_][l_] if json.dumps(_opt_sig(opts(k_)[l_][i])) == sig), hint[l_][k_]))
            ls_[l_] = tuple(e_)
        starts.append(('the last round\'s', ls_))
    best, why = None, []
    for nm_, s0 in starts:
        ts_ = time.time()
        r1 = m.Clone()
        r1.ClearObjective()
        r1.Minimize(sum(r1.GetBoolVarFromProtoIndex(xs[k_][l_][s0[l_][k_]].Index()).Not()
                        for k_ in (0, 1) for l_ in lanes if s0[l_][k_] in xs[k_][l_]))
        for k_ in (0, 1):
            for l_ in lanes:
                for i, v in xs[k_][l_].items():
                    r1.AddHint(r1.GetBoolVarFromProtoIndex(v.Index()), int(i == s0[l_][k_]))
        s1_ = cp_seed.solver()
        s1_.parameters.num_workers = 1
        s1_.parameters.max_deterministic_time = START_WORK
        q1 = s1_.Solve(r1)
        if q1 not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
            why.append(f'{nm_} ends: no state near them ({s1_.StatusName(q1)})')
            continue
        ends_ = {(k_, l_): next(i for i, v in xs[k_][l_].items() if sv_val(s1_, r1, v)) for k_ in (0, 1) for l_ in lanes}
        r2 = m.Clone()
        for k_ in (0, 1):
            for l_ in lanes:
                for i, v in xs[k_][l_].items():
                    r2.Add(r2.GetBoolVarFromProtoIndex(v.Index()) == int(i == ends_[(k_, l_)]))
        s2_ = cp_seed.solver()
        s2_.parameters.num_workers = 1
        s2_.parameters.max_deterministic_time = 30.0
        q2 = s2_.Solve(r2)
        if q2 not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
            why.append(f'{nm_} ends: not completed ({s2_.StatusName(q2)})')
            continue
        why.append(f'{nm_} ends {int(s1_.ObjectiveValue())} moved, {s2_.ObjectiveValue() / 1000:.1f} '
                   f'({time.time() - ts_:.1f} s)')
        if best is None or s2_.ObjectiveValue() < best[0]:
            best = (s2_.ObjectiveValue(), [s2_.Value(r2.GetIntVarFromProtoIndex(i_))
                                           for i_ in range(len(m.Proto().variables))], nm_)
    if best:
        for i_, v_ in enumerate(best[1]):
            m.AddHint(m.GetIntVarFromProtoIndex(i_), v_)
        why.append(f'started from {best[2]}')
    t2 = time.time()
    sv = cp_seed.solver()
    # (REPRODUCIBLE, as whole_solve's: stopped by a count of interleaved batches, the workers sharing no clauses)
    sv.parameters.num_workers = whole_solve.SOLVE_WORKERS
    sv.parameters.subsolvers.extend(whole_solve.FALLBACK)
    sv.parameters.interleave_search = True
    sv.parameters.max_num_deterministic_batches = JOINT_BATCHES
    sv.parameters.share_glue_clauses = False
    sv.parameters.share_binary_clauses = False
    if JOINT_SEED:
        sv.parameters.random_seed = int(JOINT_SEED)
    r = sv.Solve(m)
    built = f'built in {t1 - t0:.1f} s; start {t2 - t1:.1f} s: {"; ".join(why)}'
    if r not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        return None, f'{built}, {sv.StatusName(r)} in {time.time() - t2:.1f} s'
    s = {l_: (next(i for i, v in xs[0][l_].items() if sv.Value(v)), next(j for j, v in xs[1][l_].items() if sv.Value(v)))
         for l_ in lanes}
    nx = sum(1 if isinstance(x_, int) else sv.Value(x_) for x_ in X.values())
    return s, (f'{built}, {sv.StatusName(r)} in {time.time() - t2:.1f} s, {nx} crossings, objective '
               f'{sv.ObjectiveValue() / 1000:.1f} bound {sv.BestObjectiveBound() / 1000:.1f}')


def joint_choose(st, log=print, fixed=None, seed=None, learned=None, free_teeth=None):
    """choose's answer from THE JOINT ENDS (joint_state) -- on an escape base, at the far face's cut the greedy start's
    berths leave, each state put to the whole solve's strict check (solve_check, exact: the LANES its core names) up to
    JOINT_TRIES times, a refused one teaching every later state (not those lanes on those ends again, a lane with no
    room for a change on one layer, a crossing whose window could not hold not again) and its core's crossings priced
    (JOINT_LEARN, doubled when named again). None with a plan: the last state stands -- the round lays it, and a round
    left with nets open is followed by round 1 again under the next seed (whole_route). None only where the model has
    no state at all (the local search then takes over)"""
    import whole_frame
    t0 = time.time()
    E = Ends(st, src_free=True, fixed=fixed, learned=learned, free_teeth=free_teeth)
    t_e = time.time()
    s0 = E.start(seed)
    t_s = time.time()
    boxes = joint_boxes(E, s0)
    # THE CUT: the far face's (the greedy start's widest gap) and, WINDING (cut_phase's), each gap between the start's
    # berths off it -- each ranked by its descent, the best taken: the zynq LVDS K47 ends of 10 crossings at the cut a
    # winding found had 150 at the start's
    cuts = [E.score(s0, exact=True)[1]['cut']]
    if wind_on() and E.group is None:
        pts = [E.B[l_][s0[l_][1]][1] for l_, _lg in E.lanes]
        exits = [o[1] for l_, _lg in E.lanes for o in E.B[l_]] + [m.exit_pt for l_, _lg in E.lanes for o in E.B[l_]
                                                                   for m in o[0]]
        seen = {whole_frame.cut_sig(pts, boxes[1], cuts[0])}
        for c in whole_frame.cut_gaps(pts, boxes[1], avoid=exits):
            sg = whole_frame.cut_sig(pts, boxes[1], c)
            if not whole_frame.far_cut(c) and sg not in seen:
                seen.add(sg)
                cuts.append(c)
    ranked = []
    for q, c in enumerate(cuts):
        sd, cost, nx = joint_descent(E, s0, boxes, c)
        ranked.append((cost, q, c, sd, nx))
    ranked.sort(key=lambda r_: (r_[0], r_[1]))
    _cost, _q, E.wcut, hint, _nx = ranked[0]
    fc = lambda c_: (f'{c_[0]} {float(c_[1]):.3f}' if isinstance(c_, (tuple, list)) else
                     'far (none)' if c_ is None else f'far {c_:.3f}')
    log(f'  whole ends: the joint ends\' cuts by their descents: '
        + ', '.join(f'{fc(c_)} {cost_:.0f} ({nx_} x)' for cost_, _q2, c_, _s2, nx_ in ranked))
    t_c = time.time()
    xl = {}
    check_on = (awx_settings.get('ENDS_SOLVE_CHECK') or '1') == '1'
    t_chk = []
    # what each refused state teaches, held for every later state (joint_state): the LANES its exact core names not
    # all on those ends again, a lane with no room for a change on its ends on one layer, and two lanes whose crossing's
    # window the check could not hold not crossing
    E._nogoods, E._nochg, E._nocross = [], set(), set()
    last_ = None
    for k in range(JOINT_TRIES):
        s, how = joint_state(E, hint, xl, log=log, boxes=boxes)
        if s is None:
            if last_ is not None:
                log(f'  whole ends: the joint ends: no state ({how}) -- the last one stands')
                s, how = last_
                break
            log(f'  whole ends: the joint ends: no state ({how}) -- the local search')
            return None
        last_ = (s, how)
        t_k = time.time()
        chk = solve_check(E, s, exact=True) if check_on else None
        t_chk.append(time.time() - t_k)
        if chk is None or chk.get('plan'):
            break
        log(f'  whole ends: the joint ends ({how}): the solve\'s check: NO plan [{chk.get("status")}], core '
            f'{chk.get("core")}' + (f', lanes {chk.get("lanes")}' if chk.get('lanes') else '') + f' ({t_chk[-1]:.1f} s)')
        if chk.get('lanes'):
            n0_ = len(E._nochg)
            E._nochg |= {l_ for l_ in chk.get('nochange') or () if l_ in s}
            if len(E._nochg) > n0_:
                log(f'  whole ends: the joint ends: {len(E._nochg) - n0_} lane(s) with no room for a change on their '
                    f'ends, held to one layer: ' + ', '.join(sorted(chk.get('nochange') or ())))
            if chk.get('core'):
                E._nocross |= {frozenset(k_) for k_ in chk['core']}
                log(f'  whole ends: the joint ends: crossings held apart: '
                    + ', '.join('/'.join(sorted(k_)) for k_ in sorted(E._nocross, key=sorted)))
            E._nogoods.append({l_: tuple(s[l_]) for l_ in chk['lanes'] if l_ in s})
            log(f'  whole ends: the joint ends: {len(E._nogoods)} refused state(s) held -- {len(chk["lanes"])} lane(s) '
                f'this time')
        elif not chk.get('core'):
            log('  whole ends: the joint ends: nothing to learn -- this state stands')
            break
        for a, b in chk.get('core') or ():
            k_ = frozenset((a, b))
            xl[k_] = 2 * xl[k_] if k_ in xl else JOINT_LEARN
        hint = joint_descent(E, s, boxes, E.wcut, xl)[0]      # (the next solve's hint: descended with the core priced)
    else:
        log(f'  whole ends: the joint ends: no plan in {JOINT_TRIES} checks -- the last state stands')
        s, how = last_
    E._memo.clear(); E._parts.clear()
    t_f = time.time()
    v, p = E.score(s, exact=True)
    out = E.choice(s)
    log(f'  whole ends: THE JOINT ENDS ({how}; {k + 1} check(s), {time.time() - t0:.0f} s: ends model {t_e - t0:.1f} s, '
        f'greedy start {t_s - t_e:.1f} s, the cuts\' descents {t_c - t_s:.1f} s, check(s) '
        f'{" + ".join(f"{c_:.1f}" for c_ in t_chk)} s, scored {time.time() - t_f:.1f} s): {_fmt(p)}')
    # (joint_ends: these ends by signature, for the next round's start -- the plan sidecar carries them, escape_plan)
    choose.last = dict(laid=v, laid_parts=p, free=None, free_parts=None, moved=bool(out[1]), st=id(st), cut=E.wcut,
                       parts=p, joint_ends={l_: [_opt_sig(E.T[l_][s[l_][0]]), _opt_sig(E.B[l_][s[l_][1]])]
                                            for l_, _lg in E.lanes})
    return out


def choose(st, log=print, src_free=True, fixed=None, seed=None, learned=None, free_teeth=None):
    """the whole route's ends on a plan state: (berths {net: Move}, teeth to move {net: Move}). The berths are first
    searched against the teeth AS LAID (the laid objective, `choose.last['laid']`: what a realized board is judged
    on); with `src_free` the teeth are searched too, from there, and their moves asked only when that is better.
    `fixed` {net: move signature}: berths held; `seed` {net: Move}: the berths to start from; `learned`: berth pairs
    not to be laid together (Ends); `free_teeth` {lane}: with `src_free`, only these lanes' teeth move. On an escape
    base THE JOINT ENDS first (joint_choose), the search only where they have no plan"""
    if JOINT and src_free and st.get('escape'):
        got = joint_choose(st, log=log, fixed=fixed, seed=seed, learned=learned, free_teeth=free_teeth)
        if got is not None:
            return got
    quiet = lambda *a: None
    E = Ends(st, src_free=False, fixed=fixed, learned=learned)
    if E.pw is not None:
        log(f'  whole ends: {len(E.pw.balls)} plane ball(s) of the arrays kept a way down ({E.pw.n_ways} ways, {E.pw.secs:.0f} s)')
    sL = _searched(E, E.start(seed), log=log)
    sL, (vL, pL) = E.best_exact(sL, log=log)

    def teeth(berths, log_, into=None):
        """the second search, the teeth free: (its Ends, state, objective, parts) from `berths`"""
        E2_ = Ends(st, src_free=True, fixed=fixed, learned=learned, free_teeth=free_teeth)
        # a local search from the berths just chosen on the teeth as laid, iterated
        sF_ = _searched(E2_, E2_.start(berths), log=log_)
        sF_, (vF_, pF_) = E2_.best_exact(sF_, log=log_)
        if wind_on():
            # (the teeth searched on the far face's cut first, then its own winding: held at the berths' cut, the teeth
            # missed the moves the far cut's ends wanted -- s4_bulge's five teeth, 12 vias to 8)
            sF_, (vF_, pF_) = cut_phase(E2_, sF_, (vF_, pF_), log=log_, into=into)
        return E2_, sF_, vF_, pF_
    spec = None
    no_tooth_laid = all(v_ is None for v_ in E.cur.values())
    one_search = ONE_WINDING and no_tooth_laid and src_free
    if wind_on():
        if src_free and not one_search:
            # (the second search SPECULATED beside the berths' winding, from the berths as they stand: a winding that
            # leaves them so leaves the second search's ends its own -- its lines said, and its cut seen, where they
            # would have been; one that moves them has the second search run again from those. The two-end USB and
            # Ethernet bus's fanout overlapped so in each of its three plans, 115 s to 96: their windings took no cut)
            buf, seen2, sL0 = [], [], dict(sL)
            spec = (sL0, seen2, buf, _pool().submit(teeth, E.choice(sL)[0], buf.append, seen2))
        sL, (vL, pL) = cut_phase(E, sL, (vL, pL), log=log)
    out, vF, pF = E.choice(sL), None, None
    log(f'  whole ends: on the teeth as laid: {_fmt(pL)}')
    E2 = None
    if one_search:
        # (probe ENDS_ONE_WINDING: with no tooth laid -- an escape base -- the teeth 'as laid' are every tooth's whole
        # menu (Ends: a lane held only where its teeth stand), so the teeth-free model is this one, and a second search
        # of it from here only went on searching: half the ends' time on the zynq LVDS escapes)
        log('  whole ends: no tooth laid -- one search, the teeth free in it')
    elif src_free:
        if spec is not None:
            sL0, seen2, buf, fut = spec
            got = fut.result()
            if dict(sL) == sL0:
                E2, sF, vF, pF = got
                for line in buf:
                    log(line)
                WIND_SEEN.extend(seen2)
            else:
                E2, sF, vF, pF = teeth(out[0], log)
        else:
            E2, sF, vF, pF = teeth(out[0], log)
        if vF < vL - TOOTH_GAIN:
            out = E2.choice(sF)
            log(f'  whole ends: teeth moved ({len(out[1])}): {_fmt(pF)}')
        else:
            log('  whole ends: no tooth move is better')
    # (the chosen state's own reading -- its lanes' nets over two, load, crossings and blocked fronts -- for the
    # sidecar: escape_plan wrote the lane_* keys from choose.last's top level, where they never were, so the round's
    # solve had no blocked front the check's had (solve_check's front=lane_front) and whole_feedback no lane to name)
    parts_ = pF if (E2 is not None and vF < vL - TOOTH_GAIN) else pL
    # (asked of an ESCAPE's ends and a GROUP's, where the ends model's route and the solve part: lanes leaving the
    # source's near face, a group's ways in. A single destination's fanout ends the solve takes as they come -- asked
    # of a DDR bus's joint fanout four times, 260 s, the state stood each time)
    if (st.get('escape') or len(st.get('dests') or ()) > 1) and (awx_settings.get('ENDS_SOLVE_CHECK') or '1') == '1':
        Ef, sf, pf_ = (E2, sF, pF) if (E2 is not None and vF < vL - TOOTH_GAIN) else (E, sL, pL)
        sf = checked(Ef, sf, pf_, log=log)
        out = Ef.choice(sf)
        parts_ = Ef.score(sf, exact=True)[1]
    # (`st`: the plan state it chose on, and whether it asked tooth moves -- asked none, its berths ARE the choice on
    # that board's teeth as laid, and fanout_from_plan need not ask again; `cut` the cut the ends taken were chosen at:
    # the teeth's search's where its ends are taken, else the first's -- on a board with no tooth laid every tooth is a
    # move either way, and the moves alone named the second search's cut beside the first search's ends)
    # (a far-face choice -- no winding cut taken -- carries the far face's cut its state was scored at, a tied gap's
    # where it scores best (_score); without it the frame cut the far face at its widest gap: 307 crossings for the
    # model's 69, and no plan)
    cut_ = E2.wcut if E2 is not None and vF < vL - TOOTH_GAIN else E.wcut
    choose.last = dict(laid=vL, laid_parts=pL, free=vF, free_parts=pF, moved=bool(out[1]), st=id(st),
                       cut=cut_ if cut_ is not None else (parts_ or {}).get('cut'), parts=parts_)
    return out


CHECK_TRIES = 4           # the solve's check (checked): this many searches again before the ends stand as chosen
CHECK_SWEEP = 8           # ...each first asking a crowded lane's best alternatives alone, the other lanes held
X_LEARN = 5.0             # a crossing the whole solve could not place, in vias -- doubled each time it is learned again


def _same_hands(E, lane, ti, bi):
    """a pair's tooth option ti and berth option bi leave its legs on the same hands -- no crossover"""
    tp, tn = E.T[lane][ti][0]
    bp, bn = E.B[lane][bi][0]
    a_ = _pairs.hand(tp.direction, tp.exit_pt, tn.exit_pt)
    b_ = _pairs.hand(bp.direction, bp.exit_pt, bn.exit_pt, arriving=True)
    return not (a_ and b_ and a_ != b_)


def _sweep(E, state, lanes, log=print, hands=()):
    """a crowded lane's ends changed ALONE, every other lane held: its CHECK_SWEEP best (tooth, berth) options by the
    estimate, each asked of the whole solve -- the first it takes, else None. The fix is often the same place another
    way: on the zynq LVDS bus's K47 round 2, RX_D2's berth at the same exit on the same layer with its legs the other
    way round (no crossover to fit in the channel), the cheaper by the estimate as well"""
    for lane in lanes[:2]:
        if lane not in E.legs_of:
            continue
        cands = []
        for ti in range(len(E.T[lane])):
            for bi in range(len(E.B[lane])):
                if (ti, bi) == tuple(state[lane]):
                    continue
                if lane in hands and len(E.legs_of[lane]) == 2 and not _same_hands(E, lane, ti, bi):
                    # (a pair the rules core names for its HANDS: its crossover has no place -- only the ends that
                    # leave its legs the same way round; by the estimate alone the dozen best were all crossed, the
                    # zynq LVDS K47's RX_D1 with 96 of its 196 combinations uncrossed)
                    continue
                s2 = dict(state)
                s2[lane] = (ti, bi)
                v_ = E.objective(s2)
                if v_ < BIG:
                    cands.append((v_, ti, bi))
        for _v, ti, bi in sorted(cands)[:CHECK_SWEEP]:
            s2 = dict(state)
            s2[lane] = (ti, bi)
            r = solve_check(E, s2, crowd=False)
            if r and r['plan']:
                return s2
    return None


PORTFOLIO = int(awx_settings.get('ENDS_PORTFOLIO') or '0')   # (probe) other states asked before the repair


def _portfolio(E, state, v, t0, log=print):
    """(probe ENDS_PORTFOLIO) the search's OTHER best states asked of the whole solve before any repair: the ends search
    finds many distinct states on its way (its pool: the sweeps', kicks' and cuts' results) and keeps one; where that one
    has no plan, the next best by the estimate that change one of the lanes the check named (crowded, in its core or its
    rules) are asked in turn, PORTFOLIO at most, the first with a plan the choice. Each ask is one solve check (~14 s at
    K47); the repair's re-searches cost 300-475 s where they fail (zynq LVDS: K47 both arms, every repair failed while
    another search's state of the same round had a plan). None: the repair as before"""
    key0 = tuple(sorted(state.items()))
    named = set(v.get('crowded') or []) | {l_ for c_ in (v.get('core') or []) for l_ in c_} | \
        {l_ for l_, _f in (v.get('rules') or [])}
    cands = []
    for k_ in set(E.pool) | getattr(E, 'pool_all', set()):
        if k_ == key0:
            continue
        s_ = dict(k_)
        if named and not any(s_.get(l_) != state.get(l_) for l_ in named):
            continue                    # (moves none of the lanes the solve named: the same failure again)
        o_ = E.objective(s_)
        if o_ < BIG:
            cands.append((o_, k_))
    cands.sort()
    n_ask = 0
    for o_, k_ in cands[:PORTFOLIO]:
        s_ = dict(k_)
        p_ = E.score(s_, exact=True)[1]
        r_ = solve_check(E, s_, front=p_.get('lane_front'), crowd=False)
        n_ask += 1
        if r_ and r_['plan']:
            moved_ = sorted(l_ for l_ in named if s_.get(l_) != state.get(l_))
            log(f'  whole ends: the portfolio: the search\'s state {n_ask} of {len(cands)} by the estimate ({o_:.2f}; '
                f'{", ".join(moved_)} moved) -- the solve takes it ({time.time() - t0:.0f} s): {_fmt(p_)}')
            return s_
    log(f'  whole ends: the portfolio: none of {n_ask} of the search\'s {len(cands)} other states moving '
        f'{sorted(named)} has a plan ({time.time() - t0:.0f} s) -- the repair')
    return None


def checked(E, state, parts, log=print):
    """the chosen state ASKED OF THE WHOLE SOLVE (solve_check) and, where it holds no plan under its strict rules,
    searched again, CHECK_TRIES times at most: first a crowded lane's alternatives alone (_sweep); then the crossings the
    solve could not place together (its CORE: the windows it could not hold) priced in the estimate (x_learned,
    X_LEARN vias, doubled each time a crossing is learned again) and a local search from the state -- each result asked
    again. The first state the solve takes is the choice; none taken, the state as it was"""
    t0 = time.time()
    v = solve_check(E, state, front=(parts or {}).get('lane_front'))
    if v is None:
        return state
    say = lambda what, v_: log(f'  whole ends: {what}: the solve\'s check: ' + (
        'a plan' if v_ and v_['plan'] else f'NO plan [{(v_ or {}).get("status")}], crowded {(v_ or {}).get("crowded")}, '
        f'{len((v_ or {}).get("core") or [])} crossing(s) in its core'
        + (f', rules {_rules_fmt(v_.get("rules"))}' if (v_ or {}).get('rules') else '')) + f' ({time.time() - t0:.0f} s)')
    say('as chosen', v)
    if not v['plan'] and PORTFOLIO:
        s_p = _portfolio(E, state, v, t0, log)
        if s_p is not None:
            return s_p
    cur, quiet = state, (lambda *a: None)
    for k in range(CHECK_TRIES):
        # (no plan in the check's work, and no proof there is none -- UNKNOWN: the round's own solve, with its whole
        # budget, judges these ends; a searched state the check cannot refute stands before the one it proved had
        # none. The group bus's ends the check left UNKNOWN had a plan in the round's solve's 330 s)
        if v['plan'] or v.get('status') == 'UNKNOWN' or not (v['crowded'] or v['core'] or v.get('rules')):
            break
        # (a rules core's lanes, those named for their HANDS first: the sweep tries two lanes' ends alone, and a pair's
        # legs the other way round at the same exit is the cheapest end there is for a crossover with no place)
        hs_ = sorted({l_ for l_, f_ in v.get('rules') or [] if f_ == 'hands'})
        rl_ = hs_ + sorted({l_ for l_, _f in v.get('rules') or []} - set(hs_))
        if hs_:
            # (every pair the rules core names for its HANDS, together: each its best ends that leave its legs the same
            # way round, by the estimate with the others so set -- two pairs each a core of its hands alone, and a sweep
            # of one at a time left the other's core: the zynq LVDS K47's RX_D1 and RX_D4)
            s2 = dict(cur)
            for l_ in hs_:
                if l_ not in E.legs_of or len(E.legs_of[l_]) != 2:
                    continue
                best_ = None
                for ti in range(len(E.T[l_])):
                    for bi in range(len(E.B[l_])):
                        if not _same_hands(E, l_, ti, bi):
                            continue
                        s3 = dict(s2)
                        s3[l_] = (ti, bi)
                        v3 = E.objective(s3)
                        if v3 < BIG and (best_ is None or v3 < best_[0]):
                            best_ = (v3, ti, bi)
                if best_ is not None:
                    s2[l_] = (best_[1], best_[2])
            if s2 != cur:
                r2 = solve_check(E, s2, crowd=False)
                if r2 and r2['plan']:
                    log(f'  whole ends: the pairs named for their hands, uncrossed together ({", ".join(hs_)}) -- the '
                        f'solve takes them ({time.time() - t0:.0f} s)')
                    return s2
        s1 = _sweep(E, cur, v['crowded'] or rl_, log, hands=set(hs_))
        if s1 is not None:
            log(f'  whole ends: a crowded lane\'s other ends alone -- the solve takes them ({time.time() - t0:.0f} s)')
            return s1
        for a_, b_ in v['core']:
            key = frozenset((a_, b_))
            E.x_learned[key] = 2.0 * E.x_learned[key] if key in E.x_learned else X_LEARN
        if v.get('rules'):
            # (the rules the solve could not hold together, by lane (whole_solve rules=True; ENDS_CHECK_RULES=0 off):
            # every crossing among the core's lanes priced as a learned crossing, a pair named for its HANDS -- a
            # crossover it has no place for -- priced for opposite hands, and a lane named for its END LAYERS -- a
            # fixed end whose layer cannot hold -- priced for each fixed end, so the search tries its via ends. A core
            # of crossing windows names none of a lane with no change position or a clique the layers cannot colour:
            # an LVDS bus at K47, 11 lanes off its source's east face, 31 rules; and a surface tooth 1.7 mm from its berth
            # on a group's front, crossing 30 lanes there)
            for a_, b_ in itertools.combinations(rl_, 2):
                key = frozenset((a_, b_))
                E.x_learned[key] = 2.0 * E.x_learned[key] if key in E.x_learned else X_LEARN
            for l_ in sorted({l_ for l_, f_ in v['rules'] if f_ == 'hands'}):
                E.opp_learned[l_] = 2.0 * E.opp_learned[l_] if l_ in E.opp_learned else X_LEARN
            for l_ in sorted({l_ for l_, f_ in v['rules'] if f_ == 'endlay'}):
                E.end_learned[l_] = 2.0 * E.end_learned[l_] if l_ in E.end_learned else X_LEARN
        learned_ = bool(v['core'] or v.get('rules'))
        E._memo.clear(); E._parts.clear(); E.pool.clear()
        nxt, (_vn, pn) = E.best_exact(E.search(cur, log=quiet), log=quiet)
        if nxt == cur and not learned_:
            # (nothing learned, and the search again came back to the same state: its check would be this one's again,
            # and the next try this one -- the sweep, the search, its exact routes and the check all deterministic. An
            # escape bus's second round once made four such tries, 260 s each: crowded lanes and no core)
            log(f'  whole ends: searched again, nothing learned -- the same state ({time.time() - t0:.0f} s)')
            break
        v = solve_check(E, nxt, front=pn.get('lane_front'))
        say(f'searched again, {len(E.x_learned)} crossing(s) priced ({k + 1}): {_fmt(pn)}', v)
        if v and v['plan']:
            return nxt
        cur = nxt
    if not v['plan'] and v.get('status') == 'UNKNOWN' and cur is not state:
        log(f'  whole ends: the check proves nothing on the state searched again -- it stands ({time.time() - t0:.0f} s)')
        return cur
    if not v['plan']:
        log('  whole ends: no searched state has a plan under the solve\'s strict rules -- the ends stand as chosen')
        return state
    return cur


def _rules_fmt(rules):
    """a rules core, short: each lane with its rule families"""
    by = collections.defaultdict(list)
    for l_, f_ in rules or []:
        by[l_].append(f_)
    return '; '.join(f'{l_} {"/".join(sorted(fs))}' for l_, fs in sorted(by.items()))


def state_plan(E, state):
    """(plan, tracks, names) of an ESCAPE state: the plan sidecar's keys as escape_plan.sidecar writes them for its
    ends (each leg's tooth and berth exits, their layers, the escape directions, the pair frame, the ends model's cut),
    the escape runs escape_plan.lay lays for them ({start, end, width, layer, net_id}), and the run's nets in its order"""
    st = E.st
    import rules as _rules_
    fan = _rules_.active().fan_track
    plan = {'ends': {}, 'tooth_layer': {}, 'dest_layer': {}, 'tooth_dir': {}, 'stub_dir': {}, 'chi': int(st['chi'])}
    tracks = []
    for lane, lg in E.lanes:
        to_, bo_ = E.T[lane][state[lane][0]], E.B[lane][state[lane][1]]
        for leg, tm, bm in zip(lg, to_[0], bo_[0]):
            plan['ends'][leg] = [list(tm.exit_pt), list(bm.exit_pt)]
            plan['tooth_layer'][leg], plan['dest_layer'][leg] = tm.layer, bm.layer
            plan['tooth_dir'][leg], plan['stub_dir'][leg] = list(FAR[tm.direction]), list(FAR[bm.direction])
            nid = st['byname'][leg][0]
            for m in (tm, bm):
                for p, q, L in m.legs:
                    if L == m.layer and math.hypot(q[0] - p[0], q[1] - p[1]) > 1e-6:
                        tracks.append({'start': tuple(p), 'end': tuple(q), 'width': fan, 'layer': L, 'net_id': nid})
                if m.vias and m.site is not None:
                    # (its via too: the solve reads an end whose stub carries a via as a VIA END, its layer the solve's
                    # -- without it, a dog-bone the state chose stood on the bench as a fixed end)
                    tracks.append({'via': tuple(m.site), 'net_id': nid})
    sp_ = E.score(state)[1]
    # (the destination's cut the state was SCORED at: winding's, else the far face's tied cut it scores best at
    # (_score) -- not the far face's widest gap, which the frame takes where the sidecar carries no cut: the two split
    # the far face differently, and the check faced crossings the model never priced, 69 against 307)
    dc_ = E.wcut if E.wcut is not None else sp_.get('cut')
    if dc_ is not None:
        plan['dest_cut'] = list(dc_) if isinstance(dc_, (list, tuple)) else dc_
    if sp_.get('entry'):
        plan['dest_entry'] = {l_: list(p_) for l_, p_ in sp_['entry'].items()}   # (a GROUP's ways in: Group)
    scut = sp_.get('scut')
    if scut is not None:
        plan['src_cut'] = float(scut)       # (the source's far face's cut, as the model read the state)
    return plan, tracks, [nm for nm in st['launch'] if nm in plan['ends']]


def solve_check(E, state, front=None, crowd=True, exact=False):
    """THE WHOLE SOLVE'S OWN VERDICT on a state's ends, before any is laid: the bench the state makes
    (whole_ctx.candidate: the board as it stands, each end's escape run and via added in memory, the plan sidecar's
    keys from the state at the ends model's cut) given to whole_solve.solve under its strict rules (no relaxation) -- the same
    frame, spans, end rooms and crossing windows the round's solve will hold the plan to, which the ends model's own
    route (exact_route_nl: the orders alone) does not see. On the zynq LVDS bus's escapes the round's first solve had no
    plan in 21 of 29 rounds where that route found one: lanes leaving the source's near face enter the trunk a millimetre
    or two before the rings leave it, and a pair's end rooms leave no place there for the change its crossings force.
    {'plan': bool, 'crowded': [lanes the CROWD diagnosis names when there is none]}; None when it cannot be asked (a plan
    in the turned-over pair frame). `exact` (the joint loop's): with no plan, the LANES whose
    rules cannot hold together (a second solve, every rule of a lane on one assumption, the core made minimal) and the
    lanes of the crossings whose windows either solve's core named, and 'nochange' -- the lanes no change fits in on
    these ends -- in place of the rules and crowd diagnoses"""
    st = E.st
    import whole_ctx
    import whole_solve
    plan, tracks, names = state_plan(E, state)
    # (the verdict on a bench asked before -- the same ends, layers, runs, vias, cut and ways -- is the one it had: the
    # check is deterministic, and every destination pass's re-plan asks its ends again, minutes each)
    mkey = (st.get('base'), json.dumps(plan, sort_keys=True, default=str), json.dumps(sorted(tracks, key=str), default=str),
            json.dumps(front or {}, sort_keys=True, default=str), crowd, exact)
    if mkey in _CHECK_MEMO:
        return _CHECK_MEMO[mkey]
    out = _solve_check(E, st, plan, tracks, names, front, crowd, exact)
    _CHECK_MEMO[mkey] = out
    return out


_CHECK_MEMO = {}


def _solve_check(E, st, plan, tracks, names, front, crowd, exact):
    """solve_check's verdict on the bench of `plan` and `tracks` (state_plan's)"""
    import whole_ctx
    import whole_solve
    if getattr(E, '_check_cfg', None) is None:
        import braid as bd
        E._check_cfg = bd.run_config(st['pcb'], st.get('base'))
    # (a GROUP's: the whole group, the frame's one destination -- whole_frame.Group)
    dest_ = '+'.join(st['dests']) if len(st.get('dests') or ()) > 1 else st['dref']
    ctx = whole_ctx.candidate(st['pcb'], names, dest_, plan, tracks, cfg=E._check_cfg, board=st.get('base'))
    if ctx is None:
        return None
    rules_ = (awx_settings.get('ENDS_CHECK_RULES') or '1') == '1'
    with contextlib.redirect_stdout(io.StringIO()):
        J = whole_solve.solve(ctx, dest_, front=front or {}, check=True, core=crowd or exact)
        out = {'plan': bool(J and J.get('plan')), 'status': (J or {}).get('status'), 'crowded': [],
               'core': [tuple(k_) for k_ in (J or {}).get('core') or []], 'rules': []}
        if exact:
            out['lanes'] = []
            out['nochange'] = list((J or {}).get('nochange') or [])
            if not out['plan'] and out['status'] == 'INFEASIBLE':
                Jl = whole_solve.solve(ctx, dest_, front=front or {}, check=True, core=True, rules=True,
                                       lanecore=True)
                # (the lanes solve's own crossing windows too: where the first check's core is empty -- its presolve
                # proved the model infeasible with no assumption in the proof -- the crossings it names are the only
                # reason to learn; left out, a refused state taught nothing and the loop stopped)
                jc_ = [tuple(k_) for k_ in (Jl or {}).get('core') or []]
                if not out['core']:
                    out['core'] = jc_
                out['lanes'] = sorted({r_[0] for r_ in (Jl or {}).get('rules') or []}
                                      | {l_ for k_ in out['core'] + jc_ for l_ in k_})
            return out
        if rules_ and crowd and not out['plan'] and not out['core'] and out['status'] == 'INFEASIBLE':
            # (the RULES core only where the windows' names nothing: it costs a solve per rule to make minimal, and
            # a core of crossings the repair already prices -- zynq LVDS K47, 15 crossings: a plan at its first
            # search again in 180 s, by the rules in 460)
            Jr = whole_solve.solve(ctx, dest_, front=front or {}, check=True, core=True, rules=True)
            out['rules'] = [tuple(k_) for k_ in (Jr or {}).get('rules') or []]
        if not out['plan'] and crowd and not out['rules'] and out['status'] == 'INFEASIBLE':
            # (a check that found no plan in its work proves nothing to diagnose: the round's solve has the time)
            Jc = whole_solve.solve(ctx, dest_, front=front or {}, crowd=True, check=True)
            out['crowded'] = list((Jc or {}).get('crowded') or [])
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
    from coherent_nets import namer
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
        sn = namer(pcb)
        drop_s = ({sn(n) for n in ar.get('drops', ())} - {sn(n) for n in ar.get('others', ())} - run)
        items = {f'{sn(p.net_name)}#{p.pad_number}': (sn(p.net_name), p)
                 for p in foot.pads if p.net_id and p.net_name and sn(p.net_name) in drop_s}
        if not items:
            continue
        grid = em.grid_of(foot)
        sz = je._sizes(pcb0, foot)
        skip = je.movable_refs(pcb0, ref, run)
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
                # (its ring: joint_escape's own measure of a drop near a lane's way out, exit_ray_conflicts')
                ways.append(dict(end=end, site=tuple(d.site), r=d.r, dr=d.dr, stub=d.stub, layer=d.layer,
                                 ring=je._ring(tuple(d.site), d.r)))
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
            if ray is not None and _seg_d(s, s, ray[0], ray[1]) - w['ring'] < bar - 1e-9:
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
                # (from just short of half a pitch inside the exits: the outer row's balls stand ON that line, and the
                # joint escape's pocket, from the laid copper's tips a hair out of the face, leaves them out -- the
                # zynq six-rail DDR3's GND B1 between DDR3_DQS0's teeth, its via in its pad laid, read walled in by
                # 0.0003 mm here and its ends priced past any via count)
                pk = None
                if len(lg) == 2 and o[0][0].direction == o[0][1].direction and o[0][0].layer == o[0][1].layer \
                        and DIRS.get(o[0][0].direction) is not None:
                    pk = pt.pockets({0: (0, 1)}, {0: (o[0][0].exit_pt, DIRS[o[0][0].direction], o[0][0].layer),
                                                  1: (o[0][1].exit_pt, DIRS[o[0][1].direction], o[0][1].layer)},
                                    at[end]['half'] - POCKET_ROW)[0]

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


def fb_tables(fb, T, B):
    """(avoid {(lane, end): {option: weight}}, pairs [(lane, lane, end, weights, weights)]): the feedback `fb`
    (whole_feedback's) on the lanes' tooth options `T` and berth options `B` ({lane: options}), each item weighed by
    place (fb_weights), a pair of items when both name options of one end. An item for a lane with no options here --
    its ends chosen again on a field where the lane had none: sdi_mipi_conv's SDI_D13, laid in round 1 from the joint
    field's escape, no tooth move on round 2's dog-boned field -- prices nothing and keys nothing (a key the state has
    no lane for stopped the score)"""
    avoid, pairs = {}, []

    def matches(it):
        if it['lane'] not in T:
            return {}
        return fb_weights(T[it['lane']] if it['end'] == 0 else B[it['lane']], it)
    for it in fb.get('avoid', ()):
        if it['lane'] not in T:
            continue
        w_ = avoid.setdefault((it['lane'], it['end']), {})
        for i, w in matches(it).items():
            w_[i] = w_.get(i, 0.0) + w
    for a_, b_ in fb.get('pairs', ()):
        ma, mb = matches(a_), matches(b_)
        if ma and mb and a_['end'] == b_['end']:
            pairs.append((a_['lane'], b_['lane'], a_['end'], ma, mb))
    return avoid, pairs


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



def judge(st, choice):
    """(objective, parts) of berths `choice` {net: Move} against the teeth as laid -- a net `choice` leaves out at
    its greedy berth"""
    import source_realize as sr
    E = Ends(st, src_free=False, fixed={nm: sr.move_sig(m) for nm, m in choice.items()})
    # (a GROUP's ways in: the ones the ends model chose for these berths, as its last choice carries them -- its
    # objective prices a leg along a pocket, which the crossings alone do not -- the rest arranged)
    s0 = E.arrange_ways(E.start(), prefer=((getattr(choose, 'last', None) or {}).get('parts') or {}).get('entry'))
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
            + (f', {len(p["corner"])} lane(s) the frame cannot lay round the source\'s corner ('
               + ', '.join(p['corner']) + ')' if p.get('corner') else '')
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

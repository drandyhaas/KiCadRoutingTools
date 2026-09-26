"""whole_solve.py OUT.json -- the WHOLE-ROUTE crossing and layer solve (CP-SAT), one permutation: the launch order
-> the berths' order round the destination's pad box (unrolled from a cut on its far face between the branches).
Each lane's route is ONE
coordinate u: trunk s from its entry (tooth / join leg) to the handoff, then its ring (u = handoff + s_b - ring
start) to its berth leg; a head lane's trunk s to its berth. Every inverted pair crosses once, at a u both share
(a pair in one branch may cross on the ring, up to the earlier leg); the braid triple rule over ALL triples;
each lane's crossings a pitch apart along its track (a stayer), or less for a sweep (a mover); up to
KMAX changes per lane, each a via's half-room from its own crossings and a via's room apart; tooth layer at the
start, berth layer at the end. Objective: vias first, then CONGESTION (the copper area packed crossings and
vias add, priced by how full the lane's stretch of route already is).

The bench from BENCH / NETS / DEST (whole_ctx). HINT=SOLVE.json warm-starts from an earlier solve; CUTS=GEO.json,..
adds the geometry's cuts (whole_geo: the islands a lane could not be kept off, the changes it could not give their
room). The solve is bounded in WORK, not time: a count of CP-SAT's interleaved batches, its workers pinned and
sharing no clauses, so the same model gives the same answer on every run, later on a slower machine.
WHOLE_SOLVE_BATCHES sets the budget."""
import sys, os, itertools, collections, json, math
import numpy as np
import whole_ctx
from ortools.sat.python import cp_model
import braid as bd
import pairs as _pairs
# every length below is in the design rules' own units: track, clearance, via size, a via's room, the lane pitch
TRK, CLR, VIA, VNEED, PITCH = bd.TRACK, bd.CLEAR, bd.VIA_SIZE, bd.VIA_NEED, bd.LANE_MIN
OUT = sys.argv[1] if len(sys.argv) > 1 else '/dev/null'
LEGROOM = PITCH                        # a peeling leg crosses, then still runs a pitch to its berth
VR_STAY, VR_MOVE = 1.1, 1.0            # a change's room from a crossing: a stayer's via is passed at an angle
KMAX = 4                               # layer changes per lane at most
G = PITCH / 5                          # the solve's time grid: a fifth of the lane pitch
MARG = CLR                             # a crossing starts a clearance past both lanes' terminals (a lane leaving its tooth may cross at once)
DELTA = PITCH
# a crossing's room along each lane: a STAYER's crossings a pitch apart, a MOVER's (a sweep crossing a bundle nearly
# across the spine, its slope up to K_SWEEP) DELTA / sqrt(1 + K^2) apart along s; the geometry keeps the real pitch
K_SWEEP = 4.0
W_V = 1e6                              # per via: vias first
W_CONG = 1.0                           # congestion: copper area (extra track from packed crossings + via patches) x local price
LB = 2 * PITCH                         # congestion bin: two lane pitches of route
K_REACH = 1.0                          # a lane reaches a stretch of route from its terminals at this slope
SOLVE_BATCHES = int(os.environ.get('WHOLE_SOLVE_BATCHES', '100'))  # CP-SAT interleaved batches: the work budget
SOLVE_WORKERS = 4
ctx, cs = whole_ctx.plan()
c = cs[0]
prs = getattr(ctx, 'pairs', {}) or {}
M = list(c.members); bo = c.branch_of or {}
F = lambda x: 1 if x == 'B.Cu' else 0
tl = {n: F(ctx.tooth_layer[n]) for n in M}
dl = {n: F(ctx.dest_layer[n]) for n in M}
Dv = {n: 2 * VNEED + (_pairs.pitch(TRK) if n in prs else 0.0) for n in M}     # a change's room along its lane
Q = lambda s: int(round(s / G))
QU = lambda s: int(math.ceil(s / G - 1e-9))
# ---- the whole route of each lane in u. A lane's CLASS is the face of the destination's pad box its berth lies on:
# N (north face, or the east face above the cut) rides the north ring, S the south ring, W ends in the trunk.
HAND = {n: c.se[n][0] for n in M if n in bo}
H0 = max(HAND.values())
brs = {id(v): v for v in bo.values()}
ring_of = {}
for b in brs.values():
    ring_of['N' if b.spine.pts[0][1] < c.spine.xy(H0, 0.0)[1] else 'S'] = b
DST_REF = os.environ['DEST']
SRC_REF = collections.Counter(ctx.src_ref[n] for n in M).most_common(1)[0][0]
fp = ctx.pcb.footprints[DST_REF]
xs_ = [p.global_x for p in fp.pads]; ys_ = [p.global_y for p in fp.pads]
bend = {n: tuple(map(float, c.lane_xy[n][-1])) for n in M}
_d = [max(min(xs_) - x, x - max(xs_), min(ys_) - y, y - max(ys_)) for (x, y) in bend.values()]
_mg = float(np.median([d for d in _d if d > 0])) if any(d > 0 for d in _d) else 0.0     # where the berths sit: read off the board
x0, y0, x1, y1 = min(xs_) - _mg, min(ys_) - _mg, max(xs_) + _mg, max(ys_) + _mg
face = {}
for n in M:
    x, y = bend[n]
    d = {'E': abs(x - x1), 'N': abs(y - y0), 'W': abs(x - x0), 'S': abs(y - y1)}
    face[n] = min(d, key=d.get)
ey = sorted(bend[n][1] for n in M if face[n] == 'E')
# the cut on the east face: the widest gap between east-face berths
if len(ey) >= 2:
    gi = max(range(len(ey) - 1), key=lambda i: ey[i + 1] - ey[i])
    ycut = (ey[gi] + ey[gi + 1]) / 2
else:
    ycut = (y0 + y1) / 2
bname = {}
for n in M:
    f_ = face[n]
    if f_ == 'E':
        f_ = 'N' if bend[n][1] <= ycut else 'S'
    if f_ in ('N', 'S'):
        bname[n] = f_
# a ring's route coordinate starts where the geometry starts it: ahead of every class lane's handoff point, on a
# geometry column (two solve steps) -- ONE origin for the solve and the layout
GG = 2 * G
rs = {}
for k_, b_ in ring_of.items():
    sbs = []
    for n in M:
        if bname.get(n) != k_:
            continue
        ms_ = np.array([p_[0] for p_ in c.mid[n]]); mo_ = np.array([p_[1] for p_ in c.mid[n]])
        sbs.append(b_.spine.project_pt(c.spine.xy(H0, float(np.interp(H0, ms_, mo_))))[0])
    rs[k_] = math.ceil(max(sbs) / GG - 1e-9) * GG
entry = {n: (c.join_leg_s[n] if n in c.join_block else c.st[n][0]) for n in M}
def u_ring(n, s_b):
    return H0 + (s_b - rs[bname[n]])
end = {}
for n in M:
    if n in bname:
        end[n] = u_ring(n, ring_of[bname[n]].spine.project_pt(bend[n])[0])
    else:
        end[n] = c.se[n][0]
tend = {n: (H0 if n in bname else c.se[n][0]) for n in M}      # where the lane leaves the TRUNK frame
VW = VIA / 2 + TRK / 2                                               # a change's room from its lane's terminals
# a PAIR's end: no CROSSING inside its END CONNECTOR (pairs.end_connector: the legs from its tips to the pose where the
# pair router takes over; SDQS1 once crossed SDQ15 and SDQ13 in the last 0.3 mm before their berths), and no CHANGE of
# its own nearer than its DIVE ROOM (pairs.dive_room: that connector, then the router's straight from the pose into the
# via). A crossing lane is on the other layer there; only the pair's own dive has to stand beyond its pose
RIN0 = {n: (_pairs.end_connector(ctx.cfg, ctx.pair_ends[n][0]) if n in prs else 0.0) for n in M}   # at the tooth end
RIN1 = {n: (_pairs.end_connector(ctx.cfg, ctx.pair_ends[n][1]) if n in prs else 0.0) for n in M}   # ... the berth end
_axis = lambda u: u if u is not None else (1.0, 0.0)
VIN0 = {n: (_pairs.dive_room(ctx.cfg, ctx.pair_ends[n][0], _axis(ctx.tooth_dir.get(n))) if n in prs else VW) for n in M}
VIN1 = {n: (_pairs.dive_room(ctx.cfg, ctx.pair_ends[n][1], _axis(ctx.stub_dir.get(n))) if n in prs else VW) for n in M}
# a PAIR's KNOWN TURNS: the pair router neither turns at its via nor within its straight run of one, so its changes
# stay out of every stretch of its route where the frames already turn (below, as built-in via cuts) -- and, where an
# end's stub stands more than its connector's 45 degrees off the route's own way there, beyond the turn onto that way
# as well. A turn is one the router must make: half a router step or more. The rest -- a lane's own sweep onto its
# berth -- only the geometry knows, and it and the polish send those (whole_geo / whole_polish vcuts)
TURN_DEG = 22.5
L_DIVE = _pairs.via_straight(ctx.cfg, (math.sqrt(0.5), math.sqrt(0.5))) + ctx.cfg.grid_step   # the longer (diagonal) run


def turn_room(deg):
    """half the stretch a pair's turn of `deg` takes: its 45-degree turns, a turning radius's straight run apart"""
    return _pairs.turn_straight_steps(ctx.cfg) * ctx.cfg.grid_step * max(0, math.ceil(abs(deg) / 45.0 - 1e-9) - 1) / 2


def _deg(a, b):
    return math.degrees(math.acos(max(-1.0, min(1.0, (a[0] * b[0] + a[1] * b[1]) / (math.hypot(*a) * math.hypot(*b))))))


def route_dir(n, u):
    """the unit way lane n's route runs at u: its trunk's spine, or past the handoff its ring's"""
    if n in bname and u > H0:
        sp_ = ring_of[bname[n]].spine
        return tuple(sp_.d[sp_.seg_of(u - H0 + rs[bname[n]])])
    return tuple(c.spine.d[c.spine.seg_of(u)])


for n in prs:
    if n not in M:
        continue
    for k_, (u_e, esc, sg) in enumerate(((entry[n], ctx.tooth_dir.get(n), 1.0), (end[n], ctx.stub_dir.get(n), -1.0))):
        if esc is None:
            continue
        rd = route_dir(n, u_e)
        th = _deg((esc[0] * sg, esc[1] * sg), rd) - 45.0         # what the connector's 45 degrees leave to turn
        if th >= TURN_DEG:
            if k_ == 0:
                VIN0[n] += 2 * turn_room(th) + L_DIVE
            else:
                VIN1[n] += 2 * turn_room(th) + L_DIVE
print('classes:', dict(collections.Counter(bname.get(n, 'W') for n in M)), 'W ends', sorted(round(end[n], 2) for n in M if n not in bname))
W_, H_ = x1 - x0, y1 - y0
def perim(p):
    x, y = p
    d = {'E': abs(x - x1), 'N': abs(y - y0), 'W': abs(x - x0), 'S': abs(y - y1)}
    sd = min(d, key=d.get)
    if sd == 'E' and y <= ycut: return ycut - y
    if sd == 'N': return (ycut - y0) + (x1 - x)
    if sd == 'W': return (ycut - y0) + W_ + (y - y0)
    if sd == 'S': return (ycut - y0) + W_ + H_ + (x - x0)
    return (ycut - y0) + 2 * W_ + H_ + (y1 - y)
P = {n: perim(bend[n]) for n in M}
Ln = sorted(M, key=lambda n: c.launch_o[n]); li = {n: i for i, n in enumerate(Ln)}
Fn = sorted(M, key=lambda n: P[n]); fi = {n: i for i, n in enumerate(Fn)}
inv = lambda a, b: (li[a] < li[b]) == (fi[a] > fi[b])
pairs = [(a, b) for a, b in itertools.combinations(Ln, 2) if inv(a, b)]
same = lambda a, b: a in bname and b in bname and bname[a] == bname[b]
win = {}
for a, b in pairs:
    lo = max(entry[a] + max(MARG, RIN0[a]), entry[b] + max(MARG, RIN0[b]))
    # a crossing on the ring leaves the earlier lane's leg LEGROOM to reach its berth after it
    hi = min(end[a] - max(LEGROOM, RIN1[a]), end[b] - max(LEGROOM, RIN1[b])) if same(a, b) else \
        min(tend[a] - (max(MARG, RIN1[a]) if a not in bname else MARG), tend[b] - (max(MARG, RIN1[b]) if b not in bname else MARG))
    win[(a, b)] = (lo, max(hi, lo + G))
m = cp_model.CpModel()
t = {k: m.NewIntVar(Q(lo), Q(hi), f't_{k[0]}_{k[1]}') for k, (lo, hi) in win.items()}
# ---- geometry cuts (whole_geo.py's islands a lane could not be kept off): none of that lane's crossings in the span
CUTS = []
for fn_ in [x for x in os.environ.get('CUTS', '').split(',') if x]:
    CUTS += json.load(open(fn_)).get('cuts', [])
ncut = 0
for cu_ in sorted({(c_['lane'], round(c_['u_lo'], 3), round(c_['u_hi'], 3)) for c_ in CUTS}):
    n_, lo_, hi_ = cu_
    for key in t:
        if n_ not in key:
            continue
        a_ = m.NewBoolVar('')
        m.Add(t[key] <= Q(lo_)).OnlyEnforceIf(a_); m.Add(t[key] >= Q(hi_) + 1).OnlyEnforceIf(a_.Not())
        ncut += 1
if CUTS:
    print(f'   geometry cuts: {len(CUTS)} island spans, {ncut} crossing constraints')
VCUTS = []
for fn_ in [x for x in os.environ.get('CUTS', '').split(',') if x]:
    VCUTS += json.load(open(fn_)).get('vcuts', [])
# ...and a pair's built-in via cuts, at its route's known turns: its trunk's spine corners, its ring's (the pad box's
# corners) and the handoff from the one onto the other
NVC0 = len(VCUTS)
for n in prs:
    if n not in M:
        continue
    turns = [(s_, d_) for _i, s_, d_ in c.spine.corners(TURN_DEG) if entry[n] < s_ < tend[n]]
    if n in bname:
        sp_ = ring_of[bname[n]].spine
        turns += [(u_, d_) for _i, sb_, d_ in sp_.corners(TURN_DEG) for u_ in [u_ring(n, sb_)] if H0 < u_ < end[n]]
        dh = _deg(route_dir(n, H0 - G), route_dir(n, H0 + G))
        if dh >= TURN_DEG and entry[n] < H0 < end[n]:
            turns.append((H0, dh))
    VCUTS += [{'lane': n, 'u': u_, 'w': L_DIVE + turn_room(d_)} for u_, d_ in turns]
if len(VCUTS) > NVC0:
    print(f'   built-in via cuts at the pairs\' known turns: {len(VCUTS) - NVC0}')
# ---- the braid rule over every triple
nt = 0
for i, j, k in itertools.combinations(Ln, 3):
    ij, ik, jk = (i, j) in t, (i, k) in t, (j, k) in t
    if ij and ik and jk:
        b1 = m.NewBoolVar('')
        m.Add(t[(i, j)] < t[(i, k)]).OnlyEnforceIf(b1); m.Add(t[(i, k)] < t[(j, k)]).OnlyEnforceIf(b1)
        m.Add(t[(j, k)] < t[(i, k)]).OnlyEnforceIf(b1.Not()); m.Add(t[(i, k)] < t[(i, j)]).OnlyEnforceIf(b1.Not())
        nt += 1
    elif ij and ik:
        m.Add(t[(i, j)] < t[(i, k)]); nt += 1
    elif ik and jk:
        m.Add(t[(j, k)] < t[(i, k)]); nt += 1
    elif ij and jk:
        raise SystemExit(f'inconsistent triple {i} {j} {k}')
ev = collections.defaultdict(list)
for key in t:
    for n in key: ev[n].append(key)
P_STAY = PITCH                        # along a STAYER two crossings sit a pitch apart (its crossers are parallel there)
MV = {}
if True:                              # (the mover / stayer model)
    # every crossing has a MOVER (the steep lane, crossing over) and a STAYER; along a stayer its crossings are
    # P_STAY apart in s, along a mover DELTA / sqrt(1 + K_SWEEP^2) (a sweep): the mover chosen by the solve
    ivs_of = collections.defaultdict(list)
    w_move, w_stay = max(1, QU(PITCH / math.sqrt(1 + K_SWEEP ** 2))), max(1, QU(P_STAY))
    for key in t:
        a, b = key
        mv = m.NewBoolVar('')                           # True: a moves, b stays
        MV[key] = mv
        ivs_of[a].append(m.NewOptionalFixedSizeIntervalVar(t[key], w_move, mv, ''))
        ivs_of[a].append(m.NewOptionalFixedSizeIntervalVar(t[key], w_stay, mv.Not(), ''))
        ivs_of[b].append(m.NewOptionalFixedSizeIntervalVar(t[key], w_move, mv.Not(), ''))
        ivs_of[b].append(m.NewOptionalFixedSizeIntervalVar(t[key], w_stay, mv, ''))
    for n, ivs in ivs_of.items():
        m.AddNoOverlap(ivs)
# ---- layer changes
cost = []
chg, tot = {}, {}
for n in M:
    lo_n, hi_n = Q(entry[n] + max(VW, VIN0[n])), Q(end[n] - max(VW, VIN1[n]))
    cs_ = [m.NewIntVar(lo_n, hi_n + 1, f'c_{n}_{k}') for k in range(KMAX)]
    act = [m.NewBoolVar('') for _ in range(KMAX)]
    for k in range(KMAX):
        m.Add(cs_[k] <= hi_n).OnlyEnforceIf(act[k]); m.Add(cs_[k] == hi_n + 1).OnlyEnforceIf(act[k].Not())
        if k:
            m.Add(cs_[k] >= cs_[k - 1] + Q(Dv[n])).OnlyEnforceIf(act[k]); m.AddImplication(act[k], act[k - 1])
    # a change's room from each of its lane's crossings, along s: a STAYER's via is passed by a steep mover at an
    # angle (VR_STAY x the room), a MOVER's via sits on its own steep track (VR_MOVE x)
    h_st, h_mv = QU(VR_STAY * Dv[n] / 2), QU(VR_MOVE * Dv[n] / 2)      # a room rounds UP to the grid
    before = {}
    for key in ev[n]:
        if key in MV:
            stay = MV[key].Not() if key[0] == n else MV[key]
        else:
            stay = None
        bits = []
        for k in range(KMAX):
            bb = m.NewBoolVar('')
            if stay is None:
                m.Add(cs_[k] + h_st <= t[key]).OnlyEnforceIf(bb)
                m.Add(cs_[k] - h_st >= t[key]).OnlyEnforceIf([bb.Not(), act[k]])
            else:
                for h_, lit in ((h_st, stay), (h_mv, stay.Not())):
                    m.Add(cs_[k] + h_ <= t[key]).OnlyEnforceIf([bb, lit])
                    m.Add(cs_[k] - h_ >= t[key]).OnlyEnforceIf([bb.Not(), act[k], lit])
            m.AddImplication(bb, act[k]); bits.append(bb)
        before[key] = bits
    m.AddBoolXOr(act + ([m.NewConstant(1)] if tl[n] == dl[n] else []))
    tot[n] = sum(act); chg[n] = (cs_, act)
    ev[n] = before
STAGGER = VIA                          # two lanes' changes a via apart along one frame
if STAGGER > 0:
    fr_ivs = collections.defaultdict(list)
    w_s = max(1, Q(STAGGER))
    for n in M:
        cs_, act = chg[n]
        for x, a_ in zip(cs_, act):
            if n in bname:
                # its frame is the trunk before the handoff, its ring after
                inT, inR = m.NewBoolVar(''), m.NewBoolVar('')
                m.Add(x <= Q(H0)).OnlyEnforceIf(inT); m.Add(x > Q(H0)).OnlyEnforceIf(inR)
                m.AddBoolOr([inT.Not(), a_]); m.AddBoolOr([inR.Not(), a_])
                m.Add(inT + inR == 1).OnlyEnforceIf(a_); m.Add(inT + inR == 0).OnlyEnforceIf(a_.Not())
                fr_ivs['T'].append(m.NewOptionalFixedSizeIntervalVar(x, w_s, inT, ''))
                fr_ivs[bname[n]].append(m.NewOptionalFixedSizeIntervalVar(x, w_s, inR, ''))
            else:
                fr_ivs['T'].append(m.NewOptionalFixedSizeIntervalVar(x, w_s, a_, ''))
    for ivs in fr_ivs.values():
        m.AddNoOverlap(ivs)
for key in t:
    a, b = key
    # crossing lanes DIFFER: tl_a ^ tl_b ^ Ca ^ Cb == 1, i.e. XOR(parity bits [+ 1 when the teeth differ]) == 1
    lits = ev[a][key] + ev[b][key]
    if tl[a] ^ tl[b] == 1: lits = lits + [m.NewConstant(1)]
    m.AddBoolXOr(lits)
# ---- via cuts (whole_geo.py: a change the geometry could not give its room): that lane's changes stay out of the window
for vc_ in sorted({(c_['lane'], round(c_['u'], 3), round(c_['w'], 3)) for c_ in VCUTS}):
    n_, u_, w_ = vc_
    if n_ not in chg:
        continue
    cs_v, act_v = chg[n_]
    for x_, a_ in zip(cs_v, act_v):
        lo_b, hi_b = m.NewBoolVar(''), m.NewBoolVar('')
        m.Add(x_ <= Q(u_ - w_)).OnlyEnforceIf(lo_b); m.Add(x_ >= Q(u_ + w_)).OnlyEnforceIf(hi_b)
        m.AddBoolOr([lo_b, hi_b, a_.Not()])
if VCUTS:
    print(f'   via cuts: {len(VCUTS)}')
# ---- congestion
CONG = {}
if W_CONG:
    P_L, A_V = bd.LANE_MIN, 2 * (2 * bd.VIA_NEED) ** 2
    bx = []
    for r_, pts_ in ((SRC_REF, [ctx.ends[n][0] for n in M]), (DST_REF, [ctx.ends[n][1] for n in M])):
        f_ = ctx.pcb.footprints[r_]
        xs2 = [p_.global_x for p_ in f_.pads]; ys2 = [p_.global_y for p_ in f_.pads]
        dd = [max(min(xs2) - x, x - max(xs2), min(ys2) - y, y - max(ys2)) for (x, y) in pts_]
        mg_ = float(np.median([d for d in dd if d > 0])) if any(d > 0 for d in dd) else 0.0    # where the stubs end
        bx.append((min(xs2) - mg_, min(ys2) - mg_, max(xs2) + mg_, max(ys2) + mg_))
    BX0, BY0, BX1, BY1 = ctx.pcb.board_info.board_bounds
    EDGE = (float(getattr(ctx.cfg, 'board_edge_clearance', 0.0) or 0.0) or ctx.cfg.clearance) + TRK / 2
    def usable(x, y):
        if not (BX0 + EDGE <= x <= BX1 - EDGE and BY0 + EDGE <= y <= BY1 - EDGE): return False
        return not any(b_[0] <= x <= b_[2] and b_[1] <= y <= b_[3] for b_ in bx)
    _span = max(BX1 - BX0, BY1 - BY0)
    OS = np.arange(-_span, _span + 1e-9, G)
    def width(sp_, s_, piece):
        ok = np.array([usable(*map(float, sp_.xy(s_, o_))) for o_ in OS])
        if not piece: return G * ok.sum()
        i0 = int(np.argmin(np.abs(OS)))
        if not ok[i0]: return G * ok.sum()
        lo_, hi_ = i0, i0
        while lo_ > 0 and ok[lo_ - 1]: lo_ -= 1
        while hi_ < len(OS) - 1 and ok[hi_ + 1]: hi_ += 1
        return G * (hi_ - lo_ + 1)
    brobj = ring_of
    TOOTH = {n: c.st[n] for n in M}                      # (s, o) in the trunk frame
    BERTH = {n: {k: ring_of[k].spine.project_pt(bend[n]) for k in ring_of} for n in M}
    DB = (x0, y0, x1, y1)                                  # the destination's pad box grown to its berths
    _outer = {}
    def box_o(sp_, s_):
        """the offset where this ring's normal line meets the destination's box, or None"""
        hits = [o_ for o_ in OS if DB[0] <= sp_.xy(s_, o_)[0] <= DB[2] and DB[1] <= sp_.xy(s_, o_)[1] <= DB[3]]
        if not hits:
            return None
        return min(hits, key=abs)
    def ring_outer(sp_, s_, fr):
        key = (fr, round(s_, 3))
        if key not in _outer:
            ob = box_o(sp_, s_)
            if ob is None:                                  # before the box: its edge extended back along the ring
                s2 = s_
                while ob is None and s2 < sp_.L:
                    s2 += PITCH
                    ob = box_o(sp_, s2)
            cx, cy = (DB[0] + DB[2]) / 2, (DB[1] + DB[3]) / 2
            side = np.sign(sp_.project_pt((cx, cy))[1]) or 1.0
            _outer[key] = (OS * side < ob * side) if ob is not None else np.ones(len(OS), dtype=bool)
        return _outer[key]
    def reach_width(sp_, s_, pres, fr):
        """usable length of the o-line at s_ that some lane present can reach from its terminals at K_REACH"""
        ok = np.array([usable(*map(float, sp_.xy(s_, o_))) for o_ in OS])
        if not pres:
            return G * ok.sum()
        reach = np.zeros(len(OS), dtype=bool)
        for n in pres:
            lo_, hi_ = -math.inf, math.inf
            if fr == 'T':
                st_s, st_o = TOOTH[n]
                d = max(0.0, s_ - st_s)
                lo_, hi_ = st_o - K_REACH * d - PITCH, st_o + K_REACH * d + PITCH
                if n not in bname:                          # a west-face lane ends at its berth in this frame
                    e_s, e_o = c.se[n]
                    d2 = max(0.0, e_s - s_)
                    lo_, hi_ = max(lo_, e_o - K_REACH * d2 - PITCH), min(hi_, e_o + K_REACH * d2 + PITCH)
            else:
                e_s, e_o = BERTH[n][fr]
                d2 = max(0.0, e_s - s_)
                lo_, hi_ = e_o - K_REACH * d2 - PITCH, e_o + K_REACH * d2 + PITCH
            reach |= (OS >= lo_) & (OS <= hi_)
        if fr != 'T':
            ok &= ring_outer(sp_, s_, fr)
        return G * (ok & reach).sum()
    # frame of a point on the route: ('T', k) trunk, (branch, k) ring; k = floor((u - H0) / LB)
    kof = lambda u: int(math.floor((u - H0) / LB + 1e-9))
    bins = {}
    def price(fr, k):
        if (fr, k) in bins: return bins[(fr, k)][2]
        u_c = H0 + (k + 0.5) * LB
        if fr == 'T':
            pres = [n for n in M if entry[n] <= u_c <= tend[n]]
            w_ = reach_width(c.spine, u_c, pres, 'T')
        else:
            pres = [n for n in M if bname.get(n) == fr and H0 <= u_c <= end[n]]
            w_ = reach_width(brobj[fr].spine, u_c - H0 + rs[fr], pres, fr)
        npres = len(pres)
        base = npres * LB * P_L
        rho = min(0.9, base / max(2 * w_ * LB, 1e-6))
        bins[(fr, k)] = (npres, w_, 1.0 / (1.0 - rho), base, 2 * w_ * LB)
        return bins[(fr, k)][2]
    SC = 1000.0 * W_CONG
    ext = lambda n_: LB * (math.sqrt(1 + (n_ * P_L / LB) ** 2) - 1)     # extra track length, n crossings in LB
    NMAX = 10
    cnt = collections.defaultdict(list)                               # (lane, frame, k) -> membership bools
    xmem = {}
    for key, (lo, hi) in win.items():
        a, b = key
        ks = []
        for k in range(kof(lo), kof(hi) + 1):
            u0, u1 = H0 + k * LB, H0 + (k + 1) * LB
            fr = (bname[a] if (same(a, b) and u0 >= H0 - 1e-9) else 'T')
            ks.append((fr, k, u0, u1))
        xs3 = []
        for fr, k, u0, u1 in ks:
            x = m.NewBoolVar('')
            m.Add(t[key] >= Q(u0)).OnlyEnforceIf(x); m.Add(t[key] < Q(u1)).OnlyEnforceIf(x)
            xs3.append(x)
            for n in key: cnt[(n, fr, k)].append(x)
        m.AddExactlyOne(xs3)
        xmem[key] = list(zip(ks, xs3))
    for (n, fr, k), xs3 in cnt.items():
        pr = price(fr, k)
        nv = m.NewIntVar(0, NMAX, '')
        m.Add(nv == sum(xs3))
        cv = m.NewIntVar(0, 10 ** 9, '')
        m.AddElement(nv, [int(round(SC * P_L * ext(j) * pr)) for j in range(NMAX + 1)], cv)
        cost.append(cv)
    for n in M:
        cs_, act = chg[n]
        for k_, (cv_, a_) in enumerate(zip(cs_, act)):
            xs4 = []
            for k in range(kof(entry[n]), kof(end[n]) + 1):
                u0, u1 = H0 + k * LB, H0 + (k + 1) * LB
                fr = bname[n] if (n in bname and u0 >= H0 - 1e-9) else 'T'
                x = m.NewBoolVar('')
                m.Add(cv_ >= Q(u0)).OnlyEnforceIf(x); m.Add(cv_ < Q(u1)).OnlyEnforceIf(x)
                cost.append(int(round(SC * A_V * (2 if n in prs else 1) * price(fr, k))) * x)
                xs4.append(x)
            m.Add(sum(xs4) == a_)
    CONG = bins
if os.environ.get('HINT'):
    Hj = json.load(open(os.environ['HINT']))
    nh = 0
    for k_, v_ in Hj['cross'].items():
        a_, b_ = k_.split('|')
        key_ = (a_, b_) if (a_, b_) in t else (b_, a_)
        if key_ in t:
            m.AddHint(t[key_], int(round(v_['u'] / G))); nh += 1
    for n_, chs_ in Hj['changes'].items():
        cs_h, act_h = chg[n_]
        for i_ in range(KMAX):
            if i_ < len(chs_):
                m.AddHint(cs_h[i_], int(round(chs_[i_] / G))); m.AddHint(act_h[i_], 1)
            else:
                m.AddHint(act_h[i_], 0)
    print(f'   warm start from {os.path.basename(os.environ["HINT"])}: {nh} crossing hints')
# ---- no more than TWO VIAS on a net where that can be had (Andy, 2026-09-25): a net's vias on the board are its stubs'
# own (the bench's copper) and its lane's changes -- a pair's leg a barrel at each dive. The objective is
# lexicographic: first how far the nets go over two, then the vias, then congestion -- a preference, never a cap, so a
# board that cannot keep it still plans
VIA_PREF = 2
SV = {n: max(sum(1 for v in ctx.base_vias if v.net_id == ctx.byname[leg][0]) for leg in (prs[n] if n in prs else (n,)))
      for n in M}
over = {n: m.NewIntVar(0, KMAX + SV[n], f'over_{n}') for n in M}
for n in M:
    m.Add(over[n] >= SV[n] + tot[n] - VIA_PREF)
W_OVER = W_V * (KMAX * len(M) + 1)        # one via over two outweighs every via the plan could save
m.Minimize(W_OVER * sum(over.values()) + W_V * sum(tot.values()) + sum(cost))
sv = cp_model.CpSolver()
sv.parameters.num_workers = SOLVE_WORKERS
# REPRODUCIBLE: stopped by a count of interleaved batches, the workers sharing no clauses. Measured on this model
# (OR-tools 9.15): bounded by deterministic time, four solves of one model gave four answers (36051281 .. 36051642);
# by batches with clause sharing on, two gave two; by batches with sharing off, two concurrent solves agree exactly
sv.parameters.interleave_search = True
sv.parameters.max_num_deterministic_batches = SOLVE_BATCHES
sv.parameters.share_glue_clauses = False
sv.parameters.share_binary_clauses = False
st = sv.Solve(m)
print(f'whole_solve: {len(t)} crossings ({sum(1 for k in t if same(*k))} same-branch), {nt} triples, K<={KMAX}, '
      f'mover/stayer (stay {P_STAY}), stagger {STAGGER}, MARG {MARG}: [{sv.StatusName(st)}] {sv.WallTime():.0f}s', end=' ')
if st not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
    print(); sys.exit(1)
per = {n: int(sv.Value(tot[n])) for n in M}
ov = [n for n in M if SV[n] + per[n] > VIA_PREF]
print(f'vias {sum(per.values())}, nets over {VIA_PREF} vias on the board: {len(ov)} {ov}, changes per lane {dict(sorted(collections.Counter(per.values()).items()))}, obj {sv.ObjectiveValue():.0f} bound {sv.BestObjectiveBound():.0f}')
# verify: every crossing on two layers, every lane on its berth layer
lay = lambda n, u: tl[n] ^ (sum(1 for x, a_ in zip(*chg[n]) if sv.Value(a_) and sv.Value(x) * G < u) & 1)
bad = [(a, b) for (a, b), v in t.items() if lay(a, sv.Value(v) * G) == lay(b, sv.Value(v) * G)]
badb = [n for n in M if lay(n, 1e9) != dl[n]]
print(f'   check: crossings on one layer {len(bad)}, lanes off their berth layer {len(badb)}')
J = {'H0': H0, 'rs': rs, 'cross': {}, 'changes': {}, 'entry': entry, 'end': end, 'tend': tend, 'branch': bname,
     'final': Fn, 'launch': Ln}
for (a, b), v in t.items():
    J['cross'][f'{a}|{b}'] = {'u': sv.Value(v) * G, 'ring': sv.Value(v) * G > H0 + 1e-9 and same(a, b)}
for n in M:
    cs_, act = chg[n]
    J['changes'][n] = [sv.Value(x) * G for x, a_ in zip(cs_, act) if sv.Value(a_)]
json.dump(J, open(OUT, 'w'), indent=0)

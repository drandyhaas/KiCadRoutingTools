"""whole_geo.py SOLVE.json OUT.json -- per-layer GEOMETRY from a whole-route solve (whole_solve.py), ONE joint LP
over the trunk and both rings: smooth lanes, each at any angle, in their frames.

Frames: the trunk spine (u = s) and each ring's spine (u = H0 + s_b - rs). A lane of class W lives in the trunk
from its tooth to its berth; a lane of class N / S in the trunk from its tooth to the handoff (s = H0), then in
its ring from the handoff to its berth -- the two pieces joined by a linear equality (the ring's start offset is an
affine function of the trunk's handoff offset) and a bend term across the join. Columns every G mm of s in each
frame; the solve fixes the ORDER of the lanes present at every column (below(): launch order, flipped by each
solved crossing) and each lane's LAYER (flipped at each solved change; both layers within a via's reach of one).
  hard (elastic, priced W_HARD, reported): the order; same-layer neighbours P_MIN apart (+ half a pair's pitch
  each), slope corrected (tangent cuts of sqrt(1 + k^2) on the pair's mean slope); every neighbour a via's room
  from a change, changes a via pitch apart; inside the board and outside both pad boxes grown by a track's
  clearance (a via: grown by its static room), waived near the lane's own terminals; static copper to one side
  (from a first pass); the slope capped at K_MAX.
  soft: the length a lane's sideways moves add, bends, neighbours short of P_COMF (a different-layer pair only away from its own crossing).
Two passes: the second holds every lane to ONE side of each piece of static copper near it (static_sides: one split
per island and layer, in the lane order, pinned by the lanes' own ends), and a lane the polish found no room for on
its side is flipped to the other (GEO_FLIPS_FROM=POLISH.json,..: data the polish measured, never typed). What the
LP had to pay is written as CUTS for the solve (the islands a lane could not be kept off, the changes it could not
give their room). The bench from BENCH / NETS / DEST (whole_ctx)."""
import sys, os, json, math, collections, functools, time
import numpy as np
from scipy.optimize import linprog
from scipy.sparse import coo_matrix, csr_matrix, hstack
from types import SimpleNamespace
import whole_ctx
import braid as bd
import pairs as _pairs

# every length below is in the design rules' own units: track, clearance, via size, a via's room, the lane pitch
P_MIN = bd.LANE_MIN
P_COMF = bd.LANE_MIN + bd.TRACK           # a comfortable pitch: a track's width of air more
TW, CL = bd.TRACK, bd.CLEAR
B_M = TW / 2                            # a lane's copper outside the pad box line
VIA_R = bd.VIA_NEED
VIA_VV = bd.VIA_SIZE + CL
VIA_ST = bd.VIA_SIZE / 2 + CL
LANE_ST = TW / 2 + CL
PP = _pairs.pitch(TW)
K_MAX = 8.0                              # the steepest a lane runs to its spine
XW = 2 * bd.LANE_MIN                    # a different-layer pair may close up within XW of its own crossing
D_X = TW / 2                            # different-layer neighbours never touch away from their crossing
W_LEN, W_BEND, W_COMF, W_HARD = 1.0, 3.0, 0.5, 1e4
W_COMF_X = 1.0                           # the soft gap between different-layer neighbours
TANG = [0.0, 1.0, -1.0, 2.5, -2.5, 5.0, -5.0]
LEN_T = [0.5, -0.5, 1.0, -1.0, 2.0, -2.0, 4.0, -4.0]   # the slopes a column's added length is cut at
# the tangent cuts under-state sqrt(1 + k^2) between their tangent points: the SLOPED ones are scaled by the set's own
# worst ratio (the flat cut stays exact, so a flat pair is not over-held)
_kk = np.linspace(0.0, K_MAX, 4001)
TSCALE = float(1.0 / min(np.max([(1 + t_ * _kk) / math.sqrt(1 + t_ * t_) for t_ in TANG], axis=0) / np.sqrt(1 + _kk ** 2)))
F = lambda x: 1 if x == 'B.Cu' else 0
LNAME = ('F.Cu', 'B.Cu')
log = lambda *a: print(*a, flush=True)

J = json.load(open(sys.argv[1]))
OUT = sys.argv[2] if len(sys.argv) > 2 else '/dev/null'
ctx, cs = whole_ctx.plan()
c = cs[0]
M = list(c.members)
GRID2 = ctx.cfg.grid_step / 2             # half the router's grid step (the router's bar for a line off the grid)
G = 4 * ctx.cfg.grid_step                 # a column: four router grid steps
# a pair's straight run either side of its via (the longer, diagonal one, and a grid step) and a turn's own straight
# run, in columns
W_DIVE = int(math.ceil((_pairs.via_straight(ctx.cfg, (math.sqrt(0.5), math.sqrt(0.5))) + ctx.cfg.grid_step) / G - 1e-9))
W_TURN = int(math.ceil(_pairs.turn_straight_steps(ctx.cfg) * ctx.cfg.grid_step / G - 1e-9))
HOLD = max(1, int(round(TW / G)))                 # a tooth / west-face berth stub held straight a track's width
EXC = max(2, int(math.ceil(bd.LANE_MIN / G)))     # the box margin waived a lane pitch from the lane's own terminal
P_MIN = max(P_MIN, TW + CL + 2 * GRID2)      # two planned lines: each lands up to half a grid step off (the router's bar)
# the comfort penalty is GRADED (convex): a millimetre below the midpoint between the minimum and the comfortable pitch
# costs four times one above it, so the LP spreads the tightest neighbours first rather than letting a few pairs take
# all the squeeze
P_MID = (P_MIN + P_COMF) / 2
W_COMF_TIGHT = 3 * W_COMF
from fab_tiers import min_via_center_distance
VIA_VV = min_via_center_distance(bd.VIA_SIZE, CL, ctx.cfg.via_drill, getattr(ctx.cfg, 'hole_to_hole_clearance', 0.0) or 0.0)
VIA_VV += 2 * GRID2; VIA_ST += GRID2; LANE_ST += GRID2     # planned vs planned a whole step, vs static half
# a via's room off a lane's line: the router's RING round it (pairs.via_ring) and half a step for each of the two, the
# via and the line, off the grid -- the audit's bar (plan_audit.check_dives) and the polish's; the via's copper and the
# clearance alone left the polish 8 um to find at every via
VIA_R = max(VIA_R, _pairs.via_ring(ctx.cfg) + 2 * GRID2)
M_VIA = VIA_ST
bo = c.branch_of or {}
prs = getattr(ctx, 'pairs', {}) or {}
H0 = J['H0']; RS = J['rs']; cls = J['branch']
li = {n: i for i, n in enumerate(J['launch'])}
cross = {frozenset(k.split('|')): v['u'] for k, v in J['cross'].items()}
chg = {n: sorted(v) for n, v in J['changes'].items()}
tl = {n: F(ctx.tooth_layer[n]) for n in M}
# a pair's half width: its legs' reach at the snap's 45-degree corners and the half step its off-grid legs take, as the
# polish and the audit price it (half its pitch alone left the polish 23 um a side to find)
hw = {n: (PP / 2 / math.cos(math.pi / 8) + GRID2 if n in prs else 0.0) for n in M}
VS = {n: (VIA_R + PP / 2 if n in prs else VIA_R) for n in M}
# a pair's dive is TWO barrels, pairs.dive_offset either side of its centreline across the lane (where both pair
# routers stand them): its room is a single via's along the lane, widened across by that offset
VX = {n: (_pairs.dive_offset(ctx.cfg, PP / 2) if n in prs else 0.0) for n in M}
ring_sp = {}
for b in {id(v): v for v in bo.values()}.values():
    ring_sp['N' if b.spine.pts[0][1] < c.spine.xy(H0, 0.0)[1] else 'S'] = b.spine
bend_xy = {n: tuple(map(float, c.lane_xy[n][-1])) for n in M}


def below(a, b, u):
    x = frozenset((a, b))
    return (li[a] < li[b]) ^ (x in cross and cross[x] <= u + 1e-9)


def layer_of(n, u):
    return tl[n] ^ (sum(1 for cu in chg[n] if cu < u) & 1)


def layers_at(n, u):
    if any(abs(u - cu) <= VS[n] for cu in chg[n]):
        return {0, 1}
    return {layer_of(n, u)}


def pbox(ref, m):
    f_ = ctx.pcb.footprints[ref]
    xs = [p.global_x for p in f_.pads]; ys = [p.global_y for p in f_.pads]
    return (min(xs) - m, min(ys) - m, max(xs) + m, max(ys) + m)


def term_margin(ref, pts):
    """how far outside an array's pad box its lanes' terminals sit (the box the stubs end on): the median
    distance of the terminals from the pads' bounding box -- read off the board, not assumed"""
    f_ = ctx.pcb.footprints[ref]
    xs = [p.global_x for p in f_.pads]; ys = [p.global_y for p in f_.pads]
    ds = [max(min(xs) - x, x - max(xs), min(ys) - y, y - max(ys)) for (x, y) in pts]
    ds = [d for d in ds if d > 0]
    return float(np.median(ds)) if ds else 0.0


DST_REF = os.environ['DEST']
SRC_REF = collections.Counter(ctx.src_ref[n] for n in M).most_common(1)[0][0]
BOX = [pbox(SRC_REF, term_margin(SRC_REF, [ctx.ends[n][0] for n in M])),
       pbox(DST_REF, term_margin(DST_REF, [ctx.ends[n][1] for n in M]))]
log(f'pad boxes grown by the terminals: {[round(b[2] - b[0], 3) for b in BOX]}')
BX0, BY0, BX1, BY1 = ctx.pcb.board_info.board_bounds
EDGE = (float(getattr(ctx.cfg, 'board_edge_clearance', 0.0) or 0.0) or ctx.cfg.clearance) + TW / 2
_SPAN = max(BX1 - BX0, BY1 - BY0)
OS = np.arange(-_SPAN, _SPAN + 1e-9, TW / 6)


def spine_xy_vec(sp, s, O):
    k = sp.seg_of(s)
    t = s - sp.S[k]
    return (sp.P[k, 0] + t * sp.d[k, 0] + O * sp.nrm[k, 0], sp.P[k, 1] + t * sp.d[k, 1] + O * sp.nrm[k, 1])


_ivc = {}


def intervals(ftag, sp, s, margin):
    key = (ftag, round(s, 4), margin)
    if key in _ivc:
        return _ivc[key]
    X, Y = spine_xy_vec(sp, s, OS)
    ok = (X >= BX0 + EDGE) & (X <= BX1 - EDGE) & (Y >= BY0 + EDGE) & (Y <= BY1 - EDGE)
    for b in BOX:
        ok &= ~((X > b[0] - margin) & (X < b[2] + margin) & (Y > b[1] - margin) & (Y < b[3] + margin))
    out, i = [], 0
    while i < len(OS):
        if ok[i]:
            j = i
            while j + 1 < len(OS) and ok[j + 1]:
                j += 1
            out.append((float(OS[i]), float(OS[j])))
            i = j + 1
        else:
            i += 1
    _ivc[key] = out
    return out


STATIC = []
ISL = {}             # a footprint's pads on one layer set: ONE island (a lane goes round the part, not between its pads)
for ref, fp in ctx.pcb.footprints.items():
    if ref in (SRC_REF, DST_REF):
        continue
    for p in fp.pads:
        drilled = bool(p.drill and p.drill > 0)
        if p.pad_type == 'np_thru_hole':
            hx = hy = (p.drill or 0) / 2; Ls = {0, 1}
        else:
            hx, hy = p.size_x / 2, p.size_y / 2
            Ls = {0, 1} if drilled else {F(L) for L in ('F.Cu', 'B.Cu') if L in p.layers}
        if Ls and BX0 < p.global_x < BX1 and BY0 < p.global_y < BY1:
            ISL.setdefault((ref, frozenset(Ls)), []).append((p.global_x - hx, p.global_y - hy, p.global_x + hx, p.global_y + hy))
for (ref, Ls), bxs in ISL.items():
    STATIC.append((min(b[0] for b in bxs), min(b[1] for b in bxs), max(b[2] for b in bxs), max(b[3] for b in bxs),
                   set(Ls), ref))
mem = {ctx.byname[n][0] for n in M} | {ctx.byname[leg][0] for n in M for leg in prs.get(n, ()) if leg in ctx.byname}
for s_ in ctx.base_segments:
    if s_.net_id in mem:
        continue
    for (x, y) in ((s_.start_x, s_.start_y), (s_.end_x, s_.end_y)):
        if any(((abs(x - b[0]) < TW / 6 or abs(x - b[2]) < TW / 6) and b[1] - TW / 6 <= y <= b[3] + TW / 6)
               or ((abs(y - b[1]) < TW / 6 or abs(y - b[3]) < TW / 6) and b[0] - TW / 6 <= x <= b[2] + TW / 6) for b in BOX):
            r = s_.width / 2
            STATIC.append((x - r, y - r, x + r, y + r, {F(s_.layer)}, 'tooth ' + ctx.pcb.nets[s_.net_id].name.split('/')[-1]))


# every other lane's tooth and berth END on the box line, on its stub's layer (a pair: both legs' ends)
LANE_OF = {}
for n in M:
    LANE_OF[n] = n
    for leg in prs.get(n, ()):
        LANE_OF[leg] = n
TERMS = []                                   # (x, y, layer 0/1, owner lane)
for net, (src, tgt, _ref) in ctx.ends.items():
    if net not in LANE_OF:
        continue
    for pt, Lnm in ((src, ctx.tooth_layer.get(net)), (tgt, ctx.dest_layer.get(net))):
        if Lnm is None:
            continue
        TERMS.append((float(pt[0]), float(pt[1]), F(Lnm), LANE_OF[net]))
for (x, y, L, own) in TERMS:
    r = TW / 2
    STATIC.append((x - r, y - r, x + r, y + r, {L}, f'end {own}', own))
# the stubs' COPPER within a via's reach of the box line (a via beside the line meets the stub inside it, not only
# its end): every base segment of a lane's net with an end within that reach, clipped to it -- for VIAS only
NET_LANE = {ctx.byname[n][0]: n for n in M}
for n in M:
    for leg in prs.get(n, ()):
        if leg in ctx.byname:
            NET_LANE[ctx.byname[leg][0]] = n
def stub_pieces(reach):
    """every base segment END within `reach` of a pad box line, clipped to `reach` along the segment"""
    out = []
    for s_ in ctx.base_segments:
        own_ = NET_LANE.get(s_.net_id)
        for (ax, ay), (bx_, by_) in (((s_.start_x, s_.start_y), (s_.end_x, s_.end_y)), ((s_.end_x, s_.end_y), (s_.start_x, s_.start_y))):
            near = any(min(abs(ax - b[0]), abs(ax - b[2])) < reach and b[1] - reach <= ay <= b[3] + reach or
                       min(abs(ay - b[1]), abs(ay - b[3])) < reach and b[0] - reach <= ax <= b[2] + reach for b in BOX)
            if not near:
                continue
            L_ = math.hypot(bx_ - ax, by_ - ay)
            f_ = min(1.0, reach / L_) if L_ > 1e-9 else 0.0
            cx, cy = ax + (bx_ - ax) * f_, ay + (by_ - ay) * f_
            r = s_.width / 2
            out.append((min(ax, cx) - r, min(ay, cy) - r, max(ax, cx) + r, max(ay, cy) + r, {F(s_.layer)},
                        'stub ' + (own_ or ctx.pcb.nets[s_.net_id].name.split('/')[-1]), own_))
    return out


# a VIA beside the box line meets stub copper within its reach; a LANE (never inside the box) only the copper
# within a clearance block of the line
REACH = VIA_ST + bd.VIA_NEED
VSTUB = stub_pieces(REACH)                   # (x0, y0, x1, y1, {layer}, label, owner)
LSTUB = stub_pieces(TW + CL)
# base VIAS near the box lines (a stub ending in a via: SDQM1's berth) -- discs on both layers
for v_ in ctx.base_vias:
    own_ = NET_LANE.get(v_.net_id)
    if any(b[0] - REACH <= v_.x <= b[2] + REACH and b[1] - REACH <= v_.y <= b[3] + REACH for b in BOX):
        r = v_.size / 2
        VSTUB.append((v_.x - r, v_.y - r, v_.x + r, v_.y + r, {0, 1}, 'svia ' + (own_ or ctx.pcb.nets[v_.net_id].name.split('/')[-1]), own_))
# ... and the LANES keep off the stub copper within a clearance block of the line, and off the stub vias
for st_ in LSTUB + [v for v in VSTUB if v[5].startswith('svia ')]:
    STATIC.append(st_)
log(f'stub copper near the box lines: {len(VSTUB)} pieces for vias, {len(LSTUB)} for lanes')


def face_of(xy, box):
    x, y = xy
    d = {'W': abs(x - box[0]), 'E': abs(x - box[2]), 'N': abs(y - box[1]), 'S': abs(y - box[3])}
    return min(d, key=d.get)


# ---------------------------------------------------------------- frames and lane pieces
FR = {'T': dict(sp=c.spine, u=lambda s: s, s=lambda u: u)}
for k_, sp in ring_sp.items():
    FR[k_] = dict(sp=sp, u=(lambda s, k_=k_: H0 + (s - RS[k_])), s=(lambda u, k_=k_: RS[k_] + (u - H0)))
def hold_pair(tips):
    """A PAIR's end is held straight (in columns) for its END RUN (pairs.end_run: its end connector from those tips to
    the pose where the pair router takes over, then the straight the router probes past the pose): the pose lies on
    the plan, and the legs converge along the end's own direction. (SDQS1 turned 66 degrees 0.125 mm before a berth
    whose tips stand 0.45 apart, and the pair router could not leave it.)"""
    return max(HOLD, int(math.ceil(_pairs.end_run(ctx.cfg, tips) / G)))


SB0 = {}
PIECE = {}          # (frame, n) -> dict(k0, k1, o0, o1, hold0, hold1, ex0, ex1, ref)
ms_of = lambda n: (np.array([p[0] for p in c.mid[n]]), np.array([p[1] for p in c.mid[n]]))
for n in M:
    s0, o0 = c.st[n]
    f0 = face_of(c.spine.xy(s0, o0), BOX[0])
    hold0 = (hold_pair(ctx.pair_ends[n][0]) if n in prs else HOLD) if f0 in ('E', 'W') else 0
    ms, mo = ms_of(n)
    ref = (lambda s, ms=ms, mo=mo: float(np.interp(s, ms, mo)))
    if n in cls:
        PIECE[('T', n)] = dict(k0=int(round(s0 / G)), k1=int(round(H0 / G)), o0=o0, o1=None, hold0=hold0, hold1=0,
                               ex0=max(hold0 + 1, EXC), ex1=0, ref=ref)
        sp = ring_sp[cls[n]]
        o_h = float(np.interp(H0, ms, mo))                      # the old plan's offset at the handoff: the start column
        sb0, ob0 = sp.project_pt(c.spine.xy(H0, o_h))
        SB0.setdefault(cls[n], []).append(sb0)
        sb1, ob1 = sp.project_pt(bend_xy[n])
        PIECE[(cls[n], n)] = dict(k0=int(round(sb0 / G)), k1=int(round(sb1 / G)), o0=None, o1=ob1, hold0=0, hold1=0,
                                  ex0=0, ex1=EXC, ref=(lambda s, a=(sb0, ob0), b=(sb1, ob1): float(np.interp(s, [a[0], b[0]], [a[1], b[1]]))),
                                  o_h=o_h)
    else:
        s1, o1 = c.se[n]
        f1 = face_of(c.spine.xy(s1, o1), BOX[1])
        hold1 = (hold_pair(ctx.pair_ends[n][1]) if n in prs else HOLD) if f1 in ('E', 'W') else 0
        PIECE[('T', n)] = dict(k0=int(round(s0 / G)), k1=int(round(s1 / G)), o0=o0, o1=o1, hold0=hold0, hold1=hold1,
                               ex0=max(hold0 + 1, EXC), ex1=max(hold1 + 1, EXC), ref=ref)


S0C = {f: RS[f] for f in SB0}             # the solve's ring origin: ahead of every lane's trunk end (no step back at the seam)
for (f, n), v in PIECE.items():
    if f != 'T':
        v['k0'] = int(round(S0C[f] / G))                    # every lane of a ring enters it at ONE column
for f in S0C:                                               # ... and the ring's u starts THERE
    FR[f]['u'] = (lambda s, f=f: H0 + (s - S0C[f]))
    FR[f]['s'] = (lambda u, f=f: S0C[f] + (u - H0))
TERM = {n: (tuple(map(float, c.spine.xy(*c.st[n]))), bend_xy[n]) for n in M}


def build_and_solve(sides, prev=None):
    t0 = time.time()
    var = {}
    for (f, n), v in PIECE.items():
        for k in range(v['k0'], v['k1'] + 1):
            var[(f, n, k)] = len(var)
    inv = {j: key for key, j in var.items()}

    def tangents(sl):
        """The slopes to cut sqrt(1 + k^2) at for the mean slope of segments sl [(j1, j0)]: with a previous
        solution, EXACT tangents at its slope and either side of it (a tangent line under-states the curve away from
        its point, so the cuts sit where the answer is), plus a sparse far set; without one, the sparse set scaled
        by its worst ratio. [(t0, scale)]; the flat cut is the caller's."""
        if prev is None or not sl:
            return [(t_, TSCALE) for t_ in TANG if t_]
        ks = [(prev.get(inv[p1], 0.0) - prev.get(inv[p0], 0.0)) / G for (p1, p0) in sl]
        kp = sum(ks) / len(ks)
        out = {round(kp + d_, 4) for d_ in (-0.35, 0.0, 0.35)} | {1.5, -1.5, 4.0, -4.0}
        return [(t_, 1.0) for t_ in sorted(out) if abs(t_) > 1e-3]
    nv = len(var)
    extra, rows, cols, vals, rhs, tags = [], [], [], [], [], []

    def newvar(cost):
        extra.append(cost)
        return nv + len(extra) - 1

    def le(terms, b, elastic=None):
        r = len(rhs)
        for j, a_ in terms:
            rows.append(r); cols.append(j); vals.append(a_)
        if elastic is not None:
            e = newvar(W_HARD)
            rows.append(r); cols.append(e); vals.append(-1.0)
            tags.append((e, elastic))
        rhs.append(b)

    bounds = {}
    orders = {}
    for f in FR:
        pcs = {n: v for (f_, n), v in PIECE.items() if f_ == f}
        if not pcs:
            continue
        kmin = min(v['k0'] for v in pcs.values()); kmax = max(v['k1'] for v in pcs.values())
        ufun = FR[f]['u']
        for k in range(kmin, kmax + 1):
            u = ufun(k * G)
            pres = [n for n, v in pcs.items() if v['k0'] <= k <= v['k1']]
            if not pres:
                continue
            od = sorted(pres, key=functools.cmp_to_key(lambda a, b: -1 if below(a, b, u) else 1))
            orders[(f, k)] = od
            lay = {n: layers_at(n, u) for n in od}

            def slopes(n):
                """(p1, p0) for the leaving and the arriving segment at column k"""
                out = []
                if (f, n, k + 1) in var:
                    out.append((var[(f, n, k + 1)], var[(f, n, k)]))
                if (f, n, k - 1) in var:
                    out.append((var[(f, n, k)], var[(f, n, k - 1)]))
                return out
            for a, b in zip(od, od[1:]):
                ja, jb = var[(f, a, k)], var[(f, b, k)]
                le([(ja, 1.0), (jb, -1.0)], 0.0, ('order', f, k, a, b))
                if not (lay[a] & lay[b]):
                    x = frozenset((a, b))
                    if not (x in cross and abs(cross[x] - u) < XW):
                        le([(ja, 1.0), (jb, -1.0)], -D_X, ('kiss', f, k, a, b))
                        e = newvar(W_COMF_X * G)
                        le([(ja, 1.0), (jb, -1.0), (e, -1.0)], -(P_MIN + hw[a] + hw[b]))
            def term_o(n):
                """(which end, its fixed offset) when column k is within this lane's terminal zone here"""
                v = pcs[n]
                if k - v['k0'] < v['ex0'] and v['o0'] is not None:
                    return ('start', v['o0'])
                if v['k1'] - k < v['ex1'] and v['o1'] is not None:
                    return ('end', v['o1'])
                return None
            for Ly in (0, 1):
                seq = [n for n in od if Ly in lay[n]]
                for a, b in zip(seq, seq[1:]):
                    sep = P_MIN + hw[a] + hw[b]
                    ta, tb = term_o(a), term_o(b)
                    if ta and tb and ta[0] == tb[0]:
                        sep = min(sep, max(0.0, abs(tb[1] - ta[1]) - G / 20))
                    ja, jb = var[(f, a, k)], var[(f, b, k)]
                    sa_, sb_ = slopes(a), slopes(b)
                    # the mean slope of the pair, leaving segments together and arriving segments together; a pair
                    # that pass 1 laid far from parallel is held off the FLATTER lane's line only (a steep leg beside
                    # a flat ring lane: the mean asked up to 3x the room it needs)
                    flat_only = None
                    if prev is not None:
                        def k_of(n_):
                            o0_, o1_ = prev.get((f, n_, k)), prev.get((f, n_, k + 1), prev.get((f, n_, k - 1)))
                            return abs(o1_ - o0_) / G if (o0_ is not None and o1_ is not None) else 0.0
                        ka_, kb_ = k_of(a), k_of(b)
                        if abs(ka_ - kb_) > 1.0 and min(ka_, kb_) < 0.3:
                            flat_only = sa_ if ka_ < kb_ else sb_
                    combos = []
                    if flat_only is not None:
                        combos = [[x] for x in flat_only]
                    else:
                        for i_ in range(max(len(sa_), len(sb_))):
                            combos.append([x for x in (sa_[min(i_, len(sa_) - 1)] if sa_ else None,
                                                       sb_[min(i_, len(sb_) - 1)] if sb_ else None) if x])
                    for t0_ in (TANG if prev is not None else [0.0]):
                        al, be = 1 / math.sqrt(1 + t0_ ** 2), t0_ / math.sqrt(1 + t0_ ** 2)
                        if t0_:
                            al, be = al * TSCALE, be * TSCALE
                        for sl in (combos if be else [[]]):
                            terms = [(ja, 1.0), (jb, -1.0)]
                            for (p1, p0) in sl:
                                terms += [(p1, sep * be / (len(sl) * G)), (p0, -sep * be / (len(sl) * G))]
                            le(terms, -sep * al, ('pitch', f, k, a, b, Ly))
                    e = newvar(W_COMF * G)
                    le([(ja, 1.0), (jb, -1.0), (e, -1.0)], -(P_COMF + hw[a] + hw[b]))
                    # ...and steeper below the midpoint to the minimum: the tightest pairs are spread first
                    e = newvar(W_COMF_TIGHT * G)
                    le([(ja, 1.0), (jb, -1.0), (e, -1.0)], -(P_MID + hw[a] + hw[b]))
    # vias
    vias = []
    for (f, n), v in PIECE.items():
        sfun = FR[f]['s']
        for cu in chg[n]:
            if f == 'T' and n in cls and cu > H0:
                continue
            if f != 'T' and cu <= H0:
                continue
            s_c = sfun(cu)
            kc = int(round(s_c / G))
            if not (v['k0'] <= kc <= v['k1']):
                continue
            vias.append((f, n, cu, kc))
            jc = var[(f, n, kc)]
            r = VIA_R if VX[n] else VS[n]
            for k in range(int(math.floor((s_c - r) / G)), int(math.ceil((s_c + r) / G)) + 1):
                if (f, k) not in orders or (f, n, k) not in var:
                    continue
                ds = k * G - s_c
                if abs(ds) >= r:
                    continue
                h = math.sqrt(r * r - ds * ds) + VX[n]
                od = orders[(f, k)]
                i = od.index(n)
                # the neighbour's LINE a via's room off: its offset at this column, slope corrected
                # (tangent cuts of sqrt(1 + k^2) on the neighbour's own slope at the column)
                # the TWO nearest lanes each side: in a stacked F/B comb the nearest can share the via lane's offset
                # on the other layer, and the lane that matters is the next one
                for nb in [od[j] for j in (i + 1, i + 2, i - 1, i - 2) if 0 <= j < len(od)]:
                    up = od.index(nb) > i
                    jm = var[(f, nb, k)]
                    segs_ = [(var[(f, nb, k + 1)], jm)] if (f, nb, k + 1) in var else []
                    segs_ += [(jm, var[(f, nb, k - 1)])] if (f, nb, k - 1) in var else []
                    need = h + hw[nb]
                    vn_ = PIECE[(f, nb)]
                    if k - vn_['k0'] < vn_['ex0'] or vn_['k1'] - k < vn_['ex1']:
                        need += G / 4          # its terminal is drawn exact, up to half a column from its column
                    sg = 1.0 if up else -1.0
                    # up: o_m - o_v >= need * (al + be * k_m)   down: o_v - o_m >= need * (al - be * k_m)
                    le([(jc, sg), (jm, -sg)], -need, ('via', f, k, n, nb))
                    for (j1, j0) in segs_:
                        for t0_, sc_ in tangents([(j1, j0)]):
                            al, be = sc_ / math.sqrt(1 + t0_ ** 2), sc_ * t0_ / math.sqrt(1 + t0_ ** 2)
                            terms = [(jc, sg), (jm, -sg), (j1, sg * need * be / G), (j0, -sg * need * be / G)]
                            le(terms, -need * al, ('via', f, k, n, nb))
    for i, (f, n, cu, kc) in enumerate(vias):
        for (f2, m_, cu2, kc2) in vias[i + 1:]:
            ds = abs(FR[f]['s'](cu) - FR[f2]['s'](cu2)) if f2 == f else math.inf
            if f2 != f or m_ == n or ds >= VIA_VV:
                continue
            h = math.sqrt(VIA_VV ** 2 - ds ** 2) + VX[n] + VX[m_]     # a pair's barrels stand VX across
            km = (kc + kc2) // 2
            od = orders.get((f, km), [])
            if n not in od or m_ not in od:
                continue
            a, b = (n, m_) if od.index(n) < od.index(m_) else (m_, n)
            ka, kb = (kc, kc2) if a == n else (kc2, kc)
            le([(var[(f, a, ka)], 1.0), (var[(f, b, kb)], -1.0)], -h, ('viavia', f, km, a, b))
    via_at = {(f, n, kc) for (f, n, cu, kc) in vias}
    # bounds: board, pad boxes (a via further off), the interval nearest the reference
    for (f, n, k), j in var.items():
        v = PIECE[(f, n)]
        s_ = k * G
        if (k == v['k0'] and v['o0'] is not None) or (k == v['k1'] and v['o1'] is not None):
            continue                     # a fixed terminal: its offset is the board's, no bound to price
        if (0 < k - v['k0'] <= v['hold0'] and v['o0'] is not None) or (0 < v['k1'] - k <= v['hold1'] and v['o1'] is not None):
            continue
        near_end = (k - v['k0'] < v['ex0']) or (v['k1'] - k < v['ex1'])
        mg = 0.0 if near_end else B_M
        if (f, n, k) in via_at:
            mg = M_VIA + VX[n]
        iv = intervals(f, FR[f]['sp'], s_, round(mg, 4))
        if not iv:
            continue
        ref = v['ref'](s_)
        lo_, hi_ = min(iv, key=lambda q: 0 if q[0] <= ref <= q[1] else min(abs(ref - q[0]), abs(ref - q[1])))
        le([(j, -1.0)], -lo_, ('bound', f, k, n, 'lo'))
        le([(j, 1.0)], hi_, ('bound', f, k, n, 'hi'))
    # terminals, holds, slope cap, travel, bends
    for (f, n), v in PIECE.items():
        for k, o_ in ((v['k0'], v['o0']), (v['k1'], v['o1'])):
            if o_ is not None:
                bounds[var[(f, n, k)]] = (o_, o_)
        for i in range(1, v['hold0'] + 1):
            if v['o0'] is not None and (f, n, v['k0'] + i) in var:
                bounds[var[(f, n, v['k0'] + i)]] = (v['o0'], v['o0'])
        for i in range(1, v['hold1'] + 1):
            if v['o1'] is not None and (f, n, v['k1'] - i) in var:
                bounds[var[(f, n, v['k1'] - i)]] = (v['o1'], v['o1'])
        for k in range(v['k0'], v['k1']):
            a, b = var[(f, n, k)], var[(f, n, k + 1)]
            le([(b, 1.0), (a, -1.0)], K_MAX * G, ('slope', f, k, n))
            le([(a, 1.0), (b, -1.0)], K_MAX * G, ('slope', f, k, n))
            # the LENGTH a column's sideways move adds, G (sqrt(1 + k^2) - 1) for its slope k, from below by tangent
            # cuts: a small move costs next to nothing (it is quadratic), so a lane takes the comfortable pitch
            # wherever the room is -- priced as |move|, a lane gave its pitch up to save a move that added no length
            d = newvar(W_LEN)
            for t_ in LEN_T:
                r_ = math.sqrt(1 + t_ * t_)
                le([(b, t_ / r_), (a, -t_ / r_), (d, -1.0)], -G * (1 / r_ - 1))
            if k > v['k0']:
                p = var[(f, n, k - 1)]
                d2 = newvar(W_BEND)
                le([(b, 1.0), (a, -2.0), (p, 1.0), (d2, -1.0)], 0.0)
                le([(b, -1.0), (a, 2.0), (p, -1.0), (d2, -1.0)], 0.0)
    # a PAIR moves through its dives as the pair router does: straight for its straight run either side of each change
    # (no turn at or near its via), and where a dive falls within its end hold, that run and a turn of its fixed end,
    # straight from the end right through it -- its sideways shift onto its terminal comes before the dive, never
    # between the terminal and the dive (SDQS0 dived in its last 0.4 mm, where the lanes shift onto the berths, and
    # could not be laid straight). Elastic, priced as the other hard rules, so the singles crossing the pair there keep
    # their room; what the room will not give is paid, and sent to the solve as a via cut
    for (f, n, cu, kc) in vias:
        if n not in prs:
            continue
        v = PIECE[(f, n)]
        tag = ('pdive', f, kc, n)
        if v['o1'] is not None and v['k1'] - kc <= v['hold1'] + W_DIVE + W_TURN:
            for k in range(max(v['k0'], kc - W_DIVE), v['k1'] - v['hold1']):
                le([(var[(f, n, k)], 1.0)], v['o1'], tag); le([(var[(f, n, k)], -1.0)], -v['o1'], tag)
            continue
        if v['o0'] is not None and kc - v['k0'] <= v['hold0'] + W_DIVE + W_TURN:
            for k in range(v['k0'] + v['hold0'] + 1, min(v['k1'], kc + W_DIVE) + 1):
                le([(var[(f, n, k)], 1.0)], v['o0'], tag); le([(var[(f, n, k)], -1.0)], -v['o0'], tag)
            continue
        for k in range(max(v['k0'] + 1, kc - W_DIVE + 1), min(v['k1'] - 1, kc + W_DIVE - 1) + 1):
            a_, b_, p_ = var[(f, n, k - 1)], var[(f, n, k)], var[(f, n, k + 1)]
            le([(p_, 1.0), (b_, -2.0), (a_, 1.0)], 0.0, tag)
            le([(p_, -1.0), (b_, 2.0), (a_, -1.0)], 0.0, tag)
    # a lane approaches its fixed terminals monotonically over its terminal zone (pass 2: the direction from pass 1)
    if prev is not None:
        for (f, n), v in PIECE.items():
            for (kt, zone, sgn_) in ((v['k1'], range(v['k1'] - v['ex1'], v['k1']), 1), (v['k0'], range(v['k0'], v['k0'] + v['ex0']), -1)):
                ot = v['o1'] if sgn_ > 0 else v['o0']
                if ot is None or not zone:
                    continue
                ks = [k for k in zone if (f, n, k) in var and (f, n, k + 1) in var]
                if not ks:
                    continue
                far = min(ks) if sgn_ > 0 else max(ks) + 1
                d_ = ot - prev.get((f, n, far), ot)
                if abs(d_) < 1e-6:
                    continue
                dirn = 1.0 if (d_ > 0) == (sgn_ > 0) else -1.0      # o increases along s toward / away from it
                for k in ks:
                    # dirn * (o[k+1] - o[k]) >= 0
                    le([(var[(f, n, k)], dirn), (var[(f, n, k + 1)], -dirn)], 0.0, ('approach', f, k, n))
    # the handoff joins: ring start = alpha + beta * trunk handoff offset; and a bend across the join
    for n in M:
        if n not in cls:
            continue
        vT, vR = PIECE[('T', n)], PIECE[(cls[n], n)]
        sp = ring_sp[cls[n]]
        o_h = vR['o_h']
        ob1, ob2 = sp.project_pt(c.spine.xy(H0, o_h))[1], sp.project_pt(c.spine.xy(H0, o_h + 1.0))[1]
        beta = ob2 - ob1
        alpha = ob1 - beta * o_h
        jT, jR = var[('T', n, vT['k1'])], var[(cls[n], n, vR['k0'])]
        le([(jR, 1.0), (jT, -beta)], alpha); le([(jR, -1.0), (jT, beta)], -alpha)
        if ('T', n, vT['k1'] - 1) in var and (cls[n], n, vR['k0'] + 1) in var:
            jT0, jR1 = var[('T', n, vT['k1'] - 1)], var[(cls[n], n, vR['k0'] + 1)]
            d2 = newvar(W_BEND)
            le([(jR1, 1.0), (jR, -1.0), (jT, -beta), (jT0, beta), (d2, -1.0)], 0.0)
            le([(jR1, -1.0), (jR, 1.0), (jT, beta), (jT0, -beta), (d2, -1.0)], 0.0)
    # static copper sides
    for (f, n, k, lo_, hi_, side, what) in (sides or []):
        j = var.get((f, n, k))
        if j is None:
            continue
        if side < 0:
            le([(j, 1.0)], lo_, ('static', f, k, n, what))
        else:
            le([(j, -1.0)], -hi_, ('static', f, k, n, what))
    ncol = nv + len(extra)
    cost = np.zeros(ncol); cost[nv:] = extra
    Aub = coo_matrix((vals, (rows, cols)), shape=(len(rhs), ncol)).tocsr()
    bnd = [(-2 * _SPAN, 2 * _SPAN)] * nv + [(0.0, None)] * len(extra)
    for j, b in bounds.items():
        bnd[j] = b
    tb = time.time()
    res = lp_by_dual(cost, Aub, np.array(rhs), bnd)
    if res.status != 0:
        log(f'  LP status {res.status}: {res.message}')
        return None
    x = res.x
    o = {key: float(x[j]) for key, j in var.items()}
    paid = collections.defaultdict(list)
    for e, tg in tags:
        if x[e] > 1e-4:
            paid[tg[0]].append((float(x[e]),) + tuple(tg[1:]))
    log(f'  joint LP: {len(PIECE)} pieces, {nv} o-vars, {len(rhs)} rows, build {tb - t0:.0f}s solve {time.time() - tb:.0f}s, '
        f'obj {res.fun:.1f}; elastic paid: ' + (', '.join(f'{k} {len(v)} (max {max(q[0] for q in v):.3f})' for k, v in paid.items()) or 'none'))
    return dict(o=o, paid=paid, vias=vias, orders=orders)


def lp_by_dual(c, A, b, bnd):
    """min c'x subject to A x <= b and the column bounds bnd, solved as its DUAL by the same solver (scipy's HiGHS
    interior point) and x read back from the dual's multipliers: the same optimum. This LP has more rows than columns
    and an elastic slack on most rows; its dual has a row per column, and every slack's is a plain inequality --
    HiGHS's interior point takes it six times faster (K51 pass 2: 31 s, not 188). The LP has many optima; the dual
    lands on one of them, as the primal does."""
    lo = np.array([-np.inf if l_ is None else l_ for l_, _h in bnd], float)
    hi = np.array([np.inf if h_ is None else h_ for _l, h_ in bnd], float)
    A = A.tocsc()
    n = A.shape[1]
    plain = (lo == 0) & ~np.isfinite(hi)                    # x >= 0 alone: its dual row an inequality
    bj = np.flatnonzero(~plain)                             # the rest: an equality, a multiplier per finite bound
    fl, fh = bj[np.isfinite(lo[bj])], bj[np.isfinite(hi[bj])]
    AT = (-A.T).tocsr()
    pos = np.full(n, -1)
    pos[bj] = np.arange(len(bj))
    U = coo_matrix((np.ones(len(fl)), (pos[fl], np.arange(len(fl)))), shape=(len(bj), len(fl)))
    W = coo_matrix((-np.ones(len(fh)), (pos[fh], np.arange(len(fh)))), shape=(len(bj), len(fh)))
    res = linprog(np.concatenate([b, -lo[fl], hi[fh]]),
                  A_ub=hstack([AT[plain], csr_matrix((int(plain.sum()), len(fl) + len(fh)))]).tocsr(), b_ub=c[plain],
                  A_eq=hstack([AT[bj], U, W]).tocsr(), b_eq=c[bj], bounds=(0, None), method='highs-ipm')
    x = np.zeros(n)
    if res.status == 0:
        x[plain] = -res.ineqlin.marginals
        x[bj] = -res.eqlin.marginals
    return SimpleNamespace(status=res.status, message=res.message, x=x, fun=float(c @ x))


ROOMLESS = {}
# (lane, island) pairs the xy polish could not keep clear on the side pass 1 chose: that lane passes the island on
# the other side (the island then stands between two other lanes of the same order; no crossing changes). Read from
# the previous polish output(s) the loop names in GEO_FLIPS_FROM -- data it measured, never typed.
FLIP = set()
for _f in [x for x in os.environ.get('GEO_FLIPS_FROM', '').split(',') if x]:
    FLIP |= {tuple(x) for x in json.load(open(_f)).get('flips', [])}
WIN = 12 * bd.LANE_MIN                  # an island concerns the lanes within this of it (first pass)


def static_sides(sol):
    out = []
    boxes = {}
    for f in FR:
        sp = FR[f]['sp']
        bl = []
        for st_ in STATIC:
            (x0, y0, x1, y1, Ls, lab) = st_[:6]
            own = st_[6] if len(st_) > 6 else None
            so = [sp.project_pt(p) for p in ((x0, y0), (x0, y1), (x1, y0), (x1, y1))]
            bl.append((min(q[0] for q in so), max(q[0] for q in so), min(q[1] for q in so), max(q[1] for q in so), Ls, lab, own))
        boxes[f] = bl
    vboxes = {}
    for f in FR:
        sp = FR[f]['sp']
        vboxes[f] = []
        for (x0, y0, x1, y1, Ls, lab, own) in VSTUB:
            so = [sp.project_pt(p) for p in ((x0, y0), (x0, y1), (x1, y0), (x1, y1))]
            vboxes[f].append((min(q[0] for q in so), max(q[0] for q in so), min(q[1] for q in so), max(q[1] for q in so), Ls, lab, own))
    ROOMLESS.clear()
    need = 2 * LANE_ST + TW                      # one lane's centre with its clearance each side
    for f, bl in boxes.items():
        for ii, (sa, sb, oa, ob, Ls, lab, own) in enumerate(bl):
            if own is not None or sb < 0:
                continue
            below_ok = above_ok = True
            for s_ in np.arange(sa, sb + 1e-9, G):
                iv = intervals(f, FR[f]['sp'], float(s_), round(B_M, 4))
                around = [q for q in iv if q[0] <= oa + 1e-6 and q[1] >= ob - 1e-6] or \
                         [q for q in iv if q[1] >= oa - WIN and q[0] <= ob + WIN]
                if not around:
                    continue
                lo_i, hi_i = min(q[0] for q in around), max(q[1] for q in around)
                below_ok &= (oa - lo_i) >= need
                above_ok &= (hi_i - ob) >= need
            if below_ok != above_ok:
                ROOMLESS[(f, ii)] = 1 if not below_ok else -1
    vcol = collections.defaultdict(set)
    for (f, n, cu, kc) in sol['vias']:
        vcol[(f, n)].add(kc)
    # ONE side per lane and island, from the lane's mean first-pass offset over the island's span: chosen per
    # column, a lane descending through the span was told north at its first columns and south at its last
    mean_o = collections.defaultdict(list)
    for (f, n, k), o_ in sol['o'].items():
        s_ = k * G
        for ii, (sa, sb, oa, ob, Ls, lab, own) in enumerate(boxes[f]):
            g = LANE_ST + hw[n]
            if own != n and sa - g <= s_ <= sb + g and oa - WIN <= o_ <= ob + WIN:
                mean_o[(f, n, ii)].append(o_)
    # ONE SPLIT PER ISLAND AND LAYER: the island stands BETWEEN two lanes of the order (where a human puts a part
    # between two lanes) -- every same-layer lane below the split passes below it, every one above passes above.
    # Chosen per lane from its own first-pass offset, two neighbours could be told the two sides the wrong way round
    # and one paid millimetres to cross the other (SA12 / SA10 / SA15 past SVREF's stub, up to 3.15). The split costs
    # the first-pass movement it asks for; a lane whose own fixed end lies in the span pins its side; each side must
    # hold its lanes at pitch in the room the frame measures (exact on a straight octilinear piece).
    SIDE, SPLIT = {}, set()
    for f, bl in boxes.items():
        for ii, (sa, sb, oa, ob, Ls, lab, own) in enumerate(bl):
            if sb < 0:
                continue
            # the lane order where the island stands: its middle column, else the nearest column of its span that has
            # lanes (an island ON the box line stands before a lane's first column)
            k_lo, k_hi = int(math.floor(sa / G)) - 1, int(math.ceil(sb / G)) + 1
            kmid = int(round((sa + sb) / 2 / G))
            ks_ = sorted((k_ for k_ in range(k_lo, k_hi + 1) if sol.get('orders', {}).get((f, k_))),
                         key=lambda k_: abs(k_ - kmid))
            if not ks_:
                continue
            kmid = ks_[0]
            od = sol['orders'][(f, kmid)]
            um = FR[f]['u'](kmid * G)
            room = {-1: math.inf, 1: math.inf}
            span = [-math.inf, math.inf]                    # the free interval the island stands in
            for s_ in np.arange(sa, sb + 1e-9, G):
                iv = intervals(f, FR[f]['sp'], float(s_), round(B_M, 4))
                around = [q for q in iv if q[0] <= oa + 1e-6 and q[1] >= ob - 1e-6]
                if around:
                    lo_i, hi_i = min(q[0] for q in around), max(q[1] for q in around)
                    room[-1] = min(room[-1], oa - lo_i)
                    room[1] = min(room[1], hi_i - ob)
                    span = [max(span[0], lo_i), min(span[1], hi_i)]
            SPLIT.add((f, ii))
            for Ly in (0, 1):
                if Ly not in Ls:
                    continue
                # only the lanes that share the island's free interval can reach it; one in another interval is
                # kept off it by that interval's own bounds (SA12, 2.6 mm above SVREF's stub, was told to pass below)
                lanes = [n for n in od if n != own and Ly in layers_at(n, um) and (f, n, ii) in mean_o
                         and span[0] - 1e-6 <= float(np.mean(mean_o[(f, n, ii)])) <= span[1] + 1e-6]
                if not lanes:
                    continue
                cost_below, cost_above, pin = [], [], []
                for n in lanes:
                    g = LANE_ST + hw[n]
                    m_ = float(np.mean(mean_o[(f, n, ii)]))
                    cost_below.append(max(0.0, m_ - (oa - g)))
                    cost_above.append(max(0.0, (ob + g) - m_))
                    v = PIECE[(f, n)]
                    p_ = None
                    for kt_, ot_ in ((v['k0'], v['o0']), (v['k1'], v['o1'])):
                        if ot_ is not None and sa - g <= kt_ * G <= sb + g:
                            p_ = -1 if ot_ < (oa + ob) / 2 else 1
                    pin.append(p_)

                def holds(side, ls):
                    """do lanes `ls` (nearest the island first) fit at pitch in the room on `side`"""
                    used, prev_ = 0.0, None
                    for n in ls:
                        used += (LANE_ST + hw[n]) if prev_ is None else (P_MIN + hw[prev_] + hw[n])
                        prev_ = n
                    return used <= room[side] + 1e-9
                forced = ROOMLESS.get((f, ii))
                best = None
                for j in range(len(lanes) + 1):
                    if forced == 1 and j != 0 or forced == -1 and j != len(lanes):
                        continue
                    if any(pin[i] == 1 for i in range(j)) or any(pin[i] == -1 for i in range(j, len(lanes))):
                        continue
                    cst = sum(cost_below[:j]) + sum(cost_above[j:])
                    if not holds(-1, list(reversed(lanes[:j]))) or not holds(1, lanes[j:]):
                        cst += 1e6                              # over capacity: only if no split fits
                    if best is None or cst < best[0]:
                        best = (cst, j)
                if best is None:
                    continue
                for i, n in enumerate(lanes):
                    SIDE[(f, n, ii)] = -1 if i < best[1] else 1
    for (f, n, k), o_ in sol['o'].items():
        v = PIECE[(f, n)]
        s_ = k * G
        lay = layers_at(n, FR[f]['u'](s_))
        at_end = k - v['k0'] < 2 or v['k1'] - k < 2
        for ii, (sa, sb, oa, ob, Ls, lab, own) in enumerate(boxes[f]):
            if own == n or oa - WIN > o_ or o_ > ob + WIN:
                continue
            g = LANE_ST + hw[n]
            if at_end and not lab.startswith(('end ', 'stub ', 'svia ')):
                continue
            if Ls & lay and sa - g <= s_ <= sb + g:
                # ONE side for the island's whole span: where the lane's own fixed terminal is, when that terminal lies
                # in the span (it cannot move); else its mean first-pass offset over the span -- so a lane that must
                # pass the island's offset does it outside the span, never through the island
                side = SIDE.get((f, n, ii))
                if side is None and (f, ii) in SPLIT:
                    # the split left this lane out (another free interval): it keeps the side it is on -- a row it
                    # meets for free, but binding (SBA0 / SCAS ran through C6's pads with no row at all)
                    mo_ = mean_o.get((f, n, ii))
                    side = -1 if (float(np.mean(mo_)) if mo_ else o_) < (oa + ob) / 2 else 1
                if side is None:
                    for kt_, ot_ in ((v['k0'], v['o0']), (v['k1'], v['o1'])):
                        if ot_ is not None and sa - g <= kt_ * G <= sb + g:
                            side = -1 if ot_ < (oa + ob) / 2 else 1
                if side is None:
                    mo_ = mean_o.get((f, n, ii))
                    side = -1 if (float(np.mean(mo_)) if mo_ else o_) < (oa + ob) / 2 else 1
                if (n, lab) in FLIP:
                    side = -side          # the polish found no room on this side: the lane passes the island on the other
                forced = ROOMLESS.get((f, ii))
                pinned = any(ot_ is not None and sa - g <= kt_ * G <= sb + g
                             for kt_, ot_ in ((v['k0'], v['o0']), (v['k1'], v['o1'])))
                if forced and (f, ii) not in SPLIT and not pinned:
                    side = forced             # never against the lane's own fixed end
                out.append((f, n, k, oa - g, ob + g, side, lab))
            g2 = VIA_ST + hw[n]
            if k in vcol[(f, n)] and sa - g2 <= s_ <= sb + g2:
                out.append((f, n, k, oa - g2, ob + g2, -1 if o_ < (oa + ob) / 2 else 1, 'via ' + lab))
        if k in vcol[(f, n)]:
            for (sa, sb, oa, ob, Ls, lab, own) in vboxes[f]:
                g2 = VIA_ST + hw[n]
                if own != n and sa - g2 <= s_ <= sb + g2 and oa - WIN <= o_ <= ob + WIN:
                    out.append((f, n, k, oa - g2, ob + g2, -1 if o_ < (oa + ob) / 2 else 1, 'via ' + lab))
    return out


log('pass 1')
sol = build_and_solve([])
if sol is None:
    sys.exit('whole_geo: the first pass LP failed (its status above)')
log('pass 2 (static sides)')
SIDES2 = static_sides(sol)
sol = build_and_solve(SIDES2, prev=sol['o'])
if sol is None:          # every rule is elastic: a failure is the solver's, and pass 1 has no static sides or flips
    sys.exit('whole_geo: the second pass LP failed (its status above) -- no geometry without its static sides')


# ---------------------------------------------------------------- output
res = {'lanes': {}, 'vias': [], 'paid': {k: len(v) for k, v in sol['paid'].items()}}
for n in M:
    pieces, xy_all = [], []
    for f in (['T', cls[n]] if n in cls else ['T']):
        v = PIECE[(f, n)]
        sp = FR[f]['sp']; ufun = FR[f]['u']; sfun = FR[f]['s']
        # every column kept: simplified in (s, o), a straight run across a spine corner was drawn from the corner's
        # mitre straight to its far end, which is not the image of a leg whose offset changes (SA13 over SA7)
        keep = [(k * G, sol['o'][(f, n, k)]) for k in range(v['k0'], v['k1'] + 1)]
        cuts = sorted(sfun(cu) for cu in chg[n] if (f == 'T') == (cu <= H0 or n not in cls))
        pts = []
        for (sa, oa), (sb, ob) in zip(keep, keep[1:]):
            pts.append((sa, oa))
            for sc in cuts:
                if sa < sc < sb:
                    pts.append((sc, oa + (ob - oa) * (sc - sa) / (sb - sa)))
        pts.append(keep[-1])
        for (sa, oa), (sb, ob) in zip(pts, pts[1:]):
            if abs(sb - sa) < 1e-9 and abs(ob - oa) < 1e-9:
                continue
            Ly = layer_of(n, ufun((sa + sb) / 2))
            xy = sp.lane_xy([(sa, oa), (sb, ob)])
            if xy_all and pieces and len(xy) >= 2:
                # inside a spine corner the two segments' offset lines overlap: a column point stepped back from
                # the last one drawn is a mapping artifact, not a shape -- drop the reversing vertex
                (px, py), (qx, qy), (rx, ry) = (pieces[-1][0], pieces[-1][1]), tuple(xy[0]), tuple(xy[-1])
                d1, d2 = (qx - px, qy - py), (rx - qx, ry - qy)
                l1, l2 = math.hypot(*d1), math.hypot(*d2)
                if l1 > 1e-9 and l2 > 1e-9 and (d1[0] * d2[0] + d1[1] * d2[1]) / (l1 * l2) < -0.5 \
                        and min(l1, l2) < P_MIN and pieces[-1][4] == LNAME[Ly]:
                    pieces[-1] = (px, py, rx, ry, pieces[-1][4]); xy_all[-1] = (rx, ry)
                    continue
            for p, q in zip(xy, xy[1:]):
                pieces.append((p[0], p[1], q[0], q[1], LNAME[Ly]))
            xy_all += xy if not xy_all else xy[1:]
    # the exact tooth and berth REPLACE the first and last column points (the columns round them to G along s:
    # appended, a column behind the tooth made a hairpin tooth -> back -> forward)
    (tx, ty), (ex, ey) = TERM[n]
    if pieces:
        a = pieces[0]; pieces[0] = (tx, ty, a[2], a[3], a[4]); xy_all[0] = (tx, ty)
        z = pieces[-1]; pieces[-1] = (z[0], z[1], ex, ey, z[4]); xy_all[-1] = (ex, ey)
    # reversing vertices next to a short side (mapping artifacts inside spine corners, at a replaced terminal):
    # dropped over the finished line, while the two sides share a layer
    changed = True
    while changed and len(pieces) >= 2:
        changed = False
        for i in range(len(pieces) - 1):
            a, b = pieces[i], pieces[i + 1]
            if a[4] != b[4]:
                continue
            d1, d2 = (a[2] - a[0], a[3] - a[1]), (b[2] - b[0], b[3] - b[1])
            l1, l2 = math.hypot(*d1), math.hypot(*d2)
            if l1 < 1e-9 or l2 < 1e-9:
                pieces[i:i + 2] = [(a[0], a[1], b[2], b[3], a[4])]; changed = True; break
            if (d1[0] * d2[0] + d1[1] * d2[1]) / (l1 * l2) < -0.5 and min(l1, l2) < P_MIN:
                pieces[i:i + 2] = [(a[0], a[1], b[2], b[3], a[4])]; changed = True; break
    xy_all = [(pieces[0][0], pieces[0][1])] + [(pc[2], pc[3]) for pc in pieces]
    # collinear pieces merged in BOARD coordinates, same layer only
    merged = []
    for pc in pieces:
        if merged and merged[-1][4] == pc[4]:
            a = merged[-1]
            cr = (a[2] - a[0]) * (pc[3] - a[1]) - (a[3] - a[1]) * (pc[2] - a[0])
            if abs(cr) < 1e-9 and abs(a[2] - pc[0]) < 1e-9 and abs(a[3] - pc[1]) < 1e-9:
                merged[-1] = (a[0], a[1], pc[2], pc[3], a[4])
                continue
        merged.append(pc)
    pieces = merged
    res['lanes'][n] = {'xy': xy_all, 'pieces': pieces}
for (f, n, cu, kc) in sol['vias']:
    s_c = FR[f]['s'](cu)
    k0_ = int(math.floor(s_c / G))
    oa_, ob_ = sol['o'].get((f, n, k0_)), sol['o'].get((f, n, k0_ + 1))
    o_c = oa_ + (ob_ - oa_) * (s_c / G - k0_) if oa_ is not None and ob_ is not None else sol['o'][(f, n, kc)]
    res['vias'].append((n,) + tuple(FR[f]['sp'].xy(s_c, o_c)))     # where the lane changes layer (its polyline vertex)
# the islands a lane could not be kept off, as CUTS for the solve: none of that lane's crossings inside the island's
# span along the route (its clearance added) -- geometry the 1-D solve cannot see, fed back
cuts, seen = [], set()
bxs = {}
for f in FR:
    sp = FR[f]['sp']
    bxs[f] = {}
    for st_ in STATIC:
        (x0, y0, x1, y1, Ls, lab) = st_[:6]
        so = [sp.project_pt(q) for q in ((x0, y0), (x0, y1), (x1, y0), (x1, y1))]
        lo_, hi_ = min(q[0] for q in so), max(q[0] for q in so)
        if lab in bxs[f]:           # a part with pads on one layer and on both is two islands of one label: both spans
            lo_, hi_ = min(lo_, bxs[f][lab][0]), max(hi_, bxs[f][lab][1])
        bxs[f][lab] = (lo_, hi_)
vcuts, vseen = [], set()
for q in sol['paid'].get('static', []):
    _v, f, k, n, what = q[:5]
    if what.startswith('via '):
        # a CHANGE the island would not give its room: the change moves (a via cut), not the lane's crossings
        for (f2, n2, cu, kc) in sol['vias']:
            if f2 == f and n2 == n and abs(kc - k) * G <= VS[n] + G and (n, round(cu, 3)) not in vseen:
                vseen.add((n, round(cu, 3)))
                vcuts.append({'lane': n, 'u': cu, 'w': VS[n]})
        continue
    lab = what
    if lab.startswith(('end ', 'stub ', 'svia ')) or lab not in bxs[f] or (n, f, lab) in seen:
        continue
    seen.add((n, f, lab))
    sa, sb = bxs[f][lab]
    g = LANE_ST + hw[n] + P_MIN
    cuts.append({'lane': n, 'island': lab, 'u_lo': FR[f]['u'](sa - g), 'u_hi': FR[f]['u'](sb + g)})
res['cuts'] = cuts
for q in sol['paid'].get('via', []):
    _v, f, k, n, nb = q[:5]
    for (f2, n2, cu, kc) in sol['vias']:
        if f2 == f and n2 == n and abs(kc - k) * G <= VS[n] + G and (n, round(cu, 3)) not in vseen:
            vseen.add((n, round(cu, 3)))
            vcuts.append({'lane': n, 'u': cu, 'w': VS[n]})
# ...and a pair's dive the room would not give its straight run (paid 'pdive' rows)
for q in sol['paid'].get('pdive', []):
    _v, f, kc, n = q[:4]
    for (f2, n2, cu, kc2) in sol['vias']:
        if f2 == f and n2 == n and kc2 == kc and (n, round(cu, 3)) not in vseen:
            vseen.add((n, round(cu, 3)))
            vcuts.append({'lane': n, 'u': cu, 'w': W_DIVE * G})
res['vcuts'] = vcuts
res['flips'] = sorted(FLIP)
res['changes'] = {n: sorted(J['changes'].get(n, [])) for n in res['lanes']}    # each lane's changes in route u, in order
res['rules'] = {'grid': ctx.cfg.grid_step, 'track': TW, 'clear': CL, 'lane_min': bd.LANE_MIN}
json.dump(res, open(OUT, 'w'))
log(f'cuts for the solve: {[(c_["lane"], c_["island"], round(c_["u_lo"], 2), round(c_["u_hi"], 2)) for c_ in cuts]}; via cuts {[(c_["lane"], round(c_["u"], 2)) for c_ in vcuts]}')
log(f'wrote {OUT}: {len(res["lanes"])} lanes, {len(res["vias"])} vias')
for kind, v in sol['paid'].items():
    log(f'  PAID {kind} {len(v)}: ' + '; '.join(f'{q[0]:.3f} {q[1:]}' for q in sorted(v, reverse=True)[:6]))

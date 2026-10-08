"""whole_geo.py SOLVE.json OUT.json -- per-layer GEOMETRY from a whole-route solve (whole_solve.py), ONE joint LP
over the trunk and both rings: smooth lanes, each at any angle, in their frames.

Frames: the trunk spine (u = s) and each ring's spine (u = Hk + s_b - rs). A lane of class W lives in the trunk
from its tooth to its berth; a lane of class N / S in the trunk from its tooth to its ring's start (s = Hk), then in
its ring from the handoff to its berth -- the two pieces joined by a linear equality (the ring's start offset is an
affine function of the trunk's end offset), a bend term across the join, and the trunk's end held on its ring's side of
the ring's start (elastic, priced W_HARD, `ringside`). Columns every G mm of s in each
frame; the solve fixes the ORDER of the lanes present at every column (below(): launch order, flipped by each
solved crossing) and each lane's LAYER (flipped at each solved change; both layers within a via's reach of one).
  hard (elastic, priced W_HARD, reported): the order of each layer's own lanes (a lane on the other layer may run
  beside, over or under it); same-layer neighbours P_MIN apart (+ half a pair's pitch
  each), slope corrected (tangent cuts of sqrt(1 + k^2) on the pair's mean slope); every neighbour a via's room
  from a change, changes a via pitch apart; inside the board and outside both pad boxes grown by a track's
  clearance (a via: grown by its static room), waived near the lane's own terminals; static copper to one side
  (from a first pass); the slope capped at K_MAX; at most 45 degrees of turn per column.
  soft: the length a lane's sideways moves add, bends, same-layer neighbours short of P_COMF (two lane pitches: the
  room a human leaves), a lane within a clearance more than its bar of static copper.
Two passes: the second holds every lane to ONE side of each piece of static copper near it (static_sides: one split
per island and layer, in the lane order, pinned by the lanes' own ends), and a lane the polish found no room for on
its side is flipped to the other (GEO_FLIPS_FROM=POLISH.json,..: data the polish measured, never typed). What the
LP had to pay is written as CUTS for the solve (the islands a lane could not be kept off, the changes it could not
give their room). The bench from BENCH / NETS / DEST (whole_ctx)."""
import sys, json, math, collections, functools, time
import awx_settings
import numpy as np
from scipy.optimize import linprog
from scipy.sparse import coo_matrix, csr_matrix, hstack
from types import SimpleNamespace
import detmath
import whole_ctx
import whole_frame
import route_layers
import corridor as _cor
import braid as bd
import pairs as _pairs

# every length below is in the design rules' own units: track, clearance, via size, a via's room, the lane pitch
P_MIN = bd.LANE_MIN
# a comfortable pitch: two lane pitches, the room a human leaves between lanes (and a later meander needs) -- lanes
# planned at the bare minimum lost their last hundredths to the snap beside a pair or a pad (K28's SDQ13 by SDQS0)
P_COMF = 2 * bd.LANE_MIN
TW, CL = bd.TRACK, bd.CLEAR
B_M = TW / 2                            # a lane's copper outside the pad box line
VIA_R = bd.VIA_NEED
VIA_VV = bd.VIA_SIZE + CL
VIA_ST = bd.VIA_SIZE / 2 + CL
LANE_ST = TW / 2 + CL
PP = _pairs.pitch(TW)
K_MAX = 8.0                              # the steepest a lane runs to its spine
W_LEN, W_BEND, W_COMF, W_HARD = 1.0, 3.0, 1.0, 1e4
TANG = [0.0, 1.0, -1.0, 2.5, -2.5, 5.0, -5.0]
LEN_T = [0.5, -0.5, 1.0, -1.0, 2.0, -2.0, 4.0, -4.0]   # the slopes a column's added length is cut at
# the tangent cuts under-state sqrt(1 + k^2) between their tangent points: the SLOPED ones are scaled by the set's own
# worst ratio (the flat cut stays exact, so a flat pair is not over-held)
_kk = np.linspace(0.0, K_MAX, 4001)
TSCALE = float(1.0 / min(np.max([(1 + t_ * _kk) / math.sqrt(1 + t_ * t_) for t_ in TANG], axis=0) / np.sqrt(1 + _kk * _kk)))
# the ROUTING LAYERS (route_layers): a layer is its index among them -- F.Cu 0, B.Cu 1, the inner ones after -- and a
# through via's or a hole's copper stands on every one (ALL)
LNAME = route_layers.layers()
NL = len(LNAME)
F = (lambda x: 1 if x == 'B.Cu' else 0) if NL == 2 else LNAME.index
ALL = lambda: set(range(NL))
log = lambda *a: print(*a, flush=True)

J = json.load(open(sys.argv[1]))
OUT = sys.argv[2] if len(sys.argv) > 2 else '/dev/null'
ctx, _cs = whole_ctx.plan()
Fr = whole_frame.build(ctx, awx_settings.req('DEST'))   # the whole route's own frame (whole_frame.py)
M = list(Fr.M)
GRID2 = ctx.cfg.grid_step / 2             # half the router's grid step (the router's bar for a line off the grid)
G = 4 * ctx.cfg.grid_step                 # a column: four router grid steps
# a pair's straight run either side of its via (the longer, diagonal one, and a grid step) and a turn's own straight
# run, in columns
W_DIVE = int(math.ceil((_pairs.via_straight(ctx.cfg, (math.sqrt(0.5), math.sqrt(0.5))) + ctx.cfg.grid_step) / G - 1e-9))
W_TURN = int(math.ceil(_pairs.turn_straight_steps(ctx.cfg) * ctx.cfg.grid_step / G - 1e-9))
# ...and a crossed pair's CROSSOVER (pairs.opposite_hands: its legs swap once, at its first dive): the crossover's own
# runs before and after it (pairs.crossover_shape, on the diagonal as W_DIVE), and its two barrels on ONE side of the
# pair, staggered along it -- no via's room on its other side at all
_XS = _pairs.crossover_shape(ctx.cfg, (math.sqrt(0.5), math.sqrt(0.5)))
W_XB, W_XA = ((int(math.ceil((_XS[1] + ctx.cfg.grid_step) / G - 1e-9)), int(math.ceil((_XS[2] + ctx.cfg.grid_step) / G - 1e-9)))
              if _XS else (W_DIVE, W_DIVE))
XO_B = [(a_, abs(c_)) for a_, c_ in _pairs.crossover_shape(ctx.cfg, (1.0, 0.0))[0]] if _XS else []   # (along, across)
HOLD = max(1, int(round(TW / G)))                 # a tooth / west-face berth stub held straight a track's width
EXC = max(2, int(math.ceil(bd.LANE_MIN / G)))     # the box margin waived a lane pitch from the lane's own terminal
P_MIN = max(P_MIN, TW + CL + 2 * GRID2)      # two planned lines: each lands up to half a grid step off (the router's bar)
# the comfort penalty is GRADED (convex): a millimetre below the midpoint between the minimum and the comfortable pitch
# costs four times one above it, so the LP spreads the tightest neighbours first rather than letting a few pairs take
# all the squeeze
P_MID = (P_MIN + P_COMF) / 2
W_COMF_TIGHT = 3 * W_COMF
# GEO_PAIR_ROOM=1 (opt-in; off by default): an island split that leaves a side holding a PAIR with less than a lane's
# pitch to spare is taken only when every split that fits does the same (static_sides)
PAIR_ROOM = awx_settings.get('GEO_PAIR_ROOM', '0') not in ('', '0')
# GEO_PAIR_TURN_ROOM=N (router grid steps; default 1 on more routing layers than two, 0 on two): a PAIR's neighbours on
# its layer stand N grid steps further off it from its end through its turn onto the lane -- its end run, then two
# 45-degree bends a turning run apart. The snap lays that turn as the pair router turns, on the grid, and a plan at the
# bare bar there left the lane beside it a grid step short (synth w0P: SYN02 0.214 / 0.218 against 0.257 beside SYP0
# turning south off its tooth -- in every order the snap tried, whichever of the two was laid second failed)
PAIR_TURN_ROOM = float(awx_settings.get('GEO_PAIR_TURN_ROOM', '1' if NL > 2 else '0')) * ctx.cfg.grid_step
from fab_tiers import min_via_center_distance
VIA_VV = min_via_center_distance(bd.VIA_SIZE, CL, ctx.cfg.via_drill, getattr(ctx.cfg, 'hole_to_hole_clearance', 0.0) or 0.0)
VIA_VV += 2 * GRID2; VIA_ST += GRID2; LANE_ST += GRID2     # planned vs planned a whole step, vs static half
# a via's room off a lane's line: the router's RING round it (pairs.via_ring) and half a step for each of the two, the
# via and the line, off the grid -- the audit's bar (plan_audit.check_dives) and the polish's; the via's copper and the
# clearance alone left the polish 8 um to find at every via
VIA_R = max(VIA_R, _pairs.via_ring(ctx.cfg) + 2 * GRID2)
M_VIA = VIA_ST
prs = getattr(ctx, 'pairs', {}) or {}
RS = J['rs']; cls = J['branch']; HK = J['Hk']     # HK: each ring's start, where its lanes leave the trunk
li = {n: i for i, n in enumerate(J['launch'])}
cross = {frozenset(k.split('|')): v['u'] for k, v in J['cross'].items()}
# each lane's changes, in route u -- held to its own extent: the solve's step rounds a change at a blocked end's stub
# (its via room there the stub's end alone: whole_solve's LAYER cuts) a part of a step past it, and a via drawn there
# stood up the stub, the lane folding back to it (synth_handoff front_src_row, mix)
chg = {n: sorted(min(max(cu, J['entry'][n]), J['end'][n]) for cu in v) for n, v in J['changes'].items()}
tl = {n: F(ctx.tooth_layer[n]) for n in M}
# a pair's half width: its legs' reach at the snap's 45-degree corners and the half step its off-grid legs take, as the
# polish and the audit price it (half its pitch alone left the polish 23 um a side to find)
hw = {n: (PP / 2 / math.cos(math.pi / 8) + GRID2 if n in prs else 0.0) for n in M}
VS = {n: (VIA_R + PP / 2 if n in prs else VIA_R) for n in M}
# a pair's dive is TWO barrels, pairs.dive_offset either side of its centreline across the lane (where both pair
# routers stand them): its room is a single via's along the lane, widened across by that offset
VX = {n: (_pairs.dive_offset(ctx.cfg, PP / 2) if n in prs else 0.0) for n in M}
ring_sp, bend_xy = Fr.rings, Fr.bend
XO_AT = {n: min(J['changes'].get(n) or [math.inf]) for n in M if n in prs and _pairs.opposite_hands(ctx, n)}
is_xo = lambda n, cu: n in XO_AT and abs(cu - XO_AT[n]) < 1e-9       # the change that is the pair's crossover


def below(a, b, u):
    x = frozenset((a, b))
    return (li[a] < li[b]) ^ (x in cross and cross[x] <= u + 1e-9)


# each lane's runs' layers, where the solve gives them (more routing layers than two: whole_solve's 'layers'); on two
# a lane's layer flips at each change from its tooth's
RUNS = {n: [F(L_) for L_ in ls_] for n, ls_ in (J.get('layers') or {}).items()}
if NL > 2 and set(M) - set(RUNS):
    # (a solve that gives no lane's layers -- a two-layer solve, or a stale one -- read on more: each lane's layer would
    # flip between F and B at its changes, onto layers the solve never chose)
    raise SystemExit(f'whole_geo: {NL} routing layers, and the solve gives no layers for {sorted(set(M) - set(RUNS))}')


def layer_of(n, u):
    if n in RUNS:
        return RUNS[n][sum(1 for cu in chg[n] if cu < u)]
    return tl[n] ^ (sum(1 for cu in chg[n] if cu < u) & 1)


def lay_u(f, n, u):
    """the place along lane n at which its layers are read in frame f, a point of it there at `u` (corridor.piece_u:
    a change is the trunk's up to its ring's handoff HK, the ring's after, as its via is drawn -- the vias, below).
    Read at u itself, a ring piece starting short of its ring's origin took the trunk's last change again: the zynq
    DDR's DQ1 drawn F, B for 0.18 mm, F, two vias 0.15 mm apart"""
    return _cor.piece_u(u, f == 'T', HK[cls[n]]) if n in cls else u


def layers_at(n, u):
    near = [cu for cu in chg[n] if abs(u - cu) <= VS[n]]
    if near:
        if NL == 2:
            return ALL()
        # (more routing layers than two: near a change the lane's track is on the two layers it joins -- its barrel
        # stands on every layer, and the via rows hold every layer's lanes off it, the via-via rows the other vias)
        return {layer_of(n, cu - 1e-6) for cu in near} | {layer_of(n, cu + 1e-6) for cu in near}
    return {layer_of(n, u)}


DST_REF = awx_settings.req('DEST')
SRC_REF = collections.Counter(ctx.src_ref[n] for n in M).most_common(1)[0][0]
BOX = [Fr.SB, Fr.DB]       # the arrays' pad boxes, grown by how far outside them the terminals sit (whole_frame)
log(f'pad boxes grown by the terminals: {[round(b[2] - b[0], 3) for b in BOX]}')


def _pads_box(ref, b):
    """array `ref`'s pads' box, grown by their reach and a clearance -- inside its grown box `b`"""
    ps = ctx.pcb.footprints[ref].pads
    r_ = max(max(p_.size_x, p_.size_y) / 2 for p_ in ps) + CL
    return (max(b[0], min(p_.global_x for p_ in ps) - r_), max(b[1], min(p_.global_y for p_ in ps) - r_),
            min(b[2], max(p_.global_x for p_ in ps) + r_), min(b[3], max(p_.global_y for p_ in ps) + r_))


# ...and by their pads alone: what bounds a lane whose own tooth or berth stands inside its array's grown box (shorter
# than the median the box is grown by) -- its first or last columns are its stub's room beside the longer stubs
PADS_BOX = [_pads_box(SRC_REF, BOX[0]), _pads_box(DST_REF, BOX[1])]
BX0, BY0, BX1, BY1 = ctx.pcb.board_info.board_bounds
EDGE = (float(getattr(ctx.cfg, 'board_edge_clearance', 0.0) or 0.0) or ctx.cfg.clearance) + TW / 2
_SPAN = max(BX1 - BX0, BY1 - BY0)
OS = np.arange(-_SPAN, _SPAN + 1e-9, TW / 6)


def spine_xy_vec(sp, s, O):
    k = sp.seg_of(s)
    t = s - sp.S[k]
    return (sp.P[k, 0] + t * sp.d[k, 0] + O * sp.nrm[k, 0], sp.P[k, 1] + t * sp.d[k, 1] + O * sp.nrm[k, 1])


_ivc = {}


def intervals(ftag, sp, s, margin, own=(False, False)):
    """the free intervals of offset on frame `ftag`'s column at s: inside the board, outside the arrays' boxes by
    `margin` -- the source's (own[0]) or the destination's (own[1]) its pads' box, not its grown one"""
    key = (ftag, round(s, 4), margin, own)
    if key in _ivc:
        return _ivc[key]
    X, Y = spine_xy_vec(sp, s, OS)
    ok = (X >= BX0 + EDGE) & (X <= BX1 - EDGE) & (Y >= BY0 + EDGE) & (Y <= BY1 - EDGE)
    for b in [PADS_BOX[i] if own[i] else BOX[i] for i in (0, 1)]:
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
ISL = {}             # an island's pads on one layer set (a lane goes round an island, never between its pads: a part,
                     # parts no lane can pass between, a two-pad part's pad -- whole_ctx.part_islands)
# (a two-pad part split where a lane fits between its pads only where they stand across the lanes: corridor.pads_across)
_SPINES = {'T': Fr.spine, **{k_: r_ for k_, r_ in Fr.rings.items()}}
_CU = lambda p_: bool(p_.drill and p_.drill > 0) or any(L in p_.layers for L in ('F.Cu', 'B.Cu', '*.Cu'))
ISLAND = whole_ctx.part_islands(ctx, skip=(SRC_REF, DST_REF), split=lambda ref: len(
    [p_ for p_ in ctx.pcb.footprints[ref].pads if _CU(p_)]) == 2 and _cor.pads_across(
    _SPINES, [(p_.global_x, p_.global_y) for p_ in ctx.pcb.footprints[ref].pads if _CU(p_)]))
for ref, fp in ctx.pcb.footprints.items():
    if ref in (SRC_REF, DST_REF):
        continue
    for i_, p in enumerate(fp.pads):
        drilled = bool(p.drill and p.drill > 0)
        if p.pad_type == 'np_thru_hole':
            hx = hy = (p.drill or 0) / 2; Ls = ALL()
        else:
            hx, hy = p.size_x / 2, p.size_y / 2
            Ls = ALL() if drilled else {F(L) for L in LNAME if L in p.layers}
        if Ls and BX0 < p.global_x < BX1 and BY0 < p.global_y < BY1:
            # (a round pad or hole: the cross that covers it, not its box's square -- corridor.round_cover)
            ISL.setdefault((ISLAND[(ref, i_)], frozenset(Ls)), []).extend(
                _cor.round_cover(p.global_x, p.global_y, hx, hy)
                if (p.pad_type == 'np_thru_hole' or p.shape == 'circle') and abs(hx - hy) < 1e-9 else
                [(p.global_x - hx, p.global_y - hy, p.global_x + hx, p.global_y + hy)])
for (ref, Ls), bxs in ISL.items():
    STATIC.append((min(b[0] for b in bxs), min(b[1] for b in bxs), max(b[2] for b in bxs), max(b[3] for b in bxs),
                   set(Ls), ref))
OUTER = LNAME                                # the pages; copper on another inner layer meets a via's barrel, never a lane
VIA_ONLY = []                                # a tooth on an inner layer: a via's piece, on both pages (VSTUB, below)
for (x, y, r, L, nm) in whole_ctx.foreign_teeth(ctx, M, BOX):      # the teeth of the nets outside the bus
    if L in OUTER:
        STATIC.append((x - r, y - r, x + r, y + r, {F(L)}, 'tooth ' + nm))
    else:
        VIA_ONLY.append((x - r, y - r, x + r, y + r, ALL(), 'tooth ' + nm, None))


# every other lane's tooth and berth END on the box line, on its stub's layer (a pair: both legs' ends)
LANE_OF = {}
for n in M:
    LANE_OF[n] = n
    for leg in prs.get(n, ()):
        LANE_OF[leg] = n
LEAVE_ROOM = TW + CL                         # the length the audit reads a turn on
for net, (src, tgt, _ref) in ctx.ends.items():
    if net not in LANE_OF:
        continue
    for k_, (pt, Lnm) in enumerate(((src, ctx.tooth_layer.get(net)), (tgt, ctx.dest_layer.get(net)))):
        if Lnm is None:
            continue
        x, y, L, own = float(pt[0]), float(pt[1]), F(Lnm), LANE_OF[net]
        # ...and the ROOM past it, along its stub's way, where its own lane leaves it and turns: a lane's end kept to a
        # track's width let the lanes beside it pass right at the stub's end, and a lane whose stub points at them had
        # no way out but back against its stub (K28 SCKE0: tooth south into the south-face bundle, folded 126
        # degrees). A stub ending in a via has none: its lane lands on the via and leaves it any way
        if own in prs:
            u = (ctx.tooth_dir if k_ == 0 else ctx.stub_dir).get(own)
        else:
            u = whole_ctx.stub_dir(ctx, net, pt, Lnm)
        ul = math.hypot(*u) if u is not None else 0.0
        x1, y1 = (x + u[0] / ul * LEAVE_ROOM, y + u[1] / ul * LEAVE_ROOM) if ul > 1e-9 else (x, y)
        r = TW / 2
        STATIC.append((min(x, x1) - r, min(y, y1) - r, max(x, x1) + r, max(y, y1) + r, {L}, f'end {own}', own))
# the stubs' COPPER within a via's reach of the box line (a via beside the line meets the stub inside it, not only
# its end): every base segment of a lane's net with an end within that reach, clipped to it -- for VIAS only
NET_LANE = {ctx.byname[n][0]: n for n in M}
for n in M:
    for leg in prs.get(n, ()):
        if leg in ctx.byname:
            NET_LANE[ctx.byname[leg][0]] = n
def stub_pieces(reach, lanes=False):
    """every base segment END within `reach` of a pad box line, clipped to `reach` along the segment; for `lanes`,
    the pages' copper only (a piece on an inner layer is a via's, on both pages)"""
    out = []
    for s_ in ctx.base_segments:
        inner = s_.layer not in OUTER
        if inner and lanes:
            continue
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
            out.append((min(ax, cx) - r, min(ay, cy) - r, max(ax, cx) + r, max(ay, cy) + r,
                        ALL() if inner else {F(s_.layer)},
                        'stub ' + (own_ or ctx.pcb.nets[s_.net_id].name.split('/')[-1]), own_))
    return out


# a VIA beside the box line meets stub copper within its reach; a LANE (never inside the box) only the copper
# within a clearance block of the line
REACH = VIA_ST + bd.VIA_NEED
VSTUB = stub_pieces(REACH) + VIA_ONLY        # (x0, y0, x1, y1, {layer}, label, owner)
LSTUB = stub_pieces(TW + CL, lanes=True)
# base VIAS near the box lines (a stub ending in a via: SDQM1's berth) -- discs on both layers
for v_ in ctx.base_vias:
    own_ = NET_LANE.get(v_.net_id)
    if any(b[0] - REACH <= v_.x <= b[2] + REACH and b[1] - REACH <= v_.y <= b[3] + REACH for b in BOX):
        r = v_.size / 2
        VSTUB.append((v_.x - r, v_.y - r, v_.x + r, v_.y + r, ALL(), 'svia ' + (own_ or ctx.pcb.nets[v_.net_id].name.split('/')[-1]), own_))
# ... and the LANES keep off the stub copper within a clearance block of the line, and off the stub vias
for st_ in LSTUB + [v for v in VSTUB if v[5].startswith('svia ')]:
    STATIC.append(st_)
log(f'stub copper near the box lines: {len(VSTUB)} pieces for vias, {len(LSTUB)} for lanes')


def face_of(xy, box):
    x, y = xy
    d = {'W': abs(x - box[0]), 'E': abs(x - box[2]), 'N': abs(y - box[1]), 'S': abs(y - box[3])}
    return min(d, key=d.get)


# ---------------------------------------------------------------- frames and lane pieces
FR = {'T': dict(sp=Fr.spine, u=lambda s: s, s=lambda u: u)}
for k_, sp in ring_sp.items():
    FR[k_] = dict(sp=sp, u=(lambda s, k_=k_: HK[k_] + (s - RS[k_])), s=(lambda u, k_=k_: RS[k_] + (u - HK[k_])))
TURN_RUN = _pairs.turn_straight_steps(ctx.cfg) * ctx.cfg.grid_step     # a pair's 45-degree turn: its diagonal's run
NTURN = int(math.ceil((2 * TURN_RUN + _pairs.pitch(TW)) / G))    # a pair's turn onto its lane (two 45s), in columns


def _chamfer(pieces, xy_all, t):
    """the corner before the last piece cut t back along the line on either side: the last piece's start moved t on
    toward its end, the line before it cut back t (over as many pieces as that takes), a diagonal between"""
    lx, ly, ex, ey, L = pieces[-1]
    d_out = math.hypot(ex - lx, ey - ly)
    if d_out <= t or len(pieces) < 2:
        return
    c2 = (lx + (ex - lx) * t / d_out, ly + (ey - ly) * t / d_out)
    back, k = t, len(pieces) - 2
    while k >= 0 and pieces[k][4] == L:
        a0, a1, b0, b1, _ = pieces[k]
        ln = math.hypot(b0 - a0, b1 - a1)
        if ln > back:
            c1 = (b0 + (a0 - b0) * back / ln, b1 + (a1 - b1) * back / ln)
            del pieces[k + 1:]
            del xy_all[k + 2:]
            pieces[k] = (a0, a1, c1[0], c1[1], L)
            xy_all[-1] = c1
            pieces.append((c1[0], c1[1], c2[0], c2[1], L)); xy_all.append(c2)
            pieces.append((c2[0], c2[1], ex, ey, L)); xy_all.append((ex, ey))
            return
        back -= ln
        k -= 1


def hold_pair(tips):
    """A PAIR's end is held straight (in columns) for its END RUN (pairs.end_run: its end connector from those tips to
    the pose where the pair router takes over, then the straight the router probes past the pose): the pose lies on
    the plan, and the legs converge along the end's own direction. (SDQS1 turned 66 degrees 0.125 mm before a berth
    whose tips stand 0.45 apart, and the pair router could not leave it.)"""
    return max(HOLD, int(math.ceil(_pairs.end_run(ctx.cfg, tips) / G)))


# the RING STANDOFF: a ring's lanes run along it this far out from where they berth and drop in at their own berths
# only -- as deep as a ring pair's landing (its end run and its turn off the ring), so a pair turns once into its end
# run and the lanes outside it pass beyond its landing. Hugging the face, the innermost pair stepped out to its landing
# and back in, and pressed the lanes outside it into each other and the part beside the face (K35 SDQS0 at DU1's south
# face: SODT0/SODT1 0.141 apart, into C12)
_dc = [(p_.global_x, p_.global_y) for p_ in ctx.pcb.footprints[DST_REF].pads]
DCEN = (sum(x for x, _y in _dc) / len(_dc), sum(y for _x, y in _dc) / len(_dc))


def _out_of(sp, o_at):
    """+1 / -1: which way of offset o in ring frame sp points away from the destination part"""
    return 1.0 if o_at >= sp.project_pt(DCEN)[1] else -1.0


_depths = []
for n in M:
    if n in cls and n in prs:
        sp_ = ring_sp[cls[n]]
        m_ = _pairs.mid(*ctx.pair_ends[n][1])
        _depths.append(abs(sp_.project_pt(Fr.land[n])[1] - sp_.project_pt(m_)[1]))
RING_STANDOFF = max(_depths) if _depths else TURN_RUN + TW + CL

SB0 = {}
PIECE = {}          # (frame, n) -> dict(k0, k1, o0, o1, hold0, hold1, ex0, ex1, ref)
ms_of = lambda n: (np.array([p[0] for p in Fr.mid[n]]), np.array([p[1] for p in Fr.mid[n]]))
for n in M:
    s0, o0 = Fr.st[n]
    f0 = face_of(Fr.tooth[n], BOX[0])
    hold0 = (hold_pair(ctx.pair_ends[n][0]) if n in prs else HOLD) if f0 in ('E', 'W') else 0
    if Fr.start[n] != Fr.tooth[n]:
        # a pair STARTED at the end of its end run out of a side-face tooth (whole_frame) leaves it along the trunk,
        # held over its turn's run and the length the audit reads a turn on, as a ring pair arrives at its landing
        hold0 = max(HOLD, int(math.ceil((TURN_RUN + TW + CL) / G)) + 1)
    ms, mo = ms_of(n)
    ref = (lambda s, ms=ms, mo=mo: float(np.interp(s, ms, mo)))
    if n in cls:
        # its reference ends ON its handoff (the frame stacks the ring's lanes there, outside the destination's corner):
        # the taut path, then 45 degrees onto the handoff offset -- the taut path alone ran inside the corner, where the
        # free intervals it chose kept the lane (K28 SDQ10 across C9's pads at DU1's north-west corner)
        # -- never before its own tooth: a lane crossing the whole trunk had its whole reference drawn through the
        # source's balls (K28 SCKE0)
        hk_, oh_ = HK[cls[n]], Fr.o_h[n]
        dh_ = abs(oh_ - ref(hk_))
        ref = (lambda s, r=ref, a=max(hk_ - dh_, s0), b=hk_, oh=oh_: r(s) if s <= a else float(np.interp(s, [a, b], [r(a), oh])))
        PIECE[('T', n)] = dict(k0=int(round(s0 / G)), k1=int(round(HK[cls[n]] / G)), o0=o0, o1=None, hold0=hold0, hold1=0,
                               ex0=max(hold0 + 1, EXC), ex1=0, ref=ref)
        sp = ring_sp[cls[n]]
        o_h = Fr.o_h[n]                                         # its offset on the arrival line (whole_frame: the order)
        sb0, ob0 = sp.project_pt(Fr.spine.xy(HK[cls[n]], o_h))
        SB0.setdefault(cls[n], []).append(sb0)
        sb1, ob1 = sp.project_pt(Fr.land[n])                   # where the lane lands (whole_frame)
        # a PAIR arrives on its landing's offset, held along the ring over its turn's run (the diagonal the output lays
        # there, _chamfer) and the length the audit reads a turn on (track + clearance) before it, and turns there into
        # its end run: arriving inside it, it climbed out to the landing and folded back into its tips (K28 SDQS1, 113 degrees
        # at DU1's north-west corner)
        hold1 = max(HOLD, int(math.ceil((TURN_RUN + TW + CL) / G)) + 1) if n in prs else 0
        # its reference: from its handoff out to the ring's standoff at 45 degrees (a pair: its landing's offset, which is
        # that deep), held there until its own berth, then in -- a single at 45 degrees onto its stub over the standoff and a
        # turn's length (track + clearance) more, a pair along its hold into its turn
        if n in prs:
            o_run, d_in = ob1, hold1 * G + TW + CL
        else:
            o_run = ob1 + _out_of(sp, ob1) * RING_STANDOFF
            d_in = RING_STANDOFF + TW + CL
        s_run = sb1 - d_in
        s_out = sb0 + abs(o_run - ob0)                          # out to the standoff at 45 degrees past the handoff
        rpts = ([(sb0, ob0), (s_out, o_run), (s_run, o_run), (sb1, ob1)] if s_out < s_run else
                [(sb0, ob0), (s_run, o_run), (sb1, ob1)] if s_run > sb0 + 1e-9 else [(sb0, ob0), (sb1, ob1)])
        PIECE[(cls[n], n)] = dict(k0=int(round(sb0 / G)), k1=int(round(sb1 / G)), o0=None, o1=ob1, hold0=0, hold1=hold1,
                                  ex0=0, ex1=max(hold1 + 1, EXC),
                                  ref=(lambda s, P=rpts: float(np.interp(s, [q[0] for q in P], [q[1] for q in P]))),
                                  o_h=o_h)
    elif Fr.land[n] != Fr.bend[n]:
        # a pair ending from the trunk on a face across it: to its LANDING, as a ring pair (whole_frame), held and
        # turned there
        s1, o1 = Fr.spine.project_pt(Fr.land[n])
        hold1 = max(HOLD, int(math.ceil((TURN_RUN + TW + CL) / G)) + 1)
        PIECE[('T', n)] = dict(k0=int(round(s0 / G)), k1=int(round(s1 / G)), o0=o0, o1=o1, hold0=hold0, hold1=hold1,
                               ex0=max(hold0 + 1, EXC), ex1=max(hold1 + 1, EXC), ref=ref)
    else:
        s1, o1 = Fr.se[n]
        f1 = face_of(bend_xy[n], BOX[1])
        hold1 = (hold_pair(ctx.pair_ends[n][1]) if n in prs else HOLD) if f1 in ('E', 'W') else 0
        PIECE[('T', n)] = dict(k0=int(round(s0 / G)), k1=int(round(s1 / G)), o0=o0, o1=o1, hold0=hold0, hold1=hold1,
                               ex0=max(hold0 + 1, EXC), ex1=max(hold1 + 1, EXC), ref=ref)


# each ring's START on the trunk's handoff line (whole_frame: a lane pitch inside its innermost lane, clear of the
# destination's corner and the copper hugging it), and the side its lanes stand on: (sign, its trunk offset)
RING_OUT = {}
for f in ring_sp:
    o0_ = Fr.spine.project_pt(ring_sp[f].pts[0])[1]
    RING_OUT[f] = (1.0 if sum(Fr.o_h[n] - o0_ for n in M if cls.get(n) == f) > 0 else -1.0, o0_)
S0C = {f: RS[f] for f in SB0}             # the solve's ring origin: ahead of every lane's trunk end (no step back at the seam)
# each lane of a ring enters it where its trunk ends (sb0, its own column): started at the ring's one origin column
# instead, the stretch from its trunk end to there was in no column -- no pitch, static or via row -- and the output
# joined the two pieces across it straight, through whatever stood there (zynq K42: DQ10 across C98's pad at U2's
# corner, 2.4 mm of ring laid by nothing). The ring's u still starts at its origin, as the solve measures it
for f in S0C:                                               # ... and the ring's u starts THERE
    FR[f]['u'] = (lambda s, f=f: HK[f] + (s - S0C[f]))
    FR[f]['s'] = (lambda u, f=f: S0C[f] + (u - HK[f]))
TERM = {n: (Fr.tooth[n], bend_xy[n]) for n in M}


def lane_mm(f, n, ka, kb, prev):
    """millimetres along lane n's piece in frame f from column ka to kb: its offsets in the pass before (`prev`), else
    its reference path's"""
    v = PIECE[(f, n)]
    ka, kb = max(min(ka, kb), v['k0']), min(max(ka, kb), v['k1'])
    o = lambda k: prev[(f, n, k)] if prev is not None and (f, n, k) in prev else v['ref'](k * G)
    return sum(math.hypot(G, o(k + 1) - o(k)) for k in range(ka, kb))


def lane_lam(f, n, kc, prev, w=3):
    """lane n's millimetres per column's G about column kc (1 running along the spine)"""
    v = PIECE[(f, n)]
    ka, kb = max(kc - w, v['k0']), min(kc + w, v['k1'])
    return max(1.0, lane_mm(f, n, ka, kb, prev) / ((kb - ka) * G)) if kb > ka else 1.0


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
            lay = {n: layers_at(n, lay_u(f, n, u)) for n in od}

            def slopes(n):
                """(p1, p0) for the leaving and the arriving segment at column k"""
                out = []
                if (f, n, k + 1) in var:
                    out.append((var[(f, n, k + 1)], var[(f, n, k)]))
                if (f, n, k - 1) in var:
                    out.append((var[(f, n, k)], var[(f, n, k - 1)]))
                return out
            # the ORDER binds a layer's own lanes only (its pitch rows below): a lane on the other layer may run
            # beside, over or under it -- F over B, as a human stacks them, where one plane order made every B lane
            # detour round an F part with an F lane beside it. A via still keeps its order's side (the via rows)
            def term_o(n):
                """(which end, its fixed offset) when column k is within this lane's terminal zone here"""
                v = pcs[n]
                if k - v['k0'] < v['ex0'] and v['o0'] is not None:
                    return ('start', v['o0'])
                if v['k1'] - k < v['ex1'] and v['o1'] is not None:
                    return ('end', v['o1'])
                return None
            def turning(n):
                """column k within pair n's end run and its turn onto the lane, at an end this piece holds"""
                v = pcs[n]
                return n in prs and PAIR_TURN_ROOM > 0 and (
                    (v['o0'] is not None and k - v['k0'] <= v['hold0'] + NTURN)
                    or (v['o1'] is not None and v['k1'] - k <= v['hold1'] + NTURN))
            for Ly in range(NL):
                seq = [n for n in od if Ly in lay[n]]
                for a, b in zip(seq, seq[1:]):
                    sep = P_MIN + hw[a] + hw[b] + (PAIR_TURN_ROOM if turning(a) or turning(b) else 0.0)
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
                        al, be = 1 / math.sqrt(1 + t0_ * t0_), t0_ / math.sqrt(1 + t0_ * t0_)
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
            if f == 'T' and n in cls and cu > HK[cls[n]]:
                continue
            if f != 'T' and cu <= HK[f]:
                continue
            s_c = sfun(cu)
            kc = int(round(s_c / G))
            if not (v['k0'] <= kc <= v['k1']):
                continue
            vias.append((f, n, cu, kc))
            jc = var[(f, n, kc)]
            r = VIA_R if VX[n] else VS[n]
            # (a plain dive: its barrel(s) at the site, VX either side; a CROSSOVER: its two barrels along it, on the
            # side with the more room in the first pass -- both sides in the first pass itself)
            bar_ = [(0.0, VX[n], 0)]
            if is_xo(n, cu):
                side_ = 0
                od_c = orders.get((f, kc), [])
                if prev is not None and n in od_c:
                    ic = od_c.index(n)
                    o_n = prev.get((f, n, kc), 0.0)
                    up_ = min((prev.get((f, od_c[j], kc), o_n) - o_n for j in range(ic + 1, len(od_c))), default=9.0)
                    dn_ = min((o_n - prev.get((f, od_c[j], kc), o_n) for j in range(ic - 1, -1, -1)), default=9.0)
                    side_ = 1 if up_ >= dn_ else -1
                bar_ = [(a_, c_, side_) for a_, c_ in XO_B]
            for (a_b, c_b, sd_b) in bar_:
              s_b = s_c + a_b
              jb_ = var.get((f, n, int(round(s_b / G))), jc)     # the lane where the barrel stands along it
              for k in range(int(math.floor((s_b - r) / G)), int(math.ceil((s_b + r) / G)) + 1):
                if (f, k) not in orders or (f, n, k) not in var:
                    continue
                ds = k * G - s_b
                if abs(ds) >= r:
                    continue
                h = math.sqrt(r * r - ds * ds) + c_b
                od = orders[(f, k)]
                i = od.index(n)
                # the neighbour's LINE a via's room off: its offset at this column, slope corrected
                # (tangent cuts of sqrt(1 + k^2) on the neighbour's own slope at the column)
                # the TWO nearest lanes each side: in a stacked F/B comb the nearest can share the via lane's offset
                # on the other layer, and the lane that matters is the next one
                nbs = [od[j] for j in (i + 1, i + 2, i - 1, i - 2) if 0 <= j < len(od)]
                # ...and each side's nearest lane ON EACH LAYER: the order binds a layer's own lanes only, so a lane
                # further along it on one layer is held off this via by nothing else (by the pitch alone, 0.257 where
                # a via asks 0.3235)
                uk = FR[f]['u'](k * G)
                for side_ in (range(i + 1, len(od)), range(i - 1, -1, -1)):
                    for Ly in range(NL):
                        nb_ = next((od[j] for j in side_ if Ly in layers_at(od[j], lay_u(f, od[j], uk))), None)
                        if nb_ is not None and nb_ not in nbs:
                            nbs.append(nb_)
                for nb in nbs:
                    up = od.index(nb) > i
                    if sd_b and (sd_b > 0) != up:
                        continue                     # (a crossover's barrels are all on its other side)
                    jm = var[(f, nb, k)]
                    segs_ = [(var[(f, nb, k + 1)], jm)] if (f, nb, k + 1) in var else []
                    segs_ += [(jm, var[(f, nb, k - 1)])] if (f, nb, k - 1) in var else []
                    need = h + hw[nb]
                    vn_ = PIECE[(f, nb)]
                    if k - vn_['k0'] < vn_['ex0'] or vn_['k1'] - k < vn_['ex1']:
                        need += G / 4          # its terminal is drawn exact, up to half a column from its column
                    sg = 1.0 if up else -1.0
                    # up: o_m - o_v >= need * (al + be * k_m)   down: o_v - o_m >= need * (al - be * k_m)
                    le([(jb_, sg), (jm, -sg)], -need, ('via', f, k, n, nb))
                    for (j1, j0) in segs_:
                        for t0_, sc_ in tangents([(j1, j0)]):
                            al, be = sc_ / math.sqrt(1 + t0_ * t0_), sc_ * t0_ / math.sqrt(1 + t0_ * t0_)
                            terms = [(jb_, sg), (jm, -sg), (j1, sg * need * be / G), (j0, -sg * need * be / G)]
                            le(terms, -need * al, ('via', f, k, n, nb))
    for i, (f, n, cu, kc) in enumerate(vias):
        for (f2, m_, cu2, kc2) in vias[i + 1:]:
            ds = abs(FR[f]['s'](cu) - FR[f2]['s'](cu2)) if f2 == f else math.inf
            if f2 != f or m_ == n or ds >= VIA_VV:
                continue
            h = math.sqrt(VIA_VV * VIA_VV - ds * ds) + VX[n] + VX[m_]     # a pair's barrels stand VX across
            km = (kc + kc2) // 2
            od = orders.get((f, km), [])
            if n not in od or m_ not in od:
                continue
            a, b = (n, m_) if od.index(n) < od.index(m_) else (m_, n)
            ka, kb = (kc, kc2) if a == n else (kc2, kc)
            le([(var[(f, a, ka)], 1.0), (var[(f, b, kb)], -1.0)], -h, ('viavia', f, km, a, b))
    via_at = {(f, n, kc) for (f, n, cu, kc) in vias}
    # bounds: board, pad boxes (a via further off), the interval the reference lies in -- and where it lies in none
    # (it cuts a box's corner), the interval the lane was in a column before, which it cannot leave across the box,
    # and where that stood on both sides of the box, the side of its nearer terminal (corridor.held_interval)
    held_iv = {}
    # (a lane whose tooth -- or a trunk lane whose berth -- stands inside its array's grown box: bounded by the
    # array's pads on the trunk, where it leaves its tooth or reaches its berth)
    _in = lambda p_, b: b[0] + 1e-6 < p_[0] < b[2] - 1e-6 and b[1] + 1e-6 < p_[1] < b[3] - 1e-6
    OWN = {n: (_in(Fr.tooth[n], BOX[0]), n not in cls and _in(bend_xy[n], BOX[1])) for n in M}
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
        iv = intervals(f, FR[f]['sp'], s_, round(mg, 4), OWN[n] if f == 'T' else (False, False))
        if not iv:
            continue
        lo_, hi_ = held_iv[(f, n)] = _cor.held_interval(
            iv, v['ref'](s_), held_iv.get((f, n)), v['o1'] if v['k1'] - k <= k - v['k0'] else v['o0'])
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
                # ...and at most 45 degrees of TURN per column (its slope changes by at most 1: exact from a straight
                # run, stricter from a steep one), elastic: a bend costs next to nothing against the hard rules, so
                # where they conflicted a lane zigzagged (SDQS0 at K28, north-east then south-east a column apart)
                le([(b, 1.0), (a, -2.0), (p, 1.0)], G, ('turn', f, k, n))
                le([(b, -1.0), (a, 2.0), (p, -1.0)], G, ('turn', f, k, n))
            # ...and a PAIR 45 degrees per TURNING RUN: the pair router turns 45 degrees, then runs its turning radius
            # straight (pairs.turn_straight_steps) before it turns again, so two of its segments W_TURN columns apart
            # differ by 45 degrees at most. An angle is not linear in the offsets: the bound is the slope span of 45
            # degrees centred on the two segments' mean heading in the first pass -- a slope's change is a large turn
            # near the spine's way and a small one across it (bounded in slope, a pair sweeping steeply onto a ring
            # swung its whole bundle 2.7 mm wide, K35 SCK). Turning 45 degrees a column, a pair was bent round other
            # lanes' vias in V's of three columns, 110 to 134 degrees in 0.3 mm: folds the pair router cannot lay
            # (K41 SDQS1)
            if n in prs and prev is not None and k - W_TURN >= v['k0']:
                q0, q1 = var[(f, n, k - W_TURN)], var[(f, n, k - W_TURN + 1)]
                th = sum(math.atan((prev.get(inv[y1], 0.0) - prev.get(inv[y0], 0.0)) / G) for (y1, y0) in ((b, a), (q1, q0))) / 2
                lo_t, hi_t = th - math.pi / 8, th + math.pi / 8
                allow = 2 * K_MAX if max(abs(lo_t), abs(hi_t)) >= math.atan(2 * K_MAX) else min(math.tan(hi_t) - math.tan(lo_t), 2 * K_MAX)
                le([(b, 1.0), (a, -1.0), (q1, -1.0), (q0, 1.0)], allow * G, ('pturn', f, k, n))
                le([(b, -1.0), (a, 1.0), (q1, 1.0), (q0, -1.0)], allow * G, ('pturn', f, k, n))
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
        wb, wa = (W_XB, W_XA) if is_xo(n, cu) else (W_DIVE, W_DIVE)     # (a crossover's runs are its own)
        if NL > 2:
            # (more routing layers than two: the runs and the end's reach in millimetres ALONG THE LANE, not in columns --
            # a lane running along the columns, down a channel the spine crosses, covers several millimetres a column:
            # counted in columns, synth wwP's SYP2 was still 'at its tooth' millimetres down the channel, held at its
            # tooth's offset through its dive, and its dive's straight run paid a millimetre)
            lam_ = lane_lam(f, n, kc, prev)
            near1 = v['o1'] is not None and lane_mm(f, n, kc, v['k1'], prev) <= (v['hold1'] + wa + W_TURN) * G
            near0 = v['o0'] is not None and lane_mm(f, n, v['k0'], kc, prev) <= (v['hold0'] + wb + W_TURN) * G
            wb, wa = max(1, int(math.ceil(wb / lam_ - 1e-9))), max(1, int(math.ceil(wa / lam_ - 1e-9)))
        else:
            near1 = v['o1'] is not None and v['k1'] - kc <= v['hold1'] + wa + W_TURN
            near0 = v['o0'] is not None and kc - v['k0'] <= v['hold0'] + wb + W_TURN
        if near1:
            for k in range(max(v['k0'], kc - wb), v['k1'] - v['hold1']):
                le([(var[(f, n, k)], 1.0)], v['o1'], tag); le([(var[(f, n, k)], -1.0)], -v['o1'], tag)
            continue
        if near0:
            for k in range(v['k0'] + v['hold0'] + 1, min(v['k1'], kc + wa) + 1):
                le([(var[(f, n, k)], 1.0)], v['o0'], tag); le([(var[(f, n, k)], -1.0)], -v['o0'], tag)
            continue
        for k in range(max(v['k0'] + 1, kc - wb + 1), min(v['k1'] - 1, kc + wa - 1) + 1):
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
    # the handoff joins: ring start = alpha + beta * the trunk's END offset, taken at the trunk's last column (taken on
    # the handoff line, a column short of it, every join was 0.017 mm off across the ring); and a bend across the join
    for n in M:
        if n not in cls:
            continue
        vT, vR = PIECE[('T', n)], PIECE[(cls[n], n)]
        sp = ring_sp[cls[n]]
        o_h, s_e = vR['o_h'], vT['k1'] * G
        ob1, ob2 = sp.project_pt(Fr.spine.xy(s_e, o_h))[1], sp.project_pt(Fr.spine.xy(s_e, o_h + 1.0))[1]
        beta = ob2 - ob1
        alpha = ob1 - beta * o_h
        jT, jR = var[('T', n, vT['k1'])], var[(cls[n], n, vR['k0'])]
        le([(jR, 1.0), (jT, -beta)], alpha); le([(jR, -1.0), (jT, beta)], -alpha)
        # ...and the trunk ENDS on its ring's side of the ring's start, as the frame planned it: ended inside it, a lane
        # cut the destination's corner through the parts hugging it, and its end lay behind the ring's first column,
        # where the ring could not see it -- the lane jumped up to 0.62 mm along the ring to its first column, across
        # whatever stood there (zynq K44: DQ3, DQ12, DQ10 and DQ1 through C98's pads)
        sg_, o0_ = RING_OUT[cls[n]]
        le([(jT, -sg_)], -sg_ * o0_, ('ringside', 'T', vT['k1'], n))
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
        # ...and a clearance more of air, soft (comfort): a lane planned at the bar lost it to the snap
        e = newvar(W_COMF * G)
        if side < 0:
            le([(j, 1.0), (e, -1.0)], lo_ - CL)
        else:
            le([(j, -1.0), (e, -1.0)], -(hi_ + CL))
    ncol = nv + len(extra)
    cost = np.zeros(ncol); cost[nv:] = extra
    cost += detmath.lp_tie_break(ncol)             # one optimum, not a face: the same plan on every run
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


IPM_ITERS = 500      # the interior point's iterations before the dual simplex takes over (it ends in 58-103 here)


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
    args = (np.concatenate([b, -lo[fl], hi[fh]]),)
    kw = dict(A_ub=hstack([AT[plain], csr_matrix((int(plain.sum()), len(fl) + len(fh)))]).tocsr(), b_ub=c[plain],
              A_eq=hstack([AT[bj], U, W]).tocsr(), b_eq=c[bj], bounds=(0, None))
    res = linprog(*args, **kw, method='highs-ipm', options={'maxiter': IPM_ITERS})
    if res.status != 0:
        # ...an interior point that does not end -- zynq K18's pass 2 ran 2329 iterations in a minute and on for half an
        # hour, residuals at 1e-15 and never declared optimal -- by the dual simplex: the same LP, one optimum
        # (detmath.lp_tie_break), the same point after lp_round
        res = linprog(*args, **kw, method='highs-ds')
    x = np.zeros(n)
    if res.status == 0:
        x[plain] = -res.ineqlin.marginals
        x[bj] = -res.eqlin.marginals
    x = detmath.lp_round(x)
    return SimpleNamespace(status=res.status, message=res.message, x=x, fun=math.fsum(c * x))


ROOMLESS = {}
# (lane, island) pairs the xy polish could not keep clear on the side pass 1 chose: that lane passes the island on
# the other side (the island then stands between two other lanes of the same order; no crossing changes). Read from
# the previous polish output(s) the loop names in GEO_FLIPS_FROM -- data it measured, never typed.
FLIP = set()
for _f in [x for x in awx_settings.get('GEO_FLIPS_FROM', '').split(',') if x]:
    FLIP |= {tuple(x) for x in json.load(open(_f)).get('flips', [])}
WIN = 12 * bd.LANE_MIN                  # an island concerns the lanes within this of it (first pass)


def island_reach(f, sa, sb, oa, ob):
    """[(lo, hi) of the free intervals holding an island, per column] where its sides are measured: its own span -- or,
    where it lies in no free interval there (a stub ending on an array's box line, inside the box's margin), the reach
    of its rows past that span, where the lanes meet it beyond the box (K41: SDQ5's stub on the source's east face;
    judged from the one interval within WIN of it, the north, it forced SCK and SBA0 from the south over 3.7 mm).
    None when it lies in none either way"""
    gx = LANE_ST + max(hw.values())
    for lo, hi in ((sa, sb), (sa - gx, sb + gx)):
        out = []
        for s_ in np.arange(lo, hi + 1e-9, G):
            iv = intervals(f, FR[f]['sp'], float(s_), round(B_M, 4))
            around = [q for q in iv if q[0] <= oa + 1e-6 and q[1] >= ob - 1e-6]
            if around:
                out.append((min(q[0] for q in around), max(q[1] for q in around)))
        if out:
            return out
    return None


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
    # ONE SIDE ON THE BOARD: a part both a trunk and a ring see (one at the destination's corner, where the trunk hands
    # its lanes to the ring) is decided in its HOME frame -- the one that sees it whole, its box not cut at the frame's
    # ends, nearest its spine -- and carried into every other frame by a point on that side. Each frame's offset runs
    # its own way round the part: decided in each, the trunk sent DQ10 below C98 and the ring above it, the handoff
    # crossed C98 between the two, and a flip flipped both and crossed it again (zynq K44)
    home, carry = {}, {}
    spines = {f: FR[f]['sp'] for f in FR}
    for ii, st_ in enumerate(STATIC):
        (x0, y0, x1, y1) = st_[:4]
        c_ = ((x0 + x1) / 2, (y0 + y1) / 2)
        h = home[ii] = _cor.part_home(spines, {f: boxes[f][ii][:2] for f in FR}, c_)
        for f in FR:
            carry[(ii, f)] = 1 if f == h else _cor.side_carry(spines[h], spines[f], c_, boxes[h][ii][3], LANE_ST)
    # each part's own pads (an island's: whole_ctx.part_islands joins parts no lane passes between), its box else
    RECTS = [ISL.get((st_[5], frozenset(st_[4]))) or [tuple(st_[:4])] for st_ in STATIC]
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
            reach = island_reach(f, sa, sb, oa, ob)
            if reach is None:
                # in no free interval anywhere its rows reach: the intervals within WIN of it
                reach = []
                for s_ in np.arange(sa, sb + 1e-9, G):
                    iv = intervals(f, FR[f]['sp'], float(s_), round(B_M, 4))
                    around = [q for q in iv if q[1] >= oa - WIN and q[0] <= ob + WIN]
                    if around:
                        reach.append((min(q[0] for q in around), max(q[1] for q in around)))
            for lo_i, hi_i in reach:
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
    SPLITS = {}                                     # (frame, island, layer): its lanes, their costs, pins
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
            for lo_i, hi_i in island_reach(f, sa, sb, oa, ob) or ():
                room[-1] = min(room[-1], oa - lo_i)
                room[1] = min(room[1], hi_i - ob)
                span = [max(span[0], lo_i), min(span[1], hi_i)]
            if f != 'T':
                # on a RING the lanes run the ring's standoff out from the destination's face (its pairs' landings
                # lie that deep): the room between the island and the face is that much less (K35 C12 at DU1's south
                # face: four F lanes and a pair kept inside it, the pair pressed onto the face and folded to its landing)
                room[-int(_out_of(FR[f]['sp'], (oa + ob) / 2))] -= RING_STANDOFF
            SPLIT.add((f, ii))
            for Ly in range(NL):
                if Ly not in Ls:
                    continue
                # only the lanes that share the island's free interval can reach it; one in another interval is
                # kept off it by that interval's own bounds (SA12, 2.6 mm above SVREF's stub, was told to pass below)
                lanes = [n for n in od if n != own and Ly in layers_at(n, lay_u(f, n, um)) and (f, n, ii) in mean_o
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
                    if NL > 2 and (f, n, ii) in SIDE:
                        # (more routing layers than two: a lane near its change is on two layers' lists -- the side an
                        # earlier layer's split gave it stands in this one, one side of the island for the lane)
                        p_ = SIDE[(f, n, ii)]
                    pin.append(p_)

                def spare(side, ls):
                    """the room lanes `ls` (nearest the island first) leave on `side` at pitch: below 0, they do not fit"""
                    used, prev_ = 0.0, None
                    for n in ls:
                        used += (LANE_ST + hw[n]) if prev_ is None else (P_MIN + hw[prev_] + hw[n])
                        prev_ = n
                    return room[side] - used
                forced = ROOMLESS.get((f, ii))
                best = None
                for j in range(len(lanes) + 1):
                    if forced == 1 and j != 0 or forced == -1 and j != len(lanes):
                        continue
                    if any(pin[i] == 1 for i in range(j)) or any(pin[i] == -1 for i in range(j, len(lanes))):
                        continue
                    cst = sum(cost_below[:j]) + sum(cost_above[j:])
                    sp_b, sp_a = spare(-1, list(reversed(lanes[:j]))), spare(1, lanes[j:])
                    if sp_b < -1e-9 or sp_a < -1e-9:
                        cst += 1e6                              # over capacity: only if no split fits
                    elif PAIR_ROOM and ((sp_b < P_MIN and any(n in prs for n in lanes[:j]))
                                        or (sp_a < P_MIN and any(n in prs for n in lanes[j:]))):
                        # FULL beside a PAIR: its lanes fit at their bare pitch and leave less than another lane's. A
                        # single is laid within the half step its bar allows; a pair turns at its own radius and is laid
                        # up to a pitch off its line, taking its neighbours' room (K51: SDQS0, SDQ7 and SDQ6 between
                        # DU1's south face and C12, 0.056 spare). Taken only when every split that fits is as full
                        cst += 1e3
                    if best is None or cst < best[0]:
                        best = (cst, j)
                if best is None:
                    continue
                SPLITS[(f, ii, Ly)] = (lanes, cost_below, cost_above, pin)
                for i, n in enumerate(lanes):
                    SIDE[(f, n, ii)] = -1 if i < best[1] else 1
    # ...and the GAP between two islands. Each split measures its room to the ends of the free interval its island
    # stands in, which the board's edge and the arrays' boxes bound and no other island does: two islands one above the
    # other were each told there was room on the side facing the other, and the lanes told to pass between them were as
    # many as both rooms held, not as the gap holds (K41 with the stub check per layer: SA4, SA8 and SA11 sent between
    # C4/C3 and R4+R5, 0.40 mm apart on the board -- one lane's room -- six static findings and, the cuts they made, no
    # plan). Two islands side by side along the lanes (their spans overlap), one above the other with no island between,
    # hold between them only the lanes that fit the gap at pitch. Past that, the gap's lowest lane is moved below the
    # lower island or its highest above the upper one -- whichever moves its lane less from its first-pass offset, never
    # against a lane's own fixed end nor to a side with no room -- and never back across an island it was moved across,
    # so a lane pushed out of a gap goes on outward until a gap or a side holds it
    def gap_used(ls):
        used, prev_ = 0.0, None
        for n in ls:
            used += (LANE_ST + hw[n]) if prev_ is None else (P_MIN + hw[prev_] + hw[n])
            prev_ = n
        return used + (LANE_ST + hw[ls[-1]] if ls else 0.0)
    for f, bl in boxes.items():
        for Ly in range(NL):
            idx = [ii for ii in range(len(bl)) if (f, ii, Ly) in SPLITS]
            gaps = []
            for a in idx:
                for b in idx:
                    lo_s, hi_s = max(bl[a][0], bl[b][0]), min(bl[a][1], bl[b][1])
                    gap = bl[b][2] - bl[a][3]
                    if a == b or hi_s <= lo_s or gap < 0 or gap >= WIN:
                        continue
                    if any(c not in (a, b) and bl[c][0] < hi_s and bl[c][1] > lo_s and bl[c][2] >= bl[a][3] - 1e-9
                           and bl[c][3] <= bl[b][2] + 1e-9 for c in idx):
                        continue
                    # two PARTS' gap, measured on the board between their pads: the frame's boxes are the pads' boxes
                    # taken along its own slanted axes, larger than the copper (K41's C4 / R4+R5: 0.292 in the frame,
                    # 0.400 on the board -- one lane's room, refused)
                    pa_b = ISL.get((bl[a][5], frozenset(bl[a][4])))
                    pb_b = ISL.get((bl[b][5], frozenset(bl[b][4])))
                    if not pa_b or not pb_b:
                        continue
                    gap = min(math.hypot(max(0.0, q[0] - p[2], p[0] - q[2]), max(0.0, q[1] - p[3], p[1] - q[3]))
                              for p in pa_b for q in pb_b)
                    gaps.append((a, b, gap))
            pushed = set()
            for _it in range(50):                        # (each lane crosses each island once at most)
                moved = False
                for (a, b, gap) in gaps:
                    la, cba, caa, pa_ = SPLITS[(f, a, Ly)]
                    lb, cbb, cab, pb_ = SPLITS[(f, b, Ly)]
                    ia, ib = {n: i for i, n in enumerate(la)}, {n: i for i, n in enumerate(lb)}
                    mid = [n for n in la if n in ib and SIDE.get((f, n, a)) == 1 and SIDE.get((f, n, b)) == -1]
                    if not mid or gap_used(mid) <= gap + 1e-9:
                        continue
                    opts = []
                    lo_n, hi_n = mid[0], mid[-1]
                    if pa_[ia[lo_n]] != 1 and ROOMLESS.get((f, a)) != 1 and (lo_n, a) not in pushed:
                        opts.append((cba[ia[lo_n]] - caa[ia[lo_n]], a, lo_n, -1))
                    if pb_[ib[hi_n]] != -1 and ROOMLESS.get((f, b)) != -1 and (hi_n, b) not in pushed:
                        opts.append((cab[ib[hi_n]] - cbb[ib[hi_n]], b, hi_n, 1))
                    if not opts:
                        continue
                    dc, isl, n_, sd = min(opts)
                    SIDE[(f, n_, isl)] = sd
                    pushed.add((n_, isl))
                    log(f'  gap {bl[a][5]} / {bl[b][5]} ({gap:.3f} mm) holds fewer lanes than the {len(mid)} told to pass: '
                        f'{n_} {"below" if sd < 0 else "above"} {bl[isl][5]} ({dc:+.3f} mm)')
                    moved = True
                if not moved:
                    break
    def extent(f, s_, ii, g):
        """(lo, hi): where column s_'s offset line in frame f meets part ii's pads grown by g (None: it misses them)"""
        return _cor.line_extent(FR[f]['sp'], s_, RECTS[ii], g)

    # every column of every piece -- a ring piece re-anchored to the trunk's end (reanchor) has columns the first pass
    # never laid, at the handoff: unchecked, DQ10 ran through C98 there -- its offset between the trunk's end and the
    # first column the pass laid
    cols = []
    for (f, n), v in PIECE.items():
        laid = [k for k in range(v['k0'], v['k1'] + 1) if (f, n, k) in sol['o']]
        if not laid:
            continue
        k1_ = laid[0]
        for k in range(v['k0'], v['k1'] + 1):
            o_ = sol['o'].get((f, n, k))
            if o_ is None:
                if k > k1_:
                    o_ = sol['o'][(f, n, max(k_ for k_ in laid if k_ < k))]
                else:
                    o0_ = v.get('o_start', sol['o'][(f, n, k1_)])
                    o_ = o0_ + (sol['o'][(f, n, k1_)] - o0_) * (k - v['k0']) / max(1, k1_ - v['k0'])
            cols.append((f, n, k, o_))
    # ONE side per lane and part where the lane meets it in more than one frame (the columns the loop below tests): the
    # home frame's split where the lane is in it -- a split reads the lane order at the part's middle column, where a
    # lane handing off before it is absent (DQ10's trunk ends at s 34.5, C98's middle is 35.1) -- else the frame it meets
    # the part in over most columns, that frame's split or its first-pass offset; held in the home frame's terms
    meet = collections.defaultdict(lambda: collections.defaultdict(list))      # (n, ii) -> frame -> [o_]
    for (f, n, k, o_) in cols:
        v = PIECE[(f, n)]
        s_ = k * G
        lay = layers_at(n, lay_u(f, n, FR[f]['u'](s_)))
        at_end = (k - v['k0'] < 2 and v['o0'] is not None) or (v['k1'] - k < 2 and v['o1'] is not None)
        for ii, (sa, sb, oa, ob, Ls, lab, own) in enumerate(boxes[f]):
            g = LANE_ST + hw[n]
            if (own == n or oa - WIN > o_ or o_ > ob + WIN or not (Ls & lay) or not (sa - g <= s_ <= sb + g)
                    or (at_end and not lab.startswith(('end ', 'stub ', 'svia ')))):
                continue
            ext = extent(f, s_, ii, g)
            if (ext is None or max(ext[0], oa - g) > min(ext[1], ob + g)
                    or max(ext[0] - o_, o_ - ext[1], 0.0) > WIN):
                continue
            meet[(n, ii)][f].append(o_)
    DECIDED = _cor.decide_sides(meet, home, SIDE, {(f, ii): (b_[2] + b_[3]) / 2 for f in boxes
                                                   for ii, b_ in enumerate(boxes[f])}, carry)
    for (f, n, k, o_) in cols:
        v = PIECE[(f, n)]
        s_ = k * G
        lay = layers_at(n, lay_u(f, n, FR[f]['u'](s_)))
        # (a lane's own terminal, its tooth or its berth -- not the handoff between its trunk and ring pieces, whose free
        # end was left unchecked against the islands: zynq K32's DQ3 ended its trunk 0.15 from C98's pad)
        at_end = (k - v['k0'] < 2 and v['o0'] is not None) or (v['k1'] - k < 2 and v['o1'] is not None)
        for ii, (sa, sb, oa, ob, Ls, lab, own) in enumerate(boxes[f]):
            if own == n or oa - WIN > o_ or o_ > ob + WIN:
                continue
            g = LANE_ST + hw[n]
            if at_end and not lab.startswith(('end ', 'stub ', 'svia ')):
                continue
            # (the exact extent REFINES the frame's box and never replaces it: a column's offset line is infinite, and in
            # a ring it met a 2x12 header 23 mm off -- K44, P2, 27 mm paid -- so the box's span gates it and its offsets
            # bound it)
            ext = extent(f, s_, ii, g) if Ls & lay and sa - g <= s_ <= sb + g else None
            if ext is not None:
                ext = (max(ext[0], oa - g), min(ext[1], ob + g))
            if ext is not None and ext[0] <= ext[1] and max(ext[0] - o_, o_ - ext[1], 0.0) <= WIN:
                # ONE side for the island's whole span: where the lane's own fixed terminal is, when that terminal lies
                # in the span (it cannot move); else its mean first-pass offset over the span -- so a lane that must
                # pass the island's offset does it outside the span, never through the island; decided in the part's
                # home frame where the lane meets it there
                side = SIDE.get((f, n, ii))
                carried = (n, ii) in DECIDED
                if carried:
                    side = DECIDED[(n, ii)] * carry[(ii, f)]
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
                if forced and (f, ii) not in SPLIT and not pinned and not carried:
                    side = forced             # never against the lane's own fixed end, nor against its home side
                out.append((f, n, k, ext[0], ext[1], side, lab))
            # (a pair's via is two barrels VX either side of its centreline -- its half width hw is less: 74 um short)
            g2 = VIA_ST + max(hw[n], VX[n])
            if k in vcol[(f, n)] and sa - g2 <= s_ <= sb + g2:
                out.append((f, n, k, oa - g2, ob + g2, -1 if o_ < (oa + ob) / 2 else 1, 'via ' + lab))
        if k in vcol[(f, n)]:
            for (sa, sb, oa, ob, Ls, lab, own) in vboxes[f]:
                g2 = VIA_ST + max(hw[n], VX[n])
                if own != n and sa - g2 <= s_ <= sb + g2 and oa - WIN <= o_ <= ob + WIN:
                    out.append((f, n, k, oa - g2, ob + g2, -1 if o_ < (oa + ob) / 2 else 1, 'via ' + lab))
    return out


log('pass 1')
sol = build_and_solve([])
if sol is None:
    sys.exit('whole_geo: the first pass LP failed (its status above)')
log('pass 2 (static sides)')
# each ring piece starts where its trunk ENDS, as the pass before laid it: the handoff join ties the two pieces'
# offsets, not where along the ring the trunk's end lies, and a trunk end the pass moved off its planned offset lies
# elsewhere along the ring -- the lane then jumped from one to the other across whatever stood between (zynq K32:
# DQ3's trunk end 0.79 mm from its ring start, across C98's corner). The join is linearised about that offset too
def reanchor(sol_):
    for n in M:
        if n in cls and ('T', n, PIECE[('T', n)]['k1']) in sol_['o']:
            vR = PIECE[(cls[n], n)]
            oT = sol_['o'][('T', n, PIECE[('T', n)]['k1'])]
            sb_, ob_ = ring_sp[cls[n]].project_pt(Fr.spine.xy(PIECE[('T', n)]['k1'] * G, oT))
            vR['k0'] = min(max(int(round(sb_ / G)), 0), vR['k1'] - 2)
            vR['o_h'] = oT
            vR['o_start'] = ob_                     # the trunk's end, on the ring: where its first column starts


reanchor(sol)
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
        cuts = sorted(sfun(cu) for cu in chg[n] if (f == 'T') == (n not in cls or cu <= HK[cls[n]]))
        pts, fix = [], set()
        for (sa, oa), (sb, ob) in zip(keep, keep[1:]):
            pts.append((sa, oa))
            for sc in cuts:
                if sa < sc < sb:
                    fix.add(len(pts))
                    pts.append((sc, oa + (ob - oa) * (sc - sa) / (sb - sa)))
        pts.append(keep[-1])
        # ...drawn as its own lines: at a spine corner its two legs meet where they cross (Spine.lane_line), a column
        # past that crossing left out. Drawn through every column, with the mitre of a CONSTANT offset between, a lane
        # inside a corner stepped past it and back -- K44 DQ3, 0.8 mm inside the south ring's first corner and moving
        # further in: a hook of 0.18 mm, turns of 117 and 81 degrees; at the constant offset's fold only, a dip of
        # 0.14 mm. Its terminals and its layer changes (a via stands at its column's point) are always drawn
        line = sp.lane_line(pts, fixed=fix | {0, len(pts) - 1})
        for (xa, ya, sa), (xb, yb, sb) in zip(line, line[1:]):
            if math.hypot(xb - xa, yb - ya) < 1e-9:
                continue
            Ly = layer_of(n, lay_u(f, n, ufun((sa + sb) / 2)))
            xy = [(xa, ya), (xb, yb)]
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
            if pieces and math.hypot(xy[0][0] - pieces[-1][2], xy[0][1] - pieces[-1][3]) > 1e-9:
                # the HANDOFF, drawn: the trunk's end to the ring's first column. The join ties them exactly across
                # the ring and leaves them apart along it -- by the half column the ring's first column rounds to and
                # the move the second pass made from the end it was tied about (K44: up to 0.12 mm). Skipped, the line
                # ran from the trunk's end to the ring's SECOND column, a segment no frame had checked
                pieces.append((pieces[-1][2], pieces[-1][3], xy[0][0], xy[0][1], LNAME[Ly]))
            for p, q in zip(xy, xy[1:]):
                pieces.append((p[0], p[1], q[0], q[1], LNAME[Ly]))
            xy_all += xy if not xy_all else xy[1:]
    # the exact tooth and berth REPLACE the first and last column points (the columns round them to G along s:
    # appended, a column behind the tooth made a hairpin tooth -> back -> forward)
    (tx, ty), (ex, ey) = TERM[n]
    if pieces:
        a = pieces[0]; pieces[0] = (tx, ty, a[2], a[3], a[4]); xy_all[0] = (tx, ty)
        lx, ly = Fr.land[n]
        z = pieces[-1]; pieces[-1] = (z[0], z[1], lx, ly, z[4]); xy_all[-1] = (lx, ly)
        if math.hypot(ex - lx, ey - ly) > 1e-9:          # ...and from its landing straight into its berth
            pieces.append((lx, ly, ex, ey, z[4])); xy_all.append((ex, ey))
            if n in prs and Fr.land[n] != Fr.bend[n]:
                # a ring pair turns off the ring into its end run as the pair router turns: two 45-degree bends a
                # turning radius apart (TURN_RUN either side of the landing), not one 90-degree corner the polish
                # then cut (K28 SDQS1 folded 113 degrees)
                _chamfer(pieces, xy_all, TURN_RUN)
        # ...and at its tooth: a pair started past its end run (whole_frame) runs straight out of its tooth to its
        # start and turns there onto its lane, the same two bends
        sx, sy = Fr.start[n]
        if math.hypot(tx - sx, ty - sy) > 1e-9:
            a = pieces[0]; pieces[0] = (sx, sy, a[2], a[3], a[4]); xy_all[0] = (sx, sy)
            pieces.insert(0, (tx, ty, sx, sy, a[4])); xy_all.insert(0, (tx, ty))
            rp, rx = [(q[2], q[3], q[0], q[1], q[4]) for q in reversed(pieces)], xy_all[::-1]
            _chamfer(rp, rx, TURN_RUN)
            pieces[:] = [(q[2], q[3], q[0], q[1], q[4]) for q in reversed(rp)]; xy_all[:] = rx[::-1]
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
# each island's box and layers (every static label's: a lane the geometry, the audit or the snap found blocked by an
# island not on every routing layer is held off its layers -- whole_route's LAYER cuts)
_ib = {}
for st_ in STATIC:
    x0_, y0_, x1_, y1_, L_ = _ib.get(st_[5], (math.inf, math.inf, -math.inf, -math.inf, set()))
    _ib[st_[5]] = (min(x0_, st_[0]), min(y0_, st_[1]), max(x1_, st_[2]), max(y1_, st_[3]), L_ | set(st_[4]))
lcuts_g, lseen = [], set()
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
    if NL > 2 and lab in _ib and set(_ib[lab][4]) != ALL():
        # (more layers than two: an island NOT on every routing layer -- a part's pads on one face -- is answered under
        # it: the lane held off its layers there, a LAYER cut, rather than kept from crossing in its span and sent round)
        if (n, lab) not in lseen:
            lseen.add((n, lab))
            lcuts_g.append({'lane': n, 'island': lab, 'layer': 1 - min(_ib[lab][4]), 'blocked': sorted(_ib[lab][4]),
                            'box': [round(v_, 4) for v_ in _ib[lab][:4]]})
        continue
    sa, sb = bxs[f][lab]
    g = LANE_ST + hw[n] + P_MIN
    cuts.append({'lane': n, 'island': lab, 'u_lo': FR[f]['u'](sa - g), 'u_hi': FR[f]['u'](sb + g)})
res['cuts'] = cuts
res['lcuts'] = lcuts_g
res['island_boxes'] = {k_: [round(v_[0], 4), round(v_[1], 4), round(v_[2], 4), round(v_[3], 4), sorted(v_[4])]
                       for k_, v_ in _ib.items() if k_ in set(ISLAND.values())}
# each pad's island, for the polish's flips to name the islands this geometry held lanes to (whole_polish.island_of)
res['islands'] = {f'{r_}.{ctx.pcb.footprints[r_].pads[i_].pad_number}': lab_ for (r_, i_), lab_ in ISLAND.items()}
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
            # (its straight run along the lane, in u by its millimetres a column there -- more layers than two)
            vcuts.append({'lane': n, 'u': cu, 'w': (max(W_XB, W_XA) if is_xo(n, cu) else W_DIVE) * G
                          / (lane_lam(f, n, kc, sol['o']) if NL > 2 else 1.0)})
res['vcuts'] = vcuts
# (more routing layers than two) each lane's laid columns as (route u, x, y): where along its route a place on the board
# is, read by the loop to hold each cut to the stretch of the lane's own route that met it (whole_route.own_spans)
if NL > 2:
    res['uxy'] = {}
    for (f, n, k), o_ in sorted(sol['o'].items(), key=lambda kv: (kv[0][1], kv[0][0], kv[0][2])):
        x_, y_ = FR[f]['sp'].xy(k * G, o_)
        res['uxy'].setdefault(n, []).append([round(FR[f]['u'](k * G), 4), round(float(x_), 4), round(float(y_), 4)])
    for n in res['uxy']:
        res['uxy'][n].sort()
res['flips'] = sorted(FLIP)
res['changes'] = {n: list(chg.get(n, [])) for n in res['lanes']}    # each lane's changes in route u, in order
res['rules'] = {'grid': ctx.cfg.grid_step, 'track': TW, 'clear': CL, 'lane_min': bd.LANE_MIN}
json.dump(res, open(OUT, 'w'))
log(f'cuts for the solve: {[(c_["lane"], c_["island"], round(c_["u_lo"], 2), round(c_["u_hi"], 2)) for c_ in cuts]}; via cuts {[(c_["lane"], round(c_["u"], 2)) for c_ in vcuts]}'
    + (f'; layer cuts {[(c_["lane"], c_["island"], [LNAME[b_] for b_ in c_["blocked"]]) for c_ in lcuts_g]}' if lcuts_g else ''))
log(f'wrote {OUT}: {len(res["lanes"])} lanes, {len(res["vias"])} vias')
for kind, v in sol['paid'].items():
    log(f'  PAID {kind} {len(v)}: ' + '; '.join(f'{q[0]:.3f} {q[1:]}' for q in sorted(v, reverse=True)[:6]))

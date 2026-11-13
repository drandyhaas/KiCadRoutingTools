"""whole_embed.py SOLVE.json OUT.json --ref=GEO.json [--draw=PNG] -- the whole route's geometry in BOARD xy: the
solve's braid (whole_solve.py: each lane's layers, the order of the lanes that share a layer, each via's side of every
other lane) fitted into the board's free space with the vias FREE.

Each routing layer's free space -- inside the board, a lane's bar off every piece of fixed copper on that layer --
is cut into triangles on the fixed copper's own corners (CELLS: no point of the mesh stands where nothing is), and so
is the vias' own free space (a via's bar off every layer's copper). A RUN is a lane's stretch on one layer between
two of its ends or vias. It crosses a chain of cells -- which says only which side of each piece of fixed copper it
passes -- and carries its own points: one where it crosses each cell edge (one number, its place along the edge) and
free points between them (each moving ACROSS its lane, inside its cell, so the run never leaves the free space). A
lane leaves its tooth, and reaches its berth, straight along its stub: through its own stub's bar to the first free
point (the stub's MOUTH), where its run begins. A via is a free point too, inside a cell of the vias' space and the
end cells of its two runs.

One LP a round places every via and every lane's points together: the shortest and straightest lanes, with every
two lanes of a layer a pitch apart, every lane a via's room from every other lane's via on the braid's side of it,
vias their own distance apart -- each rule held as a separating line read off the layout as it stands, every point
moving at most its trust a round (halved where a point swings to and fro). Lanes, vias and copper keep a comfortable
margin more wherever there is room for it (soft). A rule may fall short, at a price, but NOTHING PASSES THROUGH
ANYTHING: two lanes of a layer, a lane and a via, two vias keep the sides they stand on, two lanes on one cell edge
their order, and a via never leaves its cells for the fixed copper's bar. A point pressed on an edge of its cell
with free space beyond goes on into the next cell (a lane pushed aside bulges into it), a via likewise; a run that
doubles back on itself with nothing in between is pulled straight. A round's LP holds only what stands near where
the round before moved, and the rounds end when nothing moves.

--ref=GEO.json: the layout it starts from and reads each run's cells off (a whole_geo.py geometry of the same solve).
Where that layout stands inside a bar of fixed copper its line is read out to the free space, the runs kept in the
order its own line has them and put a pitch apart. Where it crosses fixed copper no free cells of the layer lead
round, the run is read as it stands -- a BREACH, two fixed points either side of the copper: no geometry can lay
that lane on that layer there.

OUT.json is written as whole_geo writes a geometry -- the polish, the audit and the snap read it as they read that.
Its findings for the solve: a via cut for each via that stands short of its room, or inside a bar of fixed copper;
a layer cut for each breach (also under 'breach', with the copper's names); under 'tight' every place the slack had
to be given up. Every length the stage chooses is a multiple of the board's own pitch, track width or grid step. A
stage run on its own from a recorded round (replay_stage.py); the loop does not call it. The bench from BENCH / NETS
/ DEST (whole_ctx).

  --draw=PNG / --draw0=PNG   the layout after the rounds / as it starts, exact geometry; --view=X0,Y0,X1,Y1 a window
                             of it, --names the runs' names, --draw-ref the reference's lines over it, --draw-cells
                             the cells, --draw-points each lane's own points
  --rounds=N --h=MM --trust=MM --tol=MM --reach=MM --bend=W --slack=SHARE --wslack=W
                             the rounds at most, a lane's point spacing, a point's move a round, the move that
                             counts as one, how near a move a point is held in the next LP, the bend's price, the
                             slack asked and its price
  --show-paid --show-standing --show-hot --show-kinks
                             each round's largest paid rows by name; the layout's largest shortfalls with their
                             places; where the round moved; where the layout's wiggles are
  --watch=LANE.k,LANE.k      two runs followed round by round: their points, how near they stand, the rows
                             between them and each pull of either"""
import sys, json, math, collections, heapq, time
import awx_settings
import numpy as np
import shapely
from shapely.geometry import box as _box, Point, LineString, Polygon
from shapely.ops import unary_union
from scipy.optimize import linprog
from scipy.sparse import coo_matrix
from scipy.spatial import cKDTree
import whole_ctx
import whole_frame
import whole_passages
import route_layers
import braid as bd
import pairs as _pairs

log = lambda *a: print(*a, flush=True)
ARGS = [a for a in sys.argv[1:] if not a.startswith('--')]
OPT = {a.split('=', 1)[0]: (a.split('=', 1)[1] if '=' in a else '1') for a in sys.argv[1:] if a.startswith('--')}
T0 = time.time()
J = json.load(open(ARGS[0]))
OUT = ARGS[1] if len(ARGS) > 1 else '/dev/null'
REF = json.load(open(OPT['--ref']))
ctx, _cs = whole_ctx.plan()
Fr = whole_frame.build(ctx, awx_settings.req('DEST'))
M = list(Fr.M)
LNAME = route_layers.layers()
NL = len(LNAME)
LI = {L: i for i, L in enumerate(LNAME)}
prs = getattr(ctx, 'pairs', {}) or {}

# ---------------------------------------------------------------- the rules' bars (as whole_geo reads them)
TW, CL = bd.TRACK, bd.CLEAR
GRID2 = ctx.cfg.grid_step / 2
G = 4 * ctx.cfg.grid_step                              # the spacing the written lines are sampled at
PP = _pairs.pitch(TW)
P_MIN = max(bd.LANE_MIN, TW + CL + 2 * GRID2)          # two lanes' centrelines
P_COMF = 2 * bd.LANE_MIN                               # ...and the comfortable pitch
LANE_ST = TW / 2 + CL + GRID2                          # a lane's centreline off fixed copper
VIA_ST = bd.VIA_SIZE / 2 + CL + GRID2                  # a via's centre off fixed copper
VIA_R = max(bd.VIA_NEED, _pairs.via_ring(ctx.cfg) + 2 * GRID2)     # a via's centre off another lane's line
from fab_tiers import min_via_center_distance
VIA_VV = min_via_center_distance(bd.VIA_SIZE, CL, ctx.cfg.via_drill,
                                 getattr(ctx.cfg, 'hole_to_hole_clearance', 0.0) or 0.0) + 2 * GRID2
hw = {n: (PP / 2 / math.cos(math.pi / 8) + GRID2 if n in prs else 0.0) for n in M}      # a pair's half width
VS = {n: (VIA_R + PP / 2 if n in prs else VIA_R) for n in M}                             # a via's room off lane n
W_LEN, W_COMF, W_HARD = 1.0, 1.0, 1e4
GS = ctx.cfg.grid_step                                 # (every length this stage chooses is a multiple of the board's
#                                                        own pitch, track or grid step: none is a millimetre of its own)

# ---------------------------------------------------------------- the braid: layers, order, sides
li = {n: i for i, n in enumerate(J['launch'])}
cross = {frozenset(k.split('|')): v['u'] for k, v in J['cross'].items()}
chg = {n: sorted(min(max(cu, J['entry'][n]), J['end'][n]) for cu in J['changes'].get(n, [])) for n in M}
RUNL = {n: [LI[L_] for L_ in ls_] for n, ls_ in (J.get('layers') or {}).items()}
if NL > 2 and set(M) - set(RUNL):
    raise SystemExit(f'whole_embed: {NL} routing layers, and the solve gives no layers for {sorted(set(M) - set(RUNL))}')
F2 = lambda L_: 1 if L_ == 'B.Cu' else 0
for n in M:
    if n not in RUNL:
        t0 = F2(ctx.tooth_layer[n])
        RUNL[n] = [t0 ^ (k & 1) for k in range(len(chg[n]) + 1)]


def below(a, b, u):
    """lane a below lane b (on its -o side) at route u: the launch order, turned by their crossing once it is past"""
    x = frozenset((a, b))
    return (li[a] < li[b]) ^ (x in cross and cross[x] <= u + 1e-9)


def run_span(n, k):
    """run k of lane n in the solve's route u"""
    return (J['entry'][n] if k == 0 else chg[n][k - 1], J['end'][n] if k == len(chg[n]) else chg[n][k])


# The solve's route u is one coordinate a FRAME: the trunk's, and each ring's own. Two lanes stand side by side at a
# u only where both are in one frame there -- the trunk from a lane's entry to where it leaves it, its own ring from
# the handoff to its end -- and the braid's order is read there alone: a trunk lane and a ring lane at one u, or the
# lanes of two rings, are at two places on the board
CLS, HKJ = J['branch'], J['Hk']


def frame_span(n, f):
    """lane n's stretch of route u in frame f ('T' the trunk, or a ring), None where it is never there"""
    if f == 'T':
        return (J['entry'][n], HKJ[CLS[n]] if n in CLS else J['end'][n])
    return (HKJ[f], J['end'][n]) if CLS.get(n) == f else None


def side_by_side(a, sa, b, sb):
    """the middle of a stretch of route u where lanes a and b stand side by side in one frame, within the spans sa
    and sb of each; None where there is none"""
    for f in ['T'] + sorted(set(CLS.values())):
        fa, fb = frame_span(a, f), frame_span(b, f)
        if fa is None or fb is None:
            continue
        lo, hi = max(sa[0], sb[0], fa[0], fb[0]), min(sa[1], sb[1], fa[1], fb[1])
        if hi - lo > 1e-9:
            return (lo + hi) / 2
    return None


def order_bit(a, ka, b, kb):
    """+1: run kb of lane b stands ABOVE run ka of lane a (on a's +o side, cross(travel, b - a) > 0) wherever the two
    run side by side; -1: below; 0: the solve never has the two side by side"""
    u = side_by_side(a, run_span(a, ka), b, run_span(b, kb))
    if u is None:
        return 0
    return 1 if below(a, b, u) else -1


def via_bit(a, kv, b, kb):
    """+1: run kb of lane b passes ABOVE lane a's via kv (its change kv, in order); -1: below; 0: the solve does not
    have that run beside the via (lane b is in another frame there, or on another of its runs: it has changed layer
    before the via's place, or does after it)"""
    u = chg[a][kv]
    f = 'T' if (a not in CLS or u <= HKJ[CLS[a]] + 1e-9) else CLS[a]
    fb, rb = frame_span(b, f), run_span(b, kb)
    if fb is None or not (max(fb[0], rb[0]) - 1e-9 <= u <= min(fb[1], rb[1]) + 1e-9):
        return 0
    return 1 if below(a, b, u) else -1


# ---------------------------------------------------------------- the fixed copper, and each layer's cells
bus_ids = {ctx.byname[n][0] for n in M} | {ctx.byname[leg][0] for n in M for leg in prs.get(n, ()) if leg in ctx.byname}
CEN = whole_passages.Census(ctx, Fr, LNAME, TW, CL, bd.VIA_SIZE, bd.LANE_MIN, bus_nets=bus_ids)
BX0, BY0, BX1, BY1 = ctx.pcb.board_info.board_bounds
_allxy = np.array([p[:2] for n in M for p in REF['lanes'][n]['xy']])
WIN_M = 4.0                                            # the window: the reference layout and this far round it
WX0, WY0 = max(BX0 + CEN.EDGE, _allxy[:, 0].min() - WIN_M), max(BY0 + CEN.EDGE, _allxy[:, 1].min() - WIN_M)
WX1, WY1 = min(BX1 - CEN.EDGE, _allxy[:, 0].max() + WIN_M), min(BY1 - CEN.EDGE, _allxy[:, 1].max() + WIN_M)
ARRAY = ('src ', 'dest ')                              # an array's own box: its teeth and berths stand ON its line
# the fixed copper, piece by piece: the census's arrays, vias and tracks, and every other part's pads EACH ITS OWN
# piece -- a lane passes between two pads wherever the cells between them are free, so no part needs deciding whole
# (the census boxes a part's pads together as one island, for the frames' columns)
_isl = {lab_ for (lab_, _ls) in CEN.islands}
FIXED = [o for o in CEN.obs if o[5] not in _isl]
import board_rules
for ref_, fp_ in ctx.pcb.footprints.items():
    if ref_ == Fr.src or ref_ in Fr.dests:
        continue
    for p_ in fp_.pads:
        if p_.pad_type == 'np_thru_hole':
            hx_ = hy_ = (p_.drill or 0) / 2 + board_rules.hole_track_clear(ctx.pcb, CL) - CL
            ls_ = frozenset(range(NL))
        else:
            hx_, hy_ = _pairs.pad_extent(p_.size_x / 2, p_.size_y / 2, _pairs.pad_tilt(p_), _pairs.pad_corner_radius(p_))
            ls_ = frozenset(range(NL)) if ((p_.drill or 0) > 0 or '*.Cu' in p_.layers) else \
                frozenset(LI[L_] for L_ in p_.layers if L_ in LI)
        if ls_:
            FIXED.append((p_.global_x - hx_, p_.global_y - hy_, p_.global_x + hx_, p_.global_y + hy_, ls_,
                          f'{ref_}.{p_.pad_number}'))
# ...and the copper that ends ON an array's box line -- every net's stub there, the bus's own lanes' too, and the vias
# stubs end in: the census leaves out what stands inside the boxes, and a box is grown to that line. A stub's last
# stretch (a track and a clearance long, as whole_geo's) is a piece on its layer, a via one on every layer. A lane's
# own end stands inside its own stub's piece, and is read out to the nearest free point
_BOXES = [Fr.SB] + [Fr.DBs[r_] for r_ in Fr.dests]
_near = lambda x, y, g: any((min(abs(x - b[0]), abs(x - b[2])) < g and b[1] - g <= y <= b[3] + g) or
                            (min(abs(y - b[1]), abs(y - b[3])) < g and b[0] - g <= x <= b[2] + g) for b in _BOXES)
_have = {(round(o[0], 4), round(o[1], 4), round(o[2], 4), round(o[3], 4)) for o in FIXED}
NSTUB = 0
for s_ in ctx.base_segments:
    if s_.layer not in LI:
        continue
    for (ax, ay), (bx, by) in (((s_.start_x, s_.start_y), (s_.end_x, s_.end_y)),
                               ((s_.end_x, s_.end_y), (s_.start_x, s_.start_y))):
        if not _near(ax, ay, TW + CL):
            continue
        ln_ = math.hypot(bx - ax, by - ay)
        f_ = min(1.0, (TW + CL) / ln_) if ln_ > 1e-9 else 0.0
        cx, cy, r_ = ax + (bx - ax) * f_, ay + (by - ay) * f_, s_.width / 2
        FIXED.append((min(ax, cx) - r_, min(ay, cy) - r_, max(ax, cx) + r_, max(ay, cy) + r_,
                      frozenset([LI[s_.layer]]), 'stub ' + whole_ctx.copper_name(ctx, s_)))
        NSTUB += 1
for v_ in ctx.base_vias:
    r_ = max(v_.size / 2, _pairs.svia_reach(ctx.cfg, v_.x, v_.y, v_.size))
    k_ = (round(v_.x - v_.size / 2, 4), round(v_.y - v_.size / 2, 4), round(v_.x + v_.size / 2, 4), round(v_.y + v_.size / 2, 4))
    if k_ not in _have and _near(v_.x, v_.y, VIA_ST + bd.VIA_NEED):
        FIXED.append((v_.x - r_, v_.y - r_, v_.x + r_, v_.y + r_, frozenset(range(NL)),
                      'svia ' + whole_ctx.copper_name(ctx, v_)))
        NSTUB += 1


def holes(layer, bar, bar_array):
    """the fixed copper on `layer` (None: on any layer) grown by `bar` -- an array's box by `bar_array` -- as one
    shapely geometry, inside the window"""
    bs = []
    for (x0, y0, x1, y1, ls, lab) in FIXED:
        if layer is not None and layer not in ls:
            continue
        g = bar_array if lab.startswith(ARRAY) else bar
        if x1 + g < WX0 or x0 - g > WX1 or y1 + g < WY0 or y0 - g > WY1:
            continue
        bs.append(_box(x0 - g, y0 - g, x1 + g, y1 + g))
    return unary_union(bs) if bs else Polygon()




# ---------------------------------------------------------------- the cells: triangles on the fixed copper's corners
HPT = float(OPT.get('--h', 3.5 * P_MIN))      # a lane's own points: this far apart along a straight stretch (3.5 pitches)
WINB = _box(WX0, WY0, WX1, WY1)
FREE = [WINB.difference(holes(L_, LANE_ST, TW / 2)) for L_ in range(NL)]      # a lane's centreline, each layer
VFREE = WINB.difference(holes(None, VIA_ST, VIA_ST))                            # a via's centre


class Mesh:
    """a free space cut into triangles on its own corners: V (points), T (each triangle's three points,
    counter-clockwise), N (its neighbour across the edge opposite point i, -1 at a wall)"""

    def __init__(self, free):
        key, V, T = {}, [], []
        for t in shapely.constrained_delaunay_triangles(free).geoms:
            ids = []
            for (x, y) in list(t.exterior.coords)[:3]:
                k = (round(x, 6), round(y, 6))
                if k not in key:
                    key[k] = len(V)
                    V.append((x, y))
                ids.append(key[k])
            if len(set(ids)) < 3:
                continue
            (ax, ay), (bx, by), (cx, cy) = (V[i] for i in ids)
            if (bx - ax) * (cy - ay) - (by - ay) * (cx - ax) < 0:
                ids = [ids[0], ids[2], ids[1]]
            T.append(ids)
        self.V, self.T = np.array(V, float), np.array(T, int)
        edge = collections.defaultdict(list)
        for ti, t in enumerate(self.T):
            for i in range(3):
                a, b = int(t[(i + 1) % 3]), int(t[(i + 2) % 3])
                edge[(min(a, b), max(a, b))].append((ti, i))
        self.N = -np.ones((len(self.T), 3), int)
        for lst in edge.values():
            if len(lst) == 2:
                (t1, i1), (t2, i2) = lst
                self.N[t1, i1], self.N[t2, i2] = t2, t1
        self.free = free
        self.cen = self.V[self.T].mean(axis=1)
        # (comp: the piece of the free space each triangle is in -- two triangles of two pieces, no way between them)
        self.comp = -np.ones(len(self.T), int)
        nc = 0
        for t0 in range(len(self.T)):
            if self.comp[t0] >= 0:
                continue
            self.comp[t0] = nc
            todo = [t0]
            while todo:
                t = todo.pop()
                for t2 in self.N[t]:
                    if t2 >= 0 and self.comp[t2] < 0:
                        self.comp[t2] = nc
                        todo.append(int(t2))
            nc += 1
        self._tree = shapely.STRtree(shapely.polygons(self.V[self.T]))

    def edge(self, ti, i):
        """the two points (their indices) of triangle ti's edge opposite its point i, counter-clockwise: leaving ti
        by it, the first stands on the right (the -o side), the second on the left (+o)"""
        t = self.T[ti]
        return int(t[(i + 1) % 3]), int(t[(i + 2) % 3])

    def to(self, t, t2):
        for i in range(3):
            if self.N[t, i] == t2:
                return i
        raise KeyError((t, t2))

    def inside(self, ti, p, tol=1e-9):
        a, b, c = self.V[self.T[ti]]
        x, y = p
        return ((b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0]) >= -tol and
                (c[0] - b[0]) * (y - b[1]) - (c[1] - b[1]) * (x - b[0]) >= -tol and
                (a[0] - c[0]) * (y - c[1]) - (a[1] - c[1]) * (x - c[0]) >= -tol)

    def locate(self, xy):
        """the triangle holding each point of xy (n x 2), -1 where none does"""
        xy = np.asarray(xy, float).reshape(-1, 2)
        out = -np.ones(len(xy), int)
        qi, ti = self._tree.query(shapely.points(xy), predicate='intersects')
        for q, t in zip(qi[::-1], ti[::-1]):
            out[q] = t
        return out

    def path(self, t_from, t_to, avoid=()):
        """the shortest chain of triangles from t_from to t_to (both included) through none of `avoid`, None where
        there is none"""
        if t_from == t_to:
            return [t_from]
        dist, prev, pq = {t_from: 0.0}, {}, [(0.0, t_from)]
        while pq:
            d, t = heapq.heappop(pq)
            if d > dist.get(t, math.inf):
                continue
            if t == t_to:
                break
            for i in range(3):
                t2 = int(self.N[t, i])
                if t2 < 0 or (t2 in avoid and t2 != t_to):
                    continue
                nd = d + float(np.hypot(*(self.cen[t2] - self.cen[t])))
                if nd < dist.get(t2, math.inf):
                    dist[t2], prev[t2] = nd, t
                    heapq.heappush(pq, (nd, t2))
        if t_to not in prev:
            return None
        out = [t_to]
        while out[-1] != t_from:
            out.append(prev[out[-1]])
        return out[::-1]

    def rows_inside(self, ti):
        """[(nx, ny, c)]: triangle ti as three half planes n . p >= c (n pointing in), in its edges' order"""
        tri = self.V[self.T[ti]]
        out = []
        for i in range(3):
            a, b = tri[(i + 1) % 3], tri[(i + 2) % 3]                # the edge opposite point i
            nv = (-(b[1] - a[1]), b[0] - a[0])
            nl = math.hypot(*nv)
            out.append((nv[0] / nl, nv[1] / nl, (nv[0] * a[0] + nv[1] * a[1]) / nl))
        return out


MESH = [Mesh(f_) for f_ in FREE]
VMESH = Mesh(VFREE)
log(f'fixed copper: {len(FIXED)} piece(s), {NSTUB} of them stubs and vias on the arrays\' box lines; cells: '
    + ', '.join(f'{LNAME[L_]} {len(MESH[L_].T)}' for L_ in range(NL)) + f', vias {len(VMESH.T)}  ({time.time() - T0:.1f} s)')

# ---------------------------------------------------------------- the reference layout: each lane's runs and vias
RUN = {}             # (lane, k) -> {'L': layer, 'ref': [(x, y)..], 'pts': its own points, 'first' / 'last': end cells}
VIA = {}             # (lane, kv) -> its place
for n in M:
    runs = []
    for (x0, y0, x1, y1, L_) in REF['lanes'][n]['pieces']:
        if runs and runs[-1][0] == LI[L_]:
            runs[-1][1].append((x1, y1))
        else:
            runs.append([LI[L_], [(x0, y0), (x1, y1)]])
    if [r[0] for r in runs] != RUNL[n]:
        raise SystemExit(f'whole_embed: {n}: the reference runs on {[LNAME[r[0]] for r in runs]}, the solve on '
                         f'{[LNAME[x] for x in RUNL[n]]}')
    for k, (L_, pts) in enumerate(runs):
        RUN[(n, k)] = {'L': L_, 'ref': [tuple(p) for p in pts]}
        if k:
            VIA[(n, k - 1)] = tuple(pts[0])
KEYS, VKEYS = sorted(RUN), sorted(VIA)
VIA_REF = dict(VIA)
TV = {}              # each via's cell of the vias' space
for vk in VKEYS:
    t = int(VMESH.locate([VIA[vk]])[0])
    if t < 0:
        q = shapely.ops.nearest_points(VFREE, Point(VIA[vk]))[0]
        t = int(VMESH._tree.nearest(q))
        c = VMESH.cen[t]
        VIA[vk] = (float(q.x + (c[0] - q.x) * 1e-3), float(q.y + (c[1] - q.y) * 1e-3))
    TV[vk] = t
NVIA_MOVED = sum(1 for k in VKEYS if VIA[k] != VIA_REF[k])

# A lane LEAVES ITS TOOTH, and reaches its berth, straight along its stub: through its own stub's bar (its own
# copper) to the first free point of the layer -- the stub's MOUTH. The run begins there (ends there); the straight
# piece between the end and the mouth is the lane's own, written with it. A stub that ends in a via, or one with
# nothing free straight ahead of it within three bars, has no mouth: its run is read from the nearest free point
import whole_ctx as _wc
MOUTH_REACH = 3 * LANE_ST
NMOUTH = 0
for n in M:
    for (key, e_, i_) in (((n, 0), 0, 0), ((n, len(RUNL[n]) - 1), 1, -1)):
        r_ = RUN[key]
        r_.setdefault('lead', None); r_.setdefault('tail', None)
        pt = r_['ref'][i_]
        Ln = LNAME[r_['L']]
        if n in prs and n in getattr(ctx, 'pair_ends', {}):
            us = [_wc.stub_dir(ctx, leg_, ctx.pair_ends[n][e_][j_], Ln) for j_, leg_ in enumerate(prs[n][:2])]
            u = None if any(u_ is None for u_ in us) else ((us[0][0] + us[1][0]) / 2, (us[0][1] + us[1][1]) / 2)
        else:
            u = _wc.stub_dir(ctx, n, pt, Ln)
        ul = math.hypot(*u) if u is not None else 0.0
        if ul < 1e-9:
            continue
        ds = np.arange(0.0, MOUTH_REACH + 1e-9, 0.4 * GS)
        cand = np.array([(pt[0] + u[0] / ul * d_, pt[1] + u[1] / ul * d_) for d_ in ds])
        hit = np.nonzero(MESH[r_['L']].locate(cand) >= 0)[0]
        if not len(hit):
            continue
        d_ = float(ds[hit[0]]) + 0.08 * GS
        mth = (float(pt[0] + u[0] / ul * d_), float(pt[1] + u[1] / ul * d_))
        if MESH[r_['L']].locate([mth])[0] < 0:
            continue
        r_['lead' if e_ == 0 else 'tail'] = tuple(pt)
        r_['ref'][i_] = mth
        NMOUTH += 1


def portal(m, t, t2, x=None):
    """the point of a run where it crosses from triangle t into its neighbour t2: {'k': 'P', a, b (the edge's points:
    a on the run's right), 'len', 't' (its place along it from a), 'from', 'to'}"""
    i = m.to(t, t2)
    a, b = m.edge(t, i)
    A, B = m.V[a], m.V[b]
    ln = float(np.hypot(*(B - A)))
    tt = ln / 2 if x is None else float(min(max(((x[0] - A[0]) * (B[0] - A[0]) + (x[1] - A[1]) * (B[1] - A[1])) / ln, 0.0), ln))
    return {'k': 'P', 'a': a, 'b': b, 'len': ln, 't': tt, 'from': int(t), 'to': int(t2), 'raw': tt, 'psh': False}


def _simplify(pts, tol):
    """pts with every point dropped that stands within tol of the line through the points kept either side of it"""
    if len(pts) < 3:
        return list(pts)
    keep = [False] * len(pts)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        i0, i1 = stack.pop()
        (ax, ay), (bx, by) = pts[i0], pts[i1]
        dx, dy = bx - ax, by - ay
        dl = math.hypot(dx, dy)
        worst, wi = 0.0, None
        for i in range(i0 + 1, i1):
            px, py = pts[i]
            if dl > 1e-12:
                w = min(max(((px - ax) * dx + (py - ay) * dy) / (dl * dl), 0.0), 1.0)
                d = math.hypot(px - ax - w * dx, py - ay - w * dy)
            else:
                d = math.hypot(px - ax, py - ay)
            if d > worst:
                worst, wi = d, i
        if wi is not None and worst > tol:
            keep[wi] = True
            stack += [(i0, wi), (wi, i1)]
    return [p for p, k_ in zip(pts, keep) if k_]


def trace(key, avoid=()):
    """run `key`'s own points, read off its reference line: where it crosses each edge of the chain of cells it
    passes, and a free point every HPT between"""
    n, k = key
    r = RUN[key]
    m = MESH[r['L']]
    pts = list(r['ref'])
    if k:
        pts[0] = VIA[(n, k - 1)]
    if (n, k) in VIA:
        pts[-1] = VIA[(n, k)]
    P = np.array(pts, float)
    sam = [P[0]]
    for a, b in zip(P[:-1], P[1:]):
        ln = math.hypot(*(b - a))
        mm = max(1, int(math.ceil(ln / (0.8 * GS))))
        sam += [a + (b - a) * i / mm for i in range(1, mm + 1)]
    sam = np.array(sam)
    tri = m.locate(sam)
    sam0, psh = sam.copy(), tri < 0              # the line as the reference has it; the samples read out of a bar
    # a stretch of the reference inside the fixed copper's bar (the reference short of it there), or on the board's
    # edge bar: read along the free space's nearest edge, on the side it stands
    r['pushed'] = 0.0
    out_ = np.nonzero(tri < 0)[0]
    if len(out_):
        for j in out_:
            p = Point(sam[j])
            q = shapely.ops.nearest_points(m.free, p)[0]
            d = p.distance(q)
            r['pushed'] = max(r['pushed'], d)
            t_ = int(m._tree.nearest(q))
            c = m.cen[t_]
            sam[j] = (q.x + (c[0] - q.x) * 1e-3, q.y + (c[1] - q.y) * 1e-3)
            tri[j] = t_
    seq, at = [], []                     # the triangles in turn, and the first sample in each
    for j, t in enumerate(tri):
        t = int(t)
        if not seq or seq[-1] != t:
            seq.append(t); at.append(j)
    # (brk: the cell is reached ACROSS fixed copper -- no free cells of the layer join it to the one before: the
    # reference crosses a wall there, and the run is read with a BREACH, two fixed points either side of it)
    full, fat, brk, gaps, bridge = [seq[0]], [at[0]], [False], 0, set()
    for t, j in zip(seq[1:], at[1:]):
        if t in m.N[full[-1]]:
            full.append(t); fat.append(j); brk.append(False)
            continue
        p = (m.path(full[-1], t, avoid) or m.path(full[-1], t)) if m.comp[full[-1]] == m.comp[t] else None
        if p is None:
            full.append(t); fat.append(j); brk.append(True)
            continue
        gaps += 1
        bridge |= set(p[1:-1])
        for t2 in p[1:]:
            full.append(t2); fat.append(j); brk.append(False)
    fat.append(len(sam))
    brk.append(False)
    # out of a cell and straight back within a sample or two is the sampling flickering along the edge: dropped. A
    # longer way out and back is the lane's own shape -- round another lane's via, say -- and is kept
    i = 0
    while i + 2 < len(full):
        if full[i] == full[i + 2] and fat[i + 2] - fat[i + 1] <= 2:
            del full[i + 1:i + 3]; del fat[i + 1:i + 3]; del brk[i + 1:i + 3]
            i = max(i - 1, 0)
        else:
            i += 1
    # where it crosses into each cell after the first
    # (RAW: its place along the edge's LINE read off the reference as it has it -- past the edge's end where the
    # reference stands inside the copper's bar there: lanes read out of one bar to one boundary keep their order by it)
    X, RAW = [None], [None]
    for c in range(1, len(full)):
        if brk[c]:
            X.append(None); RAW.append(None)
            continue
        i_ = m.to(full[c - 1], full[c])
        A, B = m.V[m.edge(full[c - 1], i_)[0]], m.V[m.edge(full[c - 1], i_)[1]]
        j0, j1 = max(fat[c] - 1, 0), min(fat[c], len(sam) - 1)
        sub = sam[j0:j1 + 1]
        x = shapely.ops.nearest_points(LineString(sub) if len(sub) > 1 else Point(sub[0]), LineString([A, B]))[1]
        X.append((x.x, x.y))
        mid_ = (sam0[j0] + sam0[j1]) / 2
        ln_ = float(np.hypot(*(B - A)))
        RAW.append((float(((mid_[0] - A[0]) * (B[0] - A[0]) + (mid_[1] - A[1]) * (B[1] - A[1])) / ln_),
                    bool(psh[j0] or psh[j1])))
    out = []
    r['breach'] = []
    for c in range(len(full)):
        t = full[c]
        if c and brk[c]:
            pa, pb = tuple(float(v) for v in sam[fat[c] - 1]), tuple(float(v) for v in sam[fat[c]])
            out.append({'k': 'X', 'xy': pa, 'tri': full[c - 1], 'jump': True})
            out.append({'k': 'X', 'xy': pb, 'tri': t})
            r['breach'].append((pa, pb))
        elif c:
            out.append(portal(m, full[c - 1], t, X[c]))
            out[-1]['raw'], out[-1]['psh'] = RAW[c]
        # its free points in this cell: a point wherever the reference bends (kept to 0.4 of a grid step of it), and a
        # straight stretch longer than HPT filled -- every point inside the cell
        a0 = (tuple(sam[fat[c]]) if brk[c] else X[c]) if c else tuple(sam[0])
        a1 = (tuple(sam[fat[c + 1] - 1]) if brk[c + 1] else X[c + 1]) if c + 1 < len(full) else tuple(sam[-1])
        mid = [tuple(sam[j]) for j in range(fat[c], fat[c + 1]) if tri[j] == t and not psh[j] and m.inside(t, sam[j], -1e-6)]
        line = _simplify([a0] + mid + [a1], 0.4 * GS)
        for p, q in zip(line, line[1:]):
            ln = math.hypot(q[0] - p[0], q[1] - p[1])
            nn = int(math.ceil(ln / HPT - 1e-9)) if ln > HPT else 1
            for i_ in range(1, nn):
                out.append({'k': 'F', 'xy': (p[0] + (q[0] - p[0]) * i_ / nn, p[1] + (q[1] - p[1]) * i_ / nn), 'tri': t})
            if q is not line[-1]:
                out.append({'k': 'F', 'xy': (float(q[0]), float(q[1])), 'tri': t})
    # (a free point too near the run's end, or the cell edge after it, is no use: dropped)
    r['first'], r['last'], r['gaps'], r['bridge'] = full[0], full[-1], gaps, bridge
    return out


def run_xy(key):
    """run `key`'s line as it stands: its start, its own points, its end"""
    n, k = key
    r = RUN[key]
    m = MESH[r['L']]
    pts = [VIA[(n, k - 1)] if k else r['ref'][0]]
    for p in r['pts']:
        if p['k'] != 'P':
            pts.append(p['xy'])
        else:
            A, B = m.V[p['a']], m.V[p['b']]
            pts.append((float(A[0] + (B[0] - A[0]) * p['t'] / p['len']), float(A[1] + (B[1] - A[1]) * p['t'] / p['len'])))
    pts.append(VIA[(n, k)] if (n, k) in VIA else r['ref'][-1])
    return pts


def layout():
    return {key: run_xy(key) for key in KEYS}


for key in KEYS:
    RUN[key]['pts'] = trace(key)
def copper_between(L_, pa, pb):
    """the names of the fixed copper on layer L_ the straight line from pa to pb passes within a lane's bar of"""
    ln = LineString([pa, pb])
    out = []
    for (x0, y0, x1, y1, ls, lab) in FIXED:
        if L_ in ls and lab not in out and ln.distance(_box(x0, y0, x1, y1)) < LANE_ST - 1e-6:
            out.append(lab)
    return out


BREACH = [(k_, pa_, pb_, copper_between(RUN[k_]['L'], pa_, pb_)) for k_ in KEYS for (pa_, pb_) in RUN[k_]['breach']]
if BREACH:
    log(f'{len(BREACH)} BREACH(ES): the reference crosses fixed copper with no free way round it on the layer, in '
        f'{len({b_[0] for b_ in BREACH})} run(s) -- read as it stands, each a finding for the solve')
    for (k_, pa_, pb_, labs_) in BREACH[:12]:
        log(f'   {k_[0]}.{k_[1]} on {LNAME[RUN[k_]["L"]]} at ({pa_[0]:.2f}, {pa_[1]:.2f}) -> ({pb_[0]:.2f}, {pb_[1]:.2f}): '
            + (', '.join(labs_[:4]) or 'no copper named'))
_deep = sorted(((RUN[k_]['pushed'], k_) for k_ in KEYS if RUN[k_]['pushed'] > TW / 2 + 0.02), reverse=True)
log(f'lanes {len(M)}, runs {len(RUN)}, vias {len(VIA)} ({NVIA_MOVED} moved into the vias\' free space); points: '
    f'{sum(1 for k_ in KEYS for p_ in RUN[k_]["pts"] if p_["k"] == "P")} on cell edges, '
    f'{sum(1 for k_ in KEYS for p_ in RUN[k_]["pts"] if p_["k"] == "F")} free; the reference inside the fixed copper\'s '
    f'bar by more than a track\'s half and 0.02: {len(_deep)} run(s)'
    + (' (' + ', '.join(f'{k_[0]}.{k_[1]} {d_:.3f}' for d_, k_ in _deep[:8]) + ')' if _deep else '')
    + f'  ({time.time() - T0:.1f} s)')


def same_layer_crossings(PL):
    """[(run, run, (x, y))]: where two runs of one layer, of two lanes, cross in the layout PL (never allowed)"""
    segs = []
    for key in KEYS:
        pts = PL[key]
        for i in range(len(pts) - 1):
            if pts[i] != pts[i + 1]:
                segs.append((key, LineString([pts[i], pts[i + 1]])))
    tree = shapely.STRtree([s_ for _, s_ in segs])
    out = []
    for ia, (ka, sa) in enumerate(segs):
        for ib in tree.query(sa, predicate='crosses'):
            kb = segs[ib][0]
            if ib > ia and kb[0] != ka[0] and RUN[kb]['L'] == RUN[ka]['L']:
                c = sa.intersection(segs[ib][1])
                out.append((ka, kb, (round(c.centroid.x, 3), round(c.centroid.y, 3))))
    return out


def edge_lists():
    """{(layer, a, b): [(run, its point, crossing from a to b?)]}: the runs of a layer on each cell edge (a < b)"""
    on = collections.defaultdict(list)
    for key in KEYS:
        for p in RUN[key]['pts']:
            if p['k'] == 'P':
                on[(RUN[key]['L'], min(p['a'], p['b']), max(p['a'], p['b']))].append((key, p, p['a'] < p['b']))
    return on


def draw(path, title='', view=None, marks=()):
    """a picture of the layout (exact geometry): fixed copper dim (grey: on every layer), cell edges faint, F.Cu
    red, B.Cu blue, the inner layers green and purple, vias white rings at their size, `marks` yellow crosses"""
    from PIL import Image, ImageDraw
    x0, y0, x1, y1 = view or (WX0, WY0, WX1, WY1)
    sc = min(2400 / (x1 - x0), 2000 / (y1 - y0))
    W, H = int((x1 - x0) * sc) + 1, int((y1 - y0) * sc) + 1
    im = Image.new('RGB', (W, H), (18, 18, 22))
    dr = ImageDraw.Draw(im)
    Pt = lambda x, y: ((x - x0) * sc, (y - y0) * sc)
    col = [(255, 90, 70), (70, 160, 255), (40, 190, 80), (180, 90, 230), (230, 200, 60), (60, 210, 210)]
    cover = collections.defaultdict(set)
    for (bx0, by0, bx1, by1, ls, lab) in FIXED:
        cover[(bx0, by0, bx1, by1)] |= set(ls)
    for (bx0, by0, bx1, by1), ls in cover.items():
        c_ = (70, 70, 78) if len(ls) == NL else tuple(int(v * 0.33) for v in col[min(ls) % len(col)])
        dr.rectangle([Pt(bx0, by0), Pt(bx1, by1)], fill=c_)
    if '--draw-cells' in OPT:
        for L_ in range(NL):
            c_ = tuple(int(v * 0.3) for v in col[L_ % len(col)])
            for t in MESH[L_].T:
                pts = [Pt(*MESH[L_].V[i]) for i in t]
                dr.line(pts + [pts[0]], fill=c_, width=1)
    PL = layout()
    for L_ in reversed(range(NL)):
        for key in KEYS:
            if RUN[key]['L'] == L_:
                own = ([RUN[key]['lead']] if RUN[key].get('lead') else []) + list(PL[key]) + \
                    ([RUN[key]['tail']] if RUN[key].get('tail') else [])
                dr.line([Pt(*p) for p in own], fill=col[L_ % len(col)], width=max(1, int(round(TW * sc))))
                if '--draw-points' in OPT:
                    for p in PL[key][1:-1]:
                        cx, cy = Pt(*p)
                        dr.ellipse([cx - 2, cy - 2, cx + 2, cy + 2], fill=(255, 255, 255))
    r_ = bd.VIA_SIZE / 2 * sc
    for vk, (x, y) in VIA.items():
        cx, cy = Pt(x, y)
        dr.ellipse([cx - r_, cy - r_, cx + r_, cy + r_], outline=(255, 255, 255), width=max(1, int(sc * 0.02)))
    if '--draw-ref' in OPT:
        for key in KEYS:
            c_ = tuple(min(255, v + 110) for v in col[RUN[key]['L'] % len(col)])
            dr.line([Pt(*p) for p in RUN[key]['ref']], fill=c_, width=1)
    if '--names' in OPT:
        for key in KEYS:
            p = PL[key][len(PL[key]) // 2]
            if x0 <= p[0] <= x1 and y0 <= p[1] <= y1:
                dr.text(Pt(*p), f'{key[0]}.{key[1]}', fill=(255, 255, 255))
    for (x, y) in marks:
        cx, cy = Pt(x, y)
        dr.line([cx - 9, cy - 9, cx + 9, cy + 9], fill=(255, 235, 0), width=3)
        dr.line([cx - 9, cy + 9, cx + 9, cy - 9], fill=(255, 235, 0), width=3)
    if title:
        dr.text((8, 8), title, fill=(255, 255, 0))
    im.save(path)
    log(f'drew {path}')


# a run whose reference went through a gap the fixed copper's bars close was joined round the copper by the shortest
# way: where that way crosses another lane of its layer, the other way round is taken
for _try in range(4):
    xs_ = same_layer_crossings(layout())
    if not xs_:
        break
    for ka, kb, _xy in xs_:
        k_ = max((ka, kb), key=lambda q_: (len(RUN[q_]['bridge']) > 0, RUN[q_]['pushed']))
        if not RUN[k_]['bridge']:
            continue
        keep_ = (RUN[k_]['pts'], RUN[k_]['first'], RUN[k_]['last'], RUN[k_]['gaps'], RUN[k_]['bridge'])
        n0_ = len(xs_)
        RUN[k_]['pts'] = trace(k_, avoid=frozenset(keep_[4]))
        if len(same_layer_crossings(layout())) >= n0_:
            RUN[k_]['pts'], RUN[k_]['first'], RUN[k_]['last'], RUN[k_]['gaps'], RUN[k_]['bridge'] = keep_
        else:
            log(f'   {k_[0]}.{k_[1]}: joined round the fixed copper the other way (the shortest way crossed {(kb if k_ == ka else ka)[0]})')
            break
def _isotonic(z):
    """the nearest (least squares) never-falling list to z"""
    vals, cnt = [], []
    for v in z:
        vals.append(v); cnt.append(1)
        while len(vals) > 1 and vals[-2] > vals[-1]:
            v2, c2 = vals.pop(), cnt.pop()
            vals[-1] = (vals[-1] * cnt[-1] + v2 * c2) / (cnt[-1] + c2)
            cnt[-1] += c2
    return [v for v, c in zip(vals, cnt) for _ in range(c)]


def spread_edges():
    """on each cell edge where a run was read out of a bar, or two stand on one place: the runs in the order the
    reference has them (RAW; two at one place as the braid orders them), a pitch apart where the edge has the room
    (evenly where it has not), each as near its own place as that leaves it. -> the edges spread"""
    import functools
    done = 0
    for (L_, a, b), lst in edge_lists().items():
        ln = lst[0][1]['len']
        can = lambda e_: e_[1]['raw'] if e_[2] else ln - e_[1]['raw']

        def cmp(x, y):
            dx = can(x) - can(y)
            if abs(dx) > 1e-4 or x[0][0] == y[0][0]:
                return -1 if dx < 0 else 1 if dx > 0 else 0
            bit = order_bit(x[0][0], x[0][1], y[0][0], y[0][1])     # +1: y on x's left
            if not bit:
                return 0
            return -1 if ((bit > 0) == x[2]) else 1
        lst.sort(key=functools.cmp_to_key(cmp))
        tight = any(e_[1]['psh'] for e_ in lst) or any(can(y) - can(x) < 2 * SIDE_READ and x[0][0] != y[0][0]
                                                        for x, y in zip(lst, lst[1:]))
        if not tight:
            continue
        gaps = [0.0 if x[0][0] == y[0][0] else P_MIN + hw[x[0][0]] + hw[y[0][0]] for x, y in zip(lst, lst[1:])]
        lo, hi = hw[lst[0][0][0]], ln - hw[lst[-1][0][0]]
        if hi < lo:
            lo = hi = ln / 2
        G_ = sum(gaps)
        if G_ > hi - lo:
            gaps = [g_ * max(hi - lo, 0.0) / G_ * 0.999 for g_ in gaps] if G_ > 0 else gaps
            G_ = sum(gaps)
        cum = [0.0]
        for g_ in gaps:
            cum.append(cum[-1] + g_)
        fit = _isotonic([min(max(can(e_), 0.0), ln) - c_ for e_, c_ in zip(lst, cum)])
        for e_, f_, c_ in zip(lst, fit, cum):
            v = min(max(f_, lo), max(hi - G_, lo)) + c_
            e_[1]['t'] = float(v if e_[2] else ln - v)
        done += 1
    return done


SIDE_READ = 0.4 * GS
NSPREAD = spread_edges()
# two runs of a layer that cross in a cell as read: their free points in that cell given up (the straight lines
# between their places on its edges, which stand in order, do not cross)
for _try in range(4):
    xs_ = same_layer_crossings(layout())
    if not xs_:
        break
    for ka, kb, xy_ in xs_:
        t_ = int(MESH[RUN[ka]['L']].locate([xy_])[0])
        for k_ in (ka, kb):
            RUN[k_]['pts'] = [p_ for p_ in RUN[k_]['pts'] if not (p_['k'] == 'F' and p_['tri'] == t_)]
log(f'{NMOUTH} of {2 * len(M)} lane ends leave straight through their own stub; {NSPREAD} cell edge(s) with runs read '
    f'out of a bar, or on one place, put in order a pitch apart')
VIEW = tuple(float(v) for v in OPT['--view'].split(',')) if '--view' in OPT else None
_x0 = same_layer_crossings(layout())
log(f'the layout it starts from, read onto its points: {len(_x0)} same-layer crossing(s)')
for c_ in _x0[:10]:
    log(f'   crossing: {c_[0]} x {c_[1]} on {LNAME[RUN[c_[0]]["L"]]} at {c_[2]}')
if '--draw0' in OPT:
    draw(OPT['--draw0'], 'whole_embed: the layout it starts from, read onto its points', VIEW, [c[2] for c in _x0])

# ---------------------------------------------------------------- the LP, a round at a time
VXO = {n: (_pairs.dive_offset(ctx.cfg, PP / 2) if n in prs else 0.0) for n in M}    # a pair's barrels off its line
ROUNDS = int(OPT.get('--rounds', 40))         # the rounds at most (they end when nothing moves any more)
TRUST = float(OPT.get('--trust', 0.65 * P_MIN))   # a point's move in one round (a via's each of x and y): 0.65 of a pitch
W_BEND = float(OPT.get('--bend', 1.0))        # the price of a point a millimetre off the line through its neighbours
SLACK = float(OPT.get('--slack', 0.10))       # the SLACK asked beyond every bar: this share of it, a grid step at least
W_SLACK = float(OPT.get('--wslack', 30.0))    # ...priced this a millimetre short of it (taken wherever there is room)
TOL = float(OPT.get('--tol', 0.4 * GS))       # a point that moved less than this in a round has not moved
REACH = float(OPT.get('--reach', 0.0))        # a round's LP holds what stands this near where the round before moved
#                                               (0: the farthest any of its rows reaches -- set below)
SIDE = TRUST / 4                              # nothing passes THROUGH anything: two lanes of a layer, a lane and a via,
#                                               two vias stay this far apart at least along the line that parts them
W_ORDER = 100 * W_HARD                        # ...and two lanes on one cell edge keep their order
# a stretch's length: octilinear (what the snap will lay), the largest of these eight products -- flat to a zigzag
# inside one of its sectors, which the bend's price settles
DIRS = [(c_ * sx, s_ * sy) for (c_, s_) in ((1.0, math.sqrt(2) - 1), (math.sqrt(2) - 1, 1.0)) for sx in (1, -1) for sy in (1, -1)]
ORD = 1e-3                                    # two lanes on one cell edge: this far apart along it at least (their order)
slk = lambda need: need + max(need * SLACK, ctx.cfg.grid_step)       # a bar with its slack
if REACH <= 0:
    REACH = max(P_COMF + 2 * max(hw.values()) + TRUST, slk(VIA_R + PP + max(VXO.values())) + 3 * TRUST)


class LP:
    """rows `sum coef * x <= b` over variables with costs and bounds; a row may be ELASTIC (a slack variable it pays
    its price a millimetre for -- its own, or one it shares with other rows; capped where the rule may give only so
    far), and what each kind paid is read back"""

    def __init__(self):
        self.c, self.lo, self.hi = [], [], []
        self.I, self.Jc, self.A, self.b = [], [], [], []
        self.slack = []                      # (slack variable, kind, what)

    def var(self, cost=0.0, lo=None, hi=None):
        self.c.append(cost); self.lo.append(lo); self.hi.append(hi)
        return len(self.c) - 1

    def paying(self, kind, what, price, cap=None):
        sl = self.var(price, 0.0, None if cap is None else max(cap, 0.0))
        self.slack.append((sl, kind, what))
        return sl

    def le(self, terms, rhs, sl=None):
        r = len(self.b)
        for v, a in terms:
            if a != 0.0:
                self.I.append(r); self.Jc.append(v); self.A.append(a)
        if sl is not None:
            for s_ in (sl if isinstance(sl, (list, tuple)) else (sl,)):
                self.I.append(r); self.Jc.append(s_); self.A.append(-1.0)
        self.b.append(rhs)
        return r

    def ge(self, terms, rhs, sl=None):
        return self.le([(v, -a) for v, a in terms], -rhs, sl)

    def solve(self):
        A = coo_matrix((self.A, (self.I, self.Jc)), shape=(len(self.b), len(self.c))).tocsr() if self.b else None
        res = None
        for meth in ('highs-ipm', 'highs-ds'):   # (interior point: a third of the dual simplex's time on the largest)
            res = linprog(self.c, A_ub=A, b_ub=self.b if self.b else None, bounds=list(zip(self.lo, self.hi)), method=meth)
            if res.status == 0:
                break
        if res.status != 0:
            raise SystemExit(f'whole_embed: the LP has no answer ({res.message})')
        self.x = res.x
        self.marg = res.ineqlin.marginals if self.b else np.zeros(0)
        self.lom, self.him = res.lower.marginals, res.upper.marginals
        return res


# a point as an affine form of the LP's variables: (x0, y0, [(variable, dx, dy)])
def P_dot(nv, p):
    return nv[0] * p[0] + nv[1] * p[1], [(v, nv[0] * ax + nv[1] * ay) for v, ax, ay in p[2]]


def sep(lp, nv, pa, pb, need, sl):
    """the row nv . (pb - pa) >= need (less the slack sl)"""
    cb, tb = P_dot(nv, pb)
    ca, ta = P_dot(nv, pa)
    lp.ge(tb + [(v, -a) for v, a in ta], need - (cb - ca), sl)


def lane_line(n, per_run):
    """lane n's whole line through its vias: the runs' lists joined (a via counted once)"""
    out = []
    for k in range(len(RUNL[n])):
        out += per_run[(n, k)] if not out else per_run[(n, k)][1:]
    return out


def shortfalls(PL):
    """what stands short in the layout PL, measured on its lines: {'pitch': [(mm short, (run, run))], 'via': [(mm,
    (via, run))], 'viavia': [(mm, (via, via))], and 'slack-lane' / 'slack-via': the same against the bars with their
    slack}"""
    sg = []
    for key in KEYS:
        pts = PL[key]
        for i in range(len(pts) - 1):
            if pts[i] != pts[i + 1]:
                sg.append((key, LineString([pts[i], pts[i + 1]])))
    out = collections.defaultdict(dict)
    if not sg:
        return out
    G_ = np.array([s_ for _, s_ in sg], dtype=object)
    tr = shapely.STRtree(G_)
    hwm = 2 * max(hw.values())
    ia, ib = tr.query(G_, predicate='dwithin', distance=slk(P_MIN + hwm))
    ok = ia < ib
    ia, ib = ia[ok], ib[ok]
    dd = shapely.distance(G_[ia], G_[ib])
    for a_, b_, d_ in zip(ia, ib, dd):
        ka, kb = sg[a_][0], sg[b_][0]
        if ka[0] == kb[0] or RUN[ka]['L'] != RUN[kb]['L']:
            continue
        gap = P_MIN + hw[ka[0]] + hw[kb[0]]
        w = (ka, kb) if ka < kb else (kb, ka)
        if d_ < gap - 2e-4:
            out['pitch'][w] = max(out['pitch'].get(w, 0.0), gap - float(d_))
        elif d_ < slk(gap) - 2e-4:
            out['slack-lane'][w] = max(out['slack-lane'].get(w, 0.0), slk(gap) - float(d_))
    for vk in VKEYS:
        pv = Point(VIA[vk])
        for b_ in tr.query(pv, predicate='dwithin', distance=slk(VIA_R + PP + max(VXO.values()))):
            kb = sg[b_][0]
            if kb[0] == vk[0]:
                continue
            room = VS[kb[0]] + VXO[vk[0]]
            d_ = pv.distance(sg[b_][1])
            if d_ < room - 2e-4:
                out['via'][(vk, kb)] = max(out['via'].get((vk, kb), 0.0), room - d_)
            elif d_ < slk(room) - 2e-4:
                out['slack-via'][(vk, kb)] = max(out['slack-via'].get((vk, kb), 0.0), slk(room) - d_)
    for i, v1 in enumerate(VKEYS):
        for v2 in VKEYS[i + 1:]:
            need = VIA_VV + VXO[v1[0]] + VXO[v2[0]]
            d_ = math.hypot(VIA[v2][0] - VIA[v1][0], VIA[v2][1] - VIA[v1][1])
            if d_ < need - 2e-4:
                out['viavia'][(v1, v2)] = need - d_
            elif d_ < slk(need) - 2e-4:
                out['slack-via'][(v1, v2)] = slk(need) - d_
    return {k_: sorted(((round(v_, 4), w_) for w_, v_ in d_.items()), key=lambda q_: -q_[0]) for k_, d_ in out.items()}


def _cell_after(r, i):
    """the cell run r is in just after its point i (-1: its start)"""
    if i < 0:
        return r['first']
    p = r['pts'][i]
    return p['to'] if p['k'] == 'P' else p['tri']


def _cell_before(r, i):
    """the cell run r is in just before its point i (len: its end)"""
    if i >= len(r['pts']):
        return r['last']
    p = r['pts'][i]
    return p['from'] if p['k'] == 'P' else p['tri']


def chord_items(m, a, ta, b, tb):
    """the cell edges the straight line from a (in cell ta) to b (in cell tb) crosses, as a run's points; None where
    the line leaves the free space or its cells do not join ta to tb"""
    ln = math.hypot(b[0] - a[0], b[1] - a[1])
    nn = max(2, int(math.ceil(ln / (0.16 * GS))))
    sam = np.array([(a[0] + (b[0] - a[0]) * i / nn, a[1] + (b[1] - a[1]) * i / nn) for i in range(1, nn)])
    tri = m.locate(sam)
    if (tri < 0).any():
        return None
    seq = [int(ta)]
    hit = [None]
    for t, q in zip(tri, sam):
        if int(t) != seq[-1]:
            seq.append(int(t)); hit.append(q)
    if seq[-1] != int(tb):
        seq.append(int(tb)); hit.append(np.array(b))
    out = []
    for c in range(1, len(seq)):
        t0, t1 = seq[c - 1], seq[c]
        if t1 in m.N[t0]:
            chain = [t0, t1]
        else:
            chain = m.path(t0, t1)
            if chain is None or len(chain) > 6:
                return None
        for u0, u1 in zip(chain, chain[1:]):
            out.append(portal(m, u0, u1, hit[c]))
    i = 0
    while i + 1 < len(out):
        if out[i]['from'] == out[i + 1]['to'] and out[i]['to'] == out[i + 1]['from']:
            del out[i:i + 2]
            i = max(i - 1, 0)
        else:
            i += 1
    return out


PULL_MIN = P_MIN / 2   # a stretch of a run is pulled straight where that saves half a pitch, and half its chord
SPUR_MIN = TW / 4      # ...a spur where it saves a quarter of a track's width


def pull_runs():
    """each run PULLED TAUT where it doubles back on itself: a stretch of it whose way is half as long again as the
    straight line between its two ends, with nothing in between -- no fixed copper, no other run of its layer, no via
    -- is that straight line. (A via that has moved drags its run behind it along its own way; a lane that bulged
    into a cell and came back keeps the bulge: both are loops with nothing in them.) -> the places pulled"""
    PL = layout()
    segs = [[] for _ in range(NL)]
    for key in KEYS:
        pts = PL[key]
        for i in range(len(pts) - 1):
            if pts[i] != pts[i + 1]:
                segs[RUN[key]['L']].append((key, LineString([pts[i], pts[i + 1]])))
    trees = [shapely.STRtree([g_ for _, g_ in sg]) if sg else None for sg in segs]
    vpts = shapely.points(np.array([VIA[vk] for vk in VKEYS])) if VKEYS else None
    vtree = shapely.STRtree(vpts) if VKEYS else None
    places = []
    for key in KEYS:
        r = RUN[key]
        L_ = r['L']
        m = MESH[L_]
        i = 0
        guard = 0
        while guard < 6:
            guard += 1
            xy = run_xy(key)
            n = len(xy)
            if n < 4:
                break
            cum = [0.0]
            for a, b in zip(xy, xy[1:]):
                cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
            found = None
            for i in range(0, n - 2):
                for j in range(n - 1, i + 1, -1):
                    chord = math.hypot(xy[j][0] - xy[i][0], xy[j][1] - xy[i][1])
                    way = cum[j] - cum[i]
                    # (...or a spur: out and straight back, three times its chord, however short)
                    if not ((way - chord >= PULL_MIN and way >= 1.5 * chord) or
                            (way - chord >= SPUR_MIN and way >= 3 * chord)):
                        continue
                    if any(p['k'] == 'X' for p in r['pts'][i:j - 1]):
                        continue
                    hull = shapely.MultiPoint(xy[i:j + 1]).convex_hull
                    if hull.geom_type != 'Polygon':
                        hull = hull.buffer(1e-4)
                    if not m.free.covers(hull):
                        continue
                    if any(segs[L_][q_][0] != key for q_ in trees[L_].query(hull, predicate='intersects')):
                        continue
                    if vtree is not None and len(vtree.query(hull, predicate='contains_properly')):
                        continue
                    # ...and the straight line no nearer anything than its bar, or than the stretch it replaces was
                    ch, old = LineString([xy[i], xy[j]]), LineString(xy[i:j + 1])
                    bad = False
                    for q_ in trees[L_].query(ch, predicate='dwithin', distance=P_MIN + 2 * max(hw.values())):
                        k2, g2 = segs[L_][q_]
                        if k2 != key:
                            d1 = ch.distance(g2)
                            if d1 < P_MIN + hw[key[0]] + hw[k2[0]] and d1 < old.distance(g2) - 1e-9:
                                bad = True
                                break
                    if not bad and vtree is not None:
                        for q_ in vtree.query(ch, predicate='dwithin', distance=VIA_R + PP + max(VXO.values())):
                            vk = VKEYS[q_]
                            if vk[0] == key[0]:
                                continue
                            d1 = ch.distance(vpts[q_])
                            if d1 < VS[key[0]] + VXO[vk[0]] and d1 < old.distance(vpts[q_]) - 1e-9:
                                bad = True
                                break
                    if bad:
                        continue
                    items = chord_items(m, xy[i], _cell_after(r, i - 1), xy[j], _cell_before(r, j - 1))
                    if items is None:
                        continue
                    found = (i, j, items)
                    break
                if found:
                    break
            if not found:
                break
            i, j, items = found
            if WATCH and key in WATCH:
                hull = shapely.MultiPoint(xy[i:j + 1]).convex_hull
                log(f'         pull {key[0]}.{key[1]} [{i}..{j}] ({xy[i][0]:.2f},{xy[i][1]:.2f}) -> ({xy[j][0]:.2f},{xy[j][1]:.2f}), '
                    f'{len(items)} edge(s); vias in its hull: {[VKEYS[q_] for q_ in vtree.query(hull, predicate="intersects")]}; '
                    f'runs: {sorted({segs[L_][q_][0] for q_ in trees[L_].query(hull, predicate="intersects")})}')
            r['pts'][i:j - 1] = items
            places += [xy[i], xy[j]]
    return places


HOT = None           # where the layout moved in the round before (None: everywhere -- the first round)
WATCH = [(x.rsplit('.', 1)[0], int(x.rsplit('.', 1)[1])) for x in OPT['--watch'].split(',')] if '--watch' in OPT else None
VTR, VWAS = {}, {}   # each via's own trust (x, y) and its move in the round before
VHOPS = {}           # ...and the cell changes it has made
VHIST = {}           # ...and its last places
TR_MIN = 0.16 * GS
PRESS = 1e-4         # a point or via is PRESSED on an edge of its cell where the LP's price of that bound is this
FLIPS = 3            # the cell changes one free point makes at most (a via: four times that)


def _trust(tr, was, now):
    """a point's own trust for the next round: HALVED where its move turned back on the round before's (the LP's
    straight-line picture overshot there: the point would swing for ever), grown again while it keeps its way"""
    if abs(now) < 1e-6:
        return tr
    if was * now < 0:
        return max(tr / 2, TR_MIN)
    return min(TRUST, tr * 1.5)


def round_of(it):
    """one LP from the layout as it stands, and the layout moved to its answer. The LP holds the points and vias
    within REACH of where the round before moved (the rest stand where they are: fixed points to it); what moved in
    this one is left in HOT. -> (what its rows paid by kind, the largest move, the cell changes)"""
    global HOT
    lp = LP()
    cur = layout()
    tS = time.time()
    gs = ctx.cfg.grid_step
    if HOT is None:
        live = {key: np.ones(len(cur[key]), bool) for key in KEYS}
        vlive = {vk: True for vk in VKEYS}
    else:
        kd = cKDTree(np.array(HOT))
        live = {}
        for key in KEYS:
            mk = kd.query(np.array(cur[key]), distance_upper_bound=REACH)[0] < math.inf
            m2 = mk.copy()
            m2[1:] |= mk[:-1]; m2[:-1] |= mk[1:]
            live[key] = m2
        vlive = {vk: bool(kd.query(VIA[vk], distance_upper_bound=REACH)[0] < math.inf) or
                 bool(live[vk][-1]) or bool(live[(vk[0], vk[1] + 1)][0]) for vk in VKEYS}
    # ---- variables: a via's xy; a run's place along each cell edge it crosses; each of its free points' place ACROSS
    # the lane (along the normal of the line through its neighbours: a point slid along its lane changes nothing)
    vaff, vrows, vvar = {}, {}, {}
    for vk in VKEYS:
        x, y = VIA[vk]
        if not vlive[vk]:
            vaff[vk] = (x, y, [])
            continue
        tr_ = VTR.get(vk, (TRUST, TRUST))
        vx, vy = lp.var(0.0, x - tr_[0], x + tr_[0]), lp.var(0.0, y - tr_[1], y + tr_[1])
        vvar[vk] = (vx, vy)
        vaff[vk] = (0.0, 0.0, [(vx, 1.0, 0.0), (vy, 0.0, 1.0)])
        n, kv = vk
        cells = [('v', VMESH, TV[vk]), ('a', MESH[RUN[(n, kv)]['L']], RUN[(n, kv)]['last']),
                 ('b', MESH[RUN[(n, kv + 1)]['L']], RUN[(n, kv + 1)]['first'])]
        vrows[vk] = {}
        for tag, m, t in cells:
            # (hard: a via stands inside its cells, or no further out of one than it stands now -- every other rule
            # gives before a via goes into fixed copper)
            rws = m.rows_inside(t)
            out_ = max([0.0] + [c - 1e-7 - (nx * x + ny * y) for (nx, ny, c) in rws])
            sl = lp.paying('vcell', vk, W_HARD, out_ + 1e-9)
            vrows[vk][tag] = [lp.ge([(vx, nx), (vy, ny)], c - 1e-7, sl) for (nx, ny, c) in rws]
    aff, nlive = {}, 0
    for key in KEYS:
        n, k = key
        r = RUN[key]
        m = MESH[r['L']]
        cp, lm = cur[key], live[key]
        pts = [vaff[(n, k - 1)] if k else (cp[0][0], cp[0][1], [])]
        for j, p in enumerate(r['pts'], start=1):
            x, y = cp[j]
            p['v'] = None
            if p['k'] == 'X' or not lm[j]:                    # a breach's point, or one out of the round's reach: fixed
                pts.append((x, y, []))
                continue
            nlive += 1
            tr_ = p.get('tr', TRUST)
            if p['k'] == 'P':
                A, B = m.V[p['a']], m.V[p['b']]
                p['v'] = lp.var(0.0, max(0.0, p['t'] - tr_), min(p['len'], p['t'] + tr_))
                pts.append((float(A[0]), float(A[1]), [(p['v'], float((B[0] - A[0]) / p['len']), float((B[1] - A[1]) / p['len']))]))
                lo, hi = hw[n], p['len'] - hw[n]
                if hi < lo:
                    lo = hi = p['len'] / 2
                if hw[n] > 0:                                 # a pair's half width off the edge's two ends
                    sl = lp.paying('edge', key, W_HARD)
                    lp.ge([(p['v'], 1.0)], lo, sl)
                    lp.le([(p['v'], 1.0)], hi, sl)
                for (sgn, ref_) in ((1.0, lo), (-1.0, -hi)):  # its slack off them (fixed copper's corners)
                    if sgn * p['t'] - tr_ < ref_ + gs:
                        lp.ge([(p['v'], sgn)], ref_ + gs, lp.paying('slack-copper', key, W_SLACK))
                continue
            tx, ty = cp[j + 1][0] - cp[j - 1][0], cp[j + 1][1] - cp[j - 1][1]
            tl = math.hypot(tx, ty)
            if tl < 1e-9:
                tx, ty = cp[j + 1][0] - x, cp[j + 1][1] - y
                tl = math.hypot(tx, ty)
            if tl < 1e-9:
                tx, ty, tl = 1.0, 0.0, 1.0
            nx_, ny_ = -ty / tl, tx / tl
            # inside its cell: the stretch of its normal the cell's three edges leave it (elo / ehi: the edge at each end)
            tlo, thi, elo, ehi, walls = -math.inf, math.inf, None, None, []
            for i, (ax, ay, c) in enumerate(m.rows_inside(p['tri'])):
                g, h = ax * nx_ + ay * ny_, c - (ax * x + ay * y)
                if g > 1e-9:
                    if h / g > tlo:
                        tlo, elo = h / g, i
                elif g < -1e-9:
                    if h / g < thi:
                        thi, ehi = h / g, i
                if m.N[p['tri'], i] < 0 and abs(g) > 1e-9:
                    walls.append((g, h))
            if tlo > thi:
                lo = hi = 0.0
                elo = ehi = None
            else:
                lo, hi = min(max(tlo, -tr_), tr_), min(max(thi, -tr_), tr_)
                elo, ehi = (elo if tlo > -tr_ else None), (ehi if thi < tr_ else None)
            p['v'] = lp.var(0.0, lo, hi)
            p['n'], p['lim'] = (nx_, ny_), (elo, ehi)
            pts.append((x, y, [(p['v'], nx_, ny_)]))
            sl = None
            for (g, h) in walls:                              # its slack off the fixed copper's bar (a wall of the cell)
                b_ = (h + gs) / g
                if (g > 0 and b_ > lo) or (g < 0 and b_ < hi):
                    if sl is None:
                        sl = lp.paying('slack-copper', key, W_SLACK)
                    lp.ge([(p['v'], g)], h + gs, sl)
        pts.append(vaff[(n, k)] if (n, k) in VIA else (cp[-1][0], cp[-1][1], []))
        aff[key] = pts
        for a_, b_, pa, pb in zip(pts, pts[1:], cp, cp[1:]):  # its length (the directions a round's move can reach)
            if not a_[2] and not b_[2]:
                continue
            lv = lp.var(W_LEN, 0.0, None)
            vals = [o[0] * (pb[0] - pa[0]) + o[1] * (pb[1] - pa[1]) for o in DIRS]
            mx = max(vals)
            for o, v_ in zip(DIRS, vals):
                if v_ < mx - 6.2 * TRUST:
                    continue
                cq, tq = P_dot(o, b_)
                c0_, tp = P_dot(o, a_)
                lp.le(tq + [(v, -a) for v, a in tp] + [(lv, -1.0)], -(cq - c0_))
    # ---- the price of a bend: each point of a lane off the straight line through the points before and after it
    # (across the lane; a via both ways)
    if W_BEND > 0:
        for n in M:
            la, lc = lane_line(n, aff), lane_line(n, cur)
            for j in range(1, len(la) - 1):
                if not (la[j][2] or la[j - 1][2] or la[j + 1][2]):
                    continue
                l1 = math.hypot(lc[j][0] - lc[j - 1][0], lc[j][1] - lc[j - 1][1])
                l2 = math.hypot(lc[j + 1][0] - lc[j][0], lc[j + 1][1] - lc[j][1])
                if l1 + l2 < 1e-6:
                    continue
                w0, w2 = l2 / (l1 + l2), l1 / (l1 + l2)
                cx, cy = lc[j + 1][0] - lc[j - 1][0], lc[j + 1][1] - lc[j - 1][1]
                cl = math.hypot(cx, cy)
                axes = ((1.0, 0.0), (0.0, 1.0)) if (cl < 1e-9 or len(la[j][2]) == 2) else ((-cy / cl, cx / cl),)
                for ax in axes:
                    c1, t1_ = P_dot(ax, la[j]); c0, t0_ = P_dot(ax, la[j - 1]); c2, t2_ = P_dot(ax, la[j + 1])
                    terms = t1_ + [(v, -a * w0) for v, a in t0_] + [(v, -a * w2) for v, a in t2_]
                    cst = c1 - w0 * c0 - w2 * c2
                    bv = lp.var(W_BEND, 0.0, None)
                    lp.le(terms + [(bv, -1.0)], -cst)
                    lp.le([(v, -a) for v, a in terms] + [(bv, -1.0)], cst)
    # ---- rows: on a cell edge the lanes of a layer keep the order they stand in
    for (L_, a, b), lst in edge_lists().items():
        if len(lst) < 2:
            continue
        ln = lst[0][1]['len']
        can = lambda e_: e_[1]['t'] if e_[2] else ln - e_[1]['t']
        lst.sort(key=can)
        for x, y in zip(lst, lst[1:]):
            if x[0][0] == y[0][0] or (x[1]['v'] is None and y[1]['v'] is None):
                continue
            terms, cst = [], 0.0                              # can(y) - can(x), as the LP's form
            for e_, sg_ in ((y, 1.0), (x, -1.0)):
                if e_[1]['v'] is None:
                    cst += sg_ * can(e_)
                else:
                    terms.append((e_[1]['v'], sg_ * (1.0 if e_[2] else -1.0)))
                    cst += sg_ * (0.0 if e_[2] else ln)
            lp.ge(terms, ORD - cst, lp.paying('order', (x[0], y[0]), W_ORDER, ORD))
    # ---- rows: every lane apart from its NEIGHBOURS on its layer. Each point of a lane is held off the nearest
    # stretch of each lane beside it by ONE row: along the line from the point to its foot on that stretch as they
    # stand, the foot's place along the stretch kept. (Two lines' nearest approach is at a point of one of them, and
    # every lane's points are held, so both ways are.) A lane beyond another is no neighbour: the shortest way to it
    # crosses the lane between. The shortfall is priced in steps: free up to the comfortable pitch's, then the
    # slack's, then the pitch itself (W_HARD) -- and never to nothing: the two stay SIDE apart along that line, on
    # the sides they stand, so no lane passes through another
    tA = time.time()
    segs = [[] for _ in range(NL)]
    allseg = []
    for key in KEYS:
        cp = cur[key]
        for j in range(len(cp) - 1):
            if math.hypot(cp[j + 1][0] - cp[j][0], cp[j + 1][1] - cp[j][1]) > 1e-9:
                sg = (key, j, LineString([cp[j], cp[j + 1]]))
                segs[RUN[key]['L']].append(sg)
                allseg.append(sg)
    npair = ncand = 0
    hwm = 2 * max(hw.values())
    for L_ in range(NL):
        sl_ = segs[L_]
        if not sl_:
            continue
        G_ = np.array([s_[2] for s_ in sl_], dtype=object)
        lane_ = np.array([li[s_[0][0]] for s_ in sl_])
        tree = shapely.STRtree(G_)
        pk, pj, pxy = [], [], []                        # each run's own points on this layer (not its ends)
        for key in KEYS:
            if RUN[key]['L'] == L_:
                # (and its end where that is the lane's own fixed end -- a stub's mouth: a lane passing another's
                # mouth is held off it as off any point of it. A via end is held by the via's own rows)
                j0 = 0 if key[1] == 0 else 1
                j1 = len(cur[key]) if key[1] == len(RUNL[key[0]]) - 1 else len(cur[key]) - 1
                for j in range(j0, j1):
                    pk.append(key); pj.append(j); pxy.append(cur[key][j])
        if not pk:
            continue
        PT = shapely.points(np.array(pxy))
        plane = np.array([li[k_[0]] for k_ in pk])
        qp, qs = tree.query(PT, predicate='dwithin', distance=P_COMF + hwm + TRUST)
        ok = plane[qp] != lane_[qs]
        qp, qs = qp[ok], qs[ok]
        ncand += len(qp)
        if not len(qp):
            continue
        con = shapely.shortest_line(PT[qp], G_[qs])
        dd_ = shapely.length(con)
        # the nearest stretch of each lane beside the point
        o_ = np.lexsort((dd_, lane_[qs], qp))
        qp, qs, con, dd_ = qp[o_], qs[o_], con[o_], dd_[o_]
        first = np.ones(len(qp), bool)
        first[1:] = (qp[1:] != qp[:-1]) | (lane_[qs[1:]] != lane_[qs[:-1]])
        qp, qs, con, dd_ = qp[first], qs[first], con[first], dd_[first]
        ci, cj = tree.query(con, predicate='intersects')
        third = (lane_[cj] != plane[qp[ci]]) & (lane_[cj] != lane_[qs[ci]])
        blocked = np.zeros(len(qp), bool)
        blocked[ci[third]] = True
        cc = shapely.get_coordinates(con).reshape(-1, 2, 2)
        if WATCH:                                   # (the watched two runs: their candidate rows, and what dropped them)
            for q in range(len(qp)):
                if {pk[qp[q]], sl_[qs[q]][0]} == set(WATCH):
                    ka, j = pk[qp[q]], pj[qp[q]]
                    kb, i, _g = sl_[qs[q]]
                    why = 'BLOCKED by ' + ','.join(sorted({sl_[c_][0][0] + '.' + str(sl_[c_][0][1]) for c_ in cj[(ci == q) & third]})) \
                        if blocked[q] else ('fixed' if not (aff[ka][j][2] or aff[kb][i][2] or aff[kb][i + 1][2]) else 'row')
                    if float(dd_[q]) < 0.3:
                        log(f'         cand {ka[0]}.{ka[1]}[{j}] ({cur[ka][j][0]:.2f},{cur[ka][j][1]:.2f}) -> {kb[0]}.{kb[1]}[{i}] d {float(dd_[q]):.3f}: {why}')
        for q in np.nonzero(~blocked)[0]:
            ka, j = pk[qp[q]], pj[qp[q]]
            kb, i, _g = sl_[qs[q]]
            if not (aff[ka][j][2] or aff[kb][i][2] or aff[kb][i + 1][2]):
                continue
            gap = P_MIN + hw[ka[0]] + hw[kb[0]]
            comf = max(P_COMF + hw[ka[0]] + hw[kb[0]], slk(gap))
            dd = float(dd_[q])
            B0, B1 = cur[kb][i], cur[kb][i + 1]
            bl = math.hypot(B1[0] - B0[0], B1[1] - B0[1])
            lam = min(max(((cc[q, 1, 0] - B0[0]) * (B1[0] - B0[0]) + (cc[q, 1, 1] - B0[1]) * (B1[1] - B0[1])) / (bl * bl), 0.0), 1.0)
            if dd > 1e-6:
                nv = ((cc[q, 1, 0] - cc[q, 0, 0]) / dd, (cc[q, 1, 1] - cc[q, 0, 1]) / dd)
            else:                          # touching: apart as the braid orders them, across the other lane's line
                bit = order_bit(kb[0], kb[1], ka[0], ka[1]) or 1
                nv = ((B1[1] - B0[1]) / bl * bit, -(B1[0] - B0[0]) / bl * bit)
            c0, t0_ = P_dot(nv, aff[kb][i]); c1, t1_ = P_dot(nv, aff[kb][i + 1]); cp_, tp_ = P_dot(nv, aff[ka][j])
            terms = [(v, a_ * (1 - lam)) for v, a_ in t0_] + [(v, a_ * lam) for v, a_ in t1_] + [(v, -a_) for v, a_ in tp_]
            sls = [lp.paying('comfort', None, W_COMF, comf - slk(gap)),
                   lp.paying('slack-lane', (ka, kb), W_SLACK, slk(gap) - gap),
                   lp.paying('pitch', (ka, kb), W_HARD, gap - min(SIDE, dd))]
            lp.ge(terms + [(s_, 1.0) for s_ in sls], comf - ((1 - lam) * c0 + lam * c1 - cp_))
            npair += 1
    tB = time.time()
    # ---- rows: every lane a via's room off every other lane's via, on the braid's side of it (and never through it)
    wrong, said = 0, 0
    if allseg:
        tree = shapely.STRtree([s_[2] for s_ in allseg])
    for vk in (VKEYS if allseg else ()):
        V = VIA[vk]
        a_ = vk[0]
        for ib in tree.query(Point(V), predicate='dwithin', distance=slk(VIA_R + PP + VXO[a_]) + 3 * TRUST):
            key, j, sg = allseg[ib]
            if key[0] == a_ or not (vaff[vk][2] or aff[key][j][2] or aff[key][j + 1][2]):
                continue
            room = VS[key[0]] + VXO[a_]
            P_, Q_ = cur[key][j], cur[key][j + 1]
            hx, hy = Q_[0] - P_[0], Q_[1] - P_[1]
            hl = math.hypot(hx, hy)
            nrm = (-hy / hl, hx / hl)
            w = ((V[0] - P_[0]) * hx + (V[1] - P_[1]) * hy) / hl
            C_ = P_ if w <= 0.0 else Q_ if w >= hl else (P_[0] + hx * w / hl, P_[1] + hy * w / hl)
            dl = math.hypot(C_[0] - V[0], C_[1] - V[1])
            nv = ((C_[0] - V[0]) / dl, (C_[1] - V[1]) / dl) if dl > 1e-6 else None
            if 0.0 < w < hl and hl >= 2 * GS:      # beside it: the braid says which side the lane passes
                side = via_bit(a_, vk[1], key[0], key[1])
                if side:
                    said += 1
                    geo = 1 if (nrm[0] * (P_[0] - V[0]) + nrm[1] * (P_[1] - V[1])) > 0 else -1
                    if side != geo or nv is None:
                        wrong += 1
                        nv = (nrm[0] * side, nrm[1] * side)
            if nv is None:
                nv = nrm
            now = min(nv[0] * (q[0] - V[0]) + nv[1] * (q[1] - V[1]) for q in (P_, Q_))
            sls = [lp.paying('slack-via', (vk, key), W_SLACK, slk(room) - room),
                   lp.paying('via', (vk, key), W_HARD, room - min(SIDE, now))]
            for q in (aff[key][j], aff[key][j + 1]):
                sep(lp, nv, vaff[vk], q, slk(room), sls)
    # ---- rows: two vias their own distance apart, along the line between them as they stand
    for i, v1 in enumerate(VKEYS):
        for v2 in VKEYS[i + 1:]:
            if not (vlive[v1] or vlive[v2]):
                continue
            dx, dy = VIA[v2][0] - VIA[v1][0], VIA[v2][1] - VIA[v1][1]
            dl = math.hypot(dx, dy)
            need = VIA_VV + VXO[v1[0]] + VXO[v2[0]]
            if dl > slk(need) + 4 * TRUST:
                continue
            nv = (dx / dl, dy / dl) if dl > 1e-9 else (1.0, 0.0)
            sep(lp, nv, vaff[v1], vaff[v2], slk(need), [lp.paying('slack-via', (v1, v2), W_SLACK, slk(need) - need),
                                                       lp.paying('viavia', (v1, v2), W_HARD, need - min(SIDE, dl))])
    # ---- a via's slack off the walls of the vias' space
    for vk, (vx, vy) in vvar.items():
        sl = None
        for i, (nx, ny, c) in enumerate(VMESH.rows_inside(TV[vk])):
            if VMESH.N[TV[vk], i] < 0:
                sl = sl if sl is not None else lp.paying('slack-copper', vk, W_SLACK)
                lp.ge([(vx, nx), (vy, ny)], c + gs, sl)
    t1 = time.time()
    lp.solve()
    t2_ = time.time()
    sol = lp.x
    paid = collections.defaultdict(list)
    for (sl, kind, what) in lp.slack:
        if kind != 'comfort' and sol[sl] > 1e-6:
            paid[kind].append((round(float(sol[sl]), 4), what))
    if '--show-paid' in OPT:
        for k_ in sorted(paid):
            for v_, what in sorted(paid[k_], key=lambda q_: -q_[0])[:6]:
                log(f'      paid {k_} {v_:.3f}: {what}')
    # ---- the layout moved to the answer
    move, hot, nexc = 0.0, [], 0
    for key in KEYS:
        r = RUN[key]
        m = MESH[r['L']]
        cp = cur[key]
        over = []                                   # its free points pressed on an edge of their cell with free space beyond
        for idx, p in enumerate(r['pts']):
            if p['v'] is None:
                continue
            if p['k'] == 'P':
                d = float(sol[p['v']]) - p['t']
                p['t'] = float(sol[p['v']])
                p['tr'], p['was'] = _trust(p.get('tr', TRUST), p.get('was', 0.0), d), d
                # (its move ACROSS the lane: along its edge, less what of that runs along the lane)
                A, B = m.V[p['a']], m.V[p['b']]
                tx, ty = cp[idx + 2][0] - cp[idx][0], cp[idx + 2][1] - cp[idx][1]
                tl = math.hypot(tx, ty)
                mv = abs(d) * (abs((B[0] - A[0]) * ty - (B[1] - A[1]) * tx) / (p['len'] * tl) if tl > 1e-9 else 1.0)
                q = cp[idx + 1]
            else:
                s = float(sol[p['v']])
                q = (p['xy'][0] + s * p['n'][0], p['xy'][1] + s * p['n'][1])
                # (its way last round, turned to this round's normal: the lane's line there may have turned)
                was = p.get('was', 0.0) * (1.0 if p.get('wasn', p['n'])[0] * p['n'][0] + p.get('wasn', p['n'])[1] * p['n'][1] >= 0 else -1.0)
                p['tr'], p['was'], p['wasn'] = _trust(p.get('tr', TRUST), was, s), s, p['n']
                p['xy'] = q
                mv = abs(s)
                elo, ehi = p['lim']
                e_ = ehi if (ehi is not None and lp.him[p['v']] < -PRESS) else \
                    elo if (elo is not None and lp.lom[p['v']] > PRESS) else None
                # (a point that has gone to and fro between two cells stays: its place is ON their edge)
                if e_ is not None and m.N[p['tri'], e_] >= 0 and p.get('flips', 0) < FLIPS:
                    over.append((idx, int(m.N[p['tri'], e_])))
                    p['flips'] = p.get('flips', 0) + 1
            move = max(move, mv)
            if mv > TOL:
                hot.append(q)
        # such a point goes on into the cell beyond: the run's crossing of that edge moved past it where it has one
        # beside it, else the run crosses the edge there and back (a lane pushed aside bulges into the next cell)
        for idx, t2 in reversed(over):
            p = r['pts'][idx]
            t = p['tri']
            nxt = r['pts'][idx + 1] if idx + 1 < len(r['pts']) else None
            prv = r['pts'][idx - 1] if idx > 0 else None
            if nxt is not None and nxt['k'] == 'P' and nxt['from'] == t and nxt['to'] == t2:
                r['pts'][idx], r['pts'][idx + 1] = nxt, p
            elif prv is not None and prv['k'] == 'P' and prv['from'] == t2 and prv['to'] == t:
                r['pts'][idx - 1], r['pts'][idx] = p, prv
            else:
                r['pts'][idx:idx + 1] = [portal(m, t, t2, p['xy']), p, portal(m, t2, t, p['xy'])]
            c = m.cen[t2]
            p['tri'] = t2
            p['xy'] = (float(p['xy'][0] + (c[0] - p['xy'][0]) * 1e-4), float(p['xy'][1] + (c[1] - p['xy'][1]) * 1e-4))
            hot.append(p['xy'])
            nexc += 1
        i = 0
        while i + 1 < len(r['pts']):                # an edge crossed and crossed straight back, nothing between: neither
            a_, b_ = r['pts'][i], r['pts'][i + 1]
            if a_['k'] == 'P' and b_['k'] == 'P' and a_['from'] == b_['to'] and a_['to'] == b_['from']:
                del r['pts'][i:i + 2]
                i = max(i - 1, 0)
            else:
                i += 1
    hops = 0
    for vk, (vx, vy) in vvar.items():
        q = (float(sol[vx]), float(sol[vy]))
        mv = math.hypot(q[0] - VIA[vk][0], q[1] - VIA[vk][1])
        move = max(move, mv)
        d_ = (q[0] - VIA[vk][0], q[1] - VIA[vk][1])
        w_ = VWAS.get(vk, (0.0, 0.0))
        t_ = VTR.get(vk, (TRUST, TRUST))
        VTR[vk], VWAS[vk] = (_trust(t_[0], w_[0], d_[0]), _trust(t_[1], w_[1], d_[1])), d_
        # (...and one that WANDERS -- its last six moves end nearer where they began than half their way: halved too)
        h_ = VHIST.setdefault(vk, [VIA[vk]])
        h_.append(q)
        del h_[:-7]
        if len(h_) == 7:
            way_ = sum(math.hypot(b_[0] - a_[0], b_[1] - a_[1]) for a_, b_ in zip(h_, h_[1:]))
            if way_ > 6 * TOL and math.hypot(h_[-1][0] - h_[0][0], h_[-1][1] - h_[0][1]) < 0.5 * way_:
                VTR[vk] = (max(VTR[vk][0] / 2, TR_MIN), max(VTR[vk][1] / 2, TR_MIN))
                del h_[:-1]
        VIA[vk] = q
        n, kv = vk
        h0 = hops
        # a via pressed on an edge of one of its cells goes on to the cell beyond it
        def pressed(tag, m, t):
            best = None
            if VHOPS.get(vk, 0) >= 4 * FLIPS:
                return None
            for i, row in enumerate(vrows[vk][tag]):
                t2 = int(m.N[t, i])
                if lp.marg[row] < -PRESS and t2 >= 0 and (best is None or lp.marg[row] < best[0]):
                    best = (lp.marg[row], t2)
            if best is not None:
                VHOPS[vk] = VHOPS.get(vk, 0) + 1
            return None if best is None else best[1]
        t2 = pressed('v', VMESH, TV[vk])
        if t2 is not None:
            TV[vk] = t2
            hops += 1
        ra, rb = RUN[(n, kv)], RUN[(n, kv + 1)]
        ma, mb = MESH[ra['L']], MESH[rb['L']]
        t2 = pressed('a', ma, ra['last'])
        # (the run crosses that edge AT the via, one crossing more -- or, where its crossing of that very edge is
        # the point next to the via, that one less. Points of the run between the via and a crossing farther along
        # stay: the lane's line does not move, only the cell its end is in)
        if t2 is not None:                          # the run that ends at it
            e_ = ra['pts'][-1] if ra['pts'] else None
            if e_ is not None and e_['k'] == 'P' and e_['from'] == t2 and e_['to'] == ra['last']:
                del ra['pts'][-1]
            else:
                ra['pts'].append(portal(ma, ra['last'], t2, q))
            ra['last'] = t2
            hops += 1
        t2 = pressed('b', mb, rb['first'])
        if t2 is not None:                          # the run that starts at it
            e_ = rb['pts'][0] if rb['pts'] else None
            if e_ is not None and e_['k'] == 'P' and e_['from'] == rb['first'] and e_['to'] == t2:
                del rb['pts'][0]
            else:
                rb['pts'].insert(0, portal(mb, t2, rb['first'], q))
            rb['first'] = t2
            hops += 1
        if mv > TOL or hops > h0:
            hot.append(q)
    npull = pull_runs()
    hot += npull
    # ---- each lane's own points kept HPT apart: a long stretch gets a point, two points on top of each other lose
    # one (never the one point a run has in a cell)
    for key in KEYS:
        r = RUN[key]
        xy = run_xy(key)
        old = r['pts']
        new, cell, prev, jump = [], r['first'], xy[0], False
        for ip, (p, q) in enumerate(zip(old, xy[1:-1])):
            d = math.hypot(q[0] - prev[0], q[1] - prev[1])
            lone = (not new or new[-1]['k'] != 'F') and (ip + 1 >= len(old) or old[ip + 1]['k'] != 'F')
            if p['k'] == 'F' and d < 0.25 * HPT and not lone:
                continue
            if d > 1.6 * HPT and not jump:
                nn = int(math.ceil(d / HPT))
                new += [{'k': 'F', 'xy': (prev[0] + (q[0] - prev[0]) * i_ / nn, prev[1] + (q[1] - prev[1]) * i_ / nn),
                         'tri': cell} for i_ in range(1, nn)]
            new.append(p)
            prev = q
            jump = bool(p.get('jump'))
            if p['k'] == 'P':
                cell = p['to']
            elif p['k'] == 'X':
                cell = p['tri']
        d = math.hypot(xy[-1][0] - prev[0], xy[-1][1] - prev[1])
        if d > 1.6 * HPT:
            nn = int(math.ceil(d / HPT))
            new += [{'k': 'F', 'xy': (prev[0] + (xy[-1][0] - prev[0]) * i_ / nn, prev[1] + (xy[-1][1] - prev[1]) * i_ / nn),
                     'tri': cell} for i_ in range(1, nn)]
        r['pts'] = new
    HOT = hot
    # ---- what stands short after it
    PL = layout()
    sf = shortfalls(PL)
    xs_ = same_layer_crossings(PL)
    ln_ = sum(float(np.hypot(*(np.array(PL[k_][1:]) - np.array(PL[k_][:-1])).T).sum()) for k_ in KEYS)
    hard = {k_: v for k_, v in paid.items() if not k_.startswith('slack')}
    log(f'round {it}: {nlive} point(s) and {len(vvar)} via(s) in it, {len(lp.c)} variables, {len(lp.b)} rows, built '
        f'{t1 - tS:.1f} s, LP {t2_ - t1:.1f} s, after {time.time() - t2_:.1f} s; moved {move:.3f} ({len(hot)} place(s) over '
        f'{TOL}), {hops} via and {nexc} lane cell change(s), {len(npull) // 2} pulled; length {ln_:.1f}; rows paid '
        + (', '.join(f'{k_} {len(v)} (max {max(v)[0]:.3f})' for k_, v in sorted(hard.items())) or 'nothing')
        + f'; standing: {len(xs_)} same-layer crossing(s), '
        + ', '.join(f'{k_} {len(sf.get(k_, []))}' + (f' (worst {sf[k_][0][0]:.3f})' if sf.get(k_) else '')
                    for k_ in ('pitch', 'via', 'viavia'))
        + f'; slack given up at {len(sf.get("slack-lane", [])) + len(sf.get("slack-via", []))}; against the braid: '
        f'{wrong} of {said} via sides')
    for (ka_, kb_, xy_) in xs_[:6]:
        log(f'      crossing: {ka_[0]}.{ka_[1]} x {kb_[0]}.{kb_[1]} on {LNAME[RUN[ka_]["L"]]} at {xy_}')
    if '--watch' in OPT:                           # two runs' lines, and how near they stand, after the round
        watch(OPT['--watch'], PL)
    if '--show-standing' in OPT:
        standing(PL, sf)
    if '--show-hot' in OPT and hot:                # where the round's moves were, binned to a millimetre
        bins = collections.Counter((round(q_[0]), round(q_[1])) for q_ in hot)
        log('      moving: ' + ', '.join(f'({x_}, {y_}) x{c_}' for (x_, y_), c_ in bins.most_common(14)))
    return paid, move, hops + nexc


def standing(PL, sf, top=8):
    """the layout's largest shortfalls, each with its place"""
    for k_ in ('pitch', 'via', 'viavia'):
        for v_, (a_, b_) in sf.get(k_, [])[:top]:
            ga = Point(VIA[a_]) if k_ != 'pitch' else LineString(PL[a_])
            gb = Point(VIA[b_]) if k_ == 'viavia' else LineString(PL[b_])
            c_ = shapely.shortest_line(ga, gb).centroid
            nm = lambda q_: f'{q_[0]}.{q_[1]}' + ('' if k_ == 'pitch' or (k_ == 'via' and q_ is b_) else ' via')
            log(f'      standing {k_} {v_:.3f}: {nm(a_)} / {nm(b_)} at ({c_.x:.2f}, {c_.y:.2f})'
                + (f' on {LNAME[RUN[a_]["L"]]}' if k_ == 'pitch' else ''))


def watch(spec, PL):
    """--watch=LANE.k,LANE.k: the two runs' points as they stand, and the nearest approach of each stretch of the
    first to the second's line"""
    ka, kb = [(x.rsplit('.', 1)[0], int(x.rsplit('.', 1)[1])) for x in spec.split(',')]
    la, lb = LineString(PL[ka]), LineString(PL[kb])
    log(f'      watch {ka} ({len(PL[ka])} points, hw {hw[ka[0]]:.3f}) / {kb} ({len(PL[kb])} points, hw {hw[kb[0]]:.3f}); '
        f'pitch {P_MIN + hw[ka[0]] + hw[kb[0]]:.3f}; nearest {la.distance(lb):.3f}')
    for nm, key, other in ((ka, ka, lb), (kb, kb, la)):
        row = []
        for j, q in enumerate(PL[key]):
            d = Point(q).distance(other)
            if d < 1.0:
                kind = 'e' if j in (0, len(PL[key]) - 1) else RUN[key]['pts'][j - 1]['k']
                row.append(f'{kind}({q[0]:.2f},{q[1]:.2f}):{d:.2f}')
        log(f'         {nm[0]}.{nm[1]}: ' + ' '.join(row[:60]))


def kinks(tag):
    """where the layout's lines are KINKED. A lane that curves turns one way; a kink turns one way and back. Counted:
    a WIGGLE -- two points in turn, each 0.02 mm or more off the line through its neighbours (a fifth of a track's
    width: what shows), turning opposite ways -- and a FOLD, a turn past 90 degrees between two stretches of 0.05 mm
    or more"""
    PL = layout()
    nw, nf, worst, where = 0, 0, 0.0, []
    for n in M:
        pts = lane_line(n, PL)
        sg = [0.0] * len(pts)
        for j in range(1, len(pts) - 1):
            p0, p1, p2 = pts[j - 1], pts[j], pts[j + 1]
            cx, cy = p2[0] - p0[0], p2[1] - p0[1]
            cl = math.hypot(cx, cy)
            if cl > 1e-9:
                sg[j] = (cx * (p1[1] - p0[1]) - cy * (p1[0] - p0[0])) / cl
            a1, a2 = (p1[0] - p0[0], p1[1] - p0[1]), (p2[0] - p1[0], p2[1] - p1[1])
            if math.hypot(*a1) >= 0.05 and math.hypot(*a2) >= 0.05 and a1[0] * a2[0] + a1[1] * a2[1] < 0:
                nf += 1
        for j in range(1, len(pts) - 2):
            if abs(sg[j]) >= 0.02 and abs(sg[j + 1]) >= 0.02 and sg[j] * sg[j + 1] < 0:
                nw += 1
                worst = max(worst, abs(sg[j]), abs(sg[j + 1]))
                where.append((max(abs(sg[j]), abs(sg[j + 1])), n, pts[j]))
    log(f'kinks {tag}: {nw} wiggle(s) (worst {worst:.3f} mm off the line), {nf} fold(s)')
    if '--show-kinks' in OPT:
        vxy = np.array(list(VIA.values()))
        for sgm, n, p in sorted(where, reverse=True)[:14]:
            dv = float(np.min(np.hypot(vxy[:, 0] - p[0], vxy[:, 1] - p[1])))
            dc = min(Point(p).distance(_box(o[0], o[1], o[2], o[3])) for o in FIXED)
            log(f'      wiggle {sgm:.3f} on {n} at ({p[0]:.2f}, {p[1]:.2f}): nearest via {dv:.2f}, nearest fixed copper {dc:.2f}')
    return nw, nf


kinks('in the layout it starts from')
if '--watch' in OPT:
    watch(OPT['--watch'], layout())
for it in range(1, ROUNDS + 1):
    paid, move, hops = round_of(it)
    if not HOT:                                   # nothing moved, no cell changed: the layout has settled
        break
log(f'whole_embed: {it} round(s)' + ('' if not HOT else ' -- NOT settled') + f', {time.time() - T0:.1f} s')
kinks('after the rounds')
if '--draw' in OPT:
    draw(OPT['--draw'], f'whole_embed: after {it} round(s)', VIEW, [c[2] for c in same_layer_crossings(layout())])

# ---------------------------------------------------------------- written as whole_geo writes a geometry
def sampled(pts):
    """the line through pts with a point every G along each of its stretches (as the frames' columns give the
    polish a point every G)"""
    out = [tuple(pts[0])]
    for a, b in zip(pts, pts[1:]):
        ln = math.hypot(b[0] - a[0], b[1] - a[1])
        if ln < 1e-9:
            continue
        m = max(1, int(math.ceil(ln / G - 1e-9)))
        out += [(a[0] + (b[0] - a[0]) * i / m, a[1] + (b[1] - a[1]) * i / m) for i in range(1, m + 1)]
    return out


PL = layout()
res = {'lanes': {}, 'vias': [], 'cuts': [], 'lcuts': [], 'vcuts': []}
for n in M:
    xy, pieces = [], []
    for k in range(len(RUNL[n])):
        own = list(PL[(n, k)])
        if RUN[(n, k)].get('lead') and k == 0:                # the straight piece from its tooth to its stub's mouth
            own = [RUN[(n, k)]['lead']] + own
        if RUN[(n, k)].get('tail') and k == len(RUNL[n]) - 1:
            own = own + [RUN[(n, k)]['tail']]
        # (points within a hundredth of a millimetre of each other -- a run's places on the edges of sliver cells
        # at a copper corner -- one point: a jog that small is no shape of the lane's)
        own = [own[0]] + [q_ for i_, q_ in enumerate(own[1:-1], 1)
                          if math.hypot(q_[0] - own[i_ - 1][0], q_[1] - own[i_ - 1][1]) > 0.4 * GS] + [own[-1]]
        pts = sampled(own)
        Ln = LNAME[RUN[(n, k)]['L']]
        pieces += [(p[0], p[1], q[0], q[1], Ln) for p, q in zip(pts, pts[1:])]
        xy += pts if not xy else pts[1:]
        if k:
            res['vias'].append((n,) + tuple(PL[(n, k)][0]))
        # (the vias as the layout has them: a run's first point is its via only where the run is not the lane's first)
    res['lanes'][n] = {'xy': xy, 'pieces': pieces}
SF = shortfalls(PL)
res['paid'] = {k_: len(v_) for k_, v_ in SF.items() if not k_.startswith('slack')}
# a via that stands short of its room: the change it is, for the solve to move (whole_geo's via cut)
for v_, what in SF.get('via', []):
    (a_, kv_), _key = what
    c_ = {'lane': a_, 'u': chg[a_][kv_], 'w': VS[a_]}
    if v_ > GRID2 and c_ not in res['vcuts']:
        res['vcuts'].append(c_)
# ...and a via that stands outside the vias' free space (inside a bar of fixed copper), the same
_vout = [vk for vk in VKEYS if VMESH.locate([VIA[vk]])[0] < 0 and Point(VIA[vk]).distance(VFREE) > GRID2]
for (a_, kv_) in _vout:
    c_ = {'lane': a_, 'u': chg[a_][kv_], 'w': VS[a_]}
    if c_ not in res['vcuts']:
        res['vcuts'].append(c_)
if _vout:
    res['paid']['vcell'] = len(_vout)
    log(f'{len(_vout)} via(s) stand inside a bar of fixed copper: ' + ', '.join(f'{a_}.{kv_}' for a_, kv_ in _vout[:8]))
# where the slack had to be given up: the places the plan stands at its bars
res['tight'] = [[k_[6:], round(v_, 4), str(what)] for k_, vs_ in SF.items() if k_.startswith('slack') for v_, what in vs_]
# where a run crosses fixed copper no free way of its layer leads round: the lane cannot stand on that layer there
res['breach'] = [{'lane': k_[0], 'run': k_[1], 'layer': LNAME[RUN[k_]['L']], 'from': list(pa_), 'to': list(pb_),
                  'copper': labs_} for (k_, pa_, pb_, labs_) in BREACH]
for k_ in ('island_boxes', 'islands', 'flips', 'rules'):
    if k_ in REF:
        res[k_] = REF[k_]
# ...each a LAYER cut for the solve, as the loop's own are (whole_route.layer_cut): the lane off that layer across
# the copper it cannot pass
for (k_, pa_, pb_, labs_) in BREACH:
    L_ = RUN[k_]['L']
    c_ = {'lane': k_[0], 'island': labs_[0] if labs_ else 'copper', 'layer': 1 - L_ if L_ in (0, 1) else 0,
          'box': [min(pa_[0], pb_[0]) - LANE_ST, min(pa_[1], pb_[1]) - LANE_ST,
                  max(pa_[0], pb_[0]) + LANE_ST, max(pa_[1], pb_[1]) + LANE_ST]}
    if NL > 2:
        c_['blocked'] = [L_]
    if c_ not in res['lcuts']:
        res['lcuts'].append(c_)
res['changes'] = {n: list(chg.get(n, [])) for n in res['lanes']}
if OUT != '/dev/null':
    json.dump(res, open(OUT, 'w'))
    log(f'wrote {OUT}: {len(res["lanes"])} lanes, {len(res["vias"])} vias; via cuts {[(c_["lane"], round(c_["u"], 2)) for c_ in res["vcuts"]]}')

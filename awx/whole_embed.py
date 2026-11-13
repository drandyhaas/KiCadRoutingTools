"""whole_embed.py SOLVE.json OUT.json --ref=GEO.json [--draw=PNG] -- the whole route's geometry in BOARD xy: the
solve's braid (whole_solve.py: each lane's layers, the order of the lanes that share a layer, each via's side of every
other lane) fitted into the board's free space with the vias FREE.

Each routing layer's free space -- inside the board, a lane's bar off every piece of fixed copper on that layer --
is cut into triangles on the fixed copper's own corners (CELLS: no point of the mesh stands where nothing is), and so
is the vias' own free space (a via's bar off every layer's copper). A RUN is a lane's stretch on one layer between
two of its ends or vias. It crosses a chain of cells -- which says only which side of each piece of fixed copper it
passes -- and carries its own points: one where it crosses each cell edge (one number, its place along the edge) and
free points between them (xy, each inside its cell, so the run never leaves the free space). A via is a free point
too, inside a cell of the vias' space and the end cells of its two runs.

One LP a round places every via and every lane's points together: the shortest and straightest lanes, with every
two lanes of a layer a pitch apart, every lane a via's room from every other lane's via on the braid's side of it,
vias their own distance apart -- each rule held as a separating line read off the layout as it stands, every point
moving at most TRUST a round, so a rule the round holds is held. Lanes, vias and copper keep a comfortable margin
more wherever there is room for it (soft). A via pressed on an edge of a cell goes on to the next cell.

--ref=GEO.json: the layout it starts from and reads each run's cells and points off (a whole_geo.py geometry of the
same solve). OUT.json is written as whole_geo writes a geometry -- the polish, the audit and the snap read it as
they read that -- with a via cut for each via still short of its room, and under 'tight' every place the slack had
to be given up. A stage run on its own from a recorded round (replay_stage.py); the loop does not call it. The bench
from BENCH / NETS / DEST (whole_ctx).

  --draw=PNG / --draw0=PNG   the layout after the rounds / as it starts, exact geometry; --view=X0,Y0,X1,Y1 a window
                             of it, --names the runs' names, --draw-ref the reference's lines over it, --draw-cells
                             the cells, --draw-points each lane's own points
  --rounds=N --h=MM --trust=MM --bend=W --slack=SHARE --wslack=W   the rounds at most, a lane's point spacing, a
                             point's move a round, the bend's price, the slack asked and its price
  --show-paid --show-kinks   each round's largest shortfalls by name; where the layout's wiggles are"""
import sys, json, math, collections, heapq, time
import awx_settings
import numpy as np
import shapely
from shapely.geometry import box as _box, Point, LineString, Polygon
from shapely.ops import unary_union
from scipy.optimize import linprog
from scipy.sparse import coo_matrix
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
HPT = float(OPT.get('--h', 0.8))              # a lane's own points: this far apart along a straight stretch, mm
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


def portal(m, t, t2, x=None):
    """the point of a run where it crosses from triangle t into its neighbour t2: {'k': 'P', a, b (the edge's points:
    a on the run's right), 'len', 't' (its place along it from a), 'from', 'to'}"""
    i = m.to(t, t2)
    a, b = m.edge(t, i)
    A, B = m.V[a], m.V[b]
    ln = float(np.hypot(*(B - A)))
    tt = ln / 2 if x is None else float(min(max(((x[0] - A[0]) * (B[0] - A[0]) + (x[1] - A[1]) * (B[1] - A[1])) / ln, 0.0), ln))
    return {'k': 'P', 'a': a, 'b': b, 'len': ln, 't': tt, 'from': int(t), 'to': int(t2)}


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
        mm = max(1, int(math.ceil(ln / 0.02)))
        sam += [a + (b - a) * i / mm for i in range(1, mm + 1)]
    sam = np.array(sam)
    tri = m.locate(sam)
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
    full, fat, gaps, bridge = [seq[0]], [at[0]], 0, set()
    for t, j in zip(seq[1:], at[1:]):
        if t in m.N[full[-1]]:
            full.append(t); fat.append(j)
            continue
        p = m.path(full[-1], t, avoid) or m.path(full[-1], t)
        if p is None:
            raise SystemExit(f'whole_embed: {key}: no free cells of {LNAME[r["L"]]} join its reference\'s pieces')
        gaps += 1
        bridge |= set(p[1:-1])
        for t2 in p[1:]:
            full.append(t2); fat.append(j)
    fat.append(len(sam))
    # out of a cell and straight back within a sample or two is the sampling flickering along the edge: dropped. A
    # longer way out and back is the lane's own shape -- round another lane's via, say -- and is kept
    i = 0
    while i + 2 < len(full):
        if full[i] == full[i + 2] and fat[i + 2] - fat[i + 1] <= 2:
            del full[i + 1:i + 3]; del fat[i + 1:i + 3]
            i = max(i - 1, 0)
        else:
            i += 1
    # where it crosses into each cell after the first
    X = [None]
    for c in range(1, len(full)):
        i_ = m.to(full[c - 1], full[c])
        A, B = m.V[m.edge(full[c - 1], i_)[0]], m.V[m.edge(full[c - 1], i_)[1]]
        sub = sam[max(fat[c] - 1, 0):fat[c] + 1]
        x = shapely.ops.nearest_points(LineString(sub) if len(sub) > 1 else Point(sub[0]), LineString([A, B]))[1]
        X.append((x.x, x.y))
    out = []
    for c in range(len(full)):
        t = full[c]
        if c:
            out.append(portal(m, full[c - 1], t, X[c]))
        # its free points in this cell: a point wherever the reference bends (kept to 0.01 mm of it), and a
        # straight stretch longer than HPT filled -- every point inside the cell
        a0 = X[c] if c else tuple(sam[0])
        a1 = X[c + 1] if c + 1 < len(full) else tuple(sam[-1])
        mid = [tuple(sam[j]) for j in range(fat[c], fat[c + 1]) if tri[j] == t and m.inside(t, sam[j], -1e-6)]
        line = _simplify([a0] + mid + [a1], 0.01)
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
        if p['k'] == 'F':
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
                dr.line([Pt(*p) for p in PL[key]], fill=col[L_ % len(col)], width=max(1, int(round(TW * sc))))
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
VIEW = tuple(float(v) for v in OPT['--view'].split(',')) if '--view' in OPT else None
_x0 = same_layer_crossings(layout())
log(f'the layout it starts from, read onto its points: {len(_x0)} same-layer crossing(s)')
for c_ in _x0[:10]:
    log(f'   crossing: {c_[0]} x {c_[1]} on {LNAME[RUN[c_[0]]["L"]]} at {c_[2]}')
if '--draw0' in OPT:
    draw(OPT['--draw0'], 'whole_embed: the layout it starts from, read onto its points', VIEW, [c[2] for c in _x0])

# ---------------------------------------------------------------- the LP, a round at a time
VXO = {n: (_pairs.dive_offset(ctx.cfg, PP / 2) if n in prs else 0.0) for n in M}    # a pair's barrels off its line
ROUNDS = int(OPT.get('--rounds', 14))         # the rounds at most (no test yet of when a layout has settled)
TRUST = float(OPT.get('--trust', 0.15))       # a point's move in one round, mm (each of x and y; along its edge)
W_BEND = float(OPT.get('--bend', 1.0))        # the price of a point a millimetre off the line through its neighbours
SLACK = float(OPT.get('--slack', 0.10))       # the SLACK asked beyond every bar: this share of it, a grid step at least
W_SLACK = float(OPT.get('--wslack', 30.0))    # ...priced this a millimetre short of it (taken wherever there is room)
# a stretch's length: octilinear (what the snap will lay), the largest of these eight products -- flat to a zigzag
# inside one of its sectors, which the bend's price settles
DIRS = [(c_ * sx, s_ * sy) for (c_, s_) in ((1.0, math.sqrt(2) - 1), (math.sqrt(2) - 1, 1.0)) for sx in (1, -1) for sy in (1, -1)]
ORD = 1e-3                                    # two lanes on one cell edge: this far apart along it at least (their order)
slk = lambda need: need + max(need * SLACK, ctx.cfg.grid_step)       # a bar with its slack


class LP:
    """rows `sum coef * x <= b` over variables with costs and bounds; a row may be ELASTIC (a slack variable it pays
    its price a millimetre for -- its own, or one it shares with other rows), and what each kind paid is read back"""

    def __init__(self):
        self.c, self.lo, self.hi = [], [], []
        self.I, self.Jc, self.A, self.b = [], [], [], []
        self.slack = []                      # (slack variable, kind, what)

    def var(self, cost=0.0, lo=None, hi=None):
        self.c.append(cost); self.lo.append(lo); self.hi.append(hi)
        return len(self.c) - 1

    def paying(self, kind, what, price, cap=None):
        sl = self.var(price, 0.0, cap)
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
        A = coo_matrix((self.A, (self.I, self.Jc)), shape=(len(self.b), len(self.c))).tocsr()
        res = linprog(self.c, A_ub=A, b_ub=self.b, bounds=list(zip(self.lo, self.hi)), method='highs-ipm')   # (interior point: about 3 s a round where the dual simplex took 5 to 16)
        if res.status != 0:
            raise SystemExit(f'whole_embed: the LP has no answer ({res.message})')
        self.x, self.marg = res.x, res.ineqlin.marginals
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


def round_of(it):
    """one LP from the layout as it stands, and the layout moved to its answer.
    -> (what it paid by kind, the largest move, the vias moved on to a next cell)"""
    lp = LP()
    cur = layout()
    tS = time.time()
    # ---- variables: a via's xy, a run's place along each cell edge it crosses, its free points' xy
    vaff, vrows = {}, {}
    for vk in VKEYS:
        x, y = VIA[vk]
        vx, vy = lp.var(0.0, x - TRUST, x + TRUST), lp.var(0.0, y - TRUST, y + TRUST)
        vaff[vk] = (0.0, 0.0, [(vx, 1.0, 0.0), (vy, 0.0, 1.0)])
        n, kv = vk
        cells = [('v', VMESH, TV[vk]), ('a', MESH[RUN[(n, kv)]['L']], RUN[(n, kv)]['last']),
                 ('b', MESH[RUN[(n, kv + 1)]['L']], RUN[(n, kv + 1)]['first'])]
        vrows[vk] = {}
        for tag, m, t in cells:
            sl = lp.paying('vcell', vk, W_HARD)
            vrows[vk][tag] = [lp.ge([(vx, nx), (vy, ny)], c - 1e-7, sl) for (nx, ny, c) in m.rows_inside(t)]
    aff = {}
    for key in KEYS:
        n, k = key
        r = RUN[key]
        m = MESH[r['L']]
        pts = [vaff[(n, k - 1)] if k else (cur[key][0][0], cur[key][0][1], [])]
        for p in r['pts']:
            if p['k'] == 'P':
                A, B = m.V[p['a']], m.V[p['b']]
                lo, hi = hw[n], p['len'] - hw[n]
                if hi < lo:
                    lo = hi = p['len'] / 2
                tt = min(max(p['t'], lo), hi)
                p['v'] = lp.var(0.0, max(lo, tt - TRUST), min(hi, tt + TRUST))
                pts.append((float(A[0]), float(A[1]), [(p['v'], float((B[0] - A[0]) / p['len']), float((B[1] - A[1]) / p['len']))]))
                # its slack off the edge's two ends (fixed copper's corners)
                for (sgn, ref_) in ((1.0, lo), (-1.0, -hi)):
                    sl = lp.paying('slack-copper', key, W_SLACK)
                    lp.ge([(p['v'], sgn)], ref_ + ctx.cfg.grid_step, sl)
            else:
                x, y = p['xy']
                vx, vy = lp.var(0.0, x - TRUST, x + TRUST), lp.var(0.0, y - TRUST, y + TRUST)
                p['v'] = (vx, vy)
                pts.append((0.0, 0.0, [(vx, 1.0, 0.0), (vy, 0.0, 1.0)]))
                sl = lp.paying('cell', key, W_HARD)
                sl2 = None
                for i, (nx, ny, c) in enumerate(m.rows_inside(p['tri'])):
                    lp.ge([(vx, nx), (vy, ny)], c - 1e-7, sl)
                    if m.N[p['tri'], i] < 0:               # a wall: its slack off the fixed copper's bar
                        sl2 = sl2 if sl2 is not None else lp.paying('slack-copper', key, W_SLACK)
                        lp.ge([(vx, nx), (vy, ny)], c + ctx.cfg.grid_step, sl2)
        pts.append(vaff[(n, k)] if (n, k) in VIA else (cur[key][-1][0], cur[key][-1][1], []))
        aff[key] = pts
        for p, q in zip(pts, pts[1:]):                      # its length
            if not p[2] and not q[2]:
                continue
            lv = lp.var(W_LEN, 0.0, None)
            for o in DIRS:
                cq, tq = P_dot(o, q); cp, tp = P_dot(o, p)
                lp.le(tq + [(v, -a) for v, a in tp] + [(lv, -1.0)], -(cq - cp))
    # ---- the price of a bend: each point of a lane off the straight line through the points before and after it
    if W_BEND > 0:
        for n in M:
            la, lc = lane_line(n, aff), lane_line(n, cur)
            for j in range(1, len(la) - 1):
                l1 = math.hypot(lc[j][0] - lc[j - 1][0], lc[j][1] - lc[j - 1][1])
                l2 = math.hypot(lc[j + 1][0] - lc[j][0], lc[j + 1][1] - lc[j][1])
                if l1 + l2 < 1e-6 or not (la[j][2] or la[j - 1][2] or la[j + 1][2]):
                    continue
                w0, w2 = l2 / (l1 + l2), l1 / (l1 + l2)
                for ax in ((1.0, 0.0), (0.0, 1.0)):
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
            if x[0][0] == y[0][0]:
                continue
            tx = [(x[1]['v'], 1.0 if x[2] else -1.0)]
            ty = [(y[1]['v'], 1.0 if y[2] else -1.0)]
            off = (0.0 if y[2] else ln) - (0.0 if x[2] else ln)
            lp.ge(ty + [(v, -a_) for v, a_ in tx], ORD - off, lp.paying('order', (x[0], y[0]), W_HARD))
    # ---- rows: every lane apart from its NEIGHBOURS on its layer. Each point of a lane is held off the nearest
    # stretch of each lane beside it by ONE row: along the line from the point to its foot on that stretch as they
    # stand, the foot's place along the stretch kept. (Two lines' nearest approach is at a point of one of them, and
    # every lane's points are held, so both ways are.) A lane beyond another is no neighbour: the shortest way to it
    # crosses the lane between. The shortfall is priced in steps: free up to the comfortable pitch's, then the
    # slack's, then the pitch itself (W_HARD)
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
                for j in range(1, len(cur[key]) - 1):
                    pk.append(key); pj.append(j); pxy.append(cur[key][j])
        if not pk:
            continue
        PT = shapely.points(np.array(pxy))
        plane = np.array([li[k_[0]] for k_ in pk])
        qp, qs = tree.query(PT, predicate='dwithin', distance=P_COMF + hwm + TRUST)
        ok = plane[qp] != lane_[qs]
        qp, qs = qp[ok], qs[ok]
        ncand += len(qp)
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
        for q in np.nonzero(~blocked)[0]:
            ka, j = pk[qp[q]], pj[qp[q]]
            kb, i, _g = sl_[qs[q]]
            gap = P_MIN + hw[ka[0]] + hw[kb[0]]
            comf = P_COMF + hw[ka[0]] + hw[kb[0]]
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
                   lp.paying('pitch', (ka, kb), W_HARD)]
            lp.ge(terms + [(s_, 1.0) for s_ in sls], comf - ((1 - lam) * c0 + lam * c1 - cp_))
            npair += 1
    tB = time.time()
    # ---- rows: every lane a via's room off every other lane's via, on the braid's side of it
    tree = shapely.STRtree([s_[2] for s_ in allseg])
    wrong, said = 0, 0
    for vk in VKEYS:
        V = VIA[vk]
        a_ = vk[0]
        for ib in tree.query(Point(V), predicate='dwithin', distance=slk(VIA_R + PP + VXO[a_]) + 3 * TRUST):
            key, j, sg = allseg[ib]
            if key[0] == a_:
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
            if 0.0 < w < hl and hl >= 0.05:        # beside it: the braid says which side the lane passes
                side = via_bit(a_, vk[1], key[0], key[1])
                if side:
                    said += 1
                    geo = 1 if (nrm[0] * (P_[0] - V[0]) + nrm[1] * (P_[1] - V[1])) > 0 else -1
                    if side != geo or nv is None:
                        wrong += 1
                        nv = (nrm[0] * side, nrm[1] * side)
            if nv is None:
                nv = nrm
            sls = [lp.paying('slack-via', (vk, key), W_SLACK, slk(room) - room), lp.paying('via', (vk, key), W_HARD)]
            for q in (aff[key][j], aff[key][j + 1]):
                sep(lp, nv, vaff[vk], q, slk(room), sls)
    # ---- rows: two vias their own distance apart, along the line between them as they stand
    for i, v1 in enumerate(VKEYS):
        for v2 in VKEYS[i + 1:]:
            dx, dy = VIA[v2][0] - VIA[v1][0], VIA[v2][1] - VIA[v1][1]
            dl = math.hypot(dx, dy)
            need = VIA_VV + VXO[v1[0]] + VXO[v2[0]]
            if dl > slk(need) + 4 * TRUST:
                continue
            nv = (dx / dl, dy / dl) if dl > 1e-9 else (1.0, 0.0)
            sep(lp, nv, vaff[v1], vaff[v2], slk(need), [lp.paying('slack-via', (v1, v2), W_SLACK, slk(need) - need),
                                                       lp.paying('viavia', (v1, v2), W_HARD)])
    # ---- a via's slack off the walls of the vias' space
    for vk in VKEYS:
        sl = None
        for i, (nx, ny, c) in enumerate(VMESH.rows_inside(TV[vk])):
            if VMESH.N[TV[vk], i] < 0:
                sl = sl if sl is not None else lp.paying('slack-copper', vk, W_SLACK)
                (vx, _, _), (vy, _, _) = vaff[vk][2]
                lp.ge([(vx, nx), (vy, ny)], c + ctx.cfg.grid_step, sl)
    t1 = time.time()
    lp.solve()
    t2_ = time.time()
    x = lp.x
    paid = collections.defaultdict(list)
    for (sl, kind, what) in lp.slack:
        if kind != 'comfort' and x[sl] > 1e-6:
            paid[kind].append((round(float(x[sl]), 4), what))
    if '--show-paid' in OPT:
        for k_ in sorted(paid):
            for v_, what in sorted(paid[k_], key=lambda q_: -q_[0])[:6]:
                log(f'      paid {k_} {v_:.3f}: {what}')
    # ---- the layout moved to the answer
    move = 0.0
    for key in KEYS:
        for p in RUN[key]['pts']:
            if p['k'] == 'P':
                move = max(move, abs(float(x[p['v']]) - p['t']))
                p['t'] = float(x[p['v']])
            else:
                q = (float(x[p['v'][0]]), float(x[p['v'][1]]))
                move = max(move, math.hypot(q[0] - p['xy'][0], q[1] - p['xy'][1]))
                p['xy'] = q
    hops = 0
    for vk in VKEYS:
        (vx, _, _), (vy, _, _) = vaff[vk][2]
        q = (float(x[vx]), float(x[vy]))
        move = max(move, math.hypot(q[0] - VIA[vk][0], q[1] - VIA[vk][1]))
        VIA[vk] = q
        n, kv = vk
        # a via pressed on an edge of one of its cells goes on to the cell beyond it
        def pressed(tag, m, t):
            best = None
            for i, row in enumerate(vrows[vk][tag]):
                t2 = int(m.N[t, i])
                if lp.marg[row] < -1e-6 and t2 >= 0 and (best is None or lp.marg[row] < best[0]):
                    best = (lp.marg[row], t2)
            return None if best is None else best[1]
        t2 = pressed('v', VMESH, TV[vk])
        if t2 is not None:
            TV[vk] = t2
            hops += 1
        ra, rb = RUN[(n, kv)], RUN[(n, kv + 1)]
        ma, mb = MESH[ra['L']], MESH[rb['L']]
        t2 = pressed('a', ma, ra['last'])
        if t2 is not None:                          # the run that ends at it: one cell edge more, or its last one less
            jp = max((j for j, p in enumerate(ra['pts']) if p['k'] == 'P'), default=None)
            if jp is not None and ra['pts'][jp]['from'] == t2 and ra['pts'][jp]['to'] == ra['last']:
                del ra['pts'][jp:]
            else:
                ra['pts'].append(portal(ma, ra['last'], t2, q))
            ra['last'] = t2
            hops += 1
        t2 = pressed('b', mb, rb['first'])
        if t2 is not None:                          # the run that starts at it: its first cell edge less, or one more
            jp = min((j for j, p in enumerate(rb['pts']) if p['k'] == 'P'), default=None)
            if jp is not None and rb['pts'][jp]['from'] == rb['first'] and rb['pts'][jp]['to'] == t2:
                del rb['pts'][:jp + 1]
            else:
                rb['pts'].insert(0, portal(mb, t2, rb['first'], q))
            rb['first'] = t2
            hops += 1
    # ---- each lane's own points kept HPT apart: a long stretch gets a point, two points on top of each other lose one
    for key in KEYS:
        r = RUN[key]
        xy = run_xy(key)
        new, cell, prev = [], r['first'], xy[0]
        for p, q in zip(r['pts'], xy[1:-1]):
            d = math.hypot(q[0] - prev[0], q[1] - prev[1])
            if p['k'] == 'F' and d < 0.25 * HPT:
                continue
            if d > 1.6 * HPT:
                nn = int(math.ceil(d / HPT))
                new += [{'k': 'F', 'xy': (prev[0] + (q[0] - prev[0]) * i_ / nn, prev[1] + (q[1] - prev[1]) * i_ / nn),
                         'tri': cell} for i_ in range(1, nn)]
            new.append(p)
            prev = q
            if p['k'] == 'P':
                cell = p['to']
        d = math.hypot(xy[-1][0] - prev[0], xy[-1][1] - prev[1])
        if d > 1.6 * HPT:
            nn = int(math.ceil(d / HPT))
            new += [{'k': 'F', 'xy': (prev[0] + (xy[-1][0] - prev[0]) * i_ / nn, prev[1] + (xy[-1][1] - prev[1]) * i_ / nn),
                     'tri': cell} for i_ in range(1, nn)]
        r['pts'] = new
    # ---- what stands short after it
    PL = layout()
    sg2 = []
    for key in KEYS:
        pts = PL[key]
        for i in range(len(pts) - 1):
            if pts[i] != pts[i + 1]:
                sg2.append((key, LineString([pts[i], pts[i + 1]])))
    tr2 = shapely.STRtree([s_ for _, s_ in sg2])
    short, nvia, wshort = set(), 0, 0.0
    for ia, (ka, sa) in enumerate(sg2):
        for ib in tr2.query(sa, predicate='dwithin', distance=P_MIN + 2 * max(hw.values()) - 2e-4):
            kb = sg2[ib][0]
            if ib <= ia or ka[0] == kb[0] or RUN[ka]['L'] != RUN[kb]['L']:
                continue
            d_ = sa.distance(sg2[ib][1])
            if d_ < P_MIN + hw[ka[0]] + hw[kb[0]] - 2e-4:
                short.add((ka, kb) if ka < kb else (kb, ka))
                wshort = max(wshort, P_MIN + hw[ka[0]] + hw[kb[0]] - d_)
    for vk in VKEYS:
        pv = Point(VIA[vk])
        for ib in tr2.query(pv, predicate='dwithin', distance=VIA_R + PP + max(VXO.values())):
            kb = sg2[ib][0]
            if kb[0] != vk[0] and pv.distance(sg2[ib][1]) < VS[kb[0]] + VXO[vk[0]] - 2e-4:
                nvia += 1
    nx = len(same_layer_crossings(PL))
    ln_ = sum(float(np.hypot(*(np.array(PL[k_][1:]) - np.array(PL[k_][:-1])).T).sum()) for k_ in KEYS)
    hard = {k_: v for k_, v in paid.items() if not k_.startswith('slack')}
    soft = {k_: v for k_, v in paid.items() if k_.startswith('slack')}
    log(f'round {it}: {len(lp.c)} variables, {len(lp.b)} rows ({npair} point-to-neighbour rows of {ncand} near), '
        f'rows built {t1 - tS:.1f} s (pairs {tB - tA:.1f}), LP {t2_ - t1:.1f} s, after {time.time() - t2_:.1f} s; moved '
        f'{move:.3f}, {hops} via cell change(s); length {ln_:.1f}; paid '
        + (', '.join(f'{k_} {len(v)} (max {max(v)[0]:.3f})' for k_, v in sorted(hard.items())) or 'nothing')
        + '; slack given up: ' + (', '.join(f'{k_[6:]} {len(v)} (max {max(v)[0]:.3f})' for k_, v in sorted(soft.items())) or 'none')
        + f'; after it: {nx} same-layer crossing(s), {len(short)} run pair(s) short (worst {wshort:.3f}), {nvia} via room(s) short; '
        f'against the braid: {wrong} of {said} via sides')
    return paid, move, hops


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
for it in range(1, ROUNDS + 1):
    paid, move, hops = round_of(it)
    if move < 2e-3 and not hops:
        break
log(f'whole_embed: {it} round(s), {time.time() - T0:.1f} s')
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
        pts = sampled(PL[(n, k)])
        Ln = LNAME[RUN[(n, k)]['L']]
        pieces += [(p[0], p[1], q[0], q[1], Ln) for p, q in zip(pts, pts[1:])]
        xy += pts if not xy else pts[1:]
        if k:
            res['vias'].append((n,) + tuple(PL[(n, k)][0]))
    res['lanes'][n] = {'xy': xy, 'pieces': pieces}
res['paid'] = {k_: len(v_) for k_, v_ in paid.items()}
# a via the last round still held short of its room: the change it is, for the solve to move (whole_geo's via cut)
for v_, what in paid.get('via', []):
    (a_, kv_), _key = what
    c_ = {'lane': a_, 'u': chg[a_][kv_], 'w': VS[a_]}
    if v_ > GRID2 and c_ not in res['vcuts']:
        res['vcuts'].append(c_)
# where the slack had to be given up: the places the plan stands at its bars
res['tight'] = [[k_[6:], round(v_, 4), str(what)] for k_, vs_ in paid.items() if k_.startswith('slack') for v_, what in vs_]
for k_ in ('island_boxes', 'islands', 'flips', 'rules'):
    if k_ in REF:
        res[k_] = REF[k_]
res['changes'] = {n: list(chg.get(n, [])) for n in res['lanes']}
if OUT != '/dev/null':
    json.dump(res, open(OUT, 'w'))
    log(f'wrote {OUT}: {len(res["lanes"])} lanes, {len(res["vias"])} vias; via cuts {[(c_["lane"], round(c_["u"], 2)) for c_ in res["vcuts"]]}')

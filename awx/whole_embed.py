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

A PAIR's via is what the later stages lay there: two barrels either side of the pair's line, across ONE router
heading through the via, and the pair router's straight run along that heading either side. Its heading is read off
its lane (the router's nearest; the next either side where the via fits that way and not the nearest); it stands in
a cell of its OWN free space -- the places where both barrels stand a via's bar off every layer's copper and both
straight runs lie in their layers' free space; its two runs end at the ends of the straight run, points rigid with
the via; and every other lane keeps off each barrel, and off each of the cells the pair router tests for the dive,
by that one's own bar (--no-dives: a pair's via one point with a wider room).

One LP a round places every via and every lane's points together: the shortest and straightest lanes, with every
two lanes of a layer a pitch apart, every lane a via's room from every other lane's via on the braid's side of it,
vias their own distance apart -- each rule held as a separating line read off the layout as it stands, every point
moving at most its trust a round (halved where a point swings to and fro). Lanes, vias and copper keep a comfortable
margin more wherever there is room for it (soft). A rule may fall short, at a price, but NOTHING PASSES THROUGH
ANYTHING: two lanes of a layer, a lane and a via, two vias keep the sides they stand on, two lanes on one cell edge
their order, and a via never leaves its cells for the fixed copper's bar. A point pressed on an edge of its cell
with free space beyond goes on into the next cell (a lane pushed aside bulges into it), a via likewise; a run that
doubles back on itself with nothing in between is pulled straight -- by another way through the cells, never by
moving the places it crosses the same cell edges at: those are the LP's own. A round's LP holds only what stands
near where the round before moved, and the rounds end when nothing moves.

--ref=GEO.json: the layout it starts from and reads each run's cells off (a whole_geo.py geometry of the same solve).
Where that layout stands inside a bar of fixed copper its line is read out to the free space, the runs kept in the
order its own line has them and put a pitch apart. Where it crosses fixed copper no free cells of the layer lead
round, the run is read as it stands -- a BREACH, two fixed points either side of the copper: no geometry can lay
that lane on that layer there.

OUT.json is written as whole_geo writes a geometry -- the polish, the audit and the snap read it as they read that.
Its findings for the solve: a via cut for each via that stands short of its room, or inside a bar of fixed copper;
a layer cut for each breach (also under 'breach', with the copper's names); under 'tight' every place the slack had
to be given up. Every length the stage chooses is a multiple of the board's own pitch, track width or grid step.
The loop (whole_route.py) runs it after whole_geo.py when GEO_EMBED=1 is set -- its lanes the round's geometry, its
findings the round's cuts -- and not otherwise; on its own it runs from a recorded round (replay_stage.py). The bench
from BENCH / NETS / DEST (whole_ctx).

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
  --show-why                 every place the round moved, by its cause: a point of a run the LP moved (how far, its
                             trust), a point's or a via's cell change, a pull (the run, the stretch pulled)
  --watch=LANE.k,LANE.k      two runs followed round by round: their points, how near they stand, the rows
                             between them and each pull of either; with --show-rows, the LP's rows that hold each of
                             their points, by kind, with the price each puts on the point's move
  --probe=X,Y,R              the fixed copper and the cells' corners within R of a point (and the rules' sizes)
  --draw-thin                the lanes drawn as thin lines (a close view: each point's kind shows -- white a free
                             point, yellow one on a cell edge)"""
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


def _ringed(o):
    """a census piece that is another net's VIA, at its reach for a lane: the router's ring about the grid point the
    via rounds to (pairs.svia_reach, as whole_geo and the audit take it), not the via's own radius alone"""
    if not o[5].startswith('via '):
        return o
    cx, cy, r = (o[0] + o[2]) / 2, (o[1] + o[3]) / 2, (o[2] - o[0]) / 2
    r2 = max(r, _pairs.svia_reach(ctx.cfg, cx, cy, 2 * r))
    return (cx - r2, cy - r2, cx + r2, cy + r2) + tuple(o[4:])


_havec = {(round((o[0] + o[2]) / 2, 4), round((o[1] + o[3]) / 2, 4)) for o in CEN.obs if o[5].startswith('via ')}
FIXED = [_ringed(o) for o in CEN.obs if o[5] not in _isl]
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
NSTUB = 0
FIXNET = {}                                            # a stub's or its via's piece (its place in FIXED) -> its net
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
        FIXNET[len(FIXED) - 1] = s_.net_id
        NSTUB += 1
for v_ in ctx.base_vias:
    r_ = max(v_.size / 2, _pairs.svia_reach(ctx.cfg, v_.x, v_.y, v_.size))
    if (round(v_.x, 4), round(v_.y, 4)) not in _havec and _near(v_.x, v_.y, VIA_ST + bd.VIA_NEED):
        FIXED.append((v_.x - r_, v_.y - r_, v_.x + r_, v_.y + r_, frozenset(range(NL)),
                      'svia ' + whole_ctx.copper_name(ctx, v_)))
        FIXNET[len(FIXED) - 1] = v_.net_id
        NSTUB += 1
# ...and under a GROUP's part, on the layers it leaves open (no box there: whole_passages.Census), the bus's own stubs
# and vias standing inside its box whole -- the berths' vias between a connector's rows and their legs under it, which
# a lane laid under the part passes as it passes any copper (the census lists the other nets' there)
_OPENB = [Fr.DBs[r_] for r_ in Fr.dests if Fr.group is not None and Fr.group.open[r_]]
_inopen = lambda x, y: any(b[0] <= x <= b[2] and b[1] <= y <= b[3] for b in _OPENB)     # noqa: E731
NOPEN = 0
for s_ in (ctx.base_segments if _OPENB else ()):
    if s_.layer not in LI or s_.net_id not in bus_ids:
        continue
    ln_ = math.hypot(s_.end_x - s_.start_x, s_.end_y - s_.start_y)
    k_ = max(1, int(math.ceil(ln_ / 0.2)))
    r_ = s_.width / 2
    for j_ in range(k_):
        ax, ay = s_.start_x + (s_.end_x - s_.start_x) * j_ / k_, s_.start_y + (s_.end_y - s_.start_y) * j_ / k_
        bx, by = (s_.start_x + (s_.end_x - s_.start_x) * (j_ + 1) / k_,
                  s_.start_y + (s_.end_y - s_.start_y) * (j_ + 1) / k_)
        if not (_inopen(ax, ay) or _inopen(bx, by)):
            continue
        FIXED.append((min(ax, bx) - r_, min(ay, by) - r_, max(ax, bx) + r_, max(ay, by) + r_,
                      frozenset([LI[s_.layer]]), 'stub ' + whole_ctx.copper_name(ctx, s_)))
        FIXNET[len(FIXED) - 1] = s_.net_id
        NOPEN += 1
for v_ in (ctx.base_vias if _OPENB else ()):
    if v_.net_id not in bus_ids or not _inopen(v_.x, v_.y) or (round(v_.x, 4), round(v_.y, 4)) in _havec or \
            _near(v_.x, v_.y, VIA_ST + bd.VIA_NEED):
        continue
    r_ = max(v_.size / 2, _pairs.svia_reach(ctx.cfg, v_.x, v_.y, v_.size))
    FIXED.append((v_.x - r_, v_.y - r_, v_.x + r_, v_.y + r_, frozenset(range(NL)),
                  'svia ' + whole_ctx.copper_name(ctx, v_)))
    FIXNET[len(FIXED) - 1] = v_.net_id
    NOPEN += 1


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
# (each layer's bars as a tree of boxes, a hair inside each: whether a shape lies WHOLLY in the layer's free space is
# asked of it -- of the one large polygon the free space is, the same question took a minute a round on a layer of
# four thousand cells)
BARS, BARPIECE = [], []                                # (BARPIECE: each box's piece, its place in FIXED)
for L_ in range(NL):
    bs_, ix_ = [], []
    for i_, (x0, y0, x1, y1, ls, lab) in enumerate(FIXED):
        if L_ in ls:
            g = (TW / 2 if lab.startswith(ARRAY) else LANE_ST) - 1e-6
            if not (x1 + g < WX0 or x0 - g > WX1 or y1 + g < WY0 or y0 - g > WY1):
                bs_.append(_box(x0 - g, y0 - g, x1 + g, y1 + g))
                ix_.append(i_)
    BARS.append(shapely.STRtree(bs_) if bs_ else None)
    BARPIECE.append(ix_)


def in_free(L_, geom):
    """geom wholly in layer L_'s free space"""
    return WINB.covers(geom) and (BARS[L_] is None or len(BARS[L_].query(geom, predicate='intersects')) == 0)


OWN_IDS = {n: {ctx.byname[n][0]} | {ctx.byname[leg][0] for leg in prs.get(n, ()) if leg in ctx.byname} for n in M}
_own_corner = {}


def own_corner(n, L_, vi):
    """the cells' corner vi of layer L_ stands on the bar of lane n's OWN copper alone -- its stubs' ends, the vias
    they end in (every lane's stub is a piece of the fixed copper, its own too: its run begins where it leaves that
    bar) -- and on no other piece's, nor on the board's edge"""
    got = _own_corner.get((n, L_, vi))
    if got is None:
        hit = BARS[L_].query(Point(MESH[L_].V[vi]), predicate='dwithin', distance=4e-6) if BARS[L_] is not None else ()
        got = _own_corner[(n, L_, vi)] = bool(len(hit)) and all(FIXNET.get(BARPIECE[L_][q_]) in OWN_IDS[n] for q_ in hit)
    return got


if '--probe' in OPT:                               # (debugging: the fixed copper near a point, and then stop)
    px_, py_, pr_ = (float(v) for v in OPT['--probe'].split(','))
    for (x0, y0, x1, y1, ls, lab) in FIXED:
        if x0 - pr_ <= px_ <= x1 + pr_ and y0 - pr_ <= py_ <= y1 + pr_:
            log(f'   probe: {lab:28s} box ({x0:.3f}, {y0:.3f}) - ({x1:.3f}, {y1:.3f}) layers {sorted(LNAME[l_] for l_ in ls)}')
    for L_ in range(NL):
        vs_ = [tuple(round(float(c_), 3) for c_ in v_) for v_ in MESH[L_].V if abs(v_[0] - px_) < pr_ and abs(v_[1] - py_) < pr_]
        log(f'   probe: {LNAME[L_]} mesh corners near: {vs_}')
    log(f'   probe: LANE_ST {LANE_ST:.4f} VIA_ST {VIA_ST:.4f} VIA_R {VIA_R:.4f} VIA_VV {VIA_VV:.4f} P_MIN {P_MIN:.4f} PP {PP:.4f} hw {max(hw.values()):.4f} GS {GS} TW {TW} CL {CL} via {bd.VIA_SIZE}')
    log(f'   probe: a single\'s via off a pair\'s line {VIA_R + PP / 2:.4f}')
# ---------------------------------------------------------------- a pair's via: two barrels and a straight run
# What the later stages lay at a pair's via, and ask of it (whole_polish: pair_dive_straights, barrels, pose_cells and
# its rows 'via-lane', 'via-via', 'via-static', 'dive-lane', 'dive-via'; plan_audit.check_dives): ONE router heading
# through the via, the lane straight along it for the pair router's straight run (pairs.via_straight) and a grid
# step either side; a BARREL pairs.dive_offset either side of that line, across the heading; and the three CELLS the
# pair router tests for the dive -- the via's place, and pairs.pose_via_cells grid steps either way along the
# heading's integer perpendicular. Each barrel stands a via's bar off the fixed copper, a via's ring off every other
# lane's line and a via's distance off every other barrel; each cell the pair's own bar off every other lane's line,
# a via and the clearance off every other via's place. The sizes below are the polish's own, each read from the same
# rule of pairs.py: the place this stage leaves a pair's via in is one where what the polish lays stands as laid.
# (As ONE point with a wider room -- what it was here -- a pair's via stood where its barrels did not fit: beside a
# pad, a barrel inside the pad's bar; the polish then moved the other lanes off its cells into bars of their own.)
_cfg = ctx.cfg
_R2 = math.sqrt(0.5)
OCT8 = [(1.0, 0.0), (_R2, _R2), (0.0, 1.0), (-_R2, _R2), (-1.0, 0.0), (-_R2, -_R2), (0.0, -1.0), (_R2, -_R2)]
OCT8I = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]      # ...as the router's integer steps
DIVES = bool(prs) and '--no-dives' not in OPT
HALF = PP / 2
OFF = _pairs.dive_offset(_cfg, HALF) if prs else 0.0                 # a barrel off the pair's line
_goff = lambda v: abs(v - round(v / GS) * GS)
# (how far off its grid point a barrel can stand once the snap lays the dive on the grid, the worst of the headings,
# twice: the router rings the point it rounds to)
VX_OFF = 2 * max(math.hypot(_goff(OFF * -dy / math.hypot(dx, dy)), _goff(OFF * dx / math.hypot(dx, dy)))
                 for dx, dy in OCT8I[:4]) if prs else 0.0
RING, RING_PAIR = _pairs.via_ring(_cfg), _pairs.via_ring(_cfg, HALF)     # a via's reach into a track's grid cells
DIVE_RUN = _pairs.via_straight(_cfg, OCT8[1]) + GS       # the straight run either side of the via: the longer
#                                                          (diagonal) heading's -- the stretch the polish reads
#                                                          the dive's heading over, whatever the heading
SPC = _pairs.pose_via_cells(_cfg, HALF) if prs else 0    # the cells the pair router tests: this many steps across
R_LINE = _cfg.via_size / 2 + _cfg.track_width / 2 + _cfg.clearance + HALF     # ...each this off another lane's line
R_VIA = _cfg.via_size + _cfg.clearance                                          # ...and this off another via's place
if DIVES:
    for n_ in prs:       # (a single's via off a PAIR's line: the router's ring for a pair's centreline and a grid step,
        if n_ in VS:     # as the polish asks it -- its ring and half the pitch can fall a part of a grid step short of that)
            VS[n_] = max(VS[n_], RING_PAIR + 2 * GRID2)
VH = {}              # a pair's via -> its router heading (0..7, OCT8)
VXOV = set()         # ...those that are CROSSOVERS: a crossed pair's first dive, where its legs swap sides
VLAID = set()        # ...those whose straight run is laid: the ends of its two runs stand rigid with the via
_shape, _space = {}, {}


def dive_shape(h, xo):
    """a pair's dive on router heading h: {'barrels': each barrel's offset from the via's place, 'cells': each
    tested cell's (the first the via's own place), 'runs': the straight run (before the via, after it)}. A CROSSOVER
    (xo; pairs.crossover_shape): its two barrels stand on ONE side of the pair, staggered along the heading, and the
    side is chosen later, by the room -- both sides' barrels are kept clear here -- and its runs are its own"""
    got = _shape.get((h, xo))
    if got is None:
        u, (ix, iy) = OCT8[h], OCT8I[h]
        px, py = -u[1], u[0]
        cells = [(0.0, 0.0), (-iy * SPC * GS, ix * SPC * GS), (iy * SPC * GS, -ix * SPC * GS)]
        barrels, runs = [(px * OFF, py * OFF), (-px * OFF, -py * OFF)], (DIVE_RUN, DIVE_RUN)
        shs = [s_ for s_ in (_pairs.crossover_shape(_cfg, u, side) for side in (1, -1)) if s_ is not None] if xo else []
        if shs:
            barrels = [(a_ * u[0] + c_ * px, a_ * u[1] + c_ * py) for s_ in shs for a_, c_ in s_[0]]
            runs = (max(DIVE_RUN, max(s_[1] for s_ in shs) + GS), max(DIVE_RUN, max(s_[2] for s_ in shs) + GS))
        got = _shape[(h, xo)] = {'barrels': barrels, 'cells': cells, 'runs': runs}
    return got


def dive_space(h, xo, Lin, Lout):
    """the places a pair's via may stand with heading h, its run arriving on layer Lin and leaving on Lout -- its OWN
    free space, cut into cells as the vias' is: every place where each barrel stands a via's bar (and its reach off
    the grid) off every layer's fixed copper, and the straight run either side lies the pair's own bar off the copper
    of its layer. The fixed copper is boxes: a barrel's offset moves a box back by it, a straight run sweeps the box
    back along it"""
    got = _space.get((h, xo, Lin, Lout))
    if got is None:
        sh, u = dive_shape(h, xo), OCT8[h]
        gv, reach = VIA_ST + VX_OFF, max(sh['runs']) + OFF + VIA_ST + VX_OFF + max(hw.values())
        ps = []
        for (x0, y0, x1, y1, ls, lab) in FIXED:
            if x1 + reach < WX0 or x0 - reach > WX1 or y1 + reach < WY0 or y0 - reach > WY1:
                continue
            for (ox, oy) in sh['barrels']:
                ps.append(_box(x0 - gv - ox, y0 - gv - oy, x1 + gv - ox, y1 + gv - oy))
            for (L_, run, sg) in ((Lin, sh['runs'][0], -1.0), (Lout, sh['runs'][1], 1.0)):
                if L_ in ls:
                    g = (TW / 2 if lab.startswith(ARRAY) else LANE_ST) + max(hw.values())
                    dx, dy = -sg * u[0] * run, -sg * u[1] * run
                    ps.append(shapely.MultiPoint([(x0 - g, y0 - g), (x1 + g, y0 - g), (x1 + g, y1 + g), (x0 - g, y1 + g),
                                                  (x0 - g + dx, y0 - g + dy), (x1 + g + dx, y0 - g + dy),
                                                  (x1 + g + dx, y1 + g + dy), (x0 - g + dx, y1 + g + dy)]).convex_hull)
        got = _space[(h, xo, Lin, Lout)] = Mesh(WINB.difference(unary_union(ps)) if ps else WINB)
    return got


def vmesh(vk):
    """the cells via vk stands in: the vias' free space -- a pair's via its own (dive_space)"""
    if vk in VH:
        return dive_space(VH[vk], vk in VXOV, RUN[vk]['L'], RUN[(vk[0], vk[1] + 1)]['L'])
    return VMESH


def via_discs(vk):
    """what via vk is to the lanes and the vias round it: [(a disc's offset from the via's place, its kind)] -- a
    single's via one disc ('s', the via); a pair's its barrels ('b') and its cells ('c': the first the via's place)"""
    if vk not in VH:
        return [((0.0, 0.0), 's')]
    sh = dive_shape(VH[vk], vk in VXOV)
    return [(o, 'b') for o in sh['barrels']] + [(o, 'c') for o in sh['cells']]


def disc_room(vk, kind, lane):
    """a disc's bar off lane's line: a single's via its room; a barrel the router's ring (a pair's line by the
    pair's ring), its reach off the grid and a grid step (the snap's share of the gap holds a row); a cell the
    pair's own bar, the lane's half width where it is a pair, and three half steps (the polish's own)"""
    if kind == 's':
        return VS[lane] + VXO[vk[0]]
    if kind == 'b':
        return (RING_PAIR if lane in prs else RING) + VX_OFF + GS
    return R_LINE + hw[lane] + 3 * GRID2


def via_rooms(v1, v2):
    """what two vias keep between them: [(a disc's offset from v1's place, one's from v2's, their distance at
    least)] -- barrel to barrel (a single's via its own one) a via's distance, and what a pair's barrels stand
    off the grid; a pair's cells a via and the clearance, and three half steps, off the other via's place"""
    if v1 not in VH and v2 not in VH:
        return [((0.0, 0.0), (0.0, 0.0), VIA_VV + VXO[v1[0]] + VXO[v2[0]])]
    d1, d2 = via_discs(v1), via_discs(v2)
    need = VIA_VV + (VX_OFF + GRID2 if (v1 in VH and v2 in VH) else max(VX_OFF, GRID2))
    out = [(o1, o2, need) for o1, k1 in d1 if k1 != 'c' for o2, k2 in d2 if k2 != 'c']
    out += [(o1, (0.0, 0.0), R_VIA + 3 * GRID2) for o1, k1 in d1 if k1 == 'c']
    out += [((0.0, 0.0), o2, R_VIA + 3 * GRID2) for o2, k2 in d2 if k2 == 'c']
    return out


def via_short(vk, lane, geom):
    """how far short of its room via vk stands off lane's line geom: the most over its discs (negative: clear)"""
    V = VIA[vk]
    return max(disc_room(vk, k_, lane) - Point(V[0] + o[0], V[1] + o[1]).distance(geom) for o, k_ in via_discs(vk))


def _walk(pts, d):
    """the point d along the line pts from its first point (its last where the line is shorter)"""
    for a, b in zip(pts, pts[1:]):
        ln = math.hypot(b[0] - a[0], b[1] - a[1])
        if ln >= d and ln > 1e-12:
            return (a[0] + (b[0] - a[0]) * d / ln, a[1] + (b[1] - a[1]) * d / ln)
        d -= ln
    return tuple(pts[-1])


def dive_headings(back, fwd):
    """the router headings by how near they are the way a lane runs through its via -- the chord from DIVE_RUN back
    along the line `back` (from the via, against the lane) to DIVE_RUN on along `fwd`, as the polish reads a dive's
    heading -- the nearest first"""
    a, b = _walk(back, DIVE_RUN), _walk(fwd, DIVE_RUN)
    c = (b[0] - a[0], b[1] - a[1])
    return sorted(range(8), key=lambda h: -(OCT8[h][0] * c[0] + OCT8[h][1] * c[1]))


def fit_dive(vk, V, hs):
    """via vk's heading and place: of the headings hs (the nearest its lane's own way first) the first the via fits
    with where it stands, V -- its barrels and its straight run clear of the fixed copper -- or, where none does,
    the one whose space is nearest (the nearer heading unless another's is half a grid step nearer), and the nearest
    place of it. -> (heading, place) or None"""
    n, kv = vk
    best = None
    for h in hs:
        m = dive_space(h, vk in VXOV, RUN[vk]['L'], RUN[(n, kv + 1)]['L'])
        if not len(m.T):
            continue
        if m.locate([V])[0] >= 0:
            return h, V
        q = shapely.ops.nearest_points(m.free, Point(V))[0]
        c = m.cen[int(m._tree.nearest(q))]
        d = math.hypot(q.x - V[0], q.y - V[1])
        if best is None or d < best[0] - GRID2:
            best = (d, h, (float(q.x + (c[0] - q.x) * 1e-3), float(q.y + (c[1] - q.y) * 1e-3)))
    return None if best is None else (best[1], best[2])


log(f'fixed copper: {len(FIXED)} piece(s), {NSTUB} of them stubs and vias on the arrays\' box lines'
    + (f', {NOPEN} the bus\'s own under a group\'s parts open on some layer' if _OPENB else '') + '; cells: '
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
if DIVES:
    # a pair's via: a crossover where it is a crossed pair's first; its heading the router's nearest the reference's
    # own way through it -- or the next either side, where the via as it stands is nearer fitting that way
    for vk in VKEYS:
        if vk[0] in prs:
            if vk[1] == 0 and vk[0] in getattr(ctx, 'pair_ends', {}) and _pairs.opposite_hands(ctx, vk[0]):
                VXOV.add(vk)
            got = fit_dive(vk, VIA[vk], dive_headings(RUN[vk]['ref'][::-1], RUN[(vk[0], vk[1] + 1)]['ref'])[:3])
            if got is not None:
                VH[vk], VIA[vk] = got
for vk in VKEYS:
    vm_ = vmesh(vk)
    t = int(vm_.locate([VIA[vk]])[0])
    if t < 0:
        q = shapely.ops.nearest_points(vm_.free, Point(VIA[vk]))[0]
        t = int(vm_._tree.nearest(q))
        c = vm_.cen[t]
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
        if p['k'] == 'D':                     # a dive's end: rigid with its via
            pts.append((VIA[p['vk']][0] + p['o'][0], VIA[p['vk']][1] + p['o'][1]))
        elif p['k'] != 'P':
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
                dr.line([Pt(*p) for p in own], fill=col[L_ % len(col)],
                        width=2 if '--draw-thin' in OPT else max(1, int(round(TW * sc))))
                if '--draw-points' in OPT:
                    for p, it_ in zip(PL[key][1:-1], RUN[key]['pts']):
                        cx, cy = Pt(*p)
                        rr_ = 4 if it_['k'] == 'F' else 3
                        dr.ellipse([cx - rr_, cy - rr_, cx + rr_, cy + rr_],
                                   fill=(255, 255, 255) if it_['k'] == 'F' else (255, 220, 0) if it_['k'] == 'P' else (0, 255, 255))
    r_ = bd.VIA_SIZE / 2 * sc
    for vk, (x, y) in VIA.items():
        for (ox_, oy_) in (dive_shape(VH[vk], vk in VXOV)['barrels'] if vk in VH else [(0.0, 0.0)]):     # (a pair's barrels)
            cx, cy = Pt(x + ox_, y + oy_)
            dr.ellipse([cx - r_, cy - r_, cx + r_, cy + r_], outline=(255, 255, 255), width=max(1, int(sc * 0.02)))
    if '--draw-ref' in OPT:
        for key in KEYS:
            c_ = tuple(min(255, v + 110) for v in col[RUN[key]['L'] % len(col)])
            dr.line([Pt(*p) for p in RUN[key]['ref']], fill=c_, width=1)
    if '--names' in OPT:
        for key in KEYS:
            # (the name at the run's middle point in the window)
            inw = [p for p in PL[key] if x0 <= p[0] <= x1 and y0 <= p[1] <= y1]
            if inw:
                p = inw[len(inw) // 2]
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
# (VREACH: the farthest a via's room reaches from its place -- a pair's barrels and cells stand off it, on any heading)
VREACH = VIA_R + PP + max(VXO.values())
for h_, xo_ in {(h_, vk_ in VXOV) for vk_ in VH for h_ in range(8)}:
    VREACH = max([VREACH] + [math.hypot(*o_) + RING_PAIR + VX_OFF + GS for o_ in dive_shape(h_, xo_)['barrels']]
                 + [math.hypot(*o_) + R_LINE + max(hw.values()) + 3 * GRID2 for o_ in dive_shape(h_, xo_)['cells']])
if REACH <= 0:
    REACH = max(P_COMF + 2 * max(hw.values()) + TRUST, slk(VREACH) + 3 * TRUST)


class LP:
    """rows `sum coef * x <= b` over variables with costs and bounds; a row may be ELASTIC (a slack variable it pays
    its price a millimetre for -- its own, or one it shares with other rows; capped where the rule may give only so
    far), and what each kind paid is read back"""

    def __init__(self):
        self.c, self.lo, self.hi = [], [], []
        self.I, self.Jc, self.A, self.b = [], [], [], []
        self.slack = []                      # (slack variable, kind, what)
        self.kind, self.rk = '', []          # (debugging: each row's kind)

    def var(self, cost=0.0, lo=None, hi=None):
        self.c.append(cost); self.lo.append(lo); self.hi.append(hi)
        return len(self.c) - 1

    def paying(self, kind, what, price, cap=None):
        sl = self.var(price, 0.0, None if cap is None else max(cap, 0.0))
        self.slack.append((sl, kind, what))
        return sl

    def _run(self, A):
        res = None
        for meth in ('highs-ipm', 'highs-ds'):   # (interior point: a third of the dual simplex's time on the largest)
            res = linprog(self.c, A_ub=A, b_ub=self.b if self.b else None, bounds=list(zip(self.lo, self.hi)), method=meth)
            if res.status == 0:
                break
        return res

    def le(self, terms, rhs, sl=None):
        r = len(self.b)
        for v, a in terms:
            if a != 0.0:
                self.I.append(r); self.Jc.append(v); self.A.append(a)
        if sl is not None:
            for s_ in (sl if isinstance(sl, (list, tuple)) else (sl,)):
                self.I.append(r); self.Jc.append(s_); self.A.append(-1.0)
        self.b.append(rhs)
        self.rk.append(self.kind)
        return r

    def ge(self, terms, rhs, sl=None):
        return self.le([(v, -a) for v, a in terms], -rhs, sl)

    def solve(self):
        """the LP solved: True. The stage never stops on a round: where the capped rows leave no answer (a layout
        that already stands past a cap in a way the caps did not allow for), the round is solved with the caps
        LIFTED and what went past them is said (`over`: {kind: how many, the most}); where even that has none,
        False, and the layout stands as it is"""
        A = coo_matrix((self.A, (self.I, self.Jc)), shape=(len(self.b), len(self.c))).tocsr() if self.b else None
        self.over = {}
        res = self._run(A)
        if res.status != 0:
            # (the HARD rules' caps alone: a soft margin keeps its own, or the cheap one would take up everything
            # and what could not be held would not show)
            caps = {sl: self.hi[sl] for (sl, _k, _w) in self.slack if self.hi[sl] is not None and self.c[sl] >= W_HARD}
            for sl in caps:
                self.hi[sl] = None
            res = self._run(A)
            if res.status != 0:
                log(f'      the round\'s LP has no answer, its caps lifted too ({res.message.strip()[:80]}): the layout stands')
                return False
            for (sl, kind, _w) in self.slack:
                if sl in caps and res.x[sl] > caps[sl] + 1e-7:
                    n_, m_ = self.over.get(kind, (0, 0.0))
                    self.over[kind] = (n_ + 1, max(m_, float(res.x[sl] - caps[sl])))
            log('      the round\'s LP has no answer inside its caps: solved with them lifted; past a cap: '
                + (', '.join(f'{k_} {n_} (most {m_:.3f})' for k_, (n_, m_) in sorted(self.over.items())) or 'none'))
        self.x = res.x
        self.marg = res.ineqlin.marginals if self.b else np.zeros(0)
        self.lom, self.him = res.lower.marginals, res.upper.marginals
        return True


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
        discs = [(Point(VIA[vk][0] + o_[0], VIA[vk][1] + o_[1]), k_) for o_, k_ in via_discs(vk)]
        for b_ in tr.query(pv, predicate='dwithin', distance=slk(VREACH)):
            kb = sg[b_][0]
            if kb[0] == vk[0]:
                continue
            for pd_, k_ in discs:                   # (a pair's via: each of its barrels and its cells)
                room = disc_room(vk, k_, kb[0])
                d_ = pd_.distance(sg[b_][1])
                if d_ < room - 2e-4:
                    out['via'][(vk, kb)] = max(out['via'].get((vk, kb), 0.0), room - d_)
                elif d_ < slk(room) - 2e-4:
                    out['slack-via'][(vk, kb)] = max(out['slack-via'].get((vk, kb), 0.0), slk(room) - d_)
    for i, v1 in enumerate(VKEYS):
        for v2 in VKEYS[i + 1:]:
            for o1, o2, need in via_rooms(v1, v2):
                d_ = math.hypot(VIA[v2][0] + o2[0] - VIA[v1][0] - o1[0], VIA[v2][1] + o2[1] - VIA[v1][1] - o1[1])
                if d_ < need - 2e-4:
                    out['viavia'][(v1, v2)] = max(out['viavia'].get((v1, v2), 0.0), need - d_)
                elif d_ < slk(need) - 2e-4:
                    out['slack-via'][(v1, v2)] = max(out['slack-via'].get((v1, v2), 0.0), slk(need) - d_)
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


def dive_ends(vk):
    """where via vk's two runs end, from the via's place: (the arriving run's end, the leaving run's start) -- the
    ends of a laid dive's straight run, else the via itself"""
    if vk not in VLAID:
        return (0.0, 0.0), (0.0, 0.0)
    return RUN[vk]['pts'][-1]['o'], RUN[(vk[0], vk[1] + 1)]['pts'][0]['o']


def stands_clear(n, L_, new, old, lines):
    """a new line for a stretch of lane n on layer L_ stands no nearer anything than its bar, or than the line it
    replaces did: every other lane's run of that layer (`lines`: each run's line), every other lane's via"""
    for k2, g2 in lines.items():
        if k2[0] != n and RUN[k2]['L'] == L_:
            d1 = new.distance(g2)
            if d1 < P_MIN + hw[n] + hw[k2[0]] and d1 < old.distance(g2) - 1e-9:
                return False
    for v2 in VKEYS:
        if v2[0] != n and Point(VIA[v2]).distance(new) < VREACH:
            s1 = via_short(v2, n, new)
            if s1 > 0 and s1 > via_short(v2, n, old) + 1e-9:
                return False
    return True


def lay_dive(vk, lines):
    """a pair's dive LAID: each of its two runs ends, and begins, at the end of the via's straight run -- a point
    rigid with the via ('D': the via's place and a fixed offset along its heading), so the two runs stand straight
    and in line through the via whatever the LP moves. Each run is cut where it comes on to that end, and goes
    straight to it from there -- by a new line no nearer any other lane or via than its bar, or than the run's own
    line there was, where there is one (`lines`: each run's line as it stands). -> '' (laid), or why not: an end
    falls outside its layer's free space, or no point of the run comes on to it with a straight way to it"""
    n, kv = vk
    u = OCT8[VH[vk]]
    V = VIA[vk]
    new = {}
    for key, run, sg in (((n, kv), dive_shape(VH[vk], vk in VXOV)['runs'][0], -1.0),
                         ((n, kv + 1), dive_shape(VH[vk], vk in VXOV)['runs'][1], 1.0)):
        r = RUN[key]
        m = MESH[r['L']]
        o = (sg * u[0] * run, sg * u[1] * run)
        E = (V[0] + o[0], V[1] + o[1])
        tE = int(m.locate([E])[0])
        if tE < 0:
            return f'the end of its straight run {("before", "after")[sg > 0]} it, ({E[0]:.2f}, {E[1]:.2f}), outside its layer\'s free space'
        xy = run_xy(key)
        got = None
        # the run's first point, from the via outward, that comes on to the straight run's end within 45 degrees of
        # the heading (a router's turn); where none does -- the lane turns at its via -- the first beyond the end
        # along the heading and a straight run's length from it (the lane then turns at the end, by less than a
        # right angle, and the bend's price draws the points after it into line). Of those, the first whose new
        # line stands clear of the other lanes and vias; where none does, the first all the same -- the dive is
        # what the later stages lay there whatever stands near it, and the LP's rows then make it its room
        for near, clear in ((True, True), (False, True), (True, False), (False, False)):
            for j in (range(len(xy) - 2, -1, -1) if sg < 0 else range(1, len(xy))):
                # (never past a breach, nor another dive's end: the run's far end may be one)
                gone = r['pts'][j:] if sg < 0 else r['pts'][:j - 1]
                if any(p['k'] in ('X', 'D') for p in gone):
                    break
                al = sg * (u[0] * (xy[j][0] - E[0]) + u[1] * (xy[j][1] - E[1]))       # beyond the end, along the dive
                ac = abs(u[0] * (xy[j][1] - E[1]) - u[1] * (xy[j][0] - E[0]))         # ...and off its line
                if al < GS or (ac > al if near else math.hypot(al, ac) < run):
                    continue
                items = chord_items(m, xy[j], _cell_after(r, j - 1), E, tE) if sg < 0 else \
                    chord_items(m, E, tE, xy[j], _cell_before(r, j - 1))
                if items is not None and (not clear or stands_clear(
                        n, r['L'], LineString([xy[j], E, V]), LineString(xy[j:] if sg < 0 else xy[:j + 1]), lines)):
                    got = (j, items)
                    break
            if got is not None:
                break
        if got is None:
            return f'its run {("before", "after")[sg > 0]} it comes on to no straight run\'s end'
        j, items = got
        d = {'k': 'D', 'vk': vk, 'o': o, 'tri': tE, 'v': None}
        new[key] = (r['pts'][:j] + items + [d], None, tE) if sg < 0 else ([d] + items + r['pts'][j - 1:], tE, None)
    for key, (pts, first, last) in new.items():
        RUN[key]['pts'] = pts
        if first is not None:
            RUN[key]['first'] = first
        if last is not None:
            RUN[key]['last'] = last
    VLAID.add(vk)
    return ''


def unlay_dive(vk):
    """a laid dive's two runs ordinary runs again, each ending at the via: straight from the point it was cut at
    (the straight run's own shape is not kept: the lane's way through the via is read again off what is left) --
    or, where that straight way leaves the cells, through a free point where the straight run's end stood"""
    n, kv = vk
    V = VIA[vk]
    for key, at_end in (((n, kv), True), ((n, kv + 1), False)):
        r = RUN[key]
        m = MESH[r['L']]
        d = r['pts'][-1] if at_end else r['pts'][0]
        E = (V[0] + d['o'][0], V[1] + d['o'][1])
        tV = int(m.locate([V])[0])
        xy = run_xy(key)
        # (the point it was cut at: the first from the dive's end that is no crossing of a cell edge, else the
        # run's far end)
        order = range(len(r['pts']) - 2, -1, -1) if at_end else range(1, len(r['pts']))
        c = next((i for i in order if r['pts'][i]['k'] != 'P'), None)
        j = (0 if at_end else len(xy) - 1) if c is None else c + 1          # ...its place in the run's line
        its = None
        if tV >= 0 and not (c is not None and r['pts'][c]['k'] in ('X', 'D')):     # (a breach's point, or another
            #                                                                         dive's end: the way below)
            its = chord_items(m, xy[j], _cell_after(r, j - 1), V, tV) if at_end else \
                chord_items(m, V, tV, xy[j], _cell_before(r, j - 1))
        if its is not None:
            if at_end:
                r['pts'][j:] = its
                r['last'] = tV
            else:
                r['pts'][:j - 1] = its
                r['first'] = tV
            continue
        its = (chord_items(m, E, d['tri'], V, tV) if at_end else chord_items(m, V, tV, E, d['tri'])) if tV >= 0 else None
        f = {'k': 'F', 'xy': E, 'tri': d['tri']}
        if at_end:
            r['pts'][-1:] = [f] + (its or [])
            r['last'] = tV if its is not None else d['tri']
        else:
            r['pts'][:1] = (its or []) + [f]
            r['first'] = tV if its is not None else d['tri']
    VLAID.discard(vk)


def dive_turned(vk):
    """a laid dive's lane no longer runs its way: the lane comes on to the straight run, and goes on from it, both
    along ONE other router heading -- each read over a straight run's length outside the dive, the router heading
    nearest it. Laid on that heading the lane runs straight through; as it is, it jogs at both ends of the straight
    run. (A lane that comes on along one heading and goes on along another turns AT its via: the dive stays on the
    heading it has.) -> that heading, or None"""
    n, kv = vk
    back, on = run_xy(vk)[:-1][::-1], run_xy((n, kv + 1))[1:]         # the two runs from the straight run's ends, outward
    a, b = _walk(back, DIVE_RUN), _walk(on, DIVE_RUN)
    hs = [max(range(8), key=lambda h: OCT8[h][0] * c[0] + OCT8[h][1] * c[1])
          for c in ((back[0][0] - a[0], back[0][1] - a[1]), (b[0] - on[0][0], b[1] - on[0][1]))]
    return hs[0] if hs[0] == hs[1] != VH[vk] else None


def relay_turned(hot):
    """each laid dive whose lane's way has turned (dive_turned) taken up and laid again, on the heading its lane now
    runs where the via fits with that -- once the layout round it stands still (none of the round's moves, `hot`,
    within the LP's reach of it: its lane's way is then read where it has come to rest, and the rounds that settle
    the rest of the layout settle this too) -> the places changed"""
    places = []
    kd = cKDTree(np.array(hot)) if hot else None
    for vk in sorted(VLAID):
        if RELAID.get(vk, 0) >= FLIPS or (kd is not None and kd.query(VIA[vk], distance_upper_bound=REACH)[0] < math.inf):
            continue
        h = dive_turned(vk)
        if h is not None:
            was = VH[vk]
            RELAID[vk] = RELAID.get(vk, 0) + 1
            unlay_dive(vk)
            LAYTRY.pop(vk, None)
            ok = try_lay(vk, say=True, first=h)
            places.append(VIA[vk])
            log(f'      {vk[0]}\'s via {vk[1]}: its lane\'s way has turned off its heading ({45 * was} deg): '
                + (f'laid again, heading {45 * VH[vk]} deg' if ok else 'taken up'))
    return places


def dive_folds(vk):
    """a laid dive's lane comes on to an end of its straight run from AHEAD of that end -- measured at the lane's
    own scale (a track and a clearance along the run from the end, as the audit reads a turn), the run stands on
    the via's side of the end, along the heading: a fold where lane and straight run meet (a run pulled straight, a
    via slid, and the lane's way through the via is no longer the one the dive was laid on)"""
    n, kv = vk
    u = OCT8[VH[vk]]
    for key, sg in (((n, kv), -1.0), ((n, kv + 1), 1.0)):
        xy = run_xy(key)
        out = xy[:-1][::-1] if sg < 0 else xy[1:]            # the run from the straight run's end outward
        q = _walk(out, TW + CL)
        if sg * (u[0] * (q[0] - out[0][0]) + u[1] * (q[1] - out[0][1])) < 0:
            return True
    return False


LAYTRY = {}            # an un-laid dive -> its two runs' lines as they stood when it was last tried
RELAID = {}            # a dive -> how often it was taken up for a fold and laid again (FLIPS times at most)


def try_lay(vk, say=False, first=None):
    """a pair's dive laid where it can be: its heading read off its lane as it stands -- the nearest of the three
    router headings nearest the lane's way through the via that the via fits with, where it stands (`first`: a
    heading tried before those) -- and its straight run laid on it, unless that makes two runs of a layer cross.
    Tried again only once its lane has moved. -> laid?"""
    n, kv = vk
    kb = (n, kv + 1)
    sig = hash(tuple(round(c_, 4) for k_ in (vk, kb) for q_ in run_xy(k_) for c_ in q_))
    if LAYTRY.get(vk) == sig:
        return False
    LAYTRY[vk] = sig
    was, whys = (VH[vk], TV[vk]), []
    hs = dive_headings(run_xy(vk)[::-1], run_xy(kb))[:3]
    for h in ([first] + [h_ for h_ in hs if h_ != first]) if first is not None else hs:
        t = int(dive_space(h, vk in VXOV, RUN[vk]['L'], RUN[kb]['L']).locate([VIA[vk]])[0])
        if t < 0:
            whys.append(f'{45 * h} deg: its barrels or its straight run do not fit where it stands')
            continue
        VH[vk], TV[vk] = h, t
        keep = {k_: (list(RUN[k_]['pts']), RUN[k_]['first'], RUN[k_]['last']) for k_ in (vk, kb)}
        PL = layout()
        x0 = len(same_layer_crossings(PL))
        why = lay_dive(vk, {k_: LineString(PL[k_]) for k_ in KEYS})
        if not why and len(same_layer_crossings(layout())) > x0:
            for k_, (pts_, f_, l_) in keep.items():
                RUN[k_]['pts'], RUN[k_]['first'], RUN[k_]['last'] = pts_, f_, l_
            VLAID.discard(vk)
            why = 'laid, two runs of a layer cross'
        if not why:
            LAYTRY.pop(vk, None)
            return True
        whys.append(f'{45 * h} deg: {why}')
    VH[vk], TV[vk] = was
    if say:
        log(f'   {n}\'s via {kv} at ({VIA[vk][0]:.2f}, {VIA[vk][1]:.2f}): its straight run not laid -- ' + '; '.join(whys))
    return False


PULLED = {}            # each run's line as it stood when nothing of it was to pull
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
            sig = hash(tuple(round(c_, 4) for q_ in xy for c_ in q_))
            if PULLED.get(key) == sig:               # (looked at as it stands, and nothing of it to pull)
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
                    if any(p['k'] in ('X', 'D') for p in r['pts'][i:j - 1]):     # (never a breach, nor a dive's end)
                        continue
                    hull = shapely.MultiPoint(xy[i:j + 1]).convex_hull
                    if hull.geom_type != 'Polygon':
                        hull = hull.buffer(1e-4)
                    if not in_free(L_, hull):
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
                        for q_ in vtree.query(ch, predicate='dwithin', distance=VREACH):
                            vk = VKEYS[q_]
                            if vk[0] == key[0]:
                                continue
                            if vk in VH:                     # (a pair's via: by its barrels and its cells)
                                s1 = via_short(vk, key[0], ch)
                                if s1 > 0 and s1 > via_short(vk, key[0], old) + 1e-9:
                                    bad = True
                                    break
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
                    # ...and only where the straight line is another way through the cells, or free points go with
                    # it. A stretch that is crossings alone, the straight line through those very edges in that
                    # order, differs from it only in WHERE it crosses each -- and those places are the LP's own
                    # unknowns, put where its rules have them: pulled, the next LP puts them back, every round
                    was = r['pts'][i:j - 1]
                    if all(p['k'] == 'P' for p in was) and \
                            [(p['from'], p['to']) for p in was] == [(p['from'], p['to']) for p in items]:
                        continue
                    found = (i, j, items)
                    break
                if found:
                    break
            if not found:
                PULLED[key] = sig
                break
            i, j, items = found
            if WATCH and key in WATCH:
                hull = shapely.MultiPoint(xy[i:j + 1]).convex_hull
                log(f'         pull {key[0]}.{key[1]} [{i}..{j}] ({xy[i][0]:.2f},{xy[i][1]:.2f}) -> ({xy[j][0]:.2f},{xy[j][1]:.2f}), '
                    f'{len(items)} edge(s); vias in its hull: {[VKEYS[q_] for q_ in vtree.query(hull, predicate="intersects")]}; '
                    f'runs: {sorted({segs[L_][q_][0] for q_ in trees[L_].query(hull, predicate="intersects")})}')
            r['pts'][i:j - 1] = items
            places += [xy[i], xy[j]]
            if '--show-why' in OPT:
                log(f'      pulled {key[0]}.{key[1]} [{i}..{j}] of {n}: ' + ' '.join(f'({q_[0]:.3f},{q_[1]:.3f})' for q_ in xy[i:j + 1]) + f' -> {len(items)} edge(s)')
    return places


SLID = set()         # the vias slid along their lanes: the braid's sides of them no longer read
SLIDE_MAX = 12 * P_MIN        # how far along its lane a via may be slid (twelve pitches)


def _open(q):
    """how OPEN the board is at q: its distance to the nearest bar of fixed copper, the least over the layers"""
    pq = Point(q)
    out = math.inf
    for L_ in range(NL):
        if BARS[L_] is not None:
            out = min(out, float(pq.distance(BARS[L_].geometries[BARS[L_].nearest(pq)])))
    return out


def slide_vias(short):
    """each via of `short` -- one the rounds leave short of its room, or inside a bar -- SLID ALONG ITS OWN LANE to
    the most open place within reach: the stretch of its lane between the old place and the new one changes layer
    with it, which it may wherever that stretch lies in the other layer's free space and crosses no run of that
    layer (the order of the lanes that share a layer stands; lanes of other layers it passes under or over). The
    LP cannot make this move: inside a corridor too narrow for a via and the lane that must pass it, no small step
    along the corridor gains anything. -> the places changed"""
    PL = layout()
    segs = [[] for _ in range(NL)]
    for key in KEYS:
        pts = PL[key]
        for i in range(len(pts) - 1):
            if pts[i] != pts[i + 1]:
                segs[RUN[key]['L']].append((key, LineString([pts[i], pts[i + 1]])))
    trees = [shapely.STRtree([g_ for _, g_ in sg]) if sg else None for sg in segs]
    places = []
    # (a pair's laid dive is taken up for the slide: put back as it was where its via does not slide, laid again
    # -- its heading read off its lane where it now stands -- where it does)
    relay, slid = {}, set()
    for vk in short:
        if vk in VLAID:
            ks_ = (vk, (vk[0], vk[1] + 1))
            was_ = {k_: (list(RUN[k_]['pts']), RUN[k_]['first'], RUN[k_]['last']) for k_ in ks_}
            unlay_dive(vk)
            relay[vk] = (was_, {k_: list(RUN[k_]['pts']) for k_ in ks_})
    for vk in short:
        n, kv = vk
        ka, kb = (n, kv), (n, kv + 1)
        ra, rb = RUN[ka], RUN[kb]
        V = VIA[vk]
        here = _open(V)
        best = None
        for fwd in (True, False):
            mine, other = (kb, ka) if fwd else (ka, kb)          # the run the via slides along; the one that grows
            xy = run_xy(mine) if fwd else run_xy(mine)[::-1]
            Lo = RUN[other]['L']
            items = RUN[mine]['pts'] if fwd else RUN[mine]['pts'][::-1]
            way, S = 0.0, [V]
            for j in range(len(xy) - 1):
                if j < len(items) and items[j]['k'] in ('X', 'D'):      # (never past a breach, nor another dive's end)
                    break
                a, b = xy[j], xy[j + 1]
                ln = math.hypot(b[0] - a[0], b[1] - a[1])
                nn = max(1, int(math.ceil(ln / (P_MIN / 2))))
                stop = False
                for i_ in range(1, nn + 1):
                    if j == len(xy) - 2 and i_ == nn:
                        stop = True                             # (never onto the run's other end)
                        break
                    q = (a[0] + (b[0] - a[0]) * i_ / nn, a[1] + (b[1] - a[1]) * i_ / nn)
                    if way + ln * i_ / nn > SLIDE_MAX:
                        stop = True
                        break
                    Sq = S + [q]
                    line = LineString(Sq)
                    if not in_free(Lo, line):
                        stop = True
                        break
                    if trees[Lo] is not None and any(segs[Lo][q_][0] != other
                                                     for q_ in trees[Lo].query(line, predicate='intersects')):
                        stop = True
                        break
                    hq = None
                    if vk in VH:                # (a pair's via: where it fits with a heading near its lane's way there)
                        w_ = (b[0] - a[0], b[1] - a[1]) if fwd else (a[0] - b[0], a[1] - b[1])
                        hq = next((h_ for h_ in sorted(range(8), key=lambda h_: -(OCT8[h_][0] * w_[0] + OCT8[h_][1] * w_[1]))[:3]
                                   if dive_space(h_, vk in VXOV, ra['L'], rb['L']).locate([q])[0] >= 0), None)
                    if hq is not None if vk in VH else VMESH.locate([q])[0] >= 0:
                        o_ = _open(q)
                        if best is None or o_ > best[0]:
                            best = (o_, fwd, j, q, list(Sq), hq)
                if stop:
                    break
                way += ln
                S.append(b)
        if best is None or best[0] < here + P_MIN / 2:
            continue
        o_, fwd, j, q, Sq, hq = best
        mine, other = (kb, ka) if fwd else (ka, kb)
        rm, ro = RUN[mine], RUN[other]
        mo = MESH[ro['L']]
        # the growing run's new points, from the old place to the new one (all found before anything is changed)
        cells = [int(c_) for c_ in mo.locate(np.array(Sq[1:]))]
        if min(cells) < 0:
            continue
        new, prev, cprev, ok = [], Sq[0], (ro['last'] if fwd else ro['first']), True
        for p_, c_ in zip(Sq[1:], cells):
            its = chord_items(mo, prev, cprev, p_, c_)
            if its is None:
                ok = False
                break
            new += its
            if p_ is not Sq[-1]:
                new.append({'k': 'F', 'xy': (float(p_[0]), float(p_[1])), 'tri': c_})
            prev, cprev = p_, c_
        if not ok:
            continue
        if fwd:                                     # run b loses its first j points; run a gains the stretch
            cb = _cell_after(rm, j - 1)
            del rm['pts'][:j]
            rm['first'] = cb
            ro['pts'] += new
            ro['last'] = cprev
        else:                                       # run a loses its last j points; run b gains the stretch, turned
            nn_ = len(rm['pts'])
            ca = _cell_before(rm, nn_ - j)
            del rm['pts'][nn_ - j:]
            rm['last'] = ca
            back = []
            for it in reversed(new):
                if it['k'] == 'P':
                    it = dict(it, a=it['b'], b=it['a'], t=it['len'] - it['t'], raw=it['len'] - it['t'])
                    it['from'], it['to'] = it['to'], it['from']
                back.append(it)
            ro['pts'] = back + ro['pts']
            ro['first'] = cprev
        VIA[vk] = (float(q[0]), float(q[1]))
        if vk in VH:
            VH[vk] = hq
        TV[vk] = int(vmesh(vk).locate([q])[0])
        SLID.add(vk)
        slid.add(vk)
        VTR.pop(vk, None); VHIST.pop(vk, None); VHOPS.pop(vk, None)
        places += [V, VIA[vk]]
        log(f'   {n}\'s via {kv} slid {"on" if fwd else "back"} along its lane from ({V[0]:.2f}, {V[1]:.2f}) to '
            f'({q[0]:.2f}, {q[1]:.2f}): {here:.3f} -> {o_:.3f} off the nearest fixed copper\'s bar')
    for vk, (was_, mid_) in relay.items():
        if vk not in slid and all(RUN[k_]['pts'] == mid_[k_] for k_ in mid_):
            for k_, (pts_, f_, l_) in was_.items():
                RUN[k_]['pts'], RUN[k_]['first'], RUN[k_]['last'] = pts_, f_, l_
            VLAID.add(vk)
        else:
            LAYTRY.pop(vk, None)
            try_lay(vk, say=True)
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
    lp.kind = 'vcell'
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
        # (a pair's via stands in a cell of its own space; where its dive is laid, its two runs end at the ends of
        # its straight run, each the via's place and a fixed offset: that end stands inside the run's end cell)
        ea_, eb_ = dive_ends(vk)
        cells = [('v', vmesh(vk), TV[vk], (0.0, 0.0)), ('a', MESH[RUN[(n, kv)]['L']], RUN[(n, kv)]['last'], ea_),
                 ('b', MESH[RUN[(n, kv + 1)]['L']], RUN[(n, kv + 1)]['first'], eb_)]
        vrows[vk] = {}
        for tag, m, t, (ox_, oy_) in cells:
            # (hard: a via stands inside its cells, or no further out of one than it stands now -- every other rule
            # gives before a via goes into fixed copper)
            rws = [(nx, ny, c - (nx * ox_ + ny * oy_)) for (nx, ny, c) in m.rows_inside(t)]
            out_ = max([0.0] + [c - 1e-7 - (nx * x + ny * y) for (nx, ny, c) in rws])
            sl = lp.paying('vcell', vk, W_HARD, out_ + 1e-9)
            vrows[vk][tag] = [lp.ge([(vx, nx), (vy, ny)], c - 1e-7, sl) for (nx, ny, c) in rws]
    aff, nlive = {}, 0
    for key in KEYS:
        lp.kind = 'pt'
        n, k = key
        r = RUN[key]
        m = MESH[r['L']]
        cp, lm = cur[key], live[key]
        pts = [vaff[(n, k - 1)] if k else (cp[0][0], cp[0][1], [])]
        for j, p in enumerate(r['pts'], start=1):
            x, y = cp[j]
            p['v'] = None
            if p['k'] == 'D':                                 # a dive's end: its via's place and a fixed offset
                va_ = vaff[p['vk']]
                pts.append((va_[0] + p['o'][0], va_[1] + p['o'][1], va_[2]))
                continue
            if p['k'] == 'X' or not lm[j]:                    # a breach's point, or one out of the round's reach: fixed
                pts.append((x, y, []))
                continue
            nlive += 1
            tr_ = p.get('tr', TRUST)
            if p['k'] == 'P':
                A, B = m.V[p['a']], m.V[p['b']]
                p['v'] = lp.var(0.0, max(0.0, p['t'] - tr_), min(p['len'], p['t'] + tr_))
                pts.append((float(A[0]), float(A[1]), [(p['v'], float((B[0] - A[0]) / p['len']), float((B[1] - A[1]) / p['len']))]))
                # (the edge's two ends are corners of the fixed copper's bars: a pair keeps its half width off them,
                # every lane its slack -- but not off a corner of its OWN stubs' bar: its run begins on that bar's
                # line, nearer those corners than either, and held off them it left its mouth sideways and came
                # back -- a hook the pull pass straightened and the next LP made again, every round)
                own_ = (own_corner(n, r['L'], p['a']), own_corner(n, r['L'], p['b']))
                lo, hi = (0.0 if own_[0] else hw[n]), p['len'] - (0.0 if own_[1] else hw[n])
                if hi < lo:
                    lo = hi = p['len'] / 2
                if hw[n] > 0:                                 # a pair's half width off the edge's two ends
                    sl = lp.paying('edge', key, W_HARD)
                    lp.ge([(p['v'], 1.0)], lo, sl)
                    lp.le([(p['v'], 1.0)], hi, sl)
                for (sgn, ref_, mine) in ((1.0, lo, own_[0]), (-1.0, -hi, own_[1])):  # its slack off them
                    if not mine and sgn * p['t'] - tr_ < ref_ + gs:
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
                # (a wall: an edge with fixed copper's bar beyond it -- but for the line of its own stubs' bar)
                if m.N[p['tri'], i] < 0 and abs(g) > 1e-9 and not all(own_corner(n, r['L'], v_) for v_ in m.edge(p['tri'], i)):
                    walls.append((g, h))
            if tlo > thi:
                lo = hi = 0.0
                elo = ehi = None
            else:
                lo, hi = min(max(tlo, -tr_), tr_), min(max(thi, -tr_), tr_)
                elo, ehi = (elo if tlo > -tr_ else None), (ehi if thi < tr_ else None)
            # (a point that stands OUTSIDE its cell is not forced back by its bounds: every capped rule is capped at
            # the layout as it stands, and a forced move can leave the round's LP no answer. It may stay where it
            # is, and pays the cell's own hard price for it -- so it comes back as the other rules let it)
            back = None
            if lo > 0.0:
                back, lo = (1.0, lo), 0.0
            elif hi < 0.0:
                back, hi = (-1.0, -hi), 0.0
            p['v'] = lp.var(0.0, lo, hi)
            p['n'], p['lim'] = (nx_, ny_), (elo, ehi)
            pts.append((x, y, [(p['v'], nx_, ny_)]))
            if back is not None:
                lp.ge([(p['v'], back[0])], back[1], lp.paying('cell', key, W_HARD))
            sl = None
            for (g, h) in walls:                              # its slack off the fixed copper's bar (a wall of the cell)
                b_ = (h + gs) / g
                if (g > 0 and b_ > lo) or (g < 0 and b_ < hi):
                    if sl is None:
                        sl = lp.paying('slack-copper', key, W_SLACK)
                    lp.ge([(p['v'], g)], h + gs, sl)
        pts.append(vaff[(n, k)] if (n, k) in VIA else (cp[-1][0], cp[-1][1], []))
        aff[key] = pts
        lp.kind = 'len'
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
    # ---- a lane leaves its stub's mouth, and comes to it, on the stub's own side of it: its points within the
    # lane's own scale of the mouth stand ahead of it along the stub's way (no fold where lane and stub meet)
    lp.kind = 'join'
    for key in KEYS:
        r = RUN[key]
        for e_, lead in ((0, r.get('lead') if key[1] == 0 else None),
                         (1, r.get('tail') if key[1] == len(RUNL[key[0]]) - 1 else None)):
            if not lead:
                continue
            cp = cur[key]
            mth = cp[0] if e_ == 0 else cp[-1]
            ul = math.hypot(mth[0] - lead[0], mth[1] - lead[1])
            if ul < 1e-9:
                continue
            u = ((mth[0] - lead[0]) / ul, (mth[1] - lead[1]) / ul)
            idx = range(1, len(cp) - 1) if e_ == 0 else range(len(cp) - 2, 0, -1)
            way, prev = 0.0, mth
            for j in idx:
                way += math.hypot(cp[j][0] - prev[0], cp[j][1] - prev[1])
                prev = cp[j]
                if way > 2 * (TW + CL):
                    break
                if not aff[key][j][2]:
                    continue
                c_, t_ = P_dot(u, aff[key][j])
                now = u[0] * (cp[j][0] - mth[0]) + u[1] * (cp[j][1] - mth[1])
                lp.ge(t_, (u[0] * mth[0] + u[1] * mth[1]) - c_, lp.paying('join', key, W_HARD, max(0.0, -now) + 1e-9))
    # ...and likewise where a lane meets the end of a laid dive's straight run: its points within that scale of the
    # end stand beyond it, along the dive's heading away from the via
    for vk in sorted(VLAID):
        for key, sg in ((vk, -1.0), ((vk[0], vk[1] + 1), 1.0)):
            cp = cur[key]
            ie = len(cp) - 2 if sg < 0 else 1
            u = (sg * OCT8[VH[vk]][0], sg * OCT8[VH[vk]][1])
            way, prev = 0.0, cp[ie]
            for j in (range(ie - 1, 0, -1) if sg < 0 else range(ie + 1, len(cp) - 1)):
                way += math.hypot(cp[j][0] - prev[0], cp[j][1] - prev[1])
                prev = cp[j]
                if way > 2 * (TW + CL):
                    break
                if aff[key][j][2] or aff[key][ie][2]:
                    now = u[0] * (cp[j][0] - cp[ie][0]) + u[1] * (cp[j][1] - cp[ie][1])
                    sep(lp, u, aff[key][ie], aff[key][j], 0.0, lp.paying('join', key, W_HARD, max(0.0, -now) + 1e-9))
    # ---- the price of a bend: each point of a lane off the straight line through the points before and after it
    # (across the lane; a via both ways)
    lp.kind = 'bend'
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
    lp.kind = 'order'
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
    lp.kind = 'lane'
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
            lp.kind = f'lane:{ka[0]}.{ka[1]}[{j}]~{kb[0]}.{kb[1]}[{i}] d{dd:.3f}'
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
        for ib in tree.query(Point(V), predicate='dwithin',
                             distance=slk(VREACH if vk in VH else VIA_R + PP + VXO[a_]) + 3 * TRUST):
            key, j, sg = allseg[ib]
            if key[0] == a_ or not (vaff[vk][2] or aff[key][j][2] or aff[key][j + 1][2]):
                continue
            flip = False
            P_, Q_ = cur[key][j], cur[key][j + 1]
            hx, hy = Q_[0] - P_[0], Q_[1] - P_[1]
            hl = math.hypot(hx, hy)
            nrm = (-hy / hl, hx / hl)
            w = ((V[0] - P_[0]) * hx + (V[1] - P_[1]) * hy) / hl
            C_ = P_ if w <= 0.0 else Q_ if w >= hl else (P_[0] + hx * w / hl, P_[1] + hy * w / hl)
            dl = math.hypot(C_[0] - V[0], C_[1] - V[1])
            nv = ((C_[0] - V[0]) / dl, (C_[1] - V[1]) / dl) if dl > 1e-6 else None
            if 0.0 < w < hl and hl >= 2 * GS:      # beside it: the braid says which side the lane passes
                side = 0 if vk in SLID else via_bit(a_, vk[1], key[0], key[1])
                if side:
                    said += 1
                    geo = 1 if (nrm[0] * (P_[0] - V[0]) + nrm[1] * (P_[1] - V[1])) > 0 else -1
                    if side != geo or nv is None:
                        wrong += 1
                        flip = True
                        nv = (nrm[0] * side, nrm[1] * side)
            if nv is None:
                nv = nrm
            # (a single's via is one disc, the via; a pair's its barrels and its cells, each held its own bar off
            # the lane along its own line to it -- and every one on the side of the lane the via's place stands:
            # the lane passes the whole of it on one side)
            for (ox_, oy_), kd_ in via_discs(vk):
                room = disc_room(vk, kd_, key[0])
                if kd_ == 's':
                    D_, nd_, va_ = V, nv, vaff[vk]
                else:
                    D_ = (V[0] + ox_, V[1] + oy_)
                    wd = ((D_[0] - P_[0]) * hx + (D_[1] - P_[1]) * hy) / hl
                    Cd = P_ if wd <= 0.0 else Q_ if wd >= hl else (P_[0] + hx * wd / hl, P_[1] + hy * wd / hl)
                    dd_ = math.hypot(Cd[0] - D_[0], Cd[1] - D_[1])
                    if dd_ > slk(room) + 3 * TRUST:
                        continue
                    gv_ = 1.0 if (nrm[0] * (P_[0] - V[0]) + nrm[1] * (P_[1] - V[1])) > 0 else -1.0
                    gd_ = 1.0 if (nrm[0] * (P_[0] - D_[0]) + nrm[1] * (P_[1] - D_[1])) > 0 else -1.0
                    if flip:
                        nd_ = nv
                    elif dd_ <= 1e-6 or (0.0 < wd < hl and gd_ != gv_):
                        nd_ = (nrm[0] * gv_, nrm[1] * gv_)
                    else:
                        nd_ = ((Cd[0] - D_[0]) / dd_, (Cd[1] - D_[1]) / dd_)
                    va_ = (vaff[vk][0] + ox_, vaff[vk][1] + oy_, vaff[vk][2])
                now = min(nd_[0] * (q[0] - D_[0]) + nd_[1] * (q[1] - D_[1]) for q in (P_, Q_))
                sls = [lp.paying('slack-via', (vk, key), W_SLACK, slk(room) - room),
                       lp.paying('via', (vk, key), W_HARD, room - min(SIDE, now))]
                lp.kind = f'via:{vk[0]}.{vk[1]}{kd_}~{key[0]}.{key[1]}[{j}]'
                for q in (aff[key][j], aff[key][j + 1]):
                    sep(lp, nd_, va_, q, slk(room), sls)
    # ---- rows: two vias their own distance apart, along the line between them as they stand
    lp.kind = 'viavia'
    for i, v1 in enumerate(VKEYS):
        for v2 in VKEYS[i + 1:]:
            if not (vlive[v1] or vlive[v2]):
                continue
            if math.hypot(VIA[v2][0] - VIA[v1][0], VIA[v2][1] - VIA[v1][1]) > 2 * VREACH + 4 * TRUST:
                continue
            # (two singles' vias: the one distance between them; a pair's barrels and cells each their own)
            for o1, o2, need in via_rooms(v1, v2):
                dx, dy = VIA[v2][0] + o2[0] - VIA[v1][0] - o1[0], VIA[v2][1] + o2[1] - VIA[v1][1] - o1[1]
                dl = math.hypot(dx, dy)
                if dl > slk(need) + 4 * TRUST:
                    continue
                nv = (dx / dl, dy / dl) if dl > 1e-9 else (1.0, 0.0)
                sep(lp, nv, (vaff[v1][0] + o1[0], vaff[v1][1] + o1[1], vaff[v1][2]),
                    (vaff[v2][0] + o2[0], vaff[v2][1] + o2[1], vaff[v2][2]), slk(need),
                    [lp.paying('slack-via', (v1, v2), W_SLACK, slk(need) - need),
                     lp.paying('viavia', (v1, v2), W_HARD, need - min(SIDE, dl))])
    # ---- rows: a pair's via stands off its lane's two ends by the pair's end run and two straight runs: nearer, the
    # polish joins the dive to the end run and moves the via onto that line (a via never nearer than it stands)
    lp.kind = 'ends'
    for vk, (vx, vy) in vvar.items():
        for T_, need in (DIVE_END.get(vk[0], ()) if vk in VH else ()):
            dx, dy = VIA[vk][0] - T_[0], VIA[vk][1] - T_[1]
            dl = math.hypot(dx, dy)
            if 1e-9 < dl < need + 3 * TRUST:
                sep(lp, (dx / dl, dy / dl), (T_[0], T_[1], []), vaff[vk], need,
                    lp.paying('ends', vk, W_HARD, max(0.0, need - dl) + 1e-9))
    # ---- a via's slack off the walls of the vias' space
    lp.kind = 'vslack'
    for vk, (vx, vy) in vvar.items():
        sl = None
        vm_ = vmesh(vk)
        for i, (nx, ny, c) in enumerate(vm_.rows_inside(TV[vk])):
            if vm_.N[TV[vk], i] < 0:
                sl = sl if sl is not None else lp.paying('slack-copper', vk, W_SLACK)
                lp.ge([(vx, nx), (vy, ny)], c + gs, sl)
    t1 = time.time()
    if not lp.solve():                              # (no answer at all: nothing moves, and the rounds end here)
        HOT = []
        return {}, 0.0, 0
    t2_ = time.time()
    sol = lp.x
    if WATCH and '--show-rows' in OPT:
        Acsc = coo_matrix((lp.A, (lp.I, lp.Jc)), shape=(len(lp.b), len(lp.c))).tocsc()
        for key in WATCH:
            for idx, p in enumerate(RUN[key]['pts']):
                if p.get('v') is None:
                    continue
                col_ = Acsc.getcol(p['v'])
                got = collections.defaultdict(float)
                names = []
                for r_, a_ in zip(col_.indices, col_.data):
                    if abs(lp.marg[r_]) > 1e-7:
                        got[lp.rk[r_].split(':')[0]] += lp.marg[r_] * a_
                        if ':' in lp.rk[r_]:
                            names.append(f'{lp.rk[r_]}({lp.marg[r_] * a_:+.2f})')
                d_ = float(sol[p['v']]) - (p['t'] if p['k'] == 'P' else 0.0)
                log(f'         rows {key[0]}.{key[1]}[{idx}]{p["k"]} at ({cur[key][idx + 1][0]:.3f},{cur[key][idx + 1][1]:.3f}) d {d_:+.4f}: '
                    + ', '.join(f'{k_} {v_:+.2f}' for k_, v_ in sorted(got.items())) + ' | ' + ' '.join(names[:6]))
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
    WHY = []
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
                WHY.append(('lp', key, idx, p['k'], q, round(mv, 4), round(p.get('tr', TRUST), 4)))
        # such a point goes on into the cell beyond: the run crosses that edge AT the point, and back (a lane pushed
        # aside bulges into the next cell). Where the run's own crossing of that edge is the point beside it, the two
        # crossings between them -- back, and over again -- are dropped just below, and the run crosses where the
        # point went through. (The crossing beside it was once moved past the point where it stood: with the
        # crossing far along the edge the run then went out to it and back, the pull pass straightened that, and
        # the next round pressed the same points on the same edge again -- every round)
        for idx, t2 in reversed(over):
            p = r['pts'][idx]
            t = p['tri']
            r['pts'][idx:idx + 1] = [portal(m, t, t2, p['xy']), p, portal(m, t2, t, p['xy'])]
            c = m.cen[t2]
            p['tri'] = t2
            p['xy'] = (float(p['xy'][0] + (c[0] - p['xy'][0]) * 1e-4), float(p['xy'][1] + (c[1] - p['xy'][1]) * 1e-4))
            hot.append(p['xy'])
            WHY.append(('cell', key, idx, 'F', p['xy'], 0.0, p.get('flips', 0)))
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
        t2 = pressed('v', vmesh(vk), TV[vk])
        if t2 is not None:
            TV[vk] = t2
            hops += 1
        # (a laid dive: each run's end is its straight run's -- the run's last point, or its first, the crossing
        # beside it)
        la_ = 1 if vk in VLAID else 0
        ea_, eb_ = dive_ends(vk)
        ra, rb = RUN[(n, kv)], RUN[(n, kv + 1)]
        ma, mb = MESH[ra['L']], MESH[rb['L']]
        t2 = pressed('a', ma, ra['last'])
        # (the run crosses that edge AT the via, one crossing more -- or, where its crossing of that very edge is
        # the point next to the via, that one less. Points of the run between the via and a crossing farther along
        # stay: the lane's line does not move, only the cell its end is in)
        if t2 is not None:                          # the run that ends at it
            k_ = len(ra['pts']) - la_
            e_ = ra['pts'][k_ - 1] if k_ > 0 else None
            if e_ is not None and e_['k'] == 'P' and e_['from'] == t2 and e_['to'] == ra['last']:
                del ra['pts'][k_ - 1]
            else:
                ra['pts'].insert(k_, portal(ma, ra['last'], t2, (q[0] + ea_[0], q[1] + ea_[1])))
            ra['last'] = t2
            if la_:
                ra['pts'][-1]['tri'] = t2
            hops += 1
        t2 = pressed('b', mb, rb['first'])
        if t2 is not None:                          # the run that starts at it
            e_ = rb['pts'][la_] if len(rb['pts']) > la_ else None
            if e_ is not None and e_['k'] == 'P' and e_['from'] == rb['first'] and e_['to'] == t2:
                del rb['pts'][la_]
            else:
                rb['pts'].insert(la_, portal(mb, t2, rb['first'], (q[0] + eb_[0], q[1] + eb_[1])))
            rb['first'] = t2
            if la_:
                rb['pts'][0]['tri'] = t2
            hops += 1
        if mv > TOL or hops > h0:
            hot.append(q)
            WHY.append(('via' if hops == h0 else 'viahop', vk, 0, 'V', q, round(mv, 4), tuple(round(t__, 4) for t__ in VTR[vk])))
    npull = pull_runs()
    hot += npull
    WHY.extend(('pull', None, 0, '-', q_, 0.0, 0.0) for q_ in npull)
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
            if d > 1.6 * HPT and not jump and not (ip == 0 and p['k'] == 'D'):       # (none inside a dive's run)
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
        if d > 1.6 * HPT and not (old and old[-1]['k'] == 'D'):
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
    if '--show-why' in OPT:
        for w_ in WHY:
            log(f'      why {w_[0]:6s} {(w_[1][0] + "." + str(w_[1][1])) if w_[1] else "":12s} [{w_[2]}]{w_[3]} at ({w_[4][0]:.3f}, {w_[4][1]:.3f}) moved {w_[5]:.4f} tr {w_[6]}')
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


# ---------------------------------------------------------------- the pairs' dives laid
# each pair's via given its straight run where it can be (one that cannot yet -- its lane turns at it, it stands too
# near its lane's end -- is tried again after each round, once its lane has moved), and how far it stands off its
# lane's two ends at least: the pair's end run (pairs.end_run: the polish lays and holds it) and two straight runs
# -- nearer, the polish joins the dive to the end run and moves the via onto that line
DIVE_END = {}
for vk in sorted(VH):
    n_ = vk[0]
    try_lay(vk, say=True)
    if n_ not in DIVE_END and n_ in getattr(ctx, 'pair_ends', {}):
        st_ = 2 * max(dive_shape(1, (n_, 0) in VXOV)['runs'])
        e0_, e1_ = RUN[(n_, 0)], RUN[(n_, len(RUNL[n_]) - 1)]
        DIVE_END[n_] = [(e0_.get('lead') or e0_['ref'][0], _pairs.end_run(_cfg, ctx.pair_ends[n_][0]) + st_),
                        (e1_.get('tail') or e1_['ref'][-1], _pairs.end_run(_cfg, ctx.pair_ends[n_][1]) + st_)]
if VH:
    log(f'pairs\' vias: {len(VH)}, each two barrels {OFF:.3f} either side of its line and a straight run of '
        f'{DIVE_RUN:.3f} either side of it ({len(VXOV)} a crossover); {len(VLAID)} laid straight: '
        + ', '.join(f'{vk[0]}.{vk[1]} {45 * VH[vk]} deg' + ('' if vk in VLAID else ' NOT LAID')
                    + (f', moved {math.hypot(VIA[vk][0] - VIA_REF[vk][0], VIA[vk][1] - VIA_REF[vk][1]):.3f}'
                       if VIA[vk] != VIA_REF[vk] else '') for vk in sorted(VH)))
kinks('in the layout it starts from')
if '--watch' in OPT:
    watch(OPT['--watch'], layout())
it = 0
for phase in range(3):
    for _ in range(ROUNDS if phase == 0 else ROUNDS // 2):
        it += 1
        paid, move, hops = round_of(it)
        for vk in sorted(VLAID):                  # (a laid dive its lane now folds at: taken up, its heading read again)
            if dive_folds(vk) and RELAID.get(vk, 0) < FLIPS:
                RELAID[vk] = RELAID.get(vk, 0) + 1
                unlay_dive(vk)
                LAYTRY.pop(vk, None)
                HOT.append(VIA[vk])
                log(f'      {vk[0]}\'s via {vk[1]}: its lane folds at an end of its straight run (heading {45 * VH[vk]} deg): taken up')
        HOT += relay_turned(HOT)                  # (...one its lane's way has turned off, where the layout round it stands still)
        for vk in sorted(VH):                     # (a pair's dive not yet laid: again, where its lane has moved)
            if vk not in VLAID and try_lay(vk, say='--show-why' in OPT):
                HOT.append(VIA[vk])
                log(f'      {vk[0]}\'s via {vk[1]}: its straight run laid, heading {45 * VH[vk]} deg')
        if not HOT:                               # nothing moved, no cell changed: the layout has settled
            break
    # the vias the rounds leave short of their room, or inside a bar: each slid along its lane to where the board is
    # most open, and the rounds again from there
    sf_ = shortfalls(layout())
    short_ = sorted({w_[0] for v_, w_ in sf_.get('via', []) if v_ > GRID2} |
                    {vk for vk in VKEYS if vmesh(vk).locate([VIA[vk]])[0] < 0})
    slid_ = slide_vias([vk for vk in short_ if vk not in SLID]) if phase < 2 else []
    if not slid_:
        break
    HOT = (HOT or []) + slid_
log(f'whole_embed: {it} round(s)' + ('' if not HOT else ' -- NOT settled') + f', {len(SLID)} via(s) slid along '
    f'their lanes, {time.time() - T0:.1f} s')
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


W_SHAPE = ctx.cfg.track_width + ctx.cfg.clearance      # the lane's own scale, as the audit reads a turn over it
NOTCH = ctx.cfg.track_width + ctx.cfg.grid_step        # ...and a step or a dip shorter than this a notch


def _deg(a, b, c):
    h1, h2 = math.atan2(b[1] - a[1], b[0] - a[0]), math.atan2(c[1] - b[1], c[0] - b[0])
    return math.degrees((h2 - h1 + math.pi) % (2 * math.pi) - math.pi)


def shape_faults(pts):
    """the points of a written line (their indices) at a shape no router lays, by the audit's own measures
    (plan_audit.check_shape): a FOLD -- a turn of more than 100 degrees at a point, or over the lane's scale either
    side of it -- and a NOTCH, two opposite turns of 60 degrees or more nearer than a track and a grid step"""
    keep = [0]                                      # (as the audit reads a line: its collinear points merged)
    for i in range(1, len(pts) - 1):
        a, b, c = pts[keep[-1]], pts[i], pts[i + 1]
        if abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) < 1e-7 and \
                (b[0] - a[0]) * (c[0] - b[0]) + (b[1] - a[1]) * (c[1] - b[1]) > 0:
            continue
        keep.append(i)
    keep.append(len(pts) - 1)
    q = [pts[i] for i in keep]
    n = len(q)
    if n < 3:
        return []
    t = [0.0] + [_deg(q[i - 1], q[i], q[i + 1]) for i in range(1, n - 1)] + [0.0]
    bad = {i for i in range(1, n - 1) if abs(t[i]) > 100}
    for i in range(1, n - 2):
        if abs(t[i]) > 60 and abs(t[i + 1]) > 60 and t[i] * t[i + 1] < 0 and \
                math.hypot(q[i + 1][0] - q[i][0], q[i + 1][1] - q[i][1]) < NOTCH:
            bad |= {i, i + 1}
    sm = [0.0]
    for a, b in zip(q, q[1:]):
        sm.append(sm[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))

    def at(u):
        u = min(max(u, 0.0), sm[-1])
        k = 1
        while k < n - 1 and sm[k] < u:
            k += 1
        f = 0.0 if sm[k] - sm[k - 1] < 1e-12 else (u - sm[k - 1]) / (sm[k] - sm[k - 1])
        return (q[k - 1][0] + (q[k][0] - q[k - 1][0]) * f, q[k - 1][1] + (q[k][1] - q[k - 1][1]) * f)
    for i in range(1, n - 1):
        a, c = at(sm[i] - W_SHAPE), at(sm[i] + W_SHAPE)
        if math.hypot(q[i][0] - a[0], q[i][1] - a[1]) >= 1e-3 and math.hypot(c[0] - q[i][0], c[1] - q[i][1]) >= 1e-3 \
                and abs(_deg(a, q[i], c)) > 100:
            bad.add(i)
    return sorted(keep[i] for i in bad)


def shape_pass(OWN):
    """the written lines cleared of folds and notches where that is SAFE: a point at one is left out -- the line
    goes straight between its neighbours -- where the triangle the three make holds no fixed copper, no other run
    of the layer and no via, and the straight line stands no nearer anything than its bar, or than the line it
    replaces did. -> the points left out, the faults that stay"""
    lines = {key: LineString(OWN[key]) for key in KEYS}
    byL = [[key for key in KEYS if RUN[key]['L'] == L_] for L_ in range(NL)]
    trees = [shapely.STRtree([lines[k_] for k_ in ks]) if ks else None for ks in byL]
    vpts = [Point(VIA[vk]) for vk in VKEYS]
    vtree = shapely.STRtree(vpts) if vpts else None
    hwm = 2 * max(hw.values())
    nout = nleft = 0
    for key in KEYS:
        L_ = RUN[key]['L']
        pts = OWN[key]
        # (a stub's mouth stays, and the end of a laid dive's straight run)
        lo = 2 if ((RUN[key].get('lead') and key[1] == 0) or (key[0], key[1] - 1) in VLAID) else 1
        back = 2 if ((RUN[key].get('tail') and key[1] == len(RUNL[key[0]]) - 1) or key in VLAID) else 1
        for _ in range(80):
            bad = shape_faults(pts)
            if not bad:
                break
            done = False
            for i in sorted({j for b_ in bad for j in (b_, b_ - 1, b_ + 1)}, key=lambda j: (j not in bad, j)):
                if not (lo <= i <= len(pts) - 1 - back):
                    continue
                a, v, b = pts[i - 1], pts[i], pts[i + 1]
                ch, old = LineString([a, b]), LineString([a, v, b])
                tri = shapely.Polygon([a, v, b])
                tri = tri if tri.area > 1e-12 else ch
                if not in_free(L_, tri):
                    continue
                ok = True
                for q_ in trees[L_].query(ch, predicate='dwithin', distance=P_MIN + hwm):
                    k2 = byL[L_][q_]
                    if k2 == key:
                        continue
                    d1 = ch.distance(lines[k2])
                    if lines[k2].intersects(tri) or (d1 < P_MIN + hw[key[0]] + hw[k2[0]] and d1 < old.distance(lines[k2]) - 1e-9):
                        ok = False
                        break
                if ok and vtree is not None:
                    for q_ in vtree.query(ch, predicate='dwithin', distance=VREACH):
                        vk = VKEYS[q_]
                        if vk[0] == key[0]:
                            continue
                        if vk in VH:                         # (a pair's via: by its barrels and its cells)
                            s1 = via_short(vk, key[0], ch)
                            if tri.contains(vpts[q_]) or (s1 > 0 and s1 > via_short(vk, key[0], old) + 1e-9):
                                ok = False
                                break
                            continue
                        d1 = ch.distance(vpts[q_])
                        if tri.contains(vpts[q_]) or (d1 < VS[key[0]] + VXO[vk[0]] and d1 < old.distance(vpts[q_]) - 1e-9):
                            ok = False
                            break
                if ok:
                    del pts[i]
                    lines[key] = LineString(pts)
                    nout += 1
                    done = True
                    break
            if not done:
                nleft += len(bad)
                break
    return nout, nleft


PL = layout()
OWN = {}
for key in KEYS:
    own_ = list(PL[key])
    if RUN[key].get('lead') and key[1] == 0:                  # the straight piece from its tooth to its stub's mouth
        own_ = [RUN[key]['lead']] + own_
    if RUN[key].get('tail') and key[1] == len(RUNL[key[0]]) - 1:
        own_ = own_ + [RUN[key]['tail']]
    # (points within 0.4 of a grid step of each other -- a run's places on the edges of sliver cells at a copper
    # corner -- one point: a jog that small is no shape of the lane's)
    # (...but never the end of a laid dive's straight run: the run's second point, or its last but one)
    hold_ = ({1} if (key[0], key[1] - 1) in VLAID else set()) | ({len(own_) - 2} if key in VLAID else set())
    OWN[key] = [own_[0]] + [q_ for i_, q_ in enumerate(own_[1:-1], 1)
                            if i_ in hold_ or math.hypot(q_[0] - own_[i_ - 1][0], q_[1] - own_[i_ - 1][1]) > 0.4 * GS] + [own_[-1]]
_f0 = sum(len(shape_faults(OWN[k_])) for k_ in KEYS)
_nout, _nleft = shape_pass(OWN)
log(f'the written lines: {_f0} point(s) at a fold or notch by the audit\'s measures; {_nout} point(s) left out where '
    f'that is safe, {_nleft} stay')
res = {'lanes': {}, 'vias': [], 'cuts': [], 'lcuts': [], 'vcuts': []}
for n in M:
    xy, pieces = [], []
    for k in range(len(RUNL[n])):
        pts = sampled(OWN[(n, k)])
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
_vout = [vk for vk in VKEYS if vmesh(vk).locate([VIA[vk]])[0] < 0 and Point(VIA[vk]).distance(vmesh(vk).free) > GRID2]
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

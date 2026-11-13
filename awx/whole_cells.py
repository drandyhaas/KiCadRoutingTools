"""whole_cells.py -- each routing layer's FREE SPACE on the board, cut into its PIECES, and what the pieces say about
a bus's lanes before any plan is made.

A layer's free space is where a lane's centreline may stand on it: inside the board, a lane's bar off every piece of
fixed copper on that layer -- another part's pad, another net's track or via, an earlier bus's laid tracks -- as
whole_embed.py reads it. A PIECE is a connected part of it. Two pieces of a layer have no way between them on that
layer, anywhere on the board: a run that starts in one ends in it, wherever its lane's other runs go. That is a fact of
the board alone, known before the solve.

The solve (whole_solve.py) orders the lanes and gives each run a layer along one route coordinate; it does not see
that a layer is shut across a lane's way. walls() reads it off the pieces for each lane and layer and says it in the
solve's own form, a LAYER CUT (whole_route.layer_cut: the lane off that layer across a stretch of its route) --
whole_solve takes them with its own built-in cuts under SOLVE_LAYER_WALLS."""
import math

import numpy as np
import shapely
from shapely.geometry import box as _box, Polygon
from shapely.ops import unary_union

ARRAY = ('src ', 'dest ')              # an array's own box: its teeth and berths stand ON its line


def fixed_copper(ctx, Fr, LNAME, trk, clr, via, pitch):
    """[(x0, y0, x1, y1, layers, label)]: the fixed copper a bus's lanes go round, piece by piece, each on its routing
    layers (indices in LNAME) -- whole_embed's: the census's arrays, vias and tracks (whole_passages.Census), every
    other part's pads EACH ITS OWN piece (a lane passes between two pads wherever the space between them is free), and
    the copper of other nets that ends on an array's box line: a stub's last stretch on its layer, a via stubs end in
    on every layer. Never the bus's own stubs: before the solve a via end's layer is not chosen yet"""
    import board_rules
    import braid as bd
    import pairs as _pairs
    import whole_ctx
    import whole_passages
    M = list(Fr.M)
    NL = len(LNAME)
    LI = {L: i for i, L in enumerate(LNAME)}
    prs = getattr(ctx, 'pairs', {}) or {}
    bus_ids = {ctx.byname[n][0] for n in M} | {ctx.byname[leg][0] for n in M for leg in prs.get(n, ())
                                               if leg in ctx.byname}
    cen = whole_passages.Census(ctx, Fr, LNAME, trk, clr, via, pitch, bus_nets=bus_ids)
    isl = {lab for (lab, _ls) in cen.islands}

    def ringed(o):
        # (another net's via at its reach for a lane: the router's ring about the grid point the via rounds to)
        if not o[5].startswith('via '):
            return o
        cx, cy, r = (o[0] + o[2]) / 2, (o[1] + o[3]) / 2, (o[2] - o[0]) / 2
        r2 = max(r, _pairs.svia_reach(ctx.cfg, cx, cy, 2 * r))
        return (cx - r2, cy - r2, cx + r2, cy + r2) + tuple(o[4:])
    have = {(round((o[0] + o[2]) / 2, 4), round((o[1] + o[3]) / 2, 4)) for o in cen.obs if o[5].startswith('via ')}
    fixed = [ringed(o) for o in cen.obs if o[5] not in isl]
    for ref, fp in ctx.pcb.footprints.items():
        if ref == Fr.src or ref in Fr.dests:
            continue
        for p in fp.pads:
            if p.pad_type == 'np_thru_hole':
                hx = hy = (p.drill or 0) / 2 + board_rules.hole_track_clear(ctx.pcb, clr) - clr
                ls = frozenset(range(NL))
            else:
                hx, hy = _pairs.pad_extent(p.size_x / 2, p.size_y / 2, _pairs.pad_tilt(p), _pairs.pad_corner_radius(p))
                ls = frozenset(range(NL)) if ((p.drill or 0) > 0 or '*.Cu' in p.layers) else \
                    frozenset(LI[L] for L in p.layers if L in LI)
            if ls:
                fixed.append((p.global_x - hx, p.global_y - hy, p.global_x + hx, p.global_y + hy, ls,
                              f'{ref}.{p.pad_number}'))
    boxes = [Fr.SB] + [Fr.DBs[r] for r in Fr.dests]
    near = lambda x, y, g: any((min(abs(x - b[0]), abs(x - b[2])) < g and b[1] - g <= y <= b[3] + g) or
                               (min(abs(y - b[1]), abs(y - b[3])) < g and b[0] - g <= x <= b[2] + g) for b in boxes)
    g2 = ctx.cfg.grid_step / 2
    for s in ctx.base_segments:
        if s.layer not in LI or s.net_id in bus_ids:
            continue
        for (ax, ay), (bx, by) in (((s.start_x, s.start_y), (s.end_x, s.end_y)),
                                   ((s.end_x, s.end_y), (s.start_x, s.start_y))):
            if not near(ax, ay, trk + clr):
                continue
            ln = math.hypot(bx - ax, by - ay)
            f = min(1.0, (trk + clr) / ln) if ln > 1e-9 else 0.0
            cx, cy, r = ax + (bx - ax) * f, ay + (by - ay) * f, s.width / 2
            fixed.append((min(ax, cx) - r, min(ay, cy) - r, max(ax, cx) + r, max(ay, cy) + r,
                          frozenset([LI[s.layer]]), 'stub ' + whole_ctx.copper_name(ctx, s)))
    for v in ctx.base_vias:
        if v.net_id in bus_ids:
            continue
        r = max(v.size / 2, _pairs.svia_reach(ctx.cfg, v.x, v.y, v.size))
        if (round(v.x, 4), round(v.y, 4)) not in have and near(v.x, v.y, via / 2 + clr + g2 + bd.VIA_NEED):
            fixed.append((v.x - r, v.y - r, v.x + r, v.y + r, frozenset(range(NL)),
                          'svia ' + whole_ctx.copper_name(ctx, v)))
    return fixed, cen


class Pieces:
    """a free space cut into its pieces: `geoms` (polygons), each a connected part of it"""

    def __init__(self, free):
        self.geoms = [g for g in getattr(free, 'geoms', [free]) if isinstance(g, Polygon) and not g.is_empty]
        self._tree = shapely.STRtree(self.geoms) if self.geoms else None

    def at(self, xy):
        """the piece each point of xy (n x 2) stands in, -1 inside a bar"""
        xy = np.asarray(xy, float).reshape(-1, 2)
        out = -np.ones(len(xy), int)
        if self._tree is None or not len(xy):
            return out
        qi, gi = self._tree.query(shapely.points(xy), predicate='intersects')
        out[qi] = gi
        return out

    def near(self, xy, reach):
        """the piece nearest the point xy, within `reach` of it; -1 with none"""
        if self._tree is None:
            return -1
        p = shapely.Point(xy)
        i = int(self._tree.nearest(p))
        return i if float(self.geoms[i].distance(p)) <= reach else -1


class Free:
    """each routing layer's free space for a lane's centreline `bar` off the fixed copper (`bar_array` off an array's
    own box), inside the window `win` (x0, y0, x1, y1)"""

    def __init__(self, fixed, NL, win, bar, bar_array):
        self.wb = _box(*win)
        self._bars, self._tree, self._whole = [], [], {}
        for L in range(NL):
            bs = []
            for (x0, y0, x1, y1, ls, lab) in fixed:
                if L not in ls:
                    continue
                g = bar_array if lab.startswith(ARRAY) else bar
                if x1 + g < win[0] or x0 - g > win[2] or y1 + g < win[1] or y0 - g > win[3]:
                    continue
                bs.append(_box(x0 - g, y0 - g, x1 + g, y1 + g))
            self._bars.append(bs)
            self._tree.append(shapely.STRtree(bs) if bs else None)

    def pieces(self, L, region=None):
        """the pieces of layer L's free space inside `region` (the whole window with none)"""
        if region is None:
            if L not in self._whole:
                self._whole[L] = Pieces(self.wb.difference(unary_union(self._bars[L])) if self._bars[L] else self.wb)
            return self._whole[L]
        reg = region.intersection(self.wb)
        if self._tree[L] is None:
            return Pieces(reg)
        hit = [self._bars[L][i] for i in self._tree[L].query(reg, predicate='intersects')]
        return Pieces(reg.difference(unary_union(hit)) if hit else reg)


def mouth(P, pt, d, reach, step):
    """the piece (of the pieces P) a lane's end stands in: the one its stub's MOUTH is in -- the first free point
    straight out of the stub at `pt` along `d` (the way the lane leaves it), within `reach`; a stub with no direction
    of its own (it ends in a via) by the nearest free point. -1: nothing is free in front of the end"""
    dl = math.hypot(*d) if d is not None else 0.0
    if dl < 1e-9:
        return P.near(pt, reach)
    ds = np.arange(0.0, reach + 1e-9, step)
    pc = P.at(np.column_stack((pt[0] + d[0] / dl * ds, pt[1] + d[1] / dl * ds)))
    hit = np.nonzero(pc >= 0)[0]
    return int(pc[hit[0]]) if len(hit) else -1


def walls(ctx, Fr, LNAME, refs, entry, end, ends, trk, clr, via, pitch, band=None, log=None):
    """the layer cuts the pieces give a bus's lanes BEFORE any plan: [{'lane', 'island', 'layer', 'box', 'blocked',
    'span', 'firm'}] -- whole_route.layer_cut's form with the lane's own stretch of route u, as whole_solve reads a
    round's.

    A lane's way on a layer is read along its reference line (`refs[lane]`, rows u, x, y: the frame's line, which the
    geometry lays it along): the piece its tooth stands in (its stub's mouth; `ends` names the (lane, end) asked,
    end 0 the tooth, 1 the berth -- those the lane's line starts and ends at), the piece each sample of the line
    stands in, the piece its berth stands in. A WALL is a place where the line passes from one piece to another --
    the copper between them in the way -- and NO piece stands on both sides of that place: the lane cannot be on
    that layer there, whatever it does elsewhere. A line that leaves a piece and comes back to it has gone through
    something the lane goes round in that piece: no wall. So an end shut in by the copper on a layer has its wall
    where its line leaves its piece for good -- at the end itself where nothing is free in front of it -- and a
    lane cannot run from that end on that layer past it.

    `band` None: the pieces of the WHOLE BOARD -- two of them have no way between them anywhere on the layer, and
    a wall is exact. `band` a length: the pieces of the lane's own BAND, its line and that far either side: a way
    round farther off its line than that counts as none. (An earlier bus's tracks fanning out of the source shut a
    layer across every lane that leaves beside them; the way round the far end of those tracks is on the board, and
    no lane's.) That is a judgement of how far the geometry takes a lane off its line, not a fact of the board.

    `entry` / `end`: each lane's route u at its ends. A cut's `span` is the COPPER's own stretch, as a round's cut
    gives an island's: whole_solve holds the lane off the layer a track's bar either side of it (to the two free
    places the wall was read between) and its changes a via's bar -- so a wall at an end is written a via's bar from
    it, and a change AT the end stays the lane's to make. A pair's lane by its own bar, its legs' half width more"""
    import pairs as _pairs
    from shapely.geometry import LineString
    M = list(Fr.M)
    NL = len(LNAME)
    prs = getattr(ctx, 'pairs', {}) or {}
    g2 = ctx.cfg.grid_step / 2
    bar = trk / 2 + clr + g2                # a lane's centreline off fixed copper (whole_embed's LANE_ST)
    t, v = trk / 2 + clr, via / 2 + clr     # what whole_solve grows a cut's span by: for the lane, for its changes
    fixed, cen = fixed_copper(ctx, Fr, LNAME, trk, clr, via, pitch)
    b = ctx.pcb.board_info.board_bounds
    win = (b[0] + cen.EDGE, b[1] + cen.EDGE, b[2] - cen.EDGE, b[3] - cen.EDGE)
    hw = {n: (_pairs.pitch(trk) / 2 / math.cos(math.pi / 8) + g2 if n in prs else 0.0) for n in M}
    free = {}                               # a lane's half width -> the free space at its bar
    asked = set(ends)
    out = []
    for n in M:
        if hw[n] not in free:
            free[hw[n]] = Free(fixed, NL, win, bar + hw[n], trk / 2 + hw[n])
        F = free[hw[n]]
        R = np.asarray(refs[n], float)
        us, xy = R[:, 0], R[:, 1:3]
        ins = bar + hw[n]                   # the lane's own bar: how far inside a free place copper may begin
        reach, step = 3 * ins, 0.4 * ctx.cfg.grid_step
        stub = {e: (ctx.ends[n][e], (ctx.tooth_dir if e == 0 else ctx.stub_dir).get(n))
                for e in (0, 1) if (n, e) in asked}
        seen = set()
        for L in range(NL):
            P = F.pieces(L)
            pc = P.at(xy)
            ms = {e: mouth(P, pt, d, reach, step) for e, (pt, d) in stub.items()}
            if (pc >= 0).all() and len(set(pc.tolist()) | set(ms.values())) == 1:
                continue                    # (the line itself is a free way, its ends in the same piece: no wall)
            if band is not None:
                # (the lane's band: its line, and its stubs' mouths, `band` either side)
                parts = [LineString(xy)]
                for pt, d in stub.values():
                    dl = math.hypot(*d) if d is not None else 0.0
                    if dl > 1e-9:
                        parts.append(LineString([pt, (pt[0] + d[0] / dl * reach, pt[1] + d[1] / dl * reach)]))
                P = F.pieces(L, unary_union(parts).buffer(band))
                pc = P.at(xy)
                ms = {e: mouth(P, pt, d, reach, step) for e, (pt, d) in stub.items()}
            # the line's places in turn: (route u, piece, its point) -- an end with nothing free in front of it a
            # piece of its own
            seq = []
            if 0 in ms:
                seq.append((entry[n], ms[0] if ms[0] >= 0 else -2, stub[0][0], 'tooth'))
            seq += [(float(us[i]), int(pc[i]), xy[i], None) for i in np.nonzero(pc >= 0)[0]]
            if 1 in ms:
                seq.append((end[n], ms[1] if ms[1] >= 0 else -3, stub[1][0], 'berth'))
            last = {}
            for i, q in enumerate(seq):
                last[q[1]] = i
            far = -1                        # the farthest place a piece met so far stands at again
            for i in range(len(seq) - 1):
                far = max(far, last[seq[i][1]])
                if far > i:
                    continue
                (ua, pa, xa, ea), (ub, pb, xb, eb) = seq[i], seq[i + 1]
                # (the copper's own stretch: the lane's bar inside the two free places the wall stands between -- its
                # middle where they are nearer than that; at an end, a via's bar from it, the reader's own measure)
                if ea == 'tooth' and eb == 'berth':
                    # (no free place on its line at all: a wall at each end, and nothing said of the way between --
                    # one stretch from end to end would keep every change of the lane out as well)
                    spans = [(entry[n] + v, entry[n] + v), (end[n] - v, end[n] - v)]
                elif ea == 'tooth':
                    spans = [(entry[n] + v, max(ub - ins, entry[n] + v))]
                elif eb == 'berth':
                    spans = [(min(ua + ins, end[n] - v), end[n] - v)]
                else:
                    spans = [(ua + ins, ub - ins) if ub - ua >= 2 * ins else ((ua + ub) / 2, (ua + ub) / 2)]
                for s0, s1 in spans:
                    if (L, round(s0, 4), round(s1, 4)) in seen:
                        continue
                    seen.add((L, round(s0, 4), round(s1, 4)))
                    c = {'lane': n, 'island': f'wall {LNAME[L]}', 'layer': (1 - L) if NL == 2 else L,
                         'box': [float(min(xa[0], xb[0])), float(min(xa[1], xb[1])),
                                 float(max(xa[0], xb[0])), float(max(xa[1], xb[1]))],
                         'span': [s0, s1], 'firm': True}
                    if NL > 2:
                        c['blocked'] = [L]
                    out.append(c)
                    if log:
                        what = ('its tooth, nothing free in front of it' if pa == -2 else
                                'its berth, nothing free in front of it' if pb == -3 else
                                f'its tooth\'s piece ({P.geoms[pa].area:.2f} mm2)' if ea else
                                f'its berth\'s piece ({P.geoms[pb].area:.2f} mm2)' if eb else 'two pieces on its way')
                        log(f'      {n} on {LNAME[L]}: {what} -- off it over {s0 - t:.2f}..{s1 + t:.2f}')
    return out

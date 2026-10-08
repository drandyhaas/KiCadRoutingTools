"""whole_frame.py -- the whole route's OWN frame of a bench, read off the board's geometry alone: no other planner's
decisions (the braid's corridors, branches, launch offsets or planned paths) enter it.

  lanes      every lane from the source part to the destination part (a differential pair one lane)
  trunk      a straight spine from the source's pad box through the destination's: a lane's route coordinate u is s
             along it, from its ENTRY (its tooth on the spine)
  classes    the face of the destination's pad box a lane berths on: the near (west) face ends in the trunk (its
             END, its berth on the spine); the north and south faces ride a RING; the far (east) face is cut between
             the two rings at its widest gap
  arrival    H0: the first lane that berths on the near face (or, with none there, where the trunk meets the
             berths' box); a ring's lanes leave the trunk at its START, Hk, stepped back from the arrival line out of
             the ring's hull -- ahead of the destination's corner and whatever hugs it
  rings      one per class, round the destination: the octilinear hull of its pads and the class's berth stubs a lane
             pitch out (braid.ring_spine), passing outside the near-face berths and the other parts' pads that hug
             the destination, starting at Hk -- a lane pitch inside the ring's innermost lane on the arrival line,
             stepped back along the trunk until it clears that hull by two lane pitches -- and running round the way
             the lanes' paths sweep
  orders     the two the solve inverts, both north to south: the teeth round the source's pad box (unrolled at a cut
             on its far face) and the berths round the destination's (unrolled at the far-face cut)
  reference  each lane's taut path in the trunk frame: the side of each island it passes, until the geometry flips it

A tooth on the source's far face has no way round the source in this frame, and is refused by name."""
import collections
import math

import numpy as np

import braid as bd
import corridor as cr

FACES = ('E', 'N', 'W', 'S')          # a tie between faces goes to the first


class PairSplit(SystemExit):
    """build's stop on a bench whose pair has another lane's end between its tips on its layer -- a SystemExit, as
    every stage's stop is, carrying `splits` [(pair, 'tooth' | 'berth', [the lanes between its tips])] for a caller
    that repairs rather than stops (fanout_from_plan: a fanout never hands back a split pair)"""

    def __init__(self, splits, msg):
        super().__init__(msg)
        self.splits = splits


class FarFace(SystemExit):
    """build's stop on a bench with a lane's tooth on the source's far face (the whole frame has no way round the
    source) -- carrying `lanes`, for a caller that repairs rather than stops (fanout_from_plan's audit: the next round
    frees those teeth)"""

    def __init__(self, lanes, msg):
        super().__init__(msg)
        self.lanes = lanes


def box_of(pcb, ref):
    ps = pcb.footprints[ref].pads
    return (min(p.global_x for p in ps), min(p.global_y for p in ps),
            max(p.global_x for p in ps), max(p.global_y for p in ps))


def grown(b, pts):
    """the pad box b grown by how far outside it the terminals pts sit: their median distance, read off the board"""
    d = [max(b[0] - x, x - b[2], b[1] - y, y - b[3]) for (x, y) in pts]
    d = sorted(v for v in d if v > 0)
    # the median (numpy's own value: the middle, or the mean of the two middles) -- in python: called for every
    # candidate the ends model scores, numpy's overhead on a few dozen values was a tenth of the search
    n = len(d)
    m = (d[n // 2] if n % 2 else (d[n // 2 - 1] + d[n // 2]) / 2) if n else 0.0
    return (b[0] - m, b[1] - m, b[2] + m, b[3] + m)


def face(p, b):
    x, y = p
    d = {'E': abs(x - b[2]), 'N': abs(y - b[1]), 'W': abs(x - b[0]), 'S': abs(y - b[3])}
    return min(FACES, key=d.get)


def cut(ys, lo, hi):
    """the cut on a far face: the middle of the widest gap along it, the face's two ENDS counted as gap ends -- two
    berths together near one end go round the same ring (counted between the berths alone, any two were split
    between the rings, and one crossed the whole bundle)"""
    pts = [lo] + sorted(ys) + [hi]
    i = max(range(len(pts) - 1), key=lambda i: pts[i + 1] - pts[i])
    return (pts[i] + pts[i + 1]) / 2


def cuts_tied(ys, lo, hi, tol):
    """the cut's candidates on a far face: the middle of the widest gap (cut's), then of every other gap within `tol`
    of it -- a TIE, which the ends model decides by its own score (whole_ends: its best of them)"""
    pts = [lo] + sorted(ys) + [hi]
    gaps = [(pts[i + 1] - pts[i], (pts[i] + pts[i + 1]) / 2) for i in range(len(pts) - 1)]
    i0 = max(range(len(gaps)), key=lambda i: gaps[i][0])
    return [gaps[i0][1]] + [g[1] for i, g in enumerate(gaps) if i != i0 and g[0] > gaps[i0][0] - tol]


def in_frame(spine, path):
    """a path as (s, o) on the spine, s rising (a point that does not advance is dropped)"""
    out = []
    for p in path:
        s, o = spine.project_pt(p)
        if not out or s > out[-1][0] + 1e-9:
            out.append((float(s), float(o)))
    return out


def _dense(path, step):
    """the polyline `path` with points no more than `step` apart"""
    out = [tuple(map(float, path[0]))]
    for (ax, ay), (bx, by) in zip(path, path[1:]):
        k = max(1, int(math.ceil(math.hypot(bx - ax, by - ay) / step)))
        out += [(ax + (bx - ax) * i / k, ay + (by - ay) * i / k) for i in range(1, k + 1)]
    return out


def perim_d(p, DB, ycut):
    """where p lies round the destination's box DB, unrolled from the far (east) face's cut ycut: north first"""
    x0, y0, x1, y1 = DB
    W_, H_ = x1 - x0, y1 - y0
    x, y = p
    sd = face(p, DB)
    if sd == 'E' and y <= ycut:
        return ycut - y
    if sd == 'N':
        return (ycut - y0) + (x1 - x)
    if sd == 'W':
        return (ycut - y0) + W_ + (y - y0)
    if sd == 'S':
        return (ycut - y0) + W_ + H_ + (x - x0)
    return (ycut - y0) + 2 * W_ + H_ + (y1 - y)


def far_cut(cut):
    """whether `cut` stands on the far face: a far face's y (the whole frame's own), or ('E', y)"""
    return not isinstance(cut, (tuple, list)) or cut[0] == 'E'


def cut_point(cut, DB):
    """the point round box DB a cut stands at: a far-face y, or (face, its coordinate along that face -- x on the north
    and south faces, y on the far one)"""
    x0, y0, x1, y1 = DB
    if not isinstance(cut, (tuple, list)):
        return (x1, float(cut))
    f, v = cut[0], float(cut[1])
    return {'E': (x1, v), 'N': (v, y0), 'S': (v, y1)}[f]


def perim_c(p, DB, cut):
    """where p lies round the destination's box DB unrolled from a cut ANYWHERE round it but its facing face (a far-face
    y as perim_d takes it, or (face, coordinate): cut_point), north first -- perim_d's own arithmetic on the far face"""
    if far_cut(cut):
        return perim_d(p, DB, float(cut[1]) if isinstance(cut, (tuple, list)) else cut)
    W_, H_ = DB[2] - DB[0], DB[3] - DB[1]
    return (perim_d(p, DB, DB[1]) - perim_d(cut_point(cut, DB), DB, DB[1])) % (2 * (W_ + H_))


def ring_side(p, DB, cut):
    """the ring a berth at p is reached by with the destination's cut at `cut`: 'N' (round the north, before the
    facing face in the unrolling), 'S' (round the south, after it), or None on the facing face (the trunk's). With
    the cut on the far face, the far face's berths north of it go round the north, as they always have"""
    f_ = face(p, DB)
    if f_ == 'W':
        return None
    if far_cut(cut):
        if f_ == 'E':
            return 'N' if p[1] <= float(cut[1] if isinstance(cut, (tuple, list)) else cut) else 'S'
        return f_
    return 'N' if perim_c(p, DB, cut) < perim_c((DB[0], DB[1]), DB, cut) else 'S'


def cut_gaps(pts, DB, avoid=()):
    """the cuts to try round the destination's box DB: one in each gap between two neighbouring berths `pts` round it
    (a gap may turn a corner), but the facing face's -- each as cut_point takes it. A cut anywhere in one gap makes the
    same orders; a cut ANYWHERE is a lane's way round the destination for every lane past it (WINDING: a berth past the
    far face reached round the other side). Within its gap a cut stands in the middle of the widest stretch clear of
    the points `avoid` (every exit a berth could be laid at): at the gap's middle, another exit of a lane's menu stood
    there, the fanout laid that lane's berth on it, and the frame's orders were no longer the ones the ends model
    priced (s4_bulgeW: 16 crossings planned for 13)"""
    x0, y0, x1, y1 = DB
    W_, H_ = x1 - x0, y1 - y0
    P = 2 * (W_ + H_)
    us = sorted({round(perim_d(p, DB, y0), 6) for p in pts})
    av = sorted({round(perim_d(p, DB, y0), 6) for p in avoid})
    out = []
    for i, a in enumerate(us):
        b = us[(i + 1) % len(us)] + (P if i == len(us) - 1 else 0.0)
        if b - a < 1e-6:
            continue
        inner = sorted(v + (P if v < a else 0.0) for v in av if a < v < b or a < v + P < b)
        stops = [a] + inner + [b]
        j = max(range(len(stops) - 1), key=lambda j: stops[j + 1] - stops[j])
        m = ((stops[j] + stops[j + 1]) / 2) % P
        # (back to a face: unrolled from the north-east corner, north face west, facing face south, south face east,
        # far face north)
        if m < W_:
            out.append(('N', x1 - m))
        elif m < W_ + H_:
            continue                    # (the facing face: its berths are the trunk's)
        elif m < 2 * W_ + H_:
            out.append(('S', x0 + (m - W_ - H_)))
        else:
            out.append(('E', y1 - (m - 2 * W_ - H_)))
    return out


def cut_sig(pts, DB, cut):
    """the berths `pts` (indices) in the order a cut unrolls them: two cuts with one signature make the same orders"""
    return tuple(sorted(range(len(pts)), key=lambda i: (perim_c(pts[i], DB, cut), i)))


def perim_s(p, SB):
    """where p lies round the source's box SB, unrolled from the middle of its far (west) face -- no tooth stands
    there: north first"""
    sx0, sy0, sx1, sy1 = SB
    sW, sH = sx1 - sx0, sy1 - sy0
    scut = (sy0 + sy1) / 2
    x, y = p
    sd = face(p, SB)
    if sd == 'W' and y <= scut:
        return scut - y
    if sd == 'N':
        return (scut - sy0) + (x - sx0)
    if sd == 'E':
        return (scut - sy0) + sW + (y - sy0)
    if sd == 'S':
        return (scut - sy0) + sW + sH + (sx1 - x)
    return (scut - sy0) + 2 * sW + sH + (sy1 - y)


def between(lo, hi, v, whole):
    """v lies between a pair's two tips lo <= hi, the short way round a box of perimeter `whole` (the tips either
    side of the cut: round the far way)"""
    return lo < v < hi if hi - lo <= whole / 2 else (v > hi or v < lo)


def build(ctx, dest, _trunk=frozenset()):
    """the frame (a SimpleNamespace) of the bench ctx (braid.setup's reading of the board: ends, layers, taut paths)
    round the destination part `dest`. A ring lane whose landing lies short of where its ring's lanes enter the ring
    (two lane pitches past the farthest of their handoffs onto it) ends from the TRUNK instead --
    its berth is just round the corner (K51's SDQ4, a berth on the south face beside the corner) -- and the frame is
    built again with it there, until none does"""
    from types import SimpleNamespace
    F = SimpleNamespace()
    pcb = ctx.pcb
    # in the NETS order (setup's own, which tooth_layer keeps): ctx.paths lists the taut memo's hits first, so a
    # lane order read off it changed with the memo's state, and the solve's model with it
    lanes = [n for n in ctx.tooth_layer if n in ctx.paths and ctx.ends[n][2] == dest]
    src = collections.Counter(ctx.src_ref[n] for n in lanes).most_common(1)[0][0]
    F.M = [n for n in lanes if ctx.src_ref[n] == src]
    F.src, F.dst = src, dest
    F.tooth = {n: tuple(map(float, ctx.ends[n][0])) for n in F.M}
    F.bend = {n: tuple(map(float, ctx.ends[n][1])) for n in F.M}
    SB = grown(box_of(pcb, src), F.tooth.values())
    DB = grown(box_of(pcb, dest), F.bend.values())
    F.SB, F.DB = SB, DB                     # the arrays' pad boxes, grown to the line their stubs end on
    # ---- the source side: a tooth on the far (west) face would have to go round the source
    tf = {n: face(F.tooth[n], SB) for n in F.M}
    far = sorted(n for n in F.M if tf[n] == 'W')
    if far:
        raise FarFace(far, f'whole route: {", ".join(far)} launch from the source\'s far face -- the whole frame has no '
                         f'way round the source')
    # ---- the trunk: straight, from the source's pad box through the destination's
    ca = np.array([(SB[0] + SB[2]) / 2, (SB[1] + SB[3]) / 2])
    cb = np.array([(DB[0] + DB[2]) / 2, (DB[1] + DB[3]) / 2])
    d = (cb - ca) / np.hypot(*(cb - ca))
    back = math.hypot(SB[2] - SB[0], SB[3] - SB[1]) / 2 + bd.LPITCH
    fwd = math.hypot(DB[2] - DB[0], DB[3] - DB[1]) / 2 + bd.LPITCH
    F.spine = cr.Spine([tuple(ca - d * back), tuple(cb + d * fwd)])
    F.mid = {n: in_frame(F.spine, ctx.paths[n]) for n in F.M}
    # ---- the destination side: classes, the far face's cut, the berths' order
    x0, y0, x1, y1 = DB
    bf = {n: face(F.bend[n], DB) for n in F.M}
    dcut = cut([F.bend[n][1] for n in F.M if bf[n] == 'E'], y0, y1)
    if getattr(ctx, 'dest_cut', None) is not None:
        # the ends model's own cut (braid.setup, from the plan sidecar): recomputed here from the laid stubs, which
        # stand a hair off the menu's exits, two near-equal gaps could split the far face the other way, and the solve
        # would face crossings the model never priced (zynq K44: 176 crossings planned, 344 solved, no plan proved).
        # A cut off the far face (whole_ends' WINDING) sends the berths past it round the other side
        dcut = ctx.dest_cut
    F.dcut = dcut
    F.cls = {}
    for n in F.M:
        f_ = ring_side(F.bend[n], DB, dcut) or 'W'
        if f_ in ('N', 'S') and n not in _trunk:
            F.cls[n] = f_
    near = [n for n in F.M if n not in F.cls]
    # the REFERENCE paths (the geometry picks each column's free interval nearest one): a lane's taut path, except
    # where it runs INSIDE a pad box -- a taut string has no width and threads gaps between balls no track fits --
    # where it takes the box's edge on the lane's own side (a ring's at the destination, its tooth's face's at the
    # source) a ring margin out: SA0 was bounded south of the destination's corner, its handoff north of it
    for n in F.M:
        side = {}
        if tf[n] in ('N', 'S'):
            side['src'] = tf[n]
        if n in F.cls:
            side['dst'] = F.cls[n]
        if not side:
            continue
        pts = []
        # densified first: a taut string's straight between two corners runs through a box too (K28 SCKE0 from its
        # south-face tooth straight to a corner past the source's east face), and only its points can be moved
        for (x, y) in _dense(ctx.paths[n], bd.LPITCH / 2):
            for key_, b_ in (('src', SB), ('dst', DB)):
                if key_ in side and b_[0] < x < b_[2] and b_[1] < y < b_[3]:
                    y = b_[1] - bd.LPITCH if side[key_] == 'N' else b_[3] + bd.LPITCH
            pts.append((x, y))
        F.mid[n] = in_frame(F.spine, pts)
    F.se = {n: tuple(map(float, F.spine.project_pt(F.bend[n]))) for n in near}
    F.H0 = (min(F.se[n][0] for n in near) if near else float(F.spine.project_pt((x0, (y0 + y1) / 2))[0]))
    W_, H_ = x1 - x0, y1 - y0
    sW, sH = SB[2] - SB[0], SB[3] - SB[1]
    perim_dst = lambda p: perim_c(p, DB, dcut)
    perim_src = lambda p: perim_s(p, SB)
    # a PAIR is one lane here: its two tips must stand together at both ends, no other lane's end between them ON
    # THE PAIR'S LAYER there (a lane ending on the other layer runs under the pair's end legs: K41's SCKE1 berths on
    # B.Cu at the midpoint of SCK's F.Cu berth tips, as a human stacks them)
    prs = getattr(ctx, 'pairs', None) or {}
    split = []
    splits = []                                         # (pair, 'tooth' | 'berth', the lanes between its tips)
    import route_layers
    _nl = len(route_layers.layers())

    def via_end(n_, k_):
        """(more routing layers than two) each leg's stub at end k_ carries a via inside its array: the lane's layer
        there is the solve's (whole_solve's via ends)"""
        if _nl == 2:
            return False
        box = F.SB if k_ == 0 else F.DB
        for leg in (prs[n_] if n_ in prs else (n_,)):
            nid = ctx.byname[leg][0]
            if not any(v.net_id == nid and box[0] - 1e-6 <= v.x <= box[2] + 1e-6 and box[1] - 1e-6 <= v.y <= box[3] + 1e-6
                       for v in ctx.base_vias):
                return False
        return True
    # every lane with an end between a pair's two tips, whatever its layer there: a solve that chooses ends' layers
    # (whole_solve's via ends, on more routing layers than two) keeps each off the pair's
    F.tip_between = {}
    for k_, perim, whole in ((0, perim_src, 2 * (sW + sH)), (1, perim_dst, 2 * (W_ + H_))):
        lay_ = ctx.tooth_layer if k_ == 0 else ctx.dest_layer
        at = []                                         # (position round the box, lane, its layer there)
        for n in F.M:
            tips = ctx.pair_ends[n][k_] if n in prs else (ctx.ends[n][k_],)
            at += [(perim(tuple(map(float, t))), n, lay_[n]) for t in tips if t is not None]
        for n in prs:
            vs = sorted(v for v, o, _L in at if o == n)
            if n not in F.M or len(vs) < 2:
                continue
            F.tip_between[(n, k_)] = sorted({o for v, o, _L in at if o != n and between(vs[0], vs[-1], v, whole)})
            inside = sorted({o for v, o, L in at if o != n and L == lay_[n] and between(vs[0], vs[-1], v, whole)
                             and not (via_end(n, k_) or via_end(o, k_))})
            if inside:
                split.append(f'{n} at its {("tooth", "berth")[k_]} (round {", ".join(inside)})')
                splits.append((n, ('tooth', 'berth')[k_], inside))
    if split:
        raise PairSplit(splits, f'whole route: pair(s) split by other lanes -- {"; ".join(split)} -- a pair is one lane')
    F.P = {n: perim_dst(F.bend[n]) for n in F.M}
    # each lane's LANDING: where its lane meets its berth -- the berth itself, or for a pair on a ring's face (its stub
    # across the ring) the end of the straight run it takes out of its berth (pairs.end_run: its end connector onto
    # the pose and the router's probe past it), which the lane then runs straight into the berth -- and past that the
    # run its turn off the ring takes: the pair turns 90 degrees there in two 45-degree bends a turning radius apart
    # (pairs.turn_straight_steps), and the diagonal between them eats that much of the straight on either side of the
    # corner; and a grid step more, as the lane's line lands between grid lines. Landed at the end run alone, the turn
    # had no room (K28 SDQS1 at DU1's north-west corner: no pose reachable from any start); at the turn's run exactly,
    # the deepest pose lay one cell past the reach of a line rounded to the grid
    import pairs as _pairs
    # The same for a pair ending from the TRUNK on a face across it (a berth just round the destination's corner): it
    # arrived along the trunk and hooked into its tips with no end run (K35 SDQS0 at DU1's south face)
    F.land = dict(F.bend)
    td = tuple(float(v) for v in F.spine.d[-1])
    for n in [n for n in prs if n in F.M]:
        e = ctx.stub_dir.get(n)
        if e is None:
            continue
        el = math.hypot(*e)
        if n in F.cls or abs(e[0] * td[0] + e[1] * td[1]) < math.sqrt(0.5) * el:
            tips = ctx.pair_ends[n][1]
            m = _pairs.mid(tips[0], tips[1])
            r = _pairs.end_run(ctx.cfg, tips, e) + (_pairs.turn_straight_steps(ctx.cfg) + 1) * ctx.cfg.grid_step
            F.land[n] = (float(m[0] + e[0] * r), float(m[1] + e[1] * r))
    # ...and each lane's START, the same at its tooth: the tooth, or for a pair whose stub stands across the trunk (a
    # tooth on the source's side face) the end of its end run out of its tooth and the run its turn onto the trunk
    # takes -- the lanes launched farther along that face pass outside it there, as a ring's lanes pass outside a ring
    # pair's landing. Started at its tooth, the pair had no room to turn: the lanes from the face's far part passed
    # 0.36 to 0.58 mm out, and it folded between them and its own teeth (K35 and K41: SCK on the source's south face)
    F.start = dict(F.tooth)
    t0 = tuple(float(v) for v in F.spine.d[0])
    for n in [n for n in prs if n in F.M]:
        e = ctx.tooth_dir.get(n)
        if e is None:
            continue
        el = math.hypot(*e)
        if el > 1e-9 and abs(e[0] * t0[0] + e[1] * t0[1]) < math.sqrt(0.5) * el:
            tips = ctx.pair_ends[n][0]
            m = _pairs.mid(tips[0], tips[1])
            r = _pairs.end_run(ctx.cfg, tips, e) + (_pairs.turn_straight_steps(ctx.cfg) + 1) * ctx.cfg.grid_step
            F.start[n] = (float(m[0] + e[0] / el * r), float(m[1] + e[1] / el * r))
    F.st = {n: tuple(map(float, F.spine.project_pt(F.start[n]))) for n in F.M}
    F.launch = sorted(F.M, key=lambda n: perim_src(F.tooth[n]))
    F.final = sorted(F.M, key=lambda n: F.P[n])
    # ---- the rings
    dpads = [(p.global_x, p.global_y) for p in pcb.footprints[dest].pads]
    cen = (sum(p[0] for p in dpads) / len(dpads), sum(p[1] for p in dpads) / len(dpads))
    north = 1.0 if F.spine.xy(F.H0, 1.0)[1] < F.spine.xy(F.H0, 0.0)[1] else -1.0   # the sign of o toward the north
    width = lambda n: bd.LANE_MIN + (_pairs.pitch(bd.TRACK) if n in (getattr(ctx, 'pairs', None) or {}) else 0.0)
    F.o_h = {n: F.se[n][1] for n in near}
    tails = [tuple(map(float, F.spine.xy(F.H0, F.o_h[n]))) for n in near] + [F.bend[n] for n in near]
    # the rings go outside the other parts' copper that hugs the destination too (a decoupling cap at its corner): a
    # pad within a lane pitch and two lanes' room of the hull is one more thing the ring goes round, its corners in it
    base = dpads + [F.bend[n] for n in F.M] + tails
    for ref_, fp_ in pcb.footprints.items():
        if ref_ in (src, dest):
            continue
        for p_ in fp_.pads:
            if _rings_round(p_) and _inside((p_.global_x, p_.global_y), base, bd.LPITCH + 2 * bd.LANE_MIN):
                hx, hy = (p_.size_x or 0.0) / 2, (p_.size_y or 0.0) / 2
                tails += [(p_.global_x + sx * hx, p_.global_y + sy * hy) for sx in (-1, 1) for sy in (-1, 1)]
    # a part ON a ring's stack -- the handoff line from the ring's start to its outermost lane, its copper within a
    # clearance -- though it lies past the hull's margin: stacked through it, a ring's first lane was planned on the
    # part's pad. The ring passes INSIDE it when its lanes fit between it and the destination's copper, the stack
    # packed into that gap; else the part is one more thing the ring goes round. Taken round always, a header lying
    # along the face with the ring's berths between it and the face sent the ring outside it and its lanes back in
    # (synth_handoff pth_corner); stacked through always, a cap at the corner had the ring's start on its pad
    # (s4_pathF)
    in_hull, packed = set(), {}
    for _pass in range(2 * len(pcb.footprints) + 2):
        hits, room = _stack_rings(F, ctx, pcb, src, dest, dpads, near, tails, north, width, cen, in_hull, packed)
        acted = False
        for k in sorted(hits):
            ref_, inner = min(hits[k], key=lambda h: (h[1], h[0]))      # the nearest part on the stack
            edge_in = room_inside(*room[k], inner, bd.TRACK, bd.CLEAR, bd.LANE_MIN)
            if edge_in is not None and packed.get(k) != edge_in:
                packed[k] = edge_in                     # inside: the lanes packed between the destination and it
                acted = True
            elif edge_in is None:
                in_hull.add(ref_)                       # no room: round it
                for p_ in pcb.footprints[ref_].pads:
                    hx, hy = (p_.size_x or 0.0) / 2, (p_.size_y or 0.0) / 2
                    tails += [(p_.global_x + sx * hx, p_.global_y + sy * hy) for sx in (-1, 1) for sy in (-1, 1)]
                acted = True
        if not acted:
            break
    short = set()
    for k, ring in F.rings.items():
        mem = [n for n in F.M if F.cls.get(n) == k]
        entry = max(ring.project_pt(F.spine.xy(F.Hk[k], F.o_h[n]))[0] for n in mem) + 2 * bd.LANE_MIN
        short |= {n for n in mem if ring.project_pt(F.land[n])[0] < entry}
    if short:
        return build(ctx, dest, _trunk | frozenset(short))
    return F


def _rings_round(p_):
    """a pad the rings go round: on two routing layers any; on more, one standing on every routing layer (drilled,
    or copper on every one) -- a part's pads on one face the lanes on the other layers pass under"""
    import route_layers
    RL = route_layers.layers()
    if len(RL) == 2:
        return True
    return bool((p_.drill or 0) > 0 or '*.Cu' in p_.layers or set(RL) <= set(p_.layers))


def _stack_rings(F, ctx, pcb, src, dest, dpads, near, tails, north, width, cen, in_hull, packed):
    """each ring's lanes stacked on the handoff line outside the hull (F.o_h) -- or, a ring in `packed`, from that
    edge, between the destination and a part on its stack -- its start and spine (F.Hk, F.rings). Returns (hits,
    room): hits {ring: [(part, its copper's nearest offset toward the destination, along the ring's side)]} for the
    parts (none in the hull) whose copper lies on a ring's stack -- within a clearance of its outermost lane's copper,
    on the handoff line from the ring's start out; room {ring: (the destination's copper's farthest offset on that
    side, the facing face's lanes' edge, the ring's lanes' width)}"""
    hull = dpads + [F.bend[n] for n in F.M] + tails
    rpad = max((max(p.size_x or 0.0, p.size_y or 0.0) / 2 for p in pcb.footprints[dest].pads), default=0.0)
    # each ring lane's offset on the handoff line, from the ORDER (the berths', north to south), stacked innermost first
    # a lane apart OUTSIDE its ring's own edge: beyond the outermost near-face berth, and beyond the hull on that side
    # by the ring's margin -- so the ring starts outside the destination's corner and leaves the trunk along it, every
    # lane crossing the handoff line onto the ring where it stands (stacked from the near-face berths alone, K51's south
    # ring began inside the corner, ran down its west face, and its lanes met it up to 2 mm apart)
    edge = {}
    for k in ('N', 'S'):
        sg = north if k == 'N' else -north
        # (the outermost near-face lane's own room outside its centre: a pair's is half its pitch more)
        near_edge = max((sg * F.se[n][1] + (width(n) - bd.LANE_MIN) / 2 for n in near), default=-bd.LANE_MIN / 2)
        hull_edge = max(sg * F.spine.project_pt(p_)[1] for p_ in hull) + 2 * bd.LPITCH - bd.LANE_MIN
        edge[k] = max(near_edge, hull_edge) if k not in packed else min(max(near_edge, hull_edge), packed[k])
        mem = [n for n in F.final if F.cls.get(n) == k]
        at = edge[k]
        for n in (mem[::-1] if k == 'N' else mem):
            F.o_h[n] = sg * (at + width(n) / 2 + bd.LANE_MIN / 2)
            at += width(n)
    F.rings, F.Hk = {}, {}
    for k in sorted(set(F.cls.values())):
        mem = [n for n in F.M if F.cls.get(n) == k]
        sg = north if k == 'N' else -north
        stubs = [F.bend[n] for n in mem]
        # the ring's lead-in runs taut from its start to where it grazes the hull (corridor.build_wrap_spine), so the
        # start stands OUTSIDE the hull: a lane pitch inside the innermost ring lane on the arrival line, moved back
        # along the trunk until it clears the hull by two lane pitches
        o_start = F.o_h[min(mem, key=lambda n: sg * F.o_h[n])] - sg * bd.LPITCH
        s_start = F.H0
        hull = dpads + stubs + tails
        while _inside(F.spine.xy(s_start, o_start), hull, 2 * bd.LPITCH) and s_start > 0:
            s_start -= bd.LANE_MIN / 5
        ccw = sum(bd.sweep_round(ctx.paths[n], cen) for n in mem) > 0
        if not far_cut(F.dcut):
            # (a cut off the far face: a ring's lanes past the far face are WOUND round it, their taut paths the short
            # way round the other side -- the ring runs the way its class goes, from the facing face round its side)
            DB_ = F.DB
            wn = ((DB_[0], (DB_[1] + DB_[3]) / 2), ((DB_[0] + DB_[2]) / 2, DB_[1]), (DB_[2], (DB_[1] + DB_[3]) / 2))
            ccw = (bd.sweep_round(list(wn), cen) > 0) == (k == 'N')
        F.Hk[k] = float(s_start)                         # this ring's lanes leave the trunk here
        _core, F.rings[k] = bd.ring_spine(dpads, [F.spine.xy(s_start, F.o_h[n]) for n in mem], stubs,
                                          F.spine.xy(s_start, o_start), ccw, tuple(float(v) for v in F.spine.d[-1]),
                                          tails, wrap_side=not far_cut(F.dcut))
    hits, room = {}, {}
    for k in F.rings:
        mem = [n for n in F.M if F.cls.get(n) == k]
        sg = north if k == 'N' else -north
        o_of = lambda p_: sg * F.spine.project_pt(p_)[1]
        # the destination's copper's farthest offset on the ring's side: its pads (their reach) and the berths' ends
        e_ = max([o_of(p_) + rpad for p_ in dpads] + [o_of(F.bend[n]) + bd.TRACK / 2 for n in F.M])
        near_e = max((sg * F.se[n][1] + (width(n) - bd.LANE_MIN) / 2 for n in near), default=-bd.LANE_MIN / 2)
        room[k] = (e_, near_e, sum(width(n) for n in mem))
        out_ = max(mem, key=lambda n: sg * F.o_h[n])
        a = F.spine.xy(F.Hk[k], sg * (min(sg * F.o_h[n] for n in mem) - bd.LPITCH))     # the ring's start
        b = F.spine.xy(F.Hk[k], F.o_h[out_])                                             # its outermost lane
        reach = (width(out_) - bd.LANE_MIN) / 2 + bd.TRACK / 2 + bd.CLEAR                 # its copper and a clearance
        for ref_, fp_ in pcb.footprints.items():
            if ref_ in (src, dest) or ref_ in in_hull:
                continue
            rects = [(p_.global_x - (p_.size_x or 0.0) / 2, p_.global_y - (p_.size_y or 0.0) / 2,
                      p_.global_x + (p_.size_x or 0.0) / 2, p_.global_y + (p_.size_y or 0.0) / 2) for p_ in fp_.pads
                     if _rings_round(p_)]
            if any(_seg_rect(a, b, r_) < reach - 1e-9 for r_ in rects):
                inner = min(o_of(c_) for r_ in rects for c_ in ((r_[0], r_[1]), (r_[0], r_[3]), (r_[2], r_[1]),
                                                                  (r_[2], r_[3])))
                hits.setdefault(k, []).append((ref_, inner))
    return hits, room


def room_inside(e_, near_e, sum_w, inner, track, clear, lane_min):
    """the stack's edge (the `edge` a ring's lanes are stacked from: a lane's pitch inside its first lane) that packs a
    ring's lanes between the destination's copper (its farthest offset on the ring's side, `e_`) and a part's nearest
    copper (`inner`), never inside the facing face's lanes (`near_e`); None when they do not fit. `sum_w`: the ring's
    lanes' widths (a lane's pitch each, a pair's more). Its first lane's copper a clearance off the destination's, its
    last's a clearance off the part's"""
    edge = max(near_e, e_ + clear + track / 2 - lane_min)
    return edge if edge + sum_w + track / 2 + clear <= inner + 1e-9 else None


def _seg_rect(a, b, r):
    """the distance from segment a-b to the rectangle r (x0, y0, x1, y1): 0 where they meet"""
    x0, y0, x1, y1 = r
    inside = lambda p: x0 <= p[0] <= x1 and y0 <= p[1] <= y1

    def pt_seg(p, q0, q1):
        dx, dy = q1[0] - q0[0], q1[1] - q0[1]
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, ((p[0] - q0[0]) * dx + (p[1] - q0[1]) * dy) / L2))
        return math.hypot(p[0] - q0[0] - t * dx, p[1] - q0[1] - t * dy)

    def cross(p, q, u, v):
        d = lambda o, e, f: (e[0] - o[0]) * (f[1] - o[1]) - (e[1] - o[1]) * (f[0] - o[0])
        return d(p, q, u) * d(p, q, v) < 0 and d(u, v, p) * d(u, v, q) < 0
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    if inside(a) or inside(b) or any(cross(a, b, corners[i], corners[(i + 1) % 4]) for i in range(4)):
        return 0.0
    return min([pt_seg(c, a, b) for c in corners]
               + [pt_seg(p, corners[i], corners[(i + 1) % 4]) for p in (a, b) for i in range(4)])


def _inside(p, pts, margin):
    """p within the octilinear hull of pts pushed out by margin (corridor.octo_hull's eight support lines)"""
    for k in range(8):
        d = (math.cos(k * math.pi / 4), math.sin(k * math.pi / 4))
        if p[0] * d[0] + p[1] * d[1] > max(q[0] * d[0] + q[1] * d[1] for q in pts) + margin:
            return False
    return True


def ref(F, n, s):
    """lane n's reference offset at s along the trunk: its taut path's"""
    m = F.mid[n]
    return float(np.interp(s, [p[0] for p in m], [p[1] for p in m]))

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
             on its far face, src_cut: the gap between its teeth that crosses the fewest lanes) and the berths round
             the destination's (unrolled at the far-face cut)
  lead-ins   a tooth on the source's far face (the face its escape leaves by, tooth_face) goes round the source's
             corner on its side of that cut: straight out of the face, along it and round the corner, nested in its
             side's launch order, to a START beside the side face, where the trunk takes it as a side face's lane
  reference  each lane's taut path in the trunk frame: the side of each island it passes, until the geometry flips it
  group      (DEST naming several parts, 'J1+U3+U5': one source, several ends) ONE destination: the parts' copper
             boxes joined into one outline, its order a walk round it, its front the hull's edges facing the source,
             a lane's place where it enters the group -- out of a pocket's mouth, under a part, or on the outline
             (Group); the trunk runs to the group, its rings round all of it

A far-face tooth still INSIDE the source's array (a ball's via, its lane not yet out of the field) has no way round
the source, and is refused by name."""
import collections
import math

import numpy as np

import awx_settings
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
    """build's stop on a bench with a lane's tooth on the source's far face still inside the array (no lead-in round
    the source from there) -- carrying `lanes`, for a caller that repairs rather than stops (fanout_from_plan's audit:
    the next round frees those teeth)"""

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


def via_in_box(v, b):
    """via `v` stands in box `b` (an array's pad box grown to its stub ends' line): its barrel reaching the box. A
    dog-bone's via at its stub's end stands ON that line, and the box is grown by the MEDIAN of the stubs' standoffs --
    a face whose stubs end a hair short of it (the board's coordinates written to a tenth of a micron) left its berths'
    vias outside by that hair, read as no via ends: the solve held those lanes to the stub's layer there, and each via
    laid joined nothing"""
    r = (getattr(v, 'size', 0.0) or 0.0) / 2
    return b[0] - r <= v.x <= b[2] + r and b[1] - r <= v.y <= b[3] + r


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


def perim_s(p, SB, scut=None):
    """where p lies round the source's box SB, unrolled from the far (west) face's cut `scut` (a y on it; None its
    middle): north first -- a tooth on the far face north of the cut goes round the north, south of it round the
    south"""
    sx0, sy0, sx1, sy1 = SB
    sW, sH = sx1 - sx0, sy1 - sy0
    scut = (sy0 + sy1) / 2 if scut is None else float(scut)
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


TIE_MM = 0.005                         # exits nearer than this round a box are ONE exit


def tie_rank(vals):
    """{key: the index of its TIE CLUSTER} for {key: place round a box}: places within TIE_MM of the one before them
    are one exit -- two lanes leaving one point on two layers, a stack -- whose lanes may leave in either order; their
    order there is decided by the other end, so a tie never makes a crossing (broken by the lanes' index instead, a
    tie made crossings the ends model could not see: on one escape bus all ten it missed were stacks)"""
    out, k_, prev = {}, -1, None
    for key, v in sorted(vals.items(), key=lambda kv: kv[1]):
        if prev is None or v - prev > TIE_MM:
            k_ += 1
        out[key] = k_
        prev = v
    return out


def tooth_face(p, SB, d=None):
    """the face of the source's box SB a tooth at p leaves by: the one its escape `d` points out of, where the escape
    runs along an axis -- else the face nearest it. Read by nearness alone, an exit past the north face beside the far
    corner, or a ball's via whose escape runs north, was a far-face tooth"""
    if d is not None:
        dx, dy = float(d[0]), float(d[1])
        L = math.hypot(dx, dy)
        if L > 1e-9 and max(abs(dx), abs(dy)) > 0.9 * L:
            return ('E' if dx > 0 else 'W') if abs(dx) > abs(dy) else ('S' if dy > 0 else 'N')
    return face(p, SB)


def src_cut(ys, lo, hi, rank=None):
    """the cut on the source's far face (`ys` its teeth's y, the face from lo to hi): the middle of the gap that splits
    them with the fewest crossings -- `rank(cut)` the crossings a cut makes (None: the widest gap, cut's) -- the widest
    such gap on a tie. A far-face tooth north of the cut goes round the north (perim_s), outside every lane of the
    north face, so the cut decides which lanes it crosses: in the middle of the face, two far teeth whose berths both
    lie north were split, and the southern one crossed the whole bus"""
    pts = [lo] + sorted(ys) + [hi]
    gaps = [(pts[i + 1] - pts[i], (pts[i] + pts[i + 1]) / 2) for i in range(len(pts) - 1)]
    if rank is None:
        return max(gaps)[1]
    return min(gaps, key=lambda g: (rank(g[1]), -g[0], g[1]))[1]


HALF_G = 0.0125      # half the router's grid step: a lead-in's bar off the grid, as the audit's static bar adds it


def _seg_box(a, b, box):
    """the least distance between the segment a-b and the axis-aligned box (x0, y0, x1, y1): 0 where they meet"""
    x0, y0, x1, y1 = box
    inside = lambda p: x0 <= p[0] <= x1 and y0 <= p[1] <= y1
    if inside(a) or inside(b):
        return 0.0
    edges = [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]

    def pt_seg(p, q, r):
        dx, dy = r[0] - q[0], r[1] - q[1]
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 < 1e-18 else max(0.0, min(1.0, ((p[0] - q[0]) * dx + (p[1] - q[1]) * dy) / l2))
        return math.hypot(p[0] - q[0] - t * dx, p[1] - q[1] - t * dy)

    def cross(p, q, r, s_):
        o = lambda u, v, w: (v[0] - u[0]) * (w[1] - u[1]) - (v[1] - u[1]) * (w[0] - u[0])
        return o(p, q, r) * o(p, q, s_) < 0 and o(r, s_, p) * o(r, s_, q) < 0
    if any(cross(a, b, e0, e1) for e0, e1 in edges):
        return 0.0
    return min([pt_seg(a, e0, e1) for e0, e1 in edges] + [pt_seg(b, e0, e1) for e0, e1 in edges]
               + [pt_seg(c_, a, b) for c_ in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))])


def lead_obstacles(ctx, src, dest, SB, run_ids):
    """the copper a LEAD-IN may meet outside the source's box: other parts' pads, other nets' vias and tracks -- [(box
    (x0, y0, x1, y1), its layers: None for every layer)] (lead_ins goes round it)"""
    out = []
    for ref_, fp_ in ctx.pcb.footprints.items():
        if ref_ in (src, dest):
            continue
        for p_ in fp_.pads:
            Ls = [L for L in p_.layers if L.endswith('.Cu')]
            if not Ls and not (p_.drill or 0) > 0:
                continue
            hx, hy = (p_.size_x or 0.0) / 2, (p_.size_y or 0.0) / 2
            if p_.pad_type == 'np_thru_hole':
                hx = hy = (p_.drill or 0.0) / 2
            allL = (p_.drill or 0) > 0 or '*.Cu' in p_.layers
            out.append(((p_.global_x - hx, p_.global_y - hy, p_.global_x + hx, p_.global_y + hy),
                        None if allL else set(Ls)))
    for v_ in getattr(ctx, 'base_vias', None) or ctx.pcb.vias:
        if v_.net_id not in run_ids:
            r_ = v_.size / 2
            out.append(((v_.x - r_, v_.y - r_, v_.x + r_, v_.y + r_), None))
    for s_ in getattr(ctx, 'base_segments', None) or ctx.pcb.segments:
        if s_.net_id not in run_ids and not getattr(s_, 'graphic', False):
            w_ = s_.width / 2
            out.append(((min(s_.start_x, s_.end_x) - w_, min(s_.start_y, s_.end_y) - w_,
                         max(s_.start_x, s_.end_x) + w_, max(s_.start_y, s_.end_y) + w_), {s_.layer}))
    # (inside the source's box: the array's own field, which the lead-ins leave by the far face's teeth)
    return [(b_, L_) for b_, L_ in out if not (b_[0] >= SB[0] and b_[2] <= SB[2] and b_[1] >= SB[1] and b_[3] <= SB[3])]


def lead_ins(F, far, scut, width, SB, obst=(), layer=None):
    """{lane: [points]}: each far-face lane's LEAD-IN, from its tooth round the source's corner to its START on the
    north or south face (F.start). The trunk frame cannot carry a lane along the far face -- its columns stand across
    the spine, and that stretch runs along them -- so the lane leaves its tooth straight out of the face, runs along it
    and round its corner on a 45-degree chamfer, and starts beside the side face, where the trunk takes it as it takes a
    side face's lanes. The lanes of a side nest in the order they launch (perim_s from the cut): the tooth nearest the
    corner innermost, a lane's room apart, every lane's run a pitch out of the far face's teeth and the side face's --
    and OUTSIDE the copper standing in their way (`obst`, lead_obstacles: a part's pad, another net's via or track on a
    lead-in's layer, `layer(n)`), as the destination's rings go round the parts hugging it: its run along the face and
    its chamfer and start are moved out past it (the straight out of the tooth is the ends model's to price, its front)"""
    out = {}
    xf0 = min([SB[0]] + [F.tooth[n][0] for n in far])                 # the far face's teeth's line
    rest = [F.tooth[m][1] for m in F.M if m not in far]
    bar = bd.CLEAR + bd.TRACK / 2
    for side in ('N', 'S'):
        mem = [n for n in far if (F.tooth[n][1] <= scut) == (side == 'N')]
        if not mem:
            continue
        # (innermost first: nearest the corner it rounds)
        mem.sort(key=lambda n: F.tooth[n][1] if side == 'N' else -F.tooth[n][1])
        sg = -1.0 if side == 'N' else 1.0                              # (y toward the side, out of the box)
        # (the side face's teeth's line: the box's, or past it where a side face's stub stands further out)
        xf = xf0
        yf = min([SB[1]] + rest) if side == 'N' else max([SB[3]] + rest)
        lays = {layer(n) for n in mem} if layer else None
        boxes = [b_ for b_, L_ in obst if L_ is None or lays is None or L_ & lays]
        for _pass in range(12):
            side_out, at, d0 = {}, bd.LANE_MIN, None
            for n in mem:
                d = at + (width(n) - bd.LANE_MIN) / 2
                d0 = d if d0 is None else d0
                # (concentric 45-degree chamfers a lane's room apart: two chamfer lines x +- y = const stand (2 (d -
                # d0) - (c - c0)) / sqrt 2 apart, so c grows by (2 - sqrt 2) per mm of d)
                c = 2 * bd.LANE_MIN + (2 - math.sqrt(2)) * (d - d0)
                x_, y_ = xf - d, yf + sg * d
                t = tuple(map(float, F.tooth[n]))
                y_turn = y_ - sg * c
                side_out[n] = (d, [t, (x_, t[1])] + ([(x_, y_turn)] if sg * (t[1] - y_turn) < 0 else [])
                               + [(x_ + c, y_)])
                at += width(n)
            moved = False
            for n, (d, pts) in side_out.items():
                for k_, (a, b) in enumerate(zip(pts[1:], pts[2:])):
                    need = bar + (width(n) - bd.LANE_MIN) / 2 + HALF_G     # (the audit's bar, off the grid)
                    hit = next((b_ for b_ in boxes if _seg_box(a, b, b_) < need), None)
                    if hit is None:
                        continue
                    if k_ == 0 and abs(a[0] - b[0]) < 1e-9:
                        xf_ = min(xf, hit[0] - need + d)          # (the run along the far face: out past it)
                        moved, xf = moved or xf_ < xf - 1e-6, xf_
                    else:
                        yf_ = (min(yf, hit[1] - need + d) if side == 'N'
                               else max(yf, hit[3] + need - d))           # (the chamfer and start: out past it)
                        moved, yf = moved or abs(yf_ - yf) > 1e-6, yf_
                    break
            if not moved:
                break
        out.update({n: pts for n, (d, pts) in side_out.items()})
    return out


def between(lo, hi, v, whole):
    """v lies between a pair's two tips lo <= hi, the short way round a box of perimeter `whole` (the tips either
    side of the cut: round the far way)"""
    return lo < v < hi if hi - lo <= whole / 2 else (v > hi or v < lo)


# ---- the destination GROUP: DEST naming several parts, one source and several ends, taken as ONE destination. Its
# shape is the parts' copper boxes joined into one outline -- the strips between them that must close for the parts
# to make one shape closed (POCKETS), the wider ones left open (NOTCHES) -- and its order is one WALK round that
# outline: a lane's place is where it enters the outline, a lane ending in a notch entering its mouth in its berth's
# order along the notch's walls. The FRONT is the stretch of the walk under the convex hull's edges that face the
# source (a face itself, or a notch opening to it): its lanes end from the trunk, as a single destination's facing
# face's do, and the rest ride the rings round the hull, unrolled at a cut anywhere but the front. A lane may enter
# the group more than one way -- out of either mouth of its pocket, straight under the part in front of it on a
# layer that part leaves open, or under its own part from the side turned from the source -- each way an OPTION
# with its own place; the group's orders are its lanes' options' places, chosen with the cut (Group.arrange)
GROUP_OPTIONS = int(awx_settings.get('GROUP_OPTIONS') or 3)     # each lane's ways in searched, the shortest first
FRONT_COS = math.sqrt(0.5)      # a hull edge faces the source within 45 degrees of it: its lanes reach it head on
GROUP_TOL = 0.05                # a berth this near the outline is on it
GROUP_EPS = 0.02                # mm of walk per mm of depth: lanes leaving one mouth, the shallowest nearest its wall
GROUP_LEG_PASS = float(awx_settings.get('GROUP_LEG_PASS') or 1.0)   # vias for each pin a pocket lane's leg passes
                                # (berths a pin pitch apart a hundredth of a mm or more: past TIE_MM, never a stack)


def copper_box(pcb, ref):
    """the box of part `ref`'s pads' copper (an NPTH: its hole)"""
    xs0, ys0, xs1, ys1 = [], [], [], []
    for p in pcb.footprints[ref].pads:
        hx, hy = (p.size_x or 0.0) / 2, (p.size_y or 0.0) / 2
        if p.pad_type == 'np_thru_hole':
            hx = hy = (p.drill or 0.0) / 2
        xs0.append(p.global_x - hx)
        ys0.append(p.global_y - hy)
        xs1.append(p.global_x + hx)
        ys1.append(p.global_y + hy)
    return (min(xs0), min(ys0), max(xs1), max(ys1))


def open_layers(pcb, ref):
    """the routing layers no pad of part `ref` stands on (a drilled pad, or copper on every layer, stands on all)"""
    import route_layers
    RL = route_layers.layers()
    used = set()
    for p in pcb.footprints[ref].pads:
        if (p.drill or 0) > 0 or '*.Cu' in p.layers:
            return frozenset()
        used |= {L for L in p.layers if L in RL}
    return frozenset(L for L in RL if L not in used)


def bridge_of(boxes, pitch):
    """the width of strip the group's outline closes: its parts joined by their nearest gaps (the longest edge of the
    tree that joins every part by the shortest gaps between their boxes), and a lane's pitch more -- a strip that
    narrow closed, its lanes entering it from its ends; any wider left open, a notch whose walls the walk follows"""
    ks = list(boxes)
    gap = lambda a, b: math.hypot(max(0.0, b[0] - a[2], a[0] - b[2]), max(0.0, b[1] - a[3], a[1] - b[3]))
    inn, far = {ks[0]}, 0.0
    while len(inn) < len(ks):
        g, k = min((gap(boxes[a], boxes[b]), b) for a in inn for b in ks if b not in inn)
        inn.add(k)
        far = max(far, g)
    return far + pitch


class Group:
    """the destination GROUP's shape and walk (see above): parts `refs` of `pcb`, the source's centre `sc`. The walk's
    coordinate s runs round the outline in perim_d's sense -- along the front from north to south"""

    def __init__(self, pcb, refs, sc):
        from shapely.geometry import LinearRing, MultiPoint, Polygon, box as _box
        from shapely.ops import unary_union
        self.pcb, self.refs = pcb, list(refs)
        self.boxes = {r: copper_box(pcb, r) for r in self.refs}
        self.open = {r: open_layers(pcb, r) for r in self.refs}
        xs = [v for b in self.boxes.values() for v in (b[0], b[2])]
        ys = [v for b in self.boxes.values() for v in (b[1], b[3])]
        self.bbox = (min(xs), min(ys), max(xs), max(ys))
        gc = ((self.bbox[0] + self.bbox[2]) / 2, (self.bbox[1] + self.bbox[3]) / 2)
        dl = math.hypot(gc[0] - sc[0], gc[1] - sc[1]) or 1.0
        self.d = ((gc[0] - sc[0]) / dl, (gc[1] - sc[1]) / dl)          # from the source toward the group
        self.bridge = bridge_of(self.boxes, bd.LPITCH)
        U = unary_union([_box(*b) for b in self.boxes.values()]).buffer(self.bridge / 2, join_style=2) \
            .buffer(-self.bridge / 2, join_style=2)
        if U.geom_type != 'Polygon':
            raise SystemExit(f'whole route: the destination group {"+".join(self.refs)} makes no one outline')
        cs = list(U.exterior.coords)
        if Polygon(cs).exterior.is_ccw:
            cs = cs[::-1]               # (perim_d's sense: on the board's axes, north along the far face, south the front)
        self.E = LinearRing(cs)
        self.U = Polygon(cs)            # (a hole -- copper the parts enclose -- is no lane's)
        self.L = self.E.length
        self.parts_u = unary_union([_box(*b) for b in self.boxes.values()])
        self.hull = MultiPoint([c for b in self.boxes.values()
                                for c in ((b[0], b[1]), (b[2], b[1]), (b[2], b[3]), (b[0], b[3]))]).convex_hull
        # the FRONT: the walk under the hull's edges facing the source, from its north end f0 to its south end f1
        hv = sorted({(round(x, 9), round(y, 9)) for x, y in self.hull.exterior.coords[:-1]}, key=self.s)
        best, front = None, []
        for i, a in enumerate(hv):
            b = hv[(i + 1) % len(hv)]
            el = math.hypot(b[0] - a[0], b[1] - a[1]) or 1.0
            nx, ny = -(b[1] - a[1]) / el, (b[0] - a[0]) / el                  # (outward, in the walk's sense)
            f = -(nx * self.d[0] + ny * self.d[1])
            if f >= FRONT_COS - 1e-9:
                front.append((self.s(a), self.s(b)))
            if best is None or f > best[0]:
                best = (f, (self.s(a), self.s(b)))
        front = front or [best[1]]
        # (the hull's front edges are consecutive round it, their stretches one: it starts at the edge no other ends at)
        ends = {round(b, 6) for _a, b in front}
        self.f0 = next((a for a, _b in front if round(a, 6) not in ends), front[0][0])
        self.f1 = max((b for _a, b in front), key=lambda b: (b - self.f0) % self.L)
        self._opt, self._pk = {}, None

    def s(self, p):
        """the walk's coordinate of the outline's point nearest p"""
        from shapely.geometry import Point
        return float(self.E.project(Point(p)))

    def pt(self, s):
        q = self.E.interpolate(s % self.L)
        return (float(q.x), float(q.y))

    def in_front(self, s):
        return (s - self.f0) % self.L <= (self.f1 - self.f0) % self.L + 1e-9

    def unroll(self, s, c):
        """s unrolled at the cut c"""
        return (s - c) % self.L

    def side(self, s, c):
        """the class of a lane entering at s with the cut at c: 'W' on the front (the trunk's), 'N' before it (round
        the north), 'S' after it"""
        if self.in_front(s):
            return 'W'
        return 'N' if self.unroll(s, c) < self.unroll(self.f0, c) else 'S'

    def far_cut(self, c):
        """the cut stands under a hull edge turned away from the source: the rings meet behind the group, each going
        round its own side, as a single destination's far-face cut"""
        from shapely.geometry import LineString, Point
        p = Point(self.pt(c))
        hc = list(self.hull.exterior.coords)
        best = min(range(len(hc) - 1), key=lambda i: LineString([hc[i], hc[i + 1]]).distance(p))
        a, b = hc[best], hc[best + 1]
        el = math.hypot(b[0] - a[0], b[1] - a[1]) or 1.0
        # (the hull ring's own sense may be either: its outward normal is the one pointing away from the hull's centre)
        nx, ny = -(b[1] - a[1]) / el, (b[0] - a[0]) / el
        c_ = self.hull.centroid
        if nx * ((a[0] + b[0]) / 2 - c_.x) + ny * ((a[1] + b[1]) / 2 - c_.y) < 0:
            nx, ny = -nx, -ny
        return nx * self.d[0] + ny * self.d[1] >= FRONT_COS - 1e-9

    def _ray(self, p, sd):
        """where a berth at p outside the outline meets it: the ray from it back along its stub `sd` (toward its pad)
        -- a berth beside a corner is on the face its stub leaves -- else its nearest point"""
        from shapely.geometry import LineString, Point
        if sd is not None:
            k = math.hypot(*sd)
            if k > 1e-9:
                far = 4.0 * max(self.bridge, 1.0)
                hit = LineString([p, (p[0] - far * sd[0] / k, p[1] - far * sd[1] / k)]).intersection(self.E)
                pts = [hit] if hit.geom_type == 'Point' else \
                    [g for g in getattr(hit, 'geoms', []) if g.geom_type == 'Point']
                if pts:
                    q = min(pts, key=lambda g: g.distance(Point(p)))
                    return float(self.E.project(q))
        return self.s(p)

    def _crossed(self, a, b):
        """the parts the straight a-b crosses"""
        from shapely.geometry import LineString, box as _box
        seg = LineString([a, b])
        return [r for r, bx in self.boxes.items() if seg.intersection(_box(*bx)).length > 1e-6]

    def _straight(self, p):
        """(exit point, the parts crossed) of the straight from p toward the source through the outline: its first
        stretch inside it -- from p, or from where it enters just past p (a berth on a far face, outside the outline) --
        out to where it leaves"""
        from shapely.geometry import LineString, Point
        far = 2.0 * math.hypot(self.bbox[2] - self.bbox[0], self.bbox[3] - self.bbox[1]) + 1.0
        seg = LineString([p, (p[0] - far * self.d[0], p[1] - far * self.d[1])])
        ins = seg.intersection(self.U)
        parts = [ins] if ins.geom_type == 'LineString' else [g for g in getattr(ins, 'geoms', [])
                                                             if g.geom_type == 'LineString']
        P = Point(p)
        piece = min(parts, key=lambda g: g.distance(P)) if parts else None
        if piece is None or piece.length < 1e-6:
            return tuple(p), []
        cs = list(piece.coords)
        q = max(cs, key=lambda c: math.hypot(c[0] - p[0], c[1] - p[1]))
        return (float(q[0]), float(q[1])), self._crossed(p, q)

    def options(self, p, sd, part, layer, via):
        """[(way, place s, its mouth (a, b) or None, its length, its pocket (its mouths) or None, the pins its leg
        passes)] -- the ways a lane whose berth stands at p (its stub `sd` out of its pad on part
        `part`, on `layer`, `via` whether it carries one) enters the group, the shortest first, GROUP_OPTIONS of them.
        'on': its berth on the outline (s where its stub meets it); 'under': straight under its own part from the side
        turned from the source, out where that straight meets the outline; and a berth in a POCKET -- 'mouth' out of
        each of the pocket's mouths, its place there the shallowest berth nearest its own wall, and 'straight' under
        the part in front of it. A way under parts needs a layer open on every part it passes -- the berth's layer, or
        any with a via at the berth. Its length: from the berth to its place (a mouth's: the berth's depth from it);
        a mouth's leg runs along the part's row, past the pins between its berth and the mouth (GROUP_LEG_PASS each, in
        the ends model: deep in a strip a lane's leg past its neighbours' stubs and vias met them, where straight under
        the part in front -- a via at the pin, as a human takes them -- runs clear)"""
        key = (round(p[0], 6), round(p[1], 6), None if sd is None else (round(sd[0], 6), round(sd[1], 6)), part,
               layer, bool(via))
        if key in self._opt:
            return self._opt[key]
        from shapely.geometry import LineString, Point
        P = Point(p)
        out = []

        def under_ok(parts):
            if not parts:
                return True
            op = frozenset.intersection(*[self.open[r] for r in parts])
            return bool(op) and (via or layer in op)
        if not (self.U.contains(P) and self.E.distance(P) > GROUP_TOL):
            out.append(('on', self._ray(p, sd), None, 0.0, None, 0))
            if sd is not None and math.hypot(*sd) > 1e-9:
                k = math.hypot(*sd)
                if (sd[0] * self.d[0] + sd[1] * self.d[1]) / k >= FRONT_COS:
                    q, crossed = self._straight(p)
                    if part in crossed and under_ok(crossed):
                        qs = self.s(q)
                        # (out into the open: a notch's wall or the front -- not a pocket behind another part)
                        if self.E.distance(Point(q)) <= GROUP_TOL:
                            out.append(('under', qs, None, math.hypot(q[0] - p[0], q[1] - p[1]), None, 0))
        else:
            # (a berth inside the outline: its stub ends in a pocket, or on its own part's copper -- marched out of
            # the copper along the stub to where it stands)
            q0 = p
            if self.parts_u.contains(P) and sd is not None and math.hypot(*sd) > 1e-9:
                k = math.hypot(*sd)
                for i in range(1, 200):
                    t = (sd[0] / k * 0.05 * i, sd[1] / k * 0.05 * i)
                    if not self.parts_u.contains(Point(p[0] + t[0], p[1] + t[1])):
                        q0 = (p[0] + t[0], p[1] + t[1])
                        break
            Q0 = Point(q0)
            if not (self.U.contains(Q0) and self.E.distance(Q0) > GROUP_TOL):
                out.append(('on', self.s(q0), None, 0.0, None, 0))
            else:
                if self._pk is None:
                    pocket = self.U.difference(self.parts_u)
                    self._pk = [pocket] if pocket.geom_type == 'Polygon' else \
                        [g for g in getattr(pocket, 'geoms', []) if g.geom_type == 'Polygon']
                pk = next((g for g in self._pk if g.distance(Q0) < 1e-6), None)
                if pk is None:
                    out.append(('on', self.s(q0), None, 0.0, None, 0))
                else:
                    from shapely.ops import linemerge
                    edge = pk.boundary.intersection(self.E.buffer(1e-6))
                    segs = [edge] if edge.geom_type == 'LineString' else \
                        [g for g in getattr(edge, 'geoms', []) if g.geom_type == 'LineString']
                    if len(segs) > 1:
                        m_ = linemerge(segs)
                        segs = [m_] if m_.geom_type == 'LineString' else list(m_.geoms)
                    own = self.boxes[part]
                    mouths = []
                    for m in segs:
                        if m.length < GROUP_TOL:
                            continue
                        a, b = m.coords[0], m.coords[-1]
                        sa, sb = self.s(a), self.s(b)
                        if (sb - sa) % self.L > self.L / 2:
                            a, b, sa, sb = b, a, sb, sa
                        mouths.append(((float(a[0]), float(a[1])), (float(b[0]), float(b[1])), sa, sb, m))
                    pkey = tuple(sorted((a, b) for a, b, _sa, _sb, _m in mouths))     # (the pocket: its mouths)
                    # (a mouth's leg runs along its part's row at the stub ends, past its neighbours' escapes: the pins
                    # it passes, each a pad of the part within a stub's length of it -- its own the nearest)
                    pads_ = [(pd.global_x, pd.global_y) for pd in self.pcb.footprints[part].pads]
                    own_pad = min(pads_, key=lambda c: math.hypot(c[0] - p[0], c[1] - p[1]))
                    stub_ = math.hypot(own_pad[0] - p[0], own_pad[1] - p[1])
                    for a, b, sa, sb, m in mouths:
                        depth = float(m.distance(Q0))
                        dbox = lambda c: math.hypot(max(0.0, own[0] - c[0], c[0] - own[2]),
                                                    max(0.0, own[1] - c[1], c[1] - own[3]))
                        s_ = (sa + GROUP_EPS * depth) if dbox(a) <= dbox(b) else (sb - GROUP_EPS * depth)
                        mp = m.interpolate(m.project(Q0))
                        leg = LineString([p, (mp.x, mp.y)])
                        passes = sum(1 for c in pads_ if c != own_pad and leg.distance(Point(c)) <= stub_ + 0.05)
                        out.append(('mouth', s_ % self.L, (a, b), depth, pkey, passes))
                    q, crossed = self._straight(q0)
                    if crossed and under_ok(crossed) and self.E.distance(Point(q)) <= GROUP_TOL:
                        out.append(('straight', self.s(q), None, math.hypot(q[0] - p[0], q[1] - p[1]), pkey, 0))
        out.sort(key=lambda o: (o[3] + 2.0 * GROUP_LEG_PASS * o[5], o[0]))
        self._opt[key] = out[:max(1, GROUP_OPTIONS)]
        return self._opt[key]

    def cuts(self, places):
        """[(index into the sorted places, the cut's s)]: for each gap between two neighbouring places round the
        walk, the middle of its longest stretch off the front (none: no cut there)"""
        vs = sorted(places)
        n, out = len(vs), []
        gl = (self.f0 - self.f1) % self.L           # (off the front: from f1 on round to f0)
        for i in range(n):
            a, b = vs[i - 1] if i else vs[-1] - self.L, vs[i]
            best = None
            for k in (-2, -1, 0, 1):
                x0, x1 = max(a, self.f1 + k * self.L), min(b, self.f1 + gl + k * self.L)
                if x1 - x0 > 1e-9 and (best is None or x1 - x0 > best[1] - best[0]):
                    best = (x0, x1)
            if best is not None:
                out.append((i, ((best[0] + best[1]) / 2) % self.L))
        return out


    def units(self, opts, lanes):
        """the units Group.arrange searches, each a list of alternatives ({lane: its option's index}): each POCKET's
        lanes together -- in their order along it, the ones nearest its first mouth out of it, the ones nearest the
        other out of that, the rest straight under, every such split (a lane leaving by a mouth past one going straight
        under would cross it in the pocket, which no place on the walk shows) -- and each other lane with more than one
        way alone"""
        pockets = collections.defaultdict(list)
        single = []
        for l_ in lanes:
            pk = next((o[4] for o in opts[l_] if o[4] is not None), None)
            if pk is not None:
                pockets[pk].append(l_)
            elif len(opts[l_]) > 1:
                single.append(l_)
        units = []
        for _key, mem in sorted(pockets.items()):
            mouths = list(_key)
            # (along the pocket: by depth from its first mouth -- a lane without a way out of it the deepest)
            dep = lambda l_: next((o[3] for o in opts[l_] if o[0] == 'mouth' and o[2] == mouths[0]), math.inf)
            mem = sorted(mem, key=lambda l_: (dep(l_), l_))
            alts, k = [], len(mem)
            for g in range(k + 1):
                for o_ in range(k - g + 1):
                    ch = {}
                    for j, l_ in enumerate(mem):
                        want = mouths[0] if j < g else (mouths[-1] if j >= k - o_ else None)
                        idx = next((i for i, o in enumerate(opts[l_]) if (o[0] == 'straight' if want is None else
                                                                          o[0] == 'mouth' and o[2] == want)), None)
                        if idx is None:
                            break
                        ch[l_] = idx
                    else:
                        alts.append(ch)
            units.append(alts)
        for l_ in single:
            units.append([{l_: i} for i in range(len(opts[l_]))])
        return units

    def score(self, opts, rank, ch):
        """(crossings, the cut s): the lanes' places with each its option `ch` [lane], unrolled at the cut off the
        front that crosses the fewest against the launch order (`rank` {lane: its index in it}; a tie of places in
        that order). Every cut at once: moving the first lane of an unrolling to its end changes its crossings by the
        lanes launched after it less those launched before"""
        import numpy as np
        lanes = sorted(rank, key=rank.get)
        n = len(lanes)
        pl = {l_: opts[l_][ch[l_]][1] for l_ in lanes}
        order = sorted(lanes, key=lambda l_: (pl[l_], rank[l_]))
        a = np.array([rank[l_] for l_ in order])
        cuts = self.cuts([pl[l_] for l_ in order])
        if not cuts:
            return (math.inf, None)
        inv0 = int(np.triu(a[:, None] > a[None, :], 1).sum())
        cum = np.concatenate(([0], np.cumsum(n - 1 - 2 * a)))
        best = min(cuts, key=lambda ic: (inv0 + int(cum[ic[0]]), ic[0]))
        return (inv0 + int(cum[best[0]]), best[1])

    def best_cut(self, opts, lkey, ch):
        """the cut s that crosses the fewest with the lanes' options `ch` (Group.score)"""
        lanes = sorted(opts, key=lambda l_: lkey[l_])
        return self.score(opts, {l_: i for i, l_ in enumerate(lanes)}, ch)[1]

    def arrange(self, opts, lkey, start=None):
        """(choice {lane: its option's index}, the cut s, crossings): the lanes' ways in and the cut that cross the
        fewest lanes against the launch order (`lkey` {lane: its sort key round the source}) -- `opts` {lane: its
        options, Group.options}. The search: from each lane's shortest way -- a pocket's shortest split (or `start`) --
        each POCKET's lanes taken
        together (in their order along it: the ones nearest one mouth out of it, the ones nearest the other out of
        that, the rest straight under, every split) and each other lane with more than one way alone, in turn, a
        change kept when it crosses fewer, until none does"""
        lanes = sorted(opts, key=lambda l_: lkey[l_])
        rank = {l_: i for i, l_ in enumerate(lanes)}
        choice = dict(start or {l_: 0 for l_ in lanes})
        evaluate = lambda ch: self.score(opts, rank, ch)
        units = self.units(opts, lanes)
        if start is None:
            # (each pocket from its shortest split: a lane's own shortest way may cross another's in the pocket)
            for alts in units:
                if alts and len(alts[0]) > 1:
                    choice.update(min(alts, key=lambda a_: sum(opts[l_][i][3] for l_, i in a_.items())))
        best = evaluate(choice)
        for _pass in range(8):
            moved = False
            for alts in units:
                for alt in alts:
                    c2 = {**choice, **alt}
                    if c2 == choice:
                        continue
                    v = evaluate(c2)
                    if v[0] < best[0]:
                        best, choice, moved = v, c2, True
            if not moved:
                break
        return choice, best[1], best[0]


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
    # (DEST may name several parts, one source and several ends: ONE destination, the GROUP of them -- Group)
    dests, dest_all = bd.dest_refs(dest), dest
    dest = dests[0]
    lanes = [n for n in ctx.tooth_layer if n in ctx.paths and ctx.ends[n][2] in dests]
    src = collections.Counter(ctx.src_ref[n] for n in lanes).most_common(1)[0][0]
    F.M = [n for n in lanes if ctx.src_ref[n] == src]
    F.src, F.dst = src, dest
    F.dests = dests
    F.dest_of = {n: ctx.ends[n][2] for n in F.M}
    F.tooth = {n: tuple(map(float, ctx.ends[n][0])) for n in F.M}
    F.bend = {n: tuple(map(float, ctx.ends[n][1])) for n in F.M}
    SB = grown(box_of(pcb, src), F.tooth.values())
    # (each destination part's pad box grown by its own lanes' berths; a GROUP of them, one destination: its box the
    # parts' copper's, its shape and order the group's walk -- Group)
    F.DBs = {r: grown(box_of(pcb, r), [F.bend[n] for n in F.M if F.dest_of[n] == r]) for r in dests}
    _sraw = box_of(pcb, src)
    F.group = Group(pcb, dests, ((_sraw[0] + _sraw[2]) / 2, (_sraw[1] + _sraw[3]) / 2)) if len(dests) > 1 else None
    DB = F.group.bbox if F.group else F.DBs[dest]
    F.SB, F.DB = SB, DB                     # the arrays' pad boxes, grown to the line their stubs end on
    F.dpads = [(p.global_x, p.global_y) for r in dests for p in pcb.footprints[r].pads]   # the destination's pads
    # ---- the source side: the face each tooth leaves by (its escape's: tooth_face). A tooth on the far (west) face
    # goes round the source on a LEAD-IN to a start beside the side face of its side of the far face's cut (lead_ins,
    # below); one standing inside the array (a ball's via whose lane has not left it) has no way round, and is refused
    td_ = getattr(ctx, 'tooth_dir', None) or {}
    tf = {n: face(F.tooth[n], SB) for n in F.M}
    for n in F.M:
        # (its escape's face decides only where the far face is in question: the other faces as they always read)
        f_ = tooth_face(F.tooth[n], SB, td_.get(n))
        if 'W' in (f_, tf[n]):
            tf[n] = f_
    far = sorted(n for n in F.M if tf[n] == 'W')
    far_in = [n for n in far if F.tooth[n][0] >= _sraw[0] - 1e-6]
    if far_in:
        raise FarFace(far_in, f'whole route: {", ".join(far_in)} launch from the source\'s far face inside the array '
                              f'-- no way round the source from there')
    # ---- the trunk: straight, from the source's pad box through the destination's
    ca = np.array([(SB[0] + SB[2]) / 2, (SB[1] + SB[3]) / 2])
    cb = np.array([(DB[0] + DB[2]) / 2, (DB[1] + DB[3]) / 2])
    d = (cb - ca) / np.hypot(*(cb - ca))
    back = math.hypot(SB[2] - SB[0], SB[3] - SB[1]) / 2 + bd.LPITCH
    fwd = math.hypot(DB[2] - DB[0], DB[3] - DB[1]) / 2 + bd.LPITCH
    if F.group:
        # (a GROUP's square to the source's face toward it: the flow frame's own axis. The group's centre stands in
        # the space between its parts, and a trunk to it leant 11 degrees off that face: the lanes from the face to
        # the parts beside the trunk then crossed it at slopes to 4.9, past the 4 a sweep may take -- square, 2.4)
        d = np.array([1.0, 0.0])
        cb = np.array([DB[2], ca[1]])
        fwd = bd.LPITCH
    F.spine = cr.Spine([tuple(ca - d * back), tuple(cb + d * fwd)])
    F.mid = {n: in_frame(F.spine, ctx.paths[n]) for n in F.M}
    # ---- the destination side: classes, the far face's cut, the berths' order
    x0, y0, x1, y1 = DB
    F.PM = PM = list(F.M)                   # (the lanes round the destination: every lane, a group's too)
    prs = getattr(ctx, 'pairs', None) or {}
    F.gland = {}                            # a group lane's landing short of its berth: the mouth of its pocket
    if F.group:
        G = F.group

        def _via_at_berth(n_):
            B_ = F.DBs[F.dest_of[n_]]
            return all(any(v.net_id == ctx.byname[leg][0] and via_in_box(v, B_) for v in ctx.base_vias)
                       for leg in (prs[n_] if n_ in prs else (n_,)))
        opts = {n: G.options(F.bend[n], ctx.stub_dir.get(n), F.dest_of[n], ctx.dest_layer.get(n), _via_at_berth(n))
                for n in F.M}
        lkey = {n: (perim_s(F.tooth[n], SB, getattr(ctx, 'src_cut', None)), F.M.index(n)) for n in F.M}
        # the ends model's ways in and cut (braid.setup, from the plan sidecar: each lane's place, the cut's point),
        # each taken as the option nearest it; else the group's own arrangement
        ent, dc_ = getattr(ctx, 'dest_entry', None) or {}, getattr(ctx, 'dest_cut', None)
        if ent and all(n in ent for n in F.M):
            ch = {n: min(range(len(opts[n])), key=lambda i, n=n: math.dist(G.pt(opts[n][i][1]), ent[n])) for n in F.M}
        else:
            ch = G.arrange(opts, lkey)[0]
        dcut = G.s(dc_[1:]) if isinstance(dc_, (tuple, list)) and dc_[0] == 'P' else G.best_cut(opts, lkey, ch)
        F.way = {n: opts[n][ch[n]] for n in F.M}
        F.dcut = dcut
        F.cls = {}
        for n in F.M:
            f_ = G.side(F.way[n][1], dcut)
            if f_ in ('N', 'S') and n not in _trunk:
                F.cls[n] = f_
        F.P = {n: G.unroll(F.way[n][1], dcut) for n in F.M}
        # (the lanes leaving one mouth land across it in their order along the walk, a lane's room apart at most)
        by_m = collections.defaultdict(list)
        for n in F.M:
            if F.way[n][0] == 'mouth':
                by_m[F.way[n][2]].append(n)
        for (a, b), mem in by_m.items():
            mem.sort(key=lambda n: (F.P[n], F.M.index(n)))
            k_ = len(mem)
            for i, n in enumerate(mem):
                t = (i + 1) / (k_ + 1)
                F.gland[n] = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        F.pcut = {r: cut([F.bend[n][1] for n in F.M if F.dest_of[n] == r and face(F.bend[n], F.DBs[r]) == 'E'],
                         F.DBs[r][1], F.DBs[r][3]) for r in dests}
    else:
        bf = {n: face(F.bend[n], DB) for n in PM}
        dcut = cut([F.bend[n][1] for n in PM if bf[n] == 'E'], y0, y1)
        if getattr(ctx, 'dest_cut', None) is not None:
            # the ends model's own cut (braid.setup, from the plan sidecar): recomputed here from the laid stubs, which
            # stand a hair off the menu's exits, two near-equal gaps could split the far face the other way, and the
            # solve would face crossings the model never priced (a K44 bus: 176 crossings planned, 344 solved, no plan
            # proved). A cut off the far face (whole_ends' WINDING) sends the berths past it round the other side
            dcut = ctx.dest_cut
        F.dcut = dcut
        F.cls = {}
        for n in PM:
            f_ = ring_side(F.bend[n], DB, dcut) or 'W'
            if f_ in ('N', 'S') and n not in _trunk:
                F.cls[n] = f_
        F.P = {n: perim_c(F.bend[n], DB, dcut) for n in F.M}
        F.pcut = {dest: dcut}
    near = [n for n in PM if n not in F.cls]

    def perim_berth(n_, p):
        """where berth point p of lane n_ lies round its own part's box, unrolled at that box's cut (a pair's tips
        stand on one part)"""
        b_ = F.dest_of[n_]
        return perim_c(p, F.DBs[b_], F.pcut[b_])
    # (north to south round the destination; a stack at a berth -- a tie, tie_rank -- in its lanes' order at the
    # source, round the source's box unrolled at its middle: the far face's cut is not known yet, and the order within
    # one stack is all it decides)
    _bt = tie_rank(F.P)
    F.final = sorted(F.M, key=lambda n: (_bt[n], perim_s(F.tooth[n], SB), F.M.index(n)))
    # the source's far face's CUT (src_cut): the ends model's (the plan sidecar's, as the destination's), else the gap
    # between its teeth that makes the fewest crossings with the berths' order
    fi_ = {n: i for i, n in enumerate(F.final)}

    def _xings(c_):
        _tr = tie_rank({n: perim_s(F.tooth[n], SB, c_) for n in F.M})
        k_ = sorted(F.M, key=lambda n: (_tr[n], fi_[n]))
        return sum(1 for i, a in enumerate(k_) for b in k_[i + 1:] if fi_[a] > fi_[b])
    scut = getattr(ctx, 'src_cut', None)
    if scut is None:
        scut = src_cut([F.tooth[n][1] for n in far], SB[1], SB[3], rank=_xings if far else None)
    F.scut = float(scut)
    F.far = far
    for n in far:
        tf[n] = 'N' if F.tooth[n][1] <= F.scut else 'S'            # (the side face it goes round to)
    _prs0 = getattr(ctx, 'pairs', None) or {}
    import pairs as _pairs0
    _run_ids = {ctx.byname[n][0] for n in F.M} | {ctx.byname[g][0] for n in F.M for g in _prs0.get(n, ())
                                                  if g in ctx.byname}
    F.lead = lead_ins(F, far, F.scut, lambda n: bd.LANE_MIN + (_pairs0.pitch(bd.TRACK) if n in _prs0 else 0.0), SB,
                      obst=lead_obstacles(ctx, src, dest, SB, _run_ids) if far else (),
                      layer=lambda n: ctx.tooth_layer.get(n))
    # the REFERENCE paths (the geometry picks each column's free interval nearest one): a lane's taut path, except
    # where it runs INSIDE a pad box -- a taut string has no width and threads gaps between balls no track fits --
    # where it takes the box's edge on the lane's own side (a ring's at the destination, its tooth's face's at the
    # source) a ring margin out: SA0 was bounded south of the destination's corner, its handoff north of it
    for n in F.M:
        side = {}
        path_n = ctx.paths[n]
        if n in F.lead:
            # (a far-face lane: from the end of its lead-in along its side face, at its own offset, to past the near
            # face's corner, and on from where its taut path is past it -- the taut path itself may round the source
            # the other way, and a lane started beside the north face was drawn back down the far face to it)
            st_ = F.lead[n][-1]
            cx_ = SB[2] + bd.LPITCH
            s_c = float(F.spine.project_pt((cx_, st_[1]))[0])
            path_n = [tuple(st_), (cx_, st_[1])] + [p for p in _dense(ctx.paths[n], bd.LPITCH / 2)
                                                   if float(F.spine.project_pt(p)[0]) > s_c + bd.LPITCH]
        elif tf[n] in ('N', 'S'):
            side['src'] = tf[n]
        if n in F.cls:
            side['dst'] = F.cls[n]          # (a ring's lane passes the destination on its ring's side)
        if not side and n not in F.lead:
            continue
        pts = []
        # densified first: a taut string's straight between two corners runs through a box too (K28 SCKE0 from its
        # south-face tooth straight to a corner past the source's east face), and only its points can be moved
        for (x, y) in _dense(path_n, bd.LPITCH / 2):
            for key_, b_ in (('src', SB), ('dst', DB)):
                if key_ in side and b_[0] < x < b_[2] and b_[1] < y < b_[3]:
                    y = b_[1] - bd.LPITCH if side[key_] == 'N' else b_[3] + bd.LPITCH
            pts.append((x, y))
        F.mid[n] = in_frame(F.spine, pts)
    F.se = {n: tuple(map(float, F.spine.project_pt(F.gland.get(n, F.bend[n])))) for n in near}
    F.H0 = (min(F.se[n][0] for n in near) if near else float(F.spine.project_pt((x0, (y0 + y1) / 2))[0]))
    W_, H_ = x1 - x0, y1 - y0
    sW, sH = SB[2] - SB[0], SB[3] - SB[1]
    perim_src = lambda p: perim_s(p, SB, F.scut)
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
        box = F.SB if k_ == 0 else F.DBs[F.dest_of[n_]]
        for leg in (prs[n_] if n_ in prs else (n_,)):
            nid = ctx.byname[leg][0]
            if not any(v.net_id == nid and via_in_box(v, box) for v in ctx.base_vias):
                return False
        return True
    # every lane with an end between a pair's two tips, whatever its layer there: a solve that chooses ends' layers
    # (whole_solve's via ends, on more routing layers than two) keeps each off the pair's
    F.tip_between = {}
    for k_ in (0, 1):
        lay_ = ctx.tooth_layer if k_ == 0 else ctx.dest_layer
        at = []                         # (position round the box, lane, its layer there, the box: a berth's its part's)
        for n in F.M:
            tips = ctx.pair_ends[n][k_] if n in prs else (ctx.ends[n][k_],)
            g_ = None if k_ == 0 else F.dest_of[n]
            at += [((perim_src if k_ == 0 else lambda p_, n_=n: perim_berth(n_, p_))(tuple(map(float, t))), n, lay_[n],
                    g_) for t in tips if t is not None]
        for n in prs:
            vs = sorted(v for v, o, _L, _g in at if o == n)
            if n not in F.M or len(vs) < 2:
                continue
            gn = None if k_ == 0 else F.dest_of[n]
            B_ = SB if k_ == 0 else F.DBs[gn]
            whole = 2 * (B_[2] - B_[0] + B_[3] - B_[1])
            F.tip_between[(n, k_)] = sorted({o for v, o, _L, g in at if o != n and g == gn
                                             and between(vs[0], vs[-1], v, whole)})
            inside = sorted({o for v, o, L, g in at if o != n and g == gn and L == lay_[n]
                             and between(vs[0], vs[-1], v, whole) and not (via_end(n, k_) or via_end(o, k_))})
            if inside:
                split.append(f'{n} at its {("tooth", "berth")[k_]} (round {", ".join(inside)})')
                splits.append((n, ('tooth', 'berth')[k_], inside))
    if split:
        raise PairSplit(splits, f'whole route: pair(s) split by other lanes -- {"; ".join(split)} -- a pair is one lane')
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
    F.land.update(F.gland)
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
    # ...and a far-face lane's: the end of its LEAD-IN round the source's corner (lead_ins, above), drawn as it is
    for n, pts_ in F.lead.items():
        F.start[n] = tuple(map(float, pts_[-1]))
    F.st = {n: tuple(map(float, F.spine.project_pt(F.start[n]))) for n in F.M}
    # (a stack at the source in its lanes' order at their berths)
    _ti = tie_rank({n: perim_src(F.tooth[n]) for n in F.M})
    _fi = {n: i for i, n in enumerate(F.final)}
    F.launch = sorted(F.M, key=lambda n: (_ti[n], _fi[n]))
    # ---- the rings
    dpads = F.dpads
    cen = (sum(p[0] for p in dpads) / len(dpads), sum(p[1] for p in dpads) / len(dpads))
    north = 1.0 if F.spine.xy(F.H0, 1.0)[1] < F.spine.xy(F.H0, 0.0)[1] else -1.0   # the sign of o toward the north
    width = lambda n: bd.LANE_MIN + (_pairs.pitch(bd.TRACK) if n in (getattr(ctx, 'pairs', None) or {}) else 0.0)
    F.o_h = {n: F.se[n][1] for n in near}
    tails = [tuple(map(float, F.spine.xy(F.H0, F.o_h[n]))) for n in near] + [F.bend[n] for n in near]
    # the rings go outside the other parts' copper that hugs the destination too (a decoupling cap at its corner): a
    # pad within a lane pitch and two lanes' room of the hull is one more thing the ring goes round, its corners in it
    base = dpads + [F.bend[n] for n in PM] + tails
    for ref_, fp_ in pcb.footprints.items():
        if ref_ in (src, *dests):
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
        return build(ctx, dest_all, _trunk | frozenset(short))
    return F


def _box_run(a, b, box, n=64):
    """the length of the straight a->b inside box (whole_ends._box_run)"""
    k = 0
    for j in range(n):
        t = (j + 0.5) / n
        x, y = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
        if box[0] < x < box[2] and box[1] < y < box[3]:
            k += 1
    return k / n * math.hypot(b[0] - a[0], b[1] - a[1])


def _round_corner(p0, p1, box, tol=0.05):
    """the corner of `box` a lane from p0 to p1 goes round where its straight runs through the box -- the shorter
    detour whose two legs clear it (whole_ends.corner_slope's) -- None where the straight is clear, or no corner is"""
    if _box_run(p0, p1, box) < tol:
        return None
    best = None
    for c in ((box[0], box[1]), (box[0], box[3]), (box[2], box[1]), (box[2], box[3])):
        if _box_run(p0, c, box) > tol or _box_run(c, p1, box) > tol:
            continue
        d = math.hypot(c[0] - p0[0], c[1] - p0[1]) + math.hypot(p1[0] - c[0], p1[1] - c[1])
        if best is None or d < best[0]:
            best = (d, c)
    return None if best is None else best[1]


def _hk_floor(F, k, mem, sg, s0, o_start, hull):
    """(probe FRAME_HK_FLOOR) ring k's start along the trunk no earlier than its lanes that go round the SOURCE's corner
    can reach their places on its stack: the corner's s, and the climb from the corner's offset to the lane's stack
    offset at 45 degrees (a lane pitch more) -- the ring started at the handoff line alone, the zynq LVDS's S ring 0.5 mm
    of route past U1's south-east corner, and the E-face lanes rounding it paid 2 mm climbing onto it (whole_ends'
    ENDS_CORNER already prices the lanes as arriving there). Capped short of the hull's outermost point on the ring's
    side (the ring must start before the corner it wraps) and outside the hull"""
    best, why = s0, None
    for n in mem:
        c_ = _round_corner(F.start[n], F.spine.xy(s0, F.o_h[n]), F.SB)
        if c_ is None:
            continue
        sc_, oc_ = (float(v) for v in F.spine.project_pt(c_))
        need_ = sc_ + abs(F.o_h[n] - oc_) + bd.LANE_MIN
        if need_ > best:
            best, why = need_, (n, c_)
    if why is None:
        return s0
    far_ = max(hull, key=lambda p_: sg * float(F.spine.project_pt(p_)[1]))
    cap_ = float(F.spine.project_pt(far_)[0]) - bd.LANE_MIN
    s1 = min(best, cap_)
    while s1 > s0 and _inside(F.spine.xy(s1, o_start), hull, 2 * bd.LPITCH):
        s1 -= bd.LANE_MIN / 5
    s1 = max(s0, s1)
    print(f'   ring {k} start floored {s0:.2f} -> {s1:.2f} (wanted {best:.2f} for {why[0]} round the source corner '
          f'({why[1][0]:.2f}, {why[1][1]:.2f}); cap {cap_:.2f})')
    return s1


def _ring_start(F, k, near, s0, o_start, hull, side=None):
    """(probe FRAME_RING_START=1, or =2 with `side` the ring's sign of o: the lanes berthing on the ring's own side of
    the trunk alone, the ones its lanes stack outside of) ring k's start along the trunk a lane pitch past the LAST
    lane the trunk berths: the facing face stands across the spine at a tilt, so its berths lie along the trunk from
    the first (H0, where both rings started) some way on, and a ring started at H0 left the trunk while trunk lanes
    still ran beside it to their berths, the two frames keeping no order between their lanes -- and on the tilted
    handoff line a ring's stack stood behind the source's corner (zynq LVDS K47: the S ring's 2.4 mm behind U1's east
    face, its lanes off that face climbing at slopes of 8 to 10 past the corner). Moved back along the trunk until its
    start clears the hull, as the start at H0 is"""
    on = [n for n in near if side is None or side * F.se[n][1] > 0]
    if not on:
        return s0
    want = max(F.se[n][0] for n in on) + bd.LANE_MIN
    s1 = want
    while s1 > s0 and _inside(F.spine.xy(s1, o_start), hull, 2 * bd.LPITCH):
        s1 -= bd.LANE_MIN / 5
    s1 = max(s0, s1)
    if s1 > s0 + 1e-9:
        print(f'   ring {k} start past the trunk\'s last berth {s0:.2f} -> {s1:.2f} (wanted {want:.2f})')
    return s1


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
    PM = F.PM
    hull = dpads + [F.bend[n] for n in PM] + tails
    rpad = max((max(p.size_x or 0.0, p.size_y or 0.0) / 2 for r in F.dests for p in pcb.footprints[r].pads),
               default=0.0)
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
    wound = not (F.group.far_cut(F.dcut) if F.group else far_cut(F.dcut))
    for k in sorted(set(F.cls.values())):
        mem = [n for n in F.M if F.cls.get(n) == k]
        sg = north if k == 'N' else -north
        stubs = [F.gland.get(n, F.bend[n]) for n in mem]
        # the ring's lead-in runs taut from its start to where it grazes the hull (corridor.build_wrap_spine), so the
        # start stands OUTSIDE the hull: a lane pitch inside the innermost ring lane on the arrival line, moved back
        # along the trunk until it clears the hull by two lane pitches
        o_start = F.o_h[min(mem, key=lambda n: sg * F.o_h[n])] - sg * bd.LPITCH
        s_start = F.H0
        hull = dpads + stubs + tails
        while _inside(F.spine.xy(s_start, o_start), hull, 2 * bd.LPITCH) and s_start > 0:
            s_start -= bd.LANE_MIN / 5
        if (awx_settings.get('FRAME_HK_FLOOR') or '0') == '1':
            s_start = _hk_floor(F, k, mem, sg, s_start, o_start, hull)
        if (awx_settings.get('FRAME_RING_START') or '0') in ('1', '2'):
            s_start = _ring_start(F, k, near, s_start, o_start, hull,
                                  side=sg if awx_settings.get('FRAME_RING_START') == '2' else None)
        ccw = sum(bd.sweep_round(ctx.paths[n], cen) for n in mem) > 0
        if wound or F.group:
            # (a cut off the far face: a ring's lanes past the far face are WOUND round it, their taut paths the short
            # way round the other side -- the ring runs the way its class goes, from the facing face round its side.
            # So round a GROUP always: a taut path runs between and through its parts, its sweep round their centre
            # no side of anything -- the back row's lanes swept the north ring backwards, into a 9 mm hook)
            DB_ = F.DB
            wn = ((DB_[0], (DB_[1] + DB_[3]) / 2), ((DB_[0] + DB_[2]) / 2, DB_[1]), (DB_[2], (DB_[1] + DB_[3]) / 2))
            ccw = (bd.sweep_round(list(wn), cen) > 0) == (k == 'N')
        F.Hk[k] = float(s_start)                         # this ring's lanes leave the trunk here
        _core, F.rings[k] = bd.ring_spine(dpads, [F.spine.xy(s_start, F.o_h[n]) for n in mem], stubs,
                                          F.spine.xy(s_start, o_start), ccw, tuple(float(v) for v in F.spine.d[-1]),
                                          tails, wrap_side=wound)
    hits, room = {}, {}
    for k in F.rings:
        mem = [n for n in F.M if F.cls.get(n) == k]
        sg = north if k == 'N' else -north
        o_of = lambda p_: sg * F.spine.project_pt(p_)[1]
        # the destination's copper's farthest offset on the ring's side: its pads (their reach) and the berths' ends
        e_ = max([o_of(p_) + rpad for p_ in dpads] + [o_of(F.bend[n]) + bd.TRACK / 2 for n in PM])
        near_e = max((sg * F.se[n][1] + (width(n) - bd.LANE_MIN) / 2 for n in near), default=-bd.LANE_MIN / 2)
        room[k] = (e_, near_e, sum(width(n) for n in mem))
        out_ = max(mem, key=lambda n: sg * F.o_h[n])
        a = F.spine.xy(F.Hk[k], sg * (min(sg * F.o_h[n] for n in mem) - bd.LPITCH))     # the ring's start
        b = F.spine.xy(F.Hk[k], F.o_h[out_])                                             # its outermost lane
        reach = (width(out_) - bd.LANE_MIN) / 2 + bd.TRACK / 2 + bd.CLEAR                 # its copper and a clearance
        for ref_, fp_ in pcb.footprints.items():
            if ref_ in (src, *F.dests) or ref_ in in_hull:
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

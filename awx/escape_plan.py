"""escape_plan.py -- the ESCAPE FIELD's moves: a ball's escapes out of a ball field the way a human escapes a
DOG-BONED one, each clear of the board's copper by the fanout's own track and clearance (Copper).

A lane leaves its ball by its via (escape_menu):

  * half a pitch along its gap row onto a ball line beside the via, out along the line to the face, and a pitch (PAST)
    past the outer balls -- its EXIT, beyond the outer ring's dog-bone vias, where the whole route takes it on;
  * or a SIDESTEP: from the line beside the via along the ball row half a pitch out, to a line up to SIDE pitches
    over, then out along that one -- a deeper leg past its neighbours' vias, so a pair can take either hand;
  * or along a STREET: a pitch along its gap column, between two balls, onto the next GAP line -- the row of gap
    points between two ball rows, free of vias where the dog-bones of the rows either side point away from it (the
    human's U5: the field's two halves dog-boned out of the middle, four B.Cu lanes through it) -- and out along it;
  * on any routing layer the balls are not on (a via-in-pad -- the dog-bone that found no gap -- runs out along its
    ball's own line).

A ball with no via leaves on its own layer (escape_out): a SURFACE escape, straight along its own line or half a step
onto a gap line beside it, to the same exit.

Every such escape is a MOVE (escape_moves.Move): kind 'dogbone' (or 'via_in_pad', 'surface'), its legs the exact path,
`climb` set so the selector prices its axis-aligned runs as lanes (select_moves._lane_spans). The whole-array fanout
(joint_escape.escape_field_moves) offers them from every site a ball could take, and the group's ends
(group_ends.py) choose among them."""

import collections
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
import escape_moves as em  # noqa: E402
from relayer import TOL as _END_TOL  # noqa: E402
from kicad_parser import pad_is_plated_through  # noqa: E402

SIDE = 2          # a sidestep: up to this many pitches past the lines beside the via
PAST = 1.0        # the exit: this many pitches past the outer balls
DIRS = em.DIRS


def _pt_seg(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 < 1e-18 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def _cross(a, b, c, d):
    o = lambda p, q, r: (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    return (o(a, b, c) * o(a, b, d) < 0) and (o(c, d, a) * o(c, d, b) < 0)


def seg_seg(a, b, c, d):
    """the least distance between segments ab and cd"""
    if _cross(a, b, c, d):
        return 0.0
    return min(_pt_seg(a, c, d), _pt_seg(b, c, d), _pt_seg(c, a, b), _pt_seg(d, a, b))


class Copper:
    """The board's copper, binned per millimetre, for one question: does a track from p to q on a layer keep the
    fanout's clearance off every other net's pad on that layer (and every plated hole and NPTH drill), every via, and
    every segment on that layer"""
    CELL = 1.0

    def __init__(self, pcb, track, clear):
        from coherent_nets import namer
        self.track, self.clear = track, clear
        self.net_key = namer(pcb)           # (a net by the whole route's name for it: an escape's Move is named so)
        self.bins = collections.defaultdict(list)
        cell = lambda x: int(math.floor(x / self.CELL))
        for fp in pcb.footprints.values():
            for p in fp.pads:
                if p.pad_type == 'np_thru_hole' and not (p.drill or 0) > 0:
                    continue
                r = max(p.size_x, p.size_y) / 2 + 0.01
                for cx in range(cell(p.global_x - r), cell(p.global_x + r) + 1):
                    for cy in range(cell(p.global_y - r), cell(p.global_y + r) + 1):
                        self.bins[(cx, cy)].append(('pad', p))
        for v in pcb.vias:
            r = v.size / 2
            for cx in range(cell(v.x - r), cell(v.x + r) + 1):
                for cy in range(cell(v.y - r), cell(v.y + r) + 1):
                    self.bins[(cx, cy)].append(('via', v))
        for s in pcb.segments:
            x0, x1 = sorted((s.start_x, s.end_x))
            y0, y1 = sorted((s.start_y, s.end_y))
            r = s.width / 2
            for cx in range(cell(x0 - r), cell(x1 + r) + 1):
                for cy in range(cell(y0 - r), cell(y1 + r) + 1):
                    self.bins[(cx, cy)].append(('seg', s))
        self.cell = cell

    @staticmethod
    def _pad_dist(x, y, p):
        if p.shape == 'circle':
            return max(0.0, math.hypot(p.global_x - x, p.global_y - y) - p.size_x / 2)
        from routing_utils import point_to_pad_rect_dist
        return point_to_pad_rect_dist(x, y, p)

    def seg_clear(self, a, b, layer, nid):
        """(clear, what stopped it)"""
        g = self.track / 2 + self.clear
        reach = 1.0
        seen = set()
        cands = []
        x0, x1 = sorted((a[0], b[0]))
        y0, y1 = sorted((a[1], b[1]))
        for cx in range(self.cell(x0 - reach), self.cell(x1 + reach) + 1):
            for cy in range(self.cell(y0 - reach), self.cell(y1 + reach) + 1):
                for kind, o in self.bins.get((cx, cy), ()):
                    if id(o) not in seen:
                        seen.add(id(o))
                        cands.append((kind, o))
        n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.05))
        for kind, o in cands:
            if getattr(o, 'net_id', None) == nid and nid:
                continue
            if kind == 'via':
                if _pt_seg((o.x, o.y), a, b) < o.size / 2 + g:
                    return False, f'via {o.net_id} ({o.x:.2f},{o.y:.2f})'
            elif kind == 'seg':
                if o.layer == layer and seg_seg(a, b, (o.start_x, o.start_y), (o.end_x, o.end_y)) < o.width / 2 + g:
                    return False, f'segment {o.net_id} on {o.layer}'
            else:
                p = o
                if p.pad_type == 'np_thru_hole':
                    if _pt_seg((p.global_x, p.global_y), a, b) < (p.drill or 0) / 2 + g:
                        return False, f'hole {p.component_ref}.{p.pad_number}'
                    continue
                on = layer in p.layers or '*.Cu' in p.layers or (p.drill or 0) > 0
                if not on:
                    continue
                if _pt_seg((p.global_x, p.global_y), a, b) > max(p.size_x, p.size_y) + g:
                    continue
                for k_ in range(n + 1):
                    q = (a[0] + (b[0] - a[0]) * k_ / n, a[1] + (b[1] - a[1]) * k_ / n)
                    if self._pad_dist(q[0], q[1], p) < g:
                        return False, f'pad {p.component_ref}.{p.pad_number}'
        return True, ''

    def path_clear(self, pts, layer, nid):
        for a, b in zip(pts, pts[1:]):
            ok, _why = self.seg_clear(a, b, layer, nid)
            if not ok:
                return False
        return True


def dedupe(pts):
    """a path's points, those within the relayer's TOL of the one before merged -- the later kept, so the path still
    ends on its exit, and its first point stays the via. A leg shorter than that is no leg: the relayer, walking the
    stub back from its end, reads it and the leg before it as a fork and leaves the end's layer as the fanout laid it
    (an outer ring's dog-bone 6 nm short of the exit line: a 6 nm last leg, its lane held to that layer by the solve)"""
    out = [pts[0]]
    for q in pts[1:]:
        if math.hypot(q[0] - out[-1][0], q[1] - out[-1][1]) > _END_TOL:
            out.append(q)
        elif len(out) > 1:
            out[-1] = q
    return out


def own_via(pcb, pad, nid, grid):
    """the ball's dog-bone via: the net's via nearest the ball within a pitch (its gap point, or the ball itself for a
    via in its pad), or None"""
    reach = max(grid.pitch_x, grid.pitch_y) * 0.75
    vs = [v for v in pcb.vias if v.net_id == nid and math.hypot(v.x - pad.global_x, v.y - pad.global_y) <= reach]
    v = min(vs, key=lambda v: math.hypot(v.x - pad.global_x, v.y - pad.global_y)) if vs else None
    return (v.x, v.y) if v is not None else None


def line_step(pitch, copper):
    """THE LINES ESCAPES LEAVE ALONG, one lattice a face for every ball of the array: the ball pitch `pitch` cut in the
    most EVEN parts each at least a lane's pitch at the fanout's track -- its track, its clearance and a routing grid
    step. A ball's own line and the gap line between two balls are always on it; between them as many more lines as
    the lanes fit, so a narrower track (the fab escalation's) leaves more lanes a pitch -- where the board's copper
    lets them by (path_clear). Every escape's lines on one lattice, two runs a lattice line apart never conflict, and
    the array's rules read conflicts at its lines' points (group_ends.run_points)"""
    import rules as _rules
    tp = copper.track + copper.clear + _rules.active().grid
    return pitch / (2 * max(1, int(pitch / (2 * tp) + 1e-9)))


def lattice(reach, step):
    """the lattice's offsets from a line out to `reach` (a multiple of `step`): step, 2 step, .. reach"""
    return [step * j for j in range(1, int(round(reach / step)) + 1)]


def escape_menu(pad, site, grid, faces, layers, copper, side=SIDE, own_line=False):
    """every escape the ball at `pad` has from its via at `site`, by each of `faces`, on each of `layers` (the module's
    docstring): [Move]. `own_line`: only its runs straight out along the ball's own line. A plated through-hole pin
    has none: its barrel is on every layer already, and it leaves on each of them by its surface escape (escape_out)"""
    if pad_is_plated_through(pad):
        return []
    net = copper.net_key(getattr(pad, 'net_name', '') or '')
    nid = pad.net_id
    home = next((L for L in pad.layers if L.endswith('.Cu')), 'F.Cu')
    x0, y0, x1, y1 = grid.bbox
    hx, hy = grid.pitch_x / 2.0, grid.pitch_y / 2.0
    ball = (pad.global_x, pad.global_y)
    in_pad = math.hypot(site[0] - ball[0], site[1] - ball[1]) < 0.05
    out = []
    for d in faces:
        dx, dy = DIRS[d]
        k = 0 if dx else 1                      # the axis normal to the face
        a_ = 1 - k                              # the axis along it
        s = dx if dx else dy
        h = (hy if k == 0 else hx)              # half a pitch along the face
        hk = (hx if k == 0 else hy)             # half a pitch across it
        # (the exit a full pitch past the outer balls, PAST: the outer ring's own dog-bones stand on the gap points half
        # a pitch out, between every two ball lines -- a pair's tips there had a via between them, a split)
        e = (x0 - PAST * 2 * hx if dx < 0 else x1 + PAST * 2 * hx) if k == 0 else \
            (y0 - PAST * 2 * hy if dy < 0 else y1 + PAST * 2 * hy)
        P = lambda along, normal, a_=a_: (along, normal) if a_ == 0 else (normal, along)
        if s * (e - site[k]) < -1e-6:
            continue                            # (the via stands past this face's exit line)
        lo, hi = (y0, y1) if a_ == 1 else (x0, x1)
        st_ = line_step(2 * h, copper)
        if in_pad:
            # (its own line, and onto every line of the lattice out to the gap line either side -- escape_moves' climb
            # from a via in the pad: an earlier step's field laid such an escape for a later bus's ball, and that bus's
            # menu, its own line alone, held no move at it -- three balls whose neighbours' vias closed their own lines
            # stood open)
            lines = [(ball[a_], [site, P(ball[a_], e)])] + \
                [(ball[a_] + sg * o_, [site, P(ball[a_] + sg * o_, site[k]), P(ball[a_] + sg * o_, e)])
                 for sg in (-1, 1) for o_ in lattice(h, st_)]
        else:
            lines = []
            for sg in (-1, 1):
                ln0 = site[a_] + sg * h
                # (every line of the lattice out to the gap line a pitch over: the ball line beside the via, the
                # street -- where no via stands on it, path_clear finds which -- and, where the lanes' pitch fits more
                # than two lines a ball pitch, those between)
                for o_ in lattice(2 * h, st_):
                    ls = site[a_] + sg * o_
                    lines.append((ls, [site, P(ls, site[k]), P(ls, e)]))
                row = site[k] + s * hk
                # (a row within the relayer's TOL of the exit line IS that line, as a leg that short is no leg --
                # dedupe: an array whose balls stand a few nanometres off their grid had the outer ring's dog-bones'
                # "row" 6 to 11 nm short of the exit line, and each was offered side-steps ALONG the exit line, outside
                # the array: 354 moves of one 400-ball array, their last leg that far off its axis and so no span of
                # theirs (select_moves._lane_spans) -- a run the array's rules never saw, laid across the tips of the
                # escapes that end on that line, and the field's lessons ruled them out one state at a time)
                if s * (e - row) < _END_TOL:
                    continue                    # (no ball row between the via and the exit line to sidestep along)
                for m in range(1, side + 1):
                    ln = ln0 + sg * m * 2 * h
                    lines.append((ln, [site, P(ln0, site[k]), P(ln0, row), P(ln, row), P(ln, e)]))
        for ln, path in lines:
            # (ON THE ARRAY'S BOX WITHIN A POINT'S TOLERANCE, the relayer's TOL: the box is the outer balls'
            # coordinates and a line this ball's own -- and a line that near the ball's own line is that line. Asked
            # to the nanometre, a ball 2 nm off its grid lost the line along the box's edge, one exit of its menu
            # that its neighbours on the grid kept)
            if not (lo - _END_TOL <= ln <= hi + _END_TOL) or (own_line and abs(ln - ball[a_]) > _END_TOL):
                continue
            path = dedupe(path)
            for L in layers:
                if L == home:
                    continue
                if not copper.path_clear(path, L, nid):
                    continue
                legs = ([] if in_pad else [(ball, site, home)]) + [(p, q, L) for p, q in zip(path, path[1:])]
                mv = em.Move(net, 'via_in_pad' if in_pad else 'dogbone', d, L, P(ln, e), 1, legs, site=site,
                             climb=1)
                mv.esc_side = int(round(abs(ln - site[a_]) / (2 * h) - 0.5)) if not in_pad else 0
                mv.street = abs(abs(ln - site[a_]) - 2 * h) < 1e-6
                out.append(mv)
    return out


def escape_out(pad, grid, faces, copper, layers=None):
    """the escapes of a ball with NO via (an outer ball left to leave on its own layer): out on its own layer,
    straight along its own line or half a step onto a gap line beside it, to the exit (escape_menu's line, a pitch
    past the outer balls) -- a SURFACE escape, its run on that layer; where the lane leaves that layer is the
    route's, as the human's surface escapes dive 0.8-2.9 mm off the balls in pairs (zynq U1). A plated through-hole
    pin (a pin header's) stands on every copper layer: its surface escapes leave on each of `layers`"""
    if pad_is_plated_through(pad) and layers:
        return [mv for L in layers for mv in _escape_out(pad, grid, faces, copper, L)]
    return _escape_out(pad, grid, faces, copper,
                       next((L for L in pad.layers if L.endswith('.Cu') and not L.startswith('*')), 'F.Cu'))


def _escape_out(pad, grid, faces, copper, home):
    """escape_out's surface escapes on the one layer `home`"""
    net = copper.net_key(getattr(pad, 'net_name', '') or '')
    nid = pad.net_id
    x0, y0, x1, y1 = grid.bbox
    hx, hy = grid.pitch_x / 2.0, grid.pitch_y / 2.0
    ball = (pad.global_x, pad.global_y)
    out = []
    for d in faces:
        dx, dy = DIRS[d]
        k = 0 if dx else 1
        a_ = 1 - k
        s = dx if dx else dy
        h = (hy if k == 0 else hx)
        hk = (hx if k == 0 else hy)
        e = (x0 - PAST * 2 * hx if dx < 0 else x1 + PAST * 2 * hx) if k == 0 else \
            (y0 - PAST * 2 * hy if dy < 0 else y1 + PAST * 2 * hy)
        P = lambda along, normal, a_=a_: (along, normal) if a_ == 0 else (normal, along)
        if s * (e - ball[k]) < -1e-6:
            continue
        lo, hi = (y0, y1) if a_ == 1 else (x0, x1)
        paths = [(ball[a_], [ball, P(ball[a_], e)])]
        for sg in (-1, 1):
            for o_ in lattice(h, line_step(2 * h, copper)):
                g_ = ball[a_] + sg * o_
                paths.append((g_, [ball, P(g_, ball[k] + s * hk), P(g_, e)]))
        for ln, path in paths:
            if not (lo - h - _END_TOL <= ln <= hi + h + _END_TOL):       # (escape_menu's: within a point's tolerance)
                continue
            path = dedupe(path)
            if not copper.path_clear(path, home, nid):
                continue
            legs = [(p, q, home) for p, q in zip(path, path[1:])]
            mv = em.Move(net, 'surface', d, home, P(ln, e), 0, legs, site=None, climb=1)
            mv.esc_side, mv.street = 0, False
            out.append(mv)
    return out

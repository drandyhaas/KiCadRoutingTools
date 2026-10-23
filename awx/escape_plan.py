#!/usr/bin/env python3
"""escape_plan.py -- the ESCAPE fanout (FANOUT_ESCAPE=1): the bus's ends planned and laid as a human escapes a
DOG-BONED ball field (Andy 1010), the ends model choosing among the escapes the field has.

    python3 escape_plan.py OUT.kicad_pcb K --board=BASE      (fanout_from_plan.py hands over to it, same arguments)

THE BASE has every ball of the two arrays dog-boned already (route_bus --escape: dogbone_arrays): a short stub on the
balls' layer to a via on one of its four diagonal gap points, every ball's via on a gap point -- so the BALL LINES, the
rows and columns through the balls, are free on every layer the balls are not on. A lane leaves its ball by its via:

  * half a pitch along its gap row onto a ball line beside the via, out along the line to the face, and a pitch (PAST)
    past the outer balls -- its EXIT, beyond the outer ring's dog-bone vias, where the whole route takes it on;
  * or a SIDESTEP: from the line beside the via along the ball row half a pitch out, to a line up to SIDE pitches
    over, then out along that one -- a deeper leg past its neighbours' vias, so a pair can take either hand;
  * or along a STREET: a pitch along its gap column, between two balls, onto the next GAP line -- the row of gap
    points between two ball rows, free of vias where the dog-bones of the rows either side point away from it (the
    human's U5: the field's two halves dog-boned out of the middle, four B.Cu lanes through it) -- and out along it;
  * on any routing layer the balls are not on (a via-in-pad -- the dog-bone that found no gap -- runs out along its
    ball's own line).

Every such escape clear of the board's copper (every pad on its layer, every via, every segment on its layer, by the
fanout's own track and clearance) is a MOVE (escape_moves.Move): kind 'dogbone' (or 'via_in_pad'), its legs the exact
path, `climb` set so the selector prices its axis-aligned runs as lanes (select_moves._lane_spans). The menus feed the
ends model unchanged (whole_ends.choose, a plan state of the same keys as fanout_from_plan.plan_state's): it chooses
each lane's tooth and berth among them -- crossings round both arrays, vias, pair hands, conflicts -- with no tooth laid
before it (pages_first.current_tooth: none on an escape state). A pair's two legs are one option only on two
adjacent lines of one face and layer (whole_ends.combos), and never where its own legs pass within the rule of each
other (banned as a pair). The chosen legs are LAID AS THEY ARE -- the dog-bone stubs and vias stand on the base, so only
the runs are new copper -- and the sidecar (OUT.plan.json) records the ends as laid, the ends model's cut round the
destination, and each berth's move for an incremental round's hold. The fanout audit then refuses a board the whole
route could not take, as fanout_from_plan's does.

An INCREMENTAL round (INCREMENTAL= the previous round's sidecar, FEEDBACK= its findings) holds every berth the feedback
does not name at the move it was laid by; every tooth is free (no tooth stands on the base to hold)."""
KRT_TOOL = {'scope': [], 'kind': 'actor'}   # #937: a research tool (awx), catalogued, shown at no door

import collections
import contextlib
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
import awx_settings  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from kicad_writer import add_tracks_and_vias_to_pcb  # noqa: E402
import escape_moves as em  # noqa: E402
import route_layers  # noqa: E402
import rules as _rules  # noqa: E402

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
        self.track, self.clear = track, clear
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
    return [q for j, q in enumerate(pts) if j == 0 or math.hypot(q[0] - pts[j - 1][0], q[1] - pts[j - 1][1]) > 1e-6]


def own_via(pcb, pad, nid, grid):
    """the ball's dog-bone via: the net's via nearest the ball within a pitch (its gap point, or the ball itself for a
    via in its pad), or None"""
    reach = max(grid.pitch_x, grid.pitch_y) * 0.75
    vs = [v for v in pcb.vias if v.net_id == nid and math.hypot(v.x - pad.global_x, v.y - pad.global_y) <= reach]
    v = min(vs, key=lambda v: math.hypot(v.x - pad.global_x, v.y - pad.global_y)) if vs else None
    return (v.x, v.y) if v is not None else None


def escape_menu(pad, site, grid, faces, layers, copper, side=SIDE):
    """every escape the ball at `pad` has from its via at `site`, by each of `faces`, on each of `layers` (the module's
    docstring): [Move]"""
    net = (getattr(pad, 'net_name', '') or '').split('/')[-1]
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
        if in_pad:
            lines = [(ball[a_], [site, P(ball[a_], e)])]
        else:
            lines = []
            for sg in (-1, 1):
                ln0 = site[a_] + sg * h
                lines.append((ln0, [site, P(ln0, site[k]), P(ln0, e)]))
                # (the gap line a pitch over: a street where no via stands on it -- path_clear finds which)
                ls = site[a_] + sg * 2 * h
                lines.append((ls, [site, P(ls, site[k]), P(ls, e)]))
                row = site[k] + s * hk
                if s * (e - row) < 1e-6:
                    continue                    # (no ball row between the via and the exit line to sidestep along)
                for m in range(1, side + 1):
                    ln = ln0 + sg * m * 2 * h
                    lines.append((ln, [site, P(ln0, site[k]), P(ln0, row), P(ln, row), P(ln, e)]))
        for ln, path in lines:
            if not (lo - 1e-6 <= ln <= hi + 1e-6):
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


def escape_out(pad, grid, faces, copper):
    """the escapes of a ball with NO via (dogbone_arrays: an outer ball left to leave on its own layer): out on its
    own layer, straight along its own line or half a step onto a gap line beside it, to the exit (escape_menu's line, a
    pitch past the outer balls) -- a SURFACE escape, its run on that layer; where the lane leaves that layer is the
    route's, as the human's surface escapes dive 0.8-2.9 mm off the balls in pairs (zynq U1)"""
    net = (getattr(pad, 'net_name', '') or '').split('/')[-1]
    nid = pad.net_id
    home = next((L for L in pad.layers if L.endswith('.Cu')), 'F.Cu')
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
            g_ = ball[a_] + sg * h
            paths.append((g_, [ball, P(g_, ball[k] + s * hk), P(g_, e)]))
        for ln, path in paths:
            if not (lo - h - 1e-6 <= ln <= hi + h + 1e-6):
                continue
            path = dedupe(path)
            if not copper.path_clear(path, home, nid):
                continue
            legs = [(p, q, home) for p, q in zip(path, path[1:])]
            mv = em.Move(net, 'surface', d, home, P(ln, e), 0, legs, site=None, climb=1)
            mv.esc_side, mv.street = 0, False
            out.append(mv)
    return out


def _pair_legs_clash(ma, mb, track, clear):
    """two legs of one pair whose runs come within the rule of each other on their layer"""
    ra = [(p, q) for p, q, L in ma.legs if L == ma.layer]
    rb = [(p, q) for p, q, L in mb.legs if L == mb.layer]
    return ma.layer == mb.layer and any(seg_seg(p, q, u, v) < track + clear - 1e-6 for p, q in ra for u, v in rb)


def plan_state(pcb, names, sref, dref, banned=frozenset(), log=print, hold_teeth=None):
    """the ends model's plan state (whole_ends.Ends' keys, fanout_from_plan.plan_state's shape) on a dog-boned base:
    the escape menus at both arrays, no tooth laid"""
    import select_moves as sm
    import pairs as _pairs
    r = _rules.active()
    track, clear = r.fan_track, r.fan_clear
    copper = Copper(pcb, track, clear)
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    rlayers = list(route_layers.layers())
    sfp, dfp = pcb.footprints[sref], pcb.footprints[dref]
    sgrid, dgrid = em.grid_of(sfp), em.grid_of(dfp)
    cs = ((sgrid.bbox[0] + sgrid.bbox[2]) / 2, (sgrid.bbox[1] + sgrid.bbox[3]) / 2)
    cd = ((dgrid.bbox[0] + dgrid.bbox[2]) / 2, (dgrid.bbox[1] + dgrid.bbox[3]) / 2)
    toward_d = max(DIRS, key=lambda d: DIRS[d][0] * (cd[0] - cs[0]) + DIRS[d][1] * (cd[1] - cs[1]))   # source's near face
    far_s = min(DIRS, key=lambda d: DIRS[d][0] * (cd[0] - cs[0]) + DIRS[d][1] * (cd[1] - cs[1]))
    far_d = max(DIRS, key=lambda d: DIRS[d][0] * (cd[0] - cs[0]) + DIRS[d][1] * (cd[1] - cs[1]))     # dest's far face

    def half(pad, grid, d):
        ax = 0 if DIRS[d][0] else 1
        b = grid.bbox
        return ((pad.global_x, pad.global_y)[ax] - (b[ax] + b[ax + 2]) / 2) * DIRS[d][ax] >= -1e-6
    smenu, dmenu, src_pad, dst_pad, launch, none = {}, {}, {}, {}, {}, []
    for nm in names:
        nid, net = byname[nm]
        sp = next((p for p in net.pads if p.component_ref == sref), None)
        dp = next((p for p in net.pads if p.component_ref == dref), None)
        src_pad[nm], dst_pad[nm] = sp, dp
        if sp is None or dp is None:
            none.append(nm)
            continue
        launch[nm] = (sp.global_x, sp.global_y)
        for pad, grid, menus, faces in (
                # (the source's far face too, from a ball in its half beside it: the whole route takes such a lane
                # round the source, and the ends model prices it past any via count -- whole_ends.FAR_TOOTH)
                (sp, sgrid, smenu, [d for d in DIRS if d == toward_d or half(sp, sgrid, d)]),
                (dp, dgrid, dmenu, [d for d in DIRS if d != far_d or half(dp, dgrid, d)])):
            site = own_via(pcb, pad, nid, grid)
            if site is None:
                # (a ball with no via -- dogbone_arrays' outer ring -- leaves on its own layer: its surface escapes)
                import source_realize as sr
                menus[nm] = [m for m in escape_out(pad, grid, faces, copper) if (nm, sr.move_sig(m)) not in banned]
                continue
            import source_realize as sr
            menus[nm] = [m for m in escape_menu(pad, site, grid, faces, rlayers, copper)
                         if (nm, sr.move_sig(m)) not in banned]
    # TEETH_HOLD (a plan sidecar, the frame's): each net's TOOTH held -- where its escape reaches the boundary, by which
    # face, on which layer -- the menu cut to the moves there, the ends left to choose the berths; the shared array's
    # joint fanout (joint_escape.plan_array, hold=) laid its field round exactly those teeth, so the bus leaves it as it
    # left its own fanout. A net with no move there keeps its menu, and is named
    # (`hold_teeth` {net: {'tooth', 'direction', 'layer'}}: the same, in memory -- the escape step's own ends' teeth,
    # ESCAPE_JOINT)
    hold_ = dict(hold_teeth or {})
    hold_missed = []
    hold_path = awx_settings.get('TEETH_HOLD')
    if hold_path:
        sc_ = json.load(open(hold_path))
        for nm, t_ in (sc_.get('ends') or {}).items():
            if nm in (sc_.get('tooth_layer') or {}) and nm not in hold_:
                d_ = next(d for d, v in DIRS.items() if tuple(v) == tuple(sc_['tooth_dir'][nm]))
                hold_[nm] = {'tooth': tuple(t_[0]), 'direction': d_, 'layer': sc_['tooth_layer'][nm]}
    if hold_:
        held_, miss_ = 0, []
        for nm in sorted(smenu):
            g_ = hold_.get(nm)
            if not g_:
                continue
            at_ = [m for m in smenu[nm] if m.direction == g_['direction'] and m.layer == g_['layer']
                   and math.hypot(m.exit_pt[0] - g_['tooth'][0], m.exit_pt[1] - g_['tooth'][1]) <= 0.05]
            if at_:
                smenu[nm] = at_
                held_ += 1
            else:
                miss_.append(nm)
        hold_missed = miss_
        log(f'  teeth held: {held_} net(s) held to their tooth'
            + (f'; no move at it, menu kept: {miss_}' if miss_ else ''))
    # a PAIR's two legs never within the rule of each other: their joint options so banned (whole_ends: unbanned)
    import source_realize as sr
    banned = set(banned)
    prs = _pairs.pair_names(list(names)) if int(awx_settings.get('PLAN_PAIRS', awx_settings.get('BRAID_PAIRS', '0'))
                                                or 0) else {}
    nclash = 0
    for _b, (pn, nn) in prs.items():
        for menus in (smenu, dmenu):
            for ma in menus.get(pn, ()):
                for mb in menus.get(nn, ()):
                    if ma.direction == mb.direction and ma.layer == mb.layer and _pair_legs_clash(ma, mb, track, clear):
                        banned.add(('pair', pn, nn, sr.move_sig(ma), sr.move_sig(mb)))
                        banned.add(('pair', nn, pn, sr.move_sig(mb), sr.move_sig(ma)))
                        nclash += 1
    chi = sm.pair_chirality({nm: (src_pad[nm].global_x, src_pad[nm].global_y) for nm in launch},
                            {nm: (dst_pad[nm].global_x, dst_pad[nm].global_y) for nm in launch},
                            sgrid.bbox, dgrid.bbox)
    log(f'  escape menus: {sum(len(v) for v in smenu.values())} tooth moves, {sum(len(v) for v in dmenu.values())} '
        f'berth moves for {len(launch)} nets ({sum(1 for v in list(smenu.values()) + list(dmenu.values()) for m in v
                                                 if getattr(m, "street", False))} along streets); '
        f'{nclash} pair leg combinations clash'
        + (f'; no ball at both arrays: {none}' if none else '')
        + (f'; no tooth escape: {[n for n in launch if not smenu.get(n)]}' if any(not smenu.get(n) for n in launch)
           else '')
        + (f'; no berth escape: {[n for n in launch if not dmenu.get(n)]}' if any(not dmenu.get(n) for n in launch)
           else ''))
    import braid as te
    cache = {}

    def obs(nid, layer, own_only=False):
        key = (nid, layer, own_only)
        if key not in cache:
            cache[key] = te.build_obstacles(pcb, nid, {nid}, layer)
        return cache[key]
    return {'banned': banned, 'byname': byname, 'dmenu': dmenu, 'smenu': smenu, 'sblock': {}, 'launch': launch,
            'feedback': json.load(open(awx_settings.req('FEEDBACK'))) if awx_settings.get('FEEDBACK') else None,
            'incr': None, 'tooth0': {nm: 'F.Cu' for nm in launch}, 'tooth_vias': {nm: 1 for nm in launch},
            'src_pad': src_pad, 'dst_pad': dst_pad, 'sref': sref, 'dref': dref, 'sgrid': sgrid,
            'bundle_layer': 'F.Cu', 'chi': chi, 'dgrid': dgrid, 'dboxes': dgrid.bbox, 'buses': [list(launch)],
            'obs': obs, 'pcb': pcb, 'escape': True, 'held': {},
            'pads_of': {ref: [(p.global_x, p.global_y) for p in fp.pads] for ref, fp in pcb.footprints.items()},
            'hold_missed': hold_missed}


def incremental_fixed(st, names):
    """(INCREMENTAL= the previous round's sidecar, FEEDBACK= its findings) {net: move signature}: every berth the
    feedback does not name, held at the move it was laid by"""
    import pairs as _pairs
    import source_realize as sr
    if not (awx_settings.get('INCREMENTAL') and awx_settings.get('FEEDBACK')):
        return {}
    prev = json.load(open(awx_settings.req('INCREMENTAL')))
    fb = json.load(open(awx_settings.req('FEEDBACK')))
    items = [e for pr in fb.get('pairs', ()) for e in pr] + list(fb.get('avoid', ()))
    over = set(fb.get('over') or {})
    free_b = {e['lane'] for e in items if e['end'] == 1} | over
    pr_i = _pairs.pair_names(list(names))
    legs_b = {l_ for ln in free_b for l_ in (pr_i.get(ln) or (ln,))}
    fixed = {}
    for nm in names:
        if nm in legs_b or nm not in st['dmenu']:
            continue
        sig = (prev.get('berth_sig') or {}).get(nm)
        if sig is None:
            continue
        m = next((m for m in st['dmenu'][nm] if json.dumps(sr.move_sig(m)) == json.dumps(sig)), None)
        if m is not None:
            fixed[nm] = sr.move_sig(m)
    return fixed


def run_tracks(moves, byname, r):
    """the chosen moves' runs as tracks, each leg on its move's own layer (the stubs and vias stand on the field)"""
    tracks = []
    for nm, m in moves:
        nid = byname[nm][0]
        for p, q, L in m.legs:
            if L == m.layer and math.hypot(q[0] - p[0], q[1] - p[1]) > 1e-6:
                tracks.append({'start': tuple(p), 'end': tuple(q), 'width': r.fan_track, 'layer': L, 'net_id': nid})
    return tracks


def run_faults(a, b, r):
    """(net, net, layer) for each run of `a` and run of `b` -- two nets' -- on one layer within the rule"""
    return [(t['net_id'], u['net_id'], t['layer']) for t in a for u in b
            if t['layer'] == u['layer'] and t['net_id'] != u['net_id']
            and seg_seg(t['start'], t['end'], u['start'], u['end']) < r.fan_track + r.fan_clear - 1e-6]


def clear_lanes(moves, byname, lanes, r):
    """(the moves kept, the lanes left out): `lanes` [[net, ..]] in launch order, a pair's two legs one lane, each
    kept when its runs stand clear of every kept lane's -- two nets' runs on one layer within the rule are copper no
    stage after can lay. The rest are left out of the round and read off the board. (sdi_mipi_conv's SDI bus: its
    ends chosen again on the dog-boned field, where three lanes had no clean tooth and were offered as they stood,
    laid three runs on one channel line of U11 on In2.Cu, which the round's last resort then built on.)"""
    by = collections.defaultdict(list)
    for nm, m in moves:
        by[nm].append(m)
    kept_t, kept, left = [], [], []
    for lg in lanes:
        mv = [(nm, m) for nm in lg for m in by.get(nm, ())]
        tr = run_tracks(mv, byname, r)
        # (a pair's two legs are two nets: their own runs within the rule of each other are as unlayable as another
        # lane's -- sdi_mipi_conv's MIPI2_D2, P and N on In2.Cu, and the fanout audit set the J4 bus's whole round
        # aside for it)
        own = [f for i, t in enumerate(tr) for f in run_faults([t], tr[i + 1:], r)]
        if own or run_faults(tr, kept_t, r):
            left.append(list(lg))
            continue
        kept += mv
        kept_t += tr
    return kept, left


def lanes_of(launch):
    """the lanes of `launch` in its order: a pair's legs one lane, each other net its own"""
    import pairs as _pairs
    leg = {n: pr for pr in _pairs.pair_names(list(launch)).values() for n in pr}
    out, seen = [], set()
    for nm in launch:
        lg = list(leg.get(nm, (nm,)))
        if nm not in seen:
            out.append(lg)
            seen.update(lg)
    return out


def lanes_ended(launch, teeth, berths):
    """how many lanes of `launch` have both ends (a pair: both legs both)"""
    return sum(1 for lg in lanes_of(launch) if all(nm in teeth and nm in berths for nm in lg))


def without_lanes(launch, teeth, berths, miss):
    """(teeth, berths) less every lane of `launch` one of whose nets is in `miss`: a pair's legs together"""
    out_ = {nm for lg in lanes_of(launch) if set(lg) & set(miss) for nm in lg}
    return ({nm: m for nm, m in teeth.items() if nm not in out_},
            {nm: m for nm, m in berths.items() if nm not in out_})


def lay(base, out, moves, byname, log=print):
    """the chosen moves' runs laid on `base` as they are (their stubs and vias stand on it): OUT, and the clearance
    faults between the runs laid (two nets' runs on one layer within the rule), which clear_lanes leaves none of"""
    r = _rules.active()
    with contextlib.redirect_stdout(sys.stderr):
        pcb = parse_kicad_pcb(base)
    tracks = run_tracks(moves, byname, r)
    faults = [f for i, t in enumerate(tracks) for f in run_faults([t], tracks[i + 1:], r)]
    pcb_nets = {i: n.name for i, n in pcb.nets.items()}
    with contextlib.redirect_stdout(sys.stderr):
        ok = add_tracks_and_vias_to_pcb(base, out, tracks, [], net_id_to_name=pcb_nets)
    import fanout_from_plan as fp_
    fp_.copy_pro(base, out)
    log(f'  escapes laid: {len(moves)} ends, {len(tracks)} run segment(s)' + ('' if ok else ' (writer FAILED)')
        + (f'; {len(faults)} clearance fault(s) between runs: {sorted(set(faults))[:8]}' if faults else ''))
    return ok, faults


def sidecar(out, st, names, teeth, berths, cut, read=None):
    """OUT.plan.json, the plan sidecar braid.setup reads (fanout_from_plan.braid_plan_of's keys): each net's tooth and
    berth as laid (the chosen moves' exits: their runs end there), the layers there, the escape directions (out of
    each array, its face's), each berth's move, the pair frame, the ends model's cut round the destination, and its
    reading of each lane (READ, choose.last: its nets over two, its load on the trunk, its crossings, its blocked
    fronts -- the lanes a round the whole route lays nothing on frees next, whole_feedback --name)"""
    import source_realize as sr
    plan = {'nets': list(names), 'ends': {}, 'tooth_layer': {}, 'dest_layer': {}, 'tooth_dir': {}, 'stub_dir': {},
            'berth_sig': {}, 'chi': int(st['chi']), 'pages_first': False,
            'ends_model': {k: ((read or {}).get('parts') or {}).get('lane_' + k, {}) for k in ('over', 'load', 'x', 'front')},
            'escape': True}
    for nm in names:
        t, b = teeth.get(nm), berths.get(nm)
        if t is None or b is None:
            continue
        plan['ends'][nm] = [list(t.exit_pt), list(b.exit_pt)]
        plan['tooth_layer'][nm], plan['dest_layer'][nm] = t.layer, b.layer
        plan['tooth_dir'][nm] = list(DIRS[t.direction])
        plan['stub_dir'][nm] = list(DIRS[b.direction])
        plan['berth_sig'][nm] = sr.move_sig(b)
    if cut is not None:
        plan['dest_cut'] = list(cut) if isinstance(cut, (list, tuple)) else cut
    scut = ((read or {}).get('parts') or {}).get('scut')
    if scut is not None:
        plan['src_cut'] = float(scut)       # (the source's far face's cut: a far-face tooth's side round it)
    with open(out[:-len('.kicad_pcb')] + '.plan.json', 'w', encoding='utf-8') as f:
        json.dump(plan, f, indent=1)
    return plan


def lock_parts(src, dst, refs, unlock=False):
    """`src` written to `dst` with the footprints `refs` LOCKED (`unlock`: their lock taken off again) -- the cap step
    and the engine's foreign pads hold a locked part where it stands (placement.fanout_clearance.movable_cap_refs,
    bga_fanout.geometry.immovable_foreign_pads); the refs found"""
    from kicad_parser import iter_footprint_blocks
    txt = open(src, encoding='utf-8').read()
    out, at, got = [], 0, []
    for start, end, fp_text, _raw, key in iter_footprint_blocks(txt):
        if key not in refs:
            continue
        got.append(key)
        head = re.match(r'\(footprint\s+"(?:[^"\\]|\\.)*"', fp_text)
        if unlock:
            new = re.sub(r'\s*\(locked yes\)', '', fp_text, count=1)
        elif re.search(r'^\(footprint\s+"(?:[^"\\]|\\.)*"\s*\(locked', fp_text):
            continue
        else:
            new = fp_text[:head.end()] + ' (locked yes)' + fp_text[head.end():]
        out.append(txt[at:start] + new)
        at = end
    with open(dst, 'w', encoding='utf-8') as f:
        f.write(''.join(out) + txt[at:])
    if dst != src:
        import fanout_from_plan as fp_
        fp_.copy_pro(src, dst)
    return got


def run(out_path, K, base, log=print):
    import whole_ends
    from coherent_nets import coherent_nets
    _rules.install_defaults()
    run_nets = [n for n in (awx_settings.get('FANOUT_NETS') or '').split(',') if n]
    names = run_nets or coherent_nets(K, base)
    with contextlib.redirect_stdout(sys.stderr):
        pcb = parse_kicad_pcb(base)
    dref = awx_settings.get('DEST') or 'U5'
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    refs = collections.Counter(p.component_ref for nm in names if nm in byname for p in byname[nm][1].pads
                               if p.component_ref != dref)
    sref = refs.most_common(1)[0][0]
    log(f'escape fanout: {len(names)} nets, {sref} -> {dref}, on {os.path.basename(base)}')
    # ESCAPE_JOINT (route_bus): the source array is SHARED -- other nets and plane balls of its own, an earlier bus
    # routed through it. The ends choose the teeth on a field dog-boned as the escape fanout dog-bones an array of its
    # own; the array is then planned WHOLE round exactly those teeth (joint_escape.escape_field: one solve for the bus's
    # dog-bones, the other nets' escapes and the plane drops), and the bus leaves that field by the same teeth -- its
    # menus there cut to them (plan_state hold_teeth), each berth the move the ends chose
    jspec_ = awx_settings.get('ESCAPE_JOINT_SPEC')
    arr_ = None
    joint_laid = False
    if jspec_ and os.path.isfile(jspec_):
        arr_ = next((a_ for a_ in json.load(open(jspec_)).get('arrays', ()) if a_['ref'] == sref), None)
    # (ESCAPE_ENDS_BASE, route_bus: the board as it was before the shared array's earlier step, fanned and turned as
    # this step fans and turns its own -- the bus's OWN field, the one the escape fanout of the bus alone chooses its
    # ends on. The ends chosen there are the bus-alone's teeth and berths; the shared array is then planned round them)
    ends_base = awx_settings.get('ESCAPE_ENDS_BASE') if arr_ is not None else None
    base0 = base                    # (the shared base, before a joint field is laid on it)

    def choose_on(board_):
        with contextlib.redirect_stdout(sys.stderr):
            pcb_ = pcb if board_ == base else parse_kicad_pcb(board_)
        st_ = plan_state(pcb_, names, sref, dref, log=log)
        st_['base'] = board_             # (the board the ends are laid on: whole_ends.solve_check reads its floors)
        # (the bus's routed neighbours stand on the shared field, not on the bus's own: the tracks a far-face lane
        # crosses round each side of the source read there -- whole_ends.side_fences)
        st_['fence_pcb'] = pcb
        fixed_ = incremental_fixed(st_, names)
        if fixed_:
            log(f'  incremental: {len(fixed_)} berth(s) held as laid')
        b_, t_ = whole_ends.choose(st_, log=log, src_free=True, fixed=fixed_ or None)
        rd_ = getattr(whole_ends.choose, 'last', None) or {}
        return st_, b_, t_, rd_, rd_.get('cut')
    if ends_base:
        log(f'  the ends on the bus\'s own field: {os.path.basename(ends_base)}')
    st, berths, teeth, read, cut = choose_on(ends_base or base)
    if ends_base and not teeth:
        log(f'  the ends on the bus\'s own field: no plan -- chosen again on {os.path.basename(base)}')
        st, berths, teeth, read, cut = choose_on(base)

    def joint_try(st_, teeth_, berths_, with_site, board_=None):
        """the shared array planned whole round the ends' teeth (with their dog-bone sites, or anywhere the ball's
        escape reaches the same tooth) on `board_` (default the base); (st, teeth, berths) on that field, the nets the
        field does not leave as chosen, the field"""
        tried[0] = (st_, teeth_, berths_, with_site)
        import joint_escape as je
        import source_realize as sr
        # (each tooth held -- with its site: the dog-bone the field gave the ball, or none -- the bus's own copper on
        # the joint field is the field's, the plan placing only the other nets round it)
        hold_ = {nm: dict({'tooth': tuple(m.exit_pt), 'direction': m.direction, 'layer': m.layer, 'kind': m.kind},
                          **({'site': tuple(m.site) if m.site is not None else None} if with_site else {}))
                 for nm, m in teeth_.items()}
        if st_['base'] != base0:
            # (ends chosen on the bus's own field: each via end's run is the solve's to move onto another layer --
            # whole_solve's VBAN, relayer.clashes -- wherever no other copper stands within the rule of it. The layers
            # it is free on THERE are held free of the shared array's other copper too, so the solve has every layer
            # change it has on the bus alone's field)
            import types
            import relayer as _relayer
            with contextlib.redirect_stdout(sys.stderr):
                pcb_e = parse_kicad_pcb(st_['base'])
            ids_e = {n.name.split('/')[-1]: i for i, n in pcb_e.nets.items() if n.name}
            fw_ = _rules.active().fan_track
            runs_ = {}
            for nm, m in teeth_.items():
                if m.site is None or nm not in ids_e:
                    continue            # (a surface escape: no via end, nothing the solve moves)
                segs_ = [types.SimpleNamespace(start_x=p[0], start_y=p[1], end_x=q[0], end_y=q[1], width=fw_,
                                               net_id=ids_e[nm], layer=L)
                         for p, q, L in m.legs if L == m.layer and math.hypot(q[0] - p[0], q[1] - p[1]) > 1e-6]
                if segs_:
                    runs_[nm] = ({ids_e[nm]}, segs_)
            lays_ = list(route_layers.layers())
            ban_, _sep = _relayer.clashes(pcb_e, runs_, lays_, _rules.active().fan_clear)
            for nm in runs_:
                hold_[nm]['free_layers'] = [L for L in lays_ if L != teeth_[nm].layer and L not in ban_.get(nm, ())]
            log(f'  via ends\' runs on the bus\'s own field: {len(runs_)}, free on '
                f'{sum(len(hold_[nm]["free_layers"]) for nm in runs_)} other layer(s) -- held free of the shared '
                f'array\'s other copper')
        field_ = out_path[:-len('.kicad_pcb')] + '.field.kicad_pcb'
        run_s = set(names)
        others_ = list(arr_['others']) + [n for n in arr_.get('bus', ()) if n.split('/')[-1] not in run_s]
        # (the passives movable to the plan, as the engine lays it -- planned as fixed, the engine refused 23 of zynq
        # U1's planned drops the plan held clear; they are nudged off the round's copper once it is laid, below)
        rep_ = je.escape_field(board_ or base0, field_, sref, dref, list(names), others_, arr_['drops'], hold_, log=log,
                               passives_fixed=False)
        field_free[0] = rep_.get('bus_free') or {}
        log(f'  joint field of {sref}{"" if with_site else " (teeth held, sites free)"}: {rep_.get("status")} -- bus '
            f'{rep_.get("bus_escaped")}/{rep_.get("bus_balls")}, others {rep_.get("others_escaped")} escaped + '
            f'{rep_.get("others_strapped")} strapped + {rep_.get("others_stubbed")} dog-boned alone of '
            f'{rep_.get("others_balls")}, plane {rep_.get("dropped")}/{rep_.get("plane_balls")} dropped; unplanned '
            f'{rep_.get("unplanned")}')
        with contextlib.redirect_stdout(sys.stderr):
            pcb2 = parse_kicad_pcb(field_)
        st2 = plan_state(pcb2, names, sref, dref, log=log, hold_teeth=hold_)
        # the ends' plan laid as they chose it -- each tooth the move at it from the site the joint plan gave the
        # ball, each berth (the destination array is as it was) its own move: no second choice of the berths, whose
        # cut round the destination the descents pick afresh (zynq K40: the far cut's plan of no crossing, chosen
        # again at a winding cut, two crossings and 20 more on the objective)
        miss_ = list(st2['hold_missed'])
        teeth2 = {nm: st2['smenu'][nm][0] for nm in teeth_ if nm not in miss_ and st2['smenu'].get(nm)}
        miss_ += [nm for nm in teeth_ if nm not in teeth2 and nm not in miss_]
        berths2 = {}
        for nm, b_ in berths_.items():
            sig_ = json.dumps(sr.move_sig(b_))
            m_ = next((m for m in st2['dmenu'].get(nm, ()) if json.dumps(sr.move_sig(m)) == sig_), None)
            if m_ is None:
                miss_.append(nm)
            else:
                berths2[nm] = m_
        import pairs as _pairs
        r_ = _rules.active()
        for _b, (pn_, nn_) in _pairs.pair_names(list(teeth2)).items():
            if pn_ in teeth2 and nn_ in teeth2 and _pair_legs_clash(teeth2[pn_], teeth2[nn_], r_.fan_track,
                                                                      r_.fan_clear):
                miss_ += [pn_, nn_]
        st2['base'] = field_
        return st2, teeth2, berths2, sorted(set(miss_)), field_
    tried = [None]                  # (the last joint_try's ends: a field laid again round the parts it could not move)
    field_free = [{}]               # (the last joint_try's field: the escapes it chose for bus balls no tooth held)

    def free_ends(st_, teeth_, berths_):
        """(st, teeth, berths, read, cut) on the joint field with the lanes the ends gave no end and the field escaped
        -- each held to the tooth the joint plan chose it, the berths chosen again with every laid lane's tooth and
        berth held -- or None. (sdi_mipi_conv's SDI bus after the DDR3 bus and its terminations: the ends chosen on the
        dog-boned field, where SDI_D13, D18 and D19 had no tooth move, laid 18 lanes on a field whose plan escaped all
        21 balls; the three, never laid, stood on their dog-bones inside U11's far face, open)"""
        import source_realize as sr
        free_ = {nm: g for nm, g in field_free[0].items()
                 if nm in st_['launch'] and (nm not in teeth_ or nm not in berths_)}
        if not free_:
            return None
        hold3 = {nm: {'tooth': tuple(m.exit_pt), 'direction': m.direction, 'layer': m.layer, 'kind': m.kind}
                 for nm, m in teeth_.items()}
        hold3.update(free_)
        with contextlib.redirect_stdout(sys.stderr):
            pcb3 = parse_kicad_pcb(st_['base'])
        st3 = plan_state(pcb3, names, sref, dref, log=log, hold_teeth=hold3)
        st3['base'], st3['fence_pcb'] = st_['base'], pcb
        if any(nm in st3['hold_missed'] for nm in teeth_):
            return None                 # (a laid lane's tooth not met on the field as it was: nothing to build on)
        for nm, m in teeth_.items():
            st3['smenu'][nm] = [m]      # (its very move: a held exit can be reached by several, legs apart)
        b3, t3 = whole_ends.choose(st3, log=log, src_free=True,
                                   fixed={nm: sr.move_sig(b) for nm, b in berths_.items()})
        rd3 = getattr(whole_ends.choose, 'last', None) or {}
        sig = lambda m: json.dumps(sr.move_sig(m))
        if not all(nm in t3 and nm in b3 and sig(t3[nm]) == sig(teeth_[nm]) and sig(b3[nm]) == sig(berths_[nm])
                   for nm in teeth_):
            return None                 # (a laid lane's end not kept: the field's ends stand as they were)
        got_ = sorted(nm for nm in free_ if nm in t3 and nm in b3)
        if not got_:
            return None
        log(f'  joint field of {sref}: ends for {got_} at the teeth its plan escaped them to')
        return st3, t3, b3, rd3, rd3.get('cut')

    def partial(got_, read_, cut_):
        """(st, teeth, berths, read, cut) on the joint field `got_` laid with the lanes it leaves no way to their ends
        left out (a pair's legs together), the lanes the ends gave none given the field's escapes (free_ends) -- or
        None when it leaves no lane"""
        st2, teeth2, berths2, miss2, _f = got_
        teeth2, berths2 = without_lanes(st2['launch'], teeth2, berths2, miss2)
        if not teeth2:
            return None
        more_ = free_ends(st2, teeth2, berths2)
        return more_ if more_ is not None else (st2, teeth2, berths2, read_, cut_)
    if arr_ is not None and teeth:
        # (the ends' first choice, on the field they were chosen on, beside the round's: OUT.first.plan.json)
        sidecar(out_path[:-len('.kicad_pcb')] + '.first.kicad_pcb', st, [n for n in names if n in st['launch']],
                teeth, berths, cut, read)
        got = joint_try(st, teeth, berths, True)
        if got[3] and ends_base:
            # (the bus's own field's sites need not be the shared array's: the same teeth, each ball's dog-bone where
            # the joint plan finds room)
            got = joint_try(st, teeth, berths, False)
        if got[3] and ends_base:
            # (the bus-alone's ends do not stand on the shared array: the ends chosen on its own dog-boned field, as
            # with no bus-alone field given)
            log(f'  joint field of {sref}: the bus\'s own ends not left by it for {got[3]} -- the ends chosen again '
                f'on {os.path.basename(base)}')
            st, berths, teeth, read, cut = choose_on(base)
            got = joint_try(st, teeth, berths, True) if teeth else got
        if got[3]:
            # (never worse than the escape fanout of the bus alone: a joint field that does not leave the bus an end
            # is laid without those lanes when, with the lanes the ends gave none given its own escapes, it leaves
            # at least as many lanes their ends as the dog-boned field does -- else this round's field is given up,
            # the teeth and berths chosen on the dog-boned field laid as they are. sdi_mipi_conv's SDI bus after the
            # DDR3 bus, its ends chosen again on the dog-boned field: 14 lanes there, SDI_D12's tooth not left by the
            # field, which escaped 20 of the 21 balls -- given up, the 7 lanes with no tooth on the dog-boned field
            # stood open and U11's other balls on bare dog-bones, which route.py then failed to leave)
            part = partial(got, read, cut)
            if st['base'] != base:
                st, berths, teeth, read, cut = choose_on(base)
            n_dog = lanes_ended(st['launch'], teeth, berths)
            n_part = lanes_ended(part[0]['launch'], part[1], part[2]) if part is not None else -1
            if n_part >= n_dog:
                log(f'  joint field of {sref}: no way to its chosen end for {got[3]} -- laid without them, '
                    f'{n_part} lane(s) ended against {n_dog} on the dog-boned field')
                st, teeth, berths, read, cut = part
                base, joint_laid = got[4], True
            else:
                log(f'  joint field of {sref}: GIVEN UP this round -- no way to its chosen end for {got[3]} '
                    f'({n_part} lane(s) ended there against {n_dog}); the ends laid on the dog-boned field')
        else:
            st2, teeth2, berths2, _m, field = got
            log(f'  joint field of {sref}: the ends laid as chosen -- {len(teeth2)} teeth from the joint plan\'s '
                f'sites, {len(berths2)} berths as they were')
            st, base, teeth, berths, joint_laid = st2, field, teeth2, berths2, True
            more = free_ends(st, teeth, berths)
            if more is not None:
                st, teeth, berths, read, cut = more
    def lay_round(st_, base_, teeth_, berths_):
        """the chosen ends laid on `base_` (clear_lanes first), and on a joint field the cap nudge after: (ok, faults,
        moves, the parts the nudge could not move off the round's copper)"""
        missing = [nm for nm in st_['launch'] if nm not in berths_ or nm not in teeth_]
        if missing:
            log(f'  no escape chosen for {missing}')
        moves_ = [(nm, teeth_[nm]) for nm in st_['launch'] if nm in teeth_] + \
            [(nm, berths_[nm]) for nm in st_['launch'] if nm in berths_]
        # (no two lanes' runs laid on one layer within the rule: a lane whose runs meet a kept one's is left out)
        moves_, left = clear_lanes(moves_, byname, lanes_of(st_['launch']), _rules.active())
        if left:
            log(f'  left out of the round, their runs on one layer within the rule of another lane\'s: '
                f'{[ "+".join(lg) for lg in left]}')
            for lg in left:
                for nm in lg:
                    teeth_.pop(nm, None)
                    berths_.pop(nm, None)
        ok_, faults_ = lay(base_, out_path, moves_, byname, log=log)
        stuck_ = []
        if ok_ and joint_laid:
            # (the joint field's vias planned with the passives movable: the cap nudge moves those beneath the array
            # off the round's copper, as the chain's cap step does after a fanout -- place_fanout_clearance
            # --beneath-only)
            import subprocess
            import joint_escape as je_
            lock_ = je_.passives_beyond(out_path, [sref, dref])
            rc_ = subprocess.run([sys.executable, os.path.join(HERE, '..', 'py_router', 'place_fanout_clearance.py'),
                                  out_path, out_path, '--clearance', str(_rules.active().clearance), '--beneath-only']
                                 + (['--lock'] + lock_ if lock_ else []), capture_output=True, text=True)
            moved_ = [ln.strip() for ln in rc_.stdout.splitlines() if ln.startswith(('Moved', 'Stuck', '  Unresolved'))]
            log(f'  joint field caps: exit {rc_.returncode}' + (f' -- {"; ".join(moved_)[:300]}' if moved_ else ''))
            stuck_ = sorted({r_ for ln in moved_ if ln.startswith('Unresolved')
                             for r_ in re.split(r'[,\s]+', ln.split(':', 1)[1]) if r_})
        return ok_, faults_, moves_, stuck_
    ok, faults, moves, stuck = lay_round(st, base, teeth, berths)
    if ok and joint_laid and stuck and tried[0] is not None:
        # A PART THE NUDGE COULD NOT MOVE off the joint field's copper (at its displacement cap) stands on another
        # net's via the field planned there as if it would move: the field is planned again with that part LOCKED, so
        # its copper goes round the part, and the round laid again (sdi_mipi_conv on four layers: C118's GND pad on a
        # DDR3 address via beneath U11 -- a board violation the bus step then refused the whole bus for)
        lk_ = out_path[:-len('.kicad_pcb')] + '.locked_base.kicad_pcb'
        got_lk = lock_parts(base0, lk_, set(stuck))
        log(f'  joint field of {sref}: {", ".join(stuck)} not moved off its copper -- planned again with '
            f'{", ".join(got_lk)} held where they stand')
        got = joint_try(*tried[0], board_=lk_)
        if not got[3]:
            st2, teeth2, berths2, _m, field = got
            st, base, teeth, berths = st2, field, teeth2, berths2
            # (the lanes the ends gave none, given the field's escapes again: free_ends on this field)
            more = free_ends(st, teeth, berths)
            if more is not None:
                st, teeth, berths, read, cut = more
            ok, faults, moves, stuck = lay_round(st, base, teeth, berths)
            if ok:
                lock_parts(out_path, out_path, set(got_lk), unlock=True)     # (the board's own lock flags kept)
        else:
            # (the round as first laid stands: its copper is the board's to grade, the bus step's to put back)
            log(f'  joint field of {sref}: round the held parts the bus\'s ends are not left for {got[3]} -- the '
                f'round as first laid')
    sidecar(out_path, st, [n for n in names if n in st['launch']], teeth, berths, cut, read)
    by_face = collections.Counter((m.direction, m.layer) for _n, m in moves)
    log(f'  ends by face and layer: {dict(sorted(by_face.items()))}; sidesteps '
        f'{sum(1 for _n, m in moves if getattr(m, "esc_side", 0))}, streets '
        f'{sum(1 for _n, m in moves if getattr(m, "street", False))}')
    if not ok:
        return 2
    # THE FANOUT AUDIT, as fanout_from_plan's: a board the whole route would refuse is set aside and named -- over the
    # lanes this round laid both ends of: one with no escape chosen has no end to read (sdi_mipi_conv's SDI_D18, its
    # ball a via in pad with no stub once the DDR3 bus's address lanes took U11's field: the audit stopped on it and
    # the bus laid nothing), and the whole route leaves it out of the round, open (whole_route.with_ends)
    import fanout_from_plan as fp_
    sp, far, stops = fp_.fanout_audit(out_path, [n for n in names if n in teeth and n in berths], dref)
    if faults:
        # (never handed on: two lanes' runs on one layer within the rule -- clear_lanes leaves none, this the guard)
        stops = list(stops) + [f'{len(faults)} run(s) on one layer within the rule of another lane\'s']
    if sp or far or stops:
        held = fp_.set_aside_refused(out_path, sp, far, stops)
        log(f'REFUSED by the fanout audit: {fp_._audit_line(sp, far, stops)} -- the board set aside as '
            f'{os.path.basename(held)}')
        return 3
    log('fanout audit: clean')
    return 0


def main():
    if sys.argv[1:2] in (['-h'], ['--help']):
        print('usage: escape_plan.py OUT.kicad_pcb K --board=BASE.kicad_pcb\n')
        print(__doc__.strip())
        return
    if len(sys.argv) < 2 or sys.argv[1].startswith('-'):
        print(__doc__.strip().splitlines()[0])
        sys.exit(2)
    out_path = sys.argv[1]
    rest = [a for a in sys.argv[2:] if not a.startswith('-')]
    K = int(rest[0]) if rest else 21
    base = next((a.split('=', 1)[1] for a in sys.argv if a.startswith('--board=')), None)
    if base is None:
        sys.exit('escape_plan: --board=BASE is needed')
    sys.exit(run(out_path, K, base))


if __name__ == '__main__':
    main()

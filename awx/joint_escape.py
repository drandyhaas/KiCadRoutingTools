#!/usr/bin/env python3
"""joint_escape.py -- one escape planned for every signal ball of an array, the bus's and the other nets' together.

    python3 joint_escape.py BOARD REF --bus N1,N2,.. [--others N,..] [--prefer PREFER_BOARD] [--out OUT.kicad_pcb]

A ball-by-ball fanout lays each escape the cheapest way for that ball and cannot know which other balls still need a
way out: a bus escape run across three rows to a side face fences in every ball behind it (zynq U1, 2026-09-30: a
block of ~20 other balls left bare, boxed by the bus's runs, with no face and no layer left). So the escapes are
CHOSEN together, then laid together:

1. Every signal ball of the array gets its menu of escapes (escape_moves.enumerate_moves: surface along an adjacent
   gap, dog-bone, via-in-pad; face, exit gap, layer, kind), priced against the board's static copper the way the
   whole route prices its own (braid.build_obstacles). A bus ball's layers are the routing layers and it never leaves
   by the array's far face (the one facing away from the other array); another net's layers are the board's signal
   layers -- never an inner plane layer the run does not route on. On more routing layers than two a via move's run
   is planned on no one layer (RUN): K runs may share a lane, an exit cluster or a crossing, and each chosen run is
   given its layer after the solve (colour_runs).
2. The moves that cannot both be laid are found as the whole route finds them (select_moves._conflicts, strict: a
   shared gap stretch on a layer, a shared site, a via in the other's lane) -- two balls of one net's too, since the
   engine lays every escape on its own. A net's balls share copper through STRAPS instead: a ball of a net with more
   than one ball may join an adjacent ball of its net by a straight track rather than escape, and every chain of
   straps ends at a ball that escapes.
3. ONE CP-SAT solve picks at most one move per ball, no two in conflict: as many bus balls escaped as possible, then
   as many other balls, then the cheapest -- vias and length, and for a bus ball its distance from the tooth it is
   preferred to have (`prefer`: a board whose bus teeth the whole route has planned on).
4. The choice is handed to the production engine as a planned move for EVERY ball it covers
   (source_realize.full_move, strict), so no generic phase takes a planned ball's gap first.

plan_array() returns the hints and a report; lay() runs the one engine call (the under-pad engine's joint escape,
py_router/bga_fanout/underpad.py with joint=True: the plan first, the plane balls dropped, each net held to its
layers)."""

KRT_TOOL = {'scope': [], 'kind': 'actor'}   # #937: a research tool (awx), catalogued, shown at no door
import argparse
import bisect
import collections
import contextlib
import dataclasses
import gc
import hashlib
import heapq
import io
import itertools
import math
import os
import sys
import time
import types

import cp_seed  # noqa: E402  (every CP-SAT solve under the bus step's seed)
from coherent_nets import namer, engine_filter      # (a net by the whole route's name: its last part, else in full)

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))

OUTER = ('F.Cu', 'B.Cu')
W_BUS = 1_000_000          # a bus ball escaped
W_OTHER = 10_000           # another ball escaped
C_VIA = 300                # a via
C_MM = 20                  # a millimetre of escape
# a bus ball's PREFERRED tooth (`prefer`: the one a board the whole route has planned on has for it, preferred_teeth)
# is followed unless it cannot be -- a millimetre off it costs over three vias, another face, layer or kind
# ten; serving another ball outweighs both (W_OTHER). At 60 and 400 the plan put 18 of the zynq U1's 37 asked teeth
# elsewhere for a shorter escape, 13 of them out of order along the face the lanes leave by
C_DEV_MM = 1000            # a millimetre between a bus ball's exit and its preferred tooth's
C_DEV_KIND = 3000          # ...another face, layer or kind than its preferred tooth's
# a bus ball's exit within a pitch of a tooth its routed rounds found CROWDED (`avoid`: another bus's lane across the
# tooth on the layer its own lane leaves by): the plan of a shared array again, each round, with every bus's crowded
# teeth -- dearer than a millimetre off a preferred tooth and another face together, below serving another ball
C_AVOID = 5000
W_DROP = 10_000            # a plane ball dropped to its plane
C_VIP = 300                # a drop's via in the pad rather than a gap (filled and capped at the fab)
C_FINE_VIA = 300           # a drop's via a finer rung of the fab ladder's than the rung's own (_sizes: drop_vias)
# an escape's leg on a plane layer (plane_layers): ten vias, so a ball escapes there only where the outer layers have
# no room for it -- serving it outweighs that (W_OTHER) -- as the human's do (the zynq's U1: 15 of its nets on In1,
# 3 on In2). On F.Cu and B.Cu alone the zynq DDR's U1 left eight plane balls of its outer rings walled in by its other
# nets' escapes; offered its two planes at this price its plan served every ball, 17 escapes taking an inner layer
C_PLANE_LAYER = 3000
C_LANE_FACE = 400          # a drop's via off the array beyond a face the bus's laid lanes leave by: dearer than one
#                            in the pad -- that is the lanes' room, as the human keeps it (zynq U2's R9 and T9, dropped
#                            half a pitch off its west face where A3, A6 and BA2 turn in to their berths, and the
#                            whole route found their ends crowded there round after round)
SHARE_TOL = 1e-3           # two vias of one net this close, at one size, are one via (a shared drop): one gap
#                            computed from two of its balls differs by the array's own pitch error (zynq U5's balls
#                            0.8001 mm apart, on a 0.8 mm grid: 0.1 um)
# CP-SAT interleaved batches, phase 2's budget, on SOLVE_WORKERS threads: a thread holds its own working copy, and a
# batch is a task per thread. zynq three layers, U1 / U5: four threads and 20 batches 709 / 827 MB for the solve, two
# and 60 batches 552 / 755 MB at the same cost (97 worse on U1's 73,631, 16 better on U5's 34,812) in 17 s more
SOLVE_BATCHES = 60
# a plan that leaves a bus ball's run with no layer is asked again with each run's layer its own choice (plan_array's
# `exact_layers`)
LAYERS_EXACT = True
# ...and where phase 2 starts from the array's last plan in the process (_LAST_PLAN), laid out whole: the search starts
# at a plan's cost, not at phase 1's (any plan serving every ball, no cost: h616_lpddr4's U1 302,608 against 54,941
# after 60 batches), and in 30 batches stands about where 60 from phase 1's do -- h616_lpddr4's bus step's six such
# plans 1,350 worse to 460 better (a via is 300), zynq_ad9364's DDR two the same and 10 worse. The bus steps' joint
# plans: h616_lpddr4 519 -> 386 s, zynq_ad9364 580 -> 454, sdi_mipi_conv 740 -> 437
SOLVE_BATCHES_STARTED = 30
START_WORK = 30.0          # deterministic work to lay that start out whole (one worker)
SOLVE_WORKERS = 2
# phase 1's deterministic time, per question asked. At 120 the zynq U1 plane tier ran out of it on three layers and
# left six drops unplanned; at 240 it proved every drop (in 60 s)
P1_DET = 240.0
# A climb's reach, in pitches. zynq U1's whole array (100 batches): uncapped (to 19) 21172 moves, objective 48280503;
# capped at 8, 17356 and 48261318; at 4, 12582 and 48309493 -- the proved bound 48.44M in all three, and every climb
# any of the solves chose 1 to 4 deep but one
CLIMB = 4
# ...and the OTHER nets' (not the bus's): they need only leave, and their climbs were most of the model. zynq U1 for
# the AD9364's LVDS bus on three layers: 18,635 of its 20,727 moves were the 152 other balls', and the solve past 1.5 GB.
# At 1 the 10,667 moves served every ball of the 297 -- phase 1 proved all three tiers, where at 4 the plane tier ran
# out of time and left three balls unplanned; at 0 it left seven
CLIMB_OTHER = 1
# phase 2's full-problem subsolvers. Each loads the whole model: the default portfolio's eight held twice the memory
# (zynq U1, three layers: 1269 against 659 MB for the solve) for a cost 2% lower (1551 of 72,368: five vias)
SUBSOLVERS = ['default_lp', 'quick_restart']
# ...and with the runs on RUN (more routing layers than two) the quick restart's alone: their capacities are linear rows,
# which the LP worker holds whole -- zynq U1 on four layers, 994 against 767 MB for the plan at a cost 0.5% higher (322
# of 60,500: a via), U5 538 against 454 MB at 2.7% (665: two vias)
SUBSOLVERS_RUNS = ['quick_restart']
# (escape_to) another ball served by its DOG-BONE ALONE -- a stub and a via at a gap, no run out of the array,
# the router's to take on -- where no escape of its fits round the bus's teeth: dearer than any escape (one on a
# plane layer, C_PLANE_LAYER, included), never a ball left to the engine's own phases, which lay it blind to the plan
# (zynq U1's FB_CLK pair across the LVDS field)
C_STUB = 4000


def signal_layers(pcb):
    """every copper layer but an INNER one carrying a pour (a plane layer); an outer layer always"""
    planes = {z.layer for z in (pcb.zones or []) if z.net_id and z.layer not in OUTER}
    return [L for L in pcb.board_info.copper_layers if L not in planes]


def plane_layers(pcb):
    """the layers an escape takes only where it must: every INNER layer carrying a pour that the run does not route
    on (route_layers) -- zynq's In1 GND and In2's supply islands. The plan prices a leg on one (C_PLANE_LAYER)"""
    import route_layers
    planes = {z.layer for z in (pcb.zones or []) if z.net_id and z.layer not in OUTER}
    return sorted(planes - set(route_layers.layers()))


def far_face(pcb, ref, other_ref):
    """the face of `ref`'s array facing away from `other_ref`'s"""
    import escape_moves as em
    a, b = pcb.footprints[ref], pcb.footprints[other_ref]
    ax = sum(p.global_x for p in a.pads) / len(a.pads)
    ay = sum(p.global_y for p in a.pads) / len(a.pads)
    bx = sum(p.global_x for p in b.pads) / len(b.pads)
    by = sum(p.global_y for p in b.pads) / len(b.pads)
    return min(em.DIRS, key=lambda d: em.DIRS[d][0] * (bx - ax) + em.DIRS[d][1] * (by - ay))


def preferred_teeth(board, ref, bus):
    """{net short name: the tooth `board` has for it at `ref`} (source_realize.measure_tooth), for the bus's nets"""
    from kicad_parser import parse_kicad_pcb
    import source_realize as sr
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    sn = namer(pcb)
    byname = {sn(n.name): (i, n) for i, n in pcb.nets.items()}
    out = {}
    for n in bus:
        nm = sn(n)
        if nm not in byname:
            continue
        pad = next((q for q in byname[nm][1].pads if q.component_ref == ref), None)
        g = sr.measure_tooth(pcb, nm, pad, byname) if pad is not None else None
        if g and g.get('tooth'):
            out[nm] = g
    return out


Strap = collections.namedtuple('Strap', 'to a b layer length')     # a join to the neighbouring ball `to`
Stub = collections.namedtuple('Stub', 'site a layer')    # (escape_to) another ball's DOG-BONE alone: a stub on
#                                                           its layer from the ball `a` to a gap `site`, a via there
Drop = collections.namedtuple('Drop', 'site stub layer inpad r dr')  # a plane ball's via: in a diagonal gap behind a
#                                                                     stub (a, b) on `layer`, or in its pad; its
#                                                                     radius and its drill's, as the engine lays it
Piece = collections.namedtuple('Piece', 'net legs vias balls')      # an option's copper: legs [(a, b, layer)], vias
#                                                                     [(pt, radius, drill radius)] (every layer), its
#                                                                     own balls [pt]


def _pt_seg(c, u, v):
    ux, uy = v[0] - u[0], v[1] - u[1]
    L2 = ux * ux + uy * uy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((c[0] - u[0]) * ux + (c[1] - u[1]) * uy) / L2))
    return math.hypot(c[0] - (u[0] + t * ux), c[1] - (u[1] + t * uy))


def _seg_seg(p, q, a, b):
    def cross(o, a_, b_):
        return (a_[0] - o[0]) * (b_[1] - o[1]) - (a_[1] - o[1]) * (b_[0] - o[0])
    d1, d2, d3, d4 = cross(a, b, p), cross(a, b, q), cross(p, q, a), cross(p, q, b)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)) and d1 and d2 and d3 and d4:
        return 0.0
    return min(_pt_seg(p, a, b), _pt_seg(q, a, b), _pt_seg(a, p, q), _pt_seg(b, p, q))


def _sizes(pcb, foot):
    """The REAL sizes the engine lays the plan at, each item at its own: the fan track and clearance (the rung's),
    hole to hole, a gap site's via (the rung's, stepped down where the engine steps it down under this array:
    board_rules.fan_via_at), a ball's via-in-pad as the engine sizes it for THAT pad (clamp_via_to_pad on the fab ladder:
    a 0.35 mm ball takes no 0.45 mm via), the stacking pitch of two lanes at that track, and the engine's
    own-ball disk (underpad home_r)."""
    import rules as _rules
    import source_realize as sr
    from list_nets import board_constraint, escalation_rungs
    from routing_defaults import HOLE_TO_HOLE_CLEARANCE
    from bga_fanout.geometry import clamp_via_to_pad
    import board_rules
    vs_, vd_ = board_rules.fan_via_at(pcb, foot)
    tw, cl = sr.FAN_TRACK, sr.FAN_CLEAR
    h2h = _rules.active().hole_to_hole
    if h2h is None and getattr(pcb, 'source_path', ''):
        h2h = board_constraint(pcb.source_path, 'min_hole_to_hole')
    h2h = max(h2h or 0.0, HOLE_TO_HOLE_CLEARANCE)
    floors = escalation_rungs(len(pcb.board_info.copper_layers or ()) or 4)
    memo = {}

    def inpad(p):
        """(radius, drill radius) of the via the engine lays in pad p"""
        if id(p) not in memo:
            cs, cd = clamp_via_to_pad(vs_, vd_, p, floors)[:2]
            memo[id(p)] = (cs / 2.0, (cd or 0.0) / 2.0)
        return memo[id(p)]
    pad_r = max((max(q.size_x, q.size_y) for q in foot.pads), default=0.4) / 2
    # a drop's via where the rung's does not fit: the fab ladder's own vias below it, largest first (radius, drill
    # radius) -- the sizes the ladder steps the whole array's vias down to, here one drop's
    drop_vias = sorted({(f['via_diameter'] / 2.0, f['via_drill'] / 2.0) for f in floors
                        if f['via_diameter'] < vs_ - 1e-9}, reverse=True)
    return dict(tw=tw, cl=cl, h2h=h2h, d_seg=tw + cl, vr=vs_ / 2.0, vdr=vd_ / 2.0, inpad=inpad,
                drop_vias=drop_vias,
                grow=vs_ + cl, stack=tw + cl + _rules.HUG_OVER,
                r_home=max(pad_r + tw / 2 + cl, vs_ / 2 + tw / 2 + cl))


_BOX_EPS = 1e-9         # (_hit's box tests: a box this much farther than a bar apart is past it, whatever the rounding)


def _piece_geo(P, geo=None):
    """an option's copper `P` (a Piece) as _hit reads it: its legs by layer, each with its box, and the box of them all
    -- kept in `geo` {id: (piece, its geometry)} when given (the piece held there, so no id is met again for another)"""
    if geo is not None:
        g = geo.get(id(P))
        if g is not None and g[0] is P:
            return g[1]
    by_l = {}
    for (a, b, L) in P.legs:
        by_l.setdefault(L, []).append((a, b, min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])))
    xs = [q for (a, b, _L) in P.legs for q in (a[0], b[0])]
    ys = [q for (a, b, _L) in P.legs for q in (a[1], b[1])]
    g = (by_l, (min(xs), min(ys), max(xs), max(ys)) if xs else None)
    if geo is not None:
        geo[id(P)] = (P, g)
    return g


def _hit(P, Q, same, sz, geo=None):
    """can two options' copper not both be laid, each item at its own size? Two nets': any two legs on one layer
    closer than track + clearance, a via that close to a leg of the other's plus its radius (every layer), two vias
    closer than their radii + clearance or their drills than their radii + hole-to-hole. One net's, as the engine's
    raster sees it (it carries no nets): a leg's centreline OUTSIDE its own balls' disks that close to the other's
    copper, and drills at hole-to-hole. Two of one net's vias at one point and size are ONE via (two plane balls
    dropped at one gap site share its barrel, as the human's ground balls round a via do), and each one's leg reaches
    it inside the via's own keep (the engine's raster exempts the via's disk for the stub that ends there).
    The distances are measured only between items whose boxes come within the bar of each other (_BOX_EPS past it):
    two items farther apart on either axis are farther apart than the bar, so the answer is the one measuring every
    pair gives. `geo`: _piece_geo's store, for a caller asking of the same pieces many times"""
    tw2, cl = sz['tw'] / 2, sz['cl']
    shared = []
    for (c, r, dr) in P.vias:
        for (c2, r2, dr2) in Q.vias:
            dd = math.hypot(c[0] - c2[0], c[1] - c2[1])
            if same and dd < SHARE_TOL and abs(r - r2) < 1e-9 and abs(dr - dr2) < 1e-9:
                shared.append((c, r + tw2 + cl))
                continue
            if dd < dr + dr2 + sz['h2h'] - 1e-6 or (not same and dd < r + r2 + cl - 1e-6):
                return True
    gP, gQ = _piece_geo(P, geo), _piece_geo(Q, geo)
    ds = sz['d_seg'] - 1e-6
    dsx = ds + _BOX_EPS
    if not same:
        for L, lp in gP[0].items():
            lq = gQ[0].get(L)
            if not lq:
                continue
            for (a, b, x0, y0, x1, y1) in lp:
                for (u, v, X0, Y0, X1, Y1) in lq:
                    if X0 - x1 > dsx or x0 - X1 > dsx or Y0 - y1 > dsx or y0 - Y1 > dsx:
                        continue
                    if _seg_seg(a, b, u, v) < ds:
                        return True
        for (vs, g) in ((P.vias, gQ), (Q.vias, gP)):
            if not vs or g[1] is None:
                continue
            bx0, by0, bx1, by1 = g[1]
            for (c, r, _d) in vs:
                bar = r + tw2 + cl - 1e-6
                m = bar + _BOX_EPS
                if c[0] < bx0 - m or c[0] > bx1 + m or c[1] < by0 - m or c[1] > by1 + m:
                    continue
                for lst in g[0].values():
                    for (u, v, X0, Y0, X1, Y1) in lst:
                        if c[0] < X0 - m or c[0] > X1 + m or c[1] < Y0 - m or c[1] > Y1 + m:
                            continue
                        if _pt_seg(c, u, v) < bar:
                            return True
        return False
    import numpy as np
    rh = sz['r_home']
    for X, Y, gY in ((P, Q, gQ), (Q, P, gP)):
        yvias = [(c, r + tw2 + cl - 1e-6) for (c, r, _d) in Y.vias]
        for (a, b, L) in X.legs:
            # (only the other's items within the bar of this leg's box can be within it of a point along the leg)
            x0, y0, x1, y1 = min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])
            near_l = [(u, v) for (u, v, X0, Y0, X1, Y1) in gY[0].get(L, ())
                      if not (X0 - x1 > dsx or x0 - X1 > dsx or Y0 - y1 > dsx or y0 - Y1 > dsx)]
            near_v = [(c, k) for (c, k) in yvias
                      if not (c[0] < x0 - k - _BOX_EPS or c[0] > x1 + k + _BOX_EPS
                              or c[1] < y0 - k - _BOX_EPS or c[1] > y1 + k + _BOX_EPS)]
            if not near_l and not near_v:
                continue
            # (the leg's points every 0.02 mm, all at once: those outside its own balls' and the shared vias' disks,
            # each against the other's legs and vias near it -- point by point, a three-array model's builds spent
            # 28 of their 123 s here)
            n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.02))
            ii = np.arange(n + 1, dtype=float)
            px = a[0] + (b[0] - a[0]) * ii / n
            py = a[1] + (b[1] - a[1]) * ii / n
            keep = np.ones(n + 1, dtype=bool)
            for o in X.balls:
                keep &= ~(np.hypot(px - o[0], py - o[1]) < rh)
            for c, k in shared:
                keep &= ~(np.hypot(px - c[0], py - c[1]) < k)
            if not keep.any():
                continue
            px, py = px[keep], py[keep]
            for (u, v) in near_l:
                ux, uy = v[0] - u[0], v[1] - u[1]
                L2 = ux * ux + uy * uy
                t = 0.0 if L2 == 0 else np.clip(((px - u[0]) * ux + (py - u[1]) * uy) / L2, 0.0, 1.0)
                if (np.hypot(px - (u[0] + t * ux), py - (u[1] + t * uy)) < ds).any():
                    return True
            for (c, k) in near_v:
                if (np.hypot(px - c[0], py - c[1]) < k).any():
                    return True
    return False


def _box(pc, grow):
    pts = [q for (a, b, _L) in pc.legs for q in (a, b)] + [c for (c, _r, _d) in pc.vias] + list(pc.balls)
    return (min(q[0] for q in pts) - grow, min(q[1] for q in pts) - grow,
            max(q[0] for q in pts) + grow, max(q[1] for q in pts) + grow)


def _straps(pcb, grid, items, obs):
    """{ball: [Strap]}: a ball of a net with more than one ball here may, instead of escaping, join an adjacent ball
    of its net (one of its eight neighbours) by a straight track on its own layer -- the net's balls sharing an
    escape. The engine lays a strap after the planned escapes, exactly against the other nets' copper and on its
    raster with the two balls' own disks exempted."""
    by_net = collections.defaultdict(list)
    for key, (nm, _p) in items.items():
        by_net[nm].append(key)

    def near(d, pitch):
        return d < 0.01 or abs(d - pitch) < 0.01
    straps = collections.defaultdict(list)
    for nm, keys in by_net.items():
        if len(keys) < 2:
            continue
        for k1 in keys:
            p = items[k1][1]
            home = next((L for L in pcb.board_info.copper_layers if L in p.layers), 'F.Cu')
            for k2 in keys:
                q = items[k2][1]
                dx, dy = abs(q.global_x - p.global_x), abs(q.global_y - p.global_y)
                if k2 == k1 or home not in q.layers or not (near(dx, grid.pitch_x) and near(dy, grid.pitch_y)):
                    continue
                a, b = (p.global_x, p.global_y), (q.global_x, q.global_y)
                if obs(p.net_id, home).seg_clear(a, b):
                    straps[k1].append(Strap(k2, a, b, home, math.hypot(b[0] - a[0], b[1] - a[1])))
    return straps


# (the group's TIERS, group_ends: the pattern's wider forms, set while it builds a tier's menus) the pattern from the
# next gap point that stands clear where the first does not; and from every gap point that does and from the pad
PATTERN_NEXT_SITE = False
PATTERN_ALL_SITES = False


def escape_field_moves(p, grid, home, layers, faces, copper, site_clear, stub_clear, inpad_r, run, pattern=False,
                       lines=False):
    """(`escape_to`) ball p's moves as the ESCAPE FIELD has them (escape_plan.escape_menu / escape_out), from
    EVERY site it could take -- each of its four diagonal gap points whose via and stub stand clear (`site_clear(q, r)`,
    `stub_clear(a, b)`), its pad (a via in it, radius `inpad_r`), and no via at all (a surface escape out of the outer
    ring): the SITE is part of the move, so a ball's dog-bone is chosen with every neighbour's escape and drop, not before
    them. `run` (more routing layers than two): one move per site and path, its legs past the stub on RUN (the
    layers it clears the caller's to find) -- escape_menu's copy per layer was the model three times over.
    `pattern` (the group's ends, group_ends.py): THE PATTERN's moves alone, the fanout a hand lays -- a dog-bone to the
    gap point toward the array's nearest corner (every via away from the centre, no two balls' at one point) run
    straight out along the ball's own row or column, no side-step, by each face; and a surface escape by a face the
    ball stands within `pattern` rows of (True: one). A ball of the OUTER RING takes the gap point beside that one
    INWARD: toward the corner its via would stand outside the outer balls, in the way out of its neighbours' exits (232
    of a 248-ball array's 992 such moves, each in some bus exit's way). Its inward point is the one the ball behind it
    dog-bones to, so of the two one leaves on the surface -- the outer one, as a hand mostly has it, or the one behind
    it through the gap, the outer ball then free to run under the array to any face. No other move is enumerated --
    but, `lines` (with `pattern`): from that one via site, EVERY LINE it can leave along by each face, not the ball's
    own alone (escape_menu's lines beside the via, the streets a pitch over and their side-steps): some thirty moves
    a ball where the pattern has a handful, the exits' order round the array no longer the balls' own"""
    import copy
    import escape_plan as ep
    hx, hy = grid.pitch_x / 2.0, grid.pitch_y / 2.0
    c = (p.global_x, p.global_y)
    out = []
    if pattern:
        x0, y0, x1, y1 = grid.bbox
        # (toward the nearest corner -- but never outside the outer balls: an outer ball's the one beside it inward)
        dx, dy = (hx if c[0] >= (x0 + x1) / 2 else -hx), (hy if c[1] >= (y0 + y1) / 2 else -hy)
        q = (c[0] + (dx if x0 - 1e-6 <= c[0] + dx <= x1 + 1e-6 else -dx),
             c[1] + (dy if y0 - 1e-6 <= c[1] + dy <= y1 + 1e-6 else -dy))
        cands = [q]
        if PATTERN_NEXT_SITE:
            # (that point blocked -- a part's pad on the other side of the board under it -- the ball had NO dog-bone
            # in its pattern, and one of the outer rows only its surface escape, out of the face it stands at whatever
            # its bus's other part: the next gap point that stands clear is taken, the two beside the first before the
            # one across the ball)
            oth = [(c[0] + sx * hx, c[1] + sy * hy) for sx in (-1, 1) for sy in (-1, 1)]
            oth = [o for o in oth if x0 - 1e-6 <= o[0] <= x1 + 1e-6 and y0 - 1e-6 <= o[1] <= y1 + 1e-6
                   and math.hypot(o[0] - q[0], o[1] - q[1]) > 1e-6]
            cands += sorted(oth, key=lambda o: (round(math.hypot(o[0] - q[0], o[1] - q[1]), 6),
                                                 -round(math.hypot(o[0] - (x0 + x1) / 2, o[1] - (y0 + y1) / 2), 6), o))
        if PATTERN_ALL_SITES:
            # (the pattern from EVERY gap point that stands clear -- into a band of the array with no balls as toward
            # its corner -- and from a via in the ball's own pad, each run straight out along its own row or column by
            # each face)
            for sx in (-1, 1):
                for sy in (-1, 1):
                    q_ = (c[0] + sx * hx, c[1] + sy * hy)
                    if site_clear(q_, None) and stub_clear(c, q_):
                        out += (ep.escape_menu(p, q_, grid, faces, layers, copper) if lines else
                                ep.escape_menu(p, q_, grid, faces, layers, copper, side=0, own_line=True))
            if site_clear(c, inpad_r):
                out += (ep.escape_menu(p, c, grid, faces, layers, copper) if lines else
                        ep.escape_menu(p, c, grid, faces, layers, copper, side=0, own_line=True))
            cands = []
        for q in cands:
            if site_clear(q, None) and stub_clear(c, q):
                mv_ = (ep.escape_menu(p, q, grid, faces, layers, copper) if lines else
                       ep.escape_menu(p, q, grid, faces, layers, copper, side=0, own_line=True))
                out += mv_
                if mv_ or len(cands) == 1:        # (a point that stands clear with no line out of it is no site)
                    break
        depth = {'left': (c[0] - x0) / (2 * hx), 'right': (x1 - c[0]) / (2 * hx),
                 'up': (c[1] - y0) / (2 * hy), 'down': (y1 - c[1]) / (2 * hy)}
        surface = ep.escape_out(p, grid, [d for d in faces if depth[d] < pattern - 0.5], copper)
    else:
        for sx in (-1, 1):
            for sy in (-1, 1):
                q = (c[0] + sx * hx, c[1] + sy * hy)
                if site_clear(q, None) and stub_clear(c, q):
                    out += ep.escape_menu(p, q, grid, faces, layers, copper)
        if site_clear(c, inpad_r):
            out += ep.escape_menu(p, c, grid, faces, layers, copper)
        surface = None
    if not run:
        return out + (surface if surface is not None else ep.escape_out(p, grid, faces, copper))
    seen, col = set(), []
    for m in out:
        geo = tuple((tuple(a), tuple(b), L == home) for a, b, L in m.legs)
        k = (m.kind, m.direction, tuple(m.site or ()), geo)
        if k in seen:
            continue
        seen.add(k)
        m2 = copy.copy(m)
        m2.layer = RUN
        m2.legs = [(a, b, home if L == home else RUN) for a, b, L in m.legs]
        col.append(m2)
    return col + (surface if surface is not None else ep.escape_out(p, grid, faces, copper))


def _via_clear(pcb, obs, nid, q, r, sz):
    """a via of radius `r` at q clear of the board's static copper on every layer (the model is inflated by the
    clearance and half the fan track) -- a via's model: `obs(nid, layer, True)`, the movable passives left out"""
    return all(not (obs(nid, L, True).point_violation(q, pad=r - sz['tw'] / 2) or [0])[0]
               for L in pcb.board_info.copper_layers)


LANE_REACH = 1.6            # mm: a laid lane's straight continuation past its stub's end, held clear of a drop's via


def lane_rays(pcb, grid, foot, nets, reach=LANE_REACH):
    """[(a, b)]: the lanes the board already has leaving the array for `nets` (short names; the bus's, laid before
    the array's other balls are planned) -- each its straight continuation, from its copper's outer end out along the
    face that end lies beyond, `reach` mm. Only the stub stands at the fanout; the lane goes on from its end"""
    x0, y0, x1, y1 = grid.bbox
    grow = 1.5
    sn = namer(pcb)
    ids = {p.net_id for p in foot.pads if p.net_id and sn(p.net_name or '') in nets}
    ends = {}
    for s in pcb.segments:
        if s.net_id not in ids:
            continue
        for q in ((s.start_x, s.start_y), (s.end_x, s.end_y)):
            if not (x0 - grow <= q[0] <= x1 + grow and y0 - grow <= q[1] <= y1 + grow):
                continue
            d, u = max((q[0] - x1, (1, 0)), (x0 - q[0], (-1, 0)), (q[1] - y1, (0, 1)), (y0 - q[1], (0, -1)))
            if d > 0 and (s.net_id not in ends or d > ends[s.net_id][0]):
                ends[s.net_id] = (d, q, u)
    return [(q, (q[0] + u[0] * reach, q[1] + u[1] * reach)) for _d, q, u in ends.values()]


def _lanes_clear(site, r, rays):
    """a via of radius `r` at `site` leaves every lane of `rays` its bar: half the route's track, its clearance (the
    hug), and half a grid step for a line off the grid -- the bar the whole route's static audit holds a lane's end to
    (plan_audit.check_static), from the via's ring (_ring), as the audit measures it"""
    import braid as te
    bar = te.TRACK / 2 + te.CLEAR + te.GRID / 2
    r = _ring(site, r)
    for a, b in rays:
        dx, dy = b[0] - a[0], b[1] - a[1]
        t = max(0.0, min(1.0, ((site[0] - a[0]) * dx + (site[1] - a[1]) * dy) / (dx * dx + dy * dy or 1.0)))
        if math.hypot(a[0] + t * dx - site[0], a[1] + t * dy - site[1]) - r < bar - 1e-9:
            return False
    return True


def _ring(site, r):
    """how far a via of radius `r` at `site` reaches for a lane: the router keeps a track's grid cells off its ring
    about the grid point it rounds to (pairs.svia_reach), as the whole route's audit measures it. Measured at its own
    radius, h616's GND drop on U1's south face, between SDQ6's and SDQM1's stub ends, left each 0.175 for a bar of
    0.166; the audit read 0.130 from the ring, and the loop stopped at those ends two rounds running"""
    import braid as te
    import pairs as _pairs
    cfg = types.SimpleNamespace(grid_step=te.GRID, track_width=te.TRACK, clearance=te.CLEAR, via_size=te.VIA_SIZE)
    return max(r, _pairs.svia_reach(cfg, site[0], site[1], 2 * r))


def exit_ray_conflicts(opts, net_of, bus, vr, grid=None, bus_of=None):
    """[(key, i, key2, j)]: a plane ball's DROP option whose via stands in the way out of a BUS escape option -- its
    lane goes on from the exit, straight out past the face (lane_rays' reach), and the via's ring (_ring) leaves it
    less than its bar (_lanes_clear's), so the two are not both laid. `opts` {ball key: [(kind, option, piece)]},
    `net_of` {ball key: its net's short name}, `bus` the bus's short names, `vr` the largest via radius a drop takes.
    `bus_of` {bus net: its bus} (an array SHARED by several buses): a via of one bus's escape or dog-bone stands in
    another bus's way out just as a drop does -- the bus routed first keeps its via there, and the later one's lane
    finds no room for its end: a ball on an east face dog-boned just past the column, 0.40 mm from the leg of an FFC
    pair leaving that face, its tooth's end connector out of room for a 0.15 mm track's 0.40. An OTHER ball's via
    (no bus's) stands in every bus's way out there too: two dog-boned on the boundary line either side of a row a bus
    leaves by, 0.10 mm from its lane's edge.
    The ray starts where the laid tooth ends, half a pitch past the outer balls (`grid`'s; the lay's engine stops its
    escapes at the array's boundary), not at the plan's exit a pitch out: the lane is laid from that end, and a GND
    drop 0.57 mm past the face, between the boundary and the exit, left a pair's lane 0.151 mm from its via for 0.193.
    The drops are planned with the berths (plan_array), before any lane stands to keep them off (_drops' rays: the
    lanes laid already): on the zynq LVDS bus's U5, RFGND's G12 was dropped 0.14 mm past the west face between
    RX_FRAME's two berths, 0.400 mm from each lane for 0.4025, and two RFGND vias 0.4 mm either side of EN_AGC's berth
    on the south face -- the snap found no end connector for the pair and no way out for the single"""
    import braid as te
    import escape_plan as ep
    from escape_moves import DIRS
    bar = te.TRACK / 2 + te.CLEAR + te.GRID / 2
    cells = collections.defaultdict(list)                 # 1 mm cell -> [(key, index, a, b)]: the rays near it
    for k in sorted(opts):
        if net_of[k] not in bus:
            continue
        for j, (kind, o, _pc) in enumerate(opts[k]):
            if kind != 'escape':
                continue
            u = DIRS[o.direction]
            b = (o.exit_pt[0] + u[0] * LANE_REACH, o.exit_pt[1] + u[1] * LANE_REACH)
            back = 0.0 if grid is None else (ep.PAST - 0.5) * (grid.pitch_x if u[0] else grid.pitch_y)
            a = (o.exit_pt[0] - u[0] * back, o.exit_pt[1] - u[1] * back)
            g = _ring((te.GRID / 2, te.GRID / 2), vr) + bar     # (the widest ring a drop's via has: half a step off)
            for cx in range(int(math.floor(min(a[0], b[0]) - g)), int(math.floor(max(a[0], b[0]) + g)) + 1):
                for cy in range(int(math.floor(min(a[1], b[1]) - g)), int(math.floor(max(a[1], b[1]) + g)) + 1):
                    cells[(cx, cy)].append((k, j, a, b))
    out = []
    for k in sorted(opts):
        for i, (kind, o, _pc) in enumerate(opts[k]):
            if kind == 'drop':
                r = _ring(o.site, o.r)
            elif bus_of and kind in ('escape', 'stub') and getattr(o, 'site', None) is not None \
                    and (net_of[k] in bus_of or net_of[k] not in bus) and (kind == 'stub' or o.vias):
                r = _ring(o.site, vr)
            else:
                continue
            seen = set()
            for (k2, j, a, b) in cells.get((int(math.floor(o.site[0])), int(math.floor(o.site[1]))), ()):
                if kind != 'drop' and bus_of.get(net_of[k2]) == bus_of.get(net_of[k]):
                    continue
                if (k2, j) not in seen and _pt_seg(o.site, a, b) - r < bar - 1e-9:
                    seen.add((k2, j))
                    out.append((k, i, k2, j))
    return out


def zone_regions(pcb):
    """{layer: [(net id, priority, outline)]}: the board's pours, by layer -- and each KEEP-OUT rule area that forbids
    a pour on the layer, as a pour of no net (-1) that fills before every other: no plane is there. A plane ball's drop
    planned inside one on a board's only layer of its net's pour -- inside the pour's outline, under the array -- was
    joined to no pour once the board was filled"""
    out = collections.defaultdict(list)
    for z in pcb.zones or ():
        if z.net_id and len(z.polygon) >= 3:
            out[z.layer].append((z.net_id, z.priority or 0, z.polygon))
    for k in (getattr(pcb.board_info, 'keepouts', None) or ()):
        poly = [tuple(q[:2]) for q in (k.get('polygon') or ())]
        if k.get('copper_pour_allowed', True) or len(poly) < 3:
            continue
        Ls = set(k.get('layers') or ())
        for L in list(out):
            if L in Ls or '*.Cu' in Ls or ('F&B.Cu' in Ls and L in ('F.Cu', 'B.Cu')):
                out[L].append((-1, float('inf'), poly))
    return out


def on_own_plane(regions, nid, q):
    """a through via at q lands in its net's plane: inside the outline of a pour of its net on some layer, and inside
    no pour of another net there that fills before it (a higher priority) -- on a layer split among several supplies
    (the zynq's In2: VCC_1V0's core island under U1 among VCC_1V8's balls) the region is its net's only where no other
    net's island takes it. A ball is never dropped to a plane that is not there"""
    from obstacle_map import point_in_polygon
    for L, zs in regions.items():
        for n, pr, poly in zs:
            if n == nid and point_in_polygon(q[0], q[1], poly) and not any(
                    n2 != nid and pr2 > pr and point_in_polygon(q[0], q[1], p2) for n2, pr2, p2 in zs):
                return True
    return False


def laid_pockets(pcb, grid, nets):
    """[(layer, quad)]: the POCKET of each pair among `nets` (short names; the bus's, laid before the array's other
    balls are planned) whose two teeth leave the array by one face on one layer (pair_teeth.pockets): between them,
    from half a pitch inside to where the legs close past them. Copper of another net there is walled in once the
    pair closes"""
    import pair_teeth as pt
    import pairs as _pairs
    prs = _pairs.pair_names(sorted(nets), admit_all=True)
    legs = {leg for pr in prs.values() for leg in pr}
    if not legs:
        return []
    sn = namer(pcb)
    name_of = {i: sn(n.name) for i, n in pcb.nets.items() if n.name}
    segs = [((s.start_x, s.start_y), (s.end_x, s.end_y), s.layer, s.net_id) for s in pcb.segments
            if name_of.get(s.net_id) in legs]
    tooth = pt.teeth(segs, name_of, legs, grid.bbox)
    return [(pk[0], pk[2]) for pk in pt.pockets(prs, tooth, max(grid.pitch_x, grid.pitch_y) / 2.0).values()
            if pk != 'apart']


def lane_walls(pcb, grid, nets):
    """[(layer, quad)]: the pockets between two neighbouring LAID lanes of `nets` (short names; the lanes routed before
    the array's other balls are planned) that close past their teeth on one layer -- no room left between them for a
    via, so a track of another net that leaves the array there neither crosses them nor leaves their layer
    (pair_teeth.lane_pockets): the zynq U1's OTG_DATA5 and SD_D1, escaped on F.Cu east between the DDR3 bus's A14 and
    A10, which leave that face 1.6 mm apart and run on north-east 0.78 mm apart, OTG_DATA5 open on every board after"""
    import braid as te
    import pair_teeth as pt
    import pairs as _pairs
    sn = namer(pcb)
    name_of = {i: sn(n.name) for i, n in pcb.nets.items() if n.name}
    laid = set(nets)
    segs = [((s.start_x, s.start_y), (s.end_x, s.end_y), s.layer, s.net_id) for s in pcb.segments
            if name_of.get(s.net_id) in laid]
    if not segs:
        return []
    tooth = pt.teeth(segs, name_of, laid, grid.bbox)
    return [(L, q) for L, _u, q in pt.lane_pockets(segs, name_of, tooth, grid.bbox, max(grid.pitch_x, grid.pitch_y) / 2.0,
                                                   te.VIA_SIZE + 2 * te.CLEAR + te.TRACK,
                                                   _pairs.pair_names(sorted(laid), admit_all=True))]


def exit_walled(walls, m):
    """does escape move `m`'s EXIT -- where its lane goes on from, on its layer -- stand in one of `walls` (lane_walls)"""
    import pair_teeth as pt
    q, L = getattr(m, 'exit_pt', None), getattr(m, 'layer', None)
    return q is not None and any(L == wl and pt.in_pocket(wq, tuple(q)) for wl, wq in walls)


def in_pockets(pockets, legs, vias):
    """does copper -- `legs` [(a, b, layer)], `vias` [points] (every layer) -- enter one of `pockets` (laid_pockets)"""
    import pair_teeth as pt
    return any(any(L == pl and pt.in_pocket(q, a, b) for a, b, L in legs) or any(pt.in_pocket(q, v) for v in vias)
               for pl, q in pockets)


def lane_faces(rays):
    """{(ux, uy)}: the faces the laid lanes `rays` (lane_rays) leave the array by, as their outward unit vectors"""
    out = set()
    for a, b in rays:
        dx, dy = b[0] - a[0], b[1] - a[1]
        out.add((1 if dx > 1e-9 else -1 if dx < -1e-9 else 0, 1 if dy > 1e-9 else -1 if dy < -1e-9 else 0))
    return out


def beyond_faces(grid, q):
    """{(ux, uy)}: the faces of the array (its ball box) a point lies beyond, by more than a quarter pitch"""
    x0, y0, x1, y1 = grid.bbox
    gx, gy = grid.pitch_x / 4, grid.pitch_y / 4
    return {u for u, ok in (((1, 0), q[0] > x1 + gx), ((-1, 0), q[0] < x0 - gx), ((0, 1), q[1] > y1 + gy),
                            ((0, -1), q[1] < y0 - gy)) if ok}


def _drops(pcb, grid, p, obs, sz, foot=None, rays=(), regions=None, fine=False):
    """[Drop]: a plane ball's ways down to its plane -- a stub to one of its four diagonal gaps and a via there (the
    engine's dog-bone drop, at the rung's via), or a via in its pad (at the size the engine gives that pad) -- each
    clear of the board's static copper. An edge ball's gaps include those half a pitch off the array's edge: the via
    there is the drop's end, with no escape to leave room for past it (the zynq's human puts U5's A5 and A6, A1 and B1
    each round one such via, the bus's lanes leaving by every inner gap beside them). And toward a side of the ball
    with no ball of `foot` (the array's edge, a depopulated site), a via STRAIGHT out from it, as far as a diagonal
    gap is: on the ball's own row or column, between the two gap lanes beside it (the human's U1 A8 and U5 M5, every
    gap beside them a lane) -- each leaving the lanes laid there (`rays`, lane_rays) their bar: at the zynq DDR's U2,
    0.45 mm vias straight out between the bus's berths at 0.8 mm left each lane 0.15 mm, and the whole route found its
    ends crowded at 20 of them -- and each in the ball's own plane (`regions`, zone_regions: on_own_plane). A gap or
    straight-out site the rung's via does not fit takes the largest via of the fab ladder's that does (sz['drop_vias'],
    laid at that size: the hint's 'via'), as a pad too small for the rung's takes a clamped via"""
    home = next((L for L in pcb.board_info.copper_layers if L in p.layers), 'F.Cu')
    hx, hy = grid.pitch_x / 2.0, grid.pitch_y / 2.0
    pad = (p.global_x, p.global_y)
    sites = [(pad[0] + sx * hx, pad[1] + sy * hy) for sx in (-1, 1) for sy in (-1, 1)]
    if foot is not None and hx and hy:
        reach = math.hypot(hx, hy)
        for ux, uy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nb = (pad[0] + 2 * hx * ux, pad[1] + 2 * hy * uy)
            if not any(abs(q.global_x - nb[0]) < 0.3 * hx and abs(q.global_y - nb[1]) < 0.3 * hy for q in foot.pads):
                sites.append((pad[0] + reach * ux, pad[1] + reach * uy))
    out = []
    for site in sites:
        if not obs(p.net_id, home).seg_clear(pad, site) or \
                (regions is not None and not on_own_plane(regions, p.net_id, site)):
            continue
        # the rung's via, else the largest of the fab ladder's that fits (sz['drop_vias']): at the zynq DDR's U2 a
        # 0.45 mm via straight out between two berths' lanes left each 0.075 mm, a 0.30 one 0.15
        fits = [(r_, dr_) for r_, dr_ in [(sz['vr'], sz['vdr'])] + sz['drop_vias']
                if _via_clear(pcb, obs, p.net_id, site, r_, sz) and _lanes_clear(site, r_, rays)]
        # ...and with `fine` (plan_array's exit_rays), where the rung's fits, the largest finer one too: a berth the
        # same plan lays has no lane on the board yet, and the rung's via may stand in its way out (exit_ray_conflicts)
        # where a finer one leaves it its bar
        for r in fits[:2 if fine and fits and fits[0][0] == sz['vr'] else 1]:
            out.append(Drop(site, (pad, site), home, False, r[0], r[1]))
    r, dr = sz['inpad'](p)
    if _via_clear(pcb, obs, p.net_id, pad, r, sz) and (regions is None or on_own_plane(regions, p.net_id, pad)):
        out.append(Drop(pad, None, home, True, r, dr))
    return out


def movable_refs(pcb, ref, nets=()):
    """the movable passives: the parts the cap placement step that follows moves (place_fanout_clearance, by its own
    rule: placement.fanout_clearance.movable_cap_refs -- unlocked two-pad C/R/FB parts within its near margin of a
    BGA's ball field), so the joint escape plans and lays its VIAS as if they were not there (its tracks go round
    them: build_menus); every other passive -- the
    channel's, a terminating resistor between the arrays -- is copper nothing will move. None where the board says
    nothing will move them (`_fanout_all_foreign_immovable`, which the engine reads too:
    geometry.immovable_foreign_pads). Read apart from the engine under that mark, a plan ran CTRL_OUT0 through RX10's B
    pad under zynq U1, the engine refused it and laid it a gap over, through the via site the same plan kept for
    TX_FRAME_P, whose ball was left bare"""
    if getattr(pcb, '_fanout_all_foreign_immovable', False):
        return frozenset()
    if os.path.join(HERE, '..', 'py_placer') not in sys.path:
        sys.path.insert(0, os.path.join(HERE, '..', 'py_placer'))
    from placement.fanout_clearance import movable_cap_refs
    path = getattr(pcb, 'source_path', '') or None
    key = None
    if path and os.path.isfile(path):
        st_ = os.stat(path)
        key = (path, st_.st_mtime_ns, st_.st_size)
    got = _MOVABLE.get(key) if key else None
    if got is None:
        with contextlib.redirect_stdout(io.StringIO()):
            # (the whole route's cap step moves only the parts beneath a BGA: --beneath-only, whole_route)
            got = frozenset(movable_cap_refs(pcb, path, beneath_only=True))
        if key:
            _MOVABLE[key] = got
    return got - {ref} - tied_passives(pcb, got, nets)


_MOVABLE = {}       # (a board file, its mtime and size) -> its movable caps: the courtyards read off it once


def tied_passives(pcb, refs, nets):
    """the passives among `refs` served THROUGH A BALL: a pad of theirs, of a signal, under a ball of its own net on the
    other layer (pairs.under_pad -- a terminating resistor on the back under a DDR clock pair's balls), of one of the
    run's `nets` (full or short names): tied by a via at the ball (fanout_from_plan.tie_vias_under) or by the ball's own
    via in its pad. Their place is the design's: the plans route round them as fixed copper and the cap step leaves
    them (passives_beyond). Moved as any passive beneath the array, the zynq7020's R50 was turned a quarter off
    DDR_CK_P's ball, and the pair was handed on open"""
    import braid as te
    import pairs as _pairs
    # (a signal's: a plane net's passive -- a decoupling cap's pad under a power ball -- has its own way down, and
    # moving it off the copper is the cap step's whole work)
    planes = {z.net_id for z in (pcb.zones or []) if z.net_id}
    sn = namer(pcb)
    want = {sn(n) for n in nets}
    out = set()
    for r in refs:
        fp = pcb.footprints.get(r)
        if fp is None:
            continue
        for q in fp.pads:
            if q.net_id and q.net_id not in planes and sn(q.net_name or '') in want and \
                    any(p.component_ref != r and _pairs.under_pad(p, q, te.VIA_SIZE)
                        for p in pcb.pads_by_net.get(q.net_id, ())):
                out.add(r)
                break
    return frozenset(out)


def passives_beyond(board, refs, nets=()):
    """the passives the cap step (place_fanout_clearance --beneath-only) would move on BOARD that stand beneath NONE of
    the arrays `refs` -- beneath another array, whose step this is not: a bus step's cap nudge locks them (--lock), so it
    moves only the parts beneath its own arrays. Unscoped, the zynq's DDR step (U1 -> U2) moved RX4 and C177 beneath U5,
    the LVDS bus's array, and the LVDS step on its board found them where its own nudge would never have left them.
    ...and the ones served through a ball of the run's `nets` (tied_passives), beneath its own arrays or not: their
    place is the design's"""
    if os.path.join(HERE, '..', 'py_placer') not in sys.path:
        sys.path.insert(0, os.path.join(HERE, '..', 'py_placer'))
    from placement.fanout_clearance import movable_cap_refs, package_box
    from placement.parser import extract_courtyard_bboxes
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(board)
        moving = movable_cap_refs(pcb, board, beneath_only=True)
        court = extract_courtyard_bboxes(board)
    boxes = [package_box(pcb.footprints[r], court) for r in refs if r in pcb.footprints]
    under = lambda f: any(b[0] <= f.x <= b[2] and b[1] <= f.y <= b[3] for b in boxes)   # noqa: E731
    return sorted({r for r in moving if r in pcb.footprints and not under(pcb.footprints[r])}
                  | tied_passives(pcb, moving, nets))


def bus_route_layers(pcb):
    """the layers a BUS ball escapes on: every routing layer (route_layers.stacked) -- F.Cu and B.Cu on two, and on
    more a via's run on any of them, the plan's to choose (RUN); the solve moves it at its via end"""
    import route_layers
    return route_layers.stacked(pcb.board_info.copper_layers)


# ON MORE ROUTING LAYERS THAN TWO, a via move's RUN -- its stretch from the via out to the array's edge -- is planned on
# no one layer: one move per escape shape, its run on RUN, and the layers its run is clear on in `runs_on` (a Move
# attribute). Offered layer by layer, the zynq U1's array on four layers was 41,977 moves and 1.6 million constraints,
# past 1.5 GB at the first solve; one move a shape is its two-layer 14,955. The plan asks of the runs on RUN what K
# layers can carry -- at most K in one lane at any point and at one exit cluster, a lane's runs being intervals (K of
# them overlapping take K layers, and no more are needed); at most K of the runs through one crossing -- and gives each
# chosen run its layer after (colour_runs)
RUN = '*run*'


def colour_runs(runs, edges, together, want, kinds=None):
    """({key: layer}, [keys left uncoloured]): a layer for each run (`runs` {key: its layers}) -- no two of `edges` on
    one, the two of each `together` pair on one, each on its `want` layer where it has it, else the first it can of
    its outer layer then the inner ones in the stack's order (the inner layers kept for the bus's lanes), a plane's
    layer last (`kinds`, route_layers.plane_kinds: a power plane's, then a ground plane's, as the whole route prices
    them). One small exact solve; a run no colouring holds (a crossing's capacity met by runs some of which never meet)
    is left out"""
    from ortools.sat.python import cp_model
    if not runs:
        return {}, []
    mdl = cp_model.CpModel()
    c = {k: {L: mdl.NewBoolVar(f'c{n}_{L}') for L in Ls} for n, (k, Ls) in enumerate(sorted(runs.items()))}
    ok = {k: mdl.NewBoolVar(f'ok{n}') for n, k in enumerate(sorted(runs))}
    for k in sorted(runs):
        mdl.Add(sum(c[k].values()) == ok[k])
    for a, b in edges:
        for L in sorted(set(c[a]) & set(c[b])):          # (sorted: a set of layer names iterates in hash order)
            mdl.AddBoolOr([c[a][L].Not(), c[b][L].Not()])
    for a, b in together:
        for L in sorted(set(c[a]) | set(c[b])):
            if L in c[a] and L in c[b]:
                mdl.Add(c[a][L] == c[b][L])
            else:
                mdl.Add((c[a] if L in c[a] else c[b])[L] == 0)
    kr = {'-': 0, 'pwr': 1, 'gnd': 2}
    rank = lambda k, L: 0 if L == want.get(k) else 1 + sorted(runs[k], key=lambda L_: (
        kr[(kinds or {}).get(L_, '-')], L_ not in ('F.Cu', 'B.Cu'), runs[k].index(L_))).index(L)
    mdl.Maximize(sum(1000 * ok[k] for k in runs) - sum(rank(k, L) * x for k in runs for L, x in c[k].items()))
    s = cp_seed.solver()
    s.parameters.num_workers = 1
    s.parameters.max_deterministic_time = 30
    st = s.Solve(mdl)
    if st not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        return {}, sorted(runs)
    out = {k: L for k in runs for L, x in c[k].items() if s.Value(x)}
    return out, sorted(k for k in runs if k not in out)


def _off_layer(m, want):
    """m does not leave on the layer `want`: a run on RUN leaves on it when it is one of its layers"""
    return (want not in m.runs_on) if m.layer == RUN else (m.layer != want)


def run_layers():
    """the routing layers when there are more than two (a via move's run planned on RUN), else None"""
    import route_layers
    rl = route_layers.layers()
    return rl if len(rl) > 2 else None


MIN_BUS_NETS = 8           # a bus: this many nets or more between two parts (route_bus.MIN_BUS_NETS, find_buses)


def bus_groups(pcb, ref, bus):
    """{short net: its bus at the array `ref`}: the bus's own nets 'bus', and those of every other bus the array has
    -- as route_bus.find_buses finds them: MIN_BUS_NETS nets or more on it and one other part that the whole route
    admits (make_bench.pair_nets), whether the whole route takes the bus or the router routes it -- keyed by that
    part; {} when the bus is the array's only one"""
    import make_bench as mb
    many = {r for r, f in pcb.footprints.items() if len(f.pads) > 2}
    count = collections.Counter()
    for net in pcb.nets.values():
        ends = {p.component_ref for p in net.pads if p.component_ref in many}
        if len(ends) == 2 and ref in ends:
            count[min(ends - {ref})] += 1
    sn = namer(pcb)
    bus_s = {sn(n) for n in bus}
    out = {}
    for other, n in sorted(count.items(), key=lambda t: (-t[1], t[0])):
        if n < MIN_BUS_NETS:
            continue
        with contextlib.redirect_stdout(io.StringIO()):
            nets = {sn(x) for x in mb.pair_nets(pcb, ref, other)}
        if len(nets) < MIN_BUS_NETS or nets & bus_s:
            continue                        # (too few the whole route admits; or the bus itself)
        for x in sorted(nets):
            out.setdefault(x, other)
    if not out:
        return {}
    out.update({x: 'bus' for x in bus_s})
    return out


def array_pairs(foot, nets, sn):
    """{base: (P leg, N leg)}: the differential pairs among `nets` (the whole route's names, `sn`: coherent_nets.namer)
    with a ball on the array `foot` -- pairs.pair_names over its pads' nets, every suffix pair admitted; a base that is
    a net of its own is no pair (pairs.members)"""
    import pairs as _pairs
    here = sorted({sn(p.net_name or '') for p in foot.pads if p.net_id} & set(nets))
    return {b_: v_ for b_, v_ in _pairs.pair_names(here, admit_all=True).items() if b_ not in here}


def build_menus(pcb, ref, bus, others, other_layers, far=None, drops=(), climb=CLIMB, street=2, only=None,
                vias_only=False, exit_rays=False, escape_to=None,
                all_faces=False, pattern=False, lines=False):
    """Every ball's OPTIONS for plan_array (its escapes, its straps, its drops, each with its copper as a Piece at its
    real size), and what they were built from: a namespace of foot, grid, bus_s, oth_s, drop_s, sz, items, menu (the
    escapes alone, per ball), balls, opts, via_r (a move's via radius), t_menu. `only` (a set of ball keys, NET#PAD):
    those balls alone -- a round that plans again only the balls whose copper did not stand (carry). `all_faces` (with
    `escape_to`: the GROUP's ends, group_ends.py): each bus ball's escapes by EVERY face of the array -- which face a
    bus leaves by is the group's to choose, every bus's balls offered the same. `pattern` (the group's too): each such
    ball's PATTERN moves alone (escape_field_moves), a pair's leg those of them with room for the pair at the exit and
    no other."""
    import braid as te
    import escape_moves as em
    import fanout_from_plan as fp
    t0 = time.time()
    foot = pcb.footprints[ref]
    grid = em.grid_of(foot)
    if climb is None:
        climb = max(len(grid.xs), len(grid.ys))
    sn = namer(pcb)
    bus_s = {sn(n) for n in bus}
    oth_s = {sn(n) for n in others}
    drop_s = {sn(n) for n in drops} - bus_s - oth_s
    sz = _sizes(pcb, foot)
    skip = movable_refs(pcb, ref, bus)
    cache = {}

    def obs(nid, layer, via=False):
        # (a track clears every passive where it stands, a via only those that will not move: the cap step that
        # follows nudges a part only where it stays beneath its BGA (whole_route), off a via a little way, never off
        # a track run under its pads -- planned through, the zynq's decoupling caps under U1 were left on the others'
        # B.Cu escapes, four of them, and three balls had no way out round them after)
        key = (nid, layer, via)
        if key not in cache:
            cache[key] = te.build_obstacles(pcb, nid, {nid}, layer, margin=sz['cl'] + sz['tw'] / 2,
                                            skip_refs=skip if via else ())
        return cache[key]
    seg_memo = {}

    def seg_ok(nid, layer, a, b):
        # (obs(nid, layer).seg_clear(a, b), each leg asked once: a ball's climbs share their first stretches, and a
        # run on RUN is asked on each of its layers by enumerate_moves and again for its runs_on)
        key = (nid, layer, a[0], a[1], b[0], b[1])
        ok = seg_memo.get(key)
        if ok is None:
            ok = seg_memo[key] = obs(nid, layer).seg_clear(a, b)
        return ok
    # (VIAS IN PADS are in the menus -- a dog-bone's site, a via in the ball's own pad, a plane ball's drop -- unless
    # the board's chain recorded a SAME-NET PAD CLEARANCE (bga_fanout --same-net-pad-clearance > 0, #581): then a via
    # stands that far off every pad of its own net on the part, as every other step's vias do)
    import protected_nets as _pn
    snpc = _pn.read_snpc_for_pcb_data(pcb)
    own_pads = collections.defaultdict(list)
    if snpc > 0:
        for q_ in foot.pads:
            if q_.net_id:
                own_pads[q_.net_id].append(q_)

    def off_pads(p_, site, r):
        """a via of radius r at `site` stands `snpc` off every pad of ball p_'s net on the part"""
        for q_ in own_pads[p_.net_id]:
            dx, dy = abs(site[0] - q_.global_x), abs(site[1] - q_.global_y)
            if dx > 3.0 or dy > 3.0:
                continue
            if (q_.shape or '') == 'circle':
                d = math.hypot(dx, dy) - q_.size_x / 2
            else:
                d = math.hypot(max(dx - q_.size_x / 2, 0.0), max(dy - q_.size_y / 2, 0.0))
            if d - r < snpc - 1e-9:
                return False
        return True
    items, menu, balls, dmenu = {}, {}, {}, {}
    bus_layers = bus_route_layers(pcb)
    rls = run_layers()
    planes = set(plane_layers(pcb))
    pairs_ = array_pairs(foot, bus_s | oth_s, sn)
    partner = {leg: (pn if leg == nn else nn) for _b, (pn, nn) in pairs_.items() for leg in (pn, nn)}
    nid_of = {sn(n.name): i for i, n in pcb.nets.items() if n.name}
    # (the lanes laid already -- the array's nets this plan does not place, the bus's -- held clear of the drops)
    rays = lane_rays(pcb, grid, foot, {sn(p.net_name) for p in foot.pads if p.net_id and p.net_name}
                     - bus_s - oth_s - drop_s) if drop_s else []
    regions = zone_regions(pcb) if drop_s else None
    esc_copper = esc_faces = None
    # (`escape_to` and `far` per bus net, {short name: ref / face}: SEVERAL buses off this array, each net's escapes
    # toward its own bus's destination -- the combined plan of a shared array)
    dest_of = (lambda nm: escape_to.get(nm)) if isinstance(escape_to, dict) else (lambda nm: escape_to)
    far_of = (lambda nm: far.get(nm)) if isinstance(far, dict) else (lambda nm: far)
    if escape_to:
        # (`escape_to`, the other array's ref: the bus's menus are the ESCAPE FIELD's -- escape_field_moves -- its
        # faces a source's: toward the other array, or one on the ball's own half, never the far face)
        import escape_plan as ep
        esc_copper = ep.Copper(pcb, sz['tw'], sz['cl'])
        mid_ = ((grid.bbox[0] + grid.bbox[2]) / 2, (grid.bbox[1] + grid.bbox[3]) / 2)
        sides_ = {}

        def sides(dest):
            """(the face toward `dest`, the face away from it)"""
            if dest not in sides_:
                o_ = pcb.footprints[dest]
                ox_ = sum(q.global_x for q in o_.pads) / len(o_.pads) - mid_[0]
                oy_ = sum(q.global_y for q in o_.pads) / len(o_.pads) - mid_[1]
                sides_[dest] = (max(ep.DIRS, key=lambda d: ep.DIRS[d][0] * ox_ + ep.DIRS[d][1] * oy_),
                                min(ep.DIRS, key=lambda d: ep.DIRS[d][0] * ox_ + ep.DIRS[d][1] * oy_))
            return sides_[dest]

        def esc_faces(p, dest):
            if all_faces:
                return list(ep.DIRS)
            toward_, far_ = sides(dest)

            def half(d):
                ax = 0 if ep.DIRS[d][0] else 1
                return ((p.global_x, p.global_y)[ax] - mid_[ax]) * ep.DIRS[d][ax] >= -1e-6
            return [d for d in ep.DIRS if d != far_ and (d == toward_ or half(d))]
    for p in foot.pads:
        nm = sn(p.net_name or '')
        if not p.net_id or (nm not in bus_s and nm not in oth_s and nm not in drop_s):
            continue
        key = f'{nm}#{p.pad_number}'
        if only is not None and key not in only:
            continue
        items[key] = (nm, p)
        balls[key] = (p.global_x, p.global_y)
        if nm in drop_s:
            menu[key] = []
            dmenu[key] = _drops(pcb, grid, p, obs, sz, foot, rays, regions, fine=exit_rays)
            if snpc > 0:
                dmenu[key] = [d for d in dmenu[key] if off_pads(p, d.site, d.r)]
            continue
        home = next((L for L in pcb.board_info.copper_layers if L in p.layers), 'F.Cu')
        # (a plane layer only where a leg's layer is the plan's own and priced: on more routing layers than two a via
        # move's run is the colouring's to place, which prices none)
        lays = [home] + [L for L in (bus_layers if nm in bus_s else other_layers)
                         if L != home and not (rls and L in planes)]
        runs = lays[1:] if rls and len(lays) > 2 else None       # (a via move's run on RUN, clear on these)

        def clear(a, b, L, _n=p.net_id, _runs=runs):
            if L == RUN:
                return any(seg_ok(_n, L_, a, b) for L_ in _runs)
            return seg_ok(_n, L, a, b)
        centre = (p.global_x, p.global_y)
        # (a via is a barrel through EVERY copper layer, whatever layers the move runs on -- as a drop's is,
        # _via_clear: checked on the move's own layers alone, an other net's dog-bone via on F/B stood on the bus's
        # In2 tooth, which the plan never saw and the engine refused, the ball left bare -- zynq U1's TX_FRAME_P on
        # three layers)
        esc = bool(dest_of(nm)) and nm in bus_s
        far_nm = far_of(nm)
        if esc:
            moves = escape_field_moves(
                p, grid, home, lays, esc_faces(p, dest_of(nm)), esc_copper,
                lambda q, r, _n=p.net_id: _via_clear(pcb, obs, _n, q, r or sz['vr'], sz),
                lambda a, b, _n=p.net_id, _h=home: obs(_n, _h).seg_clear(a, b), sz['inpad'](p)[0], bool(runs),
                pattern=pattern, lines=lines)
        else:
            with contextlib.redirect_stdout(io.StringIO()):
                moves = em.enumerate_moves(
                    p, grid, [home, RUN] if runs else lays, clear,
                    lambda q, L, _n=p.net_id, _p=p, _c=centre: _via_clear(
                        pcb, obs, _n, q, sz['inpad'](_p)[0] if q == _c else sz['vr'], sz),
                    climb=climb if nm in bus_s else min(climb, CLIMB_OTHER), own_line=True, straight=True,
                    street=street, street_pitch=sz['stack'] + 1e-4, net=nm)
        if runs:
            # each run's layers: those its every stretch on RUN is clear on (enumerate_moves asked any one of them)
            kept = []
            for m in moves:
                if m.layer != RUN:
                    kept.append(m)
                    continue
                m.runs_on = tuple(L_ for L_ in runs
                                  if all(seg_ok(p.net_id, L_, a, b) for a, b, l_ in m.legs if l_ == RUN))
                if m.runs_on:
                    kept.append(m)
            moves = kept
        if snpc > 0:
            moves = [m for m in moves if m.site is None or off_pads(p, m.site, sz['vr'])]
        if not esc:
            # (one climb per exit row -- never across an escape field's SITES: which site is the plan's choice)
            moves = fp.dedupe_climbs(moves)
        if nm in bus_s and far_nm:
            # (a bus ball never leaves by the far face)
            moves = [m for m in moves if m.direction != far_nm]
        if nm in bus_s and vias_only:
            # (ESCAPE_VIAS at this array: the bus's escapes through a via -- a dog-bone, a via in its pad -- whose lane
            # leaves on whichever routing layer the solve gives it; a ball with none keeps what it has)
            moves = [m for m in moves if m.kind != 'surface'] or moves
        if nm in partner and partner[nm] in nid_of:
            # a pair leg's escapes with room for the PAIR at the exit, as the bus's own fanout keeps them
            # (fanout_from_plan.pair_exit_clear); all of them where none has. A run on RUN keeps the layers with room
            room_ = {}
            for m in moves:
                if m.layer == RUN:
                    room = tuple(L_ for L_ in m.runs_on
                                 if fp.pair_exit_clear(pcb, p.net_id, nid_of[partner[nm]], m, layer=L_))
                    if room:
                        room_[id(m)] = room
                elif fp.pair_exit_clear(pcb, p.net_id, nid_of[partner[nm]], m):
                    room_[id(m)] = None
            for m in moves:
                if room_.get(id(m)):
                    m.runs_on = room_[id(m)]
            # (`pattern`: none with room is none offered -- the ball's whole menu is the caller's to ask for)
            moves = [m for m in moves if id(m) in room_] or ([] if pattern else moves)
        menu[key] = moves
        dmenu[key] = []
    # (no option into a laid pair's POCKET (laid_pockets), the pair's own room to close past its teeth: an escape there
    # is walled in once it closes -- the zynq DDR's U1 NetR3_2 escaped on B.Cu between DDR3_DQS1's B teeth, 0.77 mm
    # apart, A14 over its end -- and a drop's via there stands in the pair's way -- U2's GND B9 dropped between
    # DDR3_DQS1's berths, the whole route's audit found the pair 0.03 mm into it)
    laid_s = {sn(p.net_name) for p in foot.pads if p.net_id and p.net_name} - bus_s - oth_s - drop_s
    pockets = laid_pockets(pcb, grid, laid_s)
    if pockets:
        for key in menu:
            menu[key] = [m for m in menu[key]
                         if not in_pockets(pockets, m.legs or [], [m.site] if m.site is not None else [])]
            dmenu[key] = [d for d in dmenu[key]
                          if not in_pockets(pockets, [d.stub + (d.layer,)] if d.stub else [], [d.site])]
    # (...and no escape whose lane would go on from between two laid lanes that close round it: lane_walls -- an
    # escape with no other option keeps its menu, the plan's to weigh)
    walls = lane_walls(pcb, grid, laid_s)
    if walls:
        for key in menu:
            menu[key] = [m for m in menu[key] if not exit_walled(walls, m)] or menu[key]
    t_menu = time.time() - t0
    # (a plane ball's straps too: the human's U5 A4 joins A5 down its column, A5 and A6 sharing one via off the
    # array's edge, the bus's lanes in every gap beside them)
    straps = _straps(pcb, grid, items, obs)
    if pockets:
        straps = {k: [s for s in v if not in_pockets(pockets, [(s.a, s.b, s.layer)], [])] for k, v in straps.items()}
    # every ball's options, escapes first (their conflicts are the whole route's own, by index); each with its copper
    # at its real size: a via in the ball's own pad the engine's clamped one, any other the rung's
    opts = {}
    for key, (nm, p) in items.items():
        own = [(p.global_x, p.global_y)]
        o = []
        for m in menu[key]:
            vias = []
            if m.site is not None:
                r, dr = sz['inpad'](p) if tuple(m.site) == own[0] else (sz['vr'], sz['vdr'])
                vias = [(tuple(m.site), r, dr)]
                # (its radius ON THE MOVE: kept by id(move), a menu widened or rebuilt left its dead moves' ids
                # behind, Python gave a new move such an id, and it took the dead one's radius -- the group's model
                # then differed with the string hash seed, in six groups of vias of one array)
                m.via_r = r
            o.append(('escape', m, Piece(nm, list(m.legs), vias, own)))
        o += [('strap', s, Piece(nm, [(s.a, s.b, s.layer)], [], [s.a, s.b])) for s in straps.get(key, ())]
        o += [('drop', d, Piece(nm, [d.stub + (d.layer,)] if d.stub else [], [(d.site, d.r, d.dr)], own))
              for d in dmenu[key]]
        if escape_to and nm in oth_s:
            # (another ball's dog-bone alone, C_STUB: where no escape of its fits round the bus's teeth)
            home_ = next((L for L in pcb.board_info.copper_layers if L in p.layers), 'F.Cu')
            for sx in (-1, 1):
                for sy in (-1, 1):
                    q_ = (p.global_x + sx * grid.pitch_x / 2.0, p.global_y + sy * grid.pitch_y / 2.0)
                    if not _via_clear(pcb, obs, p.net_id, q_, sz['vr'], sz) or \
                            not obs(p.net_id, home_).seg_clear(own[0], q_) or \
                            (pockets and in_pockets(pockets, [(own[0], q_, home_)], [q_])):
                        continue
                    s_ = Stub(q_, own[0], home_)
                    o.append(('stub', s_, Piece(nm, [(own[0], q_, home_)], [(q_, sz['vr'], sz['vdr'])], own)))
        opts[key] = o
    by_net = collections.defaultdict(list)
    for key, (nm, _p) in items.items():
        by_net[nm].append(key)
    legs = [(b_, by_net[pn][0], by_net[nn][0]) for b_, (pn, nn) in sorted(pairs_.items())
            if len(by_net.get(pn, ())) == 1 and len(by_net.get(nn, ())) == 1]
    return types.SimpleNamespace(foot=foot, grid=grid, bus_s=bus_s, oth_s=oth_s, drop_s=drop_s, sz=sz, items=items,
                                 menu=menu, balls=balls, opts=opts, straps=straps, dmenu=dmenu, t_menu=t_menu,
                                 pairs=legs, lane_faces=lane_faces(rays))


_SOLVED = {}         # a plan's model, with what phase 2 prices -> both phases' answers (plan_array: one model, one answer)
# (the SOLVED_KEEP answers used last: each holds every option variable's value and hint, and a fanout meets a model
# again soon after it is made -- kept for the process's life, they grew without bound)
SOLVED_KEEP = 16


def _keep_solved(skey, ans):
    """`ans` kept for the model `skey`, the oldest answers past SOLVED_KEEP dropped"""
    _SOLVED.pop(skey, None)
    _SOLVED[skey] = ans
    while len(_SOLVED) > SOLVED_KEEP:
        # (off a snapshot of the keys: a plan on another thread may be keeping its own answer)
        _SOLVED.pop(list(_SOLVED)[0], None)
# each array's last plan in the process, by ball: the option it took, by its copper (_opt_sig) -- recorded where the
# plan is made (_finish, on the thread that asks, in the program's order), so the next plan of the array starts from it
_LAST_PLAN = {}


def _opt_sig(kind, o, pc):
    """an option by what it is -- its kind, its copper (legs, vias and their sizes) to the micron, and an escape's own
    shape, face, layer, exit, climb and street (a surface escape has no legs): the same option in another plan's menu
    of the array"""
    r_ = lambda q: (round(q[0], 3), round(q[1], 3)) if q is not None else None
    sh = (o.kind, o.direction, o.layer, r_(o.exit_pt), r_(o.site), o.climb, o.street) if kind == 'escape' else \
        (getattr(o, 'layer', None), bool(getattr(o, 'inpad', False)))
    return (kind, sh, tuple((r_(a), r_(b), L) for a, b, L in pc.legs),
            tuple((r_(s[0]), round(s[1], 4), round(s[2], 4)) for s in pc.vias))


_GEO = {}            # a plan's geometry pairs by the digest of its options (plan_array), the GEO_KEEP latest
GEO_KEEP = 4


def _solved(skey):
    """the answer kept for the model `skey`; None where it is not known (the plan then solves it itself)"""
    got = _SOLVED.get(skey)
    if got is not None:
        _keep_solved(skey, got)              # (used: the latest again)
    return got


def _status_name(st):
    """a CP-SAT status's name (an answer kept has no solver of its own to ask)"""
    from ortools.sat.python import cp_model
    return cp_model.CpSolver().StatusName(st)


def _solved_key(mdl, ref, keys, served, items, bus_s, oth_s, drop_s, bm, rank, wt, cvals, v, site_used, rls,
                batches, workers):
    """the key a plan's answers are kept by in _SOLVED: the model's bytes as phase 1 meets it (its options, conflicts and
    hints), hashed, with what the phases read beside it -- the tiers' balls, their ranks and weights, each option's cost,
    the sites' vias, the work. (Not phase 2's start from the array's last plan: the model's first answer is its answer,
    whatever plan of the array came before)"""
    import hashlib
    import tempfile
    fd, path = tempfile.mkstemp(suffix='.pb')
    os.close(fd)
    try:
        mdl.ExportToFile(path)
        with open(path, 'rb') as f:
            h = hashlib.sha1(f.read())
    finally:
        os.remove(path)
    legs = sorted({k for _b, kp, kn in bm.pairs for k in (kp, kn)}, key=repr)
    h.update(repr((ref, [(k, k in served, items[k][0] in bus_s, items[k][0] in oth_s, items[k][0] in drop_s,
                          rank.get(k), wt.get(k), cvals[k], [x.Index() for x in v[k]]) for k in keys],
                   legs, [u.Index() for u in site_used], C_VIA, P1_DET, rls, batches or SOLVE_BATCHES,
                   workers or SOLVE_WORKERS, SUBSOLVERS, SUBSOLVERS_RUNS,
                   SOLVE_BATCHES_STARTED)).encode())
    return h.hexdigest()


def plan_array(pcb, ref, bus, others, other_layers, far=None, prefer=None, drops=(), climb=CLIMB, street=2,
               batches=None, workers=None, log=print, debug=False, only=None, hands=None,
               vias_only=False, exit_rays=False, escape_to=None, stack_bus=False,
               avoid=None, ban=frozenset(), colour_tries=None, exact_layers=False):
    """(hints, report): one planned move for every signal ball of `ref` on `pcb` -- the bus's nets (`bus`, full or
    short names) escaping on F.Cu/B.Cu, never by the `far` face; `others` escaping on `other_layers`, or a multi-ball
    net's ball strapped to a neighbour of its net; and every ball of the plane nets `drops` dropped to its plane --
    chosen together. `prefer` {bus net: measured tooth}: the teeth the bus is preferred to keep. `hands` {a pair's P
    leg (short name): (hand, arriving)}: the HAND its two exits are to have (pairs.hand -- the other array's, so the
    pair needs no crossover between them), wherever the array has an exit pair of it. hints: {ball position: strict
    planned move} for the joint escape engine.

    The menus are EVERY move escape_moves has: the surface escapes (along an adjacent gap, and straight out along
    the ball's own line), dog-bones and vias-in-pad, the CLIMBS -- a dog-bone or via-in-pad whose run first travels
    along a gap or the ball's own line, up to `climb` pitches (None: the whole array), before it leaves -- and the
    STREET dog-bones (a via in an empty band of the array, `street` sites along a lane; the whole route's 2). The
    escapes' conflicts are the whole route's own (select_moves._conflicts as its ends take them: strict, an F exit
    stacked over a B one allowed). `exit_rays` (the bus step's whole-array plans, route_bus: of the balls its comb
    refused, and of the pairs its comb split): a plane ball's drop is held out of the bus's teeth's and berths' ways out
    (exit_ray_conflicts), each gap site offering a finer via too.

    `escape_to` (the other array's ref, or {bus net: its ref}): the bus's menus are the ESCAPE FIELD's
    (escape_field_moves: escape_plan's escapes from every site a ball could take -- a gap point, its pad, none), so its
    dog-bones are chosen with every other ball's escape and drop. `avoid` {bus net: [(x, y), ..]}: the teeth its rounds
    found crowded, each exit within a pitch of one paid C_AVOID -- of one given as (x, y, layer), each such exit
    leaving on that layer. `ban` {(ball, _opt_sig)}: options the menus leave
    out -- a bus ball's run no colouring held, planned again without it, and the plan that leaves fewer balls without
    an option kept (`colour_tries`: the runs the plan before left uncoloured). `exact_layers`: each run's LAYER the
    plan's own choice -- a literal per run and layer, no two runs of a lane, a crossing or an exit cluster on one, a
    pair's two legs on one -- where the capacity rows only count them: the plan asked again so when a bus ball's run
    is left with no layer (a run that crosses more runs than there are layers meets every row's count and has no
    layer left; the model six times the size, so not the first plan)."""
    args_ = dict(locals())
    import conflict_groups as cg
    import select_moves as sm
    import source_realize as sr
    from ortools.sat.python import cp_model
    t0 = time.time()
    bm = build_menus(pcb, ref, bus, others, other_layers, far=far, drops=drops, climb=climb, street=street,
                     only=only, vias_only=vias_only, exit_rays=exit_rays, escape_to=escape_to)
    bus_s, oth_s, drop_s, sz = bm.bus_s, bm.oth_s, bm.drop_s, bm.sz
    if ban:
        for k_ in list(bm.opts):
            bm.opts[k_] = [x_ for x_ in bm.opts[k_] if (k_, _opt_sig(*x_)) not in ban]
            bm.menu[k_] = [o_ for kind_, o_, _pc in bm.opts[k_] if kind_ == 'escape']
    avoid_r = max(bm.grid.pitch_x, bm.grid.pitch_y)          # (an exit this near a crowded tooth: C_AVOID)
    # a crowded tooth ON ITS LAYER (`avoid`'s (x, y, layer)): a bus ball's run on RUN leaving within a pitch of it gives
    # that layer up -- its other layers still the colouring's -- where the price alone
    # (cost) is paid by whichever run takes the exit at all, on any of its layers: a whole-array plan priced so took the
    # same exit as unpriced, and its run was coloured onto the blocked layer again
    narrowed = 0
    if avoid and any(len(a_) > 2 for v_ in avoid.values() for a_ in v_):
        for k_ in list(bm.opts):
            places_ = [a_ for a_ in avoid.get(bm.items[k_][0], ()) if len(a_) > 2]
            if not places_ or bm.items[k_][0] not in bus_s:
                continue
            keep_ = []
            for kind_, o_, pc_ in bm.opts[k_]:
                if kind_ == 'escape' and o_.layer == RUN and getattr(o_, 'runs_on', None):
                    bad_ = {a_[2] for a_ in places_ if a_[2] in o_.runs_on
                            and math.hypot(o_.exit_pt[0] - a_[0], o_.exit_pt[1] - a_[1]) < avoid_r}
                    if bad_:
                        narrowed += 1
                        o_.runs_on = tuple(L_ for L_ in o_.runs_on if L_ not in bad_)
                        if not o_.runs_on:
                            continue
                keep_.append((kind_, o_, pc_))
            bm.opts[k_] = keep_
            bm.menu[k_] = [o_ for kind_, o_, _pc in keep_ if kind_ == 'escape']
        if narrowed:
            log(f'  whole-array escape of {ref}: {narrowed} bus run option(s) gave up a crowded tooth\'s layer')
    planes_ = set(plane_layers(pcb))             # (a leg on one priced: C_PLANE_LAYER)
    rls = run_layers()                # (more routing layers than two: the via moves' runs on RUN)
    items, menu, balls, opts, straps, dmenu, t_menu = (bm.items, bm.menu, bm.balls, bm.opts, bm.straps, bm.dmenu,
                                                       bm.t_menu)
    import braid as _bd
    _bd.forget_obstacles()           # (the menus were the models' last use here: their memory goes to the solve)
    n_moves = sum(len(v) for v in menu.values())
    # the escapes' conflicts, the whole route's own relation as groups that all conflict pairwise (conflict_groups:
    # select_moves._conflicts pair by pair was 129 s and 3.1 million pairs for the bus alone with every move kind).
    # Every crossing counts (xing 2, asked for here: select_moves' default counts one only where a move climbs) -- a
    # plan of every ball has plain escapes out of
    # the interior, and two of those cross (zynq U1, the plan at the default rule: 85 of 86 clashing pairs in the laid
    # geometry were two plain surface escapes crossing on F.Cu).
    # One net's two ESCAPES conflict as two nets' do: the engine lays each ball's escape on its own, on a raster
    # that carries no nets (zynq U1: five VCC_1V0 balls planned out through one gap, four laid nowhere)
    t1 = time.time()
    tags = {}
    groups, bigroups, gpairs = cg.conflict_groups(menu, stack=True, stack_pitch=sz['stack'],
                                                  via_r=lambda m: getattr(m, 'via_r', sz['vr']),
                                                  reach_extra=sz['cl'] + sz['tw'] / 2, xing=2, tags=tags)
    pairs = [(a[0], a[1], b[0], b[1]) for a, b in sorted(gpairs)]
    # (the groups of runs on RUN alone -- their lane, crossing and exit conflicts -- are the K layers' capacity; one a
    # via's site or reach states as well is strict: a via stands on every layer)
    on_run = lambda g: bool(tags.get(g)) and all(t[0] in ('lane', 'crossing', 'exit') and t[1] == RUN
                                                 for t in tags[g])
    cap_g = {tuple(sorted(g)) for g in groups if on_run(g)}
    cap_b = {(tuple(sorted(a)), tuple(sorted(b))) for a, b in bigroups if on_run((a, b))}
    del tags
    # built in a canonical order (select_moves._conflicts' note: the order constraints reach the CP-SAT picks its answer)
    # (a group of one ball's moves alone says nothing its own AtMostOne does not)
    groups = sorted((sorted(g) for g in groups if len({m_[0] for m_ in g}) > 1), key=repr)
    bigroups = sorted(((sorted(a), sorted(b)) for a, b in bigroups if len({m_[0] for m_ in a | b}) > 1), key=repr)
    t_lane = time.time() - t1
    t1 = time.time()
    # a strap or a drop against every other ball's option: geometry at the laid sizes, the candidates found through a
    # 1 mm cell index of the options' boxes (a climb's box spans the array; the all-pairs loop was 10^7 box tests).
    # The pairs are a function of the options' copper, their nets and the sizes alone: a plan of the same array on the
    # same menus -- each source-realize trial's, each berth plan's -- takes them from the last such plan (_GEO)
    gk_ = hashlib.blake2b(digest_size=20)
    gk_.update(repr((SHARE_TOL, sz['tw'], sz['cl'], sz['h2h'], sz['d_seg'], sz['r_home'], sz['grow'])).encode())
    for k in sorted(opts):
        gk_.update(repr((k, items[k][0], [(kind_, pc_.legs, pc_.vias, pc_.balls) for kind_, _o, pc_ in opts[k]])).encode())
    gk_ = gk_.hexdigest()
    known = _GEO.get(gk_)
    index = collections.defaultdict(list)
    boxes = {}
    geo = {}                         # (_hit's view of each option's copper, made once)
    for k in sorted(opts):
        for i, (_kind, _o, pc) in enumerate(opts[k]):
            bx = boxes[(k, i)] = _box(pc, sz['grow'])
            if known is not None:
                continue
            for cx in range(int(math.floor(bx[0])), int(math.floor(bx[2])) + 1):
                for cy in range(int(math.floor(bx[1])), int(math.floor(bx[3])) + 1):
                    index[(cx, cy)].append((k, i))
    n0_ = len(pairs)
    for k in (sorted(opts) if known is None else ()):
        for i, (kind, _o, pc) in enumerate(opts[k]):
            if kind == 'escape':
                continue
            bx = boxes[(k, i)]
            seen = set()
            for cx in range(int(math.floor(bx[0])), int(math.floor(bx[2])) + 1):
                for cy in range(int(math.floor(bx[1])), int(math.floor(bx[3])) + 1):
                    for (k2, j) in index[(cx, cy)]:
                        if k2 == k or (k2, j) in seen or (opts[k2][j][0] != 'escape' and (k2, j) <= (k, i)):
                            continue
                        seen.add((k2, j))
                        bx2 = boxes[(k2, j)]
                        if bx[0] > bx2[2] or bx2[0] > bx[2] or bx[1] > bx2[3] or bx2[1] > bx[3]:
                            continue
                        if _hit(pc, opts[k2][j][2], items[k][0] == items[k2][0], sz, geo):
                            pairs.append((k, i, k2, j))
    del geo
    if known is None:
        _GEO[gk_] = pairs[n0_:]
        while len(_GEO) > GEO_KEEP:
            _GEO.pop(next(iter(_GEO)))
    else:
        pairs += known
    if exit_rays:
        pairs += exit_ray_conflicts(opts, {k: items[k][0] for k in items}, bus_s, sz['vr'], bm.grid,
                                    bus_of=escape_to if isinstance(escape_to, dict) and len(set(escape_to.values())) > 1
                                    else None)
    t_geo = time.time() - t1
    log(f'  whole-array escape of {ref}: {len(items)} balls ({sum(1 for k in items if items[k][0] in bus_s)} bus, '
        f'{sum(1 for k in items if items[k][0] in drop_s)} plane), {n_moves} moves, '
        f'{sum(len(v) for v in straps.values())} straps, {sum(len(v) for v in dmenu.values())} drops, '
        f'{len(groups)} conflict cliques + {len(bigroups)} bicliques + {len(pairs)} pairs, {time.time() - t0:.0f} s '
        f'(menus {t_menu:.0f}, '
        f'conflicts {t_lane:.0f}, geometry {t_geo:.0f})')

    def cost(key, kind, o):
        nm, p = items[key]
        if kind == 'stub':
            return int(round(C_STUB + C_MM * math.hypot(o.site[0] - p.global_x, o.site[1] - p.global_y)))
        if kind == 'strap':
            # (a plane ball's strap is priced as the via it does without: it shares its neighbour's way down only
            # where the ball has none of its own, as a drop or a shared gap via)
            return int(round(C_MM * o.length + (C_VIA if nm in drop_s else 0)))
        if kind == 'drop':
            # (a gap drop's via is paid by its SITE, once for every ball of its net dropped there: site_used)
            if o.inpad:
                return C_VIA + C_VIP
            return int(round(C_MM * math.hypot(o.site[0] - p.global_x, o.site[1] - p.global_y)
                             + (C_LANE_FACE if beyond_faces(bm.grid, o.site) & bm.lane_faces else 0)
                             + (C_FINE_VIA if o.r < sz['vr'] - 1e-9 else 0)))
        ln = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b, _L in (o.legs or [])) or \
            math.hypot(o.exit_pt[0] - p.global_x, o.exit_pt[1] - p.global_y)
        c = C_VIA * o.vias + C_MM * ln + C_PLANE_LAYER * sum(1 for _a, _b, L_ in (o.legs or []) if L_ in planes_)
        if escape_to and nm in bus_s and o.kind == 'via_in_pad':
            # (an escape field's via in its pad, as a drop's: filled and capped at the fab, and a barrel on both of the
            # ball's lines on every layer -- the field's lanes run there. Unpriced, the plan took 12 of zynq U1's 48)
            c += C_VIP
        g = (prefer or {}).get(nm) if nm in bus_s else None
        if g:
            t = g['tooth']
            c += C_DEV_MM * math.hypot(o.exit_pt[0] - t[0], o.exit_pt[1] - t[1])
            c += C_DEV_KIND * ((o.direction != g.get('direction')) + _off_layer(o, g.get('layer'))
                               + (o.kind != g.get('kind')))
        if avoid and nm in bus_s and any(math.hypot(o.exit_pt[0] - a_[0], o.exit_pt[1] - a_[1]) < avoid_r
                                         and (len(a_) < 3 or not _off_layer(o, a_[2]))
                                         for a_ in avoid.get(nm, ())):
            c += C_AVOID
        return int(round(c))
    mdl = cp_model.CpModel()
    keys = sorted(opts)
    v = {key: [mdl.NewBoolVar(f'v{k}_{i}') for i in range(len(opts[key]))] for k, key in enumerate(keys)}
    served = {}                      # the ball takes one of its options (an escape, a strap or a drop)
    for k, key in enumerate(keys):
        if v[key]:
            served[key] = mdl.NewBoolVar(f's{k}')
            mdl.Add(sum(v[key]) == served[key])
    def capacity(members):
        # at most as many of the runs on RUN as they have layers between them -- for each set of layers some of them
        # are confined to, those as many (Hall's condition: a lane's runs are intervals, whose overlapping ones are
        # coloured with as many layers as overlap, and no more are needed); colour_runs gives each its layer
        by_set = collections.defaultdict(list)
        for k, i in members:
            by_set[frozenset(opts[k][i][1].runs_on)].append(v[k][i])
        # ...and a BUS run alone in its group, as on two layers: its layer is the solve's, at its via end (the relayer
        # moves the run there), and every layer must stay free for it. Two of the bus's runs sharing a lane on two
        # layers held the zynq's solve to 16 pairs of runs kept apart; the other nets' runs beside the bus's on the
        # inner layers banned it 51 layers -- each time the whole solve proved no plan (without those bans, a plan)
        # (`stack_bus`: the combined plan of a shared array, every bus's runs laid as planned -- a bus run counted as
        # another net's, so a lane carries as many of the buses' runs as it has layers, as a bus alone stacks them, and
        # colour_runs gives each its layer)
        free_bus = lambda k, i: not stack_bus and items[k][0] in bus_s   # noqa: E731
        bus_l = [v[k][i] for k, i in members if free_bus(k, i)]
        oth_l = [v[k][i] for k, i in members if not free_bus(k, i)]
        if len(bus_l) > 1:
            mdl.AddAtMostOne(bus_l)
        if bus_l and oth_l:
            k_ = len(frozenset().union(*by_set))
            mdl.Add(k_ * sum(bus_l) + sum(oth_l) <= k_)
        for S in sorted(set(by_set) | {frozenset().union(*by_set)}, key=lambda s_: (len(s_), sorted(s_))):
            lits = [l_ for s_, ls in by_set.items() if s_ <= S for l_ in ls]
            if len(lits) > len(S):
                mdl.Add(sum(lits) <= len(S))

    # (more routing layers than two: each group's members stand together at most `cap` -- 1 a strict clique, K a run
    # capacity -- and an option that excludes several of them says so in ONE row, cap x it + those <= cap, where a
    # clause a member was 300 MB of the zynq U1 plan's phase 2: exclude)
    cover, memb = [], collections.defaultdict(list)

    def covers(g, cap):
        if rls:
            cover.append((g, cap))
            for x in g:
                memb[x].append(len(cover) - 1)
    for g in groups:
        if tuple(g) in cap_g:
            capacity(g)
            covers(g, len(frozenset().union(*(opts[k][i][1].runs_on for k, i in g))))
        else:
            mdl.AddAtMostOne([v[k][i] for k, i in g])
            covers(g, 1)
    for a, b in bigroups:
        if (tuple(a), tuple(b)) in cap_b:
            g = sorted(set(a) | set(b))
            capacity(g)
            covers(g, len(frozenset().union(*(opts[k][i][1].runs_on for k, i in g))))
    if rls:
        # ...but a differential pair's two legs leave on ONE layer, so two of its legs' runs in one such group -- a lane,
        # an exit cluster, a crossing, where they must stand on two -- are never chosen together (zynq U5's TX_FRAME on
        # four layers: its legs' runs met in one, and no colouring held the pair)
        leg_of = {k_: (n_p, s_) for n_p, (_b, kp, kn) in enumerate(bm.pairs) for k_, s_ in ((kp, 0), (kn, 1))}
        seen_ = set()
        # (in a canonical order, as the groups above: cap_g and cap_b are SETS of ball names, iterated in the string
        # hash's order -- the model's rows came in a different order each process, and so did the plan: zynq U1's
        # LVDS bus on four layers, one realize run with two hash seeds, 26 of its 47 teeth the same)
        for g in [list(g) for g in sorted(cap_g)] + [sorted(set(a) | set(b)) for a, b in sorted(cap_b)]:
            legs_ = collections.defaultdict(lambda: ([], []))
            for k, i in g:
                if k in leg_of:
                    legs_[leg_of[k][0]][leg_of[k][1]].append((k, i))
            for n_p, (ps, ns) in sorted(legs_.items()):
                key_ = (n_p, tuple(sorted(ps)), tuple(sorted(ns)))
                if ps and ns and key_ not in seen_:
                    seen_.add(key_)
                    mdl.AddAtMostOne([v[k][i] for k, i in sorted(ps) + sorted(ns)])
    # (`exact_layers`) EACH RUN'S LAYER THE PLAN'S: a literal per layer the run may take, one of them when it is
    # chosen; in each capacity group -- a lane's overlapping runs, a crossing's, an exit cluster's -- at most one run a
    # layer; a pair's two legs' runs on one. The rows above count a group's runs against its layers, group by group,
    # and colour_runs then gives each chosen run its layer: a run through many groups can meet every count and still
    # find each of its layers taken by one neighbour or another -- an inner ball's run out across twenty-three others
    # on seven layers. Here such a plan is no plan, and the solve serves the ball another way or names it unserved
    ylay = {}
    if rls and exact_layers:
        cap_all = [list(g) for g in sorted(cap_g)] + [sorted(set(a) | set(b)) for a, b in sorted(cap_b)]
        pair_keys = {k_ for _b, kp, kn in bm.pairs for k_ in (kp, kn)}
        on_run_ = sorted({x for g in cap_all for x in g}
                         | {(k, i) for k in sorted(pair_keys) for i, (kind, o, _pc) in enumerate(opts.get(k, ()))
                            if kind == 'escape' and o.layer == RUN})
        for n_, (k, i) in enumerate(on_run_):
            ylay[(k, i)] = {L: mdl.NewBoolVar(f'y{n_}_{L}') for L in opts[k][i][1].runs_on}
            mdl.Add(sum(ylay[(k, i)].values()) == v[k][i])
        for g in cap_all:
            by_l = collections.defaultdict(list)
            for x in g:
                for L, y in ylay[x].items():
                    by_l[L].append(y)
            for L in sorted(by_l):
                if len(by_l[L]) > 1:
                    mdl.AddAtMostOne(by_l[L])
        for _b, kp, kn in bm.pairs:
            zs = []
            for k in (kp, kn):
                mine = [(i, d) for (k_, i), d in sorted(ylay.items()) if k_ == k]
                zs.append((sum(v[k][i] for i, _d in mine),
                           {L: sum(d[L] for _i, d in mine if L in d) for L in sorted({L for _i, d in mine for L in d})}))
            (rp, zp), (rn, zn) = zs
            for L in sorted(set(zp) | set(zn)):
                # (both legs on RUN: on the same layer)
                mdl.Add(zp.get(L, 0) - zn.get(L, 0) <= 1 - rn)
                mdl.Add(zn.get(L, 0) - zp.get(L, 0) <= 1 - rp)

    def exclude(lit, others):
        # `lit` excludes every option of `others`: the groups holding several of them a row each, the most first
        # (exact: a group's members never stand together past its cap anyway), the rest a clause each. Each group's
        # count of them is kept as rows take them, the most (the lowest group first among equals) off a heap whose
        # stale counts are passed over -- the same rows in the same order as counting them all again for every row
        rem = set(others)
        cnt = collections.Counter(itertools.chain.from_iterable(memb.get(x, ()) for x in rem))
        heap = [(-n, gi) for gi, n in cnt.items() if n >= 2]
        heapq.heapify(heap)
        while rem and heap:
            n, gi = heapq.heappop(heap)
            if -n != cnt[gi]:
                continue
            g, cap = cover[gi]
            hit = [x for x in g if x in rem]
            mdl.Add(cap * lit + sum(v[k][i] for k, i in hit) <= cap)
            rem.difference_update(hit)
            for gj, d_ in collections.Counter(itertools.chain.from_iterable(memb.get(x, ()) for x in hit)).items():
                cnt[gj] -= d_
                if cnt[gj] >= 2:
                    heapq.heappush(heap, (-cnt[gj], gj))
        for k, i in sorted(rem):
            mdl.AddBoolOr([lit.Not(), v[k][i].Not()])
    # every move of A against every move of B: y_A covers A's moves, y_B B's, and a move on both sides stands with
    # them -- at most one of y_A, y_B and those. (Runs on RUN through one crossing: their K layers' capacity, stated
    # above. More routing layers than two: y_A covers the smaller side, or the side that is one via site's moves --
    # never two at once -- where a move stands on both, and excludes the rest by the groups, exclude)
    one_site = lambda side: len({sm._site_key(opts[k][i][1]) for k, i in side} - {None}) == 1 and \
        all(opts[k][i][1].site is not None for k, i in side)
    for n_, (a, b) in enumerate(bigroups):
        if (tuple(a), tuple(b)) in cap_b:
            continue
        both = set(a) & set(b)
        if rls and (not both or one_site(a) or one_site(b)):
            x_side, y_side = (a, b) if (one_site(a) if both else len(a) <= len(b)) else (b, a)
            if len(x_side) == 1:
                y = v[x_side[0][0]][x_side[0][1]]
            else:
                y = mdl.NewBoolVar(f'bi{n_}')
                for k, i in x_side:
                    mdl.AddImplication(v[k][i], y)
            exclude(y, [m_ for m_ in y_side if m_ not in set(x_side)])
            continue
        lits = [v[k][i] for k, i in sorted(both)]
        for side, tag in ((a, 'a'), (b, 'b')):
            only = [m_ for m_ in side if m_ not in both]
            if only:
                y = mdl.NewBoolVar(f'bi{n_}{tag}')
                for k, i in only:
                    mdl.AddImplication(v[k][i], y)
                lits.append(y)
        if len(lits) > 1:
            mdl.AddAtMostOne(lits)
    if rls:
        by_o = collections.defaultdict(list)
        for a, i, b, j in pairs:
            by_o[(a, i)].append((b, j))
        for (a, i), others_ in sorted(by_o.items()):
            exclude(v[a][i], others_)
    else:
        for a, i, b, j in pairs:
            mdl.AddBoolOr([v[a][i].Not(), v[b][j].Not()])
    # a differential PAIR leaves the array together -- a constraint of the one solve rather than a repair after a
    # choice made ball by ball: its two legs served together, each by an escape whose
    # partner's is of the same face and layer with its exit a neighbour (within 1.3 pitches), and no other ball's exit
    # of that face and layer between the two. Planned leg by leg, the zynq U1's array left 6 of its 19 pairs split by
    # another net's tooth and 7 with a leg on each layer
    reach = 1.3 * max(bm.grid.pitch_x, bm.grid.pitch_y)
    along = lambda m: m.exit_pt[0 if m.direction in ('up', 'down') else 1]
    by_cls = collections.defaultdict(list)          # (face, layer) -> [(exit along the face, ball, option)], sorted
    for key in keys:
        for i, (kind, o, _pc) in enumerate(opts[key]):
            if kind == 'escape':
                by_cls[(o.direction, o.layer)].append((along(o), key, i))
    at_cls = collections.defaultdict(list)          # (face, layer, exit along it) -> [(ball, option)] there
    for cls in by_cls:
        by_cls[cls].sort()
        for at_, k3, l3 in by_cls[cls]:
            at_cls[cls + (at_,)].append((k3, l3))
    # The rules are stated over EXIT SLOTS and POSITIONS, which is all they see of an option. A leg's escapes of one
    # face, layer and exit point are one slot (a literal: the slot's options summed, the leg taking one); two mated
    # slots force the SPAN of every position between them (a literal per pair and position); and a spanned position
    # bars its OCCUPANCY (a literal per face, layer and position, which every exit there implies -- the legs' own
    # included, as neither leg can stand between its own two exits). A clause per mated couple of options and exit
    # between was 197,803 clauses on zynq U1's three layers, and the solve past 1.5 GB; the span's bar per exit there,
    # 101,871 implications on U5
    occ = {}
    from pairs import hand as _pairs_hand
    hand_rep = {'held': [], 'free': []}     # the pairs held to their asked hand; those the array has no exit pair of it

    def occupied(pos):
        z = occ.get(pos)
        if z is None:
            z = occ[pos] = mdl.NewBoolVar(f'occ{len(occ)}')
            for k3, l3 in at_cls[pos]:
                mdl.AddImplication(v[k3][l3], z)
        return z
    for n_p, (_b, kp, kn) in enumerate(bm.pairs):
        mdl.Add(sum(v[kp]) == sum(v[kn]))
        # (a pair's two dog-bones alone together, or its two escapes: one leg left inside the array and the other out
        # of it is no pair's start)
        stub_ = lambda k_: [v[k_][i] for i, (kind, _o, _pc) in enumerate(opts[k_]) if kind == 'stub']   # noqa: E731
        if stub_(kp) or stub_(kn):
            mdl.Add(sum(stub_(kp)) == sum(stub_(kn)))
            # ...and, both dog-boned, by the SAME diagonal: the two vias then stand as the balls do, a pitch apart
            # side by side, where the later bus whose teeth they are leaves them as a pair. Free, a later bus's
            # pairs on one ball column were dog-boned to the north-west and the north-east, their vias 1.13 mm apart,
            # and that bus's ends had no clean option for them
            def diag_(k_, o_):
                return o_.site[0] > items[k_][1].global_x, o_.site[1] > items[k_][1].global_y

            def by_diag(k_, d_):
                return sum(v[k_][i] for i, (kind, o_, _pc) in enumerate(opts[k_])
                           if kind == 'stub' and diag_(k_, o_) == d_)
            for d_ in sorted({diag_(k_, o_) for k_ in (kp, kn) for kind, o_, _pc in opts[k_] if kind == 'stub'}):
                mdl.Add(by_diag(kp, d_) == by_diag(kn, d_))
        slots, lit = {}, {}
        for k_ in (kp, kn):
            sl = slots[k_] = collections.defaultdict(list)     # (face, layer, exit x, exit y) -> the leg's options
            for i, (kind, o, _pc) in enumerate(opts[k_]):
                if kind == 'escape':
                    # (with the escape field, a run on RUN keyed by its layers too: the colouring lays a
                    # pair's two runs on ONE layer, so two legs' slots mate only where their layers meet -- zynq U1 with
                    # the LVDS teeth held, TX_FRAME's legs left with no layer in common, both runs uncoloured and the
                    # pair unplanned. Not on the other paths, whose plans stand as measured -- unless a crowded tooth
                    # took a layer from a run here, `narrowed`)
                    ro_ = tuple(sorted(getattr(o, 'runs_on', None) or ())) \
                        if (escape_to or narrowed) and o.layer == RUN else ()
                    sl[(o.direction, o.layer, round(o.exit_pt[0], 6), round(o.exit_pt[1], 6), ro_)].append(i)
                elif kind != 'stub':
                    mdl.Add(v[k_][i] == 0)          # (a pair leaves by its legs' escapes: a strap leaves no pair)
            for s_, idx in sl.items():
                if len(idx) == 1:
                    lit[(k_, s_)] = v[k_][idx[0]]
                else:
                    lit[(k_, s_)] = mdl.NewBoolVar(f'slot{n_p}_{len(lit)}')
                    mdl.Add(sum(v[k_][i] for i in idx) == lit[(k_, s_)])
        mate = collections.defaultdict(list)
        span = {}
        couples = [(sp, sn) for sp in slots[kp] for sn in slots[kn]
                   if sp[:2] == sn[:2] and 0.05 < math.hypot(sp[2] - sn[2], sp[3] - sn[3]) <= reach
                   and (not (sp[4] and sn[4]) or set(sp[4]) & set(sn[4]))]
        # ...with P on the side of its travel the other array's end has it (`hands`): a pair whose two ends disagree
        # crosses over between them, at a layer change (whole_solve's opposite hands) -- zynq's LVDS bus on four
        # layers, its ends planned array by array, had six such pairs, and no room for RX_D2's crossover. Where the
        # array has no exit pair of that hand, the pair keeps every one
        want = (hands or {}).get(items[kp][0])
        if want:
            right = [(sp, sn) for sp, sn in couples
                     if _pairs_hand(sp[0], sp[2:4], sn[2:4], arriving=want[1]) == want[0]]
            hand_rep['held' if right else 'free'].append(items[kp][0])
            couples = right or couples
        for sp, sn in couples:
            mate[(kp, sp)].append(sn)
            mate[(kn, sn)].append(sp)
            ax = 2 if sp[0] in ('up', 'down') else 3
            lo_, hi_ = sorted((sp[ax], sn[ax]))
            cls = sp[:2]
            row = by_cls[cls]
            done = set()
            for x in range(bisect.bisect_right(row, (lo_ + 0.02, '', -1)), len(row)):
                at_, k3, _l3 = row[x]
                if at_ >= hi_ - 0.02:
                    break
                if k3 in (kp, kn) or at_ in done:
                    continue
                done.add(at_)
                if (cls + (at_,)) not in span:
                    span[cls + (at_,)] = mdl.NewBoolVar(f'span{n_p}_{len(span)}')
                mdl.AddBoolOr([lit[(kp, sp)].Not(), lit[(kn, sn)].Not(), span[cls + (at_,)]])
        for pos, z in span.items():
            mdl.AddImplication(z, occupied(pos).Not())
        for k_, other_ in ((kp, kn), (kn, kp)):
            for s_ in slots[k_]:
                if mate[(k_, s_)]:
                    mdl.AddBoolOr([lit[(k_, s_)].Not()] + [lit[(other_, t)] for t in mate[(k_, s_)]])
                else:
                    mdl.Add(lit[(k_, s_)] == 0)
    # each BUS ITS OWN STRETCH of every face and layer the array's buses leave by: no exit of one between two of
    # another's there -- the second bus routed after the first finds its teeth walled in by the first's lanes
    # otherwise (Andy). Per face and layer, a bus's exits seen so far from either end (before[n], after[n]: an exit
    # of its at or before / at or after the n-th exit position) and no other bus's exit where both are. The runs on
    # RUN are one layer here: their layers are the solve's (the bus's via ends) or chosen after (colour_runs)
    groups_of = bus_groups(pcb, ref, bus)
    n_region = 0
    if groups_of:
        by_fc = collections.defaultdict(lambda: collections.defaultdict(list))   # (face, layer) -> bus -> [(at, lit)]
        for key in keys:
            g_ = groups_of.get(items[key][0])
            if g_ is None:
                continue
            for i, (kind, o, _pc) in enumerate(opts[key]):
                if kind == 'escape':
                    at_ = round(o.exit_pt[0 if o.direction in ('up', 'down') else 1], 3)
                    by_fc[(o.direction, o.layer)][g_].append((at_, v[key][i]))
        for fc in sorted(by_fc):
            per = by_fc[fc]
            if len(per) < 2:
                continue
            pos = sorted({at_ for lst in per.values() for at_, _l in lst})
            ix = {at_: n for n, at_ in enumerate(pos)}
            before, after = {}, {}
            for g_ in sorted(per):
                at_n = collections.defaultdict(list)
                for at_, l_ in per[g_]:
                    at_n[ix[at_]].append(l_)
                before[g_] = [mdl.NewBoolVar(f'rb{n_region}_{n}') for n in range(len(pos))]
                after[g_] = [mdl.NewBoolVar(f'ra{n_region}_{n}') for n in range(len(pos))]
                n_region += 1
                for n in range(len(pos)):
                    if n:
                        mdl.AddImplication(before[g_][n - 1], before[g_][n])
                        mdl.AddImplication(after[g_][n], after[g_][n - 1])
                    for l_ in at_n[n]:
                        mdl.AddImplication(l_, before[g_][n])
                        mdl.AddImplication(l_, after[g_][n])
            for h_ in sorted(per):
                for at_, l_ in per[h_]:
                    n = ix[at_]
                    if 0 < n < len(pos) - 1:
                        for g_ in sorted(per):
                            if g_ != h_:
                                mdl.AddBoolOr([l_.Not(), before[g_][n - 1].Not(), after[g_][n + 1].Not()])
    # a strap joins a ball that is itself served -- escaped, or strapped on -- and a chain of straps is a path, not a
    # loop, so every chain ends at a ball that escapes: a strap climbs one level toward it
    lvl = {key: mdl.NewIntVar(0, len(items), f'l{k}') for k, key in enumerate(keys) if straps.get(key)}
    for key in keys:
        for i, (kind, o, _pc) in enumerate(opts[key]):
            if kind != 'strap':
                continue
            c = o.to
            if not v[c]:
                mdl.Add(v[key][i] == 0)          # a neighbour with no move of its own cannot carry anyone
                continue
            mdl.Add(sum(v[c]) >= 1).OnlyEnforceIf(v[key][i])
            if c in lvl:
                mdl.Add(lvl[key] >= lvl[c] + 1).OnlyEnforceIf(v[key][i])
    # a gap site's via is ONE via however many balls of its net drop there (_hit), so it is paid once: a literal per
    # net and site, which every drop there implies
    reps = collections.defaultdict(list)          # (net, a 10 um cell) -> the sites first seen there

    def site_key(nm, q):
        # (a site as the first one seen within SHARE_TOL of it: one gap computed from each ball round it)
        c = (round(q[0] / 0.01), round(q[1] / 0.01))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for r_ in reps[(nm, c[0] + dx, c[1] + dy)]:
                    if math.hypot(r_[0] - q[0], r_[1] - q[1]) < SHARE_TOL:
                        return (nm,) + r_
        reps[(nm,) + c].append(tuple(q))
        return (nm,) + tuple(q)
    at_site = collections.defaultdict(list)
    for key in keys:
        for i, (kind, o, _pc) in enumerate(opts[key]):
            if kind == 'drop' and not o.inpad:
                at_site[site_key(items[key][0], o.site)].append(v[key][i])
    site_used = []
    for n_s, sk in enumerate(sorted(at_site)):
        lits = at_site[sk]
        if len(lits) == 1:
            site_used.append(lits[0])
            continue
        u = mdl.NewBoolVar(f'site{n_s}')
        for lit in lits:
            mdl.AddImplication(lit, u)
        site_used.append(u)
    wt = {key: W_BUS if items[key][0] in bus_s else (W_DROP if items[key][0] in drop_s else W_OTHER) for key in keys}
    rank = {key: 2 if items[key][0] in bus_s else (0 if items[key][0] in drop_s else 1) for key in keys}
    # PHASE 1, the balls served: EVERY ball is asked to be (an assumption each), with no cost in the question -- one
    # objective of count and cost together left balls unserved that a plan could serve (zynq U1, the bus fixed: 249 of
    # 251 in 200 s at 100 batches, where every ball the board allows, 250, is found in 6 s this way). A proved conflict
    # names the balls it holds (CP-SAT's core); the least of them is let go -- a plane ball before another net's, an
    # other net's before the bus's: a plane net keeps its other balls, a signal has no other way out -- and the rest
    # asked again. One worker, stopped by deterministic time: the same model gives the same answer.
    # The question is asked in TIERS, each held before the next: the bus's balls and every pair's legs, then the other
    # nets' balls, then the plane balls. Every ball at once, with the pairs held together and the plane drops in, ran
    # out of its time undecided (zynq U1 for the AD9364's LVDS bus, 297 balls: UNKNOWN at 33 s and at 128 s), and the
    # count-and-cost objective after it left the RX_D5 pair and 30 more balls unplanned. A conflict a tier's question
    # proves lets go the least ball of that tier in it; a tier out of time ends the tiers -- the rest is phase 2's.
    # (the search starts from the preferred teeth: each ball with a preferred tooth hinted to its option nearest it)
    if prefer:
        for key in keys:
            g_ = prefer.get(items[key][0]) if items[key][0] in bus_s else None
            if not g_ or not v[key]:
                continue
            esc_ = [(i, o) for i, (kind, o, _pc) in enumerate(opts[key]) if kind == 'escape']
            if esc_:
                t_ = g_['tooth']
                ibest = min(esc_, key=lambda io: (
                    (io[1].direction != g_.get('direction')) + _off_layer(io[1], g_.get('layer'))
                    + (io[1].kind != g_.get('kind')),
                    math.hypot(io[1].exit_pt[0] - t_[0], io[1].exit_pt[1] - t_[1])))[0]
                for i, var in enumerate(v[key]):
                    mdl.AddHint(var, 1 if i == ibest else 0)
    # (the menus' and the conflicts' garbage collected before the solves: their cycles otherwise wait for the
    # collector and stand under the solver's peak)
    gc.collect()
    lit = dict(served)
    go_rank = lambda k_: (rank[k_], k_)   # noqa: E731
    idx = {z_.Index(): k for k, z_ in lit.items()}
    # ONE MODEL, ONE ANSWER: both phases' answers kept per model in the process (_SOLVED) -- its bytes and everything
    # phase 2 prices, hashed. Both phases are deterministic (one worker by work; interleaved batches), so a model met
    # again has the answer solving it again would give
    cvals = {key: [cost(key, kind, o) for kind, o, _pc in opts[key]] for key in keys}
    # (phase 2's start: each ball's option the array's last plan took, where this menu has it -- read here, on the thread
    # that asks, so the plans of the array follow from one another in the program's order)
    last_ = _LAST_PLAN.get(ref) or {}
    start = {}
    for key in keys:
        g_ = last_.get(key)
        if g_ is not None:
            i0 = next((i for i, (kind, o_, pc) in enumerate(opts[key]) if _opt_sig(kind, o_, pc) == g_), None)
            if i0 is not None:
                start[key] = i0
    skey = _solved_key(mdl, ref, keys, served, items, bus_s, oth_s, drop_s, bm, rank, wt, cvals, v, site_used, rls,
                       batches, workers)
    legs_ = {k for _b, kp, kn in bm.pairs for k in (kp, kn)}
    tiers = [('bus and pairs', [k for k in keys if k in served and (items[k][0] in bus_s or k in legs_)]),
             ('others', [k for k in keys if k in served and k not in legs_ and items[k][0] in oth_s]),
             ('plane', [k for k in keys if k in served and items[k][0] in drop_s])]
    got_ = _solved(skey)

    def solve_phases():
        """both phases on the model -- CP-SAT alone, nothing printed -- their answer kept by its key (_SOLVED)"""
        t_p1 = time.time()
        let_go = []
        s1 = cp_seed.solver()
        s1.parameters.num_workers = 1
        s1.parameters.max_deterministic_time = P1_DET
        # (no LP: the question is a yes or no over booleans -- each ball one option, conflicting options never together --
        # and the relaxation only slowed the search to it. zynq7020_som's U2, 483 balls and 187,032 constraints, its three
        # tiers proved in 17 s where they took 234 with the LP; the same balls served)
        s1.parameters.linearization_level = 0
        held_keys, hint_vals, st1, tier_rep, left_out = [], None, None, [], set()
        # THE START ANSWERS PHASE 1: where the array's last plan names an option for every ball served (`start`), its
        # options replace phase 1's answer whole in phase 2's start below -- all phase 1 then decides is which balls are
        # held served, and where the start serves every one of them (laid out whole, every ball served, as phase 2's
        # start is laid out), it answers each tier's question yes: every tier held, none let go. The tiers' solves are
        # then not asked (12-20 s a plan); they would answer the same
        if start and served and all(k in start for k in served):
            m2 = mdl.Clone()
            for _tn, tier in tiers:
                for k in tier:
                    m2.Add(m2.GetBoolVarFromProtoIndex(lit[k].Index()) == 1)
            m2.ClearHints()
            for key in keys:
                for i, var in enumerate(v[key]):
                    m2.AddHint(m2.GetBoolVarFromProtoIndex(var.Index()), int(i == start[key]))
            sq = cp_model.CpSolver()
            sq.parameters.num_workers = 1
            sq.parameters.fix_variables_to_their_hinted_value = True
            sq.parameters.stop_after_first_solution = True
            sq.parameters.max_deterministic_time = START_WORK
            if sq.Solve(m2) in (cp_model.OPTIMAL, cp_model.FEASIBLE):
                for tname, tier in tiers:
                    if tier:
                        tier_rep.append((tname, len(tier), 'OPTIMAL'))
                        held_keys += list(tier)
                st1 = cp_model.OPTIMAL
                hint_vals = {var.Index(): int(i == start[key]) for key in keys for i, var in enumerate(v[key])}
            del m2
        for tname, tier in (tiers if st1 is None else ()):
            if not tier:
                continue
            while True:
                mdl.ClearAssumptions()
                mdl.AddAssumptions([lit[k] for k in held_keys + tier if k not in let_go])
                st1 = s1.Solve(mdl)
                if st1 != cp_model.INFEASIBLE:
                    break
                core = sorted((idx[c] for c in s1.SufficientAssumptionsForInfeasibility() if c in idx), key=str)
                cand = [k for k in core if k in tier] or core
                if not cand:
                    break
                let_go.append(min(cand, key=go_rank))
            st_name = s1.StatusName(st1)
            if st1 == cp_model.UNKNOWN:
                # out of time on the whole tier, undecided: the MOST of it served instead, the earlier tiers held, and
                # those held. zynq U1's 98 plane balls on four layers: every one at once undecided at 240 and at 960,
                # and phase 2's count and cost together left 7 of them unserved; given four times its budget, 2
                mdl.ClearAssumptions()
                mdl.AddAssumptions([lit[k] for k in held_keys])
                mdl.Maximize(sum(lit[k] for k in tier if k not in let_go))
                st1 = s1.Solve(mdl)
                mdl.ClearObjective()
                out_ = [k for k in tier if k not in let_go and not s1.Value(lit[k])] \
                    if st1 in (cp_model.OPTIMAL, cp_model.FEASIBLE) else []
                left_out.update(out_)
                st_name += f', the most of it {s1.StatusName(st1)}' + (f' ({len(tier) - len(out_)})' if out_ else '')
            tier_rep.append((tname, len(tier), st_name))
            if st1 not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
                break
            held_keys += [k for k in tier if k not in let_go and k not in left_out]
            hint_vals = {var.Index(): s1.Value(var) for key in keys for var in v[key]}
        mdl.ClearAssumptions()
        held = bool(held_keys)
        if held:
            mdl.ClearHints()            # (phase 1's answer replaces the preferred teeth as the start)
            for key in keys:
                for var in v[key]:
                    mdl.AddHint(var, hint_vals[var.Index()])
            for key in held_keys:
                mdl.Add(lit[key] == 1)
            st1 = cp_model.FEASIBLE if st1 not in (cp_model.OPTIMAL, cp_model.FEASIBLE) else st1
        held_set = set(held_keys)
        t_p1 = time.time() - t_p1
        # PHASE 2, the cost, every ball phase 1 held kept served; a ball it did not hold -- let go, or of a tier it ran out
        # of time on -- earns its weight if it can be served after all (count and cost together for those).
        obj = []
        for key in keys:
            w = 0 if key in held_set else wt[key]
            for i, (kind, o, _pc) in enumerate(opts[key]):
                obj.append((w - cvals[key][i]) * v[key][i])
        obj += [-C_VIA * u for u in site_used]
        mdl.Maximize(sum(obj))
        # STARTED from the array's last plan (`start`), where phase 1 held any ball: its options in place of phase 1's
        # answer, laid out whole -- every hinted value held, the rest completed by one worker -- and then EVERY
        # variable's hint, so the search has that plan from its first batch (an incomplete hint is not a plan: it
        # started from the first feasibility jump's). Where a few of its options no longer stand together (h616's U2:
        # the held berths' plan and the destination's escape, 3 of 183 balls), the most of it that does: a first plan
        # keeping as many of them as it can (one worker, no presolve: 2 s), laid out whole in its turn. Where neither is
        # a plan of this model, phase 1's answer as before
        def lay_out(val):
            mdl.ClearHints()
            for key in keys:
                for var in v[key]:
                    mdl.AddHint(var, val(var))
            sw = cp_seed.solver()
            sw.parameters.num_workers = 1
            sw.parameters.fix_variables_to_their_hinted_value = True
            sw.parameters.stop_after_first_solution = True
            sw.parameters.max_deterministic_time = START_WORK
            return sw if sw.Solve(mdl) in (cp_model.OPTIMAL, cp_model.FEASIBLE) else None
        sw = None
        if held and start:
            hv = dict(hint_vals)
            for key, i0 in start.items():
                for i, var in enumerate(v[key]):
                    hv[var.Index()] = int(i == i0)
            sw = lay_out(lambda var: hv[var.Index()])
            if sw is None:
                mdl.ClearHints()
                for key, i0 in start.items():
                    mdl.AddHint(v[key][i0], 1)
                mdl.ClearObjective()
                mdl.Maximize(sum(v[key][i0] for key, i0 in start.items()))
                sa = cp_seed.solver()
                sa.parameters.num_workers = 1
                sa.parameters.cp_model_presolve = False
                sa.parameters.stop_after_first_solution = True
                sa.parameters.max_deterministic_time = START_WORK
                st_a = sa.Solve(mdl)
                mdl.ClearObjective()
                mdl.Maximize(sum(obj))
                if st_a in (cp_model.OPTIMAL, cp_model.FEASIBLE):
                    sw = lay_out(sa.Value)
            mdl.ClearHints()
            if sw is not None:
                for i_ in range(len(mdl.Proto().variables)):
                    var = mdl.GetIntVarFromProtoIndex(i_)
                    mdl.AddHint(var, sw.Value(var))
            else:
                for key in keys:
                    for var in v[key]:
                        mdl.AddHint(var, hint_vals[var.Index()])
        started = sw is not None
        s_ = cp_seed.solver()
        # REPRODUCIBLE, as the whole solve is (whole_solve): stopped by a count of interleaved batches, the workers
        # sharing no clauses -- the same model gives the same answer (a wall-clock limit on eight workers gave two answers
        # in two runs of zynq U1's array).
        s_.parameters.num_workers = workers or SOLVE_WORKERS
        s_.parameters.interleave_search = True
        s_.parameters.max_num_deterministic_batches = batches or (SOLVE_BATCHES_STARTED if started else SOLVE_BATCHES)
        s_.parameters.share_glue_clauses = False
        s_.parameters.share_binary_clauses = False
        s_.parameters.subsolvers.extend(SUBSOLVERS_RUNS if rls else SUBSOLVERS)
        st = s_.Solve(mdl)
        vals_ = {var.Index(): s_.Value(var) for key in keys for var in v[key]} \
            if st in (cp_model.OPTIMAL, cp_model.FEASIBLE) else {}
        if vals_:
            vals_.update({y.Index(): s_.Value(y) for d in ylay.values() for y in d.values()})
        ans = dict(held=list(held_keys), hint=hint_vals, st1=st1, tiers=list(tier_rep),
                   left_out=sorted(left_out, key=repr), let_go=list(let_go), st=st, vals=vals_, t_p1=t_p1,
                   bound=s_.BestObjectiveBound(), wall=s_.WallTime(), branches=s_.NumBranches(),
                   conflicts_cp=s_.NumConflicts(), started=len(start) if started else 0)
        _keep_solved(skey, ans)
        return ans

    def _finish(ans, kept):
        """the plan from both phases' answer `ans` (`kept`: a model met before, its answer kept)"""
        let_go, st1, tier_rep, t_p1, st, vals_ = (ans['let_go'], ans['st1'], ans['tiers'], ans['t_p1'], ans['st'],
                                                  ans['vals'])
        chosen, ci = {}, {}              # ball -> (kind, option); ball -> its index
        if st in (cp_model.OPTIMAL, cp_model.FEASIBLE):
            for key in keys:
                for i, var in enumerate(v[key]):
                    if vals_[var.Index()]:
                        chosen[key] = opts[key][i][:2]
                        ci[key] = i
            # (the array's last plan, for its next: each ball's option as the solve took it, over the balls an earlier
            # plan of the array named and this one did not -- and the record kept in the report, `_last_sig`, for a
            # caller choosing between plans to put the KEPT one's back)
            last_sig_ = {key: _opt_sig(*opts[key][i]) for key, i in ci.items()}
            _LAST_PLAN.setdefault(ref, {}).update(last_sig_)
        else:
            last_sig_ = {}
        piece = {key: opts[key][ci[key]][2] for key in chosen}
        uncoloured = []
        again = None
        saved_last_ = None
        if rls:
            # each chosen run on RUN its layer: the plan let K runs share a lane, an exit cluster, a crossing; those of
            # one such group must stand on as many layers (colour_runs), a pair's two legs on one, a bus lane's on the
            # layer its preferred tooth leaves by where it can
            runs_ = {key: o.runs_on for key, (kind, o) in chosen.items() if kind == 'escape' and o.layer == RUN}
            sel = {(key, ci[key]) for key in runs_}
            edges = set()
            for g in list(cap_g) + [tuple(set(a) | set(b)) for a, b in cap_b]:
                mem = sorted(x[0] for x in g if x in sel)
                for x in range(len(mem)):
                    for y in range(x + 1, len(mem)):
                        edges.add((mem[x], mem[y]))
            together = [(kp, kn) for _b, kp, kn in bm.pairs if kp in runs_ and kn in runs_]
            want = {key: ((prefer or {}).get(items[key][0]) or {}).get('layer') for key in runs_
                    if items[key][0] in bus_s}
            import route_layers
            colour, uncoloured = colour_runs(runs_, sorted(edges), together, want,
                                             route_layers.plane_kinds(pcb, list(pcb.board_info.copper_layers or ())))
            if uncoloured and ylay:
                # (`exact_layers`: the plan chose every run's layer, so a colouring of all of them stands -- its own,
                # taken where the colouring by wishes, stopped by its time, left a run out)
                own_ = {key: L for key in runs_ for L, y in ylay.get((key, ci[key]), {}).items() if vals_.get(y.Index())}
                if all(k_ in own_ for k_ in uncoloured):
                    colour = {**colour, **own_}
                    uncoloured = sorted(k_ for k_ in runs_ if k_ not in colour)
            if uncoloured and not exact_layers and LAYERS_EXACT and any(items[k_][0] in bus_s for k_ in uncoloured):
                # A BUS BALL'S RUN NO COLOURING HOLDS: the plan asked again with every run's layer its own choice
                # (`exact_layers`), so the solve itself finds the runs that stand together on the layers there are --
                # and the better of the two kept (below). (Asked again with those runs' options out of the menus
                # instead, the next plan's runs met the same wall elsewhere: a shared array's field of four buses left
                # a bus ball with no escape after three such plans, where every ball of the buses has one)
                log(f'  whole-array escape of {ref}: {len(uncoloured)} run(s) no colouring held {sorted(uncoloured)} -- '
                    f'planned again, each run\'s layer the plan\'s own')
                saved_last_ = dict(_LAST_PLAN.get(ref) or {})   # (this plan's, put back if it is the one kept)
                again = plan_array(**{**args_, 'exact_layers': True})
            elif uncoloured and (colour_tries is None or len(uncoloured) < colour_tries) and \
                    any(items[k_][0] in bus_s for k_ in uncoloured):
                # A BUS BALL'S RUN NO COLOURING HOLDS: the plan asked again with those runs' options out of the menus,
                # while it colours more. The solve's capacity rows let the runs through and the colouring then had no
                # layer for them -- a shared array left an LVDS pair and the ball behind it with none, and the repair,
                # the engine and the bus's own comb after it laid that ball between the pair's legs; asked again, the
                # neighbours' choices make the room. The better of the two kept, once this one's uncoloured balls have
                # been given their options too (below)
                ban2 = frozenset(ban) | {(k_, _opt_sig(chosen[k_][0], chosen[k_][1], piece[k_])) for k_ in uncoloured}
                log(f'  whole-array escape of {ref}: {len(uncoloured)} run(s) no colouring held {sorted(uncoloured)} -- '
                    f'planned again without them')
                saved_last_ = dict(_LAST_PLAN.get(ref) or {})   # (this plan's, put back if it is the one kept)
                again = plan_array(**{**args_, 'ban': ban2, 'colour_tries': len(uncoloured)})
            for key, L in colour.items():
                kind, o = chosen[key]
                o2 = dataclasses.replace(o, layer=L, legs=[(a, b, L if l_ == RUN else l_) for a, b, l_ in o.legs])
                chosen[key] = (kind, o2)
                pc = piece[key]
                piece[key] = Piece(pc.net, list(o2.legs), pc.vias, pc.balls)
            for key in uncoloured:
                del chosen[key], ci[key], piece[key]
        repaired = []
        if uncoloured and escape_to:
            # (escape_to) a ball whose run the colouring could not place takes its cheapest option that STANDS with
            # the coloured plan in the laid geometry (_hit): a run on one of its layers that is free there, an escape off
            # RUN, a drop, its dog-bone alone -- a pair both its legs' dog-bones; never a strap, which leans on a neighbour's
            # choice. The capacity rows count a lane group's runs per layer and cannot see a run pushed from group to group
            # (zynq U1, the LVDS teeth held: OTG_DATA6 chosen, no layer left for it, the ball left bare)
            # ...and none of the model's conflict PAIRS with a chosen option: those are more than the copper's hits (a via in
            # another bus's way out, exit_ray_conflicts), and a ball given its dog-bone here put the via 0.40 mm
            # from the legs of two FFC pairs leaving the face, which the solve had kept clear
            want_ = set(uncoloured)
            partners = collections.defaultdict(list)
            for k_, i_, k2_, j_ in pairs:
                if k_ in want_:
                    partners[(k_, i_)].append((k2_, j_))
                if k2_ in want_:
                    partners[(k2_, j_)].append((k_, i_))

            def paired(k, i):
                return any(ci.get(k2) == j for k2, j in partners.get((k, i), ()))

            def stands(k, pc):
                bx = _box(pc, sz['grow'])
                for k2, pc2 in piece.items():
                    b2 = _box(pc2, sz['grow'])
                    if bx[0] > b2[2] or b2[0] > bx[2] or bx[1] > b2[3] or b2[1] > bx[3]:
                        continue
                    if _hit(pc, pc2, items[k][0] == items[k2][0], sz):
                        return False
                return True

            def ways(k):
                for i, (kind, o, pc) in sorted(enumerate(opts[k]), key=lambda t: (cost(k, t[1][0], t[1][1]), t[0])):
                    if kind == 'strap':
                        continue
                    if kind == 'escape' and o.layer == RUN:
                        for L in (getattr(o, 'runs_on', None) or ()):
                            o2 = dataclasses.replace(o, layer=L, legs=[(a, b, L if l_ == RUN else l_) for a, b, l_ in o.legs])
                            yield i, kind, o2, Piece(pc.net, list(o2.legs), pc.vias, pc.balls)
                    else:
                        yield i, kind, o, pc
            leg_of = {k_: (kp, kn) for _b, kp, kn in bm.pairs for k_ in (kp, kn)}
            # (the plan's PAIR rule kept: no exit of the option between two chosen legs' exits of a pair on that face
            # and layer -- a run on RUN counts on every layer it may take. Without it a shared array's plan gave an
            # uncoloured ball of one bus a tooth between an LVDS pair's legs, and that bus's fanout audit refused every
            # round's board for the split pair, the whole bus left to its last resort)
            ex_along = lambda m: m.exit_pt[0 if m.direction in ('up', 'down') else 1]       # noqa: E731
            ex_layers = lambda m: set(getattr(m, 'runs_on', None) or ()) if m.layer == RUN else {m.layer}  # noqa: E731

            def splits(kind, o):
                if kind != 'escape':
                    return False
                for _b, kp, kn in bm.pairs:
                    cp, cn = chosen.get(kp), chosen.get(kn)
                    if not cp or not cn or cp[0] != 'escape' or cn[0] != 'escape':
                        continue
                    op, on = cp[1], cn[1]
                    if not (op.direction == on.direction == o.direction) or \
                            not (ex_layers(o) & ex_layers(op) & ex_layers(on)):
                        continue
                    lo_, hi_ = sorted((ex_along(op), ex_along(on)))
                    if lo_ + 0.02 < ex_along(o) < hi_ - 0.02:
                        return True
                return False
            # (a PAIR's legs first, then the singles: a pair needs two exits that mate, a single any one -- in name
            # order, a single's surface exit took the gap an LVDS pair's two outer balls leave by, and the pair was
            # left with no escape)
            reach_ = 1.3 * max(bm.grid.pitch_x, bm.grid.pitch_y)

            def mated_escapes(kp, kn):
                for ip, kd, op, pp in ways(kp):
                    if kd != 'escape' or not stands(kp, pp) or paired(kp, ip):
                        continue
                    for jn, kd2, on, pn in ways(kn):
                        if kd2 != 'escape' or on.direction != op.direction or on.layer != op.layer or \
                                abs(ex_along(on) - ex_along(op)) > reach_ or not stands(kn, pn) or paired(kn, jn) or \
                                _hit(pp, pn, False, sz):
                            continue
                        lo_, hi_ = sorted((ex_along(op), ex_along(on)))
                        if any(c_[0] == 'escape' and c_[1].direction == op.direction and
                               op.layer in ex_layers(c_[1]) and lo_ + 0.02 < ex_along(c_[1]) < hi_ - 0.02
                               for c_ in chosen.values()):
                            continue
                        return [(kp, ip, kd, op, pp), (kn, jn, kd2, on, pn)]
                return None
            for key in sorted(uncoloured, key=lambda k_: (k_ not in leg_of, k_)):
                if key in chosen:
                    continue
                if key in leg_of:
                    kp, kn = leg_of[key]
                    if kp in chosen or kn in chosen:
                        continue
                    for ip, (kd, op, pp) in enumerate(opts[kp]):
                        if kd != 'stub' or not stands(kp, pp) or paired(kp, ip):
                            continue
                        hit = next(((jn, on, pn) for jn, (kd2, on, pn) in enumerate(opts[kn])
                                    if kd2 == 'stub' and stands(kn, pn) and not paired(kn, jn)
                                    and not _hit(pp, pn, False, sz)), None)
                        if hit:
                            for k_, i_, o_, p_ in ((kp, ip, op, pp), (kn,) + hit):
                                chosen[k_], ci[k_], piece[k_] = ('stub', o_), i_, p_
                                repaired.append(k_)
                            break
                    else:
                        # (no two stubs: two ESCAPES that mate, as the plan's pair rule has them -- one face and layer,
                        # the exits neighbours within its reach, no chosen exit between. An LVDS pair on the outer row,
                        # every option of its legs a dog-bone on a run layer still to colour, had no stub and was left
                        # with no escape, and a single's exit took its gap)
                        got = mated_escapes(kp, kn)
                        if got:
                            for k_, i_, kind_, o_, p_ in got:
                                chosen[k_], ci[k_], piece[k_] = (kind_, o_), i_, p_
                                repaired.append(k_)
                    continue
                for i, kind, o, pc in ways(key):
                    if stands(key, pc) and not paired(key, i) and not splits(kind, o):
                        chosen[key], ci[key], piece[key] = (kind, o), i, pc
                        repaired.append(key)
                        break
            if repaired:
                log(f'  whole-array escape of {ref}: {len(repaired)} uncoloured ball(s) given an option that stands '
                    f'with the plan: {sorted(repaired)}')
        # the choice against the laid geometry, apart from the lane model the pairs came from: two chosen options of two
        # balls whose copper the engine could not both lay (the lane model does not see, e.g., two plain moves crossing)
        clashes = []
        ck = sorted(chosen)
        for x_, a in enumerate(ck):
            ba = boxes[(a, ci[a])]
            for b in ck[x_ + 1:]:
                bb = boxes[(b, ci[b])]
                if ba[0] > bb[2] or bb[0] > ba[2] or ba[1] > bb[3] or bb[1] > ba[3]:
                    continue
                if _hit(piece[a], piece[b], items[a][0] == items[b][0], sz):
                    clashes.append((a, b))
        gap_sites = {site_key(items[k][0], o.site) for k, (kind, o) in chosen.items() if kind == 'drop' and not o.inpad}
        obj_orig = sum((W_BUS if items[k][0] in bus_s else (W_DROP if items[k][0] in drop_s else W_OTHER))
                       - cost(k, kind, o) for k, (kind, o) in chosen.items()) - C_VIA * len(gap_sites)
        hints = {}
        pair_of = {k_: b_ for b_, kp, kn in bm.pairs for k_ in (kp, kn)}
        for key, (kind, o) in chosen.items():
            at = (round(balls[key][0], 3), round(balls[key][1], 3))
            if kind == 'escape':
                h = sr.full_move(o)
                # the move's own legs, always (an empty list where it has none): the plan proved ITS legs clear of each
                # other, and an exit alone is laid by a search that reaches it by any cells
                h['legs'] = [(tuple(a), tuple(b), L) for (a, b, L) in o.legs]
            elif kind == 'strap':
                h = {'kind': 'strap', 'to': tuple(o.b), 'layer': o.layer}
            elif kind == 'stub':
                # (laid by lay, before the engine -- at the via the plan cleared it at, the array's own rung: laid at
                # the run's via instead, a stepped-down array's stub via stood larger than its plan)
                h = {'kind': 'stub', 'site': tuple(o.site), 'layer': o.layer,
                     'via': (round(2 * sz['vr'], 4), round(2 * sz['vdr'], 4))}
            else:
                h = {'kind': 'drop', 'site': tuple(o.site), 'layer': o.layer, 'inpad': bool(o.inpad)}
                if not o.inpad and o.r < sz['vr'] - 1e-9:
                    h['via'] = (round(2 * o.r, 4), round(2 * o.dr, 4))     # (a finer rung's: the engine lays it so)
            h['strict'] = True
            if key in pair_of and kind == 'escape':
                h['pair'] = pair_of[key]           # (lay keeps the pair's gate)
            hints[at] = h
        kinds = collections.Counter((('bus' if items[k][0] in bus_s else 'plane' if items[k][0] in drop_s
                                      else 'other'), kind) for k, (kind, _o) in chosen.items())
        rep = dict(status=_status_name(st), balls=len(items), moves=n_moves, conflicts=len(pairs), conflict_groups=len(groups), conflict_bicliques=len(bigroups),
                   bus_escaped=kinds[('bus', 'escape')],
                   bus_balls=sum(1 for k in items if items[k][0] in bus_s),
                   others_escaped=kinds[('other', 'escape')], others_strapped=kinds[('other', 'strap')],
                   others_stubbed=kinds[('other', 'stub')],
                   others_balls=sum(1 for k in items if items[k][0] in oth_s),
                   dropped=kinds[('plane', 'drop')], plane_strapped=kinds[('plane', 'strap')],
                   dropped_in_pad=sum(1 for k, (kind, o) in chosen.items() if kind == 'drop' and o.inpad),
                   drop_vias=len(gap_sites) + sum(1 for k, (kind, o) in chosen.items() if kind == 'drop' and o.inpad),
                   plane_balls=sum(1 for k in items if items[k][0] in drop_s),
                   no_move=sorted(k for k in items if not opts[k]),
                   unplanned=sorted(k for k in items if k not in chosen), clashes=clashes,
                   climbed=sum(1 for k, (kind, o) in chosen.items() if kind == 'escape' and getattr(o, 'climb', 0)),
                   streets=sum(1 for k, (kind, o) in chosen.items() if kind == 'escape' and getattr(o, 'street', 0)),
                   faces=dict(collections.Counter(f'{"bus" if items[k][0] in bus_s else "other"} {o.direction} '
                                                  f'{o.layer} {o.kind}' for k, (kind, o) in chosen.items()
                                                  if kind == 'escape')),
                   # (no tier with a ball to serve -- every ball planned again without an option, zynq U2's four plane
                   # balls held round the cap step's passives -- asks phase 1 nothing, and its solver has no status)
                   let_go=list(let_go),
                   phase1=_status_name(st1) if st1 is not None else 'nothing to ask',
                   phase1_secs=round(t_p1, 1), phase1_tiers=tier_rep,
                   uncoloured=list(uncoloured),
                   run_layers=dict(collections.Counter(o.layer for k, (kind, o) in chosen.items()
                                                       if kind == 'escape' and o.kind != 'surface')) if rls else None,
                   pairs=len(bm.pairs), pairs_escaped=sum(1 for _b, kp, kn in bm.pairs if kp in chosen and kn in chosen),
                   hands_held=sorted(hand_rep['held']), hands_free=sorted(hand_rep['free']),
                   secs=round(time.time() - t0, 1), _last_sig=last_sig_)
        if debug:
            # the model's pieces, for checking the answer against them (the conflict pairs, the options) and the solve's
            # own account of itself (objective, proved bound, wall time, branches)
            rep['debug'] = dict(items=items, opts=opts, pairs=pairs, groups=groups, bigroups=bigroups, chosen=chosen, cost=cost, sizes=sz,
                                objective=obj_orig if chosen else None, bound=ans['bound'],
                                wall=ans['wall'], branches=ans['branches'], conflicts_cp=ans['conflicts_cp'])
        log(f'  whole-array escape of {ref}: {rep["status"]} -- bus {rep["bus_escaped"]}/{rep["bus_balls"]}, others '
            f'{rep["others_escaped"]} escaped + {rep["others_strapped"]} strapped'
            + (f' + {rep["others_stubbed"]} dog-boned alone' if rep['others_stubbed'] else '')
            + f' of {rep["others_balls"]}, plane '
            f'{rep["dropped"]} dropped ({rep["dropped_in_pad"]} in pad)'
            + (f' + {rep["plane_strapped"]} strapped' if rep['plane_strapped'] else '') + f' of {rep["plane_balls"]}'
            + (f' on {rep["drop_vias"]} vias' if rep['drop_vias'] != rep['dropped'] else '') + '; '
            f'{rep["climbed"]} climbs, {rep["streets"]} street vias chosen; {len(rep["no_move"])} balls with no option; '
            f'{len(clashes)} chosen pairs clash in the laid geometry; '
            + (f'runs on {rep["run_layers"]}' + (f', {len(uncoloured)} no layer held {uncoloured}' if uncoloured else '')
               + '; ' if rls else '')
            + f'served first {rep["phase1"]} in '
            f'{rep["phase1_secs"]} s' + (f', let go (a proved conflict) {rep["let_go"]}' if rep['let_go'] else '')
            + (' (the model planned before: its answer kept)' if kept else '')
            + (f'; the cost from the array\'s last plan ({ans["started"]} balls)' if ans.get('started') else '')
            # (each tier's own status where one is not OPTIMAL: a tier out of its time is not held, and its balls are phase
            # 2's -- the line's FEASIBLE said only that an earlier tier held, and hid the plane tier that left six drops)
            + (f' (tiers {", ".join(f"{t} {n} {s}" for t, n, s in tier_rep)})'
               if any(s != 'OPTIMAL' for _t, _n, s in tier_rep) else '')
            + (f'; pairs held to the other end\'s hand {len(rep["hands_held"])}'
               + (f', NO exit pair of it {rep["hands_free"]}' if rep['hands_free'] else '') if hands else '')
            + f'; {rep["secs"]} s')
        if again is not None:
            # (the plan asked again against this one by what each leaves without an option once its uncoloured balls
            # have theirs -- a bus ball first, then any ball, then the runs no colouring held. Judged by its uncoloured
            # runs alone, a shared array's plan colouring two fewer runs was kept and left a bus ball with no escape,
            # over the plan asked after it, whose four uncoloured balls each took an option that stood)
            def left_(r_):
                un_ = r_.get('unplanned') or ()
                return (sum(1 for k_ in un_ if k_ in items and items[k_][0] in bus_s), len(un_),
                        len(r_.get('uncoloured') or ()))
            a_, b_ = left_(again[1]), left_(rep)
            log(f'  whole-array escape of {ref}: planned again, {a_[2]} uncoloured, {a_[0]} bus ball(s) of {a_[1]} '
                f'left without an option (this plan {b_[2]}, {b_[0]} of {b_[1]}) -- '
                + ('the plan asked again kept' if a_ < b_ else 'the plan before stands'))
            if a_ < b_:
                return again
            if saved_last_ is not None:
                # (...and the array's next plan starts from THIS one, the one kept: the plan asked again had written
                # its own as the array's last)
                _LAST_PLAN[ref] = saved_last_
        return hints, rep

    ans = solve_phases() if got_ is None else got_
    res = _finish(ans, got_ is not None)
    # (the model collected here, not at the collector's leisure: the CP-SAT model and its variables hold each other,
    # and left standing they were under the engine's lay that follows -- the zynq chain's U1 source realize at 1.09
    # GB, its plan's own peak 0.77)
    del mdl, v, served
    gc.collect()
    return res


def lay(board, out, ref, bus, others, other_layers, hints, plane_drop='auto'):
    """The plan laid in ONE call of the under-pad engine's joint escape (py_router/bga_fanout/underpad.py,
    joint=True): the planned moves on their own legs and the straps first, each net held to its layers (the bus to
    its routing layers, the others to `other_layers`) and the bus first; then the balls the plan could not place, by the under-pad
    grid's generic phases; the plane balls dropped (`plane_drop` auto: every net the call leaves out that owns a zone
    or six balls; 'off': none). Returns (tracks, vias, failed nets)."""
    import shutil
    from kicad_parser import parse_kicad_pcb
    from kicad_writer import add_tracks_and_vias_to_pcb
    from bga_fanout import generate_bga_fanout
    import braid as te
    import fanout_from_plan as fp
    import ship_vias
    import source_realize as sr
    stubs = {at: h for at, h in hints.items() if h.get('kind') == 'stub'}
    if stubs:
        # (the plan's DOG-BONES ALONE, laid first as the plan proved them: the engine has no such move, and leaves a
        # ball that carries its own net's copper -- so none of them is fanned again by its phases, blind to the plan)
        pcb0 = parse_kicad_pcb(board)
        at_ball = {(round(p.global_x, 3), round(p.global_y, 3)): p for p in pcb0.footprints[ref].pads}
        got = [(at_ball[a], h) for a, h in sorted(stubs.items()) if a in at_ball]
        pre = (out[:-len('.kicad_pcb')] if out.endswith('.kicad_pcb') else out) + '.stubs.kicad_pcb'
        add_tracks_and_vias_to_pcb(
            board, pre,
            [{'start': (p.global_x, p.global_y), 'end': tuple(h['site']), 'width': sr.FAN_TRACK, 'layer': h['layer'],
              'net_id': p.net_id} for p, h in got],
            [{'x': h['site'][0], 'y': h['site'][1], 'size': (h.get('via') or (te.VIA_SIZE, te.VIA_DRILL))[0],
              'drill': (h.get('via') or (te.VIA_SIZE, te.VIA_DRILL))[1],
              'layers': ['F.Cu', 'B.Cu'], 'net_id': p.net_id} for p, h in got], [],
            net_id_to_name={i: n.name for i, n in pcb0.nets.items()})
        fp.copy_pro(board, pre)
        board = pre
        hints = {a: h for a, h in hints.items() if a not in stubs}
    # the movable passives (the decoupling caps under the array) are not obstacles: the cap placement step follows
    # the bus step in the chain and moves them off this copper (geometry.immovable_foreign_pads)
    pcb = parse_kicad_pcb(board)
    spec = {'net_layers': {**{n: bus_route_layers(pcb) for n in bus}, **{n: list(other_layers) for n in others}},
            'priority': list(bus)}
    tracks, vias_add, vias_rm, failed = generate_bga_fanout(
        pcb.footprints[ref], pcb, net_filter=engine_filter(list(bus) + list(others), pcb),
        layers=list(pcb.board_info.copper_layers), track_width=sr.FAN_TRACK, clearance=sr.FAN_CLEAR,
        via_size=te.VIA_SIZE, via_drill=te.VIA_DRILL, exit_margin=0.5, escape_method='jointescape',
        plane_drop=plane_drop, escape_dir_hints=hints, bus=spec,
        net_clearances={})   # the whole-array plan is laid at FAN_CLEAR, class-blind (#1529)
    tracks, vias_add, gated = keep_pair_gates(pcb, ref, hints, tracks, vias_add)
    if gated:
        print(f'  whole-array escape of {ref}: left out, an escape through a planned pair\'s gate: '
              + ', '.join(sorted(namer(pcb)(pcb.nets[i].name) for i in gated)))
    if tracks or vias_add:
        add_tracks_and_vias_to_pcb(board, out, tracks, vias_add, vias_rm,
                                   net_id_to_name={i: n.name for i, n in pcb.nets.items()})
        ship_vias.stamp(out, 'joint escape', print)
    else:
        shutil.copy(board, out)
    fp.copy_pro(board, out)
    return len(tracks), len(vias_add), sorted(set(failed))


def _cross(p, q, a, b):
    """segments p-q and a-b meet (crossing or touching)"""
    def orient(u, v, w):
        d = (v[0] - u[0]) * (w[1] - u[1]) - (v[1] - u[1]) * (w[0] - u[0])
        return 0 if abs(d) < 1e-12 else (1 if d > 0 else -1)

    def on(u, v, w):
        return min(u[0], v[0]) - 1e-9 <= w[0] <= max(u[0], v[0]) + 1e-9 and \
            min(u[1], v[1]) - 1e-9 <= w[1] <= max(u[1], v[1]) + 1e-9
    o1, o2, o3, o4 = orient(p, q, a), orient(p, q, b), orient(a, b, p), orient(a, b, q)
    if o1 != o2 and o3 != o4:
        return True
    return (o1 == 0 and on(p, q, a)) or (o2 == 0 and on(p, q, b)) or (o3 == 0 and on(a, b, p)) or \
        (o4 == 0 and on(a, b, q))


def keep_pair_gates(pcb, ref, hints, tracks, vias):
    """A planned PAIR's GATE kept (lay): the engine's generic phases lay the balls the plan left out, a ball at a time
    and blind to the pairs, and an escape of theirs through the gate between a planned pair's two exits -- the segment
    joining them, on the pair's layer -- splits the pair (zynq U1: ETH_TXD3 between RX_D5's legs, ETH_RXCK between
    FB_CLK's). Such a net -- one with no ball planned -- is left out of the lay: the plan had left it out too. Returns
    (tracks, vias, the nets left out)."""
    legs = collections.defaultdict(list)
    for h in hints.values():
        if h.get('pair') and h.get('exit') is not None:
            # (its exit, and its own legs' points: the gate is the pair's at every depth, each leg's points from the
            # exit back joined to the other's -- an escape the engine ended at the face between the two legs, at the
            # fab floor's track, split an LVDS pair and crossed no gate at the exit line, past the face)
            pts = [tuple(h['legs'][0][0])] + [tuple(lg[1]) for lg in h['legs']] if h.get('legs') else []
            legs[h['pair']].append((tuple(h['exit']), h.get('layer'), pts))
    gates = []
    for g in legs.values():
        if len(g) == 2 and g[0][1] == g[1][1]:
            gates.append((g[0][1], g[0][0], g[1][0]))
            gates += [(g[0][1], a, b) for a, b in zip(reversed(g[0][2]), reversed(g[1][2]))]
    if not gates:
        return tracks, vias, set()
    at_ball = {(round(p.global_x, 3), round(p.global_y, 3)): p.net_id for p in pcb.footprints[ref].pads}
    planned = {at_ball[at] for at in hints if at in at_ball}
    gated = set()
    for t in tracks:
        nid = t.get('net_id')
        if nid in planned or nid in gated:
            continue
        if any(t.get('layer') == L and _cross(tuple(t['start']), tuple(t['end']), a, b) for L, a, b in gates):
            gated.add(nid)
    if not gated:
        return tracks, vias, gated
    return ([t for t in tracks if t.get('net_id') not in gated], [v for v in vias if v.get('net_id') not in gated],
            gated)


def bare_balls(board, ref, nets, track_width):
    """`ref`'s balls of `nets` with no copper of their net on them (bga_fanout.ball_has_copper, the board's copper)"""
    from kicad_parser import parse_kicad_pcb
    from bga_fanout import ball_has_copper
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    sn = namer(pcb)
    want = {sn(n) for n in nets}
    vias = [{'x': v.x, 'y': v.y, 'size': v.size, 'net_id': v.net_id} for v in pcb.vias]
    tracks = [{'start': (s.start_x, s.start_y), 'end': (s.end_x, s.end_y), 'layer': s.layer, 'net_id': s.net_id}
              for s in pcb.segments]
    return [f'{sn(p.net_name)}#{p.pad_number}' for p in pcb.footprints[ref].pads
            if p.net_id and sn(p.net_name or '') in want and not ball_has_copper(p, vias, tracks, track_width)]


def fan_array(board, out, ref, bus, others, other_layers, far=None, drops=(), log=print, rungs=None, **solve):
    """The array's joint escape at ONE size for the whole fanout: planned and laid at the chain's fan track and via,
    and -- only when a ball is left bare or a plane ball undropped -- the whole of it again at the next rung of the
    fab ladder, the via and the track stepped down TOGETHER (list_nets.escalation_rungs, as the under-pad shrink
    rescue steps them; never one via or one track on its own), until a rung serves every ball, or serves no more than
    the rung above it; the rung that left the fewest bare, then the fewest undropped. OUT is that rung's board. Returns (its sizes, as rules.Rules,
    and a report per rung tried). The chain's own rules are back in place on return. `rungs` (rules.Rules): these
    sizes instead of the ladder -- a later round keeps the first round's."""
    import dataclasses
    import shutil
    import rules as _rules
    import fanout_from_plan as fp
    from kicad_parser import parse_kicad_pcb
    from list_nets import escalation_rungs
    base = _rules.active()
    with contextlib.redirect_stdout(io.StringIO()):
        ncu = len(parse_kicad_pcb(board).board_info.copper_layers or ()) or 4
    ladder = [base]
    for f in (escalation_rungs(ncu) if not rungs else ()):
        r = dataclasses.replace(base, fan_track=min(ladder[-1].fan_track, f['track_width']),
                                via_size=min(ladder[-1].via_size, f['via_diameter']),
                                via_drill=min(ladder[-1].via_drill, f['via_drill']))
        if (r.fan_track, r.via_size, r.via_drill) != (ladder[-1].fan_track, ladder[-1].via_size, ladder[-1].via_drill):
            ladder.append(r)
    if rungs:
        ladder = list(rungs)
    stem = out[:-len('.kicad_pcb')] if out.endswith('.kicad_pcb') else out
    reports, best = [], None
    try:
        for n, r in enumerate(ladder):
            _rules.install(r)
            with contextlib.redirect_stdout(io.StringIO()):
                pcb = parse_kicad_pcb(board)
            hints, rep = plan_array(pcb, ref, bus, others, other_layers, far=far, drops=drops, log=log, **solve)
            # (the plan's debug -- its every option, group and pair -- let go before the lay but its choice)
            chosen_ = (rep.pop('debug', None) or {}).get('chosen')
            gc.collect()
            laid = f'{stem}.rung{n}.kicad_pcb'
            # (the plan alone drops the plane balls: the engine's own drop pass after it, blind to the bus's lanes, laid
            # one the plan had refused -- zynq U2's A8, half a pitch off its edge between two berths' lanes)
            with contextlib.redirect_stdout(sys.stderr):
                lay(board, laid, ref, bus, list(others), other_layers, hints, plane_drop='off')
            undropped = 0           # (no caller hands the fanout plane nets to drop, `drops`: none is left undropped)
            bare = bare_balls(laid, ref, list(bus) + list(others), r.fan_track)
            reports.append(dict(track=r.fan_track, via=r.via_size, drill=r.via_drill, bare=bare, undropped=undropped,
                                status=rep['status'], planned_bus=rep['bus_escaped'],
                                planned_others=rep['others_escaped'] + rep['others_strapped'],
                                planned_drops=rep['dropped'] + rep['plane_strapped'], pairs=rep.get('pairs'),
                                pairs_escaped=rep.get('pairs_escaped'), tiers=rep.get('phase1_tiers'),
                                hands_held=rep.get('hands_held'), hands_free=rep.get('hands_free'),
                                chosen=chosen_))
            log(f'  whole-array escape of {ref} at track {r.fan_track} / via {r.via_size}/{r.via_drill}: {len(bare)} '
                f'bare ball(s){" " + str(bare) if bare else ""}, {undropped} plane ball(s) undropped')
            if best is not None and (len(bare), undropped) >= best[0]:
                # (a rung that serves no more than the one above it: what is left is no matter of size -- the
                # zynq DDR's U1 GND R20, ringed by other nets' escapes, undropped at every rung, three whole plans
                # of the array more for nothing)
                log(f'  whole-array escape of {ref}: the rung serves no more than the one above it -- the ladder stops')
                break
            best = ((len(bare), undropped), laid, r)
            if not bare and not undropped:
                break
    finally:
        _rules.install(base)
    shutil.copy(best[1], out)
    fp.copy_pro(best[1], out)
    return best[2], reports


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('board')
    ap.add_argument('ref')
    ap.add_argument('--bus', required=True, help='the bus nets, comma separated')
    ap.add_argument('--others', default='', help='the other nets fanned with it, comma separated')
    ap.add_argument('--dest', help='the other array: the bus never leaves by the face facing away from it')
    ap.add_argument('--prefer', help='a board whose bus teeth at REF the plan prefers to keep')
    ap.add_argument('--out', help='lay the plan: the fanned board')
    ap.add_argument('--batches', type=int, default=SOLVE_BATCHES, help='the solve\'s work budget (CP-SAT batches)')
    a = ap.parse_args()
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(a.board)
    bus = [n for n in a.bus.split(',') if n]
    others = [n for n in a.others.split(',') if n]
    prefer = preferred_teeth(a.prefer, a.ref, bus) if a.prefer else None
    far = far_face(pcb, a.ref, a.dest) if a.dest else None
    hints, rep = plan_array(pcb, a.ref, bus, others, signal_layers(pcb), far=far, prefer=prefer,
                            batches=a.batches)
    print({k: v for k, v in rep.items() if k not in ('unplanned', 'no_move')})
    if a.out:
        with contextlib.redirect_stdout(sys.stderr):
            n_t, n_v, failed = lay(a.board, a.out, a.ref, bus, others, signal_layers(pcb), hints)
        print(f'laid: {n_t} tracks, {n_v} vias, failed {failed}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

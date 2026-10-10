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
2. The moves that cannot both be laid are found as the whole route finds them (pages_first._conflicts, strict: a
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
import io
import math
import os
import sys
import time
import types

import awx_settings

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))

OUTER = ('F.Cu', 'B.Cu')
W_BUS = 1_000_000          # a bus ball escaped
W_OTHER = 10_000           # another ball escaped
C_VIA = 300                # a via
C_MM = 20                  # a millimetre of escape
# a bus ball's PREFERRED tooth (the ends model's ask: the whole route's plan, joint_destination, the joint source
# realize) is followed unless it cannot be -- a millimetre off it costs over three vias, another face, layer or kind
# ten; serving another ball outweighs both (W_OTHER). At 60 and 400 the plan put 18 of the zynq U1's 37 asked teeth
# elsewhere for a shorter escape, 13 of them out of order along the face the lanes leave by
C_DEV_MM = 1000            # a millimetre between a bus ball's exit and its preferred tooth's
C_DEV_KIND = 3000          # ...another face, layer or kind than its preferred tooth's
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
SOLVE_BATCHES = int(awx_settings.get('JOINT_SOLVE_BATCHES', '60'))
SOLVE_WORKERS = 2
# phase 1's deterministic time, per question asked. At 120 the zynq U1 plane tier ran out of it on three layers and
# left six drops unplanned; at 240 it proved every drop (in 60 s)
P1_DET = float(awx_settings.get('JOINT_P1_DET', '240'))
# A climb's reach, in pitches. zynq U1's whole array (100 batches): uncapped (to 19) 21172 moves, objective 48280503;
# capped at 8, 17356 and 48261318; at 4, 12582 and 48309493 -- the proved bound 48.44M in all three, and every climb
# any of the solves chose 1 to 4 deep but one
CLIMB = int(awx_settings.get('JOINT_CLIMB', '4'))
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


def short_name(n):
    return n.split('/')[-1]


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
    byname = {short_name(n.name): (i, n) for i, n in pcb.nets.items()}
    out = {}
    for n in bus:
        nm = short_name(n)
        if nm not in byname:
            continue
        pad = next((q for q in byname[nm][1].pads if q.component_ref == ref), None)
        g = sr.measure_tooth(pcb, nm, pad, byname) if pad is not None else None
        if g and g.get('tooth'):
            out[nm] = g
    return out


Strap = collections.namedtuple('Strap', 'to a b layer length')     # a join to the neighbouring ball `to`
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
    hole to hole, a gap site's via (the rung's), a ball's via-in-pad as the engine sizes it for THAT pad
    (clamp_via_to_pad on the fab ladder: zynq U1's 0.35 mm balls take no 0.45 mm via), the stacking pitch of two lanes
    at that track, and the engine's own-ball disk (underpad home_r)."""
    import braid as te
    import rules as _rules
    import source_realize as sr
    from list_nets import board_constraint, escalation_rungs
    from routing_defaults import HOLE_TO_HOLE_CLEARANCE
    from bga_fanout.geometry import clamp_via_to_pad
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
            cs, cd = clamp_via_to_pad(te.VIA_SIZE, te.VIA_DRILL, p, floors)[:2]
            memo[id(p)] = (cs / 2.0, (cd or 0.0) / 2.0)
        return memo[id(p)]
    pad_r = max((max(q.size_x, q.size_y) for q in foot.pads), default=0.4) / 2
    # a drop's via where the rung's does not fit: the fab ladder's own vias below it, largest first (radius, drill
    # radius) -- the sizes the ladder steps the whole array's vias down to, here one drop's
    drop_vias = sorted({(f['via_diameter'] / 2.0, f['via_drill'] / 2.0) for f in floors
                        if f['via_diameter'] < te.VIA_SIZE - 1e-9}, reverse=True)
    return dict(tw=tw, cl=cl, h2h=h2h, d_seg=tw + cl, vr=te.VIA_SIZE / 2.0, vdr=te.VIA_DRILL / 2.0, inpad=inpad,
                drop_vias=drop_vias,
                grow=te.VIA_SIZE + cl, stack=tw + cl + _rules.HUG_OVER,
                r_home=max(pad_r + tw / 2 + cl, te.VIA_SIZE / 2 + tw / 2 + cl))


def _hit(P, Q, same, sz):
    """can two options' copper not both be laid, each item at its own size? Two nets': any two legs on one layer
    closer than track + clearance, a via that close to a leg of the other's plus its radius (every layer), two vias
    closer than their radii + clearance or their drills than their radii + hole-to-hole. One net's, as the engine's
    raster sees it (it carries no nets): a leg's centreline OUTSIDE its own balls' disks that close to the other's
    copper, and drills at hole-to-hole. Two of one net's vias at one point and size are ONE via (two plane balls
    dropped at one gap site share its barrel, as the human's ground balls round a via do), and each one's leg reaches
    it inside the via's own keep (the engine's raster exempts the via's disk for the stub that ends there)."""
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
    if not same:
        for (a, b, L) in P.legs:
            for (u, v, L2) in Q.legs:
                if L == L2 and _seg_seg(a, b, u, v) < sz['d_seg'] - 1e-6:
                    return True
        return any(_pt_seg(c, u, v) < r + tw2 + cl - 1e-6 for (c, r, _d) in P.vias for (u, v, _L) in Q.legs) or \
            any(_pt_seg(c, a, b) < r + tw2 + cl - 1e-6 for (c, r, _d) in Q.vias for (a, b, _L) in P.legs)
    rh = sz['r_home']
    for X, Y in ((P, Q), (Q, P)):
        for (a, b, L) in X.legs:
            n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.02))
            for i in range(n + 1):
                t = (a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n)
                if any(math.hypot(t[0] - o[0], t[1] - o[1]) < rh for o in X.balls) or \
                        any(math.hypot(t[0] - c[0], t[1] - c[1]) < k for c, k in shared):
                    continue
                if any(L == L2 and _pt_seg(t, u, v) < sz['d_seg'] - 1e-6 for (u, v, L2) in Y.legs) or \
                        any(math.hypot(t[0] - c[0], t[1] - c[1]) < r + tw2 + cl - 1e-6 for (c, r, _d) in Y.vias):
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
    ids = {p.net_id for p in foot.pads if p.net_id and short_name(p.net_name or '') in nets}
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
    (plan_audit.check_static)"""
    import braid as te
    bar = te.TRACK / 2 + te.CLEAR + te.GRID / 2
    for a, b in rays:
        dx, dy = b[0] - a[0], b[1] - a[1]
        t = max(0.0, min(1.0, ((site[0] - a[0]) * dx + (site[1] - a[1]) * dy) / (dx * dx + dy * dy or 1.0)))
        if math.hypot(a[0] + t * dx - site[0], a[1] + t * dy - site[1]) - r < bar - 1e-9:
            return False
    return True


def exit_ray_conflicts(opts, net_of, bus, vr):
    """[(key, i, key2, j)]: a plane ball's DROP option whose via stands in the way out of a BUS escape option -- its
    lane goes on from the exit, straight out past the face (lane_rays' reach), and the via leaves it less than its
    bar (_lanes_clear's), so the two are not both laid. `opts` {ball key: [(kind, option, piece)]}, `net_of` {ball
    key: its net's short name}, `bus` the bus's short names, `vr` the largest via radius a drop takes. The drops are
    planned with the berths (plan_array), before any lane stands to keep them off (_drops' rays: the lanes laid
    already): on the zynq LVDS bus's U5, RFGND's G12 was dropped 0.14 mm past the west face between RX_FRAME's two
    berths, 0.400 mm from each lane for 0.4025, and two RFGND vias 0.4 mm either side of EN_AGC's berth on the south
    face -- the snap found no end connector for the pair and no way out for the single"""
    import braid as te
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
            a = tuple(o.exit_pt)
            b = (a[0] + u[0] * LANE_REACH, a[1] + u[1] * LANE_REACH)
            g = vr + bar
            for cx in range(int(math.floor(min(a[0], b[0]) - g)), int(math.floor(max(a[0], b[0]) + g)) + 1):
                for cy in range(int(math.floor(min(a[1], b[1]) - g)), int(math.floor(max(a[1], b[1]) + g)) + 1):
                    cells[(cx, cy)].append((k, j, a, b))
    out = []
    for k in sorted(opts):
        for i, (kind, o, _pc) in enumerate(opts[k]):
            if kind != 'drop':
                continue
            seen = set()
            for (k2, j, a, b) in cells.get((int(math.floor(o.site[0])), int(math.floor(o.site[1]))), ()):
                if (k2, j) not in seen and _pt_seg(o.site, a, b) - o.r < bar - 1e-9:
                    seen.add((k2, j))
                    out.append((k, i, k2, j))
    return out


def zone_regions(pcb):
    """{layer: [(net id, priority, outline)]}: the board's pours, by layer"""
    out = collections.defaultdict(list)
    for z in pcb.zones or ():
        if z.net_id and len(z.polygon) >= 3:
            out[z.layer].append((z.net_id, z.priority or 0, z.polygon))
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
    name_of = {i: short_name(n.name) for i, n in pcb.nets.items() if n.name}
    segs = [((s.start_x, s.start_y), (s.end_x, s.end_y), s.layer, s.net_id) for s in pcb.segments
            if name_of.get(s.net_id) in legs]
    tooth = pt.teeth(segs, name_of, legs, grid.bbox)
    return [(pk[0], pk[2]) for pk in pt.pockets(prs, tooth, max(grid.pitch_x, grid.pitch_y) / 2.0).values()
            if pk != 'apart']


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


def reserve_ball_vias(pcb, spec=None):
    """The joint fanout's promise to the arrays' OTHER nets, kept by the bus's own fanout: each of their balls keeps
    the via in its own pad. `spec` the joint spec ({"arrays": [{"ref", "others", "drops"}]}), else the file
    FANOUT_JOINT names; neither -- no joint fanout -- does nothing. A stand-in via, of the size the engine lays in that
    pad and locked, is added to `pcb.vias` for each ball, so the bus's menus and the fanout engine leave the site as
    they leave any via; nothing writes it (a fanout writes the board it read plus its own copper). Without it the bus's
    fanout may run a track on B under a ball of another net and wall it in (zynq U1: DDR3_A4 under DDR3_CK_N's M2,
    the A0/A2/A3 teeth round it on F -- no escape at any rung). A PLANE ball's (`drops`: a plane's, a rail's) is not
    kept: it has a way down besides its pad -- a gap's via, a strap to its neighbour -- and kept, the zynq U2's 39 of
    them walled the bus's berths in, its ends crossing 260 times to the chain's 202. Returns the number added."""
    if spec is None:
        path = awx_settings.get('FANOUT_JOINT')
        if not path:
            return 0
        import json
        with open(path, encoding='utf-8') as f:
            spec = json.load(f)
    if getattr(pcb, '_joint_reserved', False):
        return 0
    import braid as te
    from kicad_parser import Via
    from list_nets import escalation_rungs
    from bga_fanout.geometry import clamp_via_to_pad
    floors = escalation_rungs(len(pcb.board_info.copper_layers or ()) or 4)
    n = 0
    for a in spec.get('arrays', ()):
        foot = pcb.footprints.get(a['ref'])
        if foot is None:
            continue
        want = {short_name(x) for x in a.get('others', ())}
        for p in foot.pads:
            if not p.net_id or short_name(p.net_name or '') not in want:
                continue
            size, drill = clamp_via_to_pad(te.VIA_SIZE, te.VIA_DRILL, p, floors)[:2]
            pcb.vias.append(Via(x=p.global_x, y=p.global_y, size=size, drill=drill or te.VIA_DRILL,
                                layers=['F.Cu', 'B.Cu'], net_id=p.net_id, locked=True))
            n += 1
    pcb._joint_reserved = True
    return n


def passives_fixed():
    """the passives stand where they are (FANOUT_PASSIVES_FIXED): the cap placement step moved them once, after the
    whole route's first fanout, and nothing moves them again -- every pad of theirs is copper a plan and a via clear,
    and the route goes round them (whole_route)"""
    return awx_settings.get('FANOUT_PASSIVES_FIXED') == '1'


def movable_refs(pcb, ref):
    """the movable passives: the parts the cap placement step that follows moves (place_fanout_clearance, by its own
    rule: placement.fanout_clearance.movable_cap_refs -- unlocked two-pad C/R/FB parts within its near margin of a
    BGA's ball field), so the joint escape plans and lays its VIAS as if they were not there (its tracks go round
    them: build_menus); every other passive -- the
    channel's, a terminating resistor between the arrays -- is copper nothing will move. None where the board says
    nothing will move them (`_fanout_all_foreign_immovable`, which the engine reads too:
    geometry.immovable_foreign_pads). Read apart from the engine under that mark, a plan ran CTRL_OUT0 through RX10's B
    pad under zynq U1, the engine refused it and laid it a gap over, through the via site the same plan kept for
    TX_FRAME_P, whose ball was left bare. None, too, once the cap step has moved them (passives_fixed)"""
    if getattr(pcb, '_fanout_all_foreign_immovable', False) or passives_fixed():
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
    return got - {ref}


_MOVABLE = {}       # (a board file, its mtime and size) -> its movable caps: the courtyards read off it once


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


def colour_runs(runs, edges, together, want):
    """({key: layer}, [keys left uncoloured]): a layer for each run (`runs` {key: its layers}) -- no two of `edges` on
    one, the two of each `together` pair on one, each on its `want` layer where it has it, else the first it can of
    its outer layer then the inner ones in the stack's order (the inner layers kept for the bus's lanes). One small
    exact solve; a run no colouring holds (a crossing's capacity met by runs some of which never meet) is left out"""
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
    rank = lambda k, L: 0 if L == want.get(k) else 1 + sorted(runs[k], key=lambda L_: (
        L_ not in ('F.Cu', 'B.Cu'), runs[k].index(L_))).index(L)
    mdl.Maximize(sum(1000 * ok[k] for k in runs) - sum(rank(k, L) * x for k in runs for L, x in c[k].items()))
    s = cp_model.CpSolver()
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
    bus_s = {short_name(n) for n in bus}
    out = {}
    for other, n in sorted(count.items(), key=lambda t: (-t[1], t[0])):
        if n < MIN_BUS_NETS:
            continue
        with contextlib.redirect_stdout(io.StringIO()):
            nets = {short_name(x) for x in mb.pair_nets(pcb, ref, other)}
        if len(nets) < MIN_BUS_NETS or nets & bus_s:
            continue                        # (too few the whole route admits; or the bus itself)
        for x in sorted(nets):
            out.setdefault(x, other)
    if not out:
        return {}
    out.update({x: 'bus' for x in bus_s})
    return out


def array_pairs(foot, nets):
    """{base: (P leg, N leg)}: the differential pairs among `nets` (short names) with a ball on the array `foot` --
    pairs.pair_names over its pads' nets, every suffix pair admitted; a base that is a net of its own is no pair
    (pairs.members)"""
    import pairs as _pairs
    here = sorted({short_name(p.net_name or '') for p in foot.pads if p.net_id} & set(nets))
    return {b_: v_ for b_, v_ in _pairs.pair_names(here, admit_all=True).items() if b_ not in here}


def build_menus(pcb, ref, bus, others, other_layers, far=None, drops=(), climb=CLIMB, street=2, only=None,
                vias_only=False, exit_rays=False):
    """Every ball's OPTIONS for plan_array (its escapes, its straps, its drops, each with its copper as a Piece at its
    real size), and what they were built from: a namespace of foot, grid, bus_s, oth_s, drop_s, sz, items, menu (the
    escapes alone, per ball), balls, opts, via_r (a move's via radius), t_menu. `only` (a set of ball keys, NET#PAD):
    those balls alone -- a round that plans again only the balls whose copper did not stand (carry)."""
    import braid as te
    import escape_moves as em
    import fanout_from_plan as fp
    t0 = time.time()
    foot = pcb.footprints[ref]
    grid = em.grid_of(foot)
    if climb is None:
        climb = max(len(grid.xs), len(grid.ys))
    bus_s = {short_name(n) for n in bus}
    oth_s = {short_name(n) for n in others}
    drop_s = {short_name(n) for n in drops} - bus_s - oth_s
    sz = _sizes(pcb, foot)
    skip = movable_refs(pcb, ref)
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
    items, menu, balls, dmenu = {}, {}, {}, {}
    bus_layers = bus_route_layers(pcb)
    rls = run_layers()
    planes = set(plane_layers(pcb))
    pairs_ = array_pairs(foot, bus_s | oth_s)
    partner = {leg: (pn if leg == nn else nn) for _b, (pn, nn) in pairs_.items() for leg in (pn, nn)}
    nid_of = {short_name(n.name): i for i, n in pcb.nets.items() if n.name}
    # (the lanes laid already -- the array's nets this plan does not place, the bus's -- held clear of the drops)
    rays = lane_rays(pcb, grid, foot, {short_name(p.net_name) for p in foot.pads if p.net_id and p.net_name}
                     - bus_s - oth_s - drop_s) if drop_s else []
    regions = zone_regions(pcb) if drop_s else None
    for p in foot.pads:
        nm = short_name(p.net_name or '')
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
            continue
        home = next((L for L in pcb.board_info.copper_layers if L in p.layers), 'F.Cu')
        # (a plane layer only where a leg's layer is the plan's own and priced: on more routing layers than two a via
        # move's run is the colouring's to place, which prices none)
        lays = [home] + [L for L in (bus_layers if nm in bus_s else other_layers)
                         if L != home and not (rls and L in planes)]
        runs = lays[1:] if rls and len(lays) > 2 else None       # (a via move's run on RUN, clear on these)

        def clear(a, b, L, _n=p.net_id, _runs=runs):
            if L == RUN:
                return any(obs(_n, L_).seg_clear(a, b) for L_ in _runs)
            return obs(_n, L).seg_clear(a, b)
        centre = (p.global_x, p.global_y)
        # (a via is a barrel through EVERY copper layer, whatever layers the move runs on -- as a drop's is,
        # _via_clear: checked on the move's own layers alone, an other net's dog-bone via on F/B stood on the bus's
        # In2 tooth, which the plan never saw and the engine refused, the ball left bare -- zynq U1's TX_FRAME_P on
        # three layers)
        with contextlib.redirect_stdout(io.StringIO()):
            moves = em.enumerate_moves(
                p, grid, [home, RUN] if runs else lays, clear,
                lambda q, L, _n=p.net_id, _p=p, _c=centre: _via_clear(
                    pcb, obs, _n, q, sz['inpad'](_p)[0] if q == _c else sz['vr'], sz),
                climb=climb if nm in bus_s else min(climb, CLIMB_OTHER), own_line=True, straight=True,
                street=street, street_pitch=sz['stack'] + 1e-4)
        if runs:
            # each run's layers: those its every stretch on RUN is clear on (enumerate_moves asked any one of them)
            kept = []
            for m in moves:
                if m.layer != RUN:
                    kept.append(m)
                    continue
                m.runs_on = tuple(L_ for L_ in runs
                                  if all(obs(p.net_id, L_).seg_clear(a, b) for a, b, l_ in m.legs if l_ == RUN))
                if m.runs_on:
                    kept.append(m)
            moves = kept
        moves = fp.dedupe_climbs(moves)
        if nm in bus_s and far:
            moves = [m for m in moves if m.direction != far]
        if nm in bus_s and vias_only:
            # (ESCAPE_VIAS at this array: the bus's escapes through a via -- a dog-bone, a via in its pad -- whose lane
            # leaves on whichever routing layer the solve gives it; a ball with none keeps what it has)
            moves = [m for m in moves if m.kind != 'surface'] or moves
        if nm in partner and partner[nm] in nid_of:
            # a pair leg's escapes with room for the PAIR at the exit, as the bus's own fanout keeps them
            # (fanout_from_plan.pair_exit_clear); all of them where none has. A run on RUN keeps the layers with room
            keep = []
            for m in moves:
                if m.layer == RUN:
                    room = tuple(L_ for L_ in m.runs_on
                                 if fp.pair_exit_clear(pcb, p.net_id, nid_of[partner[nm]], m, layer=L_))
                    if room:
                        keep.append((m, room))
                elif fp.pair_exit_clear(pcb, p.net_id, nid_of[partner[nm]], m):
                    keep.append((m, None))
            for m, room in keep:
                if room:
                    m.runs_on = room
            moves = [m for m, _r in keep] or moves
        menu[key] = moves
        dmenu[key] = []
    # (no option into a laid pair's POCKET (laid_pockets), the pair's own room to close past its teeth: an escape there
    # is walled in once it closes -- the zynq DDR's U1 NetR3_2 escaped on B.Cu between DDR3_DQS1's B teeth, 0.77 mm
    # apart, A14 over its end -- and a drop's via there stands in the pair's way -- U2's GND B9 dropped between
    # DDR3_DQS1's berths, the whole route's audit found the pair 0.03 mm into it)
    pockets = laid_pockets(pcb, grid, {short_name(p.net_name) for p in foot.pads if p.net_id and p.net_name}
                           - bus_s - oth_s - drop_s)
    if pockets:
        for key in menu:
            menu[key] = [m for m in menu[key]
                         if not in_pockets(pockets, m.legs or [], [m.site] if m.site is not None else [])]
            dmenu[key] = [d for d in dmenu[key]
                          if not in_pockets(pockets, [d.stub + (d.layer,)] if d.stub else [], [d.site])]
    t_menu = time.time() - t0
    # (a plane ball's straps too: the human's U5 A4 joins A5 down its column, A5 and A6 sharing one via off the
    # array's edge, the bus's lanes in every gap beside them)
    straps = _straps(pcb, grid, items, obs)
    if pockets:
        straps = {k: [s for s in v if not in_pockets(pockets, [(s.a, s.b, s.layer)], [])] for k, v in straps.items()}
    # every ball's options, escapes first (their conflicts are the whole route's own, by index); each with its copper
    # at its real size: a via in the ball's own pad the engine's clamped one, any other the rung's
    opts, via_r = {}, {}
    for key, (nm, p) in items.items():
        own = [(p.global_x, p.global_y)]
        o = []
        for m in menu[key]:
            vias = []
            if m.site is not None:
                r, dr = sz['inpad'](p) if tuple(m.site) == own[0] else (sz['vr'], sz['vdr'])
                vias = [(tuple(m.site), r, dr)]
                via_r[id(m)] = r
            o.append(('escape', m, Piece(nm, list(m.legs), vias, own)))
        o += [('strap', s, Piece(nm, [(s.a, s.b, s.layer)], [], [s.a, s.b])) for s in straps.get(key, ())]
        o += [('drop', d, Piece(nm, [d.stub + (d.layer,)] if d.stub else [], [(d.site, d.r, d.dr)], own))
              for d in dmenu[key]]
        opts[key] = o
    by_net = collections.defaultdict(list)
    for key, (nm, _p) in items.items():
        by_net[nm].append(key)
    legs = [(b_, by_net[pn][0], by_net[nn][0]) for b_, (pn, nn) in sorted(pairs_.items())
            if len(by_net.get(pn, ())) == 1 and len(by_net.get(nn, ())) == 1]
    return types.SimpleNamespace(foot=foot, grid=grid, bus_s=bus_s, oth_s=oth_s, drop_s=drop_s, sz=sz, items=items,
                                 menu=menu, balls=balls, opts=opts, straps=straps, dmenu=dmenu, t_menu=t_menu,
                                 via_r=via_r, pairs=legs, lane_faces=lane_faces(rays))


def plan_array(pcb, ref, bus, others, other_layers, far=None, prefer=None, drops=(), climb=CLIMB, street=2,
               batches=None, workers=None, time_limit=None, log=print, debug=False, only=None, hands=None,
               vias_only=False, exit_rays=False):
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
    escapes' conflicts are the whole route's own (pages_first._conflicts as its ends take them: strict, an F exit
    stacked over a B one allowed). `exit_rays` (the DESTINATION's joint plan, fanout_from_plan): a plane ball's drop
    is held out of the bus's berths' ways out (exit_ray_conflicts), each gap site offering a finer via too."""
    import conflict_groups as cg
    import select_moves as sm
    import source_realize as sr
    from ortools.sat.python import cp_model
    t0 = time.time()
    bm = build_menus(pcb, ref, bus, others, other_layers, far=far, drops=drops, climb=climb, street=street,
                     only=only, vias_only=vias_only, exit_rays=exit_rays)
    bus_s, oth_s, drop_s, sz = bm.bus_s, bm.oth_s, bm.drop_s, bm.sz
    planes_ = set(plane_layers(pcb))             # (a leg on one priced: C_PLANE_LAYER)
    rls = run_layers()                # (more routing layers than two: the via moves' runs on RUN)
    items, menu, balls, opts, straps, dmenu, t_menu = (bm.items, bm.menu, bm.balls, bm.opts, bm.straps, bm.dmenu,
                                                       bm.t_menu)
    via_r = bm.via_r
    import braid as _bd
    _bd.forget_obstacles()           # (the menus were the models' last use here: their memory goes to the solve)
    n_moves = sum(len(v) for v in menu.values())
    # the escapes' conflicts, the whole route's own relation as groups that all conflict pairwise (conflict_groups:
    # pages_first._conflicts pair by pair was 129 s and 3.1 million pairs for the bus alone with every move kind).
    # Every crossing counts (xing 2, asked for here: select_moves' default counts one only where a move climbs) -- a
    # plan of every ball has plain escapes out of
    # the interior, and two of those cross (zynq U1, the plan at the default rule: 85 of 86 clashing pairs in the laid
    # geometry were two plain surface escapes crossing on F.Cu).
    # One net's two ESCAPES conflict as two nets' do: the engine lays each ball's escape on its own, on a raster
    # that carries no nets (zynq U1: five VCC_1V0 balls planned out through one gap, four laid nowhere)
    t1 = time.time()
    tags = {}
    groups, bigroups, gpairs = cg.conflict_groups(menu, stack=True, stack_pitch=sz['stack'],
                                                  via_r=lambda m: via_r.get(id(m), sz['vr']),
                                                  reach_extra=sz['cl'] + sz['tw'] / 2, xing=2, tags=tags)
    pairs = [(a[0], a[1], b[0], b[1]) for a, b in sorted(gpairs)]
    # (the groups of runs on RUN alone -- their lane, crossing and exit conflicts -- are the K layers' capacity; one a
    # via's site or reach states as well is strict: a via stands on every layer)
    on_run = lambda g: bool(tags.get(g)) and all(t[0] in ('lane', 'crossing', 'exit') and t[1] == RUN
                                                 for t in tags[g])
    cap_g = {tuple(sorted(g)) for g in groups if on_run(g)}
    cap_b = {(tuple(sorted(a)), tuple(sorted(b))) for a, b in bigroups if on_run((a, b))}
    del tags
    # built in a canonical order (pages_first's note: the order constraints reach the CP-SAT picks its answer)
    # (a group of one ball's moves alone says nothing its own AtMostOne does not)
    groups = sorted((sorted(g) for g in groups if len({m_[0] for m_ in g}) > 1), key=repr)
    bigroups = sorted(((sorted(a), sorted(b)) for a, b in bigroups if len({m_[0] for m_ in a | b}) > 1), key=repr)
    t_lane = time.time() - t1
    t1 = time.time()
    # a strap or a drop against every other ball's option: geometry at the laid sizes, the candidates found through a
    # 1 mm cell index of the options' boxes (a climb's box spans the array; the all-pairs loop was 10^7 box tests)
    index = collections.defaultdict(list)
    boxes = {}
    for k in sorted(opts):
        for i, (_kind, _o, pc) in enumerate(opts[k]):
            bx = boxes[(k, i)] = _box(pc, sz['grow'])
            for cx in range(int(math.floor(bx[0])), int(math.floor(bx[2])) + 1):
                for cy in range(int(math.floor(bx[1])), int(math.floor(bx[3])) + 1):
                    index[(cx, cy)].append((k, i))
    for k in sorted(opts):
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
                        if _hit(pc, opts[k2][j][2], items[k][0] == items[k2][0], sz):
                            pairs.append((k, i, k2, j))
    if exit_rays:
        pairs += exit_ray_conflicts(opts, {k: items[k][0] for k in items}, bus_s, sz['vr'])
    t_geo = time.time() - t1
    log(f'  joint escape of {ref}: {len(items)} balls ({sum(1 for k in items if items[k][0] in bus_s)} bus, '
        f'{sum(1 for k in items if items[k][0] in drop_s)} plane), {n_moves} moves, '
        f'{sum(len(v) for v in straps.values())} straps, {sum(len(v) for v in dmenu.values())} drops, '
        f'{len(groups)} conflict cliques + {len(bigroups)} bicliques + {len(pairs)} pairs, {time.time() - t0:.0f} s '
        f'(menus {t_menu:.0f}, '
        f'conflicts {t_lane:.0f}, geometry {t_geo:.0f})')

    def cost(key, kind, o):
        nm, p = items[key]
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
        g = (prefer or {}).get(nm) if nm in bus_s else None
        if g:
            t = g['tooth']
            c += C_DEV_MM * math.hypot(o.exit_pt[0] - t[0], o.exit_pt[1] - t[1])
            c += C_DEV_KIND * ((o.direction != g.get('direction')) + _off_layer(o, g.get('layer'))
                               + (o.kind != g.get('kind')))
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
        bus_l = [v[k][i] for k, i in members if items[k][0] in bus_s]
        oth_l = [v[k][i] for k, i in members if items[k][0] not in bus_s]
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

    def exclude(lit, others):
        # `lit` excludes every option of `others`: the groups holding several of them a row each, the most first
        # (exact: a group's members never stand together past its cap anyway), the rest a clause each
        rem = set(others)
        while rem:
            cnt = collections.Counter(gi for x in rem for gi in memb.get(x, ()))
            best = max(cnt.items(), key=lambda t: (t[1], -t[0]), default=None)
            if best is None or best[1] < 2:
                break
            g, cap = cover[best[0]]
            hit = [x for x in g if x in rem]
            mdl.Add(cap * lit + sum(v[k][i] for k, i in hit) <= cap)
            rem.difference_update(hit)
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
    # a differential PAIR leaves the array together -- pairs.harmonise's rule, here a constraint of the one solve
    # rather than a repair after a choice made ball by ball: its two legs served together, each by an escape whose
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
        slots, lit = {}, {}
        for k_ in (kp, kn):
            sl = slots[k_] = collections.defaultdict(list)     # (face, layer, exit x, exit y) -> the leg's options
            for i, (kind, o, _pc) in enumerate(opts[k_]):
                if kind == 'escape':
                    sl[(o.direction, o.layer, round(o.exit_pt[0], 6), round(o.exit_pt[1], 6))].append(i)
                else:
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
                   if sp[:2] == sn[:2] and 0.05 < math.hypot(sp[2] - sn[2], sp[3] - sn[3]) <= reach]
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
    t_p1 = time.time()
    let_go = []
    idx = {served[k].Index(): k for k in served}
    s1 = cp_model.CpSolver()
    s1.parameters.num_workers = 1
    s1.parameters.max_deterministic_time = P1_DET
    if time_limit:
        s1.parameters.max_time_in_seconds = time_limit
    legs_ = {k for _b, kp, kn in bm.pairs for k in (kp, kn)}
    tiers = [('bus and pairs', [k for k in keys if k in served and (items[k][0] in bus_s or k in legs_)]),
             ('others', [k for k in keys if k in served and k not in legs_ and items[k][0] in oth_s]),
             ('plane', [k for k in keys if k in served and items[k][0] in drop_s])]
    held_keys, hint_vals, st1, tier_rep, left_out = [], None, None, [], set()
    for tname, tier in tiers:
        if not tier:
            continue
        while True:
            mdl.ClearAssumptions()
            mdl.AddAssumptions([served[k] for k in held_keys + tier if k not in let_go])
            st1 = s1.Solve(mdl)
            if st1 != cp_model.INFEASIBLE:
                break
            core = sorted(idx[c] for c in s1.SufficientAssumptionsForInfeasibility() if c in idx)
            cand = [k for k in core if k in tier] or core
            if not cand:
                break
            let_go.append(min(cand, key=lambda k_: (rank[k_], k_)))
        st_name = s1.StatusName(st1)
        if st1 == cp_model.UNKNOWN:
            # out of time on the whole tier, undecided: the MOST of it served instead, the earlier tiers held, and
            # those held. zynq U1's 98 plane balls on four layers: every one at once undecided at 240 and at 960,
            # and phase 2's count and cost together left 7 of them unserved; given four times its budget, 2
            mdl.ClearAssumptions()
            mdl.AddAssumptions([served[k] for k in held_keys])
            mdl.Maximize(sum(served[k] for k in tier if k not in let_go))
            st1 = s1.Solve(mdl)
            mdl.ClearObjective()
            out_ = [k for k in tier if k not in let_go and not s1.Value(served[k])] \
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
            mdl.Add(served[key] == 1)
        st1 = cp_model.FEASIBLE if st1 not in (cp_model.OPTIMAL, cp_model.FEASIBLE) else st1
    held_set = set(held_keys)
    t_p1 = time.time() - t_p1
    # PHASE 2, the cost, every ball phase 1 held kept served; a ball it did not hold -- let go, or of a tier it ran out
    # of time on -- earns its weight if it can be served after all (count and cost together for those).
    obj = []
    for key in keys:
        w = 0 if key in held_set else wt[key]
        for i, (kind, o, _pc) in enumerate(opts[key]):
            obj.append((w - cost(key, kind, o)) * v[key][i])
    obj += [-C_VIA * u for u in site_used]
    mdl.Maximize(sum(obj))
    s_ = cp_model.CpSolver()
    # REPRODUCIBLE, as the whole solve is (whole_solve): stopped by a count of interleaved batches, the workers
    # sharing no clauses -- the same model gives the same answer (a wall-clock limit on eight workers gave two answers
    # in two runs of zynq U1's array). `time_limit` caps the wall clock on top, and an answer it stops is not
    # reproducible.
    s_.parameters.num_workers = workers or SOLVE_WORKERS
    s_.parameters.interleave_search = True
    s_.parameters.max_num_deterministic_batches = batches or SOLVE_BATCHES
    s_.parameters.share_glue_clauses = False
    s_.parameters.share_binary_clauses = False
    s_.parameters.subsolvers.extend(SUBSOLVERS_RUNS if rls else SUBSOLVERS)
    if time_limit:
        s_.parameters.max_time_in_seconds = time_limit
    st = s_.Solve(mdl)
    chosen, ci = {}, {}              # ball -> (kind, option); ball -> its index
    if st in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        for key in keys:
            for i, var in enumerate(v[key]):
                if s_.Value(var):
                    chosen[key] = opts[key][i][:2]
                    ci[key] = i
    piece = {key: opts[key][ci[key]][2] for key in chosen}
    uncoloured = []
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
        colour, uncoloured = colour_runs(runs_, sorted(edges), together, want)
        for key, L in colour.items():
            kind, o = chosen[key]
            o2 = dataclasses.replace(o, layer=L, legs=[(a, b, L if l_ == RUN else l_) for a, b, l_ in o.legs])
            chosen[key] = (kind, o2)
            pc = piece[key]
            piece[key] = Piece(pc.net, list(o2.legs), pc.vias, pc.balls)
        for key in uncoloured:
            del chosen[key], ci[key], piece[key]
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
            # the move's own legs, always: the plan proved ITS legs clear of each other, and an exit alone is laid by
            # a search that reaches it by any cells (full_move carries them only under PLAN_PAGES)
            h['legs'] = [(tuple(a), tuple(b), L) for (a, b, L) in o.legs]
        elif kind == 'strap':
            h = {'kind': 'strap', 'to': tuple(o.b), 'layer': o.layer}
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
    rep = dict(status=s_.StatusName(st), balls=len(items), moves=n_moves, conflicts=len(pairs), conflict_groups=len(groups), conflict_bicliques=len(bigroups),
               bus_escaped=kinds[('bus', 'escape')],
               bus_balls=sum(1 for k in items if items[k][0] in bus_s),
               others_escaped=kinds[('other', 'escape')], others_strapped=kinds[('other', 'strap')],
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
               let_go=list(let_go), phase1=s1.StatusName(st1) if st1 is not None else 'nothing to ask',
               phase1_secs=round(t_p1, 1), phase1_tiers=tier_rep,
               uncoloured=list(uncoloured),
               run_layers=dict(collections.Counter(o.layer for k, (kind, o) in chosen.items()
                                                   if kind == 'escape' and o.kind != 'surface')) if rls else None,
               pairs=len(bm.pairs), pairs_escaped=sum(1 for _b, kp, kn in bm.pairs if kp in chosen and kn in chosen),
               hands_held=sorted(hand_rep['held']), hands_free=sorted(hand_rep['free']),
               secs=round(time.time() - t0, 1))
    if debug:
        # the model's pieces, for checking the answer against them (the conflict pairs, the options) and the solve's
        # own account of itself (objective, proved bound, wall time, branches)
        rep['debug'] = dict(items=items, opts=opts, pairs=pairs, groups=groups, bigroups=bigroups, chosen=chosen, cost=cost, sizes=sz,
                            objective=obj_orig if chosen else None, bound=s_.BestObjectiveBound(),
                            wall=s_.WallTime(), branches=s_.NumBranches(), conflicts_cp=s_.NumConflicts())
    log(f'  joint escape of {ref}: {rep["status"]} -- bus {rep["bus_escaped"]}/{rep["bus_balls"]}, others '
        f'{rep["others_escaped"]} escaped + {rep["others_strapped"]} strapped of {rep["others_balls"]}, plane '
        f'{rep["dropped"]} dropped ({rep["dropped_in_pad"]} in pad)'
        + (f' + {rep["plane_strapped"]} strapped' if rep['plane_strapped'] else '') + f' of {rep["plane_balls"]}'
        + (f' on {rep["drop_vias"]} vias' if rep['drop_vias'] != rep['dropped'] else '') + '; '
        f'{rep["climbed"]} climbs, {rep["streets"]} street vias chosen; {len(rep["no_move"])} balls with no option; '
        f'{len(clashes)} chosen pairs clash in the laid geometry; '
        + (f'runs on {rep["run_layers"]}' + (f', {len(uncoloured)} no layer held {uncoloured}' if uncoloured else '')
           + '; ' if rls else '')
        + f'served first {rep["phase1"]} in '
        f'{rep["phase1_secs"]} s' + (f', let go (a proved conflict) {rep["let_go"]}' if rep['let_go'] else '')
        # (each tier's own status where one is not OPTIMAL: a tier out of its time is not held, and its balls are phase
        # 2's -- the line's FEASIBLE said only that an earlier tier held, and hid the plane tier that left six drops)
        + (f' (tiers {", ".join(f"{t} {n} {s}" for t, n, s in tier_rep)})'
           if any(s != 'OPTIMAL' for _t, _n, s in tier_rep) else '')
        + (f'; pairs held to the other end\'s hand {len(rep["hands_held"])}'
           + (f', NO exit pair of it {rep["hands_free"]}' if rep['hands_free'] else '') if hands else '')
        + f'; {rep["secs"]} s')
    # (the model and its solvers collected here, not at the collector's leisure: the CP-SAT model and its variables
    # hold each other, and left standing they were under the engine's lay that follows -- the zynq chain's U1 source
    # realize at 1.09 GB, its plan's own peak 0.77)
    del mdl, s1, s_, v, served
    gc.collect()
    return hints, rep


def lay(board, out, ref, bus, others, other_layers, hints, other_pairs=(), plane_drop='auto'):
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
    import pairs as _pairs
    import ship_vias
    import source_realize as sr
    extra = dict(diff_pair_patterns=[f'{b}*' for b in other_pairs], diff_pair_gap=_pairs.GAP) if other_pairs else {}
    # the movable passives (the decoupling caps under the array) are not obstacles: the cap placement step follows
    # the bus step in the chain and moves them off this copper (geometry.immovable_foreign_pads) -- until it has, once
    # (passives_fixed): every foreign pad then one a via clears
    pcb = parse_kicad_pcb(board)
    if passives_fixed():
        pcb._fanout_all_foreign_immovable = True
    spec = {'net_layers': {**{n: bus_route_layers(pcb) for n in bus}, **{n: list(other_layers) for n in others}},
            'priority': list(bus)}
    tracks, vias_add, vias_rm, failed = generate_bga_fanout(
        pcb.footprints[ref], pcb, net_filter=list(bus) + list(others),
        layers=list(pcb.board_info.copper_layers), track_width=sr.FAN_TRACK, clearance=sr.FAN_CLEAR,
        via_size=te.VIA_SIZE, via_drill=te.VIA_DRILL, exit_margin=0.5, escape_method='jointescape',
        plane_drop=plane_drop, escape_dir_hints=hints, bus=spec, **extra)
    tracks, vias_add, gated = keep_pair_gates(pcb, ref, hints, tracks, vias_add)
    if gated:
        print(f'  joint escape of {ref}: left out, an escape through a planned pair\'s gate: '
              + ', '.join(sorted(pcb.nets[i].name.split('/')[-1] for i in gated)))
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
            legs[h['pair']].append((tuple(h['exit']), h.get('layer')))
    gates = [(g[0][1], g[0][0], g[1][0]) for g in legs.values() if len(g) == 2 and g[0][1] == g[1][1]]
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
    want = {short_name(n) for n in nets}
    vias = [{'x': v.x, 'y': v.y, 'size': v.size, 'net_id': v.net_id} for v in pcb.vias]
    tracks = [{'start': (s.start_x, s.start_y), 'end': (s.end_x, s.end_y), 'layer': s.layer, 'net_id': s.net_id}
              for s in pcb.segments]
    return [f'{short_name(p.net_name)}#{p.pad_number}' for p in pcb.footprints[ref].pads
            if p.net_id and short_name(p.net_name or '') in want and not ball_has_copper(p, vias, tracks, track_width)]


def carry(prev, cur, out, ref, others, drops, grow=1.5, release_near=(), in_place=False):
    """A later round of the whole route keeps the array's other nets and plane balls as the previous round laid them,
    as it keeps its own held teeth: each ball's PIECE on PREV -- its net's tracks and vias joined to it through their
    ends, inside the array's box grown by `grow` (an escape ends past the boundary line; a strap's piece reaches both
    its balls) -- is put on CUR, the round's board, if it still stands there: every track clear on its layer and every
    via on every layer, checked as plan_array checks an option (braid obstacles at the round's sizes, the movable
    passives skipped). OUT is CUR with the standing pieces. Returns (the ball keys NET#PAD to plan again -- a piece
    the round's bus copper now meets, or a ball that had none -- , pieces kept, pieces moved). `release_near` (ball
    keys): a piece whose balls stand within a pitch and a half of one of these is released too, so a ball left with
    no way down or out is planned again with its neighbours rather than round their held copper. `in_place`: the
    pieces are on CUR already (PREV is CUR) -- OUT is CUR with the pieces that do NOT stand taken off it: the first
    round's own copper held to the parts its cap step has just put back over it (fanout_from_plan.joint_hold)."""
    import braid as te
    import fanout_from_plan as fp
    import ship_vias
    from kicad_parser import parse_kicad_pcb
    from kicad_writer import add_tracks_and_vias_to_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        old, now = parse_kicad_pcb(prev), parse_kicad_pcb(cur)
    want = {short_name(n) for n in list(others) + list(drops)}
    name_of = {i: short_name(n.name) for i, n in old.nets.items()}
    id_now = {short_name(n.name): i for i, n in now.nets.items()}
    foot = old.footprints[ref]
    xs, ys = [p.global_x for p in foot.pads], [p.global_y for p in foot.pads]
    box = (min(xs) - grow, min(ys) - grow, max(xs) + grow, max(ys) + grow)
    inside = lambda x, y: box[0] <= x <= box[2] and box[1] <= y <= box[3]     # noqa: E731
    segs, vias = collections.defaultdict(list), collections.defaultdict(list)
    for s in old.segments:
        if name_of.get(s.net_id) in want and inside(s.start_x, s.start_y) and inside(s.end_x, s.end_y):
            segs[s.net_id].append(s)
    for v in old.vias:
        if name_of.get(v.net_id) in want and inside(v.x, v.y):
            vias[v.net_id].append(v)
    near = lambda a, b: abs(a[0] - b[0]) < 1e-3 and abs(a[1] - b[1]) < 1e-3    # noqa: E731
    balls = [p for p in foot.pads if p.net_id and name_of.get(p.net_id) in want]
    key_of = lambda p: f'{name_of[p.net_id]}#{p.pad_number}'                  # noqa: E731
    pieces, claimed = [], set()
    for p in balls:
        if key_of(p) in claimed:
            continue
        nid, got_s, got_v = p.net_id, [], []
        frontier = [(p.global_x, p.global_y)]
        while frontier:
            q = frontier.pop()
            for s in segs[nid]:
                if any(s is t for t in got_s):
                    continue
                a, b = (s.start_x, s.start_y), (s.end_x, s.end_y)
                if near(a, q) or near(b, q):
                    got_s.append(s)
                    frontier += [a, b]
            for v in vias[nid]:
                if not any(v is w for w in got_v) and near((v.x, v.y), q):
                    got_v.append(v)
        pts = [(s.start_x, s.start_y) for s in got_s] + [(s.end_x, s.end_y) for s in got_s] + [(v.x, v.y) for v in got_v]
        keys = sorted({key_of(b) for b in balls if b.net_id == nid
                       and any(near((b.global_x, b.global_y), q) for q in pts)} | {key_of(p)})
        claimed.update(keys)
        if got_s or got_v:
            pieces.append((keys, got_s, got_v))
    sz = _sizes(now, now.footprints[ref])
    skip = movable_refs(now, ref)
    cache = {}
    pos = {key_of(b): (b.global_x, b.global_y) for b in balls}
    stuck = [pos[k] for k in release_near if k in pos]
    if stuck:
        import escape_moves as em
        _g = em.grid_of(foot)
        reach = 1.5 * min(_g.pitch_x, _g.pitch_y)
    near_stuck = lambda keys: any(math.hypot(pos[k][0] - c[0], pos[k][1] - c[1]) < reach   # noqa: E731
                                  for k in keys if k in pos for c in stuck) if stuck else False

    def obs(nid, layer):
        if (nid, layer) not in cache:
            cache[(nid, layer)] = te.build_obstacles(now, nid, {nid}, layer, margin=sz['cl'] + sz['tw'] / 2,
                                                     skip_refs=skip)
        return cache[(nid, layer)]
    stand, moved = [], []
    for keys, ss, vv in pieces:
        nid = id_now.get(name_of[(ss or vv)[0].net_id])
        ok = nid is not None and not near_stuck(keys) \
            and all(obs(nid, s.layer).seg_clear((s.start_x, s.start_y), (s.end_x, s.end_y)) for s in ss) \
            and all(not (obs(nid, L).point_violation((v.x, v.y), pad=v.size / 2 - sz['tw'] / 2) or [0])[0]
                    for v in vv for L in now.board_info.copper_layers)
        (stand if ok else moved).append((keys, ss, vv))
    if in_place:
        from kicad_writer import remove_segments_from_content, remove_vias_from_content
        n2n = {i: n.name for i, n in now.nets.items()}
        content = open(cur, encoding='utf-8').read()
        content, _n = remove_segments_from_content(content, [s for _k, ss, _v in moved for s in ss], n2n)
        content, _n = remove_vias_from_content(content, [v for _k, _s, vv in moved for v in vv], n2n)
        with open(out, 'w', encoding='utf-8') as f:
            f.write(content)
        fp.copy_pro(cur, out)
        kept = {k for keys, _s, _v in stand for k in keys}
        return sorted(key_of(p) for p in balls if key_of(p) not in kept), len(stand), len(moved)
    tracks = [{'start': (s.start_x, s.start_y), 'end': (s.end_x, s.end_y), 'width': s.width, 'layer': s.layer,
               'net_id': id_now[name_of[s.net_id]]} for _k, ss, _v in stand for s in ss]
    new_vias = [{'x': v.x, 'y': v.y, 'size': v.size, 'drill': v.drill, 'layers': list(v.layers),
                 'net_id': id_now[name_of[v.net_id]]} for _k, _s, vv in stand for v in vv]
    with contextlib.redirect_stdout(sys.stderr):
        add_tracks_and_vias_to_pcb(cur, out, tracks, new_vias, [],
                                   net_id_to_name={i: n.name for i, n in now.nets.items()})
        ship_vias.stamp(out, 'joint escape (carried)', print)
    fp.copy_pro(cur, out)
    kept = {k for keys, _s, _v in stand for k in keys}
    return sorted(key_of(p) for p in balls if key_of(p) not in kept), len(stand), len(moved)


def undropped_balls(board, ref, drops, track_width):
    """`ref`'s balls of the plane nets `drops` with no copper of their net on them (bare_balls) and no pour of their net
    on their own layer -- measured on the board, not taken from the engine's report, which a call that lays no drop
    pass leaves as an earlier call wrote it"""
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    poured = {(short_name(z.net_name or ''), z.layer) for z in (pcb.zones or []) if z.net_id}
    layers_of = {f'{short_name(p.net_name or "")}#{p.pad_number}': set(p.layers) for p in pcb.footprints[ref].pads}
    planeless = set(planeless_balls(pcb, ref, drops))
    return [k for k in bare_balls(board, ref, drops, track_width)
            if not any((k.split('#')[0], L) in poured for L in layers_of.get(k, ())) and k not in planeless]


def planeless_balls(pcb, ref, drops):
    """`ref`'s balls of the plane nets `drops` with no plane of their own under them: their net's pour reaches neither
    the ball nor any of its four diagonal gaps (on_own_plane) -- on a split layer, a ball among another supply's (the
    zynq U1's VCC_1V8 balls in VCC_1V0's core island). Never dropped; the route step joins them to their plane, as the
    chain's own fanout leaves every plane ball to it"""
    import escape_moves as em
    foot = pcb.footprints[ref]
    g = em.grid_of(foot)
    hx, hy = g.pitch_x / 2.0, g.pitch_y / 2.0
    regions = zone_regions(pcb)
    drop_s = {short_name(n) for n in drops}
    out = []
    for p in foot.pads:
        nm = short_name(p.net_name or '')
        if not p.net_id or nm not in drop_s:
            continue
        q = (p.global_x, p.global_y)
        if not any(on_own_plane(regions, p.net_id, s) for s in
                   [q] + [(q[0] + sx * hx, q[1] + sy * hy) for sx in (-1, 1) for sy in (-1, 1)]):
            out.append(f'{nm}#{p.pad_number}')
    return out


def fan_array(board, out, ref, bus, others, other_layers, far=None, prefer=None, drops=(), log=print, only=None,
              rungs=None, filter_nets=(), **solve):
    """The array's joint escape at ONE size for the whole fanout: planned and laid at the chain's fan track and via,
    and -- only when a ball is left bare or a plane ball undropped -- the whole of it again at the next rung of the
    fab ladder, the via and the track stepped down TOGETHER (list_nets.escalation_rungs, as the under-pad shrink
    rescue steps them; never one via or one track on its own), until a rung serves every ball, or serves no more than
    the rung above it; the rung that left the fewest bare, then the fewest undropped. OUT is that rung's board. Returns (its sizes, as rules.Rules,
    and a report per rung tried). The chain's own rules are back in place on return. `rungs` (rules.Rules): these
    sizes instead of the ladder -- a later round keeps the first round's; `only`: those balls alone (plan_array);
    `filter_nets`: more other nets for the engine's net filter, already laid (a later round's carried nets) -- the
    engine leaves a ball that carries its net's copper, and never meets an empty filter, which it reads as EVERY net
    (a round that planned plane drops alone fanned a refused bus net that way)."""
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
            hints, rep = plan_array(pcb, ref, bus, others, other_layers, far=far, prefer=prefer, drops=drops, log=log,
                                    only=only, **solve)
            # (the plan's debug -- its every option, group and pair -- let go before the lay but its choice)
            chosen_ = (rep.pop('debug', None) or {}).get('chosen')
            gc.collect()
            laid = f'{stem}.rung{n}.kicad_pcb'
            lay_others = list(others) + [n for n in filter_nets if n not in others]
            # (the plan alone drops the plane balls: the engine's own drop pass after it, blind to the bus's lanes, laid
            # one the plan had refused -- zynq U2's A8, half a pitch off its edge between two berths' lanes)
            with contextlib.redirect_stdout(sys.stderr):
                lay(board, laid, ref, bus, lay_others, other_layers, hints, plane_drop='off')
            und = undropped_balls(laid, ref, drops, r.fan_track) if drops else []
            undropped = len(und)
            bare = bare_balls(laid, ref, list(bus) + list(others), r.fan_track)
            reports.append(dict(track=r.fan_track, via=r.via_size, drill=r.via_drill, bare=bare, undropped=undropped,
                                undropped_balls=und, status=rep['status'], planned_bus=rep['bus_escaped'],
                                planned_others=rep['others_escaped'] + rep['others_strapped'],
                                planned_drops=rep['dropped'] + rep['plane_strapped'], pairs=rep.get('pairs'),
                                pairs_escaped=rep.get('pairs_escaped'), tiers=rep.get('phase1_tiers'),
                                hands_held=rep.get('hands_held'), hands_free=rep.get('hands_free'),
                                chosen=chosen_))
            log(f'  joint escape of {ref} at track {r.fan_track} / via {r.via_size}/{r.via_drill}: {len(bare)} bare '
                f'ball(s){" " + str(bare) if bare else ""}, {undropped} plane ball(s) undropped'
                + (f' {und}' if und else ''))
            if best is not None and (len(bare), undropped) >= best[0]:
                # (a rung that serves no more than the one above it: what is left is no matter of size -- the
                # zynq DDR's U1 GND R20, ringed by other nets' escapes, undropped at every rung, three whole plans
                # of the array more for nothing)
                log(f'  joint escape of {ref}: the rung serves no more than the one above it -- the ladder stops')
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

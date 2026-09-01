#!/usr/bin/env python3
"""braid.py -- the general braid emitter for fanned-out buses.

Inputs: a board whose SOURCE and DESTINATION arrays are both fanned out,
the nets to route, and the destination's reference. Everything else is
read off the board's geometry. Nothing here reads a face, an axis, a
frame or a chip's orientation.

CORRIDORS. Each net's taut path (tooth -> stub end, around the static
copper) is computed and the nets are clustered by how much of their
length runs together (detect_buses): a corridor is a group of nets
that flow together, whatever shape that flow has. Corridors are laid
down largest first. Each gets a SPINE (corridor.py): the mean of its
members' taut paths, relaxed as a string against the static copper and
the corridors already laid, with the obstacles inflated by the bundle's
half-width -- so a corridor that turns a chip's corner turns it with
room for its inner lane, and every lane bends with the spine. From
there on the corridor is described in the spine's own frame: s along
it, o across it.

ORDER. A tooth is BORN IN PLACE when its own offset is free of every
tooth downstream of it (the head-on launch of a face perpendicular to
the flow); otherwise it JOINS from the side, and the first joiner takes
the lane farthest from the teeth so no join leg crosses a lane already
present. The exits mirror this: a stub receives its lane head-on when
its offset is free of every stub upstream; otherwise the lane peels
off to it, first exiter innermost. Launch order and target order are
the lanes' offsets at the two ends of the schedule region; the divers
are the complement of the longest increasing subsequence of that
permutation, and the wave schedule (schedule.py) turns the inversions
into columns of adjacent swaps -- at each crossing the mover on the
back layer, the passed net on the front. A corridor whose joins and
exits are all side legs (the flank "river" of earlier takes) is not a
second kind of thing: nearly all of it dives, and a diver that passes
everyone stays on the back layer for the whole run, which is the
constant-layer river that emerged as a hand mechanism before.

COPPER. All of it is the real router's (connect.py). Every lane is
routed from its tooth to its stub end inside its band -- the corridor
between the neighbouring lanes present on that layer, in (s, o), with
the other layer closed wherever the schedule requires one -- and the
lanes not yet routed are stamped as virtual copper on the layers they
may occupy, so the router places the dive and surface vias where they
fit and never where a later lane must pass. A refused lane feeds back
to the schedule (launch pitch, lead and spacer columns) and the
corridor reruns; what is still refused is reported and left open.

WHERE CORRIDORS MEET. Two corridors never overlap along their length
(the later spine is relaxed against the earlier corridor's tube). Where
one must cross the other -- its exits sit among the other's stubs --
the crossing stretch is reserved (no swap column there) and the later
corridor's lanes route through it on either layer against the earlier
corridor's real copper. That is the v1 of inter-corridor crossings; the
global allocation (which corridor yields, pushing corridors outward to
leave the middle for a wide one) is the next thing to build.
"""
import argparse
import math
import re
import os
import shutil
import sys
from collections import Counter

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402
import topo_strings as ts  # noqa: E402
import connect as cn  # noqa: E402
import corridor as cr  # noqa: E402
import detect_buses as db  # noqa: E402
from schedule import Schedule, lis_keep, lis_keep_weighted  # noqa: E402,F401

TRACK = 0.127
CLEAR = 0.105            # 0.1 spec + 5um so hugs don't sit exactly at 0.1
VIA_SIZE = 0.25
VIA_DRILL = 0.15

MINP = 0.38                    # lane pitch floor at the exits
LPITCH = 0.35                  # pitch of a side-join / side-exit block
BLOCK_GAP = 0.45               # a block starts this far beyond what it clears
HALF_SEP = (TRACK + 0.1) / 2   # two lanes at their band edges clear
LEG_W = 0.5                    # half-width in s of a join / exit leg's band
LEG_REQ = 0.35                 # half-width in s of the stretch a lane
                               # CROSSED by an exit leg must spend on the
                               # other layer. It is what keeps that lane's
                               # dive and surface vias out of the leg's way:
                               # a via must sit LEG_REQ from the leg's
                               # centreline, and a track needs via radius +
                               # clearance + half a track = 0.29 from a via
                               # -- at the old 0.25 the vias landed 0.31
                               # from the leg and closed it (K19 SODT1: two
                               # vias of the lane it crossed, 0.02 mm of
                               # room between them). It is NOT applied to
                               # the leg's owner: the owner turns onto its
                               # leg with a via at the corner, and a rule on
                               # its lane there contradicted the previous
                               # leg's rule 0.4 mm upstream (K19 SCAS, SWE:
                               # closed on both layers)
LEG_O = 0.2                    # ...and beyond its two ends in o: a leg
                               # runs through the densest static copper
                               # (the flank's foreign stubs, both layers)
                               # and its via needs room; the obstacle map
                               # and the other lanes' virtual copper are
                               # the law there, the band only a guide
TOL_S = 0.5                    # "at the same s" for head-on classification
DIST_O = 0.2                   # distinct offsets for head-on classification
HEAD_RUN = 3.0                 # a head-on stub's straight run-in that must
                               # be clear of static copper (its own row of
                               # balls, when it sits on a flank)
CROSS_TUBE = 1.0               # a lane's freedom through a crossing region
RESERVE = 0.3                  # room after the last column
W_FREE = 0.18                  # pitch of a FREE column (two page lanes
                               # crossing: no via, only the lanes' slope)
W_XING = 0.02                  # two-page: pitch of a column with no layer
                               # change in it -- a crossing of two lanes
                               # on different layers costs no length,
                               # only slope (the band floor grows with
                               # it, SLOPE_W); the schedule region is
                               # spent on layer CHANGES alone
SLOPE_W = 0.02                 # extra band half-width per unit |do/ds|:
                               # a steep diagonal's +-0.03 tube holds no
                               # connected cell path on the 0.025 grid
SWIM_TUBE = 1.2                # ribbon swimmer's band half-width: it
                               # WEAVES through the page lattice, so
                               # the neighbour-pinch band is wrong for
                               # it -- its same-layer neighbours are
                               # the very lanes it must cross (K11: a
                               # thin fragmented thread, refused; the
                               # obstacle map and the virtual copper
                               # are the law inside the tube)
PAGE_TUBE = 0.35               # ribbon page lane's dodge room: its
                               # band is floored at this half-width
                               # around its straight line, so copper
                               # that lands ON the line (a dive via, a
                               # swimmer's weave) can be stepped
                               # around -- the free cells were there
                               # and the pinched band cut them off
                               # (K11 SDQ13, forward stuck in 276
                               # cells at its own tooth)
HW_COL = 0.15                  # half-width of a constrained column's
                               # required-layer stretch (the converging
                               # part where two lanes are too close for
                               # one layer)
VIA_ROOM = 0.30                # room for a via between two required
                               # stretches (0.25 dia + clearances)
W_GATE = 0.33                  # narrowest swap column the gated schedule
                               # gets: every clean gated K on the bench
                               # had W >= 0.343 (K21, W=0.322, needed
                               # its third attempt); below it a lane
                               # passed in one column and diving in the
                               # next has no cell for its via, so the
                               # gate yields and columns are spent on
                               # vias instead (see Corridor.run)


def build_obstacles(pcb, nid, kids, layer):
    """A static-copper model for one net on one layer: every foreign
    pad as a disc, every foreign segment as a capsule, every foreign
    via as a disc, all inflated by clearance + half a track. The PLAN
    prices its candidate moves against it, the taut paths and the
    spines are relaxed against it; the braid's copper is routed against
    the router's own model and never consults this one."""
    obs = ts.Obstacles()
    m = CLEAR + TRACK / 2
    for ref, fp in pcb.footprints.items():
        for p in fp.pads:
            if p.net_id == nid:
                continue
            on_layer = any(L == layer or '*' in L for L in p.layers)
            if p.drill and p.drill > 0:
                on_layer = True
            if not on_layer:
                continue
            if p.pad_type == 'np_thru_hole' and p.drill:
                r0 = p.drill / 2
            elif p.shape in ('circle', 'oval'):
                r0 = max(p.size_x, p.size_y) / 2
            else:
                r0 = math.hypot(p.size_x, p.size_y) / 2
            obs.add_disc(p.global_x, p.global_y, r0 + m,
                         f'{ref}.{p.pad_number}')
    for s in pcb.segments:
        if s.net_id == nid or s.net_id in kids or s.layer != layer:
            continue
        obs.add_cap((s.start_x, s.start_y), (s.end_x, s.end_y),
                    s.width / 2 + m, f'seg:{s.net_id}')
    for v in pcb.vias:
        if v.net_id == nid:
            continue
        obs.add_disc(v.x, v.y, v.size / 2 + m, f'via:{v.net_id}')
    obs.build()
    return obs


def array_pad_obstacles(pcb, end_refs, min_pads=10):
    """The pads that decide a corridor's TOPOLOGY: those of the arrays
    the nets start and end on, and of any part large enough to be a
    barrier a bundle must go round rather than an obstacle a lane
    steps around (min_pads is the size threshold; a decoupling cap
    under an array's corner is not a barrier -- K11: it blocked the
    rung between two faces of the same array). Bare pad radii, no
    routing margin."""
    obs = ts.Obstacles()
    for ref, fp in pcb.footprints.items():
        if ref not in end_refs and len(fp.pads) < min_pads:
            continue
        for p in fp.pads:
            if p.pad_type == 'np_thru_hole':
                continue
            r0 = (max(p.size_x, p.size_y) / 2 if p.shape in ('circle', 'oval')
                  else math.hypot(p.size_x, p.size_y) / 2)
            obs.add_disc(p.global_x, p.global_y, r0, f'{ref}.{p.pad_number}')
    obs.build()
    return obs


def _owner(pt, segs, pads):
    """The component whose pad the net's copper connects `pt` to.

    Segment endpoints are graph nodes (vias join layers at one point,
    so a 2-D node graph is enough); a pad is reached when a node lies
    within its copper. Falls back to nearest-pad only when the walk
    reaches no pad at all (an isolated fragment)."""
    def key(x, y):
        return (round(x, 3), round(y, 3))
    adj = {}
    for s in segs:
        a, b = key(s.start_x, s.start_y), key(s.end_x, s.end_y)
        adj.setdefault(a, set()).add(b)
        adj.setdefault(b, set()).add(a)
    start = key(*pt)
    seen = {start}
    stack = [start]
    while stack:
        u = stack.pop()
        for v in adj.get(u, ()):
            if v not in seen:
                seen.add(v)
                stack.append(v)
    for p in pads:
        rx, ry = p.size_x / 2 + 0.02, p.size_y / 2 + 0.02
        for (x, y) in seen:
            if abs(x - p.global_x) <= rx and abs(y - p.global_y) <= ry:
                return p.component_ref
    p = min(pads, key=lambda q: ts.d2((q.global_x, q.global_y), pt))
    return p.component_ref


def endpoints(pcb, names, byname, dest_ref=None):
    """Where each net's corridor must start and finish.

    Default (dest_ref None): the source stub's free end, to the far
    PAD -- what the plan works from before the destination is fanned
    out. With dest_ref: both ends are fanned out and each net has a
    free end at both; the pair is attributed by walking the net's own
    copper (nearest-pad is wrong for a stub dragged across a field).
    A free end is a segment endpoint used once that lies inside no pad
    and no via barrel of the net."""
    ends = {}
    for nm in names:
        nid, net = byname[nm]
        segs = [s for s in pcb.segments if s.net_id == nid]
        cnt = Counter()
        for s in segs:
            cnt[(round(s.start_x, 3), round(s.start_y, 3))] += 1
            cnt[(round(s.end_x, 3), round(s.end_y, 3))] += 1
        anchors = [(p.global_x, p.global_y, max(p.size_x, p.size_y) / 2)
                   for p in net.pads] + \
            [(v.x, v.y, v.size / 2) for v in pcb.vias if v.net_id == nid]
        free = [pt for pt, c in cnt.items() if c == 1 and
                all(math.hypot(pt[0] - ax, pt[1] - ay) > max(0.02, ar)
                    for (ax, ay, ar) in anchors)]
        assert free, (nm, 'no free stub end')
        if dest_ref is not None:
            at_dest = [pt for pt in free
                       if _owner(pt, segs, net.pads) == dest_ref]
            at_src = [pt for pt in free if pt not in at_dest]
            if at_dest and at_src:
                src = max(at_src, key=lambda p: max(ts.d2(p, q)
                                                    for q in at_dest))
                tgt = max(at_dest, key=lambda q: ts.d2(src, q))
                ends[nm] = (src, tgt, dest_ref)
                continue
            free = at_src or free
        if len(free) > 1:
            free.sort(key=lambda pt: -min(ts.d2(pt, (a[0], a[1]))
                                          for a in anchors))
        src = free[0]
        tgt = max(net.pads, key=lambda p: ts.d2((p.global_x, p.global_y),
                                                src))
        ends[nm] = (src, (tgt.global_x, tgt.global_y), tgt.component_ref)
    return ends


def strip_net_segments(txt, net_ids, net_names=()):
    """Remove every (segment ...) block whose net ref matches, in
    EITHER dialect: numeric (net N) or quoted name (#749 lore: boards
    carry both). Paren-balanced."""
    return strip_net_items(txt, 'segment', net_ids, net_names)


def strip_net_items(txt, token, net_ids, net_names=()):
    """strip_net_segments for any top-level item: 'segment' or 'via'."""
    out = []
    i = 0
    while True:
        j = txt.find('(' + token, i)
        if j < 0:
            out.append(txt[i:])
            break
        k, depth = j, 0
        while True:
            c = txt[k]
            if c == '(':
                depth += 1
            elif c == ')':
                depth -= 1
                if depth == 0:
                    break
            k += 1
        block = txt[j:k + 1]
        m = re.search(r'\(net (\d+)\)', block)
        m2 = re.search(r'\(net "([^"]+)"\)', block)
        if (m and int(m.group(1)) in net_ids) or \
                (m2 and m2.group(1) in net_names):
            out.append(txt[i:j].rstrip(' \t'))
            e = k + 1
            if e < len(txt) and txt[e] == '\n':
                e += 1
            i = e
        else:
            out.append(txt[i:k + 1])
            i = k + 1
    return ''.join(out)


def _layer_at(pcb, nid, pt, default):
    return next(
        (s.layer for s in pcb.segments if s.net_id == nid
         and (abs(s.start_x - pt[0]) + abs(s.start_y - pt[1]) < 0.005
              or abs(s.end_x - pt[0]) + abs(s.end_y - pt[1]) < 0.005)),
        default)


def _end_dir(pcb, nid, pt, pads):
    """The direction the net's stub ESCAPES in at its free end `pt`:
    walk the copper from the free end to the pad it reaches, and take
    the longest segment of that stub, pointed toward the free end. The
    stub's last segment is the wrong thing to read (the fanout ends
    many escapes with a 45-degree jog), and pad-to-end is too (the
    escape runs half a pitch off its pad's row); the run is the escape
    itself, at whatever angle the array sits."""
    def key(x, y):
        return (round(x, 3), round(y, 3))
    segs = [s for s in pcb.segments if s.net_id == nid]
    adj = {}
    for s in segs:
        a, b = key(s.start_x, s.start_y), key(s.end_x, s.end_y)
        adj.setdefault(a, []).append((b, s))
        adj.setdefault(b, []).append((a, s))
    start = key(*pt)
    seen = {start}
    order = []          # (segment, node nearer the free end, far node)
    frontier = [start]
    while frontier:
        nxt = []
        for u in frontier:
            inside = any(abs(u[0] - p.global_x) <= p.size_x / 2 + 0.02
                         and abs(u[1] - p.global_y) <= p.size_y / 2 + 0.02
                         for p in pads)
            if inside and u != start:
                continue
            for v, s in adj.get(u, ()):
                if v in seen:
                    continue
                seen.add(v)
                order.append((s, u, v))
                nxt.append(v)
        frontier = nxt
    if not order:
        return (1.0, 0.0)
    s, near, far = max(order, key=lambda t: math.hypot(
        t[0].end_x - t[0].start_x, t[0].end_y - t[0].start_y))
    v = (near[0] - far[0], near[1] - far[1])
    h = math.hypot(*v)
    return (v[0] / h, v[1] / h) if h > 1e-6 else (1.0, 0.0)


def _relax_pitch(vals, floor, pinned=()):
    """Push a sorted list of offsets apart to at least `floor`,
    symmetrically, never moving a pinned index."""
    py = list(vals)
    for _ in range(60):
        moved = False
        for i in range(len(py) - 1):
            g_ = py[i + 1] - py[i]
            if g_ < floor - 1e-9:
                push = (floor - g_) / 2
                if i not in pinned:
                    py[i] -= push if i + 1 not in pinned else 2 * push
                if i + 1 not in pinned:
                    py[i + 1] += push if i not in pinned else 2 * push
                moved = True
        if not moved:
            break
    return py


def _intervals_union(ivs):
    ivs = sorted((a, b) for a, b in ivs if b > a)
    out = []
    for a, b in ivs:
        if out and a <= out[-1][1]:
            out[-1] = (out[-1][0], max(out[-1][1], b))
        else:
            out.append((a, b))
    return out


class Corridor:
    """One corridor: its members, spine, lanes, schedule and copper."""

    def __init__(self, idx, members, ctx, log):
        self.idx = idx
        self.members = list(members)
        self.ctx = ctx
        self.log = log
        self.refused = []
        self.out_segs = {}
        self.out_vias = {}
        self.lane_xy = {}          # nm -> planned board polyline (Eco)
        self.req_xy = {}           # nm -> list of B-required polylines
        self.marks = []            # '+' marks (leg ends, corners)
        self.leg_layer = {}        # side exiter -> the layer of its exit leg
        self.join_leg_s = {}       # joiner -> s of its join leg (after jog)
        self.exit_leg_s = {}       # side exiter -> s of its exit leg
        self.sched_cur = None      # the Schedule plan_columns last ran

    # ------------------------------------------------------------ geometry
    def build_spine(self):
        ctx = self.ctx
        n_m = len(self.members)
        self.H = LPITCH * (n_m - 1) / 2 + LPITCH
        extra = [(p, q, LPITCH) for poly in ctx.laid
                 for p, q in zip(poly, poly[1:])]
        teeth = {nm: ctx.ends[nm][0] for nm in self.members}
        stubs = {nm: ctx.ends[nm][1] for nm in self.members}
        # what a SPINE avoids is BIG PARTS: the pads of every array-
        # scale footprint but the corridor's own end arrays (its lanes
        # join and leave there; inflating them by the bundle's half-
        # width in a gap not much wider than that bowed a straight
        # channel into wiggles), plus the corridors already laid.
        # Tracks, vias and small parts are not barriers to a bundle --
        # a foreign track 0.9 mm off the K15 chord, then a decoupling
        # cap 2 mm off it, each inflated by the half-width, wiggled the
        # spine round them and reserved the schedule region for the
        # wiggles' corners -- they are the lanes' business, and the
        # lanes route round them (or a lane is refused, and says so).
        own = {ctx.src_ref[nm] for nm in self.members} | \
            {ctx.ends[nm][2] for nm in self.members}
        big = {ref for ref, fp in ctx.pcb.footprints.items()
               if len(fp.pads) >= 10 and ref not in own}
        obs = ts.Obstacles()
        for (x, y, r, name) in ctx.spine_obs.discs:
            if name.split('.')[0] in big:
                obs.add_disc(x, y, r, name)
        obs.build()
        spine = ctx.spine_of(self.members, extra=extra, log=self.log,
                             H=self.H, base_obs=obs)
        # extend so every free end projects strictly inside the spine
        P0, d0 = spine.P[0], spine.d[0]
        Pn, dn = spine.P[-1], spine.d[-1]
        back = max([0.3] + [-((t[0] - P0[0]) * d0[0] + (t[1] - P0[1]) * d0[1])
                            + 0.3 for t in teeth.values()])
        fwd = max([0.3] + [((t[0] - Pn[0]) * dn[0] + (t[1] - Pn[1]) * dn[1])
                           + 0.3 for t in stubs.values()])
        self.spine = spine.extend(back, fwd)
        self.teeth, self.stubs = teeth, stubs
        self.st = {nm: self.spine.project_pt(teeth[nm]) for nm in self.members}
        self.se = {nm: self.spine.project_pt(stubs[nm]) for nm in self.members}

    def classify(self):
        """Head-on vs side legs at both ends, the join / exit blocks,
        the launch and target offsets, s0 and s1."""
        ctx, sp = self.ctx, self.spine
        M = self.members
        st, se = self.st, self.se
        self.s0 = max(s for s, _o in st.values()) + 0.3

        # A tooth is BORN IN PLACE unless another member's tooth sits
        # clearly DOWNSTREAM of it at (nearly) its own offset -- its lane
        # would run into that tooth -- or its run to the schedule region
        # hits foreign copper. Teeth at the same s are no obstacle to
        # each other however close: their lanes fan out from the teeth
        # to the pitch-floored launch slots, which is what the launch
        # pitch floor is for (a spine tilted against the face brings
        # neighbouring teeth's parallel runs closer than their pitch).
        def head_launch(nm):
            s_i, o_i = st[nm]
            for om in M:
                if om == nm:
                    continue
                s_j, o_j = st[om]
                if s_j > s_i + TOL_S and abs(o_j - o_i) < DIST_O:
                    return False
            run_end = sp.xy(self.s0, o_i)
            return ctx.obs_but(nm, self.members, ctx.tooth_layer[nm]).seg_clear(
                self.teeth[nm], run_end)

        def head_exit(nm):
            s_i, o_i = se[nm]
            for om in M:
                if om == nm:
                    continue
                s_j, o_j = se[om]
                if s_j < s_i - TOL_S and abs(o_j - o_i) < DIST_O:
                    return False
            # a head-on exit runs straight in at its own offset, so
            # that run must be clear of the ARRAYS' PADS -- the mirror
            # of head_launch's check, against the pad fields only (the
            # corridor's other copper is the lanes' business; checked
            # against every static track it reclassified K21's SDQ0
            # and SRAS and left 3 lanes open). Without it the FIRST
            # stub along a flank, with nothing upstream at its offset,
            # read as head-on and its tail ran along the array's outer
            # ball row (K11 SDQ13 sent to the bottom face by the order
            # model: refused by its own band, the band being the row).
            run_from = sp.xy(s_i - HEAD_RUN, o_i)
            # NB a "direct B exit" branch was tried here (2026-08-31:
            # dest_layer B -> check the straight arrival against real
            # B copper instead of the F ball field) and measured a
            # NO-OP: every south-face stub except the westernmost
            # fails the UPSTREAM-STUB test above first -- face-line
            # stubs share one offset, so the o-crowding check, not
            # this pad-field check, is what decides the excursion.
            # Removing the excursion for B-delivered stubs therefore
            # needs a different head-on MODEL (arrival along the face
            # line, ordered by s), not a different clearance test.
            return ctx.pad_obs.seg_clear(run_from, self.stubs[nm])

        self.heads_l = [nm for nm in M if head_launch(nm)]
        self.joiners = [nm for nm in M if nm not in self.heads_l]
        self.heads_e = [nm for nm in M if head_exit(nm)]
        self.siders = [nm for nm in M if nm not in self.heads_e]
        s1 = []
        if self.heads_e:
            s1.append(min(se[nm][0] for nm in self.heads_e) - 0.6)
        if self.siders:
            s1.append(min(se[nm][0] for nm in self.siders) - 0.3)
        self.s1 = min(s1)

        # sides: a joiner's block lies on the side of the spine AWAY
        # from its own array; an exit block likewise, seen from the stub
        def side_of(o_pt, ref):
            ps = ctx.pcb.footprints[ref].pads
            c = (sum(p.global_x for p in ps) / len(ps),
                 sum(p.global_y for p in ps) / len(ps))
            _sc, oc = sp.project_pt(c)
            return 1 if o_pt >= oc else -1

        self.join_side = {nm: side_of(st[nm][1], ctx.src_ref[nm])
                          for nm in self.joiners}
        self.exit_side = {nm: side_of(se[nm][1], ctx.ends[nm][2])
                          for nm in self.siders}

    def _clear_block(self, base, sg, s_from, s_to, nm, step=0.1, tries=15):
        """Push a block's innermost lane outward until its run along
        the spine from s_from to s_to is clear of the static copper on
        both layers: a block is placed beyond the corridor's OWN teeth
        or stubs, but the same flank carries every other net's teeth
        too (K15: the first joiner lane ran 0.08 mm from a foreign stub
        end, and the router refused it)."""
        sp, ctx = self.spine, self.ctx
        base0 = base
        for k in range(tries):
            a, b = sp.xy(s_from, base), sp.xy(s_to, base)
            ok = {L: ctx.obs_but(nm, self.members, L).seg_clear(a, b)
                  for L in ('F.Cu', 'B.Cu')}
            if k == 0 and os.environ.get('BLOCK_DEBUG'):
                self.log(f'    block probe {nm}: o={base:.3f} '
                         f'({a[0]:.2f},{a[1]:.2f})->({b[0]:.2f},{b[1]:.2f}) {ok}')
            if all(ok.values()):
                break
            base += sg * step
        if abs(base - base0) > 1e-9:
            self.log(f'  block on side {"+" if sg > 0 else "-"} pushed '
                     f'{abs(base - base0):.2f} mm clear of static copper '
                     f'(s {s_from:.1f}..{s_to:.1f})')
        return base

    def _leg_s(self, nm, s_l, oa, ob, at_tooth, placed):
        """Where a leg of `nm` spanning o in [oa, ob] at s_l really runs:
        straight in (s, o) -- unless another member's free end sits in
        its way (two flank teeth in one column, 0.28 mm apart: K19's
        SRAS over SWE), or a leg already placed runs there, when it
        jogs half a pitch along the spine to the clearer side.

        `placed` is the legs placed before this one, as (s, lo, hi):
        legs are placed one at a time so two that would coincide cannot
        both jog to the same place. A free end at the leg's OWN end on
        the other layer is not in the way -- two stubs can end at one
        point on two layers (K21 SRAS on B, SCKE1 on F, the fanout's
        doing), and the two legs can end there too; it is the second
        leg's run that must move, and the leg-vs-leg test moves it."""
        own = self.st[nm] if at_tooth else self.se[nm]
        own_L = (self.ctx.tooth_layer if at_tooth else self.ctx.dest_layer)[nm]
        ends = []
        for om in self.members:
            if om == nm:
                continue
            for p, L in ((self.st[om], self.ctx.tooth_layer[om]),
                         (self.se[om], self.ctx.dest_layer[om])):
                if (L != own_L and abs(p[0] - own[0]) < 0.05
                        and abs(p[1] - own[1]) < 0.05):
                    continue
                ends.append(p)
        lo_, hi_ = min(oa, ob), max(oa, ob)

        def clash(s):
            # a jog is a whole pitch, so an end or a leg exactly a pitch
            # away is clear -- compared with a tolerance, or the float
            # residue of 2.6 - 0.35 vs 1.6 + 0.35 reads as a clash and
            # sends the leg two pitches off, over the next tooth
            n = sum(1 for p in ends
                    if abs(p[0] - s) < LPITCH - 1e-6
                    and lo_ - 0.05 < p[1] < hi_ + 0.05)
            n += sum(1 for (ps, plo, phi) in placed
                     if abs(ps - s) < TRACK + 0.1 + 0.02 and plo < hi_ and phi > lo_)
            return n
        if not clash(s_l):
            return s_l
        best = None
        for cand in (s_l + LPITCH, s_l - LPITCH, s_l + 2 * LPITCH,
                     s_l - 2 * LPITCH):
            n = clash(cand)
            if not n:
                return cand
            if best is None or n < best[0]:
                best = (n, cand)
        return best[1]

    def offsets(self, ly_floor):
        """Launch and target offsets for the current pitch floor."""
        st, se = self.st, self.se
        # head-on launches: tooth offsets at the launch pitch floor
        # (pushed one way only, as the trunk always did)
        hl = sorted(self.heads_l, key=lambda nm: st[nm][1])
        Ly = []
        for nm in hl:
            v = st[nm][1]
            Ly.append(v if not Ly else max(v, Ly[-1] + ly_floor))
        launch_o = {nm: Ly[i] for i, nm in enumerate(hl)}
        # joiner blocks, per side: the first joiner takes the lane
        # farthest from the teeth, so no join leg crosses a lane already
        # present. "First" is by the LEG's s, not the tooth's: two teeth
        # in one column jog one leg half a pitch, and ordering by the
        # teeth gave the outer lane to the leg that had jogged
        # DOWNSTREAM -- straight across the other's lane (K19 SRAS over
        # SWE, refused against SWE's virtual copper every attempt).
        self.join_block = {}
        self.join_leg_s = {}
        # (DIRECT join slots -- each joiner at its own tooth offset
        # instead of a block beyond the whole field -- were tried for
        # the two-page ribbon and measured WORSE: the reshuffled launch
        # permutation wrecks the pages (K19: F13/B5/sw1 -> F8/B6/sw5,
        # opens 2 -> 5). The block's excursion realises a joiner's
        # crossings in copper-free space, which is what keeps the
        # launch order page-friendly; the price it charges -- the
        # outermost joiner refused at the search budget -- is paid by
        # the x4 refusal retry in run() instead.)
        for sg in (-1, 1):
            js = [nm for nm in self.joiners if self.join_side[nm] == sg]
            if not js:
                continue
            ext = max([sg * v for v in Ly] + [sg * st[nm][1] for nm in js])
            base = sg * max(ext + BLOCK_GAP, -LPITCH * (len(js) - 1) / 2)
            far = base + sg * LPITCH * (len(js) - 1)
            placed = []
            for nm in sorted(js, key=lambda nm: st[nm][0]):
                s_l = self._leg_s(nm, st[nm][0], st[nm][1], far, True, placed)
                self.join_leg_s[nm] = s_l
                placed.append((s_l, min(st[nm][1], far), max(st[nm][1], far)))
            js.sort(key=lambda nm: (self.join_leg_s[nm], st[nm][0]))
            base = self._clear_block(base, sg, min(self.join_leg_s[nm] for nm in js),
                                     self.s0, js[0])
            for k, nm in enumerate(js):
                launch_o[nm] = base + sg * LPITCH * (len(js) - 1 - k)
                self.join_block[nm] = launch_o[nm]
        # head-on exits: stub offsets at the exit pitch floor
        he = sorted(self.heads_e, key=lambda nm: se[nm][1])
        py = _relax_pitch([se[nm][1] for nm in he], MINP)
        target_o = {nm: py[i] for i, nm in enumerate(he)}
        # exit blocks, per side. Head-on-launched side exits (ports)
        # take the block's inner positions in exit order (first exiter
        # innermost: no leg crosses a lane still present). Side exits
        # that JOINED from the side keep their join order -- a join
        # block's order is fixed by the join rule and its exit order
        # usually is its reverse; sorting that inside the shared region
        # cost every column a quarter of its width (K15), and sorting it
        # on the joiners' run-in had no room for the vias. Their exit
        # legs cross the lanes still between them and their stubs BY
        # LAYER instead (lay_lanes requires the crossed lanes on the
        # other layer there) -- the constant-layer "river" of the
        # earlier takes, as a rule of the exits rather than a mechanism.
        self.exit_block = {}
        for sg in (-1, 1):
            xs = [nm for nm in self.siders if self.exit_side[nm] == sg]
            if not xs:
                continue
            ports = sorted((nm for nm in xs if nm not in self.join_block),
                           key=lambda nm: se[nm][0])
            joined = sorted((nm for nm in xs if nm in self.join_block),
                            key=lambda nm: -st[nm][0])
            order = ports + joined
            ext = max([sg * v for v in py] + [sg * se[nm][1] for nm in xs])
            base = sg * max(ext + BLOCK_GAP, -LPITCH * (len(xs) - 1) / 2)
            base = self._clear_block(base, sg, self.s1,
                                     max(se[nm][0] for nm in xs), order[0])
            for k, nm in enumerate(order):
                target_o[nm] = base + sg * LPITCH * k
                self.exit_block[nm] = target_o[nm]
        if os.environ.get('TWO_PAGE') == '1' \
                and os.environ.get('FACE_COMB', '0') == '1' \
                and self.exit_block:
            # DEFAULT OFF (measured 2026-08-31, both forms): with the
            # order reshuffled, K35 69v/6 open (-11 vias, +2 opens),
            # K32 90/3 (best K32 completion), K28 +8 vias; with the
            # order PRESERVED and only the geometry compressed, K35
            # 88/7 -- worse than no comb (80/4) -- and the recorded
            # K28 +12 (its 3 under-pad B stubs' shared o's change the
            # ride geometry the lanes then pay for). A scale- and
            # form-dependent trade, not a win. FACE_COMB=1 to study;
            # comb-off baselines re-verified exact (50/0, 32/0, 75/1).
            # FACE COMB BY LAYER (the reworked head-on model for
            # face-line stubs). The block above stacks ONE comb at
            # LPITCH -- 17 side exits at K35 = a 6 mm excursion, the
            # plan-audit's 10 mm white detours, and the depth is pure
            # layer-blindness: an F ride and a B ride at the same
            # offset cross for FREE (the ribbon's own principle; the
            # human's face river interleaves layers exactly so). So
            # re-slot each side's comb per DELIVERED layer: two
            # interleaved combs sharing the o range, each ordered as
            # before within its layer (no same-layer leg crosses a
            # same-layer ride still present), the B comb 0.02 out so
            # the target order stays total. An all-F board has an
            # empty B comb and reproduces the old slots EXACTLY.
            # FACE_COMB=0 restores the single comb.
            #
            # ORDER IS PRESERVED, GEOMETRY COMPRESSED (the sp7
            # lesson): re-slotting changed the TARGET ORDER wherever B
            # stubs interleaved the comb (the recorded K28 carries 3
            # under-pad B escapes; its schedule shifted and cost +8
            # vias). The order -- and with it the pages and the whole
            # schedule -- stays the single comb's; only the ride
            # offsets (py) compress per layer. A non-monotone py is
            # legal: the pairs it re-orders are opposite-layer, and
            # those cross free.
            self._order_o = dict(target_o)
            for sg in (-1, 1):
                xs = [nm for nm in self.siders
                      if self.exit_side[nm] == sg and nm in self.exit_block]
                if not xs:
                    continue
                order = sorted(xs, key=lambda nm: sg * self.exit_block[nm])
                base = self.exit_block[order[0]]
                kL = {}
                for nm in order:
                    L = self.ctx.dest_layer[nm]
                    o_ = base + sg * LPITCH * kL.get(L, 0) \
                        + (sg * 0.02 if L == 'B.Cu' else 0.0)
                    kL[L] = kL.get(L, 0) + 1
                    target_o[nm] = o_
                    self.exit_block[nm] = o_
        self.launch_o, self.target_o = launch_o, target_o
        _oo = getattr(self, '_order_o', None) or target_o
        self.target = sorted(self.members,
                             key=lambda nm: _oo.get(nm, target_o[nm]))
        self.launch = sorted(self.members, key=lambda nm: launch_o[nm])
        self.Ly = [launch_o[nm] for nm in self.launch]
        self.py = [target_o[nm] for nm in self.target]

    def reserve_intervals(self):
        """s-intervals that hold no swap column: corner wedges, and the
        stretches where this corridor runs through one laid earlier."""
        sp, ctx = self.spine, self.ctx
        # a lane at offset o has its corner displaced by o*tan(turn/2)
        # along the spine: that is the wedge, plus a margin -- not the
        # whole half-width, which for a shallow bend reserved everything
        omax = max([abs(v) for v in self.Ly + self.py] + [0.0])
        ivs = []
        for (_i, s_c, turn) in sp.corners():
            half = omax * math.tan(math.radians(min(abs(turn), 170.0) / 2)) + 0.3
            ivs.append((s_c - half, s_c + half))
        cross = []
        if ctx.laid:
            within = self.H + LPITCH
            ss = np.arange(self.s0, self.s1, 0.1)
            marks = []
            for s in ss:
                p = sp.xy(float(s), 0.0)
                near = any(ts.seg_pt_dist(a, b, p) <= within
                           for poly in ctx.laid
                           for a, b in zip(poly, poly[1:]))
                marks.append(near)
            a = None
            for s, m in zip(ss, marks):
                if m and a is None:
                    a = s
                if not m and a is not None:
                    cross.append((a - 0.3, s + 0.3))
                    a = None
            if a is not None:
                cross.append((a - 0.3, self.s1 + 0.3))
        self.cross_iv = _intervals_union(cross)
        self.reserved = _intervals_union(ivs + cross)
        # u(s): arc-length with the reserved stretches collapsed
        S_pts, U_pts = [self.s0], [0.0]
        for (a, b) in self.reserved:
            a, b = max(a, self.s0), min(b, self.s1)
            if b <= a:
                continue
            ua = U_pts[-1] + (a - S_pts[-1])
            S_pts += [a, b]
            U_pts += [ua, ua]
        U_pts.append(U_pts[-1] + (self.s1 - S_pts[-1]))
        S_pts.append(self.s1)
        self.S_pts, self.U_pts = np.array(S_pts), np.array(U_pts)
        self.L_free = float(U_pts[-1])

    def u_of(self, s):
        return np.interp(s, self.S_pts, self.U_pts)

    def s_of_u(self, u):
        # the inverse: U_pts is non-decreasing; take the LAST s of a flat
        U, S = self.U_pts, self.S_pts
        u = float(u)
        k = int(np.searchsorted(U, u, side='right')) - 1
        k = max(0, min(len(U) - 2, k))
        if U[k + 1] > U[k]:
            return float(S[k] + (u - U[k]) * (S[k + 1] - S[k]) / (U[k + 1] - U[k]))
        return float(S[k + 1])

    # ------------------------------------------------------------ lanes
    def column_layout(self, cols, sched):
        """The columns' positions along the free length (u), and the
        length they need. A FREE column (two page lanes crossing) is a
        crossing without a via and takes W_FREE; a constrained one
        (a swimmer in it) must leave via room from the swimmer's
        previous crossing when its layer changes there, and from the
        launch for its first crossing on the other layer from its
        tooth: HW_COL either side of each column is the converging
        part where the two lanes are too close for one layer, and the
        via sits between. Returns (u, need)."""
        u = []
        last_col, last_layer = {}, {}
        tl = self.ctx.tooth_layer
        two = sched is not None and getattr(sched, 'two_page', False)
        prev_gate = False
        for k, col in enumerate(cols):
            if two:
                # TWO PAGES: a column is priced by its VIA NEED, not by
                # who is in it. A grouped swimmer crossing page lanes on
                # its settled layer is as free as an F x B pair -- every
                # crossing is between two lanes on different layers --
                # so only a column where some lane's layer CHANGES costs
                # W_GATE and via room; everything else is W_XING. A
                # change is vs the lane's previous crossing OR ITS BIRTH
                # layer: a B-page lane born on F must dive before its
                # FIRST crossing, free crossings included (K4's SDQ11
                # had all three of its crossings in tiny columns just
                # past s0 and nowhere to put its dive via). An EMPTY
                # column is a spacer the schedule put there for a via:
                # full pitch. A gated column also holds its neighbour
                # off at W_GATE (prev_gate), so its via has room along
                # the corridor on both sides.
                def _pairs(d, p):
                    if sched.is_free(d, p):
                        return ((d, sched.page[d]), (p, sched.page[p]))
                    Ld, Lp = sched.pair_layers(d, p)
                    return ((d, Ld), (p, Lp))
                changes = any(
                    last_layer.get(nm, tl.get(nm, 'F.Cu')) != L
                    for (d, p) in col for nm, L in _pairs(d, p))
                gate_col = (not col) or changes
                pitch = W_GATE if (gate_col or prev_gate) else W_XING
                prev_gate = gate_col
                uk = (u[-1] + pitch) if u else pitch
                for (d, p) in col:
                    for nm, L in _pairs(d, p):
                        # via room along the corridor between two
                        # crossings on different layers (a page lane
                        # changes at most at birth, before it is in
                        # last_layer, so this never fires for one)
                        if nm in last_layer and last_layer[nm] != L:
                            uk = max(uk, u[last_col[nm]] + 2 * HW_COL + VIA_ROOM)
                        last_col[nm], last_layer[nm] = k, L
                u.append(uk)
                continue
            # single page: the recorded ladders' exact layout
            free_col = bool(col) and sched is not None and \
                all(sched.is_free(d, p) for d, p in col)
            pitch = W_FREE if free_col else W_GATE
            uk = (u[-1] + pitch) if u else pitch
            for (d, p) in col:
                if sched is not None and sched.is_free(d, p):
                    continue
                Ld, Lp = sched.pair_layers(d, p) if sched else ('B.Cu', 'F.Cu')
                for nm, L in ((d, Ld), (p, Lp)):
                    if sched is not None and sched.page.get(nm):
                        continue                 # a page lane never changes
                    if nm in last_layer and last_layer[nm] != L:
                        uk = max(uk, u[last_col[nm]] + 2 * HW_COL + VIA_ROOM)
                    last_col[nm], last_layer[nm] = k, L
            u.append(uk)
        # the reserve after the last column is one more pitch, as the
        # uniform layout had it ((n + 1) columns' worth of length)
        need = (u[-1] + W_GATE) if u else 0.0
        return u, need

    def lay_lanes(self, cols, sched=None):
        """From a schedule: column positions, required-layer intervals,
        the diver windows, and every lane's (s, o) polyline. `sched`
        (default: the one plan_columns last ran) gives the pages."""
        sp = self.spine
        M = self.members
        sched = sched or self.sched_cur
        two = getattr(sched, 'two_page', False)
        n = len(cols)
        L_avail = self.L_free - RESERVE
        u, need = self.column_layout(cols, sched)
        # the layout is stretched (or squeezed) to the free length, as
        # the uniform layout was: with one page every column is a
        # constrained one at W_GATE and this is exactly (k + 1) W
        scale = L_avail / need if need > 0 else 1.0
        if getattr(sched, 'two_page', False) and u and scale > 1.0:
            # surplus length is WATERFILLED over the gaps: every pitch
            # is raised to a common level T (never lowered -- the gated
            # pitches and via-room gaps are minima) with sum = L_avail,
            # so the layout is as close to UNIFORM as the constraints
            # allow. Proportional stretch kept the W_XING columns tiny
            # relative to the gated ones, and an equal ADD left them at
            # 0.1 when the need approached the room (K11: 15 columns in
            # 4.95 mm, W=0.096, six lanes refused against the escape
            # field's maze -- where the single-page braid lays the same
            # count at ~0.33 uniform and routes). Compression below
            # uniform appears only at a K that genuinely lacks room.
            gaps_p = [b - a for a, b in zip([0.0] + u, u)] + [W_GATE]
            lo_, hi_ = 0.0, L_avail
            for _ in range(50):
                T = (lo_ + hi_) / 2
                if sum(max(p, T) for p in gaps_p) > L_avail:
                    hi_ = T
                else:
                    lo_ = T
            T = lo_
            filled = [max(p, T) for p in gaps_p[:-1]]
            u = []
            acc = 0.0
            for p in filled:
                acc += p
                u.append(acc)
        else:
            u = [x * scale for x in u]
        pitches = [b - a for a, b in zip([0.0] + u, u)]
        self.W = min(pitches) if pitches else L_avail
        self.layout_need, self.cols = need, cols
        self.all_cols = list(cols)
        # slot midpoints: before the first column, between columns,
        # after the last
        s_m = [self.s_of_u(u[0] / 2 if u else L_avail / 2)]
        for k in range(1, n):
            s_m.append(self.s_of_u((u[k - 1] + u[k]) / 2))
        if n:
            s_m.append(self.s_of_u((u[-1] + L_avail) / 2))
        self.invol, self.first, self.last = {}, {}, {}
        for k, col in enumerate(cols):
            for (d, p) in col:
                self.invol.setdefault(d, []).append(k)
                self.invol.setdefault(p, []).append(k)
                self.first.setdefault(d, k)
                self.last[d] = k
        # REQUIRED layers. A page lane is on its page over the whole
        # schedule region (from a via's room past s0, so a tooth on the
        # other layer can dive), which is what makes an F-page / B-page
        # crossing free. At a constrained column (a swimmer in it) the
        # two lanes are required on the layers pair_layers gives, over
        # HW_COL either side of the column -- and NOT the whole column:
        # a swimmer on F at k and on B at k+2 must change layer between
        # the two, and a requirement that spans each whole column leaves
        # it no cell to put the via in.
        req = {nm: [] for nm in M}
        for k, col in enumerate(cols):
            pitch = min(u[k] - (u[k - 1] if k else 0.0),
                        (u[k + 1] if k + 1 < n else L_avail) - u[k])
            hw = 0.4 * pitch + 0.05
            if getattr(sched, 'two_page', False):
                # a W_XING column still needs a threadable crossing
                # window: outside the req interval the crossed lane's
                # virtual copper may hold BOTH layers, and a slot
                # narrower than a track and two clearances is a wall
                # (K4 SDQ14 -- an F-page lane that never vias -- stuck
                # against SDQ11's F virtual either side of a 0.07
                # half-width crossing). Same-layer overlaps of the
                # widened intervals are harmless; different-layer ones
                # cannot happen, the change columns stand 2 HW_COL +
                # VIA_ROOM apart by the layout
                hw = max(hw, HW_COL)
            a, b = self.s_of_u(u[k] - hw), self.s_of_u(u[k] + hw)
            for (d, p) in col:
                # a free crossing: each lane on its page there (the
                # router keeps a layer between requirements on its own)
                Ld, Lp = sched.pair_layers(d, p) if sched else ('B.Cu', 'F.Cu')
                req[d].append((a, b, Ld))
                req[p].append((a, b, Lp))
        # EXIT LEGS THAT CROSS LANES. In a side-exit block the lanes
        # leave one by one, and every leg crosses the lanes still
        # present between it and its stub. The rule is the block's, not
        # the leg's: ALL its legs are on one layer -- the stubs' (a leg
        # ending on its stub's layer costs no via there) -- and a lane
        # is on the OTHER layer from the first leg that crosses it until
        # its own corner, where it turns onto its leg with a via. The
        # per-leg choice this replaces (a layer per leg by the majority
        # of the lanes it crossed) let two legs 0.4 mm apart disagree,
        # and put a rule on the leg's OWNER too: a lane crossed at s and
        # exiting at s + 0.4 was required on B until s + 0.25 and on F
        # from s + 0.15 (K19 SCAS, SWE -- closed on both layers, refused
        # before the router saw them). The owner needs no rule: the
        # crossed lanes' copper, real or virtual, is on the other layer
        # under its leg, so the leg goes where it can, and its via sits
        # at the corner, a whole pitch from the previous leg.
        trank = {nm: i for i, nm in enumerate(self.target)}
        py = self.py
        self.exit_leg_s = {}
        placed = []
        for nm in sorted(self.exit_block, key=lambda n: (self.se[n][0], abs(self.exit_block[n]))):
            s_e, o_e = self.se[nm]
            o_l = py[trank[nm]]
            s_l = self._leg_s(nm, s_e, o_l, o_e, False, placed)
            self.exit_leg_s[nm] = s_l
            placed.append((s_l, min(o_l, o_e), max(o_l, o_e)))
        self.leg_layer = {}
        crossings = {}                       # crossed lane -> [leg s]
        cross_by = {}                        # crossed lane -> [(s, owner)]
        leg_cross = {}                       # leg owner -> [crossed lanes]
        for sg in (-1, 1):
            xs = [nm for nm in self.exit_block if self.exit_side.get(nm) == sg]
            if not xs:
                continue
            n_b = sum(1 for nm in xs if self.ctx.dest_layer[nm] == 'B.Cu')
            leg_L = 'B.Cu' if 2 * n_b > len(xs) else 'F.Cu'
            for nm in xs:
                self.leg_layer[nm] = leg_L
                s_l = self.exit_leg_s[nm]
                o_l, o_e = self.exit_block[nm], self.se[nm][1]
                lo_, hi_ = min(o_l, o_e), max(o_l, o_e)
                for om in M:
                    if om == nm:
                        continue
                    if om in self.exit_block:
                        o_m, s_end = self.exit_block[om], self.exit_leg_s[om]
                    else:
                        o_m, s_end = self.target_o[om], self.se[om][0]
                    if s_end > s_l + 0.05 and lo_ < o_m < hi_:
                        crossings.setdefault(om, []).append(s_l)
                        cross_by.setdefault(om, []).append((s_l, nm))
                        leg_cross.setdefault(nm, []).append(om)
        self.crossings = crossings
        leg_req_min = {}
        if two:
            # TWO-PAGE LEG ECONOMICS. A leg crossing a lane on the
            # OTHER layer is free -- the block-wide layer rule priced
            # every crossing as a forced dive (2 vias per crossed
            # lane), which is exactly the SA7-class 4-via overspend
            # (t7 K28: the human pays 2). Each leg picks the layer
            # that minimises what is actually paid: 2 per crossed
            # PAGE lane on its own layer, 1 if it differs from its
            # lane's page (the corner via), 1 if it differs from the
            # stub's layer. A crossed page lane dives only under a
            # SAME-layer leg; a swimmer adapts per leg. Overlapping
            # opposite-layer intervals from adjacent disagreeing legs
            # are dropped in pairs (the K19 lesson: both layers
            # closed refuses the lane before the router sees it); the
            # obstacle map adjudicates there.
            for nm in self.exit_block:
                own = sched.page.get(nm) if sched else None
                pgs = [sched.page.get(om) if sched else None
                       for om in leg_cross.get(nm, ())]
                cost = {}
                for L in ('F.Cu', 'B.Cu'):
                    c = 2 * sum(1 for p in pgs if p == L)
                    if own is not None and own != L:
                        c += 1
                    if self.ctx.dest_layer[nm] != L:
                        c += 1
                    cost[L] = c
                self.leg_layer[nm] = min(('F.Cu', 'B.Cu'),
                                         key=lambda L: cost[L])
            ivs = {}
            for om, hits in cross_by.items():
                own_p = sched.page.get(om) if sched else None
                for (s_l, owner) in hits:
                    Lg = self.leg_layer[owner]
                    if own_p is not None and own_p != Lg:
                        continue             # different layers: free
                    other = 'B.Cu' if Lg == 'F.Cu' else 'F.Cu'
                    a = s_l - LEG_REQ
                    b = s_l + LEG_REQ
                    if om in self.exit_block:
                        b = min(b, self.exit_leg_s[om] - 0.03)
                    else:
                        b = min(b, self.se[om][0] - 0.03)
                    if b > a:
                        ivs.setdefault(om, []).append((a, b, other))
            for om, vv in ivs.items():
                kept_iv = [iv for iv in vv
                           if not any(o[2] != iv[2] and iv[0] < o[1]
                                      and o[0] < iv[1]
                                      for o in vv if o is not iv)]
                if kept_iv:
                    leg_req_min[om] = min(a for (a, _b, _L) in kept_iv)
                    req[om].extend(kept_iv)
        else:
            for om, legs_s in crossings.items():
                leg_L = self.leg_layer.get(om) or next(iter(self.leg_layer.values()))
                other = 'B.Cu' if leg_L == 'F.Cu' else 'F.Cu'
                a = min(legs_s) - LEG_REQ
                b = max(legs_s) + LEG_REQ
                if om in self.exit_block:
                    # ...but never past its own corner: the via goes there
                    b = min(b, self.exit_leg_s[om] - 0.03)
                else:
                    b = min(b, self.se[om][0] - 0.03)
                if b > a:
                    req[om].append((a, b, other))
        if two:
            # RIBBON page rules. A page lane is required on its page
            # over the whole schedule region -- from just past its
            # birth via (0.45 for the via and its clearances when the
            # tooth is on the other layer) to just before its landing
            # via -- which is what makes every crossing with it free
            # for a lane on the other layer. The req stops short of an
            # exit corner and of any exit leg that crosses the lane
            # (those stretches carry their own rules above).
            tlr, dlr = self.ctx.tooth_layer, self.ctx.dest_layer
            line = {}
            for nm in M:
                s_a = (self.join_leg_s[nm] if nm in self.join_block
                       else max(self.s0, self.st[nm][0]))
                line[nm] = (s_a, self.launch_o[nm], self.s1, py[trank[nm]])

            def _o_at(ln, s):
                s_a, o_a, s_b, o_b = ln
                t = (s - s_a) / max(s_b - s_a, 1e-9)
                return o_a + t * (o_b - o_a)

            def _cross_s(a_, b_):
                lo_s = max(line[a_][0], line[b_][0])
                hi_s = min(line[a_][2], line[b_][2])
                if hi_s <= lo_s:
                    return None
                d0 = _o_at(line[a_], lo_s) - _o_at(line[b_], lo_s)
                d1 = _o_at(line[a_], hi_s) - _o_at(line[b_], hi_s)
                if d0 == d1:
                    return None
                x = lo_s + (hi_s - lo_s) * d0 / (d0 - d1)
                return x if lo_s <= x <= hi_s else None
            for nm in M:
                pg = sched.page.get(nm)
                if not pg:
                    continue
                # the page req must COVER the lane's first and last
                # geometric crossings: a B-page riser that crosses its
                # neighbour inside the birth stretch left that stretch
                # stamped on BOTH layers exactly across the crossing
                # and sealed the neighbour in (K11 SDQ13, 276 cells).
                # The birth/landing via moves out toward the launch or
                # exit run when a crossing comes that early.
                xs_ = [x for om in M if om != nm and sched.inverted(nm, om)
                       and (x := _cross_s(nm, om)) is not None]
                a = self.s0 + (0.05 if tlr[nm] == pg else 0.45)
                if tlr[nm] != pg and xs_:
                    a = min(a, max(self.s0 + 0.02, min(xs_) - HW_COL))
                b = self.s1 - (0.05 if dlr[nm] == pg else 0.45)
                if dlr[nm] != pg and xs_:
                    b = max(b, min(self.s1 - 0.02, max(xs_) + HW_COL))
                if nm in self.exit_block:
                    b = min(b, self.exit_leg_s[nm] - 0.35)
                if nm in leg_req_min:
                    # the page req stops only at a CONFLICTING leg's
                    # dive stretch -- a free (other-layer) crossing no
                    # longer cuts it short
                    b = min(b, leg_req_min[nm] - 0.03)
                if b > a:
                    req[nm].append((a, b, pg))
            # SWIMMER x SWIMMER needs no assigned crossing: swimmers
            # route LAST, one at a time, against each other's REAL
            # copper -- an unrouted swimmer stamps no mid-corridor
            # virtual at all (see virtual_of), because a fixed pair of
            # layers at a fixed line intersection just recreated the
            # rigidity the ribbon removes (K28: 3/27, every swimmer
            # refused against the others' both-layer virtual walls).
            # (RESERVED HOPS -- planned via sites at each swimmer's
            # run boundaries, stamped as virtual vias for the pages to
            # clear -- were tried and measured WORSE: the hop midpoint
            # lands between page lines under a lane pitch apart, so
            # the reservation walls the PAGES instead: K11 10/11 ->
            # 5/11. The swimmer's real bottleneck on this fanout is
            # the F layer saturated by all-F escapes; step 3.)
        # POSSIBLE back-layer intervals: a diver may be on B only from
        # after the last foreign pass before its first own swap until
        # before the first foreign crossing after its last -- it must be
        # on F while passed -- derived from the SAME required intervals,
        # so the two can never disagree. A non-diver is on F in the
        # corridor (B only in the tail, for a stub the fanout left on
        # B); one born on B may stay there until it is first passed.
        bwin = {}
        for nm in M:
            wins = [(self.s1 - 0.1, 1e9)]
            if two and sched.page.get(nm) is None:
                # a ribbon swimmer may weave anywhere: the page lanes'
                # copper and its own assigned crossings are the law
                wins.append((-1e9, 1e9))
            # the diver window is derived from the SCHEDULE's B rules
            # (those inside the corridor): a B stretch in the tail --
            # an exit block's -- is already inside the tail window, and
            # fed to this formula it read as a dive whose window opened
            # at the last F pass, which closed B on a B-born tooth
            # (K21 SCAS: refused at its own tooth, stuck after 1 cell)
            b_iv = sorted((xa, xb) for (xa, xb, L) in req[nm]
                          if L == 'B.Cu' and xa < self.s1 - 0.1)
            f_iv = sorted((xa, xb) for (xa, xb, L) in req[nm] if L == 'F.Cu')
            if b_iv:
                b0, b1 = b_iv[0][0], b_iv[-1][1]
                lo = max((xb for (xa, xb) in f_iv if xb <= b0), default=-1e9)
                hi = min((xa for (xa, xb) in f_iv if xa >= b1), default=1e9)
                wins.append((lo, hi))
            if self.ctx.tooth_layer[nm] == 'B.Cu':
                # born on B: may stay there until first passed on F
                hi = min((xa for (xa, xb) in f_iv), default=1e9)
                wins.append((-1e9, hi))
            bwin[nm] = wins
        self.req, self.bwin = req, bwin

        # lane centrelines in (s, o): tooth (or its join leg's end), the
        # column midpoints in the order the schedule gives, the target
        # slot at s1, then the tail (head-on) or the run to the exit leg
        orders = [list(self.launch)]
        for col in cols:
            o = list(orders[-1])
            for (d, p) in col:
                i, j = o.index(d), o.index(p)
                o[i], o[j] = o[j], o[i]
            orders.append(o)
        Ly, py = self.Ly, self.py
        u_first = u[0] if u else L_avail

        def morph_t(s):
            return max(0.0, (self.u_of(s) - u_first) / max(self.L_free - u_first, 1e-9))

        def slot(t, i):
            return (1 - t) * Ly[i] + t * py[i]
        self.mid, self.legs, self.jogs = {}, {}, {}
        for nm in M:
            s_t, o_t = self.st[nm]
            s_e, o_e = self.se[nm]
            legs = []
            jogs = []
            if nm in self.join_block:
                s_l = self.join_leg_s[nm]
                if abs(s_l - s_t) > 1e-9:
                    jogs.append(((s_t, o_t), (s_l, o_t)))
                legs.append((s_l, o_t, self.join_block[nm]))
                pts = [(s_l, self.join_block[nm])]
            else:
                pts = [(s_t, o_t)]
            if two:
                # RIBBON: hold the launch offset at the region start,
                # then run STRAIGHT to the target slot at s1
                pts.append((self.s_of_u(0.0), self.launch_o[nm]))
            else:
                for k in range(n + 1):
                    pts.append((s_m[k], slot(morph_t(s_m[k]), orders[k].index(nm))))
            pts.append((self.s1, py[trank[nm]]))
            if nm in self.exit_block:
                s_l = self.exit_leg_s[nm]
                pts.append((min(s_l, s_e) if s_l < s_e else s_e, py[trank[nm]]))
                if s_l > s_e + 1e-9:
                    pts.append((s_l, py[trank[nm]]))
                legs.append((s_l, py[trank[nm]], o_e))
                if abs(s_l - s_e) > 1e-9:
                    jogs.append(((s_l, o_e), (s_e, o_e)))
            else:
                pts.append((s_e, o_e))
            # keep s strictly non-decreasing (a joiner far behind s0 is
            # fine; a head-on tooth sits before the first midpoint)
            clean = [pts[0]]
            for p in pts[1:]:
                if p[0] >= clean[-1][0] - 1e-9:
                    clean.append((max(p[0], clean[-1][0]), p[1]))
            self.mid[nm] = clean
            self.legs[nm] = legs
            self.jogs[nm] = jogs
        # board polylines of the plan (Eco, virtual copper, windows)
        self.lane_xy = {}
        self.mid_xy = {}
        for nm in M:
            pieces = []
            for a_, b_ in zip(self.mid[nm], self.mid[nm][1:]):
                xy = sp.lane_xy([a_, b_])
                pieces.append(((a_[0] + b_[0]) / 2, xy))
            self.mid_xy[nm] = pieces
            poly = [self.teeth[nm]]
            if nm in self.join_block:
                s_l, oa, ob = self.legs[nm][0]
                poly.append(sp.xy(s_l, oa))        # the jog, if any
                poly.append(sp.xy(s_l, ob))
            for _sm, xy in pieces:
                for p in xy:
                    if math.hypot(p[0] - poly[-1][0], p[1] - poly[-1][1]) > 1e-6:
                        poly.append(p)
            if nm in self.exit_block:
                s_l, oa, ob = self.legs[nm][-1]
                poly.append(sp.xy(s_l, oa))
                poly.append(sp.xy(s_l, ob))        # ...and back along the jog
            poly.append(self.stubs[nm])
            self.lane_xy[nm] = [p for i, p in enumerate(poly)
                                if i == 0 or math.hypot(p[0] - poly[i - 1][0],
                                                        p[1] - poly[i - 1][1]) > 1e-6]
        # ---- LEDGER-INFORMED DIAMOND RESERVATION (two-page only).
        # The cut and diamond ledgers (cut_ledger.py) measured both
        # STATIC resources ample -- what starves a swimmer is DYNAMIC:
        # by its turn, the other lanes' real+virtual copper has eaten
        # the both-layer-clear spots exactly where its layer changes
        # must land. So reserve one diamond per required change at
        # plan time, placed by the ledger's rules: statically clear on
        # BOTH layers at barrel radius, near the swimmer's own line,
        # and a lane-pitch clear of every other lane's line at that s.
        # (The naive reservation was measured WORSE once -- hops
        # landed between page lines and walled the pages; the
        # distance rule is what makes this one safe to stamp.) The
        # spots ride virtual_vias_of, so every lane routed while the
        # owner is still unrouted keeps clear of them.
        self.hops = {}
        if getattr(sched, 'two_page', False) \
                and os.environ.get('HOP_RESERVE', '1') == '1':
            two_obs = {L: self.ctx.obs_but(M[0], M, L)
                       for L in ('F.Cu', 'B.Cu')}
            pad_r = (VIA_SIZE - TRACK) / 2

            def line_o(om, s):
                ms_ = self.mid[om]
                if not (ms_[0][0] - 1e-9 <= s <= ms_[-1][0] + 1e-9):
                    return None
                return float(np.interp(s, [p[0] for p in ms_],
                                       [p[1] for p in ms_]))

            for nm in M:
                if sched.page.get(nm) is not None:
                    continue
                # the swimmer's crossings with PAGE lanes, each forcing
                # the layer opposite the page it crosses
                want = []
                for om in M:
                    P = sched.page.get(om)
                    if om == nm or P is None or not sched.inverted(nm, om):
                        continue
                    lo_s = max(self.mid[nm][0][0], self.mid[om][0][0])
                    hi_s = min(self.mid[nm][-1][0], self.mid[om][-1][0])
                    if hi_s - lo_s < 0.1:
                        continue
                    S = np.arange(lo_s, hi_s, 0.05)
                    d = np.array([line_o(nm, s) - line_o(om, s) for s in S])
                    for i in np.where(np.sign(d[:-1]) != np.sign(d[1:]))[0]:
                        want.append((float(S[i]),
                                     'B.Cu' if P == 'F.Cu' else 'F.Cu'))
                if not want:
                    continue
                want.sort()
                seq = ([(self.mid[nm][0][0], self.ctx.tooth_layer[nm])]
                       + want
                       + [(self.mid[nm][-1][0], self.ctx.dest_layer[nm])])
                spots = []
                for (s_a, L_a), (s_b, L_b) in zip(seq, seq[1:]):
                    if L_a == L_b or s_b - s_a < 0.1:
                        continue
                    got = None
                    mid_s = (s_a + s_b) / 2
                    for ds in sorted(np.arange(s_a + 0.05, s_b - 0.049, 0.05),
                                     key=lambda v: abs(v - mid_s)):
                        o0 = line_o(nm, ds)
                        if o0 is None:
                            continue
                        for do in (0.0, .15, -.15, .3, -.3, .45, -.45,
                                   .6, -.6, .9, -.9, 1.2, -1.2):
                            xy = sp.xy(ds, o0 + do)
                            if any(two_obs[L].point_violation(xy, pad=pad_r)
                                   is not None for L in ('F.Cu', 'B.Cu')):
                                continue
                            if any(om != nm and (oo := line_o(om, ds))
                                   is not None and abs(o0 + do - oo) < 0.30
                                   for om in M):
                                continue
                            got = xy
                            break
                        if got:
                            break
                    if got:
                        spots.append(got)
                if spots:
                    self.hops[nm] = spots

    def allowed(self, nm, s, L):
        if any(xa <= s <= xb and RL != L for (xa, xb, RL) in self.req.get(nm, ())):
            return False
        if L == 'B.Cu':
            return any(lo <= s <= hi for (lo, hi) in self.bwin[nm])
        return True

    def allowed_vec(self, nm, S, L):
        ok = np.ones(S.shape, dtype=bool)
        for (xa, xb, RL) in self.req.get(nm, ()):
            if RL != L:
                ok &= ~((S >= xa) & (S <= xb))
        if L == 'B.Cu':
            inb = np.zeros(S.shape, dtype=bool)
            for (lo, hi) in self.bwin[nm]:
                inb |= (S >= lo) & (S <= hi)
            ok &= inb
        return ok

    def in_cross(self, s):
        return any(a <= s <= b for (a, b) in self.cross_iv)

    def _band_samples(self, nm, s_lo, s_hi, step=0.002, eps=1e-7):
        """The s values the lane's band edges are evaluated at (see
        band_of): a fine grid over [s_lo, s_hi], every lane's polyline
        vertices, and each STEP of the structure -- a lane's ends, a
        layer rule's bounds, and every s where another lane crosses
        this one -- as a pair of samples a hair either side, so the
        interpolation between samples never straddles a step."""
        pts = {float(v) for v in np.arange(s_lo, s_hi + step, step)}
        ms = np.array([p[0] for p in self.mid[nm]])
        mo = np.array([p[1] for p in self.mid[nm]])
        for om in self.members:
            brk = [p[0] for p in self.mid[om]]
            brk += [v for (xa, xb, _L) in self.req.get(om, ()) for v in (xa, xb)]
            brk += [v for (lo, hi) in self.bwin.get(om, ()) for v in (lo, hi)
                    if abs(v) < 1e8]
            if om != nm:
                # where the two lanes cross: d(s) = o_m(s) - o_nm(s) is
                # piecewise linear between the union of both vertex
                # sets, so a sign change locates each root exactly
                os_ = np.array([p[0] for p in self.mid[om]])
                oo_ = np.array([p[1] for p in self.mid[om]])
                a, b = max(ms[0], os_[0]), min(ms[-1], os_[-1])
                if b > a:
                    v = np.array(sorted({a, b} | {float(x) for x in ms if a < x < b}
                                        | {float(x) for x in os_ if a < x < b}))
                    d = np.interp(v, os_, oo_) - np.interp(v, ms, mo)
                    for i in range(len(v) - 1):
                        if d[i] == 0.0:
                            brk.append(float(v[i]))
                        elif d[i] * d[i + 1] < 0:
                            t = d[i] / (d[i] - d[i + 1])
                            brk.append(float(v[i] + t * (v[i + 1] - v[i])))
                    if d[-1] == 0.0:
                        brk.append(float(v[-1]))
            for v in brk:
                v = float(v)
                if s_lo - 1.0 <= v <= s_hi + 1.0:
                    pts.update((v - eps, v, v + eps))
        return np.array(sorted(pts))

    def band_of(self, nm):
        """The lane's corridor as a cell mask: between the neighbouring
        lanes present on that layer (never narrower than a grid cell),
        closed where the schedule requires the other layer; the join
        and exit legs as rectangles in (s, o); a loose tube through a
        crossing region.

        The neighbour search is a function of s alone, so it runs on a
        1-D sample of s (_band_samples) and the band edges lo(s), hi(s)
        are interpolated onto the window's cells -- exact wherever they
        are linear between samples, which the sample set arranges: a
        2 um grid plus every vertex and every step, the steps doubled.
        Evaluated per cell instead (K21 profile), the 20 neighbours'
        interpolations over a 700k-cell window were 7.6 s of a 28 s
        braid, the router's own search 0.2 s."""
        sp = self.spine
        cache = {}
        BIG = 1e6

        def band(xs, ys, L):
            key = (float(xs[0]), float(xs[-1]), float(ys[0]), float(ys[-1]),
                   len(xs), len(ys))
            if key not in cache:
                X, Y = np.meshgrid(xs, ys, indexing='ij')
                cache.clear()
                S, O = sp.project(X, Y)
                cache[key] = (S, O, self._band_samples(nm, float(S.min()),
                                                       float(S.max())))
            S, O, sg = cache[key]
            ms = np.array([p[0] for p in self.mid[nm]])
            mo = np.array([p[1] for p in self.mid[nm]])
            present = (S >= ms[0] - 1e-9) & (S <= ms[-1] + 1e-9)
            o_nm = np.interp(S, ms, mo)
            okL = self.allowed_vec(nm, S, L)
            sc = getattr(self, 'sched_cur', None)
            if (sc is not None and getattr(sc, 'two_page', False)
                    and sc.page.get(nm) is None):
                # a ribbon SWIMMER weaves through the page lattice: the
                # neighbour-pinch band is wrong for it -- its same-layer
                # neighbours are the very lanes it crosses, and they
                # squeezed it to a thin fragmented thread (K11,
                # refused). A wide tube around its straight line; the
                # obstacle map and the virtual copper are the law
                lo = o_nm - SWIM_TUBE
                hi = o_nm + SWIM_TUBE
            else:
                o_nm1 = np.interp(sg, ms, mo)
                prev = np.full(sg.shape, -BIG)
                nxt = np.full(sg.shape, BIG)
                for om in self.members:
                    if om == nm:
                        continue
                    os_ = np.array([p[0] for p in self.mid[om]])
                    oo_ = np.array([p[1] for p in self.mid[om]])
                    pres = (sg >= os_[0] - 1e-9) & (sg <= os_[-1] + 1e-9)
                    if not pres.any():
                        continue
                    o_m = np.interp(sg, os_, oo_)
                    m = pres & self.allowed_vec(om, sg, L)
                    prev = np.where(m & (o_m < o_nm1), np.maximum(prev, o_m), prev)
                    nxt = np.where(m & (o_m > o_nm1), np.minimum(nxt, o_m), nxt)
                lo1 = np.where(prev > -BIG / 2, (prev + o_nm1) / 2 + HALF_SEP, -BIG)
                hi1 = np.where(nxt < BIG / 2, (nxt + o_nm1) / 2 - HALF_SEP, BIG)
                lo = np.interp(S, sg, lo1)
                hi = np.interp(S, sg, hi1)
                if sc is not None and getattr(sc, 'two_page', False):
                    # a RIBBON page lane may dodge locally: the
                    # neighbours' virtual/real copper is the law, the
                    # pinch only a guide (PAGE_TUBE)
                    lo = np.minimum(lo, o_nm - PAGE_TUBE)
                    hi = np.maximum(hi, o_nm + PAGE_TUBE)
            # never narrower than a grid cell: at the stub ends the
            # lanes are 0.25 apart and the corridor formula gives
            # 0.0115 -- a band that on an unlucky grid alignment holds
            # no cell at all. Clearance to the neighbours' copper is
            # the obstacle map's job. (A floor growing with the lane's
            # slope was tried for K28 SDQ7 -- slope 6 where two movers
            # pass it -- and measured inert THERE: band_conn.py shows
            # the 0.03 band connected end to end; what refuses that
            # lane is C5 inside the corridor, see Known walls. Under
            # TWO pages it is load-bearing: a W_XING crossing is a
            # steep diagonal, and the +-0.03 tube around it holds no
            # connected cell path, so the floor grows with the local
            # slope of the lane's own centreline.)
            if getattr(self, 'sched_cur', None) is not None \
                    and getattr(self.sched_cur, 'two_page', False):
                dms = np.maximum(np.diff(ms), 1e-9)
                sl = np.abs(np.diff(mo)) / dms
                seg_i = np.clip(np.searchsorted(ms, S, side='right') - 1,
                                0, len(sl) - 1)
                fl = 0.03 + SLOPE_W * np.minimum(sl[seg_i], 30.0)
            else:
                fl = 0.03
            lo = np.minimum(lo, o_nm - fl)
            hi = np.maximum(hi, o_nm + fl)
            ok = present & okL & (O >= lo) & (O <= hi)
            for (s_l, oa, ob) in self.legs[nm]:
                rect = ((np.abs(S - s_l) <= LEG_W)
                        & (O >= min(oa, ob) - LEG_O) & (O <= max(oa, ob) + LEG_O))
                ok |= rect & okL
            for ((sa, oa), (sb, ob)) in self.jogs.get(nm, ()):
                rect = ((S >= min(sa, sb) - LEG_O) & (S <= max(sa, sb) + LEG_O)
                        & (np.abs(O - oa) <= LEG_O))
                ok |= rect & okL
            for (a, b) in self.cross_iv:
                ok |= ((S >= a) & (S <= b) & present
                       & (np.abs(O - o_nm) <= CROSS_TUBE))
            return ok
        return band

    def virtual_of(self, unrouted):
        """Centrelines of lanes not routed yet, on every layer the
        schedule lets them occupy, as copper the router must clear --
        except through a crossing region, where the plan is not a
        promise."""
        sp = self.spine
        sc = getattr(self, 'sched_cur', None)
        two = sc is not None and getattr(sc, 'two_page', False)
        segs = []
        for om in unrouted:
            # a ribbon SWIMMER's mid-corridor line is not a promise
            # either -- it weaves wherever the A* takes it -- so only
            # its rigid ends are stamped; stamped on both layers along
            # its whole line it walled every lane that must cross it
            swim_om = two and sc.page.get(om) is None
            # the layers a lane may occupy change at every required-
            # interval boundary, so a polyline piece is split there and
            # each part stamped on its own layers -- deciding per whole
            # piece put an 8 mm exit run on BOTH layers through the one
            # millimetre where another lane's leg crosses it (K15)
            cuts = sorted({v for (xa, xb, _L) in self.req.get(om, ())
                           for v in (xa, xb)}
                          | {v for (lo, hi) in self.bwin.get(om, ())
                             for v in (lo, hi) if abs(v) < 1e8})
            s_end = self.mid[om][-1][0]
            s_start = self.mid[om][0][0]
            for a_, b_ in zip(self.mid[om], self.mid[om][1:]):
                sa, sb = a_[0], b_[0]
                inner = [v for v in cuts if sa + 1e-6 < v < sb - 1e-6]
                bounds = [sa] + inner + [sb]
                for s_a, s_b in zip(bounds, bounds[1:]):
                    s_mid = (s_a + s_b) / 2
                    if self.in_cross(s_mid):
                        continue
                    t_a = (s_a - sa) / max(sb - sa, 1e-9)
                    t_b = (s_b - sa) / max(sb - sa, 1e-9)
                    o_a = a_[1] + t_a * (b_[1] - a_[1])
                    o_b = a_[1] + t_b * (b_[1] - a_[1])
                    xy = sp.lane_xy([(s_a, o_a), (s_b, o_b)])
                    # the tail -- the piece that ends AT the stub --
                    # is stamped on the stub's layer only, like an
                    # exit jog: two nets' stubs end at one point on
                    # the two layers (the fanout gives a gap to one net
                    # per layer), and a tail on both layers walled the
                    # other net's end (K28 SWE on B, under SDQ14's
                    # tail). A lane that reaches its tail on the other
                    # layer still owns the stub's layer to land on.
                    # The head -- the piece that starts AT the tooth --
                    # likewise on the tooth's layer only (K28 SA4's F
                    # tooth under SA1's B tooth at the same point).
                    tail = abs(s_b - s_end) < 1e-6 and om not in self.exit_block
                    head = abs(s_a - s_start) < 1e-6 and om not in self.join_block
                    if swim_om and not (tail or head):
                        continue
                    for p_, q_ in zip(xy, xy[1:]):
                        for L in ('F.Cu', 'B.Cu'):
                            if tail and L != self.ctx.dest_layer[om]:
                                continue
                            if head and L != self.ctx.tooth_layer[om]:
                                continue
                            if self.allowed(om, s_mid, L):
                                segs.append((p_, q_, L))
            for i, (s_l, oa, ob) in enumerate(self.legs[om]):
                a_, b_ = sp.xy(s_l, oa), sp.xy(s_l, ob)
                # an exit leg is on its block's layer and nothing else:
                # the lanes it crosses are on the other layer under it,
                # and a stamp there would wall the very layer they are
                # required on
                is_exit = om in self.exit_block and i == len(self.legs[om]) - 1
                if is_exit and om in self.leg_layer:
                    segs.append((a_, b_, self.leg_layer[om]))
                    continue
                for L in ('F.Cu', 'B.Cu'):
                    if self.allowed(om, s_l, L):
                        segs.append((a_, b_, L))
            for j, ((sa, oa), (sb, ob)) in enumerate(self.jogs.get(om, ())):
                a_, b_ = sp.xy(sa, oa), sp.xy(sb, ob)
                # a jog runs along the free end's own row, on that end's
                # layer: the join jog leaves the tooth, the exit jog
                # arrives at the stub. Stamped on both layers, an exit
                # jog to a B stub walled the F stub another net ends
                # at the same point (K21 SCKE1: stuck after one cell)
                is_exit_jog = (om in self.exit_block and abs(sb - self.se[om][0]) < 1e-6
                               and abs(ob - self.se[om][1]) < 1e-6)
                L = (self.ctx.dest_layer[om] if is_exit_jog
                     else self.ctx.tooth_layer[om])
                segs.append((a_, b_, L))
        return segs

    def virtual_vias_of(self, unrouted):
        """The via each unrouted side exiter will need at its corner --
        where its lane turns onto its exit leg and changes to the leg's
        layer -- as a via the router must clear. The band alone does
        not protect the site: a neighbour's band edge sits exactly a
        via's clearance from this lane's centreline, so a neighbour
        hugging its edge there (K19 SCAS: SA7 0.23 mm off, SWE 0.26)
        leaves no legal via cell when the corner's owner is routed."""
        sp = self.spine
        out = [sp.xy(self.exit_leg_s[om], self.exit_block[om])
               for om in unrouted if om in self.exit_leg_s]
        # ...plus every unrouted swimmer's RESERVED DIAMONDS (#622
        # reservation pass): the spots its layer changes will need,
        # kept clear of everything routed before it
        for om in unrouted:
            out.extend(getattr(self, 'hops', {}).get(om, ()))
        return out

    # ------------------------------------------------------------ routing
    def route_lane(self, nm, virt, virt_vias=None):
        """Route one lane tooth -> stub: ONE connect() search inside its
        band by default. LANE_PIECES=1 routes it as a chain of searches
        between its planned waypoints instead (the end of its join leg,
        the start of its exit leg), each short and local -- measured on
        the K ladder it routed nothing the single search did not and
        lost K15's SRAS, so it stays an experiment. The layer at a
        waypoint is the one the schedule allows there. Returns
        (segments, vias) or None; partial copper is discarded."""
        ctx, sp = self.ctx, self.spine
        nid, _ = ctx.byname[nm]
        # a ribbon swimmer's search window must HOLD its tube: the
        # window is built from the straight lane_xy with `margin`, so
        # at the default 0.6 the SWIM_TUBE band was clipped to half --
        # the A* explored 12k cells and never saw the via diamonds a
        # lane's width away (K11 SDQ15)
        sc = getattr(self, 'sched_cur', None)
        swim = (sc is not None and getattr(sc, 'two_page', False)
                and sc.page.get(nm) is None)
        margin = SWIM_TUBE + 0.4 if swim else 0.6
        # FREE SWIMMERS (#622): route the whole swimmer class the way
        # the last call routes refusals -- no band, wide window --
        # from the start. The BAND was the refusal: band-free search
        # measured 1-2 via paths (K28 SWE 1, SDQ0 2) where the
        # in-band model wove for more or refused, and the virtual
        # lines plus reserved diamonds still protect every lane routed
        # after. Pages stay banded: their rigidity is what makes them
        # cheap. FREE_SWIM=0 restores the tube.
        free_swim = swim and os.environ.get('FREE_SWIM', '1') == '1'
        if free_swim:
            # BRAID_SWIM_MARGIN: the obs-cache A/B accidentally
            # measured that WINDOW SIZE is a completion constraint at
            # K35 -- with corridor-wide windows the swimmer class
            # closed 4 open -> 1 (at +22 vias and 2x time, via the
            # cache's fence move). This decouples the window from the
            # cache: widen just the free swimmers' search margin.
            margin = float(os.environ.get('BRAID_SWIM_MARGIN', '2.0'))
            if getattr(self, '_swim_boost', False):
                margin = max(margin, 6.0)
        pieces = os.environ.get('LANE_PIECES') == '1'
        way = [(self.teeth[nm], ctx.tooth_layer[nm])]
        if pieces and nm in self.join_block:
            s_l, oa, ob = self.legs[nm][0]
            L = 'B.Cu' if (ctx.tooth_layer[nm] == 'B.Cu'
                           and self.allowed(nm, s_l + 0.05, 'B.Cu')) else 'F.Cu'
            if not self.allowed(nm, s_l + 0.05, L):
                L = 'B.Cu' if L == 'F.Cu' else 'F.Cu'
            way.append((sp.xy(s_l, ob), L))
        if pieces and nm in self.exit_block:
            s_l, oa, ob = self.legs[nm][-1]
            L = self.leg_layer.get(nm, ctx.dest_layer[nm])
            if not self.allowed(nm, s_l - 0.05, L):
                L = 'B.Cu' if L == 'F.Cu' else 'F.Cu'
            way.append((sp.xy(s_l, oa), L))
        way.append((self.stubs[nm], ctx.dest_layer[nm]))
        band = None if free_swim else self.band_of(nm)
        segs_all, vias_all = [], []
        added = 0
        for (a, aL), (b, bL) in zip(way, way[1:]):
            res = cn.connect(ctx.pcb, nid, a, aL, b, bL, ctx.cfg, band=band,
                             virtual=virt, margin=margin,
                             window_pts=self.lane_xy[nm],
                             virtual_vias=virt_vias,
                             cache=getattr(self, '_obs_cache', None))
            if res is None:
                # discard the partial copper
                if added:
                    del ctx.pcb.segments[-added:]
                    ctx.pcb.vias = ctx.pcb.vias[:len(ctx.pcb.vias) - len(vias_all)]
                return None
            segs_o, vias_o = res
            ctx.pcb.segments.extend(segs_o)
            ctx.pcb.vias.extend(vias_o)
            added += len(segs_o)
            segs_all.extend(segs_o)
            vias_all.extend(vias_o)
        return segs_all, vias_all

    def plan_columns(self, sched, gaps, lead):
        """The schedule's columns under the braid's gate policy; returns
        (columns, gate). The take-off gate spends columns to save vias
        (a diver crossed mid-flight surfaces and dives again: two
        vias), and columns are what this bench's corridor is short of:
        the swap region between the two escape fields is the same
        4.5 mm at every K, and a column narrower than W_GATE has no
        room for a via between two passes. Gated while the columns fit,
        ungated when they do not (K28: 31 columns at W=0.147 and 9/26
        lanes gated, 16 at 0.277 and 20/26 ungated; K21 goes from 14
        columns and attempt 3 to 11 and attempt 0 at the same 28 vias;
        K4..K19 stay gated, where ungated cost +2..+6 vias). The gated
        form is 'last' (the strict gate deadlocked on a mutual crossing
        until the empty-column override: K4..K19 identical, K21 one
        attempt sooner, K28 31 -> 23 columns). SCHED_GATE pins a
        policy. The probe tools go through here too, so they diagnose
        the schedule the braid actually laid."""
        self.sched_cur = sched
        if getattr(sched, 'two_page', False):
            # RIBBON: two pages need no swap columns at all. Every
            # lane runs straight from its launch slot to its target
            # slot; same-page lanes never cross, an inverted pair
            # crosses once, where the lines cross, and a crossing of
            # two lanes on different layers costs neither length nor
            # via. The schedule's remaining job is the pages
            # themselves and the swimmers' priority.
            return [], 'last'
        gate = os.environ.get('SCHED_GATE')
        if gate is None:
            cols = sched.columns(gaps, lead, gate='last')
            _u, need = self.column_layout(cols, sched)
            gate = 'last' if need <= self.L_free - RESERVE else 'off'
        return sched.columns(gaps, lead, gate=gate), gate

    def run(self, plan_only=False):
        ctx, log = self.ctx, self.log
        M = self.members
        log(f'\ncorridor {self.idx} ({len(M)}): {M}')
        self.build_spine()
        self.classify()
        log(f'  s0={self.s0:.2f} s1={self.s1:.2f} (spine {self.spine.L:.2f} mm)'
            f'  head-on launches {len(self.heads_l)}, joiners {len(self.joiners)}'
            f'; head-on exits {len(self.heads_e)}, side exits {len(self.siders)}')
        if self.joiners:
            log('  joiners: ' + ', '.join(
                f'{nm}@s{self.st[nm][0]:.1f}{"+" if self.join_side[nm] > 0 else "-"}'
                for nm in sorted(self.joiners, key=lambda n: self.st[n][0])))
        if self.siders:
            log('  side exits: ' + ', '.join(
                f'{nm}@s{self.se[nm][0]:.1f}{"+" if self.exit_side[nm] > 0 else "-"}'
                for nm in sorted(self.siders, key=lambda n: self.se[n][0])))
        if self.s1 - self.s0 < 0.5:
            log(f'  SHORT corridor ({self.s1 - self.s0:.2f} mm): routed as '
                f'tubes, no schedule')
            return self.run_short()
        ly_floor = 0.35
        self.offsets(ly_floor)
        self.reserve_intervals()
        if self.reserved:
            log('  reserved: ' + ', '.join(f'[{a:.2f},{b:.2f}]' for a, b in self.reserved)
                + f'  (free {self.L_free:.2f} of {self.s1 - self.s0:.2f} mm)')
        self._obs_cache = None
        if os.environ.get('BRAID_OBS_CACHE', '0') == '1':
            # corridor-shared obstacle base (see cn.ObsCache): opt-in
            # perf path -- the fence moves to the corridor's edge, so
            # grades must be re-earned before any default flip
            try:
                self._obs_cache = cn.ObsCache(
                    ctx.pcb, {ctx.byname[nm][0] for nm in M},
                    [self.teeth[nm] for nm in M]
                    + [self.stubs[nm] for nm in M]
                    + list(self.spine.pts), ctx.cfg)
                log('  obs-cache: corridor window + base map built')
            except Exception as e:
                log(f'  obs-cache: disabled ({e})')
                self._obs_cache = None
        sched = Schedule(self.launch, self.target, ctx.tooth_layer, log=log,
                         dest_layer=ctx.dest_layer,
                         pages=getattr(ctx, 'pages', None))
        if plan_only:
            # #622 plan dump: the corridor's PLAN (orders + pages) with
            # no copper -- the fanout-contract emitters read it
            self.sched_cur = sched
            return
        gaps = {d: 1 for d in sched.divers}
        lead = {d: 0 for d in sched.divers}
        boost = {}                # ribbon: refused lanes route earlier
        prev_refused = None
        best = None
        for attempt in range(6):
            self.offsets(ly_floor)
            sched = Schedule(self.launch, self.target, ctx.tooth_layer,
                             dest_layer=ctx.dest_layer,
                             pages=getattr(ctx, 'pages', None))
            cols, gate = self.plan_columns(sched, gaps, lead)
            self.lay_lanes(cols)
            swaps = [sw for col in cols for sw in col]
            n_free = sum(1 for col in cols for (d, p) in col if sched.is_free(d, p))
            log(f'  attempt {attempt}: {len(swaps)} swaps ({n_free} free) in '
                f'{len(cols)} columns, need {self.layout_need:.2f} of '
                f'{self.L_free - RESERVE:.2f} mm, W={self.W:.3f}, launch pitch >= '
                f'{ly_floor:.2f}; pages F {len(M) - len(sched.divers)} / B '
                f'{len(sched.b_page)} / swimmers {len(sched.swimmers)}'
                + (f', gate {gate}' if gate != 'last' else '')
                + (f', {len(self.crossings)} lane(s) crossed by exit legs'
                   if self.crossings else '')
                + (f', gaps {dict((d, g) for d, g in gaps.items() if g > 1)}'
                   if any(g > 1 for g in gaps.values()) else '')
                + (f', lead {dict((d, g) for d, g in lead.items() if g)}'
                   if any(lead.values()) else ''))
            if attempt == 0:
                log(f'    {cols}')
            ctx.pcb.segments = list(ctx.base_segments)
            ctx.pcb.vias = list(ctx.base_vias)
            self.out_segs, self.out_vias = {}, {}
            if getattr(self, '_obs_cache', None) is not None:
                self._obs_cache.reset()
            divers = set(sched.divers)
            if sched.two_page:
                # RIBBON order: the rigid lanes first -- the pages, in
                # target order (a page diagonal cannot dodge anything)
                # -- then the swimmers, largest displacement first,
                # each weaving through the REAL copper laid so far. A
                # REFUSED lane is boosted to the front of its class on
                # the next attempt (the greedy that boxed it routes
                # after it instead); the best attempt is kept as ever
                sw_ = [nm for nm in M if sched.page.get(nm) is None]
                ti = {nm: i for i, nm in enumerate(self.target)}
                order = (sorted((nm for nm in self.target if nm not in sw_),
                                key=lambda nm: (-boost.get(nm, 0), ti[nm]))
                         + sorted(sw_, key=lambda nm: (-boost.get(nm, 0), -abs(
                             self.launch_o[nm] - self.target_o[nm]))))
            else:
                order = list(sched.priority) + \
                    [nm for nm in self.target if nm not in divers]
            routed = set()
            self.refused = []
            failed_rescues = 0
            for nm in order:
                unrouted = [om for om in M if om != nm and om not in routed]
                res = self.route_lane(nm, self.virtual_of(unrouted),
                                      self.virtual_vias_of(unrouted))
                if res is None and sched.two_page and failed_rescues < 3:
                    # BUDGET ESCALATION (two-page only). A refused lane
                    # whose plan is feasible usually died at the SEARCH
                    # budget, not at a wall: K19 SODT1's band flood
                    # reaches its stub through open corridor, its own
                    # class routes at 42..49k full iterations, and its
                    # forward frontier dies at 12.7k. Retry once with
                    # the budget quadrupled -- only on refusal, so the
                    # fast path pays nothing.
                    import copy as _copy
                    cfg0 = ctx.cfg
                    big = _copy.copy(cfg0)
                    big.max_iterations = 4 * max(cfg0.max_iterations, 50_000)
                    ctx.cfg = big
                    # the retry also WIDENS a free swimmer's window:
                    # the sm ladder measured window size as the
                    # completion constraint (K35 margin 6.0: 4 open ->
                    # 1, +22 vias) -- pay the wide search only on
                    # refusal, so the cheap path stays cheap
                    self._swim_boost = True
                    try:
                        res = self.route_lane(nm, self.virtual_of(unrouted),
                                              self.virtual_vias_of(unrouted))
                    finally:
                        ctx.cfg = cfg0
                        self._swim_boost = False
                    if res is not None:
                        log(f'    rescued at x4 budget: {nm}')
                    else:
                        # a board whose refusals are real walls (K32:
                        # 14 refused, 0 rescued) must not grind 4x on
                        # every one -- after three failed rescues the
                        # attempt stops escalating
                        failed_rescues += 1
                if res is None:
                    self.refused.append(nm)
                    log(f'    refused: {nm}')
                    if os.environ.get('LANE_DEBUG') == nm:
                        self.debug_lane(nm)
                    continue
                routed.add(nm)
                segs_o, vias_o = res
                self.out_segs[nm] = segs_o
                self.out_vias[nm] = vias_o
                if getattr(self, '_obs_cache', None) is not None:
                    self._obs_cache.note(ctx.byname[nm][0], segs_o, vias_o)
            nv = sum(len(v) for v in self.out_vias.values())
            log(f'    lanes: {len(routed)}/{len(M)} routed, {nv} via(s)')
            # keep the BEST attempt, not the last: the feedback is a
            # guess, and a later attempt can lose lanes an earlier one
            # had (the 45-degree bench: 9/11 at attempt 1, 7/11 at the
            # last, and 7/11 was what got written)
            if best is None or (len(self.refused), nv) < (len(best[0]), best[1]):
                best = (list(self.refused), nv, ly_floor, cols,
                        dict(self.out_segs), dict(self.out_vias),
                        list(ctx.pcb.segments), list(ctx.pcb.vias), sched)
            if not self.refused:
                break
            if sched.two_page:
                # attempts converge fast on the ribbon: once the
                # refused set repeats with the launch pitch maxed and
                # the boost already applied, later attempts are
                # identical -- and each one re-runs tens of thousands
                # of exhausted A* iterations per refusal (K28: 378k
                # per attempt)
                if attempt >= 2 and self.refused == prev_refused \
                        and ly_floor >= 0.40 - 1e-9:
                    log('    attempts converged; stopping early')
                    break
                prev_refused = list(self.refused)
                for nm in self.refused:
                    boost[nm] = boost.get(nm, 0) + 1
            # FEEDBACK. Room across first -- the launch pitch is the
            # cheap dimension, every lane gets it, and a via beside a
            # neighbour is what usually fails. Then room along for a
            # refused diver's dive via: before its first swap if nobody
            # passes it first (a lead column), else after the pass (a
            # spacer column).
            if ly_floor < 0.40 - 1e-9:
                ly_floor = min(0.40, ly_floor + 0.03)
                continue
            for nm in self.refused:
                if nm in sched.swimmers:
                    # a pass within two columns of one of its own moves
                    # (either side) leaves no room for the layer change:
                    # a spacer. Else, nothing passing it before its
                    # first swap: its dive via must sit on the launch
                    # leg -- a lead column.
                    moves = [k for k, col in enumerate(self.all_cols)
                             if any(m == nm for (m, _p) in col)]
                    passes = [k for k, col in enumerate(self.all_cols)
                              if any(p == nm for (_m, p) in col)]
                    tight = any(abs(k - j) <= 2 for k in moves for j in passes)
                    if tight or (passes and moves and min(passes) < min(moves)):
                        gaps[nm] += 1
                    else:
                        lead[nm] += 1
        if best is not None and best[0] != self.refused:
            refused, nv, ly_b, cols_b, segs_b, vias_b, pcb_s, pcb_v, sched = best
            log(f'  keeping attempt with {len(M) - len(refused)}/{len(M)} '
                f'routed, {nv} via(s)')
            self.offsets(ly_b)
            self.sched_cur = sched
            self.lay_lanes(cols_b, sched)
            self.refused = refused
            self.out_segs, self.out_vias = segs_b, vias_b
            ctx.pcb.segments, ctx.pcb.vias = pcb_s, pcb_v
        if self.refused and getattr(sched, 'two_page', False) \
                and os.environ.get('FREE_PASS', '1') == '1':
            # LAST CALL -- the corridor's own router, off the lattice.
            # Every routed lane is real copper by now, and what
            # refused a ribbon swimmer was its BAND -- the tube around
            # a straight line through a saturated lattice -- not the
            # corridor itself. Each refused lane gets one more search:
            # the same engine, costs and virtual copper (for the other
            # refused lanes), but no band, a wide window round its own
            # planned path, and the x4 budget. The copper comes out
            # lane-shaped because it is the lane router; the
            # generalist fallback (TAIL_RESCUE) stays a separate,
            # env-gated thing. FREE_PASS=0 disables. Two-page only, so
            # the single-page default is bit-identical by construction.
            import copy as _copy
            cfg0 = ctx.cfg
            big = _copy.copy(cfg0)
            big.max_iterations = 4 * max(cfg0.max_iterations, 50_000)
            ctx.cfg = big
            try:
                still = list(self.refused)
                for nm in list(still):
                    others = [om for om in still if om != nm]
                    nid, _ = ctx.byname[nm]
                    res = None
                    # margin escalation: a refused lane whose whole
                    # BAND is walled (wc SA0: corridor 2's band under
                    # corridor 1's copper, 5-via detour at 2.0) may
                    # have a short path just outside the first window
                    for mg in (2.0, 4.0, 6.0):
                        res = cn.connect(
                            ctx.pcb, nid, self.teeth[nm],
                            ctx.tooth_layer[nm],
                            self.stubs[nm], ctx.dest_layer[nm], ctx.cfg,
                            band=None, margin=mg,
                            virtual=self.virtual_of(others),
                            window_pts=self.lane_xy[nm],
                            virtual_vias=self.virtual_vias_of(others))
                        if res is not None:
                            break
                    if res is None and os.environ.get('RIP_CALL', '1') == '1':
                        # RIP-ASSISTED LAST CALL: the final refusals
                        # fail because ONE earlier lane took their
                        # corridor. Try the routed lanes nearest the
                        # refused chord as single rip victims; keep
                        # only a rip where BOTH nets re-route.
                        res = self._rip_assist(nm, others, log)
                    if res is None:
                        log(f'    last call: {nm} still refused')
                        continue
                    segs_o, vias_o = res
                    ctx.pcb.segments.extend(segs_o)
                    ctx.pcb.vias.extend(vias_o)
                    self.out_segs[nm] = segs_o
                    self.out_vias[nm] = vias_o
                    still.remove(nm)
                    log(f'    last call routed: {nm} ({len(vias_o)} via(s))')
                self.refused = still
            finally:
                ctx.cfg = cfg0
        if os.environ.get('ECON_RELAY',
                          '1' if getattr(sched, 'two_page', False)
                          else '0') == '1':
            # ECONOMY RE-LAY (#622 K28 flank): the human trades LENGTH
            # for vias -- the 4-via flank-joiners (SA8/SBA1/SDQ0/SRAS
            # at K28) ride constant-B around the field in the human
            # original at 2 vias each, +24% copper. Fanout-side tooth
            # moves measured a dead end (0831 e1/e2: SDQ0 4->2 but ANY
            # source perturbation flips SA4 open). So the trade is
            # taken HERE, post-hoc, where it cannot break completion:
            # rip one heavy lane, re-route it band-free against every
            # other lane's REAL copper (rip-assist's bookkeeping),
            # keep only a strictly-cheaper lane, restore exactly
            # otherwise. Widest rung drops the window so the flank
            # detour is inside the search. Default OFF.
            import copy as _copy
            cfg0 = ctx.cfg
            big2 = _copy.copy(cfg0)
            big2.max_iterations = 4 * max(cfg0.max_iterations, 50_000)
            ctx.cfg = big2
            try:
                heavy = sorted((nm for nm in self.members
                                if len(self.out_vias.get(nm) or ()) >= 3),
                               key=lambda nm: -len(self.out_vias[nm]))
                for nm in heavy:
                    nid, _ = ctx.byname[nm]
                    ids_s = {id(s) for s in self.out_segs[nm]}
                    ids_v = {id(v) for v in self.out_vias[nm]}
                    seg0 = list(ctx.pcb.segments)
                    via0 = list(ctx.pcb.vias)
                    ctx.pcb.segments = [s for s in seg0
                                        if id(s) not in ids_s]
                    ctx.pcb.vias = [v for v in via0
                                    if id(v) not in ids_v]
                    res = None
                    for mg, wp in ((2.5, self.lane_xy[nm]),
                                   (4.0, self.lane_xy[nm]),
                                   (6.0, None)):
                        r_ = cn.connect(ctx.pcb, nid, self.teeth[nm],
                                        ctx.tooth_layer[nm],
                                        self.stubs[nm],
                                        ctx.dest_layer[nm], ctx.cfg,
                                        band=None, margin=mg,
                                        window_pts=wp)
                        if r_ is not None \
                                and len(r_[1]) < len(self.out_vias[nm]):
                            res = r_
                            break
                    if res is None:
                        ctx.pcb.segments = seg0
                        ctx.pcb.vias = via0
                        continue
                    segs_o, vias_o = res
                    ctx.pcb.segments.extend(segs_o)
                    ctx.pcb.vias.extend(vias_o)
                    log(f'    econ re-lay: {nm} '
                        f'{len(self.out_vias[nm])} -> {len(vias_o)} '
                        'via(s)')
                    self.out_segs[nm], self.out_vias[nm] = segs_o, vias_o
            finally:
                ctx.cfg = cfg0
        self.sched = sched
        self.finish()

    def _rip_assist(self, nm, others, log):
        """One-victim rip for a lane the last call still refuses: the
        routed lanes nearest the refused net's own chord, tried one at
        a time -- pull the victim's copper, route the refused net wide
        (the last call's own search, ctx.cfg is already the x4
        budget), re-route the victim the same way; kept only when
        BOTH land, restored exactly otherwise. The braid-native form
        of what route.py's rip-up wins the same fights with. Returns
        the refused net's (segs, vias) NOT yet on the board (the
        caller's bookkeeping adds them), or None. RIP_CALL=0
        disables; it can only fire where something already refused,
        so a complete board is untouched by construction."""
        ctx = self.ctx
        a_, b_ = self.teeth[nm], self.stubs[nm]
        nid, _ = ctx.byname[nm]

        def chord_d(om):
            segs = self.out_segs.get(om) or []
            if not segs:
                return 1e9
            return min(ts.seg_seg_dist((s.start_x, s.start_y),
                                       (s.end_x, s.end_y), a_, b_)
                       for s in segs)
        cands = sorted((om for om in self.members
                        if om != nm and self.out_segs.get(om)),
                       key=chord_d)[:3]
        virt = self.virtual_of(others)
        vv = self.virtual_vias_of(others)
        for om in cands:
            ids_s = {id(s) for s in self.out_segs[om]}
            ids_v = {id(v) for v in self.out_vias[om]}
            seg0 = list(ctx.pcb.segments)
            via0 = list(ctx.pcb.vias)
            ctx.pcb.segments = [s for s in seg0 if id(s) not in ids_s]
            ctx.pcb.vias = [v for v in via0 if id(v) not in ids_v]
            r1 = cn.connect(ctx.pcb, nid, a_, ctx.tooth_layer[nm],
                            b_, ctx.dest_layer[nm], ctx.cfg,
                            band=None, margin=2.5, virtual=virt,
                            window_pts=self.lane_xy[nm],
                            virtual_vias=vv)
            if r1 is not None:
                s1_, v1_ = r1
                ctx.pcb.segments.extend(s1_)
                ctx.pcb.vias.extend(v1_)
                oid, _ = ctx.byname[om]
                r2 = cn.connect(ctx.pcb, oid, self.teeth[om],
                                ctx.tooth_layer[om], self.stubs[om],
                                ctx.dest_layer[om], ctx.cfg,
                                band=None, margin=2.5, virtual=virt,
                                window_pts=self.lane_xy[om],
                                virtual_vias=vv)
                if r2 is not None:
                    s2_, v2_ = r2
                    ctx.pcb.segments.extend(s2_)
                    ctx.pcb.vias.extend(v2_)
                    self.out_segs[om], self.out_vias[om] = s2_, v2_
                    # hand the refused net's copper back through the
                    # caller's bookkeeping: remove the tentative add
                    id1s = {id(x) for x in s1_}
                    id1v = {id(x) for x in v1_}
                    ctx.pcb.segments = [s for s in ctx.pcb.segments
                                        if id(s) not in id1s]
                    ctx.pcb.vias = [v for v in ctx.pcb.vias
                                    if id(v) not in id1v]
                    log(f'    rip-assist: ripped {om}; {nm} routed '
                        f'({len(v1_)} via), {om} re-laid '
                        f'({len(v2_)} via)')
                    return s1_, v1_
            ctx.pcb.segments = seg0
            ctx.pcb.vias = via0
        log(f'    rip-assist: no viable victim for {nm} '
            f'(tried {[c for c in cands]})')
        return None

    def run_free(self):
        """A small CORNER corridor: no frame, no schedule -- the
        machinery that knotted spines and routed the K32 corner group
        0/5. Each member routes the way the last call routes refusals:
        the real router, band-free, a wide window round its own chord,
        margin and budget escalated, against every earlier corridor's
        REAL copper; the unrouted members' chords ride along as
        virtual copper (both layers) so the first lane cannot wall the
        rest, and the widest retry drops even that and lets the
        obstacle map adjudicate. Farthest member first (the most
        constrained window)."""
        ctx, log = self.ctx, self.log
        M = self.members
        log(f'\ncorridor {self.idx} ({len(M)}): {M}  FREE corner route')
        self.build_spine()
        self.lane_xy = {nm: [self.teeth[nm], self.stubs[nm]] for nm in M}
        self.req_xy = {nm: [] for nm in M}
        self.marks = []
        self.out_segs, self.out_vias = {}, {}
        self.refused = []
        import copy as _copy
        big = _copy.copy(ctx.cfg)
        big.max_iterations = 4 * max(ctx.cfg.max_iterations, 50_000)
        routed = set()
        for nm in sorted(M, key=lambda n: -ts.d2(self.teeth[n],
                                                 self.stubs[n])):
            nid, _ = ctx.byname[nm]
            virt = [(self.teeth[om], self.stubs[om], L)
                    for om in M if om != nm and om not in routed
                    for L in ('F.Cu', 'B.Cu')]
            res = None
            for cfg_, vv, mg in ((ctx.cfg, virt, 2.0), (big, virt, 2.0),
                                 (big, None, 4.0), (big, None, 6.0)):
                res = cn.connect(ctx.pcb, nid, self.teeth[nm],
                                 ctx.tooth_layer[nm], self.stubs[nm],
                                 ctx.dest_layer[nm], cfg_,
                                 band=None, margin=mg, virtual=vv)
                if res is not None:
                    break
            if res is None:
                self.refused.append(nm)
                self.out_segs[nm] = []
                self.out_vias[nm] = []
                log(f'    refused: {nm}')
                continue
            segs_o, vias_o = res
            ctx.pcb.segments.extend(segs_o)
            ctx.pcb.vias.extend(vias_o)
            self.out_segs[nm] = segs_o
            self.out_vias[nm] = vias_o
            routed.add(nm)
            log(f'    free-routed: {nm} ({len(vias_o)} via(s))')
        log(f'    lanes: {len(routed)}/{len(M)} routed')
        if self.refused:
            log(f'  REFUSED lanes (left open): {sorted(self.refused)}')
        ctx.base_segments = list(ctx.pcb.segments)
        ctx.base_vias = list(ctx.pcb.vias)
        ctx.laid.extend(self.lane_xy[nm] for nm in M)

    def run_short(self):
        """A corridor too short for a schedule: every lane a straight
        (s, o) line inside its own tube."""
        ctx, sp = self.ctx, self.spine
        self.req, self.bwin = {}, {nm: [(-1e9, 1e9)] for nm in self.members}
        self.cross_iv, self.legs = [], {nm: [] for nm in self.members}
        self.jogs = {nm: [] for nm in self.members}
        self.mid = {nm: [self.st[nm], self.se[nm]] for nm in self.members}
        self.mid_xy = {nm: [((self.st[nm][0] + self.se[nm][0]) / 2,
                             sp.lane_xy(self.mid[nm]))] for nm in self.members}
        self.lane_xy = {nm: [self.teeth[nm]] + self.mid_xy[nm][0][1]
                        + [self.stubs[nm]] for nm in self.members}
        self.sched = None
        routed = set()
        for nm in sorted(self.members, key=lambda n: self.st[n][1]):
            nid, _ = ctx.byname[nm]
            virt = self.virtual_of([om for om in self.members
                                    if om != nm and om not in routed])
            res = cn.connect(ctx.pcb, nid, self.teeth[nm], ctx.tooth_layer[nm],
                             self.stubs[nm], ctx.dest_layer[nm], ctx.cfg,
                             virtual=virt, margin=0.6,
                             window_pts=self.lane_xy[nm])
            if res is None:
                self.refused.append(nm)
                self.log(f'    refused: {nm}')
                continue
            routed.add(nm)
            self.out_segs[nm], self.out_vias[nm] = res
            ctx.pcb.segments.extend(res[0])
            ctx.pcb.vias.extend(res[1])
        self.finish()

    def finish(self):
        ctx = self.ctx
        for nm in self.refused:
            self.out_segs[nm] = []
            self.out_vias[nm] = []
        if self.refused:
            self.log(f'  REFUSED lanes (left open): {sorted(self.refused)}')
        # the corridor's copper is the base for the next one
        ctx.base_segments = list(ctx.pcb.segments)
        ctx.base_vias = list(ctx.pcb.vias)
        ctx.laid.extend(self.lane_xy[nm] for nm in self.members)
        # Eco: required-B stretches on the centreline, '+' marks
        sp = self.spine
        for nm in self.members:
            self.req_xy[nm] = []
            for (a, b, L) in self.req.get(nm, ()):
                if L != 'B.Cu':
                    continue
                ms = [p[0] for p in self.mid[nm]]
                mo = [p[1] for p in self.mid[nm]]
                sub = [(a, float(np.interp(a, ms, mo)))]
                sub += [p for p in self.mid[nm] if a < p[0] < b]
                sub.append((b, float(np.interp(b, ms, mo))))
                self.req_xy[nm].append(sp.lane_xy(sub))
            for (s_l, oa, ob) in self.legs.get(nm, ()):
                self.marks.append(sp.xy(s_l, ob))
            for (p_, q_) in self.jogs.get(nm, ()):
                self.marks.append(sp.xy(*q_))
        for (_i, s_c, _t) in sp.corners():
            self.marks.append(sp.xy(s_c, 0.0))

    def debug_lane(self, nm):
        log = self.log
        log(f'    tooth {self.teeth[nm]} (s,o)=({self.st[nm][0]:.2f},'
            f'{self.st[nm][1]:.2f}) {self.ctx.tooth_layer[nm]}  stub '
            f'{self.stubs[nm]} (s,o)=({self.se[nm][0]:.2f},{self.se[nm][1]:.2f}) '
            f'{self.ctx.dest_layer[nm]}  s0={self.s0:.2f} s1={self.s1:.2f}')
        log(f'    mid: {[(round(s, 2), round(o, 3)) for s, o in self.mid[nm]]}')
        log(f'    legs: {self.legs[nm]}')
        log(f'    req: {[(round(a, 2), round(b, 2), L) for a, b, L in self.req[nm]]}')
        log(f'    bwin: {self.bwin[nm]}   cross: {self.cross_iv}')
        log(f'    launch o {self.launch_o[nm]:.3f} target o {self.target_o[nm]:.3f}'
            f'  neighbours at launch: '
            + str([(om, round(self.launch_o[om], 2)) for om in self.launch
                   if abs(self.launch_o[om] - self.launch_o[nm]) < 0.8 and om != nm]))


class Ctx:
    pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--board', required=True,
                    help='board with BOTH arrays fanned out')
    ap.add_argument('--nets', required=True, help='comma-separated names')
    ap.add_argument('--out', required=True, help='output stem')
    ap.add_argument('--dest', required=True, metavar='REF',
                    help='destination component (its stub ends are '
                         'the targets)')
    ap.add_argument('--no-smooth', action='store_true',
                    help='skip the repo #536 octolinear smoothing pass')
    ap.add_argument('--cluster', type=float, default=6.0,
                    help='stub-end linkage distance (mm) for corridor '
                         'detection (the reach check does the real work)')
    ap.add_argument('--plan-json', default=None,
                    help='dump the plan (orders/pages per net) and stop')
    a = ap.parse_args()
    names = [n.strip() for n in a.nets.split(',') if n.strip()]

    def log(msg=''):
        print(msg)
    ctx, groups = setup(a.board, names, a.dest, log, cluster=a.cluster)
    if a.plan_json:
        # #622: dump the braid's PLAN (per net: corridor, launch index,
        # target rank, page layer / swimmer + birth layer, tooth) and
        # stop -- no routing, no board. Every corridor takes the run()
        # prefix (spine, offsets, first Schedule); free-corner
        # corridors too, since only their EXECUTION is free.
        import json as _json
        plan = {}
        for ci, members in enumerate(groups):
            c = Corridor(ci, members, ctx, log)
            try:
                c.run(plan_only=True)
                sched = c.sched_cur
            except Exception as e:
                log(f'  plan-only corridor {ci} failed ({e}); '
                    'orders only')
                sched = None
            for nm in members:
                d = {'corridor': ci,
                     'tooth': list(ctx.ends[nm][0]),
                     'tooth_layer': ctx.tooth_layer[nm],
                     'dest_layer': ctx.dest_layer[nm]}
                if sched is not None:
                    d['launch_idx'] = sched.lidx[nm]
                    d['target_rank'] = sched.trank[nm]
                    if nm in getattr(sched, 'swimmers', ()):
                        d['page'] = None
                        d['birth'] = sched.tl.get(nm, 'F.Cu')
                    elif nm in getattr(sched, 'b_page', ()):
                        d['page'] = 'B.Cu'
                    else:
                        d['page'] = 'F.Cu'
                plan[nm] = d
        with open(a.plan_json, 'w') as f:
            _json.dump(plan, f, indent=1, sort_keys=True)
        log(f'wrote plan: {a.plan_json}')
        return 0
    corridors = []
    for ci, members in enumerate(groups):
        c = Corridor(ci, members, ctx, log)
        if (2 <= len(members) <= 5
                and os.environ.get('TWO_PAGE') == '1'
                and os.environ.get('FREE_CORNER', '1') == '1'):
            # small CORNER corridors skip the spine/schedule machinery
            # entirely -- it is exactly where that machinery fails
            # (K32's 5-net corner group routed 0/5 for a month; the
            # 3-net K35 group knotted its spine). Single-net corridors
            # keep run_short (recorded K28's SA0 baseline), big ones
            # keep the full braid. FREE_CORNER=0 restores.
            c.run_free()
        else:
            c.run()
        corridors.append(c)
    return write_out(a, ctx, corridors, names, log)


def setup(board, names, dest, log, cluster=6.0):
    """Everything the corridors are built from: the board, the ends,
    the static obstacles, the flow directions, the corridor groups."""
    pcb = parse_kicad_pcb(board)
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    kids = {byname[nm][0] for nm in names}
    ends = endpoints(pcb, names, byname, dest_ref=dest)
    ctx = Ctx()
    ctx.pcb, ctx.byname, ctx.ends, ctx.kids = pcb, byname, ends, kids
    ctx.names = list(names)
    ctx.tooth_layer = {nm: _layer_at(pcb, byname[nm][0], ends[nm][0], 'F.Cu')
                       for nm in names}
    ctx.dest_layer = {nm: _layer_at(pcb, byname[nm][0], ends[nm][1], 'F.Cu')
                      for nm in names}
    ctx.src_ref = {}
    for nm in names:
        nid, net = byname[nm]
        ctx.src_ref[nm] = min(net.pads, key=lambda p: ts.d2(
            (p.global_x, p.global_y), ends[nm][0])).component_ref

    _cache = {}

    def obs_for(nm, layer):
        k = (nm, layer, 'kids')
        if k not in _cache:
            _cache[k] = build_obstacles(pcb, byname[nm][0], kids, layer)
        return _cache[k]

    def obs_but(nm, members, layer):
        """Static copper on `layer` excluding this corridor's members'
        own copper (their teeth fan out from each other; every other
        net's stub is in the way)."""
        k = (nm, layer, tuple(sorted(members)))
        if k not in _cache:
            _cache[k] = build_obstacles(pcb, byname[nm][0],
                                        {byname[m][0] for m in members},
                                        layer)
        return _cache[k]
    ctx.obs_for, ctx.obs_but = obs_for, obs_but
    # the direction each free end ESCAPES in, read from the stub's own
    # copper (its run), whatever angle the array sits at
    ctx.tooth_dir = {nm: _end_dir(pcb, byname[nm][0], ends[nm][0],
                                  byname[nm][1].pads) for nm in names}
    ctx.stub_dir = {nm: _end_dir(pcb, byname[nm][0], ends[nm][1],
                                 byname[nm][1].pads) for nm in names}
    # the PLAN's page assignment, written beside the fanout board by
    # the two-page chain: with it the braid's Schedule uses the pages
    # the escapes were laid FOR, instead of re-deriving them from its
    # own orders and disagreeing (loaded here so the probe tools see
    # the same board the same way)
    ctx.pages = None
    _pages_path = os.path.splitext(board)[0] + '.pages.json'
    if os.path.exists(_pages_path):
        import json
        with open(_pages_path, encoding='utf-8') as _f:
            ctx.pages = json.load(_f)
        log(f'pages sidecar: {_pages_path} '
            f'({sum(1 for v in ctx.pages.values() if v)} paged)')

    # ---- corridors: taut paths, grouped by where they arrive and
    # whether one spine can reach them all
    log('taut paths...')
    ctx.paths = db.taut_paths(names, ends, lambda nm: obs_for(nm, 'F.Cu'))
    pad_obs = array_pad_obstacles(pcb, set(ctx.src_ref.values())
                                  | {ends[nm][2] for nm in names})
    ctx.pad_obs = pad_obs
    ctx.spine_obs = build_obstacles(pcb, -1, kids, 'F.Cu')

    def spine_of(members, extra=None, log=None, H=None, relax=True,
                 base_obs=None):
        if H is None:
            H = LPITCH * (len(members) - 1) / 2 + LPITCH
        return cr.build_spine(
            [ctx.paths[nm] for nm in members],
            ctx.spine_obs if base_obs is None else base_obs, H, extra=extra,
            log=log, teeth=[ends[nm][0] for nm in members],
            stubs=[ends[nm][1] for nm in members],
            tooth_dirs=[ctx.tooth_dir[nm] for nm in members],
            stub_dirs=[ctx.stub_dir[nm] for nm in members], relax=relax)
    ctx.spine_of = spine_of
    def centre_of(ref):
        ps = pcb.footprints[ref].pads
        return (sum(p.global_x for p in ps) / len(ps),
                sum(p.global_y for p in ps) / len(ps))
    groups = cr.cluster_corridors(
        names, ctx.paths, {nm: ends[nm][0] for nm in names},
        {nm: ends[nm][1] for nm in names}, pad_obs.seg_clear, D=cluster,
        log=log, spine_fn=lambda core: spine_of(core, relax=False),
        dest_ref={nm: ends[nm][2] for nm in names},
        centres={nm: centre_of(ends[nm][2]) for nm in names},
        src_centres={nm: centre_of(ctx.src_ref[nm]) for nm in names})
    log(f'{len(groups)} corridor(s): ' + '  '.join(
        f'[{len(g)}] {",".join(g)}' for g in groups))
    # 0.025 grid: the fanout packs stub ends at 0.25, which is the legal
    # minimum (track + clearance = 0.227) plus 23 um. On a 0.05 grid the
    # cell nearest a lane's centreline can sit 25 um off it -- outside
    # the 23 um that clear the neighbours.
    # BRAID_VIA_COST: re-price the length-for-vias trade. MEASURED A
    # LOSER as a search knob (2026-08-31): at 2x and 4x the default
    # (150/300 vs 75) the ladder is unchanged K4..K21 and K28 goes 50
    # -> 54 vias, identical at both values -- the braid's vias are
    # STRUCTURAL (page dive/surface, reserved weaves, legs), so a
    # costlier via only buys worse detours that force more of them.
    # The human's +24% length-for-vias trade lives at the PLAN level
    # (which nets ride which layer end to end), not in the A*. Kept
    # as a knob for study; unset = default, control re-verified 50/0
    # exact.
    _vc = os.environ.get('BRAID_VIA_COST')
    ctx.cfg = cn.make_config(pcb, TRACK, CLEAR, VIA_SIZE, VIA_DRILL,
                             grid_step=0.025,
                             **({'via_cost': float(_vc)} if _vc else {}))
    ctx.base_segments = list(pcb.segments)
    ctx.base_vias = list(pcb.vias)
    ctx.laid = []
    return ctx, groups


def write_out(a, ctx, corridors, names, log):
    """Smooth, write the board with the Eco overlay, report."""
    pcb, byname, ends, kids = ctx.pcb, ctx.byname, ctx.ends, ctx.kids
    out_segs = {nm: c.out_segs[nm] for c in corridors for nm in c.members}
    out_vias = {nm: c.out_vias[nm] for c in corridors for nm in c.members}
    refused = sorted(nm for c in corridors for nm in c.refused)
    if refused:
        log(f'\nREFUSED nets (left open): {refused}')

    # ---- repo octolinear smoothing (#536): collapse the distributed 45
    # nudges into single elbows, clearance-validated against ALL copper.
    # Only the BRAID's copper is a candidate (keep_input_copper): the
    # fanout's stubs are the braid's input and stay as they came, so the
    # tooth a lane was routed from stays on copper -- the smoother once
    # re-cut a stub's corner into a diagonal and left the tooth mark
    # (and the join the plan drew) hanging in free space.
    smoothed = False
    final_segs = {}
    if not a.no_smooth:
        from pcb_modification import smooth_octolinear_chains
        pre_len = {nm: sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
                           for s in out_segs[nm]) for nm in names}
        _n, _nets, _rm, _addl, stt = smooth_octolinear_chains(
            [{'new_segments': list(out_segs[nm])} for nm in names],
            pcb, kids, clearance=0.1, keep_input_copper=True)
        for nm in names:
            nid, _ = byname[nm]
            final_segs[nm] = [s for s in pcb.segments if s.net_id == nid]
        post_len = {nm: sum(math.hypot(s.end_x - s.start_x,
                                       s.end_y - s.start_y)
                            for s in final_segs[nm]) for nm in names}
        log(f'\nsmooth_octolinear_chains (#536): '
            f'{stt.get("spans", 0)} spans on {_nets} nets, '
            f'-{stt.get("saved_mm", 0):.2f} mm; segments '
            f'{sum(len(s) for s in out_segs.values())} -> '
            f'{sum(len(s) for s in final_segs.values())}; length '
            f'{sum(pre_len.values()):.2f} -> '
            f'{sum(post_len.values()):.2f} mm')
        smoothed = True

    # ---- write board
    txt = open(a.board, encoding='utf-8').read()
    add = []
    if smoothed:
        kid_names = {pcb.nets[i].name for i in kids if i in pcb.nets}
        txt = strip_net_segments(txt, kids, kid_names)
        n_deg = n_dup = 0
        for nm in names:
            keep_ = []
            seen = set()
            for s in final_segs[nm]:
                if math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) < 0.001:
                    n_deg += 1
                    continue
                k = (round(s.start_x, 4), round(s.start_y, 4),
                     round(s.end_x, 4), round(s.end_y, 4), s.layer)
                kr = (k[2], k[3], k[0], k[1], k[4])
                if k in seen or kr in seen:
                    n_dup += 1
                    continue
                seen.add(k)
                keep_.append(s)
            final_segs[nm] = keep_
        if n_deg or n_dup:
            log(f'dropped {n_deg} degenerate (< 1 um) and {n_dup} '
                f'duplicate segment(s)')
        emit = {nm: [((s.start_x, s.start_y), (s.end_x, s.end_y), s.layer,
                      s.width) for s in final_segs[nm]] for nm in names}
    else:
        emit = {nm: [((s.start_x, s.start_y), (s.end_x, s.end_y), s.layer,
                      s.width) for s in out_segs[nm]] for nm in names}
    for nm in names:
        nid, _ = byname[nm]
        for (p, q, layer, w) in emit[nm]:
            add.append(f'  (segment (start {p[0]:.4f} {p[1]:.4f}) '
                       f'(end {q[0]:.4f} {q[1]:.4f}) (width {w}) '
                       f'(layer "{layer}") (net {nid}))\n')
        for v in out_vias[nm]:
            add.append(f'  (via (at {v.x:.4f} {v.y:.4f}) (size {VIA_SIZE}) '
                       f'(drill {VIA_DRILL}) (layers "F.Cu" "B.Cu") '
                       f'(net {nid}))\n')

    # ---- Eco overlay: the PLAN, drawn where the copper is, so a render
    # (render_eco.py) shows plan against copper.
    #   Eco1.User (white)   every lane's planned centreline; the spine
    #                       of each corridor as a thick line;
    #   Cmts.User (orange)  where the schedule REQUIRES the back layer:
    #                       the planned under-passes, on the centreline;
    #   Eco2.User (yellow)  the connection ends: the FANOUT's free ends
    #                       (source teeth, stub ends) as an "x", the
    #                       braid's own points (join / exit leg ends,
    #                       spine corners) as a "+".
    def gl(p, q, layer, w=0.05):
        return (f'  (gr_line (start {p[0]:.4f} {p[1]:.4f}) '
                f'(end {q[0]:.4f} {q[1]:.4f}) '
                f'(stroke (width {w}) (type solid)) (layer "{layer}"))\n')

    def cross(p, layer='Eco2.User', r=0.12):
        return gl((p[0] - r, p[1] - r), (p[0] + r, p[1] + r), layer) + \
            gl((p[0] - r, p[1] + r), (p[0] + r, p[1] - r), layer)

    def plus(p, layer='Eco2.User', r=0.10):
        return gl((p[0] - r, p[1]), (p[0] + r, p[1]), layer) + \
            gl((p[0], p[1] - r), (p[0], p[1] + r), layer)

    n_eco = n_req = n_plus = 0
    for c in corridors:
        for p_, q_ in zip(c.spine.pts, c.spine.pts[1:]):
            add.append(gl(p_, q_, 'Eco1.User', 0.2))
        for nm in c.members:
            pts = c.lane_xy[nm]
            for p_, q_ in zip(pts, pts[1:]):
                add.append(gl(p_, q_, 'Eco1.User'))
                n_eco += 1
            for sub in c.req_xy.get(nm, ()):
                for p_, q_ in zip(sub, sub[1:]):
                    add.append(gl(p_, q_, 'Cmts.User', 0.08))
                n_req += 1
            add.append(cross(ends[nm][0]))
            add.append(cross(ends[nm][1]))
        for p_ in c.marks:
            add.append(plus(p_))
            n_plus += 1
    log(f'eco overlay: {len(corridors)} spines, {n_eco} planned centreline '
        f'segments, {n_req} planned under-passes, {2 * len(names)} fanout '
        f'ends (x), {n_plus} braid points (+)')
    k = txt.rstrip().rfind(')')
    out_board = a.out + '.kicad_pcb'
    with open(out_board, 'w') as f:
        f.write(txt[:k] + ''.join(add) + txt[k:])
    pro = os.path.splitext(a.board)[0] + '.kicad_pro'
    if os.path.exists(pro):
        shutil.copy(pro, a.out + '.kicad_pro')
    nv = sum(len(v) for v in out_vias.values())
    nseg = sum(len(emit[nm]) for nm in names)
    log(f'\nwrote {out_board}: {nseg} segments, {nv} vias'
        + (f' -- {len(refused)} net(s) REFUSED' if refused else ''))
    if refused and os.environ.get('TAIL_RESCUE') == '1':
        # TAIL RESCUE (opt-in, DEFAULT OFF -- deliberately env-gated):
        # hand each refused lane to the production generalist router,
        # ONE net per call, against the finished board. Measured at K28
        # it completes the board (49v/3 open -> 62v/0/0) but the
        # rescued weaves are the generalist's, not the ribbon's --
        # messy copper by this project's standard. The DEFAULT stays
        # the honest refusal: the goal is to solve the swimmers INSIDE
        # the topo model (escapes by page / surgical re-lay), and a
        # default fallback would hide exactly the signal that work
        # needs. One net per call is deliberate: the same-call partner
        # crossing was #817 (fixed), and separate calls also measured
        # cheaper (fewer vias) on the bench.
        import subprocess
        rp = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '..', 'py_router', 'route.py')
        cur = out_board
        for nm in refused:
            full = ctx.byname[nm][1].name
            tmp = a.out + '_tr.kicad_pcb'
            r = subprocess.run(
                [sys.executable, rp, cur, '--output', tmp,
                 '--nets', full, '--clearance', str(CLEAR),
                 '--track-width', str(TRACK)],
                capture_output=True, text=True)
            ok = ('"failed": 0' in r.stdout and os.path.exists(tmp))
            log(f'  tail rescue {nm}: '
                + ('routed' if ok else 'REFUSED (kept as-is)'))
            if ok:
                os.replace(tmp, cur)
                tpro = a.out + '_tr.kicad_pro'
                if os.path.exists(tpro):
                    os.replace(tpro, a.out + '.kicad_pro')
            elif os.path.exists(tmp):
                os.remove(tmp)
    return 1 if refused else 0


if __name__ == '__main__':
    sys.exit(main())

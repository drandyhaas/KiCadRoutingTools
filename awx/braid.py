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
CROSS_TUBE = 1.0               # a lane's freedom through a crossing region


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
            return True

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
        self.launch_o, self.target_o = launch_o, target_o
        self.target = sorted(self.members, key=lambda nm: target_o[nm])
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
    def lay_lanes(self, cols):
        """From a schedule: column positions, required-layer intervals,
        the diver windows, and every lane's (s, o) polyline."""
        sp = self.spine
        M = self.members
        n = len(cols)
        reserve = 0.3
        W = (self.L_free - reserve) / (n + 1)
        self.W, self.cols = W, cols
        self.all_cols = list(cols)
        s_m = [self.s_of_u((k + 0.5) * W) for k in range(n + 1)]
        self.invol, self.first, self.last = {}, {}, {}
        for k, col in enumerate(cols):
            for (d, p) in col:
                self.invol.setdefault(d, []).append(k)
                self.invol.setdefault(p, []).append(k)
                self.first.setdefault(d, k)
                self.last[d] = k
        # REQUIRED layers: at the crossing of column k the mover is on
        # B and the passed net on F, over the part of the column where
        # the two lanes are too close for one layer -- and NOT the whole
        # column: a net passed at k and moving at k+2 must change layer
        # between the two, and a requirement that spans each whole
        # column leaves it no cell to put the via in.
        req = {nm: [] for nm in M}
        hw = 0.4 * W + 0.05
        for k, col in enumerate(cols):
            ua = (k + 1) * W
            a, b = self.s_of_u(ua - hw), self.s_of_u(ua + hw)
            for (d, p) in col:
                req[d].append((a, b, 'B.Cu'))
                req[p].append((a, b, 'F.Cu'))
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
        self.crossings = crossings
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
        u_first = W

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

    def band_of(self, nm):
        """The lane's corridor as a cell mask: between the neighbouring
        lanes present on that layer (never narrower than a grid cell),
        closed where the schedule requires the other layer; the join
        and exit legs as rectangles in (s, o); a loose tube through a
        crossing region."""
        sp = self.spine
        cache = {}

        def band(xs, ys, L):
            key = (float(xs[0]), float(xs[-1]), float(ys[0]), float(ys[-1]),
                   len(xs), len(ys))
            if key not in cache:
                X, Y = np.meshgrid(xs, ys, indexing='ij')
                cache.clear()
                cache[key] = sp.project(X, Y)
            S, O = cache[key]
            ms = np.array([p[0] for p in self.mid[nm]])
            mo = np.array([p[1] for p in self.mid[nm]])
            present = (S >= ms[0] - 1e-9) & (S <= ms[-1] + 1e-9)
            o_nm = np.interp(S, ms, mo)
            okL = self.allowed_vec(nm, S, L)
            prev = np.full(S.shape, -np.inf)
            nxt = np.full(S.shape, np.inf)
            for om in self.members:
                if om == nm:
                    continue
                os_ = np.array([p[0] for p in self.mid[om]])
                oo_ = np.array([p[1] for p in self.mid[om]])
                pres = (S >= os_[0] - 1e-9) & (S <= os_[-1] + 1e-9)
                if not pres.any():
                    continue
                o_m = np.interp(S, os_, oo_)
                m = pres & self.allowed_vec(om, S, L)
                prev = np.where(m & (o_m < o_nm), np.maximum(prev, o_m), prev)
                nxt = np.where(m & (o_m > o_nm), np.minimum(nxt, o_m), nxt)
            lo = np.where(np.isfinite(prev), (prev + o_nm) / 2 + HALF_SEP, -np.inf)
            hi = np.where(np.isfinite(nxt), (nxt + o_nm) / 2 - HALF_SEP, np.inf)
            # never narrower than a grid cell: at the stub ends the
            # lanes are 0.25 apart and the corridor formula gives
            # 0.0115 -- a band that on an unlucky grid alignment holds
            # no cell at all. Clearance to the neighbours' copper is
            # the obstacle map's job.
            lo = np.minimum(lo, o_nm - 0.03)
            hi = np.maximum(hi, o_nm + 0.03)
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
        segs = []
        for om in unrouted:
            # the layers a lane may occupy change at every required-
            # interval boundary, so a polyline piece is split there and
            # each part stamped on its own layers -- deciding per whole
            # piece put an 8 mm exit run on BOTH layers through the one
            # millimetre where another lane's leg crosses it (K15)
            cuts = sorted({v for (xa, xb, _L) in self.req.get(om, ())
                           for v in (xa, xb)}
                          | {v for (lo, hi) in self.bwin.get(om, ())
                             for v in (lo, hi) if abs(v) < 1e8})
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
                    for p_, q_ in zip(xy, xy[1:]):
                        for L in ('F.Cu', 'B.Cu'):
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
        return [sp.xy(self.exit_leg_s[om], self.exit_block[om])
                for om in unrouted if om in self.exit_leg_s]

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
        band = self.band_of(nm)
        segs_all, vias_all = [], []
        added = 0
        for (a, aL), (b, bL) in zip(way, way[1:]):
            res = cn.connect(ctx.pcb, nid, a, aL, b, bL, ctx.cfg, band=band,
                             virtual=virt, margin=0.6,
                             window_pts=self.lane_xy[nm],
                             virtual_vias=virt_vias)
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

    def run(self):
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
        sched = Schedule(self.launch, self.target, ctx.tooth_layer, log=log)
        gaps = {d: 1 for d in sched.divers}
        lead = {d: 0 for d in sched.divers}
        best = None
        for attempt in range(6):
            self.offsets(ly_floor)
            sched = Schedule(self.launch, self.target, ctx.tooth_layer)
            cols = sched.columns(gaps, lead)
            self.lay_lanes(cols)
            swaps = [sw for col in cols for sw in col]
            log(f'  attempt {attempt}: {len(swaps)} swaps in {len(cols)} columns, '
                f'W={self.W:.3f}, launch pitch >= {ly_floor:.2f}'
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
            divers = set(sched.divers)
            order = list(sched.priority) + \
                [nm for nm in self.target if nm not in divers]
            routed = set()
            self.refused = []
            for nm in order:
                unrouted = [om for om in M if om != nm and om not in routed]
                res = self.route_lane(nm, self.virtual_of(unrouted),
                                      self.virtual_vias_of(unrouted))
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
            nv = sum(len(v) for v in self.out_vias.values())
            log(f'    lanes: {len(routed)}/{len(M)} routed, {nv} via(s)')
            # keep the BEST attempt, not the last: the feedback is a
            # guess, and a later attempt can lose lanes an earlier one
            # had (the 45-degree bench: 9/11 at attempt 1, 7/11 at the
            # last, and 7/11 was what got written)
            if best is None or (len(self.refused), nv) < (len(best[0]), best[1]):
                best = (list(self.refused), nv, ly_floor, cols,
                        dict(self.out_segs), dict(self.out_vias),
                        list(ctx.pcb.segments), list(ctx.pcb.vias))
            if not self.refused:
                break
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
                if nm in divers:
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
            refused, nv, ly_b, cols_b, segs_b, vias_b, pcb_s, pcb_v = best
            log(f'  keeping attempt with {len(M) - len(refused)}/{len(M)} '
                f'routed, {nv} via(s)')
            self.offsets(ly_b)
            self.lay_lanes(cols_b)
            self.refused = refused
            self.out_segs, self.out_vias = segs_b, vias_b
            ctx.pcb.segments, ctx.pcb.vias = pcb_s, pcb_v
        self.sched = sched
        self.finish()

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
    a = ap.parse_args()
    names = [n.strip() for n in a.nets.split(',') if n.strip()]

    def log(msg=''):
        print(msg)
    ctx, groups = setup(a.board, names, a.dest, log, cluster=a.cluster)
    corridors = []
    for ci, members in enumerate(groups):
        c = Corridor(ci, members, ctx, log)
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

    # ---- corridors: taut paths, grouped by where they arrive and
    # whether one spine can reach them all
    log('taut paths...')
    ctx.paths = db.taut_paths(names, ends, lambda nm: obs_for(nm, 'F.Cu'))
    pad_obs = array_pad_obstacles(pcb, set(ctx.src_ref.values())
                                  | {ends[nm][2] for nm in names})
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
        centres={nm: centre_of(ends[nm][2]) for nm in names})
    log(f'{len(groups)} corridor(s): ' + '  '.join(
        f'[{len(g)}] {",".join(g)}' for g in groups))
    # 0.025 grid: the fanout packs stub ends at 0.25, which is the legal
    # minimum (track + clearance = 0.227) plus 23 um. On a 0.05 grid the
    # cell nearest a lane's centreline can sit 25 um off it -- outside
    # the 23 um that clear the neighbours.
    ctx.cfg = cn.make_config(pcb, TRACK, CLEAR, VIA_SIZE, VIA_DRILL,
                             grid_step=0.025)
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
    return 1 if refused else 0


if __name__ == '__main__':
    sys.exit(main())

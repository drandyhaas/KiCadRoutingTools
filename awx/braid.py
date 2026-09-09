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
the lanes' offsets at the two ends of the schedule region. The two-page
schedule (schedule.py) keeps the longest in-order subsequence on the
front layer, the worst crossers of the rest on the back layer, and
routes what is left as SWIMMERS: free searches that take the other
layer from whichever page lane they cross and pay a via at each change.

COPPER. All of it is the real router's (connect.py). Every lane is
routed from its tooth to its stub end inside its band -- the corridor
between the neighbouring lanes present on that layer, in (s, o), with
the other layer closed wherever the schedule requires one -- and the
lanes not yet routed are stamped as virtual copper on the layers they
may occupy, so the router places the dive and surface vias where they
fit and never where a later lane must pass. A refused lane feeds back
to the schedule (launch pitch) and the
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
import time as _time
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
from schedule import Schedule  # noqa: E402
import escape_moves as em  # noqa: E402
from select_moves import pair_chirality  # noqa: E402
from bga_fanout.flip_frame import to_front_frame, other_layer, mirror_axis  # noqa: E402

TRACK = 0.127
CLEAR = 0.105            # 0.1 spec + 5um so hugs don't sit exactly at 0.1
VIA_SIZE = 0.25
VIA_DRILL = 0.15

MINP = 0.38                    # lane pitch floor at the exits
LPITCH = 0.35                  # pitch of a side-join / side-exit block
BLOCK_GAP = 0.45               # a block starts this far beyond what it clears
HALF_SEP = (TRACK + 0.1) / 2   # two lanes at their band edges clear
LEG_W = 0.5                    # half-width in s of a join / exit leg's band
ISLAND_VETO = 100              # an islanded layer is priced out of a leg's
                               # economics (see place_and_decide)
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
WRAP_REACH = 2.5               # how far past a far-face stub its leg may be placed
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
SLOPE_PITCH = True             # slot pitch scaled by the lane's angle to
                               # the spine (see Corridor.offsets); paper-
                               # checked before it is switched on
VIA_NEED = VIA_SIZE / 2 + CLEAR + TRACK / 2 + 0.03   # a via's room to a
                               # neighbouring track centre, plus a cell
PACK_MODE = int(os.environ.get('BRAID_PACK', '0') or 0)  # pack.py at write time (opt-in)
W_GATE = 0.33                  # narrowest swap column the gated schedule
                               # gets: every clean gated K on the bench
                               # had W >= 0.343 (K21, W=0.322, needed
                               # its third attempt); below it a lane
                               # passed in one column and diving in the
                               # next has no cell for its via, so the
                               # gate yields and columns are spent on
                               # vias instead (see Corridor.run)


def cross_reserve(ctx, nm):
    """CROSS-CORRIDOR RESERVATION: the PLANNED
    lanes of every corridor not routed yet, as virtual copper for the
    corridor routing now -- a page lane's whole planned line on its
    page layer, a swimmer's or a frameless corridor's rigid ENDS on
    the end layers (1.5 mm), a corner corridor's taut path likewise.
    Without it an earlier corridor's lanes take whatever a later
    corridor planned through (K35: two lanes of the 32-net corridor on
    the tooth of the 3-net corner corridor's SA6)."""
    out = []
    for c in getattr(ctx, 'corridors', ()):
        if c.idx in getattr(ctx, 'corr_done', ()):
            continue
        # the corridor routing NOW stamps its own unrouted lanes through
        # virtual_of, which follows the plan's layer rules (a B-page lane
        # born on F is F only until its B requirement starts, a
        # swimmer's mid-line is no promise). Stamping its lanes' ends
        # here as well, 1.5 mm layer-blind on the END layers, walled the
        # very lanes those rules free: K28 SDQ9's F birth stamp ran to
        # s0+1.2 across SDQ10 (SDQ9 required on B from s0+0.3), SDQ11's
        # across SDQ8, the swimmer SA4's diagonal across SDQ7, SDQ15's
        # against SDQM0 -- four of the six lanes refused in-band, each
        # re-laid at last call (wall_probe census, 2026-09-06).
        if nm in c.members:
            continue
        sc = getattr(c, 'sched_cur', None)
        for om in c.members:
            if om == nm or om in ctx.landed:
                continue
            poly = (getattr(c, 'lane_xy', {}) or {}).get(om)
            if not poly or len(poly) < 2:
                poly = ctx.paths.get(om) or [ctx.ends[om][0], ctx.ends[om][1]]
            # ENDS ONLY: corridors overlap in space (the second's spine
            # runs beside the first's), so a whole planned line of a
            # later corridor stamped through the earlier one starved it
            # (K28 0 -> 2 open, K41 5 -> 14). The landing is the
            # reservation; the run is negotiated.
            end_len = 1.5
            for pts, lay in ((poly, ctx.tooth_layer[om]),
                             (list(reversed(poly)), ctx.dest_layer[om])):
                acc = 0.0
                for p, q in zip(pts, pts[1:]):
                    d = math.hypot(q[0] - p[0], q[1] - p[1])
                    if acc >= end_len:
                        break
                    if acc + d > end_len and d > 1e-9:
                        t = (end_len - acc) / d
                        q = (p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t)
                    out.append((p, q, lay))
                    acc += d
    # a reservation must not cover another net's FREE END: the lane it
    # predicts will dodge that copper when it is routed, but stamped
    # over a tooth it seals the tooth's owner in before it starts (K35
    # SA5: corridor 1's SA9 head over SA5's tooth, a one-cell pocket)
    ends = [ctx.ends[om][k] for om in ctx.ends for k in (0, 1)]
    return clip_round_ends(out, ends)


END_KEEP = TRACK + CLEAR + 0.05    # a virtual stamp keeps this off a free end


def clip_round_ends(pieces, ends, keep_r=END_KEEP):
    """`pieces` [(p, q, layer)] with the stretch within keep_r of any
    of `ends` cut out. A virtual stamp must not cover another net's
    FREE END: the lane it predicts will dodge that copper when it is
    routed, but stamped over a tooth or a stub end it seals the end's
    owner in before it starts (K35 SA5: corridor 1's SA9 head over
    SA5's tooth, a one-cell pocket). Applied to the corridor's OWN
    lanes' stamps it freed K35's SA5/SA6 (exit legs of one stub row
    each landing a foot 0.05 mm from the next stub end) but cost K41
    three more open nets (2026-09-06), so it stays a reservation rule."""
    clipped = []
    for (p, q, lay) in pieces:
        bx0, bx1 = min(p[0], q[0]) - keep_r, max(p[0], q[0]) + keep_r
        by0, by1 = min(p[1], q[1]) - keep_r, max(p[1], q[1]) + keep_r
        near = [e for e in ends if bx0 <= e[0] <= bx1 and by0 <= e[1] <= by1]
        parts = [(p, q)]
        for e in near:
            nxt = []
            for (a_, b_) in parts:
                dx, dy = b_[0] - a_[0], b_[1] - a_[1]
                L2 = dx * dx + dy * dy
                if L2 < 1e-12:
                    continue
                t = max(0.0, min(1.0, ((e[0] - a_[0]) * dx + (e[1] - a_[1]) * dy) / L2))
                cx, cy = a_[0] + t * dx, a_[1] + t * dy
                if math.hypot(cx - e[0], cy - e[1]) >= keep_r:
                    nxt.append((a_, b_))
                    continue
                # cut the stretch within keep_r (along the piece) of the end
                Lp = math.sqrt(L2)
                t0, t1 = max(0.0, t - keep_r / Lp), min(1.0, t + keep_r / Lp)
                if t0 > 1e-6:
                    nxt.append((a_, (a_[0] + t0 * dx, a_[1] + t0 * dy)))
                if t1 < 1 - 1e-6:
                    nxt.append(((a_[0] + t1 * dx, a_[1] + t1 * dy), b_))
            parts = nxt
        clipped += [(a_, b_, lay) for (a_, b_) in parts]
    return clipped


def reserve(ctx, nm):
    return cross_reserve(ctx, nm)


_OBS_MEMO = {}
# the boards whose models the memo holds, oldest first: the plan loop
# writes a new board every realized round (src1, src2, ...) and never
# returns to one older than the round it keeps, so the models of every
# board but the last two are dead weight -- 1580 of them, 220 MB, at a
# K28 fanout stage (2026-09-08 trace; README TODO 10). A model is a pure
# function of its key, so an evicted one that is asked for again is
# simply rebuilt.
_OBS_BOARDS = []
_OBS_KEEP_BOARDS = 2


def _obs_board_of(bkey):
    """The board identity inside a memo key: everything but the excluded
    nets and the layer."""
    return bkey[:3] + bkey[5:9]      # a derived key carries the net id after


def _obs_remember(bkey):
    b = _obs_board_of(bkey)
    if b in _OBS_BOARDS:
        return
    _OBS_BOARDS.append(b)
    if len(_OBS_BOARDS) > _OBS_KEEP_BOARDS:
        dead = set(_OBS_BOARDS[:-_OBS_KEEP_BOARDS])
        del _OBS_BOARDS[:-_OBS_KEEP_BOARDS]
        for k in [k for k in _OBS_MEMO if _obs_board_of(k) in dead]:
            del _OBS_MEMO[k]


def unthreadable(fp, track=None, clear=None):
    """True when no lane can pass between two of the part's pads: the
    smallest edge-to-edge gap between any two pads is below a track
    plus two clearances."""
    track = TRACK if track is None else track
    clear = CLEAR if clear is None else clear
    ps = fp.pads
    if len(ps) < 2:
        return False
    X = np.array([p.global_x for p in ps])
    Y = np.array([p.global_y for p in ps])
    R = np.array([max(p.size_x, p.size_y) / 2 for p in ps])
    d = np.hypot(X[:, None] - X[None, :], Y[:, None] - Y[None, :]) - R[:, None] - R[None, :]
    np.fill_diagonal(d, np.inf)
    return float(d.min()) < track + 2 * clear


def build_obstacles(pcb, nid, kids, layer):
    """A static-copper model for one net on one layer: every foreign
    pad as a disc, every foreign segment as a capsule, every foreign
    via as a disc, all inflated by clearance + half a track. The PLAN
    prices its candidate moves against it, the taut paths and the
    spines are relaxed against it; the braid's copper is routed against
    the router's own model and never consults this one. Memoised per
    (board file as on disk, net, excluded nets, layer): the plan loop
    parses the same board several times per round and rebuilt the same
    model each time (531 builds at K15)."""
    # ONE BASE per (board, layer, excluded segment nets), the net's own
    # model DERIVED from it (Obstacles.exclude): the 35 nets of a plan
    # differ only by their own pads and vias, and a full build per net
    # was 111 s of a 250 s K35 fanout stage (2026-09-06 profile). The
    # base excludes the segments of `kids` as before (own included when
    # own is among them); the derivation removes the net's own pads,
    # vias and segments. Same items, same order: bit-identical answers.
    kids = frozenset(kids)
    base_kids = kids if (nid in kids and len(kids) > 1) else kids - {nid}
    src = getattr(pcb, 'source_path', None)
    bkey = dkey = None
    if src and os.path.exists(src):
        st_ = os.stat(src)
        bkey = (os.path.abspath(src), st_.st_mtime_ns, st_.st_size,
                base_kids, layer, len(pcb.segments), len(pcb.vias),
                getattr(pcb, 'frame_axis', None),   # the board turned over, or rotated,
                getattr(pcb, 'frame_rotation', None))   # is another board
        dkey = bkey + (nid,)
        hit = _OBS_MEMO.get(dkey)
        if hit is not None:
            return hit
    base = _OBS_MEMO.get(bkey) if bkey is not None else None
    if base is None:
        base = _build_obstacles(pcb, base_kids, layer)
        if bkey is not None:
            _obs_remember(bkey)
            _OBS_MEMO[bkey] = base
    obs = base.exclude({nid})
    if dkey is not None:
        _OBS_MEMO[dkey] = obs
    return obs


def _build_obstacles(pcb, kids, layer):
    """Every pad on `layer` (drilled: on both), every segment on it
    whose net is not in `kids`, every via -- each tagged with its net,
    so a per-net model is a derivation (build_obstacles)."""
    obs = ts.Obstacles()
    m = CLEAR + TRACK / 2
    for ref, fp in pcb.footprints.items():
        for p in fp.pads:
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
                         f'{ref}.{p.pad_number}', net=p.net_id)
    for s in pcb.segments:
        if s.net_id in kids or s.layer != layer:
            continue
        obs.add_cap((s.start_x, s.start_y), (s.end_x, s.end_y),
                    s.width / 2 + m, f'seg:{s.net_id}', net=s.net_id)
    for v in pcb.vias:
        obs.add_disc(v.x, v.y, v.size / 2 + m, f'via:{v.net_id}',
                     net=v.net_id)
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
        # the far pad is one on ANOTHER component than the stub's own:
        # a part placed right under the source ball (K51 SZQ: its ZQ
        # resistor's pad 0.09 mm from the ball, on the back layer)
        # made the ball itself the farthest pad by 0.1 mm, and the
        # lane looped the net onto its own source
        owner = _owner(src, segs, net.pads)
        far = [p for p in net.pads if p.component_ref != owner] or net.pads
        tgt = max(far, key=lambda p: ts.d2((p.global_x, p.global_y), src))
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
    """Layer of the net's copper at pt. Among ALL segments ending
    there, the LONGEST run wins -- `next()` took the first in file
    order, so a stray zero-ish landing on the other layer could claim
    a tooth (K15 SA7: B stub, resolved F; every lane's first via
    silently bridged the mistake until a 0-via econ re-lay had no via
    to mask with and severed the net)."""
    best = None
    for s in pcb.segments:
        if s.net_id != nid:
            continue
        if not (abs(s.start_x - pt[0]) + abs(s.start_y - pt[1]) < 0.005
                or abs(s.end_x - pt[0]) + abs(s.end_y - pt[1]) < 0.005):
            continue
        ln = math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
        if best is None or ln > best[0]:
            best = (ln, s.layer)
    if best:
        return best[1]
    # no copper ends there: a bare PAD end (the far-pad fallback of
    # endpoints) answers with its own single copper layer
    net = pcb.nets.get(nid)
    for p in sorted(net.pads if net else (),
                    key=lambda q: ts.d2((q.global_x, q.global_y), pt)):
        if abs(p.global_x - pt[0]) <= p.size_x / 2 + 0.02 \
                and abs(p.global_y - pt[1]) <= p.size_y / 2 + 0.02:
            cu = [L for L in p.layers if L.endswith('.Cu') and L != '*.Cu']
            if len(cu) == 1:
                return cu[0]
    return default


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


def bundle_layer_of(tooth_layer):
    """The layer most teeth are born on -- what the taut paths and the
    spine relax against. It used to be F by name; a board turned over
    (every tooth on B) then dodged the wrong layer's copper."""
    n_b = sum(1 for L in tooth_layer.values() if L == 'B.Cu')
    return 'B.Cu' if 2 * n_b > len(tooth_layer) else 'F.Cu'


def _relax_pitch(vals, floor):
    """Push a sorted list of offsets apart to at least `floor` (one
    value, or one per adjacent pair), symmetrically."""
    py = list(vals)
    fl = list(floor) if isinstance(floor, (list, tuple)) else [floor] * max(len(py) - 1, 0)
    for _ in range(60):
        moved = False
        for i in range(len(py) - 1):
            g_ = py[i + 1] - py[i]
            floor = fl[i]
            if g_ < floor - 1e-9:
                push = (floor - g_) / 2
                py[i] -= push
                py[i + 1] += push
                moved = True
        if not moved:
            break
    return py


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
        self.leg_split = {}        # side exiter -> o where its leg changes to the stub's layer
        self.sched_cur = None      # the Schedule the last attempt ran

    # ------------------------------------------------------------ geometry
    def build_spine(self):
        ctx = self.ctx
        n_m = len(self.members)
        self.H = LPITCH * (n_m - 1) / 2 + LPITCH
        # the corridors already laid, each as ONE tube: its spine at a
        # lane pitch's radius, inflated (ramped) by its half-width plus
        # this corridor's, so this spine runs beside it or crosses it
        # transversally -- never snakes between its lanes (per-lane tubes
        # did that: a singleton beside the K15 bundle took five corners)
        extra = [(p, q, LPITCH, H_c + self.H)
                 for (pts, H_c) in getattr(ctx, 'laid_tubes', ())
                 for p, q in zip(pts, pts[1:])]
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
        # ... and only a part the lanes cannot THREAD: a 2.54 mm header
        # with 1.7 mm pads leaves 0.84 mm between pins, and the straight
        # chord with the island logic threads 27 of 28 K28 lanes between
        # them (46 vias) where a spine bent round the header shipped 57;
        # with 2.3 mm pads (0.24 mm gaps) the chord ships 3 open and 12
        # DRC and the bent spine 0 / 0. A part is solid for the spine
        # when some pair of its pads is closer, edge to edge, than a
        # track plus two clearances.
        big = {ref for ref, fp in ctx.pcb.footprints.items()
               if len(fp.pads) >= 10 and ref not in own and unthreadable(fp)}
        obs = ts.Obstacles()
        for (x, y, r, name) in ctx.spine_obs.discs:
            if name.split('.')[0] in big:
                obs.add_disc(x, y, r, name)
        obs.build()
        spine = ctx.spine_of(self.members, extra=extra, log=self.log,
                             H=self.H, base_obs=obs)
        self.spine_core = spine          # unextended: the tube for later corridors
        # extend so every free end projects strictly inside the spine
        P0, d0 = spine.P[0], spine.d[0]
        Pn, dn = spine.P[-1], spine.d[-1]
        back = max([0.3] + [-((t[0] - P0[0]) * d0[0] + (t[1] - P0[1]) * d0[1])
                            + 0.3 for t in teeth.values()])
        fwd = max([0.3] + [((t[0] - Pn[0]) * dn[0] + (t[1] - Pn[1]) * dn[1])
                           + 0.3 for t in stubs.values()])
        # a FAR-FACE stub (past the destination array's last ball along
        # the spine) is reached by a leg placed beyond the array, so the
        # frame must run past that leg: project clamps s at the spine's
        # end, and cells past it would all read as the end
        s_ball = [((p.global_x - Pn[0]) * dn[0] + (p.global_y - Pn[1]) * dn[1])
                  for nm in self.members
                  for p in ctx.pcb.footprints[ctx.ends[nm][2]].pads]
        far = [((t[0] - Pn[0]) * dn[0] + (t[1] - Pn[1]) * dn[1])
               for t in stubs.values()]
        along_spine = any(ctx.stub_dir[nm][0] * dn[0] + ctx.stub_dir[nm][1] * dn[1] > 0.7
                          for nm in self.members)
        if s_ball and (max(far) > max(s_ball) - 1e-6 or along_spine):
            fwd = max(fwd, max(max(far), max(s_ball)) + WRAP_REACH)
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
        self.s0_base = self.s0

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
        # FAR-FACE exits: a side exit whose stub lies beyond the
        # destination array's last ball along the spine. Its leg cannot
        # cross the ball field at the stub's own s; it is placed past the
        # array's end (s_leg_min) and the lane -- the outermost of its
        # block, by the target order -- arrives round the corner and jogs
        # back into the stub. The corridor split rule admits such a stub
        # by the same geometry (corridor.cluster_corridors, wrap_clear).
        self.far_exit = set()
        self.s_leg_min = None
        if self.siders:
            refs = {ctx.ends[nm][2] for nm in self.siders}
            pads = [p for r in refs for p in ctx.pcb.footprints[r].pads]
            if pads:
                sb = [sp.project_pt((p.global_x, p.global_y))[0] for p in pads]
                s_ball = max(sb)
                r_max = max(max(p.size_x, p.size_y) / 2 for p in pads)
                self.far_exit = {nm for nm in self.siders
                                 if se[nm][0] > s_ball - 1e-6}
                # ...and a stub that ESCAPES ALONG THE SPINE (the array's
                # far face, its end just inside the last ball column): a
                # leg in o at the stub's own s runs down the face over
                # the neighbouring stubs of the same column (K41 SA9/
                # SA13/SA7, 0.25 apart in o, 0.1 apart in s: refused
                # in-band in every arm, 2-8 vias each at last call). It
                # is reached the way a far-face stub is: a leg beyond the
                # array and the jog back along the stub's own line.
                dn = sp.d[-1]
                self.far_exit |= {nm for nm in self.siders
                                  if ctx.stub_dir[nm][0] * dn[0]
                                  + ctx.stub_dir[nm][1] * dn[1] > 0.7}
                if self.far_exit:
                    self.s_leg_min = s_ball + r_max + CLEAR + TRACK / 2 + 0.05
                    self.log(f'  far-face exits: {sorted(self.far_exit)} '
                             f'(legs at s >= {self.s_leg_min:.2f}, last ball '
                             f'at s {s_ball:.2f})')

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
            if all(ok.values()):
                break
            base += sg * step
        if abs(base - base0) > 1e-9:
            self.log(f'  block on side {"+" if sg > 0 else "-"} pushed '
                     f'{abs(base - base0):.2f} mm clear of static copper '
                     f'(s {s_from:.1f}..{s_to:.1f})')
        return base

    def _leg_s(self, nm, s_l, oa, ob, at_tooth, placed, avoid=None, extra=()):
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
        ends_L = []          # ...those on the jog's own layer
        for om in self.members:
            if om == nm:
                continue
            for p, L in ((self.st[om], self.ctx.tooth_layer[om]),
                         (self.se[om], self.ctx.dest_layer[om])):
                if (L != own_L and abs(p[0] - own[0]) < 0.05
                        and abs(p[1] - own[1]) < 0.05):
                    continue
                ends.append(p)
                if L == own_L:
                    ends_L.append(p)
        lo_, hi_ = min(oa, ob), max(oa, ob)

        # a foreign FREE END is in a leg's way when the leg would run
        # closer than the legal minimum (track + clearance, as for a
        # leg already placed) -- not a whole lane pitch. Teeth along a
        # face sit a half-pitch apart (0.3-0.4 mm at K41), so at LPITCH
        # every candidate clashed and the tie-break parked SA9's leg
        # 0.05 mm from SA5's tooth, stamped on both layers: SA5 refused
        # at its first cell. The recorded chain's rule is a pitch; the
        # legal minimum was measured alone on the ladder as K35 82 -> 97
        # vias and K41 4 -> 5 open, so a pitch it stays.
        end_clash = LPITCH - 1e-6

        def clash(s):
            # a jog is a whole pitch, so an end or a leg exactly a pitch
            # away is clear -- compared with a tolerance, or the float
            # residue of 2.6 - 0.35 vs 1.6 + 0.35 reads as a clash and
            # sends the leg two pitches off, over the next tooth
            n = sum(1 for p in ends
                    if abs(p[0] - s) < end_clash
                    and lo_ - 0.05 < p[1] < hi_ + 0.05)
            n += sum(1 for (ps, plo, phi) in placed
                     if abs(ps - s) < TRACK + 0.1 + 0.02 and plo < hi_ and phi > lo_)
            return n
        def bad(s):
            return avoid is not None and avoid(nm, s)

        def jogged(s):
            # the jog from a moved leg back to its free end runs along
            # that end's own row, and may not run over another member's
            # free end there: a leg pushed 3.3 mm past a passive cluster
            # (K41 SCKE1) jogged along the stub row on F straight over
            # two neighbours' stub ends, and both were refused at the
            # stub; the K35 cascade began with a 0.35 jog that ended
            # 0.05 mm short of the next stub.
            # A jog runs on the free end's own layer (virtual_of), so
            # only the ends on THAT layer are in its way: a K41 join
            # jog on F refused the pitch over a B tooth and took the
            # other side, onto SBA0's F tooth instead.
            lo_s, hi_s = min(s, own[0]) - 0.1, max(s, own[0]) + 0.1
            return any(lo_s < p[0] < hi_s and abs(p[1] - own[1]) < LEG_O
                       for p in ends_L)
        def room(s):
            # the leg's room: its distance to the nearest foreign free
            # end in its span or leg already placed
            d = [abs(p[0] - s) for p in ends if lo_ - 0.05 < p[1] < hi_ + 0.05]
            d += [abs(ps - s) for (ps, plo, phi) in placed if plo < hi_ and phi > lo_]
            return min(d) if d else 1e9

        def too_close(s):
            # ...and a leg with less than the legal minimum of it is
            # not a candidate at all: the min-clash rule took the first
            # of four equally clashing candidates and put SA8's join
            # leg 0.007 mm from SA5's tooth (K41), a stamp on both
            # layers over the tooth -- SA5 refused at its first cell
            # every attempt
            return room(s) < TRACK + CLEAR + 0.02
        if not clash(s_l) and not bad(s_l):
            return s_l
        best = None
        cands = (s_l + LPITCH, s_l - LPITCH, s_l + 2 * LPITCH, s_l - 2 * LPITCH)
        # ...and, off an island, the first s past either of its edges
        cands = cands + tuple(sorted(extra, key=lambda v: abs(v - s_l)))
        cands = tuple(c for c in cands if not jogged(c) and not too_close(c))
        for cand in cands:
            if bad(cand):
                continue
            n = clash(cand)
            if not n:
                return cand
            # fewer clashes first, then the most room
            if best is None or (n, -room(cand)) < (best[0], -room(best[1])):
                best = (n, cand)
        if best is None:
            # every candidate on an island too: the plain rule
            for cand in cands:
                n = clash(cand)
                if best is None or (n, -room(cand)) < (best[0], -room(best[1])):
                    best = (n, cand)
        # no legal candidate: the leg stays where it is
        return best[1] if best is not None else s_l

    def pair_floor(self, a, b, base, sched, at_launch):
        """The offset pitch two adjacent slots need. Clearance is
        perpendicular to a lane, and a slot pitch is measured across
        the spine, so a pair of same-page lanes crossing the region at
        an angle needs the base pitch times the secant of the steeper
        one's angle (K28: at 45 degrees a 0.35 pitch is 0.25 of copper
        room, two hundredths over the legal minimum). Between lanes on
        different pages only a via at the slot needs room -- the lane
        born (or landing) on the other layer changes layer there -- at
        a via's clearance, scaled the same way. A swimmer's line is no
        promise: base."""
        if sched is None or not SLOPE_PITCH:
            return base
        pa, pb = sched.page.get(a), sched.page.get(b)
        if pa is None or pb is None:
            return base
        L = max(self.s1 - self.s0, 1e-6)
        m = max(abs(self._slope.get(a, 0.0)), abs(self._slope.get(b, 0.0)))
        sec = math.sqrt(1.0 + m * m)
        if pa == pb:
            return base * sec
        tl, dl = self.ctx.tooth_layer, self.ctx.dest_layer
        via = any((tl if at_launch else dl)[nm] != sched.page[nm] for nm in (a, b))
        return max(base, VIA_NEED * sec) if via else base

    def offsets(self, ly_floor, sched=None):
        """Launch and target offsets for the current pitch floor. With
        a schedule (a second pass) every adjacent pitch is the pair's
        own floor (pair_floor); the orders do not change between the
        passes, so the schedule stands."""
        st, se = self.st, self.se
        if sched is not None and hasattr(self, 'launch_o'):
            L = max(self.s1 - self.s0, 1e-6)
            self._slope = {nm: (self.target_o[nm] - self.launch_o[nm]) / L
                           for nm in self.members}
        else:
            self._slope = {}
        # head-on launches: tooth offsets at the launch pitch floor
        # (pushed one way only, as the trunk always did)
        hl = sorted(self.heads_l, key=lambda nm: st[nm][1])
        if sched is not None and SLOPE_PITCH:
            # second pass: the pair's own floors, relaxed SYMMETRICALLY
            # (a one-way push piles every widening onto the last lanes
            # of the comb: K28 SDQ0 +0.39, its fan-in through the stub
            # beside it), and the fan-in made long enough for the
            # largest shift -- a slot pushed further across than the
            # fan-in is long is a diagonal steeper than 45 degrees
            # through the neighbouring teeth
            Ly = _relax_pitch([st[nm][1] for nm in hl],
                              [self.pair_floor(hl[i], hl[i + 1], ly_floor, sched, True)
                               for i in range(len(hl) - 1)] if hl else ly_floor)
            shift = max((abs(Ly[i] - st[nm][1]) for i, nm in enumerate(hl)), default=0.0)
            self.s0 = max(self.s0_base, max(st[nm][0] for nm in hl) + shift) if hl else self.s0_base
            if abs(self.s0 - self.s0_base) > 1e-9:
                self.log(f'  fan-in extended {self.s0 - self.s0_base:.2f} mm for a '
                         f'{shift:.2f} mm launch shift (s0 {self.s0:.2f})')
        else:
            self.s0 = self.s0_base
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
        py = _relax_pitch([se[nm][1] for nm in he],
                          [self.pair_floor(he[i], he[i + 1], MINP, sched, False)
                           for i in range(len(he) - 1)] if he else MINP)
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
            acc = 0.0
            for k, nm in enumerate(order):
                if k:
                    acc += self.pair_floor(order[k - 1], nm, LPITCH, sched, False)
                target_o[nm] = base + sg * acc
                self.exit_block[nm] = target_o[nm]
        self.launch_o, self.target_o = launch_o, target_o
        self.target = sorted(self.members, key=lambda nm: target_o[nm])
        self.launch = sorted(self.members, key=lambda nm: launch_o[nm])
        self.Ly = [launch_o[nm] for nm in self.launch]
        self.py = [target_o[nm] for nm in self.target]

    def reserve_intervals(self):
        """The free length of the corridor along its spine: the whole
        of it (a straight spine has no corner wedges, and a corridor
        laid earlier that this one runs through would reserve the
        overlap -- see cross_reserve for the planned-lane form)."""
        self.reserved = []
        self.S_pts = np.array([self.s0, self.s1])
        self.U_pts = np.array([0.0, self.s1 - self.s0])
        self.L_free = float(self.s1 - self.s0)

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

    def lay_lanes(self, sched=None):
        """From a schedule: column positions, required-layer intervals,
        the diver windows, and every lane's (s, o) polyline. `sched`
        (default: the one the last attempt ran) gives the pages."""
        sp = self.spine
        M = self.members
        sched = sched or self.sched_cur
        L_avail = self.L_free - RESERVE
        # the layout is stretched (or squeezed) to the free length, as
        # the uniform layout was: with one page every column is a
        # constrained one at W_GATE and this is exactly (k + 1) W
        self.W = L_avail
        self.layout_need = 0.0
        req = {nm: [] for nm in M}
        trank = {nm: i for i, nm in enumerate(self.target)}
        py = self.py
        islands_ = self.static_islands()

        def leg_on_island(nm, s_, L):
            o_l, o_e = py[trank[nm]], self.se[nm][1]
            lo_, hi_ = min(o_l, o_e), max(o_l, o_e)
            return any(bx[0] <= s_ <= bx[1] and bx[2] < hi_ and lo_ < bx[3]
                       for bx in islands_.get(L, ()))

        def island_edges(nm, s_, L):
            o_l, o_e = py[trank[nm]], self.se[nm][1]
            lo_, hi_ = min(o_l, o_e), max(o_l, o_e)
            out = []
            for bx in islands_.get(L, ()):
                if bx[0] <= s_ <= bx[1] and bx[2] < hi_ and lo_ < bx[3]:
                    out += [bx[0] - 0.05, bx[1] + 0.05]
            return out

        def leg_split_at(nm, s_, L):
            """A leg on L islanded only at its STUB end, whose stub is
            on the other layer (so the leg owes a via there anyway): the
            o where the leg changes to the stub's layer, just past the
            island with a via's room, else None. The via moves a few
            tenths along the leg instead of the leg moving a pitch
            along the stub row: K35's SA12, a B leg to an F stub whose
            last 0.1 mm lay on C6..C9's inflated box under the bottom
            ball row, was moved onto SA1's stub end, and every leg of
            that row then jogged a pitch onto the next stub (stub ends
            0.4 apart leave no legal foreign foot) -- SA1/SA5/SA6
            refused at the stub every attempt. The stub-layer run must
            be island-free and cross no lane, or the leg's pricing is
            void; then the s move stands."""
            Ld = self.ctx.dest_layer[nm]
            if Ld == L:
                return None
            o_l, o_e = py[trank[nm]], self.se[nm][1]
            lo_, hi_ = min(o_l, o_e), max(o_l, o_e)
            n_ = hi_ - lo_
            sg = 1.0 if o_l > o_e else -1.0

            def t_span(bx):
                # the island's reach along the leg, measured from the stub end
                a_, b_ = max(bx[2], lo_), min(bx[3], hi_)
                return ((a_ - o_e, b_ - o_e) if sg > 0 else (o_e - b_, o_e - a_))
            t_split = None
            for bx in islands_.get(L, ()):
                if bx[0] <= s_ <= bx[1] and bx[2] < hi_ and lo_ < bx[3]:
                    _ta, tb = t_span(bx)
                    t_split = max(t_split or 0.0, tb + VIA_NEED)
            if t_split is None or t_split > n_ - VIA_NEED:
                return None
            for bx in islands_.get(Ld, ()):
                if bx[0] <= s_ <= bx[1] and bx[2] < hi_ and lo_ < bx[3]:
                    ta, _tb = t_span(bx)
                    if ta < t_split + VIA_NEED:
                        return None
            o_sp = o_e + sg * t_split
            for om in M:
                if om == nm:
                    continue
                if om in self.exit_block:
                    o_m, s_end = self.exit_block[om], self.exit_leg_s[om]
                else:
                    o_m, s_end = self.target_o[om], self.se[om][0]
                if s_end > s_ + 0.05 and min(o_e, o_sp) < o_m < max(o_e, o_sp):
                    return None
            return o_sp


        def place_and_decide(avoid=None, pre=None):
            """Exit legs placed (each a pitch off another leg or a
            free end in its way, and -- on the second pass -- off any
            static island on its layer), the lanes each leg crosses,
            and every leg's layer decided along s."""
            self.exit_leg_s = {}
            placed = []
            placed_by = {}
            def _order(n):
                if n in self.far_exit:
                    return (1, abs(self.exit_block[n]), self.se[n][0])
                return (0, self.se[n][0], abs(self.exit_block[n]))
            for nm in sorted(self.exit_block, key=_order):
                s_e, o_e = self.se[nm]
                o_l = py[trank[nm]]
                s_base, avoid_nm = s_e, avoid
                if nm in self.far_exit:
                    s_base = max(s_e, self.s_leg_min)
                    floor = self.s_leg_min

                    def avoid_nm(n, s, _a=avoid, _f=floor):
                        return s < _f - 1e-9 or (_a is not None and _a(n, s))
                s_l = self._leg_s(nm, s_base, o_l, o_e, False, placed, avoid_nm,
                                  extra_cands.get(nm, ()) if avoid else ())
                self.exit_leg_s[nm] = s_l
                placed.append((s_l, min(o_l, o_e), max(o_l, o_e)))
                placed_by[nm] = placed[-1]
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
            # TWO-PAGE LEG ECONOMICS, decided ALONG s. A leg crossing a
            # lane on the OTHER layer is free -- the block-wide layer rule
            # priced every crossing as a forced dive (2 vias per crossed
            # lane), which is exactly the SA7-class 4-via overspend (t7
            # K28: the human pays 2). Each leg picks the layer that
            # minimises what is actually paid: a dive for every crossed
            # lane that is on that layer THERE (its return charged only if
            # its berth is on that layer too), a corner via where the leg
            # differs from the layer its own lane is on there, a via where
            # it differs from the stub's. "There" is the point: a lane is
            # crossed only by legs EARLIER than its own (it ends at its
            # leg), so with the legs decided in ascending s every stretch
            # the earlier legs imposed -- on this lane and on the lanes it
            # crosses -- is known when a leg chooses. Judged by pages alone
            # (2026-09-06) K28's SA9 was sent under two F legs, back up to F
            # for its own leg and down again into its B berth: three
            # changes where the router found one, and the plan counted
            # zero. A crossed page lane dives only under a SAME-layer leg;
            # a swimmer adapts per leg. Overlapping opposite-layer
            # intervals from adjacent disagreeing legs are dropped in pairs
            # (the K19 lesson: both layers closed refuses the lane before
            # the router sees it); the obstacle map adjudicates there.
            ivs = {nm: list((pre or {}).get(nm, ())) for nm in M}

            def cur_layer(om, s):
                """The layer the plan has lane `om` on at s: its last
                required stretch starting before s (appended in s order),
                else its page (None for a swimmer)."""
                before = [iv for iv in ivs[om] if iv[0] < s]
                return before[-1][2] if before else (sched.page.get(om) if sched else None)

            def move_cost(nm, s_l, L):
                """What leaving an island on L by a move along the stub
                row costs, in vias: a pitch of jog is worth a via, an
                illegal jog (over a neighbour's free end, or none to be
                had) the full veto. A leg's layer is then the cheaper
                of the flip and the move: SDQ2 (K35) hops 0.3 mm off
                C12 on its own layer for less than a via, SA15 (K41)
                takes B for one where every F move jogs over a stub."""
                o_l, o_e = py[trank[nm]], self.se[nm][1]
                others = [v for k, v in placed_by.items() if k != nm]
                floor = self.s_leg_min if nm in self.far_exit else None
                s_m = self._leg_s(
                    nm, s_l, o_l, o_e, False, others,
                    lambda n, s: leg_on_island(n, s, L)
                    or (floor is not None and s < floor - 1e-9),
                    island_edges(nm, s_l, L))
                if abs(s_m - s_l) < 1e-9 or leg_on_island(nm, s_m, L):
                    return ISLAND_VETO
                return min(ISLAND_VETO, abs(s_m - s_l) / LPITCH)

            for nm in sorted(self.exit_block, key=lambda n: self.exit_leg_s[n]):
                s_l = self.exit_leg_s[nm]
                own = cur_layer(nm, s_l)
                crossed = leg_cross.get(nm, ())
                cost = {}
                for L in ('F.Cu', 'B.Cu'):
                    c = 0
                    for om in crossed:
                        if cur_layer(om, s_l) == L:
                            c += 1 + (1 if self.ctx.dest_layer[om] == L else 0)
                    if own is not None and own != L:
                        c += 1
                    if self.ctx.dest_layer[nm] != L:
                        c += 1
                    if leg_on_island(nm, s_l, L) and leg_split_at(nm, s_l, L) is None:
                        # a static island under the leg on this layer and
                        # no via to take early: the flip to the other
                        # layer competes with the move along the row (K41:
                        # three legs moved off an F-only passive cluster
                        # to one s, 3.3 mm from their stubs, where a B leg
                        # crosses nothing)
                        c += move_cost(nm, s_l, L)
                    cost[L] = c
                Lg = min(('F.Cu', 'B.Cu'), key=lambda L: cost[L])
                self.leg_layer[nm] = Lg
                other = 'B.Cu' if Lg == 'F.Cu' else 'F.Cu'
                a = s_l - LEG_REQ
                for om in crossed:
                    b = s_l + LEG_REQ
                    if om in self.exit_block:
                        b = min(b, self.exit_leg_s[om] - 0.03)
                    else:
                        b = min(b, self.se[om][0] - 0.03)
                    if b <= a:
                        continue
                    # a lane already on the other layer there gets the
                    # stretch all the same (the band closes the leg's layer
                    # under it), at no change
                    ivs[om].append((a, b, other))
            return ivs, leg_req_min

        # EARLY DIVE (#622 K35): a lane whose tail crosses a static
        # island on the layer it is on, and which owes a change to the
        # other layer anyway (its berth is there, or its page already
        # is), takes that change BEFORE the island instead of after it
        # -- no via the plan did not already count, and no bend. The
        # plan looped SRST/SA0/SA15 3 mm round a six-part passive
        # cluster on F at K35 while the router, refused, laid SA0
        # straight under it on B, the layer of its berth.
        pre = {}
        tl_, dl_ = self.ctx.tooth_layer, self.ctx.dest_layer
        for L_ in ('F.Cu', 'B.Cu'):
            other_ = 'B.Cu' if L_ == 'F.Cu' else 'F.Cu'
            for (s_lo, s_hi, o_lo, o_hi, what) in islands_.get(L_, ()):
                if s_lo < self.s1 - 0.1:
                    continue
                for nm in M:
                    pg = sched.page.get(nm) if sched else None
                    if pg is None:
                        continue
                    s_e, o_e = self.se[nm]
                    if s_e <= s_lo + 0.05:
                        continue
                    o_t = py[trank[nm]]
                    # the tail run's offset over the island (a block lane
                    # runs at its slot; a head-on tail slides to its stub)
                    if nm in self.exit_block:
                        o_here = o_t
                    else:
                        t0 = max(0.0, min(1.0, (s_lo - self.s1) / max(s_e - self.s1, 1e-9)))
                        t1 = max(0.0, min(1.0, (s_hi - self.s1) / max(s_e - self.s1, 1e-9)))
                        o_a, o_b = o_t + t0 * (o_e - o_t), o_t + t1 * (o_e - o_t)
                        o_here = (o_a + o_b) / 2
                        if not (min(o_a, o_b) < o_hi and max(o_a, o_b) > o_lo):
                            continue
                    if not (o_lo < o_here < o_hi):
                        continue
                    if pg == other_:
                        a_, b_ = self.s1 + 0.05, min(s_e - 0.05, s_hi + 0.3)
                    elif dl_[nm] == other_:
                        a_, b_ = max(self.s1 + 0.05, s_lo - 0.3), s_e - 0.05
                    else:
                        continue
                    if b_ > a_ and not any(abs(x[0] - a_) < 1e-6 for x in pre.get(nm, ())):
                        pre.setdefault(nm, []).append((a_, b_, other_))
                        self.log(f'  early dive: {nm} on {other_[0]} over {what} (s {a_:.1f}..{b_:.1f})')
        extra_cands = {}
        ivs, leg_req_min = place_and_decide(pre=pre)
        # a leg over a static island on BOTH layers (neither a clear
        # other layer nor a via to take early; K28 SDQ0's leg at s 20.6
        # on F, through C12's second pad) is re-placed a pitch off the
        # island, and the crossings and layers decided again from the
        # moved legs
        layer0 = dict(self.leg_layer)
        bad_legs = [nm for nm in self.exit_block
                    if leg_on_island(nm, self.exit_leg_s[nm], layer0[nm])
                    and leg_split_at(nm, self.exit_leg_s[nm], layer0[nm]) is None]
        extra_cands = {nm: island_edges(nm, self.exit_leg_s[nm], layer0[nm])
                       for nm in bad_legs}
        if bad_legs:
            was = {nm: self.exit_leg_s[nm] for nm in bad_legs}
            ivs, leg_req_min = place_and_decide(
                lambda nm, s_: nm in layer0 and nm in bad_legs
                and leg_on_island(nm, s_, layer0[nm]), pre=pre)
            self.log('  legs off islands: ' + ', '.join(
                f'{nm} s{was[nm]:.1f}->{self.exit_leg_s[nm]:.1f}' for nm in bad_legs))
        self.leg_split = {}
        for nm in self.exit_block:
            Lg = self.leg_layer[nm]
            if leg_on_island(nm, self.exit_leg_s[nm], Lg):
                o_sp = leg_split_at(nm, self.exit_leg_s[nm], Lg)
                if o_sp is not None:
                    self.leg_split[nm] = o_sp
        if self.leg_split:
            self.log('  legs split at an island: ' + ', '.join(
                f'{nm} {self.leg_layer[nm][0]}->{self.ctx.dest_layer[nm][0]} at o{o:+.2f} '
                f'(stub o{self.se[nm][1]:+.2f})' for nm, o in self.leg_split.items()))
        for om, vv in ivs.items():
            kept_iv = [iv for iv in vv
                       if not any(o[2] != iv[2] and iv[0] < o[1]
                                  and o[0] < iv[1]
                                  for o in vv if o is not iv)]
            if kept_iv:
                own_p = sched.page.get(om) if sched else None
                dives = [a for (a, _b, L) in kept_iv if L != own_p]
                if dives:
                    # only a CONFLICTING (dive) stretch cuts the
                    # page req short; a stay-on-page stretch is the
                    # page req continued
                    leg_req_min[om] = min(dives)
                req[om].extend(kept_iv)
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
            # the birth via goes AT the launch slot: the lane may stay
            # on its tooth layer only to just past s0, so the dive
            # lands where the slots are a full pitch apart. Given 0.45
            # of room the router dove at the END of it (a via costs the
            # same anywhere on the stretch, and the forward search
            # leaves the tooth layer only when forced), where two
            # converging neighbours had closed to 0.22 mm -- K28 SDQ14's
            # via sealed SDQM0's channel at attempt 0 (wall_probe,
            # 2026-09-06); the landing via at s1 likewise.
            a = self.s0 + 0.05
            if tlr[nm] != pg and xs_:
                a = min(a, max(self.s0 + 0.02, min(xs_) - HW_COL))
            b = self.s1 - 0.05
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
            if sched.page.get(nm) is None:
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
        py = self.py
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
            # RIBBON: hold the launch offset at the region start,
            # then run STRAIGHT to the target slot at s1
            pts.append((self.s_of_u(0.0), self.launch_o[nm]))
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
        # STATIC ISLANDS (#622 K28 C5): a part sitting inside the
        # corridor is a wall the straight lines ignored, and the page
        # rule then forbids the one cheap escape (the other layer)
        # exactly there -- three F lanes planned through C5's pads
        # were refused in-band every attempt and re-laid at last call,
        # where the third found every slot round the island taken and
        # shipped open (wall_probe census, 2026-09-06). The plan bends
        # the lanes round the island instead: see deflect_islands.
        self.deflect_islands(sched)
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
        two_obs = {L: self.ctx.obs_but(M[0], M, L)
                   for L in ('F.Cu', 'B.Cu')}
        pad_r = (VIA_SIZE - TRACK) / 2

        def line_o(om, s):
            ms_ = self.mid[om]
            if not (ms_[0][0] - 1e-9 <= s <= ms_[-1][0] + 1e-9):
                return None
            return float(np.interp(s, [p[0] for p in ms_],
                                   [p[1] for p in ms_]))

        def line_dist(om, s, o):
            """Distance in the (s, o) plane from a point to lane
            `om`'s polyline, over the pieces within 0.6 mm of s."""
            best = 1e9
            ms_ = self.mid[om]
            for (sa, oa), (sb, ob) in zip(ms_, ms_[1:]):
                if sb < s - 0.6 or sa > s + 0.6:
                    continue
                dx, dy = sb - sa, ob - oa
                L2 = dx * dx + dy * dy
                t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((s - sa) * dx + (o - oa) * dy) / L2))
                best = min(best, math.hypot(s - sa - t * dx, o - oa - t * dy))
            return best

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
                        # a lane-pitch clear of every other lane's LINE
                        # -- the line as a polyline in (s, o), not its
                        # offset at this s alone: a spot 0.30 beside a
                        # flat stretch was 0.13 from the same lane's
                        # 67-degree run-out a tenth of a millimetre on
                        # (K15 SDQ13 off the C5 island, refused in-band)
                        if any(om != nm and line_dist(om, ds, o0 + do) < 0.30
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

    def static_islands(self):
        """Static copper inside the schedule region, per layer, as (s, o)
        boxes a track centre on that layer cannot enter: every pad of a
        footprint that is not one of the run's arrays -- an SMD pad on
        its own layer, a drilled pad on both -- projected on the spine,
        inflated by the clearance plus half a track, kept when it lies
        between s0 and s1 within the corridor's offset span; boxes that
        overlap on one layer are merged."""
        ctx, sp = self.ctx, self.spine
        arrays = ({ctx.src_ref[nm] for nm in self.members}
                  | {ctx.ends[nm][2] for nm in self.members})
        grow = CLEAR + TRACK / 2
        o_all = ([o for (_s, o) in self.st.values()]
                 + [o for (_s, o) in self.se.values()]
                 + list(self.launch_o.values()) + list(self.target_o.values()))
        # the window must hold what a bend can reach, not just the
        # lanes' own offsets: bent off a six-part cluster at K35, SA0
        # landed on R3, 0.3 mm outside a +-1 window and so unseen
        o_lo_c, o_hi_c = min(o_all) - 3.0, max(o_all) + 3.0
        boxes = {'F.Cu': [], 'B.Cu': []}
        for ref, fp in ctx.pcb.footprints.items():
            if ref in arrays:
                continue
            for p in fp.pads:
                drilled = bool(p.drill and p.drill > 0)
                if p.pad_type == 'np_thru_hole':
                    hx = hy = (p.drill or 0.0) / 2
                    layers = ['F.Cu', 'B.Cu']
                else:
                    hx, hy = p.size_x / 2, p.size_y / 2
                    layers = (['F.Cu', 'B.Cu'] if drilled else
                              [L for L in ('F.Cu', 'B.Cu') if L in p.layers])
                if not layers or hx <= 0 or hy <= 0:
                    continue
                so = [sp.project_pt((p.global_x + dx, p.global_y + dy))
                      for dx in (-hx, hx) for dy in (-hy, hy)]
                s_lo = min(v[0] for v in so) - grow
                s_hi = max(v[0] for v in so) + grow
                o_lo = min(v[1] for v in so) - grow
                o_hi = max(v[1] for v in so) + grow
                if s_hi < self.s0 or s_lo > self._s_end():
                    continue
                if o_hi < o_lo_c or o_lo > o_hi_c:
                    continue
                for L in layers:
                    boxes[L].append([s_lo, s_hi, o_lo, o_hi, f'{ref}.{p.pad_number}'])
        out = {}
        for L, bx in boxes.items():
            merged = []
            # boxes that touch or leave less than a track's room between
            # them are one island: a 0402's two pads leave a 0.05 mm
            # strip a track centre could take, and a lane sent into it
            # is then squeezed by the other pad's pass (K28 C5)
            gap = 0.10
            for b in sorted(bx):
                for m in merged:
                    if (b[0] <= m[1] + gap and m[0] <= b[1] + gap
                            and b[2] <= m[3] + gap and m[2] <= b[3] + gap):
                        m[0], m[1] = min(m[0], b[0]), max(m[1], b[1])
                        m[2], m[3] = min(m[2], b[2]), max(m[3], b[3])
                        m[4] = m[4] + '+' + b[4]
                        break
                else:
                    merged.append(list(b))
            out[L] = [tuple(m) for m in merged]
        return out

    def _s_end(self):
        """Where the corridor's lanes end along the spine: the farthest
        stub (a lane's tail runs that far), or a far-face exit's leg."""
        end = max(v[0] for v in self.se.values()) + 0.5
        if getattr(self, 'far_exit', None):
            end = max(end, self.s_leg_min + WRAP_REACH)
        return end

    def _layer_at(self, nm, s):
        """The layer the plan has page lane `nm` on at s (its layer
        profile's last run starting at or before s)."""
        runs = self.layer_profile(nm)
        L = runs[0][1]
        for s_r, L_r in runs:
            if s_r <= s + 1e-9:
                L = L_r
        return L

    def deflect_islands(self, sched):
        """Bend the page lanes round the static islands in the schedule
        region. For each island on a layer, the lanes on that layer
        whose line passes through it go round the side that costs the
        smaller deflection (the island's near corner: a lane entering
        at the north-west and leaving at the south-east of a box is
        nearer its north-east corner or its south-west one) -- nearest
        the island first, at the island's edge, then outward at the
        pitch -- and the lanes already outside are pushed outward only
        where a deflected one would come closer than they were. The
        bend is written into the lane's (s, o) polyline: the island's
        s-range at the new offset, a run-in and a run-out back on the
        line, both inside the schedule region so the launch and target
        slots stand; the bands, the virtual copper and the windows all
        read the polyline, so they follow. The via model is untouched:
        a lane bent on its own layer still changes no layer. Swimmers
        weave and are left alone."""
        if sched is None:
            return
        islands = self.static_islands()
        M = self.members
        other = {'F.Cu': 'B.Cu', 'B.Cu': 'F.Cu'}

        def o_at(nm, s):
            ms_ = self.mid[nm]
            if not (ms_[0][0] - 1e-9 <= s <= ms_[-1][0] + 1e-9):
                return None
            return float(np.interp(s, [q[0] for q in ms_], [q[1] for q in ms_]))

        for L in ('F.Cu', 'B.Cu'):
            for (s_lo, s_hi, o_lo, o_hi, what) in islands.get(L, ()):
                # an island past s1 is a TAIL island: the exit comb's
                # parallel runs and the head-on tails pass it. A block
                # lane bends OUTWARD there (inward is the berth comb and
                # every inner leg), a head-on tail by the smaller shift;
                # the target slot at s1 and the lane's own leg stand.
                in_tail = s_lo >= self.s1 - 0.1
                if in_tail:
                    s_lo = max(s_lo, self.s1 + 0.1)
                else:
                    s_lo = max(s_lo, self.s0 + 0.1)
                    s_hi = min(s_hi, self.s1 - 0.1)
                if s_hi <= s_lo:
                    continue
                s_c = (s_lo + s_hi) / 2
                on_L = []
                for nm in M:
                    if sched.page.get(nm) is None:
                        continue
                    if in_tail:
                        # in the tail a lane's virtual copper is on
                        # every layer the plan allows it (a B-page lane
                        # surfaces somewhere before its F leg), so any
                        # lane allowed on L there is a neighbour the
                        # bend must carry: bent across an unpushed
                        # B-page line, K28 SCKE0 and SBA1 walled each
                        # other
                        if not self.allowed(nm, s_c, L):
                            continue
                    else:
                        if not self.allowed(nm, s_c, L):
                            continue
                        if self.allowed(nm, s_c, other[L]) and sched.page.get(nm) != L:
                            continue
                    o = o_at(nm, s_c)
                    if o is None:
                        continue
                    on_L.append((o, nm))
                def _in(v):
                    return v is not None and o_lo < v < o_hi
                inside = [(o, nm) for (o, nm) in on_L
                          if _in(o) or _in(o_at(nm, s_lo)) or _in(o_at(nm, s_hi))]
                if not inside:
                    continue
                # which side: the smaller of the two corner deflections
                side_of = {}
                for (o, nm) in inside:
                    if in_tail and nm in self.exit_block:
                        side_of[nm] = self.exit_side[nm]
                        continue
                    o_in, o_out = o_at(nm, s_lo), o_at(nm, s_hi)
                    o_in = o if o_in is None else o_in
                    o_out = o if o_out is None else o_out
                    d_n = max(o_in, o_out) - o_lo          # must be <= o_lo throughout
                    d_s = o_hi - min(o_in, o_out)          # must be >= o_hi throughout
                    side_of[nm] = -1 if d_n <= d_s else 1
                # the other islands on this layer across the same s: a
                # slot that lands on one steps past it (K35 SA0, bent
                # off a six-part cluster onto R3 just beyond it)
                others = [bx for bx in islands.get(L, ())
                          if bx[4] != what and bx[0] <= s_hi and s_lo <= bx[1]]

                def off_islands(v, sg):
                    for _ in range(8):
                        hit = [bx for bx in others if bx[2] < v < bx[3]]
                        if not hit:
                            return v
                        v = min(b[2] for b in hit) if sg < 0 else max(b[3] for b in hit)
                    return v
                want = {}
                for sg in (-1, 1):
                    edge = o_lo if sg < 0 else o_hi
                    grp = [(o, nm) for (o, nm) in on_L
                           if (side_of[nm] == sg if nm in side_of
                               else (o <= edge if sg < 0 else o >= edge))]
                    grp.sort(key=lambda t: sg * t[0])
                    prev = None
                    for (o, nm) in grp:
                        if prev is None:
                            lim = edge
                        else:
                            lim = prev[1] + sg * min(MINP, abs(o - prev[0]))
                        new = (min(o, lim) if sg < 0 else max(o, lim))
                        new = off_islands(new, sg)
                        if abs(new - o) > 1e-9:
                            want[nm] = new
                        prev = (o, new)
                if not want:
                    continue
                for nm, new in want.items():
                    ms_ = self.mid[nm]
                    o_in, o_out = o_at(nm, s_lo), o_at(nm, s_hi)
                    if o_in is None or o_out is None:
                        continue
                    d_in = max(0.3, abs(new - o_in))
                    d_out = max(0.3, abs(new - o_out))
                    if in_tail:
                        end = (min(self.exit_leg_s[nm], self.se[nm][0])
                               if nm in self.exit_block else self.se[nm][0])
                        s_a = max(self.s1 + 0.05, s_lo - d_in)
                        s_b = min(end - 0.05, s_hi + d_out)
                    else:
                        s_a = max(self.s0 + 0.05, s_lo - d_in)
                        s_b = min(self.s1 - 0.05, s_hi + d_out)
                    if s_a >= s_lo:
                        continue
                    if s_b <= s_hi and in_tail and nm in self.exit_block \
                            and self.exit_leg_s[nm] > s_hi and s_hi + 0.05 >= s_lo:
                        # no room to return before the leg: the lane
                        # keeps the bent offset to its leg, and the leg
                        # starts there
                        s_l = self.exit_leg_s[nm]
                        o_a = o_at(nm, s_a)
                        keep = [q for q in ms_ if q[0] < s_a - 1e-9 or q[0] > s_l + 1e-9]
                        self.mid[nm] = sorted(
                            keep + [(s_a, o_a), (s_lo, new), (s_l, new)],
                            key=lambda q: q[0])
                        s_leg, _oa, ob = self.legs[nm][-1]
                        self.legs[nm][-1] = (s_leg, new, ob)
                        continue
                    if s_b <= s_hi:
                        continue
                    o_a, o_b = o_at(nm, s_a), o_at(nm, s_b)
                    keep = [q for q in ms_ if q[0] < s_a - 1e-9 or q[0] > s_b + 1e-9]
                    self.mid[nm] = sorted(
                        keep + [(s_a, o_a), (s_lo, new), (s_hi, new), (s_b, o_b)],
                        key=lambda q: q[0])
                self.log(f'  {"tail " if in_tail else ""}island {what} on {L[0]} (s {s_lo:.1f}..{s_hi:.1f}, o '
                         f'{o_lo:+.2f}..{o_hi:+.2f}): '
                         + ', '.join(f'{nm} {want[nm]:+.2f}'
                                     for nm in sorted(want, key=lambda n: want[n]))
                         + f' ({len(inside)} through it: '
                         + ', '.join(f'{nm}{"N" if side_of[nm] < 0 else "S"}'
                                     for _o, nm in sorted(inside)) + ')')

    def layer_profile(self, nm):
        """The layers the plan requires of one lane along s, as runs
        [(s, layer), ...] with equal neighbours merged: the tooth's
        layer, every required stretch in s order (the page over the
        schedule region; the other layer under a same-layer exit leg in
        the tail), the exit leg's layer, the berth's. len - 1 is the
        layer changes -- the vias -- the plan implies for the lane.
        Counting only the corridor's under-passes missed every tail
        dive (K28: 32 predicted, 42 laid; every miss a lane forced to
        the other layer under an exit leg after s1)."""
        ctx = self.ctx
        seq = [(self.st[nm][0], ctx.tooth_layer[nm])]
        seq += [(xa, L) for (xa, _xb, L) in sorted(self.req.get(nm, ()))]
        if nm in self.exit_block and nm in self.leg_layer:
            seq.append((self.exit_leg_s[nm], self.leg_layer[nm]))
        seq.append((self.se[nm][0], ctx.dest_layer[nm]))
        runs = [seq[0]]
        for s, L in seq[1:]:
            if L != runs[-1][1]:
                runs.append((s, L))
        return runs

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

    def band_of(self, nm, slack=0.0, open_layers=False):
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

        # The mask in STRIPS of columns (README TODO 10): the projection
        # of every window cell and the dozen window-sized float64
        # intermediates the formulas below make were 416 MB of one
        # 1.77M-cell attempt, the largest transient of the braid. The
        # sample edges lo1/hi1 are computed ONCE on the whole window's
        # sample set; everything else here is per cell in (S, O) -- an
        # interpolation onto those samples, a comparison, a searchsorted
        # -- and Spine.project is per point (the nearest leg), so a strip
        # at a time is the identical arithmetic over a 25th of the cells.
        STRIP = 64

        def band(xs, ys, L):
            key = (float(xs[0]), float(xs[-1]), float(ys[0]), float(ys[-1]),
                   len(xs), len(ys))
            if key not in cache:
                cache.clear()
                S = np.empty((len(xs), len(ys)))
                O = np.empty((len(xs), len(ys)))
                for i in range(0, len(xs), STRIP):
                    X, Y = np.meshgrid(xs[i:i + STRIP], ys, indexing='ij')
                    S[i:i + STRIP], O[i:i + STRIP] = sp.project(X, Y)
                cache[key] = (S, O, self._band_samples(nm, float(S.min()),
                                                       float(S.max())))
            S, O, sg = cache[key]
            ms = np.array([p[0] for p in self.mid[nm]])
            mo = np.array([p[1] for p in self.mid[nm]])
            sc = getattr(self, 'sched_cur', None)
            swim = sc is not None and sc.page.get(nm) is None
            if not swim:
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
            if sc is not None:
                dms = np.maximum(np.diff(ms), 1e-9)
                sl = np.abs(np.diff(mo)) / dms
            ok = np.empty(S.shape, dtype=bool)
            for i in range(0, S.shape[0], STRIP):
                Ss = S[i:i + STRIP]
                Os = O[i:i + STRIP]
                present = (Ss >= ms[0] - 1e-9) & (Ss <= ms[-1] + 1e-9)
                o_nm = np.interp(Ss, ms, mo)
                okL = (np.ones(Ss.shape, dtype=bool) if open_layers
                       else self.allowed_vec(nm, Ss, L))
                if swim:
                    # a ribbon SWIMMER weaves through the page lattice: the
                    # neighbour-pinch band is wrong for it -- its same-layer
                    # neighbours are the very lanes it crosses, and they
                    # squeezed it to a thin fragmented thread (K11,
                    # refused). A wide tube around its straight line; the
                    # obstacle map and the virtual copper are the law
                    lo = o_nm - SWIM_TUBE
                    hi = o_nm + SWIM_TUBE
                else:
                    lo = np.interp(Ss, sg, lo1)
                    hi = np.interp(Ss, sg, hi1)
                    if sc is not None:
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
                if sc is not None:
                    seg_i = np.clip(np.searchsorted(ms, Ss, side='right') - 1,
                                    0, len(sl) - 1)
                    fl = 0.03 + SLOPE_W * np.minimum(sl[seg_i], 30.0)
                else:
                    fl = 0.03
                lo = np.minimum(lo, o_nm - fl) - slack
                hi = np.maximum(hi, o_nm + fl) + slack
                oks = present & okL & (Os >= lo) & (Os <= hi)
                for (s_l, oa, ob) in self.legs[nm]:
                    rect = ((np.abs(Ss - s_l) <= LEG_W + slack)
                            & (Os >= min(oa, ob) - LEG_O - slack)
                            & (Os <= max(oa, ob) + LEG_O + slack))
                    oks |= rect & okL
                for ((sa, oa), (sb, ob)) in self.jogs.get(nm, ()):
                    rect = ((Ss >= min(sa, sb) - LEG_O) & (Ss <= max(sa, sb) + LEG_O)
                            & (np.abs(Os - oa) <= LEG_O))
                    oks |= rect & okL
                ok[i:i + STRIP] = oks
            return ok
        return band

    def virtual_of(self, unrouted):
        """Centrelines of lanes not routed yet, on every layer the
        schedule lets them occupy, as copper the router must clear --
        except through a crossing region, where the plan is not a
        promise."""
        sp = self.spine
        sc = getattr(self, 'sched_cur', None)
        two = sc is not None
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
                    o_sp = self.leg_split.get(om)
                    if o_sp is not None and min(oa, ob) < o_sp < max(oa, ob):
                        # ...its end past an island on the stub's layer
                        m_ = sp.xy(s_l, o_sp)
                        segs.append((a_, m_, self.leg_layer[om]))
                        segs.append((m_, b_, self.ctx.dest_layer[om]))
                    else:
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
        out = [sp.xy(self.exit_leg_s[om], self.legs[om][-1][1])
               for om in unrouted if om in self.exit_leg_s]
        # ...and the via a split leg takes early, past an island
        out += [sp.xy(self.exit_leg_s[om], self.leg_split[om])
                for om in unrouted if om in self.leg_split and om in self.exit_leg_s]
        # ...plus every unrouted swimmer's RESERVED DIAMONDS (#622
        # reservation pass): the spots its layer changes will need,
        # kept clear of everything routed before it
        for om in unrouted:
            out.extend(getattr(self, 'hops', {}).get(om, ()))
        return out

    # ------------------------------------------------------------ routing
    def route_lane(self, nm, virt, virt_vias=None):
        """Route one lane tooth -> stub: ONE connect() search inside its
        band by default.
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
        swim = sc is not None and sc.page.get(nm) is None
        margin = SWIM_TUBE + 0.4 if swim else 0.6
        # FREE SWIMMERS (#622): route the whole swimmer class the way
        # the last call routes refusals -- no band, wide window --
        # from the start. The BAND was the refusal: band-free search
        # measured 1-2 via paths (K28 SWE 1, SDQ0 2) where the
        # in-band model wove for more or refused, and the virtual
        # lines plus reserved diamonds still protect every lane routed
        # after. Pages stay banded: their rigidity is what makes them
        # cheap.
        free_swim = swim
        if free_swim:
            # window size is a completion constraint (measured K35: a
            # corridor-wide window closed 4 open -> 1 at +22 vias and
            # 2x time); the free swimmers get a 2 mm search margin
            margin = 2.0
            if getattr(self, '_swim_boost', False):
                margin = max(margin, 6.0)
        way = [(self.teeth[nm], ctx.tooth_layer[nm])]
        way.append((self.stubs[nm], ctx.dest_layer[nm]))
        band = None if free_swim else self.band_of(nm)
        segs_all, vias_all = [], []
        added = 0
        # The primary lanes target the stub TIP. Finishing at any
        # dest-stub vertex (earliest same-net contact) was measured as
        # a default: K28 58 -> 54v but K21 +2 and K32 loses completion
        # (early joins reshape later lanes' worlds), so only the
        # post-completion paths (last call, econ re-lay) use b_alts.
        virt = list(virt or []) + reserve(ctx, nm)
        for hop, ((a, aL), (b, bL)) in enumerate(zip(way, way[1:])):
            final = hop == len(way) - 2
            res = cn.connect(ctx.pcb, nid, a, aL, b, bL, ctx.cfg, band=band,
                             virtual=virt, margin=margin,
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
        ly_floor = 0.35
        self.offsets(ly_floor)
        self.reserve_intervals()
        sched = Schedule(self.launch, self.target, ctx.tooth_layer, log=log,
                         dest_layer=ctx.dest_layer)
        if SLOPE_PITCH:
            self.offsets(ly_floor, sched=sched)
            self.reserve_intervals()
            sched = Schedule(self.launch, self.target, ctx.tooth_layer,
                             dest_layer=ctx.dest_layer)
        if plan_only:
            # #622 plan dump: the corridor's PLAN (orders + pages) with
            # no copper -- the fanout-contract emitters read it; the
            # planned lanes too (cross_reserve reads lane_xy)
            self.sched_cur = sched
            try:
                self.sched_cur = sched
                self.lay_lanes()
            except Exception as e:
                self.log(f'  plan-only lanes not laid ({e})')
            return
        boost = {}                # ribbon: refused lanes route earlier
        prev_refused = None

        def plan_at(ly):
            """The corridor's plan at launch pitch `ly`: offsets, the
            schedule, the required intervals and the lanes."""
            self.offsets(ly)
            sc_ = Schedule(self.launch, self.target, ctx.tooth_layer,
                           dest_layer=ctx.dest_layer)
            if SLOPE_PITCH:
                self.offsets(ly, sched=sc_)
                self.reserve_intervals()
                sc_ = Schedule(self.launch, self.target, ctx.tooth_layer,
                               dest_layer=ctx.dest_layer)
            else:
                self.reserve_intervals()
            self.sched_cur = sc_
            self.lay_lanes()
            return sc_
        # THE BEST ATTEMPT IS KEPT, not the last (#622 K35). The
        # feedback between attempts -- a wider launch pitch, refused
        # lanes boosted to the front -- is a heuristic for the refused
        # lanes and a change of world for every other: at K35 attempt
        # 0 routed 27/32 in 33 vias with every swimmer at 2, and the
        # attempts after it (pitch 0.38, 0.40) 25/32 and 26/32 in 40,
        # five swimmers weaving for 4 each; at K41 attempt 4 routed 30
        # of 37 and attempt 5, the one that shipped, 29. Each attempt
        # is a full re-route from the base copper, so the one with the
        # most lanes routed (fewest vias on a tie) is restored -- its
        # copper, its bookkeeping and its plan geometry, so the last
        # call re-lays that attempt's refusals in that attempt's world.
        best = None
        stale = 0
        for attempt in range(6):
            sched = plan_at(ly_floor)
            log(f'  attempt {attempt}: need {self.layout_need:.2f} of '
                f'{self.L_free - RESERVE:.2f} mm, W={self.W:.3f}, launch pitch >= '
                f'{ly_floor:.2f}; pages F {len(M) - len(sched.divers)} / B '
                f'{len(sched.b_page)} / swimmers {len(sched.swimmers)}'
                + (f', {len(self.crossings)} lane(s) crossed by exit legs'
                   if self.crossings else ''))
            ctx.pcb.segments = list(ctx.base_segments)
            ctx.pcb.vias = list(ctx.base_vias)
            self.out_segs, self.out_vias = {}, {}
            ctx.landed -= set(M)
            divers = set(sched.divers)
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
            routed = set()
            self.refused = []
            failed_rescues = 0
            for nm in order:
                unrouted = [om for om in M if om != nm and om not in routed]
                res = self.route_lane(nm, self.virtual_of(unrouted),
                                      self.virtual_vias_of(unrouted))
                if res is None and failed_rescues < 3:
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
                    # STRUCTURED refusal (#622 plan communication):
                    # the reason used to die in this log line, and
                    # retry drivers answered a bare net name with
                    # blunt force knobs. This is what the braid knows
                    # at refusal time.
                    sc_ = getattr(self, 'sched_cur', None)
                    ctx.refusal_info[nm] = {
                        'stage': 'attempt',
                        'corridor': self.idx,
                        'page': (sc_.page.get(nm) if sc_ is not None
                                 else None),
                        'swimmer': bool(sc_ is not None
                                        and sc_.page.get(nm) is None),
                        'tooth_layer': ctx.tooth_layer[nm],
                        'dest_layer': ctx.dest_layer[nm],
                        'tooth': list(self.teeth[nm]),
                        'berth': list(self.stubs[nm]),
                        'failed_rescues': failed_rescues,
                    }
                    log(f'    refused: {nm}')
                    continue
                routed.add(nm)
                ctx.landed.add(nm)
                segs_o, vias_o = res
                self.out_segs[nm] = segs_o
                self.out_vias[nm] = vias_o
            nv = sum(len(v) for v in self.out_vias.values())
            log(f'    lanes: {len(routed)}/{len(M)} routed, {nv} via(s)')
            key = (len(routed), -nv)
            if best is None or key > best['key']:
                best = dict(key=key, attempt=attempt, ly=ly_floor,
                            segs=list(ctx.pcb.segments), vias=list(ctx.pcb.vias),
                            out_segs=dict(self.out_segs), out_vias=dict(self.out_vias),
                            refused=list(self.refused), landed=set(ctx.landed))
            if not self.refused:
                break
            # attempts converge fast on the ribbon: once the
            # refused set repeats with the launch pitch maxed and
            # the boost already applied, later attempts are
            # identical -- and each one re-runs tens of thousands
            # of exhausted A* iterations per refusal (K28: 378k
            # per attempt). A refused set that ALTERNATES between
            # two lanes at the maxed pitch (K35: SA4 / SCKE1 with the
            # far-face exits, 2026-09-07) never repeats and ran all
            # six attempts, each a full re-route, for a best attempt
            # that was attempt 0 -- so an attempt that does not beat
            # the best attempt's routed count is STALE, and two stale
            # attempts in a row end the loop as well. The best attempt
            # is kept either way.
            if ly_floor >= 0.40 - 1e-9 and attempt >= 2:
                if self.refused == prev_refused:
                    log('    attempts converged; stopping early')
                    break
                if best is not None and len(routed) <= best['key'][0] \
                        and best['attempt'] != attempt:
                    stale += 1
                else:
                    stale = 0
                if stale >= 2:
                    log(f'    attempts stale; stopping early (best is attempt '
                        f'{best["attempt"]}, {best["key"][0]}/{len(M)} routed)')
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
        if best is not None and best['attempt'] != attempt:
            log(f'    kept attempt {best["attempt"]} ({best["key"][0]}/{len(M)} '
                f'routed, {-best["key"][1]} via(s), launch pitch >= '
                f'{best["ly"]:.2f}) over attempt {attempt} '
                f'({len(routed)}/{len(M)}, {nv})')
            if abs(best['ly'] - ly_floor) > 1e-9:
                sched = plan_at(best['ly'])
            ctx.pcb.segments = list(best['segs'])
            ctx.pcb.vias = list(best['vias'])
            self.out_segs = dict(best['out_segs'])
            self.out_vias = dict(best['out_vias'])
            self.refused = list(best['refused'])
            ctx.landed -= set(M)
            ctx.landed |= best['landed']
        if self.refused:
            # LAST CALL -- the corridor's own router, off the lattice.
            # Every routed lane is real copper by now, and what
            # refused a ribbon swimmer was its BAND -- the tube around
            # a straight line through a saturated lattice -- not the
            # corridor itself. Each refused lane gets one more search:
            # the same engine, costs and virtual copper (for the other
            # refused lanes), but no band, a wide window round its own
            # planned path, and the x4 budget. The copper comes out
            # lane-shaped because it is the lane router.
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
                    rep = {}
                    res = self.connect_ladder(
                        nm, self.virtual_of(others),
                        self.virtual_vias_of(others), 'last_call',
                        b_alts=ctx.dest_alts.get(nm), report=rep)
                    if res is None:
                        # ...and the blocker-directed rip (rip_for)
                        res = self.rip_for(nm, others, rep)
                    if res is None:
                        info = ctx.refusal_info.setdefault(nm, {})
                        info.update(stage='last_call',
                                    margins=[2.0, 4.0, 6.0],
                                    rip_assist=True)
                        log(f'    last call: {nm} still refused')
                        continue
                    segs_o, vias_o = res
                    ctx.pcb.segments.extend(segs_o)
                    ctx.pcb.vias.extend(vias_o)
                    self.out_segs[nm] = segs_o
                    self.out_vias[nm] = vias_o
                    still.remove(nm)
                    ctx.landed.add(nm)
                    log(f'    last call routed: {nm} ({len(vias_o)} via(s))')
                self.refused = still
            finally:
                ctx.cfg = cfg0
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
        # detour is inside the search.
        import copy as _copy
        cfg0 = ctx.cfg
        big2 = _copy.copy(cfg0)
        big2.max_iterations = 4 * max(cfg0.max_iterations, 50_000)
        ctx.cfg = big2
        try:
            # ECON_MIN_VIAS: the cheapest lane worth trying to
            # re-lay. 3 exempted every 2-via lane, and the K28
            # ledger says that is where the waste lives (SA0/SA4/
            # SODT1 dive once each where the cover model rides
            # free -- the last call keeps the FIRST route found at
            # the smallest margin, never asking whether a wider
            # window holds a cheaper one; this pass is the asker).
            econ_min = 3

            def lane_mm(nm):
                return sum(math.hypot(s.end_x - s.start_x,
                                      s.end_y - s.start_y)
                           for s in self.out_segs.get(nm) or ())

            def airline(nm):
                t_, u_ = self.teeth[nm], self.stubs[nm]
                return math.hypot(t_[0] - u_[0], t_[1] - u_[1])

            # candidates: via-heavy lanes AND long ones (the
            # berth overshoot lives here: primary lanes keep the
            # TIP discipline so attachment stays order-independent
            # -- early-join as a primary default let early lanes
            # park in the berth-approach channel and starved the
            # late ones (K32 open either way, live or deferred
            # trim) -- so the economy is harvested HERE, post-
            # completion, where it cannot cost a lane)
            heavy = sorted(
                (nm for nm in self.members
                 if len(self.out_vias.get(nm) or ()) >= econ_min
                 or (self.out_segs.get(nm)
                     and lane_mm(nm) - airline(nm) > 3.0)),
                key=lambda nm: -len(self.out_vias.get(nm) or ()))
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
                                    window_pts=wp,
                                    b_alts=ctx.dest_alts.get(nm))
                    # keep FEWER vias, or equal vias and clearly
                    # shorter copper (the overshoot harvest)
                    nv0 = len(self.out_vias[nm])
                    if r_ is not None and (
                            len(r_[1]) < nv0
                            or (len(r_[1]) == nv0
                                and sum(math.hypot(
                                    s.end_x - s.start_x,
                                    s.end_y - s.start_y)
                                    for s in r_[0])
                                < lane_mm(nm) - 0.5)):
                        res = r_
                        break
                if res is None:
                    ctx.pcb.segments = seg0
                    ctx.pcb.vias = via0
                    continue
                segs_o, vias_o = res
                ctx.pcb.segments.extend(segs_o)
                ctx.pcb.vias.extend(vias_o)
                if not net_walks(ctx.pcb, nid, ctx.byname[nm][1]):
                    ctx.pcb.segments = seg0
                    ctx.pcb.vias = via0
                    log(f'    econ re-lay: {nm} REJECTED '
                        '(net would disconnect)')
                    continue
                if lane_crosses_foreign(ctx.pcb, nid,
                                        res[0]):
                    ctx.pcb.segments = seg0
                    ctx.pcb.vias = via0
                    log(f'    econ re-lay: {nm} REJECTED '
                        '(path crosses foreign copper -- '
                        'engine-window bug, reproducer kept)')
                    continue
                hit = via_hits_foreign(ctx.pcb, nid, vias_o)
                if hit:
                    ctx.pcb.segments = seg0
                    ctx.pcb.vias = via0
                    log(f'    econ re-lay: {nm} REJECTED '
                        f'({hit})')
                    continue
                log(f'    econ re-lay: {nm} '
                    f'{len(self.out_vias[nm])} -> {len(vias_o)} '
                    'via(s)')
                self.out_segs[nm], self.out_vias[nm] = segs_o, vias_o
        finally:
            ctx.cfg = cfg0
        self.finish()

    def connect_ladder(self, nm, virt, virt_vias, stage, b_alts=None,
                       nm_ends=None, report=None):
        """A re-lay that keeps to the PLAN: the lane's own band widened
        in steps (slack 0.3, 0.8 in o; then 1.6 with both layers open),
        or -- for a corridor with no frame -- a tube round its taut
        path (0.6, 1.2, 2.0 mm); only then, and only with
        the band-free window that used to
        be the first thing every refusal got. ctx.rungs counts which
        rung landed each lane, so the band-free share is measured."""
        ctx = self.ctx
        nid, _ = ctx.byname[nm]
        a, aL, b, bL = nm_ends or (self.teeth[nm], ctx.tooth_layer[nm],
                                   self.stubs[nm], ctx.dest_layer[nm])
        rungs = [('slack0.3', lambda: self.band_of(nm, 0.3), 2.0),
                 ('slack0.8', lambda: self.band_of(nm, 0.8), 2.0),
                 ('slack1.6+open', lambda: self.band_of(nm, 1.6, True), 3.0),
                 ('free', lambda: None, 4.0), ('free', lambda: None, 6.0)]
        wp = (getattr(self, 'lane_xy', {}) or {}).get(nm) or [a, b]
        for label, mk, mg in rungs:
            res = cn.connect(ctx.pcb, nid, a, aL, b, bL, ctx.cfg,
                             band=mk(), margin=mg,
                             virtual=list(virt or []) + reserve(ctx, nm),
                             window_pts=wp, virtual_vias=virt_vias,
                             b_alts=b_alts, report=report)
            if res is not None:
                ctx.rungs[(stage, label)] += 1
                return res
        return None

    def rip_for(self, nm, others, rep, max_victims=3, depth=1, protect=frozenset()):
        """BLOCKER-DIRECTED RIP at last call (#622 K41 SBA2). A lane
        still refused when every other lane is real copper is boxed by
        lanes routed before it -- the sequential loss, an earlier lane
        having taken the one channel a later one needs -- and no wider
        window answers that. The refused search's blocked FRONTIER
        (connect's report) is attributed to nets by the production
        blocking analysis, the one route.py's own rip ladder uses, which
        names the lanes of THIS run on it; a MIN-CUT probe (one search
        with those lanes priced, not blocked -- connect soft=) then
        finds the path crossing the fewest of them, and the lanes it
        crosses, in path order, are the cut set. The rip ladder is the
        cut set's prefixes, then the frontier's most exposed lanes
        singly: each trial rips its victims, routes the refused lane
        through its ladder, re-lays each victim against the new lane
        through ITS ladder (band first, so a page lane stays on its page
        when it can), and a victim that cannot be re-laid negotiates one
        level down with the placed lane protected. Kept only when the
        refused lane AND every victim route (an open closed; vias may
        move); otherwise every piece of copper is put back exactly.
        Static copper (stubs, pads, other nets) is never a victim: a
        lane walled by it is a fanout matter, and says so. Returns the
        refused lane's copper, not yet in the board, like any ladder."""
        import time as _time
        ctx, log = self.ctx, self.log
        nid, _ = ctx.byname[nm]
        t0 = _time.perf_counter()
        blocked = rep.get('blocked') or []
        if not blocked:
            log(f'    rip for {nm}: the refusal reported no frontier')
            return None
        from blocking_analysis import analyze_frontier_blocking
        cand = {ctx.byname[om][0]: om for om in self.members
                if om != nm and om not in protect and self.out_segs.get(om)}
        infos = analyze_frontier_blocking(
            blocked, rep['window'], rep['cfg'], {i: None for i in cand},
            exclude_net_ids={nid}, target_xy=self.stubs[nm],
            source_xy=self.teeth[nm])
        named = [(cand[b.net_id], b.blocked_count) for b in infos
                 if b.net_id in cand]
        log(f'    rip for {nm}: frontier {len(blocked)} cells; lanes of this '
            f'run on it: ' + (', '.join(f'{v}({c})' for v, c in named)
                              if named else 'none -- walled by static copper')
            + f'  ({_time.perf_counter() - t0:.1f} s)')
        if not named:
            return None
        virt, vv = self.virtual_of(others), self.virtual_vias_of(others)
        # THE MIN-CUT PROBE: the frontier ranks lanes by EXPOSURE (the
        # perimeter of the reachable pocket), not by whether ripping
        # them opens a path. One more search with every lane of this
        # run PRICED instead of blocked (connect soft=) finds the path
        # that crosses the fewest of them; the lanes that path conflicts
        # with, in path order, are the cut set -- jointly sufficient by
        # construction, so its prefixes are the rip ladder. No path even
        # then = walled by static copper: nothing to rip.
        lanes = [(om, self.out_segs[om], self.out_vias[om]) for om, _c in named]
        ids_s = {id(x) for _, ss, _ in lanes for x in ss}
        ids_v = {id(x) for _, _, vs in lanes for x in vs}
        seg0, via0 = list(ctx.pcb.segments), list(ctx.pcb.vias)
        ctx.pcb.segments = [x for x in seg0 if id(x) not in ids_s]
        ctx.pcb.vias = [x for x in via0 if id(x) not in ids_v]
        soft = [((x.start_x, x.start_y), (x.end_x, x.end_y), x.layer, x.width)
                for _, ss, _ in lanes for x in ss]
        soft_v = [(x.x, x.y, x.size) for _, _, vs in lanes for x in vs]
        wp = (getattr(self, 'lane_xy', {}) or {}).get(nm) or [self.teeth[nm], self.stubs[nm]]
        probe = cn.connect(ctx.pcb, nid, self.teeth[nm], ctx.tooth_layer[nm],
                           self.stubs[nm], ctx.dest_layer[nm], ctx.cfg,
                           band=None, margin=6.0,
                           virtual=list(virt) + reserve(ctx, nm), window_pts=wp,
                           virtual_vias=vv, b_alts=ctx.dest_alts.get(nm),
                           soft=soft, soft_vias=soft_v)
        ctx.pcb.segments, ctx.pcb.vias = seg0, via0
        if probe is None:
            log(f'    rip for {nm}: no path even with every lane priced -- '
                f'walled by static copper  ({_time.perf_counter() - t0:.1f} s)')
            return None
        reach = ctx.cfg.clearance + ctx.cfg.track_width
        cut = []
        for ps in probe[0]:
            a, b = (ps.start_x, ps.start_y), (ps.end_x, ps.end_y)
            for om, ss, vs in lanes:
                if om in cut:
                    continue
                hit = any(x.layer == ps.layer and ts.seg_seg_dist(
                    a, b, (x.start_x, x.start_y), (x.end_x, x.end_y))
                    < reach + (x.width - ctx.cfg.track_width) / 2 + 1e-6 for x in ss) \
                    or any(ts.seg_pt_dist(a, b, (x.x, x.y))
                           < x.size / 2 + ctx.cfg.clearance + ctx.cfg.track_width / 2 + 1e-6
                           for x in vs)
                if hit:
                    cut.append(om)
        for pv in probe[1]:
            for om, ss, vs in lanes:
                if om in cut:
                    continue
                if any(ts.seg_pt_dist((x.start_x, x.start_y), (x.end_x, x.end_y), (pv.x, pv.y))
                       < pv.size / 2 + ctx.cfg.clearance + x.width / 2 + 1e-6 for x in ss) \
                        or any(math.hypot(x.x - pv.x, x.y - pv.y)
                               < (x.size + pv.size) / 2 + ctx.cfg.clearance + 1e-6 for x in vs):
                    cut.append(om)
        log(f'    rip for {nm}: min-cut probe {len(probe[1])} via(s) crosses '
            f'{cut or "nothing (a MISSED search)"}  ({_time.perf_counter() - t0:.1f} s)')
        if not cut:
            # the probe found a legal path through nothing: the last
            # call's search was starved, not walled -- route it
            trials = [[]]
        else:
            victims = cut[:max_victims]
            trials = [victims[:k] for k in range(1, len(victims) + 1)]
        # ...then the frontier's most exposed lanes, singly: a cut lane
        # that cannot be re-laid is no victim, and the lane that holds
        # the most of the frontier often can be (K41 SBA2: the cut set
        # SCKE1/SA1/SA2 each lost a victim; SA8, first by exposure,
        # re-laid at 4 then 0 vias)
        for v, _c in named[:max_victims]:
            if [v] not in trials:
                trials.append([v])
        for V in trials:
            seg0, via0 = list(ctx.pcb.segments), list(ctx.pcb.vias)
            ids_s = {id(x) for v in V for x in self.out_segs[v]}
            ids_v = {id(x) for v in V for x in self.out_vias[v]}
            ctx.pcb.segments = [x for x in seg0 if id(x) not in ids_s]
            ctx.pcb.vias = [x for x in via0 if id(x) not in ids_v]
            r1 = self.connect_ladder(nm, virt, vv, 'rip',
                                     b_alts=ctx.dest_alts.get(nm))
            if r1 is None or lane_crosses_foreign(ctx.pcb, nid, r1[0]):
                ctx.pcb.segments, ctx.pcb.vias = seg0, via0
                log(f'    rip {V}: {nm} still refused  '
                    f'({_time.perf_counter() - t0:.1f} s)')
                continue
            ctx.pcb.segments.extend(r1[0])
            ctx.pcb.vias.extend(r1[1])
            relaid, lost = {}, None
            for v in V:
                vid, _ = ctx.byname[v]
                rep2 = {}
                r2 = self.connect_ladder(v, virt, vv, 'rip',
                                         b_alts=ctx.dest_alts.get(v), report=rep2)
                if r2 is None and depth > 0:
                    # the victim negotiates in turn, the lane just placed
                    # (and its own placer) protected
                    r2 = self.rip_for(v, others, rep2, max_victims, depth - 1,
                                      protect | {nm})
                if r2 is None or lane_crosses_foreign(ctx.pcb, vid, r2[0]):
                    lost = v
                    break
                ctx.pcb.segments.extend(r2[0])
                ctx.pcb.vias.extend(r2[1])
                relaid[v] = r2
            if lost is not None:
                ctx.pcb.segments, ctx.pcb.vias = seg0, via0
                log(f'    rip {V}: {nm} routed ({len(r1[1])} via(s)) but '
                    f'{lost} lost -- put back  ({_time.perf_counter() - t0:.1f} s)')
                continue
            log(f'    rip {V}: {nm} routed ({len(r1[1])} via(s)); re-laid '
                + ', '.join(f'{v} {len(self.out_vias[v])} -> {len(r2[1])} via(s)'
                            for v, r2 in relaid.items())
                + f'  ({_time.perf_counter() - t0:.1f} s)')
            for v, r2 in relaid.items():
                self.out_segs[v], self.out_vias[v] = r2
            # the caller appends the routed lane, as after any ladder
            ids1 = {id(x) for x in r1[0]} | {id(x) for x in r1[1]}
            ctx.pcb.segments = [x for x in ctx.pcb.segments if id(x) not in ids1]
            ctx.pcb.vias = [x for x in ctx.pcb.vias if id(x) not in ids1]
            return r1
        return None




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
        ctx.laid_tubes.append((self.spine_core.pts, self.H))
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
            if nm in self.leg_split:
                self.marks.append(sp.xy(self.exit_leg_s[nm], self.leg_split[nm]))
            for (p_, q_) in self.jogs.get(nm, ()):
                self.marks.append(sp.xy(*q_))
        for (_i, s_c, _t) in sp.corners():
            self.marks.append(sp.xy(s_c, 0.0))



def _orient(ax, ay, bx, by, cx, cy):
    v = (by - ay) * (cx - bx) - (bx - ax) * (cy - by)
    return 0 if abs(v) < 1e-12 else (1 if v > 0 else -1)


def lane_crosses_foreign(pcb, nid, new_segs):
    """A re-laid lane must not CROSS foreign same-layer copper. The
    router should make this impossible, yet two econ accepts measured
    otherwise (K32 SDQM1 x SDQ9 at (127.55,61.30); K15 SA7 x SRAS) --
    an engine-window bug still under study. Until it is found, the
    acceptance refuses crossing paths outright."""
    for s in new_segs:
        for t in pcb.segments:
            if t.net_id == nid or t.layer != s.layer:
                continue
            if max(s.start_x, s.end_x) < min(t.start_x, t.end_x) - 0.01 \
                    or min(s.start_x, s.end_x) > max(t.start_x, t.end_x) + 0.01 \
                    or max(s.start_y, s.end_y) < min(t.start_y, t.end_y) - 0.01 \
                    or min(s.start_y, s.end_y) > max(t.start_y, t.end_y) + 0.01:
                continue
            o1 = _orient(s.start_x, s.start_y, s.end_x, s.end_y,
                         t.start_x, t.start_y)
            o2 = _orient(s.start_x, s.start_y, s.end_x, s.end_y,
                         t.end_x, t.end_y)
            o3 = _orient(t.start_x, t.start_y, t.end_x, t.end_y,
                         s.start_x, s.start_y)
            o4 = _orient(t.start_x, t.start_y, t.end_x, t.end_y,
                         s.end_x, s.end_y)
            if o1 != o2 and o3 != o4 and 0 not in (o1, o2, o3, o4):
                return True
    return False


def via_hits_foreign(pcb, nid, new_vias):
    """A re-laid via must not LAND ON foreign copper (audit #7 -- the
    econ guard checked re-laid SEGMENTS against foreign crossings but
    placed the re-laid VIAS unchecked; same belt-and-suspenders
    rationale as lane_crosses_foreign). Contact class only -- overlap
    plus a hair, NOT a DRC clearance pass, so legal tight spacing is
    never refused. A via barrel spans every layer, so foreign
    segments and pads count on ANY layer; NPTH pads count by their
    HOLE (they have no copper)."""
    M = 0.01
    for v in new_vias:
        vr = v.size / 2
        for t in pcb.vias:
            if t.net_id == nid or t is v:
                continue
            if math.hypot(t.x - v.x, t.y - v.y) < vr + t.size / 2 + M:
                return (f'via ({v.x:.2f},{v.y:.2f}) on foreign via '
                        f'({t.x:.2f},{t.y:.2f})')
        for s in pcb.segments:
            if s.net_id == nid:
                continue
            ax, ay, bx, by = s.start_x, s.start_y, s.end_x, s.end_y
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            t_ = 0 if L2 == 0 else max(0.0, min(1.0, (
                (v.x - ax) * dx + (v.y - ay) * dy) / L2))
            d = math.hypot(v.x - (ax + t_ * dx), v.y - (ay + t_ * dy))
            if d < vr + s.width / 2 + M:
                return (f'via ({v.x:.2f},{v.y:.2f}) on foreign seg '
                        f'{s.layer}@({ax:.2f},{ay:.2f})')
        for fp in pcb.footprints.values():
            for p in fp.pads:
                if p.net_id == nid:
                    continue
                if p.pad_type == 'np_thru_hole':
                    hx = p.hole_x if p.hole_x is not None else p.global_x
                    hy = p.hole_y if p.hole_y is not None else p.global_y
                    if math.hypot(hx - v.x, hy - v.y) \
                            < vr + (p.drill or 0) / 2 + M:
                        return (f'via ({v.x:.2f},{v.y:.2f}) on NPTH '
                                f'hole {fp.reference}.{p.pad_number}')
                    continue
                if abs(p.global_x - v.x) < vr + p.size_x / 2 + M \
                        and abs(p.global_y - v.y) < vr + p.size_y / 2 + M:
                    return (f'via ({v.x:.2f},{v.y:.2f}) on pad '
                            f'{fp.reference}.{p.pad_number}')
    return None


def net_walks(pcb, nid, net):
    """Union-find over the net's copper + pads + vias: True when every
    pad sits in one component. The acceptance guard for re-lays -- a
    lane whose end-layer was mis-resolved connects only through the
    accident of its own vias, and a via-free replacement severs the
    net (K15 SA7)."""
    parent = {}

    def find(x):
        while parent.setdefault(x, x) != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(x, y):
        parent[find(x)] = find(y)

    pts = []
    for i, s in enumerate(pcb.segments):
        if s.net_id != nid:
            continue
        union(('s', i, 0), ('s', i, 1))
        pts.append((('s', i, 0), s.start_x, s.start_y, s.layer))
        pts.append((('s', i, 1), s.end_x, s.end_y, s.layer))
    # pad anchors are LAYER-AWARE (audit #5): an SMD pad connects only
    # copper on its own layer -- a B whisker END under an F ball is
    # NOT contact, and treating it as contact let a barrel-removing
    # re-lay pass this guard while severing the net (reproduced:
    # SA9/SA6 under-pad via removed, guard still said connected).
    # Vias anchor every layer (through vias on this stack); a
    # through-hole/'*.Cu' pad does too.
    def _pad_lays(p):
        if (p.drill or 0) > 0 or not p.layers or '*.Cu' in p.layers:
            return None                    # every copper layer
        return {l for l in p.layers if l.endswith('.Cu')}

    anch = [(('v', i), v.x, v.y, v.size / 2 + 0.02, None)
            for i, v in enumerate(pcb.vias) if v.net_id == nid] + \
        [(('p', i), p.global_x, p.global_y,
          max(p.size_x, p.size_y) / 2 + 0.02, _pad_lays(p))
         for i, p in enumerate(net.pads)]
    for (k, x, y, _l) in pts:
        for (k2, x2, y2, r, lays) in anch:
            if (lays is None or _l in lays) \
                    and math.hypot(x - x2, y - y2) <= r:
                union(k, k2)
    # anchors union with EACH OTHER on copper overlap (via-in-pad:
    # the barrel sits in the pad copper with no same-layer segment
    # endpoint involved -- the old blind rule got this union only by
    # the same accident that was the bug)
    for i in range(len(anch)):
        k1, x1, y1, r1, l1 = anch[i]
        for j in range(i + 1, len(anch)):
            k2, x2, y2, r2, l2 = anch[j]
            if (l1 is None or l2 is None or (l1 & l2)) \
                    and math.hypot(x1 - x2, y1 - y2) <= r1 + r2:
                union(k1, k2)
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            ka, xa, ya, la = pts[i]
            kb, xb, yb, lb = pts[j]
            if la == lb and math.hypot(xa - xb, ya - yb) <= 0.06:
                union(ka, kb)
    roots = {find(('p', i)) for i in range(len(net.pads))}
    return len(roots) <= 1


def note_joint(ctx, nm, new_segs):
    """AT WRITE TIME ONLY: which dest-chain vertex the net's FINAL
    lane reached; the bypassed tip-side stub segments are removed
    from ctx.pcb (smoothed write re-emits from it) and their spans
    recorded for the non-smoothed writer's text strip.

    This used to run LIVE after every join, which lost K32 its
    completion: the braid rips and retries lanes across attempts, a
    trimmed stub left self.stubs[nm] pointing at a bare point, and
    later lanes routed a world that shifted underfoot (13 nets
    trimmed mid-run, SDQM0 unroutable). Deferred, the routing world
    is IDENTICAL to tip-join semantics -- lanes just end early on the
    same stub line -- and the trim is pure output economy."""
    chain = ctx.dest_chain.get(nm) or []
    if not chain:
        return
    endpts = set()
    for s in new_segs:
        endpts.add((round(s.start_x, 3), round(s.start_y, 3)))
        endpts.add((round(s.end_x, 3), round(s.end_y, 3)))
    cut = None
    for k, (_s, t, p) in enumerate(chain):
        if t in endpts:
            cut = k          # joined at the tip-side end of seg k
            break
        if p in endpts:
            cut = k + 1      # joined at its pad-side end
            break
    if not cut:              # tip join (cut 0) or no contact: no trim
        return
    gone = chain[:cut]
    ids = {id(s) for (s, _t, _p) in gone}
    ctx.pcb.segments = [s for s in ctx.pcb.segments
                        if id(s) not in ids]
    ctx.trim_spans[nm] = [(t, p) for (_s, t, p) in gone]


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
    a = ap.parse_args()
    names = [n.strip() for n in a.nets.split(',') if n.strip()]

    # MEM_TRACE=1: every log line carries the seconds since start, the
    # process's peak RSS so far (ru_maxrss, monotone: a jump names the
    # phase that allocated) and, under `python3 -X tracemalloc`, the
    # Python-side peak SINCE THE PREVIOUS LINE (reset after each), which
    # catches an allocation freed before the next line -- the 800 MB
    # swings the sampler saw at K15 (README TODO 10).
    _t0 = _time.time()
    _trace = os.environ.get('MEM_TRACE') == '1'

    def log(msg=''):
        if _trace:
            import resource
            import tracemalloc
            peak = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1048576
            tm = ''
            if tracemalloc.is_tracing():
                cur, pk = tracemalloc.get_traced_memory()
                tm = f' py {cur / 1048576:5.0f}/{pk / 1048576:5.0f}MB'
                tracemalloc.reset_peak()
            print(f'[{_time.time() - _t0:6.1f}s rss<={peak:5.0f}MB{tm}] {msg}', flush=True)
        else:
            print(msg)
    ctx, groups = setup(a.board, names, a.dest, log)
    corridors = []
    for ci, members in enumerate(groups):
        corridors.append(Corridor(ci, members, ctx, log))
    ctx.corridors = corridors

    # PHASE 1 -- every corridor PLANNED (spine, offsets, schedule,
    # planned lanes) before any is routed, each spine relaxed round
    # the ones planned before it, so that while a corridor routes,
    # the others' planned lanes are reservations (cross_reserve)
    for c in corridors:
        try:
            c.run(plan_only=True)
            ctx.laid.extend(c.lane_xy[nm] for nm in c.members
                            if nm in getattr(c, 'lane_xy', {}))
            if getattr(c, 'spine_core', None) is not None:
                ctx.laid_tubes.append((c.spine_core.pts, c.H))
        except Exception as e:
            log(f'  plan phase: corridor {c.idx} not planned ({e})')
    ctx.laid = []
    ctx.laid_tubes = []
    ctx.pcb.segments = list(ctx.base_segments)
    ctx.pcb.vias = list(ctx.base_vias)
    for c in corridors:
        c.run()
        ctx.corr_done.add(c.idx)
    if ctx.rungs:
        log('re-lay rungs: ' + ', '.join(f'{st}/{r} {n}' for (st, r), n
                                         in sorted(ctx.rungs.items())))
    return write_out(a, ctx, corridors, names, log)


def pair_chirality_of(pcb, names, byname, dest):
    """The pair's chirality as the fanout's plan computes it
    (select_moves.pair_chirality on the same balls and the same array
    boxes): the run's nets' pad on `dest` against their pad on the source
    array, the boxes those arrays' pads span."""
    src, dst, refs = {}, {}, Counter()
    for nm in names:
        _nid, net = byname[nm]
        d = [p for p in net.pads if p.component_ref == dest]
        o = [p for p in net.pads if p.component_ref != dest]
        if d:
            dst[nm] = (d[0].global_x, d[0].global_y)
        if o:
            src[nm] = (o[0].global_x, o[0].global_y)
            refs[o[0].component_ref] += 1
    if not src or not dst or dest not in pcb.footprints:
        return 1
    sref = max(refs, key=refs.get)
    return pair_chirality(src, dst, em.grid_of(pcb.footprints[sref]).bbox,
                          em.grid_of(pcb.footprints[dest]).bbox)


def mirror_plan(plan, M):
    """The plan dict in the turned frame: ends mirrored, layers swapped,
    escape directions' y negated."""
    q = dict(plan)
    if 'ends' in plan:
        q['ends'] = {nm: [list(M(*e[0])), list(M(*e[1]))]
                     for nm, e in plan['ends'].items()}
    for k in ('tooth_layer', 'dest_layer'):
        if k in plan:
            q[k] = {nm: other_layer(L) for nm, L in plan[k].items()}
    for k in ('tooth_dir', 'stub_dir'):
        if k in plan:
            q[k] = {nm: [d[0], -d[1]] for nm, d in plan[k].items()}
    return q


def setup(board, names, dest, log, plan=None):
    """Everything the corridors are built from: the board, the ends,
    the static obstacles, the flow directions, the corridor groups.

    `plan` -- ONE PLANNER (#622): the ends, their layers and their escape
    directions given by the plan instead of read off copper:
    {'ends': {net: [tooth_xy, exit_xy]}, 'tooth_layer', 'dest_layer',
    'tooth_dir': {net: unit xy}, 'stub_dir': {net: unit xy}}. The plan
    judges a candidate by calling this same stage (plan_braid) before
    any destination copper exists, and the braid, given the same plan
    beside its board (`<board>.plan.json`, written by the fanout), builds
    its corridors from the identical inputs -- so the two cannot drift.
    Without it (or for nets the plan does not name) everything is read
    off the board as before."""
    pcb = parse_kicad_pcb(board)
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    kids = {byname[nm][0] for nm in names}
    if plan is None:
        _pj = os.path.splitext(board)[0] + '.plan.json'
        if os.path.exists(_pj):
            import json as _json_pl
            try:
                with open(_pj, encoding='utf-8') as _f:
                    plan = _json_pl.load(_f)
                named = [nm for nm in names if nm in plan.get('ends', {})]
                if not named:
                    log(f'plan sidecar {os.path.basename(_pj)} names none of '
                        f'this run\'s nets -- ignored')
                    plan = None
                elif len(named) < len(names):
                    # a PARTIAL plan is still the plan for the nets it
                    # names: the fanout's selector can leave a net
                    # unplaced (K41's SA9, its menu banned away) and fan
                    # it out unplanned, and the judge that chose that
                    # fanout applied the plan to the other 40 -- so the
                    # braid must too, or it runs on a different world
                    # than the one the plan was judged in (measured at
                    # K41: every net read off copper, the spine 0.07 mm
                    # off the judged one, three legs moved to one s)
                    miss = [nm for nm in names if nm not in plan.get('ends', {})]
                    log(f'plan from {os.path.basename(_pj)}: {len(named)} of '
                        f'{len(names)} nets; {", ".join(miss)} read off the board')
                else:
                    log(f'plan from {os.path.basename(_pj)}: ends, layers and '
                        f'escape directions as the plan decided them')
            except (OSError, ValueError) as _e:
                log(f'plan sidecar unreadable ({_e}); reading the board')
                plan = None
    # THE PAIR'S FRAME: the braid is handed (two leg-layer ties fall to
    # F, the back-only stretch filter, F as the main page, first-index
    # runs), so a pair and its mirror were braided differently (the
    # board turned over: K28 40 vias against 38, its plan judged 17 up /
    # 3 down where the exact mirror is 16 / 5). Like the fanout's
    # selector (PairFrame) the braid runs every pair in its +1 frame:
    # a -1 pair's board is turned over IN MEMORY here (every part to
    # the other face, y mirrored about a lattice line, layers swapped
    # -- the engine's own flip_frame), the plan mirrored into it, the
    # planner and the braid run unchanged, and the copper and the
    # planner's layers are mirrored back at the two exits (plan_braid,
    # write_out). Per pair, not per board: with three or more parts the
    # chirality is each pair's own. The plan carries the chirality it
    # was made in ('chi'); the braid's own reading of the same balls
    # must agree, and says so if not.
    chi = pair_chirality_of(pcb, names, byname, dest)
    if plan and plan.get('chi') is not None and int(plan['chi']) != chi:
        log(f'pair frame: the plan was made at chirality {plan["chi"]:+d}, '
            f'the board reads {chi:+d} -- following the plan')
        chi = int(plan['chi'])
    ctx = Ctx()
    ctx.chi, ctx.M = chi, None
    if chi < 0:
        CY = mirror_axis(pcb)     # the engine's own rule: a lattice line
        pcb, M = to_front_frame(pcb, dest)
        ctx.M = M
        byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
        if plan:
            plan = mirror_plan(plan, M)
        log(f'pair frame: chirality -1, the board turned over about '
            f'y = {CY:.3f} for the braid (copper mirrored back on write)')
    planned = {nm for nm in names if plan and nm in plan.get('ends', {})}
    ends = endpoints(pcb, [nm for nm in names if nm not in planned], byname,
                     dest_ref=dest) if len(planned) < len(names) else {}
    for nm in planned:
        e = plan['ends'][nm]
        ends[nm] = (tuple(e[0]), tuple(e[1]), dest)
    ctx.pcb, ctx.byname, ctx.ends, ctx.kids = pcb, byname, ends, kids
    ctx.plan = plan
    ctx.tooth_layer = {nm: (plan['tooth_layer'][nm] if nm in planned else
                            _layer_at(pcb, byname[nm][0], ends[nm][0], 'F.Cu'))
                       for nm in names}
    ctx.dest_layer = {nm: (plan['dest_layer'][nm] if nm in planned else
                           _layer_at(pcb, byname[nm][0], ends[nm][1], 'F.Cu'))
                      for nm in names}
    bundle_layer = bundle_layer_of(ctx.tooth_layer)
    # the DEST STUB CHAIN per net: the stub polyline walked from the
    # tip (the free end the braid targets) back toward the pad, on
    # the tip's layer, stopping at any junction, same-net via or pad.
    # Its pad-side vertices are ALTERNATIVE lane targets (b_alts) --
    # the search finishes at the first same-net contact instead of
    # climbing to the tip and paying the span twice (the berth
    # overshoot: 4 nets / ~9 mm doubled at K28) -- and once a lane
    # joins at vertex k, the bypassed tip-side segments are removed
    # so they cannot dangle as dead copper.
    ctx.dest_chain = {}
    ctx.dest_alts = {}
    ctx.trim_spans = {}
    ctx.refusal_info = {}
    ctx.landed = set()            # nets whose lane is on the board
    ctx.rungs = Counter()         # (stage, rung) -> lanes it landed
    ctx.corridors = []
    ctx.corr_done = set()
    for nm in names:
        nid, net = byname[nm]
        segs_n = [s for s in pcb.segments if s.net_id == nid]
        vias_n = [(v.x, v.y, v.size / 2) for v in pcb.vias
                  if v.net_id == nid]
        pads_n = [(p.global_x, p.global_y,
                   max(p.size_x, p.size_y) / 2) for p in net.pads]
        lay = ctx.dest_layer[nm]

        def _k(x, y):
            return (round(x, 3), round(y, 3))

        def _stop(pt):
            return any(math.hypot(pt[0] - ax, pt[1] - ay)
                       <= max(0.02, ar)
                       for ax, ay, ar in vias_n + pads_n)

        chain = []
        cur, prev = _k(*ends[nm][1]), None
        if _stop(cur):
            # the dest end resolved to an ANCHORED point -- no free
            # stub end existed (K35 SODT1: doubled 0.01mm micro-segs
            # in the fanout's B stub made every endpoint degree-2, so
            # endpoints() fell back to the PAD itself). A chain
            # walked from a pad is INVERTED: the trim would remove
            # the pad's own leg and strand it (measured -- SODT1
            # open, never refused). No chain, no alts, no trim.
            ctx.dest_chain[nm] = []
            ctx.dest_alts[nm] = []
            continue
        for _hop in range(24):
            nxt = [s for s in segs_n if s.layer == lay
                   and (id(s) != id(prev))
                   and (_k(s.start_x, s.start_y) == cur
                        or _k(s.end_x, s.end_y) == cur)]
            if len(nxt) != 1:
                break
            s = nxt[0]
            other = _k(s.end_x, s.end_y) \
                if _k(s.start_x, s.start_y) == cur \
                else _k(s.start_x, s.start_y)
            chain.append((s, cur, other))
            cur, prev = other, s
            if _stop(other):
                break
        ctx.dest_chain[nm] = chain
        ctx.dest_alts[nm] = [(p_[0], p_[1], lay)
                             for (_s, _t, p_) in chain]
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
    ctx.tooth_dir = {nm: (tuple(plan['tooth_dir'][nm]) if nm in planned else
                          _end_dir(pcb, byname[nm][0], ends[nm][0],
                                   byname[nm][1].pads)) for nm in names}
    ctx.stub_dir = {nm: (tuple(plan['stub_dir'][nm]) if nm in planned else
                         _end_dir(pcb, byname[nm][0], ends[nm][1],
                                  byname[nm][1].pads)) for nm in names}
    # the PLAN's page assignment, written beside the fanout board by
    # the two-page chain: with it the braid's Schedule uses the pages
    # the escapes were laid FOR, instead of re-deriving them from its
    # own orders and disagreeing (loaded here so the probe tools see
    # the same board the same way)
    # ---- corridors: taut paths, grouped by where they arrive and
    # whether one spine can reach them all. CACHED to a sidecar: the
    # relax is a pure function of (board, net ends) and costs 11.6s
    # of a 25s braid, recomputed identically by every trial braid on
    # the same fanout board (measured, K35 cProfile). Key = the
    # board file's (mtime_ns, size) + a version bumped whenever the
    # relax algorithm changes; per-net entries so a different K
    # fills in only what is missing. json round-trips floats exactly
    # (repr), so the cached run stays bit-identical.
    TAUT_CACHE_VERSION = 2   # 2: taut_clean reseeds (0902)
    log('taut paths...')
    import json as _json
    _tc_path = os.path.splitext(board)[0] + '.taut.json'
    _tc_key = None
    try:
        _st = os.stat(board)
        _tc_key = [int(_st.st_mtime_ns), int(_st.st_size),
                   TAUT_CACHE_VERSION, ctx.chi]
    except OSError:
        pass
    _cached = {}
    if planned:
        _tc_key = None      # the cache keys on the board's own ends
    if _tc_key is not None and os.path.exists(_tc_path):
        try:
            with open(_tc_path, encoding='utf-8') as _f:
                _tc = _json.load(_f)
            if _tc.get('key') == _tc_key:
                _cached = _tc.get('paths', {})
        except (OSError, ValueError):
            _cached = {}
    ctx.paths = {nm: [tuple(p) for p in _cached[nm]]
                 for nm in names if nm in _cached}
    _missing = [nm for nm in names if nm not in _cached]
    if _missing:
        fresh = db.taut_paths(_missing, ends,
                              lambda nm: obs_for(nm, bundle_layer))
        ctx.paths.update(fresh)
        if _tc_key is not None:
            _cached.update({nm: [list(p) for p in fresh[nm]]
                            for nm in fresh})
            _tmp = _tc_path + '.tmp'
            with open(_tmp, 'w', encoding='utf-8') as _f:
                _json.dump({'key': _tc_key, 'paths': _cached}, _f)
            os.replace(_tmp, _tc_path)
    else:
        log(f'  taut cache: {len(names)} path(s) reused')
    pad_obs = array_pad_obstacles(pcb, set(ctx.src_ref.values())
                                  | {ends[nm][2] for nm in names})
    ctx.pad_obs = pad_obs
    ctx.spine_obs = build_obstacles(pcb, -1, kids, bundle_layer)

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
        {nm: ends[nm][1] for nm in names}, pad_obs.seg_clear, D=6.0,
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
    # The router's default via cost stays: re-pricing it was measured
    # a loser (2026-08-31; 2x and 4x: ladder unchanged, K28 50 -> 54
    # vias) because the braid's vias are structural (page dive and
    # surface, reserved weaves, legs), so a costlier via only buys
    # worse detours that force more of them.
    # the board's own hole-to-hole floor when it is tighter than the
    # config's default (the second bench, zynq_ad9364, declares 0.25 mm
    # and its K28 shipped one drill-to-drill graze at the default 0.2);
    # tighten-only, so a board declaring less than the default routes
    # as before
    from list_nets import board_constraint
    h2h = board_constraint(board, 'min_hole_to_hole')
    kw = {}
    if h2h and h2h > cn.GridRouteConfig().hole_to_hole_clearance:
        kw['hole_to_hole_clearance'] = float(h2h)
    # and the board's own copper-to-edge floor, likewise tighten-only: the
    # config's 0 falls back to the track clearance (0.1), the project
    # grades the edge at its min_copper_edge_clearance (0.2 on the
    # bench's family), and a lane along an edge -- never on the bench,
    # SCAS on the pose gate's BF article at K28 -- shipped three edge
    # violations at 0.079 mm over (2026-09-08)
    edge = board_constraint(board, 'min_copper_edge_clearance')
    if edge and edge > CLEAR:
        kw['board_edge_clearance'] = float(edge)
    ctx.cfg = cn.make_config(pcb, TRACK, CLEAR, VIA_SIZE, VIA_DRILL,
                             grid_step=0.025, **kw)
    ctx.base_segments = list(pcb.segments)
    ctx.base_vias = list(pcb.vias)
    # each net's FANOUT copper, as it came: what a rip resets to
    ctx.fo_copper = {nm: ([s for s in pcb.segments if s.net_id == byname[nm][0]],
                          [v for v in pcb.vias if v.net_id == byname[nm][0]])
                     for nm in names}
    ctx.laid = []
    ctx.laid_tubes = []
    return ctx, groups



def _poly_crossings(pa, pb):
    """Proper intersections of two polylines, as board points."""
    def d(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    out = []
    for p1, p2 in zip(pa, pa[1:]):
        for p3, p4 in zip(pb, pb[1:]):
            d1, d2 = d(p3, p4, p1), d(p3, p4, p2)
            d3, d4 = d(p1, p2, p3), d(p1, p2, p4)
            if d1 * d2 < 0 and d3 * d4 < 0:
                t = d1 / (d1 - d2)
                out.append((p1[0] + t * (p2[0] - p1[0]), p1[1] + t * (p2[1] - p1[1])))
    return out


def cross_corridor_vias(corridors):
    """Vias a lane pays for crossing EARLIER corridors' lanes. Corridors
    route in index order, so a later corridor meets the earlier ones'
    lanes as real copper on whatever layer their schedules allowed
    there; where that includes the layer this lane is on by its own
    profile it must dive: ONE dive, two vias, however many crossings --
    the lane takes the other layer through the earlier corridor's
    copper and stays there (K35: SA9 / SA13 / SA8 of the 3-lane second
    corridor cross the 32-lane corridor's planned lanes 2-3 mm-clusters
    apart each and the copper pays 2 each; priced per cluster the judge
    said 6). The planned polylines also carry block excursions the real
    lanes do not (K15 SA9 crossed the plan's SRAS three times and the
    copper once). Unpriced, the judge preferred a K15 plan that made SA9
    a corridor of its own (predicted 0) which then crossed the main
    corridor's exit legs on F for 2."""
    def layer_at(c, nm, s):
        L = None
        for (sa, La) in c.layer_profile(nm):
            if sa <= s or L is None:
                L = La
        return L
    out = {}
    for j, cj in enumerate(corridors):
        scj = getattr(cj, 'sched_cur', None)
        if scj is None or not hasattr(cj, 'req'):
            continue
        for nm in cj.members:
            poly = getattr(cj, 'lane_xy', {}).get(nm)
            if scj.page.get(nm) is None or not poly:
                continue
            hits = []
            for ci in corridors[:j]:
                sci = getattr(ci, 'sched_cur', None)
                if sci is None or not hasattr(ci, 'req'):
                    continue
                for om in ci.members:
                    po = getattr(ci, 'lane_xy', {}).get(om)
                    if not po:
                        continue
                    for pt in _poly_crossings(poly, po):
                        s_n = cj.spine.project_pt(pt)[0]
                        s_o = ci.spine.project_pt(pt)[0]
                        L = layer_at(cj, nm, s_n)
                        if sci.page.get(om) is None or ci.allowed(om, s_o, L):
                            hits.append(s_n)
            if hits:
                out[nm] = 2
    return out


def plan_braid(board, names, dest, plan, log=None):
    """THE PLANNER, callable on a plan before any destination copper
    exists: the braid's own setup (corridors as it forms them, spines)
    and plan-only run (offsets, launch and target orders, the schedule's
    pages and swimmers) on the plan's ends. Returns
    {net: {'corridor': i, 'launch_idx', 'target_idx', 'page': 'F.Cu' |
    'B.Cu' | None, 'birth_b': bool, 'joiner': bool, 'side_exit': bool}}.
    What the fanout loop judges every round on, and what the braid then
    computes again -- identically -- from the same plan beside its board."""
    _log = log or (lambda msg='': None)
    ctx, groups = setup(board, names, dest, _log, plan=plan)
    corridors = []
    for ci, members in enumerate(groups):
        corridors.append(Corridor(ci, members, ctx, _log))
    ctx.corridors = corridors
    out = {}
    for c in corridors:
        try:
            c.run(plan_only=True)
            ctx.laid.extend(c.lane_xy[nm] for nm in c.members
                            if nm in getattr(c, 'lane_xy', {}))
            if getattr(c, 'spine_core', None) is not None:
                ctx.laid_tubes.append((c.spine_core.pts, c.H))
        except Exception as e:
            _log(f'  plan phase: corridor {c.idx} not planned ({e})')
    cross = cross_corridor_vias(corridors)
    for c in corridors:
        sc = getattr(c, 'sched_cur', None)
        li = {nm: i for i, nm in enumerate(getattr(c, 'launch', []))}
        ti = {nm: i for i, nm in enumerate(getattr(c, 'target', []))}
        for nm in c.members:
            out[nm] = {'corridor': c.idx,
                       'launch_idx': li.get(nm), 'target_idx': ti.get(nm),
                       'page': (sc.page.get(nm) if sc else ctx.tooth_layer[nm]),
                       'birth_b': bool(sc and nm in sc.birth_b),
                       'joiner': nm in getattr(c, 'joiners', ()),
                       'side_exit': nm in getattr(c, 'siders', ()),
                       # a side exiter's exit leg runs on its block's layer:
                       # a via where that is not the lane's page, another
                       # where it is not the berth's layer
                       'exit_leg_layer': getattr(c, 'leg_layer', {}).get(nm),
                       # the layer changes the lane's whole profile needs
                       # (tooth, page, tail stretches, exit leg, berth):
                       # the vias the plan implies for a page lane. A
                       # swimmer (page None) has no profile; its model
                       # stays SWIM_VIAS.
                       'changes': (len(c.layer_profile(nm)) - 1
                                   if sc and sc.page.get(nm) is not None
                                   and hasattr(c, 'req') else None),
                       # dives under EARLIER corridors' lanes (2 each)
                       'cross_vias': cross.get(nm, 0)}
    if ctx.M is not None:
        for d in out.values():
            for k in ('page', 'exit_leg_layer'):
                if d.get(k) is not None:
                    d[k] = other_layer(d[k])
    return out


def write_out(a, ctx, corridors, names, log):
    """Smooth, write the board with the Eco overlay, report."""
    pcb, byname, ends, kids = ctx.pcb, ctx.byname, ctx.ends, ctx.kids
    out_segs = {nm: c.out_segs[nm] for c in corridors for nm in c.members}
    out_vias = {nm: c.out_vias[nm] for c in corridors for nm in c.members}
    refused = sorted(nm for c in corridors for nm in c.refused)
    if refused:
        log(f'\nREFUSED nets (left open): {refused}')
    if refused and a.out != os.devnull:
        import json as _json
        rp = a.out + '_refusals.json'

        def _real(info):
            # the braid may have run on the board turned over (setup):
            # the report is read in the board's own frame
            if ctx.M is None or not info:
                return info
            d = dict(info)
            for k in ('tooth', 'berth'):
                if d.get(k) is not None:
                    d[k] = list(ctx.M(*d[k]))
            for k in ('page', 'tooth_layer', 'dest_layer'):
                if d.get(k) is not None:
                    d[k] = other_layer(d[k])
            return d
        with open(rp, 'w') as _f:
            _json.dump({nm: _real(ctx.refusal_info.get(nm, {}))
                        for nm in refused}, _f, indent=1,
                       sort_keys=True)
        log(f'refusal reasons -> {rp}')
    # deferred berth trim: each successfully-routed net's FINAL lane
    # decides its joint; a refused net's stub stays whole
    for nm in names:
        if out_segs.get(nm) and nm not in refused:
            note_joint(ctx, nm, out_segs[nm])

    # ---- repo octolinear smoothing (#536): collapse the distributed 45
    # nudges into single elbows, clearance-validated against ALL copper.
    # Only the BRAID's copper is a candidate (keep_input_copper): the
    # fanout's stubs are the braid's input and stay as they came, so the
    # tooth a lane was routed from stays on copper -- the smoother once
    # re-cut a stub's corner into a diagonal and left the tooth mark
    # (and the join the plan drew) hanging in free space.
    smoothed = False
    final_segs = {}
    from pcb_modification import smooth_octolinear_chains
    pre_len = {nm: sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
                       for s in out_segs[nm]) for nm in names}
    if os.environ.get('MEM_TRACE') == '1':
        import resource as _res
        import subprocess as _sp
        import tracemalloc as _tm

        def _vm():
            r = _sp.run(['vmmap', '--summary', str(os.getpid())],
                        capture_output=True, text=True).stdout
            return {l.split()[0] if not l.startswith('MALLOC') else ' '.join(l.split()[:2]): l
                    for l in r.splitlines()
                    if l.startswith(('MALLOC', 'VM_ALLOCATE', 'TOTAL', '__DATA', 'mapped', 'IOAccel', 'Stack', '__TEXT', 'shared'))}
        _cur = int(_sp.run(['ps', '-o', 'rss=', '-p', str(os.getpid())],
                           capture_output=True, text=True).stdout or 0) // 1024
        log(f'      mem before smoother: rss {_cur} MB, peak '
            f'{_res.getrusage(_res.RUSAGE_SELF).ru_maxrss / 1048576:.0f} MB')
        _vm0 = _vm()
        for _k, _l in sorted(_vm0.items()):
            log('        vmmap before: ' + _l)
        _tm.start(1)
    _res_list = [{'new_segments': list(out_segs[nm])} for nm in names]
    _n, _nets, _rm, _addl, stt = smooth_octolinear_chains(
        _res_list, pcb, kids, clearance=0.1, keep_input_copper=True)
    if PACK_MODE:
        # PACK (README TODO 8) at write time: every corridor's smoothed
        # lanes packed into rivers (pack.py, opt-in: BRAID_PACK=1)
        import pack as pk
        _prof = None
        if os.environ.get('BRAID_PACK_PROFILE'):
            import cProfile
            _prof = cProfile.Profile()
            _prof.enable()
        for c in corridors:
            for k, nm in enumerate(names):
                if nm in c.members:
                    c.out_segs[nm] = list(_res_list[k]['new_segments'])
            pk.pack_corridor(c, log)
        if _prof is not None:
            _prof.disable()
            _prof.dump_stats(os.environ['BRAID_PACK_PROFILE'])
        # the pack moves vias: the dicts read at the top are stale
        out_segs = {nm: c.out_segs[nm] for c in corridors for nm in c.members}
        out_vias = {nm: c.out_vias[nm] for c in corridors for nm in c.members}
    for nm in names:
        nid, _ = byname[nm]
        final_segs[nm] = [s for s in pcb.segments if s.net_id == nid]
    post_len = {nm: sum(math.hypot(s.end_x - s.start_x,
                                   s.end_y - s.start_y)
                        for s in final_segs[nm]) for nm in names}
    if os.environ.get('MEM_TRACE') == '1':
        _tc, _tp = _tm.get_traced_memory()
        _snap = _tm.take_snapshot()
        _tm.stop()
        _cur = int(_sp.run(['ps', '-o', 'rss=', '-p', str(os.getpid())],
                           capture_output=True, text=True).stdout or 0) // 1024
        log(f'      mem after smoother: rss {_cur} MB, peak '
            f'{_res.getrusage(_res.RUSAGE_SELF).ru_maxrss / 1048576:.0f} MB; '
            f'python traced in the smoother: current {_tc / 1048576:.0f} MB, peak {_tp / 1048576:.0f} MB')
        for _st in _snap.statistics('lineno')[:8]:
            _fr = _st.traceback[0]
            log(f'        still held: {_st.size / 1048576:6.1f} MB {_st.count:8d}  {_fr.filename.split("/")[-1]}:{_fr.lineno}')
        for _k, _l in sorted(_vm().items()):
            log('        vmmap after:  ' + _l)
    log(f'\nsmooth_octolinear_chains (#536): '
        f'{stt.get("spans", 0)} spans on {_nets} nets, '
        f'-{stt.get("saved_mm", 0):.2f} mm; segments '
        f'{sum(len(s) for s in out_segs.values())} -> '
        f'{sum(len(s) for s in final_segs.values())}; length '
        f'{sum(pre_len.values()):.2f} -> '
        f'{sum(post_len.values()):.2f} mm')
    smoothed = True

    # ---- write board: the copper back in the board's own frame (the
    # braid may have run on the board turned over, see setup)
    if ctx.M is not None:
        M, OL = ctx.M, other_layer
    else:
        M, OL = (lambda x, y: (x, y)), (lambda L: L)
    txt = open(a.board, encoding='utf-8').read()
    n_trim = sum(len(v) for v in ctx.trim_spans.values())
    if n_trim:
        log(f'berth stub trim: {n_trim} bypassed segment(s) removed '
            f'({", ".join(sorted(nm for nm, v in ctx.trim_spans.items() if v))})')
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
        emit = {nm: [(M(s.start_x, s.start_y), M(s.end_x, s.end_y),
                      OL(s.layer), s.width) for s in final_segs[nm]]
                for nm in names}
    for nm in names:
        nid, _ = byname[nm]
        for (p, q, layer, w) in emit[nm]:
            add.append(f'  (segment (start {p[0]:.4f} {p[1]:.4f}) '
                       f'(end {q[0]:.4f} {q[1]:.4f}) (width {w}) '
                       f'(layer "{layer}") (net {nid}))\n')
        for v in out_vias[nm]:
            vx, vy = M(v.x, v.y)
            add.append(f'  (via (at {vx:.4f} {vy:.4f}) (size {VIA_SIZE}) '
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
        p, q = M(*p), M(*q)
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

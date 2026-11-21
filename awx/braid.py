#!/usr/bin/env python3
"""braid.py -- a bench's reading, its obstacles, and the routers that lay one
lane or one pair in its band.

`setup` reads a bench into a context: the board (one of the other pair
chirality turned over in memory), the run's nets with each differential
pair one member, each net's two ends -- its tooth at the source and its
stub end at the destination, as the plan sidecar beside the board gives
them or as they are read off its copper -- with their layers and escape
directions, and each net's taut path between them round the static copper.

`build_obstacles` is the static-copper model of one net on one layer: every
foreign pad, segment and via, inflated by the clearance and half a track.
The plan's moves are priced against it and the taut paths relaxed against it.

`Corridor.route_lane` and `Corridor.route_pair_lane` lay one lane, or one
pair, with the real router (connect.py) inside the band its plan gives it,
against the lines and via sites reserved for the lanes not laid yet. The
whole route's lanes borrow them as plain functions (whole_ctx.PlanLanes) and
route_lanes.py drives them; `_route_pairs_planned_in_order` is the pairs'
step, each pair in its planned band and then free of it.

Other modules also take from here the ends read off copper (`endpoints`), a
ring's wrap spine (`ring_spine`), the run's routing config (`run_config`)
and the design constants as the rules give them (rules.py).

Inputs: a board whose SOURCE and DESTINATION arrays are both fanned out,
the nets to route, and the destination's reference. Everything else is
read off the board's geometry.
"""

import math
import re
import os
import awx_settings
import time as _time
import sys
from collections import Counter

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb, Segment  # noqa: E402
import topo_strings as ts  # noqa: E402
import connect as cn  # noqa: E402
import corridor as cr  # noqa: E402
import detect_buses as db  # noqa: E402
import escape_moves as em  # noqa: E402
from select_moves import pair_chirality  # noqa: E402
from bga_fanout.flip_frame import to_front_frame, other_layer, mirror_axis  # noqa: E402
from flow_frame import dest_refs  # noqa: E402  DEST naming one part or several ('U4+U3')
from coherent_nets import block_net_name  # noqa: E402  a block's net name as the parser names it

import rules as _rules  # noqa: E402  ONE source for every design rule

# ONE SOURCE: rules.py. A stage installs them (rules.install_defaults);
# they start as rules.active()'s -- a run's supplied rules, else the
# literals they have always been -- see rules.py, "USING IT".
TRACK = ts.TRACK         # ONE source: topo_strings
CLEAR = _rules.active().hug
                         # 0.1 spec + 5um so hugs don't sit exactly at 0.1
SPEC_CLEARANCE = _rules.active().clearance
                         # the spec itself: what the fanout lays at, what the
                         # checks grade at, and what the output PROJECT records (CLEAR
                         # is the router's private margin over it, not a rule)
VIA_SIZE = _rules.active().via_size
GRID = _rules.active().grid        # the routing grid (rules.GRID)
VIA_DRILL = _rules.active().via_drill

MINP = _rules.active().exit_pitch   # lane pitch floor at the exits
LPITCH = _rules.active().lane_pitch # pitch of a side-join / side-exit block
SWIM_TUBE = 1.2                # ribbon swimmer's band half-width: it
                               # WEAVES through the page lattice, so
                               # the neighbour-pinch band is wrong for
                               # it -- its same-layer neighbours are
                               # the very lanes it must cross (K11: a
                               # thin fragmented thread, refused; the
                               # obstacle map and the virtual copper
                               # are the law inside the tube)
VIA_NEED = VIA_SIZE / 2 + CLEAR + TRACK / 2 + 0.03   # a via's room to a
                               # neighbouring track centre, plus a cell
LANE_MIN = TRACK + CLEAR + 0.02    # the least centreline pitch two lanes are PLANNED at (ring floor, block squeeze, a leg's room)
# BRAID_EXACT_PAGES, and why it is read HERE (2026-09-15, session 13): a
# pages-first plan sets `schedule.EXACT_PAGES = 1` by assignment, AFTER
# schedule has read its own env -- so `BRAID_EXACT_PAGES=0` could not turn
# the rule off on the only plans that have it, and the rule has never been
# A/B'd. It is the other half of the same K51 finding: with the marker's two
# rules off the board routes 98 with NOTHING open, with EXACT_PAGES alone it
# routes 98 and leaves SDQ11 open. None = the plan decides (today's
# behaviour, byte-identical); '0' forces it off; anything else forces it on.
EXACT_PAGES_ENV = awx_settings.get('BRAID_EXACT_PAGES')
import pairs as _pairs  # noqa: E402  the bus's differential pairs as members (#622, 2026-09-20)


def _prime_highs_threads():
    """Pin HiGHS to ONE thread for the life of the process, at IMPORT.

    WHY THIS IS NOT THE OBVIOUS `setOptionValue('threads', 1)`: HiGHS
    builds ONE GLOBAL task scheduler at the first `run()` in a process and
    silently ignores every later `threads` setting, and the solves this
    chain makes (schedule.exact_pages) go through plain
    `scipy.optimize.milp`, which has no threads option at all. Whichever
    runs FIRST fixes the pool; unpinned, HiGHS runs hardware_concurrency
    threads. In a container `hardware_concurrency` reports the HOST's
    cores, which vary across a cloud fleet -- machine-dependent behaviour,
    exactly what the no-clocks rule exists to forbid, arriving through a
    thread pool instead of a timer.
    Running a trivial LP here claims the scheduler before any other call
    site can, so the pin holds for every solve on every path.
    """
    try:
        import scipy.optimize._highspy._core as hs_core
    except Exception:
        return False
    if not hasattr(hs_core, '_Highs'):
        return False
    try:
        lp = hs_core.HighsLp()
        lp.num_col_ = 1
        lp.num_row_ = 0
        lp.a_matrix_.num_col_ = 1
        lp.a_matrix_.num_row_ = 0
        lp.a_matrix_.format_ = hs_core.MatrixFormat.kColwise
        lp.a_matrix_.start_ = np.zeros(2, dtype=np.int32)
        lp.a_matrix_.index_ = np.zeros(0, dtype=np.int32)
        lp.a_matrix_.value_ = np.zeros(0, dtype=float)
        lp.col_cost_ = np.zeros(1)
        lp.col_lower_ = np.zeros(1)
        lp.col_upper_ = np.ones(1)
        lp.row_lower_ = np.zeros(0)
        lp.row_upper_ = np.zeros(0)
        h = hs_core._Highs()
        h.setOptionValue('output_flag', False)
        h.setOptionValue('threads', 1)
        h.passModel(lp)
        h.run()
        return True
    except Exception:
        return False


HIGHS_PRIMED = _prime_highs_threads()


def _on_board(pcb, x, y, inset):
    """(x, y) inside the board outline by at least `inset` -- the bounding
    box, and the outline polygon when the board has one."""
    bi = pcb.board_info
    bb = bi.board_bounds
    if bb is None:
        return True
    if not (bb[0] + inset <= x <= bb[2] - inset and bb[1] + inset <= y <= bb[3] - inset):
        return False
    poly = list(getattr(bi, 'board_outline', None) or [])
    if len(poly) < 3:
        return True
    inside = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
        dx, dy = x2 - x1, y2 - y1
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 < 1e-12 else max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / l2))
        if math.hypot(x - x1 - t * dx, y - y1 - t * dy) < inset:
            return False
    return inside


def sweep_round(path, cen):
    """the angle `path` turns through round the point `cen` (radians, counterclockwise positive): a ring's wrap
    direction is the sign of its lanes' sum"""
    a = [math.atan2(y - cen[1], x - cen[0]) for x, y in path]
    return sum((v - u + math.pi) % (2 * math.pi) - math.pi for u, v in zip(a, a[1:]))


def ring_spine(dpads, handoffs, stubs, start, ccw, arrive, tails, wrap_side=False):
    """(core, spine) of a RING: the wrap spine round the destination's pads `dpads` from `start` on the trunk's
    handoff line, `ccw` its direction, through the lanes' berth `stubs` and outside `tails` (the head-on lanes' copper
    it must pass), `arrive` the trunk's direction at its end -- extended back past the lanes' `handoffs` and on past
    their stubs. The braid's branches ride it, and the whole route's rings where the braid made none (whole_ctx)"""
    spn = cr.build_wrap_spine(dpads, list(stubs), [start], ccw, arrive, LPITCH, hull_extra=tails, wrap_side=wrap_side)
    P0, d0 = spn.P[0], spn.d[0]
    Pn, dn_ = spn.P[-1], spn.d[-1]
    back = max([0.3] + [-((t[0] - P0[0]) * d0[0] + (t[1] - P0[1]) * d0[1]) + 0.3 for t in handoffs])
    fwd = max([0.3] + [((t[0] - Pn[0]) * dn_[0] + (t[1] - Pn[1]) * dn_[1]) + 0.3 for t in stubs])
    return spn, spn.extend(back, fwd)


END_KEEP = TRACK + CLEAR + 0.05    # a virtual stamp keeps this off a free end


def clip_round_ends(pieces, ends, keep_r=None):
    """`pieces` [(p, q, layer)] with the stretch within keep_r of any
    of `ends` cut out. A virtual stamp must not cover another net's
    FREE END: the lane it predicts will dodge that copper when it is
    routed, but stamped over a tooth or a stub end it seals the end's
    owner in before it starts (K35 SA5: corridor 1's SA9 head over
    SA5's tooth, a one-cell pocket). Applied to the corridor's OWN
    lanes' stamps it freed K35's SA5/SA6 (exit legs of one stub row
    each landing a foot 0.05 mm from the next stub end) but cost K41
    three more open nets (2026-09-06), so it stays a reservation rule.

    keep_r defaults to END_KEEP READ AT CALL TIME, not captured in the
    signature: a default argument is bound at def time, which rules.install
    (a module attribute write) cannot reach -- so on a board with a wider
    clearance this one site would have kept the bench's 0.282."""
    if keep_r is None:
        keep_r = END_KEEP
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
# (the memo is asked from the ends' worker threads too, whole_ends' pool: the boards' list and its eviction under a
# lock, the eviction over a snapshot of the keys -- walked live, another thread's insert broke it, "dictionary changed
# size during iteration")
import threading as _threading
_OBS_LOCK = _threading.RLock()


def _obs_board_of(bkey):
    """The board identity inside a memo key: everything but the excluded
    nets and the layer."""
    return bkey[:3] + bkey[5:9]      # a derived key carries the net id after


def _obs_remember(bkey):
    b = _obs_board_of(bkey)
    with _OBS_LOCK:
        if b in _OBS_BOARDS:
            return
        _OBS_BOARDS.append(b)
        if len(_OBS_BOARDS) > _OBS_KEEP_BOARDS:
            dead = set(_OBS_BOARDS[:-_OBS_KEEP_BOARDS])
            del _OBS_BOARDS[:-_OBS_KEEP_BOARDS]
            for k in [k for k in list(_OBS_MEMO) if _obs_board_of(k) in dead]:
                _OBS_MEMO.pop(k, None)


def forget_obstacles():
    """Drop every memoised obstacle model (build_obstacles). A model is a
    pure function of its key, so one asked for again is rebuilt; a step
    that is done with its models and goes on to a large solve frees their
    memory for it (joint_escape.plan_array: ~70 MB of zynq U1's)."""
    with _OBS_LOCK:
        _OBS_MEMO.clear()
        del _OBS_BOARDS[:]


def _source_stamp(pcb):
    """(path, mtime, size) of the board file `pcb` was read from, as it stood when first asked of this board -- None
    when it has none on disk. Read ONCE per board: build_obstacles is asked thousands of times a fanout (6,329 on one
    plan), and a stat and an existence test each time were 5 s of it. The board in memory is the file as it was read,
    so the stamp it was first given names it as long as it lives; a board read again after the file is rewritten is
    another object, with a stamp of its own"""
    src = getattr(pcb, 'source_path', None)
    if not src:
        return None
    got = getattr(pcb, '_obs_stamp', None)
    if got is not None and got[0] == src:
        return got[1]
    stamp = None
    if os.path.exists(src):
        st_ = os.stat(src)
        stamp = (os.path.abspath(src), st_.st_mtime_ns, st_.st_size)
    try:
        pcb._obs_stamp = (src, stamp)
    except AttributeError:
        pass
    return stamp


def build_obstacles(pcb, nid, kids, layer, margin=None, skip_refs=frozenset()):
    """A static-copper model for one net on one layer: every foreign
    pad as KiCad draws it (a rect or an oval as capsules, _build_obstacles;
    others a disc), every foreign segment as a capsule, every foreign
    via as a disc, all inflated by clearance + half a track. The PLAN
    prices its candidate moves against it and the taut paths are relaxed
    against it; the lanes' copper is routed against
    the router's own model and never consults this one. Memoised per
    (board file as on disk, net, excluded nets, layer): the plan loop
    parses the same board several times per round and rebuilt the same
    model each time (531 builds at K15). `margin` inflates every item instead of the braid's clearance + half its
    lane track (the joint escape plans at the fan track and clearance it lays at); `skip_refs` are footprints left
    out (the movable passives the cap step moves off the fanout)."""
    # ONE BASE per (board, layer, excluded segment nets), the net's own
    # model DERIVED from it (Obstacles.exclude): the 35 nets of a plan
    # differ only by their own pads and vias, and a full build per net
    # was 111 s of a 250 s K35 fanout stage (2026-09-06 profile). The
    # base excludes the segments of `kids` as before (own included when
    # own is among them); the derivation removes the net's own pads,
    # vias and segments. Same items, same order: bit-identical answers.
    kids = frozenset(kids)
    base_kids = kids if (nid in kids and len(kids) > 1) else kids - {nid}
    stamp = _source_stamp(pcb)
    bkey = dkey = None
    if stamp is not None:
        bkey = stamp + (base_kids, layer, len(pcb.segments), len(pcb.vias),
                getattr(pcb, 'frame_axis', None),   # the board turned over, or rotated,
                getattr(pcb, 'frame_rotation', None),   # is another board
                margin, frozenset(skip_refs))
        dkey = bkey + (nid,)
        hit = _OBS_MEMO.get(dkey)
        if hit is not None:
            return hit
    base = _OBS_MEMO.get(bkey) if bkey is not None else None
    if base is None and bkey is not None and base_kids:
        # a base without some nets' segments is the base without none, less those segments (the same items in the
        # same order): DERIVED from it, not built -- a full build per excluded set was a whole board per diff pair
        # per layer (pair_exit_clear's {leg, partner}: zynq U1's joint plan, 19 pairs on three layers, 57 bases,
        # 550 MB)
        rkey = bkey[:3] + (frozenset(),) + bkey[4:]
        root = _OBS_MEMO.get(rkey)
        if root is None:
            root = _build_obstacles(pcb, frozenset(), layer, margin=margin, skip_refs=skip_refs)
            _obs_remember(rkey)
            _OBS_MEMO[rkey] = root
        base = root.exclude(base_kids, where=lambda it: str(it[3]).startswith(('seg:', GFX)))
        _OBS_MEMO[bkey] = base
    if base is None:
        base = _build_obstacles(pcb, base_kids, layer, margin=margin, skip_refs=skip_refs)
        if bkey is not None:
            _obs_remember(bkey)
            _OBS_MEMO[bkey] = base
    obs = base.exclude({nid})
    if dkey is not None:
        _OBS_MEMO[dkey] = obs
    return obs


# a rect pad is modelled by capsules -- its rectangle grown by the margin, exactly (a roundrect as its square-cornered
# rectangle: never smaller than its copper) -- while half its short side is within this many margins: past it, the
# long axis's capsule and the edges' leave the pad's deep corners uncovered (exact to (sqrt 2 + 1) / (sqrt 2 - 1))
RECT_EXACT = 5.5
GFX = 'gfx:'          # a filled copper graphic's outline capsules are named GFX + net ('seg:' + net a track's)


def _build_obstacles(pcb, kids, layer, margin=None, skip_refs=frozenset()):
    """Every pad on `layer` (drilled: on both), every segment on it
    whose net is not in `kids`, every via -- each tagged with its net,
    so a per-net model is a derivation (build_obstacles)."""
    obs = ts.Obstacles()
    m = CLEAR + TRACK / 2 if margin is None else margin
    for ref, fp in pcb.footprints.items():
        if ref in skip_refs:
            continue
        for p in fp.pads:
            on_layer = any(L == layer or L == '*.Cu' for L in p.layers)     # (*.Mask, *.Paste: no copper)
            if p.drill and p.drill > 0:
                on_layer = True
            if not on_layer:
                continue
            nm_ = f'{ref}.{p.pad_number}'
            if p.pad_type == 'np_thru_hole' and p.drill:
                obs.add_disc(p.global_x, p.global_y, p.drill / 2 + m, nm_, net=p.net_id)
                continue
            hx, hy = p.size_x / 2, p.size_y / 2
            lo, hi = min(hx, hy), max(hx, hy)
            ax = (hi - lo, 0.0) if hx >= hy else (0.0, hi - lo)      # the long axis, from the centre
            a_ = (p.global_x - ax[0], p.global_y - ax[1])
            b_ = (p.global_x + ax[0], p.global_y + ax[1])
            tilted = abs(getattr(p, 'rect_rotation', 0.0) or 0.0) > 1e-6
            if p.shape == 'circle':
                obs.add_disc(p.global_x, p.global_y, hi + m, nm_, net=p.net_id)
            elif p.shape == 'oval' and not tilted:
                # an oval pad IS a stadium: the capsule of its long axis
                obs.add_cap(a_, b_, lo + m, nm_, net=p.net_id)
            elif p.shape in ('rect', 'roundrect') and not tilted:
                # a rectangle grown by the margin, exactly: its long axis's capsule (every point of the pad) and a
                # capsule of the margin along each edge (the corners' rounding) -- exact while half its short side
                # is within RECT_EXACT margins. The circumscribed disc it replaces refused lanes that clear a
                # passive's pads (a B lane 0.19 / 0.22 mm from two 0402 pads' edges, 0.15 asked)
                x0, y0, x1, y1 = p.global_x - hx, p.global_y - hy, p.global_x + hx, p.global_y + hy
                if lo <= RECT_EXACT * m:
                    obs.add_cap(a_, b_, lo + m, nm_, net=p.net_id)
                else:
                    # ...and past it, the rectangle's inside as lines along its long side, two margins apart at most
                    # (each line's capsule of the margin meets the next), with the edges' capsules the rectangle grown
                    # by the margin, exactly too. The circumscribed disc they replace stood (sqrt 2 - 1) half a side
                    # past a square pad's faces at their middles: a QFN's exposed pad refused the middle pads of its
                    # rows their only escape, straight out (escape_moves: a row part)
                    k_ = max(1, math.ceil(lo / m))
                    for i_ in range(1, k_):
                        t_ = -lo + 2 * lo * i_ / k_
                        if hx >= hy:
                            obs.add_cap((x0, p.global_y + t_), (x1, p.global_y + t_), m, nm_, net=p.net_id)
                        else:
                            obs.add_cap((p.global_x + t_, y0), (p.global_x + t_, y1), m, nm_, net=p.net_id)
                for e0, e1 in (((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))):
                    obs.add_cap(e0, e1, m, nm_, net=p.net_id)
            else:
                obs.add_disc(p.global_x, p.global_y, math.hypot(hx, hy) + m, nm_, net=p.net_id)
    for s in pcb.segments:
        if s.net_id in kids or s.layer != layer:
            continue
        # (a FILLED copper graphic -- a footprint's land drawn as a polygon -- is copper AREA, as a pad is, not a track:
        # its outline named apart, GFX, one name per shape, the taut strings see it as they see a pad drawn as
        # capsules, the one disc round them (taut_fast.pad_discs) -- where a track ('seg:') crossed is a dive they
        # pass. Crossed as tracks, X2.1's chamfered land on the zynq board took four strings in through its edges,
        # which pushed them only where they ran along them, and folded each, 30% longer every block, to 846 mm for
        # a 32 mm lane)
        name = f'seg:{s.net_id}'
        if getattr(s, 'graphic', False) and getattr(s, 'graphic_filled', False):
            ring = getattr(s, 'graphic_ring', None) or ((s.start_x, s.start_y),)
            name = f'{GFX}{s.net_id}:{ring[0][0]:.4f},{ring[0][1]:.4f}'
        obs.add_cap((s.start_x, s.start_y), (s.end_x, s.end_y),
                    s.width / 2 + m, name, net=s.net_id)
    for v in pcb.vias:
        obs.add_disc(v.x, v.y, v.size / 2 + m, f'via:{v.net_id}',
                     net=v.net_id)
    obs.build()
    return obs


def _snapper(tol=0.006):
    """A node key that treats two points within `tol` of each other as
    ONE node. Rounding alone does not: the human's board writes a shared
    vertex as 125.916519 in one segment and 125.9165 in the next, which
    round(3) sends to 125.917 and 125.916 -- two keys, a mid-lane vertex
    read as two free ends, and the braid's tooth for that net 1.8 mm
    inside the source array (SDQ6, human bench K41). Buckets of 2*tol
    with a look-up of the eight neighbouring buckets; the first point
    seen in a neighbourhood names the node."""
    seen = {}
    q = 2 * tol

    def key(x, y):
        bx, by_ = int(math.floor(x / q)), int(math.floor(y / q))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for k in seen.get((bx + dx, by_ + dy), ()):
                    if abs(k[0] - x) <= tol and abs(k[1] - y) <= tol:
                        return k
        k = (round(x, 3), round(y, 3))
        seen.setdefault((bx, by_), []).append(k)
        return k
    return key


def _owner(pt, segs, pads):
    """The component whose pad the net's copper connects `pt` to.

    Segment endpoints are graph nodes (vias join layers at one point,
    so a 2-D node graph is enough); a pad is reached when a node lies
    within its copper. Falls back to nearest-pad only when the walk
    reaches no pad at all (an isolated fragment)."""
    key = _snapper()
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
        for (x, y) in seen:
            if _pairs.in_pad(x, y, p, 0.02):
                return p.component_ref
    p = min(pads, key=lambda q: ts.d2((q.global_x, q.global_y), pt))
    return p.component_ref


def _chain_runs(pieces, L, tol=1e-4):
    """The reservation pieces on layer L chained into runs -- polylines of
    pieces sharing an end, in either direction."""
    runs = []
    for p, q, L_ in pieces:
        if L_ != L:
            continue
        p = (float(p[0]), float(p[1]))
        q = (float(q[0]), float(q[1]))
        for run in runs:
            if math.hypot(p[0] - run[-1][0], p[1] - run[-1][1]) < tol:
                run.append(q)
                break
            if math.hypot(q[0] - run[0][0], q[1] - run[0][1]) < tol:
                run.insert(0, p)
                break
        else:
            runs.append([p, q])
    merged = True
    while merged:
        merged = False
        for i, a in enumerate(runs):
            for j, b in enumerate(runs):
                if i != j and math.hypot(a[-1][0] - b[0][0], a[-1][1] - b[0][1]) < tol:
                    runs[i] = a + b[1:]
                    del runs[j]
                    merged = True
                    break
            if merged:
                break
    return runs


def _pair_legs(pieces, half):
    """A pair's two conductors about its reserved centreline: each layer's
    pieces chained into runs, each run cleared of its sub-20 um segments and
    offset to either side as ONE mitred polyline. Offset piece by piece, the
    legs broke at every bend (a gap outside, an overlap inside). Every layer
    its pieces are on: F.Cu and B.Cu, then an inner routing layer's."""
    out = []
    for L in ('F.Cu', 'B.Cu') + tuple(sorted({L_ for _p, _q, L_ in pieces} - {'F.Cu', 'B.Cu'})):
        for run in _chain_runs(pieces, L):
            run = _pairs._simplify(run)
            if len(run) < 2:
                continue
            for h in (half, -half):
                off = _offset_side(run, h)
                out.extend((a, b, L) for a, b in zip(off, off[1:]))
    return out


def _offset_side(run, h):
    """One conductor of a pair about the polyline `run`, offset by h (left
    positive), mitred -- with every centreline vertex the offset cannot
    follow on this side dropped, as the inner conductor of a real pair cuts
    a bend: where a piece is shorter than its mitres need, its offset runs
    BACKWARDS (HHa SCK: a 0.026 mm piece between two bends, offset by half a
    pair pitch, turned back 160 degrees)."""
    pts = list(run)
    while True:
        off = _pairs.offset_polyline(pts, h)
        if len(pts) <= 2:
            return off
        back = [i for i in range(len(pts) - 1)
                if (pts[i + 1][0] - pts[i][0]) * (off[i + 1][0] - off[i][0])
                + (pts[i + 1][1] - pts[i][1]) * (off[i + 1][1] - off[i][1]) <= 0]
        if not back:
            return off
        i = back[0]
        del pts[i + 1 if i + 1 < len(pts) - 1 else i]

def bare_lane(pcb, nm, byname):
    """a net with no track at all (a via in its pad at most) and pads on two parts: on the ESCAPE fanout's field
    (dogbone_arrays) a net whose balls are both outer balls left to leave on their own layer, or one whose ball found
    no gap for its dog-bone -- no stub end to read, and the escape fanout lays both of its ends from the pads
    (escape_plan.plan_state reads a lane by its pads; sdi_mipi_conv's SDI bus lost five of its 21 nets to the ladder,
    D0, D2 and D7 outer balls at both arrays)"""
    nid, net = byname[nm]
    return not any(s.net_id == nid for s in pcb.segments) and len({p.component_ref for p in net.pads}) >= 2


def endpoints(pcb, names, byname, dest_ref=None):
    """Where each net's corridor must start and finish.

    Default (dest_ref None): the source stub's free end, to the far
    PAD -- what the plan works from before the destination is fanned
    out. With dest_ref: both ends are fanned out and each net has a
    free end at both; the pair is attributed by walking the net's own
    copper (nearest-pad is wrong for a stub dragged across a field).
    A free end is a segment endpoint used once that lies inside no pad
    and no via barrel of the net. dest_ref may name several parts
    ('U4+U3', one source and several ends): each net's end is at the
    one it reaches, its third member that part."""
    ends = {}
    dset = set(dest_refs(dest_ref)) if dest_ref is not None else None
    for nm in names:
        nid, net = byname[nm]
        segs = [s for s in pcb.segments if s.net_id == nid]
        cnt = Counter()
        key = _snapper()
        # a node's degree counts the distinct places its segments lead to,
        # a via barrel of the net being ONE place: the human bench's clipped
        # stub for SDQM1 left its free end on the box line by two segments
        # ending 0.02 mm apart inside one via, the end read as a joint, and
        # the net's target fell back to its PAD inside the ball field
        # (routed round the south across the whole data bundle)
        barrels = [(v.x, v.y, v.size / 2) for v in pcb.vias if v.net_id == nid]

        def place(k):
            for i, (vx, vy, vr) in enumerate(barrels):
                if math.hypot(k[0] - vx, k[1] - vy) <= vr:
                    return ('via', i)
            return k
        nbr = {}
        for s in segs:
            ka, kb = key(s.start_x, s.start_y), key(s.end_x, s.end_y)
            if ka == kb:
                # a crumb shorter than the snap tolerance (5 um on the
                # human bench, the clipped remnant of a diagonal) counts
                # its one node twice and hides a free end: SBA2's tooth
                # read as no free end at U1, so its DU1 stub became the
                # tooth and the U1 pad the target -- a corridor of one,
                # routed last (6-7 vias)
                continue
            nbr.setdefault(ka, set()).add(place(kb))
            nbr.setdefault(kb, set()).add(place(ka))
        for k, v in nbr.items():
            cnt[k] = len(v)
        anchors = [(p.global_x, p.global_y, max(p.size_x, p.size_y) / 2)
                   for p in net.pads] + \
            [(v.x, v.y, v.size / 2) for v in pcb.vias if v.net_id == nid]
        free = [pt for pt, c in cnt.items() if c == 1 and
                all(math.hypot(pt[0] - ax, pt[1] - ay) > max(0.02, ar)
                    for (ax, ay, ar) in anchors)]
        if not free:
            # A STUB THAT ENDS IN A VIA (2026-09-22): a net re-laid from its
            # pad -- the re-escape, a tie via -- has a pad, a leg and a via,
            # and nothing else once its lane is stripped; every endpoint
            # sits in a pad or a barrel and the rule above sees no end
            # (the zynq K44 descent died in round 1 on DDR3_CS). The via IS
            # the stub's end: the lane leaves it on the other layer.
            pads_only = [(p.global_x, p.global_y, max(p.size_x, p.size_y) / 2)
                         for p in net.pads]
            free = [pt for pt, c in cnt.items() if c == 1 and
                    all(math.hypot(pt[0] - ax, pt[1] - ay) > max(0.02, ar)
                        for (ax, ay, ar) in pads_only)]
        assert free, (nm, 'no free stub end')
        if dest_ref is not None:
            at_dest = [pt for pt in free
                       if _owner(pt, segs, net.pads) in dset]
            at_src = [pt for pt in free if pt not in at_dest]
            if not at_dest or not at_src:
                # ONE end's stub ends in a via (the rule above, per end): the
                # human bench clips SDQM1's berth stub on the box line inside
                # the barrel of its last via, and the net's free tooth kept
                # the whole-net fallback from firing -- its target fell back
                # to the far PAD
                pads_only = [(p.global_x, p.global_y, max(p.size_x, p.size_y) / 2)
                             for p in net.pads]
                free_v = [pt for pt, c in cnt.items() if c == 1 and
                          all(math.hypot(pt[0] - ax, pt[1] - ay) > max(0.02, ar)
                              for (ax, ay, ar) in pads_only)]
                if not at_dest:
                    at_dest = [pt for pt in free_v if _owner(pt, segs, net.pads) in dset]
                if not at_src:
                    at_src = [pt for pt in free_v if pt not in at_dest
                              and _owner(pt, segs, net.pads) not in dset]
            if at_dest and at_src:
                src = max(at_src, key=lambda p: max(ts.d2(p, q)
                                                    for q in at_dest))
                tgt = max(at_dest, key=lambda q: ts.d2(src, q))
                ends[nm] = (src, tgt, _owner(tgt, segs, net.pads))
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
        # ...and a pad served UNDER another (a termination resistor on the
        # back side under a DDR ball, 0.06 mm further from the source than
        # the ball) is not the target: the ball on the ARRAY is
        for p in far:
            if p is not tgt and _pairs.under_pad(p, tgt, VIA_SIZE):
                tgt = p
                break
        ends[nm] = (src, (tgt.global_x, tgt.global_y), tgt.component_ref)
    return ends


def strip_net_items(txt, token, net_ids, net_names=()):
    """Remove every top-level (`token` ...) block -- 'segment' or 'via' --
    whose net ref matches, in EITHER dialect: numeric (net N) or quoted
    name (#749 lore: boards carry both). Paren-balanced."""
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
        m2 = block_net_name(block)
        if (m and int(m.group(1)) in net_ids) or \
                (m2 is not None and m2 in net_names):
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
        if _pairs.in_pad(pt[0], pt[1], p, 0.02):
            cu = [L for L in p.layers if L.endswith('.Cu') and L != '*.Cu']
            if len(cu) == 1:
                return cu[0]
    return default


def _end_dir(pcb, nid, pt, pads):
    """The direction the net's stub ESCAPES in at its free end `pt`:
    walk the copper from the free end to the pad it reaches, and take
    the longest segment of the free end's RUN -- the stub from its free
    end back while each turn stays under 60 degrees and the stub runs
    within 60 degrees of the way it leaves its free end -- pointed toward
    the free end. The stub's last segment is the wrong thing to read
    (the fanout ends many escapes with a 45-degree jog, which the run
    takes in), and pad-to-end is too (the escape runs half a pitch off
    its pad's row); the run is the escape itself, at whatever angle the
    array sits. The longest segment of the whole stub is wrong too: an
    escape down a channel and out of the array's face at a corner read
    as the channel's way where the channel was the longer, and a pair
    whose leg read so had no end connector ahead of its teeth. And a
    run held by its turns alone follows a stub drawn in a CURVE, no
    turn of it past 45 degrees, from its free end to its ball: a tooth
    that leaves the south face read as the west its longest segment
    runs at the ball -- to the frame a far-face tooth inside the
    array, which it refuses -- and teeth out of the facing face as
    the way of the channel behind them."""
    key = _snapper()
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
            inside = any(_pairs.in_pad(u[0], u[1], p, 0.02) for p in pads)
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

    def way(t):
        v_ = (t[1][0] - t[2][0], t[1][1] - t[2][1])
        h_ = math.hypot(*v_)
        return (v_[0] / h_, v_[1] / h_) if h_ > 1e-6 else None
    after = {}
    for t in order:
        after.setdefault(t[1], []).append(t)
    run = [order[0]]
    w0 = way(order[0])
    while True:
        nx = after.get(run[-1][2], ())
        if len(nx) != 1:
            break
        a, b = way(run[-1]), way(nx[0])
        if a is None or b is None or a[0] * b[0] + a[1] * b[1] < 0.5 or w0[0] * b[0] + w0[1] * b[1] < 0.5:
            break
        run.append(nx[0])
    s, near, far = max(run, key=lambda t: math.hypot(
        t[0].end_x - t[0].start_x, t[0].end_y - t[0].start_y))
    v = (near[0] - far[0], near[1] - far[1])
    h = math.hypot(*v)
    return (v[0] / h, v[1] / h) if h > 1e-6 else (1.0, 0.0)


def bundle_layer_of(tooth_layer):
    """The layer most teeth are born on -- what the taut paths relax
    against. It used to be F by name; a board turned over
    (every tooth on B) then dodged the wrong layer's copper."""
    n_b = sum(1 for L in tooth_layer.values() if L == 'B.Cu')
    return 'B.Cu' if 2 * n_b > len(tooth_layer) else 'F.Cu'


class Corridor:
    """The lane and pair routers, held as methods so that whatever holds a plan's lanes takes them as its own:
    whole_ctx.PlanLanes borrows them as plain functions. Of `self` they read ctx, log, spine, teeth, stubs,
    lane_xy, band_of, sched_cur and, where a whole-route plan is installed, _geo."""

    def route_lane(self, nm, virt, virt_vias=None, free=False, slack=0.0):
        """Route one lane tooth -> stub: ONE connect() search inside its
        band by default (`free`: in no band, the free swimmers' window,
        as route_lanes.lift_walls routes a lane again round a ball's way;
        `slack`: its band that much wider a side, as it parts a lane from
        a ball's way between two lanes).
        Returns (segments, vias) or None; partial copper is discarded."""
        ctx, sp = self.ctx, self.spine
        if nm in getattr(ctx, 'pairs', {}):
            return self.route_pair_lane(nm, virt, virt_vias)
        nid, _ = ctx.byname[nm]
        # a ribbon swimmer's search window must HOLD its tube: the
        # window is built from the straight lane_xy with `margin`, so
        # at the default 0.6 the SWIM_TUBE band was clipped to half --
        # the A* explored 12k cells and never saw the via diamonds a
        # lane's width away (K11 SDQ15)
        sc = getattr(self, 'sched_cur', None)
        swim = sc is not None and sc.page.get(nm) is None
        margin = SWIM_TUBE + 0.4 if swim else 0.6
        # FREE SWIMMERS (#622): route the whole swimmer class with no
        # band and a wide window
        # from the start. The BAND was the refusal: band-free search
        # measured 1-2 via paths (K28 SWE 1, SDQ0 2) where the
        # in-band model wove for more or refused, and the virtual
        # lines plus reserved diamonds still protect every lane routed
        # after. Pages stay banded: their rigidity is what makes them
        # cheap.
        free_swim = swim or free
        if free_swim:
            # window size is a completion constraint (measured K35: a
            # corridor-wide window closed 4 open -> 1 at +22 vias and
            # 2x time); the free swimmers get a 2 mm search margin
            margin = 2.0
            if getattr(self, '_swim_boost', False):
                margin = max(margin, 6.0)
        way = [(self.teeth[nm], ctx.tooth_layer[nm])]
        way.append((self.stubs[nm], ctx.dest_layer[nm]))
        band = None if free_swim else self.band_of(nm, slack=slack) if slack else self.band_of(nm)
        segs_all, vias_all = [], []
        added = 0
        # The lanes target the stub TIP. Finishing at any dest-stub
        # vertex (earliest same-net contact) was measured as a default:
        # K28 58 -> 54v but K21 +2 and K32 loses completion (early joins
        # reshape later lanes' worlds).
        virt = list(virt or [])
        # nothing planned may cover THIS lane's own tooth or berth: the other
        # lanes' reservations are clipped round its two ends for its own
        # search only (HHa: SODT1's line over SCK's berth at 0.000 mm, SA2's
        # tail 0.057 from SA4's -- SA4's backward search died in 1 iteration).
        # Clipped for every lane at once it cost K41 three nets (2026-09-06),
        # so the other lanes keep their reservations of each other's ends.
        virt = clip_round_ends(virt, [self.teeth[nm], self.stubs[nm]])
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
                # (a whole-route plan's single, refused at its width: again with its end stretches narrowed, as the
                # snap read them -- route_lane_necked)
                if getattr(self, '_geo', None):
                    return route_lane_necked(self, nm, nid, virt, virt_vias, band, margin)
                return None
            segs_o, vias_o = res
            ctx.pcb.segments.extend(segs_o)
            ctx.pcb.vias.extend(vias_o)
            added += len(segs_o)
            segs_all.extend(segs_o)
            vias_all.extend(vias_o)
        return segs_all, vias_all

    def _pair_band_slack(self):
        """How much wider than a single's a pair member's band is, a side:
        half the pair pitch (the outer leg's centreline) plus half a track
        and two grid cells, so the outer leg's cell at the edge of a leg
        strip -- sized for one lane -- is inside the band (measured K36:
        at half the pitch alone the berth-side pose failed on the outer
        leg's cell at every setback)."""
        return _pairs.pitch(TRACK) / 2 + TRACK / 2 + 2 * self.ctx.cfg.grid_step

    def route_pair_lane(self, nm, virt, virt_vias=None, slack=None, free=None):
        """A PAIR member's lane (pairs.py): the production pair
        router between the two teeth and the two berths, inside the
        member's band widened by half the pair pitch a side, against the
        same virtual copper a single sees. Returns (segments, vias) of
        both legs, appended to the board, or None."""
        ctx = self.ctx
        pn, nn = ctx.pairs[nm]
        pid, nid_n = ctx.byname[pn][0], ctx.byname[nn][0]
        (tp_, tn_), (sp_, sn_) = ctx.pair_ends[nm]
        # ...and a pair's four tips likewise, for its own search only
        virt = clip_round_ends(list(virt or []), [e for e in (tp_, tn_, sp_, sn_) if e is not None])
        sc = getattr(self, 'sched_cur', None)
        swim = sc is not None and sc.page.get(nm) is None
        margin = SWIM_TUBE + 0.4 if swim else 0.6
        if swim:
            margin = 2.0
            if getattr(self, '_swim_boost', False):
                margin = max(margin, 6.0)
        # the band widened by half the pair pitch a side (its legs: the
        # band's leg strips are sized for one lane)
        half = self._pair_band_slack()
        # FREE unless the caller says otherwise: a pair is searched like a free
        # swimmer -- no band, a 2 mm window margin -- with every unrouted
        # neighbour's reservation and every routed lane's copper still in
        # force. Measured K36 (2026-09-20): in its single-lane band the
        # pose router refused all three pairs -- the band's leg strips,
        # dive zones and slope pitches are sized for one track, and the
        # pair's centreline (half a pitch of extra clearance) found a
        # neck in each; free, the same search landed them at last call.
        free_pair = True if free is None else bool(free)
        if slack is not None:
            half = slack
        wpts = list(self.lane_xy[nm])
        if free_pair:
            band = None
            margin = max(margin, 2.0)
        else:
            band = None if swim else self.band_of(nm, slack=half)
            margin = max(margin, half + 0.6)
        rep = {}
        # a whole-route plan's END CONNECTORS (whole_snap: legs from the tips to the pose's own legs, pairs.end_legs):
        # given to the pair step as they are drawn, the pair router taking over at their handover points
        _ga = _gb = None
        _ends = ((getattr(self, '_geo', None) or {}).get('lanes', {}).get(nm) or {}).get('ends')
        if _ends:
            _given = []
            for e_ in _ends:
                legs_ = [[Segment(a_[0], a_[1], b_[0], b_[1], TRACK, e_['layer'], lid)
                          for a_, b_ in zip(pts, pts[1:]) if math.hypot(b_[0] - a_[0], b_[1] - a_[1]) > 1e-9]
                         for pts, lid in zip(e_['legs'], (pid, nid_n))]
                _given.append(([s_ for lg in legs_ for s_ in lg], [], tuple(e_['handover'][0]), tuple(e_['handover'][1]),
                               tuple(e_['heading']), e_['layer']))
            _ga, _gb = _given
        # ...and an opposite-hands pair's CROSSOVER (pairs.crossover), laid as drawn between its two spans
        _gx = ((getattr(self, '_geo', None) or {}).get('lanes', {}).get(nm) or {}).get('cross')
        if awx_settings.get('BRAID_PAIR_DEBUG') and virt_vias:
            xy_ = self.lane_xy.get(nm) or [self.teeth[nm], self.stubs[nm]]
            nv = [f'({vx:.2f},{vy:.2f})' for (vx, vy) in virt_vias
                  if min(math.hypot(vx - e[0], vy - e[1]) for e in (xy_[0], xy_[-1])) < 1.2]
            self.log(f'    pair {nm}: {len(virt_vias)} reserved via(s) in all, near its ends: {", ".join(nv) or "none"}')
        res = cn.connect_pair(ctx.pcb, pid, nid_n, tp_, tn_, ctx.tooth_layer[nm],
                              sp_, sn_, ctx.dest_layer[nm], ctx.cfg, band=band,
                              virtual=virt, margin=margin,
                              window_pts=wpts,
                              virtual_vias=virt_vias, gap=_pairs.GAP,
                              a_dir=ctx.tooth_dir.get(nm), b_dir=ctx.stub_dir.get(nm),
                              a_n_layer=ctx.pair_layers[nm][0], b_n_layer=ctx.pair_layers[nm][1],
                              report=rep, a_given=_ga, b_given=_gb, x_given=_gx)
        if res is None:
            # where the search died: the blocked frontier's extent, in mm,
            # with the lane's planned extent beside it
            bl = rep.get('blocked') or []
            if bl:
                g = ctx.cfg.grid_step
                xs = [c[0] * g for c in bl]
                ys = [c[1] * g for c in bl]
                ly = sorted({ctx.cfg.layers[c[2]] if c[2] < len(ctx.cfg.layers) else str(c[2]) for c in bl})
                self.log(f'    pair {nm} refused: frontier {len(bl)} cells in '
                         f'({min(xs):.2f},{min(ys):.2f})-({max(xs):.2f},{max(ys):.2f}) on {"/".join(ly)}'
                         + (f'; polarity' if rep.get('polarity') else ''))
            elif rep.get('polarity'):
                self.log(f'    pair {nm} refused: its crossing lead could not be routed')
            elif rep.get('intra'):
                self.log(f'    pair {nm} refused: the split could not be made clean ({rep["intra"]})')
            elif rep.get('empty'):
                self.log(f'    pair {nm} refused: the pose router stopped at the source (no copper)')
            else:
                self.log(f'    pair {nm} refused: no frontier reported (envelope split failed?)')
            if awx_settings.get('BRAID_PAIR_DEBUG'):
                # the refusal as a picture: the board's copper, the virtual
                # lines this search saw, the planned lane, the frontier
                try:
                    self._pair_debug_image(nm, virt, rep, tp_, tn_, sp_, sn_, virt_vias=virt_vias)
                except Exception as _e:   # noqa: BLE001 -- a picture, never the run
                    self.log(f'    (pair debug image failed: {_e})')
            return None
        segs_o, vias_o = res
        ctx.pcb.segments.extend(segs_o)
        ctx.pcb.vias.extend(vias_o)
        self.log(f'    pair {nm} landed: {len(segs_o)} segment(s), {len(vias_o)} via(s)'
                 + (' [swim]' if swim else (' [free]' if free_pair else f' [band +{half:.2f}]')))
        return segs_o, vias_o

    def _pair_debug_image(self, nm, virt, rep, tp_, tn_, sp_, sn_, virt_vias=None):
        from route_render import BoardRenderer
        from kicad_parser import Segment as _S
        ctx = self.ctx
        pts = [tp_, tn_, sp_, sn_] + list(self.lane_xy.get(nm, []))
        x0, x1 = min(p[0] for p in pts) - 1.5, max(p[0] for p in pts) + 1.5
        y0, y1 = min(p[1] for p in pts) - 1.5, max(p[1] for p in pts) + 1.5
        view = (x0, y0, x1, y1)
        r = BoardRenderer(ctx.pcb, size=1800, supersample=2, show_zones=False, view=view, layer_alpha=140)
        bl = rep.get('blocked') or []
        g = ctx.cfg.grid_step

        # the BAND the pair was searched in (its allowed cells: F.Cu green,
        # B.Cu magenta, both grey-blue) and the connector/approach pieces
        # (orange), so a refusal picture shows what boxed the pose search
        band_cells = {}
        try:
            band = self.band_of(nm, slack=self._pair_band_slack())
            xs = np.arange(x0, x1, g)
            ys = np.arange(y0, y1, g)
            for L in ctx.cfg.layers:
                ok = np.asarray(band(xs, ys, L), dtype=bool)
                bi, bj = np.nonzero(ok)
                band_cells[L] = set(zip(bi.tolist(), bj.tolist()))
        except Exception as ex:      # the picture must never fail the run
            self.log(f'    (band overlay skipped: {ex})')
        pieces = rep.get('pieces') or []
        piece_vias = rep.get('piece_vias') or []

        def ov(d, rr):
            if band_cells:
                Ls = list(band_cells)
                both = band_cells[Ls[0]] & band_cells[Ls[1]] if len(Ls) == 2 else set()
                for L, col in zip(Ls, ((0, 150, 60), (170, 40, 170))):
                    only = band_cells[L] - both
                    rr._draw_segments(d, [_S(x0 + i * g, y0 + j * g, x0 + i * g + 0.001, y0 + j * g, 0.02, 'F.Cu', 0)
                                          for i, j in list(only)[:60000]], color=col)
                rr._draw_segments(d, [_S(x0 + i * g, y0 + j * g, x0 + i * g + 0.001, y0 + j * g, 0.02, 'F.Cu', 0)
                                      for i, j in list(both)[:60000]], color=(90, 110, 160))
            rr._draw_segments(d, [_S(p[0], p[1], q[0], q[1], 0.08, L, 0) for (p, q, L) in virt],
                              color=(255, 200, 0))
            for (vx, vy) in (virt_vias or []):
                # a reserved (planned) via of a lane not routed yet: a yellow ring
                rr._draw_segments(d, [_S(vx - 0.12, vy, vx + 0.12, vy, 0.25, 'F.Cu', 0)], color=(255, 200, 0))
                rr._draw_segments(d, [_S(vx - 0.05, vy, vx + 0.05, vy, 0.1, 'F.Cu', 0)], color=(20, 30, 20))
            if pieces:
                rr._draw_segments(d, [_S(s.start_x, s.start_y, s.end_x, s.end_y, 0.10, s.layer, 0) for s in pieces],
                                  color=(255, 120, 0))
            for v in piece_vias:
                rr._draw_segments(d, [_S(v.x - 0.1, v.y, v.x + 0.1, v.y, 0.1, 'F.Cu', 0)], color=(255, 120, 0))
            lane = self.lane_xy.get(nm, [])
            rr._draw_segments(d, [_S(a[0], a[1], b[0], b[1], 0.05, 'F.Cu', 0) for a, b in zip(lane, lane[1:])],
                              color=(255, 255, 255))
            rr._draw_segments(d, [_S(c[0] * g, c[1] * g, c[0] * g + 0.001, c[1] * g, 0.03, 'F.Cu', 0)
                                  for c in bl[:20000]], color=(0, 255, 255))
        img = r.frame(segments=[s for s in ctx.pcb.segments if x0 - 1 < s.start_x < x1 + 1 and y0 - 1 < s.start_y < y1 + 1],
                      vias=[v for v in ctx.pcb.vias if x0 < v.x < x1 and y0 < v.y < y1], overlays=[ov],
                      label=f'pair {nm} refused: virtual lines (yellow), lane (white), band (green F / magenta B / blue both), pieces (orange), poses (cyan)')
        out = f'tmp/pairdbg_{nm}_{len(ctx.landed)}.png'
        os.makedirs('tmp', exist_ok=True)
        img.save(out)
        self.log(f'    pair debug image -> {out}')


class Ctx:
    pass


# THE PAIR IS PART OF THE PLAN (Andy, 2026-09-20). Every pair is a member
# of the plan, and each pair is routed FIRST in its own planned band,
# widened by the first of BRAID_PAIR_SLACKS that lands it, the other
# members' planned lanes reserved; then free in a window as the last
# resort. Its copper is protected and the singles are routed by the same
# plan around it.
# the pair step's reaches and widenings in the rules' units: PAIR PITCHES (a pair's own track + gap, PP) and LANE
# pitches (rules.PAIR_*: how many of each)
PP = _pairs.pitch(TRACK)


def pair_slacks():
    """BRAID_PAIR_SLACKS, read when the pairs are routed and not when braid is imported: the whole route routes its
    lanes with BRAID_PAIR_SLACKS=0 in the same process that planned them with the default"""
    return [float(v) * PP for v in awx_settings.get('BRAID_PAIR_SLACKS', '2,4').split(',') if v.strip()]


# PAIR_FANIN (mm, a count of lane pitches): the pair's fan-in round either of its ends (the planned pair step's
# fan-in rule)
PAIR_FANIN = _rules.PAIR_FANIN_LANES * LANE_MIN
# PAIR_APPROACH (mm, a count of pair pitches): how far in front of a pair's tips the others' planned
# lines are cut out of its reservation (the pair router's setback ladder reaches four pair pitches -- twice its
# four-spacing setback -- and this is half a pitch more)
PAIR_APPROACH = _rules.PAIR_APPROACH_PITCHES * PP


def _in_boxes(pt, boxes, grow=0.0):
    """Is pt inside any approach box ((origin, unit dir, half across, length)),
    grown by `grow`."""
    for (e, d, half, length) in boxes:
        ax, ay = pt[0] - e[0], pt[1] - e[1]
        along = ax * d[0] + ay * d[1]
        across = abs(ax * d[1] - ay * d[0])
        if -0.02 - grow <= along <= length + grow and across <= half + grow:
            return True
    return False


def _clip_boxes(pieces, boxes, step=0.02):
    """`pieces` [(p, q, layer)] with every stretch inside an approach box cut
    out; the parts outside are kept."""
    if not boxes:
        return list(pieces)
    out = []
    for (p, q, L) in pieces:
        dx, dy = q[0] - p[0], q[1] - p[1]
        n = max(4, int(math.hypot(dx, dy) / step))
        run = None
        for i in range(n + 1):
            t = i / n
            pt = (p[0] + dx * t, p[1] + dy * t)
            inside = _in_boxes(pt, boxes)
            if not inside and run is None:
                run = pt
            elif inside and run is not None:
                prev = (p[0] + dx * (i - 1) / n, p[1] + dy * (i - 1) / n)
                if math.hypot(prev[0] - run[0], prev[1] - run[1]) > 1e-6:
                    out.append((run, prev, L))
                run = None
        if run is not None and math.hypot(q[0] - run[0], q[1] - run[1]) > 1e-6:
            out.append((run, q, L))
    return out


def _route_pairs_planned_in_order(ctx, corridors, log, order, free=True, slacks=None):
    """The pairs of `order` routed in their planned bands, in that order; returns {pair: (segments, vias)}, the copper left on ctx.pcb.
    free=False: each pair in its planned band only, never the free rungs (route_lanes --plan tries those after every
    lane has had its band). slacks: the band's widenings tried, in mm (default pair_slacks())."""
    done = {}
    for nm in order:
        c = next(c for c in corridors if nm in c.members)
        M = c.members
        if True:
            t0 = _time.time()
            pn, nn = ctx.pairs[nm]
            pid, nid_n = ctx.byname[pn][0], ctx.byname[nn][0]
            others = [om for om in M if om != nm]
            # the others' planned lanes reserved: cut in front of the pair's
            # tips (the boxes below) and, by the second rule tried, left out
            # whole within the FAN-IN (PAIR_FANIN mm of either of the pair's
            # ends) -- the lanes there cross in front of the pair's teeth,
            # and a pair's pose needs room no single lane needs
            e_a, e_b = c.teeth[nm], c.stubs[nm]
            (tp_, tn_), (sp_, sn_) = ctx.pair_ends[nm]
            boxes = []
            # THE PAIR OWNS THE FIRST MILLIMETRE IN FRONT OF
            # ITS TIPS, and no more (the human's ends): the box in front of
            # either of the pair's ends (the connector's reach,
            # PAIR_APPROACH mm, out along the escape; the tips' separation
            # plus a track and a clearance across), where the others' pieces
            # are CUT instead of dropped whole within the fan-in.
            for e_, d_, (u_, w_) in ((e_a, ctx.tooth_dir.get(nm), (tp_, tn_)),
                                     (e_b, ctx.stub_dir.get(nm), (sp_, sn_))):
                if d_ is None:
                    continue
                sep_ = math.hypot(u_[0] - w_[0], u_[1] - w_[1])
                boxes.append((e_, d_, sep_ / 2 + TRACK + SPEC_CLEARANCE, PAIR_APPROACH))
            # (no exit stub is reserved for the others: the plan draws every lane's own exit, whole_ctx.install, so
            # their planned lines are the reservation there, and a synthetic stub along an escape a lane does not
            # take is an obstacle nothing lays)
            R = PAIR_FANIN

            def near_end(p, q):
                # by the piece's ENDPOINTS (measured: the segment's distance
                # freed more of the fan-in and cost H3 four vias, 84 -> 88)
                return (min(math.hypot(p[0] - e[0], p[1] - e[1]), math.hypot(q[0] - e[0], q[1] - e[1])) < R
                        for e in (e_a, e_b))
            # the FAN-IN rule: a piece with an end within PAIR_FANIN of either
            # of the pair's ends dropped whole
            fan = ([(p, q, L) for (p, q, L) in c.virtual_of(others) if not any(near_end(p, q))],
                   [v for v in c.virtual_vias_of(others)
                    if min(math.hypot(v[0] - e[0], v[1] - e[1]) for e in (e_a, e_b)) >= R])
            virt = _clip_boxes(c.virtual_of(others), boxes)
            vv = [v for v in c.virtual_vias_of(others) if not _in_boxes(v, boxes, VIA_SIZE / 2)]
            # the band tried with the approach boxes first, then with the
            # fan-in rule: a pair whose tips stand apart must converge over
            # whatever runs between them, which the boxes kept reserved --
            # zynq K44's DQS pairs (teeth 0.80 and 1.55 mm apart, DQ6 and
            # DQ0 between DQS0's) refused in their bands under the boxes
            # alone, were routed free of the plan first and walled DQ6
            rules = [(virt, vv), fan]
            ways = _pairs.pair_waypoints(ctx.pcb, pid, nid_n, ctx.src_ref.get(nm) or ctx.src_ref.get(pn), ctx.ends[nm][2])
            res = None
            how = ''
            if ways:
                (tp_, tn_), (sp_, sn_) = ctx.pair_ends[nm]
                res = _route_pair_legs(ctx, nm, pid, nid_n, (tp_, tn_), (sp_, sn_), ways, virt, log)
                if res is not None:
                    why = _pairs.intra_ok(res[0], res[1], pid, nid_n, TRACK, VIA_SIZE, SPEC_CLEARANCE)
                    if why is not None:
                        log(f'  pair {nm}: through its waypoints but not clean ({why})')
                        res = None
                    else:
                        ctx.pcb.segments.extend(res[0])
                        ctx.pcb.vias.extend(res[1])
                        how = f'through {", ".join(w[0].component_ref for w in ways)}'
            else:
                for ri, (virt_r, vv_r) in enumerate(rules):
                    for sl in (pair_slacks() if slacks is None else slacks):
                        res = c.route_pair_lane(nm, virt_r, vv_r, slack=sl, free=False)
                        if res is not None:
                            how = f'in its planned band +{sl:.1f} mm' + (' (the fan-in rule)' if ri else '')
                            break
                    if res is not None:
                        break
                if res is None and free:
                    res = c.route_pair_lane(nm, virt, vv, free=True)
                    if res is not None:
                        how = 'free (its band refused)'
                if res is None and free:
                    # the last resort: free of the plan's lanes altogether,
                    # none of them reserved, a 6 mm window -- the call that
                    # landed pairs refused in every planned attempt; the
                    # singles' plan was not made round this copper -- against
                    # a pair left open, which nothing pays for
                    (tp_, tn_), (sp_, sn_) = ctx.pair_ends[nm]
                    rep = {}
                    res = cn.connect_pair(ctx.pcb, pid, nid_n, tp_, tn_, ctx.tooth_layer[nm],
                                          sp_, sn_, ctx.dest_layer[nm], ctx.cfg, band=None, margin=6.0,
                                          virtual=None, gap=_pairs.GAP,
                                          a_dir=ctx.tooth_dir.get(nm), b_dir=ctx.stub_dir.get(nm),
                                          a_n_layer=ctx.pair_layers[nm][0], b_n_layer=ctx.pair_layers[nm][1],
                                          report=rep)
                    if res is not None:
                        why = _pairs.intra_ok(res[0], res[1], pid, nid_n, TRACK, VIA_SIZE, SPEC_CLEARANCE)
                        if why is not None:
                            res = None
                        else:
                            ctx.pcb.segments.extend(res[0])
                            ctx.pcb.vias.extend(res[1])
                            how = 'free of the plan (its band and its lane refused)'
            if res is None:
                log(f'  pair {nm}: NOT routed first ({_time.time() - t0:.1f}s) -- its corridor tries again in order')
                continue
            done[nm] = res
            log(f'  pair {nm} routed FIRST and protected {how}: {len(res[0])} segment(s), {len(res[1])} via(s) '
                f'({_time.time() - t0:.1f}s)')
    return done


def _route_pair_legs(ctx, base, pid, nid_n, teeth, berths, ways, virt, log):
    """A pair routed in LEGS through its waypoint parts: teeth -> the
    first part's pads, part -> part, the last part's pads -> the berths.
    A part's pads are pair ends like any other; the pair leaves them
    toward the next stop. Returns (segments, vias) of every leg, or None
    when any leg refuses (the earlier legs' copper is then withdrawn)."""
    stops = [(teeth, ctx.tooth_layer[base], ctx.tooth_dir.get(base), 'tooth')]
    for (pp, pn) in ways:
        stops.append((((pp.global_x, pp.global_y), (pn.global_x, pn.global_y)),
                      (pp.layers[0] if pp.layers and pp.layers[0].endswith('.Cu') else ctx.tooth_layer[base]),
                      None, pp.component_ref))
    stops.append((berths, ctx.dest_layer[base], ctx.stub_dir.get(base), 'berth'))
    all_segs, all_vias = [], []
    n_before = len(ctx.pcb.segments), len(ctx.pcb.vias)
    for (a_pts, a_layer, a_dir, a_name), (b_pts, b_layer, b_dir, b_name) in zip(stops, stops[1:]):
        am = _pairs.mid(*a_pts)
        bm = _pairs.mid(*b_pts)
        d_ab = _pairs._unit(am, bm)

        def normal_toward(pts, target):
            # a two-pad part's pads leave SQUARE to the part's axis (the
            # line through its two pads), on the side of the next stop:
            # a slanted leaving direction skews the approach and the
            # router's connector grazed the partner's pad by 0.03 mm
            ax_ = _pairs._unit(pts[0], pts[1])
            nrm = (-ax_[1], ax_[0])
            m_ = _pairs.mid(*pts)
            if (target[0] - m_[0]) * nrm[0] + (target[1] - m_[1]) * nrm[1] < 0:
                nrm = (-nrm[0], -nrm[1])
            return nrm
        ad = a_dir if a_dir is not None else normal_toward(a_pts, bm)
        bd = b_dir if b_dir is not None else normal_toward(b_pts, am)
        rep = {}
        res = cn.connect_pair(ctx.pcb, pid, nid_n, a_pts[0], a_pts[1], a_layer,
                              b_pts[0], b_pts[1], b_layer, ctx.cfg, band=None, margin=6.0,
                              virtual=virt or None, gap=_pairs.GAP, a_dir=ad, b_dir=bd,
                              a_n_layer=a_layer, b_n_layer=b_layer, report=rep)
        if res is None:
            log(f'    pair {base}: leg {a_name} -> {b_name} refused')
            del ctx.pcb.segments[n_before[0]:]
            del ctx.pcb.vias[n_before[1]:]
            return None
        segs_o, vias_o = res
        ctx.pcb.segments.extend(segs_o)       # the next leg routes against this one
        ctx.pcb.vias.extend(vias_o)
        all_segs += segs_o
        all_vias += vias_o
        log(f'    pair {base}: leg {a_name} -> {b_name}: {len(segs_o)} segment(s), {len(vias_o)} via(s)')
    del ctx.pcb.segments[n_before[0]:]        # the caller appends the whole
    del ctx.pcb.vias[n_before[1]:]
    return all_segs, all_vias


def pair_chirality_of(pcb, names, byname, dest):
    """The pair's chirality as the fanout's plan computes it
    (select_moves.pair_chirality on the same balls and the same array
    boxes): the run's nets' pad on `dest` against their pad on the source
    array, the boxes those arrays' pads span. (`dest` naming several
    parts: each net's pad on the one it reaches, and the first's box --
    the plan's own dgrid, fanout_from_plan.plan_state.)"""
    src, dst, refs = {}, {}, Counter()
    dests = dest_refs(dest)
    for nm in names:
        _nid, net = byname[nm]
        d = [p for p in net.pads if p.component_ref in dests]
        o = [p for p in net.pads if p.component_ref not in dests]
        if d:
            dst[nm] = (d[0].global_x, d[0].global_y)
        if o:
            src[nm] = (o[0].global_x, o[0].global_y)
            refs[o[0].component_ref] += 1
    if not src or not dst or dests[0] not in pcb.footprints:
        return 1
    sref = max(refs, key=refs.get)
    return pair_chirality(src, dst, em.grid_of(pcb.footprints[sref]).bbox,
                          em.grid_of(pcb.footprints[dests[0]]).bbox)


def net_dest(net, dest):
    """the destination part a net reaches, of the parts `dest` names: the first carrying a pad of it (the first named
    when none does)"""
    dests = dest_refs(dest)
    on = {p.component_ref for p in net.pads}
    return next((d for d in dests if d in on), dests[0])


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
    if plan.get('end_layers'):          # (the layers each end has room on, a group's model's: group_ends.bus_plan)
        q['end_layers'] = {nm: [None if ls_ is None else [other_layer(L) for L in ls_] for ls_ in e]
                           for nm, e in plan['end_layers'].items()}
    return q


def run_config(pcb, board):
    """the run's routing config (connect.make_config at the run's sizes), with the board's own hole-to-hole and
    copper-to-edge floors where they are tighter -- setup's, and a candidate's context's (whole_ctx.candidate)"""
    # the board's own hole-to-hole floor when it is tighter than the
    # config's default (the second bench, zynq_ad9364, declares 0.25 mm
    # and its K28 shipped one drill-to-drill graze at the default 0.2);
    # tighten-only, so a board declaring less than the default routes
    # as before
    from list_nets import board_constraint
    # (a run's supplied rules carry the floors the driver resolved -- route_bus: given, else the board's own)
    _sup = _rules.active()
    h2h = _sup.hole_to_hole if _sup.hole_to_hole is not None else board_constraint(board, 'min_hole_to_hole')
    kw = {}
    if h2h and h2h > cn.GridRouteConfig().hole_to_hole_clearance:
        kw['hole_to_hole_clearance'] = float(h2h)
    # and the board's own copper-to-edge floor, likewise tighten-only: the
    # config's 0 falls back to the track clearance (0.1), the project
    # grades the edge at its min_copper_edge_clearance (0.2 on the
    # bench's family), and a lane along an edge -- never on the bench,
    # SCAS on the pose gate's BF article at K28 -- shipped three edge
    # violations at 0.079 mm over (2026-09-08)
    edge = (_sup.edge_clearance if _sup.edge_clearance is not None
            else board_constraint(board, 'min_copper_edge_clearance'))
    if edge and edge > CLEAR:
        kw['board_edge_clearance'] = float(edge)
    return cn.make_config(pcb, TRACK, CLEAR, VIA_SIZE, VIA_DRILL, grid_step=GRID, **kw)


def setup(board, names, dest, log, plan=None, pairs=False):
    """A bench read into a context: the board, the run's nets (with
    `pairs`, a differential pair one member), their ends with their
    layers and escape directions, their taut paths. Returns (ctx, []):
    the second place held the groups a planner here once made of the
    nets, and the callers unpack two.

    `plan` -- ONE PLANNER (#622): the ends, their layers and their escape
    directions given by the plan instead of read off copper:
    {'ends': {net: [tooth_xy, exit_xy]}, 'tooth_layer', 'dest_layer',
    'tooth_dir': {net: unit xy}, 'stub_dir': {net: unit xy}} -- the
    sidecar beside the board (`<board>.plan.json`, written by the
    fanout) when none is passed. Without it (or for nets the plan does
    not name) everything is read off the board."""
    from coherent_nets import by_key
    pcb = parse_kicad_pcb(board)
    if awx_settings.get('JOINT_FOREIGN'):
        # (negotiated buses off one shared array: the other buses' lanes as they publish them, copper of their own
        # nets -- joint_lanes.foreign)
        import joint_lanes
        joint_lanes.foreign(pcb)
    byname = by_key(pcb)
    # PAIR MEMBERS (pairs.py): the two legs of a differential
    # pair become ONE member named after the pair. The P leg's net stands
    # in wherever a member needs a net id (the obstacle models, the
    # source reference); both legs' ids are the run's (kids), so neither
    # leg's copper is static copper to any lane.
    # ...only with `pairs`: a caller that reads the legs as the singles a
    # sidecar keys them by passes none, and the dicts stay keyed by the
    # nets' own names
    _pairs_here = {}
    if pairs:
        names, _pairs_here = _pairs.members(names)
        if _pairs_here:
            log('pair members: ' + ', '.join(f'{b} = {p} + {n}' for b, (p, n) in _pairs_here.items()))

    def _standin(bn):
        for _b, (_pn, _nn) in _pairs_here.items():
            bn[_b] = bn[_pn]
    _standin(byname)
    kids = {byname[nm][0] for nm in names} | {byname[n][0] for pr in _pairs_here.values() for n in pr}
    if plan is None:
        _pj = os.path.splitext(board)[0] + '.plan.json'
        if os.path.exists(_pj):
            import json as _json_pl
            try:
                with open(_pj, encoding='utf-8') as _f:
                    plan = _json_pl.load(_f)
                # (a pair member by its legs: the sidecar keys a pair's
                # ends, layers and directions by its legs, as every read
                # below takes them. Asked by its member's name alone, an
                # all-pair bus's plan named none of its nets and was
                # dropped: sdi_mipi_conv's MIPI bus to J4 read its escape
                # directions off copper, MIPI0_D3's south tooth read west,
                # a far-face tooth inside U11, and the solve refused every
                # round -- the fanout's audit, its legs as singles, clean)
                _pe = plan.get('ends', {})
                named = [nm for nm in names if nm in _pe
                         or any(l_ in _pe for l_ in _pairs_here.get(nm, ()))]
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
                    miss = [nm for nm in names if nm not in named]
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
    # planning and the routing run unchanged, and the copper mirrored
    # back where it is written (route_lanes.write). Per pair, not per
    # board: with three or more parts the
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
    ctx.pairs = _pairs_here
    if chi < 0:
        CY = mirror_axis(pcb)     # the engine's own rule: a lattice line
        pcb, M = to_front_frame(pcb, dest_refs(dest)[0])
        ctx.M = M
        byname = by_key(pcb)
        _standin(byname)
        if plan:
            plan = mirror_plan(plan, M)
        log(f'pair frame: chirality -1, the board turned over about '
            f'y = {CY:.3f} for the braid (copper mirrored back on write)')
    planned = {nm for nm in names if plan and nm in plan.get('ends', {})}
    ctx.pages_first = bool(plan and plan.get('pages_first'))
    # the destination's far-face cut the ends model read its ends with (fanout_from_plan's sidecar, whole_ends), for
    # the whole route's frame to take (whole_frame.build) -- mirrored with the board when it is turned over
    ctx.dest_cut = None
    if plan and plan.get('dest_cut') is not None:
        dc_ = plan['dest_cut']
        if isinstance(dc_, (list, tuple)) and dc_[0] == 'P':
            # (a GROUP's cut: the point on its walk, whole_frame.Group -- turned over, its y mirrored)
            ctx.dest_cut = ('P', float(dc_[1]), (2.0 * CY - float(dc_[2])) if chi < 0 else float(dc_[2]))
        elif isinstance(dc_, (list, tuple)):
            # (a cut off the far face, whole_ends' winding: (face, its coordinate) -- turned over, north and south swap)
            f_, v_ = dc_[0], float(dc_[1])
            if chi < 0:
                f_, v_ = {'N': 'S', 'S': 'N'}.get(f_, f_), (2.0 * CY - v_ if f_ == 'E' else v_)
            ctx.dest_cut = (f_, v_)
        else:
            ctx.dest_cut = (2.0 * CY - float(dc_)) if chi < 0 else float(dc_)
    # ...and a GROUP's ways in (whole_frame.Group: each lane's place on its walk, a point), mirrored the same
    ctx.dest_entry = None
    if plan and plan.get('dest_entry'):
        ctx.dest_entry = {nm: (float(p_[0]), (2.0 * CY - float(p_[1])) if chi < 0 else float(p_[1]))
                          for nm, p_ in plan['dest_entry'].items()}
    # ...and the source's far-face cut (whole_frame.src_cut: which side a far-face tooth goes round), mirrored the same
    ctx.src_cut = None
    if plan and plan.get('src_cut') is not None:
        ctx.src_cut = (2.0 * CY - float(plan['src_cut'])) if chi < 0 else float(plan['src_cut'])
    # (the nets whose ends' layers are HELD -- a group's model's, group_ends: the lanes that cross another bus's)
    ctx.held_layers = frozenset((plan or {}).get('held_layers') or ())
    # (...and the layers each end has room on, where that model read them: a via end's layer is chosen among them)
    ctx.end_layers = dict((plan or {}).get('end_layers') or {})
    if ctx.pages_first and EXACT_PAGES_ENV != '0':
        # a PAGES-FIRST plan (its sidecar's `pages_first`): its two chains
        # were chosen to cover every lane, so the schedule pages it exactly
        import schedule as _sch
        _sch.EXACT_PAGES = 1
    else:
        # RESET IT, do not merely decline to set it (review, 2026-09-17).
        # There was no else-branch, so the flag was sticky: `plan_braid`
        # is called in a LOOP in-process (judge_gate, pinch_gate,
        # fanout_from_plan's judge), and a job whose plan carries the
        # marker left EXACT_PAGES=1 behind for every job after it. Under
        # judge_gate's Pool + imap_unordered, WHICH jobs inherit it
        # depends on which worker takes which job -- a wall-clock race
        # deciding how a board is scored.
        import schedule as _sch
        _sch.EXACT_PAGES = 0
        if ctx.pages_first:
            log('plan sidecar: pages-first, but BRAID_EXACT_PAGES=0 -- the '
                'schedule chooses its own pages')

    # a pair member's ends are the MIDPOINTS of its legs' ends (the lane
    # is planned there); the legs' own ends are kept for the pair router
    _legs = [n for pr in _pairs_here.values() for n in pr]
    _pl_ends = (plan or {}).get('ends', {})
    _want = ([nm for nm in names if nm not in planned and nm not in _pairs_here]
             + [n for n in _legs if n not in _pl_ends])
    ends = endpoints(pcb, _want, byname, dest_ref=dest) if _want else {}
    # (each planned net's end at the destination part it reaches: DEST may name several, one source and several ends)
    for nm in [n for n in names if n in planned]:    # the run's order: a set's would follow the hash seed
        e = plan['ends'][nm]
        ends[nm] = (tuple(e[0]), tuple(e[1]), net_dest(byname[nm][1], dest))
    for n in _legs:
        if n in _pl_ends:
            e = _pl_ends[n]
            ends[n] = (tuple(e[0]), tuple(e[1]), net_dest(byname[n][1], dest))
    ctx.pair_ends = {}
    for _b, (_pn, _nn) in list(_pairs_here.items()):
        (sp_, tp_, _), (sn_, tn_, _) = ends[_pn], ends[_nn]
        _ds, _dt = math.sqrt(ts.d2(sp_, sn_)), math.sqrt(ts.d2(tp_, tn_))
        # the limit: two lane pitches, or 1.3 of the destination's ball
        # pitch -- a harmonised pair's tips stand one ball apart
        _dg = em.grid_of(pcb.footprints[ends[_pn][2]])
        _lim = max(_pairs.MAX_SEP, 1.3 * max(_dg.pitch_x, _dg.pitch_y))
        if max(_ds, _dt) > _lim:
            # the plan put this pair's ends APART: it stays a PAIR all the
            # same -- the pose router is tried from the ends as they are,
            # and a pair it cannot couple is REFUSED, both legs open and
            # named. It used to fall back to the legs as singles, which
            # ships a DDR strobe or clock uncoupled the whole way (Andy,
            # 2026-09-20: never).
            log(f'pair {_b}: ends apart -- teeth {_ds:.2f} mm, berths {_dt:.2f} mm (limit {_lim:.2f}); '
                f'still a pair: coupled or refused, never singles')
        ends[_b] = (_pairs.mid(sp_, sn_), _pairs.mid(tp_, tn_), ends[_pn][2])
        ctx.pair_ends[_b] = ((sp_, sn_), (tp_, tn_))
    ctx.pairs = _pairs_here
    ctx.pcb, ctx.byname, ctx.ends, ctx.kids = pcb, byname, ends, kids
    from coherent_nets import namer
    ctx.net_key = namer(pcb)                # (a net by the whole route's name for it, coherent_nets.net_keys)
    ctx.plan = plan
    ctx.tooth_layer = {nm: (plan['tooth_layer'][nm] if nm in planned else
                            _layer_at(pcb, byname[nm][0], ends[nm][0], 'F.Cu'))
                       for nm in names}
    ctx.dest_layer = {nm: (plan['dest_layer'][nm] if nm in planned else
                           _layer_at(pcb, byname[nm][0], ends[nm][1], 'F.Cu'))
                      for nm in names}
    ctx.pair_layers = {}
    for _b, (_pn, _nn) in _pairs_here.items():
        # the pair's layers are its P leg's; the N leg's own end layers are
        # kept beside them (a berth laid on the other layer gets a barrel
        # at its tip, connect_pair)
        (sp_, _sn), (tp_, _tn) = ctx.pair_ends[_b]
        _pt = (plan or {}).get('tooth_layer', {})
        _pd = (plan or {}).get('dest_layer', {})
        ctx.tooth_layer[_b] = _pt[_pn] if _pn in _pt else _layer_at(pcb, byname[_pn][0], sp_, 'F.Cu')
        ctx.dest_layer[_b] = _pd[_pn] if _pn in _pd else _layer_at(pcb, byname[_pn][0], tp_, 'F.Cu')
        _tn_l = _pt[_nn] if _nn in _pt else _layer_at(pcb, byname[_nn][0], _sn, 'F.Cu')
        _dn_l = _pd[_nn] if _nn in _pd else _layer_at(pcb, byname[_nn][0], _tn, 'F.Cu')
        ctx.pair_layers[_b] = (_tn_l, _dn_l)
    bundle_layer = bundle_layer_of(ctx.tooth_layer)
    ctx.landed = set()            # nets whose lane is on the board
    ctx.src_ref = {}
    for nm in names:
        nid, net = byname[nm]
        # (the destination's own pads are no source: a tooth whose stub ends nearer a ball of the destination than
        # any of its own -- a dog-bone run down the channel -- was read as starting AT the destination, and the whole
        # route, which plans the lanes of the run's one source, left it out of every frame: the generated c4s14g25's
        # SYN07, open every round)
        pads_ = [p for p in net.pads if p.component_ref != ends[nm][2]] or list(net.pads)
        ctx.src_ref[nm] = min(pads_, key=lambda p: ts.d2(
            (p.global_x, p.global_y), ends[nm][0])).component_ref

    _cache = {}

    def obs_for(nm, layer):
        k = (nm, layer, 'kids')
        if k not in _cache:
            _cache[k] = build_obstacles(pcb, byname[nm][0], kids, layer)
        return _cache[k]

    ctx.obs_for = obs_for
    # the direction each free end ESCAPES in, read from the stub's own
    # copper (its run), whatever angle the array sits at
    ctx.tooth_dir = {nm: (tuple(plan['tooth_dir'][nm]) if nm in planned else
                          _end_dir(pcb, byname[nm][0], ends[nm][0],
                                   byname[nm][1].pads)) for nm in names}
    ctx.stub_dir = {nm: (tuple(plan['stub_dir'][nm]) if nm in planned else
                         _end_dir(pcb, byname[nm][0], ends[nm][1],
                                  byname[nm][1].pads)) for nm in names}
    for _b, (_pn, _nn) in _pairs_here.items():
        # a pair member's escape directions are its P leg's, read at the
        # leg's own end (at the midpoint there is no copper to read)
        (sp_, _sn), (tp_, _tn) = ctx.pair_ends[_b]
        _ptd = (plan or {}).get('tooth_dir', {})
        _psd = (plan or {}).get('stub_dir', {})
        ctx.tooth_dir[_b] = (tuple(_ptd[_pn]) if _pn in _ptd else
                             _end_dir(pcb, byname[_pn][0], sp_, byname[_pn][1].pads))
        ctx.stub_dir[_b] = (tuple(_psd[_pn]) if _pn in _psd else
                            _end_dir(pcb, byname[_pn][0], tp_, byname[_pn][1].pads))
    # ---- the taut paths. CACHED to a sidecar: the
    # relax is a pure function of (board, net ends) and costs 11.6s
    # of a 25s braid, recomputed identically by every trial braid on
    # the same fanout board (measured, K35 cProfile). Key = the
    # board file's (mtime_ns, size) + a version bumped whenever the
    # relax algorithm changes; per-net entries so a different K
    # fills in only what is missing. json round-trips floats exactly
    # (repr), so the cached run stays bit-identical.
    TAUT_CACHE_VERSION = 5   # 2: taut_clean reseeds (0902); 3: rect and oval pads as capsules, a string's disc round them; 4: a filled copper graphic a wall; 5: a string no taut path its chord
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
    if planned or not db.memo_on_disk():
        _tc_key = None      # the cache keys on the board's own ends; and it is a harness's (TAUT_MEMO=1), not a
        #                     user's: nothing written beside their board
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
    # 0.025 grid: the fanout packs stub ends at 0.25, which is the legal
    # minimum (track + clearance = 0.227) plus 23 um. On a 0.05 grid the
    # cell nearest a lane's centreline can sit 25 um off it -- outside
    # the 23 um that clear the neighbours.
    # The router's default via cost stays: re-pricing it was measured
    # a loser (2026-08-31; 2x and 4x: ladder unchanged, K28 50 -> 54
    # vias) because the braid's vias are structural (page dive and
    # surface, reserved weaves, legs), so a costlier via only buys
    # worse detours that force more of them.
    ctx.cfg = run_config(pcb, board)
    ctx.base_segments = list(pcb.segments)
    ctx.base_vias = list(pcb.vias)
    return ctx, []


# (a module function, not a method: the whole route's PlanLanes borrows route_lane alone)
def route_lane_necked(self, nm, nid, virt, virt_vias, band, margin):
    """A single its full-width search refused (route_lane), laid again in three hops along its planned line: from
    its tooth to the face band's end (whole_solve.FACE_ROOM along the line) at the width the lay may narrow a track
    to (the fab escalation's first rung, never wider than the escape's stub), on at its own width, and the last face
    band into its stub at the narrow width again -- the snap's own reading of its end stretches (whole_snap), the
    narrowing disclosed in the fab ledger. A hop's waypoint stands on the plan's line, on the plan's layer there.
    (segments, vias) or None; partial copper is discarded"""
    import copy as _copy
    import fab_tiers as _ft
    import whole_solve as _ws
    ctx = self.ctx
    rg_ = _ft.escalation_rungs(len(ctx.cfg.layers))
    if not rg_ or not getattr(self, '_geo', None) or nm not in self._geo.get('lanes', {}):
        return None
    w_n = min(_rules.active().fan_track, rg_[0]['track_width'])
    if w_n >= ctx.cfg.track_width - 1e-9:
        return None
    R_ = _ws.FACE_ROOM
    xy = [tuple(map(float, p)) for p in self.lane_xy[nm]]
    cum = [0.0]
    for p, q in zip(xy, xy[1:]):
        cum.append(cum[-1] + math.hypot(q[0] - p[0], q[1] - p[1]))
    Lt = cum[-1]

    def at_(s_):
        for k in range(len(xy) - 1):
            if cum[k + 1] >= s_ and cum[k + 1] - cum[k] > 1e-12:
                u = (s_ - cum[k]) / (cum[k + 1] - cum[k])
                return (xy[k][0] + (xy[k + 1][0] - xy[k][0]) * u, xy[k][1] + (xy[k + 1][1] - xy[k][1]) * u)
        return xy[-1]
    pcs_ = self._geo['lanes'][nm]['pieces']

    def layer_at(p):
        def d_(pc):
            ax, ay, bx, by = pc[:4]
            L2 = (bx - ax) ** 2 + (by - ay) ** 2
            t = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, ((p[0] - ax) * (bx - ax) + (p[1] - ay) * (by - ay)) / L2))
            return math.hypot(ax + t * (bx - ax) - p[0], ay + t * (by - ay) - p[1])
        return min(pcs_, key=d_)[4]
    a, aL = self.teeth[nm], ctx.tooth_layer[nm]
    b, bL = self.stubs[nm], ctx.dest_layer[nm]
    if Lt <= 2 * R_ + 2 * ctx.cfg.grid_step:
        hops = [(a, aL, b, bL, w_n)]
    else:
        P1, P2 = at_(R_), at_(Lt - R_)
        L1, L2 = layer_at(P1), layer_at(P2)
        hops = [(a, aL, P1, L1, w_n), (P1, L1, P2, L2, None), (P2, L2, b, bL, w_n)]
    segs_all, vias_all = [], []
    n_s, n_v = len(ctx.pcb.segments), len(ctx.pcb.vias)
    for p, pL, q, qL, w in hops:
        cfg_ = ctx.cfg
        if w is not None:
            cfg_ = _copy.copy(ctx.cfg)
            cfg_.track_width = w
        res = cn.connect(ctx.pcb, nid, p, pL, q, qL, cfg_, band=band, virtual=virt, margin=margin,
                         window_pts=xy, virtual_vias=virt_vias)
        if res is None:
            del ctx.pcb.segments[n_s:]
            del ctx.pcb.vias[n_v:]
            return None
        ctx.pcb.segments.extend(res[0])
        ctx.pcb.vias.extend(res[1])
        segs_all.extend(res[0])
        vias_all.extend(res[1])
    _ft.note_narrowing(nid, 'track_width', ctx.cfg.track_width, w_n, 'bus lane end stretch',
                       count=sum(1 for s_ in segs_all if s_.width < ctx.cfg.track_width - 1e-9), net_name=nm)
    print(f'{nm:7s} laid with its end stretches at {w_n:g} mm (refused at {ctx.cfg.track_width:g})')
    return segs_all, vias_all

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

KRT_TOOL = {'scope': [], 'kind': 'actor'}   # #937: a research tool (awx), catalogued, shown at no door
import argparse
import math
import re
import os
import time as _time
import shutil
import sys
from collections import Counter, defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb, Segment  # noqa: E402
from kicad_writer import generate_via_sexpr  # noqa: E402
import ship_vias  # noqa: E402  a via in a pad declares Type VII (#962)
import topo_strings as ts  # noqa: E402
import connect as cn  # noqa: E402
import corridor as cr  # noqa: E402
import detect_buses as db  # noqa: E402
from schedule import Schedule  # noqa: E402
import escape_moves as em  # noqa: E402
from select_moves import pair_chirality  # noqa: E402
from bga_fanout.flip_frame import to_front_frame, other_layer, mirror_axis  # noqa: E402

import rules as _rules  # noqa: E402  ONE source for every design rule

# ONE SOURCE: rules.py. main() installs them (rules.install_defaults);
# without an install they are the literals they have always been -- see
# rules.py, "USING IT".
TRACK = ts.TRACK         # ONE source: topo_strings
CLEAR = _rules.DEFAULT.hug
                         # 0.1 spec + 5um so hugs don't sit exactly at 0.1
SPEC_CLEARANCE = _rules.DEFAULT.clearance
                         # the spec itself: what the fanout lays at, what grade_k
                         # grades at, and what the output PROJECT records (CLEAR
                         # is the router's private margin over it, not a rule)
VIA_SIZE = _rules.DEFAULT.via_size
VIA_DRILL = _rules.DEFAULT.via_drill

MINP = _rules.DEFAULT.exit_pitch   # lane pitch floor at the exits
LPITCH = _rules.DEFAULT.lane_pitch # pitch of a side-join / side-exit block
BLOCK_GAP = 0.45               # a block starts this far beyond what it clears
                               # line: a lane passing a stub's END at the
                               # legal minimum plus a hair
                               # (= TRACK + CLEAR + 0.07)
HALF_SEP = _rules.DEFAULT.half_sep  # two lanes at their band edges clear
                               # (= (TRACK + SPEC_CLEARANCE) / 2)
LEG_W = 0.5                    # half-width in s of a join / exit leg's band
ROW_O = 0.7                    # head-on berths within this across the spine form a ROW (Corridor._head_order)
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
# PROBE knobs (replan.py, 2026-09-10): a local re-braid of a few nets on a
# frozen board is a SCREEN -- a refusal there is unjudged whatever the
# budget -- and its cost is failing searches repeated: three identical
# attempts (nothing changes between them when the refused set repeats)
# and last-call searches at 1.6 M iterations for a net that will not
# route. BRAID_ATTEMPTS caps the attempt ladder, BRAID_BUDGET_X the
# rescue / last-call budget multiplier. Defaults = the braid as it was
# (measured: one attempt gives the same board 22% faster; a halved
# budget loses nets, so the probe keeps the full one).
ATTEMPTS = int(os.environ.get('BRAID_ATTEMPTS', '6'))
LADDER_MODE = os.environ.get('BRAID_LADDER', 'full')   # full | open (connect_ladder)
BUDGET_X = int(os.environ.get('BRAID_BUDGET_X', '4'))
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
LANE_MIN = TRACK + CLEAR + 0.02    # the least centreline pitch two lanes are PLANNED at (ring floor, block squeeze, a leg's room)
BIRTH_W = 0.2                  # place_dives: a bound birth dive's layer-change window, either side of its site
RING_DIP = 0.5                 # _order_ring: a lifted ring lane's dip narrower than this is filled level
PACK_MODE = int(os.environ.get('BRAID_PACK', '0') or 0)  # pack.py at write time (opt-in)
# PLAN_PAGES_SIDERS=1 (2026-09-14, pages-first plans only -- the plan sidecar's
# `pages_first` marker): a stub whose direction is ACROSS the spine (a side
# face) is always a side exit, never head-on. classify's head-on test is
# relative (the most upstream stub of a face parallel to the spine is head-on,
# every other one a side exit), so a candidate's class -- and its slot in the
# target order -- flipped with which neighbours the plan chose, and no planner
# key could follow it (K28: 26/378 target pairs off). With every side-face
# stub in its side's comb, the order on a face is a function of position alone.
PAGES_SIDERS = int(os.environ.get('PLAN_PAGES_SIDERS', '1') or 0)   # default 1: measured best on the K28/K35/K41 ladder (2026-09-14); inert without the plan marker
# ^ **K51 is not in that ladder, and at K51 it is WRONG**: on one K51 board
# (2026-09-15, session 13) turning it off took the SAME fanout board from
# 112 vias to 98. See the README, "the record".
#
# BRAID_EXACT_PAGES, and why it is read HERE (2026-09-15, session 13): a
# pages-first plan sets `schedule.EXACT_PAGES = 1` by assignment, AFTER
# schedule has read its own env -- so `BRAID_EXACT_PAGES=0` could not turn
# the rule off on the only plans that have it, and the rule has never been
# A/B'd. It is the other half of the same K51 finding: with the marker's two
# rules off the board routes 98 with NOTHING open, with EXACT_PAGES alone it
# routes 98 and leaves SDQ11 open. None = the plan decides (today's
# behaviour, byte-identical); '0' forces it off; anything else forces it on.
EXACT_PAGES_ENV = os.environ.get('BRAID_EXACT_PAGES')
BRANCH = os.environ.get('BRAID_BRANCH', '1') != '0'   # the trunk and its branches (build_branches); BRAID_BRANCH=0 off
import pairs as _pairs  # noqa: E402  the bus's differential pairs as members (#622, 2026-09-20)
PAIRS = int(os.environ.get('BRAID_PAIRS', '0') or 0)   # 1: a differential pair is ONE lane, routed coupled by the production pair router (pairs.py, connect_pair); 0: its legs are singles -- byte-identical
PROX_TRACK = 0.25              # two same-layer lines nearer than this: a short (pinch_gate.py reads it)
ECON_LONG = float(os.environ.get('BRAID_ECON_LONG', '3.0'))   # econ re-lay: a lane this far (mm) over its airline is a candidate
# BRAID_ECON_MM_PER_VIA (mm, 0 = unbounded, the rule until 2026-09-20): the
# most copper the econ re-lay may buy a via with. K36 SBA1 was re-laid
# from 2 vias / 20.5 mm to 0 vias / 44.7 mm -- round the outside of the
# DDR and back up under its balls to its own dogbone via, the neighbour
# SA0 hugging the comb in front of its berth -- and SDQ0 from 5 / 28.0
# to 3 / 47.6; the grade has no length term and never saw either. A DDR
# lane 24 mm over its group is 24 mm of meander on every other lane of
# the group at the length-matching phase.
ECON_MM_PER_VIA = float(os.environ.get('BRAID_ECON_MM_PER_VIA', '6.0'))
# BRAID_ECON_JOINT (default on): when a lane's cheaper re-lay is too long
# for the guard, or an extra-long lane has no cheaper lane alone, the
# min-cut probe of rip_for names the lane(s) of this run its short path
# would cross (SA0 in front of SBA1's berth), rips them, re-lays the lane
# and then them, and keeps the set only when it is cheaper in all.
ECON_JOINT = int(os.environ.get('BRAID_ECON_JOINT', '1'))
# BRAID_APPROACH_RESERVE (mm, 0 = off): see Corridor.approach_virt
APPROACH_RESERVE = float(os.environ.get('BRAID_APPROACH_RESERVE', '1.0') or 0)
# ...and how hard the rip tries when a lane is refused. These were hard
# coded defaults on rip_for; they are the natural dials for "repair
# harder" and there was no way to turn them.
RIP_VICTIMS = int(os.environ.get('BRAID_RIP_VICTIMS', '3'))
RIP_DEPTH = int(os.environ.get('BRAID_RIP_DEPTH', '1'))


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


BLOCK_PUSH = int(os.environ.get('BRAID_BLOCK_PUSH', '1') or 0)
# ^ 2: pushed only when the push CLEARS within its cap (a push that finds
# copper at every step is no push -- see _clear_block); 1: pushed to the
# cap regardless (the recorded chain's rule).
# ^ 0: a side block is NOT pushed whole clear of static copper (_clear_block);
# it starts a BLOCK_GAP beyond the stubs and each lane is bent round what its
# own run meets (deflect_islands). The human bench's north block: pushed 1.5 mm
# clear of the passive cluster north of DU1, ours sat 2 mm further out than
# the human's lanes (SDQ10 -7.9 against -5.5), every north diagonal steeper.
JOG_VIA = float(os.environ.get('BRAID_JOG_VIA', '1.0') or 1.0)
W_GATE = 0.33                  # narrowest swap column the gated schedule
                               # gets: every clean gated K on the bench
                               # had W >= 0.343 (K21, W=0.322, needed
                               # its third attempt); below it a lane
                               # passed in one column and diving in the
                               # next has no cell for its via, so the
                               # gate yields and columns are spent on
                               # vias instead (see Corridor.run)


class CtxView:
    """A corridor's view of the shared braid context with some per-net END
    tables overridden (BRAID_BRANCH): a trunk whose side exits end at their
    HANDOFF points, a branch whose lanes start there. Every other attribute
    -- the board, the landed set, the corridors, the config -- is the
    shared context's own, read and written through."""
    def __init__(self, base, **over):
        object.__setattr__(self, '_base', base)
        merged = {}
        for k, v in over.items():
            d = dict(getattr(base, k))
            d.update(v)
            merged[k] = d
        object.__setattr__(self, '_over', merged)
        # 'plan': the overrides hold (the corridor plans to its handoffs);
        # 'route': the shared tables (the lane is routed to its real berth)
        object.__setattr__(self, '_mode', 'plan')

    def __getattr__(self, k):
        over = object.__getattribute__(self, '_over')
        if k in over and object.__getattribute__(self, '_mode') == 'plan':
            return over[k]
        return getattr(object.__getattribute__(self, '_base'), k)

    def __setattr__(self, k, v):
        if k == '_mode':
            object.__setattr__(self, '_mode', v)
            return
        over = object.__getattribute__(self, '_over')
        if k in over:
            over[k] = v
        else:
            setattr(object.__getattribute__(self, '_base'), k, v)


HAND_DS = 0.6                  # BRAID_BRANCH: the handoff line this far past the trunk's s1


def _simplify_so(pts, tol):
    """Douglas-Peucker on an (s, o) polyline: the points a sampled lane
    needs within `tol` of its samples."""
    if len(pts) < 3:
        return list(pts)
    keep = [False] * len(pts)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        i, j = stack.pop()
        (s0, o0), (s1, o1) = pts[i], pts[j]
        ds_, do_ = s1 - s0, o1 - o0
        L = math.hypot(ds_, do_) or 1e-12
        k_best, d_best = None, tol
        for k in range(i + 1, j):
            d = abs((pts[k][0] - s0) * do_ - (pts[k][1] - o0) * ds_) / L
            if d > d_best:
                k_best, d_best = k, d
        if k_best is not None:
            keep[k_best] = True
            stack += [(i, k_best), (k_best, j)]
    return [q for q, kp in zip(pts, keep) if kp]


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


def build_branches(ctx, c1, log, _tgt=None, _before=None, _round=0):
    """BRAID_BRANCH: the corridor `c1` (planned) as a TRUNK with BRANCHES.
    Its side exits leave it at HANDOFF points -- their exit-block slot HAND_DS
    past s1, on their page (a swimmer: its berth's layer) -- and ride a
    branch per side round the destination (corridor.build_wrap_spine from
    the block's inner edge), in the order they arrive, tightening as they
    peel off on legs (Corridor.branch). The trunk is planned again with the
    side exits ENDING at their handoffs (a CtxView; its first spine kept),
    and still ROUTES every lane end to end: a side exit's band is the
    trunk's up to the handoff line and its branch's past it, its window
    and reservation the trunk's plus the branch's. Returns the trunk (the
    branches ride on it as .branch_of) or `c1` when it has no side exits."""
    xs = [nm for nm in c1.siders]
    if not xs:
        return c1
    sp = c1.spine
    s_h = c1.s1 + HAND_DS
    sc1 = c1.sched_cur
    dn = tuple(float(v) for v in sp.d[-1])
    dest = ctx.ends[xs[0]][2]
    dpads = [(p.global_x, p.global_y) for p in ctx.pcb.footprints[dest].pads]
    cen = (sum(p[0] for p in dpads) / len(dpads), sum(p[1] for p in dpads) / len(dpads))

    def sweep(nm):
        a = [math.atan2(y - cen[1], x - cen[0]) for x, y in ctx.paths[nm]]
        return sum((v - u + math.pi) % (2 * math.pi) - math.pi for u, v in zip(a, a[1:]))
    tgt = dict(_tgt) if _tgt else {nm: c1.target_o[nm] for nm in xs}
    H = {nm: sp.xy(s_h, tgt[nm]) for nm in xs}
    L_h = {nm: (sc1.page.get(nm) or ctx.dest_layer[nm]) for nm in xs}
    view = CtxView(ctx, ends={nm: (ctx.ends[nm][0], H[nm], ctx.ends[nm][2]) for nm in xs},
                   dest_layer=L_h, stub_dir={nm: (-dn[0], -dn[1]) for nm in xs})
    c2 = Corridor(c1.idx, c1.members, view, log)
    c2.pin_target = set(xs)
    real_stubs = {nm: ctx.ends[nm][1] for nm in c1.members}

    def fixed_spine(self=c2, base=c1):
        self.H = base.H
        self.wrap = False
        self.spine_core, self.spine = base.spine_core, base.spine
        self.teeth = {nm: self.ctx.ends[nm][0] for nm in self.members}
        self._stubs_plan = {nm: self.ctx.ends[nm][1] for nm in self.members}
        self.stubs = self._stubs_plan
        self.st = {nm: self.spine.project_pt(self.teeth[nm]) for nm in self.members}
        self.se = {nm: self.spine.project_pt(self._stubs_plan[nm]) for nm in self.members}
    c2.build_spine = fixed_spine

    def planning(meth, self=c2):
        def run_(*a, **k):
            view._mode = 'plan'
            if getattr(self, '_stubs_plan', None) is not None:
                self.stubs = self._stubs_plan
            try:
                return meth(*a, **k)
            finally:
                view._mode = 'route'
                if getattr(self, '_stubs_plan', None) is not None:
                    self.stubs = dict(self._stubs_plan)
                    self.stubs.update({nm: real_stubs[nm] for nm in xs})
        return run_
    for m_ in ('build_spine', 'classify', 'offsets', 'lay_lanes'):
        setattr(c2, m_, planning(getattr(c2, m_)))
    c2.run(plan_only=True)
    # the branches: one per exit side with two lanes or more
    # the trunk's HEAD-ON lanes past the handoff line run to berths on the
    # same faces the branches wrap; a ring built round the pads alone ran on
    # top of them (HHa: SA0 over SA2 for 7.7 mm once SA2's row put it outside
    # its berth, as the human's is), so the rings go round those lanes too
    tails = [tuple(float(v) for v in sp.xy(s_, o_)) for nm in c2.heads_e if nm not in xs
             for (s_, o_) in c2.mid.get(nm, ()) if s_ >= s_h]
    branch_of = {}
    for k, sg in enumerate((-1, 1)):
        mem = [nm for nm in xs if c1.exit_side[nm] == sg]
        if len(mem) < 2:
            continue
        ccw = sum(sweep(nm) for nm in mem) > 0
        o_in = min((c2.target_o[nm] for nm in mem), key=lambda v: sg * v) - sg * LPITCH
        start = sp.xy(s_h, o_in)
        vb = CtxView(ctx, ends={nm: (H[nm], ctx.ends[nm][1], ctx.ends[nm][2]) for nm in mem},
                     tooth_layer={nm: L_h[nm] for nm in mem}, tooth_dir={nm: dn for nm in mem})
        cb = Corridor(100 + c1.idx * 10 + k, mem, vb, log)
        cb.keep_order = True
        cb.branch = True

        def wrap_spine(self=cb, start=start, ccw=ccw):
            ctx_ = self.ctx
            self.H = LPITCH * (len(self.members) - 1) / 2 + LPITCH
            self.wrap = True
            teeth = {nm: ctx_.ends[nm][0] for nm in self.members}
            stubs = {nm: ctx_.ends[nm][1] for nm in self.members}
            spn = cr.build_wrap_spine(dpads, list(stubs.values()), [start], ccw, dn, LPITCH, hull_extra=tails)
            self.spine_core = spn
            P0, d0 = spn.P[0], spn.d[0]
            Pn, dn_ = spn.P[-1], spn.d[-1]
            back = max([0.3] + [-((t[0] - P0[0]) * d0[0] + (t[1] - P0[1]) * d0[1]) + 0.3 for t in teeth.values()])
            fwd = max([0.3] + [((t[0] - Pn[0]) * dn_[0] + (t[1] - Pn[1]) * dn_[1]) + 0.3 for t in stubs.values()])
            self.spine = spn.extend(back, fwd)
            self.teeth, self.stubs = teeth, stubs
            self.st = {nm: self.spine.project_pt(teeth[nm]) for nm in self.members}
            self.se = {nm: self.spine.project_pt(stubs[nm]) for nm in self.members}
        cb.build_spine = wrap_spine
        cb.run(plan_only=True)
        log(f'  branch {"ccw" if ccw else "cw"} of side {sg:+d}: {len(mem)} lanes, spine {cb.spine.L:.1f} mm, '
            f'handoffs at trunk s {s_h:.2f}')
        for nm in mem:
            branch_of[nm] = cb
    c2.branch_of, c2.handoff_s = branch_of, s_h
    # the trunk planned its side exits to their HANDOFFS on L_h; in route mode
    # its view hands back the real berth layers, and the tail rule and the
    # profile then flipped the last trunk piece of 16 of HHa's 31 branch lanes
    # to the wrong layer (audit B)
    c2._plan_dest = dict(ctx.dest_layer)
    c2._plan_dest.update(L_h)
    # the trunk ROUTES a branch lane end to end: band, window, reservation
    t_band = c2.band_of
    t_virt, t_vvias = c2.virtual_of, c2.virtual_vias_of

    def compose(nm, band_t, band_b):
        if band_b is None:
            return band_t

        def band(xs_, ys_, L):
            xs_ = np.asarray(xs_, dtype=float)
            ys_ = np.asarray(ys_, dtype=float)
            mt = (np.ones((len(xs_), len(ys_)), dtype=bool) if band_t is None
                  else np.asarray(band_t(xs_, ys_, L), dtype=bool))
            mb = np.asarray(band_b(xs_, ys_, L), dtype=bool)
            out = np.empty(mt.shape, dtype=bool)
            for i in range(0, len(xs_), 64):
                X, Y = np.meshgrid(xs_[i:i + 64], ys_, indexing='ij')
                S, _O = sp.project(X, Y)
                out[i:i + 64] = np.where(S < s_h, mt[i:i + 64], mb[i:i + 64])
            return out
        return band

    def band_of(nm, slack=0.0, open_layers=False):
        b = t_band(nm, slack=slack, open_layers=open_layers)
        cb_ = branch_of.get(nm)
        return b if cb_ is None else compose(nm, b, cb_.band_of(nm, slack=slack, open_layers=open_layers))

    def virtual_of(unrouted):
        segs = list(t_virt(unrouted))
        for cb_ in {id(v): v for v in branch_of.values()}.values():
            mine = [om for om in unrouted if branch_of.get(om) is cb_]
            if mine:
                segs.extend(cb_.virtual_of(mine))
        return segs

    def virtual_vias_of(unrouted):
        vv = list(t_vvias(unrouted) or [])
        for cb_ in {id(v): v for v in branch_of.values()}.values():
            mine = [om for om in unrouted if branch_of.get(om) is cb_]
            if mine:
                vv.extend(cb_.virtual_vias_of(mine) or [])
        return vv
    c2.band_of = band_of
    c2.virtual_of, c2.virtual_vias_of = virtual_of, virtual_vias_of
    t_lay = c2.lay_lanes

    def lay_lanes(*a, **k):
        # the planned line (the search window) runs on through the branch
        r = t_lay(*a, **k)
        for nm, cb_ in branch_of.items():
            if nm in c2.lane_xy and nm in cb_.lane_xy:
                c2.lane_xy[nm] = list(c2.lane_xy[nm]) + list(cb_.lane_xy[nm][1:])
        return r
    c2.lay_lanes = lay_lanes
    c2.lay_lanes()
    if _round < 4:
        # a replan can bring other legs onto other lanes: the pairs found
        # so far stay constrained, and the search runs again
        before = _before if _before is not None else {}
        base = {nm: c1.target_o[nm] for nm in xs}
        if _uncross_colliding_legs(ctx, branch_of, tgt, {nm: c1.exit_side[nm] for nm in xs}, before, log):
            sc_ = c1.sched_cur
            return build_branches(ctx, c1, log, _tgt=_order_handoffs(
                base, before, {nm: c1.exit_side[nm] for nm in xs},
                lambda a_, b_: (c1.pair_floor(a_, b_, LANE_MIN, None, False),
                                c1.pair_floor(a_, b_, LPITCH, sc_, False, c1.exit_side[a_]))),
                                  _before=before, _round=_round + 1)
    return c2


def _order_handoffs(base, before, side, floors=None):
    """The side's own slots handed out in a stable topological order:
    every pair in `before` (b: the lanes that must be handed off inside b)
    as constrained, every other pair as in `base`. With `floors` (a, b) ->
    (least, preferred) gap, the new order is SPACED by its own neighbours'
    floors inside the block's old width, squeezed as _fit_block squeezes:
    the old slot values were spaced for the old neighbours, and handed out
    in a new order they put HHa's SCK, a pair, 0.364 mm from singles on
    both sides (its conductors 0.228 from them; the ring lifted three lanes
    at its first point to open 0.443)."""
    out = dict(base)
    for sg in (-1, 1):
        mem = [nm for nm in base if side[nm] == sg]
        rank = {nm: sg * base[nm] for nm in mem}        # smaller = nearer the spine
        order, left = [], set(mem)
        while left:
            free = [nm for nm in left if not (before.get(nm, set()) & left)]
            nxt = min(free or left, key=lambda n: rank[n])
            order.append(nxt)
            left.discard(nxt)
        slots = sorted((base[nm] for nm in mem), key=lambda v: sg * v)
        if floors is None or len(order) < 2:
            for nm, o_ in zip(order, slots):
                out[nm] = o_
            continue
        W = abs(slots[-1] - slots[0])
        fl = [floors(order[k - 1], order[k]) for k in range(1, len(order))]
        mins, prefs = [f[0] for f in fl], [f[1] for f in fl]
        if sum(prefs) <= W + 1e-9:
            gaps = prefs
        else:
            lam = max(0.0, min(1.0, (W - sum(mins)) / max(sum(prefs) - sum(mins), 1e-9)))
            gaps = [m + (p_ - m) * lam for m, p_ in zip(mins, prefs)]
        acc = 0.0
        for k, nm in enumerate(order):
            if k:
                acc += gaps[k - 1]
            out[nm] = slots[0] + sg * acc
    return out


def _uncross_colliding_legs(ctx, branch_of, tgt, side, before, log):
    """A branch keeps the order its lanes arrive in, so a lane that peels
    off before a lane riding inside it crosses that lane on its leg -- by
    layer, which the plan allows where there is room (HHa's legs cross 67
    lanes, cleanly). Where the planned copper of the two COLLIDES on one
    layer there was none: zynq K44's DQ branch, berths 0.4 mm apart a
    millimetre inside the ring, every leg its own tail on the berth's layer,
    ten tails over lanes still riding round (DQ2 along DQ10's berth row).
    Each such pair is added to `before` -- the first to peel off is handed
    off inside the other -- so the trunk's braid does that reordering, where
    crossings are priced by page; every other pair keeps its order (handing
    EVERY crossing to the trunk cost HHa 16 vias and a net). Returns True
    when it added a pair."""
    TW, CL = ctx.cfg.track_width, ctx.cfg.clearance
    block = TW + CL - 0.005
    added = 0
    for cb in {id(v): v for v in branch_of.values()}.values():
        mem = [nm for nm in cb.members if nm in tgt]
        if len(mem) < 2:
            continue
        sg = side[mem[0]]
        s_x = {nm: cb.se[nm][0] for nm in mem}
        rank = {nm: sg * tgt[nm] for nm in mem}        # smaller = nearer the spine
        V = {nm: [(np.asarray(p, float), np.asarray(q, float), L) for (p, q, L) in cb.virtual_of([nm])]
             for nm in mem}

        def meet(a, b):
            for (p, q, L) in V[a]:
                n = max(1, int(np.hypot(*(q - p)) / 0.025))
                P = p + (q - p) * np.linspace(0, 1, n + 1)[:, None]
                for (u, v, L2) in V[b]:
                    if L2 != L:
                        continue
                    d = v - u
                    l2 = float(d @ d)
                    t = np.clip(((P - u) @ d) / l2, 0.0, 1.0) if l2 > 1e-12 else np.zeros(len(P))
                    if np.min(np.hypot(*(P - u - t[:, None] * d).T)) < block:
                        return True
            return False
        new_ = 0
        for a in mem:
            for b in mem:
                # a peels off first and b rides inside it: a's leg crosses b
                if a != b and s_x[a] < s_x[b] - 1e-9 and rank[b] < rank[a] \
                        and a not in before.get(b, set()) and meet(a, b):
                    before.setdefault(b, set()).add(a)
                    new_ += 1
        if new_:
            log(f'  branch {cb.idx}: {new_} crossing leg(s) collide -- handed off with the first to peel inside')
        added += new_
    return added > 0


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


def clip_round_lines(pieces, lines, keep_r=None):
    """`pieces` [(p, q, layer)] with every stretch within keep_r of any of
    the segments `lines` [(a, b)] cut out (a point piece a == b is a bare
    end). Sampled along the piece at 0.05 mm: exact enough for a stamp."""
    if keep_r is None:
        keep_r = END_KEEP
    out = []
    for (p, q, lay) in pieces:
        L = math.hypot(q[0] - p[0], q[1] - p[1])
        n = max(2, int(L / 0.05) + 1)
        keep = []
        for i in range(n):
            t = i / (n - 1)
            x, y = p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])
            near = False
            for (a, b) in lines:
                dx, dy = b[0] - a[0], b[1] - a[1]
                L2 = dx * dx + dy * dy
                if L2 < 1e-12:
                    d = math.hypot(x - a[0], y - a[1])
                else:
                    u = max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / L2))
                    d = math.hypot(x - a[0] - u * dx, y - a[1] - u * dy)
                if d < keep_r:
                    near = True
                    break
            keep.append(not near)
        i = 0
        while i < n:
            if not keep[i]:
                i += 1
                continue
            j = i
            while j + 1 < n and keep[j + 1]:
                j += 1
            if j > i:
                ta, tb = i / (n - 1), j / (n - 1)
                out.append(((p[0] + ta * (q[0] - p[0]), p[1] + ta * (q[1] - p[1])),
                            (p[0] + tb * (q[0] - p[0]), p[1] + tb * (q[1] - p[1])), lay))
            i = j + 1
    return out


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
        rx, ry = p.size_x / 2 + 0.02, p.size_y / 2 + 0.02
        for (x, y) in seen:
            if abs(x - p.global_x) <= rx and abs(y - p.global_y) <= ry:
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
    legs broke at every bend (a gap outside, an overlap inside)."""
    out = []
    for L in ('F.Cu', 'B.Cu'):
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
                       if _owner(pt, segs, net.pads) == dest_ref]
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
                    at_dest = [pt for pt in free_v if _owner(pt, segs, net.pads) == dest_ref]
                if not at_src:
                    at_src = [pt for pt in free_v if pt not in at_dest
                              and _owner(pt, segs, net.pads) != dest_ref]
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
        # ...and a pad served UNDER another (a termination resistor on the
        # back side under a DDR ball, 0.06 mm further from the source than
        # the ball) is not the target: the ball on the ARRAY is
        for p in far:
            if p is not tgt and _pairs.under_pad(p, tgt, VIA_SIZE):
                tgt = p
                break
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
    value, or one per adjacent pair), symmetrically. A 60-sweep
    relaxation, deliberately: it does NOT reach the floor on a long comb
    (12 stubs 0.40 apart at floor 0.55: -0.003; 40 coincident at 0.38:
    -0.33), and the exact projection (isotonic regression) was measured
    8 vias WORSE at K41 on one fixed fanout board, both DRC-clean --
    `pair_floor` is a planning heuristic for lane room, not a clearance
    rule, and honouring it makes the planner more conservative."""
    py = list(vals)
    if len(py) < 2:
        return py
    fl = list(floor) if isinstance(floor, (list, tuple)) else [floor] * (len(py) - 1)
    for _ in range(60):
        moved = False
        for i in range(len(py) - 1):
            g_ = py[i + 1] - py[i]
            fl_i = fl[i]
            if g_ < fl_i - 1e-9:
                push = (fl_i - g_) / 2
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
        self.H = LPITCH * (n_m - 1) / 2 + LPITCH + sum(self.lane_w(nm) for nm in self.members)
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
            if PAGES_SIDERS and getattr(ctx, 'pages_first', False):
                dn = sp.d[-1]
                sd = ctx.stub_dir[nm]
                al = sd[0] * dn[0] + sd[1] * dn[1]
                if al > 0.7 or (PAGES_SIDERS == 1 and al > -0.5):
                    # mode 2 (default): a FAR-face stub is always a side exit
                    # (it rides round, outermost of its side) -- head-on
                    # through the array at its row is a different place in
                    # the order and the relative test flipped it. Mode 1:
                    # side-face stubs too (measured K35 +18 vias: their legs
                    # then ran on the other layer, 2 vias each).
                    return False
            return self._head_exit(nm, se[nm], self.stubs[nm], ctx.dest_layer[nm])

        if getattr(self, 'branch', False):
            # a BRANCH (BRAID_BRANCH): its teeth are the trunk's handoff
            # points, laid across its start, and every lane peels off to
            # its berth on a leg -- none runs head-on through the others
            self.heads_l = list(M)
            self.joiners = []
            self.heads_e = []
        else:
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
        side_of = self._side_of
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
        self.s_ball = None          # the destination's last ball along the spine (a candidate berth reads it)
        if self.siders and not getattr(self, 'wrap', False):
            refs = {ctx.ends[nm][2] for nm in self.siders}
            pads = [p for r in refs for p in ctx.pcb.footprints[r].pads]
            if pads:
                sb = [sp.project_pt((p.global_x, p.global_y))[0] for p in pads]
                s_ball = max(sb)
                r_max = max(max(p.size_x, p.size_y) / 2 for p in pads)
                self.s_ball, self.r_max = s_ball, r_max
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

    def _side_of(self, o_pt, ref):
        """Which side of the spine an offset lies on, seen from array
        `ref`'s centre: +1 at or beyond it, -1 short of it."""
        ps = self.ctx.pcb.footprints[ref].pads
        c = (sum(p.global_x for p in ps) / len(ps),
             sum(p.global_y for p in ps) / len(ps))
        _sc, oc = self.spine.project_pt(c)
        return 1 if o_pt >= oc else -1

    def _head_exit(self, nm, se_nm, stub_nm, dl_nm, record=True):
        """Is a stub of `nm` at (s, o) `se_nm` (board point `stub_nm`, on
        `dl_nm`) a HEAD-ON exit -- reached by a straight run in at its own
        offset -- given the other members' stubs as they stand? classify
        asks for every member; _alt_geo asks for a CANDIDATE berth."""
        ctx, sp, M = self.ctx, self.spine, self.members
        se = self.se
        s_i, o_i = se_nm
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
        return ctx.pad_obs.seg_clear(run_from, stub_nm)

    def _need_at(self, om, s, sched):
        """The layer a lane crossing `om` at s must be on: the opposite
        of om's page."""
        L = sched.page.get(om)
        return other_layer(L) if L else None

    def _alt_geo(self, nm, alt):
        """A candidate berth's place in the corridor: its stub's (s, o) on
        the spine, head-on or side exit by classify's own test against the
        other stubs as they stand, the side, far-face or not. A candidate
        TOOTH (alt['end'] == 'src') likewise at the launch end: head-on
        by head_launch's test against the other teeth, else a joiner on
        its side (_alt_src_slot)."""
        ctx, sp = self.ctx, self.spine
        pt = (float(alt['exit'][0]), float(alt['exit'][1]))
        L_a = alt['layer']
        so = sp.project_pt(pt)
        if alt.get('end') == 'src':
            s_t, o_t = so
            head = True
            for om in self.members:
                if om == nm:
                    continue
                s_j, o_j = self.st[om]
                if s_j > s_t + TOL_S and abs(o_j - o_t) < DIST_O:
                    head = False
                    break
            if head:
                run_end = sp.xy(self.s0, o_t)
                head = ctx.obs_but(nm, self.members, L_a).seg_clear(pt, run_end)
            sg = None if head else self._side_of(o_t, ctx.src_ref[nm])
            return dict(stub=pt, layer=L_a, se=so, head=head, side=sg, far=False, src=True)
        sdir = tuple(alt.get('dir') or ctx.stub_dir[nm])
        s_e, o_e = so
        ref = ctx.ends[nm][2]
        head = self._head_exit(nm, so, pt, L_a, record=False)
        if head and PAGES_SIDERS and getattr(ctx, 'pages_first', False):
            dn_ = sp.d[-1]
            al_ = sdir[0] * dn_[0] + sdir[1] * dn_[1]
            if al_ > 0.7 or (PAGES_SIDERS == 1 and al_ > -0.5):
                head = False                # PLAN_PAGES_SIDERS: a far-face (mode 1: or side-face) candidate is a side exit
        sg, far = None, False
        if not head:
            sg = self._side_of(o_e, ref)
            if self.s_ball is not None:
                dn = sp.d[-1]
                far = (s_e > self.s_ball - 1e-6
                       or sdir[0] * dn[0] + sdir[1] * dn[1] > 0.7)
        return dict(stub=pt, layer=L_a, se=so, head=head, side=sg, far=far, src=False)

    def _alt_src_slot(self, nm, g):
        """The LAUNCH slot a candidate tooth takes, the other lanes held:
        a head-on launch's own offset, else a place inserted into that
        side's join block by the block's rule (the first joiner along s
        takes the lane farthest out): between the two joiners its tooth
        falls between along the spine, else beyond the block's end."""
        s_t, o_t = g['se']
        if g['head']:
            return o_t
        sg = g['side']
        blk = sorted((self.st[om][0], self.join_block[om]) for om in self.join_block
                     if om != nm and self.join_side.get(om) == sg)
        if not blk:
            ext = max([sg * self.launch_o[om] for om in self.heads_l if om != nm] + [sg * o_t])
            return sg * (ext + BLOCK_GAP)
        S = [b[0] for b in blk]; O = [b[1] for b in blk]
        span = max(S[-1] - S[0], LPITCH)
        if s_t <= S[0]:
            return O[0] + sg * LPITCH * (1.0 + min(1.0, (S[0] - s_t) / span))
        if s_t >= S[-1]:
            return O[-1] - sg * LPITCH * min(1.0, (s_t - S[-1]) / span)
        for k in range(len(S) - 1):
            if S[k] <= s_t < S[k + 1]:
                f = (s_t - S[k]) / max(S[k + 1] - S[k], 1e-9)
                return O[k] + (O[k + 1] - O[k]) * (0.25 + 0.5 * f)
        return O[-1] - sg * LPITCH

    def _alt_slot(self, nm, g):
        """The target slot a candidate berth takes, the other lanes held:
        a head-on exit's own offset; a side exit's own slot when it stays
        in the block it is in; otherwise a place INSERTED into that side's
        comb by the comb's own rule (first exiter innermost): between the
        two members its stub falls between along the spine, at a fraction
        of their pitch by its stub's s, so two candidates from different
        nets never share a slot and a candidate's crossings with the
        region lanes are those of its place in the target order. (The
        block's next outer slot for every mover was tried first: several
        nets moving to one side shared one o_t.)"""
        s_e, o_e = g['se']
        if g['head']:
            return o_e
        sg = g['side']
        if nm in self.exit_block and self.exit_side.get(nm) == sg:
            return self.exit_block[nm]
        blk = sorted((self.se[om][0], self.exit_block[om]) for om in self.exit_block
                     if om != nm and self.exit_side.get(om) == sg)
        if not blk:
            ext = max([sg * self.target_o[om] for om in self.heads_e if om != nm] + [sg * o_e])
            return sg * (ext + BLOCK_GAP)
        S = [b[0] for b in blk]
        O = [b[1] for b in blk]
        span = max(S[-1] - S[0], LPITCH)
        if s_e <= S[0]:
            return O[0] - sg * LPITCH * min(1.0, (S[0] - s_e) / span)
        if s_e >= S[-1]:
            return O[-1] + sg * LPITCH * (1.0 + min(1.0, (s_e - S[-1]) / span))
        for k in range(len(S) - 1):
            if S[k] <= s_e < S[k + 1]:
                f = (s_e - S[k]) / max(S[k + 1] - S[k], 1e-9)
                return O[k] + (O[k + 1] - O[k]) * (0.25 + 0.5 * f)
        return O[-1] + sg * LPITCH

    def _head_order(self, he, se):
        """The head-on exits in target order: by berth offset, except that a
        lane whose berth lies further along a ROW of berths on its own layer
        (within ROW_O of each other across the spine) passes the nearer
        berths on their OUTER side -- away from the destination array, where
        their stubs run -- so its slot is outside theirs. By offset alone,
        two berths 0.03 mm apart across and 0.49 along (HHa SA4, SA2) got
        their slots the wrong way round and SA2's tail ran 0.06 mm from SA4's
        berth end, both refused in band every attempt; the human nests such a
        row, the farthest berth outermost."""
        ctx, sp = self.ctx, self.spine
        dl = ctx.dest_layer
        side = {}
        for nm in he:
            ps = ctx.pcb.footprints[ctx.ends[nm][2]].pads
            cen = (sum(p.global_x for p in ps) / len(ps), sum(p.global_y for p in ps) / len(ps))
            d = sp.project_pt(cen)[1] - se[nm][1]
            side[nm] = 0.0 if abs(d) < 1e-6 else math.copysign(1.0, d)
        base = sorted(he, key=lambda nm: se[nm][1])
        before = {nm: set() for nm in he}      # the lanes whose slots lie below this one's
        for a in he:
            for b in he:
                if a == b or dl[a] != dl[b] or not side[a]:
                    continue
                if se[b][0] > se[a][0] + 0.05 and abs(se[b][1] - se[a][1]) < ROW_O:
                    # b runs past a's berth on the side away from a's array
                    if side[a] > 0:
                        before[a].add(b)
                    else:
                        before[b].add(a)
        # ...and the slot each row member needs: a pitch outside every
        # nearer berth of its row and outside the slot of the lane nested
        # inside it, nearer berths first. u is the offset measured outward.
        up = {nm: set() for nm in he}            # the nearer berths of nm's row
        for a in he:
            for b in he:
                if a != b and (b in before[a] or a in before[b]) and se[a][0] < se[b][0]:
                    up[b].add(a)
        self.row_slot, self.row_side, self.row_up = {}, {}, up
        for nm in sorted((n for n in he if up[n] or any(n in up[m] for m in he)), key=lambda n: se[n][0]):
            sd = side[nm] or next((side[a] for a in up[nm] if side[a]), 1.0)
            self.row_side[nm] = sd
            u = -sd * se[nm][1]
            for a in up[nm]:
                u = max(u, -sd * se[a][1] + LPITCH)
                if a in self.row_slot:
                    u = max(u, -sd * self.row_slot[a] + LPITCH)
            self.row_slot[nm] = -sd * u
        # ready lanes (every lane that must lie below placed) in the order of
        # the slot each needs: two rows on the two layers at one place (the
        # fanout gives one net per layer a point) interleave by need, where by
        # berth offset the whole F row went outside the B row (HHa SA10/SDQ10)
        want = {nm: self.row_slot.get(nm, se[nm][1]) for nm in he}
        out, left = [], sorted(he, key=lambda nm: (want[nm], se[nm][1]))
        while left:
            nxt = next((nm for nm in left if not (before[nm] & set(left))), None)
            if nxt is None:
                out += left                      # a cycle: the offset order stands
                break
            out.append(nxt)
            left.remove(nxt)
        moved = [nm for a_, nm in zip(base, out) if a_ != nm]
        if moved:
            self.log('  head-on order by berth rows: ' + ', '.join(moved)
                     + ' (a farther berth on a row passes the nearer ones outside)')
        return out

    def _secant_floor(self, a, b, base, sched):
        """pair_floor's slope rule for two lanes whose lines are both reserved
        -- the page lanes' secant, even beside a swimmer (a berth row's swimmer
        has its tail reserved from s1 on)."""
        ms_ = [abs(self._slope.get(n_, 0.0)) for n_ in (a, b)
               if sched is not None and sched.page.get(n_) is not None]
        m_ = max(ms_) if ms_ else 0.0
        return (base + self.lane_w(a) + self.lane_w(b)) * math.sqrt(1.0 + m_ * m_)

    def _turn_in(self, nm, o_slot):
        """The s where a lane holding offset o_slot meets its berth's escape
        line (the stub's direction carried on past its free end), or None
        when that line runs along the spine or never reaches o_slot inside
        the tail."""
        sp = self.spine
        p = self.ctx.ends[nm][1]
        d = self.ctx.stub_dir[nm]
        s_e, o_e = sp.project_pt(p)
        s_f, o_f = sp.project_pt((p[0] + d[0] * 0.1, p[1] + d[1] * 0.1))
        d_s, d_o = (s_f - s_e) / 0.1, (o_f - o_e) / 0.1
        if abs(d_o) < 0.2 or d_s > -0.1:
            return None
        t = (o_slot - o_e) / d_o
        s_t = s_e + t * d_s
        if t <= 0 or not (self.s1 + 0.05 < s_t < self.se[nm][0] - 0.02):
            return None
        return s_t

    def _fit_block(self, base, sg, order, gaps, s_to):
        """A side block's slot gaps, squeezed toward the least a lane pair
        can route at (track + clearance and a hair, no secant) so its
        outermost slot runs ON the board from s1 to `s_to`. A block laid
        at the plan's comfortable pitch can run past the outline, where the
        router's edge fence walls every cell: HHa's south block put five
        lanes up to 1.8 mm off the board and all five refused, band or
        none. Unchanged when the block fits."""
        if not gaps:
            return gaps
        sp = self.spine
        inset = self._edge_inset()
        ss = [self.s1 + (s_to - self.s1) * t for t in (0.0, 0.5, 1.0)]

        def fits(o):
            return all(_on_board(self.ctx.pcb, *sp.xy(s, o), inset) for s in ss)
        far = base + sg * sum(gaps)
        if fits(far):
            return gaps
        # the farthest offset on the board, in 0.02 mm steps from the base
        lim, o = None, base
        while sg * (o - far) <= 1e-9 and fits(o):
            lim = o
            o += sg * 0.02
        if lim is None:
            self.log(f'  block on side {"+" if sg > 0 else "-"}: its first slot is off the board; left as laid')
            return gaps
        need = sg * (far - lim)
        mins = [self.pair_floor(order[k - 1], order[k], LANE_MIN, None, False)
                for k in range(1, len(order))]
        room = sum(max(g - m, 0.0) for g, m in zip(gaps, mins))
        lam = max(0.0, 1.0 - need / room) if room > 1e-9 else 0.0
        out = [m + max(g - m, 0.0) * lam for g, m in zip(gaps, mins)]
        self.log(f'  block on side {"+" if sg > 0 else "-"} squeezed {sum(gaps) - sum(out):.2f} mm onto the board '
                 f'({len(order)} lanes, width {sum(gaps):.2f} -> {sum(out):.2f}'
                 f'{", STILL OFF" if need > room + 1e-9 else ""})')
        return out

    def _edge_inset(self):
        cfg = self.ctx.cfg
        edge = float(getattr(cfg, 'board_edge_clearance', 0.0) or 0.0) or cfg.clearance
        return edge + TRACK / 2 + 0.05

    def _clear_block(self, base, sg, s_from, s_to, nm, step=0.1, tries=15):
        """Push a block's innermost lane outward until its run along
        the spine from s_from to s_to is clear of the static copper on
        both layers: a block is placed beyond the corridor's OWN teeth
        or stubs, but the same flank carries every other net's teeth
        too (K15: the first joiner lane ran 0.08 mm from a foreign stub
        end, and the router refused it)."""
        sp, ctx = self.spine, self.ctx
        base0 = base
        if not BLOCK_PUSH:
            return base
        cleared = False
        for k in range(tries):
            a, b = sp.xy(s_from, base), sp.xy(s_to, base)
            ok = {L: ctx.obs_but(nm, self.members, L).seg_clear(a, b)
                  for L in ('F.Cu', 'B.Cu')}
            if all(ok.values()):
                cleared = True
                break
            base += sg * step
        if BLOCK_PUSH >= 2 and not cleared:
            # ^ 2: a push that does NOT clear within its cap is no push.
            # The human bench's north block: the passive cluster north
            # of DU1 spans 4 mm of o, every one of the 15 steps found
            # copper and the block shipped 1.5 mm out for nothing --
            # every north lane a steeper diagonal, the far-face lanes
            # refused (K51: 8 north lanes, +11..13 vias against the
            # unpushed block). Left at its base, each lane is bent round
            # what its own run meets (deflect_islands / the early dive),
            # which is what routes it anyway.
            self.log(f'  block on side {"+" if sg > 0 else "-"} not pushed: '
                     f'static copper within {tries * step:.1f} mm at every '
                     f'offset (s {s_from:.1f}..{s_to:.1f})')
            return base0
        if abs(base - base0) > 1e-9:
            self.log(f'  block on side {"+" if sg > 0 else "-"} pushed '
                     f'{abs(base - base0):.2f} mm clear of static copper '
                     f'(s {s_from:.1f}..{s_to:.1f})')
        return base

    def _leg_s(self, nm, s_l, oa, ob, at_tooth, placed, avoid=None, extra=(),
               layer_only=False, own=None, own_L=None):
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
        # (`own` / `own_L`: a CANDIDATE berth's end and layer, _alt_pieces)
        if own is None:
            own = self.st[nm] if at_tooth else self.se[nm]
        if own_L is None:
            own_L = (self.ctx.tooth_layer if at_tooth else self.ctx.dest_layer)[nm]
        ends = []
        ends_L = []          # ...those on the jog's own layer
        # a free end is (s, o, s_lo, s_hi): a point for a single, and for
        # a PAIR member the span of its two tips along the spine -- the
        # pair's end is the whole stretch between its teeth (berths), so
        # no other leg may run between them (K36 SDQS1: SDQ10's exit leg
        # and corner via 0.4 mm from each berth, straight through the
        # gap the pair's legs converge across)
        prs = getattr(self.ctx, 'pairs', {}) or {}
        pe = getattr(self.ctx, 'pair_ends', {}) or {}
        for om in self.members:
            if om == nm:
                continue
            for k_, (p, L) in enumerate(((self.st[om], self.ctx.tooth_layer[om]),
                                         (self.se[om], self.ctx.dest_layer[om]))):
                if (L != own_L and abs(p[0] - own[0]) < 0.05
                        and abs(p[1] - own[1]) < 0.05):
                    continue
                s_lo = s_hi = p[0]
                if om in prs and om in pe:
                    ss = [self.spine.project_pt(q)[0] for q in pe[om][k_]]
                    s_lo, s_hi = min(ss), max(ss)
                e = (p[0], p[1], s_lo, s_hi)
                ends.append(e)
                if L == own_L:
                    ends_L.append(e)
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
        own_hw = ((-_pairs.pitch(TRACK) / 2, _pairs.pitch(TRACK) / 2) if nm in prs and BRANCH else (0.0,))
        # ...and a free end is measured from the leg's nearest CONDUCTOR, as
        # a placed leg is: from a pair's centreline, HHa SA6's tooth read
        # 0.346 mm clear of SCK's join leg and stood 0.21 from its outer leg
        own_ext = max(abs(h_) for h_ in own_hw)

        def clash(s):
            # a jog is a whole pitch, so an end or a leg exactly a pitch
            # away is clear -- compared with a tolerance, or the float
            # residue of 2.6 - 0.35 vs 1.6 + 0.35 reads as a clash and
            # sends the leg two pitches off, over the next tooth
            n = sum(1 for p in (ends_L if layer_only else ends)
                    if p[2] - end_clash - own_ext < s < p[3] + end_clash + own_ext
                    and lo_ - 0.05 < p[1] < hi_ + 0.05)
            # under BRAID_BRANCH a PAIR's leg is its two conductors, half the
            # pair pitch either side (placed legs are recorded that way too):
            # measured as one line at its centre, HHa's SCK leg left SODT1's
            # leg on SCKN's (off the branch frame it moved zynq's pair plan
            # for the worse, and the leg stays one line)
            n += sum(1 for (ps, plo, phi) in placed
                     if min(abs(ps - s - h_) for h_ in own_hw) < TRACK + 0.1 + 0.02
                     and plo < hi_ and phi > lo_)
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
            return any(p[2] < hi_s and p[3] > lo_s and abs(p[1] - own[1]) < LEG_O
                       for p in ends_L)
        def room(s):
            # the leg's room: its distance to the nearest foreign free
            # end in its span or leg already placed
            d = [max(max(p[2] - s, s - p[3], 0.0) - own_ext, 0.0) for p in ends if lo_ - 0.05 < p[1] < hi_ + 0.05]
            d += [min(abs(ps - s - h_) for h_ in own_hw) for (ps, plo, phi) in placed if plo < hi_ and phi > lo_]
            return min(d) if d else 1e9

        def too_close(s):
            # ...and a leg with less than the legal minimum of it is
            # not a candidate at all: the min-clash rule took the first
            # of four equally clashing candidates and put SA8's join
            # leg 0.007 mm from SA5's tooth (K41), a stamp on both
            # layers over the tooth -- SA5 refused at its first cell
            # every attempt
            return room(s) < LANE_MIN
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

    def lane_w(self, nm):
        """How far a member's copper reaches beyond its centreline on
        either side, over a single's: half the pair pitch for a PAIR member
        (pairs.py), nothing for a single. Added to every slot pitch the
        member takes part in, so a pair owns the room of its two legs."""
        if nm not in getattr(self.ctx, 'pairs', {}):
            return 0.0
        # ...and not less than its DIVE needs: the pair's two barrels stand
        # side by side across the lane, an envelope via of 2 * via_half +
        # via_size, and the neighbouring single's centreline must clear it
        # by clearance + half a track. Measured (K36 refusal picture): at
        # half the pair pitch the dive fitted nowhere in-band and every pair
        # landed only in the free window at last call.
        half = _pairs.pitch(TRACK) / 2.0
        via_half = max((VIA_SIZE + CLEAR) / 2.0, (VIA_SIZE / 2.0 + CLEAR + TRACK / 2.0 - half) / 0.7071 + 0.005)
        env_r = via_half + VIA_SIZE / 2.0
        return max(half, env_r + CLEAR + TRACK / 2.0 - LPITCH)

    def pair_floor(self, a, b, base, sched, at_launch, sg=1.0):
        """The offset pitch two adjacent slots need, `b` standing on the `sg`
        side of `a` (+1: the larger offset). Clearance is perpendicular to a
        lane and a slot pitch is measured across the spine: a slot's POINT --
        a lane's bend at its slot, a via born or landing there, a swimmer's
        stamped end -- is passed by the neighbour's line leaving the slot (at
        the launch; reaching it, at the target) at the pitch times the cosine
        of THAT line's angle, and only when that line runs toward the point
        (a line running away passes it wider than the pitch). So two lanes of
        one page need the base pitch times the secant of whichever runs toward
        the other (K28: at 45 degrees a 0.35 pitch is 0.25 of copper room);
        lanes on different pages need room only for a via at a slot -- a lane
        born (or landing) on the other layer there -- at a via's clearance
        times the secant of the neighbour running toward it; a swimmer's line
        is no promise, but its stamped end is, beside a page lane on its layer.
        Taken from the steeper lane regardless of direction, HHa's SA4 -- born
        F->B at its slot, SDQ7 beside it running AWAY -- needed 0.896 mm, was
        pushed 0.93 mm the wrong way and folded 104 degrees at s0."""
        base = base + self.lane_w(a) + self.lane_w(b)
        if sched is None or not SLOPE_PITCH:
            return base
        pa, pb = sched.page.get(a), sched.page.get(b)
        end = self.ctx.tooth_layer if at_launch else self.ctx.dest_layer
        d_ = 1.0 if at_launch else -1.0

        def toward(x, side):
            # the secant of x's line where it runs toward `side` near the slot
            m = self._slope.get(x, 0.0) * d_
            return math.sqrt(1.0 + m * m) if m * side > 0 else 1.0
        sa, sb = toward(a, sg), toward(b, -sg)       # a's line toward b's point, b's toward a's
        if pa is None or pb is None:
            if pa is None and pb is None:
                return base
            sw_, pg_ = (a, b) if pa is None else (b, a)
            if end[sw_] != sched.page[pg_]:
                return base
            return base * (sa if pg_ == a else sb)
        if pa == pb:
            return base * max(sa, sb)
        need = base
        if end[a] != pa:
            need = max(need, VIA_NEED * sb)
        if end[b] != pb:
            need = max(need, VIA_NEED * sa)
        return need

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
                Ly.append(v if not Ly else max(v, Ly[-1] + self.pair_floor(hl[len(Ly) - 1], nm, ly_floor, None, True)))
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
                if nm in (getattr(self.ctx, 'pairs', None) or {}) and BRANCH:
                    # a PAIR's join leg is its two conductors, as its exit leg's is
                    h_ = _pairs.pitch(TRACK) / 2
                    placed.append((s_l - h_, min(st[nm][1], far), max(st[nm][1], far)))
                    placed.append((s_l + h_, min(st[nm][1], far), max(st[nm][1], far)))
                else:
                    placed.append((s_l, min(st[nm][1], far), max(st[nm][1], far)))
            js.sort(key=lambda nm: (self.join_leg_s[nm], st[nm][0]))
            base = self._clear_block(base, sg, min(self.join_leg_s[nm] for nm in js),
                                     self.s0, js[0])
            # (the block's pitch as each pair's own floor -- the exit block's
            # rule -- was tried and made the joined lanes CONVERGE onto the
            # exit block's tighter pitch: K51 SRAS walled by SWE and SA13 at
            # s 11.5. The recorded LPITCH stands.)
            offs = [0.0]
            for k in range(1, len(js)):
                offs.append(offs[-1] + self.pair_floor(js[k - 1], js[k], LPITCH, None, True))
            for k, nm in enumerate(js):
                launch_o[nm] = base + sg * (offs[-1] - offs[k])
                self.join_block[nm] = launch_o[nm]
        # head-on exits: stub offsets at the exit pitch floor
        # a PINNED end (a trunk's handoff, BRAID_BRANCH) is its own slot:
        # the side block that laid it had the floors and the board, and
        # relaxed again here the trunk spread it back past the outline
        pin = getattr(self, 'pin_target', None) or ()
        he = self._head_order([nm for nm in self.heads_e if nm not in pin], se)
        fl_he = [self.pair_floor(he[i], he[i + 1], MINP, sched, False) for i in range(len(he) - 1)]
        dl_ = self.ctx.dest_layer
        for i in range(len(he) - 1):
            a_, b_ = he[i], he[i + 1]
            if a_ in self.row_slot and b_ in self.row_slot and dl_[a_] != dl_[b_]:
                # two rows on the two layers at one place share their offsets:
                # only a lane landing on a layer other than its page's needs a
                # via's room beside the other (the fanout gives one net per layer
                # a point, and a full exit pitch between them pushed HHa's B row
                # 0.8 mm out, over the side exits' block)
                via_ = any(sched is not None and sched.page.get(n_) not in (None, dl_[n_]) for n_ in (a_, b_))
                fl_he[i] = VIA_NEED if via_ else 0.05
            elif (sched is not None and (a_ in self.row_slot or b_ in self.row_slot)
                  and (sched.page.get(a_) is None) != (sched.page.get(b_) is None)):
                # a row's SWIMMER has its tail reserved from s1 on, so the page
                # lane beside it still needs its secant: HHa's SA4 arrives at its
                # slot at slope ~2.3 across the bundle, and 0.35 across the spine
                # left 0.11 mm beside SDQ0's tail -- SA4's two searches stopped
                # 0.1 mm apart there
                pg_ = a_ if sched.page.get(a_) is not None else b_
                m_ = abs(self._slope.get(pg_, 0.0))
                fl_he[i] = max(fl_he[i], (MINP + self.lane_w(a_) + self.lane_w(b_)) * math.sqrt(1.0 + m_ * m_))
        py = _relax_pitch([self.row_slot.get(nm, se[nm][1]) for nm in he], fl_he if he else MINP)
        if self.row_slot:
            # a row lane's slot is a BOUND (at least this far out), not a
            # preference: the relaxation spreads symmetrically and pulled HHa's
            # SA4 and SDQ0 back inside the row. Clamped out, the floors are then
            # restored by moving only the lanes further out.
            for i, nm in enumerate(he):
                b = self.row_slot.get(nm)
                if b is not None:
                    py[i] = min(py[i], b) if self.row_side[nm] > 0 else max(py[i], b)
            lo_i = [i for i, nm in enumerate(he) if self.row_side.get(nm, 0) > 0]
            hi_i = [i for i, nm in enumerate(he) if self.row_side.get(nm, 0) < 0]
            # two row lanes of ONE layer with the other layer's between them
            # keep their own floor: the neighbour floors let HHa's SA4 and SDQ0
            # (B, SA10 on F between) arrive 0.11 mm apart
            same_row_floor = lambda a_, b_: self._secant_floor(a_, b_, MINP, sched)
            if lo_i:
                for i in range(max(lo_i) - 1, -1, -1):
                    b_ = py[i + 1] - fl_he[i]
                    if he[i] in self.row_slot:
                        j = next((j for j in range(i + 1, len(he)) if he[j] in self.row_slot
                                  and dl_[he[j]] == dl_[he[i]]), None)
                        if j is not None:
                            b_ = min(b_, py[j] - same_row_floor(he[i], he[j]))
                    py[i] = min(py[i], b_)
            if hi_i:
                for i in range(min(hi_i) + 1, len(py)):
                    b_ = py[i - 1] + fl_he[i - 1]
                    if he[i] in self.row_slot:
                        j = next((j for j in range(i - 1, -1, -1) if he[j] in self.row_slot
                                  and dl_[he[j]] == dl_[he[i]]), None)
                        if j is not None:
                            b_ = max(b_, py[j] + same_row_floor(he[i], he[j]))
                    py[i] = max(py[i], b_)
        target_o = {nm: py[i] for i, nm in enumerate(he)}
        target_o.update({nm: se[nm][1] for nm in self.heads_e if nm in pin})
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
            if getattr(self, 'keep_order', False):
                # a BRANCH (BRAID_BRANCH) keeps the order its lanes arrive in
                # from the trunk: its peel-off legs cross the lanes still
                # inside by layer, as a human ring's do
                order = sorted(xs, key=lambda nm: sg * launch_o[nm])
            ext = max([sg * v for v in py] + [sg * se[nm][1] for nm in xs])
            gap_ = BLOCK_GAP
            edge = max(he, key=lambda n_: sg * target_o[n_]) if he else None
            if edge is not None and edge in getattr(self, 'row_slot', {}):
                # a berth row's outermost lane and the block beyond it both cross
                # the bundle steeply to reach the far side: a flat gap between
                # them was 0.15 mm of copper room (HHa SA2 / SA0, slope ~2.5)
                gap_ = max(gap_, self._secant_floor(edge, order[0], LPITCH, sched))
            base = sg * max(ext + gap_, -LPITCH * (len(xs) - 1) / 2)
            # a side that leaves on a BRANCH (BRAID_BRANCH) rides round the
            # destination on the branch's own spine, never along the straight
            # run the push clears: HHa's south block went its full 1.5 mm out
            # for a run no lane takes, five of its slots off the board
            # (a branch's own block is its launch offsets, set below)
            in_branch = getattr(self, 'branch', False)
            to_branch = BRANCH and len(xs) >= 2 and not in_branch
            s_run = self.s1 + HAND_DS if to_branch else max(se[nm][0] for nm in xs)
            if not (to_branch or in_branch):
                base = self._clear_block(base, sg, self.s1, s_run, order[0])
            gaps = [self.pair_floor(order[k - 1], order[k], LPITCH, sched, False, sg)
                    for k in range(1, len(order))]
            if not in_branch:
                gaps = self._fit_block(base, sg, order, gaps, s_run)
            acc = 0.0
            for k, nm in enumerate(order):
                if k:
                    acc += gaps[k - 1]
                target_o[nm] = base + sg * acc
                self.exit_block[nm] = target_o[nm]
        if getattr(self, 'branch', False):
            # a BRANCH carries its lanes on exactly as they arrive: launch
            # at the handoff's own offset (no re-spacing -- the trunk spaced
            # them), target = launch (no re-sort; the peel-off legs cross by
            # layer), the block slot the lane's own. Corridor's own rules
            # moved them up to 3.8 mm inside a 0.9 mm ribbon (HHa south)
            self.s0 = self.s0_base
            launch_o = {nm: st[nm][1] for nm in self.members}
            target_o = dict(launch_o)
            self.exit_block = {nm: launch_o[nm] for nm in self.siders}
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

    def _round_teeth_ahead(self, nm, s_a, o_a, s_b, o_b):
        """Waypoints taking a head-on lane's fan-in ROUND the teeth that stand
        ahead of its own on its layer, a pitch off each. Teeth on one face of
        the array lie one behind another along s, and the straight run from a
        rear tooth to its slot crossed the front tooth itself: zynq K44 DQ2's
        line passed 0.047 mm over DQS0_N's tooth on the BGA's south row, the
        tooth walled by DQ2's reservation until DQ2 was routed. A tooth facing
        across the corridor is passed on the side it faces -- its stub runs
        back from it -- any other on the side of the lane's own slot."""
        L = self.ctx.tooth_layer.get(nm)
        ahead = sorted((self.st[om][0], self.st[om][1], om) for om in self.members
                       if om != nm and om in self.st and s_a + 1e-6 < self.st[om][0] < s_b - 1e-6
                       and self.ctx.tooth_layer.get(om) == L)
        out = []
        for s_k, o_k, om in ahead:
            p = TRACK + CLEAR + 0.04 + self.lane_w(nm) + self.lane_w(om)
            if s_k - s_a < p:
                continue          # beside it (a tooth column), not ahead of it
            o_line = o_a + (o_b - o_a) * (s_k - s_a) / max(s_b - s_a, 1e-9)
            if abs(o_line - o_k) >= p:
                continue
            side = 1.0 if o_b >= o_k else -1.0
            d = (self.ctx.tooth_dir or {}).get(om)
            if d is not None:
                t_ = self.ctx.ends[om][0]
                s1_, o1_ = self.spine.project_pt((t_[0] + 0.1 * d[0], t_[1] + 0.1 * d[1]))
                s0_, o0_ = self.spine.project_pt(t_)
                if abs(o1_ - o0_) > abs(s1_ - s0_):
                    side = 1.0 if o1_ > o0_ else -1.0
            # a pitch PERPENDICULAR to the run into the waypoint, which slopes:
            # the offset at the tooth's s times the run's secant
            ds_ = max(s_k - s_a, 1e-6)
            h = p
            for _ in range(6):
                h = min(3.0 * p, p * math.hypot(ds_, o_k + side * h - o_a) / ds_)
            o_w = o_k + side * h
            out.append((s_k, o_w))
            s_a, o_a = s_k, o_w
        return out

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

    def _branch_inner(self, nm):
        """A branch lane's inner neighbours, innermost first, with the gap
        each one's peel-off gives back to the lanes outside it."""
        sg = self.exit_side[nm]
        blk = sorted((m for m in self.exit_block if self.exit_side[m] == sg),
                     key=lambda m: sg * self.launch_o[m])
        r = blk.index(nm)
        return [(blk[k], abs(self.launch_o[blk[k + 1]] - self.launch_o[blk[k]])) for k in range(r)], sg

    def _branch_ramps(self, nm):
        """(s where the ramp starts, gap) for each inner lane's peel-off.
        The lanes outside a peeling lane move in TOGETHER, so their ramps
        must be one family of parallel offsets: a 45-degree jog's corner
        sits tan(22.5) x its distance further along for each lane further
        out. Started at one s, two neighbours on the ramp were gap x
        cos(45) apart -- HHa's south ring 0.171..0.185 mm at a 0.25 gap,
        under the 0.232 a track and its clearance need, a wall across ten
        lanes -- and the first one cut the peeling lane's leg corner."""
        inner, _sg = self._branch_inner(nm)
        T = math.tan(math.radians(22.5))
        out = []
        for i, (m, g) in enumerate(inner):
            e_m = self.exit_leg_s[m]
            # the o-distance from m to this lane when m peels: every gap
            # between them, less those of the lanes between that are gone
            d = sum(g_k for (m_k, g_k) in inner[i:])
            d -= sum(g_k for (m_k, g_k) in inner[i + 1:] if self.exit_leg_s[m_k] < e_m)
            out.append((e_m + T * max(d, 0.0), g))
        return out

    def _order_ring(self, ds=0.025):
        """A BRANCH's ring lanes never cross -- only the peel-off legs cross
        lanes, by layer -- so after the ramps and the island bends every
        lane is lifted until the lane inside it, all of that lane's line up
        to its leg, is a pitch away: a disc, not the offset at the same s,
        so a sloped stretch keeps its perpendicular room. The island bends
        were each lane's own: HHa's SA7 swung out round C10 across SA9 while
        SA9 was still on the ring, and SRST's return, clipped at its leg,
        ran 2.3 mm on top of SA7's. Lifts only; a ring laid at the trunk's
        gaps is already clear and does not move."""
        M = [nm for nm in self.members if nm in self.exit_block and self.mid.get(nm)]
        if len(M) < 2:
            return
        sg = self.exit_side[M[0]]
        order = sorted(M, key=lambda nm: sg * self.launch_o[nm])
        s_lo = min(self.mid[nm][0][0] for nm in M)
        s_hi = max(self.exit_leg_s[nm] for nm in M)
        S = np.arange(s_lo, s_hi + ds, ds)
        U = {}
        for nm in order:
            ms_ = self.mid[nm]
            u = sg * np.interp(S, [q[0] for q in ms_], [q[1] for q in ms_])
            u[(S < ms_[0][0] - 1e-9) | (S > self.exit_leg_s[nm] + 1e-9)] = np.nan
            U[nm] = u
        # a lane that changes layer at its leg's corner puts a VIA there,
        # which needs a via's room to the lanes either side, not a track's
        # (HHa SA9: its F->B corner one track pitch outside SA7's F line)
        def corner_via(nm):
            e = self.exit_leg_s[nm]
            prof = self.layer_profile(nm)
            before = [L for (s_, L) in prof if s_ < e - 1e-6]
            Lg = self.leg_layer.get(nm, self.ctx.dest_layer[nm])
            return (before[-1] if before else prof[0][1]) != Lg
        cv = {nm: corner_via(nm) for nm in order}

        # ...at its corner, or -- where another lane's leg (either layer; a
        # via is on both) crosses the corner's offset within a via's room --
        # at the latest s of its approach with that room, past every stretch
        # a rule pins its layer: HHa SCK's F->B corner stood 0.16 mm from
        # SODT1's F leg, both legs pinned straight to berths at one point
        def dive_s(nm):
            e = self.exit_leg_s[nm]
            pins = [xb for (xa, xb, _L) in self.req.get(nm, ()) if xa < e]
            lo = max(pins) + BIRTH_W if pins else S[0]
            legs_ = [(self.legs[om][-1], VIA_NEED + self.lane_w(nm) + self.lane_w(om))
                     for om in self.members if om != nm and self.legs.get(om) and om in self.exit_leg_s]
            u = U[nm]
            for k_ in range(int(np.searchsorted(S, e + 1e-9, side='right')) - 1, -1, -1):
                if S[k_] < lo:
                    break
                if not np.isfinite(u[k_]):
                    continue
                o_v = sg * u[k_]
                if not any(abs(s_o - S[k_]) < R_ and min(oa, ob) - R_ < o_v < max(oa, ob) + R_
                           for (s_o, oa, ob), R_ in legs_):
                    return float(S[k_])
            return e
        sv = {nm: dive_s(nm) for nm in order if cv[nm]}
        self.dive_s_plan = dict(sv)

        def dil(uj, R):
            out = np.full(len(S), -np.inf)
            n = int(R / ds)
            for t in range(-n, n + 1):
                lift = math.sqrt(max(R * R - (t * ds) ** 2, 0.0))
                sh = np.full(len(S), np.nan)
                if t >= 0:
                    sh[:len(S) - t] = uj[t:]
                else:
                    sh[-t:] = uj[:t]
                out = np.fmax(out, sh + lift)
            return out
        # ...and every static island on a layer the lane MUST take there is a
        # floor as well: the ring tightens toward the array exactly where the
        # passives sit (HHa SDQ3 slid onto C12.2 between the three points the
        # island bend samples, as SDQ5/SDQM1/SDQ6 peeled inside it)
        isl = self.static_islands()
        other_L = {'F.Cu': 'B.Cu', 'B.Cu': 'F.Cu'}
        PV = {nm: {L: np.asarray(self.planned_vec(nm, S, L), dtype=bool) for L in ('F.Cu', 'B.Cu')} for nm in order}
        moved = []
        for k, nm in enumerate(order):
            floor = np.full(len(S), -np.inf)
            e_nm = self.exit_leg_s[nm]
            u0 = sg * self.launch_o[nm]
            for L, boxes in isl.items():
                must = self.allowed_vec(nm, S, L) & ~self.allowed_vec(nm, S, other_L[L])
                for (b_lo, b_hi, o_lo, o_hi, _w) in boxes:
                    u_lo, u_hi = sorted((sg * o_lo, sg * o_hi))
                    if u0 <= u_hi:
                        continue          # starts on or inside it: the island bend's business
                    if e_nm < b_lo:
                        continue          # peels before it: its shoulder lifted HHa SA6 1.4 mm
                                          # in front of C10 for nothing, across SA5's corner
                    d_ = np.maximum(np.maximum(b_lo - S, S - b_hi), 0.0)
                    fl = u_hi + 0.02 - d_           # its side, and 45-degree shoulders
                    floor = np.where(must & (fl > u_lo), np.fmax(floor, fl), floor)
            if cv[nm]:
                # ...and a corner that changes layer puts a VIA there, on both
                # layers: a via's room off every static island of EITHER layer
                # (the boxes are grown for a track centre; a via centre needs
                # half a via more than half a track). HHa SCAS: its F->B corner
                # 0.148 mm from pad C6.1, and no legal site on its approach
                k_e = max(0, int(np.searchsorted(S, sv[nm] + 1e-9, side='right')) - 1)
                dv = (VIA_SIZE - TRACK) / 2 + 0.02
                for L, boxes in isl.items():
                    for (b_lo, b_hi, o_lo, o_hi, _w) in boxes:
                        u_lo, u_hi = sorted((sg * o_lo, sg * o_hi))
                        if u0 <= u_hi or not (b_lo - dv <= sv[nm] <= b_hi + dv):
                            continue
                        if np.isfinite(U[nm][k_e]) and U[nm][k_e] < u_hi + dv:
                            floor[k_e] = max(floor[k_e], u_hi + dv)
            for om in order[:k]:
                w_ = self.lane_w(om) + self.lane_w(nm)
                p = LANE_MIN + w_
                R = VIA_NEED + w_
                uj = U[om]
                # each LAYER keeps its own order (the human's ring: the layers
                # interleave in space): a line holds this lane off only where
                # the two share a layer in the plan. Held off on both, HHa's SA9
                # (B) rode out round SA7's swing past C10 -- SA7 on F there,
                # under SA9's own leg -- and turned back 135 degrees into it
                share = (PV[nm]['F.Cu'] & PV[om]['F.Cu']) | (PV[nm]['B.Cu'] & PV[om]['B.Cu'])
                floor = np.fmax(floor, dil(np.where(share, uj, np.nan), p))
                if cv[nm]:
                    near = (S >= sv[nm] - R) & (S <= min(sv[nm] + R, e_nm) + 1e-9)
                    floor = np.where(near, np.fmax(floor, dil(uj, R)), floor)
                if cv[om]:
                    e_om = sv[om]
                    # the last sample at or before its leg (U is NaN past it: the
                    # nearest sample dropped 5 of 12 corner discs; audit B)
                    k_ = max(0, int(np.searchsorted(S, e_om + 1e-9, side='right')) - 1)
                    if np.isfinite(uj[k_]):
                        d_ = np.abs(S - e_om)
                        lift = np.sqrt(np.maximum(R * R - d_ * d_, 0.0))
                        floor = np.where(d_ <= R, np.fmax(floor, uj[k_] + lift), floor)
            u = U[nm]
            need = np.isfinite(u) & (floor > u + 1e-6)
            if not need.any():
                continue
            u2 = np.where(need, floor, u)
            # ...lifted as a LANE, not as the floor: a disc's floor ends in a
            # vertical edge, so the lane stepped off one vertically and dipped
            # between two (HHa SCK: o 1.345 -> 1.166 in 0.025 mm of s and back
            # up, a fold no router lays and a pair cannot be offset from). No
            # stretch steeper than 45 degrees either way, by lifting only -- a
            # lift keeps every room the floor gave -- and the lane's first
            # point, where it arrives from the trunk, held
            ix = np.where(np.isfinite(u2))[0]
            v = u2[ix].copy()
            for i in range(1, len(v)):
                v[i] = max(v[i], v[i - 1] - ds)
            for i in range(len(v) - 2, 0, -1):
                v[i] = max(v[i], v[i + 1] - ds)
            # ...and no dip it climbs straight back out of: a stretch below the
            # lower of its two rims, narrower than RING_DIP, is lifted level
            # with that rim (SCK ran down at 45 degrees into its via's floor
            # 0.025 mm under its edge and back up: a notch)
            n_dip = max(1, int(round(RING_DIP / ds)))
            for _ in range(len(v)):
                changed = False
                i = 1
                while i < len(v) - 1:
                    if not (v[i] < v[i - 1] - 1e-9 and v[i] <= v[i + 1] + 1e-9):
                        i += 1
                        continue
                    l_, r_ = i, i
                    while l_ > 0 and v[l_ - 1] >= v[l_] - 1e-9:
                        l_ -= 1
                    while r_ < len(v) - 1 and v[r_ + 1] >= v[r_] - 1e-9:
                        r_ += 1
                    if l_ == i or r_ == i:
                        i += 1
                        continue
                    level = min(v[l_], v[r_])
                    a_, b_ = i, i
                    while a_ > l_ and v[a_ - 1] < level - 1e-9:
                        a_ -= 1
                    while b_ < r_ and v[b_ + 1] < level - 1e-9:
                        b_ += 1
                    if b_ - a_ + 1 <= n_dip:
                        v[a_:b_ + 1] = level
                        changed = True
                    i = b_ + 1
                if not changed:
                    break
            u2[ix] = v
            need = np.isfinite(u2) & (u2 > u + 1e-6)
            U[nm] = u2
            moved.append((nm, float(np.nanmax(np.where(need, floor - u, np.nan)))))
            live = np.isfinite(u2)
            e = self.exit_leg_s[nm]
            pts = [(float(s), float(sg * v)) for s, v in zip(S[live], u2[live]) if s < e - 1e-9]
            # the lane's own leg starts where the lifted ring ends: the old
            # end point kept at the leg's s drew a jog back across the lane
            # inside (HHa SA9: 1.92 -> 1.36 on F over SA7)
            o_e = float(sg * np.interp(e, S[live], u2[live]))
            pts.append((e, o_e))
            keep = [q for q in self.mid[nm] if q[0] < pts[0][0] - 1e-9 or q[0] > e + 1e-9]
            self.mid[nm] = sorted(keep + _simplify_so(pts, 0.004), key=lambda q: q[0])
            if self.legs.get(nm):
                s_l, _oa, ob = self.legs[nm][-1]
                self.legs[nm][-1] = (s_l, o_e, ob)
        if moved:
            self.log('  ring ordered: ' + ', '.join(f'{nm} +{d:.2f}' for nm, d in moved))

    def _branch_breaks(self, nm):
        out = []
        for s_m, g in self._branch_ramps(nm):
            out += [s_m, s_m + g]
        return out

    def _branch_o(self, nm):
        _inner, sg = self._branch_inner(nm)
        ramps = self._branch_ramps(nm)
        o0 = self.launch_o[nm]

        def o_at(s):
            d = 0.0
            for s_m, g in ramps:
                d += g * min(1.0, max(0.0, (s - s_m) / max(g, 1e-6)))
            return o0 - sg * d
        return o_at

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
                if nm in (getattr(self.ctx, 'pairs', None) or {}) and BRANCH:
                    h_ = _pairs.pitch(TRACK) / 2
                    placed.append((s_l - h_, min(o_l, o_e), max(o_l, o_e)))
                    placed.append((s_l + h_, min(o_l, o_e), max(o_l, o_e)))
                else:
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
                        # ...a lane within the leg tolerance of the leg's
                        # OWN slot is crossed too (two-page combs put a
                        # lane a hair from another's slot: K28 SDQM1's F
                        # leg started 0.02 mm from SDQM0's F lane, counted
                        # as crossing nothing, and both were refused)
                        near = (abs(o_m - o_l) < LEG_O)
                        if s_end > s_l + 0.05 and (lo_ < o_m < hi_ or near):
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
                if before:
                    return before[-1][2]
                return sched.page.get(om) if sched else None

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
                return min(ISLAND_VETO, JOG_VIA * abs(s_m - s_l) / LPITCH)

            def leg_clash(nm, s_l, L):
                """Another member's copper this leg would LIE ON if laid on L: a
                leg already decided on L, or a berth stub on L, within a lane
                pitch (pair widths included) in s and inside the leg's reach in
                o. The crossed-lane count cannot see it -- HHa: SODT1's F berth
                and SCKN's B berth share one point, and SODT1's leg, sent down
                on B for being cheaper, lay 0.16 mm from SCK's B leg."""
                o_l, o_e = py[trank[nm]], self.se[nm][1]
                lo_, hi_ = min(o_l, o_e) - LEG_O, max(o_l, o_e) + LEG_O
                for om in M:
                    if om == nm:
                        continue
                    sep_ = LPITCH + self.lane_w(nm) + self.lane_w(om)
                    if om in decided_ and self.leg_layer.get(om) == L and om in self.exit_leg_s:
                        s_o = self.exit_leg_s[om]
                        a_, b_ = sorted((py[trank[om]], self.se[om][1]))
                        if abs(s_o - s_l) < sep_ and a_ < hi_ and b_ > lo_:
                            return om
                    if (getattr(self, '_plan_dest', None) or self.ctx.dest_layer)[om] == L:
                        s_o, o_o = self.se[om]
                        if abs(s_o - s_l) < sep_ and lo_ < o_o < hi_:
                            return om
                return None

            decided_ = set()       # legs whose layer this pass has decided (the rest hold a side's placeholder)
            for nm in sorted(self.exit_block, key=lambda n: self.exit_leg_s[n]):
                s_l = self.exit_leg_s[nm]
                own = cur_layer(nm, s_l)
                crossed = leg_cross.get(nm, ())
                cost = {}
                for L in ('F.Cu', 'B.Cu'):
                    c = 0
                    clash_ = leg_clash(nm, s_l, L)
                    if clash_ is not None:
                        c += ISLAND_VETO
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
                decided_.add(nm)
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
            self.log('  exit legs: ' + ', '.join(
                f'{nm}@s{self.exit_leg_s[nm]:.1f} {self.leg_layer[nm][0]}'
                f'{"x" + str(len(leg_cross[nm])) if nm in leg_cross else ""}'
                for nm in sorted(self.exit_block, key=lambda n: self.exit_leg_s[n])))
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
        # CORNER ROOM. A leg whose corner changes layer puts a via at that
        # corner, on both layers, so every other leg across the corner's
        # offset keeps a via's room from it, whatever its own layer. Placed
        # before any layer was known, legs of different layers sit a track
        # apart: HHa's SODT1 F leg ran 0.16 mm from SCK's F->B corner, and no
        # site on SCK's whole approach was legal (plan audit: pitch 0.073,
        # dive 0.160). The later leg of each such pair (in placement order) is
        # placed again a via's room off the other, and every layer decided
        # again from the moved legs -- twice at most.
        def arriving(nm):
            before = [iv for iv in ivs.get(nm, ()) if iv[0] < self.exit_leg_s[nm]]
            if before:
                return before[-1][2]
            return sched.page.get(nm) if sched else None

        def porder(n):
            if n in self.far_exit:
                return (1, abs(self.exit_block[n]), self.se[n][0])
            return (0, self.se[n][0], abs(self.exit_block[n]))
        isl_avoid = ((lambda nm, s_: nm in layer0 and nm in bad_legs and leg_on_island(nm, s_, layer0[nm]))
                     if bad_legs else None)
        room_block = {}
        stuck = set()              # (mover, keep): a leg that could not move; the other moves instead
        for _rnd in range(3):
            found = []
            for nm in self.exit_block:
                La = arriving(nm)
                if La is None or La == self.leg_layer[nm]:
                    continue
                s_c, o_c = self.exit_leg_s[nm], py[trank[nm]]
                for om in self.exit_block:
                    if om == nm:
                        continue
                    R = VIA_NEED + self.lane_w(nm) + self.lane_w(om)
                    a_, b_ = sorted((py[trank[om]], self.se[om][1]))
                    if abs(self.exit_leg_s[om] - s_c) < R - 1e-6 and a_ - R < o_c < b_ + R:
                        found.append((nm, om, R))
            if not found:
                break
            room_block = {}
            for nm, om, R in found:
                mover, keep = (om, nm) if porder(om) > porder(nm) else (nm, om)
                if (mover, keep) in stuck:
                    mover, keep = keep, mover
                room_block.setdefault(mover, []).append((self.exit_leg_s[keep], R, keep))
            was = {mv: self.exit_leg_s[mv] for mv in room_block}
            ivs, leg_req_min = place_and_decide(
                lambda nm, s_: ((isl_avoid is not None and isl_avoid(nm, s_))
                                or any(abs(s_ - s_b) < R_b - 1e-6 for s_b, R_b, _k in room_block.get(nm, ()))),
                pre=pre)
            for mv, bl in room_block.items():
                if abs(self.exit_leg_s[mv] - was[mv]) < 1e-9:
                    stuck.update((mv, k_) for _s, _R, k_ in bl)
            self.log('  legs off diving corners: ' + ', '.join(
                f'{mv} s{was[mv]:.2f}->{self.exit_leg_s[mv]:.2f}' for mv in sorted(room_block)))
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
                pts += self._round_teeth_ahead(nm, s_t, o_t, self.s_of_u(0.0), self.launch_o[nm])
            # RIBBON: hold the launch offset at the region start,
            # then run STRAIGHT to the target slot at s1
            pts.append((self.s_of_u(0.0), self.launch_o[nm]))
            pts.append((self.s1, py[trank[nm]]))
            if nm in self.exit_block and getattr(self, 'branch', False):
                # a BRANCH tightens as it goes: when a lane inside this one
                # peels off, this one moves in by that lane's gap on a
                # 45-degree ramp from its leg, so the ring hugs the array
                # instead of keeping every slot to the end (HHa's south ring
                # ran 11 mm out up the east face)
                s_l = self.exit_leg_s[nm]
                o_now = self._branch_o(nm)
                for s_k in sorted(self._branch_breaks(nm)):
                    if self.s1 < s_k < s_l:
                        pts.append((s_k, o_now(s_k)))
                pts.append((s_l, o_now(s_l)))
                legs.append((s_l, o_now(s_l), o_e))
                if abs(s_l - s_e) > 1e-9:
                    # a leg moved off its berth (a leg or pair already there)
                    # comes back along the berth's row, as a trunk exit does
                    jogs.append(((s_l, o_e), (s_e, o_e)))
            elif nm in self.exit_block:
                s_l = self.exit_leg_s[nm]
                pts.append((min(s_l, s_e) if s_l < s_e else s_e, py[trank[nm]]))
                if s_l > s_e + 1e-9:
                    pts.append((s_l, py[trank[nm]]))
                legs.append((s_l, py[trank[nm]], o_e))
                if abs(s_l - s_e) > 1e-9:
                    jogs.append(((s_l, o_e), (s_e, o_e)))
            else:
                if nm in getattr(self, 'row_slot', {}):
                    # a BERTH ROW's lane holds its slot outside the row and turns
                    # in along its own stub's escape line, as the human's do: laid
                    # straight from slot to berth, nested lanes cut across one
                    # another and over the nearer berths' ends
                    s_t = self._turn_in(nm, py[trank[nm]])
                    if s_t is None and self.row_up.get(nm):
                        # a stub along the spine has no escape line to turn in
                        # on: hold the slot to a pitch past the row's nearer
                        # berths, then run in (HHa SA10's straight tail grazed
                        # SDQ10's berth end 0.06 mm off)
                        s_h_ = max(self.se[a][0] for a in self.row_up[nm]) + LPITCH
                        if self.s1 + 0.05 < s_h_ < s_e - 0.02:
                            s_t = s_h_
                    if s_t is not None:
                        pts.append((s_t, py[trank[nm]]))
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
        if getattr(self, 'branch', False):
            self._order_ring()
        # o held constant THROUGH every spine corner a lane piece spans:
        # lane_xy draws a piece that changes o across a corner as a chord to
        # the mitre, |o| tan(turn/2) further on than the plan's s, so the board
        # line was shallower than every (s, o) check assumed (HHa SA13 over
        # SRST: 0.30 planned, 0.188 drawn; audit B)
        cs_ = [float(sp.S[j]) for j in range(1, sp.n) if abs(sp.turn[j - 1]) >= 1.0]
        if cs_:
            for nm in M:
                ms_ = self.mid[nm]
                if len(ms_) < 2:
                    continue
                add = []
                for S_j in cs_:
                    if ms_[0][0] + 0.01 < S_j < ms_[-1][0] - 0.01:
                        o_j = float(np.interp(S_j, [q[0] for q in ms_], [q[1] for q in ms_]))
                        add += [(S_j - 0.002, o_j), (S_j + 0.002, o_j)]
                if add:
                    self.mid[nm] = sorted([q for q in ms_ if not any(abs(q[0] - a_[0]) < 0.002 for a_ in add)] + add,
                                          key=lambda q: q[0])
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
        self.swim_changes = {}
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

        # THE DIVES ARE PLACED TOGETHER: every diamond a via pitch from every
        # via site already planned -- the exit corners and leg splits, and
        # the diamonds placed before it -- and clear of every other net's
        # static copper INCLUDING this corridor's own teeth and berths
        # (obs_but leaves the members' copper out, so a diamond could sit on
        # another lane's stub). Placed one swimmer at a time, HHa's SDQ9 and
        # SA1 reserved spots 0.071 mm apart and SDQ14/SDQS0 0.235.
        placed_v = []
        for om in M:
            if om in self.exit_leg_s and om in self.legs and self.legs[om] and self._corner_dive(om):
                placed_v.append((tuple(float(v) for v in sp.xy(self.exit_leg_s[om], self.legs[om][-1][1])), om))
            if om in self.leg_split and om in self.exit_leg_s:
                placed_v.append((tuple(float(v) for v in sp.xy(self.exit_leg_s[om], self.leg_split[om])), om))
        prs_ = getattr(self.ctx, 'pairs', None) or {}
        for nm in M:
            if sched.page.get(nm) is not None:
                continue
            hw_ = (_pairs.pitch(TRACK) / 2) if nm in prs_ else 0.0
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
                    want.append((float(S[i]), self._need_at(om, float(S[i]), sched)))
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
                        # (...and a PAIR's line reaches lane_w further, on
                        # both sides: its legs, or its two barrels here)
                        if any(om != nm and line_dist(om, ds, o0 + do) < 0.30 + self.lane_w(om) + self.lane_w(nm)
                               for om in M):
                            continue
                        xy_ = (float(xy[0]), float(xy[1]))
                        if any(o_ != nm and math.hypot(xy_[0] - q[0], xy_[1] - q[1])
                               < VIA_SIZE + CLEAR + 2 * hw_ + ((_pairs.pitch(TRACK) / 2) if o_ in prs_ else 0.0)
                               for (q, o_) in placed_v):
                            continue
                        if not self._via_static_ok(nm, xy_, hw_):
                            continue
                        got = xy
                        break
                    if got:
                        break
                if got:
                    spots.append(got)
                    placed_v.append(((float(got[0]), float(got[1])), nm))
            if spots:
                self.hops[nm] = spots
            if want:
                n_ch = sum(1 for a, b in zip(seq, seq[1:]) if a[1] != b[1])
                # the plan's price for this swimmer (plan_braid 'changes'):
                # the changes its planned line implies, tooth to berth
                self.swim_changes[nm] = n_ch
                self.log(f'  swimmer {nm}: {len(want)} page crossing(s), '
                         f'{n_ch} change(s), '
                         f'{len(spots)} diamond(s) reserved')

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
        seq.append((self.se[nm][0], (getattr(self, '_plan_dest', None) or ctx.dest_layer)[nm]))
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

    def _dive_extra(self, nm):
        """How much longer than a single's a PAIR member's dive zones are:
        a pair's layer change is two barrels side by side (its via
        envelope, 2 * via_half + via wide), placed by the pose router
        where the band opens both layers -- at a single's dive gap it fits
        nowhere (zynq K44: both pairs refused at the launch, boxed
        0.6 mm past the tips where the band closed the tooth's layer).
        BRAID_PAIR_DIVE_EXTRA, mm each side of a change."""
        if nm not in (getattr(self.ctx, 'pairs', None) or {}):
            return 0.0
        return PAIR_DIVE_EXTRA

    def allowed_vec(self, nm, S, L):
        ok = np.ones(S.shape, dtype=bool)
        x = self._dive_extra(nm)
        for (xa, xb, RL) in self.req.get(nm, ()):
            if RL != L:
                ok &= ~((S >= xa + x) & (S <= xb - x))
        if L == 'B.Cu':
            inb = np.zeros(S.shape, dtype=bool)
            for (lo, hi) in self.bwin[nm]:
                inb |= (S >= lo - x) & (S <= hi + x)
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
                    # (BRAID_BRANCH only: on the default chain this and the
                    # corner rule together cost K51 its last net -- SDQ11's
                    # last-call route walled -- where the branch frame needs it)
                    m = pres & (self.planned_vec(om, sg, L) if BRANCH else self.allowed_vec(om, sg, L))
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
        """Centrelines of lanes not routed yet as copper the router must
        clear (_virtual_of_plain) -- and a PAIR member's centreline as
        its two legs, one line either side at half the pair pitch, so a
        single routed before the pair leaves it room for both."""
        prs = getattr(self.ctx, 'pairs', {})
        if not prs:
            return self._virtual_of_plain(unrouted)
        segs = self._virtual_of_plain([om for om in unrouted if om not in prs])
        half = _pairs.pitch(TRACK) / 2
        for om in unrouted:
            if om in prs:
                for (p_, q_, L) in self._virtual_of_plain([om]):
                    segs.extend((a_, b_, L) for a_, b_ in _pairs.offset_line(p_, q_, half))
        return segs

    def _virtual_of_plain(self, unrouted):
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
                            if tail and L != (getattr(self, '_plan_dest', None) or self.ctx.dest_layer)[om]:
                                continue
                            if head and L != self.ctx.tooth_layer[om]:
                                continue
                            if self.allowed(om, s_mid, L) and (swim_om or L in self._planned_layers(om, s_mid)):
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
                if om in self.join_block and i == 0:
                    # ...and a JOIN leg leaves its tooth on the tooth's
                    # layer, as the head piece does: two nets' teeth sit at
                    # one point on the two layers (the fanout gives the gap
                    # to one net per layer) and their legs run off side by
                    # side -- stamped on both layers, HHa SA7's B leg walled
                    # SA9's F tooth 0.127 mm away along its whole F leg
                    segs.append((a_, b_, self.ctx.tooth_layer[om]))
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

    def planned_vec(self, nm, S, L):
        """allowed_vec narrowed to where the PLAN puts a page lane on L
        (_planned_layers, vectorised): a neighbour pinches a band only on
        the layer it will be on. Before s0 every lane is allowed both, so a
        B lane crossing the F fan-in read as an F neighbour and closed the
        F lanes' bands at every crossing. A swimmer, whose layer is not a
        plan, stays as allowed."""
        ok = self.allowed_vec(nm, S, L)
        sc = getattr(self, 'sched_cur', None)
        if sc is None or sc.page.get(nm) is None:
            return ok
        prof = self.layer_profile(nm)
        on = np.zeros(S.shape, dtype=bool)
        for k_, (s_, L_) in enumerate(prof):
            if L_ != L:
                continue
            hi = prof[k_ + 1][0] if k_ + 1 < len(prof) else np.inf
            lo = s_ if k_ else -np.inf
            on |= (S >= lo) & (S < hi)
        both = self.allowed_vec(nm, S, 'F.Cu') & self.allowed_vec(nm, S, 'B.Cu')
        if len(prof) > 1 and both.any():
            # the allowed-both runs that hold a planned change: the via's room
            flat = both.ravel(); Sf = S.ravel()
            order = np.argsort(Sf, kind='stable')
            ss, bb = Sf[order], flat[order]
            keep = np.zeros(len(ss), dtype=bool)
            i = 0
            n = len(ss)
            while i < n:
                if not bb[i]:
                    i += 1; continue
                j = i
                while j + 1 < n and bb[j + 1]:
                    j += 1
                lo_, hi_ = ss[i], ss[j]
                if any(lo_ - 0.03 <= c_ <= hi_ + 0.03 for (c_, _L) in prof[1:]):
                    keep[i:j + 1] = True
                i = j + 1
            kk = np.zeros(len(ss), dtype=bool); kk[order] = keep
            on |= kk.reshape(S.shape)
        return ok & on

    def _planned_layers(self, nm, s, reach=3.0, step=0.05):
        """The layers the PLAN puts a page lane on at s: its profile's layer
        there, and the other one too only inside the run allowed on both
        layers that holds one of its planned changes (where its via may go).
        Reserved on every ALLOWED layer instead, HHa's SA2 -- a B lane from a
        B tooth to a B berth -- was stamped on F through the fan-in, 0.07 mm
        from SDQ6's F tooth, and closed SDQ6's channel."""
        prof = self.layer_profile(nm)
        L = prof[0][1]
        for s_, L_ in prof:
            if s_ <= s + 1e-9:
                L = L_
        other = 'B.Cu' if L == 'F.Cu' else 'F.Cu'
        if not self.allowed(nm, s, other):
            return (L,)
        changes = [s_ for (s_, _L) in prof[1:]]
        for c_ in changes:
            if abs(c_ - s) > reach:
                continue
            n = max(1, int(abs(c_ - s) / step))
            if all(self.allowed(nm, s + (c_ - s) * k / n, 'F.Cu')
                   and self.allowed(nm, s + (c_ - s) * k / n, 'B.Cu') for k in range(n + 1)):
                return (L, other)
        return (L,)

    def _via_static_ok(self, nm, xy, extra=0.0):
        """A planned via at xy clears the static copper that is not `nm`'s:
        every pad of another net, every base segment and via of another net
        (teeth and berth stubs), by a via's radius plus the clearance."""
        ctx = self.ctx
        own = {ctx.byname[nm][0]}
        for leg in (getattr(ctx, 'pairs', None) or {}).get(nm, ()):
            if leg in ctx.byname:
                own.add(ctx.byname[leg][0])
        need = VIA_SIZE / 2 + CLEAR + extra
        x, y = xy
        for fp in ctx.pcb.footprints.values():
            for pd in fp.pads:
                if pd.net_id in own or pd.pad_type == 'np_thru_hole':
                    continue
                if abs(pd.global_x - x) > 2 or abs(pd.global_y - y) > 2:
                    continue
                if pd.shape == 'circle':
                    d = math.hypot(x - pd.global_x, y - pd.global_y) - pd.size_x / 2
                else:
                    dx = abs(x - pd.global_x) - pd.size_x / 2
                    dy = abs(y - pd.global_y) - pd.size_y / 2
                    d = math.hypot(max(dx, 0), max(dy, 0)) + min(max(dx, dy), 0)
                if d < need:
                    return False
        for sg_ in ctx.base_segments:
            if sg_.net_id in own or abs(sg_.start_x - x) > 3 or abs(sg_.start_y - y) > 3:
                continue
            ax, ay, bx, by = sg_.start_x, sg_.start_y, sg_.end_x, sg_.end_y
            dx, dy = bx - ax, by - ay
            l2 = dx * dx + dy * dy
            t = 0.0 if l2 < 1e-12 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / l2))
            if math.hypot(x - ax - t * dx, y - ay - t * dy) - sg_.width / 2 < need:
                return False
        for v in ctx.base_vias:
            if v.net_id in own:
                continue
            if math.hypot(x - v.x, y - v.y) - v.size / 2 < need:
                return False
        return True

    def _corner_dive(self, nm):
        """Does the plan change `nm`'s layer where it turns onto its exit
        leg: the layer it arrives on (its profile before the leg) is not
        the leg's layer. (BRAID_BRANCH only, with planned_vec in band_of:
        off the branch frame every corner is reserved, as before.)"""
        s_l = self.exit_leg_s.get(nm)
        Lg = self.leg_layer.get(nm)
        if not BRANCH or s_l is None or Lg is None:
            return True
        prof = self.layer_profile(nm)
        before = [L for (s_, L) in prof if s_ < s_l - 1e-6]
        return (before[-1] if before else prof[0][1]) != Lg

    def virtual_vias_of(self, unrouted):
        """The via each unrouted side exiter will need at its corner --
        where its lane turns onto its exit leg and changes to the leg's
        layer -- as a via the router must clear. The band alone does
        not protect the site: a neighbour's band edge sits exactly a
        via's clearance from this lane's centreline, so a neighbour
        hugging its edge there (K19 SCAS: SA7 0.23 mm off, SWE 0.26)
        leaves no legal via cell when the corner's owner is routed."""
        sp = self.spine
        tv = getattr(self, 'tail_vias', None)
        if tv is not None:
            # level 5: exactly the sites the schedule puts a change at
            # beyond the region -- a corner, a leg split, a jog change
            out = [p for om in unrouted for p in tv.get(om, ())]
        else:
            # a corner is a via only where the plan changes layer there:
            # 17 of HHa's 31 branch corners reserved a via for a lane that
            # stays on one layer round its corner -- a phantom barrel on
            # both layers beside the berth comb (SDQ3's approach walled)
            out = [sp.xy(self.exit_leg_s[om], self.legs[om][-1][1])
                   for om in unrouted if om in self.exit_leg_s and self._corner_dive(om)]
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
        leg's cell at every setback). BRAID_PAIR_SLACK adds to it."""
        return (_pairs.pitch(TRACK) / 2 + TRACK / 2 + 2 * self.ctx.cfg.grid_step
                + float(os.environ.get('BRAID_PAIR_SLACK', '0') or 0))

    def _pair_conn_points(self, nm):
        """Where a pair member's ROUTED CONNECTORS end (connect_pair): a
        point on its planned lane past the launch fan-in -- past the join
        leg's jog when it has one, and never less than a millimetre from
        the tooth -- and the mirror point before the exit leg. The lane
        polyline runs tooth -> stub (lane_xy), so both are found by arc
        length from their own end, whichever way the spine's s runs. None
        at both ends when the two would stand within 1.5 mm."""
        xy = list((getattr(self, 'lane_xy', {}) or {}).get(nm) or [])
        if len(xy) < 3:
            return None, None
        seg_len = [math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(xy, xy[1:])]
        cum = [0.0]
        for L in seg_len:
            cum.append(cum[-1] + L)
        total = cum[-1]

        def walk(target, back=False):
            # the point `target` mm along the lane and the lane's direction
            # there (reversed for the stub end: the way the pair travels
            # when it LEAVES the berth)
            for k, L in enumerate(seg_len):
                if cum[k] + L >= target and L > 1e-9:
                    u = (target - cum[k]) / L
                    p, q = xy[k], xy[k + 1]
                    dx, dy = (q[0] - p[0]) / L, (q[1] - p[1]) / L
                    if back:
                        dx, dy = -dx, -dy
                    return ((p[0] + (q[0] - p[0]) * u, p[1] + (q[1] - p[1]) * u), (dx, dy))
            return None
        # the run LEAVING each end (the next one when it is shorter than
        # 0.6 mm): the connector turns the pair into that run's direction
        # and ends about a millimetre along it
        def on_run(k):
            return cum[k] + max(0.05, min(max(1.0, 0.5 * seg_len[k]), seg_len[k] - 0.1))
        # candidates: the run leaving the end first, then the next run --
        # a berth whose lane starts with a short jog is best served by a
        # connector turned into the jog (the tips lie along it), and one
        # whose lane starts along the tips' own direction by the next
        ka = 0
        kb = len(seg_len) - 1
        ta = on_run(ka)
        tb = cum[kb] + seg_len[kb] - (on_run(kb) - cum[kb])
        cand_a = [walk(ta)]
        cand_b = [walk(tb, back=True)]
        if len(seg_len) >= 2:
            cand_a.append(walk(on_run(1)))
            kb2 = len(seg_len) - 2
            cand_b.append(walk(cum[kb2] + seg_len[kb2] - (on_run(kb2) - cum[kb2]), back=True))
        if os.environ.get('BRAID_PAIR_DEBUG'):
            (tp_, tn_), (sp_, sn_) = self.ctx.pair_ends[nm]
            self.log(f'    pair {nm} lane: {len(xy)} pts, {total:.2f} mm; connectors end at {ta:.2f} and '
                     f'{tb:.2f} mm along (runs {ka} and {kb}); tooth {self.teeth[nm]} tips {tp_}/{tn_}; '
                     f'stub {self.stubs[nm]} tips {sp_}/{sn_}; lane end {xy[-2]} -> {xy[-1]}')
            # the reserved vias of other members within 1.2 mm of either end, with their owners
            tv = getattr(self, 'tail_vias', None) or {}
            near = []
            for om, pts in list(tv.items()) + list((getattr(self, 'hops', {}) or {}).items()):
                if om == nm:
                    continue
                for (vx, vy) in pts:
                    for lbl, e in (('tooth', xy[0]), ('stub', xy[-1])):
                        dd = math.hypot(vx - e[0], vy - e[1])
                        if dd < 1.2:
                            near.append(f'{om} ({vx:.2f},{vy:.2f}) {dd:.2f} from the {lbl}')
            if near:
                self.log(f'    pair {nm}: reserved vias near its ends: ' + '; '.join(near))
            # ...and the exit-corner sites virtual_vias_of falls back to below level 5
            if tv is None or not tv:
                for om in self.members:
                    if om == nm or om not in getattr(self, 'exit_leg_s', {}):
                        continue
                    vx, vy = self.spine.xy(self.exit_leg_s[om], self.legs[om][-1][1])
                    for lbl, e in (('tooth', xy[0]), ('stub', xy[-1])):
                        dd = math.hypot(vx - e[0], vy - e[1])
                        if dd < 1.2:
                            self.log(f'    pair {nm}: exit-corner via of {om} at ({vx:.2f},{vy:.2f}) {dd:.2f} from the {lbl}')
            self.log(f'    pair {nm}: tail_vias {"absent" if tv is None else len(tv)} members, hops {len(getattr(self, "hops", {}) or {})}')
            # ...and any reserved via ON the pair's lane (within 0.45 mm of its polyline)
            from kicad_parser import Segment as _S
            on_lane = []
            for om, pts in list(tv.items()) + list((getattr(self, 'hops', {}) or {}).items()):
                if om == nm:
                    continue
                for (vx, vy) in pts:
                    dmin = min(_pairs._seg_seg_dist(_S(vx, vy, vx, vy, 0, 'F.Cu', 0), _S(a[0], a[1], b[0], b[1], 0, 'F.Cu', 0))
                               for a, b in zip(xy, xy[1:]))
                    if dmin < 0.45:
                        on_lane.append(f'{om} ({vx:.2f},{vy:.2f}) {dmin:.2f} off the lane')
            if on_lane:
                self.log(f'    pair {nm}: reserved vias ON its lane: ' + '; '.join(on_lane))
        if tb - ta < 1.5:
            return None, None
        return [c for c in cand_a if c], [c for c in cand_b if c]

    def route_pair_lane(self, nm, virt, virt_vias=None, slack=None, free=None):
        """A PAIR member's lane (pairs.py, BRAID_PAIRS): the production pair
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
        # the band widened by half the pair pitch a side (its legs), plus
        # BRAID_PAIR_SLACK (an experiment knob: the band's leg strips are
        # sized for one lane)
        half = self._pair_band_slack()
        # BRAID_PAIR_FREE (default on): a pair is searched like a free
        # swimmer -- no band, a 2 mm window margin -- with every unrouted
        # neighbour's reservation and every routed lane's copper still in
        # force. Measured K36 (2026-09-20): in its single-lane band the
        # pose router refused all three pairs -- the band's leg strips,
        # dive zones and slope pitches are sized for one track, and the
        # pair's centreline (half a pitch of extra clearance) found a
        # neck in each; free, the same search landed them at last call.
        free_pair = (os.environ.get('BRAID_PAIR_FREE', '1') != '0') if free is None else bool(free)
        if slack is not None:
            half = slack
        wpts = list(self.lane_xy[nm])
        if free_pair:
            band = None
            margin = max(margin, 2.0)
        else:
            band = None if swim else self.band_of(nm, slack=half)
            margin = max(margin, half + 0.6)
            # (a WHOLE-ROUTE plan draws the pair's ends as the pair step lays them -- its approach and first setback
            # along the stub's own way: no widening there, and no lane-guided connectors below)
            if band is not None and PAIR_FANIN_BAND > 0 and getattr(self, '_geo', None) is None:
                band, corners = self._pair_fanin_band(nm, band)
                wpts += corners
        # the CROSS-CORRIDOR reservation (other corridors' planned lanes,
        # their ends 1.5 mm on the end layers) under the pair's fan-in rule
        # too: a piece with an end within PAIR_FANIN of either of the pair's
        # ends is left out. Measured K51 SCK (2026-09-21): SA10, a one-net
        # corridor, berths 1.2 mm east of SCK's on the DDR's comb; its end
        # stamp on F.Cu ran through SCK's launch zone, the pair refused in
        # its band, widened and free with the same 402-cell pocket, landed
        # only free of the plan, and five singles stayed open behind it
        pe_ = getattr(ctx, 'pair_ends', {}) or {}
        if nm in pe_ and PAIR_FANIN > 0 and PAIR_CROSS_FANIN:
            (tp_, tn_), (sp_, sn_) = pe_[nm]
            ends_ = [e for e in (tp_, tn_, sp_, sn_) if e is not None]

            def far_(pc):
                (p, q, _L) = pc
                return all(min(math.hypot(p[0] - e[0], p[1] - e[1]), math.hypot(q[0] - e[0], q[1] - e[1])) >= PAIR_FANIN
                           for e in ends_)
            cross_ = [pc for pc in reserve(ctx, nm) if far_(pc)]
        else:
            cross_ = reserve(ctx, nm)
        virt = list(virt or []) + cross_
        rep = {}
        _ca, _cb = self._pair_conn_points(nm) if getattr(self, '_geo', None) is None else (None, None)
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
        if os.environ.get('BRAID_PAIR_DEBUG') and virt_vias:
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
                              report=rep, a_conn=_ca, b_conn=_cb, a_given=_ga, b_given=_gb, x_given=_gx)
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
            if os.environ.get('BRAID_PAIR_DEBUG'):
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

    def _pair_fanin_band(self, nm, band):
        """THE CONVERGENCE ZONE (2026-09-20): the pair's band ORed with a
        box at each end -- from the two ends' midpoint PAIR_FANIN mm out
        along the escape (arrival) direction, half the ends' separation
        plus PAIR_FANIN_BAND across, on that end's layers -- so the legs
        can run past the foreign exit stubs between them and converge.
        The band is one lane's wedge from the pair's centre: at zynq
        K47 DQS0's P tooth, 0.78 mm off the centre, it held ONE cell, and
        the converged tips (2 mm out) lay outside it altogether. Returns
        the band and the boxes' corners (window points)."""
        ctx = self.ctx
        (tp_, tn_), (sp_, sn_) = ctx.pair_ends[nm]
        boxes = []
        corners = []
        for p, n, d, Ls in ((tp_, tn_, ctx.tooth_dir.get(nm), {ctx.tooth_layer[nm], ctx.pair_layers[nm][0]}),
                            (sp_, sn_, ctx.stub_dir.get(nm), {ctx.dest_layer[nm], ctx.pair_layers[nm][1]})):
            if d is None or p is None or n is None:
                continue
            dd = math.hypot(d[0], d[1]) or 1.0
            ux, uy = d[0] / dd, d[1] / dd
            mx, my = (p[0] + n[0]) / 2, (p[1] + n[1]) / 2
            sep = abs(-(p[0] - n[0]) * uy + (p[1] - n[1]) * ux)
            w = sep / 2 + PAIR_FANIN_BAND
            R = PAIR_FANIN
            boxes.append((mx, my, ux, uy, w, R, {L for L in Ls if L}))
            for a_ in (-0.3, R):
                for c_ in (-w, w):
                    corners.append((mx + ux * a_ - uy * c_, my + uy * a_ + ux * c_))

        def band2(xs, ys, L):
            m = np.asarray(band(xs, ys, L), dtype=bool)
            X, Y = np.meshgrid(np.asarray(xs, dtype=float), np.asarray(ys, dtype=float), indexing='ij')
            for (mx, my, ux, uy, w, R, Ls) in boxes:
                if L not in Ls:
                    continue
                along = (X - mx) * ux + (Y - my) * uy
                across = -(X - mx) * uy + (Y - my) * ux
                m = m | ((along >= -0.3) & (along <= R) & (np.abs(across) <= w))
            return m
        return band2, corners

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
        img.save(out)
        self.log(f'    pair debug image -> {out}')


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
        # PAIRS FIRST (BRAID_PAIRS_FIRST, default on with the pair member): a
        # pair lane needs the room of two and a coupled dive; routed after
        # the singles it found none and landed only at last call (K34-K36:
        # every pair). Boosted above the ribbon's own refused-lane boost.
        if getattr(ctx, 'pairs', None) and int(os.environ.get('BRAID_PAIRS_FIRST', '1') or 0):
            for nm in M:
                if nm in ctx.pairs:
                    boost[nm] = boost.get(nm, 0) + 100
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
        for attempt in range(ATTEMPTS):
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
                            key=lambda nm: (-boost.get(nm, 0), 0, ti[nm]))
                     + sorted(sw_, key=lambda nm: (-boost.get(nm, 0), 0, -abs(
                         self.launch_o[nm] - self.target_o[nm]))))
            routed = {nm for nm in M if nm in (getattr(ctx, 'protected', None) or ())}
            self.refused = []
            failed_rescues = 0
            for nm in order:
                if nm in routed:
                    continue          # a pair routed first: its copper is base copper
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
                    big.max_iterations = BUDGET_X * max(cfg0.max_iterations, 50_000)
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
            big.max_iterations = BUDGET_X * max(cfg0.max_iterations, 50_000)
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
                        nm, self.virtual_of(others) + self.approach_virt(nm),
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
        big2.max_iterations = BUDGET_X * max(cfg0.max_iterations, 50_000)
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
            # ...and EXTRA-LONG lanes (2026-09-11): a lane more than
            # ECON_LONG mm over its airline is a candidate whatever its
            # via count, and the pass says so -- K41's SA1 wrapped a
            # passive block for 49.7 mm on 0 vias (the address group's
            # median 27.5) and the grade, which has no length term,
            # never saw it; the candidate rule was there but silent
            heavy = sorted(
                (nm for nm in self.members
                 if len(self.out_vias.get(nm) or ()) >= econ_min
                 or (self.out_segs.get(nm)
                     and lane_mm(nm) - airline(nm) > ECON_LONG)),
                key=lambda nm: -len(self.out_vias.get(nm) or ()))
            longs = [nm for nm in heavy
                     if self.out_segs.get(nm) and lane_mm(nm) - airline(nm) > ECON_LONG]
            if longs:
                log('    econ re-lay: extra-long lanes '
                    + ' '.join(f'{nm} {lane_mm(nm):.1f}mm/+{lane_mm(nm) - airline(nm):.1f}'
                               for nm in longs))
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
                too_long = None
                for mg, wp in ((2.5, self.lane_xy[nm]),
                               (4.0, self.lane_xy[nm]),
                               (6.0, None)):
                    r_ = cn.connect(ctx.pcb, nid, self.teeth[nm],
                                    ctx.tooth_layer[nm],
                                    self.stubs[nm],
                                    ctx.dest_layer[nm], ctx.cfg,
                                    band=None, margin=mg,
                                    window_pts=wp,
                                    virtual=None,
                                    b_alts=ctx.dest_alts.get(nm))
                    # keep FEWER vias -- within ECON_MM_PER_VIA of copper a
                    # via -- or equal vias and clearly shorter copper (the
                    # overshoot harvest)
                    nv0 = len(self.out_vias[nm])
                    if r_ is not None:
                        nv1 = len(r_[1])
                        mm1 = sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
                                  for s in r_[0])
                        if nv1 < nv0 and (ECON_MM_PER_VIA <= 0
                                          or mm1 <= lane_mm(nm) + ECON_MM_PER_VIA * (nv0 - nv1)):
                            res = r_
                            break
                        if nv1 == nv0 and mm1 < lane_mm(nm) - 0.5:
                            res = r_
                            break
                        if nv1 < nv0:
                            too_long = (nv1, mm1)
                if ECON_JOINT and (res is None or len(res[1]) >= 1) \
                        and (ECON_JOINT >= 2 or res is None and (too_long is not None or nm in longs)):
                    # THE JOINT RE-LAY: the lane's short path crosses a
                    # neighbour laid before it (K36: SA0, routed at last
                    # call, hugged the DDR's comb 0.2 mm in front of SBA1's
                    # berth); the min-cut probe names it, the trial rips it,
                    # lays this lane, re-lays the victim, and the set is
                    # kept only when cheaper in all -- vias first under
                    # the same per-via guard, else shorter at equal vias
                    def _mm(ss):
                        return sum(math.hypot(s_.end_x - s_.start_x, s_.end_y - s_.start_y) for s_ in ss)

                    # the baseline the set must beat: the rung's lane when
                    # one was found (SBA1: a 2-via lane 0.7 mm shorter was
                    # accepted before the joint re-lay ever ran), else the
                    # lane as it stands
                    bv = len(res[1]) if res is not None else nv0
                    bm = _mm(res[0]) if res is not None else lane_mm(nm)

                    def econ_ok(r1, relaid):
                        # every lane the trial changed: its direct victims
                        # (not yet committed at this point) and any lane a
                        # nested negotiation already committed (measured:
                        # counted by the victims alone, a "2 -> 2" re-lay of
                        # SBA1 shipped +6 vias on three lanes it never named)
                        rows = [(nm, bv, bm, len(r1[1]), _mm(r1[0]))]
                        for v in set(relaid) | {v for v in self.out_segs
                                                if v != nm and self.out_segs.get(v) is not os_b.get(v)}:
                            new = relaid.get(v) or (self.out_segs[v], self.out_vias[v])
                            rows.append((v, len(ov_b.get(v) or ()), _mm(os_b.get(v) or ()), len(new[1]), _mm(new[0])))
                        for _v, a_v, a_m, b_v, b_m in rows:
                            # no lane ends with MORE vias than it had (a via
                            # moved onto a neighbour is a displacement, not
                            # an economy -- measured K36: SDQ12 2 -> 4 for
                            # SDQ13's 5 -> 3 reshaped the board and cost SA7
                            # and SA8 their 0-via re-lays after it), none
                            # buys its own with more than the guard, and one
                            # that saves none may not grow
                            if b_v > a_v or (ECON_MM_PER_VIA > 0
                                             and b_m > a_m + ECON_MM_PER_VIA * max(0, a_v - b_v) + 1.0):
                                log(f'    econ joint {nm}: refused -- {_v} {a_v} -> {b_v} via(s), '
                                    f'{a_m:.1f} -> {b_m:.1f} mm')
                                return False
                        v0 = sum(r[1] for r in rows)
                        m0 = sum(r[2] for r in rows)
                        v1 = sum(r[3] for r in rows)
                        m1 = sum(r[4] for r in rows)
                        # a neighbour is ripped for VIAS, never for millimetres
                        # (K41 arm B: SODT0 re-laid 3 mm shorter by ripping
                        # SODT1 took the space SA2's 0-via re-lay needed after
                        # it, 46 -> 48); the single rungs harvest length
                        if v1 < v0:
                            return ECON_MM_PER_VIA <= 0 or m1 <= m0 + ECON_MM_PER_VIA * (v0 - v1)
                        log(f'    econ joint {nm}: refused -- {v0} -> {v1} via(s) in all, no via saved')
                        return False
                    seg_b, via_b = list(ctx.pcb.segments), list(ctx.pcb.vias)
                    os_b, ov_b = dict(self.out_segs), dict(self.out_vias)
                    lanes_here = [om for om in self.members
                                  if om != nm and self.out_segs.get(om)
                                  and om not in (getattr(ctx, 'protected', None) or ())]
                    r1 = self.rip_for(nm, list(self.refused), {}, accept=econ_ok,
                                      lanes_from=lanes_here, probe_margin=2.5, soft_cost=1.0,
                                      bound=(bv, bm), depth=0,
                                      protect=frozenset(getattr(ctx, 'protected', None) or ()))
                    if r1 is not None:
                        if lane_crosses_foreign(ctx.pcb, nid, r1[0]) or via_hits_foreign(ctx.pcb, nid, r1[1]):
                            ctx.pcb.segments, ctx.pcb.vias = seg_b, via_b
                            self.out_segs, self.out_vias = os_b, ov_b
                        else:
                            moved = [v for v in self.out_segs if self.out_segs[v] is not os_b.get(v)]
                            log(f'    econ joint re-lay: {nm} {nv0} -> {len(r1[1])} via(s), '
                                f'{lane_mm(nm):.1f} -> {_mm(r1[0]):.1f} mm; ripped '
                                + ', '.join(f'{v} {len(ov_b[v])} -> {len(self.out_vias[v])} via(s) '
                                            f'{_mm(os_b[v]):.1f} -> {_mm(self.out_segs[v]):.1f} mm'
                                            for v in moved))
                            res = r1
                if res is None:
                    ctx.pcb.segments = seg0
                    ctx.pcb.vias = via0
                    if too_long is not None:
                        log(f'    econ re-lay: {nm} REJECTED {nv0} -> {too_long[0]} via(s) at '
                            f'{lane_mm(nm):.1f} -> {too_long[1]:.1f} mm (over {ECON_MM_PER_VIA:.0f} mm a via)')
                    elif nm in longs:
                        log(f'    econ re-lay: {nm} kept at {lane_mm(nm):.1f} mm '
                            f'({len(self.out_vias[nm])} via(s)): no cheaper lane found')
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
                mm_new = sum(math.hypot(s_.end_x - s_.start_x, s_.end_y - s_.start_y)
                             for s_ in segs_o)
                log(f'    econ re-lay: {nm} '
                    f'{len(self.out_vias[nm])} -> {len(vias_o)} '
                    'via(s)'
                    + (f', {lane_mm(nm):.1f} -> {mm_new:.1f} mm' if nm in longs else ''))
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
        if LADDER_MODE == 'open':
            # BRAID_LADDER=open (2026-09-18, probes): start at the rung that
            # opens both layers. Measured over 436 probe braids, lanes landed
            # on the first two rungs in 16 and 16 and on the third in 355; a
            # rung costs 0.06-0.33 s whether it lands or not.
            rungs = rungs[2:]
        wp = (getattr(self, 'lane_xy', {}) or {}).get(nm) or [a, b]
        # the ladder's cost per rung, in the log (2026-09-18: of 436 probe
        # braids, lanes landed on the first two rungs in 16 and 16, on the
        # third in 355 -- the layer requirement, not the width, is what the
        # early rungs cannot relax)
        spent = []
        for label, mk, mg in rungs:
            _t = _time.perf_counter()
            if nm in getattr(ctx, 'pairs', {}):
                # a PAIR member climbs the same ladder with the pair router
                # (a single-ended search from the pair's midpoint is no
                # route for it): the rung's band, widened by half a pitch.
                # Copper is appended by the caller, as for a single.
                pn, nn = ctx.pairs[nm]
                (tp_, tn_), (sp_, sn_) = ctx.pair_ends[nm]
                res = cn.connect_pair(ctx.pcb, ctx.byname[pn][0], ctx.byname[nn][0],
                                      tp_, tn_, aL, sp_, sn_, bL, ctx.cfg, band=mk(),
                                      band_slack=_pairs.pitch(TRACK) / 2, margin=mg,
                                      virtual=list(virt or []) + reserve(ctx, nm),
                                      window_pts=wp, virtual_vias=virt_vias, report=report,
                                      gap=_pairs.GAP, a_dir=ctx.tooth_dir.get(nm),
                                      b_dir=ctx.stub_dir.get(nm),
                                      a_n_layer=ctx.pair_layers[nm][0], b_n_layer=ctx.pair_layers[nm][1],
                                      a_conn=self._pair_conn_points(nm)[0], b_conn=self._pair_conn_points(nm)[1])
            else:
                res = cn.connect(ctx.pcb, nid, a, aL, b, bL, ctx.cfg,
                                 band=mk(), margin=mg,
                                 virtual=list(virt or []) + reserve(ctx, nm),
                                 window_pts=wp, virtual_vias=virt_vias,
                                 b_alts=b_alts, report=report)
            spent.append(f'{label}/{mg:g} {_time.perf_counter() - _t:.2f}s {"ok" if res is not None else "no"}'
                         + (f' [split: {report["intra"]}]' if (res is None and report is not None and report.get('intra')) else '')
                         + (' [polarity]' if (res is None and report is not None and report.get('polarity')) else ''))
            if report is not None:
                report.pop('intra', None)
                report.pop('polarity', None)
            if res is not None:
                ctx.rungs[(stage, label)] += 1
                self.log(f'    ladder {nm} ({stage}): ' + ', '.join(spent))
                return res
        self.log(f'    ladder {nm} ({stage}): ' + ', '.join(spent) + ' -- refused')
        return None

    def _free_rungs(self, nm, virt, vv):
        """The econ re-lay's search for one lane: band-free, a window
        round its planned path at 2.5 then 4.0 mm, then 6.0 mm round the
        airline; the first route found, or None."""
        ctx = self.ctx
        nid, _ = ctx.byname[nm]
        wp = (getattr(self, 'lane_xy', {}) or {}).get(nm)
        for mg, w in ((2.5, wp), (4.0, wp), (6.0, None)):
            r_ = cn.connect(ctx.pcb, nid, self.teeth[nm], ctx.tooth_layer[nm],
                            self.stubs[nm], ctx.dest_layer[nm], ctx.cfg,
                            band=None, margin=mg, window_pts=w,
                            virtual=list(virt) or None, virtual_vias=vv or None,
                            b_alts=ctx.dest_alts.get(nm))
            if r_ is not None:
                return r_
        return None

    def approach_virt(self, nm):
        """THE COMB DISCIPLINE for open searches (last call, rip, econ):
        every other member's berth APPROACH -- APPROACH_RESERVE mm out
        from its berth along the arrival direction, on its arrival layer
        -- as virtual copper, so a lane searched free of its band cannot
        park in front of a neighbour's berth. Measured K36 (2026-09-20):
        SA0, refused in band and routed at last call, hugged the DDR's
        comb 0.2 mm in front of SBA1's berth; SBA1, landed on the pad by
        another route, was then re-laid round the outside of the array
        and back up under its balls (44.7 mm for 0 vias) because the way
        in from above was taken. A member's own approach is never in its
        list; a lane whose copper already fills its approach loses
        nothing to the duplicate."""
        if APPROACH_RESERVE <= 0:
            return []
        ctx = self.ctx
        out = []
        for om in self.members:
            if om == nm:
                continue
            e = self.stubs.get(om)
            d = ctx.stub_dir.get(om)
            L = ctx.dest_layer.get(om)
            if e is None or d is None or L is None:
                continue
            out.append(((e[0], e[1]), (e[0] + d[0] * APPROACH_RESERVE, e[1] + d[1] * APPROACH_RESERVE), L))
        return out

    def rip_for(self, nm, others, rep, max_victims=None, depth=None, protect=frozenset(),
                accept=None, lanes_from=None, probe_margin=6.0, soft_cost=None, bound=None):
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
        # the two dials (see RIP_VICTIMS / RIP_DEPTH); None = whatever the
        # environment says, which defaults to the 3 and 1 this signature
        # used to hard code
        if max_victims is None:
            max_victims = RIP_VICTIMS
        if depth is None:
            depth = RIP_DEPTH
        nid, _ = ctx.byname[nm]
        t0 = _time.perf_counter()
        blocked = rep.get('blocked') or []
        if lanes_from is not None:
            # the econ joint re-lay: no refusal, no frontier -- the lanes
            # of this run are all candidates and the min-cut probe alone
            # (a tight window, a cheap soft price) says which its short
            # path would cross
            named = [(om, 0) for om in lanes_from
                     if om != nm and om not in protect and self.out_segs.get(om)]
            if not named:
                return None
        else:
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
        virt0, vv = self.virtual_of(others), self.virtual_vias_of(others)
        # the comb discipline at the last call, not in econ mode (measured
        # K36: reserved in econ it cost SA7 and SA8 their 0-via re-lays,
        # 79 -> 83)
        appr = (lambda x: self.approach_virt(x)) if lanes_from is None else (lambda x: [])
        virt = virt0 + appr(nm)
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
                           band=None, margin=probe_margin,
                           virtual=list(virt) + reserve(ctx, nm), window_pts=wp,
                           virtual_vias=vv, b_alts=ctx.dest_alts.get(nm),
                           soft=soft, soft_vias=soft_v,
                           soft_cost=5.0 if soft_cost is None else soft_cost)
        ctx.pcb.segments, ctx.pcb.vias = seg0, via0
        if probe is None:
            log(f'    rip for {nm}: no path even with every lane priced -- '
                f'walled by static copper  ({_time.perf_counter() - t0:.1f} s)')
            return None
        if bound is not None:
            # the probe is the optimistic bound of any trial (every lane
            # priced, none blocking): no cheaper probe, no trial
            pv = len(probe[1])
            pm = sum(math.hypot(x.end_x - x.start_x, x.end_y - x.start_y) for x in probe[0])
            if not (pv < bound[0] or (pv == bound[0] and pm < bound[1] - 0.5)):
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
            if lanes_from is not None:
                # the econ probe crossing nothing = the lane alone, which
                # the econ rungs have already tried
                return None
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
        if lanes_from is None:
            for v, _c in named[:max_victims]:
                if [v] not in trials:
                    trials.append([v])
        for V in trials:
            seg0, via0 = list(ctx.pcb.segments), list(ctx.pcb.vias)
            # ...and the BOOKKEEPING, which the rollback below used not to
            # restore. A NESTED rip commits self.out_segs/out_vias for its
            # own victims before returning; when the outer trial then lost
            # a victim it put the BOARD back and left those entries naming
            # copper that is no longer on it. The writer takes segments
            # from ctx.pcb and vias from out_vias, so the net shipped with
            # old tracks and new via positions -- every layer change with
            # no barrel under it, reported as routed. Measured: 47 of 48
            # "silent" opens across 391 recorded runs break at a via-less
            # layer change, and SDQ5 on the 104-via K51 plan is one.
            os0, ov0 = dict(self.out_segs), dict(self.out_vias)
            ids_s = {id(x) for v in V for x in self.out_segs[v]}
            ids_v = {id(x) for v in V for x in self.out_vias[v]}
            ctx.pcb.segments = [x for x in seg0 if id(x) not in ids_s]
            ctx.pcb.vias = [x for x in via0 if id(x) not in ids_v]
            if lanes_from is not None:
                # econ mode: the lane searched FREE round its planned path,
                # as the econ rungs search it -- the band-first ladder
                # keeps a page lane on its page, which is the last call's
                # concern, not the economy's (K36 SBA1: the probe found 0
                # vias past SDQ7, the ladder's band rungs 2 again)
                r1 = self._free_rungs(nm, virt, vv)
            else:
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
                r2 = self.connect_ladder(v, virt0 + appr(v), vv, 'rip',
                                         b_alts=ctx.dest_alts.get(v), report=rep2)
                if r2 is None and lanes_from is not None:
                    r2 = self._free_rungs(v, virt0 + appr(v), vv)
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
                self.out_segs, self.out_vias = os0, ov0
                log(f'    rip {V}: {nm} routed ({len(r1[1])} via(s)) but '
                    f'{lost} lost -- put back  ({_time.perf_counter() - t0:.1f} s)')
                continue
            if accept is not None and not accept(r1, relaid):
                ctx.pcb.segments, ctx.pcb.vias = seg0, via0
                self.out_segs, self.out_vias = os0, ov0
                log(f'    rip {V}: {nm} routed ({len(r1[1])} via(s)), every victim re-laid, '
                    f'not cheaper in all -- put back  ({_time.perf_counter() - t0:.1f} s)')
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


def drc_line_nets(ln):
    """The net names a check_drc violation line names: either side of
    '<->' is 'Seg:NET', 'Via:NET', 'Pad:NET (REF.PIN)' or a bare 'NET'
    (a track-to-track pair), the net possibly '/'-pathed."""
    out = set()
    if '<->' not in ln or ln.lstrip().startswith('Checking'):   # the checker's own header carries a '<->'
        return out
    for side in ln.split('<->'):
        tok = side.strip().split()[0] if side.strip() else ''
        if ':' in tok:
            tok = tok.split(':', 1)[1]
        tok = tok.split('/')[-1].strip('(),')
        if tok:
            out.add(tok)
    return out


def _walk_stub(segs_n, start, lay, _stop, _k, max_hops=24):
    """The stub polyline from `start` on `lay` toward the pad, as
    (segment, near point, far point) triples; stops at a junction, a
    same-net via or a pad. Empty when `start` is itself anchored (see
    the dest chain's SODT1 note)."""
    if _stop(start):
        return []
    chain, cur, prev = [], start, None
    for _hop in range(max_hops):
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
    return chain


SRC_TRIM_REACH = float(os.environ.get('SRC_TRIM_REACH', '4.0') or 0)  # mm: the longest splice tried (K44 DQ13 came back three channels over, 2.5 mm)
SRC_TRIM_TRIES = 6                                                     # candidates graded per lane, best saving first
SRC_TRIM_MIN = 0.3                                                     # mm: the least stub depth worth a splice


def note_source_joint(ctx, nm, lane, vias, board_path, log):
    """AT WRITE TIME ONLY, the source-side mirror of note_joint: a lane
    that left its tooth and ran BACK along its own stub is spliced onto
    the stub where it departs -- the stub's tip-side tail and the lane's
    backtrack go, one short cross segment joins them -- and the splice
    ships only when the net's scoped DRC is no worse than before.

    Measured need (2026-09-19, the zynq article): a singleton corridor's
    tooth planned on the FAR face of the source array is a channel
    escape through seventeen rows of the BGA, and the lane launched from
    it runs straight back up the next channel: DQ12 at K38 carried 58 mm
    of copper for a 25 mm connection, DQ13 at K44 the same. Pure output
    economy: the routed world is untouched (the same reason note_joint
    is deferred). Returns the mm saved, 0.0 when nothing was done."""
    chain = ctx.src_chain.get(nm) or []
    if not chain or len(lane) < 2 or SRC_TRIM_REACH <= 0:
        return 0.0
    lay = chain[0][0].layer
    pcb = ctx.pcb
    nid = ctx.byname[nm][0]

    def k3(x, y):
        return (round(x, 3), round(y, 3))

    tip = k3(*ctx.ends[nm][0])
    adj = defaultdict(list)
    for s in lane:
        a_, b_ = k3(s.start_x, s.start_y), k3(s.end_x, s.end_y)
        if a_ != b_:
            adj[a_].append((b_, s))
            adj[b_].append((a_, s))
    if tip not in adj:
        return 0.0
    verts, path, used, cur = [tip], [], set(), tip
    while True:
        nxt = [(q, s) for (q, s) in adj[cur] if id(s) not in used]
        if len(nxt) != 1:
            break
        q, s = nxt[0]
        used.add(id(s))
        path.append(s)
        verts.append(q)
        cur = q
    if len(path) < 2:
        return 0.0
    # the stub chain, tip -> pad, with arc positions
    cpts = [chain[0][1]] + [p_ for (_s, _t, p_) in chain]
    arc = [0.0]
    for p_, q_ in zip(cpts, cpts[1:]):
        arc.append(arc[-1] + math.hypot(q_[0] - p_[0], q_[1] - p_[1]))
    if arc[-1] < SRC_TRIM_MIN:
        return 0.0

    def project(v):
        best = None
        for i, (p_, q_) in enumerate(zip(cpts, cpts[1:])):
            dx, dy = q_[0] - p_[0], q_[1] - p_[1]
            L2 = dx * dx + dy * dy
            if L2 < 1e-12:
                continue
            t = max(0.0, min(1.0, ((v[0] - p_[0]) * dx + (v[1] - p_[1]) * dy) / L2))
            px, py = p_[0] + t * dx, p_[1] + t * dy
            d = math.hypot(v[0] - px, v[1] - py)
            if best is None or d < best[0]:
                best = (d, arc[i] + t * math.sqrt(L2), (px, py), i, t)
        return best

    via_pts = [k3(v.x, v.y) for v in vias]
    cands = []
    run = 0.0
    for j in range(1, len(verts) - 1):
        seg = path[j - 1]
        if seg.layer != lay:
            break                      # the lane left the stub's layer: nothing past here rides it
        run += math.hypot(seg.end_x - seg.start_x, seg.end_y - seg.start_y)
        if verts[j] in via_pts and path[j].layer == lay:
            continue                   # a via the splice would strand
        if path[j].layer != lay and verts[j] not in via_pts:
            break
        pr = project(verts[j])
        if pr is None or pr[0] > SRC_TRIM_REACH or pr[1] < SRC_TRIM_MIN:
            continue
        saving = run + pr[1] - pr[0]
        if saving < 0.2:
            continue
        cands.append((saving, j, pr, run))
    if os.environ.get('SRC_TRIM_DEBUG') == '1':
        log(f'  source stub trim {nm}: lane {len(verts)} vertices from the tip, stub chain {arc[-1]:.1f} mm, '
            f'{len(cands)} candidate(s): ' + ', '.join(f'depth {c[2][1]:.1f}/splice {c[2][0]:.2f}/saves {c[0]:.1f}'
                                                       for c in sorted(cands, key=lambda c: -c[0])[:5]))
    if not cands:
        return 0.0
    # best saving first, but a splice near-identical to one already
    # refused (same length, same stub depth: the router's grid steps
    # give a dozen such vertices in a row) is not a new question --
    # K44 DQ13 spent every try on 4 mm splices through the ball field
    # while its 2.5 mm one waited
    cands.sort(key=lambda c: -c[0])
    tried = []
    for saving, j, pr, run in cands:
        if any(abs(pr[0] - d_) < 0.1 and abs(pr[1] - s_) < 0.5 for d_, s_ in tried):
            continue
        if len(tried) >= SRC_TRIM_TRIES:
            break
        tried.append((pr[0], pr[1]))
        got = _apply_source_splice(ctx, nm, lane, chain, verts, path, j, pr, run, saving, board_path, log)
        if got:
            return got
    return 0.0


def _apply_source_splice(ctx, nm, lane, chain, verts, path, j, pr, run, saving, board_path, log):
    """One candidate of note_source_joint, graded and applied. Returns
    the mm saved, 0.0 when the splice would add DRC."""
    pcb = ctx.pcb
    nid = ctx.byname[nm][0]
    lay = chain[0][0].layer
    d, s_at, proj, ci, t = pr
    # the chain cut: segments tip-side of the projection go; the one
    # holding the projection keeps its pad-side part
    gone = [c[0] for c in chain[:ci]]
    keep_part = None
    seg_i, near_i, far_i = chain[ci]
    if t >= 1.0 - 1e-6:
        gone.append(seg_i)
        proj = far_i
    elif t > 1e-6:
        gone.append(seg_i)
        fx, fy = (seg_i.end_x, seg_i.end_y) \
            if math.hypot(seg_i.end_x - far_i[0], seg_i.end_y - far_i[1]) \
            <= math.hypot(seg_i.start_x - far_i[0], seg_i.start_y - far_i[1]) \
            else (seg_i.start_x, seg_i.start_y)
        keep_part = Segment(proj[0], proj[1], fx, fy, seg_i.width, lay, nid)
    else:
        proj = near_i
    v = verts[j]
    splice = None
    if math.hypot(v[0] - proj[0], v[1] - proj[1]) > 0.001:
        # the stub's own width: it already runs this channel at it
        splice = Segment(proj[0], proj[1], v[0], v[1], min(path[j].width, seg_i.width), lay, nid)
    drop = {id(x) for x in gone} | {id(x) for x in path[:j]}
    before = pcb.segments
    cand = [x for x in before if id(x) not in drop]
    cand += [x for x in (keep_part, splice) if x is not None]
    try:
        import source_realize as _sr
        pcb.segments = before
        v0 = _sr.drc_pairs(board_path, nets=[nm], pcb_data=pcb)
        pcb.segments = cand
        v1 = _sr.drc_pairs(board_path, nets=[nm], pcb_data=pcb)
        n0, n1 = len(v0), len(v1)
    except Exception as e:                                  # noqa: BLE001
        pcb.segments = before
        log(f'  source stub trim {nm}: no verdict ({e}) -- kept as laid')
        return 0.0
    if n1 > n0:
        pcb.segments = before
        # the nets the new violations name, other than this one: what a
        # coupled re-lay (pack_board) would have to lift for this splice
        _blk = set()
        for _ln in v1:
            if _ln in v0:
                continue
            _blk |= {t_ for t_ in drc_line_nets(_ln) if t_ != nm}
        if hasattr(ctx, 'src_trim_refused'):
            ctx.src_trim_refused.setdefault(nm, []).append((saving, d, s_at, frozenset(_blk)))
        log(f'  source stub trim {nm}: splice {d:.2f} mm at stub depth {s_at:.1f} would add DRC ({n0} -> {n1}): '
            + '; '.join(x[:110] for x in v1 if x not in v0)[:330] + ' -- not this one')
        return 0.0
    lane[:] = [x for x in lane if id(x) not in drop] + ([splice] if splice else [])
    # the lane's tooth end is the splice point now: whoever chains the
    # lane from its tip after this (the pack) must start there
    if hasattr(ctx, 'ends') and nm in ctx.ends:
        ctx.ends[nm] = ((float(proj[0]), float(proj[1])), ctx.ends[nm][1])
    ctx.src_trims[nm] = (saving, len(gone), j, d)
    log(f'  source stub trim {nm}: lane rode its stub {s_at:.1f} mm back toward '
        f'the pad; {len(gone)} stub + {j} lane segment(s) dropped, splice {d:.2f} mm, '
        f'-{saving:.1f} mm')
    return saving


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


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--board', required=True,
                    help='board with BOTH arrays fanned out')
    ap.add_argument('--nets', required=True, help='comma-separated names')
    ap.add_argument('--out', required=True, help='output stem')
    ap.add_argument('--dest', required=True, metavar='REF',
                    help='destination component (its stub ends are '
                         'the targets)')
    a = ap.parse_args(argv)
    return run(a.board, a.nets, a.dest, a.out)


# THE PAIR IS PART OF THE PLAN (Andy, 2026-09-20). Every pair is a corridor
# member -- its slot, page and dives are the one plan's -- and after the
# plan each pair is routed FIRST in its own planned band, widened by the
# first of BRAID_PAIR_SLACKS that lands it, the other members' planned lanes
# reserved, its layers the schedule's; then free in a window as the last
# resort. Its copper is protected and the singles are routed by the same
# plan around it.
PAIR_SLACKS = [float(v) for v in os.environ.get('BRAID_PAIR_SLACKS', '0.6,1.2').split(',') if v.strip()]
PAIR_FANIN = float(os.environ.get('BRAID_PAIR_FANIN', '2.5') or 0)
# BRAID_PAIR_CROSS_FANIN (1): the cross-corridor reservation under the pair's fan-in rule
PAIR_CROSS_FANIN = int(os.environ.get('BRAID_PAIR_CROSS_FANIN', '1') or 0)
PAIR_DIVE_EXTRA = float(os.environ.get('BRAID_PAIR_DIVE_EXTRA', '0.6') or 0)
# BRAID_PAIR_FANIN_BAND (mm, 0 = off): the convergence zone's extra half-width
# beyond the ends' separation (Corridor._pair_fanin_band)
PAIR_FANIN_BAND = float(os.environ.get('BRAID_PAIR_FANIN_BAND', '0.6') or 0)
# BRAID_PAIR_APPROACH (mm): how far in front of a pair's tips a neighbour's
# exit stub may not be reserved (the connector's setback ladder reaches 1.09)
PAIR_APPROACH = float(os.environ.get('BRAID_PAIR_APPROACH', '1.2') or 0)


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


def _clip_stub(p, q, boxes):
    """The reserved stub p -> q cut where it first enters any of the
    `boxes` ((origin, unit direction, half-width across, length along)):
    the kept part, or None when its root already lies inside one."""
    if not boxes:
        return p, q
    t_in = 1.0
    for (e, d, half, length) in boxes:
        def inside(x, y):
            ax, ay = x - e[0], y - e[1]
            along = ax * d[0] + ay * d[1]
            across = abs(ax * d[1] - ay * d[0])
            return -0.02 <= along <= length and across <= half
        if inside(*p):
            return None
        n = 40
        for i in range(1, n + 1):
            t = i / n
            if inside(p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t):
                t_in = min(t_in, max(0.0, (i - 1) / n))
                break
    if t_in >= 1.0:
        return p, q
    if t_in <= 0.0:
        return None
    return p, (p[0] + (q[0] - p[0]) * t_in, p[1] + (q[1] - p[1]) * t_in)


def _route_pairs_planned_in_order(ctx, corridors, log, order):
    """route_pairs_planned's worker over one order of the pairs; returns {pair: (segments, vias)}, the copper left on ctx.pcb."""
    done = {}
    for nm in order:
        c = next(c for c in corridors if nm in c.members)
        M = c.members
        if True:
            t0 = _time.time()
            pn, nn = ctx.pairs[nm]
            pid, nid_n = ctx.byname[pn][0], ctx.byname[nn][0]
            others = [om for om in M if om != nm]
            # the others' planned lanes reserved in the corridor; at the
            # FAN-IN (within PAIR_FANIN mm of either of the pair's ends)
            # only their EXIT STUBS are, a millimetre from each tooth and
            # berth: the lanes there are born on the tooth's layer and cross
            # in front of the pair's teeth, and a pair's pose needs room
            # no single lane needs (measured K36: every pair refused in
            # its planned band with the full lanes reserved, three of
            # three landed with the stubs)
            e_a, e_b = c.teeth[nm], c.stubs[nm]
            (tp_, tn_), (sp_, sn_) = ctx.pair_ends[nm]
            boxes = []
            if BRANCH:
                # BRAID_BRANCH: THE PAIR OWNS THE FIRST MILLIMETRE IN FRONT OF
                # ITS TIPS, and no more (the human's ends): a neighbour's exit
                # stub is reserved as a straight line along ITS escape
                # direction, and a stub that leaves its berth at an angle (the
                # human's SDQ9 enters the DDR's west edge from the north-west)
                # ran across the pair's own approach 0.5 mm out -- SDQS1 refused
                # at every setback, and landed the moment that one line was cut
                # at 0.3 mm. So a stub is clipped where it enters the box in
                # front of either of the pair's ends (the connector's reach,
                # PAIR_APPROACH mm, out along the escape; the tips' separation
                # plus a track and a clearance across), and the others' pieces
                # are CUT there instead of dropped whole within the fan-in.
                # (Off the branch frame the fan-in rule stands: on the zynq
                # article the boxes kept its spread pairs, teeth 1.55 mm apart,
                # out of their bands and the clipped stubs cost DQ6 its exit.)
                for e_, d_, (u_, w_) in ((e_a, ctx.tooth_dir.get(nm), (tp_, tn_)),
                                         (e_b, ctx.stub_dir.get(nm), (sp_, sn_))):
                    if d_ is None:
                        continue
                    sep_ = math.hypot(u_[0] - w_[0], u_[1] - w_[1])
                    boxes.append((e_, d_, sep_ / 2 + TRACK + SPEC_CLEARANCE, PAIR_APPROACH))
            stubs = []
            # a WHOLE-ROUTE plan draws every lane's own exit (whole_ctx.install): the others' planned lines are its
            # reservation there, and a synthetic stub along an escape a lane does not take is an obstacle nothing
            # lays (SDQS1 and SCK refused in their bands behind them, landed in band without)
            drawn = getattr(c, '_geo', None) is not None
            for om in ([] if drawn else others):
                for k_, dirs in ((0, ctx.tooth_dir), (1, ctx.stub_dir)):
                    d = dirs.get(om)
                    e = ctx.ends[om][k_]
                    L = (ctx.tooth_layer if k_ == 0 else ctx.dest_layer)[om]
                    if d is not None and e is not None:
                        st = _clip_stub((e[0], e[1]), (e[0] + d[0] * 1.0, e[1] + d[1] * 1.0), boxes)
                        if st is not None:
                            stubs.append((st[0], st[1], L))
            R = PAIR_FANIN

            def near_end(p, q):
                # by the piece's ENDPOINTS (measured: the segment's distance
                # freed more of the fan-in and cost H3 four vias, 84 -> 88)
                return (min(math.hypot(p[0] - e[0], p[1] - e[1]), math.hypot(q[0] - e[0], q[1] - e[1])) < R
                        for e in (e_a, e_b))
            # the FAN-IN rule: a piece with an end within PAIR_FANIN of either
            # of the pair's ends dropped whole, the stubs whole
            fan_stubs = []
            for om in ([] if drawn else others):
                for k_, dirs in ((0, ctx.tooth_dir), (1, ctx.stub_dir)):
                    d = dirs.get(om)
                    e = ctx.ends[om][k_]
                    if d is not None and e is not None:
                        fan_stubs.append(((e[0], e[1]), (e[0] + d[0] * 1.0, e[1] + d[1] * 1.0),
                                          (ctx.tooth_layer if k_ == 0 else ctx.dest_layer)[om]))
            fan = ([(p, q, L) for (p, q, L) in c.virtual_of(others) if not any(near_end(p, q))] + fan_stubs,
                   [v for v in c.virtual_vias_of(others)
                    if min(math.hypot(v[0] - e[0], v[1] - e[1]) for e in (e_a, e_b)) >= R])
            if BRANCH:
                virt = _clip_boxes(c.virtual_of(others), boxes) + stubs
                vv = [v for v in c.virtual_vias_of(others) if not _in_boxes(v, boxes, VIA_SIZE / 2)]
                # the band tried with the approach boxes first, then with the
                # fan-in rule: a pair whose tips stand apart must converge over
                # whatever runs between them, which the boxes kept reserved --
                # zynq K44's DQS pairs (teeth 0.80 and 1.55 mm apart, DQ6 and
                # DQ0 between DQS0's) refused in their bands under the boxes
                # alone, were routed free of the plan first and walled DQ6
                rules = [(virt, vv), fan]
            else:
                virt, vv = fan
                rules = [fan]
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
                    for sl in PAIR_SLACKS:
                        res = c.route_pair_lane(nm, virt_r, vv_r, slack=sl, free=False)
                        if res is not None:
                            how = f'in its planned band +{sl:.1f} mm' + (' (the fan-in rule)' if ri else '')
                            break
                    if res is not None:
                        break
                if res is None:
                    res = c.route_pair_lane(nm, virt, vv, free=True)
                    if res is not None:
                        how = 'free (its band refused)'
                if res is None:
                    # the last resort: free of the plan's lanes altogether, only
                    # the others' exit stubs reserved, a 6 mm window, no
                    # connectors -- exactly the free-first flow's call, which
                    # landed these pairs (K47: both DQS pairs refused every
                    # planned attempt); the singles' plan was not made round
                    # this copper, so their corridor's rescue pays for it --
                    # against a pair left open, which nothing pays for
                    (tp_, tn_), (sp_, sn_) = ctx.pair_ends[nm]
                    rep = {}
                    res = cn.connect_pair(ctx.pcb, pid, nid_n, tp_, tn_, ctx.tooth_layer[nm],
                                          sp_, sn_, ctx.dest_layer[nm], ctx.cfg, band=None, margin=6.0,
                                          virtual=stubs or None, gap=_pairs.GAP,
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


def route_pairs_planned(ctx, corridors, log):
    """The pairs routed first IN THEIR PLANNED BANDS:
    for each corridor, each pair member, the band ladder then free; a pair
    through waypoints (a termination part) goes by _route_pair_legs. A pair
    that lands is protected; one that refuses stays a member and is tried
    again in its corridor's order (coupled or refused, never singles)."""
    base_segs, base_vias = list(ctx.pcb.segments), list(ctx.pcb.vias)
    order = [nm for c in corridors for nm in c.members if nm in ctx.pairs]
    best = None
    tried = []
    while True:
        ctx.pcb.segments = list(base_segs)
        ctx.pcb.vias = list(base_vias)
        done = _route_pairs_planned_in_order(ctx, corridors, log, order)
        tried.append(order)
        score = (len(done), -sum(len(v) for _s, v in done.values()))
        if best is None or score > best[0]:
            best = (score, order, done, list(ctx.pcb.segments), list(ctx.pcb.vias))
        refused = [b for b in order if b not in done]
        # the ORDER retried, as route_pairs_free does: a refused pair first
        nxt = next(([b] + [o for o in order if o != b] for b in refused
                    if [b] + [o for o in order if o != b] not in tried), None)
        if nxt is None or len(done) == len(order):
            break
        log(f'  pairs: {refused} refused in the order {order}; trying {nxt}')
        order = nxt
    score, order, done, segs, vias = best
    ctx.pcb.segments = segs
    ctx.pcb.vias = vias
    if len(tried) > 1:
        log(f'  pairs: order {order} kept ({len(done)} of {len(order)} pairs)')
    for nm in done:
        ctx.landed.add(nm)
    ctx.protected = set(done)
    ctx.pre_segs = {b: s for b, (s, _v) in done.items()}
    ctx.pre_vias = {b: v for b, (_s, v) in done.items()}
    ctx.base_segments = list(ctx.pcb.segments)
    ctx.base_vias = list(ctx.pcb.vias)
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


def _route_pairs_in_order(ctx, groups, log, order):
    """route_pairs_free's worker: the pairs in `order`, each against the copper of those before it; returns {base: (segments, vias)} of the pairs that landed (their copper left on ctx.pcb)."""
    done = {}
    # every OTHER net's exits, reserved: a short virtual stub from each
    # tooth tip and each berth tip along its escape direction, on the
    # tip's layer, so a pair laid across the front of a tooth row cannot
    # seal a single into its tooth (K36 pf1: SA4 walled by static copper
    # -- the pair SDQS1's copper -- at every stage, a 6 mm free window
    # refused in 0.05 s). BRAID_PAIR_EXIT_RESERVE: the stub's length, mm.
    reach = float(os.environ.get('BRAID_PAIR_EXIT_RESERVE', '1.0') or 0)
    members = [nm for g in groups for nm in g]
    virt = []
    if reach > 0:
        for nm in members:
            if nm in ctx.pairs:
                continue
            for k_, dirs in ((0, ctx.tooth_dir), (1, ctx.stub_dir)):
                d = dirs.get(nm)
                e = ctx.ends[nm][k_]
                L = (ctx.tooth_layer if k_ == 0 else ctx.dest_layer)[nm]
                if d is None or e is None:
                    continue
                virt.append(((e[0], e[1]), (e[0] + d[0] * reach, e[1] + d[1] * reach), L))
    for base in order:
        pn, nn = ctx.pairs[base]
        pid, nid_n = ctx.byname[pn][0], ctx.byname[nn][0]
        (tp_, tn_), (sp_, sn_) = ctx.pair_ends[base]
        rep = {}
        t0 = _time.time()
        # WAYPOINTS (Andy, 2026-09-20: "the tap treatment for CK"): a two-pad
        # part with one pad on P and the other on N -- the pair's
        # termination resistor -- is a place the pair PASSES THROUGH, as
        # the human takes the zynq's CK through R20. The pair is routed in
        # legs, the part's pads standing in for a tooth or a berth, each
        # leg by the same router; every leg must land or the pair is
        # refused whole.
        ways = _pairs.pair_waypoints(ctx.pcb, pid, nid_n, ctx.src_ref.get(base) or ctx.src_ref.get(pn), ctx.ends[base][2])
        if ways:
            res = _route_pair_legs(ctx, base, pid, nid_n, (tp_, tn_), (sp_, sn_), ways, virt, log)
            if res is None:
                log(f'  pair {base}: NOT routed through its {len(ways)} waypoint(s) ({_time.time() - t0:.1f}s) '
                    f'-- it falls to its corridor')
                continue
            segs_o, vias_o = res
            why = _pairs.intra_ok(segs_o, vias_o, pid, nid_n, TRACK, VIA_SIZE, SPEC_CLEARANCE)
            if why is not None:
                log(f'  pair {base}: routed through its waypoints but its legs are not clean ({why}) -- refused')
                continue
            ctx.pcb.segments.extend(segs_o)
            ctx.pcb.vias.extend(vias_o)
            done[base] = (segs_o, vias_o)
            log(f'  pair {base} routed FIRST and protected through {", ".join(w[0].component_ref for w in ways)}: '
                f'{len(segs_o)} segment(s), {len(vias_o)} via(s) ({_time.time() - t0:.1f}s)')
            continue
        # the pair routed FREE: no band, a 6 mm window round its ends
        res = cn.connect_pair(ctx.pcb, pid, nid_n, tp_, tn_, ctx.tooth_layer[base],
                              sp_, sn_, ctx.dest_layer[base], ctx.cfg, band=None,
                              margin=6.0, window_pts=None,
                              virtual=virt or None,
                              gap=_pairs.GAP, a_dir=ctx.tooth_dir.get(base), b_dir=ctx.stub_dir.get(base),
                              a_n_layer=ctx.pair_layers[base][0], b_n_layer=ctx.pair_layers[base][1],
                              report=rep)
        if res is None:
            why = ''
            bl = rep.get('blocked') or []
            if bl:
                g = ctx.cfg.grid_step
                why = (f'frontier {len(bl)} cells in ({min(c[0] for c in bl) * g:.2f},{min(c[1] for c in bl) * g:.2f})-'
                       f'({max(c[0] for c in bl) * g:.2f},{max(c[1] for c in bl) * g:.2f})')
            elif rep.get('empty'):
                why = 'the pose router stopped at the source (no copper)'
            elif rep.get('intra'):
                why = f'legs not clean: {rep["intra"]}'
            elif rep.get('polarity'):
                why = 'polarity crossing lead unroutable'
            log(f'  pair {base}: NOT routed free ({_time.time() - t0:.1f}s{", " + why if why else ""}) '
                f'-- it falls to its corridor')
            continue
        segs_o, vias_o = res
        why = _pairs.intra_ok(segs_o, vias_o, pid, nid_n, TRACK, VIA_SIZE, SPEC_CLEARANCE)
        if why is not None:
            log(f'  pair {base}: routed free but its legs are not clean ({why}) -- it falls to its corridor')
            continue
        ctx.pcb.segments.extend(segs_o)
        ctx.pcb.vias.extend(vias_o)
        done[base] = (segs_o, vias_o)
        log(f'  pair {base} routed FIRST and protected: {len(segs_o)} segment(s), {len(vias_o)} via(s) '
            f'({_time.time() - t0:.1f}s)')
    return done



def route_pairs_free(ctx, groups, log):
    """PAIRS FIRST (Andy, 2026-09-20): every pair is routed before any
    single is planned -- on the fanout board as it stands, by the
    production pair router, free of bands and reservations -- and its
    copper is then PROTECTED: it joins the board's base copper, so the
    corridors plan and route the singles around it, and no rescue, rip
    or re-lay stage can touch it (it is no corridor's member). Measured
    K36 (2026-09-20): free on the fanout board the router landed all
    three pairs DRC-clean, coupled 0.83-0.90; inside the singles' plan --
    single-lane bands, leg strips, dive zones and reservations -- it
    landed none. A pair the free pass refuses stays a corridor member
    and takes the in-band path. Returns the corridor groups without the
    pairs routed here."""
    base_segs, base_vias = list(ctx.pcb.segments), list(ctx.pcb.vias)
    order = list(ctx.pairs)
    best = None
    tried = []
    while True:
        ctx.pcb.segments = list(base_segs)
        ctx.pcb.vias = list(base_vias)
        done = _route_pairs_in_order(ctx, groups, log, order)
        tried.append(order)
        score = (len(done), -sum(len(v) for _s, v in done.values()))
        if best is None or score > best[0]:
            best = (score, order, done, list(ctx.pcb.segments), list(ctx.pcb.vias))
        refused = [b for b in order if b not in done]
        # ORDER (2026-09-20): a pair refused because the pairs before it
        # took its room (zynq K47: DQS1 walled by DQS0's copper) is tried
        # FIRST in a new order; the order that lands the most pairs (then
        # the fewest vias) stands. At most one new order per refused pair.
        nxt = next(([b] + [o for o in order if o != b] for b in refused
                    if [b] + [o for o in order if o != b] not in tried), None)
        if nxt is None or len(done) == len(order):
            break
        log(f'  pairs: {refused} refused in the order {order}; trying {nxt}')
        order = nxt
    score, order, done, segs, vias = best
    ctx.pcb.segments = segs
    ctx.pcb.vias = vias
    if len(tried) > 1:
        log(f'  pairs: order {order} kept ({len(done)} of {len(ctx.pairs)} pairs)')
    for base in done:
        ctx.landed.add(base)
    ctx.protected = set(done)
    ctx.pre_segs = {b: s for b, (s, _v) in done.items()}
    ctx.pre_vias = {b: v for b, (_s, v) in done.items()}
    # the pairs' copper is BASE copper from here: the plan phase's reset
    # keeps it, every map sees it
    ctx.base_segments = list(ctx.pcb.segments)
    ctx.base_vias = list(ctx.pcb.vias)
    groups = [[nm for nm in g if nm not in done] for g in groups]
    return [g for g in groups if g]


def plan_corridors(board, names, dest, log):
    """The braid up to its first route: the context (setup), one corridor
    per group -- under BRAID_BRANCH each a trunk with branches -- and every
    corridor PLANNED, the board's copper reset to its base. (ctx,
    corridors): what run() routes, and what the plan tools (plan_audit.py,
    route_lanes.py) audit, so a tool never plans differently from the run."""
    ctx, groups = setup(board, names, dest, log, pairs=bool(PAIRS))
    ctx.pre_segs, ctx.pre_vias, ctx.protected = {}, {}, set()
    corridors = []
    for ci, members in enumerate(groups):
        corridors.append(Corridor(ci, members, ctx, log))
    ctx.corridors = corridors
    if BRANCH:
        # a trunk with branches (build_branches), each corridor planned
        # once as today to find its side exits and their block slots
        for i_, c_ in enumerate(corridors):
            try:
                c_.run(plan_only=True)
                corridors[i_] = build_branches(ctx, c_, log)
            except Exception as e:
                log(f'  branch build: corridor {c_.idx} left as it was ({e})')
        ctx.corridors = corridors
        ctx.laid, ctx.laid_tubes = [], []

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
    return ctx, corridors


def run(board, nets, dest, out):
    """The braid as a FUNCTION (2026-09-18): what `main` does for one
    command line, callable in-process -- a resident probe worker
    (probe_worker.py) braids one probe after another without paying the
    interpreter, the imports, the solver priming and the taut memo's
    load each time (~1 s of a 4 s probe braid, measured). The knobs a
    probe sets through the environment (BRAID_ATTEMPTS, BRAID_BUDGET_X)
    are module globals here, so a caller sets ATTEMPTS / BUDGET_X
    itself; BRAID_SMOOTH is read at write time from the environment."""
    import types as _types
    a = _types.SimpleNamespace(board=board, nets=nets, dest=dest, out=out)
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
    # THE DESIGN CONSTANTS (rules.py), installed once per stage-process --
    # inert today (it installs what the constants above already hold) and
    # the SEAM for when the main router supplies the geometry instead.
    _r = _rules.install_defaults()
    # printed from THIS MODULE'S OWN constants, never from the Rules object:
    # a wiring fix can be inert, and an early version of this one was --
    # install could not see the module because a stage runs as '__main__',
    # so it printed numbers the router was not using.
    log(f'rules: clearance {SPEC_CLEARANCE} (hug {CLEAR}), track {TRACK}, '
        f'via {VIA_SIZE}/{VIA_DRILL}, via_need {VIA_NEED:.4f}  '
        f'[{_r.source}]')
    ctx, corridors = plan_corridors(a.board, names, a.dest, log)
    if PAIRS and getattr(ctx, 'pairs', None):
        route_pairs_planned(ctx, corridors, log)
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
    if plan.get('alts'):
        q['alts'] = {nm: [dict(a, exit=list(M(*a['exit'])), layer=other_layer(a['layer']),
                             **({'dir': [a['dir'][0], -a['dir'][1]]} if a.get('dir') else {}))
                          for a in al]
                     for nm, al in plan['alts'].items()}
    return q


def setup(board, names, dest, log, plan=None, pairs=False):
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
    # PAIR MEMBERS (BRAID_PAIRS, pairs.py): the two legs of a differential
    # pair become ONE member named after the pair. The P leg's net stands
    # in wherever a member needs a net id (the obstacle models, the
    # source reference); both legs' ids are the run's (kids), so neither
    # leg's copper is static copper to any lane.
    # ...only where the braid ROUTES (run): the plan's judge (plan_braid,
    # braid_slots) keeps the legs as the singles its model plans, and the
    # dicts it reads back stay keyed by the plan's own names
    _pairs_here = {}
    if PAIRS and pairs:
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
    ctx.pairs = _pairs_here
    if chi < 0:
        CY = mirror_axis(pcb)     # the engine's own rule: a lattice line
        pcb, M = to_front_frame(pcb, dest)
        ctx.M = M
        byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
        _standin(byname)
        if plan:
            plan = mirror_plan(plan, M)
        log(f'pair frame: chirality -1, the board turned over about '
            f'y = {CY:.3f} for the braid (copper mirrored back on write)')
    planned = {nm for nm in names if plan and nm in plan.get('ends', {})}
    ctx.pages_first = bool(plan and plan.get('pages_first'))
    if ctx.pages_first and EXACT_PAGES_ENV != '0':
        # a PAGES-FIRST plan (fanout_from_plan PLAN_PAGES): its two chains
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
    for nm in [n for n in names if n in planned]:    # the run's order: a set's would follow the hash seed
        e = plan['ends'][nm]
        ends[nm] = (tuple(e[0]), tuple(e[1]), dest)
    for n in _legs:
        if n in _pl_ends:
            e = _pl_ends[n]
            ends[n] = (tuple(e[0]), tuple(e[1]), dest)
    ctx.pair_ends = {}
    for _b, (_pn, _nn) in list(_pairs_here.items()):
        (sp_, tp_, _), (sn_, tn_, _) = ends[_pn], ends[_nn]
        _ds, _dt = ts.d2(sp_, sn_) ** 0.5, ts.d2(tp_, tn_) ** 0.5
        # the limit: two lane pitches, or 1.3 of the destination's ball
        # pitch -- a harmonised pair's tips stand one ball apart
        _dg = em.grid_of(pcb.footprints[dest])
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
        ends[_b] = (_pairs.mid(sp_, sn_), _pairs.mid(tp_, tn_), dest)
        ctx.pair_ends[_b] = ((sp_, sn_), (tp_, tn_))
    ctx.pairs = _pairs_here
    ctx.pcb, ctx.byname, ctx.ends, ctx.kids = pcb, byname, ends, kids
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
    ctx.src_chain = {}
    ctx.src_trims = {}
    ctx.src_trim_refused = {}
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

        _k = _snapper()

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
        # THE SOURCE STUB CHAIN (2026-09-19): the same walk from the
        # tooth tip toward the pad, for the write-time source trim
        # (note_source_joint) -- a lane that ran back along its own
        # tooth is spliced onto the stub where it left it.
        # no hop cap to speak of: a channel escape through the array is
        # hundreds of grid steps, and the berth chain's 24 reached 3 mm of
        # DQ13's 15 (K44) -- the deep return point was never a candidate
        ctx.src_chain[nm] = _walk_stub(segs_n, _k(*ends[nm][0]),
                                       ctx.tooth_layer[nm], _stop, _k, max_hops=5000)
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
        net's stub is in the way). A PAIR member's copper is BOTH its
        legs' -- its N leg's tooth stood as foreign copper in front of
        the pair's midpoint and made every pair a joiner."""
        k = (nm, layer, tuple(sorted(members)))
        if k not in _cache:
            ids = set()
            for m in members:
                ids.add(byname[m][0])
                if m in _pairs_here:
                    ids.add(byname[_pairs_here[m][1]][0])
            _cache[k] = build_obstacles(pcb, byname[nm][0], ids, layer)
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


def _write_layer_jumps(board_path, names):
    """Nets whose own copper meets on two layers with no barrel and no
    plated pad there -- a layer change with nothing to make it. Returns
    [(net, x, y)]. Read off the WRITTEN file, so it sees what shipped
    rather than what the bookkeeping believes."""
    import collections as _c
    try:
        pcb = parse_kicad_pcb(board_path)
    except Exception:
        return []
    want = set(names)
    by = {i: n.name.split('/')[-1] for i, n in pcb.nets.items()}
    ends = _c.defaultdict(lambda: _c.defaultdict(set))
    holes = _c.defaultdict(list)
    for sg in pcb.segments:
        n = by.get(sg.net_id)
        if n in want:
            ends[n][(round(sg.start_x, 3), round(sg.start_y, 3))].add(sg.layer)
            ends[n][(round(sg.end_x, 3), round(sg.end_y, 3))].add(sg.layer)
    for v in pcb.vias:
        n = by.get(v.net_id)
        if n in want:
            holes[n].append((v.x, v.y))
    for fp in pcb.footprints.values():
        for p in fp.pads:
            if p.drill and p.drill > 0 and by.get(p.net_id) in want:
                holes[by[p.net_id]].append((p.global_x, p.global_y))
    out = []
    for n, pts in ends.items():
        for (x, y), lays in pts.items():
            if len(lays) < 2:
                continue
            if not any((hx - x) ** 2 + (hy - y) ** 2 < 0.04 for hx, hy in holes.get(n, ())):
                out.append((n, x, y))
    return out


def _seg_dist_so(p, q, a, b):
    """Distance between segments p-q and a-b in the (s, o) plane (an
    isometric frame of a straight spine)."""
    def pd(pt, u, v):
        dx, dy = v[0] - u[0], v[1] - u[1]
        L2 = dx * dx + dy * dy
        if L2 < 1e-12:
            return math.hypot(pt[0] - u[0], pt[1] - u[1])
        t = max(0.0, min(1.0, ((pt[0] - u[0]) * dx + (pt[1] - u[1]) * dy) / L2))
        return math.hypot(pt[0] - u[0] - t * dx, pt[1] - u[1] - t * dy)

    def cr(o_, a_, b_):
        return (a_[0] - o_[0]) * (b_[1] - o_[1]) - (a_[1] - o_[1]) * (b_[0] - o_[0])
    d1, d2, d3, d4 = cr(p, q, a), cr(p, q, b), cr(a, b, p), cr(a, b, q)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)) and d1 != 0 and d2 != 0:
        return 0.0
    return min(pd(p, a, b), pd(q, a, b), pd(a, p, q), pd(b, p, q))


def pitch_violations(bp, q=None):
    """The PLAN-SIDE PITCH CHECK (2026-09-15): every pair of page lanes on
    one page, in one corridor, whose planned centrelines come closer than
    `q` (TRACK + CLEAR + a router's cell or two) perpendicular somewhere
    both are on that page. Such a promise cannot be kept -- the braid
    refuses the lane in band and the last call re-lays it (the K35/K41/K51
    DQ group round C5, wall_probe census) -- so the planner should treat
    the lane as it treats a swimmer: re-plan it with that berth barred.
    Returns {net: [partner, ...]} over the nets of every violating pair."""
    q = (TRACK + CLEAR + 0.05) if q is None else q

    def on_layer(d, s_, L):
        for (a, b, RL) in d.get('req', ()):
            if a <= s_ <= b and RL != L:
                return False
        if L == 'B.Cu' and not any(lo <= s_ <= hi for (lo, hi) in d.get('bwin', ())):
            return False
        return True
    out = {}
    names = [n for n, d in bp.items() if d.get('page') and d.get('mid') and len(d['mid']) > 1]
    for i, a in enumerate(names):
        da = bp[a]
        for b in names[i + 1:]:
            db = bp[b]
            if da['page'] != db['page'] or da.get('corridor') != db.get('corridor'):
                continue
            L = da['page']
            worst = None
            for (p, qq) in zip(da['mid'], da['mid'][1:]):
                for (u, v) in zip(db['mid'], db['mid'][1:]):
                    lo, hi = max(p[0], u[0]), min(qq[0], v[0])
                    if hi < lo:
                        continue
                    d = _seg_dist_so(p, qq, u, v)
                    if d >= q:
                        continue
                    s_m = (lo + hi) / 2
                    if on_layer(da, s_m, L) and on_layer(db, s_m, L):
                        if worst is None or d < worst:
                            worst = d
            if worst is not None:
                out.setdefault(a, []).append(b)
                out.setdefault(b, []).append(a)
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

    def _chord_islands(c, nm, page):
        """The static islands (corridor parts) on `page` that the lane's
        region chord -- launch slot at s0 to target slot at s1 -- passes
        through. The plan-side ISLAND PRICE reads it (pages_first
        PLAN_PAGES_ISLAND): a berth whose lane must cross a part on its
        own layer is dearer by what the bend round it costs (2026-09-15,
        the C5-deflected DQ group refused in band at every K >= 35)."""
        try:
            isl = c.static_islands().get(page, ())
            a = (c.s0, c.launch_o[nm])
            b = (c.s1, c.target_o[nm])
        except Exception:
            return []
        hit = []
        own = {ctx.src_ref.get(nm), ctx.ends[nm][2]} if nm in ctx.ends else set()
        for (s_lo, s_hi, o_lo, o_hi, what) in isl:
            if what.split('.')[0] in own:
                continue            # the run's own arrays are not corridor parts (SZQ's chord through U1)
            # segment a-b against the box, Liang-Barsky
            t0, t1 = 0.0, 1.0
            dx, dy = b[0] - a[0], b[1] - a[1]
            ok = True
            for pq, qv in ((-dx, a[0] - s_lo), (dx, s_hi - a[0]), (-dy, a[1] - o_lo), (dy, o_hi - a[1])):
                if abs(pq) < 1e-12:
                    if qv < 0:
                        ok = False
                        break
                    continue
                t = qv / pq
                if pq < 0:
                    t0 = max(t0, t)
                else:
                    t1 = min(t1, t)
                if t0 > t1:
                    ok = False
                    break
            if ok:
                hit.append(what)
        return hit
    for c in corridors:
        sc = getattr(c, 'sched_cur', None)
        li = {nm: i for i, nm in enumerate(getattr(c, 'launch', []))}
        ti = {nm: i for i, nm in enumerate(getattr(c, 'target', []))}
        for nm in c.members:
            pg_ = sc.page.get(nm) if sc else None
            out[nm] = {'corridor': c.idx,
                       'islands': (_chord_islands(c, nm, pg_) if pg_ else []),
                       'launch_idx': li.get(nm), 'target_idx': ti.get(nm),
                       'page': (sc.page.get(nm) if sc else ctx.tooth_layer[nm]),
                       'birth_b': bool(sc and nm in sc.birth_b),
                       'joiner': nm in getattr(c, 'joiners', ()),
                       'side_exit': nm in getattr(c, 'siders', ()),
                       # a side exiter's exit leg runs on its block's layer:
                       # a via where that is not the lane's page, another
                       # where it is not the berth's layer
                       'exit_leg_layer': getattr(c, 'leg_layer', {}).get(nm),
                       # the PLANNED lane polyline, so a caller can price
                       # a lane the schedule gave no profile (a swimmer)
                       # by the crossings it will actually have to make
                       'lane': list(getattr(c, 'lane_xy', {}).get(nm, ()) or ()),
                       # the layer changes the lane's whole profile needs
                       # (tooth, page, tail stretches, exit leg, berth):
                       # the vias the plan implies for a page lane. A
                       # swimmer (page None) has no profile; its model
                       # stays SWIM_VIAS.
                       'changes': (len(c.layer_profile(nm)) - 1
                                   if sc and sc.page.get(nm) is not None
                                   and hasattr(c, 'req') else None),
                       # a SWIMMER's: the changes its hold-then-run line
                       # implies (lay_lanes' swimmer census), tooth to
                       # berth; a swimmer crossing no page lane owes only
                       # its tooth/berth mismatch. The judge reads it under
                       # SWIM_CHANGES (plan_ends.vias_from_pages); the flat
                       # SWIM_VIAS otherwise
                       'swim_changes': (getattr(c, 'swim_changes', {}).get(
                           nm, 1 if ctx.tooth_layer[nm] != ctx.dest_layer[nm] else 0)
                                        if sc and sc.page.get(nm) is None else None),
                       # dives under EARLIER corridors' lanes (2 each)
                       'cross_vias': cross.get(nm, 0),
                       # the planned (s, o) geometry, for the plan-side
                       # PITCH CHECK (pitch_violations): the lane's polyline,
                       # its required-layer stretches and back-layer windows,
                       # the corridor's region
                       'mid': [tuple(q) for q in getattr(c, 'mid', {}).get(nm, ())],
                       'req': [tuple(r) for r in getattr(c, 'req', {}).get(nm, ())],
                       'bwin': [tuple(b) for b in getattr(c, 'bwin', {}).get(nm, ())],
                       's0': getattr(c, 's0', None), 's1': getattr(c, 's1', None)}
    if ctx.M is not None:
        for d in out.values():
            for k in ('page', 'exit_leg_layer'):
                if d.get(k) is not None:
                    d[k] = other_layer(d[k])
    return out


def write_out(a, ctx, corridors, names, log):
    """Smooth, write the board with the Eco overlay, report."""
    pcb, byname, ends, kids = ctx.pcb, ctx.byname, ctx.ends, ctx.kids
    out_segs = {nm: c.out_segs.get(nm, []) for c in corridors for nm in c.members}
    out_vias = {nm: c.out_vias.get(nm, []) for c in corridors for nm in c.members}
    # ...and the pairs routed FIRST (route_pairs_free), no corridor's members
    out_segs.update(getattr(ctx, 'pre_segs', {}) or {})
    out_vias.update(getattr(ctx, 'pre_vias', {}) or {})
    refused = sorted(nm for c in corridors for nm in c.refused)
    # PAIR MEMBERS (pairs.py): the caller's `names` are the LEGS; the
    # member's copper is split back to its legs by net id for the writer,
    # a refused pair refuses both legs, and the legs are kept out of the
    # per-net trims and the smoother (each would treat one leg alone)
    _prs = getattr(ctx, 'pairs', {}) or {}
    _legs = set()
    for _b, (_pn, _nn) in _prs.items():
        log(f'  pair {_b} at write: {len(out_segs.get(_b, []))} lane segment(s), '
            f'{sum(1 for s in pcb.segments if s.net_id in (byname[_pn][0], byname[_nn][0]))} on the board'
            + (' REFUSED' if _b in refused else ''))
        for _leg in (_pn, _nn):
            _legs.add(_leg)
            _lid = byname[_leg][0]
            if _b in out_segs:
                out_segs[_leg] = [s for s in out_segs[_b] if s.net_id == _lid]
                out_vias[_leg] = [v for v in out_vias.get(_b, []) if v.net_id == _lid]
    if _prs:
        refused = sorted(set(refused) | {leg for b in refused if b in _prs for leg in _prs[b]})
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
    # THE RE-ESCAPE (re_escape.py, BRAID_RE_ESCAPE mm, 0 = off): a lane whose
    # stub + lane run far over its pad-to-berth airline is routed again from
    # its PAD, free, and ships only when cheaper at ECON_MM_PER_VIA and no
    # worse in scoped DRC -- the other half of the source trim; BEFORE the
    # trims, which re-anchor a lane's ends and drop stub tips (K36 SDQ0 was
    # routed to a berth the berth trim had just removed: open)
    import re_escape as _re
    ctx.re_escapes = {}
    ctx.re_escape_vias = []
    if _re.RE_ESCAPE > 0:
        _cands = [nm for nm in names if out_segs.get(nm) and nm not in refused and nm not in _legs]
        _cands.sort(key=lambda nm: -_re.excess(ctx, nm, out_segs[nm]))     # the worst offender first
        _mm2 = sum(_re.re_escape(ctx, nm, out_segs[nm], out_vias.setdefault(nm, []), a.board, log, cn,
                                 ECON_MM_PER_VIA if ECON_MM_PER_VIA > 0 else 6.0)
                   for nm in _cands)
        if ctx.re_escapes:
            log(f're-escape: {len(ctx.re_escapes)} lane(s) routed again from their pads, -{_mm2:.1f} mm '
                f'({", ".join(f"{k} {v[1]}->{v[2]} via(s)" for k, v in sorted(ctx.re_escapes.items()))})')
    for nm in names:
        if out_segs.get(nm) and nm not in refused and nm not in _legs:
            note_joint(ctx, nm, out_segs[nm])
    # deferred SOURCE trim (2026-09-19): a lane that rode back along its
    # own tooth is spliced onto the stub where it departed; the lane list
    # is edited in place so the smoother below sees the spliced lane
    _mm = sum(note_source_joint(ctx, nm, out_segs[nm], out_vias.get(nm, []), a.board, log)
              for nm in names if out_segs.get(nm) and nm not in refused and nm not in _legs)
    if ctx.src_trims:
        log(f'source stub trim: {len(ctx.src_trims)} lane(s) spliced onto their stubs, '
            f'-{_mm:.1f} mm ({", ".join(sorted(ctx.src_trims))})')

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
    _res_list = [{'new_segments': list(out_segs.get(nm, []))} for nm in names]
    if os.environ.get('BRAID_SMOOTH', '1') != '0':
        _n, _nets, _rm, _addl, stt = smooth_octolinear_chains(
            [r for k, r in enumerate(_res_list) if names[k] not in _legs],
            pcb, kids, clearance=0.1, keep_input_copper=True)
    else:
        # BRAID_SMOOTH=0 (2026-09-18): a PROBE braid skips the smoother. It
        # never changes a via count, and it was ~3.7 s of a ~6 s two-lane
        # local braid at K51 (it validates against the whole board's
        # copper). The probe's judge is (open, drc, vias); the final board
        # is smoothed once (replan / smooth_board.py).
        _n, _nets, _rm, _addl, stt = 0, 0, 0, 0, {}
    if a.out != os.devnull:
        # THE PACK SIDECAR (<out>.pack.json): what pack.py needs to pack
        # this board again on its own -- each lane's copper as the pack
        # would receive it (smoothed), every corridor's members and target
        # order, its planned centrelines, each lane's tooth and stub end
        # -- in the WRITTEN board's frame. `pack_board.py BOARD` packs a
        # braided board in seconds where the braid took a minute
        import json as _json_pk
        _M = ctx.M if ctx.M is not None else (lambda x, y: (x, y))
        _OL = other_layer if ctx.M is not None else (lambda L: L)
        _side = {
            'layers': [_OL(L) for L in ctx.cfg.layers],
            'board_edge_clearance': float(getattr(ctx.cfg, 'board_edge_clearance', 0.0) or 0.0),
            'corridors': [{
                'members': list(c.members),
                'target': list(getattr(c, 'target', c.members)),
                'lane_xy': {nm: [list(_M(*p_)) for p_ in poly]
                            for nm, poly in (getattr(c, 'lane_xy', {}) or {}).items()},
            } for c in corridors],
            'ends': {nm: [list(_M(*ends[nm][0])), list(_M(*ends[nm][1]))] for nm in names},
            # the destination stub chain, tip-side first, as it stands on
            # the written board (the trim above may have shortened it)
            'dest_chain': {nm: [[*_M(*_t), *_M(*_p), _OL(s_.layer)]     # tip, pad
                                for (s_, _t, _p) in (ctx.dest_chain.get(nm) or [])
                                if s_ in pcb.segments]
                           for nm in names},
            'lanes': {nm: {
                'segs': [[*_M(s.start_x, s.start_y), *_M(s.end_x, s.end_y), _OL(s.layer), s.width]
                         for s in _res_list[k]['new_segments']],
                'vias': [list(_M(v.x, v.y)) for v in out_vias.get(nm, [])],
            } for k, nm in enumerate(names)},
        }
        with open(a.out + '.pack.json', 'w') as _f:
            _json_pk.dump(_side, _f)
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
        out_segs = {nm: c.out_segs.get(nm, []) for c in corridors for nm in c.members}
        out_vias = {nm: c.out_vias[nm] for c in corridors for nm in c.members}
        # ...and may end a lane at another vertex of its stub: the trim again
        for nm in names:
            if out_segs.get(nm) and nm not in refused:
                note_joint(ctx, nm, out_segs[nm])
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
        if getattr(ctx, 're_escape_vias', None):
            txt = _re.strip_vias_at(txt, ctx.re_escape_vias)
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
            # through the writer's generator: a uuid, so the Type VII
            # stamp below can name the via (#962)
            add.append(generate_via_sexpr(round(vx, 4), round(vy, 4), VIA_SIZE, VIA_DRILL,
                                          ['F.Cu', 'B.Cu'], nid) + '\n')

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
    ship_vias.stamp(out_board, 'braid', log)     # a via in a pad declares Type VII (#962)
    pro = os.path.splitext(a.board)[0] + '.kicad_pro'
    if os.path.exists(pro):
        shutil.copy(pro, a.out + '.kicad_pro')
        # the pairs routed FIRST are PROTECTED down the chain too (#521): the
        # output project records their legs, so a later route step's rip
        # globs and plane repairs leave them alone
        prot = getattr(ctx, 'protected', None) or set()
        if prot:
            try:
                from protected_nets import persist_protected_nets
                _names = {i: n.name for i, n in ctx.pcb.nets.items()}
                _map = {}
                for _b in sorted(prot):
                    for _leg in ctx.pairs.get(_b, ()):
                        _lid = ctx.byname[_leg][0]
                        _map[_names.get(_lid, _leg)] = 'diff pair routed first by the bus braid'
                persist_protected_nets(a.out + '.kicad_pro', _map, verbose=False)
                log(f'  protected in the project: {", ".join(sorted(_map))}')
            except Exception as e:  # a project note must not lose the board
                log(f'  protected nets NOT recorded: {e}')
    # STAMP THE ROUTED FLOOR into the output project (2026-09-13), the way
    # every production CLI does: the copy above carried the bench's project
    # down every chain step with its Default class clearance at 0.0 (KiCad's
    # "not configured", inherited from the human original), which route.py
    # pins up to the fab floor and the GUI replaces with its own control
    # default -- a 99-via board opened in the plugin routed at 0.25 on
    # copper laid at 0.1. Lower-only, so a project already at the floor is
    # untouched; the copper is unaffected either way (the braid routes from
    # ctx.cfg, never from the project).
    if os.environ.get('AWX_STAMP_PRO', '1') != '0':   # 0 = the flag-off parity control
        try:
            from fix_kicad_drc_settings import fix_project_for_output
            fix_project_for_output(out_board, a.board,
                                   clearance=min(SPEC_CLEARANCE, ctx.cfg.clearance),
                                   track_width=ctx.cfg.track_width,
                                   via_diameter=VIA_SIZE, via_drill=VIA_DRILL,
                                   verbose=False)
        except Exception as e:  # a project problem must not lose the board
            log(f'  project floor NOT stamped: {e}')
    nv = sum(len(v) for v in out_vias.values())
    nseg = sum(len(emit[nm]) for nm in names)
    # A VIA-LESS LAYER CHANGE IS A BROKEN NET, and it ships silently: the
    # net is not refused, so nothing in the summary or the refusal list
    # mentions it and only check_connected finds it. Measured over 391
    # recorded runs: 48 of 146 open nets were "silent" and 47 of those 48
    # break exactly here. The invariant is cheap and total -- wherever a
    # net's own copper meets on two layers there must be a barrel or a
    # plated pad -- and it was 0 on every complete board tested.
    _jump = _write_layer_jumps(out_board, names)
    if _jump:
        log(f'  WARNING via-less layer change on {len(_jump)} net(s): '
            + ', '.join(f'{n}@({x:.2f},{y:.2f})' for n, x, y in _jump[:6])
            + ' -- these ship OPEN and are not in the refused list')
    log(f'\nwrote {out_board}: {nseg} segments, {nv} vias'
        + (f' -- {len(refused)} net(s) REFUSED' if refused else '')
        + (f' -- {len(_jump)} VIA-LESS LAYER CHANGE(S)' if _jump else ''))
    return 1 if refused else 0


if __name__ == '__main__':
    sys.exit(main())

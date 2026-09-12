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
from kicad_parser import parse_kicad_pcb, _global_to_local  # noqa: E402
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
BAND_GAP = TRACK + CLEAR + 0.07  # a band comb starts this far inside a stub-tip
                               # line: a lane passing a stub's END at the
                               # legal minimum plus a hair
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
# PROBE knobs (replan.py, 2026-09-10): a local re-braid of a few nets on a
# frozen board is a SCREEN -- a refusal there is unjudged whatever the
# budget -- and its cost is failing searches repeated: three identical
# attempts (nothing changes between them when the refused set repeats)
# and last-call searches at 1.6 M iterations for a net that will not
# route. BRAID_ATTEMPTS caps the attempt ladder, BRAID_BUDGET_X the
# rescue / last-call budget multiplier. Defaults = the braid as it was
# (measured: one attempt gives the same board 22% faster; a halved
# budget loses nets, so the probe keeps the full one).
SWIM_PRICE = 2                 # the planner's price for a lane that cannot
                               # run in-band (a dive and a surface)
ATTEMPTS = int(os.environ.get('BRAID_ATTEMPTS', '6'))
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
PACK_MODE = int(os.environ.get('BRAID_PACK', '0') or 0)  # pack.py at write time (opt-in)
FLANK_COMB = int(os.environ.get('BRAID_FLANK_COMB', '0') or 0)
# ^ arrivals on a face that runs ALONG the spine are a comb ordered along
# the face (#622, 2026-09-10 evening). 1: a stub standing beside its
# destination array (outside the ball field across the spine, at an s
# within it) is never a head-on exit -- it takes a slot in its side's
# exit block, whose ports are ordered by s already. The head-on test
# reads "another stub upstream at my offset" with DIST_O = 0.2 mm, and a
# spine tilted 4 degrees to the face spreads one face's stub offsets by
# 0.9 mm over 12 mm: mid-face stubs then read as head-on and are ordered
# among the block's lanes by that tilt (human bench K41: SA2/SA4 head-on
# at their own o INSIDE the block, their neighbour SA0 a sider; five of
# the ten swimmers were such stubs). 2: the whole block, joined lanes
# included, is ordered by the stub's s (first exiter innermost) instead
# of ports-by-s then joined-in-join-order. 0: off, byte-identical.
FLANK_MIN = 0.2                # a stub this far beyond its array's local ball line, across the spine, stands beside the array
SWIM_HOLD = int(os.environ.get('BRAID_SWIM_HOLD', '0') or 0)
# BRAID_ONE_DIVE=1 (2026-09-11): the human's rule as the SCHEDULE. A net
# whose tooth and berth share a layer STAYS on it end to end; a net whose
# ends differ changes ONCE, at a point chosen so that every crossing it
# takes is on different layers -- the change points of all such divers
# solved jointly (a small MILP, the crossings fixed where the ribbon lines
# meet); what cannot be made legal that way SWIMS as before. The LIS
# pages dove every back-page lane at the launch, so the back page had to
# be a crossing-free chain and mutually inverted divers swam at three
# changes: the K28 stayers plan (residue 1 by the one-dive check) became
# 11 swimmers and 57-61 vias for a plan the check laid at ~2 a net.
ONE_DIVE = int(os.environ.get('BRAID_ONE_DIVE', '0') or 0)
DIVE_ROOM = float(os.environ.get('BRAID_DIVE_ROOM', '0.3'))   # s either side of a scheduled change kept open on both layers
# BRAID_ONE_DIVE=2: the GENERALISED profile -- 0, 1 or 2 scheduled changes
# per net (a stayer may RIDE the other layer between two change points; a
# full-length ride is the page scheme's back-page lane exactly, laid by
# that code path), the same fixed crossings, the fewest changes + swims.
# Pages are the special case, so the model cannot lose to them; one
# change alone could (K28 greedy berths: 38 under pages, 44 under level 1).
RIDE_W = float(os.environ.get('BRAID_RIDE_W', '2.0'))    # a stayer's two-change ride
SWIM_W = float(os.environ.get('BRAID_SWIM_W', '2.5'))    # a net left to swim (exact_pages' price)
# BRAID_ONE_DIVE=3: the profiles chosen on PROXIMITY, not crossings. Level
# 2 constrained layers at the crossing points and asked DIVE_ROOM of s
# either side; measured at K35 (wall census at the last call) two lanes
# crossing at a shallow angle stay within track clearance for a
# millimetre on both sides, and a via next to a converging line has no
# room -- 8-10 lanes refused in band on a plan pages route in band
# entirely. Level 3 samples the lines along s: wherever two lines are
# within PROX_TRACK the lanes must be on different layers; a change may
# sit only in a VIA SLOT (no other line within VIA_NEED, the diamond
# rule) or at the region's ends (the page scheme's birth / landing via);
# the laid gap around a change is the via's footprint (DIVE_GAP), and
# the virtual copper is stamped on one layer either side of it.
PROX_TRACK = float(os.environ.get('BRAID_PROX_TRACK', '0.25'))  # two same-layer lines nearer than this: a short
# A VIA'S room from a foreign MID line, measured in xy rather than in o
# (BRAID_VIA_ROOM_XY). A via is a DISC: its room is radial, so pricing it
# across the spine overstates the clearance by the secant of the lanes'
# angle -- and the fan is where lanes slant. Two mid LINES keep the o
# metric (two steep parallels 0.35 apart in o are 0.2 apart in xy and
# pairing them over their length makes the fan unschedulable); only the
# via disc changes. It is affordable because SLOPE_PITCH already widens
# an adjacent same-page pitch by that same secant, so adjacent pairs
# already clear VIA_NEED in xy: what this catches is the CROSSING pair,
# which the o metric cannot see at all (K35 jc1: SDQ7's change at s 12.23
# landed its via on SCKE0's line at s 11.84-12.02, refusing SCKE0 in band
# on every attempt -- wall_probe).
# 1 = only where the two lanes CROSS (the o metric cannot see that pair
# at all); 2 = every mid pair, which is physically honest but denies the
# router slack it had: the model's slot is a PERMISSION to change layer
# near there, not a via position, and at 2 the K35 greedy plan schedules
# 6 vias worse (64 -> 70, 2035 -> 2556 segments) with nothing refused
# either way. A lane running ALONGSIDE is governed by the comb pitch
# (SLOPE_PITCH already scales it by the secant); a lane CROSSING is not
# governed by anything, and is where the via lands on the line.
VIA_ROOM_XY = int(os.environ.get('BRAID_VIA_ROOM_XY', '0') or 0)
# ...and the same test turned on AS FEEDBACK, for the attempts after one
# that refused (BRAID_VIA_ROOM_REFUSED, OPT-IN -- see the constant). As a
# STATIC knob the honest metric is one bench each way -- jc1's tight
# 7-swimmer K35 plan 62 -> 54 vias, the loose greedy K35 chain 62 -> 74 --
# because on a plan that refuses NOTHING the model was not lying: a slot
# is a permission to change layer near there, and the A* has slack inside
# the band to place the via off it. Where the corridor is congested that
# slack is gone and the lie becomes a refusal. So the loop tightens the
# metric only where reality disagreed with the model, beside the launch
# pitch it already widens -- a plan that routes in band never pays it.
# DEFAULT 0 since 2026-09-12. It shipped at 2 on "better at K41, identical
# at K15/K28/K35, worse on none" -- but K51 was outside that grading ladder
# and IS worse there: `=0` gives 137 vias / 0 open / 0 DRC, `=2` gives 128 /
# 3 open / 30 DRC. It buys 9 vias for three open nets and thirty violations.
# Opt-in until the escalation can tell a corridor it will help from one it
# will break.
VIA_ROOM_REFUSED = int(os.environ.get('BRAID_VIA_ROOM_REFUSED', '0') or 0)
DIVE_GAP = float(os.environ.get('BRAID_DIVE_GAP', '0.15'))     # s either side of a level-3 change open on both layers
PROF_DS = 0.1                                                  # the sampling step along s
# BRAID_ONE_DIVE=4: EVERY lane scheduled -- up to MAXCH changes each, all
# at via slots, all proximity-legal, the objective the exact via count
# (a change = a via); a net swims only when no profile fits at all (at a
# high price). Measured at level 3 on the greedy K35 board: the scheduled
# lanes route in band at pages' cost and the 11 FREE swimmers cost 45
# vias for pages' 33 -- the weave through a profile lattice is dearer.
MAXCH = int(os.environ.get('BRAID_MAXCH', '4'))
_PROFILE_MEMO = {}                                             # (geometry key) -> (solution, message)
VIA_SEP = 0.4                                                  # two changes of one lane at least this far apart in s
RESIDUE_W = float(os.environ.get('BRAID_RESIDUE_W', '6.0'))    # a net no profile fits (level 4)
# BRAID_ONE_DIVE=5: the schedule over the TAIL as well (2026-09-11, late).
# Level 4 models [s0, s1] as straight lines; the tail -- the exit run at
# its slot, the exit leg along o at its s, the jog along the stub's row
# back into the tip, a head-on lane's diagonal to its stub -- was the old
# leg economics: a leg's layer priced against the RUNS it crosses, and a
# leg crossing a JOG or a stub not priced at all (human bench K41: SA7's
# F leg over SRST's F jog, both refused in band every attempt, and the
# router's improvisations around it walled the far face for five more).
# Level 5 samples every lane's whole polyline in ARC LENGTH, in board
# xy: proximity between lanes is Euclidean (a leg crossing a run, a jog,
# a tail), STATIC copper -- foreign pads, vias and segments, and the
# other members' teeth and stubs -- forces a sample's layer, and a via
# slot needs room from all of it on both layers. One MILP over region
# and tail together; its result is the run's required stretches, the
# leg's layer and splits, the jog's layer, the page at s1, and the via
# sites reserved for the lanes still unrouted. Legs are still PLACED by
# `_leg_s`; only their layers move into the schedule.
TAIL_MAXCH = int(os.environ.get('BRAID_TAIL_MAXCH', str(MAXCH + 2)))  # a tail adds a corner and an end via
ISLAND_W = float(os.environ.get('BRAID_ISLAND_W', '0.5'))   # level 5: a mid stretch left on the layer static copper walls (two islands under a via pair: the dive only when owed)
L5_TIME = float(os.environ.get('BRAID_L5_TIME', '30'))       # level 5: the solver's time SAFETY (s); the budget is L5_NODES
L5_GAP = float(os.environ.get('BRAID_L5_GAP', '0.01'))       # level 5: relative MIP gap the solver stops at
SLOT_REACH = float(os.environ.get('BRAID_SLOT_REACH', '0.6')) # level 5: a via slot is kept this close (along the lane) to an event
SLOT_BG = float(os.environ.get('BRAID_SLOT_BG', '1.0'))      # level 5: ...and one every this far regardless
# Level 5's solve time (2026-09-11, late). The HiGHS log on a dumped K41
# instance said where the 20-27 s went: the incumbent within 2.5% of the
# optimum was found at 2 s, and the rest was PROVING the bound -- 110k of
# 138k LP iterations in strong branching (the LP relaxation is weak on
# the residue binaries, which take fractional values at 6 each); with the
# residue binaries FIXED to the answer the same instance solves in 0.05 s.
# So: (1) reliability branching off (`mip_pscost_minreliable` 0: 20 -> 9 s
# for the identical optimum), and (2) a two-stage warm start -- the
# residue set of the previous solve on the same members (the previous
# attempt in the braid, the previous pass in the fanout loop's judge), or
# level 4's region residue when there is none, is fixed and solved in
# milliseconds, and that feasible incumbent seeds the full solve. A
# solution mapped slot by slot from the previous attempt was tried first
# and is INFEASIBLE in the new instance (6-7 violated rows), so HiGHS
# discarded it; the residue SET is what carries over. Measured on the
# dumps: the seeded full solve holds the optimum from its first second
# and the time limit only caps the proof (K41 attempts 1-2: 106.12 and
# 102.61 at 3 s, the 20-27 s optima). Needs scipy's bundled HiGHS
# (`scipy.optimize._highspy._core._Highs`, a private API); without it the
# plain `milp` cold solve runs as before.
L5_PSCOST = int(os.environ.get('BRAID_L5_PSCOST', '0'))     # HiGHS mip_pscost_minreliable (-1: the solver's default)
# The JUDGE's cap (plan_braid -> run(plan_only=True): the fanout loop's
# judge and the residue search, ~12 s a trial at K41 with the braid's
# cap): the seeded incumbent is the answer in nearly every solve, so a
# plan being RANKED gets a shorter proof than the plan being LAID.
L5_JUDGE_TIME = float(os.environ.get('BRAID_L5_JUDGE_TIME', '10'))
# The solve's budget is a NODE count, not a clock (2026-09-11, late): a
# wall-time cap ships whichever incumbent the clock catches, so the same
# board braided under a different machine load laid different copper
# (K41: 78 or 94 vias on one fanout board). On the K41 dumps a seeded
# solve under `mip_max_nodes` returns the identical solution on every
# repeat and reaches the 20 s optimum by 100 nodes (5-7 s under load);
# the time limits above are the safety net only.
L5_NODES = int(os.environ.get('BRAID_L5_NODES', '100'))         # the braid's solves
L5_JUDGE_NODES = int(os.environ.get('BRAID_L5_JUDGE_NODES', '30'))   # a plan being ranked
L4_JUDGE_NODES = int(os.environ.get('BRAID_L4_JUDGE_NODES', '5'))    # level 4 as a seed inside a judge call
# The QUICK JUDGE (2026-09-11, late): a plan being RANKED gets one small
# solve -- the previous solve's scheduled lanes stay scheduled (w fixed
# at 0) and only its residue lanes are free -- so a trial is rewarded
# for the one thing the residue search looks for (a residue lane that
# now fits) and reported infeasible when it breaks a scheduled lane
# (then the full seeded solve runs). The braid's own solves are untouched.
L5_JUDGE_QUICK = int(os.environ.get('BRAID_L5_JUDGE_QUICK', '0') or 0)   # OFF: see the README (c7)
_HIGHS_THREADS_SET = [False]
ECON_LONG = float(os.environ.get('BRAID_ECON_LONG', '3.0'))   # econ re-lay: a lane this far (mm) over its airline is a candidate
# The HEAD scheduled like the tail (2026-09-11, late): a joiner's jog and
# join leg are pieces of level 5's polyline, so the schedule starts at
# the TOOTH -- proximity between two join legs (Euclidean, as any pair
# with a leg in it), another member's tooth stub a hard wall, a change
# anywhere along the head at a via slot -- and its answer sets the head
# leg's and jog's layers (`head_prof`, `hjog_prof`: read by band_of,
# virtual_of and virtual_vias_of exactly as the tail's leg_prof and
# jog_prof), the layer the region line begins on (`prof_L0`, which
# _prof_layer and _req_from_profile start from instead of the tooth's)
# and the birth via at the launch slot (a change at the leg's end).
# Head-on launches keep their fan-in outside the model, as level 4 did.
HEAD_L5 = int(os.environ.get('BRAID_HEAD_L5', '0') or 0)
L5_SEED = int(os.environ.get('BRAID_L5_SEED', '1') or 0)    # 0: cold solves as before
# The BERTH CHOICE inside the solve (2026-09-11, latest; _alts5): a plan
# being ranked may offer candidate berths for its residue nets (the
# fanout loop's residue search, DST_RESIDUE=2). Each candidate is a
# lane of its own in ONE more level-5 MILP -- its rows gated by a choice
# binary, the other lanes' lines held -- so a pass of the search is one
# solve instead of a trial per candidate (K41: 40-65 trials of ~3 s a
# pass). The braid's own solves are untouched; the answer is advice.
# TWO STAGES (measured on the K41 pass-0 instance, 2026-09-11): the full
# choice instance found its best solution once at 36 s and not at all
# in 64 s of another run (the LP relaxation is weak on the gating
# binaries, and the improvement is hidden from the root: dual bound 76
# against the 106 answer). Stage A fixes every LAID lane's schedule to
# level 5's own solution -- the residue nets choose their berths and
# their own schedules against neighbours that stand still -- and solves
# to optimality at the root node in 0.2 s, at the very value the 300 s
# joint solve reached. Stage B is the full instance seeded with it, a
# NODE budget (deterministic), so the neighbours may adapt -- OFF by
# default: on K35/K41 pass 0 it never improved stage A's berths where
# the full judge agreed, and where it did propose more (sweeps 1-2 at
# K41, 7-21 s each) the judge refused every one.
L5_ALT_NODES = int(os.environ.get('BRAID_L5_ALT_NODES', '0'))     # stage B's node budget (0: stage A alone)
# Stage A PER NET (2026-09-11, latest): with every net offered candidates
# (DST_RESIDUE=3) stage A holds nothing and ran 80-100 s to its node
# limit at K35. Holding everything but ONE net's candidates is exact in
# well under a second; a sweep over the nets in residue-first order,
# each net's answer kept as it is found (so the next net chooses against
# it), then the joint stage A seeded with the sweep's answer, which it
# can only improve. 0: the joint stage A alone, as before.
L5_ALT_PERNET = int(os.environ.get('BRAID_L5_ALT_PERNET', '1') or 0)
L5_ALT_TIME = float(os.environ.get('BRAID_L5_ALT_TIME', '30'))     # each stage's time safety (s)
# a CANDIDATE lane's via slots are a screen, not the plan (the berth it
# wins is re-judged with the full slot set): every third sample with
# room near an event, a background slot every 2 mm
L5_ALT_SLOT_BG = float(os.environ.get('BRAID_L5_ALT_SLOT_BG', '2.0'))
L5_ALT_SLOT_STEP = int(os.environ.get('BRAID_L5_ALT_SLOT_STEP', '3'))
_L5_SEED = {}                                               # frozenset(members) -> the last level-5 residue set


# BRAID_SOLVER (2026-09-11, latest): 'highs' (scipy's bundled HiGHS, the
# default) or 'cpsat' (OR-Tools CP-SAT, `pip install ortools`). Measured
# on the dumped instances by the solver study (scratch solver_study/
# REPORT.md): the LP relaxation of these 0/1 schedules is worthless (k41:
# 47.75 against the integer optimum 102.62, 71% of the states fractional
# -- every state at 1/2 satisfies the must-differ rows for free), so an
# LP-based branch-and-bound never closes the gap on a choice instance
# (HiGHS: dual 72 vs primal 106 after 300 s), while clause learning plus
# an LNS portfolio PROVES optimality: alt41c 8.8 s (13 residue nets for
# HiGHS's 15), alt35 8.3 s, k41 0.33 s; a third to a half of the memory.
# Determinism comes from the solver (interleave_search + a deterministic
# time budget gave bit-identical vectors across runs), not a node cap.
# HiGHS stays the fallback when ortools is not importable.
SOLVER = os.environ.get('BRAID_SOLVER', 'highs')
# ...and the CHOICE solve's own backend (_alts5), since the two answer
# different questions: the braid's plain solves are node-capped HiGHS
# incumbents that the ROUTE happens to like (K41 chain: HiGHS 84 vias,
# every plain solve OPTIMAL under CP-SAT 98 -- the model's optimum is not
# the route's), while the choice solve wants the proven optimum of a
# gated instance HiGHS never closes. Default: the same as BRAID_SOLVER.
ALT_SOLVER = os.environ.get('BRAID_ALT_SOLVER', SOLVER)
CPSAT_WORKERS = int(os.environ.get('BRAID_CPSAT_WORKERS', '4'))
# BRAID_CPSAT_REPAIR (2026-09-12): the alt stages always hand CP-SAT a hint
# (`x0`), but a CHOICE instance's hint is the PLAIN solution extended, which
# need not satisfy the choice rows -- the stage-A log has a "seed VIOLATES n
# rows" arm for exactly that. CP-SAT drops an infeasible hint after
# `hint_conflict_limit` (10) conflicts; `repair_hint` instead asks it to
# repair the hint into a feasible start, which is the whole value of having
# an incumbent on an instance whose LP relaxation is worthless.
CPSAT_REPAIR = int(os.environ.get('BRAID_CPSAT_REPAIR', '0') or 0)
CPSAT_DET = float(os.environ.get('BRAID_CPSAT_DET', '0') or 0)   # deterministic-time budget (0: wall clock only)
CPSAT_SCALE = 10000                                              # objective coefficients as integers


def _cpsat_solve(cvec, rows, lb, ub, integ, lo, hi, time_limit, gap, x0=None, det=None, workers=None):
    """CP-SAT on the same instance as _milp_solve (every variable 0/1 --
    the continuous states are implied integral by the chain rows).
    Returns (x, message, feasible); the objective the caller reads is
    recomputed from the float cvec."""
    from ortools.sat.python import cp_model
    nv = len(cvec)
    m = cp_model.CpModel()
    x = []
    for i in range(nv):
        l_, h_ = int(round(float(lo[i]))), int(round(float(hi[i])))
        if l_ == h_:
            x.append(m.NewIntVar(l_, l_, f'x{i}'))
        else:
            x.append(m.NewBoolVar(f'x{i}'))
    for i, co in enumerate(rows):
        e = sum(int(round(v)) * x[k] for k, v in co.items())
        lo_r, hi_r = float(lb[i]), float(ub[i])
        if math.isfinite(lo_r) and math.isfinite(hi_r) and lo_r == hi_r:
            m.Add(e == int(round(lo_r)))
        else:
            if math.isfinite(hi_r):
                m.Add(e <= int(math.floor(hi_r + 1e-9)))
            if math.isfinite(lo_r):
                m.Add(e >= int(math.ceil(lo_r - 1e-9)))
    ci = np.rint(np.asarray(cvec, float) * CPSAT_SCALE).astype(np.int64)
    m.Minimize(sum(int(ci[i]) * x[i] for i in np.nonzero(ci)[0]))
    if x0 is not None:
        for i in range(nv):
            m.AddHint(x[i], int(round(float(x0[i]))))
    sv = cp_model.CpSolver()
    sv.parameters.max_time_in_seconds = float(time_limit)
    sv.parameters.num_workers = int(workers or CPSAT_WORKERS)
    sv.parameters.relative_gap_limit = float(gap)
    sv.parameters.log_search_progress = False
    if x0 is not None and CPSAT_REPAIR:
        sv.parameters.repair_hint = True
        sv.parameters.hint_conflict_limit = int(CPSAT_REPAIR)
    det = CPSAT_DET if det is None else det
    if det and det > 0:
        sv.parameters.interleave_search = True
        sv.parameters.max_deterministic_time = float(det)
    st_ = sv.Solve(m)
    if st_ not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        return None, sv.StatusName(st_), False
    return np.array([float(sv.Value(x[i])) for i in range(nv)]), sv.StatusName(st_), True


def _milp_solve(cvec, rows, lb, ub, integ, lo, hi, time_limit, gap, x0=None, pscost=None, nodes=None,
                solver=None):
    """min cvec.x s.t. lb <= A x <= ub, lo <= x <= hi, integrality `integ`;
    A from `rows` (dicts of column -> coefficient). Returns (x, message,
    feasible) -- x None when the solver found nothing. Uses scipy's
    bundled HiGHS directly when it is there (for the warm start `x0` and
    the branching option), scipy.optimize.milp otherwise; CP-SAT under
    BRAID_SOLVER=cpsat (the node budget is then unused)."""
    if (solver or SOLVER) == 'cpsat':
        try:
            return _cpsat_solve(cvec, rows, lb, ub, integ, lo, hi, time_limit, gap, x0=x0)
        except ImportError:
            pass          # ortools not installed: HiGHS as before
    from scipy.sparse import coo_matrix
    nv = len(cvec)
    # the matrix from coordinate arrays: a lil_matrix filled row by row
    # took a gigabyte and minutes on the joint-order instance (100k rows)
    ri, ci, vi = [], [], []
    for i_, co in enumerate(rows):
        for k_, v_ in co.items():
            ri.append(i_); ci.append(k_); vi.append(float(v_))
    A = coo_matrix((np.asarray(vi, float), (np.asarray(ri, np.int64), np.asarray(ci, np.int64))),
                   shape=(max(1, len(rows)), nv))
    lb_ = np.asarray(lb if rows else [-np.inf], float)
    ub_ = np.asarray(ub if rows else [np.inf], float)
    hs_core = None
    try:
        import scipy.optimize._highspy._core as hs_core
        if not hasattr(hs_core, '_Highs') or not hasattr(hs_core._Highs, 'setSolution'):
            hs_core = None
    except Exception:
        hs_core = None
    if hs_core is None:
        from scipy.optimize import milp, LinearConstraint, Bounds
        res = milp(cvec, constraints=LinearConstraint(A.tocsr(), lb_, ub_), integrality=integ,
                   bounds=Bounds(lo, hi), options={'time_limit': time_limit, 'mip_rel_gap': gap})
        return res.x, res.message, res.x is not None
    Ac = A.tocsc()
    lp = hs_core.HighsLp()
    lp.num_col_ = nv
    lp.num_row_ = Ac.shape[0]
    lp.a_matrix_.num_col_ = nv
    lp.a_matrix_.num_row_ = Ac.shape[0]
    lp.a_matrix_.format_ = hs_core.MatrixFormat.kColwise
    lp.col_cost_ = np.asarray(cvec, float)
    lp.col_lower_ = np.asarray(lo, float)
    lp.col_upper_ = np.asarray(hi, float)
    lp.row_lower_ = lb_
    lp.row_upper_ = ub_
    lp.a_matrix_.start_ = Ac.indptr.astype(np.int32)
    lp.a_matrix_.index_ = Ac.indices.astype(np.int32)
    lp.a_matrix_.value_ = Ac.data.astype(float)
    lp.integrality_ = [hs_core.HighsVarType(int(i)) for i in integ]
    hs = hs_core._Highs()
    hs.setOptionValue('output_flag', False)
    if not _HIGHS_THREADS_SET[0]:
        # one thread per solve (the judge's worker processes run six at
        # once); accepted only before this process's first solve
        hs.setOptionValue('threads', 1)
        _HIGHS_THREADS_SET[0] = True
    hs.setOptionValue('time_limit', float(time_limit))
    hs.setOptionValue('mip_rel_gap', float(gap))
    if pscost is not None and pscost >= 0:
        hs.setOptionValue('mip_pscost_minreliable', int(pscost))
    if nodes is not None and nodes > 0:
        hs.setOptionValue('mip_max_nodes', int(nodes))
    hs.passModel(lp)
    if x0 is not None:
        sol = hs_core.HighsSolution()
        sol.col_value = [float(v) for v in x0]
        sol.value_valid = True
        hs.setSolution(sol)
    hs.run()
    info = hs.getInfo()
    feasible = int(info.primal_solution_status) == int(hs_core.kSolutionStatusFeasible)
    msg = hs.modelStatusToString(hs.getModelStatus())
    if not feasible:
        return None, msg, False
    return np.array(hs.getSolution().col_value), msg, True
# ^ a SWIMMER's planned line is HOLD-THEN-RUN instead of the straight
# ribbon diagonal (#622, 2026-09-10 night): hold the launch offset to c1,
# run to the target offset by c2, hold it to s1, with (c1, c2) chosen so
# the layer the page lanes force on it at each crossing (the opposite of
# the crossed lane's page) changes as few times as possible, and never
# twice within a via's room along s. Measured on the human bench K41: the
# straight lines of five swimmers imply 119 (changes + 10 per via-room
# conflict), the polylines 29 -- four of the five become ONE clean change,
# the human's F-then-B lane with one dive in the corridor. The band
# (SWIM_TUBE) and the reserved diamonds (hops) follow the planned line, so
# the router is guided to the same homotopy. 0: off, byte-identical.
SWIM_ROOM = 0.45               # two layer changes need this much s between the crossings they serve
BLOCK_PUSH = int(os.environ.get('BRAID_BLOCK_PUSH', '1') or 0)
TAIL_PAGE = int(os.environ.get('BRAID_TAIL_PAGE', '0') or 0)
# ^ 1: for a TAIL island, a block lane is "on a layer" by its PAGE up to its
# own exit leg (allowed() read just before the leg's stretch), not by what
# the plan allows in the 0.35 mm before the leg where the page requirement
# has already ended. Read there, a B-page lane whose leg lay inside the
# passive cluster north of DU1 counted as an F lane, was bent outward round
# the whole cluster, and its leg then crossed every inner lane (human bench
# K41 SA4: refused in-band in every attempt; the human runs it under the
# passives on B, as the page says).
# ^ 2: pushed only when the push CLEARS within its cap (a push that finds
# copper at every step is no push -- see _clear_block); 1: pushed to the
# cap regardless (the recorded chain's rule).
# ^ 0: a side block is NOT pushed whole clear of static copper (_clear_block);
# it starts a BLOCK_GAP beyond the stubs and each lane is bent round what its
# own run meets (deflect_islands). The human bench's north block: pushed 1.5 mm
# clear of the passive cluster north of DU1, ours sat 2 mm further out than
# the human's lanes (SDQ10 -7.9 against -5.5), every north diagonal steeper.
SWIM_SLOPE = 3.0               # the run's steepest |do/ds|
EXIT_GUARD = int(os.environ.get('BRAID_EXIT_GUARD', '0') or 0)
# ^ 1: a stub is not head-on when another member's stub on the SAME layer
# stands upstream of it within END_CLASH in o -- however little upstream.
# head_exit's upstream test exempts the half-millimetre TOL_S, copied from
# the launch side where teeth at one s fan out; at an EXIT the tail of the
# downstream stub runs at (about) its own offset over the upstream stub's
# end. Measured on the human bench (end_clash_probe.py): SA2's tail 0.06 mm
# from SA4's end (0.48 mm apart along DU1's north face, both B), SCKE0's
# 0.04 mm from SCKE1's (0.35 apart, both B), SDQ1's 0.09 from SDQ4's -- the
# nets at 4 vias instead of 2 in every arm at K41 and K51. Such a stub
# becomes a side exit: a block lane with a perpendicular leg, the comb the
# human lays along the face. Unlike BRAID_FLANK_COMB it moves ONLY the
# stubs in conflict (a stub whose neighbour is on the other layer stays
# head-on: two ends at one point on two layers are the fanout's doing).
END_CLASH = LPITCH             # a tail this close to a foreign end on its layer sweeps it
JOG_VIA = float(os.environ.get('BRAID_JOG_VIA', '1.0') or 1.0)
# BRAID_B_OUTER=1 (2026-09-11, README TODO 20): in an exit block the lanes on
# the BACK page take the outer slots, the front page's the inner ones, each
# group in its existing order. A front lane's exit leg runs on B (the
# block's majority layer) inward to its stub, crossing every lane between
# its slot and the face; a back-page lane in that span is forced up to F
# and down again under the leg -- two vias the human never pays: the
# human's one back lane in K41's south channel (SCAS) runs OUTSIDE every
# front lane's dive point. Measured forced lanes: our K28 SA8 SA1 SBA1,
# K35 SBA1 SA15, K41 SCKE1 SA8 SBA2; the human bench SCAS (72 for 70).
# Front x back crossings the re-order adds are free (different pages).
B_OUTER = int(os.environ.get('BRAID_B_OUTER', '0'))
# ^ what a PITCH of leg jog along the stub row costs against a via, when a
# leg's own layer is islanded and the move competes with the flip
# (place_and_decide's move_cost). 1.0 is the recorded rule. The human
# bench's SCAS: its B leg lands on C6 (a B-side passive); the flip to F
# costs 2 vias (page B, stub B) and the 1.4 mm move 4 "vias" at 1.0, so
# the leg flipped and SCAS shipped at 4 vias in every arm at K41 and K51
# -- the human slides the leg 1.4 mm along the face and jogs back on B,
# at 2. At 0.25 the move wins.
SEC_CAP = float(os.environ.get('BRAID_SEC_CAP', '1e9') or 1e9)
# ^ DIAGNOSTIC (2026-09-10): a cap on the secant SLOPE_PITCH widens a
# same-page pair's pitch by (pair_floor). The widening feeds itself: a
# wider exit block makes its outer lanes steeper, the next pass widens it
# more; a region 0.95 mm shorter (the face legs, or BRAID_S0_PLUS alone)
# moved K51's south block from +12.9 to +15.6 and every joiner refused.
# At 1.5 (slope 1.1) a 0.35 pitch keeps 0.23 mm across a slope-2 pair.
FACE_SORT = int(os.environ.get('BRAID_FACE_SORT', '0') or 0)
FACE_EARLY = int(os.environ.get('BRAID_FACE_EARLY', '0') or 0)   # diagnostic, see _face_comb
JOIN_FLOOR = int(os.environ.get('BRAID_JOIN_FLOOR', '0') or 0)   # diagnostic, see offsets()
# ^ 1: LAUNCH ORDER = TARGET ORDER on each side's other-layer comb, made
# so at the SOURCE FACE. The lanes born on the layer the source fanout
# leaves free (B under an F fanout) and bound for a side of the
# destination form, per side, a launch comb ordered by their target rank;
# a lane whose tooth stands inside the front bundle (a B tooth at U1's
# south face bound for DU1's north block) reaches its slot by a LEG along
# the face on its own layer -- under the front lanes, which leave the face
# on the other layer -- nested a pitch apart in s, the outermost slot's
# leg first. Read off the human's copper on the bench (family_probe /
# poly_probe, 2026-09-10): SA0, SA2, SA4 drop 4-5 mm along U1's face on B
# at s 8.2-9.3, land just inside SA1's tooth, and run parallel diagonals
# to the north block; the front-to-back swimmers then dive at s 10-12.6
# where the back layer holds no crossing diagonal, and run parallel to
# the west-face stubs. Our ribbon put those diagonals INSIDE the region,
# so the swimmers' back-layer runs crossed them (SDQ9 planned at 5
# changes, routed at 4 vias; the human 2). Off: byte-identical.
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
            cnt[ka] += 1
            cnt[kb] += 1
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

        guarded = {}
        self.guarded = guarded

        def head_exit(nm):
            return self._head_exit(nm, se[nm], self.stubs[nm], ctx.dest_layer[nm],
                                   self._band_line_of(nm))

        self.heads_l = [nm for nm in M if head_launch(nm)]
        self.joiners = [nm for nm in M if nm not in self.heads_l]
        self.flank = self._flank_stubs() if FLANK_COMB else {}
        self.heads_e = [nm for nm in M if nm not in self.flank and head_exit(nm)]
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
        if self.siders:
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

    def _band_line_at(self, xy, ref):
        """(band, side) of a stub at `xy` in a band of array `ref`: the
        line it stands on; None off any band."""
        bands = getattr(self.ctx, 'dest_bands', {}).get(ref, ())
        x, y = xy
        for b in bands:
            x0, y0, x1, y1 = b
            if x1 - x0 >= y1 - y0:
                if y0 < y < y1 and x0 - 0.5 <= x <= x1 + 0.5:
                    return (b, y < (y0 + y1) / 2)
            elif x0 < x < x1 and y0 - 0.5 <= y <= y1 + 0.5:
                return (b, x < (x0 + x1) / 2)
        return None

    def _band_line_of(self, nm):
        """(band, side) of a band stub: the line it stands on."""
        return self._band_line_at(self.ctx.ends[nm][1], self.ctx.ends[nm][2])

    def _side_of(self, o_pt, ref):
        """Which side of the spine an offset lies on, seen from array
        `ref`'s centre: +1 at or beyond it, -1 short of it."""
        ps = self.ctx.pcb.footprints[ref].pads
        c = (sum(p.global_x for p in ps) / len(ps),
             sum(p.global_y for p in ps) / len(ps))
        _sc, oc = self.spine.project_pt(c)
        return 1 if o_pt >= oc else -1

    def _head_exit(self, nm, se_nm, stub_nm, dl_nm, bl, record=True):
        """Is a stub of `nm` at (s, o) `se_nm` (board point `stub_nm`, on
        `dl_nm`, on band line `bl`) a HEAD-ON exit -- reached by a straight
        run in at its own offset -- given the other members' stubs as they
        stand? classify asks for every member; _alt_pieces asks for a
        CANDIDATE berth (record=False: nothing noted in `guarded`)."""
        ctx, sp, M = self.ctx, self.spine, self.members
        se = self.se
        s_i, o_i = se_nm
        for om in M:
            if om == nm:
                continue
            s_j, o_j = se[om]
            if s_j < s_i - TOL_S and abs(o_j - o_i) < DIST_O:
                return False
            if (EXIT_GUARD and s_j < s_i - 0.05 and abs(o_j - o_i) < END_CLASH
                    and ctx.dest_layer[om] == dl_nm):
                if record:
                    self.guarded.setdefault(nm, []).append(om)
                return False
            # two stubs on one BAND LINE are at one offset whatever
            # the spine's tilt says: a head-on run at the second's
            # own offset runs over the first's stub (SCKE1/SCKE0, a
            # corridor of two, both refused in-band every attempt)
            if bl is not None and s_j < s_i - TOL_S and self._band_line_of(om) == bl:
                return False
        if bl is not None:
            # ...and every OTHER net's stub on that line is static
            # copper the head-on run at the tip line would cross (the
            # pad-field test below sees pads only): the first stub on
            # a band line is head-on only when the run-in on its own
            # layer is clear of them
            run_from = sp.xy(s_i - HEAD_RUN, o_i)
            if not ctx.obs_but(nm, M, dl_nm).seg_clear(run_from, stub_nm):
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
        of om's layer there -- its page, or, for a scheduled diver
        (ONE_DIVE), its tooth layer before the change and its berth
        layer after."""
        L = self._prof_layer(om, s)
        if L is None:
            L = sched.page.get(om)
        return other_layer(L) if L else None

    def one_dive(self, sched, log=None):
        """ONE_DIVE: rewrite the schedule's pages by the human's rule.
        Stayers (tooth layer == berth layer) keep that layer over the
        whole region; changers change once at a point s in the region;
        each inverted pair's crossing is FIXED where their ribbon lines
        meet (launch slot at s0 -- a joiner's leg -- to target slot at
        s1), and the change points are chosen so every crossing has its
        two lanes on different layers, DIVE_ROOM clear of the change:
        a stayer on L crossed by a changer whose tooth is on L -> the
        changer changes before it; whose berth is on L -> after it; two
        changers of one class cross between their changes, of opposite
        classes before both or after both; two stayers on one layer
        cannot cross at all. The fewest nets are dropped (a MILP over
        the change points, scipy HiGHS); a dropped net swims as before.
        Sets self.dive {net: s_change} and sched.page (a diver's page
        is its BERTH layer: the layer it holds from the change on, which
        is what the exit legs and the tail read), and re-derives the
        schedule's sets."""
        self.dive = {}
        self.ride = {}
        self.prof = {}
        self.slot_via = {}
        if not ONE_DIVE or sched is None:
            return
        import itertools
        try:
            from scipy.optimize import milp, LinearConstraint, Bounds
            from scipy.sparse import lil_matrix
        except Exception as e:
            self.log(f'  one dive: scipy unavailable ({e}); LIS pages kept')
            return
        M = self.members
        tl, dl = self.ctx.tooth_layer, self.ctx.dest_layer
        trank = {nm: i for i, nm in enumerate(self.target)}
        py = self.py
        line = {}
        for nm in M:
            s_a = (self.join_leg_s[nm] if nm in getattr(self, 'join_block', {})
                   else max(self.s0, self.st[nm][0]))
            line[nm] = (s_a, self.launch_o[nm], self.s1, py[trank[nm]])

        def o_at(ln, s):
            s_a, o_a, s_b, o_b = ln
            return o_a + (s - s_a) / max(s_b - s_a, 1e-9) * (o_b - o_a)

        def cross_s(a_, b_):
            lo_s = max(line[a_][0], line[b_][0])
            hi_s = min(line[a_][2], line[b_][2])
            if hi_s <= lo_s:
                return None
            d0 = o_at(line[a_], lo_s) - o_at(line[b_], lo_s)
            d1 = o_at(line[a_], hi_s) - o_at(line[b_], hi_s)
            if d0 == d1:
                return None
            x = lo_s + (hi_s - lo_s) * d0 / (d0 - d1)
            return x if lo_s <= x <= hi_s else None
        pairs = []
        for a_, b_ in itertools.combinations(M, 2):
            if not sched.inverted(a_, b_):
                continue
            x = cross_s(a_, b_)
            if x is not None:
                pairs.append((a_, b_, x))
        changers = [nm for nm in M if tl[nm] != dl[nm]]
        r = DIVE_ROOM
        if ONE_DIVE >= 4:
            self._profiles4(sched, M, pairs, line, tl, dl, log)
            return
        if ONE_DIVE >= 3:
            self._profiles3(sched, M, pairs, line, tl, dl, log)
            return
        if ONE_DIVE >= 2:
            self._profiles(sched, M, pairs, line, tl, dl, r, log)
            return
        BIGM = (self.s1 - self.s0) + 4.0
        idx = {}
        for nm in changers:
            idx[('s', nm)] = len(idx)
        for nm in M:
            idx[('w', nm)] = len(idx)
        rows, lb, ub = [], [], []

        def add(co, lo, hi):
            rows.append(dict(co)); lb.append(lo); ub.append(hi)

        def newy():
            k = ('y', len(rows))
            idx[k] = len(idx)
            return idx[k]
        hard = []
        for a_, b_, t in pairs:
            wa, wb = idx[('w', a_)], idx[('w', b_)]
            ca, cb = ('s', a_) in idx, ('s', b_) in idx
            if not ca and not cb:
                if tl[a_] == tl[b_]:
                    hard.append((a_, b_))
                    add({wa: 1, wb: 1}, 1, np.inf)
                continue
            if ca != cb:
                st_, ch = (a_, b_) if cb else (b_, a_)
                sc = idx[('s', ch)]
                if tl[st_] == dl[ch]:
                    # the changer must still be on its tooth layer at the crossing
                    add({sc: 1, wa: BIGM, wb: BIGM}, t + r, np.inf)
                else:
                    add({sc: 1, wa: -BIGM, wb: -BIGM}, -np.inf, t - r)
                continue
            sa, sb = idx[('s', a_)], idx[('s', b_)]
            y = newy()
            if tl[a_] == tl[b_]:
                # one class: cross between the two changes (either order)
                add({sa: 1, y: BIGM, wa: -BIGM, wb: -BIGM}, -np.inf, t - r + BIGM)
                add({sb: 1, y: -BIGM, wa: BIGM, wb: BIGM}, t + r - BIGM, np.inf)
                add({sb: 1, y: -BIGM, wa: -BIGM, wb: -BIGM}, -np.inf, t - r)
                add({sa: 1, y: BIGM, wa: BIGM, wb: BIGM}, t + r, np.inf)
            else:
                # opposite classes: before both changes (y=1: both >= t+r),
                # or after both (y=0: both <= t-r). The first cut had the
                # y=0 rows trivially true and the y=1 rows contradictory,
                # so these pairs went unconstrained: K35 self-check 3-12
                # illegal crossings (2026-09-11)
                add({sa: 1, y: -BIGM, wa: BIGM, wb: BIGM}, t + r - BIGM, np.inf)
                add({sb: 1, y: -BIGM, wa: BIGM, wb: BIGM}, t + r - BIGM, np.inf)
                add({sa: 1, y: -BIGM, wa: -BIGM, wb: -BIGM}, -np.inf, t - r)
                add({sb: 1, y: -BIGM, wa: -BIGM, wb: -BIGM}, -np.inf, t - r)
        nv = len(idx)
        lo_b, hi_b = np.zeros(nv), np.ones(nv)
        for nm in changers:
            k = idx[('s', nm)]
            lo_b[k] = line[nm][0] + r
            hi_b[k] = self.s1 - r
            if hi_b[k] < lo_b[k]:
                hi_b[k] = lo_b[k]
        cvec = np.zeros(nv)
        integ = np.zeros(nv)
        for k, v in idx.items():
            if k[0] == 'w':
                cvec[v] = 1.0
                integ[v] = 1
            elif k[0] == 'y':
                integ[v] = 1
        if rows:
            A = lil_matrix((len(rows), nv))
            for i_, co in enumerate(rows):
                for k, v in co.items():
                    A[i_, k] = v
            cons = LinearConstraint(A.tocsr(), lb, ub)
        else:
            cons = LinearConstraint(lil_matrix((1, nv)).tocsr(), [-np.inf], [np.inf])
        res = milp(cvec, constraints=cons, integrality=integ, bounds=Bounds(lo_b, hi_b),
                   options={'time_limit': 30})
        if res.x is None:
            self.log(f'  one dive: no solution ({res.message}); LIS pages kept')
            return
        residue = [nm for nm in M if res.x[idx[('w', nm)]] > 0.5]
        for nm in M:
            if nm in residue:
                sched.page[nm] = None
            elif ('s', nm) in idx:
                self.dive[nm] = float(res.x[idx[('s', nm)]])
                sched.page[nm] = dl[nm]
            else:
                sched.page[nm] = tl[nm]
        keep = {nm for nm in sched.launch if sched.page[nm] == 'F.Cu'}
        sched.b_page = [nm for nm in sched.launch if sched.page[nm] == 'B.Cu']
        sched.swimmers = [nm for nm in sched.launch if sched.page[nm] is None]
        sched.divers = {nm for nm in sched.launch if nm not in keep}
        sched.birth_b = {d for d in sched.divers if tl.get(d) == 'B.Cu'}
        stay = [nm for nm in M if nm not in residue and nm not in self.dive]
        # SELF-CHECK: every crossing between two kept lanes must be on
        # different layers, DIVE_ROOM clear of a change -- a wrong sign
        # in a constraint above would otherwise ship as a short

        def layer_at(nm, s_):
            c = self.dive.get(nm)
            return tl[nm] if (c is None or s_ < c) else dl[nm]
        bad = []
        for a_, b_, t in pairs:
            if a_ in residue or b_ in residue:
                continue
            if layer_at(a_, t) == layer_at(b_, t):
                bad.append((a_, b_, round(t, 2)))
            for nm in (a_, b_):
                c = self.dive.get(nm)
                if c is not None and abs(c - t) < r - 1e-6:
                    bad.append((nm, 'change within room of crossing', round(t, 2)))
        if bad:
            (log or self.log)(f'  one dive: SELF-CHECK FAILED, {len(bad)} illegal crossing(s): {bad[:6]}')
        (log or self.log)(
            f'  one dive: {len(stay)} stayers, {len(self.dive)} divers, '
            f'{len(residue)} residue {residue} ({len(pairs)} crossings, '
            f'{len(hard)} stayer-stayer same-layer); changes: '
            + ' '.join(f'{nm}@{c:.1f}' for nm, c in sorted(self.dive.items(), key=lambda kv: kv[1])))

    def _prof_changes(self, om):
        """The lane's scheduled change points in s order, from whichever
        scheme set them: a diver's one, a rider's two, or level 4's list."""
        pf = getattr(self, 'prof', {}).get(om)
        if pf is not None:
            return list(pf)
        c = getattr(self, 'dive', {}).get(om)
        if c is not None:
            return [c]
        rd = getattr(self, 'ride', {}).get(om)
        if rd is not None:
            return list(rd)
        return None

    def _prof_layer(self, om, s):
        """The scheduled lane's layer at s: its tooth layer, flipped at
        each change; None for a lane with no schedule of its own."""
        ch = self._prof_changes(om)
        if ch is None:
            return None
        L = self._line_L0(om)
        for c in ch:
            if s >= c:
                L = other_layer(L)
        return L

    def _profiles4(self, sched, M, pairs, line, tl, dl, log=None):
        """ONE_DIVE level 4: every lane a scheduled layer profile with up
        to MAXCH changes (see the flag's note). Per lane and candidate
        slot two binaries, an UP change (tooth layer -> the other) and a
        DOWN one, and a STATE variable carrying the running sum up - down
        (so ups and downs alternate and the state is 0 or 1); it ends at 1
        for a changer and 0 for a stayer. Legality at every proximity
        sample as level 3, written on the state variables (three or four
        terms a row -- the prefix-sum form was 9-12 s a solve at K28/K35,
        this one is sparse); a lane's own changes VIA_SEP apart.
        Objective: the number of changes (vias) + RESIDUE_W per net left
        to swim."""
        from scipy.optimize import milp, LinearConstraint, Bounds
        from scipy.sparse import lil_matrix
        import time as _t
        t0 = _t.time()
        s_lo = min(line[nm][0] for nm in M)
        s_hi = self.s1
        S = np.arange(s_lo, s_hi + 1e-9, PROF_DS)
        O = {}
        for nm in M:
            s_a, o_a, s_b, o_b = line[nm]
            o = o_a + (S - s_a) / max(s_b - s_a, 1e-9) * (o_b - o_a)
            o[S < s_a - 1e-9] = np.nan
            O[nm] = o
        cand = {}
        for nm in M:
            d = np.full(S.shape, np.inf)
            for om in M:
                if om == nm:
                    continue
                dd = np.abs(O[nm] - O[om])
                dd[np.isnan(dd)] = np.inf
                d = np.minimum(d, dd)
            ok = (d >= VIA_NEED) & ~np.isnan(O[nm])
            ks = [k for k in range(len(S)) if ok[k]][::2]
            ends = [int(np.argmax(~np.isnan(O[nm]))), len(S) - 1]
            cand[nm] = sorted(set(ks) | set(ends))
        prox = []
        for i, a_ in enumerate(M):
            for b_ in M[i + 1:]:
                dd = np.abs(O[a_] - O[b_])
                for k in np.where(~np.isnan(dd) & (dd < PROX_TRACK))[0]:
                    prox.append((a_, b_, int(k)))
        memo = _PROFILE_MEMO
        mkey = ('L4', tuple((nm, tl[nm], dl[nm], tuple(round(v, 4) for v in line[nm])) for nm in M),
                round(self.s1, 4), PROX_TRACK, MAXCH, RESIDUE_W, ONE_DIVE >= 5)
        idx = {}

        def var(key):
            if key not in idx:
                idx[key] = len(idx)
            return idx[key]
        for nm in M:
            var(('w', nm))
            for k in cand[nm]:
                var(('up', nm, k)); var(('dn', nm, k)); var(('st', nm, k))
        rows, lb, ub = [], [], []

        def add(co, lo, hi):
            rows.append(dict(co)); lb.append(lo); ub.append(hi)

        def st_at(nm, k):
            """The lane's state variable in force at sample k: that of its
            last candidate at or before k (its tooth layer before any)."""
            last = None
            for kk in cand[nm]:
                if kk <= k:
                    last = kk
                else:
                    break
            return None if last is None else idx[('st', nm, last)]
        for nm in M:
            w = idx[('w', nm)]
            ks = cand[nm]
            prev = None
            for k in ks:
                st, up, dn = idx[('st', nm, k)], idx[('up', nm, k)], idx[('dn', nm, k)]
                # st_k = st_{k-1} + up_k - dn_k   (st_{-1} = 0)
                co = {st: 1, up: -1, dn: 1}
                if prev is not None:
                    co[prev] = -1
                add(co, 0, 0)
                add({up: 1, dn: 1}, -np.inf, 1)
                prev = st
            end = 1 if tl[nm] != dl[nm] else 0
            add({prev: 1, w: 4}, end, end + 8)          # the end state, unless swimming
            add({prev: 1, w: -4}, end - 8, end)
            add({**{idx[('up', nm, k)]: 1 for k in ks}, **{idx[('dn', nm, k)]: 1 for k in ks}, w: MAXCH},
                -np.inf, MAXCH)                            # at most MAXCH changes; none when swimming
            for i_, k in enumerate(ks):
                for j in ks[i_ + 1:]:
                    if S[j] - S[k] >= VIA_SEP:
                        break
                    add({idx[('up', nm, k)]: 1, idx[('dn', nm, k)]: 1,
                         idx[('up', nm, j)]: 1, idx[('dn', nm, j)]: 1}, -np.inf, 1)
        for a_, b_, k in prox:
            wa, wb = idx[('w', a_)], idx[('w', b_)]
            va, vb = st_at(a_, k), st_at(b_, k)
            ca = {va: 1} if va is not None else {}
            cb = {vb: 1} if vb is not None else {}
            if tl[a_] == tl[b_]:
                add({**ca, **cb, wa: 4, wb: 4}, 1, np.inf)      # exactly one flipped
                add({**ca, **cb, wa: -4, wb: -4}, -np.inf, 1)
            else:
                co = dict(ca)
                for v_ in cb:
                    co[v_] = co.get(v_, 0) - 1
                add({**co, wa: -4, wb: -4}, -np.inf, 0)         # both flipped or neither
                add({**{v_: -c_ for v_, c_ in co.items()}, wa: -4, wb: -4}, -np.inf, 0)
        nv = len(idx)
        cvec = np.zeros(nv)
        integ = np.zeros(nv)
        for key, v in idx.items():
            if key[0] == 'w':
                cvec[v] = RESIDUE_W; integ[v] = 1
            elif key[0] in ('up', 'dn'):
                cvec[v] = 1.0; integ[v] = 1
        if mkey in memo:
            x, msg = memo[mkey]
        else:
            A = lil_matrix((max(1, len(rows)), nv))
            for i_, co in enumerate(rows):
                for k_, v_ in co.items():
                    A[i_, k_] = v_
            cons = (LinearConstraint(A.tocsr(), lb, ub) if rows
                    else LinearConstraint(A.tocsr(), [-np.inf], [np.inf]))
            if ONE_DIVE >= 5:
                # under level 5 this solution is only the SEED of the real
                # one (and the initial plan geometry): the K41 profile had
                # six of these at ~6 s each, 37 s of a 149 s braid, so it
                # gets the seed's economics -- reliability branching off,
                # a 3 s cap, the incumbent shipped
                judge_ = getattr(self.ctx, 'judge', False)
                x, msg, ok = _milp_solve(cvec, rows, lb, ub, integ, np.zeros(nv), np.ones(nv),
                                         L5_JUDGE_TIME, L5_GAP, pscost=L5_PSCOST,
                                         nodes=L4_JUDGE_NODES if judge_ else L5_JUDGE_NODES)
                if not ok:
                    (log or self.log)(f'  profiles4: no solution ({msg}); LIS pages kept')
                    return
            else:
                res = milp(cvec, constraints=cons, integrality=integ, bounds=Bounds(0, 1),
                           options={'time_limit': 60})
                if res.x is None:
                    (log or self.log)(f'  profiles4: no solution ({res.message}); LIS pages kept')
                    return
                x, msg = res.x, res.message
            memo[mkey] = (x, msg)
        residue = [nm for nm in M if x[idx[('w', nm)]] > 0.5]
        self.prof = {}
        page_ride = []
        s_end = float(S[-1])
        for nm in M:
            if nm in residue:
                sched.page[nm] = None
                continue
            ch = sorted(float(S[k]) for k in cand[nm]
                        if x[idx[('up', nm, k)]] > 0.5 or x[idx[('dn', nm, k)]] > 0.5)
            if not ch:
                sched.page[nm] = tl[nm]
                continue
            if (tl[nm] == dl[nm] and len(ch) == 2 and ch[0] <= line[nm][0] + PROF_DS
                    and ch[1] >= s_end - PROF_DS):
                page_ride.append(nm)                  # the page scheme's own lane
                sched.page[nm] = other_layer(tl[nm])
                continue
            self.prof[nm] = ch
            if tl[nm] != dl[nm] and len(ch) == 1:
                self.dive[nm] = ch[0]
            elif tl[nm] == dl[nm] and len(ch) == 2:
                self.ride[nm] = (ch[0], ch[1])
            sched.page[nm] = self._prof_layer(nm, s_end + 1e-6)
            # a change at the region's start / end is a via at the slot:
            # the pitch model (pair_floor) gives it a via's room
            self.slot_via[nm] = (ch[0] <= line[nm][0] + PROF_DS, ch[-1] >= s_end - PROF_DS)
        keep = {nm for nm in sched.launch if sched.page[nm] == 'F.Cu'}
        sched.b_page = [nm for nm in sched.launch if sched.page[nm] == 'B.Cu']
        sched.swimmers = [nm for nm in sched.launch if sched.page[nm] is None]
        sched.divers = {nm for nm in sched.launch if nm not in keep}
        sched.birth_b = {d for d in sched.divers if tl.get(d) == 'B.Cu'}
        self._od_pairs, self._od_residue, self._od_page_ride = list(pairs), list(residue), list(page_ride)

        def layer_at(nm, s_):
            if nm in page_ride:
                return other_layer(tl[nm])
            L = self._prof_layer(nm, s_)
            return L if L is not None else tl[nm]
        bad = []
        for a_, b_, k in prox:
            if a_ in residue or b_ in residue:
                continue
            if layer_at(a_, float(S[k])) == layer_at(b_, float(S[k])):
                bad.append((a_, b_, round(float(S[k]), 2)))
        if bad:
            (log or self.log)(f'  profiles4: SELF-CHECK FAILED, {len(bad)} same-layer proximity sample(s): {bad[:6]}')
        n_ch = sum(len(v) for v in self.prof.values()) + 2 * len(page_ride)
        hist = {}
        for nm in M:
            k_ = 'swim' if nm in residue else str(2 if nm in page_ride else len(self.prof.get(nm, ())))
            hist[k_] = hist.get(k_, 0) + 1
        (log or self.log)(
            f'  profiles (ONE_DIVE=4): changes per lane {dict(sorted(hist.items()))}, {n_ch} changes in all, '
            f'{len(residue)} residue {residue} ({len(prox)} proximity samples, '
            f'{sum(len(cand[nm]) for nm in M)} via slots, {len(rows)} rows, {_t.time() - t0:.1f} s, {msg[:40]}); '
            + ' '.join(f'{nm}@' + '/'.join(f'{c:.1f}' for c in ch) for nm, ch in sorted(self.prof.items())))

    # ------------------------------------------------------------ level 5: the tail
    def _line_start(self, nm):
        """Where a lane's scheduled line begins: its join leg, else its
        launch slot (the fan-in from the tooth stays on the tooth's layer)."""
        if nm in getattr(self, 'join_block', {}):
            return self.join_leg_s[nm]
        return max(self.s0, self.st[nm][0])

    def _tail_pieces(self, nm, mid=None, legs=None, jogs=None, se=None, in_exit=None,
                     s_line=None, joiner=None):
        """The lane from its tooth (a joiner) or its launch slot to its
        stub as tagged (s, o) pieces: 'hjog' (a joiner's jog along its
        tooth's row to its join leg), 'hleg' (the join leg, along o at
        its s, tooth side first), 'mid' (the region line, then the exit
        run at the slot or the head-on tail), 'leg' (the exit leg, along
        o at its s), 'jog' (along the stub's row from the leg back into
        the tip). The HEAD pieces are level 5's since 2026-09-11 late
        (HEAD_L5): the bench's SA7/SA9 -- two join legs 0.1 mm apart in
        s, one tooth on each layer -- were refused on every attempt
        because the schedule began at the leg's END, put SA7's birth via
        there, and SA9's leg stamp on F walled it (wallt5_hb_SA7.txt).
        `mid` / `legs` / `jogs` / `se` / `in_exit`: a CANDIDATE lane's
        geometry in place of the laid one's (_alt_pieces)."""
        s_a = self._line_start(nm) if s_line is None else s_line
        pts = self.mid[nm] if mid is None else mid
        legs = self.legs.get(nm) if legs is None else legs
        jogs = self.jogs.get(nm, ()) if jogs is None else jogs
        se = self.se[nm] if se is None else se
        in_exit = (nm in self.exit_block) if in_exit is None else in_exit
        is_joiner = (nm in getattr(self, 'join_block', {})) if joiner is None else joiner
        pieces = []
        if HEAD_L5 and is_joiner and legs:
            s_l, oa, ob = legs[0]
            s_t, o_t = (self.st[nm] if mid is None else (jogs[0][0] if jogs else (s_l, oa)))
            if abs(s_l - s_t) > 1e-6:
                pieces.append(('hjog', [(s_t, o_t), (s_l, o_t)]))
            if abs(ob - oa) > 1e-6:
                pieces.append(('hleg', [(s_l, oa), (s_l, ob)]))
        clipped = []
        for (sa, oa), (sb, ob) in zip(pts, pts[1:]):
            if sb < s_a - 1e-9:
                continue
            if not clipped:
                if sa < s_a - 1e-9:
                    t = (s_a - sa) / max(sb - sa, 1e-9)
                    clipped.append((s_a, oa + t * (ob - oa)))
                else:
                    clipped.append((sa, oa))
            clipped.append((sb, ob))
        if len(clipped) < 2:
            clipped = list(pts[-2:])
        pieces.append(('mid', clipped))
        if in_exit and legs:
            s_l, oa, ob = legs[-1]
            if abs(ob - oa) > 1e-6:
                pieces.append(('leg', [(s_l, oa), (s_l, ob)]))
            s_e, o_e = se
            for (p, q) in jogs:
                if (abs(q[0] - s_e) < 1e-6 and abs(q[1] - o_e) < 1e-6
                        and abs(p[0] - q[0]) > 1e-6):
                    pieces.append(('jog', [p, q]))
        return pieces

    @staticmethod
    def _sample_pieces(pieces, ds):
        """(tag, s, o, u) every `ds` of arc length in (s, o) along the
        pieces, every vertex included, ordered by u."""
        out = []
        u = 0.0
        nxt = 0.0
        for tag, pl in pieces:
            for (sa, oa), (sb, ob) in zip(pl, pl[1:]):
                L = math.hypot(sb - sa, ob - oa)
                if L < 1e-9:
                    continue
                if not out or abs(out[-1][3] - u) > 1e-9:
                    out.append((tag, sa, oa, u))
                while nxt <= u + L + 1e-9:
                    if nxt > u + 1e-9 and nxt < u + L - 1e-9:
                        t = (nxt - u) / L
                        out.append((tag, sa + t * (sb - sa), oa + t * (ob - oa), nxt))
                    nxt += ds
                u += L
                out.append((tag, sb, ob, u))
        return out

    @staticmethod
    def _pt_seg_dist(P, A, B):
        """Distances (m, n) from m points to n segments."""
        D = B - A
        L2 = (D * D).sum(1)
        L2 = np.where(L2 < 1e-12, 1e-12, L2)
        t = ((P[:, None, :] - A[None, :, :]) * D[None, :, :]).sum(2) / L2[None, :]
        t = np.clip(t, 0.0, 1.0)
        C = A[None, :, :] + t[:, :, None] * D[None, :, :]
        return np.hypot(P[:, 0, None] - C[:, :, 0], P[:, 1, None] - C[:, :, 1])

    def _l5_member_copper(self):
        """The corridor members' own copper as it came (teeth, stubs) as
        segment arrays per layer: excluded from obs_but, real copper all
        the same -- another member's tooth or stub is a HARD wall."""
        ctx = self.ctx
        LAY = ('F.Cu', 'B.Cu')
        mem_ids = {ctx.byname[nm][0] for nm in self.members}
        mem_segs = {L: [] for L in LAY}
        for sg in getattr(ctx, 'base_segments', ctx.pcb.segments):
            if sg.net_id in mem_ids and sg.layer in LAY:
                mem_segs[sg.layer].append((sg.net_id, sg.start_x, sg.start_y,
                                           sg.end_x, sg.end_y, sg.width))
        mem_arr = {}
        for L in LAY:
            rows_ = mem_segs[L]
            mem_arr[L] = (np.array([r[0] for r in rows_]),
                          np.array([[r[1], r[2]] for r in rows_]).reshape(-1, 2),
                          np.array([[r[3], r[4]] for r in rows_]).reshape(-1, 2),
                          np.array([r[5] for r in rows_]))
        return mem_arr

    def _l5_static(self, nm, d, mem_arr, pad_r):
        """Static copper along one sampled lane of `nm` (d: S, O, U, TAG,
        XY): foreign pads / vias / segments (obs_but) and the other
        members' own teeth and stubs. Fills d['blk'][L] (samples walled
        on L), d['hard'][L] (those walled by another member's copper)
        and d['room'] (samples with a via's room from all of it on both
        layers). Returns the count of samples walled on both layers --
        the router's, not the plan's, and dropped from both."""
        ctx = self.ctx
        M = self.members
        LAY = ('F.Cu', 'B.Cu')
        XY = d['XY']
        nid = ctx.byname[nm][0]
        blk, hard = {}, {}
        room = np.ones(len(XY), dtype=bool)
        for L in LAY:
            obs = ctx.obs_but(nm, M, L)
            b = np.array([obs.point_violation((float(p[0]), float(p[1]))) is not None
                          for p in XY])
            r = np.array([obs.point_violation((float(p[0]), float(p[1])), pad=pad_r) is not None
                          for p in XY])
            ids, A, B, W = mem_arr[L]
            keep = ids != nid
            hb = np.zeros(len(XY), dtype=bool)
            if keep.any():
                dd = self._pt_seg_dist(XY, A[keep], B[keep])
                rr = W[keep] / 2 + CLEAR + TRACK / 2
                hb = (dd < rr[None, :]).any(1)
                r |= (dd < (rr + pad_r)[None, :]).any(1)
            blk[L] = b | hb
            hard[L] = hb
            room &= ~r
        both = blk['F.Cu'] & blk['B.Cu']
        for L in LAY:
            blk[L] = blk[L] & ~both          # a sample walled on both layers is the router's, not the plan's
            hard[L] = hard[L] & ~both
        d['blk'] = blk
        d['hard'] = hard
        d['room'] = room
        return int(both.sum())

    def _via_room_mode(self):
        """How a via's room from a foreign MID line is measured right now:
        0 across the spine (o at equal s), 1 in xy for lanes that cross,
        2 in xy always. Raised by run() for the attempts after one that
        refused (VIA_ROOM_REFUSED) -- see that constant."""
        return max(VIA_ROOM_XY, getattr(self, '_vr_mode', 0))

    def _l5_build(self, lanes, samp, net_of, tl, dl, y_cost=None, excl=(), coarse=(),
                  cand_fixed=None, y_pair=None):
        """Level 5's MILP over `lanes`: keys into `samp`, each a sampled
        polyline with its static walls (_l5_static), belonging to the net
        `net_of[key]`, born on `tl[key]` and ending on `dl[key]`.

        A net with SEVERAL lanes is a residue net offered its candidate
        berths (_alts5): exactly one lane is chosen, by a binary y per
        lane that GATES every row of that lane -- its static walls, its
        proximity pairs, and the room its line takes from a neighbour's
        via slot -- so an unchosen candidate constrains nothing and costs
        nothing. `y_cost[key]` is what the berth itself costs (its own
        vias and its ride, on the judge's scale); `y_pair[(ka, kb)]` what
        the PAIR costs when BOTH are chosen -- the crossing term
        (DST_XING), which cannot be carried by `y_cost` because a
        crossing belongs to two lanes and a per-candidate constant
        double-counts or misses it (measured: summed per-candidate deltas
        drove K51's crossings UP, 352 -> 394); `excl` pairs of keys
        that cannot both be chosen (two candidates through one gap);
        `coarse` the lanes whose via slots are thinned (L5_ALT_SLOT_BG /
        L5_ALT_SLOT_STEP: the candidates); `cand_fixed` {key: slots} the
        slot lists to use as given -- the laid lanes' from the plain solve,
        so its solution maps onto the choice instance slot for slot (the
        candidates' proximity events would otherwise move a laid lane's
        slots, and the seed then violated its chain rows). With one lane
        per net (the braid's own solve) this is level 5's instance
        exactly, variables and rows in the same order.

        Returns a dict (idx, rows, lb, ub, cvec, integ, cand, prox, and
        the counts the log line prints). samp[key]['room'] is narrowed in
        place by the other nets' lines, as level 4 did."""
        from scipy.spatial import cKDTree
        LAY = ('F.Cu', 'B.Cu')
        M = list(lanes)
        y_cost = y_cost or {}
        y_pair = y_pair or {}
        by_net = {}
        for k in M:
            by_net.setdefault(net_of[k], []).append(k)
        gated = {k for k in M if len(by_net[net_of[k]]) > 1}
        # ---- proximity between lanes, and the room a via needs from them.
        # Two MID pieces (the region lines, the exit runs, the head-on
        # tails) are compared as level 4 compares them: by o at equal s.
        # That is what the octilinear router lays -- neighbouring lanes
        # a pitch apart in o stay a pitch apart however steep the fan --
        # where the Euclidean distance between two steep parallel
        # diagonals 0.35 apart in o is 0.2 (K15: SDQ14 and SDQ15 paired
        # over their whole length, the fan unschedulable). A pair with a
        # LEG or a JOG in it -- a crossing, an end beside a run -- is
        # measured in board xy, where the leg really is.
        mid_ix = {k: np.array([i for i, tg in enumerate(samp[k]['TAG']) if tg == 'mid'])
                  for k in M}
        s_lo = min(float(samp[k]['S'][mid_ix[k]].min()) for k in M)
        s_hi = max(float(samp[k]['S'][mid_ix[k]].max()) for k in M)
        # the sample grid is ANCHORED at a multiple of PROF_DS, not at the
        # lowest sample: with candidate lanes in the instance the lowest
        # sample moves, the grid shifted by a fraction of a step, and the
        # nearest-sample mapping put a held pair's crossing on the NEXT
        # slot -- the plain solution then violated two of its own rows
        # (K41: a SA3/SBA0 proximity row and a SCS1 slot the plain solve
        # had a via in). Anchored, the same pair maps to the same samples
        # whatever else is in the instance.
        s_lo = math.floor(s_lo / PROF_DS) * PROF_DS
        Sg = np.arange(s_lo, s_hi + PROF_DS + 1e-9, PROF_DS)
        Og, kg, gk = {}, {}, {}
        for k in M:
            ix = mid_ix[k]
            Sm, Om = samp[k]['S'][ix], samp[k]['O'][ix]
            # a strictly increasing s for the interpolation (vertices may
            # repeat an s); the grid point maps back to its nearest sample
            keep = np.concatenate([[True], np.diff(Sm) > 1e-9])
            Sm_, Om_, ix_ = Sm[keep], Om[keep], ix[keep]
            o = np.interp(Sg, Sm_, Om_)
            o[(Sg < Sm_[0] - 1e-9) | (Sg > Sm_[-1] + 1e-9)] = np.nan
            Og[k] = o
            kk = np.clip(np.searchsorted(Sm_, Sg), 0, len(Sm_) - 1)
            kk = np.where((kk > 0) & (np.abs(Sm_[np.maximum(kk - 1, 0)] - Sg)
                                      < np.abs(Sm_[kk] - Sg)), kk - 1, kk)
            kg[k] = ix_[kk]
            # ...and each mid sample's nearest grid point (the room test)
            gk[k] = np.clip(np.searchsorted(Sg, Sm), 0, len(Sg) - 1)
        prox = set()
        # the room a via needs from the other nets' lines: a single-lane
        # net's line KILLS the slot (as level 4), a candidate's line gates
        # it on its y (`gkill`: (key, sample) -> the candidates in the way)
        dmin = {k: np.full(Sg.shape, np.inf) for k in M}
        gkill = {}

        def _room_from(a_, b_, dd, ok):
            """b_'s mid line against a_'s via room: killed or gated."""
            if b_ not in gated:
                dmin[a_] = np.where(ok, np.minimum(dmin[a_], np.where(ok, dd, np.inf)), dmin[a_])
                return
            hit = ok[gk[a_]] & (dd[gk[a_]] < VIA_NEED)
            for i_, k_ in enumerate(mid_ix[a_]):
                if hit[i_]:
                    gkill.setdefault((a_, int(k_)), set()).add(b_)
        _vr = self._via_room_mode()
        xpair = set()      # mid lines whose order SWAPS: they cross somewhere
        for i, a_ in enumerate(M):
            for b_ in M[i + 1:]:
                if net_of[a_] == net_of[b_]:
                    continue          # one net's candidates never meet
                dd = np.abs(Og[a_] - Og[b_])
                ok = ~np.isnan(dd)
                _room_from(a_, b_, dd, ok)
                _room_from(b_, a_, dd, ok)
                if _vr == 1 and ok.any():
                    ends = np.where(ok)[0]
                    d0 = Og[a_][ends[0]] - Og[b_][ends[0]]
                    d1 = Og[a_][ends[-1]] - Og[b_][ends[-1]]
                    if d0 * d1 < 0:
                        xpair.add((a_, b_)); xpair.add((b_, a_))
                # every sample counts, a single one too: a steep crossing
                # in the fan is within PROX_TRACK for one or two samples
                # only, and dropping single-sample stretches (tried, for
                # K28 SDQ0's two-via ride under a near-touch) refused a
                # third of every board in band
                # a close stretch is a CROSSING when the signed offset
                # changes sign across it, a HUG when it is long; a stretch
                # of one or two samples with no sign change is a TOUCH --
                # two lines converging to ~0.23 and receding -- which the
                # router keeps legal in place (K28 SDQ0 rode B for two
                # vias under one, laid beside it at zero by level 4).
                # Dropping every single-sample stretch was tried and
                # refused a third of every board: steep crossings in the
                # fan are single samples too, and the sign is what tells
                sg = Og[a_] - Og[b_]
                close = ok & (dd < PROX_TRACK)
                gs = np.where(close)[0]
                i0 = 0
                while i0 < len(gs):
                    i1 = i0
                    while i1 + 1 < len(gs) and gs[i1 + 1] == gs[i1] + 1:
                        i1 += 1
                    g0, g1 = int(gs[i0]), int(gs[i1])
                    lo_g = g0 - 1 if g0 > 0 and ok[g0 - 1] else g0
                    hi_g = g1 + 1 if g1 + 1 < len(ok) and ok[g1 + 1] else g1
                    crossing = np.sign(sg[lo_g]) != np.sign(sg[hi_g])
                    if crossing or g1 - g0 + 1 > 2:
                        for g in range(g0, g1 + 1):
                            prox.add((a_, int(kg[a_][g]), b_, int(kg[b_][g])))
                    i0 = i1 + 1
        for k in M:
            # a mid sample's room from the other MID lines, as level 4
            ix = mid_ix[k]
            samp[k]['room'][ix] &= dmin[k][gk[k]] >= VIA_NEED
        allxy = np.concatenate([samp[k]['XY'] for k in M])
        lane_of = np.concatenate([np.full(len(samp[k]['XY']), i) for i, k in enumerate(M)])
        local = np.concatenate([np.arange(len(samp[k]['XY'])) for k in M])
        is_mid = np.concatenate([np.array([tg == 'mid' for tg in samp[k]['TAG']]) for k in M])
        net_i = [net_of[k] for k in M]
        tree = cKDTree(allxy)
        pp = tree.query_pairs(PROX_TRACK - 1e-6, output_type='ndarray')
        for i, j in pp:
            if net_i[lane_of[i]] != net_i[lane_of[j]] and not (is_mid[i] and is_mid[j]):
                prox.add((M[lane_of[i]], int(local[i]), M[lane_of[j]], int(local[j])))
        prox = sorted(prox, key=lambda t: (str(t[0]), t[1], str(t[2]), t[3]))
        vp = tree.query_pairs(VIA_NEED, output_type='ndarray')

        def _via_room_pair(i, j):
            """Is this pair's xy distance the via's real room? Always, unless
            both samples are MID lines -- then only at VIA_ROOM_XY (1: the two
            lanes cross; 2: any pair)."""
            if not (is_mid[i] and is_mid[j]):
                return True
            if _vr >= 2:
                return True
            return _vr == 1 and (M[lane_of[i]], M[lane_of[j]]) in xpair
        for i, j in vp:
            if net_i[lane_of[i]] != net_i[lane_of[j]] and _via_room_pair(i, j):
                for p_, q_ in ((i, j), (j, i)):
                    kp, kq = M[lane_of[p_]], M[lane_of[q_]]
                    if kq in gated:
                        gkill.setdefault((kp, int(local[p_])), set()).add(kq)
                    else:
                        samp[kp]['room'][local[p_]] = False
        # via slots: every second sample with room, as level 4 -- but on
        # a long exit run most of them are equivalent (nothing constrains
        # the lane there) and the solver ground through their symmetry
        # (K41: three solves at the 30 s limit). A slot is kept where
        # something HAPPENS within SLOT_REACH along the lane -- a
        # proximity sample, a wall, a piece boundary, an end -- plus a
        # coarse background every SLOT_BG for the tie-breaks
        ev = {k: {0.0, float(samp[k]['U'][-1])} for k in M}
        for a_, ka, b_, kb in prox:
            ev[a_].add(float(samp[a_]['U'][ka])); ev[b_].add(float(samp[b_]['U'][kb]))
        for k in M:
            d = samp[k]
            for L in LAY:
                for k_ in np.where(d['blk'][L])[0]:
                    ev[k].add(float(d['U'][k_]))
            for k_ in range(1, len(d['TAG'])):
                if d['TAG'][k_] != d['TAG'][k_ - 1]:
                    ev[k].add(float(d['U'][k_]))
        cand = {}
        coarse = set(coarse)
        for k in M:
            if cand_fixed and k in cand_fixed:
                cand[k] = list(cand_fixed[k])
                continue
            d = samp[k]
            U = d['U']
            E = np.array(sorted(ev[k]))
            j = np.clip(np.searchsorted(E, U), 0, len(E) - 1)
            near = np.minimum(np.abs(E[j] - U), np.abs(E[np.maximum(j - 1, 0)] - U)) <= SLOT_REACH
            s_bg, step = (L5_ALT_SLOT_BG, L5_ALT_SLOT_STEP) if k in coarse else (SLOT_BG, 2)
            bg = (np.floor(U / s_bg) != np.floor(np.concatenate([[-1.0], U[:-1]]) / s_bg))
            ks = [k_ for k_ in range(len(U)) if d['room'][k_] and (near[k_] or bg[k_])][::step]
            cand[k] = sorted(set(ks) | {0})
        # ---- variables and rows (level 4's, indexed by each lane's own
        # samples), every row of a candidate lane gated by its y
        idx = {}

        def var(key):
            if key not in idx:
                idx[key] = len(idx)
            return idx[key]
        for k in M:
            var(('w', net_of[k]))
            if k in gated:
                var(('y', k))
            for k_ in cand[k]:
                var(('up', k, k_)); var(('dn', k, k_)); var(('st', k, k_))
        rows, lb, ub = [], [], []

        def add(co, lo, hi):
            rows.append(dict(co)); lb.append(lo); ub.append(hi)

        def st_at(k, k_):
            last = None
            for kk in cand[k]:
                if kk <= k_:
                    last = kk
                else:
                    break
            return None if last is None else idx[('st', k, last)]

        def gate(k):
            return idx[('y', k)] if k in gated else None
        n_blk = 0
        for k in M:
            w = idx[('w', net_of[k])]
            y = gate(k)
            ks = cand[k]
            U = samp[k]['U']
            prev = None
            for k_ in ks:
                st, up, dn = idx[('st', k, k_)], idx[('up', k, k_)], idx[('dn', k, k_)]
                co = {st: 1, up: -1, dn: 1}
                if prev is not None:
                    co[prev] = -1
                add(co, 0, 0)
                add({up: 1, dn: 1}, -np.inf, 1)
                prev = st
            end = 1 if tl[k] != dl[k] else 0
            # big-M of ONE throughout level 5: every state is 0/1, so a
            # unit of w (or of a soft wall, or of an unchosen candidate's
            # 1 - y) relaxes any row completely, and the LP relaxation is
            # the tighter for it (4 left the solver at its 30 s limit
            # three times a K41 run)
            if y is None:
                add({prev: 1, w: 1}, end, end + 2)
                add({prev: 1, w: -1}, end - 2, end)
            else:
                add({prev: 1, w: 1, y: -1}, end - 1, np.inf)
                add({prev: 1, w: -1, y: 1}, -np.inf, end + 1)
            add({**{idx[('up', k, k_)]: 1 for k_ in ks}, **{idx[('dn', k, k_)]: 1 for k_ in ks},
                 w: TAIL_MAXCH}, -np.inf, TAIL_MAXCH)
            for i_, k_ in enumerate(ks):
                for j in ks[i_ + 1:]:
                    if U[j] - U[k_] >= VIA_SEP:
                        break
                    add({idx[('up', k, k_)]: 1, idx[('dn', k, k_)]: 1,
                         idx[('up', k, j)]: 1, idx[('dn', k, j)]: 1}, -np.inf, 1)
            # static copper: the layer it walls is forbidden there. HARD
            # where the wall is another MEMBER's tooth or stub (a leg
            # through a far-face stub on its layer is a short no dodge
            # fixes), SOFT for the rest of the static copper -- a
            # passive's pad in the corridor or at a leg's corner -- one
            # binary per walled stretch at ISLAND_W, below a via pair,
            # so the plan dives under an island only when the dive is
            # owed anyway (the early-dive rule) and otherwise leaves the
            # dodge to the deflection and the router, as level 4 did
            # (K28 SDQM0: C12.1's pad at its corner priced a hard B leg,
            # two vias the router never needed)
            for L in LAY:
                need = 1 if L == tl[k] else 0     # walled on the tooth layer -> state 1 (the other)
                wk = [int(k_) for k_ in np.where(samp[k]['blk'][L])[0]]
                runs_ = []
                for k_ in wk:
                    if runs_ and k_ == runs_[-1][1] + 1:
                        runs_[-1][1] = k_
                    else:
                        runs_.append([k_, k_])
                for a0, b0 in runs_:
                    soft = not samp[k]['hard'][L][a0:b0 + 1].any()
                    iv = var(('iv', k, L, a0)) if soft else None
                    for k_ in range(a0, b0 + 1):
                        v = st_at(k, k_)
                        if v is None:
                            continue
                        n_blk += 1
                        if need == 1:
                            co = {v: 1, w: 1}
                            if iv is not None:
                                co[iv] = 1
                            if y is not None:
                                co[y] = -1
                            add(co, 1 - (1 if y is not None else 0), np.inf)
                        else:
                            co = {v: 1, w: -1}
                            if iv is not None:
                                co[iv] = -1
                            if y is not None:
                                co[y] = 1
                            add(co, -np.inf, 0 + (1 if y is not None else 0))
        for a_, ka, b_, kb in prox:
            wa, wb = idx[('w', net_of[a_])], idx[('w', net_of[b_])]
            va, vb = st_at(a_, ka), st_at(b_, kb)
            ca = {va: 1} if va is not None else {}
            cb = {vb: 1} if vb is not None else {}
            ya, yb = gate(a_), gate(b_)
            ng = (1 if ya is not None else 0) + (1 if yb is not None else 0)
            gm = {}                       # the gates, relaxing the row when a candidate is unchosen
            for y_ in (ya, yb):
                if y_ is not None:
                    gm[y_] = gm.get(y_, 0) + 1
            if tl[a_] == tl[b_]:
                add({**ca, **cb, wa: 1, wb: 1, **{y_: -c_ for y_, c_ in gm.items()}}, 1 - ng, np.inf)
                add({**ca, **cb, wa: -1, wb: -1, **gm}, -np.inf, 1 + ng)
            else:
                co = dict(ca)
                for v_ in cb:
                    co[v_] = co.get(v_, 0) - 1
                add({**co, wa: -1, wb: -1, **gm}, -np.inf, 0 + ng)
                add({**{v_: -c_ for v_, c_ in co.items()}, wa: -1, wb: -1, **gm}, -np.inf, 0 + ng)
        # a via slot a candidate's line takes the room of: open while that
        # candidate is unchosen (never slot 0, the birth via, which the
        # slot list keeps whatever the room says)
        for (k, k_), kills in sorted(gkill.items(), key=lambda kv: (str(kv[0][0]), kv[0][1])):
            if k_ not in cand[k] or k_ == 0:
                continue
            for kq in sorted(kills, key=str):
                add({idx[('up', k, k_)]: 1, idx[('dn', k, k_)]: 1, idx[('y', kq)]: 1}, -np.inf, 1)
        # exactly one candidate per residue net; candidates through one gap
        for n_, ks_ in by_net.items():
            if len(ks_) > 1:
                add({idx[('y', k)]: 1 for k in ks_}, 1, 1)
        for ka_, kb_ in excl:
            if ka_ in gated and kb_ in gated:
                add({idx[('y', ka_)]: 1, idx[('y', kb_)]: 1}, -np.inf, 1)
        # PAIRWISE COSTS (y_pair): one z per pair, z >= ya + yb - 1 with
        # z in [0,1] and a POSITIVE weight, so minimising drives z to
        # exactly (ya AND yb). One variable and one row per pair, and the
        # caller only sends pairs whose crossing status VARIES across the
        # two nets' candidates -- a pair that crosses in every combination
        # (or in none) is a constant and is dropped there, which is what
        # keeps this from squaring the instance.
        pair_w = {}
        for (ka_, kb_), w_ in y_pair.items():
            if not w_ or ka_ not in gated or kb_ not in gated:
                continue
            vz = var(('z', ka_, kb_))
            pair_w[vz] = float(w_)
            add({vz: 1, idx[('y', ka_)]: -1, idx[('y', kb_)]: -1}, -1.0, np.inf)
        # identical rows collapse: consecutive samples of a proximity
        # stretch, or of a walled stretch, sit on the same state
        # variables, so the row set was 3-6x its distinct size (K41: the
        # solver spent its 30 s budget three times a run)
        seen_rows = {}
        for co, lo, hi in zip(rows, lb, ub):
            key = (tuple(sorted(co.items())), lo, hi)
            seen_rows.setdefault(key, (co, lo, hi))
        n_rows_raw = len(rows)
        rows = [v[0] for v in seen_rows.values()]
        lb = [v[1] for v in seen_rows.values()]
        ub = [v[2] for v in seen_rows.values()]
        nv = len(idx)
        cvec = np.zeros(nv)
        integ = np.zeros(nv)
        for key, v in idx.items():
            if key[0] == 'w':
                cvec[v] = RESIDUE_W + 0.01; integ[v] = 1     # a tie goes to the schedule
            elif key[0] in ('up', 'dn'):
                # a change costs one via; the 1e-4 tie-break by position
                # (earlier slots a hair cheaper) breaks the symmetry
                # between the twenty equal places a dive could sit,
                # which is what the branch-and-bound was grinding on
                u_ = float(samp[key[1]]['U'][key[2]]) / max(float(samp[key[1]]['U'][-1]), 1e-9)
                cvec[v] = 1.0 + 1e-4 * u_; integ[v] = 1
            elif key[0] == 'iv':
                cvec[v] = ISLAND_W; integ[v] = 1
            elif key[0] == 'y':
                cvec[v] = float(y_cost.get(key[1], 0.0)); integ[v] = 1
            elif key[0] == 'z':
                cvec[v] = pair_w.get(v, 0.0); integ[v] = 1
        return dict(idx=idx, rows=rows, lb=lb, ub=ub, cvec=cvec, integ=integ, cand=cand,
                    prox=prox, n_blk=n_blk, n_rows_raw=n_rows_raw, n_samples=len(allxy),
                    n_gated=len(gated), n_gkill=len(gkill), n_pair=len(pair_w))

    def _profiles5(self, sched, log=None):
        """ONE_DIVE level 5: level 4's schedule over the whole lane --
        region, exit run, leg and jog (see the flag's note). Called from
        lay_lanes once the lanes' polylines and legs are laid; rewrites
        the profile, the pages, the leg layers and the required stretches
        from its own solution. Returns True when a solution was taken."""
        from scipy.optimize import milp, LinearConstraint, Bounds
        from scipy.sparse import lil_matrix
        import hashlib
        import time as _t
        t0 = _t.time()
        ctx, sp = self.ctx, self.spine
        M = self.members
        tl, dl = ctx.tooth_layer, ctx.dest_layer
        LAY = ('F.Cu', 'B.Cu')
        pad_r = (VIA_SIZE - TRACK) / 2
        # a failed solve must not leave a previous attempt's tail behind
        self.tail_vias, self.leg_prof, self.jog_prof = None, {}, {}
        self.head_prof, self.hjog_prof, self.prof_L0 = {}, {}, {}
        self.full_prof, self.full_prof_s = {}, {}
        # ---- the geometry, sampled
        samp = {}                       # nm -> dict of arrays
        for nm in M:
            pcs = self._tail_pieces(nm)
            raw = self._sample_pieces(pcs, PROF_DS)
            S = np.array([r[1] for r in raw]); O = np.array([r[2] for r in raw])
            U = np.array([r[3] for r in raw])
            TAG = [r[0] for r in raw]
            XY = np.array([sp.xy(float(s), float(o)) for s, o in zip(S, O)])
            samp[nm] = dict(S=S, O=O, U=U, TAG=TAG, XY=XY)
        # ---- static copper along every lane, then the instance: the
        # proximity pairs, the via slots, the variables and the rows are
        # the builder's (_l5_build), shared with the berth-choice solve
        # (_alts5) -- one lane per net here, so level 5's instance exactly
        mem_arr = self._l5_member_copper()
        n_both = 0
        for nm in M:
            n_both += self._l5_static(nm, samp[nm], mem_arr, pad_r)
        inst = self._l5_build(M, samp, {nm: nm for nm in M}, dict(tl), dict(dl))
        idx, rows, lb, ub = inst['idx'], inst['rows'], inst['lb'], inst['ub']
        cvec, integ, cand, prox = inst['cvec'], inst['integ'], inst['cand'], inst['prox']
        n_blk, n_rows_raw, n_samples = inst['n_blk'], inst['n_rows_raw'], inst['n_samples']
        nv = len(idx)
        # ---- the MILP (level 4's, indexed by each lane's own samples)
        h = hashlib.sha1()
        for nm in M:
            d = samp[nm]
            h.update(nm.encode()); h.update(tl[nm].encode()); h.update(dl[nm].encode())
            h.update(np.round(d['S'], 4).tobytes()); h.update(np.round(d['O'], 4).tobytes())
            h.update(d['blk']['F.Cu'].tobytes()); h.update(d['blk']['B.Cu'].tobytes())
            h.update(d['hard']['F.Cu'].tobytes()); h.update(d['hard']['B.Cu'].tobytes())
            h.update(d['room'].tobytes())
        cap = getattr(self, '_l5_time', L5_TIME)
        nodes = getattr(self, '_l5_nodes', L5_NODES)
        # the cap is part of the key: braid.py's own plan-only pass (the
        # sidecar's) runs first under the judge's cap, and a solution it
        # cut short must not serve the real attempts (K41: attempt 0 laid
        # a 3 s incumbent, 84 vias for the 76 the 10 s optimum routed)
        mkey = ('L5', h.hexdigest(), PROX_TRACK, TAIL_MAXCH, RESIDUE_W, VIA_NEED, VIA_SEP, ISLAND_W,
                SLOT_REACH, SLOT_BG, cap, nodes, self._via_room_mode())
        memo = _PROFILE_MEMO
        seed_key = frozenset(M)
        solve_note = 'memo'
        if mkey in memo:
            x, msg = memo[mkey]
        else:
            # the incumbent within 1% is the answer: HiGHS finds it in
            # seconds and spent 145 s proving one K35 instance optimal.
            # Two stages (see L5_SEED): the previous residue set fixed
            # -- milliseconds -- seeds the full solve with a feasible
            # incumbent, so the time limit caps the PROOF, not the answer
            x0, seed_from, t_seed = None, None, 0.0
            quick = None
            if L5_SEED:
                seed = _L5_SEED.get(seed_key)
                seed_from = 'previous'
                if seed is None and getattr(self, '_od_residue', None) is not None:
                    seed, seed_from = set(self._od_residue), 'level 4'
                if seed is not None and L5_JUDGE_QUICK and getattr(self.ctx, 'judge', False):
                    # the quick judge (see L5_JUDGE_QUICK): scheduled lanes
                    # stay scheduled, the seed's residue lanes choose
                    lo_b, hi_b = np.zeros(nv), np.ones(nv)
                    for nm in M:
                        if nm not in seed:
                            hi_b[idx[('w', nm)]] = 0.0
                    t_s = _t.time()
                    xq, mq, okq = _milp_solve(cvec, rows, lb, ub, integ, lo_b, hi_b,
                                              cap, L5_GAP, pscost=L5_PSCOST, nodes=nodes)
                    t_seed = _t.time() - t_s
                    if okq:
                        quick = (xq, mq)
                if seed is not None and quick is None:
                    lo_b, hi_b = np.zeros(nv), np.ones(nv)
                    for nm in M:
                        v_ = idx[('w', nm)]
                        lo_b[v_] = hi_b[v_] = 1.0 if nm in seed else 0.0
                    t_s = _t.time()
                    x0, _m0, ok0 = _milp_solve(cvec, rows, lb, ub, integ, lo_b, hi_b,
                                               min(cap, 5.0), L5_GAP, pscost=L5_PSCOST, nodes=L5_JUDGE_NODES)
                    if not ok0:
                        # the seed's non-residue lanes cannot all be
                        # scheduled here: keep the seed's residue fixed and
                        # let the rest choose, briefly -- a feasible
                        # incumbent still, where a cold solve would be
                        # 10-20 s (K41 attempt 0 on a re-planned board)
                        for nm in M:
                            if nm not in seed:
                                lo_b[idx[('w', nm)]] = 0.0
                                hi_b[idx[('w', nm)]] = 1.0
                        x0, _m0, ok0 = _milp_solve(cvec, rows, lb, ub, integ, lo_b, hi_b,
                                                   min(cap, 3.0), L5_GAP, pscost=L5_PSCOST, nodes=L5_JUDGE_NODES)
                        seed_from += ' (partial)'
                    t_seed = _t.time() - t_s
                    if not ok0:
                        x0 = None
            t_s = _t.time()
            if quick is not None:
                x, msg, ok = quick[0], quick[1], True
                solve_note = f'quick judge {len(seed)} free, {t_seed:.1f} s'
            else:
                x, msg, ok = _milp_solve(cvec, rows, lb, ub, integ, np.zeros(nv), np.ones(nv),
                                         cap, L5_GAP, x0=x0, pscost=L5_PSCOST, nodes=nodes)
                solve_note = (f'seed {seed_from} {len(seed)} {t_seed:.1f} s' if x0 is not None else 'cold') \
                    + f', solve {_t.time() - t_s:.1f} s'
            if not ok:
                (log or self.log)(f'  profiles5: no solution ({msg}); level 4 kept')
                return False
            memo[mkey] = (x, msg)
            if os.environ.get('BRAID_L5_DUMP'):
                # diagnostic: the instance and its solution, for offline solver experiments
                import pickle
                n_d = len([k for k in os.listdir(os.path.dirname(os.environ['BRAID_L5_DUMP']) or '.')
                           if k.startswith(os.path.basename(os.environ['BRAID_L5_DUMP']))])
                with open(f"{os.environ['BRAID_L5_DUMP']}_{n_d}.pkl", 'wb') as fh:
                    pickle.dump(dict(cvec=cvec, integ=integ, rows=rows, lb=lb, ub=ub, idx=idx, x=x, msg=msg,
                                     U={nm: samp[nm]['U'] for nm in M}, TAG={nm: samp[nm]['TAG'] for nm in M},
                                     S={nm: samp[nm]['S'] for nm in M}, cand=cand, tl=dict(tl), dl=dict(dl),
                                     M=list(M), s1=self.s1, secs=_t.time() - t0), fh)
        # ---- the solution, read back per lane
        residue = [nm for nm in M if x[idx[('w', nm)]] > 0.5]
        if residue and os.environ.get('BRAID_L5_WHY'):
            # diagnostic: is a residue lane priced out or infeasible?
            base_cost = float(cvec @ x)
            A = lil_matrix((max(1, len(rows)), nv))
            for i_, co in enumerate(rows):
                for k_, v_ in co.items():
                    A[i_, k_] = v_
            A = A.tocsr()
            why = []
            for nm in residue:
                lo_b, hi_b = np.zeros(nv), np.ones(nv)
                hi_b[idx[('w', nm)]] = 0.0
                r2 = milp(cvec, constraints=LinearConstraint(A, lb, ub), integrality=integ,
                          bounds=Bounds(lo_b, hi_b), options={'time_limit': 60})
                if r2.x is None:
                    why.append(f'{nm}: INFEASIBLE')
                else:
                    why.append(f'{nm}: forced cost {float(cvec @ r2.x):.0f} (+{float(cvec @ r2.x) - base_cost:.0f})')
            (log or self.log)('  profiles5 residue why: ' + '; '.join(why))
        self.prof, self.dive, self.ride = {}, {}, {}
        self.tail_ch, self.tail_vias, self.leg_prof, self.jog_prof = {}, {}, {}, {}
        self.head_prof, self.hjog_prof, self.prof_L0 = {}, {}, {}
        self.full_prof, self.full_prof_s = {}, {}
        self.slot_via = {}
        n_piece = {'hjog': 0, 'hleg': 0, 'mid': 0, 'run': 0, 'leg': 0, 'jog': 0}
        for nm in M:
            if nm in residue:
                sched.page[nm] = None
                continue
            d = samp[nm]
            tags = d['TAG']
            ch_k = sorted(k for k in cand[nm]
                          if x[idx[('up', nm, k)]] > 0.5 or x[idx[('dn', nm, k)]] > 0.5)
            L = tl[nm]
            seq = [(0.0, L)]
            seq_s = []
            mid_ch, tail_pts, leg_ch, jog_ch = [], [], [], []
            hleg_ch, hjog_ch = [], []
            birth = False
            # the sample where the region line begins (the launch slot):
            # a joiner's is its head leg's last sample, the mid's first
            # vertex having the same u (see _sample_pieces)
            k_mid0 = next((k for k, tg in enumerate(tags) if tg == 'mid'), 0)
            if k_mid0 > 0 and tags[k_mid0 - 1] in ('hleg', 'hjog'):
                k_mid0 -= 1
            for k in ch_k:
                L = other_layer(L)
                tag, s_, o_ = tags[k], float(d['S'][k]), float(d['O'][k])
                seq.append((float(d['U'][k]), L))
                seq_s.append((s_, L))
                xy = (float(d['XY'][k][0]), float(d['XY'][k][1]))
                if k == k_mid0:
                    # the birth via at the launch slot, whichever piece
                    # the sample belongs to
                    birth = True
                    mid_ch.append(s_)
                    n_piece['mid'] += 1
                elif tag == 'mid':
                    mid_ch.append(s_)
                    n_piece['run' if s_ > self.s1 + 1e-9 else 'mid'] += 1
                    if s_ > self.s1 + 1e-9:
                        tail_pts.append(xy)
                elif tag == 'leg':
                    leg_ch.append((o_, L)); tail_pts.append(xy); n_piece['leg'] += 1
                elif tag == 'jog':
                    jog_ch.append((s_, L)); tail_pts.append(xy); n_piece['jog'] += 1
                elif tag == 'hleg':
                    hleg_ch.append((o_, L)); tail_pts.append(xy); n_piece['hleg'] += 1
                else:
                    hjog_ch.append((s_, L)); tail_pts.append(xy); n_piece['hjog'] += 1
            self.prof[nm] = mid_ch
            self.full_prof[nm] = seq
            self.full_prof_s[nm] = seq_s
            self.tail_vias[nm] = tail_pts
            if k_mid0 > 0:
                # the head: the jog's and the leg's layer at their starts
                # (the tooth's, then whatever the jog left it at) and the
                # layer the region line begins on
                Lh = tl[nm]
                for (o_c, L_a) in hjog_ch:
                    Lh = L_a
                self.hjog_prof[nm] = (tl[nm], hjog_ch)
                self.head_prof[nm] = (Lh, hleg_ch)
                for (o_c, L_a) in hleg_ch:
                    Lh = L_a
                # (a birth via is in mid_ch, and flips it from here on)
                self.prof_L0[nm] = Lh
            # the page: the layer at the region's end
            k1 = int(np.searchsorted(d['S'], self.s1, side='right')) - 1
            k1 = max(0, min(len(d['U']) - 1, k1))
            u1 = float(d['U'][k1])
            L1 = tl[nm]
            for (u_, L_) in seq[1:]:
                if u_ <= u1 + 1e-9:
                    L1 = L_
            sched.page[nm] = L1
            # the leg and the jog, from the layer in force where each begins
            if nm in self.exit_block:
                tags = d['TAG']
                try:
                    k_leg = tags.index('leg')
                except ValueError:
                    k_leg = None
                if k_leg is not None:
                    u_leg = float(d['U'][k_leg])
                    Lg = tl[nm]
                    for (u_, L_) in seq[1:]:
                        if u_ <= u_leg - 1e-9:
                            Lg = L_
                    self.leg_prof[nm] = (Lg, leg_ch)
                    self.leg_layer[nm] = Lg
                    self.leg_split.pop(nm, None)
                    if leg_ch:
                        self.leg_split[nm] = leg_ch[0][0]
                try:
                    k_jog = tags.index('jog')
                except ValueError:
                    k_jog = None
                if k_jog is not None:
                    u_jog = float(d['U'][k_jog])
                    Lj = tl[nm]
                    for (u_, L_) in seq[1:]:
                        if u_ <= u_jog - 1e-9:
                            Lj = L_
                    self.jog_prof[nm] = (Lj, jog_ch)
            self.slot_via[nm] = (birth or (k_mid0 == 0 and bool(ch_k) and ch_k[0] == 0),
                                 any(abs(float(d['S'][k]) - self.s1) <= PROF_DS + 1e-9
                                     and d['TAG'][k] == 'mid' for k in ch_k))
        keep = {nm for nm in sched.launch if sched.page[nm] == 'F.Cu'}
        sched.b_page = [nm for nm in sched.launch if sched.page[nm] == 'B.Cu']
        sched.swimmers = [nm for nm in sched.launch if sched.page[nm] is None]
        sched.divers = {nm for nm in sched.launch if nm not in keep}
        sched.birth_b = {d_ for d_ in sched.divers if tl.get(d_) == 'B.Cu'}
        self._od_residue = list(residue)
        _L5_SEED[seed_key] = set(residue)
        self._l5 = dict(samp=samp, cand=cand, prox=prox, residue=list(residue))   # probes read this
        # ---- self-check: every proximity pair on different layers, no
        # sample on a layer static copper walls

        def layer_at(nm, k):
            u = float(samp[nm]['U'][k])
            L = tl[nm]
            for (u_, L_) in self.full_prof[nm][1:]:
                if u_ <= u + 1e-9:
                    L = L_
            return L
        bad = []
        for a_, ka, b_, kb in prox:
            if a_ in residue or b_ in residue:
                continue
            if layer_at(a_, ka) == layer_at(b_, kb):
                bad.append((a_, b_, round(float(samp[a_]['S'][ka]), 2)))
        for nm in M:
            if nm in residue:
                continue
            for L in LAY:
                # a soft wall left to the router is a choice, not a fault
                for k in np.where(samp[nm]['hard'][L])[0]:
                    if layer_at(nm, int(k)) == L:
                        bad.append((nm, 'stub ' + L[0], round(float(samp[nm]['S'][k]), 2)))
        if bad:
            (log or self.log)(f'  profiles5: SELF-CHECK FAILED, {len(bad)} illegal sample(s): {bad[:8]}')
        n_ch = sum(len(v) - 1 for v in self.full_prof.values())
        hist = {}
        for nm in M:
            k_ = 'swim' if nm in residue else str(len(self.full_prof[nm]) - 1)
            hist[k_] = hist.get(k_, 0) + 1
        (log or self.log)(
            f'  profiles (ONE_DIVE=5, tail): changes per lane {dict(sorted(hist.items()))}, '
            f'{n_ch} changes in all (head {n_piece["hjog"]}+{n_piece["hleg"]}, region {n_piece["mid"]}, '
            f'run {n_piece["run"]}, leg {n_piece["leg"]}, jog {n_piece["jog"]}), {len(residue)} residue {residue} '
            f'({n_samples} samples, {len(prox)} proximity pairs, {n_blk} static-walled, '
            f'{n_both} walled both layers, {sum(len(cand[nm]) for nm in M)} via slots, '
            f'{len(rows)} rows ({n_rows_raw} raw), {_t.time() - t0:.1f} s, {solve_note}, {msg[:40]}); '
            + ' '.join(f'{nm}@' + '/'.join(f'{c:.1f}' for c in ch) for nm, ch in sorted(self.prof.items()) if ch)
            + ' | legs ' + ' '.join(f'{nm}:{v[0][0]}' + ''.join(f'/{o:+.1f}{L_[0]}' for o, L_ in v[1])
                                    for nm, v in sorted(self.leg_prof.items()))
            + ' | jogs ' + ' '.join(f'{nm}:{v[0][0]}' + ''.join(f'/{s_:.1f}{L_[0]}' for s_, L_ in v[1])
                                    for nm, v in sorted(self.jog_prof.items()) if v[1] or v[0] != dl[nm])
            + ' | head ' + ' '.join(f'{nm}:{v[0][0]}' + ''.join(f'/{o_:+.1f}{L_[0]}' for o_, L_ in v[1])
                                    + (f' L0 {self.prof_L0[nm][0]}' if self.prof_L0.get(nm) != tl[nm] else '')
                                    for nm, v in sorted(self.head_prof.items())
                                    if v[1] or self.hjog_prof[nm][1] or self.prof_L0.get(nm) != tl[nm]))
        # the BERTH CHOICE inside the solve (_alts5): the residue nets the
        # plan offers candidate berths for (ctx.alts, the fanout loop's
        # residue search) choose among them in one more MILP, the other
        # lanes' lines held; the answer is advice for the loop (alt_choice),
        # the profile read back above stands
        self.alt_choice, self.alt_note = {}, None
        alts = getattr(ctx, 'alts', None) or {}
        todo = [nm for nm in M if alts.get(nm)]      # residue nets alone (DST_RESIDUE=2) or every net (=3)
        if todo:
            try:
                self._alts5(samp, todo, x, idx, cand, mkey, cap, log)
            except Exception as e:      # advice only: never fails the braid
                (log or self.log)(f'  profiles5 alts: not solved ({type(e).__name__}: {e})')
                if os.environ.get('BRAID_L5_ALT_LOG'):
                    import traceback
                    traceback.print_exc()
        return True

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
        bl = self._band_line_at(pt, ref)
        head = (nm not in self.flank) and self._head_exit(nm, so, pt, L_a, bl, record=False)
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
                     if om != nm and self.exit_side.get(om) == sg and not self._in_band(om))
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

    def _alt_pieces(self, nm, alt, o_t=None):
        """The lane of `nm` to a CANDIDATE berth `alt` ({'exit': xy,
        'layer', 'dir'}), every other lane held where it is: the tagged
        (s, o) pieces _tail_pieces gives the laid lane, built the way
        lay_lanes builds one. The candidate's stub projects on the same
        spine; it is head-on or a side exit by classify's own test
        against the other stubs as they stand; a side exit keeps the
        net's slot when it stays on its side of the spine and takes the
        block's next outer slot otherwise; its leg is placed by _leg_s
        against the legs already placed. The region line runs straight
        from the launch slot to the target slot (no swimmer hold: the
        schedule is the solve's to decide). Returns (pieces, berth
        layer, info)."""
        ctx, sp = self.ctx, self.spine
        g = self._alt_geo(nm, alt)
        if g.get('src'):
            # a candidate TOOTH: the launch end is the candidate's, the
            # exit end the laid one's (its slot, leg and jog as laid)
            s_t, o_t0 = g['se']
            tl_a = g['layer']
            o_l = self._alt_src_slot(nm, g)
            legs, jogs = [], []
            if g['head']:
                pts = [(s_t, o_t0)]
            else:
                placed = [(self.join_leg_s[om], min(self.st[om][1], self.join_block[om]),
                           max(self.st[om][1], self.join_block[om]))
                          for om in self.join_block if om != nm]
                s_l = self._leg_s(nm, s_t, o_t0, o_l, True, placed, own=g['se'], own_L=tl_a)
                if abs(s_l - s_t) > 1e-9:
                    jogs.append(((s_t, o_t0), (s_l, o_t0)))
                legs.append((s_l, o_t0, o_l))
                pts = [(s_l, o_l)]
            pts.append((self.s_of_u(0.0), o_l))
            pts.append((self.s1, self.target_o[nm]))
            if nm in self.exit_block:
                s_l2 = self.exit_leg_s[nm]
                s_e, o_e = self.se[nm]
                pts.append((min(s_l2, s_e), self.target_o[nm]))
                if s_l2 > s_e + 1e-9:
                    pts.append((s_l2, self.target_o[nm]))
                legs.append((s_l2, self.target_o[nm], o_e))
                if abs(s_l2 - s_e) > 1e-9:
                    jogs.append(((s_l2, o_e), (s_e, o_e)))
            else:
                pts.append(self.se[nm])
            clean = [pts[0]]
            for p in pts[1:]:
                if p[0] >= clean[-1][0] - 1e-9:
                    clean.append((max(p[0], clean[-1][0]), p[1]))
            # the head pieces (HEAD_L5) read self.legs[nm][0] for a joiner's
            # jog/leg: _tail_pieces takes them from `legs` when the tooth is
            # a candidate joiner; a head-on candidate has none
            pieces = self._tail_pieces(nm, mid=clean, legs=legs, jogs=jogs, se=self.se[nm],
                                       in_exit=nm in self.exit_block, s_line=(s_l if not g['head'] else max(self.s0, s_t)),
                                       joiner=not g['head'])
            info = dict(head=g['head'], far=False, side=g['side'], se=g['se'], o_t=o_l,
                        leg_s=(legs[0][0] if not g['head'] else None), layer=tl_a, src=True)
            return pieces, tl_a, info
        stub, dl_a, se_a, head, sg, far = g['stub'], g['layer'], g['se'], g['head'], g['side'], g['far']
        s_e, o_e = se_a
        o_t = self._alt_slot(nm, g) if o_t is None else o_t
        s_t, o_t0 = self.st[nm]
        headed = nm in self.join_block or nm in getattr(self, 'face_block', {})
        if nm in self.join_block:
            pts = [(self.join_leg_s[nm], self.join_block[nm])]
        elif nm in getattr(self, 'face_block', {}):
            pts = [(self.face_leg_s[nm], self.face_block[nm])]
        else:
            pts = [(s_t, o_t0)]
        if nm not in getattr(self, 'face_start', {}):
            pts.append((self.s_of_u(0.0), self.launch_o[nm]))
        pts.append((self.s1, o_t))
        legs = list(self.legs.get(nm, ())[:1]) if headed else []
        jogs = []
        in_exit = not head
        if in_exit:
            placed = [(self.exit_leg_s[om], min(self.exit_block[om], self.se[om][1]),
                       max(self.exit_block[om], self.se[om][1]))
                      for om in self.exit_block if om != nm and om in self.exit_leg_s]
            s_base, avoid = s_e, None
            if far:
                floor = self.s_leg_min
                if floor is None:
                    floor = self.s_ball + self.r_max + CLEAR + TRACK / 2 + 0.05
                s_base = max(s_e, floor)

                def avoid(n, s, _f=floor):
                    return s < _f - 1e-9
            s_l = self._leg_s(nm, s_base, o_t, o_e, False, placed, avoid,
                              own=se_a, own_L=dl_a)
            pts.append((min(s_l, s_e), o_t))
            if s_l > s_e + 1e-9:
                pts.append((s_l, o_t))
            legs.append((s_l, o_t, o_e))
            if abs(s_l - s_e) > 1e-9:
                jogs.append(((s_l, o_e), (s_e, o_e)))
        else:
            pts.append((s_e, o_e))
        clean = [pts[0]]
        for p in pts[1:]:
            if p[0] >= clean[-1][0] - 1e-9:
                clean.append((max(p[0], clean[-1][0]), p[1]))
        pieces = self._tail_pieces(nm, mid=clean, legs=legs, jogs=jogs, se=se_a, in_exit=in_exit)
        info = dict(head=head, far=far, side=sg, se=se_a, o_t=o_t,
                    leg_s=(legs[-1][0] if in_exit else None), layer=dl_a)
        return pieces, dl_a, info

    def _alts5(self, samp, todo, x_plain, idx_plain, cand_plain, mkey, cap, log=None):
        """The BERTH CHOICE inside one level-5 MILP (see L5_ALT_NODES).
        For every residue net in `todo` the plan offers candidate berths
        (ctx.alts[nm]); each becomes a lane of its own (_alt_pieces,
        sampled and walled like the laid lanes) beside the laid lane,
        and _l5_build gates every row of each by a choice binary -- so
        one solve, seeded with level 5's own answer (the laid berths
        chosen, feasible by construction), decides which berth each
        residue net takes together with every lane's schedule against
        it. The candidates' costs (their own vias, their ride) enter the
        objective on the judge's scale. Advice for the fanout loop:
        self.alt_choice (net -> candidate index, 0 = as laid), self.alt_w
        (still residue with it), self.alt_obj (plain, chosen)."""
        import hashlib
        import time as _t
        t0 = _t.time()
        ctx, M, sp = self.ctx, self.members, self.spine
        tl, dl = ctx.tooth_layer, ctx.dest_layer
        pad_r = (VIA_SIZE - TRACK) / 2
        mem_arr = self._l5_member_copper()
        lanes, samp2, net_of, tl_of, dl_of, y_cost, info = [], {}, {}, {}, {}, {}, {}
        h = hashlib.sha1()
        n_cand = 0
        for nm in M:
            if nm not in todo:
                lanes.append(nm); samp2[nm] = samp[nm]
                net_of[nm], tl_of[nm], dl_of[nm] = nm, tl[nm], dl[nm]
                continue
            k0 = (nm, 0)
            lanes.append(k0); samp2[k0] = samp[nm]
            net_of[k0], tl_of[k0], dl_of[k0], y_cost[k0] = nm, tl[nm], dl[nm], 0.0
            for j, alt in enumerate(ctx.alts[nm], 1):
                pcs, L_j, inf = self._alt_pieces(nm, alt)
                dl_j = dl[nm] if inf.get('src') else L_j
                tl_j = L_j if inf.get('src') else tl[nm]
                raw = self._sample_pieces(pcs, PROF_DS)
                S = np.array([r[1] for r in raw]); O = np.array([r[2] for r in raw])
                U = np.array([r[3] for r in raw])
                TAG = [r[0] for r in raw]
                XY = np.array([sp.xy(float(s_), float(o_)) for s_, o_ in zip(S, O)])
                d = dict(S=S, O=O, U=U, TAG=TAG, XY=XY)
                self._l5_static(nm, d, mem_arr, pad_r)
                kj = (nm, j)
                lanes.append(kj); samp2[kj] = d
                net_of[kj], tl_of[kj], dl_of[kj] = nm, tl_j, dl_j
                y_cost[kj] = float(alt.get('cost', 0.0))
                info[kj] = inf
                n_cand += 1
                h.update(f'{nm}:{j}:{dl_j}:{y_cost[kj]:.4f}'.encode())
                h.update(np.round(S, 4).tobytes()); h.update(np.round(O, 4).tobytes())
                h.update(d['blk']['F.Cu'].tobytes()); h.update(d['blk']['B.Cu'].tobytes())
                h.update(d['room'].tobytes())
        excl = []
        for e in getattr(ctx, 'alt_excl', None) or ():
            a_, i_, b_, j_ = e
            excl.append(((a_, int(i_)), (b_, int(j_))))
            h.update(f'x{a_}:{i_}:{b_}:{j_}'.encode())
        # the CROSSING term's pairwise costs (DST_XING), keyed like excl
        y_pair = {}
        for e in getattr(ctx, 'alt_xing', None) or ():
            a_, i_, b_, j_, w_ = e
            y_pair[((a_, int(i_)), (b_, int(j_)))] = float(w_)
            h.update(f'z{a_}:{i_}:{b_}:{j_}:{float(w_):.4f}'.encode())
        inst = self._l5_build(lanes, samp2, net_of, tl_of, dl_of, y_cost=y_cost, excl=excl,
                              y_pair=y_pair,
                              coarse=[k for k in lanes if isinstance(k, tuple) and k[1] > 0],
                              cand_fixed={k: cand_plain[k if not isinstance(k, tuple) else k[0]]
                                          for k in lanes if not isinstance(k, tuple) or k[1] == 0})
        idx, nv = inst['idx'], len(inst['idx'])
        # seeded with the plain solution: y at the laid berths, every other
        # candidate idle -- feasible by construction (their rows relaxed)
        x0 = np.zeros(nv)
        for key, v in idx.items():
            if key[0] == 'y':
                x0[v] = 1.0 if key[1][1] == 0 else 0.0
                continue
            k0 = key
            if key[0] in ('up', 'dn', 'st', 'iv') and isinstance(key[1], tuple):
                if key[1][1] != 0:
                    continue
                k0 = (key[0], key[1][0]) + tuple(key[2:])
            v0 = idx_plain.get(k0)
            if v0 is not None:
                x0[v] = x_plain[v0]
        obj0 = float(inst['cvec'] @ x0)
        n_viol = 0
        for co, lo_, hi_ in zip(inst['rows'], inst['lb'], inst['ub']):
            v_ = sum(c_ * x0[v] for v, c_ in co.items())
            if v_ < lo_ - 1e-6 or v_ > hi_ + 1e-6:
                n_viol += 1
        akey = mkey + ('alts', h.hexdigest(), L5_ALT_NODES)
        memo = _PROFILE_MEMO
        if akey in memo:
            x, msg, obj_a = memo[akey]
            note = 'memo'
        else:
            # stage A: the laid lanes' schedules and the nets without
            # candidates fixed as level 5 solved them; the residue nets
            # choose (see L5_ALT_NODES)
            # ...every AS-LAID lane's schedule is held too: a net that stays
            # keeps level 5's schedule, a net that moves relaxes it by its
            # y, so nothing but the candidates and the choice is free (the
            # joint order, DST_RESIDUE=3, offers every net candidates and
            # would otherwise hold nothing)
            lo_a, hi_a = np.zeros(nv), np.ones(nv)
            for key, v in idx.items():
                held = ((key[0] in ('up', 'dn', 'st', 'iv')
                         and (not isinstance(key[1], tuple) or key[1][1] == 0))
                        or (key[0] == 'w' and key[1] not in todo))
                if held:
                    lo_a[v] = hi_a[v] = float(round(float(x0[v])))
            t_s = _t.time()
            xs, n_pn = x0, 0
            if L5_ALT_PERNET and len(todo) > 1:
                # the per-net sweep (see L5_ALT_PERNET): residue nets first
                res_now = [nm for nm in M if x_plain[idx_plain[('w', nm)]] > 0.5]
                order = [nm for nm in todo if nm in res_now] + [nm for nm in todo if nm not in res_now]
                for nm in order:
                    lo_n, hi_n = lo_a.copy(), hi_a.copy()
                    for key, v in idx.items():
                        if key[0] == 'y' and key[1][0] != nm:
                            lo_n[v] = hi_n[v] = float(round(float(xs[v])))
                        elif key[0] in ('up', 'dn', 'st', 'iv') and isinstance(key[1], tuple) \
                                and key[1][0] != nm and key[1][1] > 0:
                            lo_n[v] = hi_n[v] = float(round(float(xs[v])))
                        elif key[0] == 'w' and key[1] != nm and key[1] in todo:
                            lo_n[v] = hi_n[v] = float(round(float(xs[v])))
                    xn, msg_n, ok_n = _milp_solve(inst['cvec'], inst['rows'], inst['lb'], inst['ub'], inst['integ'],
                                                  lo_n, hi_n, min(L5_ALT_TIME, 10.0), L5_GAP, x0=xs,
                                                  pscost=L5_PSCOST, nodes=200, solver=ALT_SOLVER)
                    if ok_n and float(inst['cvec'] @ xn) < float(inst['cvec'] @ xs) - 1e-9:
                        xs = xn
                        n_pn += 1
            xa, msg_a, ok_a = _milp_solve(inst['cvec'], inst['rows'], inst['lb'], inst['ub'], inst['integ'],
                                          lo_a, hi_a, L5_ALT_TIME, L5_GAP, x0=xs, pscost=L5_PSCOST,
                                          nodes=max(L5_ALT_NODES, 1000), solver=ALT_SOLVER)
            if not ok_a or (xs is not x0 and float(inst['cvec'] @ xa) > float(inst['cvec'] @ xs) + 1e-9):
                xa, msg_a, ok_a = xs, 'per-net sweep', True
            t_a = _t.time() - t_s
            if not ok_a:
                _alog = log or self.log
                if os.environ.get('BRAID_L5_ALT_LOG'):
                    _alog = lambda m: print(m, file=sys.stderr)
                _alog(f'  profiles5 alts: stage A no solution ({msg_a}); {n_cand} candidate(s), '
                      f'{len(inst["rows"])} rows, seed {"feasible" if not n_viol else f"VIOLATES {n_viol} rows"}, '
                      f'{_t.time() - t0:.1f} s')
                return
            obj_a = float(inst['cvec'] @ xa)
            note = f'A {t_a:.1f} s {msg_a[:16]}' + (f' ({n_pn} per-net gains)' if L5_ALT_PERNET else '')
            x, msg = xa, msg_a
            if L5_ALT_NODES > 0:
                # stage B: everything free, from stage A's answer
                t_s = _t.time()
                xb, msg_b, ok_b = _milp_solve(inst['cvec'], inst['rows'], inst['lb'], inst['ub'], inst['integ'],
                                              np.zeros(nv), np.ones(nv), L5_ALT_TIME, L5_GAP,
                                              x0=xa, pscost=L5_PSCOST, nodes=L5_ALT_NODES, solver=ALT_SOLVER)
                note += f', B {_t.time() - t_s:.1f} s {msg_b[:16]}'
                if ok_b and float(inst['cvec'] @ xb) <= obj_a + 1e-9:
                    x, msg = xb, msg_b
            memo[akey] = (x, msg, obj_a)
        obj = float(inst['cvec'] @ x)
        if os.environ.get('BRAID_L5_ALT_DUMP'):
            # diagnostic: the choice instance, its seed and its solution
            import pickle
            base_ = os.environ['BRAID_L5_ALT_DUMP']
            n_d = len([k for k in os.listdir(os.path.dirname(base_) or '.')
                       if k.startswith(os.path.basename(base_))])
            with open(f'{base_}_{n_d}.pkl', 'wb') as fh:
                pickle.dump(dict(inst={k: v for k, v in inst.items() if k != 'prox'}, x0=x0, x=x, msg=msg,
                                 info=info, y_cost=y_cost, todo=list(todo), lanes=lanes,
                                 samp={k: {kk: vv for kk, vv in d.items() if kk in ('S', 'O', 'U', 'TAG', 'XY', 'room')}
                                       for k, d in samp2.items()},
                                 blk={k: {L: d['blk'][L] for L in d['blk']} for k, d in samp2.items()},
                                 hard={k: {L: d['hard'][L] for L in d['hard']} for k, d in samp2.items()},
                                 prox=inst['prox'], tl=dict(tl_of), dl=dict(dl_of), alts=dict(ctx.alts),
                                 s1=self.s1, s0=self.s0), fh)
        chosen = {}
        for nm in todo:
            js = [(float(x[idx[('y', (nm, j))]]), j) for j in range(len(ctx.alts[nm]) + 1)]
            chosen[nm] = max(js)[1]
        self.alt_choice = chosen
        self.alt_w = {nm: bool(x[idx[('w', nm)]] > 0.5) for nm in todo}
        self.alt_obj = (obj0, obj)
        res2 = [nm for nm in M if x[idx[('w', nm)]] > 0.5]
        n_ch = sum(1 for key, v in idx.items() if key[0] in ('up', 'dn') and x[v] > 0.5)
        moves = []
        for nm, j in sorted(chosen.items()):
            if j == 0:
                continue
            inf = info[(nm, j)]
            moves.append(f'{nm} -> #{j} ({"TOOTH " if inf.get("src") else ""}{inf["layer"][0]}, ' + ('head-on' if inf['head'] else
                         f'side {"+" if inf["side"] > 0 else "-"} leg s{inf["leg_s"]:.1f}')
                         + (', far' if inf['far'] else '') + f', o{inf["o_t"]:+.2f}'
                         + (', swims' if self.alt_w[nm] else '') + ')')
        _alog = log or self.log
        if os.environ.get('BRAID_L5_ALT_LOG'):
            _alog = lambda m: print(m, file=sys.stderr)
        _alog(
            f'  profiles5 alts: {len(todo)} residue net(s), {n_cand} candidate berth(s), '
            f'{inst["n_gated"]} gated lane(s), {len(excl)} exclusion(s), '
            f'{inst.get("n_pair", 0)} crossing pair(s), {inst["n_gkill"]} gated slot(s), '
            f'{len(inst["rows"])} rows, {nv} vars; seed {"feasible" if not n_viol else f"VIOLATES {n_viol} rows"}; '
            f'{note}; objective {obj0:.2f} -> A {obj_a:.2f} -> {obj:.2f}, '
            f'residue {len(todo) + sum(1 for nm in M if nm not in todo and x_plain[idx_plain[("w", nm)]] > 0.5)} '
            f'-> {len(res2)} {res2}, {n_ch} changes; {_t.time() - t0:.1f} s; '
            + (f'{len(moves)} moves: ' + '; '.join(moves) if moves else 'no move'))

    def _req_from_profile(self, sched):
        """The required stretches of every scheduled lane from its level-5
        profile: its layer between changes over the whole mid polyline
        (region and run), DIVE_GAP open around each change; a change at
        the line's start is the birth via at the launch slot."""
        M = self.members
        tl = self.ctx.tooth_layer
        GAP = DIVE_GAP
        req = {nm: [] for nm in M}
        for nm in M:
            if sched.page.get(nm) is None or nm not in self.full_prof:
                continue
            s_a = self._line_start(nm)
            a = s_a + 0.05
            b_end = (self.exit_leg_s[nm] if nm in self.exit_block else self.se[nm][0]) - 0.05
            L = self._line_L0(nm)
            cur = a
            for c in self.prof.get(nm, ()):
                if c <= s_a + GAP:
                    L = other_layer(L)
                    cur = a
                    continue
                hi = min(c - GAP, b_end)
                if hi > cur:
                    req[nm].append((cur, hi, L))
                L = other_layer(L)
                cur = c + GAP
            if b_end > cur:
                req[nm].append((cur, b_end, L))
        for nm in getattr(self, 'face_block', {}):
            req[nm].append((self.st[nm][0] - 0.05,
                            max(self.face_leg_s[nm] + 0.2, self.s0 + 0.06),
                            self.ctx.tooth_layer[nm]))
        return req

    def _line_L0(self, nm):
        """The layer a lane's region line begins on BEFORE its birth via,
        if any: the tooth's, or what a scheduled head left it at (level
        5 with HEAD_L5: prof_L0, the birth via being a mid change)."""
        return getattr(self, 'prof_L0', {}).get(nm, self.ctx.tooth_layer[nm])

    def _bwin_from_req(self, req, sched):
        """The possible back-layer windows, derived from the required
        stretches (see lay_lanes: the two can never disagree)."""
        bwin = {}
        for nm in self.members:
            wins = [(self.s1 - 0.1, 1e9)]
            if sched.page.get(nm) is None:
                wins.append((-1e9, 1e9))
            b_iv = sorted((xa, xb) for (xa, xb, L) in req[nm]
                          if L == 'B.Cu' and xa < self.s1 - 0.1)
            f_iv = sorted((xa, xb) for (xa, xb, L) in req[nm] if L == 'F.Cu')
            if b_iv:
                b0, b1 = b_iv[0][0], b_iv[-1][1]
                lo = max((xb for (xa, xb) in f_iv if xb <= b0), default=-1e9)
                hi = min((xa for (xa, xb) in f_iv if xa >= b1), default=1e9)
                wins.append((lo, hi))
            if self.ctx.tooth_layer[nm] == 'B.Cu':
                hi = min((xa for (xa, xb) in f_iv), default=1e9)
                wins.append((-1e9, hi))
            bwin[nm] = wins
        return bwin

    @staticmethod
    def _piece_ok(prof, T, ta, tb, L, gap):
        """Allowed mask for layer L along a leg or jog piece: `prof` =
        (layer at the piece's start, [(coord, layer after), ...]), T the
        cells' coordinate along the piece from ta (its start) to tb; both
        layers open within `gap` of a change."""
        L0, ch = prof
        ok = np.full(T.shape, L0 == L)
        sg = 1.0 if tb >= ta else -1.0
        for c, La in ch:
            beyond = (T - c) * sg > 0
            ok = np.where(beyond, La == L, ok)
        for c, _La in ch:
            ok |= np.abs(T - c) <= gap
        return ok

    def _profiles3(self, sched, M, pairs, line, tl, dl, log=None):
        """ONE_DIVE level 3: profiles on proximity and via slots. Lines
        sampled every PROF_DS along s over the region; a changer takes
        one change at a candidate position, a stayer a ride between two
        (or none), a net that fits neither swims. Candidates: the samples
        where no other line lies within VIA_NEED in o (a via's room), plus
        the region's ends. Legality: at every sample where two lines are
        within PROX_TRACK in o the two lanes are on different layers
        (relaxed for a swimmer). State along s is the prefix sum of the
        change binaries (a stayer's must stay in {0, 1}). Objective RIDE_W
        per ride + SWIM_W per swim. Self-checked afterwards."""
        from scipy.optimize import milp, LinearConstraint, Bounds
        from scipy.sparse import lil_matrix
        import time as _t
        t0 = _t.time()
        changers = [nm for nm in M if tl[nm] != dl[nm]]
        stayers = [nm for nm in M if tl[nm] == dl[nm]]
        s_lo = min(line[nm][0] for nm in M)
        s_hi = self.s1
        S = np.arange(s_lo, s_hi + 1e-9, PROF_DS)
        O = {}
        for nm in M:
            s_a, o_a, s_b, o_b = line[nm]
            o = o_a + (S - s_a) / max(s_b - s_a, 1e-9) * (o_b - o_a)
            o[S < s_a - 1e-9] = np.nan          # not yet launched (a joiner's leg)
            O[nm] = o
        # candidate change positions: via slots + the ends
        cand = {}
        for nm in M:
            d = np.full(S.shape, np.inf)
            for om in M:
                if om == nm:
                    continue
                dd = np.abs(O[nm] - O[om])
                dd[np.isnan(dd)] = np.inf
                d = np.minimum(d, dd)
            ok = (d >= VIA_NEED) & ~np.isnan(O[nm])
            ks = [k for k in range(len(S)) if ok[k]]
            # thin to every other sample (0.2 mm) and keep the ends
            ks = ks[::2]
            ends = [int(np.argmax(~np.isnan(O[nm]))), len(S) - 1]
            cand[nm] = sorted(set(ks) | set(ends))
        # proximity samples per pair
        prox = []
        for i, a_ in enumerate(M):
            for b_ in M[i + 1:]:
                dd = np.abs(O[a_] - O[b_])
                ks = np.where(~np.isnan(dd) & (dd < PROX_TRACK))[0]
                for k in ks:
                    prox.append((a_, b_, int(k)))
        idx = {}

        def var(key):
            if key not in idx:
                idx[key] = len(idx)
            return idx[key]
        for nm in M:
            var(('w', nm))
        for nm in stayers:
            var(('u', nm))
        for nm in M:
            for k in cand[nm]:
                var(('x1', nm, k))
                if nm in stayers:
                    var(('x2', nm, k))
        rows, lb, ub = [], [], []

        def add(co, lo, hi):
            rows.append(dict(co)); lb.append(lo); ub.append(hi)

        def state(nm, k):
            """{var: coef} for the lane's state at sample k: the prefix
            sum of its changes up to k (x1 - x2 for a stayer)."""
            co = {}
            for kk in cand[nm]:
                if kk <= k:
                    co[idx[('x1', nm, kk)]] = co.get(idx[('x1', nm, kk)], 0) + 1
                    if nm in stayers:
                        co[idx[('x2', nm, kk)]] = co.get(idx[('x2', nm, kk)], 0) - 1
            return co
        for nm in M:
            w = idx[('w', nm)]
            x1 = {idx[('x1', nm, k)]: 1 for k in cand[nm]}
            if nm in changers:
                # exactly one change unless swimming
                add({**x1, w: 1}, 1, 1)
            else:
                u = idx[('u', nm)]
                x2 = {idx[('x2', nm, k)]: 1 for k in cand[nm]}
                add({**x1, u: -1}, 0, 0)
                add({**x2, u: -1}, 0, 0)
                add({u: 1, w: 1}, -np.inf, 1)           # a swimmer has no ride
                for k in cand[nm]:
                    st_ = state(nm, k)
                    add(st_, 0, 1)                       # the ride opens before it closes
        for a_, b_, k in prox:
            wa, wb = idx[('w', a_)], idx[('w', b_)]
            sa, sb = state(a_, k), state(b_, k)
            co = dict(sa)
            for v_, c_ in sb.items():
                co[v_] = co.get(v_, 0) + c_
            if tl[a_] == tl[b_]:
                # exactly one flipped
                add({**co, wa: 4, wb: 4}, 1, np.inf)
                add({**co, wa: -4, wb: -4}, -np.inf, 1)
            else:
                # both flipped or neither
                co2 = dict(sa)
                for v_, c_ in sb.items():
                    co2[v_] = co2.get(v_, 0) - c_
                add({**co2, wa: -4, wb: -4}, -np.inf, 0)
                co3 = {v_: -c_ for v_, c_ in co2.items()}
                add({**co3, wa: -4, wb: -4}, -np.inf, 0)
        nv = len(idx)
        cvec = np.zeros(nv)
        integ = np.ones(nv)
        for key, v in idx.items():
            if key[0] == 'w':
                cvec[v] = SWIM_W
            elif key[0] == 'u':
                cvec[v] = RIDE_W
        memo = _PROFILE_MEMO       # per process: the judge re-plans identical geometry many times
        mkey = ('L3', tuple((nm, tl[nm], dl[nm], tuple(round(v, 4) for v in line[nm])) for nm in M),
                round(self.s1, 4), PROX_TRACK, RIDE_W, SWIM_W)
        if mkey in memo:
            x, msg = memo[mkey]
        else:
            if rows:
                A = lil_matrix((len(rows), nv))
                for i_, co in enumerate(rows):
                    for k_, v_ in co.items():
                        A[i_, k_] = v_
                cons = LinearConstraint(A.tocsr(), lb, ub)
            else:
                cons = LinearConstraint(lil_matrix((1, nv)).tocsr(), [-np.inf], [np.inf])
            res = milp(cvec, constraints=cons, integrality=integ, bounds=Bounds(0, 1),
                       options={'time_limit': 60})
            if res.x is None:
                (log or self.log)(f'  profiles3: no solution ({res.message}); LIS pages kept')
                return
            x, msg = res.x, res.message
            memo[mkey] = (x, msg)
        residue = [nm for nm in M if x[idx[('w', nm)]] > 0.5]
        page_ride = []
        s_end = float(S[-1])
        for nm in M:
            if nm in residue:
                sched.page[nm] = None
                continue
            ks1 = [k for k in cand[nm] if x[idx[('x1', nm, k)]] > 0.5]
            if nm in changers:
                self.dive[nm] = float(S[ks1[0]]) if ks1 else float(line[nm][0])
                sched.page[nm] = dl[nm]
            elif ks1:
                ks2 = [k for k in cand[nm] if x[idx[('x2', nm, k)]] > 0.5]
                c1, c2 = float(S[ks1[0]]), float(S[ks2[0]])
                if c1 <= line[nm][0] + PROF_DS and c2 >= s_end - PROF_DS:
                    page_ride.append(nm)
                    sched.page[nm] = other_layer(tl[nm])
                else:
                    self.ride[nm] = (c1, c2)
                    sched.page[nm] = tl[nm]
            else:
                sched.page[nm] = tl[nm]
        keep = {nm for nm in sched.launch if sched.page[nm] == 'F.Cu'}
        sched.b_page = [nm for nm in sched.launch if sched.page[nm] == 'B.Cu']
        sched.swimmers = [nm for nm in sched.launch if sched.page[nm] is None]
        sched.divers = {nm for nm in sched.launch if nm not in keep}
        sched.birth_b = {d for d in sched.divers if tl.get(d) == 'B.Cu'}
        self._od_pairs, self._od_residue, self._od_page_ride = list(pairs), list(residue), list(page_ride)

        def layer_at(nm, s_):
            if nm in self.dive:
                return tl[nm] if s_ < self.dive[nm] else dl[nm]
            if nm in self.ride:
                c1, c2 = self.ride[nm]
                return other_layer(tl[nm]) if c1 <= s_ <= c2 else tl[nm]
            if nm in page_ride:
                return other_layer(tl[nm])
            return tl[nm]
        bad = []
        for a_, b_, k in prox:
            if a_ in residue or b_ in residue:
                continue
            if layer_at(a_, float(S[k])) == layer_at(b_, float(S[k])):
                bad.append((a_, b_, round(float(S[k]), 2)))
        if bad:
            (log or self.log)(f'  profiles3: SELF-CHECK FAILED, {len(bad)} same-layer proximity sample(s): {bad[:6]}')
        stay = [nm for nm in M if nm not in residue and nm not in self.dive
                and nm not in self.ride and nm not in page_ride]
        n_slots = sum(len(cand[nm]) for nm in M)
        (log or self.log)(
            f'  profiles (ONE_DIVE=3): {len(stay)} stayers, {len(self.dive)} divers, '
            f'{len(page_ride)} page rides, {len(self.ride)} partial rides, '
            f'{len(residue)} residue {residue} ({len(prox)} proximity samples, {n_slots} via slots, '
            f'{len(rows)} rows, {_t.time() - t0:.1f} s, {msg[:40]}); '
            + 'changes: ' + ' '.join(f'{nm}@{c:.1f}' for nm, c in sorted(self.dive.items(), key=lambda kv: kv[1]))
            + ('; rides: ' + ' '.join(f'{nm}@{a:.1f}-{b:.1f}' for nm, (a, b) in sorted(self.ride.items(), key=lambda kv: kv[1]))
               if self.ride else ''))

    def _profiles(self, sched, M, pairs, line, tl, dl, r, log=None):
        """ONE_DIVE level 2: every net a layer PROFILE with 0, 1 or 2
        scheduled changes. A changer (tooth layer != berth layer) changes
        once at c; a stayer keeps its layer, or RIDES the other layer over
        [c1, c2] (two changes; c1 at the region's start and c2 at its end
        is the page scheme's back-page lane, and is laid by that code path
        exactly); a net that fits neither swims. Each inverted pair's
        crossing is fixed where the ribbon lines meet, and at every
        crossing the two lanes must be on different layers, DIVE_ROOM
        clear of any change. A MILP (scipy HiGHS): per crossing a binary
        per lane saying whether it is on its OTHER layer there, tied to
        the change points; objective RIDE_W per ride + SWIM_W per swim."""
        from scipy.optimize import milp, LinearConstraint, Bounds
        from scipy.sparse import lil_matrix
        import time as _t
        t0 = _t.time()
        BIGM = (self.s1 - self.s0) + 4.0
        changers = [nm for nm in M if tl[nm] != dl[nm]]
        stayers = [nm for nm in M if tl[nm] == dl[nm]]
        # memo: the same geometry (pairs, crossings, classes, bounds) has
        # the same solution -- the braid rebuilds its schedule several
        # times per attempt on identical offsets
        memo = _PROFILE_MEMO       # per process: the judge re-plans identical geometry many times
        mkey = (tuple((a_, b_, round(t, 4)) for a_, b_, t in pairs),
                tuple((nm, tl[nm], dl[nm], round(line[nm][0], 4)) for nm in M),
                round(self.s1, 4), r, RIDE_W, SWIM_W)
        idx = {}

        def var(key):
            if key not in idx:
                idx[key] = len(idx)
            return idx[key]
        for nm in M:
            var(('w', nm))
        for nm in changers:
            var(('c', nm))
        for nm in stayers:
            var(('u', nm)); var(('c1', nm)); var(('c2', nm))
        rows, lb, ub = [], [], []

        def add(co, lo, hi):
            rows.append(dict(co)); lb.append(lo); ub.append(hi)
        for nm in stayers:
            add({idx[('c1', nm)]: 1, idx[('c2', nm)]: -1}, -np.inf, 0.0)

        def stayer_z(k, nm, t):
            """The binary 'stayer nm is on its other layer at t', tied to
            its ride [c1, c2] (a second binary v says which side of the
            ride t lies on when it is outside)."""
            z = var(('z', k, nm))
            w = idx[('w', nm)]
            u, c1, c2 = idx[('u', nm)], idx[('c1', nm)], idx[('c2', nm)]
            v = var(('v', k, nm))
            add({z: 1, u: -1}, -np.inf, 0.0)                           # no ride, no flip
            add({c1: 1, z: BIGM, w: -BIGM}, -np.inf, t - r + BIGM)    # z=1: c1 <= t-r
            add({c2: 1, z: -BIGM, w: BIGM}, t + r - BIGM, np.inf)     # z=1: c2 >= t+r
            add({c1: 1, z: BIGM, v: BIGM, u: -BIGM, w: BIGM}, t + r - BIGM, np.inf)   # z=0,u=1,v=0: before the ride
            add({c2: 1, z: -BIGM, v: BIGM, u: BIGM, w: -BIGM}, -np.inf, t - r + 2 * BIGM)  # z=0,u=1,v=1: after it
            return z
        # Per pair: a changer's state at t is an interval on its change
        # point, so only a STAYER needs a per-crossing binary (the first
        # cut gave every lane one: 1026 rows and 7-12 s a solve at K28;
        # the braid builds its schedule ~8 times a run)
        for k, (a_, b_, t) in enumerate(pairs):
            wa, wb = idx[('w', a_)], idx[('w', b_)]
            ca, cb = ('c', a_) in idx, ('c', b_) in idx
            if ca and cb:
                sa, sb = idx[('c', a_)], idx[('c', b_)]
                y = var(('y', k))
                if tl[a_] == tl[b_]:
                    # one class: cross between the two changes (either order)
                    add({sa: 1, y: BIGM, wa: -BIGM, wb: -BIGM}, -np.inf, t - r + BIGM)   # y=1: s_a <= t-r
                    add({sb: 1, y: -BIGM, wa: BIGM, wb: BIGM}, t + r - BIGM, np.inf)     # y=1: s_b >= t+r
                    add({sb: 1, y: -BIGM, wa: -BIGM, wb: -BIGM}, -np.inf, t - r)         # y=0: s_b <= t-r
                    add({sa: 1, y: BIGM, wa: BIGM, wb: BIGM}, t + r, np.inf)             # y=0: s_a >= t+r
                else:
                    # opposite classes: before both changes, or after both
                    add({sa: 1, y: -BIGM, wa: BIGM, wb: BIGM}, t + r - BIGM, np.inf)
                    add({sb: 1, y: -BIGM, wa: BIGM, wb: BIGM}, t + r - BIGM, np.inf)
                    add({sa: 1, y: -BIGM, wa: -BIGM, wb: -BIGM}, -np.inf, t - r)
                    add({sb: 1, y: -BIGM, wa: -BIGM, wb: -BIGM}, -np.inf, t - r)
            elif ca or cb:
                ch, st_ = (a_, b_) if ca else (b_, a_)
                c = idx[('c', ch)]
                z = stayer_z(k, st_, t)
                if tl[st_] == tl[ch]:
                    # same birth layer: exactly one flipped -- z=0: the
                    # changer has changed (c <= t-r); z=1: it has not
                    # (c >= t+r)
                    add({c: 1, z: -BIGM, wa: -BIGM, wb: -BIGM}, -np.inf, t - r)
                    add({c: 1, z: -BIGM, wa: BIGM, wb: BIGM}, t + r - BIGM, np.inf)
                else:
                    # opposite birth layers: both flipped or neither --
                    # z=1: the changer has changed; z=0: it has not
                    add({c: 1, z: BIGM, wa: -BIGM, wb: -BIGM}, -np.inf, t - r + BIGM)
                    add({c: 1, z: BIGM, wa: BIGM, wb: BIGM}, t + r, np.inf)
            else:
                za, zb = stayer_z(k, a_, t), stayer_z(k, b_, t)
                if tl[a_] == tl[b_]:
                    # same birth layer: exactly one of them flipped at t
                    add({za: 1, zb: 1, wa: 4, wb: 4}, 1, np.inf)
                    add({za: 1, zb: 1, wa: -4, wb: -4}, -np.inf, 1)
                else:
                    # opposite birth layers: both flipped or neither
                    add({za: 1, zb: -1, wa: -4, wb: -4}, -np.inf, 0)
                    add({za: -1, zb: 1, wa: -4, wb: -4}, -np.inf, 0)
        nv = len(idx)
        lo_b, hi_b = np.zeros(nv), np.ones(nv)
        for key, v in idx.items():
            if key[0] in ('c', 'c1', 'c2'):
                lo_b[v] = line[key[1]][0]
                hi_b[v] = max(self.s1, lo_b[v])
        cvec = np.zeros(nv)
        integ = np.zeros(nv)
        for key, v in idx.items():
            if key[0] == 'w':
                cvec[v] = SWIM_W; integ[v] = 1
            elif key[0] == 'u':
                cvec[v] = RIDE_W; integ[v] = 1
            elif key[0] in ('z', 'v', 'y'):
                integ[v] = 1
        if rows:
            A = lil_matrix((len(rows), nv))
            for i_, co in enumerate(rows):
                for k_, v_ in co.items():
                    A[i_, k_] = v_
            cons = LinearConstraint(A.tocsr(), lb, ub)
        else:
            cons = LinearConstraint(lil_matrix((1, nv)).tocsr(), [-np.inf], [np.inf])
        if mkey in memo:
            x, msg = memo[mkey]
        else:
            res = milp(cvec, constraints=cons, integrality=integ, bounds=Bounds(lo_b, hi_b),
                       options={'time_limit': 60})
            if res.x is None:
                (log or self.log)(f'  profiles: no solution ({res.message}); LIS pages kept')
                return
            x, msg = res.x, res.message
            memo[mkey] = (x, msg)
        residue = [nm for nm in M if x[idx[('w', nm)]] > 0.5]
        self._od_pairs, self._od_residue = list(pairs), list(residue)   # for the residue probe
        page_ride = []
        for nm in M:
            if nm in residue:
                sched.page[nm] = None
            elif ('c', nm) in idx:
                self.dive[nm] = float(x[idx[('c', nm)]])
                sched.page[nm] = dl[nm]
            elif x[idx[('u', nm)]] > 0.5:
                c1, c2 = float(x[idx[('c1', nm)]]), float(x[idx[('c2', nm)]])
                if c1 <= line[nm][0] + r and c2 >= self.s1 - r:
                    page_ride.append(nm)             # the page scheme's own lane
                    sched.page[nm] = other_layer(tl[nm])
                else:
                    self.ride[nm] = (c1, c2)
                    sched.page[nm] = tl[nm]
            else:
                sched.page[nm] = tl[nm]
        keep = {nm for nm in sched.launch if sched.page[nm] == 'F.Cu'}
        sched.b_page = [nm for nm in sched.launch if sched.page[nm] == 'B.Cu']
        sched.swimmers = [nm for nm in sched.launch if sched.page[nm] is None]
        sched.divers = {nm for nm in sched.launch if nm not in keep}
        sched.birth_b = {d for d in sched.divers if tl.get(d) == 'B.Cu'}

        def layer_at(nm, s_):
            if nm in self.dive:
                return tl[nm] if s_ < self.dive[nm] else dl[nm]
            if nm in self.ride:
                c1, c2 = self.ride[nm]
                return other_layer(tl[nm]) if c1 <= s_ <= c2 else tl[nm]
            if nm in page_ride:
                return other_layer(tl[nm])
            return tl[nm]
        bad = []
        for a_, b_, t in pairs:
            if a_ in residue or b_ in residue:
                continue
            if layer_at(a_, t) == layer_at(b_, t):
                bad.append((a_, b_, round(t, 2)))
            for nm in (a_, b_):
                pts = ([self.dive[nm]] if nm in self.dive else list(self.ride.get(nm, ())))
                if any(abs(c - t) < r - 1e-6 for c in pts):
                    bad.append((nm, 'change within room of crossing', round(t, 2)))
        if bad:
            def prof(nm):
                if nm in self.dive:
                    return f'{nm} diver {tl[nm][0]}->{dl[nm][0]} c={self.dive[nm]:.2f}'
                if nm in self.ride:
                    return f'{nm} rider {tl[nm][0]} ride {self.ride[nm][0]:.2f}-{self.ride[nm][1]:.2f}'
                if nm in page_ride:
                    return f'{nm} page-ride {tl[nm][0]}->{other_layer(tl[nm])[0]}'
                return f'{nm} stayer {tl[nm][0]}'
            (log or self.log)(f'  profiles: SELF-CHECK FAILED, {len(bad)} illegal crossing(s): {bad[:6]}; '
                              + '; '.join(f'[{prof(b[0])} x {prof(b[1])} @ {b[2]}]' for b in bad[:4] if b[1] != 'change within room of crossing'))
        stay = [nm for nm in M if nm not in residue and nm not in self.dive
                and nm not in self.ride and nm not in page_ride]
        self._od_page_ride = list(page_ride)
        (log or self.log)(
            f'  profiles (ONE_DIVE=2): {len(stay)} stayers, {len(self.dive)} divers, '
            f'{len(page_ride)} page rides, {len(self.ride)} partial rides, '
            f'{len(residue)} residue {residue} ({len(pairs)} crossings, {len(rows)} rows, '
            f'{_t.time() - t0:.1f} s, {msg[:40]}); '
            + 'changes: ' + ' '.join(f'{nm}@{c:.1f}' for nm, c in sorted(self.dive.items(), key=lambda kv: kv[1]))
            + ('; rides: ' + ' '.join(f'{nm}@{a:.1f}-{b:.1f}' for nm, (a, b) in sorted(self.ride.items(), key=lambda kv: kv[1]))
               if self.ride else ''))

    def _swim_hold(self, nm, sched):
        """(c1, c2) for swimmer `nm`'s hold-then-run line (SWIM_HOLD),
        or None when no polyline beats its straight diagonal. Every page
        lane is its ribbon diagonal (launch slot at s0 to target slot at
        s1); the swimmer's crossings with the page lanes it is inverted
        with each force the layer opposite that lane's page; the cost of
        a line is the number of layer changes in that sequence (tooth
        layer first, berth layer last) plus 10 for every change with less
        than SWIM_ROOM of s to make it in. Swimmer-swimmer crossings are
        no constraint (they route last, against each other's copper)."""
        s0, s1 = self.s_of_u(0.0), self.s1
        o_l, o_t = self.launch_o[nm], self.target_o[nm]
        tl, dl = self.ctx.tooth_layer[nm], self.ctx.dest_layer[nm]
        lanes = []
        for om in self.members:
            P = sched.page.get(om)
            if om == nm or P is None or not sched.inverted(nm, om):
                continue
            s_a = self.face_start.get(om, s0)      # a face-side lane's line starts at its launch point
            lanes.append((self.launch_o[om], self.target_o[om], om, s_a, max(s1 - s_a, 1e-6)))
        if not lanes or s1 - s0 < 0.5:
            return None
        L_ = s1 - s0

        def cost(poly):
            seq = [(s0 - SWIM_ROOM, tl)]
            for (a, b, om, s_a, L_a) in lanes:
                for (sa, oa), (sb, ob) in zip(poly, poly[1:]):
                    if sb <= sa + 1e-9:
                        continue
                    la = a + (b - a) * (sa - s_a) / L_a
                    lb = a + (b - a) * (sb - s_a) / L_a
                    da, db = oa - la, ob - lb
                    if da * db < 0:
                        x = sa + (sb - sa) * da / (da - db)
                        seq.append((x, self._need_at(om, x, sched)))
                    elif da == 0.0 and db != 0.0:
                        seq.append((sa, self._need_at(om, sa, sched)))
            seq.sort()
            seq.append((s1 + SWIM_ROOM, dl))
            c = 0
            for (sa, La), (sb, Lb) in zip(seq, seq[1:]):
                if La != Lb:
                    c += 1 + (10 if sb - sa < SWIM_ROOM else 0)
            return c
        straight = cost([(s0, o_l), (s1, o_t)])
        if straight <= 1:
            return None
        need = abs(o_t - o_l) / SWIM_SLOPE
        best = (straight, None)
        grid = np.arange(s0 + 0.1, s1 - 0.1 + 1e-9, 0.1)
        for c1 in grid:
            for c2 in grid:
                if c2 - c1 < need - 1e-9:
                    continue
                poly = [(s0, o_l), (float(c1), o_l), (float(c2), o_t), (s1, o_t)]
                c = cost(poly)
                if c < best[0]:
                    best = (c, (float(c1), float(c2)))
        return best[1]

    def _swim_hold2(self, nm, sched):
        """SWIM_HOLD level 2: (o_h, c1, c2) -- the swimmer moves from its
        launch slot to a HOLD offset o_h just after s0, holds it to c1,
        runs to its target offset by c2 (which may lie in the TAIL, past
        s1, for a head-on swimmer: the human's west-face nets ride the
        front bundle's edge on F and cross the whole bundle in one steep
        B run where the bundle's tails converge on their stubs), and
        holds the target to the stub / the slot. Cost as _swim_hold, the
        page lanes modelled as ribbon lines in [s0, s1] and, in the tail,
        head-on lanes as their tail run on their berth layer and block
        lanes at their slot on their page until their leg. No slope cap:
        the via-room term prices a steep run that crosses lanes needing
        different layers, and a steep run across lanes that all need the
        SAME layer is exactly the human's."""
        s0, s1 = self.s_of_u(0.0), self.s1
        o_l, o_t = self.launch_o[nm], self.target_o[nm]
        tl, dl = self.ctx.tooth_layer[nm], self.ctx.dest_layer[nm]
        head_on = nm not in self.exit_block
        s_e = self.se[nm][0]
        s_end = (min(s_e - 0.3, s1 + 1.0) if head_on else s1)
        L_ = s1 - s0
        lanes = []
        for om in self.members:
            P = sched.page.get(om)
            if om == nm or P is None or not sched.inverted(nm, om):
                continue
            need = 'B.Cu' if P == 'F.Cu' else 'F.Cu'
            # the lane's line: ribbon over [s0, s1] (a face lane's from
            # its leg), then its tail
            poly = [(self.face_start.get(om, s0), self.launch_o[om]),
                    (s1, self.target_o[om])]
            if om in self.exit_block:
                s_leg = self.exit_leg_s.get(om, self.se[om][0])
                poly.append((max(s1, s_leg), self.target_o[om]))
                need_tail = need
            else:
                poly.append((self.se[om][0], self.se[om][1]))
                need_tail = 'B.Cu' if self.ctx.dest_layer[om] == 'F.Cu' else 'F.Cu'
            lanes.append((poly, need, need_tail))
        if not lanes or L_ < 0.5 or s_end <= s0 + 0.5:
            return None

        def cross(poly_a, poly_b):
            """(s, k) of every proper crossing of two (s, o) polylines:
            k orders crossings that share one s -- a steep run meets its
            lanes one after another along o, in the run's direction --
            so a vertical run across lanes needing alternate layers is
            priced as the alternation it is (sorted by layer name it read
            as one change, and the first search chose c1 = c2 = s0)."""
            out = []
            for (sa, oa), (sb, ob) in zip(poly_a, poly_a[1:]):
                if sb <= sa + 1e-9:
                    continue
                sgn = 1.0 if ob >= oa else -1.0
                for (ta, pa), (tb, pb) in zip(poly_b, poly_b[1:]):
                    if tb <= ta + 1e-9:
                        continue
                    lo, hi = max(sa, ta), min(sb, tb)
                    if hi <= lo:
                        continue
                    oa_ = oa + (ob - oa) * (lo - sa) / (sb - sa)
                    ob_ = oa + (ob - oa) * (hi - sa) / (sb - sa)
                    pa_ = pa + (pb - pa) * (lo - ta) / (tb - ta)
                    pb_ = pa + (pb - pa) * (hi - ta) / (tb - ta)
                    da, db = oa_ - pa_, ob_ - pb_
                    if da * db < 0:
                        t = da / (da - db)
                        out.append((lo + (hi - lo) * t, sgn * (pa_ + (pb_ - pa_) * t)))
                    elif da == 0.0 and db != 0.0:
                        out.append((lo, sgn * pa_))
            return out

        def cost(poly):
            seq = [(s0 - SWIM_ROOM, -1e9, tl)]
            for (lp, need, need_tail) in lanes:
                for s, k in cross(poly, lp):
                    seq.append((s, k, need if s <= s1 + 1e-9 else need_tail))
            seq.sort()
            seq.append((s_end + SWIM_ROOM, 1e9, dl))
            c = 0
            for (sa, _ka, La), (sb, _kb, Lb) in zip(seq, seq[1:]):
                if La != Lb:
                    c += 1 + (10 if sb - sa < SWIM_ROOM else 0)
            return c
        tail_pt = [(s_e, self.se[nm][1])] if head_on else []
        straight = cost([(s0, o_l), (s1, o_t)] + tail_pt)
        if straight <= 1:
            return None
        best = (straight, None)
        g1 = np.arange(s0 + 0.4, s1 + 1e-9, 0.1)
        g2 = np.arange(s0 + 0.5, s_end + 1e-9, 0.1)
        for k in range(-3, 4):
            o_h = o_l + k * LPITCH
            for c1 in g1:
                for c2 in g2:
                    if c2 < c1 + 0.1 - 1e-9:
                        # the run needs extent in s: a piece of zero
                        # s-length crosses nothing in cross() and priced
                        # a vertical jump across the whole bundle at 0
                        continue
                    poly = [(s0, o_l), (s0 + 0.3, o_h), (float(c1), o_h), (float(c2), o_t)]
                    if head_on and c2 < s_e - 1e-9:
                        poly.append((s_e, self.se[nm][1]))
                    elif not head_on:
                        poly.append((s1, o_t))
                    poly = [q for i, q in enumerate(poly) if i == 0 or q[0] > poly[i - 1][0] - 1e-9]
                    c = cost(poly)
                    if c < best[0]:
                        best = (c, (float(o_h), float(c1), float(c2)))
        return best[1]

    def _flank_stubs(self):
        """Members whose stub stands BESIDE its destination array, on a
        face that runs ALONG the spine: {net: side}, side = the sign of
        the stub's offset from the array's centre. Measured in the
        ARRAY's own frame (the footprint's rotation, its pads' local
        bounding box): the stub is beyond one of the four face lines by
        FLANK_MIN, and that face's direction on the board is within 45
        degrees of the spine's. Not in the spine's frame -- a spine
        tilted 4 degrees to a 12 mm face drifts 0.9 mm across it, so
        the ball field's o-extent (two far corners) and any s-window
        misread the face's ends (a stub 1.0 mm outside the east end of
        the north face read 0.05 mm inside the west corner's extreme;
        a window a pitch wide in s missed the tilted first column's
        north half and called the west face's stubs flank). A stub in
        front of the array or beyond it is on a face that crosses the
        spine; one inside the field (a band stub, a via site) is beyond
        no face."""
        ctx, sp = self.ctx, self.spine
        dn = sp.d[-1]
        out, cache = {}, {}
        for nm in self.members:
            ref = ctx.ends[nm][2]
            if ref not in cache:
                fp = ctx.pcb.footprints[ref]
                rot = fp.rotation or 0.0
                loc = [_global_to_local(fp.x, fp.y, rot, p.global_x, p.global_y)
                       for p in fp.pads]
                c = (sum(p.global_x for p in fp.pads) / len(fp.pads),
                     sum(p.global_y for p in fp.pads) / len(fp.pads))
                cache[ref] = (fp, rot, (min(l[0] for l in loc), min(l[1] for l in loc),
                                        max(l[0] for l in loc), max(l[1] for l in loc)),
                              sp.project_pt(c)[1])
            fp, rot, (x0, y0, x1, y1), o_c = cache[ref]
            lx, ly = _global_to_local(fp.x, fp.y, rot, *ctx.ends[nm][1])
            beyond_y = max(y0 - ly, ly - y1)     # beyond a face running along local x
            beyond_x = max(x0 - lx, lx - x1)     # beyond a face running along local y
            if max(beyond_x, beyond_y) < FLANK_MIN:
                continue
            r = math.radians(rot)
            # the face line's direction on the board (local_to_global's
            # rotation: local x -> (cos, -sin), local y -> (sin, cos))
            fd = (math.cos(r), -math.sin(r)) if beyond_y >= beyond_x \
                else (math.sin(r), math.cos(r))
            if abs(fd[0] * dn[0] + fd[1] * dn[1]) < 0.7071:
                continue                          # a face that crosses the spine
            out[nm] = 1 if self.se[nm][1] >= o_c else -1
        return out

    def _face_cross(self):
        """FACE_SORT: ({net: target side} for every head-on-launched lane
        born on the layer the source fanout leaves free whose target lies
        on a side of the destination, the CROSSERS among them -- teeth
        inside the front bundle's launch span by more than a pitch). The
        bundle layer is the tooth layer of most head-on launches; a lane
        on the other layer runs along the source face for free."""
        self.face_edge = None
        if not FACE_SORT or not self.heads_l:
            return {}, set()
        ctx, st, se = self.ctx, self.st, self.se
        tl = ctx.tooth_layer
        bl = Counter(tl[nm] for nm in self.heads_l).most_common(1)[0][0]
        Fo = [st[nm][1] for nm in self.heads_l if tl[nm] == bl]
        if len(Fo) < 2:
            return {}, set()
        lo, hi = min(Fo), max(Fo)
        self.face_edge = (lo, hi)
        # a head-on exit's side: a stub beside the destination array on a
        # face along the spine (_flank_stubs, in the array's own frame);
        # a stub in front of the array has no side
        flank = self._flank_stubs()
        side, cross = {}, set()
        for nm in self.heads_l:
            if tl[nm] == bl:
                continue
            o_t = st[nm][1]
            sg = self.exit_side[nm] if nm in self.siders else flank.get(nm, 0)
            if sg == 0:
                continue
            side[nm] = sg
            # a CROSSER: more than half the bundle's teeth lie between its
            # tooth and its target's side (SA0 at U1's south face bound
            # north: 15 of 21); a tooth near the bundle's edge on its own
            # side (SA1, 2 of 21) launches from where it is
            between = sum(1 for v in Fo if sg * v > sg * o_t)
            if 2 * between > len(Fo):
                cross.add(nm)
        return side, cross

    def _face_comb(self, launch_o, target_o, sched):
        """FACE_SORT: per side, the other-layer launch comb in target
        order (outermost target first), one pitch apart, a native tooth
        keeping its place when it already stands a pitch inside the slot
        before it; every lane more than half a pitch from its slot takes
        a leg along the face on its own layer (_leg_s places it off the
        free ends and the legs before it; static copper on that layer
        moves it a pitch downstream), the outermost slot's leg first so
        the legs nest. The ribbon starts past the last leg."""
        self.face_block, self.face_leg_s, self.face_start = {}, {}, {}
        _plus = float(os.environ.get('BRAID_S0_PLUS', '0') or 0)
        if _plus > 0:
            # DIAGNOSTIC (2026-09-10): move the ribbon start alone, to
            # separate what a shorter region costs from what the face
            # comb costs (the K51 bench: every south joiner refused under
            # the comb, whose legs move s0 by 0.95 mm)
            self.s0 += _plus
            self.log(f'  s0 += {_plus:.2f} (BRAID_S0_PLUS) -> {self.s0:.2f}')
        if not getattr(self, 'face_side', None):
            return
        ctx, st, sp = self.ctx, self.st, self.spine
        tl = ctx.tooth_layer
        # the legs are parallel same-layer runs: a legal minimum apart in s
        # (track + clearance + a hair), not a lane pitch -- every tenth of
        # s the legs take moves the ribbon start, and a region 0.95 mm
        # shorter steepened every diagonal at K51 (the plan probe of
        # 2026-09-10: south join lanes at slope 1.8, 0.17 mm across)
        leg_gap = TRACK + CLEAR + 0.03
        s_max = None
        lines = []
        for sg in (-1, 1):
            lanes = [nm for nm, s_ in self.face_side.items() if s_ == sg]
            cross = [nm for nm in lanes if nm in self.face_cross]
            if not cross:
                continue
            natives = [nm for nm in lanes if nm not in self.face_cross]
            # a native keeps its tooth; a crosser is inserted by target
            # rank: one pitch inside the innermost launch (a native's
            # tooth, or a crosser's slot) whose target lies outside its
            # own, else one pitch outside the outermost launch of the side
            # (a side with no native: a pitch beyond the bundle's edge)
            cross.sort(key=lambda nm: -sg * target_o[nm])
            # the ribbon will start past the last leg (below): the slot
            # pitch is a pitch across the diagonal the crosser will run,
            # LPITCH times the secant of its slope over that region --
            # at 0.35 in o, three diagonals at slope 1.5 lay 0.19 mm
            # apart (v1 on the bench: ALL BLOCKED between them)
            L_est = max(self.s1 - (max(st[n][0] for n in cross) + leg_gap), 0.5)
            slot = {}
            # a native's place is its LAUNCH slot (the relaxed comb), not
            # its tooth: SA1's tooth at -4.55 relaxed to -4.30, and a
            # slot a pitch inside the tooth stood 0.11 mm from the line
            for nm in cross:
                outer = ([launch_o[n] for n in natives if sg * target_o[n] > sg * target_o[nm]]
                         + [slot[n] for n in slot if sg * target_o[n] > sg * target_o[nm]])
                if outer:
                    ref, dr = min(outer, key=lambda v: sg * v), -1.0
                else:
                    launches = [launch_o[n] for n in natives] + list(slot.values())
                    if launches:
                        ref, dr = max(launches, key=lambda v: sg * v), 1.0
                    else:
                        ref, dr = (self.face_edge[0] if sg < 0 else self.face_edge[1]), 1.0
                pos = ref + dr * sg * LPITCH
                m = abs(target_o[nm] - pos) / L_est
                pos = ref + dr * sg * LPITCH * math.sqrt(1.0 + m * m)
                slot[nm] = pos
            # the legs start a legal minimum past the last END ON THEIR OWN
            # LAYER within their o-span (the front teeth are on the other
            # layer; the far natives' teeth are outside the span): from
            # the last tooth of ANY lane the first leg stood 0.4 mm later
            # and the ribbon start moved with it (K51: s0 8.75 -> 9.43)
            L_leg = tl[cross[0]]
            o_lo = min(min(slot[n], st[n][1]) for n in cross) - LPITCH
            o_hi = max(max(slot[n], st[n][1]) for n in cross) + LPITCH
            s_lead = max([st[n][0] for n in self.members
                          if tl[n] == L_leg and o_lo <= st[n][1] <= o_hi]
                         + [st[n][0] for n in cross]) + leg_gap
            # the side's whole back-layer family runs its diagonal from its
            # launch point -- a native from its tooth, a crosser from its
            # leg -- not from s0: the ribbon starts past the legs (below),
            # and from there the far-face lanes' diagonals (SA8 -7.5 to
            # -12.9 at K51) ran at slope 1.9, a pitch apart in o and
            # 0.16 mm apart across (ALL BLOCKED); the human's north family
            # fans out from its drops at s 7.3-9.3 at slope 0.86
            # (FACE_EARLY: every diagonal from its launch point. Measured
            # on the K51 bench with the region restored: a native's
            # diagonal leaving its tooth runs into a neighbour's head
            # going the other way -- SA12 into SA10's, ALL BLOCKED at the
            # tooth -- so with the region intact every lane holds its slot
            # to s0 as the ribbon always did, the slots a secant apart.)
            if FACE_EARLY:
                for nm in natives:
                    self.face_start[nm] = st[nm][0]
            placed = []
            for nm in sorted(cross, key=lambda n: -sg * slot[n]):
                pos, o_t = slot[nm], st[nm][1]
                if abs(pos - o_t) < LPITCH / 2:
                    if FACE_EARLY:
                        self.face_start[nm] = st[nm][0]
                    continue
                s_l = self._leg_s(nm, s_lead + leg_gap * len(placed), o_t, pos, True, placed,
                                  layer_only=True)
                L = tl[nm]
                for _k in range(6):
                    a, b = sp.xy(s_l, o_t), sp.xy(s_l, pos)
                    if ctx.obs_but(nm, self.members, L).seg_clear(a, b):
                        break
                    s_l += LPITCH
                self.face_leg_s[nm] = s_l
                self.face_block[nm] = pos
                if FACE_EARLY:
                    self.face_start[nm] = s_l
                launch_o[nm] = pos
                placed.append((s_l, min(o_t, pos), max(o_t, pos)))
                s_max = s_l if s_max is None else max(s_max, s_l)
                lines.append(f'{nm} {o_t:+.2f}->{pos:+.2f}@s{s_l:.2f}')
        if lines:
            # the ribbon starts past the last leg: a front-born page-B
            # lane dives AT s0 (its page requirement starts there), so
            # its B copper is real from s0 on and a leg after s0 would
            # cross it (K51: SDQ4's and SDQ0's B lines walled SA0's leg
            # 0.05 mm past s0)
            msg = '  face comb (BRAID_FACE_SORT): ' + ', '.join(lines)
            if s_max is not None and s_max + 0.2 > self.s0:
                self.s0 = s_max + 0.2
                msg += f'; s0 -> {self.s0:.2f}'
            self.log(msg)

    def _band_of(self, nm):
        """The band of its destination array this member's stub ends in
        (setup's dest_bands; empty unless SPLIT_BLOCKS), or None."""
        bands = getattr(self.ctx, 'dest_bands', {}).get(self.ctx.ends[nm][2], ())
        if not bands:
            return None
        x, y = self.ctx.ends[nm][1]
        for band in bands:
            x0, y0, x1, y1 = band
            if x1 - x0 >= y1 - y0:
                if y0 < y < y1 and x0 - 0.5 <= x <= x1 + 0.5:
                    return band
            elif x0 < x < x1 and y0 - 0.5 <= y <= y1 + 0.5:
                return band
        return None

    def _in_band(self, nm):
        return self._band_of(nm) is not None

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
            n = sum(1 for p in (ends_L if layer_only else ends)
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
        sec = min(math.sqrt(1.0 + m * m), SEC_CAP)
        if pa == pb:
            return base * sec
        tl, dl = self.ctx.tooth_layer, self.ctx.dest_layer
        via = any((tl if at_launch else dl)[nm] != sched.page[nm] for nm in (a, b))
        # ...or a change the SCHEDULE puts at the slot itself (a level-4
        # lane changing at the region's start or end: K15 SDQ15's ride
        # ending at s1 put a via beside SDQ13's target slot at the plain
        # pitch and squeezed its band to 0.05 mm)
        sv = getattr(self, 'slot_via', {})
        via = via or any(sv.get(nm, (False, False))[0 if at_launch else 1] for nm in (a, b))
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
        self.face_side, self.face_cross = self._face_cross()
        hl = sorted((nm for nm in self.heads_l if nm not in self.face_cross),
                    key=lambda nm: st[nm][1])
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
            # (JOIN_FLOOR: the block's pitch as each pair's own floor, the
            # exit block's rule. Tried when a shortened region ran two
            # joiners' diagonals 0.17 mm apart; with the region restored it
            # made the joined lanes CONVERGE onto the exit block's tighter
            # pitch -- K51 SRAS walled by SWE and SA13 at s 11.5 -- so the
            # recorded LPITCH stands.)
            offs = [0.0]
            for k in range(1, len(js)):
                offs.append(offs[-1] + (self.pair_floor(js[k - 1], js[k], LPITCH, sched, True)
                                        if JOIN_FLOOR else LPITCH))
            for k, nm in enumerate(js):
                launch_o[nm] = base + sg * (offs[-1] - offs[k])
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
        band_xs = [nm for nm in self.siders if self._in_band(nm)]
        for sg in (-1, 1):
            xs = [nm for nm in self.siders
                  if self.exit_side[nm] == sg and nm not in band_xs]
            if not xs:
                continue
            ports = sorted((nm for nm in xs if nm not in self.join_block),
                           key=lambda nm: se[nm][0])
            joined = sorted((nm for nm in xs if nm in self.join_block),
                            key=lambda nm: -st[nm][0])
            order = ports + joined
            if FLANK_COMB >= 2:
                # the comb along the face for every lane of the block:
                # first exiter innermost, joined or not
                order = sorted(xs, key=lambda nm: (se[nm][0], -st[nm][0]))
            if B_OUTER and sched is not None:
                # back-page lanes outermost (see B_OUTER), stable within
                # each group; a swimmer stays with the front group
                order = ([nm for nm in order if sched.page.get(nm) != 'B.Cu']
                         + [nm for nm in order if sched.page.get(nm) == 'B.Cu'])
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
        # BAND exits (a banded destination, escape_moves.blocks_of): side
        # exits whose stubs end INSIDE the array, in the street between two
        # of its blocks. Their block cannot lie beyond the head-on exits on
        # their side -- that is the next block's ball field -- it lies in
        # the band: ONE comb between the two stub-tip lines. The lanes in a
        # band nest -- a lane turning off to the north line at column c
        # crosses every lane north of it that continues past c -- so the
        # north side's exiters run from the north tips inward in exit
        # order (first exiter nearest its stubs: no leg crosses a lane
        # still present), the south side's from the south tips inward
        # likewise, the two runs meeting in the middle. When the comb does
        # not fit at the block pitch the pitch is compressed and said so:
        # the plan over-subscribed the band (select_moves.band_capacity).
        if band_xs:
            by_side = {sg: sorted((nm for nm in band_xs if self.exit_side[nm] == sg),
                                  key=lambda nm: se[nm][0]) for sg in (-1, 1)}
            # the band's two tip lines in the spine's frame, from the band's
            # GEOMETRY (both bound the comb whether or not a side has
            # exiters: a comb bounded by its own stubs alone ran onto the
            # other block's ball row): the ball lines projected at the
            # exiters' mean position, each stepped a tip's length into the
            # band. The lo side is the one whose stubs lie at lower o.
            sp = self.spine
            band = self._band_of(band_xs[0])
            tip = self.ctx.dest_band_tip.get(self.ctx.ends[band_xs[0]][2], 0.45)
            mx = sum(self.ctx.ends[nm][1][0] for nm in band_xs) / len(band_xs)
            my = sum(self.ctx.ends[nm][1][1] for nm in band_xs) / len(band_xs)
            x0, y0, x1, y1 = band
            if x1 - x0 >= y1 - y0:
                la, lb = sp.project_pt((mx, y0))[1], sp.project_pt((mx, y1))[1]
            else:
                la, lb = sp.project_pt((x0, my))[1], sp.project_pt((x1, my))[1]
            lo_line, hi_line = min(la, lb), max(la, lb)
            lo_sg = -1 if all(se[nm][1] <= (lo_line + hi_line) / 2 for nm in by_side[-1]) \
                else 1
            tips = {lo_sg: lo_line + tip, -lo_sg: hi_line - tip}
            lo = lo_line + tip + BAND_GAP
            hi = hi_line - tip - BAND_GAP

            def pack(order, start, sgn):
                """Slots for one side's exiters from its tip line inward
                (`sgn` +1 growing toward hi, -1 toward lo). First pass (no
                schedule): the block pitch. With the schedule, lanes on
                DIFFERENT pages may share the band's width (nothing but a
                via separates them, and a band lane's leg runs on its own
                page -- place_and_decide -- so no via sits at a slot): a
                lane sits a block pitch from the last lane of its OWN page
                and a hair (W_XING, the two-page column pitch) from the
                lane before it, so the first pass's order stands; a
                swimmer keeps the pitch from everyone."""
                out = []
                last = {}
                o = start
                for k, nm in enumerate(order):
                    pg = sched.page.get(nm) if sched is not None else None
                    if sched is None:
                        step = LPITCH if k else 0.0
                        o = start + sgn * LPITCH * k
                    elif k:
                        m_ = abs(self._slope.get(nm, 0.0))
                        sec = math.sqrt(1.0 + m_ * m_)
                        need = [o + sgn * W_XING]
                        if pg is None:
                            need += [v + sgn * LPITCH * sec for v in last.values()]
                        else:
                            if pg in last:
                                need.append(last[pg] + sgn * LPITCH * sec)
                            if None in last:
                                need.append(last[None] + sgn * LPITCH * sec)
                        o = max(need) if sgn > 0 else min(need)
                    out.append(o)
                    if sched is not None:
                        if pg is None:
                            for key in list(last) + [None]:
                                last[key] = o
                        else:
                            last[pg] = o
                return out
            lo_pos = pack(by_side[lo_sg], lo, +1)
            hi_pos = pack(by_side[-lo_sg], hi, -1)
            # the two combs meet in the middle; where they overlap the band
            # is over-subscribed (select_moves.band_capacity): both are
            # compressed toward their tip lines so the order stands
            lo_end = lo_pos[-1] if lo_pos else lo
            hi_end = hi_pos[-1] if hi_pos else hi
            room = hi - lo
            span = (lo_end - lo) + (hi - hi_end)
            self.band_over = set()
            if hi_end < lo_end - 1e-9 and span > 0:
                if sched is not None:
                    self.log(f'  band comb: {len(lo_pos) + len(hi_pos)} lanes need '
                             f'{span:.2f} mm, the band has {room:.2f} between its tip '
                             f'lines -- pitch compressed x{room / span:.2f} (the plan '
                             f'over-subscribed the band)')
                f_ = room / span
                self.band_over = ({nm for nm, o in zip(by_side[lo_sg], lo_pos) if o > hi}
                                  | {nm for nm, o in zip(by_side[-lo_sg], hi_pos) if o < lo})
                lo_pos = [lo + (o - lo) * f_ for o in lo_pos]
                hi_pos = [hi - (hi - o) * f_ for o in hi_pos]
            for nm, o in zip(by_side[lo_sg], lo_pos):
                target_o[nm] = o
                self.exit_block[nm] = o
            for nm, o in zip(by_side[-lo_sg], hi_pos):
                target_o[nm] = o
                self.exit_block[nm] = o
            o = lo_end
            comb = by_side[lo_sg] + by_side[-lo_sg][::-1]
            self.log(f'  band comb: lo side {by_side[lo_sg]} at o '
                     f'{", ".join(f"{v:.2f}" for v in lo_pos)}; hi side {by_side[-lo_sg]} at o '
                     f'{", ".join(f"{v:.2f}" for v in hi_pos)} (tip lines {tips[lo_sg]:.2f} / '
                     f'{tips[-lo_sg]:.2f})')
        self._face_comb(launch_o, target_o, sched)
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
                    # a BAND lane's leg runs on the lane's own page: the
                    # band comb is nested (first exiter nearest its
                    # stubs), so its leg crosses no lane still present
                    # and needs no via at the slot; the berth's layer,
                    # where it differs, is met by a via at the tip
                    pg = sched.page.get(nm) if sched is not None else None
                    self.leg_layer[nm] = (pg if (pg is not None and self._in_band(nm))
                                          else leg_L)
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
                return min(ISLAND_VETO, JOG_VIA * abs(s_m - s_l) / LPITCH)

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
            ch_ = self._prof_changes(nm)
            GAP = DIVE_GAP if ONE_DIVE >= 3 else DIVE_ROOM
            if ch_:
                # a SCHEDULED lane: its tooth layer to the first change,
                # the other layer to the next, ... both layers open for
                # GAP around each change; a change at the region's start
                # or end is laid like a page lane's birth / landing via
                s_beg = self.s0 if nm not in self.join_block else self.join_leg_s[nm]
                bounds_ = [a] + [c for c in ch_] + [b]
                L_ = tlr[nm]
                for i_ in range(len(bounds_) - 1):
                    lo_, hi_ = bounds_[i_], bounds_[i_ + 1]
                    if i_ > 0:
                        lo_ = a if bounds_[i_] <= s_beg + GAP else bounds_[i_] + GAP
                    if i_ < len(bounds_) - 2:
                        hi_ = b if bounds_[i_ + 1] >= self.s1 - GAP else bounds_[i_ + 1] - GAP
                    if i_ == 0 and bounds_[1] <= s_beg + GAP:
                        L_ = other_layer(L_)
                        continue              # born on the other layer: the first stretch is empty
                    if hi_ > lo_:
                        req[nm].append((lo_, hi_, L_))
                    if i_ < len(bounds_) - 2 and bounds_[i_ + 1] >= self.s1 - GAP:
                        break                 # the last change is the landing via
                    L_ = other_layer(L_)
                continue
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
        for nm in getattr(self, 'face_block', {}):
            # the face leg, its jog and the diagonal up to the region are
            # on the tooth's layer and nothing else: the front lanes they
            # run under leave the face on the other one (stamped on both,
            # the stretch between the leg and s0 walled SDQ13's F fan-in)
            req[nm].append((self.st[nm][0] - 0.05,
                            max(self.face_leg_s[nm] + 0.2, self.s0 + 0.06),
                            self.ctx.tooth_layer[nm]))
        self.req, self.bwin = req, bwin

        # lane centrelines in (s, o): tooth (or its join leg's end), the
        # column midpoints in the order the schedule gives, the target
        # slot at s1, then the tail (head-on) or the run to the exit leg
        py = self.py
        self.mid, self.legs, self.jogs = {}, {}, {}
        self.swim_hold = {}
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
            elif nm in self.face_block:
                # FACE_SORT: the jog along the tooth's row, the leg along
                # the face to the slot, the run at the slot to the region
                s_l = self.face_leg_s[nm]
                if abs(s_l - s_t) > 1e-9:
                    jogs.append(((s_t, o_t), (s_l, o_t)))
                legs.append((s_l, o_t, self.face_block[nm]))
                pts = [(s_l, self.face_block[nm])]
            else:
                pts = [(s_t, o_t)]
            # RIBBON: hold the launch offset at the region start,
            # then run STRAIGHT to the target slot at s1 (a face lane's
            # diagonal starts at its leg: legs a pitch apart in s give
            # parallel diagonals their room, where the same slots all
            # starting at s0 lay 0.19 mm apart across a 1.5 slope)
            if nm not in getattr(self, 'face_start', {}):
                pts.append((self.s_of_u(0.0), self.launch_o[nm]))
            tail_done = False
            if SWIM_HOLD >= 2 and sched is not None and sched.page.get(nm) is None:
                # ...a SWIMMER, level 2: a chosen hold offset, a run that
                # may end in the tail (see _swim_hold2)
                hr = self._swim_hold2(nm, sched)
                if hr is not None:
                    o_h, c1, c2 = hr
                    s_a = self.s_of_u(0.0)
                    if abs(o_h - self.launch_o[nm]) > 1e-9:
                        pts.append((s_a + 0.3, o_h))
                    if c1 > pts[-1][0] + 1e-6:
                        pts.append((c1, o_h))
                    if c2 <= self.s1 + 1e-9:
                        if c2 < self.s1 - 1e-6:
                            pts.append((c2, py[trank[nm]]))
                    else:
                        pts.append((c2, py[trank[nm]]))
                        tail_done = True       # past s1: no slot vertex at s1
                    self.swim_hold[nm] = hr
            elif SWIM_HOLD and sched is not None and sched.page.get(nm) is None:
                # ...a SWIMMER: hold, run, hold (see _swim_hold)
                hr = self._swim_hold(nm, sched)
                if hr is not None:
                    c1, c2 = hr
                    if c1 > pts[-1][0] + 1e-6:
                        pts.append((c1, self.launch_o[nm]))
                    if c2 < self.s1 - 1e-6:
                        pts.append((c2, py[trank[nm]]))
                    self.swim_hold[nm] = hr
            if not tail_done:
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
        if self.swim_hold:
            self.log('  swimmers hold-then-run: ' + ', '.join(
                (f'{nm} c1 {v[0]:.1f} c2 {v[1]:.1f}' if len(v) == 2 else
                 f'{nm} hold o{v[0]:+.2f} c1 {v[1]:.1f} c2 {v[2]:.1f}')
                for nm, v in sorted(self.swim_hold.items())))
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
        # LEVEL 5: the schedule over the laid geometry, tail included;
        # its solution replaces the pages, the leg layers and the required
        # stretches decided above (see the flag's note)
        if ONE_DIVE >= 5 and sched is not None:
            if self._profiles5(sched):
                self.req = self._req_from_profile(sched)
                self.bwin = self._bwin_from_req(self.req, sched)
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

        for nm in M:
            c_ = getattr(self, 'dive', {}).get(nm)
            rd_ = getattr(self, 'ride', {}).get(nm)
            if self._prof_changes(nm):
                c_ = c_ if c_ is not None else -1.0      # any scheduled lane takes the diamond path below
            if sched.page.get(nm) is not None and c_ is None and rd_ is None:
                continue
            # the swimmer's crossings with PAGE lanes, each forcing
            # the layer opposite the page it crosses
            want = []
            for om in M:
                P = sched.page.get(om)
                if c_ is not None or rd_ is not None or om == nm or P is None or not sched.inverted(nm, om):
                    continue
                lo_s = max(self.mid[nm][0][0], self.mid[om][0][0])
                hi_s = min(self.mid[nm][-1][0], self.mid[om][-1][0])
                if hi_s - lo_s < 0.1:
                    continue
                S = np.arange(lo_s, hi_s, 0.05)
                d = np.array([line_o(nm, s) - line_o(om, s) for s in S])
                for i in np.where(np.sign(d[:-1]) != np.sign(d[1:]))[0]:
                    want.append((float(S[i]), self._need_at(om, float(S[i]), sched)))
            if not want and c_ is None and rd_ is None:
                continue
            want.sort()
            GAPd = max(DIVE_GAP if ONE_DIVE >= 3 else DIVE_ROOM, 0.1)
            ch_ = self._prof_changes(nm)
            if ch_:
                # a scheduled lane's interior changes (one at a region end
                # is a page lane's birth / landing via, no diamond)
                s_beg = self.s0 if nm not in self.join_block else self.join_leg_s[nm]
                seq = []
                for c in ch_:
                    if s_beg + GAPd < c < self.s1 - GAPd:
                        seq += [(c - GAPd, self._prof_layer(nm, c - 1e-6)),
                                (c + GAPd, self._prof_layer(nm, c + 1e-6))]
            else:
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
            if self._prof_changes(nm):
                self.log(f'  scheduled {nm}: changes at s ' + '/'.join(f'{c:.2f}' for c in self._prof_changes(nm))
                         + f' ({self.ctx.tooth_layer[nm][0]}->...->{self.ctx.dest_layer[nm][0]}), '
                         f'{len(spots)} diamond(s) reserved')
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
                        s_t = s_c
                        if TAIL_PAGE and nm in self.exit_block and nm in self.exit_leg_s:
                            s_t = max(self.s1 + 0.1, min(s_c, self.exit_leg_s[nm] - LEG_REQ - 0.05))
                        if not self.allowed(nm, s_t, L):
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
        fp = getattr(self, 'full_prof_s', {}).get(nm)
        if fp is not None:
            # level 5: the whole lane's scheduled changes, region and tail
            seq += list(fp)
        else:
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
                legs_ = self.legs[nm]
                lp = getattr(self, 'leg_prof', {}).get(nm)
                hp = getattr(self, 'head_prof', {}).get(nm)
                for i_l, (s_l, oa, ob) in enumerate(legs_):
                    rect = ((np.abs(Ss - s_l) <= LEG_W + slack)
                            & (Os >= min(oa, ob) - LEG_O - slack)
                            & (Os <= max(oa, ob) + LEG_O + slack))
                    okl = okL
                    if (lp is not None and not open_layers and nm in self.exit_block
                            and i_l == len(legs_) - 1):
                        # level 5: the exit leg's own layer profile, along o
                        okl = self._piece_ok(lp, Os, oa, ob, L, DIVE_GAP)
                    elif (hp is not None and not open_layers and i_l == 0
                            and nm in getattr(self, 'join_block', {})):
                        # level 5 (HEAD_L5): the join leg's, tooth side first
                        okl = self._piece_ok(hp, Os, oa, ob, L, DIVE_GAP)
                    oks |= rect & okl
                jp = getattr(self, 'jog_prof', {}).get(nm)
                hjp = getattr(self, 'hjog_prof', {}).get(nm)
                for ((sa, oa), (sb, ob)) in self.jogs.get(nm, ()):
                    rect = ((Ss >= min(sa, sb) - LEG_O) & (Ss <= max(sa, sb) + LEG_O)
                            & (np.abs(Os - oa) <= LEG_O))
                    okj = okL
                    if (jp is not None and not open_layers and nm in self.exit_block
                            and abs(sb - self.se[nm][0]) < 1e-6 and abs(ob - self.se[nm][1]) < 1e-6):
                        # level 5: the exit jog's own layer profile, along s
                        okj = self._piece_ok(jp, Ss, sa, sb, L, DIVE_GAP)
                    elif (hjp is not None and not open_layers
                            and abs(sa - self.st[nm][0]) < 1e-6 and abs(oa - self.st[nm][1]) < 1e-6):
                        # level 5 (HEAD_L5): the join jog's, from the tooth
                        okj = self._piece_ok(hjp, Ss, sa, sb, L, DIVE_GAP)
                    oks |= rect & okj
                ok[i:i + STRIP] = oks
            return ok
        return band

    def _gap_layer(self, om, s):
        """Inside a scheduled change's both-layer gap: the ONE layer the
        lane's virtual copper is promised on at s (its layer before the
        change, then after it); None outside every gap."""
        ch = self._prof_changes(om)
        if not ch:
            return None
        GAP = DIVE_GAP if ONE_DIVE >= 3 else DIVE_ROOM
        for c in ch:
            if c - GAP <= s <= c + GAP:
                # the layer just before the change, or just after it
                return self._prof_layer(om, c - 1e-6) if s < c else self._prof_layer(om, c + 1e-6)
        return None

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
                    head = (abs(s_a - s_start) < 1e-6 and om not in self.join_block
                            and om not in getattr(self, 'face_block', {}))
                    if swim_om and not (tail or head):
                        continue
                    gapL = self._gap_layer(om, s_mid)
                    for p_, q_ in zip(xy, xy[1:]):
                        for L in ('F.Cu', 'B.Cu'):
                            if tail and L != self.ctx.dest_layer[om]:
                                continue
                            if head and L != self.ctx.tooth_layer[om]:
                                continue
                            if gapL is not None and L != gapL:
                                # inside a scheduled change's gap: the
                                # promise is one layer either side of the
                                # change, not both over the whole gap
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
                lp = getattr(self, 'leg_prof', {}).get(om)
                if is_exit and lp is not None:
                    # level 5: the leg on its scheduled layer, split at
                    # each change (one layer either side of it)
                    L0, ch = lp
                    cur_o, curL = oa, L0
                    for (o_c, L_a) in ch:
                        segs.append((sp.xy(s_l, cur_o), sp.xy(s_l, o_c), curL))
                        cur_o, curL = o_c, L_a
                    segs.append((sp.xy(s_l, cur_o), sp.xy(s_l, ob), curL))
                    continue
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
                if om in getattr(self, 'face_block', {}):
                    # a face leg runs under the front lanes on its own layer
                    segs.append((a_, b_, self.ctx.tooth_layer[om]))
                    continue
                hp = getattr(self, 'head_prof', {}).get(om)
                if i == 0 and om in self.join_block and hp is not None:
                    # level 5 (HEAD_L5): the join leg on its scheduled
                    # layer, split at each change, tooth side first
                    L0, ch = hp
                    cur_o, curL = oa, L0
                    for (o_c, L_a) in ch:
                        segs.append((sp.xy(s_l, cur_o), sp.xy(s_l, o_c), curL))
                        cur_o, curL = o_c, L_a
                    segs.append((sp.xy(s_l, cur_o), sp.xy(s_l, ob), curL))
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
                jp = getattr(self, 'jog_prof', {}).get(om)
                if is_exit_jog and jp is not None:
                    # level 5: the jog on its scheduled layer, split at each change
                    L0, ch = jp
                    cur_s, curL = sa, L0
                    for (s_c, L_a) in ch:
                        segs.append((sp.xy(cur_s, oa), sp.xy(s_c, oa), curL))
                        cur_s, curL = s_c, L_a
                    segs.append((sp.xy(cur_s, oa), sp.xy(sb, ob), curL))
                    continue
                hjp = getattr(self, 'hjog_prof', {}).get(om)
                if (not is_exit_jog and hjp is not None
                        and abs(sa - self.st[om][0]) < 1e-6 and abs(oa - self.st[om][1]) < 1e-6):
                    # level 5 (HEAD_L5): the join jog on its scheduled layer, split at each change
                    L0, ch = hjp
                    cur_s, curL = sa, L0
                    for (s_c, L_a) in ch:
                        segs.append((sp.xy(cur_s, oa), sp.xy(s_c, oa), curL))
                        cur_s, curL = s_c, L_a
                    segs.append((sp.xy(cur_s, oa), sp.xy(sb, ob), curL))
                    continue
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
        tv = getattr(self, 'tail_vias', None)
        if tv is not None:
            # level 5: exactly the sites the schedule puts a change at
            # beyond the region -- a corner, a leg split, a jog change
            out = [p for om in unrouted for p in tv.get(om, ())]
        else:
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
        # the judge's cap for a plan being RANKED (plan_braid sets ctx.judge);
        # braid.py's own phase 1 is plan_only too and keeps the full cap
        judge = plan_only and getattr(ctx, 'judge', False)
        self._l5_time = L5_JUDGE_TIME if judge else L5_TIME
        self._l5_nodes = L5_JUDGE_NODES if judge else L5_NODES
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
        if getattr(self, 'guarded', None):
            log('  exit guard (BRAID_EXIT_GUARD): ' + ', '.join(
                f'{nm}<-{"/".join(oms)}' for nm, oms in sorted(
                    self.guarded.items(), key=lambda kv: self.se[kv[0]][0])))
        if self.flank:
            log(f'  flank stubs (BRAID_FLANK_COMB={FLANK_COMB}, never head-on): '
                + ', '.join(f'{nm}{"+" if sg > 0 else "-"}'
                            for nm, sg in sorted(self.flank.items(),
                                                 key=lambda kv: self.se[kv[0]][0])))
        ly_floor = 0.35
        self.offsets(ly_floor)
        self.reserve_intervals()
        sched = Schedule(self.launch, self.target, ctx.tooth_layer, log=log,
                         dest_layer=ctx.dest_layer)
        self.one_dive(sched, log=log)
        if SLOPE_PITCH:
            self.offsets(ly_floor, sched=sched)
            self.reserve_intervals()
            sched = Schedule(self.launch, self.target, ctx.tooth_layer,
                             dest_layer=ctx.dest_layer)
            self.one_dive(sched, log=log)
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
            self.one_dive(sc_)
            if SLOPE_PITCH:
                self.offsets(ly, sched=sc_)
                self.reserve_intervals()
                sc_ = Schedule(self.launch, self.target, ctx.tooth_layer,
                               dest_layer=ctx.dest_layer)
                self.one_dive(sc_)
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
                            vr=self._via_room_mode(),
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
            # ...AND ROOM FOR THE VIA, once the pitch has none left. A
            # corridor still refusing at the maxed pitch is the evidence
            # that the model's via room -- priced across the spine -- was
            # not the room the router had, so it re-plans with that room
            # priced in xy (VIA_ROOM_REFUSED) and gets a fresh attempt.
            # ORDER IS THE WHOLE POINT: a corridor that CONVERGES never
            # reaches this (greedy K35 routes 35/35 at 0.40 on attempt 2
            # and stops), and measured as an unconditional knob the same
            # test costs that board 6 vias -- 64 -> 70 -- while paying 8
            # on jc1's tight plan, which is still refusing 8 lanes at
            # 0.40 four attempts running. The pitch is the cheap
            # dimension; the model's own metric is the next one.
            if ly_floor >= 0.40 - 1e-9 and self.refused and VIA_ROOM_REFUSED \
                    and self._via_room_mode() < VIA_ROOM_REFUSED:
                self._vr_mode = VIA_ROOM_REFUSED
                log(f'    via room: the pitch is spent and {len(self.refused)} '
                    f'lane(s) still refuse -- priced in xy from here '
                    f'(BRAID_VIA_ROOM_REFUSED={VIA_ROOM_REFUSED})')
                prev_refused = list(self.refused)
                for nm in self.refused:
                    boost[nm] = boost.get(nm, 0) + 1
                continue
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
            # the re-plan must use the mode the kept attempt was planned
            # under, or `sched` describes a different world than the
            # copper below it (the _PROFILE_MEMO leak's lesson)
            self._vr_mode = best.get('vr', 0)
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
                    if nm in longs:
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
    if plan.get('alts'):
        q['alts'] = {nm: [dict(a, exit=list(M(*a['exit'])), layer=other_layer(a['layer']),
                             **({'dir': [a['dir'][0], -a['dir'][1]]} if a.get('dir') else {}))
                          for a in al]
                     for nm, al in plan['alts'].items()}
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
    # candidate berths for the plan's residue nets (a plan being RANKED
    # by the fanout loop's residue search; see L5_ALT_NODES): never in a
    # sidecar, so the braid proper sees none
    ctx.alts = (plan or {}).get('alts') or {}
    ctx.alt_excl = (plan or {}).get('alt_excl') or []
    ctx.alt_xing = (plan or {}).get('alt_xing') or []
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
    # a BANDED destination (SPLIT_BLOCKS, escape_moves.blocks_of): the
    # streets between its blocks, where a stub may end -- the band comb
    # in Corridor.offsets. Off (or a solid array): no bands, inert.
    ctx.dest_bands = {}
    ctx.dest_band_tip = {}
    if os.environ.get('SPLIT_BLOCKS', '0') not in ('', '0'):
        import escape_moves as _em
        for ref in {ends[nm][2] for nm in names}:
            _g = _em.grid_of(pcb.footprints[ref])
            ctx.dest_bands[ref] = _em.bands_of(_em.blocks_of(pcb.footprints[ref]))
            # a band stub's tip stands half a pitch and one occupancy cell
            # off its ball line (the under-pad engine ends at the boundary
            # cell; measured 0.425 at 0.8 mm pitch)
            ctx.dest_band_tip[ref] = max(_g.pitch_x, _g.pitch_y) / 2 + 0.05
    ctx.spine_obs = build_obstacles(pcb, -1, kids, bundle_layer)

    # the destination's box, for the spine's tail on the array's axis
    # (corridor.align_tail; SPLIT_BLOCKS only, off = the chord as before)
    def dest_box_of(members):
        if os.environ.get('SPLIT_BLOCKS', '0') in ('', '0'):
            return None
        ps = [p for nm in {ends[m][2] for m in members}
              for p in pcb.footprints[nm].pads]
        if not ps:
            return None
        return (min(p.global_x for p in ps), min(p.global_y for p in ps),
                max(p.global_x for p in ps), max(p.global_y for p in ps))

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
            stub_dirs=[ctx.stub_dir[nm] for nm in members], relax=relax,
            dest_box=dest_box_of(members))
    ctx.spine_of = spine_of
    def centre_of(ref):
        ps = pcb.footprints[ref].pads
        return (sum(p.global_x for p in ps) / len(ps),
                sum(p.global_y for p in ps) / len(ps))
    def band_line_of(nm):
        """(band, side) of a stub in a band of its destination, else None."""
        for band in ctx.dest_bands.get(ends[nm][2], ()):
            x0, y0, x1, y1 = band
            x, y = ends[nm][1]
            if x1 - x0 >= y1 - y0:
                if y0 < y < y1 and x0 - 0.5 <= x <= x1 + 0.5:
                    return (band, y < (y0 + y1) / 2)
            elif x0 < x < x1 and y0 - 0.5 <= y <= y1 + 0.5:
                return (band, x < (x0 + x1) / 2)
        return None
    _bl = {nm: band_line_of(nm) for nm in names} if ctx.dest_bands else {}
    groups = cr.cluster_corridors(
        names, ctx.paths, {nm: ends[nm][0] for nm in names},
        {nm: ends[nm][1] for nm in names}, pad_obs.seg_clear, D=6.0,
        log=log, spine_fn=lambda core: spine_of(core, relax=False),
        dest_ref={nm: ends[nm][2] for nm in names},
        centres={nm: centre_of(ends[nm][2]) for nm in names},
        src_centres={nm: centre_of(ctx.src_ref[nm]) for nm in names},
        same_line=((lambda a, b: _bl.get(a) is not None and _bl.get(a) == _bl.get(b))
                   if _bl else None))
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
    ctx.judge = True            # a plan being RANKED: level 5 solves under L5_JUDGE_TIME
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
                       # ONE_DIVE: the scheduled change point (s) of a diver
                       'dive': getattr(c, 'dive', {}).get(nm),
                       'ride': getattr(c, 'ride', {}).get(nm),
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
                       # dives under EARLIER corridors' lanes (2 each), and
                       # a band comb slot the band has no room for (a
                       # swimmer's price: the lane cannot run in-band)
                       'cross_vias': (cross.get(nm, 0)
                                      + (SWIM_PRICE if nm in getattr(c, 'band_over', ())
                                         else 0)),
                       # the BERTH CHOICE solve's advice (ctx.alts): the
                       # candidate index chosen (0 = the plan's own berth),
                       # whether the net still swims with it, and the
                       # solve's objective (plain, with the choice)
                       'alt': getattr(c, 'alt_choice', {}).get(nm),
                       'alt_w': getattr(c, 'alt_w', {}).get(nm),
                       'alt_obj': getattr(c, 'alt_obj', None)}
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
        out_segs = {nm: c.out_segs[nm] for c in corridors for nm in c.members}
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

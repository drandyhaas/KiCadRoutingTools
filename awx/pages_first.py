"""pages_first.py -- the PAGES-FIRST planner (#622, 2026-09-13; opt-in
PLAN_PAGES=1 in the fanout stage, the old planner byte-identical at 0).

The directive it serves: no net may need more than two vias. Under the
braid's two-page model a page lane costs at most two (one at each end whose
layer is not its page), and a lane that fits NEITHER page -- a swimmer --
is the only thing that costs more. So the plan must hand the braid launch
and target orders that two crossing-free chains cover completely, and
that is a property of the ENDS the plan chooses, not of the router.
Measured before this existed: on our recorded K41 / K51 plans the best any
two-page schedule can page is 28 of 41 / 28 of 47 lanes (13 / 19 must
swim); on the human's ends 35 / 41 (6 / 6).

Why the human's ends are orderable: per net the vias are 0 for an F tooth
on page F to an F berth, and exactly 2 for every other combination of
(tooth, page, berth) layers but one, so once a net has a B end its page is
free at no cost -- and a B end makes the net's RANK free, because under a
BGA the back layer has no pads and a dog-bone can climb along a gap and
leave the face at a chosen row (escape_moves climb=), or enter the field
from a chosen column. An F end pins the rank to the ball's own gap. The
recorded chain could not use that freedom: the source was frozen, the
destination menu had no climb class, and every chooser priced a swimmer at
two vias and accepted it.

This planner chooses BOTH ends and the page of every net in ONE CP-SAT:
  * per net, one destination move from its menu (walked / climbed ones
    when DST_WALK / DST_CLIMB enumerate them) and one source move from
    {the tooth as it stands} + its source menu (climbs under SRC_CLIMB);
  * a page per net; the cost of a net is its escape vias plus one per end
    whose layer differs from its page, plus the greedy's channel and reach
    weights as tie-breaks (sched_first's units);
  * HARD: two nets on one page are never inverted between the launch order
    and the target order -- keys from sched_first.Frame at the destination
    and its mirror at the source (the braid's nesting: front face by
    position across the bundle, side faces nested first-exiter innermost,
    the far face outermost). A net may still be left to swim, at a price
    of PLAN_PAGES_SWIM vias, so an infeasible instance answers with the
    fewest swimmers instead of nothing;
  * two moves that cannot both be laid (select_moves._conflict) exclude
    each other; berths a caller holds `fixed` are one-move menus; `learned`
    pairs the engine refused together exclude each other.
The braid then verifies the plan with its own planner (judge_by_braid): a
net it still swims is a disagreement between these keys and its order,
printed by name. Deterministic: CP-SAT under a deterministic-time budget
with interleaved search (PLAN_PAGES_DET), never wall-clock.
"""
from __future__ import annotations

import math
import os
import time
from typing import Dict, List, Optional, Tuple

import select_moves as sm
import source_realize as sr
from escape_moves import Move, DIRS
from sched_first import Frame, VIA_W, CHAN_W
import plan_feedback as pfb  # the route's verdict (PLAN_LOOP_FEEDBACK), plan_loop.py
import solve_memo as _smemo  # noqa: E402  a solve read back instead of run

Pt = Tuple[float, float]

PAGES_DET = float(os.environ.get('PLAN_PAGES_DET', '40'))     # CP-SAT deterministic time. 40 (2026-09-14): at 20 the K41 solve stops FEASIBLE with 4 swimmers, at 40 with 2-3, 80 adds nothing; on the ladder 20 -> 40 took K41 87 -> 79/81 and K51 125 -> 115 complete, within the time budget (K41 92 s)
PAGES_WORKERS = int(os.environ.get('PLAN_PAGES_WORKERS', '4'))
# PLAN_PAGES_CANON=n (2026-09-16): a solve that gives the SAME ANSWER ON ANY
# MACHINE, bounded by n CONFLICTS. 0 = off, and off is the default because
# this changes which plan is chosen, not just how it is reached.
#
# Why the default solve is not portable, measured: `max_deterministic_time`
# is a WORK ACCUMULATOR, not an invariant count -- at det 40 arm64 does 1685
# conflicts and x86_64 does 1970, so the same budget buys 1.17x the search
# and the solve stops somewhere else. The stopping point is the answer: at
# K28 the two platforms reach the SAME objective 727.2 and hand back
# DIFFERENT solution vectors (pages F18/B10 against F20/B8), which carries
# into a different exclusion set, a different plan, and 783 against 1144
# segments of copper. Raising the budget does NOT fix it -- at det 640 both
# sides PROVE OPTIMAL 727.2 and still disagree, because the optimum is
# degenerate and proving a VALUE does not pick a SOLUTION.
#
# So this changes both halves of the cause:
#   * num_workers=1 -- ONE search path. The 4-worker portfolio is
#     deterministic on one machine (interleave_search) but its workers
#     share bounds and clauses, and which one gets there first is a
#     function of relative speed.
#   * max_number_of_conflicts -- an INTEGER COUNT of discrete search
#     events, which is the same number on any CPU.
# max_deterministic_time stays as a BACKSTOP, because a conflict count
# bounds SEARCH and not PRESOLVE: setting a conflict limit ALONE is how a
# 40 s solve ran 31 minutes and got the earlier attempt at this withdrawn.
# The backstop is generous and the report says WHICH LIMIT FIRED -- a
# backstop nobody checks is how that class of bug comes back, and a run
# stopped by the backstop is NOT portable and must not be read as if it were.
PAGES_CANON = int(os.environ.get('PLAN_PAGES_CANON', '0') or 0)
PAGES_CANON_DET = float(os.environ.get('PLAN_PAGES_CANON_DET', '600') or 600)
# ...and the SECOND channel, which one worker and an integer stop do NOT
# close: the LP RELAXATION IS FLOATING POINT. Measured at K28 with
# num_workers=1 and a conflict budget, the two platforms reported bound
# 557.8 against 557.6 -- a different relaxation bound prunes differently,
# which moves the search path, which lands on a different conflict count
# (11641 against 12477) and a different plan (obj 778.3 against 1042.1).
# linearization_level=0 removes the LP entirely, leaving a purely INTEGER
# search whose every decision is exact. It costs search strength -- the LP
# is where good bounds come from -- so a canonical solve needs a bigger
# budget to reach the same plan. 1 keeps the LP (for measuring that this
# is really the channel); 0 is the portable arm.
PAGES_CANON_LP = int(os.environ.get('PLAN_PAGES_CANON_LP', '0') or 0)
# How many workers a canonical solve may use. 1 is the arm PROVEN portable
# at K28; >1 is on trial. Without the LP there is no floating point in the
# integer search, so a deterministic interleave might stay portable at 4x
# the search -- but CP-SAT's worker portfolio includes FEASIBILITY JUMP,
# which is a float local search with its OWN linearization level, so the
# LP is not the only float channel once the portfolio is on. Both are
# pinned here, and the arm is measured rather than assumed.
PAGES_CANON_WORKERS = int(os.environ.get('PLAN_PAGES_CANON_WORKERS', '1') or 1)
# CANON AND THE WALK DO NOT COMPOSE, and the walk loses (review, 2026-09-16).
# `_walk` controls its proposals by REASSIGNING the module global PAGES_DET
# (to PLAN_PAGES_WALK_DET, default 10) and swapping it back for the
# reference solve. A canonical solve reads NEITHER -- it is bounded by
# conflicts with PAGES_CANON_DET as its backstop -- so every walk proposal
# silently gets the backstop (default 600) instead of the 10 it asked for:
# up to PLAN_PAGES_WALK_SOLVES proposals at ~60x their intended budget, and
# PLAN_PAGES_WALK_FROM=solve's deliberate cheap-proposal / full-reference
# distinction collapses to nothing. Say so once rather than let a walk arm
# quietly cost sixty times its budget and mean something else.
if PAGES_CANON and int(os.environ.get('PLAN_PAGES_WALK', '0') or 0):
    print('  pages-first: WARNING -- PLAN_PAGES_CANON with PLAN_PAGES_WALK: the '
          'walk controls its proposals through PLAN_PAGES_WALK_DET, which a '
          f'canonical solve does not read. Every proposal gets the conflict '
          f'budget ({PAGES_CANON}) and the det backstop ({PAGES_CANON_DET:g}), '
          'not the walk budget. The two do not compose.')
PAGES_SWIM = float(os.environ.get('PLAN_PAGES_SWIM', '100'))  # vias: the price of a net left to swim
# PLAN_PAGES_SWIM_XING (2026-09-17): a swimmer's price PER PAGED LANE IT
# CROSSES, instead of the flat `PLAN_PAGES_SWIM` alone. 0 = off, and off is
# the default, so the objective is unchanged by construction.
#
# Why a flat price cannot be right, measured. The braid reports what each
# swimmer actually cost it -- `swimmer SDQ15: 8 page crossing(s), 6
# change(s)` -- and over 262 swimmers from every run of the K ladder and the
# synthetic b5 batch those changes are NOT a constant: they run from 2 to 22,
# and they track the page crossings:
#
#     changes ~ 0.44 * page_crossings + 0.79   (Pearson r = 0.83, n = 262)
#
# 70% of the variance, against a flat mean of 4.98 -- and the model charges
# 100. That is the term this campaign's anti-correlation lives in: with a
# flat price the objective cannot tell a cheap swimmer from an expensive one,
# so its optimum is DEGENERATE over plans whose real costs differ by many
# vias (awx/synth_bus.py --judge measures exactly that).
#
# The fix is expressible because "how many paged lanes does this net cross"
# is PAIRWISE, which is the shape the no-inversion constraint already has:
# for each crossing pair, one term when exactly one of the two swims. So it
# costs one bool and two linear constraints per pair and no new search
# structure. Set it with a matching `PLAN_PAGES_SWIM` base (the fit says
# 0.44 / 0.8); the flat 100 with a non-zero XING would just be the flat
# price again.
PAGES_SWIM_XING = float(os.environ.get('PLAN_PAGES_SWIM_XING', '0'))
PAGES_ISLAND = float(os.environ.get('PLAN_PAGES_ISLAND', '0') or 0)   # vias per corridor part a page lane's chord crosses on its page (learned from verify; 0 = off)
PAGES_SRC = int(os.environ.get('PLAN_PAGES_SRC', '1'))        # 0 = the source frozen (destination only)
PAGES_LOG = int(os.environ.get('PLAN_PAGES_LOG', '0'))        # 1 = per-net choice printed
PAGES_ITERS = int(os.environ.get('PLAN_PAGES_ITERS', '3'))    # re-key on the chosen plan, at most this often
PAGES_KEYS = os.environ.get('PLAN_PAGES_KEYS', 'braid')       # 'braid' = the braid's own slots; 'frame' = sched_first.Frame
PAGES_WIDEN = int(os.environ.get('PLAN_PAGES_WIDEN', '0'))
PAGES_SIDEKEY = int(os.environ.get('PLAN_PAGES_SIDEKEY', os.environ.get('PLAN_PAGES_SIDERS', '1')))
PAGES_SIDERS_MODE = int(os.environ.get('PLAN_PAGES_SIDERS', '1'))
PAGES_STRICT = int(os.environ.get('PLAN_PAGES_STRICT', '1'))
PAGES_HINT = int(os.environ.get('PLAN_PAGES_HINT', '1'))      # 1 = the seed plan (the greedy's berths, the teeth as they stand) as the CP-SAT's solution hint. At DET 20 it measured WORSE standalone (K41 obj 2359.8 / 4 swimmers -> 2659.4 / 5); at DET 40 on the LADDER it is worth 2 vias at K41 (81 -> 79) and completion at K51 (106 / SA2 open -> 115 complete), 2026-09-14 pg2 vs pg3. On, with DET 40.
PAGES_JOINKEY = int(os.environ.get('PLAN_PAGES_JOINKEY', '1'))  # 1 = a side exit's slot depends on its SOURCE class: lanes whose tooth is a joiner sit outermost of the block, by tooth position (the braid's exit-block rule)
# CLIMBS FOR THE SWIMMERS ONLY (2026-09-14 late). The climbed candidates
# (SRC_CLIMB / DST_CLIMB, escape_moves climb=) are the human's rank
# freedom, and offered to every net they multiply the model six to eight
# times (K51 tooth candidates 838 -> 1874, exclusions 41k -> 252k) so the
# CP-SAT stops at a worse feasible point and the chain routes it (K51 115
# -> 132, 112 / 2 open deduped; K41 79 -> 90 / 83). PLAN_PAGES_CLIMB_LATE=1:
# iteration 0 solves the PLAIN menu (the base instance, byte-identical),
# and the climbs reach the model only in the re-solve that frees the
# swimmers -- every other net held at the verified plan -- so the
# enlarged menu is paid for exactly where the two pages ran out.
PAGES_CLIMB_LATE = int(os.environ.get('PLAN_PAGES_CLIMB_LATE', '0'))
# A MOVE THAT IS NOT A MOVE (2026-09-14, session 8). The source menu emits
# every legal tooth of a net, the tooth AS IT STANDS included -- same kind,
# face, layer and exit within a few hundredths of a millimetre (K41: 20 of
# the 203 tooth candidates; K35 16, K51 22) -- and the model keyed the two
# DIFFERENTLY (the standing tooth by the braid's relaxed-pitch launch_o, the
# candidate by its raw offset through _alt_src_slot: 20 of 20 differ at K41,
# by up to 0.67 mm), so it could "fix" an inversion on paper by re-laying a
# tooth identically. The engine then lays the same copper and the audit
# says "= original" (every K >= 35 on the pg2 chain asked one). PLAN_PAGES_NOOP=1:
# a candidate identical to the standing tooth leaves the menu -- the
# standing tooth represents it. Measured on the ladder (2026-09-14 s8, on
# the PLAN_BATCH loop): 34 / 61 / 77 / 141 against 34 / 65 / 79 / 115 -- a
# smaller model, a different feasible stop, K51 far worse; not a default.
# (Keying the standing head-on teeth at their RAW offsets instead -- the
# candidates' scale -- was built and measured 36 / 65 / 86 / 151 + 8 open:
# the braid orders the standing teeth by the RELAXED launch_o, so the raw
# key disagrees with the order the braid will use; it is the CANDIDATE key
# that is on the wrong scale, and the fix would be to relax a candidate's
# offset into the standing sequence. Removed.)
PAGES_NOOP = int(os.environ.get('PLAN_PAGES_NOOP', '0'))
# THE RESOURCE FORM OF THE CONFLICT TEST (2026-09-14, session 8; handoff
# item 2). The pairwise test grows with the square of the candidates
# sharing a lane (K41 SRC_CLIMB=2: 31k -> 203k exclusions, K51 252k, K28
# DST 718k) and the CP-SAT at DET 40 then stops at a worse feasible point.
# PLAN_PAGES_CELLS=1: every candidate occupies a set of CELLS -- tol-sized
# (sm._EXIT_TOL) point cells on its layer along every lane stretch, both
# across-cells so lanes within tol share one, its via site on both layers
# within _VIA_REACH, its exit point -- and each cell is ONE AddAtMostOne
# over the candidates in it: linear in candidates x stretch length. A row
# run and a column run that cross share the crossing's cell, which is the
# pairwise test's own rule under PLAN_PAGES (SEL_XING=2 there). Probed at
# K41 with SRC_CLIMB=2 against the pairwise test: every pairwise conflict
# is implied (MISSED 0) and the cells imply 21% (berths) / 36% (teeth) more
# pairs -- the slop of tol-sized cells taken two across (a pair within
# 2 tol may share one) and of a via's reach as a square. Learned pairs
# stay pairwise. 0 = the pairwise test, byte-identical.
PAGES_CELLS = int(os.environ.get('PLAN_PAGES_CELLS', '0'))
# THE BERTH KIND (2026-09-14, session 8; handoff item 4). The human's 47
# DU1 ends: 36 dog-bones, no via-in-pad; ours at K41: 16 via-in-pad + 14
# bare stubs (README "how the human routes the congested region"). The
# model prices a via-in-pad and a dog-bone alike (one via) and the
# via-in-pad's run is shorter, so it wins on the channel term; and a bare
# stub on the wrong page costs the same one via as a dog-bone on the right
# page, though the braid has to find that via's room in the corridor.
# PLAN_PAGES_KIND_VIP: extra cost of a via-in-pad candidate, in via units,
# both ends (0 = off). PLAN_PAGES_MISMATCH: the price of an end whose layer
# is not its page, as a multiple of a via (1 = as it was).
PAGES_KIND_VIP = float(os.environ.get('PLAN_PAGES_KIND_VIP', '0') or 0)
# A PER-FACE-STRIP CAPACITY (2026-09-14, session 8; handoff item 5). K51's
# SCKE1: 11 'up' berths + 4 far-face lanes in the north strip between DU1
# and the passives, refused in band; the human goes round the SOUTH, the
# face with room. The strip beside a side face holds lanes stacked at the
# comb pitch between the face's stub-tip line and the nearest foreign
# copper (or the board edge) -- band_capacity's arithmetic on the strip's
# HEIGHT. PLAN_PAGES_STRIP = the price, in via units, of every lane a
# strip carries over its capacity PER PAGE: the side face's own berths
# plus the far face's berths on that half (their lanes go round that
# side); the near face has no strip (its lanes are the corridor's). 0 = off.
PAGES_STRIP = float(os.environ.get('PLAN_PAGES_STRIP', '0') or 0)
PAGES_STRIP_GAP = 0.30      # clearance to the foreign copper the strip ends at
PAGES_MISMATCH = float(os.environ.get('PLAN_PAGES_MISMATCH', '1') or 1)
# THE INSTANCE, WRITTEN OUT (2026-09-15, session 9): PLAN_PAGES_DUMP=<dir>
# writes every CP-SAT model this module solves, as built and hinted, to
# <dir>/<board>_solve<n>.pb (binary CpModelProto) beside a .json naming the
# nets and the parameters -- so the CHAIN'S OWN instance (in-process plan()
# takes a different greedy seed) can be re-solved offline against
# deterministic time, and the objective / bound curve read off it. Inert
# when unset: nothing in the solve changes.
PAGES_DUMP = os.environ.get('PLAN_PAGES_DUMP', '')
# THE BOXED-IN SWIMMER (2026-09-15, session 9). The damped re-solve frees the
# swimmers, bars each from the berth that swam, and holds every other net at
# its verified berth (a one-move menu) -- and a swimmer whose every remaining
# candidate is excluded by some held berth then has NO legal move, so the
# whole re-solve is INFEASIBLE and the loop keeps the first solve's plan.
# Measured across the session's logs: 132 of 251 K41 re-solves, 80 of 365
# at K51 (pg2 K41: SA1 17/17, SA15 17/17, SA8 14/14 candidates excluded).
# PLAN_PAGES_UNBLOCK=1: for such a swimmer, the held berths that box in its
# least-held candidate are un-held (their full menu restored; a berth the
# caller FIXED because it is already laid is never touched), so the small
# proven re-solve can move them. Off = byte-identical. Measured (ub1, K41):
# the blockers freed, the re-solve STILL infeasible -- no single hold boxes a
# freed net any more, but the freed nets' remaining candidates exclude one
# another (or a source pair, a learned pair), and the HARD bar on each
# swimmer's old berth leaves no joint assignment. =2: the bar is SOFT -- the
# old berth stays in the menu at one swimmer's price -- so the re-solve is
# always feasible (the verified plan is a solution) and moves what can move;
# the blockers are still freed (a net is boxed in when its UNBARRED
# candidates all are).
PAGES_UNBLOCK = int(os.environ.get('PLAN_PAGES_UNBLOCK', '0'))
# THE EXCHANGE RATE (2026-09-15, session 9; Andy: "one via is supposed to
# be 7.5 mm of length"). The repo's ONE rate is select_moves.VIA_MM = 7.5
# (the router's 75 grid units), and the judge (plan_ends), the realize
# loop's judged objective and the braid all convert at it. This objective
# never did: it kept the greedy's per-net ranking weights (VIA_W 3 a via,
# CHAN_W 2 a mm of channel, 1 a mm of reach), so inside the solve a via
# trades for 1.5 mm of channel or 3 mm of reach, and the length terms are
# 87% of the objective at K28 / K35 (README session 9). PLAN_PAGES_RATE=1:
# every length term is priced at VIA_W / VIA_MM per mm -- via units, the
# judge's rate. 0 = the greedy's weights, byte-identical.
PAGES_RATE = int(os.environ.get('PLAN_PAGES_RATE', '0')) or int(os.environ.get('PLAN_RATE', '0') or 0)
PAGES_PITCH = int(os.environ.get('PLAN_PAGES_PITCH', '0') or 0)   # verify: a pitch-infeasible page lane is a swimmer (braid.pitch_violations)
PAGES_PITCH_Q = float(os.environ.get('PLAN_PAGES_PITCH_Q', '0') or 0)   # its perpendicular room; 0 = TRACK + CLEAR + 0.05
# PLAN_PAGES_CERT (2026-09-15, THE PLAN item 2): 1 = a phase-A CERTIFICATE
# of the fewest swimmers (the swimmer count alone, under PLAN_PAGES_CERT_DET
# of deterministic time) caps the main solve; 2 = and phase A's plan is the
# main solve's hint. 0 = off, byte-identical. See _solve.
PAGES_CERT = int(os.environ.get('PLAN_PAGES_CERT', '0') or 0)
PAGES_CERT_DET = float(os.environ.get('PLAN_PAGES_CERT_DET', '30') or 30)
# PLAN_PAGES_WALK=r (2026-09-15, THE PLAN item 3): the TRUST-REGION WALK
# replaces the one big solve + damped re-solve. From the greedy seed, keyed
# exactly there, the CP-SAT proposes the objective-best plan that moves at
# most r ends; the braid's plan phase verifies it (the judge: PLAN_JUDGE's
# count, else residue + model vias); accepted iff better, then re-keyed
# there and repeated; rejected -> a no-good on that proposal and the next-
# best, up to PLAN_PAGES_WALK_TRIES; no accepted proposal at r -> r grows
# (bound PLAN_PAGES_WALK_RMAX); a solve that does not prove within
# PLAN_PAGES_WALK_DET shrinks r. Every budget is in deterministic time or
# counts (PLAN_PAGES_WALK_STEPS accepted steps, PLAN_PAGES_WALK_SOLVES
# solves in all), so the chain stays deterministic. 0 = off.
PAGES_WALK = int(os.environ.get('PLAN_PAGES_WALK', '0') or 0)
PAGES_WALK_RMAX = int(os.environ.get('PLAN_PAGES_WALK_RMAX', '8') or 8)
PAGES_WALK_TRIES = int(os.environ.get('PLAN_PAGES_WALK_TRIES', '3') or 3)
PAGES_WALK_STEPS = int(os.environ.get('PLAN_PAGES_WALK_STEPS', '20') or 20)
PAGES_WALK_SOLVES = int(os.environ.get('PLAN_PAGES_WALK_SOLVES', '40') or 40)
# PLAN_PAGES_WALK_STAGE=n (2026-09-15, session 13; THE PLAN item 4): the most
# solves ONE stage may take out of the run's budget. The budget is run-wide
# and the first stage is the one with everything to gain, so it spends the
# lot: measured on the K51 bench, stage 1 takes all 16 and every later stage
# -- the destination passes, which is where the berths are chosen against the
# teeth just realized -- prints "the run's solve budget is spent" and ships
# its reference unwalked. This caps a stage instead of raising the total, so
# the arm is budget-neutral and the question ("do the later passes move?")
# is asked without also asking "does more search help?". 0 = no cap.
PAGES_WALK_STAGE = int(os.environ.get('PLAN_PAGES_WALK_STAGE', '0') or 0)
PAGES_WALK_DET = float(os.environ.get('PLAN_PAGES_WALK_DET', '10') or 10)
PAGES_WALK_PROBE = int(os.environ.get('PLAN_PAGES_WALK_PROBE', '0') or 0)   # N proposals, no acceptance: the correlation probe
# PLAN_PAGES_WALK_FROM: where the walk starts -- 'seed' (the greedy seed, or
# the model-feasible plan nearest it) or 'solve' (the recorded loop's FREE
# first solve, verified: the walk as a REFINEMENT of the big solve's plan,
# re-keyed there, every step verified by the judge)
PAGES_WALK_FROM = os.environ.get('PLAN_PAGES_WALK_FROM', 'seed')   # seed | solve | damped (the loop's plan, walked after it)
# PLAN_PAGES_WALK_FALLBACK=1 (2026-09-15, session 12): the walk (from the
# seed or the solve) AND the damped loop both run, the judge keeps the
# better plan. The harness: the walk from the solve wins the crossing-heavy
# cases (reversed_k28 83 -> 54) and opens reversed_k15 (26/0 -> 18/2, the
# walk accepted nothing and shipped the raw first solve); the walk from the
# damped plan ships 0 open and finds none of those wins (341 against 298).
PAGES_WALK_FALLBACK = int(os.environ.get('PLAN_PAGES_WALK_FALLBACK', '0') or 0)
# PLAN_PAGES_WALK_RATE=1 (2026-09-15, session 12; THE PLAN's item 2): the
# walk's PROPOSALS are solved in the rate's units (VIA_MM, with a tooth's
# wrap round the source box priced) while the reference solve keeps the
# run's own units -- the rate in the one big solve displaces (jpR/jcR),
# the walk moves at most r ends. 0 = the run's units throughout.
PAGES_WALK_RATE = int(os.environ.get('PLAN_PAGES_WALK_RATE', '0') or 0)
# SRC_CLIMB_END_WALK=1 (2026-09-15, session 12): the end-of-face climbs
# (fanout_from_plan SRC_CLIMB_END) enter ONLY the walk's proposal solves,
# never the reference solve. In the one big solve they are just more
# candidates for a CP-SAT that does not converge at DET 40 (K28: 186 ->
# 234 tooth candidates, the feasible point obj 727 -> 737, the braid's
# count 123 -> 133, routed 34 -> 40 -- the s7 climb finding again); a
# walk proposal that takes one is verified by the judge before it lands.
# =2: every solve EXCEPT the first big one of a choose() call -- the damped
# loop's re-solves (the swimmers freed, their berths barred) see them too,
# where a braid swimmer's tooth can move instead of its berth.
PAGES_END_WALK = int(os.environ.get('SRC_CLIMB_END_WALK', '0') or 0)
# PLAN_PAGES_MENU=k / PLAN_PAGES_MENU_TOP=K (2026-09-15, session 12; Andy:
# "pre-filter down the likely best moves to a reasonable-sized and smart
# set, to let the solver reach the best solution within budget"): the
# solve's MENUS trimmed before the model is built -- per net, at most k
# candidates per (face, layer) class, the cheapest by the objective's own
# price, then at most K overall; the seed's own choice, a fixed / held /
# trust-reference move, the tooth as it stands and the end-of-face climbs
# are always kept. K51 today: 838 berths (~17 per net over 8 classes, the
# B dog-bone classes 3-5 deep where the F classes are 2), 40706
# exclusions, and the big solve stops FEASIBLE with a 61% gap at DET 40.
# 0 = the whole menu, byte-identical.
PAGES_MENU = int(os.environ.get('PLAN_PAGES_MENU', '0') or 0)
PAGES_MENU_TOP = int(os.environ.get('PLAN_PAGES_MENU_TOP', '0') or 0)
# PLAN_PAGES_MENU_STAGE=1: the pre-filtered menu is a FIRST STAGE -- its
# solve (small, converging) becomes the solution HINT of the full-menu
# solve, in place of the greedy seed; the full solve then improves from a
# good point instead of the greedy's. The trimmed menu alone risks cutting
# the candidate the optimum needs; the two stages keep the whole menu.
PAGES_MENU_STAGE = int(os.environ.get('PLAN_PAGES_MENU_STAGE', '0') or 0)
# PLAN_PAGES_MENU_PORTFOLIO=1: the trimmed-menu solve AND the full-menu
# solve at iteration 0, the JUDGE (the braid's count) picks -- the s9
# finding is that the model's better objective is not the copper's
# (more solve time gave a worse route at K41), so a pre-filtered menu is
# a cheap second candidate plan for the judge, not a replacement.
PAGES_MENU_PORTFOLIO = int(os.environ.get('PLAN_PAGES_MENU_PORTFOLIO', '0') or 0)
# PLAN_PAGES_SEEDS=n (2026-09-15, session 12): the first solve n times under
# different CP-SAT random seeds, every plan verified, the judge keeps the
# best. The full K28 model at DET 40 stops at obj 727.2 in most runs and
# 729.7 in some (the workers share solutions on the wall clock), and the
# 729.7 plan is judged 118.6 against 122.5 and routes 32 against 34: the
# feasible point the solver happens to stop at is worth as much as the
# menu, so ask for several and let the judge choose. 0/1 = one solve.
PAGES_SEEDS = int(os.environ.get('PLAN_PAGES_SEEDS', '0') or 0)
# PLAN_PAGES_GROUP=m (2026-09-15, session 12): the GROUP end-climb proposal
# after the damped loop -- for each destination side face and page, the
# outermost k (k <= m) berths' nets take the k end rows of the launch
# face in berth order (the outermost berth the outermost row), their
# climbs enumerated with the group's own teeth removed
# (fanout_from_plan.group_end_climbs), the composite plan verified ONCE
# and kept if the judge prefers it. The human's move (ten nets launched
# north on B in the order of their north-face berths); a single end
# climb never pays because its page-mates east of it still cross it
# (cew5x: SA15 climbed and still swam). 0 = off.
PAGES_GROUP = int(os.environ.get('PLAN_PAGES_GROUP', '0') or 0)
# (PLAN_PAGES_GROUP_FORCE, the probe that kept a realized group whatever the
# round judge said, is GONE. It did its job -- it proved the count judge
# wrong about a group by routing the board the judge threw away -- and
# `PLAN_PAGES_TIER_GROUP` then reproduced its result with copper IDENTICAL
# to it, on the copper's say-so instead of by switching the judge off. A
# flag that disables a safety check has no business outliving the question
# it answered; the measurement is in the README.)
# PLAN_PAGES_GROUP_DST=1 (2026-09-15, session 13; THE PLAN item 2): the group
# move at the DESTINATION as well. A group climb fixes its members' launch
# order by geometry -- the lane order, see _nest_assign -- and a launch order
# the berths do not follow is a swimmer per inversion, which is the whole
# reason a single end climb never paid. So the composite also re-berths its
# members, in the order they now launch. Measured on the K51 bench before it
# was built: the up/B group taken in BERTH order has no consistent assignment
# at all from m=3 up, because the berth order and the column order disagree;
# the forced head-on probe's 100 vias was this move for the DQ group, and no
# price in the solve could express it. 0 = off.
PAGES_GROUP_DST = int(os.environ.get('PLAN_PAGES_GROUP_DST', '0') or 0)
# PLAN_PAGES_PORTFOLIO=1 (2026-09-15): the FIRST solve of a plan is run under
# BOTH objectives -- the greedy's units and the rate (VIA_MM, with the source
# wrap) -- and the JUDGE (pf_key: the braid's count + its planned length)
# picks the plan the loop continues from. Measured on the s10 arms' final
# plans the judge ordered every K the way the copper did under the rule
# (K28/K41/K51 the greedy-unit plan, K35 the rate's), where each objective
# alone lost a rung (jpR 30/58/91+2o/118+3o, pg2 34/65/79/115). One extra
# DET-40 solve per plan.
PAGES_PORTFOLIO = int(os.environ.get('PLAN_PAGES_PORTFOLIO', '0') or 0)
# (the joined shift is derived per corridor in braid_slots: beyond the largest port term)   # destination exclusions by the STRICT conflict test (the engine lays the geometry asked; non-strict let 14 verbatim berths collide at K41)  # 1 = side-face berths keyed by the comb rule instead of _alt_slot (measured worse at K28: 2 -> 4 swimmers)    # 1 = when the swimmers alone cannot improve, free their crossers too (measured worse at K28: 2 -> 5)
import schedule as _schedule
import braid as te
# a row-line run and a column-line run on ONE layer that cross are a
# conflict whatever the moves' kinds -- select_moves tests it only for
# walked / climbed moves at its default SEL_XING=1 (K41 pages-first pass 0:
# 17 same-layer crossings of plain via-in-pad runs, laid as asked, DRC).
# The planner's exclusions use the full test; the standard planner's
# default is unchanged (its own SEL_XING=2 is the opt-in to measure).
sm.SEL_XING = max(sm.SEL_XING, 2)
# the two-page schedule assigned EXACTLY (BRAID_EXACT_PAGES): a plan two
# chains cover must be paged as two chains, which the greedy pager (the LIS
# of one page first) can miss. In-process for the judge; the plan sidecar
# carries `pages_first` so the braid stage does the same.
#
# HONOUR THE FLAG (review, 2026-09-17). This used to be a bare `= 1`,
# consulting no environment at all -- so `BRAID_EXACT_PAGES=0` could not
# turn it off ON THE PLANNER SIDE even though braid.py had been fixed to
# honour it. That matters because `judge_by_braid` calls the braid
# IN-PROCESS, and importing this module pins the judge's schedule to
# EXACT_PAGES=1 for the whole run: the planner then scores every
# candidate as ARM A of the braid portfolio while the chain frequently
# ships ARM B (the K51 log's "the marker-OFF arm won"). The marker is
# worth up to 14 vias, so the judge was systematically modelling the
# wrong regime on the rung where the deficit is worst.
# The braid SUBPROCESS was never affected -- braid.py does not import
# this module, so every ladder number measured through chain_k.sh stands.
_schedule.EXACT_PAGES = 0 if os.environ.get('BRAID_EXACT_PAGES') == '0' else 1
SCALE = 100                                                   # cost units -> ints


class SrcFrame(Frame):
    """The destination frame's mirror at the SOURCE: the same `t` axis (so
    `across` agrees at both ends), `u` reversed (the source's front face is
    the one facing the destination), half extents of the source box."""

    def __init__(self, fr: Frame, box):
        x0, y0, x1, y1 = box
        self.box = box
        self.c = ((x0 + x1) / 2, (y0 + y1) / 2)
        self.u = (-fr.u[0], -fr.u[1])
        self.t = fr.t
        hx, hy = (x1 - x0) / 2, (y1 - y0) / 2
        self.Hu = abs(hx * self.u[0]) + abs(hy * self.u[1])
        self.Ht = abs(hx * self.t[0]) + abs(hy * self.t[1])


def same_tooth(m: Move, c: Move, tol: float = 0.06) -> bool:
    """`m` is the tooth `c` as it stands: same kind, face and layer, exit
    within `tol` on both axes, and no climb or walk (a run the copper cannot
    tell apart from a plain stub, laid differently by the engine)."""
    return (m.kind == c.kind and m.direction == c.direction and m.layer == c.layer
            and not getattr(m, 'climb', 0) and not getattr(m, 'walk', 0)
            and abs(m.exit_pt[0] - c.exit_pt[0]) < tol and abs(m.exit_pt[1] - c.exit_pt[1]) < tol)


def face_strips(st, log=None):
    """The destination's side strips: {face: (capacity per page, height)}
    for every face but the one facing the source, and the far face's name
    (its berths load the side strip of their half). The height is the
    room from the face to the nearest foreign pad copper across the face's
    extent, or the board edge; the capacity is band_capacity's rule on it
    (one stub-tip margin, one clearance gap, the comb pitch)."""
    pcb = st['pcb']
    x0, y0, x1, y1 = st['dgrid'].bbox
    sx0, sy0, sx1, sy1 = st['sgrid'].bbox
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    vx, vy = (sx0 + sx1) / 2 - cx, (sy0 + sy1) / 2 - cy
    near = max(DIRS, key=lambda d: DIRS[d][0] * vx + DIRS[d][1] * vy)
    far = {'left': 'right', 'right': 'left', 'up': 'down', 'down': 'up'}[near]
    skip = {st['dref'], st['sref']}
    bb = pcb.board_info.board_bounds
    out = {}
    for d in DIRS:
        if d == near:
            continue
        per = {}
        for L in ('F.Cu', 'B.Cu'):
            # a page's lanes run on ONE layer: only copper on that layer (an
            # SMD pad on it, any through-hole pad) ends the strip for them
            h = 1e9
            if bb:
                h = {'up': y0 - bb[1], 'down': bb[3] - y1, 'left': x0 - bb[0], 'right': bb[2] - x1}[d]
            for ref, fp in pcb.footprints.items():
                if ref in skip:
                    continue
                for pad in fp.pads:
                    if pad.pad_type == 'np_thru_hole':
                        continue
                    lay = list(pad.layers or [])
                    if not (pad.drill > 0 or L in lay or any(l.startswith('*') and l.endswith('.Cu') for l in lay)):
                        continue
                    px0, px1 = pad.global_x - pad.size_x / 2, pad.global_x + pad.size_x / 2
                    py0, py1 = pad.global_y - pad.size_y / 2, pad.global_y + pad.size_y / 2
                    if d in ('up', 'down'):
                        if px1 < x0 or px0 > x1:
                            continue
                        g = (y0 - py1) if d == 'up' else (py0 - y1)
                    else:
                        if py1 < y0 or py0 > y1:
                            continue
                        g = (x0 - px1) if d == 'left' else (px0 - x1)
                    if g >= 0:
                        h = min(h, g)
            room = h - sm.BAND_TIP - PAGES_STRIP_GAP
            per[L] = (0 if room < 0 else int(room / sm.BAND_LPITCH + 1e-9) + 1, h)
        out[d] = per
    if log:
        log('  pages-first: strips ' + ', '.join(
            f'{d} F {per["F.Cu"][0]} ({per["F.Cu"][1]:.1f} mm) / B {per["B.Cu"][0]} ({per["B.Cu"][1]:.1f} mm)'
            for d, per in sorted(out.items())) + f'; near face {near}, far face {far}')
    return out, near, far


def strip_of(m: Move, near: str, far: str, bbox) -> Optional[str]:
    """Which side strip a berth's lane runs along: its own face for a side
    face, the neighbouring face of its half for the far face, none for the
    near face."""
    if m.direction == near:
        return None
    if m.direction != far:
        return m.direction
    x0, y0, x1, y1 = bbox
    if far in ('left', 'right'):
        return 'up' if m.exit_pt[1] < (y0 + y1) / 2 else 'down'
    return 'left' if m.exit_pt[0] < (x0 + x1) / 2 else 'right'


def current_tooth(st, nm) -> Optional[Move]:
    """The tooth AS IT STANDS on the board as a Move: what the engine laid,
    read off the copper (source_realize.measure_tooth). None for a net
    without a source pad on the source array or without copper there."""
    p = st['src_pad'].get(nm)
    if p is None or p.component_ref != st['sref']:
        return None
    g = sr.measure_tooth(st['pcb'], nm, p, st['byname'])
    if not g or g.get('direction') not in DIRS or g.get('layer') not in ('F.Cu', 'B.Cu'):
        return None
    return Move(net=nm, kind=g['kind'], direction=g['direction'], layer=g['layer'],
                exit_pt=tuple(g['tooth']), vias=g['vias'], legs=[],
                site=(tuple(g['site']) if g.get('site') else None))


def _cells(m: Move, mode: int) -> set:
    """The resource cells a candidate occupies (PLAN_PAGES_CELLS): two
    candidates that share one cannot both be laid."""
    tol = sm._EXIT_TOL
    reach = sm._VIA_REACH

    def q(v):
        c = int(math.floor(v / tol))
        return (c, c + 1)

    def rng(lo, hi):
        return range(int(math.floor(lo / tol)), int(math.floor(hi / tol)) + 1)
    out = set()
    for key, a, b in sm._lane_spans(m):
        axis, pos, L = key
        # a row run's cells are (x along, y across); a column's (x across, y along)
        for c in q(pos):
            for k in rng(a, b):
                cx, cy = (k, c) if axis == 'row' else (c, k)
                out.add(('p', L, cx, cy))
    if m.site is not None:
        sx, sy = m.site
        for L in ('F.Cu', 'B.Cu'):
            for cx in rng(sx - reach, sx + reach):
                for cy in rng(sy - reach, sy + reach):
                    out.add(('p', L, cx, cy))
    for cx in q(m.exit_pt[0]):
        for cy in q(m.exit_pt[1]):
            out.add(('x', cx, cy))
    return out


def _cell_groups(cands: Dict[str, List[Move]], mode: int) -> List[List[Tuple[str, int]]]:
    """Every cell's occupants (net, index), cells with two or more, in a
    canonical order (the model is built in it)."""
    occ: Dict[tuple, set] = {}
    for nm, ms in cands.items():
        for i, m in enumerate(ms):
            for c in _cells(m, mode):
                occ.setdefault(c, set()).add((nm, i))
    out = []
    for c in sorted(occ, key=repr):
        if len(occ[c]) > 1:
            out.append(sorted(occ[c]))
    return out


def _conflicts(cands: Dict[str, List[Move]], strict: bool) -> List[Tuple[str, int, str, int]]:
    """Every (net a, index, net b, index) whose two moves cannot both be
    laid, tested only between moves sharing a lane, a site or an exit
    (bucketed: the all-pairs test is 10^6-10^7 calls at K41)."""
    buckets: Dict[tuple, List[Tuple[str, int]]] = {}
    for nm, ms in cands.items():
        for i, m in enumerate(ms):
            keys = set()

            def q(v, tol=sm._EXIT_TOL):
                # two values within `tol` of each other share at least one
                # of these two cells (a single rounding can split them)
                c = int(math.floor(v / tol))
                return (c, c + 1)
            for key, a_, b_ in sm._lane_spans(m):
                cells = q(key[1]) if strict else (round(key[1], 3),)
                for c in cells:
                    keys.add(('lane', key[0], c, key[2]))
                # a row run and a column run on ONE layer that CROSS share no
                # lane, site or exit key, so bucket them by the 1 mm cells the
                # row run spans and the cell the column run stands in
                # (2026-09-14: 14 such pairs at K41 pass 0 were never tested)
                if key[0] == 'row':
                    for cx in range(int(math.floor(a_)), int(math.floor(b_)) + 1):
                        keys.add(('xc', key[2], cx))
                else:
                    keys.add(('xc', key[2], int(math.floor(key[1]))))
            if m.site is not None:
                # a site in another move's lane: bucket the lane through the
                # site by its line, on both layers (a via spans them all)
                keys.add(('site', round(m.site[0], 2), round(m.site[1], 2)))
                for axis, v in (('row', m.site[1]), ('col', m.site[0])):
                    cells = q(v) if strict else (round(v, 3),)
                    for c in cells:
                        for L in ('F.Cu', 'B.Cu'):
                            keys.add(('lane', axis, c, L))
            for cx in q(m.exit_pt[0]):
                for cy in q(m.exit_pt[1]):
                    keys.add(('exit', cx, cy))
            # SORTED: `keys` is a set of string-keyed tuples, whose iteration
            # order follows the process's hash seed -- and the ORDER the
            # exclusions reach the CP-SAT decides which FEASIBLE answer a
            # deterministic-time solve lands on (measured 2026-09-14, K41:
            # one process gave obj 1784, the next 1727, the next 1809 on
            # one model; in one process three solves agreed). The model
            # must be built in a canonical order.
            for k in sorted(keys, key=repr):
                buckets.setdefault(k, []).append((nm, i))
    seen = set()
    out = []
    for members in buckets.values():
        if len(members) < 2:
            continue
        for x in range(len(members)):
            a, i = members[x]
            for y in range(x + 1, len(members)):
                b, j = members[y]
                if a == b or (a, i, b, j) in seen or (b, j, a, i) in seen:
                    continue
                seen.add((a, i, b, j))
                if sm._conflict(cands[a][i], cands[b][j], strict=strict):
                    out.append((a, i, b, j))
    return out


def braid_slots(st, board, names, seed, src_seed, D, S, log):
    """Every candidate's place in the BRAID'S OWN order, the other lanes
    held at the seed plan: the corridors built on the seed (setup +
    Corridor.run(plan_only), 0.4 s), then each destination candidate's slot
    by `_alt_geo` + `_alt_slot` (a head-on exit's own offset, else inserted
    into its side's exit comb by the comb's rule) and each tooth candidate's
    by `_alt_src_slot` (its own offset, else into its side's join block);
    the tooth as it stands keeps `launch_o`. Returns (tkeys, lkeys,
    corridor index per net); a net the braid did not plan gets None."""
    import fanout_from_plan as F
    st_seed = st
    for nm, mv in (src_seed or {}).items():
        st_seed = F._st_with_src(st_seed, nm, mv)
    plan = F.braid_plan_of(st_seed, seed, board)
    plan['pages_first'] = True          # the braid's rules for a pages-first plan (siders, exact pages)
    quiet = lambda m='': None
    ctx, groups = te.setup(board, list(seed), st['dref'], quiet, plan=plan)
    corridors = [te.Corridor(ci, M, ctx, quiet) for ci, M in enumerate(groups)]
    ctx.corridors = corridors
    for c in corridors:
        try:
            c.run(plan_only=True)
            ctx.laid.extend(c.lane_xy[nm] for nm in c.members if nm in getattr(c, 'lane_xy', {}))
            if getattr(c, 'spine_core', None) is not None:
                ctx.laid_tubes.append((c.spine_core.pts, c.H))
        except Exception as e:
            log(f'  pages-first: corridor {c.idx} not planned ({e})')
    tkeys, lkeys, corr = {}, {}, {}
    jkeys, joiner = {}, {}
    errs: Dict[str, list] = {}       # net -> [(end, candidate, exception)] for the candidates no key could be built for          # PAGES_JOINKEY: the joined variant of each dest key; joiner flag per source candidate
    for c in corridors:
        if not hasattr(c, 'launch_o') or not hasattr(c, 'target_o'):
            continue
        # the braid's orders ARE its offsets sorted: say so if not
        lo = [c.launch_o[n] for n in c.launch if n in c.launch_o]
        to = [c.target_o[n] for n in c.target if n in c.target_o]
        if lo != sorted(lo) or to != sorted(to):
            log(f'  pages-first: WARNING corridor {c.idx} orders are not its offsets sorted')
        for nm in c.members:
            if nm not in D:
                continue
            corr[nm] = c.idx
            tk = []
            jk = []
            sp = c.spine
            dn = sp.d[-1]                      # the spine's direction at the destination
            ref = c.ctx.ends[nm][2]
            # the joined lanes sit beyond EVERY port of their side: the shift
            # is the largest port term any candidate of this corridor can
            # take (half the farthest stub's distance past s1) plus a pitch --
            # a fixed number would be a scale read off one board
            s_far = max([sp.project_pt(tuple(mv.exit_pt))[0] for om in c.members if om in D
                         for mv in D[om]] + [c.s1])
            join_shift = 0.5 * max(0.0, s_far - c.s1) + te.LPITCH
            for m in D[nm]:
                try:
                    g = c._alt_geo(nm, {'exit': list(m.exit_pt), 'layer': m.layer, 'dir': list(DIRS[m.direction])})
                    d_ = DIRS[m.direction]
                    along = d_[0] * dn[0] + d_[1] * dn[1]
                    # mode 2: a side-face candidate is keyed by the comb whether
                    # or not it tests head-on on the seed -- among a face's
                    # stubs the order is by position either way, and the one
                    # the braid makes head-on (the most upstream) is innermost
                    if PAGES_SIDEKEY and (not g.get('head') or (PAGES_SIDERS_MODE == 2 and abs(along) < 0.5)):
                        # a SIDE-FACE stub (its direction across the spine).
                        # The braid's relative head-on test makes the most
                        # upstream stub of the face head-on and every other
                        # one a side exit, so a candidate's class -- and its
                        # slot -- flips with which neighbours are chosen;
                        # with no stub of the face in the seed EVERY candidate
                        # tested head-on at the face line, one key for the
                        # whole face. The ORDER the braid ends up with is the
                        # comb's whatever the class: beyond the front lanes on
                        # that side, the first exiter innermost. Key it so.
                        # ONE formula for every side exit of a side, the far
                        # face included: beyond that side's head-on lanes, then
                        # nested by position along the spine (the comb's rule,
                        # first exiter innermost, the far face -- the largest s
                        # -- outermost). Two scales (the insertion slot for the
                        # far face, this for the side faces) could not be
                        # compared: every remaining K28 disagreement was a
                        # far-face berth against a side-face comb.
                        s_e, o_e = g['se']
                        sg = g['side'] if g.get('side') else c._side_of(o_e, ref)
                        ext = max([sg * c.target_o[om] for om in getattr(c, 'heads_e', ()) if om != nm]
                                  + [sg * abs(o_e) * 0 + 0.0])
                        tk.append(float(sg * (ext + te.BLOCK_GAP) + sg * 0.5 * max(0.0, s_e - c.s1)))
                        # the JOINED variant: a lane whose tooth is a joiner
                        # sits outermost of its exit block, the joiners
                        # ordered by their tooth's s (the nearest the
                        # corridor innermost) -- offsets(): order = ports + joined
                        s_t = c.st[nm][0] if nm in c.st else c.s0
                        jk.append(float(sg * (ext + te.BLOCK_GAP + join_shift + 0.5 * max(0.0, c.s0 - s_t))))
                    else:
                        v = float(c._alt_slot(nm, g))
                        tk.append(v); jk.append(v)
                except Exception as e:
                    tk.append(None); jk.append(None)
                    errs.setdefault(nm, []).append(('dst', sr.fmt_ask(m), repr(e)))
            tkeys[nm] = tk
            jkeys[nm] = jk
            # the source class per candidate: the tooth as it stands by the
            # corridor's own classification, a candidate by _alt_geo
            jf = []
            for i, s in enumerate(S[nm]):
                if i == 0 and not s.legs:
                    jf.append(nm in getattr(c, 'join_block', {}))
                    continue
                try:
                    g = c._alt_geo(nm, {'end': 'src', 'exit': list(s.exit_pt), 'layer': s.layer})
                    jf.append(not g['head'])
                except Exception:
                    jf.append(False)
            joiner[nm] = jf
            lk = []
            for i, s in enumerate(S[nm]):
                if i == 0 and not s.legs:
                    lk.append(float(c.launch_o.get(nm, 0.0)))
                    continue
                try:
                    g = c._alt_geo(nm, {'end': 'src', 'exit': list(s.exit_pt), 'layer': s.layer})
                    lk.append(float(c._alt_src_slot(nm, g)))
                except Exception as e:
                    lk.append(None)
                    errs.setdefault(nm, []).append(('src', sr.fmt_ask(s), repr(e)))
            lkeys[nm] = lk
    braid_slots.extra = (jkeys, joiner)
    braid_slots.errors = errs
    if errs:
        log(f'  pages-first: {len(errs)} net(s) with candidates the braid could not key: '
            + '; '.join(f'{nm} {len(v)} ({v[0][0]} {v[0][1]}: {v[0][2][:80]})' for nm, v in sorted(errs.items())))
    return tkeys, lkeys, corr


def verify(st, board, names, dst_choice, src_choice):
    """The braid's own planner on the chosen plan (exact pages): the nets it
    still swims. This is what the chain will do, so it is the verdict."""
    import fanout_from_plan as F
    st2 = st
    for nm, mv in src_choice.items():
        st2 = F._st_with_src(st2, nm, mv)
    plan = F.braid_plan_of(st2, dst_choice, board)
    plan['pages_first'] = True
    bp = te.plan_braid(board, list(dst_choice), st['dref'], plan)
    if PAGES_PITCH:
        # PLAN_PAGES_PITCH (2026-09-15): a page lane whose planned pair is
        # geometrically infeasible (braid.pitch_violations) is a lane the
        # braid will refuse in band; it is treated here exactly like a lane
        # the braid swims -- page None -- so the damped loop re-plans it
        # with that berth barred, the key counts it, and the judge prices
        # it as a swimmer
        viol = te.pitch_violations(bp, PAGES_PITCH_Q or None)
        for nm in viol:
            if nm in bp and bp[nm].get('page') is not None:
                bp[nm]['page'] = None
                bp[nm]['pitch_violation'] = viol[nm]
        verify.last_pitch = viol
    swim = [nm for nm in dst_choice if bp.get(nm, {}).get('page') is None]
    verify.last_islands = ({nm: bp[nm]['islands'] for nm in dst_choice
                            if bp.get(nm, {}).get('page') is not None and bp[nm].get('islands')}
                           if PAGES_ISLAND else {})
    # PLAN_JUDGE: the braid's plan-implied count of this plan (the teeth
    # as chosen, priced on the same planner answer); None as recorded
    cost = F.judge_by_braid(st2, dst_choice, board, bp=bp)[0] if F.PLAN_JUDGE else None
    return swim, bp, cost


_WALK_BUDGET = {'solves': None}     # ONE budget per run, shared by every choose() call (the re-plan passes call it again)


def _pairs_off(names, bp):
    """The model-vs-braid KEY disagreement on a verified plan (section E's
    instrument): over the pairs the braid put in one corridor, how many
    the last solve's keys (L, T) order differently from the braid's slots
    (launch_idx, target_idx). Returns (off, pairs) or None."""
    keys = getattr(_solve, 'last_keys', None)
    if not keys:
        return None
    L, T = keys['L'], keys['T']
    ns = [n for n in names if n in L and bp.get(n, {}).get('launch_idx') is not None
          and bp[n].get('target_idx') is not None]
    off = tot = 0
    for i, a in enumerate(ns):
        for b in ns[i + 1:]:
            if bp[a]['corridor'] != bp[b]['corridor']:
                continue
            tot += 1
            # does the MODEL think the pair crosses where the braid does not,
            # or the reverse? (orientation-invariant: a mirrored frame flips
            # every order and no crossing)
            x_model = (L[a] < L[b]) != (T[a] < T[b])
            x_braid = (bp[a]['launch_idx'] < bp[b]['launch_idx']) != (bp[a]['target_idx'] < bp[b]['target_idx'])
            if x_model != x_braid:
                off += 1
    return off, tot


def _spearman(xs, ys):
    n = len(xs)
    if n < 3:
        return float('nan')

    def rk(v):
        order = sorted(range(n), key=lambda i: v[i])
        out = [0.0] * n
        i = 0
        while i < n:
            j = i
            while j + 1 < n and v[order[j + 1]] == v[order[i]]:
                j += 1
            for k in range(i, j + 1):
                out[order[k]] = (i + j) / 2 + 1
            i = j + 1
        return out
    rx, ry = rk(xs), rk(ys)
    mx, my = sum(rx) / n, sum(ry) / n
    num = sum((p - mx) * (q - my) for p, q in zip(rx, ry))
    den = (sum((p - mx) ** 2 for p in rx) * sum((q - my) ** 2 for q in ry)) ** 0.5
    return num / den if den else float('nan')


def _nest_assign(group, cands, ax, ed, od, edge, taken_rows, held=(), budget=20000):
    """PLAN_PAGES_GROUP, the CONSISTENT assignment: one climb per member of
    `group`, such that

      * the exit rows NEST BY LANE -- the member whose lane runs FURTHEST
        FROM the face edge exits furthest toward the end of the face, and
        each next lane out exits one row nearer the bundle;
      * no two members share an exit row (two teeth at one point is no
        order at all), and none takes a row a standing tooth or an earlier
        group already holds (`taken_rows`);
      * no two members contend for the same room
        (`source_realize.moves_clash`): distinct lanes, and no run through
        another member's barrel;
      * no member contends with a climb an EARLIER group of the same call
        already holds (`held`). Every accepted group is laid in one engine
        call, so consistency inside a group is not enough: at K51 three
        groups accepted in one stage (12 members) collided and six of them
        were degraded, the same failure one group down.

    **The nesting is geometry, not preference, and it is not the berth
    order.** A member's leg out to the face crosses every lane beyond it
    that is still running at that row, so of two members the inner one must
    leave first; the launch order of a group is therefore fixed by the
    COLUMNS its balls sit in. Measured on the K51 bench: the up/B group
    taken in berth order (SA12, SBA1, SA15, ...) has NO consistent
    assignment at m >= 3, because SA15's lane is west of the other two and
    its berth is east of them. The berth order is the destination's to
    give -- that is the re-berth (`PLAN_PAGES_GROUP_DST`), not this.

    The members are enumerated with each other's copper stripped (they are
    all about to be re-fanned together), so this is the ONLY place their
    collisions can be seen before the engine meets them -- and meeting them
    there is what made the K51 smoke lay 9 of 17 climbs in the asked gap.

    DFS in lane order, fewest blockers first, at most `budget` nodes.
    Returns {net: Move}, or None when the group has no consistent set."""
    import fanout_from_plan as F   # (`sr` is a module-level import; `F` cannot be)
    pos = {n: (sum(F._lane_of(m, ax) for m in cands.get(n, ())) / len(cands[n]))
           for n in group if cands.get(n)}
    if len(pos) < len(group):
        return None
    seq = sorted(group, key=lambda n: (od * pos[n], group.index(n)))
    # the candidate order: FEWEST BLOCKERS first (a climb that needs six
    # other teeth stripped is re-laying six teeth with no hint, and that
    # scatter is what the count judge then charges the group for), then the
    # row nearest the bundle, then the innermost lane
    order = {n: sorted(cands[n], key=lambda m: (max(m.blockers, 0),
                                                abs(m.exit_pt[ax] - edge),
                                                od * F._lane_of(m, ax)))
             for n in seq}
    seen = [0]

    def rec(i, prev, used, chosen):
        if i == len(seq):
            return dict(chosen)
        for mv in order[seq[i]]:
            seen[0] += 1
            if seen[0] > budget:
                return None
            v = round(mv.exit_pt[ax], 2)
            k = -ed * mv.exit_pt[ax]
            if v in used or (prev is not None and k <= prev + 1e-9):
                continue
            if any(sr.moves_clash(mv, m2) for m2 in chosen.values()):
                continue
            if any(sr.moves_clash(mv, h) for h in held):
                continue
            chosen[seq[i]] = mv
            got = rec(i + 1, k, used | {v}, chosen)
            if got is not None:
                return got
            del chosen[seq[i]]
        return None
    return rec(0, None, set(taken_rows), {})


def _regroup_berths(st, dst, group, assigned, fr, page, log):
    """PLAN_PAGES_GROUP_DST: the group's members re-berthed in the order their
    climbs now LAUNCH.

    A berth is not a slot that can be handed to another net -- it is an escape
    of that net's OWN ball -- so this is not a permutation of berths but a
    small monotone re-choice: each member picks from its own destination menu,
    taken in the new launch order, so that the frame keys come out
    non-decreasing (which is exactly "no swimmer among them"). Every candidate
    is held inside the key BAND the group already occupies, so the re-berth
    cannot invert the group against a net outside it, and each member's
    standing berth is always a candidate -- the re-choice can decline.

    Returns {net: Move} for the members whose berth changed."""
    import sched_first as sf
    keys = [fr.key(dst[n]) for n in group]
    lo_k, hi_k = min(keys), max(keys)
    _names, _fr, _order, cost = sf._setup(st, st['dmenu'])
    seq = sorted(group, key=lambda n: fr.across(assigned[n].exit_pt))
    cands = {}
    for n in seq:
        here = sr.move_sig(dst[n])
        ms = [m for m in st['dmenu'].get(n, ())
              if lo_k - 1e-9 <= fr.key(m) <= hi_k + 1e-9 or sr.move_sig(m) == here]
        if not any(sr.move_sig(m) == here for m in ms):
            ms.append(dst[n])
        cands[n] = [(m, fr.key(m), cost(n, m, page)) for m in ms]
    choice, skipped, _total = sf.monotone_assign(seq, cands, swim=1e6)
    if skipped or len(choice) < len(seq):
        return {}
    out = {n: m for n, m in choice.items() if sr.move_sig(m) != sr.move_sig(dst[n])}
    if out:
        log(f'  pages-first: group climb: re-berth in launch order {seq}: '
            + ', '.join(f'{n} {dst[n].direction}/{dst[n].layer[0]}'
                        f'@{fr.key(dst[n]):.2f} -> {out[n].direction}/{out[n].layer[0]}'
                        f'@{fr.key(out[n]):.2f}' for n in seq if n in out))
    return out


def _group_climb(st, board, log, best):
    """PLAN_PAGES_GROUP: see the flag. `best` = (key, dst, src, model,
    swim, bp) as choose keeps it; returns (best, report lines)."""
    import fanout_from_plan as F
    from escape_moves import DIRS, LAYERS
    rep = []
    names = [n for n in st['launch'] if st['dmenu'].get(n)]
    key0, dst, src, model, swim, bp = best
    dbox = st['dgrid'].bbox
    fr = Frame({n: st['launch'][n] for n in names}, dbox)
    sg = st['sgrid']
    x0, y0, x1, y1 = sg.bbox
    hx, hy = sg.pitch_x / 2.0, sg.pitch_y / 2.0

    def face_of(pt):
        d = {'left': abs(pt[0] - (x0 - hx)), 'right': abs(pt[0] - (x1 + hx)),
             'up': abs(pt[1] - (y0 - hy)), 'down': abs(pt[1] - (y1 + hy))}
        return min(d, key=d.get)
    cnt = {}
    for n in names:
        cnt[face_of(st['launch'][n])] = cnt.get(face_of(st['launch'][n]), 0) + 1
    face = max(cnt, key=cnt.get)
    ax = 1 if face in ('left', 'right') else 0
    od = DIRS[face][1 - ax]            # the OUTWARD sign across the face
    # rows already taken on the launch face: the standing launches, plus
    # every accepted group's rows (two teeth cannot share one exit point,
    # whatever their layers -- the smoke's up/F and up/B groups were both
    # handed rows 57.43-58.73 and the engine degraded half of them)
    on_face = [n for n in names if face_of(st['launch'][n]) == face]
    held_rows = set()       # rows an ACCEPTED group already took this call
    held_moves = []         # and their climbs: every group is laid in ONE call
    tried = 0
    gid = 0
    for dface in [d for d in DIRS if DIRS[d][ax] != 0]:
        end = 'lo' if DIRS[dface][ax] < 0 else 'hi'
        ed = -1 if end == 'lo' else 1      # the sign of "toward the end"
        for page in LAYERS:
            members = [n for n in names if n in dst and dst[n].direction == dface
                       and (bp.get(n, {}).get('page') or dst[n].layer) == page]
            if len(members) < 2:
                continue
            members.sort(key=lambda n: -abs(fr.key(dst[n])))      # outermost first
            cands = F.group_end_climbs(st, members[:2 * PAGES_GROUP], face, end)
            rep.append(f'  pages-first: group climb: {dface}/{page[0]} berths outermost first '
                       + ' '.join(f'{n}:{len(cands.get(n, []))}' for n in members[:2 * PAGES_GROUP]))
            # the outermost members THAT HAVE a climb (a member without one
            # keeps its tooth; the judge says whether the rest still pay)
            climbers = [n for n in members if cands.get(n)]
            m_max = min(len(climbers), PAGES_GROUP)
            for m in range(m_max, 1, -1):
                group = climbers[:m]
                # the rows this group must avoid: every tooth still
                # STANDING on the launch face (its own members are about to
                # be stripped, so their old rows are free) and every row an
                # accepted group already took
                standing = [st['launch'][n][ax] for n in on_face if n not in group]
                if not standing:
                    continue
                taken_rows = held_rows | {round(v, 2) for v in standing}
                edge = min(standing) if end == 'lo' else max(standing)
                assigned = _nest_assign(group, cands, ax, ed, od, edge, taken_rows,
                                        held=held_moves)
                if assigned is None:
                    rep.append(f'  pages-first: group climb: {dface}/{page[0]} outermost {m} {group}: '
                               f'no consistent assignment (nested rows, distinct lanes, no shared room)')
                    continue
                tried += 1
                gid += 1
                tag = f'{dface}/{page[0]}/{gid}'
                for _n, _mv in assigned.items():
                    _mv.group = tag          # laid all or nothing (fanout_from_plan)
                    _mv.replaces = src.get(_n)   # what to fall back to if it is dropped
                src2 = dict(src)
                src2.update(assigned)
                dst2 = dst
                if PAGES_GROUP_DST:
                    reb = _regroup_berths(st, dst, group, assigned, fr, page, rep.append)
                    if reb:
                        dst2 = dict(dst)
                        dst2.update(reb)
                swim2, bp2, cost2 = verify(st, board, names, dst2, src2)
                key2 = F.pf_key(dst2, bp2, cost2, model.get('vias'))
                better = F.pf_better(key2, key0)
                rep.append(f'  pages-first: group climb: {dface}/{page[0]} outermost {m} {group} -> {face} rows '
                           f'{[round(assigned[n].exit_pt[ax], 2) for n in group]} '
                           f'(lane order {[n for n in sorted(assigned, key=lambda q: ed * assigned[q].exit_pt[ax])][::-1]}, '
                           f'{sum(max(assigned[n].blockers, 0) for n in group)} blocker-slot(s)); the braid swims {len(swim2)}'
                           + (f', count {cost2:.0f}' if cost2 is not None else '')
                           + f'; key {key2} vs {key0}: ' + ('ACCEPTED' if better else 'rejected'))
                if better:
                    key0, dst, src, swim, bp = key2, dst2, src2, swim2, bp2
                    model = dict(model, count=cost2)
                    held_rows |= {round(assigned[n].exit_pt[ax], 2) for n in group}
                    held_moves += [assigned[n] for n in group]
                    break
                for _mv in assigned.values():
                    _mv.group = ''          # a rejected proposal is not a group
    if not tried:
        rep.append('  pages-first: group climb: no group with climbs for every member')
    return (key0, dst, src, model, swim, bp), rep


def _mv_tag(n, k, dst, src):
    """A moved end in the walk's log, with WHERE it went: face/layer, `^` a
    climb, `*` an end-of-face climb (SRC_CLIMB_END)."""
    m = dst.get(n) if k == 'd' else src.get(n)
    if m is None:
        return f'{n}:{k}'
    return (f'{n}:{k}>{m.direction[0]}{m.layer[0]}'
            + ('*' if getattr(m, 'end_climb', False) else ('^' if getattr(m, 'climb', 0) else '')))


def _walk(st, board, log, fixed, learned, src_free, seed, ref=None):
    """THE PLAN item 3: the trust-region walk (PLAN_PAGES_WALK), after the
    solve review of 2026-09-15: the reference is the seed COMPLETED by a
    radius-0 solve (its objective and model swimmers read off that solve,
    then verified by the braid); the cuts stay until the reference moves;
    r resets to the base on acceptance, shrinks when a solve does not
    prove, grows when the region is proven exhausted or TRIES proven
    proposals were rejected; the swimmer cap is the reference's own
    model count; one solve budget per run. PLAN_PAGES_WALK_PROBE=N:
    acceptance off, N proposals at the base radius, and Spearman(d obj,
    d count) at the end -- does the model's ranking point the braid's way?
    Returns (dst_choice, src_choice, report) like `choose`."""
    import fanout_from_plan as F
    global PAGES_DET
    rep = []
    names = [n for n in st['launch'] if st['dmenu'].get(n)]
    seed_d = {n: mv for n, mv in (seed or {}).items() if n in st['dmenu']}
    if _WALK_BUDGET['solves'] is None:
        _WALK_BUDGET['solves'] = PAGES_WALK_SOLVES
    r0 = max(1, PAGES_WALK)
    r = r0
    nogoods = []
    steps = solves = tries = 0
    # this STAGE's share of the run budget (PLAN_PAGES_WALK_STAGE)
    stage_left = [PAGES_WALK_STAGE if PAGES_WALK_STAGE else 10 ** 9]

    def spend():
        _WALK_BUDGET['solves'] -= 1
        stage_left[0] -= 1

    def budget_left():
        return min(_WALK_BUDGET['solves'], stage_left[0])
    cap = None
    probe = []
    best = None
    det0 = PAGES_DET
    PAGES_DET = PAGES_WALK_DET

    def solve_at(rr, ref_d, ref_s, plan):
        return _solve(st, board, log, dict(fixed or {}), learned, src_free, plan, ref_s,
                      trust=(ref_d, ref_s, rr), nogoods=nogoods, hard_fixed=fixed, swim_cap=cap,
                      rate=(1 if PAGES_WALK_RATE else None),
                      end_climbs=(rr not in (0, None)))       # a PROPOSAL, not the reference

    def fmt(model, swim, cost, key, po):
        return (f'obj {model["obj"]:.1f} {model["status"]}, model swims {model["swim"]}; '
                f'the braid swims {len(swim)}' + (f', count {cost:.0f}' if cost is not None else '')
                + f'; key {key}' + (f'; pairs off {po[0]}/{po[1]}' if po else ''))
    try:
        # step 0: the reference = the greedy seed, completed at the cheapest
        # berth where it left a net unplaced, VERIFIED by the braid. The model
        # is asked for it at radius 0 as a DIAGNOSTIC only: its objective and
        # model swimmers when the seed is a model solution, else the
        # exclusions the seed violates (K8: five pairs the engine laid and
        # routed clean at the human's count -- the model's legality is
        # stricter than the engine's, so the seed is not always in the model)
        if ref is None:
            ref = dict(seed_d)
            dbox = st['dgrid'].bbox
            for n in names:
                if n not in ref:
                    ref[n] = min(st['dmenu'][n], key=lambda mv: (
                        VIA_W * (mv.vias + (sm._length(mv) + sm.around_box(st['launch'][n], mv.exit_pt, dbox)) / sm.VIA_MM)
                        if PAGES_RATE else
                        VIA_W * mv.vias + CHAN_W * sm._length(mv) + sm.around_box(st['launch'][n], mv.exit_pt, dbox)))
            ref_sig = {n: sr.move_sig(mv) for n, mv in ref.items()}
            how = 'the seed'
            if PAGES_WALK_FROM == 'solve':
                # the FREE first solve (the recorded loop's, at PLAN_PAGES_DET) is
                # the reference: the walk refines the big solve's plan
                PAGES_DET = det0
                dst0, src0, lines, model0 = _solve(st, board, log, dict(fixed or {}), learned, src_free,
                                                   seed_d, {}, hard_fixed=fixed)
                PAGES_DET = PAGES_WALK_DET
                rep += lines
                solves += 1
                spend()
                if dst0:
                    how = f'the free first solve ({len(src0)} teeth to move)'
                    ref, src_ref = dst0, src0
                    ref_sig = {n: sr.move_sig(mv) for n, mv in ref.items()}
            else:
                dst0, src0, lines, model0 = solve_at(0, ref_sig, {}, ref)
                rep += lines
                solves += 1
                spend()
            if not dst0:
                # the seed is not a model solution: the reference is the model-
                # feasible plan NEAREST it (one proximity solve, radius None)
                dst0, src0, lines, model0 = solve_at(None, ref_sig, {}, ref)
                rep += lines
                solves += 1
                spend()
                if dst0:
                    mv0 = model0.get('moved', [])
                    how = (f'the model-feasible plan nearest the seed ({len(mv0)} end(s) moved: '
                           + ', '.join(f'{n}:{k}' for n, k, _s in mv0) + ')')
                    ref, src_ref = dst0, src0
                    model0 = dict(model0, obj=(model0['obj'] - len(mv0) * 10000))   # the cost part alone
                else:
                    rep.append('  pages-first: walk: the model has NO solution near the seed (proximity solve infeasible)')
                    src_ref = {}
            elif PAGES_WALK_FROM != 'solve':
                src_ref = {}
            swim, bp, cost = verify(st, board, names, ref, src_ref)
            if dst0:
                model = dict(model0, count=cost)
                cap = model0['swim']
            else:
                model = {'vias': cost if cost is not None else 0, 'swim': None, 'obj': None,
                         'status': 'not a model solution', 'value': {}, 'moved': [], 'count': cost}
            key = F.pf_key(ref, bp, cost, model0['vias'] if dst0 else None)
            best = (key, ref, src_ref, model, swim, bp)
            rep.append(f'  pages-first: walk: reference = {how}'
                       + (' (unplaced nets completed)' if any(n not in seed_d for n in names) else '')
                       + (f': obj {model0["obj"]:.1f} {model0["status"]}, model swims {model0["swim"]}' if dst0
                          else ': NOT a model solution (radius 0 infeasible; its objective is unknown)')
                       + f'; the braid swims {len(swim)}' + (f', count {cost:.0f}' if cost is not None else '')
                       + f'; key {key}' + ((lambda po: f'; pairs off {po[0]}/{po[1]}' if po else '')(_pairs_off(names, bp)) if dst0 else ''))
        else:
            # the caller's verified plan (choose's damped loop) as the reference
            best = ref
            cap = ref[3].get('swim')
            rep.append(f'  pages-first: walk: reference = the damped loop\'s plan: model swims {cap}; '
                       f'the braid swims {len(ref[4])}'
                       + (f', count {ref[3]["count"]:.0f}' if ref[3].get('count') is not None else '')
                       + f'; key {ref[0]}')
        if budget_left() <= 0:
            rep.append('  pages-first: walk: the '
                       + ('stage' if stage_left[0] <= 0 < _WALK_BUDGET['solves'] else 'run')
                       + '\'s solve budget is spent -- the reference ships')
        while (steps < PAGES_WALK_STEPS and budget_left() > 0 and r <= PAGES_WALK_RMAX
               and (not PAGES_WALK_PROBE or len(probe) < PAGES_WALK_PROBE)):
            ref_d = {n: sr.move_sig(mv) for n, mv in best[1].items()}
            ref_s = dict(best[2])
            dst, src, lines, model = solve_at(r, ref_d, ref_s, best[1])
            rep += lines
            solves += 1
            spend()
            if not dst:
                if not nogoods and cap is None:
                    # no cut and no cap: the model has NO solution within r for
                    # its own reasons (held berths vs a banned net's menu, K8
                    # re-plan) and a wider radius will not mend that -- the
                    # reference ships, as the recorded loop's "NO SOLUTION" did
                    rep.append(f'  pages-first: walk: the model has no solution within r={r} (no cuts) -- the reference ships')
                    break
                # PROVEN exhausted within r (every plan there is barred, or over the cap): widen; the cuts stay
                rep.append(f'  pages-first: walk: nothing within r={r} ({len(nogoods)} barred'
                           + (f', cap {cap}' if cap is not None else '') + f') -> r={r + 1}')
                r += 1
                tries = 0
                continue
            mv = model.get('moved', [])
            if not mv:
                r_next = PAGES_WALK_RMAX if (model['status'] == 'OPTIMAL' and r < PAGES_WALK_RMAX) else r + 1
                rep.append(f'  pages-first: walk: the proposal at r={r} is the reference itself '
                           f'(obj {model["obj"]:.1f}, {model["status"]}) -> r={r_next}')
                r = r_next
                tries = 0
                continue
            swim, bp, cost = verify(st, board, names, dst, src)
            key = F.pf_key(dst, bp, cost, model['vias'])
            po = _pairs_off(names, bp)
            what = ', '.join(_mv_tag(n, k, dst, src) for n, k, _s in mv)
            if best[3].get('obj') is None:
                best[3]['obj'] = model['obj']        # the first proposal is the objective's reference
            dobj = model['obj'] - best[3]['obj']
            line = f'moved {len(mv)} [{what}] d obj {dobj:+.1f}; ' + fmt(model, swim, cost, key, po)
            if PAGES_WALK_PROBE:
                dc = (cost - best[3].get('count', cost)) if cost is not None else float('nan')
                probe.append((dobj, dc, len(swim) - len(best[4]), po))
                nogoods.append(list(mv))
                rep.append(f'  pages-first: walk probe {len(probe)} r={r}: {line}')
                continue
            if F.pf_better(key, best[0]):
                rep.append(f'  pages-first: walk step {steps + 1} r={r}: {line} vs {best[0]}: ACCEPTED')
                best = (key, dst, src, dict(model, count=cost), swim, bp)
                cap = model['swim']
                steps += 1
                nogoods = []
                tries = 0
                r = r0
            else:
                nogoods.append(list(mv))
                rep.append(f'  pages-first: walk r={r}: {line} vs {best[0]}: rejected')
                if model['status'] != 'OPTIMAL' and r > 1:
                    r -= 1                      # too big to prove within the budget
                    tries = 0
                else:
                    tries += 1
                    if tries >= PAGES_WALK_TRIES:
                        r += 1                  # the cuts stay: the reference has not moved
                        tries = 0
    finally:
        PAGES_DET = det0
    if probe:
        rho = _spearman([p[0] for p in probe], [p[1] for p in probe])
        rep.append(f'  pages-first: walk PROBE: {len(probe)} proposal(s) at r={r0}: Spearman(d obj, d count) = {rho:.2f}; '
                   f'd count {[round(p[1]) for p in probe]}; d resid {[p[2] for p in probe]}')
    rep.append(f'  pages-first: walk done: {steps} accepted step(s), {solves} solve(s) '
               f'({_WALK_BUDGET["solves"]} left in the run'
               + (f', {max(stage_left[0], 0)} in the stage' if PAGES_WALK_STAGE else '')
               + f'), final r={r}'
               + (f'; the braid swims {len(best[4])} {best[4] if best[4] else ""}, key {best[0]}' if best else ''))
    _walk.last_best = best
    if best is None:
        choose.last = {}
        return {}, {}, rep
    choose.last = dict(best[3], swim_braid=len(best[4]), count=(best[0][0] if F.PLAN_JUDGE else None))
    return best[1], best[2], rep


def choose(st, board, log=print, fixed=None, learned=None, src_free=True, seed=None):
    """The pages-first choice, re-keyed on its own answer until the braid's
    planner agrees (at most PAGES_ITERS solves). `src_free` False: the tooth
    as it stands is the only source candidate (the destination re-plan
    loop, which realizes no source move). `seed`: the plan the corridors
    are first built on (the greedy's choice)."""
    walk_best = None
    if PAGES_WALK and PAGES_WALK_FROM != 'damped':
        dw, sw, repw = _walk(st, board, log, fixed, learned, src_free, seed)
        if not PAGES_WALK_FALLBACK:
            return dw, sw, repw
        walk_best = getattr(_walk, 'last_best', None)      # the walk's plan; the damped loop runs too
    rep = list(repw) if walk_best is not None else []
    best = None
    seed_d = dict(seed or {})
    seed_s: Dict[str, Move] = {}
    hold_d, hold_s, avoid = None, None, None
    widen = False
    priced: Dict[str, Dict] = {}         # PLAN_PAGES_ISLAND: berth candidates priced by the parts their lane crosses
    for it in range(max(1, PAGES_ITERS)):
        fx = dict(fixed or {})
        if hold_d:
            fx.update(hold_d)
        stage_hint = None
        stage_menu = None
        if PAGES_MENU_STAGE and it == 0 and (PAGES_MENU or PAGES_MENU_TOP):
            # PLAN_PAGES_MENU_STAGE: the trimmed menu first; its plan hints the full solve
            d1, s1, lines1, m1 = _solve(st, board, log, fx, learned, src_free,
                                        seed_d if PAGES_KEYS == 'braid' else None, seed_s,
                                        hold_s, avoid,
                                        no_climb=bool(PAGES_CLIMB_LATE and it == 0),
                                        hard_fixed=fixed, priced=priced or None,
                                        end_climbs=(it > 0) if PAGES_END_WALK == 2 else None)
            rep += lines1
            if d1:
                stage_hint = (d1, s1)
                rep.append(f'  pages-first: menu stage: the trimmed solve ({m1["status"]} obj {m1["obj"]:.1f}, '
                           f'model swims {m1["swim"]}) hints the full-menu solve')
            stage_menu = (0, 0)
        dst_choice, src_choice, lines, model = _solve(st, board, log, fx, learned, src_free,
                                                       seed_d if PAGES_KEYS == 'braid' else None, seed_s,
                                                       hold_s, avoid,
                                                       no_climb=bool(PAGES_CLIMB_LATE and it == 0),
                                                       hard_fixed=fixed, priced=priced or None,
                                                       end_climbs=(it > 0) if PAGES_END_WALK == 2 else None,
                                                       menu=stage_menu, hint=stage_hint)
        rep += lines
        if PAGES_PORTFOLIO and it == 0 and dst_choice:
            # the same instance under the OTHER objective; the judge picks
            import fanout_from_plan as F
            d2, s2, lines2, m2 = _solve(st, board, log, fx, learned, src_free,
                                        seed_d if PAGES_KEYS == 'braid' else None, seed_s,
                                        hold_s, avoid, no_climb=bool(PAGES_CLIMB_LATE and it == 0),
                                        hard_fixed=fixed, rate=1 - PAGES_RATE,
                                        end_climbs=False if PAGES_END_WALK == 2 else None)
            rep += lines2
            if d2:
                sw1, bp1, c1 = verify(st, board, list(dst_choice), dst_choice, src_choice)
                sw2, bp2, c2 = verify(st, board, list(d2), d2, s2)
                k1 = F.pf_key(dst_choice, bp1, c1, model['vias'])
                k2 = F.pf_key(d2, bp2, c2, m2['vias'])
                pick2 = F.pf_better(k2, k1)
                u1 = 'rate' if PAGES_RATE else 'greedy'
                u2 = 'greedy' if PAGES_RATE else 'rate'
                rep.append(f'  pages-first: portfolio: {u1} units -> key {k1} (braid swims {len(sw1)}); '
                           f'{u2} units -> key {k2} (braid swims {len(sw2)}): the judge takes the '
                           f'{u2 if pick2 else u1} plan')
                if pick2:
                    dst_choice, src_choice, model = d2, s2, m2
        if PAGES_MENU_PORTFOLIO and it == 0 and dst_choice and (PAGES_MENU or PAGES_MENU_TOP) and not PAGES_MENU_STAGE:
            # the same instance on the FULL menu; the judge picks
            import fanout_from_plan as F
            d2, s2, lines2, m2 = _solve(st, board, log, fx, learned, src_free,
                                        seed_d if PAGES_KEYS == 'braid' else None, seed_s,
                                        hold_s, avoid, no_climb=bool(PAGES_CLIMB_LATE and it == 0),
                                        hard_fixed=fixed, priced=priced or None,
                                        end_climbs=False if PAGES_END_WALK == 2 else None,
                                        menu=(0, 0))
            rep += lines2
            if d2:
                sw1, bp1, c1 = verify(st, board, list(dst_choice), dst_choice, src_choice)
                sw2, bp2, c2 = verify(st, board, list(d2), d2, s2)
                k1 = F.pf_key(dst_choice, bp1, c1, model['vias'])
                k2 = F.pf_key(d2, bp2, c2, m2['vias'])
                pick2 = F.pf_better(k2, k1)
                rep.append(f'  pages-first: menu portfolio: trimmed menu -> key {k1} (braid swims {len(sw1)}, '
                           f'{model["status"]} obj {model["obj"]:.1f}); full menu -> key {k2} (braid swims {len(sw2)}, '
                           f'{m2["status"]} obj {m2["obj"]:.1f}): the judge takes the {"full" if pick2 else "trimmed"} plan')
                if pick2:
                    dst_choice, src_choice, model = d2, s2, m2
        if PAGES_SEEDS > 1 and it == 0 and dst_choice:
            # PLAN_PAGES_SEEDS: the same instance under other random seeds; the judge keeps the best
            import fanout_from_plan as F
            sw1, bp1, c1 = verify(st, board, list(dst_choice), dst_choice, src_choice)
            k1 = F.pf_key(dst_choice, bp1, c1, model['vias'])
            keys = [f'seed 0: key {k1} ({model["status"]} obj {model["obj"]:.1f}, braid swims {len(sw1)})']
            pick = 0
            for k in range(1, PAGES_SEEDS):
                d2, s2, lines2, m2 = _solve(st, board, log, fx, learned, src_free,
                                            seed_d if PAGES_KEYS == 'braid' else None, seed_s,
                                            hold_s, avoid, no_climb=bool(PAGES_CLIMB_LATE and it == 0),
                                            hard_fixed=fixed, priced=priced or None,
                                            end_climbs=False if PAGES_END_WALK == 2 else None,
                                            menu=stage_menu, hint=stage_hint, rseed=k)
                rep += lines2
                if not d2:
                    continue
                sw2, bp2, c2 = verify(st, board, list(d2), d2, s2)
                k2 = F.pf_key(d2, bp2, c2, m2['vias'])
                keys.append(f'seed {k}: key {k2} ({m2["status"]} obj {m2["obj"]:.1f}, braid swims {len(sw2)})')
                if F.pf_better(k2, k1):
                    dst_choice, src_choice, model, k1, pick = d2, s2, m2, k2, k
            rep.append(f'  pages-first: seeds: {"; ".join(keys)}: the judge takes seed {pick}')
        if not dst_choice:
            if best is None:
                break
            # the swimmers alone cannot move: widen to the lanes they cross
            if not PAGES_WIDEN:
                break
            widen = True
            dst_choice, src_choice, model = best[1], best[2], best[3]
            swim, bp = best[4], best[5]
        else:
            swim, bp, cost = verify(st, board, list(dst_choice), dst_choice, src_choice)
            pv = getattr(verify, 'last_pitch', None) if PAGES_PITCH else None
            rep.append(f'  pages-first: iteration {it}: the braid\'s planner swims {len(swim)} '
                       f'{swim if swim else ""} on this plan (model {model["swim"]})'
                       + (f'; pitch-infeasible {sorted(pv)}' if pv else '')
                       + (f'; the braid\'s count {cost:.0f}' if cost is not None else ''))
            import fanout_from_plan as F
            # the key: (residue, the model's vias) as recorded; under
            # PLAN_JUDGE (the braid's count, residue) -- fanout_from_plan.pf_key
            key = F.pf_key(dst_choice, bp, cost, model['vias'])
            if best is None or F.pf_better(key, best[0]):
                best = (key, dst_choice, src_choice, dict(model, swim_braid=len(swim), count=cost), swim, bp)
            elif it > 0:
                if not PAGES_WIDEN:
                    break               # no better, and no wider search wanted
                widen = True            # no better: give the next solve more room
        hits = getattr(verify, 'last_islands', {}) if (PAGES_ISLAND and dst_choice) else {}
        if hits:
            # PLAN_PAGES_ISLAND (2026-09-15): the plan just verified sends
            # these page lanes' chords through a corridor part on their own
            # layer (K35+: the DQ group round C5, refused in band every time
            # and re-laid at last call). Price each such berth by the parts
            # it crosses and solve again with EVERYTHING free -- the held
            # re-solve of the damped loop moved the violators onto berths
            # the held plan could not accommodate (19 swimmers, K35), while
            # the forced probe with everything free found the head-on plan
            # the model's own prices already prefer. The prices accumulate
            # across iterations; the judge (pf_key) keeps the best plan.
            new_pr = 0
            for n, isl in hits.items():
                sig = sr.move_sig(dst_choice[n])
                if sig not in priced.get(n, {}):
                    priced.setdefault(n, {})[sig] = PAGES_ISLAND * len(isl)
                    new_pr += 1
            rep.append(f'  pages-first: island-crossing lanes {sorted(hits)} '
                       f'({sum(len(v) for v in hits.values())} crossing(s)); '
                       f'{new_pr} berth(s) newly priced at {PAGES_ISLAND:g}/part; re-solving everything free')
            if new_pr:
                hold_d, hold_s, avoid = None, None, None
                seed_d, seed_s = best[1], best[2]
                continue
        if not best[4] or PAGES_KEYS != 'braid':
            break
        # DAMPED re-key: the braid's verdict is exact for the moves it just
        # planned, and the insertion rule is exact only with the OTHER lanes
        # held -- so the next solve frees the SWIMMERS, each barred from the
        # berth that swam, and holds every other net at the best plan (its
        # berth as a one-move menu, its tooth as chosen); when that does not
        # reduce the count, the lanes they cross are freed too. Free to move
        # everything, the loop oscillated (K28: 6 -> 5 -> 6 swimmers).
        base_d, base_s, base_swim, base_bp = best[1], best[2], best[4], best[5]
        free = set(base_swim)
        if widen:
            for w in base_swim:
                bw = base_bp.get(w, {})
                for o, bo in base_bp.items():
                    if o == w or bo.get('corridor') != bw.get('corridor'):
                        continue
                    if None in (bw.get('launch_idx'), bw.get('target_idx'), bo.get('launch_idx'), bo.get('target_idx')):
                        continue
                    if (bw['launch_idx'] - bo['launch_idx']) * (bw['target_idx'] - bo['target_idx']) < 0:
                        free.add(o)
        hold_d = {n: sr.move_sig(mv) for n, mv in base_d.items() if n not in free}
        hold_s = {n: base_s.get(n) for n in base_d if n not in free}   # None = the tooth as it stands
        avoid = None if widen else {w: {sr.move_sig(base_d[w])} for w in base_swim}
        rep.append(f'  pages-first: re-solving {len(free)} net(s) {sorted(free)} with {len(hold_d)} held'
                   + (' (widened to their crossers)' if widen else ' (the swimmers, each barred from its berth)'))
        seed_d, seed_s = base_d, base_s
    if walk_best is not None:
        import fanout_from_plan as F
        if best is None or F.pf_better(walk_best[0], best[0]):
            rep.append(f'  pages-first: walk fallback: the walk\'s plan (key {walk_best[0]}) over the loop\'s '
                       f'({best[0] if best else "none"})')
            best = walk_best
        else:
            rep.append(f'  pages-first: walk fallback: the loop\'s plan (key {best[0]}) over the walk\'s ({walk_best[0]})')
    if best is None:
        choose.last = {}
        return {}, {}, rep
    if PAGES_GROUP:
        best, lines_g = _group_climb(st, board, log, best)
        rep += lines_g
    if PAGES_WALK and PAGES_WALK_FROM == 'damped':
        # PLAN_PAGES_WALK_FROM=damped (2026-09-15, session 12): the walk's
        # reference is the plan the damped loop settled on, so a walk that
        # accepts nothing ships THAT and not the raw first solve (the
        # harness: reversed_k15 26 / 0 open -> 18 / 2 open under the walk
        # from the solve, its every stage 0 steps)
        d2, s2, rep2 = _walk(st, board, log, fixed, learned, src_free, seed, ref=best)
        return d2, s2, rep + rep2
    choose.last = best[3]
    return best[1], best[2], rep


def _solve(st, board, log, fixed, learned, src_free, seed, src_seed, hold_s=None, avoid=None,
           no_climb=False, hard_fixed=None, trust=None, nogoods=None, swim_cap=None, rate=None,
           priced=None, end_climbs=None, menu=None, hint=None, rseed=None):
    """The pages-first choice on a plan state. Returns (dst_choice,
    src_choice, report): dst_choice {net: Move} for every net with a
    destination menu, src_choice {net: Move} for the nets whose tooth
    should MOVE (the source menu's move; a net keeping its tooth is not
    in it), and the report lines."""
    from ortools.sat.python import cp_model
    t0 = time.time()
    rate = PAGES_RATE if rate is None else int(rate)     # the objective's units: the greedy's (0) or VIA_MM (1)
    # the end-of-face climbs: in every solve unless SRC_CLIMB_END_WALK
    # confines them to the walk's proposals (the caller says which this is)
    end_ok = (not PAGES_END_WALK) if end_climbs is None else bool(end_climbs)
    # the menu pre-filter's caps: the run's (PLAN_PAGES_MENU / _TOP) or the
    # caller's `menu=(k, K)` (the two-stage solve passes the full menu as (0, 0))
    menu_k, menu_top = (PAGES_MENU, PAGES_MENU_TOP) if menu is None else (int(menu[0] or 0), int(menu[1] or 0))
    fixed = dict(fixed or {})
    learned = learned or set()
    launch: Dict[str, Pt] = st['launch']
    dbox = st['dgrid'].bbox
    names = [n for n in launch if st['dmenu'].get(n)]
    fr = Frame({n: launch[n] for n in names}, dbox)
    sfr = SrcFrame(fr, st['sgrid'].bbox)
    # PLAN_LOOP_FEEDBACK holds (plan_loop.py): on the FREE first solve -- no
    # berth fixed and no tooth held by the caller -- every net the feedback
    # holds keeps the incumbent's berth (a one-move menu, else its class) and
    # its tooth as it stands; a freed net keeps its whole menu. The solver's
    # freedom is then exactly the hypothesis the loop is testing, not its own
    # objective's taste: hinted and priced but UNHELD (K51 loop round 1,
    # 2026-09-18) the solve moved 33 of 48 ends and routed 125 for 98.
    # ...in EVERY solve, not only the free first one (the confirm re-plan
    # passes `fixed`, the damped loop passes its own `hold_s` and FREES every
    # swimmer -- measured on the K51 loop's src arm, the second solve asked
    # for seven more teeth the loop had held). The held tooth takes
    # precedence over a caller's hold list in the source loop below.
    fb_hold = bool(pfb.HOLD) and trust is None
    fb_force_s: set = set()      # nets whose source hold names a move (index 1 forced)

    # ---- candidates
    D: Dict[str, List[Move]] = {}
    barred: Dict[str, set] = {}          # PAGES_UNBLOCK >= 2: the soft-barred candidates per net
    dref = ((dbox[0] + dbox[2]) / 2, (dbox[1] + dbox[3]) / 2)
    sbox = st['sgrid'].bbox

    def price_d(n, mv):
        # the destination objective's own price of a berth (the cost terms below)
        if rate:
            return VIA_W * (mv.vias + (sm._length(mv) + sm.around_box(launch[n], mv.exit_pt, dbox)) / sm.VIA_MM)
        return VIA_W * mv.vias + CHAN_W * sm._length(mv) + sm.around_box(launch[n], mv.exit_pt, dbox)

    def price_s(mv):
        # the source objective's own price of a tooth (the cost terms below)
        if rate:
            ex = mv.exit_pt
            straight = math.hypot(dref[0] - ex[0], dref[1] - ex[1])
            wrap = max(0.0, sm.around_box(ex, dref, sbox) - straight)
            return VIA_W * (mv.vias + ((sm._length(mv) if mv.legs else 0.0) + wrap) / sm.VIA_MM)
        return VIA_W * mv.vias + (CHAN_W * sm._length(mv) if mv.legs else 0.0)

    menu_cut = [0, 0, 0, 0]          # berths before / after, teeth before / after

    def prefilter(ms, price, keep_sigs, cls):
        # PLAN_PAGES_MENU / _TOP: the k cheapest per class, then the K
        # cheapest overall; `keep_sigs` survive regardless; the menu's own
        # order is kept (the model is built in it)
        if not (menu_k or menu_top) or not ms:
            return ms
        pr = {id(m): price(m) for m in ms}
        keep = {id(m) for m in ms if sr.move_sig(m) in keep_sigs}
        if menu_k:
            by = {}
            for m in ms:
                by.setdefault(cls(m), []).append(m)
            for lst in by.values():
                lst.sort(key=lambda m: pr[id(m)])
                keep |= {id(m) for m in lst[:menu_k]}
            ms = [m for m in ms if id(m) in keep]
        if menu_top and len(ms) > menu_top:
            ranked = sorted(ms, key=lambda m: pr[id(m)])
            keep |= {id(m) for m in ranked[:menu_top]}
            ms = [m for m in ms if id(m) in keep]
        return ms

    def d_keep(n):
        ks = set()
        if n in fixed:
            ks.add(fixed[n])
        if seed and n in seed and seed[n] is not None:
            ks.add(sr.move_sig(seed[n]))
        if trust and trust[0] and n in trust[0]:
            ks.add(trust[0][n])
        return ks
    for n in names:
        ms = list(st['dmenu'][n])
        if no_climb:
            ms = [m for m in ms if not getattr(m, 'climb', 0)]   # PAGES_CLIMB_LATE: the plain menu
        menu_cut[0] += len(ms)
        ms = prefilter(ms, lambda m, _n=n: price_d(_n, m), d_keep(n),
                       lambda m: (m.direction, m.layer, bool(getattr(m, 'climb', 0))))
        menu_cut[1] += len(ms)
        if fb_hold:
            ms = pfb.hold_dst(n, ms)
        if n in fixed:
            hit = [m for m in ms if sr.move_sig(m) == fixed[n]]
            if hit:
                ms = hit[:1]
        if avoid and n in avoid:
            if PAGES_UNBLOCK >= 2:
                barred[n] = {j for j, m in enumerate(ms) if sr.move_sig(m) in avoid[n]}
            else:
                ms2 = [m for m in ms if sr.move_sig(m) not in avoid[n]]
                ms = ms2 or ms
        D[n] = ms
    if PAGES_UNBLOCK and avoid:
        # the boxed-in swimmers (see the flag): un-hold what boxes them in
        hard = set(hard_fixed or {})
        one = {n for n in names if len(D[n]) == 1 and n in fixed}
        blk: Dict[str, Dict[int, set]] = {}
        for (a, i, b, j) in _conflicts(D, strict=bool(PAGES_STRICT)):
            if a in avoid and b in one:
                blk.setdefault(a, {}).setdefault(i, set()).add(b)
            if b in avoid and a in one:
                blk.setdefault(b, {}).setdefault(j, set()).add(a)
        unheld: set = set()
        boxed = []
        for n in sorted(avoid):
            bm = {i: v for i, v in blk.get(n, {}).items() if i not in barred.get(n, ())}
            if len(bm) < len(D[n]) - len(barred.get(n, ())):
                continue                    # some unbarred candidate is compatible with every hold
            if not bm:
                continue
            i_best = min(bm, key=lambda i: (len(bm[i] - hard), len(bm[i]), i))
            if bm[i_best] & hard:
                boxed.append(f'{n} (by laid copper)')
                continue                    # boxed in by berths already laid: nothing to un-hold
            unheld |= bm[i_best]
            boxed.append(f'{n} <- {sorted(bm[i_best])}')
        for b in sorted(unheld):
            ms = list(st['dmenu'][b])
            if no_climb:
                ms = [m for m in ms if not getattr(m, 'climb', 0)]
            D[b] = ms
        if boxed:
            log(f'  pages-first: unblock: {len(boxed)} boxed-in swimmer(s) {boxed}; {len(unheld)} held berth(s) freed')
    S: Dict[str, List[Move]] = {}
    cur: Dict[str, Optional[Move]] = {}
    for n in names:
        c = current_tooth(st, n)
        cur[n] = c
        opts = [c] if c is not None else []
        if fb_hold and pfb.held_src(n) and c is not None:
            # PLAN_LOOP_FEEDBACK: held -- the tooth as it stands, or at the
            # move the hold names (a crossover): [standing, the move], the
            # move forced below exactly as a caller's hold_s move is
            mv_ = pfb.held_src_move(n, st['smenu'].get(n, []))
            if mv_ is not None and not same_tooth(mv_, c):
                opts = [c, mv_]
                fb_force_s.add(n)
        elif hold_s is not None and n in hold_s:
            # held at the verified plan: its chosen tooth move, or as it stands
            if hold_s[n] is not None and not (PAGES_NOOP and c is not None and same_tooth(hold_s[n], c)):
                opts = [c, hold_s[n]] if c is not None else [hold_s[n]]
        elif PAGES_SRC and src_free and c is not None:
            more = [m for m in st['smenu'].get(n, [])
                    if not (no_climb and getattr(m, 'climb', 0))     # PAGES_CLIMB_LATE: the plain menu
                    and (end_ok or not getattr(m, 'end_climb', False))   # SRC_CLIMB_END_WALK: proposals only
                    and not (PAGES_NOOP and same_tooth(m, c))]       # PLAN_PAGES_NOOP: not a move
            menu_cut[2] += len(more)
            ks = {sr.move_sig(m) for m in more if getattr(m, 'end_climb', False)}
            if src_seed and n in src_seed and src_seed[n] is not None:
                ks.add(sr.move_sig(src_seed[n]))
            if trust and trust[1] and n in trust[1] and trust[1][n] is not None:
                ks.add(sr.move_sig(trust[1][n]))
            more = prefilter(more, price_s, ks,
                             lambda m: (m.direction, m.layer, bool(getattr(m, 'climb', 0))))
            menu_cut[3] += len(more)
            opts += more
        S[n] = opts
    fb_hint = None
    if fb_hold:
        # UNBLOCK a freed net the holds box in: one whose EVERY candidate a
        # held one-move berth excludes has no move at all (the damped loop's
        # own INFEASIBLE, 132 of 251 re-solves at K41) -- the fewest holders
        # of its least-held candidate are freed with it (PAGES_UNBLOCK's rule)
        held1 = {n for n in names if n in pfb.HOLD and len(D[n]) == 1}
        freed_d = [n for n in names
                   if not (pfb.HOLD.get(n) and ('dst' in pfb.HOLD[n] or 'dst_cls' in pfb.HOLD[n]))]
        cut_: Dict[str, Dict[int, set]] = {}
        for (a, i, b, j) in _conflicts(D, strict=bool(PAGES_STRICT)):
            if a in freed_d and b in held1:
                cut_.setdefault(a, {}).setdefault(i, set()).add(b)
            if b in freed_d and a in held1:
                cut_.setdefault(b, {}).setdefault(j, set()).add(a)
        unheld_: set = set()
        boxed_ = []
        for f_ in freed_d:
            bm = cut_.get(f_, {})
            if not D[f_] or len(bm) < len(D[f_]):
                continue
            i_best = min(bm, key=lambda i: (len(bm[i]), i))
            unheld_ |= bm[i_best]
            boxed_.append(f'{f_} <- {sorted(bm[i_best])}')
        for b in sorted(unheld_):
            ms = list(st['dmenu'][b])
            if no_climb:
                ms = [m for m in ms if not getattr(m, 'climb', 0)]
            D[b] = prefilter(ms, lambda m, _n=b: price_d(_n, m), d_keep(b),
                             lambda m: (m.direction, m.layer, bool(getattr(m, 'climb', 0))))
        log(f'  pages-first: feedback hold: {len(held1)} berth(s) held, '
            f'{sum(1 for n in names if pfb.held_src(n))} tooth/teeth standing; '
            f'free at the berth {freed_d}'
            + (f'; boxed in {boxed_} -> {len(unheld_)} holder(s) freed' if boxed_ else ''))
    if hint is None and pfb.HINT:
        # PLAN_LOOP_FEEDBACK: the incumbent ROUTED plan as the hint -- its
        # berth and tooth per net matched on these menus by signature; a
        # net the hint leaves without a tooth is hinted at the tooth as it
        # stands (i0 = 0 in the hint block below). Replaces the seed's hint.
        hd_, hs_ = {}, {}
        for n in names:
            mv_ = pfb.hint_move(n, 'dst', D[n])
            if mv_ is not None:
                hd_[n] = mv_
            if S[n]:
                mv_ = pfb.hint_move(n, 'src', S[n])
                if mv_ is not None:
                    hs_[n] = mv_
        if hd_:
            fb_hint = (hd_, hs_)
            log(f'  pages-first: feedback hint: {len(hd_)} of {len(names)} berth(s) and '
                f'{len(hs_)} moved tooth/teeth matched on the menus')
            if pfb.RADIUS and trust is None and not fixed and hold_s is None:
                # the ABLATION arm: no holds, at most RADIUS ends off the
                # hinted plan (the walk's trust region, `moved` below)
                ref_d = {n: sr.move_sig(mv) for n, mv in hd_.items()}
                ref_s = {}
                for n in names:
                    mv_ = hs_.get(n)
                    if mv_ is not None and cur.get(n) is not None and same_tooth(mv_, cur[n]):
                        mv_ = None          # the hinted tooth is the one standing here
                    ref_s[n] = mv_
                trust = (ref_d, ref_s, int(pfb.RADIUS))
                log(f'  pages-first: feedback trust region: at most {pfb.RADIUS} end(s) off the hinted plan')
    if menu_k or menu_top:
        log(f'  pages-first: menu pre-filter (class cap {menu_k or "-"}, top {menu_top or "-"}): '
            f'berths {menu_cut[0]} -> {menu_cut[1]}, tooth moves {menu_cut[2]} -> {menu_cut[3]}')
    tkey = {n: [int(round(fr.key(m) * 1000)) for m in D[n]] for n in names}
    lkey = {}
    for n in names:
        if S[n]:
            lkey[n] = [int(round(sfr.key(m) * 1000)) for m in S[n]]
        else:
            lkey[n] = [int(round(fr.across(launch[n]) * 1000))]
    corr = {n: 0 for n in names}
    jkey, jflag = {}, {}
    if seed is not None:
        # THE BRAID'S OWN ORDER as the keys, the other lanes held at the seed.
        # A net the greedy left UNPLACED is not in the seed, so the braid's
        # plan phase never sees it: no corridor, frame keys, corr -1 -- and
        # therefore OUTSIDE every planarity pair, free to swim in the model
        # at no cost (K41: 6 of 41 nets, and they were the braid's swimmers).
        # For KEYING ONLY, such a net is seeded at its cheapest berth (the
        # greedy's own ranking, conflicts ignored); the solve still chooses
        # among all of its candidates.
        seed = dict(seed)
        unplaced = [n for n in names if n not in seed and D[n]]
        for n in unplaced:
            seed[n] = min(D[n], key=lambda mv: (VIA_W * (mv.vias + (sm._length(mv) + sm.around_box(launch[n], mv.exit_pt, dbox)) / sm.VIA_MM)
                                                 if rate else
                                                 VIA_W * mv.vias + CHAN_W * sm._length(mv)
                                                 + sm.around_box(launch[n], mv.exit_pt, dbox)))
        if unplaced:
            log(f'  pages-first: {len(unplaced)} net(s) the seed left unplaced, keyed at their cheapest berth: {unplaced}')
        tk_b, lk_b, corr_b = braid_slots(st, board, names, seed, src_seed, D, S, log)
        n_keyed = 0
        for n in names:
            if n in tk_b and all(v is not None for v in tk_b[n]) and all(v is not None for v in lk_b.get(n, [])):
                tkey[n] = [int(round(v * 1000)) for v in tk_b[n]]
                if lk_b.get(n):
                    lkey[n] = [int(round(v * 1000)) for v in lk_b[n]]
                corr[n] = corr_b[n]
                n_keyed += 1
            else:
                corr[n] = -1      # not planned by the braid: Frame keys, its own corridor
        missing = [n for n in names if corr[n] == -1]
        if missing:
            log(f'  pages-first: {len(missing)} net(s) keyed by the frame, not the braid: {missing}')
        jkeys_b, joiner_b = getattr(braid_slots, 'extra', ({}, {}))
        if PAGES_JOINKEY:
            for n in names:
                if corr[n] != -1 and n in jkeys_b and all(v is not None for v in jkeys_b[n]):
                    jkey[n] = [int(round(v * 1000)) for v in jkeys_b[n]]
                    jflag[n] = list(joiner_b.get(n, []))

    if pfb.REACH and fb_hold:
        # PLAN_LOOP_FEEDBACK reach: a free end may land only between the keys
        # of its two bounding nets (its held neighbours k ranks away in the
        # incumbent's order) -- locality in ORDER space. The keys are the
        # braid's own slots, final here; the per-candidate lists are
        # filtered together. A window that would empty a menu is skipped
        # and said; index 0 of a source menu (the tooth as it stands) stays.
        cut_lines = []
        for n in names:
            for end in ('dst', 'src'):
                w = pfb.reach_window(n, end)
                if w is None:
                    continue
                cands = D[n] if end == 'dst' else S[n]
                if len(cands) <= 1:
                    continue
                keys = tkey[n] if end == 'dst' else lkey[n]
                if len(keys) != len(cands):
                    continue
                lo_n, hi_n = w
                bound = []
                for b_, side in ((lo_n, 'lo'), (hi_n, 'hi')):
                    if b_ is None or b_ not in names or corr.get(b_, -1) != corr.get(n, -1) or corr.get(n, -1) == -1:
                        bound.append(None)
                        continue
                    bk = tkey[b_] if end == 'dst' else lkey[b_]
                    bound.append(min(bk) if side == 'lo' else max(bk))
                if bound[0] is None and bound[1] is None:
                    continue
                keep = [i for i, k in enumerate(keys)
                        if (bound[0] is None or k >= bound[0]) and (bound[1] is None or k <= bound[1])
                        or (end == 'src' and i == 0)]
                if len(keep) == len(cands):
                    continue
                if not keep or (end == 'dst' and not keep) or (end == 'src' and keep == [0] and len(cands) > 1 and pfb.banned(n, 'src', cands[0])):
                    cut_lines.append(f'{n}.{end}: window would empty the menu -- not applied')
                    continue
                if end == 'dst':
                    D[n] = [cands[i] for i in keep]
                    tkey[n] = [keys[i] for i in keep]
                    if n in jkey:
                        jkey[n] = [jkey[n][i] for i in keep]
                else:
                    S[n] = [cands[i] for i in keep]
                    lkey[n] = [keys[i] for i in keep]
                    if n in jflag:
                        jflag[n] = [jflag[n][i] for i in keep]
                cut_lines.append(f'{n}.{end} {len(cands)}->{len(keep)}')
        if cut_lines:
            log(f'  pages-first: feedback reach: {"; ".join(cut_lines)}')
    # the destination box's centre and the source box: the reference for a
    # tooth candidate's wrap round its own array (the rate)
    dref = ((dbox[0] + dbox[2]) / 2, (dbox[1] + dbox[3]) / 2)
    sbox = st['sgrid'].bbox
    # ---- the model
    m = cp_model.CpModel()
    xd = {n: [m.NewBoolVar(f'd_{n}_{j}') for j in range(len(D[n]))] for n in names}
    xs = {n: [m.NewBoolVar(f's_{n}_{i}') for i in range(max(1, len(S[n])))] for n in names}
    pg = {n: m.NewBoolVar(f'p_{n}') for n in names}          # 1 = page B
    sw = {n: m.NewBoolVar(f'w_{n}') for n in names}          # left to swim
    # ---- the TRUST REGION (PLAN_PAGES_WALK, THE PLAN item 3): at most r
    # ends moved off the reference plan (a berth off its reference berth, a
    # tooth off its reference tooth -- as it stands, or the accepted move);
    # a net the reference leaves unplaced has no reference berth and is
    # free. `nogoods`: rejected proposals, each the exact candidates its
    # moved ends took, barred as a set (the next-best proposal differs in
    # at least one of them).
    moved: Dict[tuple, object] = {}
    if trust is not None:
        ref_d, ref_s, r_trust = trust
        holes = []
        for n in names:
            js_ = [j for j, mv in enumerate(D[n]) if sr.move_sig(mv) == ref_d.get(n)]
            if js_ and len(D[n]) > 1:
                moved[(n, 'd')] = xd[n][js_[0]].Not()
            elif n in ref_d and len(D[n]) > 1:
                holes.append(f'{n}:berth')          # a reference berth no longer on the menu: free of the radius
            if len(S[n]) > 1:
                ref = ref_s.get(n)
                i0 = 0
                if ref is not None:
                    i0 = next((i for i, mv in enumerate(S[n]) if i > 0 and sr.move_sig(mv) == sr.move_sig(ref)), -1)
                    if i0 < 0:
                        holes.append(f'{n}:tooth')  # the reference tooth is not on the menu: the standing tooth stands in
                        i0 = 0
                moved[(n, 's')] = xs[n][i0].Not()
        if r_trust is not None:
            m.Add(sum(moved.values()) <= int(r_trust))
        if holes:
            log(f'  pages-first: trust region: {len(holes)} end(s) without a reference candidate {holes}')
        for ng in (nogoods or []):
            # a rejected proposal is barred by its SET OF MOVED ENDS: not all
            # of these ends moved together again (a proposal moving a subset
            # of them, or another end, stays open). Barring the exact
            # candidates instead was too fine: the solver re-proposed the
            # same three ends at a neighbouring gap, three times (K8 smoke).
            lits = [moved[(n, kind)] for (n, kind, _sig) in ng if (n, kind) in moved]
            if lits and len(lits) == len(ng):
                m.AddBoolOr([v.Not() for v in lits])
    T = {n: m.NewIntVar(min(tkey[n] + jkey.get(n, [])), max(tkey[n] + jkey.get(n, [])), f'T_{n}') for n in names}
    L = {n: m.NewIntVar(min(lkey[n]), max(lkey[n]), f'L_{n}') for n in names}
    cost_terms = []
    n_fb = [0, 0, 0]     # PLAN_LOOP_FEEDBACK: candidates priced (berths, teeth), standing teeth banned
    for n in names:
        m.AddExactlyOne(xd[n])
        m.AddExactlyOne(xs[n])
        if pfb.BANS and len(S[n]) > 1 and cur.get(n) is not None and S[n][0] is cur[n] \
                and pfb.banned(n, 'src', cur[n]):
            # PLAN_LOOP_FEEDBACK: the tooth AS IT STANDS is of a banned class.
            # The menu filter cannot remove it (it is not a menu move), so
            # the solve is told it must move -- index 0 stays the standing
            # tooth, which the read-out below relies on.
            m.Add(xs[n][0] == 0)
            n_fb[2] += 1
        if hold_s is not None and hold_s.get(n) is not None and len(S[n]) == 2:
            m.Add(xs[n][1] == 1)
        if n in fb_force_s and len(S[n]) == 2:
            m.Add(xs[n][1] == 1)     # PLAN_LOOP_FEEDBACK: the named tooth
        if n in jkey and n in jflag and len(jflag[n]) == len(S[n]) and any(jflag[n]) \
                and any(a != b for a, b in zip(tkey[n], jkey[n])):
            # T = the port key, or the joined key when the chosen tooth is a
            # joiner: w_j = xd_j AND jn, jn = the joiner flag of the chosen tooth
            jn = sum(v for i, v in enumerate(xs[n]) if jflag[n][i])
            terms = []
            for j, v in enumerate(xd[n]):
                dlt = jkey[n][j] - tkey[n][j]
                terms.append(tkey[n][j] * v)
                if dlt:
                    w = m.NewBoolVar(f'w_{n}_{j}')
                    m.Add(w <= v); m.Add(w <= jn); m.Add(w >= v + jn - 1)
                    terms.append(dlt * w)
            m.Add(T[n] == sum(terms))
        else:
            jd = m.NewIntVar(0, len(D[n]) - 1, f'jd_{n}')
            m.Add(jd == sum(j * v for j, v in enumerate(xd[n])))
            m.AddElement(jd, tkey[n], T[n])
        if len(lkey[n]) > 1:
            js = m.NewIntVar(0, len(S[n]) - 1, f'js_{n}')
            m.Add(js == sum(i * v for i, v in enumerate(xs[n])))
            m.AddElement(js, lkey[n], L[n])
        else:
            m.Add(L[n] == lkey[n][0])
        # layers at the two ends, as linear expressions of the choice
        dB = sum(v for j, v in enumerate(xd[n]) if D[n][j].layer == 'B.Cu')
        if S[n]:
            tB = sum(v for i, v in enumerate(xs[n]) if S[n][i].layer == 'B.Cu')
        else:
            tB = 1 if st['tooth0'].get(n, 'F.Cu') == 'B.Cu' else 0
        # mismatch of each end with the page (xor), as bools
        mt = m.NewBoolVar(f'mt_{n}')
        md = m.NewBoolVar(f'md_{n}')
        for (b, e) in ((mt, tB), (md, dB)):
            m.Add(b >= e - pg[n]); m.Add(b >= pg[n] - e)
            m.Add(b <= e + pg[n]); m.Add(b <= 2 - e - pg[n])
        # the greedy's units, scaled: vias 3, channel 2, reach 1
        for j, mv in enumerate(D[n]):
            if rate:
                c = VIA_W * (mv.vias + (sm._length(mv) + sm.around_box(launch[n], mv.exit_pt, dbox)) / sm.VIA_MM)
            else:
                c = VIA_W * mv.vias + CHAN_W * sm._length(mv) + sm.around_box(launch[n], mv.exit_pt, dbox)
            if PAGES_KIND_VIP and mv.kind == 'via_in_pad':
                c += VIA_W * PAGES_KIND_VIP
            if j in barred.get(n, ()):
                c += PAGES_SWIM * VIA_W          # the soft bar: the berth that swam, at a swimmer's price
            pr_ = (priced or {}).get(n, {}).get(sr.move_sig(mv))
            if pr_:
                c += VIA_W * pr_                 # PLAN_PAGES_ISLAND: a berth whose lane crosses a corridor part
            if pfb.PRICES:
                fb_ = pfb.price(n, 'dst', mv)
                if fb_:
                    c += VIA_W * fb_             # PLAN_LOOP_FEEDBACK: the route's residual for this class
                    n_fb[0] += 1
            cost_terms.append(int(round(c * SCALE)) * xd[n][j])
        if S[n]:
            for i, mv in enumerate(S[n]):
                if rate:
                    # the tooth's own run, and its way ROUND THE SOURCE ARRAY to
                    # the destination's side (ride_mm's src_box term, per
                    # candidate): a far-face tooth pays the wrap it forces on
                    # the corridor (K35: SDQ13, U1's east-most column, sent 13 mm
                    # west on B and ridden 13 mm back -- priced 3 + 2 x 13 = 29
                    # against a 300 swimmer in the greedy's units, invisible to
                    # every judge without a ride term)
                    ex = mv.exit_pt
                    straight = math.hypot(dref[0] - ex[0], dref[1] - ex[1])
                    wrap = max(0.0, sm.around_box(ex, dref, sbox) - straight)
                    c = VIA_W * (mv.vias + ((sm._length(mv) if mv.legs else 0.0) + wrap) / sm.VIA_MM)
                else:
                    c = VIA_W * mv.vias + (CHAN_W * sm._length(mv) if mv.legs else 0.0)
                if PAGES_KIND_VIP and mv.kind == 'via_in_pad':
                    c += VIA_W * PAGES_KIND_VIP
                if pfb.PRICES:
                    fb_ = pfb.price(n, 'src', mv)
                    if fb_:
                        c += VIA_W * fb_         # PLAN_LOOP_FEEDBACK: the route's residual for this class
                        n_fb[1] += 1
                cost_terms.append(int(round(c * SCALE)) * xs[n][i])
        else:
            cost_terms.append(int(round(VIA_W * st['tooth_vias'].get(n, 0) * SCALE)))
        cost_terms.append(int(round(VIA_W * PAGES_MISMATCH * SCALE)) * mt)
        cost_terms.append(int(round(VIA_W * PAGES_MISMATCH * SCALE)) * md)
        cost_terms.append(int(round(PAGES_SWIM * VIA_W * SCALE)) * sw[n])
    # ---- the side strips' capacity (PLAN_PAGES_STRIP)
    strip_load = {}
    if PAGES_STRIP:
        strips, near_f, far_f = face_strips(st, log=log)
        members = {}
        for n in names:
            for j, mv in enumerate(D[n]):
                sd = strip_of(mv, near_f, far_f, dbox)
                if sd in strips:
                    members.setdefault(sd, []).append((n, j))
        for sd, mem in sorted(members.items()):
            for p in (0, 1):
                cap = strips[sd]['B.Cu' if p else 'F.Cu'][0]
                lits = []
                for (n, j) in mem:
                    w = m.NewBoolVar(f'st_{sd}_{p}_{n}_{j}')
                    pv = pg[n] if p else pg[n].Not()
                    m.AddBoolAnd([xd[n][j], pv]).OnlyEnforceIf(w)
                    m.AddBoolOr([xd[n][j].Not(), pv.Not()]).OnlyEnforceIf(w.Not())
                    lits.append(w)
                over = m.NewIntVar(0, len(mem), f'over_{sd}_{p}')
                m.Add(over >= sum(lits) - cap)
                cost_terms.append(int(round(PAGES_STRIP * VIA_W * SCALE)) * over)
                strip_load[(sd, p)] = (lits, cap)
    # ---- the pages: no inversion inside a page
    npairs = 0
    for a_i in range(len(names)):
        a = names[a_i]
        for b in names[a_i + 1:]:
            if corr[a] != corr[b] or corr[a] == -1:
                continue                # different corridors never cross
            ltL = m.NewBoolVar(f'lL_{a}_{b}')
            m.Add(L[a] < L[b]).OnlyEnforceIf(ltL)
            m.Add(L[a] >= L[b]).OnlyEnforceIf(ltL.Not())
            ltT = m.NewBoolVar(f'lT_{a}_{b}')
            m.Add(T[a] < T[b]).OnlyEnforceIf(ltT)
            m.Add(T[a] >= T[b]).OnlyEnforceIf(ltT.Not())
            same = m.NewBoolVar(f'sm_{a}_{b}')
            m.Add(pg[a] == pg[b]).OnlyEnforceIf(same)
            m.Add(pg[a] != pg[b]).OnlyEnforceIf(same.Not())
            m.Add(ltL == ltT).OnlyEnforceIf([same, sw[a].Not(), sw[b].Not()])
            npairs += 1
            if PAGES_SWIM_XING:
                # this pair CROSSES iff its launch and target orders
                # disagree; it is a PAGE crossing for a swimmer iff exactly
                # one of the two swims. Both are xors, encoded the way the
                # end-mismatch pair above is.
                inv = m.NewBoolVar(f'inv_{a}_{b}')
                m.Add(inv >= ltL - ltT)
                m.Add(inv >= ltT - ltL)
                m.Add(inv <= ltL + ltT)
                m.Add(inv <= 2 - ltL - ltT)
                one = m.NewBoolVar(f'one_{a}_{b}')
                m.Add(one >= sw[a] - sw[b])
                m.Add(one >= sw[b] - sw[a])
                m.Add(one <= sw[a] + sw[b])
                m.Add(one <= 2 - sw[a] - sw[b])
                xg = m.NewBoolVar(f'xg_{a}_{b}')
                # the cost is positive and minimised, so it only has to be
                # forced UP: it cannot sit at 0 when both halves hold, and
                # nothing has to stop it sitting at 0 otherwise
                m.Add(xg >= inv + one - 1)
                cost_terms.append(
                    int(round(PAGES_SWIM_XING * VIA_W * SCALE)) * xg)
    # ---- moves that cannot both be laid
    nconf = 0
    excl_d = []
    Sreal = {n: [mv for mv in S[n] if mv.legs] for n in names}
    idx_real = {n: [i for i, mv in enumerate(S[n]) if mv.legs] for n in names}
    ncell = 0
    if PAGES_CELLS:
        # the resource form: one at-most-one per occupied cell
        for grp in _cell_groups(D, PAGES_CELLS):
            m.AddAtMostOne([xd[a][i] for a, i in grp]); nconf += 1; ncell += len(grp)
        for grp in _cell_groups(Sreal, PAGES_CELLS):
            m.AddAtMostOne([xs[a][idx_real[a][i]] for a, i in grp]); nconf += 1; ncell += len(grp)
    else:
        excl_d = _conflicts(D, strict=bool(PAGES_STRICT))
        for (a, i, b, j) in excl_d:
            m.AddBoolOr([xd[a][i].Not(), xd[b][j].Not()]); nconf += 1
        for (a, i, b, j) in _conflicts(Sreal, strict=True):
            m.AddBoolOr([xs[a][idx_real[a][i]].Not(), xs[b][idx_real[b][j]].Not()]); nconf += 1
    if learned:
        sig_d = {n: [sr.move_sig(mv) for mv in D[n]] for n in names}
        # CANONICAL, and `key=repr` was NOT (review, 2026-09-16). `_conflicts`
        # sorts a set of TUPLES, whose repr is deterministic; `learned` is a
        # set of FROZENSETS, and a frozenset renders its elements in internal
        # hash order -- which for strings depends on PYTHONHASHSEED, pinned
        # NOWHERE in this chain. Measured: one two-element `learned` sorted
        # into two different orders across seeds 1..6. The exclusions below
        # are added to the model in this order, so two identical re-plan
        # passes could stop at different feasible points and lay different
        # copper from identical inputs -- the defect `_conflicts` documents
        # (obj 1784 / 1727 / 1809 on one model).
        for pair in sorted(learned, key=lambda p: sorted(p)):
            pair = list(pair)
            if len(pair) != 2:
                continue
            for a in names:
                for b in names:
                    if a >= b:
                        continue
                    for i, sa in enumerate(sig_d[a]):
                        for j, sb in enumerate(sig_d[b]):
                            if {sa, sb} == set(pair):
                                m.AddBoolOr([xd[a][i].Not(), xd[b][j].Not()]); nconf += 1
    if hint is None and fb_hint is not None:
        hint = fb_hint          # PLAN_LOOP_FEEDBACK: the incumbent plan, built above
    if hint is not None and hint[0]:
        # PLAN_PAGES_MENU_STAGE: the first stage's plan as the hint -- its
        # berth AND its tooth per net (a net it did not move: the tooth as
        # it stands)
        hd, hs = hint
        for n in names:
            sig = sr.move_sig(hd[n]) if n in hd and hd[n] is not None else None
            js_ = [j for j, mv in enumerate(D[n]) if sig is not None and sr.move_sig(mv) == sig]
            if js_:
                for j, v in enumerate(xd[n]):
                    m.AddHint(v, 1 if j == js_[0] else 0)
            if S[n] and cur.get(n) is not None:
                ssig = sr.move_sig(hs[n]) if hs and n in hs and hs[n] is not None else None
                is_ = [i for i, mv in enumerate(S[n]) if ssig is not None and sr.move_sig(mv) == ssig]
                i0 = is_[0] if is_ else 0
                for i, v in enumerate(xs[n]):
                    m.AddHint(v, 1 if i == i0 else 0)
    elif PAGES_HINT and seed:
        # the seed as a hint: its berth per net (the greedy's, or the cheapest
        # for a net it left unplaced) and the tooth as it stands; the pages
        # and the rest are the solver's to complete
        sig_seed = {n: sr.move_sig(seed[n]) for n in names if n in seed}
        for n in names:
            js_ = [j for j, mv in enumerate(D[n]) if sr.move_sig(mv) == sig_seed.get(n)]
            if js_:
                for j, v in enumerate(xd[n]):
                    m.AddHint(v, 1 if j == js_[0] else 0)
            if S[n] and cur.get(n) is not None:
                for i, v in enumerate(xs[n]):
                    m.AddHint(v, 1 if i == 0 else 0)
    cert_lines = []
    cap = None
    if swim_cap is not None:
        # the walk's MONOTONE cap: no more model swimmers than the reference
        # plan has (the review: a cap at the certified floor steers into the
        # plans that routed worst -- 2-swimmer K41 plans 85-98, the 4-swimmer
        # one 79)
        cap = int(swim_cap)
        m.Add(sum(sw.values()) <= cap)
    elif PAGES_CERT and trust is None:
        # THE PLAN item 2 (2026-09-15): a CERTIFICATE of the fewest swimmers
        # the menus admit. The swimmer terms own the objective's bound gap
        # (K41 50%, K51 61%: the solver cannot search big-M terms), while the
        # count ALONE proves in seconds (K41 2 at DET 30, K51 4 at DET 209);
        # with the count pinned the rest plateaus at once (K41's optimum at
        # DET 22 where the uncapped solve needed 406). Phase A: the same
        # model, the objective replaced by the swimmer count, under its own
        # deterministic budget; the main solve is then capped at what phase
        # A found -- PROVEN when phase A proved it, else the best it found,
        # said so. PLAN_PAGES_CERT=2 also hands phase A's whole plan to the
        # main solve as its hint (a complete solution under the cap).
        mA = m.clone()
        mA.Minimize(sum(sw.values()))
        sA = cp_model.CpSolver()
        sA.parameters.num_workers = PAGES_WORKERS
        sA.parameters.interleave_search = True
        sA.parameters.max_deterministic_time = PAGES_CERT_DET
        tA = time.time()
        stA = _smemo.solve(mA, sA, log=cert_lines.append, label='pages-first cert')
        if stA in (cp_model.OPTIMAL, cp_model.FEASIBLE):
            cap = int(round(sA.ObjectiveValue()))
            if stA == cp_model.OPTIMAL:
                # a cap only on a PROVEN certificate: a FEASIBLE stop is
                # budget-dependent (DET stops differ machine to machine)
                m.Add(sum(sw.values()) <= cap)
            if PAGES_CERT >= 2:
                m.clear_hints()
                for n in names:
                    for v in xd[n] + xs[n] + [pg[n], sw[n]]:
                        m.add_hint(v, sA.Value(v))
            cert_lines.append(
                f'  pages-first: certificate: min swimmers {cap} '
                f'{"PROVEN" if stA == cp_model.OPTIMAL else "UNCERTIFIED (best found, bound " + str(int(sA.BestObjectiveBound())) + ")"} '
                f'in {time.time() - tA:.1f} s (det {PAGES_CERT_DET:g}); '
                + (f'the main solve capped at {cap}' if stA == cp_model.OPTIMAL else 'reported only, NO cap')
                + (', hinted with phase A\'s plan' if PAGES_CERT >= 2 else ''))
        else:
            cert_lines.append(f'  pages-first: certificate: phase A found NO SOLUTION ({sA.StatusName(stA)}) '
                              f'in {time.time() - tA:.1f} s -- the main solve runs uncapped')
    if trust is not None and trust[2] is None:
        # the PROXIMITY solve: the model-feasible plan NEAREST the reference
        # (fewest ends moved; the cost only breaks ties) -- the walk's
        # reference when the greedy seed is not a model solution (K41: the
        # seed violates 26 of the model's strict exclusions, 83 under the
        # rate, and no plan within r=3 of it is feasible at all)
        m.Minimize(sum(moved.values()) * (SCALE * 10000) + sum(cost_terms))
    else:
        m.Minimize(sum(cost_terms))
    if PAGES_DUMP:
        import json as _json
        os.makedirs(PAGES_DUMP, exist_ok=True)
        _solve.n_dump = getattr(_solve, 'n_dump', 0) + 1
        _stem = os.path.join(PAGES_DUMP, f'{os.path.splitext(os.path.basename(board))[0]}_solve{_solve.n_dump}')
        m.export_to_file(_stem + '.pb')
        with open(_stem + '.json', 'w', encoding='utf-8') as _f:
            _json.dump({'board': board, 'names': names, 'det': PAGES_DET, 'workers': PAGES_WORKERS,
                        'hint': bool(PAGES_HINT and seed), 'held': sorted(hold_s) if hold_s else [],
                        'n_berth': sum(len(v) for v in D.values()), 'n_tooth': sum(len(v) for v in S.values()),
                        'pairs': npairs, 'exclusions': nconf,
                        'via_w': VIA_W, 'chan_w': CHAN_W, 'swim': PAGES_SWIM, 'mismatch': PAGES_MISMATCH, 'scale': SCALE,
                        # per candidate (vias, channel length mm, reach mm): the offline
                        # reader splits any solution's objective into its terms
                        'berth_terms': {n: [(mv.vias, round(sm._length(mv), 4),
                                             round(sm.around_box(launch[n], mv.exit_pt, dbox), 4)) for mv in D[n]]
                                        for n in names},
                        'tooth_terms': {n: [(mv.vias, round(sm._length(mv), 4) if mv.legs else 0.0, 0.0) for mv in S[n]]
                                        for n in names},
                        'tooth0_vias': {n: st['tooth_vias'].get(n, 0) for n in names if not S[n]}}, _f, indent=1)
        log(f'  pages-first: instance written to {_stem}.pb')
    solver = cp_model.CpSolver()
    if PAGES_CANON:
        solver.parameters.num_workers = PAGES_CANON_WORKERS
        solver.parameters.interleave_search = PAGES_CANON_WORKERS > 1
        solver.parameters.max_number_of_conflicts = PAGES_CANON
        solver.parameters.max_deterministic_time = PAGES_CANON_DET   # backstop only
        solver.parameters.linearization_level = PAGES_CANON_LP
        # the OTHER float channel: feasibility jump linearizes on its own
        solver.parameters.feasibility_jump_linearization_level = PAGES_CANON_LP
    else:
        solver.parameters.num_workers = PAGES_WORKERS
        solver.parameters.interleave_search = True
        solver.parameters.max_deterministic_time = PAGES_DET
    if rseed is None and pfb.SEED:
        rseed = pfb.SEED                                # PLAN_LOOP_FEEDBACK seed: a jump
    if rseed is not None:
        solver.parameters.random_seed = int(rseed)      # PLAN_PAGES_SEEDS: another feasible point
    rep = list(cert_lines)
    status = _smemo.solve(m, solver, log=rep.append, label='pages-first')
    if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        rep.append(f'  pages-first: NO SOLUTION ({solver.StatusName(status)}) -- the greedy choice stands')
        # ...AND SAY WHAT STOPPED IT (review, 2026-09-16). The CANON report
        # lives after this early return, so UNKNOWN -- the one status that
        # MEANS "a limit fired before a solution was found" -- printed no
        # portability warning at all: the warning written to catch a
        # non-portable stop was unreachable on exactly the stop it was
        # written for. An UNKNOWN here ships the GREEDY plan, so two
        # machines diverge with nothing in the log saying so.
        if PAGES_CANON:
            _dt = solver.ResponseProto().deterministic_time
            rep.append(f'  pages-first: CANON stopped with NO SOLUTION -- '
                       f'{solver.NumConflicts()} conflicts, det {_dt:.1f}/{PAGES_CANON_DET:g}'
                       + (' -- THE DETERMINISTIC BACKSTOP FIRED, NOT PORTABLE'
                          if _dt >= 0.99 * PAGES_CANON_DET else
                          ' -- conflict-bounded, but the GREEDY plan ships'))
        if status == cp_model.INFEASIBLE and excl_d:
            # WHY (2026-09-15, session 9; log only, changes nothing): the
            # damped re-solve holds every non-swimmer at a one-move menu and
            # bars each swimmer from the berth that swam -- and a swimmer
            # whose every remaining candidate is excluded by some held berth
            # has no move at all, so the whole solve dies (K41: 132 of 251
            # re-solves across the session's logs). Name them.
            one = {n for n in names if len(D[n]) == 1}
            cut: Dict[str, set] = {}
            for (a, i, b, j) in excl_d:
                if b in one and a not in one:
                    cut.setdefault(a, set()).add(i)
                if a in one and b not in one:
                    cut.setdefault(b, set()).add(j)
            dead = [f'{n} {len(cut[n])}/{len(D[n])}' for n in sorted(cut) if len(cut[n]) >= len(D[n])]
            rep.append(f'  pages-first: infeasible re-solve: {len(one)} one-move menus, {len(dead)} freed net(s) with '
                       f'EVERY candidate excluded by a held berth: {dead}'
                       + ('' if dead else ' (the cause is elsewhere: a source exclusion, a learned pair or a key bound)'))
            if trust is not None and int(trust[2]) == 0:
                # the reference itself: name the reference pairs the model's own
                # exclusions forbid (the greedy seed tests conflicts by its own
                # rule; the model's strict cells can forbid a pair it allowed)
                ref_j = {n: next((j for j, mv in enumerate(D[n]) if sr.move_sig(mv) == trust[0].get(n)), None)
                         for n in names}
                bad = [f'{a}-{b}' for (a, i, b, j) in excl_d if ref_j.get(a) == i and ref_j.get(b) == j]
                rep.append(f'  pages-first: the reference plan violates {len(bad)} of the model\'s own '
                           f'exclusions: {bad[:12]}')
        return {}, {}, rep, {}
    dst_choice: Dict[str, Move] = {}
    src_choice: Dict[str, Move] = {}
    swim = []
    pages = {}
    vias = 0
    ff = 0
    for n in names:
        j = next(j for j, v in enumerate(xd[n]) if solver.Value(v))
        dst_choice[n] = D[n][j]
        pages[n] = 'B.Cu' if solver.Value(pg[n]) else 'F.Cu'
        if S[n]:
            i = next(i for i, v in enumerate(xs[n]) if solver.Value(v))
            if i > 0:
                src_choice[n] = S[n][i]
            tl, tv = S[n][i].layer, S[n][i].vias
        else:
            tl, tv = st['tooth0'].get(n, 'F.Cu'), st['tooth_vias'].get(n, 0)
        v = tv + D[n][j].vias + (tl != pages[n]) + (D[n][j].layer != pages[n])
        vias += v
        ff += (v == 0)
        if solver.Value(sw[n]):
            swim.append(n)
    # THE PLANNER'S VALUE OF EACH SOURCE MOVE it chose (PLAN_BATCH's bisect
    # order, fanout_from_plan): the model cost the move saves for its own
    # net (tooth vias, the tooth/page mismatch) plus the inversions the
    # STANDING key would have with the net's same-page, non-swimming
    # partners under the chosen keys, each priced as a swimmer. Read off
    # the solution; changes nothing in it.
    value: Dict[str, float] = {}
    if src_choice:
        Lv = {n: solver.Value(L[n]) for n in names}
        Tv = {n: solver.Value(T[n]) for n in names}
        pgv = {n: solver.Value(pg[n]) for n in names}
        swv = {n: solver.Value(sw[n]) for n in names}
        for n in src_choice:
            i = next(k for k, mv in enumerate(S[n]) if mv is src_choice[n])
            c0 = VIA_W * S[n][0].vias + VIA_W * ((S[n][0].layer == 'B.Cu') != bool(pgv[n]))
            ci = (VIA_W * S[n][i].vias + CHAN_W * sm._length(S[n][i])
                  + VIA_W * ((S[n][i].layer == 'B.Cu') != bool(pgv[n])))
            inv = 0
            if corr[n] != -1 and not swv[n]:
                l0 = lkey[n][0]
                for b in names:
                    if b == n or corr[b] != corr[n] or pgv[b] != pgv[n] or swv[b]:
                        continue
                    if (l0 < Lv[b]) != (Tv[n] < Tv[b]):
                        inv += 1
            value[n] = round((c0 - ci) + PAGES_SWIM * VIA_W * inv, 3)
    import ortools as _ortools
    rep.append(f'  pages-first: {len(names)} nets, {sum(len(v) for v in D.values())} berth + '
               f'{sum(len(v) for v in S.values())} tooth candidates, {npairs} pairs, {nconf} exclusions'
               + (f' (cells: at-most-one over {ncell} memberships)' if PAGES_CELLS else '') + '; '
               f'{solver.StatusName(status)} obj {solver.ObjectiveValue() / SCALE:.1f} '
               f'bound {solver.BestObjectiveBound() / SCALE:.1f} in {time.time() - t0:.1f} s'
               # the conflict figure is NOT a budget fraction:
               # max_number_of_conflicts applies PER SUBSOLVER, so the total
               # overshoots it (measured 1034 against a budget of 200 at one
               # worker). It is reported for comparison BETWEEN runs, which
               # is what portability needs, not as a budget check.
               + (f' (CANON: {solver.NumConflicts()} conflicts [budget {PAGES_CANON}/subsolver], '
                  f'{solver.NumBranches()} branches, '
                  f'det {solver.ResponseProto().deterministic_time:.1f}/{PAGES_CANON_DET:g}'
                  # WHICH LIMIT FIRED, tested on the BACKSTOP ITSELF. The first
                  # spelling of this asked whether the conflict count was under
                  # budget -- which is not the question, and is wrong the moment
                  # there is more than one worker: NumConflicts() is the SUM over
                  # workers, so it reads 141824 against a 20000 budget while the
                  # run was in fact stopped by the deterministic backstop at
                  # 604.8/600. The warning stayed silent on exactly the run it
                  # exists to catch. A run stopped by the backstop is bounded by
                  # a WORK ACCUMULATOR and is therefore NOT portable.
                  + (' -- STOPPED BY THE DETERMINISTIC BACKSTOP, NOT PORTABLE'
                     if solver.ResponseProto().deterministic_time >= 0.99 * PAGES_CANON_DET
                        and solver.StatusName(status) not in ('OPTIMAL', 'INFEASIBLE')
                     else '')
                  + f', {PAGES_CANON_WORKERS} worker(s), lin {PAGES_CANON_LP}, ortools {_ortools.__version__})'
                  if PAGES_CANON else
                  f' (det {PAGES_DET:g}, {PAGES_WORKERS} workers, ortools {_ortools.__version__})'))
    if pfb.PRICES or n_fb[2]:
        rep.append(f'  pages-first: feedback prices on {n_fb[0]} berth + {n_fb[1]} tooth candidate(s)'
                   + (f'; {n_fb[2]} standing tooth/teeth banned (must move)' if n_fb[2] else ''))
    if strip_load:
        rep.append('  pages-first: strip loads ' + ', '.join(
            f'{sd} {"B" if p else "F"} {sum(solver.Value(v) for v in lits)}/{cap}'
            for (sd, p), (lits, cap) in sorted(strip_load.items())))
    rep.append(f'  pages-first: model vias {vias} ({ff} nets at 0), pages F '
               f'{sum(1 for n in names if pages[n] == "F.Cu")} / B {sum(1 for n in names if pages[n] == "B.Cu")}, '
               f'swimmers {len(swim)} {swim if swim else ""}, teeth to move {len(src_choice)} {sorted(src_choice) if src_choice else ""}')
    if PAGES_LOG:
        for n in sorted(names, key=lambda n: solver.Value(L[n])):
            rep.append(f'     {n:6s} page {pages[n][0]}  L {solver.Value(L[n]) / 1000:7.2f}  T {solver.Value(T[n]) / 1000:7.2f}'
                       f'  tooth {sr.fmt_ask(src_choice[n]) if n in src_choice else "(as is)"}  berth {sr.fmt_ask(dst_choice[n])}')
    _solve.last_keys = {'L': {n: solver.Value(L[n]) for n in names}, 'T': {n: solver.Value(T[n]) for n in names},
                        'corr': dict(corr)}
    moved_out = []
    for (n, kind), lit in moved.items():
        if solver.Value(lit):
            moved_out.append((n, kind, sr.move_sig(dst_choice[n]) if kind == 'd'
                              else (sr.move_sig(src_choice[n]) if n in src_choice else None)))
    return dst_choice, src_choice, rep, {'vias': vias, 'swim': len(swim), 'ff': ff, 'pages': pages, 'value': value,
                                         'moved': moved_out, 'status': solver.StatusName(status),
                                         'obj': solver.ObjectiveValue() / SCALE, 'cap': cap}

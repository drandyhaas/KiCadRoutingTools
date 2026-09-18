# awx -- the K-bus chain (#622)

Routing a fanned-out bus between two BGAs: one PLAN decides both ends of
every net, the FANOUT lays exactly the plan's moves and reports what it
could not, the BRAID routes the lanes in a corridor of two pages, and a
refused lane is negotiated rather than left open.

    bash chain_k.sh TAG 15 28 41            # -> tmp/TAG_k<K>.kicad_pcb, graded
    python3 make_bench.py BOARD SRC DST OUT # another array pair, from any board
    bash pose_gate.sh BOARD SRC DST 15 28   # the same pair in every pose
    modal run awx/modal_k.py --arms awx/arms.example.json   # a sweep, one container per arm

(We have no idea what `awx` stands for. The name predates every note
that mentions it.)

## Where it stands

The bench is `fb_t2q_fresh`: an H3 BGA `U1` to a DDR3 `DU1`, routed over
the coherent K-ladder. Byte-deterministic given a plan, 0 DRC at the
routed 0.1 mm floor, identical under translation, turn-over and quarter
rotation (the pose gate). Note K51 on this bench is a **48-net** problem
-- `coherent_nets` counts whole rivers -- on every platform.

**Best measured, local** (`awx/tmp/s13/rp_rp_k*`, each 0 open / 0 DRC,
re-graded independently and carrying the same out-of-run nets as the
reference boards):

| | K28 | K35 | K41 | K51 |
|---|---|---|---|---|
| `jcl`, the chain's reference arm | 34 | 60 | 80 | 115 |
| the two-level portfolio (NOW THE DEFAULT) | 34 | 60 | 74 | 98 |
| **+ `replan.py` on top** | **34** | **58** | **68** | **96** |
| human | 46 | 58 | 70 | **81** |
| rule (vias + mm/7.5) | 121.5 | 175.3 | 218.9 | 281.1 |

**K28 beats the human, K35 ties it, K41 beats it. K51 is the only rung it
still wins** -- 96 against 81, down from 107.

**A number is meaningless without its arm, and cloud numbers are a
DIFFERENT MEASUREMENT from local ones.** The two platforms stop the first
CP-SAT solve at different feasible points, so they start from different
plans; the same configuration that gives 98 at K51 locally gives 115 on
Modal. Compare cloud to cloud, local to local, and always check the
canary (objective AND bound) before reading two runs against each other.
Worse, the canary is not even stable run-to-run on one idle machine: the
same model gave 727.2 / 729.7 / 730.2 across six K28 runs. The only
configuration that escapes this is `PLAN_PAGES_CANON` (below), which is a
measurement instrument, not a production setting.

**Defaults that changed 2026-09-16/17:** the two-level portfolio
(`CHAIN_FANOUT_AB` + `CHAIN_BRAID_AB`) is ON -- set `CHAIN_FANOUT_AB=0`
for the single-shot chain, NOT `CHAIN_BRAID_AB=0` alone.

## The chain

1. **`coherent_nets.py K`** -- the first K routable nets of the coherent
   ladder (`k_ladder_coherent.txt`: whole rivers, tightest first; a
   prefix never splits a river).
2. **`fanout_from_plan.py OUT K --board=BASE`** -- one loop over the
   plan and both fanouts.
   - `plan_state` reads the board as it is: the legal escape menus at
     both ends (`escape_moves.py`), the launch points, each tooth's
     layer and vias, the taut-path buses.
   - The destination is chosen against those teeth (`select_moves.py`),
     the source refined against that destination
     (`plan_ends.refine_source`), and the refinement REALIZED by
     `source_realize.py` with the production engine, audited per
     dimension (face, gap along the face, layer, kind).
   - **Feedback**: a move the engine did not lay exactly leaves that
     net's menu (`banned`, keyed by `source_realize.move_sig`) and the
     round re-plans. The engine is the authority on what is possible.
   - The plan is judged by `plan_ends.judged_cost` -- the vias its own
     model implies plus the ride round both arrays at `VIA_MM` per via,
     inside the corridors the braid will actually form.
3. **`braid.py --board FO --dest DU1 --nets ... --out STEM`** --
   corridors from the geometry (`corridor.py`), a spine per corridor,
   launch and target orders from the lanes' offsets, the two-page
   schedule (`schedule.py`), and every lane routed by the real router
   inside its band (`connect.py`, `topo_strings.py`). Up to six attempts
   widen the launch pitch; the best is kept. A refused lane gets a wider
   last call, then a BLOCKER-DIRECTED RIP (the router's blocked frontier
   attributed to this run's lanes, a min-cut probe naming the cut set,
   victims re-laid or negotiated one level down). What is still refused
   is reported and left open.
4. **`grade_k.py BOARD NETS`** -- connectivity scoped to the run's nets,
   whole-board DRC at the routed floor, and the via census.

`chain_k.sh` runs a **flow frame** first (`flow_frame.py`): the pair is
turned by the quarter turn that points source-to-destination along +x,
every stage runs on that file, and the result is turned back -- so a
pair dropped at any of the four angles is the identical computation.

## The ingredients, in pictures

<img src="img/k28_corridor.png" alt="K28 on the bench" width="760">

*K28: two pages. A front lane and a back lane cross for free; a page
lane keeps its layer through the schedule region and pays a via only
where its tooth or berth is on the other layer. The board shown is 38
vias for 28 nets; the reference arm is 36 today.*

<img src="img/k41_east_face.png" alt="The east face at K41" width="520">

*Far-face exits: a berth on the destination's far face is a side exit of
the main corridor whose leg lies beyond the array and whose jog runs
back along the stub's own line -- not a corridor of its own through the
ball field.*

<img src="img/k41_rip_sba2.png" alt="SBA2 after the rip at K41" width="760">

*The blocker-directed rip: SBA2 (highlighted) was refused at the last
call, boxed by lanes routed before it. Its blocked frontier named the
lanes on it, the min-cut probe found the cheapest crossing set, SCKE1
was ripped, SBA2 routed, SCKE1 refused and negotiated in turn by
ripping SA8 -- every lane routed, K41 complete. This is the mechanism
that first completed K41 and K51.*

<img src="img/k51_corridor.png" alt="K51 on the bench" width="760">

*K51: 48 routable nets, complete. The board shown is an early 141-via
one; the joint-solve arm is 137 and the best complete K51 measured is
107. The vias are the work.*

<img src="img/k51_human.png" alt="The human's K51 on the original board" width="760">

*The same 48 nets as the human routed them (`allwinner_h3_ddr3`, the
original board): 81 vias, most nets on ONE layer between their two
escape vias, the address rides nested round the destination, the data
lanes meandered to length. A benchmark to approach, not a pose to
match. `ledger_cal.py` says its realization slack is 6 vias -- the same
as ours -- so what this picture shows that ours does not is a better
PLAN, not tidier copper.*

<img src="img/zynq_k28.png" alt="The second array pair at K28" width="380">

*The zynq article (`make_bench.py --two-layer`): the corridor runs north
from the Zynq to the DDR3, berths on three faces. Complete at 55 vias,
but 17 of 28 in-band -- the gap a second board shows.*

<img src="img/pack_k28_before.png" alt="K28 before the pack" width="400"> <img src="img/pack_k28_after.png" alt="K28 after the pack" width="400">

<img src="img/pack_k41_before.png" alt="K41 before the pack" width="400"> <img src="img/pack_k41_after.png" alt="K41 after the pack" width="400">

*The pack (`pack_board.py`, 4 s, opt-in `BRAID_PACK=1`): 1574 -> 875
segments at K28, every lane, 0 open, 0 DRC, the vias exactly where they
were. Before, the bottom river is a fan of router staircases; after, it
is horizontal lanes with 45-degree jogs. K41 goes 2258 -> 1843. What the
segments BUY is still unmeasured. At K41 the far-face rides
round the top and east become grid legs and the source's wraps nested
chamfers; what stays off the grid is the 6-degree fan from the source's
south row and two coupled corners at the passives.*

<img src="img/spine_chanD_k15_chord.png" alt="chanD K15, the straight chord" width="400"> <img src="img/spine_chanD_k15_relaxed.png" alt="chanD K15, the relaxed spine in grid legs" width="400">

*The spine. With a straight chord the frame runs through the header and
the lanes are squeezed under it one by one (24 vias, 899 segments). With
the relaxed medial spine -- the default -- the ribbon rounds the part in
45-degree legs, lanes a pitch apart through the bend (26 vias, 692
segments).*

<img src="img/gate_mirror_article.png" alt="The bench turned over" width="760">

*The mirror article: the fanned bench flipped through its plane -- every
part on the other face, every stub on the other layer, y mirrored,
self-verified. Every isometry grades as the control to the via and the
segment, which took the selector and the braid each running a pair in
the pair's own canonical frame.*

<img src="img/gate_r30_k15.png" alt="The bench rotated 30 degrees, K15" width="760">

*A 30-degree rotation: complete, but 33 vias and 6 of 15 in-band. The
plan's faces are compass directions and the router's lattice is
octilinear, so a non-orthogonal pose is outside both models today.*

## The tools

| | |
|---|---|
| `chain_k.sh` | the chain; `grade_k.py`, `via_census.py` grade it |
| `fanout_from_plan.py` | the planner and both fanouts |
| `braid.py` | the router: corridors, schedule, lanes, rip |
| `select_moves.py`, `escape_moves.py`, `plan_ends.py` | menus, greedy choice, plan cost |
| `source_realize.py` | realize a source plan with the production engine; `blockers_of` names what stands in a tooth's way |
| `schedule.py`, `corridor.py`, `sched_first.py` | pages, corridors, the schedule-first planner |
| `connect.py`, `topo_strings.py`, `taut_fast.py` | the real router and the taut relaxation |
| `replan.py` | the ROUTE as the judge; re-plans the ends the braid paid for |
| `pack.py`, `pack_board.py` | every lane a taut string against its neighbour |
| **`ledger_cal.py`** | **per net: DP floor vs slack. The instrument that says whether to work on the plan or the realization -- but its per-net floor is CIRCULAR (each net priced against the others AS LAID); read it beside `joint_floor.py`** |
| **`joint_floor.py`** | **the NON-circular floor: one MILP over the fixed paths that picks every path's layer at every crossing at once. Validated against the audit's independent parity+max-cut on three boards. `JOINT_FLOOR_NODES` bounds it -- no clock** |
| `room_probe.py` | **does a plan-time feature predict a swimmer's vias?** Takes FANOUT/ROUTED board pairs, rebuilds the crossing geometry the judge sees (`braid.plan_braid`) and correlates candidate features against the routed count. Every family tried is null (|r| <= 0.13 over 97 swimmers) -- run it before building any new per-lane cost term |
| `modal_k.py` | cloud arms. `return_board: true` returns the routed board, `return_files: [globs]` any tmp/ artifact (plan sidecar, raw logs, judge dumps) -- without these a cloud-only phenomenon cannot be diagnosed at all, and K44's regression is cloud-only |
| `human_at_k.py` | the human's vias for a coherent K set. **Mind the label**: `coherent_nets(51)` returns 48 nets, so "K51" is a 48-net problem -- the human is 81 over those 48 and 85 over the full 51. Both are right; ours route 48 |
| `census_vs_human.py` | per-net vias/copper/layers against the human, and where each via sits |
| `collapse_dives.py` | collapse short dives on a routed board (2 vias each) |
| `cut_ledger.py` | Maley cut capacity of a plan, before any lane is routed |
| `wall_probe.py`, `copper_same.py`, `cmp_copper.py` | track-level wall census, set-compare copper |
| `make_bench.py`, `rotate_board.py`, `mirror_board.py`, `bend_bench.py`, `channel_bench.py` | build an article from any board, and its poses |
| `pose_gate.sh` | the chain over FF / BF / FB / BB / R90 / R180 / R270 |
| `modal_k.py` | fan a sweep onto Modal, one container per (arm, K); `memory=(4096, 12288)` -- the request is the bill, the limit is the safety net |
| `arms.*.json` | the sweeps: `arms.judge.json` (the comparator: `SF_ESC_W`, `SF_ACCEPT_MARGIN`), `arms.prune.json` (`BRAID_L5_ALT_PRUNE` x cap), `arms.next.json` (everything else untested), `arms.solver.json` (the older matrix) |

## One source for every routing number

**Superseded in part 2026-09-15: every quantity below is now DEFINED in
one place, `awx/rules.py`, and installed into these names by each stage.
See "`rules.py` -- one place for the topo chain's design constants" at
the end of this file for the table and the formulas.**

Audited 2026-09-12, after a swimmer was found priced five different ways
and `collapse_dives` was found re-laying at the wrong track AND the wrong
via size. Every routing quantity now has exactly one home:

| quantity | the one source | value | who reads it |
|---|---|---|---|
| braid lane track | `topo_strings.TRACK` | 0.127 | braid (`= ts.TRACK`), pack, cut_ledger, collapse_dives, replan, fanout |
| braid hug clearance | `braid.CLEAR` | 0.105 | braid, pack, cut_ledger, collapse_dives |
| spec clearance | `topo_strings.SPEC_CLEAR` | 0.1 | topo_strings |
| via size / drill | `braid.VIA_SIZE` / `VIA_DRILL` | 0.25 / 0.15 | braid, cut_ledger, replan, fanout, collapse_dives |
| **swimmer price** | **`prices.SWIM`** | one number, five sites | plan_ends, braid x3, sched_first |
| `DIRS`, `LAYERS` | `escape_moves` | | fanout, source_realize, sched_first, replan |

**The board carries TWO track widths on purpose.** The fanout's stubs are
laid by the production engine at its own 0.1 / 0.1
(`source_realize.FAN_TRACK` / `FAN_CLEAR`) -- 23 segments on a K51 board
-- and the braid's lanes at 0.127 / 0.105 -- 2530. A tool that rips a
braid lane and re-lays it must use the BRAID's numbers; `collapse_dives`
used its own 0.1 / 0.1 and 0.45 / 0.25 and so put thinner copper and
oversized vias into the board, which also inflated its reported saving
(137 -> 133 claimed, 137 -> 135 real).

**`braid.CLEAR` (0.105) and `topo_strings.SPEC_CLEAR` (0.1) are different
quantities** -- the spec, and the spec plus 5 um so a hug does not sit
exactly on it. They were both called `CLEAR` until this audit. Do not
import one where the other is meant.

Names that LOOK shared and are not: `TOL`, `STEP`, `MARGIN`, `CAP`,
`PROX_TRACK`, `HW_COL` -- different local quantities that happen to
share a generic name or a number. Leave them alone.

## Measuring honestly

Every one of these cost a session to learn.

- **A board with open nets has artificially LOW vias.** An unrouted net
  lays no copper. Only 0-open boards compare. (`fa` K41 once read 67
  vias -- better than the human -- with 3 nets open.)
- **A single K is not a result.** Run-to-run spread on one board is
  +-2..3 vias. Judge on K28 AND K35 AND K41 together, and include K51
  before claiming "worse on none".
- **K15 and K28 first.** Higher K only once those are great. Nearly
  every richer planner wins at one K and loses at another.
- **Quote the arm with the number**, or it cannot be compared.
- **`BRAID_CPSAT_DET` is not optional.** Without it CP-SAT stops on wall
  clock across 4 workers and the same input gives 80 / 82 / 85 / 80.
- **Verify every new flag byte-identical with the flag OFF** before
  reading anything off it.
- **The probe must use the chain's own env and board.** `DST_WALK` and
  `DST_FACE_ASK` change the MENU; a probe run without them measures a
  different problem (measured: 45 berths seated vs the chain's 34).
- **THERE ARE NO CLOCKS. Every budget is in WORK, and that is a rule,
  not a preference.** A clock budget does not make a slow machine answer
  later, it makes it answer DIFFERENTLY. Measured: two identical cloud
  runs of the K35 baseline -- same image, same env -- came back **72
  vias / 1436 segs and 58 / 1840**, because a container is ~2x slower
  than the laptop (K51 1330-1574 s against 600-900) and five wall-clock
  budgets that never bind locally bound there. That was also the likely
  source of the "knife edge" +-2..3 via spread under load.
  So: every search loop is capped in **judge calls**
  (`fanout_from_plan.PLAN_CALLS`; `DST_RESIDUE_CALLS`,
  `DST_SEARCH_CALLS`, `SRC_REPLAN_CALLS`, `SF_REPAIR_CALLS`), HiGHS is
  capped in **nodes** (`BRAID_MILP_NODES`, or a stage's own `nodes`) and
  CP-SAT in **deterministic time** (`interleave_search` +
  `max_deterministic_time`, always on -- `_milp_solve`'s `time_limit`
  parameter is accepted and IGNORED). `py_router` was already clean.
  **Do not add a `time_limit`, a `max_time_in_seconds` or a
  `time.time()` guard that decides an output.**
- **Modal is a different numeric era.** The cloud image pins numpy,
  scipy, ortools, shapely and grid_router to the local versions but runs
  python 3.13 against the local 3.14, and the baseline ladder comes back
  72 / 76+1open / 118+2open against the local 58 / 80 / 137. Cloud
  numbers compare ONLY to cloud numbers.
- **Look at the renders.** `../py_router/route_render.py`; the copper is
  the plan.

## What the human does

Two vias per net, at the ENDS, on a constant layer -- `census_vs_human.py`
shows it as `DS` or `DD` on nearly every net. Our excess is at the ends
too, but piled: at K51, SBA1 `DDDDDDSS` (+6), SA1 `DSSSSS` (+4), SCS1
`DDDDDS` (+4) carry 14 of the 23-via deficit between them, with near
double path length (SA1 61.4 mm against 30.9). Five nets beat the human
by crossing the field on F with zero vias.

The human's berth order gives 68 crossings at K41 against our 250, and a
longest crossing-free chain of 29 against our 21 -- but see the floor
finding above before treating crossings as the objective.

## The pages-first planner (`PLAN_PAGES=1`, 2026-09-13 night)

Andy's directive: **no net may need more than two vias.** A page lane costs
at most two (one at each end whose layer is not its page); only a SWIMMER
costs more. So the plan must hand the braid orders two crossing-free chains
cover, and that is a property of the ENDS. Measured first (`two_chain.py`,
`one_change.py` in the c79d1062 scratchpad; `hbn_k41/51` = the human's
copper stripped to a fanout, `human_bench.py` rewritten): on our recorded
K41 / K51 plans the best any two-page schedule can page is 28 of 41 / 28 of
47 (13 / 19 must swim); on the human's ends 35 / 41 (6 / 6). Why the human's
ends are orderable: once a net has a B end its page is free at no cost, and
a B end makes its RANK free (a dog-bone climbs under the array); an F end
pins it. Our chain froze the source, had no destination climb class, and
every chooser priced a swimmer at 2 vias and took it.

`pages_first.py` (opt-in, the old planner byte-identical at 0 -- K15 and
K28 flag-off copper IDENTICAL to the recorded boards): ONE CP-SAT chooses a
destination move, a source move ({the tooth as it stands} + `smenu`) and a
page for every net; HARD: two nets on one page are never inverted between
the launch and target orders; a net may still swim at 100 vias. Keys = the
braid's own slots (`braid_slots`: corridors built on the seed plan,
`_alt_geo` + `_alt_slot` / `_alt_src_slot`, the current tooth by `launch_o`);
`verify` runs the braid's planner on the answer and a DAMPED loop re-solves
only the nets it swims (each barred from the berth that swam) with every
other net held, PLAN_PAGES_ITERS times. Exclusions from `_conflict`,
bucketed. Deterministic (interleaved CP-SAT under PLAN_PAGES_DET). Hooks:
`dest_choice` (after the greedy; `src_free` only where a source move can be
realized), `plan()` realizes the planner's teeth through the existing
realize-confirm loop and skips the paper refine, `fanout_destination`
re-plans with the exactly-laid berths FIXED, the judge under the flag is
(braid residue with EXACT pages, the planner's own vias), the sidecar
carries `pages_first` and `braid.setup` pages such a plan exactly.
`DST_CLIMB=k` enumerates destination climbs (off; the class that frees a
B berth's rank -- not yet measured).

| K | base (this tree) | first pages-first | after the night's fixes (arm) | human |
|---|---|---|---|---|
| 15 | 16 / 0 | **14** (0 swimmers) | 14 | 22 |
| 28 | 36 / 0 | 44 (2 swimmers) | **32** (siders=1 + joined key, 31 s) / 34 (far-only) | 46 |
| 35 | 62 / 0 | **58** (8 swimmers shipped) | 60 (siders=1 + joined key, 60 s) | 58 |
| 41 | 91 / 0, 174 s | 122 (19 swimmers), 132 s | 96 clean (conflict fixes, 113 s); 92 with 2 open (siders=1 + joined key) | 70 |
| 51 | 112 / SBA2 open, 266 s | 134 / 0 open, 234 s | -- | 85 |

The night's fixes, in order (each measured, all opt-in under the flag except
the two shared bugs): (1) a strict, sticky destination loop; (2) keys from
the braid's own slots with a damped verify loop; (3) the judge = braid
residue with exact pages + the planner's vias; (4) `schedule.exact_pages`
tie-break (shared); (5) `select_moves._site_in_lane` tests ANY via, not
only dog-bones (shared; standard planner's K15/K28 copper identical);
(6) the full row-vs-column crossing test under the flag (`sm.SEL_XING=2`)
plus crossing BUCKETS in the planner's own conflict finder -- K41 122 -> 96;
(7) `PLAN_PAGES_SIDERS` (2 = far-face stubs always side exits; 1 = side
faces too, which costs 2 vias per formerly head-on F arrival: K35 58 -> 76);
(8) `PLAN_PAGES_JOINKEY`: a side exit's slot depends on its SOURCE class
(the braid's exit block: ports innermost by stub s, then the lanes whose
tooth is a joiner outermost by tooth s) -- with (7) the K28 keys match the
braid on 378/378 launch and 377-378/378 target pairs, 0 swimmers, and the
chain gives 32. Open at K41: the SOURCE side's relative head-on test (a
tooth chosen downstream on the same line turns a tooth into a joiner) and
the moved teeth's insertion slots -- 37/820 launch pairs off. Generality:
every rule is geometric (direction against the spine, face lines, comb
order along the spine, a via on a lane); no net, face or bench name; the
joined shift is derived per corridor.

All 0 open / 0 DRC. Two findings the ladder exposed, both open:
- **The braid's relative head-on test** (`_head_exit`: on a face parallel
  to the spine the most upstream stub is head-on, every other a side exit)
  makes a candidate's class -- and with the spine relaxed on a new stub set,
  every (s, o) -- flip with which neighbours are chosen. Launch order agreed
  378/378 at K28, target 26/378 off, all up face and far face. The damped
  loop gets K28 from 4 to 2 braid swimmers; keying side faces by the comb
  rule directly (`PLAN_PAGES_SIDEKEY=1`) measured WORSE (2 -> 4), off.
- **The destination loop erodes the plan.** K35 started at 1 braid swimmer;
  every pass banned the berths the engine did not lay as asked and re-planned
  with the rest fixed, and the shipped plan had 8 (5 -> 7 -> 10 in the log).
  Faithfulness of the engine to the asked berth (EXACT_LANE legs, the
  bans' scope) is the next lever.
- **Fixed on the way, in the shared code:** `schedule.exact_pages` priced a
  swimmer at exactly the price of a lane with both ends off its page and was
  free to leave it swimming (4 of 6 on one plan); `SWIM_TIE=0.01` prefers the
  page lane. Applies to the old planner's opt-in `BRAID_EXACT_PAGES` arm; the
  default greedy pager is untouched.
- **Applied to the standard planner too (Andy, 2026-09-14):** (1) the
  source is frozen wherever it cannot be realized -- `residue_choice` offers
  a candidate tooth only when its caller has an outlet (`src_out`), so the
  destination re-plan loop no longer solves with teeth that never move
  (DST_RESIDUE_SRC arms; the default proposes none, byte-identical); (2) the
  exact pager's tie-break is shared code, so the standard planner has it
  wherever `BRAID_EXACT_PAGES=1` is on (its default greedy pager pages
  whenever it can and has no tie). Still standard-planner-only defects:
  sched_first's Frame keys carry the order defect the braid slots fix; the
  realize-confirm acceptance key is shared by the DST_RESIDUE_SRC arms.

### The arms measured on this planner, in brief

The six dated logs that were here (2026-09-14: the base re-measured, the
exit-leg term, K51 completion and the braid's count as the judge, the
BUNDLES diagnosis, the six-pages probes, the wrap spine) are in git. What
they settled and is still live:

* **the judge is the braid's own count plus lane length** (`PLAN_JUDGE=count
  PLAN_JUDGE_LEN=lane`), the `jcl` reference arm -- every hand-built model
  lost to it;
* **the human routes BUNDLES by (arc, layer)** where we route lanes, which
  is the K51 shape and is still unsolved;
* **more solve time gives a better plan and a worse route** -- first seen
  here, later proven at the optimum (see "Recent findings");
* the wrap spine and the six-page probes were built and did not land.

## What this adds to `py_router`

**`KICAD_SEG_DIST_EXACT` (default ON, and it changes EVERY board).**
`single_ended_routing._seg_foreign_seg_dist` used to sample our segment
every 0.02 mm and measure each sample to the foreign segments, so its
minimum was always >= the truth. The minimum distance between two
segments is attained at an endpoint of one of them unless they properly
cross, so four point-to-segment distances over the foreign segments give
the exact value -- smaller or equal, never larger, so the router sees
obstacles a hair earlier and can only become MORE conservative.
Profiled at 27,900 calls / 25 s of a 149 s K41 braid. This is the one
change here that is not scoped to the awx chain, and it owes the corpus
A/B before it reaches main (TODO item 7). `KICAD_SEG_DIST_EXACT=0` restores
the sampled sweep.


`generate_bga_fanout(..., escape_dir_hints=...)`: a per-pad planned
escape keyed by board-frame pad position -- a bare FACE (`'down'`) or a
FULL MOVE (`{'face', 'exit', 'layer', 'kind', 'site'}`). The under-pad
engine follows a full move in its plan-follow phase
(`underpad._follow_plan`): planned balls leave the generic phases, their
via sites are reserved first, every ball is routed to its EXACT move
deepest-first, a ball whose exact move is blocked negotiates (its
blockers among the same call's escapes are found on a pre-commit
occupancy snapshot, ripped, the ball laid, the blockers re-laid, and the
state kept only if the count of balls landed as asked rises), and what
is still short degrades along the least damaging dimension: nearest free
gaps first, then the other layer/kind, then any face. Every ball's
outcome is reported per dimension in `pcb_data._fanout_plan_report`.

### The arc line, ABANDONED (2026-09-14)

The four subsections that were here -- the arc-corridor MODE, the slanted
comb, the plan choosing the arcs, and that evening's handoff -- document a
line of work that was **abandoned**, and the tree was reverted to
`2daff560`. Flag-off was byte-identical and the arcs either lost or routed
open. The one durable result from it is the slanted comb reaching the
human's 80 / 0 on `hbn3_k51` when handed the human's ends, which is what
sent the work to the PLAN side rather than the braid side. Detail in git.

## The synthetic harness (agent, 2026-09-15)

Every number above this line comes from **one** bench -- `fb_t2q_fresh`, an
H3 escaping into a DDR3 -- graded against **one** human layout. That gives a
ranking (better or worse than 81 vias at K51) but never an *optimum*, so a
result of "106 vias" carries no information about whether 106 is 30 too many
or 2. Andy's ask: *"a test harness that gives the planner and router a large
set of generated test cases with known answers to solve and route, to see
where it is not optimal."*

Two files, both general -- nothing in either names a board, a net, a face or
the bench:

* **`synth_bus.py`** writes a case: a 2-layer `.kicad_pcb` + `.kicad_pro` at
  the bench's 0.1 mm process, two BGA arrays facing across a channel, `K`
  two-pad nets in a chosen pin pattern, optional foreign parts, and a
  `<stem>.truth.json` carrying the planted permutation and its answer.
* **`synth_ladder.py`** runs a batch: generate, `make_bench.py` (source
  fanout + DRC floor + coherent ladder), `chain_k.sh` (plan + destination
  fanout + braid), then grade against the answer.

```bash
python3 synth_bus.py --self-test              # the truth model checks itself
python3 synth_bus.py out.kicad_pcb --k 15 --pattern interleave
python3 synth_ladder.py --batch b1            # 23 cases, patterns x K
python3 synth_ladder.py --batch b3            # the corridor-obstacle ladder
python3 synth_ladder.py --batch b1 --regrade  # re-grade, no chain
```

### Three answers, and they bracket

For a peripheral bus (every used ball on the facing column, so both ends of
every lane are on F) the channel is a permutation and two lanes cross iff
their order is inverted. Then:

| | what it is | when it exists |
|---|---|---|
| `lb` | `2 * (K - LIS(pi))` | always -- and still valid with an obstacle |
| `opt` | best **whole-lane** solution = the proper 2-colourings of the crossing graph, `2 * sum min(part)` | only when that graph is bipartite |
| `dp` | best over **all** routings, mid-channel layer changes included, by a DP over the crossing events | K <= `--dp-cap` (22; 0.02 s at K=15, 4.5 s at 22, hopeless at 28) |

`dp` is the number to grade against. It is what turned `reversed` and most
`shuffle` seeds from "no answer" into an answer: the whole-lane model returns
nothing for an odd cycle, which is exactly the interesting half of the space.
`--self-test` asserts `lb <= dp <= opt` on every pattern at K in 4..12 and
checks the two closed forms, and it reports two facts worth keeping:

* on `sorted`, `blocks`, `interleave` and `riffle` the whole-lane optimum and
  the exact optimum **agree** -- mid-channel changes buy nothing there, which
  is what makes those planted cases clean;
* on `reversed` the LIS bound is **tight** at every K tested, so `2*(K-1)` is
  the answer even though the crossing graph is a clique.

The DP is exact for **one crossing order** (the straight-line one, with a
deterministic tie-break) and one homotopy class. That is the honest caveat
and it is written into `crossing_events`' docstring.

### The gaps

```
PLANNER gap = the braid's plan-implied count (judge_gate c_sw) - the optimum
BRAID   gap = the routed vias                                 - that plan count
DP      gap = the routed vias                                 - the optimum
DETOUR      = routed copper mm / straight-line mm
inband/offered = the braid's own `lanes: a/b routed` lines, summed
```

`inband` is there because "all lanes in band" is the objective the planner is
actually optimising, and a case can route perfectly while leaving it.

Three columns decide whether a number means anything:

* **`thru`** -- lanes with copper *inside* an array. The optimum is
  channel-confined, so `thru > 0` means a cheaper topology than the model
  describes was available and a negative gap is **not a win**. The `b1`/`b2`
  pair exists to separate this: the same 23 cases, `b2` with the array
  interiors closed by fattening the unused balls (`--pad-inner 0.6`).
* **`slot_cap`** -- on an obstacle case, how many lanes can physically pass by
  counting. Without it `open=4` cannot be read: a router defect if the lanes
  fit, a correct refusal if they do not.
* **`k_real` vs `k_asked`** -- `coherent_nets` counts whole *rivers*, so the
  ladder can hand the chain fewer nets than asked. The truth is re-derived
  from the prepared board, never trusted from the sidecar.

### What the first batches measured

`b1` (23 cases, open arrays), `b2` (the same, interiors closed), `b3` (14
corridor-obstacle cases). Summary lines as run:

```
b1: 18 with an exact answer: routed 206 vs optimum 254; 11 exact, 0 open, 0 DRC.
    11 channel-confined (thru=0): routed 112 vs 112 -- ALL 11 EXACT.
    in band 285/310 lanes; 4 cases left the band.
b2: 17 with an exact answer: routed 228 vs optimum 240; 11 exact, 0 open, 0 DRC.
    13 channel-confined: routed 130 vs 124, 10 exact.
    in band 271/311; 4 cases left the band.
b3: 14 with an exact answer: routed 260 vs optimum 118; 1 exact, 4 with opens.
    in band 171/580; ALL 14 cases left the band.
```

**1. On a clear channel the chain is at the optimum.** Every one of the 11
channel-confined `b1` cases routed exactly its known optimum -- `sorted`,
`blocks`, `interleave` and `riffle` at K=8, 15 and 28, DRC-clean and with no
open nets. That is a real result and it is the first time the campaign has
been able to say it. It also means the corpus bench's remaining gap is not a
generic "the braid wastes vias" defect.

**2. Two vias on a bus with zero crossings.** `sorted_k15` with the array
interiors open routes at **0 vias**, every lane straight on F (left). The
identical permutation with the interiors closed pays **2** (right) -- one
lane dives to B and back at the destination berth, and no crossing requires
it. Same for `sorted_k8` and `sorted_k28`. A 15-net, 3-second, known-answer
reproduction of a berth-side via the plan did not need.

<img src="img/synth_sorted_k15_open.png" alt="sorted K15, open interiors: 0 vias" width="380"> <img src="img/synth_sorted_k15_closed.png" alt="sorted K15, closed interiors: 2 vias" width="380">

**3. The escape field breaks the planner, not the braid.**
`interleave_k15_dep2` (bus balls drawn two columns deep, so the teeth are a
real escape field rather than one clean column) is the largest planner gap in
`b1`: plan-implied **28** against an optimum of **14**, with the braid then
recovering 10 of them (routed 18). The braid gap is negative and the planner
gap is +14 -- the plan was the wrong half.

**4. A part in the corridor is where it falls apart.** `b3` walks a
through-hole blocker up the middle of the channel and grows it, so the bus
must fan in past it through a shrinking slot at each end. On `sorted`, whose
clear-channel optimum is **0 vias**:

| obstacle | room each side | `slot_cap` | routed | open |
|---|---|---|---|---|
| 2x12 mm | 6.4 mm | 116 | 8 | 0 |
| 2x18 mm | 3.4 mm | 56 | 14 | 0 |
| 2x20 mm | 2.4 mm | 36 | 22 | 0 |
| 2x22 mm | 1.4 mm | 16 | 26 | **4** |

Nothing in this ladder needs a single layer change: the lanes do not cross,
and a group passing above and a group passing below both stay on F. The
render (left, the 2x20 row) shows what happens instead -- the bundle wraps to
the board edge at 2.11x detour with most lanes flipped to B and back, and a
knot of stacked vias at the destination.

The `open` column is the sharper finding, because `slot_cap` says the lanes
fit. At 2x22 the counting bound is 16 lanes against K=15 and the chain
strands 4. The off-centre case is worse (right): the blocker is pushed down
so every lane must pass over the top, `slot_cap` is **68**, and the chain
still strands **4 of 15** -- crowding the survivors into a tight bundle
hugging the obstacle while the top third of the board sits empty. This is
the session-11 "parallel bends at the lane pitch go infeasible" class,
reproduced on a generated board with a known answer.

<img src="img/synth_obs_sorted_k15.png" alt="sorted K15 with a 2x20 blocker: 22 vias against an optimum of 0" width="380"> <img src="img/synth_obs_offcentre_k15.png" alt="off-centre blocker: 4 of 15 nets stranded with room for 68" width="380">

**5. The in-band objective collapses under an obstacle.** All 14 `b3` cases
left the band, 171 of 580 lanes in band on the first attempt, against
285/310 on the clear-channel `b1`. The band model has no representation for
"the corridor is obstructed", so the plan it hands the braid is one the braid
cannot execute, and everything after that is recovery.

**6. A negative control that fired.** `interleave_k8_pi0.6_obs2x18` reports
`slot_cap = 0`: the K=8 article is shorter, so an 18 mm blocker seals the
channel. The chain leaves 8 nets open and the harness says that is **correct
refusal, not a defect**. Keep this case -- it is the proof that `slot_cap`
discriminates, and without it the 4 opens in the rows above would be
unreadable.

**7. One case does not build, and it is a real refusal.**
`interleave_k15_dep2_pi0.6` (depth 2 *and* the interiors closed) fails in
`make_bench`: with 0.6 mm unused balls there is no escape for a ball in the
second column. The driver now names it instead of crashing -- the first
`b2` run died in `print_table` on a `KeyError: 'routed'` because a
build-failure row carried only a tag.

### What this harness can and cannot see

**Can:**

* price a plan and a routed board against a **real optimum**, not a ranking;
* separate the **planner** gap from the **braid** gap on the same case;
* say whether an open net is a router defect or a physical impossibility
  (`slot_cap`), and whether a *negative* gap is a win or a broken model
  (`thru`);
* sweep pin pattern, K, channel width, destination rotation, escape depth,
  foreign parts, array porosity and a corridor obstruction independently,
  deterministically and with no clock in any budget;
* run a case in seconds, so a defect found here is a fixture, not an
  expedition.

**Cannot:**

* **judge anything about a real board's geometry.** These are two facing
  rectangular arrays in a straight channel. No corners, no mixed pitches, no
  irregular ball maps, no power/ground obstruction of the escape field, no
  more than two layers -- and the bench's K51 defect lives partly in exactly
  those. A pass here is necessary, not sufficient.
* **price anything but vias and copper length.** Nothing here scores
  crosstalk, matched length, or the *quality* of a berth.
* **claim its optimum outside the model.** The `dp` answer assumes each
  inverted pair crosses exactly once, in the straight-line order, in the
  channel. A lane that reaches its ball through the array is outside it --
  which is why `thru` is printed and why `b2` exists, and why a negative gap
  must be read as "the model does not apply here", never as "the router beat
  the optimum".
* **see the obstacle in its own optimum.** `b3`'s `dp` is the *clear-channel*
  answer; the excess over it is the price of the obstruction, and no claim is
  made that the excess is minimal. The `slot_cap` bound is likewise an upper
  bound that ignores the bends into and out of the slot -- it says "there was
  room", not "a routing exists".
* **stand in for the corpus.** It is a microscope, not a regression suite.

### Where to take it next

1. **Case 2 is the cheapest open defect in the repo**: 15 nets, no crossings,
   3 seconds, 2 vias where 0 is provably right, and it only appears when the
   array interior is closed. Diagnose the berth.
2. **Case 3** says the escape-field plan is the thing to fix, and a depth
   sweep (`depth` 1..4) would say how it degrades.
3. **Case 4/5** want the band model to know a corridor can be obstructed. The
   obstacle ladder is the gate: `sorted` with a blocker should cost 0 vias.
4. A `--pattern shuffle --inversions N` sweep at fixed K would give a
   dose-response of gap against crossing count, which none of these batches
   has (the patterns are corners, not a curve). **Done** -- it is the second
   half of `b5`, below.

## The harness grades the JUDGE (2026-09-17)

TODO item 1 is "fix the judge", and the evidence for it was one number on one
board: at `PLAN_PAGES_DET=5000` the K41 plan solves to PROVEN OPTIMAL with a
24% better objective and routes **twelve vias worse**. That says the objective
is wrong somewhere but not where, and re-measuring it costs 43 minutes. The
synthetic harness can say where, in seconds, because it knows the answer.

### The two new truths

* **`pages_model(...)`** -- the exact optimum of the **planner's own model**.
  Two lanes are inverted exactly when they cross, so "a page" is a
  crossing-free set is an *increasing subsequence*: the model's optimum is to
  cover the lanes with two increasing subsequences, pay each its end-mismatch
  price and pay `PLAN_PAGES_SWIM` for every lane left over. An O(K^3) DP over
  (last rank on page F, last rank on page B) -- so it answers at K=51 in
  milliseconds, where `exact_dp` stops at 22.
* **`exact_dp(..., fixed=...)`** -- the exact optimum with lanes PINNED to a
  page for the whole channel. That is what a whole-lane plan *is*, so
  `exact_dp(fixed=the model's pages) - exact_dp()` is the exact price of
  planning in whole lanes. A pinned lane is not a variable, so the walk runs
  over the FREE lanes only: the cap applies to the swimmers, not to K.

Both are checked three ways in `--self-test`: brute force over F/B/swim at
K<=8, **Greene's theorem** (the largest union of two increasing subsequences
is `lambda1 + lambda2` of the RSK shape, computed by a different algorithm),
and closed forms. A ten-mutant battery kills nine; the survivor
(`bisect_left` -> `bisect_right` in the RSK insertion) is provably equivalent
on a permutation and is recorded in the code as such.

### What a planner gap is made of

    MODEL error = the best plan the two-page model can express  - the optimum
    solve_vs_model = the plan this solve actually found         - that best plan

They answer opposite questions, and a bare `planner_gap` mixes them: a model
error means more solving is **wasted**, because the answer is not in the model
to be found. New ladder columns: `paged`, `m_swim`, `m_obj`, `m_dp`,
`model_err`, `m2_dp`, `swim_price_cost`, `solve_vs_model`.

The first thing it measured is that **b1..b4 cannot see the model at all**. On
`sorted`, `blocks`, `interleave`, `riffle` -- and even on `reversed`, where the
model swims K-2 lanes -- the model's best plan costs *exactly* the optimum. The
model error lives on IRREGULAR permutations, which is why `b5` exists:
`shuffle` at three seeds and three K (the only family whose crossing graph is a
general permutation graph), interiors closed, plus the inversion
dose-response. Measured: `shuffle` K=12 seed 0 is **+14 vias** (34 against 20),
K=15 +10, K=18 +4. All three are pinned as self-test witnesses.

### `--judge`: is the objective pointing the right way?

```bash
python3 synth_bus.py --judge                     # ~1 min, no board, no chain
python3 synth_bus.py --judge --judge-swims 1.5,2,100 --judge-k 10,12
```

It builds a pool of model-feasible plans, scores each one **both ways** -- the
planner's objective, and the exact via count of the best routing consistent
with it -- and reports the rank correlation, the regret of the plan the
objective would choose, and the spread of true cost inside the objective's own
optimum set. Over 60 cases at K=10..18:

| swim price | mean rho | total regret | mean spread inside the optimum |
|---|---|---|---|
| 2 | +0.18 | +340 | 5.6 |
| 3 | +0.06 | +326 | -- |
| 6 | -0.03 | +330 | 4.4 |
| 20 | -0.05 | +330 | -- |
| **100 (shipped)** | **-0.05** | **+330** | **4.4** |

**The objective is rank-uninformative about the routed via count, and at the
shipped weights very slightly anti-correlated.** Regret is a STEP with the step
at 2 -- every price below 2 measures +8..+12, every price above it +326..+340 --
because a swimmer that routes freely costs exactly 2.

**The blind spot, and why the price is not the finding.** In this model a
swimmer is FREE: `exact_dp` routes the unpinned lanes optimally. The real braid
routes a swimmer outside its page chains and may pay far more, or fail. So an
objective that prefers swimmers scores well here **by construction**, and
nothing in this table sets `PLAN_PAGES_SWIM`. Only a chain run can.

### The arm the blind spot cannot reach

Group the plans by how many lanes they swim. Inside one group every plan pays
the same swim total, so whatever a swimmer really costs, it costs the same in
all of them and the price cancels. What is left is the objective's opinion
about *which* lanes to page:

```
case                  plans  swims     rho   TRUE spans  objective spans
blocks_k12               59      7   -0.03       12..40         702..708
interleave_k12           56      6   +0.56       10..22         602..610
reversed_k12             37     10     --        34..42       1002..1002
blocks_k15               53      9   +0.23       14..48         902..910
reversed_k15             36     13     --        50..58       1302..1302
```

**mean rho +0.40 over 36 groups, mean true span inside a group 13.7 vias, and
9 groups where the objective is CONSTANT across a spread of up to 12 vias.**
Not weakly correlated: one value, no ranking. No budget, no solver and no
tie-break inside this model can choose between those plans, which is why
proving the K41 plan optimal moved the board twelve vias the wrong way, and
why `PLAN_PAGES_CANON` (making the degenerate choice *reproducible*) treats
the symptom. `blocks_k12` is the sharpest non-degenerate row: the objective
spans 6 over plans whose real costs span 28, and does not rank them.

**The jitter, and why it is on by default.** `reversed` crosses every pair at
the MIDPOINT, so on the article exactly as drawn the crossing *order* is
`crossing_events`' deterministic tie-break rather than a fact about the
problem -- and a spread read off it is partly an artifact. Measured at K=10:
14 vias degenerate, 6..12 across five jitter seeds. `--judge-jitter` (0.30 of
a pitch by default) moves the ends by less than half a pitch, so no pair's
order changes and the crossing GRAPH is identical, but every crossing gets its
own place along the channel. The numbers above are the jittered ones; the
degenerate article reads 14/18/24/30 and that reading is **withdrawn**.

The degeneracy is concentrated where the model must SWIM. When it pages
everything the objective is unique and its regret is zero -- `blocks`,
`interleave`, `riffle` at every K. **And the bench sits in the swimming
regime**: with the ends as the board has them, `pages_model` pages 13 of 28 at
K28, 16 of 41 at K41 and 20 of 48 at K51 (the planner does better because it
also *chooses* the ends, so read those as a floor on pageability, not as the
planner's own number).

### And then run it, because only the chain prices a swimmer

`b5` twice, same 17 cases, the two arms in parallel:

```
                   vias   open   in band (first attempt)   wall clock
swim=100 (shipped)  292      0   364/397, 6 cases retried     16.5 min
swim=2              272      0   231/231, none retried        10.8 min
      B better on 6, worse on 0, tied on 11.  -20 vias, and 1.5x faster.
```

The two arms produced a **different plan on 12 of the 17 cases**, so this is
the price moving the plan and not noise in the braid. The `in band` column is
the mechanism in one number: arm A's offered counts are 36, 52, 54 against a
K of 12 and 18, which is the braid *retrying* -- the high price hands it a plan
whose lanes do not fit their band, and everything after that is recovery.

**This is a result on ONE generated family and it is not a default.** `b5` is
`shuffle` at three K and one inversion sweep, all from one generator, on a
straight two-array channel. Per the edicts, the gate is the K28/K35/K41 bench.

### What a swimmer really costs, from the braid's own mouth

The harness's blind spot has a number, and it was in the logs the whole time.
The braid reports, per swimmer:

```
swimmer SYN00: 8 page crossing(s), 6 change(s), 6 diamond(s) reserved
```

`swim_price_real` reads those and the ladder now carries `swimmers` and
`swim_changes`. Both `b5` arms agree, and they agree independently of the
price the planner charged:

| arm | swimmers | changes | per swimmer |
|---|---|---|---|
| swim=100 | 20 over 10 cases | 70 | **3.5** |
| swim=2 | 86 over 15 cases | 291 | **3.4** |
| the bench, K28, swim=2 | 9 | 34 | **3.8** |

**Neither model has this number.** The exact DP prices a swimmer at 2, because
it routes the free lanes optimally; `pages_first` charges 100. The braid pays
about 3.5.

### The bench says NO, and says it in the clearest possible way

`chain_k.sh` at K28, `PLAN_PAGES=1`, one price apart:

| PLAN_PAGES_SWIM | what the planner chose | its objective | **routed** |
|---|---|---|---|
| 100 (shipped) | pages F 18 / B 10, **0 swimmers** | model vias 28 | **34** |
| 2 | pages F 25 / B 3, **10 swimmers** | model vias **8** | **42** |

The objective improved by 71% and the board got **eight vias worse**. That is
the campaign's headline anti-correlation -- which cost a 43-minute K41 solve at
`PLAN_PAGES_DET=5000` to establish -- reproduced in two minutes by moving a
price instead of a budget. It also explains *why*: at 2 the model buys
swimmers at less than half what the braid charges for them, so it buys too
many; at 100 it will not buy one at any price.

**So the swim price is NOT a default change, and the synthetic gain does not
transfer.**

### A flat price cannot be right, so: `PLAN_PAGES_SWIM_XING`

Over **262 swimmers** from every run of the K ladder and of `b5`, the braid's
own reported cost is not a constant -- it runs from 2 to 22 changes, and it
tracks the page crossings:

```
changes ~ 0.44 * page_crossings + 0.79      (Pearson r = 0.83, n = 262)
70% of the variance, against a flat mean of 4.98 -- and the model charges 100.
```

`page_crossings` -- how many PAGED lanes a swimmer crosses -- is **pairwise**,
which is the shape the no-inversion constraint already has, so it costs one
bool and two linear constraints per pair and no new search structure.
`PLAN_PAGES_SWIM_XING` adds it (0 = off = the default; flag-off copper is
IDENTICAL to the pre-change chain on `copper_same` and `cmp_copper`).

### Every arm, and none of them ships

`chain_k.sh`, `PLAN_PAGES=1`, all 0 DRC:

| arm | K28 | K35 | K41 | total | open |
|---|---|---|---|---|---|
| **flat 100 (shipped)** | **34** | **62** | **79** | **175** | 0 |
| flat 4 (~the measured mean) | 36 | 54 | 86 | 176 | 0 |
| flat 2 | 42 | 64 | 80 | 186 | 0 |
| 0.8 + 0.44/crossing (the fit) | 44 | 64 | 97 | 205 | 0 |
| 1.6 + 0.88/crossing (2x the fit) | 44 | 60 | 78 | 182 | 0 |
| 100 + 1/crossing (**tie-break only**) | **30** | 62 | 91 | 183 | **2** |

**Nothing beats the shipped 175, so nothing ships** (edict 3). Two results in
that table are worth keeping anyway, and both were repeated because a single
K28 run on this bench is not a measurement:

* **The tie-break arm takes K28 to 30 vias, 0 open, 0 DRC -- below every
  recorded arm** (the reference `jcl`, the portfolio and `+replan.py` all sit
  at 34) and 16 below the human's 46. Repeated: **30 / 30 / 30**, against a
  base that measures **34 / 34 / 32 / 32**. So it is outside the base's own
  run-to-run band, not inside it.
* **And it breaks K41: 91 vias with 2 open (`SDQ10`, `SDQ13`), repeated
  91 / 91 / 91 against a base of 79 / 79 / 79.** Deterministic, not noise.

That arm is the one the analysis predicted: it does not change how MANY lanes
swim (the base stays at 100), only WHICH -- exactly the degeneracy `--judge`
says the objective cannot see. It moves the right rung by the right mechanism
and breaks another, which is what "the direction is right and the term is not
finished" looks like.

**One hypothesis is closed off.** A swimmer does not pay for itself by pushing
its neighbours out of band: across the arms, MORE swimmers measured BETTER
first-attempt in-band, not worse (4 swimmers -> 75%, 41 -> 89%, 77 -> 95%)
while routing more vias. Swimmers route fine; they just cost.

### A LOWER BOUND for K41 and K51, and why it does not grade them

`exact_dp` costs `2**(free lanes)`, so the rungs that matter have never had a
number to be compared against -- only each other. `channel_lower_bound` gives
one at any K, in milliseconds, and it is a bound rather than an estimate:

> Any routing splits the lanes into A (holds layer F for the whole channel),
> B (holds layer B) and S (the rest). A and B are each crossing-free, so each
> is an increasing subsequence; a lane in A costs exactly its end mismatches
> and so does one in B; and a lane in S changes layer at least once, so it
> costs at least 1 -- at least 2 when its two ends share a layer. The minimum
> of that sum over all valid (A, B) is below every routing's cost, and it is
> exactly `pages_model` with each lane's swim price set to its own floor,
> which the O(K^3) DP computes exactly.

It is checked two ways that cannot both be wrong the same way: with every end
on F it must land **exactly** on the LIS formula `2*(K - LIS)` that patience
sorting computes by a different algorithm, and it must never exceed
`exact_dp` -- including with the teeth moved, where the LIS formula stops
applying and the bound does not. 5/5 mutants killed.

On the bench (`fb_t2q_fresh`, U1 -> DU1, fanout vias added):

| K | nets | crossings | LIS | teeth on B | bound + fanout | routed |
|---|---|---|---|---|---|---|
| 28 | 28 | 195 | 7 | 3 | 44 | **34** |
| 35 | 35 | 302 | 9 | 4 | 54 | 62 |
| 41 | 41 | 399 | 9 | 6 | 66 | 79 |
| 51 | 48 | 539 | 11 | 8 | 76 | -- |

**K28 routes ten vias BELOW its own floor, and that is the finding.** The
bound holds for the channel-confined homotopy class -- every inverted pair
crossing once, no other pair crossing -- and **the bench does not stay in
it**: measured on the shipped boards, lanes with copper past the deepest bus
ball are **5 of 28, 8 of 35 and 8 of 41**. A fifth of the bus reaches its pad
through or around an array, which un-crosses pairs at no via cost at all.

So the table is not a scorecard, and 79 against 66 at K41 is **not** "13 vias
of room". What it is worth is the diagnosis it hands the harness, below.

### Where the room actually is: K41 is done, K51 is the whole gap

With a floor to compare against, the ladder reads completely differently:

| K | floor | ours (best recorded) | human | our room | human room |
|---|---|---|---|---|---|
| 28 | 44 | 34 | 46 | **-10** | +2 |
| 35 | 54 | 58 | 58 | +4 | +4 |
| 41 | 66 | **68** | 70 | **+2** | +4 |
| 51 | 76 | 96 | 81 | **+20** | +5 |

The human sits a consistent **+2 to +5** above the floor at every rung, which
is the sanity check the floor needed -- a hand layout should land just above a
rigorous bound. Against that:

* **K41 is essentially solved.** 68 against a floor of 66, and *closer to it
  than the human is*. Plan-side work at K41 is chasing at most two vias.
* **K51 carries the entire remaining opportunity** -- 20 vias against the
  human's 5, and the human proves 81 is reachable.
* K28's -10 is the escape again, not a win over physics.

### K51's twenty vias are FIFTEEN NETS

Per-net breakdown of a clean K51 baseline (116 vias, 0 open, 0 DRC):

```
 vias  nets  subtotal          9 nets pay 0   (they hold F end to end; LIS 11)
    6     5        30         23 nets pay 2   (the expected dive and return)
    5     1         5         ---------------------------------------------
    4     7        28         15 nets pay MORE than 2, carrying 69 vias,
    3     2         6            of which 39 are excess over 2.
    2    23        46
    0     9         0         if every net paid at most 2: 78 vias.
```

**78 is the floor (76) and the human (81).** So K51 is not a plan that is
diffusely bad -- it is fifteen nets that weave, and `pages_first`'s own
directive is *"no net may need more than two vias"*. `SCKE1` at K41 is the
same shape in miniature: 8 vias, detour only 1.29x, three extra dive-and-
return pairs -- not wandering, weaving.

### The mechanism built for those nets does nothing

`DST_RESIDUE` aims at exactly this population -- "the nets its schedule cannot
place and would leave to weave". Measured this session:

* **`DST_RESIDUE` defaults to 0**, so `DST_RESIDUE_CANDS` (README TODO item 2's
  lever) is dead on the default chain: 8, 12 and 16 all gave **116 vias and an
  identical 838 berth candidates** at K51. A vacuous null until the candidate
  count was checked.
* With `DST_RESIDUE=3` the machinery runs and **identifies the right nets** --
  16 residue nets at K51, against the 15 that carry the whole gap -- then
  builds 374 candidate berths / 788 moves / 11418 exclusions, and its solve
  returns **`objective n/a`, 0 moves, in about one second**. Every sweep.
* **Zero moves at K28, K35 AND K51**, and unchanged grades (32 / 62 / 116).
  Not a solver issue: `BRAID_ALT_SOLVER=cpsat` behaves identically to the
  default HiGHS. `alt_obj` is set at the *end* of the alternatives solve, so
  "n/a" means that solve never completed -- the silent-exhaustion class TODO
  item 12 already flags for `braid_tier`, on the one mechanism pointed at
  K51's actual defect.

**That is the lead to pick up**: not a knob, a mechanism that is switched off
by default and returns nothing when switched on.

### The residue mechanism: diagnosed, and the silence fixed

`DST_RESIDUE` aims at exactly the nets that carry K51's gap, and measured
this session it made **zero moves at K28, K35 and K51**, reporting `solve
objective n/a; 0 move(s)` on every sweep. Root-caused by reading the call
path, not by inference:

```
fanout_from_plan.residue_choice        builds the candidates, sets plan['alts']
  -> braid.plan_braid(..., plan)       the in-process judge
    -> braid.setup                     ctx.alts = plan['alts']          OK
    -> ...
       braid._profiles5                the ONLY caller of _alts5
                                       gated at line 5814:
                                         if ONE_DIVE >= 5 and sched is not None
```

**`BRAID_ONE_DIVE` defaults to 0**, and `_alts5` is the only thing that sets
`alt_obj` or returns a move. So the fanout builds 374 candidate berths, 788
moves and 11418 exclusions, calls the planner once a sweep, and hands the
answer to a consumer that is not running. Three defaults have to line up for
this mechanism to do anything -- `PLAN_PAGES=1`, `DST_RESIDUE>=2` **and**
`BRAID_ONE_DIVE=5` -- and only the first two are documented as the switch.

**Why it was silent, which is the part worth keeping.** The first fix put the
warning in `braid.setup`, where `ctx.alts` arrives -- and it printed nothing,
because the braid is reached through `plan_braid`, whose logger is
`lambda msg='': None`. A warning there goes nowhere. The refusal is therefore
in `residue_choice`, before any of the work:

```
residue choice: REFUSED -- DST_RESIDUE needs BRAID_ONE_DIVE=5 and it is 0.
The solve that chooses among the candidate berths (braid._alts5) runs only
inside the level-5 profile solve, so every sweep would report "objective
n/a; 0 move(s)" and change nothing.
```

Measured: the wasted sweeps go **2 -> 0** at K28 and the grade is unchanged
(34 vias), which is the proof that it really was doing nothing. `_profiles5`
also now says so when nets are offered but none is in its model -- the other
way this arrives at an empty answer.

**Refused, not auto-enabled**, for two measured reasons.

1. `ONE_DIVE=5` replaces the pages, the leg layers and the required
   stretches, so switching it on silently would change copper on every board
   that sets `DST_RESIDUE`. At K28 with the residue search OFF it routes
   **39 vias against the baseline's 34** -- the mechanism starts five vias
   behind before it chooses a single berth.
2. With the prerequisite satisfied it is **not affordable**. Every sweep of
   the residue search calls the planner, and each of those calls now runs the
   full level-5 profile MILP over the whole lane set. Measured: `ONE_DIVE=5
   DST_RESIDUE=3` at **K28** ran **34 minutes of CPU at 100% in the fanout
   stage alone and never finished it** (killed), against an edict budget of
   ~2 minutes for the whole of K41. The control -- the same `ONE_DIVE=5`
   with the residue search off -- completed in a couple of minutes, so the
   cost is the residue loop calling the level-5 solve once a sweep, not
   level 5 itself. K51 is strictly worse.

So the honest state of TODO item 2's mechanism: it was never running, it now
says so, and when it does run it is five vias behind and an order of
magnitude over budget. The berth choice is still the right lever for K51's
fifteen weaving nets -- `_alts5` is not the affordable way to pull it.

### What the synthetic bench can and cannot do at K41+

* **It cannot be a fast screening loop there.** A generated K41 case routes in
  ~40 minutes against the bench's own ~5, because the synthetic array is far
  more porous than a real BGA and the router searches the whole interior.
* **Its truth machinery is the instrument, and that transferred**: the floor,
  the crossing census and the escape count above are all `synth_bus` functions
  run on the real board.
* **The permutation family calibrates well.** The bench's bus IS statistically
  a uniform random permutation, and `shuffle` matches it on every quantity the
  planner consumes (K41: crossings 406 vs 399, LIS 10 vs 9, paged 18 vs 16,
  swimmers 23 vs 25). `b6` is that batch.
* **The geometry does not calibrate, yet.** `thru` on the bench is about a
  fifth; the harness's settings give 40% (open, lanes cut through) and 40%
  again (closed -- the lanes go *around* instead, which the `cal` batch shows
  in renders). `thru` conflates the two escapes and is non-monotone in ball
  diameter, so it is the wrong calibration target as it stands.
* **K51 needs a generator that can hit a target LIS.** Pinning the crossing
  count with `--inversions 539` reproduces the bench's crossings exactly and
  lands at **LIS 6.2 +- 0.6 against the bench's 11**, because the walk does not
  sample uniformly at a given inversion count.

### A measurement trap this work fell into, recorded

The first bench A/B graded **identically in both arms** at K28, K35 and K41 --
36 / 62 / 91 vias, same segment counts. The reason: **`chain_k.sh` does not set
`PLAN_PAGES`**, and `pages_first` defaults to 0, so both arms ran the OLD
planner and `PLAN_PAGES_SWIM` reached nothing. A vacuous pass that looked
exactly like a clean null result. `synth_ladder.run_chain` sets `PLAN_PAGES=1`
itself, which is why `b5` was never affected -- and why the two harnesses
disagreed about whether the knob did anything.

`planner_ran` now reads the FANOUT stage's log (where `pages_first` prints --
not the braid's, which was this function's own first bug, reporting 16 of 17
cases as planner-less on a batch where it had run every time) and the table
carries a `planner` column; a case with no `pages-first` line is NAMED in the
summary, with the caveat that an INFEASIBLE solve prints no such line either
(README TODO item 8).

## `rules.py` -- one place for the topo chain's design constants (agent, 2026-09-15)

Half of the generality debt recorded above is paid, and the other half was
DECLINED on purpose. `awx/rules.py` is now the single definition of the
chain's design constants; every module's constant defaults to it and every
stage installs from it.

**Andy's decision, and the reason.** The first version of this module
RESOLVED the numbers from the board -- the sibling `.kicad_pro` Default net
class, `.kicad_dru` layer rules, `list_nets.board_constraint`, the
`fab_tiers` floor. That was removed. **py_router already does that
resolution**, properly and in one place, and the topo chain is not a second
front for it: two resolvers reading one board are two chances to disagree
about what it asks for. The topo chain will be DRIVEN by the main router, and
when it is, the geometry will be SUPPLIED. So this module holds the chain's
own constants and offers exactly one seam for that handover --
`Rules.from_router_config(cfg)` -- which is unused by the chain today and
covered by a unit test only.

| quantity | the one source | value | formula |
|---|---|---|---|
| spec clearance | `rules.SPEC_CLEARANCE` -> `topo_strings.SPEC_CLEAR`, `braid.SPEC_CLEARANCE` | 0.1 | the chain's spec: what the fanout lays at, what `chain_k.sh` / `grade_k.py` grade at, what the output project records |
| braid hug clearance | `Rules.hug` -> `braid.CLEAR` | 0.105 | **= clearance + 5 um** -- the spec plus a hair, so a hug does not sit exactly on it |
| braid lane track | `rules.TRACK` -> `topo_strings.TRACK` | 0.127 | the lane track (5 mil) |
| fanout track / clearance | `Rules.fan_track` / `.fan_clear` -> `source_realize.FAN_TRACK` / `FAN_CLEAR` | 0.1 / 0.1 | the production engine's stub width; clearance **= the spec**. The board carries TWO track widths on purpose |
| via size / drill | `rules.VIA_SIZE` / `VIA_DRILL` -> `braid.VIA_SIZE` / `VIA_DRILL` | 0.25 / 0.15 | |
| lane slice | `Rules.lane_slice` -> `select_moves.NEST_IN`, `cut_ledger.NEED` | 0.232 | **= track + hug** -- two parallel tracks of width w at clearance c sit at pitch w + c |
| lane pitch / exit pitch | `Rules.lane_pitch` / `.exit_pitch` -> `braid.LPITCH` / `MINP`, `select_moves.BAND_LPITCH` | 0.35 / 0.38 | **= max(the chain's pitch, one lane's slice)** -- the floor binds above clearance 0.218 |
| band tip | `Rules.band_tip` -> `select_moves.BAND_TIP` | 0.9 | **= array pitch / 2 + the engine's exit margin** (DU1's 0.8 + 0.5). A DEAD default -- see below |
| `BAND_GAP`, `HALF_SEP`, `VIA_NEED`, `END_KEEP`, `MARGIN_OUT` | `Rules` properties | | braid's / topo_strings' own expressions, re-evaluated on install so the formula has one home |
| hole-to-hole / edge | `Rules.hole_to_hole` / `.edge_clearance` | None | the braid reads these two off the board itself and applies them tighten-only; that is left exactly where it was |

**How it is wired.** Each stage is its own process, so each entry point calls
`rules.install_defaults()` once: `braid.main`, `fanout_from_plan.main`,
`make_bench.main`, `pack_board.main`, `replan.main`, `cut_ledger.main`,
`collapse_dives` (after its argparse). `install` writes the values into the
module constants the chain already reads and re-evaluates the ones derived
from them. **The literals stay as each module's DEFAULT**, so a module
imported without an install behaves exactly as before -- the chain is
byte-identical BY CONSTRUCTION, not by measurement. Constants were chosen
over functions because the consumers read them as module ATTRIBUTES at call
time (`te.VIA_SIZE`, `br.CLEAR`) in ~30 places, so one install reaches all of
them and no hot loop grows a call. `chain_k.sh` and `grade_k.py` grade at the
module's clearance and PRINT it (`GRADE ... clr=0.1`), instead of each
carrying its own literal 0.1.

`install_defaults()` is inert today -- it installs what the modules already
hold -- and that is the point: it is the seam. When the main router drives
the chain, that call becomes `install(Rules.from_router_config(cfg))` and the
whole chain moves onto the router's geometry at once.

**`from_router_config` does not invent what a router config cannot say.** It
takes `clearance` / `track_width` / `via_size` / `via_drill` (plus
`hole_to_hole_clearance` / `board_edge_clearance` when present) and derives
the chain's quantities by the formulas above. Two things it refuses to guess:
the **two track widths** (a config has ONE `track_width`, so the supplied
width becomes both unless the caller passes `fan_track` -- the split is a
chain decision and stays an explicit argument rather than a silent ratio),
and **band_tip**, which is array geometry, not routing geometry. A config
carrying neither clearance nor track_width raises rather than falling back to
the chain defaults: a mis-wired handover must not look like a working one.

**Float bits are part of the contract.** `0.1 + 0.005` is one ULP ABOVE the
double `0.105`, and `0.127 + 0.105` one ULP BELOW `0.232`. A 1-ULP clearance
moves a grid cell, moves a lane, changes the via count. So every DERIVED
quantity is rounded to 6 decimals (the writeback's own normalization) and
lands exactly on the literal it replaced, while the expressions the modules
already spelled are re-evaluated in their original order and not rounded.
`tests/test_622_rules_of.py` compares with `.hex()`, not `approx`.

**The gate: flag-off byte-identity.** `PLAN_PAGES=1 bash chain_k.sh T 15 28`
before and after: K15 and K28, fanout board AND routed board, **IDENTICAL
copper** on `copper_same.py` and on `cmp_copper.py` (counters, so a
duplicated segment cannot hide), and the stamped `.kicad_pro` byte-identical.
Ladder unchanged: K15 16 vias, K28 34 vias, 0 open, 0 DRC.

**A bug this work introduced, and what caught it.** The first `install`
resolved its targets with `sys.modules.get(name)`. A stage runs as
`python3 braid.py`, so the router's own module is named `__main__` and
`sys.modules['braid']` does not exist: the install wrote NOTHING into the
router, silently, while the stage printed the numbers it was not using -- the
"a wiring fix can be INERT" trap exactly. `install` now also matches a module
running as `__main__` by its `__file__`, every stage prints **its own
constants** rather than the `Rules` object, and the test runs a stand-in
stage as a real subprocess. A second, quieter one: `braid.clip_round_ends`
captured `END_KEEP` as a DEFAULT ARGUMENT, bound at def time where a module
attribute write cannot reach it; it now reads it in the body, and the test
AST-scans every awx file for that shape.

**Found and NOT fixed (recorded, as the audit above records its own):**

- **`select_moves.BAND_TIP` is a DEAD default.** Its only readers
  (`band_leg`, `band_capacity`) are on the `SPLIT_BLOCKS=1` path, and that
  path's caller (`fanout_from_plan.plan_state`) already overwrites it with a
  different formula -- `max(pitch_x, pitch_y) / 2 + 0.05`, half a pitch plus
  one occupancy cell, because the under-pad engine ends its stubs at the
  boundary cell and not at `exit_margin` (measured 0.425 at 0.8 mm pitch).
  The 0.9 is reproduced exactly by the formula in its own comment, and is not
  what runs.
- **`select_moves.BAND_BLOCK_GAP = 0.30` calls itself "the braid's
  BAND_GAP", which is `TRACK + CLEAR + 0.07` = 0.302.** A stale hand-copy.
  Making it follow the formula changes the chain's output, so it is named
  here instead of changed.
- **The 0.025 routing grid in `braid.setup` is still a literal**, justified
  in its own comment by a rule computation ("the legal minimum, track +
  clearance = 0.227, plus 23 um") that is not true at another geometry -- so
  a supplied geometry would not move it.
- **Per-NET-CLASS clearance.** The braid prices ONE scalar for the whole bus,
  and whatever the main router supplies will be one number too, so a board
  whose bus class differs from its Default class needs a decision that does
  not exist yet on either side.
- (Observed while the resolver still read the board, and still true:
  `list_nets.board_constraint` cannot see `min_via_drill` -- it is not in
  `_CONSTRAINT_FIELDS` -- although this repo's own writeback writes that key
  and the bench carries it. That is py_router's to fix if it matters.)


## Settled -- do not re-run these

**`DST_RESIDUE>=2` (the joint berth choice) is a DEAD END, measured end to
end (2026-09-17).** It needs `BRAID_ONE_DIVE=5` to do anything at all (see
the refusal), and with that satisfied, at K28:

| | vias | wall clock |
|---|---|---|
| baseline | **34** | **75 s** |
| `ONE_DIVE=5` alone -- the prerequisite | 39 | 146 s |
| + `DST_RESIDUE=3`, CP-SAT | 40 | 551 s |
| + a pass budget, a no-good cut and a retry cap | 42 | 359 s |

The residue pass adds **+1 via over its own prerequisite**, which is itself
**+5 over baseline**, at 5-7x the wall clock -- on K28, the *fastest* rung,
against an edict budget of ~2 minutes for all of K41. The speedups were
built and they worked (confirmations 11 -> 2, passes 3 -> 1, 600 s -> 359 s)
and were then REVERTED, because correct code that makes a dead end faster is
still a dead end. What they measured is worth keeping:

* a judge at `ONE_DIVE=5` costs **~13 s** against **~0.2 s** at
  `ONE_DIVE=0` -- 65x, and it is the level-5 model CONSTRUCTION (5565
  samples, 907 proximity pairs), not the MILP, which is 2256 rows and
  0.1-0.3 s;
* `_alts5` exists to replace one-at-a-time trials with one joint solve, and
  its REJECTION path fell straight back to the trials it replaced -- ten
  full judges against one cheap solve, a trade that inverted when CP-SAT
  arrived and nobody re-checked;
* with HiGHS (the default `ALT_SOLVER`) the solve returns nothing at all,
  exactly as the solver study fifteen lines above it predicts.

**`DST_RESIDUE=1` is the arm that works, and it is the cheap one.** It uses
`residue_search`, needs no `ONE_DIVE`, and its judges are the 0.2 s kind: 41
judges in 7 s. Measured K35 **62 -> 60** (0 open, 0 DRC). It does NOT
transfer to K51 (116 -> 119) and the reason is the familiar one -- of its 6
accepted moves only 1 lowered the residue count; the other 5 were taken on
the judged cost, which `--judge` measures as rank-uninformative. That is
where a K51 attempt should start: the search is right, its acceptance rule
is not.


| arm | verdict |
|---|---|
| `BRAID_SOLVER=cpsat` (plain solves) | **never** -- K41 98. `BRAID_ALT_SOLVER=cpsat` (choice solves) is the good one |
| `BRAID_VIA_ROOM_REFUSED=2` | breaks K51 (3 open / 30 DRC) on the OLD 137-via baseline. **Re-measured 2026-09-13 on the 99-via `xing` board: bit-identical to the base -- INERT, not harmful, in that regime.** Read the verdict WITH its board; this one is congestion-dependent. Default 0 |
| `BRAID_ATTEMPTS=10` / `BRAID_BUDGET_X=8` | on K51's refused net: `ATTEMPTS=10` bit-identical, `BUDGET_X=8` gives 97 vias but **2 open** instead of 1. More search does not close a lane with no room; it moves which lane refuses |
| `BRAID_RIP_DEPTH=2` | K51 xing: 142 vias / SA11 open / SDQ9 silently broken, 259 s; with `BRAID_RIP_ONBOARD=1` **141 / 0 open** (complete, 173 s). Inert at K41 (no trial there loses a victim). Completion at +42 vias is not a line |
| `BRAID_RIP_ONBOARD=1` (NOT COMMITTED) | the phantom-victim fix. Correct locally (SA1 closes at 3 vias) and 97/2 against 99/1 on the board: the refusal moves to SA11+SDQ11, the same pair `BUDGET_X=8` evicts. Inert at K41 |
| `BRAID_RIP_SOFT=5` (NOT COMMITTED) | the refused lane routed with its victims' old copper priced: it detours instead (SA1 7 vias, SA8 1 -> 6); K51 116 / 2 open. Harmful |
| `BRAID_RESCUES=99` (NOT COMMITTED) | K41 91 -> 100, K51 99/1 -> 104/**5 open**; the base logs have ZERO successful rescues, so the cap of 3 is what keeps the x4 rescue from forcing 25 lanes through bad paths. A regulariser, keep it |
| `SEL_XLAYER=1` | crossings WORSE (K41 196 -> 301) -- but that arm ran through a `zip` desync pairing trial legs with stale nets, so the verdict is **not evidence**; unmeasured |
| `DST_XING` per-candidate | pairwise deltas are not additive; the pairwise MILP form fixes the bug and still beats nothing |
| `DST_XING_SCREEN` | helps nothing, alone or with the pattern seed |
| `SEL_CONTEND` / `DST_CONTEND` | fail in both placements, at every weight |
| `DST_WALK_OFF` (off-array walk) | built, general, loses |
| `SF_ACCEPT_MARGIN` | **neutral at 1, harmful at 2-3** (K41 88 -> 122 / 1 open), measured on the cloud 2026-09-13. The residue is not a completion guard; margin 0 stands |
| the per-net DP "floor" as a target | REFUTED 2026-09-12: "every lane is 0 or 2" is an artefact of a CIRCULAR instrument, and "bring every violator down to 2" is NOT the objective -- at K35 the joint optimum takes four lanes from 2 to 0 and pays ONE up to 4, which is 4 vias cheaper. The COMPARATIVE form survives (human one lane above 2, we six) |
| `PLAN_PAGES_GROUP_DST` (the destination re-berth) | worse at K35 by 8 and K41 by 1, better on none; also inert on the planted b4 cases. Kept as a change detector |
| `PLAN_PAGES_WALK_STAGE` | ties three rungs and opens two nets at K51. Kept as a change detector |
| `BRAID_PITCH_EXACT` (the exact lane-pitch projection) | CORRECT and costs 8 vias at K41 (88 against 80, both DRC-clean) -- `pair_floor` is a planning heuristic, not a clearance rule, so the under-relaxed comb was never illegal. Shipped OFF |
| optimising the SWIMMER count | anti-correlated with vias on 3 of 4 benches; a swimmer costs ~1 via |
| `DST_SEED_ORDER` = `perp` / `arc` / `win` / `dp` | one global order round the array loses at every bench; the face carries topology AND partitions the monotone run |
| `DST_ASK_BAN=1` | an ask the engine answered a layer/kind away is not repeated: INERT on the pattern arm (18 bans, identical copper), harmful on the baseline (K35 58 -> 66; K51 0 open -> 5 open + 23 DRC) |
| `SRC_REPLAN=1` | names a better tooth, the one-net re-fan cannot lay it; superseded by `SRC_REFAN_JOINT` |
| `BRAID_EXACT_PAGES=1` | the two pages by exact MILP instead of the LIS greedy: never won a chain |
| `SF_LIS_GUARD=1` | protects the crossing-free chain: **+19 vias at K41** (107 vs 88), +4 K35, and 3 open at K51. Page assignment is ALREADY optimal -- 3 of 5 plans sit exactly on the Greene bound lambda_1+lambda_2, so this guards a solved sub-problem |
| `SF_LDS_W` (crossing depth) | wired and verified (f shifts by exactly 4x11 at weight 4) and INERT at every weight on K35/K41; at K51 2.0 and 4.0 give 115 vias but 2 OPEN. LDS is an integer 6-11 that rarely differs between neighbouring plans, so it adds a near-constant and reorders nothing |
| `SWIM_CHANGES=1` | the braid's own per-swimmer change count. Inert at K35/K41 (most swimmers there really do need 2 changes, which is what the flat price assumed); K51 124v/0open vs base 129v/1open. Superseded: it could only ever move the SECOND element of the judge's key |
| `SF_SWIM_MODEL=1` | the same count computed in the PLANNER (verified to emit mean 3.4-4.8 vias/swimmer against the flat 2.0, spread 0-7). K41 88=88, K51 129=129. Inert for the same reason |
| `BRAID_W_PER_NET=1` | per-net swim cost in the berth-choice MILP. On an ISOLATED braid over a fixed board it is 65 -> 63 vias; **in the chain it is K41 88 -> 124 with an open net**, K51 115v/2open. The isolated test exercises the profile solve only -- in the chain the same cost also drives `_alts5`, the berth CHOICE |
| `BRAID_KEEP_SCHED_PAGES=1` (NOT COMMITTED) | stop `_profiles5` rewriting pages from the CP-SAT vector (the cross-arch divergence runs through that line). **Costs 12-16 vias on EVERY bench** (K35 62->74, K41 88->104, K51 105->117). The page rewrite is load-bearing: the page is meant to BE the layer the lane ends up on, and freezing it leaves swimmer accounting and the exit blocks reading a page that disagrees with the copper |
| `BRAID_CPSAT_SCALE=1000000` | the via tie-break is `1.0 + 1e-4*u`; at the default 10000 that rounds 20 slot positions to 2 integers, at 1e6 it keeps 20. Neutral at K41, **costs 8 vias at K35 (62->70) and 22 at K51 (105->127)**. The coarse rounding was acting as a REGULARISER: collapsing near-equal slot choices beats letting the solver chase 1e-4 differences that do not survive into the routed board |
| `BRAID_CPSAT_CONFLICTS` / `BRAID_CPSAT_CONVERGE` | count-based and convergence-based budgets, built to replace the non-portable `max_deterministic_time`. **Unusable: a 40 s solve had not finished in 31 MINUTES.** A conflict count bounds the SEARCH, not PRESOLVE, and setting one means no time budget is set at all; the convergence loop repeats presolve every round |
| `SWIM_CHANGES=1` | a swimmer priced by the braid's implied changes. **MEASURED 2026-09-13**: it needs `SF_SWIM_MODEL=1` too (on the judge's path `bp['swim_changes']` is empty, so alone it silently falls back to the constant). Together: bit-identical at K41 and K51, and on the cloud K44 bench **122/1-open against 98/0**. See the per-swimmer row below |
| `SWIM_PRICE` (the ONE constant) | **cannot re-rank at equal swimmer count, by construction.** A uniform price shifts every candidate's `f` by the same amount whenever two candidates swim the same NUMBER of lanes, so it only re-ranks candidates whose counts differ. Measured at K44 locally: `SWIM_PRICE=3` on top of `SF_KEY_COST=1` is **bit-identical** to base (same 2154 segs, same 126 vias), while `SF_SWIM_MODEL=1 SWIM_CHANGES=1` -- which prices WHICH nets swim, not how many -- moves the board. This qualifies `prices.py`'s standing "calibrate the ONE number on the ladder" instruction: the constant is the right model of the BIAS (swimmers really cost mean 3.15 vias at K51 vs the flat 2.0, and that -23 is exactly the K51 predicted-89/actual-112 gap) but the wrong instrument for the judge's actual job, which is ORDERING |
| `SPLIT_BLOCKS=1` | **verdict VOID** -- it raised NameError on the first band exit (NEST_IN/NEST_STEP/BAND_LPITCH undefined) so it was never measured. Fixed 0912; unmeasured |
| the wave schedule, `refine_sides`, the source chooser | reverted (the wave never existed on take5; it is in the bundle) |
| `BRAID_LAY_ORDER=xing` | **the largest single-knob win measured, and slack-dependent**: K51 112->99, K41 91->97. Not a default -- an arm, like the pattern seed |
| `SF_ESC_W` (0, 0.25, 0.5, 0.75) | all bit-identical at K51; `=10` gives 142/3-open, so the knob is LIVE and the nulls are real. The escape term does not discriminate between K51's candidates |
| `SRC_REFAN_JOINT=1` | bit-identical at K51 -- the recorded "only pays at K51" does not survive a chain run |
| `replan.py --mode=rebraid` | inert at K41 and K51; `incremental` wins or ties. The candidates it was meant to unlock (logged "unjudged ... would need the full braid") did not materialise into a better board |
| `SF_SWIM_MODEL=1 SWIM_CHANGES=1` on the CLOUD (NOT COMMITTED) | the per-lane model, tested on the benches where these effects live. **K44**: with `SF_KEY_COST` it recovers 6 of the 14 (112->**106**) but never reaches the 98 baseline; WITHOUT it, **122 / 1 open against 98 / 0**. **K51**: **105 -> 131**, a 26-via loss against `SF_KEY_COST` alone. Bit-identical at K41/K51 locally. So it is not merely inert -- on net it is HARMFUL, exactly as the predictability measurement predicts: you cannot rank by an estimate whose correlation with the truth is -0.03 |
| a per-swimmer via MODEL of any kind | **do not build another one without a plan-time feature correlating above \|r\|=0.2 with routed vias.** Four families tested null over 97-98 swimmers / 5 boards: crossings, a learned per-net prior, geometry, and ROOM. `room_probe.py` is the instrument |
| `SF_KEY_COST=1` (NOT COMMITTED) | the residue judge returned `k = (len(res), f)`, so swimmer COUNT was the PRIMARY key and the via cost `f` only a tie-break -- it minimised swimmers, not vias. Keying on `(0, f)` instead is a **big win at the top and a real loss in the middle**: K28 38->36, K35 70->62, K47 120/4open->114/0open, K51 129/1open->**105/0open**, K15 and K41 unchanged -- but **K44 98->112**. The K44 regression is NOT bench noise: 3 identical containers per arm gave spread 0 on both (98,98,98 vs 112,112,112). Not clearly better across the ladder, so NOT committed. The finding that stands is the DIAGNOSIS -- the judge was never pricing vias -- and `SF_KEY_COST` is the instrument that proved it |
| `group_pages`, `plan_nest`, `improve_k`, `channel_shift` | dropped with take4 (bundle: `~/Downloads/bus/bus622-take4.bundle`) |

Opt-in and kept, all present here: `replan.py` (the route as the judge),
`DST_WALK=k`, `SRC_CLIMB=k`, `pack.py`. The re-berth backstop was a take4
file and is in the bundle only.

## Recent findings (2026-09-16 / 17)

**The plan objective is ANTI-CORRELATED with the route, proven at the
optimum.** At `PLAN_PAGES_DET=5000`, K41's plan solves to PROVEN OPTIMAL
with a 24% better objective (2308.9 -> 1759.5) and routes **twelve vias
worse** (79 -> 91); K51 takes a 33% better objective and opens two nets.
K35 is unchanged. So there is no "more search would have helped" left to
say, and every search-side idea is pushing an objective that points the
wrong way at the rungs that matter. It also means `PLAN_PAGES_DET=40` is
not a budget but an accidental REGULARIZER -- which is why the ladder's
numbers are not portable. **Fixing the judge is item 1 of the TODO.**

**The braid-tier judge is neutral, not a win.** `PLAN_PAGES_TIER` routes
candidate plans and decides on `(open, vias)`. It was judging a board
nobody would build -- `braid_tier` fanned out through `fanout_once`,
which never writes the `.plan.json` sidecar, so the braid fell back to
`plan = None` (marker off, `EXACT_PAGES` off, `SIDERS` off), a third
regime this chain measures 14 vias apart from the shipped ones at K51.
Fixed. Re-measured against the correct (portfolio) baseline it is three
ties and one via worse at K35.

**Cloud vs laptop, bisected to ONE CP-SAT solve.** With `modal_k` fixed
the two agree on everything graded (34 vias, 0 open, 0 DRC, identical
per-net via counts) and differ only in segment count, 783 against 1144.
It is NOT the router binary: braiding the CLOUD's fanout board ON THE
LAPTOP reproduces the cloud to the digit, so braid + `grid_router` +
smoother are a pure, platform-independent function of the fanout board.
It enters at the first solve, which is feasible and not proven: same
instance and objective 727.2, different bound and different feasible
point, leaving one different exclusion so both sides prove `OPTIMAL
702.5` on models that are not the same. **Raising the budget does not fix
it** -- at det 640 both PROVE optimal and still disagree, because the
optimum is degenerate.

**`PLAN_PAGES_CANON` -- reproducible across machines, and not shippable.**
All three of `num_workers=1`, an integer `max_number_of_conflicts` stop,
and `linearization_level=0` are required (the LP is the one most easily
missed: with one worker and a conflict budget the platforms still
diverged on a float bound). With all three, local and Modal are
bit-identical. Cost: free at K35, +6 at K28, +10 at K41, and it stops
completing at K51. **Use it as an INSTRUMENT** -- the one configuration
in which a cloud number and a laptop number are the same measurement --
but note it can be inert where production is not, so it cannot be the
only instrument.

**Rejected, with numbers** (the flags stay as change detectors):
`PLAN_PAGES_GROUP_DST` (item 2) is worse at K35 by 8 and K41 by 1, better
on none. `PLAN_PAGES_WALK_STAGE` (item 3) ties three rungs and opens two
nets at K51. The `b4` planted-case harness reports the group machinery
INERT -- measured inert, not unreached: groups of 18 and 30 members form
and 14 members were re-berthed, with identical copper on 16 of 16 cases.

**The five-way code review (2026-09-17)** found and fixed: a back-side
BGA ignoring the layer NAMES it was given (gated by
`tests/test_flip_frame_layer_args.py`); `exact_lane` using an exemption
CELL SET as a layer index, so a safety check never ran; `move_sig`
omitting the legs, collapsing 28% of enumerated moves into shared
identities; `BRAID_EXACT_PAGES` unreadable on the planner side and never
reset; and four measurement holes (`grade_k` not asserting
`check_connected` ran, `modal_k` reading its grade off stderr, and two
regressions from the portfolio default flip). **`move_sig` +
`exact_lane` measured -3 at K41 and -2 at K51, canary-matched -- an
edict-3 pass.**

**And one that did not land, which is the lesson worth keeping.**
`_relax_pitch` never converges (60 sweeps against the 278-3229 a real
comb needs, leaving lane slots up to 0.33 mm tighter than their floor).
The exact projection is one O(n) PAVA pass agreeing with the converged
loop to 1.7e-08 -- and on ONE FIXED fanout board it gives **88 vias
against 80, both DRC-clean**. `pair_floor` is a PLANNING heuristic, not a
clearance rule, so the under-relaxed comb was never illegal: the cap was
relaxing an over-conservative heuristic and buying eight vias. Shipped
OFF behind `BRAID_PITCH_EXACT`. **Correct is not the same as better.**

## TODO

Ordered, highest value first. An item leaves this list only when it is
**done** or **abandoned with a measurement** -- the ones that ended that
way are in "Settled -- do not re-run these" and in the history.

1. **Fix the judge.** Solving the plan to proven optimality makes the
   board WORSE (K41 -24% objective for +12 vias; K51 -33% for two open
   nets), so the objective the planner optimises is not the thing worth
   optimising and more search on it is actively harmful. The braid-tier
   judge -- route the candidates, decide on `(open, vias)` -- is now
   correct and measures neutral, so judging by routing is not
   automatically the answer either. The one mechanism that has paid is
   `replan.py`, which evaluates per net against the FINAL board with the
   router as an oracle; widening it is the obvious next experiment, since
   the recorded K35 46 came from a much wider sweep than the
   `--rounds=4 --probes=2` used lately. Hours, so it wants the cloud.
   `BRAID_CPSAT_REPAIR` (hint repair) is built and still unmeasured.

   **2026-09-17 -- WHERE the objective is wrong is now measured, and it is
   the SWIM TERM** (see "The harness grades the JUDGE"). Three findings,
   in order of how much they constrain what comes next:

   * The objective is **rank-uninformative** about the routed via count
     (mean Spearman -0.05 at the shipped weights over 60 known-answer
     cases) and its optimum is **DEGENERATE**: inside a fixed swimmer
     count it spans 6 units over plans whose real costs span 28 vias, and
     on 9 of 36 groups it takes ONE value across a spread of up to 12
     vias. Neither is reachable by more search -- both are properties of
     the objective, which is why proving K41 optimal moved the board the
     wrong way and why `PLAN_PAGES_CANON` treats a symptom.
   * **The flat swim price is the term at fault, and it has a measured
     right answer that is not a constant**: the braid's own report over
     262 swimmers gives `changes ~ 0.44 * page_crossings + 0.79`, r =
     0.83, against a flat mean of 4.98 and a charged 100.
   * **Per-crossing pricing is built (`PLAN_PAGES_SWIM_XING`, default off,
     flag-off copper identical) and does not ship**: nothing beats the
     shipped ladder total of 175. But the tie-break arm (base 100, +1 a
     crossing) takes **K28 to 30 vias, 0 open, below every recorded arm's
     34**, reproducibly (30/30/30 against a base band of 34/34/32/32) --
     and breaks K41 (91 with 2 open, 91/91/91). The mechanism is right and
     the term is not finished. The next arm is the one that does not exist
     yet: a per-crossing price **gated on slack** (item 4's shape), or the
     crossing count taken against the ACTUAL corridor rather than all
     pairs.

2. **The berth menu. `DST_RESIDUE` is DIAGNOSED and the silence is fixed;
   the mechanism itself is not affordable (2026-09-17).** Root cause: its
   answer is consumed by `braid._alts5`, which runs only inside
   `_profiles5`, gated at `ONE_DIVE >= 5` -- and `BRAID_ONE_DIVE` defaults
   to **0**. So three defaults must line up (`PLAN_PAGES=1`,
   `DST_RESIDUE>=2`, `BRAID_ONE_DIVE=5`) and only two are documented as the
   switch; with the third missing the fanout built 374 candidate berths and
   11418 exclusions a sweep and handed them to nobody, reporting `objective
   n/a; 0 move(s)` -- zero moves at K28, K35 and K51, on both solvers. It is
   now REFUSED up front with the reason (sweeps 2 -> 0 at K28, grade
   unchanged, which is the proof it was inert). `DST_RESIDUE_CANDS` was
   dead for the same reason: 8 / 12 / 16 gave an identical 838 berth
   candidates.

   **And with the prerequisite satisfied it still is not the answer**:
   `ONE_DIVE=5` alone routes 39 at K28 against the baseline's 34, and
   `ONE_DIVE=5 DST_RESIDUE=3` burned 34 minutes of CPU on K28's fanout
   stage without finishing it (the control finished in minutes, so the cost
   is the residue loop calling the level-5 MILP once a sweep). The berth
   choice is still the right lever for K51's fifteen weaving nets; `_alts5`
   is not an affordable way to pull it.

   **The berth menu is the binding constraint, and the fix is ROW pruning
   -- not column generation.** Measured on the joint arm's K41 instance:
   each net has ~19 distinct berth geometries across ~4 faces, and at
   `CANDS=4` the one-per-face guarantee consumes all four slots for 28 of
   41 nets, so the `vias + ride` ranking never chooses WHICH berth, only
   one representative per face. This is the most concrete plan-side lever
   on the list and it feeds item 1 directly.

3. **K51 is now the ONLY rung with room, and it is 20 vias (2026-09-17).**
   Against a rigorous floor (`channel_lower_bound`): K41 is 68 against 66 and
   is *closer to the floor than the human* -- essentially solved, worth at
   most two vias. K51 is 96 against 76 where the human is 81. And the gap is
   **fifteen nets**: on a clean K51 baseline 23 nets pay the expected 2 and 9
   pay 0, while 15 pay more, carrying 39 vias of excess. At 2 vias a net the
   board would be 78 -- the floor, and the human. `pages_first`'s own
   directive is "no net may need more than two vias"; K51 breaks it on 15 of
   48 nets, and item 2's `DST_RESIDUE` is the mechanism aimed at exactly
   those.

4. **Slack-gated arms.** Two knobs are the largest single-knob wins
   measured and both LOSE where there is capacity to spare, in the same
   shape: `BRAID_LAY_ORDER=xing` (K51 -13, K41 +6) and the pattern seed
   (K35 69 against 58, K41 88 against 80). Either gate them on slack or
   find what they give up when they are not needed. A working slack test
   would settle both at once.

5. **The `.kicad_dru` is read with real layer names inside the turned
   frame.** The unfixed half of the back-side layer bug: a per-layer
   clearance rule lands on the opposite face for a back-side part, and
   per CLAUDE.md the dru outranks `--clearance` on every routing step.
   Shipped `py_router` code, so it blocks the merge to main.

6. **The corpus A/B for the production changes, then the PR to main.**
   Everything in "What this adds to `py_router`" changes copper on boards
   that have nothing to do with this chain, and a change that moves
   copper but is not on the list is the one nobody thinks to measure. The
   list now also carries this session's work: the back-side layer-name
   fix, `exact_lane`, and `KICAD_SEG_DIST_EXACT`, which ships ON by
   default and whose own comment says the A/B is owed.

7. **Generality -- the router is tuned on ONE bench.** zynq_ad9364 at K28
   is 0 open / 55 vias but only 17 of 28 in band, and the leg rules were
   tuned on `fb_t2q_fresh`; `flow_frame.py turn` needs fixing first,
   because that article does not run through `chain_k.sh` today. Poses
   off the axes (R30, R45) break the plan's compass faces and need a
   trigonometric turn of the file. Both matter more than their position
   here suggests if any of this is to be general.

8. **Audit `modal_k`'s `KEEP`.** Only about two of ~40 distinct
   `pages-first:` log shapes survive the filter, and an INFEASIBLE solve
   returns no `pages-first` line at all -- which reads exactly like "the
   planner never ran". Three separate misreads have come from this (the
   canary, the smoother, the tier judge). One audit pass against what the
   chain actually prints, not another line-by-line patch.

9. **`pick_braid` ignores DRC.** It judges `(open, vias)` only, so now
   that the portfolio is default-on it can ship a board with violations
   over a clean one with a via more, silently. Every other selector in
   the chain treats a DRC-dirty board as unshippable.

10. **Merge main and re-baseline.** The branch is ~123 commits diverged
    and predates `#958`, `#441` and `#521`/`#906`, so the whole recorded
    ladder sits on older engine code. It also has no
    `tests/stress/modal_suite/run_all_modal.py`, so the suite is
    local-only (~48 min) until the merge.

11. **Built, default off, never finished.** `DST_ASK_BAN` (an ask the
    engine answered a layer/kind/face away is not asked again) had one
    negative reading at K35 and the ladder was never finished.
    `BRAID_PACK=1` ships 0 open / 0 DRC with vias unchanged and far fewer
    segments, and nobody has measured what those segments buy.
    `SRC_EXCHANGE=1` is a probe whose first K15 reading found 2 of 60
    pairs improving the judge -- it was built to decide whether a source
    re-fan is worth writing, and the question is still open. Note the
    SOURCE ITSELF is no longer frozen in the default chain: the portfolio
    fans out both with and without `SRC_REFAN_JOINT`, and that arm is
    half of the K51 98.

12. **The remaining review findings**, none verified beyond the
    reviewer's evidence: `dedupe_boards` fingerprints copper but not the
    sidecar, so two copper-identical fanouts with different plans lose
    one; `braid_tier`'s budget exhaustion is silent and its `_near` test
    compares different key scales under bare `PLAN_PAGES`;
    `_realize_group_first` bans nothing on a pure DRC rejection;
    `blockers_of` double-counts half a track; `collapse_dives` calls
    `os.chdir` at import and keys nodes at 10 um; several planner items
    (a wasted seed solve, a silently unenforced fixed berth, the walk
    budget going negative). `flip_frame` also does not mirror
    `pad.polygons`.

13. **Tooling.** Still owed: the flag-off parity gate that a hand check
    does today. Promote session probes into `awx/` with a line each here
    when they earn it -- `room_probe.py` and `modal_k`'s
    `return_board`/`return_files` came in that way.

14. **Three pre-existing suite failures**, confirmed to predate this
    session: `test_703_predictor_regen`,
    `test_782_nondefault_netclass_clamp`, `test_fanout_cancel`, plus
    `test_459_group_routing` hitting its own 1200 s budget. Nobody has
    diagnosed them.

## History, in brief

The detail is in git; this is the shape of it, and the findings that are
still load-bearing. Sessions are numbered as they were logged.

* **Early (take-2 through take-4).** The braid reached human parity at
  K4-K8 and a clean chain to K21. The old-engine replan TEACHER was
  measured HARMFUL. Take-3 beat nothing and was restarted from the braid.
* **2026-09-12, the floor audit.** "Every lane's DP floor is 0 or 2" was
  REFUTED -- an artefact of a CIRCULAR per-net instrument, and acting on
  it would have walked the optimiser away from the optimum. What survived
  is the comparative form under a non-circular joint floor: the human
  carries ONE lane above 2, we carry six.
* **2026-09-13.** The judge has no resolution on vias (K44 cloud dumps).
  `replan.py` verified end to end. K51's last open net traced to the
  VICTIM RE-LAY and a capacity wall, not to the net itself. The human
  routes the congested region as bundles by (arc, layer). The
  **pages-first planner** (`PLAN_PAGES=1`) was written that night and is
  the planner still in use.
* **2026-09-14, arcs ABANDONED.** Flag-off byte-identical; arcs either
  lose or route open. Source and destination climbs help K28/K35 and hurt
  K41/K51.
* **Sessions 8-10.** The batch realize-and-reject loop; the mid-corridor
  page change measured INERT on fixed ends; THE PLAN's items 1-3 built.
  The judge switched to count+lane (`jcl` = 34/60/80/115), which is still
  the reference arm. First recorded as a slogan here: **more solve time =
  better plan = WORSE route** -- since proven at the optimum.
* **Session 11.** K51 read at the TRACK level: every first-pass refusal is
  a launch-side fan-in wall, and the braid side is exhausted.
* **Session 12.** The island price settled; the end-of-face climb built;
  `cew5d` = 54 / 71 / 109.
* **Session 13.** The two-level PORTFOLIO (fan out both ways, braid each
  both ways, keep the better by `(open, vias)`) plus `replan.py` on top
  gave **34 / 58 / 68 / 96**, two all-time records. The K51 98 is the
  joint re-fan plus the braid portfolio and **earns nothing from the group
  climb**, which was proposed, laid exactly, and rejected by the judge.
  The plan sidecar's one-word `pages_first` marker is worth up to 14 vias
  IN EITHER DIRECTION depending on the board, which is why the answer is
  a portfolio and not a default.
* **2026-09-16/17.** See "Recent findings" above: the cloud/laptop
  bisect, `PLAN_PAGES_CANON`, the anti-correlation proof, items 2 and 3
  rejected, and the five-way code review.


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

`jcl` is the judge every hand-built model lost to: the braid's own count
plus lane length (`PLAN_JUDGE=count PLAN_JUDGE_LEN=lane`).

**K51's 15 vias are TEN NETS, and the obvious fix is refuted.** Those nets
pay four vias where the human pays two, but capping every net at two is
INFEASIBLE over our own copper and costs MORE where it is possible -- the
double-divers are load-bearing. See *K51's gap: what it is, and what it is
NOT*, which is where the current work stands.

**A number is meaningless without its arm, and cloud numbers are a
DIFFERENT MEASUREMENT from local ones.** The two platforms stop the first
CP-SAT solve at different feasible points, so they start from different
plans; the same configuration that gives 98 at K51 locally gives 115 on
Modal. Compare cloud to cloud, local to local, and always check the
canary (objective AND bound) before reading two runs against each other.
Worse, the canary is not even stable run-to-run on one idle machine: the
same model gave 727.2 / 729.7 / 730.2 across six K28 runs. The only
configuration that escapes this is `PLAN_PAGES_CANON=1` -- a canonicalised
solve that gives the same answer on any machine. It is a measurement
instrument, not a production setting: it costs vias.

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
| **`joint_floor.py`** | **the NON-circular floor: one MILP over the fixed paths that picks every path's layer at every crossing at once. Validated against the audit's independent parity+max-cut on three boards. `JOINT_FLOOR_NODES` bounds it -- no clock. `--cap N` adds a per-net via bound, so "no net over two vias" is ASKED rather than assumed: INFEASIBLE means no layer assignment over these paths can do it, and the only fix is a different plan** |
| `room_probe.py` | **does a plan-time feature predict a swimmer's vias?** Takes FANOUT/ROUTED board pairs, rebuilds the crossing geometry the judge sees (`braid.plan_braid`) and correlates candidate features against the routed count. Every family tried is null (|r| <= 0.13 over 97 swimmers) -- run it before building any new per-lane cost term |
| `modal_k.py` | cloud arms. `return_board: true` returns the routed board, `return_files: [globs]` any tmp/ artifact (plan sidecar, raw logs, judge dumps) -- without these a cloud-only phenomenon cannot be diagnosed at all, and K44's regression is cloud-only |
| `human_at_k.py` | the human's vias for a coherent K set. **Mind the label**: `coherent_nets(51)` returns 48 nets, so "K51" is a 48-net problem -- the human is 81 over those 48 and 85 over the full 51. Both are right; ours route 48 |
| `census_vs_human.py` | per-net vias/copper/layers against the human, and where each via sits |
| `collapse_dives.py` | collapse short dives on a routed board (2 vias each) |
| `cut_ledger.py` | Maley cut capacity of a plan, before any lane is routed |
| `synth_bus.cap_floor` / `--cap-survey` | the same per-lane cap on a GENERATED channel, plus the sweep that shows the two-via directive dying with K. Uncapped it must equal `exact_dp` (different algorithm, same model) and the self-test checks that on 90 cases |
| `synth_bus.escape_search` | the same move steered by `cap_floor` ITSELF, with `start=` to seed it from the proxy. On the bench the HYBRID wins or ties 8 of 8 (-42% floor) and the floor-alone arm SATURATES (46/46/46 at 120/240/480 calls) -- so the proxy earns its place structurally, not on compute |
| `synth_bus.escape_move_floor` / `--escape-survey` | the ESCAPE move priced: each lane may shift `reach` slots at either end, both ends staying a permutation. Lowers the TRUE floor by a third at reach 3 and restores two-via feasibility -- but maximising its own objective (the free-rider ceiling) can make the true floor WORSE, which the survey flags. Steer by `cap_floor` |
| `synth_bus.cap_sat_feasible` | the cap as SATISFIABILITY (CP-SAT), for when only the answer matters. Proves the real K51 channel infeasible at two vias a lane in ~1 min where the MILP ran 30+ and was killed. `UNKNOWN` is a budget, never a negative; budgeted in DETERMINISTIC time |
| `wall_probe.py`, `copper_same.py`, `cmp_copper.py` | track-level wall census, set-compare copper |
| `make_bench.py`, `rotate_board.py`, `mirror_board.py`, `bend_bench.py`, `channel_bench.py` | build an article from any board, and its poses |
| `pose_gate.sh` | the chain over FF / BF / FB / BB / R90 / R180 / R270 |
| `modal_k.py` | fan a sweep onto Modal, one container per (arm, K); `memory=(4096, 12288)` -- the request is the bill, the limit is the safety net |
| `arms.*.json` | the sweeps: `arms.judge.json` (the comparator: `SF_ESC_W`, `SF_ACCEPT_MARGIN`), `arms.prune.json` (`BRAID_L5_ALT_PRUNE` x cap), `arms.next.json` (everything else untested), `arms.solver.json` (the older matrix) |

## One source for every routing number (`rules.py`)

`awx/rules.py` is the single definition of the chain's design constants;
every module's constant defaults to it and every stage installs from it.
This exists because a swimmer was once priced five different ways, and
`collapse_dives` re-laid ripped copper at the wrong track AND the wrong via
size.

**It does NOT resolve numbers from the board, on purpose.** The first
version read the sibling `.kicad_pro` class, `.kicad_dru` rules and the
`fab_tiers` floor; that was removed. **py_router already does that
resolution**, properly and in one place, and two resolvers reading one
board are two chances to disagree. The topo chain will be DRIVEN by the
main router and the geometry SUPPLIED; `Rules.from_router_config(cfg)` is
the one seam for that handover (unused today, unit-tested only).

| quantity | the one source | value | formula |
|---|---|---|---|
| spec clearance | `rules.SPEC_CLEARANCE` -> `topo_strings.SPEC_CLEAR` | 0.1 | what the fanout lays at, what `grade_k.py` grades at, what the output project records |
| braid hug clearance | `Rules.hug` -> `braid.CLEAR` | 0.105 | **= clearance + 5 um**, so a hug does not sit exactly on the spec |
| braid lane track | `rules.TRACK` -> `topo_strings.TRACK` | 0.127 | the lane track (5 mil) |
| fanout track / clearance | `Rules.fan_track` / `.fan_clear` | 0.1 / 0.1 | the production engine's stub width. **The board carries TWO track widths on purpose** -- a tool that rips a braid lane must re-lay it with the BRAID's numbers |
| via size / drill | `rules.VIA_SIZE` / `VIA_DRILL` | 0.25 / 0.15 | |
| lane slice | `Rules.lane_slice` | 0.232 | **= track + hug** |
| lane pitch / exit pitch | `Rules.lane_pitch` / `.exit_pitch` | 0.35 / 0.38 | **= max(the chain's pitch, one lane's slice)**; the floor binds above clearance 0.218 |
| band tip | `Rules.band_tip` | 0.9 | **= array pitch / 2 + the engine's exit margin** |
| swimmer price | `prices.SWIM` | | one number, five sites |
| hole-to-hole / edge | `Rules.hole_to_hole` / `.edge_clearance` | None | read off the board itself, applied tighten-only |

**Wiring.** Each stage is its own process, so each entry point calls
`rules.install_defaults()` once (`braid.main`, `fanout_from_plan.main`,
`make_bench.main`, `pack_board.main`, `replan.main`, `cut_ledger.main`,
`collapse_dives`). The literals stay as each module's DEFAULT, so a module
imported without an install behaves exactly as before -- the chain is
byte-identical BY CONSTRUCTION, not by measurement. Constants rather than
functions because consumers read them as module ATTRIBUTES at call time in
~30 places.

**`braid.CLEAR` (0.105) and `topo_strings.SPEC_CLEAR` (0.1) are different
quantities** -- the spec, and the spec plus 5 um. They were both called
`CLEAR` until this was audited; do not import one where the other is meant.
Names that LOOK shared and are not: `TOL`, `STEP`, `MARGIN`, `CAP`,
`PROX_TRACK`, `HW_COL`. Leave them alone.

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

**Never more than two vias on any net** -- at K51, 40 nets at exactly two,
7 at zero, one at one, none above. That is the whole of its advantage over
us (96 against 81); see *K51's gap* below for why it is not a target we
can simply adopt.

**But NOT "two vias at the ends", which this section used to claim.**
`census_vs_human.py` classes each via as source-end, destination-end or
mid-field, and on the human's 41 two-via nets the dominant pattern is
**`DM` (21 of 41)** -- one via at the destination, one in the FIELD --
with a mid-field via in **32 of 41**. `DS` and `DD` together are 8. That
is exactly the signature the F-block law predicts: a lane that holds F at
both pads and dives ONCE in the middle, where the lanes it crosses are on
F. The old reading ("`DS` or `DD` on nearly every net") was contradicted
by the tool it cited.

We beat the human on **7** nets (zero vias, straight across on F) and lose
on **14**; the difference is 15.

Its berth order gives 68 crossings at K41 against our 250 -- but that is a
PLAN quantity. Over the routed copper at K51 the two boards TIE (337
against 338), so crossings are not what separates them.

**And it routes BUNDLES by (arc, layer) where we route lanes.** That is the
K51 shape and it is still unsolved. The one measurement that came out of
the abandoned arc line says the same thing from the other side: a slanted
comb reached the human's 80 / 0 on `hbn3_k51` when it was handed the
HUMAN's ends -- which is what sent this work to the PLAN side rather than
the braid side.


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
B berth's rank -- MEASURED and losing at both K28 and K51, see the ledger).

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

## The synthetic harness (agent, 2026-09-15)

Every number above this line comes from **one** bench graded against **one**
human layout. That gives a ranking but never an *optimum*, so "106 vias"
carries no information about whether 106 is 30 too many or 2. Two files,
both general -- nothing in either names a board, a net, a face or the bench:

* **`synth_bus.py`** writes a case: a 2-layer board at the bench's 0.1 mm
  process, two BGA arrays facing across a channel, `K` two-pad nets in a
  chosen pin pattern, optional foreign parts, and a `<stem>.truth.json`
  carrying the planted permutation and its answer.
* **`synth_ladder.py`** runs a batch: generate, `make_bench.py`,
  `chain_k.sh`, then grade against the answer.

```bash
python3 synth_bus.py --self-test              # the truth model checks itself
python3 synth_bus.py out.kicad_pcb --k 15 --pattern interleave
python3 synth_ladder.py --batch b1            # 23 cases, patterns x K
python3 synth_ladder.py --batch b1 --regrade  # re-grade, no chain
```

### Three answers, and they bracket

For a peripheral bus (both ends of every lane on F) the channel is a
permutation and two lanes cross iff their order is inverted. Then:

| | what it is | when it exists |
|---|---|---|
| `lb` | `2 * (K - LIS(pi))` | always -- still valid with an obstacle |
| `opt` | best **whole-lane** solution: the proper 2-colourings of the crossing graph | only when that graph is bipartite |
| `dp` | best over **all** routings, mid-channel changes included | K <= 22 (4.5 s at 22, hopeless at 28) |

`dp` is the number to grade against -- the whole-lane model returns nothing
for an odd cycle, which is exactly the interesting half of the space.
`--self-test` asserts `lb <= dp <= opt` on every pattern at K in 4..12. It is
exact for **one crossing order** (the straight-line one) and one homotopy
class; that caveat is in `crossing_events`' docstring.

### Reading a result honestly

`PLANNER gap` = plan-implied count - optimum; `BRAID gap` = routed vias -
plan count; `DP gap` = routed vias - optimum; `DETOUR` = routed mm /
straight-line mm. Three columns decide whether any of them means anything:

* **`thru`** -- lanes with copper INSIDE an array. The optimum is
  channel-confined, so `thru > 0` means a cheaper topology than the model
  describes was available and a negative gap is **not a win**. The `b1`/`b2`
  pair separates it: the same cases with the array interiors closed.
* **`slot_cap`** -- on an obstacle case, how many lanes can physically pass.
  Without it `open=4` cannot be read: a router defect if they fit, a correct
  refusal if they do not.
* **`planner`** -- whether `pages_first` ran at all. It prints in the FANOUT
  log, and an INFEASIBLE solve prints nothing, which reads exactly like
  "never ran".

### What it can and cannot do at K41+

* **Not a fast screening loop there.** A generated K41 case routes in ~40
  minutes against the bench's ~5, because the synthetic array is far more
  porous than a real BGA and the router searches the whole interior.
* **Its TRUTH MACHINERY is the instrument, and that transfers** -- the
  floor, the crossing census and the escape count all run on the real board.
* **The permutation family calibrates; the geometry does not.** The bench's
  bus IS statistically a uniform random permutation and `shuffle` matches it
  on every quantity the planner consumes (K41: crossings 406 vs 399, LIS 10
  vs 9). But `thru` is ~20% on the bench against 40% either way in the
  harness, and is non-monotone in ball diameter -- the wrong calibration
  target as it stands.
* **K51 needs a target-LIS generator.** Pinning the crossing count with
  `--inversions 539` reproduces the bench's crossings exactly and lands at
  LIS 6.2 against the bench's 11.


## The harness grades the JUDGE, and K51 gets diagnosed (2026-09-17/18)

TODO item 1 is "fix the judge", and the evidence for it was one number on
one board: at `PLAN_PAGES_DET=5000` the K41 plan solves to PROVEN OPTIMAL
with a 24% better objective and routes **twelve vias worse**. That says the
objective is wrong somewhere, but not where. The synthetic harness says
where, in seconds, because it knows the answer.

### The truths it adds

* **`pages_model(...)`** -- the exact optimum of the **planner's own
  model**. Two lanes are inverted exactly when they cross, so a "page" is
  a crossing-free set is an *increasing subsequence*: cover the lanes with
  two of them, pay each its end-mismatch price, pay `PLAN_PAGES_SWIM` for
  the rest. An O(K^3) DP, so it answers at K=51 in milliseconds where
  `exact_dp` stops at 22.
* **`exact_dp(..., fixed=)`** -- the exact optimum with lanes PINNED to a
  page for the whole channel, which is what a whole-lane plan IS. So
  `exact_dp(fixed=the model's pages) - exact_dp()` is the exact price of
  planning in whole lanes. Pinned lanes are not variables, so the 2**n cap
  applies to the swimmers, not to K.

Both are checked against brute force, **Greene's theorem** (a different
algorithm) and closed forms in `--self-test`.

**A planner gap splits in two, and the halves want opposite work:**

    MODEL error    = the best plan the two-page model can express - the optimum
    solve_vs_model = the plan this solve actually found           - that best plan

A model error means more solving is **wasted** -- the answer is not in the
model to be found. The model error lives only on IRREGULAR permutations
(on `sorted`/`blocks`/`interleave`/`riffle`/`reversed` the model's best
plan costs exactly the optimum), which is what the `shuffle` batch is for:
K=12 seed 0 is **+14 vias**, K=15 +10, K=18 +4.

### The objective is DEGENERATE, and the swim term is where

`python3 synth_bus.py --judge` builds a pool of model-feasible plans and
scores each both ways -- the planner's objective, and the exact via count
of the best routing consistent with it.

Group the plans by how many lanes they swim and the swim price cancels,
leaving only the objective's opinion about WHICH lanes to page. Over 36
groups: **mean true span inside a group 13.7 vias, and 9 groups where the
objective is CONSTANT across a spread of up to 12 vias.** Not weakly
correlated -- one value, no ranking. No budget, solver or tie-break inside
this model can choose between those plans.

**The anti-correlation, reproduced in two minutes by moving a price** --
`chain_k.sh` at K28, `PLAN_PAGES=1`:

| `PLAN_PAGES_SWIM` | what the planner chose | its objective | **routed** |
|---|---|---|---|
| 100 (shipped) | F 18 / B 10, **0 swimmers** | 28 model vias | **34** |
| 2 | F 25 / B 3, **10 swimmers** | **8** model vias | **42** |

The objective improved 71% and the board got eight vias worse.

**What a swimmer really costs, from the braid's own mouth.** The braid
reports `swimmer X: 8 page crossing(s), 6 change(s)` and always did. Over
262 swimmers it is about **3.5 changes each**, and not a constant:
`changes ~ 0.44 * page_crossings + 0.79` (r = 0.83). Neither model has
this number -- the exact DP prices a swimmer at 2, `pages_first` at 100,
the braid pays ~3.5. `PLAN_PAGES_SWIM_XING` adds the pairwise crossing
term (default off; flag-off copper IDENTICAL).

**No price ever beat the shipped flat 100** on the K28/K35/K41 ladder, so
nothing shipped (edict 3; the arms and their numbers are in git). One
result is worth carrying: a **tie-break-only** price takes K28 to 30 vias,
0 open -- below every recorded arm and 16 below the human, repeated
30/30/30 -- **and breaks K41** (91 with 2 open, repeated). It changes not
how MANY lanes swim but WHICH, which is exactly the degeneracy above:
right mechanism, unfinished term.


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
  human's 5, and the human proves 81 is reachable. But read 76 as a BOUND,
  not a target: it prices every swimmer at 2, and two vias a net is
  infeasible on this bench (next section). The reachable comparison is the
  human's 81.
* K28's -10 is the escape again, not a win over physics.

### K51's gap: what it is, and what it is NOT (2026-09-17/18)

The floor above says K51 carries the whole remaining opportunity. This is
what that gap is made of, measured on the best local arm
(`tmp/s13/rp_rp_k51`, 96 vias, 0 open, 0 DRC) against the human.

**It is ten nets, not fifteen.** The "fifteen nets / 39 excess" once
recorded here was taken on a 116-via baseline; on the shipping arm it is
**ten nets and 19 excess vias**. The human exceeds two vias on NOTHING.

| vias a net | ours | human |
|---|---|---|
| 0 | 9 | 7 |
| 2 | 28 | 40 |
| 3 | 1 | 0 |
| 4 | **9** | **0** |

**Crossing COUNT is not the discriminator.** Over the routed copper the
two boards are the same -- **337 crossings against 338**, 14.3 a net
against 14.4. The human routes a **32-crossing** net with two vias; we
route an **8-crossing** net with four. (The "68 against our 250" under
*What the human does* is the PLAN's berth order at K41, a different
quantity from routed geometry.)

**What decides it is an exact law.** Every pad at both ends of every net
on this bench is on F.Cu, so at a crossing a lane must take the opposite
layer from its partner. Walk a lane, write down the partner's layer at
each crossing in path order, and

> **vias = 2 x (maximal blocks of partners on the lane's OWN pad layer)**

which reproduces the routed count on **47 of 47 nets on BOTH boards** (the
few misses are escape vias, which cross nothing). It is the RUN STRUCTURE
that costs: `B...F...B` is two vias at any length, `F...B...F` is four.

| F-blocks along a lane | ours | human |
|---|---|---|
| 0 (holds F, 0 vias) | 11 | 10 |
| 1 (one dive, 2 vias) | 30 | 37 |
| 2 (two dives, 4 vias) | **6** | **0** |

### The two-via directive is infeasible -- and obeying it is WORSE

`pages_first`'s directive is *no net may need more than two vias*.
`joint_floor.py --cap N` asks that instead of assuming it: the minimum
vias over the board's OWN paths with every net capped.

| over its own routed paths | ours | human |
|---|---|---|
| uncapped joint floor | 84 | 70 |
| `--cap 2` | **INFEASIBLE** | 72 |
| `--cap 4` | 84 (the cap costs nothing) | -- |

Over our own copper **no layer assignment whatever gives every net two
vias** (HiGHS status 8, asserted -- never inferred from a missing
solution). It is not a realization failure either: 92 routed against an
84 floor is *tighter* than the human's 80 against 70.

**And the weaving nets are LOAD-BEARING.** Our paths are only two
un-crossings from two-via-feasible (`SA6xSA7`, `SA7xSBA0`). Make them:

| over our own paths | free (0v) | one dive | two dives | total |
|---|---|---|---|---|
| uncapped optimum | **10** | 32 | 5 | **84** |
| +2 un-crossings, uncapped | 10 | 33 | 4 | **82** |
| +2 un-crossings, **capped at 2** | **2** | 45 | 0 | **90** |

**Five double-divers buy eight free rides.** A lane that dives twice holds
B across a stretch, and that is what lets eight others hold F for nothing;
cap it and they all dive, 84 -> 90. Replicated on an independent board:
`cl_ctl_k51` (116 vias) is also cap-2 infeasible, shape 10 free / 30 / 6 /
1 = floor 90. The human's shape is **13 free / 33 / 1 = 70** -- more free
riders AND fewer double-divers, which are not in tension.

So the directive is not a target that was missed. It costs six vias where
it is met, and the ten weaving nets are not a defect to remove. (The same
conclusion was reached from the K35 side on 2026-09-12; the K51
infeasibility proof is the stronger form of it.) **`channel_lower_bound`
prices a swimmer at 2 as a LOWER bound, not an achievable one** -- read 76
as a bound, never as "78 is one via away".

**The channel itself is infeasible too**, not just our copper.
`synth_bus.cap_sat_feasible` asks it as satisfiability, which is what
scales -- the MILP ran 30+ minutes on this instance and was killed, CP-SAT
proves it in about a minute. On the channel our own fanout induces (48
lanes, 580 inverted pairs, LIS 11): **cap 2 INFEASIBLE, cap 3
INFEASIBLE**; a second K51 fanout (566 pairs) proves infeasible at two in
a second. `--cap-survey` shows why in general: on uniform-random
permutations a clean channel stops being two-via-feasible at about K=16
and no seed succeeds from K=20 up. Reaching it at all requires LEAVING the
channel -- un-crossing pairs by going around -- which is the one move
`pages_first` has no variable for.

### What is NOT the lever (measured, do not rebuild)

* **Un-crossing pairs.** Two moves buy two vias (84 -> 82). The gap is not
  removable crossings: both boards carry the same count.
* **The free-rider ceiling** (max crossing-free set; ours 13, human 16).
  Over ten routed K51 boards Spearman(ceiling, floor) is **+0.13**, the
  wrong sign -- `grpF_k51` has a HIGHER ceiling than our best board with a
  worse floor and 106 vias. Its bound `2*(K-ceiling)` is loose (68 against
  a floor of 84).
* **More escape reach on its own.** See `DST_CLIMB` below.
* **A plan-time channel floor as a RANKER.** Over the K51 fanouts, deduped
  to **6 DISTINCT plans** (21 of 33 boards were one plan under different
  knob names -- dedupe or every rho is inflated): inversions -0.03, LIS
  +0.10, 2(K-LIS) -0.10. The channel floor's +0.85 rests entirely on two
  disasters and is **FLAT at 61** across everything routing 99-123. Two of
  the six plans routed to two different counts (99 and 113; 116 and 123),
  so the plan does not fix the outcome to within 14 vias. As a REFUSAL it
  works -- see the held-out test below -- but it cannot rank the field.

The reason is structural: at the array boundaries the effective inversions
are **103-119 against 337-338 routed crossings**, so the channel model
carries about a THIRD of the real crossing system. A floor computed on it
is a floor on a third of the problem.

### The escape move: the bench says HYBRID, the board says not yet

The move the fanout really has is the SLOT each lane escapes and berths
at -- the pads are fixed, the order they present to the channel is not.
`synth_bus.escape_move_floor` maximises the free-rider ceiling (a global
MILP on a proxy); `escape_search` scores the same moves with `cap_floor`
itself; `start=` seeds the second from the first. Four arms, same cases,
all scored on the TRUE floor:

| total true floor, 8 cases | none | ceiling | floor | hybrid |
|---|---|---|---|---|
| | 332 | 232 | **250** | **192** |

**Hybrid wins or ties 8 of 8, -42% against no move.** The instructive part
is that steering by the true floor ALONE (250) is WORSE than the proxy
alone (232) -- and it is not a compute artefact: giving the floor arm two
and four times the budget changes nothing at all,

```
 K=20 seed 0 reach 2 | floor@120 46   floor@240 46   floor@480 46 | hybrid@120 34
```

The local search SATURATES at a local optimum and the global seed is what
escapes it. So the shape is **proxy for the coarse structure, `cap_floor`
for the refinement** -- not "replace the proxy". Steering by the ceiling
alone is wrong for the same reason it is not a screen: `--escape-survey`
prints the true floor beside it and flags the rows where the ceiling rises
while the true floor gets WORSE (K=16 seed 0: ceiling 6 -> 7, floor 28 ->
32).

**On the real chain, reach alone LOSES.** `DST_CLIMB=k` is exactly this
move -- destination dog-bones whose run climbs along the array before it
leaves, "the class that makes a B berth's rank free". Recorded losing at
K28 (2812 berth candidates, 718k exclusions, the solve stopping worse,
**42 against 34**), and run as a paired arm at K51, `PLAN_PAGES=1`, both
arms launched together:

| K51 arm | vias | open | plan-time channel floor |
|---|---|---|---|
| control | **116** | **0** | 61 |
| `DST_CLIMB=2` | 123 | **3** (SA10, SDQ0, SDQ11) | 66 |

The arm is LIVE (the two fanouts differ byte-wise), so this is not a
vacuous null. It is worse three ways: **not one of its four portfolio
boards completed** (139/7 open, 154/3, 137/3, 123/3, against the control's
121/0 and 116/0); its braid ran five to ten times longer, past edict 2;
and an open board's via count is artificially LOW, so +7 understates it.

**The screen called it before the braid ran** -- inversions 566 -> 692,
LIS 11 -> 9, channel floor **61 -> 66**, the worst value on record -- and
the prediction was written down first, which is the held-out test the
refusal reading needed.

The lesson is the bench's, in the real chain: **reach is only worth having
if the thing choosing among the candidates can use it.** The planner's
objective is measured anti-correlated with the route, so more candidates
move the solve's stopping point, not its quality.

**Nothing here is committed to the chain** (edict 3). What ships is the
instruments and this record.

### The instruments, and one method lesson

| | |
|---|---|
| `joint_floor.py BOARD K --cap N` | minimum vias over a board's own paths with every net capped. **INFEASIBLE means re-plan, not re-solve** -- it is a fact about the paths |
| `synth_bus.cap_floor` | the same on a generated channel. Uncapped it must equal `exact_dp` (different algorithm, same model) |
| `synth_bus.cap_sat_feasible` | the cap as satisfiability (CP-SAT), for when only the answer matters -- it scales where the MILP does not |
| `synth_bus.py --cap-survey` | the two-via directive dying with K |
| `synth_bus.escape_move_floor` / `--escape-survey` | the escape move by the ceiling proxy, with the true floor printed beside it |
| `synth_bus.escape_search` | the same move steered by `cap_floor`, `start=` to seed it from the proxy |

**One method rule came out of building these, and it generalises: a
feasibility cross-check run only on FEASIBLE cases cannot fail.** The first
CP-SAT model here called the real channel feasible at cap 2, contradicting
the MILP -- a hand-spelt XOR with its transition literal inverted, so the
constraint bounded the NON-transitions, which every alternating assignment
satisfies. The cross-check that should have caught it ran only over cases
that were all feasible, so it agreed with the bug and printed ALL PASS.
`--self-test` now pins INFEASIBLE witnesses, which is the only half that
can fail.

## TODO

Ordered, highest value first. An item leaves this list when it is **done**
or **abandoned with a measurement**; the abandoned ones and their numbers
are in the git history of this file.

1. **Fix the judge.** Solving the plan to proven optimality makes the board
   WORSE (K41 -24% objective for +12 vias; K51 -33% for two open nets), so
   more search on this objective is actively harmful, and `PLAN_PAGES_DET=40`
   is an accidental REGULARIZER rather than a budget. The braid-tier judge
   (route the candidates, decide on `(open, vias)`) is correct and measures
   neutral, so judging by routing is not automatically the answer either.
   The one mechanism that has paid is `replan.py` -- per net against the
   FINAL board with the router as an oracle -- and widening it is the
   obvious next experiment (the recorded K35 46 came from a much wider
   sweep than the `--rounds=4 --probes=2` used lately). Hours; wants the
   cloud.

2. **The berth menu: ROW pruning, not column generation.** Each net has
   ~19 distinct berth geometries across ~4 faces, and at `CANDS=4` the
   one-per-face guarantee consumes all four slots for 28 of 41 nets -- so
   the `vias + ride` ranking never chooses WHICH berth, only one
   representative per face. Most concrete plan-side lever on the list, and
   it feeds item 1. **Caution:** giving the planner MORE candidates made
   the solve stop somewhere worse at both K28 and K51 (`DST_CLIMB`), so a
   menu change is only worth making with a judge that can use it.

3. **K51 is the only rung with room, and the room is NOT "cap every net at
   two".** K41 is 68 against a floor of 66 and closer to it than the human
   -- essentially solved. K51 is 96 against the human's 81, and the gap is
   ten nets. The obvious target is refuted (see *K51's gap*): the cap is
   infeasible over our own paths and costs MORE where it is possible.
   What is open: the human reaches 70 with 13 free riders / 33 one-dive /
   1 two-dive against our 10 / 32 / 5. **The next concrete piece is a
   plan-time floor over the PLANNED LANES (not the channel), graded
   against the routed board** -- if it clears |r| > 0.2 it is the first
   judge this campaign has had that points the right way.

4. **Slack-gated arms.** The two largest single-knob wins both LOSE where
   there is capacity to spare, in the same shape: `BRAID_LAY_ORDER=xing`
   (K51 -13, K41 +6) and the pattern seed (K35 69 against 58, K41 88
   against 80). A working slack test would settle both at once.

5. **The `.kicad_dru` is read with real layer names inside the turned
   frame.** A per-layer clearance rule lands on the opposite face for a
   back-side part, and the dru outranks `--clearance` on every routing
   step. Shipped `py_router` code, so it blocks the merge to main.

6. **The corpus A/B for the production changes, then the PR to main.**
   Everything in *What this adds to `py_router`* changes copper on boards
   unrelated to this chain -- including `KICAD_SEG_DIST_EXACT`, which
   ships ON and whose own comment says the A/B is owed.

7. **Generality -- the router is tuned on ONE bench.** zynq_ad9364 at K28
   is 0 open / 55 vias but only 17 of 28 in band, and the leg rules were
   tuned on `fb_t2q_fresh`. `flow_frame.py turn` needs fixing first (that
   article does not run through `chain_k.sh`), and off-axis poses (R30,
   R45) break the plan's compass faces.

8. **Audit `modal_k`'s `KEEP`.** Only ~2 of ~40 distinct `pages-first:`
   log shapes survive the filter, and an INFEASIBLE solve prints no such
   line at all -- which reads exactly like "the planner never ran". Three
   misreads have come from this.

9. **`pick_braid` ignores DRC.** It judges `(open, vias)` only, so with
   the portfolio default-on it can silently ship a board with violations
   over a clean one a via worse.

10. **Merge main and re-baseline.** ~123 commits diverged, predating
    `#958`, `#441` and `#521`/`#906`, so the recorded ladder sits on older
    engine code -- and there is no `run_all_modal.py`, so the suite is
    local-only (~48 min) until the merge.

11. **Built, default off, never finished.** `DST_ASK_BAN` (one negative
    K35 reading, ladder never finished); `BRAID_PACK=1` (0 open / 0 DRC,
    vias unchanged, far fewer segments -- nobody has measured what those
    segments buy); `SRC_EXCHANGE=1` (2 of 60 pairs improved the judge at
    K15, and the question it was built to answer is still open).

12. **Unverified review findings.** `dedupe_boards` fingerprints copper
    but not the sidecar, so two copper-identical fanouts with different
    plans lose one; `braid_tier`'s budget exhaustion is silent;
    `_realize_group_first` bans nothing on a pure DRC rejection;
    `blockers_of` double-counts half a track; `collapse_dives` calls
    `os.chdir` at import; `flip_frame` does not mirror `pad.polygons`.

13. **Owed tooling**: the flag-off parity gate a hand check does today.
    Promote session probes into `awx/` with a line each here when they
    earn it.

14. **Three pre-existing suite failures** (predate this session):
    `test_703_predictor_regen`, `test_782_nondefault_netclass_clamp`,
    `test_fanout_cancel`, plus `test_459_group_routing` hitting its own
    1200 s budget.

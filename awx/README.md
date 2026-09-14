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
the coherent K-ladder. Byte-deterministic, 0 DRC at the routed 0.1 mm
floor, and identical under translation, turn-over and quarter rotation
(the pose gate).

Two arms are current, and **a number is meaningless without its arm**:

| | K15 | K28 | K35 | K41 | K51 |
|---|---|---|---|---|---|
| reference arm (one-pass plan) | 14 | 36 | 69 | 86 | -- |
| joint-solve arm | -- | -- | 58 | 80 | 137 |
| **BEST MEASURED** (`replan.py`, opt-in) | 14 | 36 | **46** | **76** | **107** |
| **human** | 22 | 46 | 58 | 70 | **81** (48 nets; 85 over 51) |

**The best row is the one to beat, and it is not the arm the chain runs by
default.** Every number in it is a real board in `awx/tmp/`, re-graded
2026-09-12 at 0 open / 0 DRC: `rw35e_rp_k35` **46**, `c3b_k41` **76**,
`rp6b_rp_k51` **107** (also `rw41e_rp_k41` 91, `rw35d_rp_k35_r1` 51).
**At K35 we BEAT the human by 12 vias.** The deficit is real at K41 (+6)
and K51 (+26), and it grows with congestion -- that is the shape of the
problem, not "we match at K35".

**2026-09-13, LOCAL, and the K51 line moved a long way -- but read the open
column.** `BRAID_LAY_ORDER=xing` then `replan.py`:

| board | vias | open | how |
|---|---|---|---|
| `rpW51_rp_k51` | **91** | **1 (SA1)** | `xing` 112->99, then `--worst=48 --probes=2 --rounds=6` |
| `rpX51_rp_k51` | 95 | 1 (SA1) | `xing`, then the default-width replan |
| `rpA41_rp_k41` | **82** | **0** | replan `--rounds=4` at the default width |
| `rpW41_rp_k41` | 83 | 0 | replan `--worst=41 --probes=2 --rounds=6` |

**K41 82 at 0 open is a real board and beats the cloud's 88**, though it is
still short of the recorded 76. **K51 91 is NOT a valid board** -- SA1 ships
open, and an open net UNDER-counts vias, so 91 is not comparable to the 107
until it closes. Closing SA1 is the single thing standing between this and a
new K51 record. Note the width did not decide either bench: K41 was BETTER at
the default width (82 vs 83), K51 better wide (91 vs 95).

These boards come from `replan.py` -- the route as the judge -- run wide
(`--worst=<the net count> --probes=2 --apply=strip`; `--worst` takes ONE
integer, so the `35..41` written here before was a range across K, not
syntax -- `int('35..41')` raises and no round runs). `--worst=N` re-plans
every REFUSED net plus the N most via-expensive ones, so N = the net count
puts the whole board in play against the default 3; `--probes=N` is how
many screened candidates per net AND end get a real router probe (5-20 s
each), so the recipe is ~25-30x the default's work per round. The K51 line was never run
at that width; it ran `--worst=6 --probes=1` and stopped at 107.

- **reference arm**: `SRC_ROUNDS=0 SEL_RETRY=6 EXACT_LANE=1
  DST_FACE_ASK=1 DST_WALK=3 SF_SWIM=30 BRAID_EXIT_GUARD=1
  BRAID_SWIM_HOLD=1`
- **joint-solve arm**: the above plus `SEL_XING=2 SF_EQUIV=2
  SF_JUDGE=braid BRAID_ONE_DIVE=5 DST_RESIDUE=3
  DST_RESIDUE_POOL=displaced DST_RESIDUE_CANDS=4
  BRAID_ALT_SOLVER=cpsat BRAID_CPSAT_DET=40`

The human's numbers come from `human_at_k.py`, scoped to the same
coherent net set the chain routes.

<img src="img/k41_corridor.png" alt="K41 on the bench: one corridor of 41 lanes, both pages, the rides round the destination" width="760">

*K41: the corridor from `U1` (left) to `DU1` (right) -- front lanes red,
back lanes blue, every lane in its band, the far-face exits riding round
the destination's east face.*

### The gap is FLOOR, not realization, and crossings do not set it

`ledger_cal.py BOARD K` splits every net's vias into the **DP floor**
(the layer changes its real crossings force, everyone else fixed) and
its **slack** (realization waste). On K51:

| board | crossings | vias | DP floor | slack |
|---|---|---|---|---|
| baseline | 392 | 136 | 130 | 6 |
| pattern seed | 275 | 100 | 96 | 4 |
| pattern + seat repair | 360 | 104 | 98 | 6 |
| **human** | **338** | **80** | **74** | **6** |

1. **Slack is ~6 vias on every board measured, the human's included.**
   The router turns a plan into copper as tightly as the human does, so
   the whole deficit is the PLAN. It also caps every realization-side
   idea at about 6 vias -- `collapse_dives.py` saved 2 on the baseline
   (137 -> 135, 0 open 0 DRC) and found nothing on the pattern arm. Note
  it re-lays a ripped lane, so it must use the BRAID's track and
  clearance: laying at its own 0.1/0.1 put 30 segments of thinner copper
  into the board and reported a bigger saving than it had earned.
2. **Crossing COUNT does not set the floor.** The human has MORE
   crossings than our best plan (338 against 275) and a floor of 74
   against 96 -- 0.22 layer changes per crossing where we pay 0.35.
   What sets the floor is how the crossings are STRUCTURED: which
   partner is on which layer at each crossing, and what end layers a
   lane must hold. **The mechanism is open and is TODO 1.**

`cut_ledger.py` (the Maley cut-capacity check on a plan before any lane
is routed) says **every cut fits** -- capacity 84-100 lanes against a
load of 2-7 even in the escape fields. Space is not the constraint at
either end; order and structure are.

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
but 17 of 28 in-band -- the gap a second board shows (TODO 8).*

<img src="img/pack_k28_before.png" alt="K28 before the pack" width="400"> <img src="img/pack_k28_after.png" alt="K28 after the pack" width="400">

<img src="img/pack_k41_before.png" alt="K41 before the pack" width="400"> <img src="img/pack_k41_after.png" alt="K41 after the pack" width="400">

*The pack (`pack_board.py`, 4 s, opt-in `BRAID_PACK=1`): 1574 -> 875
segments at K28, every lane, 0 open, 0 DRC, the vias exactly where they
were. Before, the bottom river is a fan of router staircases; after, it
is horizontal lanes with 45-degree jogs. K41 goes 2258 -> 1843. What the
segments BUY is still unmeasured (TODO 11). At K41 the far-face rides
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
octilinear, so a non-orthogonal pose is outside both models today
(TODO 9).*

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

## What the 2026-09-12 audit changed

Ten parallel audits over everything this branch adds. The defects are
fixed and in the log; what matters here is which RECORDED CONCLUSIONS
they invalidate, because those are the ones that would otherwise be
believed forever:

- **"The berth chooser accepts nothing" was never a null result.**
  `_alts5` stage A pinned a moving net's as-laid lane vias into the
  objective, so every candidate was charged the vias it would stop
  paying as well as its own. Only a move flipping a swimmer could win.
  Every source and berth arm judged through it was measured under a
  systematic bias.
- **`SPLIT_BLOCKS` was never measured.** `NEST_IN`, `NEST_STEP` and
  `BAND_LPITCH` were used and defined nowhere in any commit, so the band
  path raised NameError on the first band exit. Its recorded verdict
  ("complete, general, lost") is void -- it is a crash.
- **`SEL_XLAYER`'s verdict is void too**: that arm ran through a
  `zip(placed_nets, placed_legs)` desync pairing a trial leg with a
  stale net.
- **The joint solve had no compatibility constraints at all.**
  `plan['alt_excl']` was nested under `if xing:`, and `DST_XING` is 0 by
  default -- so every recorded `DST_RESIDUE=3` result was produced by a
  solve free to pick two berths that cannot both be laid.
- **The DRC feedback could not see a pad short**, so a pass could print
  "every berth laid as planned" on a shorted board.
- **"The human's vias are at the ENDS" could not be falsified**: the
  census's MID class did not exist (it was nearest-of-two). With it
  implemented, several of the human's vias are mid-field.
- **`grade_k` graded a crashed checker as clean**, and missed a net with
  no copper at all -- an arm that DROPPED a net scored as a via win.
  Re-graded every headline board with the strict tool: all unchanged, so
  no recorded number was actually wrong.
- **The judge's answer depended on the machine's core count** -- pool
  workers inherited each other's level-5 seeds, and at a 30-node budget
  the seed is the answer. Same class as the clock rule, different hat.

Two lessons worth keeping: an instrument that cannot measure must FAIL,
never report clean; and a knob measured negative through a broken path
has not been measured.

## What the 2026-09-13 session established

**Read this before running anything.** It invalidates a comparison this
campaign made routinely, and it closes off a whole class of work.

> **THIRTEEN KNOBS NAMED IN THIS FILE ARE NOT COMMITTED** -- they live only
> in the working tree, so a fresh checkout CANNOT reproduce the measurements
> that name them. They are instruments and records of NEGATIVE results, not
> improvements, which is why they were held back:
>
> | knob | why it is not committed |
> |---|---|
> | `SF_KEY_COST` | confirmed K44 regression, 98 -> 112, spread 0 over three containers |
> | `SF_SWIM_MODEL` | harmful on the cloud: K51 105 -> 131, K44 only 106 against a 98 baseline |
> | `BRAID_KEEP_SCHED_PAGES` | costs 12-16 vias on EVERY bench (K35 62->74, K41 88->104, K51 105->117) |
> | `BRAID_CPSAT_CONFLICTS`, `BRAID_CPSAT_CONVERGE` | unusable: a 40 s solve had not finished in 31 MINUTES |
> | `BRAID_ALT_TWOSTAGE`, `BRAID_L5_W_PER_LANE`, `BRAID_CPSAT_ENF`, `BRAID_W_PER_NET` | built for the two-stage/CP-SAT study; never shown to beat the baseline |
> | `DST_RIDE_W` | rejected -- K51 regression |
> | `SF_LDS_W` | inert (wired and verified live; it just decides nothing) |
> | `SF_RIDE_W` | correct and default-inert, but NEVER exercised through a chain |
> | `SF_REPAIR_CALLS` | a counter from the CP-SAT repair probe |
>
> The FINDINGS they produced are real and are recorded here. Everything else
> this file names is committed. **This list is auditable** -- every
> backticked `*_KNOB` in this file was checked against the committed
> sources; re-run that check when adding one.



**The short version, in the order it matters:**
1. **The plan-side swimmer price is CLOSED, and the per-lane model is
   HARMFUL, not merely inert.** A swimmer's via cost is not predictable at
   plan time (four feature families, |r| <= 0.13 over 97-98 swimmers on 5
   boards), so no cost term built on the plan can rank it -- and the cloud
   arms confirm it end to end: `SF_SWIM_MODEL`+`SWIM_CHANGES` costs **K51
   105 -> 131** and reaches only 106 at K44 against a 98 baseline. Do not
   add another without clearing the |r| > 0.2 bar (`room_probe.py`).
2. **The judge has no resolution on vias** -- it gave the SAME 73.0 predicted
   vias to a 98-via and a 112-via board, so the choice fell through to
   corridor length: 5.3 mm, costing 14 vias.
3. **`replan.py` (the route as the judge) is the only lever that moved a
   bench**: K41 91->82 at 0 open, K51 112->99->91 (SA1 still open).
4. **`BRAID_LAY_ORDER=xing` is the largest single-knob win** (K51 -13) and
   is slack-dependent (K41 +6). An arm, never a default.
5. **K51 COMPLETION, not the via count, is the open problem.** The 91-via
   board is invalid for ONE net (SA1) and it is fully diagnosed: the lane
   was never laid, walled at both ends, ends on opposite layers, tail
   exhausted. Search budget and via room are both measured INERT on it.
6. **Never compare local to cloud** (below) -- and note a verdict in the
   settled table belongs to the BOARD it was measured on:
   `BRAID_VIA_ROOM_REFUSED=2` "breaks K51" on the old 137-via baseline and
   is bit-identical on the 99-via one.

### Never compare a LOCAL result to a CLOUD one

They are not the same experiment, for two separate reasons, and the gap is
large: K35 local 76 vs cloud 60; K41 local 107 vs cloud 88.

1. **`max_deterministic_time` is not portable across architectures.** On a
   byte-identical instance (md5 verified both sides), `BRAID_CPSAT_DET=40`
   gives arm64 **1685 conflicts / obj 41.845872** and x86_64 **1970 / obj
   40.845888** (Intel and AMD agree exactly with each other). x86_64 gets
   1.17x the search per deterministic unit. ortools never claimed otherwise
   -- `sat_parameters.proto` documents it only as "correlated with the real
   time used by the solver". It buys immunity to machine LOAD, which is what
   this chain needed, and nothing about different CPUs.
   `BRAID_CPSAT_CONFLICTS` (new, default 0 = unchanged) bounds the solve by
   `max_number_of_conflicts` instead -- an integer count of discrete search
   events, identical on any CPU. **2000** matches today's x86_64 depth.
   CAVEAT: a conflict count bounds SEARCH, not TIME. It does not bound
   presolve at all, and the conflict rate is instance-specific (~42/s on the
   K35 alt instance, unmeasured elsewhere). Ship it with a generous
   deterministic-time backstop AND record which of the two fired -- a
   backstop nobody checks is how this class of bug returns.
2. **The second divergence is SOLVED, and it runs through the CP-SAT
   solution VECTOR rather than its budget.** Equalising the budget leaves
   the routed board byte-identical (arm64 DET=40 and DET=80 both give K35
   76v/1376s), which is what made the solver look innocent -- but CP-SAT
   returns a DIFFERENT SOLUTION VECTOR per architecture at the same
   objective, and `braid._profiles5` rewrites `sched.page[nm]` from that
   vector. One net's page flips, the residue goes 14 vs 13, `f` differs by
   exactly 1.0000 (the fraction bit-identical), a different berth is
   accepted, and 10 of 35 berths end up different. Freezing the rewrite
   (`BRAID_KEEP_SCHED_PAGES`) does remove the channel and costs 12-16 vias
   on every bench, so the rewrite is load-bearing: see the settled table.

### The local-vs-cloud bisect, as far as it went

Everything upstream of `residue_choice` is bit-identical across arm64 and
x86_64 -- net set, escape MENUS 35/35, cost vectors 0 of 35, ranking order,
and the `lane_free`/`_conflict`/`band_room` accept decisions 140/140. The
divergence enters ONLY at `residue_choice`, the stage that judges berths by
running the braid, and then 10 of 35 berths differ. ELIMINATED by direct
test: `grid_router` (identical copper on a byte-identical board, sha
`fd04a350d0ad` on arm64 and two x86_64 containers), the CP-SAT budget,
`BRAID_L5_SEED` (0 and 1 give the same local board), and the 449 MB on-disk
taut memo (cold gives the identical board -- it IS a pure cache).
LOCALISED TO: `plan_braid`'s PAGE ASSIGNMENT. The same plan is judged
residue 14 locally and 13 in a container, `f` differing by exactly 1.0000 --
one swimmer -- with every net's launch_idx, target_idx and corridor
IDENTICAL. Five nets differ only in their page (SA15, SBA1, SODT0 gain one
in the cloud; SDQ0, SDQ15 lose one). The page branch in `schedule.py` is
integer-only -- `lis_keep_weighted`'s weights are {0.5, 1.0, 1.5}, exact
dyadic; `inverted` compares indices; the fill sorts on a count -- so it
cannot diverge on identical inputs. **ANSWERED: the input that diverges is
the CP-SAT solution vector, via `_profiles5`'s page rewrite** (see item 2
above).

### Every parameter response is JAGGED -- single arms are not evidence

Five independent sweeps in one session, all non-monotone:

    DST_RIDE_W      K41  0.25->107  0.5->92  1.0->88  **1.5->124**  2.0->84  3.0->82*  4.0->82*
    DST_RIDE_W      K35  62  62  60  **62**  59  **83**  68
    DST_RESIDUE_CANDS K41 joint 4->107 8->94 16->106 ; two-stage 4->94 8->82* 16->118
    BRAID_RESIDUE_W K41  6->84  10->89  14->114
    BRAID_L5_SEED   K28  net-set 34 / none 36 / plan 40          (* = ships open nets)

**K35 is BISTABLE** -- three identical cloud containers gave [60, 70, 70],
mode 70 -- so quoting 60 as "the baseline" understates every K35 comparison.
**K41 and K51 baselines reproduce EXACTLY** (K41 88v/2545s over three
independent arms; K51 129v/1open twice). Grade there, never on K35 alone.
The `nodes` column of `BRAID_SOLVE_DUMP` was None for every CP-SAT solve
ever recorded; it now carries NumConflicts, the one portable unit.

### The excess is a TAIL, and the machinery that closes it is via-blind

First-pass scheduled lanes are at per-lane parity with the human (abp K41
1.73 vias/lane vs 1.71). The gap sits in lanes REFUSED on the first pass and
then closed by the x4 budget, the last call, or a blocker rip, at 5-8 vias
where a first-pass lane costs 2. On the best K41 board (76) the whole +6 over
the human is four rescued lanes; at 2 each that board is 66. Every step of
that chain accepts the FIRST completion without pricing vias: the x4 rescue
takes whatever the wider search returns (`braid.py:6960`), `connect_ladder`
returns the first rung that succeeds (`:7308`), and `rip_for` runs only when
the route outright fails and keeps the first complete trial (`:7337`) -- the
K51 log shows it accepting a victim at 2 -> 4 vias. That is why every
PRICE term measured inert: they price the plan, and the excess is set
downstream by acceptance rules no price term can see.
`modal_k.py`'s `KEEP` filter dropped exactly those attribution lines, so a
cloud via count could be read and never explained; it now carries them
(`lanes: N/K`, `rescued at x4`, `last call`, `rip [`, `econ re-lay`,
`unplaced`, `NOT escaped`).

### Measured this session, and NOT committable

| arm | verdict |
|---|---|
| `BRAID_ALT_TWOSTAGE=1` (choose, then schedule) | every solve PROVED optimal (14/14, zero gap) vs the joint model's 2 of 4 with gaps to 28.77 -- and the board does not improve: K35 identical, K41 -8 fair vias but +1 open. **Convergence is not what costs vias.** |
| Dantzig-Wolfe | refuted before building: only **15%** of the choice MILP's rows are lane-local, 84% are 2-lane coupling, so the master would carry the model |
| CP-SAT enforcement literals (`BRAID_CPSAT_ENF`) | gap 32.49 -> 32.10. CP-SAT's presolve already does it -- its bound is 9.36 where the LP is -19.61 |
| `BRAID_L5_W_PER_LANE=1` | gap -> 31.08, identical board. The "swimmers 24.45" slack was LP-relaxation slack, which CP-SAT does not use |
| `DST_RIDE_W=2.0` | K41 88->84 (-18% copper), K35 59, K15/K28 byte-identical -- but **K51 129v/1open -> 118v/3open**, a completion regression, and its neighbours 1.5 and 3.0 are much worse on both benches. A lucky point, not a corrected rate |
| `DST_XING` / `DST_CONTEND` / `DST_SWIM` | all three built-but-zero terms are **correctly zero**. At K41 vs base 88: XING 93/100/110/119, CONTEND 90, SWIM 98 |
| `replan.py --apply=strip` | its branch is gated off by **"a SOURCE move stands"** and never executed in any arm; the whole A/B compared two identical code paths |
| `BRAID_RESIDUE_W=10` | **the one live lead.** K28 38->36, K44 98->94, K47 120v/4open->108v/3open, K51 **129v/1open -> 117v/0open** (first complete K51), K41 +1, K35 60->78. Pays where the two-page capacity binds; wants to be a RULE scaled by the plan's own residue fraction, not a constant |

`replan.py` had been crashing on every round since its first commit (`:+d`
on a float residual); fixed, and its first working run took cloud K35
60 -> **56**, matching the best K35 on record and beating the human's 58.

## The judge has NO RESOLUTION on vias (2026-09-13, K44 cloud dumps)

The clearest measurement of the planning metric yet, from the two K44 arms
that differ ONLY in the judge's sort key (`d44base` 98 vias, `d44key` 112):

| arm | model's predicted vias | judged `f` | => ride | ROUTED |
|---|---|---|---|---|
| `d44base` | **73.0** | 167.25 | 94.25 | **98** |
| `d44key`  | **73.0** | 167.96 | 94.96 | **112** |

1. **Both plans predict the SAME 73.0 vias.** The model cannot tell a 98-via
   board from a 112-via one. It has zero resolution on the quantity being
   optimised.
2. **So the decision was made on `ride` alone** -- corridor length, 56% of
   `f` (94 of 167). `VIA_MM = 7.5`, so the 0.71 gap in `f` is **5.3 mm**.
   The judge took a plan 5.3 mm shorter and 14 vias worse.
3. **The error is DIFFERENTIAL, not a bias**: 73 vs 98 is -25, 73 vs 112 is
   -39. This is why NO constant reprice can fix it -- a per-swimmer delta
   adds 21*d to one plan and 22*d to the other, moving the gap by 1*d, so
   closing 14 vias needs d = 14 vias PER SWIMMER. Measured exactly as
   predicted: `SWIM_PRICE` 3 and 3.5 both returned 112, and
   `SF_SWIM_MODEL`+`SWIM_CHANGES` is bit-identical at K41 and K51.
4. Where the 14 lives: swimmers 70 -> 78 (+8, mean 3.33 -> 3.55) and PAGE
   LANES 28 -> 34 (+6, mean 1.22 -> 1.55). Not a swimmer-only defect.

**The model prices by CLASS (swimmer = flat 2.0, page lane = its changes),
never by the CONGESTION the lane will actually meet.** Two plans with the
same class histogram are indistinguishable to it however differently they
route.

**Note the arithmetic above retires the CONSTANT only.** A per-swimmer delta
moves a 21-vs-22-swimmer gap by 1*delta; a PER-LANE model is free to move the
two plans by different amounts, so it is not refuted by that argument.
It is refuted by a different measurement -- see below.

### A swimmer's via cost is NOT PREDICTABLE from the plan (98 swimmers, 5 boards)

Actual swimmer vias: mean **3.19**, sd **1.77**, range 0-10. Against every
plan-time feature:

| predictor | corr with actual vias |
|---|---|
| page crossings | +0.016 |
| changes (what `SF_SWIM_MODEL` prices) | **-0.032** |
| diamonds reserved | +0.074 |
| airline length | +0.056 |
| copper length | +0.168 |
| detour ratio copper/airline | +0.268 (and POST-route) |

Per board the crossing correlation is not merely weak but SIGN-UNSTABLE:
-0.380 (d44base), +0.121 (d44key), +0.181 (K51 local). And a learned per-net
prior is not available either -- over 23 nets that swam on >=3 boards the
**WITHIN-net sd is 1.34 against a BETWEEN-net sd of 1.01**, so the same net
varies more across boards than nets differ from each other (SA8:
[2,10,6,3,2,6,2]).

**The cost is a property of the REALIZED board, not of the net or the plan** --
the braid absorbs most predicted crossings without a via (a 13-change swimmer
came out at 2), and whether it can is local ROOM, which has not happened yet
when the judge runs. Consequences:
- a better CONSTANT (3.19, not 2.0) fixes the level and adds no ranking power;
- the swimmer COUNT is a rational REGULARISER, not an accident: refusing to
  trade a structural property for a sub-2-via gain in an unpredictable
  quantity is correct, which is the argument against shipping `SF_KEY_COST`
  alone;
- **ROOM WAS TRIED AND IS ALSO NULL** (`room_probe.py`, 97 swimmers over 5
  boards). Longitudinal room between a change's two crossings, lateral
  crowding in the window a change needs, and a room-WEIGHTED change count
  all fail: best |r| = 0.125 (`loose0.1`), `crowd` -0.003..-0.026, and the
  room-weighted count -- the actual candidate term -- is **-0.001**. At
  n=97 significance needs |r| > ~0.20, so none of these is distinguishable
  from zero. CAVEAT: this room is measured in the crossing-ORDER coordinate
  (position 0-1 along the lane), which is topological; PHYSICAL room (mm of
  channel against via diameter, the radial via-room result) is not tested
  by it -- but is not available where the judge runs either;
- the only accurate estimator is the braid itself (`replan.py`), which is where
  every best board comes from.

## The replan, verified end to end (2026-09-13)

`replan.py` is the only estimator of a lane's cost that is ACCURATE, because
it is the braid itself -- and after today it is also the only lever that has
moved a bench this session. Verified on a real K51 run, not inferred:

```
round 1: apply path = DERIVED (strip)
round 1: KEPT derived -- open ['SBA2'], drc 0, vias 110 (round start ['SBA2']/112)
```

- **The faithful apply path fires.** `--apply=strip` derives the round's
  fanout board from the probes' own routed board, so the ends agree BY
  CONSTRUCTION. Confirmed by artifact as well as by the log: the strip branch
  writes `_r<N>.census.json` and no `_r<N>_fo.log`; refan writes the log.
- **Source moves are really probed** (`ends_try = ['dst','src']` by default,
  plus `both` pairs) and really stand.
- **It improves boards**: K51 112->110 at the default width, K41 91->82.

**THE GATE TO KNOW ABOUT.** The strip path needs four conditions and one of
them is `'a SOURCE move stands'` being FALSE (`replan.py:1447`). So when the
source arm succeeds, strip is silently disabled and the apply falls back to
`refan`. Observed firing 2026-09-13 at the wide width:

```
round 1: apply path = incremental/refan -- blocked by a SOURCE move stands
```

Two things temper it, and both were measured rather than assumed:
- **refan is AUDITED and was FAITHFUL there** -- "every move laid in its
  class", "39/39 unmoved teeth unchanged", "ends of 4 changed net(s) agree on
  both boards", and the round was KEPT (91->83). When the audit fails the
  round is REJECTED and the moves banned, so the current behaviour is SAFE,
  not broken. What the gate costs is WORK (a full re-fan, 466 s that round)
  and the rounds the audit throws away.
- **the gate looks unnecessary**: `fan_src[nm]` points at the probe's
  `_dst.kicad_pcb`, which is copied from `cur` AFTER `sr.realize` lays the
  new tooth -- verified on three probes (SA11, SA12, SA14): the source
  copper changes and `_dst` carries it through identically. `git log -S "a
  SOURCE move stands"` finds no commit and no measurement behind it; it
  traces to the strip commit's stated scope, "Destination moves only."

**Width did not decide either bench** (K41 better at the default width, 82 vs
83; K51 better wide, 91 vs 95), so `--worst`/`--probes` is not a free win --
it is ~25-30x the work per round for a board that may be worse.
**`--mode=rebraid` is INERT at both K41 and K51.**

## SA1: why K51's last net is open, and what does NOT close it (2026-09-13)

K51's best board (`xing` + replan, 91 vias) is INVALID for one net. The
refusal record (`<board>_refusals.json`) and the braid log name it exactly:

```
SA1  tooth (127.027, 67.829) B.Cu   berth (146.354, 63.361) F.Cu
     page null (a SWIMMER), stage last_call,
     failed_rescues 3, margins [2.0, 4.0, 6.0], rip_assist true
```
```
forward  cell ... layer=1: ok, 6/8 neighbors blocked
    Blocking obstacles: /DDR3 16x1/SA1(3 track)      <- its OWN stub
backward cell ... layer=0: ok, 6/8 neighbors blocked
    Blocking obstacles: /DDR3 16x1/SA4(9 track)
```

**The lane was never laid at all.** On the shipped board SA1 has ONE B.Cu
segment at the tooth and nine F.Cu segments at the berth -- the whole 17 mm
corridor run between x=127.03 and x=144.33 is missing. The A* is walled at
BOTH ends (6 of 8 neighbours blocked): its own stub at the tooth, and SA4 --
47 segments sprawling x 126.7-144.3, y 60.7-70.3 -- across the berth
approach. Its ends are on OPPOSITE layers, so it must change layer somewhere
in a corridor that has no room for the dive.

**The tail is EXHAUSTED, not unused.** Three rescues at 2/4/6 mm, the
blocker-directed rip (`rip_for`) taking out SBA1 and then SCKE1, and
`last_call` -- all ran, all refused. Note `failed_rescues < 3` and
`margins=[2.0, 4.0, 6.0]` are HARDCODED in `braid.py` with no env knob;
only `BRAID_ATTEMPTS` (6) and `BRAID_BUDGET_X` (4) are tunable.

**What does NOT close it** (all on the 99-via `xing` board):

| arm | result |
|---|---|
| `BRAID_VIA_ROOM_REFUSED=2` | **bit-identical** to the base. The settled table's "breaks K51 (3 open / 30 DRC)" was measured on the OLD 137-via baseline and does not reproduce here -- it is simply INERT in this regime |
| `BRAID_ATTEMPTS=10` | **bit-identical** to the base |
| `BRAID_BUDGET_X=8` | 97 vias but **2 open** (SA11, SDQ11) -- a deeper budget moved the refusal, it did not remove it |
| `replan.py --worst=48 --probes=2 --rounds=6` | 99->91 vias, SA1 STILL open, though the wide replan puts every refused net first in its queue by construction |

**So the block is not search budget and not via room -- it is that the
corridor has no room for this lane's dive at all.** The next thing to try is
the thing none of these touch: change what SA1 is ASKED for (its berth face
or its tooth layer, so the ends stop disagreeing), or move SA4, which is the
net actually in the way. `blockers_of` names the movable-vs-pinned split.

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

### 2026-09-14 (day): the base re-measured, determinism, the unplaced nets

Three findings, in the order they were forced.

1. **The flag-off ladder had regressed at K35+ and nobody had looked.**
   Andy asked for the OLD planner's numbers first. K15 16 / K28 36 matched
   the record; **K35 82 (record 62), K41 103 (91), K51 126 / 2 open (112 /
   1 open)**. The first divergence is round 0's braid-judged cost on the
   untouched bench, i.e. the greedy seed itself, and the cause is the
   night's "shared bug fix" to `select_moves._site_in_lane` (any via, not
   only a dog-bone's). The pair it catches is real -- 9 via-in-pad barrels
   on other nets' row-line B runs at K41 pass 0 -- but the standard
   planner's engine loop catches it too (the DRC gate bans them), and the
   changed seed lost 20 / 12 / 14 vias. Now `SEL_SITE_ANY` (default 0),
   set to 1 by `fanout_from_plan` at import under PLAN_PAGES, from the
   FIRST select. Re-verified (bs1): K35 62 / 1339 segs, K41 91 / 2486, K51
   112 / SBA2 open / 2625 -- the recorded boards, so the flag-off ladder is
   16 / 36 / 62 / 91 / 112 again. The rule this teaches: "K15 and K28 copper identical"
   is not "byte-identical" -- a fix that can change the seed is measured
   on the whole ladder.
2. **The pages-first solve was not reproducible across processes.**
   `_conflicts` bucketed on a `set()` of string-keyed tuples, whose
   iteration order follows the process's hash seed; the exclusions reached
   the CP-SAT in that order, and a `max_deterministic_time` solve that
   stops FEASIBLE lands on a different answer per order. Measured at K41,
   one model (728 + 244 candidates, 595 pairs, 31102 exclusions): obj
   1784.3 / 1726.8 / 1809.0 in three processes, three tooth sets; in ONE
   process three solves identical. Sorted (`sorted(keys, key=repr)`, the
   `learned` set likewise); `PYTHONHASHSEED=1` and `=2` now agree exactly.
   **Every pfB / pfC number of the night was one sample** (pg0 re-ran K28
   at 35 where pfB had 32). The braid's own CP-SAT paths were already
   order-stable (the Modal reproducibility check).
3. **The source-side disagreement was the greedy's UNPLACED nets, not the
   relative head-on test.** `src_diag.py` (scratchpad 38603fde) classifies
   every launch pair the keys and the braid order differently: at K41, 28
   of 40 off pairs involved SA0, SDQ7, SA2, SA9, SWE, SA7 -- nets the
   greedy seed leaves unplaced, so the braid's plan phase on the seed never
   sees them: no corridor, frame keys, `corr = -1`, hence OUTSIDE every
   planarity pair and free to swim in the model at no cost. They were the
   braid's verify swimmers. Fix: for KEYING only, an unplaced net is seeded
   at its cheapest berth (the greedy's own ranking, conflicts ignored); the
   solve still chooses among all its candidates. **K41 launch pairs off:
   40/780 -> 1/706**, and the braid's planner now swims exactly the nets
   the model swims (4 = 4). The line-mate flips (SDQ8 / SDQ15 made joiners
   by SDQ10 chosen downstream on their line) inverted NO pair: a flipped
   tooth keeps its s-order on its side, so the "source-side relative test"
   was never the gap. The one remaining pair (SA8-SA5) is the join block
   ordered by the LEG's s (`_leg_s` jogs), not the tooth's.

With the keys right, the limiter is the SOLVE: at DET 20 the CP-SAT stops
FEASIBLE with 4 swimmers against a bound near 0 (obj 2359.8, bound 1163.6;
each swimmer is 300 in the objective). `PLAN_PAGES_HINT=1` (the seed as a
solution hint) measured WORSE at DET 20 (2659.4 / 5 swimmers) -- the hint
steers the search into the greedy's neighbourhood -- and is off. The
deterministic-time sweep at K41 (one process; `det_sweep.py`, scratchpad
38603fde; the standalone first solve, seed = the greedy, unplaced nets
keyed at their cheapest berth):

| DET | hint | status | obj | bound | wall | model swimmers | braid swims on it |
|---|---|---|---|---|---|---|---|
| 20 | 0 | FEASIBLE | 2359.8 | 1163.6 | 12.6 s | 4 | 4 (the same four) |
| 20 | 1 | FEASIBLE | 2659.4 | 1140.5 | 13.6 s | 5 | 5 |
| 40 | 0 | FEASIBLE | 2036.6 | 1440.5 | 27.1 s | 3 | 3 |
| 40 | 1 | FEASIBLE | **1777.4** | 1440.5 | 26.1 s | 2 | 3 |
| 80 | 0 | FEASIBLE | 1841.7 | 1440.5 | 50.4 s | 2 | 5 (model vias 66) |
| 80 | 1 | FEASIBLE | 1777.4 | 1440.5 | 52.5 s | 2 | 3 |

So the bound proves at least ONE swimmer (1440 = 1140 + 300) and the search
finds 2 given 40 det-seconds; 80 adds nothing; the hint is worth a swimmer
at 40 and costs one at 20 -- local optima, not a trend. Arms on the ladder:
pg1 = DET 20 no hint (the night's budget), pg2 = DET 40 + hint, pg3 = DET
40 no hint. The fanout stage's wall time is the other axis (edict 2).

**Ladder, 2026-09-14 (all 0 DRC; vias / open, wall s of the whole chain
stage pair; base = flag off after the gate, timed alone at K35+; human =
`via_census` of the original):**

| K | base (bs1) | pg1: DET 20, no hint | pg2: DET 40 + hint (= the defaults now) | pg3: DET 40 | pg4: DET 20 + seed crossing test | human |
|---|---|---|---|---|---|---|
| 15 | 16, 12 s | 16, 12 s | 16, 23 s | 16, 22 s | 16 | 22 |
| 28 | 36, 28 s | **34**, 28 s | **34**, 37 s | 34, 29 s | 34 | 46 |
| 35 | 62, 53 s | 65, 56 s | 65, 58 s | 65, 54 s | 76 | 58 |
| 41 | 91, 128 s | 87, 61 s | **79**, 92 s | 81, 75 s | 89 / 2 open | 70 |
| 51 | 112 / SBA2 open, 241 s | 125 / 0 open, 123 s | **115 / 0 open**, 136 s | 106 / SA2 open, 147 s | 131 / SDQ1 open | 85 |

pg4 (`PLAN_PAGES_XING_SEED=1`, the row-vs-column crossing test for the
seed select as well as the re-plans) loses everywhere it matters (K35 +11,
two open at K41, one at K51); pg5 (the same on top of pg2's stack) gives
16 / 34 / **60** / 85 / 116 with THREE open at K51 -- one bench up (K35
60, below the base's 62 and two above the human), K41 +6, K51 broken.
Not a default by edict 3. Off, kept as the record. **`PLAN_PAGES=1`
alone now runs pg2's stack** (`PLAN_PAGES_DET` 40, `PLAN_PAGES_HINT` 1).

pg3 (DET 40, no hint) says the budget is the main effect (K41 87 -> 81)
and the hint is worth two more vias at K41 and COMPLETION at K51 (106 with
SA2 open against pg2's 115 complete). pg2 (DET 40 + hint) is the arm to beat: K41 **79 clean is the best K41
this chain has produced** (the recorded best was 88; 84 with one open),
K51 115 complete, K28 34 below the base, K15 equal, K35 +3 -- within the
time budget (92 s at K41, 136 s at K51, both under the base's own). K41
histogram 0:8 2:29 4:2 5:1 8:1 -- 37 of 41 nets at two vias or fewer, the
human 41 of 41; K51 0:9 1:1 2:23 4:11 6:4 with the model itself swimming
11 (over the two-page capacity). pg1 is the night's stack made
reproducible plus the unplaced-net keying:
below the base at K28 / K41, complete at K51 (the recorded plain K51 was
99 / SA1 open, the record complete 107), K15 equal, K35 +3, and about
HALF the base's wall time at K41 / K51 (the planner replaces the
realize-confirm rounds the paper refine used to spend). At K51 the model
itself swims 10-12 nets (the plan is over the two-page capacity, as
measured on 2026-09-12) and the braid 10-14.

**Where the vias are (`weave_census.py` / `via_where.py`, scratchpad
38603fde), K41:**

| board | vias | via-in-pad | dog-bone | weave pairs | other (dives) | per-net histogram |
|---|---|---|---|---|---|---|
| pg1 | 87 | 21 | 23 | 4 (8 vias) | 35 | 0:8 2:22 3:1 4:10 |
| base | 91 | 20 | 22 | 5 (10) | 39 | 0:7 2:24 4:8 5:1 6:1 |
| human | 70 | **0** | 29 | **0** | 41 | 0:6 **2:35** |

By region (source x<128 / corridor / destination x>132): pg1 25 / 11 / 51,
base 14 / 13 / 64, human 14 / 5 / 51. So the destination now costs what the
human pays; the remaining 17 are 11 at the SOURCE (the planner's moved
teeth are via-in-pad teeth, 1 via each, on top of the bench's F stubs) and
6 in the corridor (weaves: a lane crossed BY LAYER by a side exit's leg
pays 2). The human has no net over 2 because every net is F at one end and
pays its dive + dog-bone at the other; ours ship 11 nets at 3-4 because a
via-in-pad tooth (1) + dive (1) + dog-bone (1) + a weave or a leg crossing
is 4 -- the model prices the first three and not the fourth. That leg
crossing term is the next lever.

## Settled -- do not re-run these

| arm | verdict |
|---|---|
| `BRAID_SOLVER=cpsat` (plain solves) | **never** -- K41 98. `BRAID_ALT_SOLVER=cpsat` (choice solves) is the good one |
| `BRAID_VIA_ROOM_REFUSED=2` | breaks K51 (3 open / 30 DRC) on the OLD 137-via baseline. **Re-measured 2026-09-13 on the 99-via `xing` board: bit-identical to the base -- INERT, not harmful, in that regime.** Read the verdict WITH its board; this one is congestion-dependent. Default 0 |
| `BRAID_ATTEMPTS=10` / `BRAID_BUDGET_X=8` | on K51's refused net: `ATTEMPTS=10` bit-identical, `BUDGET_X=8` gives 97 vias but **2 open** instead of 1. More search does not close a lane with no room; it moves which lane refuses |
| `SEL_XLAYER=1` | crossings WORSE (K41 196 -> 301) -- but that arm ran through a `zip` desync pairing trial legs with stale nets, so the verdict is **not evidence**; unmeasured |
| `DST_XING` per-candidate | pairwise deltas are not additive; the pairwise MILP form fixes the bug and still beats nothing |
| `DST_XING_SCREEN` | helps nothing, alone or with the pattern seed |
| `SEL_CONTEND` / `DST_CONTEND` | fail in both placements, at every weight |
| `DST_WALK_OFF` (off-array walk) | built, general, loses |
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
| `BRAID_LAY_ORDER=xing` | **the largest single-knob win measured, and slack-dependent**: K51 112->99, K41 91->97. Not a default -- an arm, like the pattern seed. See TODO 10 |
| `SF_ESC_W` (0, 0.25, 0.5, 0.75) | all bit-identical at K51; `=10` gives 142/3-open, so the knob is LIVE and the nulls are real. The escape term does not discriminate between K51's candidates. See TODO 1b |
| `SRC_REFAN_JOINT=1` | bit-identical at K51 -- the recorded "only pays at K51" does not survive a chain run |
| `replan.py --mode=rebraid` | inert at K41 and K51; `incremental` wins or ties. The candidates it was meant to unlock (logged "unjudged ... would need the full braid") did not materialise into a better board |
| `SF_SWIM_MODEL=1 SWIM_CHANGES=1` on the CLOUD (NOT COMMITTED) | the per-lane model, tested on the benches where these effects live. **K44**: with `SF_KEY_COST` it recovers 6 of the 14 (112->**106**) but never reaches the 98 baseline; WITHOUT it, **122 / 1 open against 98 / 0**. **K51**: **105 -> 131**, a 26-via loss against `SF_KEY_COST` alone. Bit-identical at K41/K51 locally. So it is not merely inert -- on net it is HARMFUL, exactly as the predictability measurement predicts: you cannot rank by an estimate whose correlation with the truth is -0.03 |
| a per-swimmer via MODEL of any kind | **do not build another one without a plan-time feature correlating above \|r\|=0.2 with routed vias.** Four families tested null over 97-98 swimmers / 5 boards: crossings, a learned per-net prior, geometry, and ROOM. `room_probe.py` is the instrument |
| `SF_KEY_COST=1` (NOT COMMITTED) | the residue judge returned `k = (len(res), f)`, so swimmer COUNT was the PRIMARY key and the via cost `f` only a tie-break -- it minimised swimmers, not vias. Keying on `(0, f)` instead is a **big win at the top and a real loss in the middle**: K28 38->36, K35 70->62, K47 120/4open->114/0open, K51 129/1open->**105/0open**, K15 and K41 unchanged -- but **K44 98->112**. The K44 regression is NOT bench noise: 3 identical containers per arm gave spread 0 on both (98,98,98 vs 112,112,112). Not clearly better across the ladder, so NOT committed. The finding that stands is the DIAGNOSIS -- the judge was never pricing vias -- and `SF_KEY_COST` is the instrument that proved it |
| `group_pages`, `plan_nest`, `improve_k`, `channel_shift` | dropped with take4 (bundle: `~/Downloads/bus/bus622-take4.bundle`) |

Opt-in and kept, all present here: `replan.py` (the route as the judge),
`DST_WALK=k`, `SRC_CLIMB=k`, `pack.py`. The re-berth backstop was a take4
file and is in the bundle only.

## TODO

**Priorities after 2026-09-13, highest first.** The via-count work on the
PLAN side is closed (item 1d); what is left is completion and the search's
acceptance rule.

1. **Close SA1** (item 2) -- K51's 91-via board is invalid for one net and
   nothing in the search budget or via-room family touches it. The untried
   lever is the ASK (its berth face / tooth layer, so the ends stop
   disagreeing on layer) or moving SA4, the net actually in the way.
2. **`SF_ACCEPT_MARGIN`** (item 1c) -- the most promising UNMEASURED knob in
   this file, now that three separate regularisers have each been found to
   be load-bearing by removing them.
3. **`BRAID_LAY_ORDER=xing` as a slack-GATED arm** (item 10) -- the largest
   single-knob win measured (K51 -13) and a loss where there is slack
   (K41 +6), exactly like the pattern seed.
4. Everything below, in the order written.


1. **The FLOOR gap, and the instrument that measures it honestly.**
   (The original framing here -- "every lane's DP floor is 0 or 2, never
   more, 123 of 123", with a prize of K41 -8 / K51 -26 for bringing every
   violator down to 2 -- was REFUTED on 2026-09-12 and has been removed.
   It was an artefact of the circular per-net instrument, and acting on it
   would have walked the optimiser AWAY from the optimum: at K35 the joint
   optimum takes four lanes from 2 down to 0 and pays ONE lane up to 4,
   which is 4 vias cheaper. What follows is what survived.)

   **What `joint_floor.py` established.** "0 or 2, never more" is a
   property of the CIRCULAR per-net instrument. Under the non-circular
   joint floor the human has exactly ONE lane above 2 on every board:

   | | routed | per-net floor | JOINT floor |
   |---|---|---|---|
   | human K35 | -- | 2x27, 0x8 | 2x23, **4x1**, 0x11 |
   | human K41 | -- | 2x32, 0x9 | 2x28, **4x1**, 0x12 |
   | human K51 | -- | 2x37, 0x10 | 2x32, **6x1**, 0x14 |
   | ours K35 (76 v) | 2x16, 4x8, 6x2, 0x9 | 2x17, 4x7, 6x2, 0x9 | 2x21, **4x6**, 0x8 |

   The cheapest assignment is NOT the one where every lane is 0 or 2: at
   K35 the joint optimum takes four lanes from 2 down to 0 and pays ONE
   lane up to 4, which is 4 vias cheaper (54 -> 50). So **"bring every
   violator down to 2" is not the objective** -- a violator can be the
   thing that buys the zeros, and an optimiser told to eliminate them
   would walk away from the optimum.
   What DOES survive, and is the finding worth keeping, is the
   COMPARATIVE form: under the same non-circular instrument the human
   carries **one** lane above 2 and we carry **six**. The separation is
   real; the absolute "never more than 2" was the instrument talking.

   **CAVEAT, from a third audit the same day, and it matters: the
   per-net DP floor is CIRCULAR.** `ledger_cal._dp` prices each net with
   every OTHER net pinned at its ACTUAL layer, so on a badly realized
   board the partners alternate and each net's "floor" alternates with
   them -- the instrument moves with the thing it is meant to be
   independent of. The non-circular quantity is the JOINT floor: over
   the same fixed paths, require opposite layers at every crossing, pin
   the pad layers, minimise total changes (a parity system plus a
   max-cut; 2-85 s per board, exact). It disagrees where it matters:

   | board | routed | per-net floor | JOINT floor | joint slack |
   |---|---|---|---|---|
   | human K35 | 58 | 54 | **50** | 8 |
   | ours K35 (the 58-via board) | 58 | 54 | **50** | 8 |
   | human K41 | 70 | 64 | **60** | 10 |
   | ours K41 (76) | 76 | 72 | **70** | 6 |
   | ours K41 (131) | 130 | 108 | **88** | **42** |

   So at K35 our 58-via board and the human's are STRUCTURALLY
   IDENTICAL, the K41 floor gap is 8 rather than 12, and -- the part
   that bites -- **the per-net floor is 20-40 vias loose on exactly the
   boards the search walks past and rejects.** "Slack is ~6 on every
   board, so realization ideas are capped at 6" is true of the boards we
   SHIP and false of the ones we REJECT, and telling those apart is the
   search's whole job. Build `joint_floor.py` before scoring plans by
   any floor.
   **BUILT 2026-09-12: `joint_floor.py`.** One MILP over the FIXED paths:
   a binary per crossing SITE (the layer there), the equality
   `y[m,j] + y[o,k] = 1` at every crossing, both pad layers pinned, and
   the changes along each path as `d >= |difference|` pairs. Exact, and
   small -- K51's 338 crossings give ~700 binaries and HiGHS closes it in
   seconds. It reuses `ledger_cal`'s own `Path` and `_cross_pair`, and
   takes its nets from `coherent_nets` WITHOUT `--board` exactly as
   `ledger_cal` does, so both instruments run over one set (passing the
   routed output instead gave 10 paths of 35).
   **It reproduces the audit's table exactly, by a different method** --
   the audit derived these with a parity system plus a max-cut, this is a
   MILP, and they agree on every published row:

   | board | routed | per-net | JOINT | joint slack |
   |---|---|---|---|---|
   | human K35 | 58 | 54 | **50** | 8 |
   | human K41 | 70 | 64 | **60** | 10 |
   | human K51 | 80 | 74 | **70** | 10 |
   | ours K35 (the local 76-via control) | 76 | 74 | **66** | 10 |

   Note the last row against the first: our 76-via board floors at 66
   where the human's 58-via board floors at 50. The per-net floor said 74
   against 54 -- it hid 8 vias of our headroom and 4 of theirs. Infeasible
   is a first-class answer (an odd cycle in the parity system means these
   paths have no two-layer realization at all) and is reported, not
   raised.
1b. **The judge's ESCAPE term is anti-informative -- drop it from the
   comparator.** `judge_by_braid` returns `sum(pred) + ride`, where the
   escape half is `tooth_vias + m.vias + ride/VIA_MM`. Measured over the
   distinct plans on disk, pairwise rank agreement with the ROUTED via
   count:

   | statistic | K41 | K35 |
   |---|---|---|
   | `pred` (the judge as it stands) | 59% | 51% |
   | its CORRIDOR half alone | 62% | **64%** |
   | its ESCAPE half alone | 53% | **41%** |
   | LIS of launch->target alone | **64%** | **76%** |

   The judge is worse than its own corridor half at both K, because at
   K35 `pearson(esc, LIS) = +0.57` while `pearson(esc, routed) = -0.34`:
   **a plan that spends more escape vias has a longer crossing-free
   chain and routes BETTER** -- the human's trade, a dogbone at the ball
   to buy the order -- and the judge charges it +1 per via. Sweeping
   `corridor + w*escape` at K35 goes 64% (w=0) to 51% (w=1). It is a
   SIGN error, not a scale error, which is why the swimmer-price work
   (a LEVEL error) barely moved rank.
   Ship LIS as a GUARD on acceptance, never as a maximand -- the settled
   table is full of correlations that became objectives and lost.
   **BUILT 2026-09-12: `SF_ESC_W`** (`fanout_from_plan`), the weight on
   the escape half. 1.0 is the judge exactly as it has always been and is
   the default; 0.0 is the corridor half alone. The split is exact --
   `vias_from_pages` emits `tooth_vias + cross + changes + m.vias` per
   net, so the escape half is the tooth and berth vias plus the ride and
   everything else is corridor, cross-corridor dives included. The LIS
   acceptance GUARD is NOT built.
   **MEASURED 2026-09-13 at K51, and it buys nothing there.** `SF_ESC_W` at
   0, 0.25, 0.5 and 0.75 are ALL bit-identical to the default (112 vias,
   1 open, 2625 segs) -- and `SF_ESC_W=10` gives **142 vias / 3 open**, so
   the knob is demonstrably LIVE and the nulls are real. Down-weighting the
   escape half cannot help at K51 because the term does not DISCRIMINATE
   between the candidates the search sees there; only up-weighting moves the
   board, and that direction is harmful. The rank-agreement result was
   measured over plans ON DISK at K35/K41 and does not transfer to K51's
   search. Unmeasured at K35/K41 through a chain.
   **DEFECT FIXED the same day: the RIDE was inside the escape weight.**
   `judged_cost` returned `cor + SF_ESC_W * (esc + ride)`, so `SF_ESC_W=0`
   also made the CORRIDOR LENGTH free -- the one term already under-priced.
   Split out as **`SF_RIDE_W`** (NOT COMMITTED; default 1, and the default path is verified
   numerically identical over 2000 random cases). An escape via sits at the
   pad in room dedicated to that ball; a corridor via takes room IN the
   channel and pushes its neighbours round it, so escape SHOULD cost less
   than corridor -- but the corridor LENGTH has no reason to scale with it.

1c. **The search accepts at 1e-6 and takes 142 judged REGRESSIONS.**
   965 accepted moves across the K35/K41 logs, median improvement 2.00
   judged vias, 527 of 823 under 3; 142 accepted with the judged cost
   RISING because the residue count fell. A comparator right 51-64% of
   the time on a 2-via difference, accepting at 1e-6, is a random walk
   with a drift -- and that is why removing the wall clock cost K35
   58->66: **the clock was an early stop, and an early stop is a crude
   regulariser.** Replace it with an acceptance MARGIN (work-free,
   deterministic, satisfies the no-clocks rule), calibrated to the
   comparator's measured resolution. A margin large enough to accept
   nothing reproduces the pre-search plan exactly -- a free control.
   **BUILT 2026-09-12: `SF_ACCEPT_MARGIN`** (`fanout_from_plan
   .accept_key`, wired into all three key-tuple acceptance sites). 0 is
   off and is the default -- the plain `k1 < k0` tuple compare, verified
   inert. Above 0 a move must win by the margin on the judged cost, and a
   residue drop stops trumping a cost rise of any size: it may cost at
   most the margin. **Still UNMEASURED, and after 2026-09-13 it is the most
   promising unmeasured knob in this file.** Three separate regularisers
   have now been found by accident, each costing vias when removed or
   loosened: the wall clock (deleting it cost K35 58->66), `CPSAT_SCALE`'s
   coarse rounding (raising it to 1e6 cost K51 105->127), and the swimmer
   COUNT itself (`SF_KEY_COST` cost K44 98->112, and the K44 dumps show the
   inversions are decided on gaps of 0.875-1.613 predicted vias). A
   comparator with NO resolution on vias -- it scored a 98-via and a 112-via
   board at the same 73.0 -- accepting at 1e-6 is the disease all three are
   treating. `SF_ACCEPT_MARGIN` is the principled, deterministic form of the
   same medicine and nobody has run it.

1d. **The swimmer count is a closed form, and the human has MORE.**
   `n - (lambda1 + lambda2)` of the RSK shape of the launch->target
   permutation is 12 on the K41 bench; the level-5 MILP's residue set is
   12. Microseconds, no solver, no board. And reconstructing the human's
   permutation from its copper: human K41 residue 14 against our 13.
   It carries MORE structurally unpageable lanes and routes 70 to our
   80. Repricing or minimising swimmers is EXHAUSTED -- the count is a
   symptom of tangle in our own generator's distribution, not a cost.
   (Also measured: judging at node budgets 1, 5, 15, 30, 100, 400 gives
   an identical answer every time, so "more search is worse" is real
   surrogate bias, not solver jitter.)
   **CLOSED 2026-09-13. "Repricing swimmers is exhausted" is now PROVEN,
   four independent ways, and the reason is stronger than "it is a
   symptom": a swimmer's via cost is NOT PREDICTABLE AT PLAN TIME.**
   See "A swimmer's via cost is NOT PREDICTABLE from the plan" above --
   crossing structure, a learned per-net prior, plan geometry and ROOM all
   return |r| <= 0.13 over 97-98 swimmers on 5 boards. Every knob in the
   family measured inert or harmful on a chain: `SWIM_PRICE` (3 and 3.5,
   cloud K44 both 112), `SF_SWIM_MODEL`+`SWIM_CHANGES` (bit-identical at
   K41 and K51), `SF_ESC_W` (bit-identical at four values). **Do not add
   another per-swimmer cost term without first showing a plan-time feature
   that correlates with routed vias above |r| = 0.2** -- `room_probe.py`
   is the instrument and takes fanout/routed board pairs.

2. **K51 COMPLETION is THE open problem, and it is now the only thing
   between us and a K51 record.** Two arms reach a low via count and both
   ship ONE open net:
   - `xing` + replan: **91 vias, SA1 open** (2026-09-13, the best line)
   - `DST_SEED=pattern`: 104 vias against the baseline's 137, chain 25 /
     slack +3, but **2 open** (SDQ12, SDQ5)

   An open net UNDER-counts vias, so none of these numbers is comparable to
   the 107 record until it closes -- **closing one net is worth more than
   any further via-count work at K51.**
   **SA1 is fully diagnosed -- see "SA1: why K51's last net is open" above.**
   Short form: the lane was never laid (only its two stubs exist), it is
   walled at both ends, its ends are on OPPOSITE layers so it must dive in a
   corridor with no room, and the tail is exhausted (3 rescues, rips,
   last_call). Search budget and via room BOTH measured inert on it
   (`BRAID_VIA_ROOM_REFUSED=2` and `BRAID_ATTEMPTS=10` are bit-identical;
   `BRAID_BUDGET_X=8` just moves the refusal to two other nets). The
   untried lever is changing what SA1 is ASKED for, or moving SA4.
3. **The pattern seed loses at K35/K41** (69 vs 58, 88 vs 80) where
   there is capacity to spare. Either gate it on slack, or find what it
   gives up when it is not needed.
4. **The berth menu is the binding constraint, and the fix is ROW
   pruning -- not column generation.** Measured on the joint arm's own
   K41 instance:
   - Each net has ~19 distinct berth geometries (median; 7-31) across
     ~4 faces. At `CANDS=4` the one-per-face guarantee consumes ALL FOUR
     slots for 28 of 41 nets, so the `vias + ride` ranking never chooses
     WHICH berth -- only one representative per face. That is why
     `DST_XING_SCREEN` beat nothing: a better ranking still returns four
     representatives. **The screen is not the lever; the cap is.**
   - The blow-up is ROWS, not columns: plain -> cap 4 grows variables
     7.9x and rows **48x**. And **77% of the proximity rows at cap 4
     (88% at cap 8) are candidate-vs-candidate** -- pairs that can never
     both be chosen, since at most one candidate per net is taken. Hard
     compatibility is already carried separately by `alt_excl`.
   - Dropping those rows: cap 4 156k -> **48k**, cap 8 527k -> **95k**,
     the WHOLE 26-berth menu -> 255k. **Pruned cap 8 is 40% SMALLER than
     the unpruned cap 4 that ships today.** Restore the dropped rows
     lazily for the chosen set (bounded by the plain instance's ~2k
     pairs, i.e. 1-4% growth per round).
   - **BUILT 2026-09-12: `BRAID_L5_ALT_PRUNE`** (`braid._l5_build`, both
     proximity emission sites), default off. **It is a RELAXATION, and
     the claim above that such a pair "can never both be chosen" is
     WRONG** -- the exactly-one row is PER NET, so net A's candidate and
     net B's candidate are routinely chosen together and the row between
     them can bind. What the prune gives up is precisely the
     mover-vs-mover interaction, the same blind spot that made the
     per-candidate crossing term non-additive at K51. The model can
     therefore return a schedule that is infeasible once both movers are
     seated; the chosen set is re-judged by the full judge and laid by
     the real router, so the cost is a worse route, not a wrong board.
     The LAZY RESTORE is NOT built. Arms staged in `arms.prune.json`.
   - **Column generation is REFUTED for this model**, by this repo's own
     solver study. The pricing subproblem is a per-lane chain DP, which
     has the INTEGRALITY PROPERTY, so the Dantzig-Wolfe master's bound
     equals the compact LP bound -- the same worthless 47.75 against an
     integer 102.62 that killed the Lagrangian arm. The claim written
     here on 2026-09-12 ("a DW master's bound is generally strictly
     stronger, so the usual objection does not apply") is FALSE here.
     And a bound is not what is missing: CP-SAT already PROVES these
     instances optimal. Model size is the problem, and that is row
     generation, not column generation.
5. **Solver budget against quality.** CP-SAT proves the 550k-row cap-8
   instances optimal in 180-310 s, which is 7x over the time edict.
   Measure the incumbent at a short `BRAID_CPSAT_DET` before buying any
   approximate solver. `BRAID_CPSAT_REPAIR` (hint repair) is built and
   unmeasured.
   **FOUND 2026-09-12, and it is a defect, not a knob: `CPSAT_SCALE` was
   destroying the level-5 tie-break.** CP-SAT takes integer objective
   coefficients, so the float objective is multiplied by `CPSAT_SCALE`
   (10000) and rounded. An up/dn costs `1.0 + 1e-4 * u_` with u_ in
   [0,1] -- and `1e-4 * u_ * 10000 = u_`, which rounds to 0 below 0.5 and
   1 above. The tie-break exists to break the symmetry "between the
   twenty equal places a dive could sit, which is what the
   branch-and-bound was grinding on", and under CP-SAT it survived as ONE
   coarse step. At 1e6 it is 100 levels. This is on the hot path: every
   cloud arm runs `BRAID_ALT_SOLVER=cpsat`. Exposed as
   `BRAID_CPSAT_SCALE` at its old value, because raising it changes the
   integer objective and therefore possibly the answer -- a knob to
   measure, not a silent fix.
   Also **`BRAID_L5_AUX_CONT`**: `z` (the DST_XING pair) and `iv` (the
   island) are implied integral at any optimum -- `z` sits in exactly one
   row `z >= ya + yb - 1` with positive cost, `iv` only in
   `st + w + iv (- y) >= c` rows with every other term binary -- so
   declaring them continuous removes branching candidates without moving
   the optimal value. It can still change WHICH optimum comes back, so it
   is off by default. Inert under CP-SAT, which builds every variable as
   a Bool regardless.
6. **`DST_ASK_BAN`** -- an ask the engine answered a layer/kind/face
   away is not asked again. Built, default off. First measurement is
   negative (K35 66 against 58); finish the ladder before deciding.
7. **The corpus A/B for the production changes, then the PR to main.**
   Audited 2026-09-12; the list was FIVE and is really this. Everything
   here changes copper on boards that have nothing to do with this
   chain, and a change that moves copper but is not on the list is the
   one nobody thinks to measure.
   - **`KICAD_SEG_DIST_EXACT`** (`single_ended_routing`) -- default ON,
     every route on every board. The maths is verified exact (3,008
     cases against brute force: it is never larger than the truth, max
     deviation 1.1e-16), so it cannot admit close copper. But it tightens
     a boolean gate `d >= clr + w/2` at ~10 sites in `pcb_modification`
     -- smoothing, dangling-bridge repair, re-bend, stub trim,
     castellated retract, via nudge -- so those passes now DECLINE moves
     they used to accept. Fewer sub-clearance grazes, possibly more
     repairs left undone; only the A/B can say which dominates.
   - the pad keep-out's sub-cell offset quantised (`routing_utils`, both
     obstacle stampers, every route)
   - `plane_fill_model` cell rounding -- changes which fill cell a
     pour-direct stub taps, on every board with planes and #678
     pour-served balls. CLAUDE.md: a plane/oracle change cannot be judged
     by one replay pair.
   - `bga_fanout/escape.py` tie-breaks -- escape direction and channel
     choice on EVERY BGA/QFN fanout
   - the `underpad` ordering/quantisation family (`depth()`, `CELL_EPS`,
     the keep-out raster rounding whose own comment measures "244
     boundary cells stamped differently") -- reorders Phase A's claim
     sequence, which decides which ball claims each rim gap, on every
     under-pad fanout
   - `rotate_frame.to_axis_aligned_frame` -- every non-orthogonal BGA/QFN
     board. This one is a FIX (main saw pours and outline in the
     un-rotated frame) but it is a copper-changing fix.
   - **`underpad._follow_plan`'s blocker order -- a DETERMINISM DEFECT,
     found 2026-09-12, patch held at `scratchpad/determinism_blockers.patch`
     (NOT applied: the tree must match the sweeps in flight).**
     `blockers_of` returns a SET of `id(pad)` -- memory addresses -- and
     Python's stable sort therefore breaks depth ties in whatever order the
     allocator produced that run. Measured: two runs of IDENTICAL code on
     the identical board printed `ripped ['SDQ14','SBA0','SDQ6']` and
     `ripped ['SDQ14','SDQ6','SBA0']`. The re-lay order is load-bearing --
     which blocker gets its gap back first decides the copper -- so this is
     a WALL-CLOCK-CLASS defect: same input, different output. It also makes
     a "byte-identical with the flag off" check fail for the wrong reason,
     which cost one false alarm today. The fix ties the break to the pad's
     own identity (`-depth, net_name, pad_number`), a property of the board
     rather than of the process. Audited the siblings: `escaped` and
     `coupled_pairs` are membership/count only and never iterated, and the
     `net_id` sets are board values, so this was the only instance.
   - `_foreign_seg_arr_trust` -- weakens a shared cache's staleness
     digest. The invariant holds today; a future in-window copper edit
     would route against phantom copper with nothing to catch it.
   - the back-side BGA fanned as the front's mirror, and the fanout's
     plan-follow (`escape_dir_hints`, which also owes a GUI call site and
     a `FLAG_PARAMS` entry -- the Class-2 drift CLAUDE.md forbids)
8. **The second bench's in-band gap.** zynq_ad9364 at K28: 0 open, 55
   vias, but 17 of 28 in band. The leg rules were tuned on ONE bench.
   Fix `flow_frame.py turn` first -- the article does not run through
   `chain_k.sh` today.
9. **Poses off the axes.** R30 and R45 break the plan's compass faces
   and need a trigonometric turn of the file.
10. **A better routing order.** Lanes are laid sequentially and every
    refusal the rip repairs is a sequential loss. Most-constrained-first
    is the reference.
    **BUILT 2026-09-12: `BRAID_LAY_ORDER=xing`**, default ''. The
    constrainedness measure is free and general -- how many other lanes
    this one CROSSES, i.e. the pairs whose launch and target ranks
    disagree. Ties keep the target order, so off is byte-identical. Also
    **`BRAID_RIP_VICTIMS` / `BRAID_RIP_DEPTH`**, which were hard-coded 3
    and 1 on `rip_for`'s signature with no way to turn them.
    **MEASURED 2026-09-13, and it is the LARGEST single-knob win on the
    board -- but it is SLACK-DEPENDENT and must not become a default.**

    | bench | off | `xing` | |
    |---|---|---|---|
    | K51 | 112 / 1 open | **99 / 1 open** | **-13** |
    | K41 | 91 / 0 open | 97 / 0 open | **+6, worse** |

    Same shape as the pattern seed (item 3): it pays where the board is over
    capacity and costs where there is slack. It is the front half of the best
    K51 line on record -- `xing` 112->99, then `replan.py` ->91. Gate it on
    slack, or carry it as an ARM, never as the default. K35/K28/K15
    unmeasured.
11. **The packing's purpose is unmeasured.** `BRAID_PACK=1` ships 0 open
    0 DRC with vias unchanged and far fewer segments; nobody has
    measured what the segments buy.
12. **The source is frozen.** Every chain arm runs `SRC_ROUNDS=0`, so
    U1's teeth are the bench's own fanout and the source disagreement
    with the human is an INPUT, not a result. The joint source re-fan is
    built (`SRC_REFAN_JOINT=1`) and only pays at K51, where the blocking
    net is in the run. **MEASURED 2026-09-13 at K51: bit-identical to the
    base (112 / 1 open). The "only pays at K51" claim does not survive a
    chain run.** Note the replan DOES move source ends and they DO stand
    (see the replan section), so the source is not immovable -- what is
    null is this knob.
    **BUILT 2026-09-12: `SRC_EXCHANGE=1`, a PROBE that changes nothing.**
    A tooth is physical copper, so a plan exchanging two nets' launch
    points is not realizable without a re-fan -- and three sessions of
    source arms measured null, which may be the engine or may be the
    idea. The probe asks the cheap half first: under the braid's own
    judge, does ANY pairwise exchange improve the plan? If none does the
    realize build is not worth writing; if some do, the gains name the
    pairs a re-fan should target. Capped at `SRC_EXCHANGE_PAIRS` (60).
    **FIRST READING, K15: `2 of 60 pair(s) improve the judge; SA9<->SCAS
    +2.00, SA7<->SCAS +2.00`.** So the source order is NOT dead on the
    merits -- gains exist and they are worth 2 judged vias, which is the
    median accepted move's size. The null results of three source arms are
    therefore about the REALIZE step, not about the idea. Read the ladder
    arm (`sx`) before drawing anything stronger from one K15.
12b. **MEMORY, measured in situ 2026-09-12.** A K35 joint arm peaks at
    **640 MB** for the whole fanout stage, and the peak is ONE `_alts5`
    call built in two steps:
    - `_l5_build` holds 329,837 row DICTS alive at once -- 434 B/row, a
      7.7x blow-up over the 7 MB CSC they encode -- for **+164 MB in one
      call**. The fix is a flat `(cols, vals, rowptr)` store plus a hash
      dedup key; measured 5-6x smaller, and the COO/CSC it builds is
      bit-identical. NOT BUILT (half a day; `_milp_solve`, `_cpsat_solve`,
      the `n_viol` loop and both dump paths are the consumers).
    - **CP-SAT never returns its arena**, so repeated solves of the SAME
      model ratchet the process: 303 -> 357 -> 441 -> 488 -> 640 MB across
      five. HiGHS on the same harness is flat (82 -> 131 -> 127 -> 130).
      The fix is to run the solve in a short-lived child. NOT BUILT.
    **DONE:** `_milp_solve` builds its COO with `np.repeat`/`np.fromiter`
    instead of three Python lists (+46 MB / 0.31 s -> +11 MB / 0.14 s at
    cap 4, ~-155 MB at cap 8; COO and CSC verified bit-identical), and
    `modal_k.py` asks for `memory=(4096, 12288)` instead of a flat 12 GB
    -- the 12 GB was a guess standing in for a diagnosis of a SIGABRT
    nobody confirmed as an OOM.
    **Ruled out, do not spend time here:** `_PROFILE_MEMO` (34 entries /
    1.1 MB), `_L5_SEED`, the 6 spawn workers (the joint arm never opens
    the pool -- `DST_RESIDUE=3` runs `residue_choice`, which is single
    process), the sharded taut memo (457 MB on disk, 38 MB resident for
    all 256), CP-SAT `num_workers` (4/2/1 -> 674/681/658 MB, no saving,
    and it CHANGES THE ANSWER), row dedup (already exact, 0 duplicates
    left), and the obstacle windows.
    **And a measurement trap for the honesty list: on macOS a memory
    reading taken while the box is loaded is low by up to 2x** -- the
    same CP-SAT solve read 329 MB loaded and 577-693 MB idle, because the
    compressor moves pages out of RSS. CP-SAT peak also has a ~20%
    run-to-run spread on an identical instance, so one peak is not a
    result any more than one K is.

13. **Tooling.** Promote the session probes into `awx/` with a line each
    here; add the flag-off parity gate that the hand check does today.
    **2026-09-13: `room_probe.py` promoted** (does a plan-time feature
    predict a swimmer's vias? -- run it BEFORE building any per-lane cost
    term) and **`modal_k.py` grew `return_board` / `return_files`**, without
    which no cloud-only phenomenon can be diagnosed at all: it returned
    filtered log lines only, so a cloud arm's copper could be counted but
    never looked at, and K44's regression is cloud-only. Still owed: the
    flag-off parity gate. **A trap worth the line: on zsh `env $VARS cmd`
    does NOT word-split**, so `env "A=1 B=2" cmd` sets ONE variable named
    `A` to `1 B=2`. It cost a wasted arm today (and once before); write the
    assignments out, and note the chain failed LOUDLY (`ValueError`) rather
    than silently measuring the wrong thing.
14. ~~Re-express the two `*_TIME` stage knobs in nodes.~~ **WITHDRAWN
    2026-09-12 -- the premise was wrong on both halves.** The node
    budgets ALREADY EXIST and are already the live ones:
    `BRAID_L5_JUDGE_NODES` (30) and `BRAID_L5_ALT_NODES` reach
    `_milp_solve(nodes=)` at every level-5 call site. And the `*_TIME`
    names are not dead either -- `_l5_time` is part of the level-5 MEMO
    KEY, where the separation is load-bearing: a plan-only pass solved
    under the judge's cap must not serve the real attempts (K41: attempt
    0 laid a 3 s incumbent, 84 vias for the 76 the full-cap optimum
    routed). Renaming them would collapse that key onto the lay pass's.
    Attempting the rename duplicated two live constants and left
    `L5_ALT_TIME` undefined at three call sites; reverted. What is true
    is only that they are confusingly named for a value that is a memo
    discriminator and a `min(cap, ...)` argument, never a budget.

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
A/B (TODO 7) before it reaches main. `KICAD_SEG_DIST_EXACT=0` restores
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

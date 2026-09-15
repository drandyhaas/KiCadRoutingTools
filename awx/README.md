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
column.** `BRAID_LAY_ORDER=xing` then `replan.py`. **Its arm is NEITHER of
the two above**: it was `env SF_KEY_COST=1 BRAID_LAY_ORDER=xing bash
chain_k.sh xing51 51` -- the PLAIN defaults (8 source rounds, no exit guard,
no ONE_DIVE, no CP-SAT) plus the uncommitted `SF_KEY_COST`. Verified
2026-09-13 evening by re-braiding its fanout board: the plain env gives its
99 / SA1 copper-IDENTICAL in 119 s, and the joint arm's BRAID_* env over the
same fanout board gives 159 / 2 open. The fanout board and the braid env
must match; a braid-only re-run needs the env the fanout was planned under.

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
> | `BRAID_RIP_ONBOARD` | the phantom-victim fix (2026-09-13 evening): closes K51 SA1 at 3 vias and opens SA11+SDQ11 -- moves the refusal, 97/2 against 99/1; inert at K41 |
> | `BRAID_RIP_SOFT` | rip trials with the victims' old copper PRICED: K51 116/2 against 99/1 -- harmful |
> | `BRAID_RESCUES` | the per-attempt failed-rescue cap (was a hard-coded 3): lifting it is K41 91->100, K51 99/1 -> 104/**5** -- the cap is a load-bearing regulariser |
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

## SA1 corrected: the loss is the VICTIM RE-LAY, and the tail is a capacity wall (2026-09-13 evening)

The section above says the corridor "has no room for this lane's dive at
all". **Its own transcript says otherwise.** In the xing braid's last call:

```
rip for SA1: min-cut probe 3 via(s) crosses ['SBA1', 'SDQ4', 'SA12']
rip ['SBA1']: SA1 still refused
rip ['SBA1', 'SDQ4']: SA1 routed (3 via(s)) but SBA1 lost -- put back
rip ['SBA1', 'SDQ4', 'SA12']: SA1 routed (9 via(s)) but SBA1 lost -- put back
```

SA1 routes at 3 vias the moment SBA1 and SDQ4 are lifted. What fails is
SBA1's RE-LAY, and its nested negotiation was reading a phantom:

```
rip for SBA1: min-cut probe 3 via(s) crosses ['SDQ4', 'SA15', 'SDQ1', 'SDQ13', 'SDQ0', 'SDQM1']
rip ['SDQ4']: SBA1 still refused
rip ['SDQ4', 'SA15']: SBA1 still refused
rip ['SDQ4', 'SA15', 'SDQ1']: SBA1 routed (1 via(s)) but SDQ4 lost -- put back
```

SDQ4 was ALREADY lifted for SA1 -- its copper was off the board -- but
`rip_for` builds its candidates from `out_segs`, which still named it, so
the nested probe priced it as a soft obstacle, the cut set led with it, one
of three victim slots went on a no-op rip, and SDQ4 was re-laid INSIDE
SBA1's negotiation (before SBA1's own victims were back) at depth 0, where
it was lost. **`BRAID_RIP_ONBOARD=1`** (NOT COMMITTED) requires a nested
rip's candidates to have copper on the board. With it SBA1's cut leads
with SDQ1, one rip re-lays SBA1 at 1 via, and the trial closes:
`rip ['SBA1', 'SDQ4']: SA1 routed (3 via(s)); re-laid SBA1 1 -> 1, SDQ4 0 -> 2`.

**And then SA11 and SDQ11 are open.** Every arm on the tail, all braid-only
re-runs of the `xing51` fanout board under its own (plain + xing) env, all
flag-off parity verified copper-identical, all inert at K41 (bs41 base 91/0,
whose four rip trials never lose a victim), times under 2-3 concurrent runs:

| arm | vias | open | s |
|---|---|---|---|
| base | 99 | 1 (SA1) | 119 |
| `BRAID_RIP_DEPTH=2` | 142 | 2 (SA11; **SDQ9 SILENT** -- 6 via-less layer changes, not in the refused list) | 259 |
| `BRAID_RIP_ONBOARD=1` | 97 | 2 (SA11, SDQ11) | 121 |
| `BRAID_RIP_ONBOARD=1 BRAID_RIP_DEPTH=2` | **141** | **0** -- the first COMPLETE board on this line | 173 |
| `BRAID_RIP_SOFT=5` (victims' old copper priced in the trial route, with or without ONBOARD) | 116 | 2 (SDQ11, SDQ15) | 105 |
| `BRAID_RESCUES=99` (the per-attempt failed-rescue cap lifted) | 104 | **5** | 150 |
| `BRAID_BUDGET_X=8` (earlier today) | 97 | 2 (SA11, SDQ11) | |

Read together: **the region SA1 / SA11 / SDQ11 / SBA1 / SDQ4 is over
capacity in this PLAN.** Every fix to the tail's mechanics closes SA1 and
evicts a neighbour -- the same two neighbours as the deeper search budget --
and the only arm that completes negotiates so deeply that it pays 42 vias
for it. The failed trials in every arm have one shape, `X routed (N vias)
but Y lost -- put back`: the refused lane, given its victims' room for
nothing, sprawls through all of it, and a victim has nowhere to go. Pricing
that room (`RIP_SOFT`) makes the refused lane detour instead (SA1 at 7 vias,
SA8 re-laid 1 -> 6), which is worse. So the tail cannot be tuned into a
complete cheap board here; the PLAN must move a berth, which is the
replan's job -- and at the DEFAULT width the replan does not do it either:

| replan (`--rounds=4`, default `--worst=3 --probes=1`, `BRAID_RIP_ONBOARD=1` in its braids) | result |
|---|---|
| from the `ONBOARD` board (97 / SA11+SDQ11) | round 1 KEPT (strip): SDQ11 closed, **100 / SA11 open**; round 2 nothing judged better, stopped (304 s) |
| from the base (99 / SA1) | round 1 KEPT (refan, a source move stood): **95 / SA1 open**; round 2 nothing judged better, stopped (241 s) |

| the WIDE recipe (`--worst=48 --probes=2 --rounds=6`) from the ONBOARD board (`rpQ51`) | round 1 KEPT (refan): SDQ11 closed, **101 / SA11 open** (617 s); round 2 nothing judged better, 8 unjudged moves "would need the full braid", stopped (1005 s in all) |

So on this plan the replan closes SDQ11 for 3-4 vias and cannot close SA11
at either width. The K51 line as it stands: base 99 / SA1 -> phantom fix
97 / SA11+SDQ11 -> replan 100-101 / SA11 -> and the only complete board is
141 / 0 (depth 2). A replan FROM that complete board (`rpR51`, default
width, the way the recorded 137 -> 107 line was made, `ONBOARD` + depth 2 in
its braids) went **141 -> 134 / 0 open** in one kept round (185 s in all;
re-graded 134 / 0 open / 0 DRC) and stopped. A valid complete K51 board on
the xing line, and 27 vias short of the 107 record, so not a line -- the
wide recipe from it is the obvious next step if this line is continued.

Two things the rescue-cap arm teaches. The base logs carry ZERO `rescued at
x4` lines at K41 and K51 -- the first three rescues fail and the cap stops
the rest -- and lifting it rescues 25-27 lanes into WORSE positions (K41
91 -> 100, K51 5 open). **The cap is the fourth accidental regulariser**
after the wall clock, `CPSAT_SCALE` and the swimmer count: a lane forced
through at 4x the budget takes a path the plain budget refused for a reason.
And `BRAID_RIP_DEPTH=2` re-opens the nested-rollback bookkeeping defect the
rip section above describes (SDQ9 shipped open with via-less layer changes
and was not in the refused list) -- not chased, because depth 2 is harmful
anyway, but do not ship a depth-2 board without reading the WARNING line.

## How the human routes the congested region (2026-09-13 evening)

Measured on `~/Downloads/bus/00_human_original.kicad_pcb` (same frame as
the bench: U1 at (120.29,63.93), DU1 at (139.93,64.56) rot 90) against our
`xing` board (`rb1`, 99 / SA1 open), scoped to the K51 net set. Scripts in
the session scratchpad: `human_vs_ours.py`, `group_census.py`,
`xing_census.py`, `layer_census.py`, `berth_kind.py`.

**What is the SAME -- the topology.** Address group (29 nets): two ring
roads round DU1, human N 11 / S 16 / both 2, ours N 7 / S 17 / both 3;
DU1 exit faces human E10 N6 S11 W2, ours E8 N7 S11 W3; layer share human
B58/F42, ours B56/F44; copper under DU1 human B 90 / F 75 mm, ours 94 / 72;
lane crossings human **338** (addr-addr 150, addr-data 135), ours 333
(209 / 77). Data group (18 nets): human enters the west face 9, ours 7;
copper human 309 mm, ours 284 mm (ours is SHORTER -- the human carries
length-matching meanders in the gap, which is how much room they have left).
So the human does not avoid crossings and does not go round where we go
through. The difference is entirely WHERE THE LAYER CHANGES SIT.

**What is DIFFERENT -- the via discipline and the berth kind.**

| | human | ours (rb1) | record (rp6b, 107) |
|---|---|---|---|
| max vias on any net | **2** | 6 | 10 |
| address nets constant-layer (exactly 2 END vias) | **22 of 29** | 11 | 4 |
| address nets with one MID change | 7 | 8 | 14 |
| address nets with 3+ vias | **0** | 7 | 10 |
| data vias / net | **1.22** | 1.89 | -- |
| data nets with 0-1 via (single layer) | 7 | 5 | 6 |
| data nets with 4 vias | 0 | **4** (SDQ0, SDQ11, SDQ15, SDQM1) | 2 |

The four 4-via data nets dive INSIDE the 6.5 mm gap between the arrays:
SDQ0's vias sit at x 128.6, 130.3, 132.0, 133.1 -- four layer changes in
5 mm to thread past other lanes -- where the human's SDQ15 is a pure F
lane with no via at all. That is +10 of the data group's +12.

| DU1 berth kind (47 balls) | human | ours (rb1) | record (rp6b) |
|---|---|---|---|
| edge dog-bone (via 0.7-3.5 mm from the ball, the stub on the ball's layer) | **22** | 5 | 11 |
| between-ball dog-bone (via 0.1-0.7 mm) | **14** | 2 | 1 |
| via-IN-pad | **0** | 16 | 15 |
| bare stub (no via within 6 mm) | 8 | 14 | 15 |
| far via (3.5-6 mm) | 3 | 10 | 5 |

36 of the human's 47 DU1 ends are dog-bones, none via-in-pad. Our fanout
answers the same balls with via-in-pad (16) or a bare stub (14) or a via
3.5-6 mm out (10). A via-in-pad berth forces the lane to arrive on B at the
ball's exact position, through the field; a bare F stub forces an F arrival
on the surface every other stub is on; either way the ONLY place left for
the lane's layer change is the corridor, which is where our excess vias
are. The human's dog-bone puts the change at the array's edge, in room that
belongs to that ball, and the stub direction picks the face. The human's
DU1 dog-bones point NE 11, SW 7, SE 5, N 4, E 4, NW 3 -- every ball gets
its own exit.

**The contested balls, one by one** (SA1 P7, SBA1 N8, SA11 R7, SA4 P8 --
the east half of the array, rows 7-8): human SA1 and SBA1 leave on F 2.8-3.0
mm NORTH to vias just outside the array's top edge, then ride B round the
north ring to a 0.5-0.8 mm dog-bone at U1 -- 2 vias, 31 mm, B 88%. SA11
leaves EAST 1.6 mm to an edge via (its column is the east-most). SA4 goes
WEST 1.25 mm to a between-ball via. Ours: SA1's berth is a bare F stub 2 mm
EAST (`stub_dir [1,0]`) with its tooth a via-in-pad on B at U1, so the
lane's ends disagree on layer and the change has to happen in the corridor
that has no room for it; SA4 is a 47-segment F loop over the north caps.

**What this says to the plan, generally.** (1) The 2-via rule is the
human's whole method: a lane is one layer with a dog-bone at each end, and
the plan should price a corridor dive as what it costs the neighbours, not
as one via. (2) The berth menu's `dogbone` kind is what the human uses 36
times in 47; the judge chooses `via_in_pad` and `surface` (bare stub) for
30 of ours. A plan that reads "via-in-pad = 1 via, cheapest" is pricing the
forced arrival layer at zero. (3) The data group wants a crossing-free
order across the gap (0-2 vias); that is a SOURCE exit-order question, and
the source is frozen in the arms. Nothing here is a face rule or a ref: it
is a berth KIND and a via budget per lane.

## Routing ONE net of a chain board from the GUI / route.py (2026-09-13 evening)

Andy opened `xing51_k51` (99 / SA1 open) in the plugin, selected SA1 alone,
and the log (`scratch_15.txt` in the worktree root) read as if it routed
much more. What it did, and what it found:

- **It routed only SA1** (`MPS Round 1: 1 units: SA1`, `Routing 1
  single-ended net(s)`). Everything else in the log is the production
  router's main-pass RIP of pre-existing copper: 25 nets registered as rip
  candidates, SA14 then SBA1 ripped for SA1, both reroutes failed, the #134
  recovery re-routed the two left ripped, that failed too, and the
  IMPROVEMENT GATE rejected the run ("broke 1, connected 0") and
  **discarded everything -- the board was not changed.**
- **Why the victims could not be re-laid -- a real defect.** A pre-existing
  net is registered with ALL its copper (`route.py` ~2250: every segment
  and via of the net), so the rip removes its fanout ESCAPE too, and the
  reroute then starts from a bare ball inside the BGA field: `Hint: pad
  U1.T18 is a fanout-dropped ball (no escape stub) ... no rip authority or
  retry can reach it`. The braid's `rip_for` keeps the stubs (it re-lays the
  LANE between tooth and berth); `route.py`'s rip cannot, so on a fanned-out
  board every pre-existing rip of a BGA net is unrecoverable. Reproduced
  from the CLI: `route.py ... --nets '/DDR3 16x1/SA1' --rip-existing-nets
  SBA1 SDQ4` (the braid's own victims) rips both and fails, "Restoring 2
  net(s)".
- **The GUI asked for 0.25 mm clearance, the CLI for 0.1, on the same
  project** -- and the chain laid this copper at 0.1, so at 0.25 (used
  0.2193 after descents in the main pass, 0.127 in the recovery pass: the
  run's `JSON_SUMMARY` says `"clearance": 0.25, "min_clearance_used":
  0.2193`) no channel on the board is legal and every ball is "boxed in by
  static obstacles". The bench projects (`fb_t2q_fresh`, every chain output)
  carry a Default net-class clearance of **0.0** (KiCad's "not configured")
  with `min_clearance` 0.0889. The CLI takes base 0.0 and pins it UP to the
  physical 2-layer floor 0.1 (`enforce_fab_floors`, "pinning up to 0.1");
  the GUI's `_effective_geometry_floor` treats 0 as unset and falls back to
  its Min Clearance CONTROL, whose default is `routing_defaults.CLEARANCE`
  0.25. Same project, 0.1 against 0.25: a CLI/GUI parity gap for any board
  whose Default class clearance is 0 -- and the bench IS such a board, so
  the chain's DRC-floor writeback (which should stamp the routed 0.1 into
  the class, #900) is not doing so here either. To repeat the experiment in
  the GUI, check Min Clearance and enter 0.1.
  **FIXED the same evening, on the writer side**: `fix_kicad_drc_settings
  .apply_targets_to_project` read a Default class clearance of 0.0 as
  "already below the target" (its write is lower-only) and never stamped
  it, so `make_bench`'s `fix_project_for_output(clearance=0.1)` reported
  "already consistent" and the 0.0 rode down every step. It now treats a
  declared 0 as UNSET, the rule every reader in that module already
  applies, and stamps `net_class[Default].clearance: None -> 0.1`
  (`tests/test_fix_drc_settings.py` + the `test_530_*` clearance tests
  pass). And the chain no longer bare-copies the project: `braid.write_out`
  and `fanout_from_plan.copy_pro` call `fix_project_for_output` with the
  SPEC (`braid.SPEC_CLEARANCE` 0.1 / track 0.1 / via 0.25 / 0.15 -- not
  `CLEAR` 0.105, the router's private margin over the spec, which the
  first cut stamped); `AWX_STAMP_PRO=0` is the flag-off control. The
  tracked bench project `fb_t2q_fresh.kicad_pro` still carries the 0.0 (not
  re-stamped, to leave the bench article as recorded); every chain output
  from it now records 0.1, and the copy in `~/Downloads/bus` was re-stamped
  by hand. Copper is unaffected (both engines route from their own
  cfg, never from the project) -- verified by re-braiding `xing51_fo_k51`
  (copper-identical to `rb1`, output project Default 0.1) and by a K15 chain
  pair with the stamp on and off. The READER divergence (an unset class
  still resolves to 0.1 on the CLI and 0.25 in the GUI on any board this
  tool has not written) is **issue #966**.
- **Even at 0.1 the production router does not route SA1** (`Boxed in at
  this geometry after 862 iterations (grid 0.1, clearance 0.1, track 0.1)`),
  which agrees with the braid: SA1 needs SBA1 and SDQ4 lifted AND re-laid,
  the negotiation `route.py` cannot do.

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

### 2026-09-14 (session 3): why not two vias, and the exit-leg term

Andy's two asks: build the leg-crossing term, and understand why not every
net at K41 fits on two pages at two vias when the human's do. Bench pg2
(the committed defaults): K41 79 / 0 open, human 70.

**The gap is four nets.** Per net against the human (`struct_table.py`,
`net_vias.py`, scratchpad e78852db): ours 79 = 8 nets at 0, 29 at 2, and
SCKE1 8 / SBA1 5 / SA8 4 / SA2 4 (+13 against the human's 2 each); five
DQ nets at 2 where the human is at 0 (+10); seven nets at 0 where the
human pays 2 (-14). Fix the four at 2 and K41 is 66. What each one is:
SA2 is a page-B joiner crossed by the B exit legs of eight outer joiners
that exit earlier, so it dives under them and returns (+2, priced 0);
SCKE1's 'up' via-in-pad berth sits in the north strip between DU1 and the
passives, which carries 11 'up' berths and 4 far-face lanes -- refused in
band, the last call laid a 4-via staircase through the top block (the
human goes round the SOUTH, dives at (138.1, 71.8), rides B north under
the whole array and dog-bones just north of J9: the face with ROOM);
SBA1's tooth was bought on U1's WEST face (against the flow, 11 mm under
U1 at 3 + 2 x 11 = 25 against a 300 swimmer), which the braid cannot join
to the corridor -- its own 34 mm spine, refused every attempt; SA8 is a
south-side joiner given a north-side far-face berth (its ball is in the
north half) and swims the whole bundle.

**Model 46, braid plan 82, copper 79 (`plan_vias.py`).** The pages-first
objective counts the end vias (28) and the end-page mismatches (18). The
braid's own planner on the same plan implies 54 lane changes. The 36 the
model never saw: the JOINERS' exit legs (the joined block keeps the
source's join order, last joiner innermost, so a joiner's leg crosses
every port and every inner joiner exiting later; `place_and_decide` flips
such a leg to B and pays a corner + a tip: SA5 SBA0 SDQ6 SODT0 SRAS SWE at
2 each, +12, all priced 0 as F/F/F), the page-B lanes those B legs cross
(SA2, SCS1, +4), and the swimmers' changes (+22, of which the router paid
9). The human's joiners cost 2 as well ('DD' / 'dD': F corridor, dive at
or under DU1, B ride, surface, F in), so the joiners are at parity; the
mispricing hurts by making the model F-HUNGRY for them (0 on F, 1-2 on B,
when both are 2) and blind to what their legs cross.

**The human is not a two-page router.** `two_chain.py` on the human's
ends (hbn_k41) needs 6 swimmers under our keys (the braid 10: SDQM1 SDQ9
SA1 SDQ11 SDQ12 SDQ14 SDQ0 SA4 SCKE1 SCKE0), and the human routes all ten
at 2 vias: 'Cd' -- F tooth, ONE dive mid-corridor, B to a dog-bone. A
two-page lane pays its mismatch at an end; the human puts the change
where the crossings say. Under-array copper is not the difference (B
under DU1 81 mm on both boards); the perimeter is (human B 185 / F 97 mm,
ours 108 / 143).

**Proven: our menus need two swimmers.** `PLAN_PAGES_MAX_SWIM=k` (a probe
knob) on the bench's instance (728 berth + 244 tooth candidates, 820
pairs): 0 swimmers INFEASIBLE in 5 s, 1 in 8 s, 2 FEASIBLE. On the chain's
own seed the uncapped DET-40 solve stops at 4 swimmers / obj 2309 while
the same model capped at 2 finds obj 1761 -- a search failure, not a
capacity. `PLAN_PAGES_LEX=1` (phase A minimises the swimmer count alone,
proves 2 OPTIMAL in 10 s, caps the main solve and hints it) finds a
2-swimmer plan the braid then swims 6 on: 12 teeth moved, and the keys
built on the seed no longer hold. Off.

**The exit-leg term, `PLAN_PAGES_LEG=1`.** A leg-layer bool per net (the
page for a head-on berth), corner + tip in place of the berth mismatch,
and per same-corridor pair with a shared side the crossing reified from
the keys (same side, |T| inner, exit s earlier by 0.05 mm) -- only a net
with a joiner tooth on offer owns a crossing leg -- with a crossed lane's
dive charged once however many legs cross it. Two things the build
taught: the term in ONE phase stops at 7 swimmers / obj 3318 where the
leg-free solve has 2 / 1777 (every leg-free answer is feasible under the
term at about 1830: the bigger model searches worse), and the leg
constraints built up front degrade the leg-free phase on their own (obj
2662 for 1777), so phase 1 is the pristine model and the leg variables
are added to it afterwards, phase 1's answer the hint
(`PLAN_PAGES_LEG_DET`). `PLAN_PAGES_SRC_AWAY=0` drops tooth candidates
pointing against the spine's launch direction (SBA1's west tooth).

Ladder (vias / open, PLAN_PAGES=1 + the arm; pg2 = the defaults; lg1 =
the term with the first crossed-lane rule, lg3 / lg4 = the corrected one):

| K | pg2 | lg1: LEG=1 (old rule) | lg3: LEG=1 | aw0: SRC_AWAY=0 | lg4: LEG=1 + SRC_AWAY=0 | human |
|---|---|---|---|---|---|---|
| 15 | 16 | 16 | 16 | 16 | 16 | 22 |
| 28 | 34 | 40 | 34 | 35 | 34 | 46 |
| 35 | 65 | 62 | 62 | 62 | **60** | 58 |
| 41 | 79 | 75 | 79 (copper = pg2) | **72** | **72** | 70 |
| 51 | 115 | 120 + SDQ11 open | 117 + SDQ11 open | 113 + SA10 open | 96 + 4 open | 85 |

K41 wall (fanout + braid): pg2 92 s, lg1 147 s, lg3 126 s, aw0 73 s, lg4
114 s.

lg1's K41 75 was the best clean K41 this chain had produced (the four bad
nets went 8/5/4/4 -> 3/2/2/2; SA15 6, SDQ9 4, SA1 4 appeared), K35 -3,
but K28 +6 and K51 +5 with an open net. **K28's mechanism:**
same pages, same joiners, the same 24 plan-implied changes -- but the
model, indifferent at 2 between an F berth with a B leg and a B berth,
took B berths for five joiners; the braid then put their legs on F (its
own tie: corner 1 against tip 1) over 7-10 F lanes whose berths are B,
which the term priced at 0 (the change "taken early") and which then
needed their OWN F legs: up again and down again. The braid's plan
predicted both boards exactly (ends 10 + 24 = 34; 16 + 24 = 40). The
corrected rule: a crossed lane pays two unless it leaves its page at its
own leg (z >= hit_page and not corner), and a lane hit by legs of both
layers pays two whatever it does. **The braid's plan-implied count (ends
+ changes) predicts the routed board within 3 at every K seen; the
pages-first objective does not.**

**Verdict (edict 3): nothing here is a default.** The corrected term
(lg3) repairs K28 and buys K35 -3, leaves K41's copper identical to pg2,
and costs an open net at K51 with 30 s more solve time. The away filter
(aw0) is the largest single effect -- K41 72 (histogram 0:10 2:26 4:5)
and K35 -3 in 73 s -- and with the term (lg4) K35 60 / K41 72, the best
clean K35 and K41 this chain has produced; but every arm that helps at
K41 ships an open net at K51 (four in lg4), where the plan is over the
two-page capacity and completion is the knife edge. `PLAN_PAGES=1` alone
is still pg2's stack, byte-identical (checked at K28 / K41 after the
edits). The open items, in order: the K51 completion under the away
filter; the model's disagreement with the braid (the braid's plan count
is the judge to trust); the solver's swimmer search (2 is feasible where
it settles for 4).

### 2026-09-14 (session 4): K51 completion, the braid's count as the judge, the swimmer descent

Andy's order for the day: K51 completion under the away filter; judging
plans by the braid's plan-implied count instead of the model's; the
swimmer search that settles for 4 where 2 exists. Bench pg2 (the
committed defaults): 16 / 34 / 65 / 79 / 115; re-verified IDENTICAL
copper at K28 (fanout board and routed board) after every edit below.

**K51 under the away filter completes -- a braid fix, `BRAID_RIP_PROBE_ALL=1`.**
aw0 refused SA10 (a swimmer with 17 page crossings) at the last call
with "no path even with every lane priced -- walled by static copper".
It was not: its tooth exit sat in a wedge between SA4's and SCKE1's F
diagonals, with SODT0 and SA2 on B right under it. `rip_for`'s min-cut
probe priced only the lanes on the refused search's FRONTIER (SA4 SCKE1
SRST SCS1), and the frontier is one lane deep by construction -- the
boundary of the reachable pocket -- so the probe could cross the first
wall and no other: crossing SA4 needed B, B was hard copper to it, no
path. With every lane of the run priced the cut set reads [SA4 SA1 SA2
...], the trial [SA4, SA1, SA2] routes SA10 at 2 and re-lays the three
victims at +2 each: **K51 121 / 0 open / 0 DRC** (aw0 113 + SA10 open,
pg2 115 complete). The knob acts only where the false verdict occurred
-- aw0 K51 and lg3 K51, once each; never at K15-K41 on any arm, and not
on the old planner's bs1 K51 (its SBA2 refusal is a cut set no trial
can re-lay) -- so everything else is byte-identical with it. Default 0.

**The judge: what predicts the routed board (`pred_vs_routed.py`,
scratchpad e90127ce, 19 fanout/routed pairs).** Per class, the braid's
plan-implied vias (the fanout board's ends + `plan_braid`'s per-lane
count) against the routed vias:

| board | page lanes: ends + `changes` -> routed | swimmers: ends + `swim_changes` -> routed | swimmers: ends + FLAT 2 -> routed |
|---|---|---|---|
| pg2 K28 | 40 -> 34 | -- | -- |
| pg2 K35 | 56 -> 59 | 8 -> 6 | 6 -> 6 |
| pg2 K41 | 54 -> 67 | 28 -> 12 | 13 -> 12 |
| pg2 K51 | 61 -> 73 | 74 -> 42 | 33 -> 42 |
| aw0 K41 | 46 -> 48 | 58 -> 24 | 25 -> 24 |
| aw0 K51 | 67 -> 81 | 98 -> 32 | 29 -> 32 |
| bs1 K41 | 50 -> 53 | 108 -> 38 | 36 -> 38 |
| bs1 K51 | 47 -> 49 | 144 -> 63 | 51 -> 63 |

So the count that tracks the copper is `plan_ends.vias_from_pages` as
`judge_by_braid` already sums it -- ends, each page lane's profile
changes, a FLAT `SWIM_VIAS` per swimmer -- WITHOUT the ride term (the
ride is what reverted a needed batch of teeth and why the model's count
replaced it). `swim_changes` over-predicts a swimmer 2-3x (the router
finds a smarter line than the hold-then-run), and the pages-first
model's count sees no exit legs (K41 pg2: model 46, braid 67, routed
79). Page lanes under-predict at K41+ by 12-14 -- the jammed strips
(SCKE1 8, SBA1 5) that the plan phase does not model.
`PLAN_PAGES_JUDGE=braid`: `pages_first.verify` returns the count
(`braid_count`), `choose`'s key becomes (count, braid swimmers), and
both realize-confirm sites in `fanout_from_plan` (`_pf_key`) become
(sum of `pred`, residue). `PLAN_PAGES_JUDGE=resbraid`: the residue
first, the count where it ties (only the model's vias replaced).
Default `model` = pg2.

**The swimmer descent, `PLAN_PAGES_DESCENT=1`: built, and it loses on
the merits.** After phase 1 the model is re-solved on a `clone()` with
the swimmer count capped one below the answer's and the answer as the
hint (a constraint cannot be taken back out of a CpModel; assumptions
work too), per level under `PLAN_PAGES_DESCENT_DET`, until INFEASIBLE
(a proof) or UNKNOWN. On the K41 chain's own instance (`desc_probe.py`):

| solve | model swimmers | model vias | teeth moved | braid swims | braid-implied vias |
|---|---|---|---|---|---|
| plain (pg2) | 4 (obj 2309) | 46 | 9 | 6 | **72** |
| descent, DET 20 | 2 (obj 1905) | 80 | 22 | 5 | 97 |
| descent, DET 40 | 2 (obj 1761) | 56 | 10 | 4 | 83 |

<= 1 swimmer is proven INFEASIBLE in 8 s at both budgets. The 2-swimmer
plans the descent finds are the ones the standalone probes called
"better" (obj 1761 against 2309), and under the braid's count they are
worse by 11 and 25: the objective's 300 per swimmer buys fewer swimmers
with more moved teeth and more end vias, which is what the copper pays
for. So "settles for 4 where 2 exists" was a search failure only in the
model's own objective; the search was never the lever.

**Ladder of the arms above (vias / open):** jb (`PLAN_PAGES_JUDGE=braid`)
16 / 38 / 68 / 79 / 119 + SCS1 open; jba (jb + `SRC_AWAY=0` +
`RIP_PROBE_ALL`) 16 / 38 / 62 / 72 / 134 + 3 open; desc
(`PLAN_PAGES_DESCENT=1`, DET 40) K41 93; sw4 (`PLAN_PAGES_SWIM=4`) K28 36,
K35 **54** (stopped there; the best K35 this chain has produced, human
58 -- unmeasured at K41/K51). pg2 16 / 34 / 65 / 79 / 115. The count-first
judge tolerates swimmers at the flat 2 and K51 then ships 1-3 open;
resbraid (residue first) was queued and not run. Andy stopped the arms:
"everything you're doing seems like a failure at K51 -- diagnose".

### The K51 diagnosis (2026-09-14): the human routes BUNDLES, we route lanes

Andy's reading of the renders, which the census confirms: **the human
keeps tracks on the same page in bundles, over length.** `bundles.py`
(scratchpad e90127ce) classes each K51 net by the ARC it takes round
DU1 (copper above DU1's box = north, below = south, neither = mid) and
the layer it runs on outside both arrays:

| arc / layer | human: nets (vias) | ours pg2: nets (vias) |
|---|---|---|
| mid F | 4 (0) | 7 (2) |
| mid F+B | 6 (12) | 8 (**36**: SA12 6, SDQ13 6, SDQ4 6, SDQ11/12/15/9 at 4) |
| north B | **10 (20)** | 4 (10) |
| north F / F+B | 2 (2) / 1 (2) | 0 / 3 (8) |
| south B | 3 (6) | 1 (2) |
| south F | 8 (12) | 6 (8) |
| south F+B | 13 (26) | 18 (48) |
| total | 47 (80) | 47 (114) |

The human's north bundle is ten address nets on B END TO END (F 0-2 mm
each, two dog-bones): SA0 SA1 SA11 SA12 SA14 SA15 SA2 SA4 SA8 SBA1 --
every one a ball in DU1's EAST columns (M..T), reached by riding B round
the north of DU1 and down into the east end of the array, where B is
free under the balls. The south bundle is the bottom block's east end
and the gap-S row: eight nets pure F, thirteen F with a short B tail at
DU1 (the 'dD' diver), three B. The middle is the west columns (A..H),
straight in. Bundles are chosen by DESTINATION GEOMETRY (which block,
how deep from the west face), each bundle is planar and one layer, and
bundles never cross because they are spatially apart. Length is not a
cost: the south bundle is a 25-30 mm detour for 17 mm pads.

Ours: one corridor, 47 lanes, two LAYER pages plus 11-12 swimmers. Per
arc against the human: the middle 15 nets / 38 vias against 10 / 12 (the
corridor's sorting and the swimmers' dives land there -- corridor vias 23
against 7), the north 7 / 18 against 13 / 24, the south 25 / 58 against
24 / 44 with 18 mixed-layer lanes. Ten nets are in the WRONG bundle:
SA2 SA4 SA8 SA12 (human north-B; ours south or mid at 2-6), SRST (human
south-B; ours north), SDQ13 (human mid-F at 0; ours mid at 6), SDQ4 SDQ5
(human south; ours mid). And the source is the other half: the human
peels the north bundle out of U1's NORTH face on B (7 N-B exits; ours N 2,
E 29, S 16), so ours must cross the whole bundle inside the corridor to
reach a north berth -- **170 of the plan's 224 crossings involve one of
the 11 'up' berths** (down-up 66, left-up 54, up-up 27, right-up 23);
the human's ends have 101 crossings in all. Via classes: human 0
via-in-pad / 34 dog-bones / 47 free; ours 27 / 25 / 7 weave pairs.
Region: human src 17 / corr 7 / dst 57; ours 21 / 23 / 71.

What K51 is, then: K41 fits in two layer pages of one corridor with
4-6 swimmers; the seven nets K51 adds (SA10 SA14 SDQ1 SDQ3 SDQ4 SDQ5 SZQ:
deep U1 balls, DU1 columns E-H and L/T) push it over that capacity, and
the human's answer is not a better two-page schedule but MORE PAGES:
(north, B), (south, F), (mid, F), (mid, B) -- pages by (arc, layer),
with the arc decided by where the ball sits in DU1 and the north
bundle launched from U1's north face. Nothing in the chain chooses an
arc: the berth chooser prices faces by vias/channel/reach, the braid
forms ONE corridor and pages it by layer, and the source menu has no
north-face exit. Every arm of this session (judge, descent, swimmer
price, rip probe) worked inside that structure.

**Andy's direction after the diagnosis:** finish the cheap-swimmer arm
(sw4 final: 36 / 54 / 86 / 119 + SBA2 open -- K35 only, not a default),
and look into MORE PAGES like the human: north / south / middle for
both F and B, six pages, each guided towards its own region.

### The six pages: what the probes say (2026-09-14, afternoon)

**Phantom crossings.** `phantom_xing.py` (scratchpad e90127ce) runs the
braid's plan phase on a board and classes every launch/target crossing
by the two nets' (arc, layer) as routed on a reference board. On the
HUMAN'S ENDS (hbn_k51, the clipped bench; SZQ dropped) our one-corridor
order model counts 101 crossings: **67 are phantom** (59 between lanes
on different arcs, 8 between different layers -- pairs that never meet)
and 34 real, 20 of them inside the middle bundle. On our pg2 plan: 224
counted, 147 phantom, 77 real. So the order model that drives every
schedule and every judge is counting crossings between lanes that go
round opposite sides of DU1.

**Arcs as corridors: the plan can now name them.** `braid.setup` takes
`plan['corridors']` (a list of net lists) as the grouping when a sidecar
carries it, and accepts a grouping-only sidecar (no ends). The human's
three K51 bundles written beside hbn_k51 (`tmp/hbn_k51.plan.json`,
north 13 / mid 10 / south 24):

| ends | corridors | schedule | plan swimmers | routed | open |
|---|---|---|---|---|---|
| human | 1 (as clustered) | greedy pages (hb1) | 11 | 93 | 2 (SCS0 SCS1) |
| human | 3 arcs | greedy pages (hb3) | 3+4+3 = 10 | 96 | 3 |
| human | 3 arcs | exact pages (hb4) | 2+2+1 = 5 (= MIN) | 101 | **8** (the whole middle) |
| human | 3 arcs | one-dive 1 (hb5) | -- | 111 | 12 |
| human | 3 arcs | one-dive 2 (hb6) | -- | 112 | 14 |
| human | 3 arcs | one-dive 3 (hb8) | -- | 105 | 9 |
| human | 3 arcs | one-dive 5 (hb7) | -- | 100 | 4 |
| human | 1 | exact pages (hb10) | 11 | 95 | 2 |
| human | 1 | one-dive 5 (hb9) | -- | 100 | 3 |
| human, real board | -- | -- | -- | **81** | 0 |

Two-chain capacity of the human's ends (`two_chain.py`): one corridor
101 crossings / MIN 6 swimmers; three arcs 17 + 20 + 14 = 51 crossings /
MIN 2 + 1 + 2 = 5. **One-change residue (`one_change.py`): one corridor 4;
three arcs 0 + 0 + 0, model 88 (ends 43 + changes 45) against the
human's 81.** So the STRUCTURE that reproduces the human is exactly
"arcs + one change per lane": the arc removes the phantom crossings,
the free change point resolves the real ones (the human's mid-bundle
lanes cross each other with one dive each, F before it and B after --
a two-page lane cannot, its change is pinned to an end).

**Why the braid cannot route it yet.** hb4's render: the three
corridors all get STRAIGHT two-vertex spines between the arrays (north
15.3 mm, mid 5.5 mm, south 16.6 mm). The north spine runs under the
passives along DU1's top row -- the 2.2 mm strip -- not above them
where the human's north bundle rides, and all three share the launch
region, so the bundles overlap and the middle corridor's lanes (planned
after the north one, whose lanes are its reservations) are refused six
at a time. `build_spine` draws a straight line whenever the launch and
arrival flows are within 30 degrees, and the destination array is its
own corridor's end (`own`), never an obstacle: nothing pushes an arc's
spine round the array. The general rule the human's board suggests: an
arc corridor's spine must wrap the destination's box inflated by the
corridor's half-width H (LPITCH x (n-1)/2 + LPITCH: 2.45 mm for 13
lanes, wider than the 2.2 mm strip, so it goes ABOVE the passives; a
thinner bundle would use the strip), with the parts inside that margin
solid to it; the middle corridor planned first, the arcs relaxed round
its tube. The source half is the other missing piece: the human peels
the north bundle out of U1's north face on B (7 of its 10), and our
source menu offers an 'up' move to 2 of those 10 (SA0 and SA4 have
EMPTY menus: boxed in by the bench's teeth).

**The build, in order (none of it written):**
1. region-guided spines for arc corridors (the rule above), middle
   first; measure on the human's ends with the arc sidecar until hb3/hb4
   route at ~88 with 0 open -- that is the gate for the braid half;
2. one-change scheduling per corridor (`BRAID_ONE_DIVE` exists at levels
   1-5; on the arcs it shipped 4-14 open, so it must be re-measured once
   the spines are right);
3. the planner: an arc per net (corridor membership) -- keys from
   `braid_slots` on a 3-corridor seed, pairs only within a corridor (the
   model's existing rule, which is what deletes the phantoms), the arc
   decided by the berth's side; the source menu extended with the N/S
   face B exit for a deep ball (a dog-bone into the gap, a B run along
   the column out the face) so a north-arc lane launches outermost-up
   instead of crossing the bundle.

### The wrap spine (2026-09-14, later): built, and what it exposed

`Corridor._arc_spine` (braid.py): an arc corridor (the plan's
`arcs` list beside `corridors`, 'N' / 'S') gets a WRAP SPINE instead of
`spine_of`: the destination's pad box inflated by H + a ball's radius +
clearance + a track's half-width + 0.2, merged with every part OUTSIDE
the array's own box on the arc's side whose pads reach into the inflated
box (re-inflated after each merge, until nothing touches), then the
polyline teeth-centroid -> out along the launch flow -> the box's near
corner on the arc side -> along it -> round the far corner and down the
far side to where the last stub projects, plus 0.5 mm. Axis-aligned, no
relaxation. On the human's K51 ends: N spine along y 52.7 (13 parts
merged, above both passive rows), S spine along y 74.3 (C5 C12 C6 C10
merged), each up the east side to x 148-153. The render has the human's
shape. Two lessons on the way: (1) a decoupling cap BETWEEN the balls
must not widen the wrap (merge only parts outside the array's box), and
(2) an arc's members must be the nets whose STUBS are on that arc's
faces -- classing by the copper's y-extent put the west-column SDQ nets
that merely dip under DU1's south-west corner into the south corridor
(24 lanes, 8.7 mm wide, 20 legs through the caps). With a west-column
net in the north corridor s1 < s0 and `route_lane` asked numpy for
3.5 PiB: a guard is owed there.

| grouping (human ends, SZQ dropped; SCS0/SCS1 are a BENCH DEFECT -- two unconnected pieces, no via -- open in every run, so real opens = open - 2) | schedule | swimmers | vias | open |
|---|---|---|---|---|
| v3: mid 19 / N 10 (the human's north-B) / S 18 (bottom block + gap-S east end + SCKE) | greedy (ha3) | 6 + 1 + 7 | 115 | 4 |
| v3 | exact pages (ha4) | | 109 | 4 (SA15 SCKE1) |
| v3 | one-dive 5 (ha5), + exact (ha6) | | 116 | 5 (SA11 SA12 SA15) |
| one corridor, greedy (hb1) | | 11 | 93 | 2 (= 0 real) |
| human | | | 81 | 0 |

**Why the arc still loses under the two-page schedule: the order keys.**
The south corridor alone, in the wrap frame (`two_chain` v3: 66
crossings, MIN 6 swimmers; one-change residue 4). Launch: the human's
deep balls (SODT1 V11, SODT0 W11, SA13 V12, SRAS V13, SWE W13) escape
STRAIGHT SOUTH through the array on F and leave the clip box at its
south edge at x 120-123, so they are the OUTERMOST lanes of the
eastbound band; the shallow east-face teeth (SA3 SCKE1 SCS1 at x 127.7)
the innermost. Target: the braid's exit comb puts the first exiter
innermost, and the human's first exits (J1 SODT1, K1 SODT0, J3 SRAS at
x 140-142) are exactly the outermost lanes -- the comb's order is the
reverse of the band's. The human resolves it without any reordering:
**the south bundle is ONE layer (F) end to end in launch order, and
every lane leaves it by a leg on the OTHER layer (the dive) at its own
stub** -- a B leg under an all-F band crosses anything, so the exit
order needs no comb and the schedule has nothing to resolve; 2 vias per
net = the corner where the leg dives + the berth's own via. The north
bundle is the mirror: an all-B band whose legs enter the array on B
(same layer), so there the comb applies -- first exiter innermost -- and
the human orders the launch to match by running the B lanes up U1's
east side under the F teeth in the order the comb wants (a B end makes
the rank free; residue 1 under our keys). The braid has the leg
economics already (`place_and_decide`: a band lane's leg on its own page
crosses nothing by the nested comb; other legs on the side's majority
berth layer, crossings priced along s) but no mode that says "band =
one layer, target offsets = launch offsets, legs on the other layer".

**The arc-corridor mode to build (not written):** per arc corridor a
band layer (from the plan: N = B, S = F on this pair; in general the
layer whose escapes are cheaper at that arc's faces), every lane paged
on it, no morph (target_o = launch_o for side exits, the comb only for
same-layer legs), side-exit legs on the other layer at the stub's own
s, far-face lanes carried round the corner by the wrap spine. Cost per
lane = tooth/band mismatch + corner + tip. Gate: the v3 grouping on the
human's ends at ~88 / 0 real open.

**Choosing the arc paths as part of the plan (design).** An arc is a
homotopy class round the destination: which side, and which channel on
that side -- between the array and the parts attached to it, or outside
them. The wrap rule decides the channel from the bundle's WIDTH (13
lanes need 4.9 mm + margins, the strip under the passives has 2.2, so
the box swallows them; a 5-lane bundle would ride the strip): capacity
is the criterion, so the plan's choice is the MEMBERSHIP of each arc
(which nets, hence the width) and the band layer, and the channel
follows. The plan can price an arc candidate by: the wrap path's length
(the human ignores it, so a low weight), its narrowest gap against the
bundle's width (infeasible when narrower), the end costs of its members
(tooth/band and tip mismatches, 1 via each), and -- for a same-layer
comb (the N case) -- the inversions between the launch order and the
comb order, which are the only crossings that survive in this model.
The pages-first CP-SAT already has the pieces: an arc index per net as
a variable, `braid_slots` keys built per arc (a 3-corridor seed), pair
constraints only within an arc, and the source menu extended with the
N/S-face and edge-hugging B exits that let a north-arc lane launch in
comb order.

**The bench defect, and the fix (Andy: "fix the bench by moving the
line").** `human_bench.py` clips the human's copper at each array's pad
box + 1.0 mm. The human dives to B exactly on U1's edge line for the
lanes that turn along the edge (SCS0 0.01 mm outside the line, SA4 0.16,
SCS1 0.21), and dog-bones 1.0-1.5 mm outside DU1 on every side (SA13
0.05 out, SA14 0.24, SDQ5 0.29 at 1.0), so ANY single margin cuts through
some via: no line to move to. The fix is a via-aware clip
(`human_bench2.py`, scratchpad e90127ce): per net and box, the box steps
out to swallow a via of that net lying within 0.5 mm outside the line
(+0.1), so a dive on the line keeps its run. New benches
`tmp/hbn2_k51` / `tmp/hbn2_k41` (54 vias kept against 44); the old
`hbn_*` stay for the numbers above. Why only SCS0/SCS1 were open: SA4's
via was cut the same way, but the endpoint finder took its F piece as
the tooth, while for SCS0/SCS1 it took the orphan B piece whose free end
points out of the box.

**Three benches, three answers (10:10-10:25).** The via-aware bench
(`hbn2`, edge vias kept) turned out harder for the braid, not easier:
the human's dog-bones sit 1.0-1.5 mm outside DU1 on every side and on
U1's edge line, and kept as static copper they crowd the launch and
arrival zones (SA4's via 0.8 mm from SDQ7's tooth; SDQ7 refused with its
start cell boxed by virtual copper). It also exposed a second defect of
the old bench: the human's board carries 68 `(arc ...)` copper items
that `human_bench.py` never stripped, so orphan arcs survived every clip
as obstacles (and the braid's writer re-emitted their expansions beside
them -- the "same-net soft joint" DRC pairs). The clean bench is
`human_bench3.py` -> `tmp/hbn3_k51` / `hbn3_k41`: the plain 1.0 clip,
arcs stripped, and every piece not connected to its own pad dropped
(SCS0/SCS1 get a clean F tooth like SA4; SDQ4 lost 21 orphan pieces
inside the DU1 box). Invariant asserted: every K net is exactly two
copper components. The same runs on all three:

| bench | 1 corridor, greedy | 3 arcs, greedy | 3 arcs, exact |
|---|---|---|---|
| old `hbn` (orphan arcs + stubs; opens minus SCS0/SCS1) | 93 / 0 | 115 / 2 | 109 / 2 |
| `hbn2` (arcs stripped, edge vias kept) | 87 / 7 refused + SCS1 | 111 / 1 (SA8) | 125 / 1 (SA8) |
| **`hbn3` (clean)** | **106 / 0** | 122 / 4 (south arc) | 80 / 11 (south arc collapsed) |
| human | 81 / 0 | | |

So the bench moves the one-corridor answer by 19 vias and the arc
answer by up to 42; every number above is only comparable within its
row, and `hbn3` is the one to use from here. On it the single corridor
is 106 / 0 against the human's 81, and the two-page schedule on the
three arcs loses (the south arc's comb order, as diagnosed) -- the
arc-corridor MODE is still the build. One reading survives all three
benches: with the human's edge vias in place (`hbn2`) the arcs COMPLETE
where the single corridor refuses seven, which is the region separation
doing what it should even before the mode exists.

## Settled -- do not re-run these

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

1. **Close K51** (item 2) -- but read "SA1 corrected" first: SA1 itself
   closes at 3 vias with the phantom-victim fix, and the refusal then moves
   to SA11 + SDQ11, so the tail cannot be tuned into a complete cheap board
   on this plan. The lever is the PLAN (a berth in the SA1 / SA11 / SDQ11 /
   SBA1 / SDQ4 region must move), i.e. the replan; the tail's mechanics are
   measured out (depth, phantom, soft, rescues -- the table in that section).
2. ~~**`SF_ACCEPT_MARGIN`** (item 1c)~~ -- MEASURED 2026-09-13 evening on the
   cloud: neutral at 1, harmful at 2-3 (K41 88 -> 122 / 1 open). See 1c.
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
   **RUN 2026-09-13 evening, on Modal (`awx/tmp/sweep_margin_0913.json`;
   `cb` baseline, `e0m<M>` = `SF_ESC_W=0 SF_ACCEPT_MARGIN=<M>`, `mo<M>` =
   the margin alone, K35/K41/K51 each), and it is NOT the medicine:**

   | | K35 | K41 | K51 |
   |---|---|---|---|
   | `cb` (margin 0) / `e0m0` (`SF_ESC_W=0`, margin 0) | 60 / 60 | 88 / 88 | **129, 1 open** (= the recorded cloud baseline, a third time) / **124, 0 open** |
   | margin 1 (`mo1` / `e0m1`) | 60 / 68 | 88 / **110, 1 open** | 114, **2 open** / 114, **2 open** |
   | margin 2 (`mo2` / `e0m2`) | 70 / 68 | **122, 1 open** / **110, 1 open** | 118, **4 open** / 118, 4 open |
   | margin 3 (`mo3` / `e0m3`) | 70 / 80 | **122, 1 open** / **110, 1 open** | 118, 4 open / 118, 4 open |
   | `stk` (`SF_ESC_W=0`, margin 2, `BRAID_L5_SEED=0`) | 68 | 102 | 123, 2 open |

   **Verdict: not the medicine.** Margin 1 is neutral where it is not worse
   (K35 60 = 60, K41 88 = 88; with `SF_ESC_W=0` K35 +8 and K51 trades 10
   vias for two OPEN nets, which under-count). Margins 2-3 are HARMFUL at
   every K: K41 +34 vias and an open net, K51 four open nets, and the K51
   outcome is IDENTICAL (118 / SA1, SCS1, SDQ11, SRST) for margin 2 and 3
   with or without `SF_ESC_W=0`, so the margin, not the escape weight, is
   what decides it (and at K41 margins 1-3 under `SF_ESC_W=0` are all the
   same 110 / SBA2 open). So the three accidental regularisers were not
   "accepting at 1e-6" in disguise: a margin that refuses the small judged
   wins refuses REAL ones, because the judge's 2-via median improvement is
   where its signal lives as much as its noise. (Six arms died with the
   Modal client -- the harness killed it for host memory while a K51 replan
   ran beside it; `--out` resumes, and those cells are re-running.)
   One number in that table is NOT about the margin and is worth its own
   line: **`e0m0`, i.e. `SF_ESC_W=0` alone, gives K51 124 / 0 open on the
   cloud**, and `cb`/K51 in the SAME sweep is 129 / 1 open (SDQ3) -- the
   recorded cloud baseline for the third time. So on the cloud instance
   `SF_ESC_W=0` is -5 vias AND closes the open net: **the first complete
   cloud K51 from a one-knob arm**, where locally the same knob was
   bit-identical at K51. Read it as the local-vs-cloud divergence says to:
   a different instance, so a lead to confirm with repeats (K51 baselines
   reproduce exactly on the cloud, so one repeat is evidence there), not a
   local result.

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

### The arc-corridor MODE (2026-09-14, session 5): built, and what each arm taught

Andy: "the arc-corridor mode in the braid (band layer per arc, target
offsets equal launch offsets when legs are on the other layer, comb only
for same-layer legs), gated on the human's ends routing at about 88 with
0 open; then the plan chooses arc membership and band layer, the channel
following from the width."

**The mode (braid.py, gated by `plan['bands']` beside `plan['corridors']`
/ `plan['arcs']`: `'F.Cu'` / `'B.Cu'` / null per corridor; `Corridor.band`,
`Corridor.arc_inner`, `Corridor.LP` / `.BG`).** An arc corridor with a
band is one bundle on one layer end to end: every stub a side exit reached
by a leg at its own s (no head-on exits, no far-face legs -- the wrap
carries the bundle round the corner), the exit block in LAUNCH order
(`offsets`: no morph, so nothing crosses inside the band), one page for
every lane (`_make_sched`: the two-page pager's answer overwritten, no
swimmers), the page held to the lane's own leg (`lay_lanes`), the leg's
layer by `place_and_decide`'s economics with the tie to the OTHER layer
(a leg there crosses the whole band for its corner via -- the human's
south bundle; the band's own layer wins only where the stub is on it and
nothing still present lies between slot and stub -- the north comb).
"Comb only for same-layer legs" turned out to need no comb at all:
re-ranking a lane inside the band is a swimmer at 2 vias, the same price
as the other-layer leg, and the other-layer leg needs no weave; the
nesting is the LAUNCH order's business, which the join comb along the
face gives for free (below) and the plan will choose.

Nothing in that design routed. Every arm below is a defect the design did
not contain, found by a probe, fixed in general terms; the flag-off chain
stays byte-identical (K28 fanout board and routed board IDENTICAL to pg2
after every edit). Bench `tmp/hbn3_k51` (human ends, SZQ dropped, v3
grouping mid 19 / N 10 / S 18), `tmp/arc_arm.sh TAG`; probes in
`tmp/arcprobe/` (`mask_probe`, `hop_probe`, `flood_probe --route-before`
= a two-layer flood of a lane's tube minus every obstacle the router
sees, which is the instrument that found most of these; `plan_dump`,
`inmask`, `zoom.py` renders copper + the Eco plan lines, `vs_human.py`
per-net vias against the human's board):

| arm | what changed | vias / open | s |
|---|---|---|---|
| hb1c | one corridor, two pages (the base on this bench) | 106 / 0 | |
| am1 | the mode as designed | 131 / 3 | 118 |
| am4 | the stub's spine frame by the ARRAY's box (`_arc_project`: a wrap spine passes a stub on two sides; the nearest leg was the east one for DU1's north-line stubs, and every leg ran along the north line at one s) | 141 / 3 | 132 |
| am5 | the legs' and jogs' mask rectangles in their own segment's frame (the lower half of each north leg projected to the east frame and fell outside the mask: 8/8 neighbours blocked by nothing) | 141 / 3 | |
| am6 | the launch leg ALONG the teeth's line toward the arc (out along the mean escape the ten teeth spread along it, every one head-on over the next one's stub); a tooth with any tooth downstream joins; join legs stamped on the tooth's layer only | 110 / 4 | 100 |
| am8 | the wrap merges only parts with copper on the BAND's layer (the F-only passives north of DU1 had pushed the B band 8 mm out, past the board edge -- eight of ten lanes planned off the board) | 112 / 3 | 128 |
| am9-11 | `Spine.lane_xy` fold clamp (a vertex inside a corner's inner fold rendered 0.8-3 mm behind its mitre); the band mask as an XY TUBE round the lane's own mitred polyline (`_band_xy`, `lane_sxy`: the (s, o) mask interpolated the offset through the corner and lay on the NEIGHBOURS' true paths; the extreme lanes' open side was unbounded and SA8 searched DU1's ball field); pitch 0.42; innermost first | 119 / 2 | 139 |
| am12-13 | band corridors first + ribbon start pushed past the combs: the middle corridor's channel is 4 mm and it collapsed (15/19 -> 2/19, 9 open) | 91 / 9 | 90 |
| **am14** | plan order; an unrouted band corridor's LAUNCH COMB reserved whole for the corridors before it (`cross_reserve`); the ribbon push only where 2 mm of region remain; the launch leg CENTRED on its block (`BG` + `LP`(n-1)/2 off the teeth: through the teeth the inner lanes launched 4 mm inside the spine and cut the corner's margin through C9); pitch 0.40, gap the legal minimum, axis snapped to the source array's own axes | **93 / 1** (SBA2) | 79 |
| am15 | + the econ re-lay honours reservations (`BRAID_ECON_RESERVE`): north 20 = the human's, middle 24 -> 38 | 115 / 7 | 89 |
| am16 | + arc lanes routed HOP BY HOP through their polyline corners (`BRAID_ARC_HOPS`: the two-layer flood proved the south tubes connected while one search died at 72-120k iterations) | 115 / 1 | 57 |
| am17 | hops without the econ reservation: south 42 -> 39 and 10/18 in band, north 27 -> 33 (SBA1 10, a hop's end pinned where the lane wanted to dodge), and 5 whole-board DRC -- the south band's outer lanes and a via 0.06-0.10 mm over the board-edge clearance. Hops off by default. | 96 / 1 + 5 DRC | 62 |
| am18 | the wrap side clamped to the board edge less the band's half-width (the south side 74.75 -> 74.58); hops off | 91 / 2 (SBA2 SODT1) | 97 |
| human | | 81 / 0 | |

am14 per arc against the human: middle **24 = 24**, north 27 / 20, south
42 / 36 (the bench keeps 40 of the human's 80). In band: middle 13/19,
north 4/10, south 9/18 -- the rest at last call, 2-8 vias each. am18 is
the tree as left (the edge clamp is a DRC class, kept): 91 / 2, the same
in-band counts, SODT1 lost on the knife edge the clamp's 0.17 mm moved.
**The gate (~88 / 0 real open) is not met**; the base on this bench is
106 / 0 and the mode is 91-93 / 1-2 in 80-100 s.

**What the probes say the rest is.** (1) U1's south-east corner: the
north corridor's SA8 stands on the SOUTH face and its join leg must cross
the east-face line between the south corridor's SA3 / SCS1 / SCS0 teeth
0.3-0.5 mm apart; the north lanes born on the southern teeth (SA0 SA2 SA4
SA1 SBA1) are walled at their join legs by the middle corridor's B copper
-- its econ re-lay ignored the comb reservation (SDQ12 / SDQ14 3 -> 1 via
straight through the slots), and with the reservation honoured the
middle pays 14. The human's comb SLANTS north-east from the southern
teeth and its middle dives sit just east of it at x 129.4-131.7; ours is
vertical along the face, so the two want the same channel. (2) The
south corridor's long lanes die at the search budget in one search
(hops fix that: 4-9/18 -> 11/18) and SA5 / SBA2 end INSIDE DU1's box
(the bench's clip), reached only through the ball rows. (3) A last-call
lane is 2-8 vias where the in-band lane would be 2.

**Next**, in order: the launch comb slanted toward the wrap (the human's:
NNE from the southern teeth, which also frees the channel's east half
for the middle bundle's dives); a corner tooth (SA8) joining OUTSIDE the
other corridor's teeth; then the plan (arc membership + band layer as
CP-SAT variables, keys per arc, the source berths chosen so the comb
nests), and the mid corridor as a band with dives at DU1.

### The slanted comb (2026-09-14, session 6): the human's count on the human's ends

Andy: "Next, in order: the slanted comb, corner teeth joining outside
the other corridor's teeth, then the plan choosing arc membership and
band layer." And, mid-session: keep the router GENERAL.

**What the bench really is.** `human_bench3.py` clips the human's
copper at U1's box + 1 mm (x 127.69 on the east, y 71.43 on the south),
so eight of the north corridor's ten teeth are the ENDS OF THE HUMAN'S
OWN 45-DEGREE RUNS, pointing north-east at x 127.69, y 59.3-67.6; the
south teeth point south-east along y 71.43. Measured segment by segment
(`tmp/arcprobe/comb_geom.py`): the human's north lane is a short east
leg, a 45-degree jog, a NORTH run beside the face for the three southern
teeth (SA0 4.4 mm at x 127.85, SA2 4.7 at 128.25, SA4 4.8 at 128.6 -- a
pitch apart), then 4-6.7 mm at 45 degrees into the band at y 57-59.6.
The session-5 comb ran the launch leg ALONG the teeth's line: the block
a gap beyond the teeth (x 127.7-131.2), every tooth a joiner with a leg
across to it, and the whole block parallel to the face -- through the
channel's east half where the human's middle bundle dives (x 129.4-131.7,
y 61.7-66.3).

**The mode (`BRAID_ARC_SLANT`, default 1 in the arc mode, band
corridors only).** `_arc_spine`: the launch leg runs from the teeth's
centroid toward the wrap's near corner along the nearest OCTILINEAR
direction (north-east for the north arc, south-east for the south),
straight to the wrap's side. `classify`: every band tooth is head-on in
that frame. `offsets`: the slots are the compact block in the teeth's
PERPENDICULAR order, its outer edge (the side away from the destination)
at the outermost tooth, every other tooth pulled toward it by a 45-degree
run in the frame -- north on the board for a north-east leg -- after a
JOG along the spine that staggers pulls sharing a line a pitch apart
(teeth on one face line all pull on one line otherwise); a shift under
half a pitch is the router's wiggle, not a run, and a pull is staggered
only past an earlier pull whose offset range comes within a pitch of
its own (the first rule, rank order with every line staggered, jogged
SA8 5 mm and chained the south's 0.1-0.7 mm pushes to 5 mm). Outward
pulls are laid outer slot first, inward pushes inner slot first: the
order in each family that lets no pull cross a lane already at its
slot. The fan-in vertices (jog end, pull end) are in the lane's own
polyline, so the band tube and the reservation follow them; `s0` is the
longest fan-in's end. The flag-off chain touches none of this (`slant`
is set only by `_arc_spine`).

**Then two reservations the slant exposed** (both by
`flood_probe --route-before`, the two-layer flood of a lane's tube minus
every obstacle):

- The middle corridor's ECON RE-LAY had put SDQ0, SDQ5 and SDQ7 through
  the pulled lanes' tubes (SA2's pocket ended at y 65.2 on SDQ0, SA4's
  at x 128.0 on SDQ7) -- the known shared defect (`BRAID_ECON_RESERVE`,
  measured a LOSS under the vertical comb: the middle paid 14). Under
  the slant it costs the middle 2.
- The band was reserved only to `s0 + 0.3`, which with the vertical
  comb lay past the channel and with the slant lies mid-channel: the
  middle's swimmer SDQ9 dove ON SA0's line 0.02 mm past the stamp's end
  and ran across SA2's and SA4's. A band corridor is one layer with
  exact slots end to end, so `cross_reserve` now stamps a band corridor
  WHOLE for the corridors routed before it: every lane's line on the
  band's layer from the tooth to its exit leg, the leg on the layer the
  economics chose, the tail on the stub's (`BRAID_ARC_RESERVE_BAND`,
  default 1). The two-page corridors keep ENDS ONLY, where whole lines
  were measured to starve the earlier corridor.

| arm | env | vias / open | in band M / N / S | s |
|---|---|---|---|---|
| am18 / sl0 | ARC_SLANT=0 (session 5 as left; reproduced) | 91 / 2 | 15 / 4 / 9 | 93 |
| sl1 | slant | 93 / 1 (SBA2) | 15 / 6 / 9 | 50 |
| sl2 | + ECON_RESERVE | 98 / 0 | 15 / 6 / 9 | 37 |
| sl3 | + ARC_HOPS | 98 / 0 | 15 / 6 / 9 | 50 |
| **sl4** | **+ RESERVE_BAND (the defaults now, with ECON_RESERVE=1)** | **80 / 0** | **14 / 9 / 11** | **35** |
| human | | 81 / 0 (80 on the K nets) | | |
| base | one corridor, two pages, this bench | 106 / 0 | | |

sl4 per arc against the human: middle 26 / 24, north **20 / 20**, south
34 / 36 (SCS0 and SCS1 route with no via where the human spends two;
SCAS 4). Every north lane is 2 vias; the renders
(`tmp/arcprobe/zs4_launchN.png`, `zs4_S.png`, `zs4_Nend.png`) show the
human's shape: the six northern teeth run north-east directly, SA0 /
SA2 / SA4 go north beside the face a pitch apart before turning, the
band runs east under the parts north of DU1 and drops into the
north-east stubs; the south teeth run south-east nested and turn north
into DU1's south face. Hops (sl3) change nothing under the slant: the
pulled lanes were not dying of budget but walled. The gate (~88 / 0)
is passed; the run is 35 s.

**The flag-off chain is byte-identical, with a caveat worth its own line.**
`PLAN_PAGES=1 bash chain_k.sh sl_chk3 28` reproduces `tmp/pg2_{fo_,}k28`
segment for segment and via for via (the session-5 reference `am_chk`
is the same copper). The FIRST attempt did not: run beside another
chain (sl4b), the pages-first planner's first CP-SAT solve -- the same
model, 501 + 186 candidates, 14975 exclusions -- stopped at a different
FEASIBLE solution (obj 729.7 / 22.0 s against 727.2 / 20.3 s), moved
SA1 as well as SCAS, and the chain shipped 32 vias against 34. So
`max_deterministic_time` with 4 workers is reproducible on an idle
machine and NOT under concurrent load (the comment in `chain_k.sh`
claims more than that); an identity check runs ALONE. The default
environment (no PLAN_PAGES) lands at the base's own 36.

**The corner tooth (the second item), built.** `offsets` tests every
band tooth's planned fan-in -- the jog on the tooth's layer, the pull on
the band's -- against the corridor's other copper (the members' teeth
and stubs are hard walls) and the static copper the router sees
(`_fanin_walled`, 0.05 mm samples). A walled tooth takes the OUTERMOST
slot, one pitch beyond the outermost free tooth (the human's SA8 rides
0.37 mm outside SA14), its fan-in is the router's: `route_lane` routes
it as a band-free first hop from the tooth to the lane's hold point at
the region's start, in a window round the hop's own ends twice the
band's half-width wide (room to skirt the band, none to circle the
array -- the whole lane's window let a hop circle DU1 in 707k
iterations), and `cross_reserve` promises nothing there. On the bench
it finds SA8 (north; its pull ran through SA0 / SBA1 / SA12's kept B
copper under the array) and SRST (south; its jog on B would lie 0.13 mm
from SCAS's kept B run).

| arm | | vias / open | in band M / N / S | s |
|---|---|---|---|---|
| sl5 | + corner teeth, whole-lane free-hop window | 80 / 0 | 14 / 10 / 13 | 31 |
| **sl6** | **+ the hop's own window (the defaults now)** | **80 / 0** | **14 / 10 / 13** | **30** |

SA8 routes in band: a 12 mm free hop to the hold point (28k
iterations), then the band, 46.8 mm and 2 vias -- the human's shape --
and the ECON RE-LAY then replaces it with the 35 mm route round DU1's
SOUTH at the same 2 vias (its rule: fewer vias, else shorter), through
the channel the south corridor's exit legs cross. The count is the
human's either way; the shape is a policy question for the re-lay (a
wrap's outermost lane is long by design). SRST is detected but its free
hop fails: its hold point sits on the south band's OUTER edge on F,
where every neighbouring lane's line is already reserved, so its via
cannot land -- the human keeps SRST (and SA7, SCAS) on B for the whole
band, a per-lane layer the arc mode does not have; last call, 2 vias,
the human's count. The middle pays 2 more (SDQ7 0 -> 2) with the north
block shifted a pitch outward for SA8's slot.

**What is NOT done, and what the renders say is left.** (1) The econ
re-lay's length economy on a wrap's outermost lane (SA8 above), and a
per-lane band layer for a corner tooth on the other layer (SRST). (2) The south
band: its wrap merges C10 (east of DU1) and stands its far side at
x 153.15, past the board's edge at 152.4 (the edge clamp guards the wrap
side only), and 7 of 18 lanes still land at last call; the human turns
every south lane north into DU1's south face at x 137.5-140.3 and
reaches the three east-end stubs from below at x 143.5. (3) The middle
bundle at 26 against 24: SDQ3 costs 2 where the human has 0. (4) The
plan: arc membership, band layer and the launch order are still the
sidecar's (`tmp/hbn3_k51.plan.json`); with the comb slanted, the plan's
lever is exactly the teeth's perpendicular order, which the source
berths set.

### The plan choosing the arcs (the third item): the fresh chain says what it needs

`BRAID_ARCS=1` (default 0) is the first, geometric form of "the plan
chooses arc membership and band layer": with no corridors named by the
plan, `braid.setup` splits a geometric corridor whose members share one
destination array by where each stub escapes it along the flow from the
source -- toward the source: the middle; sideways: the arc on that
side; the far face: the arc on the side of the stub's row -- an arc of
fewer than three nets folding back into the middle, the band layer the
majority of the arc's nets' two end layers. It runs in the judge and
the braid stage alike, so the pages-first iterations plan against it.
Nothing board-specific: the flow direction, the box, the escapes.

Measured on the fresh chain (`PLAN_PAGES=1 BRAID_ARCS=1
BRAID_ECON_RESERVE=1 bash chain_k.sh arc1 K`):

| K | pg2 (one corridor, two pages) | arcs |
|---|---|---|
| 28 | 34 / 0 | 50 / 1 (SDQM0): middle 8, N 10 on F, S 10 on F |
| 51 | 115 / 1 | 96 / **19 open** (arc4, with the s0 guard; 10 min braid): middle 9, N 20 on F, S 18 on F -- the arcs' launch regions clamped to a hair, 12 + 6 corner teeth |

The first K51 runs died before grading (the root cause and the guard are
below); with the guard the chain completes and the split loses 19 nets.
Two things the fresh chain shows that the human-ends bench could not:

1. **The arcs are a CHOICE, not a rule.** At K28 one corridor of two
   pages is under capacity and the split costs 16 vias and an open net;
   at K51 it is the human's structure. The choice has to be judged --
   the braid's own count on both plans, as `plan_search` judges chain
   candidates -- and the geometric split is the candidate generator,
   not the decision.
2. **An arc's SOURCE teeth must be the plan's too.** On the bench the
   north teeth are the ends of the human's runs, on one line at U1's
   east face; on the fresh chain the 20 nets the split puts round the
   north have teeth on every face of U1 (span 9.75 mm across a 7.6 mm
   block, the launch leg's centroid INSIDE the array, 12 of 20 teeth
   walled -- all corner teeth), and the south's 18 likewise. The human
   peels the north bundle out U1's north face on B before it is a
   bundle. So the arc variable belongs in the pages-first model, where
   the source moves are chosen: an arc per net, pairs priced only
   within an arc (67 of 101 counted crossings on the human's ends were
   between arcs), the source menu offering N/S-face and edge-hugging
   exits, the arc's band layer a variable the two end layers price --
   the design already written in the session-4 notes, now with the
   comb it needs on the braid's side. That is the next build.

The K51 braid's death was chased three ways, two of them traps: the
chain reported `Killed: 9`; a resident-memory monitor
(`tmp/arcprobe/memwatch.py`) saw the process die at 817 MB, so not
memory; a `faulthandler.dump_traceback_later` run died at 3 s -- and
its crash report names the FAULTHANDLER THREAD itself (`dump_frame` ->
`PyUnstable_InterpreterFrame_GetLine`, SIGSEGV), so a periodic-dump
diagnostic is a crash source on this process, not an instrument. Four
capped braid runs on the same board (arcs off / on, econ reservation
off, whole-band reservation off; `tmp/arcprobe/braid_try.py`) all ran
75 s past the death point without dying, so nothing in the new paths
crashes deterministically; the chain was re-run alone under nohup with
a monitor (arc2) -- and died the same way, inside a blocking call too
(arc3), and alone from a wrapper (`braid_try.py`, exit -9 at 8 s
whatever the output path), so not the tool's parenting either. What
found it: `ps -o vsz=` every second (the VIRTUAL size jumps by 44 GB
between t=6 and t=7 s while the resident set climbs to 770 MB -- the
kernel kills it when the pages are touched), then an in-process
sampler thread printing the main thread's stack every half second
(`sys._current_frames()`, no faulthandler): the death is in
`route_lane -> connect -> build_base_obstacle_map -> segment_blocked_spans
-> _capsule_mask -> np.meshgrid`, a capsule over a virtual-copper piece
whose endpoint lies 24 000 km off the board. The piece is an arc lane's
target: on the arc1 board the north arc's fan-ins end at s 26.4 while
its first stub projects at s 26.1 -- the teeth span every face of U1,
so the pulls are 5 mm -- and with s0 > s1 the region's length is the
1e-6 floor, every slope explodes and `target_o` lands at -1.35e7. The
session-4 guard clamps s1 against the TEETH's base s0; the slanted
comb sets s0 later, from the fan-ins, so `offsets` now clamps s0 to
s1 - 0.2 with a WARNING naming the corridor ("the teeth are not on one
face"), and the corridor refuses and says so instead of dying. The
four concurrent survivors were slower (four on eight cores) and had
not yet touched the pages when capped. The bench arm is unchanged by
the guard (sl7 copper-identical to sl6). `BRAID_ARCS=0` leaves the chain untouched (the split is gated
before the corridor log line).

### Handoff: the plan for the next session (written 2026-09-14 evening)

**Where it stands.** Worktree `bus622-take5` @ 2daff560 + sessions 3-6,
all UNCOMMITTED (`awx/tmp/session6_0914_slant.patch`, 2979 lines; the
flag-off chain is byte-identical to pg2, verified at K28, run alone).
The braid side of the arc mode is ready: given ends like the human's
(bench `tmp/hbn3_k51` + its sidecar) it routes them at the human's
count, 80 / 0 in 30 s, deterministic. The chain side is not: arcs as a
geometric rule lose (K28 50 / 1 against 34 / 0, K51 96 / 19 open against
115 / 1), because the plan chooses berths and source moves for ONE
two-page corridor and the arc's teeth then span every face of U1. The
bench measures the braid given the ends; the chain measures the plan;
never compare the two.

**Goal.** The fresh K51 chain better than 115 / 1 with the arcs chosen
by the plan, K28 / K35 / K41 no worse than pg2's 34 / 65 / 79, about
two minutes a K. Nothing lands unless the ladder says so (the edicts).

**Step 0 -- baselines, alone.** `PLAN_PAGES=1 bash chain_k.sh base K`
for K in 28 35 41 51, ONE AT A TIME on an idle machine (the pages-first
CP-SAT stops at a different feasible solution under concurrent load);
compare with `tmp/pg2_{fo_,}kK.kicad_pcb` by segment and via multisets
(UUIDs differ per run), record the times. K51's identity was not
re-verified this session, only K28's. Bench sanity:
`tmp/arc_arm.sh sl8 "BRAID_ECON_RESERVE=1"` must be copper-identical to
`tmp/sl6_k51`.

**Step 1 -- arc labels on the destination candidates
(`pages_first._solve`).** Every destination candidate `m` in
`st['dmenu'][n]` gets `arc(m)` in {M, N, S} by the rule already in
`braid.arc_split_groups` (flow = destination centre minus source
centre; an escape toward the source is M; sideways, the arc on that
side; the far face, the arc on the side of the row), applied PER
CANDIDATE. Per-net indicators `A[n][a] = sum of xd[n][j] over the
candidates with arc a` are linear. Then price the pair terms
(inversions, crossings: the `ltT` / `ltL` / `same` / `cr` bools) only
within an arc, gated by a `same_arc(a, b)` bool: on the human's ends 67
of the 101 counted crossings were between arcs
(`tmp/session4_0914_probes/phantom_xing.py` is the check). An arc net's
page is its arc's band layer and it never swims. An arc holds at least
`ARC_MIN` = 3 nets or none (a use bool per arc).

**Step 2 -- the band layer per arc.** One bool per arc, F or B. Costs
per arc net: a tooth on the other layer is a birth via, a stub on the
other layer an exit via; the arc mode's exit rule (every exit a leg,
the leg on the other layer for two vias unless the stub is on the band
and nothing lies between slot and stub) is priced as the layer
mismatches first and calibrated against the judge (Step 4).

**Step 3 -- source exits for the arcs (`fanout_from_plan.py`, the
`smenu` built at lines ~229-234 by `menu(p, sgrid, net, own_only=True,
climb=SRC_CLIMB)`).** The human peels the north bundle out U1's NORTH
face on B before it is a bundle. Add tooth candidates on the arc's face
and edge-hugging exits on the band layer at the near face's end, and
price a (tooth candidate, arc) pair by its PULL in the arc's launch
frame: the candidate's perpendicular distance from the arc's outer
edge line (the octilinear leg toward the wrap corner, as
`_arc_spine` draws it) -- a per-candidate constant once the arc is
fixed, so a joint bool per arc-capable candidate carries it. A
candidate is arc-capable only when that pull fits the launch strip
(the fan-ins must end before the first stub; the geometric split's
K51 failure is exactly the fan-ins overrunning the region, and the
braid now clamps and WARNS "the teeth are not on one face").

**Step 4 -- the judged choice (`fanout_from_plan.judge_by_braid` /
`explain_plan`).** Solve twice, arcs forbidden (today's model) and arcs
allowed, judge both by the braid's own plan-only count with the
corridors / arcs / bands passed in the plan dict (so `braid.setup`
builds arc corridors for the judge), keep the lower, and write
`corridors` / `arcs` / `bands` into the `.plan.json` only when the
arcs won. First calibrate the judge on arc corridors: its predicted
vias against the routed 80 on the bench and against `tmp/arc4_k51`.
Gate the whole feature on one env (`PLAN_ARCS=1`), default off.

**Step 5 -- braid-side gaps, each small and independent, each measured
on the bench (`tmp/arc_arm.sh`) with a render:** (a) the south wrap's
far side stands at x 153.15, past the board edge at 152.4 -- clamp it
like the near side and stop merging parts beyond the array's far end
(C10); (b) the econ re-lay pulls a band's outermost lane out of its
band (SA8: 46.8 mm in band, re-laid to 35 mm round DU1's south at the
same 2 vias) -- skip band-corridor lanes in the re-lay, or hold the
re-laid path inside its corridor's tube; (c) a corner tooth on the
other layer whose free hop cannot land its via (SRST) keeps its own
layer through the band, the human's SA7 / SRST / SCAS on B; (d)
`BRAID_ARC_PITCH` 0.35 (the human's is ~0.30).

**Rules.** Every arm reports vias / open / in-band / seconds and gets a
render (`tmp/arcprobe/zoom2.py`, `plan_probe.py`); a repeat run before
a number is believed; commit nothing unless clearly better on K28 /
K35 / K41 / K51 within the time budget -- the arc mode is plan-gated
and byte-identical off, so committing it as opt-in is Andy's call, not
the session's. Keep the router general: geometry, flows, faces, never
a name or a bench constant.

**Traps recorded this session.** CP-SAT drifts under a concurrent chain;
`faulthandler.dump_traceback_later` segfaults its own thread on this
process; py-spy needs root on macOS; macOS has no `timeout` and
`ulimit -v` fails -- a runaway is found with `ps -o vsz=` per second
and an in-process sampler thread (`tmp/arcprobe/braid_try.py`,
`memwatch.py`); s0 > s1 in a band corridor is guarded now.

**Files.** braid.py: `ARC_SLANT`, `ARC_RESERVE_BAND`, `BRAID_ARCS`,
`ARC_MIN`; `_arc_spine`, `offsets` (the slant block, `_fanin_walled`),
`route_lane` (the free hop), `cross_reserve`, `arc_split_groups`.
Probes `tmp/arcprobe/{comb_geom,plan_probe,reserve_probe,zoom2,
flood_probe,vs_human,braid_try,memwatch}.py`, `tmp/arc_arm.sh`,
`chain_k.sh`. Boards: `tmp/hbn3_k51` (bench), `tmp/sl6_k51` (best),
`tmp/arc4_k51` (fresh chain with the split), `tmp/pg2_*` (baseline),
the human's `~/Downloads/bus/00_human_original.kicad_pcb`.

## REVERTED 2026-09-14 (late evening): the arc line is abandoned

Andy's decision after session 7: "Let's give up on the arc idea. Revert
it, and whatever else is not working well." The CODE of sessions 3-7
(the exit-leg term, the swimmer descent, the wrap spine, the arc-corridor
mode, the slanted comb and corner teeth, and the plan choosing the arcs)
is reverted to commit 2daff560 -- `braid.py`, `corridor.py`,
`fanout_from_plan.py`, `pages_first.py` as committed, `arc_plan.py`
removed. All of it was opt-in and inert when off (the flag-off chain was
byte-identical to pg2 at every K before and after), so the default path
is unchanged: PLAN_PAGES=1 gives 16 / 34 / 65 / 79 / 115 (K15..K51),
the old planner what 901108e5 records. The write-ups above stay as the
record of what was tried and measured; the code is archived as
`tmp/session6_0914_slant.patch` (sessions 3-6), `tmp/session7_0914_arcs.patch`
(sessions 3-7, the plan-chooses-arcs work included) and
`tmp/s7/arc_plan.py.archived`; the bench `tmp/hbn3_k51` and its
routed 80/0 (`tmp/sl6_k51`) remain as the measurement that the braid
CAN route the human's ends at the human's count when given them.

The verdict, in one paragraph: on the human's ends the arc mode routes
at the human's 80 vias; on the fresh chain no arc plan ever beat one
corridor of two pages (K51 routed: base 115/0; the arc arms 112/4,
88/16, 122/6 open). The reasons found in session 7, each measured: the
plan's choice of arcs is a search problem the CP-SAT does not solve from
the one-corridor instance in its budget; the launch frame had to be
tied down (the flow turned 45 degrees, not the teeth's centroid nor the
wrap corner); an arc must OWN THE OUTERMOST TEETH of its launch face --
any middle tooth outer to an arc tooth walls its fan-in, and the
walled corner teeth were the open nets -- which with the teeth as they
stand admits arcs of three nets, and with climbed teeth (the human's
riders) blows the pairwise exit-leg pricing up to 281k pairs. What
would remain: compact order variables for the exit legs, a warm start
or decomposition for the arc arm, and a judge that prices a swimmer at
what it routes for (~3.5 at K51, not the flat 2).

**The revert's own trap (21:20-21:45, the same night).** The first K28 run
of the reverted tree graded 32 vias / 1107 segments and its copper
differed from pg2 (a different fanout board, one corridor of 28 where
pg2 has 27 + SA0): it read as "one of the reverted changes was
load-bearing for the default chain". A per-file bisect (each session-6
file's diff applied alone) then gave 34 / 783, copper-identical to pg2,
for pages_first alone, braid alone AND corridor alone -- which no code
cause can explain -- and a plain re-run of the committed tree is
identical to pg2 too. The 32 is the pages-first CP-SAT's SECOND feasible
stop at DET 40 (obj 729.7 against the usual 727.2), the same 32-against-34
pair "The base re-measured, determinism" records under concurrent load;
that run shared the machine with a memory write on a box with 55 MB
free. Rule: one divergent chain run is not evidence of a code change --
re-run it alone before bisecting. The committed tree's ladder was
re-verified afterwards (`tmp/s7/verify.log`, the base10_* boards).

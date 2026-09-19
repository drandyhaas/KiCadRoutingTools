# awx -- the K-bus chain and its evolution (#622)

Routing a fanned-out bus between two BGAs. One PLAN decides both ends of
every net, the FANOUT lays exactly the plan's moves and reports what it
could not, the BRAID routes the lanes in a corridor of two pages -- and
then a POPULATION of routed boards evolves: each world descends by
probing single moves with the real router as the judge, jumps to nearby
worlds, and exchanges ends with the others, until the best board is a
local optimum of every move the menus offer. The records on every rung
came from that evolution.

    bash chain_k.sh TAG 28 41 51                      # the chain: -> tmp/TAG_k<K>.kicad_pcb, graded
    python3 evolve.py TAG 51 --seeds=STEM,... --pop=4 --gens=3 \
            --descend="--rounds=2 --worst=6 --probes=2 --min-vias=2 --coupled=census --widen=0 --grade=inproc --par=4"
    python3 replan.py STEM 51 --from=STEM --out=OUT --mode=incremental --apply=strip ...   # one descent
    python3 evolve_movie.py TAG 51 --gif                # the movie of a run
    python3 make_bench.py BOARD SRC DST OUT             # another array pair, from any board
    bash pose_gate.sh BOARD SRC DST 15 28               # the same pair in every pose

Run everything under the chain's own environment, `PLAN_PAGES=1
PLAN_JUDGE=count PLAN_JUDGE_LEN=lane`. (We have no idea what `awx`
stands for. The name predates every note that mentions it.)

## Where it stands

The bench is `fb_t2q_fresh`: an H3 BGA `U1` to a DDR3 `DU1`, routed over
the coherent K-ladder (`coherent_nets` counts whole rivers, so "K51" is a
48-net problem). Every board below is 0 open, 0 DRC at the routed 0.1 mm
floor, graded independently (`grade_k.py`), and carries the same
out-of-run nets as the reference boards. `tmp/records/` holds each one
with its fanout board and sidecars.

| | K15 | K28 | K35 | K41 | K51 |
|---|---|---|---|---|---|
| the chain alone (the portfolio's best arm) | 16 | 34 | 60 | 74 | 98 |
| **the evolution (2026-09-18/19)** | **14** | **34** | **56** | **64** | **83** |
| human | 22 | 46 | 58 | 70 | **81** |
| how the record was found | a jump world descended | the chain; nothing improved it | a jump world from the 58, descended | the 74 arm descended to 67, that to 64 | an open-net arm descended to 91, then 91 -> 87 -> 85 -> 83 with the climb menus |
| evolution time, this laptop | 2 min | 2 min (the chain) | 7 min | 12 min | 35 min |

**K15, K28, K35 and K41 beat the human; K51 is two vias short of it.**
The 83 carries two grazes of 7 and 8 um under 0.1 on B.Cu -- the
grid-quantisation class the chain's `--clearance-margin 0.1` filters, as
`check_drc` documents -- and the 85 (`tmp/records/k51_85_climbs`) is the
last board clean with no margin at all. 83 is the local optimum of every
single-net move class with the climb menus, at descent thresholds of
three and two lane vias: five generations of descents, near jumps and
crossovers at K51 over two runs stood nothing, and the last run
(`tmp/ev51f`) ended with four distinct 83s -- every jump and crossover
world descended back to 83.

**The second article: zynq** (2026-09-19). `zynq_ad9364` from the
stress corpus (set 4), the Zynq `U1` (CLG400) to its DDR3 `U2`, built by
`make_bench.py --two-layer` in 15 s: 46 pair nets, two refused at the
source, 44 in seven rivers, checkpoints 9 18 26 32 38 42 44. The human
routed this bus on F and B only (the inner layers are planes; 103 vias
over the same 44 nets, every one F-to-B), so the two-layer article is a
fair comparison. The pair points -y, so the chain runs it through one
quarter turn of the flow frame; the frame article (`zynqF`) grades
identically at every rung, and the evolution runs on that.

| | K9 | K18 | K26 | K32 | K38 | K42 | K44 |
|---|---|---|---|---|---|---|---|
| the chain alone | 14 | 16 | 39 | 54 | 68 | 90 | 105 |
| **the evolution** | **12** | 16 | **34** | **50** | **59** | **71** | **88** |
| human | 25 | 45 | 57 | 74 | 86 | 97 | 103 |
| chain wall, this laptop | 33 s | 1 min | 2 min | 3 min | 5 min | 6 min | 8 min |

All 0 open, 0 DRC at 0.1 mm. Below the human at every rung but the
top, where the chain is two over and the evolution fifteen under. The
evolution row: K9/K18/K26/K32 by the population as the H3 ladder runs
it (pop 3, two generations, K9 one; K18 gained nothing), K38 by both the
population (a near-jump world descended) and the descent alone,
K42/K44 by the descent alone carrying `--length=1 --worst=8` (three
rounds: 90 -> 77 -> 71, 105 -> 97 -> 95), then a K44 population seeded
from the 95 (pop 4, two generations, length rule on, 53 min): every
descent of the 95 null, the CROSSOVER of the 95 with the chain's 105
(seven of their fourteen differing ends) descended 108 -> 93, that 93
descended to 89, and a near jump from the 93 (two nets) descended
101 -> 88. Population at the end 88 / 89 / 93 / 95; the 88 is
`tmp/records/zynq_k44_88_pop` (its board-frame copy beside it). The length tie rule -- an equal-via board that
shortens the copper stands -- is what let the K38 descent take nine
vias in one round where the same round without it took four. Where the
human's board never puts more than four vias on a net (one net), the
chain's K44 carries seven nets at four and three at six -- the
multi-divers the descent exists for. Two defects the article shows that
the H3 bench cannot: a net the main spine cannot reach is split into a
one-net corridor and planned onto a FAR-FACE source tooth (a channel
escape through seventeen rows of the Zynq, then straight back: DQ12 at
K38, DQ13 at K44, ~24 mm of hairpin each, priced by the count judge as
a via saved), and at K42 the whole bus becomes one 42-net corridor with
eleven in-band refusals. Renders: `tmp/zynq/img/`.

Two rules the router keeps. **It is general**: no net, face, board or
part name anywhere in the code; every rule is geometric. **It is an
autorouter**: the human's boards are tests, never seeds. The one test
that used the human's ends (their ends, our braid, then our descent)
reached 79, below the human's own 81 -- which is how it was learned that
the gap at K51 is the ENDS, not the realization, and that the human's
ends differ from our menus only in the climb classes. The descent's
menus carry those classes now (`DST_CLIMB=2` in descents, `SRC_CLIMB=4`
in jumps); the CP-SAT plan cannot use them (measured: they made its
solve stop worse), the route-judged descent can.

**A number is meaningless without its arm, and cloud numbers are a
different measurement from local ones.** CP-SAT stops the plan solve at
platform-dependent feasible points, so a chain on Modal starts from a
different plan than the same chain here. Compare cloud to cloud, local
to local. `PLAN_PAGES_CANON=1` is the one canonicalised solve that agrees
across machines; it is an instrument, not a production setting.

**The source stub trim, the served-under-the-part rule, the away-face
ban** (2026-09-19, the second pass over the zynq article). Three things
the article asked for, each general:

* **The source stub trim** (`braid.note_source_joint`, on by default;
  `SRC_TRIM_REACH=0` turns it off) is the source-side mirror of the berth
  trim: at write time every lane is walked from its tooth, every vertex
  projected onto the net's own stub chain, and the deepest splice that
  shortens the copper and grades no worse on the net's scoped DRC stands
  -- the stub's dead tail and the lane's backtrack go, one cross segment
  joins them. Vias never change. Measured, the trim on: zynq K38 68 = 68
  vias with DQ12 58 -> 37 mm, K44 105 = 105 (six lanes, -20 mm); H3 K28
  34 = 34 (SA1 -3.4 mm), K35 60 = 60 (SDQ10 + SDQ13 -28 mm), K41 74 = 74
  (-13 mm), K51 98 = 98 (SBA1 + SDQ11 + SDQ13 -57 mm). A dead-copper
  audit of every board finds no dangling tail at either end -- the berth
  trim is complete on these boards; what it does not catch is a lane that
  never touches its stub again (DQ13 at K44 climbs six millimetres up the
  far face before turning back, and the 3.6 mm splice the trim found took
  5 mm, not 20).
* **The served-under-the-part rule** (`py_router`, `KICAD_FANOUT_SKIP_UNDER=1`,
  `bga_fanout.escape.under_part_candidates`): a ball whose net's every
  off-footprint pad lies inside the ball field -- a ZQ resistor or a
  decoupling cap on the far side, straight under it -- gets no escape
  stub; its connection is a via at the ball and a short far-side track.
  The H3 bench's `SZQ` (ball V10 to R6.2, 0.09 mm away on B.Cu, drawn a
  2.4 mm stub toward the edge) is the case; on the corpus H3 board the
  switch drops exactly that stub and touches no bus net. It rides #472's
  deferral plumbing, so the balls stay routable through the route steps'
  zone exemption. Opt-in: the always-on form is a fanout-laid pad drop
  (via at the ball, track to the pad), not built. A bench rebuilt with it
  loses `SZQ` from the K51 ladder, which is right: it is not a bus net.
* **The away-face ban** (`SRC_AWAY_BAN=1`, `escape_moves.away_faces`: the
  source faces whose outward direction runs against the source-to-
  destination vector, in any frame) removes the far-face teeth at the
  menu. It is what stops the hairpins rather than trimming them (zynq
  K38: DQ12 58 -> 25 mm, the run 1172 -> 1139 mm) -- and it is not a
  default, because the menu's deep-ball-met-on-the-far-side move is real:
  with the ban zynq K38 68 = 68 but K44 105 -> 106, H3 K28 34 = 34 (copper
  652 -> 643 mm) but K35 60 -> 61 (878 -> 919 mm), K41 74 -> 82. Worse on
  three of five rungs is not a default; it stays a knob the evolution can try.

**The pack, made to work on a finished board** (2026-09-19, Andy: "I
see easy shorter paths", then "still slack in the outer lanes"). Six
things were wrong, in the order they surfaced:

* An evolved board's `.pack.json` is the LAST PROBE braid's sidecar, and
  a probe lays one lane and its coupled set -- on the K44 record the
  packer saw one corridor with one lane of 44. The whole-board mode
  (`pack_board.py BOARD --fanout BOARD_fo --nets ... --src U1 --passes 4`)
  packs every lane of the run, one corridor per lane, the passes
  repeated so each sees the room the last one left.
* The FOLLOW force -- each lane snapped into the tube of the lane packed
  before it -- copies that lane's jogs, and from the wall inward the outer
  lanes copy the router's still-ragged inner ones: the K18 bundle read as
  a wave. Four passes with the follow grew K18's lanes 392.6 -> 394.8 mm;
  taut (`PK_FOLLOW=0`, the whole-board default) they went to 386 and bend
  together at the cap pads.
* A via moved by its lane was checked against copper only; the hole-to-
  hole rule (net-agnostic, 0.25 mm here = 0.40 centre to centre for these
  vias against copper's 0.355) is now a disc per drilled pad and per
  other via in the via world, read off the board.
* "Routed minus fanout" is not the lane on an evolved board: the derive
  step keeps lane fragments as the berth's copper, so WE and A3 came out
  in pieces with fanout-matched gaps and were skipped. The lane is now
  everything outside the two STUB CHAINS -- the copper reachable from a
  pad through fanout-matched segments only -- with the chains' tips as
  its ends, taken at their exact coordinates (the packer chains on four
  decimals; a rounded 80.070 never met its 80.0703).
* A run that meets its via inside the annulus reads as a break: a tiny
  gap (WE: 22 um) is snapped onto the centre, a larger one bridged with a
  link -- not snapped, since moving one end of a 2.1 mm segment by 0.106
  mm tilted it into a foreign via's clearance (K32 DQ8 vs DQ14). A 35 um
  duplicate whose both ends lie on the chain (A0) is dropped as a loop;
  112 lanes had come back "unchained" over it.
* The grade is the gate: a scoped DRC before any edit and after the
  passes, and every lane a new violation names goes back to the copper
  it came with. The emitter's own piece validation had let a re-emitted
  end into a via's clearance.

* Two more, from the renders: a kink -- the any-angle repair replaces
  an unclear grid leg by the string's chords between its ends, and an
  elbow whose SECOND leg was unclear keeps its first, a 45-degree leg to
  the corner and a jog back (K44 BA2, 2.5 mm out and 0.79 back where the
  string ran straight). A vertex the path doubles back at now goes when
  the chord past it is clear (`_unkink`; 23 -> 6 sharp turns at K44).
  And the source trim runs again on the finished board before the pack
  (its splice, its DRC), because a board the evolution assembled from
  probes carries backtracks no braid saw whole: 34 mm at K44.

| K | 9 | 18 | 26 | 32 | 38 | 42 | 44 |
|---|---|---|---|---|---|---|---|
| lanes, mm (best -> trimmed + taut pack) | 185 -> 181 | 393 -> 383 | 662 -> 630 | 801 -> 769 | 954 -> 915 | 1054 -> 996 | 1203 -> 1104 |
| run copper, mm | 221 -> 217 | 468 -> 458 | 774 -> 736 | 971 -> 940 | 1171 -> 1121 | 1300 -> 1235 | 1477 -> 1350 |

Vias unchanged on every rung, 0 open, 0 DRC with and without the
margin, every lane packed.

* **The coupled re-lay** (`--relay`, on; before the pack): a splice the
  trim refused because another lane of the run stands between the stub
  and the backtrack is tried again with that lane LIFTED, and the lifted
  lane is then routed anew between its own two tips by the production
  router (`connect`, the chain's config) on the board as it stands. It
  is kept only when the pair's copper is shorter, no lifted lane gained
  a via, and the scoped DRC over the nets involved names nothing new;
  else every piece goes back. 1.2-1.4 s an attempt. On this article it
  keeps nothing: A13's neighbour DQS1_N comes back from the router with
  four vias where it had two, and is refused.
* **Why the south tooth needed no re-lay.** DQ13's 11 mm backtrack at
  K44 was walled by A10 -- but by A10's own HAIRPIN, one row over: its
  far-face stub and its lane straight back, both crossing DQ13's splice
  line. A10's own trim removes exactly that, and the pre-pass had asked
  DQ13 first, in ladder order. The pre-pass now runs in ROUNDS, the stub
  chains walked again on the board as it stands each round (a chain
  walked once still describes the tail a splice cut, and the next round
  books the saving twice), until a round splices nothing: DQ13 goes in
  round two, 22.5 mm, and round three finds nothing. 56 s for the whole
  K44 pack, the rounds' scoped DRC calls being most of it.
* What a lane keeps after all that is its wrap: a lane that goes the long
  way round its bundle at the same via count is invisible to the chain's
  judge, which prices vias and never copper.

**The source stub trim, the served-under-the-part rule, the away-face
ban** (2026-09-19, the second pass over the zynq article). Three things
the article asked for, each general:

* **The source stub trim** (`braid.note_source_joint`, on by default;
  `SRC_TRIM_REACH=0` turns it off) is the source-side mirror of the berth
  trim: at write time every lane is walked from its tooth, every vertex
  projected onto the net's own stub chain, and the deepest splice that
  shortens the copper and grades no worse on the net's scoped DRC stands
  -- the stub's dead tail and the lane's backtrack go, one cross segment
  joins them. Vias never change. Measured, the trim on: zynq K38 68 = 68
  vias with DQ12 58 -> 37 mm, K44 105 = 105 (six lanes, -20 mm); H3 K28
  34 = 34 (SA1 -3.4 mm), K35 60 = 60 (SDQ10 + SDQ13 -28 mm), K41 74 = 74
  (-13 mm), K51 98 = 98 (SBA1 + SDQ11 + SDQ13 -57 mm). A dead-copper
  audit of every board finds no dangling tail at either end -- the berth
  trim is complete on these boards; what it does not catch is a lane that
  never touches its stub again (DQ13 at K44 climbs six millimetres up the
  far face before turning back, and the 3.6 mm splice the trim found took
  5 mm, not 20).
* **The served-under-the-part rule** (`py_router`, `KICAD_FANOUT_SKIP_UNDER=1`,
  `bga_fanout.escape.under_part_candidates`): a ball whose net's every
  off-footprint pad lies inside the ball field -- a ZQ resistor or a
  decoupling cap on the far side, straight under it -- gets no escape
  stub; its connection is a via at the ball and a short far-side track.
  The H3 bench's `SZQ` (ball V10 to R6.2, 0.09 mm away on B.Cu, drawn a
  2.4 mm stub toward the edge) is the case; on the corpus H3 board the
  switch drops exactly that stub and touches no bus net. It rides #472's
  deferral plumbing, so the balls stay routable through the route steps'
  zone exemption. Opt-in: the always-on form is a fanout-laid pad drop
  (via at the ball, track to the pad), not built. A bench rebuilt with it
  loses `SZQ` from the K51 ladder, which is right: it is not a bus net.
* **The away-face ban** (`SRC_AWAY_BAN=1`, `escape_moves.away_faces`: the
  source faces whose outward direction runs against the source-to-
  destination vector, in any frame) removes the far-face teeth at the
  menu. It is what stops the hairpins rather than trimming them (zynq
  K38: DQ12 58 -> 25 mm, the run 1172 -> 1139 mm) -- and it is not a
  default, because the menu's deep-ball-met-on-the-far-side move is real:
  with the ban zynq K38 68 = 68 but K44 105 -> 106, H3 K28 34 = 34 (copper
  652 -> 643 mm) but K35 60 -> 61 (878 -> 919 mm), K41 74 -> 82. Worse on
  three of five rungs is not a default; it stays a knob the evolution can try.

**The pack, made to work on a finished board** (2026-09-19, Andy: "I
see easy shorter paths"). Three things were wrong, in order of size:

* An evolved board's `.pack.json` is the LAST PROBE braid's sidecar, and
  a probe lays one lane and its coupled set -- on the K44 record the
  packer saw one corridor with one lane of 44 and packed that. The
  whole-board mode (`pack_board.py BOARD --fanout BOARD_fo --nets ...
  --src U1 --passes 4`) derives every lane from the routed/fanout pair
  (`replan.lane_items`), takes each lane's two ends from its own copper
  (the trims split stubs, so the fanout board's tips need not exist on
  the routed one), packs one corridor per lane and repeats the passes so
  each pass sees the room the last one left.
* The FOLLOW force -- each lane snapped into the tube of the lane packed
  before it -- copies that lane's jogs wherever they are, and from the
  wall inward the outer lanes copy the router's still-ragged inner ones:
  the K18 bundle read as a wave, every lane bending at a different
  height. Four passes with the follow grew K18's lanes 392.6 -> 394.8
  mm. Taut (`PK_FOLLOW=0`, the whole-board default) they went 392.6 ->
  386 mm and bend together where the cap pads are.
* A via moved by its lane was checked against copper clearance only;
  KiCad's hole-to-hole rule is net-agnostic and 0.25 mm between drills
  here (0.40 mm centre to centre for these vias, where the copper rule
  allows 0.355). Two taut-packed vias settled at the copper distance and
  `check_drc` named them (K26, K32). Every drilled pad and every other
  via is now a disc in the via world at drill/2 + the board's rule +
  this via's drill/2.

| K | 9 | 18 | 26 | 32 | 38 | 42 | 44 |
|---|---|---|---|---|---|---|---|
| lanes, mm (best -> taut pack) | 185 -> 185 | 393 -> 386 | 662 -> 637 | 801 -> 776 | 954 -> 932 | 1054 -> 1010 | 1118 -> 1058 |
| segments | 163 -> 142 | 1124 -> 309 | 1389 -> 640 | 3071 -> 875 | 3506 -> 995 | 3294 -> 1197 | 3854 -> 1378 |

Vias unchanged on every rung, 0 open, 0 DRC with and without the
margin. What is left is the ORDER: a lane that wraps the long way round
its bundle at the same via count is invisible to the chain's judge,
which prices vias and never copper, and no packer can move a lane
across its neighbours.

## How it works, end to end

**The chain** (`chain_k.sh`) makes the seeds. `coherent_nets.py K` picks
the first K routable nets of the coherent ladder (`k_ladder_coherent.txt`:
whole rivers, tightest first). `flow_frame.py` turns the pair by the
quarter turn that points source to destination along +x and turns the
result back, so any of the four poses is the same computation.
`fanout_from_plan.py` plans both ends of every net with one CP-SAT
(`pages_first.py`: a destination move, a source move and a page per net,
two crossing-free chains, a swimmer priced at 100) and realises the plan
with the production fanout engine in a realise-and-confirm loop (a move
the engine did not lay as asked leaves the menu and the round re-plans).
Two fanout arms (`SRC_REFAN_JOINT` 0 and 1) and two braid arms per fanout
board (the sidecar's pages-first marker on and off) give four routed
boards; `pick_braid` keeps the best by (open, vias). Deduplicated by
copper, all four are the population's seeds -- the K51 record's lineage
began in an arm with two nets open.

**The braid** (`braid.py`) routes a fanout board: corridors from the
geometry (`corridor.py`), a relaxed spine per corridor, launch and target
orders from the lanes' offsets, the two-page schedule (`schedule.py`),
every lane routed by the production grid router inside its band
(`connect.py`, the Rust A* behind it). A refused lane climbs a rescue
ladder -- its band widened, both layers opened, then a free window --
and what is still refused gets a blocker-directed rip: the router's
blocked frontier attributed to this run's lanes, a min-cut probe naming
the cut set, victims re-laid or negotiated one level down.

**A world** is a fanout board with its plan sidecar and the routed board
with its braid record, graded `(open nets, DRC, vias)`. **The evolution**
(`evolve.py`) keeps a population of them and runs three operators, each
one subprocess:

* **descend** (`replan.py`): the route as the judge. Each round takes the
  worst nets -- refused ones, then those carrying the most lane vias
  (`--min-vias`, two at the frontier) -- and for each ranks its other
  classes at both ends by the plan model, screens them with an engine
  dry run, and PROBES the survivors: the net and its COUPLED SET (the
  lanes its new end conflicts with, the lanes the braid's own census
  says walled it, the co-moved berths) are stripped to their fanout
  copper, the moved end is re-fanned by the engine, the set is braided
  alone in the frozen field, and the board is graded. A probe that
  grades strictly better STANDS, and in incremental mode its board IS
  the board the next net is probed on. The round ends by deriving the
  next fanout board from the routed one (its lanes stripped) so the
  standing ends are exactly the laid ones. Monotone by construction.
* **jump** (`replan --perturb=N`): a few random nets moved to random
  other classes through the same probes, the landing taken whatever its
  grade. Lands a move or two away, routed.
* **cross** (`replan --cross=STEM_B`): B's ends asked for on A's board
  for a random half of the nets whose ends differ, one probe each, taken
  where they leave no net open. Population members differ in a handful
  of ends (2-10 of 41 at K41), so this is a few probes.

Every new jump and crossover world is descended in the same generation
before it is judged ("jump, then evolve to its local minimum, then
judge"); selection is elitist on exact grades, deduplicated by copper; a
world whose descent gained nothing is CLOSED and never descended again;
and every probe, engine screen and CP-SAT solve is memoised so the same
question on the same copper is read back. Descents run their probes on
resident worker processes (`--par=N`).

## The descent, in detail (`replan.py`)

    python3 replan.py STEM K --from=STEM --out=OUT --mode=incremental --apply=strip \
        --rounds=2 --worst=6 --probes=2 --min-vias=2 --coupled=census --widen=0 --grade=inproc --par=4

| | |
|---|---|
| `--from=STEM` | the world: `STEM_fo.kicad_pcb` (+ `.plan.json`) and `STEM.kicad_pcb` (+ `.log`, `.pack.json`, `.census.json`) |
| `--worst=N`, `--min-vias=V` | the nets probed each round: refused first, then the N with the most lane vias (at least V; their vias off the board minus their ends' vias) |
| `--probes=P` | candidates probed per end (the top P of the ranked, screened menu); joint tooth-and-berth pairs are probed too |
| `--coupled=census` | the re-lay set: the end's conflicts + the braid's blocker census + co-moves (`chord` adds every lane crossing the chord between the ends -- measured too big at K51: 11-25 lanes re-laid, 110-144 vias) |
| `--widen=N` | answer a local refusal with room, N times (off: widened probes cost 60-180 s and never stood) |
| `--mode=incremental --apply=strip` | a standing probe's board is the next board; the round's fanout board is DERIVED from the routed board by stripping the lanes (the re-fan apply was unfaithful) |
| `--grade=inproc` | the checks in-process; a probe's grade is SCOPED to the nets it changed when the reference board is DRC-clean (same answer, less than half the time) |
| `--par=N` | N resident probe workers (`probe_worker.py`): each holds the round's Board and applies the parent's advances; one net's candidates are probed N at a time, the engine screens too |
| `--perturb=N --seed=S` | the near jump; `--perturb-tries` candidates per net |
| `--cross=STEM_B --cross-frac=F --seed=S` | the probe crossover |
| env `DST_CLIMB`, `SRC_CLIMB` | the climb classes in the menus (the descent runs `DST_CLIMB=2`; the chain's solve cannot afford them) |
| env `PROBE_LADDER` | `open` (default): a probe braid's rescue ladder starts at the rung that opens both layers; `full` is the full braid's ladder |
| env `PROBE_MEMO`, `PROBE_MEMO_DIR`, `PROBE_MEMO_CODE` | the memo (`tmp/memo/k<K>/{probe,screen,closed}`), keyed on copper, move, code hash and knob hash; `0` off; a pinned code hash carries the memo across an edit known not to change copper |

A probe's log line says what the engine laid (exact / in class / another
class -- a substitute the engine lays instead of the ask is probed as the
ask from then on), what was re-laid with it, and the whole-board grade
against the reference: `STANDS`, `rejected`, or `unjudged` (the local
braid refused a lane with everything else frozen, which is not a verdict
on the move). A move the engine will not lay is banned at that net for
the run.

## The population (`evolve.py`)

    python3 evolve.py TAG K --seeds=STEM[,STEM...] [--pop=4] [--gens=3] [--jumps=2] [--cross=1]
        [--jobs=1] [--jump=near|chain] [--jump-nets=2] [--cross-mode=probe|chain]
        [--descend="..."] [--descend-env="DST_CLIMB=2"] [--jump-env="DST_CLIMB=2 SRC_CLIMB=4"] [--seed=N]

Seeds are chain stems (`STEM_fo_k<K>` + `STEM_k<K>`, every portfolio arm
imported) or replan stems. Each generation: every population member
descends; `--jumps` near jumps and `--cross` crossovers land; the new
worlds descend; selection. Outputs `tmp/TAG/g<N>/...`, `tmp/TAG/best_k<K>`
whenever the best changes, the ledger `tmp/TAG/evolve_k<K>.json`.
`--jump=chain` and `--cross-mode=chain` are the retired operators (a
plan-level re-solve with class bans, and the hold channel over two
parents): 665 s and 214 s at K51, landing 84..141 and worse than either
parent; kept for the record.

`evolve_movie.py TAG K [--view X0,Y0,X1,Y1] [--gif] [--verify]` films a run
from its ledger: one canvas per generation, the population row, each
descent under its parent, jump and crossover worlds with lineage arrows,
per-probe steps with the copper that changed lit and ghosted, a lineage
ribbon. `--runs TAG,... --descents DIR,...` films several runs and
standalone descents as one continuous evolution, worlds identified by
their copper; `tmp/movie/k51_day.mp4` is the whole K51 lineage 98 -> 83.

`--jobs` runs operators side by side; on this 8 GB machine one job with
four workers is the budget (a worker is 300-450 MB, and six workers with
two parents got a run killed for memory), so a laptop population runs
`--jobs=1 --par=4`. Each operator is the shape of a cloud container.

## Speed, measured (this laptop: 8 cores, 8 GB)

| | now | before (2026-09-18 morning) |
|---|---|---|
| one probe, serial | 3.7 s | 7-9 s |
| the K51 null descent, 31 probes | 50 s with four workers, 120 s in one process, 5 s warm from the memo | 216 s |
| the K41 descent 67 -> 64, 55 probes | 77 s with four workers, 205 s in one process, 13 s warm | 440 s |
| a jump | 20-60 s, landing a few vias away | 665 s (a chain), landing 84..141 |
| a crossover | 14-36 s, landing near the parents | 214 s (a chain), landing 96 from 64 x 66 |
| the chain, K41 / K51 | 124 s / ~345 s | 147 s / 372 s |
| a two-generation K41 population | 1241 s | 5430 s |
| a two-generation K51 population | 1028 s (550 + 475) | 7503 s for three |

What made it, each checked for identical verdict lines on the K51 null
descent and the same standing moves on the K41 descent:

* **The memo** (`probe_memo.py`): a probe's verdict is a function of the
  copper of every net outside its coupled set, the fanout copper the set
  keeps, the move and co-moves, the code (a hash over every `.py` of
  `awx/` and `py_router/` and the router binary) and the knobs. A hit is
  read back with the boards the original probe wrote; screens and closed
  worlds the same way. Warm re-descents are seconds.
* **Resident workers** (`probe_worker.py`): the braid called in-process
  (`braid.run`), the round's Board built once per worker, N candidates
  of one net probed at once. Utilisation with four workers is 52-60
  percent (a menu of about five is a wave of four then one).
* **The scoped grade**: on a DRC-clean board every new violation
  involves a changed net, so the DRC and connectivity checks run over
  the coupled set only; the fanout board's graze check and the realize
  gate the same way, in-process. 0.56 -> 0.25 s a grade, same answer.
* **The probe ladder starts where it lands**: over 436 probe braids,
  lanes landed on the ladder's first two rungs 16 and 16 times and on
  the third (both layers open) 355 times, and a rung costs 0.06-0.33 s
  whether it lands or not. Probes start at the third rung.
* **A CP-SAT solve read back instead of run** (`solve_memo.py`): the
  plan solver is deterministic, so the model's text plus its parameters
  is an exact key; a hit is replayed through the caller's solver with
  every variable fixed. A fanout arm 35 s cold, 12 s warm; the chain's
  second arm shares all four solves with the first.
* Smaller: probe braids skip the smoother (`smooth_board.py` smooths the
  final board once, byte-identical to the braid's own); lane spans cached
  on each move; four of a probe's twelve board parses gone; sidecars
  copied rather than re-scanned inside a probe; `taut_fast` chunks its
  Hausdorff and freezes a string past 5,000 points (one diverged string
  once asked for 26 GB).

Where a probe's 3.7 s goes now: the braid about 2 s (Rust A* and Rust
obstacle stamps a third each of that, the corridor band strips and the
Python map build the rest), the engine's re-fan and realize about 0.3 s
(the fanout engine's cost is stamping its occupancy grid, not its
under-pad search, which is 5 ms a call), the scoped grade 0.25 s, board
parses and writes the remainder. Measured and left off: a whole-board
obstacle map cloned per connect (`CONNECT_MAP_CACHE`; copper identical,
a full braid 42.7 -> 44.1 s, the ladders too short to repay a 97 ms
build), the descent without the engine screen (`--screen=0`; same moves,
not faster), a Rust port of the under-pad search (1 percent of a
descent), batching the post-route distance checks (2 percent of a braid).

## The chain's other pieces

**The pages-first planner** (`pages_first.py`, `PLAN_PAGES=1`). One CP-SAT
chooses a destination move, a source move ({the tooth as it stands} +
the source menu) and a page for every net; two nets on one page are
never inverted between the launch and target orders; a net may still
swim at 100 vias. Keys are the braid's own slots (`braid_slots`: corridors
built on the seed plan), `verify` runs the braid's planner on the answer,
and a damped loop re-solves only the nets it swims, each barred from the
berth that swam. Deterministic: one worker under `PLAN_PAGES_DET`
deterministic time, never a clock. The judge is the braid's own count
plus lane length (`PLAN_JUDGE=count PLAN_JUDGE_LEN=lane`). Its objective
is measured ANTI-correlated with the routed count at K41 and K51 -- a
proven-optimal solve routes worse -- which is why the plan is a seed and
the route-judged descent does the optimising.

**The braid's arms.** `BRAID_EXACT_PAGES` / `PLAN_PAGES_SIDERS` (the
sidecar's marker, the B arm turns both off); `BRAID_ATTEMPTS` (6: the
launch pitch widened) and `BRAID_BUDGET_X` (the rescue budget); `BRAID_LADDER`
(`full` | `open`); `BRAID_SMOOTH` (the octolinear smoother at write time);
`BRAID_PACK=1` (`pack_board.py`: every lane a taut string against its
neighbour, far fewer segments, vias unchanged -- opt-in, unmeasured what
the segments buy). Every budget is in work (judge calls, HiGHS nodes,
CP-SAT deterministic time), never wall clock.

**Grading** (`grade_k.py BOARD NETS`): connectivity scoped to the run's
nets, whole-board DRC at the routed floor with `--clearance-margin 0.1`,
the via census over the run's nets. `via_census.py`, `census_vs_human.py`
break a board down per net.

## One source for every routing number (`rules.py`)

`awx/rules.py` is the single definition of the chain's design constants;
every module's constant defaults to it and every stage installs from it
(`rules.install_defaults()` in each entry point). This exists because a
swimmer was once priced five different ways. It does NOT resolve numbers
from the board, on purpose: `py_router` already does that resolution in
one place, and the topo chain will be driven by the main router with the
geometry supplied (`Rules.from_router_config(cfg)` is the seam).

| quantity | source | value |
|---|---|---|
| spec clearance | `rules.SPEC_CLEARANCE` -> `topo_strings.SPEC_CLEAR` | 0.1 (fanout, grade, the project written) |
| braid hug clearance | `Rules.hug` -> `braid.CLEAR` | 0.105 = clearance + 5 um |
| lane track / fanout track | `rules.TRACK` / `Rules.fan_track` | 0.127 / 0.1 -- the board carries two widths on purpose |
| via size / drill | `rules.VIA_SIZE` / `VIA_DRILL` | 0.25 / 0.15 |
| lane slice, lane pitch, exit pitch | `Rules.lane_slice` / `.lane_pitch` / `.exit_pitch` | 0.232, 0.35, 0.38 |
| band tip | `Rules.band_tip` | 0.9 = array pitch / 2 + the engine's exit margin |
| hole-to-hole / edge | `Rules.hole_to_hole` / `.edge_clearance` | read off the board, tighten-only |

`braid.CLEAR` (0.105) and `topo_strings.SPEC_CLEAR` (0.1) are different
quantities; `TOL`, `STEP`, `MARGIN`, `CAP`, `PROX_TRACK`, `HW_COL` look
shared and are not.

## Measuring honestly

- **A board with open nets has artificially low vias.** Only 0-open
  boards compare, and `better()` requires 0 DRC.
- **A single K is not a result.** Judge on K28, K35, K41 and K51
  together; run-to-run spread on one board is 2-3 vias.
- **Quote the arm with the number.**
- **There are no clocks.** Every budget is in work; a clock budget makes
  a slower machine answer differently, not later (two identical cloud
  runs of one baseline came back 72 and 58 vias).
- **Verify every new flag byte-identical with the flag off**, and every
  speed change by identical verdict lines on a recorded descent.
- **Grade at the routed floor with the right checker**, and re-verify a
  "clean" board's connectivity separately from its DRC.
- **The profiler inflates hot Python rows three to five times.** Time
  without it before deciding what to port or cache.
- **Look at the renders.** `../py_router/route_render.py`; the copper is
  the plan.

## What the human does, and what the K51 gap is

The human never routes a net above two vias: at K51, 40 nets at exactly
two, 7 at zero. Class each via as source-end, destination-end or
mid-field and the dominant pattern is one via at the destination and one
in the FIELD (21 of 41 two-via nets), a mid-field via on 32 of 41: a lane
that holds F at both pads and dives once where the lanes it crosses are
on F. That is the F-block law, exact on 47 of 47 nets on both boards:

> vias = 2 x (maximal blocks of crossing partners on the lane's own pad layer)

so the run structure costs, not the crossing count -- the two boards
carry the same crossings (337 against 338). Our double-divers (two
blocks, four vias) are load-bearing: `joint_floor.py --cap 2` is
INFEASIBLE over our own paths, and where the cap can be met it costs
six vias, because five double-divers buy eight free rides. The channel
itself is two-via-infeasible from about K=16 on uniform permutations
(`synth_bus.cap_sat_feasible`). So the directive "no net above two vias"
is not a target; what the human has is better ENDS, and the descent with
the climb classes in its menus is what closes that gap from 98 to 83.

<img src="img/k51_human.png" alt="The human's K51 on the original board" width="760">

*The same 48 nets as the human routed them: 81 vias, most nets on one
layer between two escape vias, the address nested round the destination.
A benchmark to approach, not a pose to match.*

## The ingredients, in pictures

<img src="img/k28_corridor.png" alt="K28 on the bench" width="760">

*K28, two pages. A front lane and a back lane cross for free; a page lane
keeps its layer through the schedule region and pays a via only where its
tooth or berth is on the other layer.*

<img src="img/k41_east_face.png" alt="The east face at K41" width="520">

*Far-face exits: a berth on the destination's far face is a side exit of
the main corridor whose leg lies beyond the array.*

<img src="img/k41_rip_sba2.png" alt="SBA2 after the rip at K41" width="760">

*The blocker-directed rip: SBA2 (highlighted) was refused at the last
call, boxed by lanes routed before it. Its blocked frontier named the
lanes on it, the min-cut probe found the cheapest crossing set, SCKE1
was ripped, SBA2 routed, SCKE1 negotiated in turn. This is what first
completed K41 and K51.*

<img src="img/spine_chanD_k15_chord.png" alt="chanD K15, the straight chord" width="400"> <img src="img/spine_chanD_k15_relaxed.png" alt="chanD K15, the relaxed spine" width="400">

*The spine. With a straight chord the lanes are squeezed under the part
one by one (24 vias); with the relaxed medial spine, the default, the
ribbon rounds the part in 45-degree legs (26 vias, far fewer segments).*

<img src="img/gate_mirror_article.png" alt="The bench turned over" width="760">

*The pose gate: the bench flipped through its plane, every part on the
other face, every stub on the other layer. Every isometry grades as the
control to the via and the segment; the selector and the braid each run
a pair in the pair's own canonical frame.*

<img src="img/zynq_k44.png" alt="The second array pair, all 44 nets" width="380"> <img src="img/zynq_k44_human.png" alt="The same 44 nets as the human routed them" width="380">

*The second article (`make_bench.py --two-layer`, zynq to DDR3), all 44
nets, after the evolution: 88 vias against the human's 103
(`img/zynq_k44_human.png` is the human's, meanders and all). Three general defects had to
be fixed before this article ran at all, none visible on the H3 bench:
the flow frame turned point tokens at depth 2 only (a board-level copper
polygon and every zone stayed put while the pads turned; the verifier
also assumed orthogonal pads), `dedupe_boards.py` let the parser's
warning onto the stdout the chain word-splits into its board list, and
the evolution never handed `--board`/`--dest` to its descents. Each fix
is byte-inert on the H3 bench (K28: 34 vias, 786 segments, as recorded).*

## The tools

| | |
|---|---|
| `chain_k.sh` | the chain; `grade_k.py`, `via_census.py` grade it |
| `fanout_from_plan.py`, `pages_first.py` | the planner and both fanouts |
| `braid.py` | the router: corridors, schedule, lanes, ladder, rip; `braid.run` is the callable form |
| `select_moves.py`, `escape_moves.py`, `plan_ends.py` | menus (with climbs), conflicts, plan cost |
| `source_realize.py` | realise a source plan with the production engine, audited per dimension |
| `schedule.py`, `corridor.py`, `connect.py`, `topo_strings.py`, `taut_fast.py` | pages, corridors, the real router, the taut relaxation |
| `replan.py` | the descent, the near jump, the probe crossover; the route as the judge |
| `evolve.py` | the population: descend / jump / cross, elitist, closed worlds |
| `evolve_movie.py` | the movie of a run, or of several runs stitched by copper |
| `probe_memo.py`, `probe_worker.py`, `solve_memo.py` | the memo, the resident workers, the plan-solve memo |
| `smooth_board.py` | the octolinear smoother once over an assembled board's lanes |
| `dedupe_boards.py` | boards identical by copper (the portfolio, the population) |
| `flow_frame.py`, `pose_gate.sh`, `make_bench.py`, `rotate_board.py`, `mirror_board.py` | the canonical frame, the poses, articles from any board |
| `human_at_k.py`, `census_vs_human.py`, `cmp_copper.py`, `wall_probe.py` | the human's count at a K, per-net comparisons, copper diffs |
| `joint_floor.py`, `ledger_cal.py`, `cut_ledger.py` | floors: the non-circular MILP over a board's own paths (`--cap N`), the per-net DP floor vs slack, the Maley cut capacity of a plan |
| `synth_bus.py`, `synth_ladder.py` | the synthetic channel with a known optimum (below) |
| `pack.py`, `pack_board.py`, `collapse_dives.py` | opt-in post-passes |
| `modal_k.py`, `arms.*.json` | cloud arms, one container per (arm, K); `return_board`, `return_files` bring artifacts back |
| `plan_loop.py`, `plan_feedback.py` | the retired plan-level loop (below); byte-identical unset |

## What this adds to `py_router`

**`KICAD_SEG_DIST_EXACT=1`** (default OFF since 2026-09-19; changes every
board when on): the segment-to-segment distance in `single_ended_routing`
is exact -- four point-to-segment distances -- instead of a 0.02 mm
sampled sweep whose minimum was always at or above the truth. Only ever
more conservative. It ran ON by default while the ladder records up to
2026-09-19 were measured, so a replay of those needs `=1`; it is off now
so that merging this branch does not change main's behaviour, and it
owes the corpus A/B before it becomes the default anywhere.

**`generate_bga_fanout(..., escape_dir_hints=...)`**: a per-pad planned
escape, a bare face or a full move; the under-pad engine follows a full
move in its plan-follow phase, negotiates a blocked ball against the same
call's escapes, degrades along the least damaging dimension, and reports
every ball per dimension in `pcb_data._fanout_plan_report`.

**`check_drc.run_drc(..., pcb_data=)`, `check_connected.run_connectivity_check(..., pcb_data=)`**:
a caller with the board parsed hands it over (additive; the default
parses as ever).

## The synthetic harness

Every number above comes from one bench against one human. `synth_bus.py`
writes a case with a known answer -- a 2-layer board at the bench's
process, two arrays across a channel, K nets in a chosen pin pattern, a
`.truth.json` -- and `synth_ladder.py` runs batches through the chain.

```bash
python3 synth_bus.py --self-test              # the truth model checks itself
python3 synth_bus.py out.kicad_pcb --k 15 --pattern interleave
python3 synth_ladder.py --batch b1 [--regrade]
```

Its truths: `lb = 2(K - LIS)` always; `opt`, the best whole-lane
solution; `dp`, the best over all routings (K <= 22); `pages_model`, the
exact optimum of the planner's own model at any K; `channel_lower_bound`,
a bound below every routing in the channel-confined class (the bench
leaves that class: a fifth of the bus reaches its pad through or around
an array, so K28 routes ten below its own floor, and the bound is a
diagnosis, not a scorecard). It is what showed the planner's objective
DEGENERATE (constant across plans that route 12 vias apart) and the
escape move worth 42 percent of the floor on the bench.

## Measured dead ends (do not rebuild)

* **The plan-level loop** (`plan_loop.py`: the route's verdict fed back
  into a pages-first re-solve as class bans, prices and a hint, one full
  chain per candidate, the best kept). No arm beat the incumbent 98 at
  K51 (125 / 113 / 114 / 138 / 100+1 open): a free re-solve moves 33 of 48
  ends, holds leak the loss onto lanes whose ends never moved (a whole-
  board re-braid re-realises all 47 lanes and its spread of 20 vias
  swamps a 2-4 via hypothesis), and windowing the moved ends starved the
  hypothesis. The CP-SAT re-solve is the wrong chooser and the full
  re-braid the wrong instrument for local information.
* **The chord coupled set and widened probes**: 11-25 lanes re-laid in a
  frozen field of 30 is a far worse router than the full braid.
* **Climb classes in the CP-SAT solve** (`DST_CLIMB` at plan time): the
  solve stops somewhere worse (K28 42 against 34; K51 123 with 3 open
  against 116). The route-judged descent can afford them.
* **Plan-time floors as rankers** (the channel floor, inversions, LIS,
  the free-rider ceiling): |rho| <= 0.13 over the K51 fanouts; the
  channel model carries a third of the real crossing system.
* **The two-via cap**, above.
* **Chain jumps and chain crossovers**, replaced by the probe forms.
* **The whole-board obstacle map cache, the screen-less descent, a Rust
  under-pad search**, above.

## TODO

Ordered, highest value first. An item leaves this list when it is done or
abandoned with a measurement.

1. **Speculation inside a descent.** Rank and dispatch the next net's
   probes while the current wave runs, discard them when a net stands.
   Four workers are 52-60 percent busy; ideal wall is 38-40 s where 63-77
   is measured. No verdict changes.

2. **Stop the evolution when it stalls, and braid the chain's arms side
   by side.** A stalled K51 generation still costs its jumps, crossover
   and their descents, about 300 s; the chain's four braids are 245 of
   its 345 s and independent. Both are small.

3. **The evolution on the cloud.** A generation is seven independent
   operators; one container each (`modal_k.py` ships the tree and pins
   the stack) makes its wall the slowest operator, 3-4x on a population
   run, and width is free. The memo store wants a shared volume.

4. **K51's last two vias.** 83 is the local optimum of every single-net
   move class with climbs. The next move class is a GROUP move: re-layer
   a lane together with its crossing partners in one probe (the coupled
   probe already routes such a set); or jumps that land nearer than two
   random nets.

5. **The chain's seeds.** The plan's objective is anti-correlated with
   the route; the evolution optimises past it, but better seeds are a
   better start. The berth menu is one-per-face at `CANDS=4` (row
   pruning, not column generation), and a plan-time floor over the
   PLANNED LANES rather than the channel is the one untested ranker.

6. **Generality.** Tuned on one bench. The zynq article now runs
   through the chain and the evolution (the table above); what it shows:
   a singleton corridor's source tooth may be planned on the FAR face of
   the source array (the count judge sees a via saved, the length judge
   prices the lane from the tooth's exit and the berth's run but not the
   tooth's own escape through the array -- `_length` of the source move
   is the missing term), and the top rungs lose their in-band execution
   (58 refusals at K44). Off-axis poses (R30, R45) still break the
   plan's compass faces.

7. **The corpus A/B for the `py_router` changes, then the PR to main.**
   `KICAD_SEG_DIST_EXACT` ships OFF (2026-09-19) so the merge leaves
   main's copper alone; the A/B decides whether it turns on.

8. **The `.kicad_dru` is read with real layer names inside the turned
   frame**; a per-layer rule lands on the opposite face for a back-side
   part. Shipped `py_router` code, so it blocks the merge.

9. **`pick_braid` ignores DRC** -- it judges (open, vias) only.

10. **Merge main and re-baseline.** The branch diverged before `#958`,
    `#441` and `#521`/`#906`; the suite is local-only until the merge.

11. **Audit `modal_k`'s `KEEP`**: an INFEASIBLE solve prints no
    `pages-first:` line and reads like "never ran".

12. **Built, default off, never finished**: `DST_ASK_BAN`, `BRAID_PACK=1`,
    `SRC_EXCHANGE=1`, `CONNECT_MAP_CACHE=1`, `--screen=0`.

13. **Unverified review findings**: `dedupe_boards` fingerprints copper
    but not the sidecar; `braid_tier`'s budget exhaustion is silent;
    `_realize_group_first` bans nothing on a pure DRC rejection;
    `blockers_of` double-counts half a track; `collapse_dives` calls
    `os.chdir` at import; `flip_frame` does not mirror `pad.polygons`.

14. **Three pre-existing suite failures**: `test_703_predictor_regen`,
    `test_782_nondefault_netclass_clamp`, `test_fanout_cancel`, plus
    `test_459_group_routing` at its own 1200 s budget.

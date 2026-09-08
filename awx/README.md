# awx -- the K-bus chain (#622): one plan, a fanout that follows it, a braid

(We have no idea what `awx` stands for. The name predates every note
that mentions it.)

The tool set for routing a fanned-out bus between two BGAs, where the
PLAN decides both ends of every net, the FANOUT lays exactly the plan's
moves (and tells the plan what it could not), the BRAID routes the lanes
in a corridor of pages, and a refused lane is negotiated rather than
left open:

    bash chain_k.sh TAG 15 28 41           # -> tmp/TAG_k<K>.kicad_pcb, graded
    python3 make_bench.py BOARD SRC DST OUT # another array pair, any board
    bash pose_gate.sh BOARD SRC DST 15 28   # the same pair in every pose

## Where it stands (2026-09-08)

The bench (`fb_t2q_fresh`: an H3 BGA `U1` to a DDR3 `DU1`, the coherent
K-ladder), one fanout per K, byte-deterministic, 0 DRC at the routed 0.1
mm floor everywhere, and since 2026-09-08 the SAME result for the board
moved anywhere on the sheet (the translation section below):

| K  | open | vias | in-band | chain | human vias | 09-07 (before the pose work) |
|----|------|------|---------|-------|------------|------------------------------|
| 15 | 0 | 14  | 15 / 15 | 15 s  | 22 | 14, 15 s |
| 28 | 0 | 36  | 28 / 28 | 37 s  | 46 | 38, 28 s |
| 35 | 0 | 54  | 34 / 35 | 81 s  | 58 | 54, 76 s |
| 41 | 0 | 86  | 37 / 41 | 200 s | 70 | 82, 155 s |
| 51 | **1 (SA4)** | 130 | 33 / 47 | 429 s | 85 | 0 open, 141, 339 s |

Timed alone, 2026-09-08 (tag `ft`). K41 and K51 completed for the first
time on 09-07 (the blocker-directed rip at the last call); the pose work
since then re-decided every exact-tie stamp in the fanout and the main
router's pad keep-outs, and the large-K draws moved with them: K28 -2,
K41 +4, and K51's SA4 (the chronic one: TODO 1's re-berth case) is open
again on this draw. Those draws are the chain's knife edge, not the
rules' -- the same rules grade the moved board identically. "In-band" is
the lanes the braid routed inside their planned bands; the rest were
re-laid at the last call.

![K41 on the bench: one corridor of 41 lanes, both pages, the rides round the destination](img/k41_corridor.png)

*K41: the corridor from `U1` (left) to `DU1` (right) -- front lanes red,
back lanes blue, every lane in its band, the far-face exits riding round
the destination's east face.*

A second array pair (the corpus's `zynq_ad9364` made two-layer, `U1` ->
`U2`, 44 nets; `make_bench.py`): K11 0 open 24 vias, K20 0 open 32,
K28 0 open 55, 0 DRC -- complete, but 17 of 28 in-band where the bench
has 28 of 28. The pose gate (`pose_gate.sh`: both arrays on the back,
either one, the board rotated) completes every pose; the rotations are
exact isometries of the plan and the braid's rules, the residual being
the router's octilinear lattice; and the board TURNED OVER
(`mirror_board.py`) exposed the chain's own front call-outs: the layer
the taut paths relax against (fixed to the layer the teeth are born
on), the destination selector, which is handed and now runs every
pair in the pair's own frame (`PairFrame`: the mirror gets the mirror
of the plan, the bench is untouched by construction), and the braid's
planner and the braid, which take the same frame at their own boundary
(`braid.setup`: a -1 pair's board turned over in memory, the plan
mirrored in, the copper mirrored back). The mirror now grades 16 and 38
against the front's 16 and 38, the same plan to the letter (the pose
gate section below).

## The chain

1. `coherent_nets.py K` -- the first K routable nets of the coherent
   ladder (`k_ladder_coherent.txt`: whole rivers, tightest first; a
   prefix never splits a river).
2. `fanout_from_plan.py OUT.kicad_pcb K --board=BASE` -- ONE consistent
   loop over the plan and both fanouts:
   * `plan_state` reads everything off the board AS IT IS: the menus of
     legal escape moves at both ends (`escape_moves.py`; the source menu
     prices its moves against the other nets' real stubs), the launch
     points (the source teeth on the board), each tooth's layer and
     vias, the taut-path buses.
   * The destination is chosen against those teeth (`select_moves.py`),
     the source refined on paper against that destination
     (`plan_ends.refine_source`), and the refinement is REALIZED:
     `source_realize.py` strips those nets' source copper and re-fans
     them with the production engine in the plan's full moves, restores
     any ball the engine refuses, DRC-gates the board, and audits every
     tooth -- original vs asked vs achieved, per dimension (face, exit
     gap along the face, layer, kind) and as an ORDER along each face.
     The next round chooses the destination against the teeth that
     copper produced; the best round's board and choice are kept.
   * FEEDBACK: a move the engine did not lay exactly leaves that net's
     menu (`banned`, keyed by `source_realize.move_sig`) and the round
     re-plans; the destination is its own select -> fan out -> audit ->
     ban -> re-select loop that ends at "every berth laid as planned"
     or, when it never converges (K41), ships its LAST pass with that
     pass's own sidecar. The engine is the authority on what is possible.
   * What the plan is JUDGED on (`plan_ends.judged_cost`): the vias its
     own model implies -- per net a dive if the corridor cannot keep it
     on its tooth layer, a via where the delivered layer is not the berth
     escape's, the berth escape's vias (`select_moves.true_vias`) -- plus
     the SOURCE escape's vias, plus the ride round BOTH arrays at
     `VIA_MM` per via (`around_box`, hit-tested against a box shrunk by a
     hair so a leg along a face is not a hit). Keepers are judged within
     the corridors the braid will form: `planned_buses` calls the braid's
     own `corridor.cluster_corridors` on taut paths from each tooth to
     its planned exit. `explain_plan` prints the model per net (launch
     and exit order, keepers, predicted vias, crossing pairs) so it can
     be held against `via_census.py`.
   * The plan's lane model: two moves cannot share a gap on one layer
     over overlapping stretches, a dog-bone via is a THROUGH obstacle in
     any other lane, two teeth cannot share an exit point whatever their
     layers, and a dog-bone site must be an inter-ball gap (not the
     boundary line).
3. `braid.py --board FO.kicad_pcb --dest DU1 --nets ... --out STEM` --
   corridors from the geometry (`corridor.py`: nets whose stubs one
   spine can reach), a straight spine per corridor, launch and target
   orders from the lanes' offsets, the two-page schedule
   (`schedule.py`: pages BY TOOTH LAYER -- page F seeded by the largest
   crossing-free set among front-born nets, page B among back-born, the
   rest filling whichever page they do not cross, own layer first; what
   fits neither swims), and every lane routed by the real router inside
   its band (`connect.py`, `topo_strings.py`), the lanes not yet routed
   stamped as virtual copper. Up to six attempts widen the launch pitch
   and route refused lanes earlier, the best attempt is kept; refused
   lanes then get a wider last call, and a lane still refused there a
   BLOCKER-DIRECTED RIP (the router's blocked frontier attributed to
   this run's lanes, a min-cut probe naming the cut set, victims re-laid
   or negotiated one level down); lanes with three or more vias get an
   economy re-lay kept only when strictly cheaper. What is still refused
   is reported and left open. The output is smoothed (the repo's
   octolinear pass) and written with an Eco overlay of the planned lanes.
5. `make_bench.py`, `rotate_board.py`, `mirror_board.py`, `pose_gate.sh`
   -- an article from any board (a two-layer version, the source fanned
   out the chain's way, the floor stamped, the ladder beside it), its
   rotations and its mirror, and the gate that runs the chain in every
   pose (below).
4. `grade_k.py BOARD NETS` -- connectivity scoped to the run's nets,
   whole-board DRC at the routed floor, the via census
   (`via_census.py`).

No environment variables, no options beyond BASE / DEST / LADDER (the
inputs: the board, the destination reference, the ladder beside it).

## The ingredients, in pictures

![K28 on the bench](img/k28_corridor.png)

*K28: two pages. A front lane and a back lane cross for free; a page
lane keeps its layer through the schedule region and pays a via only
where its tooth or berth is on the other layer. 38 vias for 28 nets.*

![The east face at K41](img/k41_east_face.png)

*Far-face exits: a berth on the destination's far face is a side exit
of the main corridor whose leg lies beyond the array and whose jog runs
back along the stub's own line -- not a corridor of its own through the
ball field.*

![SBA2 after the rip at K41](img/k41_rip_sba2.png)

*The blocker-directed rip: SBA2 (the highlighted lane) was refused at the
last call, boxed by lanes routed before it. Its blocked frontier named
the lanes on it, the min-cut probe found the cheapest crossing set,
SCKE1 was ripped, SBA2 routed, SCKE1 refused and negotiated in turn by
ripping SA8 -- every lane routed, K41 complete.*

![K51 on the bench](img/k51_corridor.png)

*K51: 48 routable nets, complete at 141 vias (human 85). Twenty-four rips
landed a lane; the vias are the work now.*

![The human's K51 on the original board](img/k51_human.png)

*The same 48 nets as the human routed them (`allwinner_h3_ddr3`, the
original board): 85 vias, most nets on one layer between their two
escape vias, the address rides nested round the destination, the data
lanes meandered to length. The benchmark to approach, not a pose to
match.*

![The second array pair at K28](img/zynq_k28.png)

*The zynq article (`make_bench.py --two-layer`): the corridor runs north
from the Zynq to the DDR3, berths on three faces. Complete at 55 vias;
17 of 28 in-band, the gap a second board shows.*

![The bench turned over](img/gate_mirror_article.png)

*The mirror article: the fanned bench flipped through its plane -- every
part on the other face, every stub on the other layer, y mirrored,
self-verified -- so the chain's own front call-outs show without the
fanout engine's.*

![The bench rotated 30 degrees, K15](img/gate_r30_k15.png)

*A 30-degree rotation: complete, but 33 vias and 6 of 15 in-band -- the
plan's faces are compass directions and the router's lattice is
octilinear, so a non-orthogonal pose is outside both models today.*

## Benches and gates

    python3 make_bench.py BOARD SRC DST OUT.kicad_pcb [--two-layer]
                          [--src-side F|B] [--dst-side F|B] [--rotate DEG]
    python3 rotate_board.py IN OUT DEG      # the whole board, self-verified
    python3 mirror_board.py IN OUT          # the board turned over, self-verified
    [POSES="R90 R30"] [GATE=name] [LADDER=file] bash pose_gate.sh BOARD SRC DST K...

`make_bench.py` prepares an article the way the bench was prepared:
inner layers out (`--two-layer`), the pair's two-pad nets, SRC fanned
out with the chain's own destination engine call, the project stamped
with the chain's floor, DRC-gated, the ladder beside it from the plan's
river detection; `--src-side` / `--dst-side` mirror an array to the
other face (the caps under it following), `--rotate` turns the finished
article. `pose_gate.sh` runs the chain over FF / BF / FB / BB / R90 /
R180 / R270 (or any `R<deg>`) with one ladder beside every pose and
prints open / DRC / vias / in-band / seconds per pose and K.

## TODO for future sessions

1. **A re-berth AND a re-fan move for trapped stubs.** The rip stops at
   "walled by static copper -- a fanout matter". Take4's negotiator
   answered that by re-berthing (`negotiate_stubs`, `relay_net.py
   --ref`): rip the berth and fan the ball out again in another move.
   Do the same for the TEETH -- a re-fan of the source escape for a
   trapped stub, at either end, judged by the chain. First concrete
   case: the FB pose (destination array on the back) at K28 leaves SA0
   open after a depth-2 rip, walled by static copper at its berth.
2. **Try the packing** (take4's `pack_lanes` / `relax_attract`: a
   follow-the-neighbour force pulling each lane to `pitch` from the
   nearest packed lane on its layer). Tidier rivers for the same grade
   at K28, and packed rivers leave room for the swimmers and their vias.
3. **A better routing order.** Lanes are laid sequentially -- pages in
   target order, then swimmers largest displacement first, refused
   lanes boosted next attempt -- and every refusal the rip repairs is a
   sequential loss. Take4's order model (`plan_order.BraidOrder`, the
   braid's own rules as the plan's cost) and its rip assist are the
   references; candidates are most-constrained-first, the min-cut
   probe's crossing counts as the order, and the negotiator's history.
4. **Better spines.** The spine is the straight chord between the two
   end zones (two corners when the flows bend). Take4 relaxed the
   members' mean taut path against ramped obstacles (`mean_path`,
   `relax_path`, `resample`), so a corridor bent only where something
   was in the way; it was pruned here as never reached at K28, and its
   absence crashed K51's singleton corridor until the chord took both
   branches. A corridor that must bend round a part needs it back.
5. **Collapse the short dives** (take4's `collapse_dives.py`, 388 lines,
   never ported): on a routed board, a short dive is two same-net vias
   joined by a brief single-layer bridge -- the A*'s zigzag escapes, a
   lane that surfaces for 0.85 mm and dives again. Each pair is tried
   serially, accept-and-build: rip the two vias and the bridge, ask the
   real router band-free for a path between the cut ends, keep it only
   if it adds no via (two saved per accept), verify by re-walking the
   net's endpoint degrees, grade as ever. The cheapest via reducer on
   the table for K41's 82 and K51's 141.


## Still on take4, worth a port

Beyond the TODOs above, these on the `bus622-take4` branch still earn a
port, grouped by what they would answer today. (The branch's full
inventory -- 136 files by purpose, the mechanisms cut from the modules
kept here -- lived in this README's history section until 2026-09-07;
`git show bus622-take4:awx/README.md` has it.)

**Where the extra vias are** (K41 82 against 70, K51 141 against 85).

- `ledger_cal.py`: per net, the DP floor -- the vias its real crossings
  force -- against its slack, the realization waste. That split is the
  first question to ask of the 141: slack is cheap to recover, floor
  means the plan's crossing set.
- `harvest_k.py` and `surgical.py`: re-braid each slack net alone, in
  place, against everyone else's frozen copper, keep strictly better.
  The primitives are exactly the "one net swapped or stripped and
  braided alone" that the re-berth TODO also needs.
- `census_vs_human.py` and `human_at_k.py`: per net, our vias, copper
  and via positions (source end, destination end, mid-field) against
  the human's on the same K set. The human's vias are at the ends; the
  census names which of ours are not.

**Why the plan makes so many swimmers** (22 of 47 at K51).

- `cut_ledger.py`: the Maley cut-capacity check on the plan before any
  lane is routed, so "not enough room" is measured, not inferred from
  refusals.
- `group_pages.py` and `plan_nest.py`: joint ride assignment over a
  crossing group, and homotopy nesting. Their measured lesson at K35
  was that the gap to the human was the crossing set, not the pages.
- `channel_shift.py`: the human's one-track-per-channel pattern at an
  array face, which set the K35 record at 66. It is a concrete re-fan
  move for the teeth TODO.

**Output hygiene.**

- `nudge_grazes.py`: the write-time micro-nudge for the ~35 um
  quantization grazes, the class the 30-degree pose showed 15 of.
- `prune_debris.py`: dead tails and twin arms left by mid-stub joints,
  which count as copper against the human.

**Poses off the axes.**

- `flow_frame.py`: rotate the pair into its own frame before planning,
  the way `rotate_frame.py` already does for a rotated BGA, then rotate
  back. That is the answer to the compass-direction faces the 30 and 45
  degree rows hit.

**Workflow.**

- `drive_k.py` composes "complete first, then cheapen"; `retry_chain.py`
  is chain-level directed iteration with the refused nets forced
  through the back-side arms. Completion is the rip's job now, but the
  composition is the shape the via work will take.
- `band_dump.py`: the per-lane world picture, for the second bench's 17
  of 28 in-band.

`improve_k.py` (the diagnose-and-move loop) is the heaviest and leaned
on the pages sidecar that take5 cut, so it is last.

## What this adds to `py_router` (and nothing else)

`generate_bga_fanout(..., escape_dir_hints=...)`: a per-pad planned
escape keyed by board-frame pad position -- a bare FACE (`'down'`), or a
FULL MOVE (`{'face', 'exit', 'layer', 'kind', 'site'}`: the exit point on
the boundary line, the layer the run leaves on, `surface` /
`via_in_pad` / `dogbone`, the dog-bone via point). Re-keyed and
transformed into the footprint frame for a rotated part
(`rotate_frame.forward_transform`), threaded through the escape-priority
passes and the auto-retry ladder; the channel engine reads the face of
either. The UNDER-PAD engine follows a full move in its plan-follow
phase (`underpad._follow_plan`): planned balls leave the generic phases,
their via sites are reserved first (an asked dog-bone site validated
exactly as the engine's own), every ball is routed to its EXACT move
deepest-first (the A* takes a goal cell: the boundary cell at the asked
gap, the only way out), a ball whose exact move is blocked negotiates --
its blockers among the same call's escapes are found by routing it on a
pre-commit occupancy snapshot, ripped, the ball laid, the blockers
re-laid, the state kept only if the count of balls landed as asked rises
-- and what is still short degrades along the least damaging dimension:
the nearest free gaps first (+-6 pitches), then the other layer/kind,
then any face. Every ball's outcome is reported per dimension and
returned in `pcb_data._fanout_plan_report`. With face-only hints every
path is unchanged (copper identical to the previous chain); with no hints
nothing changes at all.

## One planner

The braid's own planning stage is the planner. `braid.setup(plan=)` takes
the ends, their layers and their escape directions from a plan instead
of reading them off copper, and `braid.plan_braid(board, names, dest,
plan)` runs the plan-only stage on them: corridors as the braid forms
them, spines, offsets, launch and target orders, the schedule's pages
and swimmers, side-exit legs. The fanout loop judges EVERY round and
every destination re-plan with it (`fanout_from_plan.judge_by_braid`:
per net the vias the pages imply -- `plan_ends.vias_from_pages`: tooth
vias, tooth/page mismatch, the arrival through a side-exit leg, the
berth's vias, a swimmer's dive and surface -- plus the ride round both
arrays); the fast proxy (`plan_ends.judged_cost`) serves only the source
refinement's inner loop. The shipped plan is written beside the fanout
board as `<board>.plan.json` with the ACHIEVED stub ends, layers and
faces, and the braid reads it (`setup` finds it; a sidecar that names
only some of the run's nets is the plan for those, the rest read off
the board) and builds its corridors from the identical inputs. At
every K the braid's orders and pages are the planner's.

Results (bench, same engine, previous chain -> this one): K4 4 -> 4 vias
(predicted 4, per net identical), K8 8 -> 6 (predicted 6), K15 22 -> 14
(predicted 12), K28 38 -> 42 (predicted 32); all complete and DRC-clean,
every berth laid as planned. The prediction missed every page lane the
braid had to route with an under-pass; see the via model below.

### The plan's via model

What a page lane costs is the number of LAYER CHANGES along its whole
profile (`braid.Corridor.layer_profile`): the tooth's layer, then every
stretch the schedule requires in s order -- its page over the schedule
region, and in the tail the OTHER layer wherever a same-layer exit leg
crosses it -- then its exit leg's layer, then the berth's. Adjacent
equal layers merge; the changes plus the tooth's and the berth's own
vias are the lane's prediction (`plan_ends.vias_from_pages(changes=)`).
The old count saw only the corridor's interior (`xa < s1`), so every
dive under an exit leg in the tail was free on paper: K28 predicted 32
for 42 laid, and each miss was a lane forced to the other layer after
s1. Two things follow from the profile:

- Exit legs choose their layer ALONG s (`lay_lanes`): a lane is crossed
  only by legs earlier than its own, so with the legs decided in
  ascending s every stretch the earlier legs imposed -- on the leg's
  own lane and on the lanes it crosses -- is known when it chooses. A
  crossed lane already on the other layer pays nothing more; a leg on
  the layer its lane is already on needs no corner via. Judged by pages
  alone the old rule sent K28's SA9 F -> B -> F -> B (three changes) for
  the one the router found. In-band the braid now routes 22 of 28 K28
  lanes at the first attempt where it routed 8.
- A later corridor's lane that crosses an earlier corridor's planned
  lanes on a layer they may use pays one dive, two vias
  (`braid.cross_corridor_vias`). Unpriced, the honest judge preferred a
  K15 plan that made SA9 a corridor of its own (predicted 0, realized
  2, the board 18 for 14).

Measured (HEAD f8b04714 -> this, same bench, warm taut memo, chain time
unchanged): K15 14 -> 14 vias with the prediction exact per net; K28 42
-> 40 (predicted 36); K35 56 -> 54 (predicted 59); K41 12 open -> 9
open, 72 -> 92 vias (three more nets routed; 18 of 37 lanes swim, the
two-page schedule is past its capacity there, and the plan is the same
in both arms). The residual at K28 is the braid's in-band execution:
the lanes it refuses are re-laid at last call, where some pick up a
dive the plan never asked for. Six execution changes were measured on
K15/K28 the same day (a layer-blind self-stamp in `cross_reserve`, a
symmetric launch pitch with a longer fan-in, re-running the best
attempt, the dodge tube kept out of the fan-in, wider virtual copper,
exit-corner via reservations set in along the leg) and none was a net
gain: each moved vias by two to four between lanes. They are not here.

### The walls, named cell by cell (2026-09-06 evening)

Every in-band refusal at K28 was traced with `tmp/wall_probe.py` (the
router intercepted at the lane's first call; the pocket flood-filled
from the tooth and its wall attributed by clearance zone to an owner --
a virtual lane by net, a reserve piece, real copper, a via, a hop, a
pad, the band; a channel profile along the planned polyline; the
farthest s reached in the whole window). Six lanes were refused at
HEAD and each had a name: a B-page lane's 1.5 mm reserve stamp on F
past its own B requirement (SDQ9 over SDQ10, SDQ11 over SDQ8), the
swimmer SA4's reserved diagonal over SDQ7, SDQ14's dive via beside
SDQM0's tooth, and the pads of C5 -- a front-side 0402 inside the
corridor, through which three lanes were planned straight while the
page rule closed the other layer exactly there. What was built from
that, each keyed on geometry read off the board:

- `cross_reserve` no longer stamps the corridor being routed (its own
  lanes are stamped by `virtual_of`, which follows the layer rules), and
  a reservation is clipped round every other net's free end.
- STATIC ISLANDS: every pad of a part that is not one of the arrays,
  projected on the spine and inflated by a track's clearance, merged
  when less than 0.1 mm apart, from s0 to the farthest stub
  (`static_islands`). `deflect_islands` bends the lanes on the island's
  layer round it -- the side by the smaller corner deflection, nearest
  lane at the edge, the rest outward at their own gap, other islands on
  that side stepped over -- into the (s, o) polylines the bands, the
  virtual copper and the windows read; in the tail the exit comb bends
  outward, a lane with no room to return before its leg stays bent to
  the leg and the leg starts there, and a leg over an island on its own
  layer is re-placed off it (`place_and_decide` run twice). The via
  model does not change: a lane bent on its own layer changes no layer.
- The birth and landing via at the slot (a via costs the same anywhere
  on a stretch, and the forward search leaves the tooth layer only when
  forced -- so a 0.45 mm stretch put the via at its far end, where the
  neighbours had converged).
- The slot pitch scaled by the secant of the lane's angle to the spine
  (`pair_floor`): clearance is perpendicular to a lane, a slot pitch is
  measured across the spine, and at 45 degrees 0.35 mm is 0.25 mm of
  room; a slot where a lane changes layer gets a via's room. The launch
  relax is symmetric and the fan-in grows with the largest shift, so no
  fan-in is steeper than 45 degrees (checked on paper first,
  `tmp/pitch_check.py`).
- A swimmer's reserved hop keeps a 2-D distance from every lane's
  polyline, not from its offset at one s.

- EARLY DIVE: a lane whose tail crosses an island on the layer it is on
  and which owes a change to the other layer anyway (its berth is there,
  or its page already is) takes that change BEFORE the island -- no via
  the plan did not already count, and no bend. Seeded into the leg
  placement so the leg's layer follows. At K35 the plan had looped SRST,
  SA0 and SA15 3 mm round a six-part passive cluster on F; they now go
  under it on B in-band. K41: 98 -> 84 vias at the same 8 open; K35 58
  -> 62, three other swimmers landing at 4.

The remaining walls are the swimmers (ten at K35, eighteen at K41, 2..4
vias each, and every K41 open is one): the two-page ribbon's capacity.

Speed: a taut path depends only on its two ends and the static copper it
relaxes against, and the loop asked for the same ones at every judgment
(210 relaxations for 15 nets), so `detect_buses.taut_paths` memoises on
the ends and `Obstacles.signature()`, persisted in `tmp/taut_memo.json`
across processes (the braid reuses the fanout stage's paths);
`braid.build_obstacles` memoises per board file. K15: 67 s -> 40 s cold,
19 s with a warm memo, copper identical. The obstacle model still counts
the run's nets' VIAS while excluding their segments (inconsistent, and it
changes the memo key on every realized board); excluding them changes
taut paths and needs an A/B.

### The best attempt is kept, and stale attempts end the loop (2026-09-06/07)

The attempt loop's feedback -- a wider launch pitch, refused lanes
boosted to the front -- is a heuristic for the refused lanes and a
change of world for every other lane, and the LAST attempt used to
ship. At K35 attempt 0 routed 27/32 in 33 vias with every swimmer at
2 (the router's own world flooded at each swimmer's first call says 2
is the minimum there); attempt 3, the one that shipped, routed 26/32
in 40 with five swimmers weaving for 4 each. Each attempt is a full
re-route from the base copper, so the one with the most lanes routed
(fewest vias on a tie) is restored -- copper, bookkeeping and plan
geometry -- before the last call. K35 62 -> 57 vias at 0 open, K41 84
-> 80 at the same 8 open, K15/K28 unchanged.

The loop used to stop only when the refused SET repeated at the maxed
pitch; a set alternating between two lanes (K35 with the far-face
exits: SA4 / SCKE1) never repeats and ran all six attempts for a best
that was attempt 0. An attempt at the maxed pitch that does not beat
the best attempt's routed count is stale; two in a row end the loop.
Copper identical (the best attempt is kept either way); K35 braid 85
-> 43 s.

### Far-face exits (2026-09-07)

A net whose berth sits on the destination array's FAR face -- past the
last ball along the spine, escaping away from the bundle -- used to be
split into a corridor of its own: the split rule asked whether its lane
could run from the stub back along the spine to the spine's end, and
that run goes through the ball field. The corridor it then got was
spined straight from its teeth to its berths, through the main bundle,
so every lane it planned was fiction: at K35 SA9/SA13/SA8 were refused
in-band on every attempt and re-laid at last call round the south and
east of everything (2 vias each, the human's homotopy and count), while
the judge priced them 4 each; at K41 that corridor held three of the
eight open nets.

The copper that ships is an ordinary side exit of the MAIN corridor
whose leg lies beyond the array: the bundle's outermost lane on that
side runs past the far face, a leg turns in along it, and the jog runs
back into the stub tip. So:

- `corridor.cluster_corridors` admits a stub past the spine's end when
  a short run FORWARD from it (away from the array, a pitch to a block's
  width) and a leg from there out across the array's side are pad-clear
  (`wrap_clear`); the old run-back test is tried first.
- `braid.Corridor.classify` marks a side exit whose stub lies beyond
  the last ball as `far_exit`, with `s_leg_min` = last ball + its radius
  + clearance + half a track: its exit leg is placed at or past that
  (`place_and_decide`, the floor enforced through `_leg_s`'s avoid), the
  target order already gives it the outermost slot (largest exit s), and
  the spine's forward extension and the static-island window reach
  `WRAP_REACH` past it, because `Spine.project` clamps s at the spine's
  end and an island at the array's corner (K35: C10 on F) is what the
  leg must clear.

Measured (bench fb_t2q_fresh, 0 DRC, warm taut memo): K15 14v and K28
38v unchanged, copper identical; K35 ONE corridor of 35, SA9/SA13/SA8
in-band at attempt 0, 57 -> 55 vias, plan 59 -> 52 predicted (SA13 and
SA7 0 vias); **K41 8 open -> 1 open (SA4), 80 -> 86 vias**, one corridor
of 41. Chain times K35 67 s, K41 151 s (fanout 68 + braid 82). The
first run after any plan change pays the taut memo cold (K41: 342
recomputations, ~5 minutes) -- a one-time cost, not the mechanism.
`chain_k.sh` now stamps the fanout and braid stage boundaries. (The
memo, `tmp/taut_memo.json`, is content-keyed and has never been wrong,
but it has grown to 158 MB and every process loads all of it, 1.6 s
each and two processes per K; a mirror or a new pose pays it cold --
K15 61 s against 21 s warm. Pruning it, or keying a file per board, is
a time item of its own.)

### The fanout engine's axes: translation first, then rotation (2026-09-08)

The pose gate had shown the engine's fanout depends on the angle the
array was dropped at: a FRESH fanout of the same source array on the
board rotated by 0 / 90 / 180 / 270 degrees gave 502 / 540 / 425 / 430
tracks and the chain 38 / 42 / 54 / 38 vias at K28. Routing every
rotated part in its own frame (`rotate_frame`, extended from the
non-orthogonal case to every angle, exact quarter turns about a lattice
point, the whole board carried: holes, zones, keep-outs, bounds, foreign
parts' angles by MINUS the turn, pad sizes swapped on odd quarter turns,
`frame_rotation` stamped for the braid's obstacle memo) made the half
turn exact and the quarter turns agree with each other -- and left them
different from the unrotated article. The reason was not rotation: the
engine was not TRANSLATION-invariant. The same board shifted 1 mm in x
fanned out to 447 tracks instead of 502.

**Translation invariance, found by bisecting the engine's own log and
its per-ball searches on the native and the shifted board
(`tmp/frame/src_*.py`, `trace_ball.py`, `gnd_trace*.py`).** Every cause
was a decision made by the last bit of a coordinate where exact
arithmetic has a tie, and every fix is the same: round the key so equal
stays equal, and let a deterministic order decide.

- `underpad.depth`, the ball routing order: balls on one ring are
  equally deep; the noise ordered them, and a different order routed one
  ball (SDQ4) into a corner, whose rip-swap rescue re-assigned five nets.
  Rounded to a nanometre; the stable sort keeps the footprint's own pad
  order, which moves and turns with the part.
- `escape.py`: the four edge distances of a ball (a corner ball ties two),
  the nearest channel to a pad that sits exactly midway between two, the
  nearest far end of a net, and the target-side comparison |dx| >= |dy| (a
  far pad on the array's own diagonal ties). `__init__._surface_gap_escape`'s
  four-exit order. All rounded.
- `reroute._seg_hits_pad`: a sample point exactly on a pad edge. With no
  margin a centreline on the edge is copper on copper (a hit); with a
  clearance margin exactly-at-clearance is clear, as KiCad grades it.
- `plane_fill_model.nearest_component_point`: the pour-direct tap of a
  corner plane ball is equally near two fill cells; the argmin is rounded.
- `underpad._Occ.cell`: the occupancy lattice is a NODE lattice (cell
  `(ix, iy)` is the point `x0 + ix*res`), and balls sit EXACTLY on nodes
  whenever the window margin is a whole number of cells (0.8 / 1.0 / 0.5
  mm pitches: 1.0 mm = 40 cells of pitch/32); the truncation let the last
  bit choose between the node and the one below. A point within 1e-9 of a
  node now belongs to THAT node (the upper cell). The LOWER cell was
  shipped first -- translation-invariant too, and the bench draw the user
  accepted (14 / 38 / 98) -- until `tests/test_bga_fanout_dogbone.py`
  showed what it does: every on-node ball moves one cell down, so a
  via-in-pad at the ball centre has its stub start 25 um away (ulx3s B12,
  the one orphan of 8 checks). The upper cell is the right one; what had
  made it look wrong was the next two items.
- `underpad._Occ._disk_spans` / `_capsule_spans`, the rasterisers behind
  every stamp: the centre in cell units (`(x - x0)/res`) carries the
  coordinate's noise, and a boundary cell at an exact integer radius
  (`i*i + j*j == (r/res)**2`: the 3-4-5 cells of a 0.125 mm disc, the
  edge of a capsule on a node row) was decided by it -- 244 and 288 cells
  of the initial stamp on the orangecrab article, and one net's jog took
  the other side of its corridor. Quantised to a billionth of a cell;
  everything downstream is arithmetic on those numbers and is then
  bit-identical in every frame.

`tests/test_fanout_translation.py` pins it wx-free on a real 96-ball
BGA (orangecrab U4): under three lattice translations the engine's
INITIAL OCCUPANCY STAMP is byte-identical (the check that exposed both
rasteriser ties) and the routing is the same (vias exact, every segment
endpoint within one occupancy cell of the other run's copper; with the
upper cell the moved runs have the same track count as in place), with a
non-vacuity check that the board has ring ties the shift re-orders in
raw arithmetic. The origin article (`U1` on `h3_FF_base`) is the same
447 tracks and 8 vias under eleven shifts. `translate_board.py` makes a
moved article for the pose gate (the third isometry, beside rotation and
the mirror).

**The bench's cost, and the decision.** The ordering fixes alone leave
the bench at 14 / 38 / 82. The cell rule cannot: every deterministic
convention re-decides the exact-edge stamps at the destination array,
and K41's draw moves with them (K15 14 and K28 38 hold under all). The
tuned 82 was the noise's draw, and that K41 sits on a knife edge is the
chain's (the rip and plan-adherence items). Decided by the user
2026-09-08: whichever rule is right ships and K41 is to be won back in
the chain. The lower cell went in first (14 / 38 / 98); the section
below is what it took to make the CHAIN invariant, and the bench with
the final rules is **14 / 36 / 86**.

**Rotation, measured with that in** (`tmp/frame/rot10.out`, a fresh
fanout on the rotated article, then the chain; K15 / K28 vias):

| pose | frame off (default) | frame on (`KICAD_FANOUT_FRAME_QUARTER=1`) |
|---|---|---|
| R0 | 14 / 42 | 12 / 40 (417 / 2107 segments) |
| R90 | 16 / 50 | 12 / 36 + SA4 open (705 / 1578) |
| R180 | 14 / 54 | 12 / 40, identical to R0 to the segment |
| R270 | 16 / 42 | 12 / 36 + SA4 open, identical to R90 |

With the frame on all four poses fan out to the same 447 tracks and 8
vias: the FANOUT is exactly rotation-covariant now. The half turn is an
exact symmetry of the whole chain; the quarter turns agree with each
other and differ from R0 by the PLAN's and the BRAID's own axes (the
braid's octilinear search leans on one axis, the plan's faces are
compass directions), which is the next frame to build (TODO: the pair's
flow frame, take4's idea). The quarter-turn frame stays opt-in until
then: making it the default re-fans the bench's destination array (at
90 degrees) and moves that draw again.

### The chain after the fanout: three more ties, and the pad offset in the main router (2026-09-08)

With the fanout engine exactly translation-invariant, the chain on the
bench moved by (10.3, -7.7) mm (`translate_board.py`, `tmp/gate/fbT`)
still graded K41 109 against 98 in place (K15 and K28 identical). Found
by diffing the two chains' logs stage by stage with the numbers
stripped, and fixed in order; each fix was measured by re-running both
chains and diffing again.

1. **The rip-swap rescue's victim order** (`bga_fanout.__init__.
   _underpad_rip_rescue`). In destination pass 2 the plan-follow stage
   was identical on both boards to the ball, and then the rescue of SA3
   evicted SA2 in place and SA15 moved: the two neighbours are EQUALLY far
   from the ball (0.425 mm, the symmetric pair on a regular pitch), the
   log printed the last bit as 0.42 and 0.43, and whichever came first
   opened a different corridor. Both rescues succeed. Rule now: the
   distance to a nanometre, and victims at the SAME distance are all
   tried and the RESULT decides (fewest vias, least copper, then name) --
   a general rule, not a coin. The corridor band's widest-gap choice got
   the same nanometre key. Chain identical in both frames after this,
   grading 109 in both: the tie-by-name draw. (`_surface_gap_escape`'s
   `KICAD_FANOUT_RESCUE_DEBUG=1` prints every corridor it tries.)
2. **The occupancy rasterisers** (the bullet above): found when the
   upper cell, adopted for the dogbone orphan, failed the translation test
   on the orangecrab article -- 244 boundary cells of the initial stamp
   differed, and the fix for the disc showed the capsule's 288. With the
   upper cell and both rasterisers quantised the chain grades 14 / 36 in
   both frames at K15 / K28, and at K41 86 in place against 84 moved:
   the FANOUT stage identical to the log line, the braid's first lane
   differing by ONE A* iteration (92960 against 92959).
3. **The main router's pad keep-out** (`routing_utils.
   pad_blocked_cells_array`, shared by every routing step). Dumping the
   obstacle map the braid hands the router for that first lane on both
   boards (`is_blocked` / `is_via_blocked` / the two cost maps over the
   lane's region, `tmp/frame/braid_obs_*.py`) and comparing them shifted:
   blocked, layer costs and stub costs identical, THREE via-keep-out cells
   different -- each exactly at the keep-out boundary of a roundrect
   passive's pad (R3.1, R3.2, C12.2 at 0.5663 mm: a 3-4-5 cell of the
   corner arc). The rasteriser is exact in the pad's own frame, but its
   sub-cell offset `pad.global_x - gx*grid_step` is computed from the
   absolute coordinate and carries its last bit into `dist_sq <
   margin_sq`, which has no tie epsilon. The offset is quantised to a
   nanometre at the function's entry (and in `iter_pad_blocked_cells`,
   its bit-identical twin); `_capsule_mask` already resolves its ties
   through `GRID_TIE_EPS`, and the via stamps' `off_cells` is a hypot
   (never negative), so those were robust. After this the first lane's
   map is byte-identical, and the whole chain is: **K15 14 / 673, K28
   36 / 1596, K41 86 / 2489 in both frames, the K41 braid logs identical
   line for line.** This one is production code: it joins the engine
   changes owed a corpus A/B before main (TODO 7).

What the exercise says about the chain: every stage that orders by a
raw distance or rasterises from an absolute coordinate has a tie on a
regular pitch, and a translation is the cheapest instrument for finding
them -- the two chains differ only where a last bit decided something.
The same instrument on the braid's own keys (`braid.py` has thirty
keyed sorts on raw projections and distances) is available whenever a
pose gate shows the braid leaning.

### The relaxation, vectorised and convergent: `taut_fast` (2026-09-08, `TAUT_FAST=1`)

The user asked for the current algorithm sped up, its oscillations
removed and its rounds vectorised, and `taut_fast.relax_many` is that:
the same model (curve shortening on a densified polyline against discs
and capsules, the ends frozen, a shortcut every 25 rounds), with

- contact as a CONSTRAINT: a pushed point lands on the boundary plus a
  micron, not 0.01 mm past it, so smooth-then-project is a projected
  gradient step and the round trip that kept every contact string at its
  limit cycle for 400 rounds is gone;
- a capsule the string crosses transversally (more than 30 degrees off
  its direction) is transparent for the crossing -- a dive, which the
  cleanliness check already tolerates -- while one it runs along still
  pushes, so the mean spine stays off foreign tracks;
- every string of a `taut_paths` call in ONE array (a Jacobi sweep,
  diffusion number 1/4; a trust region of 0.05 mm per round so no step
  can jump a thin capsule; per-point candidate lists of the 16 nearest
  obstacles within 0.9 mm from the shared base model, the string's own
  net masked), coarse spacing (0.48 mm) then fine (0.12);
- convergence judged where it can be seen: the shortcut-and-densify
  polyline stopping between blocks (Hausdorff below 0.02 mm), and a
  string that touches nothing leaves after one block;
- a Douglas-Peucker pass before each shortcut -- a relaxed string is
  straight between contacts and the chord along a straight run needs no
  clearance test (the shortcut had become 1.8 of 2.2 s: 64,000 tests);
- the output projected once more after its final densify, so chord
  points between two contact points are not left inside the disc.

Measured on the front article (`tmp/frame/fast_batch.py`): K28's 28
strings 8.9 -> 1.9 s (x4.6), K41's 41 strings 7.9 -> 3.0 s (x2.7,
its strings still spend their round budget: the tangential creep of
contact points is diffusion-limited too). The strings are cleaner: the
old relaxation's wedged oscillation left SA7's and SWE's strings 0.19
mm INSIDE pads U1.Y20 and Y21, the new goes round them; the disc-side
"disagreements" between old and new are those, and ties at touching
pads. The bench through the chain (`TAUT_FAST=1`, `tmp/frame/tf1.out`):
K15 16 / K28 38 / K41 94 vias against 14 / 38 / 98 -- equivalent, a
different draw of the same plan loop. Chain time, memo warm: 10 / 35 /
161 s against 17 / 27 / 182 s; cold (every string computed): 22 / 87 /
342 s. The memo tags its entries `#fast`, so the two algorithms'
strings never mix inside a run.

What it does not reach: the 100x that would retire the memo. A cold
K41 still computes its 342 strings (~35 s batched against ~160 s), so
the sharded memo stays. The rounds are the limit now, not the
arithmetic: sliding a contact point along its disc is as diffusion-
limited as bending the string was, and only an implicit smoothing step
(with a per-round crossing check, since a large step can carry a run of
points across a 0.23 mm capsule) or the vertex solver (contacts as the
state, tangent geometry, a few iterations) goes further. Translation:
the batched relaxation is a float algorithm like the old one, so the
chain downstream of the fanout is not translation-invariant at K41's
knife edge either way (the bench moved by (10.3, -7.7) mm,
`translate_board.py`, grades 14 / 38 / 109 against 14 / 38 / 98 in
place; K15 and K28 to the segment).

### The taut memo, sharded; and why the relaxation itself is not fast (2026-09-08)

`detect_buses.taut_paths` memoises each taut string on `ends@signature`
(the obstacle model's content hash) and persists the memo across the
chain's processes. It had grown into ONE file of 158 MB and 31,000
entries, loaded in full by every process (1.6 s, two processes per K)
and rewritten in full whenever a run added an entry -- a cold K41
dumped it twenty times. It is now `tmp/taut_memo/<xx>.json`, one shard
per two-hex-digit prefix of the signature, loaded on first touch, only
dirty shards written, each merged with the shard on disk first (a
parallel chain's additions survive), entries untouched for 14 days
dropped at write time; the old file is migrated into shards once and
renamed `.migrated`. Nothing about the answers changes.

Whether the memo could go altogether -- the user's question -- was
measured on the front article's 28 K28 strings (`tmp/frame/taut_*.py`):
13.3 s cold, 0.47 s mean, 1.0-1.5 s for the long ones, 83 % of it in
`point_violation` (59 million `hypot` calls: 400 iterations x ~200
points x the pushes). Every string that touches copper runs ALL 400
iterations: the exit test (total movement below 1e-4 mm per point)
never triggers, because a point where the string crosses a foreign
track's capsule -- a legitimate dive on a two-layer ribbon, the
assert-only "violating" strings -- is pushed out and smoothed back
every iteration, an oscillation of 0.01-0.2 mm that the periodic
shortcut-and-densify rebuilds each time. The movement plateaus by
iteration 50-100 and then creeps (SRAS: 0.34 at 25, 0.30 at 100,
0.27 at 200, 0.25 at 400). What was tried against the 400-iteration
result: stop when the movement stagnates (5 % over 50 iterations):
2.5x fewer iterations, paths drift up to 0.17 mm; 2 %: 2.3x, 0.15 mm;
count only un-wedged movement: nothing (their neighbours keep moving);
freeze the wedged points: nothing, and one string drifted 0.57 mm (the
rebuild re-forms them). A numpy red-black sweep is worth about 2x on
top and changes the update scheme; numba is not installed and mypyc
was measured and rejected for this repo. The relaxation's floor is the
contact oscillation, and a 0.17 mm drift is a quarter of the lane
pitch -- a different plan, not a faster one. The real answer is an
exact shortest-homotopic-path solver with explicit contacts (~100x, and
it converges), which is a rewrite, not a fix. Decision: keep the memo,
sharded; a new pose pays its strings once (K15 mirror: 61 s cold, 21 s
warm).

### The sidecar describes the board it sits beside (2026-09-07)

Two ways the chain's braid ran on a different world than the one the
fanout's judge had chosen, both at K41 and both silent:

- The fanout's selector can leave a net UNPLACED (K41: SA9, its menu
  banned away over eight destination passes) and fan it out unplanned,
  so the sidecar named 40 of 41 nets -- and `braid.setup` discarded the
  whole plan ("does not name every net of this run -- ignored", line 1
  of every K41 log) and read every end, layer and direction off copper,
  while the judge that chose that fanout had applied the plan to the
  other 40. A partial sidecar is now the plan for the nets it names
  (`plan from X: 40 of 41 nets; SA9 read off the board`), which is what
  `setup(plan=)` always did for a plan passed in.
- `fanout_from_plan.braid_plan_of` wrote the ASKED berth (`m.layer`,
  `DIRS[m.direction]`) while the board carries the LAID one; at K41 the
  destination passes never converge and 22 of 41 berth layers (and 5
  faces) disagreed with the copper. A braid trusting that sidecar
  routed 15 lanes to a stub end on a layer with no copper there,
  reported them routed, and shipped them open (measured: 19 of 41).
  The sidecar now carries the laid layer and face from the `achieved`
  audit record (`source_realize.measure_tooth`) whenever the fanout
  has laid the berth. Checked against the copper at K41: no berth
  point off copper, no layer wrong.

Neither fix touches the judge's inputs: the K15/K28/K35/K41 fanouts and
their sidecar ends are byte-identical to before. K15/K28/K35 sidecars
were complete already, so those grades are unchanged; at K41 the braid
now follows the plan (braid stage 82 -> 59 s) and grades the same opens
as the copper-read fallback did on the same tree (2 open on the split-
rule tree, 82 -> 88 vias). A probe that passes the plan dict explicitly
already applied a partial plan, so until this the K41 probes and the
chain disagreed (spine 0.07 mm off, different legs): a probe is
trustworthy only when line 1 of the chain's log says the plan was read.

### Exit legs and static islands: the split leg (2026-09-07)

`_leg_s` moved a leg off a static island by a lane pitch along the stub
row. At K35 that started a cascade: SA12's B leg to an F stub was
islanded only in its last 0.1 mm (C6..C9's inflated box under DU1's
bottom ball row), moved a pitch onto SA1's stub end, and every leg of
that row then jogged a pitch onto the next stub -- stub ends 0.4 apart
leave no legal foreign foot -- so SA1/SA5/SA6 were refused at the stub
on every attempt and re-laid at last call (SA1 through DU1's central gap
with three vias). The router had laid SA12's leg straight down from its
own stub all along (F to y 70.75, a via, B under the lanes); the damage
was the planned leg's VIRTUAL stamp on the neighbours.

A leg whose layer is islanded only at its stub end, with the stub on the
other layer, now keeps its s and takes the via it owes anyway just past
the island (a via's room), the last stretch on the stub's layer --
provided that stretch is island-free and crosses no lane
(`leg_split_at`, `Corridor.leg_split`). The virtual stamp is split there
(`virtual_of`), the via reserved (`virtual_vias_of`), a mark drawn. The
via count the plan implies is unchanged.

Measured (bench fb_t2q_fresh, 0 DRC, chain, one fanout): K15 14v and
K28 38v identical; **K35 55 -> 50 vias**, 0 open, SA1/SA6 in-band (30 ->
32 of 35); K41 2 open 89v -> 2 open 88v, 29 -> 30 of 41 in-band.

### Exit legs and static islands: flip or hop, and the jog rule (2026-09-07)

The same pitch move, at K41, pushed SCKE1, SCKE0 and SA15 off the F
passive cluster to one s, 3.3 mm from their stubs, and their jogs then
ran along the stub row on F over each other's stub ends: SCKE0 refused at
its stub every attempt. Two rules replace the move, both keyed on the
island's place along the leg (the island helpers are built once, above
`place_and_decide`):

- **Flip or hop, priced** (`move_cost` in `place_and_decide`): an
  islanded layer that cannot split costs, in the leg's layer economics,
  the cheaper of the veto (`ISLAND_VETO`) and the move along the row that
  clears the island, a pitch of jog worth a via -- so SDQ2 (K35) hops
  0.3 mm off C12 on its own layer for less than a via while SA15 (K41)
  takes B for one where every F move jogs over a stub. A leg still
  islanded on the layer it chose is moved as before.
- **A jog may not run over a free end** (`_leg_s.jogged`): the jog from a
  moved leg back to its end runs on that end's own layer (`virtual_of`),
  so a candidate whose jog passes over another member's free end ON THAT
  LAYER is out; with none left the leg stays. Layer-aware because a K41
  join jog on F, refused the pitch over a B tooth, took the other side
  onto SBA0's F tooth instead. Join legs go through the same `_leg_s`.

Measured (chain, one fanout): K15/K28/K35 identical to the split leg
alone (14 / 38 / 50); **K41 2 open 88v -> 2 open 77v, 30 -> 33 of 41
in-band**, SCKE1/SCKE0/SA15/SBA1/SDQ6 in-band; the fanout's judge runs
these rules too and its K41 choice did not move (sidecars byte-identical).

### The east face, and a leg's room (2026-09-07)

Two walls left at K41 after the rules above, both at a free end:

- **The east face.** SA9, SA13 and SA7 berth on DU1's east face, escaping
  east -- along the spine -- 0.25 mm apart in o with their ends just
  inside the last ball column, so they are not `far_exit` by the
  past-the-last-ball test; a leg in o at each stub's own s runs down the
  face over the neighbouring stubs (SA9's search reached s 29.82 with its
  stub enclosed by SA13's leg stamp and SA7's copper). Refused in-band in
  every arm, they cost 2-8 vias each at last call (SA6 8) -- the largest
  via pool at K41. A side exit whose berth escape direction lies within
  45 degrees of the spine's is now a far-face exit (`classify`,
  `stub_dir . spine dir > 0.7`): its leg goes beyond the array and the
  jog runs back along the stub's own line, the spine's frame extended
  for it (`build_spine`); far exits are placed last, innermost lane first,
  so their legs cross nothing; the jog's o-tolerance is the stamp's own
  reach (`LEG_O`). SA9/SA13/SA6 2 vias each, SA7 0, all in-band.
- **A leg's room.** With every candidate clashing, `_leg_s` took the
  first least-clashing one however close: SA8's join leg 0.007 mm from
  SA5's tooth, SBA2's on SDQ6's -- a stamp on both layers over the tooth,
  the net refused at its first cell every attempt. A candidate within the
  legal minimum (`TRACK + CLEAR`) of a foreign end or a placed leg is no
  candidate (`too_close`); ties among the least-clashing break by the
  most room; with none legal the leg stays.

Measured (chain, one fanout; each rule also graded alone: east face
K41 2 open 86v 38/41, room rule K41 2 open 83v 34/41 with SA4 routed for
the first time): K15 14v / K28 38v identical; K35 0 open 54v, **34 of 35
in-band** (only SODT0 refused); **K41 1 open (SBA2) 78v, 40 of 41
in-band**, 130 s (braid 60 s). Against the committed baseline before this
day's leg work: K35 55 -> 54, K41 2 open 89v 29/41 -> 1 open 78v 40/41
(human 70v). Every constant is a design constant or a direction
quadrant; nothing reads this board's names or coordinates -- but all of
it is measured on ONE bench (fb_t2q_fresh), so a second array pair is the
next confirmation before any of it is treated as a default elsewhere.

### Refusals at last call: the blocker-directed rip (2026-09-07)

A lane still refused at last call, when every other lane is real copper,
is boxed by lanes routed before it -- the sequential loss, an earlier
lane having taken the one channel a later one needs -- and no wider
window answers that. K41's SBA2 (a swimmer, tooth and stub both on B)
was refused only at the kept attempt: SCKE1's B run had crossed its
approach a millimetre before the stub, and at last call SA1 and SA2
closed the B corridor at s 10 while SA9 and SBA0 walled F at the tooth.

`connect(report=)` now hands a refusal's blocked FRONTIER back -- the
cells the A* tried to expand into and found blocked, the window it
searched, the config -- and `Corridor.rip_for` attributes it to the
lanes of this run with the production router's own blocking analysis
(`blocking_analysis.analyze_frontier_blocking`, the one route.py's rip
ladder uses). The frontier ranks lanes by EXPOSURE (the perimeter of the
reachable pocket), not by whether ripping them opens anything, so a
MIN-CUT PROBE measures it: one more search with every lane of this run
PRICED instead of blocked (`connect(soft=)`, the take4 mechanism, ported:
the clearance footprint of each lane stamped as a per-cell cost through
`set_layer_proximity_batch`) finds the path that crosses the fewest of
them, and the lanes that path conflicts with, in path order, are the cut
set -- jointly sufficient by construction. The rip ladder is that set's
prefixes, then the most exposed lanes singly: each trial rips its
victims, routes the refused lane against the rest through its ladder,
re-lays each victim against the new lane through ITS ladder (band first,
so a page lane stays on its page when it can), and a victim that cannot
be re-laid NEGOTIATES one level down with the placed lane protected (the
PathFinder move, in the braid's own vocabulary). The state is kept only
when the refused lane and every victim route; otherwise every piece of
copper goes back exactly. Static copper is never a victim: a lane whose
probe finds no path even with every lane priced is walled by stubs, pads
or other nets, says so and stays open -- a fanout matter.

Measured on K41's SBA2 (chain, one fanout, byte-identical fanout
boards; K15 14v / K28 38v / K35 0 open 54v identical, no last-call
refusal to rip):

- exposure order alone: SA8 (766 of 20000 frontier cells) ripped, SBA2
  routed at 6 vias -- a ride south round the bundle and up the
  destination's west face -- SA8 re-laid at 4 then 0 in the economy
  re-lay: **1 open 78v -> 0 open 84v**, the rip 1.4 s;
- the min-cut probe: SBA2's straight B lane, 0 vias, crosses SCKE1, SA1,
  SA2, SDQ6, SA3, SCKE0; ripping SCKE1 routes SBA2 at 4 but SCKE1 is lost,
  the pairs likewise lose SA1 -- so the cut set alone is not enough;
- the cut set with one level of negotiation: SCKE1 ripped, SBA2 at 4
  vias, SCKE1 refused and negotiated in turn by ripping SA8 (SCKE1 2
  vias, SA8 2 then 0): **0 open 82v, 0 DRC**, the rip 21 s. The first
  complete K41 on this chain (human 70v).

Time (the chain alone on the machine): K15 15 s, K28 28 s, K35 76 s,
K41 155 s (was 130; fanout 69 of it) -- most of the braid's extra time
is not the rip (18 s on its log lines) but the economy re-lay, which now
has heavier lanes to try at three widening windows each.

K51 on the same chain: the plan named 44 of 48 routable nets (SA9, SDQ7,
SA2, SZQ read off the board), the 47-net corridor routed 31 in-band and
16 at the last call, six of them by the rip (SA4, SCKE1, SA11, SBA1,
SDQ3, SA14; up to three victims, two levels), and the braid then crashed
on the singleton corridor SZQ: `corridor.build_spine` had no initial
polyline for a corridor whose launch and arrival flows bend by more than
30 degrees, because the mean-path relaxation that used to fill it was
pruned from this chain as never reached at K28. The middle is now the
chord between the two end zones in both branches (identical for a
straight corridor). Re-run: **K51 (48 routable nets) 0 open, 0 DRC, 141
vias, 339 s** (fanout 4.5 min, braid 3.2 min; 24 rips landed a lane) --
the first complete K51 on this chain, against the human's 85 vias: the
vias are where the work is now, not completion.

### The destination passes never converge, and the last one ships (2026-09-07)

At K41 the destination loop runs all eight passes (misses 10, 10, 4, 4,
2, 2, 7, 6) because a miss is not a property of the banned move alone: a
berth laid as asked in one pass fails in the next when its neighbours
change. Two things were wrong with what shipped from that, one measured
harmless and one fixed:

- The sidecar was written from the RE-PLAN after the last pass -- a
  choice no board was ever laid to -- with the last pass's `achieved`
  patched over it. At K41 the two choices named the same nets, so no
  copper differed; the sidecar now comes from the last pass fanned out
  and audited (`laid_pass`), which is the only thing it can honestly
  describe.
- "Ship the best pass by audit" was built and measured WORSE: pass 5
  (38/40 berths exact, 40/40 layers) graded 1 open 98v 30/41 in-band in
  216 s against pass 7's (34/40 exact) 1 open 78v 40/41 in 130 s. Every
  pass board was then braided (`tmp/passes_k41.sh`, committed braid):
  passes 0..7 graded 6 / 3 / 4 / 5 / 3 / 1 / 1 / 1 open at 86 / 108 / 92
  / 74 / 81 / 98 / 102 / 78 vias (pass 6 with 6 DRC). The last pass is
  the best on this board, and neither the audit's exact count nor the
  judge's cost of the choice (pass 6 the lowest at 152.56, pass 7
  156.88, pass 0 160.93 -> 6 open) predicts the braid's grade. So the
  last pass ships, as before -- with the ban set that makes it the
  engine's most feasible choice -- and the fanout's convergence stays a
  fanout-stage problem: SA9's menu is banned away by pass 3 and it is
  fanned out unplanned every time (`plan from X: 40 of 41 nets; SA9 read
  off the board`).

### A second array pair: `make_bench.py`, and the ladder beside its board (2026-09-07)

Every rule above is measured on one bench. `make_bench.py BOARD SRC DST
OUT [--two-layer]` now prepares an article from any board the way the
first one was prepared: the inner copper layers and the zones on them
removed (`--two-layer`: the braid is a two-page router, and the corpus
holds no 2-layer BGA-to-BGA DDR pair), the pair's two-pad nets found,
SRC fanned out for them with the chain's own destination engine call
(`fanout_from_plan.fanout_once`: the production engine, F/B, the
braid's track / clearance / via, foreign parts immovable, no plane
drop; a refused net is left out of the ladder), the project stamped
with the chain's floor (`fix_project_for_output`; a stock 0.2 mm class
graded a clean 0.1 mm fanout as 959 phantom violations), the article
DRC-gated, and the ladder written beside it from the plan's own river
detection (`plan_state` -> detect_buses on taut paths; whole rivers,
largest first, singletons last). `coherent_nets.py` reads `<board
stem>.ladder.txt` beside a bench board when there is one, else the
bench's `k_ladder_coherent.txt`; `chain_k.sh` passes `BASE` through
(`--board=`), `fanout_from_plan.main` passes its base, and the parser's
board warnings go to stderr (they were on the stdout a chain captures as
the net list, and the braid received them as net names).

The second article: the corpus's `zynq_ad9364` (Zynq CLG400, 0.8 mm,
U1 -> DDR3 BGA-96, U2; 46 two-pad nets, 44 with a tooth -- A14 and ODT
refused), six rivers of 11/9/8/6/6/4 nets:

    python3 make_bench.py .../zynq_ad9364.kicad_pcb U1 U2 tmp/bench2/zynq.kicad_pcb --two-layer
    BASE=tmp/bench2/zynq.kicad_pcb DEST=U2 bash chain_k.sh z 11 20 28

Its first hour: K11 0 open 24v 30 s, K20 0 open 32v 51 s, K28 0 open
55v 0 DRC 136 s -- complete, but **17 of 28 in-band** where the first
bench has 28 of 28: the in-band execution is what the leg rules learned
on one board, and that gap is the next thing to probe there. It also
found the braid routing to the config's default hole-to-hole (0.2 mm)
on a board whose project declares 0.25, one drill-to-drill graze at
K28: `setup` now reads the board's `min_hole_to_hole` and tightens to it
(tighten-only, so the first bench at 0.127 routes as before).

### The pose gate: rotations and faces (2026-09-07)

Take4's rotation gate is back (`rotate_board.py`, a whole-board rotation
that walks the s-expression by depth -- a footprint's nested coordinates
are local and ride along -- and self-verifies every pad, segment and
via against the transform), and widened to the FACES: `make_bench.py
--src-side B` / `--dst-side B` put an array on the other face through
the placement writer's mirror (the #714 path), and every part its pads
then collide with (the decoupling caps under a BGA sit on the far face)
goes over with it until the article is pad-clean; `--rotate DEG`
rotates the finished article. A fanned bench can be the input: the
pair's copper is stripped first, so the source is fanned out in its
final pose. `pose_gate.sh BOARD SRC DST K...` builds FF (the control),
BF, FB, BB, R90, R180, R270 into `tmp/gate/`, puts ONE ladder beside
every pose (`LADDER=`, else FF's own) so the K prefixes name the same
nets everywhere, runs the chain on each, and prints the table.

A rotation is an isometry: a grade that changes there is a stage leaning
on the board's axes. A side switch is a different article, so its grade
may differ -- but the chain must complete it, and nothing may assume a
tooth is on F. First reading, the bench (`fb_t2q_fresh`, its own copper)
rotated 90 degrees: K15 0 open 12 vias against 14, K28 0 open 36
against 38, 0 DRC both. The PLAN is invariant -- the same predicted
vias (16, 38) and the same judged cost (40.99, 95.86) in both frames --
and the difference is one lane each (SA9 at K15 2 -> 0, SA4 at K28 4 ->
2), a swimmer the grid router laid cheaper in the rotated frame: the
A* lattice is the stage that leans on the axes, not the braid's rules.

The gate on the origin board (`allwinner_h3_ddr3` unrouted, the human's
passive poses, a fresh source fanout per article, the bench's ladder;
`LADDER=k_ladder_coherent.txt bash pose_gate.sh tmp/gate/h3.kicad_pcb
U1 DU1 15 28`), open / DRC / vias / in-band:

| pose | K15 | K28 |
|------|-----|-----|
| FF (control) | 0 / 0 / 16 / 13 of 15 | 0 / 0 / 38 / 22 of 28 |
| BF, source on the back | 0 / 0 / 21 / 14 | 0 / 0 / 46 / 23 |
| FB, destination on the back | 0 / 0 / 21 / 11 | **1 open** / 0 / 47 / 21 |
| BB, both on the back | 0 / 0 / 24 / 13 | 0 / 0 / 44 / 25 |
| R90, the FF article rotated | 0 / 0 / 14 / 13 | 0 / 0 / 38 / 22 |
| R180 | 0 / 0 / 16 / 13 | 0 / 0 / 38 / 22 |
| R270 | 0 / 0 / 14 / 13 | 0 / 0 / 38 / 22 |

Three readings. Every back-side article completes (the one open, SA0
with the destination on the back at K28, is a rip whose victim SDQ14
lost its own victim one level down -- the re-berth TODO below), so
nothing in the chain assumes a tooth on F; a back-born tooth reaching
a front berth or a dog-bone costs about a via per lane, which is what
the +5..+9 vias are. The rotations are exact at K28 in all four frames
and exact at 180 degrees at K15, and two vias cheaper at 90 and 270:
the octilinear lattice's relation to the lanes is what a quarter turn
changes and a half turn keeps, so the residual is the router's grid,
not a rule. And a FRESH fanout on a rotated board is not invariant at
all (the first run of the gate, `tmp/gate/h3_gate1_*`: K28 38 / 42 /
54 / 38 vias across the four frames, R180 at 28 of 28 in-band) --
`bga_fanout`'s escape order leans on the axes, which is why
`make_bench.py` rotates after the fanout and why the fanout's own
sensitivity is a finding for the engine, not for this chain.

**The board turned over.** Both arrays on the back should grade like
both on the front -- a reflection through the board's plane is an
isometry too -- and the gate's BB pose (24 / 44 vias) is nowhere near
FF (16 / 38). The fresh back-side fanout was one reason (611 tracks and
6 vias for the same balls the front fans out in 502 and 8, four teeth
on the far side) -- FIXED in the engine: `bga_fanout/flip_frame.py`
turns the board over in memory for a part on the back, runs the
pipeline on the part now on F and mirrors the copper back, the way
`rotate_frame.py` handles an angle, so a chip on the back now fans out
as the exact mirror of the same chip on the front (0 of 51 escapes
differ on the origin board and its mirror; `tests/test_fanout_flip_frame.py`
pins it with a change detector). Before that, `mirror_board.py` had
turned the FANNED front article over instead -- every part to the other face through the placement
writer, y mirrored, every layer swapped, self-verified -- and the chain
on that mirror measures its own front call-outs alone: K15 16 vias (=
FF), **K28 50 against 38**. Every literal `F.Cu` in the chain was then
read (`grep`): the schedule seeds its pages symmetrically by tooth layer
and `divers` only feeds a log line; the braid breaks two exact ties to
F (an exit block's shared leg layer on an even split, a leg's layer at
equal cost) and filters back-side required stretches near s1 only; and
the taut paths, the spine and the plan's pad-clear test relaxed against
FRONT copper by name. That last one is fixed -- the layer the majority of
teeth are born on (`braid.bundle_layer_of`), identical on the bench by
construction -- and moved the mirror to 48. What remained was the PLAN:
the front chose 17 berths on its down face where the mirror should
choose 17 on its up face and chose 9 up, 12 down, with the predicted
vias 55 against 37; the mirrored geometry ties every cost exactly, so
the selector's tie-breaks -- the menu's gap order, the face iteration
order, a first-index LIS, a quarter-turn axis, a crossing test that
counts a shared endpoint on one side only -- decided, and none is
mirror-invariant.

**The selector's frame** (`select_moves.PairFrame`, 2026-09-08). Two
answers were measured. Making every tie canonical (invariant keys, an
oriented axis, a symmetric crossing test) did make the selector
symmetric, but the crossing count had been tuned into the plan: the
physical count lost the bench at every K and every weight (K28 38 ->
44..52, K41 82 -> 114), and the tie changes alone turned K41 into a
different draw (100 vias; five single reverts all 100..112). So the
handed selector stays exactly as it is, and runs every pair in the
pair's own canonical frame instead -- the move `flip_frame` makes for
the fanout engine, done at the selector's boundary: `pair_chirality`
reads the sign of the run's BALLS' moment about the line between the
two array centres (+1395 on the front article, -1395 on its mirror,
positive on the bench at every K and on the zynq; the balls, not the
teeth, because the plan's rounds move the teeth and at K15 their sign
flipped at round 1 and mirrored the bench against itself), and a -1
pair has its menus, launches, box and pads mirrored in, the chosen
moves mapped back by identity. Three things the mirror alone did not
give, each found by comparing the two worlds stage by stage: the
menus re-sorted into the generator's own order (`menu_order`, read off
a move's geometry and verified equal to the generated order on every
net of the bench, the origin board and the zynq -- the selector breaks
ties by list order), the layer NAMES swapped (the refinement sorts
slots by name), and a mirror line on the 0.0005 mm lattice. The judge's
`plan_pages` and the source refinement take the same frame. Read off
the pair alone, so a board with three arrays gives every pair its own
frame; a +1 pair never enters the wrapper, so the bench is unchanged
by construction: K15 14 / K28 38 / K41 82, same segment counts.
Measured on the origin board and its mirror: the selector alone
chooses identically through every stage (0 of 15, 0 of 28 differ);
the chain then graded K15 16 = 16, K28 38 against 40, the residual
being the braid's own planner, which judges the plan loop's rounds and
still broke two ties toward F.

**The braid's frame** (`braid.setup`, 2026-09-08). The same move at
the braid's boundary, for the planner (`plan_braid`, the judge) and the
braid alike: `setup` reads the pair's chirality off the same balls and
boxes as the fanout (`pair_chirality_of`), checks it against the one
the plan was made in (`plan['chi']`, written by `braid_plan_of`), and
for a -1 pair turns the board over in memory with the engine's own
`flip_frame.to_front_frame`, mirrors the plan into that frame
(`mirror_plan`: ends, layers, escape directions), and runs everything
unchanged; `plan_braid` swaps the pages and leg layers back and
`write_out` mirrors the copper, the Eco overlay and the refusal report
back. A +1 pair never enters it, so the bench is unchanged to the
segment (K15 14 / 673, K28 38 / 1530, K41 82 / 2051). Two things the
frame alone did not give, each found by comparing the two worlds stage
by stage until the planner agreed on every net:

- the braid's obstacle memo (`_OBS_MEMO`) is keyed on the board FILE,
  and the turned board is, as a file, the same board: it was handed the
  real board's model, on which the mirror's front layer holds 22 discs
  where the turned board's front holds 473, and every taut path was a
  straight chord where the front's bent. The turned copy now carries
  `frame_axis` and the key includes it. (The taut memo is content-keyed
  and was never wrong, only cold: the mirror's first run at K15 took
  61 s to the front's 21, and 21 s warm.)
- the engine's mirror line was the bounds' centre, off the lattice;
  the router's grids are anchored at the origin, so a mirror about it
  mapped the grid onto a grid shifted by a fraction of a cell, and the
  turned board fanned out with 0.2 mm jogs the front did not have (50
  of 266 segments at K15). `mirror_axis` now snaps the line so that
  twice it is a multiple of 0.1 mm, and the bounds are mirrored with
  everything else (the flip-frame test asserts it).

Measured with that: the mirror's plan is the front's to the letter at
every round, its fanout has the same escapes and vias, and the chain
grades **K15 16 = 16 (466 against 464 segments), K28 38 = 38 (1687 =
1687)**. The segments that still differ are the fanout engine's
exact-edge cells: the plan's exit points sit at pitch fractions that
land exactly on the occupancy grid's cell edges, and a plain truncation
lets the last bit of floating-point noise choose the cell. An epsilon
before the truncation made the copper identical in both frames and was
measured and REJECTED: it moves every exact-edge decision on every
board, the bench included (K28 38 -> 36, K41 82 -> 85 and 120 s
slower), because those decisions had been made by the same noise when
the bench was tuned. The bar for the mirror is the same grade, not the
same copper (user decision, 2026-09-08).

**Non-orthogonal rotations.** 30 degrees: K15 complete at 33 vias, 6 of
15 in-band; K28 0 open but 15 DRC, 95 vias, 12 of 28 in-band, 661 s.
The plan's faces are compass directions (`DIRS`), the audit measures a
gap ALONG a face by x or y, and the router's lattice is octilinear, so a
pose off the axes is outside both models today; that it completes at
all is the braid's ladder. 45 degrees is the octilinear-friendly angle:
K15 complete at 20 vias, 11 of 15 in-band; at K28 the plan stage itself
fails (`endpoints`: a realized tooth's free stub end not found on the
rotated copper), so the pose is refused before the braid.

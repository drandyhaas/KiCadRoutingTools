# awx -- the K-bus chain (#622): one plan, a fanout that follows it, a braid

The tool set for routing a fanned-out bus between two BGAs (the
`fb_t2q_fresh` bench: an FPGA `U1` and a DDR3 `DU1`, the coherent
K-ladder), where the PLAN decides both ends of every net, the FANOUT lays
exactly the plan's moves (and tells the plan what it could not), and the
braid routes the lanes:

    bash chain_k.sh TAG 4 8 15 28   # -> tmp/TAG_k<K>.kicad_pcb, graded

Results on the bench (2026-09-06), against the previous chain on the same
engine (a face-hinted `auto` fanout, the bench's own source teeth):

| K  | previous vias | this chain | in-band at attempt 0 | plan's own prediction | human |
|----|---------------|------------|----------------------|-----------------------|-------|
| 4  | 4             | 4          |                      | 4                     | --    |
| 8  | 8             | 6          |                      | 6                     | --    |
| 15 | 22            | 14         | 15 / 15              | 14, per net           | 22    |
| 28 | 38, 0 open    | 38         | 28 / 28              | 36 (the swimmer SA4 at 4) | 46 |
| 35 | 56            | 62         | 27 / 32 + 0 / 3      | 59                    | 58    |
| 41 | 9 open, 92    | 8 open, 84 | 30 / 37 + 0 / 4      | 90 (18 swimmers)      | 70    |

All complete and DRC-clean at the routed 0.1 mm floor (K15 ~20 s, K28
~70 s, K35 ~170 s with a warm taut memo; the plan loop is most of it).
The plan's prediction is exact per net at K15; at K28 every page lane is
laid on its prediction at the first attempt and the residual is one
swimmer; at K35 the total meets the prediction while ten swimmers pay
2..4 each (see the via model and the walls below). Every board the chain writes
-- each realized source board, the fanout board, the braided board --
is DRC-gated at 0.1 with the quantization margin, and the fanout boards
are clean with the margin off as well.

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
     ban -> re-select loop that ends only at "every berth laid as
     planned". The engine is the authority on what is possible.
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
   and route refused lanes earlier; refused lanes then get a wider last
   call, and lanes with three or more vias an economy re-lay that is
   kept only when strictly cheaper. What is still refused is reported
   and left open. The output is smoothed (the repo's octolinear pass)
   and written with an Eco overlay of the planned lanes.
4. `grade_k.py BOARD NETS` -- connectivity scoped to the run's nets,
   whole-board DRC at the routed floor, the via census
   (`via_census.py`).

No environment variables, no options beyond BASE / DEST (the inputs).

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
`chain_k.sh` now stamps the fanout and braid stage boundaries.

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

## History: what `bus622-take4` still has

This tree was cut from the `bus622-take4` branch at `7c384245`
(2026-09-06) by keeping only what the K28 chain executes, measured
with a function trace and a line trace of the chain run in-process.
Everything below is on that branch, most of it behind an env knob or
a flag; the knob names are given so `git show bus622-take4:awx/FILE`
finds the code. None of it changed the K28 copper.

### Mechanisms cut from the modules kept here

`braid.py`
- RIVERS (`river_groups`, `BRAID_RIVERS=face`, `BRAID_RIVER_XT/MIN/
  ORDER/PAGES/FOLD`, `BRAID_XRES_FULL/END`): corridors by destination
  face and passing side, a page per river coloured by the crossing
  graph, tiny rivers folded into the river they cross most, and the
  `.rivers.json` sidecar for the fanout. K41 2 open vs 5 for the single
  corridor, but more vias at every K; grouping without faces lost.
- Lane PACKING (`pack_lanes`, `relax_attract`, `BRAID_PACK`).
- NEGOTIATION (`negotiate_refusals`, `BRAID_NEGOTIATE`, `BRAID_NEG_*`,
  the post-smoothing negotiate in `write_out`; module `negotiate.py`
  with `connect(soft=)` blocker discovery).
- TAIL RESCUE (`TAIL_RESCUE`: the generalist `route.py` on what the
  braid refused).
- The page SIDECAR (`.pages.json`, `Schedule(pages=)`, the
  `--plan-json` plan dump, `BRAID_RIVER_PAGES`) and the chain's
  ROUND TRIP (`chain_k.sh ROUNDTRIP=1`, `src_roundtrip.py`).
- LANDING STRIPS (`landing_strips`, `BRAID_LAND`); the OBSTACLE CACHE
  (`connect.ObsCache`, `BRAID_OBS_CACHE`); lane PIECES (`LANE_PIECES`);
  finishing at any dest-stub vertex on primary lanes (`BRAID_BALTS`);
  the FACE COMB re-slotting (`FACE_COMB`); STAY-ON-PAGE requirements
  under a different-layer leg (`BRAID_STAY_PAGE`); the end-clash rule at
  the legal minimum (`BRAID_END_CLASH=min`); via cost and A* heuristic
  weight knobs (`BRAID_VIA_COST`, `BRAID_H_WEIGHT`); `LANE_DEBUG` /
  `BLOCK_DEBUG` / `debug_lane`.
- The RIP ASSIST at last call (`_rip_assist`, `RIP_CALL`); the
  BEST-ATTEMPT restore in the retry loop; FREE-CORNER corridors of 2..5
  lanes routed band-free (`run_free`, `FREE_CORNER`); SHORT corridors
  routed as tubes (`run_short`); the swimmer TUBE band (`FREE_SWIM=0`,
  `connect.tube_band`); `--no-smooth`, `--cluster`.
- The SWAP-COLUMN wave schedule (`plan_columns`, `column_layout`, the
  column loop in `lay_lanes`, the order morph, the `gaps`/`lead`
  feedback, `SCHED_GATE`, `HOP_RESERVE`): on the two-page ribbon it
  always produced no columns.
- Corner-wedge and cross-corridor interval RESERVATIONS
  (`reserve_intervals`, `in_cross`, `_intervals_union`): inert on a
  straight single spine. `cross_reserve` (planned lanes of the other
  corridors as virtual copper) is still here.

`schedule.py`: `columns` / `col_layers` / `pair_layers` / `is_free` /
`ensure_recolor` (the wave schedule and the `BRAID_WORD` recolouring),
the `TWO_PAGE_B` policies (`wmax`, `lis`; `worst` is what is kept),
`TWO_PAGE=0` single-page mode, verbatim pages with the `BRAID_RIVERS`
other-page fallback.

`fanout_from_plan.py` (1676 lines on take4, 221 here): the SOURCE
apply (`--source`, `SRC_APPLY`, `TP_SRC_B`, `TP_SRC_B_NETS`,
`SRC_OBJECTIVE`); the face restriction (`--dirs`, `flow_frame.py`);
exit-LINE hints (`--no-lines` off, `escape_line_hints`); the ORDER
MODEL (`--order-model`, `plan_order.py`); the two-page re-pick scopes
(`TP_SCOPE=pages|swim|surgical|split`) with the destination B PASS
(berths re-laid on the back layer: `TP_SPLIT_NETS`, `TP_SPLIT_ONLY`,
`TP_RELAY_NETS`, `TP_EMIT`, `TP_PAD_PROX`, `TP_RIDE_TUBE`, `TP_FAR`,
`TP_DEBUG`) and its capacity gate; the asks dump (`_asks.json`);
`PLAN_DUMP_BUSES`, `PLAN_ONLY`, `--no-hints` (the negative control),
`--escape-method=`, `--pages-json`.

`plan_ends.py` / `select_moves.py`: the order model (`model=`,
`refine_faces`), the `spend` objective (`true_vias` for an applied
source), the WALL term (`PLAN_WALL`), forced sides (`PLAN_FORCE_SIDES`),
bus dumps (`dump_buses`, `PLAN_DUMP_BUSES`), `RIDE_MM_PER_VIA`,
`only_dirs`, site-block conflicts, `summarise`.

`corridor.py`: mean-path spines relaxed against ramped obstacles
(`mean_path`, `relax_path`, `resample`), bent-spine corner geometry
(`lane_xy` arcs and mitres, `project` wedges, `dir`), the degenerate
spine fallback. `connect.py`: `ObsCache`, soft-copper pricing
(`soft`, `_stamp_soft`), the tuple and dict band forms, `tube_band`,
`net_clearances` / `track_width` overrides, `CONNECT_DEBUG`.
`topo_strings.py`: its CLI (`main`), `crossings`, `hugs`, `in_field`,
`polyline_len`, the unpacked `point_violation`, grid-less `near_*`
fallbacks. `taut_clean.py`: the reseed policy (`TAUT_RESEED=1|disc`,
`relax_from`, `reseeds`). `detect_buses.py`: single linkage.

### `py_router` on take4 that was not ported

`escape_line_hints` (the exit gap line per pad), `escape_layer_hints`,
`escape_slot_hints`, the LANE-RUN escape in the under-pad engine
(`_lane_run_escape`, `KICAD_FANOUT_LANE_ESCAPE`), the dog-bone gap-site
override on the planned side, and branch-side edits main never
received (an A* heuristic variant and `layer_assignment` changes in
`bga_fanout`, `kicad_writer.swap_pad_nets_in_content`). The branch's
whole `bga_fanout` diff reproduces K28 too, but it deletes 55 lines of
main's code; the port here is additive.

### Tools never copied (136 files on take4, by purpose)

Drivers and searches: `plan_search.py` (plan candidates judged through
the chain; the K35 60v and K41 79v records), `improve_k.py` (single-net
rip and re-braid loop; slack harvest), `harvest_k.py` (the calibrated
ledger's slack harvest), `close_net.py` (the close ladder for one
stranded net; K51 121v complete), `retry_chain.py` (directed iteration
on refusal at chain level), `drive_k.py` (complete, then cheapen),
`floor_evolve.py` / `floor_compose.py` / `floor_sweep.py` (judge-guided
floor moves), `surgical.py` (one net swapped or stripped and braided
alone, `drc_partners`), `negotiate.py`, `relay_net.py`,
`channel_shift.py`, `src_roundtrip.py`, `prune_debris.py`,
`nudge_grazes.py`, `collapse_dives.py`, `run_ladder.sh`,
`run_pinpages.sh`, `run_rec44.sh`, `river_loop.sh`.

Plan-side models: `plan_global.py` + `plan_lattice.py` (ledger, gap
lattice menus, solver), `plan_nest.py` (homotopy nesting), `plan_order.py`
(the braid's order model), `plan_fanout.py` (human-derived hints),
`plan_model_check.py`, `group_pages.py` (joint ride assignment),
`sched_whatif.py`, `solve_probe.py`, `cut_ledger.py` (cut-capacity
ledger), `ledger_cal.py` (calibrated floor judge), `flow_frame.py` +
`rotate_board.py` + `make_bench.py` + `chain_rot.sh` (the rotation
GENERALITY GATE: a rotated bench must grade the same), `sweep_cross.py`,
`scale_check.sh`, `free_plan.sh`, `fanout_ladder.sh`, `fanout_plain.sh`,
`fanout_k.sh`, `braid_k.sh`, `ladder_n.sh`, `ladder_head.sh`,
`measure_braid.sh` (opens / vias / in-band tally).

Diagnostics: `band_dump.py` (the world one lane's router saw, as a
picture and a census -- what found the 0906 walls), `band_check.py`,
`band_conn.py`, `blocked_cells.py`, `congestion_where.py`,
`diagnose_open.py` (why each open net is open, with a picture),
`dump_net.py`, `dup_ends.py`, `ends_of.py`, `net_faces.py`, `near_pt.py`,
`line_probe.py`, `via_probe.py`, `via_where.py`, `wall_census.py`,
`drc_census.py`, `audit_nets.py`, `audit_fanout.py`, `prof_top.py` +
`prof_k.sh` + `run_prof_chain.sh`, `test_connect.py`,
`test_taut_clean.py`, and the `probe_*.py` family (30 files: buses,
clusters, corridors, faces, handoff, lanes, LIS headroom, ports, reach,
req, select, side order, source menu, stub ends, targets, trace, why).

Human comparisons: `census_vs_human.py`, `compare_human.py`,
`human_asks.py`, `human_at_k.py`, `human_fanout.py`, `escape_census.py`,
`hfan_overlay.py`, `classify_k51.py`.

Rendering and handoff: `render_eco.py` (the Eco overlay renderer),
`render_all.sh`, `render_chain.sh`, `render_plan.sh`, `gallery.py`,
`make_overlay.py`, `make_handoff.py`, `handoff_all.sh`, `view_of.py`,
`deliver.sh`; grading wrappers `grade_all.sh`, `grade_kb.sh`,
`grade_one.sh`, `check_k.sh`, `drc_k.sh`, `drc_of.sh`, `try_bench.sh`,
`run_fanout_tests.sh`.

Benches: `fb_t2q_base` (the original ladder bench; FIVE back-side
passives under DU1 sit where our own placement step moved them, not
where the human has them) and `fb_t2q_hp` (the human's poses restored;
the target for any exact planner). The records K28 34v / K35 60v /
K41 79v / K51 117v were set on `fb_t2q_base` and are not like-for-like
with this bench.

### Not preserved anywhere (2026-09-06)

The last session's `awx/dogbone_berths.py` -- dog-bone berths at the
ball for a named set of nets: a via in the diagonal gap, a 45-degree
stub, and either a short back-layer leg or a RIDE routed by `connect()`
to the face or the source-facing edge, the side chosen by a
two-subsequence cover of the launch order -- and its braid-side
companions (rivers keyed on the axis flank or the stub position,
`_end_dir` stopping at a same-net via, a B-berth river coloured B) were
working-tree edits in the old worktree that were never committed, and
were gone from that tree when it was removed. Both measured and lost at
K41 (the braid's schedule still sees one ribbon order), so nothing to
revive; the design is recorded in the session notes. The old worktree's
record boards (the `*_final` and `*rec*` outputs of the take3/take4
ladders, 165 MB) were copied to
`~/Documents/kicad_stress_test/bus622_take4_records/` before removal.

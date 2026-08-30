# #622 topo bus routing — take 4 (`bus622-take4`)

A bus is routed in three stages, each a separate program, each graded on
its own board before the next runs:

```
plan  ──►  fanout the berth  ──►  braid: corridors + schedule + router-routed lanes
(fanout_from_plan.py)             (braid.py + corridor.py + schedule.py + connect.py)
```

Bench: `fb_t2q_base.kicad_pcb` — U1 (347-ball 0.65 mm BGA, the source,
already fanned out) to DU1 (96-ball 0.8 mm DDR3, the destination), the DDR
subset of `allwinner_h3_ddr3`. **The board is only a test article.** Nothing
in the routing reads a footprint name, a net name, a coordinate, a face or an
axis: every quantity comes from the board's geometry in a frame derived from
it. `fb_d45.kicad_pcb` (`make_bench.py`: DU1 rotated 45° and shifted
diagonally, four colliding caps dropped) is the article that has no axis to
lean on; the rotation gate (`chain_rot.sh`) is the other.

## Run it

```bash
bash chain_k.sh TAG 4 11 15 19 21 -- --no-plane-drop        # the chain, plan free to use every face
DIRS=left,down bash chain_k.sh TAG 4 11 15 19 21 -- --no-plane-drop   # the A/B arm: plan restricted
bash chain_rot.sh 15 19 -- --no-plane-drop                  # the rotation gate
bash braid_k.sh FO_BOARD TAG K                              # braid stage alone on a fanout board
bash ladder_n.sh TAG 4 11 15 19 21                          # braid stage over the ladder (FO=prefix)
bash ladder_head.sh TAG 4 11 15 19 21                       # the same with HEAD's braid (braid_head.py)
bash fanout_ladder.sh 4 11 15 19 21                         # fanout boards, restricted (r_) and free (f_)
bash fanout_plain.sh fb_d45.kicad_pcb OUT 11                # plain bga_fanout (no plan) for a rotated array
python3 probe_lane.py FO_BOARD K NET [--after]              # route ONE lane under each constraint set
python3 probe_plan.py FO_BOARD K                            # the corridor plan: legs, offsets, layer rules, no routing
python3 probe_req.py FO_BOARD K                             # lanes whose rules close BOTH layers somewhere
python3 probe_map.py FO_BOARD K NET --box x0,y0,x1,y1 [--no-virtual] [--drop NET]   # ASCII obstacle+band map of one lane
python3 probe_trace.py FO_BOARD K [--nets ..] [--box s0,s1,o0,o1]    # routed copper in (s, o) against the plan
python3 blocked_cells.py PROBE_OUTPUT [n] [x0 x1]           # a CONNECT_DEBUG failure's frontier, in mm
python3 ends_of.py FO_BOARD K                               # every net's two free ends + corridor grouping (diff two fanouts)
python3 dup_ends.py BOARD [REF]                             # stub ends two nets share at one point
python3 probe_cluster.py FO_BOARD K                         # the corridor grouping's decisions
python3 probe_reach.py FO_BOARD K NET                       # replay the corridor reach test for a split-off net
python3 band_conn.py FO_BOARD K NET [--copper]              # flood one lane's band tooth->stub (and minus the router's copper): where it pinches
python3 band_check.py FO_BOARD K                            # band_of against its per-cell reference: differing cells + time
bash prof_k.sh FO_BOARD K OUTSTEM                           # cProfile of one braid run (prof_top.py prints the top)
python3 compare_human.py OUR_BOARD K [--top N]              # ours vs 00_human_original on the same nets: vias, copper, per net
python3 human_at_k.py [K ...]                               # the human's via count per checkpoint
python3 net_faces.py BOARD NET [NET ...]                    # per net: faces left/entered, vias (in an array / open), copper per layer
python3 sched_whatif.py FO_BOARD K [--exit-block NET,..]    # the schedule under another target order; two-page count
python3 plan_model_check.py FO_BOARD K                      # the plan's order model vs the braid's orders on a recorded board
PLAN_OPTS=--order-model DIRS=left,down bash chain_k.sh TAG 21 28 -- --no-plane-drop   # the chain with the order model (opt-in)
python3 test_connect.py ch6_fo_k15.kicad_pcb SDQ0           # connect() unit test
```

K is a checkpoint of the **coherent K-ladder** (`k_ladder_coherent.txt`,
read by `coherent_nets.py`): whole rivers only, so a prefix never splits a
bus. Checkpoints 4 / 11 / 15 / 19 / 21 / 32 / 47 / 51.

Grading is `grade_k.py`: `check_connected.py` scoped to the run's nets,
whole-board `check_drc.py --clearance 0.1 --clearance-margin 0.1`, and a
via/segment census from the parsed board. Never a program's own tally.

## Results (2026-08-30, corridor braid, restricted-plan boards `r_fo_k*`)

| K | corridor braid | HEAD braid (`46a268b3`) | human |
|---|---|---|---|
| 4 | **4 vias**, 0 / 0 | 4, 0 / 0 | 4 |
| 11 | **10 vias**, 0 / 0 | 10, 0 / 0 | 14 |
| 15 | **16 vias**, 0 / 0 | 18, 0 / 0 | 22 |
| 19 | **23 vias**, 0 / 0 | 26, 0 / 0 | 30 |
| 21 | **28 vias**, 0 / 0 | 30, 0 / 0 | 34 |
| 28 | 75 vias, **1 open** / 0 (`kh_fo_k28`, gate off) | — | 46 |
| 32 | 56 vias, 10 open / 7 (`kh_fo_k32`; the 7 are the fanout's, SA12 through R1) | — | 52 |

(braid vias; open / DRC; the human column is `human_at_k.py`, the same
nets on `00_human_original`, and counts EVERY via on them — our
whole-board totals on the same terms are 4/10/20/28/34 for K4..K21:
below the human at K11..K19, level at K21.) `compare_human.py OUR_BOARD
K` puts the two side by side per net: at K28 the human spends 1.64
vias/net (2 on every corridor net, 0 on three data lanes it keeps on
one layer end to end) and 733 mm of copper; we spend 2.68 and 596 mm —
9 nets equal, 3 fewer, 16 more, all of the "more" side exits at 4 and
6 (SA6, SCKE1): the exit-leg layer rule dives every lane a leg crosses,
and the ungated schedule surfaces a diver passed mid-flight. Both arms on identical inputs (`fanout_ladder.sh`,
`--no-plane-drop`), graded the same way. The whole ladder is clean, at fewer
vias than HEAD from K15 up. Before the exit-block rework of 2026-08-30 the
same code refused 4 lanes at K19 and 7 at K21 — all flank joiners — and
spent 20 at K15; what was wrong and what changed is under "Launch and exit"
and "Known walls". The 45° bench (`d45_fo_k11`, plain fanout) routes 9 of
11 lanes at 20 vias, DRC-clean (SDQ14, SDQ15 open; was 5 of 11).

**K28 and K32 (2026-08-30 pm)** are a COLUMNS wall, not a speed one. The
schedule region between the two escape fields is the same 4.5 mm at every
K (`s0` 7.9 → `s1` 12.9); K4..K21 needed 6/13/13/13/13 columns of it at
W ≥ 0.34, K28 asked for 31 (W 0.147, where no via fits between two
passes) and routed 9/26. Three things took it from 17 open to 1: the
schedule's take-off gate (`last`, and OFF when the columns do not fit —
`W_GATE`, see "Schedule"), 17 → 7; the corridor reach test stepping a
flank end outward as the join block does (SODT1 had been split into its
own corridor and collided with the main one), plus the virtual copper's
head and tail pieces on their end's own layer only (two ends at one point
on two layers, K28 SWE/SDQ14 and SA4/SA1), 7 → 1. The last lane, SDQ7, is
C5 — a decoupling capacitor sitting INSIDE the corridor's cross-section
on F, exactly where the schedule requires SDQ7 on F (`band_conn.py
--copper` shows the pinch; without copper the band is connected end to
end). K32's 10 open are 5 balls the fanout could not escape on the
plan's face (they went up/right — the plan's side-capacity model again,
Known walls) and so formed a second corridor that collides with the
first, plus 5 refused in the 27-lane corridor at 22 columns (W 0.204).

The free-plan arm (`chain_k.sh` without `DIRS`, plane-drop on, so its fanout
boards carry the plane-drop's cap collisions): K15 2 open (SDQ10, SDQM0),
K19 1 open (SDQM0), K21 8 open — from 2 / 4 / 14 with the previous braid on
the same boards. Its refusals are head-on lanes at the launch (SDQM0 at
K15: forward search boxed at its tooth, band-only refused too), not flank
joiners: the free plan sends data lanes out DU1's north and south faces,
and the schedule region then carries side exits on both sides. Rotation
gate at K15 on that arm: 2 open on every rotation (SDQ14/SDQM0, SDQ0/SDQ14,
SDQ0/SDQ14) — the same count unrotated, so nothing leans on the axes, but
the open lanes differ, which says the tie-breaks do.

## Stage 1 — the plan (`fanout_from_plan.py`)

Input: the bench with the source fanned out and the destination bare.

1. `braid.endpoints(pcb, names, byname)` — per net, the source stub's free
   end (the **tooth**) and the far pad (the **ball**).
2. `escape_moves.enumerate_moves` — for every ball, the legal escape moves
   (direction, exit point, layer) off the destination grid, and likewise
   for the source's pads; `detect_buses.taut_paths` + `cluster` group the
   nets into buses by taut-path similarity.
3. `plan_ends.plan_ends(smenu, dmenu, launch, …)` — chooses, per net, an
   exit side and line at the destination (and refines the source teeth),
   minimising the **via floor** `2 × (K − |largest pairwise non-crossing
   set|)` plus inter-corridor crossings (`select_moves`: `Corridor`,
   `plan_floor`, `bus_sides`, `cross_weight`).
4. The plan's **directions** go to the production fanout as
   `escape_dir_hints` (`py_router/bga_fanout`, `generate_bga_fanout`), then
   the emitted copper is measured against them ("plan obeyed: N/N"). The
   plan is free to use every face of the destination (the corridor braid
   delivers to any of them); `DIRS=left,down` restricts it to the two faces
   the pre-corridor braid could deliver to, the A/B arm. Free, the floor is
   much lower (K21 12 vs 20, K32 26 vs 38, K51 48) but the fanout obeys
   fewer of the plan's directions (17/21, 21/32, 34/48): a plan/fanout
   mismatch the corridor braid exposes rather than causes — the braid reads
   the faces off the copper, never off the plan.
   `escape_line_hints` (which row gap) exist but are OFF (`--no-lines`):
   measured to end two nets' stubs at one identical point.
5. `fanout_k.sh` / `chain_k.sh` grade the fanout board on its own before
   the braid runs — a berth that ships stub-vs-stub contact is broken
   before the braid starts.
6. **The source end** (`SOURCE=1` / `--source`, off by default). The plan
   refines the source teeth too (`plan_ends.refine_source`), now against
   what the braid SPENDS — `true_vias` with the source escape's own vias,
   so a tooth the fanout put on B that the corridor delivers on F is
   priced (K15 SRAS) — with each tooth kept on the side it already
   escapes on (optimising the crossing floor alone sent four E-face
   teeth out the north flank, which the braid paid for as joiners of a
   second corridor). The stubs already on the board are seeded as moves
   so the lane checks see every net the plan leaves alone (a new escape
   was planned through the gap an unmoved neighbour's stub occupied),
   and a dogbone's via site blocks its gap on every layer. With
   `--source` the moved nets' source copper is stripped and re-fanned by
   kind (via-in-pad → underpad, dogbone → dogbone, surface → channel,
   in that order) with the plan's directions and lines. Measured at K15:
   the plan's own cost 18 → 14, the re-fanned board DRC-clean and 9/9
   obeyed — and the braid then refuses 5 lanes (14 vias on 10) where
   the untouched teeth route 15/15 at 20. The plan's cost and the
   braid's spend still disagree; the mechanism is in, the objective is
   not yet trustworthy. Note the channel engine ignores
   `escape_line_hints` (only the underpad/dogbone engine reads them).
   **That objective is used ONLY under `--source`** (`plan_ends(...,
   objective='spend')`; the default is `'floor'`, the plan's original):
   run for a chain that never applies the source moves, it still
   re-shaped the DESTINATION choice through the launch points it fed
   back, and the strict lane test it needs (`select_moves._conflict`
   `strict=True`: lanes matched within 0.16 mm, a dogbone's via site
   blocking its gap) was applied to the destination too — the fanout
   takes only a direction from the destination plan, so the strict test
   only removed moves it was free to use. Measured after the fact: the
   restricted K19 plan went from floor 12 to 20, SDQ0 moved from the
   south face to the west, the braid saw two corridors and left 5 open;
   K21 floor 14 → 18, 3 open. **The ladder never saw it because it ran
   on fanout boards recorded before the change — run the CHAIN.**
7. **The dogbone lane-run escape** (`KICAD_FANOUT_LANE_ESCAPE=1`, opt-in,
   ported from bus622-take2 into `bga_fanout/underpad.py`): pad, 45° jog
   into the adjacent gap lane, straight down the lane to the boundary,
   exact-checked against the registries with the raster's flanking-pad
   exemption; tried on the PLANNED side before the side-restricted raster
   A* (which accepts any point of the face and, when the raster closes the
   straight lane, "succeeds" with a 4–6 mm snake down a column gap),
   then on any side after the unrestricted A* fails; a planned line
   (`escape_line_hints`) restricts it to that gap. `KICAD_FANOUT_LANE_DEBUG=
   NET,NET` prints every candidate and what rejected it. Measured on the
   free-plan arm, K21, obedience: auto 17/21; plain dogbone 15/21;
   dogbone + lane run 16/21 (after the A*) / 14/21 (before it); with
   line hints on, auto 17/21 = dogbone + lane run 17/21 (3 vias instead
   of 7). **The same four balls take another face in every arm** (SDQ15,
   SDQ10, SDQ13, SDQM1 at K21; SDQ14, SDQM1 at K19): each planned lane is
   already taken by an earlier ball's escape (the trace names it — SDQ8's
   jog on SDQ15's row gap, SDQ15's own snake on SDQ10's), so this is the
   plan over-assigning the west face's two row gaps, not the escape
   primitive. It is not the lever for the free-plan arm; it stays as an
   opt-in primitive (straight escapes, fewer vias in dogbone mode).
   `dup_ends.py BOARD [REF]` lists stub free ends two nets share at one
   point (the fanout artefact behind K21 SRAS/SCKE1); `ends_of.py BOARD K`
   prints every net's two free ends and the corridor grouping, to diff two
   fanout boards of the same K.
8. **The braid's order model** (`plan_order.BraidOrder`, `--order-model` /
   `PLAN_OPTS=--order-model`, opt-in). The plan's floor was read off a
   transverse projection of launch and exit points; the braid lays lanes
   by other rules — a flank tooth joins from the side and its join block
   puts it OUTERMOST in join order, a side exit takes a slot in an exit
   block (ports by exit *s*, then the joined in reverse join order) — so
   the projection had K28's SWE and SCAS (joiners) next to SCKE0 in the
   middle of the bundle and sent them head-on into the left face among
   the data lanes: 22 of 71 swaps. `BraidOrder` subclasses
   `select_moves.Corridor` with the braid's spine (built from the taut
   pre-routes with the arrival flow), its launch classification and
   offsets rules (fixed once the teeth are), its exit rules (head-on
   unless something upstream at the offset, AND a straight run-in clear
   of the arrays' pads — the first stub along a flank is otherwise
   "head-on" and its tail runs along the ball row), and prices a choice
   by `schedule.Schedule`: **corridor vias** (2 per diver, 2 per lane an
   exit leg crosses, 2 per diver passed in flight when the gated
   columns exceed the corridor's capacity and the braid drops its gate),
   plus the corridor count (`cluster_corridors`' linkage: a second
   corridor is laid against the first's copper). `plan_model_check.py
   FO_BOARD K` replays the model on a recorded board: at K28 its launch
   order matches the braid's to a near-tie (SODT0/SODT1, 0.01 mm of
   *s*), its target order and exit kinds are IDENTICAL. Integration:
   `select()` still places every net by its projection; `refine_faces`
   then moves the JOINERS between faces while the model's cost falls.
   Measured (restricted chain): K4..K19 bit-identical; K21 moves SWE
   down/B (as the human does) — 32 vias from 34 and **2 open** (SA7,
   SA9); K28 moves SWE down — 68 vias from 75, 14 columns from 16, and
   **3 open** (SA1, SA7, SDQ7) from 1. Both times the refusals are
   exit-block lanes behind the late-exiting joiner — a braid defect at
   the exit legs, the next thing to chase; until it is fixed the model
   stays opt-in. Letting the model move EVERY net was worse (K11..K21
   two open each, more vias: the head-on nets' faces the projection
   chose are what the braid was tuned on), and handing the fanout the
   plan's exit LINES (`LINES=1`, so the model's gap choices would be
   laid) is broken at the fanout (K11: 10 DRC on the fanout board,
   K21: 21) — the gap is still the fanout's choice, which is the
   model's residual uncertainty for head-on exits.

The plan's move menus are array-grid based (`escape_moves.grid_of` measures
rows and columns in board axes), so a rotated array has no plan;
`fanout_plain.sh` fans it out with the production `bga_fanout` (which
handles rotation) and no hints, so the braid can still be exercised on it
(`KICAD_ALLOW_STAGGERED_BGA=1`: its lattice detector also runs in board
axes and reads a 45° grid as staggered).

## Stage 2 — the braid (`braid.py`)

Input: the board with BOTH arrays fanned out, the nets, `--dest REF`.

**Endpoints.** `endpoints(pcb, names, byname, dest_ref)` — each net's two
free stub ends, attributed to a component by WALKING THE NET'S OWN COPPER
to the pad it reaches (nearest-pad is wrong for a stub dragged across a
field; a segment end inside a via barrel is not a free end). `_end_dir`
reads the direction each end ESCAPES in from the stub's own run (its
longest segment, walked from the free end) — the last segment is the wrong
thing (the fanout ends many escapes with a 45° jog) and pad-to-end is too
(the escape runs half a pitch off its pad's row).

**Corridors** (`corridor.cluster_corridors`). Each net's taut path (tooth →
stub end, around the static copper) is computed; two nets link when their
stubs are on the same destination array within 6 mm and their taut paths
arrive from within 60° of each other seen from the array's centre — single
linkage, so a face of stubs chains up whatever its length and a corner is
crossed in two or three links. A group is then checked against the spine
it would get: a member whose stub lies past the spine's end, or whose tooth
lies behind its start, is kept only if its OWN lane — at its own offset —
can run that stretch without crossing an array-scale part's pads (a stub
along the next face can; one on the far face cannot), else it is split off
and regroups. Corridors are laid down largest first. (Rungs between the
stubs themselves were the first test and clip the neighbouring pads of
whatever row a stub exits along; a decoupling cap under the array's corner
also blocked them — the topology test only sees parts of ≥ 10 pads.)

**Spine** (`corridor.build_spine`). Its two end legs follow the FLOW at the
teeth and at the stubs (`flow_dir`: the stub's escape direction when the
taut path keeps to it, else the taut path's direction 1.5 mm in — a flank
tooth points south and flows east). When the two flows are parallel the
spine is ONE straight line through the midpoint (the channel between two
facing arrays, however their centroids are offset — the offset is the
lanes' morph, not a tilt of the frame); otherwise the members' mean path
between the end legs, relaxed as a string (`topo_strings.relax`) against
the big parts (≥ 10 pads, never the corridor's own arrays) inflated by the
bundle's half-width, and the corridors already laid. Tracks, vias and small
parts are the lanes' business (inflating a foreign track 0.9 mm off the K15
chord wiggled the spine round it and reserved the schedule region for the
wiggles' corners). A clear straight channel is not relaxed at all.

**The frame.** Everything from here on is in the spine's own coordinates:
`s` along it, `o` across it (positive to the right of travel). `Spine.project`
maps board points and the router's grid cells to `(s, o)` — at a corner, a
cell on the outer side projects to the corner's `s` at its own radius (the
lanes round the corner on concentric arcs), the inner side to the mitre.
`Spine.lane_xy` maps an `(s, o)` polyline back. No swap column is placed
inside a corner's wedge (`max|o|·tan(turn/2) + 0.3`).

**Launch and exit.** A tooth is BORN IN PLACE unless another member's tooth
sits clearly downstream of it at (nearly) its own offset — its lane would
run into that tooth — or its run to the schedule region hits foreign copper
(teeth at the same `s` are no obstacle to each other however close: their
lanes fan out to the pitch-floored launch slots). Otherwise it JOINS from the
side: the joiners of a side form a BLOCK beyond everything on that side
(pushed further out until its innermost lane's run is clear of the static
copper on both layers — the flank carries every other net's teeth too), the
first joiner farthest, so no join leg crosses a lane already present. "First"
is by the LEG's `s`, not the tooth's: a leg that would run through another
member's free end (two flank teeth in one column) jogs a pitch along the
spine first, and the legs are placed one at a time so a leg already placed
counts as in the way too (two stubs ending at one point on two layers — K21
SRAS on B, SCKE1 on F, the fanout's doing — otherwise sent both legs to the
same jog). Ordering by the teeth gave the outer lane to a leg that had
jogged downstream, straight across the other's lane (K19 SRAS over SWE).

The exits mirror this: a stub receives its lane HEAD-ON (a diagonal tail
from `s1`) when its offset is free of every stub upstream, else the lane
peels off to it from an exit block: ports (head-on launched) inner, in exit
order; joiners outer, in JOIN order — their exit legs cross the lanes still
between them and their stubs BY LAYER. The layer rule is the BLOCK's: all
its legs are on one layer (the stubs' — a leg ending on its stub's layer
costs no via there), and a lane is on the other from the first leg that
crosses it until its own corner, where it turns onto its leg with a via.
The crossed stretch reaches `±0.35` (`LEG_REQ`) past each crossing leg —
a via must sit that far from the leg, because a track needs via radius +
clearance + half a track = 0.29 from a via — and the corner itself is
stamped as a VIRTUAL VIA while its owner is unrouted, so no neighbour hugs
its band edge there (the edge is exactly a via's clearance from the
centreline). The leg's owner carries no rule on its lane: the crossed
lanes' copper is on the other layer under it, real or virtual, so the leg
goes where it can. That is the constant-layer "river" of the earlier takes,
as a rule of the exits rather than a mechanism (sorting the joiners'
reversal inside the shared region cost every column a quarter of its width;
sorting it on their run-in had no room for the vias).

What this replaced, measured on K19's south flank (eight joiners, exit legs
at 0.4 mm pitch): a layer per leg by the majority of the lanes it crossed,
`±0.25`, and a rule on the owner too. Adjacent legs disagreed, and a lane
crossed at `s` exiting at `s + 0.4` was required on B until `s + 0.25` and
on F from `s + 0.15` — closed on both layers, refused before the router saw
it (SCAS, SWE). Where the rules were consistent, the crossed lanes dived
and surfaced at the rule's edge, 0.31 from the leg, and two such vias 0.63
apart walled the leg's layer to 0.02 mm (SODT1). `probe_req.py` lists
both-layers-closed stretches; `probe_map.py` draws the router's obstacle
map with the band for one lane, which is what found each of these.

**Schedule** (`schedule.Schedule`). Launch order and target order are the
lanes' offsets at the two ends of the schedule region `[s0, s1]` (`s0` =
last tooth + 0.3, `s1` = first head-on stub − 0.6 / first side exit − 0.3);
the divers are the complement of the longest increasing subsequence of that
permutation. The columns are a TRANSPOSITION SORT: each column swaps a
maximal set of disjoint adjacent inverted pairs, so every swap is a real
inversion and every pair is swapped once (the earlier wave schedule derived
passes from a serial pass and mis-placed a diver whose launch slot was its
target slot — K32's, and the merged corridor's, "phantom swap"). The mover
is the diver; of two divers the one in flight (a flying diver passed by a
grounded one would have to surface). The take-off GATE: a diver takes off
only when no diver in flight still has to cross it — unless that flyer's
only crossing left IS this diver (the crossing is its landing on either
layer, and it cannot land until this one reaches it: the strict form
deadlocked until the empty-column override, K28 SDQ0 serialised 16
columns behind SDQ15 for nothing) — policy `last`, `Schedule.columns(...,
gate=)`. The gate spends columns to save vias, and the corridor's columns
are finite: `Corridor.plan_columns` drops it (`off`) when the gated
schedule's column width falls under `W_GATE` 0.33 — measured: K4..K19
identical either way at `last`, ungated cost them +2..+6 vias; K21 11
columns / attempt 0 / same 28 vias ungated vs 13 / attempt 1 gated; K28
31 columns and 9/26 strict, 23 and 12/26 `last`, 16 and 20/26 off.
`SCHED_GATE=strict|last|off` pins it; every probe tool goes through
`plan_columns` so it diagnoses the schedule the braid laid. Gaps and
leads (a layer change is a column of its own) are raised by the router's
refusals: a pass within two columns of one of a diver's own moves → a
spacer; nothing passing it before its first swap → a lead. Reserved
`s`-intervals (corner wedges, crossings of earlier corridors) hold no
column; columns are laid in the free length (`u(s)`).

**Layers.** At the crossing of column *k* the mover is REQUIRED on B and the
passed net on F over `0.4 W + 0.05` either side (not the whole column: a net
passed at *k* and moving at *k+2* needs a cell between to put its via in);
the diver windows are derived from the schedule's intervals only (an exit
block's B stretch lies in the tail, which is open to B anyway; fed to the
diver formula it read as a dive opening at the last F pass and closed B on a
B-born tooth — K21 SCAS, stuck after one cell), and a lane born on B may
stay there until it is first passed on F. Exit-leg crossings add their
intervals. Virtual copper of the lanes not yet routed is stamped on the
layers the schedule lets each occupy — per polyline piece SPLIT at every
required-interval boundary (deciding per whole piece put an 8 mm exit run on
both layers through the one millimetre where another lane's leg crosses it);
an exit leg on its block's layer only, a jog on its free end's layer only
(stamped on both, a jog to a B stub walled the F stub another net ends at
the same point).

**Copper — the router's.** Every lane is routed by `connect()` from its tooth
to its stub end inside its BAND — a cell mask in `(s, o)`: between the
neighbouring lanes present on that layer (never narrower than a grid cell),
closed where the schedule requires the other layer; join and exit legs as
rectangles (`±0.5` along, `±0.2` beyond the ends — the legs run through the
densest static copper on the board and their vias need room; the obstacle
map is the law, the band a guide); a loose tube through a crossing of an
earlier corridor. Divers first, then the rest in target order; each result
is copper for the next. Grid 0.025 mm. `LANE_PIECES=1` routes a lane as a
chain of searches between its waypoints instead (measured: no gain).

**Feedback.** A refused lane reruns the schedule: launch pitch first (0.35 →
0.40), then a spacer or a lead for a refused diver. Six attempts; what is
still refused is reported and left open. Nothing is patched by hand.

**Where corridors meet.** Two corridors never overlap along their length
(the later spine is relaxed against the earlier corridor's lane tubes).
Where one must run through another — its exits sit among the other's stubs
— the stretch is reserved (no column) and the later corridor's lanes route
through it on either layer against the earlier corridor's real copper. That
is v1; the global allocation (which corridor yields, corridors pushed
outward to leave the middle for a wide one — what the human does around
DU1's back) is the next thing to build.

**Finish.** `smooth_octolinear_chains` (#536, clearance-validated) over the
BRAID's copper only (`keep_input_copper`): the fanout's stubs are the
braid's input and stay as they came, so the tooth a lane was routed from
stays on copper — the smoother once re-cut a stub's corner into a
diagonal and left the tooth mark, and the join the plan drew, hanging in
free space. Then the board is written with the Eco overlay (below).

## Stage 3 — `connect()` (`connect.py`)

The one join primitive, on the REAL router:

```
connect(pcb, net_id, a, a_layer, b, b_layer, cfg, band=…, virtual=…, window_pts=…)
```

routes the net between the copper island at `a` and the island at `b` with
`route_net_with_obstacles` (the production grid A*) inside a fenced
`make_local_window` (covering `window_pts`, the planned path, when the lane
goes somewhere the two points' box does not), against `build_base_obstacle_map`
plus the same-net via/drill guards the scoped rescue uses. `band` is either
`(lo(x), hi(x))`, a per-layer dict `{layer: fn(x) -> (lo, hi)}`, or a
vectorised callable `band(xs, ys, layer) -> mask` (the general form:
`tube_mask` helps build one); `virtual` is copper that does not exist yet.
Returns (segments, vias) or None; `CONNECT_DEBUG=1` prints the rejection.

`test_connect.py` exercises it on the case that broke every straight-line
rule (cap C5 sitting on both the linear and the 45° descent to a south
stub): free route DRC-clean, a band changes the answer, a foreign wall is
routed around.

## Eco overlay (`render_eco.py`)

`braid.py` writes the PLAN onto the user layers so a render shows plan
against copper:

| layer | colour | meaning |
|---|---|---|
| Eco1.User, thick | pale blue | each corridor's spine |
| Eco1.User | white | every lane's planned centreline (join leg, columns, tail or exit leg) |
| Cmts.User | orange | where the schedule REQUIRES the back layer — the planned under-passes |
| Eco2.User | yellow | connection ends: the fanout's free ends (source teeth, stub ends) as an "×", the braid's own points (join/exit leg ends, jogs, spine corners) as a "+" |

Copper that leaves its white line is the router disagreeing with the plan
(a via it had to place elsewhere, an obstacle it went round); a via outside
an orange stretch is one the schedule did not ask for.

## Constants

All in track/clearance/via units; none is a board fact.

| what | value | why |
|---|---|---|
| track / clearance / via | 0.127 / 0.105 / 0.25 (0.15 drill) | the bench's routed floor; CLEAR carries 5 µm so hugs never sit exactly at 0.1 |
| splice offset | first head-on stub − 0.6 | a 45° jog from 0.38-pitch lanes onto 0.25-pitch stubs |
| lane pitch floor at the exits | 0.38 | a via beside a lane needs 0.2885 + deviation |
| launch pitch floor | 0.35 → 0.40 | same, plus a grid step; raised on refusal |
| block pitch / gap | 0.35 / 0.45 | a join or exit block beyond what it clears |
| required-layer half-width | 0.4 W + 0.05 | the converging part of a column |
| leg layer rule / band rect | ±0.25 / ±0.5, ±0.2 | the crossing itself / the leg's freedom |
| band inset | (track + 0.1) / 2 | two lanes at their band edges still clear |
| corridor reserve | 0.3 | room after the last column |
| gate column floor `W_GATE` | 0.33 | the narrowest column a gated schedule gets; every clean gated K had W ≥ 0.343 |
| big part | ≥ 10 pads | what a spine avoids and a topology rung respects |
| routing grid | 0.025 | see "Copper" |

## Speed (cProfile, K21 braid on `re_fo_k21`, 2026-08-30)

`bash prof_k.sh re_fo_k21.kicad_pcb 21 OUT`. The router's own search
is 0.2 s of a 28 s run; everything else is Python around it:

| where | before | after | what |
|---|---|---|---|
| `setup` → `detect_buses.taut_paths` | 10.5 s | 10.5 s | `topo_strings.relax`, pure-Python geometry, 0.5 s per net (486k `point_violation`, 21M `hypot`); the plan step computes the same paths again |
| `route_lane` → `connect` | 14.2 s (84 calls) | 5.0 s (42) | per call 0.17 → 0.12 s; K21 now routes at attempt 1, not 3 |
| — `band_of` | 7.6 s | 1.0 s | the neighbour search is a function of `s` alone: done on a 1-D sample (2 µm grid + every vertex, every step doubled, every crossing root) and interpolated onto the cells. `band_check.py`: 0 of 16 M cells differ at K21, 0 of 22 M at K28 |
| — `build_base_obstacle_map` | 4.2 s | 2.5 s | rebuilt per connect over a SQUARE window of the lane's extent (a 25 mm joiner lane = 1 M cells) |
| `write_out` → `smooth_octolinear_chains` (#536) | 3.2 s | 7.1 s | `_seg_foreign_seg_dist`, pure Python, grows with the segment count |
| whole run | 28.4 s | 24.3 s | |

At K28/K32 the time is REFUSALS: a refused lane is an exhausted search
(tens of thousands of iterations), six attempts each — K32's braid took
5.5 min for 15 open lanes. Cutting refusals cuts time; the gate policy
did both. Next in line: `taut_paths` (vectorise `point_violation`, or
share the plan step's paths), the square window (a rectangle of the
lane's bbox), and skipping an attempt whose schedule is unchanged.

## Known walls

- **K19/K21 flank joiners — CLOSED 2026-08-30.** The refusals were never
  the join zone's freedom against the earlier lanes' vias, as the earlier
  entry here guessed: `probe_map.py` showed each one. Two were plans closed
  on both layers by adjacent exit-leg rules (SCAS, SWE); one a leg walled
  by the vias of the lanes it crossed, placed at the rule's edge 0.31 mm
  from it (SODT1); one a join leg jogged downstream across the lane it
  should have been outside of (SRAS); then, once those routed, a corner
  whose neighbours hugged their band edge to exactly a via's clearance
  (SCAS again), and at K21 two stubs ending at one point on two layers
  (SRAS/SCKE1) plus a B window that closed a B-born tooth (SCAS). Six
  distinct mechanisms, one lane each; every one a rule of the plan that
  contradicted itself or the router's clearances, none of them the router.
- the free-plan arm (floor objective, `--no-plane-drop`): K15 4 open,
  K19 1, K21 5. Its fanout obeys 10/15, 17/19, 17/21 of the plan's
  directions and the braid's refusals are those balls: the plan assigns
  more "left" escapes to DU1's west face (rows 7–8, two row gaps past
  column A) than the fanout can lay, whatever escape primitive is used
  (Stage 1, item 7). The lever is the plan's side-capacity model, not the
  fanout. **The restricted arm hits the same wall at K32**: 5 of 32 balls
  (SBA1, SA15, SA4, SA6, SA11) had no route out the planned face and
  escaped up/right, became a corridor of their own, and that corridor is
  refused whole against the first one's copper (crossings between
  corridors are v1).
- **K28 SDQ7: a part inside the corridor.** C5 (0402, F.Cu, pads at
  (131.29, 67.55)/(131.29, 68.51)) sits in the bundle's cross-section
  at `s` ≈ 10.5–11.2, `o` ≈ 1.0–2.2. The spine avoids big parts; the
  lanes spread ±6 mm about it and a lane whose band crosses C5 where the
  schedule REQUIRES it on F (SDQ7, passed by SDQ12 and SCAS there) is
  dead — `band_conn.py kh_fo_k28.kicad_pcb 28 SDQ7 --copper`: the flood
  stops at `s` 10.93, `o` 1.10, the band's next cells at `o` 2.0 are
  under C5. Every other lane happens to cross C5's shadow on B or beside
  it. The general form is a per-lane layer rule from static copper
  inside the corridor (an F-only obstacle = B required over its
  `s`-stretch for the lanes whose `o` crosses it), fed to the schedule
  as a column-free reserved stretch; not built.
- **K28 columns.** With the gate off K28 needs 16 columns in 4.2 mm
  (W 0.277, below `W_GATE`); the plan sent two flank JOINERS (SWE, SCAS)
  to DU1's down face, 12- and 10-lane chains that are 22 of its 71
  swaps, because the left face's 14 slots were full. The plan prices
  divers (2 vias each), not the lanes a diver passes; a column-depth
  term, or knowing that the join block puts a joiner outermost, is the
  plan-side lever.
- the 45° bench: 9 of 11 lanes (SDQ14, SDQ15); the side-exit block sits
  3 mm from its stubs (it is placed beyond every head-on lane of the
  schedule region, which have left by then) and the schedule needs 24
  columns at W 0.24. The braid now keeps its BEST attempt, not its last
  (this bench had 9/11 at attempt 1 and wrote 7/11).
- crossings between corridors are v1 (see "Where corridors meet").
- the plan's floor and the fanout's stub order are not rotation-invariant
  (tie-breaks in `select_moves`; escape order in `bga_fanout`); the plan has
  no menus for a rotated array at all.
- the plane-drop pass collides with the decoupling caps under the array.

## History

- **take 2** (`bus622-take2`): the route.py plan-lanes engine (attraction
  corridors, plan-via guard, ribbons; rust 0.21.4 soft via cost). Best there:
  K51 0 broken / clean / 194 vias, human 85. Its README (on that branch)
  records the laws and refuted ideas of that campaign; three default-ON
  engine commits from it (`c3725b31` #666 rescue-short guard,
  `947698d6` verdict-integrity merge, `a693919b` victim-priority restore)
  are general route.py fixes that still owe a corpus screen before main.
- **take 3** (`bus622-take3`): architecture C (triangulated corridor order)
  and the braid+A* hybrid — superseded by `connect()`.
- **take 4** (this branch): the braid restarted from 2df9ddae, the plan
  (both ends, inter-corridor pricing), the chain, `connect()`, the
  router-routed trunk (`46a268b3`: K4..K21 clean at 4/10/18/26/30, rotation
  gate 6/6 — the numbers to beat), then the corridor braid: faces, frames
  and the trunk/river split replaced by corridors from the geometry, a
  spine per corridor, `(s, o)`, one lane machinery.

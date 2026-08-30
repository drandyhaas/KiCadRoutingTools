# #622 topo bus routing — take 4 (`bus622-take4`)

A bus is routed in three stages, each a separate program, each graded on
its own board before the next runs:

```
plan  ──►  fanout the berth  ──►  braid: schedule + router-routed lanes
(fanout_from_plan.py)             (braid.py + connect.py)
```

Bench: `fb_t2q_base.kicad_pcb` — U1 (347-ball 0.65 mm BGA, the source,
already fanned out) to DU1 (96-ball 0.8 mm DDR3, the destination), the DDR
subset of `allwinner_h3_ddr3`. **The board is only a test article.** Nothing
in the routing reads a footprint name, a net name, a coordinate or a face
name: every quantity comes from the board's geometry in a frame derived from
it, and the rotation gate (`chain_rot.sh`) is what says so.

## Run it

```bash
bash chain_k.sh TAG 4 11 15 19 21 -- --no-plane-drop   # the ladder
bash chain_rot.sh 15 19 -- --no-plane-drop             # the rotation gate
bash braid_on.sh TAG_fo_k15.kicad_pcb OUT 15 render     # braid stage alone
bash fanout_k.sh OUT 15 --dirs=left,down --no-lines     # fanout stage alone
python3 test_connect.py ch6_fo_k15.kicad_pcb SDQ0       # connect() unit test
```

K is a checkpoint of the **coherent K-ladder** (`k_ladder_coherent.txt`,
read by `coherent_nets.py`): whole rivers only, so a prefix never splits a
bus. Checkpoints 4 / 11 / 15 / 19 / 21 / 32 / 47 / 51.

Grading is `grade_k.py`: `check_connected.py` scoped to the run's nets,
whole-board `check_drc.py --clearance 0.1 --clearance-margin 0.1`, and a
via/segment census from the parsed board. Never a program's own tally.

## Results (2026-08-29)

| K | vias | open / DRC | plain braid (retired) | human |
|---|---|---|---|---|
| 4 | 4 | 0 / 0 | 4 | 4 |
| 11 | 10 | 0 / 0 | 14 | 14 |
| 15 | 18 | 0 / 0 | 22 | 22 |
| 19 | 26 | 0 / 0 | 30 | 30 |
| 21 | 30 | 0 / 0 | 5 DRC | 38 |

Rotation gate (the same chain on the bench rotated 90/180/270°): all six of
{90, 180, 270} × {K15, K19} open=0 drc=0 (vias 22/26, 18/26, 18/28 — not
equal to the unrotated counts, because the production fanout lays a
different stub order on a rotated footprint and the plan's floor differs
by rotation; the rotated chains are different permutations, not the same
problem rotated). With the plane-drop pass on, every rung carries 5–6 extra
DRC that are that pass's own decoupling-cap collisions, present with no
braid at all (`--no-plane-drop` isolates them).

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
   the emitted copper is measured against them ("plan obeyed: N/N").
   `--dirs=left,down` restricts the plan to the faces the braid can deliver
   to; the names are **flow-frame** names (left = the face toward the
   source, down = the source's flank side) mapped to board directions
   through the same flow angle the braid rotates by.
   `escape_line_hints` (which row gap) exist but are OFF (`--no-lines`):
   measured to end two nets' stubs at one identical point.
5. `fanout_k.sh` / `chain_k.sh` grade the fanout board on its own before
   the braid runs — a berth that ships stub-vs-stub contact is broken
   before the braid starts.

## Stage 2 — the braid (`braid.py`)

Input: the board with BOTH arrays fanned out, the nets, `--dest REF`.

**Flow frame.** `flow_frame.flow_angle` snaps the source→destination
direction to a cardinal and `rotate_pcb` rigidly rotates the board so the
source is due WEST; everything below is written in that frame and the
copper is rotated back at write time. (`KICAD_FLOW_FRAME=0` is the negative
control.)

**Endpoints.** `endpoints(pcb, names, byname, dest_ref)` — each net's two
free stub ends, attributed to a component by WALKING THE NET'S OWN COPPER
to the pad it reaches (nearest-pad is wrong for a stub dragged across a
field; a segment end inside a via barrel is not a free end).

**Corridors.** A tooth south of the source's own pad field is on its
flank and rides the **flank corridor** (the "river"); the rest ride the
**trunk**. The trunk runs from the launch line `x0` (max tooth x + 0.3) to
the splice line `x1` (min stub x − 0.6, room for a 45° jog onto stubs the
fanout packs at 0.25 pitch). A trunk net whose stub exits the south face
is a **port** net: it rides the trunk's bottom lanes and leaves after its
last swap.

**Order.** Launch order = tooth y; target order = stub-end y (port lanes
below everything, nearest stub uppermost); **divers** = the complement of
the longest increasing subsequence of that permutation. A **serial pass**
fixes who passes whom and in which direction: each diver, in priority
order (up-movers by rank, then down-movers), takes the slot after the last
PLACED element of smaller rank; a diver not yet moved is no anchor. The
**wave schedule** then turns the inversions into columns of concurrent
adjacent swaps, gated so every diver–diver crossing has exactly one side on
B, with per-diver **gaps** (spacer columns after being passed) and
**leads** (columns before a first swap) — both raised by the router's
refusals. The layer-change gap is enforced in both directions: after being
passed before moving, and after moving before being passed.

**Geometry.** Launch slots = tooth ys at a pitch floor (0.35, raised to 0.40
on refusal); splice lanes = stub ys at a 0.38 floor (ports pinned); the
morph between them in the per-column order the schedule gives. That is a
**centreline per net** — a corridor, not copper.

**Layers.** At the crossing of column *s* (between the midpoints of *s* and
*s+1*), the mover is REQUIRED on B and the passed net on F, over
`0.4 W + 0.05` either side of the crossing — the part of the column where
the two lanes are too close for one layer, and NOT the whole column: a net
passed at *s* and moving at *s+2* needs a cell between the two to put its
via in. A diver may be on B only between the foreign passes that bracket
its own crossings (derived from the same intervals, so the two can never
disagree); a non-diver only in the tail; one born on B until first passed.

**Copper — the router's.** Every lane is routed by `connect()` from its
tooth to its stub end inside its corridor: per-layer BANDS (between the
neighbouring centrelines present on that layer, never narrower than a
grid cell) close the layer the schedule forbids, and the lanes not yet
routed are stamped as VIRTUAL copper on the layers they may occupy. So the
router places the dive and surface vias where they actually fit and never
where a later lane must pass. Divers are routed first, then the rest in
target order; each result is copper for the next. Grid 0.025 mm: the
fanout's 0.25 stub pitch is the legal minimum plus 23 µm, and on a 0.05
grid the cell nearest a centreline can sit 25 µm off it.

**Feedback.** A refused lane reruns the schedule: launch pitch first, then
a lead column (a diver swapping in column 0 has no room for its dive via)
or a spacer (after a pass). Six attempts; what is still refused is reported
and left open. Nothing is patched by hand.

**The flank corridor.** Nested B.Cu runs under everything (deepest = the
west-most descent, so descents never cross a run), run pitch 0.35 on the
grid, each net's run from its descent x to under its stub; both ends —
tooth → run start, run end → stub end — are `connect()` calls.

**Finish.** `smooth_octolinear_chains` (#536, clearance-validated), then the
board is written with the Eco overlay (below).

## Stage 3 — `connect()` (`connect.py`)

The one join primitive, on the REAL router:

```
connect(pcb, net_id, a, a_layer, b, b_layer, cfg, band=…, virtual=…)
```

routes the net between the copper island at `a` and the island at `b` with
`route_net_with_obstacles` (the production grid A*) inside a fenced
`make_local_window`, against `build_base_obstacle_map` plus the same-net
via/drill guards the scoped rescue uses — the production obstacle model:
exact pad shapes, per-net clearances, hole-to-hole. A layer mismatch is
solved by the search placing the via. `band` is either (lo(x), hi(x)) or a
per-layer dict `{layer: fn(x) -> (lo, hi)}` where lo > hi CLOSES that layer
at that x; `virtual` is copper that does not exist yet, stamped as a
foreign net. Returns (segments, vias) or None — a refusal is a refusal.

`test_connect.py` exercises it on the case that broke every straight-line
rule (cap C5 sitting on both the linear and the 45° descent to a south
stub): free route DRC-clean, a band changes the answer, a foreign wall is
routed around.

## Eco overlay (`render_eco.py`)

`braid.py` writes the PLAN onto the user layers so a render shows plan
against copper:

| layer | colour | meaning |
|---|---|---|
| Eco1.User | white | every lane's planned centreline; the flank runs |
| Cmts.User | orange | where the schedule REQUIRES the back layer — the planned under-passes |
| Eco2.User | yellow crosses | connection ends: source teeth, stub ends, port leave points, run ends |

Copper that leaves its white line is the router disagreeing with the plan
(a via it had to place elsewhere, an obstacle it went round); a via outside
an orange stretch is one the schedule did not ask for.

## Constants

All in track/clearance/via units; none is a board fact.

| what | value | why |
|---|---|---|
| track / clearance / via | 0.127 / 0.105 / 0.25 (0.15 drill) | the bench's routed floor; CLEAR carries 5 µm so hugs never sit exactly at 0.1 |
| splice offset | stub x − 0.6 | a 45° jog from 0.38-pitch lanes onto 0.25-pitch stubs |
| lane pitch floor at the splice | 0.38 | a via beside a lane needs 0.2885 + deviation |
| launch pitch floor | 0.35 → 0.40 | same, plus a grid step; raised on refusal |
| required-layer half-width | 0.4 W + 0.05 | the converging part of a column |
| band inset | (track + 0.1) / 2 | two lanes at their band edges still clear |
| trunk reserve | 0.3 | room after the last column |
| flank run pitch | 0.35, base on the grid | every run end is a representable via site |
| routing grid | 0.025 | see "Copper" |

## Declared capability limits (not board facts)

- the trunk delivers to the destination face that faces the source, and
  the flank corridor is the SOUTH one in the flow frame — the plan is told
  so (`--dirs=left,down`). The corridor-per-face-pair refactor (a trunk
  instance per (source face, destination face) pair, each in its own flow
  frame, the river as the L-shaped instance) lifts both;
- source teeth and destination stubs must both exist (both arrays fanned
  out): the braid never enters a ball field.

## Known walls

- **K32**: the serial pass asserts `phantom swap SCKE1/SDQ7` — a mover has
  to pass a diver it is not inverted with because that diver has not moved
  out of the way yet; who-passes-whom needs to become order-aware. The K32
  fanout board also carries an SA12-stub-vs-R1.2 graze from the fanout.
- the plan's floor and the fanout's stub order are not rotation-invariant
  (tie-breaks in `select_moves`; escape order in `bga_fanout`).
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
  (both ends, inter-corridor pricing), the chain, `connect()`, and the
  router-routed trunk.

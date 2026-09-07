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
board as `<board>.plan.json` with the ACHIEVED stub ends, and the braid
reads it (`setup` finds it) and builds its corridors from the identical
inputs. At every K the braid's orders and pages are the planner's.

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

# awx -- the K-bus chain (#622): one plan, a fanout that follows it, a braid

The tool set for routing a fanned-out bus between two BGAs (the
`fb_t2q_fresh` bench: an FPGA `U1` and a DDR3 `DU1`, the coherent
K-ladder), where the PLAN decides both ends of every net, the FANOUT lays
exactly the plan's moves (and tells the plan what it could not), and the
braid routes the lanes:

    bash chain_k.sh TAG 4 8 15 28   # -> tmp/TAG_k<K>.kicad_pcb, graded

Results on the bench (2026-09-06), against the previous chain on the same
engine (a face-hinted `auto` fanout, the bench's own source teeth):

| K  | previous vias | this chain | plan's own prediction | fanout vs plan |
|----|---------------|------------|-----------------------|----------------|
| 4  | 4             | 4          | 4                     | 100 %, per net |
| 8  | 8             | 10         | 10                    | 100 %          |
| 15 | 22            | 20         | 20                    | 100 %          |
| 28 | 38, 0 open    | 45, 2 open | 40                    | 100 %          |

All complete and DRC-clean at the routed 0.1 mm floor except K28 (SA9,
SDQ9 refused by the braid). The plan's total prediction equals the
braid's result at K4/K8/K15; K28 is the open frontier (see the end).
Every board the chain writes -- each realized source board, the fanout
board, the braided board -- is DRC-gated at 0.1 with the quantization
margin, and the fanout boards are clean with the margin off as well.

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

## What is next: one planner

The plan decides each net's lane layer (keeper: its tooth layer; diver:
the other) and the braid's schedule decides it again from its own orders.
The two agree in total at K4/K8/K15 and disagree net by net in
tie-breaks; at K28 the plan predicts 40 and the braid pays 45, because the
plan prices every non-keeper as a two-via diver while the braid's B page
must itself be crossing-free, so some divers swim. The plan should compute
pages with the schedule's own code, price that, and hand the pages to the
braid. Two K28 specifics: a far-face berth (SA13, DU1's east face) is
predicted free and costs a corridor of its own, and two lanes (SA9, SDQ9)
are refused by the braid's virtual-copper walling.

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

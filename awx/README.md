# awx -- the K28 bus chain (#622)

The minimal tool set that reproduces the current best practice for
routing a fanned-out bus between two BGAs (the `fb_t2q_fresh` bench:
an FPGA `U1` and a DDR3 `DU1`, 28 nets of the coherent K-ladder):

    bash chain_k.sh TAG 28          # -> tmp/TAG_k28.kicad_pcb, graded

Reference result on the bench (2026-09-06): 28/28 connected, 0 DRC at
the routed 0.1 mm floor, 38 vias, 1440 segments, ~52 s end to end.
The chain is deterministic: `cmp_copper.py A.kicad_pcb B.kicad_pcb`
reports IDENTICAL copper between two runs (UUIDs differ, copper does
not), which is the regression test for any change here. Every commit
on this branch was gated on that identity, for the fanout board and
the braided board both.

## The chain

1. `coherent_nets.py K` -- the first K routable nets of the coherent
   ladder (`k_ladder_coherent.txt`: whole rivers, tightest first; a
   prefix never splits a river).
2. `fanout_from_plan.py OUT.kicad_pcb K --board=BASE` -- the PLAN
   and the destination fanout. `plan_ends.py` picks one escape per
   ball at the destination from menus of legal moves
   (`escape_moves.py`), judged by the crossing floor of the lane order
   the braid will see (`select_moves.py`: bus sides certified by
   capacity, greedy selection, LIS refinement, layer alignment;
   `taut_clean.py` and `detect_buses.py` for the taut-path buses,
   average-linkage clustering), with the source refinement run for its
   effect on the launch points (its moves are never applied). The
   chosen directions go to the production engine
   (`py_router/bga_fanout`, `escape_dir_hints`) which lays the copper;
   the copper that actually leaves each ball is measured against the
   plan (`obeyed`). A fanout that is not DRC-clean and complete stops
   the chain.
3. `braid.py --board FO.kicad_pcb --dest DU1 --nets ... --out STEM` --
   corridors from the geometry (`corridor.py`: nets whose stubs one
   spine can reach), a straight spine per corridor, launch and target
   orders from the lanes' offsets, the two-page schedule
   (`schedule.py`: the longest in-order subsequence on the front
   layer, the worst crossers of the rest on the back layer, the
   remainder swimmers), and every lane routed by the real router inside
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

`generate_bga_fanout(..., escape_dir_hints=None)`: a per-pad planned
escape side, keyed by board-frame pad position. Re-keyed into the
footprint frame for a rotated part, threaded through the
escape-priority passes and the auto-retry ladder, taken first by the
channel engine (`preferred_dir`; the target-side preference fills the
rest when it is on) and by the under-pad engine (a side-constrained
A* -- per-side heuristic, exit on that side only -- tried before the
unconstrained one, misses reported). With no hints every path is
unchanged. The under-pad engine is what lays the whole K28 destination
fanout (it wins the auto retry after the channel engine drops three
balls); without the side-aware A* the plan is obeyed by 13 of 28 balls
instead of 24 and the braid ships 1 open at 64 vias.

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

### Only in the `bus622-take3` worktree, uncommitted (2026-09-06)

`awx/dogbone_berths.py` -- dog-bone berths at the ball for a named set
of nets: a via in the diagonal gap, a 45-degree stub, and either a
short back-layer leg or a RIDE routed by `connect()` to the face or the
source-facing edge, the side chosen by a two-subsequence cover of the
launch order. Its braid-side companions there (rivers keyed on the
axis flank or the stub position, `_end_dir` stopping at a same-net via,
a B-berth river coloured B) are working-tree edits to `braid.py`. Both
measured and lost at K41 (the braid's schedule still sees one ribbon
order), and neither is committed anywhere.

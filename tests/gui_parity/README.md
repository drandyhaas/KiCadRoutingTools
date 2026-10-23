# GUI/CLI engine-parity harness

Measures whether the claude tab's "run selected steps" (GUI engine calls on
the live pcbnew board) produces the same final board as the recorded stress
chain's CLI steps run file-to-file.

- `replay_plan_vs_run.py` — **start here.** Loads a stress run's
  `<board>_plan.json` into a genuine headless `RoutingDialog` and runs it with
  the genuine `PlanExecutor`, then diffs against the manifest replayed at HEAD.
  Nothing is mirrored or mocked. See "Whole-plan replay" below.
- `test_gui_engine_parity.py`, `test_gui_livechain_rp2350.py` — **MIGRATED onto
  the real dialog (2026-07-26)**: the two gates that run a whole plan on an
  in-repo board (`splitflap_driver`, `rp2350_fpga_eensy_prePlane`). Each
  expresses its GUI leg as a Claude-tab **plan** and runs it through
  `replay_plan_vs_run.replay()` — the same real `RoutingDialog` +
  `PlanExecutor`. `replay()` touches its `info` argument only for
  `input_board`, so the corpus driver works unchanged on a checked-in board;
  that is the whole trick. Nothing is mirrored or shimmed any more.
- The other gates each pin one class of GUI/CLI divergence, most of them one
  that was found and fixed; the index below lists them all.

## Running the gates

    python3 tests/gui_parity/<gate>.py

Plain `python3` is right: a gate that needs pcbnew or wx re-execs itself into
KiCad's bundled python (`KICAD_PYTHON` names one explicitly), and the dialog is
built headless (`parent=None`, never shown), so no window appears. Without a
KiCad python a gate either prints `SKIP:` or refuses with exit 2 (`NOT RUN`);
neither is a pass. `tests/run_all.py` never collects this directory; many
gates have a wx-free half under `tests/` that it does. If a gate hangs at ~0%
CPU on macOS, it is the invisible restore-windows alert after a killed wx
process: `defaults write -g ApplePersistenceIgnoreState -bool YES`.

## Gate index

Whole plans through the real dialog:
- `replay_plan_vs_run.py` — a stress run's plan against its CLI chain replayed
  at HEAD (needs the stress corpus; see "Whole-plan replay" below).
- `test_gui_engine_parity.py` — splitflap_driver's signal + plane plan against
  the CLI chain; it REPORTS divergence rather than passing or failing.
- `test_gui_livechain_rp2350.py` — rp2350's pour + route chain on one live
  board: no stage introduces DRC.

Plan conversion and delivery:
- `test_manifest_plan_parity.py` (no wx) — every routing flag survives
  `manifest_to_plan` into its plan step.
- `test_plan_flags_reach_dialog.py` — those flags reach the real controls and
  engines, and the next step is back at the CLI default.
- `test_772_cap_params_reach_engine.py` — an `optimize_caps` step's params
  reach the cap engine.
- `test_group_scope_parity.py` — a `route.py --group BLOCK` step routes the
  CLI's net list (#459).
- `test_860_qfn_escape_method_panel.py` — a QFN step's `escape_method` reaches
  the QFN panel, not the BGA one.
- `test_geometry_floor_leak.py` — a geometry-floor override does not leak into
  the next step.
- `test_fix_drc_preference_plan.py` — a plan keeps the user's "Fix DRC
  settings" choice (#693).
- `test_plan_refused_step_stops.py` — a step that produces nothing stops the
  plan.

Engine parameters and defaults:
- `test_engine_kwarg_parity.py` — every engine kwarg a CLI main passes reaches
  the GUI call (Class 1 by AST; Class 2, passed-but-inert, on the live dialog).
- `test_cli_postpass_coverage.py` (no wx) — every CLI `main()` post-engine pass
  has a GUI counterpart.
- `test_cli_defaults_parity.py` (no wx) — no argparse default disagrees with
  the `routing_defaults` constant it shadows.
- `test_live_drc_floors_gated.py` (no wx) — every DRC-floor writer a GUI run
  reaches is gated on "Fix DRC settings" (#693).
- `test_966_unset_clearance.py` — clearance resolution in every tab's shared
  config (#966).
- `test_zero_diff_pair_width_gui.py` — a 0 class diff-pair width/gap resolves
  the way `route_diff.py` resolves it.
- `test_settings_roundtrip.py` — the dialog's settings save and restore.

Planes tab: `test_plane_all_layers_parity.py`, `test_plane_layer_costs_gui.py`,
`test_gnd_vias_gui.py`, `test_skip_existing_zones_gui.py`,
`test_1056_renamed_layers_gui.py` (all described below).

Fanout tab and cap placement:
- `test_fanout_rotated_gui.py` — QFN fanout on a rotated footprint: the only
  routing consumer of the pad locals the GUI computes.
- `test_fanout_backside_gui.py` — BGA fanout on a part flipped to B.Cu.
- `test_fanout_threaded_gui.py` — the escape runs on a worker thread and still
  applies (#621).
- `test_581_fanout_via_in_pad_gui.py`, `test_1195_qfn_floors_gui.py`,
  `test_fanout_floor_parity_gui.py` — described below.
- `test_666_cap_move_gui.py` — a scoped cap move is applied, re-welded and
  kept the same way by the CLI and the Route tab.
- `test_733_cap_edge_clearance_gui.py`, `test_733_cap_edge_control_real_dialog.py`
  — the cap pass's edge margin has its own control, which the signal edge
  control cannot move.
- `test_746_cap_summary_gui.py` — the cap-optimization summary names all three
  graze channels.
- `test_768_cap_ceiling_real_dialog.py` — the netclass ceiling the cap pass
  receives on both GUI call paths.
- `test_782_fanout_netclass_clamp.py` — the fanout tab lowers the non-Default
  classes it priced at that ceiling.
- `test_1067_cap_intent_gui.py`, `test_1122_cap_rotation_gui.py` — the cap pass
  under an intent's decap gate, and its declared rotations, land where the
  CLI's `--intent` run lands.

Live-board writeback and records: `test_900_live_class_clearance.py`,
`test_live_fab_floor_origin.py`, `test_records_survive_save.py`,
`test_1187_live_web_floor.py`, `test_live_hole_clearance.py`,
`test_1133_staged_save_widths.py` (all described below), and
`test_ui_thread_save.py` — `save_board_on_ui_thread` marshals a save from a
worker thread, and gives up instead of blocking when the UI thread is not
pumping (#688).

The live model's re-sync between steps: `test_footprint_position_sync.py` and
`test_1279_sense_branch_live.py` (below), `test_726_gui_sync.py` (both blocks
of a duplicated reference), `test_962_paste_resync.py` (paste openings move
with their parts).

Text parser vs pcbnew (both parse paths, with pcbnew as the referee where it
can be): `test_726_parse_path_parity.py` (duplicated references),
`test_ref_label_pcbnew_parity.py` (reference labels, #481),
`test_829_edge_cuts_owner_parity.py` (who owns the outline),
`test_751_via_protection_readback.py` (via protection on a SWIG without the
enums), `test_via_type_parse_parity.py` (via spans, #1268),
`test_962_paste_parity.py` and `test_962_native_graphic_overrun.py` (paste
openings and graphic copper), `test_967_pad_geometry_coverage.py`,
`test_pad_loader_parity.py` (pcbnew's load-time rewrites),
`test_1164_teardrop_parity.py`, `test_1211_sheet_name_parity.py`,
`test_1248_die_length_parity.py`, `test_edge_poly_outline_parity.py`,
`test_noncopper_zone_parity.py`, `test_zone_arc_outline_parity.py`,
`test_zone_closing_vertex_parity.py`, and `test_714_mirror_pcbnew_parity.py`
(the placement writer's layer flip against pcbnew's own `Flip`; the flip's
acceptance gate, which exits 2 rather than skipping without pcbnew).

Placement tab: `test_placement_apply_locks.py` (below),
`test_placement_apply_graphics.py` (Apply lays tracks, never the result's
copper graphics), `test_placement_fake_run.py` (the whole runner against a
fake claude CLI, `fixtures/fake_claude.py`).

Tools, not gates: `diff_engine_kwargs.py` (below), `probe_effective_floors.py`
(the one-off probe behind the splitflap track-width measurement below), and
`wx_pump.py` (waits on a tab's worker with a real wx MainLoop: Yield-pumping
never fires wx timers on Windows).

## Why the shims went (kept for the reasoning)

The shim-based harnesses predated the discovery that the REAL dialog
constructs headless (`parent=None`, never shown, with
`WXSUPPRESS_SIZER_FLAGS_CHECK=1`). They bound real tab methods onto plain shim
objects and hand-built the config the engine is called with, which has a
structural blind spot: **anything between a dialog CONTROL and the engine
argument is never executed.**

That is not hypothetical. #493 item 3 was `swig_gui` reading the board's net
class with `nm * 1e-6`, landing one ULP low (`0.2` → `0.19999999999999998`) and
handing that to every engine as `clearance`. It reaches the engine through
`_effective_geometry_floor` → `_get_netclass_parameters`; the shim harness called
`batch_route(clearance=step['clearance'], ...)` with its own literal, so it
could never have seen it. `replay_plan_vs_run.py` caught it on its first run.

The shims also rot. `test_plane_all_layers_parity.py` used to die with
`AttributeError: 'Shim' object has no attribute '_cancel_requested'` — the tab
grew a field the shim never modelled, so that gate failed on its own
scaffolding, not on the behaviour it guards (pre-existing at 75acd26, before
the #493 work). It now drives the real PlanesTab. A harness that mirrors an
interface has to be maintained in lockstep with it; one that instantiates the
real thing does not.

**And a shim does not only MISS bugs — it can INVENT them.** Because
`test_gui_engine_parity`'s shim hand-built the plane step's config, it never ran
`_effective_track_width()`, so the engine's own
`config.get('track_width', defaults.TRACK_WIDTH)` fallback supplied 0.3 where
the real dialog resolves the board's Default net class to 0.127. The gate
reported 73 "divergent" GND plane-tap segments on splitflap, they were written
up here as a real unfixed defect, and they do not exist in the real GUI (see the
splitflap measurement below). That is what finally motivated the migration.

Their findings are still recorded below. Both gates now use the real-dialog
path; use it for new work too.

Workdir: `tests/gui_parity/work/` (gitignored).

## Comparison bars

1. **Grade parity** (the harness VERDICT): fully-connected, DRC-clean, and
   kicad-cli-refilled unconnected counts equal. This is the acceptance bar
   the stress tests themselves use.
2. **Copper-set identity** (the `COPPER-SET COMPARISON` block): segments and
   vias as canonical UUID-independent sets. Byte equality is meaningless by
   design (.kicad_pcb outputs carry per-run random UUIDs -- see project
   notes: never whole-file-diff routing outputs).

> **Superseded (2026-07-25, #493).** The two sections after this one
> ("Command/input identity findings" and the overlap ladder in
> "Parameter-identity verification") were measured with the shim harness
> before the real-dialog driver existed, and their central conclusion — that
> bit-identical GUI/CLI copper would need engine-level order canonicalization
> — is **wrong**. The residual was not
> item ordering: it was the GUI quantising applied copper twice (1 um position
> rounding plus `pcbnew.FromMM` truncating) and reading its net class one ULP
> low. With those fixed, nano_eeprom_prog replays **byte-for-byte identical** at
> 1 nm through `replay_plan_vs_run.py`. Kept for the history of how the forks
> were found; do not quote the numbers as current.

**Current measurement on splitflap (2026-07-26, after 78f1731, through the REAL
dialog): COPPER SETS IDENTICAL.**

    CLI        conn_full=True conn_issues=0 drc=0 kicad_unconnected=0
    GUI        conn_full=True conn_issues=0 drc=0 kicad_unconnected=0
    segments:  CLI=1305 GUI=1305 common=1305 cli-only=0 gui-only=0
    vias:      CLI=142  GUI=142  common=142  cli-only=0 gui-only=0
    copper sets identical: True

That "canonical PCBData representation shared by both fronts" the older note
below called an open follow-up is **done**. It was not container ordering and
not a general float problem: pad globals are the ONLY PCBData value that is
COMPUTED rather than read verbatim from the file, pcbnew composed them in
integer nm while the text parser composed them in float mm, and 64 of 1699
pads landed ~0.3 nm apart -- enough to flip A* tie-breaks. `local_to_global`
now snaps to the integer-nm grid. This board went from ~300/1200 segments
common to 1305/1305.

The last 73 segments came off when the gate itself was migrated to the real
dialog. They were a HARNESS ARTIFACT: 73 GND plane-tap segments with identical
endpoints but different WIDTH (CLI 0.127, GUI 0.3) at step 2 (`route_planes`),
the only step where neither side passes an explicit track width.

The CLI chain is file-to-file: step 1 (`route.py --track-width 0.127`) writes a
sibling `cli_step1.kicad_pro` carrying Default `track_width: 0.127`, and step 2
resolves its omitted `--track-width` from that Default net class.

The REAL GUI does the same thing. Measured on `work/cli_step1.kicad_pcb` with a
genuine headless `RoutingDialog`:

    board .kicad_pro Default track_width : 0.127
    defaults.TRACK_WIDTH (shim fallback) : 0.3
    track_width override checkbox        : False
    raw control value                    : 0.3
    _effective_track_width()             : 0.127   <-- board-derived
    planes tab shared track_width        : 0.127   <-- what create_plane gets

`_effective_geometry_floor` reads the board's Default net class whenever the
override checkbox is unchecked (#439), and `planes_tab.get_shared_params()`
passes that through. So the real GUI taps at 0.127, like the CLI.

**The 0.3 was the shim.** The shim-era `test_gui_engine_parity.py` hand-built
its step config (`PLANES = dict(nets=['GND'], layers=['B.Cu'], clearance=0.15,
...)` -- no track_width) and called the engine directly, so
`_effective_track_width()` never ran and the engine's own
`config.get('track_width', defaults.TRACK_WIDTH)` fallback supplied 0.3. This
is exactly the structural blind spot described above: **anything between a
dialog CONTROL and the engine argument is never executed.** The 73-segment
delta was harness noise, not a defect, and it went when the gate moved onto
the real dialog.

## Command/input identity findings (2026-07-08)

- Harness bug found by asking 'were the commands the same?': the GUI leg
  hardcoded ordering_strategy='inside_out' while the real GUI default (and
  CLI default) is 'mps'. Fixed; the two legs now run the same effective
  parameters.
- With commands matched: 381/~1200 segments common. With the shim harness's
  GUI_PARITY_INPUT=parser diagnostic (since removed; the GUI leg text-parsed a
  temp-saved board, isolating the builder-vs-parser representation):
  444/~1200.
- Conclusion: the dominant copper fork is NOT parameters and NOT coordinate
  representation -- it is item ORDERING (pcbnew re-save normalizes element
  order; MPS ordering, spatial-index insertion and A* tie-breaks all follow
  it) plus the CLI chain's per-step .kicad_pro floor carryover. Grade
  parity is unaffected. Bit-identical copper would require engine-level
  order canonicalization (sort nets/pads/segments deterministically before
  routing) -- a follow-up decision, not a bug.

## Parameter-identity verification (KICAD_DUMP_BATCH_KWARGS)

route.py's batch_route dumps its FULL parameter set (every keyword of the
call) and returns when KICAD_DUMP_BATCH_KWARGS=<file> is set -- diffing a CLI
invocation against the GUI leg's call verifies the two fronts hand the engine
the same parameters. At the time of the ladder below, only `return_results`
(definitional) and `layer_costs` [] vs None (proven equivalent via
get_layer_costs) differed.

The probe found two phantom forks in this harness itself (both the exact
CLAUDE.md 'defaults must match in both places' class): ordering_strategy
'inside_out' vs the real mps default, and track_proximity_cost 0.2 vs the
real 0.0. Copper-overlap ladder on splitflap as each fork was removed:

    300/1200 segments common   (initial)
    381/1200                   (+ ordering matched)
    914/1200, 135/165 vias     (+ track_proximity_cost matched = all params)
    1168/1190, 164/165 vias    (+ GUI_PARITY_INPUT=parser: same text-parsed
                                representation -- 98.2% identical copper)

Residual ~2%: pcbnew re-save normalization (element order/precision of the
temp file vs the original bytes), per-step .kicad_pro floor carryover, and
in-memory vs file-based post-pass sequencing. The production GUI path
(builder representation) then sat at ~77% copper identity with full grade
parity. That gap has since closed without any order canonicalization (the
splitflap measurement above).

### Plane engines + diff_engine_kwargs.py (the #362 sweep)

The dump now also covers the PLANE engines: `create_plane` (route_planes.py)
and the repair `route_planes` (repair_planes.py) each write a line
via `route._dump_engine_config` in CONTINUE mode, INCLUDING `all_layers` /
`plane_layers` (layer content/order is a live divergence class), and so does
`generate_bga_fanout`. So one `KICAD_DUMP_BATCH_KWARGS_CONTINUE=1` run of a
whole plan captures every engine call -- route, diff, BGA fanout,
create_plane, repair -- in one file.

`diff_engine_kwargs.py <cli.jsonl> <gui.jsonl>` reports the non-benign per-engine
divergences (see its docstring for how to produce the two captures and which
keys are benign-by-design). Plane engines pair 1:1 and are authoritative; route
calls pair by index and are unreliable when the two fronts made a different
number of rip-up/reconnect calls -- read the plane rows.

TWO HARD LESSONS from the #362 rp2350 plane sweep (both baked into the tool's docs):
1. Generate the CLI reference chain FRESH AT HEAD. The recorded stress boards were
   routed at an older commit; diffing GUI-at-HEAD against them manufactured phantom
   divergences (+127 segs, +8 unconnected that vanished against a HEAD reference).
2. The dump reflects the REAL GUI; an ad-hoc shim that omits a control's value
   silently uses the ENGINE default and hides the divergence. The plane_subchain
   shim omitted same_net_pad_clearance -> fell back to the engine -1.0 -> hid the
   67-vs-43 create divergence the real GUI control (0.25) caused.

The sweep this tool drove found + closed EIGHT GUI/CLI divergences (see
.gui-parity-checked at the repo root for the full list): same_net_pad_clearance
0.25 vs -1.0 (the big one -- blocked plane stitches, drove a +430-segment repair
overshoot), board_edge_clearance 0.0 vs PLANE_EDGE_CLEARANCE, min_track_width
conflated with track_width, all_layers all-6 vs outer+pour, and no_bga_zone +
max_iterations leaking from the route tab's shared controls into the plane step
(root cause: the plan executor reset params only once at load, not before each
step). Post-fix: every plane call MATCHes, GUI board 0 DRC / plane-copper delta
+13 (was +430).

## Converter parity (test_manifest_plan_parity.py)

The whole-plan harnesses run whatever plan they are handed, so they cannot see
a flag that `manifest_to_plan` (converter) dropped before the plan existed,
and they exercise `ai_plan.apply_step_params` (apply) only for the flags their
plans happen to carry. Those two translation layers are where the set11
rp2350_fpga_eensy GUI replay silently diverged from its CLI board (242 DRC
violations vs 0; issue #361).

`test_manifest_plan_parity.py` is the CONVERTER-half gate: no wx, no pcbnew.
It reuses manifest_to_plan's own pruning to pair each kept CLI command 1:1
with its emitted plan step, then asserts each routing-affecting flag survived
into the step's params/assignments using an INDEPENDENT expectation table (so
a converter that drops a flag fails even though it agrees with itself).

    python3 tests/gui_parity/test_manifest_plan_parity.py            # whole corpus
    python3 tests/gui_parity/test_manifest_plan_parity.py <manifest> # one board

Current: 0 mismatches over ~6200 flag-checks / 157 corpus manifests. Catches
all three converter-side gaps from the set11 regression (--no-bga-zones drop,
diff pairs emitted as net names, layerless repair_planes). The apply-side
gaps (escape_method value->index, no_gnd_vias inversion) are ai_plan.py's
job, and the wx gates own them (`test_plan_flags_reach_dialog.py` below).

Those checks only see a flag that a manifest uses AND a table here names, so
`route.py --bus` replayed with bus mode OFF and nothing noticed. The gate's
`check_flag_coverage` therefore enumerates EVERY flag the argparse of each
`FLAG_COVERAGE` tool accepts -- route.py, route_diff.py, route_planes.py,
bga_fanout.py and qfn_fanout.py -- from `krt_capabilities.script_flags`, and
requires each one
to reach a control that tool's plan action can set, or to be listed for that tool as
CLI-only or as a known gap with the reason. It also fails a reached control
that `reset_params_to_defaults` does not restore, since a plan step that sets
it would leak it into every later step. Its run_all half, with the negative
controls, is `tests/test_route_flag_plan_coverage.py`. The widget classes it
counts as settable are READ from `ai_plan._set_control`'s own `isinstance`
branches, so a control the executor cannot set (a RadioBox, before it had a
branch) cannot pass as reached.

Two converter rules it also holds. FILE flags (`--net-clearances`,
`--fab-overrides`, its `PATH_FLAGS`) must reach the plan as the absolute path
the command opened them at, since the GUI would resolve a relative one against
KiCad's own cwd. And a recorded command its tool REFUSES ran nothing on the
CLI, so it must not become a plan step: `tests/stress/cli_argv_check.py` asks
each tool's real parser (captured in a child process), then the post-parse
checks its main makes on the arguments and on files they name;
`manifest_to_plan.prune_manifest` prunes as if such a command were absent, and
`check_rejected_steps` holds that on four synthetic chains (two argparse
rejections, a route_planes count mismatch, a scope-less --force-reroute) and
on the checked-in fixture itself, whose every line must be one the real CLIs
accept or its flag checks would silently drop out of the count.
`check_refusal_inventory` finds every post-parse refusal in the converted
tools' mains by AST and requires each to be classified in
`cli_argv_check.REFUSAL_SITES` (judged from the arguments, or out of scope
because it reads the board), and `check_post_parse_witnesses` makes each
argument check refuse a witness command and accept its near miss.

What an AST cannot see is the run itself, which is
`test_plan_flags_reach_dialog.py` (KiCad python, seconds): for each flag a
closed gap made plan-reachable, a manifest line carrying it and then the same
step without it go through `manifest_to_plan`'s CLI and the REAL PlanExecutor
on the headless dialog, each action replaced by a probe. The flagged step's
control AND the tab config its engine reads must hold the recorded value; the
next step must be back at the CLI default. It also drives the QFN tab's real
fanout run (engine spied) for `--layer` and the Route, Differential and
Planes workers for `--net-clearances`, so the value is shown reaching the
engine call.

The same blindness applies one level up, to whole TOOLS: an unknown tool only
bumps the converter's `skipped` count, so a converted plan reads complete
without its step. `check_registry_dispositions` therefore reads every
board-writing `KRT_TOOL` declaration (`kind` actor or conditional, by AST via
`krt_registry`) and requires each to be MAPPED to a plan action (or
`CAP_TOOL`), REFUSED with a reason (`REFUSED_TOOLS`, loud on stderr), or a
named NO-OP with a reason (`NOOP_TOOLS`: `copy_board`, `fill_for_delivery`,
`plane_score` -- nothing to replay on a live board); a `check_*` conditional
is dropped by `is_check_cmd`. It runs three negative controls every time (one
entry removed from each table must be reported). `KRT_REGISTRY_ROOT` points
the registry read at the real checkout when a harness runs the gate on a
partial copy of the tree (`tests/test_route_flag_plan_coverage_controls.py`).

## Placement Apply locks (test_placement_apply_locks.py)

Needs KiCad's python; re-execs into it. Drives the REAL
`PlacementTab._on_apply` on a pcbnew board and requires every live footprint
lock to equal the result board's (#1054) -- locked in place, moved and
locked, unlocked and moved -- with each change named in the confirmation.
It runs on the sys.path KiCad's own plugin loader builds (no
`kicad_routing_plugin/` entry, asserted), which is how it caught Apply dying
on `from gui_utils import` in KiCad itself; `tests/test_plugin_sibling_imports.py`
is that rule's wx-free half.

## A step that produces nothing stops the plan (test_plan_refused_step_stops.py)

Needs KiCad's python; re-execs into it. A CLI step that refuses writes no
board, so the chain stops on the missing file. The REAL `PlanExecutor` on the
REAL dialog stops at a refused optimize_caps (#1067) or a declined route
step with the next step unrun, completes a plan whose steps produce, runs a
plane-net-only route step as a no-op, and each tab's real completion handler
marks its success ending only (`gui_utils.mark_plan_step_produced`).

## Cap-param delivery (test_772_cap_params_reach_engine.py)

Needs KiCad's python (wx + pcbnew); re-execs into it automatically.

    python3 tests/gui_parity/test_772_cap_params_reach_engine.py

The converter gate above asserts a flag survives into `step['params']`, and
`check_param_resolution` asserts the param NAME matches some control
somewhere. **#772 shipped past both of them.** `ai_plan._owners()` returned
`[dialog]` for an `optimize_caps` step, and all ten "Cap Placement
(advanced)" controls live on `fanout_tab.bga_options` -- so every `cap_*`
param was logged "no control, ignored" and the cap engine ran at its
signature defaults, while `--board-edge-clearance` resolved onto the Basic
tab's SIGNAL copper-to-edge control and left its override box ticked for the
next step. Conversion was fine; DELIVERY was not, and nothing measured
delivery.

This gate drives the REAL `PlanExecutor._next_step` on a real headless
`RoutingDialog` and reads the kwargs `repair_fanout_clearance` is handed.
Eleven groups: all ten knobs delivered; the Basic-tab override left
unticked; the legacy spelling re-homed; 0-means-UNSET on both fronts; no
leak between two consecutive cap steps; a bare step still inheriting; the
shared Basic-tab knobs untouched; #768's clearance and ceiling still
arriving; the full reset covering all ten controls; the defaults table true
of a freshly constructed panel; and that table equal to the engine
signature.

**The spy must patch `placement.fanout_clearance`, not `fanout_gui`** --
`_optimize_decoupling_caps` imports the engine inside the method body, so a
`fanout_gui` patch records nothing, silently. The gate also refuses a run in
which the engine was never reached, because otherwise every kwarg reads back
absent and it passes vacuously.

Its wx-free half is `tests/test_772_ai_plan_cap_params.py` (arms over the
shape of the fix, ~0.1 s, collected by `run_all.py`), which also asserts
this file exists -- `run_all.py`'s flat glob never collects this directory.

`check_owner_scoping` in the converter gate is the third piece: it
AST-extracts `ai_plan._ACTION_OWNERS` and a PER-CLASS control map (where
`_gui_control_attrs` is a flat union) and asks whether the action carrying a
param can actually REACH it. Verified as a change detector: removing
`bga_options` from the cap step's owners makes it name all ten dropped
params and exit 1.

## Declared cap rotations (test_1122_cap_rotation_gui.py)

Needs KiCad's python (wx + pcbnew); re-execs into it automatically, and
exits 2 without it.

    python3 tests/gui_parity/test_1122_cap_rotation_gui.py

#1122: the cap pass reads the intent's rotation claims through
`fanout_clearance.declared_cap_rotations`, which resolves the blocks
against the PCBData the GUI built from pcbnew, so the CLI test cannot
speak for this front. On the CLI test's fixture (the U30 crop, clearance
0.1, budget 0.6 / 2.0, C24 declared at 270 -- the case in which the run
KEPT is the one without the decap gate) it asserts that a contradictory
declaration stops the step and moves nothing, that the live C24 keeps
its angle, and that the GUI moves the caps the CLI moves, to the same
poses.

## Class-2 post-pass coverage (test_cli_postpass_coverage.py)

The converter gate above covers the plan->params translation; this one covers
the OTHER drift axis: a CLI `main()` running a finalization pass AFTER its
shared engine call that the GUI must separately replicate (Class 2). That is
how the set11 GUI board shipped 35 plane shorts the CLI board didn't have --
repair_planes.main() ran clean_plane_copper and the planes tab
never did.

Static, no wx/pcbnew. It AST-scans each CLI main() for post-engine passes,
and for each registered pass asserts a GUI counterpart symbol exists under
kicad_routing_plugin/; a finalization-module symbol used in a CLI main but not
registered fails, so a NEW CLI-only post-pass can't be added without wiring the
GUI. Fault-injection verified: renaming the GUI counterpart -> FAIL.

    python3 tests/gui_parity/test_cli_postpass_coverage.py

When you add a post-engine pass to a CLI main(), either put its core in the
shared engine (best -- both fronts inherit it), or refactor a board-level core
and call it from both fronts (as clean_plane_copper now does), then register
the pass + its GUI counterpart here.

## Grade parity on a set11-class board (test_gui_livechain_rp2350.py)

The copper-identity harness measures overlap %, which diverges even when both
fronts grade clean (rip-up routing is chaotic; #362). The invariant that
matters is GRADE parity. This gate chains the rp2350 PLANE sub-chain in its
#562 shape — pour → ONE route step carrying the plane nets in its net list,
whose in-run finalize is the weld/repair/oracle — on ONE live board, as the
Claude-tab plan executor does, in-memory across steps, and asserts that no
stage, on either front, INTRODUCES DRC: each stage is graded against its own
project, and what the input board already had is graded under that same
project and counted apart. (It was reshaped 2026-08-04: the old
create → repair → reconnect → repair2 plan had rotted into comparing
different chains, because the executor skips `repair_planes` steps as #562
no-ops while the CLI leg still shelled the old repair script.) The fixture
is staged WITH a pcbnew-authored .kicad_pro — a project-less board makes
the fronts legitimately diverge. It caught the swig_gui route-apply
width-rounding bug (0.0762 → 0.076 fab-floor violations, #362) that
per-step isolation on file inputs missed.

    python3 tests/gui_parity/test_gui_livechain_rp2350.py

## Focused regression gates (from the #362 plane sweep on)

Gates that each lock in one fixed GUI/CLI divergence. Each needs KiCad's
python and re-execs into it; without one it prints `SKIP:` or exits 2, never a
pass. Run any directly:

- `test_footprint_position_sync.py` -- `_sync_pcb_data_from_board` refreshes
  footprint/pad positions after optimize_caps (matched by iteration ORDER, not
  pad number -- U6 has 11 pads numbered "61"); a no-op sync moves ZERO pads.
- `test_settings_roundtrip.py` -- save/restore of the dialog's settings dict
  against the REAL headless dialog: the close path (`get_dialog_settings`),
  the reopen path (`restore_dialog_settings`), restore from a LEGACY dict
  carrying keys a newer version dropped, restore from a dict MISSING each key
  in turn (a settings file saved before that control existed), and re-save
  key parity. Deleting or
  renaming a control without updating persistence crashes on CLOSE and loses
  the user's settings -- this gate is what catches that. Key parity cannot see
  a VALUE that silently fails to come back, so it also round-trips one: a
  non-current Escalation choice must reappear in a fresh dialog (it never did
  from #857 until the fix: a swallowed NameError on `wx.NOT_FOUND`).
- `test_900_live_class_clearance.py` -- `apply_targets_to_board` must write the
  net classes at the ROUTED clearance, never at the `rules.min_clearance` value
  capped at a pad's `(clearance ...)` override (#530/#900). The signal, planes
  and differential tabs all reach that writer with live minima carrying the
  override, and the correct `update_live_drc_floors` write that follows is
  only-lower, so it can never undo a capped class. `flat_hierarchy` (Default
  0.2 / Wide 0.4) with a 0.0508 override set through pcbnew's own setter; a
  second arm at a different routed value catches a writer that ignores its
  argument. The wx-free half (`tests/test_900_class_clearance_not_capped.py`)
  can only guard this writer by source text.
- `test_plane_all_layers_parity.py` -- GUI create passes `all_layers` =
  outer+pour (the route_planes default), not all 6 copper layers (mocks
  create_plane to capture the kwarg).
- `test_plane_layer_costs_gui.py` -- and one Layer Costs value per entry of
  that `all_layers`, placed by name from the Basic-tab field (one per copper
  or per selected layer). The tab used to hand over the SELECTED layers'
  costs, which create_plane read positionally against its own list; a
  per-selected-layer field that cannot price an unselected pour layer is
  refused before the engine. Real dialog, create_plane mocked.
- `test_live_fab_floor_origin.py` -- a GUI routing step's live writeback
  (`apply_targets_to_board`, then `update_live_drc_floors`) records the board's
  ORIGINAL fab floors in `kicad_routing_tools.fab_floor_origin` before lowering
  them, and prints FAB FLOOR RELAXED against it with a census of the live
  copper -- the file writers' rule (ad7f24de), which the GUI never followed.
  It also runs the fanout tab's step, where `update_live_drc_floors` is the
  ONLY writer: the other steps call `apply_targets_to_board` first, which
  seeds, so they cannot see an `update_live_drc_floors` that seeds after
  lowering (it then records the lowered 0.15/0.4 and prints nothing). And a
  PROJECT-LESS board through two steps on both fronts: pcbnew's factory floors
  (track 0.2, via 0.5, hole 0.3) are not the board's, so step 1 prints nothing
  on either front, and step 2 prints the same relaxations (via drill
  included, the repo's floor pcbnew has no attribute for). An unchanged step
  says "unchanged by this step but still below", in the CLI's words. On the
  code before that fix it fails 3 checks: the GUI's step 1 reported pcbnew's
  factory floors as relaxed, and its step 2 named different floors and values.
- `test_gnd_vias_gui.py` -- the Planes tab's **Add GND vias** step, run for
  real (nothing mocked) against `route_planes.py --add-gnd-vias` on the same
  files: return vias keep a net-class clearance (#1030), a `.kicad_dru` layer
  rule (#498), every copper layer (not the plane step's outer+pour set), and
  the fab copper-to-edge floor; GUI and CLI place identical vias. On the
  pre-2026-09-24 code it fails 9 checks, CLI included (its rules lookup read
  the output's not-yet-copied `.kicad_dru`). Its `file` arm (#1272) sets the
  Net Clearances File / `--net-clearances`: the file replaces the class map
  as given on both fronts (2 checks fail before #1272: the tab ignored the
  file, route_planes.py refused the flag).
- `test_skip_existing_zones_gui.py` -- the Planes tab's **Skip existing
  zones** box against `route_planes.py` with and without
  `--skip-existing-zones`, on the real tab (create + apply on a live board):
  a net's single-layer pour is replaced (old one deleted, one new pour at the
  run's clearance and width), kept with the box, a multi-layer pour is never
  deleted to replace one of its layers and is out-ranked, and a GND teardrop
  neither blocks the pour nor goes with it. GUI and CLI leave identical GND
  pours; a negative control strips the engine's `replace_existing` marks.
- `test_1299_multilayer_zone_replace_gui.py` -- a `(layers "F.Cu" "B.Cu")`
  GND pour, poured on both layers and on B.Cu alone, on the real Planes tab
  (create + apply) against `route_planes.py` (#1299). Both layers: the 2-layer
  pour is deleted and one GND pour per layer added. One layer: it is kept
  under a new B.Cu pour. GUI and CLI leave identical pours. Negative control:
  the engine's check given no call layers keeps the 2-layer pour under the new
  ones. With the tab's pre-fix apply (single-layer pours only) the both-layers
  arm fails on the GUI side alone.
- `test_1056_renamed_layers_gui.py` -- a board whose In1.Cu/In2.Cu are
  renamed "GND"/"+5V" in Board Setup (#1056). `board.GetLayerName()` returns
  that display name, so every live-board site that mapped through it missed:
  the Planes tab poured In1/In2 planes onto its F.Cu fallback and duplicated
  them on a second Create, the Route tab's rip strip left the ripped track,
  `live_fill_islands` keyed `('GND', 'GND')`, and the builder read a User.1
  renamed "In1.Cu" as copper. Real Planes/Route tabs, each arm with a
  negative control (the display-name mapping patched back in). On the
  pre-fix code the pour lands on F.Cu and a re-apply doubles it.
- `test_1133_staged_save_widths.py` -- the GUI oracle leg
  (`gui_utils.run_kicad_oracle_on_live_board`) on a real `pcbnew.SaveBoard`
  of `flat_hierarchy`, whose save renumbers 94 of its 111 nets (pcbnew 8 ->
  staged 81 for `/pic_programmer/PC-CLOCK-OUT`). A power width posted BY NAME
  for one net must reach exactly that net on the staged board, and the track
  the oracle lays must land on that net on the LIVE board (`SetNetCode`
  receives the live netcode, not the save's id). Negative control: the live
  netcode names another net (GND) in the save. KiCad's link source, the
  routers and the sliver weld are stubbed; the wx-free half is
  `tests/test_1133_oracle_width_keying.py`.
- `test_1195_qfn_floors_gui.py` -- the fanout tab's REAL
  `_apply_fanout_results` on a real `flat_hierarchy` board (via 0.5 / drill
  0.3, no vias): a QFN step with no via leaves the via and drill floors, a QFN
  step with no copper writes nothing, a BGA step lowers the via floors (the
  control), and both fronts decide with
  `fix_kicad_drc_settings.fanout_written_floors`. On the pre-fix tab it fails
  3 checks: a stub QFN run lowered the via floor 0.5 -> 0.3.
- `test_fanout_floor_parity_gui.py` -- the fanout tab's REAL
  `_apply_fanout_results` writes the floors bga_fanout's / qfn_fanout's mains
  write (`fix_kicad_drc_settings.fanout_fix_kwargs`, through the live twin
  `apply_fix_to_board`): a BGA step with a track and a via, a QFN step with a
  track, a QFN step with no copper, on flat_hierarchy (a project, Default 0.2 /
  Wide 0.4) and on project-less splitflap_driver. The live copper is saved, the
  original project put beside it and the CLI writeback run over it with the
  mains' keywords spelled out in the test; every declared rule, the via-drill
  floor and every class clearance must agree. On the pre-fix code it fails 8
  of 12 checks: the copper-to-hole floor kept at 0.25 (CLI 0.15), Wide not
  clamped (0.4 vs 0.15), and on the project-less board hole-to-hole written,
  min_clearance 0 / edge 0.5 / hole diameter 0.3 left where the CLI writes
  0.15 / 0.2 / 0.95 and creates the Default class. A static arm checks every
  `fanout_config` dict the tab really builds carries what the writeback reads:
  the QFN one had no clearance, via size or drill, so the real QFN run wrote no
  clearance where qfn_fanout does.
- `test_records_survive_save.py` -- what a GUI step records survives the
  user's KiCad save. KiCad's project save rewrites the `.kicad_pro` from the
  copy it loaded and drops a `kicad_routing_tools` record written after load,
  so the GUI keeps the namespace on the board too (`py_router/tool_records.py`,
  the board property `kicad_routing_tools` that the same save writes into the
  `.kicad_pcb`), and every reader takes the newer copy by `records_rev`. On
  flat_hierarchy, then on project-less splitflap_driver: a manual GUI step's
  writeback, `pcbnew.SaveBoard`, then the CLI's own readers find the origin,
  protected nets, impedance specs and pour promises and the hole floor's
  origin, and a CLI step still says FAB FLOOR RELAXED against the original.
  Also the board's copy parses the same on both fronts and the CLI writer
  carries it, and the plan executor's REAL `_write_drc_floors`, then a save,
  keeps every rule it wrote (it now writes them to the live board too, through
  `apply_fix_to_board`) and its origin. On the pre-fix code it fails 14
  checks. It also runs one arm per record a manual GUI step used to leave
  unwritten -- each a GUI run, a KiCad save, then the CLI's own reader against
  what the CLI step writes: an active #581 same-net pad via clearance (and every
  tab's step boundary passing it), a #1160 Default-class descent record and its
  disclosure, and the repo's `min_via_drill` floor (mirrored into
  `kicad_routing_tools.repo_rules`, since KiCad's save drops a non-KiCad rule
  written after load), including a next CLI step leaving both chains at the
  same floor. #856 relaxed severities: KiCad 10.0.0's SWIG cannot change a
  severity (no SetSeverity, no DRCE_* names), so the GUI changes none, records
  no `saved_severities`, and now says so; where a pcbnew exposes the setter the
  arm compares the records instead. Those arms fail 9 checks on the code
  before them. The wx-free half is `tests/test_tool_records.py`.
- `test_1187_live_web_floor.py` -- the narrow-pad-joint floor
  (`connection_width_floor`) on a live board reads the board's design settings
  through `PCBData.live_rules_provider`, at call time: after one GUI step's
  writeback the SAME PCBData answers the lowered live floor while the
  `.kicad_pro` on disk still declares the original, and a live
  `min_connection` outranks `min_track_width`. On the pre-fix code it fails 3
  checks (the floor read the stale file, 0.3 where the board said 0.15).
- `test_live_hole_clearance.py` -- the copper-to-hole floor
  (`obstacle_map.resolve_hole_clearance`, NPTH track keep-out plus #1038's
  via keep-out) on a live board is the board's in-memory `m_HoleClearance`,
  read at call time: an UNSAVED Board Setup edit on `flat_hierarchy` is priced
  by a PCBData built before it while the file still says 0.25, and
  compare_pcb_data names the difference; after `pcbnew.SaveBoard` a file
  parse in the same process reads the saved value; a project-less board loaded
  at pcbnew's factory 0.25 reads 0.0 on both fronts, and after one GUI step's
  writeback -- including one that targets exactly 0.25, which moves nothing in
  memory -- the GUI reads the floor the CLI's next step reads from its own
  writeback; the snapshot the oracle legs parse (`stage_live_project_rules`)
  carries only the rules the board declares, so its hole floor and edge band
  are the CLI's; an unsaved board (#1238) prices a live edit; and the
  `pcbnew.DEFAULT_*` constants the declared view reads match a fresh board's
  design settings. Its first 17 checks failed 7 on the code before the live
  read (b861d208); the 5 the declared view added fail on the code before it
  (0ec26d35). The wx-free half is `tests/test_1038_hole_floor_fresh.py`.
- `test_581_fanout_via_in_pad_gui.py` -- the Basic tab's via-in-pad policy
  (#581) reaches BOTH escape engines from the REAL fanout tab, on glasgow's
  U30 (BGA) and U1 (QFN), engines spied: unticked with a 0.15 spin, each
  receives `same_net_pad_clearance=0.15`; ticked (the control), -1.0. On the
  pre-fix tab it fails 4 checks -- #621's move to the worker thread had dropped
  the kwarg from both calls, and `test_engine_kwarg_parity` could not see it
  because it read no `**kwargs` call (it now follows `GUI_KWARGS_DICTS`, and a
  pair it cannot read FAILs instead of printing SKIP).
- `test_via_type_parse_parity.py` -- a via's copper span (#1268). The text
  parse, the pcbnew parse and the Route tab's live re-sync
  (`_sync_pcb_data_from_board`) are each held, layer by layer, to pcbnew's
  own `PCB_VIA.IsOnLayer` on a board of through, blind, buried and micro vias
  and on the same board with every type word stripped -- which KiCad loads as
  through vias, and which is the shape this repo's writer emits. A span read
  off the layer pair fails the stripped board on the text side only. A third
  arm re-creates every typed via the way the plugin does
  (`gui_utils.set_via_layers`: the type before the layer pair, which pcbnew
  otherwise snaps to F.Cu/B.Cu) and holds it to the original copper.
- `test_1279_sense_branch_live.py` -- the live board keeps a `sense:` branch
  (#1279). After the Route tab's re-sync the tracks, arc pieces and vias carry
  what a fresh `build_pcb_data_from_board` reads (one shared extractor), copper
  graphics survive, the branches and the protection map (memo included) match
  a fresh build, a group drawn or removed in the session counts at the next
  sync, and a Route-tab run with the branch's net rippable never strips the
  sense track. Before the shared extractor the re-sync carried no uuid, no
  KiCad lock and no arcs: 9 checks fail there. Its Apply arm holds the
  Placement tab's Place + Route Apply to keeping the copper the run left
  unchanged (uuid, arc, group membership) and writing no lock token the result
  file does not carry; the wholesale remove-and-recreate failed 6 checks. Its
  last arm moves and turns a part whose land pattern draws F.Cu copper (#908)
  in the live session: after the sync the model's copper graphics must equal
  a fresh build's. A sync that kept the graphics built at dialog open left the
  tab where the part had been.

## Whole-plan replay through the real plugin (replay_plan_vs_run.py)

Every stress board leaves a GUI-loadable plan beside its chain (`run_board.sh`
→ `manifest_to_plan.py` → `<board>_plan.json`; `py_router/make_plan.py` builds the same
file from any recorded manifest). This driver does what a user does with it —
open the unrouted board, AI tab → **Load...** → **Run All Selected Steps** —
with no buttons and no LLM, then diffs the result against the CLI chain.

The headless driving itself lives in **`py_router/headless_plan.py`**, which
is also what the user-facing `py_router/run_plan.py` runs (#507) — so this
harness exercises the shipped driver instead of a private copy. Same for the conversion:
`_regen_steps` calls `manifest_to_plan.plan_steps_from_manifest`, the converter's
own function, rather than re-implementing its loop.

    python3 tests/gui_parity/replay_plan_vs_run.py <rundir>
    python3 tests/gui_parity/replay_plan_vs_run.py --set <runs_setN> [--boards a,b]

**Nothing is mirrored.** The real
`swig_gui.RoutingDialog` is constructed headless (`parent=None`, never shown —
wx needs `WXSUPPRESS_SIZER_FLAGS_CHECK=1`, which the script sets, because
`about_tab`'s `wxEXPAND|wxALIGN_*` sizer flags trip a debug assert) with its
real tabs and controls, and the real `ai_plan.PlanExecutor` drives it inside
a `wx.MainLoop`. So the whole `parse_plan_result` → `reset_params_to_defaults` →
`apply_step_params` → `apply_step_selection` → `tab._on_*()` path is under test,
including the two translation layers the old shim harness could not see (the
converter and the apply side — the #361 class).

**Two lessons are baked in; both were measured, not assumed.**

1. **The reference is RE-RUN at HEAD, never read off disk.** Recorded corpus
   boards carry their run-date engine. On nano_eeprom_prog the recorded
   (2026-07-09) board has 419 segments and the *same manifest replayed at HEAD*
   has 570 — so grading a HEAD GUI replay against the recorded board reports a
   466-segment "divergence" that is entirely the router having moved on. The
   default replays the pruned chain into `<workdir>/cli_head` with
   `redo_stress_test.py` (same pruning ⇒ intermediate board names line up 1:1
   with the plan's steps, which is what makes per-step pairing valid) and grades
   the recorded board alongside, reported separately as *engine drift*.
   `--cli-ref recorded` opts out and is only honest at the recording commit.

2. **The comparison is exact, to the nanometre.** Both fronts emit copper
   through `kicad_parser.mm_to_iu`, which rounds to pcbnew's integer-nm grid
   (the GUI used to snap applied copper to 1 µm and truncate in
   `pcbnew.FromMM`; see #493 below), so `--dp` defaults to 6 (1 nm, the
   board's own resolution). The comparator still reports a **tri-state**
   rather than hiding a residue: `IDENTICAL` (exact multiset equality),
   `EQUIVALENT` (every leftover pairs within `--tol`, default 1 µm: a
   representational difference only), `DIFFERS` (genuinely unmatched copper
   remains). Loosen `--dp` only to characterise a divergence you are already
   chasing.

Compared at four levels, on the finals **and after every step** (the GUI board
is snapshotted per step, so the report names the FIRST step producing genuinely
different copper): grade (router-attributable DRC + connectivity through
`kicad_drc_compare.compare_board_data`, the corpus's own core, so the numbers
are comparable by construction), segments, vias, zones, and footprint poses
(`optimize_caps` moves parts — the #362 position-sync class).

Exit codes: 0 same routing (IDENTICAL or EQUIVALENT), 1 copper genuinely
differs, 2 grade differs, 3 replay failed. Other options (`--plan-source
regen`, `--raw-plan`, `--max-steps N`, `--gui-board`, `--json`, `--fresh`,
...) are in the script's docstring.

Note the cost: it runs **both** legs (CLI chain + GUI chain), so budget roughly
twice a single replay per board.

### What it found, and what got fixed (2026-07-25) — #493

The driver's first two runs turned up five divergences, all sitting underneath a
**green grade on both sides** — which is the point: none of them would ever
surface as a failing DRC or connectivity number. All five are now fixed.

1. **`!GND` never excluded `/GND`.** `net_queries.matches_net_filter` and
   `expand_net_patterns` fnmatched the FULL, sheet-qualified net name, so the
   unqualified spelling everybody writes matched nothing and the exclusion
   silently no-opped. eth_tap's step-1 fanout shipped 267 `/GND` + 171 `/3V3`
   segments (438 of 750) from a command asking to exclude both. This was already
   diagnosed once as #292 and closed with a "did you mean" warning instead of a
   fix. Now a pattern naming no path also matches the trailing component
   (`net_pattern_matches`), shared by the CLI, engine and GUI matchers.
2. **The GUI's fanout ignored "!" exclusions outright** —
   `ai_plan._component_net_names` tested `any(glob matches)` over the whole
   list, so `'*'` always won. It only looked right where `_drop_plane_nets`
   happened to remove the same nets.
3. **The GUI routed against a slightly different clearance.** `swig_gui` read
   the board's net class with `nm * 1e-6`; 1e-6 is not exactly representable, so
   `200000 * 1e-6` is `0.19999999999999998` and `450000 * 1e-6` is
   `0.44999999999999996` (0.3/0.25/0.127/0.09 are clean, which is why it only
   bit some boards). `_effective_geometry_floor` clamps the entered value to the
   Default class, so those epsilons went to every engine as `clearance` /
   `via_size`. Now `_nm_to_mm` divides.
4. **The GUI's plane oracle processed power nets the CLI's does not.** The
   post-apply kicad-oracle recheck built its net list from the plane assignments
   *plus* `power_nets` — but `power_nets` is a track-WIDTH assignment, not a set
   of nets to reconnect. On nano_eeprom_prog it reported 4 missing links on
   `+5V`, failed to clear them across all 3 rounds, and routed 11 of them
   anyway: 30 segments of `+5V` copper the CLI board does not have, while the
   engine itself reported "0 regions, 0 pads repaired".
5. **GUI-applied copper was quantised twice.** Positions went through
   `pcbnew.FromMM(round(v, POSITION_DECIMALS=3))`, snapping off-grid copper to
   1 um (via SIZE and DRILL too — the same class as the #362 track-width bug);
   and `pcbnew.FromMM` itself TRUNCATES (`66.1 * 1e6` is `66099999.99999999`, so
   `FromMM(66.1) == 66099999`, one nm short of what the CLI's text writer
   produces). Both fronts now emit through `kicad_parser.mm_to_iu`, which rounds.

**Result: nano_eeprom_prog is now byte-for-byte identical** between the GUI
replay and the CLI chain — every segment, via, zone and footprint pose, at
1 nm resolution, at every step. Bit-identical GUI/CLI copper was previously
believed impossible (see the older note above about ~77% overlap); it isn't, and
`--dp` now defaults to an exact nanometre comparison rather than a tolerance.

## Checked-in test inputs

Every input the gates read is tracked: boards under `kicad_files/` and one
under `awx/`, fixtures under `tests/fixtures/`
and `tests/gui_parity/fixtures/`, and boards the gates build from those in a
temp dir. `test_manifest_plan_parity.py` always includes
`fixtures/sample_redo_commands.sh`, and reads the stress corpus's manifests
only when `$STRESS_DIR` has them. So every gate runs on a fresh checkout
without the external stress corpus, except `replay_plan_vs_run.py`, whose
input IS a stress run.

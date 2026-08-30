# GUI/CLI engine-parity harness

Measures whether the claude tab's "run selected steps" (GUI engine calls on
the live pcbnew board) produces the same final board as the recorded stress
chain's CLI steps run file-to-file.

- `replay_plan_vs_run.py` — **start here.** Loads a stress run's
  `<board>_plan.json` into a genuine headless `RoutingDialog` and runs it with
  the genuine `PlanExecutor`, then diffs against the manifest replayed at HEAD.
  Nothing is mirrored or mocked. See "Whole-plan replay" below.
- `test_gui_engine_parity.py`, `test_gui_livechain_rp2350.py` — **MIGRATED onto
  the real dialog (2026-07-26)**, and the only gates that run on IN-REPO boards
  (`splitflap_driver`, `rp2350_fpga_eensy_prePlane`). Each now expresses its GUI
  leg as a Claude-tab **plan** and runs it through
  `replay_plan_vs_run.replay()` — the same real `RoutingDialog` +
  `PlanExecutor`. `replay()` touches its `info` argument only for
  `input_board`, so the corpus driver works unchanged on a checked-in board;
  that is the whole trick. Nothing is mirrored or shimmed any more.
  `diag_fullchain_carry_rp2350.py` was **deleted** — a corpus-only, one-off
  diagnostic for the #361 carry investigation, whose findings are recorded
  below.

### Why the shims went (kept for the reasoning)

The shim-based harnesses predated the discovery that the REAL dialog
constructs headless (`parent=None`, never shown, with
`WXSUPPRESS_SIZER_FLAGS_CHECK=1`). They bound real tab methods onto plain shim
objects and hand-built the config the engine is called with, which has a
structural blind spot: **anything between a dialog CONTROL and the engine
argument is never executed.**

That is not hypothetical. #493 item 3 was `swig_gui` reading the board's net
class with `nm * 1e-6`, landing one ULP low (`0.2` → `0.19999999999999998`) and
handing that to every engine as `clearance`. It reaches the engine through
`_effective_geometry_floor` → `_get_netclass_parameters`; the shim harness calls
`batch_route(clearance=step['clearance'], ...)` with its own literal, so it
could never have seen it. `replay_plan_vs_run.py` caught it on its first run.

The shims also rot. `test_plane_all_layers_parity.py` currently dies with
`AttributeError: 'Shim' object has no attribute '_cancel_requested'` — the tab
grew a field the shim never modelled, so that gate has been failing on its own
scaffolding, not on the behaviour it guards (confirmed pre-existing at 75acd26,
before the #493 work). A harness that mirrors an interface has to be maintained
in lockstep with it; one that instantiates the real thing does not.

**And a shim does not only MISS bugs — it can INVENT them.** Because
`test_gui_engine_parity`'s shim hand-built the plane step's config, it never ran
`_effective_track_width()`, so the engine's own
`config.get('track_width', defaults.TRACK_WIDTH)` fallback supplied 0.3 where
the real dialog resolves the board's Default net class to 0.127. The gate
reported 73 "divergent" GND plane-tap segments on splitflap, they were written
up here as a real unfixed defect, and they do not exist in the real GUI (see the
splitflap measurement above). That is what finally motivated the migration.

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

> **Superseded (2026-07-25, #493).** Everything in this section and the two
> below it was measured with the shim harness before the real-dialog driver
> existed, and its central conclusion — that bit-identical GUI/CLI copper would
> need engine-level order canonicalization — is **wrong**. The residual was not
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

**The 0.3 is the shim.** `test_gui_engine_parity.py` hand-builds its step config
(`PLANES = dict(nets=['GND'], layers=['B.Cu'], clearance=0.15, ...)` -- no
track_width) and calls the engine directly, so `_effective_track_width()` never
runs and the engine's own `config.get('track_width', defaults.TRACK_WIDTH)`
fallback supplies 0.3. This is exactly the structural blind spot described
above: **anything between a dialog CONTROL and the engine argument is never
executed.** Treat this 73-segment delta as harness noise, not a defect -- and as
one more argument for migrating these gates onto the real dialog.

## Command/input identity findings (2026-07-08)

- Harness bug found by asking 'were the commands the same?': the GUI leg
  hardcoded ordering_strategy='inside_out' while the real GUI default (and
  CLI default) is 'mps'. Fixed; the two legs now run the same effective
  parameters.
- With commands matched: 381/~1200 segments common. With the
  GUI_PARITY_INPUT=parser diagnostic (GUI leg text-parses a temp-saved
  board, isolating the builder-vs-parser representation): 444/~1200.
- Conclusion: the dominant copper fork is NOT parameters and NOT coordinate
  representation -- it is item ORDERING (pcbnew re-save normalizes element
  order; MPS ordering, spatial-index insertion and A* tie-breaks all follow
  it) plus the CLI chain's per-step .kicad_pro floor carryover. Grade
  parity is unaffected. Bit-identical copper would require engine-level
  order canonicalization (sort nets/pads/segments deterministically before
  routing) -- a follow-up decision, not a bug.

## Parameter-identity verification (KICAD_DUMP_BATCH_KWARGS)

route.py's batch_route dumps its FULL parameter set (76 keys) and returns
when KICAD_DUMP_BATCH_KWARGS=<file> is set -- diffing a CLI invocation
against the GUI leg's call verifies the two fronts hand the engine the
same parameters. Current state: only `return_results` (definitional) and
`layer_costs` [] vs None (proven equivalent via get_layer_costs) differ.

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
(builder representation) sits at ~77% copper identity with full grade
parity; closing it to ~98% is the order-canonicalization follow-up.

### Plane engines + diff_engine_kwargs.py (the #362 sweep)

The dump now also covers the PLANE engines: `create_plane` (route_planes.py)
and the repair `route_planes` (repair_planes.py) each write a line
via `route._dump_engine_config` in CONTINUE mode, INCLUDING `all_layers` /
`plane_layers` (layer content/order is a live divergence class). So one
`KICAD_DUMP_BATCH_KWARGS_CONTINUE=1` run of a whole plan captures every engine
call -- route, diff, create_plane, repair -- in one file.

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

The harness above proves the ENGINE half (same batch_route kwargs -> same
board) but hand-mirrors the plan->params mapping, so it can't catch a bug in
`manifest_to_plan` (converter) or `ai_plan.apply_step_params` (apply).
Those two translation layers are where the set11 rp2350_fpga_eensy GUI replay
silently diverged from its CLI board (242 DRC violations vs 0; issue #361).

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
job and belong to the wx harness / a future stub-dialog apply test.

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

Its wx-free half is `tests/test_772_ai_plan_cap_params.py` (16 arms over the
shape of the fix, ~0.1 s, collected by `run_all.py`), which also asserts
this file exists -- `run_all.py`'s flat glob never collects this directory.

`check_owner_scoping` in the converter gate is the third piece: it
AST-extracts `ai_plan._ACTION_OWNERS` and a PER-CLASS control map (where
`_gui_control_attrs` is a flat union) and asks whether the action carrying a
param can actually REACH it. Verified as a change detector: removing
`bga_options` from the cap step's owners makes it name all ten dropped
params and exit 1.

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
Claude-tab plan executor does, in-memory across steps, and asserts every
stage grades 0 DRC like the CLI. (It was reshaped 2026-08-04: the old
create → repair → reconnect → repair2 plan had rotted into comparing
different chains, because the executor skips `repair_planes` steps as #562
no-ops while the CLI leg still shelled the old repair script.) The fixture
is staged WITH a pcbnew-authored .kicad_pro — a project-less board makes
the fronts legitimately diverge. It caught the swig_gui route-apply
width-rounding bug (0.0762 → 0.076 fab-floor violations, #362) that
per-step isolation on file inputs missed.

    python3 tests/gui_parity/test_gui_livechain_rp2350.py

## #362 plane-parity regression gates

Focused gates that each lock in one fixed GUI/CLI plane divergence (wx-gated;
skip cleanly without KiCad python). Run any directly:

- `test_footprint_position_sync.py` -- `_sync_pcb_data_from_board` refreshes
  footprint/pad positions after optimize_caps (matched by iteration ORDER, not
  pad number -- U6 has 11 pads numbered "61"); a no-op sync moves ZERO pads.
- `test_settings_roundtrip.py` -- save/restore of the dialog's settings dict
  against the REAL headless dialog: the close path (`get_dialog_settings`),
  the reopen path (`restore_dialog_settings`), restore from a LEGACY dict
  carrying keys a newer version dropped, and re-save key parity. Deleting or
  renaming a control without updating persistence crashes on CLOSE and loses
  the user's settings -- this gate is what catches that.
- `test_plane_all_layers_parity.py` -- GUI create passes `all_layers` =
  outer+pour (the route_planes default), not all 6 copper layers (mocks
  create_plane to capture the kwarg).
- `test_movie_recorder.py` -- the Advanced tab's **Make routing movie** debug
  checkbox (#506): default OFF and inert while off; one routing step renders
  ONE movie; a plan run (`begin_group`/`end_group`, what the AI tab's Run
  Selected Steps brackets) renders ONE movie for ALL its steps; the path is
  logged in GREEN; and `reset_params_to_defaults` must NOT untick it (the plan
  executor calls that before every step, so a reset there would abandon the
  run's movie halfway) while an explicit reset-all does.
- `diag_fullchain_carry_rp2350.py` -- **DELETED 2026-07-26.** Was a full
  GUI-carry reproduction (ONE board + ONE shared pcb_data across all 10 plan
  steps, graded per stage vs the CLI) built to localize where the carry
  diverges. A one-off diagnostic, not a gate: it needed the set11 corpus so it
  skipped for anyone without it, and `replay_plan_vs_run.py` now does the same
  carry through the real dialog. Its findings stay recorded above.

## Whole-plan replay through the real plugin (replay_plan_vs_run.py)

Every stress board leaves a GUI-loadable plan beside its chain (`run_board.sh`
→ `manifest_to_plan.py` → `<board>_plan.json`; `make_plan.py` builds the same
file from any recorded manifest). This driver does what a user does with it —
open the unrouted board, AI tab → **Load...** → **Run All Selected Steps** —
with no buttons and no LLM, then diffs the result against the CLI chain.

The headless driving itself lives in the repo-root **`headless_plan.py`**, which
is also what the user-facing `run_plan.py` runs (#507) — so this harness
exercises the shipped driver instead of a private copy. Same for the conversion:
`_regen_steps` calls `manifest_to_plan.plan_steps_from_manifest`, the converter's
own function, rather than re-implementing its loop.

    python3 tests/gui_parity/replay_plan_vs_run.py <rundir>
    python3 tests/gui_parity/replay_plan_vs_run.py --set <runs_setN> [--boards a,b]

Unlike `test_gui_engine_parity.py`, **nothing is mirrored**. The real
`swig_gui.RoutingDialog` is constructed headless (`parent=None`, never shown —
wx needs `WXSUPPRESS_SIZER_FLAGS_CHECK=1`, which the script sets, because
`about_tab`'s `wxEXPAND|wxALIGN_*` sizer flags trip a debug assert) with its
real tabs and controls, and the real `ai_plan.PlanExecutor` drives it inside
a `wx.MainLoop`. So the whole `parse_plan_result` → `reset_params_to_defaults` →
`apply_step_params` → `apply_step_selection` → `tab._on_*()` path is under test,
including the two translation layers the engine harness cannot see (the
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

2. **GUI copper is snapped to 1 µm, CLI copper is not.** Everything the GUI
   applies goes through `pcbnew.FromMM(round(v, POSITION_DECIMALS))` with
   `POSITION_DECIMALS = 3` (`kicad_parser.py`), so any point not already on a
   1 µm boundary (diagonal joins, nudged vias) lands up to ~0.5 µm from the
   CLI's full-precision text value — `167.609` vs `167.6094`. Bit-identical
   copper is therefore impossible today for non-grid-aligned points. The
   comparator reports a **tri-state** rather than hiding it: `IDENTICAL` (exact
   multiset equality), `EQUIVALENT` (every leftover pairs within `--tol`, i.e.
   only the apply rounding), `DIFFERS` (genuinely unmatched copper remains).
   Related but separate: `--dp` defaults to 5 because pcbnew's integer-nm
   storage vs decimal-mm text round-trips as `66.1` / `66.099999`, a 1 nm
   artifact; dp=5 is the finest resolution at which both representations agree.

Compared at four levels, on the finals **and after every step** (the GUI board
is snapshotted per step, so the report names the FIRST step producing genuinely
different copper): grade (router-attributable DRC + connectivity through
`kicad_drc_compare.compare_board_data`, the corpus's own core, so the numbers
are comparable by construction), segments, vias, zones, and footprint poses
(`optimize_caps` moves parts — the #362 position-sync class).

Exit codes: 0 same routing (IDENTICAL or EQUIVALENT), 1 copper genuinely
differs, 2 grade differs, 3 replay failed.

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

All boards these gates need are committed under `kicad_files/`
(`splitflap_driver.*`, `rp2350_fpga_eensy_prePlane.*`), and
`test_manifest_plan_parity.py` falls back to `fixtures/sample_redo_commands.sh`
-- so every gate runs on a fresh checkout without the external stress corpus.

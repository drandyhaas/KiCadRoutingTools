# GUI/CLI engine-parity harness

Measures whether the claude tab's "run selected steps" (GUI engine calls on
the live pcbnew board) produces the same final board as the recorded stress
chain's CLI steps run file-to-file.

- `test_gui_engine_parity.py` — headless: CLI leg via subprocesses, GUI leg
  inside KiCad's bundled python (auto re-exec) driving the real tab apply
  methods on shimmed dialog instances. Prints a parity report; known
  deliberate divergences are listed in the module docstring.

Workdir: `tests/gui_parity/work/` (gitignored).

## Comparison bars

1. **Grade parity** (the harness VERDICT): fully-connected, DRC-clean, and
   kicad-cli-refilled unconnected counts equal. This is the acceptance bar
   the stress tests themselves use.
2. **Copper-set identity** (the `COPPER-SET COMPARISON` block): segments and
   vias as canonical UUID-independent sets. Byte equality is meaningless by
   design (.kicad_pcb outputs carry per-run random UUIDs -- see project
   notes: never whole-file-diff routing outputs).

Current measurement on splitflap: grade PARITY, copper sets DIFFER
(~300/1200 segments common, same via count at different positions). The
router is deterministic, so the fork is input-representation differences:
the GUI leg routes PCBData built from pcbnew (float coordinates from nm
conversion, container orderings) while the CLI parses the file text --
A* tie-breaks diverge on the first sub-micron difference, and the .kicad_pro
floor carryover differs per step. Both outputs are equally valid; making
them bit-identical would require a canonical PCBData representation shared
by both fronts (open follow-up).

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
and the repair `route_planes` (route_disconnected_planes.py) each write a line
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
`manifest_to_plan` (converter) or `claude_plan.apply_step_params` (apply).
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
gaps (escape_method value->index, no_gnd_vias inversion) are claude_plan.py's
job and belong to the wx harness / a future stub-dialog apply test.

## Class-2 post-pass coverage (test_cli_postpass_coverage.py)

The converter gate above covers the plan->params translation; this one covers
the OTHER drift axis: a CLI `main()` running a finalization pass AFTER its
shared engine call that the GUI must separately replicate (Class 2). That is
how the set11 GUI board shipped 35 plane shorts the CLI board didn't have --
route_disconnected_planes.main() ran clean_plane_copper and the planes tab
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
matters is GRADE parity. This gate chains the rp2350 PLANE sub-chain (create →
repair → reconnect route → repair2) on ONE live board -- as the Claude-tab plan
executor does, in-memory across steps -- and asserts every stage grades 0 DRC
like the CLI. It caught the swig_gui route-apply width-rounding bug (0.0762 →
0.076 fab-floor violations, #362) that per-step isolation on file inputs missed.

    python3 tests/gui_parity/test_gui_livechain_rp2350.py

## #362 plane-parity regression gates

Focused gates that each lock in one fixed GUI/CLI plane divergence (wx-gated;
skip cleanly without KiCad python). Run any directly:

- `test_footprint_position_sync.py` -- `_sync_pcb_data_from_board` refreshes
  footprint/pad positions after optimize_caps (matched by iteration ORDER, not
  pad number -- U6 has 11 pads numbered "61"); a no-op sync moves ZERO pads.
- `test_plane_rip_blocker_panel.py` -- the plan executor sets `rip_blocker_nets`
  on the CORRECT plane options panel per action (repair vs create), not the
  first panel sharing the control name.
- `test_plane_all_layers_parity.py` -- GUI create passes `all_layers` =
  outer+pour (the route_planes default), not all 6 copper layers (mocks
  create_plane to capture the kwarg).
- `diag_fullchain_carry_rp2350.py` -- full GUI-carry reproduction: ONE board +
  ONE shared pcb_data across all 10 plan steps, graded per stage vs the CLI. The
  investigation harness that localized where the carry diverges (needs the set11
  corpus + kicad-cli; skips otherwise). `SYNC_PLANES=1` toggles the plane-step
  pcb_data resync.

## Checked-in test inputs

All boards these gates need are committed under `kicad_files/`
(`splitflap_driver.*`, `rp2350_fpga_eensy_prePlane.*`), and
`test_manifest_plan_parity.py` falls back to `fixtures/sample_redo_commands.sh`
-- so every gate runs on a fresh checkout without the external stress corpus.

# Project Notes for Claude

## Running Python

Invoke Python as `python3` (bare `python` does not exist on macOS and many
Linux distros). On Windows, if `python3` is missing, fall back to `py -3`
or `python` — don't retry blindly. Add `-X utf8` when a script prints
special characters (Ω etc.) to avoid Windows encoding errors.

## Building the Rust Router

Use `build_router.py` to build the Rust router:

```bash
python build_router.py
```

This builds the Rust module, copies the library to the correct location, and verifies the version. Do not run `cargo build` directly.

**Prefer Python-only solutions; avoid changing the Rust router (`rust_router/`)
unless clearly necessary and agreed.** A Rust change forces a crate version bump,
a `build_router.py --from-source` rebuild, and re-distributing prebuilt per-platform
binaries via GitHub Releases — heavy overhead. When a feature seems to need a Rust
change, surface that cost early and check for a Python-only approach first.

**Important:** When making changes to the Rust router, bump the version in `rust_router/Cargo.toml` and update the version history in `rust_router/README.md`.

## Testing & Verification

Validate routed boards against the *real* spec, with the right checker — most
"mystery" bugs here turn out to be grading mistakes, not routing bugs:

- **Connectivity is orthogonal to DRC.** A DRC-clean board can be fully
  disconnected (isolated copper has no clearance conflicts). Always run
  `check_connected.py` in addition to `check_drc.py` before calling a route clean.
- **Grade DRC at the clearance the board was actually routed to** — the route
  step's `--clearance` (recorded in `redo_commands.sh`), or the board's
  `.kicad_pro`/netclass — never a guessed/round value. Grading stricter than the
  route used manufactures phantom sub-clearance grazes. `check_drc.py
  --clearance-margin 0.1` filters ~grid-quantization noise (~8 µm artifacts).
- **Routing is deterministic, but outputs carry per-run random UUIDs.** Never hash
  or whole-file-`diff` `.kicad_pcb` outputs to judge determinism — compare
  `check_drc` / `check_connected` counts (stable run-to-run) instead.
- **`route.py` reads/writes a sibling `<output>.kicad_pro` DRC floor.** Re-running
  to the same output path reads it back and silently changes the routing (looks
  like non-determinism; it isn't). For clean A/B comparisons, route to a FRESH
  output path each run (or `rm` the `.kicad_pro` first).
- **Never `cp` a board without its `.kicad_pro` (#441).** The sibling `.kicad_pro`
  carries the DRC floor (the Default-netclass clearance/track/via the chain routed
  to). A bare `cp a.kicad_pcb b.kicad_pcb` strands it: the next route step reads no
  project, resolves its floor from the STOCK (looser) netclass, and stamps that over
  tighter copper — so KiCad grades correct sub-floor copper as phantom clearance DRC
  (icepi_zero: a dropped 0.09 floor became 0.10 → 160 phantom grazes). Use
  `python3 copy_board.py src.kicad_pcb dst.kicad_pcb` (copies `.kicad_pcb` + every
  sibling, self-records into the redo manifest), or copy the `.kicad_pro` too. The
  route scripts WARN when an input board has no sibling `.kicad_pro`.
- **Routers can report false success.** A router's own "routed" tally may come from
  a local/heuristic proxy while pads stay disconnected; re-verify with the
  authoritative, zone/fill-aware `check_net_connectivity` before trusting it.
- **Net classes are RESPECTED (PR392), and `--clearance` is a pure CEILING over ALL
  of them (#439).** The router honors KiCad's pairwise `max(classA, classB)` between
  nets of different classes — including copper routed earlier in the SAME call (in-run)
  — pricing each foreign obstacle at `config.obstacle_clearance(net_id)` (see
  `docs/api-routing-config.md`). `route.py` / `route_diff.py` / the fanout and plane
  scripts **always auto-read** every net's class clearance from the sibling `.kicad_pro`
  (override with `--net-clearances <json>`; all-Default boards are inert). **The
  PRESENCE of `--clearance` is the clamp switch, and there is nothing special about the
  Default class:**
  - **`--clearance` GIVEN** → it is a ceiling on *every* class (Default included): each
    net routes and grades at `min(its class, --clearance)` (the base/Default-net
    clearance is `min(Default class, --clearance)`; non-Default classes are capped in
    the map). A class tighter than `--clearance` survives; a looser one is capped. The
    output `.kicad_pro` writeback clamps every class DOWN to the routed floor so KiCad
    grades exactly what was routed.
  - **`--clearance` OMITTED** → no ceiling: each net routes at its OWN net-class
    clearance (base = the board's Default class, else `routing_defaults.CLEARANCE`
    0.25), and the writeback PRESERVES the classes. This is how you honor a genuine
    impedance board's class spec — just don't pass `--clearance`.
  - `--hole-to-hole-clearance` / `--board-edge-clearance` work the same way: omitted →
    the board's own `min_hole_to_hole` / `min_copper_edge_clearance` constraint (via
    `list_nets.board_constraint`), else the fixed default.
  **Why clamp on a ceiling:** stock net classes are largely *aspirational* — corpus and
  real boards route below them, and even the human-routed references violate their own
  class (zynq: 499 clearance violations at its 0.2 class, routed ~0.1), so keeping the
  stock class in the output manufactures phantom sub-class DRC on copper routed
  correctly at the fab floor. Helpers: `list_nets.board_default_netclass_clearance` /
  `board_constraint`; the GUI mirrors this with per-floor **override checkboxes** (Min
  Clearance / Min Hole-to-Hole / Min Edge Clearance — unchecked = use the board's own
  minimum, checked = clamp to the entered value). The old `--clamp-netclasses` **and**
  `--no-clamp-netclasses` flags are **removed** (the `--clearance` ceiling replaces
  both; `--net-clearances <json>` gives explicit per-net control). Grade multi-class
  boards at the netclasses that survived (`kicad_drc_compare._staged_copy`).

## Stress testing & A/B replay

Every recorded stress run leaves a `redo_commands.sh` manifest that replays the
full chain with **no LLM**. To regression-test or A/B an engine change across the
board corpus, use `tests/stress/ab_replay_grade.py` (whole-set replay + DRC/
connectivity grading) or `tests/stress/redo_diff_stage.py` (diff-pair stages only).
See `tests/stress/RUNBOOK.md` ("Replaying & A/B (no LLM)") for the recipes.

## Keep CLI and GUI routing in sync

There are two front-ends to the same routing engine, and a fix to one is
**not** automatically a fix to the other:

- **CLI scripts** — `route.py`, `route_diff.py`, `route_planes.py`,
  `route_disconnected_planes.py`, `bga_fanout.py`, etc. Their `main()`
  parses args and calls the shared engine functions (`batch_route`,
  `batch_route_diff_pairs`, `create_plane`, `generate_bga_fanout`, ...).
- **GUI plugin** — `kicad_routing_plugin/` (`swig_gui.py`,
  `differential_gui.py`, `planes_gui.py`, `fanout_gui.py`) calls those
  **same shared functions directly** (with `return_results=True`,
  `dry_run=True`/`output_file=""`), building `PCBData` from the live
  pcbnew board via `build_pcb_data_from_board` rather than parsing a file.

Because both call the shared engine, fixes **inside** those functions are
picked up by both for free. The gaps appear at the edges:

- **A fix in a CLI `main()` only** (argparse, defaults, output writing,
  post-processing) is invisible to the GUI — the GUI re-implements that
  layer. Put shared logic in the engine function, not in `main()`.
- **A new engine parameter / flag** must be threaded through *both* the
  argparse layer *and* every GUI call site (plus its config dict, the
  options panel, and `settings_persistence.py`). A new `batch_route`
  kwarg that only `route.py` passes silently does nothing in the GUI.
  It must also stay **Claude-settable end to end**: (1) the GUI plan
  executor (`claude_plan.py`) applies any snake_case param whose name
  matches a dialog control — so name the control after the param and add
  it to `reset_params_to_defaults` (the plan executor resets through
  that, or the param leaks between steps); (2) add the `--flag` →
  param-name mapping to `tests/stress/manifest_to_plan.py` `FLAG_PARAMS`
  so recorded manifests convert to `*_plan.json` with the param intact;
  (3) mirror it in `tests/gui_parity/test_gui_engine_parity.py`'s config
  map. Verify with: convert a manifest carrying the flag and check the
  plan JSON step params include it.
- **A changed default** must match in both places — the GUI sets its own
  values from UI controls and does not inherit argparse defaults.
- **Parser/obstacle/writer fixes** in shared low-level modules are used by
  both, *except* file-text-parsing fixes in `parse_kicad_pcb`: the GUI
  builds `PCBData` from pcbnew instead, so `build_pcb_data_from_board`
  must be kept at parity with the text parser separately.

- **A post-pass added to a CLI `main()`** (running *after* the shared engine
  call — cleanup, oracle recheck, DRC-floor writeback) is invisible to the
  GUI unless separately replicated (the set11 plane-shorts bug:
  `route_disconnected_planes.main()` ran `clean_plane_copper`, the planes tab
  didn't). Prefer putting the pass INSIDE the shared engine function; when it
  must operate on the written file, refactor a **board-level core** and call
  it from both fronts (as `compute_plane_copper_cleanup` now backs both
  `clean_plane_copper` and `planes_gui._run_plane_copper_cleanup`).

**Rule of thumb:** whenever you change routing behavior via the CLI, check
whether the corresponding GUI call site (and its options panel) needs the
same change — and vice versa. When adding a flag, grep the
`kicad_routing_plugin/` call sites for the function you changed and wire it
through there too.

**Parity gates (run these when touching CLI/GUI routing):**
- `tests/gui_parity/test_manifest_plan_parity.py` — no wx; asserts every CLI
  `--flag` survives `manifest_to_plan` into the GUI plan step (plan→params).
- `tests/gui_parity/test_cli_postpass_coverage.py` — no wx; asserts every CLI
  `main()` post-engine pass has a GUI counterpart, and blocks a new CLI-only
  post-pass (Class-2 drift). Register new passes there.
- `tests/gui_parity/test_gui_engine_parity.py` — needs KiCad python; runs the
  plan through the GUI engine path and grades against the CLI chain
  (`KICAD_DUMP_BATCH_KWARGS` diffs the 76-key param set).

**Tracking the last-audited commit:** `.gui-parity-checked` (repo root,
git-committed) holds the SHA of the last commit a full CLI/GUI parity audit
covered, plus the date and outcome. To bring it up to date: `git log
--oneline <that-sha>..HEAD` to see what's new, `git diff <that-sha>..HEAD --
<CLI scripts> kicad_routing_plugin/` to see the engine-side vs GUI-side
diffs, then check every new engine parameter/flag/results-data key against
the GUI call sites (per the rule of thumb above). When the audit finds and
fixes a gap, commit the fix first, note it in the file, then update the file
to current `HEAD` and commit that too — so the recorded SHA always reflects
"parity confirmed as of here," not "parity assumed."

## KiCad Parser Usage

Full user-facing API docs (parser, writer, modification, config, net
analysis, impedance) live in `docs/python-api.md` and the `docs/api-*.md`
pages — keep them in sync when changing these modules. The doc examples are
verified by `tests/run_doc_examples.py`. Quick reference:

The project uses `kicad_parser` module to parse KiCad PCB files:

```python
from kicad_parser import parse_kicad_pcb, Pad, Footprint, PCBData

pcb = parse_kicad_pcb('path/to/file.kicad_pcb')
```

### PCBData Structure

- `pcb.footprints` - Dict[str, Footprint] keyed by reference (e.g., 'U9', 'R1')
- `pcb.nets` - Dict[int, Net] keyed by net_id
- `pcb.segments` - List of track segments
- `pcb.vias` - List of vias
- `pcb.board_info` - BoardInfo (layers, bounds, stackup)

### BoardInfo / Stackup Attributes

- `pcb.board_info.copper_layers` - List[str] of copper layer names (e.g., ['F.Cu', 'B.Cu'])
- `pcb.board_info.layers` - Dict[int, str] layer_id -> layer_name
- `pcb.board_info.board_bounds` - (min_x, min_y, max_x, max_y) or None
- `pcb.board_info.stackup` - List[StackupLayer], ordered top to bottom
  (NOT `pcb.stackup`). Empty list if the board has no stackup section.
- StackupLayer fields: `name`, `layer_type` ('copper', 'core', 'prepreg', ...),
  `thickness` (mm), `epsilon_r`, `loss_tangent`, `material`

### Footprint Attributes

- `footprint.reference` - Component reference (e.g., 'U9')
- `footprint.footprint_name` - Footprint library name (e.g., 'interf_u:PGA120')
- `footprint.pads` - List[Pad] of pads
- `footprint.x`, `footprint.y` - Footprint position
- `footprint.rotation` - Rotation in degrees
- `footprint.layer` - Layer (e.g., 'F.Cu')
- `footprint.net_tie_groups` - List[List[str]] of pad-number groups the
  footprint deliberately shorts (`(net_tie_pad_groups "1, 2")`, Kelvin shunts /
  net-ties). KiCad's clearance exemption between the grouped pads is LOCAL:
  the tied net's copper may contact the partner pad only where the contact
  lies on its own pad. Consumers: `PCBData.net_tie_exempt_pad_ids(net_id)`,
  the obstacle builders (own-pad-sliver lift), and check_drc's waiver.

### Pad Attributes

- `pad.pad_number` - Pad identifier (e.g., 'H2', '1')
- `pad.net_id` - Net ID (int)
- `pad.net_name` - Net name (e.g., '/PC-A7')
- `pad.global_x`, `pad.global_y` - Absolute position
- `pad.local_x`, `pad.local_y` - Position relative to footprint
- `pad.size_x`, `pad.size_y` - Pad dimensions in board space (resolved from the
  pad's absolute angle; swapped for ~90° pads so they're axis-aligned)
- `pad.rect_rotation` - Residual rect tilt (deg, in (-90,90]); 0 for axis-aligned
  pads, non-zero only for pads on non-orthogonal angles. Obstacle/DRC geometry
  rotates the pad rectangle by this. Run `check_pads.py` before fanout to catch
  mis-modelled (overlapping) pad geometry.
- `pad.shape` - 'circle', 'oval', 'rect', etc.
- `pad.layers` - List of layer names
- `pad.drill` - Drill diameter (0 for SMD, >0 for through-hole)
- `pad.hole_x`, `pad.hole_y` - Drill/hole position when the pad copper is
  OFFSET from it (`(drill (offset x y))`, castellated-module paddles);
  `None` = hole at `global_x/global_y`. `global_x/global_y` is always the
  COPPER center (clearance/DRC/obstacle consumers use it directly); drill
  geometry must use `pad_drill_capsule`/`pad_drill_circles` or hole_x/y.
- `pad.pad_type` - 'smd', 'thru_hole', 'np_thru_hole', 'connect'. NPTH pads have
  NO copper even when `layers` lists `*.Cu` (size = mask opening only): skip them
  in copper-clearance logic, only their drill hole matters. For "does this pad's
  barrel tie copper layers together", use `pad_is_plated_through(pad)` — never
  bare `pad.drill > 0` (a net-tied NPTH mounting hole is not a connection, #328)
- `pad.component_ref` - Parent component reference
- `pad.pinfunction`, `pad.pintype` - Pin metadata
- `pad.local_clearance` - RESOLVED per-pad clearance override in mm (#326): the
  pad's own `(clearance ...)`, else the footprint-level override (recorded raw
  in `footprint.clearance`), else 0 (= global/netclass clearance applies).
  KiCad enforces max(the two items' clearances) per pair; the obstacle stamps
  and check_drc honor it the same way. Clearance consumers should read this
  field, never re-derive footprint inheritance.

### Through-Hole vs SMD Pads

- Through-hole pads (`pad.drill > 0`) block tracks on ALL layers
- SMD pads (`pad.drill == 0`) only block their specific layer
- Even unconnected through-hole pads (net_id=0) physically block tracks

### Net Attributes

- `net.net_id` - Net ID (int)
- `net.name` - Net name string
- `net.pads` - List[Pad] of connected pads

### Segment (Track) Attributes

- `segment.start_x`, `segment.start_y` - Start point
- `segment.end_x`, `segment.end_y` - End point
- `segment.width` - Track width
- `segment.layer` - Layer name
- `segment.net_id` - Net ID

### Via Attributes

- `via.x`, `via.y` - Position
- `via.size` - Via outer diameter
- `via.drill` - Drill diameter
- `via.layers` - Layer span
- `via.net_id` - Net ID

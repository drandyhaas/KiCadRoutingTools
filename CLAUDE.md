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
- **Routers can report false success.** A router's own "routed" tally may come from
  a local/heuristic proxy while pads stay disconnected; re-verify with the
  authoritative, zone/fill-aware `check_net_connectivity` before trusting it.

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
- **A changed default** must match in both places — the GUI sets its own
  values from UI controls and does not inherit argparse defaults.
- **Parser/obstacle/writer fixes** in shared low-level modules are used by
  both, *except* file-text-parsing fixes in `parse_kicad_pcb`: the GUI
  builds `PCBData` from pcbnew instead, so `build_pcb_data_from_board`
  must be kept at parity with the text parser separately.

**Rule of thumb:** whenever you change routing behavior via the CLI, check
whether the corresponding GUI call site (and its options panel) needs the
same change — and vice versa. When adding a flag, grep the
`kicad_routing_plugin/` call sites for the function you changed and wire it
through there too.

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
- `pad.pad_type` - 'smd', 'thru_hole', 'np_thru_hole', 'connect'. NPTH pads have
  NO copper even when `layers` lists `*.Cu` (size = mask opening only): skip them
  in copper-clearance logic, only their drill hole matters
- `pad.component_ref` - Parent component reference
- `pad.pinfunction`, `pad.pintype` - Pin metadata

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

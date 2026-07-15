# Python API Overview

The routing tools are built from plain Python modules that can be used as a
library — parse a board, query nets and footprints, add or remove copper, and
write a valid `.kicad_pcb` back out — without going through the `route.py` /
`route_diff.py` CLIs.

This page is the entry point. Each module has its own reference page with
complete signatures and runnable examples:

| Page | Modules | What it covers |
|------|---------|----------------|
| [KiCad Parser](api-kicad-parser.md) | `kicad_parser` | Reading `.kicad_pcb` files: pads, nets, tracks, vias, stackup, board outline, package detection |
| [KiCad Writer](api-kicad-writer.md) | `kicad_writer`, `output_writer` | Writing tracks, vias, and zones back to `.kicad_pcb` files; net/layer swaps; the routing output pipeline |
| [PCB Modification & Geometry](api-pcb-modification.md) | `pcb_modification`, `geometry_utils` | Adding/removing routes in parsed data, route cleanup, and geometry primitives (distances, intersections, UnionFind) |
| [Routing Configuration](api-routing-config.md) | `routing_config`, `routing_utils` | `GridRouteConfig` (every parameter), grid coordinate conversion, shared helpers |
| [Net Analysis](api-net-analysis.md) | `net_queries`, `connectivity` | Net filtering, differential pair detection, power net detection, MPS ordering, stub/connectivity queries |
| [Impedance](api-impedance.md) | `impedance` | Microstrip/stripline impedance formulas, width-for-impedance solving, propagation delay |

All examples run from the repository root against boards shipped in
`kicad_files/`.

## Conventions

These apply across every module:

- **Units are millimeters.** All coordinates, widths, clearances, and
  distances are mm unless a name says otherwise (e.g. `time_match_tolerance`
  is picoseconds). Angles are degrees.
- **Position matching uses `POSITION_DECIMALS`** (3 decimal places = 1 µm).
  When you build sets or dict keys from coordinates, round with
  `routing_utils.pos_key(x, y)` so lookups don't fail on floating-point noise.
- **Net IDs are integers; 0 means unconnected.** KiCad 9 files store numeric
  net IDs; KiCad 10 files store only names, so the parser assigns synthetic
  IDs and records the mapping in `pcb.net_id_to_name`. Code that writes files
  must pass that mapping for KiCad 10 boards (see
  [KiCad 9 vs 10](api-kicad-writer.md#kicad-9-vs-kicad-10)).
- **Layer names are strings** like `'F.Cu'`, `'In1.Cu'`, `'B.Cu'`. The A*
  router uses integer layer indices internally; convert with
  `routing_utils.build_layer_map(layers)`.
- **Through-hole pads (`pad.drill > 0`) block all copper layers.** SMD pads
  (`drill == 0`) only block the layers they are on. Even unconnected
  through-hole pads (`net_id == 0`) physically block tracks.

## Quickstart: parse → inspect → modify → write

The canonical library flow. Parse a board, look around, add a track and a
via, and write a new file KiCad can open:

```python
from kicad_parser import parse_kicad_pcb, KICAD_10_MIN_VERSION
from kicad_writer import add_tracks_and_vias_to_pcb

pcb = parse_kicad_pcb('kicad_files/kit-dev-coldfire-xilinx_5213.kicad_pcb')

# --- Inspect ---
print(f"Copper layers: {pcb.board_info.copper_layers}")
print(f"Board bounds:  {pcb.board_info.board_bounds}")
print(f"{len(pcb.nets)} nets, {len(pcb.footprints)} footprints, "
      f"{len(pcb.segments)} segments, {len(pcb.vias)} vias")

# Find a net by name and list its pads
gnd = next(net for net in pcb.nets.values() if net.name == 'GND')
print(f"GND is net {gnd.net_id} with {len(gnd.pads)} pads")

# --- Modify: draw a track on F.Cu and drop a via at its end ---
tracks = [{
    'start': (100.0, 100.0),
    'end': (105.0, 100.0),
    'width': 0.25,
    'layer': 'F.Cu',
    'net_id': gnd.net_id,
}]
vias = [{
    'x': 105.0, 'y': 100.0,
    'size': 0.6, 'drill': 0.3,
    'layers': ['F.Cu', 'B.Cu'],
    'net_id': gnd.net_id,
}]

# KiCad 10 files need the net-name mapping; KiCad 9 files must not get it
names = pcb.net_id_to_name if pcb.kicad_version >= KICAD_10_MIN_VERSION else None

add_tracks_and_vias_to_pcb(
    'kicad_files/kit-dev-coldfire-xilinx_5213.kicad_pcb',
    'quickstart_output.kicad_pcb',
    tracks, vias, net_id_to_name=names)
print("Wrote quickstart_output.kicad_pcb")
```

The output file opens in KiCad with the new copper present and correctly
netted.

## Which module do I want?

- **"What's on this board?"** → [`kicad_parser`](api-kicad-parser.md). One
  call (`parse_kicad_pcb`) gives you everything as plain dataclasses.
- **"Add copper to a file."** → [`kicad_writer`](api-kicad-writer.md) for
  dict-based tracks/vias/zones, or [`output_writer`](api-kicad-writer.md#output_writerpy)
  if you have full routing results with swaps.
- **"Add copper to the in-memory model"** (so later routing sees it) →
  [`pcb_modification.add_route_to_pcb_data`](api-pcb-modification.md#add_route_to_pcb_data).
- **"Which nets are diff pairs / power nets / unrouted?"** →
  [`net_queries`](api-net-analysis.md).
- **"Are these segments connected? Where do the stubs end?"** →
  [`connectivity`](api-net-analysis.md#connectivitypy).
- **"What track width for 50 Ω on this stackup?"** →
  [`impedance`](api-impedance.md).
- **"Configure a programmatic routing run."** →
  [`routing_config.GridRouteConfig`](api-routing-config.md).

For the internals of the router itself (obstacle maps, A*, rip-up), see
[Routing Architecture](routing-architecture.md).

## Routing-engine internals (brief)

These modules are not part of the parse → modify → write user API above, but
they are heavily imported across the engine and both front-ends, so they are
documented here for orientation. See [Routing Architecture](routing-architecture.md)
for how they fit together.

- **`obstacle_map`** — builds the per-net obstacle/cost grid the Rust A* routes
  on. `build_base_obstacle_map(pcb_data, config, exclude_net_ids)` is the entry
  point; the many `add_*`/`block_*` helpers layer on keepouts, board-edge and
  drill clearances, stub/pad/diff-pair blocking, and stub-proximity soft costs.
  Consumes `GridRouteConfig` clearances and emits a Rust `GridObstacleMap`.

- **`obstacle_cache`** — per-net incremental obstacle caching so a rip-up/retry
  doesn't rebuild the whole map. `precompute_net_obstacles` /
  `precompute_all_net_obstacles` build `NetObstacleData`;
  `add_net_obstacles_from_cache` / `remove_net_obstacles_from_cache` /
  `update_net_obstacles_after_routing` keep a working map in sync cell-for-cell
  (ref-counted). `precompute_via_placement_obstacles` does the same for via
  placement. `KICAD_OBSTACLE_LEDGER=1` audits add/remove balance via
  `run_obstacle_audit` / `obstacle_ledger_report`.

- **`fab_tiers`** — JLCPCB fab-capability floors as selectable cost tiers
  (issue #237). `fab_floor_ladder` / `fab_floors` / `fab_floor_for_param` give
  the minimum manufacturable value per parameter for a tier;
  `enforce_fab_floors` / `check_param_floors` clamp or reject below-floor
  params; `add_fab_tier_args` + `fab_tier_from_args` /
  `set_fab_tier_from_config` thread the tier through the CLI and GUI;
  `parse_fab_overrides` reads a `--fab-overrides` file.

- **`clearance_ledger`** — process-scoped bridge for the smallest copper
  clearance any step actually used, so a board routed with a deep sub-nominal
  step is graded (and its `.kicad_pro` DRC floor set) at that clearance rather
  than the nominal one. `record(clearance)` at sub-nominal sites; `get_min()` /
  `effective(nominal)` read once in each `main()`; `reset()` (or the
  `fresh_run()` context manager) starts a clean operation in a long-lived
  process like the GUI plugin.

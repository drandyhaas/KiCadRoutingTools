# Rust Grid Router

The compiled half of KiCad Routing Tools: the Python extension module
`grid_router`, built from this crate with PyO3. It holds the router's A*
searches and obstacle map, the placer's per-move kernels, and the grid searches
of the `awx/` whole-route bus router. The rest is Python that calls in here.

**Current Version: 0.23.7** (the crate version in `Cargo.toml`)

> **Release note:** the newest published release is
> [v0.23.7](https://github.com/drandyhaas/KiCadRoutingTools/releases/tag/v0.23.7)
> (2026-10-23), and its binaries are crate 0.23.7 (the placement kernels).
>
> A release's binaries are built from the crate at its tag, and `/VERSION` can
> lead `Cargo.toml` (a Python-only release bumps only `/VERSION`), so a release
> can carry an older crate than its own number:
>
> | Release | Crate binaries |
> |---|---|
> | v0.23.7 | 0.23.7 |
> | v0.23.1 | 0.23.1 |
> | v0.23.0 | 0.23.0 |
> | v0.22.0, v0.22.1 | 0.22.0 |
> | v0.21.5 | 0.21.4 |
> | v0.21.1 to v0.21.4 | 0.21.1 |
> | v0.20.2 to v0.20.4 | 0.20.1 |
> | v0.20.0, v0.20.1 | 0.20.0 |

## Features

Routing (`GridObstacleMap`, `GridRouter`, `PoseRouter`), about 10x faster than
the original pure-Python A* it replaced (measured then; there is no Python A* left to compare with):

- Grid A* with octilinear moves (8 directions), multi-source and multi-target, with via cost and layer transitions
- **Per-layer costs** (`layer_costs`) to prefer some layers (route.py's default: all 1.0x on 4+ layers, else F.Cu 1.0x and the others 3.0x); a negative entry makes a layer FORBIDDEN -- never routed on, still an obstacle, spanned by through-vias (#206)
- **Per-layer direction preferences**: a penalty on progress along a layer's non-preferred axis, diagonals included (#452), in both routers (#658)
- **Turn cost** proportional to the turn angle, for straighter paths with fewer wiggles
- **Proximity costs**: stub proximity, per-layer track proximity, BGA exclusion zones and their proximity rings, and an optional proximity term in the heuristic for dense boards
- **Cross-layer track attraction** ("vertical alignment"): routes stack over tracks on other layers, leaving room for vias
- **Bus path attraction**: a member follows an earlier member's path, by a direction-aware step discount and an optional potential-shaped term (#656)
- **Wide tracks**: an exact swept-capsule clearance check with a fractional, per-layer margin (#156/#173)
- **Via legality per via size** ("rungs", #568/#530), and **free vias**: an existing same-net hole is reused at no cost
- **Pose-based routing with a Dubins heuristic** for differential-pair centerlines: headings at both ends, P/N via spacing, straight runs around vias, GND-via sites, a cumulative-turn limit
- **Frontier analysis**: a failed search returns the cells that stopped it, and `identify_blocking_obstacles` maps those cells to the nets that own them, for rip-up
- A reference-counted obstacle map that nets are stamped into and removed from incrementally, a bitmap for permanent board geometry (#422), and column-span batch stamping (#815)

Placement (`placement_kernels.rs`): the airwire trees, crossings, foreign-pad
hits, group gaps, courtyard overlaps and congestion that the placer prices for
every candidate move.

Whole-route bus router (`awx/`): the snap step's grid search for a single lane
and for a pair (`lane_search`, `pair_search`).

## Installing

Most users do **not** build this themselves. From the project root:

```bash
python3 build_router.py
```

It installs `rust_router/grid_router.so` (`grid_router.pyd` on Windows) and
checks that it imports, in a fresh interpreter. In order:

1. If this checkout's `rust_router/` differs from `origin/main` (committed
   branch changes, local edits or untracked files; outside a git checkout this
   step is skipped), it builds from source:
   release binaries are built from main's crate, and one could carry the right
   version string yet take different arguments (#615).
2. Otherwise it looks up the latest GitHub Release. If `Cargo.toml` is newer
   than that release, no published binary can match, so it builds from source.
   Otherwise it downloads the binary for your platform (Linux x86_64, macOS
   arm64 or x86_64, Windows x86_64) from
   [GitHub Releases](https://github.com/drandyhaas/KiCadRoutingTools/releases).
   Prebuilts use PyO3's `abi3-py39`, so one binary works for any Python 3.9+.
3. If the download fails, or the downloaded binary's version is not the one in
   `Cargo.toml` (a stale release asset), it builds from source.

```bash
python3 build_router.py --from-source   # skip the download and build locally
python3 build_router.py --tag v0.23.7   # download that release (skips 1 and 2; 3 still applies)
python3 build_router.py --clean         # remove target/ and every grid_router.so/.pyd copy
```

## Building from source

`--from-source`, and every fallback above, runs `cargo build --release` in
`rust_router/` and copies the library to where Python imports it:

| Platform | Built | Installed as |
|---|---|---|
| Linux | `target/release/libgrid_router.so` | `rust_router/grid_router.so` |
| macOS | `target/release/libgrid_router.dylib` | `rust_router/grid_router.so` |
| Windows | `target/release/grid_router.dll` | `rust_router/grid_router.pyd` |

It also deletes stale `grid_router.*` copies from the project root, which would
shadow this one, and checks the import. Use it rather than running cargo by hand.

### Prerequisites

**Windows:**
1. Install Rust from https://rustup.rs/
2. Install Visual Studio Build Tools:
   - Download from: https://visualstudio.microsoft.com/visual-cpp-build-tools/
   - Run the installer and select **"Desktop development with C++"** workload
   - This installs the MSVC compiler and linker required by Rust

**Linux/macOS:**
1. Install Rust from https://rustup.rs/
2. Ensure you have a C compiler (gcc/clang) installed

## Releases (maintainers)

The checklist is [docs/release-pipeline.md](../docs/release-pipeline.md). In
short: pushing a `vX.Y.Z` tag runs `.github/workflows/release.yml`, which builds
the four binaries from the tagged crate (macOS x86_64 cross-compiled on the
arm64 runner) and attaches them to the GitHub Release that `build_router.py`
downloads from. A crate change bumps `Cargo.toml` and adds an entry to the
[Version History](#version-history), and the release that ships it sets
`/VERSION` and `metadata.json` to the same version; a Python-only release bumps
only `/VERSION` and republishes the current crate's binaries.

## Version check at startup

`route.py`, `route_diff.py`, `route_planes.py` and `repair_planes.py` run
`py_router/startup_checks.py` as they start. It puts `rust_router/` first on
`sys.path`, imports `grid_router`, and requires its `__version__` to equal the
version in `Cargo.toml` (it also checks numpy, scipy and shapely against
`requirements.txt`). A missing or mismatched module stops the run with the
reason and `python3 build_router.py`; nothing is rebuilt automatically.
`install_plugin.py` makes the same version check, and the plugin's About tab
shows the version it loaded.

The crate's global allocator is mimalloc. The routing modules import
`py_router/rust_alloc.py` before `grid_router`; it sets `MIMALLOC_PURGE_DELAY=0`
(unless already set) so the memory of discarded obstacle maps goes back to the
OS (#419).

## Usage from Python

```python
import sys
sys.path.insert(0, "rust_router")   # from the project root
import grid_router
from grid_router import GridObstacleMap, GridRouter

# Check version
print(f"grid_router version: {grid_router.__version__}")

# Create obstacle map with 4 layers
obstacles = GridObstacleMap(4)

# Add blocked cells (from segments, pads, etc.)
obstacles.add_blocked_cell(100, 200, 0)  # gx, gy, layer
obstacles.add_blocked_via(150, 250)       # gx, gy

# Set stub proximity costs (optional - discourages routing near unrouted stubs)
obstacles.set_stub_proximity(180, 190, 3000)  # gx, gy, cost

# Set BGA exclusion zone (optional - blocks routing through this area)
obstacles.set_bga_zone(1859, 935, 2049, 1125)  # min_gx, min_gy, max_gx, max_gy

# Add allowed cells that override BGA zone blocking (for source/target stubs inside BGA)
obstacles.add_allowed_cell(1900, 1000)  # gx, gy

# Create router
router = GridRouter(via_cost=500000, h_weight=1.5)

# Route from sources to targets
sources = [(100, 200, 0), (101, 200, 0)]  # (gx, gy, layer)
targets = [(300, 400, 0), (301, 400, 0)]

path, iterations, stats = router.route_multi(obstacles, sources, targets, max_iterations=200000)

if path:
    print(f"Found path with {len(path)} points in {iterations} iterations")
    for gx, gy, layer in path:
        print(f"  ({gx}, {gy}) on layer {layer}")
    # stats contains A* search statistics (cells_expanded, heuristic_ratio, etc.)
else:
    print(f"No path found after {iterations} iterations")
```

The path has one point per grid step. Real callers stamp many cells at once
with the batch methods, which take numpy `int32` arrays, e.g.
`obstacles.add_blocked_cells_batch(np.array(cells, dtype=np.int32))`.

## Microbenchmark

```bash
python3 rust_router/microbench.py              # 9 repeats, 400 x 400 maze, 2 layers
python3 rust_router/microbench.py --repeats 5 --size 500
```

It times `route_with_frontier` through a serpentine maze (walls every
`--wall-pitch` cells with a `--gap`-cell opening alternating top and bottom, on
`--layers` layers) and prints the iterations and the fastest of the repeats;
building the map is outside the timer. For a change meant to leave routing
byte-identical, the printed `iterations` must be the same for both binaries;
only the time may differ.

## API Reference

Everything the module exports:

```bash
python3 -c "import sys; sys.path.insert(0, 'rust_router'); import grid_router; print(grid_router.__version__, [n for n in dir(grid_router) if not n.startswith('_')])"
```

Grid coordinates are integer cells and layers are indices into the routing
layer list. Costs are integers scaled by 1000: an orthogonal step costs 1000, a
diagonal 1414.

### GridObstacleMap

```python
obstacles = GridObstacleMap(num_layers)   # obstacles.num_layers is read-only
```

Batch methods take numpy `int32` arrays (a list or an `int64` array is
refused); the rest take ints and lists of tuples. Used by `py_router/`
(`obstacle_map.py`, `obstacle_cache.py`, `obstacle_costs.py`, the plane
builders and every router).

**Blocking.** Cells and vias are reference-counted, so nets can be stamped and
removed independently: a cell is blocked while its count is above zero.
- `add_blocked_cell(gx, gy, layer)`, `add_blocked_via(gx, gy)`
- `add_blocked_cells_batch(cells)` / `remove_blocked_cells_batch(cells)`: N x 3 (gx, gy, layer)
- `add_blocked_vias_batch(vias)` / `remove_blocked_vias_batch(vias)`: N x 2 (gx, gy)
- `add_blocked_cell_spans_batch(spans)` / `remove_blocked_cell_spans_batch(spans)`: N x 4 (gx, y_lo, y_hi, layer), both y bounds inclusive -- the same cells as the cell form, one row per column
- `add_blocked_via_spans_batch(spans)` / `remove_blocked_via_spans_batch(spans)`: N x 3 (gx, y_lo, y_hi)
- `merge_blocked_from(other)`: add another map's counts to this one

**Permanent geometry** (#422): one bit per cell, never removed -- the board
edge band, outside the outline, cutouts.
- `add_static_blocked_cell(gx, gy, layer)`, `add_static_blocked_cells_batch(cells)`, `add_static_blocked_cell_spans_batch(spans)`
- `add_static_blocked_via(gx, gy)`, `add_static_blocked_vias_batch(vias)`, `add_static_blocked_via_spans_batch(spans)`
- `freeze_dynamic_to_static()`: move every counted cell and via into the permanent bitmaps (for a base map that is never edited again)

**Via sizes ("rungs", #568/#530).** Rung 0 is the configured via; rung r >= 1
is another via size with its own counted map. A rung nobody populated answers
as rung 0.
- `add_blocked_via_rung(rung, gx, gy)`, `remove_blocked_via_rung(rung, gx, gy)`
- `add_blocked_vias_rung_batch(rung, vias)`, `remove_blocked_vias_rung_batch(rung, vias)`
- `is_via_blocked_rung(gx, gy, rung)`, `rung_count()`, `rung_len(rung)`, `blocked_via_cells_at_rung(rung)` (sorted list of (gx, gy))
- `add_blocked_via_small(gx, gy)`, `remove_blocked_via_small(gx, gy)`, `add_blocked_vias_small_batch(vias)`, `remove_blocked_vias_small_batch(vias)`: the rung-1 forms

**Queries**
- `is_blocked(gx, gy, layer)`, `is_via_blocked(gx, gy)`
- `segment_blocked(gx1, gy1, gx2, gy2, layer, r)`: is any blocked cell centre within `r` cells of the segment (the wide-track swept-capsule test; `r <= 0` tests the end cell only)
- `is_blocked_with_margin(gx, gy, layer, margin)`: any blocked cell in the square of half-side `margin` (what `segment_blocked` replaced)
- `open_via_cells_within(cx, cy, radius)`: every via-legal cell within Chebyshev `radius`, nearest first (the plane via-site search)

**Per-route overrides**
- `set_bga_zone(min_gx, min_gy, max_gx, max_gy)`: add a BGA exclusion zone (several allowed)
- `add_allowed_cell(gx, gy)`, `add_allowed_rect(min_gx, min_gy, max_gx, max_gy)` (inclusive; an inverted range adds nothing), `clear_allowed_cells()`: cells a BGA zone does not block
- `add_source_target_cell(gx, gy, layer)`, `clear_source_target_cells()`: endpoint cells that can be routed to through ordinary blocking (not through a BGA zone)
- `add_free_via(gx, gy)`, `add_free_vias_batch([(gx, gy), ...])`, `is_free_via(gx, gy)`, `clear_free_vias()`: existing same-net holes, where a layer change costs nothing and via blocking does not apply

**Proximity costs**
- `set_stub_proximity(gx, gy, cost)`, `get_stub_proximity_cost(gx, gy)`, `clear_stub_proximity()`
- `add_stub_proximity_costs_batch(stubs, radius, max_cost)`: a linear falloff from `max_cost` at each (gx, gy) to 0 at `radius`, keeping the larger value where fields overlap
- `add_stub_proximity_costs_grouped(groups, max_zone_rects=None)`: groups of `(points, radius, max_cost)`, the larger value within a group and the sum across groups (the larger inside `max_zone_rects`); what `KICAD_PROXIMITY_SUM` uses
- `set_layer_proximity(gx, gy, layer, cost)`, `set_layer_proximity_batch(costs)` (N x 4: layer, gx, gy, cost), `get_layer_proximity_cost(gx, gy, layer)`, `clear_layer_proximity()`
- `set_endpoint_exempt([(gx, gy), ...], radius)`, `clear_endpoint_exempt()`: no stub proximity cost near this route's own ends
- `set_bga_proximity_radius(radius)`, `is_in_bga_proximity(gx, gy)`, `is_in_any_proximity_zone(gx, gy)` (stub cost here, or within the BGA ring)

**Cross-layer attraction**
- `add_cross_layer_track(gx, gy, layer)`, `clear_cross_layer_tracks()`
- `get_cross_layer_attraction(gx, gy, current_layer, attraction_radius, attraction_bonus)`: the bonus for tracks on OTHER layers near (gx, gy); none in stub-proximity or BGA zones
- `build_attraction_field(attraction_radius, attraction_bonus)`: precompute that bonus per layer so the lookup is O(1) (boards of up to 8 layers)

**Copies and statistics**
- `clone()`, `clone_fresh()` (a copy without the source/target cells), `shrink_to_fit()`
- `get_stats()`: (blocked cells, blocked vias, stub-proximity cells, layer-proximity cells, cross-layer cells, source/target cells, free vias, via-rung cells)
- `get_static_stats()`: (static cells, static vias); `dynamic_refcount_stats()`: (distinct cells, cells counted >= 2, highest count, distinct vias, vias counted >= 2)

The Python side does not call `merge_blocked_from`, `is_blocked_with_margin`,
`is_in_any_proximity_zone`, `build_attraction_field` or
`dynamic_refcount_stats` at present.

### GridRouter

The single-ended A*. Used by `single_ended_routing.py`, `route.py`,
`route_diff.py`, `route_planes.py`, `plane_region_connector.py` and
`diff_pair_routing.py`.

```python
router = GridRouter(via_cost, h_weight, turn_cost=None, via_proximity_cost=1,
                    vertical_attraction_radius=0, vertical_attraction_bonus=0,
                    layer_costs=None, proximity_heuristic_cost=None,
                    layer_direction_preferences=None, direction_preference_cost=0,
                    attraction_radius=0, attraction_bonus=0,
                    attraction_cross_layer_pct=0, attraction_potential=0)
```

Parameters:
- `via_cost`: cost of a layer change
- `h_weight`: heuristic weight (>1 for faster but less optimal routes)
- `turn_cost`: the cost of a 90-degree turn (None = 1000); a 45-degree turn costs half, a 135-degree turn 1.5x, a reversal 2x
- `via_proximity_cost`: multiplier on the proximity cost (stub + destination-layer) at a via's cell, added to the via cost; 0 = no extra cost
- `vertical_attraction_radius`, `vertical_attraction_bonus`: cross-layer track attraction (0 = off); a step never drops below a tenth of its move cost
- `layer_costs`: per-layer multipliers (1000 = 1.0x, 3000 = 3.0x) on moves, on starting on a layer and on vias (a via onto a cheaper layer is discounted, down to free); a negative entry forbids the layer
- `proximity_heuristic_cost`: expected proximity cost per grid step added to the heuristic (None = 0, off). Python sets it before each route with `set_proximity_heuristic_cost`, from the proximity zones the route's ends lie in (`GridRouteConfig.get_proximity_heuristic_for_zones`: each such zone's cost times its radius, summed, times `--proximity-heuristic-factor`); a route with both ends outside every zone gets 0
- `layer_direction_preferences`: per layer 0 = horizontal, 1 = vertical, 255 = none; `direction_preference_cost`: charged per move that advances along the non-preferred axis, diagonals included (0 = off)
- `attraction_radius`, `attraction_bonus`: bus path attraction to the path given by `set_attraction_path` -- a discount on moves in the same direction as the path near it, `attraction_bonus / 50` percent at full strength, capped at 90% (0 = off)
- `attraction_cross_layer_pct`: percent of that attraction granted on layers other than the path's (0 = same layer only)
- `attraction_potential`: strength of the potential-shaped attraction (#656): moves are charged the change in a potential that peaks on the path, so cycles never pay and a via onto the path's layer earns a one-time gain (0 = off)

Methods:
- `route_multi(obstacles, sources, targets, max_iterations, collinear_vias=False, via_exclusion_radius=0, start_direction=None, end_direction=None, direction_steps=2, track_margin=0.0, max_iterations_ceiling=0, quantum_cells=2.0, quantum_pct=2.0, grace_tranches=0, via_rung=0)` -- a path from any source to any target; sources and targets are lists of (gx, gy, layer)
  - Returns `(path, iterations, stats)`: `path` a list of (gx, gy, layer) or None; `stats` a dict: `cells_expanded`, `cells_pushed`, `cells_revisited`, `duplicate_skips`, `path_length`, `path_cost`, `initial_h`, `best_h`, `iteration_tranches`, `final_g`, `open_set_size`, `closed_set_size`, `via_count`, and when defined `heuristic_ratio`, `expansion_ratio`, `revisit_ratio`, `skip_ratio`
  - `collinear_vias`: symmetric geometry around each via, `±45° -> D -> via -> D -> ±45°`, with at least 2 steps before it
  - `via_exclusion_radius`: cells kept clear around the route's own vias
  - `start_direction` (dx, dy): the first `direction_steps` moves stay within 45 degrees of it; `end_direction` (float dx, dy): the arrival stays within 60 degrees of it
  - `track_margin`: extra half-width of a wide track in fractional cells, a float or a per-layer list (#156); above 0 the swept-capsule test applies
  - `max_iterations_ceiling`: above `max_iterations`, the search earns further `max_iterations` tranches up to the ceiling while it keeps closing in by `max(quantum_cells, quantum_pct% of the remaining estimate)` (#529; 0 = off); `grace_tranches` tolerates that many tranches without progress (0 = off; see 0.19.3)
  - `via_rung`: which via size's legality to search with (0 = the configured via)
- `route_with_frontier(...)`: the same arguments; returns `(path, iterations, blocked_cells)`, where `blocked_cells` (a list of (gx, gy, layer) that stopped the search) is empty on success
- `set_proximity_heuristic_cost(cost)`: set before each route
- `set_attraction_path([(gx, gy, layer), ...])`, `clear_attraction_path()`: the path bus members are attracted to (an empty list clears it)

### PoseRouter

Orientation-aware A* using the Dubins path length as its heuristic, for
differential-pair centerlines whose start and end headings are fixed by the
stubs. Used by `diff_pair_routing.py` and `diff_pair_custody.py`.

```python
router = PoseRouter(via_cost, h_weight, turn_cost, min_radius_grid,
                    via_proximity_cost=10, diff_pair_spacing=0, max_turn_units=4,
                    gnd_via_perp_offset=0, gnd_via_along_offset=0,
                    vertical_attraction_radius=0, vertical_attraction_bonus=0,
                    proximity_heuristic_cost=0, layer_costs=None,
                    layer_direction_preferences=None, direction_preference_cost=0)
```

Parameters:
- `via_cost`, `h_weight`: as for GridRouter
- `turn_cost`: cost of a 45-degree turn (typically `min_radius * pi/4 * 1000`)
- `min_radius_grid`: minimum turning radius in grid units (the Dubins heuristic, and the straight run around vias)
- `via_proximity_cost`: as for GridRouter -- (stub + destination-layer proximity) x this, added to the via cost; 0 = no extra cost
- `diff_pair_spacing`: P/N spacing in grid units; above 0 the cumulative-turn limit applies
- `max_turn_units`: that limit, in 45-degree units, so a pair cannot loop
- `gnd_via_perp_offset`, `gnd_via_along_offset`: where GND vias go beside each signal via pair (grid units; 0 = no GND vias)
- `vertical_attraction_radius`, `vertical_attraction_bonus`, `layer_costs`, `layer_direction_preferences`, `direction_preference_cost`: as for GridRouter
- `proximity_heuristic_cost`: as for GridRouter; Python passes a tenth of the single-ended value, the pose search being more constrained

Methods:
- `route_pose(obstacles, src_x, src_y, src_layer, src_theta_idx, tgt_x, tgt_y, tgt_layer, tgt_theta_idx, max_iterations, diff_pair_via_spacing=None)`
  - Headings are direction indices 0-7: 0 = East, 1 = NE, 2 = North, ... 7 = SE (y grows downward, so North is -y)
  - `diff_pair_via_spacing`: grid units; a via is placed only where the P and N via positions (perpendicular offsets from the centerline) are clear too
  - Returns `(path, iterations, gnd_via_directions)`: `path` a list of (gx, gy, theta_idx, layer) or None; `gnd_via_directions` one entry per layer change, 1 = GND vias ahead, -1 = behind
- `route_pose_with_frontier(...)`: the same arguments; returns `(path, iterations, blocked_cells, gnd_via_directions)`, `blocked_cells` filled only on failure
- `set_proximity_heuristic_cost(cost)`: set before each route

The search's rules:
- **Turns are moves**: a turn changes the heading by 45 degrees while stepping (no turning in place) and pays `turn_cost`; the first move from the start goes straight on its heading
- **Straight around vias**: a via needs at least 2 steps from the start and `straight_after_via` straight steps before it, and owes that many straight steps after it -- `max(ceil(min_radius_grid) + 1, 3)`, or `gnd_via_along_offset + 2` if larger -- so the P/N tracks clear the vias before turning
- **Diff-pair via clearance**: with `diff_pair_via_spacing`, both P/N via sites must be via-legal; with GND vias on, the GND sites ahead of the via or, failing that, behind it must be too
- **Cumulative turn**: for a pair, the turn summed over each of two staggered 100-step windows stays within `max_turn_units` x 45 degrees
- **Via proximity**: a via in a proximity zone is priced, never banned

The Dubins heuristic is the shortest path length between the start and end
poses under the minimum turning radius, which keeps routes smooth at the pads'
entry and exit angles.

### identify_blocking_obstacles

```python
counts = identify_blocking_obstacles(blocked, segments, vias, pads,
                                     expansion_grid, via_expansion_grid, num_layers)
```

Which nets' copper touches the cells a failed search was stopped by, as
`{net_id: count}`: each segment, via or pad counts once for its net. The arrays
are numpy `int64`: `blocked` N x 3 (gx, gy, layer), `segments` M x 6 (gx1, gy1,
gx2, gy2, layer, net_id), `vias` V x 3 (gx, gy, net_id), `pads` P x 6 (gx, gy,
expansion x, expansion y, net_id, layer bitmask), all foreign to the net being
routed. Used by the rip-up blocking analysis in `single_ended_routing.py`.

### Placement kernels

Called by `py_placer/placement/` for every candidate move. Each computes the
Python expression its docstring names, with the same tie rules; arrays are
numpy `float64` (labels, owners and net ids `int64`, `pad_use`'s `on` bool). The placer has no Python
fallback, so placement needs a binary that has all of them.

| Function | Computes | Used by |
|---|---|---|
| `crossing_pairs(a, b, net_w=None)` | The airwire pairs (rows x1, y1, x2, y2, net) of `a` and `b` that cross -- never two of one net, never two sharing an end -- as (ii, jj, w), `w` the larger `net_w` of the two | `quench.py` |
| `mst_prim(xs, ys)` | `compute_mst_edges`' Euclidean Prim tree in O(n) memory, as [(i, j, d)] | `quench.py` |
| `mst_l1(xs, ys)` | A net's Manhattan Prim tree: (total, [(i, j)]) | `simple.py` |
| `mst_l1_tree(xs, ys, owners)` | The same over sorted pad points, with each edge's end parts: (total, edges m x 4, ends m x 2); over 160 points by octant neighbours and Kruskal | `simple.py` |
| `gap_tree(rects)` | A group's Prim tree over courtyard gaps: (total, [(i, j, gap)]) | `simple.py` |
| `seg_rect_hits(s, p)` | Segments that touch rects (box test, then Liang-Barsky): (ii, jj) | `simple.py` |
| `seg_crossings(a, b=None, labels=None)`, `seg_cross_pairs(a, b=None, labels=None)` | Proper crossings between two segment sets, or within one across `labels`: the count, or (ii, jj) | `simple.py` |
| `seg_cross_counts(m, mlab, o=None, olab=None, skip=None)` | Crossings tallied per pair of labels: (pairs, [(a, b, count)]) | `simple.py` |
| `pad_hit_sums(s, ends, labs, p, w, pnets, owners, skip_nets=None, skip_owners=None)` | Tree edges over foreign pads, pad weight summed per (net, owner) | `simple.py` |
| `net_box(e, length, h, ex, ey, gx0, gy0, cell, nx, ny)` | A net's congestion demand on the grid, as its two factors | `simple.py` |
| `pad_use(a, on, ex, ey, gx0, gy0, cell, nx, ny)` | A part's pad copper per layer on the congestion grid | `simple.py` |
| `cong_delta(d, c, uses, win, dem, usev, cell2, pitch)` | The change a move makes to the congestion overflow total, over the cells it reaches | `simple.py` |
| `cong_block_slack(d, c, pitch, block, win, out)`, `cong_delta_blocks(d, c, uses, win, dem, usev, cell2, pitch, slack, block)` | Each block's least slack, and `cong_delta` visiting only the blocks that can overflow | `simple.py` |
| `overlap_sum(total, h, mine, live, crects, faces)` | The courtyard overlap of the moving parts against every other part, added to `total` | `simple_anneal.py` |

### lane_search, pair_search

```python
path, expanded = lane_search(band, bad, vbad, arc, dist, dv, s, e, d0, dn, lays, gate, room0,
                             arc_hi, rvia, w_via, w_dev, w_bend, g, tw, step,
                             start_ok, end_ok, out_ok, in_ok, strict)
path, expanded, farthest, state = pair_search(band, bad, vbad, arc, dist, tol, soft, dv, starts, ends,
                                              multi, lays, gate, room0, room1, total, rvia, w_via,
                                              w_dev, w_bend, w_keep, w_share, g, step, cap, rt, st,
                                              pr, cross, xs, xo)
```

The whole-route snap's grid searches (`awx/whole_snap.py`) for a single lane and
for a pair, ported so exactly that the snapped plan is byte-identical to the
Python search's. `whole_snap` searches in Python when the loaded binary lacks
them. The parameters are described at the top of `src/lane_search.rs` and
`src/pair_search.rs`.

## Architecture

### Source Files

```
src/
├── lib.rs                # module registration, identify_blocking_obstacles, the mimalloc allocator
├── types.rs              # state keys, open-set entries, DIRECTIONS, costs, search sinks, RouteStats
├── obstacle_map.rs       # GridObstacleMap
├── router.rs             # GridRouter: the grid A* (one GridSearch core, tiled NodeStore)
├── dubins.rs             # Dubins path length, the PoseRouter heuristic
├── pose_router.rs        # PoseRouter: orientation-aware A* for diff-pair centerlines
├── placement_kernels.rs  # the placer's per-move kernels
├── lane_search.rs        # awx whole-route snap: a single lane's grid search
└── pair_search.rs        # awx whole-route snap: a pair's grid search
```

### Key Components

- **GridObstacleMap**: reference-counted FxHashMaps (cells per layer, vias, one per via rung) are the truth; a per-layer bitmap caches "count above zero" for the hot `is_blocked` test, and permanent geometry is held only in static bitmaps
- **GridRouter**: one `GridSearch` core behind both entry points (`route_multi` collects stats, `route_with_frontier` collects the frontier); a binary heap, and nodes in a tiled store (64 x 64-cell tiles, 12 bytes a node, the closed flag in the parent index's top bit)
- **PoseRouter**: one `pose_search` core behind `route_pose` / `route_pose_with_frontier`, each pose's state in one FxHashMap entry
- **DubinsCalculator**: Computes shortest Dubins path length (LSL, RSR, LSR, RSL, RLR, LRL)
- **State keys**: packed into u64 for fast hashing (grid: 20 bits x, 20 bits y, 8 bits layer; pose: 19 bits x, 19 bits y, 3 bits heading, 8 bits layer)
- **Hash function**: Uses rustc-hash (FxHash) for faster integer hashing than default SipHash
- **Costs**: ORTHO_COST=1000, DIAG_COST=1414 (sqrt(2) * 1000), DEFAULT_TURN_COST=1000
- **lane_search / pair_search**: each orders its heap as Python's tuples order, so the same states are expanded in the same order as the Python search
- **Placement kernels**: the trees and gap trees keep the Python's float expressions in order; the crossing and pad-hit kernels skip pairs whose boxes cannot meet; the congestion kernels sum a move's change over the cells it reaches, so their totals can differ from a whole-window Python sum in the last bits

## Version History

### 0.23.7 (2026-10-18)
- **Congestion priced only where it can change** (`placement_kernels.rs`):
  `cong_block_slack(d, c, pitch, block, win, out)` keeps each 8 x 8 block's
  least slack (capacity less demand), and `cong_delta_blocks(..., slack,
  block)` is `cong_delta` to the bit, visiting only the blocks where a cell
  can overflow before or after the move; the rest add nothing to the
  overflow total either way.

### 0.23.6 (2026-10-18)
- **The simple path's overlap in Rust** (`placement_kernels.rs`):
  `overlap_sum(total, h, mine, live, crects, faces)` adds the overlap of the
  moving parts against every other part (their committed rects, grown by
  half the courtyard gap, faces that meet only) to `total`, in the order the
  Python loop added it, so the same value to the bit.
- `mst_l1_tree` builds the tree of a net with more than 160 points by an
  octant sweep over a Fenwick tree instead of a BTreeMap, candidates sorted
  on their lengths' bits: the same lengths summed in the same ascending
  order, so the same total to the bit, 2.8x faster at 340 points (111 to
  40 us).
- `pad_hit_sums` also drops the segments that miss the rects' union box, and
  buckets the rects on a grid when there are many pairs: the same tallies.

### 0.23.5 (2026-10-18)
- **Congestion priced where a move reaches** (`placement_kernels.rs`):
  `net_box(...)` gives a net's congestion share as its two factors (the
  cells' overlap with its box along y and x, and its length over the box's
  area), `pad_use(a, on, ...)` a part's pad copper per layer, and
  `cong_delta(d, c, uses, win, dem, usev, cell2, pitch)` the change a move
  makes to the overflow total, summed over the cells its terms reach. They
  replace `over2` and `net_demand`.
- `pad_hit_sums` and `seg_cross_counts` test each rect or committed edge
  against the union box of the moving segments before testing pairs, and
  `seg_cross_counts` skips pairs whose boxes do not meet.
- `mst_l1_tree` builds the tree of a net with more than 160 points (a
  ground with hundreds of pads) by Kruskal over each point's nearest
  neighbour per octant, O(n log n) where Prim is O(n^2): the same length,
  other ties (1200 points: 0.56 ms against 2.2 ms).

### 0.23.4 (2026-10-18)
- **More simple-path kernels** (`placement_kernels.rs`), so a move of the
  annealer in `py_placer/placement/simple_anneal.py` is priced mostly in Rust:
  `gap_tree(rects)`, a group's Prim tree over courtyard gaps;
  `mst_l1_tree(xs, ys, owners)`, a net's Manhattan tree over its sorted pad
  positions, returning its edges and end parts as arrays;
  `seg_cross_counts(m, mlab, o=None, olab=None, skip=None)`, crossings
  tallied per pair of nets; `pad_hit_sums(...)`, tree edges over foreign pads
  summed per (net, owner); `over2(dw, uses, cell2, pitch)`, a congestion
  window's overflow squared, summed as numpy's pairwise sum does; and
  `net_demand(...)`, a net's congestion block. Each gives the values the
  Python code it replaced gave, bit for bit, so a placement is unchanged.

### 0.23.3 (2026-10-17)
- **Simple-path kernels** (`placement_kernels.rs`) for the annealer in
  `py_placer/placement/simple_anneal.py`, which prices its cost model
  (`py_placer/placement/simple.py`) on every move: `mst_l1(xs, ys)`, a net's
  Manhattan Prim tree; `seg_rect_hits(s, p)`, tree edges against pads (box
  test, then Liang-Barsky); `seg_crossings(a, b=None, labels=None)`, proper
  crossings between two sets of tree edges or within one set across labels,
  and `seg_cross_pairs`, the same returning the crossing pairs.
  Each matches the Python code it replaced, operation for operation, so a
  placement is unchanged.

### 0.23.2 (2026-10-16)
- **Placement kernels** (`placement_kernels.rs`) for the quench polish in
  `py_placer/placement/quench.py`, which calls them for every candidate pose:
  `crossing_pairs(a, b, net_w=None)`, the airwire pairs that cross (box test,
  orientation test, same-net and shared-endpoint exclusions, in row-major
  order), and `mst_prim(xs, ys)`, `compute_mst_edges`' Prim tree in O(n)
  memory. Both are the Python computations they replace, operation for
  operation, so a placement is unchanged.

### 0.23.1 (2026-10-13)

- **`pair_search`**: the whole-route snap's grid search for a PAIR
  (`awx/whole_snap.py`, `route()` -> `search(..., poses=True)`), ported
  exactly as `lane_search` ports a single's -- A* over (cell, heading, vias
  taken, straight steps owed, straight steps taken) from pose to pose, the pair
  router's own moves (45-degree turns at most, each only with no straight step
  owed; a via only after the via's straight steps, one heading through it), an
  opposite-hands pair's first dive its crossover (its clearance a test the
  Python makes, called back and kept), the staircase and share prices, and the
  multi-start search from every start pose to any end pose (no heuristic; with
  no path, the farthest along the lane it got). The heap orders entries as
  Python's tuples do, so the same states are expanded in the same order: the
  snapped plans are byte-identical to the Python search's. An older binary
  searches in Python.

### 0.23.0 (2026-09-26)

- **`lane_search`**: the whole-route snap's grid search for a single lane
  (`awx/whole_snap.py`, `route()` -> `search()`), ported exactly -- A* over
  (cell, heading, vias taken) in the lane's window, its band, its layer's
  blocked cells, a via's rules (the heading's via map, the arc gates, the
  plan's via site within RVIA), the join and stub rules at its ends. Every
  cost is formed from the same operations in the same order as the Python,
  and the heap orders entries as Python's tuples do (estimate, cost, state),
  so the same states are expanded in the same order: the snapped plan is
  byte-identical. The heuristic and the near-end tests are hypot of two
  integers, sqrt(a*a + b*b) exactly; a via's distance from its planned site
  comes in as a table `math.hypot` filled. A pair's search (its pose
  counters, its crossover) stays in Python; a Python build against an older
  binary detects the missing function and searches in Python.
- The grid router's leftovers from the visualizer it no longer has
  (`NodeStore::closed_cells` / `visited_open_cells`, `GridSearch::is_done`,
  and the state `SearchStep::Progress` carried for its cursor) removed: the
  crate builds without warnings, the search unchanged.

### 0.22.0 (2026-09-03)

- **N via-legality rungs** (#530 decision 4, per-net via sizes). The single
  #568 "small fab rung" map `blocked_vias_small` becomes
  `blocked_vias_rungs: Vec<FxHashMap>`, one refcounted map per via geometry
  a search may use (rung r at index r-1; rung 0 stays `blocked_vias`, the
  configured via). New methods: `add_blocked_via_rung(rung, gx, gy)`,
  `remove_blocked_via_rung`, `add_blocked_vias_rung_batch(rung, N x 2)`,
  `remove_blocked_vias_rung_batch`, `rung_count()`, `rung_len(rung)`, and
  `blocked_via_cells_at_rung(rung)` (sorted list of (gx, gy)) so Python can
  build a base map at another via geometry and copy its via cells into a
  rung of the working map. The `*_small*` methods remain as rung-1 aliases;
  `is_via_blocked_rung(gx, gy, rung)` consults rung r's own map when it is
  populated and falls back to rung 0 otherwise (conservative for a smaller
  via; a LARGER via's rung must be populated by the caller). `get_stats()`'s
  8th element is now the sum over every rung. `freeze_static` keeps a rung's
  cells that lie outside the frozen full-size set (a larger via's overlay)
  and clears the rest, as before.
- Python-side consumer: `GridRouteConfig.net_via_sizes` /
  `obstacle_cache.via_rungs`; a Python build against a 0.21.x binary detects
  the missing `add_blocked_vias_rung_batch` and routes single-rung.

### 0.21.4 (2026-08-30)

- **The 0.21.4 crate binaries are published in the v0.21.5 release**
  (2026-09-01) -- v0.21.4 itself shipped before this crate work landed and
  carries 0.21.1-built assets, so `build_router.py` on a 0.21.2..0.21.4 crate
  had to build from source until v0.21.5 (the 0.20.1/0.20.2 pattern).
- The UNSTAMP twins of 0.21.3's span adds: `remove_blocked_cell_spans_batch`
  (N x 4) and `remove_blocked_via_spans_batch` (N x 3), same inclusive-bounds
  row layout, expanded in Rust.

  0.21.3 could only ADD spans, which is why only `build_base_obstacle_map`
  (build-once, never unstamped) could use them. The per-net obstacle cache --
  the consumer carrying most of the traffic -- stamps a net's capsules on entry
  and removes the identical arrays on exit, so it needed a remove that expands
  to exactly the same cell multiset. Without symmetry cells stay blocked after
  their net is ripped and the router silently loses routable space, breaking
  the invariant `working_obstacles == base_obstacles + sum(net_obstacles_cache)`
  that `run_obstacle_audit`, `tests/test_obstacle_addremove_parity.py` and
  `tests/test_obstacle_map_balance.py` defend. See #815.

### 0.21.3 (2026-08-30)

- Capsule keep-outs can be stamped as SPANS instead of cells:
  `add_blocked_cell_spans_batch` / `add_blocked_via_spans_batch` (+ the
  `add_static_*` twins for the base build's stamp proxy). Each row is
  `(gx, y_lo, y_hi[, layer])` with both y bounds INCLUSIVE, expanded in Rust.

  A capsule is convex, so its rasterization is one contiguous run per column --
  measured **8.3 cells per column** over 400 representative capsules. A span
  costs 12 bytes per column against 8 bytes per cell, i.e. **5.2x denser** for
  the identical cell multiset (each span lists each of its cells exactly once,
  so refcounts land the same). Verified against the cell path on 720 capsules:
  zero cell-state mismatches, identical `get_stats()`.

  This exists so the Python-side capsule memo can cache spans rather than
  materialised cells. It thrashes badly in the cell form -- on glasgow_revC,
  2,270,477 of 2,317,497 misses were EVICTIONS with the cache pinned at its
  16M-row ceiling. Expanding spans back to cells in Python costs 7.44 us
  against a 0.17 us cache hit (~65 s/route), which is why the expansion has to
  be here and the consumer has to take spans. See #815.

### 0.21.2 (2026-08-30)

- #800: `GridObstacleMap.add_allowed_rect(min_gx, min_gy, max_gx, max_gy)` --
  add a whole inclusive rectangle of allowed cells in ONE Python->Rust
  crossing. The terminal-exemption block in `route_net_with_obstacles` was
  calling `add_allowed_cell` once per cell -- the issue measured 509,109,266
  crossings per route on rp2350.

  Measured: **~50 ns/cell -> ~3 ns/cell (15-17x)**, against the ~2.5x an
  `N x 3` cell batch would have given. One rp2350 route emits **655,531,651**
  exemption cells in 1,487,872 blocks, counted directly. Output is
  byte-identical. No whole-route percentage is quoted: it divides by a route
  time that swings with board, machine and run.

  A rect rather than the `N x 3` cell batch the issue sketched, for two reasons
  in the code: `allowed_cells` is ONE set keyed by `pack_xy` with **no layer
  dimension**, so an `N x 3` array carries a column the map cannot store; and
  all five callers are already rectangles around a terminal, so a rect costs 4
  ints where a cell batch would have Python build a 441-row array first --
  paying much of the per-cell cost the batch exists to remove.

  An inverted range (min > max) inserts nothing, which is what lets callers
  hand over their `bounds` clipping verbatim: a terminal whose block lies
  wholly outside `bounds` must add no cells, exactly as Python's `range()` did.
  `add_allowed_cell` is unchanged and still exported.

### 0.21.1 (2026-08-16)

- PoseRouter (diff pairs): `layer_direction_preferences` +
  `direction_preference_cost` -- the same per-layer H/V off-axis penalty
  GridRouter has had, previously never forwarded to the diff engine
  (#658 diff-step parity; measured 26mm of off-axis diff legs on
  orangecrab walling single-ended corridors).

### 0.21.0 (2026-08-16)
- #656: `attraction_potential` parameter on `GridRouter` -- potential-based
  path-attraction shaping. phi(x, y, layer) peaks at the lane on its layer,
  quadratic falloff over `attraction_radius`, cross-layer states hold
  `attraction_cross_layer_pct` percent. Lateral and VIA edges are charged
  `cost - (phi(next) - phi(prev))` with positive floors: cycles always cost
  > 0 (no loop farming), constant-potential travel earns nothing (no meander
  reward), and a via onto the lane's layer collects a one-time jump that can
  compete with `via_cost`. 0 (default) = bit-identical legacy behavior.

### 0.20.1 and earlier

- **0.20.1**: **`via_proximity_cost 0` = no extra cost, ban mode removed.**
  0 used to hard-block vias in stub/BGA proximity zones -- the only knob whose
  0 was its strongest setting (and a ~200x CPU hazard). 0 now means "no extra
  via cost from proximity", matching every other knob's 0-is-off. Removed:
  `add_stub_proximity_costs_batch`'s `block_vias` parameter (now 3 args), the
  B2 `stub_via_block_cells` refcount ledger + its `clear_stub_proximity`
  drain, and the pose router's `via_proximity_cost == 0` via-ban branch.
  **Pose via-proximity penalty is now GRADED**, matching the single-ended
  router: `(stub + dest-layer proximity) * via_proximity_cost` added to the
  via cost, replacing the binary `via_cost * multiplier` cliff (~10x for any
  via anywhere in the 7mm BGA ring / any stub-zone rim cell, ~a 90mm-detour
  equivalent that effectively forbade diff-pair layer changes near escape
  fields). BGA proximity rides the layer-proximity map, so
  `--bga-proximity-cost 0` now truly disarms it for diff pairs too.
  Added later at the same crate version: `add_stub_proximity_costs_grouped(groups,
  max_zone_rects=None)`, the per-source SUM composition that `KICAD_PROXIMITY_SUM`
  selects (off by default). First published in v0.20.2.
- **0.20.0**: Removed `VisualRouter`/`SearchSnapshot` and `visual_router.rs` with the pygame visualizer (#569) -- the debugging workflow it served is covered by grading, manifest replay and the GUI. **#568 per-rung via legality** — `blocked_vias_small`, a second
  refcounted via-block map at the small fab-rung reserve (subset semantics:
  EMPTY = unpopulated, rung>=1 queries fall back to `blocked_vias`, so
  single-rung callers are byte-identical). `is_via_blocked_rung(gx, gy, rung)`;
  add/remove single+batch mirrors; clone/clone_fresh copy it; freeze clears it
  (the static bitmap blocks every rung — conservative). `route_multi` /
  `route_with_frontier` accept `via_rung=0` per search. `get_stats` appends the
  small-map count as the 8th element (positional consumers unaffected) so the
  #309 refcount balance audit covers the new map.
- **0.19.3**: **#529 plateau grace** — `route_multi` / `route_with_frontier`
  accept `grace_tranches=0`: up to N consecutive quantum-failing tranches
  are tolerated before extension is denied, with the progress reference
  frozen at the last real-improvement mark (so the eventual grant is judged
  cumulatively over the plateau). Intended to emulate a wider progress
  window without a bigger base. **MEASURED DEAD — leave at the default 0**:
  because the streak resets on every success, searches alternating progress
  and plateau chain grace windows indefinitely (more permissive than a true
  wide window, not equivalent). A/B at K=4 on 200k bases: zynq7020 30 open
  + 2 DRC (worse than the feature OFF), duodyne 3.6x the CPU. Kept as an
  inert dial so the idea is not re-tried blind. Default 0 reproduces
  0.19.2 exactly.

- **0.19.2**: **#529 dynamic iterations (tranche-earned budget extension)** —
  `route_multi` / `route_with_frontier` accept an optional
  `max_iterations_ceiling` (default 0 = off, existing callers unchanged).
  When above `max_iterations`, hitting the cap grants another
  +1×`max_iterations` tranche iff the tranche just completed improved the
  search's closest approach — `best_h`, the minimum cost-to-go `f − g` over
  expanded nodes, tracked free at pop time — by a quantum:
  `max(quantum_cells grid cells, quantum_pct% of best_h at the tranche
  start)` — tunable via the optional `quantum_cells=2.0` / `quantum_pct=2.0`
  kwargs. The first grant references a `best_h` snapshot taken at half the
  base budget (improvement "vs iteration 0" is trivially satisfied).
  Deterministic (pure search-internal state). New stats: `best_h`,
  `iteration_tranches`.
- **0.19.0**: **#156 fractional per-layer track_margin** — `route_multi` /
  `route_with_frontier` accept `track_margin` as a FLOAT (scalar) or a
  per-layer `list[float]`, in fractional grid cells, instead of an integer
  Chebyshev cell count. The margin feeds the existing swept-capsule
  `segment_blocked` check per destination layer, so wide (power) and
  impedance-width nets reserve their exact extra half-width — no `ceil`,
  no blunt `+1` cell of over-blocking. Integer arguments still coerce
  (backward compatible). Python side now computes
  `(net_width(layer) - reserved_width(layer)) / 2 / grid_step` per layer.
- **0.18.5**: **P3 attraction-field precompute** (soft-knobs review) -- new
  `build_attraction_field(radius, bonus)` on `GridObstacleMap` precomputes the
  per-layer max cross-layer attraction bonus once per net (entries x disk x
  layers, <= 8 layers), turning `get_cross_layer_attraction` from an
  O(radius^2)-per-candidate-move scan into an O(1) lookup with identical
  falloff semantics; empty field = the scan fallback, so callers on an older
  Python side (no build call) are unaffected. Cleared with
  `clear_cross_layer_tracks`.
- **0.18.4**: **cross-layer bus attraction** (#296 R9 phase B) — new
  `GridRouter` kwarg `attraction_cross_layer_pct` (default 0 = legacy): when
  > 0, the bus attraction-path bonus is granted at that percentage on layers
  OTHER than the path point's layer (same-layer points still win at 100%),
  so a planned bus corridor keeps guiding a member after a via transition
  instead of going silent. Wildcard-layer bucket keys added to the
  attraction spatial hash (only consulted when the fraction is enabled).
  Also converts the bus path attraction from a subtractive bonus to a
  multiplicative step DISCOUNT (0-90%; 50 legacy bonus units = 1%, so the
  default 5000 = full strength): the old subtraction exceeded the move cost
  (~1000), creating negative-cost edges that break A*'s first-arrival
  invariant, and once floored it saturated -- collapsing every proximity/
  alignment/layer gradation (soft-knobs review finding #2). The vertical
  (cross-layer copper) attraction stays subtractive but is now capped so a
  step can never go below a tenth of the base move cost (router.rs +
  pose_router.rs). Riders from the soft-knobs audit: the attraction path
  lookup now uses a spatial POINT index instead of a whole-path linear scan
  per candidate move (P2); attraction distance is Euclidean instead of
  Manhattan-vs-Euclidean-radius, which halved diagonal reach (N4); the
  stub-proximity endpoint-exemption scan runs only on a nonzero cost hit
  (P4); VisualRouter now passes layer direction preferences through, so
  --visualize replays the production cost model (C2); and the turn cost is
  ANGLE-PROPORTIONAL (N5): the knob is the 90-degree anchor -- 45-degree
  kinks cost half, 135-degree hairpins 1.5x, reversals 2x (was a flat
  charge that priced a gentle jog like a hairpin).

- **0.18.3**: **#452 direction preference no longer bypassed by diagonals** — the
  per-layer H/V preference penalty (`direction_preference_cost`, applied via
  `layer_direction_preferences`) charged only PURE axis moves against the
  preference (`dy != 0 && dx == 0` on a horizontal layer). A 45 diagonal
  satisfies neither test, so it cost ZERO on every layer — and since the router
  moves 8-way, the cheapest way to travel along the non-preferred axis was a
  chain of free diagonals. The preference was not weak, it was *bypassed*:
  raising the knob could not restore discipline. The penalty now charges any
  move that advances along the non-preferred axis, diagonals included (one cell
  of wrong-axis progress either way, so the same charge). Deliberately gentle:
  50 vs DIAG_COST 1414 is a ~3.5% nudge that tips ties toward the preferred
  axis without discouraging useful 45 routing. Measured motivation (#296 corpus
  study): humans hold per-layer discipline (urchin F.Cu 68% H / B.Cu 86% V) and
  spend 1.67 vias/net; we spent 2.41 (+44%) while routing SHORTER, i.e. paying
  for length in layer changes, and our texture showed the staircase signature
  (segments/mm 0.80 vs 0.49 human, off-grid bends 0.13 vs 0.02).

- **0.18.2**: **#422 static keep-out bitmap (memory)** — permanent board
  geometry (board-edge clearance band, off-board / outside-outline area, board
  cutouts / switch holes) is now stored in a separate never-cleared per-layer
  **static bitmap** (`add_static_blocked_cell[s_batch]` / `..._via[s_batch]`)
  instead of the `blocked_cells` / `blocked_vias` refcount hashmaps. These cells
  never change during routing, so a set bit (1 bit/cell) suffices where a hashmap
  entry costs ~16 B (u64 key + u16 refcount padded to a 16 B bucket + control byte
  + power-of-two slack). `is_blocked` / `is_via_blocked` OR the static bitmap with
  the dynamic map, so a statically-blocked cell behaves **byte-identically** to a
  refcounted one (same source/target override) — it is simply cheaper to store
  and immune to the per-net remove/restore cycle (no coincident-cell desync: the
  two structures are independent). On sparse large boards (split keyboards: many
  switch-hole cutouts + big off-board area) this static class is ~60% of all
  blocked cells; moving it out of the hashmaps cuts the obstacle-map (and its
  per-run working clone) memory proportionally. New pymethods:
  `add_static_blocked_cell(s_batch)`, `add_static_blocked_via(s_batch)`,
  `get_static_stats()`. `clone`/`clone_fresh`/`shrink_to_fit` handle the new
  bitmaps; `get_stats()` arity is unchanged. Routing results are identical.
  Added later at the same crate version: `freeze_dynamic_to_static()`, which
  moves a finished base map's counted cells and vias into the static bitmaps
  (the working clone then carries the base as bits and only the per-net caches
  as counts), and the `dynamic_refcount_stats()` diagnostic.

- **0.18.1**: **S3-b (#385)** — admissible bounding-box heuristic for large
  target lists. `heuristic_to_targets` was O(targets) per push (min octile
  distance over the whole list); a multipoint/tap backward probe passes
  hundreds of tap points, making the heuristic the hot cost. When the target
  count exceeds 16, it now computes the heuristic against a precomputed
  `TargetHint` (target bounding box + a layer bitmask, built once per search):
  distance to the clamped box point, `via_cost` iff no target shares the
  node's layer, and min path-steps to the box. Each term is a per-component
  **lower bound** on the exact min, so the summed heuristic is **admissible**
  — it never overestimates true cost, so it keeps the FULL goal set and can
  never make a routable net unreachable (unlike the rejected S3-a Python
  target cap, which dropped goals and regressed congested multi-drop nets).
  Behavior-changing (weaker heuristic → different exploration → different
  paths), gated on corpus A/B, NOT byte-identical. Scoped to the single-ended
  `GridSearch` (the tap-route probe); the pose/diff-pair router uses a
  single-goal Dubins heuristic and is untouched. cparti_fpga: failed 43→40,
  iterations +0.09% (no expansion explosion), ~35% less CPU/net; urchin
  (all nets ≤16 targets) byte-identical.
- **0.18.0**: The rust-router-review batch (issues #384/#385/#386/#387). Verified byte-identical vs 0.17.4 on a 3-board identity gate (bitaxe_ultra / urchin / castor_pollux: `total_iterations` + `total_vias` + all JSON_SUMMARY keys EXACT) for everything except the two explicitly behavior-changing items called out below. (Audit note for future gates: Python-side additive JSON_SUMMARY keys — e.g. `blockers`, #409, present only on runs with attributable failures, and `pad_pairs_connected`/`pad_pairs_total`/`pad_pairs_open`, its follow-up — are not produced by the Rust router; compare them key-by-key across Python releases, absent-vs-present is not a Rust identity failure.)
  - **S1-A (#384)**: the separate `closed` FxHashSet is gone from all four search loops — the closed flag lives in the top bit of the node's parent field (`NodeMap`, then `NodeStore`; `PoseNodeMap` likewise), saving a hashset lookup + insert per expansion.
  - **S1-B (#384)**: tiled flat-array node store for the grid A* (`NodeStore`): 64x64-cell tiles per layer, per-node record 12B (`g: i32`, `parent: u32` node index carrying the closed bit, `steps: i32`). A node lookup is one small per-TILE hashmap probe + direct indexing; path reconstruction and the collinear-via ancestry walks follow u32 parent indices. The store is per-search, so peak memory scales with explored area at 12B/node (the M1 transient-blowup fix) and is freed on return. (The review's per-tile generation stamps were deliberately dropped: they only pay off with a cross-search tile cache, which would pin explored-union x 12B for a whole batch to save microseconds of tile zeroing. The pose router keeps its `PoseNodeMap` hashmap — pose keys carry theta, so a tile is 8x larger for a much sparser visit pattern.)
  - **S2 (#384)**: per-layer blocked-cell bitmap in `GridObstacleMap` — `is_blocked`'s refcount-hashmap lookup becomes a bounds check + one bit load. The bitmap is a PURE CACHE of "refcount > 0": the refcount hashmaps stay authoritative and the bitmap is written ONLY at 0->1 / 1->0 transitions inside the same four refcount-mutating functions (`add_blocked_cell`, `add_blocked_cells_batch`, `merge_blocked_from`, `remove_blocked_cells_batch`), keeping the #208/#309 desync class closed. Lazy window growth (128-cell margin); coordinates beyond a 1<<26-cell cap fall back to per-layer overflow hash sets (pathological coords degrade speed, never correctness). Also hoists a `bga_zones.is_empty()` early-out above the zone scan.
  - **C1 (#387)**: one search core per state space. `route_multi` / `route_with_frontier` are thin wrappers over a single `GridSearch::step()` monomorphized over a `SearchSink` trait (`StatsSink` / `FrontierSink` / `NullSink`); `route_pose` / `route_pose_with_frontier` likewise share one `pose_search` core. The duplicated `check_via_exclusion` closure is a single free fn; the GND-via ahead/behind block (3 verbatim copies) is `gnd_via_sites()`, shared with `compute_gnd_via_directions`.
  - **B1 (#386)**: the visualizer was a drifted third fork missing the A1 negative-sentinel `min_layer_cost` filter, the `layer_forbidden` guards, and the `free_here` via override — `--visualize` routed a DIFFERENT search than production. `VisualRouter` now owns a production `GridRouter` and steps the shared `GridSearch` core (verified: path, iterations, expansion counts exactly equal `route_multi`'s).
  - **B2 (#386)**: `add_stub_proximity_costs_batch(block_vias=true)` had no removal path — with `--via-proximity-cost 0` the per-net via bans accumulated monotonically across nets. The batch now records every `blocked_vias` increment in an internal ledger and `clear_stub_proximity` drains it (decrement, remove at 0), so independently-added via bans survive. The per-net prepare/restore cycle already calls `clear_stub_proximity` on both sides of every route, and the GUI reaches stub proximity only through those shared engine functions — CLI/GUI parity with zero Python changes. Behavior change confined to `--via-proximity-cost 0` runs.
  - **B3 (#386)**: `cross_layer_tracks` layer mask widened u8 -> u32 with shift guards — the u8 mask wrapped (release) or panicked (debug) for layers >= 8 on 10-12L boards.
  - **B4 (#386)**: `heuristic_to_targets` returns 0 for an empty target list (was i32::MAX -> `f = g + h` wrapped in release, corrupting heap order into a silent max-iterations burn), and all f-score computations use `saturating_add`.
  - **B5 (#386)**: `direction_to_index` returns `Option` instead of silently aliasing a (0,0) delta to East; in `is_within_45_degrees` a degenerate BASE direction means "no constraint" (allow) and a degenerate test direction rejects.
  - **B6 (#386)** *(behavior-changing, forbidden-layer boards only)*: the pose frontier tracker no longer reports forbidden-layer cells as "blocked" (a forbidden layer is a routing rule, not copper; the grid router never tracked them), so rip-up candidate scoring is no longer polluted on `--layer-costs -1` boards.
  - **S6 (#384)**: micro-hoists — `layer_forbidden(current.layer)` out of the direction loop and the pose move/turn branches; loop-invariant `is_free_via` reused across the via destination-layer loop.
  - **C2 (#387)**: dead API deleted — the `release_memory()` no-op export, `attraction_path_hash` demoted map -> set, `pack_bucket_key`'s unused param, the never-written `max_f_score` stat, and `GridRouter::unpack_key`. `route_pose` KEPT (the S4 Python change calls it). `is_blocked_with_margin` with margin > 0 KEPT — the review marked it dead, but `single_ended_routing._segment_fits_wide` (power-net neckdown) passes real wide-track margins through it.
  - **S5 (#385)** evaluated and REJECTED: preferring larger g on f-ties measured
    +22% microbench wall time (extra heap-comparison work) for a <=0.03%
    iteration change across 6 real boards -- reverted in-branch, FIFO tie-break stands.
  - Companion Python-only change (S4, #385): the diff-pair coupled-middle theta-fan calls `route_pose` instead of `route_pose_with_frontier` — it always discarded the blocked-cell frontier but paid tracker inserts + a 10k-cell sort/marshal per failed probe (verified output-identical).
- **0.17.4**: Memory/cache refactor of the `PoseRouter` A* (`route_pose` / `route_pose_with_frontier`), no behavior change — the follow-up promised by 0.17.2. Each function kept SEVEN parallel per-explored-pose maps — `g_costs`, `parents`, `steps_from_source`, `straight_steps_remaining`, `straight_steps_taken`, `cumulative_turn_1`, `cumulative_turn_2` (each an `FxHashMap<u64,_>`) — all storing and hashing the SAME pose key. They are folded into one `FxHashMap<u64, PoseNodeState{g,parent,steps,straight_remaining,straight_taken,turn_1,turn_2}>` behind a thin `PoseNodeMap` wrapper: each pose key is stored ONCE instead of seven times and hashed once per access, cutting A* peak RSS (dominant on boxed-in pose searches toward `--max-iterations`) and improving cache locality. Source poses use a `NO_PARENT = u64::MAX` sentinel to preserve the old `parents.get -> None` reconstruct-terminates semantics. The via relaxation intentionally writes only g/parent/steps/straight_remaining and leaves the straight-taken + both turn counters untouched (exactly as the old code, which never re-inserted them on a via) — modelled with a distinct `relax_via` entry-default path so a pre-existing pose keeps its prior values and a fresh one reads back 0. Byte-identical routing verified vs 0.17.3 (same segments + vias). Mirrors the 0.17.2 `NodeMap` fold done for the single-ended grid A*.
- **0.17.3**: Switched the global allocator to **mimalloc** (`#[global_allocator]`, default-features off). The system allocator (esp. macOS) retains freed pages — a hard net's A* frontier allocates hundreds of MB, frees them on return, but the pages stay resident, so process RSS never falls back. mimalloc returns freed pages to the OS with short decay, cutting *sustained* RSS substantially (daisho mid-run 761 -> 515 MB, ~-32%), with byte-identical copper and no measurable time cost (+0.16%, noise). Directly reduces the `--jobs N` concurrent RSS that swap-thrashes small-RAM machines. (Peak RSS is less affected when the peak is a live allocation, e.g. a second-phase obstacle rebuild, rather than retained frontier.)
- **0.17.2**: Memory/cache refactor of the single-ended A* (`route_multi` / `route_with_frontier`), no behavior change. The three parallel per-explored-cell maps — `g_costs` (`FxHashMap<u64,i32>`), `parents` (`FxHashMap<u64,u64>`), `steps_from_source` (`FxHashMap<u64,i32>`) — each stored the SAME cell key; they are folded into one `FxHashMap<u64, NodeState{g,parent,steps}>` (via a thin `NodeMap` wrapper). This stores each explored cell's `u64` key ONCE instead of three times and hashes it once per access, cutting A* peak RSS (dominant on boxed-in searches toward `--max-iterations`) and improving cache locality (all three fields co-located). Source cells (no parent) use a `NO_PARENT = u64::MAX` sentinel to preserve the old `parents.get -> None` / `parents.contains_key -> false` semantics exactly (used by the collinear-via ancestry walk and `get_via_direction_constraints`). `path_vias` is left as a separate conditional map on purpose (a heap `Vec`, only allocated when `via_exclusion_radius > 0`). Byte-identical routing (same cells explored, same costs, same path) — verified against 0.17.1 on the corpus. `PoseRouter` is unchanged (its 14-map consolidation landed in 0.17.4).
- **0.17.1**: FORBIDDEN LAYER mode for `layer_costs` (issue #206). A negative entry in the `layer_costs` vector (canonically `-1`, but the router treats ANY negative value identically) marks a layer the router never places a track on, nor ends a via on — yet the layer stays in the obstacle map (its copper blocks via barrels) and through-vias may SPAN it (a via is one direct edge to its target layer). Used by passing every copper layer in `--layers` for full obstacle awareness while forbidding routed copper on the inner ones (e.g. `--layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 -1 -1 3.0`). Implemented in both `GridRouter` (`route_multi`/`route_with_frontier`) and `PoseRouter` (`route_pose`/`route_pose_with_frontier`): A* skips any successor that would lay track on, turn on, or end a via on a forbidden layer; `min_layer_cost` and the via layer-transition delta sanitize the sentinel out so it can never underflow the cost math. With no negative cost the path is byte-identical to before (the feature is inert). On the Python side any negative `--layer-costs` value is accepted and folded to the single forbidden sentinel.
- **0.17.0**: Wide-track clearance during A* now uses an exact swept-capsule check instead of a Chebyshev square around only the destination cell. New `GridObstacleMap.segment_blocked(gx1, gy1, gx2, gy2, layer, r)` measures the true Euclidean point-to-segment distance from every nearby blocked cell to the A* move's swept segment (the extra half-width `r`, in grid cells, of a wide track swept along the step). `route_multi` / `route_with_frontier` call it for the neighbor and layer-change checks (`track_margin` is reused as the radius; `track_margin == 0` falls back to the plain destination-cell `is_blocked`, so base-width routing is byte-identical). The old `is_blocked_with_margin` (1) over-covered corners (square vs disc) and (2) never checked the *swept body* of a 45° move, so a diagonal step could slip a blocked cell sub-cell between its endpoints — the residual wide-net grazes in #173, and the reason #156's point-disc margin was a no-win (a disc at the endpoint still misses the swept segment). Fixes wide power/impedance-net and plane-connection diagonal grazes. `is_blocked_with_margin` is retained (still used by `visual_router` with margin 0).
- **0.16.2**: `PoseRouter` (the coupled differential-pair centerline router) now takes an optional `layer_costs` argument (per-layer cost multipliers, 1000 = 1.0x), mirroring `GridRouter`. The A* scales each step's base move cost by the current layer's multiplier and adds a via layer-transition cost (cheaper-layer vias discounted, costlier penalized), so diff pairs can be biased onto preferred layers. Empty/omitted `layer_costs` (the default) leaves every layer at 1.0x — behavior is unchanged. Wires up `route_diff.py --layer-costs` (issue #193).
- **0.16.1**: Added the `identify_blocking_obstacles(blocked, segments, vias, pads, expansion_grid, via_expansion_grid, num_layers)` free function - a Rust port of `single_ended_routing._identify_blocking_obstacles` (the rip-up blocking analysis). Given the blocked frontier cells and foreign segment/via/pad geometry (as grid-integer numpy arrays), it returns `net_id -> count` of how many of each net's elements overlap the frontier (each element counted at most once, matching the Python break-on-first-hit). Exact integer grid math, so byte-identical to the Python it replaces. Python calls it when available and falls back to the pure-Python implementation on older binaries.
- **0.16.0**: Added `GridObstacleMap.open_via_cells_within(cx, cy, radius)`, which returns every non-via-blocked grid cell within Chebyshev `radius` of a center, excluding the center, sorted nearest-first by squared Euclidean distance. This lets the Python via-site search (`find_via_position` in `route_planes.py`) replace its per-cell `is_via_blocked()` spiral - one batched FFI query instead of O(radius^2) calls - which matters for the wide-radius plane-tap via search (the forced last-resort plane-pad repair).
- **0.15.1**: A free via (an existing same-net via-in-pad or through-hole pad, registered in `free_via_positions`) now overrides both the `is_via_blocked` veto and the path's via-too-close test when deciding whether to change layers. Previously the via branch was gated on `!is_via_blocked` *before* `is_free_via` was consulted, so a cell flagged as a free via could still be blocked by its own clearance ring (its center is added to `blocked_vias` by `add_net_vias_as_obstacles`). The router then refused to reuse the existing hole and dropped a redundant via a couple cells away beside a perfectly good via-in-pad (e.g. orangecrab RAM_A15/RAM_CS#/RAM_WE#). Reusing an existing same-net hole is always legal and adds no new drill, so the override is safe.
- **0.15.0**: Changed bus routing attraction from proximity-based to direction-based. The router now only gives attraction bonus when moving in the same direction as the neighbor path was moving, not just for being near it. This prevents spiraling behavior where strong attraction could cause the router to circle around the neighbor path instead of making progress toward the target. Same direction = full bonus, 45° off = 70% bonus, perpendicular or opposite = no bonus. Bonus has quadratic distance falloff (stronger near the guide path). Python-side changes: bus detection now tracks `clique_endpoint` (source vs target clustering), routes from clustered endpoints using single-direction mode, and uses dense path sampling for direction vectors.
- **0.14.0**: Added path attraction feature for bus routing. New `attraction_path`, `attraction_radius`, and `attraction_bonus` parameters to GridRouter. Routes can be attracted to a previously routed path (same layer only) using `set_attraction_path()`. Enables parallel routing of bus groups where each net follows its neighbor.
- **0.13.0**: Added layer direction preference feature. New `layer_direction_preferences` parameter (list of u8: 0=horizontal, 1=vertical, 255=none) and `direction_preference_cost` parameter (penalty for non-preferred direction moves). Encourages horizontal routing on some layers and vertical on others, creating more organized, human-like routing patterns.
- **0.12.2**: Reverted to flat proximity costs (removed direction-aware cost adjustment from v0.10.0). Testing showed flat costs produce better results: 19% fewer iterations, 40% faster, and improved routing success (100% vs 97.9% on benchmark). The direction-aware heuristic mismatch caused more search exploration than the "efficient zone exit" guidance saved. Removed dead code: `get_directional_proximity_cost()`, `add_proximity_zone_center()`, `add_proximity_zone_centers_batch()`, `clear_proximity_zone_centers()`, and `proximity_zone_centers` field.
- **0.12.1**: Made VisualRouter fully consistent with GridRouter. Added `turn_cost`, `via_proximity_cost`, `vertical_attraction_radius`, and `vertical_attraction_bonus` parameters. Updated to use direction-aware proximity costs (`get_directional_proximity_cost()` and `get_layer_proximity_cost()`) and cross-layer attraction. Visualization mode now produces identical iteration counts to non-visualization mode.
- **0.12.0**: Added proximity-aware heuristic for faster routing on dense boards. The heuristic now auto-estimates expected proximity costs per step based on stub/track/BGA proximity settings and radii, dramatically reducing search space (up to 6x speedup) while keeping high proximity costs to prevent blocking later routes. Formula: `sum(cost_i * radius_i) * factor` where factor defaults to 0.02 (tuned for ~5mm typical radius). Diff pair routing uses 1/10th of the factor due to the more constrained pose-based search. **Smart endpoint detection**: The heuristic is only applied when source or target is inside a proximity zone (checked via `is_in_any_proximity_zone()`); routes with both endpoints outside proximity zones use h=0 for optimal search. New `proximity_heuristic_cost` parameter and `set_proximity_heuristic_cost()` setter in both GridRouter and PoseRouter, `--proximity-heuristic-factor` CLI option (route.py and route_diff.py), and `GridRouteConfig.get_proximity_heuristic_cost()` method.
- **0.11.0**: Added A* search statistics collection. `route_multi` now returns `(path, iterations, stats)` where `stats` is a dict containing: `cells_expanded`, `cells_pushed`, `cells_revisited`, `duplicate_skips`, `path_length`, `path_cost`, `initial_h`, `final_g`, `via_count`, and computed metrics `heuristic_ratio`, `expansion_ratio`, `revisit_ratio`, `skip_ratio`. Enable stats printing with `--stats` flag.
- **0.10.0**: Added direction-aware proximity costs. When routing within stub or BGA proximity zones, steps moving away from zone centers cost less than steps moving towards them. This encourages the router to exit proximity zones efficiently. New methods: `add_proximity_zone_center()`, `add_proximity_zone_centers_batch()`, `clear_proximity_zone_centers()`, `get_directional_proximity_cost()`. Zone centers are automatically registered when adding stub/BGA proximity costs in Python (`add_stub_proximity_costs()`, `add_bga_proximity_costs()`). `clear_stub_proximity()` now also clears zone centers.
- **0.9.0**: Added `layer_costs` parameter to GridRouter and VisualRouter for layer preference routing. Per-layer cost multipliers (1000 = 1.0x) affect movement costs, source initialization penalty for expensive layers, and via transition costs. Switching to a cheaper layer discounts the via cost (can reduce to 0). Default in route.py: F.Cu=1.0x, all others=3.0x. Values must be 1.0-1000x.
- **0.8.4**: Added `vertical_attraction_radius` and `vertical_attraction_bonus` parameters to GridRouter (previously only in PoseRouter). Single-ended routing can now attract to tracks on other layers, consolidating routing corridors and leaving more room for through-hole vias.
- **0.8.3**: Added `via_proximity_cost` parameter to GridRouter (was only in PoseRouter). Multiplies stub proximity cost when placing vias - higher values discourage vias near stubs, 0 blocks vias entirely in stub proximity zones. Now both single-ended and diff pair routing respect via proximity costs. Added batch FFI operations (`add_blocked_cells_batch`, `remove_blocked_cells_batch`, `add_stub_proximity_costs_batch`) to reduce Python-Rust call overhead. Changed obstacle map to use reference-counted `HashMap<u64, u16>` instead of `HashSet<u64>` for correct incremental updates when nets share blocked cells.
- **0.8.2**: Added `turn_cost` parameter to GridRouter - penalizes direction changes to encourage straighter paths with fewer wiggles. Default is 1000 (same as ORTHO_COST). Configurable via `--turn-cost` CLI option.
- **0.8.1**: Added cross-layer track attraction for vertical alignment. New `vertical_attraction_radius` and `vertical_attraction_bonus` parameters to PoseRouter. New GridObstacleMap methods: `add_cross_layer_track()`, `get_cross_layer_attraction()`, `clear_cross_layer_tracks()`. Attracts routes to stack on top of tracks on other layers, consolidating routing corridors and leaving more room for through-hole vias.
- **0.8.0**: Added `via_proximity_cost` parameter to PoseRouter - allows vias near stubs with cost penalty instead of blocking (default: 10, set to 0 for old blocking behavior). Improved turn radius enforcement: minimum turn radius is now enforced for ALL route steps (not just near vias), ensuring smooth paths throughout. `straight_after_via` is based on `min_radius_grid + 1` instead of hardcoded 2 steps, preventing DRC violations where P/N tracks turn too sharply after vias. Added `route_pose_with_frontier()` method that returns blocked cells on failure for blocking analysis.
- **0.7.0**: Added `PoseRouter` with Dubins path heuristic for orientation-aware differential pair centerline routing. State space expanded to (x, y, θ, layer) where θ is one of 8 directions (45° increments). Dubins path length used as heuristic for better routing with prescribed start/end orientations.
- **0.5.1**: Added `via_exclusion_radius` parameter to prevent routes from conflicting with their own vias. Tracks via positions along each path and blocks moves that would cause P/N offset tracks to intersect P/N vias.
- **0.5.0**: Added `collinear_vias` parameter for differential pair routing - enforces symmetric via geometry: `±45° → D → VIA → D → ±45°` (requires 2 steps before via, approach within ±45° of previous, exit same as approach, then ±45° allowed)
- **0.4.0**: Performance and stability improvements
- **0.3.0**: Added `clone()` method for GridObstacleMap to support incremental obstacle caching
- **0.2.1**: Fixed `is_blocked()` to check blocked_cells before allowed_cells (prevents allowed_cells from overriding regular obstacles)
- **0.2.0**: Added `add_allowed_cell()` for BGA zone overrides, added `__version__` attribute
- **0.1.0**: Initial release with basic A* routing

## Pad obstacles (Python side)

Pads reach the map from Python, as batches: `_add_pad_obstacle` in
`py_router/obstacle_map.py` rasterizes each one with `pad_blocked_cells_array`
(`py_router/routing_utils.py`). A pad is a rounded rectangle at its board-space
size (a circle or oval as a stadium, a roundrect by its corner ratio), tilted by
`pad.rect_rotation` and measured from its true, sub-cell centre (#70); a custom
pad is rasterized from its own polygons (#188). Track cells are blocked within
half the track width plus the clearance, resolved per layer (net classes,
`.kicad_dru` layer rules, the pad's own clearance override); via cells within
half the via size plus that clearance. So a via lands between fine-pitch QFN
pads only where it clears every one of them.

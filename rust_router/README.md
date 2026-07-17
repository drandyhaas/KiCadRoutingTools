# Rust Grid Router

High-performance A* grid router implemented in Rust with Python bindings via PyO3.

**Current Version: 0.19.0**

## Features

- Grid-based A* pathfinding with octilinear routing (8 directions)
- Multi-source, multi-target routing
- Via cost and layer transitions
- BGA exclusion zone with allowed cell overrides
- Stub proximity costs to discourage routing near unrouted stubs (flat cost model for accurate heuristic estimation)
- **Turn cost penalty** for direction changes - encourages straighter paths with fewer wiggles
- **Cross-layer track attraction** for vertical alignment - attracts routes to stack on top of tracks on other layers
- **Layer cost preferences** - per-layer cost multipliers to prefer certain layers (e.g., keep signals on F.Cu, avoid B.Cu). Default: F.Cu=1.0x, others=3.0x
- **Pose-based routing with Dubins heuristic** for differential pair centerlines (orientation-aware A*)
- **Rectangular pad obstacle blocking** with proper rotation handling
- **Collinear via constraint** for differential pair routing (ensures clean via geometry)
- **Via exclusion zones** to prevent routes from conflicting with their own vias (for diff pair P/N offset tracks)
- ~10x speedup vs Python implementation

## Installing

Most users do **not** need to build this themselves. From the project root:

```bash
python build_router.py
```

This downloads a prebuilt binary for your OS (Linux x86_64, macOS arm64
+ x86_64, or Windows x86_64) from the project's
[GitHub Releases](https://github.com/drandyhaas/KiCadRoutingTools/releases)
and drops it into `rust_router/`. Prebuilts use PyO3's `abi3-py39`, so one
binary works for any Python 3.9+ interpreter.

Useful flags:

```bash
python build_router.py --from-source   # build locally instead of downloading
python build_router.py --tag v0.15.0   # pin to a specific release
python build_router.py --clean         # remove all build artifacts
```

## Building from source

If a prebuilt isn't available for your platform, or you're hacking on the
router, build with cargo via `python build_router.py --from-source`.

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

## Release process (maintainers)

Tagging a commit `vX.Y.Z` triggers `.github/workflows/release.yml`, which
builds the router on four runners and attaches the binaries to a GitHub
Release of the same name:

```bash
git tag v0.15.0
git push origin v0.15.0
```

`build_router.py` then serves that release to users automatically.

**Manual build (alternative):**
```bash
cd rust_router
cargo build --release
```

### Installing the Python Module

After building, copy the compiled library to this directory:

**Windows:**
```bash
cp target/release/grid_router.dll grid_router.pyd
```

**Linux:**
```bash
cp target/release/libgrid_router.so grid_router.so
```

**macOS:**
```bash
cp target/release/libgrid_router.dylib grid_router.so
```

The `route.py` script automatically adds this directory to the Python path.

### Automatic Version Checking

When `route.py` starts, it automatically:
1. Checks that numpy is installed
2. Verifies the Rust library version matches `Cargo.toml`
3. Rebuilds automatically if there's a version mismatch

This ensures you're always running the correct version of the Rust router.

### Using maturin (alternative)

```bash
pip install maturin
maturin develop --release
```

## Usage from Python

```python
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

## Benchmark

Run the full 32-net benchmark from the parent directory:

```bash
python route.py kicad_files/fanout_starting_point.kicad_pcb kicad_files/routed.kicad_pcb \
  "Net-(U2A-DATA_23)" "Net-(U2A-DATA_20)" ...
```

### Performance Results

| Metric | Value |
|--------|-------|
| 32 net routing | ~7 seconds |
| Success rate | **32/32 (100%)** |
| Total iterations | ~285,000 |
| DRC violations | None (DATA nets) |

Speedup vs Python implementation: **~10x**

Note: With proper rectangular pad blocking using `int()` instead of `round()` for grid discretization, the router correctly places vias on QFN pads while maintaining clearance to adjacent pads.

## API Reference

### GridObstacleMap

```python
obstacles = GridObstacleMap(num_layers: int)
```

Methods:
- `add_blocked_cell(gx, gy, layer)` - Mark a cell as blocked on a specific layer
- `add_blocked_cells_batch(cells)` - Add multiple blocked cells at once. `cells` is a list of `(gx, gy, layer)` tuples. Uses reference counting for correct incremental updates.
- `remove_blocked_cells_batch(cells)` - Remove multiple blocked cells at once. `cells` is a list of `(gx, gy, layer)` tuples. Decrements reference counts and only unblocks when count reaches 0.
- `add_blocked_via(gx, gy)` - Mark a position as blocked for vias
- `set_bga_zone(min_gx, min_gy, max_gx, max_gy)` - Set BGA exclusion zone
- `add_allowed_cell(gx, gy)` - Override BGA zone blocking for a cell
- `set_stub_proximity(gx, gy, cost)` - Set proximity cost for a cell
- `add_stub_proximity_costs_batch(costs)` - Set multiple stub proximity costs at once. `costs` is a list of `(gx, gy, cost)` tuples.
- `is_blocked(gx, gy, layer)` - Check if cell is blocked
- `is_via_blocked(gx, gy)` - Check if via position is blocked
- `get_stub_proximity_cost(gx, gy)` - Get proximity cost for a cell
- `add_cross_layer_track(gx, gy, layer)` - Mark a position as having a track on specified layer (for vertical attraction)
- `get_cross_layer_attraction(gx, gy, current_layer, radius, bonus)` - Get attraction bonus for positions near tracks on other layers
- `clear_cross_layer_tracks()` - Clear all cross-layer track data
- `is_in_any_proximity_zone(gx, gy)` - Check if position is in any proximity zone (stub or BGA). Used to decide if proximity heuristic should be applied for a route.
- `clone()` - Create a deep copy of the obstacle map (for incremental caching)

### GridRouter

```python
router = GridRouter(via_cost: int, h_weight: float, turn_cost: int = 1000, via_proximity_cost: int = 1,
                    vertical_attraction_radius: int = 0, vertical_attraction_bonus: int = 0,
                    layer_costs: List[int] = None, proximity_heuristic_cost: int = 0)
```

Parameters:
- `via_cost`: Cost for layer transitions (scaled by 1000)
- `h_weight`: Heuristic weight (>1 for faster but less optimal routes)
- `turn_cost`: Cost for direction changes (encourages straighter paths, default 1000)
- `via_proximity_cost`: Multiplier for stub proximity cost when placing vias. Higher values discourage vias near stubs. Use 0 to block vias entirely in stub proximity zones.
- `vertical_attraction_radius`: Grid units radius for cross-layer track attraction (0 = disabled)
- `vertical_attraction_bonus`: Cost reduction for positions aligned with tracks on other layers
- `layer_costs`: Per-layer cost multipliers (1000 = 1.0x, 3000 = 3.0x). Affects movement cost, source initialization, and via transitions. Switching to a cheaper layer discounts via cost (can be free)
- `proximity_heuristic_cost`: Expected proximity cost per grid step added to heuristic (0 = auto-compute). Auto-computed as `max(stub, track, bga) * 0.375 * 1000 / grid_step`. This tightens the heuristic for boards with high proximity costs, dramatically reducing search space (up to 6x speedup).

Methods:
- `route_multi(obstacles, sources, targets, max_iterations, collinear_vias=False, via_exclusion_radius=0)` - Find path from any source to any target
  - `collinear_vias`: If True, after a via the route must continue straight for one step, then can only turn ±45° (for differential pair routing)
  - `via_exclusion_radius`: Grid cells to exclude around placed vias. Prevents the route from drifting near its own vias, which is important for diff pair routing where P/N tracks are offset from centerline.
  - Returns `(path, iterations, stats)` where:
    - `path`: `List[(gx, gy, layer)]` or `None`
    - `iterations`: Number of A* iterations
    - `stats`: Dict with search statistics (cells_expanded, cells_pushed, heuristic_ratio, expansion_ratio, etc.)
- `set_proximity_heuristic_cost(cost)` - Set the proximity heuristic cost before each route. Called by Python when source/target endpoints are inside a proximity zone.

### PoseRouter

Orientation-aware A* router using Dubins path length as heuristic. Used for differential pair centerline routing where start and end orientations are constrained by stub directions.

```python
router = PoseRouter(via_cost: int, h_weight: float, turn_cost: int, min_radius_grid: float,
                    via_proximity_cost: int = 50,
                    vertical_attraction_radius: int = 0, vertical_attraction_bonus: int = 0,
                    proximity_heuristic_cost: int = 0)
```

Parameters:
- `via_cost`: Cost for layer transitions (scaled by 1000)
- `h_weight`: Heuristic weight (>1 for faster but less optimal routes)
- `turn_cost`: Cost for 45° in-place turn (typically `min_radius * π/4 * 1000`)
- `min_radius_grid`: Minimum turning radius in grid units
- `via_proximity_cost`: Multiplier for stub proximity cost when placing vias (0 = block vias near stubs)
- `vertical_attraction_radius`: Grid units radius for cross-layer track attraction (0 = disabled)
- `vertical_attraction_bonus`: Cost reduction for positions aligned with tracks on other layers
- `proximity_heuristic_cost`: Expected proximity cost per grid step added to Dubins heuristic (0 = disabled). Tightens the heuristic for boards with high proximity costs, reducing search space. Note: Python passes 1/10th of the computed value for diff pairs due to the more constrained pose-based search.

Methods:
- `route_pose(obstacles, src_x, src_y, src_layer, src_theta, tgt_x, tgt_y, tgt_layer, tgt_theta, max_iterations, diff_pair_via_spacing=None)`
  - `src_theta`, `tgt_theta`: Direction indices 0-7 (0=East, 1=NE, 2=North, ..., 7=SE)
  - `diff_pair_via_spacing`: Optional grid units for P/N via offset check. When set, via placement verifies that both +offset and -offset positions perpendicular to heading are clear.
  - Returns `(path, iterations)` where path is `List[(gx, gy, theta_idx, layer)]` or `None`
- `route_pose_with_frontier(...)` - Same as `route_pose` but returns blocked cells on failure for blocking analysis
  - Returns `(path, iterations, blocked_cells)` where `blocked_cells` is a list of `(gx, gy, layer)` tuples
- `set_proximity_heuristic_cost(cost)` - Set the proximity heuristic cost before each route. Called by Python when source/target endpoints are inside a proximity zone.

Constraints enforced by PoseRouter:
- **First move straight**: First move from start must be in the start direction (no immediate turn)
- **Minimum turn radius**: Enforced for ALL route steps - turns must respect `min_radius_grid` to ensure smooth paths
- **Straight after via**: After placing a via, must continue straight for `min_radius_grid + 1` steps before turning (ensures P/N offset tracks clear vias before turning)
- **Diff pair via clearance**: When `diff_pair_via_spacing` is set, checks that P/N via positions (perpendicular offsets from centerline) are clear
- **Via proximity cost**: When `via_proximity_cost > 0`, vias near stubs incur a cost penalty instead of being blocked

The Dubins heuristic computes the shortest path length considering:
- Start and end positions
- Required start and end headings
- Minimum turning radius constraint

This produces smoother routes that properly respect entry/exit angles at pads.

## Architecture

### Source Files

```
src/
├── lib.rs           # Module declarations, Python bindings
├── types.rs         # Shared types: GridState, OpenEntry, PoseState, constants
├── obstacle_map.rs  # GridObstacleMap implementation
├── router.rs        # GridRouter A* implementation
├── visual_router.rs # VisualRouter for debugging/visualization
├── dubins.rs        # Dubins path calculator for orientation heuristic
└── pose_router.rs   # PoseRouter for orientation-aware routing
```

### Key Components

- **GridObstacleMap**: Pre-computed obstacle data using reference-counted FxHashMap for O(1) lookups and correct incremental updates
- **GridRouter**: A* implementation with binary heap and packed state keys
- **PoseRouter**: Orientation-aware A* with Dubins path heuristic
- **DubinsCalculator**: Computes shortest Dubins path length (LSL, RSR, LSR, RSL, RLR, LRL)
- **State keys**: Packed into u64 for fast hashing (20 bits x, 20 bits y, 8 bits layer)
- **Hash function**: Uses rustc-hash (FxHash) for faster integer hashing than default SipHash
- **Costs**: ORTHO_COST=1000, DIAG_COST=1414 (sqrt(2) * 1000), DEFAULT_TURN_COST=1000

## Version History

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
- **0.18.0**: The rust-router-review batch (issues #384/#385/#386/#387). Verified byte-identical vs 0.17.4 on a 3-board identity gate (bitaxe_ultra / urchin / castor_pollux: `total_iterations` + `total_vias` + all JSON_SUMMARY keys EXACT) for everything except the two explicitly behavior-changing items called out below. (Audit note for future gates: Python-side additive JSON_SUMMARY keys — e.g. `blockers`, #409, present only on runs with attributable failures — are not produced by the Rust router; compare them key-by-key across Python releases, absent-vs-present is not a Rust identity failure.)
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

## Pad Rotation and Rectangular Blocking

The `route.py` script uses rectangular pad blocking with proper rotation handling:

```python
# Pads are blocked with rectangular bounds based on their rotated dimensions
# For a QFN pad with size (0.875, 0.2) and 90° rotation:
# - Board-space size becomes (0.2, 0.875)
# - Via blocking zone: pad_half_x + via_radius + clearance in X
#                      pad_half_y + via_radius + clearance in Y

for pad in pads:
    # size_x and size_y are already rotated by kicad_parser.py
    # Use int() instead of round() to avoid over-blocking
    via_expand_x = int((pad.size_x / 2 + via_clear_mm) / grid_step)
    via_expand_y = int((pad.size_y / 2 + via_clear_mm) / grid_step)
    for ex in range(-via_expand_x, via_expand_x + 1):
        for ey in range(-via_expand_y, via_expand_y + 1):
            obstacles.add_blocked_via(gx + ex, gy + ey)
```

This ensures vias are only placed where they have proper clearance to all adjacent pads, even for tightly-spaced QFN pins (0.4mm pitch).

# Rust Grid Router

High-performance A* grid router implemented in Rust with Python bindings via PyO3.

**Current Version: 0.8.3**

## Features

- Grid-based A* pathfinding with octilinear routing (8 directions)
- Multi-source, multi-target routing
- Via cost and layer transitions
- BGA exclusion zone with allowed cell overrides
- Stub proximity costs to avoid blocking unrouted nets
- **Turn cost penalty** for direction changes - encourages straighter paths with fewer wiggles
- **Cross-layer track attraction** for vertical alignment - attracts routes to stack on top of tracks on other layers
- **Pose-based routing with Dubins heuristic** for differential pair centerlines (orientation-aware A*)
- **Rectangular pad obstacle blocking** with proper rotation handling
- **Collinear via constraint** for differential pair routing (ensures clean via geometry)
- **Via exclusion zones** to prevent routes from conflicting with their own vias (for diff pair P/N offset tracks)
- ~10x speedup vs Python implementation

## Building

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

### Build Commands

**Recommended: Use the build script from the parent directory:**
```bash
python build_router.py
```

This builds the Rust module, copies the library to the correct location, and verifies the version.

To clean all build artifacts and compiled libraries:
```bash
python build_router.py --clean
```

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

path, iterations = router.route_multi(obstacles, sources, targets, max_iterations=200000)

if path:
    print(f"Found path with {len(path)} points in {iterations} iterations")
    for gx, gy, layer in path:
        print(f"  ({gx}, {gy}) on layer {layer}")
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
- `clone()` - Create a deep copy of the obstacle map (for incremental caching)

### GridRouter

```python
router = GridRouter(via_cost: int, h_weight: float, turn_cost: int = 1000, via_proximity_cost: int = 1,
                    vertical_attraction_radius: int = 0, vertical_attraction_bonus: int = 0)
```

Parameters:
- `via_cost`: Cost for layer transitions (scaled by 1000)
- `h_weight`: Heuristic weight (>1 for faster but less optimal routes)
- `turn_cost`: Cost for direction changes (encourages straighter paths, default 1000)
- `via_proximity_cost`: Multiplier for stub proximity cost when placing vias. Higher values discourage vias near stubs. Use 0 to block vias entirely in stub proximity zones.
- `vertical_attraction_radius`: Grid units radius for cross-layer track attraction (0 = disabled)
- `vertical_attraction_bonus`: Cost reduction for positions aligned with tracks on other layers

Methods:
- `route_multi(obstacles, sources, targets, max_iterations, collinear_vias=False, via_exclusion_radius=0)` - Find path from any source to any target
  - `collinear_vias`: If True, after a via the route must continue straight for one step, then can only turn ±45° (for differential pair routing)
  - `via_exclusion_radius`: Grid cells to exclude around placed vias. Prevents the route from drifting near its own vias, which is important for diff pair routing where P/N tracks are offset from centerline.
  - Returns `(path, iterations)` where path is `List[(gx, gy, layer)]` or `None`

### PoseRouter

Orientation-aware A* router using Dubins path length as heuristic. Used for differential pair centerline routing where start and end orientations are constrained by stub directions.

```python
router = PoseRouter(via_cost: int, h_weight: float, turn_cost: int, min_radius_grid: float,
                    via_proximity_cost: int = 50,
                    vertical_attraction_radius: int = 0, vertical_attraction_bonus: int = 0)
```

Parameters:
- `via_cost`: Cost for layer transitions (scaled by 1000)
- `h_weight`: Heuristic weight (>1 for faster but less optimal routes)
- `turn_cost`: Cost for 45° in-place turn (typically `min_radius * π/4 * 1000`)
- `min_radius_grid`: Minimum turning radius in grid units
- `via_proximity_cost`: Multiplier for stub proximity cost when placing vias (0 = block vias near stubs)
- `vertical_attraction_radius`: Grid units radius for cross-layer track attraction (0 = disabled)
- `vertical_attraction_bonus`: Cost reduction for positions aligned with tracks on other layers

Methods:
- `route_pose(obstacles, src_x, src_y, src_layer, src_theta, tgt_x, tgt_y, tgt_layer, tgt_theta, max_iterations, diff_pair_via_spacing=None)`
  - `src_theta`, `tgt_theta`: Direction indices 0-7 (0=East, 1=NE, 2=North, ..., 7=SE)
  - `diff_pair_via_spacing`: Optional grid units for P/N via offset check. When set, via placement verifies that both +offset and -offset positions perpendicular to heading are clear.
  - Returns `(path, iterations)` where path is `List[(gx, gy, theta_idx, layer)]` or `None`
- `route_pose_with_frontier(...)` - Same as `route_pose` but returns blocked cells on failure for blocking analysis
  - Returns `(path, iterations, blocked_cells)` where `blocked_cells` is a list of `(gx, gy, layer)` tuples

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

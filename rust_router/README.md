# Rust Grid Router

High-performance A* grid router implemented in Rust with Python bindings via PyO3.

**Current Version: 0.5.0**

## Features

- Grid-based A* pathfinding with octilinear routing (8 directions)
- Multi-source, multi-target routing
- Via cost and layer transitions
- BGA exclusion zone with allowed cell overrides
- Stub proximity costs to avoid blocking unrouted nets
- **Rectangular pad obstacle blocking** with proper rotation handling
- **Collinear via constraint** for differential pair routing (ensures clean via geometry)
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

path, iterations = router.route_multi(obstacles, sources, targets, max_iterations=100000)

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
python route.py fanout_starting_point.kicad_pcb routed.kicad_pcb \
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
- `add_blocked_via(gx, gy)` - Mark a position as blocked for vias
- `set_bga_zone(min_gx, min_gy, max_gx, max_gy)` - Set BGA exclusion zone
- `add_allowed_cell(gx, gy)` - Override BGA zone blocking for a cell
- `set_stub_proximity(gx, gy, cost)` - Set proximity cost for a cell
- `is_blocked(gx, gy, layer)` - Check if cell is blocked
- `is_via_blocked(gx, gy)` - Check if via position is blocked
- `get_stub_proximity_cost(gx, gy)` - Get proximity cost for a cell
- `clone()` - Create a deep copy of the obstacle map (for incremental caching)

### GridRouter

```python
router = GridRouter(via_cost: int, h_weight: float)
```

Methods:
- `route_multi(obstacles, sources, targets, max_iterations, collinear_vias=False)` - Find path from any source to any target
  - `collinear_vias`: If True, after a via the route must continue straight for one step, then can only turn ±45° (for differential pair routing)
  - Returns `(path, iterations)` where path is `List[(gx, gy, layer)]` or `None`

## Architecture

- **GridObstacleMap**: Pre-computed obstacle data using FxHashSet for O(1) lookups
- **GridRouter**: A* implementation with binary heap and packed state keys
- **State keys**: Packed into u64 for fast hashing (20 bits x, 20 bits y, 8 bits layer)
- **Hash function**: Uses rustc-hash (FxHash) for faster integer hashing than default SipHash
- **Costs**: ORTHO_COST=1000, DIAG_COST=1414 (sqrt(2) * 1000)

## Version History

- **0.5.0**: Added `collinear_vias` parameter for differential pair routing - ensures vias are placed on straight sections with gradual exit turns
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

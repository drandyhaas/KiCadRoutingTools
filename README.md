# KiCad PCB Grid Router

A fast Rust-accelerated A* autorouter for KiCad PCB files using integer grid coordinates. Routes nets between existing stub endpoints with multi-layer support and automatic via placement.

## Features

- **Grid-based A* pathfinding** - Integer coordinates for fast collision detection
- **Octilinear routing** - Horizontal, vertical, and 45-degree diagonal moves
- **Multi-layer routing** - Configurable layers with automatic via insertion
- **Differential pair routing** - Centerline-based routing with automatic P/N pairing, polarity swap handling, and via pairs
- **Obstacle avoidance** - Respects existing tracks, vias, and pads with configurable clearance
- **Rectangular pad blocking** - Proper clearance for non-square pads with rotation handling
- **Auto-detected BGA exclusion zones** - Prevents vias under BGA packages (detected from footprint)
- **Auto-detected component types** - BGA and QFN/QFP components detected by footprint pattern
- **Multiple net ordering strategies** - Inside-out (default), MPS (crossing conflicts), or original order
- **Inside-out BGA routing** - Nets with BGA pads are automatically sorted by distance from center
- **MPS ordering** - Maximum Planar Subset algorithm minimizes crossing conflicts between nets
- **Via-to-via clearance** - Proper spacing between vias (separate from track clearance)
- **Batch routing** - Routes multiple nets sequentially, each avoiding previously routed tracks
- **Stub proximity avoidance** - Penalizes routes near unrouted stubs to prevent blocking
- **Incremental obstacle caching** - Base obstacles built once, cloned per-net for ~7x speedup
- **Pad-only routing** - Routes nets with only pads (no stubs) or pad+stub combinations

## Requirements

The router requires the Rust module for A* pathfinding. Use the build script:

```bash
python build_router.py
```

This script:
- Builds the Rust module with `cargo build --release`
- Copies the compiled library to the correct location (`rust_router/grid_router.pyd` on Windows, `.so` on Linux/macOS)
- Verifies the build by importing and printing the version
- Removes any stale module copies from the parent directory

For manual building, see [rust_router/README.md](rust_router/README.md).

## Files

| File | Description |
|------|-------------|
| `build_router.py` | Build script for Rust module (handles all platforms) |
| `batch_grid_router.py` | Main router - routes multiple nets using Rust A* |
| `kicad_parser.py` | Parses .kicad_pcb files into Python data structures (handles pad rotation) |
| `kicad_writer.py` | Generates KiCad S-expressions for segments and vias |
| `check_drc.py` | DRC checker for detecting clearance violations |
| `test_diffpair.py` | Test script for single differential pair routing |
| `test_all_diffpairs.py` | Batch test script for all differential pairs |
| `bga_fanout.py` | BGA fanout stub generation for differential pairs |
| `qfn_fanout.py` | QFN fanout stub generation |
| `list_nets.py` | List all net names on a component (sorted alphabetically) |
| `rust_router/` | Rust A* implementation with Python bindings |
| `pygame_visualizer/` | Real-time visualization with persistent route display |
| `MPS.html` | Interactive visualization of MPS crossing conflict algorithm |

## Quick Start

### Route Multiple Nets

```bash
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-(U2A-DATA_0)" "Net-(U2A-DATA_1)"
```

### Route with Wildcard Patterns

Use `*` and `?` wildcards to match multiple nets:

```bash
# Route all DATA nets (matches Net-(U2A-DATA_0) through Net-(U2A-DATA_31))
python batch_grid_router.py fanout_starting_point.kicad_pcb routed.kicad_pcb "Net-(U2A-DATA_*)"

# Route all nets containing "CLK"
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "*CLK*"

# Mix wildcards and explicit names
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-(U2A-DATA_*)" "Net-(U2A-CLK)"
```

### Net Ordering Strategies

The router supports three net ordering strategies via the `--ordering` flag:

```bash
# MPS ordering (default) - minimizes crossing conflicts between nets
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-(U2A-DATA_*)" --ordering mps

# Inside-out ordering - best for BGA fanout routing
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-(U2A-DATA_*)" --ordering inside_out

# Original ordering - routes nets in the order specified
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-(U2A-DATA_*)" --ordering original
```

#### Strategy Comparison (32 DATA nets)

| Metric | mps | inside_out | original |
|--------|-----|-----------|----------|
| Success Rate | 32/32 (100%) | 32/32 (100%) | 32/32 (100%) |
| Time | 4.06s | 4.08s | 4.60s |
| Iterations | 1,029,824 | 723,666 | 1,071,958 |
| Via Count | 52 | 47 | 50 |

**Recommendation**: Use `mps` (default) for general routing. Use `inside_out` for pure BGA fanout routing where nets radiate from center.

### Route 32 DATA Nets (Inside-Out Ordering is Automatic)

The router automatically detects BGA components and sorts nets with BGA pads inside-out (center pads first). With wildcards, this is now a single command:

```bash
python batch_grid_router.py fanout_starting_point.kicad_pcb routed_output.kicad_pcb "Net-(U2A-DATA_*)"
```

The router will output:
```
Auto-detected 3 BGA exclusion zone(s):
  IC1: (135.7, 94.2) to (152.3, 110.8)
  U3: (185.9, 93.5) to (204.9, 112.5)
  U1: (165.3, 95.5) to (175.5, 110.5)

Sorted 32 BGA nets inside-out (0 non-BGA nets unchanged)
```

### Disable BGA Exclusion Zones

For nets that need to route directly between pads inside BGA areas (e.g., LVDS signals between BGAs), use `--no-bga-zones` to disable the BGA exclusion zone blocking:

```bash
# Route LVDS nets that connect pads inside BGA packages
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "*lvds*" --no-bga-zones
```

## Command-Line Options

All routing parameters can be configured via command-line arguments:

```bash
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-(U2A-*)" [OPTIONS]
```

### Ordering and Strategy Options

| Option | Default | Description |
|--------|---------|-------------|
| `--ordering`, `-o` | `mps` | Net ordering: `mps`, `inside_out`, or `original` |
| `--direction`, `-d` | `forward` | Search direction: `forward`, `backwards`, or `random` |
| `--no-bga-zones` | (disabled) | Disable BGA exclusion zone detection |
| `--layers`, `-l` | `F.Cu In1.Cu In2.Cu B.Cu` | Routing layers to use |

### Track and Via Geometry

| Option | Default | Description |
|--------|---------|-------------|
| `--track-width` | `0.1` | Track width in mm |
| `--clearance` | `0.1` | Clearance between tracks in mm |
| `--via-size` | `0.3` | Via outer diameter in mm |
| `--via-drill` | `0.2` | Via drill size in mm |

### Router Algorithm Parameters

| Option | Default | Description |
|--------|---------|-------------|
| `--grid-step` | `0.1` | Grid resolution in mm |
| `--via-cost` | `25` | Penalty for placing a via (in grid steps) |
| `--max-iterations` | `100000` | Max A* iterations before giving up |
| `--heuristic-weight` | `1.5` | A* heuristic weight (higher = faster but less optimal) |

### Stub Proximity Penalty

| Option | Default | Description |
|--------|---------|-------------|
| `--stub-proximity-radius` | `1.5` | Radius around stubs to penalize routing (mm) |
| `--stub-proximity-cost` | `2.0` | Cost penalty near stubs (mm equivalent) |

### Example with Custom Parameters

```bash
# Route with wider tracks and larger clearance
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-(U2A-*)" \
    --track-width 0.15 --clearance 0.15 --via-size 0.4 --via-drill 0.25

# Route with higher via penalty (fewer layer changes)
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-(U2A-*)" \
    --via-cost 100

# Route with more iterations for difficult nets
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-(U2A-*)" \
    --max-iterations 500000 --heuristic-weight 1.2

# Use specific layers
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-(U2A-*)" \
    --layers F.Cu In1.Cu In2.Cu In8.Cu B.Cu
```

### Parameter Notes

- **clearance**: Edge-to-edge spacing between tracks. A 0.1mm clearance means tracks with 0.1mm width can pass with 0.2mm center-to-center spacing.
- **via-to-via spacing**: Automatically calculated as `via_size + clearance` (0.4mm with defaults). This is larger than track-to-via spacing to ensure proper clearance between via barrels.
- **heuristic_weight**: Values > 1.0 make the search greedier (faster but potentially suboptimal). 1.5 is a good balance.
- **max_iterations**: Increase for complex routes. Easy routes need ~200 iterations, hard routes may need 10,000+.
- **via_cost**: Higher values discourage layer changes. 25 means a via costs as much as 0.025 grid steps of travel (encourages using vias freely for optimal routing).
- **stub_proximity_radius/cost**: Routes passing near unrouted stub endpoints incur extra cost. This prevents early routes from blocking later ones. Vias near stubs are penalized 2x.
- **bga_exclusion_zones**: Automatically detected from BGA footprints in the PCB. Can be disabled with `--no-bga-zones`.
- **inside-out ordering**: Nets with pads inside a BGA zone are automatically sorted by distance from BGA center (closest first). This improves routing success for BGA escape routing.
- **mps ordering**: Maximum Planar Subset algorithm that detects crossing conflicts between nets and orders them to minimize blocking. See [MPS Algorithm](#mps-algorithm) section below.

## Performance

Tested on 32 DATA nets with BGA fanout:

| Metric | Value |
|--------|-------|
| Success Rate | **32/32 (100%)** |
| Total Time | ~3.5 seconds |
| Total Iterations | ~724,000 |
| DRC Violations | 0 (DATA nets) |

The batch router uses incremental obstacle caching for ~7x speedup over rebuilding obstacles from scratch for each net.

**Key features for high success rates:**
- **Stub proximity avoidance**: Routes are penalized for passing near unrouted stub endpoints, preventing early routes from blocking later ones
- **Forward-then-backwards search**: Try forward direction first, then backwards if forward fails. Some routes are only possible in one direction.
- **Rectangular pad blocking**: Proper clearance for non-square pads (e.g., QFN 0.2x0.875mm pads)
- **BGA zone blocking**: All layers blocked inside BGA exclusion zones (not just vias)

### Iterative Retry Strategy

When some nets fail, retry routing just the failed nets on the already-routed output file. This often succeeds because the routing order changes:

```bash
# Initial routing
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-A" "Net-B" "Net-C"

# Retry failed nets
python batch_grid_router.py output.kicad_pcb output.kicad_pcb "Net-B"
```

## How It Works

### 1. Grid Discretization

Coordinates are converted to integers: `grid_x = round(mm_x / grid_step)`

This enables fast set-based collision detection with O(1) lookups.

### 2. Obstacle Map

Before routing, all obstacles are rasterized to grid cells:
- Existing tracks (expanded by half-width + clearance)
- Existing vias (all layers blocked)
- Component pads with **rectangular bounds** (respects pad rotation for correct orientation)
- Via placement zones (tracks and pads block via placement with appropriate clearances)
- **Stub proximity cost field** (unrouted stub locations have decaying cost penalty)

The net being routed is excluded from obstacles.

**Pad Rotation Handling**: The parser correctly transforms pad dimensions based on rotation. For example, QFN pads defined as (0.875 x 0.2)mm with 90° rotation become (0.2 x 0.875)mm in board coordinates, ensuring rectangular via blocking zones are correctly oriented.

**Via Blocking**: Uses `int()` instead of `round()` for grid discretization to avoid over-blocking valid via positions.

### 3. A* Search (Rust)

States are `(grid_x, grid_y, layer_index)` tuples. Moves include:
- 8 directions on same layer (orthogonal + diagonal)
- Via transitions to adjacent layers

Cost = distance + via penalty + **stub proximity penalty**. Heuristic = octile distance to goal.

The Rust implementation uses:
- FxHashSet for O(1) obstacle lookups
- BinaryHeap with counter-based tie-breaking for deterministic ordering
- Packed u64 state keys for fast hashing

### 4. Path Output

The grid path is converted back to mm coordinates. Segments and vias are generated for the KiCad file.

## MPS Algorithm

The Maximum Planar Subset (MPS) ordering strategy identifies crossing conflicts between nets and orders them to minimize blocking. This is useful when nets have paths that geometrically cross each other.

### How It Works

1. **Find endpoints**: For each net, identify the two stub endpoint centroids (source and target)
2. **Angular projection**: Project all endpoints onto a circular boundary centered on the routing region
3. **Detect crossings**: Two nets A and B cross if their endpoints interleave on the boundary:
   - Order A1, B1, A2, B2 means the paths must cross
   - Order A1, A2, B1, B2 means paths don't cross (can be routed without conflict)
4. **Build conflict graph**: Create edges between all pairs of crossing nets
5. **Greedy ordering**: In rounds:
   - Select nets with fewest active conflicts
   - Add them to the routing order
   - Remove conflicting neighbors (they go to next round)
6. **Output**: Ordered list where non-conflicting nets are routed first

### Interactive Visualization

[Open `MPS.html` in a browser](MPS.html) to see the algorithm in action:
- Draw nets by clicking pairs of pins on the circular boundary
- Click "Convert to Graph" to build the conflict graph
- Click "Run Micro-Step" repeatedly to watch the greedy selection process
- Green nets are selected for "Layer 1" (route first), red nets go to "Layer 2" (route later)

### When to Use MPS

MPS ordering is most effective when:
- Nets connect pins that are interleaved (e.g., alternating connections between two chip edges)
- Routes would naturally cross if drawn as straight lines
- The routing region is relatively flat (not radial like BGA fanout)

For BGA fanout routing where nets radiate from center, `inside_out` ordering typically performs better.

## Differential Pair Routing

The router supports differential pair routing with automatic P/N track pairing, polarity swap detection, and via pair handling.

### Basic Usage

```bash
# Route LVDS differential pairs
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "*lvds_rx4_1*" \
    --diff-pairs "*lvds*" --no-bga-zones
```

### Command-Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `--diff-pairs` | (none) | Pattern to match differential pair net names (e.g., `*lvds*`) |
| `--diff-pair-centerline-setback` | `1.5` | Distance from via to start centerline routing (mm) |

### How It Works

1. **Pair Detection**: Nets matching the `--diff-pairs` pattern are grouped by base name. Nets ending in `_P` and `_N` are paired together (e.g., `lvds_rx4_1_P` and `lvds_rx4_1_N`).

2. **Centerline Routing**: The A* pathfinder routes the centerline between the P and N tracks. Both tracks are then generated at equal offsets from this centerline.

3. **Spacing Detection**: The P-N spacing is automatically detected from the source and target stub endpoints, ensuring the routed pair maintains the same spacing as the existing stubs.

4. **Polarity Detection**: The router detects if P and N positions are swapped between source and target (polarity swap) and handles the crossover automatically.

### Polarity Swap Handling

When differential pairs need to swap polarity (P becomes N and vice versa), the router:

1. **Detects polarity swap**: Compares P/N positions at source vs target endpoints
2. **Places via pairs**: When layer changes occur, places paired vias with correct spacing
3. **Executes crossover**: Uses a circular arc detour around the outer via to swap track positions

The crossover is implemented as:
- Inner track goes straight through the via
- Outer track arcs around the other via at 1.25× track-via clearance
- Arc continues until tracks are at proper differential pair spacing
- Interpolation ensures the arc ends at exactly the right distance for parallel tracks

### Via Pair Handling

When a differential pair changes layers:

1. **Via positions**: Vias are placed at the correct P-N spacing perpendicular to the track direction
2. **Short exit segments**: Small segments connect vias to the main track path
3. **Layer continuity**: Both P and N tracks transition together, maintaining spacing

### Testing Differential Pairs

Test scripts are provided for validating differential pair routing:

```bash
# Test a single differential pair
python test_diffpair.py lvds_rx4_1

# Test all differential pairs matching a pattern
python test_all_diffpairs.py --pattern "*lvds*"

# Test with verbose output
python test_all_diffpairs.py --verbose
```

The test scripts:
1. Route the specified differential pair(s)
2. Run DRC checking on the output
3. Report success/failure for each pair

### Example Output

```
[1/1] Routing diff pair /fpga_adc/lvds_rx4_1
  P: /fpga_adc/lvds_rx4_1_P (id=822)
  N: /fpga_adc/lvds_rx4_1_N (id=840)
----------------------------------------
  P-N spacing: src=0.216mm, tgt=0.217mm, using=0.216mm (offset=0.108mm)
  Centerline setback: 1.5mm
  Source direction: (-0.71, -0.71), target direction: (-0.32, -0.95)
Route found in 486 iterations, path: 486 -> 4 points
  Polarity swap with vias: using TARGET polarity p_sign=1
  Arc stopped at diff pair spacing after 12 segments (interpolated to dist=0.216mm)
  Circular arc detour: 13 points around N via at radius 0.375mm
  SUCCESS: 28 segments, 2 vias, 486 iterations (0.01s)
```

### Known Limitations

- **Polarity swap without vias**: If polarity swap is needed but no layer change occurs, the tracks will cross (known limitation)
- **Fixed arc radius**: The circular arc uses 1.25× track-via clearance, which works for most cases but may need adjustment for unusual geometries

## BGA Fanout Generation

The `bga_fanout.py` script generates differential pair fanout stubs from BGA pads to channel exit points. It creates 45° stubs that route pairs from their pads to vertical or horizontal channels between pad rows/columns.

### Basic Usage

```bash
python bga_fanout.py input.kicad_pcb --output output.kicad_pcb --component U3 \
    --nets "*/lvds*" --diff-pairs "*lvds*" --primary-escape vertical
```

### Command-Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `--output`, `-o` | (required) | Output PCB file |
| `--component`, `-c` | (auto) | Component reference (auto-detected if not specified) |
| `--layers`, `-l` | `F.Cu In1.Cu In2.Cu B.Cu` | Routing layers to use |
| `--width`, `-w` | `0.1` | Track width in mm |
| `--clearance` | `0.1` | Track clearance in mm |
| `--nets`, `-n` | (all) | Net patterns to include (wildcards supported) |
| `--diff-pairs`, `-d` | (none) | Differential pair patterns (e.g., `*lvds*`) |
| `--diff-pair-gap` | `0.1` | Gap between P/N traces in mm |
| `--exit-margin` | `0.5` | Distance past BGA boundary for stub endpoints |
| `--primary-escape`, `-p` | `horizontal` | Primary escape direction (`horizontal` or `vertical`) |
| `--force-escape-direction` | (disabled) | Only use the primary escape direction (no fallback to secondary) |
| `--rebalance-escape` | (disabled) | Rebalance escape directions for even distribution around chip |
| `--check-for-previous` | (disabled) | Skip pads with existing fanouts and avoid occupied channels |

### Features

- **Differential pair routing**: P/N traces are routed together with constant spacing
- **Multi-layer support**: Automatically distributes pairs across 4 layers to avoid collisions
- **45° stubs**: Traces exit pads at 45° angles to reach channel positions
- **Collision detection and resolution**: Detects collisions and reassigns layers to resolve them
- **Edge pair handling**: Pairs on BGA edges route directly outward
- **Alternate channel fallback**: When a channel is blocked, tries the one neighboring channel on the opposite side of the pad
- **Jogged route support**: When both primary and alternate channels are blocked, moves conflicting routes to a farther channel using a jogged path (45° stub → vertical/horizontal jog → farther channel)
- **Escape rebalancing**: Optional rebalancing spreads exits evenly around chip perimeter for easier routing
- **Incremental fanout**: With `--check-for-previous`, skips already-fanned-out pads and avoids occupied channels
- **Graceful failure handling**: Reports nets that cannot be routed without conflicts and removes them from output

### Escape Direction Assignment Algorithm

The fanout generator assigns escape directions to differential pairs and single-ended signals using a multi-pass algorithm that balances routing quality with collision avoidance.

#### Escape Directions

Each pad can escape in one of four directions:
- **Horizontal**: `left` or `right` (uses horizontal channels between pad rows)
- **Vertical**: `up` or `down` (uses vertical channels between pad columns)

The `--primary-escape` option sets which orientation is tried first (default: `horizontal`).

#### Pass 1: Edge Pairs (Fixed Assignment)

Pads on the outer edge of the BGA have fixed escape directions - they route directly outward:
- Left edge → `left`
- Right edge → `right`
- Top edge → `up`
- Bottom edge → `down`

Edge pairs always use the top layer (F.Cu) and are assigned first before inner pairs.

#### Pass 2: Primary Orientation

Inner pairs are sorted by distance to the primary escape edge (closest first, since they have fewer routing options). For each pair:

1. Get all escape options in the primary orientation (e.g., `left`/`right` for horizontal)
2. Include alternate channels (neighboring channels on opposite side of pad) as fallback options
3. Try each option, checking if the exit position is available on any inner layer (In1.Cu, In2.Cu, B.Cu)
4. If available, assign the pair to that channel/layer combination

#### Pass 3: Secondary Orientation (Fallback)

Pairs that couldn't be assigned in Pass 2 try the secondary orientation:

1. Get all escape options in the secondary orientation (e.g., `up`/`down` for vertical)
2. Try each option with the same availability check
3. If available, assign the pair

**Note**: This pass is skipped when `--force-escape-direction` is set.

#### Pass 4: Forced Assignment

Any pairs still unassigned are force-assigned:
- With `--force-escape-direction`: Uses the primary orientation (may cause overlaps)
- Without: Uses automatic direction selection (nearest edge)

A warning is printed for each forced assignment.

#### Pass 5: Rebalancing (Optional)

When `--rebalance-escape` is enabled, the algorithm attempts to balance the distribution of horizontal vs vertical escapes:

1. Count pairs assigned to each orientation
2. If one direction has significantly more pairs, identify candidates to switch
3. Candidates are pairs far from the primary edge but close to the secondary edge
4. Switch candidates to the underpopulated direction if a free channel/layer is available

#### Collision Resolution

After initial assignment, the generator detects collisions (overlapping tracks) and resolves them:

1. **Layer reassignment**: Move one of the colliding routes to a different layer
2. **Alternate channel**: Try the neighboring channel on the opposite side of the pad
3. **Jogged route**: Move a blocking route to a farther channel using a jogged path:
   - 45° stub from pad
   - Vertical/horizontal jog (one pitch distance)
   - Continue to farther channel
   - 45° exit jog

Routes that cannot be resolved are removed and reported as failures.

### Example Output

```
Found U3: BGA-529_23x23_19.0x19.0mm
  Channels: 22 horizontal, 22 vertical
  Found 56 differential pairs
  Escape direction distribution:
    up: 56
    down: 56
  Generated 399 track segments
  INFO: 50 potential collisions detected (will attempt to resolve)
    Reassigned /fpga_adc/lvds_rx1_7 to B.Cu
    Reassigned /fpga_adc/lvds_rx4_6 to In2.Cu
    Moved Net-(U2A-CLK) to jogged path on In2.Cu
    Rerouted net_321 after freeing channel on In2.Cu
  Rerouted 3 signals to alternate channels
  After resolution: 0 collisions remaining
    B.Cu: 14 routes
    F.Cu: 38 routes
    In1.Cu: 38 routes
    In2.Cu: 22 routes
```

## QFN Fanout Generation

The `qfn_fanout.py` script generates fanout stubs from QFN/QFP pads.

### Basic Usage

```bash
python qfn_fanout.py input.kicad_pcb --output output.kicad_pcb --component U1
```

### Command-Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `--output`, `-o` | (required) | Output PCB file |
| `--component`, `-c` | (auto) | Component reference (auto-detected if not specified) |
| `--layer`, `-l` | `F.Cu` | Routing layer |
| `--width`, `-w` | `0.1` | Track width in mm |
| `--clearance` | `0.1` | Track clearance in mm |
| `--nets`, `-n` | (all) | Net patterns to include (wildcards supported) |
| `--stub-length`, `-s` | (auto) | Stub length (default: chip width / 2) |

## Utility Scripts

### List Nets on a Component

The `list_nets.py` script prints all net names connected to a component, sorted alphabetically:

```bash
# List nets on auto-detected BGA component
python list_nets.py input.kicad_pcb

# List nets on a specific component
python list_nets.py input.kicad_pcb --component U3
```

### Command-Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `pcb` | (required) | Input PCB file |
| `--component`, `-c` | (auto) | Component reference (auto-detected BGA if not specified) |

Example output:
```
Auto-detected BGA component: U3

Nets on U3 (144 total):

  Net-(U2A-BE_0)
  Net-(U2A-BE_1)
  Net-(U2A-CLK)
  ...
```

## Limitations

- Requires existing stub tracks to identify connection points
- Grid-based (0.1mm default) - may miss very tight fits
- No length matching (differential pair support is available, see below)
- No push-and-shove (routes around obstacles, doesn't move them)
- No rip-up and reroute (failed nets stay failed)

## DRC Checking

The `check_drc.py` script verifies routed output has no clearance violations.

### Usage

```bash
python check_drc.py input.kicad_pcb [OPTIONS]
```

### Command-Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `pcb` | (required) | Input PCB file to check |
| `--clearance`, `-c` | `0.1` | Minimum clearance in mm |
| `--nets`, `-n` | (all) | Net patterns to focus on (fnmatch wildcards supported) |

### Examples

```bash
# Check all nets
python check_drc.py routed_output.kicad_pcb --clearance 0.1

# Check only specific nets (much faster for large PCBs)
python check_drc.py routed_output.kicad_pcb --nets "*lvds_rx3_10*"

# Check multiple net patterns
python check_drc.py routed_output.kicad_pcb --nets "*DATA*" "*CLK*"
```

### Net Filtering

The `--nets` option allows focusing DRC checks on specific nets, which is significantly faster for large PCBs with many nets. When net patterns are provided:

- Only violations involving at least one matching net are reported
- Net pairs where neither matches the filter are skipped entirely (not just filtered at output)
- Uses fnmatch wildcards: `*` matches any characters, `?` matches single character

This is useful for:
- Quickly checking newly routed nets without waiting for full PCB analysis
- Debugging specific differential pairs or signal groups
- Iterative routing where you only care about recently changed nets

The checker reports:
- **Segment-to-segment** violations (tracks too close on same layer)
- **Via-to-segment** violations (via too close to track)
- **Via-to-via** violations (vias too close together)

Note: Pre-existing differential pair routing (e.g., LVDS signals) may intentionally have tight spacing and will show as violations.

## Real-Time Visualizer

Watch the A* algorithm explore the routing grid in real-time using the PyGame visualizer:

```bash
pip install pygame-ce
python pygame_visualizer/run_visualizer.py input.kicad_pcb "Net-(U2A-DATA_*)"
```

Uses the same Rust A* engine as the batch router with identical results. Supports all `batch_grid_router.py` command-line options plus `--auto` for automatic net advancement.

See [pygame_visualizer/README.md](pygame_visualizer/README.md) for full documentation, keyboard controls, and examples.

## Dependencies

- Python 3.7+
- Rust toolchain (for building the router module)
- pygame-ce (optional, for visualizer)

# Power/Ground Plane Via Connections

The `route_planes.py` script creates copper pour zones and places vias to connect SMD pads on other layers to the plane.

## Overview

When creating a ground or power plane on an inner or bottom layer, SMD pads on other layers need via connections to reach the plane. This tool automates:

1. **Zone creation** - Creates a copper pour zone covering the board. Replaces existing zones for the same net/layer with new parameters
2. **Pad classification** - Identifies which pads need vias vs direct zone connection
3. **Via placement** - Places vias near pads, avoiding obstacles on all copper layers
4. **Trace routing** - Routes traces from offset vias to pads using A* pathfinding
5. **Blocker rip-up** - Optionally removes blocking nets to place more vias
6. **Automatic re-routing** - Optionally re-routes ripped nets after via placement
7. **Resistance analysis** - Calculates and displays plane resistance and max current capacity

## Basic Usage

```bash
# Create GND plane on bottom layer (outputs to input_routed.kicad_pcb)
python route_planes.py input.kicad_pcb --nets GND --plane-layers B.Cu

# Create GND plane, overwrite input file
python route_planes.py input.kicad_pcb --overwrite --nets GND --plane-layers B.Cu

# Create GND plane to specific output file
python route_planes.py input.kicad_pcb output.kicad_pcb --nets GND --plane-layers B.Cu

# Create multiple planes at once (each net paired with corresponding plane layer)
python route_planes.py input.kicad_pcb --nets GND +3.3V --plane-layers In1.Cu In2.Cu

# Create VCC plane on inner layer with larger vias
python route_planes.py input.kicad_pcb --nets VCC --plane-layers In2.Cu --via-size 0.5 --via-drill 0.4

# Rip up blocking nets and automatically re-route them
python route_planes.py input.kicad_pcb --nets GND +3.3V --plane-layers In1.Cu In2.Cu --rip-blocker-nets --reroute-ripped-nets

# Preview what would be placed without writing
python route_planes.py input.kicad_pcb --nets GND --plane-layers B.Cu --dry-run
```

## Command-Line Options

### Required Options

| Option | Description |
|--------|-------------|
| `--nets`, `-n` | Net name(s) for the plane(s). Can specify multiple (e.g., "GND" "+3.3V") |
| `--plane-layers`, `-p` | Plane layer(s) for the zone(s), one per net (e.g., "In1.Cu" "In2.Cu") |

When specifying multiple nets, each net is paired with its corresponding plane layer. For example, `--nets GND VCC --plane-layers In1.Cu In2.Cu` creates a GND plane on In1.Cu and a VCC plane on In2.Cu.

### Via/Trace Geometry

| Option | Default | Description |
|--------|---------|-------------|
| `--via-size` | 0.5 | Via outer diameter in mm |
| `--via-drill` | 0.3 | Via drill size in mm |
| `--track-width` | 0.3 | Track width for via-to-pad connections in mm |
| `--clearance` | 0.25 | Clearance from other copper in mm |

### Zone Options

| Option | Default | Description |
|--------|---------|-------------|
| `--zone-clearance` | 0.2 | Zone clearance from other copper in mm |
| `--min-thickness` | 0.1 | Minimum zone copper thickness in mm |

### Algorithm Options

| Option | Default | Description |
|--------|---------|-------------|
| `--grid-step` | 0.1 | Grid resolution in mm |
| `--max-search-radius` | 10.0 | Maximum radius to search for valid via position in mm |
| `--max-via-reuse-radius` | 1.0 | Prefer reusing existing vias within this radius in mm |
| `--hole-to-hole-clearance` | 0.2 | Minimum clearance between drill holes in mm |
| `--layers`, `-l` | F.Cu + plane-layers + B.Cu | All copper layers for routing and via span (auto-computed) |
| `--layer-costs` | 1.0 per layer (4+) or F.Cu=1.0/B.Cu=3.0 (2-layer) | Per-layer routing cost multipliers (1.0-1000) |

### Multi-net Plane Layer Options

These options control MST-based routing between vias when multiple nets share the same plane layer.

| Option | Default | Description |
|--------|---------|-------------|
| `--plane-proximity-radius` | 3.0 | Radius around other nets' vias for proximity cost (mm) |
| `--plane-proximity-cost` | 2.0 | Maximum proximity cost around other nets' vias (mm equivalent) |
| `--plane-track-via-clearance` | 0.8 | Clearance from MST track center to other nets' via centers (mm) |
| `--voronoi-seed-interval` | 2.0 | Sample interval for Voronoi seed points along routes (mm) |
| `--plane-max-iterations` | 200000 | Maximum A* iterations for routing plane connections |

The `--plane-track-via-clearance` parameter ensures MST routes don't pass through narrow gaps between other nets' vias. A larger value ensures more room for polygon fill but may cause routing failures on dense boards.

### Blocker Rip-up Options

| Option | Default | Description |
|--------|---------|-------------|
| `--rip-blocker-nets` | off | Enable blocker identification and removal |
| `--max-rip-nets` | 3 | Maximum blocker nets to rip up per pad |
| `--reroute-ripped-nets` | off | Automatically re-route ripped nets after via placement |
| `--power-nets` | - | Glob patterns for power nets to route with wider tracks |
| `--power-nets-widths` | - | Track widths in mm for each power-net pattern |

When `--rip-blocker-nets` is enabled, if via placement or routing fails for a pad, the tool identifies which net is blocking and temporarily removes it from the PCB data. It then retries via placement. This process repeats up to `--max-rip-nets` times per pad.

When `--reroute-ripped-nets` is also enabled, after all plane vias are placed, the tool automatically re-routes the ripped nets using the batch router from `route.py`. This uses all copper layers specified by `--layers` for proper via clearance checking.

**Note:** The plane nets being processed are protected and will never be ripped up, even if they block each other during multi-net processing.

### Debug Options

| Option | Description |
|--------|-------------|
| `--dry-run` | Analyze and report without writing output file |
| `--verbose`, `-v` | Print detailed debug messages |
| `--debug-lines` | Draw MST routes on User.1, User.2, etc. per net |

## How It Works

### Pad Classification

The tool classifies each pad on the target net into three categories:

1. **Through-hole pads** - Have drill holes that connect all layers. No via needed.
2. **SMD pads on plane layer** - Already on the zone layer. No via needed.
3. **SMD pads on other layers** - Need a via + trace to connect to the plane.

### Via Placement Algorithm

For each pad needing a via, the algorithm:

1. **Check for nearby existing via** - If a via on the same net exists within `--max-via-reuse-radius`, reuse it
2. **Try pad center first** - If the pad center is not blocked, place via there (no trace needed)
3. **Spiral search outward** - Search in expanding rings for a valid position that:
   - Has clearance from existing vias, tracks, and pads on ALL copper layers
   - Can be routed to the pad using A* pathfinding
4. **Fallback to farther via** - If no new via position works, try reusing any existing via within `--max-search-radius`

### Via Obstacle Checking

Since vias are through-hole (spanning all layers), the obstacle map checks for clearance on **all copper layers**, not just the plane layer. This prevents DRC violations from vias overlapping tracks on inner layers.

### A* Trace Routing

When a via cannot be placed at the pad center, the tool routes a trace from the via to the pad using A* pathfinding. The routing:

- Avoids other-net pads and tracks
- Respects clearance requirements
- Uses the pad's layer for the trace

### Blocker Rip-up

When `--rip-blocker-nets` is enabled and via placement or routing fails:

1. **Identify blocker** - Analyzes what net is blocking:
   - For via placement failures: finds the nearest segment/via from another net
   - For routing failures: analyzes the A* frontier to find which net's obstacles are blocking the most cells
   - **Protected nets** (the plane nets being processed) are never identified as blockers

2. **Remove blocker** - Temporarily removes the blocking net's segments and vias from the PCB data

3. **Rebuild obstacles** - Rebuilds the obstacle maps without the ripped net (also re-blocks any vias already placed in this run)

4. **Retry placement** - Attempts via placement and routing again

5. **Repeat** - If still blocked, identifies the next blocker and repeats (up to `--max-rip-nets` times)

The tool also uses a skip optimization: when routing fails from a particular via position, nearby positions (within 2x via-size) are skipped to avoid redundant attempts.

### Automatic Re-routing

When `--reroute-ripped-nets` is enabled, after all plane vias are placed:

1. The tool collects all ripped net names
2. Calls `batch_route` from `route.py` with the ripped nets
3. Uses all copper layers (`--layers`) for routing and via clearance checking
4. Reports how many nets were successfully re-routed

This ensures that ripped nets are re-routed while respecting clearance from the newly placed plane vias on all layers.

### Multi-Net Layer Zone Generation

When multiple nets share the same plane layer (e.g., `--nets "VA19|VA11" --plane-layers In5.Cu`), the tool uses MST-based routing to ensure connected Voronoi zones:

1. **Compute MST** - For each net, computes a Minimum Spanning Tree between all its vias
2. **Route MST edges** - Routes each MST edge on the plane layer using A* pathfinding, avoiding other nets' vias and previously routed paths
3. **Retry with reordering** - If some edges fail to route, retries with failed nets processed first (up to 5 iterations), keeping the best result
4. **Sample routes for Voronoi** - Samples points along successful routes as additional Voronoi seed points
5. **Compute final zones** - Uses Voronoi diagram with augmented seeds to create non-overlapping zone polygons per net

The MST routes ensure that each net's zone polygons are connected (if routing succeeds). The `--debug-lines` option outputs the MST routes on User.1, User.2, etc. for visualization.

### Plane Resistance Analysis

After zone creation, the tool calculates and displays approximate plane resistance and maximum current capacity for each polygon:

**For single-net layers:**
- Uses the bounding box diagonal as the path length
- Samples polygon width perpendicular to the diagonal path

**For multi-net layers:**
- Uses the longest path through the MST (tree diameter) as the path length
- Follows the actual routed traces, not straight-line distance
- Samples polygon width perpendicular to the path at 1mm intervals

**Calculations:**
- **Resistance:** `R = ρ × L / (W × t)` where ρ = 1.68×10⁻⁸ Ω·m (copper), L = path length, W = average width, t = copper thickness
- **Max current:** IPC-2152 formula `I = k × ΔT^0.44 × A^0.725` where k = 0.024 for internal layers, 0.048 for external, A = cross-sectional area in mils²

**Assumptions:**
- 1 oz copper (35 µm thickness)
- 10°C maximum temperature rise

**Example output (single-net layer):**
```
Plane Resistance Analysis (1 oz copper, 10°C rise):
  Path length: 117.0 mm (diagonal)
  Avg width:   52.2 mm
  Resistance:  1.075 mΩ
  Max current: 21.05 A
```

**Example output (multi-net layer):**
```
Plane Resistance Analysis (1 oz copper, 10°C rise):
----------------------------------------------------------------------
Net                       Path(mm)   AvgW(mm)   R(mΩ)      Imax(A)
----------------------------------------------------------------------
/fpga_adc/VA19            33.6       3.6        4.457      3.03
/fpga_adc/VA11            96.3       5.7        8.121      4.22
/fpga_adc/VLVDS           28.4       78.1       0.175      28.18
/fpga_adc/VD11            31.7       4.3        3.537      3.44
----------------------------------------------------------------------
Path = longest MST route, AvgW = avg polygon width along path
```

This helps identify potential current bottlenecks in power distribution networks. Narrow polygon sections (low AvgW) will have higher resistance and lower current capacity.

## Error Messages

The tool provides clear error messages in red:

### VIA PLACEMENT FAILED
```
VIA PLACEMENT FAILED - no valid position within 10.0mm of pad at (x, y)
```
No position was found where a via could be placed. All candidate positions within the search radius were blocked by existing vias, tracks, or pads.

### ROUTING FAILED
```
ROUTING FAILED - no path from via (vx, vy) to pad at (px, py)
```
A via position was found (or an existing via was selected), but the A* router could not find a valid path from the via to the pad. This typically happens on dense boards where routing channels are blocked.

## Tips for Dense Boards

1. **Reduce via size** - Smaller vias have more placement options
   ```bash
   --via-size 0.4 --via-drill 0.3
   ```

2. **Increase search radius** - Search farther for valid positions
   ```bash
   --max-search-radius 15.0
   ```

3. **Reduce reuse radius** - Place more local vias instead of routing long distances
   ```bash
   --max-via-reuse-radius 0.5
   ```

4. **Run after other routing** - Place plane vias last so they work around existing routes

5. **Use blocker rip-up with re-routing** - For maximum via placement, enable both options:
   ```bash
   --rip-blocker-nets --reroute-ripped-nets --max-rip-nets 5
   ```

## Example Output

### Single Net

```
Loading PCB from input.kicad_pcb...
Found net 'GND' with ID 91
Board bounds: (71.12, 55.88) to (228.60, 147.32)

============================================================
Processing net 'GND' on layer B.Cu
============================================================

Pad analysis for net 'GND':
  Through-hole pads (no via needed): 31
  SMD pads on B.Cu (no via needed): 16
  SMD pads on other layers (via needed): 81

Building obstacle map for via placement...
  Pad C107.2... via at pad center
  Pad U102.3... via at (140.40, 92.70), 12 segs
  Pad U102.15... via at (134.40, 92.70), 12 segs
  ...

Results for 'GND':
  Zone created on B.Cu
  New vias placed: 74
  Existing vias reused: 6
  Traces added: 362

============================================================
OVERALL TOTALS
============================================================
  Nets processed: 1
  Total new vias placed: 74
  Total existing vias reused: 6
  Total traces added: 362

Writing output to output.kicad_pcb...
Output written to output.kicad_pcb
Note: Open in KiCad and press 'B' to refill zones
```

### Multiple Nets

```
Loading PCB from input.kicad_pcb...
Found net 'GND' with ID 91
Found net '+3.3V' with ID 104
Board bounds: (71.12, 55.88) to (228.60, 147.32)

============================================================
Processing net 'GND' on layer In1.Cu
============================================================
...
Results for 'GND':
  Zone created on In1.Cu (replaced existing)
  New vias placed: 89
  Existing vias reused: 6
  Traces added: 358

============================================================
Processing net '+3.3V' on layer In2.Cu
============================================================
...
Results for '+3.3V':
  Zone created on In2.Cu (replaced existing)
  New vias placed: 83
  Existing vias reused: 5
  Traces added: 346

============================================================
OVERALL TOTALS
============================================================
  Nets processed: 2
  Total new vias placed: 172
  Total existing vias reused: 11
  Total traces added: 704
```

### With Blocker Rip-up and Re-routing

```
  Pad U102.3... blocked, trying rip-up... ripping /NET_A... via at (140.40, 92.70), ripped 1 nets
  Pad U102.15... via at pad center
  Pad U301.24... blocked, trying rip-up... ripping /NET_B... ripping /NET_C... FAILED after 2 rip-ups
  ...

Results for 'GND':
  New vias placed: 77
  Existing vias reused: 6
  Traces added: 380
  Failed pads: 1

============================================================
OVERALL TOTALS
============================================================
  Nets processed: 1
  Total new vias placed: 77
  Total existing vias reused: 6
  Total traces added: 380
  Total failed pads: 1

Writing output to output.kicad_pcb...
Output written to output.kicad_pcb
Note: Open in KiCad and press 'B' to refill zones

============================================================
Re-routing 3 ripped net(s)...
============================================================
Loading output.kicad_pcb...
...
Routing complete
  Single-ended:  2/3 routed (1 FAILED)
...

Re-routing complete: 2 routed, 1 failed in 15.32s
```

## Post-Processing

After running the tool:

1. **Open in KiCad** - Load the output file
2. **Refill zones** - Press `B` or use Edit > Fill All Zones to generate the copper pour
3. **Run DRC** - Verify no design rule violations
4. **Manual cleanup** - Address any failed placements or re-routes manually if needed

## Code Organization

The plane generation code is organized into several modules:

| Module | Description |
|--------|-------------|
| `route_planes.py` | Main CLI and orchestration - loads PCB, coordinates via placement per-net, and writes output |
| `plane_io.py` | I/O utilities - zone extraction, PCB file reading/writing, net ID resolution |
| `plane_obstacle_builder.py` | Obstacle map construction - builds grid-based maps for via placement and routing |
| `plane_blocker_detection.py` | Blocker detection and rip-up - identifies which nets are blocking via placement |
| `plane_zone_geometry.py` | Voronoi zone computation - computes non-overlapping zone polygons for multi-net layers |
| `plane_resistance.py` | Resistance analysis - calculates plane resistance and max current capacity |

### Key Functions

**route_planes.py:**
- `create_plane()` - Main orchestration function
- `find_via_position()` - Searches for valid via positions with routing verification
- `route_via_to_pad()` - A* routing from via to pad
- `route_plane_connection()` - Routes MST edges between vias on multi-net layers

**plane_io.py:**
- `extract_zones()` - Reads existing zones from PCB file
- `check_existing_zones()` - Validates zone conflicts
- `write_plane_output()` - Writes vias, traces, and zones to output file

**plane_obstacle_builder.py:**
- `build_via_obstacle_map()` - Creates obstacle map for via placement (all layers)
- `build_routing_obstacle_map()` - Creates obstacle map for single-layer routing
- `identify_target_pads()` - Classifies pads by connection type

**plane_blocker_detection.py:**
- `find_via_position_blocker()` - Identifies net blocking a via position
- `find_route_blocker_from_frontier()` - Identifies net blocking A* routing
- `try_place_via_with_ripup()` - Iterative rip-up and retry logic

**plane_zone_geometry.py:**
- `compute_zone_boundaries()` - Computes Voronoi-based zone polygons
- `find_polygon_groups()` - Groups adjacent polygons for connectivity analysis
- `sample_route_for_voronoi()` - Samples route paths for Voronoi seeding

**plane_resistance.py:**
- `analyze_single_net_plane()` - Calculates resistance using bounding box diagonal
- `analyze_multi_net_plane()` - Calculates resistance using longest MST path
- `find_mst_diameter_path()` - Finds longest path through MST (tree diameter)
- `calculate_average_width_along_path()` - Samples polygon width perpendicular to path
- `calculate_resistance()` - Computes R = ρL/Wt
- `calculate_max_current_ipc()` - Computes max current using IPC-2152 formula

## Repairing Disconnected Plane Regions

After power planes are created, regions may become effectively split due to vias and traces from other nets cutting through the plane. The `route_disconnected_planes.py` script detects these disconnected regions and routes tracks between them to ensure electrical continuity.

Key features:
- **Per-net processing** - Zones with the same net on multiple layers (e.g., GND on B.Cu and In1.Cu) are processed together, avoiding redundant routes since vias connect all layers
- **Cross-layer connectivity** - Uses vias and through-hole pads to track connectivity across all zone layers for a net
- **Wide track routing** - Tries track widths from max (2.0mm) down to min, using the widest that fits

### Basic Usage

```bash
# Auto-detect all zones in PCB and repair disconnected regions (outputs to input_routed.kicad_pcb)
python route_disconnected_planes.py input.kicad_pcb

# Auto-detect all zones, overwrite input
python route_disconnected_planes.py input.kicad_pcb --overwrite

# Auto-detect all zones to specific output file
python route_disconnected_planes.py input.kicad_pcb output.kicad_pcb

# Specific nets and layers
python route_disconnected_planes.py input.kicad_pcb --nets GND --plane-layers B.Cu

# Customize track width and clearance
python route_disconnected_planes.py input.kicad_pcb --max-track-width 1.0 --clearance 0.2

# Increase iterations for difficult routes
python route_disconnected_planes.py input.kicad_pcb --max-iterations 500000
```

### Command-Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `--nets`, `-n` | auto | Net name(s) to process. If omitted, all nets with zones are processed |
| `--plane-layers`, `-p` | auto | Layer(s) to process. If omitted, all layers with zones are processed |
| `--layers`, `-l` | all Cu | Layer(s) available for routing |
| `--max-track-width` | 2.0 | Maximum track width for connections (mm) |
| `--min-track-width` | 0.2 | Minimum track width for connections (mm) |
| `--track-width` | 0.2 | Default track width for routing config (mm) |
| `--clearance` | 0.2 | Trace-to-trace clearance (mm) |
| `--zone-clearance` | 0.2 | Zone fill clearance around obstacles (mm) |
| `--track-via-clearance` | 0.8 | Clearance from tracks to other nets' vias (mm) |
| `--hole-to-hole-clearance` | 0.3 | Minimum clearance between drill holes (mm) |
| `--board-edge-clearance` | 0.5 | Clearance from board edge (mm) |
| `--via-size` | 0.5 | Via outer diameter (mm) |
| `--via-drill` | 0.4 | Via drill diameter (mm) |
| `--grid-step` | 0.1 | Routing grid step (mm) |
| `--analysis-grid-step` | 0.5 | Grid step for connectivity analysis (coarser = faster) |
| `--max-iterations` | 200000 | Maximum A* iterations per route attempt |
| `--dry-run` | off | Analyze without writing output |
| `--verbose`, `-v` | off | Print detailed debug messages |
| `--debug-lines` | off | Add debug lines on User.4 layer showing route paths |

### How It Works

#### 1. Region Detection (Flood Fill)

Uses flood fill on a coarse grid (`--analysis-grid-step`) to identify disconnected regions:

1. Build obstacle grid from other nets' vias, traces, and pads
2. Mark cells blocked by obstacles (with clearance)
3. Find all anchor points (vias + through-hole pads) for the target net
4. Flood fill from each anchor to identify connected regions
5. Group anchors by their connected region

#### 2. MST-Based Region Selection

Uses Kruskal's algorithm to build a Minimum Spanning Tree connecting all regions:

1. For each pair of regions, find the closest anchor points
2. Sort edges by distance
3. Build MST using union-find, selecting edges that connect previously unconnected regions

This ensures we connect all regions with minimum total trace length.

#### 3. Multi-Point A* Routing

For each MST edge connecting two regions, the router uses **multi-point A*** with all anchors from each region:

```
Region A (N anchors) <-> Region B (M anchors)
```

Instead of routing between the single closest pair, the router:
1. Sets ALL N anchors from region A as source cells
2. Sets ALL M anchors from region B as target cells
3. A* expands from all sources simultaneously, finding the best path to ANY target

This allows the router to find viable paths even when the closest anchor pair is blocked.

#### 4. Bidirectional Routing

If the forward search fails, the router tries the reverse direction:

1. **Direction 1:** Region A sources → Region B targets
2. **Direction 2:** Region B sources → Region A targets (if direction 1 fails)

A* can find different paths depending on which direction it expands from, so trying both directions increases success rate.

#### 5. Open-Space Fallback

If both directions fail, the router looks for "open space" points with maximum clearance:

1. Search near each region's centroid for cells with maximum distance from obstacles
2. Add these open-space points as additional anchors
3. Retry routing with the augmented anchor sets

If successful via an open-space point, a via is placed there to connect the route.

```
Example output:
[42/59] Region 6 (37 anchors) <-> Region 23 (2 anchors)... OK width=0.20mm, length=2.1mm (via open-space)
```

#### 6. Wide Track Routing

For power plane connections, wider tracks are preferred to minimize resistance. The router tries track widths from max to min:

1. Start with `--max-track-width` (default 2.0mm)
2. If routing fails, try half the width (1.0mm, 0.5mm, 0.25mm, ...)
3. Continue until `--min-track-width` (default 0.2mm) is reached
4. Use the widest width that successfully routes

This uses the Rust router's `track_margin` parameter to check extra clearance around the path without rebuilding the obstacle map, making width attempts fast.

```
Example output:
[1/3] Region 0 <-> Region 2... OK width=2.00mm, length=14.4mm
[2/3] Region 0 <-> Region 1... OK width=0.50mm, length=8.2mm (narrower due to obstacles)
[3/3] Region 0 <-> Region 3... OK width=1.00mm, length=12.1mm
```

#### 7. Via Placement at Layer Transitions

At layer transitions, the router checks if a via already exists at the transition point:

1. At each layer transition, check if an existing via is already at that exact position
2. If a via exists, no new via is added (the route uses the existing via)
3. If no via exists, a new via is placed at the transition point

Since routes can start and end on any layer at via locations, this simple approach avoids duplicate vias while keeping the routing logic straightforward.

#### 8. Incremental Obstacle Updates

After each successful route:

1. Add the route's segments to the obstacle map (blocks future routes)
2. Add new vias to the obstacle map with proper clearances:
   - Track-to-via clearance for routing
   - Hole-to-hole clearance for via placement
3. Add new vias to the reusable via list for subsequent routes

### Example Output

```
Loading PCB from input.kicad_pcb...
Board bounds: (71.12, 55.88) to (228.60, 147.32)

Routing disconnected plane regions
============================================================

[GND] on B.Cu, In1.Cu (clearances: B.Cu=0.3mm, In1.Cu=0.5mm):
  Building obstacle map... done
  Found 4 disconnected regions (131 total anchors)
  Routing 3 connection(s) to join regions...
    [1/3] Region 0 (2 anchors) <-> Region 2 (127 anchors)... OK width=1.00mm, length=52.2mm, 4 via(s)
    [2/3] Region 0 (2 anchors) <-> Region 1 (1 anchors)... OK width=0.25mm, length=14.4mm, 1 via(s)
    [3/3] Region 0 (2 anchors) <-> Region 3 (1 anchors)... OK width=0.50mm, length=13.0mm, 1 via(s)
  Result: All 3 route(s) succeeded

[+3.3V] on F.Cu, In2.Cu (clearances: F.Cu=0.2mm, In2.Cu=0.5mm):
  Building obstacle map... done
  Found 4 disconnected regions (128 total anchors)
  Routing 3 connection(s) to join regions...
    [1/3] Region 0 (114 anchors) <-> Region 1 (7 anchors)... OK width=1.00mm, length=10.3mm
    [2/3] Region 0 (114 anchors) <-> Region 3 (4 anchors)... OK width=2.00mm, length=14.4mm
    [3/3] Region 0 (114 anchors) <-> Region 2 (3 anchors)... OK width=2.00mm, length=16.5mm, 2 via(s)
  Result: All 3 route(s) succeeded

============================================================
SUMMARY
============================================================
  Zones processed: 4
  Total routes added: 6
  Total vias added: 8
```

Note: Zones with the same net on multiple layers are processed together (e.g., "GND on B.Cu, In1.Cu") since vias connect all layers. Per-layer zone clearances are shown and used for obstacle map construction.

### Why Routes Fail

Routes can fail for several reasons:

1. **All anchors blocked** - If every anchor point in a region is surrounded by obstacles with no path out
2. **Iteration limit reached** - Complex routes may exceed `--max-iterations` before finding a path
3. **No viable path exists** - Dense obstacle configurations may completely block all paths between regions

Increasing `--max-iterations` can help with complex routes. Reducing `--analysis-grid-step` provides finer region detection but is slower.

### Code Organization

| Module | Description |
|--------|-------------|
| `route_disconnected_planes.py` | CLI and orchestration - loads PCB, detects zones, coordinates region repair |
| `plane_region_connector.py` | Region detection and routing - flood fill analysis, multi-point A* routing, open-space fallback |

### Key Functions

**route_disconnected_planes.py:**
- `route_planes()` - Main orchestration: loads PCB, iterates over nets, writes output
- `auto_detect_zones()` - Scans PCB for existing zones and returns net/layer pairs

**plane_region_connector.py:**
- `find_disconnected_zone_regions()` - Flood fill to identify regions and their anchors
- `find_region_connection_points()` - Builds MST edges between regions
- `route_disconnected_regions()` - Orchestrates routing for one net/layer
- `route_plane_connection_wide()` - Multi-point A* routing between regions
- `find_open_space_point()` - Finds cell with maximum clearance from obstacles
- `build_base_obstacles()` - Builds obstacle map for routing
- `add_route_to_obstacles()` - Incrementally updates obstacles after each route

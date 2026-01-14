# Power/Ground Plane Via Connections

The `route_plane.py` script creates copper pour zones and places vias to connect SMD pads on other layers to the plane.

## Overview

When creating a ground or power plane on an inner or bottom layer, SMD pads on other layers need via connections to reach the plane. This tool automates:

1. **Zone creation** - Creates a copper pour zone covering the board (or uses existing zone)
2. **Pad classification** - Identifies which pads need vias vs direct zone connection
3. **Via placement** - Places vias near pads, avoiding obstacles on all copper layers
4. **Trace routing** - Routes traces from offset vias to pads using A* pathfinding
5. **Blocker rip-up** - Optionally removes blocking nets to place more vias
6. **Automatic re-routing** - Optionally re-routes ripped nets after via placement

## Basic Usage

```bash
# Create GND plane on bottom layer
python route_plane.py input.kicad_pcb output.kicad_pcb --net GND --plane-layer B.Cu

# Create multiple planes at once (each net paired with corresponding plane layer)
python route_plane.py input.kicad_pcb output.kicad_pcb --net GND +3.3V --plane-layer In1.Cu In2.Cu

# Create VCC plane on inner layer with larger vias
python route_plane.py input.kicad_pcb output.kicad_pcb --net VCC --plane-layer In2.Cu --via-size 0.5 --via-drill 0.4

# Rip up blocking nets and automatically re-route them
python route_plane.py input.kicad_pcb output.kicad_pcb --net GND +3.3V --plane-layer In1.Cu In2.Cu --rip-blocker-nets --reroute-ripped-nets

# Preview what would be placed without writing
python route_plane.py input.kicad_pcb output.kicad_pcb --net GND --plane-layer B.Cu --dry-run
```

## Command-Line Options

### Required Options

| Option | Description |
|--------|-------------|
| `--net`, `-n` | Net name(s) for the plane(s). Can specify multiple (e.g., "GND" "+3.3V") |
| `--plane-layer`, `-p` | Plane layer(s) for the zone(s), one per net (e.g., "In1.Cu" "In2.Cu") |

When specifying multiple nets, each net is paired with its corresponding plane layer. For example, `--net GND VCC --plane-layer In1.Cu In2.Cu` creates a GND plane on In1.Cu and a VCC plane on In2.Cu.

### Via/Trace Geometry

| Option | Default | Description |
|--------|---------|-------------|
| `--via-size` | 0.3 | Via outer diameter in mm |
| `--via-drill` | 0.2 | Via drill size in mm |
| `--track-width` | 0.2 | Track width for via-to-pad connections in mm |
| `--clearance` | 0.1 | Clearance from other copper in mm |

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
| `--layers`, `-l` | F.Cu In1.Cu In2.Cu B.Cu | All copper layers for routing and via span |

### Blocker Rip-up Options

| Option | Default | Description |
|--------|---------|-------------|
| `--rip-blocker-nets` | off | Enable blocker identification and removal |
| `--max-rip-nets` | 3 | Maximum blocker nets to rip up per pad |
| `--reroute-ripped-nets` | off | Automatically re-route ripped nets after via placement |

When `--rip-blocker-nets` is enabled, if via placement or routing fails for a pad, the tool identifies which net is blocking and temporarily removes it from the PCB data. It then retries via placement. This process repeats up to `--max-rip-nets` times per pad.

When `--reroute-ripped-nets` is also enabled, after all plane vias are placed, the tool automatically re-routes the ripped nets using the batch router from `route.py`. This uses all copper layers specified by `--layers` for proper via clearance checking.

**Note:** The plane nets being processed are protected and will never be ripped up, even if they block each other during multi-net processing.

### Debug Options

| Option | Description |
|--------|-------------|
| `--dry-run` | Analyze and report without writing output file |
| `--verbose`, `-v` | Print detailed debug messages |

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
  Using existing zone on In1.Cu
  New vias placed: 89
  Existing vias reused: 6
  Traces added: 358

============================================================
Processing net '+3.3V' on layer In2.Cu
============================================================
...
Results for '+3.3V':
  Using existing zone on In2.Cu
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
| `route_plane.py` | Main CLI and orchestration - loads PCB, coordinates via placement per-net, and writes output |
| `plane_io.py` | I/O utilities - zone extraction, PCB file reading/writing, net ID resolution |
| `plane_obstacle_builder.py` | Obstacle map construction - builds grid-based maps for via placement and routing |
| `plane_blocker_detection.py` | Blocker detection and rip-up - identifies which nets are blocking via placement |
| `plane_zone_geometry.py` | Voronoi zone computation - computes non-overlapping zone polygons for multi-net layers |
| `plane_create_helpers.py` | Helper functions for create_plane - pad processing, multi-net zones, result output |

### Key Functions

**route_plane.py:**
- `create_plane()` - Main orchestration function
- `find_via_position()` - Searches for valid via positions with routing verification
- `route_via_to_pad()` - A* routing from via to pad
- `route_plane_connection()` - Routes between disconnected zone regions

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

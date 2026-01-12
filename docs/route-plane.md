# Power/Ground Plane Via Connections

The `route_plane.py` script creates copper pour zones and places vias to connect SMD pads on other layers to the plane.

## Overview

When creating a ground or power plane on an inner or bottom layer, SMD pads on other layers need via connections to reach the plane. This tool automates:

1. **Zone creation** - Creates a copper pour zone covering the board (or uses existing zone)
2. **Pad classification** - Identifies which pads need vias vs direct zone connection
3. **Via placement** - Places vias near pads, avoiding obstacles on all copper layers
4. **Trace routing** - Routes traces from offset vias to pads using A* pathfinding

## Basic Usage

```bash
# Create GND plane on bottom layer
python route_plane.py input.kicad_pcb output.kicad_pcb --net GND --layer B.Cu

# Create VCC plane on inner layer with larger vias
python route_plane.py input.kicad_pcb output.kicad_pcb --net VCC --layer In2.Cu --via-size 0.5 --via-drill 0.4

# Preview what would be placed without writing
python route_plane.py input.kicad_pcb output.kicad_pcb --net GND --layer B.Cu --dry-run
```

## Command-Line Options

### Required Options

| Option | Description |
|--------|-------------|
| `--net`, `-n` | Net name for the plane (e.g., "GND", "VCC", "+3V3") |
| `--layer`, `-l` | Copper layer for the zone (e.g., "B.Cu", "In1.Cu", "In2.Cu") |

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
| `--all-layers` | F.Cu B.Cu | Copper layers for via span |

### Debug Options

| Option | Description |
|--------|-------------|
| `--dry-run` | Analyze and report without writing output file |

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

## Example Output

```
Loading PCB from input.kicad_pcb...
Found net 'GND' with ID 91
Board bounds: (71.12, 55.88) to (228.60, 147.32)

Pad analysis for net 'GND':
  Through-hole pads (no via needed): 31
  SMD pads on B.Cu (no via needed): 16
  SMD pads on other layers (via needed): 81

Building obstacle map for via placement...
  Finding via position for pad C107.2... new via at pad center (136.65, 118.68)
  Finding via position for pad U102.3... new via at (140.40, 92.70), routed 12 segs, 1.10mm
  Finding via position for pad U102.15... new via at (134.40, 92.70), routed 12 segs, 1.10mm
  ...

Results:
  Zone created on B.Cu
  New vias placed: 74
  Existing vias reused: 6
  Traces added: 362
  Failed via placements: 1

Writing output to output.kicad_pcb...
Output written to output.kicad_pcb
Note: Open in KiCad and press 'B' to refill zones
```

## Post-Processing

After running the tool:

1. **Open in KiCad** - Load the output file
2. **Refill zones** - Press `B` or use Edit > Fill All Zones to generate the copper pour
3. **Run DRC** - Verify no design rule violations
4. **Manual cleanup** - Address any failed placements manually if needed

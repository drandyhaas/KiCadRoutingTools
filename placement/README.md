# Placement

Component placement tool for KiCad PCB files. Spaces components evenly within the board boundary on an unrouted PCB, respecting courtyard boundaries.

## Usage

```bash
# Place components, output to input_placed.kicad_pcb
python place.py input.kicad_pcb

# Place to specific output file
python place.py input.kicad_pcb placed.kicad_pcb

# Overwrite input file
python place.py input.kicad_pcb -O

# Custom clearance and grid
python place.py input.kicad_pcb --clearance 0.5 --grid-step 0.05

# Verbose output (shows per-component placement details)
python place.py input.kicad_pcb -v
```

## Options

| Option | Default | Description |
|--------|---------|-------------|
| `--clearance` | 0.25 mm | Minimum gap between components |
| `--grid-step` | 0.1 mm | Grid resolution for snapping positions |
| `--overwrite, -O` | off | Overwrite input file |
| `--verbose, -v` | off | Print detailed placement info |

## Algorithm

1. Parse board boundary from Edge.Cuts layer
2. Extract courtyard boundaries (F.CrtYd/B.CrtYd) for each footprint; falls back to pad extents if no courtyard defined
3. Account for footprint rotation when computing placed dimensions (swap width/height for 90/270 degree rotations)
4. Sort components largest-first by area
5. Place in rows left-to-right, top-to-bottom within the usable board area
6. Snap all positions to the routing grid

## Module Structure

| File | Purpose |
|------|---------|
| `engine.py` | Core placement algorithm (row-based packing) |
| `parser.py` | Courtyard boundary extraction from PCB file |
| `writer.py` | Footprint position modification in output file |
| `utility.py` | Shared utilities (bbox from pads, grid snapping) |

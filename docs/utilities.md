# Utility Scripts

This document describes the utility scripts included with the KiCad Grid Router.

## DRC Checker (`check_drc.py`)

Checks a routed PCB for Design Rule Check violations.

### Usage

```bash
python check_drc.py output.kicad_pcb [OPTIONS]

Options:
  --clearance FLOAT    Track-to-track clearance in mm (default: 0.1)
  --via-clearance FLOAT  Via-to-track clearance in mm (uses --clearance if not set)
  --hole-to-hole-clearance FLOAT  Minimum drill hole edge-to-edge clearance in mm (default: 0.2)
  --board-edge-clearance FLOAT    Minimum clearance from board edge in mm (0 = use --clearance)
  --nets PATTERN       Only check nets matching pattern
  --debug-lines        Output debug lines on User.7 showing violation locations
```

### Examples

```bash
# Check with default clearances
python check_drc.py routed.kicad_pcb

# Check with tighter clearances
python check_drc.py routed.kicad_pcb --clearance 0.15

# Check with hole-to-hole clearance (for via drill spacing)
python check_drc.py routed.kicad_pcb --clearance 0.2 --hole-to-hole-clearance 0.2

# Check specific nets only
python check_drc.py routed.kicad_pcb --nets "*DATA*"
```

### Checks Performed

The DRC checker validates:

1. **Track-to-track clearance** - Segments on the same layer maintain minimum clearance
2. **Track-to-pad clearance** - Tracks maintain clearance from pads on other nets
3. **Via-to-track clearance** - Vias maintain clearance from tracks on other nets
4. **Via-to-via clearance** - Vias maintain clearance from each other
5. **Pad-to-via clearance** - Vias maintain clearance from pads on other nets
6. **Hole-to-hole clearance** - Drill holes (via drills and through-hole pad drills) maintain edge-to-edge clearance, even on the same net (manufacturing constraint)
7. **Board edge clearance** - Tracks and vias maintain clearance from the board outline
8. **Same-net crossings** - Detects tracks crossing on the same layer within a net

### Clearance Margin

The DRC checker uses a 5% clearance margin by default to ignore tiny geometric artifacts from corner handling. For example, with a 0.1mm clearance, violations smaller than 0.005mm are ignored.

### Debug Visualization

With `--debug-lines`, the checker outputs debug lines on the `User.7` layer in the PCB file, showing the closest points between violating segments:

```bash
python check_drc.py routed.kicad_pcb --debug-lines
```

Output includes the distance of each violation:

```
Debug line: (10.200, 15.300) to (10.205, 15.305), distance: 0.0025mm
Writing 3 debug lines to User.7 layer
```

Open the PCB in KiCad and enable the `User.7` layer to visualize where clearance violations occur.

### Output

```
Checking clearances in routed.kicad_pcb...
Track clearance: 0.1mm
Via clearance: 0.1mm

Checking 1,234 segments and 89 vias...

Found 0 DRC violations
```

If violations are found:

```
VIOLATION: Track too close to track
  Net-(U2A-DATA_0) segment (10.2, 15.3)-(10.4, 15.5) on F.Cu
  Net-(U2A-DATA_1) segment (10.3, 15.2)-(10.5, 15.4) on F.Cu
  Distance: 0.085mm (minimum: 0.1mm)
```

## Connectivity Checker (`check_connected.py`)

Verifies that all nets are fully connected after routing.

### Usage

```bash
python check_connected.py output.kicad_pcb [OPTIONS]

Options:
  --nets PATTERN       Only check nets matching pattern
  --component, -C REF  Check all nets connected to a component (e.g., U1)
  --tolerance FLOAT    Connection tolerance in mm (default: 0.02)
  --verbose            Show detailed break location info
  --quiet              Only print summary line unless issues found
```

### Examples

```bash
# Check all nets
python check_connected.py routed.kicad_pcb

# Check specific nets
python check_connected.py routed.kicad_pcb --nets "*lvds*"

# Check all nets on a component
python check_connected.py routed.kicad_pcb --component U102

# Check specific patterns on a component
python check_connected.py routed.kicad_pcb --component U1 --nets "*DATA*"

# Verbose output with break locations
python check_connected.py routed.kicad_pcb --verbose
```

### Output

```
Checking connectivity in routed.kicad_pcb...

Checking 64 nets...

All nets are fully connected!
```

If disconnected nets are found:

```
DISCONNECTED: Net-(U2A-DATA_5)
  Group 1: 3 segments near (10.2, 15.3)
  Group 2: 2 segments near (25.1, 30.4)
  Missing connection between groups
```

## Net Lister (`list_nets.py`)

Lists all nets connected to a component.

### Usage

```bash
python list_nets.py input.kicad_pcb REFERENCE [OPTIONS]

Options:
  --pattern GLOB    Filter nets by pattern
  --pads            Show pad assignments
```

### Examples

```bash
# List all nets on U2A
python list_nets.py board.kicad_pcb U2A

# List only DATA nets
python list_nets.py board.kicad_pcb U2A --pattern "*DATA*"

# Show pad assignments
python list_nets.py board.kicad_pcb U2A --pads
```

### Output

```
Nets on U2A:
  Net-(U2A-DATA_0)
  Net-(U2A-DATA_1)
  ...
  Net-(U2A-CLK)
  GND
  VCC

Total: 42 nets
```

With `--pads`:

```
Nets on U2A:
  A1: Net-(U2A-DATA_0)
  A2: Net-(U2A-DATA_1)
  A3: GND
  ...
```

## BGA Fanout Generator (`bga_fanout.py`)

Generates stub tracks for BGA differential pair fanout.

### Usage

```bash
python bga_fanout.py input.kicad_pcb output.kicad_pcb REFERENCE [OPTIONS]

Options:
  --pattern GLOB      Net pattern to fanout
  --stub-length FLOAT Stub length in mm (default: 0.5)
  --layer LAYER       Target layer (default: F.Cu)
```

### Example

```bash
# Generate fanout stubs for LVDS nets on U2A
python bga_fanout.py board.kicad_pcb fanout.kicad_pcb U2A --pattern "*lvds*"
```

### What It Does

Creates short stub tracks from BGA pads pointing outward:

```
Before:        After:
  o o o          o─ o─ o─
  o o o    =>    o─ o─ o─
  o o o          o─ o─ o─
```

These stubs give the router starting points for escape routing.

## QFN/QFP Fanout Generator (`qfn_fanout.py`)

Generates stub tracks for QFN/QFP package fanout.

### Usage

```bash
python qfn_fanout.py input.kicad_pcb output.kicad_pcb REFERENCE [OPTIONS]

Options:
  --pattern GLOB      Net pattern to fanout
  --stub-length FLOAT Stub length in mm (default: 0.3)
  --layer LAYER       Target layer (default: F.Cu)
```

### Example

```bash
# Generate fanout stubs for all signal nets on U3
python qfn_fanout.py board.kicad_pcb fanout.kicad_pcb U3
```

## Layer Switcher (`switch_to_layer.py`)

Moves all segments for matching nets to a specified layer, adding vias where needed.

### Usage

```bash
python switch_to_layer.py NET_PATTERNS --input input.kicad_pcb --output output.kicad_pcb --to-layer LAYER

Options:
  --input, -i FILE     Input PCB file
  --output, -o FILE    Output PCB file
  --to-layer, -l LAYER Target layer (e.g., "In3.Cu", "F.Cu", "B.Cu")
  --via-size FLOAT     Via outer diameter in mm (default: 0.3)
  --via-drill FLOAT    Via drill diameter in mm (default: 0.2)
  --dry-run, -n        Show what would be changed without writing output
```

### Examples

```bash
# Move all rx_top nets to In3.Cu
python switch_to_layer.py "*rx*top*" --input board.kicad_pcb --output modified.kicad_pcb --to-layer In3.Cu

# Preview changes without modifying
python switch_to_layer.py "*lvds*" -i board.kicad_pcb -o out.kicad_pcb -l B.Cu --dry-run
```

### What It Does

1. Finds all segments belonging to nets matching the patterns
2. Changes each segment's layer to the target layer
3. Adds vias at segment endpoints where layer transitions occur

Useful for post-routing layer adjustments when you want to move certain signals to a different layer.

## Build Script (`build_router.py`)

Builds the Rust router module.

### Usage

```bash
python build_router.py [OPTIONS]

Options:
  --release    Build in release mode (default)
  --debug      Build in debug mode
  --clean      Clean before building
```

### What It Does

1. Runs `cargo build --release` in `rust_router/`
2. Copies the compiled library to the correct location:
   - Windows: `grid_router.pyd`
   - Linux/Mac: `grid_router.so`

### Requirements

- Rust toolchain installed (`rustup`)
- PyO3 and maturin for Python bindings

## Board Data Extractor (`extract_board_data.py`)

Extracts board data for analysis or debugging.

### Usage

```bash
python extract_board_data.py input.kicad_pcb [OPTIONS]

Options:
  --output FILE    Output JSON file
  --nets PATTERN   Filter nets by pattern
```

### Example

```bash
# Extract all board data
python extract_board_data.py board.kicad_pcb --output board_data.json

# Extract specific nets
python extract_board_data.py board.kicad_pcb --nets "*DATA*" --output data_nets.json
```

### Output Format

```json
{
  "nets": {
    "1": {"name": "GND", "segments": 45, "vias": 12},
    "2": {"name": "VCC", "segments": 23, "vias": 8},
    ...
  },
  "components": [
    {"reference": "U2A", "footprint": "BGA-256", "pads": 256},
    ...
  ],
  "layers": ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"],
  "board_outline": {"min_x": 0, "min_y": 0, "max_x": 100, "max_y": 80}
}
```

## Test Scripts

### Full Fanout and Route Test (`test_fanout_and_route.py`)

Runs a complete integration test: fanout generation followed by routing and verification.

#### Usage

```bash
python test_fanout_and_route.py
```

This script runs a sequence of fanout and routing operations on the test board, including:
1. BGA fanout for multiple components (FTDI, ADC, FPGA, DDR)
2. Single-ended and differential pair routing
3. DRC and connectivity verification

#### Cross-Platform

This Python script replaces the previous PowerShell script and works on any platform (Linux, macOS, Windows).

## Common Workflows

### Route and Verify

```bash
# Route the nets
python route.py input.kicad_pcb routed.kicad_pcb "Net-*" --ordering mps

# Check for DRC violations
python check_drc.py routed.kicad_pcb

# Verify connectivity (all routed nets)
python check_connected.py routed.kicad_pcb

# Or verify connectivity for a specific component
python check_connected.py routed.kicad_pcb --component U1
```

### BGA Differential Pair Flow

```bash
# 1. Generate fanout stubs
python bga_fanout.py board.kicad_pcb fanout.kicad_pcb U2A --pattern "*lvds*"

# 2. Route differential pairs
python route.py fanout.kicad_pcb routed.kicad_pcb "*lvds*" \
    --diff-pairs "*lvds*" \
    --ordering inside_out

# 3. Verify results
python check_drc.py routed.kicad_pcb
python check_connected.py routed.kicad_pcb --nets "*lvds*"
```

### Debug Routing Issues

```bash
# List nets on problematic component
python list_nets.py board.kicad_pcb U2A --pads

# Route with debug lines enabled
python route.py input.kicad_pcb debug.kicad_pcb "Net-(*)" --debug-lines

# Open debug.kicad_pcb in KiCad, check User.3/4/8/9 layers
```

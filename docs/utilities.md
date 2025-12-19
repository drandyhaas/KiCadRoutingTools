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
  --nets PATTERN       Only check nets matching pattern
  --debug-lines        Output debug lines on User.7 showing violation locations
```

### Examples

```bash
# Check with default clearances
python check_drc.py routed.kicad_pcb

# Check with tighter clearances
python check_drc.py routed.kicad_pcb --clearance 0.15

# Check specific nets only
python check_drc.py routed.kicad_pcb --nets "*DATA*"
```

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
  --nets PATTERN    Only check nets matching pattern
  --verbose         Show connection details
```

### Examples

```bash
# Check all nets
python check_connected.py routed.kicad_pcb

# Check specific nets
python check_connected.py routed.kicad_pcb --nets "*lvds*"

# Verbose output
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

### Single Diff Pair Tester (`test_diffpair.py`)

Routes one or more differential pairs and runs DRC/connectivity checks.

#### Usage

```bash
python test_diffpair.py [PATTERNS...] [OPTIONS]

Options:
  --build              Build the Rust router before running
  --list               List all available diff pairs and exit
  --dry-run            Show which pairs would be routed without routing
  --skip-drc           Skip DRC checks after routing
  --skip-connectivity  Skip connectivity checks after routing
  --input FILE         Input PCB file (default: routed_output.kicad_pcb)
  --output FILE        Output PCB file (default: test_batch_diffpair.kicad_pcb)
```

#### Examples

```bash
# Route a single diff pair
python test_diffpair.py lvds_rx4_1

# Route using wildcard patterns
python test_diffpair.py lvds_rx1_?           # lvds_rx1_0 through lvds_rx1_9
python test_diffpair.py lvds_rx*_9           # All rx channel 9s
python test_diffpair.py 'lvds_rx1_*' 'lvds_rx2_*'  # Multiple patterns

# List available diff pairs
python test_diffpair.py --list

# Route with custom parameters
python test_diffpair.py lvds_rx2_? --stub-proximity-radius 2.0
```

### Batch Diff Pair Tester (`test_all_diffpairs.py`)

Tests all differential pairs in parallel with configurable thread count.

#### Usage

```bash
python test_all_diffpairs.py [OPTIONS]

Options:
  --pattern, -p PATTERN  Pattern to filter diff pairs (default: *lvds*)
  --threads, -t N        Number of parallel threads (default: 14)
  --verbose, -v          Show detailed output for each test
  --stop-on-error, -s    Stop on first error
  --build                Build the Rust router before running
  --skip-drc             Skip DRC checks after routing
  --skip-connectivity    Skip connectivity checks after routing
  --input FILE           Input PCB file (default: routed_output.kicad_pcb)
```

#### Examples

```bash
# Test all LVDS diff pairs with 14 threads (default)
python test_all_diffpairs.py

# Test with 8 threads
python test_all_diffpairs.py -t 8

# Test specific pattern
python test_all_diffpairs.py -p '*rx1*'

# Sequential execution (like single-threaded)
python test_all_diffpairs.py --threads 1

# Stop on first failure
python test_all_diffpairs.py --stop-on-error
```

#### Output

```
Loading routed_output.kicad_pcb to find differential pairs...
Found 56 differential pairs to test
Running with 14 threads
============================================================
[1/56] lvds_rx1_11: PASS
[2/56] lvds_rx1_13: PASS
...
[56/56] lvds_rx_top_clkin3: PASS

============================================================
SUMMARY
============================================================
Total:              56
Passed:             56
Known limitations:  0
Routing failed:     0
DRC failed:         0

All tests passed!
```

Polarity fixing is enabled by default, automatically swapping target pad assignments
when needed for 100% pass rate. Use `--no-fix-polarity` to disable.

## Common Workflows

### Route and Verify

```bash
# Route the nets
python route.py input.kicad_pcb routed.kicad_pcb "Net-*" --ordering mps

# Check for DRC violations
python check_drc.py routed.kicad_pcb

# Verify connectivity
python check_connected.py routed.kicad_pcb
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

# Open debug.kicad_pcb in KiCad, check User.2/3/8/9 layers
```

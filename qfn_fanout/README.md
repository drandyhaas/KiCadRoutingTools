# QFN/QFP Fanout Generator

Creates escape routing for QFN (Quad Flat No-leads) and QFP (Quad Flat Package) packages in KiCad PCB files.

## Features

- **Generic package support** - Works with any QFN/QFP package regardless of pin count or size
- **Automatic geometry detection** - Analyzes pad positions and sizes to determine layout
- **Two-segment stubs** - Straight segment + 45-degree fan-out for optimal endpoint separation
- **Side detection** - Automatically determines which side each pad is on
- **Collision validation** - Checks endpoint spacing after generation

## Usage

```bash
# Basic fanout
python qfn_fanout.py kicad_files/input.kicad_pcb --component U1 --output kicad_files/output.kicad_pcb

# With net filter
python qfn_fanout.py kicad_files/input.kicad_pcb --component U1 --output kicad_files/output.kicad_pcb --nets "*DATA*"

# Specify layer and track width
python qfn_fanout.py kicad_files/input.kicad_pcb --component U1 --output kicad_files/output.kicad_pcb \
    --layer F.Cu --width 0.15
```

## Options

| Option | Description | Default |
|--------|-------------|---------|
| `--component`, `-c` | Component reference | Auto-detect QFN/QFP |
| `--output`, `-o` | Output PCB file | qfn_fanout_test.kicad_pcb |
| `--layer`, `-l` | Routing layer | F.Cu |
| `--width`, `-w` | Track width (mm) | 0.1 |
| `--clearance` | Track clearance (mm) | 0.1 |
| `--nets`, `-n` | Net patterns to include | All nets |
| `--stub-length`, `-s` | Stub length (mm) | chip width / 2 |

## Module Structure

```
qfn_fanout/
├── __init__.py    # Main fanout logic and public API
├── layout.py      # Layout analysis functions
├── geometry.py    # Stub position calculations
└── types.py       # Data types: QFNLayout, PadInfo, FanoutStub
```

### Key Components

- **`generate_qfn_fanout()`** - Main function that generates fanout tracks
- **`analyze_qfn_layout()`** - Extracts package parameters from pad geometry
- **`analyze_pad()`** - Determines pad side and escape direction
- **`calculate_fanout_stub()`** - Calculates two-segment stub positions
- **`QFNLayout`** - Package layout parameters
- **`PadInfo`** - Analyzed pad information
- **`FanoutStub`** - Represents a fanout stub with two segments

## Algorithm

1. **Layout Analysis** - Extract package center, dimensions, and pitch from pad positions
2. **Pad Classification** - Determine which side (top/bottom/left/right) each pad is on
3. **Escape Direction** - Calculate perpendicular escape direction for each pad
4. **Stub Generation** - Create two-segment stubs:
   - Straight segment: perpendicular to chip edge (pad_length / 2)
   - 45-degree segment: fans outward, length varies by position (0 at center, max at corners)
5. **Collision Check** - Validate endpoint spacing meets clearance requirements

## Stub Pattern

```
Corner pads: Long 45° fan-out
    ╲
     ╲
      ●  (pad)

Center pads: Straight only (no 45°)
      |
      |
      ●  (pad)
```

The 45-degree segment length varies linearly:
- Corner pads: max_diagonal_length (= chip_width / 3)
- Center pads: 0 (no diagonal)
- Intermediate: linear interpolation based on position along edge

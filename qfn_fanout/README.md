# QFN/QFP Fanout Generator

Creates escape routing for QFN (Quad Flat No-leads) and QFP (Quad Flat Package) packages in KiCad PCB files.

## Features

- **Generic package support** - Works with any QFN/QFP package regardless of pin count or size
- **Automatic geometry detection** - Analyzes pad positions and sizes to determine layout
- **Rotation-agnostic** - Edge classification runs in the footprint's local frame and the
  escape follows each pad's own long axis, so packages placed at any board angle (including
  non-orthogonal, e.g. an LQFP144 at -135°) fan out correctly. The previous global-bounding-
  box edge detection saw a rotated part as a diamond and escaped almost none of its pads.
- **Two-segment stubs** - Straight segment + 45-degree fan-out for optimal endpoint separation
- **Side detection** - Automatically determines which side each pad is on
- **Collision validation** - Checks endpoint spacing after generation
- **Pad-geometry sanity check** - Runs `check_pads.py` on the component first; if its pads
  overlap (a sign the pad rotation/size is modelled wrong) it warns before escaping.

## Usage

```bash
# Basic fanout
python qfn_fanout.py kicad_files/input.kicad_pcb --component U1 --output kicad_files/output.kicad_pcb

# With net filter (include pattern)
python qfn_fanout.py kicad_files/input.kicad_pcb --component U1 --output kicad_files/output.kicad_pcb --nets "*DATA*"

# With exclusion pattern (all nets except GND and VCC)
python qfn_fanout.py kicad_files/input.kicad_pcb --component U1 --output kicad_files/output.kicad_pcb \
    --nets "*" "!GND" "!VCC"

# Specify layer and track width
python qfn_fanout.py kicad_files/input.kicad_pcb --component U1 --output kicad_files/output.kicad_pcb \
    --layer F.Cu --width 0.15
```

## Net Pattern Syntax

The `--nets` option supports fnmatch-style wildcards (`*`, `?`) and exclusion patterns:

| Pattern | Description |
|---------|-------------|
| `*DATA*` | Nets containing "DATA" |
| `/*` | Nets starting with "/" (hierarchical) |
| `!GND` | Exclude net named "GND" |
| `!*VCC*` | Exclude nets containing "VCC" |
| `"*" "!GND" "!VCC"` | All nets except GND and VCC |

**Note:** Nets starting with "unconnected-" (KiCad pins not connected in schematic) are automatically excluded.

## Options

| Option | Description | Default |
|--------|-------------|---------|
| `--component`, `-c` | Component reference | Auto-detect QFN/QFP |
| `--output`, `-o` | Output PCB file | qfn_fanout_test.kicad_pcb |
| `--layer`, `-l` | Routing layer | F.Cu |
| `--width`, `-w` | Track width (mm) | 0.1 |
| `--extension` | Extension past pad edge before bend (mm) | 0.1 |
| `--nets`, `-n` | Net patterns to include | All nets |
| `--escape-method` | `stub` (surface 45° fan) or `underpad` (via-drop) | stub |
| `--via-size` | Underpad escape via outer diameter (mm) | 0.45 |
| `--via-drill` | Underpad escape via drill diameter (mm) | 0.25 |
| `--allow-via-in-pad` | Underpad escape: let the via overlap its **own** pad so it can stagger *inward* (via-in-pad) | off |

### Under-pad (via-drop) escape — `--escape-method underpad` (issue #164)

The default `stub` fan spreads each pad laterally at 45°, so on a **crowded
fine-pitch edge** (a neighbour pair on one side, a foreign track on the other)
it runs into them and the stub is dropped. `--escape-method underpad` instead
drops a **through-via just past each pad** and lets signal routing pick the net
up on an inner/back layer — going *straight out* past the lateral congestion
rather than fanning into it. Adjacent vias are **staggered** along the escape
axis so two neighbours one pitch apart still clear (e.g. 0.45 mm vias on 0.5 mm
pitch escape with centres ~0.56 mm apart). Each via **and its stub** are
obstacle-checked against foreign tracks/vias/pads — and "foreign" means *any net
other than the one being escaped, even on the same chip* (a routed neighbour
pair, a crossing track), so the via can't land on a neighbour's copper and short
it (issue #161). A via that can't find a clear offset is dropped and reported.
Pair the via with `--via-drill` to keep a sane annular ring (e.g.
`--via-size 0.45 --via-drill 0.25`).

Placement is greedy per edge, but if the default nearest-first stagger **drops**
a leg the edge is re-tried under alternative stagger configurations — reversed
order, and per-leg direction biases that send one leg back while its neighbour
goes forward — keeping whichever escapes the most legs (issue #161 follow-up).
The default is tried first, so an edge that already escapes fully is unchanged;
the alternatives only kick in to rescue an otherwise-dropped leg.

#### `--allow-via-in-pad` (issue #161)

On a genuinely boxed-in pair (the outer leg has a neighbour pad one pitch away
*and* the neighbour's diagonal escape sweeping through the only outward room) the
via has nowhere to go outward, so the plain escape **drops** it. With
`--allow-via-in-pad` the escape via may sit on its **own** pad (via-in-pad). Its
candidate offsets then become a **mix**: on-pad positions (centre, then inward
toward the chip) *and* off-pad positions (outward past the pad), so each leg
independently takes whichever escapes — a boxed-in leg staggers *inward*, away
from the neighbour, instead of being dropped, while a leg with outward room still
goes out. The via still must clear every other-net pad, via and track — it only
gains permission to overlap its own pad. Name and behaviour match the
"Allow via-in-pad" option elsewhere in the tools. Example:

```bash
python3 qfn_fanout.py board.kicad_pcb --component U1 --nets 'DP1*' \
    --escape-method underpad --allow-via-in-pad \
    --via-size 0.45 --via-drill 0.2 --clearance 0.15 --grid-step 0.05
```

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
   - Straight segment: perpendicular to chip edge (pad_width / 2 + extension, clears pad before bend)
   - 45-degree segment: fans outward, length varies by position (0 at center, max at corners)
5. **Collision Check** - Validate endpoint spacing meets minimum spacing requirements

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

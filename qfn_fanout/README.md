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
| `--escape-method` | `stub` (surface 45° fan) or `underpad` (via-drop); with `--nets`, `underpad` also escapes **interior (under-body) pads** (issue #410) | stub |
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

#### Interior pads — `--escape-method underpad --nets ...` (issue #410)

Some packages carry pads **under the body** that are not on the perimeter:
AQFN inner rings, and connector-style parts whose signal pins sit surrounded by
ground pads. The surface 45° fan can never escape these (there is no free edge
to route from), so the fanout classifies them `center` and skips them. A
via-drop needs no free surface edge, though — the via goes straight down — so
on a run that is **both** `--escape-method underpad` **and** scoped with
`--nets`, interior pads are escaped like any other pad, each along its own
long-axis direction.

Requiring `--nets` is a deliberate safety guard: an unscoped run never
via-drops the exposed/thermal pad. The EP escapes only if your filter
explicitly matches its net (e.g. `--nets 'GND'`), which is a valid, deliberate
choice — not a default. Interior pads are usually boxed in by their neighbours,
so pair this with `--allow-via-in-pad`; the via then sits centred on the pad
and no stub is emitted at all. Example:

```bash
python3 qfn_fanout.py board.kicad_pcb --component U1 --nets 'INT*' \
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
   - Fan segment: angles outward; both its length AND its angle grow with
     position along the edge (0 at the edge midpoint, full length and a 45°
     angle at the outermost/corner pad)
5. **Collision Check** - Validate endpoint spacing meets minimum spacing requirements

## Stub Pattern

```
Corner pad: long, full 45° fan-out
    ╲
     ╲
      ●  (pad)

Mid-edge pad: shorter, shallower angle
     ╲
      ●  (pad)

Edge-midpoint pad: straight only (~0° fan)
      |
      ●  (pad)
```

The fan **length** varies linearly with edge position (0 at the midpoint,
max_diagonal_length = chip_width / 3 at the corner). The fan **angle** ramps
separately from ~0° at the midpoint to a full 45° at the *outermost pad* on each
side (issue #200). A uniform 45° fan keeps every adjacent stub parallel, so
neighbouring tips stay only pitch/√2 apart — too tight on a fine-pitch edge to
fit an escape via between them. Ramping the angle makes adjacent stubs *diverge*
instead, spreading the tips apart while the corner pad keeps its full 45° reach.
Tips are then snapped to the routing grid in a direction-preserving way, so a
near-midpoint stub's bend never reverses toward its neighbour.

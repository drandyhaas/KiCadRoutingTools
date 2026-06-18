# BGA Fanout Generator

Creates escape routing for BGA (Ball Grid Array) packages in KiCad PCB files.

## Features

- **Generic BGA support** - Works with any BGA package pitch and size
- **Any placement angle** - The grid/channel/escape logic is global-axis-based, so a
  BGA placed at a non-orthogonal angle (anything other than 0/90/180/270°) is routed
  by rigidly rotating the whole board into the footprint's own frame, running the
  pipeline there, and mapping the resulting tracks/vias back (issue #137). The transform
  is an isometry, so the result is identical to solving the un-rotated board; orthogonal
  placements skip it entirely and are unaffected. Foreign-pad clearance is exact, including
  for rotated rectangular neighbours (both the routing obstacle map and the explicit
  pad-collision test honour each pad's rotation).
- **Differential pair routing** - P/N pairs routed together on same layer
- **Collision-free routing** - Automatic layer assignment to avoid conflicts
- **Multi-layer support** - Distributes routes evenly across available layers
- **Adjacent same-net optimization** - Connects neighboring pads on same net directly
- **Existing fanout detection** - Skips pads that already have fanouts
- **Via management** - Adds vias only for SMD pads on non-top layers (through-hole pads connect all layers)
- **Under-pad escape for dense arrays** - An alternate engine (`--escape-method underpad`) that escapes fully-populated BGAs the channel router can't (see [Escape methods](#escape-methods))

## Usage

```bash
# Basic fanout
python bga_fanout.py kicad_files/input.kicad_pcb --component U3 --output kicad_files/output.kicad_pcb

# With net filter (include pattern)
python bga_fanout.py kicad_files/input.kicad_pcb --component U3 --output kicad_files/output.kicad_pcb --nets "*DATA*"

# With exclusion pattern (all nets except GND and VCC)
python bga_fanout.py kicad_files/input.kicad_pcb --component U3 --output kicad_files/output.kicad_pcb \
    --nets "*" "!GND" "!VCC"

# Differential pairs
python bga_fanout.py kicad_files/input.kicad_pcb --component IC1 --output kicad_files/output.kicad_pcb \
    --nets "*lvds*" --diff-pairs "*lvds*" --primary-escape vertical

# Specify layers
python bga_fanout.py kicad_files/input.kicad_pcb --component U3 --output kicad_files/output.kicad_pcb \
    --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu
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
| `--component`, `-c` | Component reference | Auto-detect BGA |
| `--output`, `-o` | Output PCB file | fanout_test.kicad_pcb |
| `--layers`, `-l` | Routing layers | F.Cu B.Cu |
| `--track-width`, `-w` | Track width (mm) | 0.3 |
| `--clearance` | Track clearance (mm) | 0.25 |
| `--via-size` | Via outer diameter (mm) | 0.5 |
| `--via-drill` | Via drill size (mm) | 0.3 |
| `--nets`, `-n` | Net patterns to include | All nets |
| `--diff-pairs`, `-d` | Differential pair patterns | None |
| `--diff-pair-gap` | Gap between P/N traces (mm) | 0.1 |
| `--exit-margin` | Distance past BGA boundary (mm) | 0.5 |
| `--primary-escape`, `-p` | Primary escape direction | horizontal |
| `--force-escape-direction` | Only use primary direction | False |
| `--rebalance-escape` | Rebalance escape directions | False |
| `--check-for-previous` | Skip existing fanouts | False |
| `--no-inner-top-layer` | Prevent inner pads from using F.Cu | False |
| `--escape-method` | `channel` (default) or `underpad` (dense arrays) | channel |

## Escape methods

`--escape-method` selects the fanout engine:

- **`channel`** (default) - the 45°-stub + channel router. Each ball jogs to a
  routing channel between the ball rows and runs out to the boundary. Full
  differential-pair support. Best for sparse/medium BGAs and signal-grouped escapes.

- **`underpad`** - a grid escape for **dense, fully-populated arrays** (issue
  #122). The channel router confines every layer to the gaps *between* ball
  rows, so on a fully-populated array a few channels over-subscribe and the
  deepest balls can't escape (e.g. ulx3s 22×22 dropped ~23 balls). The under-pad
  method instead drops a **via in each signal ball's pad** and routes it
  *straight under the pad field* on an inner layer (SMD pads only block their own
  layer), jogging into a between-ball channel only to dodge a via. Near-edge
  balls escape via-less on the BGA's own layer, keeping the rim clear so the
  deeper balls can run underneath. Balls are routed deepest-first (inside-out).

  Notes / when to use it:
  - Use it when `channel` reports dropped balls (`failed > 0`) on a dense array.
  - **Power/plane nets are skipped** - they tap their plane through a via, not a
    lateral escape. (Plane the power nets first, or exclude them with `--nets`.)
  - Diff pairs are routed as **single-ended** (the pair geometry isn't preserved).
  - Use a **small via and track** for the pitch - the escape needs one clean
    track between adjacent via-in-pads, roughly `via ≤ pitch − 2·track − 2·clearance`.
    For 0.8 mm pitch, `--via-size 0.35 --track-width 0.12 --clearance 0.1` works well.
  - Works for top- **and** bottom-side (B.Cu) BGAs - the via-less edge escape
    runs on the BGA's own layer.

```bash
# Dense BGA that the channel router can't fully escape
python bga_fanout.py board.kicad_pcb -c U1 -o out.kicad_pcb \
    --layers F.Cu In1.Cu In2.Cu B.Cu \
    --escape-method underpad --via-size 0.35 --track-width 0.12 --clearance 0.1
```

## Module Structure

```
bga_fanout/
├── __init__.py          # Main fanout logic and public API
├── types.py             # Data types: Track, BGAGrid, FanoutRoute, etc.
├── escape.py            # Escape channel finding and assignment
├── reroute.py           # Collision resolution and rerouting
├── layer_balance.py     # Layer rebalancing for even distribution
├── layer_assignment.py  # Layer assignment for collision avoidance
├── underpad.py          # Under-pad grid escape engine (dense arrays, #122)
├── tracks.py            # Track generation and collision detection
├── geometry.py          # 45° stub and jog calculations
├── collision.py         # Low-level collision detection utilities
├── grid.py              # BGA grid analysis
├── diff_pair.py         # Differential pair detection
└── constants.py         # Configuration constants
```

### Key Components

- **`generate_bga_fanout()`** - Main function that generates fanout tracks
- **`generate_tracks_from_routes()`** - Converts FanoutRoute objects to track dictionaries
- **`find_differential_pairs()`** - Finds P/N pairs in a footprint
- **`assign_layers_smart()`** - Assigns routes to layers avoiding collisions
- **`rebalance_layers()`** - Rebalances routes across layers for even distribution
- **`detect_collisions()`** - Detects track spacing violations
- **`resolve_collisions()`** - Attempts to fix collisions via rerouting
- **`BGAGrid`** - Analyzes BGA pad grid geometry
- **`FanoutRoute`** - Represents a single pad's fanout route
- **`Channel`** - Routing channel between pad rows/columns

## Algorithm

1. **Grid Analysis** - Detect BGA pitch, grid size, and boundary
2. **Channel Calculation** - Identify routing channels between pad rows/columns
3. **Escape Direction Assignment** - Determine which edge each pad exits toward
4. **Adjacent Pad Connection** - Connect neighboring pads on same net directly
5. **Layer Assignment** - Distribute routes across layers to avoid collisions
6. **Track Generation** - Create 45° stubs, channel segments, and exit jogs
7. **Collision Resolution** - Reassign layers or reroute to resolve conflicts
8. **Layer Rebalancing** - Even out route distribution across layers
9. **Via Management** - Add vias for SMD pads on non-top layers

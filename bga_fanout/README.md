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

### Sizing the escape via to the pitch (issue #158)

The channel engine runs one escape track down the **half-pitch** between adjacent
via columns, so the via, track, and clearance must fit it or **every** escape
grazes the neighbouring column's via by a few µm (a sub-clearance DRC violation
the fanout would otherwise report as a clean `failed: 0`). The budget, per array:

```
via_size  ≤  pitch − track_width − 2·clearance        (one escape track per channel)
via_size  ≥  via_drill + 2·min_annular_ring           (fab floor)
```

Example: at 0.8 mm pitch with `--track-width 0.127 --clearance 0.1`, the escape
via must be ≤ 0.473 mm — **Ø0.5 grazes (every escape ~13 µm short), Ø0.45 is
clean** (pair with `--via-drill 0.25` to keep a 0.1 mm annular ring). When handed
infeasible params, `bga_fanout` prints a `WARNING: escape via ... busts the
half-pitch budget` with the recommended maximum. The `plan-pcb-routing` skill
computes this automatically.

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
  - **Differential pairs are escaped coupled** when `--diff-pairs` is given
    (issue #182): each pair's P and N exit on the **same layer**, converged to
    the diff spacing and extended past the boundary, so `route_diff` picks them
    up. Edge pairs escape **via-free on the top layer** (broadside pairs straight
    off the boundary; stacked pairs *END-ON*, the trail ball bulging through the
    pad gap), which keeps the rim via-free so the **deeper stacked pairs escape
    END-ON on an inner layer** (via-in-pad, same layer for P/N) running underneath
    them. A pair that can't fit a coupled corridor falls back to single-ended
    (still connected, just not coupled). On `glasgow_revC` (BGA U30, 0.8 mm
    pitch) all 13 FPGA Z-pairs escape coupled and route_diff couples them.
  - Use a **small via and track** for the pitch - the escape needs one clean
    track between adjacent via-in-pads, roughly `via ≤ pitch − 2·track − 2·clearance`.
    For 0.8 mm pitch, `--via-size 0.35 --track-width 0.12 --clearance 0.1` works well.
  - Works for top- **and** bottom-side (B.Cu) BGAs - the via-less edge escape
    runs on the BGA's own layer.
  - **Movable passives' pads are soft keep-outs** (#278): escapes prefer to
    detour around foreign C/R/FB pads in the region (a wide cap between two
    escape legs can't be moved anywhere by the cap-placement step) and only
    graze one when boxed in - `place_fanout_clearance.py` then moves the cap.
    Keep-out disks are stamped against the pad/via's true coordinate, so a
    clear grid cell genuinely guarantees the clearance (no more few-µm
    VIA-SEGMENT quantization grazes).

```bash
# Dense BGA that the channel router can't fully escape (diff pairs escaped coupled)
python bga_fanout.py board.kicad_pcb -c U1 -o out.kicad_pcb \
    --layers F.Cu In1.Cu In2.Cu B.Cu --diff-pairs "*" \
    --escape-method underpad --via-size 0.35 --track-width 0.12 --clearance 0.1
```

## After fanout: optimize decoupling-cap placement (issue #130)

A fanout drops vias near the ball field. Where a foreign-net via lands under a
decoupling cap placed at a ball, the via copper overlaps the cap pad — a real
`PAD-VIA` DRC violation at the clearance floor. The fix is *placement*, so run
[`place_fanout_clearance.py`](../placement/README.md) on the **fanned** board
to nudge those caps clear and pull each pad toward its nearest same-net ball
(so a power/GND via dropped there later shares the via):

```bash
python place_fanout_clearance.py out.kicad_pcb capclean.kicad_pcb --clearance 0.1
```

Use the same `--clearance` as the fanout. It only moves 2-pad caps near a BGA,
never overlaps caps, and is a no-op when nothing collides — so run it after
each fanout, before signal routing. (GUI: the "Optimize decoupling cap
placement" checkbox on the BGA fanout tab does this automatically.)

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

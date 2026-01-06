# BGA Fanout Generator

Creates escape routing for BGA (Ball Grid Array) packages in KiCad PCB files.

## Features

- **Generic BGA support** - Works with any BGA package pitch and size
- **Differential pair routing** - P/N pairs routed together on same layer
- **Collision-free routing** - Automatic layer assignment to avoid conflicts
- **Multi-layer support** - Distributes routes evenly across available layers
- **Existing fanout detection** - Skips pads that already have fanouts
- **Via management** - Adds/removes vias as needed for layer changes

## Usage

```bash
# Basic fanout
python bga_fanout.py input.kicad_pcb --component U3 --output output.kicad_pcb

# With net filter
python bga_fanout.py input.kicad_pcb --component U3 --output output.kicad_pcb --nets "*DATA*"

# Differential pairs
python bga_fanout.py input.kicad_pcb --component IC1 --output output.kicad_pcb \
    --nets "*lvds*" --diff-pairs "*lvds*" --primary-escape vertical

# Specify layers
python bga_fanout.py input.kicad_pcb --component U3 --output output.kicad_pcb \
    --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu
```

## Options

| Option | Description | Default |
|--------|-------------|---------|
| `--component`, `-c` | Component reference | Auto-detect BGA |
| `--output`, `-o` | Output PCB file | fanout_test.kicad_pcb |
| `--layers`, `-l` | Routing layers | F.Cu In1.Cu In2.Cu B.Cu |
| `--width`, `-w` | Track width (mm) | 0.1 |
| `--clearance` | Track clearance (mm) | 0.1 |
| `--nets`, `-n` | Net patterns to include | All nets |
| `--diff-pairs`, `-d` | Differential pair patterns | None |
| `--diff-pair-gap` | Gap between P/N traces (mm) | 0.1 |
| `--exit-margin` | Distance past BGA boundary (mm) | 0.5 |
| `--primary-escape`, `-p` | Primary escape direction | horizontal |
| `--force-escape-direction` | Only use primary direction | False |
| `--rebalance-escape` | Rebalance escape directions | False |
| `--check-for-previous` | Skip existing fanouts | False |

## Module Structure

```
bga_fanout/
├── __init__.py          # Main fanout logic and public API (1300 lines)
├── types.py             # Data types: Track, BGAGrid, FanoutRoute, etc. (159 lines)
├── escape.py            # Escape channel finding and assignment (780 lines)
├── reroute.py           # Collision resolution and rerouting (667 lines)
├── layer_balance.py     # Layer rebalancing for even distribution (360 lines)
├── layer_assignment.py  # Layer assignment for collision avoidance (350 lines)
├── tracks.py            # Track generation and collision detection (210 lines)
├── geometry.py          # 45° stub and jog calculations (166 lines)
├── collision.py         # Low-level collision detection utilities (144 lines)
├── grid.py              # BGA grid analysis (105 lines)
├── diff_pair.py         # Differential pair detection (57 lines)
└── constants.py         # Configuration constants (26 lines)
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
4. **Layer Assignment** - Distribute routes across layers to avoid collisions
5. **Track Generation** - Create 45° stubs, channel segments, and exit jogs
6. **Collision Resolution** - Reassign layers or reroute to resolve conflicts
7. **Layer Rebalancing** - Even out route distribution across layers
8. **Via Management** - Add vias for non-top-layer routes

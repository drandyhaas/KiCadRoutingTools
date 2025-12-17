# KiCad PCB Grid Router

A fast Rust-accelerated A* autorouter for KiCad PCB files using integer grid coordinates. Routes nets between existing stub endpoints with multi-layer support and automatic via placement.

## Features

- **Grid-based A* pathfinding** with Rust acceleration (~10x faster than Python)
- **Octilinear routing** - Horizontal, vertical, and 45-degree diagonal moves
- **Multi-layer routing** with automatic via insertion
- **Differential pair routing** with centerline-based P/N pairing and polarity swap handling
- **Batch routing** with incremental obstacle caching (~7x speedup)
- **Net ordering strategies** - MPS (crossing conflicts), inside-out (BGA), or original order
- **BGA exclusion zones** - Auto-detected from footprints, prevents vias under BGAs
- **Stub proximity avoidance** - Penalizes routes near unrouted stubs

## Quick Start

### 1. Build the Rust Router

```bash
python build_router.py
```

### 2. Route Nets

```bash
# Route specific nets
python route.py input.kicad_pcb output.kicad_pcb "Net-(U2A-DATA_0)" "Net-(U2A-DATA_1)"

# Route with wildcard patterns
python route.py input.kicad_pcb output.kicad_pcb "Net-(U2A-DATA_*)"

# Route differential pairs
python route.py input.kicad_pcb output.kicad_pcb "*lvds*" --diff-pairs "*lvds*" --no-bga-zones
```

### 3. Verify Results

```bash
# Check for DRC violations
python check_drc.py output.kicad_pcb --clearance 0.1

# Check connectivity
python check_connected.py output.kicad_pcb --nets "*DATA*"
```

## Documentation

| Document | Description |
|----------|-------------|
| [Routing Architecture](docs/routing-architecture.md) | Module structure, obstacle maps, A* algorithm |
| [Configuration](docs/configuration.md) | Command-line options, GridRouteConfig parameters |
| [Differential Pairs](docs/differential-pairs.md) | P/N pairing, polarity swaps, via handling |
| [Net Ordering](docs/net-ordering.md) | MPS algorithm, inside-out ordering, strategy comparison |
| [Utilities](docs/utilities.md) | DRC checker, connectivity checker, fanout generators |
| [Rust Router](rust_router/README.md) | Building and using the Rust A* module |
| [Visualizer](pygame_visualizer/README.md) | Real-time A* visualization with PyGame |

## Project Structure

```
KiCadRoutingTools/
├── route.py                  # Main CLI - batch routing orchestration
├── routing_config.py         # GridRouteConfig, GridCoord, DiffPair classes
├── routing_utils.py          # Shared utilities (connectivity, MPS, cleanup)
├── obstacle_map.py           # Obstacle map building functions
├── single_ended_routing.py   # Single-ended net routing
├── diff_pair_routing.py      # Differential pair routing
├── kicad_parser.py           # KiCad .kicad_pcb file parser
├── kicad_writer.py           # KiCad S-expression generator
├── check_drc.py              # DRC violation checker
├── check_connected.py        # Connectivity checker
├── bga_fanout.py             # BGA differential pair fanout generator
├── qfn_fanout.py             # QFN/QFP fanout generator
├── list_nets.py              # List nets on a component
├── build_router.py           # Rust module build script
├── test_diffpair.py          # Test single/multiple diff pairs with DRC
├── test_all_diffpairs.py     # Batch test all diff pairs (parallel)
├── rust_router/              # Rust A* implementation
├── pygame_visualizer/        # Real-time visualization
└── docs/                     # Documentation
```

## Module Overview

| Module | Lines | Purpose |
|--------|-------|---------|
| `routing_config.py` | 65 | Configuration dataclasses (`GridRouteConfig`, `GridCoord`, `DiffPair`) |
| `routing_utils.py` | 1136 | Shared utilities: connectivity, endpoint finding, MPS ordering, segment cleanup |
| `obstacle_map.py` | 330 | Obstacle map building from PCB data |
| `single_ended_routing.py` | 343 | Single-ended net routing with A* |
| `diff_pair_routing.py` | 1091 | Differential pair centerline + offset routing |
| `route.py` | 643 | CLI and batch routing orchestration |

## Performance

| Metric | Value |
|--------|-------|
| 56 LVDS diff pairs | 100% with `--fix-polarity` |
| Parallel testing | 14 threads default |
| Speedup vs Python | ~10x |

## Common Options

```bash
python route.py input.kicad_pcb output.kicad_pcb "Net-*" [OPTIONS]

# Geometry
--track-width 0.1       # Track width (mm)
--clearance 0.1         # Track clearance (mm)
--via-size 0.3          # Via diameter (mm)
--via-drill 0.2         # Via drill (mm)

# Algorithm
--grid-step 0.1         # Grid resolution (mm)
--via-cost 25           # Via penalty (grid steps)
--max-iterations 100000 # A* iteration limit
--heuristic-weight 1.5  # A* greediness (>1 = faster)

# Strategy
--ordering mps          # mps | inside_out | original
--layers F.Cu In1.Cu In2.Cu B.Cu
--no-bga-zones          # Allow routing through BGA areas

# Differential pairs
--diff-pairs "*lvds*"   # Pattern for diff pair nets
--diff-pair-gap 0.1     # P-N gap (mm)
```

See [Configuration](docs/configuration.md) for complete option reference.

## Requirements

- Python 3.7+
- Rust toolchain (for building the router module)
- pygame-ce (optional, for visualizer)

## Limitations

- Requires existing stub tracks to identify connection points
- Grid-based (0.1mm default) - may miss very tight fits
- No length matching
- No push-and-shove (routes around obstacles, doesn't move them)
- No rip-up and reroute

## License

MIT License

# KiCad PCB Grid Router

A fast Rust-accelerated A* autorouter for KiCad PCB files using integer grid coordinates. Routes nets between existing stub endpoints with multi-layer support and automatic via placement.

## Features

- **Grid-based A* pathfinding** with Rust acceleration (~10x faster than Python)
- **Octilinear routing** - Horizontal, vertical, and 45-degree diagonal moves
- **Multi-layer routing** with automatic via insertion
- **Differential pair routing** with pose-based A* and Dubins path heuristic for orientation-aware centerline routing
- **Rip-up and reroute** - When routing fails, automatically rips up blocking routes and retries with progressive N+1 strategy (tries 1 blocker, then 2, up to configurable max). Re-analyzes blocking tracks after each failure for better recovery. Also triggers rip-up when quick probes detect blocking early, before attempting full routes.
- **Blocking analysis** - Shows which previously-routed nets are blocking when routes fail
- **Stub layer switching** - Optimization that moves stubs to different layers to avoid vias when source/target are on different layers. Works for both differential pairs and single-ended nets. Finds compatible swap pairs (two nets that can exchange layers to help each other) or moves stubs solo when safe. Tries multiple swap options (source/source, target/target, source/target, target/source) to find valid combinations.
- **Batch routing** with incremental obstacle caching (~7x speedup)
- **Net ordering strategies** - MPS (crossing conflicts with diff pairs treated as units, shorter routes first using BGA-aware distance), inside-out (BGA), or original order
- **BGA exclusion zones** - Auto-detected from footprints, prevents vias under BGAs
- **Stub proximity avoidance** - Penalizes routes near unrouted stubs
- **Track proximity avoidance** - Penalizes routes near previously routed tracks on the same layer, encouraging spread-out routing
- **Adaptive setback angles** - Evaluates 9 setback angles (0°, ±max/4, ±max/2, ±3max/4, ±max) and selects the one that maximizes separation from neighboring stub endpoints, improving routing success when stubs are tightly spaced. Uses 0° when clearance to the nearest stub is sufficient (≥2× spacing), only angling away when stubs are too close
- **Loop detection** - Prevents differential pair routes from forming loops (>270° turns)
- **Target swap optimization** - For swappable nets (e.g., memory lanes), uses Hungarian algorithm to find optimal source-to-target assignments that minimize crossings. Works for both differential pairs and single-ended nets
- **Chip boundary crossing detection** - Uses chip boundary "unrolling" to accurately detect route crossings for MPS ordering and target swap optimization

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
| [Utilities](docs/utilities.md) | DRC checker, connectivity checker, fanout generators, layer switcher |
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
├── stub_layer_switching.py   # Stub layer swap optimization
├── check_drc.py              # DRC violation checker
├── check_connected.py        # Connectivity checker
├── bga_fanout.py             # BGA differential pair fanout generator
├── qfn_fanout.py             # QFN/QFP fanout generator
├── switch_to_layer.py        # Move net segments to a different layer
├── list_nets.py              # List nets on a component
├── build_router.py           # Rust module build script
├── test_diffpair.py          # Test single/multiple diff pairs with DRC
├── test_all_diffpairs.py     # Batch test all diff pairs (parallel, extensive options)
├── rust_router/              # Rust A* implementation
├── pygame_visualizer/        # Real-time visualization
└── docs/                     # Documentation
```

## Module Overview

| Module | Lines | Purpose |
|--------|-------|---------|
| `routing_config.py` | 87 | Configuration dataclasses (`GridRouteConfig`, `GridCoord`, `DiffPair`) |
| `routing_utils.py` | 1944 | Shared utilities: connectivity, endpoint finding, MPS ordering, segment cleanup |
| `obstacle_map.py` | 1015 | Obstacle map building from PCB data |
| `single_ended_routing.py` | 589 | Single-ended net routing with A* |
| `diff_pair_routing.py` | 1777 | Differential pair centerline + offset routing |
| `stub_layer_switching.py` | 682 | Stub layer swap optimization for diff pairs and single-ended nets |
| `route.py` | 3456 | CLI and batch routing orchestration |

## Performance

| Metric | Value |
|--------|-------|
| 56 LVDS diff pairs | 100% (polarity fix enabled by default) |
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
--via-cost 50           # Via penalty (grid steps, doubled for diff pairs)
--max-iterations 200000      # A* iteration limit
--max-probe-iterations 5000  # Quick probe per direction to detect stuck routes
--heuristic-weight 1.9       # A* greediness (>1 = faster)
--max-ripup 3                # Max blockers to rip up at once (1-N progressive)
--max-setback-angle 45       # Max angle for setback search (degrees)
--routing-clearance-margin 1.0   # Multiplier on track-via clearance (1.0 = min DRC, default)

# Strategy
--ordering mps          # mps | inside_out | original
--layers F.Cu In1.Cu In2.Cu B.Cu
--no-bga-zones          # Allow routing through BGA areas

# Proximity penalties
--stub-proximity-radius 2.0  # Radius around stubs to penalize (mm)
--stub-proximity-cost 0.2    # Cost penalty near stubs (mm equivalent)
--bga-proximity-radius 10.0  # Radius around BGA edges to penalize (mm)
--bga-proximity-cost 0.2     # Cost penalty near BGA edges (mm equivalent)
--track-proximity-distance 2.0  # Radius around routed tracks to penalize (mm, same layer)
--track-proximity-cost 0.2   # Cost penalty near routed tracks (mm equivalent)

# Differential pairs
--diff-pairs "*lvds*"   # Pattern for diff pair nets
--diff-pair-gap 0.101   # P-N gap (mm)
--diff-pair-centerline-setback  # Setback distance (default: 2x P-N spacing)
--min-turning-radius 0.2      # Min turn radius (mm)
--direction backward    # Route from target to source

# Layer optimization (stub layer swap enabled by default for both diff pairs and single-ended)
--no-stub-layer-swap    # Disable stub layer switching
--can-swap-to-top-layer # Allow swapping stubs to F.Cu (off by default for diff pairs)

# Target swap optimization (works for both diff pairs and single-ended nets)
--swappable-nets "*rx*"  # Glob patterns for nets that can have targets swapped
--crossing-penalty 1000  # Penalty for crossing assignments (default: 1000)
--no-crossing-layer-check  # Count crossings regardless of layer (default: same-layer only)
--mps-reverse-rounds     # Route most-conflicting MPS groups first (instead of least)

# Debug
--verbose               # Print detailed diagnostic output
--debug-lines           # Output debug geometry on User layers
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

## License

MIT License

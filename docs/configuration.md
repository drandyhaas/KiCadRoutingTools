# Configuration Reference

This document describes all configuration options for the KiCad Grid Router.

## Command-Line Options

### Basic Usage

```bash
python route.py input.kicad_pcb output.kicad_pcb "Net-*" [OPTIONS]
```

### Net Selection

Net names support glob wildcards:

```bash
# Exact net names
python route.py in.kicad_pcb out.kicad_pcb "Net-(U2A-DATA_0)" "Net-(U2A-DATA_1)"

# Wildcard patterns
python route.py in.kicad_pcb out.kicad_pcb "Net-(U2A-DATA_*)"

# Multiple patterns
python route.py in.kicad_pcb out.kicad_pcb "Net-(*CLK*)" "Net-(*DATA*)"
```

### Geometry Options

| Option | Default | Description |
|--------|---------|-------------|
| `--track-width` | 0.1 | Track width in mm |
| `--clearance` | 0.1 | Track-to-track clearance in mm |
| `--via-size` | 0.3 | Via outer diameter in mm |
| `--via-drill` | 0.2 | Via drill diameter in mm |
| `--grid-step` | 0.1 | Grid resolution in mm |

### Algorithm Options

| Option | Default | Description |
|--------|---------|-------------|
| `--via-cost` | 50 | Via penalty in grid steps (doubled for diff pairs) |
| `--max-iterations` | 200000 | A* iteration limit per route |
| `--heuristic-weight` | 1.9 | A* greediness (>1 = faster, <1 = more optimal) |
| `--max-ripup` | 3 | Max blockers to rip up at once during rip-up and retry |
| `--max-setback-angle` | 45.0 | Maximum angle for setback position search (degrees) |

### Routing Strategy Options

| Option | Default | Description |
|--------|---------|-------------|
| `--ordering` / `-o` | mps | Net ordering: `mps`, `inside_out`, or `original` |
| `--direction` / `-d` | forward | Direction: `forward`, `backward`, or `random` |
| `--layers` / `-l` | F.Cu In1.Cu In2.Cu B.Cu | Routing layers |
| `--no-bga-zones` | false | Disable BGA exclusion zone detection |

### Proximity Penalty Options

| Option | Default | Description |
|--------|---------|-------------|
| `--stub-proximity-radius` | 2.0 | Radius around stubs to penalize (mm) |
| `--stub-proximity-cost` | 0.2 | Cost penalty at stub center (mm equivalent) |
| `--bga-proximity-radius` | 10.0 | Radius around BGA edges to penalize (mm) |
| `--bga-proximity-cost` | 0.2 | Cost penalty at BGA edge (mm equivalent) |
| `--track-proximity-distance` | 2.0 | Radius around routed tracks to penalize on same layer (mm) |
| `--track-proximity-cost` | 0.2 | Cost penalty near routed tracks (mm equivalent) |

### Differential Pair Options

| Option | Default | Description |
|--------|---------|-------------|
| `--diff-pairs` / `-D` | - | Glob patterns for diff pair nets |
| `--diff-pair-gap` | 0.101 | Gap between P and N traces (mm) |
| `--diff-pair-centerline-setback` | 2x P-N dist | Distance in front of stubs to start centerline (mm) |
| `--min-turning-radius` | 0.2 | Minimum turning radius for pose-based routing (mm) |
| `--max-setback-angle` | 45.0 | Maximum angle for setback position search (degrees) |
| `--no-fix-polarity` | false | Don't swap target pad nets when polarity swap needed |
| `--swappable-nets` | - | Glob patterns for diff pair nets that can have targets swapped |
| `--crossing-penalty` | 1000.0 | Penalty for crossing assignments in target swap optimization |
| `--mps-reverse-rounds` | false | Route most-conflicting MPS groups first (instead of least) |

### Layer Optimization Options

These options control stub layer switching, which moves stubs to different layers before routing to avoid vias. Works for both differential pairs and single-ended nets.

| Option | Default | Description |
|--------|---------|-------------|
| `--no-stub-layer-swap` | false | Disable stub layer switching (enabled by default) |
| `--can-swap-to-top-layer` | false | Allow swapping stubs to F.Cu (off by default for diff pairs due to clearance) |

**How it works:**
- When source and target stubs are on different layers, a via is normally required
- Stub layer switching moves one stub to match the other's layer, eliminating the via
- For **swap pairs**, two nets exchange layers to help each other (e.g., Net1 src:A→B and Net2 src:B→A)
- For **solo switches**, a single stub moves when it doesn't conflict with other stubs
- Multiple swap options are tried: src/src, tgt/tgt, src/tgt, tgt/src

### Debug Options

| Option | Default | Description |
|--------|---------|-------------|
| `--debug-lines` | false | Output debug geometry on User.3/4/8/9 layers |
| `--verbose` / `-v` | false | Print detailed diagnostic output (setback checks, etc.) |

## GridRouteConfig Class

The `GridRouteConfig` dataclass holds all routing parameters:

```python
@dataclass
class GridRouteConfig:
    # Track geometry
    track_width: float = 0.1      # mm
    clearance: float = 0.1        # mm between tracks
    via_size: float = 0.3         # mm via outer diameter
    via_drill: float = 0.2        # mm via drill

    # Grid
    grid_step: float = 0.1        # mm grid resolution

    # A* algorithm
    via_cost: int = 50            # grid steps penalty for via (doubled for diff pairs)
    max_iterations: int = 200000
    max_probe_iterations: int = 5000  # quick probe per direction to detect stuck routes
    heuristic_weight: float = 1.9
    max_rip_up_count: int = 3     # max blockers to rip up at once (progressive N+1)
    max_setback_angle: float = 45.0  # degrees

    # Layers
    layers: List[str] = ['F.Cu', 'B.Cu']

    # BGA zones
    bga_exclusion_zones: List[Tuple[float, float, float, float]] = []

    # Stub proximity
    stub_proximity_radius: float = 2.0   # mm
    stub_proximity_cost: float = 0.2     # mm equivalent
    via_proximity_cost: float = 50.0     # multiplier for vias near stubs (uses via_cost value)

    # Track proximity (same layer)
    track_proximity_distance: float = 2.0  # mm
    track_proximity_cost: float = 0.2      # mm equivalent

    # Direction
    direction_order: str = "forward"     # forward, backward, or random

    # Differential pairs
    diff_pair_gap: float = 0.101         # mm between P and N
    diff_pair_centerline_setback: float = None  # mm in front of stubs (None = 2x P-N spacing)
    fix_polarity: bool = True            # swap target pads if polarity swap needed
    stub_layer_swap: bool = True         # enable stub layer switching optimization
    target_swap_crossing_penalty: float = 1000.0  # penalty for crossing assignments

    # Debug
    debug_lines: bool = False
```

## Parameter Guidelines

### Via Cost

The `via_cost` parameter controls how much the router penalizes layer changes:

| Value | Effect |
|-------|--------|
| 0-25 | Many vias, shorter paths |
| 50 (default) | Balanced, discourages unnecessary vias |
| 75-100 | Few vias, longer paths |

For BGA escape routing, lower values (10-25) work well since vias are necessary.

### Heuristic Weight

Controls A* behavior:

| Value | Effect |
|-------|--------|
| 1.0 | Optimal paths (slower) |
| 1.9 (default) | Good balance of speed and quality |
| 2.5+ | Faster but may miss tight routes |

### Grid Step

Smaller grid steps allow finer routing but increase computation:

| Value | Use Case |
|-------|----------|
| 0.05 | Fine-pitch BGAs, tight clearances |
| 0.1 (default) | General purpose |
| 0.2 | Fast routing, less detail |

### Stub Proximity

Penalizes routes that pass near unrouted stubs:

```
Cost = stub_proximity_cost * (1 - distance/stub_proximity_radius)
```

This encourages routes to avoid blocking future routing paths.

## Layer Configuration

### Standard 4-Layer Stack

```bash
--layers F.Cu In1.Cu In2.Cu B.Cu
```

### 2-Layer Board

```bash
--layers F.Cu B.Cu
```

### 6-Layer with Ground Planes

```bash
# Skip In1.Cu and In4.Cu (ground planes)
--layers F.Cu In2.Cu In3.Cu B.Cu
```

## Example Configurations

### BGA Fanout (Dense, Many Vias)

```bash
python route.py input.kicad_pcb output.kicad_pcb "Net-(*)" \
    --ordering inside_out \
    --via-cost 10 \
    --heuristic-weight 1.2 \
    --stub-proximity-radius 2.0 \
    --stub-proximity-cost 5.0
```

### Long Routes (Few Vias)

```bash
python route.py input.kicad_pcb output.kicad_pcb "Net-(*)" \
    --ordering mps \
    --via-cost 50 \
    --heuristic-weight 2.0
```

### Differential Pairs (LVDS)

```bash
python route.py input.kicad_pcb output.kicad_pcb "*lvds*" \
    --diff-pairs "*lvds*" \
    --diff-pair-gap 0.1 \
    --track-width 0.1 \
    --clearance 0.1 \
    --no-bga-zones
```

### Fast Routing (Large Boards)

```bash
python route.py input.kicad_pcb output.kicad_pcb "Net-(*)" \
    --grid-step 0.2 \
    --heuristic-weight 2.0 \
    --max-iterations 50000
```

### Fine-Pitch BGA

```bash
python route.py input.kicad_pcb output.kicad_pcb "Net-(*)" \
    --grid-step 0.05 \
    --track-width 0.075 \
    --clearance 0.075 \
    --via-size 0.2 \
    --via-drill 0.1
```

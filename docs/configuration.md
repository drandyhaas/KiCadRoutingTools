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
| `--via-cost` | 25 | Via penalty in grid steps (doubled for diff pairs) |
| `--max-iterations` | 200000 | A* iteration limit per route |
| `--heuristic-weight` | 1.5 | A* greediness (>1 = faster, <1 = more optimal) |

### Routing Strategy Options

| Option | Default | Description |
|--------|---------|-------------|
| `--ordering` / `-o` | mps | Net ordering: `mps`, `inside_out`, or `original` |
| `--direction` / `-d` | forward | Direction: `forward`, `backwards`, or `random` |
| `--layers` / `-l` | F.Cu In1.Cu In2.Cu B.Cu | Routing layers |
| `--no-bga-zones` | false | Disable BGA exclusion zone detection |

### Stub Proximity Options

| Option | Default | Description |
|--------|---------|-------------|
| `--stub-proximity-radius` | 1.5 | Radius around stubs to penalize (mm) |
| `--stub-proximity-cost` | 2.0 | Cost penalty at stub center (mm equivalent) |

### Differential Pair Options

| Option | Default | Description |
|--------|---------|-------------|
| `--diff-pairs` / `-D` | - | Glob patterns for diff pair nets |
| `--diff-pair-gap` | 0.1 | Gap between P and N traces (mm) |
| `--diff-pair-centerline-setback` | 0.4 | Distance in front of stubs to start centerline (mm) |
| `--fix-polarity` | false | Swap target pad nets when polarity swap needed |

### Debug Options

| Option | Default | Description |
|--------|---------|-------------|
| `--debug-layers` | false | Output debug geometry on In4/In5/User.8/User.9 layers |

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
    via_cost: int = 25            # grid steps penalty for via (doubled for diff pairs)
    max_iterations: int = 200000
    heuristic_weight: float = 1.5

    # Layers
    layers: List[str] = ['F.Cu', 'B.Cu']

    # BGA zones
    bga_exclusion_zones: List[Tuple[float, float, float, float]] = []

    # Stub proximity
    stub_proximity_radius: float = 1.0   # mm
    stub_proximity_cost: float = 3.0     # mm equivalent

    # Direction
    direction_order: str = "forward"     # forward, backwards, or random

    # Differential pairs
    diff_pair_gap: float = 0.1           # mm between P and N
    diff_pair_centerline_setback: float = 0.4  # mm in front of stubs
    fix_polarity: bool = False           # swap target pads if polarity swap needed

    # Debug
    debug_layers: bool = False
```

## Parameter Guidelines

### Via Cost

The `via_cost` parameter controls how much the router penalizes layer changes:

| Value | Effect |
|-------|--------|
| 0-10 | Many vias, shorter paths |
| 25 (default) | Balanced |
| 50-100 | Few vias, longer paths |

For BGA escape routing, lower values (10-15) work well since vias are necessary.

### Heuristic Weight

Controls A* behavior:

| Value | Effect |
|-------|--------|
| 1.0 | Optimal paths (slower) |
| 1.5 (default) | Good balance |
| 2.0+ | Faster but may miss tight routes |

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
    --heuristic-weight 1.5
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

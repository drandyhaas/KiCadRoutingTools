# Configuration Reference

This document describes all configuration options for the KiCad Grid Router.

## Command-Line Options

### Basic Usage

```bash
# Single-ended routing (all nets by default)
python route.py input.kicad_pcb output.kicad_pcb [OPTIONS]
python route.py input.kicad_pcb --overwrite [OPTIONS]  # Overwrite input file

# Differential pair routing (all nets by default)
python route_diff.py input.kicad_pcb output.kicad_pcb [OPTIONS]
python route_diff.py input.kicad_pcb --overwrite [OPTIONS]  # Overwrite input file
```

Use `route.py` for single-ended nets and `route_diff.py` for differential pairs. By default, all nets are routed. Use `--nets` to filter specific patterns.

### Net Selection

Net names support glob wildcards. Use `--nets` (or `-n`) to specify patterns:

```bash
# Exact net names
python route.py in.kicad_pcb out.kicad_pcb --nets "Net-(U2A-DATA_0)" "Net-(U2A-DATA_1)"

# Wildcard patterns
python route.py in.kicad_pcb out.kicad_pcb --nets "Net-(U2A-DATA_*)"

# Multiple patterns
python route.py in.kicad_pcb out.kicad_pcb --nets "Net-(*CLK*)" "Net-(*DATA*)"

# Route all nets on a component (auto-excludes GND/VCC/VDD/unconnected)
python route.py in.kicad_pcb out.kicad_pcb --component U1
```

### Geometry Options

| Option | Default | Description |
|--------|---------|-------------|
| `--track-width` | 0.1 | Track width in mm (ignored if `--impedance` specified) |
| `--impedance` | - | Target single-ended impedance in ohms (calculates width per layer from stackup) |
| `--clearance` | 0.1 | Track-to-track clearance in mm |
| `--via-size` | 0.3 | Via outer diameter in mm |
| `--via-drill` | 0.2 | Via drill diameter in mm |
| `--grid-step` | 0.1 | Grid resolution in mm |

**Impedance-controlled routing:** When `--impedance` is specified, track widths are automatically calculated per layer using IPC-2141 formulas based on the board stackup. Outer layers use microstrip formulas (typically wider tracks) and inner layers use stripline formulas (typically narrower tracks). Via clearance calculations account for the varying track widths per layer.

### Power Net Options

| Option | Default | Description |
|--------|---------|-------------|
| `--power-nets` | - | Glob patterns for power nets (e.g., `"*GND*" "*VCC*"`) |
| `--power-nets-widths` | - | Track widths in mm for each power-net pattern (must match `--power-nets` length) |

```bash
# Route with wider tracks for power nets
python route.py input.kicad_pcb output.kicad_pcb --nets "Net*" \
  --power-nets "*GND*" "*VCC*" "+3.3V" \
  --power-nets-widths 0.4 0.5 0.3 \
  --track-width 0.2  # default for non-power nets
```

Patterns are matched in order - first match determines width. Obstacle clearances automatically adjust for wider power traces.

See [Power Net Analysis](power-nets.md) for automatic detection, AI-powered analysis, and IPC-2152 track width guidelines.

### Algorithm Options

| Option | Default | Description |
|--------|---------|-------------|
| `--via-cost` | 50 | Via penalty in grid steps (doubled for diff pairs) |
| `--max-iterations` | 200000 | A* iteration limit per route |
| `--max-probe-iterations` | 5000 | Quick probe per direction to detect stuck routes |
| `--heuristic-weight` | 1.9 | A* greediness (>1 = faster, <1 = more optimal) |
| `--turn-cost` | 1000 | Penalty for direction changes (encourages straighter paths) |
| `--max-ripup` | 3 | Max blockers to rip up at once during rip-up and retry |
| `--max-setback-angle` | 45.0 | Maximum angle for setback position search (degrees) |
| `--routing-clearance-margin` | 1.0 | Multiplier on track-via clearance (1.0 = minimum DRC) |
| `--hole-to-hole-clearance` | 0.2 | Minimum drill hole edge-to-edge clearance (mm) |
| `--board-edge-clearance` | 0.0 | Clearance from board edge in mm (0 = use track clearance) |

### Routing Strategy Options

| Option | Default | Description |
|--------|---------|-------------|
| `--ordering` / `-o` | mps | Net ordering: `mps`, `inside_out`, or `original` |
| `--direction` / `-d` | forward | Direction: `forward` or `backward` |
| `--layers` / `-l` | F.Cu In1.Cu In2.Cu B.Cu | Routing layers |
| `--no-bga-zones [REFS...]` | (auto-detect) | Disable BGA exclusion zones. No args = all. With refs (U1 U3) = only those |

### Proximity Penalty Options

| Option | Default | Description |
|--------|---------|-------------|
| `--stub-proximity-radius` | 2.0 | Radius around stubs to penalize (mm) |
| `--stub-proximity-cost` | 0.2 | Cost penalty at stub center (mm equivalent) |
| `--via-proximity-cost` | 10.0 | Via cost multiplier in stub proximity zones (0 = block vias) |
| `--bga-proximity-radius` | 7.0 | Radius around BGA edges to penalize (mm) |
| `--bga-proximity-cost` | 0.2 | Cost penalty at BGA edge (mm equivalent) |
| `--track-proximity-distance` | 2.0 | Radius around routed tracks to penalize on same layer (mm) |
| `--track-proximity-cost` | 0.2 | Cost penalty near routed tracks (mm equivalent) |
| `--vertical-attraction-radius` | 1.0 | Radius for cross-layer track attraction (mm) |
| `--vertical-attraction-cost` | 0.1 | Cost bonus for aligning with tracks on other layers (mm) |

### Differential Pair Options (route_diff.py only)

These options are only available in `route_diff.py`. All nets passed to route_diff.py are treated as differential pairs.

| Option | Default | Description |
|--------|---------|-------------|
| `--impedance` | - | Target differential impedance in ohms (calculates width per layer from stackup) |
| `--diff-pair-gap` | 0.101 | Gap between P and N traces (mm), also used as spacing for impedance calculation |
| `--diff-pair-centerline-setback` | 2x P-N dist | Distance in front of stubs to start centerline (mm) |
| `--min-turning-radius` | 0.2 | Minimum turning radius for pose-based routing (mm) |
| `--max-turn-angle` | 180 | Max cumulative turn angle (degrees) to prevent U-turns |
| `--max-setback-angle` | 45.0 | Maximum angle for setback position search (degrees) |
| `--no-fix-polarity` | false | Don't swap target pad nets when polarity swap needed |
| `--no-gnd-vias` | false | Disable GND via placement near signal vias |
| `--diff-chamfer-extra` | 1.5 | Chamfer multiplier for diff pair meanders (>1 avoids P/N crossings) |
| `--diff-pair-intra-match` | false | Match P/N lengths within each diff pair |
| `--swappable-nets` | - | Glob patterns for diff pair nets that can have targets swapped |
| `--crossing-penalty` | 1000.0 | Penalty for crossing assignments in target swap optimization |
| `--mps-reverse-rounds` | false | Route most-conflicting MPS groups first (instead of least) |
| `--mps-segment-intersection` | auto | Force segment intersection method for MPS (auto-enabled when no nets on BGAs) |

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

### Length Matching Options

Length matching adds trombone-style meanders to match route lengths within groups. Useful for DDR4 DQ/DQS signals.

| Option | Default | Description |
|--------|---------|-------------|
| `--length-match-group` | - | Group of nets to length match. Use `auto` for DDR4 auto-grouping, or specify patterns |
| `--length-match-tolerance` | 0.1 | Acceptable length variance within group (mm) |
| `--meander-amplitude` | 1.0 | Height of meanders perpendicular to trace (mm) |
| `--diff-chamfer-extra` | 1.5 | Chamfer multiplier for diff pair meanders (>1 avoids P/N crossings) |
| `--diff-pair-intra-match` | false | Match P/N lengths within each diff pair (meander shorter track) |

**Auto-grouping for DDR4:**
```bash
--length-match-group auto
```
Groups nets by byte lane: DQ0-7 + DQS0, DQ8-15 + DQS1, etc.

**Manual grouping:**
```bash
--length-match-group "Net-(*DQ[0-7]*)" "Net-(*DQS0*)"
--length-match-group "Net-(*DQ[8-9]*)" "Net-(*DQ1[0-5]*)" "Net-(*DQS1*)"
```

**How it works:**
- Routes all nets first, then adds meanders to shorter routes
- Calculates pad-to-pad length including existing stub segments and via barrel lengths (parsed from board stackup)
- Stub via barrel lengths use actual stub-layer-to-pad-layer distance (not full via span for through-hole BGA pad vias)
- Via barrel length matches KiCad's length calculation for accurate matching
- Finds longest straight segment for meander insertion
- Iteratively adds meander bumps until length exceeds target, then scales down amplitude to hit exact target
- Per-bump clearance checking reduces amplitude to avoid conflicts with other traces
- Uses 45° chamfered corners for smooth trombone patterns
- Supports multi-layer routes: meanders are applied to same-layer sections, preserving via positions

### Visualization Options

| Option | Default | Description |
|--------|---------|-------------|
| `--visualize` / `-V` | false | Show real-time visualization of routing (requires pygame) |
| `--auto` | false | Auto-advance to next net without waiting (with `--visualize`) |
| `--display-time` | 0.0 | Seconds to display completed route before advancing (with `--visualize --auto`) |

### Debug Options

| Option | Default | Description |
|--------|---------|-------------|
| `--debug-lines` | false | Output debug geometry on User.3/4/8/9 layers |
| `--verbose` / `-v` | false | Print detailed diagnostic output (setback checks, etc.) |
| `--skip-routing` | false | Skip actual routing, only do swaps and write debug info |
| `--debug-memory` | false | Print memory usage statistics at key points during routing |

## GridRouteConfig Class

The `GridRouteConfig` dataclass holds all routing parameters:

```python
@dataclass
class GridRouteConfig:
    # Track geometry
    track_width: float = 0.1      # mm (default for non-power nets)
    clearance: float = 0.1        # mm between tracks
    via_size: float = 0.3         # mm via outer diameter
    via_drill: float = 0.2        # mm via drill
    power_net_widths: Dict[int, float] = {}  # net_id -> width for power nets

    # Grid
    grid_step: float = 0.1        # mm grid resolution

    # A* algorithm
    via_cost: int = 50            # grid steps penalty for via (doubled for diff pairs)
    max_iterations: int = 200000
    max_probe_iterations: int = 5000  # quick probe per direction to detect stuck routes
    heuristic_weight: float = 1.9
    turn_cost: int = 1000         # penalty for direction changes (straighter paths)
    max_rip_up_count: int = 3     # max blockers to rip up at once (progressive N+1)
    max_setback_angle: float = 45.0  # degrees
    routing_clearance_margin: float = 1.0  # multiplier on track-via clearance (1.0 = min DRC)
    hole_to_hole_clearance: float = 0.2  # mm - minimum drill hole edge-to-edge clearance
    board_edge_clearance: float = 0.0    # mm - clearance from board edge (0 = use clearance)

    # Layers
    layers: List[str] = ['F.Cu', 'B.Cu']

    # BGA zones
    bga_exclusion_zones: List[Tuple[float, float, float, float]] = []

    # Stub proximity
    stub_proximity_radius: float = 2.0   # mm
    stub_proximity_cost: float = 0.2     # mm equivalent
    via_proximity_cost: float = 10.0     # multiplier for vias near stubs

    # BGA proximity
    bga_proximity_radius: float = 7.0   # mm from BGA edges
    bga_proximity_cost: float = 0.2      # mm equivalent

    # Track proximity (same layer)
    track_proximity_distance: float = 2.0  # mm
    track_proximity_cost: float = 0.2      # mm equivalent

    # Vertical track alignment (cross-layer attraction)
    vertical_attraction_radius: float = 1.0  # mm
    vertical_attraction_cost: float = 0.1    # mm equivalent bonus

    # Direction
    direction_order: str = "forward"     # forward or backward

    # Differential pairs
    diff_pair_gap: float = 0.101         # mm between P and N
    diff_pair_centerline_setback: float = None  # mm in front of stubs (None = 2x P-N spacing)
    min_turning_radius: float = 0.2      # mm for pose-based routing
    max_turn_angle: float = 180.0        # degrees, prevents U-turns
    fix_polarity: bool = True            # swap target pads if polarity swap needed
    stub_layer_swap: bool = True         # enable stub layer switching optimization
    gnd_via_enabled: bool = True         # place GND vias near signal vias
    target_swap_crossing_penalty: float = 1000.0  # penalty for crossing assignments
    crossing_layer_check: bool = True    # only count crossings on same layer

    # Length matching
    length_match_groups: List[List[str]] = []  # groups of net patterns to match
    length_match_tolerance: float = 0.1        # mm - acceptable length variance
    meander_amplitude: float = 1.0             # mm - height of meander perpendicular to trace
    diff_chamfer_extra: float = 1.5            # chamfer multiplier for diff pair meanders
    diff_pair_intra_match: bool = False        # match P/N lengths within each diff pair

    # Debug
    debug_lines: bool = False
    verbose: bool = False
    debug_memory: bool = False
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
python route.py input.kicad_pcb output.kicad_pcb --nets "Net-(*)" \
    --ordering inside_out \
    --via-cost 10 \
    --heuristic-weight 1.2 \
    --stub-proximity-radius 2.0 \
    --stub-proximity-cost 5.0
```

### Long Routes (Few Vias)

```bash
python route.py input.kicad_pcb output.kicad_pcb --nets "Net-(*)" \
    --ordering mps \
    --via-cost 50 \
    --heuristic-weight 2.0
```

### Differential Pairs (LVDS)

```bash
python route_diff.py input.kicad_pcb output.kicad_pcb --nets "*lvds*" \
    --diff-pair-gap 0.1 \
    --track-width 0.1 \
    --clearance 0.1 \
    --no-bga-zones
```

### Fast Routing (Large Boards)

```bash
python route.py input.kicad_pcb output.kicad_pcb --nets "Net-(*)" \
    --grid-step 0.2 \
    --heuristic-weight 2.0 \
    --max-iterations 50000
```

### Fine-Pitch BGA

```bash
python route.py input.kicad_pcb output.kicad_pcb --nets "Net-(*)" \
    --grid-step 0.05 \
    --track-width 0.075 \
    --clearance 0.075 \
    --via-size 0.2 \
    --via-drill 0.1
```

### Power and Signal Mixed (Wider Power Tracks)

```bash
python route.py input.kicad_pcb output.kicad_pcb --nets "*" \
    --power-nets "*GND*" "*VCC*" "*VDD*" "+*V" \
    --power-nets-widths 0.5 0.4 0.4 0.3 \
    --track-width 0.2 \
    --clearance 0.2
```

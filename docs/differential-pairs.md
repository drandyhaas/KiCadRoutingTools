# Differential Pair Routing

This document describes how the router handles differential pairs, including P/N pairing, centerline routing, and via placement.

## Overview

Differential pairs are routed using a **centerline + offset** approach with **pose-based A* routing**:

1. **Setback position finding** - Find clear positions in front of stubs at fixed setback distance, scanning 9 angles (0°, ±max/4, ±max/2, ±3max/4, ±max)
2. **Pose-based centerline routing** - Route using orientation-aware A* with Dubins path heuristic
3. **Path simplification** - Remove collinear points and in-place turns for cleaner geometry
4. **P/N offset generation** - Create P and N paths as perpendicular offsets from centerline
5. **Via handling** - Add approach/exit tracks at layer changes to keep P/N parallel

The pose-based approach treats routing as searching through (x, y, θ) space, ensuring routes properly respect entry/exit angles at pads.

## Identifying Differential Pairs

The router recognizes common differential pair naming conventions:

| Convention | Example P | Example N |
|------------|-----------|-----------|
| `_P` / `_N` suffix | `LVDS_CLK_P` | `LVDS_CLK_N` |
| `_t` / `_c` suffix | `DQS0_t_A` | `DQS0_c_A` |
| `P` / `N` suffix | `DATA0P` | `DATA0N` |
| `+` / `-` suffix | `CLK+` | `CLK-` |

## Usage

Use `route_diff.py` for differential pair routing. All nets specified are treated as differential pairs:

```bash
# Route a specific diff pair
python route_diff.py input.kicad_pcb output.kicad_pcb --nets "*lvds_rx1_11*" \
    --stub-proximity-radius 4

# Route with debug visualization
python route_diff.py input.kicad_pcb output.kicad_pcb --nets "*lvds_rx1_11*" \
    --stub-proximity-radius 4 --debug-lines

# Route all LVDS nets with custom gap
python route_diff.py input.kicad_pcb output.kicad_pcb --nets "*lvds*" \
    --diff-pair-gap 0.1 \
    --diff-pair-centerline-setback 1.5
```

Nets with _P/_N, P/N, or +/- suffixes will be paired automatically.

## Centerline Routing

### Endpoint Detection

The router finds paired P/N endpoints by:

1. Getting free ends of P and N stub groups
2. Finding closest P-N pairs on the same layer
3. Designating one pair as source, other as target

### Setback from Stubs

The centerline starts a configurable distance ("setback") in front of the stubs, in the direction the stubs are pointing:

```
Stub tips:  P---o       o---P
                 \     /
Centerline:       o---o
                  ^
                  Setback distance (diff-pair-centerline-setback)
```

The centerline endpoints are positioned:
- At the midpoint between P and N stub tips
- Offset by the setback distance in the stub direction

### Pose-Based Centerline Routing

The centerline is routed using orientation-aware A* search with state space (x, y, θ, layer):

```
Source stub                                              Target stub
    |                                                        |
    o----> setback ======== Dubins path ========= setback <---o
           position                               position
```

**State Space**: Each node is defined by position AND heading (θ discretized to 8 directions at 45° intervals).

**Dubins Heuristic**: Instead of Euclidean distance, the heuristic uses Dubins path length - the shortest curve connecting two poses with prescribed headings and minimum turning radius.

**Transitions**:
- Move forward in current direction (cost = distance)
- Turn in place by ±45° (cost based on arc length at min turning radius)
- Layer change via (keeps position and heading)

This produces routes that:
- Start in the stub direction at the source
- End approaching from the correct direction at the target
- Smoothly curve between orientations respecting the minimum turning radius

### Setback Position Finding

The router uses a fixed setback distance (default: 2× P-N spacing) and scans 9 angles to find an unblocked position:

```python
# With max_setback_angle = 45° (default):
angles_deg = [0, 11.25, -11.25, 22.5, -22.5, 33.75, -33.75, 45, -45]
```

For each angle:
1. Check if the position is blocked in the obstacle map
2. Check if the connector path from stub center is clear

This allows finding unblocked positions even when the straight path is blocked by nearby stubs. With 9 angles (vs 5 previously), there's finer granularity for finding optimal positions.

### Extra Clearance

The centerline is routed with extra clearance to accommodate both P and N tracks:

```python
# Extra clearance = offset from centerline to P/N track outer edge
extra_clearance = (track_width + diff_pair_gap) / 2 + track_width / 2
```

This accounts for the P/N track offset (half the track-to-track spacing) plus half the track width.

## P/N Path Generation

### Perpendicular Offsets

P and N paths are created as perpendicular offsets from the simplified centerline:

```python
def create_parallel_path_float(centerline_path, coord, sign, spacing_mm, start_dir, end_dir):
    for i, (gx, gy, layer) in enumerate(centerline_path):
        # Calculate perpendicular direction using bisector at corners
        if i == 0:
            dx, dy = start_dir      # Use stub direction at start
        elif i == len(centerline_path) - 1:
            dx, dy = end_dir        # Use stub direction at end
        else:
            dx, dy = compute_bisector(prev, curr, next_pt)  # Bisector at corners

        # Apply perpendicular offset with corner scaling for miters
        perp_x = -dy * sign * spacing_mm * corner_scale
        perp_y = dx * sign * spacing_mm * corner_scale
        result.append((x + perp_x, y + perp_y, layer))
```

### Corner Handling (Miters)

At corners, the perpendicular offset is scaled using the bisector angle to maintain constant P-N spacing:

```python
# Corner compensation: scale offset by 2/length to maintain perpendicular distance
# When summing two unit vectors, length = 2*cos(theta/2)
corner_scale = min(2.0 / bisector_length, 3.0)  # Cap at 3x to avoid extreme miters
```

### Polarity Detection

P is assigned +1 or -1 based on which side of the centerline it's on at the source:

```python
# Cross product with path direction determines side
cross = path_dir_x * to_p_dy - path_dir_y * to_p_dx
src_p_sign = +1 if cross >= 0 else -1
```

The router also detects if polarity differs between source and target (polarity swap needed) and prints this information:

```
Polarity: src_p_sign=1, tgt_p_sign=-1, swap_needed=True, has_vias=True
```

**Note:** Polarity swaps are automatically fixed by default, which swaps the target pad net assignments (P↔N) so polarity matches. Use `--no-fix-polarity` to disable this behavior.

## Via Placement

### Collinear Via Constraint

The Rust router enforces a **collinear via constraint** for differential pair routing to ensure clean via geometry. The constraint requires:

1. **2 steps minimum before via** - At least 2 grid steps must exist before placing a via
2. **Approach direction within ±45°** - The approach direction must be within ±45° of the previous direction
3. **Exit same as approach** - After the via, must continue in the same direction
4. **Then ±45° allowed** - After the exit step, can turn up to ±45°

This creates symmetric geometry around vias: `±45° → D → VIA → D → ±45°`

### Via Cost

The via cost is **doubled** for differential pairs since each layer change requires placing two vias (P and N). This discourages unnecessary layer transitions.

### Via Exclusion Zones

The router tracks via positions along the centerline path and enforces exclusion zones to prevent P/N offset tracks from conflicting with P/N vias. When a via is placed:

1. **Re-entry blocked** - After escaping the exclusion radius, the route cannot re-enter
2. **Perpendicular drift blocked** - Within half the exclusion radius, perpendicular movement is blocked
3. **Only escape allowed** - Within the exclusion zone, only moves that increase distance from the via are allowed

This prevents the centerline from drifting near its own via locations, which would cause the offset P/N tracks to intersect the offset P/N vias.

### Via Positions

P and N vias are placed perpendicular to the centerline direction at layer changes:

```python
# Average perpendicular direction from incoming and outgoing segments
perp_x = (-in_dir_y + -out_dir_y) / 2
perp_y = (in_dir_x + out_dir_x) / 2

# Via spacing (may be larger than track spacing for clearance)
via_spacing = max(spacing_mm, min_via_spacing / 2, track_via_clearance)

# Place vias perpendicular to centerline
p_via = (cx + perp_x * p_sign * via_spacing, ...)
n_via = (cx + perp_x * n_sign * via_spacing, ...)
```

### Approach and Exit Tracks

To keep the main P/N tracks parallel while routing to vias, approach and exit segments are added:

```
Main P track ────┐
                 │ approach
             [Via P]
                 │ exit
Main P track ────┘

Main N track ────┐
                 │ approach
             [Via N]
                 │ exit
Main N track ────┘
```

The approach/exit positions are calculated to:
1. Maintain track-via clearance from the inner via
2. Keep P and N tracks at constant diff pair spacing

```python
# Determine which via is "inner" (closer to turn direction)
cross = in_dir_x * out_dir_y - in_dir_y * out_dir_x
inner_is_p = (p_sign > 0) if cross >= 0 else (p_sign < 0)

# Position outer track with clearance from inner via
outer_approach = inner_via + perpendicular * track_via_clearance
```

### GND Via Placement

By default, the router places GND vias adjacent to each differential pair signal via. This provides a return current path and improves signal integrity.

```
         GND via
            ○
            │
    P via ○─┼─○ N via    (signal vias)
            │
            ○
         GND via
```

**How it works:**

1. **Rust router checks clearance** - During A* search, when evaluating via positions, the router also checks if GND via positions are clear
2. **Ahead or behind** - GND vias can be placed ahead of or behind the signal vias along the route direction. The router tries both and picks the clear option
3. **Direction stored** - The chosen direction (1=ahead, -1=behind) is stored per layer change
4. **Python places vias** - After routing, Python uses the direction info to place GND vias connected to the GND net

**GND via positions:**
- Perpendicular offset: `P/N spacing + track_width/2 + clearance + via_size/2`
- Along-heading offset: `via_size + clearance` (ahead or behind signal vias)

Use `--no-gnd-vias` to disable this feature.

## Connectors

Simple straight connectors link the original stub endpoints to the corresponding P/N track start/end points.

## Debug Visualization

With `--debug-lines`, debug geometry is output on User layers as graphic lines:

| Layer | Content |
|-------|---------|
| `User.3` | Connectors (stub to P/N track) |
| `User.4` | Stub direction arrows (1mm arrows from midpoint at src/tgt) |
| `User.7` | DRC violation debug lines (from `check_drc.py --debug-lines`) |
| `User.8` | Simplified centerline path |
| `User.9` | Raw A* centerline path |

This helps visualize the routing structure without affecting the actual routed copper layers.

## Configuration Options

| Option | Default | Description |
|--------|---------|-------------|
| `--diff-pairs` | - | Glob patterns for diff pair nets |
| `--diff-pair-gap` | 0.1 | Gap between P and N traces (mm) |
| `--diff-pair-centerline-setback` | 2x P-N dist | Distance in front of stubs to start centerline (mm) |
| `--min-turning-radius` | 0.2 | Minimum turning radius for pose-based routing (mm) |
| `--max-turn-angle` | 180 | Max cumulative turn angle (degrees) to prevent U-turns |
| `--no-fix-polarity` | false | Don't swap target pad nets when polarity swap is needed |
| `--no-gnd-vias` | false | Disable GND via placement near signal vias |
| `--swappable-nets` | - | Glob patterns for diff pair nets that can have targets swapped |
| `--crossing-penalty` | 1000.0 | Penalty for crossing assignments in target swap optimization |
| `--debug-lines` | false | Output debug geometry on User.3/4/8/9 layers |
| `--stub-proximity-radius` | 2.0 | Radius around stubs to penalize routing (mm) |
| `--stub-proximity-cost` | 0.2 | Cost penalty near stubs (mm equivalent) |
| `--bga-proximity-radius` | 10.0 | Radius around BGA edges to penalize routing (mm) |
| `--bga-proximity-cost` | 0.2 | Cost penalty near BGA edges (mm equivalent) |
| `--track-proximity-distance` | 2.0 | Radius around routed tracks to penalize (mm, same layer) |
| `--track-proximity-cost` | 0.0 | Cost penalty near routed tracks (0 = disabled) |
| `--mps-reverse-rounds` | false | Route most-conflicting MPS groups first (instead of least) |
| `--diff-pair-intra-match` | false | Match P/N lengths within each diff pair (meander shorter track) |

## Track Proximity Avoidance

The `--track-proximity-distance` and `--track-proximity-cost` options penalize routes that run close to previously routed tracks on the same layer. This encourages spread-out routing and reduces the risk of DRC violations. Disabled by default (cost = 0).

**Note:** Track proximity works correctly for differential pair routing (pose-based A*).

## Target Swap Optimization

For swappable diff pairs (e.g., memory data lanes where any source can connect to any target), the router can optimize target assignments to minimize crossings.

### Usage

```bash
python route_diff.py input.kicad_pcb output.kicad_pcb --nets "*rx*" \
    --swappable-nets "*rx*"
```

### How It Works

1. **Chip boundary detection** - Identifies source and target chips from pad positions
2. **Boundary position computation** - "Unrolls" each chip's boundary into a linear ordering
3. **Crossing detection** - Two routes cross if their source order is inverted relative to their target order
4. **Hungarian algorithm** - Finds optimal assignment minimizing total cost (distance + crossing penalty)
5. **Pad net swapping** - Swaps target pad net assignments in the PCB data before routing

The crossing penalty (default 1000) heavily discourages crossing assignments, prioritizing non-crossing routes even if they're slightly longer.

## Length Matching for Differential Pairs

Differential pairs support length matching with trombone-style meanders. This works for both single-layer and multi-layer routes:

- **Single-layer routes**: Meanders are added to straight sections of the centerline, then P/N paths are regenerated
- **Multi-layer routes**: Meanders are applied to same-layer straight sections, preserving via positions. GND vias are regenerated after meander application
- **Via barrel length**: Route length calculations include via barrel length (parsed from board stackup) for accurate length matching that matches KiCad's measurements
- **Stub via barrel length**: BGA pad vias in stubs are included using the actual stub-layer-to-pad-layer distance (not the full via span)

### Intra-Pair P/N Length Matching

Use `--diff-pair-intra-match` to match P and N track lengths within each differential pair. This is useful when P and N have different lengths due to:
- Connector regions near pads
- Curves (inner vs outer radius)
- Different via positions

When enabled:
1. After routing and inter-pair length matching, the router calculates P and N lengths for each pair
2. If the difference exceeds `--length-match-tolerance` (default 0.1mm), meanders are added to the shorter track
3. The meanders are placed with clearance checking against the other track of the pair
4. For polarity-swapped pairs, stub lengths are correctly recalculated using post-swap assignments (P gets P_source + N_target stubs, N gets N_source + P_target stubs)

```bash
# Enable intra-pair matching
python route_diff.py input.kicad_pcb output.kicad_pcb --nets "*DQS*" \
    --diff-pair-intra-match

# Combine with inter-pair matching
python route_diff.py input.kicad_pcb output.kicad_pcb --nets "*DQS*" "*CK*" \
    --length-match-group auto \
    --diff-pair-intra-match
```

**Execution order**: Inter-pair matching runs first (on centerline), then intra-pair matching adds meanders to individual P/N tracks. This order is intentional - inter-pair regenerates P/N from the meandered centerline, so intra-pair must run last to preserve its meanders.

### Time Matching

Use `--time-matching` as an alternative to length matching when routes traverse layers with different dielectric properties. Time matching accounts for the different signal propagation speeds:

- **Outer layers (microstrip)**: ~5.4 ps/mm for typical FR4
- **Inner layers (stripline)**: ~6.9 ps/mm for typical FR4

This is useful when differential pairs in a group are routed on different layers. Time matching ensures equal propagation delay rather than equal physical length.

```bash
# Time match differential pairs
python route_diff.py input.kicad_pcb output.kicad_pcb --nets "*DQS*" \
    --length-match-group auto \
    --time-matching \
    --time-match-tolerance 1.0
```

## Limitations

1. **Polarity swap** - Enabled by default; use `--no-fix-polarity` to disable automatic target pad swapping
2. **Fixed spacing** - Spacing is constant along the route (no tapering)
3. **Grid snapping** - Centerline endpoints snap to grid

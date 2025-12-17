# Differential Pair Routing

This document describes how the router handles differential pairs, including P/N pairing, centerline routing, and via placement.

## Overview

Differential pairs are routed using a **centerline + offset** approach:

1. Route a single centerline path using A* with extra clearance
2. Add turn segments at start/end to align with stub directions
3. Generate P and N paths as perpendicular offsets from centerline
4. Add approach/exit tracks at vias to keep P/N parallel

## Identifying Differential Pairs

The router recognizes common differential pair naming conventions:

| Convention | Example P | Example N |
|------------|-----------|-----------|
| `_P` / `_N` suffix | `LVDS_CLK_P` | `LVDS_CLK_N` |
| `P` / `N` suffix | `DATA0P` | `DATA0N` |
| `+` / `-` suffix | `CLK+` | `CLK-` |

## Usage

### Basic Usage

```bash
# Route a specific diff pair
python test_diffpair.py "*lvds_rx1_11" --stub-proximity-radius 4

# Route with debug visualization
python test_diffpair.py "*lvds_rx1_11" --stub-proximity-radius 4 --debug-layers
```

### Direct Router Usage

```bash
python route.py input.kicad_pcb output.kicad_pcb "*lvds*" \
    --diff-pairs "*lvds*" \
    --diff-pair-gap 0.1 \
    --diff-pair-centerline-setback 1.5
```

The `--diff-pairs` pattern identifies which nets should be routed as pairs. Both P and N nets must match the pattern.

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

### Turn Segments

Turn segments are added at the start and end of the centerline to align the route with the stub directions. This creates a smooth transition from the routed path to the connectors.

### Extra Clearance

The centerline is routed with extra clearance to accommodate both P and N tracks:

```python
# Extra clearance = spacing from centerline + one track width
extra_clearance = (track_width + diff_pair_gap) / 2 + track_width
```

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

**Note:** Polarity swaps can be automatically fixed using `--fix-polarity`, which swaps the target pad net assignments (P↔N) so polarity matches. Without this option, routes requiring polarity swaps may need manual adjustment.

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

## Connectors

Simple straight connectors link the original stub endpoints to the corresponding P/N track start/end points.

## Debug Visualization

With `--debug-layers`, debug geometry is output on special layers:

| Layer | Content |
|-------|---------|
| `In4.Cu` | Turn segments (first and last segments of P/N paths) |
| `In5.Cu` | Connectors (stub to P/N track) |
| `User.8` | Simplified centerline path |
| `User.9` | Raw A* centerline path |

This helps visualize the routing structure without affecting the actual routed layers.

## Configuration Options

| Option | Default | Description |
|--------|---------|-------------|
| `--diff-pairs` | - | Glob patterns for diff pair nets |
| `--diff-pair-gap` | 0.1 | Gap between P and N traces (mm) |
| `--diff-pair-centerline-setback` | 1.5 | Distance in front of stubs to start centerline (mm) |
| `--fix-polarity` | false | Swap target pad nets when polarity swap is needed |
| `--debug-layers` | false | Output debug geometry on In4/In5/User.8/User.9 |
| `--stub-proximity-radius` | 1.0 | Radius around stubs to penalize routing (mm) |
| `--stub-proximity-cost` | 3.0 | Cost penalty near stubs (mm equivalent) |

## Limitations

1. **Polarity swap requires --fix-polarity** - Polarity swaps are detected; use `--fix-polarity` to automatically swap target pads
2. **No length matching** - P and N paths may have slightly different lengths
3. **Fixed spacing** - Spacing is constant along the route (no tapering)
4. **Grid snapping** - Centerline endpoints snap to grid

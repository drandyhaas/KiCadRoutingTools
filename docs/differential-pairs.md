# Differential Pair Routing

This document describes how the router handles differential pairs, including P/N pairing, polarity swaps, and via placement.

## Overview

Differential pairs are routed using a **centerline + offset** approach:

1. Route a single centerline path using A* with extra clearance
2. Generate P and N paths as perpendicular offsets from centerline
3. Handle polarity swaps at layer transitions (vias)

## Identifying Differential Pairs

The router recognizes common differential pair naming conventions:

| Convention | Example P | Example N |
|------------|-----------|-----------|
| `_P` / `_N` suffix | `LVDS_CLK_P` | `LVDS_CLK_N` |
| `P` / `N` suffix | `DATA0P` | `DATA0N` |
| `+` / `-` suffix | `CLK+` | `CLK-` |

```python
def extract_diff_pair_base(net_name):
    if net_name.endswith('_P'):
        return (net_name[:-2], True)  # (base_name, is_positive)
    if net_name.endswith('_N'):
        return (net_name[:-2], False)
    # ... other conventions
```

## Usage

```bash
# Route nets matching *lvds* as differential pairs
python batch_grid_router.py input.kicad_pcb output.kicad_pcb "*lvds*" \
    --diff-pairs "*lvds*" \
    --diff-pair-gap 0.1
```

The `--diff-pairs` pattern identifies which nets should be routed as pairs. Both P and N nets must match the pattern.

## Centerline Routing

### Endpoint Detection

The router finds paired P/N endpoints by:

1. Getting free ends of P and N stub groups
2. Finding closest P-N pairs on the same layer
3. Designating one pair as source, other as target

```python
def get_diff_pair_endpoints(pcb_data, p_net_id, n_net_id, config):
    # Get stub free ends for P and N
    p_sources, p_targets, _ = get_net_endpoints(pcb_data, p_net_id, config, use_stub_free_ends=True)
    n_sources, n_targets, _ = get_net_endpoints(pcb_data, n_net_id, config, use_stub_free_ends=True)

    # Find closest P-N pair (this becomes "source")
    p_src, n_src, _ = find_closest_pair(p_all, n_all)

    # Remaining closest pair becomes "target"
    p_tgt, n_tgt, _ = find_closest_pair(p_remaining, n_remaining)

    return paired_sources, paired_targets
```

### Centerline Path

The centerline is routed with extra clearance to accommodate both P and N tracks:

```python
# Extra clearance = spacing from centerline + one track width
extra_clearance = (track_width + diff_pair_gap) / 2 + track_width

# Build obstacle map with extra clearance
obstacles = build_base_obstacle_map(pcb_data, config, nets_to_route, extra_clearance)

# Route centerline
router = GridRouter(via_cost=config.via_cost * 1000)
path, iterations = router.route_multi(obstacles, center_sources, center_targets)
```

### Setback from Stubs

The centerline starts a configurable distance ("setback") in front of the stubs:

```
Stub tips:  P---o       o---P
                 \     /
Centerline:       o---o
                  ^
                  Setback distance
```

Configure with `--min-diff-pair-centerline-setback` and `--max-diff-pair-centerline-setback`.

## P/N Path Generation

### Perpendicular Offsets

P and N paths are created as perpendicular offsets from the simplified centerline:

```python
def create_parallel_path_float(centerline_path, coord, sign, spacing_mm, start_dir, end_dir):
    for i, (gx, gy, layer) in enumerate(centerline_path):
        x, y = coord.to_float(gx, gy)

        # Calculate perpendicular direction using bisector at corners
        if i == 0:
            # Use stub direction at start
            dx, dy = start_dir
        elif i == len(centerline_path) - 1:
            # Use stub direction at end
            dx, dy = end_dir
        else:
            # Use bisector of incoming/outgoing directions
            dx, dy = compute_bisector(prev, curr, next_pt)

        # Apply perpendicular offset
        perp_x = -dy * sign * spacing_mm
        perp_y = dx * sign * spacing_mm
        result.append((x + perp_x, y + perp_y, layer))
```

### Polarity Detection

P is assigned +1 or -1 based on which side of the centerline it's on at the source:

```python
# Vector from centerline to P position
to_p_dx = p_src_x - midpoint_x
to_p_dy = p_src_y - midpoint_y

# Cross product with path direction
cross = path_dir_x * to_p_dy - path_dir_y * to_p_dx
p_sign = +1 if cross >= 0 else -1
n_sign = -p_sign
```

## Polarity Swaps

### When Swaps Occur

A polarity swap is needed when P and N are on opposite sides of the centerline at source vs target:

```
Source side:  P---N       (P on left)
                  |
Target side:  N---P       (P on right)
```

### Handling at Vias

When a polarity swap is needed and there's a layer change (via), the swap happens at the via position:

1. P and N vias are placed perpendicular to centerline
2. For polarity swap, via positions are swapped
3. An arc detour is added to route one track around the other's via

```python
if polarity_swap_needed and has_layer_change:
    # Swap via positions
    p_via_x, n_via_x = n_via_x, p_via_x
    p_via_y, n_via_y = n_via_y, p_via_y
```

### Arc Detour

When P needs to swap sides, it routes around N's via in an arc:

```python
def _handle_polarity_swap_detour(p_path, n_path, ...):
    # Generate arc points around N via
    arc_radius = 1.25 * track_via_clearance

    for angle in arc_angles:
        arc_x = n_via_x + arc_radius * cos(angle)
        arc_y = n_via_y + arc_radius * sin(angle)
        arc_points.append((arc_x, arc_y))

    # Insert arc into P path
    p_path[via_idx:via_idx] = arc_points
```

## Via Placement

### Via Positions

P and N vias are placed perpendicular to the centerline direction:

```python
# Get direction at via location
perp_x, perp_y = perpendicular_to_centerline(via_idx)

# Via spacing (may be larger than track spacing for clearance)
via_spacing = max(spacing_mm, min_via_spacing / 2, track_via_clearance)

# Place vias
p_via = (cx + perp_x * p_sign * via_spacing, cy + perp_y * p_sign * via_spacing)
n_via = (cx + perp_x * n_sign * via_spacing, cy + perp_y * n_sign * via_spacing)
```

### Approach and Exit Segments

Special segments connect the main diff pair path to vias while maintaining clearance:

```
Main path ─────┐
               │  Approach
           [Via P]
               │  Exit
             ─────
```

The approach/exit angles are calculated to avoid the inner via:

```python
# Determine which via is "inner" (closer to turn direction)
if cross < 0:
    inner_via = p_via if p_sign < 0 else n_via

# Route outer track around inner via with clearance
outer_approach_offset = track_via_clearance + diff_pair_spacing
```

## Configuration Options

| Option | Default | Description |
|--------|---------|-------------|
| `--diff-pairs` | - | Glob patterns for diff pair nets |
| `--diff-pair-gap` | 0.1 | Gap between P and N (mm) |
| `--min-diff-pair-centerline-setback` | 0.4 | Min setback from stubs (mm) |
| `--max-diff-pair-centerline-setback` | 0.4 | Max setback from stubs (mm) |

## Limitations

1. **Single via transition** - Currently limited to at most one layer change per diff pair route
2. **No length matching** - P and N paths may have slightly different lengths
3. **Fixed spacing** - Spacing is constant along the route (no tapering)
4. **Grid snapping** - Endpoints snap to grid, may not perfectly align with off-grid stubs

# Routing Architecture

This document describes the internal architecture of the KiCad Grid Router, including module structure, obstacle maps, and the A* pathfinding algorithm.

## Module Structure

The router is organized into focused modules with clear separation of concerns:

```
KiCadRoutingTools/
├── route.py                   # Main CLI - single-ended routing
├── route_diff.py              # Main CLI - differential pair routing
├── routing_config.py          # Configuration dataclasses
├── routing_state.py           # RoutingState - tracks progress and results
├── routing_context.py         # Helper functions for obstacle building
├── routing_common.py          # Shared utilities for route.py and route_diff.py
├── routing_utils.py           # Shared utilities
├── obstacle_map.py            # Obstacle map building
├── bresenham_utils.py         # Bresenham line-walking utilities
│
├── diff_pair_loop.py          # Diff pair routing main loop
├── single_ended_loop.py       # Single-ended routing main loop
├── reroute_loop.py            # Reroute queue processing
├── diff_pair_routing.py       # Diff pair A* implementation
├── single_ended_routing.py    # Single-ended A* implementation
│
├── layer_swap_optimization.py # Upfront layer optimization
├── layer_swap_fallback.py     # Fallback layer swap on failure
├── rip_up_reroute.py          # Rip-up and reroute logic
├── blocking_analysis.py       # Analyze blocking nets
├── target_swap.py             # Target assignment optimization
├── length_matching.py         # Trombone meander length matching
│
├── kicad_parser.py            # KiCad file parsing (including stackup for via barrel length)
├── kicad_writer.py            # KiCad S-expression output
├── output_writer.py           # Route output and debug geometry
├── geometry_utils.py          # Shared geometry calculations
└── rust_router/               # Rust A* implementation
```

### Module Responsibilities

#### Core

| Module | Purpose |
|--------|---------|
| `route.py` | CLI for single-ended routing |
| `route_diff.py` | CLI for differential pair routing |
| `routing_config.py` | `GridRouteConfig`, `GridCoord`, `DiffPair` dataclasses |
| `routing_state.py` | `RoutingState` class tracking routing progress, results, and PCB modifications |
| `routing_context.py` | Helper functions for building obstacles and recording route success |
| `routing_common.py` | Shared utilities: BGA zone setup, net resolution, length matching, pcb_data sync |
| `routing_utils.py` | Core utilities: `pos_key()`, `segment_length()`, `build_layer_map()`, `POSITION_DECIMALS` |
| `obstacle_map.py` | Building `GridObstacleMap` from PCB data (segments, vias, pads, BGA zones) |
| `bresenham_utils.py` | Bresenham line-walking utilities used by obstacle building and blocking analysis |
| `geometry_utils.py` | Shared geometry calculations (point-to-segment distance, segment intersection, closest points) |

#### Routing Loops

| Module | Purpose |
|--------|---------|
| `diff_pair_loop.py` | Main loop iterating over differential pairs to route |
| `single_ended_loop.py` | Main loop iterating over single-ended nets to route |
| `reroute_loop.py` | Processes queue of routes that need rerouting after rip-up |
| `diff_pair_routing.py` | Routes P/N pairs using centerline + offset approach with GND vias |
| `single_ended_routing.py` | Routes individual nets using A* pathfinding; multi-point routing (3+ pads) |

#### Optimization

| Module | Purpose |
|--------|---------|
| `layer_swap_optimization.py` | Analyzes and performs layer swaps before routing starts |
| `layer_swap_fallback.py` | Attempts layer swap when a route fails |
| `rip_up_reroute.py` | Rips up blocking routes and adds them to reroute queue |
| `blocking_analysis.py` | Identifies which previously-routed nets are blocking |
| `target_swap.py` | Hungarian algorithm for optimal source-to-target assignment |
| `length_matching.py` | Adds trombone meanders to match route lengths within groups |

## Grid Coordinate System

The router operates on an integer grid for efficient pathfinding. The `GridCoord` class handles conversions:

```python
class GridCoord:
    def __init__(self, grid_step: float = 0.1):
        self.grid_step = grid_step  # mm per grid unit

    def to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert mm coordinates to grid coordinates."""
        return (round(x / self.grid_step), round(y / self.grid_step))

    def to_float(self, gx: int, gy: int) -> Tuple[float, float]:
        """Convert grid coordinates to mm coordinates."""
        return (gx * self.grid_step, gy * self.grid_step)
```

With the default 0.1mm grid step:
- Grid coordinate (100, 200) = (10.0mm, 20.0mm)
- All routing decisions happen in integer grid space
- Final output is converted back to floating-point mm

## Obstacle Map Architecture

The `GridObstacleMap` (implemented in Rust) tracks blocked cells efficiently using hash sets.

### Obstacle Types

1. **Blocked Cells** - Cells where tracks cannot be routed on a specific layer
2. **Blocked Vias** - Cells where vias cannot be placed (any layer)
3. **BGA Zones** - Rectangular areas blocking both tracks and vias
4. **Allowed Cells** - Override for BGA zones at source/target positions
5. **Source/Target Cells** - Override for track clearance at endpoints
6. **Stub Proximity Costs** - Soft penalties to avoid routing near unrouted stubs

### Building the Obstacle Map

```python
def build_base_obstacle_map(pcb_data, config, nets_to_route, extra_clearance=0.0):
    """Build base obstacle map with static obstacles."""
    obstacles = GridObstacleMap(num_layers)

    # 1. Add BGA exclusion zones (block all layers + vias)
    for zone in config.bga_exclusion_zones:
        obstacles.set_bga_zone(min_x, min_y, max_x, max_y)

    # 2. Add existing segments (with clearance expansion)
    for seg in pcb_data.segments:
        if seg.net_id not in nets_to_route:
            _add_segment_obstacle(obstacles, seg, ...)

    # 3. Add existing vias
    for via in pcb_data.vias:
        if via.net_id not in nets_to_route:
            _add_via_obstacle(obstacles, via, ...)

    # 4. Add pads
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id not in nets_to_route:
            for pad in pads:
                _add_pad_obstacle(obstacles, pad, ...)

    return obstacles
```

### Clearance Expansion

Obstacles are expanded by track clearance to ensure DRC compliance. Track widths can vary based on:
- **Layer** (impedance-controlled routing): microstrip on outer layers is wider than stripline on inner layers
- **Net** (power net routing): power/ground nets can use wider tracks than signal nets

Clearance calculations use per-net-per-layer widths:

```python
# Per-net-per-layer track clearance expansion
for layer_name in config.layers:
    # get_net_track_width checks power net overrides first, then layer widths (impedance)
    layer_width = config.get_net_track_width(net_id, layer_name)
    expansion_mm = layer_width / 2 + clearance
    expansion_grid = coord.to_grid_dist(expansion_mm)

    # Via blocking near tracks - uses to_grid_dist_safe() for safety margin
    via_block_mm = via_size / 2 + layer_width / 2 + clearance
    via_block_grid = coord.to_grid_dist_safe(via_block_mm)
```

A segment at position (x, y) blocks all cells within `expansion_grid` radius on its layer, and blocks via placement within `via_block_grid` radius.

**Grid quantization safety**: Via-related clearances use `to_grid_dist_safe()` which adds half a grid step before rounding. This prevents edge cases where clearances round down and cause minor DRC violations (e.g., 0.01mm overlaps).

## A* Pathfinding Algorithm

The core routing uses A* pathfinding implemented in Rust for performance.

### State Space

Each state is a tuple `(gx, gy, layer_idx)`:
- `gx, gy`: Grid coordinates
- `layer_idx`: Layer index (0 = F.Cu, 1 = In1.Cu, etc.)

### Moves

From any cell, the router can move:
- **8 directions on same layer**: N, NE, E, SE, S, SW, W, NW (octilinear)
- **Layer change** (via): Stay at same (gx, gy), change layer

### Cost Function

```
g(n) = actual cost from start to n
h(n) = heuristic estimate from n to goal
f(n) = g(n) + h(n) * heuristic_weight
```

Move costs:
- Orthogonal move: 1000 (1 grid step)
- Diagonal move: 1414 (sqrt(2) grid steps)
- Via (layer change): `via_cost * 1000`

### Heuristic

The heuristic uses 3D Chebyshev distance to the nearest target:

```python
def heuristic(gx, gy, layer, targets):
    min_dist = infinity
    for tx, ty, tl in targets:
        dx = abs(gx - tx)
        dy = abs(gy - ty)
        dz = abs(layer - tl) * via_cost
        dist = max(dx, dy) + dz
        min_dist = min(min_dist, dist)
    return min_dist * 1000 * heuristic_weight
```

### Multi-Source/Multi-Target Routing

The router supports multiple sources and targets for flexible endpoint matching:

```python
path, iterations = router.route_multi(
    obstacles,
    sources=[(gx1, gy1, layer1), (gx2, gy2, layer1), ...],
    targets=[(gx3, gy3, layer2), (gx4, gy4, layer2), ...],
    max_iterations=200000
)
```

This allows routing from any source endpoint to any target endpoint.

### Direction Fallback

If routing fails in one direction, the router tries the reverse:

```python
# Try forward direction
path, iter1 = router.route_multi(obstacles, sources, targets, max_iter)

if path is None:
    # Try reverse direction
    path, iter2 = router.route_multi(obstacles, targets, sources, max_iter)
    if path:
        path = reversed(path)
```

## Incremental Obstacle Updates

For batch routing, obstacles are updated incrementally as each net is routed:

```python
# Build base obstacle map once
base_obstacles = build_base_obstacle_map(pcb_data, config, all_nets)

for net_id in nets_to_route:
    # Clone base obstacles
    obstacles = base_obstacles.clone()

    # Add other unrouted nets' stubs as obstacles
    for other_id in remaining_nets:
        if other_id != net_id:
            add_net_stubs_as_obstacles(obstacles, pcb_data, other_id, config)

    # Route the net
    result = route_net_with_obstacles(pcb_data, net_id, config, obstacles)

    # Add routed path to pcb_data for next iteration
    if result:
        add_route_to_pcb_data(pcb_data, result)
```

This approach:
1. Builds static obstacles once (O(n) where n = total PCB elements)
2. Clones and updates per-net (O(m) where m = remaining nets)
3. Achieves ~7x speedup over rebuilding from scratch

## Path Conversion

After A* finds a path, it's converted to KiCad geometry:

```python
for i in range(len(path) - 1):
    gx1, gy1, layer1 = path[i]
    gx2, gy2, layer2 = path[i + 1]

    x1, y1 = coord.to_float(gx1, gy1)
    x2, y2 = coord.to_float(gx2, gy2)

    if layer1 != layer2:
        # Layer change = via
        vias.append(Via(x=x1, y=y1, ...))
    else:
        # Same layer = segment
        segments.append(Segment(x1, y1, x2, y2, layer=layer1, ...))
```

## Multi-Point Routing

Nets with 3+ pads require special handling to work correctly with length matching. The router uses an MST-based 3-phase approach:

### The Problem

When a net has more than 2 pads (e.g., a signal that fans out to multiple destinations), routing all connections before length matching creates issues:
- Tap connections that branch off the main route add non-linear geometry
- The length-matching algorithm can't cleanly add meanders to branching routes
- Tap segments shouldn't be included in length-matched measurements

### MST-Based 3-Phase Solution

The router computes a Minimum Spanning Tree (MST) between all pad positions using Manhattan distance, then routes edges in length order (longest first):

```
Phase 1: Route longest MST edge    Phase 2: Length match      Phase 3: Route remaining MST edges

    B                                  B                           B
    |                                  |                           |
A---+  (longest edge A-C)          A~~~+~~~C                   A~~~+~~~C
    |                                 meanders                     |
    C                                  D (unrouted)                D (connected via MST edge)
    D (unrouted)
```

#### Phase 1: Main Route
- Compute MST between all pads using Manhattan distance
- Sort MST edges by length (longest first)
- Route the **longest** MST edge using standard A* routing
- Track pending multi-point nets in `state.pending_multipoint_nets`
- Result includes `mst_edges` (sorted longest-first) for Phase 3

#### Phase 2: Length Matching
- Apply meanders to the clean 2-point main routes
- Works exactly like regular 2-point net length matching
- Meanders are added to the main route segments

#### Phase 3: Tap Connections
- Route remaining MST edges in length order (longest first)
- Each edge must connect an already-routed pad to an unrouted pad
- Find tap point on existing segments near the source pad
- Route from tap point to the target pad
- Uses `route_multipoint_taps()` in `single_ended_routing.py`

### Implementation Details

Key functions in `single_ended_routing.py`:
- `route_multipoint_main()` - Phase 1: Computes MST, routes longest edge
- `route_multipoint_taps()` - Phase 3: Routes remaining MST edges by length

Key utility in `routing_utils.py`:
- `compute_mst_edges()` - Computes MST edges with indices using Prim's algorithm (supports Manhattan or Euclidean distance)

State tracking in `routing_state.py`:
- `pending_multipoint_nets` - Dict mapping net_id to main_result for Phase 3

### Limitations

- **Layer swaps not supported** - Multi-point nets are skipped during layer swap optimization (warning printed)
- Layer swaps would need to handle all pads consistently, which is complex when pads may be on different layers

## Post-Processing

After routing, segments undergo cleanup:

1. **Self-intersection fixing** - Adjusts short connector segments that cross existing tracks
2. **Appendix collapsing** - Removes short dead-end segments created by grid snapping
3. **Degenerate segment removal** - Filters segments shorter than 0.01mm

See `route_modification.py` for implementation details.

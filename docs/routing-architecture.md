# Routing Architecture

This document describes the internal architecture of the KiCad Grid Router, including module structure, what one route step does from start to finish, obstacle maps, and the A* pathfinding algorithm.

For per-module API references (function signatures, dataclass fields, usage examples) see the [Python API documentation](python-api.md).

## Module Structure

The router is organized into focused modules. The main ones, by role:

```
KiCadRoutingTools/
├── py_router/                     # Python engine + CLI entry points (#522)
│   ├── route.py                   # Main CLI + batch_route() - single-ended routing, ends with the plane finalize (#562)
│   ├── route_diff.py              # Main CLI + batch_route_diff_pairs() - differential pair routing
│   ├── route_planes.py            # Main CLI - plane pours (#562 bare pour), stitching, GND return vias
│   ├── repair_planes.py           # Plane repair engine (the finalize's) + standalone CLI
│   │
│   ├── routing_config.py          # GridRouteConfig, GridCoord, DiffPairNet
│   ├── routing_defaults.py        # Every default parameter value
│   ├── routing_state.py           # RoutingState - tracks progress and results
│   ├── routing_context.py         # Per-net obstacle preparation, recording a success
│   ├── routing_common.py          # Shared utilities for route.py and route_diff.py
│   ├── routing_utils.py           # Shared utilities (pad rasterization, pos_key, layer map)
│   ├── design_rules.py            # KiCad design rules, resolved in KiCad's order (#530)
│   ├── kicad_dru.py               # Per-layer clearance rules from .kicad_dru (#498)
│   ├── fab_tiers.py               # Fab floors and the escalation policy
│   ├── obstacle_map.py            # Base obstacle map building
│   ├── obstacle_cache.py          # Per-net obstacle cache for incremental updates
│   ├── obstacle_costs.py          # Stub / track proximity soft costs
│   ├── history_congestion.py      # Negotiated-congestion history cost (#590)
│   ├── plane_fragility.py         # Soft cost on the narrow parts of a pour (#424)
│   ├── bresenham_utils.py         # Bresenham line-walking utilities
│   │
│   ├── single_ended_loop.py       # Single-ended routing main loop
│   ├── diff_pair_loop.py          # Diff pair routing main loop
│   ├── reroute_loop.py            # Reroute queue processing
│   ├── phase3_routing.py          # Multi-point tap routing (Phase 3)
│   ├── single_ended_routing.py    # Single-ended A* routing, multi-point main + taps
│   ├── diff_pair_routing.py       # Diff pair A* (centerline + P/N offset)
│   ├── diff_pair_multipoint.py    # Multi-point and hybrid diff pairs
│   │
│   ├── net_ordering.py            # MPS, inside-out, original, bus ordering
│   ├── bus_detection.py           # Bus groups (--bus)
│   ├── bus_corridor.py            # One planned centerline per bus
│   ├── layer_swap_optimization.py # Upfront layer swaps
│   ├── layer_swap_fallback.py     # Layer swap when a route fails
│   ├── rip_up_reroute.py          # Rip-up and restore
│   ├── blocking_analysis.py       # Which routed nets block a failed route
│   ├── target_swap.py             # Target assignment optimization
│   ├── length_matching.py         # Trombone meander length matching
│   ├── net_rescue.py              # Last-chance rescue + terminal geometry escalation
│   │
│   ├── plane_pad_tap.py           # Plane pad taps (finalize / repair)
│   ├── plane_region_connector.py  # Joins between disconnected plane regions
│   ├── kicad_oracle.py            # Reconnect against KiCad's own connectivity
│   ├── cleanup_pipeline.py        # The one ordered post-route cleanup pipeline
│   ├── improvement_gate.py        # Never ship a board a run made worse (#600)
│   ├── fix_kicad_drc_settings.py  # Write the routed floors into the .kicad_pro
│   │
│   ├── kicad_parser.py            # KiCad file parsing (including stackup for via barrel length)
│   ├── kicad_writer.py            # KiCad S-expression output
│   ├── output_writer.py           # Route output and debug geometry
│   ├── pcb_modification.py        # Add/remove routes in PCBData; the cleanup passes
│   ├── connectivity.py            # Endpoints, connected groups, MST edges
│   └── geometry_utils.py          # Shared geometry calculations
└── rust_router/                   # Rust A* implementation (grid_router module)
```

The other `py_router/` modules are documented in the README's [Module Overview](../README.md#module-overview) and in their own docstrings.

### Module Responsibilities

#### Core

| Module | Purpose |
|--------|---------|
| `route.py` | CLI for single-ended routing; `batch_route()` is the shared engine both the CLI and the GUI call |
| `route_diff.py` | CLI for differential pair routing; `batch_route_diff_pairs()` is its shared engine |
| `routing_config.py` | `GridRouteConfig`, `GridCoord`, `DiffPairNet`; per-net widths and clearances (`get_net_track_width`, `obstacle_clearance`, `pair_clearance`) |
| `routing_state.py` | `RoutingState` class tracking routing progress, results, and PCB modifications |
| `routing_context.py` | Prepares the working obstacle map for one net in place (`prepare_obstacles_inplace` / `restore_obstacles_inplace`) and records a route's success |
| `routing_common.py` | Shared utilities: BGA zone setup, net resolution, length matching, pcb_data sync |
| `routing_utils.py` | Core utilities: `pos_key()`, `segment_length()`, `build_layer_map()`, rotation-aware pad rasterization (`pad_blocked_cells_array`, `iter_pad_blocked_cells`) |
| `design_rules.py` | One resolver for every constraint, in KiCad's DRC order: pad overrides, net-class implicit rules, `.kicad_dru` rules, Board Setup minimums, the fab floor last |
| `obstacle_map.py` | Building `GridObstacleMap` from PCB data (segments, vias, pads, BGA zones, board edge, keep-outs, drill holes) |
| `obstacle_cache.py` | Per-net obstacle cache: remove a net's own copper, route, add the new copper back (ref-counted) |
| `bresenham_utils.py` | Bresenham line-walking utilities used by obstacle building and blocking analysis |
| `geometry_utils.py` | Shared geometry calculations (point-to-segment distance, segment intersection, closest points, `simplify_path`) |

#### Routing Loops

| Module | Purpose |
|--------|---------|
| `single_ended_loop.py` | Main loop iterating over single-ended nets to route, with the rip-up ladder |
| `diff_pair_loop.py` | Main loop iterating over differential pairs to route; defers electrically-short legs (< ~5× connector setback) to single-ended |
| `diff_pair_multipoint.py` | Multi-point chain routing; same-component terminal pairing, per-leg coupled/single-ended decision (`leg_electrically_short`) |
| `reroute_loop.py` | Processes the queue of nets that need rerouting after rip-up |
| `phase3_routing.py` | Phase 3 of multi-point routing: the remaining pads tapped onto each net's main route |
| `single_ended_routing.py` | Routes individual nets using A* pathfinding; multi-point routing (3+ pads) |
| `diff_pair_routing.py` | Routes P/N pairs using centerline + offset approach with GND vias (the Rust `PoseRouter`, orientation-aware) |

#### Optimization and Recovery

| Module | Purpose |
|--------|---------|
| `net_ordering.py` | Net order: MPS (default, crossing-aware), inside-out, original, or bus groups first |
| `layer_swap_optimization.py` | Analyzes and performs layer swaps before routing starts |
| `layer_swap_fallback.py` | Attempts layer swap when a route fails |
| `rip_up_reroute.py` | Rips up blocking routes and adds them to the reroute queue; restores them when the retry fails |
| `blocking_analysis.py` | Identifies which previously-routed nets block a failed route's search frontier |
| `target_swap.py` | Hungarian algorithm for optimal source-to-target assignment |
| `length_matching.py` | Adds trombone meanders to match route lengths within groups |
| `history_congestion.py` | PathFinder-style history cost: cells repeatedly contested get dearer (#590) |
| `net_rescue.py` | Scoped retry of still-failed nets at a finer grid / narrower floor, then a whole-net retry with width and via stepping down to the fab floor |

#### Planes, Post-Route and Output

| Module | Purpose |
|--------|---------|
| `repair_planes.py` | Plane repair engine: pad taps (`plane_pad_tap.py`) and region joins (`plane_region_connector.py`), plus the plane-copper cleanup |
| `kicad_oracle.py` | Reconnect pass that routes the exact links KiCad's own connectivity (exact zone fill) reports open |
| `cleanup_pipeline.py` | `run_post_route_cleanup()`: the ordered cleanup passes both routing fronts run |
| `improvement_gate.py` | Compares the written board's connectivity against the input's and reverts a step that made it worse |
| `fix_kicad_drc_settings.py` | Writes the clearances and sizes the board was routed to into the output's `.kicad_pro` |
| `output_writer.py` / `kicad_writer.py` | Write the routed copper, swaps and debug geometry into the output file |

## A Route Step, End to End

`route.py`'s `batch_route()` runs these phases in order. The GUI's Route tab
calls the same function (with `return_results=True`), so everything below
happens on both fronts.

1. **Read the board and its rules.** Parse the board (or take the GUI's
   in-memory `PCBData`), resolve the `--nets` / `--component` / `--group`
   scope, and build the `GridRouteConfig`. The config carries the per-net
   clearances from the net classes in the sibling `.kicad_pro`, the per-layer
   rules from `.kicad_dru`, net-filtered keep-outs, keep-away rules, BGA
   exclusion zones and any guide corridor.
2. **Order the nets.** Upfront layer swaps
   (`layer_swap_optimization.py`), then MPS ordering (or inside-out,
   original, bus).
3. **Build the obstacle maps.** The base map holds everything that does not
   move during the run (`build_base_obstacle_map`); the per-net cache holds
   each routed net's own copper (`precompute_all_net_obstacles`); the
   working map is the base plus every cached net. Soft cost fields are
   registered too: track proximity, plane fragility, history congestion.
4. **Route the nets** (`route_single_ended_nets`). Each net gets the working
   map with its own copper lifted out, a probe-then-full A* search, and on
   failure the rip-up ladder: blocking analysis names the nets in the way,
   they are ripped, the net retries, and the ripped nets go to the reroute
   queue (`run_reroute_loop`). See [Rip-Up and Reroute](rip-up-reroute.md).
5. **Length matching** for any `--length-match-group` (Phase 2 of
   multi-point routing, below).
6. **Phase 3 taps** (`run_phase3_tap_routing`): every multi-point net's
   remaining pads join its main route, followed by another reroute loop.
7. **Recovery.** Nets ripped to avoid a short are re-routed (#134); ripped
   nets whose reroute never landed get their copper restored where it does
   not collide (`run_casualty_reconcile`); `net_rescue` retries each
   still-open net in a scoped window at finer parameters, then the terminal
   geometry escalation retries the whole net with width and via stepping down
   toward the fab floor.
8. **Cleanup** (`run_post_route_cleanup`, below), then the stale-input strip,
   via de-duplication and the authoritative connectivity count.
9. **Write** the output board and the routed floors into its `.kicad_pro`.
10. **Plane finalize (#562).** On a pours-first chain the plane-net pads were
    already welded into the fill during routing; the finalize runs the
    `repair_planes` engine in-run for whatever the fill cannot reach (pad
    taps, region joins, with rip authority over unprotected blockers) and a
    KiCad-oracle pass against the exact fill. It runs even when the step had
    nothing else to route (#1112). `KICAD_PLANE_FINALIZE=0` is the kill
    switch.
11. **Final reconciliation.** Up to three laps that re-run `batch_route` on
    the nets still incomplete, against the finished board.
12. **Late passes on the board it ships:** the audit of every ball a fanout
    promised to serve by fill contact (#678), soft-joint bridges, the strict
    redundant-copper collapse (#1063), dangling-via and orphan sweeps,
    via-in-pad protection (#962), the power-width report, a final re-grade,
    and the improvement gate (#600).

`--quick` skips the rescue and escalation of step 7, the plane finalize and
the reconciliation; `--quick 2` also skips the passes that only polish copper
for shipping (among them the late passes of step 12). Such a board judges a
placement; it is never shipped.

`route_diff.py`'s `batch_route_diff_pairs()` follows the same shape for
pairs: the diff-pair loop, the reroute loop, casualty reconcile, length
matching, the same cleanup pipeline and the same writer. It has no plane
finalize or reconciliation; the `route.py` step that follows in a chain
does that.

`route_planes.py` is a separate step that runs **first** in a chain: it pours
the zones and places no per-pad taps (#562 bare pour). See
[Plane Routing](route-plane.md).

## Grid Coordinate System

The router operates on an integer grid for efficient pathfinding. The `GridCoord` class handles conversions:

```python
class GridCoord:
    def __init__(self, grid_step: float = 0.1):
        self.grid_step = grid_step  # mm per grid unit
        self.inv_step = 1.0 / grid_step

    def to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert mm coordinates to grid coordinates."""
        return (round(x * self.inv_step), round(y * self.inv_step))

    def to_float(self, gx: int, gy: int) -> Tuple[float, float]:
        """Convert grid coordinates to mm coordinates."""
        return (gx * self.grid_step, gy * self.grid_step)

    def to_grid_dist(self, dist_mm: float) -> int:
        """Distance in grid units, rounded down."""
        return int(dist_mm * self.inv_step)

    def to_grid_dist_safe(self, dist_mm: float) -> int:
        """Distance in grid units, rounded UP for safety."""
        return math.ceil(dist_mm * self.inv_step)
```

With the default 0.1mm grid step:
- Grid coordinate (100, 200) = (10.0mm, 20.0mm)
- All routing decisions happen in integer grid space
- Final output is converted back to floating-point mm

## Obstacle Map Architecture

The `GridObstacleMap` (implemented in Rust) tracks blocked cells per layer as
**reference counts**: a cell is blocked while its count is above zero, so one
net's copper can be removed and added back exactly even where it overlaps
another's. Keep-outs that never change during a run (board-edge clearance,
the area outside the outline, cutouts) live in a separate per-layer bitmap
(#422), which costs a bit per cell instead of a hash-map entry.

### Obstacle Types

1. **Blocked Cells** - Cells where tracks cannot be routed on a specific layer (ref-counted)
2. **Static Blocked Cells** - Permanent keep-outs: board-edge clearance, off-board area, cutouts (bitmap)
3. **Blocked Vias** - Cells where vias cannot be placed (any layer), plus one map per alternative via size a run may use (#568, #530)
4. **BGA Zones** - Rectangular areas blocking both tracks and vias
5. **Allowed Cells** - Override for BGA zones at source/target positions
6. **Source/Target Cells** - Override for track clearance at endpoints
7. **Free Via Positions** - Same-net through-hole pads, where a layer change costs nothing
8. **Soft costs** - Stub proximity, per-layer track proximity, plane fragility, history congestion, cross-layer attraction
9. **Keep-out areas** (stamped as blocked cells by Python) - KiCad `(zone ... (keepout ...))` rule areas blocking tracks and/or vias (per the keepout flags) on their listed layers, or all routing layers if none are listed; User-layer keepout polygons (`--keepout`); net-filtered keep-outs from `.kicad_dru` `disallow` rules, stamped only while a net a rule binds is routing; foreign pours this run cannot repair (#1260)

### Building the Obstacle Map

```python
def build_base_obstacle_map(pcb_data, config, nets_to_route, extra_clearance=0.0,
                            net_clearances=None, static_base=False, ...):
    """Base obstacle map: everything that does not move during the run.
    The nets being routed are left out; their copper is added per net."""
    obstacles = GridObstacleMap(num_layers)

    # 1. BGA exclusion zones (block all layers + vias)
    for zone in config.bga_exclusion_zones:
        obstacles.set_bga_zone(min_gx, min_gy, max_gx, max_gy)

    # 2. Existing segments, and a footprint's own copper (an SOT89 tab, an
    #    antenna: #908), each expanded by its clearance
    # 3. Existing vias
    # 4. Pads (pcb_data.pads_by_net) -- each foreign obstacle priced at
    #    max(routing-side clearance, that net's own class clearance)
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id not in nets_to_route:
            ...

    # 5. Board edge clearance, User-layer keepouts, KiCad keep-out rule areas,
    #    foreign-pour barriers, drill hole-to-hole clearance
    add_board_edge_obstacles(obstacles, pcb_data, config, ...)
    add_user_keepout_obstacles(obstacles, pcb_data, config, ...)
    add_rule_area_keepout_obstacles(obstacles, pcb_data, config, ...)
    add_pour_barrier_obstacles(obstacles, pcb_data, config, ...)
    add_drill_hole_obstacles(obstacles, pcb_data, config, nets_to_route, ...)

    return obstacles
```

### Pad Rotation (Python rasterization vs. the orthogonal grid)

The Rust router is purely **cell-based**: `GridObstacleMap` stores only blocked
`(gx, gy, layer)` cells — it has no concept of a pad, its size, shape, or rotation.
All pad geometry is rasterized into blocked cells **in Python**
(`pad_blocked_cells_array` / `iter_pad_blocked_cells` in `routing_utils.py`) and
handed to Rust as a flat array via `add_blocked_cells_batch`.

A pad's `size_x`/`size_y` are board-resolved (axis-aligned for orthogonal pads);
any residual tilt for a non-orthogonally-placed part is carried in
`Pad.rect_rotation` (0 for the common case). The rasterizers honour it, so a tilted
pad blocks the cells its **true rotated rectangle** covers (plus the clearance
margin) — represented on the orthogonal grid as the staircase of cells it overlaps,
exactly as circles/ovals always have been. The grid never stored true shapes; it
quantizes whatever cells Python marks, so no Rust change is needed and a finer
`--grid-step` tightens the approximation. The *exact* geometry still lives in the
continuous Python checks (`check_drc.py`, `check_pads.py`, the fanout/clearance
helpers in `routing_utils.py`).

**Performance.** Every rotation-aware helper short-circuits to the original
axis-aligned path when `rect_rotation == 0`, so orthogonal boards (the vast
majority) pay essentially nothing. Measured per-step when the rotation support
landed (June 2026; redo harness, same machine):

| board | non-orthogonal pads | total routing time Δ |
|-------|--------------------|----------------------|
| lpddr4_testbed | 0 / 677 | **−0.6%** (noise — routing is bit-identical) |
| hackrf_one | 191 / 1623 | **+4.2%** (concentrated in the heavy signal route) |

So there is no measurable slowdown on orthogonal boards; the only cost is a few
percent on boards with many genuinely tilted pads, where the extra trig is doing
necessary work (and part of hackrf's delta is the more-accurate geometry slightly
changing the A* search, not pure overhead).

### Clearance Expansion

Obstacles are expanded by track width and clearance to ensure DRC compliance. Track widths can vary based on:
- **Layer** (impedance-controlled routing): microstrip on outer layers is wider than stripline on inner layers
- **Net** (power net routing and net classes): power/ground nets can use wider tracks than signal nets

and the clearance between two nets is KiCad's: `max(class a, class b)` from the
net classes, replaced on a layer by a `.kicad_dru` layer rule (#498), and
raised by a pad's own clearance override (`GridRouteConfig.obstacle_clearance`,
`pair_clearance`; see [Routing Configuration](api-routing-config.md)).

Clearance calculations use per-net-per-layer widths:

```python
# Per-net-per-layer track clearance expansion
for layer_name in config.layers:
    # get_net_track_width checks power net overrides first, then net-class
    # widths, then layer widths (impedance)
    layer_width = config.get_net_track_width(net_id, layer_name)
    expansion_mm = layer_width / 2 + clearance
    expansion_grid = coord.to_grid_dist(expansion_mm)

    # Via blocking near tracks - uses to_grid_dist_safe() for safety margin
    via_block_mm = via_size / 2 + layer_width / 2 + clearance
    via_block_grid = coord.to_grid_dist_safe(via_block_mm)
```

A segment at position (x, y) blocks all cells within `expansion_grid` radius on its layer, and blocks via placement within `via_block_grid` radius.

**Grid quantization safety**: Via-related clearances use `to_grid_dist_safe()`, which rounds up instead of down. This prevents edge cases where clearances round down and cause minor DRC violations (e.g., 0.01mm overlaps).

## A* Pathfinding Algorithm

The core routing uses A* pathfinding implemented in Rust for performance
(`grid_router.GridRouter`; differential pairs use `grid_router.PoseRouter`,
which also tracks heading).

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
- Orthogonal move: 1000 (1 grid step), diagonal move: 1414 (sqrt(2) grid steps), each times the layer's cost multiplier (`--layer-costs`, 1000 = 1.0x)
- Turn: `--turn-cost` (default 1000) per 90° of direction change, so a 45° kink costs half
- Via (layer change): `via_cost * 1000`, scaled so a via costs the same mm-equivalent detour at any grid step (`--via-cost`, default 75); free at a same-net through-hole pad
- Soft costs: stub and track proximity, plane fragility, history congestion, a layer-direction preference, minus a cross-layer attraction bonus

### Heuristic

The heuristic is the octile distance to the nearest target, scaled by the
cheapest layer's cost so it never overestimates a move, plus one via when
the target is on another layer:

```python
def heuristic(gx, gy, layer, targets):
    min_h = infinity
    for tx, ty, tl in targets:
        dx, dy = abs(gx - tx), abs(gy - ty)
        diag, orth = min(dx, dy), abs(dx - dy)
        h = (diag * 1414 + orth * 1000) * min_layer_cost / 1000
        if layer != tl:
            h += via_cost
        min_h = min(min_h, h)
    return min_h * heuristic_weight
```

With more than 16 targets (a multi-point tap search) it uses the targets'
bounding box instead, which is cheaper and still never overestimates.
`--heuristic-weight` (default 2.3) above 1 trades optimality for speed.

### Multi-Source/Multi-Target Routing

The router supports multiple sources and targets for flexible endpoint matching:

```python
path, iterations, blocked_cells = router.route_with_frontier(
    obstacles,
    sources=[(gx1, gy1, layer1), (gx2, gy2, layer1), ...],
    targets=[(gx3, gy3, layer2), (gx4, gy4, layer2), ...],
    max_iterations=200000
)
```

This allows routing from any source endpoint to any target endpoint. On
failure `blocked_cells` is the search frontier the obstacles stopped, which
the rip-up ladder's blocking analysis reads. (`route_multi` is the same
search returning statistics instead of the frontier.)

### Probe, Then Full Search

Each connection starts with a short probe in both directions
(`--max-probe-iterations`, default 5000):

```python
# Probe forward, then backward, with the probe budget
path, it1, blocked = router.route_with_frontier(obstacles, sources, targets, probe_iters)
if path is None:
    path, it2, blocked = router.route_with_frontier(obstacles, targets, sources, probe_iters)
    # A direction whose probe ran out of cells before its budget is stuck:
    # its endpoint is walled in, so the full search is skipped.
    if path is None and both probes used their whole budget:
        path = full search forward, then backward
```

The full search's budget is dynamic (#529): it starts at `--max-iterations`
(default 200000) and is extended while the search keeps getting closer. A path
whose own vias land closer together than the drill floor is routed once more
with that spacing enforced (#230).

## Incremental Obstacle Updates

For batch routing, obstacles are updated incrementally as each net is routed,
in place on one working map:

```python
base_obstacles = build_base_obstacle_map(pcb_data, config, all_nets)
cache = precompute_all_net_obstacles(pcb_data, all_nets, config)   # each net's own copper
working = build_working_obstacle_map(base_obstacles, cache)         # base + every net

for net_id in nets_to_route:
    # Lift this net's own copper out of the map, clear the last net's
    # per-route costs and overrides, add proximity costs for this net
    prepare_obstacles_inplace(working, pcb_data, config, net_id, ...)

    result = route_net_with_obstacles(pcb_data, net_id, config, working)

    restore_obstacles_inplace(working, net_id, ...)
    if result:
        add_route_to_pcb_data(pcb_data, result)
        # remove the net's old cache entry, recompute it from the board, add it back
        refresh_net_obstacles(working, cache, pcb_data, config, [net_id])
```

A rip-up and a restore go through the same refresh, so the working map always
matches the copper on the board. `KICAD_OBSTACLE_LEDGER=1` audits that the
adds and removes balance.

## Path Conversion

After A* finds a path, `_path_to_segments_vias` in `single_ended_routing.py`
converts it to KiCad geometry:

```python
path = simplify_path(path)   # drop collinear intermediate points
for i in range(len(path) - 1):
    gx1, gy1, layer1 = path[i]
    gx2, gy2, layer2 = path[i + 1]

    x1, y1 = coord.to_float(gx1, gy1)
    x2, y2 = coord.to_float(gx2, gy2)

    if layer1 != layer2:
        # Layer change = via (none where a same-net through-hole pad already is)
        vias.append(Via(x=x1, y=y1, ...))
    else:
        # Same layer = segment
        segments.append(Segment(x1, y1, x2, y2, layer=layer1, ...))
```

Short connectors join the grid path's ends to the original, off-grid pad or
stub positions.

## Multi-Point Routing

Nets with 3+ pads require special handling to work correctly with length matching. The router uses an MST-based 3-phase approach:

### The Problem

When a net has more than 2 pads (e.g., a signal that fans out to multiple destinations), routing all connections before length matching creates issues:
- Tap connections that branch off the main route add non-linear geometry
- The length-matching algorithm can't cleanly add meanders to branching routes
- Tap segments shouldn't be included in length-matched measurements

### MST-Based 3-Phase Solution

The router computes a Minimum Spanning Tree (MST) over the net's terminals using Manhattan distance, then routes edges in length order (longest first). Terminals that existing copper already connects count as one node, so the MST joins connected components and never re-routes copper that is already connected (#317):

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
- Route the **longest** MST edge using standard A* routing — unless a
  selection override picks a different main edge: bus members with a
  planned corridor route the corridor-spanning terminal pair first
  (re-realized from the corridor's endpoints), and
  `KICAD_MULTIPOINT_DENSE_FIRST=1` prefers edges landing on a fanned-out
  dense package (see [Bus Routing](bus-routing.md), "Multi-Point Bus
  Members"). Which edge goes first matters because the deferred edges wait
  for Phase 3, after every other net's main route.
- If that edge cannot route, Phase 1 tries the next one (up to 8 edges,
  #101), so one boxed pad does not fail the net. An edge that routes but
  whose terminal copper the terminal gate refuses — it would overlap a
  foreign track/via, or under `--escalation off` sits inside a clearance
  only a narrower terminal could clear — is deferred the same way: it goes
  to the back of the list and Phase 3 routes it as a tap, under the same
  gate. The net story records `mode: terminal-gate-deferred`.
- Bus members' main edges route with corridor/neighbor attraction, like
  point-to-point members
- Track pending multi-point nets in `state.pending_multipoint_nets`
- Result includes `mst_edges` (routed edge first) for Phase 3

#### Phase 2: Length Matching
- Apply meanders to the clean 2-point main routes
- Works exactly like regular 2-point net length matching
- Meanders are added to the main route segments

#### Phase 3: Tap Connections
- Route remaining MST edges in length order (longest first)
- Each edge must connect an already-routed pad to an unrouted pad
- Find tap point on existing segments near the source pad
- Route from tap point to the target pad
- Uses `route_multipoint_taps()` in `single_ended_routing.py`, driven by `run_phase3_tap_routing()` in `phase3_routing.py`

### Implementation Details

Key functions in `single_ended_routing.py`:
- `route_multipoint_main()` - Phase 1: Computes MST, routes longest edge
- `route_multipoint_taps()` - Phase 3: Routes remaining MST edges by length

Key utilities in `connectivity.py`:
- `compute_component_mst_edges()` - The MST over connected components of terminals (Prim's algorithm, Manhattan distance)
- `compute_mst_edges()` - A plain point MST (Prim's algorithm, Manhattan or Euclidean distance)

State tracking in `routing_state.py`:
- `pending_multipoint_nets` - Dict mapping net_id to main_result for Phase 3

### Limitations

- **Layer swaps: common-layer collapse only** (#265) - When a multi-point net's endpoints span
  several layers, the upfront optimization tries to move ALL movable stubs onto one common layer
  (through-hole/via'd pads count as any-layer, bare SMD pads as fixed, committed pad-to-pad copper
  as immovable), choosing the destination needing the fewest new pad vias. Nets with no feasible
  common layer (e.g. bare SMD pads on both faces) are left as-is with a note.
- Mixed assignments (different layers per endpoint cluster, MST-aware) are not attempted

## Post-Processing

After routing, `cleanup_pipeline.run_post_route_cleanup()` runs one ordered
pipeline of passes (each implemented in `pcb_modification.py`), the same on
both routing fronts (#319). Every pass edits `pcb_data` in place, so the board
in memory is always the board that will be written:

1. **Snap stub gaps** - extend a stub that stopped a fraction short of same-net copper (#84)
2. **Drop phantom copper** - copper that was ripped and never restored never ships (#133)
3. **Prune / nudge grazes** - drop a redundant segment grazing foreign copper, or re-bend / micro-shift / move a via that must stay (#224)
4. **Prune redundant cycles**, then collapse strictly redundant copper and remove orphan islands (#217)
5. **Sweep dead ends** - trim dead-end spurs and unsupported vias
6. **Neck wide segments** that graze a fine-pitch foreign pad
7. **Smooth** staircase micro-jogs into octolinear shortcuts (#536; the route step only)
8. **Close soft joints** - the last pass that adds copper: bridge any same-net joint that only touches
9. **Merge collinear segments** - join same-net, same-layer, same-width pieces into one track (#811)

`route.py` runs the strict collapse once more at the very end, on the board it
ships, after the plane finalize and the reconciliation have laid their copper
(#1063). `KICAD_BOARD_LEDGER=1` checks, per net, that the copper in memory and
the copper the writer produces are identical.

## Visualizing the Search

The interactive PyGame visualizer was removed in #569: the debugging workflow it predated (grade with `check_drc`/`check_connected`, replay a recorded chain from its `redo_commands.sh` manifest, read the per-net JSON summary and blocker report, or step a board in the GUI plugin) covers the same questions without a second rendering path to keep in sync with the engine.

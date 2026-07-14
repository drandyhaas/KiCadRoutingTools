# Routing Configuration API (`routing_config`, `routing_utils`)

- **`routing_config.py`** — `GridRouteConfig` (every routing parameter in
  one dataclass), `GridCoord` (mm ↔ grid conversion), `DiffPairNet`.
- **`routing_utils.py`** — small shared helpers used throughout the
  codebase (`pos_key`, `build_layer_map`, pad-cell iteration).

Most `GridRouteConfig` fields map 1:1 to CLI flags; the flag-oriented view
with tuning advice is in [Configuration](configuration.md). This page is the
programmatic view.

## `GridRouteConfig`

```python
from routing_config import GridRouteConfig

config = GridRouteConfig(
    track_width=0.2, clearance=0.15,
    via_size=0.5, via_drill=0.3,
    layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'],
    grid_step=0.1)
```

All defaults shown below; lengths in mm, angles in degrees. "Cost" values
are calibrated at the reference grid step (0.1 mm) and rescaled
automatically at other grid steps (see [cost scaling](#cost-scaling)).

### Track, via, and grid

| Field | Default | Meaning |
|-------|---------|---------|
| `track_width` | `0.1` | Default trace width |
| `clearance` | `0.1` | Copper-to-copper clearance |
| `via_size` | `0.3` | Via outer diameter |
| `via_drill` | `0.2` | Via hole diameter |
| `grid_step` | `0.1` | Routing grid resolution |
| `layers` | `['F.Cu', 'B.Cu']` | Copper layers available to the router |

### Search algorithm

| Field | Default | Meaning |
|-------|---------|---------|
| `max_iterations` | `200000` | A* iteration cap per attempt |
| `max_probe_iterations` | `5000` | Quick probe per direction to detect blocked routes early |
| `heuristic_weight` | `1.9` | A* greediness (>1 = faster, possibly non-optimal paths) |
| `via_cost` | `50` | Via penalty, in grid steps at the reference grid |
| `turn_cost` | `1000` | Penalty per direction change (straighter routes) |
| `direction_order` | `'forward'` | Try `'forward'` or `'backward'` direction first |

### Proximity penalties (soft costs)

| Field | Default | Meaning |
|-------|---------|---------|
| `stub_proximity_radius` | `2.0` | Radius around unrouted stubs to penalize |
| `stub_proximity_cost` | `0.2` | Cost at the stub center (mm-equivalent detour) |
| `via_proximity_cost` | `10.0` | Via-cost multiplier inside stub/BGA zones (0 = block vias there) |
| `bga_proximity_radius` | `7.0` | Distance from BGA edges to penalize |
| `bga_proximity_cost` | `0.2` | Cost at the BGA edge |
| `track_proximity_distance` | `2.0` | Radius around routed tracks (same layer) |
| `track_proximity_cost` | `0.0` | Cost near routed tracks (0 = off) |
| `ripped_route_avoidance_radius` | `1.0` | Radius around just-ripped routes |
| `ripped_route_avoidance_cost` | `0.1` | Cost near just-ripped routes (helps reroutes diverge) |

### Layer preferences and alignment

| Field | Default | Meaning |
|-------|---------|---------|
| `layer_costs` | `[]` | Per-layer cost multipliers (1.0 = neutral), parallel to `layers` |
| `direction_preference_cost` | `50` | Penalty for off-direction moves; layers alternate H/V starting horizontal on top (0 = off) |
| `vertical_attraction_radius` | `0.2` | Radius for cross-layer alignment bonus (0 = off) |
| `vertical_attraction_cost` | `0.0` | Bonus (negative cost) for vertically aligned positions |

### BGA zones

| Field | Default | Meaning |
|-------|---------|---------|
| `bga_exclusion_zones` | `[]` | `(min_x, min_y, max_x, max_y[, edge_tolerance])` regions blocking vias; from `auto_detect_bga_exclusion_zones` |

### Impedance and per-net widths

| Field | Default | Meaning |
|-------|---------|---------|
| `impedance_target` | `None` | Target Z0 in Ω; when set, per-layer widths come from `layer_widths` |
| `layer_widths` | `{}` | Layer name → width (filled by `impedance.calculate_layer_widths_for_impedance`) |
| `power_net_widths` | `{}` | net_id → width override for power nets (never below `track_width`) |
| `net_clearances` | `{}` | net_id → netclass clearance (mm, #326): each net's own copper is stamped as an obstacle at `max(clearance, its value)` so same-run nets keep the class spacing to it; `get_net_clearance(net_id)` resolves it (never below `clearance`) |

### Differential pairs

| Field | Default | Meaning |
|-------|---------|---------|
| `diff_pair_gap` | `0.101` | P-to-N edge-to-edge gap (center-to-center = `track_width + gap`) |
| `diff_pair_centerline_setback` | `None` | Distance in front of stubs to start the centerline (None = 2 × spacing) |
| `min_turning_radius` | `0.2` | Minimum turning radius for pose-based diff routing |
| `max_setback_angle` | `45.0` | Max angle when searching setback positions |
| `max_turn_angle` | `180.0` | Cumulative turn limit before reset (prevents U-turns) |
| `gnd_via_enabled` | `True` | Place GND return vias next to diff-pair signal vias |
| `diff_pair_intra_match` | `False` | Meander the shorter of P/N to match lengths within the pair |
| `ac_couple_match` | `False` | End-to-end length-match AC-coupled pairs split by series caps: concatenated P vs N path (#196) |
| `diff_chamfer_extra` | `1.5` | Meander chamfer multiplier for pairs (avoids P/N crossings) |
| `diff_pair_hybrid_escape` | `True` | When a coupled pair's terminal connector can't clear foreign copper (#165 graze), keep the coupled middle and defer each terminal leg to a single-ended point-to-point join instead of failing the whole pair |
| `diff_pair_setback_no_ladder` | `False` | When `True`, the setback ladder yields ONLY the configured setback (no 0.75/0.5/floor/1.5/2× expansion) — used by the pinch retry so each attempt routes at the exact setback asked |
| `diff_pair_uncouple_factor` | `6.0` | Multiples of pair spacing (`track_width + diff_pair_gap`); a multi-point terminal whose P/N pads are farther apart than this is treated as uncoupled and routed single-ended (#121) |

### Length / time matching

| Field | Default | Meaning |
|-------|---------|---------|
| `length_match_groups` | `[]` | Groups of net-name patterns to match |
| `length_match_tolerance` | `0.1` | Allowed spread within a group (mm) |
| `meander_amplitude` | `1.0` | Meander height |
| `time_matching` | `False` | Match propagation delay instead of length |
| `time_match_tolerance` | `1.0` | Allowed spread (picoseconds) |

### Power tap neck-down

| Field | Default | Meaning |
|-------|---------|---------|
| `power_tap_neckdown` | `True` | Retry failed wide power taps with a narrow neck from the pad |
| `neckdown_length` | `2.5` | Length of the narrow section |
| `neckdown_taper_length` | `0.5` | Taper from narrow to full width (0 = abrupt) |

### Clearance details and DRC margins

| Field | Default | Meaning |
|-------|---------|---------|
| `routing_clearance_margin` | `1.0` | Multiplier on track-to-via clearance (1.0 = exact DRC minimum) |
| `hole_to_hole_clearance` | `0.25` | Drill-to-drill clearance, edge to edge |
| `board_edge_clearance` | `0.0` | Clearance from board edge (0 = use `clearance`) |

### Strategies and recovery

| Field | Default | Meaning |
|-------|---------|---------|
| `max_rip_up_count` | `3` | Max blocking routes ripped up at once (progressive 1..N) |
| `ripup_abandon_metric` | `'stranded'` | Keep-retry vs abandon rule for multipoint tap rip-ups (see [rip-up-reroute.md](rip-up-reroute.md#abandon-metrics)) |
| `stub_layer_swap` | `True` | Allow moving stubs to other layers to resolve conflicts (never moves an SMD pad's stub off the one layer that pad lives on — that would orphan the pad) |
| `target_swap_crossing_penalty` | `1000.0` | Penalty for crossing assignments during target swap |
| `crossing_layer_check` | `True` | Only count crossings between routes sharing a layer |

### Bus routing

| Field | Default | Meaning |
|-------|---------|---------|
| `bus_enabled` | `False` | Detect and route bus groups together |
| `bus_detection_radius` | `5.0` | Max endpoint spread to form a bus |
| `bus_min_nets` | `2` | Minimum nets per bus |
| `bus_attraction_radius` | `5.0` | Attraction radius from the neighbor's track |
| `bus_attraction_bonus` | `5000` | Cost bonus for hugging the neighbor (cost units) |

### Guide corridor and keepouts

| Field | Default | Meaning |
|-------|---------|---------|
| `guide_corridor_enabled` | `False` | Follow a user-drawn polyline as waypoints |
| `guide_corridor_layer` | `'User.1'` | Layer the guide is drawn on |
| `guide_corridor_spacing` | `0.0` | Waypoint subdivision spacing (0 = endpoints only) |
| `corridor_waypoints` | `[]` | Prebuilt grid `(gx, gy)` waypoints the router threads in order; set programmatically as an alternative to a drawn guide |
| `keepout_enabled` | `False` | Honor a user-drawn keepout polygon |
| `keepout_layer` | `'User.2'` | Layer the keepout is drawn on |

### Debug and output

| Field | Default | Meaning |
|-------|---------|---------|
| `debug_lines` | `False` | Emit debug geometry on User layers |
| `verbose` | `False` | Detailed diagnostics |
| `debug_memory` | `False` | Print memory statistics |
| `collect_stats` | `False` | Collect A* statistics |
| `add_teardrops` | `False` | Add teardrops to all pads in the output |
| `proximity_heuristic_factor` | `0.02` | Tightens the A* heuristic for proximity costs (speed) |

### Methods

```python
config.get_track_width(layer) -> float
```
Layer-aware width: `layer_widths[layer]` if impedance-controlled, else
`track_width`.

```python
config.get_net_track_width(net_id, layer) -> float
```
Net- and layer-aware width. Priority: `power_net_widths[net_id]` →
`layer_widths[layer]` → `track_width`. This is what obstacle expansion uses.

```python
config.get_max_track_width() -> float
```
Maximum width across layers (for worst-case via clearance).

```python
config.get_layer_costs() -> List[int]            # ×1000 for the Rust router
config.get_layer_direction_preferences() -> List[int]  # 0=H, 1=V, 255=none
```

#### Cost scaling

Cost knobs are calibrated at `REFERENCE_GRID_STEP` (0.1 mm) and rescaled so
that the *cost per mm* is grid-independent:

```python
config.cell_cost(cost_mm) -> int        # per-cell costs (proximity penalties)
config.scaled_cell_units(units) -> int  # raw per-cell knobs (bus bonus)
config.via_cost_units() -> int          # per-via penalty in cost units
config.get_proximity_heuristic_cost() -> int
```

You rarely call these yourself, but if you hand costs to the Rust router
directly, use them rather than multiplying by 1000 manually.

### Example

```python
from routing_config import GridRouteConfig

config = GridRouteConfig(track_width=0.15, layers=['F.Cu', 'In1.Cu', 'B.Cu'])
config.power_net_widths = {7: 0.8}                 # net 7 routes at 0.8mm
config.layer_widths = {'F.Cu': 0.18, 'In1.Cu': 0.13, 'B.Cu': 0.18}

print(config.get_net_track_width(7, 'F.Cu'))       # 0.8  (power override wins)
print(config.get_net_track_width(3, 'In1.Cu'))     # 0.13 (impedance width)
print(config.via_cost_units())                     # 50000 at 0.1mm grid
```

## `GridCoord`

Converts between mm and integer grid coordinates. All routing decisions
happen in integer grid space.

```python
from routing_config import GridCoord

coord = GridCoord(grid_step=0.1)
coord.to_grid(12.34, 56.78)      # (123, 568)  — rounds to nearest
coord.to_float(123, 568)         # (12.3, 56.8)
coord.to_grid_dist(0.25)         # 2  — floor: distances round down
coord.to_grid_dist_safe(0.25)    # 3  — adds half a step, rounds up
```

Use `to_grid_dist_safe` for **blocking margins** (clearances): rounding a
clearance down can produce hairline DRC violations after grid quantization.

## `DiffPairNet`

```python
@dataclass
class DiffPairNet:
    base_name: str
    p_net_id: Optional[int] = None
    n_net_id: Optional[int] = None
    p_net_name: Optional[str] = None
    n_net_name: Optional[str] = None
    polarity_swap_allowed: bool = False  # may P/N pad nets be swapped (#279)?

    is_complete  # property: both net IDs present
```

`polarity_swap_allowed` is stamped by `batch_route_diff_pairs` from its
`polarity_swap_nets` glob patterns (CLI `--polarity-swap-nets`); the default
denies swaps for every pair.

Produced by
[`net_queries.find_differential_pairs`](api-net-analysis.md#find_differential_pairs)
and consumed by the diff-pair routing loop.

## `routing_utils.py`

```python
POSITION_DECIMALS   # = 3 (re-exported from kicad_parser)

pos_key(x, y) -> Tuple[float, float]
```
Rounds a coordinate pair to `POSITION_DECIMALS` for use as a dict/set key.
Use it on **both** sides of every position lookup.

```python
build_layer_map(layers) -> Dict[str, int]
# ['F.Cu', 'In1.Cu', 'B.Cu'] -> {'F.Cu': 0, 'In1.Cu': 1, 'B.Cu': 2}
```
The layer-name → index mapping shared by the obstacle map and router.

```python
segment_length(seg: Segment) -> float
```
Euclidean length of one segment.

```python
iter_pad_blocked_cells(pad_gx, pad_gy, half_width, half_height,
                       margin, grid_step,
                       corner_radius=0.0, corner_buffer=None)
    # yields (gx, gy) grid cells
pad_blocked_cells_array(...) -> np.ndarray   # vectorized twin, (N, 2) int32
```
Enumerate the grid cells a pad blocks (rounded-rect aware), used by obstacle
building and the DRC checker. The two variants produce bit-identical cell
sets.

```python
dist_sq_to_rounded_rect(px, py, half_width, half_height,
                        corner_radius=0.0) -> float
```
Squared distance from a point to a rounded rectangle centered at the origin
(0 inside).

```python
circle_offsets(block_range, effective_sq) -> np.ndarray
```
Cached offset grids for expanding obstacles (circular footprint).

### Example

```python
from routing_utils import pos_key, build_layer_map, segment_length
from kicad_parser import parse_kicad_pcb

pcb = parse_kicad_pcb('kicad_files/routed_output.kicad_pcb')
layer_map = build_layer_map(pcb.board_info.copper_layers)
print(layer_map)

# Total routed copper length per layer
totals = {}
for seg in pcb.segments:
    totals[seg.layer] = totals.get(seg.layer, 0.0) + segment_length(seg)
for layer, mm in sorted(totals.items()):
    print(f"{layer:8s} {mm:8.1f} mm")

# Position keys survive float noise
assert pos_key(1.0000004, 2.0) == pos_key(1.0, 2.0)
```
